

/**
 * Driver for VectorNav 300 INS
 */


#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
//#include <sys/ioctl.h>
#include <poll.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>


#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/vehicle_gps_position.h>

#include <board_config.h>

#include "vectornav300.hpp"
#include "codec/vn300_encoder.h"



#define INS_BASE_DEVICE_PATH	"/dev/ins"
#define INS0_DEVICE_PATH	    "/dev/ins0"

#define INS_WAIT_BEFORE_READ   20 //ms to wait before reading to avoid many read calls

#define MIN(X,Y)	((X) < (Y) ? (X) : (Y))


/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int vn300_main(int argc, char *argv[]);

VectorNav300::VectorNav300(const char *port) :
    _call{},
    _call_interval(0),
	_sensor_ok(false),
	_measure_ticks(0),
	_last_read(0),
    _echo_send_msg({0}),
    _echo_send_wrap(nullptr),
    _recv_msg({0}),
	_recv_wrap(nullptr),

    _gps_pos_reports(nullptr),

	_consecutive_fail_count(0),
	_sample_perf(perf_alloc(PC_ELAPSED, "vn300_read")),
	_buffer_overflows(perf_alloc(PC_COUNT, "vn300_buf_of")),

    _write_errors(perf_alloc(PC_COUNT, "vn300_TX_err")),
    _read_errors(perf_alloc(PC_COUNT, "vn300_RX_err"))
{
	// store port name
	strncpy(_port, port, sizeof(_port));
	// enforce null termination
	_port[sizeof(_port) - 1] = '\0';

}

VectorNav300::~VectorNav300()
{
	// make sure we are truly inactive
	stop();

	// free any existing reports
	if (_gps_pos_reports != nullptr) {
		delete _gps_pos_reports;
        _gps_pos_reports = nullptr;
	}

	perf_free(_sample_perf);
	perf_free(_buffer_overflows);
    perf_free(_write_errors);
    perf_free(_read_errors);
}


int
VectorNav300::initSerialPort()
{
    // open the serial port
    _serial_fd = ::open(_port, O_RDWR | O_NOCTTY);//TODO add O_NONBLOCK ?

    if (_serial_fd < 0) {
        PX4_ERR("VN300: serial port open failed: %s err: %d", _port, errno);
        return -1;
    }

    struct termios uart_config = {0};

    int termios_state;

    // fill the struct for the new configuration
    tcgetattr(_serial_fd, &uart_config);

    // clear ONLCR flag (which appends a CR for every LF)
    uart_config.c_oflag &= ~ONLCR;

    // no parity, one stop bit
    uart_config.c_cflag &= ~(CSTOPB | PARENB);

    // default VN300 serial baud rate
    const unsigned kSerialBaudRate = B115200;

    /* set baud rate */
    if ((termios_state = cfsetispeed(&uart_config, kSerialBaudRate)) < 0) {
        warnx("ERR CFG: %d ISPD", termios_state);
    }

    if ((termios_state = cfsetospeed(&uart_config, kSerialBaudRate)) < 0) {
        warnx("ERR CFG: %d OSPD\n", termios_state);
    }

    if ((termios_state = tcsetattr(_serial_fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR baud %d ATTR", termios_state);
    }

    return OK;
}


int
VectorNav300::init()
{
	//TODO load any configuration params here
	//	param_get(param_find("SENS_EN_VN300"), &hw_model);

	int ret = initSerialPort();
	if (OK != ret) {
			return -1;
	}

	_echo_send_wrap = vn300_alloc_msg_wrap();
	if (nullptr == _echo_send_wrap) {
		warnx("_echo_send_wrap mem err");
		return -1;
	}

	_recv_wrap = vn300_alloc_msg_wrap();
	if (nullptr == _recv_wrap) {
			warnx("_recv_wrap mem err");
			return -1;
	}

	_std_msg_len = _echo_send_wrap->len;

	_echo_send_msg.vel_uncertainty = 0.25f;
	_echo_send_msg.pos_uncertainty = 0.88f;

	vn300_encode_res encode_res =  vn300_encode_standard_msg(&_echo_send_msg , _echo_send_wrap);
	if (VN300_ENCODE_OK != encode_res) {
			warnx("encode_standard_msg fail");
			return -1;
	}

	_gps_pos_reports = new ringbuffer::RingBuffer(2, sizeof(vehicle_gps_position_s));
	if (nullptr == _gps_pos_reports) {
		warnx("mem err");
		return -1;
	}

	return OK;
}


ssize_t
VectorNav300::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct vehicle_gps_position_s);
	struct vehicle_gps_position_s *rbuf = reinterpret_cast<struct vehicle_gps_position_s *>(buffer);
	int ret = 0;

	// buffer must be large enough
	if (count < 1) {
		return -ENOSPC;
	}

    // If no data is available, try again later
    if (_gps_pos_reports->empty()) {
        return -EAGAIN;
    }

	// if automatic measurement is enabled
	if (_measure_ticks > 0) {
		// While there is space in the caller's buffer, and reports, copy them.
		// Note that we may be preempted by the workq thread while we are doing this;
		// we are careful to avoid racing with them.
		while (count--) {
			if (_gps_pos_reports->get(rbuf)) {
				ret += sizeof(*rbuf);
				rbuf++;
			}
		}

		// if there was no data, warn the caller
		return ret ? ret : -EAGAIN;
	}

    _gps_pos_reports->flush();

    // trigger a measurement
    if (OK != measure()) {
        return -EIO;
    }

	return ret;
}

int
VectorNav300::measure()
{
	//With VectorNav VN300 we don't need to do anything to begin a "measurement":
	//we just need to wait until the receive buffer fills up

	sendEchoMsg();

	pollOrRead(_rawReadBuf, sizeof(_rawReadBuf), 1000);
	publish();

	return OK;
}



void
VectorNav300::start()
{
	// reset the report ring and state machine
	_gps_pos_reports->flush();

	// schedule a cycle to start things
    hrt_call_every(&_call,
                   1000,
                   _call_interval ,
                   (hrt_callout)&VectorNav300::measure_trampoline, this);

}

void
VectorNav300::stop()
{
    hrt_cancel(&_call);
    _gps_pos_reports->flush();
}

void
VectorNav300::measure_trampoline(void *arg)
{
	VectorNav300 *dev = static_cast<VectorNav300 *>(arg);
	dev->measure();
}



void
VectorNav300::publish()
{
    int gps_multi;
    orb_publish_auto(ORB_ID(vehicle_gps_position), &_report_gps_pos_topic, &_report_gps_pos, &gps_multi, ORB_PRIO_HIGH);

    //TODO publish other messages

}
void
VectorNav300::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_buffer_overflows);
    perf_print_counter(_write_errors);
    perf_print_counter(_read_errors);
	printf("poll interval:  %d ticks\n", _measure_ticks);
    _gps_pos_reports->print_info("report queue");
}

void VectorNav300:: sendEchoMsg()
{
    int written = ::write(_serial_fd, _echo_send_wrap->buf, _echo_send_wrap->len);
    if (written != _echo_send_wrap->len) {
        perf_count(_write_errors);
    }
}


int VectorNav300::pollOrRead(uint8_t *buf, size_t buf_length, int timeout)
{
	//Poll only for the serial data. In the same thread we also need to handle orb messages,
	//so ideally we would poll on both, the serial fd and orb subscription. Unfortunately the
	//two pollings use different underlying mechanisms (at least under posix), which makes this
	//impossible. Instead we limit the maximum polling interval and regularly check for new orb
	//messages.
	const int kMaxTimeout = 50;

	pollfd fds[1];
	fds[0].fd = _serial_fd;
	fds[0].events = POLLIN;

	int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), MIN(kMaxTimeout, timeout));

	if (ret > 0) {
		// if we have new data from INS, go handle it
		if (fds[0].revents & POLLIN) {
			/*
			 * We are here because poll says there is some data, so this
			 * won't block even on a blocking device. But don't read immediately
			 * by 1-2 bytes, wait for some more data to save expensive read() calls.
			 * If we have all requested data available, read it without waiting.
			 * If more bytes are available, we'll go back to poll() again.
			 */
			int err = 0, bytesAvailable = 0;
			err = ioctl(_serial_fd, FIONREAD, (unsigned long)&bytesAvailable);

			if ((err != 0) || (bytesAvailable < buf_length)) {
				usleep(INS_WAIT_BEFORE_READ * 1000);
			}

			ret = ::read(_serial_fd, buf, buf_length);
            if (ret <  0) {
                perf_count(_read_errors);
            }

		} else {
			ret = -1;
		}
	}

	return ret;


}


/**
 * Local functions in support of the shell command.
 */
namespace vectornav300
{

VectorNav300	*g_dev;

void	start(const char *port);
void	stop();
void	info();

/**
 * Start the driver.
 */
void
start(const char *port)
{
	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	// create the driver
	g_dev = new VectorNav300(port);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver
 */
void stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		errx(1, "driver not running");
	}

	exit(0);
}


/**
 * Print a little info about the driver.
 */
void
info()
{
	if (nullptr == g_dev) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} // namespace

int
vn300_main(int argc, char *argv[])
{
	if (!strcmp(argv[1], "start")) {
        // Start/load the driver.
        if (argc > 2) {
            //alternative port
			vectornav300::start(argv[2]);
		}
        else {
			vectornav300::start(VN300_DEFAULT_PORT);
		}
	}
	else if (!strcmp(argv[1], "stop")) {
        vectornav300::stop();
	}
    else if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		vectornav300::info();
	}

	errx(1, "unrecognized command, try 'start', 'stop', 'test', 'reset' or 'info'");
}
