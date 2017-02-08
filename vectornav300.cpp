

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
#include <uORB/topics/ins_common.h>

#include <systemlib/scheduling_priorities.h>

#include <board_config.h>

#include "vectornav300.hpp"
#include "vn300_encoder.h"
#include "vn300_decoder.h"

#define INS_WAIT_BEFORE_READ   20 //ms to wait before reading to avoid many read calls

#define MIN(X,Y)	((X) < (Y) ? (X) : (Y))

//used for manipulating FrySky_INV signal
#define GPIO_FRSKY_INV		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN10)
#define INVERT_FRSKY(_s)	px4_arch_gpiowrite(GPIO_FRSKY_INV, _s);


/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int vn300_main(int argc, char *argv[]);

namespace
{
    //global reference
    VectorNav300	*g_dev = nullptr;
}

VectorNav300::VectorNav300(const char *port) :
		_task_should_exit(false),
		_echo_test(true), //TODO read from params instead
		_serial_fd(-1),
		_stream_synced(false),
		_echo_send_msg({0}),
		_echo_send_wrap(nullptr),
		_recv_msg({0}),
		_recv_wrap(nullptr),
    _orb_pub_instance(-1),

    _report_gps_pos({0}),
		_report_gps_pos_topic(nullptr),

		_report_ins({0}),
		_report_ins_topic(nullptr),


		_rawReadAvailable(0),
		_read_perf(perf_alloc(PC_ELAPSED, "vn300_read")),
		_resync_perf(perf_alloc(PC_COUNT, "vn300_resync")),
		_decode_perf(perf_alloc(PC_ELAPSED, "vn300_decode_perf")),

		_decode_errors(perf_alloc(PC_COUNT, "vn300_decode_err")),

		_write_errors(perf_alloc(PC_COUNT, "vn300_TX_err")),
		_poll_errors(perf_alloc(PC_COUNT, "vn300_poll_err")),

		_read_errors(perf_alloc(PC_COUNT, "vn300_RX_err"))
{
	// store port name
	strncpy(_port, port, sizeof(_port));
	// enforce null termination
	_port[sizeof(_port) - 1] = '\0';

//	g_dev = this;

}

VectorNav300::~VectorNav300()
{
	// make sure we are truly inactive
	stop();

	perf_free(_read_perf);
	perf_free(_resync_perf);
	perf_free(_decode_perf);

	perf_free(_decode_errors);

	perf_free(_write_errors);
	perf_free(_poll_errors);
	perf_free(_read_errors);

}


int
VectorNav300::openUART()
{
    // turn off FrSky_INV  (disables the inverter covering the serial port we use)
    INVERT_FRSKY(0);

    // open fd
    _serial_fd = ::open(_port,  O_RDWR | O_NOCTTY);

    if (_serial_fd <= 0) {
        warnx("VN300: serial port open failed: %s err: %d", _port, errno);
        return -1;
    }

    struct termios uart_config;
    int termios_state;

    // fill the struct for the new configuration
    tcgetattr(_serial_fd, &uart_config);

    // clear ONLCR flag (which appends a CR for every LF)
    uart_config.c_oflag &= ~ONLCR;

    // no parity, one stop bit
    uart_config.c_cflag &= ~(CSTOPB | PARENB);

    unsigned speed = B115200; //B9600; //B115200;

    // set baud
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        warnx("ERR CFG: %d ISPD", termios_state);
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        warnx("ERR CFG: %d OSPD\n", termios_state);
    }

    if ((termios_state = tcsetattr(_serial_fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR baud %d ATTR", termios_state);
    }

    return _serial_fd;
}


void
VectorNav300::task_main_trampoline(void *arg)
{
	g_dev->taskMain();
}

void
VectorNav300::setupThings()
{
    openUART();
    if (_serial_fd <= 0) {
        warnx("failed to open serial port: %s err: %d", _port, errno);
        // tell the dtor that we are exiting, set error code
        _task = -1;
        _exit(1);
    }
}

void
VectorNav300::teardownThings()
{
    if (_serial_fd > 0) {
        ::close(_serial_fd);
        _serial_fd = -1;
    }
}

void
VectorNav300::taskMain()
{
    setupThings();

    while (!_task_should_exit) {
        // poll descriptor
        struct pollfd fds[1];
        fds[0].fd = _serial_fd;
        fds[0].events = POLLIN;

				if (_echo_test) {
					sendEchoMsg();
				}

			int status = ::poll(fds, sizeof(fds) / sizeof(fds[0]), 250);
        if (status > 0) {
            if (fds[0].revents & POLLIN) {
							//Some read data is available: read and process it
							perf_begin(_read_perf);
							handleSerialData();
							perf_end(_read_perf);
            }
        }
        else if (status < 0) {
            warnx("poll status: %d errno: %d",status, errno);
        }
    }

    warnx("exiting");
    teardownThings();

}

int
VectorNav300::init()
{
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

	//setup some crappy test data
	_echo_send_msg.gps_nanoseconds = 1234567890;

	_echo_send_msg.angular_rate.c[0] = 1.0f;
	_echo_send_msg.angular_rate.c[1] = 2.0f;
	_echo_send_msg.angular_rate.c[2] = 3.0f;

	_echo_send_msg.euler_yaw_pitch_roll.c[0] = 2.0f;
	_echo_send_msg.euler_yaw_pitch_roll.c[1] = 4.0f;
	_echo_send_msg.euler_yaw_pitch_roll.c[2] = 8.0f;

	_echo_send_msg.pos_lla.c[0] = 37.827514;
	_echo_send_msg.pos_lla.c[1] = -122.372918;
	_echo_send_msg.pos_lla.c[2] = 314.59;

	_echo_send_msg.pos_ecef.c[0] = 100.0;
	_echo_send_msg.pos_ecef.c[1] = 200.0;
	_echo_send_msg.pos_ecef.c[2] = 300.0;

	_echo_send_msg.vel_ned.c[0] = 111.0;
	_echo_send_msg.vel_ned.c[1] = 222.0;
	_echo_send_msg.vel_ned.c[2] = 333.0;

	_echo_send_msg.vel_body.c[0] = 222.0;
	_echo_send_msg.vel_body.c[1] = 444.0;
	_echo_send_msg.vel_body.c[2] = 888.0;


	_echo_send_msg.vel_uncertainty = 0.25f;
	_echo_send_msg.pos_uncertainty = 0.88f;

	vn300_encode_res encode_res =  vn300_encode_standard_msg(&_echo_send_msg , _echo_send_wrap);
	if (VN300_ENCODE_OK != encode_res) {
			warnx("encode_standard_msg fail");
			return -1;
	}

	_task = px4_task_spawn_cmd("vn300t", SCHED_DEFAULT,
														 SCHED_PRIORITY_SLOW_DRIVER,
														 1100,
														 (main_t)&VectorNav300::task_main_trampoline,
														 nullptr);

	if (_task < 0) {
			warnx("task start failed: %d", errno);
			return -errno;
	}

warnx("init OK");
return OK;
}



void
VectorNav300::stop()
{
    _task_should_exit = true;
}



void
VectorNav300::publish()
{

	//time_nsec
  _report_ins.time_nsec = _recv_msg.gps_nanoseconds;


  //angular_rate
	memcpy((void*)&_report_ins.angular_rate, (void*)&_recv_msg.angular_rate, sizeof(vn_vec3f));

  //euler_yaw_pitch_roll
	memcpy((void*)&_report_ins.euler_yaw_pitch_roll, (void*)&_recv_msg.euler_yaw_pitch_roll, sizeof(vn_vec3f));

  //att_q
	memcpy((void*)&_report_ins.att_q, (void*)&_recv_msg.att_quaternion, sizeof(vn_vec4f));

  // LLA position
  _report_ins.lat = (int32_t)( _recv_msg.pos_lla.c[0] * 1E7); //Latitude in 1E-7 degrees
  _report_ins.lon = (int32_t)( _recv_msg.pos_lla.c[1] * 1E7); //Longitude in 1E-7 degrees
  _report_ins.alt_ellipsoid = (_recv_msg.pos_lla.c[2] * 1E3); //altitude in meters above WGS84 ellipsoid

  //pos_ecef
	memcpy((void*)&_report_ins.pos_ecef, (void*)&_recv_msg.pos_ecef, sizeof(vn_pos3_t));

  //vel_body
	memcpy((void*)&_report_ins.vel_body, (void*)&_recv_msg.vel_body, sizeof(vn_vel3_t));

  //vel_ned
	memcpy((void*)&_report_ins.vel_ned, (void*)&_recv_msg.vel_ned, sizeof(vn_vel3_t));

  _report_ins.pos_uncertainty = _recv_msg.pos_uncertainty;
  _report_ins.vel_uncertainty = _recv_msg.vel_uncertainty;

	_report_ins.timestamp = hrt_absolute_time();
  orb_publish_auto(ORB_ID(ins_common), &_report_ins_topic, &_report_ins, &_orb_pub_instance, ORB_PRIO_HIGH);


  /*
	_report_gps_pos.time_utc_usec = (uint64_t )_recv_msg.gps_nanoseconds + (dumb_counter++);

	// LLA position
	_report_gps_pos.lat = (int32_t)( _recv_msg.pos_lla.c[0] * 1E7); //Latitude in 1E-7 degrees
	_report_gps_pos.lon = (int32_t)( _recv_msg.pos_lla.c[1] * 1E7); //Longitude in 1E-7 degrees
	_report_gps_pos.alt = (int32_t)(_recv_msg.pos_lla.c[2] * 1E3); //Altitude in 1E-3 meters above MSL, (millimetres)
	_report_gps_pos.alt_ellipsoid = _report_gps_pos.alt;

	// NED velocity
	_report_gps_pos.vel_n_m_s = _recv_msg.vel_ned.c[0];
	_report_gps_pos.vel_e_m_s = _recv_msg.vel_ned.c[1];
	_report_gps_pos.vel_d_m_s = _recv_msg.vel_ned.c[2];
	_report_gps_pos.vel_ned_valid = true;

	_report_gps_pos.fix_type = 3; //3D Fix

	_report_gps_pos.eph = _recv_msg.pos_uncertainty;
	_report_gps_pos.epv = _recv_msg.pos_uncertainty;

	_report_gps_pos.s_variance_m_s = _recv_msg.vel_uncertainty; //GPS speed accuracy estimate, (metres/sec)

	float vx,vy,vz,vtotal;
	vx = _recv_msg.vel_body.c[0];
	vy = _recv_msg.vel_body.c[1];
	vz = _recv_msg.vel_body.c[2];
	vtotal = sqrt(vx*vx + vy*vy + vz*vz);
	_report_gps_pos.vel_m_s = vtotal;

	orb_publish_auto(ORB_ID(vehicle_gps_position), &_report_gps_pos_topic, &_report_gps_pos, &_orb_pub_instance, ORB_PRIO_HIGH);
  */


}


void
VectorNav300::print_info()
{
	perf_print_counter(_read_perf);
	perf_print_counter(_resync_perf);
	perf_print_counter(_decode_perf);
	perf_print_counter(_decode_errors);

	perf_print_counter(_write_errors);
	perf_print_counter(_poll_errors);
	perf_print_counter(_read_errors);

}

void VectorNav300:: sendEchoMsg()
{
    int written = ::write(_serial_fd, _echo_send_wrap->buf, _echo_send_wrap->len);
    if (written != _echo_send_wrap->len) {
			perf_count(_write_errors);
    }
}


int VectorNav300::doRawRead(void)
{
	int writeOffset = _rawReadAvailable;
	int maxRead = sizeof(_rawReadBuf) - _rawReadAvailable;
	if (maxRead > 0) {
		int nRead = ::read(_serial_fd, _rawReadBuf + writeOffset, maxRead);
		if (nRead <= 0) {
			return -1;
		}
		_rawReadAvailable += nRead;
	}

	return _rawReadAvailable;
}

int VectorNav300::resync(void)
{
	perf_count(_resync_perf);
	//search for the sync byte in input stream.
	//we assume that there are at least _std_msg_len bytes available
	//in our internal buffer

	for (uint32_t i = 0; i < _rawReadAvailable; i++) {
		uint8_t b = _rawReadBuf[i];
		if (VECTORNAV_HEADER_SYNC_BYTE == b) {
			//we found the start of a VN message
			//copy these bytes to the front of the buffer
			memcpy(_rawReadBuf, _rawReadBuf, (_rawReadAvailable - i));
			_rawReadAvailable -= i;
			_stream_synced = true;
			break;
		}
	}
	if (!_stream_synced) {
		_rawReadAvailable = 0;
		memset(_rawReadBuf,0, sizeof(_rawReadBuf));
	}

	return 0;
}

void VectorNav300::handleSerialData(void)
{

	if (doRawRead() >= _std_msg_len) {
		if (_rawReadAvailable >= _std_msg_len) {
			if (VECTORNAV_HEADER_SYNC_BYTE != _rawReadBuf[0]) {
				_stream_synced = false;
			}
		}

		if (!_stream_synced) {
			resync();
		}

		if (_stream_synced && (_rawReadAvailable >= _std_msg_len)) {
			//put encoded data in wrapper
			memcpy(_recv_wrap->buf, _rawReadBuf, _std_msg_len);
			_rawReadAvailable -= _std_msg_len;

			perf_begin(_decode_perf);
			vn300_decode_res res = vn300_decode_standard_msg(_recv_wrap, &_recv_msg);
			perf_end(_decode_perf);

			if (VN300_DECODE_OK == res) {
				publish(); //TODO
			}
			else {
				perf_count(_decode_errors);
			}
		}
	}

}

/*
int VectorNav300::pollOrRead(uint8_t *buf, size_t buf_length, int timeout)
{
	//Poll only for the serial data. In the same thread we also need to handle orb messages,
	//so ideally we would poll on both, the serial fd and orb subscription. Unfortunately the
	//two pollings use different underlying mechanisms (at least under posix), which makes this
	//impossible. Instead we limit the maximum polling interval and regularly check for new orb
	//messages.
	const int kMaxTimeout = 10;

	pollfd fds[1];
	fds[0].fd = _serial_fd;
	fds[0].events = POLLIN;

	int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), MIN(kMaxTimeout, timeout));
	if (ret > 0) {
		// if we have new data from INS, go handle it
		if (fds[0].revents & POLLIN) {

//			 * We are here because poll says there is some data, so this
//			 * won't block even on a blocking device. But don't read immediately
//			 * by 1-2 bytes, wait for some more data to save expensive read() calls.
//			 * If we have all requested data available, read it without waiting.
//			 * If more bytes are available, we'll go back to poll() again.
//			 *
			int err = 0, bytesAvailable = 0;
			err = ioctl(_serial_fd, FIONREAD, (unsigned long)&bytesAvailable);

			if ((err != 0) || (bytesAvailable < buf_length)) {
				usleep(INS_WAIT_BEFORE_READ * 1000);
			}

			ret = ::read(_serial_fd, buf, buf_length);
			if (ret <  0) {
					perf_count(_read_errors);
			}

		}
		else {
			warnx("poll failed: %d",ret);
			perf_count(_poll_errors);
			ret = -1;
		}
	}

	return ret;


}
*/


/**
 * Local functions in support of the shell command.
 */
namespace vectornav300
{

//VectorNav300	*g_dev;

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
