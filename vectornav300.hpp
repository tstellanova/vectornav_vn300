//
// Created by Todd Stellanova on 2/6/17.
//

#ifndef PX4_VECTORNAV300_H
#define PX4_VECTORNAV300_H


#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>

#include "codec/vn300_msg_types.h"



//  UART mapping on FMUv1/2/3/4:
//#
//# UART1		/dev/ttyS0		IO debug
//# USART2		/dev/ttyS1		TELEM1 (flow control)
//# USART3		/dev/ttyS2		TELEM2 (flow control)
//# UART4                       GPS ? USART4
//# UART7							CONSOLE ("Serial Debug" / dronecode port DCP)
//# USART8							SERIAL4

// designated SERIAL4/5 on Pixhawk
#define VN300_DEFAULT_PORT		"/dev/ttyS6"

//normally on v4 FrSky telemetry is started:
// frsky_telemetry start -d /dev/ttyS6
// need to ensure that frsky_telemetry is not started
// also need to ensure that FrSky_INV is turned off ?



class VectorNav300
{
public:
    VectorNav300(const char *port = VN300_DEFAULT_PORT);
    virtual ~VectorNav300();

    virtual int 			init();

    virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);

    //print some basic information about the driver.
    void				print_info();

protected:

    /**
     * This is an abstraction for the poll on serial used.
     *
     * @param buf: pointer to read buffer
     * @param buf_length: size of read buffer
     * @param timeout: timeout in ms
     * @return: 0 for nothing read, or poll timed out
     *	    < 0 for error
     *	    > 0 number of bytes read
     */
    virtual int pollOrRead(uint8_t *buf, size_t buf_length, int timeout);
    void            sendEchoMsg();

    int         initSerialPort();


private:
    char 				_port[20];

    struct hrt_call		_call;
    unsigned		    _call_interval;

    bool				_sensor_ok;
    int				_measure_ticks;

    int				_serial_fd;	 ///< serial interface to INS

    hrt_abstime			_last_read;

    uint32_t                _std_msg_len;

    vn300_standard_msg_t    _echo_send_msg;
    vn300_msg_buf_wrap_t*    _echo_send_wrap;

    vn300_standard_msg_t    _recv_msg;
    vn300_msg_buf_wrap_t*    _recv_wrap;


    ringbuffer::RingBuffer *_gps_pos_reports;
    struct vehicle_gps_position_s	_report_gps_pos;///< uORB topic for gps position
    orb_advert_t			_report_gps_pos_topic;	///< uORB pub for gps position

    uint8_t     _rawReadBuf[256];
    unsigned			_consecutive_fail_count;

    perf_counter_t			_sample_perf;
    perf_counter_t			_buffer_overflows;
    perf_counter_t			_write_errors;
    perf_counter_t			_read_errors;


    /**
    * Initialise the automatic measurement state machine and start it.
    */
    void			start();

    /**
    * Stop the automatic measurement state machine.
    */
    void			stop();

    /**
     * Fetch messages from the INS and update the report buffers.
     */
    int				measure();

    /**
    * Publish to uORB
    */
    void 			publish();


    /**
    * @param arg		Instance pointer for the driver that is polling.
    */
    static void			measure_trampoline(void *arg);


};

#endif //PX4_VECTORNAV300_H
