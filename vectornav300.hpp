//
// Created by Todd Stellanova on 2/6/17.
//

#ifndef PX4_VECTORNAV300_H
#define PX4_VECTORNAV300_H


#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>

#include "vn300_msg_types.h"



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



/**
 * Collects INS data from the VectorNav VN300
 * and publishes it to uORB.
 */
class VectorNav300
{
public:
    VectorNav300(const char *port = VN300_DEFAULT_PORT);
    virtual ~VectorNav300();

    virtual int 			init();

    virtual void	print_info();  ///< print status info
    virtual void  start_test(); ///< Start loopback test

protected:

    /**
    * Trampoline to the worker task
    */
    static void	task_main_trampoline(void *arg);

    /**
     * Worker task: main thread that parses incoming data, always running
     */
    void taskMain(void);


    void setup(void);///< Prepare to start receiving INS data from serial port
    void teardown(void);///< Release system resources

    int openUART(void);
    void handleSerialData(void);

    /**
     * Read available data into our internal buffer
     * @return < 0 if any error
     */
    int doRawRead(void);

  /**
   * Clear out our raw receive buffer
   */
    void clearReceiveBuffer(void);

    /**
     * Find the sync byte in incoming stream
     */
    void resync(void);

    /**
    * Used for loopback testing
    */
    void sendEchoMsg(void);

    /**
    * Publish our topics to uORB
    */
    void 			publish();


private:
    char 				    _port[20];///< The serial port we need to open
    volatile int		_task;  ///< worker task
    bool				   _task_should_exit;  ///< flag to make the main worker task exit

    bool      _echo_test; ///< Should we send a message to the receiver for loopback?

    int				_serial_fd;	 ///< serial interface to INS
    bool      _stream_synced; ///< Has the input stream been synchronized so that buf[0] is SYNC?

    uint32_t                _std_msg_len;

    vn300_standard_msg_t    _echo_send_msg;
    vn300_msg_buf_wrap_t*   _echo_send_wrap;

    vn300_standard_msg_t    _recv_msg;
    vn300_msg_buf_wrap_t*   _recv_wrap;

    int _orb_pub_instance;///< uORB publisher for all topics we publish

    struct vehicle_gps_position_s	_report_gps_pos;///< uORB message for vehicle_gps_position
    orb_advert_t			_report_gps_pos_topic;	///< uORB advert for vehicle_gps_position

    struct ins_common_s  _report_ins; ///< uORB message for ins_common
    orb_advert_t			_report_ins_topic;	///< uORB advert for ins_common


    uint8_t     _rawReadBuf[256];
    uint32_t    _rawReadAvailable;///< How much of _rawReadBuf contains fresh valid data

    perf_counter_t			_read_perf;
    perf_counter_t      _resync_perf;
    perf_counter_t      _decode_perf;
    perf_counter_t      _decode_errors;


  perf_counter_t			_write_errors;
    perf_counter_t			_poll_errors;
    perf_counter_t			_read_errors;


    /**
    * Stop reading and publishing INS data
    */
    void			stop();


};

#endif //PX4_VECTORNAV300_H
