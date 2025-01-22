// @file function_factory.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-15)
#ifndef HJ_DEVICE_H
#define HJ_DEVICE_H
#include <string>
#include <sys/epoll.h>

#define RET_ERR     -1
#define RET_OK      0
#define FALSE       0
#define TRUE        1

#ifdef TIOCGRS485
#undef TIOCGRS485
#endif

#ifdef TIOCSRS485
#undef TIOCSRS485
#endif


#ifndef TIOCGRS485
#define TIOCGRS485 _IOR('T', 0x2E, struct serial_rs485)
#endif
#ifndef TIOCSRS485
#define TIOCSRS485 _IOWR('T', 0x2F, struct serial_rs485)
#endif

#include <map>
namespace hj_bf {
class Uart {
  public:
    Uart(std::string dev);
    ~Uart();

  /**
   * \brief open and run uart
   * \return whether openning the uart was successful
   */
    bool run();
  
  /**
   * \brief set uart parameter
   * \param baudrate baud rate
   * \param flow_ctrl  data flow control
   * \param databits  data bit
   * \param stopbits  data stop bit
   * \param parity  parity bit
   * \return whether openning and setting the uart was successful
   */
    void initialize(int baudrate, int flow_ctrl, int databits, int stopbits, int parity);

  /**
   * \brief send data to uart
   * \param data buffer to be sent 
   * \param len data length to be sent
   * \return whether uart data sending was successful
   */
    bool send(uint8_t* data, int len);

  /**
   * \brief receive data from uart
   * \param data buffer where received data was stored 
   * \return >0 : recieved data length
   *         =0 : no data received
   *         <0 : error occur
   */
    int recv(uint8_t* data);

  /**
   * \brief flush uart output buffer
   */
    void flushout();

  /**
   * \brief flush uart input buffer
   */
    void flushin();
  
  /**
   * \brief flush uart input and output buffer
   */
    void flushinout();
  
  /**
   * \brief close uart and then open with parameter passed by 'run' method
   * \return whether reset was successful
   */
    bool reset();

  private:
    int fd_;
    std::string dev_;
    int baud_rate_;
    int flow_ctrl_;
    int databits_;
    int stopbits_;
    int parity_;

    bool initFlag_;
    bool runFlag_;
    
    static const int MAX_PACKAGE_SIZE = 64;
    static const int MAX_EVENTS_NUM = 2;
    struct epoll_event events_[MAX_EVENTS_NUM];
    int epoll_fd_;
    uint8_t timeout_sec_;
    std::map<int, int> baudMap_;

  private:
    bool open();
    void close();
    bool setuart();
};

/*spi转接串口uart，负责对串口的读写和打开关闭，以及属性设置*/
class SpiUart {
 public:
    SpiUart() = default;
    explicit SpiUart(std::string dev_path);
    ~SpiUart();
    int initialize(int speed, int flow_ctrl, int databits, int stopbits, int parity);
    int get_fd() {return fd_;}

 private:
    int fd_;
    std::string dev_;

 private:
    int uart_open();
    int libtty_rs485set(char enable);
    int uart_set(int speed, int flow_ctrl, int databits, int stopbits, int parity);
};

}  // namespace hj_bf
#endif
