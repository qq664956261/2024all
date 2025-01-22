/**
 * @file i2c.h
 * @author hao wu (clayderman@yardbot.net)
 * @brief 
 * @version 0.1
 * @date 2024-06-21
 * 
 * @copyright Copyright 2023 HJ Technology Co.Ltd. All rights reserved
 * 
 */
#ifndef SRC_BASE_DEVICE_INCLUDE_I2C_H_
#define SRC_BASE_DEVICE_INCLUDE_I2C_H_

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <string>
#include "log.h"

namespace hj_bf {
class I2C {
 public:
  I2C() = default;
  explicit I2C(std::string dev);
  I2C(const I2C& other);
  I2C& operator =(const I2C& other);
  ~I2C();
  bool Initialize();
  // void CloseI2c();
  void SetDev(std::string dev_path) {dev_ = dev_path;}
  bool I2cReadBytes(uint8_t slave_addr, uint8_t reg_addr, uint8_t *values, uint8_t len);
  bool I2cWriteBytes(uint8_t slave_addr, uint8_t reg_addr, uint8_t *values, uint8_t len);
 private:
  std::string dev_;
  int fd_;
  int read_error_count_{0};
  int write_error_count_{0};
  struct i2c_rdwr_ioctl_data packets_;
  struct i2c_msg messages[2];
  uint8_t read_buf[1];
  uint8_t write_buf[2];
};
}  // namespace hj_bf
#endif  // SRC_BASE_DEVICE_INCLUDE_I2C_H_
