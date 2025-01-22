// @file function_factory.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-15)
#ifndef SRC_BASE_DEVICE_INCLUDE_I2C_H_
#define SRC_BASE_DEVICE_INCLUDE_I2C_H_

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <string>
#include "log.h"

#define RET_ERR     -1
#define RET_OK      0
#define FALSE       0
#define TRUE        1

namespace hj_bf {
class I2C {
 public:
  I2C() = default;
  explicit I2C(std::string dev);
  I2C(const I2C& other);
  I2C& operator =(const I2C& other);
  ~I2C();
  int initialize();
  void close_i2c();
  void set_dev(std::string dev_path) {dev_ = dev_path;}
  int i2c_read_bytes(uint8_t slave_addr, uint8_t reg_addr, uint8_t *values, uint8_t len);
  int i2c_write_bytes(uint8_t slave_addr, uint8_t reg_addr, uint8_t *values, uint8_t len);
 private:
  std::string dev_;
  int fd_;
  int read_error_count_{0};
  int write_error_count_{0};
  struct i2c_rdwr_ioctl_data packets;
  struct i2c_msg messages[2];
  uint8_t read_buf[1];
  uint8_t write_buf[2];
};
}  // namespace hj_bf
#endif  // SRC_BASE_DEVICE_INCLUDE_I2C_H_
