
// @file function_factory.h
// @brief
//
// Copyright 2024 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2024-3-7)

#ifndef LOG_FACTORY_H
#define LOG_FACTORY_H
#include <iostream>
#include <sstream>

#include "ros/ros.h"
namespace hj_bf {
constexpr char g_all_log_close[] = "HJ_ALL_LOG_CLOSE";
bool logInit(const std::string& config_path);
}
#endif
