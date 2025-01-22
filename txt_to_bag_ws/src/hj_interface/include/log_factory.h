
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
constexpr char g_log_config_path_param_name[] = "HJ_LOG_CONFIG_PATH";
constexpr char g_slam_node_log_config_path_param_name[] = "HJ_SLAM_NODE_LOG_CONFIG_PATH";
bool logInit(const std::string& config_path, const std::vector<unsigned char>& pass_word);
}  // namespace hj_bf
#endif
