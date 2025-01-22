// @file def.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-15)
#ifndef INCLUDE_DEF_H
#define INCLUDE_DEF_H
#include <string>
#ifdef HJ_AMD64
constexpr char g_configure_file_name[] = CONFIG_PATH;
#endif
#ifdef HJ_ARM64_RK
constexpr char g_configure_file_name[] = "/home/wangqing/work_space/hj_workspace/src/collect_node/collect_node/config/config.json";
#endif

constexpr char g_lib_path_param_name[] = "/hj_so_path";
constexpr char g_config_path_param_name[] = "/hj_config_path";
#endif
