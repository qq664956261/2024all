// @file slam_node_manager.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-15)
#include <sstream>
#include <thread>

#include "def.h"
#include "function_factory.h"
#include "node_factory.h"
#include "log_factory.h"
#include "node_cache.h"
#include "log.h"
#include "shm.h"
#include "big_data.h"
#include <fstream>
int main(int argc, char** argv) {
  std::string temp_config_path;

#if 1  // for debug, 优化开机时间
  std::fstream file("/proc/uptime", std::ios::in);
  if (file.is_open()) {
    std::string line;
    std::getline(file, line);
    std::cout << "slam Current uptime:" << line.c_str() << std::endl;
    file.close();
  }
#endif
  char* switch_cstr = NULL;
  switch_cstr = getenv(hj_bf::g_node_config_file_env_set);
  if (switch_cstr) {
    temp_config_path = switch_cstr;
  }

  std::shared_ptr<hj_bf::NodeConfig> node_config = std::make_shared<hj_bf::NodeConfig>();
  node_config->node_name = NODE_NAME;
  node_config->log_config_path = LOG_CONFIG_PATH;

  // std::vector<unsigned char> crypt_val(CRYPT_VAL, CRYPT_VAL + strlen(CRYPT_VAL));
  std::string crypt_val = CRYPT_VAL;
  // std::cout << "CRYPT_VAL: " << crypt_val << std::endl;
  node_config->crypt_val.assign(crypt_val.begin(), crypt_val.end());
#ifdef HJ_AMD64
  if (temp_config_path.empty()) {
    hj_bf::readConfigure(CONFIG_PATH, node_config);
  } else {
    temp_config_path =
        temp_config_path + "/" + NODE_NAME + "/" + NODE_NAME + "/config" + "/" + PROJECT_NUMBER + "/amd64/config.json";
    hj_bf::readConfigure(temp_config_path, node_config);
  }
#endif
#ifdef HJ_AARCH64
  temp_config_path = temp_config_path + "/" + NODE_NAME + "/config.json";
  hj_bf::readConfigure(temp_config_path, node_config);
#endif
  uint32_t temp_ops = 0;
  if (node_config->all_log_close == true) {
    std::cout << "close all log: " << node_config->node_name << std::endl;
    temp_ops = ros::init_options::NoRosout;
  }
  hj_bf::nodeInit(argc, argv, node_config->node_name, node_config, temp_ops);
  if (node_config->node_log_close == false) {
    hj_bf::logInit(node_config->log_config_path, node_config->crypt_val);
  } else {
    std::cout << "close log, node name:" << node_config->node_name << std::endl;
  }

  hj_bf::Shm::createInstance();
  big_data::Init();
  hj_bf::HealthCheckInit();

  hj_bf::nodeInstance();
  hj_bf::nodeStart();

#if 1  // for debug, 优化开机时间
  std::fstream file1("/proc/uptime", std::ios::in);
  if (file1.is_open()) {
    std::string line;
    std::getline(file1, line);
    std::cout << "slam end Current uptime:" << line.c_str() << std::endl;
    file1.close();
  }
#endif
  hj_bf::nodeSpin();
  //  ros::waitForShutdown();
  return 0;
}
// %EndTag(FULLTEXT)%
