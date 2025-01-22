// @file demo.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-22)
#include "aiper_map.h"

HJ_REGISTER_FUNCTION(factory)
{
  std::cerr << "minos register factory" << FUNCTION_NAME << std::endl;
  factory.registerCreater<aiper_map_ns::AiperMap>(FUNCTION_NAME);
}
namespace aiper_map_ns
{
  
  AiperMap::AiperMap(const rapidjson::Value &json_conf) : hj_bf::Function(json_conf)
  {
    std::cout<<"111"<<std::endl;
    pRelocalization = std::shared_ptr<Relocalization>(new Relocalization());
    std::cout<<"222"<<std::endl;

  }
} // namespace aiper_map_ns
