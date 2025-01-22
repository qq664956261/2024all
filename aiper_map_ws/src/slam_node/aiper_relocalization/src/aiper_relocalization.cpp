// @file demo.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-22)
#include "aiper_relocalization.h"

HJ_REGISTER_FUNCTION(factory)
{
  std::cerr << "minos register factory" << FUNCTION_NAME << std::endl;
  factory.registerCreater<aiper_relocalization_ns::AiperMap>(FUNCTION_NAME);
}
namespace aiper_relocalization_ns
{
  
  AiperMap::AiperMap(const rapidjson::Value &json_conf) : hj_bf::Function(json_conf)
  {
    _relocalization_ptr = std::make_shared<Relocalization>();

  }
} // namespace aiper_relocalization_ns
