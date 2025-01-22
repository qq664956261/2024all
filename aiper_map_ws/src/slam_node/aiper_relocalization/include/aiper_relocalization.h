// @file demo.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-22)
#ifndef INCLUDE_DEMO_H//your macro
#define INCLUDE_DEMO_H
#include "function_factory.h"
#include "node_factory.h"
#include "relocalization/relocalization.h"

namespace aiper_relocalization_ns {//your namespace

class AiperMap : public hj_bf::Function {
 public:
  explicit AiperMap(const rapidjson::Value &json_conf);
  ~AiperMap(){};

 private:
  Relocalization::Ptr _relocalization_ptr;
};
}  // namespace aiper_relocalization_ns

#endif
