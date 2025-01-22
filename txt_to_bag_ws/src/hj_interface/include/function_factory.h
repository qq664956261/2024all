// @file function_factory.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-15)
#ifndef FUNCTION_FACTORY_H
#define FUNCTION_FACTORY_H
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rapidjson/document.h"
namespace hj_bf {

#define STR1(var) #var
#define STR2(var) STR1(var)

#define HJ_REGISTER_NAME HJRegisterFunction
#define HJ_REGISTER_NAME_STR STR2(HJ_REGISTER_NAME)
#define HJ_REGISTER_FUNCTION(factory) \
  extern "C" void __attribute__((visibility("default"))) HJ_REGISTER_NAME(hj_bf::FunctionFactory &factory)
#define HJ_INTERFACE_VERSION "v0.1.00"
class Function {
 public:
  void setHJInterfaceVersion(const std::string &interface_ver);
  Function(const rapidjson::Value &json_conf);
  Function() = delete;
  Function(const Function &) = delete;
  Function(Function &&) = delete;
  Function &operator=(const Function &) = delete;
  Function &operator=(Function &&) = delete;
  virtual ~Function();
  const std::string &name();
  const std::string &getHJInterfaceVersion();
  struct FunctionConfig;

 private:
  std::unique_ptr<FunctionConfig> config_;
};
typedef std::function<std::unique_ptr<Function>(const rapidjson::Value &)> FunctionCreater;
class FunctionFactory {
 public:
  FunctionFactory() {}
  FunctionFactory(const FunctionFactory &) = delete;
  FunctionFactory(FunctionFactory &&) = delete;
  FunctionFactory &operator=(const FunctionFactory &) = delete;
  FunctionFactory &operator=(FunctionFactory &&) = delete;
  virtual ~FunctionFactory() = default;
  template <typename T>
  void registerCreater(const std::string &ID) {
    FunctionCreater creater = getCreater<T>();
    insertCreater(ID, creater);
  }
  void registerFunction(const std::string &so_path);
  std::unique_ptr<Function> instantiateFunction(const rapidjson::Value &json_conf) const;

 private:
  template <typename T>
  FunctionCreater getCreater() {
    return [](const rapidjson::Value &json_conf) {
      auto function_instance = std::unique_ptr<Function>(new T(json_conf));
      function_instance->setHJInterfaceVersion(HJ_INTERFACE_VERSION);
      return function_instance;
    };
  }
  void insertCreater(const std::string &ID, FunctionCreater creater);
  std::map<std::string, FunctionCreater> creaters_;
};
}  // namespace hj_bf
#endif
