// @file node_impl.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-15)
#ifndef INCLUDE_UNODE_IMPL_H_  // NOLINT
#define INCLUDE_UNODE_IMPL_H_  // NOLINT
#include <string>
#include <vector>
namespace hj_bf {

class NodeImpl {
 public:
  virtual ~NodeImpl() = default;
  virtual bool init(int argc, char** argv, std::string& name) = 0;
  virtual bool publisher(const std::string& param_path, std::vector<std::string>* names) = 0;
  virtual bool subscriber(const std::string& remote_path, const std::string& filedata) = 0;
  //  virtual bool deleteSpace(const std::string& name){};
  virtual bool nodePublish(const std::string& remote_path, const std::string& file) = 0;
  virtual bool nodeSucribe(const std::string& remote_path, std::string* outString) = 0;
};

}  // namespace hj_bf
#endif  //  INCLUDE_UNODE_IMPL_H_  // NOLINT
