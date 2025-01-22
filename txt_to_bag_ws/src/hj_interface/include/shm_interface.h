// @file shm_interface.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2024-5-16)
#ifndef INCLUDE_SHM_INTERFACE_H_  // NOLINT
#define INCLUDE_SHM_INTERFACE_H_  // NOLINT
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/make_shared.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "log.h"
#include "shm.h"
namespace hj_bf {
/*//创建共享变量
 * \param name
 *      变量名
 * \param value
 *      变量值
 */
template <typename T>
bool createVariable(const std::string& name, const T& value) {
  VariableImpl* p_value = Shm::g_shm_ptr->segment_->find_or_construct<VariableInstance<T>>(name.c_str())(value);
  std::string value_type = typeid(T).name();
  return Shm::g_shm_ptr->createVariable(value_type, name, p_value);
}

/*//创建共享变量
 * \param name
 *      变量名
 * \param value
 *      变量值
 */
template <typename T, int N>
bool createVariable(const std::string& name, T (&value)[N]) {
  VariableImpl* p_value = Shm::g_shm_ptr->segment_->find_or_construct<VariableInstanceArray<T, N>>(name.c_str())(value);
  std::string value_type = typeid(value).name();
  return Shm::g_shm_ptr->createVariable(value_type, name, p_value);
}

/*//获取共享变量
 * \param name
 *      变量名
 * \param value
 *      返回变量
 */
template <typename T>
bool getVariable(const std::string& name, T& value) {
  bool need_insert = false;
  bool ret = false;
  std::string value_type = typeid(T).name();
  ret = Shm::g_shm_ptr->getVariable(value_type, name, need_insert);
  if (true == ret) {
    value = (reinterpret_cast<VariableInstance<T>*>(Shm::g_shm_ptr->variables_maps_[name]))->getVariable();
  }
  if (true == need_insert) {
    std::pair<VariableInstance<T>*, std::size_t> get_shm_obj =
        Shm::g_shm_ptr->segment_->find<VariableInstance<T>>(name.c_str());
    bool temp_insert = (get_shm_obj.second != 0);
    if (temp_insert) {
      value = get_shm_obj.first->getVariable();
    }
    return Shm::g_shm_ptr->insertVariable(name, get_shm_obj.first, temp_insert);
  }
  return ret;
}

/*//获取共享变量
 * \param name
 *      变量名
 * \param value
 *      返回变量
 */
template <typename T, int N>
bool getVariable(const std::string& name, T (&value)[N]) {
  bool need_insert = false;
  bool ret = false;
  std::string value_type = typeid(value).name();
  ret = Shm::g_shm_ptr->getVariable(value_type, name, need_insert);
  if (true == ret) {
    memcpy(value,
           ((reinterpret_cast<VariableInstanceArray<T, N>*>(Shm::g_shm_ptr->variables_maps_[name]))->getVariable()),
           sizeof(T) * N);
  }
  if (true == need_insert) {
    std::pair<VariableInstanceArray<T, N>*, std::size_t> get_shm_obj =
        Shm::g_shm_ptr->segment_->find<VariableInstanceArray<T, N>>(name.c_str());
    bool temp_insert = (get_shm_obj.second != 0);
    if (temp_insert) {
      memcpy(value, (get_shm_obj.first->getVariable()), sizeof(T) * N);
    }
    return Shm::g_shm_ptr->insertVariable(name, get_shm_obj.first, temp_insert);
  }
  return ret;
}

/*//设置共享变量
 * \param name
 *      变量名
 * \param value
 *      变量值
 */
template <typename T, int N>
bool setVariable(const std::string& name, T (&value)[N]) {
  bool need_insert = false;
  bool need_create = false;
  bool ret = false;
  std::string value_type = typeid(value).name();
  ret = Shm::g_shm_ptr->setVariable(value_type, name, need_insert, need_create);
  if (true == ret) {
    (reinterpret_cast<VariableInstanceArray<T, N>*>(Shm::g_shm_ptr->variables_maps_[name]))->setVariable(value);
    return ret;
  }
  if (true == need_insert) {
    std::pair<VariableInstanceArray<T, N>*, std::size_t> get_shm_obj =
        Shm::g_shm_ptr->segment_->find<VariableInstanceArray<T, N>>(name.c_str());
    bool temp_insert = (get_shm_obj.second != 0);
    if (temp_insert) {
      get_shm_obj.first->setVariable(value);
    }
    return Shm::g_shm_ptr->insertVariable(name, get_shm_obj.first, temp_insert);
  }
  if (true == need_create) {
    if (createVariable(name, value)) {
      HJ_INFO("you should create array variable first  %s", name.c_str());
      return setVariable(name, value);
    } else {
      HJ_ERROR("setVariable fail!");
      return false;
    }
  }
  return ret;
}
/*//设置共享变量
 * \param name
 *      变量名
 * \param value
 *      变量值
 */
template <typename T>
bool setVariable(const std::string& name, T& value) {
  bool need_insert = false;
  bool need_create = false;
  bool ret = false;
  std::string value_type = typeid(T).name();
  ret = Shm::g_shm_ptr->setVariable(value_type, name, need_insert, need_create);
  if (true == ret) {
    (reinterpret_cast<VariableInstance<T>*>(Shm::g_shm_ptr->variables_maps_[name]))->setVariable(value);
    return ret;
  }
  if (true == need_insert) {
    std::pair<VariableInstance<T>*, std::size_t> get_shm_obj =
        Shm::g_shm_ptr->segment_->find<VariableInstance<T>>(name.c_str());
    bool temp_insert = (get_shm_obj.second != 0);
    if (temp_insert) {
      get_shm_obj.first->setVariable(value);
    }
    return Shm::g_shm_ptr->insertVariable(name, get_shm_obj.first, temp_insert);
  }
  if (true == need_create) {
    if (createVariable(name, value)) {
      HJ_INFO("you should create normal variable first  %s", name.c_str());
      return setVariable(name, value);
    } else {
      HJ_ERROR("setVariable fail!");
      return false;
    }
  }
  return ret;
}
}  // namespace hj_bf
#endif  //  INCLUDE_SHM_H_  // NOLINT
