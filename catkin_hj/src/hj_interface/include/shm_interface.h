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
  {
    std::unique_lock<std::mutex> lk(Shm::g_shm_ptr->variables_maps_mutex_);
    if (Shm::g_shm_ptr->variables_maps_.find(name) != Shm::g_shm_ptr->variables_maps_.end()) {
      lk.unlock();
      HJ_ERROR("createVariable fail ,have the same name %s", name.c_str());
      return false;
    }
  }
  VariableImpl* p_value = Shm::g_shm_ptr->segment_->find_or_construct<VariableInstance<T>>(name.c_str())(value);
  Shm::g_shm_ptr->insertValue(name, p_value);
  std::string value_type = typeid(T).name();
  if (value_type.size() >= SHM_TYPE_LEN) {
    HJ_ERROR("createVariable not support too complex type  %s", value_type.c_str());
    return false;
  }
  std::string value_name = SHM_TYPE_PREFIX + name;
  VariableImpl* p_type_value = Shm::g_shm_ptr->segment_->find_or_construct<VariableInstanceArray<char, SHM_TYPE_LEN>>(
      value_name.c_str())(value_type.c_str());
  Shm::g_shm_ptr->insertValue(value_name, p_type_value);
  return true;
}

/*//创建共享变量
 * \param name
 *      变量名
 * \param value
 *      变量值
 */
template <typename T, int N>
bool createVariable(const std::string& name, T (&value)[N]) {
  {
    std::unique_lock<std::mutex> lk(Shm::g_shm_ptr->variables_maps_mutex_);
    if (Shm::g_shm_ptr->variables_maps_.find(name) != Shm::g_shm_ptr->variables_maps_.end()) {
      lk.unlock();
      HJ_ERROR("createVariable array fail ,have the same name %s", name.c_str());
      return false;
    }
  }
  VariableImpl* p_value = Shm::g_shm_ptr->segment_->find_or_construct<VariableInstanceArray<T, N>>(name.c_str())(value);
  Shm::g_shm_ptr->insertValue(name, p_value);
  std::string value_type = typeid(value).name();
  if (value_type.size() >= SHM_TYPE_LEN) {
    HJ_ERROR("createVariable array not support too complex type  %s", value_type.c_str());
    return false;
  }
  std::string value_name = SHM_TYPE_PREFIX + name;
  VariableImpl* p_type_value = Shm::g_shm_ptr->segment_->find_or_construct<VariableInstanceArray<char, SHM_TYPE_LEN>>(
      value_name.c_str())(value_type.c_str());
  Shm::g_shm_ptr->insertValue(value_name, p_type_value);
  return true;
}

/*//获取共享变量
 * \param name
 *      变量名
 * \param value
 *      返回变量
 */
template <typename T>
bool getVariable(const std::string& name, T& value) {
  std::string value_type = typeid(T).name();
  if (value_type.size() >= SHM_TYPE_LEN) {
    HJ_ERROR("getVariable not support too complex type  %s", value_type.c_str());
    return false;
  }
  std::string type_name = SHM_TYPE_PREFIX + name;
  std::unique_lock<std::mutex> lk(Shm::g_shm_ptr->variables_maps_mutex_);
  if (Shm::g_shm_ptr->variables_maps_.find(type_name) != Shm::g_shm_ptr->variables_maps_.end()) {
    std::string type_value =
        (reinterpret_cast<VariableInstanceArray<char, SHM_TYPE_LEN>*>(Shm::g_shm_ptr->variables_maps_[type_name]))
            ->getVariable();
    if (type_value == value_type) {
      value = (reinterpret_cast<VariableInstance<T>*>(Shm::g_shm_ptr->variables_maps_[name]))->getVariable();
    } else {
      lk.unlock();
      HJ_ERROR("getVariable error ,in type:%s, default type:%s", value_type.c_str(), type_value.c_str());
      return false;
    }
  } else {
    lk.unlock();
    std::pair<VariableInstanceArray<char, SHM_TYPE_LEN>*, std::size_t> get_shm_obj_name =
        Shm::g_shm_ptr->segment_->find<VariableInstanceArray<char, SHM_TYPE_LEN>>(type_name.c_str());
    if (get_shm_obj_name.second != 0) {
      std::string temp_shm_obj_name_str = get_shm_obj_name.first->getVariable();
      if (temp_shm_obj_name_str == value_type) {
        Shm::g_shm_ptr->insertValue(type_name, get_shm_obj_name.first);
        std::pair<VariableInstance<T>*, std::size_t> get_shm_obj =
            Shm::g_shm_ptr->segment_->find<VariableInstance<T>>(name.c_str());
        if (get_shm_obj.second != 0) {
          Shm::g_shm_ptr->insertValue(name, get_shm_obj.first);
          value = get_shm_obj.first->getVariable();
        } else {
          Shm::g_shm_ptr->deleteValue(type_name);
          HJ_ERROR("getVariable have the type name :%s, but not have the value", type_name.c_str());
          return false;
        }

      } else {
        HJ_ERROR("getVariable error2 ,get type name:%s, default type:%s", value_type.c_str(),
                 get_shm_obj_name.first->getVariable());
        return false;
      }
    } else {
      HJ_ERROR("getVariable have not name :%s", name.c_str());
      return false;
    }
  }
  return true;
}

/*//获取共享变量
 * \param name
 *      变量名
 * \param value
 *      返回变量
 */
template <typename T, int N>
bool getVariable(const std::string& name, T (&value)[N]) {
  std::string value_type = typeid(value).name();
  if (value_type.size() >= SHM_TYPE_LEN) {
    HJ_ERROR("getVariable array not support too complex type  %s", value_type.c_str());
    return false;
  }
  std::string type_name = SHM_TYPE_PREFIX + name;
  std::unique_lock<std::mutex> lk(Shm::g_shm_ptr->variables_maps_mutex_);
  if (Shm::g_shm_ptr->variables_maps_.find(type_name) != Shm::g_shm_ptr->variables_maps_.end()) {
    std::string type_value =
        (reinterpret_cast<VariableInstanceArray<char, SHM_TYPE_LEN>*>(Shm::g_shm_ptr->variables_maps_[type_name]))
            ->getVariable();
    if (type_value == value_type) {
      memcpy(value,
             ((reinterpret_cast<VariableInstanceArray<T, N>*>(Shm::g_shm_ptr->variables_maps_[name]))->getVariable()),
             sizeof(T) * N);
     } else {
      lk.unlock();
      HJ_ERROR("getVariable array error ,in type:%s, default type:%s", value_type.c_str(), type_value.c_str());
      return false;
    }
  } else {
    lk.unlock();
    std::pair<VariableInstanceArray<char, SHM_TYPE_LEN>*, std::size_t> get_shm_obj_name =
        Shm::g_shm_ptr->segment_->find<VariableInstanceArray<char, SHM_TYPE_LEN>>(type_name.c_str());
    if (get_shm_obj_name.second != 0) {
      std::string temp_shm_obj_name_str = get_shm_obj_name.first->getVariable();
      if (temp_shm_obj_name_str == value_type) {
        Shm::g_shm_ptr->insertValue(type_name, get_shm_obj_name.first);
        std::pair<VariableInstanceArray<T, N>*, std::size_t> get_shm_obj =
            Shm::g_shm_ptr->segment_->find<VariableInstanceArray<T, N>>(name.c_str());
        if (get_shm_obj.second != 0) {
          Shm::g_shm_ptr->insertValue(name, get_shm_obj.first);
          memcpy(value, (get_shm_obj.first->getVariable()), sizeof(T) * N);
        } else {
          Shm::g_shm_ptr->deleteValue(type_name);
          HJ_ERROR("getVariable array ave the type name :%s, but not have the value", type_name.c_str());
          return false;
        }
      } else {
        HJ_ERROR("getVariable array error2 ,get type name:%s, default type:%s", value_type.c_str(),
                 get_shm_obj_name.first->getVariable());
        return false;
      }
    } else {
      HJ_ERROR("getVariable array have not name :%s", name.c_str());
      return false;
    }
  }
  return true;
}

/*//设置共享变量
 * \param name
 *      变量名
 * \param value
 *      变量值
 */
template <typename T, int N>
bool setVariable(const std::string& name, T (&value)[N]) {
  std::string value_type = typeid(value).name();
  if (value_type.size() >= SHM_TYPE_LEN) {
    HJ_ERROR("setVariable array not support too complex type  %s", value_type.c_str());
    return false;
  }
  std::string type_name = SHM_TYPE_PREFIX + name;
  std::unique_lock<std::mutex> lk(Shm::g_shm_ptr->variables_maps_mutex_);
  if (Shm::g_shm_ptr->variables_maps_.find(type_name) != Shm::g_shm_ptr->variables_maps_.end()) {
    std::string type_value =
        (reinterpret_cast<VariableInstanceArray<char, SHM_TYPE_LEN>*>(Shm::g_shm_ptr->variables_maps_[type_name]))
            ->getVariable();
    if (type_value == value_type) {
      (reinterpret_cast<VariableInstanceArray<T, N>*>(Shm::g_shm_ptr->variables_maps_[name]))->setVariable(value);
    } else {
      lk.unlock();
      HJ_ERROR("setVariable array error ,in type:%s, default type:%s", value_type.c_str(), type_value.c_str());
      return false;
    }
  } else {
    lk.unlock();
    std::pair<VariableInstanceArray<char, SHM_TYPE_LEN>*, std::size_t> get_shm_obj_name =
        Shm::g_shm_ptr->segment_->find<VariableInstanceArray<char, SHM_TYPE_LEN>>(type_name.c_str());
    if (get_shm_obj_name.second != 0) {
      std::string temp_shm_obj_name_str = get_shm_obj_name.first->getVariable();
      if (temp_shm_obj_name_str == value_type) {
        Shm::g_shm_ptr->insertValue(type_name, get_shm_obj_name.first);
        std::pair<VariableInstanceArray<T, N>*, std::size_t> get_shm_obj =
            Shm::g_shm_ptr->segment_->find<VariableInstanceArray<T, N>>(name.c_str());
        if (get_shm_obj.second != 0) {
          Shm::g_shm_ptr->insertValue(name, get_shm_obj.first);
          get_shm_obj.first->setVariable(value);
        } else {
          Shm::g_shm_ptr->deleteValue(type_name);
          HJ_ERROR("setVariable array have the type name :%s, but not have the value", type_name.c_str());
          return false;
        }
      } else {
        HJ_ERROR("setVariable array error2 ,in type:%s, default type:%s", value_type.c_str(),
                 get_shm_obj_name.first->getVariable());
        return false;
      }
    } else {
      if (createVariable(name, value)) {
        HJ_INFO("you should create variable first  %s", name.c_str());
        return setVariable(name, value);
      } else {
        HJ_ERROR("setVariable array fail!");
        return false;
      }
    }
  }
  return true;
}

/*//设置共享变量
 * \param name
 *      变量名
 * \param value
 *      变量值
 */
template <typename T>
bool setVariable(const std::string& name, T& value) {
  std::string value_type = typeid(T).name();
  if (value_type.size() >= SHM_TYPE_LEN) {
    HJ_ERROR("setVariable not support too complex type  %s", value_type.c_str());
    return false;
  }
  std::string type_name = SHM_TYPE_PREFIX + name;
  std::unique_lock<std::mutex> lk(Shm::g_shm_ptr->variables_maps_mutex_);
  if (Shm::g_shm_ptr->variables_maps_.find(type_name) != Shm::g_shm_ptr->variables_maps_.end()) {
    std::string type_value =
        (reinterpret_cast<VariableInstanceArray<char, SHM_TYPE_LEN>*>(Shm::g_shm_ptr->variables_maps_[type_name]))
            ->getVariable();
    if (type_value == value_type) {
      (reinterpret_cast<VariableInstance<T>*>(Shm::g_shm_ptr->variables_maps_[name]))->setVariable(value);
    } else {
      lk.unlock();
      HJ_ERROR("setVariable error ,in type:%s, default type:%s", value_type.c_str(), type_value.c_str());
      return false;
    }
  } else {
    lk.unlock();
    std::pair<VariableInstanceArray<char, SHM_TYPE_LEN>*, std::size_t> get_shm_obj_name =
        Shm::g_shm_ptr->segment_->find<VariableInstanceArray<char, SHM_TYPE_LEN>>(type_name.c_str());
    if (get_shm_obj_name.second != 0) {
      std::string temp_shm_obj_name_str = get_shm_obj_name.first->getVariable();
      if (temp_shm_obj_name_str == value_type) {
        Shm::g_shm_ptr->insertValue(type_name, get_shm_obj_name.first);
        std::pair<VariableInstance<T>*, std::size_t> get_shm_obj =
            Shm::g_shm_ptr->segment_->find<VariableInstance<T>>(name.c_str());
        if (get_shm_obj.second != 0) {
          Shm::g_shm_ptr->insertValue(name, get_shm_obj.first);
          get_shm_obj.first->setVariable(value);
        } else {
          Shm::g_shm_ptr->deleteValue(type_name);
          HJ_ERROR("setVariable have the type name :%s, but not have the value", type_name.c_str());
          return false;
        }
      } else {
        HJ_ERROR("setVariable error2 ,in type:%s, default type:%s", value_type.c_str(),
                 get_shm_obj_name.first->getVariable());
        return false;
      }
    } else {
      if (createVariable(name, value)) {
        HJ_INFO("you should create variable first  %s", name.c_str());
        return setVariable(name, value);
      } else {
        HJ_ERROR("setVariable fail!");
        return false;
      }
    }
  }
  return true;
}
}  // namespace hj_bf
#endif  //  INCLUDE_SHM_H_  // NOLINT
