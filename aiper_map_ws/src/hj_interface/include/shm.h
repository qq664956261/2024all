// @file shm.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2024-5-16)
#ifndef INCLUDE_SHM_H_  // NOLINT
#define INCLUDE_SHM_H_  // NOLINT
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/make_shared.hpp>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "log.h"

namespace hj_bf {

class VariableImpl {
 public:
  // VariableInstance(T value, std::string name){
  //   value_ = segment->find_or_construct < T > (name.c_str())();
  // }
};
template <typename T>
class VariableInstance : public VariableImpl {
 public:
  // VariableInstance(T value, std::string name){
  //   value_ = segment->find_or_construct < T > (name.c_str())();
  // }

  explicit VariableInstance(const T& value) { value_ = value; }
  VariableInstance(const VariableInstance&) = delete;
  VariableInstance(VariableInstance&&) = delete;
  VariableInstance& operator=(const VariableInstance&) = delete;
  VariableInstance& operator=(VariableInstance&&) = delete;
  ~VariableInstance() = default;

  T getVariable() {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex_);
    return value_;
  }
  void setVariable(const T& value) {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex_);
    value_ = value;
  }

 private:
  T value_;
  boost::interprocess::interprocess_mutex mutex_;
};

template <typename T, int N>
class VariableInstanceArray : public VariableImpl {
 public:
  // VariableInstance(T value, std::string name){
  //   value_ = segment->find_or_construct < T > (name.c_str())();
  // }
  VariableInstanceArray(const T value[N]) { memcpy(value_, value, sizeof(T) * N); }
  VariableInstanceArray(const VariableInstanceArray&) = delete;
  VariableInstanceArray(VariableInstanceArray&&) = delete;
  VariableInstanceArray& operator=(const VariableInstanceArray&) = delete;
  VariableInstanceArray& operator=(VariableInstanceArray&&) = delete;
  ~VariableInstanceArray() = default;

  T* getVariable() {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex_);
    return value_;
  }
  void setVariable(const T (&value)[N]) {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex_);
    memcpy(value_, value, sizeof(T) * N);
    // value_ = value;
  }

 private:
  T value_[N];
  boost::interprocess::interprocess_mutex mutex_;
};

class Shm {
 public:
#define SHM_SIZE 100  // pages
#define SHM_TYPE_LEN 50
#define SHM_NAME "MINOS_SHM"
#define SHM_TYPE_PREFIX "type_"
  Shm(const Shm&) = delete;
  Shm(Shm&&) = delete;
  Shm& operator=(const Shm&) = delete;
  Shm& operator=(Shm&&) = delete;
  ~Shm();
  template <typename T>
  friend bool createVariable(const std::string& name, const T& value);
  template <typename T, int N>
  friend bool createVariable(const std::string& name, T (&value)[N]);
  template <typename T>
  friend bool getVariable(const std::string& name, T& value);
  template <typename T, int N>
  friend bool getVariable(const std::string& name, T (&value)[N]);

  template <typename T>
  friend bool setVariable(const std::string& name, T& value);

  template <typename T, int N>
  friend bool setVariable(const std::string& name, T (&value)[N]);
  template <typename T>
  friend void destroyVariable(const std::string& name);
  static void createInstance();

 private:
  friend class MinosLock;
  Shm();
  void insertValue(const std::string& name, VariableImpl* ptr);
  void deleteValue(const std::string& name);
  bool createVariable(const std::string& value_type, const std::string& name, VariableImpl* p_value);
  bool insertVariable(const std::string& name, VariableImpl* ptr, bool insert);
  bool getVariable(const std::string& value_type, const std::string& name, bool& need_insert);
  bool setVariable(const std::string& value_type, const std::string& name, bool& need_insert, bool& need_create);
  std::shared_ptr<boost::interprocess::managed_shared_memory> segment_;
  std::unordered_map<std::string, VariableImpl*> variables_maps_;
  std::mutex variables_maps_mutex_;
  static Shm* g_shm_ptr;
};

class MinosLock {
 public:
  MinosLock(const std::string &name, bool lock_im = true);
  MinosLock(const MinosLock&) = delete;
  MinosLock(MinosLock&&) = delete;
  MinosLock& operator=(const MinosLock&) = delete;
  MinosLock& operator=(MinosLock&&) = delete;
  ~MinosLock();
  void lock();
  void unlock();
  std::shared_ptr<boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex>> getLock();
  struct MinosLockVariable;
private:

#define LOCK_PREFIX "m_"
  std::unique_ptr<MinosLockVariable> variable_ptr_;
};

template <typename T>
void destroyVariable(const std::string& name) {
  std::string type_name = SHM_TYPE_PREFIX + name;
  {
    std::lock_guard<std::mutex> lk(Shm::g_shm_ptr->variables_maps_mutex_);
    Shm::g_shm_ptr->deleteValue(type_name);
    Shm::g_shm_ptr->deleteValue(name);
  }
  Shm::g_shm_ptr->segment_->destroy<T>(name.c_str());
}
}  // namespace hj_bf
#endif  //  INCLUDE_SHM_H_  // NOLINT
