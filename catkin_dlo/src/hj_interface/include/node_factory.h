// @file node_factory.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-15)
#ifndef INCLUDE_COMMUNICATION_H
#define INCLUDE_COMMUNICATION_H
#include <iostream>
#include <map>
#include <mutex>
#include <unordered_map>

#include "function_factory.h"
#include "ros/ros.h"

namespace hj_bf {
class HJPublisher {
 public:
  HJPublisher(const std::string& topic, const std::shared_ptr<ros::Publisher>& pub)
      : publisher_ptr_(pub), name_(topic) {}
  HJPublisher(){};
  HJPublisher(const HJPublisher&) = default;
  HJPublisher(HJPublisher&&) = default;
  HJPublisher& operator=(const HJPublisher&) = default;
  HJPublisher& operator=(HJPublisher&&) = default;
  ~HJPublisher();
  template <typename M>
  void publish(const M& message) const {
    // minos record
    if (nullptr != publisher_ptr_) {
      publisher_ptr_->publish(message);
    } else {
      std::cerr << "publisher:" << name_ << " is been deleted" << std::endl;
    }
  }
  uint32_t getNumSubscribers() const;
  void shutdown() = delete;
  inline std::string getTopic() const { return name_; };

 private:
  std::shared_ptr<ros::Publisher> publisher_ptr_;
  std::string name_;
};

class HJSubscriber {
 public:
  HJSubscriber(const std::string& topic, const std::shared_ptr<ros::Subscriber>& sub, uint32_t index)
      : subscriber_ptr_(sub), name_(topic), index_(index) {}
  HJSubscriber(){};
  HJSubscriber(const HJSubscriber&) = default;
  HJSubscriber(HJSubscriber&&) = default;
  HJSubscriber& operator=(const HJSubscriber&) = default;
  HJSubscriber& operator=(HJSubscriber&&) = default;
  ~HJSubscriber(){};
  void shutdown() = delete;
  std::string getTopic() const { return name_; };

 private:
  std::shared_ptr<ros::Subscriber> subscriber_ptr_;
  std::string name_;
  uint32_t index_;
};

class HJClient {
 public:
  HJClient(const std::string& topic, const std::shared_ptr<ros::ServiceClient>& client, uint32_t index)
      : client_ptr_(client), name_(topic), index_(index) {}
  HJClient(){};
  HJClient(const HJClient&) = default;
  HJClient(HJClient&&) = default;
  HJClient& operator=(const HJClient&) = default;
  HJClient& operator=(HJClient&&) = default;
  ~HJClient();
  template <class Service>
  bool call(Service& service) {
    return client_ptr_->call(service);
  }
  void shutdown() = delete;
  std::string getService() const { return name_; };
  bool isValid() const;

 private:
  std::shared_ptr<ros::ServiceClient> client_ptr_;
  std::string name_;
  uint32_t index_;
};

class HJServer {
 public:
  HJServer(const std::string& service, const std::shared_ptr<ros::ServiceServer>& server)
      : server_ptr_(server), name_(service) {}
  HJServer() {}
  HJServer(const HJServer&) = default;
  HJServer(HJServer&&) = default;
  HJServer& operator=(const HJServer&) = default;
  HJServer& operator=(HJServer&&) = default;
  ~HJServer();
  void shutdown() = delete;
  std::string getService() const { return name_; }

 private:
  std::shared_ptr<ros::ServiceServer> server_ptr_;
  std::string name_;
};

class HJTimer {
 public:
  HJTimer(const double us, const std::shared_ptr<ros::Timer>& timer, const std::string& name)
      : duration_(us), timer_ptr_(timer), name_(name) {}
  HJTimer() {}
  HJTimer(const HJTimer&) = default;
  HJTimer(HJTimer&&) = default;
  HJTimer& operator=(const HJTimer&) = default;
  HJTimer& operator=(HJTimer&&) = default;
  ~HJTimer();

  void shutdown() = delete;
  std::string getName() { return name_; }
  void start();
  void stop();
  void setPeriod(const double us);

 private:
  double duration_;
  std::shared_ptr<ros::Timer> timer_ptr_;
  std::string name_;
};
typedef ros::TimerEvent HJTimerEvent;
typedef std::function<void(const HJTimerEvent&)> HJTimerCallback;

class ManagerNode {
 public:
  static void createInstance(uint32_t thread_count);
  friend void nodeInit(int argc, char** argv, const std::string& node_name, uint32_t thread_count, uint32_t ops);
  friend void nodeStart();
  friend void nodeSpin();
  friend void getConfigure(const std::string& file, const std::string& so_path);
  friend ros::NodeHandle& getHandle();

  template <class T>
  friend HJPublisher HJAdvertise(const std::string& topic, uint32_t queue_size);
  template <class M>
  friend HJSubscriber HJSubscribe(const std::string& topic, uint32_t queue_size, void (*fp)(M));
  template <class C>
  friend HJSubscriber HJSubscribe(const std::string& topic, uint32_t queue_size,
                                  const boost::function<void(C)>& callback);
  template <class T>
  friend HJClient HJCreateClient(const std::string& topic);

  template <class MReq, class MRes>
  friend HJServer HJCreateServer(const std::string& service, bool (*srv_func)(MReq&, MRes&));

  template <class MReq, class MRes>
  friend HJServer HJCreateServer(const std::string& service, const boost::function<bool(MReq&, MRes&)>& callback);

  template <class MReq, class MRes, class T>
  friend HJServer HJCreateServer(const std::string& service, bool (T::*srv_func)(MReq&, MRes&), T* obj);

  friend HJTimer HJCreateTimer(const std::string name, double us, const HJTimerCallback& callback);
  template <class T>
  friend HJTimer HJCreateTimer(const std::string name, double us, void (T::*callback)(const HJTimerEvent&), T* obj);
  template <typename T>
  friend bool HJGetParam(const std::string& key, T& param_val);
  template <typename T>
  friend void HJSetParam(const std::string& key, T& param_val);

  friend void deletePublisher(const std::string& publisher_name);
  friend void deleteSubscriber(const std::string& subscriber_name, uint32_t index);
  friend void deleteClient(const std::string& service_name, uint32_t index);
  friend void deleteServer(const std::string& service_name);
  friend void deleteTimer(const std::string& timer_name);
  friend uint32_t getNumPublisher();
  friend uint32_t getNumSubscriber(const std::string topic);
  friend uint32_t getAllSubscriber();

  friend uint32_t getNumServer();
  friend uint32_t getNumClient(const std::string topic);
  friend uint32_t getAllClient();

  friend uint32_t getNumTimer();
  ManagerNode() = delete;
  ManagerNode(const ManagerNode&) = delete;
  ManagerNode(ManagerNode&&) = delete;
  ManagerNode& operator=(const ManagerNode&) = delete;
  ManagerNode& operator=(ManagerNode&&) = delete;
  ~ManagerNode();

 private:
  ManagerNode(uint32_t thread_count) : node_handle_ptr_(std::make_shared<ros::NodeHandle>()), spinner_(thread_count){};

  std::unordered_map<std::string, std::unique_ptr<Function>> hj_functions_;
  std::shared_ptr<ros::NodeHandle> node_handle_ptr_;
  std::unordered_map<std::string, std::shared_ptr<ros::Publisher>> publishers_;
  std::unordered_map<std::string, std::map<uint32_t, std::shared_ptr<ros::Subscriber>>> subscribers_;
  std::unordered_map<std::string, std::map<uint32_t, std::shared_ptr<ros::ServiceClient>>> clients_;
  std::unordered_map<std::string, std::shared_ptr<ros::ServiceServer>> servers_;
  std::unordered_map<std::string, std::shared_ptr<ros::Timer>> timers_;
  uint32_t subscribers_index_ = 0;
  uint32_t clients_index_ = 0;
  ros::AsyncSpinner spinner_;

  std::mutex publishers_mutex_;
  std::mutex subscribers_mutex_;
  std::mutex clients_mutex_;
  std::mutex servers_mutex_;
  std::mutex timers_mutex_;
};

extern ManagerNode* g_manager_node_ptr;
template <class T>
HJPublisher HJAdvertise(const std::string& topic, uint32_t queue_size) {
  //
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->publishers_mutex_);
  auto it = g_manager_node_ptr->publishers_.find(topic);
  if (it == g_manager_node_ptr->publishers_.end()) {
    g_manager_node_ptr->publishers_[topic] =
        std::make_shared<ros::Publisher>(g_manager_node_ptr->node_handle_ptr_->advertise<T>(topic, queue_size));
    return HJPublisher(topic, g_manager_node_ptr->publishers_[topic]);
  }

  std::cerr << "Registering publishers with the same name is not allowed!" << std::endl;
  throw std::runtime_error("Registering publishers with the same name is not allowed!");
  //  return
}

template <class M>
HJSubscriber HJSubscribe(const std::string& topic, uint32_t queue_size, void (*fp)(M)) {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->subscribers_mutex_);
  auto it = g_manager_node_ptr->subscribers_.find(topic);
  if (it == g_manager_node_ptr->subscribers_.end()) {
    std::map<uint32_t, std::shared_ptr<ros::Subscriber>> temp;
    temp[g_manager_node_ptr->subscribers_index_] =
        std::make_shared<ros::Subscriber>(g_manager_node_ptr->node_handle_ptr_->subscribe(topic, queue_size, fp));
    g_manager_node_ptr->subscribers_[topic] = temp;
  } else {
    (it->second)[g_manager_node_ptr->subscribers_index_] =
        std::make_shared<ros::Subscriber>(g_manager_node_ptr->node_handle_ptr_->subscribe(topic, queue_size, fp));
  }
  g_manager_node_ptr->subscribers_index_++;
  return HJSubscriber(topic, g_manager_node_ptr->subscribers_[topic][g_manager_node_ptr->subscribers_index_ - 1],
                      g_manager_node_ptr->subscribers_index_ - 1);
}

template <class C>
HJSubscriber HJSubscribe(const std::string& topic, uint32_t queue_size, const boost::function<void(C)>& callback) {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->subscribers_mutex_);
  auto it = g_manager_node_ptr->subscribers_.find(topic);
  if (it == g_manager_node_ptr->subscribers_.end()) {
    std::map<uint32_t, std::shared_ptr<ros::Subscriber>> temp;
    temp[g_manager_node_ptr->subscribers_index_] =
        std::make_shared<ros::Subscriber>(g_manager_node_ptr->node_handle_ptr_->subscribe(topic, queue_size, callback));
    g_manager_node_ptr->subscribers_[topic] = temp;
  } else {
    (it->second)[g_manager_node_ptr->subscribers_index_] =
        std::make_shared<ros::Subscriber>(g_manager_node_ptr->node_handle_ptr_->subscribe(topic, queue_size, callback));
  }
  g_manager_node_ptr->subscribers_index_++;
  return HJSubscriber(topic, g_manager_node_ptr->subscribers_[topic][g_manager_node_ptr->subscribers_index_ - 1],
                      g_manager_node_ptr->subscribers_index_ - 1);
}

template <class M, class T>
HJSubscriber HJSubscribe(const std::string& topic, uint32_t queue_size, void (T::*fp)(M), T* obj) {
  boost::function<void(M)> sub_callback = boost::bind(fp, obj, boost::placeholders::_1);
  return HJSubscribe(topic, queue_size, sub_callback);
}
/**/
template <class T>
HJClient HJCreateClient(const std::string& topic) {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->clients_mutex_);
  auto it = g_manager_node_ptr->clients_.find(topic);
  if (it == g_manager_node_ptr->clients_.end()) {
    std::map<uint32_t, std::shared_ptr<ros::ServiceClient>> temp;
    temp[g_manager_node_ptr->clients_index_] =
        std::make_shared<ros::ServiceClient>(g_manager_node_ptr->node_handle_ptr_->serviceClient<T>(topic));
    g_manager_node_ptr->clients_[topic] = temp;
  } else {
    (it->second)[g_manager_node_ptr->clients_index_] =
        std::make_shared<ros::ServiceClient>(g_manager_node_ptr->node_handle_ptr_->serviceClient<T>(topic));
  }
  g_manager_node_ptr->clients_index_++;
  return HJClient(topic, g_manager_node_ptr->clients_[topic][g_manager_node_ptr->clients_index_ - 1],
                  g_manager_node_ptr->clients_index_ - 1);
}
template <class MReq, class MRes>
HJServer HJCreateServer(const std::string& service, bool (*srv_func)(MReq&, MRes&)) {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->servers_mutex_);
  auto it = g_manager_node_ptr->servers_.find(service);
  if (it == g_manager_node_ptr->servers_.end()) {
    g_manager_node_ptr->servers_[service] =
        std::make_shared<ros::ServiceServer>(g_manager_node_ptr->node_handle_ptr_->advertiseService(service, srv_func));
    return HJServer(service, g_manager_node_ptr->servers_[service]);
  }

  std::cerr << "Registering server with the same name is not allowed!" << std::endl;
  throw std::runtime_error("Registering server with the same name is not allowed!");
  //  return
}

template <class MReq, class MRes>
HJServer HJCreateServer(const std::string& service, const boost::function<bool(MReq&, MRes&)>& callback) {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->servers_mutex_);
  auto it = g_manager_node_ptr->servers_.find(service);
  if (it == g_manager_node_ptr->servers_.end()) {
    g_manager_node_ptr->servers_[service] =
        std::make_shared<ros::ServiceServer>(g_manager_node_ptr->node_handle_ptr_->advertiseService(service, callback));
    return HJServer(service, g_manager_node_ptr->servers_[service]);
  }

  std::cerr << "Registering server with the same name is not allowed!" << std::endl;
  throw std::runtime_error("Registering server with the same name is not allowed!");
  //  return
}


template <class MReq, class MRes, class T>
HJServer HJCreateServer(const std::string& service, bool (T::*srv_func)(MReq&, MRes&), T* obj) {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->servers_mutex_);
  auto it = g_manager_node_ptr->servers_.find(service);
  if (it == g_manager_node_ptr->servers_.end()) {
    g_manager_node_ptr->servers_[service] =
        std::make_shared<ros::ServiceServer>(g_manager_node_ptr->node_handle_ptr_->advertiseService(service, srv_func, obj));
    return HJServer(service, g_manager_node_ptr->servers_[service]);
  }

  std::cerr << "Registering server with the same name is not allowed!" << std::endl;
  throw std::runtime_error("Registering server with the same name is not allowed!");
  //  return
}

template <class T>
HJTimer HJCreateTimer(const std::string name, double us, void (T::*callback)(const HJTimerEvent&), T* obj) {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->timers_mutex_);
  auto it = g_manager_node_ptr->timers_.find(name);
  if (it == g_manager_node_ptr->timers_.end()) {
    g_manager_node_ptr->timers_[name] = std::make_shared<ros::Timer>(
        g_manager_node_ptr->node_handle_ptr_->createTimer(ros::Duration(us / 1000000), callback, obj));
    return HJTimer(us, g_manager_node_ptr->timers_[name], name);
  }
  std::cerr << "Registering timer with the same name is not allowed!" << std::endl;
  throw std::runtime_error("Registering timer with the same name is not allowed!");
}
HJTimer HJCreateTimer(const std::string name, double us, const HJTimerCallback& callback);
/**/

template <typename T>
bool HJGetParam(const std::string& key, T& param_val) {
  return g_manager_node_ptr->node_handle_ptr_->getParam(key, param_val);
}
template <typename T>
void HJSetParam(const std::string& key, T& param_val) {
  g_manager_node_ptr->node_handle_ptr_->setParam(key, param_val);
}
void nodeInit(int argc, char** argv, const std::string& node_name, uint32_t thread_count, uint32_t ops);
void getConfigure(const std::string& file, const std::string& so_path);
void nodeStart();
void nodeSpin();
ros::NodeHandle& getHandle();
typedef std::function<void(int)> FdCallBack;
void registerFdCtrl(int fd, FdCallBack callback);

uint32_t getNumPublisher();
uint32_t getNumSubscriber(const std::string topic);
uint32_t getAllSubscriber();

uint32_t getNumServer();
uint32_t getNumClient(const std::string topic);
uint32_t getAllClient();

uint32_t getNumTimer();
typedef ros::Time HJTime;
}  // namespace hj_bf

#endif
