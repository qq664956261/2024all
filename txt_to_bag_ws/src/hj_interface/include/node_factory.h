// @file node_factory.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-15)
#ifndef INCLUDE_COMMUNICATION_H
#define INCLUDE_COMMUNICATION_H
#include <csignal>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include "function_factory.h"
#include "ros/ros.h"
#define BOOST_STACKTRACE_USE_ADDR2LINE
#include <boost/stacktrace.hpp>

namespace hj_bf {
  constexpr char g_node_config_file_env_set[] = "HJ_NODE_CONFIG_FILE";
struct NodeConfig {
  bool all_log_close;
  bool node_log_close;
  int node_threads;
  std::string log_config_path;
  std::string so_path;
  std::string value_str;
  std::string node_name;
  std::vector<unsigned char> crypt_val;
};
void readConfigure(const std::string &config_file_name, std::shared_ptr<struct NodeConfig> out_config) ;
class HJPublisher {
 public:
  HJPublisher(const std::string& topic, const std::shared_ptr<ros::Publisher>& pub, uint32_t index)
      : publisher_ptr_(pub), name_(topic), index_(index) {}
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
  uint32_t index_;
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
  ~HJSubscriber();
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
  bool exists(); 
  bool waitForExistence(const double us);
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
  bool hasPending();

 private:
  double duration_;
  std::shared_ptr<ros::Timer> timer_ptr_;
  std::string name_;
};

class HJSteadyTimer {
 public:
  HJSteadyTimer(const double us, const std::shared_ptr<ros::SteadyTimer>& timer, const std::string& name)
      : duration_(us), timer_ptr_(timer), name_(name) {}
  HJSteadyTimer() {}
  HJSteadyTimer(const HJSteadyTimer&) = default;
  HJSteadyTimer(HJSteadyTimer&&) = default;
  HJSteadyTimer& operator=(const HJSteadyTimer&) = default;
  HJSteadyTimer& operator=(HJSteadyTimer&&) = default;
  ~HJSteadyTimer();

  void shutdown() = delete;
  std::string getName() { return name_; }
  void start();
  void stop();
  void setPeriod(const double us);
  bool hasPending();

 private:
  double duration_;
  std::shared_ptr<ros::SteadyTimer> timer_ptr_;
  std::string name_;
};

class HighResolutionTimer {
 public:
  HighResolutionTimer() : running_(false), interval_(0), callback_(nullptr) {}
  ~HighResolutionTimer();
  void start(double interval, std::function<void()> callback);
  void stop();

 private:
  bool running_;
  double interval_;
  std::function<void()> callback_;
  std::thread timer_thread_;
};

typedef ros::TimerEvent HJTimerEvent;
typedef ros::SteadyTimerEvent HJSteadyTimerEvent;
typedef std::function<void(const HJTimerEvent&)> HJTimerCallback;
typedef std::function<void(const HJSteadyTimerEvent&)> HJSteadyTimerCallback;
template <class T>
HJPublisher HJAdvertise(const std::string& topic, uint32_t queue_size, bool latch = false);
template <class T>
HJTimer HJCreateTimer(const std::string name, double us, void (T::*callback)(const HJTimerEvent&), T* obj,
                      bool autostart = true);
template <class T>
HJSteadyTimer HJCreateSteadyTimer(const std::string name, double us, void (T::*callback)(const HJSteadyTimerEvent&), T* obj,
                      bool oneshot = false, bool autostart = true);

class ManagerNode {
 public:
  static void createInstance(const std::shared_ptr<struct NodeConfig>& in_config);
  friend void nodeInstance();
  friend void nodeInit(int argc, char** argv, const std::string& node_name, const std::shared_ptr<struct NodeConfig>& in_config, uint32_t ops);
  friend void nodeStart();
  friend void nodeSpin();
  friend void getConfigure(const std::string& file, const std::string& so_path);
  friend ros::NodeHandle& getHandle();

  template <class T>
  friend HJPublisher HJAdvertise(const std::string& topic, uint32_t queue_size, bool latch);
  template <class M>
  friend HJSubscriber HJSubscribe(const std::string& topic, uint32_t queue_size, void (*fp)(M));
  template <class C>
  friend HJSubscriber HJSubscribe(const std::string& topic, uint32_t queue_size,
                                  const boost::function<void(C)>& callback);
  template <class M, class T>
  friend HJSubscriber HJSubscribe(const std::string& topic, uint32_t queue_size, void (T::*fp)(M), T* obj);
  template <class T>
  friend HJClient HJCreateClient(const std::string& topic);

  template <class MReq, class MRes>
  friend HJServer HJCreateServer(const std::string& service, bool (*srv_func)(MReq&, MRes&));

  template <class MReq, class MRes>
  friend HJServer HJCreateServer(const std::string& service, const boost::function<bool(MReq&, MRes&)>& callback);

  template <class MReq, class MRes, class T>
  friend HJServer HJCreateServer(const std::string& service, bool (T::*srv_func)(MReq&, MRes&), T* obj);

  friend HJSteadyTimer HJCreateSteadyTimer(const std::string name, double us, const HJSteadyTimerCallback& callback,
                                            bool oneshot, bool autostart);
  template <class T>
  friend HJSteadyTimer HJCreateSteadyTimer(const std::string name, double us, void (T::*callback)(const HJSteadyTimerEvent&), T* obj,
                                           bool oneshot, bool autostart);
  friend HJTimer HJCreateTimer(const std::string name, double us, const HJTimerCallback& callback, bool autostart);
  template <class T>
  friend HJTimer HJCreateTimer(const std::string name, double us, void (T::*callback)(const HJTimerEvent&), T* obj,
                               bool autostart);
  template <typename T>
  friend bool HJGetParam(const std::string& key, T& param_val);
  template <typename T>
  friend void HJSetParam(const std::string& key, T& param_val);

  friend void deletePublisher(const std::string& publisher_name, uint32_t index);
  friend void deleteSubscriber(const std::string& subscriber_name, uint32_t index);
  friend void deleteClient(const std::string& service_name, uint32_t index);
  friend void deleteServer(const std::string& service_name);
  friend void deleteTimer(const std::string& timer_name);
  friend void deleteSteadyTimer(const std::string& timer_name);
  friend uint32_t getNumPublisher(const std::string topic);
  friend uint32_t getNumSubscriber(const std::string topic);
  friend uint32_t getAllPublisher();
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
  struct ManagerParam;

 private:
  explicit ManagerNode(const std::shared_ptr<struct NodeConfig>& in_config);
  HJPublisher CreatePublisher(const std::string& topic, const std::string& msg_type_name,
                              std::shared_ptr<ros::Publisher> publisher_ptr);
  HJSubscriber CreateSubscriber(const std::string& topic, std::shared_ptr<ros::Subscriber> subscriber_ptr);
  HJClient CreateClient(const std::string& topic, std::shared_ptr<ros::ServiceClient> client_ptr);
  HJServer CreateServer(const std::string& service, std::shared_ptr<ros::ServiceServer> server_ptr);
  std::unique_ptr<ManagerParam> manager_ptr_;
  std::shared_ptr<ros::NodeHandle> node_handle_ptr_;
  std::unordered_map<std::string, std::unique_ptr<Function>> hj_functions_;
  ros::AsyncSpinner spinner_;
  std::shared_ptr<struct NodeConfig> config_ptr_;
};

extern ManagerNode* g_manager_node_ptr;

template <class T>
HJPublisher HJAdvertise(const std::string& topic, uint32_t queue_size, bool latch) {
  std::string msg_type_name = typeid(T).name();
  return g_manager_node_ptr->CreatePublisher(
      topic, msg_type_name,
      std::make_shared<ros::Publisher>(g_manager_node_ptr->node_handle_ptr_->advertise<T>(topic, queue_size, latch)));
}

template <class M>
HJSubscriber HJSubscribe(const std::string& topic, uint32_t queue_size, void (*fp)(M)) {
  return g_manager_node_ptr->CreateSubscriber(
      topic, std::make_shared<ros::Subscriber>(g_manager_node_ptr->node_handle_ptr_->subscribe(topic, queue_size, fp)));
}

template <class C>
HJSubscriber HJSubscribe(const std::string& topic, uint32_t queue_size, const boost::function<void(C)>& callback) {
  return g_manager_node_ptr->CreateSubscriber(
      topic,
      std::make_shared<ros::Subscriber>(g_manager_node_ptr->node_handle_ptr_->subscribe(topic, queue_size, callback)));
}

template <class M, class T>
HJSubscriber HJSubscribe(const std::string& topic, uint32_t queue_size, void (T::*fp)(M), T* obj) {
  return g_manager_node_ptr->CreateSubscriber(
      topic,
      std::make_shared<ros::Subscriber>(g_manager_node_ptr->node_handle_ptr_->subscribe(topic, queue_size, fp, obj)));
}
/**/
template <class T>
HJClient HJCreateClient(const std::string& topic) {
  return g_manager_node_ptr->CreateClient(
      topic, std::make_shared<ros::ServiceClient>(g_manager_node_ptr->node_handle_ptr_->serviceClient<T>(topic)));
}

template <class MReq, class MRes>
HJServer HJCreateServer(const std::string& service, bool (*srv_func)(MReq&, MRes&)) {
  return g_manager_node_ptr->CreateServer(
      service,
      std::make_shared<ros::ServiceServer>(g_manager_node_ptr->node_handle_ptr_->advertiseService(service, srv_func)));
}

template <class MReq, class MRes>
HJServer HJCreateServer(const std::string& service, const boost::function<bool(MReq&, MRes&)>& callback) {
  return g_manager_node_ptr->CreateServer(
      service,
      std::make_shared<ros::ServiceServer>(g_manager_node_ptr->node_handle_ptr_->advertiseService(service, callback)));
}

template <class MReq, class MRes, class T>
HJServer HJCreateServer(const std::string& service, bool (T::*srv_func)(MReq&, MRes&), T* obj) {
  return g_manager_node_ptr->CreateServer(
      service, std::make_shared<ros::ServiceServer>(
                   g_manager_node_ptr->node_handle_ptr_->advertiseService(service, srv_func, obj)));
}

HJTimer HJCreateTimer(const std::string name, double us, const HJTimerCallback& callback, bool autostart = true);

template <class T>
HJTimer HJCreateTimer(const std::string name, double us, void (T::*callback)(const HJTimerEvent&), T* obj,
                      bool autostart) {
  HJTimerCallback callback_temp = std::bind(callback, obj, std::placeholders::_1);
  return HJCreateTimer(name, us, callback_temp, autostart);
}

HJSteadyTimer HJCreateSteadyTimer(const std::string name, double us, const HJSteadyTimerCallback& callback,
                              bool oneshot = false, bool autostart = true);

template <class T>
HJSteadyTimer HJCreateSteadyTimer(const std::string name, double us, void (T::*callback)(const HJSteadyTimerEvent&), T* obj,
                      bool oneshot, bool autostart) {
  HJSteadyTimerCallback callback_temp = std::bind(callback, obj, std::placeholders::_1);
  return HJCreateSteadyTimer(name, us, callback_temp, oneshot, autostart);
}

/**/

template <typename T>
bool HJGetParam(const std::string& key, T& param_val) {
  return g_manager_node_ptr->node_handle_ptr_->getParam(key, param_val);
}
template <typename T>
void HJSetParam(const std::string& key, T& param_val) {
  g_manager_node_ptr->node_handle_ptr_->setParam(key, param_val);
}
void nodeInit(int argc, char** argv, const std::string& node_name, const std::shared_ptr<struct NodeConfig>& in_config, uint32_t ops);
void nodeInstance();
void getConfigure(const std::string& file, const std::string& so_path);
void nodeStart();
void nodeSpin();
ros::NodeHandle& getHandle();
typedef std::function<void(int)> FdCallBack;
void registerFdCtrl(int fd, FdCallBack callback);

uint32_t getNumPublisher(const std::string topic);
uint32_t getAllPublisher();
uint32_t getNumSubscriber(const std::string topic);
uint32_t getAllSubscriber();

uint32_t getNumServer();
uint32_t getNumClient(const std::string topic);
uint32_t getAllClient();

uint32_t getNumTimer();
typedef ros::Time HJTime;

void handler(int signo);
void registerSignal();

}  // namespace hj_bf
#endif
