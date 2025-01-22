#ifndef SRC_BASE_FRAMEWORK_INCLUDE_NODE_CACHE_H_
#define SRC_BASE_FRAMEWORK_INCLUDE_NODE_CACHE_H_


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <deque>


#include "node_factory.h"
#include "hj_interface/HealthCheckCode.h"
#define  MAX_NUM 100
#define HJ_HEALTH_MONITOR  "/hj_health_monitor"


namespace hj_bf {

typedef boost::function<bool(hj_interface::HealthCheckCodeRequest &req,
                          hj_interface::HealthCheckCodeResponse &res)> srvCallBack;

/**
 * 客户端向cache添加数据, 添加一条会notify client 执行send
 * param: 健康检查码信息
*/
void HjPushSrv(const hj_interface::HealthCheckCode& srv);

/**
 * 节点线程函数，用于srv send
*/
void HjSendSrv();

/**
 * cache 初始化
 * param : 队列最大容量
*/
void cacheInit(uint32_t max_size);

/**
 * set cache成员变量client_
*/
void HjSetClient(const hj_bf::HJClient& client);

/**
 * 服务端注册callback
 * param : boost::function<>()
*/
void registerServerCallback(const srvCallBack& callback);

class DequeCache {
 public:
  static void createInstance(uint32_t max_size);

  friend void cacheInit(uint32_t max_size);
  friend void HjSendSrv();
  friend void HjPushSrv(const hj_interface::HealthCheckCode& srv);
  friend void HjSetClient(const hj_bf::HJClient& client);

  void push(const hj_interface::HealthCheckCode& val);
  bool get_front(hj_interface::HealthCheckCode* val);
  void pop();
  void send_msg();
  void setClient(const hj_bf::HJClient& client);
  ~DequeCache() {}

 private:
  explicit DequeCache(int max_size): max_size_(max_size) {}
  int max_size_;
  hj_bf::HJClient client_;
  std::deque<hj_interface::HealthCheckCode> cache_;
};

}  // namespace hj_bf

#endif  // SRC_BASE_FRAMEWORK_INCLUDE_NODE_CACHE_H_
