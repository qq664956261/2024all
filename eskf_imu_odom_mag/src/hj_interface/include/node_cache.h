#ifndef SRC_BASE_FRAMEWORK_INCLUDE_NODE_CACHE_H_
#define SRC_BASE_FRAMEWORK_INCLUDE_NODE_CACHE_H_


#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include "node_factory.h"
#include "hj_interface/HealthCheckCode.h"
#define  MAX_NUM 100
#define HJ_HEALTH_MONITOR  "/hj_health_monitor"


namespace hj_bf {

typedef boost::function<bool(hj_interface::HealthCheckCodeRequest &req,
                          hj_interface::HealthCheckCodeResponse &res)> srvCallBack;

/**
 * @brief 客户端向cache添加数据, 添加一条会notify client
 * 
 * @param srv 
 */
void HjPushSrv(const hj_interface::HealthCheckCode& srv);

/**
 * 健康检测初始化
*/
void HealthCheckInit();

/**
 * set cache成员变量service_
*/
void HjSetService();

/**
 * 服务端注册callback
 * param : boost::function<>()
*/
void registerServerCallback(const srvCallBack& callback);

class DequeCache {
 public:
  static void createInstance(uint32_t max_size);

  friend void HjSendSrv();
  friend void HjPushSrv(const hj_interface::HealthCheckCode& srv);
  friend void HjSetClient(const hj_bf::HJClient& client);
  friend void HjSetService();
  friend void registerServerCallback(const srvCallBack& callback);

  void push(const hj_interface::HealthCheckCode& val);
  bool get_front(hj_interface::HealthCheckCode* val);
  void pop();
  void send_msg();
  void setClient(const hj_bf::HJClient& client);
  void setService();
  bool server_callback(hj_interface::HealthCheckCodeRequest& req,
                       hj_interface::HealthCheckCodeResponse& res);
  ~DequeCache() {}

 private:
  explicit DequeCache(int max_size): max_size_(max_size) {}
  int max_size_;
  hj_bf::HJClient client_;
  hj_bf::HJServer service_;
  std::vector<srvCallBack> callbacks_;
  std::deque<hj_interface::HealthCheckCode> cache_;
};

}  // namespace hj_bf

#endif  // SRC_BASE_FRAMEWORK_INCLUDE_NODE_CACHE_H_
