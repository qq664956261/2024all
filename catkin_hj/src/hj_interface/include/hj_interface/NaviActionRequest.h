// Generated by gencpp from file hj_interface/NaviActionRequest.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_NAVIACTIONREQUEST_H
#define HJ_INTERFACE_MESSAGE_NAVIACTIONREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace hj_interface
{
template <class ContainerAllocator>
struct NaviActionRequest_
{
  typedef NaviActionRequest_<ContainerAllocator> Type;

  NaviActionRequest_()
    : action_cmd(0)
    , clean_mode(0)  {
    }
  NaviActionRequest_(const ContainerAllocator& _alloc)
    : action_cmd(0)
    , clean_mode(0)  {
  (void)_alloc;
    }



   typedef uint8_t _action_cmd_type;
  _action_cmd_type action_cmd;

   typedef uint8_t _clean_mode_type;
  _clean_mode_type clean_mode;





  typedef boost::shared_ptr< ::hj_interface::NaviActionRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::NaviActionRequest_<ContainerAllocator> const> ConstPtr;

}; // struct NaviActionRequest_

typedef ::hj_interface::NaviActionRequest_<std::allocator<void> > NaviActionRequest;

typedef boost::shared_ptr< ::hj_interface::NaviActionRequest > NaviActionRequestPtr;
typedef boost::shared_ptr< ::hj_interface::NaviActionRequest const> NaviActionRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::NaviActionRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::NaviActionRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::NaviActionRequest_<ContainerAllocator1> & lhs, const ::hj_interface::NaviActionRequest_<ContainerAllocator2> & rhs)
{
  return lhs.action_cmd == rhs.action_cmd &&
    lhs.clean_mode == rhs.clean_mode;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::NaviActionRequest_<ContainerAllocator1> & lhs, const ::hj_interface::NaviActionRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::NaviActionRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::NaviActionRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::NaviActionRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::NaviActionRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::NaviActionRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::NaviActionRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::NaviActionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7c42549eae1b55ea53ee41ca3c40947a";
  }

  static const char* value(const ::hj_interface::NaviActionRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7c42549eae1b55eaULL;
  static const uint64_t static_value2 = 0x53ee41ca3c40947aULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::NaviActionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/NaviActionRequest";
  }

  static const char* value(const ::hj_interface::NaviActionRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::NaviActionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 action_cmd    # 11: 清扫水面 12: 池底清扫 13: 池壁清扫\n"
"                    # 19: 清洁模式\n"
"                    # 21: 召回 22: 回充\n"
"                    # 32: navi状态请求\n"
"                    # 111: 暂停清扫 112: 继续清扫 113: 停止清扫\n"
"                    # 123: 停止召回 124: 停止回充\n"
"                    \n"
"uint8 clean_mode    # 1: 变频清洁 2: 标准清洁 3: 深度清洁\n"
"\n"
;
  }

  static const char* value(const ::hj_interface::NaviActionRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::NaviActionRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.action_cmd);
      stream.next(m.clean_mode);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NaviActionRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::NaviActionRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::NaviActionRequest_<ContainerAllocator>& v)
  {
    s << indent << "action_cmd: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.action_cmd);
    s << indent << "clean_mode: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.clean_mode);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_NAVIACTIONREQUEST_H
