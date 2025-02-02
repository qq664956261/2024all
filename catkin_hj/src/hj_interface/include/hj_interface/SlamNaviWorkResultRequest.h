// Generated by gencpp from file hj_interface/SlamNaviWorkResultRequest.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_SLAMNAVIWORKRESULTREQUEST_H
#define HJ_INTERFACE_MESSAGE_SLAMNAVIWORKRESULTREQUEST_H


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
struct SlamNaviWorkResultRequest_
{
  typedef SlamNaviWorkResultRequest_<ContainerAllocator> Type;

  SlamNaviWorkResultRequest_()
    : action_cmd(0)
    , action_result(0)  {
    }
  SlamNaviWorkResultRequest_(const ContainerAllocator& _alloc)
    : action_cmd(0)
    , action_result(0)  {
  (void)_alloc;
    }



   typedef uint8_t _action_cmd_type;
  _action_cmd_type action_cmd;

   typedef uint8_t _action_result_type;
  _action_result_type action_result;





  typedef boost::shared_ptr< ::hj_interface::SlamNaviWorkResultRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::SlamNaviWorkResultRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SlamNaviWorkResultRequest_

typedef ::hj_interface::SlamNaviWorkResultRequest_<std::allocator<void> > SlamNaviWorkResultRequest;

typedef boost::shared_ptr< ::hj_interface::SlamNaviWorkResultRequest > SlamNaviWorkResultRequestPtr;
typedef boost::shared_ptr< ::hj_interface::SlamNaviWorkResultRequest const> SlamNaviWorkResultRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::SlamNaviWorkResultRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::SlamNaviWorkResultRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::SlamNaviWorkResultRequest_<ContainerAllocator1> & lhs, const ::hj_interface::SlamNaviWorkResultRequest_<ContainerAllocator2> & rhs)
{
  return lhs.action_cmd == rhs.action_cmd &&
    lhs.action_result == rhs.action_result;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::SlamNaviWorkResultRequest_<ContainerAllocator1> & lhs, const ::hj_interface::SlamNaviWorkResultRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::SlamNaviWorkResultRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::SlamNaviWorkResultRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::SlamNaviWorkResultRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::SlamNaviWorkResultRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::SlamNaviWorkResultRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::SlamNaviWorkResultRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::SlamNaviWorkResultRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6eb56731533b0dbee7979e5055514f36";
  }

  static const char* value(const ::hj_interface::SlamNaviWorkResultRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6eb56731533b0dbeULL;
  static const uint64_t static_value2 = 0xe7979e5055514f36ULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::SlamNaviWorkResultRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/SlamNaviWorkResultRequest";
  }

  static const char* value(const ::hj_interface::SlamNaviWorkResultRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::SlamNaviWorkResultRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 action_cmd        # 1: 建图 2: 重定位 \n"
"                        # 11: 清扫 \n"
"                        # 21: 召回 22: 回充\n"
"\n"
"uint8 action_result     # 1: slam建图成功 2: slam建图失败 \n"
"                        # 11: slam定位成功 12: slam定位失败 \n"
"                        # 21: navi清扫成功 22: navi清扫失败\n"
"                        # 41: navi召回成功 42: navi召回失败 \n"
"                        # 51: navi回充成功 52: navi回充失败\n"
"\n"
;
  }

  static const char* value(const ::hj_interface::SlamNaviWorkResultRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::SlamNaviWorkResultRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.action_cmd);
      stream.next(m.action_result);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SlamNaviWorkResultRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::SlamNaviWorkResultRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::SlamNaviWorkResultRequest_<ContainerAllocator>& v)
  {
    s << indent << "action_cmd: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.action_cmd);
    s << indent << "action_result: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.action_result);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_SLAMNAVIWORKRESULTREQUEST_H
