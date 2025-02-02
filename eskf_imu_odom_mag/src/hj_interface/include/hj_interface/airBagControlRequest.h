// Generated by gencpp from file hj_interface/airBagControlRequest.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_AIRBAGCONTROLREQUEST_H
#define HJ_INTERFACE_MESSAGE_AIRBAGCONTROLREQUEST_H


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
struct airBagControlRequest_
{
  typedef airBagControlRequest_<ContainerAllocator> Type;

  airBagControlRequest_()
    : type(0)
    , stime(0)  {
    }
  airBagControlRequest_(const ContainerAllocator& _alloc)
    : type(0)
    , stime(0)  {
  (void)_alloc;
    }



   typedef uint8_t _type_type;
  _type_type type;

   typedef uint16_t _stime_type;
  _stime_type stime;





  typedef boost::shared_ptr< ::hj_interface::airBagControlRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::airBagControlRequest_<ContainerAllocator> const> ConstPtr;

}; // struct airBagControlRequest_

typedef ::hj_interface::airBagControlRequest_<std::allocator<void> > airBagControlRequest;

typedef boost::shared_ptr< ::hj_interface::airBagControlRequest > airBagControlRequestPtr;
typedef boost::shared_ptr< ::hj_interface::airBagControlRequest const> airBagControlRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::airBagControlRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::airBagControlRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::airBagControlRequest_<ContainerAllocator1> & lhs, const ::hj_interface::airBagControlRequest_<ContainerAllocator2> & rhs)
{
  return lhs.type == rhs.type &&
    lhs.stime == rhs.stime;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::airBagControlRequest_<ContainerAllocator1> & lhs, const ::hj_interface::airBagControlRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::airBagControlRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::airBagControlRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::airBagControlRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::airBagControlRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::airBagControlRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::airBagControlRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::airBagControlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "58a79381056366842d561fd009ba42bd";
  }

  static const char* value(const ::hj_interface::airBagControlRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x58a7938105636684ULL;
  static const uint64_t static_value2 = 0x2d561fd009ba42bdULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::airBagControlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/airBagControlRequest";
  }

  static const char* value(const ::hj_interface::airBagControlRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::airBagControlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8  type   # 气囊控制类型 0: 查询气囊控制状态 1:充气 2:放气\n"
"uint16 stime  # 充放气时间  \n"
;
  }

  static const char* value(const ::hj_interface::airBagControlRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::airBagControlRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.type);
      stream.next(m.stime);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct airBagControlRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::airBagControlRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::airBagControlRequest_<ContainerAllocator>& v)
  {
    s << indent << "type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type);
    s << indent << "stime: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.stime);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_AIRBAGCONTROLREQUEST_H
