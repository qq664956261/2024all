// Generated by gencpp from file hj_interface/X6Pump.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_X6PUMP_H
#define HJ_INTERFACE_MESSAGE_X6PUMP_H


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
struct X6Pump_
{
  typedef X6Pump_<ContainerAllocator> Type;

  X6Pump_()
    : timestamp()
    , pump_ctl_l(0)
    , pump_speed_l(0)
    , pump_ctl_r(0)
    , pump_speed_r(0)  {
    }
  X6Pump_(const ContainerAllocator& _alloc)
    : timestamp()
    , pump_ctl_l(0)
    , pump_speed_l(0)
    , pump_ctl_r(0)
    , pump_speed_r(0)  {
  (void)_alloc;
    }



   typedef ros::Time _timestamp_type;
  _timestamp_type timestamp;

   typedef uint8_t _pump_ctl_l_type;
  _pump_ctl_l_type pump_ctl_l;

   typedef uint8_t _pump_speed_l_type;
  _pump_speed_l_type pump_speed_l;

   typedef uint8_t _pump_ctl_r_type;
  _pump_ctl_r_type pump_ctl_r;

   typedef uint8_t _pump_speed_r_type;
  _pump_speed_r_type pump_speed_r;





  typedef boost::shared_ptr< ::hj_interface::X6Pump_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::X6Pump_<ContainerAllocator> const> ConstPtr;

}; // struct X6Pump_

typedef ::hj_interface::X6Pump_<std::allocator<void> > X6Pump;

typedef boost::shared_ptr< ::hj_interface::X6Pump > X6PumpPtr;
typedef boost::shared_ptr< ::hj_interface::X6Pump const> X6PumpConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::X6Pump_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::X6Pump_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::X6Pump_<ContainerAllocator1> & lhs, const ::hj_interface::X6Pump_<ContainerAllocator2> & rhs)
{
  return lhs.timestamp == rhs.timestamp &&
    lhs.pump_ctl_l == rhs.pump_ctl_l &&
    lhs.pump_speed_l == rhs.pump_speed_l &&
    lhs.pump_ctl_r == rhs.pump_ctl_r &&
    lhs.pump_speed_r == rhs.pump_speed_r;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::X6Pump_<ContainerAllocator1> & lhs, const ::hj_interface::X6Pump_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::X6Pump_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::X6Pump_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::X6Pump_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::X6Pump_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::X6Pump_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::X6Pump_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::X6Pump_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5e2db79fc592cb10d575a4498590c4ac";
  }

  static const char* value(const ::hj_interface::X6Pump_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5e2db79fc592cb10ULL;
  static const uint64_t static_value2 = 0xd575a4498590c4acULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::X6Pump_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/X6Pump";
  }

  static const char* value(const ::hj_interface::X6Pump_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::X6Pump_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time timestamp\n"
"\n"
"uint8 pump_ctl_l       #!< 0 停止； 1 运行 水泵控制   左侧       \n"
"uint8 pump_speed_l       #!< 占空比 0到100 水泵PWM控制   左侧\n"
"\n"
"uint8 pump_ctl_r       #!< 0 停止； 1 运行 水泵控制  右侧        \n"
"uint8 pump_speed_r       #!< 占空比 0到100 水泵PWM控制  右侧\n"
;
  }

  static const char* value(const ::hj_interface::X6Pump_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::X6Pump_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.timestamp);
      stream.next(m.pump_ctl_l);
      stream.next(m.pump_speed_l);
      stream.next(m.pump_ctl_r);
      stream.next(m.pump_speed_r);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct X6Pump_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::X6Pump_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::X6Pump_<ContainerAllocator>& v)
  {
    s << indent << "timestamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.timestamp);
    s << indent << "pump_ctl_l: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.pump_ctl_l);
    s << indent << "pump_speed_l: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.pump_speed_l);
    s << indent << "pump_ctl_r: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.pump_ctl_r);
    s << indent << "pump_speed_r: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.pump_speed_r);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_X6PUMP_H
