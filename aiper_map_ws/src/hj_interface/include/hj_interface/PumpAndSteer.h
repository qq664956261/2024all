// Generated by gencpp from file hj_interface/PumpAndSteer.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_PUMPANDSTEER_H
#define HJ_INTERFACE_MESSAGE_PUMPANDSTEER_H


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
struct PumpAndSteer_
{
  typedef PumpAndSteer_<ContainerAllocator> Type;

  PumpAndSteer_()
    : timestamp()
    , turn_motor_ctl(0)
    , turn_motor(0)
    , pump_ctl(0)
    , pump_speed_l(0)
    , pump_speed_r(0)  {
    }
  PumpAndSteer_(const ContainerAllocator& _alloc)
    : timestamp()
    , turn_motor_ctl(0)
    , turn_motor(0)
    , pump_ctl(0)
    , pump_speed_l(0)
    , pump_speed_r(0)  {
  (void)_alloc;
    }



   typedef ros::Time _timestamp_type;
  _timestamp_type timestamp;

   typedef uint8_t _turn_motor_ctl_type;
  _turn_motor_ctl_type turn_motor_ctl;

   typedef int16_t _turn_motor_type;
  _turn_motor_type turn_motor;

   typedef uint8_t _pump_ctl_type;
  _pump_ctl_type pump_ctl;

   typedef uint8_t _pump_speed_l_type;
  _pump_speed_l_type pump_speed_l;

   typedef uint8_t _pump_speed_r_type;
  _pump_speed_r_type pump_speed_r;





  typedef boost::shared_ptr< ::hj_interface::PumpAndSteer_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::PumpAndSteer_<ContainerAllocator> const> ConstPtr;

}; // struct PumpAndSteer_

typedef ::hj_interface::PumpAndSteer_<std::allocator<void> > PumpAndSteer;

typedef boost::shared_ptr< ::hj_interface::PumpAndSteer > PumpAndSteerPtr;
typedef boost::shared_ptr< ::hj_interface::PumpAndSteer const> PumpAndSteerConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::PumpAndSteer_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::PumpAndSteer_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::PumpAndSteer_<ContainerAllocator1> & lhs, const ::hj_interface::PumpAndSteer_<ContainerAllocator2> & rhs)
{
  return lhs.timestamp == rhs.timestamp &&
    lhs.turn_motor_ctl == rhs.turn_motor_ctl &&
    lhs.turn_motor == rhs.turn_motor &&
    lhs.pump_ctl == rhs.pump_ctl &&
    lhs.pump_speed_l == rhs.pump_speed_l &&
    lhs.pump_speed_r == rhs.pump_speed_r;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::PumpAndSteer_<ContainerAllocator1> & lhs, const ::hj_interface::PumpAndSteer_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::PumpAndSteer_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::PumpAndSteer_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::PumpAndSteer_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::PumpAndSteer_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::PumpAndSteer_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::PumpAndSteer_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::PumpAndSteer_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d097c874865c9dec50e5a9f2ca806621";
  }

  static const char* value(const ::hj_interface::PumpAndSteer_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd097c874865c9decULL;
  static const uint64_t static_value2 = 0x50e5a9f2ca806621ULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::PumpAndSteer_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/PumpAndSteer";
  }

  static const char* value(const ::hj_interface::PumpAndSteer_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::PumpAndSteer_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time timestamp\n"
"uint8 turn_motor_ctl     #!< 0 刹车； 1 运动； 2 滑行 \n"
"int16 turn_motor        #!< 矢量碰口电机， 0，90，180>\n"
"\n"
"uint8 pump_ctl       #!< 控制状态：[0x00]关闭；[0x01]开环运行；[0x02]速度环运行   占空比 1% == 40的转速\n"
"uint8 pump_speed_l       #!< 占空比 0到100 水泵PWM控制 左边水泵\n"
"uint8 pump_speed_r      #!< 占空比 0到100 水泵PWM控制 右边水泵\n"
;
  }

  static const char* value(const ::hj_interface::PumpAndSteer_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::PumpAndSteer_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.timestamp);
      stream.next(m.turn_motor_ctl);
      stream.next(m.turn_motor);
      stream.next(m.pump_ctl);
      stream.next(m.pump_speed_l);
      stream.next(m.pump_speed_r);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PumpAndSteer_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::PumpAndSteer_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::PumpAndSteer_<ContainerAllocator>& v)
  {
    s << indent << "timestamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.timestamp);
    s << indent << "turn_motor_ctl: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.turn_motor_ctl);
    s << indent << "turn_motor: ";
    Printer<int16_t>::stream(s, indent + "  ", v.turn_motor);
    s << indent << "pump_ctl: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.pump_ctl);
    s << indent << "pump_speed_l: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.pump_speed_l);
    s << indent << "pump_speed_r: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.pump_speed_r);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_PUMPANDSTEER_H
