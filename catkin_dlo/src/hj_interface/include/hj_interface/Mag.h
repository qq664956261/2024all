// Generated by gencpp from file hj_interface/Mag.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_MAG_H
#define HJ_INTERFACE_MESSAGE_MAG_H


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
struct Mag_
{
  typedef Mag_<ContainerAllocator> Type;

  Mag_()
    : custom_time()
    , mag_x(0)
    , mag_y(0)
    , mag_z(0)  {
    }
  Mag_(const ContainerAllocator& _alloc)
    : custom_time()
    , mag_x(0)
    , mag_y(0)
    , mag_z(0)  {
  (void)_alloc;
    }



   typedef ros::Time _custom_time_type;
  _custom_time_type custom_time;

   typedef int16_t _mag_x_type;
  _mag_x_type mag_x;

   typedef int16_t _mag_y_type;
  _mag_y_type mag_y;

   typedef int16_t _mag_z_type;
  _mag_z_type mag_z;





  typedef boost::shared_ptr< ::hj_interface::Mag_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::Mag_<ContainerAllocator> const> ConstPtr;

}; // struct Mag_

typedef ::hj_interface::Mag_<std::allocator<void> > Mag;

typedef boost::shared_ptr< ::hj_interface::Mag > MagPtr;
typedef boost::shared_ptr< ::hj_interface::Mag const> MagConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::Mag_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::Mag_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::Mag_<ContainerAllocator1> & lhs, const ::hj_interface::Mag_<ContainerAllocator2> & rhs)
{
  return lhs.custom_time == rhs.custom_time &&
    lhs.mag_x == rhs.mag_x &&
    lhs.mag_y == rhs.mag_y &&
    lhs.mag_z == rhs.mag_z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::Mag_<ContainerAllocator1> & lhs, const ::hj_interface::Mag_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::Mag_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::Mag_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::Mag_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::Mag_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::Mag_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::Mag_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::Mag_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5cab63a10c5d367061e41cd1b7497522";
  }

  static const char* value(const ::hj_interface::Mag_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5cab63a10c5d3670ULL;
  static const uint64_t static_value2 = 0x61e41cd1b7497522ULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::Mag_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/Mag";
  }

  static const char* value(const ::hj_interface::Mag_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::Mag_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time custom_time\n"
"int16 mag_x\n"
"int16 mag_y\n"
"int16 mag_z\n"
;
  }

  static const char* value(const ::hj_interface::Mag_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::Mag_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.custom_time);
      stream.next(m.mag_x);
      stream.next(m.mag_y);
      stream.next(m.mag_z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Mag_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::Mag_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::Mag_<ContainerAllocator>& v)
  {
    s << indent << "custom_time: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.custom_time);
    s << indent << "mag_x: ";
    Printer<int16_t>::stream(s, indent + "  ", v.mag_x);
    s << indent << "mag_y: ";
    Printer<int16_t>::stream(s, indent + "  ", v.mag_y);
    s << indent << "mag_z: ";
    Printer<int16_t>::stream(s, indent + "  ", v.mag_z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_MAG_H
