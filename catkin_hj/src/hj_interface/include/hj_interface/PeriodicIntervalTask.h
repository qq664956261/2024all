// Generated by gencpp from file hj_interface/PeriodicIntervalTask.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_PERIODICINTERVALTASK_H
#define HJ_INTERFACE_MESSAGE_PERIODICINTERVALTASK_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <hj_interface/CleanAreas.h>

namespace hj_interface
{
template <class ContainerAllocator>
struct PeriodicIntervalTask_
{
  typedef PeriodicIntervalTask_<ContainerAllocator> Type;

  PeriodicIntervalTask_()
    : name_id()
    , date()
    , start_time()
    , interval_days(0)
    , clean_areas()
    , clean_mode(0)  {
    }
  PeriodicIntervalTask_(const ContainerAllocator& _alloc)
    : name_id(_alloc)
    , date(_alloc)
    , start_time(_alloc)
    , interval_days(0)
    , clean_areas(_alloc)
    , clean_mode(0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _name_id_type;
  _name_id_type name_id;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _date_type;
  _date_type date;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _start_time_type;
  _start_time_type start_time;

   typedef int32_t _interval_days_type;
  _interval_days_type interval_days;

   typedef std::vector< ::hj_interface::CleanAreas_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::hj_interface::CleanAreas_<ContainerAllocator> >> _clean_areas_type;
  _clean_areas_type clean_areas;

   typedef int32_t _clean_mode_type;
  _clean_mode_type clean_mode;





  typedef boost::shared_ptr< ::hj_interface::PeriodicIntervalTask_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::PeriodicIntervalTask_<ContainerAllocator> const> ConstPtr;

}; // struct PeriodicIntervalTask_

typedef ::hj_interface::PeriodicIntervalTask_<std::allocator<void> > PeriodicIntervalTask;

typedef boost::shared_ptr< ::hj_interface::PeriodicIntervalTask > PeriodicIntervalTaskPtr;
typedef boost::shared_ptr< ::hj_interface::PeriodicIntervalTask const> PeriodicIntervalTaskConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::PeriodicIntervalTask_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::PeriodicIntervalTask_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::PeriodicIntervalTask_<ContainerAllocator1> & lhs, const ::hj_interface::PeriodicIntervalTask_<ContainerAllocator2> & rhs)
{
  return lhs.name_id == rhs.name_id &&
    lhs.date == rhs.date &&
    lhs.start_time == rhs.start_time &&
    lhs.interval_days == rhs.interval_days &&
    lhs.clean_areas == rhs.clean_areas &&
    lhs.clean_mode == rhs.clean_mode;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::PeriodicIntervalTask_<ContainerAllocator1> & lhs, const ::hj_interface::PeriodicIntervalTask_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::PeriodicIntervalTask_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::PeriodicIntervalTask_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::PeriodicIntervalTask_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::PeriodicIntervalTask_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::PeriodicIntervalTask_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::PeriodicIntervalTask_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::PeriodicIntervalTask_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1baf03ed10d0fc70b5b1affa93a375ee";
  }

  static const char* value(const ::hj_interface::PeriodicIntervalTask_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1baf03ed10d0fc70ULL;
  static const uint64_t static_value2 = 0xb5b1affa93a375eeULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::PeriodicIntervalTask_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/PeriodicIntervalTask";
  }

  static const char* value(const ::hj_interface::PeriodicIntervalTask_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::PeriodicIntervalTask_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string name_id\n"
"string date\n"
"string start_time\n"
"int32 interval_days\n"
"CleanAreas[] clean_areas\n"
"int32 clean_mode   # 1-变频 2-标准 3-深度\n"
"================================================================================\n"
"MSG: hj_interface/CleanAreas\n"
"int32 clean_area  #清扫区域 1-水面 2-池底 3-池壁\n"
"int32 count       #清扫次数\n"
"int32 time        #清扫时间单位分钟\n"
;
  }

  static const char* value(const ::hj_interface::PeriodicIntervalTask_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::PeriodicIntervalTask_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name_id);
      stream.next(m.date);
      stream.next(m.start_time);
      stream.next(m.interval_days);
      stream.next(m.clean_areas);
      stream.next(m.clean_mode);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PeriodicIntervalTask_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::PeriodicIntervalTask_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::PeriodicIntervalTask_<ContainerAllocator>& v)
  {
    s << indent << "name_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.name_id);
    s << indent << "date: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.date);
    s << indent << "start_time: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.start_time);
    s << indent << "interval_days: ";
    Printer<int32_t>::stream(s, indent + "  ", v.interval_days);
    s << indent << "clean_areas[]" << std::endl;
    for (size_t i = 0; i < v.clean_areas.size(); ++i)
    {
      s << indent << "  clean_areas[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::hj_interface::CleanAreas_<ContainerAllocator> >::stream(s, indent + "    ", v.clean_areas[i]);
    }
    s << indent << "clean_mode: ";
    Printer<int32_t>::stream(s, indent + "  ", v.clean_mode);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_PERIODICINTERVALTASK_H
