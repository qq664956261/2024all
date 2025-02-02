// Generated by gencpp from file hj_interface/PeriodicWeekdayTask.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_PERIODICWEEKDAYTASK_H
#define HJ_INTERFACE_MESSAGE_PERIODICWEEKDAYTASK_H


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
struct PeriodicWeekdayTask_
{
  typedef PeriodicWeekdayTask_<ContainerAllocator> Type;

  PeriodicWeekdayTask_()
    : name_id()
    , weekday(0)
    , start_time()
    , taskExecute(0)
    , smart(0)
    , clean_areas()
    , clean_mode(0)  {
    }
  PeriodicWeekdayTask_(const ContainerAllocator& _alloc)
    : name_id(_alloc)
    , weekday(0)
    , start_time(_alloc)
    , taskExecute(0)
    , smart(0)
    , clean_areas(_alloc)
    , clean_mode(0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _name_id_type;
  _name_id_type name_id;

   typedef int32_t _weekday_type;
  _weekday_type weekday;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _start_time_type;
  _start_time_type start_time;

   typedef int32_t _taskExecute_type;
  _taskExecute_type taskExecute;

   typedef int32_t _smart_type;
  _smart_type smart;

   typedef std::vector< ::hj_interface::CleanAreas_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::hj_interface::CleanAreas_<ContainerAllocator> >> _clean_areas_type;
  _clean_areas_type clean_areas;

   typedef int32_t _clean_mode_type;
  _clean_mode_type clean_mode;





  typedef boost::shared_ptr< ::hj_interface::PeriodicWeekdayTask_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::PeriodicWeekdayTask_<ContainerAllocator> const> ConstPtr;

}; // struct PeriodicWeekdayTask_

typedef ::hj_interface::PeriodicWeekdayTask_<std::allocator<void> > PeriodicWeekdayTask;

typedef boost::shared_ptr< ::hj_interface::PeriodicWeekdayTask > PeriodicWeekdayTaskPtr;
typedef boost::shared_ptr< ::hj_interface::PeriodicWeekdayTask const> PeriodicWeekdayTaskConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::PeriodicWeekdayTask_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::PeriodicWeekdayTask_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::PeriodicWeekdayTask_<ContainerAllocator1> & lhs, const ::hj_interface::PeriodicWeekdayTask_<ContainerAllocator2> & rhs)
{
  return lhs.name_id == rhs.name_id &&
    lhs.weekday == rhs.weekday &&
    lhs.start_time == rhs.start_time &&
    lhs.taskExecute == rhs.taskExecute &&
    lhs.smart == rhs.smart &&
    lhs.clean_areas == rhs.clean_areas &&
    lhs.clean_mode == rhs.clean_mode;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::PeriodicWeekdayTask_<ContainerAllocator1> & lhs, const ::hj_interface::PeriodicWeekdayTask_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::PeriodicWeekdayTask_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::PeriodicWeekdayTask_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::PeriodicWeekdayTask_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::PeriodicWeekdayTask_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::PeriodicWeekdayTask_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::PeriodicWeekdayTask_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::PeriodicWeekdayTask_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7e72fd9ae92674838365cc432d3b0b3c";
  }

  static const char* value(const ::hj_interface::PeriodicWeekdayTask_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7e72fd9ae9267483ULL;
  static const uint64_t static_value2 = 0x8365cc432d3b0b3cULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::PeriodicWeekdayTask_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/PeriodicWeekdayTask";
  }

  static const char* value(const ::hj_interface::PeriodicWeekdayTask_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::PeriodicWeekdayTask_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string name_id\n"
"int32 weekday\n"
"string start_time\n"
"int32 taskExecute  #表示当天任务是否生效 0-不生效 1-生效\n"
"int32 smart        # 0-普通模式 1-智能模式\n"
"CleanAreas[] clean_areas\n"
"int32 clean_mode   # 1-变频 2-标准 3-深度\n"
"================================================================================\n"
"MSG: hj_interface/CleanAreas\n"
"int32 clean_area  #清扫区域 1-水面 2-池底 3-池壁\n"
"int32 count       #清扫次数\n"
"int32 time        #清扫时间单位分钟\n"
;
  }

  static const char* value(const ::hj_interface::PeriodicWeekdayTask_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::PeriodicWeekdayTask_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name_id);
      stream.next(m.weekday);
      stream.next(m.start_time);
      stream.next(m.taskExecute);
      stream.next(m.smart);
      stream.next(m.clean_areas);
      stream.next(m.clean_mode);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PeriodicWeekdayTask_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::PeriodicWeekdayTask_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::PeriodicWeekdayTask_<ContainerAllocator>& v)
  {
    s << indent << "name_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.name_id);
    s << indent << "weekday: ";
    Printer<int32_t>::stream(s, indent + "  ", v.weekday);
    s << indent << "start_time: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.start_time);
    s << indent << "taskExecute: ";
    Printer<int32_t>::stream(s, indent + "  ", v.taskExecute);
    s << indent << "smart: ";
    Printer<int32_t>::stream(s, indent + "  ", v.smart);
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

#endif // HJ_INTERFACE_MESSAGE_PERIODICWEEKDAYTASK_H
