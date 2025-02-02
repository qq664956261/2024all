// Generated by gencpp from file hj_interface/IotShadowResponse.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_IOTSHADOWRESPONSE_H
#define HJ_INTERFACE_MESSAGE_IOTSHADOWRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <hj_interface/AppData.h>
#include <hj_interface/AppData.h>
#include <hj_interface/IotReturn.h>

namespace hj_interface
{
template <class ContainerAllocator>
struct IotShadowResponse_
{
  typedef IotShadowResponse_<ContainerAllocator> Type;

  IotShadowResponse_()
    : desired()
    , metadata()
    , iotret()
    , timestamp(0)  {
    }
  IotShadowResponse_(const ContainerAllocator& _alloc)
    : desired(_alloc)
    , metadata(_alloc)
    , iotret(_alloc)
    , timestamp(0)  {
  (void)_alloc;
    }



   typedef std::vector< ::hj_interface::AppData_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::hj_interface::AppData_<ContainerAllocator> >> _desired_type;
  _desired_type desired;

   typedef std::vector< ::hj_interface::AppData_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::hj_interface::AppData_<ContainerAllocator> >> _metadata_type;
  _metadata_type metadata;

   typedef  ::hj_interface::IotReturn_<ContainerAllocator>  _iotret_type;
  _iotret_type iotret;

   typedef uint64_t _timestamp_type;
  _timestamp_type timestamp;





  typedef boost::shared_ptr< ::hj_interface::IotShadowResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::IotShadowResponse_<ContainerAllocator> const> ConstPtr;

}; // struct IotShadowResponse_

typedef ::hj_interface::IotShadowResponse_<std::allocator<void> > IotShadowResponse;

typedef boost::shared_ptr< ::hj_interface::IotShadowResponse > IotShadowResponsePtr;
typedef boost::shared_ptr< ::hj_interface::IotShadowResponse const> IotShadowResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::IotShadowResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::IotShadowResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::IotShadowResponse_<ContainerAllocator1> & lhs, const ::hj_interface::IotShadowResponse_<ContainerAllocator2> & rhs)
{
  return lhs.desired == rhs.desired &&
    lhs.metadata == rhs.metadata &&
    lhs.iotret == rhs.iotret &&
    lhs.timestamp == rhs.timestamp;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::IotShadowResponse_<ContainerAllocator1> & lhs, const ::hj_interface::IotShadowResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::IotShadowResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::IotShadowResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::IotShadowResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::IotShadowResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::IotShadowResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::IotShadowResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::IotShadowResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d48edf896f9dc7bcd7c37d637b5c70c3";
  }

  static const char* value(const ::hj_interface::IotShadowResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd48edf896f9dc7bcULL;
  static const uint64_t static_value2 = 0xd7c37d637b5c70c3ULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::IotShadowResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/IotShadowResponse";
  }

  static const char* value(const ::hj_interface::IotShadowResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::IotShadowResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"hj_interface/AppData[]  desired       		# shadow doc desired json string\n"
"hj_interface/AppData[]  metadata      		# shadow doc meta json string\n"
"hj_interface/IotReturn iotret   	        # iot response from AWS\n"
"uint64  timestamp     			        # the Epoch date and time the response was generated by AWS IoT\n"
"\n"
"\n"
"================================================================================\n"
"MSG: hj_interface/AppData\n"
"string key\n"
"string payload\n"
"int8  res    	#response code\n"
"\n"
"\n"
"================================================================================\n"
"MSG: hj_interface/IotReturn\n"
"int64  code\n"
"string msg\n"
;
  }

  static const char* value(const ::hj_interface::IotShadowResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::IotShadowResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.desired);
      stream.next(m.metadata);
      stream.next(m.iotret);
      stream.next(m.timestamp);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct IotShadowResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::IotShadowResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::IotShadowResponse_<ContainerAllocator>& v)
  {
    s << indent << "desired[]" << std::endl;
    for (size_t i = 0; i < v.desired.size(); ++i)
    {
      s << indent << "  desired[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::hj_interface::AppData_<ContainerAllocator> >::stream(s, indent + "    ", v.desired[i]);
    }
    s << indent << "metadata[]" << std::endl;
    for (size_t i = 0; i < v.metadata.size(); ++i)
    {
      s << indent << "  metadata[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::hj_interface::AppData_<ContainerAllocator> >::stream(s, indent + "    ", v.metadata[i]);
    }
    s << indent << "iotret: ";
    s << std::endl;
    Printer< ::hj_interface::IotReturn_<ContainerAllocator> >::stream(s, indent + "  ", v.iotret);
    s << indent << "timestamp: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.timestamp);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_IOTSHADOWRESPONSE_H
