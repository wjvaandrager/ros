/* Auto-generated by genmsg_cpp for file /home/pc_willem/ros/basecontrol/msg/Array.msg */
#ifndef BASECONTROL_MESSAGE_ARRAY_H
#define BASECONTROL_MESSAGE_ARRAY_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace basecontrol
{
template <class ContainerAllocator>
struct Array_ {
  typedef Array_<ContainerAllocator> Type;

  Array_()
  : data()
  {
  }

  Array_(const ContainerAllocator& _alloc)
  : data(_alloc)
  {
  }

  typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _data_type;
  std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  data;


  typedef boost::shared_ptr< ::basecontrol::Array_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::basecontrol::Array_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Array
typedef  ::basecontrol::Array_<std::allocator<void> > Array;

typedef boost::shared_ptr< ::basecontrol::Array> ArrayPtr;
typedef boost::shared_ptr< ::basecontrol::Array const> ArrayConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::basecontrol::Array_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::basecontrol::Array_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace basecontrol

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::basecontrol::Array_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::basecontrol::Array_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::basecontrol::Array_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f43a8e1b362b75baa741461b46adc7e0";
  }

  static const char* value(const  ::basecontrol::Array_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xf43a8e1b362b75baULL;
  static const uint64_t static_value2 = 0xa741461b46adc7e0ULL;
};

template<class ContainerAllocator>
struct DataType< ::basecontrol::Array_<ContainerAllocator> > {
  static const char* value() 
  {
    return "basecontrol/Array";
  }

  static const char* value(const  ::basecontrol::Array_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::basecontrol::Array_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint8[] data\n\
";
  }

  static const char* value(const  ::basecontrol::Array_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::basecontrol::Array_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.data);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Array_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::basecontrol::Array_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::basecontrol::Array_<ContainerAllocator> & v) 
  {
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.data[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // BASECONTROL_MESSAGE_ARRAY_H
