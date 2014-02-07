/* Auto-generated by genmsg_cpp for file /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/threemxl/msg/MultiArrayFloat32.msg */
#ifndef THREEMXL_MESSAGE_MULTIARRAYFLOAT32_H
#define THREEMXL_MESSAGE_MULTIARRAYFLOAT32_H
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


namespace threemxl
{
template <class ContainerAllocator>
struct MultiArrayFloat32_ {
  typedef MultiArrayFloat32_<ContainerAllocator> Type;

  MultiArrayFloat32_()
  : data()
  , mic1()
  , mic2()
  , mic3()
  , mic4()
  {
  }

  MultiArrayFloat32_(const ContainerAllocator& _alloc)
  : data()
  , mic1(_alloc)
  , mic2(_alloc)
  , mic3(_alloc)
  , mic4(_alloc)
  {
  }

  typedef ros::Time _data_type;
  ros::Time data;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _mic1_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  mic1;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _mic2_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  mic2;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _mic3_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  mic3;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _mic4_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  mic4;


  typedef boost::shared_ptr< ::threemxl::MultiArrayFloat32_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::threemxl::MultiArrayFloat32_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct MultiArrayFloat32
typedef  ::threemxl::MultiArrayFloat32_<std::allocator<void> > MultiArrayFloat32;

typedef boost::shared_ptr< ::threemxl::MultiArrayFloat32> MultiArrayFloat32Ptr;
typedef boost::shared_ptr< ::threemxl::MultiArrayFloat32 const> MultiArrayFloat32ConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::threemxl::MultiArrayFloat32_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::threemxl::MultiArrayFloat32_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace threemxl

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::threemxl::MultiArrayFloat32_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::threemxl::MultiArrayFloat32_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::threemxl::MultiArrayFloat32_<ContainerAllocator> > {
  static const char* value() 
  {
    return "20fc89e40d151d6a5828699657cffb3c";
  }

  static const char* value(const  ::threemxl::MultiArrayFloat32_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x20fc89e40d151d6aULL;
  static const uint64_t static_value2 = 0x5828699657cffb3cULL;
};

template<class ContainerAllocator>
struct DataType< ::threemxl::MultiArrayFloat32_<ContainerAllocator> > {
  static const char* value() 
  {
    return "threemxl/MultiArrayFloat32";
  }

  static const char* value(const  ::threemxl::MultiArrayFloat32_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::threemxl::MultiArrayFloat32_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# Please look at the MultiArrayLayout message definition for\n\
# documentation on all multiarrays.\n\
\n\
time		  data\n\
float32[]         mic1          # array of mic1\n\
float32[]         mic2          # array of mic2\n\
float32[]         mic3          # array of mic3\n\
float32[]         mic4          # array of mic4\n\
";
  }

  static const char* value(const  ::threemxl::MultiArrayFloat32_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::threemxl::MultiArrayFloat32_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.data);
    stream.next(m.mic1);
    stream.next(m.mic2);
    stream.next(m.mic3);
    stream.next(m.mic4);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct MultiArrayFloat32_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::threemxl::MultiArrayFloat32_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::threemxl::MultiArrayFloat32_<ContainerAllocator> & v) 
  {
    s << indent << "data: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.data);
    s << indent << "mic1[]" << std::endl;
    for (size_t i = 0; i < v.mic1.size(); ++i)
    {
      s << indent << "  mic1[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.mic1[i]);
    }
    s << indent << "mic2[]" << std::endl;
    for (size_t i = 0; i < v.mic2.size(); ++i)
    {
      s << indent << "  mic2[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.mic2[i]);
    }
    s << indent << "mic3[]" << std::endl;
    for (size_t i = 0; i < v.mic3.size(); ++i)
    {
      s << indent << "  mic3[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.mic3[i]);
    }
    s << indent << "mic4[]" << std::endl;
    for (size_t i = 0; i < v.mic4.size(); ++i)
    {
      s << indent << "  mic4[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.mic4[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // THREEMXL_MESSAGE_MULTIARRAYFLOAT32_H

