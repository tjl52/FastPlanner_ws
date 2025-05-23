// Generated by gencpp from file quadrotor_msgs/Serial.msg
// DO NOT EDIT!


#ifndef QUADROTOR_MSGS_MESSAGE_SERIAL_H
#define QUADROTOR_MSGS_MESSAGE_SERIAL_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace quadrotor_msgs
{
template <class ContainerAllocator>
struct Serial_
{
  typedef Serial_<ContainerAllocator> Type;

  Serial_()
    : header()
    , channel(0)
    , type(0)
    , data()  {
    }
  Serial_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , channel(0)
    , type(0)
    , data(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _channel_type;
  _channel_type channel;

   typedef uint8_t _type_type;
  _type_type type;

   typedef std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> _data_type;
  _data_type data;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(SO3_CMD)
  #undef SO3_CMD
#endif
#if defined(_WIN32) && defined(TRPY_CMD)
  #undef TRPY_CMD
#endif
#if defined(_WIN32) && defined(STATUS_DATA)
  #undef STATUS_DATA
#endif
#if defined(_WIN32) && defined(OUTPUT_DATA)
  #undef OUTPUT_DATA
#endif
#if defined(_WIN32) && defined(PPR_OUTPUT_DATA)
  #undef PPR_OUTPUT_DATA
#endif
#if defined(_WIN32) && defined(PPR_GAINS)
  #undef PPR_GAINS
#endif

  enum {
    SO3_CMD = 115u,
    TRPY_CMD = 112u,
    STATUS_DATA = 99u,
    OUTPUT_DATA = 100u,
    PPR_OUTPUT_DATA = 116u,
    PPR_GAINS = 103u,
  };


  typedef boost::shared_ptr< ::quadrotor_msgs::Serial_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::quadrotor_msgs::Serial_<ContainerAllocator> const> ConstPtr;

}; // struct Serial_

typedef ::quadrotor_msgs::Serial_<std::allocator<void> > Serial;

typedef boost::shared_ptr< ::quadrotor_msgs::Serial > SerialPtr;
typedef boost::shared_ptr< ::quadrotor_msgs::Serial const> SerialConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::quadrotor_msgs::Serial_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::quadrotor_msgs::Serial_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::quadrotor_msgs::Serial_<ContainerAllocator1> & lhs, const ::quadrotor_msgs::Serial_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.channel == rhs.channel &&
    lhs.type == rhs.type &&
    lhs.data == rhs.data;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::quadrotor_msgs::Serial_<ContainerAllocator1> & lhs, const ::quadrotor_msgs::Serial_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace quadrotor_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::quadrotor_msgs::Serial_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::quadrotor_msgs::Serial_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::quadrotor_msgs::Serial_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::quadrotor_msgs::Serial_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::quadrotor_msgs::Serial_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::quadrotor_msgs::Serial_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::quadrotor_msgs::Serial_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e448fb7595af9a8adfcab5ec241c7d4f";
  }

  static const char* value(const ::quadrotor_msgs::Serial_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe448fb7595af9a8aULL;
  static const uint64_t static_value2 = 0xdfcab5ec241c7d4fULL;
};

template<class ContainerAllocator>
struct DataType< ::quadrotor_msgs::Serial_<ContainerAllocator> >
{
  static const char* value()
  {
    return "quadrotor_msgs/Serial";
  }

  static const char* value(const ::quadrotor_msgs::Serial_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::quadrotor_msgs::Serial_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Note: These constants need to be kept in sync with the types\n"
"# defined in include/quadrotor_msgs/comm_types.h\n"
"uint8 SO3_CMD = 115 # 's' in base 10\n"
"uint8 TRPY_CMD = 112 # 'p' in base 10\n"
"uint8 STATUS_DATA = 99 # 'c' in base 10\n"
"uint8 OUTPUT_DATA = 100 # 'd' in base 10\n"
"uint8 PPR_OUTPUT_DATA = 116 # 't' in base 10\n"
"uint8 PPR_GAINS = 103 # 'g'\n"
"\n"
"Header header\n"
"uint8 channel\n"
"uint8 type # One of the types listed above\n"
"uint8[] data\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::quadrotor_msgs::Serial_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::quadrotor_msgs::Serial_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.channel);
      stream.next(m.type);
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Serial_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::quadrotor_msgs::Serial_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::quadrotor_msgs::Serial_<ContainerAllocator>& v)
  {
    if (false || !indent.empty())
      s << std::endl;
    s << indent << "header: ";
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    if (true || !indent.empty())
      s << std::endl;
    s << indent << "channel: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.channel);
    if (true || !indent.empty())
      s << std::endl;
    s << indent << "type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type);
    if (true || !indent.empty())
      s << std::endl;
    s << indent << "data: ";
    if (v.data.empty() || true)
      s << "[";
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      if (true && i > 0)
        s << ", ";
      else if (!true)
        s << std::endl << indent << "  -";
      Printer<uint8_t>::stream(s, true ? std::string() : indent + "    ", v.data[i]);
    }
    if (v.data.empty() || true)
      s << "]";
  }
};

} // namespace message_operations
} // namespace ros

#endif // QUADROTOR_MSGS_MESSAGE_SERIAL_H
