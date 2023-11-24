// Generated by gencpp from file carla_msgs/CarlaLaneInvasionEvent.msg
// DO NOT EDIT!


#ifndef CARLA_MSGS_MESSAGE_CARLALANEINVASIONEVENT_H
#define CARLA_MSGS_MESSAGE_CARLALANEINVASIONEVENT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace carla_msgs
{
template <class ContainerAllocator>
struct CarlaLaneInvasionEvent_
{
  typedef CarlaLaneInvasionEvent_<ContainerAllocator> Type;

  CarlaLaneInvasionEvent_()
    : header()
    , crossed_lane_markings()  {
    }
  CarlaLaneInvasionEvent_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , crossed_lane_markings(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> _crossed_lane_markings_type;
  _crossed_lane_markings_type crossed_lane_markings;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(LANE_MARKING_OTHER)
  #undef LANE_MARKING_OTHER
#endif
#if defined(_WIN32) && defined(LANE_MARKING_BROKEN)
  #undef LANE_MARKING_BROKEN
#endif
#if defined(_WIN32) && defined(LANE_MARKING_SOLID)
  #undef LANE_MARKING_SOLID
#endif

  enum {
    LANE_MARKING_OTHER = 0,
    LANE_MARKING_BROKEN = 1,
    LANE_MARKING_SOLID = 2,
  };


  typedef boost::shared_ptr< ::carla_msgs::CarlaLaneInvasionEvent_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::carla_msgs::CarlaLaneInvasionEvent_<ContainerAllocator> const> ConstPtr;

}; // struct CarlaLaneInvasionEvent_

typedef ::carla_msgs::CarlaLaneInvasionEvent_<std::allocator<void> > CarlaLaneInvasionEvent;

typedef boost::shared_ptr< ::carla_msgs::CarlaLaneInvasionEvent > CarlaLaneInvasionEventPtr;
typedef boost::shared_ptr< ::carla_msgs::CarlaLaneInvasionEvent const> CarlaLaneInvasionEventConstPtr;

// constants requiring out of line definition

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::carla_msgs::CarlaLaneInvasionEvent_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::carla_msgs::CarlaLaneInvasionEvent_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::carla_msgs::CarlaLaneInvasionEvent_<ContainerAllocator1> & lhs, const ::carla_msgs::CarlaLaneInvasionEvent_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.crossed_lane_markings == rhs.crossed_lane_markings;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::carla_msgs::CarlaLaneInvasionEvent_<ContainerAllocator1> & lhs, const ::carla_msgs::CarlaLaneInvasionEvent_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace carla_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::carla_msgs::CarlaLaneInvasionEvent_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::carla_msgs::CarlaLaneInvasionEvent_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::carla_msgs::CarlaLaneInvasionEvent_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::carla_msgs::CarlaLaneInvasionEvent_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::carla_msgs::CarlaLaneInvasionEvent_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::carla_msgs::CarlaLaneInvasionEvent_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::carla_msgs::CarlaLaneInvasionEvent_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3846cea49c4897522899dbda091314e8";
  }

  static const char* value(const ::carla_msgs::CarlaLaneInvasionEvent_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3846cea49c489752ULL;
  static const uint64_t static_value2 = 0x2899dbda091314e8ULL;
};

template<class ContainerAllocator>
struct DataType< ::carla_msgs::CarlaLaneInvasionEvent_<ContainerAllocator> >
{
  static const char* value()
  {
    return "carla_msgs/CarlaLaneInvasionEvent";
  }

  static const char* value(const ::carla_msgs::CarlaLaneInvasionEvent_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::carla_msgs::CarlaLaneInvasionEvent_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#\n"
"# Copyright (c) 2019 Intel Corporation.\n"
"#\n"
"# This work is licensed under the terms of the MIT license.\n"
"# For a copy, see <https://opensource.org/licenses/MIT>.\n"
"#\n"
"# A lane invasion event\n"
"\n"
"Header header\n"
"\n"
"int32[] crossed_lane_markings\n"
"\n"
"int32 LANE_MARKING_OTHER=0\n"
"int32 LANE_MARKING_BROKEN=1\n"
"int32 LANE_MARKING_SOLID=2\n"
"\n"
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

  static const char* value(const ::carla_msgs::CarlaLaneInvasionEvent_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::carla_msgs::CarlaLaneInvasionEvent_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.crossed_lane_markings);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CarlaLaneInvasionEvent_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::carla_msgs::CarlaLaneInvasionEvent_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::carla_msgs::CarlaLaneInvasionEvent_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "crossed_lane_markings[]" << std::endl;
    for (size_t i = 0; i < v.crossed_lane_markings.size(); ++i)
    {
      s << indent << "  crossed_lane_markings[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.crossed_lane_markings[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // CARLA_MSGS_MESSAGE_CARLALANEINVASIONEVENT_H
