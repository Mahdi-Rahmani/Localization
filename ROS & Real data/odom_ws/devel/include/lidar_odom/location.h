// Generated by gencpp from file lidar_odom/location.msg
// DO NOT EDIT!


#ifndef LIDAR_ODOM_MESSAGE_LOCATION_H
#define LIDAR_ODOM_MESSAGE_LOCATION_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace lidar_odom
{
template <class ContainerAllocator>
struct location_
{
  typedef location_<ContainerAllocator> Type;

  location_()
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  location_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
  (void)_alloc;
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _z_type;
  _z_type z;





  typedef boost::shared_ptr< ::lidar_odom::location_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lidar_odom::location_<ContainerAllocator> const> ConstPtr;

}; // struct location_

typedef ::lidar_odom::location_<std::allocator<void> > location;

typedef boost::shared_ptr< ::lidar_odom::location > locationPtr;
typedef boost::shared_ptr< ::lidar_odom::location const> locationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lidar_odom::location_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lidar_odom::location_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::lidar_odom::location_<ContainerAllocator1> & lhs, const ::lidar_odom::location_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::lidar_odom::location_<ContainerAllocator1> & lhs, const ::lidar_odom::location_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace lidar_odom

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::lidar_odom::location_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lidar_odom::location_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lidar_odom::location_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lidar_odom::location_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lidar_odom::location_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lidar_odom::location_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lidar_odom::location_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4a842b65f413084dc2b10fb484ea7f17";
  }

  static const char* value(const ::lidar_odom::location_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4a842b65f413084dULL;
  static const uint64_t static_value2 = 0xc2b10fb484ea7f17ULL;
};

template<class ContainerAllocator>
struct DataType< ::lidar_odom::location_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lidar_odom/location";
  }

  static const char* value(const ::lidar_odom::location_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lidar_odom::location_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::lidar_odom::location_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lidar_odom::location_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct location_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lidar_odom::location_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lidar_odom::location_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LIDAR_ODOM_MESSAGE_LOCATION_H