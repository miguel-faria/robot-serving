// Generated by gencpp from file robot_serving/MovementResponse.msg
// DO NOT EDIT!


#ifndef ROBOT_SERVING_MESSAGE_MOVEMENTRESPONSE_H
#define ROBOT_SERVING_MESSAGE_MOVEMENTRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <robot_serving/PMPTraj.h>

namespace robot_serving
{
template <class ContainerAllocator>
struct MovementResponse_
{
  typedef MovementResponse_<ContainerAllocator> Type;

  MovementResponse_()
    : robot_trajectory()  {
    }
  MovementResponse_(const ContainerAllocator& _alloc)
    : robot_trajectory(_alloc)  {
  (void)_alloc;
    }



   typedef  ::robot_serving::PMPTraj_<ContainerAllocator>  _robot_trajectory_type;
  _robot_trajectory_type robot_trajectory;




  typedef boost::shared_ptr< ::robot_serving::MovementResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robot_serving::MovementResponse_<ContainerAllocator> const> ConstPtr;

}; // struct MovementResponse_

typedef ::robot_serving::MovementResponse_<std::allocator<void> > MovementResponse;

typedef boost::shared_ptr< ::robot_serving::MovementResponse > MovementResponsePtr;
typedef boost::shared_ptr< ::robot_serving::MovementResponse const> MovementResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robot_serving::MovementResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robot_serving::MovementResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace robot_serving

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'robot_serving': ['/home/miguel/catkin_ws/src/robot_serving/msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::robot_serving::MovementResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_serving::MovementResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_serving::MovementResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_serving::MovementResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_serving::MovementResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_serving::MovementResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robot_serving::MovementResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a40735db44e0b9258ee4aa94858123bd";
  }

  static const char* value(const ::robot_serving::MovementResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa40735db44e0b925ULL;
  static const uint64_t static_value2 = 0x8ee4aa94858123bdULL;
};

template<class ContainerAllocator>
struct DataType< ::robot_serving::MovementResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robot_serving/MovementResponse";
  }

  static const char* value(const ::robot_serving::MovementResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robot_serving::MovementResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
PMPTraj 	robot_trajectory\n\
\n\
================================================================================\n\
MSG: robot_serving/PMPTraj\n\
# ROS message for a PMP trajectory. each entry of traj and time_step must have the same length\n\
PMPPoint[] 	traj		# vector, of the same size as the robot's DOFs, with the sequence of joint values at each time step for each DOF.\n\
float64[] 	time_step	# times of observations, in seconds, starting at zero\n\
================================================================================\n\
MSG: robot_serving/PMPPoint\n\
# Value of joint angles for one DOF\n\
float64[] 	joint_angles 	# sequence of angle values for one joint\n\
";
  }

  static const char* value(const ::robot_serving::MovementResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robot_serving::MovementResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.robot_trajectory);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct MovementResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robot_serving::MovementResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robot_serving::MovementResponse_<ContainerAllocator>& v)
  {
    s << indent << "robot_trajectory: ";
    s << std::endl;
    Printer< ::robot_serving::PMPTraj_<ContainerAllocator> >::stream(s, indent + "  ", v.robot_trajectory);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOT_SERVING_MESSAGE_MOVEMENTRESPONSE_H
