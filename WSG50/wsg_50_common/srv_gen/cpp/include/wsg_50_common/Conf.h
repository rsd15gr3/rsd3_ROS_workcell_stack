/* Auto-generated by genmsg_cpp for file /home/clonecomputer/catkin_ws/src/wsg50/wsg_50_common/srv/Conf.srv */
#ifndef WSG_50_COMMON_SERVICE_CONF_H
#define WSG_50_COMMON_SERVICE_CONF_H
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

#include "ros/service_traits.h"




namespace wsg_50_common
{
template <class ContainerAllocator>
struct ConfRequest_ {
  typedef ConfRequest_<ContainerAllocator> Type;

  ConfRequest_()
  : val(0.0)
  {
  }

  ConfRequest_(const ContainerAllocator& _alloc)
  : val(0.0)
  {
  }

  typedef float _val_type;
  float val;


  typedef boost::shared_ptr< ::wsg_50_common::ConfRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::wsg_50_common::ConfRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct ConfRequest
typedef  ::wsg_50_common::ConfRequest_<std::allocator<void> > ConfRequest;

typedef boost::shared_ptr< ::wsg_50_common::ConfRequest> ConfRequestPtr;
typedef boost::shared_ptr< ::wsg_50_common::ConfRequest const> ConfRequestConstPtr;



template <class ContainerAllocator>
struct ConfResponse_ {
  typedef ConfResponse_<ContainerAllocator> Type;

  ConfResponse_()
  : error(0)
  {
  }

  ConfResponse_(const ContainerAllocator& _alloc)
  : error(0)
  {
  }

  typedef uint8_t _error_type;
  uint8_t error;


  typedef boost::shared_ptr< ::wsg_50_common::ConfResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::wsg_50_common::ConfResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct ConfResponse
typedef  ::wsg_50_common::ConfResponse_<std::allocator<void> > ConfResponse;

typedef boost::shared_ptr< ::wsg_50_common::ConfResponse> ConfResponsePtr;
typedef boost::shared_ptr< ::wsg_50_common::ConfResponse const> ConfResponseConstPtr;


struct Conf
{

typedef ConfRequest Request;
typedef ConfResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct Conf
} // namespace wsg_50_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::wsg_50_common::ConfRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::wsg_50_common::ConfRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::wsg_50_common::ConfRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c9ee899b5f0899afa6060c9ba611652c";
  }

  static const char* value(const  ::wsg_50_common::ConfRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xc9ee899b5f0899afULL;
  static const uint64_t static_value2 = 0xa6060c9ba611652cULL;
};

template<class ContainerAllocator>
struct DataType< ::wsg_50_common::ConfRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "wsg_50_common/ConfRequest";
  }

  static const char* value(const  ::wsg_50_common::ConfRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::wsg_50_common::ConfRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32 val\n\
\n\
";
  }

  static const char* value(const  ::wsg_50_common::ConfRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::wsg_50_common::ConfRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::wsg_50_common::ConfResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::wsg_50_common::ConfResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::wsg_50_common::ConfResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bf8e3bc5443691736455ca47e3128027";
  }

  static const char* value(const  ::wsg_50_common::ConfResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xbf8e3bc544369173ULL;
  static const uint64_t static_value2 = 0x6455ca47e3128027ULL;
};

template<class ContainerAllocator>
struct DataType< ::wsg_50_common::ConfResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "wsg_50_common/ConfResponse";
  }

  static const char* value(const  ::wsg_50_common::ConfResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::wsg_50_common::ConfResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint8 error\n\
\n\
\n\
";
  }

  static const char* value(const  ::wsg_50_common::ConfResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::wsg_50_common::ConfResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::wsg_50_common::ConfRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.val);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ConfRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::wsg_50_common::ConfResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.error);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ConfResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<wsg_50_common::Conf> {
  static const char* value() 
  {
    return "0c7a1839200830facdc1205cf36615d5";
  }

  static const char* value(const wsg_50_common::Conf&) { return value(); } 
};

template<>
struct DataType<wsg_50_common::Conf> {
  static const char* value() 
  {
    return "wsg_50_common/Conf";
  }

  static const char* value(const wsg_50_common::Conf&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<wsg_50_common::ConfRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0c7a1839200830facdc1205cf36615d5";
  }

  static const char* value(const wsg_50_common::ConfRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<wsg_50_common::ConfRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "wsg_50_common/Conf";
  }

  static const char* value(const wsg_50_common::ConfRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<wsg_50_common::ConfResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0c7a1839200830facdc1205cf36615d5";
  }

  static const char* value(const wsg_50_common::ConfResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<wsg_50_common::ConfResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "wsg_50_common/Conf";
  }

  static const char* value(const wsg_50_common::ConfResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // WSG_50_COMMON_SERVICE_CONF_H

