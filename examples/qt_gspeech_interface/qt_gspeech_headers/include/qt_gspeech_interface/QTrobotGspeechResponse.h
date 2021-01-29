// Generated by gencpp from file qt_gspeech_interface/QTrobotGspeechResponse.msg
// DO NOT EDIT!


#ifndef QT_GSPEECH_INTERFACE_MESSAGE_QTROBOTGSPEECHRESPONSE_H
#define QT_GSPEECH_INTERFACE_MESSAGE_QTROBOTGSPEECHRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace qt_gspeech_interface
{
template <class ContainerAllocator>
struct QTrobotGspeechResponse_
{
  typedef QTrobotGspeechResponse_<ContainerAllocator> Type;

  QTrobotGspeechResponse_()
    : transcript()  {
    }
  QTrobotGspeechResponse_(const ContainerAllocator& _alloc)
    : transcript(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _transcript_type;
  _transcript_type transcript;





  typedef boost::shared_ptr< ::qt_gspeech_interface::QTrobotGspeechResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::qt_gspeech_interface::QTrobotGspeechResponse_<ContainerAllocator> const> ConstPtr;

}; // struct QTrobotGspeechResponse_

typedef ::qt_gspeech_interface::QTrobotGspeechResponse_<std::allocator<void> > QTrobotGspeechResponse;

typedef boost::shared_ptr< ::qt_gspeech_interface::QTrobotGspeechResponse > QTrobotGspeechResponsePtr;
typedef boost::shared_ptr< ::qt_gspeech_interface::QTrobotGspeechResponse const> QTrobotGspeechResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::qt_gspeech_interface::QTrobotGspeechResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::qt_gspeech_interface::QTrobotGspeechResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace qt_gspeech_interface

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::qt_gspeech_interface::QTrobotGspeechResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::qt_gspeech_interface::QTrobotGspeechResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::qt_gspeech_interface::QTrobotGspeechResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::qt_gspeech_interface::QTrobotGspeechResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::qt_gspeech_interface::QTrobotGspeechResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::qt_gspeech_interface::QTrobotGspeechResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::qt_gspeech_interface::QTrobotGspeechResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d57af9fafe11c5c53756ce2839af175d";
  }

  static const char* value(const ::qt_gspeech_interface::QTrobotGspeechResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd57af9fafe11c5c5ULL;
  static const uint64_t static_value2 = 0x3756ce2839af175dULL;
};

template<class ContainerAllocator>
struct DataType< ::qt_gspeech_interface::QTrobotGspeechResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "qt_gspeech_interface/QTrobotGspeechResponse";
  }

  static const char* value(const ::qt_gspeech_interface::QTrobotGspeechResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::qt_gspeech_interface::QTrobotGspeechResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string transcript\n\
\n\
";
  }

  static const char* value(const ::qt_gspeech_interface::QTrobotGspeechResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::qt_gspeech_interface::QTrobotGspeechResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.transcript);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct QTrobotGspeechResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::qt_gspeech_interface::QTrobotGspeechResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::qt_gspeech_interface::QTrobotGspeechResponse_<ContainerAllocator>& v)
  {
    s << indent << "transcript: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.transcript);
  }
};

} // namespace message_operations
} // namespace ros

#endif // QT_GSPEECH_INTERFACE_MESSAGE_QTROBOTGSPEECHRESPONSE_H
