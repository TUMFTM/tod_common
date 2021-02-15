// Generated by gencpp from file image_exposure_msgs/ImageExposureStatistics.msg
// DO NOT EDIT!


#ifndef IMAGE_EXPOSURE_MSGS_MESSAGE_IMAGEEXPOSURESTATISTICS_H
#define IMAGE_EXPOSURE_MSGS_MESSAGE_IMAGEEXPOSURESTATISTICS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <statistics_msgs/Stats1D.h>
#include <statistics_msgs/Stats1D.h>

namespace image_exposure_msgs
{
template <class ContainerAllocator>
struct ImageExposureStatistics_
{
  typedef ImageExposureStatistics_<ContainerAllocator> Type;

  ImageExposureStatistics_()
    : stamp()
    , time_reference()
    , shutterms(0.0)
    , gaindb(0.0)
    , underExposed(0)
    , overExposed(0)
    , pixelVal()
    , pixelAge()
    , meanIrradiance(0.0)
    , minMeasurableIrradiance(0.0)
    , maxMeasurableIrradiance(0.0)
    , minObservedIrradiance(0.0)
    , maxObservedIrradiance(0.0)  {
    }
  ImageExposureStatistics_(const ContainerAllocator& _alloc)
    : stamp()
    , time_reference(_alloc)
    , shutterms(0.0)
    , gaindb(0.0)
    , underExposed(0)
    , overExposed(0)
    , pixelVal(_alloc)
    , pixelAge(_alloc)
    , meanIrradiance(0.0)
    , minMeasurableIrradiance(0.0)
    , maxMeasurableIrradiance(0.0)
    , minObservedIrradiance(0.0)
    , maxObservedIrradiance(0.0)  {
  (void)_alloc;
    }



   typedef ros::Time _stamp_type;
  _stamp_type stamp;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _time_reference_type;
  _time_reference_type time_reference;

   typedef float _shutterms_type;
  _shutterms_type shutterms;

   typedef float _gaindb_type;
  _gaindb_type gaindb;

   typedef uint32_t _underExposed_type;
  _underExposed_type underExposed;

   typedef uint32_t _overExposed_type;
  _overExposed_type overExposed;

   typedef  ::statistics_msgs::Stats1D_<ContainerAllocator>  _pixelVal_type;
  _pixelVal_type pixelVal;

   typedef  ::statistics_msgs::Stats1D_<ContainerAllocator>  _pixelAge_type;
  _pixelAge_type pixelAge;

   typedef double _meanIrradiance_type;
  _meanIrradiance_type meanIrradiance;

   typedef double _minMeasurableIrradiance_type;
  _minMeasurableIrradiance_type minMeasurableIrradiance;

   typedef double _maxMeasurableIrradiance_type;
  _maxMeasurableIrradiance_type maxMeasurableIrradiance;

   typedef double _minObservedIrradiance_type;
  _minObservedIrradiance_type minObservedIrradiance;

   typedef double _maxObservedIrradiance_type;
  _maxObservedIrradiance_type maxObservedIrradiance;





  typedef boost::shared_ptr< ::image_exposure_msgs::ImageExposureStatistics_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::image_exposure_msgs::ImageExposureStatistics_<ContainerAllocator> const> ConstPtr;

}; // struct ImageExposureStatistics_

typedef ::image_exposure_msgs::ImageExposureStatistics_<std::allocator<void> > ImageExposureStatistics;

typedef boost::shared_ptr< ::image_exposure_msgs::ImageExposureStatistics > ImageExposureStatisticsPtr;
typedef boost::shared_ptr< ::image_exposure_msgs::ImageExposureStatistics const> ImageExposureStatisticsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::image_exposure_msgs::ImageExposureStatistics_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::image_exposure_msgs::ImageExposureStatistics_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace image_exposure_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'image_exposure_msgs': ['/home/andis/tof/wsp/src/image_exposure_msgs/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'statistics_msgs': ['/home/andis/tof/wsp/src/statistics_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::image_exposure_msgs::ImageExposureStatistics_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::image_exposure_msgs::ImageExposureStatistics_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::image_exposure_msgs::ImageExposureStatistics_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::image_exposure_msgs::ImageExposureStatistics_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::image_exposure_msgs::ImageExposureStatistics_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::image_exposure_msgs::ImageExposureStatistics_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::image_exposure_msgs::ImageExposureStatistics_<ContainerAllocator> >
{
  static const char* value()
  {
    return "334dc170ce6287d1de470f25be78dd9e";
  }

  static const char* value(const ::image_exposure_msgs::ImageExposureStatistics_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x334dc170ce6287d1ULL;
  static const uint64_t static_value2 = 0xde470f25be78dd9eULL;
};

template<class ContainerAllocator>
struct DataType< ::image_exposure_msgs::ImageExposureStatistics_<ContainerAllocator> >
{
  static const char* value()
  {
    return "image_exposure_msgs/ImageExposureStatistics";
  }

  static const char* value(const ::image_exposure_msgs::ImageExposureStatistics_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::image_exposure_msgs::ImageExposureStatistics_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# message for exposure statistics reported a single image\n"
"time stamp         # image time stamp\n"
"string time_reference # The name of the reference clock for this message's timestamp.\n"
"float32 shutterms  # shutter durations in ms\n"
"float32 gaindb     # gain in decibels\n"
"# pixel exposure and latency statistics\n"
"uint32 underExposed # number of pixels underexposed\n"
"uint32 overExposed  # number of pixels overexposed\n"
"statistics_msgs/Stats1D pixelVal   # distribution of pixel values in the image\n"
"statistics_msgs/Stats1D pixelAge   # distribution of pixel ages in frames\n"
"# irradiance = pixelval/(shutterTime*10^(gaindb/10.0))\n"
"float64 meanIrradiance\n"
"float64 minMeasurableIrradiance\n"
"float64 maxMeasurableIrradiance\n"
"float64 minObservedIrradiance\n"
"float64 maxObservedIrradiance\n"
"\n"
"\n"
"\n"
"================================================================================\n"
"MSG: statistics_msgs/Stats1D\n"
"# statistics of a 1-D distribution\n"
"float64 min\n"
"float64 max \n"
"float64 mean \n"
"float64 variance\n"
"int64 N\n"
;
  }

  static const char* value(const ::image_exposure_msgs::ImageExposureStatistics_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::image_exposure_msgs::ImageExposureStatistics_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.stamp);
      stream.next(m.time_reference);
      stream.next(m.shutterms);
      stream.next(m.gaindb);
      stream.next(m.underExposed);
      stream.next(m.overExposed);
      stream.next(m.pixelVal);
      stream.next(m.pixelAge);
      stream.next(m.meanIrradiance);
      stream.next(m.minMeasurableIrradiance);
      stream.next(m.maxMeasurableIrradiance);
      stream.next(m.minObservedIrradiance);
      stream.next(m.maxObservedIrradiance);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ImageExposureStatistics_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::image_exposure_msgs::ImageExposureStatistics_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::image_exposure_msgs::ImageExposureStatistics_<ContainerAllocator>& v)
  {
    s << indent << "stamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.stamp);
    s << indent << "time_reference: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.time_reference);
    s << indent << "shutterms: ";
    Printer<float>::stream(s, indent + "  ", v.shutterms);
    s << indent << "gaindb: ";
    Printer<float>::stream(s, indent + "  ", v.gaindb);
    s << indent << "underExposed: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.underExposed);
    s << indent << "overExposed: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.overExposed);
    s << indent << "pixelVal: ";
    s << std::endl;
    Printer< ::statistics_msgs::Stats1D_<ContainerAllocator> >::stream(s, indent + "  ", v.pixelVal);
    s << indent << "pixelAge: ";
    s << std::endl;
    Printer< ::statistics_msgs::Stats1D_<ContainerAllocator> >::stream(s, indent + "  ", v.pixelAge);
    s << indent << "meanIrradiance: ";
    Printer<double>::stream(s, indent + "  ", v.meanIrradiance);
    s << indent << "minMeasurableIrradiance: ";
    Printer<double>::stream(s, indent + "  ", v.minMeasurableIrradiance);
    s << indent << "maxMeasurableIrradiance: ";
    Printer<double>::stream(s, indent + "  ", v.maxMeasurableIrradiance);
    s << indent << "minObservedIrradiance: ";
    Printer<double>::stream(s, indent + "  ", v.minObservedIrradiance);
    s << indent << "maxObservedIrradiance: ";
    Printer<double>::stream(s, indent + "  ", v.maxObservedIrradiance);
  }
};

} // namespace message_operations
} // namespace ros

#endif // IMAGE_EXPOSURE_MSGS_MESSAGE_IMAGEEXPOSURESTATISTICS_H
