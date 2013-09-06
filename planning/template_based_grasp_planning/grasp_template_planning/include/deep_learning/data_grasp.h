/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         log_loader.h

 \author       Daniel Kappler
 \date         July 30, 2013

 *********************************************************************/

#ifndef DATA_GRASP_H
#define DATA_GRASP_H

#include <ostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <grasp_template/grasp_template.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>

#include <ros/serialization.h>
#include <ros/message_traits.h>
#include <ros/message_operations.h>
#include <ros/builtin_message_traits.h>
#include <ros/macros.h>
#include <ros/assert.h>
#include <ros/time.h>

#include "geometry_msgs/PoseStamped.h"
#include "grasp_template/Heightmap.h"
#include "grasp_template_planning/DoubleVector.h"
#include "grasp_template_planning/GraspAnalysis.h"

namespace deep_learning {

template<class ContainerAllocator>
struct Data_grasp_log_ {
	typedef Data_grasp_log_<ContainerAllocator> Type;

	typedef ros::Time _stamp_type;
	ros::Time stamp;

	typedef ::grasp_template_planning::DoubleVector_<ContainerAllocator> _gripper_joints_type;
	::grasp_template_planning::DoubleVector_<ContainerAllocator> gripper_joints;

	typedef ::geometry_msgs::PoseStamped_<ContainerAllocator> _gripper_pose_type;
	::geometry_msgs::PoseStamped_<ContainerAllocator> gripper_pose;

	typedef ::geometry_msgs::PoseStamped_<ContainerAllocator> _grasp_template_pose_type;
	::geometry_msgs::PoseStamped_<ContainerAllocator> grasp_template_pose;

	typedef ::grasp_template::Heightmap_<ContainerAllocator> _grasp_template_heightmap_type;
	::grasp_template::Heightmap_<ContainerAllocator> grasp_template_heightmap;

	std::size_t uuid_database;
	std::size_t uuid_database_template;

	int success;

	Data_grasp_log_() :
			stamp(), gripper_joints(), gripper_pose(), grasp_template_pose(), grasp_template_heightmap(), uuid_database(), success() {
	}

	Data_grasp_log_(const ContainerAllocator& _alloc) :
			stamp(), gripper_joints(_alloc), gripper_pose(_alloc), grasp_template_pose(
					_alloc), grasp_template_heightmap(_alloc), uuid_database(
					_alloc), success(_alloc) {
	}

	typedef boost::shared_ptr<
			::deep_learning::Data_grasp_log_<ContainerAllocator> > Ptr;
	typedef boost::shared_ptr<
			::deep_learning::Data_grasp_log_<ContainerAllocator> const> ConstPtr;
	boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
};

typedef ::deep_learning::Data_grasp_log_<std::allocator<void> > Data_grasp_log;
typedef boost::shared_ptr< ::deep_learning::Data_grasp_log> Data_grasp_log_ptr;
typedef boost::shared_ptr< ::deep_learning::Data_grasp_log const> Data_grasp_log_const_ptr;

class Data_grasp {
private:
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	std::vector<double> gripper_joints;
	geometry_msgs::Pose gripper_pose;
	grasp_template::GraspTemplate grasp_template;
	std::size_t uuid_database;
	std::size_t uuid_database_template;
	int success;

	Data_grasp();
	Data_grasp(Data_grasp_log &d_grasp);
	Data_grasp(const std::vector<double> &gripper_joints,
			const geometry_msgs::Pose &pgripper_pose,
			const grasp_template::GraspTemplate &pgrasp_template,
			const size_t &puuid_database, const size_t &puuid_database_template,
			int psuccess);
	Data_grasp(const std::vector<double> &gripper_joints,
			const geometry_msgs::Pose &pgripper_pose,
			const grasp_template::GraspTemplate &pgrasp_template);
	Data_grasp(const std::vector<double> &gripper_joints,
			const geometry_msgs::Pose &pgripper_pose,
			const grasp_template::GraspTemplate &pgrasp_template, int psuccess);

	static void Add(
			grasp_template_planning::GraspAnalysis &grasp_analysis,
			int success,
			std::vector<Data_grasp> &result_data_grasps);

	void Valid_uuid();

	virtual ~Data_grasp() {
	}
	;
};

template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s,
		const ::deep_learning::Data_grasp_log_<ContainerAllocator> & v) {
	ros::message_operations::Printer<
			::deep_learning::Data_grasp_log_<ContainerAllocator> >::stream(s,
			"", v);
	return s;
}

} // namespace deep_learning

namespace ros {
namespace message_traits {
template<class ContainerAllocator> struct IsMessage<
		::deep_learning::Data_grasp_log_<ContainerAllocator> > : public TrueType {
};
template<class ContainerAllocator> struct IsMessage<
		::deep_learning::Data_grasp_log_<ContainerAllocator> const> : public TrueType {
};
template<class ContainerAllocator>
struct MD5Sum< ::deep_learning::Data_grasp_log_<ContainerAllocator> > {
	static const char* value() {
		return "86ffeb77ffc13eae26ac32a006e21c61";
	}

	static const char* value(
			const ::deep_learning::Data_grasp_log_<ContainerAllocator> &) {
		return value();
	}
};

template<class ContainerAllocator>
struct DataType< ::deep_learning::Data_grasp_log_<ContainerAllocator> > {
	static const char* value() {
		return "deep_learning/Data_grasp_log";
	}

	static const char* value(
			const ::deep_learning::Data_grasp_log_<ContainerAllocator> &) {
		return value();
	}
};

template<class ContainerAllocator>
struct Definition< ::deep_learning::Data_grasp_log_<ContainerAllocator> > {
	static const char* value() {
		return "not given";
	}

	static const char* value(
			const ::deep_learning::Data_grasp_log_<ContainerAllocator> &) {
		return value();
	}
};

} // namespace message_traits
} // namespace ros

namespace ros {
namespace serialization {

template<class ContainerAllocator> struct Serializer<
		::deep_learning::Data_grasp_log_<ContainerAllocator> > {
	template<typename Stream, typename T> inline static void allInOne(
			Stream& stream, T m) {
		stream.next(m.stamp);
		stream.next(m.gripper_joints);
		stream.next(m.gripper_pose);
		stream.next(m.grasp_template_pose);
		stream.next(m.grasp_template_heightmap);
		stream.next(m.uuid_database);
		stream.next(m.uuid_database_template);
		stream.next(m.success);
	}

	ROS_DECLARE_ALLINONE_SERIALIZER;
};
// struct Data_grasp_log_
}// namespace serialization
} // namespace ros

namespace ros {
namespace message_operations {

template<class ContainerAllocator>
struct Printer< ::deep_learning::Data_grasp_log_<ContainerAllocator> > {
	template<typename Stream> static void stream(Stream& s,
			const std::string& indent,
			const deep_learning::Data_grasp_log_<ContainerAllocator> & v) {
		s << indent << "uuid_database: ";
		Printer<std::size_t>::stream(s, indent + "  ", v.uuid_database);
		s << indent << "uuid_database_template: ";
		Printer<std::size_t>::stream(s, indent + "  ",
				v.uuid_database_template);

		s << indent << "stamp: ";

		Printer<ros::Time>::stream(s, indent + "  ", v.stamp);
		s << indent << "gripper_joints: ";
		s << std::endl;
		Printer< ::grasp_template_planning::DoubleVector_<ContainerAllocator> >::stream(
				s, indent + "  ", v.gripper_joints);

		s << indent << "gripper_pose: ";
		s << std::endl;
		Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s,
				indent + "  ", v.gripper_pose);

		s << indent << "grasp_template_heightmap: ";
		s << std::endl;
		Printer< ::grasp_template::Heightmap_<ContainerAllocator> >::stream(s,
				indent + "  ", v.grasp_template_heightmap);
		s << indent << "grasp_template_pose: ";
		s << std::endl;
		Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s,
				indent + "  ", v.grasp_template_pose);
		s << std::endl;
	}
};

} // namespace message_operations
} // namespace ros

#endif /*DATA_STORAGE_H*/
