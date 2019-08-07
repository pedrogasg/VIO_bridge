#include "../../include/PX4_realsense_bridge/PX4_realsense_bridge.h"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

namespace bridge {

PX4_Realsense_Bridge::PX4_Realsense_Bridge(const ros::NodeHandle& nh)
    : nh_(nh) {
  tf_listener_ = new tf::TransformListener(
      ros::Duration(tf::Transformer::DEFAULT_CACHE_TIME), true);

  // initialize subscribers
  odom_sub_ = nh_.subscribe<const nav_msgs::Odometry&>(
      "/camera/odom/sample", 1, &PX4_Realsense_Bridge::odomCallback, this);
  // publishers
  mavros_odom_pub_ =
      nh_.advertise<nav_msgs::Odometry>("/mavros/odometry/out", 10);
  mavros_system_status_pub_ =
      nh_.advertise<mavros_msgs::CompanionProcessStatus>("/mavros/companion_process/status", 1);

};

PX4_Realsense_Bridge::~PX4_Realsense_Bridge() { delete tf_listener_; }


void PX4_Realsense_Bridge::odomCallback(const nav_msgs::Odometry& msg) {

	if(!world_link_established_){
		tf::StampedTransform transform_px4, transform_cam;
		tf_listener_->lookupTransform("local_origin", "camera_downward", ros::Time(0), transform_px4);
		tf_listener_->lookupTransform("camera_pose_frame", "camera_odom_frame", ros::Time(0), transform_cam);

		//publish world link transform
		static tf2_ros::StaticTransformBroadcaster br;
		geometry_msgs::TransformStamped tf_msg;
		//tf::Transform transform_world = transform_px4.serialize(transform_cam);
		transform_cam.frame_id_ = "camera_downward";
		transform_cam.child_frame_id_ = "camera_odom_frame";
		transformStampedTFToMsg(transform_cam, tf_msg);
		br.sendTransform(tf_msg);
		world_link_established_ = true;
	}

  // publish odometry msg
  nav_msgs::Odometry output = msg;
  //output.header.frame_id = "local_origin";
  //output.child_frame_id = "camera_downward";
  mavros_odom_pub_.publish(output);

  last_system_status_ = system_status_;

  // check confidence in vision estimate by looking at covariance
  if( msg.pose.covariance[0] > 0.1 ) // low confidence -> reboot companion
  {
    system_status_ = MAV_STATE::MAV_STATE_FLIGHT_TERMINATION;
  }
  else if( msg.pose.covariance[0] == 0.1 ) // medium confidence
  {
    system_status_ = MAV_STATE::MAV_STATE_CRITICAL;
  }
  else if( msg.pose.covariance[0] == 0.01 ) // high confidence
  {
    system_status_ = MAV_STATE::MAV_STATE_ACTIVE;
  }
  else
  {
    ROS_WARN_STREAM("Unexpected vision sensor variance");
  }  

  // publish system status immediately if it changed
  if( last_system_status_ != system_status_ )
  {
    publishSystemStatus();
  }

}

void PX4_Realsense_Bridge::publishSystemStatus(){
  
  mavros_msgs::CompanionProcessStatus status_msg;

  status_msg.header.stamp = ros::Time::now();
  status_msg.component = 197;  // MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
  status_msg.state = (int)system_status_;

  mavros_system_status_pub_.publish(status_msg);
}

}
