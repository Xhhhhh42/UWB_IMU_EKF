#ifndef SLAM_ABSTRACT_IMPL_HPP
#define SLAM_ABSTRACT_IMPL_HPP

#include "slam_abstract.h"

namespace uwb_imu_fusion{

SLAM_System::SLAM_System( ros::NodeHandle& nh )
        :nh_(nh)
{
    raw_pointcloud_ = std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>( new pcl::PointCloud<pcl::PointXYZ>() );
    processed_pointcloud_ = std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>( new pcl::PointCloud<pcl::PointXYZ>() );

    // set base_link to the original of map.
    tf_map_base_ = tf::StampedTransform( tf::Transform( tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0) ), ros::Time::now(), "map", "base_link" );

    // send the common_msgs::MeasurementPosition/Velocity to the rest of the uav_os
    pub_measured_posi_ = nh_.advertise<geometry_msgs::Pose>( "measurement_position", 10 );
    pub_measured_vel_ = nh_.advertise<geometry_msgs::Twist>( "measurement_velocity", 10 );
    // pub_measured_posi_vel_ = nh_.advertise<common_msgs::MeasurementPosVel>("measurement_position_velocity", 10);

    pub_pointcloud_ = nh_.advertise<sensor_msgs::PointCloud2>( "/cloud_in", 10 );
    // visualize
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 10 );

    // parameters
    nh_.param<double>("slam_result_print_freq", slam_result_print_freq_, 1);
    slam_result_print_duration_ = 1.0 / slam_result_print_freq_;

    // nh_.param<bool>("is_xy_valid", is_xy_valid_, true);
    // nh_.param<bool>("is_z_valid", is_z_valid_, true);
    // nh_.param<bool>("is_yaw_valid", is_yaw_valid_, true);

    // nh_.param<bool>("is_v_xy_valid", is_v_xy_valid_, false);
    // nh_.param<bool>("is_v_z_valid", is_v_z_valid_, false);
}


Eigen::Matrix4d SLAM_System::tfNWU_2_GCamera( const tf::Transform& ros_transform )
{
    Eigen::Matrix3d R_camera;
    Eigen::Vector3d T_camera;

    // convert R
    tf::Quaternion tf_rotation_NWU = ros_transform.getRotation();
    tf::Quaternion tf_rotation_camera = tf::Quaternion( -tf_rotation_NWU.getY(), -tf_rotation_NWU.getZ(),
                                                        tf_rotation_NWU.getX(), tf_rotation_NWU.getW() ); // (x,y,z,w)
    tf::matrixTFToEigen( tf::Matrix3x3( tf_rotation_camera ), R_camera );

    // convert T
    tf::Vector3 tf_translation_NWU = ros_transform.getOrigin();
    T_camera(0) = -tf_translation_NWU.getY();
    T_camera(1) = -tf_translation_NWU.getZ();
    T_camera(2) = tf_translation_NWU.getX();

    // assemble
    Eigen::MatrixXd G_camera_inhomo(3,4);
    G_camera_inhomo<<R_camera, T_camera;
    Eigen::MatrixXd tmp(1,4);
    tmp<<0,0,0,1;
    Eigen::Matrix4d G_camera;
    G_camera<<G_camera_inhomo, tmp;

    return G_camera;
}

tf::Transform SLAM_System::GCamera_2_tfNWU( const Eigen::Matrix4d& G )
{
    // convert rotation
    tf::Matrix3x3 basis_camera;
    tf::matrixEigenToTF(G.topLeftCorner(3,3), basis_camera);
    tf::Quaternion rotation_camera;
    basis_camera.getRotation(rotation_camera);
    tf::Quaternion rotation_NWU(rotation_camera.getZ(),
                                -rotation_camera.getX(),
                                -rotation_camera.getY(),
                                rotation_camera.getW());


    // convert translation
    tf::Vector3 translation_camera(G(0,3), G(1,3), G(2,3));
    tf::Vector3 translation_NWU(translation_camera.getZ(),
                                -translation_camera.getX(),
                                -translation_camera.getY());

    // assemble
    tf::Transform tf_NWU(rotation_NWU, translation_NWU);

    return tf_NWU;
}



geometry_msgs::PoseStamped SLAM_System::tfNWU_2_PositionNWU(const tf::Transform &ros_transform){

    // get x,y,z
    tf::Vector3 tf_translation_NWU = ros_transform.getOrigin();

    // get yaw
    tf::Quaternion tf_rotation_NWU = ros_transform.getRotation();
    geometry_msgs::Quaternion q_NWU;
    tf::quaternionTFToMsg(tf_rotation_NWU, q_NWU);


    geometry_msgs::PoseStamped position_NWU;
    position_NWU.pose.position.x = tf_translation_NWU.getX();
    position_NWU.pose.position.y = tf_translation_NWU.getY();
    position_NWU.pose.position.z = tf_translation_NWU.getZ();
    position_NWU.pose.orientation = q_NWU;

    return position_NWU;
}


geometry_msgs::PoseStamped SLAM_System::GCamera_2_PositionNWU(const Eigen::Matrix4d &G){

    geometry_msgs::PoseStamped position_NWU;

    // convert rotation
    tf::Matrix3x3 basis_camera;
    tf::matrixEigenToTF(G.topLeftCorner(3,3), basis_camera);
    double roll_camera, pitch_camera, yaw_camera;
    basis_camera.getRPY(roll_camera,pitch_camera,yaw_camera);

    geometry_msgs::Quaternion q_NWU =  tf::createQuaternionMsgFromRollPitchYaw(yaw_camera, -roll_camera, -pitch_camera);
    position_NWU.pose.orientation = q_NWU;


    // convert translation
    position_NWU.pose.position.x = G(2,3);
    position_NWU.pose.position.y = -G(0,3);
    position_NWU.pose.position.z = -G(1,3);

    return position_NWU;

}

// visualize
void SLAM_System::visualizeTrajectoryCallback( const ros::TimerEvent &event ){

    // avoid adding visualization marker when UAV doesn't move.
    if( trajectory_.size()==0){
        trajectory_.push_back(curr_world_posi_);
    }
    
    geometry_msgs::PoseStamped last_trajectory = trajectory_.at(trajectory_.size()-1);

    if(std::fabs(curr_world_posi_.pose.position.x-last_trajectory.pose.position.x)>0.01 &&
       std::fabs(curr_world_posi_.pose.position.y-last_trajectory.pose.position.y)>0.01 &&
       std::fabs(curr_world_posi_.pose.position.z-last_trajectory.pose.position.z)>0.01){
        trajectory_.push_back(curr_world_posi_);
    }


    visualization_msgs::Marker points;
    points.header.frame_id = "/map";
    points.header.stamp = ros::Time::now();
    points.type = visualization_msgs::Marker::POINTS;
    points.action = visualization_msgs::Marker::ADD;
    points.id = 0;
    points.lifetime = ros::Duration();

    points.scale.x = 0.1;
    points.scale.y = 0.1;
    points.scale.z = 0.1;
    points.color.g = 1.0;
    points.color.a = 1.0;

    for(int i=0;i<trajectory_.size();++i){
        geometry_msgs::Point point = trajectory_.at(i).pose.position;
        points.points.push_back(point);
    }

    pub_marker_.publish(points);

//    ROS_INFO("marker point count: %d", int(points.points.size()));
}

} //namespace uwb_imu_fusion

#endif // SLAM_ABSTRACT_IMPL_HPP