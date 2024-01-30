#ifndef SLAM_ABSTRACT_HPP
#define SLAM_ABSTRACT_HPP

#include <ros/ros.h>

// ros callback
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <ros/spinner.h>

#include <vector>
// #include "basic_function.hpp"

// #include <sensor_msgs/Image.h>
// #include <cv_bridge/cv_bridge.h>
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

// tf
#include <tf/transform_broadcaster.h>
#include "tf_conversions/tf_eigen.h"

// // opencv 2
// #include <opencv2/opencv.hpp>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/core/eigen.hpp>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/radius_outlier_removal.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl/filters/passthrough.h>


// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// // boost thread
// #include <boost/thread/thread.hpp>
// #include <boost/thread/mutex.hpp>

namespace uwb_imu_fusion{


class SLAM_System{
public:
    explicit SLAM_System( ros::NodeHandle& nh );        

    ~SLAM_System() {}

    void visualizeTrajectoryCallback( const ros::TimerEvent &event );

    virtual void solveSLAM( const ros::TimerEvent& event ) = 0;

private:
    ros::NodeHandle nh_;

    // all in ros NWU coordinate
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > raw_pointcloud_;  // raw input from sensor driver
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > processed_pointcloud_; // usually down-sampled

    // tf transform that represent camera/base's pose, in NWU coordinate
    tf::StampedTransform tf_map_base_;
    tf::TransformBroadcaster tf_broadcaster_;

    // common_msgs::MeasurementPosition m_position_world_curr_NWU;
    // common_msgs::MeasurementVelocity m_velocity_world_curr_NWU;
    geometry_msgs::PoseStamped curr_world_posi_;
    geometry_msgs::TwistStamped curr_world_vel_;
    Eigen::Matrix4d curr_world_Camera_; 

    // send the common_msgs::MeasurementPosition/Velocity to the rest of the uav_os
    ros::Publisher pub_measured_posi_;
    ros::Publisher pub_measured_vel_;
    // ros::Publisher pub_measured_posi_vel_;

    // send point cloud
    ros::Publisher pub_pointcloud_;
    // visualization
    ros::Publisher pub_marker_;

    // debug parameters
    double slam_result_print_duration_, slam_result_print_freq_;
    
    // bool is_xy_valid_;
    // bool is_z_valid_;
    
    // bool is_yaw_valid_;

    // bool is_v_xy_valid_;
    // bool is_v_z_valid_;

    // store the trajectory
    std::vector<geometry_msgs::PoseStamped> trajectory_;

protected:
    // spinOnce for my own callback queue
    inline void spinOnce() {
        ros::CallbackQueue* queue = dynamic_cast<ros::CallbackQueue*>( nh_.getCallbackQueue() );
        queue->callAvailable();
    }

    Eigen::Matrix4d tfNWU_2_GCamera(const tf::Transform& ros_transform); // camera is East-Down-North
    tf::Transform GCamera_2_tfNWU(const Eigen::Matrix4d& G);

    geometry_msgs::PoseStamped tfNWU_2_PositionNWU(const tf::Transform& ros_transform);
    geometry_msgs::PoseStamped GCamera_2_PositionNWU(const Eigen::Matrix4d& G);

    inline std::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > getRawPointcloudPointer() const{
        return raw_pointcloud_;
    }

    inline std::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > getProcessedPointcloudPointer() const{
        return processed_pointcloud_;
    }

    inline void setRawPointcloud( const pcl::PointCloud<pcl::PointXYZ>& raw_pc ){
        *raw_pointcloud_ = raw_pc;
    }

    inline void setProcessedPointcloud( const pcl::PointCloud<pcl::PointXYZ>& processed_pc ){
        *processed_pointcloud_ = processed_pc;
    }

    inline void setRawPointcloudHeader(){
        raw_pointcloud_->header.frame_id = "base_link";
        pcl_conversions::toPCL( ros::Time::now(), raw_pointcloud_->header.stamp );
    }

    inline void setProcessedPointcloudHeader(){
        processed_pointcloud_->header.frame_id = "base_link";
        pcl_conversions::toPCL(ros::Time::now(), processed_pointcloud_->header.stamp);
    }

    inline void sendRawPointCloud(){
        std::shared_ptr<sensor_msgs::PointCloud2> pd_ros( new sensor_msgs::PointCloud2() );
        pcl::toROSMsg( *raw_pointcloud_, *pd_ros );
        pub_pointcloud_.publish( pd_ros );
    }
    inline void sendProcessedPointCloud(){
        std::shared_ptr<sensor_msgs::PointCloud2> pd_ros( new sensor_msgs::PointCloud2() );
        pcl::toROSMsg( *processed_pointcloud_, *pd_ros );
        pub_pointcloud_.publish( pd_ros );
    }

    inline geometry_msgs::PoseStamped getCurrWorld_Position() const{
        return curr_world_posi_;
    }

    inline Eigen::Matrix4d getCurrWorld_Camera() const{
        return curr_world_Camera_;
    }

    inline tf::StampedTransform getStampedTransform() const{
        return tf_map_base_;
    }

    inline void setCurrWorld_Position( const geometry_msgs::PoseStamped& pose){
        curr_world_posi_ = pose;
        curr_world_posi_.header.stamp = ros::Time::now();
        // curr_world_posi_.is_xy_valid = is_xy_valid_;
        // curr_world_posi_.is_z_valid = is_z_valid_;
        // curr_world_posi_.is_yaw_valid = is_yaw_valid_;

        pub_measured_posi_.publish( curr_world_posi_ );
    }

    inline void setCurrWorld_Posi_Velocity( const geometry_msgs::Pose& pose, const geometry_msgs::Twist& velocity ){
        curr_world_posi_.pose = pose;
        curr_world_posi_.header.stamp = ros::Time::now();
        // curr_world_posi_.is_xy_valid = is_xy_valid_;
        // curr_world_posi_.is_z_valid = is_z_valid_;
        // curr_world_posi_.is_yaw_valid = is_yaw_valid_;

        curr_world_vel_.twist = velocity;
        curr_world_vel_.header.stamp = ros::Time::now();
        // curr_world_vel_.is_xy_valid = is_v_xy_valid_;
        // curr_world_vel_.is_z_valid = is_v_z_valid_;
        // curr_world_vel_.is_yaw_valid = is_yaw_valid_; // not in use

        // common_msgs::MeasurementPosVel mea_pos_vel;
        // mea_pos_vel.position = curr_world_posi_;
        // mea_pos_vel.velocity = curr_world_vel_;
        // pub_measured_posi_vel_.publish(mea_pos_vel);
    }

    inline void setCurrWorld_Camera( const Eigen::Matrix4d& G ){
        curr_world_Camera_ = G;
    }

    inline void setStampedTransform( const tf::Transform& transform ){
        // don't use now()! Think about the synchronization problem.
        ros::Time pointcloud_stamp;
        pcl_conversions::fromPCL( raw_pointcloud_->header.stamp, pointcloud_stamp );

        tf_map_base_ = tf::StampedTransform( transform, pointcloud_stamp, "map", raw_pointcloud_->header.frame_id );
        tf_broadcaster_.sendTransform( tf_map_base_ );
    }

    inline double getSLAMResult_PrintDuration() const{
        return slam_result_print_duration_;
    } 
};





// class RGBD_SLAM : public SLAM_System{
// public:
//     explicit RGBD_SLAM(ros::NodeHandle& nh):SLAM_System(nh){
//         //ROS_INFO("inside rgbd_slam constructor");

//         // check the existence of parameters
//         if( !nh.hasParam("rgb_camera_K")
//                 || !nh.hasParam("depth_camera_K") ){
//             ROS_FATAL("RGBD_SLAM: RGB/Depth camera matrix non found in setting.");
//         }

//         double fx_rgb, fy_rgb, cx_rgb, cy_rgb;
//         nh.param<double>("rgb_camera_K/fx", fx_rgb, 525);
//         nh.param<double>("rgb_camera_K/fy", fy_rgb, 525);
//         nh.param<double>("rgb_camera_K/cx", cx_rgb, 319.5);
//         nh.param<double>("rgb_camera_K/cy", cy_rgb, 239.5);

//         double fx_depth, fy_depth, cx_depth, cy_depth;
//         nh.param<double>("depth_camera_K/fx", fx_depth, 525);
//         nh.param<double>("depth_camera_K/fy", fy_depth, 525);
//         nh.param<double>("depth_camera_K/cx", cx_depth, 319.5);
//         nh.param<double>("depth_camera_K/cy", cy_depth, 239.5);


//         m_K_rgb<<fx_rgb,    0,   cx_rgb,
//                     0,   fy_rgb, cy_rgb,
//                     0,      0,     1;
//         m_K_depth<<fx_depth,    0,   cx_depth,
//                       0,   fy_depth, cy_depth,
//                       0,      0,      1;

//         // load tf for depth registration
//         nh.param<bool>("is_depth_need_register", m_is_depth_need_register, true);
//         nh.param<float>("depth_tx", m_t_x, -0.059);
//         nh.param<float>("depth_ty", m_t_y, 0);
//         nh.param<float>("depth_tz", m_t_z, 0);

//         // depth parameters, pcl::PointXYZ is float
//         nh.param<float>("pd_min_depth", m_min_depth, 0.4);
//         nh.param<float>("pd_max_depth", m_max_depth, 12);
//         nh.param<float>("img_depth_scale", m_depth_scale, 1000);
//         m_depth_scale_recip = 1.0f / m_depth_scale;

//         // filters
//         nh.param<bool>("is_enable_passthrough_filter", m_is_enable_passthrough_filter, false);
//         nh.param<bool>("is_enable_downsample_filter", m_is_enable_downsample_filter, true);
//         nh.param<float>("pd_downsample_leaf_size", m_pd_downsample_leaf_size, 0.05);
//         nh.param<bool>("is_enable_radius_filter", m_is_enable_radius_filter, false);
//         nh.param<float>("pd_filter_radius", m_pd_filter_radius, 0.2);
//         nh.param<float>("pd_filter_radius_neighbour", m_pd_filter_radius_neighbour, 15);
//         nh.param<bool>("is_enable_stat_filter", m_is_enable_stat_filter, true);
//         nh.param<float>("pd_filter_mean_k", m_pd_filter_mean_k, 50);
//         nh.param<float>("pd_filter_stddev_mul", m_pd_filter_stddev_mul, 1);
//     }

// protected:
//     // they should be in the same resolution
//     sensor_msgs::ImageConstPtr m_msgRGB; // CV_8U
//     sensor_msgs::ImageConstPtr m_msgDepth; // CV_16U

//     cv::Mat m_registered_depth; // CV_32F

//     // calibration
//     Eigen::Matrix3f m_K_rgb, m_K_depth;
//     float m_t_x, m_t_y, m_t_z; // camera coordinate
//     bool m_is_depth_need_register;

//     // depth parameters, pcl::PointXYZ is float
//     float m_min_depth, m_max_depth;
//     float m_depth_scale, m_depth_scale_recip;

//     // filters
//     bool m_is_enable_passthrough_filter;

//     bool m_is_enable_downsample_filter;
//     float m_pd_downsample_leaf_size;

//     bool m_is_enable_radius_filter;
//     float m_pd_filter_radius;
//     float m_pd_filter_radius_neighbour;

//     bool m_is_enable_stat_filter;
//     float m_pd_filter_mean_k;
//     float m_pd_filter_stddev_mul;

// public:
//     void saveImages(const ros::TimerEvent& event);

//     virtual void registerDepth();
//     virtual void generateProcessedPointcloud();
//     virtual void processedPointcloudInPlace(pcl::PointCloud<pcl::PointXYZ> &pc);
//     virtual void testDepthRegistration();

//     virtual void prepareSLAM(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgDepth);
//     virtual void solveSLAM(const ros::TimerEvent& event)=0;
// };


} //namespace uwb_imu_fusion

#include "slam_abstract_impl.hpp"

#endif // SLAM_ABSTRACT_HPP