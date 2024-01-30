#ifndef UWB_LOCALIZATION_IMPL_H
#define UWB_LOCALIZATION_IMPL_H

#include <ros/ros.h>

#include "uwb_localization.h"
#include "uwb_loc_init.h"

namespace uwb_imu_fusion
{
using namespace std;

UWB_Localization::UWB_Localization( ros::NodeHandle& nh ) 
    : nh_( nh ), SLAM_System( nh )
{
    // load param
    nh_.param<double>( "slam_fps", slam_fps_, 0 );

    if( slam_fps_<20 ){ is_fix_fps_ = false; } 
    else { is_fix_fps_ = true; }

    nh_.param<bool>( "is_initialize_with_ceres", is_initialize_with_ceres_, true );

    // create UWB_Mobile
    if( nh_.getParam( std::string( "tag_id"), tag_id_ )) {
        ROS_INFO( "Tag ID (yaml): %d", tag_id_ );
    } else {
        ROS_ERROR( "Tag ID not found!" );
        return;
    }

    create_UWB_Tag( tag_id_ );
    ROS_INFO( "UWB_Mobile created." );

    // initialize by zero or Ceres
    if( true == is_initialize_with_ceres_ ){
        UWB_Loc_Init initializer( nh, tag_ );
        initializer.initializeMobileByCeres();
    } else {
        tag_->simpleInitializeEKF();
    }

    // initialize range info callback
    m_range_info_sub = nh_.subscribe( "/time_domain/full_range_info", 10, &UWB_Localization::rangeInfoCallback, this );
    print_timer_ = nh_.createTimer( ros::Duration( 1 ), &UWB_Localization::printLocalizationCallback, this );
}

void UWB_Localization::solveSLAM( const ros::TimerEvent &event ){
    if( tag_->getStablizationFlag() == false ) { return; } 
    else {
        if( is_fix_fps_ == true ) {
            setPositionVelocityWorldCurrNWU(position_, velocity_);
        }
    }
}


void UWB_Localization::rangeInfoCallback(const common_msgs::UWB_FullRangeInfo::ConstPtr &msg){
    //ROS_INFO("full range info received.");
    // 检查是否可以使用接收到的信息来更新滤波器，并且标签已经稳定
    if( tag_->filter3DUpdate( *msg ) && tag_->getStablizationFlag() == true ){
        // set measurement to uav_os
        // 如果可以更新滤波器且标签已稳定，则执行以下操作：

        // 设置测量位置信息到 UAV 操作系统
        position_.position.x = tag_->getPosition().point.x;
        position_.position.y = tag_->getPosition().point.y;
        position_.position.z = tag_->getPosition().point.z;
        position_.orientation.x = 0;
        position_.orientation.y = 0;
        position_.orientation.z = 0;
        position_.orientation.w = 1;

        // 设置测量速度信息到 UAV 操作系统
        velocity_.linear.x = tag_->getVelocity().point.x;
        velocity_.linear.y = tag_->getVelocity().point.y;
        velocity_.linear.z = tag_->getVelocity().point.z;
        velocity_.angular.x = tag_->getVelocity().point.x;
        velocity_.angular.y = tag_->getVelocity().point.y;
        velocity_.angular.z = tag_->getVelocity().point.z;

        // 如果不是固定帧率，则将位置和速度信息发送到 UAV 操作系统
        if( false == is_fix_fps_ ){
            setPositionVelocityWorldCurrNWU( position_, velocity_ );
        }
    } else {

    }
}

void UWB_Localization::printLocalizationCallback( const ros::TimerEvent &event ){
    if( tag_->getStablizationFlag() == false ) {
        ROS_INFO( "Not yet steady." );
        return;
    }

    // // print the SLAM result.
    // tag_->printPosition();
    // tag_->printVelocity();
    // tag_->printAccelerationBias();
    
    // print innovation threshold
    std::cout<<"innovation threshold: "<<tag_->getInnovationThreshold()<<std::endl;
    
    std::cout<<"--------------"<<std::endl;
}

void UWB_Localization::create_UWB_Tag( const int tag_id ) {
    // get anchor list from parameter server
    std::string param_key( "anchor_list" );

    if( !nh_.getParam( param_key, anchor_list_ )) {
        ROS_ERROR( "Can't find anchor list param." );
        return;
    }

    // get anchor position, build anchor_map
    for( int i = 0; i < anchor_list_.size(); i++ ){
        int anchor_id = anchor_list_.at( i );
        param_key = std::string( "anchor_" ) + to_string( anchor_id );

        std::vector<double> position;
        if( nh_.getParam( param_key, position ) && position.size() == 3 ) {
            // has valid position
            // set anchor, variance set to constant 1
            std::shared_ptr<UWB_Anchor> anchor( new UWB_Anchor( anchor_id, position.at(0), position.at(1), position.at(2) ));
            anchor_map_.insert( std::pair<int, std::shared_ptr<UWB_Anchor>>( anchor_id, anchor ));
        }
    }

    // create pointer
    tag_ = std::shared_ptr<UWB_Tag>( new UWB_Tag( tag_id, anchor_map_, nh_ ));
}

} //namespace uwb_imu_fusion

#endif // UWB_LOCALIZATION_IMPL_H