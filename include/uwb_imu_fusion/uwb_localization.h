#ifndef UWB_LOCALIZATION_H
#define UWB_LOCALIZATION_H

#include "uwb_node.h"
#include "slam_abstract.h"
#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <nlink_parser/LinktrackAoaNodeframe0.h>

namespace uwb_imu_fusion
{

class UWB_Localization : public SLAM_System
{
public:
    explicit UWB_Localization( ros::NodeHandle& nh );

    void create_UWB_Tag( const int tag_id );

    // void rangeInfoCallback(const common_msgs::UWB_FullRangeInfo::ConstPtr& msg);

    void printLocalizationCallback( const ros::TimerEvent& event );

    void solveSLAM( const ros::TimerEvent& event );

    void tagframe0Callback( const nlink_parser::LinktrackAoaNodeframe0 &msg );

private:
    ros::NodeHandle nh_;
    ros::Subscriber m_range_info_sub;
    ros::Subscriber nlink_aoa_sub_;
    ros::Timer print_timer_;

    double slam_fps_;
    bool is_fix_fps_;
    bool is_initialize_with_ceres_;

    int tag_id_;
    std::shared_ptr<UWB_Tag> tag_;
    geometry_msgs::Pose position_;
    geometry_msgs::Twist velocity_;

    std::vector<int> anchor_list_;
    std::map<int, std::shared_ptr<UWB_Anchor> > anchor_map_;    
};

#include "uwb_localization_impl.hpp"

} //namespace uwb_imu_fusion

#endif // UWB_LOCALIZATION_H