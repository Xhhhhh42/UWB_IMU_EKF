#ifndef UWB_LOC_INIT_H
#define UWB_LOC_INIT_H

#include <ros/ros.h>
// #include "slam/anchor_triangulation_error_term.hpp"
#include "uwb_node.h"
// #include "basic_function.hpp"

namespace uwb_imu_fusion
{

class UWB_Loc_Init
{
public:
    // this class should be constructed and destructed before creating the range info callback
    explicit UWB_Loc_Init( ros::NodeHandle& nh, std::shared_ptr<UWB_Tag> tag );

    ~UWB_Loc_Init(){
        ROS_INFO( "UWB Localization Initialization module is now destructed." );
    }

    // inline void ndbCallback(const common_msgs::UWB_FullNeighborDatabase::ConstPtr& msg){
    //     m_ndb = *msg;
    // }

    inline void spinOnce(){
        (( ros::CallbackQueue* )nh_.getCallbackQueue())->callAvailable();
    }

    bool initializeMobileByCeres();

    bool getValidNeighborDatabse();
    
    // void buildOptimizationProblem(const common_msgs::UWB_FullNeighborDatabase& ndb);
    
    bool solveOptimizationProblem();

private:
    ros::NodeHandle nh_;
    std::shared_ptr<ros::CallbackQueue> callback_queue_;
    std::shared_ptr<UWB_Tag> tag_;
    geometry_msgs::PointStamped tag_posi_;

    ros::Subscriber m_ndb_sub;

    // common_msgs::UWB_FullNeighborDatabase m_ndb;

    ceres::Problem m_problem;
};

} //namespace uwb_imu_fusion

#endif // UWB_LOC_INIT_H