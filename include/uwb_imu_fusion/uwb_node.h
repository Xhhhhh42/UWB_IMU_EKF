#ifndef UWB_IMU_FUSION_UWB_NODE_H
#define UWB_IMU_FUSION_UWB_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <map>

#include <nlink_parser/LinktrackAoaNodeframe0.h>

namespace uwb_imu_fusion
{

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

class UWB_Node
{ 

enum Node_Type {
    TAG,
    ANCHOR,
    MONITOR,
};

public:
    explicit UWB_Node( const int node_id ): node_id_( node_id ) {}

    inline int getNodeId() const {
        return node_id_;
    }

    inline geometry_msgs::PointStamped getPosition() const {
        return node_posi_;
    }

    inline geometry_msgs::PointStamped* getPositionPtr() {
        return &node_posi_;
    }

    inline geometry_msgs::PointStamped getVelocity() const {
        return node_vel_;
    }

    inline void setPosition( const double x, const double y, const double z ) 
    {
        node_posi_.header.stamp = ros::Time::now();
        node_posi_.point.x = x;
        node_posi_.point.y = y;
        node_posi_.point.z = z;
    }

    inline void setVelocity( const double vx, const double vy, const double vz )
    {
        node_vel_.header.stamp = ros::Time::now();
        node_vel_.point.x = vx;
        node_vel_.point.y = vy;
        node_vel_.point.z = vz;
    }

    void setNodeType( std::string node ) {
        if( node.empty() ) return;
        node_type_ = node == "TAG" ? TAG : node == "ANCHOR" ? ANCHOR : MONITOR;
    }

protected:
    int node_id_;
    Node_Type node_type_;
    geometry_msgs::PointStamped node_posi_;
    geometry_msgs::PointStamped node_vel_;
};


class UWB_Anchor : public UWB_Node 
{

public:
    explicit UWB_Anchor( const int node_id, const double x, const double y, const double z ) 
        : UWB_Node( node_id ) 
    {
        std::string node = "ANCHOR";
        setNodeType( node );
        setPosition( x, y, z );
    };
};


class UWB_Tag : public UWB_Node
{

public:
    explicit UWB_Tag( const int node_id, 
                      const std::map<int, std::shared_ptr<UWB_Anchor> > anchor_map,
                      ros::NodeHandle& nh );

    inline void setAccelerationBias( const double ax_bias, const double ay_bias, const double az_bias ){
        acc_bias_.header.stamp = ros::Time::now();
        acc_bias_.point.x = ax_bias;
        acc_bias_.point.y = ay_bias;
        acc_bias_.point.z = az_bias;
    }

    inline Eigen::MatrixXd getNineState_Covariance() const {
        return nine_state_cov_Mat_;
    }

    inline void setNineState_Covariance( const Eigen::MatrixXd P ){
        nine_state_cov_Mat_ = P;
    }

    inline Matrix6d getSixState_Covariance() const {
        return six_state_cov_Mat_;
    }
    inline void setSixState_Covariance( const Matrix6d state_covariance ){
        six_state_cov_Mat_ = state_covariance;
    }

    // inline common_msgs::UWB_FullRangeInfo getRangeInfoToAnchor(const int anchor_id) const{
    //     return m_ranges_to_anchors.find(anchor_id)->second;
    // }

    // inline double getRangeMeterToAnchor(const int anchor_id) const{
    //     return double(m_ranges_to_anchors.find(anchor_id)->second.precisionRangeMm) / 1000.0;
    // }

    // inline void setRangeInfoToAnchor(const common_msgs::UWB_FullRangeInfo& range_info){
    //     m_ranges_to_anchors[range_info.responderId] = range_info;
    // }

    inline bool getAnchorPosition(const int anchor_id, geometry_msgs::PointStamped& anchor_position) const {
        if(anchor_map_.find(anchor_id)!=anchor_map_.end()){
            anchor_position = anchor_map_.find(anchor_id)->second->getPosition();
            return true;
        } else {
            return false;
        }
    }

    inline int getAnchorList_Size() const{
        return anchor_map_.size();
    }


    // stablization of filter
    bool checkStablization();

    inline bool getStablization_Flag() const{
        return is_stablized_;
    }

    inline double getInnovation_Threshold() const{
        return innovation_threshold_;
    }

    // // EKF for position, velocity estimation
    // void navigationStateCallback(const common_msgs::NavigationState::ConstPtr& msg){
    //     m_uav_state = *msg;
        
    //     // threshold the acceleration to prevent filter divergence
    //     double acceleration_linear_threshold_xy = 3.0;
    //     double acceleration_linear_threshold_z = 2.0;
    //     if(m_uav_state.acceleration.linear.x>=acceleration_linear_threshold_xy){
    //         m_uav_state.acceleration.linear.x = acceleration_linear_threshold_xy;
    //     } else if(m_uav_state.acceleration.linear.y<=-acceleration_linear_threshold_xy){
    //         m_uav_state.acceleration.linear.y = -acceleration_linear_threshold_xy;
    //     }
    //     if(m_uav_state.acceleration.linear.y>=acceleration_linear_threshold_xy){
    //         m_uav_state.acceleration.linear.y = acceleration_linear_threshold_xy;
    //     } else if(m_uav_state.acceleration.linear.y<=-acceleration_linear_threshold_xy){
    //         m_uav_state.acceleration.linear.y = -acceleration_linear_threshold_xy;
    //     }
    //     if(m_uav_state.acceleration.linear.z>=acceleration_linear_threshold_z){
    //         m_uav_state.acceleration.linear.z = acceleration_linear_threshold_z;
    //     } else if(m_uav_state.acceleration.linear.z<=-acceleration_linear_threshold_z){
    //         m_uav_state.acceleration.linear.z = -acceleration_linear_threshold_z;
    //     }
        
    // }
    
    void simple_InitializeEKF();
    void full_InitializeEKF( const geometry_msgs::PointStamped& position );
    
    // dynamic innovation threshold, to avoid filter divergence
    void getInnovation();
    
    // bool filter3DUpdate(const common_msgs::UWB_FullRangeInfo& range_info);
    bool filter3DUpdate( const nlink_parser::LinktrackAoaNodeframe0 &msg );    
    // bool kalmanFilter_3DUpdate(const common_msgs::UWB_FullRangeInfo& range_info);
    bool kalmanFilter_3DUpdate( const nlink_parser::LinktrackAoaNodeframe0 &msg );
    // bool unscented_KalmanFilter_3DUpdate(const common_msgs::UWB_FullRangeInfo& range_info);
    bool unscented_KalmanFilter_3DUpdate( const nlink_parser::LinktrackAoaNodeframe0 &msg );

    // bool ekf_withAcc(const common_msgs::UWB_FullRangeInfo& range_info);
    bool ekf_withAcc( const nlink_parser::LinktrackAoaNodeframe0 &msg );
    // bool ukf_withAcc(const common_msgs::UWB_FullRangeInfo& range_info);
    bool ukf_withAcc( const nlink_parser::LinktrackAoaNodeframe0 &msg );


    // // print
    // inline void printPosition(){
    //     std::cout<<m_node_id<<":"<<std::endl;

    //     if(std::string("EKF")==filter_type_ || std::string("UKF")==filter_type_){
    //         std::cout<<"pose: "<<m_position.point.x<<", "<<m_position.point.y<<", "<<m_position.point.z
    //                 <<"   cov: "<<six_state_cov_Mat_(0,0)<<", "<<six_state_cov_Mat_(2,2)<<", "<<six_state_cov_Mat_(4,4)
    //                 <<std::endl;
    //     } else {
    //         std::cout<<"pose: "<<m_position.point.x<<", "<<m_position.point.y<<", "<<m_position.point.z
    //                 <<"   cov: "<<nine_state_cov_Mat_(0,0)<<", "<<nine_state_cov_Mat_(3,3)<<", "<<nine_state_cov_Mat_(6,6)
    //                 <<std::endl;
    //     }

    // }
    // inline void printVelocity(){
    //     if(std::string("EKF")==filter_type_ || std::string("UKF")==filter_type_){
    //         std::cout<<"velo: "<<m_velocity.point.x<<", "<<m_velocity.point.y<<", "<<m_velocity.point.z
    //                 <<"   cov: "<<six_state_cov_Mat_(1,1)<<", "<<six_state_cov_Mat_(3,3)<<", "<<six_state_cov_Mat_(5,5)
    //                 <<std::endl;
    //     } else {
    //         std::cout<<"velo: "<<m_velocity.point.x<<", "<<m_velocity.point.y<<", "<<m_velocity.point.z
    //                 <<"   cov: "<<nine_state_cov_Mat_(1,1)<<", "<<nine_state_cov_Mat_(4,4)<<", "<<nine_state_cov_Mat_(7,7)
    //                 <<std::endl;
    //     }
    // }
    // inline void printAccelerationBias(){
    //     if(std::string("EKF")==filter_type_ || std::string("UKF")==filter_type_){

    //     } else {
    //         std::cout<<"acc bias: "<<acc_bias_.point.x<<", "<<acc_bias_.point.y<<", "<<acc_bias_.point.z
    //                  <<"   cov: "<<nine_state_cov_Mat_(2,2)<<", "<<nine_state_cov_Mat_(5,5)<<", "<<nine_state_cov_Mat_(8,8)
    //                  <<std::endl;
    //     }
    // }

private:
    double tag_anchor_dis_;
    double tag_anchor_angle_;

    // stores last correct/accepted measurements
    // std::map<int, common_msgs::UWB_FullRangeInfo> m_ranges_to_anchors;
    ros::Time m_last_range_time;
    bool last_filter_succeed_;

    // provided by parameter server
    std::map<int, std::shared_ptr<UWB_Anchor> > anchor_map_;

    // provided by parameter server
    double kalman_sigma_a_;
    double m_snr_threshold;
    double innovation_threshold_;
    double m_tao_acc_sqrt;
    double m_tao_bias_sqrt;

    // check whether the filter is properly initialized and ready to use
    bool is_stablized_;
    std::vector<geometry_msgs::PointStamped> init_inspector_posi_array_;

    // variables for 6/9 state filter, including acceleration bias
    Matrix6d six_state_cov_Mat_; // covariance
    Eigen::MatrixXd nine_state_cov_Mat_; 
    geometry_msgs::PointStamped acc_bias_;
    
    // common_msgs::NavigationState m_uav_state;
    ros::Subscriber m_uav_state_sub;


    // parameters
    ros::NodeHandle nh_;
    std::string filter_type_;
    
    double init_inno_threshold_;
    double normal_inno_threshold_;
    double innovation_threshold__inc_on_succ;
    double innovation_threshold__inc_on_fail;

    double z_damping_factor_;
    double Q_scale_;
    double R_scale_;

    // UKF parameters
    double m_ukf_alpha, m_ukf_beta, m_ukf_kappa;

    // debug
    int m_counter;
};



} //namespace uwb_imu_fusion

#include "uwb_node_impl.hpp"

#endif // UWB_IMU_FUSION_UWB_NODE_H