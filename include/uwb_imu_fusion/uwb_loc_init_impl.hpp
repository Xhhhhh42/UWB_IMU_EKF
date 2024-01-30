#ifndef UWB_LOC_INIT_IMPL_HPP
#define UWB_LOC_INIT_IMPL_HPP

#include "uwb_loc_init.h"

namespace uwb_imu_fusion
{

UWB_Loc_Init::UWB_Loc_Init( ros::NodeHandle& nh, std::shared_ptr<UWB_Tag> tag )
    : tag_(tag), nh_(nh)
{
    tag_posi_ = tag_->getPosition();
    m_ndb_sub = nh_.subscribe( "/time_domain/NDB", 10, &UWB_Loc_Init::ndbCallback, this );
}

bool UWB_Loc_Init::initializeMobileByCeres(){
    getValidNeighborDatabse();
    buildOptimizationProblem(m_ndb);
    if( solveOptimizationProblem() ){
        tag_->fullInitializeEKF(m_ndb, tag_posi_);
        ROS_INFO("Innitialization with Ceres OK. %f, %f, %f",
                 tag_posi_.point.x,
                 tag_posi_.point.y,
                 tag_posi_.point.z);
        return true;
    } else {
        ROS_INFO("Fail to initialize with Ceres.");
        return false;
    }
}

bool UWB_Loc_Init::getValidNeighborDatabse(){
    const int anchor_number = tag_->getAnchorNumber();
    int valid_anchor_counter = 0;

    while(valid_anchor_counter<anchor_number){
        ros::Duration(1.0).sleep();
        spinOnce();

        valid_anchor_counter = 0;
        for(int i=0;i<m_ndb.numNeighborEntries;++i){
            geometry_msgs::PointStamped anchor_position;
            std::cout<<m_ndb.neighbors.at(i).nodeId<<", "<<m_ndb.neighbors.at(i).rangeMm<<std::endl;
            if(tag_->getAnchorPosition(m_ndb.neighbors.at(i).nodeId, anchor_position)
                    && m_ndb.neighbors.at(i).rangeMm/1000.0>0.3){
                valid_anchor_counter += 1;
            }
        }
        ROS_INFO("[Ceres Init] Total anchor: %d, ndb anchor: %d", anchor_number, valid_anchor_counter);
    }

    return true;
}

void UWB_Loc_Init::buildOptimizationProblem(const common_msgs::UWB_FullNeighborDatabase& ndb){
    ceres::LossFunction* loss_function = new ceres::CauchyLoss(1.0);
    for(int i=0;i<m_ndb.numNeighborEntries;++i){
        geometry_msgs::PointStamped anchor_position;
        if(tag_->getAnchorPosition(m_ndb.neighbors.at(i).nodeId, anchor_position)){
            ceres::CostFunction* cost_function = AnchorTriangulationErrorTerm::Create(anchor_position.point.x,
                                                                                             anchor_position.point.y,
                                                                                             anchor_position.point.z,
                                                                                             m_ndb.neighbors.at(i).rangeMm/1000.0,
                                                                                             m_ndb.neighbors.at(i).rangeErrorEstimate/1000.0*5);
            m_problem.AddResidualBlock(cost_function, loss_function,
                                       &(tag_posi_.point.x),
                                       &(tag_posi_.point.y),
                                       &(tag_posi_.point.z));
        }
    }
}

bool UWB_Loc_Init::solveOptimizationProblem(){
    ceres::Solver::Options options;
    options.max_num_iterations = 10000;
    options.minimizer_progress_to_stdout = false;
    options.linear_solver_type = ceres::DENSE_QR;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &m_problem, &summary);

    std::cout<<"\n"<<summary.BriefReport()<<"\n";
//    std::cout << "Initial cost: " << summary.initial_cost << "  Final cost: "<< summary.final_cost << '\n';

    return summary.IsSolutionUsable();

}

} //namespace uwb_imu_fusion

#endif // UWB_LOC_INIT_IMPL_HPP