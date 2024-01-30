#ifndef UWB_IMU_FUSION_UWB_NODE_IMPL_H
#define UWB_IMU_FUSION_UWB_NODE_IMPL_H

#include "uwb_node.h"

namespace uwb_imu_fusion
{
using namespace std;

UWB_Tag::UWB_Tag( const int node_id, 
                  const std::map<int, std::shared_ptr<UWB_Anchor> > anchor_map,
                  ros::NodeHandle& nh )
    : UWB_Node( node_id ),
      anchor_map_( anchor_map ),
      nh_( nh )
{
    // subscribe uav state
    // m_uav_state_sub = nh_.subscribe( "/uav_state/navigation_state", 10, &UWB_Tag::navigationStateCallback, this );

    kalman_sigma_a_ = 0.125;
    m_snr_threshold = -100;  // it is wrong when saturated!

    // general parameters
    nh_.param<std::string>( "filter_type", filter_type_, std::string("EKF"));
    nh_.param<double>( "z_damping_factor", z_damping_factor_, 1);
    nh_.param<double>( "Q_scale", Q_scale_, 1);
    nh_.param<double>( "R_scale", R_scale_, 1);
    nh_.param<double>( "initialization_innovation_threshold", init_inno_threshold_, 50);
    nh_.param<double>( "normal_innovation_threshold", normal_inno_threshold_, 2);
    nh_.param<double>( "innovation_threshold_inc_on_succ", innovation_threshold__inc_on_succ, 0);
    nh_.param<double>( "innovation_threshold_inc_on_fail", innovation_threshold__inc_on_fail, 0);
    nh_.param<double>( "tao_acc_sqrt", m_tao_acc_sqrt, 0.3);
    nh_.param<double>( "tao_bias_sqrt", m_tao_bias_sqrt, 0.001);

    // EKF parameters

    // UKF parameters
    nh_.param<double>("ukf_alpha", m_ukf_alpha, 0.1);
    nh_.param<double>("ukf_beta", m_ukf_beta, 2);
    nh_.param<double>("ukf_kappa", m_ukf_kappa, 0);

    is_stablized_ = false;
}


void UWB_Tag::simple_InitializeEKF(){
    setPosition( 10, 10, 0 );
    setVelocity( 0, 0, 0 );
    setAccelerationBias( 0,0,0 );
    setSixState_Covariance( Eigen::MatrixXd::Identity( 6, 6 ));
    setNineState_Covariance( Eigen::MatrixXd::Identity( 9, 9 ));

    // common_msgs::UWB_FullRangeInfo zero_range_info;
    for( std::map<int, std::shared_ptr<UWB_Anchor>>::iterator ite = anchor_map_.begin();
         ite != anchor_map_.end(); ++ite ) 
    {
        int anchor_id = ite->second->getNodeId();

        // zero_range_info.responderId = anchor_id;
        // zero_range_info.header.stamp = ros::Time::now();
        // setRangeInfoToAnchor(zero_range_info);
    }
    m_last_range_time = ros::Time::now();
}

void UWB_Tag::full_InitializeEKF( const geometry_msgs::PointStamped& position ){
    setPosition( position.point.x, position.point.y, position.point.z);
    setVelocity(0,0,0);
    setAccelerationBias(0,0,0);

    Eigen::MatrixXd six_cov = Eigen::MatrixXd::Identity(6,6);
    // set cov of vel
    six_cov( 1, 1 ) = 0.01;
    six_cov( 3, 3 ) = 0.01;
    six_cov( 5, 5 ) = 0.01;
    setSixState_Covariance( six_cov );

    Eigen::MatrixXd nine_cov = Eigen::MatrixXd::Identity(9,9);
    // set cov of vel
    nine_cov(1,1) = 0.01;
    nine_cov(4,4) = 0.01;
    nine_cov(7,7) = 0.01;
    // set cov of acc_bias
    nine_cov(2,2) = 1;
    nine_cov(5,5) = 1;
    nine_cov(8,8) = 1;
    setNineState_Covariance(nine_cov);

    // common_msgs::UWB_FullRangeInfo zero_range_info;
    // for(int i=0;i<ndb.numNeighborEntries;++i){
    //     int anchor_id = ndb.neighbors.at(i).nodeId;

    //     zero_range_info.header.stamp = ndb.header.stamp;
    //     zero_range_info.responderId = anchor_id;
    //     zero_range_info.precisionRangeMm = ndb.neighbors.at(i).rangeMm;
    //     zero_range_info.precisionRangeErrEst = ndb.neighbors.at(i).rangeErrorEstimate;

    //     // avoid invalid ndb entry
    //     if(zero_range_info.precisionRangeMm>50){
    //         setRangeInfoToAnchor(zero_range_info);
    //     }

    // }

    // m_last_range_time = ros::Time::now();
}


bool UWB_Tag::checkStablization() {
    // 如果标签已经稳定，则直接返回 true
    if( is_stablized_ == true ) { return true; }

    // 将标签最近的位置信息添加到初始化检查器位置数组中
    init_inspector_posi_array_.push_back( this->getPosition() );

    // get time nearby positions / 获取最近时间范围内的位置信息数组
    std::vector<geometry_msgs::PointStamped> valid_posi_array;

    for( int i = init_inspector_posi_array_.size() - 1; i >= 0; --i ) {
        geometry_msgs::PointStamped position = init_inspector_posi_array_.at(i);
        // 如果位置信息的时间戳与当前时间相差小于 5 秒，则将其添加到有效位置数组中
        if( ros::Time::now().toSec() - position.header.stamp.toSec() < 5 ) {
            valid_posi_array.push_back( position );
        } else { break; }
    }

    // 如果有效位置数组的大小小于 10，则返回 false
    if( valid_posi_array.size() < 10 ) { return false; } 
    else {
        // get x,y,z mean/计算有效位置数组中位置坐标的平均值
        float x_sum = 0;
        float y_sum = 0;
        float z_sum = 0;

        for( int i = 0; i < valid_posi_array.size(); ++i ){
            geometry_msgs::PointStamped position = valid_posi_array.at( i );
            x_sum += position.point.x;
            y_sum += position.point.y;
            z_sum += position.point.z;
        }
        float x_mean = x_sum / float( valid_posi_array.size() );
        float y_mean = y_sum / float( valid_posi_array.size() );
        float z_mean = z_sum / float( valid_posi_array.size() );

        // 计算位置坐标的标准差
        float squared_residual_error_sum_x = 0;
        float squared_residual_error_sum_y = 0;
        float squared_residual_error_sum_z = 0;

        for( int i = 0; i < valid_posi_array.size(); ++i ){
            geometry_msgs::PointStamped position = valid_posi_array.at(i);
            squared_residual_error_sum_x += std::pow( position.point.x-x_mean, 2 );
            squared_residual_error_sum_y += std::pow( position.point.y-y_mean, 2 );
            squared_residual_error_sum_z += std::pow( position.point.z-z_mean, 2 );
        }

        float sigma_x = std::sqrt( squared_residual_error_sum_x / float( valid_posi_array.size() ));
        float sigma_y = std::sqrt( squared_residual_error_sum_y / float( valid_posi_array.size() ));
        float sigma_z = std::sqrt( squared_residual_error_sum_z / float( valid_posi_array.size() ));

        // 输出标准差信息
        ROS_INFO_THROTTLE( 1.0, "[Check Init] Sigma in the last 5 second (x, y, z): %f, %f, %f", sigma_x, sigma_y, sigma_z );
        if( sigma_x < 0.5 && sigma_y < 0.5 && sigma_z < 0.5 ) {
            return true;
        } else {
            return false;
        }
    }
}


void UWB_Tag::getInnovation() {
    bool previous_stablized_flag = is_stablized_;
  
    if( is_stablized_ == false ){
        innovation_threshold_ = init_inno_threshold_;
        is_stablized_ = checkStablization();
    }else {
        if( previous_stablized_flag == false ) {
            innovation_threshold_ = normal_inno_threshold_;
        }

        // dynamic innovation
        if( last_filter_succeed_ == true ) {
            innovation_threshold_ += innovation_threshold__inc_on_succ;
        } else {
            innovation_threshold_ += innovation_threshold__inc_on_fail;
        }
        
        // upper and lower limit
        if( innovation_threshold_ < normal_inno_threshold_ ) {
            innovation_threshold_ = normal_inno_threshold_;
        } else if( innovation_threshold_ > init_inno_threshold_ ) {
            innovation_threshold_ = init_inno_threshold_;
        }
    }
}


// bool UWB_Tag::filter3DUpdate( const common_msgs::UWB_FullRangeInfo &range_info ){
//     if( filter_type_ == std::string( "EKF" )) {
//         return kalmanFilter_3DUpdate( range_info );
//     } else if( filter_type_ == std::string( "UKF" )) {
//         return unscented_KalmanFilter_3DUpdate( range_info );
//     } else if( filter_type_ == std::string( "EKF_Acc" )) {
//         return ekf_withAcc( range_info );
//     } else if( filter_type_ == std::string( "UKF_Acc" )) {
//         return ukf_withAcc( range_info );
//     }
//     else {
//         ROS_WARN( "Unknown filter type, use EKF_Acc now." );
//         return ekf_withAcc( range_info );
//     }
// }


bool UWB_Tag::filter3DUpdate( const nlink_parser::LinktrackAoaNodeframe0 &msg ) {
    if( filter_type_ == std::string( "EKF" )) {
        return kalmanFilter_3DUpdate( msg );
    } else if( filter_type_ == std::string( "UKF" )) {
        return unscented_KalmanFilter_3DUpdate( msg );
    } else if( filter_type_ == std::string( "EKF_Acc" )) {
        return ekf_withAcc( msg );
    } else if( filter_type_ == std::string( "UKF_Acc" )) {
        return ukf_withAcc( msg );
    }
    else {
        ROS_WARN( "Unknown filter type, use EKF_Acc now." );
        return ekf_withAcc( msg );
    }
}

bool UWB_Tag::kalmanFilter_3DUpdate( const nlink_parser::LinktrackAoaNodeframe0 &msg ) {
    // KALMANFILTER3D_UPDATE - updates a EKF with 6 state variables
    // Called each time a new mobile-to-anchor measurement is available

    // INPUTS:
    // X - the 6x1 a priori state vector - [x,xdot,y,ydot,z,zdot]'
    // P - the 6x6 a priori covariance matrix
    // T - the delta time between updates
    // sigma_r - the estimated standard deviation of the new range measurement
    // r_meas - the actual range measurement
    // XYZ0 - the [x y z] location of the associated anchor node

    // OUTPUTS:
    // X - the new state vector estimate
    // P - the new covariance matrix estimate
    // innovation - the difference between the range measurement and the estimated
    // range measurement (perhaps useful for outlier filtering.)


    int anchor_id = msg.id;
    // readingCounterInc(anchor_id); // for statistics, to evaluate valid frequency

    // determine whether to perform filter using SNR
    // According to experiments on 2017-03-24,
    // Time Domain UWB range info have incorrect vPeak when satuatured,
    // probably because of Int16 overflow
//    float SNR = 20 * std::log( range_info.vPeak / ( range_info.noise + 0.1) );
//    if(SNR < m_snr_threshold){
//        //ROS_WARN("Anchor %d SNR %f too small, discard.", anchor_id, SNR);
//        return false;
//    }

    // set innovation threshold
    getInnovation();
    // ------------------------------------------------------------------

    Vector6d X;
    X(0) = node_posi_.point.x;
    X(1) = node_vel_.point.x;
    X(2) = node_posi_.point.y;
    X(3) = node_vel_.point.y;
    X(4) = node_posi_.point.z;
    X(5) = node_vel_.point.z;

    Matrix6d P = getSixState_Covariance();

    double T = msg.system_time - m_last_range_time.toSec();
    
    if( T > 1 ) { T = 1; } 
    else if( T < 0 ) { T = 0.1; }

//    double T = range_info.header.stamp.toSec() - getRangeInfoToAnchor(anchor_id).header.stamp.toSec();
    // double sigma_r = double(range_info.precisionRangeErrEst) / 1000.0;
    double sigma_r = kalman_sigma_a_;
    double sigma_a = kalman_sigma_a_;
    // double r_meas = double(range_info.precisionRangeMm) / 1000.0;
    double r_meas = 0.1;

    Eigen::Vector3d XYZ0 = Eigen::VectorXd::Zero(3);
    geometry_msgs::PointStamped anchor_position = anchor_map_.find( anchor_id )->second->getPosition();
    XYZ0(0) = anchor_position.point.x;
    XYZ0(1) = anchor_position.point.y;
    XYZ0(2) = anchor_position.point.z;


    // F is a 6x6 State Transition Matrix
    Matrix6d F;
    F << 1,  T,  0,  0,  0,  0,
         0 , 1,  0,  0,  0,  0,
         0,  0,  1,  T,  0,  0,
         0,  0,  0,  1,  0,  0,
         0,  0,  0,  0,  1,  T,
         0,  0,  0,  0,  0,  1;

    // Q is the acceleration model
    Matrix6d Q;
    Q << std::pow(T,4)/3.0,  std::pow(T,3)/2.0,  0,                  0,                  0,                                    0,
         std::pow(T,3)/2.0,  std::pow(T,2),      0,                  0,                  0,                                    0,
         0,                  0,                  std::pow(T,4)/3.0,  std::pow(T,3)/2.0,  0,                                    0,
         0,                  0,                  std::pow(T,3)/2.0,  std::pow(T,2),      0,                                    0,                 
         0,                  0,                  0,                  0,                  std::pow(T,4)/3.0*z_damping_factor_,  std::pow(T,3)/2.0*z_damping_factor_,
         0,                  0,                  0,                  0,                  std::pow(T,3)/2.0*z_damping_factor_,  std::pow(T,2)*z_damping_factor_;
    Q *= std::pow( sigma_a, 2 );
    Q *= Q_scale_;

    // X is the predicted state vector and the predicted covariance matrix
    X = F * X;

    // M is the predicted covariance matrix
    Matrix6d M = F * P * F.transpose() + Q;

    // r_pred is the predicted range measure
    double r_pred = std::sqrt( std::pow( X(0) - XYZ0(0), 2 ) +
                               std::pow( X(2) - XYZ0(1), 2 ) +
                               std::pow( X(4) - XYZ0(2), 2 )) + 1e-6; // 添加了一个小的偏移量 1e-6，以避免除以零的情况
    // H is the linearized measurement matrix
    Eigen::MatrixXd H( 1, 6 );
    H << ( X(0) - XYZ0(0) ) / r_pred,
         0,
         ( X(2) - XYZ0(1) ) / r_pred,
         0,
         ( X(4) - XYZ0(2) ) / r_pred,
         0;

    // K is the Kalman Gain
    double R = std::pow( sigma_r, 2 ) * R_scale_;
    Vector6d K = M * H.transpose() / (( H * M * H.transpose() )( 0, 0 ) + R );

    // Update P for the a posteriori covariance matrix
    P = ( Eigen::MatrixXd::Identity( 6, 6 ) - K * H ) * M;

    // Return the measurement innovation
    double innovation = std::fabs( r_meas - r_pred );

    // Update the state
    X = X + K * ( r_meas - r_pred );

    // decide to take the range info or not.
    if( innovation < innovation_threshold_ ){
        setPosition(X(0),X(2),X(4));
        setVelocity(X(1),X(3),X(5));
        setSixState_Covariance(P);

        // setRangeInfoToAnchor(range_info);
        // m_last_range_time = range_info.header.stamp;

        // successfulReadingCounterInc(anchor_id); // for statistics, to evaluate valid frequency
        return true;
    } else {
        ROS_WARN( "Anchor id %d, Innovation too large: %f",  anchor_id, innovation );
        return false;
    }


}

bool UWB_Tag::unscented_KalmanFilter_3DUpdate( const nlink_parser::LinktrackAoaNodeframe0 &msg ){
    // KALMANFILTER3D_UPDATE - updates a UKF with 6 state variables
    // Called each time a new mobile-to-anchor measurement is available

    // INPUTS:
    // X - the 6x1 a priori state vector - [x,xdot,y,ydot,z,zdot]'
    // P - the 6x6 a priori covariance matrix
    // T - the delta time between updates
    // sigma_r - the estimated standard deviation of the new range measurement
    // r_meas - the actual range measurement
    // XYZ0 - the [x y z] location of the associated anchor node

    // OUTPUTS:
    // X - the new state vector estimate
    // P - the new covariance matrix estimate
    // innovation - the difference between the range measurement and the estimated
    // range measurement (perhaps useful for outlier filtering.)


    int anchor_id = msg.id;
    // readingCounterInc(anchor_id); // for statistics, to evaluate valid frequency

    // determine whether to perform filter using SNR
    // According to experiments on 2017-03-24,
    // Time Domain UWB range info have incorrect vPeak when satuatured,
    // probably because of Int16 overflow
//    float SNR = 20 * std::log( range_info.vPeak / ( range_info.noise + 0.1) );
//    if(SNR < m_snr_threshold){
//        //ROS_WARN("Anchor %d SNR %f too small, discard.", anchor_id, SNR);
//        return false;
//    }

    // set innovation threshold
    getInnovation();

    // ------------------------------------------------------------------

    Vector6d X;
    X(0) = node_posi_.point.x;
    X(1) = node_vel_.point.x;
    X(2) = node_posi_.point.y;
    X(3) = node_vel_.point.y;
    X(4) = node_posi_.point.z;
    X(5) = node_vel_.point.z;

    Matrix6d P = getSixState_Covariance();

    double T = msg.system_time - m_last_range_time.toSec();
    if(T>1){
        T = 1;
    } else if(T<0){
        T = 0.1;
    }
    double sigma_r = double(range_info.precisionRangeErrEst) / 1000.0;
    double sigma_a = kalman_sigma_a_;
    double r_meas = double(range_info.precisionRangeMm) / 1000.0;

    Eigen::Vector3d XYZ0 = Eigen::VectorXd::Zero(3);
    geometry_msgs::PointStamped anchor_position = anchor_map_.find(anchor_id)->second->getPosition();
    XYZ0(0) = anchor_position.point.x;
    XYZ0(1) = anchor_position.point.y;
    XYZ0(2) = anchor_position.point.z;


    // F is a 6x6 State Transition Matrix
    Matrix6d F;
    F << 1,  T,  0,  0,  0,  0,
         0 , 1,  0,  0,  0,  0,
         0,  0,  1,  T,  0,  0,
         0,  0,  0,  1,  0,  0,
         0,  0,  0,  0,  1,  T,
         0,  0,  0,  0,  0,  1;

    // Q is the acceleration model
    Matrix6d Q;
    Q << std::pow(T,4)/3.0,  std::pow(T,3)/2.0,  0,                  0,                  0,                  0,
         std::pow(T,3)/2.0,  std::pow(T,2),      0,                  0,                  0,                  0,
             0,              0,                  std::pow(T,4)/3.0,  std::pow(T,3)/2.0,  0,                  0,
             0,              0,                  std::pow(T,3)/2.0,  std::pow(T,2),      0,                  0,
             0,              0,                  0,                  0,                  std::pow(T,4)/3.0*z_damping_factor_,  std::pow(T,3)/2.0*z_damping_factor_,
             0,              0,                  0,                  0,                  std::pow(T,3)/2.0*z_damping_factor_,  std::pow(T,2)*z_damping_factor_;
    Q *= std::pow(sigma_a,2);
    Q *= Q_scale_;


    // ========================== UKF ==============================
    // dimension
    int L = X.rows();
    double ukf_lambda = m_ukf_alpha*m_ukf_alpha * (L+m_ukf_kappa) - L;
    double measurement_covariance = std::pow(sigma_r, 2) * R_scale_;

    // calculate sigma points ++++++++++++++++
    // first of all, compute square root of (L+lambda)P
    // here we can use SVD for matrix square root, because P is guaranteed to be sysmetric,
    // which is proven to be diagonalizable.
    Eigen::MatrixXd P_prime = (L+ukf_lambda)*P;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(P_prime, Eigen::ComputeFullV);
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::VectorXd S = svd.singularValues();
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(L,L);
    for(int i=0;i<L;++i){
        D(i,i) = std::sqrt(S(i));
    }
    Eigen::MatrixXd P_prime_sqrt = V*D*V.transpose();

    Eigen::MatrixXd X_sigma = Eigen::MatrixXd::Zero(L, 2*L+1);

    X_sigma.col(0) = X;
    for(int i=1;i<=L;++i){
        X_sigma.col(i) = X + P_prime_sqrt.row(i-1).transpose();
    }
    for(int i=L+1;i<=2*L;++i){
        X_sigma.col(i) = X - P_prime_sqrt.row(i-L-1).transpose();
    }

    Eigen::VectorXd W_m = Eigen::VectorXd::Constant(2*L+1, 1.0/(2*(L+ukf_lambda)));
    W_m(0) = ukf_lambda / (L+ukf_lambda);


    Eigen::VectorXd W_c = Eigen::VectorXd::Constant(2*L+1, 1.0/(2*(L+ukf_lambda)));
    W_c(0) = ukf_lambda / (L+ukf_lambda) + (1 - m_ukf_alpha*m_ukf_alpha + m_ukf_beta);


    // use additive (zero mean) noise case,
    // see The Unscented Kalman Filter by Eric A. Wan and Rudolph van der Merwe

    // time update ++++++++++++++++
    Eigen::MatrixXd X_sigma_bar = F * X_sigma;
    Eigen::VectorXd X_bar = X_sigma_bar * W_m;
    // calculate P_bar
    Eigen::MatrixXd P_bar = Q;
    for(int i=0;i<2*L+1;++i){
        P_bar += W_c(i) * (X_sigma_bar.col(i)-X_bar) * (X_sigma_bar.col(i)-X_bar).transpose();
    }
    // calculate Y_sigma_bar
    Eigen::VectorXd Y_sigma_bar = Eigen::VectorXd::Zero(2*L+1);
    for(int i=0;i<2*L+1;++i){
        Y_sigma_bar(i) = std::sqrt(   std::pow(X_sigma_bar(0, i)-XYZ0(0), 2)
                                    + std::pow(X_sigma_bar(2, i)-XYZ0(1), 2)
                                    + std::pow(X_sigma_bar(4, i)-XYZ0(2), 2)  );
    }
    double Y_bar = (Y_sigma_bar.transpose() * W_m)(0);

    // debug
//    std::cout<<"P_prime"<<std::endl<<P_prime<<std::endl;
//    std::cout<<"V"<<std::endl<<V<<std::endl;
//    std::cout<<"S"<<std::endl<<S<<std::endl;
//    std::cout<<"P_prime_sqrt"<<std::endl<<P_prime_sqrt<<std::endl;
//    std::cout<<"F"<<std::endl<<F<<std::endl;
//    std::cout<<"W_m"<<std::endl<<W_m.transpose()<<std::endl;
//    std::cout<<"X_sigma"<<std::endl<<X_sigma<<std::endl;
//    std::cout<<"X_bar"<<std::endl<<X_bar.transpose()<<std::endl;
//    std::cout<<"Y_sigma_bar"<<std::endl<<Y_sigma_bar.transpose()<<std::endl;
//    std::cout<<"Y_bar"<<std::endl<<Y_bar<<std::endl;

    // measurement update ++++++++++++++++
    // Pyy
    double Pyy = measurement_covariance;
    Eigen::ArrayXd Y_residual_suare = (Y_sigma_bar.array()-Y_bar).square();
    Pyy += W_c.transpose() * Y_residual_suare.matrix();

    // Pxy
    Eigen::VectorXd Pxy = Eigen::VectorXd::Zero(L);
    for(int i=0;i<2*L+1;++i){
        Pxy += W_c(i) * (X_sigma_bar.col(i)-X_bar) * (Y_sigma_bar(i)-Y_bar);
    }

    Eigen::VectorXd K = Pxy / Pyy;
    X = X_bar + K * (r_meas - Y_bar);

    P = P_bar - K*Pyy*K.transpose();
    // ========================== UKF ==============================


    // Return the measurement innovation
    double innovation = std::fabs(r_meas - Y_bar);

    // decide to take the range info or not.
    if(innovation<innovation_threshold_){
        setPosition(X(0),X(2),X(4));
        setVelocity(X(1),X(3),X(5));
        setSixState_Covariance(P);

        // setRangeInfoToAnchor(range_info);
        m_last_range_time = range_info.header.stamp;

        // successfulReadingCounterInc(anchor_id); // for statistics, to evaluate valid frequency
        return true;
    } else {
        ROS_WARN("Anchor id %d, Innovation too large: %f",  anchor_id, innovation);
        return false;
    }
}


bool UWB_Tag::ekf_withAcc( const nlink_parser::LinktrackAoaNodeframe0 &msg ){
    // KALMANFILTER3D_UPDATE - updates a EKF with 9 state variables
    // Called each time a new mobile-to-anchor measurement is available

    // INPUTS:
    // X - the 9x1 a priori state vector - [x,xdot,ax_bias,y,ydot,ay_bias,z,zdot,az_bias]'
    // P - the 9x9 a priori covariance matrix
    // T - the delta time between updates
    // tao_acc - variance of acceleratmetor's gaussian noise
    // tao_bias - variance of acceleratmetor bias's noise
    // sigma_r - the estimated standard deviation of the new range measurement
    // r_meas - the actual range measurement
    // XYZ0 - the [x y z] location of the associated anchor node


    // OUTPUTS:
    // X - the new state vector estimate
    // P - the new covariance matrix estimate
    // innovation - the difference between the range measurement and the estimated
    // range measurement (perhaps useful for outlier filtering.)


    int anchor_id = msg.id;
    // readingCounterInc(anchor_id); // for statistics, to evaluate valid frequency

    // determine whether to perform filter using SNR
    // According to experiments on 2017-03-24,
    // Time Domain UWB range info have incorrect vPeak when satuatured,
    // probably because of Int16 overflow
//    float SNR = 20 * std::log( range_info.vPeak / ( range_info.noise + 0.1) );
//      if(SNR < m_snr_threshold){
//          //ROS_WARN("Anchor %d SNR %f too small, discard.", anchor_id, SNR);
//          return false;
//      }

    // set innovation threshold
    getInnovation();
    
    // ------------------------------------------------------------------

    Eigen::VectorXd X(9);
    X(0) = node_posi_.point.x;
    X(1) = node_vel_.point.x;
    X(2) = acc_bias_.point.x;
    X(3) = node_posi_.point.y;
    X(4) = node_vel_.point.y;
    X(5) = acc_bias_.point.y;
    X(6) = node_posi_.point.z;
    X(7) = node_vel_.point.z;
    X(8) = acc_bias_.point.z;

    Eigen::MatrixXd P = getNineState_Covariance();

    double T = msg.system_time - m_last_range_time.toSec();
    if(T>1){
        T = 1;
    } else if(T<0){
        T = 0.1;
    }
    double sigma_r = double(range_info.precisionRangeErrEst) / 1000.0;
    double sigma_a = kalman_sigma_a_;
    double r_meas = double(range_info.precisionRangeMm) / 1000.0;

    Eigen::Vector3d XYZ0 = Eigen::VectorXd::Zero(3);
    geometry_msgs::PointStamped anchor_position = anchor_map_.find(anchor_id)->second->getPosition();
    XYZ0(0) = anchor_position.point.x;
    XYZ0(1) = anchor_position.point.y;
    XYZ0(2) = anchor_position.point.z;


    // F is a 9x9 State Transition Matrix
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(9,9);
    Eigen::MatrixXd block_F(3,3);
    block_F << 1, T, -T*T/2.0,
               0, 1, -T,
               0, 0, 1;
    F.block<3,3>(0,0) = block_F;
    F.block<3,3>(3,3) = block_F;
    F.block<3,3>(6,6) = block_F;

    // Q is the acceleration model
    double tao_acc = m_tao_acc_sqrt * m_tao_acc_sqrt;
    double tao_bias = m_tao_bias_sqrt * m_tao_bias_sqrt;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(9,9);
    Eigen::MatrixXd block_Q(3,3);
    block_Q <<
         std::pow(T,3)/3.0*tao_acc+std::pow(T,5)/20.0*tao_bias,  std::pow(T,2)/2*tao_acc+std::pow(T,4)/8.0*tao_bias,  -std::pow(T,3)/6*tao_bias,
         std::pow(T,2)/2.0*tao_acc+std::pow(T,4)/8.0*tao_bias ,  T*tao_acc+std::pow(T,3)/3*tao_bias                ,  -std::pow(T,2)/2*tao_bias,
         -std::pow(T,3)/6.0*tao_bias                          ,  -std::pow(T,2)/2*tao_bias                         ,  T*tao_bias               ;
    Q.block<3,3>(0,0) = block_Q;
    Q.block<3,3>(3,3) = block_Q;
    Q.block<3,3>(6,6) = block_Q * z_damping_factor_;
    Q *= Q_scale_;

    // time update
    Eigen::VectorXd u(9);
    u << m_uav_state.acceleration.linear.x, 0, 0,
         m_uav_state.acceleration.linear.y, 0, 0,
         m_uav_state.acceleration.linear.z, 0, 0;
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(9,9);
    Eigen::MatrixXd block_B(3,3);
    block_B << T*T/2.0,  0,  0,
               T      ,  0,  0,
               0      ,  0,  0;
    B.block<3,3>(0,0) = block_B;
    B.block<3,3>(3,3) = block_B;
    B.block<3,3>(6,6) = block_B;


    // time update
    // X is the predicted state vector and the predicted covariance matrix
    X = F*X + B * u;

    // M is the predicted covariance matrix
    Eigen::MatrixXd M = F*P*F.transpose() + Q;

    // r_pred is the predicted range measure
    double r_pred = std::sqrt( std::pow(X(0) - XYZ0(0), 2) +
                               std::pow(X(3) - XYZ0(1), 2) +
                               std::pow(X(6) - XYZ0(2), 2) ) + 1e-9;
    // H is the linearized measurement matrix
    Eigen::MatrixXd H(1,9);
    H << (X(0) - XYZ0(0))/r_pred,
         0,
         0,
         (X(3) - XYZ0(1))/r_pred,
         0,
         0,
         (X(6) - XYZ0(2))/r_pred,
         0,
         0;

    // K is the Kalman Gain
    double R = std::pow(sigma_r,2) * R_scale_;
    Eigen::VectorXd K = M*H.transpose() / ( (H*M*H.transpose())(0,0) + R );

    // Update P for the a posteriori covariance matrix
    P = ( Eigen::MatrixXd::Identity(9,9) - K*H ) * M;

    // Return the measurement innovation
    double innovation = std::fabs(r_meas - r_pred);

    // Update the state
    X = X + K * (r_meas - r_pred);




    // decide to take the range info or not.
    if(innovation<innovation_threshold_){
        setPosition(X(0),X(3),X(6));
        setVelocity(X(1),X(4),X(7));
        setAccelerationBias(X(2),X(5),X(8));
        setNineState_Covariance(P);

        // setRangeInfoToAnchor(range_info);
        m_last_range_time = range_info.header.stamp;

        // successfulReadingCounterInc(anchor_id); // for statistics, to evaluate valid frequency
        last_filter_succeed_ = true;
        return true;
    } else {
        last_filter_succeed_ = false;
        ROS_WARN("Anchor id %d, Innovation too large: %f",  anchor_id, innovation);
        return false;
    }

}

}


#endif // UWB_IMU_FUSION_UWB_NODE_IMPL_H