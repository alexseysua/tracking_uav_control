#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <fw_control/Dynamics.h>
#include <fw_control/CameraFeatures.h>
#include <fw_control/EstimateOutput.h>

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <deque>

using namespace std;

/******************************* Data **************************************/
typedef struct
{
    double roll;
    double pitch;
    double yaw;
}rpy;

typedef struct
{
    double fx;
    double fy;
    double cu;
    double cv;
    double u;
    double v;
    bool isDetected;
}cam;

// under camera frame
typedef struct
{
    Eigen::Vector3d car_velcam;
    Eigen::Vector3d uav_velcam;
    Eigen::Vector3d uav_wvelcam;
    Eigen::Vector3d pt_wvelcam;
    Eigen::Vector3d camuav_pose;
    Eigen::Vector3d carcam_pose;
    double depth;
    Eigen::Vector3d car_velgnd;
    Eigen::Vector3d cam_posegnd;
    Eigen::Vector3d car_posegnd;
    Eigen::Matrix3d rot_g2c;
    Eigen::Matrix3d rot_c2g;
    Eigen::Vector3d trans_g2c;
    Eigen::Vector3d trans_c2g;
}dynamics_element;

int callback_spin_count = 0;
bool measurement_flag = true;
bool ibvs_mode = false;
bool ukf_mode = false;

/******************************* ROS **************************************/
int loop_rate = 80;
cam cam1;
dynamics_element dyn1;

void dynamics_cb(const fw_control::Dynamics::ConstPtr& msg)
{
    dyn1.car_velcam(0) = msg->target_vel.x;
    dyn1.car_velcam(1) = msg->target_vel.y;
    dyn1.car_velcam(2) = msg->target_vel.z;
    dyn1.uav_velcam(0) = msg->uav_vel.x;
    dyn1.uav_velcam(1) = msg->uav_vel.y;
    dyn1.uav_velcam(2) = msg->uav_vel.z;
    dyn1.uav_wvelcam(0) = msg->uav_wvel.x;
    dyn1.uav_wvelcam(1) = msg->uav_wvel.y;
    dyn1.uav_wvelcam(2) = msg->uav_wvel.z;
    dyn1.pt_wvelcam(0) = msg->pt_wvel.x;
    dyn1.pt_wvelcam(1) = msg->pt_wvel.y;
    dyn1.pt_wvelcam(2) = msg->pt_wvel.z;
    dyn1.camuav_pose(0) = msg->camuav_pose.x;
    dyn1.camuav_pose(1) = msg->camuav_pose.y;
    dyn1.camuav_pose(2) = msg->camuav_pose.z;
    // for x3
    dyn1.carcam_pose(0) = msg->targetcam_pose.x;
    dyn1.carcam_pose(1) = msg->targetcam_pose.y;
    dyn1.carcam_pose(2) = msg->targetcam_pose.z;

    dyn1.depth = msg->depth.data;
    
    dyn1.car_velgnd(0) = msg->target_vel_inertial.x;
    dyn1.car_velgnd(1) = msg->target_vel_inertial.y;
    dyn1.car_velgnd(2) = msg->target_vel_inertial.z;
    // measure
    dyn1.cam_posegnd(0) = msg->cam_pose_inertial.x;
    dyn1.cam_posegnd(1) = msg->cam_pose_inertial.y;
    dyn1.cam_posegnd(2) = msg->cam_pose_inertial.z;

    dyn1.car_posegnd(0) = msg->target_pose_inertial.x;
    dyn1.car_posegnd(1) = msg->target_pose_inertial.y;
    dyn1.car_posegnd(2) = msg->target_pose_inertial.z;

    dyn1.rot_g2c << msg->rot_g2c.data[0], msg->rot_g2c.data[1], msg->rot_g2c.data[2],
                    msg->rot_g2c.data[3], msg->rot_g2c.data[4], msg->rot_g2c.data[5],
                    msg->rot_g2c.data[6], msg->rot_g2c.data[7], msg->rot_g2c.data[8];
    dyn1.rot_c2g << msg->rot_c2g.data[0], msg->rot_c2g.data[1], msg->rot_c2g.data[2],
                    msg->rot_c2g.data[3], msg->rot_c2g.data[4], msg->rot_c2g.data[5],
                    msg->rot_c2g.data[6], msg->rot_c2g.data[7], msg->rot_c2g.data[8];
    
    dyn1.trans_g2c(0) = msg->trans_g2c.x;
    dyn1.trans_g2c(1) = msg->trans_g2c.y;
    dyn1.trans_g2c(2) = msg->trans_g2c.z;
    dyn1.trans_c2g(0) = msg->trans_c2g.x;
    dyn1.trans_c2g(1) = msg->trans_c2g.y;
    dyn1.trans_c2g(2) = msg->trans_c2g.z;
}

void features_cb(const fw_control::CameraFeatures::ConstPtr& msg)
{
    cam1.fx = msg->fx.data;
    cam1.fy = msg->fy.data;
    cam1.cu = msg->cu.data;
    cam1.cv = msg->cv.data;
    cam1.u = msg->u.data;
    cam1.v = msg->v.data;
    cam1.isDetected = msg->isDetected.data;
}

/******************************* UKF **************************************/
void initialize();
void predict();
void correct(Eigen::VectorXd measure);
Eigen::MatrixXd state_to_measurement(Eigen::MatrixXd sigma_state);
Eigen::MatrixXd dynamics(Eigen::MatrixXd sigma_state);
void noise_estimate(int window_size);

int x_size, y_size, x_sigmavector_size, y_sigmavector_size;

Eigen::VectorXd x;        // states
Eigen::VectorXd y;        // measurements
Eigen::VectorXd x_hat;    // states mean
Eigen::VectorXd y_hat;    // measurements mean
Eigen::VectorXd x_hat_;   // without noise
Eigen::VectorXd y_hat_;   // without noise

double dt;
double L;
double alpha;
double kappa;
double beta;
double lambda;

Eigen::VectorXd w_c;     // weight cov 
Eigen::VectorXd w_m;     // weight 

Eigen::MatrixXd x_sigmavector;
Eigen::MatrixXd y_sigmavector;
Eigen::MatrixXd x_sigmavector_;  // without noise
Eigen::MatrixXd y_sigmavector_;  // without noise
Eigen::MatrixXd H;       // measurment model transform
Eigen::MatrixXd P;       // covariance matrix
Eigen::MatrixXd Q;       // covarriance of process noise
Eigen::MatrixXd R;       // covarriance of measurement noise
Eigen::MatrixXd P_;       // without process noise covariance

Eigen::VectorXd q;       // process noise
Eigen::VectorXd r;       // measurement noise
deque<Eigen::VectorXd> q_window, r_window;
deque<Eigen::MatrixXd> Q_window, R_window;
deque<double> w_window;

Eigen::MatrixXd P_yy;
Eigen::MatrixXd P_xy;
Eigen::MatrixXd P_yy_;   // without measurement noise covariance

Eigen::MatrixXd Kalman_gain;


enum state{
    member_x1 = 0,
    member_x2,
    //member_x3,
    member_xq,
    member_yq,
    member_zq,
    member_vqx,
    member_vqy,
    member_vqz,
    statesize
};

enum measurement{
    member_mu = 0,
    member_mv,
    //member_ma,
    member_mxc,
    member_myc,
    member_mzc,
    measurementsize
};


Eigen::MatrixXd dynamics(Eigen::MatrixXd sigma_state)
{
    Eigen::MatrixXd predict_sigma_state(x_size, x_sigmavector_size);

    for(int i=0; i < x_sigmavector_size; i++)
    {

        double x1 = sigma_state(member_x1, i);
        double x2 = sigma_state(member_x2, i);
        //double x3 = sigma_state(member_x3, i);
        //double x3 = 1 / dyn1.carcam_pose(2);
        double x3 = 1 / dyn1.depth;
        
        Eigen::VectorXd rq;
        Eigen::VectorXd vq_gnd;
        Eigen::Vector3d vq_cam;
        rq.setZero(3);
        vq_gnd.setZero(3);

        rq << sigma_state(member_xq, i), sigma_state(member_yq, i), sigma_state(member_zq, i);
        vq_gnd << sigma_state(member_vqx, i), sigma_state(member_vqy, i), sigma_state(member_vqz, i);
        vq_cam = dyn1.rot_g2c * vq_gnd;
        
        double x1_ ;
        double x2_ ;
        double x3_ ;
        Eigen::VectorXd rq_;
        rq_.setZero(3);
        Eigen::VectorXd vq_gnd_;
        vq_gnd_.setZero(3);

        x1_ = x1 + ( vq_cam(0)*x3 - vq_cam(2)*x1*x3 + (dyn1.uav_velcam(2)*x1 - dyn1.uav_velcam(0))*x3 + dyn1.pt_wvelcam(0)*x1*x2 - dyn1.pt_wvelcam(1) - dyn1.pt_wvelcam(1)*x1*x1 + dyn1.pt_wvelcam(2)*x2 + dyn1.uav_wvelcam(0)*x1*(x2+dyn1.camuav_pose(1)*x3) + dyn1.uav_wvelcam(1)*(-(x3*dyn1.camuav_pose(2)+1)-x1*(x3*dyn1.camuav_pose(0)+x1)) + dyn1.uav_wvelcam(2)*(x3*dyn1.camuav_pose(1)+x2)  ) * dt;
        x2_ = x2 + ( vq_cam(1)*x3 - vq_cam(2)*x2*x3 + (dyn1.uav_velcam(2)*x2 - dyn1.uav_velcam(1))*x3 + dyn1.pt_wvelcam(0) + dyn1.pt_wvelcam(0)*x2*x2 - dyn1.pt_wvelcam(1)*x1*x2 - dyn1.pt_wvelcam(2)*x1 + dyn1.uav_wvelcam(0)*(x3*dyn1.camuav_pose(2)+1+x2*(x3*dyn1.camuav_pose(1)+x2)) + dyn1.uav_wvelcam(1)*(-x2*(x3*dyn1.camuav_pose(0)+x1)) + dyn1.uav_wvelcam(2)*(-x3*dyn1.camuav_pose(0)+x1)   ) * dt;
        //x3_ = x3 + (-vq_cam(2)*x3*x3 + camera_vel(2)*x3*x3 - (camera_wvel(1)*x1 - camera_wvel(0)*x2)*x3)*dt;
        //x3_ = x3 + ( -vq_cam(2)*x3*x3 + dyn1.uav_velcam(2)*x3*x3 + dyn1.pt_wvelcam(0)*x2*x3 - dyn1.pt_wvelcam(1)*x1*x3 + dyn1.uav_wvelcam(0)*(x3*x3*dyn1.camuav_pose(1) + x2*x3) + dyn1.pt_wvelcam(1)*(-x3*x3*dyn1.camuav_pose(0)-x1*x3) ) * dt;
        rq_ = rq + vq_gnd * dt;
        vq_gnd_ = vq_gnd;

        //ROS_INFO("vq: x:%.3f y:%.3f z:%.3f",vq_(0),vq_(1),vq_(2));

        predict_sigma_state(member_x1, i) =  x1_;
        predict_sigma_state(member_x2, i) =  x2_;
        //predict_sigma_state(member_x3, i) =  x3_;
        predict_sigma_state(member_xq, i) =  rq_(0);
        predict_sigma_state(member_yq, i) =  rq_(1);
        predict_sigma_state(member_zq, i) =  rq_(2);
        predict_sigma_state(member_vqx, i) =  vq_gnd_(0);
        predict_sigma_state(member_vqy, i) =  vq_gnd_(1);
        predict_sigma_state(member_vqz, i) =  vq_gnd_(2);
    }

    //ROS_INFO("x1:%.3f x2:%.3f x3:%.3f, z:%.3f",sigma_state(member_x1,0),sigma_state(member_x2,0),sigma_state(member_x3,0),1/sigma_state(member_x3,0));
    //ROS_INFO("x1_:%.3f x2_:%.3f x3_:%.3f z:%.3f",predict_sigma_state(member_x1,0),predict_sigma_state(member_x2,0),predict_sigma_state(member_x3,0),1/predict_sigma_state(member_x3,0));
    //ROS_INFO("xq:%.3f yq:%.3f zq:%.3f",sigma_state(member_xq,0),sigma_state(member_yq,0),sigma_state(member_zq,0));
    //ROS_INFO("vqx_:%.3f vqy_:%.3f vqz_:%.3f",predict_sigma_state(member_vqx,0),predict_sigma_state(member_vqy,0),predict_sigma_state(member_vqz,0));

    return predict_sigma_state;
}

//////////////////////////// ukf intialize
void initialize()
{
    x_size = statesize;
    y_size = measurementsize;
    L = x_size;
    alpha = 1e-3;
    kappa = 0;
    beta = 2;

    lambda = alpha * alpha * (L + kappa) - L;
    x_sigmavector_size = 2 * L + 1;

    x.setZero(x_size);
    y.setZero(y_size);
    x_hat.setZero(x_size);
    y_hat.setZero(y_size);

    x_sigmavector.setZero(x_size, x_sigmavector_size);
    y_sigmavector.setZero(y_size, y_sigmavector_size);

    H.setZero(y_size, x_size);

    w_c.setZero(x_sigmavector_size);
    w_m.setZero(x_sigmavector_size);
    w_c(0) = (lambda / (L + lambda)) + (1 - alpha*alpha + beta);
    w_m(0) = lambda / (L + lambda);

    for(int i = 1; i < x_sigmavector_size; i++)
    {
        w_c(i) = 1 / (2 * (L + lambda));
        w_m(i) = 1 / (2 * (L + lambda));
    }

    Q = 5e-7 * Eigen::MatrixXd::Identity(x_size, x_size);
    R = 5e-4 * Eigen::MatrixXd::Identity(y_size, y_size);
    P = 1e-3 * Eigen::MatrixXd::Identity(x_size, x_size);

    q.setZero(x_size);
    r.setZero(y_size);

    P_.setZero(x_size, x_size);
    P_yy.setZero(y_size, y_size);
    P_xy.setZero(x_size, y_size);
    P_yy_.setZero(y_size, y_size);
}

//////////////////////////// time update, predict states
void predict()
{
    // calculate sigma point
    P = (L + lambda) * P;
    Eigen::MatrixXd M = (P).llt().matrixL();
    Eigen::VectorXd sigma;

    x_sigmavector.col(0) = x;
    for(int i = 0; i < x_size; i++)
    {
        sigma = (M.row(i)).transpose();
        x_sigmavector.col( i+1 ) = x + sigma;
        x_sigmavector.col( i+x_size+1 ) = x - sigma;
    }

    x_sigmavector_ = dynamics(x_sigmavector);
    x_sigmavector = x_sigmavector_ ;//+ q * Eigen::MatrixXd::Constant(1, x_sigmavector_size, 1);

    // mean
    x_hat_.setZero(x_size);
    x_hat.setZero(x_size);
    for(int i = 0; i < x_sigmavector_size; i++)
    {
        x_hat_ += w_m(i) * x_sigmavector_.col(i);
    }
    // add process noise
    x_hat = x_hat_;// + q;

    // covariance
    P_.setZero(x_size, x_size);
    P.setZero(x_size, x_size);
    for(int i = 0; i < x_sigmavector_size; i++)
    {
        P_ += w_c(i) * (x_sigmavector.col(i) - x_hat) * ( (x_sigmavector.col(i) - x_hat).transpose() );
    }
    // add process noise covariance
    P = P_ + Q;

    y_sigmavector_ = state_to_measurement(x_sigmavector);
    y_sigmavector = y_sigmavector_; // + r * Eigen::MatrixXd::Constant(1, x_sigmavector_size, 1);
 
    // mean
    y_hat_.setZero(y_size);
    y_hat.setZero(y_size);
    for(int i = 0; i < x_sigmavector_size; i++)
    {
        y_hat_ += w_m(i) * y_sigmavector_.col(i);
    }
    // add measurement noise
    y_hat = y_hat_;// + r;

    //cout << endl << "prediced data" << endl;
    //cout << "x mean: " << endl << x_hat << endl;
    cout << "y mean: " << endl << y_hat << endl;
}

//////////////////////////// predict measurements
Eigen::MatrixXd state_to_measurement(Eigen::MatrixXd sigma_state)
{
    y_size = measurementsize;
    x_sigmavector_size = 2 * statesize + 1;
    Eigen::MatrixXd predict_sigma_measure(y_size, x_sigmavector_size);
    Eigen::Vector3d rel_pose, rel_posec2g;
    //double x3 = 1 / dyn1.carcam_pose(2);
    double x3 = 1 / dyn1.depth;

    for(int i = 0; i < x_sigmavector_size; i++)
    {
        //rel_pose << sigma_state(member_x1, i) / sigma_state(member_x3, i), sigma_state(member_x2, i) / sigma_state(member_x3, i), 1 / sigma_state(member_x3, i);
        rel_pose << sigma_state(member_x1, i) / x3, sigma_state(member_x2, i) / x3, 1 / x3;
        rel_posec2g = dyn1.rot_c2g * rel_pose;
        //cout << "rot g2c: " << endl << dyn1.rot_g2c << endl;
        //cout << "rot g2c trans: " << endl << (dyn1.rot_g2c).transpose() << endl;

        predict_sigma_measure(member_mu, i) = cam1.fx * sigma_state(member_x1, i) + cam1.cu;
        predict_sigma_measure(member_mv, i) = cam1.fy * sigma_state(member_x2, i) + cam1.cv;
        predict_sigma_measure(member_mxc, i) = sigma_state(member_xq, i) - rel_posec2g(0);
        predict_sigma_measure(member_myc, i) = sigma_state(member_yq, i) - rel_posec2g(1);
        predict_sigma_measure(member_mzc, i) = sigma_state(member_zq, i) - rel_posec2g(2);
        //ROS_INFO("rel_pose: x:%.3f y:%.3f z:%.3f",rel_pose(0),rel_pose(1),rel_pose(2));
        //ROS_INFO("rel_pose: x:%.3f y:%.3f z:%.3f",sigma_state(member_xq, i),sigma_state(member_yq, i),sigma_state(member_zq, i));
        //ROS_INFO("rel_pose: x:%.3f y:%.3f z:%.3f",predict_sigma_measure(member_mxc, i),predict_sigma_measure(member_myc, i),predict_sigma_measure(member_mzc, i));
    }

    ROS_INFO("rel_pose cam: x:%.3f y:%.3f z:%.3f",rel_pose(0),rel_pose(1),rel_pose(2));
    ROS_INFO("rel_pose gnd: x:%.3f y:%.3f z:%.3f",rel_posec2g(0),rel_posec2g(1),rel_posec2g(2));
    ROS_INFO("car_pose gnd: x:%.3f y:%.3f z:%.3f",sigma_state(member_xq, 5),sigma_state(member_yq, 5),sigma_state(member_zq, 5));
    ROS_INFO("cam_pose gnd: x:%.3f y:%.3f z:%.3f",(sigma_state(member_xq, 5) - rel_posec2g(0)),predict_sigma_measure(member_myc, 5),predict_sigma_measure(member_mzc, 5));

    cout << "g2c: " << endl << dyn1.rot_g2c  << endl;
    cout << "c2g: " << endl << dyn1.rot_c2g  << endl;

    //ROS_INFO("xq:%.3f",sigma_state(member_xq,0));
    return predict_sigma_measure;
}

//////////////////////////// measurement update
void correct(Eigen::VectorXd measure)
{
    y = measure;

    P_yy_.setZero(y_size, y_size);
    P_yy.setZero(y_size, y_size);
    P_xy.setZero(x_size, y_size);

    for(int i = 0; i < x_sigmavector_size; i++)
    {
        Eigen::MatrixXd err;
        Eigen::MatrixXd err_t;
        err = y_sigmavector.col(i) - y_hat;
        err_t = err.transpose();
        P_yy_ += w_c(i) * err * err_t;
    }
    //add measurement noise covarinace
    P_yy = P_yy_ + R;

    for(int i = 0; i < x_sigmavector_size; i++)
    {
        Eigen::VectorXd err_y , err_x;
        err_y = y_sigmavector.col(i) - y_hat;
        err_x = x_sigmavector.col(i) - x_hat;
        P_xy += w_c(i) * err_x * err_y.transpose();
    }

    Kalman_gain = P_xy * (P_yy.inverse());

    if(!measurement_flag)
    {
        y(0) = y_hat(0);
        y(1) = y_hat(1);
        //y(2) = y_hat(2);
        //y(6) = y_hat(6);
    }
    x = x_hat + Kalman_gain * (y - y_hat);

    P = P - Kalman_gain * P_yy * (Kalman_gain.transpose());

    //cout << "Kalman gain: " << Kalman_gain << endl;
    cout << "y measure: " << endl << y << endl;
    cout << "y measure diff: " << endl << (y - y_hat) << endl;
    //cout << "y measure: " << endl << y << endl;
    cout << "x correct: " << endl << x << endl;
}

//////////////////////////// process noise estimate
void noise_estimate(int window_size)
{
    Eigen::VectorXd q_window_element, r_window_element;
    Eigen::MatrixXd Q_window_element, R_window_element;
    Eigen::MatrixXd Q_window_element_, R_window_element_;
    double delta_S, w_element;
    deque<double> v_weight;
    float S_gain = 8;
    Q_window_element.setZero(x_size, x_size);
    R_window_element.setZero(y_size, y_size);
    v_weight.resize(window_size);
    double accu = 0;

    Eigen::VectorXd q_window_sum, r_window_sum;
    Eigen::MatrixXd Q_window_sum, R_window_sum;

    q_window_element = x - x_hat_;
    r_window_element = y - y_hat_;
    Q_window_element_ = P + Kalman_gain * (y - y_hat) * ((y - y_hat).transpose()) * (Kalman_gain.transpose()) - P_;
    R_window_element_ = (y - y_hat) * ((y - y_hat).transpose()) - P_yy_;
    Q_window_element = Q_window_element_.cwiseAbs().diagonal().asDiagonal();
    R_window_element = R_window_element_.cwiseAbs().diagonal().asDiagonal();
    delta_S = ( S_gain * (P_yy_ + R).trace() ) / ( ((y - y_hat).transpose()) * (y - y_hat) );
    w_element = sqrt( ((x - x_hat).transpose()) * (x - x_hat) ) * sqrt( ((y_hat - y).transpose()) * (y_hat - y) ) * delta_S;

    //q_window.push_front(q_window_element);
    //r_window.push_front(r_window_element);
    Q_window.push_front(Q_window_element);
    //R_window.push_front(R_window_element);
    w_window.push_front(w_element);

    if(q_window.size() > window_size || r_window.size() > window_size || Q_window.size() > window_size || R_window.size() > window_size)
    {
        //ROS_INFO("estimate noise");
        //q_window.resize(window_size);
        //r_window.resize(window_size);
        Q_window.resize(window_size);
        //R_window.resize(window_size);
        w_window.resize(window_size);
    }

    if(q_window.size() == window_size || r_window.size() == window_size || Q_window.size() == window_size || R_window.size() == window_size)
    {
        //ROS_INFO("estimate noise");
        //q_window_sum.setZero(x_size);
        //r_window_sum.setZero(y_size);
        Q_window_sum.setZero(x_size, x_size);
        //R_window_sum.setZero(y_size,y_size);

        for(int i = 0; i < window_size; i++)
        {
            accu += w_window.at(i);
        }
        for(int i = 0; i < window_size; i++)
        {
            v_weight.at(i) = w_window.at(i) / accu;
            //cout<<" w"<<i<<": "<<w_window.at(i);
            //cout<<" accu"<<i<<": "<<std::accumulate(w_window.begin(), w_window.end(), 0);
        }
        
        for(int i = 0; i < window_size; i++)
        {
            //q_window_sum += q_window.at(i)*v_weight.at(i);
            //r_window_sum += r_window.at(i)*v_weight.at(i);
            Q_window_sum += Q_window.at(i) * v_weight.at(i);
            //R_window_sum += R_window.at(i)*v_weight.at(i);
            //cout<<"v_weight"<<i<<": "<<v_weight.at(i);
        }
        if(callback_spin_count > 4 * loop_rate)
        {
            //q = q_window_sum;
            //r = r_window_sum;
            Q = Q_window_sum;
            //R = R_window_sum;
        }
    }

    cout << "\nQsum\n" << Q_window_sum << "\n";
    //cout << "\nRsum\n" << R_window_sum << "\n";
    callback_spin_count++;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_estimate");
    ros::NodeHandle nh;
    ros::Subscriber dynamics_sub = nh.subscribe<fw_control::Dynamics>("estimation/ukf/dynamics", 10, dynamics_cb);
    ros::Subscriber gimbal_sub = nh.subscribe<fw_control::CameraFeatures>("estimation/ukf/camera_features", 10, features_cb);
    ros::Publisher estimate_pub = nh.advertise<fw_control::EstimateOutput>("estimation/ukf/output_data", 1);
    ros::Publisher gt_pub = nh.advertise<fw_control::EstimateOutput>("estimation/ukf/groundtruth", 1);

    ros::Rate rate(loop_rate);

    initialize();
    ros::Time current_time = ros::Time::now();
    ros::Time previous_time = ros::Time::now();
    int measurement_false_count = 0;
    int measurement_true_count = 0;

    /********** set initial value of states ***************/
    x(0) = (cam1.u - cam1.cu) / cam1.fx;
    x(1) = (cam1.v - cam1.cv) / cam1.fy;

    // increase the initial value of P can increase the speed of convergence
    Eigen::MatrixXd P_init;
    P_init.setZero(statesize, statesize);
    ros::param::get("P_init_0", P_init(0, 0) );
    ros::param::get("P_init_1", P_init(1, 1) );
    //ros::param::get("P_init_2", P_init(2, 2) );
    ros::param::get("P_init_3", P_init(2, 2) );
    ros::param::get("P_init_4", P_init(3, 3) );
    ros::param::get("P_init_5", P_init(4, 4) );
    ros::param::get("P_init_6", P_init(5, 5) );
    ros::param::get("P_init_7", P_init(6, 6) );
    ros::param::get("P_init_8", P_init(7, 7) );
    P = P_init;                     // set initial P matrix

    Eigen::MatrixXd measurement_noise;
    measurement_noise.setZero(measurementsize, measurementsize);
    measurement_noise = 1 * Eigen::MatrixXd::Identity(measurementsize, measurementsize);
    ros::param::get("measurement_noise_0", measurement_noise(0, 0) );
    ros::param::get("measurement_noise_1", measurement_noise(1, 1) );
    //ros::param::get("measurement_noise_2", measurement_noise(2, 2) );
    ros::param::get("measurement_noise_3", measurement_noise(2, 2) );
    ros::param::get("measurement_noise_4", measurement_noise(3, 3) );
    ros::param::get("measurement_noise_5", measurement_noise(4, 4) );
    R = measurement_noise;         // set measurement noise

    Eigen::MatrixXd process_noise;
    process_noise.setZero(statesize, statesize);
    process_noise = 1 * Eigen::MatrixXd::Identity(statesize, statesize);
    ros::param::get("process_noise_0", process_noise(0, 0) );
    ros::param::get("process_noise_1", process_noise(1, 1) );
    //ros::param::get("process_noise_2", process_noise(2, 2) );
    ros::param::get("process_noise_3", process_noise(2, 2 ));
    ros::param::get("process_noise_4", process_noise(3, 3) );
    ros::param::get("process_noise_5", process_noise(4, 4) );
    ros::param::get("process_noise_6", process_noise(5, 5) );
    ros::param::get("process_noise_7", process_noise(6, 6) );
    ros::param::get("process_noise_8", process_noise(7, 7) );
    Q = process_noise;             // set process noise

    fw_control::EstimateOutput gt_value, estimate_value;

    cout << "process_noise_8: " << process_noise(7, 7) << endl;

    while(ros::ok())
    {
        if( !isnormal(x(0)) )
        {
            initialize();
            x(0) = (cam1.u - cam1.cu) / cam1.fx;
            x(1) = (cam1.v - cam1.cv) / cam1.fy;
            P = P_init;
            R = measurement_noise;
            Q = process_noise;
            callback_spin_count = 50;
            ibvs_mode = false;
            ROS_INFO("x(0): %.3f, x(1): %.3f", x(0), x(1));
        }

        current_time = ros::Time::now();
        dt = current_time.toSec() - previous_time.toSec();
        //cout << "time step: " << dt << endl;
        previous_time = current_time;

        predict();

        //save measurement data to matrix for correct()
        Eigen::VectorXd measure_vector;
        measure_vector.setZero(measurementsize);
        measure_vector << cam1.u, cam1.v, dyn1.cam_posegnd(0), dyn1.cam_posegnd(1), dyn1.cam_posegnd(2);

        //cout << "GT cam pose: " << endl << dyn1.cam_posegnd(0) << ", " << dyn1.cam_posegnd(1) << ", " << dyn1.cam_posegnd(2) << endl;

        // execute correct if the target is detected
        if(cam1.u >= 0.5)
        {
            measurement_flag = true;
            //if the measurement is lost for 3s, set the feature vector state as measurement when the bounding box appear
            if(measurement_false_count > 3 * loop_rate)
            {
                //initialize();
                x(0) = (cam1.u - cam1.cu) / cam1.fx;
                x(1) = (cam1.v - cam1.cv) / cam1.fy;
                ROS_INFO("YOLO False more than 3 sec .....");
            }
            measurement_true_count++;
            measurement_false_count = 0;
        }
        else
        {
            if(measurement_false_count > 7 * loop_rate)
            {
                ibvs_mode = false;
                ROS_INFO("YOLO False more than 7 sec .....");
            }
            measurement_flag = false;
            measurement_false_count++;
            measurement_true_count = 0;
            ROS_INFO("YOLO Measure False Count: %d", measurement_false_count);
        }

        correct(measure_vector);
        //noise_estimate(2 * loop_rate);

/*
        if( Q(3,3) < 0.002 )
        {
            Q(3,3) = 0.002;
        }
        if( Q(4,4) < 0.002 )
        {
            Q(4,4) = 0.002;
        }
        
*/
/*
        if (Q(5,5) < 0.002 )
        {
            Q(5,5) = 0.002;
        }
        if( Q(6,6) < 0.002 )
        {
            Q(6,6) = 0.002;
        }
        if( Q(7,7) < 0.002 )
        {
            Q(7,7) = 0.002;
        }
*/


        // error
        cout << "Position error: " << endl <<
            "x (m)   " << ( x(2) - dyn1.car_posegnd(0) ) << endl <<
            "y (m)   " << ( x(3) - dyn1.car_posegnd(1) ) << endl <<
            "z (m)   " << ( x(4) - dyn1.car_posegnd(2) ) << endl <<
            "Velocity error: " << endl <<
            "x (m/s) " << ( x(5) - dyn1.car_velgnd(0) ) << endl <<
            "y (m/s) " << ( x(6) - dyn1.car_velgnd(1) ) << endl <<
            "z (m/s) " << ( x(7) - dyn1.car_velgnd(2) ) << endl << endl;


        gt_value.feature_1.data = (cam1.u - cam1.cu) / cam1.fx;
        gt_value.feature_2.data = (cam1.v - cam1.cv) / cam1.fy;
        gt_value.target_pose.x = dyn1.car_posegnd(0);
        gt_value.target_pose.y = dyn1.car_posegnd(1);
        gt_value.target_pose.z = dyn1.car_posegnd(2);
        gt_value.target_vel.x = dyn1.car_velgnd(0);
        gt_value.target_vel.y = dyn1.car_velgnd(1);
        gt_value.target_vel.z = dyn1.car_velgnd(2);

        estimate_value.feature_1.data = x(0);
        estimate_value.feature_2.data = x(1);
        estimate_value.target_pose.x = x(2);
        estimate_value.target_pose.y = x(3);
        estimate_value.target_pose.z = x(4);
        estimate_value.target_vel.x = x(5);
        estimate_value.target_vel.y = x(6);
        estimate_value.target_vel.z = x(7);

        gt_pub.publish(gt_value);
        estimate_pub.publish(estimate_value);
   
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}