#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  n_x_ = 5;

  n_aug_ = 7;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.15;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  lambda_ = 3 - n_aug_;

  is_initialized_ = false;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if(!is_initialized_){
      MatrixXd I = MatrixXd::Identity(n_x_, n_x_);
      P_ = I ;
      float th = 0.001;
      if(meas_package.sensor_type_ == MeasurementPackage::LASER){
        
        if(fabs(meas_package.raw_measurements_(0)) < th && fabs(meas_package.raw_measurements_(1)) <th)
          x_ << th, th, 0, 0, 0;
        else
          x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 0, 0, 0;
      }
      else if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
        double rho = meas_package.raw_measurements_(0);
        double phi = meas_package.raw_measurements_(1);
        double rho_dot = meas_package.raw_measurements_(2);

        double px = rho * cos(phi);
        double py = rho * sin(phi);
        double vx = rho_dot * cos(phi);
        double vy = rho_dot * sin(phi); 
        double v = sqrt(vx*vx + vy*vy);
        double yaw = 0;
        double yaw_rate = 0;
        x_ << px, py, v, yaw, yaw_rate;
      }
      weights_ = VectorXd(2 * n_aug_ +1);
      weights_.fill(0.5/(lambda_+ n_aug_));
      weights_(0) = lambda_/(lambda_ + n_aug_);

      time_us_ = meas_package.timestamp_;
      is_initialized_ = true;
      return ;
  }

  //convert to second
  double delta_t = (meas_package.timestamp_ - time_us_)/ 1000000.0;
  time_us_ = meas_package.timestamp_;
  

  //Predict
  cout<<"-Sensor: "<< meas_package.sensor_type_<<endl;
  Prediction(delta_t);

  //Update
  if(meas_package.sensor_type_ == MeasurementPackage::LASER){
    UpdateLidar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
    UpdateRadar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  

  // --------------------------------
  // Step 1: Generate Sigma points
  cout << delta_t<<endl;
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(n_x_) = x_;
  x_aug(5)=0;
  x_aug(6)=0;

  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;

  // sqrt of P_aug
  MatrixXd A = P_aug.llt().matrixL();

  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
  
  Xsig_aug.col(0)= x_aug;
  
  for(int i=0; i<n_aug_; i++){
    Xsig_aug.col(i+1) = x_aug + sqrt(lambda_ + n_aug_)*A.col(i);
    Xsig_aug.col(n_aug_+i+1) = x_aug - sqrt(lambda_ + n_aug_)*A.col(i);
  }
  
  // --------------------------------
  // step 2 : Predicting Sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

  for(int i=0; i< 2 * n_aug_ +1; i++){
    double px = Xsig_aug(0,i);
    double py = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yaw_d = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // cout<< "Pred: " <<px <<"**"<< py <<"**"<< v <<"**"<< yaw <<"**"<< yaw_d <<"**"<<nu_a<<"**"<<nu_yawdd<<endl;
    float th = 0.001;
    if(fabs(yaw_d) > th){
      px = px + (v/yaw_d) * (sin(yaw + yaw_d * delta_t ) - sin(yaw));
      py = py + (v/yaw_d) * (cos(yaw) - cos(yaw + yaw_d * delta_t ));  
    }else{
      px = px + v * cos(yaw) * delta_t;
      py = py + v * sin(yaw) * delta_t;
    }
    px += 0.5 * pow(delta_t,2) * cos(yaw) * nu_a;
    py += 0.5 * pow(delta_t,2) * sin(yaw) * nu_a;

    v = v + nu_a * delta_t;
    yaw = yaw + yaw_d * delta_t  + 0.5 * pow(delta_t,2) * nu_yawdd;
    yaw_d = yaw_d + delta_t * nu_yawdd;

    Xsig_pred_(0,i) = px;
    Xsig_pred_(1,i) = py;
    Xsig_pred_(2,i) = v;
    Xsig_pred_(3,i) = yaw;
    Xsig_pred_(4,i) = yaw_d;
  }
  cout<<"--Created Sigmapoints"<<endl;
  // --------------------------------
  // Step3: Predict Mean & Cov
  VectorXd x_pred = VectorXd(n_x_);
  x_pred.fill(0.0);
  for(int i = 0; i< 2*n_aug_+1; i++){
    x_pred = x_pred + weights_(i) * Xsig_pred_.col(i);
  }

  MatrixXd P_pred = MatrixXd(n_x_,n_x_);
  P_pred.fill(0.0);
  for(int i = 0; i< 2*n_aug_+1; i++){
    VectorXd x_diff = Xsig_pred_.col(i) - x_pred;
    //normalize
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    P_pred = P_pred + weights_(i) * x_diff * x_diff.transpose();
  }

  x_ = x_pred;
  P_ = P_pred;
  
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  // --------------------------------
  // Step1: Z Sigma points
  n_z_ = 2;
  Zsig_pred_ = MatrixXd(n_z_,2*n_aug_+1);
  Zsig_pred_ = Xsig_pred_.block(0,0, n_z_, 2*n_aug_+1);

  // --------------------------------
  // Step3: Update
  UpdateHelper(meas_package);

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  n_z_ =3;
  // --------------------------------
  // Step1: Z Sigma points
  Zsig_pred_ = MatrixXd(n_z_,2*n_aug_+1);
  Zsig_pred_.fill(0.0);
  for(int i = 0; i< 2*n_aug_*1 ; i++){
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double rho = sqrt(pow(px,2) + pow(py,2));
    float th = 0.001;
    if(fabs(rho) < th) rho = th;

    double phi = atan2(py,px);
    Zsig_pred_(0,i) = rho;
    Zsig_pred_(1,i) = phi;
    Zsig_pred_(2,i) = (px * cos(yaw) * v + py * sin(yaw) * v)/rho;
  }
  
  // --------------------------------

  // --------------------------------
  // Step3: Update
  
  UpdateHelper(meas_package);

}

void UKF::UpdateHelper(MeasurementPackage meas_package){

  // Predict Z mean and S cov
  VectorXd z_pred = VectorXd(n_z_);
  z_pred.fill(0.0);
  for(int i = 0 ; i< 2* n_aug_ +1 ; i++){
    z_pred = z_pred + weights_(i)* Zsig_pred_.col(i); 
  }

  MatrixXd S_pred = MatrixXd(n_z_,n_z_);
  S_pred.fill(0.0);
  for(int i = 0 ; i< 2* n_aug_ +1 ; i++){
    VectorXd z_diff = z_pred - Zsig_pred_.col(i);
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
      while(z_diff(1) > M_PI) z_diff(1) -= 2* M_PI;
      while(z_diff(1) < -M_PI) z_diff(1) += 2* M_PI;
    }
    S_pred = S_pred + weights_(i) * z_diff * z_diff.transpose();
  }

  MatrixXd R = MatrixXd(n_z_,n_z_);
  R.fill(0);
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
    R(0,0) = std_radr_*std_radr_;
    R(1,1) = std_radphi_*std_radphi_;
    R(2,2) = std_radrd_*std_radrd_;
    
  }else{
    R(0,0) = std_laspx_*std_laspx_;
    R(1,1) = std_laspx_*std_laspx_;
  }
  S_pred = S_pred+R;

  MatrixXd T = MatrixXd(n_x_, n_z_);
  T.fill(0.0);
  for(int i = 0; i< 2* n_aug_ +1 ; i++){
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    while(x_diff(3) > M_PI) x_diff(3) -= 2*M_PI;
    while(x_diff(3) < -M_PI) x_diff(3) += 2*M_PI;

    VectorXd z_diff = Zsig_pred_.col(i) - z_pred;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
      while(z_diff(1) > M_PI) z_diff(1) -= 2*M_PI;
      while(z_diff(1) < -M_PI) z_diff(1) += 2*M_PI;
    }
    T = T + weights_(i)*x_diff * z_diff.transpose();
  }
  MatrixXd K_gain = T * S_pred.inverse();

  VectorXd z = meas_package.raw_measurements_;
  x_ = x_ + K_gain * (z- z_pred);
  P_ = P_ - K_gain * S_pred * K_gain.transpose();

  // Calculate NIS
  VectorXd z_diff = z - z_pred;
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
      while(z_diff(1) > M_PI) z_diff(1) -= 2*M_PI;
      while(z_diff(1) < -M_PI) z_diff(1) += 2*M_PI;
  }
  NIS_ = z_diff.transpose()*S_pred.inverse()*z_diff;

  cout<<"NIS: "<<NIS_<<endl;
}
