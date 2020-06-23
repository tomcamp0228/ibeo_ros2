#include <eigen3/Eigen/Dense>
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>

using Eigen::Matrix4d;
using Eigen::Vector4d;

namespace ObjectTracker
{


class CarKF{

    public:
        CarKF(){
            Init();
            err_mea = 1;
            err_sys = 0.2;
        }
        CarKF(double _sys, double _mea){
            err_mea = _mea;
            err_sys = _sys;
            Init();
        }

        double getPosX(){
            return X(0);
        }
        double getPosY(){
            return X(1);
        }
        double getVelX(){
            return X(2);
        }
        double getVelY(){
            return X(3);
        }
        
    private:
        rclcpp::Time dataTime;
        bool firstscan;
        Vector4d X;
        Matrix4d P;
        Matrix4d Q;
        Matrix4d R;
        Matrix4d K;
        // Matrix4d H;
        Matrix4d F;
        double err_mea, err_sys;

        void Init(){
            // X.resize(4, 1);
            // P.resize(4, 4);
            // Q.resize(4, 4);
            // R.resize(4, 4);
            // F.resize(4, 4);
            // H.resize(4, 4);

            P << 0.2, 0, 0, 0,
                0, 0.2, 0, 0,
                0, 0, 0.2, 0,
                0, 0, 0, 0.2;

            Q << err_sys, 0, 0, 0,
                0, err_sys, 0, 0,
                0, 0, err_sys, 0,
                0, 0, 0, err_sys;
            
            R << err_mea, 0, 0, 0,
                0, err_mea, 0, 0,
                0, 0, err_mea, 0,
                0, 0, 0, err_mea;
            
            // H << 1, 0, 0, 0,
            //          0, 1, 0, 0,
            //          0, 0, 1, 0,
            //          0, 0, 0, 1;

            F << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
            
            firstscan = true;
        }
    
    public:
        void prediction(rclcpp::Time t){

            F(0, 2) = F(1, 3) = (t - dataTime).seconds();
            X = F * X;
            // ROS_WARN_STREAM("Predict F: " << F << std::endl);
            // ROS_WARN_STREAM("Predict X: " << X << std::endl);
            P = F * P * F.transpose() + Q;
            // ROS_WARN_STREAM("Predict P: " << P << std::endl);
            dataTime = t;
        }

        void measurementUpdate(Vector4d Z, rclcpp::Time t){

            if(Z.rows() != 4 || Z.cols() != 1){
                // RCLCPP_ERROR(logger,"Wrong measurement dimensions. Dropped.\n");
                return;
            }

            if(firstscan){
                firstscan = false;
                dataTime = t;
                X = Z;
                return;
            }
            prediction(t);

            // K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
            K = P * (P + R).inverse();
            // X = X + K*(Z - H * X);
            X = X + K * (Z - X);
            // P = (MatrixXd::Identity(size, size) - K * H) * P;
            P = (Matrix4d::Identity(4, 4) - K) * P;

            // ROS_WARN_STREAM("Update K: " << K << std::endl);
            // ROS_WARN_STREAM("Measure Z: " << Z << std::endl);
            // ROS_WARN_STREAM("Update X: " << X << std::endl);
            // ROS_WARN_STREAM("Update P: " << P << std::endl);
        }
};


} //namespace ObjectTracker


