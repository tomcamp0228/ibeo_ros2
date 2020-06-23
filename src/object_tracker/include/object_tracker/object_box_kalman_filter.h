
// #include <ros/time.h>
#include <rclcpp/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <math.h>
#include <eigen3/Eigen/Dense>
namespace object_tracker
{

    /*
       3 ___ 2
        |   |    .
        |   |   / \ yaw
        | . |    |
        | 4 |    |
       0-----1
    */

class ObjectBoxKalmanFilter{

    public:
        uint id;
        enum OBJ_CLASS{
            UNCLASSIFIED = 0,
            UNKNOWN_SMALL = 1,
            UNKNOWN_BIG = 2,
            PEDESTRIAN = 3,
            CAR = 5            
        };
        OBJ_CLASS classification;
        bool is_lost;
        rclcpp::Time lost_time;
        
        double system_error_position;
        double system_error_velocity;
        double system_error_yaw;
        double measurement_error_position;
        double measurement_error_yaw;

        uint ref_point_id;
        double x[4], y[4], center_x, center_y;      //calculated by the function calContourPoints()

    public:
        ObjectBoxKalmanFilter(){
            rclcpp::Clock cl;
            rclcpp::Time t_now=cl.now();
            initFilter(0, 0, 0, 0, 0, t_now, 99999.0, 99999.0);//TODO: get current time without using node? Is process above available?
            system_error_position = 0.3;
            system_error_yaw = 0.1;
            measurement_error_position = 0.4;
            measurement_error_yaw = 0.1;
            updateObserveAndNoiseMatrix();
            ref_point_id = 5;
        }

        ObjectBoxKalmanFilter(uint _id, double _size_x, double _size_y, double _x, double _y, double _yaw, rclcpp::Time _t, uint _ref_point_id = 5,
                              double _system_error_position = 0.3, double _system_error_yaw = 0.2,
                              double _measurement_error_position = 0.4, double _measurement_error_yaw = 0.1,
                              double first_scan_position_sigma = 0.4, double first_scan_yaw_sigma = 0.3){
            id = _id;
            initFilter(_size_x, _size_y, _x, _y, _yaw, _t, first_scan_position_sigma, first_scan_yaw_sigma);
            system_error_position = _system_error_position;
            system_error_yaw = _system_error_yaw;
            measurement_error_position = _measurement_error_position;
            measurement_error_yaw = _measurement_error_yaw;
            updateObserveAndNoiseMatrix();
            ref_point_id = _ref_point_id;
            is_lost = false;
        }
        
        void initFilter(double _size_x, double _size_y, double _x, double _y, double _yaw, rclcpp::Time _t,
                        double first_scan_position_sigma = 0.4, double first_scan_yaw_sigma = 0.3){
            
            size_x = _size_x;
            size_y = _size_y;
            state(0) = _x;
            state(1) = _y;
            state(2) = 0.0;
            state(3) = 0.0;
            state(4) = _yaw;
            state(5) = 0.0;

            track_start_time = time = _t;

            F.setIdentity();
            P.setZero();
            P(0, 0) = P(1, 1) = first_scan_position_sigma;
            P(2, 2) = P(3, 3) = 99999.0;
            P(4, 4) = first_scan_yaw_sigma;
            P(5, 5) = 99999.0;
        }

        void updateObserveAndNoiseMatrix(){

            H_pos.setZero();
            H_pos(0, 0) = H_pos(1, 1) = 1;
            H_pos_yaw.setZero();
            H_pos_yaw(0, 0) = H_pos_yaw(1, 1) = H_pos_yaw(2, 4) = 1;
            H_yaw.setZero();
            H_yaw(0, 4) = 1;

            Q.setZero();
            Q(0, 0) = Q(1, 1) = Q(2, 2) = Q(3, 3) = system_error_position;
            Q(4, 4) = Q(5, 5) = system_error_yaw;
            R_pos(0, 0) = R_pos(1, 1) = measurement_error_position;
            R_pos_yaw(0, 0) = R_pos_yaw(1, 1) = measurement_error_position;
            R_pos_yaw(2, 2) = measurement_error_yaw;
            R_yaw(0, 0) = measurement_error_yaw;
        }

        /*
        F=[
            1   0   dt  0   0   0
            0   1   0   dt  0   0
            0   0   1   0   0   0
            0   0   0   1   0   0
            0   0   0   0   1   dt
            0   0   0   0   0   1
        ]
        */
       //prediction step for KF
        void prediction(rclcpp::Time t){
            
            if(t < time)
                return;
            
            F(0, 2) = F(1, 3) = F(4, 5) = (t - time).seconds();
            state = F * state;
            P = F * P * F.transpose() + Q;
            time = t;
        }

        void positionMeasurementUpdate(double mea_x, double mea_y, rclcpp::Time t){

            if(t != time)
                prediction(t);
            
            Eigen::Matrix<double, 2, 1> Z;
            Z << mea_x , mea_y;
            K = P * H_pos.transpose() * (H_pos * P * H_pos.transpose() + R_pos).inverse();
            state = state + K * (Z - H_pos * state);
            P = (Eigen::MatrixXd::Identity(6, 6) - K * H_pos) * P;

            // ROS_WARN_STREAM("Update K: " << K << std::endl);
            // ROS_WARN_STREAM("Measure Z: " << Z << std::endl);
            // ROS_WARN_STREAM("Update X: " << X << std::endl);
            // ROS_WARN_STREAM("Update P: " << P << std::endl);
        }
        
        void positionAndYawMeasurementUpdate(double mea_x, double mea_y, double mea_yaw, rclcpp::Time t){

            if(t != time)
                prediction(t);
            
            Eigen::Matrix<double, 3, 1> Z;
            Z << mea_x, mea_y, mea_yaw;
            K = P * H_pos_yaw.transpose() * (H_pos_yaw * P * H_pos_yaw.transpose() + R_pos_yaw).inverse();
            state = state + K * (Z - H_pos_yaw * state);
            P = (Eigen::MatrixXd::Identity(6, 6) - K * H_pos_yaw) * P;
        }

        void yawMeasurementUpdate(double mea_yaw, rclcpp::Time t){
            
            if(t != time)
                prediction(t);
            
            Eigen::Matrix<double, 1, 1> Z;
            Z << mea_yaw;
            K = P * H_yaw.transpose() * (H_yaw * P * H_yaw.transpose() + R_yaw).inverse();
            state = state + K * (Z - H_yaw * state);
            P = (Eigen::MatrixXd::Identity(6, 6) - K * H_yaw) * P;
        }

        void updateSize(double _size_x, double _size_y, uint fix_point_id){

            calContourPoints();
            uint using_ref_point_id;
            using_ref_point_id = ref_point_id;
            ref_point_id = fix_point_id;
            size_x = _size_x;
            size_y = _size_y;
            calContourPoints();
            if(fix_point_id != 4){
                state(0) = x[using_ref_point_id];
                state(1) = y[using_ref_point_id];
            }
            else{
                state(0) = center_x;
                state(1) = center_y;
            }

            ref_point_id = using_ref_point_id;
        }



    public:
        inline double getRefPositionX(){
            return state(0);
        }
        inline double getRefPositionY(){
            return state(1);
        }
        inline double getRefVelocityX(){
            return state(2);
        }
        inline double getRefVelocityY(){
            return state(3);
        }
        inline double getYaw(){
            return state(4);
        }
        inline double getYawRate(){
            return state(5);
        }
        inline double getSizeX(){
            return size_x;
        }
        inline double getSizeY(){
            return size_y;
        }
        inline rclcpp::Duration getTrackDuration(){
            return time - track_start_time;
        }
        void calContourPoints(){
            if(ref_point_id == 5) return;
            switch(ref_point_id){
                case 0:
                    x[0] = state(0);
                    y[0] = state(1);
                    break;

                case 1:
                    x[0] = state(0) + size_y * cos(state(4) + M_PI_2);
                    y[0] = state(1) + size_y * sin(state(4) + M_PI_2);
                    break;

                case 2:
                    x[0] = state(0) + size_y * cos(state(4) + M_PI_2) + size_x * cos(state(4) + M_PI);
                    y[0] = state(1) + size_y * sin(state(4) + M_PI_2) + size_x * sin(state(4) + M_PI);
                    break;

                case 3:
                    x[0] = state(0) + size_x * cos(state(4) + M_PI);
                    y[0] = state(1) + size_x * sin(state(4) + M_PI);
                    break;
                
                case 4:
                    x[0] = state(0) + size_y * cos(state(4) + M_PI_2) / 2 + size_x * cos(state(4) + M_PI) / 2;
                    y[0] = state(1) + size_y * sin(state(4) + M_PI_2) / 2 + size_x * sin(state(4) + M_PI) / 2;
                    break;                    
            }
            x[1] = x[0] + size_y * cos(state(4) - M_PI_2); 
            y[1] = y[0] + size_y * sin(state(4) - M_PI_2);
            x[2] = x[0] + size_y * cos(state(4) - M_PI_2) + size_x * cos(state(4)); 
            y[2] = y[0] + size_y * sin(state(4) - M_PI_2) + size_x * sin(state(4));
            x[3] = x[0] + size_x * cos(state(4)); 
            y[3] = y[0] + size_x * sin(state(4));
            
            if(ref_point_id != 4){
                center_x = (x[0] + x[2]) / 2.0;
                center_y = (y[0] + y[2]) / 2.0;
            }
            else{
                center_x = state(0);
                center_y = state(1);
            }
        }

        void changeRefPoint(uint _ref_point_id){
            if(_ref_point_id == 5){
                ref_point_id = 5;
                return;
            }
            calContourPoints();
            if(_ref_point_id != 4){
                state(0) = x[_ref_point_id];
                state(1) = y[_ref_point_id];
                ref_point_id = _ref_point_id;
            }
            else{
                state(0) = center_x;
                state(1) = center_y;
                ref_point_id = _ref_point_id;
            }
        }

        void rotatePointId(uint n){
            if(n == 0) return;
            if(n % 2 != 0){
                std::swap(size_x, size_y);
            }
            if(ref_point_id != 4){
                ref_point_id = (ref_point_id + 3) % 4;
                state(4) += (n % 4) * M_PI_2;
                while(state(4) < - M_PI) state(4) += 2 * M_PI;
                while(state(4) >= M_PI) state(4) -= 2 * M_PI;
                calContourPoints();
            }
            else{
                state(4) += (n % 4) * M_PI_2;
                while(state(4) < - M_PI) state(4) += 2 * M_PI;
                while(state(4) >= M_PI) state(4) -= 2 * M_PI;
                calContourPoints();
            }

        }


    private:
        double size_x;
        double size_y;
        Eigen::Matrix<double, 6, 1> state;  // x, y, x', y', yaw, yaw'
        uint tracking_point;                // = 0, 1, 2, 3 (box edge point)
        rclcpp::Time time;
        rclcpp::Time track_start_time;

        Eigen::Matrix<double, 6, 6> F;          //state transition matrix
        Eigen::Matrix<double, 6, 6> P;          //state covariation matrix

        Eigen::Matrix<double, 2, 6> H_pos;      //position only observe matrix
        Eigen::Matrix<double, 3, 6> H_pos_yaw;  //position and yaw observe matrix
        Eigen::Matrix<double, 1, 6> H_yaw;      //yaw observe matrix
        Eigen::Matrix<double, 6, 6> Q;          //system noise
        Eigen::Matrix<double, 2, 2> R_pos;      //observation noise position only
        Eigen::Matrix<double, 3, 3> R_pos_yaw;  //observation noise postion and yaw
        Eigen::Matrix<double, 1, 1> R_yaw;      //observation noise yaw

        Eigen::MatrixXd K;

};


} //namespace object_tracker
