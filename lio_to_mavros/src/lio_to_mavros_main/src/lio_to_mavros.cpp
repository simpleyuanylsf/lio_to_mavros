
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <cmath>
#include <queue>
 
Eigen::Vector3d p_lidar_body, p_enu;
Eigen::Quaterniond q_mav;
Eigen::Quaterniond q_px4_odom;

// 【修改1】声明全局时间戳变量，否则编译报错
ros::Time lidar_timestamp; 

class SlidingWindowAverage {
public:
    SlidingWindowAverage(int windowSize) : windowSize(windowSize), windowSum(0.0) {}

    double addData(double newData) {
        if(!dataQueue.empty()&&fabs(newData-dataQueue.back())>0.01){
            dataQueue = std::queue<double>();
            windowSum = 0.0;
            dataQueue.push(newData);
            windowSum += newData;
        }
        else{            
            dataQueue.push(newData);
            windowSum += newData;
        }

        if (dataQueue.size() > windowSize) {
            windowSum -= dataQueue.front();
            dataQueue.pop();
        }
        windowAvg = windowSum / dataQueue.size();
        return windowAvg;
    }

    int get_size(){
        return dataQueue.size();
    }

    double get_avg(){
        return windowAvg;
    }

private:
    int windowSize;
    double windowSum;
    double windowAvg;
    std::queue<double> dataQueue;
};

int windowSize = 8;
SlidingWindowAverage swa=SlidingWindowAverage(windowSize);

double fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}

void lio_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    p_lidar_body = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    q_mav = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    
    // 【修改2】正确获取并保存 Fast-LIO 的原始时间戳
    lidar_timestamp = msg->header.stamp;
}
 
void px4_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    q_px4_odom = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    swa.addData(fromQuaternion2yaw(q_px4_odom));
} 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lio_to_mavros");
    ros::NodeHandle nh("~");
 
    ros::Subscriber slam_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 100, lio_callback);
    ros::Subscriber px4_odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 5, px4_odom_callback);
 
    ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
 
    ros::Rate rate(50.0); // 建议提高到 30Hz-50Hz，20Hz 对 Fast-LIO 略低
 
    bool init_flag = 0;
    float init_yaw = 0.0;
    Eigen::Quaterniond init_q;

    while(ros::ok()){
        // 初始化逻辑
        
        // 原始代码：
// if(swa.get_size()==windowSize && !init_flag){
//     init_yaw = swa.get_avg();  <-- 这里会计算初始偏航角，导致坐标系旋转
//     init_flag = 1;
//     init_q = Eigen::AngleAxisd(init_yaw, Eigen::Vector3d::UnitZ());
//     ROS_INFO("Initialization Complete. Yaw Offset: %f rad", init_yaw);
// }

        // 【修改后代码】：强制不旋转，直接对齐
        if(swa.get_size()==windowSize && !init_flag){
            // 强制设为 0，保证 Fast-LIO 的 X 轴就是 MAVROS 的 X 轴
            init_yaw = 0.0; 
            init_flag = 1;
            init_q = Eigen::AngleAxisd(init_yaw, Eigen::Vector3d::UnitZ());
            ROS_INFO("Initialization Complete. FORCE Yaw Offset to 0.0");
        }

        if(init_flag){
            geometry_msgs::PoseStamped vision;
            
            // 1. 位置旋转 (你原来的代码是对的)
            p_enu = init_q * p_lidar_body;
            
            // 2. 【核心修改】姿态旋转 (必须加上这一步！)
            // 即使禁用了罗盘，为了保证坐标系严格对齐，这里必须旋转
            Eigen::Quaterniond q_enu = init_q * q_mav; 

            vision.pose.position.x = p_enu[0];
            vision.pose.position.y = p_enu[1];
            vision.pose.position.z = p_enu[2];
    
            vision.pose.orientation.x = q_enu.x();
            vision.pose.orientation.y = q_enu.y();
            vision.pose.orientation.z = q_enu.z();
            vision.pose.orientation.w = q_enu.w();
    
            // 3. 【修改3】使用原始时间戳，减少 EKF 滞后
            vision.header.stamp = lidar_timestamp;
            vision.header.frame_id = "map"; // 建议加上 Frame ID
            
            vision_pub.publish(vision);
        }
 
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}
// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Odometry.h>
// #include <Eigen/Eigen>
// #include <cmath>
// #include <queue>
 
// Eigen::Vector3d p_lidar_body, p_enu;
// Eigen::Quaterniond q_mav;
// Eigen::Quaterniond q_px4_odom;

//  class SlidingWindowAverage {
// public:
//     SlidingWindowAverage(int windowSize) : windowSize(windowSize), windowSum(0.0) {}

//     double addData(double newData) {
//         if(!dataQueue.empty()&&fabs(newData-dataQueue.back())>0.01){
//             dataQueue = std::queue<double>();
//             windowSum = 0.0;
//             dataQueue.push(newData);
//             windowSum += newData;
//         }
//         else{            
//             dataQueue.push(newData);
//             windowSum += newData;
//         }

//         // 如果队列大小超过窗口大小，弹出队列头部元素并更新窗口和队列和
//         if (dataQueue.size() > windowSize) {
//             windowSum -= dataQueue.front();
//             dataQueue.pop();
//         }
//         windowAvg = windowSum / dataQueue.size();
//         // 返回当前窗口内的平均值
//         return windowAvg;
//     }

//     int get_size(){
//         return dataQueue.size();
//     }

//     double get_avg(){
//         return windowAvg;
//     }

// private:
//     int windowSize;
//     double windowSum;
//     double windowAvg;
//     std::queue<double> dataQueue;
// };

// int windowSize = 8;
// SlidingWindowAverage swa=SlidingWindowAverage(windowSize);

// double fromQuaternion2yaw(Eigen::Quaterniond q)
// {
//   double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
//   return yaw;
// }

// void lio_callback(const nav_msgs::Odometry::ConstPtr &msg)
// {

//     p_lidar_body = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
//     q_mav = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
//     // 保存原始时间戳
//     lidar_timestamp = msg->header.stamp;
// }
 
// void px4_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
// {
//     q_px4_odom = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
//     swa.addData(fromQuaternion2yaw(q_px4_odom));
// } 

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "lio_to_mavros");
//     ros::NodeHandle nh("~");
 
//     ros::Subscriber slam_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 100, lio_callback);
//     ros::Subscriber px4_odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 5, px4_odom_callback);
 
//     ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
 
 
//     // the setpoint publishing rate MUST be faster than 2Hz
//     ros::Rate rate(20.0);
 
//     ros::Time last_request = ros::Time::now();
//     float init_yaw = 0.0;
//     bool init_flag = 0;
//     Eigen::Quaterniond init_q;
//     while(ros::ok()){
//         if(swa.get_size()==windowSize&&!init_flag){
//             init_yaw = swa.get_avg();
//             init_flag = 1;
//             init_q = Eigen::AngleAxisd(init_yaw,Eigen::Vector3d::UnitZ())//des.yaw
//     * Eigen::AngleAxisd(0.0,Eigen::Vector3d::UnitY())
//     * Eigen::AngleAxisd(0.0,Eigen::Vector3d::UnitX());
//         // delete swa;
//         }

//         if(init_flag){
//             geometry_msgs::PoseStamped vision;
//             p_enu = init_q*p_lidar_body;
    
//             vision.pose.position.x = p_enu[0];
//             vision.pose.position.y = p_enu[1];
//             vision.pose.position.z = p_enu[2];
    
//             vision.pose.orientation.x = q_mav.x();
//             vision.pose.orientation.y = q_mav.y();
//             vision.pose.orientation.z = q_mav.z();
//             vision.pose.orientation.w = q_mav.w();
    
//             vision.header.stamp = ros::Time::now();
//             vision_pub.publish(vision);
    
//             ROS_INFO("\nposition in enu:\n   x: %.18f\n   y: %.18f\n   z: %.18f\norientation of lidar:\n   x: %.18f\n   y: %.18f\n   z: %.18f\n   w: %.18f", \
//             p_enu[0],p_enu[1],p_enu[2],q_mav.x(),q_mav.y(),q_mav.z(),q_mav.w());

//         }

 
//         ros::spinOnce();
//         rate.sleep();
//     }
 
//     return 0;
// }
