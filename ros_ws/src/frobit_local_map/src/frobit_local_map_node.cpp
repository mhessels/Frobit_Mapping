#include"frobit_local_map/local_map.h"
#include"sensor_msgs/LaserScan.h"
#include"nav_msgs/Odometry.h"
#include<iostream>
#include<math.h>
#include<Eigen/Core>

#define offset_low 119 // 30 degrees 
#define nr_scans 480 //120 degrees

bool updated = false;

Eigen::Vector3f scans[nr_scans];
Eigen::Matrix3f t;

void handle_odom(const nav_msgs::Odometry::ConstPtr& msg){
    	double ysqr = msg->pose.pose.orientation.y * msg->pose.pose.orientation.y;
    	// yaw (z-axis rotation)
	double t3 = +2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double t4 = +1.0 - 2.0 * (ysqr + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  
	double yaw = std::atan2(t3, t4);
    t(0,0) = std::cos(yaw);
    t(0,1) = -std::sin(yaw);
    t(0,2) = msg->pose.pose.position.x;
    t(1,0) = std::sin(yaw);
    t(1,1) = std::cos(yaw);
    t(1,2) = msg->pose.pose.position.y;
    t(2,0) = 0;
    t(2,1) = 0;
    t(2,2) = 1;
}

void handle_scan(const sensor_msgs::LaserScan::ConstPtr& msg){
    for(int i = 0;i<nr_scans;i++){
        Eigen::Vector3f temp;
        if(msg->ranges[i+offset_low] > msg->range_max){
            temp(0) = -1;
            temp(1) = -1;
            temp(2) = 1;
        }else{
        double angle = (i+offset_low)*msg->angle_increment;
        double r = msg->ranges[i+offset_low];
        temp(0) = r*cos(angle);
        temp(1) = r*sin(angle);
        temp(2) = 1;
        }
        scans[i] = temp;
    }
    updated = true;
    
}

int main(int argc, char** argv){
    ros::init(argc, argv, "local_map_node");
    ros::NodeHandle nh;
    ros::service::waitForService("metaDataService",5000);
    local_map l(nh);
    
    ros::Subscriber scan_sub = nh.subscribe("/frobit/scan",1,handle_scan);
    ros::Subscriber odom_sub = nh.subscribe("/frobit/odom",1,handle_odom);
    
    while(ros::ok()){
        if(updated){
            updated = false;
            for(int i = 0;i<nr_scans;i++){
                if(scans[i](0) > 0){
//                     stdD::cout << scans[i] << std::endl;
                Eigen::Vector3f res = t*scans[i];
                l.setMapData(res(0),res(1));
                }
            }
            l.visualize();
        }
        ros::spinOnce();
        
    }
    return 0;
}

