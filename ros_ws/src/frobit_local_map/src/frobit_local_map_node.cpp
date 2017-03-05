#include <grid_map_ros/grid_map_ros.hpp>
// #include <grid_map_msgs/GridMap.h>
#include <ros/ros.h>
#include<iostream>
#include"frobit_mapping/MetaData.h"
#include"frobit_mapping/GetPixelFromPosition.h"
#include"frobit_mapping/GetMapService.h"
#include<geometry_msgs/Transform.h>
// using namespace grid_map;

#define layer_name "local_map_occupancy"

std::vector<std::pair<int,int> > onMapBorder(double direction,double x, double y,ros::ServiceClient c);

int main(int argc, char** argv){
    ros::init(argc, argv, "local_map_node");
    ros::NodeHandle nh;
    grid_map::GridMap rawMap;
    rawMap.add(layer_name), 255;
    ros::ServiceClient meta_client = nh.serviceClient<frobit_mapping::MetaData>("metaDataService");
    ros::ServiceClient pixPos_client = nh.serviceClient<frobit_mapping::GetPixelFromPosition>("pixelFromPosService");
    ros::ServiceClient getMap_cluent = nh.serviceClient<frobit_mapping::GetMapService>("getMapService");
    double size,res;
    frobit_mapping::MetaData meta_data;
    meta_data.request.dummy = 1;
    if (meta_client.call(meta_data)){
        size = 3*meta_data.response.len;
        res = meta_data.response.resolution;
    }else{
        return 1;
    }
    
    frobit_mapping::GetPixelFromPosition pixPos;
    geometry_msgs::Transform t;
    t.translation.x = 31.5;
    t.translation.y = 16.1;
    pixPos.request.position = t;
    if (pixPos_client.call(pixPos)){
        std::cout << "Position: " << t.translation.x << ", " << t.translation.y << std::endl;
        std::cout << "Map nr: "<<pixPos.response.result[0] << ", "<<pixPos.response.result[1]  << std::endl;
        std::cout << "Offset: "<<pixPos.response.result[2] << ", "<<pixPos.response.result[3]  << std::endl;
    }else{
        return 1;
    }

    //getMetaData(); // Do service call to map database to get the size and resolution of the local map.
    grid_map::Length l(size,size);
    rawMap.setGeometry(l,res);
    std::vector<std::pair<int,int> > changed_maps;
    changed_maps = onMapBorder(0.0,35.0,20.0,pixPos_client);
    
    update_raw_map(rawMap);
    
    while(ros::ok()){
        ros::spin();
    }
    return 0;
}


//Direction is the current yaw angle between 0 and 360, x,y is the current position in the field frame.
std::vector<std::pair<int,int> > onMapBorder(double direction,double x, double y,ros::ServiceClient c){
    
    //Get the current map number from database
    frobit_mapping::GetPixelFromPosition pixPos;
    geometry_msgs::Transform t;
    t.translation.x = x;
    t.translation.y = y;
    pixPos.request.position = t;
    if (c.call(pixPos)){
        std::cout << "Position: " << t.translation.x << ", " << t.translation.y << std::endl;
        std::cout << "Map nr: "<<pixPos.response.result[0] << ", "<<pixPos.response.result[1]  << std::endl;
        std::cout << "Offset: "<<pixPos.response.result[2] << ", "<<pixPos.response.result[3]  << std::endl;
    }else{
        ROS_ERROR("Could not call position from pixel service");
    }
    
    //Find maps to replace the old once
    std::vector<std::pair<int,int> > new_maps;
    std::vector<std::pair<int,int> > old_maps;
    if(direction < 135 && direction >= 45){
        new_maps.push_back(std::make_pair(-1,-1));
        new_maps.push_back(std::make_pair(-1,0));
        new_maps.push_back(std::make_pair(-1,1));
    }else if(direction < 225 && direction >= 135){
        new_maps.push_back(std::make_pair(-1,-1));
        new_maps.push_back(std::make_pair(0,-1));
        new_maps.push_back(std::make_pair(1,-1));
    }else if(direction < 315 && direction >= 255){
        new_maps.push_back(std::make_pair(1,-1));
        new_maps.push_back(std::make_pair(1,0));
        new_maps.push_back(std::make_pair(1,1));
    }else if((direction < 45 && direction >= 0) ||(direction <= 360 && direction >= 315) ){
        new_maps.push_back(std::make_pair(-1,1));
        new_maps.push_back(std::make_pair(0,1));
        new_maps.push_back(std::make_pair(1,1));
    }else{
        ROS_ERROR("Direction error. Current direction = %f",direction);
    }
    
    for(auto ite : new_maps){
        old_maps.push_back(std::make_pair(ite.first*-1,ite.second*-1));
    }
    
    new_maps.insert(new_maps.end(),old_maps.begin(),old_maps.end());
    
    return new_maps;
}