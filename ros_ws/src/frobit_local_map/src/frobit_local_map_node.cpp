#include"frobit_local_map/local_map.h"

#define layer_name "local_map_occupancy"

typedef std::vector<std::pair<int,int> > MapNrV;

MapNrV onMapBorder(double direction,double x, double y,ros::ServiceClient c);

int main(int argc, char** argv){
    ros::init(argc, argv, "local_map_node");
    ros::NodeHandle nh;
    local_map l(nh);
    
    l.visualize();
    
    l.move(8,16,0);
    l.move(8,32,0);
//     l.move(8,8,0);
//     l.move(8,15,0);
    
}


//Direction is the current yaw angle between 0 and 360, x,y is the current position in the field frame.
MapNrV onMapBorder(double direction,double x, double y,ros::ServiceClient c){
    
    MapNrV new_maps;
    MapNrV old_maps;

    //Get the current map number from database
    frobit_mapping::GetPixelFromPosition pixPos;
    geometry_msgs::Transform t;
    t.translation.x = x;
    t.translation.y = y;
    pixPos.request.position = t;
    if (c.call(pixPos)){
        new_maps.push_back(std::make_pair(pixPos.response.result[0],pixPos.response.result[1]));
    }else{
        ROS_ERROR("Could not call position from pixel service");
    }
    
    //Find maps to replace the old once
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