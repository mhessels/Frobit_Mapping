#include"frobit_mapping/MetaData.h"
#include"frobit_mapping/GetPixelFromPosition.h"
#include"frobit_mapping/GetMapService.h"
#include"frobit_mapping/setMapService.h"

#include<geometry_msgs/Transform.h>
#include<nav_msgs/OccupancyGrid.h>

#include <ros/ros.h>

#include <grid_map_ros/grid_map_ros.hpp>

typedef std::vector<std::pair<int,int> > MapNrV;

class local_map{
private:
    ros::NodeHandle nh;
    
    ros::ServiceClient meta_client;
    ros::ServiceClient pixPos_client;
    ros::ServiceClient getMap_client;
    ros::ServiceClient setMap_client;
    ros::Publisher local_map_publisher;
        
    const std::string layer_name = "detected_obstacles";
    
    grid_map::GridMap rawMap;
    double size;
    double resolution;
    double start_x;
    double start_y;
    
    grid_map::Position current_pos;
    double current_rotation;
    
    MapNrV onMapBorder(double direction,double x, double y);
    void update_raw_map(grid_map::GridMap& map, MapNrV map_numbers, double res,double size);
    
    void init();
    
    void data_fill(MapNrV);
    
    bool initialized;
    
    //Gets the top left corner of the map, so data_fill puts the data in the right position.
    grid_map::Position getMapPosition(double x,double y);
public:
    local_map(ros::NodeHandle n);
    bool isInitialized(){return initialized;}
    void visualize();
    
    grid_map::Position getCurrentPosition(){return current_pos;}
    void setCurrentPosition(grid_map::Position p){current_pos = p;}
    
    double getCurrentRotation(){return current_rotation;}
    void setCurrentRotation(double r){current_rotation = r;}
    
    //Check wether a new map should be loaded
    bool checkForMapChange();
};