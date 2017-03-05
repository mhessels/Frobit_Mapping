#include <grid_map_ros/grid_map_ros.hpp>
#include <ros/ros.h>
#include<nav_msgs/OccupancyGrid.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "local_map_node");
    ros::NodeHandle nh;
    ros::Publisher p = nh.advertise<nav_msgs::OccupancyGrid>("map",1);
    grid_map::GridMap rawMap;
    rawMap.add("one"), 255;
    double size=15,res=0.05;
    grid_map::Length l(size,size);
    rawMap.setGeometry(l,res);
    
    
    
    double x=0,y=0;
    
    
    // -------------- Simple write test ---------------------- 
//     ros::Rate r(1);
//     
//     while(ros::ok()){
//         std::cout << x << ", " << y << std::endl;;
//         grid_map::Index index;
//         grid_map::Position position(x,y);
//         if (!rawMap.getIndex(position, index)) continue;
//         auto& data = rawMap.at("one",index);
//         
//         data = 0;
//         
//         
//         //Publish the changed map
//         nav_msgs::OccupancyGrid grid;
//         grid_map::GridMapRosConverter::toOccupancyGrid(rawMap, "one",0,1,grid);
//         
//         p.publish(grid);
//         
//         ros::spinOnce();
//         r.sleep();
//         x += 0.1;
//         
//         if(x == 10){
//             if(y == 10){
//                 ros::shutdown();
//             }else{
//                 y += 0.1;
//             }
//         x = 0;
//         }
//     }
    
    
     //--------------------- Submap test--------------------------
    grid_map::Position pos(0,0);
    bool success = false;
    grid_map::Length ll(5,5);

    grid_map::GridMap sub1 = rawMap.getSubmap(pos,ll,success);
    
    for(double x = 0;x<5;x+=0.02){
        for(double y= 0;y<5;y+=0.02){
            grid_map::Position position(x,y);
            grid_map::Index index;
            //Is the requested position valid
            if (!sub1.getIndex(position, index)) continue;
            
            auto& data = sub1.at("one",index);
            
            data = 0;
        }
    }
    
    
    //Publish the changed map
    nav_msgs::OccupancyGrid grid;
    grid_map::GridMapRosConverter::toOccupancyGrid(rawMap, "one",0,1,grid);
    
    p.publish(grid);
        
    return 0;
    
}