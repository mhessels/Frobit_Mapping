#include"frobit_local_map/local_map.h"

local_map::local_map(ros::NodeHandle n){
    nh = n;
    initialized = false;
    meta_client = nh.serviceClient<frobit_mapping::MetaData>("metaDataService");
    pixPos_client = nh.serviceClient<frobit_mapping::GetPixelFromPosition>("pixelFromPosService");
    getMap_client = nh.serviceClient<frobit_mapping::GetMapService>("getMapService");
    setMap_client = nh.serviceClient<frobit_mapping::SetMapService>("setMapService");
    local_map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("map",1);
    
    //Get the map meta data from the database
    frobit_mapping::MetaData meta_data;
    meta_data.request.dummy = 1;
    if (meta_client.call(meta_data)){
        size = meta_data.response.len;
        resolution = meta_data.response.resolution;
    }else{
        ROS_ERROR("Could not call metaDataService");
        initialized = false;
        return;
    }
    
    rawMap.add(layer_name);
    
    grid_map::Length l(size*3,size*3);
    rawMap.setGeometry(l,resolution);
    
    //Move to initial position 
    start_x = 0;//GET ROS PARAM
    start_y = 0;//GET ROS PARAM
    
    current_pos[0] = start_x;
    current_pos[1] = start_y;
    
    // Set the center of the local map to the current center
    rawMap.move(getMapPosition(start_x,start_y));
    
    init();
    initialized = true;
}

void local_map::init(){


    MapNrV map_numbers;
    map_numbers.push_back(std::make_pair(0,0));
    map_numbers.push_back(std::make_pair(-1,-1));
    map_numbers.push_back(std::make_pair(0,-1));
    map_numbers.push_back(std::make_pair(1,-1));
    map_numbers.push_back(std::make_pair(-1,0));
    map_numbers.push_back(std::make_pair(1,0));
    map_numbers.push_back(std::make_pair(-1,1));
    map_numbers.push_back(std::make_pair(0,1));
    map_numbers.push_back(std::make_pair(1,1));
    
    //Add the center map number. Not used for getting this map.
    map_numbers.push_back(std::make_pair(0,0));
    
    data_fill(map_numbers);
   
}

void local_map::data_fill(MapNrV map_numbers){
    
    std::pair<int,int> center = map_numbers.back();
    map_numbers.pop_back();
    int centerx = center.first;
    int centery = center.second;
        
    frobit_mapping::GetMapService s;

    grid_map::GridMap temp_local_map;
    temp_local_map.add("tmp");
    
    for(auto ite : map_numbers){ // Start from index 1, because 0 is center map
        s.request.x = centerx + ite.first;
        s.request.y = centery + ite.second;
        
        if (getMap_client.call(s)){
            if(s.response.succes){
                grid_map::GridMapRosConverter::fromOccupancyGrid(s.response.map,"tmp",temp_local_map);

            }else{
                ROS_ERROR("Internal Datebase error when getting the map (%d , %d)", s.request.x,s.request.y);
                return;
                initialized = false;
            }
        }else{
            ROS_ERROR("Could not get the map  (%d , %d) Service Error", s.request.x,s.request.y);
            initialized = false;
        }

    //Datafill
     for(double i = 0.001;i < size;i+= resolution){
            for(double j = 0.001;j< size ; j += resolution){
            grid_map::Position global_pos(s.request.x*size+i-size/2,s.request.y*size+j-size/2);
            grid_map::Index i_global;
            grid_map::Position pos(i,j);
            grid_map::Index index;
            
            // Test global point validity
            if (!rawMap.getIndex(global_pos, i_global)){
                 std::cout << "Global position not found" << std::endl;
                continue;
            }
            if (!temp_local_map.getIndex(pos, index)){ // Test new point validity
                std::cout << "Local position not found" << std::endl;
                continue;
            }
            
            auto& data_l = temp_local_map.at("tmp",index);
            auto& data_g = rawMap.at(layer_name,i_global);
            
            data_g = data_l;
            

            }
        }
    }
}

void local_map::visualize(){
    nav_msgs::OccupancyGrid grid;
    grid_map::GridMapRosConverter::toOccupancyGrid(rawMap, layer_name ,0,1,grid);
    local_map_publisher.publish(grid);
}

grid_map::Position local_map::getMapPosition(double x, double y){
    grid_map::Position pos(static_cast<int>(start_x/size)*size,static_cast<int>(start_y/size)*size);
    return pos;
}