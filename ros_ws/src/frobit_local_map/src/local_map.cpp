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
        
        //NOTE Should not be here. Only for demo purpose
//         visualize();
    }
}

void local_map::visualize(){
    nav_msgs::OccupancyGrid grid;
    grid_map::GridMapRosConverter::toOccupancyGrid(rawMap, layer_name ,0,1,grid);
    local_map_publisher.publish(grid);
}

grid_map::Position local_map::getMapPosition(double x, double y){
    grid_map::Position pos(static_cast<int>(x/size)*size,static_cast<int>(y/size)*size);
    return pos;
}

std::pair<MapNrV,MapNrV> local_map::onMapBorder(double direction,double x, double y){
    
    MapNrV new_maps;
    MapNrV old_maps;

    
    //Find maps to replace the old once
    if(direction == 2){
        new_maps.push_back(std::make_pair(-1,-1));
        new_maps.push_back(std::make_pair(-1,0));
        new_maps.push_back(std::make_pair(-1,1));
    }else if(direction == 4){
        new_maps.push_back(std::make_pair(-1,-1));
        new_maps.push_back(std::make_pair(0,-1));
        new_maps.push_back(std::make_pair(1,-1));
    }else if(direction == 8){
        new_maps.push_back(std::make_pair(1,-1));
        new_maps.push_back(std::make_pair(1,0));
        new_maps.push_back(std::make_pair(1,1));
    }else if(direction == 6 ){
        new_maps.push_back(std::make_pair(-1,1));
        new_maps.push_back(std::make_pair(0,1));
        new_maps.push_back(std::make_pair(1,1));
    }else{
        ROS_ERROR("Direction error. Current direction = %f",direction);
    }
    
    for(auto ite : new_maps){
        old_maps.push_back(std::make_pair(ite.first*-1,ite.second*-1));
    }
    
    //Get the current map number from database
    frobit_mapping::GetPixelFromPosition pixPos;
    geometry_msgs::Transform t;
    t.translation.x = x;
    t.translation.y = y;
    pixPos.request.position = t;
    if (pixPos_client.call(pixPos)){
        new_maps.push_back(std::make_pair(pixPos.response.result[0],pixPos.response.result[1])); 
        old_maps.push_back(std::make_pair(pixPos.response.result[0],pixPos.response.result[1]));// Hack that makes sure the right map is saved
    }else{
        ROS_ERROR("Could not call position from pixel service");
    }
    
    return std::make_pair(new_maps,old_maps);
}

void local_map::move(int dir, double x, double y){
    grid_map::Position p = getMapPosition(x,y);
    std::pair<MapNrV,MapNrV> map_numbers = onMapBorder(dir,x,y);
    saveMaps(map_numbers.second);
    rawMap.move(p);
    data_fill(map_numbers.first);
    visualize();
}

void local_map::saveMaps(MapNrV maps){
    
    std::pair<int,int> center = maps.back(); // Get the center map
    maps.pop_back(); // Remove center. We dont want to save that one
    std::cout << center.first << ", " << center.second << std::endl;
    nav_msgs::OccupancyGrid grid;
    
    bool success = false;
    
    for( auto ite : maps){
    std::cout << ite.first << ", " << ite.second << std::endl;
        grid_map::Position p((center.first + ite.first)*size,(center.second + ite.second)*size);
        grid_map::Length l(size,size);
        grid_map::GridMap sub = rawMap.getSubmap(p,l,success);
        
        if(success){
            grid_map::GridMapRosConverter::toOccupancyGrid(sub, layer_name,0,1,grid);
            frobit_mapping::SetMapService s;
            s.request.map = grid;
            s.request.x = center.first + ite.first;
            s.request.y = center.second + ite.second;
            if(!setMap_client.call(s)){
                ROS_ERROR("Could not save map nr (%d,%d)",center.first + ite.first,center.second+ite.second);
            }
        }
    }
}

bool local_map::setMapData(double x, double y){
    grid_map::Position pos(x,y);
    grid_map::Index index;
    if (!rawMap.getIndex(pos, index)){
        std::cout << "Position in map does not exist" << std::endl;
        return false;
    }else{
        auto& data = rawMap.at(layer_name,index);
        data = 1;
    }
    std::cout << "alive" << std::endl;
//     visualize();
}