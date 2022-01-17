//
// Created by 오창석 on 2022/01/03.
//

#ifndef ADCM_MAP_ENGINE_MAP_PARSER_H
#define ADCM_MAP_ENGINE_MAP_PARSER_H

#include <string>
#include "road_network.pb.h"
#include <fstream>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "MapStructs.h"
#include "MDAL.h"


class map_parser {
private:
    std::vector<LOCAL_MAP> ADCM_MAP;
    std::string map_type;
    std::string pb_path;
    l5kit::maps::MapFragment f;
    std::fstream ifs;
    double resolution_multiplier;
    double displacement_offset;
    std::unordered_map<std::string, std::pair<cv::Point, cv::Point>> bounds;
    std::unordered_map<std::string, std::vector<cv::Point>> lanes;

    cv::Mat latest_map;


public:
    // initializer
    map_parser();
    map_parser(std::string _map_type, std::string protopath);
    // get bounds of each element of interest
    bool get_bounds();
    // load map file locate in protopath
    bool load_map();
    // [ADCM] load map
    bool ADCM_load_map();
    // parse map polygons from map object
    bool parse_map();
    // [ADCM]parse_map
    bool ADCM_parse_map();
    // set resolution_multiplier
    void set_resolution_multiplier(double value);
    // get resolution_multiplier
    double get_resolution_multiplier();
    // set protopath
    void set_proto_path(std::string path);
    // get protopath
    std::string get_proto_path();
    // render map image
    void draw_map();
    // show specific location of map image
    void show_map(int fov, int loc_x, int loc_y);
    cv::Mat get_map(int fov, int loc_x, int loc_y);
    // for debug
    void show_map_debug(int fov);




};


#endif //ADCM_MAP_ENGINE_MAP_PARSER_H
