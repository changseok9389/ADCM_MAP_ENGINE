//
// Created by 오창석 on 2022/01/03.
//
#include <iostream>
#include "map_parser.h"
#include <cmath>
#include <algorithm>
#include <vector>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <algorithm>
#include <google/protobuf/text_format.h>

#include "COLORS.h"
#include <string>
#include "road_network.pb.h"
#include <fstream>

map_parser::map_parser(){
    resolution_multiplier = 1.0;
    displacement_offset = 3797.47 * resolution_multiplier;
    std::cout << "default initializer called" << std::endl;
}

map_parser::map_parser(std::string _map_type, std::string protopath){
    map_type = _map_type;
    pb_path = protopath;
    resolution_multiplier = 1.0;
    displacement_offset = 3797.47 * resolution_multiplier;
    ifs.open(pb_path, std::ios::in | std::ios::binary);
    if (ifs.is_open()){
        f.ParseFromIstream(&ifs);
        std::cout << "map parser initialized with file << " << pb_path << " >> " << std::endl;
    }

    else
        std::cout << "initialization failed... file open error" << std::endl;
    ifs.close();
}

bool map_parser::get_bounds() {
    for(auto element : lanes){
        cv::Rect boundingArea = cv::boundingRect(element.second);
//        boundingArea.tl();
//        boundingArea.br() - cv::Point(1,1);
        bounds[element.first] = std::pair(boundingArea.tl(), boundingArea.br() - cv::Point(1,1));
        std::cout << "(" << bounds[element.first].first.x << ", " << bounds[element.first].first.y << "), (" << bounds[element.first].second.x << ", " << bounds[element.first].second.y << ")" << std::endl;
    }
    return true;
}

bool map_parser::load_map()
{
    if (pb_path == "")
        std::cout << "load failed... unset protobuf path" << std::endl;

    ifs.open(pb_path, std::ios::in | std::ios::binary);
    if (ifs.is_open()){
        f.ParseFromIstream(&ifs);
        std::cout << "load map file << " << pb_path << " >> " << std::endl;
        ifs.close();
        return true;
    }

    else
        std::cout << "load failed... file open error" << std::endl;
    ifs.close();
    return false;

}

bool map_parser::ADCM_load_map() {
    setMapFilePath(pb_path.c_str());
    ADCM_MAP.push_back(requestMapDataInGrid_ID(1035988));
}


bool map_parser::parse_map(){
    // extract polygons
    for (auto k = f.elements().begin(); k < f.elements().end(); k++){
        // extract lane polygons
//        std::vector<cv::Point> lane;
        std::vector<cv::Point> lane_l;
        std::vector<cv::Point> lane_r;

        if (k->element().element_case() == 3){
//            lane.clear();
            lane_l.clear();
            lane_r.clear();
            l5kit::maps::Lane Lane = k->element().lane();

            double frame_lat = Lane.geo_frame().origin().lat_e7() / 1e7;
            double frame_lng = Lane.geo_frame().origin().lng_e7() / 1e7;

//            std::cout << frame_lat << ", " << frame_lng << std::endl;

            // parse left lane
            int size = Lane.left_boundary().vertex_deltas_x_cm_size();
            double x = 0, y = 0;
            for (int i = 0; i < size; i++){
                x += Lane.left_boundary().vertex_deltas_x_cm(i) / (double)100.0 * resolution_multiplier;
                y += Lane.left_boundary().vertex_deltas_y_cm(i) / (double)100.0 * resolution_multiplier;


                lane_l.push_back(cv::Point((int)(x+displacement_offset), (int)(y+displacement_offset)));
            }

            // parse right lane
            size = Lane.right_boundary().vertex_deltas_x_cm_size();
            x = 0, y = 0;
            for (int i = 0; i < size; i++){
                x += Lane.right_boundary().vertex_deltas_x_cm(i) / (double)100.0 * resolution_multiplier;
                y += Lane.right_boundary().vertex_deltas_y_cm(i) / (double)100.0 * resolution_multiplier;

                lane_r.push_back(cv::Point((int)(x+displacement_offset), (int)(y+displacement_offset)));
//                lane.insert(lane.begin(), cv::Point((int)(x+3797.47), (int)(y+3797.47)));
            }

            std::reverse(lane_r.begin(), lane_r.end());
            lane_l.insert(lane_l.end(), lane_r.begin(), lane_r.end());
            lanes[k->id().id()] = lane_l;

        }
    }
    return true;
}

bool map_parser::ADCM_parse_map() {
    for(auto map : ADCM_MAP){
        // parse Roads
        for(auto v : map.vRoads){
            // parse lane
            for(auto l : v.vLanes){
            }

            // parse line
            for(auto l : v.vLines){

            }
        }

        // parse Border
        for(auto v : map.vBorders){

        }

        // parse Junction
        for(auto v : map.vJunctions){

        }

        // parse Stop Line
        for (auto v : map.vStopLines){

        }

        // parse Crosswalk
        for (auto v : map.vCrosswalks_out){

        }

        // parse Parkinglot
        for (auto v : map.vParkinglots){

        }
    }

}

void map_parser::set_resolution_multiplier(double value){
    resolution_multiplier = value;
    displacement_offset *= resolution_multiplier;
}

double map_parser::get_resolution_multiplier(){
    return resolution_multiplier;
}

void map_parser::set_proto_path(std::string path){
    pb_path = path;
}
std::string map_parser::get_proto_path(){
    return pb_path;
}

void map_parser::draw_map(){
    cv::Mat img(12000 * resolution_multiplier,6000 * resolution_multiplier, CV_8UC3, WHITE);
    int counter = 0;
    for(auto l = lanes.begin(); l != lanes.end(); l++){
        counter++;
        cv::fillPoly(img, {l->second}, BLUE);
        cv::polylines(img, {l->second}, true, BLACK, 1, cv::LINE_AA);

    }
    latest_map = img;
    cv::imwrite("/home/changseok/Desktop/ADCM_MAP_ENGINE/img.png", img);
}


void map_parser::show_map(int fov, int loc_x, int loc_y){
    cv::Rect boader(0, 0, latest_map.cols, latest_map.rows);
    cv::Rect bounds(loc_x, loc_y, fov, fov);
    cv::Mat roi = latest_map(boader& bounds);
    cv::imshow("ROI", roi);
    cv::waitKey(0);
    std::cout << "(x, y) : (" << loc_x << ", " << loc_y << ")" << std::endl;
    return ;
}

cv::Mat map_parser::get_map(int fov, int loc_x, int loc_y){
    cv::Rect boader(0, 0, latest_map.cols, latest_map.rows);
    cv::Rect bounds(loc_x, loc_y, fov, fov);
    cv::Mat roi = latest_map(boader& bounds);

    return roi;
}


void map_parser::show_map_debug(int fov){
    int outer_crop = fov * sqrt(2);
    char in;
    int loc_x = latest_map.cols/2;
    int loc_y = latest_map.rows/2;

    while(true){
        std::cout << "(x, y) : (" << loc_x << ", " << loc_y << ")" << std::endl;
        cv::Rect boader(0, 0, latest_map.cols, latest_map.rows);
        cv::Rect outer_bounds(loc_x, loc_y, outer_crop, outer_crop);
        cv::Rect bounds(loc_x, loc_y, fov, fov);
        cv::Mat roi = latest_map(boader& bounds);
        cv::imshow("ROI", roi);
        in = cv::waitKey(0);
        std::cout << (int)in << std::endl;
        switch((int)in){
            case 82:
                loc_y -= 10;
                break;
            case 84:
                loc_y += 10;
                break;
            case 81:
                loc_x -= 10;
                break;
            case 83:
                loc_x += 10;
                break;
            default:
                return;
        }

    }


}