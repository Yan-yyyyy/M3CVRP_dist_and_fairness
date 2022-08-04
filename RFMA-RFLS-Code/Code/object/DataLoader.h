//
// Created by Joe Lan on 2020/7/3.
//

#ifndef UWCP_DATALOADER_H
#define UWCP_DATALOADER_H

#include <iostream>
#include "Point.h"
#include "Vehicle.h"
#include <unordered_map>
#include "json/json.h"
#include <cmath>
#include <fstream>
#include <unordered_set>
using namespace std;

extern int dist_matrix[3005][3005];

class DataLoader {
private:
    string m_v_file_path;
    string m_c_file_path;
    string m_p_file_path;
    string m_d_file_path;
    string m_dist_file_path;

public:

    int point_index = 0;

    unordered_set<int> p_d_set;
    unordered_map<string, int> id_index_dict;
    unordered_map<int, Point> collection_site_data;
    unordered_map<int, Point> depot_data;
    unordered_map<int, Point> disposal_facility_data;
//    unordered_map<pair<int, int>, int, pair_hash> dist_dict;

    unordered_map<int, Vehicle> vehicle_data;
    unordered_map<int, vector<int> > c_neighbor_dict;
    unordered_set<int> no_served_c;
    unordered_map<int, vector<int>> depot_vehicle_dict;
    
    unordered_map<int, vector<int>> depot_vehicle_dict_copy;

    DataLoader(const string & v_file_path, const string & c_file_path, const string & p_file_path, const string & d_file_path, const string & dist_file_path);
    static Json::Value get_json_data(const string & data_file_path);
    void get_collection_site_data(const string & file_path);
    void get_depot_data(const string & file_path);
    void get_disposal_facility_data(const string & file_path);
    void get_distance_data(const string & dist_file_path, int threshold);
    void get_vehicle_data(const string & file_path);

};


#endif //UWCP_DATALOADER_H
