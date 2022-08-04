//
// Created by Joe Lan on 2020/7/3.
//


#include "DataLoader.h"
int dist_matrix[3005][3005];

DataLoader::DataLoader(const string &v_file_path, const string &c_file_path, const string &p_file_path,
                       const string &d_file_path, const string &dist_file_path) {
    this->m_v_file_path = v_file_path;
    this->m_c_file_path = c_file_path;
    this->m_p_file_path = p_file_path;
    this->m_d_file_path = d_file_path;
    this->m_dist_file_path = dist_file_path;

    this->get_collection_site_data(this->m_c_file_path);
    this->get_depot_data(this->m_p_file_path);
    this->get_disposal_facility_data(this->m_d_file_path);
    this->get_distance_data(this->m_dist_file_path, 5000);
    this->get_vehicle_data(m_v_file_path);
    this->depot_vehicle_dict_copy = this->depot_vehicle_dict;
}

Json::Value DataLoader::get_json_data(const string &data_file_path) {
    Json::Value root;
    ifstream data_file(data_file_path, ifstream::binary);
    Json::CharReaderBuilder readerBuilder;
    string errs;
    if(!Json::parseFromStream(readerBuilder, data_file, &root, &errs)){
        cout << errs << endl;
        return EXIT_FAILURE;
    }
    return root;
}

void DataLoader::get_collection_site_data(const string &file_path) {
    Json::Value root = get_json_data(file_path);
    int site_number = root["point"].size();

    Json::Value individual_info;
    for(int i=0; i<site_number; i++){
        individual_info = root["point"][i];
        Point collection = Point(
                individual_info["id"].asString(),
                this->point_index,
                individual_info["lat"].asInt(),
                individual_info["lng"].asInt(),
                individual_info["stopLength"].asInt(),
                individual_info["capacity"].asInt(),
                3 ,
                8*3600);
        this->id_index_dict.insert(pair<string, int>(collection.getPointId(), this->point_index));
        this->collection_site_data.insert(pair<int, Point>(this->point_index, collection));
        this->no_served_c.insert(point_index);
        this->point_index += 1;
    }

}

void DataLoader::get_depot_data(const string &file_path) {
    Json::Value root = get_json_data(file_path);
    int depot_number = root["depot"].size();

    Json::Value individual_info;
    for(int i=0; i<depot_number; i++){
        individual_info = root["depot"][i];
        Point depot = Point(individual_info["id"].asString(), this->point_index, individual_info["lat"].asInt(), individual_info["lng"].asInt(), 0, 0, 2, INT32_MAX);
        this->id_index_dict.insert(pair<string, int>(depot.getPointId(), this->point_index));
        this->depot_data.insert(pair<int, Point>(this->point_index, depot));
        this->p_d_set.insert(this->point_index);
        this->point_index += 1;
    }

}

void DataLoader::get_disposal_facility_data(const string &file_path) {
    Json::Value root = get_json_data(file_path);

    int disposal_facility_number = root["disposal_facility"].size();

    Json::Value individual_info;
    for(int i=0; i<disposal_facility_number; i++){
        individual_info = root["disposal_facility"][i];

        int p_index;
        auto p_it = this->id_index_dict.find(individual_info["id"].asString());
        if(p_it == id_index_dict.end()){
            p_index = this->point_index;
            point_index += 1;
        }else{
            p_index = p_it->second;
        }

        Point disposal_facility = Point(
                    individual_info["id"].asString(),
                    p_index,
                    individual_info["lat"].asInt(),
                    individual_info["lng"].asInt(),
                    0,
                    0,
                    4,
                    INT32_MAX
                );
        this->id_index_dict.insert(pair<string, int>(disposal_facility.getPointId(), p_index));
        this->disposal_facility_data.insert(pair<int, Point>(p_index, disposal_facility));
        this->p_d_set.insert(p_index);
    }
}

void DataLoader::get_distance_data(const string & dist_file_path, int threshold) {
    ifstream data_file(dist_file_path);
    string line_text;
    getline(data_file, line_text);
    string data;
    int start_index = -1;
    int end_index = -1;
    int dist = 0;

    while(getline(data_file, line_text)){
        istringstream str_reader(line_text);

        getline(str_reader, data, ',');
        start_index = this->id_index_dict.find(data)->second;
        getline(str_reader, data, ',');
        end_index = this->id_index_dict.find(data)->second;
        getline(str_reader, data, ',');
        dist = (int)stof(data);

        dist_matrix[start_index][end_index] = dist;
        auto start_it = p_d_set.find(start_index);
        if(start_it == p_d_set.end()){
            auto end_it = p_d_set.find(end_index);
            if(end_it == p_d_set.end()){
                if(dist <= threshold){
                    auto res_it = this->c_neighbor_dict.find(start_index);
                    if(res_it == this->c_neighbor_dict.end()){
                        vector<int> neighbor_vector = {end_index};
                        this->c_neighbor_dict.emplace(pair<int, vector<int> >(start_index, neighbor_vector));

                    }else{
                        res_it->second.emplace_back(end_index);
                    }
                }
            }
        }

    }
}

void DataLoader::get_vehicle_data(const string &file_path) {
    Json::Value root = get_json_data(file_path);
    int vehicle_number = root["vehicle"].size();

    Json::Value individual;
    for(int i=0; i< vehicle_number; i++){
        individual = root["vehicle"][i];
        Vehicle vehicle = Vehicle(individual["id"].asString(), individual["stopId"].asString(), i, id_index_dict[individual["stopId"].asString()], individual["maxCapacity"].asInt()*100, 12.5, 8*3600, 0, 0);
        vehicle_data.insert(pair<int, Vehicle> (i, vehicle));
        auto it = this->depot_vehicle_dict.find(vehicle.getParkingIndex());
        if(it==depot_vehicle_dict.end()){
           vector<int> vehicle_vector = {i};
           this->depot_vehicle_dict.insert(pair<int, vector<int> >(vehicle.getParkingIndex(), vehicle_vector));
        }else{
            it->second.emplace_back(i);
        }
    }
}