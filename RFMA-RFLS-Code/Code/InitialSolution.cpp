//
// Created by Joe Lan on 2020/7/6.
// The implementation of heuristic-assisted solution initialisation (HaSI).
//

#include "InitialSolution.h"

InitialSolution::InitialSolution(DataLoader &dataLoader) : m_dataLoader(dataLoader) {

}

pair<int, int> InitialSolution::drop_selection(int cur_p_index) {
    int min_dist = INT32_MAX;
    int drop = -1;

//    pair<int, int> query_pair;
//    query_pair.first = cur_p_index;
    int current_dist = -4;
    for(auto & it: this->m_dataLoader.disposal_facility_data){
//        query_pair.second = it.first;
//        current_dist = this->m_dataLoader.dist_dict[query_pair];
        current_dist = dist_matrix[cur_p_index][it.first];
        if(current_dist < min_dist){
            min_dist = current_dist;
            drop = it.first;
        }
    }
    pair<int, int> result = make_pair(drop, min_dist);
    return result;
}

pair<int, int> InitialSolution::get_drop_according_to_depot(int site_index, int depot) {
    int min_dist = INT32_MAX;
    int drop = -1;

//    pair<int, int> query_pair;
//    query_pair.first = site_index;
//    pair<int, int> d_p_pair;
//    d_p_pair.second = depot;
    for(auto & it: this->m_dataLoader.disposal_facility_data){
//        query_pair.second = it.first;
//        d_p_pair.first = it.first;

//        int current_dist = this->m_dataLoader.dist_dict[query_pair] + this->m_dataLoader.dist_dict[d_p_pair];
        int current_dist = dist_matrix[site_index][it.first] + dist_matrix[it.first][depot];
        if(current_dist < min_dist){
            min_dist = current_dist;
            drop = it.first;
        }
    }

    pair<int, int> result = make_pair(drop, min_dist);
    return result;
}

pair<int, int> InitialSolution::collection_selection(int cur_p_index, int vehicle_index) {
    int min_dist = INT32_MAX;
    int result = -1;

    bool time_limit = false;
    bool capacity_limit = false;

    Vehicle vehicle = this->m_dataLoader.vehicle_data.find(vehicle_index)->second;

    for(auto it: this->m_dataLoader.no_served_c){
        if(vehicle.getCurrentCapacity()+m_dataLoader.collection_site_data.find(it)->second.getPointCapacity()<=vehicle.getMaxCapacity()){
            capacity_limit = true;
            int temp = INT32_MAX;

            int dist = -1;
            for(auto & d_it: this->m_dataLoader.disposal_facility_data){
                dist = dist_matrix[it][d_it.first] + dist_matrix[d_it.first][vehicle.getParkingIndex()];
                if(dist <= temp){
                    temp = dist;
                }
            }
            int distance = dist_matrix[cur_p_index][it];
            auto c_it = this->m_dataLoader.collection_site_data.find(it);
            if(vehicle.getCurrentTimeCost()+(temp+distance)/vehicle.getSpeed()+c_it->second.getServerCost()<=vehicle.getTotalTimeCost()){
                time_limit = true;
                if(min_dist > distance){
                    min_dist = distance;
                    result = it;
                }
            }
        }

    }
    pair<int, int> result_pair = make_pair(result, min_dist);
    int flag = -10;
    if(time_limit && capacity_limit){
        flag = 3;
    }else if((!time_limit) && capacity_limit){
        flag = 2;
        result_pair.second = flag;
    }else{
        flag = 0;
        result_pair.second = flag;
    }

    return result_pair;
}

pair<int, int> InitialSolution::arrange_vehicle() {
    unordered_map<int, int> count;
    for(auto & it: this->m_dataLoader.depot_data){
        count.insert(pair<int, int>(it.first, 0));
    }

//    pair<int, int> query_pair;
    for(auto it: this->m_dataLoader.no_served_c){
        int depot = -1;
        int dist = INT32_MAX;
        int current_dist = 0;
        for(auto & map_it: count){
//            query_pair = make_pair(map_it.first, it);
//            current_dist = this->m_dataLoader.dist_dict[query_pair];
            current_dist = dist_matrix[map_it.first][it];
            if (current_dist < dist){
                depot = map_it.first;
                dist = current_dist;
            }
        } //find the closest depot of each collection
        count[depot] += 1; //count<depot number, number of collections whose closest depot is it>
    }
    vector<pair<int,int> > temp(this->m_dataLoader.depot_data.size());
    int index = 0;
    for(auto & it: count){ //it is pair<depot, number of closest collections>
        temp[index] = it;
        index += 1;
    }
    sort(temp.begin(), temp.end(), [](const pair<int, int> & a, const pair<int, int> & b)->int{return a.second>b.second;});
    pair<int, int> result;

    for(auto & pair_it: temp){
        result.second = pair_it.first;
        if(!this->m_dataLoader.depot_vehicle_dict[result.second].empty()){
            result.first = this->m_dataLoader.depot_vehicle_dict[result.second].back();
            this->m_dataLoader.depot_vehicle_dict[result.second].pop_back();
            break;
        }
    }
    return result;
}

unordered_map<int, vector<int> > InitialSolution::init_solution() {
    float time_rate = this->getTimeRate();
    float capacity_rate = this->getCapacityRate();
    // cout << time_rate << " " << capacity_rate << endl;
    unordered_map<int, vector<int> > solution;
    int vehicle_number = this->m_dataLoader.vehicle_data.size();
    while(!this->m_dataLoader.no_served_c.empty()){
        // cout << m_dataLoader.no_served_c.size() << endl;
        if(solution.size()>=vehicle_number){
            cout << "no enough vehicle!";
            break;
        }else{
            pair<int, int> vehicle_depot_pair = this->arrange_vehicle();
            vector<int> single_route = {vehicle_depot_pair.second};
            int current_site = vehicle_depot_pair.second;
            auto v_it = this->m_dataLoader.vehicle_data.find(vehicle_depot_pair.first);
            float speed = v_it->second.getSpeed();
            int max_capacity = v_it->second.getMaxCapacity();
            float total_time_cost = v_it->second.getTotalTimeCost();

            while(true){
                int site_index = -1;
                int flag = -2;
                if(v_it->second.getCurrentTimeCost() >= total_time_cost*time_rate || this->m_dataLoader.no_served_c.empty()){
                    flag = 2; // no enough time
                }else if(v_it->second.getCurrentCapacity() >= int(max_capacity*capacity_rate)){
                    flag = 0; //no enough capacity
                }else{
                    pair<int, int> result_pair = this->collection_selection(current_site, vehicle_depot_pair.first);
                    site_index = result_pair.first;
                    flag = result_pair.second;
                }
                if(site_index >= 0){
                    current_site = site_index;
                    single_route.emplace_back(current_site);
                    auto site_it = this->m_dataLoader.collection_site_data.find(current_site);
                    v_it->second.setCurrentCapacity(site_it->second.getPointCapacity());
                    v_it->second.setCurrentTimeCost((flag/speed+site_it->second.getServerCost()));
                    this->served_c.insert(current_site);
                    auto c_it = this->m_dataLoader.no_served_c.find(current_site);
                    this->m_dataLoader.no_served_c.erase(c_it);
                }else{
                    if(flag == 2){
                        auto result_it = this->m_dataLoader.p_d_set.find(single_route.back());
                        if(result_it != this->m_dataLoader.p_d_set.end()){
                            single_route.pop_back();
                        }
                        current_site = single_route.back();
                        pair<int, int> result_pair = this->get_drop_according_to_depot(current_site, single_route.front());
                        single_route.emplace_back(result_pair.first);
                        single_route.emplace_back(single_route.front());
                        v_it->second.setCurrentTimeCost((result_pair.second/speed));
                        break;
                    }else if(flag == 0){
                        pair<int, int> result_pair = this->drop_selection(current_site);
                        single_route.emplace_back(result_pair.first);
                        v_it->second.setCurrentCapacity(-v_it->second.getCurrentCapacity());
                        v_it->second.setCurrentTimeCost((result_pair.second/speed));
                        current_site = result_pair.first;
                    };
                }

            }

            solution.emplace(pair<int, vector<int> >(v_it->first, single_route));
        }
    }

    return solution;
}

void InitialSolution::check_solution(const unordered_map<int, vector<int> > &solution) {
    unordered_map<int, int> check_map;
    for(int i=0; i<this->m_dataLoader.point_index; i++){
        check_map.insert(pair<int, int> (i,0));
    }

    for(auto & single_route: solution){
        for(int site: single_route.second){
            auto site_it = check_map.find(site);
            if(site_it==check_map.end()){
                cout << site << endl;
            }
            site_it->second += 1;
        }
    }

    for(auto it: check_map){
        auto it2 = this->m_dataLoader.p_d_set.find(it.first);
        if(it2 == this->m_dataLoader.p_d_set.end()){
            if(it.second!=1){
                cout << it.first << " " << it.second << endl;
            }
        }
    }
}

float InitialSolution::getTimeRate() const {
    return t_rate;
}

void InitialSolution::setTimeRate(float timeRate) {
    t_rate = timeRate;
}

float InitialSolution::getCapacityRate() const {
    return c_rate;
}

void InitialSolution::setCapacityRate(float capacityRate) {
    c_rate = capacityRate;
}
