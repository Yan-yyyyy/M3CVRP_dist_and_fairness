//
// Created by Joe Lan on 2020/7/6.
//

#include "LocalSearch.h"
#include <fstream>


LocalSearch::LocalSearch(DataLoader & dataLoader): m_dataLoader(dataLoader) {
    srand(time(nullptr));
}

int LocalSearch::get_random_number(unsigned int begin, unsigned int end) {
    return (int)((rand()%(end-begin+1))+begin);
}

int LocalSearch::calculate_single_route_distance(const vector<int> &complete_route) {
    int total_dist = 0;
    for(int i=0; i<complete_route.size()-1; i++){
        total_dist += dist_matrix[complete_route[i]][complete_route[i+1]];

    }
    return total_dist;
}

pair<int, float>
LocalSearch::calculate_single_route_dist_and_time_cost(const vector<int> &complete_route, float speed) {
    int total_dist = 0;
    float total_time_cost = 0;
    for(int i=0; i<complete_route.size()-1; i++){
        total_dist += dist_matrix[complete_route[i]][complete_route[i+1]];

        auto it = this->m_dataLoader.collection_site_data.find(complete_route[i]);
        if(it != this->m_dataLoader.collection_site_data.end()){
            total_time_cost += float(it->second.getServerCost());
        }
    }
    total_time_cost += float(total_dist)/speed;

    return make_pair(total_dist, total_time_cost);
}

pair<int, pair<float,int>>
LocalSearch::calculate_single_route_dist_and_time_cost_and_capacity(const vector<int> &complete_route, float speed) {
    int total_dist = 0;
    float total_time_cost = 0;
    int total_capacity = 0;
    for(int i=0; i<complete_route.size()-1; i++){
        total_dist += dist_matrix[complete_route[i]][complete_route[i+1]];
        auto it = this->m_dataLoader.collection_site_data.find(complete_route[i]);
        if(it != this->m_dataLoader.collection_site_data.end()){
            total_time_cost += float(it->second.getServerCost());
            total_capacity += it->second.getPointCapacity();
        }
    }
    total_time_cost += float(total_dist)/speed;

    return make_pair(total_dist, make_pair(total_time_cost,total_capacity));
}

pair<int, double> LocalSearch::multiobjective_fitness_function(const unordered_map<int, vector<int> > &solution, int capacity, const string &strategy) {
    int total_dist = 0;
    vector<double> time_list;
    vector<double> capacity_list;
    for(auto & it: solution){
        vector<int> complete_solution = this->insert_for_single_route(it.second, capacity, strategy);
        pair<int, pair<float,int>> result_pair = this->calculate_single_route_dist_and_time_cost_and_capacity(complete_solution, 12.5);//distance and (time and capacity)
        total_dist += result_pair.first;
        time_list.push_back((double)result_pair.second.first);
        capacity_list.push_back((double)result_pair.second.second);
    }
    double time_fairness=this->calculateStandardDeviation(time_list);
    double capacity_fairness=this->calculateStandardDeviation(capacity_list);
    return make_pair(total_dist,time_fairness+capacity_fairness);
}

// 极差标准化
vector<double> LocalSearch::minMaxNormalization(const vector<double>& data) {
    if (data.empty()) {
        return data;
    }
    double minVal = *std::min_element(data.begin(), data.end());
    double maxVal = *std::max_element(data.begin(), data.end());
    if (maxVal == minVal) {
        vector<double> normalizedData(data.size(), 0.0);
        return normalizedData;
    }
    vector<double> normalizedData;
    for (double value : data) {
        double normalizedValue = (value - minVal) / (maxVal - minVal);
        normalizedData.push_back(normalizedValue);
    }

    return normalizedData;
}

double LocalSearch::calculateStandardDeviation(const vector<double>& data) {
    vector<double> norm_data = this->minMaxNormalization(data);
    if (norm_data.empty()) {
        return 0.0;
    }
    double mean = std::accumulate(norm_data.begin(), norm_data.end(), 0.0) / norm_data.size();
    double variance = 0.0;
    for (double value : norm_data) {
        variance += (value - mean) * (value - mean);
    }
    variance /= norm_data.size();
    return std::sqrt(variance);
}

int LocalSearch::get_closest_disposal_facility(int start, int end) {
    int min_dist = INT32_MAX;
    int closet_d = -1;
    int dist = 0;
    for(auto & it: this->m_dataLoader.disposal_facility_data){
        dist = dist_matrix[start][it.first] + dist_matrix[it.first][end];
        if(dist < min_dist){
            min_dist = dist;
            closet_d = it.first;
        }
    }
    return closet_d;
}

pair<int, int> LocalSearch::get_closest_park(int start, int end) {
    int min_dist = INT32_MAX;
    int closest_p = -1;
    int closest_d = -2;
    int dist = 0;
    for(auto & p_it: this->m_dataLoader.depot_data){
        for(auto & d_it: this->m_dataLoader.disposal_facility_data){
            dist = long(dist_matrix[p_it.first][start] + dist_matrix[end][d_it.first] + dist_matrix[d_it.first][p_it.first]);
            if(dist<min_dist){
                min_dist = dist;
                closest_d = d_it.first;
                closest_p = p_it.first;
            }
        }
    }
    return make_pair(closest_p, closest_d);
}

vector<int> LocalSearch::insert_disposal_facility_with_greedy(const vector<int> &route, int car_capacity) {
    vector<int> update_route;
    int current_capacity = 0;

    for(int i=0; i<route.size(); i++){
        current_capacity += this->m_dataLoader.collection_site_data.find(route[i])->second.getPointCapacity();
        if(current_capacity>car_capacity){
            int closest_d = this->get_closest_disposal_facility(update_route.back(), route[i]);
            update_route.emplace_back(closest_d);
            current_capacity = this->m_dataLoader.collection_site_data.find(route[i])->second.getPointCapacity();
            update_route.emplace_back(route[i]);
        }else if(current_capacity == car_capacity){
            if(route[i]!=route.back()){
                update_route.emplace_back(route[i]);
                int closest_d = this->get_closest_disposal_facility(route[i], route[i+1]);
                update_route.emplace_back(closest_d);
                current_capacity = 0;
            }else{
                update_route.emplace_back(route[i]);
            }
        }else{
            update_route.emplace_back(route[i]);
        }
    }
    pair<int, int> p_d_pair = this->get_closest_park(update_route.front(), update_route.back());
    update_route.insert(update_route.begin(), p_d_pair.first);
    update_route.emplace_back(p_d_pair.second);
    update_route.emplace_back(p_d_pair.first);

    return update_route;
}

void LocalSearch::inner_insert(const vector<int> &original_route, int capacity, int win_len, int begin_index, int end_index, vector<int> new_route, vector<int> &best_route, int & minimum_dist) {
    // cout << "hello world" << endl;
    if(begin_index!=-1 && end_index!=-1){
        for(int i=begin_index; i<=end_index; i++){
            new_route.emplace_back(original_route[i]);
        }
        if(end_index == original_route.size()-1){
            pair<int, int> p_d_pair = this->get_closest_park(new_route.front(), new_route.back());
            new_route.insert(new_route.begin(), p_d_pair.first);
            new_route.emplace_back(p_d_pair.second);
            new_route.emplace_back(p_d_pair.first);
            int total_dist = this->calculate_single_route_distance(new_route);
            // cout << "total_dist = " << total_dist << endl;
            if(total_dist < minimum_dist){
                minimum_dist = total_dist;
                best_route = new_route;
            }
            return;
        }
        int disposal_facility = this->get_closest_disposal_facility(original_route[end_index], original_route[end_index+1]);
        new_route.emplace_back(disposal_facility);
    }
    int current_capacity = 0;
    for(int i=end_index+1; i<original_route.size(); i++){
        // cout << "i=" << i << endl;
        current_capacity += this->m_dataLoader.collection_site_data.find(original_route[i])->second.getPointCapacity();
        if(current_capacity>capacity){
            for(int j=0; j<win_len; j++){
                inner_insert(original_route, capacity, win_len, end_index+1, i-1-j, new_route, best_route, minimum_dist);

            }
            return;
        }else if(current_capacity==capacity){
            for(int j=0; j<win_len; j++){
                inner_insert(original_route, capacity, win_len, end_index+1, i-j, new_route, best_route, minimum_dist);
            }
            return;
        }else if(i==original_route.size()-1 && current_capacity<=capacity){
            inner_insert(original_route, capacity, win_len, end_index+1, i, new_route, best_route, minimum_dist);
            current_capacity = 0;
        }
    }
}

vector<int>
LocalSearch::insert_disposal_facility_with_backspacing(const vector<int> &route, int car_capacity, int window_length) {
    int minimum_dist = INT32_MAX;
    vector<int> best_route;
    vector<int> new_route;
    this->inner_insert(route, car_capacity, window_length, -1, -1, new_route, best_route, minimum_dist);
    return best_route;
}

vector<int>
LocalSearch::insert_for_single_route(const vector<int> &single_route, int car_capacity, const string &strategy) {
    vector<int> complete_route;
    if(strategy=="Greedy"){
        // cout << "here" << endl;
        complete_route = this->insert_disposal_facility_with_greedy(single_route, car_capacity);
        // cout << "there" << endl;
    }else{
        complete_route = this->insert_disposal_facility_with_backspacing(single_route, car_capacity, this->window_len);
    }
    return complete_route;
}

unordered_map<int, vector<int> > LocalSearch::insert_for_all_route(const unordered_map<int, vector<int> > &solution, int car_capacity, const string &strategy) {
    unordered_map<int, vector<int> > complete_solution;
    if(strategy=="Greedy"){
        for(auto & it: solution){
            vector<int> complete_route = this->insert_disposal_facility_with_greedy(it.second, car_capacity);
            complete_solution.emplace(pair<int, vector<int> >(it.first, complete_route));
        }
    }else{
        for(auto & it: solution){
            vector<int> complete_route = this->insert_disposal_facility_with_backspacing(it.second, car_capacity, this->window_len);
            complete_solution.emplace(pair<int, vector<int> >(it.first, complete_route));
        }
    }
    return complete_solution;
}

int LocalSearch::get_total_distance(unordered_map<int, vector<int> > &complete_solution) {
    int total_dist = 0;
    for(auto & it: complete_solution){
        total_dist += calculate_single_route_distance(it.second);
    }
    return total_dist;
}

unordered_map<int, vector<int> > LocalSearch::delete_double_d(unordered_map<int, vector<int> > &solution) {
    unordered_map<int, vector<int> > solution_without_ds;
    for(auto & it: solution){
        vector<int> new_route;
        for(auto site_it: it.second){
            auto res_it = this->m_dataLoader.p_d_set.find(site_it);
            if(res_it==this->m_dataLoader.p_d_set.end()){
                new_route.emplace_back(site_it);
            }
        }
        solution_without_ds.emplace(pair<int, vector<int> >(it.first, new_route));
    }
    return solution_without_ds;
}

vector<int> LocalSearch::insert_operator(vector<int> &route, int car_capacity, const string &strategy) {
    vector<int> all_route = this->insert_for_single_route(route, car_capacity, strategy);
    int current_dist = this->calculate_single_route_distance(all_route);

    vector<int> new_route = route;
    int index = this->get_random_number(0, new_route.size()-1);
    int c_point = new_route[index];
    new_route.erase(new_route.begin()+index);

    vector<int> better_route = route;

    for(int i=0; i<=new_route.size(); i++){
        vector<int> new_route_copy = new_route;
        new_route_copy.insert(new_route_copy.begin()+i, c_point);

        vector<int> new_all_route = this->insert_for_single_route(new_route_copy, car_capacity, strategy);
        int new_dist = this->calculate_single_route_distance(new_all_route);
        if(new_dist<current_dist){
            better_route = new_route_copy;
            current_dist = new_dist;
        }
    }
    return better_route;
}

void LocalSearch::swap(int &start, int &end) {
    if(end < start){
        int temp = start;
        start = end;
        end = temp;
    }
}

vector<int> LocalSearch::exchange_operator(vector<int> &route, int car_capacity, int interval, const string & strategy) {
    vector<int> all_route = this->insert_for_single_route(route, car_capacity, strategy);
    int original_dist = this->calculate_single_route_distance(all_route);
    vector<int> route_copy = route;
    int index = this->get_random_number(0, route_copy.size()-1);
    int index2 = this->get_random_number(max(0, index-interval), min(index+interval, (int)route_copy.size()-1));

    while (index==index2){
        index2 = this->get_random_number(max(0, index-interval), min(index+interval, (int)route_copy.size()-1));
    }
    this->swap(index, index2);

    route_copy[index] = route[index2];
    route_copy[index2] = route[index];

    vector<int> new_all_route = this->insert_for_single_route(route_copy, car_capacity, strategy);
    int new_dist = this->calculate_single_route_distance(new_all_route);
    if(new_dist<original_dist){
        return route_copy;
    }else{
        return route;
    }
}

vector<int> LocalSearch::reverse_operator(vector<int> &route, int car_capacity, int interval, const string &strategy) {
    vector<int> all_route = this->insert_for_single_route(route, car_capacity, strategy);
    int origin_dist = this->calculate_single_route_distance(all_route);

    vector<int> route_copy = route;
    int index = this->get_random_number(0, route_copy.size()-1);
    int index2 = this->get_random_number(max(0, index-interval), min(index+interval, (int)route_copy.size()-1));
    while (index==index2){
        index2 = this->get_random_number(max(0, index-interval), min(index+interval, (int)route_copy.size()-1));
    }
    this->swap(index,index2);
    for(int i=0; i<=index2-index; i++){
        route_copy[index+i] = route[index2-i];
    }
    vector<int> new_all_route = this->insert_for_single_route(route_copy, car_capacity, strategy);
    int new_dist = this->calculate_single_route_distance(new_all_route);

    if(new_dist<origin_dist){
        return route_copy;
    }else{
        return route;
    }
}

void LocalSearch::mutation_with_classic_operator(unordered_map<int, vector<int> > &solution, int iteration, int car_capacity, const string &strategy, int operator_name) {
    for(int i=0; i<iteration; i++){
        for(auto & it: solution){
            if(it.second.size() > 1){
                if(operator_name==1){
                    it.second = this->insert_operator(it.second, car_capacity, strategy);
                }else if(operator_name==2){
                    it.second = this->reverse_operator(it.second, car_capacity, (int)ceil(it.second.size()/13.0)+1, strategy);
                }else{
                    it.second = this->exchange_operator(it.second, car_capacity, (int)ceil(it.second.size()/7.0)+1, strategy);
                }
            }
            
        }
    }
}

unordered_map<int, int> LocalSearch::get_site_route_dict(const unordered_map<int, vector<int> > &solution) {
    unordered_map<int, int> c_route_dict;
    int total_length = 0;
    for(auto & it: solution){
        total_length += it.second.size();
        for(int it2: it.second){
            c_route_dict.emplace(pair<int, int>(it2, it.first));
        }
    }
    return c_route_dict;
}

vector<int> LocalSearch::get_key_vector(const unordered_map<int, vector<int> > & solution) {
    vector<int> key_vector;

    for(auto & it: solution){
        key_vector.emplace_back(it.first);
    }

    return key_vector;
}

int LocalSearch::find_index(const vector<int> &goal_vector, int element) {
    int index = -1;
    for(int i=0; i<goal_vector.size();i++){
        if(goal_vector[i]==element){
            index = i;
            break;
        }
    }
    return index;
}

void LocalSearch::region_constrained_single_point_swap(unordered_map<int, vector<int> > &solution, int iteration, int car_capacity, const string &strategy) {
    vector<int> key_vector = this->get_key_vector(solution);
    int r_index = this->get_random_number(0, solution.size()-1);
    int r_key = key_vector[r_index];
    auto route_it = solution.find(r_key);

    int length = route_it->second.size();

    for(int j=0; j<length-1; j++){
        int site = route_it->second[j];
        unordered_map<int, int> c_route_dict = this->get_site_route_dict(solution);
        auto neighbor_it = this->m_dataLoader.c_neighbor_dict.find(site);
        if(neighbor_it == this->m_dataLoader.c_neighbor_dict.end()){
            continue;
        }
        int neighbor_length = neighbor_it->second.size();

        for(int i=0; i<iteration; i++){
            int index = this->get_random_number(0, neighbor_length-1);
            int new_p = neighbor_it->second[index];
            int new_route_id = c_route_dict[new_p];
            if(new_route_id == r_key){
                continue;
            }else{
                auto new_route_it = solution.find(new_route_id);
                vector<int> original_r = route_it->second;
                vector<int> another_r = new_route_it->second;

                int old_dist = this->calculate_single_route_distance(this->insert_for_single_route(original_r, car_capacity, strategy))
                        + this->calculate_single_route_distance(this->insert_for_single_route(another_r, car_capacity, strategy));
                int another_index = find_index(another_r, new_p);
                original_r[j] = new_p;
                another_r[another_index] = site;

                int new_dist = this->calculate_single_route_distance(this->insert_for_single_route(original_r, car_capacity, strategy))
                               + this->calculate_single_route_distance(this->insert_for_single_route(another_r, car_capacity, strategy));

                if(new_dist<old_dist){
                    route_it->second[j] = new_p;
                    new_route_it->second[another_index] = site;
                    break;
                }
            }

        }
    }
}

void LocalSearch::swap_segment(vector<int> &original_r, vector<int> &another_r, int original_index, int another_index) {
    vector<int> original_route = original_r;
    vector<int> another_route = another_r;

    original_r.erase(original_r.begin()+original_index, original_r.end());
    another_r.erase(another_r.begin()+another_index, another_r.end());

    for(int i=another_index; i<another_route.size(); i++){
        original_r.emplace_back(another_route[i]);
    }


    for(int i=original_index; i<original_route.size(); i++){
        another_r.emplace_back(original_route[i]);
    }
}

void LocalSearch::region_constrained_segment_swap(unordered_map<int, vector<int> > &solution, int iteration, int capacity, float time_threshold, float speed, const string & strategy) {
    vector<int> key_vector = this->get_key_vector(solution);
    int r_key = key_vector[this->get_random_number(0, key_vector.size()-1)];
    auto route_it = solution.find(r_key);
    unordered_map<int, int> c_route_dict = this->get_site_route_dict(solution);

    int r_length = route_it->second.size();

    for(int i=0; i<iteration; i++){
        int original_index = this->get_random_number(0, r_length-1);
        int point = route_it->second[original_index];

        auto neighbor_it = this->m_dataLoader.c_neighbor_dict.find(point);
        if(neighbor_it == this->m_dataLoader.c_neighbor_dict.end()){
            continue;
        }
        int index = get_random_number(0, neighbor_it->second.size()-1);
        int new_p = neighbor_it->second[index];
        int new_route_id = c_route_dict.find(new_p)->second;

        if(new_route_id==r_key){
            continue;
        }else{
            auto new_route_it = solution.find(new_route_id);
            vector<int> original_r = route_it->second;
            vector<int> another_r = new_route_it->second;
            pair<int, float> original_r_pair = this->calculate_single_route_dist_and_time_cost(this->insert_for_single_route(original_r, capacity, strategy), speed);
            pair<int, float> another_r_pair = this->calculate_single_route_dist_and_time_cost(this->insert_for_single_route(another_r, capacity, strategy), speed);

            int old_dist = original_r_pair.first+another_r_pair.first;
            int another_index = this->find_index(another_r, new_p);

            swap_segment(original_r, another_r, original_index, another_index);

            pair<int, float> new_original_pair = this->calculate_single_route_dist_and_time_cost(this->insert_for_single_route(original_r, capacity, strategy), speed);
            pair<int, float> new_another_pair = this->calculate_single_route_dist_and_time_cost(this->insert_for_single_route(another_r, capacity, strategy), speed);
            int new_dist = new_original_pair.first+new_another_pair.first;
            if(new_dist<old_dist && new_original_pair.second<=time_threshold && new_another_pair.second<=time_threshold){
                route_it->second = original_r;
                new_route_it->second = another_r;
                break;
            }
        }

    }
}

void LocalSearch::mutation_with_novel_operator(unordered_map<int, vector<int> > &solution, int outer_iteration, int inner_iteration, int capacity, int operator_name, const string & strategy) {
    for(int i=0; i<outer_iteration; i++){
        if(operator_name==1){
            this->region_constrained_single_point_swap(solution, inner_iteration, capacity, strategy);
        }else{
            this->region_constrained_segment_swap(solution, inner_iteration, capacity, 8*3600, 12.5, strategy);

        }
    }
}

void LocalSearch::relaxed_multi_point_swap(unordered_map<int, vector<int> > &solution, int dist_threshold, int capacity, float percentage, float speed, float time_threshold, const string &strategy) {
    vector<int> key_vector = this->get_key_vector(solution);
    int r_index_1 = this->get_random_number(0, key_vector.size()-1);
    int r_key_1 = key_vector[r_index_1];
    int r_key_2 = key_vector[r_index_1];

    while(r_key_1 == r_key_2){
        r_key_2 = key_vector[this->get_random_number(0, key_vector.size()-1)];
    }
    auto route_1_it = solution.find(r_key_1);
    auto route_2_it = solution.find(r_key_2);

    int exchange_num_1 = ceil(route_1_it->second.size()*percentage);
    int exchange_num_2 = ceil(route_2_it->second.size()*percentage);

    vector<int> exchange_list_1(exchange_num_1);
    vector<int> exchange_list_2(exchange_num_2);

    vector<int> route_1 = route_1_it->second;
    vector<int> route_2 = route_2_it->second;

    int index;
    int r_index = 0;

    for(int i=0; i<exchange_num_1; i++){
        // cout << "j=" << i << endl;
        index = this->get_random_number(0, route_1.size()-1);
        exchange_list_1[r_index] = route_1[index];
        route_1.erase(route_1.begin()+index);
        r_index += 1;
    }
    r_index = 0;
    for(int i=0; i<exchange_num_2; i++){
        // cout << "k=" << i << endl;
        index = this->get_random_number(0, route_2.size()-1);
        exchange_list_2[r_index] = route_2[index];
        route_2.erase(route_2.begin()+index);
        r_index += 1;
    }

    // cout << "nothing" << endl;

    for(int site: exchange_list_1){
        int best_dist = INT32_MAX;
        int insert_pos = -1;

        for(int j=0; j<route_2.size()+1; j++){
            route_2.insert(route_2.begin()+j, site);
            vector<int> complete_route = this->insert_for_single_route(route_2, capacity, strategy);
            int temp_dist = this->calculate_single_route_distance(complete_route);
            if(temp_dist<best_dist){
                insert_pos = j;
                best_dist = temp_dist;
            }
            route_2.erase(route_2.begin()+j);
        }
        route_2.insert(route_2.begin()+insert_pos, site);
    }

    for(int site: exchange_list_2){
        int best_dist = INT32_MAX;
        int insert_pos = -1;

        for(int j=0;j<route_1.size()+1; j++){
            route_1.insert(route_1.begin()+j, site);
            vector<int> complete_route = this->insert_for_single_route(route_1, capacity, strategy);
            int temp_dist = this->calculate_single_route_distance(complete_route);
            if(temp_dist<best_dist){
                insert_pos = j;
                best_dist = temp_dist;
            }
            route_1.erase(route_1.begin()+j);
        }
        route_1.insert(route_1.begin()+insert_pos, site);
    }
    // cout << "here100" << endl;

    int old_dist = this->calculate_single_route_distance(this->insert_for_single_route(route_1_it->second, capacity, strategy)) +
            this->calculate_single_route_distance(this->insert_for_single_route(route_2_it->second, capacity, strategy));
    pair<int, int> route_1_pair = this->calculate_single_route_dist_and_time_cost(this->insert_for_single_route(route_1, capacity, strategy), speed);
    pair<int, int> route_2_pair = this->calculate_single_route_dist_and_time_cost(this->insert_for_single_route(route_2, capacity, strategy), speed);
    int new_dist = route_1_pair.first + route_2_pair.first;
    if(new_dist<=old_dist+dist_threshold && route_1_pair.second<=time_threshold && route_2_pair.second<=time_threshold){
        route_1_it->second = route_1;
        route_2_it->second = route_2;
    }

}

void LocalSearch::mutation_with_relaxed_multi_point_swap(unordered_map<int, vector<int> > &solution, int iteration, int capacity, int dist_threshold, float percentage, float speed, float time_threshold, const string &strategy) {
    int dist = INT32_MAX;
    unordered_map<int, vector<int> > result_solution = solution;
    for(int i=0; i<iteration; i++){
        // cout << "i=" << i << endl;
        this->relaxed_multi_point_swap(solution, dist_threshold, capacity, percentage, speed, time_threshold, strategy);
        unordered_map<int, vector<int> > complete_solution = this->insert_for_all_route(solution, capacity, strategy);
        int new_dist = this->get_total_distance(complete_solution);

        if(new_dist < dist){
            dist = new_dist;
            result_solution = solution;
        }
    }
    solution = result_solution;
}

void LocalSearch::optimize_solution(unordered_map<int, vector<int> > &solution, int runtime, const string & strategy) {
    cout << "2958" << endl;
    vector<int> best_distance_vector;
    string file_name = "/home/yezy/cplusplus_code/UWCP/solution_release/ELS_70_45/" + to_string(time(nullptr)) + ".txt";


    ofstream f;
    f.open(file_name);
    f.close();

    fstream f1;
    f1.open(file_name, ios::out);
    f1 << "no violate constraints, shortest distance, more detail, time_cost = 1 min" << endl;
    f1 << "solution_size = " << solution.size() << endl;

    clock_t start_time, finish_time;
    start_time = clock();
    int dist_threshold = 2000;
    unordered_map<int, vector<int> > best_solution = solution;
    int best_dist = INT32_MAX;
    int time_index = 1;
    while(time_index<=runtime){
        int judge_break = 0;
        int best_distance = INT32_MAX;

        while(true){
            this->mutation_with_novel_operator(solution, 100, 10, 72000, 1, strategy);

            this->mutation_with_novel_operator(solution, 100, 100, 72000, 2, strategy);

            vector<int> random_method = {1, 2, 3};
            shuffle(random_method.begin(), random_method.end(), std::default_random_engine(time(nullptr)));

            for(auto & it: random_method){
                this->mutation_with_classic_operator(solution, 100, 72000, strategy, it);
            }
            unordered_map<int, vector<int> > complete_solution = this->insert_for_all_route(solution, 72000, strategy);
            int current_dist = this->get_total_distance(complete_solution);
            if(current_dist < best_distance){
                if(current_dist<=best_distance-dist_threshold){
                    judge_break = 0;
                }else{
                    judge_break += 1;
                }
                best_distance = current_dist;
                if(current_dist < best_dist){
                    best_dist = current_dist;
                    best_solution = solution;
                }
                
            }else{
                judge_break += 1;
            }
            
            finish_time = clock();
            if(((finish_time-start_time)/CLOCKS_PER_SEC) >= time_index * 3000){
                best_distance_vector.emplace_back(best_dist);
                f1 << "after " << time_index << " * 50 minutes, best_dist = " << best_dist << endl;
                time_index += 1;
            }
            if(judge_break>=5){
                break;
            }
        }

        if(best_dist < best_distance){
            solution = best_solution;
        }
        if(solution.size()>=2){
            this->mutation_with_relaxed_multi_point_swap(solution, 100, 72000, 45000, 0.20, 12.5, 8*3600, strategy);

            unordered_map<int, vector<int> > complete_solution = this->insert_for_all_route(solution, 72000, strategy);
            int current_dist = this->get_total_distance(complete_solution);
            if(current_dist < best_dist){
                best_solution = solution;
            }

            finish_time = clock();
            if(((finish_time-start_time)/CLOCKS_PER_SEC) >= time_index * 104256){
                time_index += 1;
            }
        }
        
        
    }
    show_solution(best_solution, f1);
    for(int bd: best_distance_vector){
        f1 << bd << endl;
    }
    this->valid_solution(best_solution, 8*3600, f1);
    unordered_map<int, vector<int> > complete_solution = this->insert_for_all_route(best_solution, 72000, "Backspacing");
    int total_dist = LocalSearch::get_total_distance(complete_solution);
    f1 << "after insert total_dist = " << total_dist << endl;
    f1.close();
}

void LocalSearch::show_solution(unordered_map<int, vector<int> > &solution) {
    cout << "solution = {";
    for(auto & it: solution){
        cout << "{" << it.first << ", {";
        for(int site: it.second){
            if(site!=it.second.back()){
                cout << site << ", ";
            }else{
                cout << site << "}, ";
            }

        }
        cout << "};" << endl;
    }
}

void LocalSearch::show_solution(unordered_map<int, vector<int> > &solution, fstream & fstream1) {
    fstream1 << "solution = {";
    for(auto & it: solution){
        fstream1 << "{" << it.first << ", {";
        for(int i=0; i<it.second.size(); i++){
            int site = it.second[i];
            if(i!=it.second.size()-1){
                fstream1 << site << ", ";
            }else{
                fstream1 << site << "}, ";
            }

        }
        fstream1 << "};" << endl;
    }
}

void LocalSearch::check_solution(const unordered_map<int, vector<int> > &solution) {
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

void LocalSearch::optimizee_solution_ma(unordered_map<int, vector<int> > &solution, int iteration, const string &strategy) {
    int dist_threshold = 2000;
    unordered_map<int, vector<int> > best_solution = solution;
    int best_dist = INT32_MAX;
    int time_index = 1;
    while(time_index<=iteration){
        int judge_break = 0;
        int best_distance = INT32_MAX;
        while(true){
            // clock_t st = clock();
            // cout << st << endl;
            this->mutation_with_novel_operator(solution, 40, 10, 72000, 1, strategy);
            // clock_t ct = clock();
            // cout << "time_cost = " << ct-st<< endl;
            this->mutation_with_novel_operator(solution, 40, 20, 72000, 2, strategy);
            // cout << "tc = " << clock()-ct << endl;
            vector<int> random_method = {1, 2, 3};
            shuffle(random_method.begin(), random_method.end(), std::mt19937(std::random_device()()));
            for(auto & it: random_method){
                this->mutation_with_classic_operator(solution, 40, 72000, strategy, it);
            }
            unordered_map<int, vector<int> > complete_solution = this->insert_for_all_route(solution, 72000, strategy);
            int current_dist = this->get_total_distance(complete_solution);
            
            if(current_dist < best_distance){
                if(current_dist<=best_distance-dist_threshold){
                    // cout << current_dist << " cur vs best dist " << best_distance << endl;
                    judge_break = 0;
                }else{
                    judge_break += 1;
                }
                best_distance = current_dist;

                if(current_dist < best_dist){
                    best_dist = current_dist;
                    best_solution = solution;
                }
            }else{
                judge_break += 1;
            }
            if(judge_break>=5){
                break;
            }
        }
        time_index += 1;
        if(best_dist < best_distance){
            solution = best_solution;
        }
        if(solution.size()>=2){
            this->mutation_with_relaxed_multi_point_swap(solution, 100, 72000, 45000, 0.20, 12.5, 8*3600, strategy);
        }
    }
    solution = best_solution;
}

void LocalSearch::valid_solution(const unordered_map<int, vector<int> > &solution, float time_constraints, fstream &f1) {
    int time_cons = 0;
    int total_dist = 0;

    for(auto & it: solution){
        vector<int> complete_route = this->insert_for_single_route(it.second, 72000, "Backspacing");
        pair<int, float> dist_time_pair = this->calculate_single_route_dist_and_time_cost(complete_route, 12.5);
        total_dist += dist_time_pair.first;
        if(dist_time_pair.second > time_constraints){
            time_cons += 1;
            f1 << "route = " << it.first << ", time_cost = " << dist_time_pair.second/3600 << " h" << endl;
        }
    }

    f1 << "total_dist = " << total_dist << ", time constraints violate = " << time_cons << endl;
}

int LocalSearch::valid_solution(const unordered_map<int, vector<int> > &solution, float time_constraints) {
    int time_cons = 0;
    int total_dist = 0;

    for(auto & it: solution){
        pair<int, float> dist_time_pair = this->calculate_single_route_dist_and_time_cost(it.second, 12.5);
        total_dist += dist_time_pair.first;
        if(dist_time_pair.second > time_constraints){
            time_cons += 1;
            cout << "route = " << it.first << ", time_cost = " << dist_time_pair.second/3600 << " h" << endl;
        }
    }

    cout << "total_dist = " << total_dist << ", time constraints violate = " << time_cons << endl;
    return time_cons;
}

