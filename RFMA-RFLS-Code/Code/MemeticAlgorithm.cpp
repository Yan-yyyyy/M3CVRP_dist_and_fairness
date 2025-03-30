//
// Created by Joe Lan on 2020/7/16.
//

#include "MemeticAlgorithm.h"

#include <iostream>
using namespace std;

MemeticAlgorithm::MemeticAlgorithm(LocalSearch &localSearch, int pSize, int opSize, float mPLS, unordered_map<int, unordered_map<int, vector<int> > > &populationDict):local_search(localSearch), p_size(pSize), op_size(opSize), pls(mPLS), population_dict(populationDict) {
    this->generator.seed(time(nullptr));
}

double MemeticAlgorithm::generate_random_number() {
    uniform_real_distribution<double> distribution(0.0, 1.0);
    return distribution(this->generator);
}
void MemeticAlgorithm::delete_double_d_for_population() {
    for(auto & it: this->population_dict){
        it.second = this->local_search.delete_double_d(it.second);
    }
}

void MemeticAlgorithm::sequence_base_crossover(vector<int> &first_route, vector<int> &second_route) {
    int first_route_size = first_route.size();
    int second_route_size = second_route.size();

    int difference = abs((first_route_size-second_route_size));
    int first_index = LocalSearch::get_random_number(0, first_route_size-1);
    int second_index = LocalSearch::get_random_number(max(0, first_index-difference), min(first_index+difference, second_route_size-1));

    vector<int> first_r_copy = first_route;
    vector<int> second_r_copy = second_route;
    first_route.erase(first_route.begin()+first_index, first_route.end());
    for(int i=second_index; i<second_route_size; i++){
        first_route.emplace_back(second_r_copy[i]);
    }


    second_route.erase(second_route.begin()+second_index, second_route.end());
    for(int i=first_index; i<first_route_size; i++){
        second_route.emplace_back(first_r_copy[i]);
    }
}

vector<int> MemeticAlgorithm::get_key_vector(unordered_map<int, unordered_map<int, vector<int> > > &population_map) {
    vector<int> key_vector(population_map.size());
    int index = 0;
    for(auto & it: population_map){
        key_vector[index] = it.first;
        index += 1;
    }
    return key_vector;
}

void MemeticAlgorithm::delete_repeat_and_insert_missing(unordered_map<int, vector<int> > &solution, int capacity,
                                                        const string &strategy,
                                                        int route_id, const vector<int> &original_route) {
    // there are two options here. One is exactly same as the paper; the other one is delete the worst one among all
    // route in solution and insert into the best position among all routes in solution.
    // The code below is exactly same as the paper.
    unordered_map<int, int> collection_route_dict;

    for(auto & r_it: solution){
        if(r_it.first!=route_id){
            for(int site: r_it.second){
                collection_route_dict.emplace(pair<int, int>(site, r_it.first));
            }
        }
    }

    // the sites in this set which are need to be inserted.
    unordered_set<int> insert_set;
    for(int site: original_route){
        insert_set.emplace(site);
    }

    auto r_it = solution.find(route_id);
    // the copy of the route which has do crossover.
    vector<int> route_copy = r_it->second;
    int delete_index  = 0;
    for(int i=0; i<r_it->second.size(); i++){
        int site = r_it->second[i];
        auto i_it = insert_set.find(site);
        if(i_it==insert_set.end()){
            auto d_it = collection_route_dict.find(site);
            if(d_it!=collection_route_dict.end()){
                route_copy.erase(route_copy.begin()+(i-delete_index));
                delete_index += 1;
            }
        }else{
            insert_set.erase(i_it);
        }
    }

    for(int site: insert_set){
        int best_dist = INT32_MAX;
        int insert_pos = -1;

        for(int i=0; i<=route_copy.size(); i++){
            route_copy.insert(route_copy.begin()+i, site);
            vector<int> complete_route = this->local_search.insert_for_single_route(route_copy, capacity, strategy);
            int temp_dist = LocalSearch::calculate_single_route_distance(complete_route);
            if(temp_dist<best_dist){
                best_dist = temp_dist;
                insert_pos = i;
            }
            route_copy.erase(route_copy.begin()+i);
        }

        route_copy.insert(route_copy.begin()+insert_pos, site);
    }
    r_it->second = route_copy;
}

void MemeticAlgorithm::delete_repeat_and_insert_missing_v2(unordered_map<int, vector<int> > &solution, int capacity, int car_number, const string &strategy, int route_id, const vector<int> &original_route) {
    unordered_map<int, vector<int> > collection_route_dict;
    for(auto & s_it: this->local_search.m_dataLoader.collection_site_data){
        vector<int> route_id_vector;
        collection_route_dict.emplace(pair<int, vector<int> >(s_it.first, route_id_vector));
    }
    for(auto & r_it: solution){
        for(int site: r_it.second){
            auto s_it = collection_route_dict.find(site);
            s_it->second.emplace_back(r_it.first);
        }
    }

    vector<int> insert_list;
    vector<int> delete_list;

    for(const auto& c_it: collection_route_dict){
        if(c_it.second.size()>=2){
            delete_list.emplace_back(c_it.first);
        }else if(c_it.second.empty()){
            insert_list.emplace_back(c_it.first);
        }
    }

    shuffle(delete_list.begin(), delete_list.end(), std::default_random_engine(time(nullptr)));
    for(int d_site: delete_list){
        auto d_it = collection_route_dict.find(d_site);
        int maximum_cost = INT32_MIN;
        vector<int> new_route;
        int delete_route_id = -1;

        for(int r_id: d_it->second){
            auto route_it = solution.find(r_id);
            vector<int> old_complete_route = this->local_search.insert_for_single_route(route_it->second, capacity, strategy);
            int old_dist = LocalSearch::calculate_single_route_distance(old_complete_route);

            vector<int> route_copy = route_it->second;
            int remove_index = LocalSearch::find_index(route_copy, d_site);
            route_copy.erase(route_copy.begin()+remove_index);
            int saving_dist = 0;
            if(!route_copy.empty()){
                vector<int> new_complete_route = this->local_search.insert_for_single_route(route_copy, capacity, strategy);
                int new_dist = LocalSearch::calculate_single_route_distance(new_complete_route);
                saving_dist = old_dist-new_dist;
            }else{
                saving_dist = old_dist;
            }
            if(saving_dist>maximum_cost){
                maximum_cost = saving_dist;
                delete_route_id = r_id;
                new_route = route_copy;
            }
        }

        solution.find(delete_route_id)->second = new_route;
    }

    shuffle(insert_list.begin(), insert_list.end(), std::default_random_engine(time(nullptr)));

    for(int i_site: insert_list){
        int minimum_cost = INT32_MAX;
        int insert_route_id = -1;
        vector<int> new_route;

        for(const auto& r_it: solution){
            vector<int> old_complete_route = this->local_search.insert_for_single_route(r_it.second, capacity, strategy);
            int old_dist = LocalSearch::calculate_single_route_distance(old_complete_route);

            for(int i=0; i<=r_it.second.size(); i++){
                vector<int> route_copy = r_it.second;
                route_copy.insert(route_copy.begin()+i, i_site);

                vector<int> new_complete_route = this->local_search.insert_for_single_route(route_copy, capacity, strategy);
                int new_dist = LocalSearch::calculate_single_route_distance(new_complete_route);

                if(new_dist-old_dist < minimum_cost){
                    minimum_cost = new_dist-old_dist;
                    route_id = r_it.first;
                    new_route = route_copy;
                }
            }
        }
        solution.find(route_id)->second = new_route;
    }
}

unordered_map<int, vector<int>> MemeticAlgorithm::crossover(int capacity, const string &strategy) {
    // this key_vector is for population_dict
    vector<int> key_vector = this->get_key_vector(this->population_dict);
    // randomly choose two solution from population_dict
    int first_s_index = LocalSearch::get_random_number(0, key_vector.size()-1);
    int second_s_index = LocalSearch::get_random_number(0, key_vector.size()-1);
    while(first_s_index==second_s_index){
        second_s_index = LocalSearch::get_random_number(0, key_vector.size()-1);
    }
    auto first_solution_it = this->population_dict.find(key_vector[first_s_index]);
    auto second_solution_it = this->population_dict.find(key_vector[second_s_index]);

    unordered_map<int, vector<int> > first_solution = first_solution_it->second;
    unordered_map<int, vector<int> > second_solution = second_solution_it->second;

    vector<int> first_key_vector = LocalSearch::get_key_vector(first_solution);
    vector<int> second_key_vector = LocalSearch::get_key_vector(second_solution);

    int first_r_index = LocalSearch::get_random_number(0, first_solution_it->second.size()-1);
    int second_r_index = LocalSearch::get_random_number(0, second_solution_it->second.size()-1);

    int first_route_id = first_key_vector[first_r_index];
    int second_route_id = second_key_vector[second_r_index];

    auto first_route_it = first_solution.find(first_route_id);
    auto second_route_it = second_solution.find(second_route_id);

    this->sequence_base_crossover(first_route_it->second, second_route_it->second);

    // this could be change
    this->delete_repeat_and_insert_missing(first_solution, capacity, strategy, first_route_id, first_solution_it->second.find(first_route_id)->second);

    this->delete_repeat_and_insert_missing(second_solution, capacity, strategy, second_route_id, second_solution_it->second.find(second_route_id)->second);

    unordered_map<int, vector<int> > first_complete_solution = this->local_search.insert_for_all_route(first_solution, capacity, strategy);
    int first_solution_dist = LocalSearch::get_total_distance(first_complete_solution);

    unordered_map<int, vector<int> > second_complete_solution = this->local_search.insert_for_all_route(second_solution, capacity, strategy);
    int second_solution_dist = LocalSearch::get_total_distance(second_complete_solution);

    if(first_solution_dist < second_solution_dist){
        return first_solution;
    }else{
        return second_solution;
    }
}

bool MemeticAlgorithm::is_two_solution_same(const unordered_map<int, vector<int> > &solution, const unordered_map<int, vector<int> > &solution2) {
    for(auto & r_it: solution){
        auto r_it_2 = solution2.find(r_it.first);
        if(r_it_2==solution2.end()){
            return false;
        }else{
            if(r_it.second!=r_it_2->second){
                return false;
            }
        }
    }
    return true;
}

bool MemeticAlgorithm::is_in_population(const unordered_map<int, unordered_map<int, vector<int> > > &population, const unordered_map<int, vector<int> > &solution) {
    return false;
}

int MemeticAlgorithm::get_best_solution_among_population(
        const unordered_map<int, unordered_map<int, vector<int> > > &population,
        unordered_map<int, vector<int> > &best_solution, int capacity, const string &strategy) {
    int best_dist = INT32_MAX;
    for(const auto& individual_it: population){
        unordered_map<int, vector<int> > complete_solution = this->local_search.insert_for_all_route(individual_it.second, capacity, strategy);
        int total_dist = LocalSearch::get_total_distance(complete_solution);
        if(total_dist < best_dist){
            best_dist = total_dist;
            best_solution = individual_it.second;
        }
    }
    return best_dist;
}

void MemeticAlgorithm::stochastic_ranking(const unordered_map<int, unordered_map<int, vector<int> > > &population,
                                          float time_threshold, int capacity, const string &strategy,
                                          int best_distance) {
    vector<int> population_key(population.size());
    unordered_map<int, pair<int, double> > score_map;
    const double pf = 0.45;
    int index = 0;
    for(auto & solution_it: population){
        population_key[index] = solution_it.first;
        index += 1;
        pair<int, double> fitness_result = this->fitness_function(solution_it.second, time_threshold, best_distance, capacity, strategy);
        score_map.emplace(pair<int, pair<int, double> >(solution_it.first, fitness_result));
    }

    for(int i=0; i<=this->p_size; i++){
        int swap_time = 0;
        for(int j=0; j<=this->p_size-1; j++){
            double u = this->generate_random_number();
            int start = population_key[j];
            int end = population_key[j+1];
            auto start_it = score_map.find(start);
            auto end_it = score_map.find(end);
            if((start_it->second.second==0 && end_it->second.second==0) || u < pf){
                if(start_it->second.first > end_it->second.first){
                    population_key[j] = end;
                    population_key[j+1] = start;
                    swap_time += 1;
                }
            }else{
                if(start_it->second.second > end_it->second.second){
                    population_key[j] = end;
                    population_key[j+1] = start;
                    swap_time += 1;
                }
            }
        }
        if(swap_time == 0){
            break;
        }
    }
    this->population_dict.clear();
    index = 0;
    for(int key: population_key){
        this->population_dict.emplace(pair<int, unordered_map<int, vector<int> > >(index, population.find(key)->second));
        index += 1;
    }
}

pair<int, double> MemeticAlgorithm::fitness_function(const unordered_map<int, vector<int> > &solution, float time_threshold,
                                                     int best_distance, int capacity, const string &strategy) {
    int violate_time = 0;
    int total_dist = 0;
    for(auto & it: solution){
        vector<int> complete_solution = this->local_search.insert_for_single_route(it.second, capacity, strategy);
        pair<int, float> result_pair = this->local_search.calculate_single_route_dist_and_time_cost(complete_solution, 12.5);
        total_dist += result_pair.first;
        if(result_pair.second > time_threshold){
            violate_time += 1;
        }
    }

    double lambda = (double)best_distance/this->p_size * ((double)best_distance/total_dist+(double)violate_time/this->p_size+1);

    return make_pair(total_dist,lambda*violate_time);
}

void MemeticAlgorithm::fast_non_dominated_sort(const unordered_map<int, unordered_map<int, vector<int> > > &population,
                                          float time_threshold, int capacity, const string &strategy) {
    vector<int> population_key(population.size());
    unordered_map<int, pair<int, double> > score_map;
    const double pf = 0.45;
    int index = 0;
    for(auto & solution_it: population){
        population_key[index] = solution_it.first;
        index += 1;
        pair<int, double> fitness_result = this->multiobjective_fitness_function(solution_it.second, capacity, strategy);
        score_map.emplace(pair<int, pair<int, double> >(solution_it.first, fitness_result));
    }
    unordered_map<int, int> rank;  // 存储每个个体的非支配层级
    unordered_map<int, vector<int>> dominated_set;  // 存储每个个体支配的集合
    unordered_map<int, int> domination_count;  // 存储每个个体被支配的次数
    for (const auto &solution : population) {
        int solution_id = solution.first;
        dominated_set[solution_id] = {};
        domination_count[solution_id] = 0;
    }

    // 计算支配关系
    for (const auto &solution1 : population) {
        int solution1_id = solution1.first;
        auto solu1_fitness = score_map.find(solution1_id);

        for (const auto &solution2 : population) {
            int solution2_id = solution2.first;

            if (solution1_id == solution2_id) continue;

            auto solu2_fitness = score_map.find(solution2_id);
            bool dominates = true;
            bool is_dominated = true;
            if (solu1_fitness->second.first>solu2_fitness->second.first||solu1_fitness->second.second>solu2_fitness->second.second){
                dominates = false;
            }
            if (solu1_fitness->second.first<solu2_fitness->second.first||solu1_fitness->second.second<solu2_fitness->second.second){
                is_dominated = false;
            }

            if (dominates)
                dominated_set[solution1_id].push_back(solution2_id);
            if (is_dominated)
                domination_count[solution1_id]++;
        }

        if (domination_count[solution1_id] == 0)
            rank[solution1_id] = 1;
    }
    
    vector<vector<int>> fronts = {{}};
    for (const auto &solution : population) {
        if (rank[solution.first] == 1)
            fronts[0].push_back(solution.first);
    }

    size_t i = 0;
    while (!fronts[i].empty()) {
        vector<int> next_front;
        for (int p : fronts[i]) {
            for (int q : dominated_set[p]) {
                domination_count[q]--;
                if (domination_count[q] == 0) {
                    rank[q] = i + 2;
                    next_front.push_back(q);
                }
            }
        }
        i++;
        fronts.push_back(next_front);
    }
    
    unordered_map<int, vector<int>> fronts1;
    for (const auto &solu : rank) {
        int solu_id = solu.first;
        fronts1[rank[solu_id]].push_back(solu_id);
    }
    size_t front_index = 1;
    vector<unordered_map<int, double>> crowding_distance_list;
    //逐层计算拥挤度
    while (!fronts1[front_index].empty()) {
        const vector<int> front = fronts1[front_index];
        unordered_map<int, double> crowding_distance;
        for (int s_id : front)
            crowding_distance[s_id] = 0;

        vector<int> sorted_front1 = front;
        sort(sorted_front1.begin(),sorted_front1.end(),[&](int a, int b){
            return score_map.find(a)->second.first<score_map.find(b)->second.first;
        });

        crowding_distance[sorted_front1[0]] = crowding_distance[sorted_front1.back()] = numeric_limits<double>::infinity();
        
        for (size_t i = 1; i < sorted_front1.size() - 1; i++) {
            crowding_distance[sorted_front1[i]] +=
                (double)(score_map.find(sorted_front1[i + 1])->second.first - score_map.find(sorted_front1[i - 1])->second.first) /
                (double)(score_map.find(sorted_front1.back())->second.first - score_map.find(sorted_front1[0])->second.first);
        }

        vector<int> sorted_front2 = front;
        sort(sorted_front2.begin(),sorted_front2.end(),[&](int a, int b){
            return score_map.find(a)->second.second<score_map.find(b)->second.second;
        });

        crowding_distance[sorted_front2[0]] = crowding_distance[sorted_front2.back()] = numeric_limits<double>::infinity();
        
        for (size_t i = 1; i < sorted_front2.size() - 1; i++) {
            crowding_distance[sorted_front2[i]] +=
                (score_map.find(sorted_front2[i + 1])->second.first - score_map.find(sorted_front2[i - 1])->second.first) /
                (score_map.find(sorted_front2.back())->second.first - score_map.find(sorted_front2[0])->second.first);
        }
        crowding_distance_list.push_back(crowding_distance);
        front_index += 1;
    }

    this->population_dict.clear();
    index = 0;
    //按照优先低层级，同层级优先拥挤度大进行最后排序
    for (size_t i = 1; i < front_index; i++){
        vector<pair<int, double>> sorted_front;
        for (int solu_id : fronts1[i])
            sorted_front.push_back({solu_id,crowding_distance_list[i-1][solu_id]});
        
        sort(sorted_front.begin(), sorted_front.end(),[&](const pair<int, double> &a, const pair<int, double> &b){
            return a.second>b.second;
        });
        if (index + sorted_front.size() <= this->p_size){
            for (size_t i = 0; i < sorted_front.size(); i++){
                this->population_dict.emplace(pair<int, unordered_map<int, vector<int> > >(index, population.find(sorted_front[i])->second));
                index += 1;
            }
        }else{
            for (size_t i = 0; i < this->p_size-index; i++){
                this->population_dict.emplace(pair<int, unordered_map<int, vector<int> > >(index, population.find(sorted_front[i])->second));
                index += 1;
            }
        }
    }
}

pair<int, double> MemeticAlgorithm::multiobjective_fitness_function(const unordered_map<int, vector<int> > &solution, int capacity, const string &strategy) {
    int total_dist = 0;
    vector<double> time_list;
    vector<double> capacity_list;
    for(auto & it: solution){
        vector<int> complete_solution = this->local_search.insert_for_single_route(it.second, capacity, strategy);
        pair<int, pair<float,int>> result_pair = this->local_search.calculate_single_route_dist_and_time_cost_and_capacity(complete_solution, 12.5);//distance and (time and capacity)
        total_dist += result_pair.first;
        time_list.push_back((double)result_pair.second.first);
        capacity_list.push_back((double)result_pair.second.second);
    }
    double time_fairness=this->calculateStandardDeviation(time_list);
    double capacity_fairness=this->calculateStandardDeviation(capacity_list);
    return make_pair(total_dist,time_fairness+capacity_fairness);
}

// 极差标准化
vector<double> MemeticAlgorithm::minMaxNormalization(const vector<double>& data) {
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

double MemeticAlgorithm::calculateStandardDeviation(const vector<double>& data) {
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

void MemeticAlgorithm::memetic_algorithm(int capacity, float time_threshold, int runtime, const string &strategy) {
    string file_name = R"(./)" + to_string(time(nullptr)) +".txt";

    vector<int> best_distance_vector;
    ofstream f;
    f.open(file_name);
    f.close();

    fstream f1;
    f1.open(file_name, ios::out);
    double entropy = this->calculate_diversity(this->population_dict);
    f1 << "size = 30 entropy maximum" << endl;
    f1 << "entropy = " << entropy << endl;
    this->delete_double_d_for_population();

    // fstream ff1;
    // ff1.open(file_name_dist, ios::out);

    unordered_map<int, vector<int> > best_solution;
    int best_distance = this->get_best_solution_among_population(this->population_dict, best_solution, capacity, strategy);
    f1 << "initial best_dist = " << best_distance << endl;

    // time_t start_time = time(nullptr);

    clock_t start_time, finish_time;
    start_time = clock();
    int time_index = 1;
    int time_interval = 20*60;
    while(time_index<=runtime){
        cout << "time_index" << time_index << endl;
        unordered_map<int, unordered_map<int, vector<int> > > population_map = this->population_dict;
        for(int i=0; i<this->op_size; i++){
            // cout << "i = " << i << endl;
            // cout << "population size: " << population_map.size() << endl;
            unordered_map<int, vector<int> > individual = this->crossover(capacity, strategy);
            unordered_map<int, vector<int> > individual_copy = individual;
            double r = this->generate_random_number();
            // f1 << "probability = " << r << endl;
            if(r < this->pls){
                this->local_search.optimizee_solution_ma(individual, 10, strategy);
                // cout << "WEN" << endl;
                unordered_map<int, vector<int> > complete_solution = this->local_search.insert_for_all_route(individual, capacity, strategy);
                
                int new_solution_dist = LocalSearch::get_total_distance(complete_solution);

                // cout << "Xing" << endl;
                if(new_solution_dist < best_distance){
                    best_distance = new_solution_dist;
                    best_solution = individual;
                    f1 << "best_dist = " << best_distance << endl;
                }

                population_map.emplace(pair<int, unordered_map<int, vector<int> > >(population_map.size(), individual));

                finish_time = clock();
                if(((finish_time-start_time)/CLOCKS_PER_SEC) >= time_index*time_interval){
                    // cout << (finish_time-start_time)/CLOCKS_PER_SEC << "seconds" <<endl;
                    // f1 << "after " << time_index << " * "<< time_interval/60 <<" minutes, best_distance = " << best_distance << endl;
                    // best_distance_vector.emplace_back(best_distance);
                    double avg_dist = 0;
                    double avg_fair = 0;
                    for(auto & solution_it: this->population_dict){
                        pair<int, double> fitness_result = this->multiobjective_fitness_function(solution_it.second, capacity, strategy);
                        avg_dist += (double)fitness_result.first;
                        avg_fair += fitness_result.second;
                    }
                    avg_dist/=this->population_dict.size();
                    avg_fair/=this->population_dict.size();
                    f1 << "after " << time_index << " * "<< time_interval/60 <<" minutes, the average distance and fairness of the population :  " << avg_dist << ", " << avg_fair << endl;
                    time_index += 1;
                }
                if(time_index > runtime){
                    break;
                }

                if(!is_two_solution_same(individual, individual_copy)){
                    population_map.emplace(pair<int, unordered_map<int, vector<int> > >(population_map.size(), individual_copy));

                }
            }else{
                //this->local_search.optimizee_solution_ma(individual, 5, strategy);
                unordered_map<int, vector<int> > complete_solution = this->local_search.insert_for_all_route(individual, capacity, strategy);
                int new_solution_dist = LocalSearch::get_total_distance(complete_solution);

                if(new_solution_dist < best_distance){
                    best_distance = new_solution_dist;
                    best_solution = individual;
                    f1 << "best_dist = " << best_distance << endl;
                }

                population_map.emplace(pair<int, unordered_map<int, vector<int> > >(population_map.size(), individual));

                finish_time = clock();
                if(((finish_time-start_time)/CLOCKS_PER_SEC) >= time_index*time_interval){
                    // f1 << "after " << time_index << " * "<<time_interval/60 <<" minutes, best_distance = " << best_distance << endl;
                    // time_index += 1;
                    // best_distance_vector.emplace_back(best_distance);
                    double avg_dist = 0;
                    double avg_fair = 0;
                    for(auto & solution_it: this->population_dict){
                        pair<int, double> fitness_result = this->multiobjective_fitness_function(solution_it.second, capacity, strategy);
                        avg_dist += (double)fitness_result.first;
                        avg_fair += fitness_result.second;
                    }
                    avg_dist/=this->population_dict.size();
                    avg_fair/=this->population_dict.size();
                    f1 << "after " << time_index << " * "<< time_interval/60 <<" minutes, the average distance and fairness of the population :  " << avg_dist << ", " << avg_fair << endl;
                    time_index += 1;
                }
                if(time_index > runtime){
                    break;
                }
            }
        }

        // this->stochastic_ranking(population_map, time_threshold, capacity, strategy, best_distance);
        this->fast_non_dominated_sort(population_map, time_threshold, capacity, strategy);
    }
    f1 << "End while, the final population as below:" << endl;
    double avg_dist = 0;
    double avg_fair = 0;
    for(int i = 0; i < this->population_dict.size(); i++){
        pair<int, double> fitness_result = this->multiobjective_fitness_function(population_dict[i].second, capacity, strategy);
        f1 << i << "-th solution, dist " << fitness_result.first << ", fairness " << fitness_result.second << endl;
        avg_dist += (double)fitness_result.first;
        avg_fair += fitness_result.second;
    }
    avg_dist/=this->population_dict.size();
    avg_fair/=this->population_dict.size();
    f1 << "Finally , the average distance and fairness of the population :  " << avg_dist << ", " << avg_fair << endl;
                    
    LocalSearch::show_solution(best_solution, f1);
    local_search.valid_solution(best_solution, 8*3600, f1);
    MemeticAlgorithm::show_vector(best_distance_vector, f1);
    f1.close();
}

void MemeticAlgorithm::show_vector(const vector<int> &goal_vector) {
    for(int site: goal_vector){
        cout << site << " ";
    }
    cout << endl;
}

void MemeticAlgorithm::show_vector(const vector<int> & goal_vector, fstream & f1){
    for(int site: goal_vector){
        f1 << site << endl;
    }
}

pair<int, bool> MemeticAlgorithm::is_in_trip_list(const vector<struct Trip> &trip_list, const unordered_set<int> &trip_set) {
    for(int i=0; i<trip_list.size(); i++){
        if(trip_list[i].trip == trip_set){
            return make_pair(i, true);
        }
    }
    return make_pair(-1, false);
}

double MemeticAlgorithm::calculate_diversity(const unordered_map<int, unordered_map<int, vector<int> > > &population) {
    // The solution in population need to have depot and disposal facility.
    vector<struct Trip> trip_list;
    int trip_number = 0;
    int population_size = population.size();
    for(auto & solution_it: population){
        for(auto & route_it: solution_it.second){
            unordered_set<int> trip_set;
            for(int i=1; i<route_it.second.size()-1; i++){
                auto result_it = this->local_search.m_dataLoader.p_d_set.find(route_it.second[i]);
                if(result_it!=this->local_search.m_dataLoader.p_d_set.end()){
                    pair<int, bool> result_pair = this->is_in_trip_list(trip_list, trip_set);
                    if(result_pair.second){
                        trip_list[result_pair.first].number += 1;
                    }else{
                        struct Trip new_trip = {1, trip_set};
                        trip_list.emplace_back(new_trip);
                    }
                    trip_set.clear();
                    trip_number += 1;
                }else{
                    trip_set.insert(route_it.second[i]);
                }
            }
        }
    }
    double entropy = 0;
    for(auto & trip_it: trip_list){
        entropy += (-(double)trip_it.number/population_size * log2(trip_it.number*1.0/population_size));
    }
    return entropy;
}


