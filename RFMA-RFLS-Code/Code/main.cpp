#include <iostream>
#include "object/json/json.h"
#include <fstream>
#include <string>
#include <unordered_map>
#include <algorithm>
#include <ctime>
#include "InitialSolution.h"
#include "LocalSearch.h"
#include "MemeticAlgorithm.h"
using namespace std;

struct pair_hash{
    template<class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &p) const{
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1^h2;
    }
};
unordered_map<int, vector<int> > get_solution(const DataLoader & dataLoader);
void get_population_parameter(vector<pair<int, int> > &parameter_vector, int iteration);
void show_vector(vector<pair<int, int> > & parameter_vector, fstream & f1);
void initial_solution_fine_tuning(DataLoader & dataLoader);
void population_fine_tuning(DataLoader & dataLoader, int iteration, int p_size, const unordered_map<pair<int, int>,
        unordered_map<int, vector<int> >, pair_hash > & solution_map, fstream & f1);
void get_all_solution(DataLoader & dataLoader, unordered_map<pair<int, int>, unordered_map<int, vector<int> >,
        pair_hash > & solution_map);
void get_population_according_parameter(DataLoader & dataLoader, unordered_map<int,
        unordered_map<int, vector<int> > > & population, const vector<pair<int, int> > & parameter_vector);
void delete_double_d_for_population(unordered_map<int, unordered_map<int, vector<int> > > & population,
                                    LocalSearch & localSearch);
void vehicle_number_fine_tuning(int start, int end);
void get_all_diversity_capacity(DataLoader & dataLoader, unordered_map<pair<int, int>,
        unordered_map<int, vector<int> >, pair_hash > & solution_map, int c_rate);
void get_all_diversity_time(DataLoader & dataLoader, unordered_map<pair<int, int>,
        unordered_map<int, vector<int> >, pair_hash > & solution_map, int t_rate);


int main() {
    const string v_file_path = R"(./data/VehicleData_100.json)";
    const string c_file_path = R"(./data/WasteCollectionSites.json)";
    const string p_file_path = R"(./data/Depots.json)";
    const string d_file_path = R"(./data/DisposalFacilities.json)";
    // 1st_rebuttal_dataset
    const string dist_file_path = R"(./data/FinalResult_V2.csv)";
    // const string dist_file_path = R"(/home/yezy/cplusplus_code/UWCP/data/1_1_rebuttal_dataset/70sites_distance.csv)";
    DataLoader dataLoader = DataLoader(
            v_file_path, c_file_path, p_file_path, d_file_path, dist_file_path
            );

    /*------Optimize the solution with region-focused local search only.------- */
    // InitialSolution initialSolution = InitialSolution(dataLoader);
    // unordered_map<int, vector<int> > solution = initialSolution.init_solution();
    // solution = ma_solution_map.find(1)->second;

//     LocalSearch localSearch = LocalSearch(dataLoader);

    // int total_dist = localSearch.get_total_distance(solution);
    // cout << "original total_dist = " << total_dist << endl;
    // solution = localSearch.delete_double_d(solution);

    // localSearch.optimize_solution(solution, 40, "Greedy");
    // unordered_map<int, vector<int> > complete_solution = localSearch.insert_for_all_route(solution, 72000, "Backspacing");
    // total_dist = localSearch.get_total_distance(complete_solution);
    // cout << "after insert total_dist = " << total_dist << endl;
    /*-----------------------------------------------------------------*/

    /*------Optimize the solution with region-focused memetic algorithm.------- */
    unordered_map<int, unordered_map<int, vector<int> > > population;
     // High Diversity
    vector<pair<int, int> > parameter_vector = {{89, 94}, {98, 95}, {99, 98}, {92, 92}, {95, 98}, {95, 97}, {100, 93}, {99, 90}, {92, 91}, {97, 94}, {98, 98}, {100, 94}, {92, 98}, {98, 91}, {100, 90}, {93, 95}, {92, 94}, {97, 92}, {96, 92}, {97, 90}, {80, 94}, {86, 96}, {89, 91}, {94, 94}, {95, 95}, {97, 93}, {96, 95}, {97, 91}, {97, 97}, {84, 99}};
     // Low Diversity
     // vector<pair<int, int> > parameter_vector = {{86, 94}, {81, 97}, {95, 90}, {87, 95}, {87, 94}, {85, 92}, {82, 98}, {85, 94}, {87, 90}, {98, 98}, {92, 93}, {89, 92}, {85, 100}, {81, 93}, {82, 93}, {94, 94}, {88, 100}, {90, 91}, {89, 97}, {100, 98}, {80, 100}, {87, 93}, {92, 95}, {83, 90}, {87, 96}, {94, 92}, {86, 97}, {93, 96}, {89, 93}, {84, 99}};
    get_population_according_parameter(dataLoader, population, parameter_vector);
    cout << population.size() << endl;

    LocalSearch localSearch = LocalSearch(dataLoader);
    MemeticAlgorithm memeticAlgorithm = MemeticAlgorithm(localSearch, population.size(), (int)population.size()*2, 0.2, population);
    memeticAlgorithm.calculate_diversity(population);

    memeticAlgorithm.delete_double_d_for_population();
    unordered_map<int, vector<int> > best_solution;
    string strategy = "Greedy";
    int best_dist = memeticAlgorithm.get_best_solution_among_population(memeticAlgorithm.population_dict, best_solution, 72000, strategy);
    cout << best_dist << endl;

    memeticAlgorithm.delete_double_d_for_population();
    memeticAlgorithm.memetic_algorithm(72000, 8*3600, 30, "Greedy");

    return 0;
}

void vehicle_number_fine_tuning(int start, int end){
    string v_file = R"(/home/yezy/cplusplus_code/UWCP/data/VehicleData_)";
    fstream f1;
    f1.open(R"(/home/yezy/cplusplus_code/UWCP/vehicle_number_detail_small.csv)", ios::out);
    f1 << "vehicle_number,distance,no_server_number,using_vehicle,time_cost,number_time_constraints,3000,3001,3002" << endl;

    for(int index=start; index<=end; index++){
        cout << "index = " << index << endl;
        const string v_file_path = v_file+to_string(index) + ".json";
        const string c_file_path = R"(/home/yezy/cplusplus_code/UWCP/data/ExtractPoints.json)";
        const string p_file_path = R"(/home/yezy/cplusplus_code/UWCP/data/ParkingLots.json)";
        const string d_file_path = R"(/home/yezy/cplusplus_code/UWCP/data/Dropping.json)";
        const string dist_file_path = R"(/home/yezy/cplusplus_code/UWCP/data/FinalResult.csv)";
        DataLoader dataLoader = DataLoader(
                v_file_path, c_file_path, p_file_path, d_file_path, dist_file_path
        );
        InitialSolution initialSolution = InitialSolution(dataLoader);
        time_t start_time = time(nullptr);
        
        unordered_map<int, vector<int> > solution = initialSolution.init_solution();

        int time_cost = time(nullptr) - start_time;
        cout << solution.size() << endl;
        LocalSearch localSearch = LocalSearch(dataLoader);
        int total_dist = LocalSearch::get_total_distance(solution);
        int num_tc = localSearch.valid_solution(solution, 8*3600);

        unordered_map<int, int> depot_number = {{3000, 0}, {3001, 0}, {3002, 0}};

        for(auto r_it: solution){
            depot_number.find(r_it.second.front())->second += 1;
        }

        f1 << index << "," << total_dist << "," << initialSolution.m_dataLoader.no_served_c.size() << "," << solution.size() << "," << time_cost << "," << num_tc << "," << depot_number.find(3000)->second << "," << depot_number.find(3001)->second << "," << depot_number.find(3002)->second << endl;
    }
    f1.close();
}

void delete_double_d_for_population(unordered_map<int, unordered_map<int, vector<int> > > & population, LocalSearch & localSearch){
    for(auto & it: population){
        it.second = localSearch.delete_double_d(it.second);
    }
}

void get_population_according_parameter(DataLoader & dataLoader, unordered_map<int, unordered_map<int, vector<int> > > & population, const vector<pair<int, int> > & parameter_vector){
    int index = 0;
    InitialSolution initialSolution = InitialSolution(dataLoader);
    //LocalSearch localSearch = LocalSearch(dataLoader);

    for(auto & it: parameter_vector){
        float time_r = it.first/100.0;
        float capacity_r = it.second/100.0;

        initialSolution.setCapacityRate(capacity_r);
        initialSolution.setTimeRate(time_r);
        unordered_map<int, vector<int> > solution = initialSolution.init_solution();
        initialSolution.check_solution(solution);
        //solution = localSearch.delete_double_d(solution);
        population.emplace(pair<int, unordered_map<int, vector<int> > >(index, solution));
        index += 1;
        for(auto & v_it: initialSolution.m_dataLoader.vehicle_data){
            v_it.second.set_current_time_cost_and_current_capacity(0,0);
        }
        //cout << "no_servered_c size = " << initialSolution.m_dataLoader.no_served_c.size() << endl;
        for(auto & c_it: initialSolution.m_dataLoader.collection_site_data){
            initialSolution.m_dataLoader.no_served_c.insert(c_it.first);
        }
        initialSolution.m_dataLoader.depot_vehicle_dict = initialSolution.m_dataLoader.depot_vehicle_dict_copy;
    }
}


void show_vector(vector<pair<int, int> > & parameter_vector, fstream & f1){
    cout << "{ ";
    for(auto & it: parameter_vector){
        if(it!=parameter_vector.back()){
            f1 << "{" << it.first << ", " << it.second << "}, ";
        }else{
            f1 << "{" << it.first << ", " << it.second << "}";
        }

    }
    f1 << "}" << endl;
}

void get_population_parameter(vector<pair<int, int> > &parameter_vector, int iteration) {
    // parameter_vector <time_rate, capacity_rate>
    vector<int> tr_vector(21);
    vector<int> cr_vector(11);

    int ttrr = 84;
    int ccrr = 99;

    int time_rate = 80;
    for(int i=0; i<21; i++){
        tr_vector[i] = time_rate+i;
    }

    int capacity_rate = 90;
    for(int i=0; i<11; i++){
        cr_vector[i] = capacity_rate+i;
    }

    while(true){
        // cout << "here" << endl;
        int tr_index = LocalSearch::get_random_number(0, 20);
        int cr_index = LocalSearch::get_random_number(0, 10);

        time_rate = tr_vector[tr_index];
        capacity_rate = cr_vector[cr_index];
        // cout << time_rate << " " << capacity_rate << endl;
        if(time_rate!=ttrr && capacity_rate!=ccrr){
            pair<int, int> tr_cr_pair = make_pair(time_rate, capacity_rate);
            bool symbol = false;
            for(auto & it: parameter_vector){
                if(it.first==time_rate && it.second==capacity_rate){
                    symbol = true;
                    break;
                }else{
                    continue;
                }
            }

            if(!symbol){
                parameter_vector.emplace_back(tr_cr_pair);
            }
            if(parameter_vector.size()==iteration-1){
                break;
            }
        }
        
    }
    pair<int, int> pp = make_pair(ttrr, ccrr);
    parameter_vector.emplace_back(pp);
}

void get_all_diversity_capacity(DataLoader & dataLoader, unordered_map<pair<int, int>, unordered_map<int, vector<int> >, pair_hash > & solution_map, int c_rate){
    int time_rate = 80;
    int capacity_rate = c_rate;
    //unordered_map<pair<float, float>, unordered_map<int, vector<int> >, pair_hash > solution_map;
    //vector<unordered_map<int, vector<int> > > solution_vector(21*11);
    int index = 0;
    InitialSolution initialSolution = InitialSolution(dataLoader);
    for(int i=0; i<21; i++){
        int time_r = time_rate + i;
        // cout << index << endl;
        int capacity_r = capacity_rate + 0;
        initialSolution.setCapacityRate(capacity_r/100.0);
        initialSolution.setTimeRate(time_r/100.0);
        // cout << time_r << " " << capacity_r << endl;
        unordered_map<int, vector<int> > solution = initialSolution.init_solution();
        pair<int, int> key_pair = make_pair(time_r, capacity_r);
        solution_map.emplace(pair<pair<int, int>, unordered_map<int, vector<int> > >(key_pair, solution));

        //solution_vector[index] = solution;
        index += 1;

        for(auto & v_it: initialSolution.m_dataLoader.vehicle_data){
            v_it.second.set_current_time_cost_and_current_capacity(0,0);
        }
        //cout << "no_servered_c size = " << initialSolution.m_dataLoader.no_served_c.size() << endl;
        for(auto & c_it: initialSolution.m_dataLoader.collection_site_data){
            initialSolution.m_dataLoader.no_served_c.insert(c_it.first);
        }
        initialSolution.m_dataLoader.depot_vehicle_dict = initialSolution.m_dataLoader.depot_vehicle_dict_copy;
    
    }
}
void get_all_diversity_time(DataLoader & dataLoader, unordered_map<pair<int, int>, unordered_map<int, vector<int> >, pair_hash > & solution_map, int t_rate){
    int time_rate = t_rate;
    int capacity_rate = 90;
    //unordered_map<pair<float, float>, unordered_map<int, vector<int> >, pair_hash > solution_map;
    //vector<unordered_map<int, vector<int> > > solution_vector(21*11);
    int index = 0;
    InitialSolution initialSolution = InitialSolution(dataLoader);
    for(int i=0; i<11; i++){
        int time_r = time_rate + 0;
        // cout << index << endl;
        int capacity_r = capacity_rate + i;
        initialSolution.setCapacityRate(capacity_r/100.0);
        initialSolution.setTimeRate(time_r/100.0);
        // cout << time_r << " " << capacity_r << endl;
        unordered_map<int, vector<int> > solution = initialSolution.init_solution();
        pair<int, int> key_pair = make_pair(time_r, capacity_r);
        solution_map.emplace(pair<pair<int, int>, unordered_map<int, vector<int> > >(key_pair, solution));

        //solution_vector[index] = solution;
        index += 1;

        for(auto & v_it: initialSolution.m_dataLoader.vehicle_data){
            v_it.second.set_current_time_cost_and_current_capacity(0,0);
        }
        //cout << "no_servered_c size = " << initialSolution.m_dataLoader.no_served_c.size() << endl;
        for(auto & c_it: initialSolution.m_dataLoader.collection_site_data){
            initialSolution.m_dataLoader.no_served_c.insert(c_it.first);
        }
        initialSolution.m_dataLoader.depot_vehicle_dict = initialSolution.m_dataLoader.depot_vehicle_dict_copy;
    
    }
}

void get_all_solution(DataLoader & dataLoader, unordered_map<pair<int, int>, unordered_map<int, vector<int> >, pair_hash > & solution_map){
    int time_rate = 80;
    int capacity_rate = 90;
    //unordered_map<pair<float, float>, unordered_map<int, vector<int> >, pair_hash > solution_map;
    //vector<unordered_map<int, vector<int> > > solution_vector(21*11);
    int index = 0;
    InitialSolution initialSolution = InitialSolution(dataLoader);
    for(int i=0; i<21; i++){
        int time_r = time_rate + i;
        for(int j=0; j<11; j++){
            // cout << index << endl;
            int capacity_r = capacity_rate +  j;
            initialSolution.setCapacityRate(capacity_r/100.0);
            initialSolution.setTimeRate(time_r/100.0);
            // cout << time_r << " " << capacity_r << endl;
            unordered_map<int, vector<int> > solution = initialSolution.init_solution();
            pair<int, int> key_pair = make_pair(time_r, capacity_r);
            solution_map.emplace(pair<pair<int, int>, unordered_map<int, vector<int> > >(key_pair, solution));

            //solution_vector[index] = solution;
            index += 1;

            for(auto & v_it: initialSolution.m_dataLoader.vehicle_data){
                v_it.second.set_current_time_cost_and_current_capacity(0,0);
            }
            //cout << "no_servered_c size = " << initialSolution.m_dataLoader.no_served_c.size() << endl;
            for(auto & c_it: initialSolution.m_dataLoader.collection_site_data){
                initialSolution.m_dataLoader.no_served_c.insert(c_it.first);
            }
            initialSolution.m_dataLoader.depot_vehicle_dict = initialSolution.m_dataLoader.depot_vehicle_dict_copy;
        }
    }
}

void initial_solution_fine_tuning(DataLoader & dataLoader){
    float tr = 0.8;
    float cr = 0.9;
    fstream f1;
    f1.open(R"(E:\ComputerFile\Summer_Work\C++_Code\UWCP\parameter_dist.csv)", ios::out);
    f1 << "time_rate" << "," << "capacity_rate" << "," << "distance" << "," << "number of vehicle"<< "," << "number_of_violate_constraints" << "," << "total_time_violate(h)" << "," << "time avearge" << "," << "time sdv" << endl;
    while(tr<=1){
        cr = 0.9;
        while(cr<=1){
            InitialSolution initialSolution = InitialSolution(dataLoader);
            initialSolution.setTimeRate(tr);
            initialSolution.setCapacityRate(cr);
            unordered_map<int, vector<int> > solution = initialSolution.init_solution();
            cout << solution.size() << endl;
            LocalSearch localSearch = LocalSearch(dataLoader);
            //int total_dist = localSearch.get_total_distance(solution);
            int total_dist = 0;
            int number_constraint = 0;
            vector<float> time_vector;
            float total_time_violate = 0;
            float time_average = 0;
            for(auto & route_it: solution){
                pair<int, float> result_pair = localSearch.calculate_single_route_dist_and_time_cost(route_it.second, 12.5);
                total_dist += result_pair.first;
                float total_time = result_pair.second/3600;
                if(total_time > 8){
                    number_constraint += 1;
                    total_time_violate += (total_time-8);
                }
                time_average += total_time;
                time_vector.push_back(total_time);
            }
            float time_sd = 0;
            time_average /= solution.size();
            for(auto t: time_vector){
                time_sd += (t-time_average)*(t-time_average);
            }
            time_sd /= solution.size();
            time_sd = (float)sqrt(time_sd);
            f1 << initialSolution.getTimeRate() << "," << initialSolution.getCapacityRate()<< ","  << total_dist << "," <<solution.size() << "," << number_constraint << "," << total_time_violate << "," << time_average << "," << time_sd << endl;
            for(auto & v_it: initialSolution.m_dataLoader.vehicle_data){
                v_it.second.set_current_time_cost_and_current_capacity(0,0);
            }
            for(auto & s_it: initialSolution.m_dataLoader.collection_site_data){
                initialSolution.m_dataLoader.no_served_c.insert(s_it.first);

            }
            initialSolution.m_dataLoader.depot_vehicle_dict = initialSolution.m_dataLoader.depot_vehicle_dict_copy;
            cr += 0.01;

        }
        tr += 0.01;
    }
    f1.close();
}

void population_fine_tuning(DataLoader & dataLoader, int iteration, int p_size, const unordered_map<pair<int, int>, unordered_map<int, vector<int> >, pair_hash > & solution_map, fstream & f1){
    InitialSolution initialSolution = InitialSolution(dataLoader);
    LocalSearch localSearch = LocalSearch(dataLoader);
    double minimum_entropy = 100000000000000000000.0;
    double maximum_entropy = -100000000000000000000.0;
    for(int j=0; j<iteration; j++){
        unordered_map<int, unordered_map<int, vector<int> > > population;
        vector<pair<int, int> > parameter_vector;
        get_population_parameter(parameter_vector, p_size);
        for(int i=0; i<parameter_vector.size(); i++){
            cout << i  << endl;
            // pair<float, float> key_pair = make_pair(parameter_vector[i].first*100, parameter_vector[i].second*100);
            auto it = solution_map.find(parameter_vector[i]);
            // unordered_map<int, vector<int>> solution = initialSolution.init_solution();
            unordered_map<int, vector<int>> solution = it->second;
            initialSolution.check_solution(solution);
            population.emplace(pair<int, unordered_map<int, vector<int> > >(i, solution));
            // pair<float, float> tr_cr_pair = parameter_vector[i];
            // initialSolution.setCapacityRate(tr_cr_pair.second);
            // initialSolution.setTimeRate(tr_cr_pair.first);
            // for(auto & v_it: initialSolution.m_dataLoader.vehicle_data){
            //     v_it.second.set_current_time_cost_and_current_capacity(0,0);
            // }
            // //cout << "no_servered_c size = " << initialSolution.m_dataLoader.no_served_c.size() << endl;
            // for(auto & c_it: initialSolution.m_dataLoader.collection_site_data){
            //     initialSolution.m_dataLoader.no_served_c.insert(c_it.first);
            // }
            // initialSolution.m_dataLoader.depot_vehicle_dict = initialSolution.m_dataLoader.depot_vehicle_dict_copy;
            //cout << "no_servered_c size = " << initialSolution.m_dataLoader.no_served_c.size() << endl;
            // solution = initialSolution.init_solution();
        }

        MemeticAlgorithm memeticAlgorithm = MemeticAlgorithm(localSearch, 15, 15, 0.2, population);
        double entropy = memeticAlgorithm.calculate_diversity(population);
        if(entropy > maximum_entropy){
            maximum_entropy = entropy;
        }
        if(entropy < minimum_entropy){
            minimum_entropy = entropy;
        }
        f1 << "entropy = " << entropy << endl;
        show_vector(parameter_vector, f1);
        parameter_vector.clear();
        
    }
    f1 << "maximum_entropy = " << maximum_entropy << endl;
    f1 << "minimum_entropy = " << minimum_entropy << endl;
}


