//
// Created by Joe Lan on 2020/7/6.
//

#ifndef UWCP_LOCALSEARCH_H
#define UWCP_LOCALSEARCH_H

#include "object/DataLoader.h"
#include <random>
#include <ctime>
#include <fstream>
#include <algorithm>


class LocalSearch {
public:
    DataLoader m_dataLoader;
    int window_len = 5;
    int seed_index = 0;

    LocalSearch(DataLoader & dataLoader);
    static int get_random_number(unsigned int begin, unsigned int end);
    static int calculate_single_route_distance(const vector<int> & complete_route);
    pair<int, float> calculate_single_route_dist_and_time_cost(const vector<int> & complete_route, float speed);
    int get_closest_disposal_facility(int start, int end);
    pair<int, int> get_closest_park(int start, int end);
    vector<int> insert_disposal_facility_with_greedy(const vector<int> & route, int car_capacity);
    void inner_insert(const vector<int> & original_route, int capacity, int win_len, int begin_index, int end_index, vector<int> new_route, vector<int> & best_route, int & minimum_dist);
    vector<int> insert_disposal_facility_with_backspacing(const vector<int> & route, int car_capacity, int window_length);
    vector<int> insert_for_single_route(const vector<int> & single_route, int car_capacity, const string & strategy);
    unordered_map<int, vector<int> > insert_for_all_route(const unordered_map<int, vector<int> > & solution, int car_capacity, const string & strategy);
    static int get_total_distance(unordered_map<int, vector<int> > & complete_solution);
    unordered_map<int, vector<int> > delete_double_d(unordered_map<int, vector<int> > & solution);
    vector<int> insert_operator(vector<int> & route, int car_capacity, const string & strategy);
    vector<int> exchange_operator(vector<int> & route, int car_capacity, int interval, const string & strategy);
    static void swap(int & start, int & end);
    vector<int> reverse_operator(vector<int> & route, int car_capacity, int interval, const string & strategy);
    void mutation_with_classic_operator(unordered_map<int, vector<int> > & solution, int iteration, int car_capacity, const string & strategy, int operator_name);
    static unordered_map<int, int> get_site_route_dict(const unordered_map<int, vector<int> > & solution);
    static vector<int> get_key_vector(const unordered_map<int, vector<int> > & solution);
    void region_constrained_single_point_swap(unordered_map<int, vector<int> > & solution, int iteration, int car_capacity, const string & strategy);
    static int find_index(const vector<int> &goal_vector, int element);
    static void swap_segment(vector<int> & original_r, vector<int> &another_r, int original_index, int another_index);
    void region_constrained_segment_swap(unordered_map<int, vector<int> > & solution, int iteration, int capacity, float time_threshold, float speed, const string & strategy);
    void mutation_with_novel_operator(unordered_map<int, vector<int> > & solution, int outer_iteration, int inner_iteration, int capacity, int operator_name, const string & strategy);
    void relaxed_multi_point_swap(unordered_map<int, vector<int> > & solution, int dist_threshold, int capacity, float percentage, float speed, float time_threshold, const string & strategy);
    void mutation_with_relaxed_multi_point_swap(unordered_map<int, vector<int> > & solution, int iteration, int capacity, int dist_threshold, float percentage, float speed, float time_threshold, const string & strategy);
    void optimize_solution(unordered_map<int, vector<int> > & solution, int runtime, const string & strategy);
    static void show_solution(unordered_map<int, vector<int> > & solution);
    static void show_solution(unordered_map<int, vector<int> > & solution, fstream & fstream1);
    void check_solution(const unordered_map<int, vector<int> > & solution);
    void optimizee_solution_ma(unordered_map<int, vector<int> > & solution, int iteration, const string & strategy);
    void valid_solution(const unordered_map<int, vector<int> > & solution, float time_constraints, fstream & f1);
    int valid_solution(const unordered_map<int, vector<int> > & solution, float time_constraints);
};


#endif //UWCP_LOCALSEARCH_H
