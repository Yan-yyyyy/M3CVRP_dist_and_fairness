//
// Created by Joe Lan on 2020/7/16.
//

#ifndef UWCP_MEMETICALGORITHM_H
#define UWCP_MEMETICALGORITHM_H
#include "LocalSearch.h"
#include <random>
#include <algorithm>
#include <cmath>
#include <numeric>

class MemeticAlgorithm {
private:
    default_random_engine generator;
    struct Trip{
        int number;
        unordered_set<int> trip;
    };
public:
    unordered_map<int, unordered_map<int, vector<int> > > population_dict;
    int p_size;
    int op_size;
    float pls;
    LocalSearch local_search;

    static void sequence_base_crossover(vector<int> & first_route, vector<int> & second_route);
    static vector<int> get_key_vector(unordered_map<int, unordered_map<int, vector<int> > > & population_map);
    MemeticAlgorithm(LocalSearch & localSearch, int pSize, int opSize, float mPLS, unordered_map<int, unordered_map<int, vector<int> > > & populationDict);
    void
    delete_repeat_and_insert_missing(unordered_map<int, vector<int> > &solution, int capacity, const string &strategy,
                                     int route_id, const vector<int> &original_route);
    void delete_repeat_and_insert_missing_v2(unordered_map<int, vector<int> > &solution, int capacity, int car_number, const string & strategy, int route_id, const vector<int> & original_route);
    unordered_map<int, vector<int>> crossover(int capacity, const string &strategy);
    void memetic_algorithm(int capacity, float time_threshold, int runtime, const string &strategy);
    double generate_random_number();
    static bool is_in_population(const unordered_map<int, unordered_map<int, vector<int> > > & population, const unordered_map<int, vector<int> > & solution);
    static bool is_two_solution_same(const unordered_map<int, vector<int> > & solution, const unordered_map<int, vector<int> > & solution2);
    void stochastic_ranking(const unordered_map<int, unordered_map<int, vector<int> > > &population,
                            float time_threshold, int capacity, const string &strategy,
                            int best_distance);
    int get_best_solution_among_population(
            const unordered_map<int, unordered_map<int, vector<int> > > &population,
            unordered_map<int, vector<int> > &best_solution, int capacity, const string &strategy);
    pair<int, double> fitness_function(const unordered_map<int, vector<int> > &solution, float time_threshold,
                                       int best_distance, int capacity, const string &strategy);
    void fast_non_dominated_sort(const unordered_map<int, unordered_map<int, vector<int> > > &population,
                            float time_threshold, int capacity, const string &strategy);
    pair<int, double> multiobjective_fitness_function(const unordered_map<int, vector<int> > &solution, int capacity, const string &strategy);
    pair<int, pair<double,double>> multiobjective_fitness_function_depart(const unordered_map<int, vector<int> > &solution, int capacity, const string &strategy);
    vector<double> minMaxNormalization(const vector<double>& data);
    double calculateStandardDeviation(const vector<double>& data);
    double calculate_diversity(const unordered_map<int, unordered_map<int, vector<int> > > & population);
    static void show_vector(const vector<int> & goal_vector);
    static void show_vector(const vector<int> & goal_vector, fstream & f1);
    static pair<int, bool> is_in_trip_list(const vector<struct Trip> & trip_list, const unordered_set<int> & trip_set);
    void delete_double_d_for_population();
};


#endif //UWCP_MEMETICALGORITHM_H
