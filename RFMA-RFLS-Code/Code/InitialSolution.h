//
// Created by Joe Lan on 2020/7/6.
//

#ifndef UWCP_INITIALSOLUTION_H
#define UWCP_INITIALSOLUTION_H
#include "object/DataLoader.h"
#include <algorithm>
#include <stdlib.h>

class InitialSolution {
private:
    // float t_rate = 0.86;
    // float c_rate = 0.98;
    float t_rate = 1;
    float c_rate = 1;
    // float t_rate = 0.84;
    // float c_rate = 0.99;
public:
    float getTimeRate() const;

    void setTimeRate(float timeRate);

    float getCapacityRate() const;

    void setCapacityRate(float capacityRate);

public:
    DataLoader m_dataLoader;
    unordered_set<int> served_c;

    InitialSolution(DataLoader & dataLoader);
    pair<int, int> drop_selection(int cur_p_index);
    pair<int, int> get_drop_according_to_depot(int site_index, int depot);
    pair<int, int> collection_selection(int cur_p_index, int vehicle_index);
    pair<int, int> arrange_vehicle();
    unordered_map<int, vector<int> > init_solution();
    void check_solution(const unordered_map<int, vector<int> > & solution);
};


#endif //UWCP_INITIALSOLUTION_H
