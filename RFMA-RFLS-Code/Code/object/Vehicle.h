//
// Created by Joe Lan on 2020/7/2.
//

#ifndef UWCP_VEHICLE_H
#define UWCP_VEHICLE_H

#include <string>
using namespace std;

class Vehicle {
private:
    string vehicle_id; // The id of vehicle.
    int vehicle_index; // The index of vehicle.
    string parking_id; // The id of parking which the vehicle owns.
    int parking_index; // The index of the parking.
    int max_capacity; // The maximum capacity of the vehicle.
    float speed; // The average speed of the vehicle.
    float total_time_cost;
    float current_time_cost;
    int current_capacity;

public:
    Vehicle(const string & vehicleId, const string & parkingId, int vehicleIndex, int parkingIndex, int maxCapacity,
            float mSpeed, float totalTimeCost, float currentTimeCost, int currentCapacity);

    int getVehicleIndex() const;

    void setVehicleIndex(int vehicleIndex);

    const string &getVehicleId() const;

    void setVehicleId(const string &vehicleId);

    const string &getParkingId() const;

    void setParkingId(const string &parkingId);

    int getParkingIndex() const;

    void setParkingIndex(int parkingIndex);

    int getMaxCapacity() const;

    void setMaxCapacity(int maxCapacity);

    float getSpeed() const;

    void setSpeed(float mSpeed);

    float getTotalTimeCost() const;

    void setTotalTimeCost(float totalTimeCost);

    float getCurrentTimeCost() const;

    void setCurrentTimeCost(float currentTimeCost);

    int getCurrentCapacity() const;

    void setCurrentCapacity(int currentCapacity);

    void set_current_time_cost_and_current_capacity(float currentTimeCost, int currentCapacity);
};


#endif //UWCP_VEHICLE_H
