//
// Created by Joe Lan on 2020/7/2.
//

#include "Vehicle.h"

Vehicle::Vehicle(const string & vehicleId, const string & parkingId, int vehicleIndex, int parkingIndex,
                 int maxCapacity, float mSpeed, float totalTimeCost, float currentTimeCost, int currentCapacity) {
    vehicle_id = vehicleId;
    vehicle_index = vehicleIndex;
    parking_id = parkingId;
    parking_index = parkingIndex;
    max_capacity = maxCapacity;
    speed = mSpeed;
    total_time_cost = totalTimeCost;
    current_time_cost = currentTimeCost;
    current_capacity = currentCapacity;
}

const string &Vehicle::getVehicleId() const {
    return vehicle_id;
}

void Vehicle::setVehicleId(const string &vehicleId) {
    vehicle_id = vehicleId;
}

const string &Vehicle::getParkingId() const {
    return parking_id;
}

void Vehicle::setParkingId(const string &parkingId) {
    parking_id = parkingId;
}

int Vehicle::getParkingIndex() const {
    return parking_index;
}

void Vehicle::setParkingIndex(int parkingIndex) {
    parking_index = parkingIndex;
}

int Vehicle::getMaxCapacity() const {
    return max_capacity;
}

void Vehicle::setMaxCapacity(int maxCapacity) {
    max_capacity = maxCapacity;
}

float Vehicle::getSpeed() const {
    return speed;
}

void Vehicle::setSpeed(float mSpeed) {
    Vehicle::speed = mSpeed;
}

float Vehicle::getTotalTimeCost() const {
    return total_time_cost;
}

void Vehicle::setTotalTimeCost(float totalTimeCost) {
    total_time_cost += totalTimeCost;
}

float Vehicle::getCurrentTimeCost() const {
    return current_time_cost;
}

void Vehicle::setCurrentTimeCost(float currentTimeCost) {
    current_time_cost += currentTimeCost;
}

int Vehicle::getCurrentCapacity() const {
    return current_capacity;
}

void Vehicle::setCurrentCapacity(int currentCapacity) {
    current_capacity += currentCapacity;
}

int Vehicle::getVehicleIndex() const {
    return vehicle_index;
}

void Vehicle::setVehicleIndex(int vehicleIndex) {
    vehicle_index = vehicleIndex;
}

void Vehicle::set_current_time_cost_and_current_capacity(float currentTimeCost, int currentCapacity) {
    this->current_capacity = currentCapacity;
    this->current_time_cost = currentTimeCost;
}