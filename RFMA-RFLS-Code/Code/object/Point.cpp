//
// Created by Joe Lan on 2020/7/2.
//

#include "Point.h"

Point::Point(const string & pointId, int pointIndex, int latitude, int longitude, int serverCost, int pointCapacity,
             int pointType, int workingTime) {
    this->point_id = pointId;
    this->point_index = pointIndex;
    this->lat = latitude;
    this->lng = longitude;
    this->server_cost = serverCost;
    this->point_capacity = pointCapacity;
    this->point_type = pointType;
    this->working_time = workingTime;
}

const string &Point::getPointId() const {
    return point_id;
}

void Point::setPointId(const string &pointId) {
    point_id = pointId;
}

int Point::getPointIndex() const {
    return point_index;
}

void Point::setPointIndex(int pointIndex) {
    point_index = pointIndex;
}

float Point::getLat() const {
    return lat;
}

void Point::setLat(float latitude) {
    Point::lat = latitude;
}

float Point::getLng() const {
    return lng;
}

void Point::setLng(float longitude) {
    Point::lng = longitude;
}

int Point::getServerCost() const {
    return server_cost;
}

void Point::setServerCost(int serverCost) {
    server_cost = serverCost;
}

int Point::getPointCapacity() const {
    return point_capacity;
}

void Point::setPointCapacity(int pointCapacity) {
    point_capacity = pointCapacity;
}

int Point::getPointType() const {
    return point_type;
}

void Point::setPointType(int pointType) {
    point_type = pointType;
}

int Point::getWorkingTime() const {
    return working_time;
}

void Point::setWorkingTime(int workingTime) {
    working_time = workingTime;
}
