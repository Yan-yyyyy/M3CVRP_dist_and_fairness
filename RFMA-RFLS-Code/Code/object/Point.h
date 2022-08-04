//
// Created by Joe Lan on 2020/7/2.
//

#ifndef UWCP_POINT_H
#define UWCP_POINT_H

#include <string>
using namespace std;

class Point {
private:
    string point_id;
    int point_index;
    int lat;
    int lng;
    int server_cost;
    int point_capacity;
    int point_type;
    int working_time;

public:
    Point(const string & pointId, int pointIndex, int latitude, int longitude, int serverCost, int pointCapacity,
          int pointType, int workingTime);

    const string &getPointId() const;

    void setPointId(const string &pointId);

    int getPointIndex() const;

    void setPointIndex(int pointIndex);

    float getLat() const;

    void setLat(float latitude);

    float getLng() const;

    void setLng(float longitude);

    int getServerCost() const;

    void setServerCost(int serverCost);

    int getPointCapacity() const;

    void setPointCapacity(int pointCapacity);

    int getPointType() const;

    void setPointType(int pointType);

    int getWorkingTime() const;

    void setWorkingTime(int workingTime);
};


#endif //UWCP_POINT_H
