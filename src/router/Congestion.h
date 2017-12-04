/*
 * Bom.h
 *
 *  Created on: Nov 29, 2017
 *      Author: florian
 */

#ifndef SRC_ROUTER_CONGESTION_H_
#define SRC_ROUTER_CONGESTION_H_

#include <functional>
#include <vector>

#include "../grdb/EdgePlane.h"
#include "../misc/geometry.h"
#include "DataDef.h"

class RoutingRegion;

struct CacheEdge {
    double cost;               //Used as cache of cost in whole program
    int MMVisitFlag;        //Used as cache of hash table lookup result in MM_mazeroute

    CacheEdge() :
            cost(0.0), MMVisitFlag(-1) {
    }
};

class Congestion {
public:

    struct Statistic {

        double min;
        double max;
        double avg;
    };

    std::function<void(int i, int j, OrientationType dir)> pre_evaluate_congestion_cost_fp;

    int via_cost;
    int used_cost_flag;
    double exponent;
    double WL_Cost;
    double factor;

    EdgePlane<Edge_2d> congestionMap2d;
    EdgePlane<CacheEdge> cache;
    Congestion(int x, int y);
    virtual ~Congestion();
    double get_cost_2d(int i, int j, OrientationType dir, int net_id, int *distance);
    int cal_max_overflow();
    void pre_evaluate_congestion_cost_all(int i, int j, OrientationType dir);
    void pre_evaluate_congestion_cost();
    bool check_path_no_overflow(std::vector<Coordinate_2d>&path, int net_id, int inc_flag);
    int find_overflow_max();
    void init_2d_map(RoutingRegion& rr_map);
    int cal_total_wirelength();
    Statistic stat_congestion();
};

#endif /* SRC_ROUTER_CONGESTION_H_ */
