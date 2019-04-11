#include "Route_2pinnets.h"

#include <boost/multi_array/base.hpp>
#include <boost/multi_array/multi_array_ref.hpp>
#include <boost/multi_array/subarray.hpp>
#include <boost/range/iterator_range_core.hpp>
#include <stdlib.h>
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <iterator>
#include <queue>
#include <stack>
#include <utility>
#include <omp.h>

#include "../flute/flute-ds.h"
#include "flute4nthuroute.h"
#include "../grdb/EdgePlane.h"
#include "../grdb/RoutingComponent.h"
#include "../grdb/RoutingRegion.h"

#define SPDLOG_TRACE_ON
#include "../spdlog/common.h"
#include "../spdlog/details/logger_impl.h"
#include "../spdlog/details/spdlog_impl.h"
#include "../spdlog/logger.h"
#include "Congestion.h"
#include "Construct_2d_tree.h"
#include "Range_router.h"

#include "../spdlog/spdlog.h"

namespace NTHUR {

using namespace std;

Route_2pinnets::Route_2pinnets(Construct_2d_tree& construct_2d_tree, RangeRouter& rangerouter, Congestion& congestion) :
        rr_map { construct_2d_tree.rr_map }, //
        gridcell { boost::extents[rr_map.get_gridx()][rr_map.get_gridy()] }, //
        colorMap { boost::extents[rr_map.get_gridx()][rr_map.get_gridy()] }, //
        dirTransferTable { 1, 0, 3, 2 }, //
        construct_2d_tree { construct_2d_tree }, //
        rangerouter { rangerouter }, //
        congestion { congestion } {
    log_sp = spdlog::get("NTHUR");
}

void Route_2pinnets::allocate_gridcell() {
	#pragma omp parallel for // new -> praticamente a mesma coisa, pode continuars
    for (u_int32_t x = 0; x < gridcell.size(); ++x) {
		#pragma omp parallel for // new -> rodou de boa, melhorou tempo
        for (u_int32_t y = 0; y < gridcell[0].size(); ++y) {
            gridcell[x][y].set(x, y);
        }
    }
    SPDLOG_TRACE(log_sp, "initialize gridcell successfully");
}

void Route_2pinnets::init_gridcell() {
    for (u_int32_t x = 0; x < gridcell.size(); ++x) {
        for (u_int32_t y = 0; y < gridcell[0].size(); ++y) {
            gridcell[x][y].points.clear();
        }
    }
    std::for_each(construct_2d_tree.two_pin_list.begin(), construct_2d_tree.two_pin_list.end(), [&](auto&& two_pin) {
    	int cur_x = two_pin.pin1.x;
		int cur_y = two_pin.pin1.y;
		gridcell[cur_x][cur_y].points.push_back(&two_pin);
		//add pin2
		cur_x = two_pin.pin2.x;
		cur_y = two_pin.pin2.y;
		gridcell[cur_x][cur_y].points.push_back(&two_pin);
    });

//    for (Two_pin_element_2d& two_pin : construct_2d_tree.two_pin_list) {
//        //add pin1
//        int cur_x = two_pin.pin1.x;
//        int cur_y = two_pin.pin1.y;
//        gridcell[cur_x][cur_y].points.push_back(&two_pin);
//        //add pin2
//        cur_x = two_pin.pin2.x;
//        cur_y = two_pin.pin2.y;
//        gridcell[cur_x][cur_y].points.push_back(&two_pin);
//    }
}

void Route_2pinnets::route_all_2pin_net() {
    init_gridcell();
    rangerouter.define_interval();
    rangerouter.divide_grid_edge_into_interval();
    rangerouter.specify_all_range(gridcell);
}

void Route_2pinnets::reset_c_map_used_net_to_one() {

	std::for_each(congestion.congestionMap2d.all().begin(), congestion.congestionMap2d.all().end(), [&](auto&& edge){
//		#pragma omp parallel for // new aqui nao da
		for (auto& routeNetTable : edge.used_net) {
			edge.used_net[routeNetTable.first] = 1;
		}
	});
}

//set terminal type to c_map_2d
//void set_c_map_terminal_type(int net_id)
void Route_2pinnets::put_terminal_color_on_colormap(int net_id) {
	std::for_each(rr_map.get_net(net_id).get_pinList().begin(), rr_map.get_net(net_id).get_pinList().end(), [&](auto&& pin) {
		colorMap[pin.x][pin.y].terminal = net_id;
	});
}

//return: one-degree terminal, non-one-degree terminal, one-degree nonterminal, steiner point, two-degree (dir)
Coordinate_2d Route_2pinnets::determine_is_terminal_or_steiner_point(Coordinate_2d& c, Coordinate_2d& head, int net_id, PointType& pointType) {
    Coordinate_2d result;
     if (colorMap[c.x][c.y].terminal == net_id) {
//    	std::for_each(congestion.congestionMap2d.neighbors(c).begin(), congestion.congestionMap2d.neighbors(c).end(), [&](auto&& h) {
//    		if (h.vertex() != head && h.edge().lookupNet(net_id)) {
//				pointType = severalDegreeTerminal;
//				return result;
//			}
//    	});

         for (EdgePlane<Edge_2d>::Handle& h : congestion.congestionMap2d.neighbors(c)) {
             if (h.vertex() != head && h.edge().lookupNet(net_id)) {
                 pointType = severalDegreeTerminal;
                 return result;
             }
         }
         pointType = oneDegreeTerminal; // CHANGED AFTER GCOV

     } else {
        int other_passed_edge = 0;

//        std::for_each(congestion.congestionMap2d.neighbors(c).begin(), congestion.congestionMap2d.neighbors(c).end(), [&](auto&& h) {
//        	if (h.vertex() != head && h.edge().lookupNet(net_id)) {
//			++other_passed_edge;
//			if (other_passed_edge > 1) {
//				pointType = steinerPoint;
//				return -1;
//			}
//			result = h.vertex();
//		}
//		});

         for (EdgePlane<Edge_2d>::Handle& h : congestion.congestionMap2d.neighbors(c)) {
             if (h.vertex() != head && h.edge().lookupNet(net_id)) {
                 ++other_passed_edge;
                 if (other_passed_edge > 1) {
                     pointType = steinerPoint;
                     return -1;
                 }
                 result = h.vertex();
             }
         } // CHANGED AFTER GCOVER
        if (other_passed_edge == 0) {
            pointType = oneDegreeNonterminal;

        } else {
            pointType = twoDegree;
        }
     }
    return result;
}

void Route_2pinnets::add_two_pin(int net_id, std::vector<Coordinate_2d>& path) {
    if (path.size() > 1) {
//		#pragma omp parallel // novo teste parallel sem for -> aqui trava no inicio
//    	{
			construct_2d_tree.two_pin_list.emplace_back();
			Two_pin_element_2d& two_pin = construct_2d_tree.two_pin_list.back();
			two_pin.pin1 = path.front();
			two_pin.net_id = net_id;
			two_pin.pin2 = path.back();
			two_pin.path = path;
			path.clear();
			path.push_back(two_pin.pin2);
//    	}
    }
}

void Route_2pinnets::fillTree(int offset, int net_id) {
    int sizeTree = construct_2d_tree.two_pin_list.size() - offset;
    TreeFlute& tree = construct_2d_tree.net_flutetree[net_id];

    tree.branch.resize(sizeTree + 1);

    std::unordered_map<Coordinate_2d, int> indexmap;
    indexmap.reserve(sizeTree + 1);

    Two_pin_element_2d& first_pin = construct_2d_tree.two_pin_list.at(offset);
    indexmap.emplace(first_pin.pin1, 0);

    tree.branch[0].x = first_pin.pin1.x;
    tree.branch[0].y = first_pin.pin1.y;
    tree.branch[0].n = 0;

    for (int i = 1; i < sizeTree + 1; ++i) {
        Two_pin_element_2d& two_pin = construct_2d_tree.two_pin_list.at(offset + i - 1);

        indexmap.emplace(two_pin.pin2, i);

        tree.branch[i].x = two_pin.pin2.x;
        tree.branch[i].y = two_pin.pin2.y;
        tree.branch[i].n = indexmap.at(two_pin.pin1);

    }
    tree.number = sizeTree + 1;
}

void Route_2pinnets::bfs_for_find_two_pin_list(Coordinate_2d start_coor, int net_id) {
    std::stack<std::vector<Coordinate_2d>> stack;
    stack.emplace();
    stack.top().push_back(start_coor);

    int offset = construct_2d_tree.two_pin_list.size();

    while (!stack.empty()) {
        std::vector<Coordinate_2d>& path = stack.top();
        const Coordinate_2d& c = path.back();
        colorMap[c.x][c.y].traverse = net_id;
        if (colorMap[c.x][c.y].terminal == net_id) {
            add_two_pin(net_id, path);
        }
        std::vector<Coordinate_2d> neighbors;
        neighbors.reserve(4);
        std::for_each(congestion.congestionMap2d.neighbors(c).begin(), congestion.congestionMap2d.neighbors(c).end(), [&](auto&& h) {
        	if (h.edge().lookupNet(net_id) && //
					(colorMap[h.vertex().x][h.vertex().y].traverse != net_id)) {
				neighbors.push_back(h.vertex());
			}
        });
        switch (neighbors.size()) {
        case 0:
            congestion.update_congestion_map_remove_two_pin_net(path, net_id);
            stack.pop();
            break;
        default:
            add_two_pin(net_id, path);
//			#pragma omp parallel for // new aqui trava no inicio ctz
            for (std::size_t i = 1; i < neighbors.size(); ++i) {
                stack.emplace(path);
                stack.top().emplace_back(neighbors.at(i));
            }  // no break on purpose !!!
        case 1:
            path.emplace_back(neighbors.at(0));
            break;
        }
    }
    fillTree(offset, net_id);
}

void Route_2pinnets::reallocate_two_pin_list() {
//	#pragma omp parallel for // new -> nao eh bom aqui, aumenta mt o tempo
    for (u_int32_t i = 0; i < colorMap.num_elements(); ++i) {
        colorMap.data()[i].set(-1, -1);
    }

    reset_c_map_used_net_to_one();

    vector<Two_pin_element_2d>& v = construct_2d_tree.two_pin_list;
// erase-remove idiom
    v.erase(std::remove_if(v.begin(), v.end(), [& ](const Two_pin_element_2d& pin) {
        return construct_2d_tree.NetDirtyBit[pin.net_id];
    }), v.end());

//	#pragma omp parallel for // new -> aqui d[a problema terminate called recursively
    for (uint32_t netId = 0; netId < rr_map.get_netNumber(); ++netId) {
        if (construct_2d_tree.NetDirtyBit[netId]) {
            put_terminal_color_on_colormap(netId);

            bfs_for_find_two_pin_list(rr_map.get_net(netId).get_pinList()[0].xy(), netId);

            construct_2d_tree.NetDirtyBit[netId] = false;

        }
    }
}

} // namespace NTHUR
