/*
 * OutputGeneration.cpp
 *
 *  Created on: Jan 15, 2018
 *      Author: florian
 */

#include "OutputGeneration.h"

#include <boost/range/iterator_range_core.hpp>
#include <algorithm>
#include <cstddef>
#include <limits>
#include <utility>

#include "../grdb/EdgePlane.h"
#include "../grdb/plane.h"
#include "../grdb/RoutingRegion.h"
#include "../spdlog/spdlog.h"

namespace NTHUR {

OutputGeneration::OutputGeneration(const RoutingRegion& rr) :
        rr_map { rr }, //
        cur_map_3d { Coordinate_3d { rr.get_gridx(), rr.get_gridy(), rr.get_layerNumber() } }    //
{
    // TODO Auto-generated constructor stub
    for (Edge_3d& edge : cur_map_3d.all()) {
        edge.cur_cap = 0;
    }
    for (int k = 0; k < rr.get_layerNumber(); ++k) {

        const EdgePlane<int> edges = rr.getLayer(k).edges();
        for (int x = 0; x < rr.get_gridx(); ++x) {
            for (int y = 0; y < rr.get_gridy(); ++y) {
                Coordinate_3d c = Coordinate_3d { x, y, k };
                cur_map_3d.east(c).max_cap = edges.east(Coordinate_2d { x, y });
                cur_map_3d.south(c).max_cap = edges.south(Coordinate_2d { x, y });
                cur_map_3d.front(c).max_cap = std::numeric_limits<int>::max();
            }

        }
    }
    log_sp = spdlog::get("NTHUR");
}

void OutputGeneration::generate_all_output(std::ostream & output) const {

    std::vector<std::vector<Segment3d> > comb { static_cast<std::size_t>(rr_map.get_netNumber()) };
    Coordinate_3d c2;
    for (Coordinate_3d c { 0, 0, 0 }; c.x < cur_map_3d.getXSize(); ++c.x) {
        c2.x = c.x;
        for (c.y = 0; c.y < cur_map_3d.getYSize(); ++c.y) {
            c2.y = c.y;
            for (c.z = 1; c.z < cur_map_3d.getZSize(); ++c.z) {
                c2.z = c.z - 1;
                collectComb(c2, c, comb);
            }
        }
        for (c.z = 0; c.z < cur_map_3d.getZSize(); ++c.z) {
            c2.z = c.z;
            for (c.y = 1; c.y < cur_map_3d.getYSize(); ++c.y) {
                c2.y = c.y - 1;
                collectComb(c2, c, comb);
            }
        }
    }
    for (Coordinate_3d c { 0, 0, 0 }; c.y < cur_map_3d.getYSize(); ++c.y) {
        c2.y = c.y;
        for (c.z = 0; c.z < cur_map_3d.getZSize(); ++c.z) {
            c2.z = c.z;
            for (c.x = 1; c.x < cur_map_3d.getXSize(); ++c.x) {
                c2.x = c.x - 1;
                collectComb(c2, c, comb);
            }
        }
    }

    for (uint32_t i = 0; i < rr_map.get_netNumber(); ++i) {
        generate_output(i, comb[i], output);
    }

}

void OutputGeneration::printEdge(const Coordinate_3d& c, const Coordinate_3d& c2) const {

    log_sp->info("Edge3d {} between {} and {}", cur_map_3d.edge(c, c2).toString(), c.toString(), c2.toString());
}

void OutputGeneration::plotNet(int net_id) const {

    std::vector<std::vector<Segment3d> > comb { static_cast<std::size_t>(rr_map.get_netNumber()) };
    Coordinate_3d c2;
    for (Coordinate_3d c { 0, 0, 0 }; c.x < cur_map_3d.getXSize(); ++c.x) {
        c2.x = c.x;
        for (c.y = 0; c.y < cur_map_3d.getYSize(); ++c.y) {
            c2.y = c.y;
            for (c.z = 1; c.z < cur_map_3d.getZSize(); ++c.z) {
                c2.z = c.z - 1;
                if (cur_map_3d.edge(c, c2).used_net.find(net_id) != cur_map_3d.edge(c, c2).used_net.end()) {
                    printEdge(c, c2);
                }
            }
        }
        for (c.z = 0; c.z < cur_map_3d.getZSize(); ++c.z) {
            c2.z = c.z;
            for (c.y = 1; c.y < cur_map_3d.getYSize(); ++c.y) {
                c2.y = c.y - 1;
                if (cur_map_3d.edge(c, c2).used_net.find(net_id) != cur_map_3d.edge(c, c2).used_net.end()) {
                    printEdge(c, c2);
                }
            }
        }
    }
    for (Coordinate_3d c { 0, 0, 0 }; c.y < cur_map_3d.getYSize(); ++c.y) {
        c2.y = c.y;
        for (c.z = 0; c.z < cur_map_3d.getZSize(); ++c.z) {
            c2.z = c.z;
            for (c.x = 1; c.x < cur_map_3d.getXSize(); ++c.x) {
                c2.x = c.x - 1;
                if (cur_map_3d.edge(c, c2).used_net.find(net_id) != cur_map_3d.edge(c, c2).used_net.end()) {
                    printEdge(c, c2);
                }
            }
        }
    }

    log_sp->info("end Layer plotNet true");
}
void OutputGeneration::calculate_wirelength(const int global_via_cost) const {

    int xy = 0;
    int z = 0;

    for (Coordinate_3d c { 0, 0, 0 }; c.x < cur_map_3d.getXSize(); ++c.x) {
        for (c.y = 0; c.y < cur_map_3d.getYSize(); ++c.y) {
            for (c.z = 0; c.z < cur_map_3d.getZSize(); ++c.z) {
                xy += cur_map_3d.east(c).used_net.size();
                xy += cur_map_3d.south(c).used_net.size();
                z += cur_map_3d.front(c).used_net.size();
            }
        }
    }
    z *= global_via_cost;

    log_sp->info("total wire length = {} + {} = {}", xy, z, (xy + z));
}

void OutputGeneration::generate_output(int net_id, const std::vector<Segment3d>& v, std::ostream & output) const {

    int xDetailShift = rr_map.get_llx() + (rr_map.get_tileWidth() / 2);
    int yDetailShift = rr_map.get_lly() + (rr_map.get_tileHeight() / 2);

// the beginning of a net of output file

    output << rr_map.get_netName(net_id) << " " << rr_map.get_netSerialId(net_id) << " " << v.size() << "\n";

    // have edge
    for (const Segment3d& seg : v) {
        Coordinate_3d o = seg.first;
        Coordinate_3d d = seg.last;
        o.x = o.x * rr_map.get_tileWidth() + xDetailShift;
        o.y = o.y * rr_map.get_tileHeight() + yDetailShift;
        ++o.z;
        d.x = d.x * rr_map.get_tileWidth() + xDetailShift;
        d.y = d.y * rr_map.get_tileHeight() + yDetailShift;
        ++d.z;
        output << "(" << o.x << "," << o.y << "," << o.z << ")-(" << d.x << "," << d.y << "," << d.z << ")\n";
    }

// the end of a net of output file
    output << "!\n";
}
void OutputGeneration::collectComb(Coordinate_3d c2, Coordinate_3d& c, std::vector<std::vector<Segment3d> >& comb) const {
    for (const std::pair<const int, int>& net : cur_map_3d.edge(c, c2).used_net) {
        std::vector<Segment3d>& v = comb[net.first];
        if (v.empty()) {
            v.push_back(Segment3d { c2, c });
        } else {
            Segment3d& interval = v.back();
            if (interval.last == c2 && interval.first.isAligned(c)) {
                interval.last = c;
            } else {
                v.push_back(Segment3d { c2, c });
            }
        }
    }
}
void OutputGeneration::print_max_overflow() const {
    int lines = 0;
    int max = 0;
    int sum = 0;
    for (const Edge_3d& edgeLeft : cur_map_3d.all()) {

        if (edgeLeft.isOverflow()) {
            max = std::max(edgeLeft.overUsage(), max);
            sum += edgeLeft.overUsage();
            ++lines;
        }
    }
    log_sp->info("       3D # of overflow = {} ", sum);
    log_sp->info("       3D max  overflow = {} ", max);
    log_sp->info("3D overflow edge number = {} ", lines);

}


} /* namespace NTHUR */