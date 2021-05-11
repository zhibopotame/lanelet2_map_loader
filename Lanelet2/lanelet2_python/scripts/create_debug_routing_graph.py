#!/usr/bin/env python

import lanelet2
import sys
import argparse

if __name__ == "__main__":
    # parser = argparse.ArgumentParser()
    # parser.add_argument("filename", help="Path to the input osm file", default="/media/zlin/DATA/catkin_local_map/src/lanelet_map_loader/map_loader/data/lanelet2_map.osm")
    # parser.add_argument("output", help="Path to resulting debug routing graph", default="./")
    # parser.add_argument(
    #     "--participant",
    #     help="traffic participant type (one of vehicle, bicycle, pedestrian, train",
    #     type=str,
    #     required=False,
    #     default="vehicle")
    # parser.add_argument("--lat", help="Lateral position of origin", type=float, default=49)
    # parser.add_argument("--lon", help="Longitudinal position of origin", type=float, default=8)
    # args = parser.parse_args()

    rules_map = {"vehicle": lanelet2.traffic_rules.Participants.Vehicle,
                 "bicycle": lanelet2.traffic_rules.Participants.Bicycle,
                 "pedestrian": lanelet2.traffic_rules.Participants.Pedestrian,
                 "train": lanelet2.traffic_rules.Participants.Train}
    proj = lanelet2.projection.UtmProjector(lanelet2.io.Origin(0, 0))
    laneletmap = lanelet2.io.load("/media/zlin/DATA/catkin_local_map/src/lanelet_map_loader/map_loader/data/hs_complexe.osm", proj)

    routing_cost = lanelet2.routing.RoutingCostDistance(0.)  # zero cost for lane changes
    traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  rules_map["vehicle"])
    graph = lanelet2.routing.RoutingGraph(laneletmap, traffic_rules, [routing_cost])
    debug_map = graph.getDebugLaneletMap()

    lanelet2.io.write("./output.osm", debug_map, proj)
