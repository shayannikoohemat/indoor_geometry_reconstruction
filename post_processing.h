//
// Created by NikoohematS on 23-11-2017.
//

#include <boost/geometry.hpp>

#ifndef INDOOR_GEOMETRY_REOCNSTRUCTION_POST_PROCESSING_H
#define INDOOR_GEOMETRY_REOCNSTRUCTION_POST_PROCESSING_H

#endif //INDOOR_GEOMETRY_REOCNSTRUCTION_POST_PROCESSING_H

void intersect_segments(const LaserPoints &segmented_lp, double start_threshold, double increase_step_size,
                        double stop_threshold, char* output_dir);

typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> boost_point;
typedef boost::geometry::model::polygon<boost_point> boost_polygon;
std::deque<boost_polygon> intersect_polygons(ObjectPoints const &poly1_corners, ObjectPoints const &poly2_corners);
