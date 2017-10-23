//
// Created by NikoohematS on 1-9-2017.
//

#ifndef INDOOR_GEOMETRY_REOCNSTRUCTION_INDOOR_RECONSTRUCTION_H
#define INDOOR_GEOMETRY_REOCNSTRUCTION_INDOOR_RECONSTRUCTION_H

#endif //INDOOR_GEOMETRY_REOCNSTRUCTION_INDOOR_RECONSTRUCTION_H

#include <iostream>
#include <cstdlib>
#include <limits>
#include <LaserPoints.h>
#include "Buffers.h"

/// looking at segments connection (vetical, horizontal,...) and build a structured graph
/// use the graph(adjacency list) to detect walls, floor, ceiling and clutter
void indoorTopology(char *laserFile, int minsizesegment, bool verbose);

/// detection of openings by occlusion reasoning
/// build a ray from scanner position to the surface(walls, or any segment)
/// outputs are openings, occlusion and occupied in the form of labeled center voxels
void occlusion_test(LaserPoints , LaserPoints , LaserPoints , Buffers , double );

/// DepricATED FUNCTION, replaced with boost::unordered::multimap
/// This function use a simple binary function to find corresponding point by its trajectory point using time
void FilterSegments_ByTimeTag(LaserPoints , LaserPoints , double , double , double );

/// merge segments in a defined buffer based on their geometry similarity (co-planarity, proximity,...)
/// Buffers are result of the function in the form of renumbered segments
Buffers GenerateWallPatches(LaserPoints , double , double , double );

/// reshpae segments with strange shapes with calculating TIN. expensive function
LaserPoints segment_refinement(LaserPoints , int , double );

///
void DepthMap(LaserPoints , LaserPoints);

/// calculate wall accuracy, F-score, precision, by comparing label with label2 as ground truth
void WallAccuracy(int);

/// This function is not complete, it doesn't perform as expected
void RemoveDuplicatePoints(LaserPoints , LaserPoints );