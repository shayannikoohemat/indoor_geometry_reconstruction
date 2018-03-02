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
void indoorTopology(char *laserFile, int minsizesegment, char* root_dir, bool verbose);

void indoorTopology_v1_old(char *laserFile, int minsizesegment, char* root_dir, bool verbose);

/// detection of openings by occlusion reasoning
/// build a ray from scanner position to the surface(walls, or any segment)
/// outputs are openings, occlusion and occupied in the form of labeled center voxels
void occlusion_test(LaserPoints , const LaserPoints &, LaserPoints , Buffers , double , char *);

/// DepricATED FUNCTION, replaced with boost::unordered::multimap
/// This function use a simple binary function to find corresponding point by its trajectory point using time
void FilterSegments_ByTimeTag(LaserPoints , LaserPoints , double , double , double );

/// merge segments in a defined buffer based on their geometry similarity (co-planarity, proximity,...)
/// Buffers are result of the function in the form of renumbered segments
Buffers GenerateWallPatches(LaserPoints lp, double dist, double angle, double dist_along_twosegments, char* root);

/// reshpae segments with strange shapes with calculating TIN. expensive function
LaserPoints segment_refinement(LaserPoints , int , double );

/// not complete, not used, not correct
void DepthMap(LaserPoints , LaserPoints);

/// calculate wall accuracy, F-score, precision, by comparing label with label2 as ground truth
void WallAccuracy(int);

/// This function is not complete, it doesn't perform as expected
void RemoveDuplicatePoints(LaserPoints , LaserPoints );

/// collect laserpoints per similar_tags and return a vector of them,
std::vector<LaserPoints> PartitionLpByTag(LaserPoints const& lp, LaserPointTag int_tag, const char* output= nullptr);

/// enlarge bounding box of the data by the given size
DataBoundsLaser EnlargeDBounds(const DataBoundsLaser &bounds, double enlargement_size);
/// shrink bounding box of the data by the given size
DataBoundsLaser ShrinkDBounds(const DataBoundsLaser &bounds, double shrinking_size);
/// rescale a rectangle by the given scale factor/ affine transformation
/// e.g. scalefactor = 0.98 makes the rectangle around 5 cm smaller
/// e.g. scalefactor = 1.-2 makes the rectangle around 5 cm bigger
ObjectPoints ScaleRectangle (ObjectPoints &corners, LineTopology &edges, double scalefactor);

bool OverlapXY1(const DataBoundsLaser &bounds1, const DataBoundsLaser &bounds2);

bool compare_lp_segmenttag(const LaserPoints &lp1, const LaserPoints &lp2);

bool compare_lp_labeltag(const LaserPoints &lp1, const LaserPoints &lp2);

bool compare_lp_size(const LaserPoints &lp1, const LaserPoints &lp2);

bool compare_lp_label2tag(const LaserPoints &lp1, const LaserPoints &lp2);
