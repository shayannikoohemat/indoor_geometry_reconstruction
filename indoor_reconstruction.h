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


/// ReadAscii.cpp
LaserPoints read_ascii(char *ascii_file);

/// conversion of customized ascii files e.g. Navvis trajectory
bool read_custom_ascii (const string &custom_ascii_file, LaserPoints &lp_out, char * lp_outfile);


/// select a sub-section/partition of laserpoints between two given time_tag as double
LaserPoints PartitionPointsByTime(LaserPoints &sorted_lpoints,vector<double> lpoints_v, double t_low, double t_up);

/// test function for partition points by time tag
void PartitionPointsByTrajecotry_test(LaserPoints &lpoints, LaserPoints &trajectory, char* out_root= nullptr, bool sort_input=false);

vector<LaserPoints> PartitionPointsByTrajecotry (LaserPoints &sorted_lpoints,
                                                 LaserPoints &segmented_trajectory, char* output= nullptr, bool sort_input=false);

/// mirror points about a plane, glass_points is making the plane and reflected_points are points to be mirrored
LaserPoints mirror_PointsToPlane(LaserPoints &glass_points, LaserPoints &reflected_points);

/// test function for mirror points about a plane and visualize the result
void mirror_PointsToPlane_test_function(LaserPoints &glass_points, LaserPoints &reflected_points,
                                         LaserPoints &traj_points, char *out_root);

LaserPoints Segment_Trajectory_ByTime (LaserPoints trajectory, double time_threshold);

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

bool compare_p_time (const LaserPoint &p1, const LaserPoint &p2);

void LaserPoints_info (LaserPoints &laserpoints);

/// trim the long float of a DoubleTag e.g. time, used for conversion to and from CloudCompare
/// the function is not tested
int Trim_doubleTag (LaserPoints &laserpoints, LaserPointTag double_tag, double trim_value);

/// add value the long float of a DoubleTag e.g. time,
/// the function is not tested
int add_to_doubleTag (LaserPoints &laserpoints, LaserPointTag double_tag, double add_value);

/// flip normals toward the scanner position
void Normal_Flip (LaserPoints &laserpoints, LaserPoints &scanner_positions, int knn=10);

/// return labeled laserpoints with label=1 as horizontal and label=2 as vertical
/// the angle_threshold is in degree and is used for checking the normal deviation
/// of the plane for each segment with the Z-axis direction
LaserPoints LabelPoints_To_Ver_Horizon (LaserPoints &laserpoints, double angle_threshold);

/// partition points by a give vector of Z_peaks (e.g. extracted from histogram) and a tolerance
void PartitionPoints_By_Zpeaks(LaserPoints &laserpoints, vector<double> Z_peaks, double z_tolerance);

/// temporary function to remove not necessary points in backpack data
/// remove points with a specific tag's value or without tag
void FilterPointsByTag(LaserPoints &laserpoints, LaserPointTag tag, int tag_value, LaserPoints &lp_filtered);

void Segmentation_SurfaceGrowing(char *input, char * output, double max_dis_to_plane=0.05, double max_dist_to_surf=0.05);
/*void Segmentation_SurfaceGrowing(LaserPoints lp_input, LaserPoints &lp_segmented,
                                 double max_dis_to_plane, double max_dist_to_surf);*/

/// For using in ChangeDetection: find the location of changes in the original pointclouds and relabel them
/// find the result of changedetection in the walls, floor and ceiling
LaserPoints relabel_changed_points (LaserPoints &detected_changes, LaserPoints & original_points, double dist_threshold=0.001);

/// merge segmented point clouds and resegment them
void merge_lp_segmented (char *input_directory, char *output_file);
