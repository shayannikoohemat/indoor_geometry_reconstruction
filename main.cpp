#include <iostream>
#include "InlineArguments.h"
#include <cstdlib>
#include <limits>
#include <LaserPoints.h>
#include "Buffers.h"
#include "indoor_reconstruction.h"
#include "post_processing.h"

using namespace std;

void PrintUsage()
{
    printf("Usage: indoor_reconstruction -input_laser <input laserpoints , foo.laser>\n");
    printf("                  -minsizesegment <minimum segment size e.g. 200>\n");
}

void VisulizePlane3D (LaserPoints segment_lp, double max_dist, ObjectPoints &plane_corners,
                      LineTopology &plane_edges, bool verbose);

int main(int argc, char *argv[]) {

    InlineArguments     *args = new InlineArguments(argc, argv);

    /// Check on required input files
/*    if (args->Contains("-usage") ||
        !args->Contains("-input_laser"))
        //!args->Contains("-method"))
    {
        if (!args->Contains("-usage")) printf("Error: missing programme option.\n");
        PrintUsage();
        return EXIT_SUCCESS;
    }*/

    /// Call the main functions
    LaserPoints lpoints, surfpoints;
    //segment2D(lpoints, 250);
    LaserPoints traj_lp;
    //double time, percentage;
    //FilterSegments_ByTimeTag(lpoints, traj_lp, 150.0 , 10.0, 0.60);
    //FilterPoints_ByTimeTag(lpoints, traj_lp, 150);
    //char* root = (char*) "D://test//indoor_reconstruction//";
    //GenerateWallPatches(lpoints, 0.70, 20.0, 0.5, root);

    char* file_lp;
    //file_lp = (char*) "D://test//indoorgraph//data//fireBr_Cloud_16mil_sub_1cm_3rdFloor_2mil_thinned_seg1_12cm.laser"; // gym_raw_thinned_455k.laser //
    //file_lp = (char*) "D://test//indoor_reconstruction//data//FB_sub1cm_seg12cm_refined_1770k_reflectremoved.laser";
    //file_lp = (char*) "E://BR_data//ZebR//cc_sub2mil_basement_crop_seg10cm.laser";
    //file_lp  = (char *) "E://BR_data//Diemen//process//block3_seg05cm_1-7mil_seg10cm_label0.laser" ; //tworooms_labeled0.laser"
    //file_lp  = (char *) "D:/test/zebrevo_ahmed/Revo_cloud_thinned_seg10cm.laser" ; //Revo_cloud_crop.laser" ; //
    file_lp = (char*) "E:/Laser_data/ETH_dataset/penthouse/sample_collection_furniture.laser" ;   //slantedwalls.laser" ;  //penthouse_seg8cm_oneblock.laser" ;
    //lpoints.Read(file_lp);
    //traj_lp.Read("E://BR_data//ZebR//traj_basement_crop.laser");
    //traj_lp.Read("E://BR_data//Diemen//process//traj_diemen1_s_rec.laser");
   //surfpoints.Read("E://BR_data//Diemen//process//out//outputlaser.laser");

    /*remove points with label -1 from backpack system*/
/*    LaserPoints lp_labeled0;
    for (auto p : lpoints){
        if(p.Attribute(LabelTag) != -1) {
            p.SetAttribute(LabelTag, 0);
            lp_labeled0.push_back(p);
        }
    }
    lp_labeled0.Write("E://BR_data//Diemen//process//block3_seg05cm_1-7mil_seg10cm_label0.laser", false);*/

    //auto *root = (char*) "D:/test/zebrevo_ahmed/out/";
    auto *root = (char*) "E:/Laser_data/ETH_dataset/penthouse/out/" ;
    //indoorTopology(file_lp, 100, root, false);
    Buffers bfs;
    //traj_lp.Read("D:/test/zebrevo_ahmed/Revo_Trajectory.laser");
    //surfpoints.Read("D:/test/zebrevo_ahmed/out/indoortopology_nobuffer/walls.laser");
    //occlusion_test(lpoints, surfpoints, traj_lp, bfs , 0.12, root);

    //LaserPoints refined_lp;
    //refined_lp = segment_refinement(lpoints, 2, 0.50);
    //refined_lp.Write("D://test//buffer_Kada//FB_sub1cm_seg8cm_665k_refined2_reflectremoved.laser", false);

    //LaserPoints traj_lp, lp;
    //DepthMap(traj_lp, lp);

    //int label;
    //WallAccuracy(label);
    //RemoveDuplicatePoints(lpoints, lp);

    /// post_processing test
    //auto * output = (char*) "D:/test/sims3d/out/";
    auto * output = (char*) "E:/Laser_data/ETH_dataset/penthouse/out/";
    LaserPoints lp_segmented;
    //lp_segmented.Read ("D:/test/sims3d/data/post_processing/wall_crop2.laser");   //walls_simplified  //
    //lp_segmented.Read("E:/Laser_data/ETH_dataset/penthouse/intersection_problem2.laser") ; //sample_collection.laser");
    //intersect_segments (lp_segmented, 0.10, 0.10, 1.0, output);

    /// external function: visualization of planes
/*    LaserPoints lp_segments;
    lp_segments.Read("E:/Laser_data/ETH_dataset/penthouse/out/ceiling.laser") ;
    /// make laserpoints from each segment
    vector<LaserPoints> segments;
    segments = PartitionLpByTag (lp_segments, SegmentNumberTag);
    ObjectPoints plane_corners;
    LineTopology plane_edges;
    LineTopologies planes_edges;

    bool verbose=false;
    for(auto &segment : segments) {
        segment.Write ("E:/Laser_data/ETH_dataset/penthouse/out/segment.laser", false);
        VisulizePlane3D (segment, 0.10, plane_corners, plane_edges, true);
        planes_edges.push_back (plane_edges);
    }

    plane_corners.Write("E:/Laser_data/ETH_dataset/penthouse/out/plane_corners.objpts");
    planes_edges.Write("E:/Laser_data/ETH_dataset/penthouse/out/plane_edges.top", false);*/

    /* testing boost geometry library for intersecting minimum rectangles
     *
     * */
    LaserPoints segments_lp;
    segments_lp.Read ("E:/Laser_data/ETH_dataset/penthouse/floor_ceil.laser");

    vector<LaserPoints> segments;
    segments = PartitionLpByTag (segments_lp, SegmentNumberTag);

    vector <ObjectPoints> corners_vec ;
    vector <LineTopology> edges_vec;
    for (auto &segment : segments){
        ObjectPoints corners;
        LineTopology edges;
        DataBoundsLaser db = segment.DeriveDataBounds (0);
        segment.DeriveTIN ();
        segment.EnclosingRectangle (1.0, corners, edges);
        corners_vec.push_back (corners);
        edges_vec.push_back (edges);
    }

    //ObjectPoints corners1, corners2;
    std::deque<boost_polygon> poly_out;
    poly_out = intersect_polygons (corners_vec[0], corners_vec[1]);

    /*        {
        //get the databounds from the ls TO visualize
        DataBoundsLaser db = segment.DeriveDataBounds (0);
        LineTopology rect_topology;
        ObjectPoints corners;
        LineTopologies bounding_box;
        segment.DeriveTIN();
        //db = ls.DeriveDataBounds(0);
        segment.EnclosingRectangle(0.1, corners, rect_topology);

        int next_number; /// the next_number is the number after the last number in the corners file
        if (corners.empty()) next_number = 4; /// later next_number-4 =0
        else next_number = (corners.end() - 1)->Number() + 1;

        /// add z values to 4 corners to make upper rectangle
        for (int i=next_number -4; i < next_number ; i++){
            corners[i].Z() = db.Maximum().GetZ(); // upper rectangle
        }

        ObjectPoint obj_pnt;
        /// constructing below recangle
        for (int i=0; i < 4; i++){

            obj_pnt.X() = corners[next_number -4 + i].X();
            obj_pnt.Y() = corners[next_number -4 + i].Y();
            obj_pnt.Z() = db.Minimum().GetZ(); // below rectangle
            obj_pnt.Number() = next_number + i;
            corners.push_back(obj_pnt);
            rect_topology.push_back(PointNumber(next_number + i));
        }
        rect_topology.push_back(PointNumber(next_number)); // close the polygon // clockwise
        rect_topology.MakeCounterClockWise(corners);

        /// write minium enclosing rectangle to the disk
        corners.Write("E:/Laser_data/ETH_dataset/penthouse/out/min_rectangle.objpts");
        bounding_box.insert(bounding_box.end(), rect_topology);
        bounding_box.Write("E:/Laser_data/ETH_dataset/penthouse/out/min_rectangle.top", false);
    }*/


    std::cout << "Press ENTER to continue...";
    std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
}