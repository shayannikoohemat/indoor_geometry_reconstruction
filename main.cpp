#include <iostream>
#include "InlineArguments.h"
#include <cstdlib>
#include <limits>
#include <LaserPoints.h>
#include "Buffers.h"
#include "indoor_reconstruction.h"
#include "post_processing.h"
#include "PlaneFittingResidual.h"

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
    file_lp = (char*) "E:/Laser_data/ETH_dataset/penthouse/penthouse_ConnectComp_01_segGV_1-4mil_crop.laser";//sample_collection_furniture.laser";  //slantedwalls.laser" ;  //penthouse_seg8cm_oneblock.laser" ;
    //file_lp = (char*) "E:/Laser_data/Kadaster/test_data/level2_kadaster_crop2_newsegmentation.laser";
    //file_lp = (char*) "D:/test/sims3d/test_seri2/ceiling_floor_test.laser";       //firebrigade_3rdfloor.laser";
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
    //auto *root = (char*) "E:/Laser_data/Kadaster/test_data/out/";
    //auto *root = (char*) "D:/test/sims3d/test_seri2/out/";
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
     * testing EnclosingRectangle and ScaleRectangle functions
     * */
/*    LaserPoints segments_lp;
    segments_lp.Read ("E:/Laser_data/ETH_dataset/penthouse/intersection_problem.laser");    //floor_ceil.laser //intersection_problem

    vector<LaserPoints> segments;
    segments = PartitionLpByTag (segments_lp, SegmentNumberTag);

    vector <ObjectPoints> corners_vec ;
    vector <LineTopology> edges_vec;
    ObjectPoints corners;
    LineTopology edges;
    LineTopologies rectangles;
    int next_number; /// the next_number is the number after the last number in the corners file
    for (auto &segment : segments){
        /// derive databounds and derivetin for calculating the EnclosingRectangle
        DataBoundsLaser db = segment.DeriveDataBounds (0);
        segment.DeriveTIN ();
        /// EnclosingRectangle function handles the numbering of the corners for many segments
        /// corners and edges of the rectangle are the output of the function
        segment.EnclosingRectangle(0.2, corners, edges);
        /// get the numbering of corners for updating the Z value of the corners
        if (corners.empty()) next_number = 4; /// later next_number-4 =0
        else next_number = (corners.end() - 1)->Number() + 1;
        /// add z values to the 4 new corners
        for (int i=next_number-4; i < next_number ; i++){
            corners[i].Z() = (db.Maximum().GetZ() + db.Minimum ().GetZ ()) / 2;
        }
*//*        edges.push_back(PointNumber(next_number)); // close the polygon // clockwise
        edges.MakeClockWise(corners);*//*
        edges.SetAttribute (BuildingPartNumberTag, 1);
        rectangles.push_back (edges);

        corners_vec.push_back (corners);
        edges_vec.push_back (edges);
    }
    corners.Write("E:/Laser_data/ETH_dataset/penthouse/out/min_rectangles.objpts");
    rectangles.Write("E:/Laser_data/ETH_dataset/penthouse/out/min_rectangles.top", false);

    vector <ObjectPoints> new_corners_vec ;
    vector <LineTopology> new_edges_vec;
    ObjectPoints scaled_corners;
    LineTopologies new_rectangles;
    for (int i =0; i < corners_vec.size (); i++){

        scaled_corners = ScaleRectangle (corners_vec[i], edges_vec[i], 0.98);
        edges_vec[i].SetAttribute (BuildingPartNumberTag, 3);
        new_rectangles.push_back (edges_vec[i]);

        new_corners_vec.push_back (scaled_corners);
        new_edges_vec.push_back (edges_vec[i]);
    }*/
/*    new_corners_vec[0].Write("E:/Laser_data/ETH_dataset/penthouse/out/scaled_rectangles0.objpts");
    LineTopologies new_rectangles0; new_rectangles0.push_back (new_edges_vec[0]);
    new_rectangles0.Write("E:/Laser_data/ETH_dataset/penthouse/out/scaled_rectangles0.top", false);

    new_corners_vec[1].Write("E:/Laser_data/ETH_dataset/penthouse/out/scaled_rectangles1.objpts");
    LineTopologies new_rectangles1; new_rectangles1.push_back (new_edges_vec[1]);
    new_rectangles1.Write("E:/Laser_data/ETH_dataset/penthouse/out/scaled_rectangles1.top", false);

    new_corners_vec[2].Write("E:/Laser_data/ETH_dataset/penthouse/out/scaled_rectangles2.objpts");
    LineTopologies new_rectangles2; new_rectangles2.push_back (new_edges_vec[2]);
    new_rectangles2.Write("E:/Laser_data/ETH_dataset/penthouse/out/scaled_rectangles2.top", false);

    new_corners_vec[3].Write("E:/Laser_data/ETH_dataset/penthouse/out/scaled_rectangles3.objpts");
    LineTopologies new_rectangles3; new_rectangles3.push_back (new_edges_vec[3]);
    new_rectangles3.Write("E:/Laser_data/ETH_dataset/penthouse/out/scaled_rectangles3.top", false);

    new_corners_vec[4].Write("E:/Laser_data/ETH_dataset/penthouse/out/scaled_rectangles4.objpts");
    LineTopologies new_rectangles4; new_rectangles4.push_back (new_edges_vec[4]);
    new_rectangles4.Write("E:/Laser_data/ETH_dataset/penthouse/out/scaled_rectangles4.top", false);*/

    //scaled_corners.Write("E:/Laser_data/ETH_dataset/penthouse/out/scaled_rectangles.objpts");
    /// topology file is the same as before scale
    //new_rectangles.Write("E:/Laser_data/ETH_dataset/penthouse/out/scaled_rectangles.top", false);

    /*
     * test boost geometry for making polygons from objectpoints and intersect them
     * */
    //ObjectPoints corners1, corners2;
    //std::deque<boost_polygon> poly_out;
    //poly_out = intersect_polygons (corners_vec[0], corners_vec[1]);

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


    /*
     * function planefittingresidual test
     * */
    //LaserPoints lp;
    //lp.Read("D:/test/reflection_removal/reflection_data/wall_glass_reflection_balcon_2.laser");   //wall_reflection_balcon_1.laser"
    //lp.Read("D:/test/indoor_reconstruction/final_segmentation.laser");
    //root = (char*) "D:/test/indoor_reconstruction/";
    //planefittingresidual_test (lp, root);


    /*
     * test reflection removal and reflection reconstruction
     * */
    LaserPoints lp, trajectory;
    //lp.Read("D:/test/laserpoints_partitioning/3rdFloor_2mil_thinned.laser");
    //trajectory.Read("D:/test/laserpoints_partitioning/trajectory_balcon.laser");

    char *out;
    out = (char*) "D:/test/laserpoints_partitioning/out/";
    PartitionByTime_test_function (lp, trajectory, out, true);

/*   LaserPoints glass_lp, reflection_lp, trajectory_lp;
    glass_lp.Read("D:/test/reflection_removal/reflection_data/glass.laser");
    reflection_lp.Read("D:/test/reflection_removal/reflection_data/reflection.laser");
    trajectory_lp.Read("D:/test/reflection_removal/reflection_data/traj_wall_reflection_balcon_1.laser");
    out = (char*) "D:/test/reflection_removal/reflection_data/";*/
    //mirror_PointsToPlane (glass_lp, reflection_lp);

    /// visulize the reflection lines for few points
    //mirror_PointsToPlane_test_function (glass_lp, reflection_lp, trajectory_lp, out);


    std::cout << "Press ENTER to continue...";
    std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
}