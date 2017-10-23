#include <iostream>
#include "InlineArguments.h"
#include <cstdlib>
#include <limits>
#include <LaserPoints.h>
#include "Buffers.h"
#include "indoor_reconstruction.h"

using namespace std;

void PrintUsage()
{
    printf("Usage: indoor_reconstruction -input_laser <input laserpoints , foo.laser>\n");
    printf("                  -minsizesegment <minimum segment size e.g. 200>\n");
}

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
    //GenerateWallPatches(lpoints, 0.70, 20.0, 0.5);

    char* file_lp;
    //file_lp = (char*) "D://test//indoorgraph//data//fireBr_Cloud_16mil_sub_1cm_3rdFloor_2mil_thinned_seg1_12cm.laser"; // gym_raw_thinned_455k.laser //
    //file_lp = (char*) "D://test//indoor_reconstruction//data//FB_sub1cm_seg12cm_refined_1770k_reflectremoved.laser";
    //file_lp = (char*) "E://BR_data//ZebR//cc_sub2mil_basement_crop_seg10cm.laser";
    file_lp  = (char *) "E://BR_data//Diemen//process//block3_seg05cm_1-7mil_seg10cm_label0.laser" ; //tworooms_labeled0.laser"
    lpoints.Read(file_lp);
    //traj_lp.Read("E://BR_data//ZebR//traj_basement_crop.laser");
    traj_lp.Read("E://BR_data//Diemen//process//traj_diemen1_s_rec.laser");
   surfpoints.Read("E://BR_data//Diemen//process//out//outputlaser.laser");

    /*remove points with label -1 from backpack system*/
/*    LaserPoints lp_labeled0;
    for (auto p : lpoints){
        if(p.Attribute(LabelTag) != -1) {
            p.SetAttribute(LabelTag, 0);
            lp_labeled0.push_back(p);
        }
    }
    lp_labeled0.Write("E://BR_data//Diemen//process//block3_seg05cm_1-7mil_seg10cm_label0.laser", false);*/

    //indoorTopology(file_lp, 10, 0);
    Buffers bfs;
    occlusion_test(lpoints, surfpoints, traj_lp, bfs , 0.12);

    //LaserPoints refined_lp;
    //refined_lp = segment_refinement(lpoints, 2, 0.50);
    //refined_lp.Write("D://test//buffer_Kada//FB_sub1cm_seg8cm_665k_refined2_reflectremoved.laser", false);

    //LaserPoints traj_lp, lp;
    //DepthMap(traj_lp, lp);

    int label;
    //WallAccuracy(label);
    //RemoveDuplicatePoints(lpoints, lp);

    std::cout << "Press ENTER to continue...";
    std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
}