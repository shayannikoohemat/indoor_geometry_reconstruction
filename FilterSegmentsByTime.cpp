//
// Created by NikoohematS on 21-2-2017.
//

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <LaserPoints.h>
#include <KNNFinder.h>


vector<double> GetDoubleAttribute(LaserPoints lpoints, const LaserPointTag tag) {
    /// generate a vector of double tags
    std::vector<double> tags_vec;

    for (int i = 0; i < lpoints.size(); i++) {
        double value;
        if (lpoints[i].HasAttribute(tag)) {
            value = lpoints[i].DoubleAttribute(tag);
            tags_vec.push_back(value);
        }
    }
    return tags_vec;
}


bool comparison(int i, int j) { return (i < j); }

/* This function detect reflected from glass mls points by their time difference with closest trajectory-point time.
 * If there is a time difference more than threshold between the point and closest traj point and if
 * this happens for more than percentage of points in a segment, then that segment is labeled
 * as reflected segment.
 * */

 //@param: reflectedpoints_percentage: lower than 0.70 has the risk of losing real segments

void FilterSegments_ByTimeTag(LaserPoints lp, LaserPoints trajectory_lp, double timestamp_difference,
                               double max_dist_to_traj, double reflected_points_percentage) {

    std::clock_t start;
    double duration;
    start = std::clock();

    char *laserFile;
    //laserFile = (char *) "D://test//filterpoints//fb_crop_thinned_1cm_flcl_410k_noceiling.laser";
    //laserFile = (char*) "D://test//occlusion_test//3rdFloor_2mil_thinned_seg3_22cm_refined//FBr_3rdfloor_122k_nofloorceiling.laser";
    //laserFile = (char*) "D://test//filterpoints//fireBr_sub_1cm_3rdFloor_thinned_seg22cm_refinedsegments_noCeil_1mil.laser";
    laserFile = (char*) "D://test//filterpoints//3rdFloor_thinned_rk4_3600k_seg1012cm.laser";  //FB_sub1cm_seg12cm_695k_crop_refined1.laser
    lp.Read(laserFile);
    printf (" input point size:  %d \n ", lp.size());

/*    LaserPoints floorceiling_lp;
    char *fc_laserFile;
    fc_laserFile = (char *) "D://test//filterpoints//floor_ceiling.laser";
    floorceiling_lp.Read(fc_laserFile);*/
    /// Build KNN for points for later search
    //KNNFinder <LaserPoint> finder(floorceiling_lp);

    /// read trajectory points (scanner positions)
    char *trajFile;
    trajFile = (char *) "D://test//filterpoints//trajectory3.laser";
    //trajFile = (char*) "D://test//filterpoints//trajectory_crop.laser";
    trajectory_lp.Read(trajFile);
    /// Build KNN for points for later search
    KNNFinder<LaserPoint> finder_traj(trajectory_lp);

    vector<int> segment_numbers;
    vector<int>::iterator segment_it;

    segment_numbers = lp.AttributeValues(SegmentNumberTag);  // vector of segment numbers
    printf("Number of laser segments: %d\n", segment_numbers.size());
    if (segment_numbers.size() == 0) printf("Error: Points are not segmented.\n");

    //timestamp_difference =130.0;
    //reflected_points_percentage = 0.60;
    double PI;
    PI = 3.14159;
    double flat_angle, vertical_angle;
    flat_angle = 10.0;
    vertical_angle = 15.0;

    double z_floor, z_ceiling;
    z_floor = -0.75;
    z_ceiling = 1.60;

    LaserPoints modified_points, reflected_points;
    lp.SetAttribute(LabelTag, 0);
    modified_points = lp;
    for (segment_it = segment_numbers.begin(); segment_it != segment_numbers.end(); segment_it++) {
        /// selecting points by segment and saving in segment_lpoints
        LaserPoints segment_lpoints;
        segment_lpoints = lp.SelectTagValue(SegmentNumberTag, *segment_it);

        /*       vector<double>  segment_timetags;
               //segment_timetags = AttributeValuesDouble(segment_lpoints, TimeTag);  // Too slow
               segment_timetags = GetDoubleAttribute(segment_lpoints, TimeTag);
               sort(segment_timetags.begin(), segment_timetags.end(), comparison); // sort ascending
               printf ("Number of tags: %d\n", segment_timetags.size());

               double timetag_max, timetag_min;
               timetag_min = *segment_timetags.begin();
               timetag_max = *segment_timetags.end();

               DataBoundsLaser db;
               db = segment_lpoints.DeriveDataBounds(0);

               LaserPoints croped_floorceil;
               floorceiling_lp.Select(croped_floorceil, db);
               croped_floorceil.Write("D://test//filterpoints//croped_lp2D.laser", false);*/

        Plane plane;
        plane = lp.FitPlane(*segment_it, *segment_it, SegmentNumberTag);
        LaserPoints floorpoints_underwallsegment, ceilingpoints_abovewallsegment;
        /// get Vertical planes
        if (plane.IsVertical(vertical_angle * PI / 180)) { //  radian
            //printf("Number of segment'points: %d\n", segment_lpoints.size());
            LaserPoints::iterator segpoint_it;
            int reflectedpointF_count = 1;
            int reflectedpointC_count = 1;
            int reflectedpointTraj_count = 1;
            int index1 = 1;
            for (segpoint_it = segment_lpoints.begin(); segpoint_it != segment_lpoints.end(); segpoint_it++) {
                index1++;
                double segmentpoint_timetag;
                segmentpoint_timetag = segpoint_it->DoubleAttribute(TimeTag);

                /// check the segment point timetag with trajectory timetag
                int traj_index;
                double traj_dist;
                //traj_index = finder_traj.FindIndex(*segpoint_it, 1, EPS_DEFAULT);
                finder_traj.FindIndexAndDistance(*segpoint_it, traj_index, traj_dist, 1, EPS_DEFAULT);
                if (traj_dist < max_dist_to_traj){  /// this is to avoid getting finding wrong trajectories
                    double traj_timetag;
                    traj_timetag = trajectory_lp[traj_index].DoubleAttribute(TimeTag);

                    /// check the segment point timetag with floor and ceiling timetag
/*                LaserPoint pfloor, pceiling;
                pfloor = *segpoint_it;
                pfloor.SetZ(z_floor);

                int floorpoint_index;
                floorpoint_index =  finder.FindIndex (pfloor, 1, EPS_DEFAULT);
                double floorpoint_timetag;
                floorpoint_timetag = floorceiling_lp[floorpoint_index].DoubleAttribute(TimeTag);
                floorpoints_underwallsegment.push_back(floorceiling_lp[floorpoint_index]); // for debug

                pceiling = *segpoint_it;
                pceiling.SetZ(z_ceiling);

                int ceiling_index;
                ceiling_index = finder.FindIndex(pceiling, 1, EPS_DEFAULT);
                double ceilingpoint_timetag;
                ceilingpoint_timetag = floorceiling_lp[ceiling_index].DoubleAttribute(TimeTag);

                if (abs(segmentpoint_timetag - floorpoint_timetag) > 200){
                    reflectedpointF_count++;
                };

                if (abs(segmentpoint_timetag - ceilingpoint_timetag) > 200){
                    reflectedpointC_count++;
                }; */

                    if (abs(segmentpoint_timetag - traj_timetag) > timestamp_difference) {
                        reflectedpointTraj_count++;
                    };
                }
            }
            double reflectedpointsF_percentage, reflectedpointsC_percentage, reflectedpointTraj_percentage;
            reflectedpointsF_percentage = reflectedpointF_count / (double) index1;
            reflectedpointsC_percentage = reflectedpointC_count / (double) index1;
            reflectedpointTraj_percentage = reflectedpointTraj_count / (double) index1;
            if (reflectedpointTraj_percentage > reflected_points_percentage) {
                lp.ConditionalReTag(0, 3, LabelTag, *segment_it, SegmentNumberTag); // label 7 has the same color as 0
                modified_points.RemoveTaggedPoints(*segment_it, SegmentNumberTag);
            }
        }
    }

    reflected_points = lp.SelectTagValue(LabelTag, 3);

    lp.Write("D://test//filterpoints//fb_relabeled.laser", false);
    modified_points.Write("D://test//filterpoints//fb_modified_points.laser", false);
    reflected_points.Write("D://test//filterpoints//fb_reflected_points.laser", false);

    duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;
    std::cout << "total processing time: " << duration / 60 << "m" << '\n';
}

