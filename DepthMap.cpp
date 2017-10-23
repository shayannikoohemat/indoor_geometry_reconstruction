//
// Created by NikoohematS on 3-4-2017.
//

#include <ctime>
#include <iostream>
#include <LaserPoints.h>
/*
 * This code is not reliable in generating a correct depth map. Using it is on your own risk.
 * */

void DepthMap(LaserPoints trajectorypoints, LaserPoints laserpoints) {

    std::clock_t start;
    double duration;
    start = std::clock();

    bool verbose = 0;
    bool sort_trajectory = false;

    /// read laser points
    char *laserFile;
    laserFile = (char *) "D:\\test\\indoor_reconstruction\\depthmap\\reflected_balcony.laser";
    laserpoints.Read(laserFile);
    printf(" input laser points size:  %d \n ", laserpoints.size());

/*    if (!laserpoints.Read(laserFile)) {
        printf("Error reading laser points from file %s\n", laserFile);
        exit(0);
    }*/

    /// read trajectory points (scanner positions)
    char *trajFile;
    trajFile = (char *) "D:\\test\\indoor_reconstruction\\depthmap\\trajectory3.laser";
    trajectorypoints.Read(trajFile);
    printf(" input trajectory points size:  %d \n ", trajectorypoints.size());

/*    if (!trajectorypoints.Read(trajFile)) {
        printf("Error reading trajectory points from file %s\n", trajFile);
        exit(0);
    }*/

    /// sort trajectory points based on timetag
/*    if (sort_trajectory){
        std::sort (trajectorypoints.begin(), trajectorypoints.end(), comparison);
    }*/

    /// generate a vector of sorted traj_TimeTag
    std::vector<double> trajectory_timetags_v;

    LaserPoints sorted_traj_points, scannerpositions;
    sorted_traj_points = trajectorypoints;

    for (int i = 0; i < sorted_traj_points.size(); i++) {
        double current_timetag;
        current_timetag = sorted_traj_points[i].DoubleAttribute(TimeTag);
        trajectory_timetags_v.push_back(current_timetag);
    }

    LaserPoints::iterator point_it;
    int index1 = 0;

    for (point_it = laserpoints.begin(); point_it != laserpoints.end(); point_it++) {

        if (verbose) printf("Point TIME_TAG: %lf \n ", point_it->DoubleAttribute(TimeTag)); // debugger
        index1++;
        printf("points counter: %d / %d\r", index1, laserpoints.size());

        double p_timeTag;
        p_timeTag = point_it->DoubleAttribute(TimeTag);

        /// lower_bound returns the lower bound iterator of found time_tag
        std::vector<double>::iterator lower_traj_it;
        lower_traj_it = std::lower_bound(trajectory_timetags_v.begin(), trajectory_timetags_v.end(), p_timeTag);
        long long int scanner_position_index;
        scanner_position_index = lower_traj_it - trajectory_timetags_v.begin() - 1;
        LaserPoint scanner_position;
        //scanner_position = Laserpoint2ScanningPosition(*point_it, trajpoints, trajectory_timetags_v); // too expensive
        scanner_position = sorted_traj_points[scanner_position_index];

        if (verbose) { // to check if time_tag is correct
            double scanner_position_time;
            scanner_position_time = scanner_position.DoubleAttribute(TimeTag);
            printf("Traj TIME_TAG: %lf \n ", scanner_position_time); // debugger
        }
        /// if current_traj is not null then calculate the dist to the point
        if ((scanner_position.X() && scanner_position.Y()) != 0) {
            //scannerpositions.push_back(scanner_position);

            double dist;
            dist = fabs(scanner_position.Distance(Position3D(*point_it)));
            point_it->SetAttribute(ResidualTag, (float) dist);
            point_it->SetDoubleAttribute(ReflectanceTag, dist);
        }
    }
   laserpoints.Write("D:\\test\\indoor_reconstruction\\depthmap\\laserpoints_depthmap.laser", false);


    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"DepthMap processing time: "<< duration/60 << "m" << '\n';
}
