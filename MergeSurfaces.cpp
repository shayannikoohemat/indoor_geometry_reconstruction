//
// Created by NikoohematS on 5-11-2018.
//

#include <KNNFinder.h>
#include <Vector3D.h>
#include "LaserPoints.h"
#include "indoor_reconstruction.h"

struct merged_surfaces_and_planes{
    LaserPoints surface;
    Plane plane;
};

/// this function is similar to GenerateWallPatches.cpp but faster and for any arbitrary surface
void Mergesurfaces (const LaserPoints &segmented_lp, double max_dist_between_planes, double max_angle_between_normals,
                        double max_dist, int min_segment_size, char *root, double max_second_dist_between_planes){

    //TODO : add the distance of the parallel merged-segments as an attribute for later use, e.g. fitting boxes, wall thickness

    bool    merge_segments = true;
    bool    verbose = true;
    double  pi = 4.0 * atan(1.0);
    double  max_angle_radian;
    max_angle_radian = max_angle_between_normals * pi / 180;
    /// first make a list of segments and sort them by size
    vector <LaserPoints> segments_vec;
    segments_vec = PartitionLpByTag (segmented_lp, SegmentNumberTag);

    /// sort segments by size of the segment
    sort(segments_vec.begin (), segments_vec.end (), compare_lp_size );

    /// collect segment_numbers of size-sorted segments,
    /// fit a plane to each segment using PlaneFitting function and least square
    vector<int> segment_numbers;
    vector <Plane> planes_vec;
    for(auto &s : segments_vec){
        if(s.HasAttribute (SegmentNumberTag)){
            segment_numbers.push_back (s[0].SegmentNumber ());
            /// fit the plane to the segment
            planes_vec.push_back (s.FitPlane (s[0].SegmentNumber ()));
        }
    }

    vector <int> merged_segment_numbers;
    vector <std::pair<LaserPoints, Plane> > merged_surfaces_and_planes;
    LaserPoints merged_segments;
    /// compare the segments two by two
    for (int i=0; i<= segments_vec.size (); i++){
        if (segments_vec[i].size () < min_segment_size) continue; /// for finding distances of other segment to this one
        /// check if the segment is already merged go to the next one
        if(find(merged_segment_numbers.begin (), merged_segment_numbers.end (),
                segment_numbers[i]) != merged_segment_numbers.end ()) continue;
        /// add the current segment to merged segments
        merged_segments.AddPoints (segments_vec[i]);

        /// output after merge
        LaserPoints updated_segment;
        updated_segment.AddPoints (segments_vec[i]);
        Plane recalcualted_plane;

        Plane plane1;
        plane1.Recalculate ();
        plane1 = planes_vec[i];
        KNNFinder <LaserPoint> finder(segments_vec[i]); /// to find distances of other segment to this one
        for(int j=i+1; j<= segments_vec.size (); j++){
            if(segments_vec[j].size () < min_segment_size) continue; /// if segment is small go to the next one
            /// check if the segment is already merged go to the next one
            if(find(merged_segment_numbers.begin (), merged_segment_numbers.end (),
                    segment_numbers[j]) != merged_segment_numbers.end ()) continue;
            LaserPoints segment2;
            segment2 = segments_vec[j];
            Plane plane2;
            plane2 = planes_vec[j];

            /// borrowed from MergSurfaces fucntion in SurfaceGrowing.cpp
            /*            // Test if the slope angle is acceptable
            if (merge_segments) {
                angle = Angle(plane2->Normal(), Vector3D(0.0, 0.0, 1.0));
                if (angle > pi / 2.0) angle = pi - angle;
                if (angle > parameters.MergingMaxSlopeAngle())
                    merge_segments = false;
                if (debug && !merge_segments)
                    printf("Segment %lld has a large slope %.2f\n",
                           new_segment_number2,
                           Angle(plane2->Normal(), Vector3D(0.0, 0.0, 1.0)) * 180 / pi);
            }*/

            /// 1st consition: check if two normal vectors of segments are parallel
            /// Test angle between normal vectors
            double  angle;
            if (merge_segments) {
                angle = Angle(plane1.Normal(), plane2.Normal());
                if (angle > pi/2.0) angle -= pi;
                if (fabs(angle) > (max_angle_radian * 180 / pi)) merge_segments = false;
                if (verbose && !merge_segments)
                    printf("Segments %d and %d make large angle %.2f\n",
                           segment_numbers[i], segment_numbers[j],
                           Angle(plane1.Normal(), plane2.Normal()) * 180 / pi);
            } /// end of 1st condition (angle threshold)

            /// 2nd consition: check if two planes are within the threshold
            /// Test point to Plane distance
            if (merge_segments){
                double planes_distance;
                planes_distance = fabs(plane1.Distance(plane2.CentreOfGravity()));
                if (planes_distance > max_dist_between_planes) merge_segments = false;
                if(verbose && !merge_segments)
                    printf("plane %d and %d have large distance %.2f\n",
                           segment_numbers[i], segment_numbers[j], planes_distance);

                /// check if the planes distance is less than the second_distance threshold for later checking
                if (planes_distance < max_second_dist_between_planes){
                    /// we set the segment number of the first segment to the Component attribute of this one for later check
                    segment2.SetAttribute (ComponentNumberTag, segment_numbers[i]);
                }
            } /// end of 2nd condition (planes' distance)

            /// 3rd condition for merge: check if two segments are approximate
            /// check if the points of two segments are within a proximity (max_dist) of each other
            if (merge_segments){
                int count=0;
                merge_segments = false;
                vector <double> dists;
                dists = finder.FindDistance (segment2, 5, EPS_DEFAULT);
                double average_dist=1000000.0;
                double sum = 1000000.0;
                if (!dists.empty ()){
                    for (auto &d : dists)
                    {
                        sum +=d;
                        average_dist = sum / dists.size ();
                    }
                }
                if (fabs(average_dist) < max_dist) merge_segments = true;
/*                for (auto &p : segment2){
                    double dist_nbh;
                    dist_nbh = finder.FindDistance (p, 1, EPS_DEFAULT);
                    if (fabs(dist_nbh) < max_dist){
                        count++;
                    }
                    /// if we found 5 points of segment2 in the proximity of segment1 then we break the loop and
                    /// continue merging process as all the conditions are fulfilled
                    if (count > 5) {
                        merge_segments = true;
                        break;
                    }
                }*/
            } /// end of 3rd condition

            /// if we decided not to merge becasue of the 3rd condition,
            /// then the ComponentNumberTag for the segment2 is not useful anymore.
            if(!merge_segments) segment2.RemoveAttribute (ComponentNumberTag);

            /// now we merged two segments and recalculate the plane becasue all three conditions are fulfilled
            /// merging means points in segment2 get the segment number of segment1
            if (merge_segments){
                segment2.SetAttribute (SegmentNumberTag, segment_numbers[i]);
                merged_segments.AddPoints (segment2); /// add the merged segment (segment2) to the merged points
                merged_segment_numbers.push_back (segment_numbers[j]);
                updated_segment.AddPoints (segment2);
            }

            /// calculate the plane of the new updated segment
            if (merge_segments){
                /// calculate the new plane
                /// calculate


            } else {
                /// use the same standard plane for it

            }


        } /// end of second for (segment2)
        std::pair <LaserPoints, Plane > pair;
        pair = std::make_pair (updated_segment, recalcualted_plane);
        merged_surfaces_and_planes.push_back (pair);

    } /// end of first for (segment1)

    /// renumber merged_segments if necessary

}

