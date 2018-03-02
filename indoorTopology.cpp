//
// Created by NikoohematS on 11/4/2016.
//

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <LaserPoints.h>
#include <map>
#include "Buffer.h"
#include "Buffers.h"
#include "indoor_reconstruction.h"


enum Color { DARKBLUE = 1, DARKGREEN, DARKTEAL, DARKRED, DARKPINK, DARKYELLOW, GRAY ,
                DARKGRAY, BLUE, GREEN, TEAL, RED, PINK, YELLOW, WHITE };
void SetColor(enum Color);

///sort descending
bool compare (int i,int j) { return (i>j); };

//LaserPoints segment_refinement(LaserPoints lp, int min_segment_size, double maxdistanceInComponent);

//void occlusion_test(LaserPoints laserpoints, LaserPoints surfacepoints,
//                    LaserPoints trajpoints, Buffers segment_buffers, double closeness_to_surface_threshold);

void indoorTopology(char* laserFile, int minsizesegment, char* root, bool verbose){

    std::clock_t start;
    double duration;
    start = std::clock();

    char str_root[500];
    //char *root = (char*) "D://test//publication//TUM_testresult//";
    //char *root = (char*) "D://test//indoor_reconstruction//3rdfloor_601040_accuracycehck//";
    //auto *root = (char*) "E://BR_data//Diemen//process//out//";
    strcpy (str_root,root); // initialize the str_root with root string

    verbose = false;
    bool do_segmentation = false;
    bool do_segment_refinement = false;
    bool generate_wallpatches = false; // generate wallpatches (M. Kada's approach) includes segment refinement
    bool do_occlusion_test= false;
    bool crop_walls_abovefloor_and_belowceiling =false; /// for firebrigade dataset or for glass floor and ceiling cases

    /// read laser points
    LaserPoints laserpoints;
    //// FB_sub1cm_seg12cm_refined_1770k_reflectremoved.laser
    //laserFile = (char*) "D://test//indoor_reconstruction//data//3rdfloor_modified_laserpoints_occlusiontestoutput.laser";
    //laserFile = (char*) "D://test//indoor_reconstruction//test_samples//crop_merge_seg8cm_withCeiling.laser";
    //laserFile = (char*) "D://test//indoor_reconstruction//3rdfloor_601040_accuracycehck//source//3rdFloor_thinned_rk4_3600k_seg1012cm.laser";
    //laserFile = (char*) "E://BR_data//ZebR//cc_sub2mil_basement_crop_seg10cm.laser";
    //laserFile = (char*) "D://test//publication//TUM_entrance.laser";
    laserpoints.Read(laserFile);
    SetColor(BLUE);
    printf (" input point size:  %d \n ", laserpoints.size());
    SetColor(GRAY); /// set color back to default console

    if (!laserpoints.Read(laserFile)) {
        printf("Error reading laser points from file %s\n", laserFile);
        exit(0);
    }

    SegmentationParameters *seg_parameter;
    seg_parameter = new SegmentationParameters;
    /// segmenting laserpoints
    if (do_segmentation){
        seg_parameter -> MaxDistanceInComponent()  = 0.3;
        seg_parameter -> SeedNeighbourhoodRadius() = 1.0;
        seg_parameter -> MaxDistanceSeedPlane()    = 0.20; // MaX Distance to the Plane
        seg_parameter -> GrowingRadius()           = 1.0;
        seg_parameter -> MaxDistanceSurface()      = 0.20;

        /// if segmentation crashes set the compatibility of generated exe to windows7
        printf("segmentation process... \n ");
        laserpoints.SurfaceGrowing(*seg_parameter);
    }

    /// collect points without segment nr
    LaserPoints notsegmented_points;
    if (verbose){
        LaserPoints::iterator point_it;
        for (point_it = laserpoints.begin(); point_it != laserpoints.end(); point_it++){
            if (!point_it->HasAttribute(SegmentNumberTag)){
                notsegmented_points.push_back(*point_it);
            }
        }
    }

    if (do_segment_refinement){
        printf("Processing segments larger than %d points.\n", minsizesegment);
        /// resegmenting laserpoints by removing longedges in the TIN and removing deformed segments
        LaserPoints refined_lp;
        refined_lp = segment_refinement(laserpoints, 2, 0.30);
        /// renew laserpoints with the result of segment_refinement
        laserpoints = refined_lp;
        strcpy (str_root,root);
        laserpoints.Write(strcat(str_root,"segment_refine_relabeled.laser"), false);
    }

    delete seg_parameter;

    /// default label ==0
    laserpoints.SetAttribute(LabelTag, 0);
    /// collecting list of point_numbers and planes for each segment
    Planes                  planes;
    PointNumberLists        point_lists;
    double                  PI;
    PI                      = 3.14159;
    double                  flat_angle, vertical_angle;
    flat_angle              = 50.01; //10.0 ;
    vertical_angle          = 39.99; //10.0;
    double                  max_intersection_dist;
    max_intersection_dist   = 0.10;


    std::map<int, DataBoundsLaser> horizon_segmens_databounds_map;
    std::map<int, int>      horizon_segment_sizep_map;
    std::map<int, double>   horizon_segmens_centroidheight_map;
    vector<int>             segment_numbers;
    vector<LaserPoints>     horizon_segments_vec;
    vector<int>::iterator   segment_it;
    LaserPoints             small_segments;
    Buffers                 segment_buffers;

    /*
 *  *** Important function for generating bigger segments as wall patch that is called buffer ***
 * */
    if(generate_wallpatches){
        printf ("\n Generate wall-patch segments ... (Martin-Kada's generalisation approach) \n");
        segment_buffers = GenerateWallPatches(laserpoints, 0.70, 20.0, 0.40, root);

        LaserPoints buffer_points;
        buffer_points = segment_buffers.Buffers_laserpoints();
        buffer_points.SetAttribute(LabelTag, 0);
        laserpoints = buffer_points; // this initializes pointnumbers (because of pointnumbers bug)

        LaserPoints relabeled_buffer_points;  // relabeled to vertical & horizontal
        for (int i=0; i< segment_buffers.size(); i++){
            LaserPoints bufflp;
            bufflp = segment_buffers[i].BuffLaserPoints();
            //bufflp[i].GetPointNumber(); // bug

            if (bufflp.size() > minsizesegment){
                Plane plane;
                plane = segment_buffers[i].BuffPlane();

                int segment_nr;
                segment_nr = segment_buffers[i].BuffSegmentNumber();

                /// label laserpoints with horizontal planes as 1
                if (plane.IsHorizontal(flat_angle * PI / 180)){ // radian
                    bufflp.ConditionalReTag(0, 1, LabelTag, segment_nr , SegmentNumberTag); // Horizontal 1
                }

                /// label laserpoints with vertical planes as 2
                if (plane.IsVertical(vertical_angle * PI / 180)) { //  radian
                    bufflp.ConditionalReTag(0, 2, LabelTag, segment_nr, SegmentNumberTag); // Vertical 2
                }

                planes.push_back(plane); // this plane belongs to the buffer and is NOT calculated from bufflaserpoints

                PointNumberList point_list;
                /// Collect a vector of point numbers with SegmentNumberTag value
                point_list = laserpoints.TaggedPointNumberList(SegmentNumberTag, segment_nr);
                //point_list.size(); //debug
                point_lists.push_back(point_list);
            } else{
                small_segments.AddPoints(bufflp);
            }

            relabeled_buffer_points.AddPoints(bufflp);
        }
        strcpy (str_root,root);
        relabeled_buffer_points.Write(strcat(str_root,"buffer_segmentation.laser"), false);
        laserpoints = relabeled_buffer_points; // contains all buffers after renumbering their segmentation
    }

    FILE *segments_infofile;
    strcpy (str_root,root);
    segments_infofile = fopen(strcat(str_root, "segments_infofile.txt"),"w");
    fprintf(segments_infofile, "segment_nr, segment_pointsize, centroid_height, segment_angle \n");

    /// this file is to analysis almost_horizontal segments to estimate floor and ceiling height
    FILE *almost_horizon_infofile;
    strcpy (str_root,root);
    almost_horizon_infofile = fopen(strcat(str_root, "almost_horizontal_segments_infofile.txt"),"w");
    fprintf(almost_horizon_infofile, "segment_nr, segment_pointsize, centroid_height, segment_angle \n");
    /*
* if we didn't calculate wallpatches then we use original segments for wall detection
 *  and if we dont have boost library
* */
    if (!generate_wallpatches){
        segment_numbers = laserpoints.AttributeValues(SegmentNumberTag);  // vector of segment numbers
        std::sort(segment_numbers.begin(), segment_numbers.end());
        printf ("Number of laser segments: %d\n", segment_numbers.size());

        /// segment processing and fitting planes and labeling to 1 (horiz) and 2 (vertical)
        for(segment_it = segment_numbers.begin(); segment_it != segment_numbers.end(); segment_it++){
            //segment_lpoints.ErasePoints();
            /// selecting points by segment and saving in segment_lpoints
            LaserPoints     segment_lpoints;
            segment_lpoints = laserpoints.SelectTagValue(SegmentNumberTag, *segment_it);
            Plane   plane;
            if(segment_lpoints.size() > minsizesegment) {

                plane = laserpoints.FitPlane (*segment_it, *segment_it, SegmentNumberTag);
                //plane = segment_lpoints.FitPlane (segment_lpoints[0].SegmentNumber ());
                //plane.Number () = segment_lpoints[0].SegmentNumber ();  /// set the segment number as the plane number

                PointNumberList point_list;
                //point_list.erase(point_list.begin(), point_list.end());  ???
                /// Collect a vector of point numbers with SegmentNumberTag value, used for polygon
                point_list = laserpoints.TaggedPointNumberList (SegmentNumberTag, *segment_it);
                point_lists.push_back (point_list);

                /* get the distribution of segment's size, plane's angle and segment's center height
                 this information is useful for extraction of the ceiling and floor height candidate
                 and getting the angle distribution of the segments */
                double segment_angle=0.0;
                ObjectPoint segment_centroid;
                if (verbose) {
                    segment_angle = Angle (plane.Normal (), Vector3D (0, 0, 1)) * 180.0 / PI;
                    if (segment_angle > 90.0) segment_angle = segment_angle - 90.0;   //// not sure if this is correct
                }
                segment_centroid = laserpoints.Centroid (point_list, *segment_it);
                fprintf (segments_infofile, "%d,%d,%.2f,%.2f \n",
                         *segment_it, segment_lpoints.size (), segment_centroid.Z (), segment_angle);

                /// label laserpoints with horizontal planes as 1
                if (plane.IsHorizontal(flat_angle * PI / 180)) { // 0.17 radian
                    laserpoints.ConditionalReTag (0, 1, LabelTag, *segment_it, SegmentNumberTag); // Horizontal 1
                    /// make an info file of almost horizontal segments
                    fprintf(almost_horizon_infofile, "%d,%d,%.2f,%.2f \n",
                            *segment_it, segment_lpoints.size(), segment_centroid.Z (), segment_angle);
                    horizon_segmens_centroidheight_map.insert ( std::pair<int, double>(*segment_it, segment_centroid.Z ()));
                    horizon_segment_sizep_map.insert(std::pair<int, int> (*segment_it, segment_lpoints.size ()));
                    horizon_segments_vec.push_back (segment_lpoints);
                }

                /// label laserpoints with vertical planes as 2
                if (plane.IsVertical(vertical_angle * PI / 180)) //  radian
                    laserpoints.ConditionalReTag(0, 2, LabelTag, *segment_it, SegmentNumberTag); // Vertical 2

                planes.push_back(plane);

            } else {
                small_segments.AddPoints(segment_lpoints);
            }
        } /// end of segment plane fitting
    }

    LaserPoints relabeled_vert_horiz;
    relabeled_vert_horiz = laserpoints;
    strcpy (str_root,root);
    relabeled_vert_horiz.Write(strcat(str_root, "relabeled_vert_horiz.laser"), false);

    LaserPoints almost_horizontal_segments;
    almost_horizontal_segments = relabeled_vert_horiz.SelectTagValue(LabelTag, 1);
    strcpy (str_root,root);
    almost_horizontal_segments.Write(strcat(str_root, "almost_horizontal_segments.laser"), false);  /// debug

    /* estimate the ceiling and floor height by choosing dominant almost-horizontal segments
     *  and check their height distribution *** this is just a rough estimation
     * */
    /// sort vector of horizontal_segments by size
    sort(horizon_segments_vec.begin (), horizon_segments_vec.end (), compare_lp_size); // descending
    /// select 10% of big horizontal segments to check their height distribution
    int count_subset_horizon_segments; /// count of 10% of horizontal segments
    count_subset_horizon_segments = static_cast<int> (horizon_segments_vec.size () * 0.20);
    printf(" 10 percent count of horizontal segments are : %d \n", count_subset_horizon_segments); //debug
    /// this file is for analysiing the almost_horizontal segments with the bigger size, to estimate the floor and ceiling height
    FILE *almost_horizon_subset_infofile;
    strcpy (str_root,root);
    almost_horizon_subset_infofile = fopen(strcat(str_root, "almost_horizon_subset_infofile.txt"),"w");
    fprintf(almost_horizon_subset_infofile, "segment_nr, segment_pointsize, centroid_height \n");
    /// collect horizontal segments centroid height and store in an info_file
    /// just for double check
    LaserPoints horizon_segments_subset; /// 10% of big horizontal segments
    for( int i=0; i < count_subset_horizon_segments ; i++){
        if(verbose){
            int horizon_segment_nr;
            horizon_segment_nr = horizon_segments_vec[i][0].SegmentNumber ();
            double segment_centroid_height;
            segment_centroid_height = horizon_segmens_centroidheight_map.find (horizon_segment_nr)->second;
            /// print the information in a file
            fprintf(almost_horizon_subset_infofile, "%d,%d,%.2f \n",
                    horizon_segment_nr, horizon_segments_vec[i].size(), segment_centroid_height);
        }
        horizon_segments_subset.AddPoints (horizon_segments_vec[i]);
    }

    /// store horizontal segments to the disk for double check
    strcpy (str_root,root);
    horizon_segments_subset.Write(strcat(str_root, "almost_horizon_segs_10%subset.laser"), false);  /// debug

    /* important estimation of floor and ceiling height */
    /// if the height of floor to the ceiling is less than 2 meters exit the program
    double maxima_z, minima_z;
    double ceiling_z, floor_z;
    minima_z = horizon_segments_subset.DeriveDataBounds (0).Minimum ().GetZ (); /// important floor estimation
    maxima_z = horizon_segments_subset.DeriveDataBounds (0).Maximum ().GetZ (); /// /// important ceiling estimation

    if(fabs(maxima_z - minima_z) < 2.0){
        printf ("Ceiling - Floor height = %.2f \n", fabs(maxima_z - minima_z));
        printf ("Error in the floor and ceiling height estimation, less than 2 meters!!! \n");
        exit (EXIT_FAILURE);
    } else {
        ceiling_z = maxima_z ;
        floor_z   = minima_z;
        printf("Ceiling Height estimation: %.2f \n", ceiling_z);
        printf("Floor Height estimation: %.2f \n", floor_z);
    }

    /* exclude almost horizontal segments that are far below the ceiling or far above the floor */
    double ceiling_threshold, floor_threshold;
    vector <int> candidate_ceil_segment_nr, candidate_floor_segment_nr;
    vector <LaserPoints> candidate_ceil_segment_vec, candidate_floor_segment_vec;
    vector<tuple <ObjectPoints,LineTopology>> min_rectangle_ceil;
    vector<tuple <ObjectPoints,LineTopology>> min_rectangle_floor;
    //vector <ObjectPoints> obbox_ceil_corners_vec, obbox_floor_corners_vec;
    //vector <LineTopology> obbox_ceil_edges_vec, obbox_floor_edges_vec;
    LaserPoints candidate_ceil_floor_segment_lp_byheight;
    ceiling_threshold = 1.0;
    floor_threshold   = 0.5;
    for (auto &segment : horizon_segments_vec){
        int horizon_segment_nr;
        horizon_segment_nr = segment[0].SegmentNumber ();
        double segment_centroid_height;
        segment_centroid_height = horizon_segmens_centroidheight_map.find (horizon_segment_nr)->second;
        ObjectPoints mbr_corners;
        LineTopology mbr_edges;
        DataBoundsLaser db = segment.DeriveDataBounds (0); /// is required for EnclosingRectangle
        /// collect candidate ceiliengs and their minimum enclosing rectangle
        if(fabs(ceiling_z - segment_centroid_height) <= ceiling_threshold){
            candidate_ceil_segment_nr.push_back(horizon_segment_nr);
            candidate_ceil_floor_segment_lp_byheight.AddPoints (segment);
            candidate_ceil_segment_vec.push_back (segment);

            segment.DeriveTIN (); /// is required for EnclosingRectangle
            segment.EnclosingRectangle (0.10, mbr_corners, mbr_edges);
            min_rectangle_ceil.push_back (make_tuple (mbr_corners, mbr_edges));
            //obbox_ceil_corners_vec.push_back (mbr_corners);
            //obbox_ceil_edges_vec.push_back (mbr_edges);
            //printf ("candidate ceiling number: %d \n", horizon_segment_nr);
        }
        /// collect candidate floors and their minimum enclosing rectangle
        if(fabs(floor_z - segment_centroid_height) <= floor_threshold){
            candidate_floor_segment_nr.push_back(horizon_segment_nr);
            candidate_ceil_floor_segment_lp_byheight.AddPoints (segment);
            candidate_floor_segment_vec.push_back (segment);
            segment.DeriveTIN (); /// is required for EnclosingRectangle
            segment.EnclosingRectangle (0.10, mbr_corners, mbr_edges);
            min_rectangle_floor.push_back (make_tuple (mbr_corners, mbr_edges));
            //obbox_floor_corners_vec.push_back (mbr_corners);
            //obbox_floor_edges_vec.push_back (mbr_edges);
            //printf ("candidate floor number: %d \n", horizon_segment_nr);
        }
    }

    /// store candidate floor and ceiling for double check
    strcpy (str_root,root);
    candidate_ceil_floor_segment_lp_byheight.Write(strcat(str_root, "candidate_ceil_floor_segment_lp_byheight.laser"), false);  /// debug

    /* loop through almost horizontal segments and check if their oriented bbox overlap, if yes the lower segments
     * would be excluded from the list of candidate ceilings
     * NOTE: there is no check if the minimum bounding rectangle exists and is correct
     */
    vector<int> not_ceiling_segments_nr;
    for (int i=0; i < candidate_ceil_segment_vec.size (); i++) {
        /// get the bbox bounds
        DataBoundsLaser db1_original, db1;
        db1_original = candidate_ceil_segment_vec[i].DeriveDataBounds (0);
        db1 = ShrinkDBounds (db1_original, 0.05);
        /// get the segment number
        int ceil1_segment_nr;
        ceil1_segment_nr = candidate_ceil_segment_vec[i][0].SegmentNumber ();
        /// check if the segment is already excluded or not
        std::vector<int>::iterator it1;
        it1 = find(not_ceiling_segments_nr.begin (), not_ceiling_segments_nr.end (), ceil1_segment_nr);
        if(it1 !=not_ceiling_segments_nr.end ()) continue; //if it1 found in the vector continue with the next segment
        printf("ceiling candidate1: %d \n", ceil1_segment_nr); // debug
        /// get the centroid height
        double ceil1_centroid_height;
        ceil1_centroid_height = horizon_segmens_centroidheight_map.find (ceil1_segment_nr)->second;
        /// get the next segment in the list
        for (int j=i+1; j < candidate_ceil_segment_vec.size (); j++){
            /// get the bbox bounds
            DataBoundsLaser db2_original, db2;
            db2_original = candidate_ceil_segment_vec[j].DeriveDataBounds (0);
            db2 = ShrinkDBounds (db2_original, 0.05);
            ///get the segment number
            int ceil2_segment_nr;
            ceil2_segment_nr = candidate_ceil_segment_vec[j][0].SegmentNumber ();
            /// check if it is already excluded or not
            std::vector<int>::iterator it2;
            it2 = find(not_ceiling_segments_nr.begin (), not_ceiling_segments_nr.end (), ceil1_segment_nr);
            if(it2 !=not_ceiling_segments_nr.end ()) continue; // if it2 fount in the vector continue with the next segment
            /// get the segment centroid height
            double ceil2_centroid_height;
            ceil2_centroid_height = horizon_segmens_centroidheight_map.find (ceil2_segment_nr)->second;
            /// check if they have overlap, if yes then exclude the lower segment from the ceiling candidates
            if(OverlapXY (db1, db2)){
                if((ceil2_centroid_height < ceil1_centroid_height) &&
                        (candidate_ceil_segment_vec[j].size () < candidate_ceil_segment_vec[i].size ())){
                    not_ceiling_segments_nr.push_back (ceil2_segment_nr);
                    printf("ceiling candidate2 excluded: %d \n", ceil2_segment_nr);
                }
                if((ceil1_centroid_height < ceil2_centroid_height) &&
                        (candidate_ceil_segment_vec[i].size () < candidate_ceil_segment_vec[j].size ())){
                    not_ceiling_segments_nr.push_back (ceil1_segment_nr);
                    printf("ceiling candidate2 excluded: %d \n", ceil2_segment_nr);
                }
            }
        }
    }

    /// collect final candidate ceiling segments (for double check)
    vector <LaserPoints> final_ceil_segment_vec;
    LaserPoints final_ceil_segment_lp;
    for (auto &ceil : candidate_ceil_segment_vec){

        std::vector<int>::iterator it;
        it = find(not_ceiling_segments_nr.begin (), not_ceiling_segments_nr.end (), ceil[0].SegmentNumber ());
        if(it != not_ceiling_segments_nr.end ()){
            /// do nothing
        }else{
            final_ceil_segment_vec.push_back (ceil);
            final_ceil_segment_lp.AddPoints (ceil);
        }
    }

    strcpy (str_root,root);
    final_ceil_segment_lp.Write(strcat(str_root, "final_ceil_segment_lp.laser"), false);  /// debug

    LaserPoints almost_horizontalpoints_tmp;
    //LaserPoints dominanat_seg1_lp, dominanat_seg2_lp;
    PointNumberList point_numlist1, point_numlist2;


    int count1, count2;
    int dominanat_seg_no1, dominanat_seg_no2;
    dominanat_seg_no1 = almost_horizontalpoints_tmp.MostFrequentAttributeValue(SegmentNumberTag, count1);
    //dominanat_seg1_lp.AddTaggedPoints(lp_tmp, dominanat_seg_no1, SegmentNumberTag);
    point_numlist1 = almost_horizontalpoints_tmp.TaggedPointNumberList(SegmentNumberTag, dominanat_seg_no1);

    almost_horizontalpoints_tmp.RemoveTaggedPoints(dominanat_seg_no1, SegmentNumberTag);
    dominanat_seg_no2 = almost_horizontalpoints_tmp.MostFrequentAttributeValue(SegmentNumberTag, count2);
    //dominanat_seg2_lp.AddTaggedPoints(lp_tmp, dominanat_seg_no2, SegmentNumberTag);
    point_numlist2 = almost_horizontalpoints_tmp.TaggedPointNumberList(SegmentNumberTag, dominanat_seg_no2);

    ObjectPoint seg1_cent, seg2_cent;
    seg1_cent = laserpoints.Centroid(point_numlist1, dominanat_seg_no1);
    seg2_cent = laserpoints.Centroid(point_numlist2, dominanat_seg_no2);
    /// use dominant segment concept to estimate floor and ceiling height,
    /// so if floor, and ceiling have the most no of points
    //ceiling_z = max(seg1_cent.Z(), seg2_cent.Z());
    //floor_z   = min(seg1_cent.Z(), seg2_cent.Z());

    /// hard coded for visualisation, should be changed also in >> ceiling_z_average, end of the code
    //ceiling_z = 1.07 ; //5.10; //1.70; //1.36  ;  //1.95; //4.75; 2.90; -1.62;  // change also
    //floor_z   = -1.35; //1.60; //-1.75; //-0.85; //-0.73; //0.0;  0.0; -4.29; // hardcoded
    //printf("Ceiling Height estimation: %.2f \n", ceiling_z);
   // printf("Floor Height estimation: %.2f \n", floor_z);
/*    ceiling_z = max(dominanat_seg1_lp.DeriveDataBounds(0).Minimum().GetZ(),
                    dominanat_seg2_lp.DeriveDataBounds(0).Minimum().GetZ());*/

    printf("%d segments are larger than %d\n", planes.size(), minsizesegment);

    ///  *** processing segments two by two for topology relation ***
    LineTopology      intersection_line_segment;
    ObjectPoint       beginp_objpnt, endp_objpnt;
    ObjectPoints      segments_pairpoints, shapepoints_intersection;
    LineTopologies    segments_pairlines, shapelines_intersection;
    vector <std::pair <int , int> >  pending_walls;

    int     takelabel      =0,
            labwallceiling =1,
            //labwall_notceiling = 11,
            labwallfloor   =2,
            labwallwall    =3,
            labelceilceil = 4,
            labelfloorfloor = 5,
            labelwallslantedwall= 6;

    PointNumberLists::iterator  point_list_it1;
    Planes::iterator            plane_it1;
    int     intersection_cnt = 0;
    int     line_number      = 0;

    FILE *statfile;
    strcpy (str_root,root);
    statfile = fopen(strcat(str_root, "graphinfo.txt"),"w");
    fprintf(statfile, "segment_nr1, segment_nr2, takelabel, planes' angle \n");

    FILE *statfile2;
    strcpy (str_root,root);
    statfile2 = fopen(strcat(str_root, "graphinfo2.txt"),"w");
    fprintf(statfile2, "segment_nr1, segment_nr2, takelabel, planes' angle, planes' distance \n");

    // debugger
    FILE *statWalls;
    strcpy (str_root,root);
    statWalls = fopen(strcat(str_root, "statWalls.txt"),"w");
    fprintf(statWalls, "Segments may belong to the same wall..\n");
    fprintf(statWalls, "segment_nr1, segment_nr2, planes' angle, planes' distance \n");

    /// looping through point_lists and corresponding plane, each point_list representing a segment
    for (point_list_it1 = point_lists.begin(), plane_it1 = planes.begin();
         point_list_it1 != point_lists.end(),  plane_it1 != planes.end(); point_list_it1++, plane_it1++){

        int segment_nr1, label1;
        /// get the segment nr of the current pointlist (segment)
        segment_nr1 = (laserpoints[point_list_it1 -> begin() -> Number()]).Attribute(SegmentNumberTag);
        /// get the label, should be 1 for Horiz or 2 for vertic
        label1      = (laserpoints[point_list_it1 -> begin() -> Number()]).Attribute(LabelTag);

        PointNumberLists::iterator  point_list_it2;
        Planes::iterator            plane_it2;
        /// selecting second segment point list and second plane
        for (point_list_it2 = point_list_it1 + 1, plane_it2 = plane_it1 +1;
             point_list_it2 != point_lists.end(), plane_it2 != planes.end(); point_list_it2++, plane_it2++){

            int segment_nr2, label2;
            segment_nr2 = (laserpoints[point_list_it2 -> begin() -> Number()]).Attribute(SegmentNumberTag);
            label2      = (laserpoints[point_list_it2 -> begin() -> Number()]).Attribute(LabelTag);

            SetColor(GREEN);
            printf("Analysing segments: %4d and %4d \r", segment_nr1, segment_nr2);
            SetColor(GRAY); /// set color back to default console

            // debugger
            if (verbose && segment_nr1 == 2 && segment_nr2 == 7) {
                cout << "stop" << endl;
            }

            ///Determine the intersection line of two planes,
            Line3D      intersection_line;
            Vector3D    line_direction;
            if (Intersect2Planes(*plane_it1, *plane_it2, intersection_line)){

                bool b1, b2;
                double planes_angle, planes_distance;
                if(verbose){ /// not necessary computations just for information
                    b1 = (*plane_it1).IsHorizontal(flat_angle * PI / 180);
                    b2 = (*plane_it2).IsHorizontal(flat_angle * PI / 180);
                    /// calculate planes' normal
                    Vector3D normal1, normal2;
                    normal1 = (*plane_it1).Normal();
                    normal2 = (*plane_it2).Normal();
                    /// plane angle is calculated by angle between their normals
                    planes_angle = (acos(normal1.DotProduct(normal2) / (normal1.Length() * normal2.Length()))) * 180.0 / PI;
                    /// planes distance is calculated by the distance of one plane to centerofgravity of another plane
                    planes_distance = fabs((*plane_it1).Distance(plane_it2 -> CentreOfGravity()));
                }

                /// intersect two planes to get the positions of intersection
                /// **** important function ****
                Position3D  intersection_pos1, intersection_pos2;
                if (laserpoints.IntersectFaces(*point_list_it1, *point_list_it2,
                                               *plane_it1, *plane_it2, max_intersection_dist,/// 0.1 is intersection threshold
                                               intersection_pos1, intersection_pos2)){

                    PointNumber pn1, pn2;
                    Covariance3D cov3d;
                    cov3d = Covariance3D(0, 0, 0, 0, 0, 0);

                    /// convert begin and end-point of line intersect to objpoint
                    intersection_cnt++;
                    pn1 = PointNumber(intersection_cnt);
                    beginp_objpnt = ObjectPoint(intersection_pos1, pn1, cov3d);

                    intersection_cnt++;
                    pn2 = PointNumber(intersection_cnt);
                    endp_objpnt = ObjectPoint(intersection_pos2, pn2, cov3d);
                    /// create the line_topology
                    intersection_line_segment = LineTopology(line_number, 1, pn1, pn2);

                    line_direction = intersection_line.Direction();

                    /// labeling the intersection line segment ... not necessary procedure
                    if(verbose){
                        /// label horizontal lines
                        if (fabs(line_direction[2]) < flat_angle * PI / 180){
                            intersection_line_segment.Label() = 6;  // almost horizontal, label 6
                        }else{
                            intersection_line_segment.Label()= 3;  // non-horizontal lines
                        }

                        if(b1 && b2) intersection_line_segment.Label() = 4; /// both planes are horizontal
                    }

                    /// calculate the intersection_line length
                    double line_length;
                    double min_linelength = 0.1;
                    line_length = intersection_pos1.Distance(intersection_pos2);
                    //printf("line lenght: %.2f \n", line_length);

                    if (line_length > min_linelength){
                        ObjectPoint seg1_cent_objp, seg2_cent_objp;
                        seg1_cent_objp = laserpoints.Centroid(*point_list_it1, segment_nr1);
                        seg2_cent_objp = laserpoints.Centroid(*point_list_it2, segment_nr2);

                        /* labeling process of intersection line ***
                         * already we had label =1 as horizontal and label =2 as vertical for segments
                         * labwallceiling  = 1, labwall_notceiling  = 11,
                         * labwallfloor    = 2,
                         * labwallwall     = 3;
                         * */
                        if(segment_nr1 == 57 && segment_nr2 == 68)
                            printf("debug \n");
                        takelabel = 0;
                        //printf ("\n"); // debug
                        //TODO: handle the case when two almost vertical segments where one above each other their intersection is horizontal
                        if (label1 ==2 && label2 == 2) takelabel = labwallwall;
                        if (label1 ==2 && label2 ==2){
                            /// check if the intersection of two wall candidates is horizontal
                            if (intersection_line.IsHorizontal (10.0 * PI / 180)){
                                LaserPoints seg1_lp, seg2_lp;
                                DataBoundsLaser db1, db2;  /// collect data bounds of the segments to check the support relation
                                seg1_lp = laserpoints.SelectTagValue(SegmentNumberTag, segment_nr1);  // too expensive
                                seg2_lp = laserpoints.SelectTagValue(SegmentNumberTag, segment_nr2);  // too expensive
                                db1 = seg1_lp.DeriveDataBounds(0);
                                db2 = seg2_lp.DeriveDataBounds(0);
                                /// check the center of which segment is higher
                                if (seg1_cent_objp.Z () > seg2_cent_objp.Z ()){
                                    /* /// check the support relation between two segments ///
                                     if the minimum height of upper segment is higher than the maximum height
                                     of the lower segment, then the lower segment is supporting the upper segment */
                                    printf ("db1.minZ: %2.2f , db2.maxZ %2.2f \n", db1.Minimum ().GetZ (), db2.Maximum ().GetZ ()); //debug
                                    printf ("db1.minZ - db2.maxZ: %2.2f \n", fabs(db1.Minimum ().GetZ ()- db2.Maximum ().GetZ ())); // debug
                                    if(fabs(db1.Minimum ().GetZ () - db2.Maximum ().GetZ () ) <= 0.10 ) {
                                        takelabel = labelwallslantedwall;
                                    }
                                } else { /// else center of seg2 is higher than center of seg1
                                    printf ("ZC2 > ZC1 and \n db2.minZ: %2.2f , db1.maxZ %2.2f \n",
                                            db2.Minimum ().GetZ (), db1.Maximum ().GetZ ()); //debug
                                    printf ("db2.minZ - db1.maxZ: %2.2f \n", fabs(db2.Minimum ().GetZ ()- db1.Maximum ().GetZ ())); // debug
                                    if(fabs(db2.Minimum ().GetZ () - db1.Maximum ().GetZ () ) <= 0.10) {
                                        takelabel = labelwallslantedwall;
                                    }
                                }
                                  printf("takelabel: %d \n", takelabel);  //debug
                            }
                        }

                        if (label1 == 2 && label2 == 1 && seg2_cent_objp.Z() > seg1_cent_objp.Z() &&
                                (fabs(seg2_cent_objp.Z() - ceiling_z)) <= 1.0) // check if horiz-segment is close to the ceiling
                            takelabel = labwallceiling;

                        if (label1 == 1 && label2 == 2 && seg1_cent_objp.Z() > seg2_cent_objp.Z() &&
                                (fabs(seg1_cent_objp.Z() - ceiling_z)) <= 1.0) // check if horiz-segment is close to the ceiling
                            takelabel = labwallceiling;

                        if (label1 == 2 && label2 == 1 && seg1_cent_objp.Z() > seg2_cent_objp.Z() &&
                                (fabs(seg2_cent_objp.Z() - floor_z) <= 0.25))  // check if horiz-segment is close to the floor
                            takelabel = labwallfloor;
                        if (label1 == 1 && label2 == 2 && seg2_cent_objp.Z() > seg1_cent_objp.Z() &&
                                (fabs(seg1_cent_objp.Z() - floor_z) <= 0.25)) // check if horiz-segment is close to the floor
                            takelabel = labwallfloor;

                        /// if both segments are almost-horizontal and near the ceiling
                        if (label1 ==1 && label2 == 1){
                            if(((fabs(seg1_cent_objp.Z() - ceiling_z)) <= 1.0) ||
                               ((fabs(seg2_cent_objp.Z() - ceiling_z)) <= 1.0))
                                takelabel = labelceilceil;
                        }

                        /// if both segments are almost-horizontal and near the floor
                        if (label1 ==1 && label2 == 1){
                            if(((fabs(seg1_cent_objp.Z() - floor_z)) <= 0.25) ||
                               ((fabs(seg2_cent_objp.Z() - floor_z)) <= 0.25))
                                takelabel = labelfloorfloor;
                        }

                        //printf ("TakeLabel2: %d \n", takelabel); //debug

                        /// labwall_notceiling is for wall and horizontal_plane connection
                        /// by this we avoid vertical_planes connected to a horiz_plane get wall label
/*                            if (label1 == 2 && label2 == 1 && seg2_cent_objp.Z() > seg1_cent_objp.Z() &&
                                 (fabs(seg2_cent_objp.Z() - ceiling_z)) > 0.20)
                            takelabel = labwall_notceiling;
                        if (label1 == 1 && label2 == 2 && seg1_cent_objp.Z() > seg2_cent_objp.Z() &&
                                (fabs(seg1_cent_objp.Z() - ceiling_z)) > 0.20)
                            takelabel = labwall_notceiling;*/

                        line_number++;
                        LineTopology segments_pairline;
                        segments_pairline = LineTopology(line_number, 1, segment_nr1, segment_nr2);
                        /// label the line that pairs two segments (this is not intersection line)
                        segments_pairline.SetAttribute(BuildingPartNumberTag, takelabel); // to be visualised in PCM
                        segments_pairline.Label() = takelabel;

                        //LineTopologies segments_pairlines;
                        segments_pairlines.push_back(segments_pairline);

                        /// this is intersection line, we set to it new labels
                        LineTopology shapeline_intersection;
                        shapeline_intersection = intersection_line_segment;
                        shapeline_intersection.Label() = takelabel;
                        shapeline_intersection.SetAttribute(BuildingPartNumberTag, takelabel); // to be visualised in PCM

                        //ObjectPoints shapepoints_intersection;
                        shapepoints_intersection.push_back(beginp_objpnt);
                        shapepoints_intersection.push_back(endp_objpnt);

                        //LineTopologies shapelines_intersection;
                        shapelines_intersection.push_back(shapeline_intersection);
                        //shapelines_intersection_test.push_back(intersection_line_segment);

                        fprintf (statfile, "%4d, %4d, %d, %3.2f \n", segment_nr1, segment_nr2, takelabel, planes_angle);
                    }
                }
                // debugger
                fprintf (statfile2, "%4d, %4d, %d, %3.2f, %.2f \n", segment_nr1, segment_nr2, takelabel, planes_angle, planes_distance);
            }
        } // end of for point_list_it2

        ObjectPoint seg1_cent_objp; /// ??? define and calculate again?
        seg1_cent_objp = laserpoints.Centroid(*point_list_it1, segment_nr1);
        segments_pairpoints.push_back(seg1_cent_objp);
    } // end of for loop point_list_it

    printf ("\n");
    strcpy (str_root,root);
    shapepoints_intersection.Write(strcat(str_root, "intersection_lines.objpts"));
    strcpy (str_root,root);
    shapelines_intersection.Write(strcat(str_root, "intersection_lines.top"), false);
    //shapelines_intersection_test.Write("D://test//indoor_reconstruction//intersection_lines_test.top", false);

    segments_pairpoints.RemoveDoublePoints(segments_pairlines, 0.01);

    /// loop through graph nodes, each node (segments_pairpoints) representing a segment
    int wall_count = 0;
    ObjectPoints::iterator segments_pairpoints_it; /// this representing a segment
    LaserPoints outputpoints, wall_points, unknown_points, ceiling_points, floor_points;
    vector <int> relabeled_walls;  /// contain segment numbers that can be wall

    fprintf(statfile, "\n print statistics..\n");
    fprintf(statWalls, "\n List of walls ...\n");
    for (segments_pairpoints_it = segments_pairpoints.begin();
         segments_pairpoints_it != segments_pairpoints.end(); segments_pairpoints_it++){

        int connection_count    = 0;
        int count_wallwall      = 0,
            count_wallceiling   = 0,
            count_wallfloor     = 0,
            count_ceiliceil     = 0,
            count_floorfloor    = 0,
            count_wallslantedwall = 0;

/*        if (segments_pairpoints_it -> Number() == 67){
            cout << "debug" << endl;
        }*/

        /// for each node, we loop through graph-edges (segments_pairlines), graph-edges connect segments
        LineTopologies::iterator segments_pairlines_it;
        for (segments_pairlines_it = segments_pairlines.begin();
                segments_pairlines_it != segments_pairlines.end(); segments_pairlines_it++){
            /// find edges that contain this node (segments_pairpoints_it)
            if(segments_pairlines_it -> Contains(segments_pairpoints_it -> NumberRef())){
                connection_count++; /// count number of connections
                /// important for labeling
                if(segments_pairlines_it -> Attribute(LineLabelTag) == labwallceiling)        count_wallceiling++;
                //if(segments_pairlines_it -> Attribute(LineLabelTag) == labwall_notceiling)  count_wall_notceiling++;
                if(segments_pairlines_it -> Attribute(LineLabelTag) == labwallfloor)          count_wallfloor++;
                if(segments_pairlines_it -> Attribute(LineLabelTag) == labwallwall)           count_wallwall++;
                if(segments_pairlines_it -> Attribute(LineLabelTag) == labelceilceil)         count_ceiliceil++;
                if(segments_pairlines_it -> Attribute(LineLabelTag) == labelfloorfloor)       count_floorfloor++;
                if(segments_pairlines_it -> Attribute(LineLabelTag) == labelwallslantedwall)  count_wallslantedwall++;
            }
        }
        //printf ("Count slanted walls: %d \n", count_wallslantedwall); //debug
        /// collect points for current segment
        LaserPoints segment_laserpoints;
        int segments_pairpoints_nr; /// current segment_number
        segments_pairpoints_nr = segments_pairpoints_it -> Number();
        segment_laserpoints = laserpoints.SelectTagValue(SegmentNumberTag, segments_pairpoints_nr);

        if (connection_count == 0){
            segment_laserpoints.SetAttribute(LabelTag, 0); /// 0 is default labeltag
        }

        //if (verbose){
            int count1;
            takelabel = 0;
            takelabel = segment_laserpoints.MostFrequentAttributeValue(LabelTag, count1);
            // 0 default, 1 horizon and 2 vertical
        if (takelabel==0) fprintf(statfile, "segment %d (%d points) is not connected.\n",
                                  segments_pairpoints_nr, segment_laserpoints.size());
        if (takelabel==1) fprintf(statfile, "segment %d (%d points) is horizontal, has %d connections,\n",
                                  segments_pairpoints_nr, segment_laserpoints.size(), connection_count);
        if (takelabel==2) fprintf(statfile, "segment %d (%d points) is vertical, has %d connections,\n",
                                  segments_pairpoints_nr, segment_laserpoints.size(), connection_count);

            fprintf(statfile, "of which %d ww, %d wcl and %d wfl  and %d clcl and %d flfl and %d wslw connections.\n",
                    count_wallwall, count_wallceiling, count_wallfloor, count_ceiliceil, count_floorfloor, count_wallslantedwall);
        //}

        /// The segment connected to the floor AND ceiling, would be a wall is not always true
        //if (count_wallceiling == 1 && count_wallfloor ==1)  segment_laserpoints.SetAttribute(LabelTag, 4); /// label wall

        /* Not connected segments to the ceiling may get wall label, because any horizontal segment is considered ceiling
        * count_wallceiling == 1 always is not correct, because every segment may be connected more than once to the ceiling
        */
        if (count_wallceiling >= 1 && takelabel != 1){ /// we exclude horizontal planes getting wall label
            segment_laserpoints.SetAttribute(LabelTag, 4); /// label wall
            wall_count++;
            /// list of wall segments
            fprintf(statWalls, "%d, ", segments_pairpoints_nr);
            /// collect segments that can be wall because of closeness and almost parallel plane to current wall
            if (pending_walls.size()){
                vector<std::pair<int, int>>::iterator it;
                for (it = pending_walls.begin(); it != pending_walls.end(); it++){
                    if (it ->first == segments_pairpoints_nr){
                        relabeled_walls.push_back(it ->second);
                    }
                    if(it -> second == segments_pairpoints_nr){
                        relabeled_walls.push_back(it -> first);
                        //fprintf(statWalls, "(%d,%d), ", it ->first, it ->second);
                    }
                }
/*            it = find_if(pending_walls.begin(), pending_walls.end(),
                         [&segments_pairpoints_nr](const std::pair<int, int>& element)
                                { return element.second == segments_pairpoints_nr;}
                        );*/
            }
        }
        if (count_wallslantedwall > 0 && count_wallwall >0 && takelabel != 1)
            segment_laserpoints.SetAttribute(LabelTag, 3); /// label wall-slanted-wall

        if (count_wallfloor > 2 && count_wallwall==0)       segment_laserpoints.SetAttribute(LabelTag, 5); /// label floor
        if (count_wallceiling > 2 && count_wallwall==0)     segment_laserpoints.SetAttribute(LabelTag, 6); /// label ceiling
        if (count_wallceiling > 0 && count_wallwall==0 && count_ceiliceil > 0 && takelabel !=2) /// for non horizontal ceiling
            segment_laserpoints.SetAttribute(LabelTag, 6);
        if (count_wallfloor > 0 && count_wallwall==0 && count_floorfloor > 0 && takelabel !=2) /// label non horizontal floor (ramp)
            segment_laserpoints.SetAttribute(LabelTag, 5);

        /// store floor data
        if (segment_laserpoints[0].Attribute(LabelTag) == 5)
            floor_points.AddPoints(segment_laserpoints);
        /// store ceiling data
        if (segment_laserpoints[0].Attribute(LabelTag) == 6)
            ceiling_points.AddPoints(segment_laserpoints);
        /// store wall points
        if (segment_laserpoints[0].Attribute(LabelTag) == 4 ||    // walls
                segment_laserpoints[0].Attribute(LabelTag) == 3) // slannted walls
            wall_points.AddPoints(segment_laserpoints);
        /// store unknown points
        if (segment_laserpoints[0].Attribute(LabelTag) == 2 || //almost vertical points
                segment_laserpoints[0].Attribute(LabelTag) == 1 || // almost horizontal points
                segment_laserpoints[0].Attribute(LabelTag) == 0) // not_vetical and not_horizontal
            unknown_points.AddPoints(segment_laserpoints);
        /// store all labeled points
        outputpoints.AddPoints(segment_laserpoints);
    }

    /// Prune wall points above the floor and below the ceiling // this is not necessary, unless we want to have
    /// a rectangle wall shape for occlusion test
    if(crop_walls_abovefloor_and_belowceiling){
        DataBoundsLaser db_floor, db_ceiling;
        db_floor    = floor_points.DeriveDataBounds(0);
        db_ceiling  = ceiling_points.DeriveDataBounds(0);

        double floor_z_average, ceiling_z_average;
        if (db_floor.MinimumZIsSet() && db_floor.MaximumZIsSet() &&
            db_ceiling.MinimumZIsSet() && db_ceiling.MaximumZIsSet()){
            floor_z_average = (db_floor.Minimum().GetZ() + db_floor.Maximum().GetZ())/2;
            ceiling_z_average = (db_ceiling.Minimum().GetZ() + db_ceiling.Maximum().GetZ())/2;

            ceiling_z_average = 5.11; //1.70;  //1.35; //1.95;   //4.75; 2.90 ; -1.62; // 1.95;   // hardcoded because of estimation problem
            floor_z_average   = 1.64; //-1.75; //-0.85; //-0.75;  //0.0;  0.0 ;-4.29; // -0.75;  // hardcoded because of estimation problem

            LaserPoints::iterator wallpoint_it;
            LaserPoints wall_croped;
            wall_croped = wall_points;
            for (wallpoint_it = wall_points.begin(); wallpoint_it != wall_points.end(); wallpoint_it++){

                if (wallpoint_it->GetZ() > ceiling_z_average || wallpoint_it->GetZ() < floor_z_average){
                    wallpoint_it->SetAttribute(LabelTag, 8);  // wall points above the ceiling or lower than floor
                }
            }
            strcpy (str_root,root);
            wall_points.Write(strcat(str_root, "walls_cropped.laser"), false);
            wall_points.RemoveTaggedPoints(8, LabelTag);
        }
    }

    strcpy (str_root,root);
    segments_pairpoints.Write(strcat(str_root, "pairlines.objpts")), strcpy (str_root,root);
    segments_pairlines.Write(strcat(str_root, "pairlines.top"), false), strcpy (str_root,root);

    unknown_points.Write(strcat(str_root, "unknown_points.laser"), false), strcpy (str_root,root);
    floor_points.Write(strcat(str_root, "floor.laser"), false), strcpy (str_root,root);
    ceiling_points.Write(strcat(str_root, "ceiling.laser"), false), strcpy (str_root,root);
    wall_points.Write(strcat(str_root, "walls.laser"), false), strcpy (str_root,root);
    outputpoints.Write(strcat(str_root, "outputlaser.laser"), false);

    /// look into unknown segments for candidate walls
    if (relabeled_walls.size()){
        SetColor(GREEN);
        printf ("Relabeling pending walls ... \n");
        SetColor(GRAY);

        vector<int>             seg_nrs;
        vector<int>::iterator   seg_it;
        LaserPoints relabeled_walls_lp;
        seg_nrs = unknown_points.AttributeValues(SegmentNumberTag);  // vector of segment numbers
        for(seg_it = seg_nrs.begin(); seg_it != seg_nrs.end(); seg_it++){

            vector<int>::iterator it;
            it = find(relabeled_walls.begin(), relabeled_walls.end(), *seg_it);
            if (*it == *seg_it){
                LaserPoints seg_lp;
                seg_lp = unknown_points.SelectTagValue(SegmentNumberTag, *seg_it);
                seg_lp.SetAttribute(LabelTag, 44); /// relabel as wall
                relabeled_walls_lp.AddPoints(seg_lp);
            }
        }
        strcpy (str_root,root);
        relabeled_walls_lp.Write(strcat(str_root, "relabeled_walls.laser"), false);
    }



    strcpy (str_root,root);
    small_segments.Write(strcat(str_root, "small_segments.laser"), false);
    if (verbose) strcpy (str_root,root),
                notsegmented_points.Write(strcat(str_root, "notsegmented_points.laser"), false);

    fclose(statfile);
    fclose(statfile2);
    fclose(statWalls);
    fclose(segments_infofile);
    fclose(almost_horizon_infofile);
    fclose(almost_horizon_subset_infofile);


    strcpy (str_root,root);
    laserpoints.Write(strcat(str_root, "laserpoints.laser"), false);

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"Total wall processing time: "<< duration/60 << "m" << '\n';

    /*
     * Occlusion test for opening detection
     * */
    if (do_occlusion_test){
        /// read trajectory points (scanner positions)
        char* trajFile;
        LaserPoints trajpoints;
        //trajFile = (char*) "D://test//indoor_reconstruction//data//trajectory3.laser";
         trajFile = (char *) "D:/test/zebrevo_ahmed/Revo_Trajectory.laser";
        trajpoints.Read(trajFile);
        strcpy (str_root,root);
        occlusion_test(laserpoints, wall_points, trajpoints, segment_buffers, 0.60, root);
    }

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"Total processing time: "<< duration/60 << "m" << '\n';

}

