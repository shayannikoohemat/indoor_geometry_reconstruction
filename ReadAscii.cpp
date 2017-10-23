//
// Created by NikoohematS on 23-1-2017.
//

#include <stdlib.h>
#include <string.h>
#include "LaserPoints.h"



LaserPoints read_ascii(char *ascii_file){

    FILE *ascii;
    printf ("Reading in ascii laser points...\n");
    ascii = Open_Compressed_File(ascii_file, "r");
    if (!ascii) {
        fprintf(stderr, "Error opening input file %s\n", ascii_file);
        exit(0);
    }
//read in ascii laser points, transfer to laser format
    char    line[2048], *comma;
    double  value[21];
    int     index1=0;
    LaserPoint point;
    LaserPoints temp_laser_points, laser_points;

    fgets(line, 2048, ascii);//ignore the first line as this is probably a header

    do {
        if (fgets(line, 1024, ascii)) {
            index1++;
            // Remove the comma's
            while ((comma = strchr(line, ',')) != NULL) * comma = ' ';

            // Read the next line
            sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   value + 1, value + 2, value + 3, value + 4, value + 5,
                   value + 6, value + 7, value + 8, value + 9, value + 10,
                   value + 11, value + 12, value + 13, value + 14, value + 15,
                   value + 16, value + 17, value + 18, value + 19, value + 20);

            // Copy the data to the laser point
            point.X() = value[6] ;
            point.Y() = value[7] ;
            point.Z() = value[8] ;
            point.SetDoubleAttribute(TimeTag, value[2]); // value [1] hard coded

            temp_laser_points.push_back(point);
        }
        //if (index1 == (index1 / 1000) * 1000) {
        //    temp_laser_points.RemoveAlmostDoublePoints(false, 0.01);
            printf(" %d / %d\r", laser_points.size(), index1);
            fflush(stdout);
            laser_points.AddPoints(temp_laser_points);
            temp_laser_points.ErasePoints();
        //}
        //    }
    } while (!feof(ascii));
    printf("\nRead %d points\n", laser_points.size());

    return laser_points;
}


