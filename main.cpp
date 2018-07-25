#include <iostream>
#include <ctime>
#include "InlineArguments.h"
#include <cstdlib>
#include <limits>
#include <LaserPoints.h>
#include "Buffers.h"
#include "indoor_reconstruction.h"
#include "post_processing.h"
#include "PlaneFittingResidual.h"
#include "Laservoxel.h"

using namespace std;

void PrintUsage()
{
    printf("Usage: indoor_reconstruction -i <input laserpoints , foo.laser>\n");
    printf("                             -o <output laser file, foo_segmented.laser>\n");
    printf(  "                  -maxd_p <optional: max dist to plane, default: 0.05>\n");
    printf("                    -maxd_s <optional: max dist to surface, default: 0.05>\n");
}

void test_function(char*, char*);

void VisulizePlane3D (LaserPoints segment_lp, double max_dist, ObjectPoints &plane_corners,
                      LineTopology &plane_edges, bool verbose);

int main(int argc, char *argv[]) {

    //auto     args = new InlineArguments(argc, argv);
    InlineArguments     *args = new InlineArguments(argc, argv);


    std::clock_t start;
    double duration;
    start = std::clock();

    //test_function ();

    /// Check on required input files
/*    if (args->Contains("-usage")) {
        PrintUsage();
        exit(0);
    }

    if (!args->Contains("-i") ||
        !args->Contains("-o"))
    {
        printf("Error: missing programme option.\n");
        PrintUsage();
        return EXIT_SUCCESS;
    }*/

    /// main function
/*    Segmentation_SurfaceGrowing (args->String("-i"),
                                 args->String("-o"),
                                 args->Double("-maxd_p", 0.05),
                                 args->Double("-maxd_s", 0.05));*/

    test_function (args->String ("-i"), args->String ("-o"));



    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"Total processing time: "<< duration/60 << "m" << '\n';


/*    std::cout << "Press ENTER to continue...";
    std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );*/

    return EXIT_SUCCESS;
}