
#include <ctime>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

using json = nlohmann::json;

#include "CSVParser.hpp"
#include "Geodesy.hpp"
#include "GeodesyThirdParty.hpp"
#include "StandaloneFunctions.hpp"
#include "Mapper.hpp"

std::string imgFn0 = "../Data/space_eye.jpg";
std::string imgFn1 = "../Data/space_milkyway.jpg";

double scaleFactor = 0.2;

void show_data_type_sizes(void)
{
    std::cout << "sizeof(int) = " << sizeof( int ) << std::endl;
    std::cout << "sizeof(float) = " << sizeof( float ) << std::endl;
    std::cout << "sizeof(double) = " << sizeof( double ) << std::endl;
}

int main(int argc, char* argv[])
{
    std::cout << "Hello TestBlending!" << std::endl;

    show_data_type_sizes();

    // test_side_by_side_blending(imgFn0, imgFn1, scaleFactor);
    // test_region_blending(imgFn0, imgFn1, scaleFactor);
    // test_region_blending_and_overwriting(imgFn0, imgFn1, scaleFactor);
    // test_boost_filesystem_and_opencv_yaml();
    // test_two_image_homography();
    // test_two_image_homography_direct_blending();
    // csv::test_read_csv( "/media/yaoyu/DiskE/WJKJ/Datasets/fushun1114/A_offset_XY.csv" );
    // csv::test_read_csv( "/media/yaoyu/DiskE/WJKJ/Datasets/fushun1114/A_offset.csv" );
    // geo::test_geodesy();
    // geo::test_geodesy_thirdparty();
    // test_multi_image_homography_direct_blending(true);

    std::cout << "argc = " << argc << endl;
    for ( int i = 0; i < argc; ++i )
    {
        std::cout << "argv[" << i << "] = " << argv[i] << endl;
    }

    if ( argc <= 1 )
    {
        std::cout << "Not enough arguments." << endl;
        return -1;
    }

    // Read the input JSON file.
    std::ifstream ifs( argv[1] );
    json j;
    ifs >> j;

    std::cout << "The input JSON file is: " << std::endl;
    std::cout << j.dump(4) << std::endl;

    if ( j["app"] == "direct" )
    {
        mapper::BlendedMapper bm;

        // // bm.enable_debug();
        // // bm.set_debug_count(5);

        std::clock_t begin = std::clock();
        bm.multi_image_homography_direct_blending( 
            j["direct"]["inputDir"], j["direct"]["outputDir"], j["direct"]["inputCSV"],
            j["direct"]["skipBlending"], j["direct"]["skipSeamFinding"]
        );
        std::clock_t end = std::clock();
        std::cout << "Time elapsed " << double( end - begin ) / CLOCKS_PER_SEC << "s." << std::endl;
    }
    else if ( j["app"] == "incremental" )
    {
        std::clock_t begin = std::clock();
        test_multi_image_homography(j["incremental"]["inputDir"], j["incremental"]["outputDir"]);
        std::clock_t end = std::clock();
        std::cout << "Time elapsed " << double( end - begin ) / CLOCKS_PER_SEC << "s." << std::endl;
    }
    else
    {
        std::cout << "Unexpected \"app\" setting. \"app\" = \"" << j["app"] << "\"" << std::endl;
        return -1;
    }

    return 0;
}
