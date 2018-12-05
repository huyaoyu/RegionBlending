
#include <iostream>
#include <string>
#include <ctime>

#include "CSVParser.hpp"
#include "StandaloneFunctions.hpp"

std::string imgFn0 = "../Data/space_eye.jpg";
std::string imgFn1 = "../Data/space_milkyway.jpg";

double scaleFactor = 0.2;

void show_data_type_sizes(void)
{
    std::cout << "sizeof(int) = " << sizeof( int ) << std::endl;
    std::cout << "sizeof(float) = " << sizeof( float ) << std::endl;
    std::cout << "sizeof(double) = " << sizeof( double ) << std::endl;
}

int main(void)
{
    std::cout << "Hello TestBlending!" << std::endl;

    show_data_type_sizes();

    // test_side_by_side_blending(imgFn0, imgFn1, scaleFactor);

    // test_region_blending(imgFn0, imgFn1, scaleFactor);

    // test_region_blending_and_overwriting(imgFn0, imgFn1, scaleFactor);

    // test_boost_filesystem_and_opencv_yaml();

    // test_two_image_homography();

    // std::clock_t begin = std::clock();
    // test_multi_image_homography();
    // std::clock_t end = std::clock();
    // std::cout << "Time elapsed " << double( end - begin ) / CLOCKS_PER_SEC << "s." << std::endl;

    // test_two_image_homography_direct_blending();

    // csv::test_read_csv( "/media/yaoyu/DiskE/WJKJ/Datasets/fushun1114/A_offset_XY.csv" );

    std::clock_t begin = std::clock();
    test_multi_image_homography_direct_blending(true);
    std::clock_t end = std::clock();
    std::cout << "Time elapsed " << double( end - begin ) / CLOCKS_PER_SEC << "s." << std::endl;

    return 0;
}
