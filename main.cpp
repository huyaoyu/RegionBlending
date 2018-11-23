
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/optional.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/stitching/detail/blenders.hpp>

std::string imgFn0 = "../Data/space_eye.jpg";
std::string imgFn1 = "../Data/space_milkyway.jpg";

double scaleFactor = 0.2;

void test_side_by_side_blending(void)
{
    // Read the input images.
    cv::Mat img0_ori = cv::imread(imgFn0, cv::IMREAD_COLOR);
    cv::Mat img1_ori = cv::imread(imgFn1, cv::IMREAD_COLOR);

    // Resize the images.
    cv::Mat img0, img1;
    cv::resize( img0_ori, img0, cv::Size(0, 0), scaleFactor, scaleFactor, cv::INTER_LINEAR );
    cv::resize( img1_ori, img1, cv::Size(0, 0), scaleFactor, scaleFactor, cv::INTER_LINEAR );

    // // Show the two images.
    // cv::namedWindow("Img0", cv::WINDOW_NORMAL);
    // cv::namedWindow("Img1", cv::WINDOW_NORMAL);

    // cv::imshow("Img0", img0);
    // cv::imshow("Img1", img1);

    // Create the masks for the two images.
    cv::Mat mask0 = cv::Mat(img0.size(), CV_8UC1);
    cv::Mat mask1 = cv::Mat(img1.size(), CV_8UC1);

    std::cout << "mask0.size() = " << mask0.size() << std::endl;

    mask0.setTo( cv::Scalar(255) );
    mask1.setTo( cv::Scalar(255) );

    // Overlapping pixels.
    int overlappingPixels = (int)std::floor( img0.cols * 0.5 );

    // Filled the mask with 0.
    cv::Point maskP0 = cv::Point( img0.cols/2 - 1 + overlappingPixels, 0 );
    cv::Point maskP1 = cv::Point( img0.cols - 1, img0.rows - 1 );
    cv::rectangle( mask0, maskP0, maskP1, cv::Scalar(0), -1, cv::LINE_4  );

    // Save the mask to file for debugging.
    std::vector<int> jpegParams;
    jpegParams.push_back( cv::IMWRITE_JPEG_QUALITY );
    jpegParams.push_back( 100 );

    cv::imwrite( "mask0.jpg", mask0, jpegParams );

    maskP0.x = 0;
    maskP0.y = 0;
    maskP1.x = img1.cols / 2 - 1 - 1 - overlappingPixels;
    maskP1.y = img1.rows - 1;
    cv::rectangle( mask1, maskP0, maskP1, cv::Scalar(0), -1, cv::LINE_4  );

    // Save the mask to file for debugging.
    cv::imwrite( "mask1.jpg", mask1, jpegParams );

    // Create the blender.
    cv::detail::MultiBandBlender mb = cv::detail::MultiBandBlender();
    mb.setNumBands(5);

    // Prepare the blender.
    mb.prepare(cv::Rect(0, 0, cv::max(img0.cols, img1.cols), cv::max(img0.rows, img1.rows)));

    // Feed images to the blender.
    mb.feed( img0, mask0, cv::Point( 0, 0 ) );
    mb.feed( img1, mask1, cv::Point( 0, 0 ) );

    cv::Mat rst, rstMask;
    mb.blend( rst, rstMask );

    // Convert the data type to integer.
    rst.convertTo( rst, CV_8UC3 );

    // Save the result image.
    imwrite( "rst.jpg", rst, jpegParams );

    cv::namedWindow("Blended", cv::WINDOW_NORMAL);
    cv::imshow("Blended", rst);

    cv::waitKey();
}

void test_region_blending(void)
{
    // Read the input images.
    cv::Mat img0_ori = cv::imread(imgFn0, cv::IMREAD_COLOR);
    cv::Mat img1_ori = cv::imread(imgFn1, cv::IMREAD_COLOR);

    // Resize the images.
    cv::Mat img0, img1;
    // cv::resize( img0_ori, img0, cv::Size(0, 0), scaleFactor, scaleFactor, cv::INTER_LINEAR );
    cv::resize( img1_ori, img1, cv::Size(0, 0), scaleFactor, scaleFactor, cv::INTER_LINEAR );
    img0 = img0_ori;

    // // Show the two images.
    // cv::namedWindow("Img0", cv::WINDOW_NORMAL);
    // cv::namedWindow("Img1", cv::WINDOW_NORMAL);

    // cv::imshow("Img0", img0);
    // cv::imshow("Img1", img1);

    // Create the masks for the two images.
    cv::Mat mask0 = cv::Mat(img0.size(), CV_8UC1);
    cv::Mat mask1 = cv::Mat(img1.size(), CV_8UC1);

    std::cout << "mask0.size() = " << mask0.size() << std::endl;

    mask0.setTo( cv::Scalar(0) );
    mask1.setTo( cv::Scalar(255) );

    // Filled the mask with 0.
    cv::Point maskP0 = cv::Point( std::floor((img0.cols - img1.cols)/2), std::floor((img0.rows - img1.rows)/2) );
    cv::Point maskP1 = cv::Point( img0.cols - maskP0.x - 1, img0.rows - maskP0.y - 1 );

    std::cout << "maskP0 = " << maskP0 << ", maskP1 = " << maskP1 << std::endl;

    cv::rectangle( mask0, maskP0, maskP1, cv::Scalar(255), -1, cv::LINE_4  );

    // Save the mask to file for debugging.
    std::vector<int> jpegParams;
    jpegParams.push_back( cv::IMWRITE_JPEG_QUALITY );
    jpegParams.push_back( 100 );

    cv::imwrite( "region_mask0.jpg", mask0, jpegParams );

    // Create the blender.
    cv::detail::MultiBandBlender mb = cv::detail::MultiBandBlender();
    mb.setNumBands(5);

    // Prepare the blender.
    mb.prepare(cv::Rect(0, 0, cv::max(img0.cols, img1.cols), cv::max(img0.rows, img1.rows)));

    // Feed images to the blender.
    mb.feed( img0, mask0, cv::Point( 0, 0 ) );
    mb.feed( img1, mask1, maskP0 );

    cv::Mat rst, rstMask;
    mb.blend( rst, rstMask );

    // Convert the data type to integer.
    rst.convertTo( rst, CV_8UC3 );

    // Save the result image.
    imwrite( "region_rst.jpg", rst, jpegParams );

    cv::namedWindow("Region blended", cv::WINDOW_NORMAL);
    cv::imshow("Region blended", rst);

    cv::waitKey();
}

void test_region_blending_and_overwriting(void)
{
    // Read the input images.
    cv::Mat img0_ori = cv::imread(imgFn0, cv::IMREAD_COLOR);
    cv::Mat img1_ori = cv::imread(imgFn1, cv::IMREAD_COLOR);

    // Resize the images.
    cv::Mat img0, img1;
    // cv::resize( img0_ori, img0, cv::Size(0, 0), scaleFactor, scaleFactor, cv::INTER_LINEAR );
    cv::resize( img1_ori, img1, cv::Size(0, 0), scaleFactor, scaleFactor, cv::INTER_LINEAR );
    img0 = img0_ori;

    // // Show the two images.
    // cv::namedWindow("Img0", cv::WINDOW_NORMAL);
    // cv::namedWindow("Img1", cv::WINDOW_NORMAL);

    // cv::imshow("Img0", img0);
    // cv::imshow("Img1", img1);

    // Create the masks for the two images.
    cv::Mat mask0 = cv::Mat(img0.size(), CV_8UC1);
    cv::Mat mask1 = cv::Mat(img1.size(), CV_8UC1);

    std::cout << "mask0.size() = " << mask0.size() << std::endl;

    mask0.setTo( cv::Scalar(0) );
    mask1.setTo( cv::Scalar(255) );

    // Filled the mask with 0.
    cv::Point maskP0 = cv::Point( std::floor((img0.cols - img1.cols)/2), std::floor((img0.rows - img1.rows)/2) );
    cv::Point maskP1 = cv::Point( img0.cols - maskP0.x - 1, img0.rows - maskP0.y - 1 );

    std::cout << "maskP0 = " << maskP0 << ", maskP1 = " << maskP1 << std::endl;

    cv::rectangle( mask0, maskP0, maskP1, cv::Scalar(255), -1, cv::LINE_4  );

    // Save the mask to file for debugging.
    std::vector<int> jpegParams;
    jpegParams.push_back( cv::IMWRITE_JPEG_QUALITY );
    jpegParams.push_back( 100 );

    cv::imwrite( "region_overwrite_mask0.jpg", mask0, jpegParams );

    // Create the blender.
    cv::detail::MultiBandBlender mb = cv::detail::MultiBandBlender();
    mb.setNumBands(5);

    // Prepare the blender.
    mb.prepare( cv::Rect(0, 0, img1.cols, img1.rows) );

    // Get the portion out of img0.
    cv::Mat portion = cv::Mat( img1.size(), CV_8UC3 );
    cv::Rect rect = cv::Rect( maskP0.x, maskP0.y, img1.cols, img1.rows );
    std::cout << "rect = " << rect << std::endl;
    img0( rect ).copyTo( portion );
    std::cout << "Original pixels copied." << std::endl;

    // Feed images to the blender.
    mb.feed( portion, mask1, cv::Point( 0, 0 ) );
    mb.feed( img1, mask1, cv::Point( 0, 0 ) );

    cv::Mat rst, rstMask;
    mb.blend( rst, rstMask );

    // Convert the data type to integer.
    rst.convertTo( rst, CV_8UC3 );

    // Overwrite img0.
    rst.copyTo( img0(rect) );

    // Save the result image.
    imwrite( "region_overwrite_rst.jpg", img0, jpegParams );

    cv::namedWindow("Region blending & overwrite", cv::WINDOW_NORMAL);
    cv::imshow("Region blending & overwrite", img0);

    cv::waitKey();
}

using namespace boost;
using namespace boost::filesystem;

/*
* This is found at
* https://stackoverflow.com/questions/11140483/how-to-get-list-of-files-with-a-specific-extension-in-a-given-folder
*/
void find_files(const boost::filesystem::path& root, 
    const std::string& ext, std::vector<std::string>& ret)
{
    if(!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root)) return;

    boost::filesystem::directory_iterator it(root);
    boost::filesystem::directory_iterator endit;

    while(it != endit)
    {
        if(boost::filesystem::is_regular_file(*it) && it->path().extension() == ext)
        {
            ret.push_back( 
                root.string() + "/" + it->path().filename().string() );
        }

        ++it;
    }

    // This is found at
    // https://stackoverflow.com/questions/18723984/sorting-files-with-boost-filesystem
    std::sort( ret.begin(), ret.end() );
}

void test_boost_filesystem_and_opencv_yaml(void)
{
    const path inputPath = "../Data/bimos/homographies";
    const path filePattern = "*.yml";

    std::vector<std::string> files;
    find_files(inputPath, ".yml", files);

    std::vector<std::string>::iterator iter;

    for ( iter = files.begin(); iter != files.end(); iter++ )
    {
        std::cout << (*iter) << std::endl;
    }

    // Test reading the yaml files.
    cv::FileStorage fs(files.at(0), cv::FileStorage::READ);
    std::string HFilename = fs["filename"];
    cv::Mat m;
    fs["H"] >> m;

    std::cout << "filename = " << HFilename << std::endl;
    std::cout << "H = " << m << std::endl;
}

int main(void)
{
    std::cout << "Hello TestBlending!" << std::endl;

    // test_side_by_side_blending();

    // test_region_blending();

    // test_region_blending_and_overwriting();

    test_boost_filesystem_and_opencv_yaml();

    return 0;
}
