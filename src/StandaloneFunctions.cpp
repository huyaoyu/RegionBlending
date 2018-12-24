
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/optional.hpp>

#include <Eigen/Eigen>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>

#include "CSVParser.hpp"
#include "StandaloneFunctions.hpp"
#include "GeodesyThirdParty.hpp"

void test_side_by_side_blending(std::string& imgFn0, std::string& imgFn1, double scaleFactor)
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

void test_region_blending(std::string& imgFn0, std::string& imgFn1, double scaleFactor)
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

void test_region_blending_and_overwriting(std::string& imgFn0, std::string& imgFn1, double scaleFactor)
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

void find_bounding_xy(const cv::Mat& cp, 
    double& minX, double& maxX, double& minY, double& maxY)
{
    // Clear minX, etc.
    minX = cp.at<double>(0, 0); maxX = minX;
    minY = cp.at<double>(1, 0); maxY = minY;

    double x, y;

    // Compare.
    for ( int i = 1; i < cp.cols; i++ )
    {
        x = cp.at<double>(0, i);
        if ( x < minX )
        {
            minX = x;
        }
        else if ( x > maxX )
        {
            maxX = x;
        }

        y = cp.at<double>(1, i);
        if ( y < minY )
        {
            minY = y;
        }
        else if ( y > maxY )
        {
            maxY = y;
        }
    }  
}

void update_homography( cv::Mat& h, double shiftX, double shiftY )
{
    h.at<double>(0, 2) += shiftX;
    h.at<double>(1, 2) += shiftY;
}

/** 
 * @param cps The corner points.
 * @param hs  The homography matrices associated to each corner points.
 * @param gsh The initial global shifting homography matrix.
 */
void warp_corners( const std::vector<cv::Mat>& cps, 
    const std::vector<cv::Mat>& hs,
    cv::Mat& gsh, 
    double& borderX, double& borderY)
{
    // Clear borderX and borderY.
    borderX = 0.0;
    borderY = 0.0;

    int nCPs = cps.size();
    int nHs  = hs.size();

    if ( nCPs != nHs )
    {
        std::cout << "nCPs != nHs." << std::endl 
                  << "nCPs = " << nCPs
                  << ", nHs = " << nHs
                  << std::endl;
        return;
    }

    cv::Mat cp, h;
    cv::Mat warppedCP;
    double minX, maxX, minY, maxY;
    double shiftX, shiftY;

    // All corner points
    for ( int i=0; i < nCPs; i++ )
    {
        // Retreive corner points and homography matrix.
        cp = cps.at(i);
        h  = hs.at(i);

        // Warp the corner points.
        warppedCP = gsh * h * cp;

        // Find minX, etc.
        find_bounding_xy( warppedCP, minX, maxX, minY, maxY );

        // Show information.
        std::cout << "minX = " << minX
                  << ", maxX = " << maxX
                  << ", minY = " << minY
                  << ", maxY = " << maxY
                  << std::endl;

        // Clear shiftX and shiftY.
        shiftX = 0.0; shiftY = 0.0;

        // x-direction.
        if ( minX < 0 )
        {
            shiftX = -minX;
        }

        if ( maxX > borderX )
        {
            borderX = maxX;
        }

        borderX += shiftX;

        // y-direction.
        if ( minY < 0 )
        {
            shiftY = -minY;
        }

        if ( maxY > borderY )
        {
            borderY = maxY;
        }

        borderY += shiftY;

        // Show information.
        std::cout << "borderX = " << borderX 
                  << ", borderY = " << borderY
                  << ", shiftX = " << shiftX
                  << ", shiftY = " << shiftY
                  << std::endl;

        // Update global shifting homography matrix.
        update_homography( gsh, shiftX, shiftY );
    }    
}

/**
 * Put the four corners into corners matrix. Each column is [x, y, 1]^T.
 * The coordinates will be represented by double precision floating point numbers, that is CV_64F.
 */
void put_corners(const cv::Mat& img, cv::OutputArray _corners)
{
    // Create the Mat for corners.
    _corners.create( 3, 4, CV_64FC1 );
    cv::Mat corners = _corners.getMat();

    // Put the values into corners matrix.
    corners.at<double>(0, 0) =          0.0; corners.at<double>(1, 0) =          0.0; corners.at<double>(2, 0) = 1.0;
    corners.at<double>(0, 1) = img.cols - 1; corners.at<double>(1, 1) =          0.0; corners.at<double>(2, 1) = 1.0;
    corners.at<double>(0, 2) = img.cols - 1; corners.at<double>(1, 2) = img.rows - 1; corners.at<double>(2, 2) = 1.0;
    corners.at<double>(0, 3) =          0.0; corners.at<double>(1, 3) = img.rows - 1; corners.at<double>(2, 3) = 1.0;
}

void put_shifts_and_borders_single(int oriWidth, int oriHeight, const cv::Mat& corners, 
    double& shiftX, double& shiftY, double& borderX, double& borderY)
{
    // Find the bounding x and y coordinates.
    double minX, maxX, minY, maxY;
    find_bounding_xy( corners, minX, maxX, minY, maxY );

    // Clear shiftX and shiftY.
    shiftX = 0; shiftY = 0;

    // Initialize the borders.
    borderX = oriWidth - 1; borderY = oriHeight - 1;

    // Width.
    shiftX = -minX;
    minX  += shiftX;
    maxX  += shiftX;

    borderX = maxX;

    // Hegith.
    shiftY = -minY;
    minY  += shiftY;
    maxY  += shiftY;

    borderY = maxY;
}

void put_pure_translation_homography(cv::OutputArray _dst, double tx, double ty)
{
    _dst.create( 3, 3, CV_64FC1 );
    cv::Mat dst = _dst.getMat();

    dst.at<double>( 0, 0 ) = 1.0; dst.at<double>( 0, 1 ) = 0.0; dst.at<double>( 0, 2 ) =  tx;
    dst.at<double>( 1, 0 ) = 0.0; dst.at<double>( 1, 1 ) = 1.0; dst.at<double>( 1, 2 ) =  ty;
    dst.at<double>( 2, 0 ) = 0.0; dst.at<double>( 2, 1 ) = 0.0; dst.at<double>( 2, 2 ) = 1.0;
}

/**
 * @param canvas The canvas image, mast be big enough to hold the warpped image.
 * @param regionImage The input image that waits to be warpped into canvas.
 * @param rih The original homography matrix of regionImage without any shifting.
 * @param gsh The global shifting homography matrix.
 * @param res The resulting blended image, the size is the same with canvas.
 */
void region_blend(const cv::Mat& canvas, const cv::Mat& regionImage, const cv::Mat& rih, const cv::Mat& gsh, cv::OutputArray res)
{
    // Get the corner matrix of regionImage.
    cv::Mat corners;
    put_corners( regionImage, corners );

    std::cout << "The corners of regionImage are: " << std::endl;
    std::cout << corners << std::endl;

    // Warp the corners.
    cv::Mat warppedCorners;
    warppedCorners = rih * corners;

    std::cout << "The warpped corners are: " << std::endl;
    std::cout << warppedCorners << std::endl;

    // Find the local shifts and borders of regionImage.
    double shiftX, shiftY, borderX, borderY;
    put_shifts_and_borders_single( regionImage.cols, regionImage.rows, warppedCorners,
        shiftX, shiftY, borderX, borderY );

    // Find the bounding box.
    cv::Mat bbUpperLeft = cv::Mat::ones( 3, 1, CV_64FC1 );
    bbUpperLeft.at<double>(0, 0) = std::ceil( -shiftX );
    bbUpperLeft.at<double>(1, 0) = std::ceil( -shiftY );
    
    int bbWH[2] = { std::ceil(borderX), std::ceil(borderY) };

    std::cout << "The upper left point of the original bounding box is: " << std::endl;
    std::cout << bbUpperLeft << std::endl;
    std::cout << "The width and height of the bounding box are: ("
              << bbWH[0] << ", " << bbWH[1] << ")." << std::endl;

    // Warp regionImage within its new bounding box.
    cv::Mat localShiftH = cv::Mat::eye( 3, 3, CV_64FC1 );
    cv::Mat bbri; // Bounding box of regionImage.

    put_pure_translation_homography( localShiftH, shiftX, shiftY );
    cv::warpPerspective( regionImage, bbri, localShiftH * rih, cv::Size(bbWH[0], bbWH[1]) );

    // // Test.
    // cv::namedWindow( "Local warpped image", cv::WINDOW_NORMAL );
    // cv::imshow( "Local warpped image", bbri );
    // cv::waitKey();

    // Calculate the new coordinate of the upper left corner of the bounding box inside the canvas.
    cv::Mat bbulCanvas = gsh * bbUpperLeft;

    std::cout << "bbulCanvas = " << bbulCanvas << std::endl;

    // Copy the bounding box region into a temporary Mat.
    // NOTE: The copied data is shared between regionInCanvas and canvas.
    //       Modifications to regionInCavas will affect data in canvas.
    cv::Rect rectRegionInCanvas(std::floor( bbulCanvas.at<double>(0, 0) ), 
                  std::floor( bbulCanvas.at<double>(1, 0) ),
                  bbWH[0], bbWH[1]);

    std::cout << "rectRegionInCanvas ( " 
              << rectRegionInCanvas.x << ", "
              << rectRegionInCanvas.y << ", "
              << rectRegionInCanvas.x + rectRegionInCanvas.width - 1 << ", "
              << rectRegionInCanvas.y + rectRegionInCanvas.height - 1 << " ). "
              << "With canvas size ( "
              << canvas.cols << ", " << canvas.rows << " )."
              << std::endl;

    cv::Mat regionInCanvas( canvas, rectRegionInCanvas );
    
    // Create mask for bbri.
    cv::Mat maskBBRI, bbriGray;
    cv::cvtColor( bbri, bbriGray, cv::COLOR_BGR2GRAY );
    cv::threshold( bbriGray, maskBBRI, 0.0, 255.0, cv::THRESH_BINARY );
    maskBBRI.convertTo( maskBBRI, CV_8UC1 );

    std::cout << "maskBBRI.type() = " << maskBBRI.type() << std::endl;

    // // Test.
    // cv::namedWindow("Mask for BBRI", cv::WINDOW_NORMAL);
    // cv::imshow("Mask for BBRI", maskBBRI);
    // cv::waitKey();

    // Create mask for regionInCanvas. May be cached for future use.
    cv::Mat maskRIC( regionInCanvas.size(), CV_8UC1 );
    cv::Mat ricGray;
    cv::cvtColor( regionInCanvas, ricGray, cv::COLOR_BGR2GRAY );
    cv::threshold( ricGray, maskRIC, 0.0, 255.0, cv::THRESH_BINARY );
    maskRIC.convertTo( maskRIC, CV_8UC1 );

    std::cout << "maskRIC.type() = " << maskRIC.type() << std::endl;

    // Create the blender.
    cv::detail::MultiBandBlender blender;
    blender.setNumBands(5);

    // Dummy corners for the blender.
    std::vector<cv::Point> dummyCorners;
    dummyCorners.push_back( cv::Point( 0, 0 ) );
    dummyCorners.push_back( cv::Point( 0, 0 ) );

    // Sizes.
    std::vector<cv::Size> vecSizes;
    vecSizes.push_back( regionInCanvas.size() );
    vecSizes.push_back( bbri.size() );

    std::cout << "regionInCanvas.size() = " << regionInCanvas.size() << std::endl;
    std::cout << "bbri.size() = " << bbri.size() << std::endl;

    // Prepare the blender.
    blender.prepare( cv::Rect( dummyCorners.at(0), vecSizes.at(0) ) );

    // Feed data to the blendre.
    blender.feed( regionInCanvas,  maskRIC, dummyCorners.at(0) );
    blender.feed(           bbri, maskBBRI, dummyCorners.at(1) );

    // Blend.
    cv::Mat regionRes, maskRes;
    blender.blend( regionRes, maskRes );

    // Convert the data type to integer.
    regionRes.convertTo( regionRes, CV_8UC3 );

    // // Test.
    // cv::namedWindow( "Blended region", cv::WINDOW_NORMAL );
    // cv::imshow( "Blended region", regionRes );
    // cv::waitKey();

    // Copy the result.
    res.create( canvas.size(), canvas.type() );
    cv::Mat resMat = res.getMat();
    canvas.copyTo(resMat);
    regionRes.copyTo( resMat( rectRegionInCanvas ) );
}

void test_two_image_homography(void)
{
    // Find all the homography files at once.
    const path inputPath = "../Data/bimos/homographies";
    const path filePattern = "*.yml";

    std::vector<std::string> files;
    find_files(inputPath, ".yml", files);

    // Read the homography matrices for the first and second images.
    std::string imgFn0, imgFn1, tempString;
    cv::Mat H0, H1;
    {
        cv::FileStorage fs(files.at(0), cv::FileStorage::READ);
        fs["filename"] >> tempString;

        boost::filesystem::path p(tempString);
        imgFn0 = "../Data/bimos/images/" + p.filename().string();

        fs["H"] >> H0;

        std::cout << "imgFn0 = " << imgFn0 << std::endl;
        std::cout << "H0 = " << H0 << ", type is " << H0.type() << std::endl;

        fs.release();
    }
    {
        cv::FileStorage fs(files.at(1), cv::FileStorage::READ);
        fs["filename"] >> tempString;

        boost::filesystem::path p(tempString);
        imgFn1 = "../Data/bimos/images/" + p.filename().string();

        fs["H"] >> H1;

        std::cout << "imgFn1 = " << imgFn1 << std::endl;
        std::cout << "H1 = " << H1 << ", type is " << H1.type() << std::endl;

        fs.release();
    }

    // Read the two images.
    cv::Mat img0 = cv::imread( imgFn0, cv::IMREAD_COLOR );
    cv::Mat img1 = cv::imread( imgFn1, cv::IMREAD_COLOR );

    // Create 4 corner poitns for img0.
    cv::Mat CPs = cv::Mat(3, 4, CV_64FC1);
    CPs.at<double>(0, 0) =           0.0; CPs.at<double>(1, 0) =           0.0; CPs.at<double>(2, 0) = 1.0;
    CPs.at<double>(0, 1) = img0.cols - 1; CPs.at<double>(1, 1) =           0.0; CPs.at<double>(2, 1) = 1.0;
    CPs.at<double>(0, 2) = img0.cols - 1; CPs.at<double>(1, 2) = img0.rows - 1; CPs.at<double>(2, 2) = 1.0;
    CPs.at<double>(0, 3) =           0.0; CPs.at<double>(1, 3) = img0.rows - 1; CPs.at<double>(2, 3) = 1.0;

    std::cout << "The original corner points of img0 are: " << std::endl;
    std::cout << CPs << std::endl;

    // Warp the corner points.
    cv::Mat warppedCorners;
    warppedCorners = H0 * CPs;

    std::cout << "Warpped corner points of img0 are: " << std::endl;
    std::cout << warppedCorners << std::endl;

    // Create 4 corner poitns for img0.
    cv::Mat CPs1 = cv::Mat(3, 4, CV_64FC1);
    CPs1.at<double>(0, 0) =           0.0; CPs1.at<double>(1, 0) =           0.0; CPs1.at<double>(2, 0) = 1.0;
    CPs1.at<double>(0, 1) = img1.cols - 1; CPs1.at<double>(1, 1) =           0.0; CPs1.at<double>(2, 1) = 1.0;
    CPs1.at<double>(0, 2) = img1.cols - 1; CPs1.at<double>(1, 2) = img1.rows - 1; CPs1.at<double>(2, 2) = 1.0;
    CPs1.at<double>(0, 3) =           0.0; CPs1.at<double>(1, 3) = img1.rows - 1; CPs1.at<double>(2, 3) = 1.0;

    warppedCorners = H1 * CPs1;

    std::cout << "Warpped corner points of img1 are: " << std::endl;
    std::cout << warppedCorners << std::endl;

    // New canvas.
    std::vector<cv::Mat> cpVec;
    cpVec.push_back( CPs  );
    cpVec.push_back( CPs1 );

    std::vector<cv::Mat> hVec;
    hVec.push_back( H0 );
    hVec.push_back( H1 );

    cv::Mat gsh = cv::Mat::eye( 3, 3, CV_64FC1 );

    double borderX = img0.cols - 1;
    double borderY = img0.rows - 1;

    warp_corners( cpVec, hVec, gsh, borderX, borderY );

    std::cout << "New image canvas: " << std::endl;
    std::cout << "gsh = " << std::endl;
    std::cout << gsh << std::endl;
    std::cout << "borderX = " << borderX << ", borderY = " << borderY << std::endl;

    // Create a large canvas.
    cv::Mat canvas = cv::Mat::zeros( std::ceil( borderY ), std::ceil( borderX ), CV_8UC3 );
    cv::Mat blended;

    std::cout << "Canvas size is " << canvas.size() << std::endl;

    // Warp the actual images.
    region_blend( canvas, img0, H0, gsh, blended );

    // Show the blended image.
    cv::namedWindow("First blended image", cv::WINDOW_NORMAL);
    cv::imshow("First blended image", blended);
    // cv::waitKey();

    canvas = blended;
    region_blend( canvas, img1, H1, gsh, blended );

    // Show the blended image.
    cv::namedWindow("Second blended image", cv::WINDOW_NORMAL);
    cv::imshow("Second blended image", blended);
    cv::waitKey();
}

void test_two_image_homography_direct_blending(void)
{
    // Find all the homography files at once.
    const path inputPath = "../Data/bimos/homographies";
    const path filePattern = "*.yml";

    std::vector<std::string> files;
    find_files(inputPath, ".yml", files);

    // Read the homography matrices for the first and second images.
    std::string imgFn0, imgFn1, tempString;
    cv::Mat H0, H1;
    {
        cv::FileStorage fs(files.at(0), cv::FileStorage::READ);
        fs["filename"] >> tempString;

        boost::filesystem::path p(tempString);
        imgFn0 = "../Data/bimos/images/" + p.filename().string();

        fs["H"] >> H0;

        std::cout << "imgFn0 = " << imgFn0 << std::endl;
        std::cout << "H0 = " << H0 << ", type is " << H0.type() << std::endl;

        fs.release();
    }
    {
        cv::FileStorage fs(files.at(1), cv::FileStorage::READ);
        fs["filename"] >> tempString;

        boost::filesystem::path p(tempString);
        imgFn1 = "../Data/bimos/images/" + p.filename().string();

        fs["H"] >> H1;

        std::cout << "imgFn1 = " << imgFn1 << std::endl;
        std::cout << "H1 = " << H1 << ", type is " << H1.type() << std::endl;

        fs.release();
    }

    // Read the two images.
    cv::Mat img0 = cv::imread( imgFn0, cv::IMREAD_COLOR );
    cv::Mat img1 = cv::imread( imgFn1, cv::IMREAD_COLOR );

    // Create 4 corner poitns for img0.
    cv::Mat CPs = cv::Mat(3, 4, CV_64FC1);
    CPs.at<double>(0, 0) =           0.0; CPs.at<double>(1, 0) =           0.0; CPs.at<double>(2, 0) = 1.0;
    CPs.at<double>(0, 1) = img0.cols - 1; CPs.at<double>(1, 1) =           0.0; CPs.at<double>(2, 1) = 1.0;
    CPs.at<double>(0, 2) = img0.cols - 1; CPs.at<double>(1, 2) = img0.rows - 1; CPs.at<double>(2, 2) = 1.0;
    CPs.at<double>(0, 3) =           0.0; CPs.at<double>(1, 3) = img0.rows - 1; CPs.at<double>(2, 3) = 1.0;

    std::cout << "The original corner points of img0 are: " << std::endl;
    std::cout << CPs << std::endl;

    // Warp the corner points.
    cv::Mat warppedCorners;
    warppedCorners = H0 * CPs;

    std::cout << "Warpped corner points of img0 are: " << std::endl;
    std::cout << warppedCorners << std::endl;

    double minX0, maxX0, minY0, maxY0;
    find_bounding_xy( warppedCorners, minX0, maxX0, minY0, maxY0);

    // Create 4 corner poitns for img0.
    cv::Mat CPs1 = cv::Mat(3, 4, CV_64FC1);
    CPs1.at<double>(0, 0) =           0.0; CPs1.at<double>(1, 0) =           0.0; CPs1.at<double>(2, 0) = 1.0;
    CPs1.at<double>(0, 1) = img1.cols - 1; CPs1.at<double>(1, 1) =           0.0; CPs1.at<double>(2, 1) = 1.0;
    CPs1.at<double>(0, 2) = img1.cols - 1; CPs1.at<double>(1, 2) = img1.rows - 1; CPs1.at<double>(2, 2) = 1.0;
    CPs1.at<double>(0, 3) =           0.0; CPs1.at<double>(1, 3) = img1.rows - 1; CPs1.at<double>(2, 3) = 1.0;

    warppedCorners = H1 * CPs1;

    std::cout << "Warpped corner points of img1 are: " << std::endl;
    std::cout << warppedCorners << std::endl;

    double minX1, maxX1, minY1, maxY1;
    find_bounding_xy( warppedCorners, minX1, maxX1, minY1, maxY1);

    std::cout << "minX0 = " << minX0 << ", minY0 = " << minY0 << std::endl
              << "maxX0 = " << maxX0 << ", maxY0 = " << maxY0 << std::endl
              << "minX1 = " << minX1 << ", minY1 = " << minY1 << std::endl
              << "maxX1 = " << maxX1 << ", maxY1 = " << maxY1 << std::endl;

    // Create the masks.
    cv::Mat mask0, mask1;

    std::cout << "img0.size() = " << img0.size() << std::endl;

    mask0.create( img0.size(), CV_8UC1 );
    mask0.setTo( cv::Scalar::all(255) );

    mask1.create( img1.size(), CV_8UC1 );
    mask1.setTo( cv::Scalar::all(255) );

    std::cout << "Mask initialized." << std::endl;

    // Warp the images and the associated masks.
    int warppedWidth  = (int)( maxX0 - minX0 ) + 1;
    int warppedHeight = (int)( maxY0 - minY0 ) + 1;

    cv::Mat localShift = cv::Mat::eye( 3, 3, CV_64FC1 );
    localShift.at<double>(0, 2) = -minX0;
    localShift.at<double>(1, 2) = -minY0;

    cv::Mat warppedImg0, warppedMask0;
    cv::warpPerspective(  img0,  warppedImg0, localShift * H0, cv::Size( warppedWidth, warppedHeight ) );
    cv::warpPerspective( mask0, warppedMask0, localShift * H0, cv::Size( warppedWidth, warppedHeight ) );

    warppedWidth  = (int)( maxX1 - minX1 ) + 1;
    warppedHeight = (int)( maxY1 - minY1 ) + 1;

    localShift.at<double>(0, 2) = -minX1;
    localShift.at<double>(1, 2) = -minY1;

    cv::Mat warppedImg1, warppedMask1;
    cv::warpPerspective(  img1,  warppedImg1, localShift * H1, cv::Size( warppedWidth, warppedHeight ) );
    cv::warpPerspective( mask1, warppedMask1, localShift * H1, cv::Size( warppedWidth, warppedHeight ) );

    cv::imwrite("TestWappedImage0.jpg", warppedImg0);
    cv::imwrite("TestWappedImage1.jpg", warppedImg1);

    // Compensator.
    cv::Ptr<cv::detail::ExposureCompensator> compensator = 
        cv::detail::ExposureCompensator::createDefault(cv::detail::ExposureCompensator::GAIN_BLOCKS);
    std::vector<cv::Point> compensatorPoints;
    std::vector<cv::UMat> compensatorImages;
    std::vector<cv::UMat> compensatorMasks;

    // Seam findeer.
    cv::Ptr<cv::detail::SeamFinder> seamFinder;
    cv::UMat tempFloat;
    std::vector<cv::UMat> seamFinderImages;

// #ifdef HAVE_OPENCV_CUDALEGACY
//         if ( cv::cuda::getCudaEnabledDeviceCount() > 0)
//         {
//             seamFinder = cv::makePtr<cv::detail::GraphCutSeamFinderGpu>(cv::detail::GraphCutSeamFinderBase::COST_COLOR);
//             std::cout << "GPU seam finder created." << std::endl;
//         }
//         else
// #endif
        {
            seamFinder = cv::makePtr<cv::detail::GraphCutSeamFinder>(cv::detail::GraphCutSeamFinderBase::COST_COLOR);
            std::cout << "No GPU seam finder." << std::endl;
        }

    // Blender.
    cv::detail::MultiBandBlender blender;
    blender.setNumBands(5);

    double minX = std::min( minX0, minX1 );
    double minY = std::min( minY0, minY1 );
    double maxX = std::max( maxX0, maxX1 );
    double maxY = std::max( maxY0, maxY1 );

    cv::Rect blenderRect((int)minX, (int)minY, (int)( maxX - minX ) + 1, (int)( maxY - minY ) + 1);

    blender.prepare( blenderRect );

    std::cout << "Rect: " << blenderRect << std::endl;

    std::cout << "Blender prepared." << std::endl;

    std::cout << "wappedMask0.depth() = " << warppedMask0.depth() << std::endl;

    compensatorPoints.push_back( cv::Point( minX0, minY0 ) );
    compensatorPoints.push_back( cv::Point( minX1, minY1 ) );

    compensatorImages.push_back( warppedImg0.getUMat(cv::ACCESS_READ) );
    compensatorImages.push_back( warppedImg1.getUMat(cv::ACCESS_READ) );

    compensatorMasks.push_back( warppedMask0.getUMat(cv::ACCESS_READ) );
    compensatorMasks.push_back( warppedMask1.getUMat(cv::ACCESS_READ) );

    compensatorImages[0].convertTo( tempFloat, CV_32F );
    seamFinderImages.push_back( tempFloat );
    compensatorImages[1].convertTo( tempFloat, CV_32F );
    seamFinderImages.push_back( tempFloat );

    compensator->feed( compensatorPoints, compensatorImages, compensatorMasks );

    compensator->apply( 0, compensatorPoints[0], warppedImg0, warppedMask0 );
    compensator->apply( 1, compensatorPoints[1], warppedImg1, warppedMask1 );

    seamFinder->find( seamFinderImages, compensatorPoints, compensatorMasks );

    cv::Mat dilatedMask0, seamMask0;
    dilate(compensatorMasks[0], dilatedMask0, cv::Mat());
    imwrite("BeforeDilateMask0.jpg", compensatorMasks[0]);
    resize(dilatedMask0, seamMask0, warppedMask0.size(), 0, 0, cv::INTER_LINEAR_EXACT);
    imwrite("TestDilatedMask0.jpg", dilatedMask0);
    warppedMask0 = seamMask0 & warppedMask0;

    cv::Mat dilatedMask1, seamMask1;
    dilate(compensatorMasks[1], dilatedMask1, cv::Mat());
    imwrite("BeforeDilateMask1.jpg", compensatorMasks[1]);
    resize(dilatedMask1, seamMask1, warppedMask1.size(), 0, 0, cv::INTER_LINEAR_EXACT);
    imwrite("TestDilatedMask1.jpg", dilatedMask1);
    warppedMask1 = seamMask1 & warppedMask1;

    blender.feed( warppedImg0, warppedMask0, cv::Point( minX0, minY0 ) );
    blender.feed( warppedImg1, warppedMask1, cv::Point( minX1, minY1 ) );

    cv::Mat blendedImage, blendedMask;
    blender.blend( blendedImage, blendedMask );

    blendedImage.convertTo( blendedImage, CV_8UC3 );

    cv::imwrite("TwoImageBlendingDirectly.jpg", blendedImage);

    cv::namedWindow("Blended", cv::WINDOW_NORMAL);
    cv::imshow("Blended", blendedImage);
    cv::waitKey();
}

void test_multi_image_homography(const std::string& inputDir, const std::string& outputDir)
{
    // Find all the homography files at once.
    const path inputPath = inputDir + "/homographies";
    // const path inputPath = "../Data/edge_scale_0.1_B/homographies";
    const path filePattern = "*.yml";

    std::vector<std::string> files;
    find_files(inputPath, ".yml", files);

    // Read the homography matrices.
    std::string imgFn, tempString;
    cv::Mat H;
    cv::Mat img;

    std::vector<std::string>::iterator iterFile;
    std::vector<cv::Mat> cpVec;
    std::vector<cv::Mat> hVec;
    std::vector<std::string> vecImgFn;

    int nFiles = files.size();
    int count  = 1;

    for( iterFile = files.begin(); iterFile != files.end(); iterFile++ )
    {
        cv::FileStorage fs(*iterFile, cv::FileStorage::READ);
        fs["filename"] >> tempString;

        boost::filesystem::path p(tempString);
        imgFn = inputDir + "/images/" + p.filename().string();
        // imgFn = "../Data/edge_scale_0.1_B/images/" + p.filename().string();
        vecImgFn.push_back( imgFn );
        std::cout << count << "/" << nFiles<< ": " << imgFn << std::endl;

        fs["H"] >> H;

        // std::cout << "imgFn = " << imgFn << std::endl;
        // std::cout << "H = " << H << ", type is " << H.type() << std::endl;

        fs.release();

        img = cv::imread( imgFn, cv::IMREAD_COLOR );

        // Create 4 corner poitns for img.
        cv::Mat CPs = cv::Mat(3, 4, CV_64FC1);
        CPs.at<double>(0, 0) =          0.0; CPs.at<double>(1, 0) =          0.0; CPs.at<double>(2, 0) = 1.0;
        CPs.at<double>(0, 1) = img.cols - 1; CPs.at<double>(1, 1) =          0.0; CPs.at<double>(2, 1) = 1.0;
        CPs.at<double>(0, 2) = img.cols - 1; CPs.at<double>(1, 2) = img.rows - 1; CPs.at<double>(2, 2) = 1.0;
        CPs.at<double>(0, 3) =          0.0; CPs.at<double>(1, 3) = img.rows - 1; CPs.at<double>(2, 3) = 1.0;

        // std::cout << "The original corner points of img are: " << std::endl;
        // std::cout << CPs << std::endl;

        cpVec.push_back( CPs.clone() );
        hVec.push_back( H.clone() );

        count++;
    }

    cv::Mat gsh = cv::Mat::eye( 3, 3, CV_64FC1 );

    double borderX = img.cols - 1;
    double borderY = img.rows - 1;

    warp_corners( cpVec, hVec, gsh, borderX, borderY );

    std::cout << "New image canvas: " << std::endl;
    std::cout << "gsh = " << std::endl;
    std::cout << gsh << std::endl;
    std::cout << "borderX = " << borderX << ", borderY = " << borderY << std::endl;

    // Create a large canvas.
    cv::Mat canvas = cv::Mat::zeros( std::ceil( borderY ) + 1, std::ceil( borderX ) + 1, CV_8UC3 );
    cv::Mat blended;

    std::cout << "Canvas size is " << canvas.size() << std::endl;

    // Warp the actual images.
    std::vector<cv::Mat>::iterator iterMat = hVec.begin();
    nFiles = vecImgFn.size();
    count  = 1;
    for( iterFile = vecImgFn.begin(); iterFile != vecImgFn.end(); iterFile++, iterMat++ )
    {
        // if ( 12 == count )
        // {
        //     break;
        // }

        std::cout << count << "/" << nFiles << ": Blending " << *iterFile << "..." << std::endl;

        img = cv::imread( *iterFile, cv::IMREAD_COLOR );
        H   = *iterMat;

        region_blend( canvas, img, H, gsh, blended );
        canvas = blended;

        count++;
    }

    // Show the blended image.
    cv::namedWindow("Blended image", cv::WINDOW_NORMAL);
    cv::imshow("Blended image", canvas);
    cv::waitKey();

    cv::imwrite(outputDir + "/canvas.jpg", canvas);
}

void convert_csv_table(csv::Table_t& table)
{
    typedef std::vector<geo::GeoTransform::Point_t> PointVec_t;

    PointVec_t lpVec;
    PointVec_t enuVec;

    int count = 0; // Debug.

    for ( auto iter = table.begin(); iter != table.end(); ++iter )
    {
        lpVec.push_back( geo::GeoTransform::Point_t{ PJ_D2R( (*iter).east ), PJ_D2R( (*iter).north ) } );
        count++;

        // if ( 1 == count )
        // {
        //     break;
        // }
    }

    geo::GeoTransform gt(51);
    gt.lat_lon_to_UTM( lpVec, enuVec );

    for ( int i = 0; i < lpVec.size(); ++i )
    {
        table[i].east  = enuVec[i].x;
        table[i].north = enuVec[i].y;
    }
}

void shift_csv_table(csv::Table_t& table, double& shiftEast, double& shiftNorth)
{
    shiftEast  = -table[0].east;
    shiftNorth = -table[0].north;

    for ( auto iter = table.begin(); iter != table.end(); ++iter )
    {
        (*iter).east  += shiftEast;
        (*iter).north += shiftNorth;
    }
}

void test_multi_image_homography_direct_blending(bool skipBlending)
{
    // Find all the homography files at once.
    // std::string basePath = "../Data/edge_scale_0.1_B";
    const std::string basePath = "/media/yaoyu/DiskE/WJKJ/Datasets/fushun1114/Resized/BimosWD";

    const path inputPath = basePath + "/homographies";
    const path filePattern = "*.yml";

    // Parse the CSV file first.
    // const std::string csvFile = basePath + "/A_offset_XY.csv";
    const std::string csvFile = basePath + "/A_offset.csv";
    // csv::CSVParser csvParser;

    csv::Table_t table;
    // Eigen::MatrixXd csvMatrixData;
    std::vector<int> idxKFInCSVTable;
    // csv::CSVParser::parse( csvFile, table, -552835.874403, -4629657.72452, 1);
    csv::CSVParser::parse( csvFile, table, 0.0, 0.0, 0); convert_csv_table(table);
    double shiftEast = 0.0, shiftNorth = 0.0;
    shift_csv_table( table, shiftEast, shiftNorth );
    csv::CSVParser::show_table( table );
    // csv::CSVParser::convert_data_to_Eigen( table, csvMatrixData );

    std::cout << csvFile << " parsed." << std::endl;

    std::vector<std::string> files;
    find_files(inputPath, ".yml", files);

    // Read the homography matrices.
    std::string imgFn, tempString;
    cv::Mat H;
    cv::Mat img;

    std::vector<std::string>::iterator iterFile;
    std::vector<cv::Point> cpVec;
    std::vector<cv::Mat> warppedImgVec;
    std::vector<cv::Mat> warppedMaskVec;
    std::vector<cv::UMat> warppedImgUMatVec;
    std::vector<cv::UMat> warppedMaskUMatVec;

    cv::Mat centerPoint, warppedCenterPoint;
    std::vector<cv::Mat> warppedCenterPointVec;
    std::vector<cv::Mat> HVec;

    cv::Mat warppedCorners;
    double minX0, maxX0, minY0, maxY0;

    cv::Mat maskOri;
    int warppedWidth;
    int warppedHeight;

    cv::Mat localShift;
    cv::Mat warppedImg, warppedMask;
    cv::Size tempSize;

    cv::Ptr<cv::detail::SeamFinder> seamFinder = 
        cv::makePtr<cv::detail::GraphCutSeamFinder>(cv::detail::GraphCutSeamFinderBase::COST_COLOR);
    cv::UMat tempFloat;
    std::vector<cv::UMat> seamFinderImages;

    // Blender.
    cv::detail::MultiBandBlender blender;
    blender.setNumBands(5);

    double minX = 0, minY = 0, maxX = 0, maxY = 0;

    int nFiles = files.size();
    int count  = 0;

    std::stringstream ss;

    for( iterFile = files.begin(); iterFile != files.end(); iterFile++ )
    {
        cv::FileStorage fs(*iterFile, cv::FileStorage::READ);
        fs["filename"] >> tempString;

        boost::filesystem::path p(tempString);
        imgFn = basePath + "/images/" + p.filename().string();
        std::cout << count+1 << "/" << nFiles<< ": " << imgFn << std::endl;

        fs["H"] >> H;

        HVec.push_back( H );

        // std::cout << "imgFn = " << imgFn << std::endl;
        // std::cout << "H = " << H << ", type is " << H.type() << std::endl;

        tempString = p.stem().string();
        idxKFInCSVTable.push_back( std::stoi( tempString.substr( 5, tempString.length() - 5 ) ) );

        fs.release();

        img = cv::imread( imgFn, cv::IMREAD_COLOR );

        // Create 4 corner poitns for img.
        cv::Mat CPs = cv::Mat(3, 4, CV_64FC1);
        CPs.at<double>(0, 0) =          0.0; CPs.at<double>(1, 0) =          0.0; CPs.at<double>(2, 0) = 1.0;
        CPs.at<double>(0, 1) = img.cols - 1; CPs.at<double>(1, 1) =          0.0; CPs.at<double>(2, 1) = 1.0;
        CPs.at<double>(0, 2) = img.cols - 1; CPs.at<double>(1, 2) = img.rows - 1; CPs.at<double>(2, 2) = 1.0;
        CPs.at<double>(0, 3) =          0.0; CPs.at<double>(1, 3) = img.rows - 1; CPs.at<double>(2, 3) = 1.0;

        centerPoint = cv::Mat(3, 1, CV_64FC1);
        centerPoint.at<double>(0, 0) = img.cols / 2.0; centerPoint.at<double>(1, 0) = img.rows / 2.0; centerPoint.at<double>(2, 0) = 1.0;

        // std::cout << "The original corner points of img are: " << std::endl;
        // std::cout << CPs << std::endl;

        warppedCorners     = H * CPs;
        warppedCenterPoint = H * centerPoint;
        // std::cout << "warppedCenterPoint = " << warppedCenterPoint << std::endl;
        warppedCenterPointVec.push_back( warppedCenterPoint.clone() );

        // std::cout << "Warpped corner points are: " << std::endl;
        // std::cout << warppedCorners << std::endl;

        find_bounding_xy( warppedCorners, minX0, maxX0, minY0, maxY0);
        cpVec.push_back( cv::Point( minX0, minY0 ) );

        // std::cout << "minX0 = " << minX0 << ", minY0 = " << minY0 << std::endl
        //           << "maxX0 = " << maxX0 << ", maxY0 = " << maxY0 << std::endl;

        if ( 0 == count )
        {
            minX = minX0; minY = minY0; maxX = maxX0; maxY = maxY0;
        }
        else
        {
            minX = std::min( minX0, minX );
            minY = std::min( minY0, minY );
            maxX = std::max( maxX0, maxX );
            maxY = std::max( maxY0, maxY );
        }

        if ( false == skipBlending )
        {
            maskOri = cv::Mat( img.size(), CV_8UC1 );
            maskOri.setTo( cv::Scalar::all(255) );

            // Warp the images and the associated masks.
            warppedWidth  = (int)( maxX0 - minX0 ) + 1;
            warppedHeight = (int)( maxY0 - minY0 ) + 1;

            localShift = cv::Mat::eye( 3, 3, CV_64FC1 );
            localShift.at<double>(0, 2) = -minX0;
            localShift.at<double>(1, 2) = -minY0;

            tempSize = cv::Size( warppedWidth, warppedHeight );

            cv::warpPerspective(     img,  warppedImg, localShift * H, tempSize );
            cv::warpPerspective( maskOri, warppedMask, localShift * H, tempSize );

            warppedImgVec.push_back( warppedImg.clone() );
            warppedMaskVec.push_back( warppedMask.clone() );

            warppedImgUMatVec.push_back(   warppedImgVec[count].getUMat(cv::ACCESS_READ) );
            warppedMaskUMatVec.push_back( warppedMaskVec[count].getUMat(cv::ACCESS_READ) );

            warppedImgUMatVec.at(count).convertTo( tempFloat, CV_32F );
            seamFinderImages.push_back( tempFloat.clone() );
        }

        count++;

        // // Debug.
        // if ( 30 == count )
        // {
        //     break;
        // }
    }

    if ( false == skipBlending )
    {
        std::cout << "Finding the seams..." << std::endl;

        seamFinder->find( seamFinderImages, cpVec, warppedMaskUMatVec );

        // seamFinderImages.clear();

        cv::Rect blenderRect((int)minX, (int)minY, (int)( maxX - minX ) + 1, (int)( maxY - minY ) + 1);

        blender.prepare( blenderRect );

        int nImgs = warppedImgVec.size();

        cv::Mat dilatedMask, seamMask;
        cv::Mat dilateKernel = cv::Mat();
        cv::Mat warppedMaskBlender;
        cv::Mat warppedImgS;

        for ( int i = 0; i < nImgs; i++ )
        {
            std::cout << "Blender feeding image " << i << "." << std::endl;
            cv::dilate(warppedMaskUMatVec[i], dilatedMask, dilateKernel);
            cv::resize(dilatedMask, seamMask, warppedMaskUMatVec[i].size(), 0, 0, cv::INTER_LINEAR_EXACT);
            warppedMaskBlender = seamMask & warppedMaskVec[i];

            warppedImgVec[i].convertTo( warppedImgS, CV_16S );

            blender.feed( warppedImgS, warppedMaskBlender, cpVec[i] );

            // // Debug
            // if ( 8 == i || 9 == i || 10 == i )
            // {
            //     ss.str(""); ss.clear();
            //     ss << "seamMask_" << i << ".jpg";

            //     cv::imwrite(ss.str(), seamMask);

            //     ss.str(""); ss.clear();
            //     ss << "warppedMaskBlender_" << i << ".jpg";
            //     cv::imwrite(ss.str(), warppedMaskBlender);
            // }
        }

        cv::Mat blendedImage, blendedMask;
        blender.blend( blendedImage, blendedMask );

        blendedImage.convertTo( blendedImage, CV_8UC3 );

        // // Debug.
        // cv::Mat gsh = cv::Mat::eye( 3, 3, CV_64FC1 );
        // gsh.at<double>(0, 2) = -minX;
        // gsh.at<double>(1, 2) = -minY;
        // cv::Mat shiftedCenterPoint;
        // for ( int i = 0; i < nImgs; i++ )
        // {
        //     shiftedCenterPoint = gsh * warppedCenterPointVec.at(i);

        //     // std::cout << "shiftedCenterPoint = " << shiftedCenterPoint << std::endl;

        //     ss.str(""); ss.clear();
        //     ss << i;
        //     cv::putText( blendedImage, ss.str(), 
        //         cv::Point( (int)(shiftedCenterPoint.at<double>(0, 0)), (int)(shiftedCenterPoint.at<double>(1, 0)) ), 
        //         cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar::all(255), 1, 0 );
        // }

        cv::imwrite("MultiImageBlendingDirectly.jpg", blendedImage);

        cv::namedWindow("Blended", cv::WINDOW_NORMAL);
        cv::imshow("Blended", blendedImage);
        cv::waitKey();
    }
    
    cv::Mat gsh = cv::Mat::eye( 3, 3, CV_64FC1 );
    gsh.at<double>(0, 2) = -minX;
    gsh.at<double>(1, 2) = -minY;
    cv::Mat shiftedCenterPoint;
    int idxCSV;
    std::vector<cv::Point2f> pointSrc, pointDst;

    for ( int i = 0; i < idxKFInCSVTable.size(); i++ )
    {
        shiftedCenterPoint = gsh * warppedCenterPointVec.at(i);

        pointSrc.push_back( 
            cv::Point2f( shiftedCenterPoint.at<double>(0, 0), shiftedCenterPoint.at<double>(0, 1) ) );

        idxCSV = idxKFInCSVTable[i];

        pointDst.push_back( 
            cv::Point2f( table[idxCSV].east, table[idxCSV].north ) );
    }

    std::cout << "Finding GPSHomography..." << std::endl;

    cv::Mat GPSHomography;

    GPSHomography = cv::findHomography( pointSrc, pointDst, cv::RANSAC, 3, cv::noArray(), 1000, 0.995 );

    std::cout << "GPSHomography = " << std::endl;
    std::cout << GPSHomography << std::endl;

    // Calculate the upper left and the bottom right corener points.
    cv::Mat finalCornerPoints(3, 2, CV_64FC1);
    finalCornerPoints.at<double>(0, 0) =                      0.0; finalCornerPoints.at<double>(1, 0) =                      0.0; finalCornerPoints.at<double>(2, 0) = 1.0;
    finalCornerPoints.at<double>(0, 1) = (int)( maxX - minX ) + 1; finalCornerPoints.at<double>(1, 1) = (int)( maxY - minY ) + 1; finalCornerPoints.at<double>(2, 1) = 1.0;

    cv::Mat finalCornerPointsGPS( finalCornerPoints.size(), CV_64FC1 );

    finalCornerPointsGPS = GPSHomography * finalCornerPoints;

    // Shift according the east and north shifts.
    finalCornerPointsGPS.at<double>(0, 0) += -shiftEast;
    finalCornerPointsGPS.at<double>(1, 0) += -shiftNorth;
    finalCornerPointsGPS.at<double>(0, 1) += -shiftEast;
    finalCornerPointsGPS.at<double>(1, 1) += -shiftNorth;

    std::cout << "The final corner points are: " << std::endl;
    std::cout << finalCornerPointsGPS << std::endl;

    geo::GeoTransform gt(51);

    geo::Real_t lon = 0.0, lat = 0.0;

    gt.UTM_to_lat_lon( finalCornerPointsGPS.at<double>(0, 0), finalCornerPointsGPS.at<double>(1, 0),
        lon, lat );

    std::cout.precision(12);
    std::cout << " Upper left: " << PJ_R2D(lon) << "E, " << PJ_R2D(lat) << "N." << std::endl;

    gt.UTM_to_lat_lon( finalCornerPointsGPS.at<double>(0, 1), finalCornerPointsGPS.at<double>(1, 1),
        lon, lat );

    std::cout << " Bottom right: " << PJ_R2D(lon) << "E, " << PJ_R2D(lat) << "N." << std::endl;
}
