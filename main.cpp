
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <ctime>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/optional.hpp>

#include <Eigen/Dense>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/stitching/detail/blenders.hpp>

std::string imgFn0 = "../Data/space_eye.jpg";
std::string imgFn1 = "../Data/space_milkyway.jpg";

double scaleFactor = 0.2;

void show_data_type_sizes(void)
{
    std::cout << "sizeof(int) = " << sizeof( int ) << std::endl;
    std::cout << "sizeof(float) = " << sizeof( float ) << std::endl;
    std::cout << "sizeof(double) = " << sizeof( double ) << std::endl;
}

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

void test_multi_image_homography(void)
{
    // Find all the homography files at once.
    const path inputPath = "../Data/bimos/homographies";
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
        imgFn = "../Data/bimos/images/" + p.filename().string();
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

    cv::imwrite("canvas.jpg", canvas);
}

int main(void)
{
    std::cout << "Hello TestBlending!" << std::endl;

    show_data_type_sizes();

    // test_side_by_side_blending();

    // test_region_blending();

    // test_region_blending_and_overwriting();

    // test_boost_filesystem_and_opencv_yaml();

    // test_two_image_homography();

    std::clock_t begin = std::clock();
    test_multi_image_homography();
    std::clock_t end = std::clock();

    std::cout << "Time elapsed " << double( end - begin ) / CLOCKS_PER_SEC << "s." << std::endl;

    return 0;
}
