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

#include "Mapper.hpp"

using namespace boost;
using namespace boost::filesystem;
using namespace cv;
using namespace mapper;
using namespace std;

BlendedMapper::BlendedMapper()
: mBlenderBand(5),
  mImageType(PNG),
  mFlagDebug(false), mDebugCount(0)
{

}

BlendedMapper::~BlendedMapper()
{

}

void BlendedMapper::set_blender_band(int b)
{
    mBlenderBand = b;
}

int BlendedMapper::get_blender_band(void)
{
    return mBlenderBand;
}

void BlendedMapper::set_output_image_type(ImageType_t t)
{
    mImageType = t;
}

BlendedMapper::ImageType_t BlendedMapper::get_output_image_type(void)
{
    return mImageType;
}

void BlendedMapper::enable_debug(void)
{
    mFlagDebug = true;
}

void BlendedMapper::disable_debug(void)
{
    mFlagDebug = false;
}

void BlendedMapper::set_debug_count(int c)
{
    mDebugCount = c;
}

static void convert_csv_table(csv::Table_t& table)
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

static void shift_csv_table(csv::Table_t& table, double& shiftEast, double& shiftNorth)
{
    shiftEast  = -table[0].east;
    shiftNorth = -table[0].north;

    for ( auto iter = table.begin(); iter != table.end(); ++iter )
    {
        (*iter).east  += shiftEast;
        (*iter).north += shiftNorth;
    }
}

/*
* This is found at
* https://stackoverflow.com/questions/11140483/how-to-get-list-of-files-with-a-specific-extension-in-a-given-folder
*/
static void find_files(const boost::filesystem::path& root, 
    const std::string& ext, std::vector<std::string>& ret)
{
    if(!exists(root) || !is_directory(root)) return;

    directory_iterator it(root);
    directory_iterator endit;

    while(it != endit)
    {
        if(is_regular_file(*it) && it->path().extension() == ext)
        {
            ret.push_back( 
                root.string() + "/" + it->path().filename().string() );
        }

        ++it;
    }

    // This is found at
    // https://stackoverflow.com/questions/18723984/sorting-files-with-boost-filesystem
    sort( ret.begin(), ret.end() );
}

static void find_bounding_xy(const Mat& cp, 
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

static void parse_csv(const string& gpsFile, csv::Table_t& table, double& shiftEast, double& shiftNorth)
{
    csv::CSVParser::parse( gpsFile, table, 0.0, 0.0, 0); convert_csv_table(table);
    shift_csv_table( table, shiftEast, shiftNorth );
    csv::CSVParser::show_table( table );
}

void BlendedMapper::directly_blend( const string& baseDir, const vector<string>& files,
    vector<Mat>& warppedCenterPointVec, vector<int>& idxKFInCSVTable, 
    Mat& gsh, Mat& finalCornerPoints,
    const string& blendedFilename, 
    int blenderBand, bool skipBlending )
{
    // Read the homography matrices.
    string imgFn, tempString;
    Mat H;
    Mat img;
    Mat centerPoint, warppedCenterPoint;

    vector<Point> cpVec;
    vector<Mat>   warppedImgVec;
    vector<Mat>   warppedMaskVec;
    vector<UMat>  warppedImgUMatVec;
    vector<UMat>  warppedMaskUMatVec;
    vector<Mat>   HVec;

    Mat warppedImg, warppedMask;
    Mat warppedCorners;
    Mat localShift;
    double minX0, maxX0, minY0, maxY0;
    double minX = 0, minY = 0, maxX = 0, maxY = 0;

    Mat maskOri;
    int warppedWidth, warppedHeight;

    Size tempSize;
    
    // Seam finder.
    Ptr<cv::detail::SeamFinder> seamFinder = 
        makePtr<cv::detail::GraphCutSeamFinder>(cv::detail::GraphCutSeamFinderBase::COST_COLOR);
    UMat tempFloat;
    vector<UMat> seamFinderImages;

    // Blender.
    cv::detail::MultiBandBlender blender;
    blender.setNumBands(blenderBand);

    // Loop.
    int nFiles = files.size();
    int count  = 0;
    stringstream ss;

    for( auto iterFile = files.begin(); iterFile != files.end(); iterFile++ )
    {
        FileStorage fs(*iterFile, FileStorage::READ);
        fs["filename"] >> tempString;

        boost::filesystem::path p(tempString);
        imgFn = baseDir + "/images/" + p.filename().string();
        cout << count+1 << "/" << nFiles<< ": " << imgFn << endl;

        fs["H"] >> H;

        HVec.push_back( H );

        // cout << "imgFn = " << imgFn << endl;
        // cout << "H = " << H << ", type is " << H.type() << endl;

        tempString = p.stem().string();
        idxKFInCSVTable.push_back( stoi( tempString.substr( 5, tempString.length() - 5 ) ) );

        fs.release();

        img = imread( imgFn, IMREAD_COLOR );

        // Create 4 corner poitns for img.
        Mat CPs = Mat(3, 4, CV_64FC1);
        CPs.at<double>(0, 0) =          0.0; CPs.at<double>(1, 0) =          0.0; CPs.at<double>(2, 0) = 1.0;
        CPs.at<double>(0, 1) = img.cols - 1; CPs.at<double>(1, 1) =          0.0; CPs.at<double>(2, 1) = 1.0;
        CPs.at<double>(0, 2) = img.cols - 1; CPs.at<double>(1, 2) = img.rows - 1; CPs.at<double>(2, 2) = 1.0;
        CPs.at<double>(0, 3) =          0.0; CPs.at<double>(1, 3) = img.rows - 1; CPs.at<double>(2, 3) = 1.0;

        centerPoint = Mat(3, 1, CV_64FC1);
        centerPoint.at<double>(0, 0) = img.cols / 2.0; centerPoint.at<double>(1, 0) = img.rows / 2.0; centerPoint.at<double>(2, 0) = 1.0;

        // cout << "The original corner points of img are: " << endl;
        // cout << CPs << endl;

        warppedCorners     = H * CPs;
        warppedCenterPoint = H * centerPoint;
        // cout << "warppedCenterPoint = " << warppedCenterPoint << endl;
        warppedCenterPointVec.push_back( warppedCenterPoint.clone() );

        // cout << "Warpped corner points are: " << endl;
        // cout << warppedCorners << endl;

        find_bounding_xy( warppedCorners, minX0, maxX0, minY0, maxY0);
        cpVec.push_back( Point( minX0, minY0 ) );

        // cout << "minX0 = " << minX0 << ", minY0 = " << minY0 << endl
        //           << "maxX0 = " << maxX0 << ", maxY0 = " << maxY0 << endl;

        if ( 0 == count )
        {
            minX = minX0; minY = minY0; maxX = maxX0; maxY = maxY0;
        }
        else
        {
            minX = min( minX0, minX );
            minY = min( minY0, minY );
            maxX = max( maxX0, maxX );
            maxY = max( maxY0, maxY );
        }

        if ( false == skipBlending )
        {
            maskOri = Mat( img.size(), CV_8UC1 );
            maskOri.setTo( Scalar::all(255) );

            // Warp the images and the associated masks.
            warppedWidth  = (int)( maxX0 - minX0 ) + 1;
            warppedHeight = (int)( maxY0 - minY0 ) + 1;

            localShift = Mat::eye( 3, 3, CV_64FC1 );
            localShift.at<double>(0, 2) = -minX0;
            localShift.at<double>(1, 2) = -minY0;

            tempSize = Size( warppedWidth, warppedHeight );

            warpPerspective(     img,  warppedImg, localShift * H, tempSize );
            warpPerspective( maskOri, warppedMask, localShift * H, tempSize );

            warppedImgVec.push_back( warppedImg.clone() );
            warppedMaskVec.push_back( warppedMask.clone() );

            warppedImgUMatVec.push_back(   warppedImgVec[count].getUMat(ACCESS_READ) );
            warppedMaskUMatVec.push_back( warppedMaskVec[count].getUMat(ACCESS_READ) );

            warppedImgUMatVec.at(count).convertTo( tempFloat, CV_32F );
            seamFinderImages.push_back( tempFloat.clone() );
        }

        count++;

        // Debug.
        if ( true == mFlagDebug )
        {
            if ( mDebugCount == count )
            {
                cout << "Stop due to the enabled debug flag." << endl;
                break;
            }
        }
    }

    if ( false == skipBlending )
    {
        cout << "Finding the seams..." << endl;

        seamFinder->find( seamFinderImages, cpVec, warppedMaskUMatVec );

        // seamFinderImages.clear();

        Rect blenderRect((int)minX, (int)minY, (int)( maxX - minX ) + 1, (int)( maxY - minY ) + 1);

        blender.prepare( blenderRect );

        int nImgs = warppedImgVec.size();

        Mat dilatedMask, seamMask;
        Mat dilateKernel = Mat();
        Mat warppedMaskBlender;
        Mat warppedImgS;

        for ( int i = 0; i < nImgs; i++ )
        {
            cout << "Blender feeding image " << i << "." << endl;
            dilate(warppedMaskUMatVec[i], dilatedMask, dilateKernel);
            resize(dilatedMask, seamMask, warppedMaskUMatVec[i].size(), 0, 0, INTER_LINEAR_EXACT);
            warppedMaskBlender = seamMask & warppedMaskVec[i];

            warppedImgVec[i].convertTo( warppedImgS, CV_16S );

            blender.feed( warppedImgS, warppedMaskBlender, cpVec[i] );

            // // Debug
            // if ( 8 == i || 9 == i || 10 == i )
            // {
            //     ss.str(""); ss.clear();
            //     ss << "seamMask_" << i << ".jpg";

            //     imwrite(ss.str(), seamMask);

            //     ss.str(""); ss.clear();
            //     ss << "warppedMaskBlender_" << i << ".jpg";
            //     imwrite(ss.str(), warppedMaskBlender);
            // }
        }

        Mat blendedImage, blendedMask;
        blender.blend( blendedImage, blendedMask );

        blendedImage.convertTo( blendedImage, CV_8UC3 );

        // // Debug.
        // Mat gsh = Mat::eye( 3, 3, CV_64FC1 );
        // gsh.at<double>(0, 2) = -minX;
        // gsh.at<double>(1, 2) = -minY;
        // Mat shiftedCenterPoint;
        // for ( int i = 0; i < nImgs; i++ )
        // {
        //     shiftedCenterPoint = gsh * warppedCenterPointVec.at(i);

        //     // cout << "shiftedCenterPoint = " << shiftedCenterPoint << endl;

        //     ss.str(""); ss.clear();
        //     ss << i;
        //     putText( blendedImage, ss.str(), 
        //         Point( (int)(shiftedCenterPoint.at<double>(0, 0)), (int)(shiftedCenterPoint.at<double>(1, 0)) ), 
        //         FONT_HERSHEY_COMPLEX, 1, Scalar::all(255), 1, 0 );
        // }

        string outputFilename = blendedFilename;
        vector<int> imgParams;
        Mat tempMat;

        switch ( mImageType )
        {
            case JPEG:
            {
                outputFilename += ".jpg";

                imgParams.push_back( IMWRITE_JPEG_QUALITY );
                imgParams.push_back( 100 );

                break;
            }
            case PNG:
            {
                outputFilename += ".png";
                Mat tempGray;
                Mat tempMask;

                cvtColor( blendedImage, tempGray, COLOR_BGR2GRAY );
                cvtColor( blendedImage, tempMat, COLOR_BGR2BGRA );

                threshold( tempGray, tempMask, 0.0, 255.0, THRESH_BINARY_INV );
                tempMask.convertTo( tempMask, CV_8UC1 );

                tempMat.setTo( Scalar::all(0), tempMask );

                blendedImage = tempMat;

                imgParams.push_back( IMWRITE_PNG_COMPRESSION );
                imgParams.push_back( 0 );

                break;
            }
            default:
            {
                // Should never be here.
                stringstream ss;
                ss << "Unexpected image format code " << mImageType;
                BOOST_THROW_EXCEPTION( MapperException() << wz::ExceptionInfoString( ss.str() ) );
            }
        }

        imwrite(outputFilename, blendedImage, imgParams);

        namedWindow("Blended", WINDOW_NORMAL);
        imshow("Blended", blendedImage);
        waitKey(1);
    }

    gsh.at<double>(0, 2) = -minX;
    gsh.at<double>(1, 2) = -minY;

    finalCornerPoints.at<double>(0, 0) =                      0.0; finalCornerPoints.at<double>(1, 0) =                      0.0; finalCornerPoints.at<double>(2, 0) = 1.0;
    finalCornerPoints.at<double>(0, 1) = (int)( maxX - minX ) + 1; finalCornerPoints.at<double>(1, 1) = (int)( maxY - minY ) + 1; finalCornerPoints.at<double>(2, 1) = 1.0;
}

static void scale_points(vector<Point2f>& points, OutputArray _T, OutputArray _S)
{
    // Find the bounding values of points.
    float maxX = points[0].x, minX = points[0].x;
    float maxY = points[0].y, minY = points[0].y;

    float x = 0.0, y = 0.0;

    for ( auto iter = points.begin(); iter != points.end(); ++iter )
    {
        x = (*iter).x;
        y = (*iter).y;

        if ( x < minX )
        {
            minX = x;
        }
        else if ( x > maxX )
        {
            maxX = x;
        }

        if ( y < minY )
        {
            minY = y;
        } 
        else if ( y > maxY)
        {
            maxY = y;
        }
    }

    // Calculate the average values and the scale factors.
    float negativeAvgX = -( minX + maxX ) / 2.0;
    float negativeAvgY = -( minY + maxY ) / 2.0;
    float factorX = 1.0 / ( ( maxX - minX ) / 2.0 );
    float factorY = 1.0 / ( ( maxY - minY ) / 2.0 );

    // Matrix of translation.
    _T.create( 3, 3, CV_32FC1 );
    Mat T = _T.getMat();
    T.setTo(Scalar::all(0));
    T.at<float>(0, 0) = 1.0;
    T.at<float>(1, 1) = 1.0;
    T.at<float>(2, 2) = 1.0;
    T.at<float>(0, 2) = negativeAvgX;
    T.at<float>(1, 2) = negativeAvgY;

    // Matrix of scale.
    _S.create( 3, 3, CV_32FC1 );
    Mat S = _S.getMat();
    S.setTo(Scalar::all(0));
    S.at<float>(0, 0) = factorX;
    S.at<float>(1, 1) = factorY;
    S.at<float>(2, 2) = 1.0;

    for ( auto iter = points.begin(); iter != points.end(); ++iter )
    {
        (*iter).x = ( (*iter).x + negativeAvgX ) * factorX;
        (*iter).y = ( (*iter).y + negativeAvgY ) * factorY;
    }
}

static void find_final_cornerpoints_GPS( 
    const vector<Mat>& warppedCenterPointVec, 
    const vector<int>& idxKFInCSVTable, const csv::Table_t& table,
    const Mat& gsh, const Mat& finalCornerPoints, 
    double shiftEast, double shiftNorth,
    Mat& finalCornerPointsGPS )
{
    Mat shiftedCenterPoint;
    int idxCSV;
    vector<Point2f> pointSrc, pointDst;

    for ( int i = 0; i < warppedCenterPointVec.size(); i++ )
    {
        shiftedCenterPoint = gsh * warppedCenterPointVec.at(i);

        // What the object is in reality.
        pointSrc.push_back( 
            Point2f( shiftedCenterPoint.at<double>(0, 0), shiftedCenterPoint.at<double>(0, 1) ) );

        idxCSV = idxKFInCSVTable[i];

        // What you get in the filming device.
        pointDst.push_back( 
            Point2f( table[idxCSV].east, table[idxCSV].north ) );
    }

    // Scale the src and dst points.
    Mat TSrc, TDst; // Matrices of translation.
    Mat SSrc, SDst; // Matrices of rotation.

    scale_points( pointSrc, TSrc, SSrc );
    scale_points( pointDst, TDst, SDst );

    cout << "TSrc = " << endl << TSrc << endl;
    cout << "SSrc = " << endl << SSrc << endl;
    cout << "TDst = " << endl << TDst << endl;
    cout << "SDst = " << endl << SDst << endl;

    Mat TDstInv = Mat::eye(3, 3, CV_32FC1);
    TDstInv.at<float>(0, 2) = -TDst.at<float>(0, 2);
    TDstInv.at<float>(1, 2) = -TDst.at<float>(1, 2);

    Mat SDstInv = Mat::eye(3, 3, CV_32FC1);
    SDstInv.at<float>(0, 0) = 1.0 / SDst.at<float>(0, 0);
    SDstInv.at<float>(1, 1) = 1.0 / SDst.at<float>(1, 1);

    cout << "Finding GPSHomography..." << endl;

    Mat GPSHomography;

    GPSHomography = findHomography( pointSrc, pointDst, RANSAC, 3, noArray(), 1000, 0.995 );
    GPSHomography.convertTo( GPSHomography, CV_32FC1 );

    cout << "TDstInv.type() = " << TDstInv.type() << endl;
    cout << "SDstInv.type() = " << SDstInv.type() << endl;
    cout << "SSrc.type()    = " << SSrc.type() << endl;
    cout << "TSrc.type()    = " << TSrc.type() << endl;
    cout << "GPSHomography.type() = " << GPSHomography.type() << endl;

    GPSHomography = TDstInv * SDstInv * GPSHomography * SSrc * TSrc;

    GPSHomography.convertTo( GPSHomography, CV_64FC1 );

    cout << "GPSHomography = " << endl;
    cout << GPSHomography << endl;

    finalCornerPointsGPS = GPSHomography * finalCornerPoints;

    // Shift according the east and north shifts.
    finalCornerPointsGPS.at<double>(0, 0) += -shiftEast;
    finalCornerPointsGPS.at<double>(1, 0) += -shiftNorth;
    finalCornerPointsGPS.at<double>(0, 1) += -shiftEast;
    finalCornerPointsGPS.at<double>(1, 1) += -shiftNorth;

    cout << "The final corner points are: " << endl;
    cout << finalCornerPointsGPS << endl;

    geo::GeoTransform gt(51);

    geo::Real_t lon = 0.0, lat = 0.0;

    gt.UTM_to_lat_lon( finalCornerPointsGPS.at<double>(0, 0), finalCornerPointsGPS.at<double>(1, 0),
        lon, lat );

    cout.precision(12);
    cout << " Upper left: " << PJ_R2D(lon) << "E, " << PJ_R2D(lat) << "N." << endl;

    gt.UTM_to_lat_lon( finalCornerPointsGPS.at<double>(0, 1), finalCornerPointsGPS.at<double>(1, 1),
        lon, lat );

    cout << " Bottom right: " << PJ_R2D(lon) << "E, " << PJ_R2D(lat) << "N." << endl;
}

void BlendedMapper::multi_image_homography_direct_blending(
        const string& baseDir, const string& homoDir, const string& gpsFile,
        bool skipBlending)
{
    // Parse the CSV file first.
    csv::Table_t table;
    double shiftEast = 0.0, shiftNorth = 0.0;

    try
    {
        parse_csv(gpsFile, table, shiftEast, shiftNorth);
    }
    catch ( csv::CSVException& exp )
    {
        cout << "Parsing CSV file failed with exception. " << std::endl;
        if ( string const * expInfoString = boost::get_error_info<wz::ExceptionInfoString>(exp) )
        {
            cout << *expInfoString << std::endl;
        }
        
        return;
    }
    

    cout << gpsFile << " parsed." << endl;

    // Find all the homography files at once.
    const path inputPath   = homoDir;
    const path filePattern = "*.yml";
    vector<string> files;
    find_files(inputPath, ".yml", files);

    vector<Mat> warppedCenterPointVec;
    vector<int> idxKFInCSVTable;
    Mat gsh = Mat::eye( 3, 3, CV_64FC1 );
    
    // Calculate the upper left and the bottom right corener points.
    Mat finalCornerPoints(3, 2, CV_64FC1);

    directly_blend( baseDir, files,
        warppedCenterPointVec, idxKFInCSVTable, 
        gsh, finalCornerPoints,
        "MultiImageBlendingDirectly", 
        mBlenderBand, skipBlending );
    
    // Final corner points.
    Mat finalCornerPointsGPS( finalCornerPoints.size(), CV_64FC1 );
    find_final_cornerpoints_GPS( 
        warppedCenterPointVec, idxKFInCSVTable, table,
        gsh, finalCornerPoints, shiftEast, shiftNorth,
        finalCornerPointsGPS );
}
