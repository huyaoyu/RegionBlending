#include <algorithm>
#include <cmath>
#include <fstream>
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
using namespace wz;

typedef float Real_t;

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

static void shift_csv_table(csv::Table_t& table, Real_t& shiftEast, Real_t& shiftNorth)
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
    Real_t& minX, Real_t& maxX, Real_t& minY, Real_t& maxY)
{
    // Clear minX, etc.
    minX = cp.at<Real_t>(0, 0); maxX = minX;
    minY = cp.at<Real_t>(1, 0); maxY = minY;

    Real_t x, y;

    // Compare.
    for ( int i = 1; i < cp.cols; i++ )
    {
        x = cp.at<Real_t>(0, i);
        if ( x < minX )
        {
            minX = x;
        }
        else if ( x > maxX )
        {
            maxX = x;
        }

        y = cp.at<Real_t>(1, i);
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

static void arrange_bounding_xy_as_corners(OutputArray _corners, 
    Real_t minX, Real_t maxX, Real_t minY, Real_t maxY)
{
    _corners.create(3, 4, CV_32FC1);
    Mat corners = _corners.getMat();
    corners.setTo(Scalar::all(1));

    corners.at<Real_t>(0, 0) = minX;
    corners.at<Real_t>(1, 0) = minY;

    corners.at<Real_t>(0, 1) = maxX;
    corners.at<Real_t>(1, 1) = minY;

    corners.at<Real_t>(0, 2) = maxX;
    corners.at<Real_t>(1, 2) = maxY;

    corners.at<Real_t>(0, 3) = minX;
    corners.at<Real_t>(1, 3) = maxY;
}

static void parse_csv(const string& gpsFile, csv::Table_t& table, Real_t& shiftEast, Real_t& shiftNorth)
{
    csv::CSVParser::parse( gpsFile, table, 0.0, 0.0, 0); convert_csv_table(table);
    shift_csv_table( table, shiftEast, shiftNorth );
    csv::CSVParser::show_table( table );
}

static void write_image(const string& fn, const Mat& img, BlendedMapper::ImageType_t t)
{
    string outputFilename = fn;
    vector<int> imgParams;
    Mat tempMat;

    switch ( t )
    {
        case BlendedMapper::JPEG:
        {
            outputFilename += ".jpg";

            imgParams.push_back( IMWRITE_JPEG_QUALITY );
            imgParams.push_back( 100 );

            imwrite(outputFilename, img, imgParams);

            break;
        }
        case BlendedMapper::PNG:
        {
            outputFilename += ".png";
            Mat tempGray;
            Mat tempMask;

            cvtColor( img, tempGray, COLOR_BGR2GRAY );
            cvtColor( img, tempMat, COLOR_BGR2BGRA );

            threshold( tempGray, tempMask, 0.0, 255.0, THRESH_BINARY_INV );
            tempMask.convertTo( tempMask, CV_8UC1 );

            tempMat.setTo( Scalar::all(0), tempMask );

            imgParams.push_back( IMWRITE_PNG_COMPRESSION );
            imgParams.push_back( 0 );

            imwrite(outputFilename, tempMat, imgParams);

            break;
        }
        default:
        {
            // Should never be here.
            stringstream ss;
            ss << "Unexpected image format code " << t;
            BOOST_THROW_EXCEPTION( MapperException() << wz::ExceptionInfoString( ss.str() ) );
        }
    }
}

void BlendedMapper::directly_blend( const string& baseDir, const vector<string>& files,
    vector<Mat>& warppedCenterPointVec, vector<int>& idxKFInCSVTable, 
    Mat& gsh, Mat& blendedCornerPoints,
    const string& blendedFilename, 
    int blenderBand, bool skipBlending, bool skipSeamFinding, OutputArray _blended )
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
    Real_t minX0, maxX0, minY0, maxY0;
    Real_t minX = 0, minY = 0, maxX = 0, maxY = 0;

    Mat maskOri;
    int warppedWidth, warppedHeight;

    Size tempSize;
    
    // Seam finder.
    Ptr<cv::detail::SeamFinder> seamFinder = 
        makePtr<cv::detail::GraphCutSeamFinder>(cv::detail::GraphCutSeamFinderBase::COST_COLOR);
    UMat tempReal_t;
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
        H.convertTo(H, CV_32FC1);

        HVec.push_back( H );

        // cout << "imgFn = " << imgFn << endl;
        // cout << "H = " << H << ", type is " << H.type() << endl;

        tempString = p.stem().string();
        idxKFInCSVTable.push_back( stoi( tempString.substr( 5, tempString.length() - 5 ) ) );

        fs.release();

        img = imread( imgFn, IMREAD_COLOR );

        // Create 4 corner poitns for img.
        Mat CPs = Mat(3, 4, CV_32FC1);
        CPs.at<Real_t>(0, 0) =          0.0; CPs.at<Real_t>(1, 0) =          0.0; CPs.at<Real_t>(2, 0) = 1.0;
        CPs.at<Real_t>(0, 1) = img.cols - 1; CPs.at<Real_t>(1, 1) =          0.0; CPs.at<Real_t>(2, 1) = 1.0;
        CPs.at<Real_t>(0, 2) = img.cols - 1; CPs.at<Real_t>(1, 2) = img.rows - 1; CPs.at<Real_t>(2, 2) = 1.0;
        CPs.at<Real_t>(0, 3) =          0.0; CPs.at<Real_t>(1, 3) = img.rows - 1; CPs.at<Real_t>(2, 3) = 1.0;

        centerPoint = Mat(3, 1, CV_32FC1);
        centerPoint.at<Real_t>(0, 0) = img.cols / 2.0; centerPoint.at<Real_t>(1, 0) = img.rows / 2.0; centerPoint.at<Real_t>(2, 0) = 1.0;

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

            localShift = Mat::eye( 3, 3, CV_32FC1 );
            localShift.at<Real_t>(0, 2) = -minX0;
            localShift.at<Real_t>(1, 2) = -minY0;

            tempSize = Size( warppedWidth, warppedHeight );

            warpPerspective(     img,  warppedImg, localShift * H, tempSize );
            warpPerspective( maskOri, warppedMask, localShift * H, tempSize );

            warppedImgVec.push_back( warppedImg.clone() );
            warppedMaskVec.push_back( warppedMask.clone() );

            if ( false == skipSeamFinding )
            {
                warppedImgUMatVec.push_back(   warppedImgVec[count].getUMat(ACCESS_READ) );
                warppedMaskUMatVec.push_back( warppedMaskVec[count].getUMat(ACCESS_READ) );

                warppedImgUMatVec.at(count).convertTo( tempReal_t, CV_32F );
                seamFinderImages.push_back( tempReal_t.clone() );
            }
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
        if ( false == skipSeamFinding )
        {
            cout << "Finding the seams..." << endl;

            seamFinder->find( seamFinderImages, cpVec, warppedMaskUMatVec );
            seamFinder.release();
            seamFinderImages.clear();
        }

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

            if ( false == skipSeamFinding )
            {
                dilate(warppedMaskUMatVec[i], dilatedMask, dilateKernel);
                resize(dilatedMask, seamMask, warppedMaskUMatVec[i].size(), 0, 0, INTER_LINEAR_EXACT);
                warppedMaskBlender = seamMask & warppedMaskVec[i];
            }
            else
            {
                warppedMaskBlender = warppedMaskVec[i];
            }

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
        // Mat gsh = Mat::eye( 3, 3, CV_32FC1 );
        // gsh.at<Real_t>(0, 2) = -minX;
        // gsh.at<Real_t>(1, 2) = -minY;
        // Mat shiftedCenterPoint;
        // for ( int i = 0; i < nImgs; i++ )
        // {
        //     shiftedCenterPoint = gsh * warppedCenterPointVec.at(i);

        //     // cout << "shiftedCenterPoint = " << shiftedCenterPoint << endl;

        //     ss.str(""); ss.clear();
        //     ss << i;
        //     putText( blendedImage, ss.str(), 
        //         Point( (int)(shiftedCenterPoint.at<Real_t>(0, 0)), (int)(shiftedCenterPoint.at<Real_t>(1, 0)) ), 
        //         FONT_HERSHEY_COMPLEX, 1, Scalar::all(255), 1, 0 );
        // }

        write_image(blendedFilename, blendedImage, mImageType);

        namedWindow("Blended", WINDOW_NORMAL);
        imshow("Blended", blendedImage);
        waitKey(1);

        _blended.assign( blendedImage );
    }

    gsh.at<Real_t>(0, 2) = -minX;
    gsh.at<Real_t>(1, 2) = -minY;

    blendedCornerPoints.at<Real_t>(0, 0) =                      0.0; blendedCornerPoints.at<Real_t>(1, 0) =                      0.0; blendedCornerPoints.at<Real_t>(2, 0) = 1.0;
    blendedCornerPoints.at<Real_t>(0, 1) = (int)( maxX - minX ) + 1; blendedCornerPoints.at<Real_t>(1, 1) =                      0.0; blendedCornerPoints.at<Real_t>(2, 1) = 1.0;
    blendedCornerPoints.at<Real_t>(0, 2) = (int)( maxX - minX ) + 1; blendedCornerPoints.at<Real_t>(1, 2) = (int)( maxY - minY ) + 1; blendedCornerPoints.at<Real_t>(2, 2) = 1.0;
    blendedCornerPoints.at<Real_t>(0, 3) =                      0.0; blendedCornerPoints.at<Real_t>(1, 3) = (int)( maxY - minY ) + 1; blendedCornerPoints.at<Real_t>(2, 3) = 1.0;
}

static void scale_points(vector<Point2f>& points, OutputArray _T, OutputArray _S)
{
    // Find the bounding values of points.
    Real_t maxX = points[0].x, minX = points[0].x;
    Real_t maxY = points[0].y, minY = points[0].y;

    Real_t x = 0.0, y = 0.0;

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
    Real_t negativeAvgX = -( minX + maxX ) / 2.0;
    Real_t negativeAvgY = -( minY + maxY ) / 2.0;
    Real_t factorX = 1.0 / ( ( maxX - minX ) / 2.0 );
    Real_t factorY = 1.0 / ( ( maxY - minY ) / 2.0 );

    // Matrix of translation.
    _T.create( 3, 3, CV_32FC1 );
    Mat T = _T.getMat();
    T.setTo(Scalar::all(0));
    T.at<Real_t>(0, 0) = 1.0;
    T.at<Real_t>(1, 1) = 1.0;
    T.at<Real_t>(2, 2) = 1.0;
    T.at<Real_t>(0, 2) = negativeAvgX;
    T.at<Real_t>(1, 2) = negativeAvgY;

    // Matrix of scale.
    _S.create( 3, 3, CV_32FC1 );
    Mat S = _S.getMat();
    S.setTo(Scalar::all(0));
    S.at<Real_t>(0, 0) = factorX;
    S.at<Real_t>(1, 1) = factorY;
    S.at<Real_t>(2, 2) = 1.0;

    for ( auto iter = points.begin(); iter != points.end(); ++iter )
    {
        (*iter).x = ( (*iter).x + negativeAvgX ) * factorX;
        (*iter).y = ( (*iter).y + negativeAvgY ) * factorY;
    }
}

static void points_flip_y( vector<Point2f>& points )
{
    for ( auto iter = points.begin(); iter != points.end(); ++iter )
    {
        (*iter).y *= -1;
    }
}

static void write_points(const vector<Point2f>& points, const string& fn)
{
    ofstream ofs;
    ofs.open( fn );

    for ( auto iter = points.begin(); iter != points.end(); ++iter )
    {
        ofs << showpos << scientific << (*iter).x << " " << (*iter).y << endl;
    }
    ofs << endl;

    ofs.close();
}

static void break_frontal_plane_homography(InputArray _H, OutputArray _FS, OutputArray _R)
{
    Mat H = _H.getMat();

    // Make a local copy of H.
    Mat LH(3, 3, CV_32FC1);

    H.copyTo(LH);

    // Normalize LH;
    LH /= LH.at<Real_t>(2, 2);

    _FS.create( 3, 3, CV_32FC1 );
    Mat FS = _FS.getMat();
    FS.setTo(Scalar::all(0));

    _R.create( 3, 3, CV_32FC1 );
    Mat R = _R.getMat();
    R.setTo(Scalar::all(0));

    Real_t f = sqrt( 
        fabs( LH.at<Real_t>(0, 0) * LH.at<Real_t>(1, 1) ) + 
        fabs( LH.at<Real_t>(0, 1) * LH.at<Real_t>(1, 0) )  );
    
    FS.at<Real_t>(0, 0) = f; FS.at<Real_t>(0, 2) = LH.at<Real_t>(0, 2);
    FS.at<Real_t>(1, 1) = f; FS.at<Real_t>(1, 2) = LH.at<Real_t>(1, 2);
    FS.at<Real_t>(2, 2) = 1.0;

    // Mat FSI( 3, 3, CV_32FC1);
    // FSI.setTo( Scalar::all(0) );

    // FSI.at<Real_t>(0, 0) = 1/f; FSI.at<Real_t>(0, 2) = -LH.at<Real_t>(0, 2) / f;
    // FSI.at<Real_t>(1, 1) = 1/f; FSI.at<Real_t>(1, 2) = -LH.at<Real_t>(1, 2) / f;
    // FSI.at<Real_t>(2, 2) = 1.0;


    R.at<Real_t>(0, 0) = LH.at<Real_t>(0, 0) / f; R.at<Real_t>(0, 1) = LH.at<Real_t>(0, 1) / f;
    R.at<Real_t>(1, 0) = LH.at<Real_t>(1, 0) / f; R.at<Real_t>(1, 1) = LH.at<Real_t>(1, 1) / f;
    R.at<Real_t>(2, 2) = 1.0;

    // Mat temp = FSI * LH;
    // temp.copyTo( R );
}

template<typename _T> 
static void write_mat_single_channel(const string& fn, const Mat& m)
{
    using namespace wz;

    // Assuming the single channel Mat object.
    if ( 1 != m.channels() )
    {
        EXCEPTION_BASE( "write_mat_single_channel only accepts single channel arrays" )
    }

    ofstream ofs;
    ofs.open( fn );

    const _T* p = NULL;

    // Local copy of m.
    for ( int i = 0; i < m.rows; ++i )
    {
        p = m.ptr<_T>( i );
        for ( int j = 0; j < m.cols; ++j )
        {
            ofs << showpos << scientific << *(p + j) << " ";
        }
        ofs << endl;
    }

    ofs.close();
}

template <typename _T> 
static void normalize_points(Mat& points)
{
    // Check if rows of points is 3.
    if ( 3 != points.rows )
    {
        EXCEPTION_BASE("Rows of points must be three.");
    }

    // Check if chennals is 1.
    if ( 1 != points.channels() )
    {
        EXCEPTION_BASE("Chennals of points must be 1.")
    }

    for ( int i = 0; i < points.cols; ++i )
    {
        points.at<_T>(0, i) /= points.at<_T>(2, i);
        points.at<_T>(1, i) /= points.at<_T>(2, i);
        points.at<_T>(2, i) /= points.at<_T>(2, i);
    }
}

static void find_final_cornerpoints_GPS( 
    const vector<Mat>& warppedCenterPointVec, 
    const vector<int>& idxKFInCSVTable, const csv::Table_t& table,
    const Mat& gsh, const Mat& blendedCornerPoints, 
    Real_t shiftEast, Real_t shiftNorth,
    OutputArray _finalCornerPointsGPS, 
    OutputArray _finalRotateH,
    const string& outDir )
{
    Mat shiftedCenterPoint;
    int idxCSV;
    vector<Point2f> pointSrc, pointDst;

    for ( int i = 0; i < warppedCenterPointVec.size(); i++ )
    {
        shiftedCenterPoint = gsh * warppedCenterPointVec.at(i);

        // What the object is in reality.
        pointSrc.push_back( 
            Point2f( shiftedCenterPoint.at<Real_t>(0, 0), shiftedCenterPoint.at<Real_t>(0, 1) ) );

        idxCSV = idxKFInCSVTable[i];

        // What you get in the filming device.
        pointDst.push_back( 
            Point2f( table[idxCSV].east, table[idxCSV].north ) );
    }

    // Flip the y-axis of the dst points.
    // This is due to the fact that the pixel coordinate and the UTM coordinate have
    // opposite z-axis direction.
    points_flip_y(pointDst);

    // Scale the src and dst points.
    Mat TSrc, TDst; // Matrices of translation.
    Mat SSrc, SDst; // Matrices of rotation.

    scale_points( pointSrc, TSrc, SSrc );
    scale_points( pointDst, TDst, SDst );

    write_points( pointSrc, outDir + "/pointSrc.dat" );
    write_points( pointDst, outDir + "/pointDst.dat" );

    cout << "TSrc = " << endl << TSrc << endl;
    cout << "SSrc = " << endl << SSrc << endl;
    cout << "TDst = " << endl << TDst << endl;
    cout << "SDst = " << endl << SDst << endl;

    Mat TDstInv = Mat::eye(3, 3, CV_32FC1);
    TDstInv.at<Real_t>(0, 2) = -TDst.at<Real_t>(0, 2);
    TDstInv.at<Real_t>(1, 2) = -TDst.at<Real_t>(1, 2);

    Mat SDstInv = Mat::eye(3, 3, CV_32FC1);
    SDstInv.at<Real_t>(0, 0) = 1.0 / SDst.at<Real_t>(0, 0);
    SDstInv.at<Real_t>(1, 1) = 1.0 / SDst.at<Real_t>(1, 1);

    cout << "Finding GPSHomography..." << endl;

    Mat GPSHomography, HomoFS, HomoR, HomoFlip;

    GPSHomography = findHomography( pointSrc, pointDst, RANSAC, 2, noArray(), 10000, 0.999 );
    GPSHomography.convertTo( GPSHomography, CV_32FC1 );

    write_mat_single_channel<Real_t>(outDir + "/GPSHomography.dat", GPSHomography);

    break_frontal_plane_homography( GPSHomography, HomoFS, HomoR );
    write_mat_single_channel<Real_t>(outDir + "/ComposedHomography.dat", HomoFS*HomoR);

    cout << "TDstInv.type() = " << TDstInv.type() << endl;
    cout << "SDstInv.type() = " << SDstInv.type() << endl;
    cout << "SSrc.type()    = " << SSrc.type() << endl;
    cout << "TSrc.type()    = " << TSrc.type() << endl;
    cout << "GPSHomography.type() = " << GPSHomography.type() << endl;

    GPSHomography = TDstInv * SDstInv * GPSHomography * SSrc * TSrc;

    GPSHomography /= GPSHomography.at<Real_t>(2, 2);

    cout << "GPSHomography = " << endl;
    cout << GPSHomography << endl;

    // Break GPSHomography into two separate homography matrices.

    break_frontal_plane_homography( GPSHomography, HomoFS, HomoR );
    // write_mat_single_channel<Real_t>("ComposedHomography.dat", HomoFS*HomoR);

    HomoFlip = Mat::eye(3, 3, CV_32FC1);
    HomoFlip.at<Real_t>(1, 1) = -1.0;

    Real_t rotMinX = 0.0, rotMaxX = 0.0, rotMinY = 0.0, rotMaxY = 0.0;

    _finalCornerPointsGPS.create(3, 4, CV_32FC1);
    Mat finalCornerPointsGPS = _finalCornerPointsGPS.getMat();

    // // Calculate new bounding points based on HomoR.
    // Mat rotatedCornerPoints = HomoR * blendedCornerPoints;
    // // normalize_points<Real_t>(rotatedCornerPoints);

    // find_bounding_xy(rotatedCornerPoints, rotMinX, rotMaxX, rotMinY, rotMaxY);
    // // Re-use rotatedCornerPoints.
    // arrange_bounding_xy_as_corners(rotatedCornerPoints, rotMinX, rotMaxX, rotMinY, rotMaxY);

    // finalCornerPointsGPS = HomoFlip * HomoFS * rotatedCornerPoints;
    // // normalize_points<Real_t>(finalCornerPointsGPS); // Not necessary.

    // finalCornerPointsGPS = HomoFlip * GPSHomography * blendedCornerPoints;
    finalCornerPointsGPS = HomoFS * HomoR * blendedCornerPoints;
    normalize_points<Real_t>(finalCornerPointsGPS);
    cout << "Final coner points before flipping: " << endl;
    cout << finalCornerPointsGPS << endl;
    finalCornerPointsGPS = HomoFlip * finalCornerPointsGPS;
    cout << "Final coner points arter flipping: " << endl;
    cout << finalCornerPointsGPS << endl;

    find_bounding_xy( finalCornerPointsGPS, rotMinX, rotMaxX, rotMinY, rotMaxY );
    arrange_bounding_xy_as_corners( finalCornerPointsGPS, rotMinX, rotMaxX, rotMinY, rotMaxY );

    // Shift according the east and north shifts.
    finalCornerPointsGPS.at<Real_t>(0, 0) += -shiftEast;
    finalCornerPointsGPS.at<Real_t>(1, 0) += -shiftNorth;
    finalCornerPointsGPS.at<Real_t>(0, 1) += -shiftEast;
    finalCornerPointsGPS.at<Real_t>(1, 1) += -shiftNorth;
    finalCornerPointsGPS.at<Real_t>(0, 2) += -shiftEast;
    finalCornerPointsGPS.at<Real_t>(1, 2) += -shiftNorth;
    finalCornerPointsGPS.at<Real_t>(0, 3) += -shiftEast;
    finalCornerPointsGPS.at<Real_t>(1, 3) += -shiftNorth;

    cout << "The final corner points are: " << endl;
    cout << finalCornerPointsGPS << endl;

    // Write upper left and bottom right corners into file.
    ofstream ofsGeoTiff;
    ofsGeoTiff.open( outDir + "/GeoTiffInput.dat" );

    geo::GeoTransform gt(51);

    geo::Real_t lon = 0.0, lat = 0.0;

    cout.precision(12);
    ofsGeoTiff.precision(12);

    gt.UTM_to_lat_lon( finalCornerPointsGPS.at<Real_t>(0, 3), finalCornerPointsGPS.at<Real_t>(1, 3),
        lon, lat );

    cout << fixed << " Upper left (E, N): " << PJ_R2D(lon) << ", " << PJ_R2D(lat) << "." << endl;
    ofsGeoTiff << fixed << PJ_R2D(lon) << " " << PJ_R2D(lat) << endl;

    gt.UTM_to_lat_lon( finalCornerPointsGPS.at<Real_t>(0, 2), finalCornerPointsGPS.at<Real_t>(1, 2),
        lon, lat );

    cout << fixed << " Upper right (E, N): " << PJ_R2D(lon) << ", " << PJ_R2D(lat) << "." << endl;

    gt.UTM_to_lat_lon( finalCornerPointsGPS.at<Real_t>(0, 1), finalCornerPointsGPS.at<Real_t>(1, 1),
        lon, lat );

    cout << fixed << " Bottom right (E, N): " << PJ_R2D(lon) << ", " << PJ_R2D(lat) << "." << endl;
    ofsGeoTiff << fixed << PJ_R2D(lon) << " " << PJ_R2D(lat) << endl;

    gt.UTM_to_lat_lon( finalCornerPointsGPS.at<Real_t>(0, 0), finalCornerPointsGPS.at<Real_t>(1, 0),
        lon, lat );

    cout << fixed << " Bottom left (E, N): " << PJ_R2D(lon) << ", " << PJ_R2D(lat) << "." << endl;

    ofsGeoTiff.close();

    // Final rotation homograph.
    _finalRotateH.create(3, 3, CV_32FC1);
    Mat finalRotateH = _finalRotateH.getMat();
    HomoR.copyTo( finalRotateH );
}

static void warp_and_shift(const Mat& src, const Mat& H, OutputArray _dst)
{
    // Create 4 corner poitns for src.
    Mat CPs = Mat(3, 4, CV_32FC1);
    CPs.at<Real_t>(0, 0) =          0.0; CPs.at<Real_t>(1, 0) =          0.0; CPs.at<Real_t>(2, 0) = 1.0;
    CPs.at<Real_t>(0, 1) = src.cols - 1; CPs.at<Real_t>(1, 1) =          0.0; CPs.at<Real_t>(2, 1) = 1.0;
    CPs.at<Real_t>(0, 2) = src.cols - 1; CPs.at<Real_t>(1, 2) = src.rows - 1; CPs.at<Real_t>(2, 2) = 1.0;
    CPs.at<Real_t>(0, 3) =          0.0; CPs.at<Real_t>(1, 3) = src.rows - 1; CPs.at<Real_t>(2, 3) = 1.0;

    // Warp these 4 corner points.
    Mat warppedCorners = H * CPs;
    normalize_points<Real_t>(warppedCorners);

    // Debug.
    cout << "warp_and_shift: warppedCorners: " << endl << warppedCorners << endl;

    // Find the bounding x and y.
    Real_t minX = 0.0, maxX = 0.0, minY = 0.0, maxY = 0.0;
    find_bounding_xy( warppedCorners, minX, maxX, minY, maxY );

    // New H.
    Mat newH( H.size(), H.type() );
    H.copyTo( newH );

    newH.at<Real_t>(0, 2) -= minX;
    newH.at<Real_t>(1, 2) -= minY;

    _dst.create( maxY - minY + 1, maxX - minX + 1, src.type() );
    Mat dst = _dst.getMat();

    warpPerspective( src, dst, newH, Size( dst.cols, dst.rows ));
}

void BlendedMapper::multi_image_homography_direct_blending(
        const string& baseDir, const string& outDir, const string& gpsFile,
        bool skipBlending, bool skipSeamFinding)
{
    // Parse the CSV file first.
    csv::Table_t table;
    Real_t shiftEast = 0.0, shiftNorth = 0.0;

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
    const path inputPath   = baseDir + "/homographies";
    const path filePattern = "*.yml";
    vector<string> files;
    find_files(inputPath, ".yml", files);

    vector<Mat> warppedCenterPointVec;
    vector<int> idxKFInCSVTable;
    Mat gsh = Mat::eye( 3, 3, CV_32FC1 );
    
    // Calculate the upper left and the bottom right corener points.
    Mat blendedCornerPoints(3, 4, CV_32FC1);

    Mat blended;
    string blendedFn = outDir + "/MultiImageBlendingDirectly";

    directly_blend( baseDir, files,
        warppedCenterPointVec, idxKFInCSVTable, 
        gsh, blendedCornerPoints,
        blendedFn, 
        mBlenderBand, skipBlending, skipSeamFinding, blended );
    
    // Final corner points.
    Mat finalCornerPointsGPS, finalRotateH;

    find_final_cornerpoints_GPS( 
        warppedCenterPointVec, idxKFInCSVTable, table,
        gsh, blendedCornerPoints, shiftEast, shiftNorth,
        finalCornerPointsGPS, finalRotateH, outDir );

    // Final rotation.
    cout << "finalRotateH = " << endl << finalRotateH << endl;
    Mat finalMat;
    string finalMatFn = outDir + "/finalMat";

    if ( false == skipBlending )
    {
        warp_and_shift( blended, finalRotateH, finalMat );

        // Write file.
        write_image(finalMatFn, finalMat, mImageType);
    }
    else
    {
        // Read the image.
        blended = imread(blendedFn + ".png", IMREAD_COLOR);
        
        warp_and_shift( blended, finalRotateH, finalMat );

        // Write file.
        write_image(finalMatFn, finalMat, mImageType);
    }
}
