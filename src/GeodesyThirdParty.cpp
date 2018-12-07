#include <iostream>
#include <sstream>
#include <string>

#include "GeodesyThirdParty.hpp"

using namespace std;
using namespace geo;

PJ_CONTEXT* GeoTransform::mPJContext = NULL;

GeoTransform::GeoTransform(int zone)
: mZone(zone)
{
    if ( NULL != mPJContext )
    {
        cout << "Only one PROJ4 context is allowed currently." << endl;
    }
    else
    {
        mPJContext = proj_context_create();

        stringstream ss;
        ss << "+proj=utm +zone=" << mZone << " +ellps=WGS84";
        mProjectorString = ss.str();

        mProjector = proj_create( mPJContext, mProjectorString.c_str() );

        if ( 0 == mProjector )
        {
            cout << "Projector creation failed." << endl;
        }
    }
}

GeoTransform::~GeoTransform()
{
    if ( NULL != mPJContext )
    {
        proj_context_destroy(mPJContext);
        mPJContext = NULL;
    }

    proj_destroy(mProjector);
}

int GeoTransform::get_zone(void)
{
    return mZone;
}

string& GeoTransform::get_projector_string(void)
{
    return mProjectorString;
}

void GeoTransform::lat_lon_to_UTM(Real_t lon, Real_t lat, Real_t& east, Real_t& north) const
{
    PJ_COORD inCoord, outCoord;
    inCoord = proj_coord( lon, lat, 0, 0 );

    outCoord = proj_trans( mProjector, PJ_FWD, inCoord );

    east  = outCoord.enu.e;
    north = outCoord.enu.n;
}

void GeoTransform::lat_lon_to_UTM(const vector<Point_t>& lps, vector<Point_t>& utms) const
{
    // Convert the std::vector into PJ_COORD array.
    const int n = lps.size();

    PJ_COORD* coordArray = new PJ_COORD[n];
    int i = 0;
    for ( auto iter = lps.begin(); iter != lps.end(); ++iter, ++i )
    {
        coordArray[i] = proj_coord( (*iter).x, (*iter).y, 0, 0 );
    }

    // Transform.
    size_t res = 0;

    res = proj_trans_array( mProjector, PJ_FWD, n, coordArray );

    if ( 0 != res )
    {
        delete coordArray; coordArray = NULL;
        
        cout << "Error: proj_trans_array() failed." << endl;

        stringstream expSS;
        expSS << "PROJ4 fails with error message: " 
              << proj_errno_string( proj_errno(mProjector) );

        GEO_ERROR_MESSAGE( expSS.str() );
    }

    // Conver the PJ_COORD array back into std::vector.

    i = 0;
    utms.clear();
    Point_t tempPoint;

    for ( ; i < n; ++i )
    {
        tempPoint.x = coordArray[i].enu.e;
        tempPoint.y = coordArray[i].enu.n;

        utms.push_back( tempPoint );
    }

    delete coordArray; coordArray = NULL;
}

void GeoTransform::UTM_to_lat_lon(Real_t east, Real_t north, Real_t& lon, Real_t& lat) const
{
    PJ_COORD inCoord, outCoord;
    inCoord = proj_coord( east, north, 0, 0 );

    outCoord = proj_trans( mProjector, PJ_INV, inCoord );

    lon = outCoord.lp.lam;
    lat = outCoord.lp.phi;
}

void GeoTransform::UTM_to_lat_lon(const vector<Point_t>& utms, vector<Point_t>& lps) const
{
    // Convert the std::vector into PJ_COORD array.
    const int n = utms.size();

    PJ_COORD* coordArray = new PJ_COORD[n];
    int i = 0;
    for ( auto iter = utms.begin(); iter != utms.end(); ++iter, ++i )
    {
        coordArray[i] = proj_coord( (*iter).x, (*iter).y, 0, 0 );
    }

    // Transform.
    size_t res = 0;

    res = proj_trans_array( mProjector, PJ_INV, n, coordArray );

    if ( 0 != res )
    {
        delete coordArray; coordArray = NULL;

        cout << "Error: proj_trans_array() failed." << endl;

        stringstream expSS;
        expSS << "PROJ4 fails with error message: " 
              << proj_errno_string( proj_errno(mProjector) );

        GEO_ERROR_MESSAGE( expSS.str() );
    }

    // Conver the PJ_COORD array back into std::vector.

    i = 0;
    lps.clear();
    Point_t tempPoint;

    for ( ; i < n; ++i )
    {
        tempPoint.x = coordArray[i].lp.lam;
        tempPoint.y = coordArray[i].lp.phi;

        lps.push_back( tempPoint );
    }

    delete coordArray; coordArray = NULL;
}

void geo::test_geodesy_thirdparty(void)
{
    GeoTransform gt(51);

    Real_t lon = 123.6357387, lat = 41.8003577; // Degrees.
    Real_t east = 0.0, north = 0.0;

    gt.lat_lon_to_UTM( PJ_D2R(lon), PJ_D2R(lat), east, north );

    cout.precision(12);
    cout << "east = " << east << ", north = " << north << endl;

    gt.UTM_to_lat_lon( east, north, lon, lat );

    cout << "lon = " << PJ_R2D( lon ) << ", lat = " << PJ_R2D( lat ) << endl;

    // Batch test.

    typedef vector<GeoTransform::Point_t> PointVec_t;

    PointVec_t lpVec;
    PointVec_t enuVec;

    lpVec.push_back( GeoTransform::Point_t{ PJ_D2R(123.6357387), PJ_D2R(41.8003577)} );
    lpVec.push_back( GeoTransform::Point_t{ PJ_D2R(123.6357387), PJ_D2R(41.8003841)} );

    gt.lat_lon_to_UTM( lpVec, enuVec );

    cout << "lpVec -> enuVec: " << endl;
    for ( auto iter = enuVec.begin(); iter != enuVec.end(); ++iter )
    {
        cout << "east = " << (*iter).x << ", north = " << (*iter).y << endl;
    }

    gt.UTM_to_lat_lon( enuVec, lpVec );

    cout << "enuVec -> lpVec: " << endl;
    for ( auto iter = lpVec.begin(); iter != lpVec.end(); ++iter )
    {
        cout << "lon = " << PJ_R2D( (*iter).x ) << ", lat = " << PJ_R2D( (*iter).y ) << endl;
    }
}
