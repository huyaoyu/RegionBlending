
#include <cmath>
#include <iostream>
#include <string>

#include <boost/math/constants/constants.hpp>

#include <Eigen/Eigen>

#include "Geodesy.hpp"

using namespace std;
using namespace geo;

const Real_t Geodesy::WGS84_a = 6378137.0;
const Real_t Geodesy::WGS84_b = 6356752.3142;
const Real_t Geodesy::WGS84_f = 0.00335281;
const Real_t Geodesy::WGS84_e = 0.08181919;

Geodesy::Geodesy()
: F_D2R( boost::math::constants::pi<Real_t>() / 180.0 ),
  E2( WGS84_e * WGS84_e )
{

}

Geodesy::~Geodesy()
{

}

Real_t Geodesy::get_RN( Real_t lat )
{
    Real_t temp = std::sin( lat * F_D2R );

    return WGS84_a / std::sqrt( 1 - E2 * temp * temp );
}

void Geodesy::geodetic_to_rectangular_ECEF( Real_t lat, Real_t lon, Real_t height, 
    Real_t RN, Real_t ecc,
    Real_t& x, Real_t& y, Real_t& z )
{
    Real_t phi    = lat * F_D2R;
    Real_t lambda = lon * F_D2R;

    x = ( RN + height ) * std::cos( phi ) * std::cos( lambda );
    y = ( RN + height ) * std::cos( phi ) * std::sin( lambda );
    z = ( RN * ( 1 - E2 ) + height ) * std::sin( phi );
}

void Geodesy::LLF_to_ECEF_rectangular( Real_t east, Real_t north, Real_t height,
        Real_t oriLat, Real_t oriLon, Real_t oriHeight,
        Real_t& x, Real_t& y, Real_t& z )
{
    Real_t phi    = oriLat * F_D2R;
    Real_t lambda = oriLon * F_D2R;

    Real_t RN = get_RN(oriLat); // The normal radius.

    Real_t cosPhi    = std::cos( phi ),    sinPhi    = std::sin( phi );
    Real_t cosLambda = std::cos( lambda ), sinLambda = std::sin( lambda );

    Eigen::MatrixXd ori( 3, 1 );
    ori(0, 0) = ( RN + oriHeight ) * cosPhi * cosLambda;
    ori(1, 0) = ( RN + oriHeight ) * cosPhi * sinLambda;
    ori(2, 0) = ( RN * ( 1 - E2 ) + oriHeight ) * sinPhi;

    Eigen::MatrixXd Rle( 3, 3 );
    Rle(0, 0) = -sinLambda; Rle(0, 1) = -sinPhi * cosLambda; Rle(0, 2) = cosPhi * cosLambda;
    Rle(1, 0) =  cosLambda; Rle(1, 1) = -sinPhi * sinLambda; Rle(1, 2) = cosPhi * sinLambda;
    Rle(2, 0) =        0.0; Rle(2, 1) =              cosPhi; Rle(2, 2) =             sinPhi;

    Eigen::MatrixXd point( 3, 1 );
    point(0, 0) = east; point(1, 0) = north; point(2, 0) = height;

    Eigen::MatrixXd ecef;

    ecef = Rle * point + ori;

    x = ecef( 0, 0 ); y = ecef( 1, 0 ); z = ecef( 2, 0 );
}

void Geodesy::rectangular_to_geodetic_in_ECEF( Real_t x, Real_t y, Real_t z,
        Real_t& lat, Real_t& lon, Real_t& height, 
        int maxIters, Real_t maxResPhi, Real_t maxResHeight, 
        int* iters, Real_t* resPhi, Real_t* resHeight)
{
    height = 0.0;
    Real_t P = std::sqrt( x * x + y * y );
    Real_t phi = std::atan2( z, P * (1 - E2) );
    Real_t lambda = std::atan2( y, x );

    int i = 0;
    Real_t resP = maxResPhi    + 1.0;
    Real_t resH = maxResHeight + 1.0;

    Real_t RN, sinPhi;
    Real_t lastPhi = phi, lastHeight = height;

    while ( i < maxIters && ( resP >= maxResPhi || resH >= maxResHeight ) )
    {
        sinPhi = std::sin( phi );

        RN = WGS84_a / std::sqrt( 1 - E2 * sinPhi * sinPhi );
        height = P / std::cos( phi ) - RN;
        phi = std::atan2( z * ( RN + height ), P * ( RN * ( 1 - E2 ) + height) );

        resP = std::fabs( phi - lastPhi );
        resH = std::fabs( height - lastHeight );

        lastPhi    = phi;
        lastHeight = height;

        i++;
    }

    lat = phi / F_D2R;
    lon = lambda / F_D2R;

    if ( NULL != iters )
    {
        *iters = i;
    }

    if ( NULL != resPhi )
    {
        *resPhi = resP;
    }

    if ( NULL != resHeight )
    {
        *resHeight = resH;
    }
}

void Geodesy::LLF_to_ECEF( Real_t east, Real_t north, Real_t height, 
        Real_t oriLat, Real_t oriLon, Real_t oriHeight,
        Real_t& lat, Real_t& lon, 
        int* iters, Real_t* resPhi, Real_t* resHeight )
{
    // Transform from LLF to ECEF in rectangular coordinate system.
    Real_t x = 0.0, y = 0.0, z = 0.0;

    LLF_to_ECEF_rectangular( east, north, height, oriLat, oriLon, oriHeight, x, y, z );

    // Transform from rectanular to geodetic coordinate in ECEF.
    Real_t heightTemp;

    rectangular_to_geodetic_in_ECEF( x, y, z,
        lat, lon, heightTemp, 
        100, 1e-6, 1e-1, 
        iters, resPhi, resHeight);
}

void geo::test_geodesy(void)
{
    Geodesy gd;

    Real_t lat = 0.0, lon = 0.0;
    int iters = 0;
    Real_t resPhi = 0.0, resHeight = 0.0;

    gd.LLF_to_ECEF( 0, 0, 0.0, 41.8003577, 123.6357387, 0.0, lat, lon, &iters, &resPhi, &resHeight );
    
    cout.precision(12);
    cout << "lat = " << lat << ", lon = " << lon << endl;
    cout << "iters = " << iters << ", resPhi = " << resPhi << ", resHeight = " << resHeight << endl;

    gd.LLF_to_ECEF( 552845.231128 - 552835.874403, 4629752.366287 - 4629657.72452, 0.0, 41.8003577, 123.6357387, 0.0, lat, lon, &iters, &resPhi, &resHeight );
    
    cout << "lat = " << lat << ", lon = " << lon << endl;
    cout << "iters = " << iters << ", resPhi = " << resPhi << ", resHeight = " << resHeight << endl;
}
