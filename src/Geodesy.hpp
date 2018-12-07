#ifndef __GEODESY_HPP__
#define __GEODESY_HPP__

#include <iostream>

namespace geo
{

typedef double Real_t;

class Geodesy
{
public:
    Geodesy();
    ~Geodesy();

    void LLF_to_ECEF( Real_t east, Real_t north, Real_t height, 
        Real_t oriLat, Real_t oriLon, Real_t oriHeight,
        Real_t& lat, Real_t& lon,
        int* iters = NULL, Real_t* resPhi = NULL, Real_t* resHeight = NULL );

protected:
    Real_t get_RN( Real_t lat );

    /**
     * @param lat Latitude, degree.
     * @param lon Longitude, degree.
     */
    void geodetic_to_rectangular_ECEF( Real_t lat, Real_t lon, Real_t height, 
        Real_t RN, Real_t ecc,
        Real_t& x, Real_t& y, Real_t& z );

    void LLF_to_ECEF_rectangular( Real_t east, Real_t north, Real_t height,
        Real_t oriLat, Real_t oriLon, Real_t oriHeight,
        Real_t& x, Real_t& y, Real_t& z );

    void rectangular_to_geodetic_in_ECEF( Real_t x, Real_t y, Real_t z,
        Real_t& lat, Real_t& lon, Real_t& height,
        int maxIters = 100, Real_t maxResPhi = 1e-6, Real_t maxResHeight = 1e-1, 
        int* iters = NULL, Real_t* resPhi = NULL, Real_t* resHeight = NULL);

public:
    static const Real_t WGS84_a; // Semimajor axis (equatorial radius), m.
    static const Real_t WGS84_b; // Semiminor axis, m.
    static const Real_t WGS84_f; // Flatness.
    static const Real_t WGS84_e; // Eccentricity.

protected:
    const Real_t F_D2R; // Degree to radian factor.
    const Real_t E2;    // Eccentricity squared.
};

void test_geodesy(void);

}

#endif // __GEODESY_HPP__