#ifndef __GEODESYTHIRDPARTY_HPP__
#define __GEODESYTHIRDPARTY_HPP__

#include <iostream>
#include <string>
#include <vector>
#include <proj.h>

#include "Common.hpp"

#define PJ_D2R(x) \
    proj_torad(x)

#define PJ_R2D(x) \
    proj_todeg(x)

#define GEO_ERROR_MESSAGE(msg) \
    {\
        BOOST_THROW_EXCEPTION( GEOException() << wz::ExceptionInfoString(msg) );\
    }

using namespace std;

namespace geo
{

struct GEOException : virtual wz::exception_base { };

typedef double Real_t;

class GeoTransform
{
public:
    typedef struct
    {
        Real_t x;
        Real_t y;
    } Point_t;

public:
    GeoTransform(int zone);
    ~GeoTransform();

    /**
     * @param lon Latitude, radian.
     * @param lat Longitude, radian.
     */
    void lat_lon_to_UTM(Real_t lon, Real_t lat, Real_t& east, Real_t& north) const;
    void lat_lon_to_UTM(const vector<Point_t>& lps, vector<Point_t>& utms) const;

    /**
     * @param lon Latitude, radian.
     * @param lat Longitude, radian.
     */
    void UTM_to_lat_lon(Real_t east, Real_t north, Real_t& lon, Real_t& lat) const;
    void UTM_to_lat_lon(const vector<Point_t>& utms, vector<Point_t>& lps) const;

    int  get_zone(void);

    string& get_projector_string(void);

protected:
    int mZone;
    PJ *mProjector;
    string mProjectorString;

private:
    static PJ_CONTEXT *mPJContext;
};

void test_geodesy_thirdparty(void);

}

#endif // __GEODESYTHIRDPARTY_HPP__