#ifndef __MAPPER_HPP__
#define __MAPPER_HPP__

#include <string>

using namespace std;

namespace mapper
{

class BlendedMapper
{
public:
    BlendedMapper();
    ~BlendedMapper();

    void multi_image_homography_direct_blending(
        const string& baseDir, const string& homoDir, const string& gpsFile,
        bool skipBlending = false);

    void set_blender_band(int b);
    int  get_blender_band(void);

protected:
    int mBlenderBand;

};


}

#endif // __MAPPER_HPP__