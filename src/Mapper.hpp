#ifndef __MAPPER_HPP__
#define __MAPPER_HPP__

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "Common.hpp"

using namespace std;

namespace mapper
{

struct MapperException : virtual wz::exception_base { };

class BlendedMapper
{
public:
    typedef enum
    {
        JPEG = 0,
        PNG
    } ImageType_t;

public:
    BlendedMapper();
    ~BlendedMapper();

    void multi_image_homography_direct_blending(
        const string& baseDir, const string& outDir, const string& gpsFile,
        bool skipBlending = false, bool skipSeamFinding = false);

    void set_blender_band(int b);
    int  get_blender_band(void);

    void set_output_image_type(ImageType_t t);
    ImageType_t get_output_image_type(void);

    void enable_debug(void);
    void disable_debug(void);

    void set_debug_count(int c);

private:
    void directly_blend( const string& baseDir, const vector<string>& files,
        vector<cv::Mat>& warppedCenterPointVec, vector<int>& idxKFInCSVTable, 
        cv::Mat& gsh, cv::Mat& finalCornerPoints,
        const string& blendedFilename, 
        int blenderBand, bool skipBlending, bool skipSeamFinding, cv::OutputArray _blended );

protected:
    int mBlenderBand;
    ImageType_t mImageType;

    bool mFlagDebug;
    int mDebugCount;

};

} // namespace mapper;

#endif // __MAPPER_HPP__