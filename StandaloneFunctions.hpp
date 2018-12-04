#ifndef __STANDALONEFUNCTIONS_HPP__
#define __STANDALONEFUNCTIONS_HPP__

#include <string>

void test_side_by_side_blending(std::string& imgFn0, std::string& imgFn1, double scaleFactor = 0.1);
void test_region_blending(std::string& imgFn0, std::string& imgFn1, double scaleFactor = 0.1);
void test_region_blending_and_overwriting(std::string& imgFn0, std::string& imgFn1, double scaleFactor = 0.1);
void test_boost_filesystem_and_opencv_yaml(void);
void test_two_image_homography(void);
void test_two_image_homography_direct_blending(void);
void test_multi_image_homography(void);
void test_multi_image_homography_direct_blending(void);

#endif // __STANDALONEFUNCTIONS_HPP__
