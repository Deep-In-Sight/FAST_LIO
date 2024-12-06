#ifndef LENS_DISTORTION_CORRECTION_H
#define LENS_DISTORTION_CORRECTION_H

#include <opencv2/opencv.hpp>
#include <utility>

class LensDistortionCorrection
{
private:
    const double pixel_size = 0.00274;
    const double angle_per_pixel = 0.06435562966942021;
    const double polyfit_dist_coeff[7] = {
    1.476651611450123e-05,
    -8.720389140454326e-05,
    0.00020375035767809894,
    -0.0003831669432987341,
    0.005164684960052346,
    -5.2301765814904146e-05,
    0.9999892807161488
    };

    double evaluatePolynomial(const double coeffs[], double x,  int degree);
public:
    std::pair<int, int> angleToPixel(double azimuth, double elevation);
};

#endif // LENS_DISTORTION_CORRECTION_H