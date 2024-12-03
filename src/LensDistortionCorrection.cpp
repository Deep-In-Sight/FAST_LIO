#include "LensDistortionCorrection.h"

double LensDistortionCorrection::evaluatePolynomial(const double coeffs[], double x,  int degree = 6) {
    double result = 0.0;
    for (int i = degree; i >= 0; --i) {
        result = result * x + coeffs[i];
    }
    return result;
}

std::pair<int, int> LensDistortionCorrection::angleToPixel(double azimuth, double elevation)
{
    int u, v;

    double x_ref = azimuth / angle_per_pixel;
    double y_ref = elevation / angle_per_pixel;

    double r = sqrt(x_ref * x_ref + y_ref * y_ref) * pixel_size;
    double dist = evaluatePolynomial(polyfit_dist_coeff, r);
    u = static_cast<int>(x_ref * dist);
    v = static_cast<int>(y_ref * dist);
    
    return std::make_pair(u, v);
}