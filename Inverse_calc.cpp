#include "Inverse_calc.h"

double q_calc(double alpha, double belta, double phi, quint8 flag)
{
    double h = 12.35, r = 24.5;
    double q_x, q_y, q_z, q;

    phi = (double) ((flag * 120) - phi) * PI / 180.0;
    alpha = (double) alpha * PI / 180.0;
    belta = (double) belta * PI / 180.0;
    q_x = r * cos(phi) * cos(belta) + h * sin(belta) - r * cos(phi);
    q_y = r * cos(phi) * sin(alpha) * sin(belta) + r * sin(phi) * cos(alpha) - h * sin(alpha) * cos(belta) - r * sin(phi);
    q_z = -r * cos(phi) * cos(alpha) * sin(belta) + r * sin(phi) * sin(alpha) + h * cos(alpha) * cos(belta) + h;

    q=sqrt(q_x*q_x+q_y*q_y+q_z*q_z);

    return q;

}
