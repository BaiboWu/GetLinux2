#ifndef INVERSE_CALC_H
#define INVERSE_CALC_H

#include <QWidget>
#include <stdlib.h>
#include <math.h>
#define PI 3.14159

/*Calculate cable length from joint angles*/
double q_calc(double alpha, double belta, double phi, quint8 flag);

#endif // INVERSE_CALC_H
