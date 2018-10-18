#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <QWidget>
#include <Eigen/Dense>
#include "Inverse_calc.h"

/*Calculate driving value with angles*/
Eigen::Matrix3d algorithm(Eigen::MatrixXd sec_ang_real, Eigen::MatrixXd sec_ang_targ);

#endif // ALGORITHM_H
