#include "Algorithm.h"

Eigen::Matrix3d algorithm(Eigen::MatrixXd sec_ang_real, Eigen::MatrixXd sec_ang_targ)
{
    double d_ang, P_ang=0.05;
    double sec_ang_step[3][2] = {0}, sec_phi[3] = {30, -90, -30};
    double sec_dst_real[3][3] = {0}, sec_dst_step[3][3] = {0};
    float deg_yuzhi[3][2] = {0}, deg_thred[2] = {0.1, 0.2};
    quint8 addeg_flag[3][2] = {0};
    quint8 i, j;

    Eigen::Matrix3d sec_delta_dst;

    //PD Control with degree
    for(i = 0; i < 3; i++)
    {
        for(j = 0; j < 2; j++)
        {
            d_ang = P_ang * (sec_ang_targ(i, j) - sec_ang_real(i, j));
            if(fabs(d_ang) > 0.2)
            {
                d_ang = 0.2 * (d_ang/fabs(d_ang));
            }

            sec_ang_step[i][j] = sec_ang_real(i, j) + d_ang;
        }
    }

    //calculate delta motion for next circle
    for(i = 0; i < 3; i++)
    {
        for(j = 0; j < 3; j++)
        {
            sec_dst_real[i][j] = q_calc(sec_ang_real(i, 0), sec_ang_real(i, 1), sec_phi[i], j);
            sec_dst_step[i][j] = q_calc(sec_ang_step[i][0], sec_ang_step[i][1], sec_phi[i], j);
            if(i == 0)
                sec_delta_dst(i, j) = sec_dst_real[i][j] - sec_dst_step[i][j];
        }
    }
    //Add extra motion for behind sections
    for(j = 0; j < 3; j++)
    {
        //#section 2
        sec_dst_real[1][j] = sec_dst_real[1][j] + q_calc(sec_ang_real(0, 0), sec_ang_real(0, 1), 0, j);
        sec_dst_step[1][j] = sec_dst_step[1][j] + q_calc(sec_ang_step[0][0], sec_ang_step[0][1], 0, j);
        sec_delta_dst(1, j) = sec_dst_real[1][j] - sec_dst_step[1][j];

        //#section 3
        sec_dst_real[2][j] = sec_dst_real[2][j] + q_calc(sec_ang_real(0, 0), sec_ang_real(0, 1), -30, j) + q_calc(sec_ang_real(1, 0), sec_ang_real(1, 1), -120, j);
        sec_dst_step[2][j] = sec_dst_step[2][j] + q_calc(sec_ang_step[0][0], sec_ang_step[0][1], -30, j) + q_calc(sec_ang_step[1][0], sec_ang_step[1][1], -120, j);
        sec_delta_dst(2, j) = sec_dst_real[2][j] - sec_dst_step[2][j];
    }

    //degree limit for stability
    for(i = 0; i < 3; i++)
    {
        //Vary between small and big threshold
        for(j = 0; j < 2; j++)
        {
            if(fabs(sec_ang_real(i, j)-sec_ang_targ(i, j)) < deg_yuzhi[i][j])
            {
                addeg_flag[i][j] = 1;
            }
            else
            {
                addeg_flag[i][j] = 0;
            }
            deg_yuzhi[i][j] = deg_thred[addeg_flag[i][j]];
        }

        //Check whether lies in the threshold and The back section is checked behind former.
        if((fabs(sec_ang_targ(i, 0)-sec_ang_real(i, 0))< deg_yuzhi[i][0]) && (fabs(sec_ang_targ(i, 1)-sec_ang_real(i, 1))< deg_yuzhi[i][1]))
        {
            for(j = 0; j < 3; j++)
            {
                sec_delta_dst(i, j) = 0;
            }
        }
        else
        {
            //count_iter = count_iter + 1;
            break;
        }
    }

    for(i = 0; i < 3; i++)
    {
        if((fabs(sec_ang_targ(i, 0)) > 30) || (fabs(sec_ang_targ(i, 1)) > 30))
        {
            for(j = 0; j < 3; j++)
            {
                sec_delta_dst(i, j) = 0.0;
            }
        }
    }

    return sec_delta_dst;
}
