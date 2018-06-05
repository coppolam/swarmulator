#include "controller_lattice_basic.h"
#include "agent.h"
#include "particle.h"
#include "main.h"
#include "randomgenerator.h"
#include "auxiliary.h"

float Controller_Lattice_Basic::f_attraction(float u, float b_eq)
{
    float w;
    if (!(abs(b_eq - M_PI / 4.0) < 0.1 || abs(b_eq - (3 * M_PI / 4.0)) < 0.1))
    {
        w = log((_ddes / _kr - 1) / exp(-_ka * _ddes)) / _ka;
    }
    else
    {
        w = log((sqrt(pow(_ddes, 2.0) + pow(_ddes, 2.0)) / _kr - 1) / exp(-_ka * sqrt(pow(_ddes, 2.0) + pow(_ddes, 2.0)))) / _ka;
    }

    return 1 / (1 + exp(-_ka * (u - w)));
}

float Controller_Lattice_Basic::get_attraction_velocity(float u, float b_eq)
{
    return f_attraction(u, b_eq) + f_repulsion(u);
}

void Controller_Lattice_Basic::attractionmotion(const float &v_r, const float &v_b, float &v_x, float &v_y)
{
    v_x += v_r * cos(v_b);
    v_y += v_r * sin(v_b);
}

void Controller_Lattice_Basic::latticemotion(const float &v_r, const float &v_adj, const float &v_b, const float &bdes, float &v_x, float &v_y)
{
    attractionmotion(v_r + v_adj, v_b, v_x, v_y);
    v_x += -v_adj * cos(bdes * 2 - v_b);
    v_y += -v_adj * sin(bdes * 2 - v_b);
}

void Controller_Lattice_Basic::actionmotion(const int selected_action, float &v_x, float &v_y)
{
    float actionspace_y[8] = {0, sqrt(1), 1, sqrt(1), 0, -sqrt(1), -1, -sqrt(1)};
    float actionspace_x[8] = {1, sqrt(1), 0, -sqrt(1), -1, -sqrt(1), 0, sqrt(1)};
    v_x = _v_adj * actionspace_x[selected_action];
    v_y = _v_adj * actionspace_y[selected_action];
}