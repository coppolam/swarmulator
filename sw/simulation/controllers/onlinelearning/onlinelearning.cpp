#include "onlinelearning.h"
#include "agent.h"
#include "main.h"
#include "randomgenerator.h"
#include "auxiliary.h"
#include "draw.h"

// Use the Eigen library for optimization
#define OPTIM_ENABLE_ARMA_WRAPPERS
#include "optim.hpp"

using namespace std;
using namespace arma;

onlinelearning::onlinelearning(): Controller(), p(8, 1)
{
  // Initial values
  moving = false;
  moving_timer = rg.uniform_int(0, timelim);
  v_x_ref = rg.gaussian_float(0.0, 1.0);
  v_y_ref = rg.gaussian_float(0.0, 1.0);

  // Control values
  timelim = 2.0 * param->simulation_updatefreq();
  vmean = 0.5;

  // Policy
  if (!strcmp(param->policy().c_str(), "")) { motion_p.assign(7, 0.5); }
  else { motion_p = read_array_double(param->policy()); }
  vec temp(motion_p);
  pol = temp;

  // Initialize individual
  p.init(false);
}

mat pagerank(const mat &G)
{
  // Define parameters
  uint maxiter = 10000;
  float tol = 0.0000001;

  // Initialize
  uint i = 0;
  float residual = 1;
  mat pr = normalise(ones(1, G.n_rows) / G.n_rows, 1, 1);
  mat pr_prev;

  // PageRank iteration routine
  while (residual >= tol && i < maxiter) {
    pr_prev = pr;
    pr = pr * G;
    residual = norm(pr - pr_prev);
    i++;
  }

  // Return
  return normalise(pr, 1, 1);
}

double fitness(const vec &inputs, vec *grad_out, void *opt_data)
{
  // Load up pagerank estimator as pointer
  pagerank_estimator *p = reinterpret_cast<pagerank_estimator *>(opt_data);

  // Determine pagerank
  mat m(inputs);
  mat G = normalise(p->H, 1, 1);
  G.each_row() %= m.t();
  mat alpha = arma::cumsum(G) / arma::cumsum(p->E);
  mat pr = pagerank(normalise(G + p->E, 1, 1));

  // Calculate fitness
  mat des = ones(8, 1);
  des[0] = 0;
  double f = (dot(pr.t(), des) / mean(mean(des))) / mean(mean(pr));
  // if (f>0.){ //debug
  //   cout << "\n\n\np->H     " << endl;
  //   p->H.print();
  //   cout << "inputs     " << endl;
  //   inputs.print();
  //   cout << "G     " << endl;
  //   G.print();
  //   cout << "pr     " << endl;
  //   pr.print();
  //   cout << "f     \n" << f << endl;
  // }
  return f;
}

void onlinelearning::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{
  v_x = 0;
  v_y = 0;

  vector<double> a = {1, 2, 3, 4};
  mat m(&a.front(), 2, 2);

  get_lattice_motion_all(ID, v_x, v_y); // Repulsion from neighbors

  // Sense neighbors
  vector<float> r, b;
  o.relative_location_inrange(ID, rangesensor, r, b);

  if (st != r.size() || moving_timer == 1) { // state change
    // state, action
    st = std::min(r.size(), motion_p.size());

    // Onboard estimator
    int a;
    if (moving) {a = 1;} else {a = 0;}
    p.update(0, st, a); // pr update

    if (rg.bernoulli(1.0 - pol[st])) {
      v_x_ref = 0.0;
      v_y_ref = 0.0;
      moving = false;
    } else { // Else explore randomly, change heading
      float ang = rg.uniform_float(0.0, 2 * M_PI);
      if (moving) {
        float ext = rg.gaussian_float(0.0, 0.5);
        float temp;
        cart2polar(v_x_ref, v_y_ref, temp, ang);
        ang += ext;
      }
      wrapTo2Pi(ang);
      polar2cart(vmean, ang, v_x_ref, v_y_ref);
      moving = true;
    }
  }
  increase_counter_to_value(moving_timer, timelim, 1);
  wall_avoidance_bounce(ID, v_x_ref, v_y_ref);

  // Every 10 seconds of new data, optimize
  if (moving_timer == 1) {
    optim::algo_settings_t settings;
    // bounds
    settings.vals_bound = true;
    settings.upper_bounds = arma::ones(motion_p.size(), 1);
    settings.lower_bounds = arma::zeros(motion_p.size(), 1);
    // vec x = ones(motion_p.size(), 1)/2.;
    bool success = optim::nm(pol, fitness, &p, settings);
    if (success) {cout << "pol" << endl; pol.t().raw_print();} // debug
  }
  // Final output
  v_x += v_x_ref;
  v_y += v_y_ref;
}

void onlinelearning::animation(const uint16_t ID)
{
  draw d;
  d.circle_loop(rangesensor);
}