#include "onlinelearning.h"
#include "agent.h"
#include "main.h"
#include "randomgenerator.h"
#include "auxiliary.h"
#include "draw.h"

// Use the Eigen library for optimization
#define OPTIM_ENABLE_ARMA_WRAPPERS
#include "optim.hpp"
#include <armadillo>

using namespace std;

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
  else { motion_p = read_array(param->policy()); }

  // Initialize individual
  p.init(false);
}

arma::mat pagerank(const arma::mat &G)
{
  // Define parameters
  uint maxiter = 1000;
  float tol = 0.01;

  // Initialize
  uint i = 0;
  float residual = 1;
  arma::mat pr = arma::randu(1, G.n_rows);
  arma::mat pr_prev;

  // PageRank iteration routine
  while (residual >= tol && i < maxiter) {
    pr_prev = pr;
    pr = pr * G;
    residual = arma::norm(pr - pr_prev);
    i++;
  }

  // Return
  return pr;
}

float fitness(const arma::vec &inputs, arma::vec *grad_out, void *opt_data)
{
  // Load up pagerank estimator as pointer
  pagerank_estimator *p = reinterpret_cast<pagerank_estimator *>(opt_data);

  // Determine pagerank
  arma::mat pr = pagerank(arma::normalise(p->H));

  // Calculate fitness
  arma::mat des(8, 1);
  des[2] = 1;
  float f = (float)arma::dot(pr, des);
  cout << f << endl;
  return f;
}

void onlinelearning::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{
  v_x = 0;
  v_y = 0;

  vector<double> a = {1, 2, 3, 4};
  arma::mat m(&a.front(), 2, 2);

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
    // p.print();

    if (rg.bernoulli(1.0 - motion_p[st])) {
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
    arma::vec x = arma::ones(motion_p.size(), 1);
    optim::bfgs(x, fitness, &p);
    // x.print();
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