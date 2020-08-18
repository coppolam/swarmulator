#ifndef CONTROLLER_LATTICE_BASIC_H
#define CONTROLLER_LATTICE_BASIC_H

#include "controller.h"
#include "template_calculator.h"

/**
 * Controller that holds a basic lattice between particles by handling attraction and
 * repulsion forces
 * This class is meant to be used as a parent class for any higher level controller
 * that wants to hold a lattice at all times, as is the case for the pattern formation
 * controller for instance.
 */
class lattice_basic : public Controller
{
public:
  Template_Calculator t;
  std::vector<float> beta_des;
  float _v_adj; // Adjustment velocity
  float d_safe; // Safety distance after which only nearest neighbor is considered
  float sensor_range;

  /**
   * @brief Construct a new lattice_basic object
   */
  lattice_basic();

  /**
   * @brief Destroy the lattice basic object
   *
   */
  ~lattice_basic() {};

  /**
   * @brief
   *
   * @param v_r
   * @param v_b
   * @param v_x
   * @param v_y
   */
  void attractionmotion(const float &v_r, const float &v_b, float &v_x, float &v_y);

  /**
   * @brief Lattice motion
   *
   * @param v_r
   * @param v_adj
   * @param v_b
   * @param bdes
   * @param v_x
   * @param v_y
   */
  void latticemotion(const float &v_r, const float &v_adj, const float &v_b, const float &bdes, float &v_x, float &v_y);

  /**
   * @brief Attraction force
   *
   * @param u
   * @param b_eq
   * @return float
   */
  float f_attraction(const float &u, const float &b_eq);

  /**
   * @brief Get the attraction velocity object
   *
   * @param u
   * @param b_eq
   * @return float
   */
  float get_attraction_velocity(const float &u, const float &b_eq);

  /**
   * @brief Move along the lattice
   *
   * @param selected_action
   * @param v_x
   * @param v_y
   */
  void actionmotion(const int &selected_action, float &v_x, float &v_y);

  /**
   * Determine if neighbors are moving
   *
   * @param state_ID
   * @return true
   * @return false
   */
  bool check_motion(const std::vector<int> &state_ID);

  /**
   * @brief Get the lattice motion object
   *
   * @param ID
   * @param state_ID
   * @param v_x
   * @param v_y
   */
  void get_lattice_motion(const int &ID, const int &state_ID, float &v_x, float &v_y);

  /**
   * @brief Handles all the above in one command, it can be called by the child controller
   * to handle the lower level lattice control (see pattern formation controller for an example)
   *
   * @param ID
   * @param state_ID
   * @param closest
   * @param v_x
   * @param v_y
   */
  void get_lattice_motion_all(const int &ID, const std::vector<int> &state_ID, const std::vector<uint> &closest,
                              float &v_x, float &v_y);

  /**
   * @brief Get the velocity command object. Not implemented here, but by child controllers of this class.
   *
   * @param ID
   * @param v_x
   * @param v_y
   */
  virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y) = 0;
};

#endif /*CONTROLLER_LATTICE_BASIC_H*/
