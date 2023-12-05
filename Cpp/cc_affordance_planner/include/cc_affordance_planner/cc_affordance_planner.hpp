/*
   Author: Crasun Jans
*/
#ifndef CC_AFFORDANCE_PLANNER
#define CC_AFFORDANCE_PLANNER

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <affordance_util/affordance_util.hpp>
#include <chrono>
#include <iostream>
#include <vector>

// Struct to store result from planner
/**
 * @brief Designed to contain the result of an IK planner with fields:
 * success of type bool, trajFullorPartial of type string with values: "Full",
 * "Partial", or "Unset", and jointTraj of type std::vector<Eigen::VectorXd
 */
struct PlannerResult {
  bool success;
  std::string trajFullorPartial;
  std::vector<Eigen::VectorXd> jointTraj;
};

/**
 * @brief Given a list of closed-chain screw axes, corresponding starting joint
 * angles, and affordance goal, this planner computes the inverse kinematics
 * joint trajectory solution.
 */
class CcAffordancePlanner {
public:
  //* Algorithm control parameters (non-const to adjust from outside)
  // For stepper loop
  double affStep = 0.1;
  // For IK loop
  double accuracy = 1.0 * (1.0 / 100.0); // accuracy for error threshold
  int maxItr = 50;
  //* Other parameters
  int taskOffset = 1;

  // Constructor
  /**
   * @brief Given a list of closed-chain screw axes, corresponding starting
   joint angles, and affordance goal, this planner computes the inverse
   kinematics joint trajectory solution.
   *
   * @param slist List of all closed-chain screw axes
   * @param thetalist0 Corresponding joint angles for starting configuration
   * @param affGoal Desired affordance goal
   */
  CcAffordancePlanner(const Eigen::MatrixXd &slist,
                      const Eigen::VectorXd &thetalist0, const double &affGoal);

  // Methods
  /**
   * @brief Steps the IK solver towards the affordance goal affGoal by an
   * affordance step (affStep)
   *
   * @return Struct containing the result of the planning with fields:
   success of type bool, trajFullorPartial of type string with values: "Full",
   "Partial", or "Unset", and jointTraj of type std::vector<Eigen::VectorXd
   */
  PlannerResult affordance_stepper();

  /**
   * @brief Closed-chain IK solver. Not standalone.
   *
   * @return
   */
  bool cc_ik_solver();

  /**
   * @brief Closed-chain error corrector. Not standalone.
   */
  void closure_error_optimizer();

private:
  //* Thresholds
  double taskErrThreshold_;
  const double closureErrThreshold_ = 1e-6;
  //* Other parameters
  const int twistLength_ = 6; // length of a twist vector
  const double dt_ = 1e-2;    // time step to compute joint velocities
  // Helper variables to share info between functions
  const Eigen::MatrixXd slist_;
  const Eigen::VectorXd thetalist0_;
  const double affGoal_;
  const Eigen::Matrix4d mErr_ = Eigen::Matrix4d::Identity();
  Eigen::VectorXd thetalist_;
  Eigen::VectorXd qp_guess_;
  Eigen::VectorXd qsb_guess_;
  Eigen::VectorXd qp_;
  Eigen::VectorXd oldqp_;
  Eigen::VectorXd qsb_;
  Eigen::VectorXd qsd_;
  Eigen::VectorXd errTwist_;
  Eigen::MatrixXd Np_;
  Eigen::MatrixXd Ns_;
};
#endif // CC_AFFORDANCE_PLANNER
