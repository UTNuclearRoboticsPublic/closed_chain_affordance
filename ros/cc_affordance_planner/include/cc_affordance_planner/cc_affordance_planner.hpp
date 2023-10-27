/*
   Author: Crasun Jans
*/
#ifndef CC_AFFORDANCE_PLANNER
#define CC_AFFORDANCE_PLANNER

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <affordance_util/affordance_util.hpp>
#include <vector>

class CcAffordancePlanner {
public:
  // Variables
  // Algorithm control parameters
  double affStep;
  double accuracy; // accuracy for error threshold
  double taskErrThreshold;
  int maxItr;        // for IK solver
  int stepperMaxItr; // for total steps
  double dt;         // time step to compute joint velocities
  int taskOffset;
  CcAffordancePlanner(Eigen::MatrixXd slist, Eigen::Matrix4d mErr,
                      Eigen::VectorXd thetalist0)
      : screwAxes_(screwAxes), mErr_(mErr), thetalist0_(thetalist0) {
    affStep = 0.1;
    accuracy = 1.0 * (1.0 / 100.0);
    taskErrThreshold = accuracy * affStep;
    maxItr = 50;
    stepperMaxItr = 75;
    dt = 1e-2;
    taskOffset = 1;

    // Guesses
    qsb_guess_ = thetalist0.tail(taskOffset);
    qp_guess_ = thetalist0.head(thetalist0.size() - taskOffset);
  };

  Eigen::VectorXd affordance_stepper() {

    int stepperItr = 0;
    while (stepperItr < stepperMaxItr) {
      // Define Network Matrices as relevant Jacobian columns
      Eigen::MatrixXd rJ = AffordanceUtil::JacobianSpace(slist, thetalist0);
      Eigen::MatrixXd Np = rJ.leftCols(rJ.cols() - taskOffset);
      Eigen::MatrixXd Ns = rJ.rightCols(taskOffset);

      // Set desired secondary task (affordance and maybe gripper orientation)
      // as just a few radians away from the current position
      Eigen::VectorXd qsd_ = thetalist0_.tail(taskOffset);
      Eigen::VectorXd onesVector = Eigen::VectorXd::Ones(taskOffset);
      qsd_ = qsd_ + stepperItr * affStep * onesVector;

      // Set starting guess for the primary joint angles
      oldqp_ = Eigen::VectorXd::Zero(qp.size());
      qp_ = qp_guess_;
      qsb_ = qsb_guess_;

      // Set closure error to zero and pass it along with joint angles to the
      // ik_solver
      errTwist_ = Eigen::VectorXd::Zero(twistLength_)

          // Call IK solver
          cc_ik_solver();
    }
  }

  Eigen::VectorXd cc_ik_solver() {

    int ikIter = 0;

    // Check Newton-Raphson error
    bool err = ((qsd - qsb).norm() > taskErrThreshold);
    while (err && ikIter < maxItr) {

      // Update Jacobians
      Eigen::VectorXd thetalist(qp_.rows() + qsb_.rows());
      thetalist << qp_, qsb_;
      Eigen::MatrixXd rJ = AffordanceUtil::JacobianSpace(slist, thetalist);
      Eigen::MatrixXd Np = rJ.leftCols(rJ.cols() - taskOffset);
      Eigen::MatrixXd Ns = rJ.rightCols(taskOffset);

      // Compute primary joint angles
      Eigen::VectorXd qp_dot = (qp_ - oldqp_) / dt;
      // Calculate the pseudo-inverse of Ns
      Eigen::MatrixXd pinv_Ns =
          (Ns.transpose() * Ns).ldlt().solve(Ns.transpose());
      // Calculate the pseudo-inverse of qp_dot
      Eigen::MatrixXd pinv_qp_dot =
          (qp_dot.transpose() * qp_dot).ldlt().solve(qp_dot.transpose());
      // Calculate cJ
      Eigen::MatrixXd cJ = -pinv_Ns * (Np + errTwist_ * pinv_qp_dot);
      oldqp_ = qp_;
      // Calculate the pseudo-inverse of cJ
      Eigen::MatrixXd pinv_cJ =
          (cJ.transpose() * cJ).ldlt().solve(cJ.transpose());

      // Calculate the update for qp
      qp_ = qp_ + pinv_cJ * (qsd - qsb);

      // Optimize closure error right here

      // Check Newton-Raphson error
      ikIter = ikIter + 1;
    }
  }

private:
  const twistLength_ = 6;
  const Eigen::MatrixXd screwAxes_;
  const Eigen::Matrix4d mErr_;
  const Eigen::VectorXd thetalist0_;
  const Eigen::VectorXd thetalist_;
  Eigen::VectorXd qp_guess_;
  Eigen::VectorXd qsb_guess_;
  Eigen::VectorXd qp_;
  Eigen::VectorXd oldqp_;
  Eigen::VectorXd qsb_;
  Eigen::VectorXd qsd_;
  Eigen::VectorXd errTwist_;
};
#endif // CC_AFFORDANCE_PLANNER
