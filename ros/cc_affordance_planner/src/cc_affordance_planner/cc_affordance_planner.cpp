#include <cc_affordance_planner/cc_affordance_planner.hpp>
CcAffordancePlanner::CcAffordancePlanner(const Eigen::MatrixXd &slist,
                                         const Eigen::Matrix4d &mErr,
                                         const Eigen::VectorXd &thetalist0)
    : slist(slist), mErr_(mErr), thetalist0_(thetalist0) {
  taskErrThreshold = accuracy * affStep;

  // Guesses
  qsb_guess_ = thetalist0.tail(taskOffset);
  qp_guess_ = thetalist0.head(thetalist0.size() - taskOffset);
};

std::vector<Eigen::VectorXd> CcAffordancePlanner::affordance_stepper() {

  std::vector<Eigen::VectorXd> jointTraj;
  int stepperItr = 0;
  while (stepperItr < stepperMaxItr) {
    // Define Network Matrices as relevant Jacobian columns
    Eigen::MatrixXd rJ = JacobianSpace(slist, thetalist0_);
    Eigen::MatrixXd Np = rJ.leftCols(rJ.cols() - taskOffset);
    Eigen::MatrixXd Ns = rJ.rightCols(taskOffset);

    // Set desired secondary task (affordance and maybe gripper orientation)
    // as just a few radians away from the current position
    Eigen::VectorXd qsd_ = thetalist0_.tail(taskOffset);
    Eigen::VectorXd onesVector = Eigen::VectorXd::Ones(taskOffset);
    qsd_ = qsd_ + stepperItr * affStep * onesVector;

    // Set starting guess for the primary joint angles
    qp_ = qp_guess_;
    qsb_ = qsb_guess_;
    oldqp_ = Eigen::VectorXd::Zero(qp_.size());

    // Set closure error to zero and pass it along with joint angles to the
    // ik_solver
    errTwist_ = Eigen::VectorXd::Zero(twistLength_);

    // Call IK solver and store solution if successful
    if (cc_ik_solver())
      jointTraj.push_back(thetalist_);
  }
  return jointTraj;
}

bool CcAffordancePlanner::cc_ik_solver() {

  int ikIter = 0;

  // Check Newton-Raphson error
  bool err = ((qsd_ - qsb_).norm() > taskErrThreshold);
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
    qp_ = qp_ + pinv_cJ * (qsd_ - qsb_);

    // Optimize closure error right here

    // Check Newton-Raphson error
    ikIter = ikIter + 1;

    err = ((qsd_ - qsb_).norm() > taskErrThreshold);
  }
  return err;
}
