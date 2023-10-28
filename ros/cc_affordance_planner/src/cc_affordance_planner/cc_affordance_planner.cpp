#include <cc_affordance_planner/cc_affordance_planner.hpp>
CcAffordancePlanner::CcAffordancePlanner(const Eigen::MatrixXd &slist,
                                         const Eigen::Matrix4d &mErr,
                                         const Eigen::VectorXd &thetalist0,
                                         const Eigen::MatrixXd &Tsd)
    : slist_(slist), mErr_(mErr), Tsd_(Tsd), thetalist0_(thetalist0) {
  taskErrThreshold = accuracy * affStep;

  // Guesses
  qsb_guess_ = thetalist0.tail(taskOffset);
  qp_guess_ = thetalist0.head(thetalist0.size() - taskOffset);

  // Resizings
  thetalist_.conservativeResize(thetalist0_.size());
}

std::vector<Eigen::VectorXd> CcAffordancePlanner::affordance_stepper() {

  std::vector<Eigen::VectorXd> jointTraj;
  int stepperItr = 1;
  while (stepperItr < stepperMaxItr) {
    // Define Network Matrices as relevant Jacobian columns
    Eigen::MatrixXd rJ = JacobianSpace(slist_, thetalist0_);
    Eigen::MatrixXd Np = rJ.leftCols(rJ.cols() - taskOffset);
    Eigen::MatrixXd Ns = rJ.rightCols(taskOffset);

    // Set desired secondary task (affordance and maybe gripper orientation)
    // as just a few radians away from the current position
    qsd_ = thetalist0_.tail(taskOffset);
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
    if (CcAffordancePlanner::cc_ik_solver())
      jointTraj.push_back(thetalist_);

    // increment counter
    stepperItr = stepperItr + 1;
  }
  return jointTraj;
}

bool CcAffordancePlanner::cc_ik_solver() {

  int ikIter = 0;

  // Check Newton-Raphson error
  bool err = ((qsd_ - qsb_).norm() > taskErrThreshold);
  while (err && ikIter < maxItr) {

    // Update Jacobians
    thetalist_ << qp_, qsb_;
    std::cout << "thetalist_\n" << thetalist_ << std::endl;
    Eigen::MatrixXd rJ = CcAffordancePlanner::JacobianSpace(slist_, thetalist_);
    Np_ = rJ.leftCols(rJ.cols() - taskOffset);
    Ns_ = rJ.rightCols(taskOffset);

    // Compute primary joint angles
    Eigen::MatrixXd qp_dot = (qp_ - oldqp_) / dt;
    // Calculate the pseudo-inverse of Ns
    Eigen::MatrixXd pinv_Ns;
    /* if (Ns_.isZero()) { */
    /*   pinv_Ns = Ns_.transpose(); */
    /* } else { */
    pinv_Ns = Ns_.completeOrthogonalDecomposition().pseudoInverse();
    /* } */
    // Calculate the pseudo-inverse of qp_dot
    Eigen::MatrixXd pinv_qp_dot;
    /* if (qp_dot.isZero()) { */
    /* pinv_qp_dot = qp_dot.transpose(); */
    /* } else { */
    std::cout << "qp_dot\n" << qp_dot << std::endl;
    pinv_qp_dot = qp_dot.completeOrthogonalDecomposition().pseudoInverse();
    /* } */
    /* std::cout << "pinv_qp_dot\n" << pinv_qp_dot << std::endl; */
    /* std::cout << "errTwist_\n" << errTwist_ << std::endl; */
    /* std::cout << "Np_\n" << Np_ << std::endl; */
    /* std::cout << "pinv_Ns_\n" << pinv_Ns << std::endl; */
    // Calculate cJ
    Eigen::MatrixXd cJ = -pinv_Ns * (Np_ + errTwist_ * pinv_qp_dot);
    oldqp_ = qp_;
    // Calculate the pseudo-inverse of cJ
    Eigen::MatrixXd pinv_cJ;
    /* if (cJ.isZero()) { */
    /* pinv_cJ = cJ.transpose(); */
    /* } else { */
    pinv_cJ = cJ.completeOrthogonalDecomposition().pseudoInverse();
    /* } */

    // Calculate the update for qp
    qp_ = qp_ + pinv_cJ * (qsd_ - qsb_);

    // Optimize closure error right here
    CcAffordancePlanner::closure_error_optimizer();

    // Check Newton-Raphson error
    ikIter = ikIter + 1;

    err = ((qsd_ - qsb_).norm() > taskErrThreshold);
    std::cout << "ikIter: " << ikIter << std::endl;
  }
  return !err;
}

void CcAffordancePlanner::closure_error_optimizer() {
  thetalist_ << qp_, qsb_;

  // Calculate the closure error
  Eigen::Matrix4d Tse =
      CcAffordancePlanner::FKinSpace(mErr_, slist_, thetalist_);
  errTwist_ = CcAffordancePlanner::Adjoint(Tse) *
              CcAffordancePlanner::se3ToVec(CcAffordancePlanner::MatrixLog6(
                  CcAffordancePlanner::TransInv(Tse) * Tsd_));

  // Adjust qp and qs based on this error;
  // Combine Np and Ns horizontally
  Eigen::MatrixXd N(Np_.rows(), Np_.cols() + Ns_.cols());
  N << Np_, Ns_;
  // Calculate the pseudo-inverse and compute q
  Eigen::MatrixXd N_pseudo_inverse;
  /* if (N.isZero()) { */
  /* N_pseudo_inverse = N.transpose(); */
  /* } else { */
  N_pseudo_inverse = N.completeOrthogonalDecomposition().pseudoInverse();
  /* } */
  Eigen::VectorXd q = N_pseudo_inverse * errTwist_;
  qp_ = qp_ + q.head(q.size() - taskOffset);
  qsb_ = qsb_ + q.tail(taskOffset);

  // Update thetalist
  thetalist_ << qp_, qsb_;

  // Compute final error
  Tse = CcAffordancePlanner::FKinSpace(mErr_, slist_, thetalist_);
  errTwist_ = CcAffordancePlanner::Adjoint(Tse) *
              CcAffordancePlanner::se3ToVec(CcAffordancePlanner::MatrixLog6(
                  CcAffordancePlanner::TransInv(Tse) * Tsd_));
}
