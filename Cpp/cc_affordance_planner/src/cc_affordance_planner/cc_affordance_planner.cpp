#include <cc_affordance_planner/cc_affordance_planner.hpp>
CcAffordancePlanner::CcAffordancePlanner(const Eigen::MatrixXd &slist,
                                         const Eigen::VectorXd &thetalist0,
                                         const double &affGoal)
    : slist_(slist), thetalist0_(thetalist0), affGoal_(affGoal) {
  taskErrThreshold_ = accuracy * affStep;

  // Resizings
  thetalist_.conservativeResize(thetalist0_.size());
}

PlannerResult CcAffordancePlanner::affordance_stepper() {

  PlannerResult plannerResult; // Result of the planner

  // Initial guesses
  qsb_guess_ = thetalist0_.tail(taskOffset);
  qp_guess_ = thetalist0_.head(thetalist0_.size() - taskOffset);

  // Compute how many steps it takes to reach the affordance goal
  const int stepperMaxItr = affGoal_ / affStep + 1;

  // Stepper loop
  int stepperItr = 1;
  int successStepperItr = 1;
  qsd_ = thetalist0_.tail(
      taskOffset); // We extract the size for qsd and also the reference to
                   // start from. We'll set the affordance goal in the loop with
                   // respect to this reference
  while (stepperItr <= stepperMaxItr) {

    // Define Network Matrices as relevant Jacobian columns
    Eigen::MatrixXd rJ = AffordanceUtil::JacobianSpace(slist_, thetalist0_);
    Eigen::MatrixXd Np = rJ.leftCols(rJ.cols() - taskOffset);
    Eigen::MatrixXd Ns = rJ.rightCols(taskOffset);

    // Set desired secondary task (affordance and maybe gripper orientation)
    // as affStep away from the current position
    if (stepperItr == (stepperMaxItr)) // adjust the step on the last iteration
                                       // to reach affordance goal
    {
      affStep = affGoal_ - affStep * (stepperMaxItr - 1);
    }

    /* qsd_ = qsd_ + stepperItr * affStep * Eigen::VectorXd::Ones(taskOffset);
     */
    qsd_(taskOffset - 1) =
        qsd_(taskOffset - 1) + affStep; // end element is affordance

    // Set starting guess for the primary joint angles
    qp_ = qp_guess_;
    qsb_ = qsb_guess_;
    oldqp_ =
        Eigen::VectorXd::Zero(qp_.size()); // set old joint velocities as zero

    // Set closure error to zero to enter the IK loop
    errTwist_ = Eigen::VectorXd::Zero(twistLength_);

    // Call IK solver and store solution if successful
    if (CcAffordancePlanner::cc_ik_solver()) {
      plannerResult.jointTraj.push_back(thetalist_);

      // Update guesses for next iteration
      qsb_guess_ = thetalist_.tail(taskOffset);
      qp_guess_ = thetalist_.head(thetalist0_.size() - taskOffset);
      successStepperItr = successStepperItr + 1;
    }

    // increment stepper counter
    stepperItr = stepperItr + 1;
  }

  if (!plannerResult.jointTraj.empty()) {
    plannerResult.success = true;

    if (stepperItr == successStepperItr) {
      plannerResult.trajFullorPartial = "Full";
    } else {
      plannerResult.trajFullorPartial = "Partial";
    }
  } else {
    plannerResult.success = false;
    plannerResult.trajFullorPartial = "Unset";
  }
  return plannerResult;
}

bool CcAffordancePlanner::cc_ik_solver() {

  int ikIter = 0;

  // Check error
  bool err = (((qsd_ - qsb_).norm() > taskErrThreshold_) ||
              errTwist_.norm() > closureErrThreshold_);
  while (err && ikIter < maxItr) {

    // Update Jacobians
    thetalist_ << qp_, qsb_;
    Eigen::MatrixXd rJ = AffordanceUtil::JacobianSpace(slist_, thetalist_);
    Np_ = rJ.leftCols(rJ.cols() - taskOffset);
    Ns_ = rJ.rightCols(taskOffset);

    // Compute primary joint angles
    Eigen::MatrixXd qp_dot = (qp_ - oldqp_) / dt_;

    //* Calculate constraint Jacobian, cJ
    Eigen::MatrixXd pinv_Ns; // pseudo-inverse of Ns
    pinv_Ns = Ns_.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd pinv_qp_dot; // pseudo-inverse of qp_dot
    pinv_qp_dot = qp_dot.completeOrthogonalDecomposition().pseudoInverse();

    Eigen::MatrixXd cJ = -pinv_Ns * (Np_ + errTwist_ * pinv_qp_dot);

    //*Calculate the update for qp
    oldqp_ = qp_; // store last joint values first
    Eigen::MatrixXd pinv_cJ;
    pinv_cJ = cJ.completeOrthogonalDecomposition()
                  .pseudoInverse(); // pseudo-inverse of cJ

    qp_ = qp_ + pinv_cJ * (qsd_ - qsb_); // Update using Newton-Raphson

    // Correct for closure error
    CcAffordancePlanner::closure_error_optimizer();

    // Update IK stepper
    ikIter = ikIter + 1;

    // Check error
    err = (((qsd_ - qsb_).norm() > taskErrThreshold_) ||
           errTwist_.norm() > closureErrThreshold_);
  }
  return !err;
}

void CcAffordancePlanner::closure_error_optimizer() {

  // Calculate the closure error using forward kinematics
  thetalist_ << qp_, qsb_;
  Eigen::Matrix4d Tse = AffordanceUtil::FKinSpace(
      mErr_, slist_, thetalist_); // HTM of actual end of ground link
  errTwist_ =
      AffordanceUtil::Adjoint(Tse) *
      AffordanceUtil::se3ToVec(AffordanceUtil::MatrixLog6(
          AffordanceUtil::TransInv(Tse) *
          mErr_)); // mErr_ is the desired htm for the end of ground link

  //* Adjust qp_ and qs_ based on this error;
  Eigen::MatrixXd N(Np_.rows(), Np_.cols() + Ns_.cols());
  N << Np_, Ns_; // Combine Np_ and Ns_ horizontally
  Eigen::MatrixXd pinv_N;
  pinv_N = N.completeOrthogonalDecomposition()
               .pseudoInverse();                // pseudo-inverse of N
  const Eigen::VectorXd q = pinv_N * errTwist_; // correction differential

  qp_ = qp_ + q.head(q.size() - taskOffset);
  qsb_ = qsb_ + q.tail(taskOffset);

  // Compute final error
  thetalist_ << qp_, qsb_;
  Tse = AffordanceUtil::FKinSpace(mErr_, slist_, thetalist_);
  errTwist_ = AffordanceUtil::Adjoint(Tse) *
              AffordanceUtil::se3ToVec(AffordanceUtil::MatrixLog6(
                  AffordanceUtil::TransInv(Tse) * mErr_));
}
