#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cc_affordance_planner/cc_affordance_planner.hpp>

// This tester includes hardcoded UR5 robot info to test the CC Affordance
// planner
/*
   Author: Crasun Jans
*/
int main() {

  //** Geometry
  const double mconv = 1000.0; // conversion factor from mm to m
  const double W1 = 109.0 / mconv, W2 = 82.0 / mconv, L1 = 425.0 / mconv,
               L2 = 392.0 / mconv, H1 = 89.0 / mconv, H2 = 95.0 / mconv,
               W3 = 135.85 / mconv, W4 = 119.7 / mconv,
               W6 = 93.0 / mconv;    // Link lengths
  const double aff = -100.0 / mconv; // affordance location from the last joint

  const size_t nofJoints = 6, nofVirtualJoints = 3,
               nofAffordance = 1; // number of joints
  const size_t nofJointsTotal = nofJoints + nofVirtualJoints + nofAffordance;

  //** Creation of screw axes
  // Screw axis locations from base frame in home position
  Eigen::MatrixXd q(3, nofJointsTotal);
  q.col(0) << 0, 0, H1;
  q.col(1) << 0, W3, H1;
  q.col(2) << L1, W3 - W4, H1;
  q.col(3) << L1 + L2, W3 - W4, H1;
  q.col(4) << L1 + L2, W3 - W4 + W6, H1;
  q.col(5) << L1 + L2, W3 - W4 + W6, H1 - H2;
  q.col(6) << L1 + L2, W3 - W4 + W6 + W2, H1 - H2; // Imaginary joint
  q.col(7) << L1 + L2, W3 - W4 + W6 + W2, H1 - H2; // Imaginary joint
  q.col(8) << L1 + L2, W3 - W4 + W6 + W2, H1 - H2; // Imaginary joint
  q.col(9) << L1 + L2, W3 - W4 + W6 + W2 + aff,
      H1 - H2; // Location of affordance frame

  // Type and alignment of screw axes
  Eigen::MatrixXd w(3, 10);
  w.col(0) << 0, 0, 1;
  w.col(1) << 0, 1, 0;
  w.col(2) << 0, 1, 0;
  w.col(3) << 0, 1, 0;
  w.col(4) << 0, 0, -1;
  w.col(5) << 0, 1, 0;
  w.col(6) << 1, 0, 0; // Imaginary joint
  w.col(7) << 0, 1, 0; // Imaginary joint
  w.col(8) << 0, 0, 1; // Imaginary joint
  w.col(9) << 1, 0, 0; // Affordance

  Eigen::MatrixXd slist(6, nofJointsTotal);

  // Construct screw axes and frames
  for (int i = 0; i < w.cols(); i++) {
    Eigen::Vector3d wcurr =
        w.col(i); // required to convert to VectorXd type for use with cross
    Eigen::Vector3d qcurr =
        q.col(i); // required to convert to VectorXd type for use with cross
    slist.col(i) << wcurr, -wcurr.cross(qcurr);
  }

  // Output screw axes for debugging purposes
  /* for (size_t i = 0; i < nofJointsTotal; i++) { */
  /*   std::cout << "slist.col(" << i << "):\n" << slist.col(i) << "\n"; */
  /* } */
  std::cout << "Here is the list of screws: \n" << slist << std::endl;
  ;

  const Eigen::Matrix4d mErr = Eigen::Matrix4d::Identity(); // Error frame
  const Eigen::Matrix4d Tsd = mErr; // Desired closure frame is the error frame
  // Start angles
  const Eigen::VectorXd thetalist0 = Eigen::VectorXd::Zero(10);
  const double affGoal = 0.3;

  // Construct the planner object
  CcAffordancePlanner ccAffordancePlanner(slist, thetalist0, affGoal);

  // Set algorithm parameters
  ccAffordancePlanner.affStep = 0.1;
  ccAffordancePlanner.accuracy = 1.0 * (1.0 / 100);

  // Run the planner
  PlannerResult plannerResult = ccAffordancePlanner.affordance_stepper();

  // Print the first point in the trajectory if planner succeeds
  if (plannerResult.success) {
    std::vector<Eigen::VectorXd> solution = plannerResult.jointTraj;
    std::cout << "Planner succeeded with " << plannerResult.trajFullorPartial
              << " solution. and here is the first point in the trajectory \n"
              << solution.at(0) << std::endl;
  } else {
    std::cout << "No solution found" << std::endl;
  }
  return 0;
}
