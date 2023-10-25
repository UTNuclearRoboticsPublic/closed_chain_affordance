/*
   Author: Crasun Jans
   Credits: Some functions are translated to Cpp from Kevin Lynch's Modern
   Robotics Matlab package
*/
#ifndef AFFORDANCE_UTIL
#define AFFORDANCE_UTIL

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

class AffordanceUtil {
public:
  AffordanceUtil();
  /**
   * @brief Given a homogenenous transformation matrix, computes its adjoint
   * representation
   *
   * @param htm Homogeneous transformation matrix
   *
   * @return
   */
  Eigen::MatrixXd Adjoint(const Eigen::Matrix4d &htm);

  /**
   * @brief Given a list of space-form screws and corresponding magnitudes,
   * computes the space-form Jacobian
   *
   * @param Slist Space-form screws as columns of a matrix
   * @param thetalist Screw magnitudes as a column vector
   *
   * @return Space-form Jacobian
   */
  Eigen::MatrixXd JacobianSpace(const Eigen::MatrixXd &Slist,
                                const Eigen::VectorXd &thetalist);
  /**
   * @brief Given a 4x4 se(3) representation of exponential coordinates, returns
   * a T matrix in SE(3) that is achieved by traveling along/about the screw
   * axis S for a distance theta from an initial configuration T = I.
   *
   * @param se3mat 4x4 skew-symmetric representation of exponential coordinates
   *
   * @return 4x4 homogenous transformation matrix
   */
  Eigen::Matrix4d MatrixExp6(const Eigen::Matrix4d &se3mat);

  /**
   * @brief Given a 3x3 so(3) representation of exponential coordinates, returns
   * R in SO(3) that is achieved by rotating about omghat by theta from an
   * initial orientation R = I.
   *
   * @param so3mat 3x3 skew-symmetric representation of exponential coordinates
   *
   * @return 3x3 Rotation matrix
   */
  Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d &so3mat);
  /**
   * @brief Given a 3x3 skew-symmetric matrix (an element of so(3)), returns the
   * corresponding 3-vector (angular velocity).
   *
   * @param so3mat 3x3 skew-symmetric matrix
   *
   * @return 3-vector angular velocity
   */
  Eigen::Vector3d so3ToVec(const Eigen::Matrix3d &so3mat);
  /**
   * @brief Given a 3-vector of exponential coordinates for rotation, returns
   * the unit rotation axis omghat and the corresponding rotation angle theta.
   *
   * @param expc3 3-vector exponential coordinates for rotation
   *
   * @return Tuple of 3-vector axis and corresponding angle
   */
  std::tuple<Eigen::Vector3d, double> AxisAng3(const Eigen::Vector3d &expc3);
  /**
   * @brief Computes the space-form forward kinematics of the robot
   *
   * @param M Home configuration (position and orientation) of the
   * end-effector
   *
   * @param Slist Joint screw axes in the space frame when the manipulator
   * is at the home position
   *
   * @param thetalist List of joint coordinates
   *
   * @return T in SE(3) representing the end-effector frame, when the joints are
   * at the specified coordinates (i.t.o Space Frame)
   */
  Eigen::Matrix4d FKinSpace(const Eigen::Matrix4d &M,
                            const Eigen::MatrixXd &Slist,
                            const Eigen::VectorXd &thetalist);
  /**
   * @brief Given a 6-vector (representing a spatial velocity), returns the
   * corresponding 4x4 se(3) matrix.
   *
   * @param V 6-vector spatial velocity
   *
   * @return 4x4 se(3) matrix
   */
  Eigen::Matrix4d VecTose3(const Eigen::VectorXd &V);
  /**
   * @brief Given a 3-vector (angular velocity), returns the skew symmetric
   * matrix in so(3).
   *
   * @param omg 3-vector angular velocity
   *
   * @return skew-symmetric representation of the passed 3-vector
   */
  Eigen::Matrix3d VecToso3(const Eigen::Vector3d &omg);
};
#endif // AFFORDANCE_UTIL
