#include <Eigen/Core>
#include <affordance_util/affordance_util.hpp>
#include <iomanip> // for std::precision
#include <iostream>

int main() {

  // Precision for testing
  std::cout << std::fixed
            << std::setprecision(4); // Display up to 4 decimal places

  AffordanceUtil affordanceUtil;
  std::cout << "Testing VecToSo3" << std::endl;
  const Eigen::Vector3d omg(1.0, 2.0, 3.0);
  std::cout << "Input: \n" << omg << std::endl;
  const Eigen::Matrix3d &so3mat = affordanceUtil.VecToso3(omg);
  std::cout << "Output: \n" << so3mat << std::endl;

  std::cout << "\nTesting so3ToVec" << std::endl;
  std::cout << "Input: \n" << so3mat << std::endl;
  std::cout << "Output: \n" << affordanceUtil.so3ToVec(so3mat) << std::endl;

  std::cout << "\nTesting VecTose3" << std::endl;
  const Eigen::VectorXd V = (Eigen::VectorXd(6) << 1, 2, 3, 4, 5, 6).finished();
  std::cout << "Input: \n" << V << std::endl;
  std::cout << "Output: \n" << affordanceUtil.VecTose3(V) << std::endl;

  std::cout << "\nTesting AxisAng3" << std::endl;
  const Eigen::Vector3d expc3 = (Eigen::Vector3d() << 1, 2, 3).finished();
  std::cout << "Input: \n" << expc3 << std::endl;
  const auto &[expc3Axis, expc3Angle] = affordanceUtil.AxisAng3(expc3);
  std::cout << "Output axis: \n" << expc3Axis << std::endl;
  std::cout << "Output angle: \n" << expc3Angle << std::endl;

  std::cout << "\nTesting MatrixExp3" << std::endl;
  std::cout << "Input: \n" << so3mat << std::endl;
  std::cout << "Output: \n" << affordanceUtil.MatrixExp3(so3mat) << std::endl;

  const Eigen::Matrix4d se3mat =
      (Eigen::Matrix4d() << 0, 0, 0, 0, 0, 0, -1.5708, 2.3562, 0, 1.5708, 0,
       2.3562, 0, 0, 0, 0)
          .finished();
  std::cout << "\nTesting MatrixExp6" << std::endl;
  std::cout << "Input: \n" << se3mat << std::endl;
  std::cout << "Output: \n" << affordanceUtil.MatrixExp6(se3mat) << std::endl;

  const Eigen::Matrix4d M =
      (Eigen::Matrix4d() << -1, 0, 0, 0, 0, 1, 0, 6, 0, 0, -1, 2, 0, 0, 0, 1)
          .finished();

  Eigen::MatrixXd Slist(6, 3);
  Slist << 0, 0, 0, 0, 0, 0, 1, 0, -1, 4, 0, -6, 0, 1, 0, 0, 0, -0.1;

  Eigen::Vector3d thetalist =
      (Eigen::Vector3d() << M_PI / 2, 3, M_PI).finished();
  std::cout << "\nTesting FKinSpace" << std::endl;
  std::cout << "Input M: \n" << M << std::endl;
  std::cout << "Input Slist: \n" << Slist << std::endl;
  std::cout << "Input thetalist: \n" << thetalist << std::endl;
  std::cout << "Output: \n"
            << affordanceUtil.FKinSpace(M, Slist, thetalist) << std::endl;

  Eigen::MatrixXd Slist2(6, 4);

  Slist2 << 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 2, 0, 0.2, 0.2, 0, 2, 0.3,
      0.2, 3, 1, 0.4;
  Eigen::VectorXd thetalist2(4);
  thetalist2 << 0.2, 1.1, 0.1, 1.2;
  std::cout << "\nTesting JacobianSpace" << std::endl;
  std::cout << "Input Slist: \n" << Slist2 << std::endl;
  std::cout << "Input thetalist: \n" << thetalist2 << std::endl;
  std::cout << "Output: \n"
            << affordanceUtil.JacobianSpace(Slist2, thetalist2) << std::endl;

  Eigen::Matrix4d htm;
  htm << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 3, 0, 0, 0, 1;
  std::cout << "\nTesting Adjoint" << std::endl;
  std::cout << "Input htm: \n" << htm << std::endl;
  std::cout << "Output: \n" << affordanceUtil.Adjoint(htm) << std::endl;

  std::cout << "\nTesting TransInv" << std::endl;
  std::cout << "Input htm: \n" << htm << std::endl;
  std::cout << "Output: \n" << affordanceUtil.TransInv(htm) << std::endl;

  Eigen::Matrix4d se3mat2;
  se3mat2 << 0, -3, 2, 4, 3, 0, -1, 5, -2, 1, 0, 6, 0, 0, 0, 0;
  std::cout << "\nTesting se3ToVec" << std::endl;
  std::cout << "Input se3mat: \n" << se3mat2 << std::endl;
  std::cout << "Output: \n" << affordanceUtil.se3ToVec(se3mat2) << std::endl;

  const Eigen::Matrix3d R =
      (Eigen::Matrix3d() << 0, 0, 1, 1, 0, 0, 0, 1, 0).finished();
  std::cout << "\nTesting MatrixLog3" << std::endl;
  std::cout << "Input R: \n" << R << std::endl;
  std::cout << "Output: \n" << affordanceUtil.MatrixLog3(R) << std::endl;

  const double near = -1e-7;
  std::cout << "\nTesting NearZero" << std::endl;
  std::cout << "Input near: \n" << near << std::endl;
  std::cout << "Output: \n" << affordanceUtil.NearZero(near) << std::endl;

  const Eigen::Matrix4d T =
      (Eigen::Matrix4d() << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 3, 0, 0, 0, 1)
          .finished();
  std::cout << "\nTesting MatrixLog6" << std::endl;
  std::cout << "Input T: \n" << T << std::endl;
  std::cout << "Output: \n" << affordanceUtil.MatrixLog6(T) << std::endl;

  return 0;
}
