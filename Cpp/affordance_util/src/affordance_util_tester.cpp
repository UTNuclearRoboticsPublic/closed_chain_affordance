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
  return 0;
}
