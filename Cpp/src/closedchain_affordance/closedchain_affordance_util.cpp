#include <closedchain_affordance/closedchain_affordance_util.hpp>

ClosedChainAffordanceUtil::ClosedChainAffordanceUtil() {}
ClosedChainAffordanceUtil::Adjoint(const Eigen::Isometry3d &htm) {
  Eigen::MatrixXd adjoint(6, 6); // Output

  // Extract the rotation matrix (3x3) and translation vector (3x1) from htm
  Eigen::Matrix3d rotationMatrix = htm.linear();
  Eigen::Vector3d translationVector = htm.translation();

  // Construct the skew-symmetric matrix (3x3) for the translation vector
  Eigen::Matrix3d skewSymmetricMatrix;
  skewSymmetricMatrix << 0, -translationVector(2), translationVector(1),
      translationVector(2), 0, -translationVector(0), -translationVector(1),
      translationVector(0), 0;

  // Build the adjoint matrix
  adjoint << rotationMatrix, Eigen::Matrix3d::Zero(), skewSymmetricMatrix,
      rotationMatrix; // htm.linear() is simply the rotation part of the htm

  return adjoint;
}
Eigen::MatrixXd
ClosedChainAffordanceUtil::JacobianSpace(const MatrixXd &Slist,
                                         const VectorXd &thetalist) {
  int n = Slist.cols();
  Eigen::MatrixXd Js(6, n);
  Eigen::Matrix4d T = Matrix4d::Identity();

  for (int i = 0; i < n; i++) {
    Js.col(i) = Adjoint(T) * Slist.col(i);
    T *= MatrixExp6(VecTose3(Slist.col(i) * thetalist(i)));
  }

  return Js;
}

Eigen::Matrix4d
ClosedChainAffordanceUtil::MatrixExp6(const Eigen::Matrix4d &se3mat) {
  Eigen::Matrix3d omgtheta = so3ToVec(se3mat.block<3, 3>(0, 0));
  if (omgtheta.isZero()) {
    return (Eigen::Matrix4d() << Eigen::Matrix3d::Identity(),
            se3mat.block<3, 1>(0, 3), 0, 0, 0, 1)
        .finished();
  } else {
    Eigen::Vector3d omghat;
    double theta;
    std::tie(omghat, theta) = AxisAng3(omgtheta);
    Eigen::Matrix3d omgmat = se3mat.block<3, 3>(0, 0) / theta;
    Eigen::Matrix3d R = MatrixExp3(se3mat.block<3, 3>(0, 0));
    Eigen::Matrix3d V =
        (Eigen::Matrix3d::Identity() * theta + (1 - cos(theta)) * omgmat +
         (theta - sin(theta)) * omgmat * omgmat);
    Eigen::Vector3d p = V * se3mat.block<3, 1>(0, 3) / theta;
    return (Eigen::Matrix4d() << R, p, 0, 0, 0, 1).finished();
  }
}
Matrix3d ClosedChainAffordanceUtil::MatrixExp3(const Matrix3d &so3mat) {
  Vector3d omgtheta = so3ToVec(so3mat);
  if (omgtheta.norm() < 1e-6) {
    return Matrix3d::Identity();
  } else {
    Vector3d omghat;
    double theta;
    AxisAng3(so3mat, omghat, theta);
    Matrix3d omgmat = so3mat / theta;
    return Matrix3d::Identity() + sin(theta) * omgmat +
           (1 - cos(theta)) * omgmat * omgmat;
  }
}
Eigen::Vector3d
ClosedChainAffordanceUtil::so3ToVec(const Eigen::Matrix3d &so3mat) {
  return Eigen::Vector3d(so3mat(2, 1), so3mat(0, 2), so3mat(1, 0));
}

std::tuple<Eigen::Vector3d, double>
ClosedChainAffordanceUtil::AxisAng3(const Eigen::Vector3d &expc3) {
  double theta = expc3.norm();
  Eigen::Vector3d omghat = expc3 / theta;
  return std::make_tuple(omghat, theta);
}

Eigen::Matrix4d
ClosedChainAffordanceUtil::FKinSpace(const Eigen::Matrix4d &M,
                                     const std::vector<Eigen::Vector6d> &Slist,
                                     const Eigen::VectorXd &thetalist) {
  Eigen::Matrix4d T = M;
  for (int i = thetalist.size() - 1; i >= 0; --i) {
    Eigen::Matrix4d expMat = MatrixExp6(VecTose3(Slist[i] * thetalist(i)));
    T = expMat * T;
  }
  return T;
}

ClosedChainAffordanceUtil::Eigen::Matrix4d VecTose3(const Eigen::Vector6d &V) {
  Eigen::Matrix3d omgmat = VecToso3(V.segment<3>(0));
  Eigen::Vector3d v(V.segment<3>(3));

  Eigen::Matrix4d se3mat;
  se3mat << omgmat, v, 0, 0, 0, 0;

  return se3mat;
}

Eigen::Matrix3d
ClosedChainAffordanceUtil::VecToso3(const Eigen::Vector3d &omg) {
  Eigen::Matrix3d so3mat;
  so3mat << 0, -omg(2), omg(1), omg(2), 0, -omg(0), -omg(1), omg(0), 0;
  return so3mat;
}
