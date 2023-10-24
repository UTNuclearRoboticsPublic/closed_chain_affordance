#include <affordance_util/affordance_util.hpp>

AffordanceUtil::AffordanceUtil() {}
Eigen::MatrixXd AffordanceUtil::Adjoint(const Eigen::Isometry3d &htm) {
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
AffordanceUtil::JacobianSpace(const Eigen::MatrixXd &Slist,
                              const Eigen::VectorXd &thetalist) {
  int n = Slist.cols();
  Eigen::MatrixXd Js(6, n);
  /* Eigen::Matrix4d T = Eigen::Matrix4d::Identity(); */
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  for (int i = 0; i < n; i++) {
    Js.col(i) = Adjoint(T) * Slist.col(i);
    T *= MatrixExp6(VecTose3(Slist.col(i) * thetalist(i)));
  }

  return Js;
}

Eigen::Matrix4d AffordanceUtil::MatrixExp6(const Eigen::Matrix4d &se3mat) {

  // Compute the 3-vector exponential coordinate form of the rotation matrix
  const Eigen::Matrix3d so3mat = se3mat.block<3, 3>(0, 0);
  const Eigen::Vector3d &omgtheta = so3ToVec(so3mat);

  // If the norm of the 3-vector exponential coordinate form is very small,
  // return the rotation part as identity and the translation part as is from
  // se3mat
  const double expNormThreshold = 1e-6;
  if (omgtheta.norm() < expNormThreshold) {
    return (Eigen::Matrix4d() << Eigen::Matrix3d::Identity(),
            se3mat.block<3, 1>(0, 3), 0, 0, 0, 1)
        .finished();
  } else {

    // Else compute the HTM using the Rodriguez formula
    const auto &[ignore, theta] = AxisAng3(omgtheta);
    const Eigen::Matrix3d omgmat = so3mat / theta;
    const Eigen::Matrix3d &R = MatrixExp3(so3mat);
    const Eigen::Vector3d p =
        (Eigen::Matrix3d::Identity() * theta + (1 - cos(theta)) * omgmat +
         (theta - sin(theta)) * omgmat * omgmat) *
        (se3mat.block<3, 1>(0, 3) / theta);
    return (Eigen::Matrix4d() << R, p, 0, 0, 0, 1).finished();
  }
}
Eigen::Matrix3d AffordanceUtil::MatrixExp3(const Eigen::Matrix3d &so3mat) {
  // Compute the 3-vector exponential coordinate form of the 3x3 skew-symmetric
  // matrix so3mat
  const Eigen::Vector3d &omgtheta = so3ToVec(so3mat);

  // If the norm of the 3-vector exponential coordinate form is very small,
  // return rotation matrix as identity
  const double expNormThreshold = 1e-6;
  if (omgtheta.norm() < expNormThreshold) {
    return Eigen::Matrix3d::Identity();
  } else {
    // Else compute the rotation matrix using the Rodriguez formula
    const auto &[ignore, theta] = AxisAng3(omgtheta);
    const Eigen::Matrix3d omgmat = so3mat / theta;
    return Eigen::Matrix3d::Identity() + sin(theta) * omgmat +
           (1 - cos(theta)) * omgmat * omgmat;
  }
}
Eigen::Vector3d AffordanceUtil::so3ToVec(const Eigen::Matrix3d &so3mat) {
  // Extract and return the vector from the skew-symmetric matrix so3mat
  return Eigen::Vector3d(so3mat(2, 1), so3mat(0, 2), so3mat(1, 0));
}

std::tuple<Eigen::Vector3d, double>
AffordanceUtil::AxisAng3(const Eigen::Vector3d &expc3) {
  // Angle is simply the norm of the 3-vector exponential coordinates of
  // rotation
  const double theta = expc3.norm();

  // Axis is simply the 3-vector exponential coordinates of rotation normalized
  // by the angle
  const Eigen::Vector3d omghat = expc3 / theta;
  return std::make_tuple(omghat, theta);
}

Eigen::Matrix4d
AffordanceUtil::FKinSpace(const Eigen::Matrix4d &M,
                          const std::vector<Eigen::VectorXd> &Slist,
                          const Eigen::VectorXd &thetalist) {
  Eigen::Matrix4d T = M;
  for (int i = thetalist.size() - 1; i >= 0; --i) {
    Eigen::Matrix4d expMat = MatrixExp6(VecTose3(Slist[i] * thetalist(i)));
    T = expMat * T;
  }
  return T;
}

Eigen::Matrix4d AffordanceUtil::VecTose3(const Eigen::VectorXd &V) {
  // Extract the rotation part of the vector, V and get it's skew-symmetric form
  const Eigen::Matrix3d omgmat = VecToso3(V.segment<3>(0));

  // Extract the translation part of the vector, V
  const Eigen::Vector3d v(V.segment<3>(3));

  // Start the 4x4 se3 matrix as an Identity. Then, fill out the rotation and
  // translation parts
  Eigen::Matrix4d se3mat = Eigen::Matrix4d::Zero();
  se3mat.block<3, 3>(0, 0) = omgmat;
  se3mat.block<3, 1>(0, 3) = v;

  return se3mat;
}

Eigen::Matrix3d AffordanceUtil::VecToso3(const Eigen::Vector3d &omg) {
  // Fill out the 3x3 so3 matrix as the skew-symmetric form of the passed
  // 3-vector, omg
  const Eigen::Matrix3d so3mat = (Eigen::Matrix3d() << 0, -omg(2), omg(1),
                                  omg(2), 0, -omg(0), -omg(1), omg(0), 0)
                                     .finished();
  return so3mat;
}
