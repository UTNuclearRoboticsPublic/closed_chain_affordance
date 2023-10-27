#include <affordance_util/affordance_util.hpp>

AffordanceUtil::AffordanceUtil() {}
Eigen::MatrixXd AffordanceUtil::Adjoint(const Eigen::Matrix4d &htm) {
  Eigen::MatrixXd adjoint(6, 6); // Output

  // Extract the rotation matrix (3x3) and translation vector (3x1) from htm
  Eigen::Matrix3d rotationMatrix = htm.block<3, 3>(0, 0);
  Eigen::Vector3d translationVector = htm.block<3, 1>(0, 3);

  // Construct the bottom-left 3x3 part
  Eigen::Matrix3d botLeft = VecToso3(translationVector) * rotationMatrix;

  // Build the adjoint matrix
  adjoint << rotationMatrix, Eigen::Matrix3d::Zero(), botLeft, rotationMatrix;

  return adjoint;
}
Eigen::MatrixXd
AffordanceUtil::JacobianSpace(const Eigen::MatrixXd &Slist,
                              const Eigen::VectorXd &thetalist) {

  const int jacColSize = thetalist.size();
  Eigen::MatrixXd Js(6, jacColSize);
  Js.col(0) = Slist.col(0); // first column is simply the first screw axis
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

  // Compute the Jacobian using the POE formula and adjoint representation
  for (int i = 1; i < jacColSize; i++) {
    T *= MatrixExp6(VecTose3(Slist.col(i - 1) * thetalist(i - 1)));
    Js.col(i) = Adjoint(T) * Slist.col(i);
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
  if (AffordanceUtil::NearZero(omgtheta.norm())) {
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
  if (AffordanceUtil::NearZero(omgtheta.norm())) {
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

Eigen::Matrix4d AffordanceUtil::FKinSpace(const Eigen::Matrix4d &M,
                                          const Eigen::MatrixXd &Slist,
                                          const Eigen::VectorXd &thetalist) {
  // Compute space-form forward kinematics using the product of exponential
  // formula
  Eigen::Matrix4d T = M;
  for (int i = thetalist.size() - 1; i >= 0; --i) {
    Eigen::Matrix4d expMat = MatrixExp6(VecTose3(Slist.col(i) * thetalist(i)));
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

Eigen::Matrix4d AffordanceUtil::TransInv(const Eigen::Matrix4d &T) {

  // Extract rotation matrix and translation vector
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  Eigen::Vector3d p = T.block<3, 1>(0, 3);

  // Calculate the inverse transformation matrix
  Eigen::Matrix4d invT;
  Eigen::Matrix3d invR = R.transpose();
  Eigen::Vector3d invP = -invR * p;
  invT << invR, invP, 0, 0, 0, 1;

  return invT;
}

Eigen::VectorXd AffordanceUtil::se3ToVec(const Eigen::Matrix4d &se3mat) {
  // Extract and construct the vector from the skew-symmetric matrix
  const Eigen::VectorXd V = (Eigen::VectorXd(6) << se3mat(2, 1), se3mat(0, 2),
                             se3mat(1, 0), se3mat.block<3, 1>(0, 3))
                                .finished();

  return V;
}

Eigen::Matrix3d AffordanceUtil::MatrixLog3(const Eigen::Matrix3d &R) {

  const double acosinput = (R.trace() - 1) / 2;
  Eigen::Matrix3d so3mat;

  if (acosinput >= 1) {
    so3mat.setZero();
  } else if (acosinput <= -1) {
    Eigen::Vector3d omg;
    if (!AffordanceUtil::NearZero(1 + R(2, 2))) {
      omg = (1 / sqrt(2 * (1 + R(2, 2)))) *
            Eigen::Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
    } else if (!AffordanceUtil::NearZero(1 + R(1, 1))) {
      omg = (1 / sqrt(2 * (1 + R(1, 1)))) *
            Eigen::Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
    } else {
      omg = (1 / sqrt(2 * (1 + R(0, 0)))) *
            Eigen::Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
    }
    so3mat = VecToso3(M_PI * omg);
  } else {
    const double theta = acos(acosinput);
    so3mat = theta * (1 / (2 * sin(theta))) * (R - R.transpose());
  }

  return so3mat;
}

Eigen::Matrix4d AffordanceUtil::MatrixLog6(const Eigen::Matrix4d &T) {
  const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  const Eigen::Vector3d p = T.block<3, 1>(0, 3);
  const Eigen::Matrix3d omgmat = MatrixLog3(R);

  Eigen::Matrix<double, 4, 4> expmat;

  if (omgmat.isApprox(Eigen::Matrix3d::Zero())) {
    expmat.block<3, 3>(0, 0) = Eigen::Matrix3d::Zero();
    expmat.block<3, 1>(0, 3) = p;
    expmat.block<1, 4>(3, 0) = Eigen::Matrix<double, 1, 4>::Zero();
  } else {
    const double theta = acos((R.trace() - 1) / 2);
    const Eigen::Matrix3d eye3 = Eigen::Matrix3d::Identity();
    expmat.block<3, 3>(0, 0) = omgmat;
    expmat.block<3, 1>(0, 3) = (eye3 - 0.5 * omgmat +
                                (1.0 / theta - 1.0 / (2 * tan(0.5 * theta))) *
                                    omgmat * omgmat / theta) *
                               p;
    expmat.block<1, 4>(3, 0) = Eigen::Matrix<double, 1, 4>::Zero();
  }

  return expmat;
}
bool AffordanceUtil::NearZero(const double &near) {
  return std::abs(near) < nearZeroTol_;
}
