#include <affordance_util/affordance_util.hpp>

namespace YAML
{
template <int N> bool convert<Eigen::Matrix<double, N, 1>>::decode(const Node &node, Eigen::Matrix<double, N, 1> &vec)
{
    if (!node.IsSequence() || node.size() != N)
    {
        return false; // Invalid YAML node for Eigen::VectorXd
    }

    // Fill out the elements of the Eigen::Vector one-by-one
    for (std::size_t i = 0; i < N; ++i)
    {
        vec(i) = node[i].as<double>();
    }

    return true; // Successfully decoded Eigen::VectorXd
}

template <int N> Node convert<Eigen::Matrix<double, N, 1>>::encode(const Eigen::Matrix<double, N, 1> &vec)
{
    Node node;
    if (vec.size() != N)
    {
        // Handle the case where the vector size doesn't match the expected size
        throw std::invalid_argument("Invalid vector size for encoding.");
    }

    for (int i = 0; i < N; ++i)
    {
        node.push_back(vec(i));
    }
    return node;
}
} // namespace YAML

namespace affordance_util
{

Eigen::MatrixXd compose_cc_model_slist(const Eigen::MatrixXd &robot_slist, const Eigen::VectorXd &thetalist,
                                       const Eigen::Matrix4d &M, ScrewInfo &aff, const std::string &vir_screw_order)
{
    try
    {
        // Compute robot Jacobian
        Eigen::MatrixXd robot_jacobian = JacobianSpace(robot_slist, thetalist);
        Eigen::MatrixXd slist;

        // If aff screw is not set, compute it
        if (aff.screw.isZero())
        {
            aff.screw = affordance_util::get_screw(aff);
        }

        // If pure rotation, the motion of the last closed-chain joint is in the opposite direction of the affordance
        // since the ground link is fixed and it is the affordance link that moves instead
        Eigen::VectorXd aff_screw(6);

        if (aff.type == "rotation")
        {
            aff_screw = -aff.screw;
        }
        else
        {

            aff_screw = aff.screw;
        }

        if (vir_screw_order == "none")
        {
            const size_t nof_sjoints = 1; // 1 for one affordance
            slist.conservativeResize(robot_slist.rows(), (robot_slist.cols() + nof_sjoints));
            slist << robot_jacobian, aff_screw;
        }
        else
        {
            const size_t screw_length = 6;                    // Length of the screw vector
            const size_t screw_axis_length = 3;               // Length of the screw axis
            const size_t nof_vir_ee_joints = 3;               // Number of virtual ee joints
            const size_t nof_sjoints = nof_vir_ee_joints + 1; // Number of joints to be appended, + 1 for one affordance

            // Append virtual EE screw axes as well as the affordance screw.
            // Note: In the future it might be desired to append the virtual EE screws as Jacobians as well. This would
            // allow one to track the orientation of the gripper as the magnitudes (joint angles) of these screws.
            // Currently, we model the Virtual EE screws as aligned with the space frame at the start pose of the
            // affordance. An advantage of this is physical intuition in controlling the gripper.
            Eigen::MatrixXd app_slist(screw_length, nof_sjoints);

            // Extract robot palm location
            const Eigen::Matrix4d ee_htm = FKinSpace(M, robot_slist, thetalist);
            const Eigen::Vector3d q_vir = ee_htm.block<3, 1>(0, 3); // Translation part of the HTM

            // Virtual EE screw axes
            Eigen::Matrix<double, screw_axis_length, nof_vir_ee_joints> w_vir; // Virtual EE screw axes

            // Assign virtual EE screw axes in requested order
            if (vir_screw_order == "yzx")
            {
                w_vir.col(0) << 0, 1, 0; // y
                w_vir.col(1) << 0, 0, 1; // z
                w_vir.col(2) << 1, 0, 0; // x
            }
            else if (vir_screw_order == "zxy")
            {
                w_vir.col(0) << 0, 0, 1; // z
                w_vir.col(1) << 1, 0, 0; // x
                w_vir.col(2) << 0, 1, 0; // y
            }
            else if (vir_screw_order == "xyz")
            {
                w_vir.col(0) << 1, 0, 0; // x
                w_vir.col(1) << 0, 1, 0; // y
                w_vir.col(2) << 0, 0, 1; // z
            }
            else
            {

                throw std::runtime_error("Invalid virtual screw order specified while composing cc model slist");
            }

            // Compute virtual EE screws
            for (size_t i = 0; i < nof_vir_ee_joints; ++i)
            {
                app_slist.col(i) = get_screw(w_vir.col(i), q_vir);
            }

            // Affordance screw
            app_slist.col(nof_sjoints - 1) = aff_screw;

            // Altogether
            slist.conservativeResize(robot_slist.rows(), (robot_slist.cols() + nof_sjoints));
            slist << robot_jacobian, app_slist;
        }

        return slist;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception occured while composing cc model slist: " << e.what() << std::endl;
        return Eigen::MatrixXd();
    }
}

RobotConfig robot_builder(const std::string &config_file_path)
{
    try
    {

        RobotConfig robotConfig; // Output of the function

        // Load the YAML file
        const YAML::Node config = YAML::LoadFile(config_file_path);
        if (!config)
        {
            throw std::runtime_error("Robot screw list cannot be built without a valid robot config yaml file");
        }

        // Access the reference frame info
        const YAML::Node &refFrameNode = config["ref_frame"];
        const std::string &ref_frame_name = refFrameNode[0]["name"].as<std::string>(); // access with [0] since only
                                                                                       // one reference frame

        // Access the 'joints' array
        const YAML::Node &jointsNode = config["joints"];

        // Parse each joint
        std::vector<JointData> jointsData;
        for (const YAML::Node &jointNode : jointsNode)
        {
            JointData joint;
            joint.name = jointNode["name"].as<std::string>();
            joint.w = jointNode["w"].as<Eigen::Vector3d>();
            joint.q = jointNode["q"].as<Eigen::Vector3d>();
            jointsData.push_back(joint);
        }

        // Access the tool info
        const YAML::Node &toolNode = config["tool"];
        const std::string &tool_name = toolNode[0]["name"].as<std::string>(); // access with [0] since only one tool

        // Compute screw axes
        const size_t screwSize = 6;
        const size_t &totalNofJoints = jointsData.size();
        Eigen::MatrixXd Slist(screwSize, totalNofJoints);

        for (size_t i = 0; i < totalNofJoints; i++)
        {
            const JointData &joint = jointsData[i];
            Slist.col(i) << joint.w, -joint.w.cross(joint.q);
            /* Start setting the output of the function */
            // Joint names
            robotConfig.joint_names.push_back(joint.name);
        }

        /* Fill out the remaining members of the output and return it*/
        // Screw list
        robotConfig.Slist = Slist;

        // EE homogenous transformation matrix
        const Eigen::Vector3d toolLocation = toolNode[0]["q"].as<Eigen::Vector3d>();
        Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
        M.block<3, 1>(0, 3) = toolLocation;
        robotConfig.M = M;

        // Reference frame name
        robotConfig.ref_frame_name = ref_frame_name;

        // Tool name
        robotConfig.tool_name = tool_name;

        return robotConfig;
    }
    catch (const YAML::Exception &e)
    {
        std::cerr << "Error reading YAML file: " << e.what() << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Standard exception: " << e.what() << std::endl;
    }
}
Eigen::MatrixXd Adjoint(const Eigen::Matrix4d &htm)
{
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
Eigen::MatrixXd JacobianSpace(const Eigen::MatrixXd &Slist, const Eigen::VectorXd &thetalist)
{

    const int jacColSize = thetalist.size();
    Eigen::MatrixXd Js(6, jacColSize);
    Js.col(0) = Slist.col(0); // first column is simply the first screw axis
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    // Compute the Jacobian using the POE formula and adjoint representation
    for (int i = 1; i < jacColSize; i++)
    {
        T *= MatrixExp6(VecTose3(Slist.col(i - 1) * thetalist(i - 1)));
        Js.col(i) = Adjoint(T) * Slist.col(i);
    }

    return Js;
}

Eigen::Matrix4d MatrixExp6(const Eigen::Matrix4d &se3mat)
{

    // Compute the 3-vector exponential coordinate form of the rotation matrix
    const Eigen::Matrix3d so3mat = se3mat.block<3, 3>(0, 0);
    const Eigen::Vector3d &omgtheta = so3ToVec(so3mat);

    // If the norm of the 3-vector exponential coordinate form is very small,
    // return the rotation part as identity and the translation part as is from
    // se3mat
    if (NearZero(omgtheta.norm()))
    {
        return (Eigen::Matrix4d() << Eigen::Matrix3d::Identity(), se3mat.block<3, 1>(0, 3), 0, 0, 0, 1).finished();
    }
    else
    {
        // Else compute the HTM using the Rodriguez formula
        const auto &[ignore, theta] = AxisAng3(omgtheta);
        const Eigen::Matrix3d omgmat = so3mat / theta;
        const Eigen::Matrix3d &R = MatrixExp3(so3mat);
        const Eigen::Vector3d p =
            (Eigen::Matrix3d::Identity() * theta + (1 - cos(theta)) * omgmat + (theta - sin(theta)) * omgmat * omgmat) *
            (se3mat.block<3, 1>(0, 3) / theta);
        return (Eigen::Matrix4d() << R, p, 0, 0, 0, 1).finished();
    }
}
Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d &so3mat)
{
    // Compute the 3-vector exponential coordinate form of the 3x3 skew-symmetric
    // matrix so3mat
    const Eigen::Vector3d &omgtheta = so3ToVec(so3mat);

    // If the norm of the 3-vector exponential coordinate form is very small,
    // return rotation matrix as identity
    if (NearZero(omgtheta.norm()))
    {
        return Eigen::Matrix3d::Identity();
    }
    else
    {
        // Else compute the rotation matrix using the Rodriguez formula
        const auto &[ignore, theta] = AxisAng3(omgtheta);
        const Eigen::Matrix3d omgmat = so3mat / theta;
        return Eigen::Matrix3d::Identity() + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat;
    }
}
Eigen::Vector3d so3ToVec(const Eigen::Matrix3d &so3mat)
{
    // Extract and return the vector from the skew-symmetric matrix so3mat
    return Eigen::Vector3d(so3mat(2, 1), so3mat(0, 2), so3mat(1, 0));
}

std::tuple<Eigen::Vector3d, double> AxisAng3(const Eigen::Vector3d &expc3)
{
    // Angle is simply the norm of the 3-vector exponential coordinates of
    // rotation
    const double theta = expc3.norm();

    // Axis is simply the 3-vector exponential coordinates of rotation normalized
    // by the angle
    const Eigen::Vector3d omghat = expc3 / theta;
    return std::make_tuple(omghat, theta);
}

Eigen::Matrix4d FKinSpace(const Eigen::Matrix4d &M, const Eigen::MatrixXd &Slist, const Eigen::VectorXd &thetalist)
{
    // Compute space-form forward kinematics using the product of exponential
    // formula
    Eigen::Matrix4d T = M;
    for (int i = thetalist.size() - 1; i >= 0; --i)
    {
        Eigen::Matrix4d expMat = MatrixExp6(VecTose3(Slist.col(i) * thetalist(i)));
        T = expMat * T;
    }
    return T;
}

Eigen::Matrix4d VecTose3(const Eigen::VectorXd &V)
{
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

Eigen::Matrix3d VecToso3(const Eigen::Vector3d &omg)
{
    // Fill out the 3x3 so3 matrix as the skew-symmetric form of the passed
    // 3-vector, omg
    const Eigen::Matrix3d so3mat =
        (Eigen::Matrix3d() << 0, -omg(2), omg(1), omg(2), 0, -omg(0), -omg(1), omg(0), 0).finished();
    return so3mat;
}

Eigen::Matrix4d TransInv(const Eigen::Matrix4d &T)
{

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

Eigen::VectorXd se3ToVec(const Eigen::Matrix4d &se3mat)
{
    // Extract and construct the vector from the skew-symmetric matrix
    const Eigen::VectorXd V =
        (Eigen::VectorXd(6) << se3mat(2, 1), se3mat(0, 2), se3mat(1, 0), se3mat.block<3, 1>(0, 3)).finished();

    return V;
}

Eigen::Matrix3d MatrixLog3(const Eigen::Matrix3d &R)
{

    const double acosinput = (R.trace() - 1) / 2;
    Eigen::Matrix3d so3mat;

    if (acosinput >= 1)
    {
        so3mat.setZero();
    }
    else if (acosinput <= -1)
    {
        Eigen::Vector3d omg;
        if (!NearZero(1 + R(2, 2)))
        {
            omg = (1 / sqrt(2 * (1 + R(2, 2)))) * Eigen::Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
        }
        else if (!NearZero(1 + R(1, 1)))
        {
            omg = (1 / sqrt(2 * (1 + R(1, 1)))) * Eigen::Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
        }
        else
        {
            omg = (1 / sqrt(2 * (1 + R(0, 0)))) * Eigen::Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
        }
        so3mat = VecToso3(M_PI * omg);
    }
    else
    {
        const double theta = acos(acosinput);
        so3mat = theta * (1 / (2 * sin(theta))) * (R - R.transpose());
    }

    return so3mat;
}

Eigen::Matrix4d MatrixLog6(const Eigen::Matrix4d &T)
{
    const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    const Eigen::Vector3d p = T.block<3, 1>(0, 3);
    const Eigen::Matrix3d omgmat = MatrixLog3(R);

    Eigen::Matrix<double, 4, 4> expmat;

    if (omgmat.isApprox(Eigen::Matrix3d::Zero()))
    {
        expmat.block<3, 3>(0, 0) = Eigen::Matrix3d::Zero();
        expmat.block<3, 1>(0, 3) = p;
        expmat.block<1, 4>(3, 0) = Eigen::Matrix<double, 1, 4>::Zero();
    }
    else
    {
        const double theta = acos((R.trace() - 1) / 2);
        const Eigen::Matrix3d eye3 = Eigen::Matrix3d::Identity();
        expmat.block<3, 3>(0, 0) = omgmat;
        expmat.block<3, 1>(0, 3) =
            (eye3 - 0.5 * omgmat + (1.0 / theta - 1.0 / (2 * tan(0.5 * theta))) * omgmat * omgmat / theta) * p;
        expmat.block<1, 4>(3, 0) = Eigen::Matrix<double, 1, 4>::Zero();
    }

    return expmat;
}

Eigen::Matrix<double, 6, 1> get_screw(affordance_util::ScrewInfo &si)
{

    Eigen::VectorXd screw(6); // Output of the function

    // If Screw is specified and axis is not, compute and fill out axis from Screw
    if (si.axis.isZero() && !si.screw.isZero())
    {

        si.axis = si.screw.head(3);
    }

    // If Screw is not specified, compute it
    if (si.screw.isZero())
    {
        if (si.type == "translation")
        {
            si.screw << Eigen::Vector3d::Zero(), si.axis;
        }
        else if (si.type == "rotation")
        {
            si.screw.head(3) = si.axis;
            si.screw.tail(3) = si.location.cross(si.axis);
        }
        else if (si.type == "screw")
        {
            si.screw.head(3) = si.axis;
            si.screw.tail(3) = si.location.cross(si.axis) + si.pitch * si.axis;
        }
        else
        {
            throw std::runtime_error("Invalid screw type specified when getting screw");
        }
    }

    return si.screw;
}
Eigen::Matrix<double, 6, 1> get_screw(const Eigen::Vector3d &w, const Eigen::Vector3d &q)
{

    Eigen::VectorXd screw(6); // Output of the function

    screw.head(3) = w;
    screw.tail(3) = -w.cross(q); // q cross w

    return screw;
}

bool NearZero(const double &near)
{
    const double nearZeroTol_ = 1e-6;
    return std::abs(near) < nearZeroTol_;
}
} // namespace affordance_util
