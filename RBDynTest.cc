#include <iostream>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>


#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/Jacobian.h>

#include <manifolds/Manifold.h>
#include <manifolds/CartesianProduct.h>
#include <manifolds/SO3.h>
#include <manifolds/ExpMapMatrix.h>
#include <manifolds/RealSpace.h>

Eigen::Vector3d getPoint(Eigen::Matrix3d rot, Eigen::Vector3d pos, Eigen::Vector3d trans)
{
  return rot * (trans) + pos;
}

Eigen::Vector3d getPoint(Eigen::Quaterniond rot, Eigen::Vector3d pos, Eigen::Vector3d trans)
{
  return rot * (trans) + pos;
}

Eigen::Vector4d hamiltonProduct(Eigen::Vector4d& q1, Eigen::Vector4d& q2)
{
  Eigen::Vector4d ans;

  Eigen::Vector3d v1;
  Eigen::Vector3d v2;

  v1 << q1.bottomRows(3);
  v2 << q2.bottomRows(3);

  ans << q1(0) * q2(0) - v1.dot(v2),
    q1(0) * v2 + q2(0) * v1 + v1.cross(v2);

  return ans;
}

Eigen::Vector4d inverse(Eigen::Vector4d q)
{
  double norm = q.norm();
  Eigen::Vector4d ans;
  ans << q(0) / norm, -q(1) / norm, -q(2) / norm, -q(3) / norm;
  return ans;
}

Eigen::Vector3d myGetPoint(Eigen::Vector4d rot, Eigen::Vector3d pos, Eigen::Vector3d trans)
{
  Eigen::Vector4d quat;
  quat << rot;
  Eigen::Vector4d invQuat = inverse(quat);

  Eigen::Vector4d fakeQuat;

  fakeQuat << 0, trans;

  Eigen::Vector4d firstStep = hamiltonProduct(fakeQuat, invQuat);

  return hamiltonProduct(quat, firstStep).tail<3>() + pos;
}

Eigen::Matrix3d hat(Eigen::Vector3d& r)
{
  Eigen::Matrix3d hat;
  hat <<
    0, -r.z(), r.y(),
    r.z(), 0, -r.x(),
    -r.y(), r.x(), 0;

  return hat;
}

Eigen::MatrixXd pseudoInverse(Eigen::MatrixXd& mat)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType s = svd.singularValues();

  double tolerance = 1e-8;
  for (int i = 0; i < svd.nonzeroSingularValues(); ++i)
    {
      if (s(i) > tolerance)
	{
	  s(i) = 1.0/s(i);
	}
    }

  //std::cout << "svd.matrixV(): " << std::endl << svd.matrixV() << std::endl;
  //std::cout << "svd.matrixU(): " << std::endl << svd.matrixU() << std::endl;

  Eigen::MatrixXd sMat(mat.rows(), mat.cols());
  sMat.setZero();
  int dim = svd.singularValues().asDiagonal().cols();
  sMat.topLeftCorner(dim, dim) = s.asDiagonal();

  return svd.matrixV() * sMat.transpose() * svd.matrixU().transpose();
}

int main()
{
  rbd::MultiBodyGraph mbg;
  sva::Matrix3d I = sva::Matrix3d::Identity();
  sva::Matrix3d I_o = sva::Matrix3d::Identity();
  sva::Vector3d h = sva::Vector3d::Zero();
  sva::RBInertiad rbi(1., h, I);
  rbi = sva::RBInertiad(0, sva::Vector3d::Zero(), sva::Matrix3d::Identity());

  rbd::Body b(rbi, 0, "Hello");
  mbg.addBody(b);

  rbd::Body b1(rbi, 1, "Hello_1");
  mbg.addBody(b1);

  Eigen::Vector3d axis;
  axis << 0, 0, 1;
  axis.normalize();

  rbd::Joint::Type jointType = rbd::Joint::Type::Fixed;

  rbd::Joint j(jointType, axis, false, 0, "Joint");
  mbg.addJoint(j);

  Eigen::Vector3d translation;
  translation << -1, 5, 2;

  sva::PTransformd toBody2(translation);

  mbg.linkBodies(0, toBody2, 1, sva::PTransformd::Identity(), 0);

  Eigen::Matrix3d baseRot = sva::RotX(M_PI/18) * sva::RotZ(15*M_PI/4);
  sva::Quaterniond baseRotAsQuat(baseRot);
  Eigen::Vector3d basePos;
  basePos << 0.5, -2, 3.2;

  rbd::MultiBody mb = mbg.makeMultiBody(0, false);
  rbd::MultiBodyConfig mbc(mb);
  mbc.zero(mb);

  mbc.q[0][0] = baseRotAsQuat.w();
  mbc.q[0][1] = baseRotAsQuat.x();
  mbc.q[0][2] = baseRotAsQuat.y();
  mbc.q[0][3] = baseRotAsQuat.z();
  for(int i = 0; i < 3; ++i)
    {
      mbc.q[0][4 + i] = basePos(i);
    }

  rbd::sForwardVelocity(mb, mbc);
  rbd::sForwardKinematics(mb, mbc);

  // Jacobian "translations"
  //
  // RBDyn allows us to compute the jacobian of a specific body
  // in the robot tree we defined.
  //
  // The input of the jacobian (its columns) :
  // - instant rotation vector of the free flyer
  // - linear speed vector of the free flyer
  // - any user defined joint's speed
  //
  // The output of the jacobian (its rows) :
  // - instant rotation vector of the specified body
  // - linear speed vector of the specified body
  //
  //
  // From a manifold point of view, it means that RBDyn computations
  // are done in the tangent space of a SO(3) x R3 manifold.
  // Since we want this to be transparent to our manifold-wrapping
  // layer, we will have to apply an "inverse map" so as to expose
  // to our manifold a jacobian computed in the representation space.
  //

  // RBDyn jacobian for the first body
  Eigen::MatrixXd rbdynJac_B1 = Eigen::MatrixXd::Zero(6, mb.nrDof());
  {
    rbd::Jacobian jac(mb, mb.body(0).id());

    const Eigen::MatrixXd& partJac = jac.jacobian(mb, mbc);

    jac.fullJacobian(mb, partJac, rbdynJac_B1);

    std::cout << "rbdynJac_B1: " << std::endl << rbdynJac_B1 << std::endl;
  }
  // RBDyn jacobian for the second body
  Eigen::MatrixXd rbdynJac_B2 = Eigen::MatrixXd::Zero(6, mb.nrDof());
  {
    rbd::Jacobian jac(mb, mb.body(1).id());

    const Eigen::MatrixXd& partJac = jac.jacobian(mb, mbc);

    jac.fullJacobian(mb, partJac, rbdynJac_B2);

    std::cout << "rbdynJac_B2: " << std::endl << rbdynJac_B2 << std::endl;
  }

  // Finite differences representing rotations as matrices
  // Also computes the jacobian part representing the position
  Eigen::MatrixXd maniJac(3, 6);
  double deltaStep = 1e-2;
  mnf::SO3<mnf::ExpMapMatrix> so3;
  mnf::RealSpace r3(3);
  mnf::CartesianProduct manifold(so3, r3);
  Eigen::VectorXd value(9 + 3);
  {
    for(int i = 0; i < 9; ++i)
      {
	value(i) = baseRot(i);
      }
    for(int i = 0; i < 3; ++i)
      {
	value(i + 9) = basePos(i);
      }
  }
  {
    Eigen::MatrixXd jac(3, 9 + 3);
    jac.setZero();

    for(int i = 0; i < 9; ++i)
      {
	Eigen::Matrix3d delta;
	delta.setZero();
	delta(i) = deltaStep;

	jac.col(i) = (getPoint(baseRot + delta, basePos, translation) -
		      getPoint(baseRot - delta, basePos, translation)) / (2*deltaStep);
      }

    for(int i = 0; i < 3; ++i)
      {
	Eigen::Vector3d delta;
	delta.setZero();
	delta(i) = deltaStep;

	jac.col(9 + i) = (getPoint(baseRot, basePos + delta, translation) -
			  getPoint(baseRot, basePos - delta, translation)) / (2*deltaStep);
      }

    //std::cout << "jac (finite diff): " << std::endl << jac << std::endl;

    manifold.applyDiffRetractation(maniJac, jac, value);

    std::cout << "maniJac: " << std::endl << maniJac << std::endl;
  }

  // Finite differences representing quaternions as matrices
  Eigen::MatrixXd quatJac(3, 4);
  deltaStep = 1e-4;
  {
    Eigen::Vector4d quat;
    quat <<
      baseRotAsQuat.w(),
      baseRotAsQuat.x(),
      baseRotAsQuat.y(),
      baseRotAsQuat.z();

    for(int i = 0; i < 4; ++i)
      {
	Eigen::Vector4d delta;
	delta.setZero();

	delta(i) = deltaStep;

	quatJac.col(i) = (myGetPoint(quat + delta, basePos, translation) -
			  myGetPoint(quat - delta, basePos, translation)) / (2*deltaStep);
      }

    //std::cout << "quatJac: " << std::endl << quatJac << std::endl;
  }

  Eigen::Vector3d r = mbc.bodyPosW[1].translation();
  Eigen::Matrix3d Et = mbc.bodyPosW[1].rotation().transpose();

  Eigen::MatrixXd adjoint(6, 6);
  adjoint <<
    Et, Eigen::Matrix3d::Zero(),
    hat(r) * Et, Et;

  adjoint <<
    Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(),
    Eigen::Matrix3d::Zero(), Et.transpose();

  {
    // Featherstone transformation matrix
    Eigen::MatrixXd instToQuat(4, 3);
    Eigen::Quaterniond p = baseRotAsQuat;
    instToQuat <<
      -p.x(), -p.y(), -p.z(),
      p.w() , -p.z(),  p.y(),
      p.z() , p.w() , -p.x(),
      -p.y(), p.x() ,  p.w();
    instToQuat /= 2;

    Eigen::MatrixXd isItRight(3, 3);
    isItRight << quatJac * instToQuat;

    Eigen::MatrixXd finalResult(3, 6);

    finalResult << rbdynJac_B2.bottomRows<3>() * adjoint;

    std::cout << "adjoint: " << std::endl << adjoint << std::endl;
    std::cout << "finalResult: " << std::endl << finalResult << std::endl;
  }

  // Recomputing the jacobian of a point in a body
  // without using a second, fixed to the first body
  {
    Eigen::MatrixXd body2Jac(6, 6);
    body2Jac << rbdynJac_B1 * adjoint;

    for(int i = 0; i < body2Jac.cols(); ++i)
      {
	Eigen::Vector3d w;
	Eigen::Vector3d v;
	w << body2Jac.col(i).head<3>();
	v << body2Jac.col(i).tail<3>();

	body2Jac.col(i).tail<3>() << v + w.cross(mbc.bodyPosW[0].rotation().transpose() * translation);
      }

    // std::cout << "body2Jac: " << std::endl << body2Jac << std::endl;
    // std::cout << "rbdynJac_B2: " << std::endl << rbdynJac_B2 * adjoint << std::endl;
  }

  // Inversing the mapping, going from the tangent
  // space to the representation space
  {
    Eigen::MatrixXd diffMap = manifold.diffRetractation(value);
    //std::cout << "diffMap: " << std::endl << diffMap << std::endl;

    Eigen::MatrixXd invDiffMap = pseudoInverse(diffMap);
    //std::cout << "invDiffMap: " << std::endl << invDiffMap << std::endl;

    Eigen::MatrixXd reprJac(6, 12);
    reprJac << rbdynJac_B2 * adjoint * invDiffMap;

    Eigen::MatrixXd resultJac(6, 6);
    manifold.applyDiffRetractation(resultJac, reprJac, value);

    // std::cout << "resultJac: " << std::endl << resultJac << std::endl;


    reprJac << rbdynJac_B2 * invDiffMap;
    manifold.applyDiffRetractation(resultJac, reprJac, value);
    resultJac *= adjoint;

    // std::cout << "resultJac: " << std::endl << resultJac << std::endl;
  }


}
