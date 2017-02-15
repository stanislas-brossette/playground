#include <iostream>
#include <stdexcept>
#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <yaml-cpp/yaml.h>

using namespace Eigen;
using namespace std;

void testFunction(Eigen::VectorXd& x)
{
  std::cout << "Fine" << std::endl;
  std::cout << "x = \n" << x.transpose() << std::endl;
}


typedef Ref<VectorXd> RefVec; 

RefVec foo()
{
  VectorXd testVec = VectorXd::LinSpaced(5,1,5);
  std::cout << "testVec = \n" << testVec << std::endl;
  return testVec;
}

int main()
{
  //srand((unsigned)time(NULL));
  std::cout << "Using: Eigen" << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION <<"." << EIGEN_MINOR_VERSION<< std::endl;
  {
    YAML::Node config = YAML::LoadFile("../../share/config.yaml");
    
    int username = config["username"].as<int>();
    std::string password = config["password"].as<std::string>();
    bool boolTest = config["boolTest"].as<bool>();
    double dblTest = config["dblTest"].as<double>();

    std::cout << "username = " << username << std::endl;
    std::cout << "password = " << password << std::endl;
    std::cout << "boolTest = " << boolTest << std::endl;
    std::cout << "dblTest = " << dblTest << std::endl;

    YAML::Node obstacles = config["obstacles"];
    std::cout << "obstacles.Type(): " << obstacles.Type() << std::endl;
    std::cout << "obstacles.size(): " << obstacles.size() << std::endl;
    std::cout << "obstacles[0][size]: " << obstacles[0]["size"] << std::endl;
    //for (YAML::const_iterator it = obstacles.begin(); it != obstacles.end(); ++it) {
      //std::cout << it->first.as<int>() << "\n";
    //}

  }
  {
    //VectorXd v;
    //v = VectorXd::LinSpaced(100,1,100);
    //std::cout << "v = \n" << v.transpose() << std::endl;
    //MatrixXd M;
    //M = Map<MatrixXd, 0, OuterStride<> >(v.data(), 10, 10, OuterStride<>(10));
    //std::cout << "M = \n" << M << std::endl;
    //Map<MatrixXd, 0, OuterStride<> > Mrest = Map<MatrixXd, 0, OuterStride<> >(M.data(), 10, 10, OuterStride<>(10));
    //std::cout << "Mrest = \n" << Mrest << std::endl;
    //new (&Mrest) Map<MatrixXd, 0, OuterStride<> >(M.data(), 8, 10, OuterStride<>(10));
    //std::cout << "Mrest = \n" << Mrest << std::endl;
    //new (&Mrest) Map<MatrixXd, 0, OuterStride<> >(M.data(), 4, 10, OuterStride<>(10));
    //std::cout << "Mrest = \n" << Mrest << std::endl;

    //std::cout << "M.topRows(3) = \n" << M.topRows(3) << std::endl;
    //std::cout << "M.topRows(3).outerStride() = \n" << M.topRows(3).outerStride() << std::endl;
    
  }
  {
    
    //Eigen::Matrix2d Q;
    //Eigen::Vector2d d;
    //Q << 1,2,4,5;
    //d << 1,2;
    //Eigen::Matrix2d M;
    //M << 7,43,8,2;
    //d(0) = std::max(d(0), 0.0);
    //d(1) = std::max(d(1), 0.);
    //Eigen::Matrix2d DQt = d.asDiagonal()*Q.transpose();
    //Eigen::Matrix2d QDQt(Q*DQt);
    //Eigen::Matrix2d QDQt2 = Q*DQt;
    //QDQt.noalias() = Q*DQt;
  }
  {
    //Eigen::VectorXd x(3);
    //x << 1.3, 4, 5;
    //testFunction(x);
    //testFunction(-x);
  }
  {
    //Eigen::Matrix2d mat;
    //mat << 1, 2,
    //       3, -4;
    //mat = mat.array().abs();
    //cout << "Here is mat.sum():       " << mat.sum()       << endl;
    //cout << "Here is mat.prod():      " << mat.prod()      << endl;
    //cout << "Here is mat.mean():      " << mat.mean()      << endl;
    //cout << "Here is mat.minCoeff():  " << mat.minCoeff()  << endl;
    //cout << "Here is mat.maxCoeff():  " << mat.maxCoeff()  << endl;
    //cout << "Here is mat.trace():     " << mat.trace()     << endl;
  }
  {
    //Eigen::Vector4d v0;
    //v0 << 1,0,0,0;
    //Eigen::Map<Eigen::Quaterniond> vQ(v0.data());
    //std::cout << "v0 = \n" << v0 << std::endl;
    //std::cout << "vQ.matrix() = \n" << vQ.matrix() << std::endl;
    //v0 << 0,0,0,1;
    //std::cout << "v0 = \n" << v0 << std::endl;
    //std::cout << "vQ.matrix() = \n" << vQ.matrix() << std::endl;
    //Eigen::Quaterniond q(1, 0, 0, 0);
    //std::cout << "q.matrix() = \n" << q.matrix() << std::endl;
    //q.setIdentity();
    //std::cout << "q.vec() = \n" << q.vec() << std::endl;
    //std::cout << "q.w() = \n" << q.w() << std::endl;
    //std::cout << "q.matrix() = \n" << q.matrix() << std::endl;
  }
  return 0;
}

