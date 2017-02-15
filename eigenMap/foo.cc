#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

class ReverseQuaternion : public Eigen::Quaterniond {
  double *refData_;

public:
  ReverseQuaternion(double *data)
      : Eigen::Quaterniond(data[0], data[1], data[2], data[3]), refData_(data) 
  {
  }
  void print() const 
  {
    std::cout << "w: " << w() << std::endl;
    std::cout << "x: " << x() << std::endl;
    std::cout << "y: " << y() << std::endl;
    std::cout << "z: " << z() << std::endl;
  }
  void writeChanges() 
  {
    refData_[0] = w();
    refData_[1] = x();
    refData_[2] = y();
    refData_[3] = z();
  }
  ~ReverseQuaternion() { writeChanges(); }

  ReverseQuaternion &operator=(const Eigen::Quaterniond &quat) 
  {
    w() = quat.w();
    vec() = quat.vec();
    return *this;
  }
};

class ConstReverseQuaternion : public Eigen::Quaterniond 
{
public:
  ConstReverseQuaternion(const double *data)
      : Eigen::Quaterniond(data[0], data[1], data[2], data[3]) {}
  void print() const {
    std::cout << "w: " << w() << std::endl;
    std::cout << "x: " << x() << std::endl;
    std::cout << "y: " << y() << std::endl;
    std::cout << "z: " << z() << std::endl;
  }
  ~ConstReverseQuaternion() {}
};

int main() {
  // Eigen::MatrixXd lineMat(1,4);
  // lineMat.setZero();
  // std::cout << "lineMat:\n" << lineMat << std::endl;
  // Eigen::Map<Eigen::VectorXd> vecMap(lineMat.data(),3);
  // std::cout << "vecMap:\n" << vecMap << std::endl;

  Eigen::Matrix<double, 4, 1> init;
  init << 1, 0, 0, 0;
  Eigen::Matrix<double, 4, 1> input;
  Eigen::Map<Eigen::Quaterniond> vQ(input.data());
  vQ.w() = init(0);
  vQ.vec() = init.segment(1,3);
  //Eigen::Map<const Eigen::Quaterniond> vQconst(input.data());
  const Eigen::Quaterniond vQconst(input.data());
  std::cout << "vQ.w():\n" << vQ.w() << std::endl;
  std::cout << "vQ.vec():\n" << vQ.vec() << std::endl;
  std::cout << "vQConst.x():\n" << vQconst.vec() << std::endl; 
  return 0;
}
