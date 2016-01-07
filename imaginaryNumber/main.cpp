#include <cmath>
#include <iostream>
#include <complex>



double e3(double d1, double d2, double d3, double b1, double b2, double b3) const
{
  std::cout << "Computing e3" << std::endl;
  std::cout << "d1 = " << d1 << std::endl;
  std::cout << "d2 = " << d2 << std::endl;
  std::cout << "d3 = " << d3 << std::endl;
  std::cout << "b1 = " << b1 << std::endl;
  std::cout << "b2 = " << b2 << std::endl;
  std::cout << "b3 = " << b3 << std::endl;

  Matrix3d D, Dpbb;
  Vector3d b;
  D << d1, 0, 0, 0, d2, 0, 0, d3;
  b << b1, b2, b3;
  Dpbb = D + b*b.transpose();
  std::cout << "Dpbb = \n" << Dpbb << std::endl;

  EigenSolver<Matrix3d> es(A);
  cout << "The eigenvalues of Dpbb are:" << endl << es.eigenvalues() << endl;
  cout << "The matrix of eigenvectors, V, is:" << endl << es.eigenvectors() << endl << endl;
  Eigen::Vector3cd eig = es.eigenvalues();
  std::cout << "eig = \n" << eig << std::endl;

  std::cout << "e3 = " << e3 << std::endl;
  double res = e3.real() - pow(b1,2) - pow(b2,2) - pow(b3,2);
  return res;
}

int main(int argc, char *argv[])
{
  double d = 4 + sqrt(-1.0);
  std::cout << d << std::endl;
  std::complex<double> d2 = (4 + 1i);
  std::cout << d2.real()<< std::endl;
  return 0;
}
