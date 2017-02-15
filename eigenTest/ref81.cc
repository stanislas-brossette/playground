#include <iostream>
#include <Eigen/Core>

void compute(Eigen::Ref<Eigen::Matrix<double, 8, 1> > out)
{
  std::cout << out << std::endl;
}

int main(void)
{
  Eigen::Matrix<double,12,4> res;
  res.setZero();
  std::cout << "res: " << res << std::endl;
  compute(res.block(0,0,8,1));

  return 0;
}
