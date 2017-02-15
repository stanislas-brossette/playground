# include <iostream>
#include <vector>
# include <Eigen/Core>

//int main(int argc, char *argv[])
//{
  //Eigen::MatrixXd M(0,0);
  //std::cout << "M:\n" << M << std::endl;
  //Eigen::Vector3d v;
  //v << 10, 20, 30;
  //std::cout << "v = " << v << std::endl;
  //std::vector<Eigen::MatrixXd> vec(4);

  //for (size_t i = 0; i < vec.size(); ++i)
  //{
    //std::cout << "vec[i]:\n" << vec[i] << std::endl;
  //}
  //return 0;
//}

int main(int argc, char *argv[])
{
  Eigen::MatrixXd M(0,0);
  std::cout << "M:\n" << M << std::endl;
  Eigen::Vector3d v;
  v << 10, 20, 30;
  std::cout << "v = " << v << std::endl;
  std::vector<Eigen::MatrixXd> vec(4);

  for (size_t i = 0; i < vec.size(); ++i)
  {
    std::cout << "vec[i]:\n" << vec[i] << std::endl;
  }
  return 0;
}
