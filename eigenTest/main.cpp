# include <iostream>
# include <Eigen/Core>

int main(int argc, char *argv[])
{
  Eigen::Vector3d v;
  v << 10, 20, 30;
  std::cout << "v = " << v << std::endl;
  return 0;
}
