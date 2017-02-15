#include <iostream>
#include <vector>
#include <Eigen/Core>



void rotate(Eigen::MatrixXi& m)
{
  if(m.rows() != m.cols())
    std::cerr << "Must use square matrix" << std::endl;

  int n = m.rows();
  for (int layer = 0; layer < n/2; layer++)
  {
    std::cout << "layer: " << layer << std::endl;
    std::cout << "n: " << n << std::endl;
    for (int i = layer; i < n-1-layer; i++)
    {
      std::cout << "i: " << i << std::endl;
      int top = m(layer,i);
      // top << left
      m(layer,i) = m(n-layer-i,layer);
      // left << bottom
      m(n-layer-i,layer) = m(n-1-layer,n-layer-i);
      //bottom << right
      m(n-1-layer,n-layer-i) = m(i,n-1-layer);
      //right << top
      m(i,n-1-layer) = top;
    }
  }
}

int main(void)
{
  int n = 4;
  Eigen::MatrixXi m = Eigen::MatrixXi::Random(n,n);
  m << 1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4;
  std::cout << m << std::endl;
  rotate(m);
  std::cout << std::endl << m << std::endl;
  rotate(m);
  std::cout << std::endl << m << std::endl;
  rotate(m);
  std::cout << std::endl << m << std::endl;
  rotate(m);
  std::cout << std::endl << m << std::endl;
  return 0;
}
