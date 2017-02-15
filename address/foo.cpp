#include <iostream>
class Bar
{
public:
  Bar (double& d1, double& d2)
    :d1_(d1),
    d2_(d2)
  {
    std::cout << "d1:\n" << &d1 << std::endl; 
    std::cout << "d1_:\n" << &d1_ << std::endl; 
    std::cout << "d2:\n" << &d2 << std::endl; 
    std::cout << "d2_:\n" << &d2_ << std::endl; 
  };

private:
  double& d1_;
  double& d2_;
};


int main()
{
  double a1 = 0.4;
  double a2 = 0.3;
  Bar b(a1, a2);
  
  return 0;
}
