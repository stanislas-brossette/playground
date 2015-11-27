#include <iostream>

class A
{
public:
  A (double a)
    : a_(a)
  {}

  std::ostream& print (std::ostream& o) const
  {
    return o << a_;
  };

  std::ostream& print (std::ostream& o, const double& b) const
  {
    return o << a_;
  };

  A with(double

  virtual ~A (){}

private:
  double a_;
};

std::ostream& operator<< (std::ostream& o, const A& a, const double& b)
{
  return a.print(o, b);
};

std::ostream& operator<< (std::ostream& o, const A& a)
{
  return a.print(o);
};

int main(int argc, char *argv[])
{
  A myA(4);
  std::cout << myA.with(6) << std::endl;
  std::cout << myA << std::endl;
  return 0;
}
