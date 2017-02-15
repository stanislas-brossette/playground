#include <iostream>

class A
{
public:
  A () {};
  virtual ~A (){};
  std::ostream& print(std::ostream& o) const
  {
    o << "Hello";
    return o;
  }
};

std::ostream& operator<<(std::ostream& os, const A& m)
{
  return m.print(os);
}

int main(void)
{
  A a0;
  std::cout << a0.print(std::cout) << std::endl;
  std::cout << a0 << std::endl;
  return 0;
}

