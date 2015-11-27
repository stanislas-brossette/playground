#include <iostream>
#include <vector>
#include <memory>

typedef double T;

class A
{
public:
  A ()
  {
    a = std::make_shared<std::vector<T>>(std::initializer_list<T>{1, 2, 3});
  }

  std::shared_ptr<std::vector<T>> a;
};

int main(int argc, char *argv[])
{
  std::vector<T> vec({4, 6, 7});
  A myA;
  return 0;
}
