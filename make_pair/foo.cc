#include <iostream>
#include <memory>
#include <vector>

class foo
{
public:
  foo (){};
  void add(int a, double& b)
  {
    vec.push_back(std::make_pair<int, double&>(std::move(a),std::move(b)));
  }

private:
  std::vector<std::pair<int, double&>> vec;
};

int main(int argc, char *argv[])
{
  foo Foo;
  double d = 4.0;
  Foo.add(2, d);
  return 0;
}
