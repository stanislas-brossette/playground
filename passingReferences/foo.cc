#include <iostream> 

class Bar
{
public:
  Bar (double& a)
    : a_(a)
  {
  };
  void print()
  {
    std::cout << a_ << std::endl;
  }
  virtual ~Bar (){};

private:
  double& a_;
};

class Foo
{
public:
  Foo (double a)
    : a_(a),
    b_(a_)
  {
    a_ = 1234;
  };

  virtual ~Foo (){};
  Bar& b(){return b_;};

private:
  double a_;
  Bar b_; 
};

int main(int argc, char *argv[])
{
  Foo f(14);
  f.b().print();
 
  return 0;
}
