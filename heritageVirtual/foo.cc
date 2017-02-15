#include <iostream>

class A
{
public:
  A (){};
  virtual ~A (){};
  virtual int foo(int i) const;
  int foo() const{ std::cout << "A::foo()" << std::endl; return 4; };

private:
  /* data */
};

class B: public A
{
public:
  B ():A(){};
  virtual ~B (){};
  int foo(int i) const{ std::cout << "B::foo(i)" << std::endl; return i;};

private:
  /* data */
};

int main(void)
{
  B b;
  b.foo();
  return 0;
}
