#include <iostream>
#include <memory>

class B;

class A_Base
{
  friend A;
  friend B;
protected:
  A_Base (){};
public:
  virtual ~A_Base (){};
};

class B_Base : A_Base
{
  friend B;
protected:
  A_Base (){};
public:
  virtual ~A_Base (){};
};

class A
{
protected:
  A ( std::shared_ptr<A_Base> a) : aBase_(std::make_shared<const Manifold_Base>(a)){};
  static A makeA(std::shared_ptr<A_Base> a)
  {
    return A(a);
  }
public:
  virtual ~A (){};

private:
  std::shared_ptr<const A_Base> aBase_;
};

int main()
{
  Manifold_Base a;
  B b(a);
  b.test();
  return 0;
}
