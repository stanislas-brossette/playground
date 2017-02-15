#include <iostream>
#include <Eigen/Core>

class A
{
public:
  A (){};
  virtual ~A (){};
  void compute(const Eigen::Ref<const Eigen::Vector3d> v) const
  {
    std::cout << v << std::endl;
  };
private:
};

class B : A
{
public:
  B(const Eigen::Vector3d &v) : v_(v){};
  virtual ~B(){};
  void compute() const
  {
    A::compute(v_);
  };

private:
  Eigen::Vector3d v_;
};

int main(void)
{
  Eigen::Vector3d v(1,2,3);
  B b(v);
  b.compute();

  return 0;
}
