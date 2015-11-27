#include <memory>
#include <iostream>

template<typename T>
class A
{
public:
  A (T* ptr)
    : 
      ptr_(ptr)
  {
  }
  void print()
  {
    std::cout << *ptr_ << std::endl;
  }


private:
  std::shared_ptr<T> ptr_;
};

int main(int argc, char *argv[])
{
  std::unique_ptr<double> a(new double(6));
  std::cout << *a << std::endl;
  {
    A<double> myA(a.release());
    myA.print();
    std::cout << *a << std::endl;
  }

  return 0;
}
