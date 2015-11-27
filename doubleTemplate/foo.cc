# include <typeinfo>
# include <iostream>

template <typename A>
class Foo
{
public:
  template <typename B> 
    Foo (B b);

  void print()
  {
    std::cout << typeid(myA_).name() << std::endl;
    std::cout << myA_ << std::endl;
  }
  virtual ~Foo (){};

private:
  A myA_;
};

template <typename A>
  template <typename B>
Foo<A>::Foo(B b)
{
  myA_ = static_cast<A>(b);
  myA_++;
}

int main(int argc, char *argv[])
{
  int b = 2;
  Foo<double> myFood(b);
  Foo<float> myFoof(b);
  myFood.print();
  myFoof.print();
  return 0;
}
