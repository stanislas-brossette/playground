#include <memory>
#include <iostream>

class Foo
{
public:
  Foo ()
    : bar_ (new int (3)),
      duh_ (42)
  {}
  virtual ~Foo (){}

  const int* getBar () const
  {
    *bar_ = 4; // /!\ VALID!
    return bar_.get();
  }

  const int& getDuh () const
  {
    //duh_ = 4; // ---> not valid
    return duh_;
  }

private:
  std::shared_ptr<int> bar_;
  int duh_;
};

int main(int argc, char *argv[])
{
  Foo foo;
  std::cout << *(foo.getBar()) << std::endl;
  std::cout << foo.getDuh() << std::endl;
  return 0;
}
