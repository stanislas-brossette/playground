# include <iostream>
# include <vector>
# include <memory>

class Base
{
public:
  Base (){};
  virtual ~Base(){};
};

class Derived : public Base
{
public:
  Derived (){};
};

void foo(std::shared_ptr<Base>)
{
  std::cout << "BASE" << std::endl;
};

void foo(std::shared_ptr<Derived>)
{
  std::cout << "DERIVED" << std::endl;
};

int main(int argc, char *argv[])
{
  std::vector<std::shared_ptr<Base>> myVec;
  myVec.push_back(std::make_shared<Base>());
  myVec.push_back(std::static_pointer_cast<Base>(std::make_shared<Derived>()));
  for(auto s: myVec)
  {
    if(std::dynamic_pointer_cast<Derived>(s) != nullptr)
      foo(std::dynamic_pointer_cast<Derived>(s));
    else
      foo(s);
  }
  return 0;
}
