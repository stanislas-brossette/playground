#include <sstream>
#include <iostream>

class B
{
public:
  B () {};
  virtual ~B() {};
  virtual bool isSameType(const B& other)
  {
    if(dynamic_cast<const B*>(&other))
    {
      std::cout << "Yup its a Base" << std::endl;
      return true;
    }
    else
    {
      std::cout << "Nope its not Base" << std::endl;
      return false;
    }
  }
};

class D1: public B
{
public:
  D1 () {};
  virtual ~D1 () {};
  virtual bool isSameType(const B& other)
  {
    if(dynamic_cast<const D1*>(&other))
    {
      std::cout << "Yup its a D1" << std::endl;
      return true;
    }
    else
    {
      std::cout << "Nope its not D1" << std::endl;
      return false;
    }
  }
};

class D2: public B 
{
public:
  D2 () {};
  virtual ~D2 () {};
  virtual bool isSameType(const B& other)
  {
    if(dynamic_cast<const D2*>(&other))
    {
      std::cout << "Yup its a D2" << std::endl;
      return true;
    }
    else
    {
      std::cout << "Nope its not D2" << std::endl;
      return false;
    }
  }
};

int main()
{
  B b, bb;
  D1 d1, d1b;
  D2 d2, d2b;

  std::cout << "b.isSameType(b);" << std::endl;
  b.isSameType(b);
  std::cout << "b.isSameType(bb);" << std::endl;
  b.isSameType(bb);
  std::cout << "b.isSameType(d1);" << std::endl;
  b.isSameType(d1);
  std::cout << "b.isSameType(d1b);" << std::endl;
  b.isSameType(d1b);
  std::cout << "b.isSameType(d2);" << std::endl;
  b.isSameType(d2);
  std::cout << "b.isSameType(d2b);" << std::endl;
  b.isSameType(d2b);

  std::cout << "d1.isSameType(b);" << std::endl;
  d1.isSameType(b);
  std::cout << "d1.isSameType(bb);" << std::endl;
  d1.isSameType(bb);
  std::cout << "d1.isSameType(d1);" << std::endl;
  d1.isSameType(d1);
  std::cout << "d1.isSameType(d1b);" << std::endl;
  d1.isSameType(d1b);
  std::cout << "d1.isSameType(d2);" << std::endl;
  d1.isSameType(d2);
  std::cout << "d1.isSameType(d2b);" << std::endl;
  d1.isSameType(d2b);

  std::cout << "d2.isSameType(b);" << std::endl;
  d2.isSameType(b);
  std::cout << "d2.isSameType(bb);" << std::endl;
  d2.isSameType(bb);
  std::cout << "d2.isSameType(d1);" << std::endl;
  d2.isSameType(d1);
  std::cout << "d2.isSameType(d1b);" << std::endl;
  d2.isSameType(d1b);
  std::cout << "d2.isSameType(d2);" << std::endl;
  d2.isSameType(d2);
  std::cout << "d2.isSameType(d2b);" << std::endl;
  d2.isSameType(d2b);

  std::stringstream s = "1";
  s << "2";
  s << "3";
  std::cout << s.str() << std::endl;
}
