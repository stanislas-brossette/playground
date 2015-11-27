#include <iostream>

class exprPtr
{
public:
  exprPtr ();
  virtual ~exprPtr ();

  std::shared_ptr<double> ptr_;
};

int main(int argc, char *argv[])
{
  
  return 0;
}
