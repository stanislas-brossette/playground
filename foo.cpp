# include <memory>

using namespace std;

unique_ptr<double> foo()
{
  return unique_ptr<double>(new double(8));
}

struct A
{
  A (){};
  shared_ptr<double> a_;
};

int main(int argc, char *argv[])
{
  A myA;
  myA.a_.reset(foo().release());
  return 0;
}
