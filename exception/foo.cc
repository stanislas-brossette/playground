#include <sstream>
#include <stdexcept>

int main(void)
{
  int a = 3;
  std::stringstream s;
  s << "Coucou" << a;
  throw std::length_error(s.str());

  return 0;
}
