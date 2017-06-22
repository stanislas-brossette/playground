#include <iostream>
#include <vector>
#include <cassert>


int main(void)
{
  std::cout << "HelloWorld" << std::endl;
  assert(false && "It's debug");
  std::vector<int> a;
  a.resize(4);
  for (size_t i = 0; i < 7; i++)
  {
      std::cout << a.at(i) << std::endl;
  }
  return 0;
}
