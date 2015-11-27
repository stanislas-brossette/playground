#include <iostream>

void foo(int i)
{
  if (i < 50) {
    std::cout << i << std::endl;
    ++i;
  }
  else
    return;
  foo(i);
}

int main(int argc, char *argv[])
{
  foo(24);
  
  return 0;
}
