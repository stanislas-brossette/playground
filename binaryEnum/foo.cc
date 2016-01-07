#include <iostream>

enum E {
  TX = 1 << 0,
  TY = 1 << 1,
  TZ = 1 << 2,
  RX = 1 << 3,
  RY = 1 << 4,
  RZ = 1 << 5
};

int size(int e)
{
  int r= 0;
  for (int i = 0; i < 6; ++i)
  {
    r+= (int)(bool)(e & (1 << i));
  }
  return r;
}

int main(int argc, char *argv[])
{
  int A1 = E::TX;
  int A2 = E::TY | E::TZ;
  int A3 = E::TZ;
  int A4 = E::RX | E::TX | E::TY | E::RZ;
  std::cout << "A1 = " << size(A1) << std::endl;
  std::cout << "A2 = " << size(A2) << std::endl;
  std::cout << "A3 = " << size(A3) << std::endl;
  std::cout << "A4 = " << size(A4) << std::endl;
  return 0;
}
