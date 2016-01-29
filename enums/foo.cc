#include <iostream>

enum COORD_FLAG
{
  TRANSLATION_X = 1 << 0,
  TRANSLATION_Y = 1 << 1,
  TRANSLATION_Z = 1 << 2,
  ROTATION_X = 1 << 3,
  ROTATION_Y = 1 << 4,
  ROTATION_Z = 1 << 5
};
enum TRANS
{
  T___ = 0,
  TX__ = ROTATION_X,
  T_Y_ = ROTATION_Y,
  T__Z = ROTATION_Z,
  TXY_ = ROTATION_X | ROTATION_Y,
  T_YZ = ROTATION_Y | ROTATION_Z,
  TX_Z = ROTATION_X | ROTATION_Z,
  TXYZ = ROTATION_X | ROTATION_Y | ROTATION_Z,
};
enum ROT
{
  R___ = 0,
  RX__ = TRANSLATION_X,
  R_Y_ = TRANSLATION_Y,
  R__Z = TRANSLATION_Z,
  RXY_ = TRANSLATION_X | TRANSLATION_Y,
  R_YZ = TRANSLATION_Y | TRANSLATION_Z,
  RX_Z = TRANSLATION_X | TRANSLATION_Z,
  RXYZ = TRANSLATION_X | TRANSLATION_Y | TRANSLATION_Z,
};
int main(int argc, char *argv[])
{
  std::cout << "R___ = " << R___ << std::endl;
  std::cout << "RX__ = " << RX__ << std::endl;
  std::cout << "R_Y_ = " << R_Y_ << std::endl;
  std::cout << "R__Z = " << R__Z << std::endl;
  std::cout << "RXY_ = " << RXY_ << std::endl;
  std::cout << "R_YZ = " << R_YZ << std::endl;
  std::cout << "RX_Z = " << RX_Z << std::endl;
  std::cout << "RXYZ = " << RXYZ << std::endl;

  std::cout << "T___ = " << T___ << std::endl;
  std::cout << "TX__ = " << TX__ << std::endl;
  std::cout << "T_Y_ = " << T_Y_ << std::endl;
  std::cout << "T__Z = " << T__Z << std::endl;
  std::cout << "TXY_ = " << TXY_ << std::endl;
  std::cout << "T_YZ = " << T_YZ << std::endl;
  std::cout << "TX_Z = " << TX_Z << std::endl;
  std::cout << "TXYZ = " << TXYZ << std::endl;
  return 0;
}

