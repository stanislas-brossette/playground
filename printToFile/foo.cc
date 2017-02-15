#include <iostream>
#include <fstream>
#include <string>

int main(void)
{
  std::ofstream file;
  file.open("textFile.txt");
  file << "transLFoot: " << std::endl;
  file << "transRFoot: " << std::endl;
  file.close();
  std::ofstream file2;
  file2.open("textFile.txt", std::ios_base::app);
  file2 << "transLHand2: " << std::endl;
  file2 << "transRHand2: " << std::endl;
  file2.close();
  return 0;
}
