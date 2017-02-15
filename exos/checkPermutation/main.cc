#include <iostream>
#include <algorithm>

bool checkPermutation(const std::string& s1, const std::string& s2)
{
  if(s1.length() != s2.length())
    return false;
  std::string s1Sorted = s1;
  std::string s2Sorted = s2;
  std::sort(s1Sorted.begin(), s1Sorted.end());
  std::sort(s2Sorted.begin(), s2Sorted.end());
  if(s1Sorted.compare(s2Sorted) == 0)
    return true;
  return false;
}

void printIsPerm(const std::string& s1, const std::string& s2)
{
  if(checkPermutation(s1, s2))
    std::cout << "\"" << s1 << "\" and \"" << s2 << "\" are permutations" << std::endl;
  else                                                 
    std::cout << "\"" << s1 << "\" and \"" << s2 << "\" are NOT permutations" << std::endl;
}

int main(void)
{
  std::string s1("hello");
  std::string s2("helloagehalekh");
  std::string s3("lohle");
  std::string s4("");

  printIsPerm(s1, s2);
  printIsPerm(s1, s3);
  printIsPerm(s2, s3);
  printIsPerm(s2, s2);
  printIsPerm(s4, s4);

  return 0;
}
