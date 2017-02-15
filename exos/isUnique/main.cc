#include <iostream>
#include <string>
#include <vector>

bool isUnique(const std::string & s)
{
  if(s.length() > 128)
  {
    return false;
  }

  std::vector<bool> used_char(128, false);
  for (int i = 0; i < s.length(); i++) 
  {
    size_t is = s[i];
    if(used_char[is])
    {
      std::cout << s[i] << " found twice" << std::endl;
      return false;
    }
    else
    {
      used_char[is] = true;
    }
  }
  return true;
}


int main(void)
{
  std::string sUnique("qwerty");
  std::string sNonUnique("qwertey");
  std::string sEmpty("");
  std::cout << "sUnique is unique: " << isUnique(sUnique) << std::endl;
  std::cout << "sNonUnique is unique: " << isUnique(sNonUnique) << std::endl;
  std::cout << "sEmpty is unique: " << isUnique(sEmpty) << std::endl;
  return 0;
}
