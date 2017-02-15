#include <iostream>
#include <string>

void permutation(std::string s, std::string pref)
{
  if (s.length() == 0)
  {
    std::cout << pref << std::endl;
  }
  else
  {
    for (int i = 0; i < s.length(); i++) 
    {
      std::string rem = s.substr(0,i) + s.substr(i+1);
      permutation(rem, pref+s.at(i));
    }
  }
}

void permutation(std::string s)
{
  permutation(s,"");
}


int main(void)
{
  std::string s("Hel");
  permutation(s);
  return 0;
}
