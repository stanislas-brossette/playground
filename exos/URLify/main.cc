#include <iostream>
#include <string>

int countSpaces(const std::string& str, const int& lgth)
{
  int res = 0;
  for (int i = 0; i < lgth; i++) 
  {
    if(str[i] == ' ')
      res++;
  }
  return res;
}

std::string urlify(std::string& str, const int& lgth)
{
  std::cout << "\"" <<  str << "\" becomes \"";
  int nSpace = countSpaces(str, lgth);
  int cursorURL = str.length() - 1;

  for (int i = lgth - 1; i >= 0; i--) 
  {
    if (str[i] == ' ')
    {
      str[cursorURL] = '0';
      str[cursorURL-1] = '2';
      str[cursorURL-2] = '%';
      cursorURL -= 3;
    }
    else
    {
      str[cursorURL] = str[i];
      cursorURL--;
    }
  }
  std::cout << str << "\"" << std::endl;
  return str;
}

int main(void)
{
  std::string s1("");
  int i1 = 0;
  std::string s2("string");
  int i2 = 6;
  std::string s3("it is a string      ");
  int i3 = 14;
  std::string s4(" and another one         ");
  int i4 = 17;

  urlify(s1, i1);
  urlify(s2, i2);
  urlify(s3, i3);
  urlify(s4, i4);

  return 0;
}
