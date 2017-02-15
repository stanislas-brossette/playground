#include <iostream>
#include <map>

void print( const std::map<std::string, double>& a = std::map<std::string, double>())
{
  std::cout << "printing map" << std::endl;
  for (auto& e: a)
    std::cout << e.first << " " << e.second << std::endl;
}

int main()
{
  print();
  std::map<std::string, double> b;
  b["c"] = 3;
  b["e"] = 4;
  b["t"] = 5;
  print(b);
  std::string name("c");
  std::cout << "b(name):\n" << b[name] << std::endl;  

  return 0;
}

