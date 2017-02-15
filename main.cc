#include <iostream>
#include <string>

int main(void)
{
  std::string a("HelloWorld");
  std::string b("HelloWorld");
  std::string c("Coucou");
  std::cout << "a.compare(b): " << a.compare(b) << std::endl;
  std::cout << "a.compare(c): " << a.compare(c) << std::endl;
  std::cout << "std::string(\"Hello\").compare(a): " << std::string("Hello").compare(a) << std::endl;
  std::cout << "std::string(\"HelloWorld\").compare(a): " << std::string("HelloWorld").compare(a) << std::endl;

  return 0;
}
