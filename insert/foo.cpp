#include <iostream>
#include <vector>

using namespace std;

void print(vector<int> v)
{
  for (auto e: v)
  {
    std::cout << e << ", ";
  }
  std::cout << std::endl;
}

int main(int argc, char *argv[])
{
  vector<int> a = {3, 4, 5, 6, 7};
  vector<int> b = {4, 3, 2, 1, 0};
  print(a);
  print(b);
  a.insert (a.end(),b.begin(), b.end());
  print(a);

  return 0;
}
