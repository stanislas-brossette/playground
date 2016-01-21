#include <iostream>
#ifndef NDEBUG
#   define ASSERT(condition, message) \
    do { \
        if (! (condition)) { \
            std::cerr << "Assertion `" #condition "` failed in " << __FILE__ \
                      << " line " << __LINE__ << ": " << message << std::endl; \
            std::exit(EXIT_FAILURE); \
        } \
    } while (false)
#else
#   define ASSERT(condition, message) do { } while (false)
#endif

void bar(int i)
{
  ASSERT(i>0, "Fail i = " << i << std::endl);
  std::cout << "Success i = " << i << std::endl;
}

int main()
{
  bar(5);
  bar(-4);
  return 0;
}