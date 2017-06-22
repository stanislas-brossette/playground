#include <iostream>
#include <boost/shared_ptr.hpp>

int main(void)
{
    boost::shared_ptr<double> test(new double);
    *test = 334;
    std::cout << *test << std::endl;
    return 0;
}
