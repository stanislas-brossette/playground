#include <iostream>
#include <functional>
#include <string>
#include <vector>

class AddressBook
{
    public:
      AddressBook()
        : _addresses({"a.org", "test", "myAddress", "stan@gmail.com"})
      {
      }

    // using a template allows us to ignore the differences between functors, function pointers 
    // and lambda
    std::vector<std::string> findMatchingAddresses (std::function<bool (const std::string&)> func)
    { 
        std::vector<std::string> results;
        for ( auto itr = _addresses.begin(), end = _addresses.end(); itr != end; ++itr )
        {
            // call the function passed into findMatchingAddresses and see if it matches
            if ( func( *itr ) )
            {
                results.push_back( *itr );
            }
        }
        return results;
    }

    private:
    std::vector<std::string> _addresses;
};

using namespace std;
int main(int argc, char *argv[])
{
  AddressBook myBook;
  string name;
  cin>> name;
  auto a = myBook.findMatchingAddresses( 
    // notice that the lambda function uses the the variable 'name'
    [&] (const string& addr) { return addr.find( name ) != string::npos; });

  for (auto e: a)
  {
    std::cout << e << std::endl;
  }

  return 0;
}
