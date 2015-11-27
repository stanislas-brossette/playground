#pragma once

#include <memory>
#include <iostream>

using namespace std;

class A
{
public:
  class A_ 
  {
    friend A;
  public:
    A_ () {}
    virtual ~A_ () {}
  private:
    virtual void print() {
      cout << "A_::print()" << endl;
    }
  };

  A () : myA_(new A_) {}
  A (shared_ptr<A_> a) : myA_(a) {}

  virtual ~A () {};

  virtual void print() {
    cout << "A::print()" << endl;
    myA_->print();
  }

protected:
  shared_ptr<A_> myA_;
};

