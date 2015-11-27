#pragma once

#include <memory>
#include <iostream>

#include "A.hh"

using namespace std;

class B: public A
{
public:
  class B_ : public A::A_
  {
    friend B;
  public:
    B_ () {}
    virtual ~B_ () {}
  private:
    void print() {
      cout << "B_::print()" << endl;
    }
  };

  B () 
    : A(static_pointer_cast<A_>(shared_ptr<B_>(new B_))) 
  {}

  virtual ~B () {}

  void print() {
    cout << "B::print()" << endl;
    static_pointer_cast<B_>(myA_)->print();
  }
};

