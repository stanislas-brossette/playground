#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/variant.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/variant/static_visitor.hpp>

class Foo
{
public:
  Foo (int a)
    : a_ (a)
  {}

  virtual ~Foo (){}

  virtual std::string name () const
  {
    std::stringstream ss;
    ss << "foo " << a_;
    return ss.str ();
  }

private:
  int a_;
};

/// \brief Yo dawg, I heard you like decorators, so here's
/// a decorator decorating a string that needs decorating
/// by a decorator.
template <typename F>
class Decorator : public F
{
public:
  Decorator (boost::shared_ptr<F> f,
             const std::string& pre, const std::string& post)
    : F (*f),
      f_ (f),
      pre_ (pre),
      post_ (post)
  {}

  virtual ~Decorator (){}

  virtual std::string name () const
  {
    return pre_ + f_->name () + post_;
  }

  const std::string& pre () const
  {
    return pre_;
  }

  const std::string& post () const
  {
    return post_;
  }

private:
  boost::shared_ptr<F> f_;
  std::string pre_;
  std::string post_;
};

typedef boost::variant<boost::shared_ptr<Foo> > typeList_t;
typedef std::vector<typeList_t> vector_t;

struct PrintVisitor : boost::static_visitor<void>
{
  template <typename T>
  void operator () (const T& f) const
  {
    std::cout << f->name () << std::endl;
  }
};

struct DecoratorCheckVisitor : boost::static_visitor<void>
{
  template <typename T>
  void operator () (const T& f) const
  {
    typedef typename T::element_type U;
    const Decorator<U>* d = NULL;

    d = dynamic_cast<const Decorator<U>*> (f.get ());

    if (d != NULL)
    {
      std::cout << "so pretty" << std::endl;
    }
    else
    {
      std::cout << "booooooring" << std::endl;
    }
  }
};

int main (int, char *[])
{
  boost::shared_ptr<Foo> f = boost::make_shared<Foo> (42);
  std::cout << f->name () << std::endl;

  boost::shared_ptr<Decorator<Foo> >
    d = boost::make_shared<Decorator<Foo> > (f, "(>'-')> ", " <('-'<)");
  std::cout << d->name () << std::endl;

  vector_t v;
  v.push_back (f);
  v.push_back (d);

  for (vector_t::const_iterator
       iter  = v.begin ();
       iter != v.end ();
       ++iter)
  {
    boost::apply_visitor (PrintVisitor (), *iter);
    boost::apply_visitor (DecoratorCheckVisitor (), *iter);
  }

  return 0;
}
