#include <iostream>
#include <limits>

#include <boost/test/unit_test.hpp>

#include <Eigen/Core>

#include <manifolds/RealSpace.h>
#include <manifolds/Point.h>

#include <pgsolver/solver/SolverTrustRegionFilter.h>
#include <pgsolver/utils/finiteDiff.h>
#include <pgsolver/utils/loadFromYaml.h>
#include <pgsolver/solver/Results.h>

using namespace mnf;
using namespace pgs;

BOOST_AUTO_TEST_CASE(testCV)
{
  SolverTrustRegionFilter * mySolver = new SolverTrustRegionFilter();
  BOOST_CHECK(1 == 0);
  delete mySolver;
}


//int main()
//{
//  SolverTrustRegionFilter mySolver;
//  return 0;
//}
