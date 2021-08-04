#include <pibt_plus.hpp>

#include "gtest/gtest.h"

TEST(PIBT_PLUS, solve)
{
  Problem P = Problem("../tests/instances/example.txt");
  auto solver = std::make_unique<PIBT_PLUS>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}
