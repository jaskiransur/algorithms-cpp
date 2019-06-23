#include "gtest/gtest.h"
#include "..\Algorithms\graph.hpp"

TEST(test_directed_graph_functions, test_directed_graph) {
  jas::algo::Graph<char, true> graph(1);
  auto& vertex = graph.AddVertex('a');
  EXPECT_FALSE(vertex.Visited());
}
