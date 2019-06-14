#include "gtest/gtest.h"
#include "..\Algorithms\graph.hpp"

TEST(test_graph_application, test_mst) {
  jas::algo::Graph<char> graph(1);
  auto& vertex = graph.AddVertex('a');
  EXPECT_FALSE(vertex.Visited());
}
