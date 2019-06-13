#include "gtest/gtest.h"
#include "..\Algorithms\graph.hpp"

//TEST(TestCaseName, TestName) {
//  EXPECT_EQ(1, 1);
//  EXPECT_TRUE(true);
//}
TEST(simple_construction_test, test_graph) {
  jas::algo::Graph<char> graph(1);
  auto& vertex = graph.AddVertex('a');
  EXPECT_FALSE(vertex.Visited());
}

TEST(simple_construction_test, test_simple_graph) {
  jas::algo::Graph<char> graph(3);
  auto& vertexA = graph.AddVertex('a');
  auto& vertexB = graph.AddVertex('b');
  auto& vertexC = graph.AddVertex('c');
  graph.AddEdge(vertexA, vertexB);
  graph.AddEdge(vertexA, vertexC);
  EXPECT_TRUE(graph.GetAdjUnvisitedVertex(vertexA).is_initialized());
  auto& vertexNext = graph.GetAdjUnvisitedVertex(vertexA).get().Type();
  EXPECT_TRUE(vertexB.Type() ==vertexNext);
}

TEST(simple_construction_test, test_graph_dfs) {
	jas::algo::Graph<char> graph(3);
	auto& vertexA = graph.AddVertex('a');
	auto& vertexB = graph.AddVertex('b');
	auto& vertexC = graph.AddVertex('c');
	graph.AddEdge(vertexA, vertexB);
	graph.AddEdge(vertexA, vertexC);
	auto& vertexNext = graph.DepthFirstSearch('a').Type();
	EXPECT_TRUE(vertexA.Type() == vertexNext);
	auto& vertexNext1 = graph.DepthFirstSearch('b').Type();
	EXPECT_TRUE(vertexB.Type() == vertexNext1);
	auto& vertexNext2 = graph.DepthFirstSearch('c').Type();
	EXPECT_TRUE(vertexC.Type() == vertexNext2);
}

TEST(simple_construction_test, test_graph_bfs) {
	jas::algo::Graph<char> graph(3);
	auto& vertexA = graph.AddVertex('a');
	auto& vertexB = graph.AddVertex('b');
	auto& vertexC = graph.AddVertex('c');
	auto& vertexD = graph.AddVertex('d');
	auto& vertexE = graph.AddVertex('e');
	auto& vertexF = graph.AddVertex('f');
	auto& vertexG = graph.AddVertex('g');

	graph.AddEdge(vertexA, vertexB);
	graph.AddEdge(vertexA, vertexC);
	graph.AddEdge(vertexB, vertexD);
	graph.AddEdge(vertexB, vertexE);
	graph.AddEdge(vertexB, vertexF);
	graph.AddEdge(vertexB, vertexG);

	auto& vertexNext = graph.BreadthFirstSearch('a').Type();
	EXPECT_TRUE(vertexA.Type() == vertexNext);
	auto& vertexNext1 = graph.BreadthFirstSearch('b').Type();
	EXPECT_TRUE(vertexB.Type() == vertexNext1);
	auto& vertexNext2 = graph.BreadthFirstSearch('c').Type();
	EXPECT_TRUE(vertexC.Type() == vertexNext2);

	auto& vertexNext3 = graph.BreadthFirstSearch('d').Type();
	EXPECT_TRUE(vertexD.Type() == vertexNext3);

	auto& vertexNext4 = graph.BreadthFirstSearch('e').Type();
	EXPECT_TRUE(vertexE.Type() == vertexNext4);

	auto& vertexNext5 = graph.BreadthFirstSearch('f').Type();
	EXPECT_TRUE(vertexF.Type() == vertexNext5);

	auto& vertexNext6 = graph.BreadthFirstSearch('g').Type();
	EXPECT_TRUE(vertexG.Type() == vertexNext6);
}
