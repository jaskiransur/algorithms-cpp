#include "gtest/gtest.h"
#include "..\Algorithms\graph.hpp"

TEST(test_graph_functions, test_graph) {
  jas::algo::Graph<char> graph(1);
  auto& vertex = graph.AddVertex('a');
  EXPECT_FALSE(vertex.Visited());
}

TEST(test_graph_functions, test_simple_graph) 
{
  jas::algo::Graph<char> graph(3);
  auto& vertexA = graph.AddVertex('a');
  auto& vertexB = graph.AddVertex('b');
  auto& vertexC = graph.AddVertex('c');
  graph.AddEdge(vertexA, vertexB);
  graph.AddEdge(vertexA, vertexC);
  EXPECT_TRUE(graph.GetAdjUnvisitedVertex(vertexA) != -1);
}

TEST(test_graph_functions, test_graph_dfs) 
{
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

TEST(test_graph_functions, test_graph_dfs_traversal)
{
	jas::algo::Graph<char> graph(3);
	auto& vertexA = graph.AddVertex('a');
	auto& vertexB = graph.AddVertex('b');
	auto& vertexC = graph.AddVertex('c');
	graph.AddEdge(vertexA, vertexB);
	graph.AddEdge(vertexA, vertexC);
	EXPECT_NO_THROW(graph.DFS());
}
	
TEST(test_graph_functions, test_graph_bfs_throws)
{
	auto vertices = { 'a', 'b', 'c', 'd', 'e', 'f', 'g' };

	jas::algo::Graph<char> graph(vertices.size());
	std::vector<jas::algo::Vertex<char>> addedVertices;

	for (auto&& vertex : vertices)
	{
		addedVertices.emplace_back(graph.AddVertex(vertex));
	}

	graph.AddEdge('a', 'b');
	graph.AddEdge('a', 'c');
	graph.AddEdge('b', 'd');
	graph.AddEdge('b', 'e');
	graph.AddEdge('b', 'f');
	graph.AddEdge('b', 'g');

	EXPECT_THROW(graph.BreadthFirstSearch('h'), std::runtime_error);
}

TEST(test_graph_functions, test_graph_bfs)
{
	auto vertices = { 'a', 'b', 'c', 'd', 'e', 'f', 'g' };

	jas::algo::Graph<char> graph(vertices.size());
	std::vector<jas::algo::Vertex<char>> addedVertices;

	for (auto&& vertex : vertices)
	{
		addedVertices.emplace_back(graph.AddVertex(vertex));
	}

	graph.AddEdge('a', 'b');
	graph.AddEdge('a', 'c');
	graph.AddEdge('b', 'd');
	graph.AddEdge('b', 'e');
	graph.AddEdge('b', 'f');
	graph.AddEdge('b', 'g');

	auto& vertexNext = graph.BreadthFirstSearch('a').Type();
	EXPECT_TRUE('a' == vertexNext);

	auto& vertexNext1 = graph.BreadthFirstSearch('b').Type();
	EXPECT_TRUE('b' == vertexNext1);

	auto& vertexNext2 = graph.BreadthFirstSearch('c').Type();
	EXPECT_TRUE('c' == vertexNext2);

	auto& vertexNext3 = graph.BreadthFirstSearch('d').Type();
	EXPECT_TRUE('d' == vertexNext3);

	auto& vertexNext4 = graph.BreadthFirstSearch('e').Type();
	EXPECT_TRUE('e' == vertexNext4);

	auto& vertexNext5 = graph.BreadthFirstSearch('f').Type();
	EXPECT_TRUE('f' == vertexNext5);

	auto& vertexNext6 = graph.BreadthFirstSearch('g').Type();
	EXPECT_TRUE('g' == vertexNext6);
}

TEST(test_graph_functions, test_graph_bfs_traversal)
{
	auto vertices = { 'a', 'b', 'c', 'd', 'e', 'f', 'g' };

	jas::algo::Graph<char> graph(vertices.size());
	std::vector<jas::algo::Vertex<char>> addedVertices;

	for (auto&& vertex : vertices)
	{
		addedVertices.emplace_back(graph.AddVertex(vertex));
	}

	graph.AddEdge('a', 'b');
	graph.AddEdge('a', 'c');
	graph.AddEdge('b', 'd');
	graph.AddEdge('b', 'e');
	graph.AddEdge('b', 'f');
	graph.AddEdge('b', 'g');
	
	EXPECT_NO_THROW(graph.BFS());
}
