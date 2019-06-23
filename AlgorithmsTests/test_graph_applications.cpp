#include "gtest/gtest.h"
#include "..\Algorithms\graph.hpp"

namespace
{
template<bool directed= false>
jas::algo::Graph<char, directed> CreateGraph()
{
	auto vertices = { 'a', 'b', 'c', 'd', 'e'};

	jas::algo::Graph<char, directed> graph(vertices.size());
	std::vector<jas::algo::Vertex<char>> addedVertices;

	for (auto&& vertex : vertices)
	{
		addedVertices.emplace_back(graph.AddVertex(vertex));
	}

	graph.AddEdge('a', 'b');
	graph.AddEdge('a', 'c');
	graph.AddEdge('a', 'd');
	graph.AddEdge('a', 'e');
	graph.AddEdge('b', 'c');
	graph.AddEdge('b', 'd');
	graph.AddEdge('b', 'e');
	graph.AddEdge('c', 'd');
	graph.AddEdge('c', 'e');
	graph.AddEdge('d', 'e');
	return graph;
}
}
TEST(test_graph_application, test_mst_a) {
	jas::algo::Graph<char> graph = CreateGraph();
	jas::algo::detail::GraphTraversal<jas::algo::Vertex<char>> graphTraversal = graph.MST('a');

	std::vector<jas::algo::Vertex<char>> vertices = { 'a', 'b', 'c', 'd', 'e'};
	EXPECT_TRUE(*graphTraversal.vertices_.data() == *vertices.data());
}

TEST(test_graph_application, test_mst_b)
{
	jas::algo::Graph<char> graph = CreateGraph();
	jas::algo::detail::GraphTraversal<jas::algo::Vertex<char>> graphTraversal = graph.MST('b');

	std::vector<jas::algo::Vertex<char>> vertices = { 'b', 'c', 'd', 'e', 'a'};
	EXPECT_TRUE(*graphTraversal.vertices_.data() == *vertices.data());
}

TEST(test_graph_application, test_mst_c)
{
	jas::algo::Graph<char> graph = CreateGraph();
	jas::algo::detail::GraphTraversal<jas::algo::Vertex<char>> graphTraversal = graph.MST('c');

	std::vector<jas::algo::Vertex<char>> vertices = { 'c', 'a', 'b', 'd', 'e'};
	EXPECT_TRUE(*graphTraversal.vertices_.data() == *vertices.data());
}

TEST(test_graph_application, test_topological_sort)
{
	jas::algo::Graph<char, true> graph = CreateGraph<true>();
	jas::algo::detail::GraphTraversal<jas::algo::Vertex<char>> graphTraversal = graph.TopologicalSort();

	std::vector<jas::algo::Vertex<char>> vertices = { 'a', 'b', 'c', 'd', 'e'};
	EXPECT_TRUE(*graphTraversal.vertices_.data() == *vertices.data());
}
