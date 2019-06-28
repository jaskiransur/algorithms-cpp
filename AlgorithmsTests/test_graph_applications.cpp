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

jas::algo::Graph<std::string, true> CreateSchoolCoursesGraph()
{
	std::vector<std::string> vertices =
	{ "differential equations",  //0
		"Infinite series", //1
		"numerical analysis", //2
		"calculus", //3
		"pre-calculus", //4 
		"trignometry", //5
		"algebra",  //6
		"geometry", //7
		"pre-algebra" };//8

	jas::algo::Graph<std::string, true> graph(vertices.size());
	std::vector<jas::algo::Vertex<std::string>> addedVertices;

	for (auto&& vertex : vertices)
	{
		addedVertices.emplace_back(graph.AddVertex(vertex));
	}

	graph.AddEdge(vertices[0], vertices[3]);
	graph.AddEdge(vertices[0], vertices[1]);
	graph.AddEdge(vertices[3], vertices[4]);
	graph.AddEdge(vertices[4], vertices[5]);
	graph.AddEdge(vertices[5], vertices[6]);
	graph.AddEdge(vertices[5], vertices[7]);
	graph.AddEdge(vertices[6], vertices[8]);
	graph.AddEdge(vertices[1], vertices[6]);
	graph.AddEdge(vertices[2], vertices[0]);
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

TEST(test_graph_application, test_topological_sort_courses)
{
	jas::algo::Graph<std::string, true> graph = CreateSchoolCoursesGraph();
	jas::algo::detail::GraphTraversal<jas::algo::Vertex<std::string>> graphTraversal = graph.TopologicalSort();
	EXPECT_TRUE(graphTraversal.vertices_.size() == 9);
	
	std::vector<std::string> expectedVertices =
	{ 
		"numerical analysis", //2
		"differential equations",  //0
		"calculus", //3
		"pre-calculus", //4 
		"trignometry", //5
		"Infinite series", //1
		"algebra",  //6
		"pre-algebra", //8
		"geometry", //7
	};

	for (int i = 0; i< graphTraversal.vertices_.size(); ++i)
		EXPECT_TRUE(graphTraversal.vertices_[i].Type() == expectedVertices[i]);
}

