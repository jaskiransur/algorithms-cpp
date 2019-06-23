#ifndef jas_algo_weighted_graph_hpp
#define jas_algo_weighted_graph_hpp

#include <algorithm>
#include <boost/optional.hpp>
#include <boost/format.hpp>
#include <queue>
#include <stack>
#include <vector>

namespace jas { namespace algo {
namespace detail {
	
	template<typename T>
	struct ResetVisits
	{
		ResetVisits(std::vector<T>& vertices)
			:vertices_(vertices)
		{
		}

		~ResetVisits()
		{
			for (auto& vertex : vertices_)
				vertex.Visit(false);
		}
		std::vector<T>& vertices_;
	};
	
	template<typename T>
	struct WeightedGraphTraversal
	{
		WeightedGraphTraversal() = default;
		WeightedGraphTraversal(const WeightedGraphTraversal&) = default;
		WeightedGraphTraversal& operator=(const WeightedGraphTraversal&) = default;

		WeightedGraphTraversal(WeightedGraphTraversal&&) = default;
		WeightedGraphTraversal& operator=(WeightedGraphTraversal&&) = default;
		~WeightedGraphTraversal() = default;
		void Push(T& vertex)
		{
			vertices_.emplace_back(vertex);
		}
		std::vector<T> vertices_;
	};
}

template<class T>
class Vertex
{
public:
	Vertex(T type);
	bool& Visited();
	const bool& Visited() const;
	void Visit(bool state = true);
	T& Type();
	const T& Type() const;
	bool operator==(const Vertex<T>& other);
private:
	T type_;
	bool visited_ = false;
};

template<class T>
inline Vertex<T>::Vertex(T type)
{
	type_ = std::move(type);
}

template<class T>
inline bool & Vertex<T>::Visited()
{
	return visited_;
}

template<class T>
inline const bool & Vertex<T>::Visited() const
{
	return visited_;
}

template<class T>
inline void Vertex<T>::Visit(bool state)
{
	visited_ = state;
}

template<class T>
inline bool Vertex<T>::operator==(const Vertex<T>& other)
{
	return this->Type() == other.Type();
}

template<class T>
inline T& Vertex<T>::Type()
{
	return type_;
}

template<class T>
inline const T& Vertex<T>::Type() const
{
	return type_;
}

template<class T, bool directed = false>
class WeightedGraph
{
public:
	using value_type = Vertex<T>;
	WeightedGraph(size_t vertices);
	Vertex<T>& AddVertex(T type);

	void AddEdge(const Vertex<T>& first, const Vertex<T>& second);
	void AddEdge(const T& first, const T& second);

	//minimum spanning tree weighted
	detail::WeightedGraphTraversal<Vertex<T>> MSTW(T startingVertex);

	int FindLeafNode();
	int GetAdjUnvisitedVertex(Vertex<T>& vertex);

private:
	const size_t verticesSize_;	
	std::vector<Vertex<T>> vertices_;
	std::vector<std::vector<unsigned int>> adjMatrix_;
};

#define TEMPLATE template<class T, bool directed>
#define SCOPE WeightedGraph<T, directed> 

TEMPLATE
inline SCOPE::WeightedGraph(size_t verticesSize)
	:verticesSize_(verticesSize)
{
	vertices_.reserve(verticesSize_);
	adjMatrix_.resize(verticesSize_);
	for (size_t i = 0; i < verticesSize_; i++)
	{
		adjMatrix_[i].resize(verticesSize_);
		for (size_t j = 0; j < verticesSize_; j++)
		{
			adjMatrix_[i][j] = 0;
		}
	}
}

TEMPLATE
inline Vertex<T>& SCOPE::AddVertex(T type)
{
	if (vertices_.size() >= verticesSize_)
		throw std::runtime_error("WeightedGraph has no space for new vertex");
	vertices_.emplace_back(Vertex<T>(type));
	return vertices_.back();
}

TEMPLATE
inline void SCOPE::AddEdge(const T& first, const T& second)
{
	auto index1 = std::distance(vertices_.begin(), std::find_if(vertices_.begin(), vertices_.end(),
		[first](const Vertex<T>& vertex) {return vertex.Type() == first;}));
	auto index2 = std::distance(vertices_.begin(), std::find_if(vertices_.begin(), vertices_.end(), 
		[second](const Vertex<T>& vertex) {return vertex.Type() == second;}));
	
	adjMatrix_[index1][index2] = 1;
	if (!directed)
		adjMatrix_[index2][index1] = 1;
}

TEMPLATE
inline void SCOPE::AddEdge(const Vertex<T>& first, const Vertex<T>& second)
{
	auto index1 = std::distance(vertices_.begin(), std::find(vertices_.begin(), vertices_.end(), first));
	auto index2 = std::distance(vertices_.begin(), std::find(vertices_.begin(), vertices_.end(), second));
	
	adjMatrix_[index1][index2] = 1;

	if (!directed)
		adjMatrix_[index2][index1] = 1;
}

TEMPLATE
inline Vertex<T>& SCOPE::FindVertex(T type)
{
	auto index = std::distance(vertices_.begin(), std::find(vertices_.begin(), vertices_.end(), type));
	return vertices_[index];
}

TEMPLATE
inline int SCOPE::GetAdjUnvisitedVertex(Vertex<T>& vertex)
{
	size_t index = std::distance(vertices_.begin(), std::find(vertices_.begin(), vertices_.end(), vertex));
	for (size_t i = 0; i < adjMatrix_[index].size(); i++)
	{
		if (adjMatrix_[index][i] == 1 && !vertices_[i].Visited())
		{
			return i;
		}
	}
	return -1;
}

TEMPLATE
inline detail::WeightedGraphTraversal<Vertex<T>> SCOPE::MSTW(T startingVertex) 
{
	//raii to reset the visits back to not visited
	detail::ResetVisits<Vertex<T>> resetVisits(vertices_);

	detail::WeightedGraphTraversal<Vertex<T>> graphTraversal;

	std::stack<Vertex<T>> dfsStack;
	auto& start = FindVertex(startingVertex);
	start.Visit();
	dfsStack.push(start);
	graphTraversal.Push(start);

	while (!dfsStack.empty())
	{
		int adjacentVertex = GetAdjUnvisitedVertex(dfsStack.top());
		if (adjacentVertex == -1)
		{ 
			dfsStack.pop();
		}
		else
		{
			graphTraversal.Push(vertices_[adjacentVertex]);
			vertices_[adjacentVertex].Visit();
			dfsStack.push(vertices_[adjacentVertex]);
		}
	}
	return graphTraversal;
}

TEMPLATE
int SCOPE::FindLeafNode()
{
	for (int row = 0; row < vertices_.size(); ++row)
	{
		bool edge = false;
		for (int col = 0; col < adjMatrix_[row].size(); ++col)
		{
			if (adjMatrix_[row][col] == 1)
			{
				edge = true;
				break;
			}
		}
		if (edge == false)
			return row;
	}
	return -1;
}
#undef TEMPLATE
#undef SCOPE

}}
#endif//jas_algo_weighted_graph_hpp