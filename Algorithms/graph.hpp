#ifndef jas_algo_graph_hpp
#define jas_algo_graph_hpp

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

template<class T>
class Graph
{
public:
	using value_type = Vertex<T>;
	Graph(size_t vertices);
	Vertex<T>& DepthFirstSearch(T type);
	Vertex<T>& BreadthFirstSearch(T type);
	Vertex<T>& AddVertex(T type);
	void AddEdge(const Vertex<T>& first, const Vertex<T>& second);
	void AddEdge(const char& first, const char& second);

	void DFS();
	void BFS();
	int GetAdjUnvisitedVertex(Vertex<T>& vertex);

private:
	const size_t verticesSize_;	
	std::vector<Vertex<T>> vertices_;
	std::vector<std::vector<unsigned int>> adjMatrix_;
};

template<class T>
inline Graph<T>::Graph(size_t verticesSize)
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

template<class T>
inline Vertex<T>& Graph<T>::AddVertex(T type)
{
	if (vertices_.size() >= verticesSize_)
		throw std::runtime_error("Graph has no space for new vertex");
	vertices_.emplace_back(Vertex<T>(type));
	return vertices_.back();
}

template<class T>
inline void Graph<T>::AddEdge(const char& first, const char& second)
{
	auto index1 = std::distance(vertices_.begin(), std::find_if(vertices_.begin(), vertices_.end(),
		[first](const Vertex<T>& vertex) {return vertex.Type() == first;}));
	auto index2 = std::distance(vertices_.begin(), std::find_if(vertices_.begin(), vertices_.end(), 
		[second](const Vertex<T>& vertex) {return vertex.Type() == second;}));
	
	adjMatrix_[index1][index2] = 1;
	adjMatrix_[index2][index1] = 1;
}

template<class T>
inline void Graph<T>::AddEdge(const Vertex<T>& first, const Vertex<T>& second)
{
	auto index1 = std::distance(vertices_.begin(), std::find(vertices_.begin(), vertices_.end(), first));
	auto index2 = std::distance(vertices_.begin(), std::find(vertices_.begin(), vertices_.end(), second));
	
	adjMatrix_[index1][index2] = 1;
	adjMatrix_[index2][index1] = 1;
}

template<class T>
inline int Graph<T>::GetAdjUnvisitedVertex(Vertex<T>& vertex)
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


template<class T>
inline Vertex<T>& Graph<T>::BreadthFirstSearch(T type)
{
	//raii to reset the visits back to not visited
	detail::ResetVisits<Vertex<T>> resetVisits(vertices_);

	if (vertices_[0].Type() == type)
		return vertices_[0];

	std::queue<Vertex<T>> bfsQueue;
	vertices_[0].Visit();
	bfsQueue.push(vertices_[0]);
	while (!bfsQueue.empty())
	{
		//remove from the top and 
		auto&& front = bfsQueue.front();
		int adjacentVertexIndex = -1;
		while((adjacentVertexIndex = GetAdjUnvisitedVertex(front)) != -1)
		{
			if (type == vertices_[adjacentVertexIndex].Type())
			{
				return vertices_[adjacentVertexIndex];
			}
			vertices_[adjacentVertexIndex].Visit();
			bfsQueue.push(vertices_[adjacentVertexIndex]);
		}
		bfsQueue.pop();
	}
	throw std::runtime_error((boost::format("%1%: unable to find a vertex for the requested type") %type).str());
}

template<class T>
inline void Graph<T>::BFS()
{
	//raii to reset the visits back to not visited
	detail::ResetVisits<Vertex<T>> resetVisits(vertices_);
	std::queue<Vertex<T>> bfsQueue;
	vertices_[0].Visit();
	bfsQueue.push(vertices_[0]);
	while (!bfsQueue.empty())
	{
		//remove from the top and 
		auto&& front = bfsQueue.front();
		int adjacentVertexIndex = -1;
		while((adjacentVertexIndex = GetAdjUnvisitedVertex(front)) != -1)
		{
			vertices_[adjacentVertexIndex].Visit();
			bfsQueue.push(vertices_[adjacentVertexIndex]);
		}
		bfsQueue.pop();
	}
}

template<class T>
inline Vertex<T>& Graph<T>::DepthFirstSearch(T type) 
{
	//raii to reset the visits back to not visited
	detail::ResetVisits<Vertex<T>> resetVisits(vertices_);

	std::stack<Vertex<T>> dfsStack;
	dfsStack.push(vertices_[0]);
	while (!dfsStack.empty())
	{
		int adjacentVertex = GetAdjUnvisitedVertex(dfsStack.top());
		if (adjacentVertex == -1)
		{ 
			dfsStack.pop();
		}
		else
		{
			if (vertices_[adjacentVertex].Type() == type)
				return vertices_[adjacentVertex];
			vertices_[adjacentVertex].Visit();
			dfsStack.push(vertices_[adjacentVertex]);
		}
	}
	throw std::runtime_error("Unable to find a vertex");
}

template<class T>
inline void Graph<T>::DFS() 
{
	//raii to reset the visits back to not visited
	detail::ResetVisits<Vertex<T>> resetVisits(vertices_);

	std::stack<Vertex<T>> dfsStack;
	vertices_[0].Visit();
	dfsStack.push(vertices_[0]);

	while (!dfsStack.empty())
	{
		int adjacentVertex = GetAdjUnvisitedVertex(dfsStack.top());
		if (adjacentVertex == -1)
		{ 
			dfsStack.pop();
		}
		else
		{
			vertices_[adjacentVertex].Visit();
			dfsStack.push(vertices_[adjacentVertex]);
		}
	}}
}
}
#endif//jas_algo_graph_hpp