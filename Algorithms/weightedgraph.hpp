#ifndef jas_algo_weighted_graph_hpp
#define jas_algo_weighted_graph_hpp

#include <algorithm>
#include <boost/optional.hpp>
#include <boost/format.hpp>
#include <queue>

namespace jas{ namespace algo {

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

struct Edge
{
	int srcVert_=0;
	int destVert_=0;
	int weight_=0;
};

template<class T>
class Vertex
{
public:
	Vertex(T type);
	bool& Visited();
	const bool& InTree() const;
	void InTree(bool state = true);
	T& Type();
	const T& Type() const;
	bool operator==(const Vertex<T>& other);
private:
	T type_;
	bool inTree_ = false;
};

template<class T>
inline Vertex<T>::Vertex(T type)
{
	type_ = std::move(type);
}

template<class T>
inline const bool & Vertex<T>::InTree() const
{
	return inTree_;
}

template<class T>
inline void Vertex<T>::InTree(bool state)
{
	inTree_ = state;
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

	void AddEdge(const Vertex<T>& first, const Vertex<T>& second, int weight);
	void AddEdge(const T& first, const T& second, int weight);

	//minimum spanning tree weighted
	detail::WeightedGraphTraversal<Vertex<T>> MSTW(T startingVertex);

private:
	void PutInPQ(int vertexIndex, int weight);
	const size_t verticesSize_; //total supported vertices
	std::vector<Vertex<T>> vertices_;
	std::vector<std::vector<unsigned int>> adjMatrix_;
	auto cmp = [](Edge left, Edge right) { return left.weight_ < right.weight_; };
	std::priority_queue<Edge, std::vector<Edge>, decltype(cmp)> prQueue_;
	int nVerts_=0; //number of vertices
	int currentVert_=0; //current vertices
	int verticesInTree_=0; //vertices in tree
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
			adjMatrix_[i][j] = std::numeric_limits<int>::infinity();
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
inline void SCOPE::AddEdge(const T& first, const T& second, int weight)
{
	auto index1 = std::distance(vertices_.begin(), std::find_if(vertices_.begin(), vertices_.end(),
		[first](const Vertex<T>& vertex) {return vertex.Type() == first; }));
	auto index2 = std::distance(vertices_.begin(), std::find_if(vertices_.begin(), vertices_.end(),
		[second](const Vertex<T>& vertex) {return vertex.Type() == second; }));

	adjMatrix_[index1][index2] = weight;
	if (!directed)
		adjMatrix_[index2][index1] = weight;
}

TEMPLATE
inline void SCOPE::AddEdge(const Vertex<T>& first, const Vertex<T>& second, int weight)
{
	auto index1 = std::distance(vertices_.begin(), std::find(vertices_.begin(), vertices_.end(), first));
	auto index2 = std::distance(vertices_.begin(), std::find(vertices_.begin(), vertices_.end(), second));

	adjMatrix_[index1][index2] = weight;

	if (!directed)
		adjMatrix_[index2][index1] = weight;
}

TEMPLATE
inline void SCOPE::PutInPQ(int vertexIndex, int weight)
{
	prQueue_.
}

TEMPLATE
inline detail::WeightedGraphTraversal<Vertex<T>> SCOPE::MSTW(T startingVertex)
{
	detail::WeightedGraphTraversal<Vertex<T>> weightedGraphTraversal;
	currentVert_ = 0;
	while (verticesInTree_ < nVerts_ - 1)
	{
		vertices_[currentVert_].InTree(true);
		verticesInTree_++;
		for (int j = 0; j < nVerts_; ++j)
		{
			if (j == currentVert_ || vertices_[j].InTree())
				continue;
			int weight = adjMatrix_[currentVert_][j]; //get weight
			if (std::isinf<int>(weight))//has no edge
				continue;
			//put in queue
			PutInPQ(j, weight);
		}
		//check pr queue has elements in it
		if (prQueue_.empty())
			throw std::runtime_error("priority queue is empty; graph is not connected");
		Edge edge = prQueue_.top();
		weightedGraphTraversal.Push(vertices_[edge.srcVert_]);
		weightedGraphTraversal.Push(vertices_[edge.destVert_]);
		prQueue_.pop();
	}
}
#undef TEMPLATE
#undef SCOPE
}}
#endif//jas_algo_weighted_graph_hpp