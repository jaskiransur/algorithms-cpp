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
	
	template<typename T>
	struct GraphTraversal
	{
		GraphTraversal() = default;
		GraphTraversal(size_t size);
		GraphTraversal(const GraphTraversal&) = default;
		GraphTraversal& operator=(const GraphTraversal&) = default;

		GraphTraversal(GraphTraversal&&) = default;
		GraphTraversal& operator=(GraphTraversal&&) = default;
		~GraphTraversal() = default;
		void PushFront(T& vertex);
		void PushBack(T& vertex);
		std::vector<T> vertices_;
	};

	template<typename T>
	inline GraphTraversal<T>::GraphTraversal(size_t size)
	{
		vertices_.reserve(size);
	}

	template<typename T>
	inline void GraphTraversal<T>::PushFront(T& vertex)
	{
		vertices_.insert(vertices_.begin(), vertex);
	}
	
	template<typename T>
	inline void GraphTraversal<T>::PushBack(T& vertex)
	{
		vertices_.emplace_back(vertex);
	}
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
class Graph
{
public:
	using value_type = Vertex<T>;
	Graph(size_t vertices);
	Vertex<T>& DepthFirstSearch(T type);
	Vertex<T>& BreadthFirstSearch(T type);
	Vertex<T>& AddVertex(T type);
	Vertex<T>& FindVertex(T type);

	void AddEdge(const Vertex<T>& first, const Vertex<T>& second);
	void AddEdge(const T& first, const T& second);

	detail::GraphTraversal<Vertex<T>> BFS();
	detail::GraphTraversal<Vertex<T>> DFS();
	detail::GraphTraversal<Vertex<T>> MST(T startingVertex);

	detail::GraphTraversal<Vertex<T>> TopologicalSort();
	int GetAdjUnvisitedVertex(Vertex<T>& vertex);

private:
	const size_t verticesSize_;	
	std::vector<Vertex<T>> vertices_;
	std::vector<std::vector<unsigned int>> adjMatrix_;
};

#define TEMPLATE template<class T, bool directed>
#define SCOPE Graph<T, directed> 

TEMPLATE
inline SCOPE::Graph(size_t verticesSize)
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
		throw std::runtime_error("Graph has no space for new vertex");
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
	
	//add an edge to adjacent matrix
	//if it is directed then add it only on the top part of the diagonal in the matrix
	adjMatrix_[index1][index2] = 1;
	if (!directed)
		adjMatrix_[index2][index1] = 1;
	//observe the symmetry across the diagonal in a matrix of an undirected graph
	//   A B C
	//A [0 1 1]
	//B [1 0 1]
	//C [1 1 0]

	//observe the non-symmetry across the diagonal in a matrix of a directed graph
	//   A B C
	//A [0 1 0]
	//B [0 0 1]
	//C [0 0 0]
}

TEMPLATE
inline void SCOPE::AddEdge(const Vertex<T>& first, const Vertex<T>& second)
{
	auto index1 = std::distance(vertices_.begin(), std::find(vertices_.begin(), vertices_.end(), first));
	auto index2 = std::distance(vertices_.begin(), std::find(vertices_.begin(), vertices_.end(), second));
	
	adjMatrix_[index1][index2] = 1;

	if (!directed)
		adjMatrix_[index2][index1] = 1;

	//observe the symmetry across the diagonal in a matrix of an undirected graph
	//   A B C
	//A [0 1 1]
	//B [1 0 1]
	//C [1 1 0]

	//observe the non-symmetry across the diagonal in a matrix of a directed graph
	//   A B C
	//A [0 1 0]
	//B [0 0 1]
	//C [0 0 0]
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
inline Vertex<T>& SCOPE::BreadthFirstSearch(T type)
{
	//raii to reset the visits back to not visited
	detail::ResetVisits<Vertex<T>> resetVisits(vertices_);

	if (vertices_[0].Type() == type)
		return vertices_[0];

	std::queue<Vertex<T>> bfsQueue;
	vertices_[0].Visit();
	bfsQueue.push(vertices_[0]);
//----------
//         + //add to queue
//----------

	while (!bfsQueue.empty())
	{
		//get an element from beginning of the 
		auto&& front = bfsQueue.front();
		int adjacentVertexIndex = -1;
		//while there are eges for this vertex
		//get them and add behind the queue

		while((adjacentVertexIndex = GetAdjUnvisitedVertex(front)) != -1)
		{
			if (type == vertices_[adjacentVertexIndex].Type())
			{
				return vertices_[adjacentVertexIndex];
			}

			vertices_[adjacentVertexIndex].Visit();
			bfsQueue.push(vertices_[adjacentVertexIndex]);
			//----------
			//		 +++ //add to queue on its end
			//----------
			//we are adding the children of a vertices right after it
			//so it becomes a breadth search
		}
		//remove an element from beginning of the queue
		bfsQueue.pop();
		//----------
		//		 ++ //remove from the begining of queue
		//----------

	}
	throw std::runtime_error((boost::format("%1%: unable to find a vertex for the requested type") %type).str());
}

// a-b-c
// |  |
// e  d
TEMPLATE
inline detail::GraphTraversal<Vertex<T>> SCOPE::BFS()
{
	//raii to reset the visits back to not visited
	detail::ResetVisits<Vertex<T>> resetVisits(vertices_);

	detail::GraphTraversal<Vertex<T>> graphTraversal;
	std::queue<Vertex<T>> bfsQueue;
	vertices_[0].Visit();
	bfsQueue.push(vertices_[0]);
//----------
//         a //add to queue
//----------

	while (!bfsQueue.empty())
	{
		//get from the front, don't remove yet
		//we could have used pop, but it doesn't return element
		auto&& front = bfsQueue.front();
		int adjacentVertexIndex = -1;
		while((adjacentVertexIndex = GetAdjUnvisitedVertex(front)) != -1)
		{
			vertices_[adjacentVertexIndex].Visit();
			graphTraversal.PushBack(vertices_[adjacentVertexIndex]);
			bfsQueue.push(vertices_[adjacentVertexIndex]);
			//----------
			//	   dceba //add to queue on its end
			//----------
			//we are adding the children of a vertices right after it
			//so it becomes a breadth search
		}
		//remove an element from beginning of the queue
		bfsQueue.pop();
		//----------
		//		 ++ //remove from the begining of queue
		//----------

	}
	return graphTraversal;
}

TEMPLATE
inline Vertex<T>& SCOPE::DepthFirstSearch(T type) 
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


// a-b-c
// |  |
// e  d
TEMPLATE
inline detail::GraphTraversal<Vertex<T>> SCOPE::DFS() 
{
	//raii to reset the visits back to not visited
	detail::ResetVisits<Vertex<T>> resetVisits(vertices_);

	detail::GraphTraversal<Vertex<T>> graphTraversal;
	std::stack<Vertex<T>> dfsStack;
	vertices_[0].Visit();
	dfsStack.push(vertices_[0]);
	//add an element to stack
	//|  |
	//|  |
	//|  |
	//| a| parent node goes here

	//iterate over the elements in the stack
	//keep on adding and removing element from top
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
			graphTraversal.PushBack(vertices_[adjacentVertex]);
			dfsStack.push(vertices_[adjacentVertex]);
			//add an element to stack
			//| e|      second child of a (e) 
			//| d|      second child of b (d); d doesn't have any child 
			//| c|      first child of b (c); c doesn't have any child
			//| b|      first child of a (b); 
			//| a|      a
			//observe that it goes in the depth to a child as far as the leaf node
			// and then then traverse back and move to other children
			//could be useful for searching to go deep
		}
	}
	return graphTraversal;
}

TEMPLATE
inline detail::GraphTraversal<Vertex<T>> SCOPE::MST(T startingVertex) 
{
	//raii to reset the visits back to not visited
	detail::ResetVisits<Vertex<T>> resetVisits(vertices_);

	detail::GraphTraversal<Vertex<T>> graphTraversal;

	std::stack<Vertex<T>> dfsStack;
	auto& start = FindVertex(startingVertex);
	start.Visit();
	dfsStack.push(start);
	graphTraversal.PushBack(start);

	while (!dfsStack.empty())
	{
		int adjacentVertex = GetAdjUnvisitedVertex(dfsStack.top());
		if (adjacentVertex == -1)
		{ 
			dfsStack.pop();
		}
		else
		{
			graphTraversal.PushBack(vertices_[adjacentVertex]);
			vertices_[adjacentVertex].Visit();
			dfsStack.push(vertices_[adjacentVertex]);
		}
	}
	return graphTraversal;
}

namespace ToplogicalSort
{

template<typename T>
class TopologicalSortImpl
{
public:
	explicit TopologicalSortImpl(std::vector<Vertex<T>> vertices, std::vector<std::vector<unsigned int>> adjMatrix);
	detail::GraphTraversal<Vertex<T>> Sort();
private:
	int FindLeafNode();
	void DeleteVertex(size_t index);
	std::vector<Vertex<T>> vertices_;
	int verticesSize_=0;
	std::vector<std::vector<unsigned int>> adjMatrix_;
};

template<typename T>
TopologicalSortImpl<T>::TopologicalSortImpl(std::vector<Vertex<T>> vertices, std::vector<std::vector<unsigned int>> adjMatrix)
:vertices_(std::move(vertices))
, adjMatrix_(std::move(adjMatrix))
{
	verticesSize_ = vertices_.size();
}

template<typename T>
int TopologicalSortImpl<T>::FindLeafNode()
{
	//find the leaf node with no successor
	for (int row = 0; row < verticesSize_; ++row)
	{
		bool edge = false;
		for (int col = 0; col < verticesSize_; ++col)
		{
			//if it has a 1, i.e. an edge, then it is not a leaf node,
			//exit and continue looking at the next leaf node
			if (adjMatrix_[row][col] == 1)
			{
				edge = true;
				break;
			}
		}
		//looked through all the vertex, and found a vertex with no edges
		//this is the leaf node
		if (edge == false)
			return row;
	}
	//no leaf node is found
	return -1;
}

template<typename T>
void TopologicalSortImpl<T>::DeleteVertex(size_t index)
{
	//remove the value from vertices
	for (size_t i = index; i < verticesSize_-1; ++i)
		vertices_[i] = vertices_[i + 1];

	//move the rows up
	for (size_t row = index; row < verticesSize_-1; ++row)
	{ 
		for (size_t col = 0; col < verticesSize_; ++col)
		{
			adjMatrix_[row][col] = adjMatrix_[row+1][col];
		}
	}

	//move the column to left
	for (size_t col = index; col < verticesSize_-1; ++col)
	{
		for (size_t row = 0; row < verticesSize_; ++row)
		{
			adjMatrix_[row][col] = adjMatrix_[row][col + 1];
		}
	}
	--verticesSize_;
}

template<typename T>
detail::GraphTraversal<Vertex<T>> TopologicalSortImpl<T>::Sort()
{
	verticesSize_ = vertices_.size();
	detail::GraphTraversal<Vertex<T>> graphTraversal(verticesSize_);
	while (verticesSize_ > 0)
	{
		//find the one with no successor
		auto leafNode = FindLeafNode();
		if (leafNode == -1)
		{
			//topological sort requires the graph to be DAG
			//direct acyclic graph
			throw std::runtime_error("the graph has no leaf node or has cycles; topological search is not possible!");
		}
		graphTraversal.PushFront(vertices_[leafNode]);
		DeleteVertex(leafNode);
	}
	return graphTraversal;
}
}

TEMPLATE
detail::GraphTraversal<Vertex<T>> SCOPE::TopologicalSort()
{
	ToplogicalSort::TopologicalSortImpl<T> topSort(vertices_, adjMatrix_);
	return topSort.Sort();
}
 
#undef TEMPLATE
#undef SCOPE

}}
#endif//jas_algo_graph_hpp