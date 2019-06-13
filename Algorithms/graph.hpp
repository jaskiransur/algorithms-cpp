#ifndef jas_algo_graph_hpp
#define jas_algo_graph_hpp

#include <algorithm>
#include <boost/optional.hpp>
#include <queue>
#include <stack>
#include <vector>

namespace jas {
	namespace algo {

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
			Graph(size_t vertices);
			void VisitUsingDepthFirstSearch(T type);
			Vertex<T>& AddVertex(T type);
			void AddEdge(const Vertex<T>& first, const Vertex<T>& second);

			Vertex<T>& DepthFirstSearch(T type);
			Vertex<T>& BreadthFirstSearch(T type);
			boost::optional<Vertex<T>> GetAdjUnvisitedVertex(Vertex<T>& vertex);

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
		inline void Graph<T>::AddEdge(const Vertex<T>& first, const Vertex<T>& second)
		{
			auto index1 = std::distance(vertices_.begin(), std::find(vertices_.begin(), vertices_.end(), first));
			auto index2 = std::distance(vertices_.begin(), std::find(vertices_.begin(), vertices_.end(), second));
			
			adjMatrix_[index1][index2] = 1;
			adjMatrix_[index2][index1] = 1;
		}

		template<class T>
		inline boost::optional<Vertex<T>> Graph<T>::GetAdjUnvisitedVertex(Vertex<T>& vertex)
		{
			size_t index = std::distance(vertices_.begin(), std::find(vertices_.begin(), vertices_.end(), vertex));
			for (size_t i = 0; i < adjMatrix_[index].size(); i++)
			{
				if (adjMatrix_[index][i] == 1 && !vertices_[i].Visited())
				{
					//vertices_[i].Visit();
					return vertices_[i];
				}
			}
			return boost::none;
		}

		template<class T>
		inline Vertex<T>& Graph<T>::BreadthFirstSearch(T type)
		{
			std::queue<Vertex<T>> queue;
			queue.push(vertices_[0]);
			while (!queue.empty())
			{
				auto adjacentVertex = GetAdjUnvisitedVertex(queue.front());
				if (!adjacentVertex.is_initialized())
				{
					queue.pop();
				}
				else if (adjacentVertex.get().Type() == type)
				{
					return adjacentVertex.get();
				}
				else if (!adjacentVertex.get().Visited())
				{
					queue.push(adjacentVertex.get());
					adjacentVertex.get().Visit();
				}
			}
			for (auto& vertex : vertices_)
				vertex.Visit(false);
			throw std::runtime_error("unable to find a vertex for the requested type");

		}

		template<class T>
		inline Vertex<T>& Graph<T>::DepthFirstSearch(T type) 
		{
			std::stack<Vertex<T>> stack;
			stack.push(vertices_[0]);
			while (!stack.empty())
			{
				auto adjacentVertex = GetAdjUnvisitedVertex(stack.top());
				if (!adjacentVertex.is_initialized())
				{
					stack.pop();
				}
				else if (adjacentVertex.get().Type() == type)
				{
					return adjacentVertex.get();
				}
				else if (!adjacentVertex.get().Visited())
				{
					stack.push(adjacentVertex.get());
					adjacentVertex.get().Visit();
				}
			}
			for (auto& vertex : vertices_)
				vertex.Visit(false);
			throw std::runtime_error("unable to find a vertex for the requested type");
		}

		template<class T>
		inline void Graph<T>::VisitUsingDepthFirstSearch(T type) 
		{
			std::stack<Vertex<T>> stack;
			stack.push(vertices_[0]);
			while (!stack.empty())
			{
				Vertex<T>& adjacentVertex = GetAdjUnvisitedVertex(stack.top());
				if (!adjacentVertex.Visited())
				{
					stack.push(adjacentVertex);
					adjacentVertex.Visit();
				}
				else
				{
					stack.pop();
				}
			}
			for (auto& vertex : vertices_)
				vertex.Visit(false);
		}
	}
}
#endif//jas_algo_graph_hpp