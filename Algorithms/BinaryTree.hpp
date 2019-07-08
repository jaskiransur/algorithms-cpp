#ifndef jas_algo_binary_tree_h
#define jas_algo_binary_tree_h
#include <vector>
template<typename T>
struct Node
{
	T value;
	Node* left = nullptr;
	Node* right = nullptr;
};

template<typename T>
using Nodes  = std::vector<T>;

template<typename T>
struct Tree
{
public:
//interface
	Node<T>* root = nullptr;
	Nodes<T> Inorder();
	Nodes<T> PreOrder();
	Nodes<T> PostOrder();

private://implementation
	void InorderImpl(Node<T>* root, Nodes<T>& nodes);
	void PreOrderImpl(Node<T>* root, Nodes<T>& nodes);
	void PostOrderImpl(Node<T>* root, Nodes<T>& nodes);
};

template<typename T>
inline Nodes<T> Tree<T>::Inorder()
{
	Nodes<T> nodes;
	if (root)
		InorderImpl(root, nodes);
	return nodes;
}

template<typename T>
inline Nodes<T> Tree<T>::PreOrder()
{
	Nodes<T> nodes;
	if (root)
		PreOrderImpl(root, nodes);
	return nodes;
}

template<typename T>
inline Nodes<T> Tree<T>::PostOrder()
{
	Nodes<T> nodes;
	if (root)
		PostOrderImpl(root, nodes);
	return nodes;
}

template<typename T>
inline void Tree<T>::InorderImpl(Node<T>* root, Nodes<T>& nodes)
{
	if (root->left)
		InorderImpl(root->left, nodes);
	nodes.emplace_back(root);
	if (root->right)
		InorderImpl(root->right, nodes);
}

template<typename T>
inline void Tree<T>::PreOrderImpl(Node<T>* root, Nodes<T>& nodes)
{
	nodes.emplace_back(root);
	if (root->left)
		PreOrderImpl(root->left, nodes);
	if (root->right)
		PreOrderImpl(root->right, nodes);
}
template<typename T>
inline void Tree<T>::PostOrderImpl(Node<T>* root, Nodes<T>& nodes)
{
	if (root->left)
		PostOrderImpl(root->left, nodes);
	if (root->right)
		PostOrderImpl(root->right, nodes);
	nodes.emplace_back(root);
}
#endif 
