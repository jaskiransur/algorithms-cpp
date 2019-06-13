// Algorithms.cpp : Defines the exported functions for the DLL application.
//
#ifndef jas_algo_symbol_table_hpp
#define jas_algo_symbol_table_hpp

#include <vector>

template<class key_type, class value_type>
struct item
{
	key_type key;
	value_type value;
};

template<class key_type, class value_type>
using item_type = item;

template<class key_type, class value_type>
struct node
{
	item_type<key_type, value_type> item;
	node* left; //smaller or equal
	node* right; //greater or equal
};

template<class item, class value_type>
class SymbolTable
{
public:
	Search()
private:
	node* root_;
};
#endif jas_algo_symbol_table_hpp
