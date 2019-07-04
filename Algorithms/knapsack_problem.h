#ifndef jas_algo_knapsack_problem_hpp
#define jas_algo_knapsack_problem_hpp
#include <algorithm>
#include <initializer_list>
#include <vector>

using namespace std;
namespace jas { namespace algo {
template<typename weight, typename value>
struct Item
{
	Item(weight w, value v, string name)
		:weight_(std::move(w)), value_(std::move(v))
		, name_(name)
	{
	}
	typedef weight weight_type;
	typedef value value_type;
	weight_type weight_;
	value_type value_;
	string name_;
};

template<typename Item>
struct SolveKnapSack
{
	SolveKnapSack(initializer_list<Item> items);
	typename Item::value_type SolveForWeight(typename Item::weight_type maxWeight);
	void SolveForValue(typename Item::value_type maxValue);
	vector<Item> items_;
};
#define TEMPLATE template<typename Item>
#define SCOPE SolveKnapSack<Item>

TEMPLATE 
SCOPE::SolveKnapSack(initializer_list<Item> items)
:items_(std::move(items))
{
}

TEMPLATE 
typename Item::value_type SCOPE::SolveForWeight(typename Item::weight_type maxWeight)
{
	vector<vector<typename Item::value_type>> grid(items_.size());
	for (auto& row: grid)
		row.resize(maxWeight);

//	sort(items_.begin(), items_.end(),
//		[](const Item& item1, const Item& item2) {return item1.value_< item2.value_; });

	auto maxItemValue = items_[items_.size()-1];	
	//rows
	for (int row=0; row<grid.size(); ++row)
	{ 
		//columns
		//J's here are the weights, if j = 0=> weight = j+1
		for (int col = 0; col < grid[row].size(); ++col)
		{
			auto weight = col+1;//current column is also weight
			auto previousRowColValue = row>0? grid[row - 1][col]: 0;
			auto leftoverWeightsValue = 0;
			auto currentRowColValue = previousRowColValue;
			if (row == 0)
				currentRowColValue = items_[row].value_;
			else if (weight >= items_[row].weight_)
			{
				//leftover weight value is there if there was a weight difference between the current weight and item's weight
				if (weight - items_[row].weight_>0)
					leftoverWeightsValue = grid[row - 1][weight - items_[row].weight_];
				//can we also fit the current item, that could be possible 'cause the current weight is more than item's weight
				currentRowColValue = items_[row].value_;
			}
			grid[row][col] = max(previousRowColValue, leftoverWeightsValue + currentRowColValue);
		}
	}
	//last row, last column
	auto lastRow = grid.size() - 1;
	auto lastCol = grid[lastRow].size()-1;

	return grid[lastRow][lastCol];
}

TEMPLATE 
void SCOPE::SolveForValue(typename Item::value_type maxValue)
{

}
#undef TEMPLATE 
#undef SCOPE 
}}
#endif //jas_algo_knapsack_problem_hpp
