#include "graph.hpp"
#include "knapsack_problem.h"

template jas::algo::Graph<char>;
template jas::algo::Graph<char, true>;
template jas::algo::Item<int, int>;
typedef jas::algo::Item<int, int> Item1;
template jas::algo::SolveKnapSack<Item1>;