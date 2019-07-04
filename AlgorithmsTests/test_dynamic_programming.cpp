#include "gtest/gtest.h"
#include "..\Algorithms\knapsack_problem.h"

TEST(test_dynamic_programming, test_knapsack_max_value)
{
	typedef jas::algo::Item<int, int> item;
	jas::algo::SolveKnapSack<typename item> 
		knapSack({ item(1, 1500, "Guitar"), item(4, 3000, "Stereo"),  item(3, 2000, "Laptop")});
	auto value = knapSack.SolveForWeight(4); //4 lbs
	EXPECT_EQ(value, 3500);
}

