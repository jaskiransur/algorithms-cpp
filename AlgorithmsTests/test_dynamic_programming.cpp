#include "gtest/gtest.h"
#include "..\Algorithms\knapsack_problem.h"

TEST(test_dynamic_programming, test_knapsack_three_items)
{
	typedef jas::algo::Item<int, int> item;
	jas::algo::SolveKnapSack<typename item> 
		knapSack({ item(1, 1500, "Guitar"), item(4, 3000, "Stereo"),  item(3, 2000, "Laptop")});
	auto value = knapSack.SolveForWeight(4); //4 lbs
	EXPECT_EQ(value, 3500);
}

TEST(test_dynamic_programming, test_knapsack_four_items)
{
	typedef jas::algo::Item<int, int> item;
	jas::algo::SolveKnapSack<typename item>
		knapSack({ item(1, 1500, "Guitar"), item(4, 3000, "Stereo"),  item(3, 2000, "Laptop"), item(1, 2000, "Iphone")});

	auto value = knapSack.SolveForWeight(4); //4 lbs
	EXPECT_EQ(value, 4000);
}

TEST(test_dynamic_programming, test_knapsack_five_items)
{
	typedef jas::algo::Item<int, int> item;
	jas::algo::SolveKnapSack<typename item>
		knapSack1({ item(1, 1500, "Guitar"), item(4, 3000, "Stereo"),
			item(3, 2000, "Laptop"), item(1, 2000, "Iphone")});

	auto value1 = knapSack1.SolveForWeight(4); //4 lbs

	jas::algo::SolveKnapSack<typename item>
		knapSack2({ item(1, 1500, "Guitar"), item(4, 3000, "Stereo"), 
			item(3, 2000, "Laptop"), item(1, 2000, "Iphone"), 
			item(1, 1000, "Mp3")});

	auto value2 = knapSack2.SolveForWeight(4); //4 lbs
	EXPECT_EQ(value2, 4500);
	EXPECT_GT(value2, value1);
}


TEST(test_dynamic_programming, test_knapsack_add_item)
{
	typedef jas::algo::Item<int, int> item;
	jas::algo::SolveKnapSack<typename item> 
		knapSack({ item(1, 1500, "Guitar"), item(4, 3000, "Stereo"),  item(3, 2000, "Laptop")});
	auto value = knapSack.SolveForWeight(4); //4 lbs
	EXPECT_EQ(value, 3500);
	knapSack.AddItem(item(1, 2000, "Iphone"));

	value = knapSack.SolveForWeight(4); //4 lbs
	EXPECT_EQ(value, 4000);
}

