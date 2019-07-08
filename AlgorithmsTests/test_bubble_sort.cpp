#include "gtest/gtest.h"
#include "..\Algorithms\BubbleSort.hpp"
#include <vector>

TEST(test_sort, test_bubblesort)
{
	std::vector<int> a{10,3,4,2,8,0,3};
	jas::algo::sort::BubbleSort(a, a.size());

	std::vector<int> expected{0,2,3,3,4,8,10};
	EXPECT_EQ(a, expected);
}

TEST(test_sort, test_max_element_using_bubblesort)
{
	std::vector<int> a{10,3,4,2,8,0,3};
	auto max = jas::algo::sort::MaxElement(a, a.size());

	EXPECT_EQ(10, max);
}