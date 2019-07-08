#ifndef jas_algo_bubblesort_hpp
#define jas_algo_bubblesort_hpp
namespace jas { namespace algo { namespace sort{
namespace detail {
template<typename T>
void BubbleSortImpl(T& container, size_t length, size_t pass) 
{
	auto passes = std::min(length-1, pass);
	for (size_t i=0;i<passes; i++)
	{
		bool isAlreadySorted = true;
		for (size_t j=0; j<length-1-i; j++)
		{
			if (container[j] > container[j + 1])
			{
				isAlreadySorted = false;
				std::swap(container[j], container[j + 1]);
			}
		}
		//adaptive: 
		//if it is already sorted then break, there is no need to sort it again
		if (isAlreadySorted)
			break;
	}
}
}

template<typename T>
void BubbleSort(T& t, size_t length) {
	detail::BubbleSortImpl<T>(t, length, length);
}

//time complexity for this is O(n)//better than merge sort's O(nlogn)
template<typename T>
auto MaxElement(T t, size_t length) {
	detail::BubbleSortImpl<T>(t, length, 1);
	//max element is moved to the end
	return t[length - 1];
}
}}}

#endif