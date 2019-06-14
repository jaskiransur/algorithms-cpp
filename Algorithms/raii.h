#ifndef raii_h
#define raii_h
namespace std
{
	template<typename acquirer_type,
		typename releaser_type,
		typename resource_type,
		typename... Args>
		class raii
	{
	public:
		/*! takes a resource_type and multiple number of arguments (0 or more) arguments
		   precondition: The resource_type is already constructor, and the creator merely applies a predicate on it.
		The predicate may take an argument or none at all.
		*/
		raii(resource_type& resource, Args&& ...args);
		/*! woraround for a tempResource&
		takes a resource and multiple number of arguments (0 or more) arguments
		precondition: The resource is already constructed, and the creator merely applies a predicate on it.
		The predicate may take an argument or none at all.
		*/
		raii(resource_type* resource, Args&& ...args);
		/*! takes a resource and multiple number of arguments (0 or more) arguments
		   precondition: The resource is already constructor, and the creator merely applies a predicate on it.
		   The predicate may take an argument or none at all.
		*/
		raii(resource_type&& resource, Args&& ...args);
		/*! creates a resource with default argument
		   or with more than one argument
		   precondition: The resource must have a default constructor implemented
		   It may have more than constructor taking ...args (variable arguments)
		*/
		raii(Args&& ...args);
		raii(const raii& other) = delete;
		raii& operator=(raii& other) = delete;
		/*! move constructor
		*/
		raii(raii&& other);
		/*! move assignement operator
		*/
		raii& operator=(raii&& other);
		/*! destructor, which also calls the predicate to cleanup
		*   the resource
		*/
		~raii();
		/*! returns a const reference to resource
		*/
		const resource_type& get() const
		{
			return resource_;
		}
		/*! returns a reference to resource
		*/
		resource_type& get()
		{
			return resource_;
		}
	private:
		resource_type resource_;
	};
}
#include "raii.hpp"
#endif // !raii_h