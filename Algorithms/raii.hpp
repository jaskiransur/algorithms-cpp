#ifndef raii_hpp
#define raii_hpp
#define TEMPLATE template<typename acquirer_type, typename releaser_type, typename resource_type, typename... Args>
#define SCOPE std::raii<typename acquirer_type, typename releaser_type, typename resource_type, typename... Args>
TEMPLATE
SCOPE::raii(resource_type& resource, Args&& ...args)
	:resource_(resource)
{
	acquirer_type creator;
	creator(resource, std::forward<Args>(args)...);
}
TEMPLATE
SCOPE::raii(resource_type* resource, Args&& ...args)
	:resource_(resource)
{
	acquirer_type creator;
	creator(resource, std::forward<Args>(args)...);
}
TEMPLATE
SCOPE::raii(resource_type&& resource, Args&& ...args)
	:resource_(std::forward<resource_type>(resource))
{
	acquirer_type creator;
	creator(resource, std::forward<Args>(args)...);
}
TEMPLATE
SCOPE::raii(Args&& ...args)
{
	acquirer_type creator;
	resource_ = creator(std::forward<Args>(args)...);
}
TEMPLATE
SCOPE::raii(raii&& other)
	:resource_(std::move(other.resource_))
{
}
TEMPLATE
raii& SCOPE::operator=(raii&& other)
{
	resource_ = std::move(other.resource_);
	return *this
}
TEMPLATE
SCOPE::~raii()
{
	releaser_type destroyer;
	destroyer(resource_);
}
TEMPLATE
const resource_type& SCOPE::get() const
{
	return resource_;
}
TEMPLATE
resource_type& SCOPE::get()
{
	return resource_;
}
#undef TEMPLATE
#undef SCOPE
#endif //raii_hpp