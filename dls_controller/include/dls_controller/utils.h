#ifndef UTILS_H
#define UTILS_H

namespace dls_controller
{

template <typename T>
inline T secondOrderFilter(T& varOutputSecondFilter , T& varOutputFirstFilter , T const& varNew , T const& gain)
{ 
    varOutputFirstFilter = (1- gain) * varOutputFirstFilter + gain * varNew;
    varOutputSecondFilter = (1 - gain) * varOutputSecondFilter + gain * varOutputFirstFilter;
    return varOutputSecondFilter;
} 

} // namespace

#endif
