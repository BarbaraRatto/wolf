#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <vector>
#include <assert.h>

namespace dls_controller
{

template <typename T>
inline T secondOrderFilter(T& varOutputSecondFilter , T& varOutputFirstFilter , T const& varNew , T const& gain)
{ 
    varOutputFirstFilter = (1- gain) * varOutputFirstFilter + gain * varNew;
    varOutputSecondFilter = (1 - gain) * varOutputSecondFilter + gain * varOutputFirstFilter;
    return varOutputSecondFilter;
} 

enum leg_id {LF=0,RH,RF,LH};

std::vector<std::string> sortByLegName(const std::vector<std::string>& names)
{
    // Sort the names following this convention:
    assert(names.size() == 4);
    std::string lf="lf"; // 0
    std::string rh="rh"; // 1
    std::string rf="rf"; // 2
    std::string lh="lh"; // 3
    std::vector<std::string> ordered_names(4);
    for(unsigned int i=0;i<names.size();i++)
    {
        if(names[i].find(lf) != std::string::npos)
            ordered_names[leg_id::LF] = names[i]; //LF
        if(names[i].find(rh) != std::string::npos)
            ordered_names[leg_id::RH] = names[i]; //RH
        if(names[i].find(rf) != std::string::npos)
            ordered_names[leg_id::RF] = names[i]; //RF
        if(names[i].find(lh) != std::string::npos)
            ordered_names[leg_id::LH] = names[i]; //LH
    }
    return ordered_names;
}

} // namespace

#endif
