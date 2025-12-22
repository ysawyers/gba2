#include <concepts>
#include <vector>


namespace common
{
    /*!
        \brief Generates all permutations of a bitmask given a wildcard mask.

        \param[in] bitmask The base bitmask
        \param[in] wildcard The wildcard mask indicating which bits can vary

        \return A vector containing all permutations of the bitmask.
    */
    template <std::integral T>
    constexpr std::vector<T> generatePermutations(T bitmask, T wildcard) noexcept
    {
        std::vector<T> permutations;

        for (T permutation = 0; permutation <= wildcard; permutation++)
        {
            if ((~wildcard & permutation) == 0)
            {
                permutations.push_back(bitmask | permutation);
            }
        }

        return permutations;
    }
}
