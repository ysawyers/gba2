#include <type_traits>


namespace common
{
    template <typename T>
    concept Word_c =
        std::is_same_v<std::uint8_t, T> ||
        std::is_same_v<std::uint16_t, T> ||
        std::is_same_v<std::uint32_t, T>;
}