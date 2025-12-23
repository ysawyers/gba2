#pragma once

#include <utility>


namespace common
{
    namespace __internal
    {
        /*!
            ...
        */
        template <typename F, std::integral T, T... Is>
        consteval void staticForImpl(F&& func, std::integer_sequence<T, Is...>)
        {
            (func(std::integral_constant<T, Is>{}), ...);
        }
    }

    /*!
        \brief compile-time for loop with templated index

        \tparam T integral type
        \tparam N number of iterations
        \tparam F func(std::integral_constant<T, I>) where I is the current index

        \param[in] func callable
    */
    template <std::integral T, T N, typename F>
    consteval void staticFor(F&& func)
    {
        __internal::staticForImpl(
            std::forward<F>(func),
            std::make_integer_sequence<T, N>{}
        );
    }
}
