/**
 * Copyright (c) 2020
 * umlaut Software Development and contributors
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ARA_CORE_VARIANT_H_
#define ARA_CORE_VARIANT_H_

#include "type_traits.h"  //
#include "utility.h"      //in_place_t
#include <variant>        //std::variant

namespace ara::core {

// helpers - custom type traits
namespace {

}  // namespace

// forward declaration
template<typename... Alternatives> class Variant;

// 20.7.4[C++ Standard], Variant helper classes
template<class T> struct variant_size;
template<class T> struct variant_size<const T> : variant_size<T>
{};
template<class T> struct variant_size<volatile T> : variant_size<T>
{};
template<class T> struct variant_size<const volatile T> : variant_size<T>
{};

template<class... Alternatives> struct variant_size<Variant<Alternatives...>>
  : std::integral_constant<std::size_t, sizeof...(Alternatives)>
{};

template<class T> constexpr std::size_t variant_size_v = variant_size<T>::value;

template<std::size_t I, class T> struct variant_alternative;

template<std::size_t I, class Head, class... Tail>
struct variant_alternative<I, Variant<Head, Tail...>>
  : variant_alternative<I - 1, Variant<Tail...>>
{};

template<class Head, class... Tail>
struct variant_alternative<0, Variant<Head, Tail...>>
{
    using type = Head;
};

template<std::size_t I, class T> using variant_alternative_t =
  typename variant_alternative<I, T>::type;

template<std::size_t I, class T> struct variant_alternative<I, const T>
{
    using type = std::add_const_t<variant_alternative_t<I, T>>;
};

template<std::size_t I, class T> struct variant_alternative<I, volatile T>
{
    using type = std::add_volatile_t<variant_alternative_t<I, T>>;
};

template<std::size_t I, class T> struct variant_alternative<I, const volatile T>
{
    using type = std::add_cv_t<variant_alternative_t<I, T>>;
};

inline constexpr std::size_t variant_npos = -1;

template<class T, class... Alternatives> constexpr bool
holds_alternative(const Variant<Alternatives...>& variant) noexcept
{
    static_assert(is_unique_v<T, Alternatives...>, "T must be unique");
    return (variant.index() == element_pos_v<T, Alternatives...>);
}

struct Monostate
{};

constexpr bool operator==(Monostate, Monostate) noexcept
{
    return true;
}
constexpr bool operator!=(Monostate, Monostate) noexcept
{
    return false;
}
constexpr bool operator<(Monostate, Monostate) noexcept
{
    return false;
}
constexpr bool operator>(Monostate, Monostate) noexcept
{
    return false;
}
constexpr bool operator<=(Monostate, Monostate) noexcept
{
    return true;
}
constexpr bool operator>=(Monostate, Monostate) noexcept
{
    return true;
}

/**
 * @brief Representation of a type-safe union
 *
 * Variant holds and manages lifetime of a value. Value can be one of
 * alternatives provided in template variadic arguments Alternatives
 *
 * @req {SWS_CORE_01601}
 *
 */
template<typename... Alternatives> class Variant
{
 private:
    // helpers

    template<class T> static constexpr bool equals_self_v =
      is_same_v<Variant, decay_t<T>>;

    /**
     * Below section is (too?) heavy inspiread by GCC
     * https://github.com/gcc-mirror/gcc/blob/master/libstdc%2B%2B-v3/include/std/variant.
     * For the time while I am writing this code, I can not find better solution
     * to solve it. It has to be corrected and made more unique in the next
     * releases!
     *
     * FIXME: not used yet, there are still issues.
     */
    // Section begin
    template<std::size_t I, class T, class U, typename = void>
    struct CreateImaginaryFoo
    {
        void staticImaginary();
    };

    template<std::size_t I, class T, class U>
    struct CreateImaginaryFoo<I,
                              T,
                              U,
                              conditional_t<is_convertible_v<U, T>, T, void>>
    {
        static std::integral_constant<size_t, I> staticImaginary(U);
    };

    template<class T,
             class Var,
             typename Sequence = std::make_index_sequence<variant_size_v<Var>>>
    struct CreateImaginaries;

    template<class T, class... Ts, std::size_t... Indexes>
    struct CreateImaginaries<T, Variant<Ts...>, std::index_sequence<Indexes...>>
    {
        using CreateImaginaryFoo<Indexes, T, Ts>::staticImaginary...;
    };

    template<class T, class Var> using resolved_type_t =
      decltype(CreateImaginaries<T, Var>::staticImaginary(std::declval<T>()));

    template<class T, class Var, typename = void> struct matching_index
      : std::integral_constant<std::size_t, variant_npos>
    {};

    template<class T, class Var>
    struct matching_index<T, Var, void_t<resolved_type_t<T, Var>>>
      : resolved_type_t<T, Var>
    {};

    template<class T> static constexpr size_t matching_index_v =
      matching_index<T, Variant>::value;
    // Section end


    template<std::size_t I,
             typename = requires_<is_in_range_v<I, Alternatives...>>>
    using T_i = variant_alternative_t<I, Variant>;
    using T_0 = T_i<0>;

    template<class T, typename = requires_<not_<equals_self_v<T>>>>
    using matching_type = T_i<matching_index_v<T>>;


 public:
    static_assert(sizeof...(Alternatives) > 0,
                  "Variant must have at least one alternative");
    static_assert(not_<(is_reference_v<Alternatives> || ...)>,
                  "Variant must have no reference alternative");
    static_assert(not_<(is_void_v<Alternatives> || ...)>,
                  "variant must have no void alternative");

    // Constructors
    constexpr Variant() noexcept(is_nothrow_default_constructible_v<T_0>)
      : _impl()
    {}

    constexpr Variant(const Variant& other) : _impl(other._impl) {}

    constexpr Variant(Variant&& other) noexcept(
      (is_nothrow_move_constructible_v<Alternatives> && ...))
      : _impl(std::forward<WrappedType>(other._impl))
    {}

    // FIXME: Commented out -> overload resultion issues. noexcept can't be specified
    template<class T,
             typename    = requires_<not_<equals_self_v<T>>>,
             typename    = requires_<not_<is_in_place_v<T>>>/*,
             typename Ti = matching_type<T&&>,
             typename    = requires_<is_unique_v<Ti> && is_constructible_v<Ti, T>>,
             size_t I   = element_pos_v<T, Alternatives...>,
             class Type = variant_alternative_t<I, Variant>*/>
    constexpr Variant(T&& t) noexcept/*(is_nothrow_constructible_v<Ti, T>)*/
      : _impl(std::forward<T>(t))
    {}

    template<class T, class... Args>
    constexpr explicit Variant(std::in_place_type_t<T> t, Args&&... args)
      : _impl(t, std::forward<Args>(args)...)
    {}

    template<class T, class U, class... Args>
    constexpr explicit Variant(std::in_place_type_t<T>  t,
                               std::initializer_list<U> il,
                               Args&&... args)
      : _impl(t, il, std::forward<Args>(args)...)
    {}

    template<std::size_t I, class... Args>
    constexpr explicit Variant(std::in_place_index_t<I> i, Args&&... args)
      : _impl(i, std::forward<Args>(args)...)
    {}

    template<std::size_t I, class U, class... Args>
    constexpr explicit Variant(std::in_place_index_t<I> i,
                               std::initializer_list<U> il,
                               Args&&... args)
      : _impl(i, il, std::forward<Args>(args)...)
    {}

    // Destructor
    ~Variant() = default;

    constexpr Variant& operator=(const Variant& rhs)
    {
        _impl = rhs._impl;
        return *this;
    }

    constexpr Variant& operator=(Variant&& rhs) noexcept(
      ((is_nothrow_move_constructible_v<
          Alternatives> && is_nothrow_move_assignable_v<Alternatives>) &&...))
    {
        _impl = std::move(rhs._impl);
        return *this;
    }

    // FIXME: Commented out -> overload resultion issues. noexcept can't be specified
    template<class T,
             typename   = requires_<not_<equals_self_v<T>>>/*,
             size_t I   = element_pos_v<T, Alternatives...>,
             class Type = variant_alternative_t<I, Variant>*/>
    Variant& operator=(T&& t) noexcept/*(
      is_nothrow_assignable_v<Type&, T>&& is_nothrow_constructible_v<Type&, T>)*/
    {
        _impl = _impl.template operator=<T>(std::forward<T>(t));
        return *this;
    }

    // Observers
    constexpr std::size_t index() const noexcept { return _impl.index(); }
    constexpr bool        valueless_by_exception() const noexcept
    {
        return _impl.valueless_by_exception();
    }

    // Modifiers
    template<typename T, class... Args>
    requires_<is_constructible_v<T, Args...> && is_unique_v<T, Args...>, T&>
    emplace(Args&&... args)
    {
        return _impl.template emplace<T, Args...>(std::forward<Args>(args)...);
    }

    template<class T, class U, class... Args>
    requires_<is_constructible_v<T,
                                 std::initializer_list<U>&,
                                 Args...> && is_unique_v<T, Args...>,
              T&>
    emplace(std::initializer_list<U> il, Args&&... args)
    {
        return _impl.template emplace<T, U, Args...>(il,
                                                     std::forward<Args>(
                                                       args)...);
    }

    template<size_t I, class... Args>
    requires_<is_constructible_v<variant_alternative_t<I, Variant>, Args...>,
              variant_alternative_t<I, Variant>&>
    emplace(Args&&... args)
    {
        static_assert(I < sizeof...(Args),
                      "Index must be in range of alternatives number");
        return _impl.template emplace<I>(std::forward<Args>(args)...);
    }

    template<size_t I, class U, class... Args>
    requires_<is_constructible_v<variant_alternative_t<I, Variant>,
                                 std::initializer_list<U>&,
                                 Args...>,
              variant_alternative_t<I, Variant>&>
    emplace(std::initializer_list<U> il, Args&&... args)
    {
        static_assert(I < sizeof...(Args),
                      "Index must be in range of alternatives number");
        return _impl.template emplace<I, U, Args...>(il,
                                                     std::forward<Args>(
                                                       args)...);
    }

    void swap(Variant& rhs) noexcept(
      ((is_nothrow_move_constructible_v<
          Alternatives> && is_nothrow_swappable_v<Alternatives>) &&...))
    {
        _impl.swap(rhs._impl);
    }

 private:
    // wrapped member
    std::variant<Alternatives...> _impl;
    using WrappedType = std::variant<Alternatives...>;
};

/**
 * @brief Exchange values of variants
 *
 * Overload for std::swap function in ara::core namespace for variant
 *
 * @req {SWS_CORE_01696}
 *
 */
template<typename... Alternatives> void
swap(Variant<Alternatives...>& lhs, Variant<Alternatives...>& rhs)
{
    lhs.swap(rhs);
}

template<std::size_t I, class... Alternatives>
constexpr variant_alternative_t<I, Variant<Alternatives...>>&
get(Variant<Alternatives...>& v)
{
    static_assert(I < sizeof...(Alternatives),
                  "Index must be in range of alternatives number");
    return std::get<I>(v);
}

template<std::size_t I, class... Alternatives>
constexpr variant_alternative_t<I, Variant<Alternatives...>>&&
get(Variant<Alternatives...>&& v)
{
    static_assert(I < sizeof...(Alternatives),
                  "Index must be in range of alternatives number");
    return std::get<I>(std::forward<Variant<Alternatives...>>(v));
}

template<std::size_t I, class... Alternatives>
constexpr const variant_alternative_t<I, Variant<Alternatives...>>&
get(Variant<Alternatives...>& v)
{
    static_assert(I < sizeof...(Alternatives),
                  "Index must be in range of alternatives number");
    return std::get<I>(v);
}

template<std::size_t I, class... Alternatives>
constexpr const variant_alternative_t<I, Variant<Alternatives...>>&&
get(Variant<Alternatives...>&& v)
{
    static_assert(I < sizeof...(Alternatives),
                  "Index must be in range of alternatives number");
    return std::get<I>(std::forward<Variant<Alternatives...>>(v));
}

template<class Visitor, class... Variants> constexpr decltype(auto)
visit(Visitor&& vis, Variants&&... vars)
{
    return std::visit<Visitor, Variants...>(std::forward<Visitor>(vis),
                                            std::forward<Variants>(vars)...);
}

template<std::size_t I, class... Alternatives>
constexpr std::add_pointer_t<variant_alternative_t<I, Variant<Alternatives...>>>
get_if(Variant<Alternatives...>* pv) noexcept
{
    static_assert(I < sizeof...(Alternatives),
                  "Index must be in range of alternatives number");
    static_assert(not_<is_void_v<
                    variant_alternative_t<I, Variant<Alternatives...>>>>,
                  "Indexed type can't be void");
    if (pv && pv->index() == I)
    {
        return std::addressof(get<I>(*pv));
    }
    return nullptr;
}

template<std::size_t I, class... Alternatives> constexpr std::add_pointer_t<
  const variant_alternative_t<I, Variant<Alternatives...>>>
get_if(const Variant<Alternatives...>* pv) noexcept
{
    static_assert(I < sizeof...(Alternatives),
                  "Index must be in range of alternatives number");
    static_assert(not_<is_void_v<
                    variant_alternative_t<I, Variant<Alternatives...>>>>,
                  "Indexed type can't be void");
    if (pv && pv->index() == I)
    {
        return std::addressof(get<I>(*pv));
    }
    return nullptr;
}


template<class T, class... Alternatives> constexpr std::add_pointer_t<T>
get_if(Variant<Alternatives...>* pv) noexcept
{
    static_assert(is_unique_v<T, Alternatives...>, "T must be unique");
    static_assert(not_<is_void_v<T>>, "T can't be void");
    return get_if<element_pos_v<T, Alternatives...>>(pv);
}

template<class T, class... Alternatives> constexpr std::add_pointer_t<const T>
get_if(const Variant<Alternatives...>* pv) noexcept
{
    static_assert(is_unique_v<T, Alternatives...>, "T must be unique");
    static_assert(not_<is_void_v<T>>, "T can't be void");
    return get_if<element_pos_v<T, Alternatives...>>(pv);
}

template<class... Alternatives> constexpr bool
operator==(const Variant<Alternatives...>& v, const Variant<Alternatives...>& w)
{
    if (v.index() != w.index())
    {
        return false;
    }
    else if (v.valueless_by_exception())
    {
        return true;
    }
    else
    {
        return get<v.index()>(v) == get<w.index()>(w);
    }
}

template<class... Alternatives> constexpr bool
operator!=(const Variant<Alternatives...>& v, const Variant<Alternatives...>& w)
{
    return ! (v == w);
}

template<class... Alternatives> constexpr bool
operator<(const Variant<Alternatives...>& v, const Variant<Alternatives...>& w)
{
    if (w.valueless_by_exception())
    {
        return false;
    }
    else if (v.valueless_by_exception())
    {
        return true;
    }
    else if (v.index() < w.index())
    {
        return true;
    }
    else if (v.index() > w.index())
    {
        return false;
    }
    else
    {
        return get<v.index()>(v) < get<w.index()>(w);
    }
}

template<class... Alternatives> constexpr bool
operator>(const Variant<Alternatives...>& v, const Variant<Alternatives...>& w)
{
    if (v.valueless_by_exception())
    {
        return false;
    }
    else if (w.valueless_by_exception())
    {
        return true;
    }
    else if (v.index() > w.index())
    {
        return true;
    }
    else if (v.index() < w.index())
    {
        return false;
    }
    else
    {
        return get<v.index()>(v) > get<w.index()>(w);
    }
}

template<class... Alternatives> constexpr bool
operator<=(const Variant<Alternatives...>& v, const Variant<Alternatives...>& w)
{
    if (v.valueless_by_exception())
    {
        return true;
    }
    else if (w.valueless_by_exception())
    {
        return false;
    }
    else if (v.index() < w.index())
    {
        return true;
    }
    else if (v.index() > w.index())
    {
        return false;
    }
    else
    {
        return get<v.index()>(v) <= get<w.index()>(w);
    }
}

template<class... Alternatives> constexpr bool
operator>=(const Variant<Alternatives...>& v, const Variant<Alternatives...>& w)
{
    if (w.valueless_by_exception())
    {
        return true;
    }
    else if (v.valueless_by_exception())
    {
        return false;
    }
    else if (v.index() > w.index())
    {
        return true;
    }
    else if (v.index() < w.index())
    {
        return false;
    }
    else
    {
        return get<v.index()>(v) >= get<w.index()>(w);
    }
}

}  // namespace ara::core

#endif  // ARA_CORE_VARIANT_H_
