/**
 * Copyright (c) 2020
 * umlaut Software Development and contributors
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ARA_CORE_VARIANT_H_
#define ARA_CORE_VARIANT_H_

#include "utility.h"    //in_place_t
#include <type_traits>  //
#include <variant>      //std::variant

namespace ara::core  // is_reference_v
{

// helpers - custom type traits
namespace {

/**
 * Placeholder for variadic types.
 *
 * Empty helper struct which represents place holder for variadic types.
 *
 * @tparam Ts variadic types
 *
 */
template<class... Ts> struct TypeList
{};

template<class T, class Types> struct type_occurrence;  // undefined case

/**
 * Count type occurrence in list of types
 *
 * Type trait to count number of occurrence for given T type in types TS
 *
 * @tparam T type which occurrence is counted
 * @tparam Ts collection of types to check
 *
 */
template<class T, class... Ts>
struct type_occurrence<T, TypeList<Ts...>> : std::integral_constant<size_t, 0>
{};

/**
 * specialization used for metaprogramming - recursively.
 *
 * Template traverse all given types and if given T is the same as current
 * visited Head constexpr counter value is increased.
 *
 **/
template<class T, class Head, class... Tail>
struct type_occurrence<T, TypeList<Head, Tail...>>
  : std::integral_constant<size_t,
                           std::is_same_v<T, Head>
                             ? type_occurrence<T, TypeList<Tail...>>::value + 1
                             : type_occurrence<T, TypeList<Tail...>>::value>
{};

/**
 * Helper constexpr value for type_occurrence.
 *
 * @tparam T type which occupance is counted
 * @tparam Ts collection of types to check
 * @return value size_t constexpr number of occurrence
 *
 */
template<class T, class... Tail> constexpr std::size_t type_occurrence_v =
  type_occurrence<T, TypeList<Tail...>>::value;

/**
 * Find type in list of types by index
 *
 * Type trait to return type which is inside of list of types Tail, on specified
 * index I
 *
 * @tparam I index of searched type
 * @tparam Tail collection of types to check
 */
template<std::size_t I, class... Tail> struct pack_element;  // undefined case

/**
 * specialization used for metaprogramming - recursively.
 *
 * Template traverse Index decrementing it recursively
 *
 **/
template<std::size_t I, class Head, class... Tail>
struct pack_element<I, Head, Tail...> : pack_element<I - 1, Tail...>
{};

/**
 * specialization used for metaprogramming - recursively.
 *
 * In case Index is 0, Head is returned as result type
 *
 **/
template<class Head, class... Tail> struct pack_element<0, Head, Tail...>
{
    using type = Head;
};

/**
 * Checks if type is unique in list of types
 *
 * contant expression checking if given T type is unique in list of types Ts
 *
 * @tparam I index of searched type
 * @tparam Tail collection of types to check
 * @return true if type is unique
 * @return false otherwise
 */
template<class T, class... Ts> constexpr bool
  is_unique_v = (type_occurrence_v<T, Ts...> == 1);

/**
 * Find Index of type in list of types.
 *
 * Type trait to return Index of type T which is inside of list of types Alternatives
 *
 * @tparam T Type of which index is searched
 * @tparam Alternatives collection of types to check
 * @return Index of first found T in Alternatives
 * @return sizeof...(Alternatives) otherwise
 */
template<class T, class... Alternatives> struct element_pos
  : std::integral_constant<std::size_t, 0>
{};

/**
 * specialization used for metaprogramming - recursively.
 *
 * Template traverse all types recursively, stops if T is found
 *
 **/
template<class T, class Head, class... Alternatives>
struct element_pos<T, Head, Alternatives...>
  : std::integral_constant<
      size_t,
      std::is_same_v<T, Head> ? 0 : element_pos<T, Alternatives...>::value + 1>
{};

/**
 * Helper constexpr value for element_pos.
 *
 * @tparam T which index if searched for
 * @tparam Alternatives collection of types to check
 * @return value size_t constexpr index of T
 * @return sizeof...(Alternatives) otherwise
 *
 */
template<class T, class... Alternatives> constexpr std::size_t element_pos_v =
  element_pos<T, Alternatives...>::value;

template<std::size_t I, class... Ts>
  static constexpr bool is_in_range_v = (I < sizeof...(Ts));
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
    template<bool Condition>
    static constexpr bool not_= ! Condition;

    template<class T>
    static constexpr bool equals_self_v = std::is_same_v<Variant, std::decay_t<T>>;

    template<std::size_t I,
             typename = std::enable_if_t<is_in_range_v<I, Alternatives...>> >
    using T_i = variant_alternative_t<I, Variant>;
    using T_0 = T_i<0>;

    // template<std::size_t I,
    //          typename = std::enable_if_t<_not<equals_self_v>>>
    // using resolved_type_t =

 public:
    static_assert(sizeof...(Alternatives) > 0,
                  "Variant must have at least one alternative");
    static_assert(! (std::is_reference_v<Alternatives> || ...),
                  "Variant must have no reference alternative");
    static_assert(! (std::is_void_v<Alternatives> || ...),
                  "variant must have no void alternative");

    // Constructors
    constexpr Variant() noexcept(std::is_nothrow_default_constructible_v<T_0>)
      : _impl()
    {}

    constexpr Variant(const Variant& other) : _impl(other._impl) {}

    constexpr Variant(Variant&& other) noexcept(
      (std::is_nothrow_move_constructible_v<Alternatives> && ...))
      : _impl(std::forward<WrappedType>(other._impl))
    {}

    // FIXME: Commented out -> type deduced as const char[] for std::string
    template<class T,
             typename = std::enable_if_t<not_<equals_self_v<T>>>/*,
             size_t I   = element_pos_v<T, Alternatives...>,
             class Type = variant_alternative_t<I, Variant>*/>
    constexpr Variant(T&& t) noexcept/*(std::is_nothrow_constructible_v<Type, T>)*/
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

    constexpr Variant& operator=(Variant&& rhs) noexcept(((
      std::is_nothrow_move_constructible_v<
        Alternatives> && std::is_nothrow_move_assignable_v<Alternatives>) &&...))
    {
        _impl = std::move(rhs._impl);
        return *this;
    }

    template<class T,
             typename = std::enable_if_t<not_<equals_self_v<T>>>,
             size_t I   = element_pos_v<T, Alternatives...>,
             class Type = variant_alternative_t<I, Variant>>
    Variant&
    operator=(T&& t) noexcept(std::is_nothrow_assignable_v<Type&, T>&&
                                std::is_nothrow_constructible_v<Type&, T>)
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
    template<typename T, class... Args> typename std::enable_if<
      std::is_constructible_v<T, Args...> && is_unique_v<T, Args...>,
      T&>::type
    emplace(Args&&... args)
    {
        return _impl.template emplace<T, Args...>(std::forward<Args>(args)...);
    }

    template<class T, class U, class... Args> typename std::enable_if<
      std::is_constructible_v<T,
                              std::initializer_list<U>&,
                              Args...> && is_unique_v<T, Args...>,
      T&>::type
    emplace(std::initializer_list<U> il, Args&&... args)
    {
        return _impl.template emplace<T, U, Args...>(il,
                                                     std::forward<Args>(
                                                       args)...);
    }

    template<size_t I, class... Args> typename std::enable_if<
      std::is_constructible_v<variant_alternative_t<I, Variant>, Args...>,
      variant_alternative_t<I, Variant>&>::type
    emplace(Args&&... args)
    {
        static_assert(I < sizeof...(Args),
                      "Index must be in range of alternatives number");
        return _impl.template emplace<I, Args...>(std::forward<Args>(args)...);
    }

    template<size_t I, class U, class... Args> typename std::enable_if<
      std::is_constructible_v<variant_alternative_t<I, Variant>,
                              std::initializer_list<U>&,
                              Args...>,
      variant_alternative_t<I, Variant>&>::type
    emplace(std::initializer_list<U> il, Args&&... args)
    {
        static_assert(I < sizeof...(Args),
                      "Index must be in range of alternatives number");
        return _impl.template emplace<I, U, Args...>(il,
                                                     std::forward<Args>(
                                                       args)...);
    }

    void swap(Variant& rhs) noexcept(
      ((std::is_nothrow_move_constructible_v<
          Alternatives> && std::is_nothrow_swappable_v<Alternatives>) &&...))
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
    static_assert(! std::is_void_v<
                    variant_alternative_t<I, Variant<Alternatives...>>>,
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
    static_assert(! std::is_void_v<
                    variant_alternative_t<I, Variant<Alternatives...>>>,
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
    static_assert(! std::is_void_v<T>, "T can't be void");
    return get_if<element_pos_v<T, Alternatives...>>(pv);
}

template<class T, class... Alternatives> constexpr std::add_pointer_t<const T>
get_if(const Variant<Alternatives...>* pv) noexcept
{
    static_assert(is_unique_v<T, Alternatives...>, "T must be unique");
    static_assert(! std::is_void_v<T>, "T can't be void");
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
