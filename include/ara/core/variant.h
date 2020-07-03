/**
 * Copytight (c) 2020
 * umlaut Software Development and contributors
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ARA_CORE_VARIANT_H_
#define ARA_CORE_VARIANT_H_

#include <type_traits>  //
#include <variant>      //std::variant
namespace ara::core     // is_reference_v
{

namespace {
template<class... Ts> struct TypeList
{};

template<class T, class List> struct type_occurence;

template<class T, class... Ts>
struct type_occurence<T, TypeList<Ts...>> : std::integral_constant<size_t, 0>
{};

template<class T, class Head, class... Tail>
struct type_occurence<T, TypeList<Head, Tail...>>
  : std::integral_constant<size_t,
                           std::is_same_v<T, Head>
                             ? type_occurence<T, TypeList<Tail...>>::value + 1
                             : type_occurence<T, TypeList<Tail...>>::value>
{};

template<class T, class... Tail> constexpr bool type_occurence_v =
  type_occurence<T, TypeList<Tail...>>::value;

template<std::size_t I, class... Tail> struct pack_element;

template<std::size_t I, class Head, class... Tail>
struct pack_element<I, Head, Tail...> : pack_element<I - 1, Tail...>
{};

template<class Head, class... Tail> struct pack_element<0, Head, Tail...>
{
    using type = Head;
};

template<class T, class... Ts> constexpr bool
  is_unique_v = (type_occurence_v<T, Ts...> == 1);

template<class T, class... Alternatives> struct element_pos
  : std::integral_constant<std::size_t, 0>
{};

template<class T, class Head, class... Tail>
struct element_pos<T, Head, Tail...>
  : std::integral_constant<
      size_t,
      std::is_same_v<T, Head> ? 0 : element_pos<Head, Tail...>::value + 1>
{};

template<class Head, class... Tail> constexpr std::size_t element_pos_v =
  element_pos<Head, Tail...>::value;
}  // namespace

// forward declaration
template<typename... Alternatives> class Variant;

// 20.7.4[C++ Standard], Variant helper classes
template<class T> struct Variant_size;
template<class T> struct Variant_size<const T> : Variant_size<T>
{};
template<class T> struct Variant_size<volatile T> : Variant_size<T>
{};
template<class T> struct Variant_size<const volatile T> : Variant_size<T>
{};

template<class... Alternatives> struct Variant_size<Variant<Alternatives...>>
  : std::integral_constant<std::size_t, sizeof...(Alternatives)>
{};

template<class T> constexpr std::size_t Variant_size_v = Variant_size<T>::value;

template<std::size_t I, class T> struct Variant_alternative;

template<std::size_t I, class Head, class... Tail>
struct Variant_alternative<I, Variant<Head, Tail...>>
  : Variant_alternative<I - 1, Variant<Tail...>>
{};

template<class Head, class... Tail>
struct Variant_alternative<0, Variant<Head, Tail...>>
{
    using type = Head;
};

template<std::size_t I, class T> using Variant_alternative_t =
  typename Variant_alternative<I, T>::type;

template<std::size_t I, class T> struct Variant_alternative<I, const T>
{
    using type = std::add_const_t<Variant_alternative_t<I, T>>;
};

template<std::size_t I, class T> struct Variant_alternative<I, volatile T>
{
    using type = std::add_volatile_t<Variant_alternative_t<I, T>>;
};

template<std::size_t I, class T> struct Variant_alternative<I, const volatile T>
{
    using type = std::add_cv_t<Variant_alternative_t<I, T>>;
};

inline constexpr std::size_t variant_npos = -1;

// this can be done like that since std::monostace is just a placeholder
struct Monostate : std::monostate
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

template<class T, class... Alternatives> constexpr bool
holds_alternative(const Variant<Alternatives...>& variant) noexcept
{
    static_assert(is_unique_v<T, Alternatives...>, "T must be unique");
    return (variant.index() == element_pos_v<T, Alternatives...>);
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
    template<std::size_t I> using T_i =
      typename pack_element<I, Alternatives...>::type;
    using T_0 = T_i<0>;

 public:
    static_assert(sizeof...(Alternatives) > 0,
                  "Variant must have at least one alternative");
    static_assert(is_unique_v<Alternatives...>,
                  "All alternatives must be unique");
    static_assert(! (std::is_reference_v<Alternatives> || ...),
                  "Variant must have no reference alternative");
    static_assert(! (std::is_void_v<Alternatives> || ...),
                  "variant must have no void alternative");

    // Constructors
    constexpr Variant() noexcept(std::is_nothrow_default_constructible_v<T_0>)
      : _impl()
    {}

    constexpr Variant(const Variant& other) : _impl(other) {}

    constexpr Variant(Variant&& other) noexcept(
      (std::is_nothrow_move_constructible_v<Alternatives> && ...))
      : _impl(other)
    {}

    template<class T> constexpr Variant(T&& t) noexcept
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
      std::is_constructible_v<Variant_alternative_t<I, Variant>, Args...>,
      Variant_alternative_t<I, Variant>&>::type
    emplace(Args&&... args)
    {
        static_assert(I < sizeof...(Args),
                      "Index must be in range of alternatives number");
        return _impl.template emplace<I, Args...>(std::forward<Args>(args)...);
    }

    template<size_t I, class U, class... Args> typename std::enable_if<
      std::is_constructible_v<Variant_alternative_t<I, Variant>,
                              std::initializer_list<U>&,
                              Args...>,
      Variant_alternative_t<I, Variant>&>::type
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
constexpr Variant_alternative_t<I, Variant<Alternatives...>>&
get(Variant<Alternatives...>& v)
{
    static_assert(I < sizeof...(Alternatives),
                  "Index must be in range of alternatives number");
    return std::get<I>(v);
}

template<std::size_t I, class... Alternatives>
constexpr Variant_alternative_t<I, Variant<Alternatives...>>&&
get(Variant<Alternatives...>&& v)
{
    static_assert(I < sizeof...(Alternatives),
                  "Index must be in range of alternatives number");
    return std::get<I>(std::forward<Variant<Alternatives...>>(v));
}

template<std::size_t I, class... Alternatives>
constexpr const Variant_alternative_t<I, Variant<Alternatives...>>&
get(Variant<Alternatives...>& v)
{
    static_assert(I < sizeof...(Alternatives),
                  "Index must be in range of alternatives number");
    return std::get<I>(v);
}

template<std::size_t I, class... Alternatives>
constexpr const Variant_alternative_t<I, Variant<Alternatives...>>&&
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
constexpr std::add_pointer_t<Variant_alternative_t<I, Variant<Alternatives...>>>
get_if(Variant<Alternatives...>* pv) noexcept
{
    static_assert(I < sizeof...(Alternatives),
                  "Index must be in range of alternatives number");
    static_assert(! std::is_void_v<
                    Variant_alternative_t<I, Variant<Alternatives...>>>,
                  "Indexed type can't be void");
    if (pv && pv->index() == I)
    {
        return std::addressof(get<I>(*pv));
    }
    return nullptr;
}

template<std::size_t I, class... Alternatives> constexpr std::add_pointer_t<
  const Variant_alternative_t<I, Variant<Alternatives...>>>
get_if(const Variant<Alternatives...>* pv) noexcept
{
    static_assert(I < sizeof...(Alternatives),
                  "Index must be in range of alternatives number");
    static_assert(! std::is_void_v<
                    Variant_alternative_t<I, Variant<Alternatives...>>>,
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
operator==(const std::variant<Alternatives...>& v,
           const std::variant<Alternatives...>& w)
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
operator!=(const std::variant<Alternatives...>& v,
           const std::variant<Alternatives...>& w)
{
    return ! (v == w);
}

template<class... Alternatives> constexpr bool
operator<(const std::variant<Alternatives...>& v,
          const std::variant<Alternatives...>& w)
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
operator>(const std::variant<Alternatives...>& v,
          const std::variant<Alternatives...>& w)
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
operator<=(const std::variant<Alternatives...>& v,
           const std::variant<Alternatives...>& w)
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
operator>=(const std::variant<Alternatives...>& v,
           const std::variant<Alternatives...>& w)
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
