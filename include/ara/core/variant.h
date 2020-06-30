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
template<std::size_t I, class... Tail> struct pack_element;

template<std::size_t I, class Head, class... Tail>
struct pack_element<I, Head, Tail...> : pack_element<I - 1, Tail...>
{};

template<class Head, class... Tail> struct pack_element<0, Head, Tail...>
{
    using type = Head;
};

template<class... Tail> struct exactly_once;

template<class T, class U, class... Tail>
struct exactly_once<T, U, Tail...> : exactly_once<U, Tail...>
{};

template<class T, class... Tail>
struct exactly_once<T, T, Tail...> : std::false_type
{};

template<class T> struct exactly_once<T> : std::true_type
{};

template<class... Ts> constexpr bool exactly_once_v =
  exactly_once<Ts...>::value;

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
    static_assert(exactly_once_v<Alternatives...>,
                  "All alternatives must be unique");
    static_assert(! (std::is_reference_v<Alternatives> || ...),
                  "Variant must have no reference alternative");
    static_assert(! (std::is_void_v<Alternatives> || ...),
                  "variant must have no void alternative");

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
    return swap(lhs, rhs);
}

}  // namespace ara::core

#endif  // ARA_CORE_VARIANT_H_