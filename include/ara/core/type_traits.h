/**
 * Copyright (c) 2020
 * umlaut Software Development and contributors
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ARA_CORE_TYPE_TRAITS_H_
#define ARA_CORE_TYPE_TRAITS_H_

#include <type_traits>
namespace ara::core {

template<bool Condition> static constexpr bool not_ = ! Condition;

template<bool B, class T = void> using requires_ =
  typename std::enable_if<B, T>::type;

// Aliases on STL types, which are not avaiable in C++11
// Copy of those aliases from C++14/17/20
template<class T, class U> constexpr bool is_same_v = std::is_same<T, U>::value;

template<class T> using decay_t = typename std::decay<T>::type;

template<class T> inline constexpr bool is_default_constructible_v =
  std::is_default_constructible<T>::value;

template<class T> inline constexpr bool is_trivially_default_constructible_v =
  std::is_trivially_default_constructible<T>::value;

template<class T> inline constexpr bool is_nothrow_default_constructible_v =
  std::is_nothrow_default_constructible<T>::value;

template<class T> inline constexpr bool is_move_constructible_v =
  std::is_move_constructible<T>::value;

template<class T> inline constexpr bool is_trivially_move_constructible_v =
  std::is_trivially_move_constructible<T>::value;

template<class T> inline constexpr bool is_nothrow_move_constructible_v =
  std::is_nothrow_move_constructible<T>::value;

template<class T> inline constexpr bool is_move_assignable_v =
  std::is_move_assignable<T>::value;

template<class T> inline constexpr bool is_trivially_move_assignable_v =
  std::is_trivially_move_assignable<T>::value;

template<class T> inline constexpr bool is_nothrow_move_assignable_v =
  std::is_nothrow_move_assignable<T>::value;

template<class T, class U> inline constexpr bool is_assignable_v =
  std::is_assignable<T, U>::value;

template<class T, class U> inline constexpr bool is_trivially_assignable_v =
  std::is_trivially_assignable<T, U>::value;

template<class T, class U> inline constexpr bool is_nothrow_assignable_v =
  std::is_nothrow_assignable<T, U>::value;

template<class T, class... Args> inline constexpr bool is_constructible_v =
  std::is_constructible<T, Args...>::value;

template<class T, class... Args>
inline constexpr bool is_trivially_constructible_v =
  std::is_trivially_constructible<T, Args...>::value;

template<class T, class... Args>
inline constexpr bool is_nothrow_constructible_v =
  std::is_nothrow_constructible<T, Args...>::value;

template<class T, class U> inline constexpr bool is_swappable_with_v =
  std::is_swappable_with<T, U>::value;

template<class T> inline constexpr bool is_swappable_v =
  std::is_swappable<T>::value;

template<class T, class U> inline constexpr bool is_nothrow_swappable_with_v =
  std::is_nothrow_swappable_with<T, U>::value;

template<class T> inline constexpr bool is_nothrow_swappable_v =
  std::is_nothrow_swappable<T>::value;

template<class T> inline constexpr bool is_void_v = std::is_void<T>::value;

template<class T> inline constexpr bool is_reference_v =
  std::is_reference<T>::value;

template<bool B, class T, class F> using conditional_t =
  typename std::conditional<B, T, F>::type;

template<class From, class To> inline constexpr bool is_convertible_v =
  std::is_convertible<From, To>::value;

template<class...> using void_t = void;

template<class T> using remove_reference_t =
  typename std::remove_reference<T>::type;

template<class T> struct remove_cvref
{
    using type = std::remove_cv_t<remove_reference_t<T>>;
};

template<class T> using remove_cvref_t = typename std::remove_cvref<T>::type;


/**
 * Find Index of type in list of types.
 *
 * Type trait to return Index of type T which is inside of list of types
 * Alternatives
 *
 * @tparam T Type of which index is searched
 * @tparam Ts collection of types to check
 * @return Index of first found T in Alternatives
 * @return sizeof...(Alternatives) otherwise
 */
template<class T, class... Ts> struct element_pos
  : std::integral_constant<std::size_t, 0>
{};

/**
 * specialization used for metaprogramming - recursively.
 *
 * Template traverse all types recursively, stops if T is found
 *
 **/
template<class T, class Head, class... Tail>
struct element_pos<T, Head, Tail...>
  : std::integral_constant<
      size_t,
      is_same_v<T, Head> ? 0 : element_pos<T, Tail...>::value + 1>
{};

/**
 * Helper constexpr value for element_pos.
 *
 * @tparam T which index if searched for
 * @tparam Ts collection of types to check
 * @return value size_t constexpr index of T
 * @return sizeof...(Alternatives) otherwise
 *
 */
template<class T, class... Ts> constexpr std::size_t element_pos_v =
  element_pos<T, Ts...>::value;

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

template<template<class...> class Condition, class T, class Types> struct type_occurrence;  // undefined case

/**
 * Count type occurrence in list of types
 *
 * Type trait to count number of occurrence for given T type in types TS
 *
 * @tparam T type which occurrence is counted
 * @tparam Ts collection of types to check
 *
 */
template<template<class...> class Condition, class T, class... Ts>
struct type_occurrence<Condition, T, TypeList<Ts...>>
  : std::integral_constant<size_t, 0>
{};

/**
 * specialization used for metaprogramming - recursively.
 *
 * Template traverse all given types and if given T is the same as current
 * visited Head constexpr counter value is increased.
 *
 **/
template<template<class...> class Condition, class T, class Head, class... Tail>
struct type_occurrence<Condition, T, TypeList<Head, Tail...>>
  : std::integral_constant<
      size_t,
      Condition<T, Head>::value
        ? type_occurrence<Condition, T, TypeList<Tail...>>::value + 1
        : type_occurrence<Condition, T, TypeList<Tail...>>::value>
{};

/**
 * Helper constexpr value for type_occurrence.
 *
 * @tparam T type which occupance is counted
 * @tparam Ts collection of types to check
 * @return value size_t constexpr number of occurrence
 *
 */
template<template<class...> class Condition, class T, class... Ts>
constexpr std::size_t type_occurrence_v =
  type_occurrence<Condition, T, TypeList<Ts...>>::value;

template<bool B, template<class...> class Condition, class T, class... Ts>
struct find_matching_type;

template<class T, template<class...> class Condition, class Head, class... Tail>
struct find_matching_type<false, Condition, T, Head, Tail...>
  : std::conditional_t<Condition<T, Head>::value,
                       find_matching_type<true, Condition, T, Head, Tail...>,
                       find_matching_type<false, Condition, T, Tail...>>
{};

template<class T, template<class...> class Condition, class Head, class... Tail>
struct find_matching_type<true, Condition, T, Head, Tail...>
{
    using type = Head;
};

template<template<class...> class Condition, class T, class... Ts>
using find_matching_type_t =
  find_matching_type<false, Condition, T, Ts...>::type;

template<template<class...> class Condition, class T, class... Ts>
  constexpr std::size_t find_matching_index_v = element_pos_v<find_matching_type_t<Condition, T, Ts...>, Ts...>;

/**
 * Checks if type is unique in list of types
 *
 * contant expression checking if given T type is unique in list of types Ts
 *
 * @tparam I index of searched type
 * @tparam Ts collection of types to check
 * @return true if type is unique
 * @return false otherwise
 */
template<class T, class... Ts> constexpr bool
  is_unique_v = (type_occurrence_v<std::is_same, T, Ts...> == 1);

template<class T, class... Ts> constexpr bool
  is_convertible_unique_v = (type_occurrence_v<std::is_convertible, T, Ts...> == 1);

template<class T, class... Ts> constexpr bool
  is_any_convertible_v = (type_occurrence_v<std::is_convertible, T, Ts...> > 0);
/**
 * Find type in list of types by index
 *
 * Type trait to return type which is inside of list of types Tail, on specified
 * index I
 *
 * @tparam I index of searched type
 * @tparam Ts collection of types to check
 */
template<std::size_t I, class... Ts> struct pack_element;  // undefined case

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


template<std::size_t I, class... Ts> static constexpr bool
  is_in_range_v = (I < sizeof...(Ts));

template<typename T> struct is_in_place : std::false_type
{};

template<typename T>
struct is_in_place<std::in_place_type_t<T>> : std::true_type
{};

template<std::size_t I>
struct is_in_place<std::in_place_index_t<I>> : std::true_type
{};

template<typename T> static constexpr bool is_in_place_v =
  is_in_place<remove_cvref_t<T>>::value;


}  // namespace ara::core

#endif  // ARA_CORE_TYPE_TRAITS_H_