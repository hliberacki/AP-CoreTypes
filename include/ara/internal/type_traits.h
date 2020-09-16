/**
 * Copyright (c) 2020
 * umlaut Software Development and contributors
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ARA_INTERNAL_TYPE_TRAITS_H_
#define ARA_INTERNAL_TYPE_TRAITS_H_

#include <cstddef>  //std::size_t
#include <type_traits>

#include "ara/core/utility.h"  //in_place{type|index}_t
namespace ara::internal {

// Aliases on STL types, which are not available in C++11
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
 * Negation for given constexpr boolean expression - Condition.
 *
 * Negates given boolean expression.
 *
 * @tparam Condition constexpr boolean expression.
 *
 */
template<bool Condition> static constexpr bool not_ = ! Condition;

/**
 * Concept-friendly syntax for enable_if_t.
 *
 * Acts as enable_if_t
 *
 * @tparam B constexpr boolean expression.
 * @tparam T type returned is B is true
 *
 */
template<bool B, class T = void> using requires_ =
  typename std::enable_if<B, T>::type;

/**
 * Placeholder for variadic types.
 *
 * Empty helper struct which represents place holder for variadic types.
 *
 * @tparam Ts variadic types.
 *
 */
template<class... Ts> struct TypeList
{};

/**
 * Find Index of type in list of types.
 *
 * Type trait to return Index of type T which is inside of list of types
 * Alternatives.
 *
 * @tparam T Type of which index is searched.
 * @tparam Ts collection of types to check.
 * @return Index of first found T in Alternatives.
 * @return sizeof...(Alternatives) otherwise.
 */
template<class T, class... Ts> struct element_pos
  : std::integral_constant<std::size_t, 0>
{};

/**
 * specialization used for metaprogramming - recursively.
 *
 * Template traverse all types recursively, stops if T is found.
 *
 **/
template<class T, class Head, class... Tail>
struct element_pos<T, Head, Tail...>
  : std::integral_constant<
      std::size_t,
      is_same_v<T, Head> ? 0 : element_pos<T, Tail...>::value + 1>
{};

/**
 * Helper constexpr value for element_pos.
 *
 * @tparam T which index if searched for.
 * @tparam Ts collection of types to check.
 * @return value size_t constexpr index of T.
 * @return sizeof...(Alternatives) otherwise.
 *
 */
template<class T, class... Ts> constexpr std::size_t element_pos_v =
  element_pos<T, Ts...>::value;

template<template<class...> class Condition, class T, class Types>
struct type_occurrence;  // undefined case

/**
 * Count type occurrence in list of types. Condition determines on which
 * condition type occurres.
 *
 * Type trait to count number of occurrence for given T type in types TS.
 *
 * @tparam Condition predicate on which type is compared e.g. is_same.
 * @tparam T type which occurrence is counted.
 * @tparam Ts collection of types to check.
 *
 */
template<template<class...> class Condition, class T, class... Ts>
struct type_occurrence<Condition, T, TypeList<Ts...>>
  : std::integral_constant<std::size_t, 0>
{};

/**
 * specialization used for metaprogramming - recursively.
 *
 * Template traverse all given types and if given T and Head meets Condition
 * then constexpr counter value is increased.
 *
 **/
template<template<class...> class Condition, class T, class Head, class... Tail>
struct type_occurrence<Condition, T, TypeList<Head, Tail...>>
  : std::integral_constant<
      std::size_t,
      Condition<T, Head>::value
        ? type_occurrence<Condition, T, TypeList<Tail...>>::value + 1
        : type_occurrence<Condition, T, TypeList<Tail...>>::value>
{};

/**
 * Helper constexpr value for type_occurrence.
 *
 * @tparam Condition predicate on which type is compared e.g. is_same
 * @tparam T type which occupance is counted
 * @tparam Ts collection of types to check
 * @return value size_t constexpr number of occurrence
 *
 */
template<template<class...> class Condition, class T, class... Ts>
constexpr std::size_t type_occurrence_v =
  type_occurrence<Condition, T, TypeList<Ts...>>::value;

/**
 * Finds first type in list of Types, which meets given boolean Condition.
 *
 * Traverse given types Ts, and check Condition for T and current Head,
 * if Condition is true then given Head is returned as matching type.
 * Program is ill-formed if type is not found.
 *
 * @tparam Condition predicate on which type is compared e.g. is_same.
 * @tparam T type to be compared against list of types Ts
 * @tparam Ts collection of types to check
 * @return type first found matching type
 *
 */
template<bool B, template<class...> class Condition, class T, class... Ts>
struct find_matching_type;  // undefined case

/**
 * specialization used for metaprogramming - recursively.
 *
 * Template traverse all given types and if given T and Head meets Condition
 *then types derives after positive case, keeps traversing otherwise.
 *
 **/
template<class T, template<class...> class Condition, class Head, class... Tail>
struct find_matching_type<false, Condition, T, Head, Tail...>
  : std::conditional_t<Condition<T, Head>::value,
                       find_matching_type<true, Condition, T, Head, Tail...>,
                       find_matching_type<false, Condition, T, Tail...>>
{};

/**
 * specialization used for metaprogramming - positive case.
 *
 * Condition returned true, therefore type is found.
 *
 **/
template<class T, template<class...> class Condition, class Head, class... Tail>
struct find_matching_type<true, Condition, T, Head, Tail...>
{
    using type = Head;
};

/**
 * Helper constexpr type for find_matching_type_t.
 *
 * @tparam Condition predicate on which type is compared e.g. is_same.
 * @tparam T type to be compared against list of types Ts
 * @tparam Ts collection of types to check
 * @return type first found matching type
 *
 */
template<template<class...> class Condition, class T, class... Ts>
using find_matching_type_t =
  typename find_matching_type<false, Condition, T, Ts...>::type;


/**
 * Finds first type in list of Types, which meets given boolean Condition
 * and returns its index location.
 *
 * Traverse given types Ts, and check Condition for T and current Head,
 * if Condition is true then given Head is returned as matching type.
 * Index is calculated for found matched type.
 * Program is ill-formed if type is not found.
 *
 * @tparam Condition predicate on which type is compared e.g. is_same.
 * @tparam T type to be compared against list of types Ts
 * @tparam Ts collection of types to check
 * @return value Index of found type
 *
 */
template<template<class...> class Condition, class T, class... Ts>
constexpr std::size_t find_matching_index_v =
  element_pos_v<find_matching_type_t<Condition, T, Ts...>, Ts...>;

/**
 * Checks if type is unique in list of types
 *
 * constexpr expression checking if given T type is unique in list of types Ts
 *
 * @tparam I index of searched type
 * @tparam Ts collection of types to check
 * @return true if type is unique
 * @return false otherwise
 */
template<class T, class... Ts> constexpr bool
  is_unique_v = (type_occurrence_v<std::is_same, T, Ts...> == 1);

/**
 * Checks if Index is not greater than size of list of types Ts.
 *
 * constexpr expression checking if given Index is not greater than type list
 * size.
 *
 * @tparam I index.
 * @tparam Ts collection of types.
 * @return true if type is lesser than size of Ts.
 * @return false otherwise.
 */
template<std::size_t I, class... Ts> static constexpr bool
  is_in_range_v = (I < sizeof...(Ts));

/**
 * Checks if provided type is a in_place{type|index}_t from ara:core or std::.
 *
 * @tparam T type used in tag.
 * @return std::true_type if type is {ara::core::|std::}in_place{type|index}_t.
 * @return false otherwise.
 */
template<typename T> struct is_in_place : std::false_type
{};

/**
 * Specialization for std::in_place_type_t.
 */
template<typename T>
struct is_in_place<std::in_place_type_t<T>> : std::true_type
{};

/**
 * Specialization for std::in_place_index_t.
 */
template<std::size_t I>
struct is_in_place<std::in_place_index_t<I>> : std::true_type
{};

/**
 * Specialization for ara::core::in_place_type_t.
 */
template<typename T>
struct is_in_place<ara::core::in_place_type_t<T>> : std::true_type
{};

/**
 * Specialization for ara::core::in_place_index_t.
 */
template<std::size_t I>
struct is_in_place<ara::core::in_place_index_t<I>> : std::true_type
{};

/**
 * Helper constexpr value if type is a {ara::core::|std::}in_place{type|index}_t.
 *
 * @tparam T type used in tag.
 * @return value returns true if tag is a
 * {ara::core::|std::}in_place{type|index}_t.
 * @return false otherwise.
 *
 */
template<typename T> static constexpr bool is_in_place_v =
  is_in_place<remove_cvref_t<T>>::value;

}  // namespace ara::internal

#endif  // ARA_INTERNAL_TYPE_TRAITS_H_
