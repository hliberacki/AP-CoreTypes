/**
 * Copytight (c) 2020
 * umlaut Software Development and contributors
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ARA_CORE_VARIANT_H_
#define ARA_CORE_VARIANT_H_

#include <variant> //std::variant
#include <type_traits> //
namespace ara::core
{

/**
* @brief Representation of a type-safe union
*
* Variant holds and manages lifetime of a value. Value can be one of alternatives
* provided in template variadic arguments Alternatives
*
* @req {SWS_CORE_01601}
*
*/
template <typename... Alternatives>
class Variant
{
public:
    static_assert(sizeof...(Alternatives) > 0,
                  "Variant must have at least one alternative");
    static_assert(exactly_once_v<Alternatives...>,
                  "All alternatives must be unique");
    static_assert(!(std::is_reference_v<Alternatives> || ...),
		              "Variant must have no reference alternative");
    static_assert(!(std::is_void_v<Alternatives> || ...),
		              "variant must have no void alternative");

    constexpr Variant() noexcept(std::is_nothrow_default_constructible_v<T_0>)
      : _impl() {}

    constexpr Variant(const Variant& other) : _impl(other) {}

    constexpr Variant(Variant&& other) noexcept((std::is_nothrow_move_constructible_v<Alternatives> && ...))
     : _impl(other) {}

    template <class T>
    constexpr Variant( T&& t ) noexcept : _impl(std::forward<T>(t)) {}

    template <class T, class... Args>
    constexpr explicit Variant( std::in_place_type_t<T> t, Args&&... args )
      : _impl(t, std::forward<Args>(args)...) {}

    template <class T, class U, class... Args>
    constexpr explicit Variant( std::in_place_type_t<T> t,
                                std::initializer_list<U> il, Args&&... args )
      : _impl(t, il, std::forward<Args>(args)...) {}

    template <std::size_t I, class... Args>
    constexpr explicit Variant( std::in_place_index_t<I> i, Args&&... args )
      : _impl(i, std::forward<Args>(args)...) {}

    template< std::size_t I, class U, class... Args >
    constexpr explicit Variant( std::in_place_index_t<I> i,
                                std::initializer_list<U> il, Args&&... args )
      : _impl(i, il, std::forward<Args>(args)...) {}

private:
  //helpers
  template<size_t I, class... Tail>
  struct pack_element;

  template<size_t I, class Head, class... Tail>
  struct pack_element<I, Head, Tail...> : pack_element<I-1, Tail...> {};

  template<class Head, class... Tail>
  struct pack_element<0, Head, Tail...>
  {
    using type = Head;
  };

  template<class I>
  using T_i = pack_element<I, Alternatives...>::type;
  using T_0 = T_i<0>;

  template<class... Tail>
  struct exactly_once;

  template<class T, class U, class... Tail>
  struct exactly_once<T, U, Tail...> : exactly_once<U, Tail...> {};

  template<class T, class... Tail>
  struct exactly_once<T, T, Tail...> : std::false_type {};

  template<class T>
  struct exactly_once<T> : std::true_type {};

  template<class... Ts>
  using exactly_once_v = exactly_once<Ts...>::value;

private:
  //wrapped member
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
template <typename... Alternatives>
void swap(Variant<Alternatives...>& lhs, Variant<Alternatives...>& rhs)
{
  return std::swap(lhs, rhs);
}


} //namespace ara::core

#endif //ARA_CORE_VARIANT_H_