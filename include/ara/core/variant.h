/**
 * Copyright (c) 2020
 * umlaut Software Development and contributors
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ARA_CORE_VARIANT_H_
#define ARA_CORE_VARIANT_H_

#include "ara/core/utility.h"  //ara::core::in_place_{type|index}_t
#include "ara/internal/type_traits.h"

#include <variant>  //std::variant

namespace ara::core {
namespace inter = ara::internal;

// forward declaration
template<typename... Ts> class Variant;


/** @defgroup Helpers
 *  Helpers classes and objects
 *  @{
 */

// 20.7.4[C++ Standard], Variant helper classes
/**
 * @brief Empty variant type
 *
 * @note Unit type intended for use as a well-behaved empty alternative in
 * Variant. In particular, a variant of non-default-constructible types may
 * list Monostate as its first alternative: this makes the variant itself
 * default-constructible. All instances of Monostate compare equal.
 */
struct Monostate
{};

/**
 * @brief Equality operator.
 *
 * @return true, Monostate is always equal.
 */
constexpr bool operator==(Monostate, Monostate) noexcept
{
    return true;
}

/**
 * @brief Inequality operator.
 *
 * @return false, Monostate is always equal.
 */
constexpr bool operator!=(Monostate, Monostate) noexcept
{
    return false;
}

/**
 * @brief Less operator operator.
 *
 * @return false, Monostate is always equal.
 */
constexpr bool operator<(Monostate, Monostate) noexcept
{
    return false;
}

/**
 * @brief Greater operator operator.
 *
 * @return false, Monostate is always equal.
 */
constexpr bool operator>(Monostate, Monostate) noexcept
{
    return false;
}

/**
 * @brief Less-equal operator operator.
 *
 * @return true, Monostate is always equal.
 */
constexpr bool operator<=(Monostate, Monostate) noexcept
{
    return true;
}

/**
 * @brief Greater-equal operator operator.
 *
 * @return true, Monostate is always equal.
 */
constexpr bool operator>=(Monostate, Monostate) noexcept
{
    return true;
}

/**
 * @brief Get the size of the Variant's list of alternatives at compile time.
 *
 * @tparam T Variant on which size is calculated.
 */
template<class T> struct variant_size;  // undefined case

/**
 * @brief Get the size of the Variant's list of alternatives at compile time.
 *
 * @note Specialization for const Variant.
 *
 * @tparam T const Variant on which size is calculated.
 */
template<class T> struct variant_size<const T> : variant_size<T>
{};

/**
 * @brief Get the size of the Variant's list of alternatives at compile time.
 *
 * @note Specialization for volatile Variant.
 *
 * @tparam T volatile Variant on which size is calculated.
 */
template<class T> struct variant_size<volatile T> : variant_size<T>
{};

/**
 * @brief Get the size of the Variant's list of alternatives at compile time.
 *
 * @note Specialization for const volatile Variant.
 *
 * @tparam T const volatile Variant on which size is calculated.
 */
template<class T> struct variant_size<const volatile T> : variant_size<T>
{};


/**
 * @brief Get the size of the Variant's list of alternatives at compile time.
 *
 * @tparam Ts list of types hold by Variant.
 */
template<class... Ts> struct variant_size<Variant<Ts...>>
  : std::integral_constant<std::size_t, sizeof...(Ts)>
{};

/**
 * @brief Helper function to get size of Variant.
 *
 * @tparam T list of types hold by Variant.
 */
template<class T> inline constexpr std::size_t variant_size_v =
  variant_size<T>::value;


/**
 * @brief Provides compile-time indexed access to the types of the alternatives
 * of the possibly cv-qualified variant, combining cv-qualifications of the
 * variant (if any) with the cv-qualifications of the alternative.
 *
 * @pre Index must be in range of Variant size [0; variant_size_v<Variant>)
 * otherwise program is ill-formed.
 *
 * @tparam I index of the type of Ith alternative of the Variant.
 */
template<std::size_t I, class T> struct variant_alternative;  // undefined case

/**
 * @note Specialization used for metaprogramming - recursively.
 *
 * @brief Traverse Alternatives until Index is 0. Each time decrement Index.
 * If Index goes beyond range,  program is ill-formed.
 */

template<std::size_t I, class Head, class... Tail>
struct variant_alternative<I, Variant<Head, Tail...>>
  : variant_alternative<I - 1, Variant<Tail...>>
{
    static_assert(inter::is_in_range_v<I, Head, Tail...>,
                  "Index must be in range of alternatives number");
};


/**
 * @brief Specialization used for metaprogramming - positive case.
 *
 * Index is 0, current Head is resolved as found type.
 */
template<class Head, class... Tail>
struct variant_alternative<0, Variant<Head, Tail...>>
{
    using type = Head;
};

/**
 * @brief Helper function to get type Ith of Variant.
 *
 * @tparam I index of the type of Ith alternative of the Variant.
 */
template<std::size_t I, class T> using variant_alternative_t =
  typename variant_alternative<I, T>::type;


/**
 * @brief Specialization for const Variant.
 */
template<std::size_t I, class T> struct variant_alternative<I, const T>
{
    using type = std::add_const_t<variant_alternative_t<I, T>>;
};

/**
 * @brief Specialization for volatile Variant.
 */
template<std::size_t I, class T> struct variant_alternative<I, volatile T>
{
    using type = std::add_volatile_t<variant_alternative_t<I, T>>;
};

/**
 * @brief Specialization for const volatile Variant.
 */
template<std::size_t I, class T> struct variant_alternative<I, const volatile T>
{
    using type = std::add_cv_t<variant_alternative_t<I, T>>;
};

/**
 * Index of the variant in the invalid state.
 */
inline constexpr std::size_t variant_npos = static_cast<std::size_t>(-1);
/** @} */  // end of Helpers

/** @defgroup Non-member
 *  Non-member functions
 *  @{
 */

/**
 * @brief Checks if the variant holds the alternative T.
 * The call is ill-formed if T does not appear exactly once in Alternatives.
 *
 * @tparam T type which is checked for being held.
 * @tparam Ts List of all Variant types.
 * @return true if the variant currently holds the alternative T.
 * @return false otherwise.
 */
template<class T, class... Ts> constexpr bool
holds_alternative(const Variant<Ts...>& variant) noexcept
{
    static_assert(inter::is_unique_v<T, Ts...>, "T must be unique");
    return (variant.index() == inter::element_pos_v<T, Ts...>);
}

/**
 * @brief Exchange values of variants.
 *
 * @overload Overload for std::swap function in ara::core namespace for variant.
 *
 * @req {SWS_CORE_01696}
 *
 * @tparam Ts types used in Variant
 * @param lhs Left hand side Variant
 * @param rhs Right hand side Variant
 */
template<typename... Ts> void swap(Variant<Ts...>& lhs, Variant<Ts...>& rhs)
{
    lhs.swap(rhs);
}

/**
 * @brief Get Variant value by Index.
 *
 * Reads the value of the variant given the index.
 *
 * @pre Function is enabled if I is in the range of Ts.
 *
 * @tparam I index of accessed type.
 * @tparam Ts Variant types.
 * @param v Variant.
 * @return value currently assigned in Variant.
 */
template<std::size_t I, class... Ts>
constexpr variant_alternative_t<I, Variant<Ts...>>& get(Variant<Ts...>& v)
{
    static_assert(inter::is_in_range_v<I, Ts...>,
                  "Index must be in range of alternatives number");
    return std::get<I>(v._impl);
}

/**
 * @brief Get Variant value by Index.
 *
 * Reads the value of the variant given the index. Using move semantics.
 *
 * @pre Function is enabled if I is in the range of Ts.
 *
 * @tparam I index of accessed type.
 * @tparam Ts Variant types.
 * @param v moved Variant.
 * @return value currently assigned in Variant.
 */
template<std::size_t I, class... Ts>
constexpr variant_alternative_t<I, Variant<Ts...>>&& get(Variant<Ts...>&& v)
{
    static_assert(inter::is_in_range_v<I, Ts...>,
                  "Index must be in range of alternatives number");
    return std::get<I>(std::forward<std::variant<Ts...>>(v._impl));
}

/**
 * @brief Get Variant value by Index.
 *
 * Reads the value of the variant given the index.
 *
 * @pre Function is enabled if I is in the range of Ts.
 *
 * @tparam I index of accessed type.
 * @tparam Ts Variant types.
 * @param v const reference Variant.
 * @return value currently assigned in Variant.
 */
template<std::size_t I, class... Ts>
constexpr const variant_alternative_t<I, Variant<Ts...>>&
get(const Variant<Ts...>& v)
{
    static_assert(inter::is_in_range_v<I, Ts...>,
                  "Index must be in range of alternatives number");
    return std::get<I>(v._impl);
}

/**
 * @brief Get Variant value by Index.
 *
 * Reads the value of the variant given the index. Move semantics.
 *
 * @pre Function is enabled if I is in the range of Ts.
 *
 * @tparam I index of accessed type.
 * @tparam Ts Variant types.
 * @param v moved Variant.
 * @return value currently assigned in Variant.
 */
template<std::size_t I, class... Ts>
constexpr const variant_alternative_t<I, Variant<Ts...>>&&
get(const Variant<Ts...>&& v)
{
    static_assert(inter::is_in_range_v<I, Ts...>,
                  "Index must be in range of alternatives number");
    return std::get<I>(std::forward<std::variant<Ts...>>(v._impl));
}

/**
 * @brief Get Variant value by Type.
 *
 * Reads the value of the variant given the type.
 *
 * @pre Function is enabled if I is in the range of Ts.
 *
 * @tparam T accessed type.
 * @tparam Ts Variant types.
 * @param v Variant.
 * @return value currently assigned in Variant.
 */
template<class T, class... Ts> constexpr T& get(Variant<Ts...>& v)
{
    static_assert(inter::is_unique_v<T, Ts...>, "T must be unique");
    return std::get<T>(v._impl);
}

/**
 * @brief Get Variant value by Type.
 *
 * Reads the value of the variant given the type. Move semantics.
 *
 * @pre Function is enabled if I is in the range of Ts.
 *
 * @tparam T accessed type.
 * @tparam Ts Variant types.
 * @param v moved Variant.
 * @return value currently assigned in Variant.
 */
template<class T, class... Ts> constexpr T&& get(Variant<Ts...>&& v)
{
    static_assert(inter::is_unique_v<T, Ts...>, "T must be unique");
    return std::get<T>(std::forward<std::variant<Ts...>>(v._impl));
}

/**
 * @brief Get Variant value by Type.
 *
 * Reads the value of the variant given the type.
 *
 * @pre Function is enabled if I is in the range of Ts.
 *
 * @tparam T accessed type.
 * @tparam Ts Variant types.
 * @param v const reference Variant.
 * @return value currently assigned in Variant.
 */
template<class T, class... Ts> constexpr const T& get(const Variant<Ts...>& v)
{
    static_assert(inter::is_unique_v<T, Ts...>, "T must be unique");
    return std::get<T>(v._impl);
}

/**
 * @brief Get Variant value by Type.
 *
 * Reads the value of the variant given the type. Move semantics.
 *
 * @pre Function is enabled if I is in the range of Ts.
 *
 * @tparam T accessed type.
 * @tparam Ts Variant types.
 * @param v moved Variant.
 * @return value currently assigned in Variant.
 */
template<class T, class... Ts> constexpr const T&& get(const Variant<Ts...>&& v)
{
    static_assert(inter::is_unique_v<T, Ts...>, "T must be unique");
    return std::get<T>(std::forward<std::variant<Ts...>>(v._impl));
}

/**
 * @brief Applies the visitor to the Variants.
 *
 * @remarks This function is realised by std::visit implementation. Behaviour
 * and constraits meets 23.7.7 requirement from C++17 standard.
 *
 * @tparam Visitor a Callable that accepts every possible alternative from every
 * variant.
 * @tparam Variants list of Variant on which Visitor is applied.
 * @param vis instance of Visitor
 * @param vars list of Variants
 * @return The value returned by the selected invocation of the visitor.
 * @return Nothing if R is void. Otherwise the value returned by the selected
 * invocation of the visitor, implicitly converted to R.
 */
template<class Visitor, class... Variants> constexpr decltype(auto)
visit(Visitor&& vis, Variants&&... vars)
{
    return std::visit(std::forward<Visitor>(vis),
                      std::forward<decltype(vars._impl)>(vars._impl)...);
}

/**
 * @brief Get a pointer to the value of a pointed-to variant given the index.
 *
 * @pre Index must be in range of Alternatives.
 *
 * @tparam I index of type.
 * @tparam Ts types of Variant.
 * @param Pv pointer to Variant or null in case of error.
 */
template<std::size_t I, class... Ts>
constexpr std::add_pointer_t<variant_alternative_t<I, Variant<Ts...>>>
get_if(Variant<Ts...>* pv) noexcept
{
    static_assert(inter::is_in_range_v<I, Ts...>,
                  "Index must be in range of alternatives number");
    static_assert(inter::not_<
                    inter::is_void_v<variant_alternative_t<I, Variant<Ts...>>>>,
                  "Indexed type can't be void");
    if (pv && pv->index() == I)
    {
        return std::addressof(get<I>(*pv));
    }
    return nullptr;
}

/**
 * @brief Get a pointer to the value of a pointed-to const variant given the
 * index.
 *
 * @pre Index must be in range of Ts.
 *
 * @tparam I index of type.
 * @tparam Ts types of Variant.
 * @param Pv pointer to Variant or null in case of error.
 */
template<std::size_t I, class... Ts>
constexpr std::add_pointer_t<const variant_alternative_t<I, Variant<Ts...>>>
get_if(const Variant<Ts...>* pv) noexcept
{
    static_assert(inter::is_in_range_v<I, Ts...>,
                  "Index must be in range of alternatives number");
    static_assert(inter::not_<
                    inter::is_void_v<variant_alternative_t<I, Variant<Ts...>>>>,
                  "Indexed type can't be void");
    if (pv && pv->index() == I)
    {
        return std::addressof(get<I>(*pv));
    }
    return nullptr;
}

/**
 * @brief Get a pointer to the value of a pointed-to variant given the type.
 *
 * @pre Type must be unique in Ts
 *
 * @tparam T unique type to look up.
 * @tparam Ts types of Variant.
 * @param Pv pointer to Variant or null in case of error.
 */
template<class T, class... Ts> constexpr std::add_pointer_t<T>
get_if(Variant<Ts...>* pv) noexcept
{
    static_assert(inter::is_unique_v<T, Ts...>, "T must be unique");
    static_assert(inter::not_<inter::is_void_v<T>>, "T can't be void");
    return get_if<inter::element_pos_v<T, Ts...>>(pv);
}

/**
 * @brief Get a pointer to the value of a pointed-to variant given the const
 * type.
 *
 * @pre Type must be unique in Ts
 *
 * @tparam T const unique type to look up.
 * @tparam Ts types of Variant.
 * @param Pv pointer to Variant or null in case of error.
 */
template<class T, class... Ts> constexpr std::add_pointer_t<const T>
get_if(const Variant<Ts...>* pv) noexcept
{
    static_assert(inter::is_unique_v<T, Ts...>, "T must be unique");
    static_assert(inter::not_<inter::is_void_v<T>>, "T can't be void");
    return get_if<inter::element_pos_v<T, Ts...>>(pv);
}

/**
 * @brief Equality operator for Variant.
 *
 * @remarks This operator is realised by std::variant implementation. Behaviour
 * and constraits meets 23.7.6 requirement from C++17 standard.
 *
 * @param v left hand side Variant.
 * @param w right hand side Variant.
 * @return false if v.index() != w.index.
 * @return true if v is valueless_by_exception or currently held values in v and
 * w are equal.
 */
template<class... Ts> constexpr bool
operator==(const Variant<Ts...>& v, const Variant<Ts...>& w)
{
    return v._impl == w._impl;
}

/**
 * @brief Inequality operator for Variant.
 *
 * @remarks This operator is realised by std::variant implementation. Behaviour
 * and constraits meets 23.7.6 requirement from C++17 standard.
 *
 * @param v left hand side Variant.
 * @param w right hand side Variant.
 * @return true if v.index() != w.index.
 * @return false if v is valueless_by_exception or currently held values in v
 * and w are equal.
 */
template<class... Ts> constexpr bool
operator!=(const Variant<Ts...>& v, const Variant<Ts...>& w)
{
    return ! (v == w);
}

/**
 * @brief Less-than operator for Variant.
 *
 * @remarks This operator is realised by std::variant implementation. Behaviour
 * and constraits meets 23.7.6 requirement from C++17 standard.
 *
 * @param v left hand side Variant.
 * @param w right hand side Variant.
 * @return true if v is value_less_by_exception or index of v is lesser than
 * index of w
 * @return false if w is valueless_by_exception or index of v is greater than
 * index of w
 * @return std::get<v.index()>(v) < std::get<v.index()>(w) otherwise.
 */
template<class... Ts> constexpr bool
operator<(const Variant<Ts...>& v, const Variant<Ts...>& w)
{
    return v._impl < w._impl;
}

/**
 * @brief Greater-than operator for Variant.
 *
 * @remarks This operator is realised by std::variant implementation. Behaviour
 * and constraits meets 23.7.6 requirement from C++17 standard.
 *
 * @param v left hand side Variant.
 * @param w right hand side Variant.
 * @return true if w is value_less_by_exception or index of v is greater than
 * index of w
 * @return false if v is valueless_by_exception or index of v is lesser than
 * index of w
 * @return std::get<v.index()>(v) > std::get<v.index()>(w) otherwise.
 */
template<class... Ts> constexpr bool
operator>(const Variant<Ts...>& v, const Variant<Ts...>& w)
{
    return v._impl > w._impl;
}

/**
 * @brief Less-equal operator for Variant.
 *
 * @remarks This operator is realised by std::variant implementation. Behaviour
 * and constraits meets 23.7.6 requirement from C++17 standard.
 *
 * @param v left hand side Variant.
 * @param w right hand side Variant.
 * @return true if v is value_less_by_exception or index of v is lesser than
 * index of w
 * @return false if w is valueless_by_exception or index of v is greater than
 * index of w
 * @return std::get<v.index()>(v) <= std::get<v.index()>(w) otherwise.
 */
template<class... Ts> constexpr bool
operator<=(const Variant<Ts...>& v, const Variant<Ts...>& w)
{
    return v._impl <= w._impl;
}

/**
 * @brief Greater-equal operator for Variant.
 *
 * @remarks This operator is realised by std::variant implementation. Behaviour
 * and constraits meets 23.7.6 requirement from C++17 standard
 *
 * @param v left hand side Variant.
 * @param w right hand side Variant.
 * @return true if w is value_less_by_exception or index of v is greater than
 * index of w
 * @return false if v is valueless_by_exception or index of v is lesser than
 * index of w
 * @return std::get<v.index()>(v) >= std::get<v.index()>(w) otherwise.
 */
template<class... Ts> constexpr bool
operator>=(const Variant<Ts...>& v, const Variant<Ts...>& w)
{
    return v._impl >= w._impl;
}
/** @} */  // end of Non-member

/**
 * @brief Representation of a type-safe union.
 *
 * Variant holds and manages lifetime of a value. Value can be one of
 * alternatives provided in template variadic arguments Ts.
 *
 * @req {SWS_CORE_01601}.
 * @tparam Ts the types that may be stored in this Variant.
 */
template<typename... Ts> class Variant
{
    /**
     * FIXME: Current implementation with place_holder for core logic replaced
     * by std::variant has to be changed. std::variant shall be removed and
     * implementation shall be provided. Current solution makes it impossible or
     * hardly possible to properly implement some requirements which needs to
     * exclude functions from overload resolution. In future implementation pure
     * SFINAE (inter::requires_) must be replaced by "base class" inheritance
     * using type_traits as conditions.
     *
     * Missing requirements are listed within each function FIXME.
     *
     * Github issue:
     * @link
     * https://github.com/UmlautSoftwareDevelopmentAccount/AP-CoreTypes/issues/62
     */

 private:
    // helpers

    /**
     * @brief Helper checks if given T is the same as Variant - self type.
     *
     * @tparam T type to compare against self.
     */
    template<class T> static constexpr bool equals_self_v =
      inter::is_same_v<Variant, inter::decay_t<T>>;

    /**
     * @brief Helper for accesing type at index 0 of Alternatives.
     */
    using T_0 = variant_alternative_t<0, Variant>;

 public:
    static_assert(sizeof...(Ts) > 0,
                  "Variant must have at least one alternative");
    static_assert(inter::not_<(inter::is_reference_v<Ts> || ...)>,
                  "Variant must have no reference alternative");
    static_assert(inter::not_<(inter::is_void_v<Ts> || ...)>,
                  "Variant must have no void alternative");
    static_assert(inter::not_<(std::is_array<Ts>::value || ...)>,
                  "Variant must have no array alternative");

    /** @defgroup Constructors
     *  @{
     */

    /**
     * @brief Default constructor.
     *
     * Constructs a Variant holding the value-initialized value of the first
     * Alternative.
     */
    template<typename = inter::requires_<inter::is_default_constructible_v<T_0>>>
    constexpr Variant() noexcept(inter::is_nothrow_default_constructible_v<T_0>)
      : _impl()
    {}

    /**
     * FIXME: adding precondition 23.7.3.1.8 breaks the compilation.
     *        problem has to be investigated because inter::requires_<true> is
     * not valid as well. Concepts and requires clause from C++20 works as
     * expected. Final version will use different technique to acheave the goal.
     *
     * @brief Copy constructor.
     *
     * Constructs a Variant holding the same alternative as other.
     *
     * @param other Variant to be copied from.
     */
    constexpr Variant(const Variant& other) : _impl(other._impl) {}

    /**
     * FIXME: adding precondition 23.7.3.1.11 breaks the compilation.
     *        problem has to be investigated because inter::requires_<true> is
     * not valid as well. Concepts and requires clause from C++20 works as
     * expected. Final version will use different technique to acheave the goal.
     *
     * @brief Move constructor.
     *
     * Constructs a Variant holding the same alternative as other.
     *
     * @param other Variant to be moved value from.
     */
    constexpr Variant(Variant&& other) noexcept(
      (inter::is_nothrow_move_constructible_v<Ts> && ...))
      : _impl(std::forward<WrappedType>(other._impl))
    {}

    /**
     * @brief Converting constructor.
     *
     * @remarks Postconditions as in 23.7.3.1.14 are realised by the underlying
     * std::variant implementation from C++17 standard.
     *
     * Constructs a Variant holding the alternative type T that would be
     * selected by overload resolution. Constructor is enabled only if: T !=
     * Self, in_place tag is not used, T is convertible to any type from
     * Ts.
     *
     * @tparam T type which shall be converted into one of Ts.
     * @param t value which will initialize Alternative
     */
    template<
      class T,
      typename = inter::requires_<inter::not_<equals_self_v<T>>>,
      typename =
        inter::requires_<inter::not_<inter::is_in_place_v<std::decay_t<T>>>>,
      /**
       * FIXME: Add missing 23.7.1.16 "unless is_constructible_v<Tj, T> is true,
       * and unless the expression FUN(std::forward<T>(t)) (with FUN being the
       * above mentioned set of imaginary functions) is well formed"
       */
      class Ti = inter::find_matching_type_t<std::is_convertible, T, Ts...>>
    constexpr Variant(T&& t) noexcept(inter::is_nothrow_constructible_v<Ti, T>)
      : _impl(std::forward<T>(t))
    {}

    /**
     * @brief In place by type constructor.
     *
     * @remarks Postconditions as in 23.7.3.1.20 are realised by the underlying
     * std::variant implementation from C++17 standard.
     *
     * Constructs a Variant with the specified alternative T and initializes the
     * contained value with the arguments.
     *
     * @tparam T type in_place which constructor is called.
     * @tparam Args arguments passed to constructor.
     * @param t in_place_type_t tag.
     * @param args arguments passed to constructor.
     */
    template<
      class T,
      class... Args,
      typename = inter::requires_<
        inter::is_unique_v<T, Ts...> && inter::is_constructible_v<T, Args...>>>
    constexpr explicit Variant(ara::core::in_place_type_t<T>, Args&&... args)
      : _impl(std::in_place_type_t<T>{}, std::forward<Args>(args)...)
    {}


    /**
     * @brief In place by type constructor with initializer_list.
     *
     * @remarks Postconditions as in 23.7.3.1.24 are realised by the underlying
     * std::variant implementation from C++17 standard.
     *
     * Constructs a Variant with the specified alternative T and initializes the
     * contained value with the arguments.
     *
     * @tparam T type in_place which constructor is called.
     * @tparam U type used in initializer_list.
     * @tparam Args arguments passed to constructor.
     * @param t in_place_type_t tag.
     * @param il initializer_list.
     * @param args arguments passed to constructor.
     */
    template<
      class T,
      class U,
      class... Args,
      typename = inter::requires_<
        inter::is_unique_v<
          T,
          Ts...> && inter::is_constructible_v<T, std::initializer_list<U>&, Args...>>>
    constexpr explicit Variant(ara::core::in_place_type_t<T>,
                               std::initializer_list<U> il,
                               Args&&... args)
      : _impl(std::in_place_type_t<T>{}, il, std::forward<Args>(args)...)
    {}

    /**
     * @brief In place by index constructor.
     *
     * @remarks Postconditions as in 23.7.3.1.28 are realised by the underlying
     * std::variant implementation from C++17 standard.
     *
     * Constructs a Variant with the specified alternative T and initializes the
     * contained value with the arguments.
     *
     * @tparam I index of type in_place which constructor is called.
     * @tparam Args arguments passed to constructor.
     * @param t in_place_index_t tag.
     * @param args arguments passed to constructor.
     */
    template<std::size_t I,
             class... Args,
             typename = inter::requires_<inter::is_in_range_v<I, Ts...>>,
             typename =
               inter::requires_<(inter::is_constructible_v<Ts, Args...> && ...)>>
    constexpr explicit Variant(ara::core::in_place_index_t<I>, Args&&... args)
      : _impl(std::in_place_index_t<I>{}, std::forward<Args>(args)...)
    {}

    /**
     * @brief In place by index constructor with initializer_list.
     *
     * @remarks Postconditions as in 23.7.3.1.32 are realised by the underlying
     * std::variant implementation from C++17 standard.
     *
     * Constructs a Variant with the specified alternative T and initializes the
     * contained value with the arguments.
     *
     * @tparam I index of type in_place which constructor is called.
     * @tparam U type used in initializer_list.
     * @tparam Args arguments passed to constructor.
     * @param t in_place_index_t tag.
     * @param il initializer_list.
     * @param args arguments passed to constructor.
     */
    template<
      std::size_t I,
      class U,
      class... Args,
      typename = inter::requires_<inter::is_in_range_v<I, Ts...>>,
      typename = inter::requires_<(
        std::is_constructible_v<Ts, std::initializer_list<U>&, Args...> || ...)>>
    constexpr explicit Variant(ara::core::in_place_index_t<I>,
                               std::initializer_list<U> il,
                               Args&&... args)
      : _impl(std::in_place_index_t<I>{}, il, std::forward<Args>(args)...)
    {}
    /** @} */  // end of Constructors

    /**
     * @brief Destructor.
     *
     * @remarks Requirements as in 23.7.3.2.{1-2} are realised by the underlying
     * std::variant implementation from C++17 standard.
     *
     * FIXME: Implement base class inheritance with type_traits to choose
     * correct destructor - in final version.
     *
     * If valueless_by_exception is true, does nothing. Otherwise, destroys the
     * currently contained value.
     */
    ~Variant() = default;

    /**
     * @brief Copy-assignment.
     *
     * @remarks Postconditions as in 23.7.3.3.4 are realised by the underlying
     * std::variant implementation from C++17 standard.
     *
     * FIXME: implement support of 23.7.3.3.5. Should be implemented as
     * combination of type_traits and base class inheritance. In current
     * implementation regular SFINAE will not work.
     *
     * Assigns a new value to an existing variant object.
     *
     * @param rhs Variant to be assigned from.
     * @return Variant
     */
    constexpr Variant& operator=(const Variant& rhs)
    {
        _impl = rhs._impl;
        return *this;
    }

    /**
     * @brief Move-assignment.
     *
     * Assigns a new value to an existing variant object.
     *
     * FIXME: Implement support of 23.7.3.3.9. Should be implemented as
     * combination of type_traits and base class inheritance. In current
     * implementation regular SFINAE will not work.
     *
     * @param rhs Variant to be assigned from.
     * @return Variant
     */
    constexpr Variant& operator=(Variant&& rhs) noexcept(
      ((inter::is_nothrow_move_constructible_v<
          Ts> && inter::is_nothrow_move_assignable_v<Ts>) &&...))
    {
        _impl = std::move(rhs._impl);
        return *this;
    }

    /**
     * @brief Converting assignment.
     *
     * Assigns the alternative of type T that would be
     * selected by overload resolution. Operator is enabled only if: T !=
     * Self, in_place, T is convertible to any type from Ts.
     *
     * @remarks Postconditions as in 23.7.3.3.12 are realised by the underlying
     * std::variant implementation from C++17 standard.
     *
     * FIXME: Implement missing part of 23.7.3.3.14. Lack of functionality for
     * imaginary function FUN is blocking
     *
     * @tparam T type which
     * @param rhs Variant to be assigned from.
     * @return Variant
     */
    template<
      class T,
      typename = inter::requires_<inter::not_<equals_self_v<T>>>,
      class Ti = inter::find_matching_type_t<std::is_convertible, T, Ts...>>
    Variant& operator=(T&& t) noexcept(
      inter::is_nothrow_assignable_v<Ti, inter::decay_t<T>>&&
        inter::is_nothrow_constructible_v<Ti, inter::decay_t<T>>)
    {
        _impl = _impl.template operator=<T>(std::forward<T>(t));
        return *this;
    }

    /** @defgroup Observers
     *  @{
     */

    /**
     * @brief Returns the zero-based index of the alternative that is currently
     * held by the variant. If the variant is valueless_by_exception, returns
     * variant_npos.
     *
     * @return zero-based index of the alternative that is currently held by the
     * variant
     * @return returns variant_npos otherwise.
     */
    constexpr std::size_t index() const noexcept { return _impl.index(); }

    /**
     * @brief Checks if the variant is in the invalid state.
     *
     * @return returns true if Variant is in invalid state.
     * @return false otherwise.
     */
    constexpr bool valueless_by_exception() const noexcept
    {
        return _impl.valueless_by_exception();
    }

    /** @}*/  // end of Observers

    /** @defgroup Modifiers
     *  @{
     */

    /**
     * @brief Creates a new value in-place, in an existing variant object.
     *
     * @pre Method is enabled if: T is constructible with Args and T is unique
     * in Ts.
     *
     * @tparam T type in which value is emplaced.
     * @tparam Args arguments to be emplaced.
     * @param args arguments to be emplaced.
     */
    template<class T, class... Args> inter::requires_<
      inter::is_constructible_v<T, Args...> && inter::is_unique_v<T, Ts...>,
      T&>
    emplace(Args&&... args)
    {
        constexpr std::size_t index = inter::element_pos_v<T, Ts...>;
        return emplace<index>(std::forward<Args>(args)...);
    }

    /**
     * @brief Creates a new value in-place, in an existing variant object.
     *
     * Value is emplaced using initializer_list.
     *
     * @pre Method is enabled if: T is constructible with initializer_list and
     * Args, T is unique in Ts.
     *
     * @remarks Postconditions as in 23.7.3.4.9 are realised by the underlying
     * std::variant implementation from C++17 standard.
     *
     * @tparam T type in which value is emplaced.
     * @tparam U type of initializer_list.
     * @tparam Args arguments to be emplaced.
     * @param il initializer_list.
     * @param args arguments to be emplaced.
     */
    template<class T, class U, class... Args> inter::requires_<
      inter::is_constructible_v<T,
                                std::initializer_list<U>&,
                                Args...> && inter::is_unique_v<T, Ts...>,
      T&>
    emplace(std::initializer_list<U> il, Args&&... args)
    {
        return _impl.template emplace<T>(il, std::forward<Args>(args)...);
    }

    /**
     * @brief Creates a new value in-place, in an existing variant object.
     *
     * @pre Method is enabled if: found T by index I is constructible with Args
     * and T is unique in Ts. I has to be in the range of Ts.
     *
     * @remarks Postconditions as in 23.7.3.4.15 are realised by the underlying
     * std::variant implementation from C++17 standard.
     *
     * @tparam I index of T in which value is emplaced.
     * @tparam Args arguments to be emplaced.
     * @param args arguments to be emplaced.
     */
    template<size_t I, class... Args> inter::requires_<
      inter::is_constructible_v<variant_alternative_t<I, Variant>, Args...>,
      variant_alternative_t<I, Variant>&>
    emplace(Args&&... args)
    {
        static_assert(inter::is_in_range_v<I, Ts...>,
                      "Index must be in range of alternatives number");
        return _impl.template emplace<I>(std::forward<Args>(args)...);
    }

    /**
     * @brief Creates a new value in-place, in an existing variant object.
     *
     * Value is emplaced using initializer_list.
     *
     * @pre Method is enabled if: found T by index I is constructible with
     * initializer_list and Args, T is unique in Ts. I has to be in
     * the range of Ts.
     *
     * @tparam I index of T in which value is emplaced.
     * @tparam U type of initializer_list.
     * @tparam Args arguments to be emplaced.
     * @param il initializer_list.
     * @param args arguments to be emplaced.
     */
    template<size_t I, class U, class... Args>
    inter::requires_<inter::is_constructible_v<variant_alternative_t<I, Variant>,
                                               std::initializer_list<U>&,
                                               Args...>,
                     variant_alternative_t<I, Variant>&>
    emplace(std::initializer_list<U> il, Args&&... args)
    {
        static_assert(inter::is_in_range_v<I, Ts...>,
                      "Index must be in range of alternatives number");
        return _impl.template emplace<I, U, Args...>(il,
                                                     std::forward<Args>(
                                                       args)...);
    }

    /**
     * @brief Swaps two variant objects.
     *
     * @param rhs Variant which will be swapped with *this.
     */
    void swap(Variant& rhs) noexcept(
      ((inter::is_nothrow_move_constructible_v<
          Ts> && inter::is_nothrow_swappable_v<Ts>) &&...))
    {
        _impl.swap(rhs._impl);
    }
    /** @} */  // end of Modifiers


    /** @addtogroup Non-member
     *  Non-member friend functions
     *  @{
     */
    template<std::size_t I, class... Types>
    friend constexpr variant_alternative_t<I, Variant<Types...>>&
    get(Variant<Types...>& v);
    template<std::size_t I, class... Types>
    friend constexpr variant_alternative_t<I, Variant<Types...>>&&
    get(Variant<Types...>&& v);
    template<std::size_t I, class... Types>
    friend constexpr const variant_alternative_t<I, Variant<Types...>>&
    get(const Variant<Types...>& v);
    template<std::size_t I, class... Types>
    friend constexpr const variant_alternative_t<I, Variant<Types...>>&&
    get(const Variant<Types...>&& v);
    template<class T, class... Types> friend constexpr T&
    get(Variant<Types...>& v);
    template<class T, class... Types> friend constexpr T&&
    get(Variant<Types...>&& v);
    template<class T, class... Types> friend constexpr const T&
    get(const Variant<Types...>& v);
    template<class T, class... Types> friend constexpr const T&&
    get(const Variant<Types...>&& v);

    template<class Visitor, class... Variants> friend constexpr decltype(auto)
    visit(Visitor&& vis, Variants&&... vars);

    template<class... Types> friend constexpr bool
    operator==(const Variant<Types...>& v, const Variant<Types...>& w);
    template<class... Types> friend constexpr bool
    operator<(const Variant<Types...>& v, const Variant<Types...>& w);
    template<class... Types> friend constexpr bool
    operator<=(const Variant<Types...>& v, const Variant<Types...>& w);
    template<class... Types> friend constexpr bool
    operator>(const Variant<Types...>& v, const Variant<Types...>& w);
    template<class... Types> friend constexpr bool
    operator>=(const Variant<Types...>& v, const Variant<Types...>& w);
    /** @} */  // end of Non-member

 private:
    // wrapped member
    std::variant<Ts...> _impl;
    using WrappedType = std::variant<Ts...>;
};


/**
 * FIXME: Implement 23.7.11 (bad_variant_access) along with exception handling -
 * Github issue:
 * @link
 * https://github.com/UmlautSoftwareDevelopmentAccount/AP-CoreTypes/issues/60.
 */

/**
 * FIXME: Implement 23.7.12 (hash support) and inject it into std namespace as
 * std::hash specialization, Github issue:
 * @link
 * https://github.com/UmlautSoftwareDevelopmentAccount/AP-CoreTypes/issues/61.
 */

}  // namespace ara::core

#endif  // ARA_CORE_VARIANT_H_
