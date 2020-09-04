/**
 * Copyright (c) 2020
 * umlaut Software Development and contributors
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ARA_CORE_VARIANT_H_
#define ARA_CORE_VARIANT_H_

#include "ara/internal/type_traits.h"  //
#include "utility.h"                   //std::in_place_{type|index}_t
#include <variant>                     //std::variant

namespace ara::core {
namespace inter = ara::internal;

// forward declaration
template<typename... Alternatives> class Variant;

// 20.7.4[C++ Standard], Variant helper classes
/**
 * Empty variant type
 *
 * Unit type intended for use as a well-behaved empty alternative in
 * Variant. In particular, a variant of non-default-constructible types may
 * list Monostate as its first alternative: this makes the variant itself
 * default-constructible. All instances of Monostate compare equal.
 *
 */
struct Monostate
{};

/**
 * Equality operator.
 *
 * @return true, Monostate is always equal.
 */
constexpr bool operator==(Monostate, Monostate) noexcept
{
    return true;
}

/**
 * Inequality operator.
 *
 * @return false, Monostate is always equal.
 */
constexpr bool operator!=(Monostate, Monostate) noexcept
{
    return false;
}

/**
 * Less operator operator.
 *
 * @return false, Monostate is always equal.
 */
constexpr bool operator<(Monostate, Monostate) noexcept
{
    return false;
}

/**
 * Greater operator operator.
 *
 * @return false, Monostate is always equal.
 */
constexpr bool operator>(Monostate, Monostate) noexcept
{
    return false;
}

/**
 * Less-equal operator operator.
 *
 * @return true, Monostate is always equal.
 */
constexpr bool operator<=(Monostate, Monostate) noexcept
{
    return true;
}

/**
 * Greater-equal operator operator.
 *
 * @return true, Monostate is always equal.
 */
constexpr bool operator>=(Monostate, Monostate) noexcept
{
    return true;
}

/**
 * Get the size of the Variant's list of alternatives at compile time.
 *
 * @tparam T Variant on which size is calculated.
 * @return value std::size constexpr number of Alternatives in Variant.
 *
 */
template<class T> struct variant_size;  // undefined case

/**
 * Get the size of the Variant's list of alternatives at compile time.
 *
 * Specialization for const Variant.
 *
 * @tparam T const Variant on which size is calculated.
 * @return value std::size constexpr number of Alternatives in Variant.
 *
 */
template<class T> struct variant_size<const T> : variant_size<T>
{};

/**
 * Get the size of the Variant's list of alternatives at compile time.
 *
 * Specialization for volatile Variant.
 *
 * @tparam T volatile Variant on which size is calculated.
 * @return value std::size constexpr number of Alternatives in Variant.
 *
 */
template<class T> struct variant_size<volatile T> : variant_size<T>
{};

/**
 * Get the size of the Variant's list of alternatives at compile time.
 *
 * Specialization for const volatile Variant.
 *
 * @tparam T const volatile Variant on which size is calculated.
 * @return value std::size constexpr number of Alternatives in Variant.
 *
 */
template<class T> struct variant_size<const volatile T> : variant_size<T>
{};


/**
 * Get the size of the Variant's list of alternatives at compile time.
 *
 * @tparam Alternatives list of types hold by Variant.
 * @return value std::size constexpr number of Alternatives in Variant.
 *
 */
template<class... Alternatives> struct variant_size<Variant<Alternatives...>>
  : std::integral_constant<std::size_t, sizeof...(Alternatives)>
{};

/**
 * Helper function to get size of Variant.
 *
 * @tparam T list of types hold by Variant.
 * @return value std::size constexpr number of Alternatives in Variant.
 *
 */
template<class T> constexpr std::size_t variant_size_v = variant_size<T>::value;


/**
 * Provides compile-time indexed access to the types of the alternatives of the
 * possibly cv-qualified variant, combining cv-qualifications of the variant (if
 * any) with the cv-qualifications of the alternative.
 *
 * Index must be in range of Variant size [0; variant_size_v<Variant>) otherwise
 * program is ill-formed.
 *
 * @tparam I index of the type of Ith alternative of the Variant.
 * @return type the type of Ith alternative of the variant.
 *
 */
template<std::size_t I, class T> struct variant_alternative;  // undefined case

/**
 * Specialization used for metaprogramming - recursively.
 *
 * Traverse Alternatives until Index is 0. Each time decrement Index.
 * If Index goes beyond range,  program is ill-formed.
 *
 */

template<std::size_t I, class Head, class... Tail>
struct variant_alternative<I, Variant<Head, Tail...>>
  : variant_alternative<I - 1, Variant<Tail...>>
{};


/**
 * Specialization used for metaprogramming - positive case.
 *
 * Index is 0, current Head is resolved as found type.
 *
 */
template<class Head, class... Tail>
struct variant_alternative<0, Variant<Head, Tail...>>
{
    using type = Head;
};

/**
 * Helper function to get type Ith of Variant.
 *
 * @tparam I index of the type of Ith alternative of the Variant.
 * @return type the type of Ith alternative of the variant.
 *
 */
template<std::size_t I, class T> using variant_alternative_t =
  typename variant_alternative<I, T>::type;


/**
 * Specialization for const Variant.
 */
template<std::size_t I, class T> struct variant_alternative<I, const T>
{
    using type = std::add_const_t<variant_alternative_t<I, T>>;
};

/**
 * Specialization for volatile Variant.
 */
template<std::size_t I, class T> struct variant_alternative<I, volatile T>
{
    using type = std::add_volatile_t<variant_alternative_t<I, T>>;
};

/**
 * Specialization for const volatile Variant.
 */
template<std::size_t I, class T> struct variant_alternative<I, const volatile T>
{
    using type = std::add_cv_t<variant_alternative_t<I, T>>;
};

/**
 * Index of the variant in the invalid state.
 */
inline constexpr std::size_t variant_npos = static_cast<std::size_t>(-1);

/**
 * Checks if the variant holds the alternative T.
 * The call is ill-formed if T does not appear exactly once in Alternatives.
 *
 * @tparam T type which is checked for being held.
 * @tparam Alternatives List of all Variant types.
 * @return true if the variant currently holds the alternative T.
 * @return false otherwise.
 */
template<class T, class... Alternatives> constexpr bool
holds_alternative(const Variant<Alternatives...>& variant) noexcept
{
    static_assert(inter::is_unique_v<T, Alternatives...>, "T must be unique");
    return (variant.index() == inter::element_pos_v<T, Alternatives...>);
}

/**
 * Representation of a type-safe union.
 *
 * Variant holds and manages lifetime of a value. Value can be one of
 * alternatives provided in template variadic arguments Alternatives.
 *
 * @req {SWS_CORE_01601}.
 * @tparam Alternatives the types that may be stored in this Variant.
 *
 */
template<typename... Alternatives> class Variant
{
 private:
    // helpers

    /**
     * Helper checks if given T equals to Variant.
     *
     * Checks if provided type T is the same as Variant - self type.
     *
     * @tparam T type to compare against self.
     * @return value true if T equals Variant.
     * @return false otherwise.
     *
     */
    template<class T> static constexpr bool equals_self_v =
      inter::is_same_v<Variant, inter::decay_t<T>>;

    /**
     * Helper for accesing type at index 0 of Alternatives.
     *
     * @return type 0 index based type from Alternatives.
     *
     */
    using T_0 = variant_alternative_t<0, Variant>;

 public:
    static_assert(sizeof...(Alternatives) > 0,
                  "Variant must have at least one alternative");
    static_assert(inter::not_<(inter::is_reference_v<Alternatives> || ...)>,
                  "Variant must have no reference alternative");
    static_assert(inter::not_<(inter::is_void_v<Alternatives> || ...)>,
                  "variant must have no void alternative");

    // Constructors

    /**
     * Default constructor.
     *
     * Constructs a Variant holding the value-initialized value of the first
     * Alternative.
     *
     */
    constexpr Variant() noexcept(inter::is_nothrow_default_constructible_v<T_0>)
      : _impl()
    {}

    /**
     * Copy constructor.
     *
     * Constructs a Variant holding the same alternative as other.
     *
     * @param other Variant to be copied from.
     *
     */
    constexpr Variant(const Variant& other) : _impl(other._impl) {}

    /**
     * Move constructor.
     *
     * Constructs a Variant holding the same alternative as other.
     *
     * @param other Variant to be moved value from.
     *
     */
    constexpr Variant(Variant&& other) noexcept(
      (inter::is_nothrow_move_constructible_v<Alternatives> && ...))
      : _impl(std::forward<WrappedType>(other._impl))
    {}

    /**
     * Converting constructor.
     *
     * Constructs a Variant holding the alternative type T that would be
     * selected by overload resolution. Constructor is enabled only if: T !=
     * Self, in_place tag is not used, T is convertible to any type from
     * Alternatives.
     *
     * @tparam T type which shall be converted into one of Alternatives
     * @param t value which will initialize Alternative
     *
     */
    template<
      class T,
      typename = inter::requires_<inter::not_<equals_self_v<T>>>,
      typename = inter::requires_<inter::not_<inter::is_in_place_v<T>>>,
      class Ti =
        inter::find_matching_type_t<std::is_convertible, T, Alternatives...>>
    constexpr Variant(T&& t) noexcept(
      inter::is_nothrow_constructible_v<Ti, inter::decay_t<T>>)
      : _impl(std::forward<T>(t))
    {}

    /**
     * In place by type constructor.
     *
     * Constructs a Variant with the specified alternative T and initializes the
     * contained value with the arguments.
     *
     * @tparam T type in_place which constructor is called.
     * @tparam Args arguments passed to constructor.
     * @param t in_place_type_t tag.
     * @param args arguments passed to constructor.
     */
    template<class T, class... Args>
    constexpr explicit Variant(ara::core::in_place_type_t<T>, Args&&... args)
      : _impl(std::in_place_type_t<T>{}, std::forward<Args>(args)...)
    {}


    /**
     * In place by type constructor with initializer_list.
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
    template<class T, class U, class... Args>
    constexpr explicit Variant(ara::core::in_place_type_t<T>,
                               std::initializer_list<U> il,
                               Args&&... args)
      : _impl(std::in_place_type_t<T>{}, il, std::forward<Args>(args)...)
    {}

    /**
     * In place by index constructor.
     *
     * Constructs a Variant with the specified alternative T and initializes the
     * contained value with the arguments.
     *
     * @tparam I index of type in_place which constructor is called.
     * @tparam Args arguments passed to constructor.
     * @param t in_place_index_t tag.
     * @param args arguments passed to constructor.
     */
    template<std::size_t I, class... Args>
    constexpr explicit Variant(ara::core::in_place_index_t<I>, Args&&... args)
      : _impl(std::in_place_index_t<I>{}, std::forward<Args>(args)...)
    {}

    /**
     * In place by index constructor with initializer_list.
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
    template<std::size_t I, class U, class... Args>
    constexpr explicit Variant(ara::core::in_place_index_t<I>,
                               std::initializer_list<U> il,
                               Args&&... args)
      : _impl(std::in_place_index_t<I>{}, il, std::forward<Args>(args)...)
    {}

    /**
     * Destructor.
     *
     * If valueless_by_exception is true, does nothing. Otherwise, destroys the
     * currently contained value.
     */
    ~Variant() = default;

    /**
     * Copy-assignment.
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
     * Move-assignment.
     *
     * Assigns a new value to an existing variant object.
     *
     * @param rhs Variant to be assigned from.
     * @return Variant
     */
    constexpr Variant& operator=(Variant&& rhs) noexcept(((
      inter::is_nothrow_move_constructible_v<
        Alternatives> && inter::is_nothrow_move_assignable_v<Alternatives>) &&...))
    {
        _impl = std::move(rhs._impl);
        return *this;
    }

    /**
     * Converting assignment.
     *
     * Assigns the alternative of type T that would be
     * selected by overload resolution. Operator is enabled only if: T !=
     * Self, in_place, T is convertible to any type from Alternatives.
     *
     * @tparam T type which
     * @param rhs Variant to be assigned from.
     * @return Variant
     */
    template<
      class T,
      typename = inter::requires_<inter::not_<equals_self_v<T>>>,
      class Ti =
        inter::find_matching_type_t<std::is_convertible, T, Alternatives...>>
    Variant& operator=(T&& t) noexcept(
      inter::is_nothrow_assignable_v<Ti, inter::decay_t<T>>&&
        inter::is_nothrow_constructible_v<Ti, inter::decay_t<T>>)
    {
        _impl = _impl.template operator=<T>(std::forward<T>(t));
        return *this;
    }

    // Observers
    /**
     * Returns the zero-based index of the alternative that is currently held by
     * the variant. If the variant is valueless_by_exception, returns
     * variant_npos.
     *
     * @return zero-based index of the alternative that is currently held by the
     * variant
     * @return returns variant_npos otherwise.
     */
    constexpr std::size_t index() const noexcept { return _impl.index(); }

    /**
     * Checks if the variant is in the invalid state.
     *
     * @return returns true if Variant is in invalid state.
     * @return false otherwise.
     *
     */
    constexpr bool valueless_by_exception() const noexcept
    {
        return _impl.valueless_by_exception();
    }

    // Modifiers
    /**
     * Creates a new value in-place, in an existing variant object.
     *
     * Method is enabled if: T is constructible with Args and T is unique in
     * Alternatives.
     *
     * @tparam T type in which value is emplaced.
     * @tparam Args arguments to be emplaced.
     * @param args arguments to be emplaced.
     *
     */
    template<class T, class... Args>
    inter::requires_<inter::is_constructible_v<
                       inter::decay_t<T>,
                       Args...> && inter::is_unique_v<T, Alternatives...>,
                     T&>
    emplace(Args&&... args)
    {
        constexpr std::size_t index = inter::element_pos_v<T, Alternatives...>;
        return emplace<index>(std::forward<Args>(args)...);
    }

    /**
     * Creates a new value in-place, in an existing variant object.
     *
     * Value is emplaced using initializer_list.
     * Method is enabled if: T is constructible with initializer_list and Args,
     * T is unique in Alternatives.
     *
     * @tparam T type in which value is emplaced.
     * @tparam U type of initializer_list.
     * @tparam Args arguments to be emplaced.
     * @param il initializer_list.
     * @param args arguments to be emplaced.
     *
     */
    template<class T, class U, class... Args>
    inter::requires_<inter::is_constructible_v<
                       T,
                       std::initializer_list<U>&,
                       Args...> && inter::is_unique_v<T, Alternatives...>,
                     T&>
    emplace(std::initializer_list<U> il, Args&&... args)
    {
        return _impl.template emplace<T>(il, std::forward<Args>(args)...);
    }

    /**
     * Creates a new value in-place, in an existing variant object.
     *
     * Method is enabled if: found T by index I is constructible with Args and T
     * is unique in Alternatives. I has to be in the range of Alternatives.
     *
     * @tparam I index of T in which value is emplaced.
     * @tparam Args arguments to be emplaced.
     * @param args arguments to be emplaced.
     *
     */
    template<size_t I, class... Args> inter::requires_<
      inter::is_constructible_v<variant_alternative_t<I, Variant>, Args...>,
      variant_alternative_t<I, Variant>&>
    emplace(Args&&... args)
    {
        static_assert(inter::is_in_range_v<I, Alternatives...>,
                      "Index must be in range of alternatives number");
        return _impl.template emplace<I>(std::forward<Args>(args)...);
    }

    /**
     * Creates a new value in-place, in an existing variant object.
     *
     * Value is emplaced using initializer_list.
     * Method is enabled if: found T by index I is constructible with
     * initializer_list and Args, T is unique in Alternatives. I has to be in
     * the range of Alternatives.
     *
     * @tparam I index of T in which value is emplaced.
     * @tparam U type of initializer_list.
     * @tparam Args arguments to be emplaced.
     * @param il initializer_list.
     * @param args arguments to be emplaced.
     *
     */
    template<size_t I, class U, class... Args>
    inter::requires_<inter::is_constructible_v<variant_alternative_t<I, Variant>,
                                               std::initializer_list<U>&,
                                               Args...>,
                     variant_alternative_t<I, Variant>&>
    emplace(std::initializer_list<U> il, Args&&... args)
    {
        static_assert(inter::is_in_range_v<I, Alternatives...>,
                      "Index must be in range of alternatives number");
        return _impl.template emplace<I, U, Args...>(il,
                                                     std::forward<Args>(
                                                       args)...);
    }

    /**
     * Swaps two variant objects.
     *
     * @param rhs Variant which will be swapped with *this.
     */
    void swap(Variant& rhs) noexcept(
      ((inter::is_nothrow_move_constructible_v<
          Alternatives> && inter::is_nothrow_swappable_v<Alternatives>) &&...))
    {
        _impl.swap(rhs._impl);
    }

    // friend methods
    template<std::size_t I, class... Ts>
    friend constexpr variant_alternative_t<I, Variant<Ts...>>&
    get(Variant<Ts...>& v);
    template<std::size_t I, class... Ts>
    friend constexpr variant_alternative_t<I, Variant<Ts...>>&&
    get(Variant<Ts...>&& v);
    template<std::size_t I, class... Ts>
    friend constexpr const variant_alternative_t<I, Variant<Ts...>>&
    get(const Variant<Ts...>& v);
    template<std::size_t I, class... Ts>
    friend constexpr const variant_alternative_t<I, Variant<Ts...>>&&
                                                        get(const Variant<Ts...>&& v);
    template<class T, class... Ts> friend constexpr T&  get(Variant<Ts...>& v);
    template<class T, class... Ts> friend constexpr T&& get(Variant<Ts...>&& v);
    template<class T, class... Ts> friend constexpr const T&
    get(const Variant<Ts...>& v);
    template<class T, class... Ts> friend constexpr const T&&
    get(const Variant<Ts...>&& v);

    template<class Visitor, class... Variants> friend constexpr decltype(auto)
    visit(Visitor&& vis, Variants&&... vars);

    template<class... Ts> friend constexpr bool
    operator==(const Variant<Ts...>& v, const Variant<Ts...>& w);
    template<class... Ts> friend constexpr bool
    operator<(const Variant<Ts...>& v, const Variant<Ts...>& w);
    template<class... Ts> friend constexpr bool
    operator<=(const Variant<Ts...>& v, const Variant<Ts...>& w);
    template<class... Ts> friend constexpr bool
    operator>(const Variant<Ts...>& v, const Variant<Ts...>& w);
    template<class... Ts> friend constexpr bool
    operator>=(const Variant<Ts...>& v, const Variant<Ts...>& w);


 private:
    // wrapped member
    std::variant<Alternatives...> _impl;
    using WrappedType = std::variant<Alternatives...>;
};

/**
 * Exchange values of variants.
 *
 * Overload for std::swap function in ara::core namespace for variant.
 *
 * @req {SWS_CORE_01696}
 *
 * @tparam Alternatives types used in Variant
 * @param lhs Left hand side Variant
 * @param rhs Right hand side Variant
 *
 */
template<typename... Alternatives> void
swap(Variant<Alternatives...>& lhs, Variant<Alternatives...>& rhs)
{
    lhs.swap(rhs);
}

/**
 * Get Variant value by Index.
 *
 * Reads the value of the variant given the index.
 * Function is enabled if I is in the range of Alternatives.
 *
 * @tparam I index of accessed type.
 * @tparam Alternatives Variant types.
 * @param v Variant.
 * @return value currently assigned in Variant.
 *
 */
template<std::size_t I, class... Alternatives>
constexpr variant_alternative_t<I, Variant<Alternatives...>>&
get(Variant<Alternatives...>& v)
{
    static_assert(inter::is_in_range_v<I, Alternatives...>,
                  "Index must be in range of alternatives number");
    return std::get<I>(v._impl);
}

/**
 * Get Variant value by Index.
 *
 * Reads the value of the variant given the index. Using move semantics.
 * Function is enabled if I is in the range of Alternatives.
 *
 * @tparam I index of accessed type.
 * @tparam Alternatives Variant types.
 * @param v moved Variant.
 * @return value currently assigned in Variant.
 *
 */
template<std::size_t I, class... Alternatives>
constexpr variant_alternative_t<I, Variant<Alternatives...>>&&
get(Variant<Alternatives...>&& v)
{
    static_assert(inter::is_in_range_v<I, Alternatives...>,
                  "Index must be in range of alternatives number");
    return std::get<I>(std::forward<std::variant<Alternatives...>>(v._impl));
}

/**
 * Get Variant value by Index.
 *
 * Reads the value of the variant given the index.
 * Function is enabled if I is in the range of Alternatives.
 *
 * @tparam I index of accessed type.
 * @tparam Alternatives Variant types.
 * @param v const reference Variant.
 * @return value currently assigned in Variant.
 *
 */
template<std::size_t I, class... Alternatives>
constexpr const variant_alternative_t<I, Variant<Alternatives...>>&
get(const Variant<Alternatives...>& v)
{
    static_assert(inter::is_in_range_v<I, Alternatives...>,
                  "Index must be in range of alternatives number");
    return std::get<I>(v._impl);
}

/**
 * Get Variant value by Index.
 *
 * Reads the value of the variant given the index. Move semantics
 * Function is enabled if I is in the range of Alternatives.
 *
 * @tparam I index of accessed type.
 * @tparam Alternatives Variant types.
 * @param v moved Variant.
 * @return value currently assigned in Variant.
 *
 */
template<std::size_t I, class... Alternatives>
constexpr const variant_alternative_t<I, Variant<Alternatives...>>&&
get(const Variant<Alternatives...>&& v)
{
    static_assert(inter::is_in_range_v<I, Alternatives...>,
                  "Index must be in range of alternatives number");
    return std::get<I>(std::forward<std::variant<Alternatives...>>(v._impl));
}

/**
 * Get Variant value by Type.
 *
 * Reads the value of the variant given the type.
 * Function is enabled if I is in the range of Alternatives.
 *
 * @tparam T accessed type.
 * @tparam Alternatives Variant types.
 * @param v Variant.
 * @return value currently assigned in Variant.
 *
 */
template<class T, class... Alternatives> constexpr T&
get(Variant<Alternatives...>& v)
{
    return std::get<T>(v._impl);
}

/**
 * Get Variant value by Type.
 *
 * Reads the value of the variant given the type. Move semantics.
 * Function is enabled if I is in the range of Alternatives.
 *
 * @tparam T accessed type.
 * @tparam Alternatives Variant types.
 * @param v moved Variant.
 * @return value currently assigned in Variant.
 *
 */
template<class T, class... Alternatives> constexpr T&&
get(Variant<Alternatives...>&& v)
{
    return std::get<T>(std::forward<std::variant<Alternatives...>>(v._impl));
}

/**
 * Get Variant value by Type.
 *
 * Reads the value of the variant given the type.
 * Function is enabled if I is in the range of Alternatives.
 *
 * @tparam T accessed type.
 * @tparam Alternatives Variant types.
 * @param v const reference Variant.
 * @return value currently assigned in Variant.
 *
 */
template<class T, class... Alternatives> constexpr const T&
get(const Variant<Alternatives...>& v)
{
    return std::get<T>(v._impl);
}

/**
 * Get Variant value by Type.
 *
 * Reads the value of the variant given the type. Move semantics.
 * Function is enabled if I is in the range of Alternatives.
 *
 * @tparam T accessed type.
 * @tparam Alternatives Variant types.
 * @param v moved Variant.
 * @return value currently assigned in Variant.
 *
 */
template<class T, class... Alternatives> constexpr const T&&
get(const Variant<Alternatives...>&& v)
{
    return std::get<T>(std::forward<std::variant<Alternatives...>>(v._impl));
}

/**
 * Applies the visitor to the Variants.
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
 * Get a pointer to the value of a pointed-to variant given the index.
 *
 * Index must be in range of Alternatives.
 *
 * @tparam I index of type.
 * @tparam Alternatives types of Variant.
 * @param Pv pointer to Variant or null in case of error.
 *
 */
template<std::size_t I, class... Alternatives>
constexpr std::add_pointer_t<variant_alternative_t<I, Variant<Alternatives...>>>
get_if(Variant<Alternatives...>* pv) noexcept
{
    static_assert(inter::is_in_range_v<I, Alternatives...>,
                  "Index must be in range of alternatives number");
    static_assert(inter::not_<inter::is_void_v<
                    variant_alternative_t<I, Variant<Alternatives...>>>>,
                  "Indexed type can't be void");
    if (pv && pv->index() == I)
    {
        return std::addressof(get<I>(*pv));
    }
    return nullptr;
}

/**
 * Get a pointer to the value of a pointed-to const variant given the index.
 *
 * Index must be in range of Alternatives.
 *
 * @tparam I index of type.
 * @tparam Alternatives types of Variant.
 * @param Pv pointer to Variant or null in case of error.
 *
 */
template<std::size_t I, class... Alternatives> constexpr std::add_pointer_t<
  const variant_alternative_t<I, Variant<Alternatives...>>>
get_if(const Variant<Alternatives...>* pv) noexcept
{
    static_assert(inter::is_in_range_v<I, Alternatives...>,
                  "Index must be in range of alternatives number");
    static_assert(inter::not_<inter::is_void_v<
                    variant_alternative_t<I, Variant<Alternatives...>>>>,
                  "Indexed type can't be void");
    if (pv && pv->index() == I)
    {
        return std::addressof(get<I>(*pv));
    }
    return nullptr;
}

/**
 * Get a pointer to the value of a pointed-to variant given the type.
 *
 * Type must be unique in Alternatives
 *
 * @tparam T unique type to look up.
 * @tparam Alternatives types of Variant.
 * @param Pv pointer to Variant or null in case of error.
 *
 */
template<class T, class... Alternatives> constexpr std::add_pointer_t<T>
get_if(Variant<Alternatives...>* pv) noexcept
{
    static_assert(inter::is_unique_v<T, Alternatives...>, "T must be unique");
    static_assert(inter::not_<inter::is_void_v<T>>, "T can't be void");
    return get_if<inter::element_pos_v<T, Alternatives...>>(pv);
}

/**
 * Get a pointer to the value of a pointed-to variant given the const type.
 *
 * Type must be unique in Alternatives
 *
 * @tparam T const unique type to look up.
 * @tparam Alternatives types of Variant.
 * @param Pv pointer to Variant or null in case of error.
 *
 */
template<class T, class... Alternatives> constexpr std::add_pointer_t<const T>
get_if(const Variant<Alternatives...>* pv) noexcept
{
    static_assert(inter::is_unique_v<T, Alternatives...>, "T must be unique");
    static_assert(inter::not_<inter::is_void_v<T>>, "T can't be void");
    return get_if<inter::element_pos_v<T, Alternatives...>>(pv);
}

/**
 * Equality operator for Variant.
 *
 * @param v left hand side Variant.
 * @param w right hand side Variant.
 * @return false if v.index() != w.index.
 * @return true if v is valueless_by_exception or currently held values in v and
 * w are equal.
 */
template<class... Alternatives> constexpr bool
operator==(const Variant<Alternatives...>& v, const Variant<Alternatives...>& w)
{
    return v._impl == w._impl;
}

/**
 * Inequality operator for Variant.
 *
 * @param v left hand side Variant.
 * @param w right hand side Variant.
 * @return true if v.index() != w.index.
 * @return false if v is valueless_by_exception or currently held values in v
 * and w are equal.
 */
template<class... Alternatives> constexpr bool
operator!=(const Variant<Alternatives...>& v, const Variant<Alternatives...>& w)
{
    return ! (v == w);
}

/**
 * Less-than operator for Variant.
 *
 * @param v left hand side Variant.
 * @param w right hand side Variant.
 * @return true if v is value_less_by_exception or index of v is lesser than
 * index of w
 * @return false if w is valueless_by_exception or index of v is greater than
 * index of w
 * @return std::get<v.index()>(v) < std::get<v.index()>(w) otherwise.
 */
template<class... Alternatives> constexpr bool
operator<(const Variant<Alternatives...>& v, const Variant<Alternatives...>& w)
{
    return v._impl < w._impl;
}

/**
 * Greater-than operator for Variant.
 *
 * @param v left hand side Variant.
 * @param w right hand side Variant.
 * @return true if w is value_less_by_exception or index of v is greater than
 * index of w
 * @return false if v is valueless_by_exception or index of v is lesser than
 * index of w
 * @return std::get<v.index()>(v) > std::get<v.index()>(w) otherwise.
 */
template<class... Alternatives> constexpr bool
operator>(const Variant<Alternatives...>& v, const Variant<Alternatives...>& w)
{
    return v._impl > w._impl;
}

/**
 * Less-equal operator for Variant.
 *
 * @param v left hand side Variant.
 * @param w right hand side Variant.
 * @return true if v is value_less_by_exception or index of v is lesser than
 * index of w
 * @return false if w is valueless_by_exception or index of v is greater than
 * index of w
 * @return std::get<v.index()>(v) <= std::get<v.index()>(w) otherwise.
 */
template<class... Alternatives> constexpr bool
operator<=(const Variant<Alternatives...>& v, const Variant<Alternatives...>& w)
{
    return v._impl <= w._impl;
}

/**
 * Greater-equal operator for Variant.
 *
 * @param v left hand side Variant.
 * @param w right hand side Variant.
 * @return true if w is value_less_by_exception or index of v is greater than
 * index of w
 * @return false if v is valueless_by_exception or index of v is lesser than
 * index of w
 * @return std::get<v.index()>(v) >= std::get<v.index()>(w) otherwise.
 */
template<class... Alternatives> constexpr bool
operator>=(const Variant<Alternatives...>& v, const Variant<Alternatives...>& w)
{
    return v._impl >= w._impl;
}

}  // namespace ara::core

#endif  // ARA_CORE_VARIANT_H_
