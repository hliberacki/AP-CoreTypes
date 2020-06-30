#include <catch2/catch.hpp>

#include <string>
#include <type_traits>

#include "ara/core/variant.h"

namespace core = ara::core;

TEST_CASE("Variant_size")
{
    CHECK(core::Variant_size_v<core::Variant<>> == 0);
    CHECK(core::Variant_size_v<core::Variant<int>> == 1);
    CHECK(core::Variant_size_v<core::Variant<int, float>> == 2);
    CHECK(core::Variant_size_v<core::Variant<int, float, std::string>> == 3);
}

TEST_CASE("Variant_alternative")
{
    using var = core::Variant<int, float>;
    CHECK(std::is_same_v<int, core::Variant_alternative_t<0, var>>);
    CHECK(std::is_same_v<float, core::Variant_alternative_t<1, var>>);

    CHECK(std::is_same_v<const int, core::Variant_alternative_t<0, const var>>);
    CHECK(std::is_same_v<const float, core::Variant_alternative_t<1, const var>>);

    CHECK(std::is_same_v<volatile int,
                         core::Variant_alternative_t<0, volatile var>>);
    CHECK(std::is_same_v<volatile float,
                         core::Variant_alternative_t<1, volatile var>>);

    CHECK(std::is_same_v<const volatile int,
                         core::Variant_alternative_t<0, const volatile var>>);
    CHECK(std::is_same_v<const volatile float,
                         core::Variant_alternative_t<1, const volatile var>>);
}