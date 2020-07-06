#include <catch2/catch.hpp>

#include <string>
#include <type_traits>

#include "ara/core/variant.h"

namespace core = ara::core;

TEST_CASE("Variant default constructor", "[SWS_CORE], [SWS_CORE_01601]")
{
    core::Variant<int, std::string> v;
    CHECK(v.index() == 0);
}

TEST_CASE("Variant copy constructor", "[SWS_CORE], [SWS_CORE_01601]")
{
    {
        const std::variant<int, std::string> v1{"abc"};
        std::variant<int, std::string> v2(v1);
        CHECK(v2.index() == 1);
    }
    {
        std::variant<int, std::string> v1{"abc"};
        std::variant<int, std::string> v2(v1);
        CHECK(v2.index() == 1);
    }
}

TEST_CASE("Variant move constructor", "[SWS_CORE], [SWS_CORE_01601]")
{
    {
        const std::variant<int, std::string> v1{"abc"};
        std::variant<int, std::string> v3(std::move(v1));
        CHECK(v3.index() == 1);
    }
    {
        std::variant<int, std::string> v1{"abc"};
        std::variant<int, std::string> v3(std::move(v1));
        CHECK(v3.index() == 1);
    }
}

TEST_CASE("Variant converting constructor", "[SWS_CORE], [SWS_CORE_01601]")
{
    {
        core::Variant<int, std::string> v{"abc"};
        CHECK(v.index() == 1);
    }
    {
        core::Variant<int, std::string> v{1};
        CHECK(v.index() == 0);
    }
}

TEST_CASE("Variant in_place constructor", "[SWS_CORE], [SWS_CORE_01601]")
{
    core::Variant<int, float> vTypeSimple {std::in_place_type<int>, 10.5};
    CHECK(vTypeSimple.index() == 0);

    core::Variant<int, float> vIndexSimple {std::in_place_index<1>, 10.5};
    CHECK(vIndexSimple.index() == 1);

    core::Variant<std::string, std::vector<int>> vTypeList {std::in_place_type<std::vector<int>>,
                                                            {1, 2, 3}};
    CHECK(vTypeList.index() == 1);

    core::Variant<std::string, std::vector<int>> vIndexList {std::in_place_index<1>,
                                                            {1, 2, 3}};
    CHECK(vIndexList.index() == 1);
}

TEST_CASE("Variant operator=", "[SWS_CORE], [SWS_CORE_01601]")
{
    core::Variant<int, std::string> v1{12};
    core::Variant<int, std::string> v2{"abc"};
    v1 = v2;
}

TEST_CASE("variant_size", "[SWS_CORE], [SWS_CORE_01601]")
{
    CHECK(core::variant_size_v<core::Variant<>> == 0);
    CHECK(core::variant_size_v<core::Variant<int>> == 1);
    CHECK(core::variant_size_v<core::Variant<int, float>> == 2);
    CHECK(core::variant_size_v<core::Variant<int, float, std::string>> == 3);
}

TEST_CASE("variant_alternative", "[SWS_CORE], [SWS_CORE_01601]")
{
    using var = core::Variant<int, float>;
    CHECK(std::is_same_v<int, core::variant_alternative_t<0, var>>);
    CHECK(std::is_same_v<float, core::variant_alternative_t<1, var>>);

    CHECK(std::is_same_v<const int, core::variant_alternative_t<0, const var>>);
    CHECK(
      std::is_same_v<const float, core::variant_alternative_t<1, const var>>);

    CHECK(std::is_same_v<volatile int,
                         core::variant_alternative_t<0, volatile var>>);
    CHECK(std::is_same_v<volatile float,
                         core::variant_alternative_t<1, volatile var>>);

    CHECK(std::is_same_v<const volatile int,
                         core::variant_alternative_t<0, const volatile var>>);
    CHECK(std::is_same_v<const volatile float,
                         core::variant_alternative_t<1, const volatile var>>);
}

TEST_CASE("holds_alternative", "[SWS_CORE], [SWS_CORE_01601]")
{
    core::Variant<int, std::string> v {"abc"};
    CHECK(! core::holds_alternative<int>(v));
    CHECK(core::holds_alternative<std::string>(v));
    v = 1;
    CHECK(core::holds_alternative<int>(v));
    CHECK(! core::holds_alternative<std::string>(v));
}
