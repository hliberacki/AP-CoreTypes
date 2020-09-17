#include <catch2/catch.hpp>

#include "ara/core/variant.h"

#include <string>
#include <type_traits>
#include <vector>

namespace core = ara::core;

template<bool B> struct MoveFoo
{
    MoveFoo() {}
    MoveFoo& operator=(const MoveFoo&) = delete;
    MoveFoo(MoveFoo&&) { CHECK(! B); }

    MoveFoo& operator=(MoveFoo&&)
    {
        CHECK(B);
        return *this;
    }
};

template<bool B> struct AssignFoo
{
    AssignFoo() {}
    AssignFoo(AssignFoo&&) = delete;
    AssignFoo(const AssignFoo&) { CHECK(! B); }
    AssignFoo& operator=(const AssignFoo&)
    {
        CHECK(B);
        return *this;
    }
    AssignFoo& operator=(AssignFoo&&) = delete;
};

template<class... Ts> struct overloaded : Ts...
{
    using Ts::operator()...;
};

template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

TEST_CASE("Variant default constructor", "[SWS_CORE], [SWS_CORE_01601]")
{
    core::Variant<int, std::string> v;

    CHECK(v.index() == 0);
    CHECK(0 == core::get<int>(v));

    SECTION("Custom type")
    {
        struct Foo
        {
            Foo() : val{12} {}
            int val;
        };

        core::Variant<Foo, int> vCustom;

        CHECK(vCustom.index() == 0);
        CHECK(12 == core::get<Foo>(vCustom).val);
    }
}

TEST_CASE("Variant copy constructor", "[SWS_CORE], [SWS_CORE_01601]")
{
    SECTION("constexpr Variant")
    {
        constexpr core::Variant<int, char> v1{1};

        static_assert(v1.index() == 0);
        static_assert(1 == core::get<int>(v1));

        SECTION("copy constructor")
        {
            constexpr core::Variant<int, char> v2{v1};

            static_assert(v2.index() == 0);
            static_assert(core::get<int>(v1) == core::get<int>(v2));
        }
    }

    SECTION("Const variant with std::string")
    {
        const core::Variant<int, std::string> v1{"abc"};

        CHECK(v1.index() == 1);
        CHECK("abc" == core::get<std::string>(v1));

        SECTION("copy constructor")
        {
            core::Variant<int, std::string> v2(v1);

            CHECK(v2.index() == 1);
            CHECK(core::get<std::string>(v1) == core::get<std::string>(v2));
        }
    }

    SECTION("Variant with std::string")
    {
        core::Variant<int, std::string> v1{"abc"};

        CHECK(v1.index() == 1);
        CHECK("abc" == core::get<std::string>(v1));

        SECTION("copy constructor")
        {
            core::Variant<int, std::string> v2(v1);

            CHECK(v2.index() == 1);
            CHECK(core::get<std::string>(v1) == core::get<std::string>(v2));
        }
    }

    SECTION("Custom type")
    {
        struct Foo
        {
            Foo(int v) : val{v} {}
            int val;
        };

        SECTION("Variant with Foo type")
        {
            core::Variant<int, Foo, std::string> vCustom{Foo{10}};

            CHECK(10 == core::get<Foo>(vCustom).val);

            SECTION("copy constructor")
            {
                core::Variant<int, Foo, std::string> vCopied{vCustom};

                CHECK(core::get<Foo>(vCustom).val
                      == core::get<Foo>(vCopied).val);
            }
        }
    }
}

TEST_CASE("Variant move constructor", "[SWS_CORE], [SWS_CORE_01601]")
{
    SECTION("const Variant")
    {
        const core::Variant<int, std::string> v1{"abc"};

        SECTION("move constructor")
        {
            core::Variant<int, std::string> v2(std::move(v1));
            CHECK(v2.index() == 1);
        }
    }

    SECTION("Variant")
    {
        core::Variant<int, std::string> v1{"abc"};

        SECTION("move contructor")
        {
            core::Variant<int, std::string> v2(std::move(v1));

            CHECK(v2.index() == 1);
            CHECK("abc" == core::get<std::string>(v2));
        }
    }

    SECTION("Custom type")
    {
        SECTION("move constructor the same types")
        {
            core::Variant<MoveFoo<true>, int> v1, v2;
            v1 = std::move(v2);
        }

        SECTION("move constructor the different types")
        {
            core::Variant<MoveFoo<false>, int> v1{0}, v2;
            v1 = std::move(v2);
        }
    }
}

TEST_CASE("Variant converting constructor", "[SWS_CORE], [SWS_CORE_01601]")
{
    SECTION("assign string")
    {
        core::Variant<int, std::string> v{"abc"};

        CHECK(v.index() == 1);
        CHECK("abc" == core::get<std::string>(v));
    }

    SECTION("assign int")
    {
        core::Variant<int, std::string> v{1};

        CHECK(v.index() == 0);
        CHECK(1 == core::get<int>(v));
    }

    SECTION("exact match")
    {
        using variantExactMatch = core::Variant<std::string, const char*>;

        SECTION("implicit")
        {
            variantExactMatch vImplicit{"abc"};
            CHECK(vImplicit.index() == 1);
        }

        SECTION("explicit")
        {
            variantExactMatch vExplicit{std::string{"abc"}};
            CHECK(vExplicit.index() == 0);
        }
    }
}

TEST_CASE("Variant in_place constructor", "[SWS_CORE], [SWS_CORE_01601]")
{
    SECTION("in place by index")
    {
        SECTION("simple type")
        {
            core::Variant<int, float>
              vIndexSimple{ara::core::in_place_index_t<1>(), 10.5};

            CHECK(vIndexSimple.index() == 1);
        }

        SECTION("container type")
        {
            core::Variant<std::string, std::vector<int>>
              vIndexList{ara::core::in_place_index_t<1>(), {1, 2, 3}};

            CHECK(vIndexList.index() == 1);
        }
    }

    SECTION("in place by type")
    {

        SECTION("simple type")
        {
            core::Variant<int, float>
              vTypeSimple{ara::core::in_place_type_t<int>(), 10.5};
            CHECK(vTypeSimple.index() == 0);
        }

        SECTION("container type")
        {
            core::Variant<std::string, std::vector<int>>
              vTypeList{ara::core::in_place_type_t<std::vector<int>>(),
                        {1, 2, 3}};

            CHECK(vTypeList.index() == 1);
        }
    }
}

TEST_CASE("Variant operator=", "[SWS_CORE], [SWS_CORE_01601]")
{
    SECTION("assign variant")
    {
        core::Variant<int, std::string> v1{12};
        core::Variant<int, std::string> v2{"abc"};
        v1 = v2;

        CHECK(core::get<std::string>(v1) == core::get<std::string>(v2));
    }

    SECTION("assign value")
    {
        core::Variant<int, std::string> v;
        v = "abc";

        CHECK("abc" == core::get<std::string>(v));
    }

    WHEN("custom type")
    {
        SECTION("different alternatives")
        {
            core::Variant<AssignFoo<false>, int> v1(12), v2;
            v1 = v2;
        }

        SECTION("same alternatives")
        {
            core::Variant<AssignFoo<true>, int> v1, v2;
            v1 = v2;
        }
    }
}

TEST_CASE("Variant comparision operators", "[SWS_CORE], [SWS_CORE_01601]")
{
    SECTION("operator==")
    {
        SECTION("different alternatives")
        {
            core::Variant<std::string, int> v, v2(1);
            CHECK(! (v == v2));
        }

        SECTION("same alternatives")
        {
            core::Variant<std::string, int> v, v2;
            CHECK(v == v2);

            SECTION("difference values")
            {
                core::Variant<std::string, int> v{1}, v2{2};
                CHECK(v != v2);
            }
        }
    }

    SECTION("operator!=")
    {
        SECTION("different alternatives")
        {
            core::Variant<std::string, int> v, v2(1);
            CHECK(v != v2);
        }

        SECTION("difference values")
        {
            core::Variant<std::string, int> v{1}, v2{2};
            CHECK(v != v2);
        }
    }

    SECTION("operators > >= <= <")
    {
        SECTION("different alternatives")
        {
            core::Variant<std::string, int> v, v2(1);

            CHECK(v < v2);
            CHECK(! (v > v2));
            CHECK(v <= v2);
            CHECK(! (v >= v2));
        }

        SECTION("different values")
        {
            core::Variant<std::string, int> v{1}, v2{2};

            CHECK(v < v2);
            CHECK(! (v > v2));
            CHECK(v <= v2);
            CHECK(! (v >= v2));
        }
    }
}

TEST_CASE("Variant index", "[SWS_CORE], [SWS_CORE_01601]")
{
    core::Variant<int, std::string> v;
    CHECK(v.index() == 0);

    v = 12;
    CHECK(v.index() == 0);

    v = "abc";
    CHECK(v.index() == 1);
}

TEST_CASE("Variant emplace", "[SWS_CORE], [SWS_CORE_01601]")
{
    core::Variant<int, std::string> v;

    v.emplace<0>(1);
    CHECK(v.index() == 0);
    CHECK(ara::core::get<0>(v) == 1);
    int i = 12;
    v.emplace<0>(std::move(i));

    v.emplace<1>("abc");
    CHECK(v.index() == 1);

    v.emplace<std::string>("abc");
    CHECK(v.index() == 1);

    core::Variant<std::string, std::string> vNotUnique;

    vNotUnique.emplace<0>("abc");
    CHECK(vNotUnique.index() == 0);

    core::Variant<int, std::vector<int>> vInitList;
    vInitList.emplace<std::vector<int>>({1, 2, 3, 4});
    CHECK(vInitList.index() == 1);
    vInitList.emplace<1>({3, 2, 1});
}


TEST_CASE("Get variant value", "[SWS_CORE], [SWS_CORE_01601]")
{
    core::Variant<int, std::string> v;
    SECTION("holds string")
    {
        v = "abc";

        CHECK(core::get<std::string>(v) == "abc");
        CHECK(core::get<std::string>(v) == core::get<1>(v));
    }

    SECTION("holds int")
    {
        v = 1;

        CHECK(core::get<int>(v) == 1);
        CHECK(core::get<0>(v) == core::get<int>(v));
    }
}

TEST_CASE("Conditionally get variant value", "[SWS_CORE], [SWS_CORE_01601]")
{
    core::Variant<int, float> v{12};

    SECTION("holds int")
    {
        CHECK(core::get_if<int>(&v));
        CHECK(! core::get_if<float>(&v));
        CHECK(core::get_if<0>(&v));
        CHECK(! core::get_if<1>(&v));
    }

    SECTION("holds float")
    {
        v = 1.2f;
        CHECK(! core::get_if<int>(&v));
        CHECK(core::get_if<float>(&v));
        CHECK(! core::get_if<0>(&v));
        CHECK(core::get_if<1>(&v));
    }
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
    core::Variant<int, std::string> v{"abc"};

    SECTION("holds string")
    {
        CHECK(! core::holds_alternative<int>(v));
        CHECK(core::holds_alternative<std::string>(v));
    }

    v = 1;

    SECTION("holds int")
    {
        CHECK(core::holds_alternative<int>(v));
        CHECK(! core::holds_alternative<std::string>(v));
    }
}

TEST_CASE("visit", "[SWS_CORE], [SWS_CORE_01601]")
{
    using var_t = core::Variant<int, long, double, std::string>;

    std::vector<var_t> collection = {10, 15l, 1.5, "hello"};
}

TEST_CASE("monostate", "[SWS_CORE], [SWS_CORE_01601]")
{
    struct notTrivial
    {
        notTrivial() = delete;
        notTrivial(int i) : inner{i} {};

        int inner;
    };

    SECTION("Non-trivial object inside Variant")
    {
        core::Variant<core::Monostate, notTrivial> v;

        CHECK(v.index() == 0);

        WHEN("assign value")
        {
            v = 0;
            CHECK(v.index() == 1);
        }
    }

    SECTION("Monostate equality operators")
    {
        core::Variant<core::Monostate> v1, v2;
        CHECK(v1 == v2);
        CHECK(v1 >= v2);
        CHECK(v1 <= v2);
        CHECK(! (v1 != v2));
        CHECK(! (v1 > v2));
        CHECK(! (v1 < v2));
    }
}
