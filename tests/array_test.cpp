#include <catch2/catch.hpp>

#include "ara/core/array.h"

TEST_CASE("Array can be constructed", "[SWS_CORE], [SWS_CORE_01201]")
{
    ara::core::Array<int, 1> array{0};

    REQUIRE(array[0] == 0);
}

TEST_CASE("Array.at()", "[SWS_CORE], [SWS_CORE_01201]")
{
    ara::core::Array<int, 1> array{0};
    REQUIRE(array.at(0) == 0);
}

TEST_CASE("Array operator[]", "[SWS_CORE], [SWS_CORE_01201]")
{
    ara::core::Array<int, 1> array;

    array[0] = 1;
    REQUIRE(array[0] == 1);
}

TEST_CASE("Array.front(), Array.back()", "[SWS_CORE], [SWS_CORE_01201]")
{
    ara::core::Array<int, 2> array{0, 1};
    REQUIRE(array.front() == 0);
    REQUIRE(array.back() == 1);
}

TEST_CASE("Array.data()", "[SWS_CORE], [SWS_CORE_01201]")
{
    ara::core::Array<int, 1> array{0};

    REQUIRE(array.data()[0] == 0);
}

TEST_CASE("Array.begin(), Array.end(), Array.rbegin(), Array.rend()", "[SWS_CORE], [SWS_CORE_01201]")
{
    ara::core::Array<int, 3> array{0, 1, 2};
    int i = 0;
    std::for_each(array.begin(), array.end(), [&](int v)
    {
        REQUIRE(v == i++);
    });

    i--;

    std::for_each(array.rbegin(), array.rend(), [&](int v)
    {
        REQUIRE(v == i--);
    });
}

TEST_CASE("Array.empty()", "[SWS_CORE], [SWS_CORE_01201]")
{
    ara::core::Array<int, 0> array;

    REQUIRE(array.empty() == true);
}

TEST_CASE("Array.size(). Array.max_size()", "[SWS_CORE], [SWS_CORE_01201]")
{
    ara::core::Array<int, 1> array;

    REQUIRE(array.size() == 1);
    REQUIRE(array.max_size() == 1);
}

TEST_CASE("Array.fill()", "[SWS_CORE], [SWS_CORE_01201]")
{
    ara::core::Array<int, 3> array;
    array.fill(1);

    std::for_each(array.begin(), array.end(), [](int v)
    {
        REQUIRE(v == 1);
    });
}

TEST_CASE("ara::core::swap", "[SWS_CORE], [SWS_CORE_01296]")
{
    ara::core::Array<int, 3> p{0, 1, 2};
    ara::core::Array<int, 3> q{2, 1, 0};

    ara::core::swap(p, q);

    REQUIRE(p[0] == 2);
    REQUIRE(q[0] == 0);
}

TEST_CASE("operator==", "[SWS_CORE], [SWS_CORE_01201]")
{
    ara::core::Array<int, 3> a{0, 1, 2};
    ara::core::Array<int, 3> b{0, 1, 2};
    ara::core::Array<int, 3> c{2, 1, 0};

    REQUIRE((a==b) == true);
    REQUIRE((b==c) == false);
}

TEST_CASE("operator<=>", "[SWS_CORE], [SWS_CORE_01201]")
{
    ara::core::Array<int, 3> a{0, 1, 2};
    ara::core::Array<int, 3> b{0, 1, 2};
    ara::core::Array<int, 2> c{0, 1};

    REQUIRE((a<=>b) == 0);
}

TEST_CASE("get<N>", "[SWS_CORE], [SWS_CORE_01201]")
{
    ara::core::Array<int, 3> a{0, 1, 2};
    REQUIRE(ara::core::get<1>(a) == 1);
}

TEST_CASE("to_array", "[SWS_CORE], [SWS_CORE_01201]")
{
    REQUIRE(ara::core::to_array({0, 1, 2}).size() == 3);
}