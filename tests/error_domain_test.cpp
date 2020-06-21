#include <catch2/catch.hpp>

#include <type_traits>

#include "ara/core/error_domain.h"

namespace core = ara::core;

struct ErrorDomainTestImpl : ara::core::ErrorDomain
{
    ErrorDomainTestImpl(IdType id) : ErrorDomain(id) {}

    virtual ~ErrorDomainTestImpl() = default;

    char const* Name() const noexcept { return ""; }

    char const* Message(CodeType /*errorCode*/) const noexcept { return ""; }

    void ThrowAsException(core::ErrorCode const& /*errorCode*/) const
      noexcept(false)
    {}
};

TEST_CASE("ErrorDomain can be constructed", "[SWS_CORE], [SWS_CORE_00135]")
{
    constexpr ErrorDomainTestImpl::IdType id{0};

    REQUIRE_NOTHROW(ErrorDomainTestImpl{id});
}

TEST_CASE("ErrorDomain copy constructor is disabled", "[SWS_CORE], [SWS_CORE_00131]")
{
    REQUIRE_FALSE(std::is_copy_constructible_v<ErrorDomainTestImpl>);
}

TEST_CASE("ErrorDomain move constructor is disabled", "[SWS_CORE], [SWS_CORE_00132]")
{
    REQUIRE_FALSE(std::is_move_constructible_v<ErrorDomainTestImpl>);
}

TEST_CASE("ErrorDomain copy assignment is disabled", "[SWS_CORE], [SWS_CORE_00133]")
{
    REQUIRE_FALSE(std::is_copy_assignable_v<ErrorDomainTestImpl>);
}

TEST_CASE("ErrorDomain move assignment is disabled", "[SWS_CORE], [SWS_CORE_00134]")
{
    REQUIRE_FALSE(std::is_move_assignable_v<ErrorDomainTestImpl>);
}

TEST_CASE("ErrorDomain can be compared for equality", "[SWS_CORE], [SWS_CORE_00137]")
{
    constexpr ErrorDomainTestImpl::IdType id1{1};
    constexpr ErrorDomainTestImpl::IdType id2{2};

    ErrorDomainTestImpl error1{id1};
    ErrorDomainTestImpl error2{id2};

    REQUIRE_FALSE(error1 == error2);
}

TEST_CASE("ErrorDomain can be compared for non-equality", "[SWS_CORE], [SWS_CORE_00138]")
{
    constexpr ErrorDomainTestImpl::IdType id1{1};
    constexpr ErrorDomainTestImpl::IdType id2{2};

    ErrorDomainTestImpl error1{id1};
    ErrorDomainTestImpl error2{id2};

    REQUIRE(error1 != error2);
}

TEST_CASE("Id call returns correct ID", "[SWS_CORE], [SWS_CORE_00151]")
{
    constexpr ErrorDomainTestImpl::IdType id1{1};
    ErrorDomainTestImpl                   errorDomain{id1};

    REQUIRE(id1 == errorDomain.Id());
}
