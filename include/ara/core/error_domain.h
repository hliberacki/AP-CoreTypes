/**
 * Copytight (c) 2020
 * umlaut Software Development and contributors
 * 
 * SPDX-License-Identifier: MIT
 */
#ifndef ARA_CORE_ERRORDOMAIN_H_
#define ARA_CORE_ERRORDOMAIN_H_

#include <cstdint>  // uint64_t

#include "ara/core/error_code.h"  // ErrorDomain
#include "ara/exposition.h"       // IMPLEMENTATION_DEFINED

namespace ara::core {
/**
 * Encapsulation of an error domain.
 *
 * An error domain is the controlling entity for ErrorCode’s error code values,
 * and defines the mapping of such error code values to textual representations.
 * This class is a literal type, and subclasses are strongly advised to be
 * literal types as well.
 *
 * @req {SWS_CORE_00110}
 */
class ErrorDomain
{
public:
    /**
     * Alias type for a unique ErrorDomain identifier type.
     *
     * @req {SWS_CORE_00121}
     */
    using IdType = std::uint64_t;

    /**
     * Alias type for a domain-specific error code value.
     *
     * @req {SWS_CORE_00122}
     */
    using CodeType = std::int32_t;

    /**
     * Alias type for vendor-specific supplementary data.
     *
     * @req {SWS_CORE_00123}
     */
    using SupportDataType = ara::exposition::SupportDataType_impl;

 protected:
    /**
     * Construct a new instance with the given identifier.
     *
     * Identifiers are expected to be system-wide unique.
     *
     * @param id the unique identifier
     *
     * @req {SWS_CORE_00135}
     */
    explicit constexpr ErrorDomain(IdType id) noexcept : id{id} {}

    /**
     * Destructor.
     *
     * This dtor is non-virtual (and trivial) so that this class can be a
     * literal type. While this class has virtual functions, no polymorphic
     * destruction is needed.
     *
     * @req {SWS_CORE_00136}
     */
    ~ErrorDomain() = default;

 public:
    /**
     * Copy construction shall be disabled.
     *
     * @req {SWS_CORE_00131}
     */
    ErrorDomain(ErrorDomain const&) = delete;

    /**
     * Move construction shall be disabled.
     *
     * @req {SWS_CORE_00132}
     */
    ErrorDomain(ErrorDomain&&) = delete;

    /**
     * Copy assignment shall be disabled.
     *
     * @return ErrorDomain&
     * @req {SWS_CORE_00133}
     */
    ErrorDomain& operator=(ErrorDomain const&) = delete;

    /**
     * Move assignment shall be disabled.
     *
     * @return ErrorDomain&
     * @req {SWS_CORE_00134}
     */
    ErrorDomain& operator=(ErrorDomain&&) = delete;

    /**
     * Compare for equality with another ErrorDomain instance.
     *
     * Two ErrorDomain instances compare equal when their identifiers (returned
     * by Id()) are equal.
     *
     * @param other the other instance
     * @return true if other is equal to *this
     * @return false otherwise
     * @req {SWS_CORE_00137}
     */
    constexpr bool operator==(ErrorDomain const& other) const noexcept
    {
        return id == other.id;
    }

    /**
     * Compare for non-equality with another ErrorDomain instance.
     *
     * @param other the other instance
     * @return true if other is not equal to *this
     * @return false otherwise
     * @req {SWS_CORE_00138}
     */
    constexpr bool operator!=(ErrorDomain const& other) const noexcept
    {
        return ! (*this == other);
    }

    /**
     * Return the unique domain identifier.
     *
     * @return constexpr IdType the identifier
     * @req {SWS_CORE_00151}
     */
    constexpr IdType Id() const noexcept { return id; }

    /**
     * Return the name of this error domain.
     *
     * The returned pointer remains owned by class ErrorDomain and shall not be
     * freed by clients.
     *
     * @return char const* the name as a null-terminated string, never nullptr
     * @req {SWS_CORE_00152}
     */
    virtual char const* Name() const noexcept = 0;

    /**
     * Return a textual representation of the given error code.
     *
     * It is a Violation if the errorCode did not originate from this error
     * domain, and thus be subject to SWS_CORE_00003. The returned pointer
     * remains owned by the ErrorDomain subclass and shall not be freed by
     * clients.
     *
     * @param errorCode the domain-specific error code
     * @return char const* the text as a null-terminated string, never nullptr
     * @req {SWS_CORE_00153]}
     */
    virtual char const* Message(CodeType errorCode) const noexcept = 0;

    /**
     * Throw the given error as exception.
     *
     * This function will determine the appropriate exception type for the given
     * ErrorCode and throw it. The thrown exception will contain the given
     * ErrorCode.
     *
     * @param errorCode the ErrorCode
     * @req {SWS_CORE_00154}
     */
    virtual void ThrowAsException(ErrorCode const& errorCode) const
      noexcept(false) = 0;

 private:
    /**
     * The unique domain identifier.
     */
    IdType id;
};
}  // namespace ara::core

#endif  // ARA_CORE_ERRORDOMAIN_H_
