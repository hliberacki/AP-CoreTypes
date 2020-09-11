#include <catch2/catch.hpp>
#include <stdexcept>

#include "ara/core/result.h"

struct ErrorTest
{
    ErrorTest(int intVal) : intVal_{intVal}, stringVal_{"Init"} {}
    ErrorTest(std::string& stringVal) : intVal_{0}, stringVal_{stringVal} {}
    ErrorTest(int intVal, std::string& stringVal)
      : intVal_{intVal}, stringVal_{stringVal}
    {}

    void ThrowAsException() const
    {
        throw std::runtime_error("ErrorTest exception");
    }

    int         getIntVal() const { return intVal_; }
    std::string getStringVal() const { return stringVal_; }


 private:
    int         intVal_;
    std::string stringVal_;
};

bool operator==(ErrorTest const& lhs, ErrorTest const& rhs)
{
    return lhs.getIntVal() == rhs.getIntVal();
}

struct ValueTest
{
    ValueTest(int intVal) : intVal_{intVal}, stringVal_{"Init"} {}
    ValueTest(std::string& stringVal) : intVal_{0}, stringVal_{stringVal} {}
    ValueTest(int intVal, std::string& stringVal)
      : intVal_{intVal}, stringVal_{stringVal}
    {}

    int operator+(int valueToAdd)
    {
        intVal_ += valueToAdd;
        return intVal_;
    }

    int         getIntVal() const { return intVal_; }
    std::string getStringVal() const { return stringVal_; }

 private:
    int         intVal_;
    std::string stringVal_;
};

TEST_CASE("Constructs and destructs a Result based on value",
          "[SWS_CORE], [SWS_CORE_00721], [SWS_CORE_00722], [SWS_CORE_00731]")
{
    SECTION("Value provided as the lvalue")
    {
        ValueTest                               value(10);
        ara::core::Result<ValueTest, ErrorTest> resultLValue(value);
        CHECK(resultLValue.HasValue());
    }

    SECTION("Value provided as the rvalue")
    {
        ara::core::Result<ValueTest, ErrorTest> resultRValue(ValueTest(10));
        CHECK(resultRValue.HasValue());
    }
}

TEST_CASE("Constructs and destructs a Result based on error",
          "[SWS_CORE], [SWS_CORE_00723], [SWS_CORE_00724], [SWS_CORE_00731]")
{
    SECTION("Error provided as the lvalue")
    {
        ErrorTest                               error(10);
        ara::core::Result<ValueTest, ErrorTest> resultLValue(error);
        CHECK(! resultLValue.HasValue());
    }

    SECTION("Error provided as the rvalue")
    {
        ara::core::Result<ValueTest, ErrorTest> resultRValue(ErrorTest(10));
        CHECK(! resultRValue.HasValue());
    }
}

TEST_CASE("Copy and Move constructs a Result based on other instance",
          "[SWS_CORE], [SWS_CORE_00725], [SWS_CORE_00726]")
{
    GIVEN("Result with a value and Result with an error")
    {
        ValueTest                               value(10);
        ara::core::Result<ValueTest, ErrorTest> resultBasedOnValue(value);

        ErrorTest                               error(10);
        ara::core::Result<ValueTest, ErrorTest> resultBasedOnError(error);

        SECTION("Copied Result has a value")
        {
            ara::core::Result<ValueTest, ErrorTest> copiedResultBasedOnValue(
              resultBasedOnValue);
            CHECK(copiedResultBasedOnValue.HasValue());
        }

        SECTION("Moved Result has a value")
        {
            ara::core::Result<ValueTest, ErrorTest> movedResultBasedOnValue(
              std::move(resultBasedOnValue));
            CHECK(movedResultBasedOnValue.HasValue());
        }

        SECTION("Copied Result has an error")
        {
            ara::core::Result<ValueTest, ErrorTest> copiedResultBasedOnError(
              resultBasedOnError);
            CHECK(! copiedResultBasedOnError.HasValue());
        }

        SECTION("Moved Result has an error")
        {
            ara::core::Result<ValueTest, ErrorTest> movedResultBasedOnError(
              std::move(resultBasedOnError));
            CHECK(! movedResultBasedOnError.HasValue());
        }
    }
}

TEST_CASE("Construct a Result by FromValue method",
          "[SWS_CORE], [SWS_CORE_00731], [SWS_CORE_00732], [SWS_CORE_00733]")
{
    SECTION("Value passed as the lvalue")
    {
        const ValueTest value(10);
        auto            resultLValue =
          ara::core::Result<ValueTest, ErrorTest>::FromValue(value);
        CHECK(resultLValue.HasValue());
    }

    SECTION("Value passed as the rvalue")
    {
        auto resultRValue =
          ara::core::Result<ValueTest, ErrorTest>::FromValue(ValueTest(15));
        CHECK(resultRValue.HasValue());
    }

    SECTION("Variadic arguments passed to create a value")
    {
        std::string testString = "Test string";
        auto        resultFromArguments =
          ara::core::Result<ValueTest, ErrorTest>::FromValue(10, testString);
        CHECK(resultFromArguments.HasValue());
    }
}

TEST_CASE("Construct a Result by FromError method",
          "[SWS_CORE], [SWS_CORE_00734], [SWS_CORE_00735], [SWS_CORE_00736]")
{
    SECTION("Error passed as the lvalue")
    {
        const ErrorTest error(10);
        auto            resultLValue =
          ara::core::Result<ValueTest, ErrorTest>::FromError(error);
        CHECK(! resultLValue.HasValue());
    }

    SECTION("Error passed as the rvalue")
    {
        auto resultRValue =
          ara::core::Result<ValueTest, ErrorTest>::FromError(ErrorTest(15));
        CHECK(! resultRValue.HasValue());
    }

    SECTION("Variadic arguments passed to create an error")
    {
        std::string testString = "Test string";
        auto        resultFromArguments =
          ara::core::Result<ValueTest, ErrorTest>::FromError(10, testString);
        CHECK(! resultFromArguments.HasValue());
    }
}

TEST_CASE("Copy-assign", "[SWS_CORE], [SWS_CORE_00741]")
{
    GIVEN("Two Results with a value and two Results with an error")
    {
        auto resultWithValue1 =
          ara::core::Result<ValueTest, ErrorTest>::FromValue(ValueTest(10));
        auto resultWithValue2 =
          ara::core::Result<ValueTest, ErrorTest>::FromValue(ValueTest(20));

        auto resultWithError1 =
          ara::core::Result<ValueTest, ErrorTest>::FromError(ErrorTest(30));
        auto resultWithError2 =
          ara::core::Result<ValueTest, ErrorTest>::FromError(ErrorTest(40));

        WHEN("Result1 has a value and Result2 has a value")
        {
            CHECK(resultWithValue1.HasValue());
            CHECK(resultWithValue2.HasValue());
            CHECK(resultWithValue1.Value().getIntVal() == 10);
            CHECK(resultWithValue2.Value().getIntVal() == 20);
            THEN("Both results have the same value")
            {
                resultWithValue1 = resultWithValue2;

                CHECK(resultWithValue1.Value().getIntVal() == 20);
                CHECK(resultWithValue2.Value().getIntVal() == 20);
            }
        }

        WHEN("Result1 doesn't have a value and Result2 doesn't have a value")
        {
            CHECK(! resultWithError1.HasValue());
            CHECK(! resultWithError2.HasValue());
            CHECK(resultWithError1.Error().getIntVal() == 30);
            CHECK(resultWithError2.Error().getIntVal() == 40);

            THEN("Both results have the same error")
            {
                resultWithError1 = resultWithError2;

                CHECK(resultWithError1.Error().getIntVal() == 40);
                CHECK(resultWithError2.Error().getIntVal() == 40);
            }
        }

        WHEN("Result1 has a value and Result2 doesn't have a value")
        {
            THEN("Result1 has an error")
            {
                resultWithValue1 = resultWithError1;
                CHECK(! resultWithValue1.HasValue());
                CHECK(resultWithValue1.Error().getIntVal() == 30);
            }
        }

        WHEN("Result1 doesn't have a value and Result2 has a value")
        {
            THEN("Result1 has a value")
            {
                resultWithError2 = resultWithValue2;
                CHECK(resultWithError2.HasValue());
                CHECK(resultWithError2.Value().getIntVal() == 20);
            }
        }
    }
}

TEST_CASE("Move-assign", "[SWS_CORE], [SWS_CORE_00742]")
{
    GIVEN("Result with a value and Result with an error")
    {
        auto resultWithValue1 =
          ara::core::Result<ValueTest, ErrorTest>::FromValue(ValueTest(10));

        auto resultWithError1 =
          ara::core::Result<ValueTest, ErrorTest>::FromError(ErrorTest(30));

        WHEN("Result1 has a value and Result2 has a value")
        {
            CHECK(resultWithValue1.HasValue());
            CHECK(resultWithValue1.Value().getIntVal() == 10);
            THEN("Result1 has the same value as Result2")
            {
                resultWithValue1 =
                  ara::core::Result<ValueTest, ErrorTest>::FromValue(
                    ValueTest(20));

                CHECK(resultWithValue1.Value().getIntVal() == 20);
            }
        }

        WHEN("Result1 doesn't have a value and Result2 doesn't have a value")
        {
            CHECK(! resultWithError1.HasValue());
            CHECK(resultWithError1.Error().getIntVal() == 30);

            THEN("Result1 has the same error as Result2")
            {
                resultWithError1 =
                  ara::core::Result<ValueTest, ErrorTest>::FromError(
                    ErrorTest(40));

                CHECK(resultWithError1.Error().getIntVal() == 40);
            }
        }

        WHEN("Result1 has a value and Result2 doesn't have a value")
        {
            THEN("Result1 has the same error as Result2")
            {
                resultWithValue1 =
                  ara::core::Result<ValueTest, ErrorTest>::FromError(
                    ErrorTest(50));
                CHECK(! resultWithValue1.HasValue());
                CHECK(resultWithValue1.Error().getIntVal() == 50);
            }
        }

        WHEN("Result1 doesn't have a value and Result2 has a value")
        {
            THEN("Result1 has the same value as Result2")
            {
                resultWithError1 =
                  ara::core::Result<ValueTest, ErrorTest>::FromValue(
                    ValueTest(60));
                CHECK(resultWithError1.HasValue());
                CHECK(resultWithError1.Value().getIntVal() == 60);
            }
        }
    }
}

TEST_CASE("EmplaceValue method", "[SWS_CORE], [SWS_CORE_00743]")
{
    GIVEN("Result with a value and Result with an error")
    {
        std::string message = "New value message";

        WHEN("Result already has a value")
        {
            auto resultWithValue1 =
              ara::core::Result<ValueTest, ErrorTest>::FromValue(ValueTest(10));

            CHECK(resultWithValue1.Value().getIntVal() == 10);
            CHECK(resultWithValue1.Value().getStringVal() == "Init");

            THEN("it is replaced by the new one")
            {
                resultWithValue1.EmplaceValue(20, message);

                CHECK(resultWithValue1.HasValue());
                CHECK(resultWithValue1.Value().getIntVal() == 20);
                CHECK(resultWithValue1.Value().getStringVal() == message);
            }
        }

        WHEN("Result already has an error")
        {
            auto resultWithError1 =
              ara::core::Result<ValueTest, ErrorTest>::FromError(ErrorTest(10));
            CHECK(! resultWithError1.HasValue());
            CHECK(resultWithError1.Error().getIntVal() == 10);
            CHECK(resultWithError1.Error().getStringVal() == "Init");

            THEN("it is destroyed and Result has a value")
            {
                resultWithError1.EmplaceValue(20, message);
                CHECK(resultWithError1.HasValue());
                CHECK(resultWithError1.Value().getIntVal() == 20);
                CHECK(resultWithError1.Value().getStringVal() == message);
            }
        }
    }
}

TEST_CASE("EmplaceError method", "[SWS_CORE], [SWS_CORE_00744]")
{
    GIVEN("Result with a value and Result with an error")
    {
        std::string message = "New value message";

        WHEN("Result already has a value")
        {
            auto resultWithValue1 =
              ara::core::Result<ValueTest, ErrorTest>::FromValue(ValueTest(10));

            CHECK(resultWithValue1.Value().getIntVal() == 10);
            CHECK(resultWithValue1.Value().getStringVal() == "Init");

            THEN("it is destroyed and Result has an error")
            {
                resultWithValue1.EmplaceError(20, message);

                CHECK(! resultWithValue1.HasValue());
                CHECK(resultWithValue1.Error().getIntVal() == 20);
                CHECK(resultWithValue1.Error().getStringVal() == message);
            }
        }

        WHEN("Result already has an error")
        {
            auto resultWithError1 =
              ara::core::Result<ValueTest, ErrorTest>::FromError(ErrorTest(10));
            CHECK(! resultWithError1.HasValue());

            THEN("it is replaced by the new one")
            {
                resultWithError1.EmplaceError(20, message);
                CHECK(! resultWithError1.HasValue());
                CHECK(resultWithError1.Error().getIntVal() == 20);
                CHECK(resultWithError1.Error().getStringVal() == message);
            }
        }
    }
}

TEST_CASE("Swap method", "[SWS_CORE], [SWS_CORE_00745]")
{
    GIVEN("Two Results with a value and two Results with an error")
    {
        int         value1        = 11;
        int         value2        = 12;
        int         error1        = 21;
        int         error2        = 22;
        std::string valueMessage1 = "Value Message1";
        std::string valueMessage2 = "Value Message2";
        std::string errorMessage1 = "Error Message1";
        std::string errorMessage2 = "Error Message2";

        auto resultWithValue1 =
          ara::core::Result<ValueTest, ErrorTest>::FromValue(
            ValueTest(value1, valueMessage1));
        auto resultWithValue2 =
          ara::core::Result<ValueTest, ErrorTest>::FromValue(
            ValueTest(value2, valueMessage2));
        CHECK(resultWithValue1.HasValue());
        CHECK(resultWithValue2.HasValue());
        CHECK(resultWithValue1.Value().getIntVal() == value1);
        CHECK(resultWithValue2.Value().getIntVal() == value2);
        CHECK(resultWithValue1.Value().getStringVal() == valueMessage1);
        CHECK(resultWithValue2.Value().getStringVal() == valueMessage2);

        auto resultWithError1 =
          ara::core::Result<ValueTest, ErrorTest>::FromError(
            ErrorTest(error1, errorMessage1));
        auto resultWithError2 =
          ara::core::Result<ValueTest, ErrorTest>::FromError(
            ErrorTest(error2, errorMessage2));
        CHECK(! resultWithError1.HasValue());
        CHECK(! resultWithError2.HasValue());
        CHECK(resultWithError1.Error().getIntVal() == error1);
        CHECK(resultWithError2.Error().getIntVal() == error2);
        CHECK(resultWithError1.Error().getStringVal() == errorMessage1);
        CHECK(resultWithError2.Error().getStringVal() == errorMessage2);

        WHEN("Swap a Result1 with a value and Result2 with a value")
        {
            resultWithValue1.Swap(resultWithValue2);
            THEN(
              "Result1 has a value from th Result2, and Result2 has a value from the Result1")
            {
                CHECK(resultWithValue1.HasValue());
                CHECK(resultWithValue2.HasValue());
                CHECK(resultWithValue1.Value().getIntVal() == value2);
                CHECK(resultWithValue2.Value().getIntVal() == value1);
                CHECK(resultWithValue1.Value().getStringVal() == valueMessage2);
                CHECK(resultWithValue2.Value().getStringVal() == valueMessage1);
            }
        }

        WHEN("Swap a Result1 with an error and Result2 with an error")
        {
            resultWithError1.Swap(resultWithError2);
            THEN(
              "Result1 has an error from th Result2, and Result2 has an error from the Result1")
            {
                CHECK(! resultWithError1.HasValue());
                CHECK(! resultWithError2.HasValue());
                CHECK(resultWithError1.Error().getIntVal() == error2);
                CHECK(resultWithError2.Error().getIntVal() == error1);
                CHECK(resultWithError1.Error().getStringVal() == errorMessage2);
                CHECK(resultWithError2.Error().getStringVal() == errorMessage1);
            }
        }

        WHEN("Swap a Result1 with an error and Result2 with a value")
        {
            resultWithError1.Swap(resultWithValue1);
            THEN(
              "Result1 has a value from th Result2, and Result2 has an error from the Result1")
            {
                CHECK(resultWithError1.HasValue());
                CHECK(! resultWithValue1.HasValue());
                CHECK(resultWithError1.Value().getIntVal() == value1);
                CHECK(resultWithValue1.Error().getIntVal() == error1);
                CHECK(resultWithError1.Value().getStringVal() == valueMessage1);
                CHECK(resultWithValue1.Error().getStringVal() == errorMessage1);
            }
        }

        WHEN("Swap a Result1 with a value and Result2 with an error")
        {
            resultWithValue2.Swap(resultWithError2);
            THEN(
              "Result1 has an error from th Result2, and Result2 has a value from the Result1")
            {
                CHECK(resultWithError2.HasValue());
                CHECK(! resultWithValue2.HasValue());
                CHECK(resultWithError2.Value().getIntVal() == value2);
                CHECK(resultWithValue2.Error().getIntVal() == error2);
                CHECK(resultWithError2.Value().getStringVal() == valueMessage2);
                CHECK(resultWithValue2.Error().getStringVal() == errorMessage2);
            }
        }
    }
}

TEST_CASE(
  "Value getters and checkers",
  "[SWS_CORE], [SWS_CORE_00751], [SWS_CORE_00752], [SWS_CORE_00753], [SWS_CORE_00759], [SWS_CORE_00754], [SWS_CORE_00755], [SWS_CORE_00756]")
{
    GIVEN("Result with a value")
    {
        auto resultWithValue1 =
          ara::core::Result<ValueTest, ErrorTest>::FromValue(ValueTest(10));

        SECTION("Get value by HasValue()")
        {
            CHECK(resultWithValue1.HasValue());
        }

        SECTION("Get value by operator bool()") { CHECK(resultWithValue1); }

        SECTION("Get value by operator*()")
        {
            ValueTest value1 = *resultWithValue1;
            CHECK(value1.getIntVal() == 10);

            ValueTest value2(
              *(ara::core::Result<ValueTest, ErrorTest>::FromValue(
                ValueTest(10))));
            CHECK(value2.getIntVal() == 10);
        }

        SECTION("Get value by operator->()")
        {
            CHECK(resultWithValue1->getIntVal() == 10);
        }

        SECTION("Get value by Value()")
        {
            CHECK(resultWithValue1.Value().getIntVal() == 10);
            auto value3(
              ara::core::Result<ValueTest, ErrorTest>::FromValue(ValueTest(10))
                .Value());
            CHECK(value3.getIntVal() == 10);
        }
    }
}

TEST_CASE("Error getters", "[SWS_CORE], [SWS_CORE_00757], [SWS_CORE_00758]")
{
    GIVEN("Result with an error")
    {
        auto resultWithError1 =
          ara::core::Result<ValueTest, ErrorTest>::FromError(ErrorTest(10));
        ErrorTest error(
          ara::core::Result<ValueTest, ErrorTest>::FromError(ErrorTest(10))
            .Error());

        SECTION("Get error by Error()")
        {
            CHECK(resultWithError1.Error().getIntVal() == error.getIntVal());
        }
    }
}

TEST_CASE("ValueOr method", "[SWS_CORE], [SWS_CORE_00761], [SWS_CORE_00762]")
{
    GIVEN("Result with a value and Result with an error")
    {
        auto resultWithValue =
          ara::core::Result<ValueTest, ErrorTest>::FromValue(ValueTest(10));
        auto resultWithError =
          ara::core::Result<ValueTest, ErrorTest>::FromError(ErrorTest(20));

        WHEN("Result has a value")
        {
            auto      value1 = resultWithValue.ValueOr(88);
            ValueTest value3(
              ara::core::Result<ValueTest, ErrorTest>::FromValue(ValueTest(10))
                .ValueOr(88));

            THEN("it is returned")
            {
                CHECK(value1.getIntVal() == 10);
                CHECK(value3.getIntVal() == 10);
            }
        }

        WHEN("Result has an error")
        {
            auto      value2 = resultWithError.ValueOr(88);
            ValueTest value4(
              ara::core::Result<ValueTest, ErrorTest>::FromError(ErrorTest(20))
                .ValueOr(88));

            THEN("value specified by user is returned")
            {
                CHECK(value2.getIntVal() == 88);
                CHECK(value4.getIntVal() == 88);
            }
        }
    }
}

TEST_CASE("ErrorOr method", "[SWS_CORE], [SWS_CORE_00763]")
{
    GIVEN("Result with a value and Result with an error")
    {
        auto resultWithValue =
          ara::core::Result<ValueTest, ErrorTest>::FromValue(ValueTest(10));
        auto resultWithError =
          ara::core::Result<ValueTest, ErrorTest>::FromError(ErrorTest(20));

        WHEN("Result has a value")
        {
            auto error1 = resultWithValue.ErrorOr(88);
            THEN("error specified by user is returned")
            {
                CHECK(error1.getIntVal() == 88);
            }
        }

        WHEN("Result has an error")
        {
            auto error2 = resultWithError.ErrorOr(88);
            THEN("it is returned") { CHECK(error2.getIntVal() == 20); }
        }
    }
}

TEST_CASE("CheckError method", "[SWS_CORE], [SWS_CORE_00765]")
{
    GIVEN("Result with a value and Result with an error")
    {
        auto resultWithValue =
          ara::core::Result<ValueTest, ErrorTest>::FromValue(ValueTest(20));
        auto resultWithError =
          ara::core::Result<ValueTest, ErrorTest>::FromError(ErrorTest(20));

        WHEN("Result has a value")
        {
            THEN("returned value is false")
            {
                CHECK(! resultWithValue.CheckError(20));
            }
        }

        WHEN("Result has an error with the same value")
        {
            THEN("returned value is true")
            {
                CHECK(resultWithError.CheckError(20));
            }
        }

        WHEN("Result has an error with the different value")
        {
            THEN("returned value is false")
            {
                CHECK(! resultWithError.CheckError(30));
            }
        }
    }
}

#if __cpp_exceptions == 199711
TEST_CASE("ValueOrThrow method",
          "[SWS_CORE], [SWS_CORE_00766], [SWS_CORE_00769]")
{
    GIVEN("Result with a value and Result with an error")
    {
        auto resultWithValue =
          ara::core::Result<ValueTest, ErrorTest>::FromValue(ValueTest(10));
        auto resultWithError =
          ara::core::Result<ValueTest, ErrorTest>::FromError(ErrorTest(20));

        bool exceptionThrown1 = false;
        bool exceptionThrown2 = false;

        WHEN("Result contains a value")
        {
            THEN("Exception is not thrown, and value is returned")
            {
                CHECK(resultWithValue.ValueOrThrow().getIntVal() == 10);
                ValueTest value(
                  ara::core::Result<ValueTest, ErrorTest>::FromValue(
                    ValueTest(10))
                    .ValueOrThrow());
                CHECK(value.getIntVal() == 10);
            }
        }

        WHEN("Result doesn't contain a value")
        {
            try
            {
                auto value = resultWithError.ValueOrThrow();
            }
            catch (std::runtime_error&)
            {
                exceptionThrown1 = true;
            }

            try
            {
                auto value1(ara::core::Result<ValueTest, ErrorTest>::FromError(
                              ErrorTest(20))
                              .ValueOrThrow());
            }
            catch (std::runtime_error&)
            {
                exceptionThrown2 = true;
            }

            THEN("Exception is thrown")
            {
                CHECK(exceptionThrown1);
                CHECK(exceptionThrown2);
            }
        }
    }
}
#endif  // __cpp_exceptions == 199711

TEST_CASE("Resolve method", "[SWS_CORE], [SWS_CORE_00767]")
{
    GIVEN("Result with a value and Result with an error")
    {
        auto resultWithValue =
          ara::core::Result<ValueTest, ErrorTest>::FromValue(ValueTest(10));
        auto resultWithError =
          ara::core::Result<ValueTest, ErrorTest>::FromError(ErrorTest(20));

        WHEN("Result has a value")
        {
            auto value1 =
              resultWithValue.Resolve([](ErrorTest const&) { return 50; });
            THEN("it is returned") { CHECK(value1.getIntVal() == 10); }
        }

        WHEN("Result has an error")
        {
            auto value2 =
              resultWithError.Resolve([](ErrorTest const&) { return 50; });
            THEN("Provided function is called and a new value created")
            {
                CHECK(value2.getIntVal() == 50);
            }
        }
    }
}

TEST_CASE("Bind method", "[SWS_CORE], [SWS_CORE_00768]")
{
    GIVEN("Result with a value.")
    {
        auto resultWithValue =
          ara::core::Result<ValueTest, ErrorTest>::FromValue(ValueTest(10));
        SECTION("Provided function returns value_type")
        {
            auto value1 = resultWithValue.Bind(
              [](ValueTest val) -> ValueTest { return val + 50; });
            CHECK(value1.Value().getIntVal() == 60);
        }

        SECTION("Provided function returns Result<value_type, error_type>")
        {
            auto value2 = resultWithValue.Bind(
              [](ValueTest val) -> ara::core::Result<ValueTest, ErrorTest> {
                  ValueTest toReturn(val + 100);
                  return ara::core::Result<ValueTest, ErrorTest>(toReturn);
              });
            CHECK(value2.Value().getIntVal() == 110);
        }
    }
}

TEST_CASE(
  "Specialization for value_type=void: Constructs and destructs a Result with void value",
  "[SWS_CORE], [SWS_CORE_00801], [SWS_CORE_00825]")
{
    ara::core::Result<void, ErrorTest> result;

    CHECK(result.HasValue());
}

TEST_CASE(
  "Specialization for value_type=void: Constructs and destructs a Result based on error",
  "[SWS_CORE], [SWS_CORE_00823], [SWS_CORE_00825]")
{
    SECTION("Error provided as the lvalue")
    {
        ErrorTest                          error(10);
        ara::core::Result<void, ErrorTest> resultLValue(error);
        CHECK(! resultLValue.HasValue());
    }

    SECTION("Error provided as the rvalue")
    {
        ara::core::Result<void, ErrorTest> resultRValue(ErrorTest(10));
        CHECK(! resultRValue.HasValue());
    }
}

TEST_CASE(
  "Specialization for value_type=void: Copy and Move constructs a Result based on other instance",
  "[SWS_CORE], [SWS_CORE_00785], [SWS_CORE_00826]")
{
    GIVEN("Result with a value and Result with an error")
    {
        ara::core::Result<void, ErrorTest> resultBasedOnValue;

        ErrorTest                          error(10);
        ara::core::Result<void, ErrorTest> resultBasedOnError(error);

        SECTION("Copied Result has a value")
        {
            ara::core::Result<void, ErrorTest> copiedResultBasedOnValue(
              resultBasedOnValue);
            CHECK(copiedResultBasedOnValue.HasValue());
        }

        SECTION("Moved Result has a value")
        {
            ara::core::Result<void, ErrorTest> movedResultBasedOnValue(
              std::move(resultBasedOnValue));
            CHECK(movedResultBasedOnValue.HasValue());
        }

        SECTION("Copied Result has an error")
        {
            ara::core::Result<void, ErrorTest> copiedResultBasedOnError(
              resultBasedOnError);
            CHECK(! copiedResultBasedOnError.HasValue());
        }

        SECTION("Moved Result has an error")
        {
            ara::core::Result<void, ErrorTest> movedResultBasedOnError(
              std::move(resultBasedOnError));
            CHECK(! movedResultBasedOnError.HasValue());
        }
    }
}

TEST_CASE(
  "Specialization for value_type=void: Constuct a Result by FromValue method",
  "[SWS_CORE], [SWS_CORE_00831]")
{
    SECTION("Create an empty value")
    {
        auto value = ara::core::Result<void, ErrorTest>::FromValue();
        CHECK(value.HasValue());
    }
}

TEST_CASE(
  "Specialization for value_type=void: Constuct a Result by FromError method",
  "[SWS_CORE], [SWS_CORE_00834], [SWS_CORE_00835], [SWS_CORE_00836]")
{
    SECTION("Error passed as the lvalue")
    {
        const ErrorTest error(10);
        auto            resultLValue =
          ara::core::Result<void, ErrorTest>::FromError(error);
        CHECK(! resultLValue.HasValue());
    }

    SECTION("Error passed as the rvalue")
    {
        auto resultRValue =
          ara::core::Result<void, ErrorTest>::FromError(ErrorTest(15));
        CHECK(! resultRValue.HasValue());
    }

    SECTION("Variadic arguments passed to create an error")
    {
        std::string testString = "Test string";
        auto        resultFromArguments =
          ara::core::Result<void, ErrorTest>::FromError(10, testString);
        CHECK(! resultFromArguments.HasValue());
    }
}

TEST_CASE("Specialization for value_type=void: Copy-assign",
          "[SWS_CORE], [SWS_CORE_00841]")
{
    GIVEN("Two Results with a value and two Results with an error")
    {
        auto resultWithValue1 = ara::core::Result<void, ErrorTest>::FromValue();
        auto resultWithValue2 = ara::core::Result<void, ErrorTest>::FromValue();

        auto resultWithError1 =
          ara::core::Result<void, ErrorTest>::FromError(ErrorTest(30));
        auto resultWithError2 =
          ara::core::Result<void, ErrorTest>::FromError(ErrorTest(40));

        WHEN("Result1 has a value and Result2 has a value")
        {
            CHECK(resultWithValue1.HasValue());
            CHECK(resultWithValue2.HasValue());
            THEN("Both results still have a value")
            {
                resultWithValue1 = resultWithValue2;
                CHECK(resultWithValue1.HasValue());
                CHECK(resultWithValue2.HasValue());
            }
        }

        WHEN("Result1 doesn't have a value and Result2 doesn't have a value")
        {
            CHECK(! resultWithError1.HasValue());
            CHECK(! resultWithError2.HasValue());
            CHECK(resultWithError1.Error().getIntVal() == 30);
            CHECK(resultWithError2.Error().getIntVal() == 40);

            THEN("Both results have the same error")
            {
                resultWithError1 = resultWithError2;

                CHECK(resultWithError1.Error().getIntVal() == 40);
                CHECK(resultWithError2.Error().getIntVal() == 40);
            }
        }

        WHEN("Result1 has a value and Result2 doesn't have a value")
        {
            THEN("Result1 has an error")
            {
                resultWithValue1 = resultWithError1;
                CHECK(! resultWithValue1.HasValue());
                CHECK(resultWithValue1.Error().getIntVal() == 30);
            }
        }

        WHEN("Result1 doesn't have a value and Result2 has a value")
        {
            THEN("Result1 has a value")
            {
                resultWithError2 = resultWithValue2;
                CHECK(resultWithError2.HasValue());
            }
        }
    }
}

TEST_CASE("Specialization for value_type=void: Move-assign",
          "[SWS_CORE], [SWS_CORE_00842]")
{

    GIVEN("Result with a value and Result with an error")
    {
        auto resultWithValue1 = ara::core::Result<void, ErrorTest>::FromValue();
        auto resultWithError1 =
          ara::core::Result<void, ErrorTest>::FromError(ErrorTest(30));

        WHEN("Result1 has a value and Result2 has a value")
        {
            CHECK(resultWithValue1.HasValue());
            THEN("Result1 still has a value")
            {
                resultWithValue1 =
                  ara::core::Result<void, ErrorTest>::FromValue();
                CHECK(resultWithValue1.HasValue());
            }
        }


        WHEN("Result1 doesn't have a value and Result2 doesn't have a value")
        {
            CHECK(! resultWithError1.HasValue());
            CHECK(resultWithError1.Error().getIntVal() == 30);
            THEN("Result1 has the same error as Result2")
            {
                resultWithError1 =
                  ara::core::Result<void, ErrorTest>::FromError(ErrorTest(40));
                CHECK(resultWithError1.Error().getIntVal() == 40);
            }
        }

        WHEN("Result1 has a value and Result2 doesn't have a value")
        {
            THEN("Result1 has the same error as Result2")
            {
                resultWithValue1 =
                  ara::core::Result<void, ErrorTest>::FromError(ErrorTest(50));
                CHECK(! resultWithValue1.HasValue());
                CHECK(resultWithValue1.Error().getIntVal() == 50);
            }
        }

        WHEN("Result1 doesn't have a value and Result2 has a value")
        {
            THEN("Result1 has the same value as Result2")
            {
                resultWithError1 =
                  ara::core::Result<void, ErrorTest>::FromValue();
                CHECK(resultWithError1.HasValue());
            }
        }
    }
}

TEST_CASE("Specialization for value_type=void: EmplaceValue method",
          "[SWS_CORE], [SWS_CORE_00843]")
{
    GIVEN("Result with a value and Result with an error")
    {

        WHEN("Result already has a value")
        {
            auto resultWithValue1 =
              ara::core::Result<void, ErrorTest>::FromValue();
            CHECK(resultWithValue1.HasValue());
            THEN("it still has a value")
            {
                resultWithValue1.EmplaceValue();
                CHECK(resultWithValue1.HasValue());
            }
        }

        WHEN("Result already has an error")
        {
            auto resultWithError1 =
              ara::core::Result<void, ErrorTest>::FromError(ErrorTest(10));
            CHECK(! resultWithError1.HasValue());

            THEN("it is destroyed and Result has a value")
            {
                resultWithError1.EmplaceValue();
                CHECK(resultWithError1.HasValue());
            }
        }
    }
}

TEST_CASE("Specialization for value_type=void: EmplaceError method",
          "[SWS_CORE], [SWS_CORE_00844]")
{
    GIVEN("Result with a value and Result with an error")
    {
        std::string message = "New value message";

        WHEN("Result already has a value")
        {
            auto resultWithValue1 =
              ara::core::Result<void, ErrorTest>::FromValue();

            THEN("it is destroyed and Result has an error")
            {
                resultWithValue1.EmplaceError(20, message);

                CHECK(! resultWithValue1.HasValue());
                CHECK(resultWithValue1.Error().getIntVal() == 20);
                CHECK(resultWithValue1.Error().getStringVal() == message);
            }
        }

        WHEN("Result already has an error")
        {
            auto resultWithError1 =
              ara::core::Result<void, ErrorTest>::FromError(ErrorTest(10));
            CHECK(! resultWithError1.HasValue());

            THEN("it is replaced by the new one")
            {
                resultWithError1.EmplaceError(20, message);
                CHECK(! resultWithError1.HasValue());
                CHECK(resultWithError1.Error().getIntVal() == 20);
                CHECK(resultWithError1.Error().getStringVal() == message);
            }
        }
    }
}

TEST_CASE("Specialization for value_type=void: Swap method",
          "[SWS_CORE], [SWS_CORE_00845]")
{
    GIVEN("Two Results with a value and two Results with an error")
    {
        int         error1        = 21;
        int         error2        = 22;
        std::string valueMessage1 = "Value Message1";
        std::string valueMessage2 = "Value Message2";
        std::string errorMessage1 = "Error Message1";
        std::string errorMessage2 = "Error Message2";

        auto resultWithValue1 = ara::core::Result<void, ErrorTest>::FromValue();
        auto resultWithValue2 = ara::core::Result<void, ErrorTest>::FromValue();
        CHECK(resultWithValue1.HasValue());
        CHECK(resultWithValue2.HasValue());

        auto resultWithError1 = ara::core::Result<void, ErrorTest>::FromError(
          ErrorTest(error1, errorMessage1));
        auto resultWithError2 = ara::core::Result<void, ErrorTest>::FromError(
          ErrorTest(error2, errorMessage2));
        CHECK(! resultWithError1.HasValue());
        CHECK(! resultWithError2.HasValue());
        CHECK(resultWithError1.Error().getIntVal() == error1);
        CHECK(resultWithError2.Error().getIntVal() == error2);
        CHECK(resultWithError1.Error().getStringVal() == errorMessage1);
        CHECK(resultWithError2.Error().getStringVal() == errorMessage2);

        WHEN("Swap a Result1 with a value and Result2 with a value")
        {
            resultWithValue1.Swap(resultWithValue2);
            THEN("Result1 has a value and Result2 has a value")
            {
                CHECK(resultWithValue1.HasValue());
                CHECK(resultWithValue2.HasValue());
            }
        }

        WHEN("Swap a Result1 with an error and Result2 with an error")
        {
            resultWithError1.Swap(resultWithError2);

            THEN(
              "Result1 has an error from th Result2, and Result2 has an error from the Result1")
            {
                CHECK(! resultWithError1.HasValue());
                CHECK(! resultWithError2.HasValue());
                CHECK(resultWithError1.Error().getIntVal() == error2);
                CHECK(resultWithError2.Error().getIntVal() == error1);
                CHECK(resultWithError1.Error().getStringVal() == errorMessage2);
                CHECK(resultWithError2.Error().getStringVal() == errorMessage1);
            }
        }

        WHEN("Swap a Result1 with an error and Result2 with a value")
        {
            resultWithError1.Swap(resultWithValue1);
            THEN(
              "Result1 has a value from th Result2, and Result2 has an error from the Result1")
            {
                CHECK(resultWithError1.HasValue());
                CHECK(! resultWithValue1.HasValue());
                CHECK(resultWithValue1.Error().getIntVal() == error1);
                CHECK(resultWithValue1.Error().getStringVal() == errorMessage1);
            }
        }

        WHEN("Swap a Result1 with a value and Result2 with an error")
        {
            resultWithValue2.Swap(resultWithError2);
            THEN(
              "Result1 has an error from th Result2, and Result2 has a value from the Result1")
            {
                CHECK(resultWithError2.HasValue());
                CHECK(! resultWithValue2.HasValue());
                CHECK(resultWithValue2.Error().getIntVal() == error2);
                CHECK(resultWithValue2.Error().getStringVal() == errorMessage2);
            }
        }
    }
}

TEST_CASE(
  "Specialization for value_type=void: Value getters and checkers",
  "[SWS_CORE], [SWS_CORE_00851], [SWS_CORE_00852], [SWS_CORE_00853], [SWS_CORE_00855]")
{
    GIVEN("Result with a value")
    {
        auto resultWithValue1 = ara::core::Result<void, ErrorTest>::FromValue();

        SECTION("Get value by HasValue()")
        {
            CHECK(resultWithValue1.HasValue());
        }

        SECTION("Get value by operator bool()") { CHECK(resultWithValue1); }

        // just for coverage
        SECTION("Get value by operator*()") { *resultWithValue1; }


        SECTION("Get value by Value()") { resultWithValue1.Value(); }
    }
}

TEST_CASE("Specialization for value_type=void: Error getters",
          "[SWS_CORE], [SWS_CORE_00857], [SWS_CORE_00858]")
{
    GIVEN("Result with an error")
    {
        auto resultWithError1 =
          ara::core::Result<void, ErrorTest>::FromError(ErrorTest(10));
        ErrorTest error(
          ara::core::Result<void, ErrorTest>::FromError(ErrorTest(10)).Error());

        SECTION("Get error by Error()")
        {
            CHECK(resultWithError1.Error().getIntVal() == error.getIntVal());
        }
    }
}

TEST_CASE("Specialization for value_type=void: ValueOr method",
          "[SWS_CORE], [SWS_CORE_00861]")
{
    GIVEN("Result with a value")
    {
        auto resultWithValue = ara::core::Result<void, ErrorTest>::FromValue();

        CHECK(resultWithValue.HasValue());

        // just for coverage - ValueOr returns void
        resultWithValue.ValueOr(5);
    }
}

TEST_CASE("Specialization for value_type=void: ErrorOr method",
          "[SWS_CORE], [SWS_CORE_00863]")
{
    GIVEN("Result with a value and Result with an error")
    {
        auto resultWithValue = ara::core::Result<void, ErrorTest>::FromValue();
        auto resultWithError =
          ara::core::Result<void, ErrorTest>::FromError(ErrorTest(20));

        WHEN("Result has a value")
        {
            auto error1 = resultWithValue.ErrorOr(88);
            THEN("error specified by user is returned")
            {
                CHECK(error1.getIntVal() == 88);
            }
        }

        WHEN("Result has an error")
        {
            auto error2 = resultWithError.ErrorOr(88);
            THEN("it is returned") { CHECK(error2.getIntVal() == 20); }
        }
    }
}

TEST_CASE("Specialization for value_type=void: CheckError method",
          "[SWS_CORE], [SWS_CORE_00865]")
{
    GIVEN("Result with a value and Result with an error")
    {
        auto resultWithValue = ara::core::Result<void, ErrorTest>::FromValue();
        auto resultWithError =
          ara::core::Result<void, ErrorTest>::FromError(ErrorTest(20));

        WHEN("Result has a value")
        {
            THEN("returned value is false")
            {
                CHECK(! resultWithValue.CheckError(20));
            }
        }

        WHEN("Result has an error with the same value")
        {
            THEN("returned value is true")
            {
                CHECK(resultWithError.CheckError(20));
            }
        }

        WHEN("Result has an error with the different value")
        {
            THEN("returned value is false")
            {
                CHECK(! resultWithError.CheckError(30));
            }
        }
    }
}

#if __cpp_exceptions == 199711
TEST_CASE("Specialization for value_type=void: ValueOrThrow method",
          "[SWS_CORE], [SWS_CORE_00866]")
{
    GIVEN("Result with a value and Result with an error")
    {
        auto resultWithValue = ara::core::Result<void, ErrorTest>::FromValue();
        auto resultWithError =
          ara::core::Result<void, ErrorTest>::FromError(ErrorTest(20));

        bool exceptionThrown = false;

        WHEN("Result contains a value")
        {
            THEN("Exception is not thrown, and value is returned")
            {
                resultWithValue.ValueOrThrow();
            }
        }

        WHEN("Result doesn't contain a value")
        {
            try
            {
                resultWithError.ValueOrThrow();
            }
            catch (std::runtime_error&)
            {
                exceptionThrown = true;
            }
            THEN("Exception is thrown") { CHECK(exceptionThrown); }
        }
    }
}
#endif  // __cpp_exceptions == 199711

TEST_CASE("Specialization for value_type=void: Resolve method",
          "[SWS_CORE], [SWS_CORE_00867]")
{
    GIVEN("Result with a value and Result with an error")
    {
        auto resultWithValue = ara::core::Result<void, ErrorTest>::FromValue();
        auto resultWithError =
          ara::core::Result<void, ErrorTest>::FromError(ErrorTest(20));

        // just for coverage
        resultWithValue.Resolve([](ErrorTest const&) { return; });

        // just for coverage
        resultWithError.Resolve([](ErrorTest const&) { return; });
    }
}

TEST_CASE("Compare two Result instances for equality",
          "[SWS_CORE], [SWS_CORE_00780]")
{
    WHEN("Results have the same value")
    {
        THEN("operator== returns true")
        {
            CHECK(ara::core::Result<int, ErrorTest>::FromValue(10)
                  == ara::core::Result<int, ErrorTest>::FromValue(10));
        }
    }

    WHEN("Results have the different value")
    {
        THEN("operator== returns false")
        {
            CHECK(! (ara::core::Result<int, ErrorTest>::FromValue(10)
                     == ara::core::Result<int, ErrorTest>::FromValue(20)));
        }
    }

    WHEN("One Result has a value and second Result has an error")
    {
        THEN("operator== returns false")
        {
            CHECK(! (ara::core::Result<int, ErrorTest>::FromError(10)
                     == ara::core::Result<int, ErrorTest>::FromValue(10)));
        }
    }
}

TEST_CASE("Compare two Result instances for inequality",
          "[SWS_CORE], [SWS_CORE_00781]")
{
    WHEN("Results have the same value")
    {
        THEN("operator!= returns false")
        {
            CHECK(! (ara::core::Result<int, ErrorTest>::FromValue(10)
                     != ara::core::Result<int, ErrorTest>::FromValue(10)));
        }
    }

    WHEN("Results have the different value")
    {
        THEN("operator!= returns true")
        {
            CHECK(ara::core::Result<int, ErrorTest>::FromValue(10)
                  != ara::core::Result<int, ErrorTest>::FromValue(20));
        }
    }

    WHEN("One Result has a value and second Result has an error")
    {
        THEN("operator!= returns true")
        {
            CHECK(ara::core::Result<int, ErrorTest>::FromError(10)
                  != ara::core::Result<int, ErrorTest>::FromValue(10));
        }
    }
}

TEST_CASE(
  "Compare a Result instance for equality to a value",
  "[SWS_CORE], [SWS_CORE_00782], [SWS_CORE_00783], [SWS_CORE_00784], [SWS_CORE_00785]")
{
    WHEN("Results have the same value as value")
    {
        THEN("operator== returns true")
        {
            CHECK(ara::core::Result<int, ErrorTest>::FromValue(10) == 10);
        }
    }

    WHEN("Results with error is compared to a value")
    {
        THEN("operator== returns false")
        {
            CHECK(! (ara::core::Result<int, ErrorTest>::FromError(10) == 10));
        }
    }

    WHEN("Results have the same value as value")
    {
        THEN("operator== returns true")
        {
            CHECK(10 == ara::core::Result<int, ErrorTest>::FromValue(10));
        }
    }

    WHEN("Results with error is compared to a value")
    {
        THEN("operator== returns false")
        {
            CHECK(! (10 == ara::core::Result<int, ErrorTest>::FromError(10)));
        }
    }

    WHEN("Results have the different value as value")
    {
        THEN("operator== returns false")
        {
            CHECK(ara::core::Result<int, ErrorTest>::FromValue(10) != 15);
            CHECK(15 != ara::core::Result<int, ErrorTest>::FromValue(10));
        }
    }

    WHEN("Results with error is compared to a value")
    {
        THEN("operator== returns false")
        {
            CHECK(ara::core::Result<int, ErrorTest>::FromError(10) != 10);
            CHECK(10 != ara::core::Result<int, ErrorTest>::FromError(10));
        }
    }
}

TEST_CASE(
  "Compare a Result instance for equality to an error",
  "[SWS_CORE], [SWS_CORE_00786], [SWS_CORE_00787], [SWS_CORE_00788], [SWS_CORE_00789]")
{
    CHECK(ara::core::Result<int, ErrorTest>::FromError(10) == ErrorTest(10));
    CHECK(
      ! (ara::core::Result<int, ErrorTest>::FromValue(10) == ErrorTest(10)));

    CHECK(ErrorTest(10) == ara::core::Result<int, ErrorTest>::FromError(10));
    CHECK(
      ! (ErrorTest(10) == ara::core::Result<int, ErrorTest>::FromValue(10)));

    CHECK(ara::core::Result<int, ErrorTest>::FromError(10) != ErrorTest(15));
    CHECK(ErrorTest(15) != ara::core::Result<int, ErrorTest>::FromError(10));

    CHECK(ara::core::Result<int, ErrorTest>::FromValue(10) != ErrorTest(10));
    CHECK(ErrorTest(10) != ara::core::Result<int, ErrorTest>::FromValue(10));
}

TEST_CASE("Swap the contents of the two given arguments",
          "[SWS_CORE], [SWS_CORE_00796]")
{
    GIVEN("Two Results with a value and two Results with an error")
    {
        int         value1        = 11;
        int         value2        = 12;
        int         error1        = 21;
        int         error2        = 22;
        std::string valueMessage1 = "Value Message1";
        std::string valueMessage2 = "Value Message2";
        std::string errorMessage1 = "Error Message1";
        std::string errorMessage2 = "Error Message2";

        auto resultWithValue1 =
          ara::core::Result<ValueTest, ErrorTest>::FromValue(
            ValueTest(value1, valueMessage1));
        auto resultWithValue2 =
          ara::core::Result<ValueTest, ErrorTest>::FromValue(
            ValueTest(value2, valueMessage2));
        CHECK(resultWithValue1.HasValue());
        CHECK(resultWithValue2.HasValue());
        CHECK(resultWithValue1.Value().getIntVal() == value1);
        CHECK(resultWithValue2.Value().getIntVal() == value2);
        CHECK(resultWithValue1.Value().getStringVal() == valueMessage1);
        CHECK(resultWithValue2.Value().getStringVal() == valueMessage2);

        auto resultWithError1 =
          ara::core::Result<ValueTest, ErrorTest>::FromError(
            ErrorTest(error1, errorMessage1));
        auto resultWithError2 =
          ara::core::Result<ValueTest, ErrorTest>::FromError(
            ErrorTest(error2, errorMessage2));
        CHECK(! resultWithError1.HasValue());
        CHECK(! resultWithError2.HasValue());
        CHECK(resultWithError1.Error().getIntVal() == error1);
        CHECK(resultWithError2.Error().getIntVal() == error2);
        CHECK(resultWithError1.Error().getStringVal() == errorMessage1);
        CHECK(resultWithError2.Error().getStringVal() == errorMessage2);

        WHEN("Swap a Result1 with a value and Result2 with a value")
        {
            swap(resultWithValue1, resultWithValue2);
            THEN(
              "Result1 has a value from th Result2, and Result2 has a value from the Result1")
            {
                CHECK(resultWithValue1.HasValue());
                CHECK(resultWithValue2.HasValue());
                CHECK(resultWithValue1.Value().getIntVal() == value2);
                CHECK(resultWithValue2.Value().getIntVal() == value1);
                CHECK(resultWithValue1.Value().getStringVal() == valueMessage2);
                CHECK(resultWithValue2.Value().getStringVal() == valueMessage1);
            }
        }

        WHEN("Swap a Result1 with an error and Result2 with an error")
        {
            swap(resultWithError1, resultWithError2);
            THEN(
              "Result1 has an error from th Result2, and Result2 has an error from the Result1")
            {
                CHECK(! resultWithError1.HasValue());
                CHECK(! resultWithError2.HasValue());
                CHECK(resultWithError1.Error().getIntVal() == error2);
                CHECK(resultWithError2.Error().getIntVal() == error1);
                CHECK(resultWithError1.Error().getStringVal() == errorMessage2);
                CHECK(resultWithError2.Error().getStringVal() == errorMessage1);
            }
        }

        WHEN("Swap a Result1 with an error and Result2 with a value")
        {
            swap(resultWithError1, resultWithValue1);
            THEN(
              "Result1 has a value from th Result2, and Result2 has an error from the Result1")
            {
                CHECK(resultWithError1.HasValue());
                CHECK(! resultWithValue1.HasValue());
                CHECK(resultWithError1.Value().getIntVal() == value1);
                CHECK(resultWithValue1.Error().getIntVal() == error1);
                CHECK(resultWithError1.Value().getStringVal() == valueMessage1);
                CHECK(resultWithValue1.Error().getStringVal() == errorMessage1);
            }
        }

        WHEN("Swap a Result1 with a value and Result2 with an error")
        {
            swap(resultWithValue2, resultWithError2);
            THEN(
              "Result1 has an error from th Result2, and Result2 has a value from the Result1")
            {
                CHECK(resultWithError2.HasValue());
                CHECK(! resultWithValue2.HasValue());
                CHECK(resultWithError2.Value().getIntVal() == value2);
                CHECK(resultWithValue2.Error().getIntVal() == error2);
                CHECK(resultWithError2.Value().getStringVal() == valueMessage2);
                CHECK(resultWithValue2.Error().getStringVal() == errorMessage2);
            }
        }
    }
}