/*================================ FILE INFO =================================*/
/* Filename           : test_simulation_common.cpp                            */
/*                                                                            */
/* Test implementation for simulation_common.cpp                              */
/*                                                                            */
/*============================================================================*/

/*============================================================================*/
/*                               Include Files                                */
/*============================================================================*/
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include <string>
#include "simulation_common.hpp"

using namespace simulation_common;

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
constexpr double FLOAT_TOLERANCE{1e-6};

/*============================================================================*/
/*                            Mock Implementations                            */
/*============================================================================*/
/* none */

/*============================================================================*/
/*                                 Test Group                                 */
/*============================================================================*/
TEST_GROUP(CommonTests)
{
    void setup() override
    {
        
    }

    void teardown() override
    {
        
    }
};

/*============================================================================*/
/*                                    Tests                                   */
/*============================================================================*/
TEST(CommonTests, DoubleToFilenameBasic)
{
    std::string s{double_to_filename(1.23)};
    STRCMP_EQUAL("1p23", s.c_str());
}

TEST(CommonTests, DoubleToFilenameNegative)
{
    std::string s{double_to_filename(-1.23)};
    STRCMP_EQUAL("n1p23", s.c_str());
}

TEST(CommonTests, DoubleToFilenamePrecision)
{
    std::string s{double_to_filename(1.2345, 3)};
    STRCMP_EQUAL("1p234", s.c_str()); /* truncated */
}

TEST(CommonTests, DoubleToFilenameNoDecimal)
{
    std::string s{double_to_filename(5.0, 0)};
    STRCMP_EQUAL("5", s.c_str());
}

TEST(CommonTests, DoubleToFilenameSubOneLeadingZeros)
{
    std::string s{double_to_filename(0.00123, 3)};
    STRCMP_EQUAL("p123", s.c_str());
}

TEST(CommonTests, DoubleToFilenameSubOneMorePrecision)
{
    std::string s{double_to_filename(0.00004567, 4)};
    STRCMP_EQUAL("p4567", s.c_str());
}

TEST(CommonTests, DoubleToFilenameZero)
{
    std::string s{double_to_filename(0.0, 3)};
    STRCMP_EQUAL("0", s.c_str());
}

TEST(CommonTests, DoubleToFilenameNegativeSubOne)
{
    std::string s{double_to_filename(-0.0123, 3)};
    STRCMP_EQUAL("np123", s.c_str());
}

TEST(CommonTests, DoubleToFilenameIntegerWithFractionIgnored)
{
    std::string s{double_to_filename(12.3400, 3)};
    STRCMP_EQUAL("12p34", s.c_str());
}
