/*================================ FILE INFO =================================*/
/* Filename           : test_optimizer_common.cpp                             */
/*                                                                            */
/* Test implementation for optimizer_common.cpp                               */
/*                                                                            */
/*============================================================================*/

/*============================================================================*/
/*                               Include Files                                */
/*============================================================================*/
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include <fstream>
#include <vector>
#include "optimizer_common.hpp"

using namespace optimizer_common;

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
/* none */

/*============================================================================*/
/*                            Mock Implementations                            */
/*============================================================================*/
/* none */

/*============================================================================*/
/*                                 Test Group                                 */
/*============================================================================*/
TEST_GROUP(OptimizerCommonTests)
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
TEST(OptimizerCommonTests, OpenOutputFileCreatesFileAndAppliesFormatting)
{
    const std::string filename{"test_output.txt"};

    {
        auto out{open_output_file(filename)};
        out << 1.23456789;
    }

    std::ifstream in(filename);
    CHECK(in.is_open());

    std::string contents{};
    in >> contents;

    /* verify fixed + precision(6) */
    STRCMP_EQUAL("1.234568", contents.c_str());

    std::remove(filename.c_str());
}

TEST(OptimizerCommonTests, OpenOutputFile_ThrowsOnInvalidPath)
{
    const std::string bad_path{"?:/invalid/test.txt"};

    CHECK_THROWS(std::runtime_error, open_output_file(bad_path));
}

TEST(OptimizerCommonTests, ControlToKeyBasicFormatting)
{
    const std::vector<double> x{1.0, 2.5, 3.123456789};

    const auto key{control_to_key(x)};

    STRCMP_EQUAL("1.000000,2.500000,3.123457,", key.c_str());
}

TEST(OptimizerCommonTests, ControlToKeyDifferentInputsProduceDifferentKeys)
{
    const std::vector<double> a{1.0, 2.0, 3.0};
    const std::vector<double> b{1.0, 2.0, 3.000001};

    const auto key_a{control_to_key(a)};
    const auto key_b{control_to_key(b)};

    CHECK(key_a != key_b);
}

TEST(OptimizerCommonTests, ControlToKeyEmptyVector)
{
    const std::vector<double> x{};

    const auto key{control_to_key(x)};

    STRCMP_EQUAL("", key.c_str());
}

TEST(OptimizerCommonTests, ControlToKeyRoundingBehavior)
{
    const std::vector<double> x{1.1234564};
    const std::vector<double> y{1.1234565};

    const auto key_x{control_to_key(x)};
    const auto key_y{control_to_key(y)};

    /* 6th decimal rounding */
    CHECK(key_x != key_y);
}
