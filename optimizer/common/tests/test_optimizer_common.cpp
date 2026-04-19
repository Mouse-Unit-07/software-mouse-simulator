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

    std::string contents;
    in >> contents;

    /* verify fixed + precision(6) */
    CHECK_EQUAL("1.234568", contents);

    std::remove(filename.c_str());
}

TEST(OptimizerCommonTests, OpenOutputFile_ThrowsOnInvalidPath)
{
    const std::string bad_path{"?:/invalid/test.txt"};

    CHECK_THROWS(std::runtime_error, open_output_file(bad_path));
}
