/*================================ FILE INFO =================================*/
/* Filename           : test_repeat_hello_world.cpp                           */
/*                                                                            */
/* Test implementation for repeat_hello_world_cpp.cpp                         */
/*                                                                            */
/*============================================================================*/

/*============================================================================*/
/*                               Include Files                                */
/*============================================================================*/
#include "repeat_hello_world_cpp.hpp"
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
/* none */

/*============================================================================*/
/*                            Mock Implementations                            */
/*============================================================================*/
extern "C" void print_hello_world(void)
{
    mock().actualCall("print_hello_world");
}

/*============================================================================*/
/*                                 Test Group                                 */
/*============================================================================*/
TEST_GROUP(RepeatHelloWorldTests)
{
    void setup() override
    {
        mock().clear();
    }

    void teardown() override
    {
        mock().checkExpectations();
        mock().clear();
    }
};

/*============================================================================*/
/*                                    Tests                                   */
/*============================================================================*/
TEST(RepeatHelloWorldTests, CallsPrintTwice)
{
    mock().expectNCalls(2, "print_hello_world");
    print_hello_world_twice();
}
