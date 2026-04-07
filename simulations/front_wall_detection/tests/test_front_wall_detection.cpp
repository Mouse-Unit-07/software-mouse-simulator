/*================================ FILE INFO =================================*/
/* Filename           : test_front_wall_detection.cpp                         */
/*                                                                            */
/* Test implementation for front_wall_detection.cpp                           */
/*                                                                            */
/*============================================================================*/

/*============================================================================*/
/*                               Include Files                                */
/*============================================================================*/
extern "C"
{

}

#include <cstdint>
#include <vector>
#include <string>
#include <map>
#include "simulation_common.hpp"
#include "front_wall_detection.hpp"

#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

using namespace front_wall_detection;

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
constexpr double FLOAT_TOLERANCE{1e-6};

ConfigSweeper create_no_variance_sweeper(void)
{
    ConfigSweeper sweeper;

    sweeper.ir_reading_scale = {1.0};
    sweeper.mouse_angle = {0.0};
    sweeper.horizontal_position_variance = {0.0};
    sweeper.vertical_position_variance = {0.0};

    sweeper.reading_threshold = {750u};

    return sweeper;
}

/*============================================================================*/
/*                            Mock Implementations                            */
/*============================================================================*/
/* none */

/*============================================================================*/
/*                                 Test Group                                 */
/*============================================================================*/
TEST_GROUP(FrontWallDetectionTests)
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
TEST(FrontWallDetectionTests, ConfigSweeperProducesFirstValue)
{
    ConfigSweeper sweeper{create_no_variance_sweeper()};

    CHECK(sweeper.next());

    auto cfg{sweeper.value()};

    DOUBLES_EQUAL(1.0, cfg.ir_reading_scale, FLOAT_TOLERANCE);
    CHECK_EQUAL(750u, cfg.reading_threshold);
}

TEST(FrontWallDetectionTests, ConfigSweeperIteratesAllCombinations)
{
    ConfigSweeper sweeper{create_no_variance_sweeper()};
    sweeper.ir_reading_scale = {1.0, 1.05};
    sweeper.reading_threshold = {100u, 200u};

    int count{0};

    while (sweeper.next()) {
        sweeper.value();
        count++;
    }

    CHECK_EQUAL(4, count); /* 2 ir_reading_scale * 2 reading_threshold */
}

TEST(FrontWallDetectionTests, ConfigSweeperStopsAtEnd)
{
    ConfigSweeper sweeper{create_no_variance_sweeper()};

    CHECK(sweeper.next());
    CHECK_FALSE(sweeper.next());
}

TEST(FrontWallDetectionTests, ConfigSweeperOrderIsStable)
{
    ConfigSweeper sweeper{create_no_variance_sweeper()};
    sweeper.ir_reading_scale = {1.0, 1.05};
    sweeper.reading_threshold = {100u, 200u};

    std::vector<std::pair<double, uint32_t>> seen;

    while (sweeper.next()) {
        auto cfg{sweeper.value()};
        seen.emplace_back(cfg.ir_reading_scale, cfg.reading_threshold);
    }

    CHECK_EQUAL(4, seen.size());

    /* Expected order: ir_reading_scale outer, reading_threshold inner */
    CHECK(seen.at(0) == std::make_pair(1.0, 100u));
    CHECK(seen.at(1) == std::make_pair(1.0, 200u));
    CHECK(seen.at(2) == std::make_pair(1.05, 100u));
    CHECK(seen.at(3) == std::make_pair(1.05, 200u));
}
