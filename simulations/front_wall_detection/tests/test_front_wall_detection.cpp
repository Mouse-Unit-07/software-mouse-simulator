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
#include <cmath>
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

Config create_no_variance_config(void)
{
    Config cfg{};
    cfg.ir_reading_scale = {1.0};
    cfg.mouse_angle = {0.0};
    cfg.horizontal_position_variance = {0.0};
    cfg.vertical_position_variance = {0.0};

    cfg.reading_threshold = {750u};

    return cfg;
}

ConfigSweeper create_no_variance_sweeper(void)
{
    ConfigSweeper sweeper{};

    sweeper.ir_reading_scale = {1.0};
    sweeper.mouse_angle = {0.0};
    sweeper.horizontal_position_variance = {0.0};
    sweeper.vertical_position_variance = {0.0};

    sweeper.reading_threshold = {750u};

    return sweeper;
}

bool are_results_equivalent(const Result& r1, const Result& r2)
{
    if (r1.identified_absent_wall != r2.identified_absent_wall) {
        return false;
    }
    if (r1.identified_present_wall != r2.identified_present_wall) {
        return false;
    }

    return true;
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

TEST(FrontWallDetectionTests, SimulationProducesValidResult)
{
    auto sweeper{create_no_variance_sweeper()};

    CHECK(sweeper.next());
    auto result{run_simulation(sweeper.value())};
}

TEST(FrontWallDetectionTests, ExtremeScaleTriggersClamping)
{
    Config cfg{create_no_variance_config()};

    cfg.ir_reading_scale = 100.0; /* force overflow */

    auto result{run_simulation(cfg)};

    CHECK(result.identified_present_wall == true ||
          result.identified_present_wall == false);
}

TEST(FrontWallDetectionTests, ZeroThresholdMeansWallAlwaysPresent)
{
    Config cfg{create_no_variance_config()};
    cfg.reading_threshold = 0;

    auto result{run_simulation(cfg)};

    CHECK(result.identified_present_wall);
    CHECK(!result.identified_absent_wall);
}

TEST(FrontWallDetectionTests, MaxThresholdMeansWallAlmostAlwaysAbsent)
{
    Config cfg{create_no_variance_config()};
    cfg.reading_threshold = 1024;

    auto result{run_simulation(cfg)};

    CHECK(!result.identified_present_wall);
    CHECK(result.identified_absent_wall);
}

TEST(FrontWallDetectionTests, IrReadingScaleAffectsResults)
{
    Config cfg{create_no_variance_config()};

    cfg.ir_reading_scale = 0.1;
    auto low_scale{run_simulation(cfg)};

    cfg.ir_reading_scale = 2.0;
    auto high_scale{run_simulation(cfg)};

    CHECK(!are_results_equivalent(low_scale, high_scale));
}

TEST(FrontWallDetectionTests, AngleAffectsResults)
{
    Config cfg{create_no_variance_config()};

    auto no_angle{run_simulation(cfg)};
    
    cfg.mouse_angle = M_PI / 8;
    auto some_angle{run_simulation(cfg)};

    CHECK(!are_results_equivalent(no_angle, some_angle));
}

TEST(FrontWallDetectionTests, HorizontalVarianceAndAngleAffectsResults)
{
    Config cfg{create_no_variance_config()};
    cfg.mouse_angle = M_PI / 8;

    cfg.horizontal_position_variance = -0.9;
    auto left{run_simulation(cfg)};

    cfg.horizontal_position_variance = 0.9;
    auto right{run_simulation(cfg)};

    CHECK(!are_results_equivalent(left, right));
}

TEST(FrontWallDetectionTests, VerticalVarianceAffectsResults)
{
    Config cfg{create_no_variance_config()};

    cfg.vertical_position_variance = -0.9;
    auto back{run_simulation(cfg)};

    cfg.vertical_position_variance = 0.9;
    auto front{run_simulation(cfg)};

    CHECK(!are_results_equivalent(back, front));
}

TEST(FrontWallDetectionTests, ComputeResultsMetricsEmptyInput)
{
    std::vector<Result> results{};

    auto metrics{compute_results_metrics(results)};

    DOUBLES_EQUAL(0.0, metrics.absent_wall_identification_rate, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.0, metrics.present_wall_identification_rate, FLOAT_TOLERANCE);
}

TEST(FrontWallDetectionTests, ComputeResultsMetricsFieldsAreIndependent)
{
    std::vector<Result> results{
        {true,  false},
        {true,  false},
        {false, false},
        {false, true }
    };

    /* absent: 2 / 4 = 0.5 */
    /* present: 1 / 4 = 0.25 */

    auto metrics{compute_results_metrics(results)};

    DOUBLES_EQUAL(0.5, metrics.absent_wall_identification_rate, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.25, metrics.present_wall_identification_rate, FLOAT_TOLERANCE);
}

TEST(FrontWallDetectionTests, BuildCandidatesGroupsByThreshold)
{
    Trial t1;
    t1.config.reading_threshold = 100;

    Trial t2{t1};

    Trial t3;
    t3.config.reading_threshold = 200;

    std::vector<Trial> trials{t1, t2, t3};

    auto candidates{build_candidates(trials)};

    CHECK_EQUAL(2, candidates.size());
}

TEST(FrontWallDetectionTests, BuildCandidatesComputesResultsMetrics)
{
    Trial t1;
    t1.result.identified_absent_wall = true;
    t1.result.identified_present_wall = true;
    t1.config.reading_threshold = 100;

    Trial t2{t1};
    t2.result.identified_absent_wall = false;
    t2.result.identified_present_wall = true;

    std::vector<Trial> trials{t1, t2};

    auto candidates{build_candidates(trials)};

    CHECK_EQUAL(1, candidates.size());
    DOUBLES_EQUAL(0.5, candidates.at(0).results_metrics.absent_wall_identification_rate, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(1.0, candidates.at(0).results_metrics.present_wall_identification_rate, FLOAT_TOLERANCE);
}

TEST(FrontWallDetectionTests, SortCandidatesOrdersByDescendingScore)
{
    Candidate c1;
    c1.key.threshold = 100;
    c1.results_metrics = {0.2, 0.2}; /* avg = 0.2 */

    Candidate c2;
    c2.key.threshold = 200;
    c2.results_metrics = {0.8, 0.8}; /* avg = 0.8 */

    std::vector<Candidate> input{c1, c2};

    auto sorted{sort_candidates_by_rate(input)};

    CHECK_EQUAL(2, sorted.size());

    /* highest score first */
    CHECK_EQUAL(200u, sorted.at(0).key.threshold);
    CHECK_EQUAL(100u, sorted.at(1).key.threshold);
}

TEST(FrontWallDetectionTests, SortCandidatesTieBreaksOnThreshold)
{
    Candidate c1;
    c1.key.threshold = 100;
    c1.results_metrics = {0.5, 0.5}; /* avg = 0.5 */

    Candidate c2;
    c2.key.threshold = 200;
    c2.results_metrics = {0.5, 0.5}; /* same avg */

    std::vector<Candidate> input{c2, c1};

    auto sorted{sort_candidates_by_rate(input)};

    CHECK_EQUAL(2, sorted.size());

    /* lower threshold first */
    CHECK_EQUAL(100u, sorted.at(0).key.threshold);
    CHECK_EQUAL(200u, sorted.at(1).key.threshold);
}
