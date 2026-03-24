/*================================ FILE INFO =================================*/
/* Filename           : test_optimizer.cpp                                    */
/*                                                                            */
/* Test implementation for optimizer.cpp                                      */
/*                                                                            */
/*============================================================================*/

/*============================================================================*/
/*                               Include Files                                */
/*============================================================================*/
#include <cmath>
#include <vector>
#include <string>
#include <functional>
#include <algorithm>
#include "optimizer.hpp"

#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

using namespace optimizer;

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
constexpr double FLOAT_TOLERANCE {1e-6};

/*============================================================================*/
/*                            Mock Implementations                            */
/*============================================================================*/
/* none */

/*============================================================================*/
/*                                 Test Group                                 */
/*============================================================================*/
TEST_GROUP(OptimizerTests)
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
TEST(OptimizerTests, SweepGeneratesCorrectNumberOfCombinations)
{
    std::vector<SweepConfig> configs
    {
        {"a", 0, 1, 2},
        {"b", 0, 1, 3}
    };

    auto sim_fn {[](const std::vector<double>& vals) {
        return vals; // identity
    }};

    auto results {run_config_sweep<std::vector<double>>(configs, sim_fn)};

    CHECK_EQUAL(6, results.size()); // 2 * 3
}

TEST(OptimizerTests, SweepSingleStepProducesSingleCombination)
{
    std::vector<SweepConfig> configs
    {
        {"a", 5, 5, 1},
        {"b", 10, 10, 1}
    };

    auto sim_fn {[](const std::vector<double>& vals) {
        return vals;
    }};

    auto results {run_config_sweep<std::vector<double>>(configs, sim_fn)};

    CHECK_EQUAL(1, results.size());
}

TEST(OptimizerTests, SweepValuesAreInterpolatedCorrectly)
{
    std::vector<SweepConfig> configs
    {
        {"a", 0.0, 10.0, 3}
    };

    SweepCursor cursor(configs);

    auto v0 {cursor.values()};
    cursor.next();
    auto v1 {cursor.values()};
    cursor.next();
    auto v2 {cursor.values()};

    DOUBLES_EQUAL(0.0, v0[0], FLOAT_TOLERANCE);
    DOUBLES_EQUAL(5.0, v1[0], FLOAT_TOLERANCE);
    DOUBLES_EQUAL(10.0, v2[0], FLOAT_TOLERANCE);
}

TEST(OptimizerTests, SweepPassesCorrectValuesToSimFn)
{
    std::vector<SweepConfig> configs
    {
        {"a", 1, 2, 2}
    };

    std::vector<double> captured;

    auto sim_fn {[&](const std::vector<double>& vals) {
        captured.push_back(vals[0]);
        return 0;
    }};

    run_config_sweep<int>(configs, sim_fn);

    CHECK_EQUAL(2, captured.size());
    CHECK(captured[0] == 1.0);
    CHECK(captured[1] == 2.0);
}

TEST(OptimizerTests, ComputeStatsBasic)
{
    std::vector<double> data {10.0, 20.0, 30.0};

    auto s {compute_stats(data)};

    DOUBLES_EQUAL(20.0, s.mean, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(10.0, s.min, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(30.0, s.max, FLOAT_TOLERANCE);

    double expected_stddev {std::sqrt(66.6666667)};
    DOUBLES_EQUAL(expected_stddev, s.stddev, FLOAT_TOLERANCE);
}

TEST(OptimizerTests, ComputeStatsEmpty)
{
    std::vector<double> data;

    auto s {compute_stats(data)};

    DOUBLES_EQUAL(0.0, s.mean, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.0, s.stddev, FLOAT_TOLERANCE);
}

TEST(OptimizerTests, ExtractMetricWorks)
{
    std::vector<std::pair<int, double>> trials
    {
        {1, 10.0},
        {2, 20.0},
        {3, 30.0}
    };

    auto result {extract_metric(trials,
        [](const auto& t) { return t.second; })};

    CHECK_EQUAL(3, result.size());
    DOUBLES_EQUAL(10.0, result[0], FLOAT_TOLERANCE);
    DOUBLES_EQUAL(30.0, result[2], FLOAT_TOLERANCE);
}

TEST(OptimizerTests, ComputeRateBasic)
{
    std::vector<int> data {0, 1, 0, 1};

    auto rate {compute_rate(data,
        [](int v) { return v == 1; })};

    DOUBLES_EQUAL(0.5, rate, FLOAT_TOLERANCE);
}

TEST(OptimizerTests, ComputeRateEmpty)
{
    std::vector<int> data;

    auto rate {compute_rate(data,
        [](int) { return true; })};

    DOUBLES_EQUAL(0.0, rate, FLOAT_TOLERANCE);
}
