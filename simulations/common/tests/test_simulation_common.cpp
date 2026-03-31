/*================================ FILE INFO =================================*/
/* Filename           : test_simulation_common.cpp                            */
/*                                                                            */
/* Test implementation for simulation_common.cpp                              */
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
#include "simulation_common.hpp"

#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

using namespace simulation_common;

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
TEST(CommonTests, GenerateSweepValuesReturnsEmptyForInvalidSteps)
{
    auto v0 {generate_sweep_values(0, 10, 0)};
    auto vNeg {generate_sweep_values(0, 10, -5)};

    CHECK(v0.empty());
    CHECK(vNeg.empty());
}

TEST(CommonTests, GenerateSweepValuesSingleStep)
{
    auto v {generate_sweep_values(5, 10, 1)};

    CHECK_EQUAL(1, v.size());
    CHECK_EQUAL(5, v[0]);
}

TEST(CommonTests, GenerateSweepValuesTwoSteps)
{
    auto v {generate_sweep_values(5, 10, 2)};

    CHECK_EQUAL(2, v.size());
    CHECK_EQUAL(5, v[0]);
    CHECK_EQUAL(10, v[1]);
}

TEST(CommonTests, GenerateSweepValuesInterpolatesCorrectly)
{
    auto v {generate_sweep_values(0.0, 10.0, 5)};

    CHECK_EQUAL(5, v.size());
    DOUBLES_EQUAL(0.0,  v[0], FLOAT_TOLERANCE);
    DOUBLES_EQUAL(5.0,  v[2], FLOAT_TOLERANCE);
    DOUBLES_EQUAL(10.0, v[4], FLOAT_TOLERANCE);
}

TEST(CommonTests, GenerateSweepValuesIntegerTypeTruncates)
{
    auto v {generate_sweep_values<int>(0, 10, 4)};

    CHECK_EQUAL(4, v.size());
    CHECK_EQUAL(0, v[0]);
    CHECK_EQUAL(3, v[1]); /* truncated */
    CHECK_EQUAL(6, v[2]); /* truncated */
    CHECK_EQUAL(10, v[3]);
}

TEST(CommonTests, ComputeStatsSingleValue)
{
    std::vector<double> data {42.0};

    auto s {compute_stats(data)};

    DOUBLES_EQUAL(42.0, s.mean, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(42.0, s.min, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(42.0, s.max, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.0,  s.stddev, FLOAT_TOLERANCE);
}

TEST(CommonTests, ComputeStatsBasic)
{
    std::vector<double> data {10.0, 20.0, 30.0};

    auto s {compute_stats(data)};

    DOUBLES_EQUAL(20.0, s.mean, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(10.0, s.min, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(30.0, s.max, FLOAT_TOLERANCE);

    double expected_stddev {std::sqrt(66.6666667)};
    DOUBLES_EQUAL(expected_stddev, s.stddev, FLOAT_TOLERANCE);
}

TEST(CommonTests, ComputeStatsEmpty)
{
    std::vector<double> data;

    auto s {compute_stats(data)};

    DOUBLES_EQUAL(0.0, s.mean, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.0, s.stddev, FLOAT_TOLERANCE);
}

TEST(CommonTests, ComputeStatsAllSameValues)
{
    std::vector<double> data {5.0, 5.0, 5.0};

    auto s {compute_stats(data)};

    DOUBLES_EQUAL(5.0, s.mean, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.0, s.stddev, FLOAT_TOLERANCE);
}

TEST(CommonTests, ExtractMetricEmpty)
{
    std::vector<int> trials;

    auto result {extract_metric(trials, [](int v) { return static_cast<double>(v); })};

    CHECK(result.empty());
}

TEST(CommonTests, ExtractMetricWorks)
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

TEST(CommonTests, ComputeRateBasic)
{
    std::vector<int> data {0, 1, 0, 1};

    auto rate {compute_rate(data,
        [](int v) { return v == 1; })};

    DOUBLES_EQUAL(0.5, rate, FLOAT_TOLERANCE);
}

TEST(CommonTests, ComputeRateEmpty)
{
    std::vector<int> data;

    auto rate {compute_rate(data,
        [](int) { return true; })};

    DOUBLES_EQUAL(0.0, rate, FLOAT_TOLERANCE);
}

TEST(CommonTests, ComputeRateAllTrue)
{
    std::vector<int> data {1, 1, 1};

    auto rate {compute_rate(data, [](int v) { return v == 1; })};

    DOUBLES_EQUAL(1.0, rate, FLOAT_TOLERANCE);
}

TEST(CommonTests, ComputeRateAllFalse)
{
    std::vector<int> data {0, 0, 0};

    auto rate {compute_rate(data, [](int v) { return v == 1; })};

    DOUBLES_EQUAL(0.0, rate, FLOAT_TOLERANCE);
}
