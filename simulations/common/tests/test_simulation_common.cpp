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
#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <string>
#include <vector>
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
TEST(CommonTests, SweeperRejectsZeroSize)
{
    CommonConfigSweeper sweeper;

    CHECK_THROWS(std::invalid_argument, sweeper.init_sizes({3, 0, 2}));
}

TEST(CommonTests, SweeperSingleDimension)
{
    CommonConfigSweeper sweeper;
    sweeper.init_sizes({3});

    std::vector<size_t> expected{0};

    CHECK(sweeper.next());
    CHECK_EQUAL(expected.at(0), sweeper.get_indices().at(0));

    expected.at(0) = 1;
    CHECK(sweeper.next());
    CHECK_EQUAL(expected.at(0), sweeper.get_indices().at(0));

    expected.at(0) = 2;
    CHECK(sweeper.next());
    CHECK_EQUAL(expected.at(0), sweeper.get_indices().at(0));

    CHECK_FALSE(sweeper.next());
}

TEST(CommonTests, SweeperMultiDimensionIteration)
{
    CommonConfigSweeper sweeper;
    sweeper.init_sizes({2, 3});

    std::vector<std::vector<size_t>> expected{
        {0,0}, {0,1}, {0,2},
        {1,0}, {1,1}, {1,2}
    };

    size_t idx{0};

    while (sweeper.next()) {
        const auto& indices{sweeper.get_indices()};
        CHECK_EQUAL(expected.at(idx).at(0), indices.at(0));
        CHECK_EQUAL(expected.at(idx).at(1), indices.at(1));
        idx++;
    }

    CHECK_EQUAL(expected.size(), idx);
}

TEST(CommonTests, SweeperFirstCallReturnsInitialIndices)
{
    CommonConfigSweeper sweeper;
    sweeper.init_sizes({2, 2});

    CHECK(sweeper.next());

    const auto& indices{sweeper.get_indices()};
    CHECK_EQUAL(0, indices.at(0));
    CHECK_EQUAL(0, indices.at(1));
}

TEST(CommonTests, GenerateSweepValuesReturnsEmptyForInvalidSteps)
{
    auto v0{generate_sweep_values(0, 10, 0)};
    auto vNeg{generate_sweep_values(0, 10, -5)};

    CHECK(v0.empty());
    CHECK(vNeg.empty());
}

TEST(CommonTests, GenerateSweepValuesSingleStep)
{
    auto v{generate_sweep_values(5, 10, 1)};

    CHECK_EQUAL(1, v.size());
    CHECK_EQUAL(5, v.at(0));
}

TEST(CommonTests, GenerateSweepValuesTwoSteps)
{
    auto v{generate_sweep_values(5, 10, 2)};

    CHECK_EQUAL(2, v.size());
    CHECK_EQUAL(5, v.at(0));
    CHECK_EQUAL(10, v.at(1));
}

TEST(CommonTests, GenerateSweepValuesInterpolatesCorrectly)
{
    auto v{generate_sweep_values(0.0, 10.0, 5)};

    CHECK_EQUAL(5, v.size());
    DOUBLES_EQUAL(0.0, v.at(0), FLOAT_TOLERANCE);
    DOUBLES_EQUAL(5.0, v.at(2), FLOAT_TOLERANCE);
    DOUBLES_EQUAL(10.0, v.at(4), FLOAT_TOLERANCE);
}

TEST(CommonTests, GenerateSweepValuesIntegerTypeTruncates)
{
    auto v{generate_sweep_values<int>(0, 10, 4)};

    CHECK_EQUAL(4, v.size());
    CHECK_EQUAL(0, v.at(0));
    CHECK_EQUAL(3, v.at(1)); /* truncated */
    CHECK_EQUAL(6, v.at(2)); /* truncated */
    CHECK_EQUAL(10, v.at(3));
}

TEST(CommonTests, ComputeStatsSingleValue)
{
    std::vector<double> data{42.0};

    auto s{compute_stats(data)};

    DOUBLES_EQUAL(42.0, s.mean, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(42.0, s.min, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(42.0, s.max, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.0, s.stddev, FLOAT_TOLERANCE);
}

TEST(CommonTests, ComputeStatsBasic)
{
    std::vector<double> data{10.0, 20.0, 30.0};

    auto s{compute_stats(data)};

    DOUBLES_EQUAL(20.0, s.mean, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(10.0, s.min, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(30.0, s.max, FLOAT_TOLERANCE);

    double expected_stddev{std::sqrt(66.6666667)};
    DOUBLES_EQUAL(expected_stddev, s.stddev, FLOAT_TOLERANCE);
}

TEST(CommonTests, ComputeStatsEmpty)
{
    std::vector<double> data;

    auto s{compute_stats(data)};

    DOUBLES_EQUAL(0.0, s.mean, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.0, s.stddev, FLOAT_TOLERANCE);
}

TEST(CommonTests, ComputeStatsAllSameValues)
{
    std::vector<double> data{5.0, 5.0, 5.0};

    auto s{compute_stats(data)};

    DOUBLES_EQUAL(5.0, s.mean, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.0, s.stddev, FLOAT_TOLERANCE);
}

TEST(CommonTests, ComputeStatsNegativeValues)
{
    std::vector<double> data{-10.0, -20.0, -30.0};

    auto s{compute_stats(data)};

    DOUBLES_EQUAL(-20.0, s.mean, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(-30.0, s.min, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(-10.0, s.max, FLOAT_TOLERANCE);
}

TEST(CommonTests, ComputeStatsMixedValues)
{
    std::vector<double> data{-10.0, 0.0, 10.0};

    auto s{compute_stats(data)};

    DOUBLES_EQUAL(0.0, s.mean, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(-10.0, s.min, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(10.0, s.max, FLOAT_TOLERANCE);
}

TEST(CommonTests, ExtractMetricPreservesOrder)
{
    std::vector<int> trials{3, 1, 2};

    auto result{extract_metric(trials, [](int v) { return static_cast<double>(v); })};

    CHECK_EQUAL(3, result.at(0));
    CHECK_EQUAL(1, result.at(1));
    CHECK_EQUAL(2, result.at(2));
}

TEST(CommonTests, ExtractMetricEmpty)
{
    std::vector<int> trials;

    auto result{extract_metric(trials, [](int v) { return static_cast<double>(v); })};

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

    auto result{extract_metric(trials, [](const auto& t) { return t.second; })};

    CHECK_EQUAL(3, result.size());
    DOUBLES_EQUAL(10.0, result.at(0), FLOAT_TOLERANCE);
    DOUBLES_EQUAL(30.0, result.at(2), FLOAT_TOLERANCE);
}

TEST(CommonTests, ComputeRateBasic)
{
    std::vector<int> data{0, 1, 0, 1};

    auto rate{compute_rate(data, [](int v) { return v == 1; })};

    DOUBLES_EQUAL(0.5, rate, FLOAT_TOLERANCE);
}

TEST(CommonTests, ComputeRateEmpty)
{
    std::vector<int> data;

    auto rate{compute_rate(data, [](int) { return true; })};

    DOUBLES_EQUAL(0.0, rate, FLOAT_TOLERANCE);
}

TEST(CommonTests, ComputeRateAllTrue)
{
    std::vector<int> data{1, 1, 1};

    auto rate{compute_rate(data, [](int v) { return v == 1; })};

    DOUBLES_EQUAL(1.0, rate, FLOAT_TOLERANCE);
}

TEST(CommonTests, ComputeRateAllFalse)
{
    std::vector<int> data{0, 0, 0};

    auto rate{compute_rate(data, [](int v) { return v == 1; })};

    DOUBLES_EQUAL(0.0, rate, FLOAT_TOLERANCE);
}

TEST(CommonTests, ComputeRatePartial)
{
    std::vector<int> data{1, 2, 3, 4};

    auto rate{compute_rate(data, [](int v) { return v > 2; })};

    DOUBLES_EQUAL(0.5, rate, FLOAT_TOLERANCE);
}

TEST(CommonTests, GroupByBasic)
{
    struct Item {
        int key;
        int value;
    };

    std::vector<Item> data{
        {1, 10},
        {2, 20},
        {1, 30},
        {2, 40}
    };

    auto grouped = group_by(
        data,
        [](const Item& i) { return i.key; },
        [](const Item& i) { return i.value; }
    );

    CHECK_EQUAL(2, grouped[1].size());
    CHECK_EQUAL(10, grouped[1][0]);
    CHECK_EQUAL(30, grouped[1][1]);

    CHECK_EQUAL(2, grouped[2].size());
    CHECK_EQUAL(20, grouped[2][0]);
    CHECK_EQUAL(40, grouped[2][1]);
}

TEST(CommonTests, GroupBySingleGroup)
{
    std::vector<int> data{1, 2, 3};

    auto grouped = group_by(
        data,
        [](int) { return 0; },
        [](int v) { return v; }
    );

    CHECK_EQUAL(3, grouped[0].size());
}

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

TEST(CommonTests, CollapseMetricBasic)
{
    MetricStats s{};
    s.mean = 1.0;
    s.stddev = 2.0;
    s.min = 3.0;
    s.max = 4.0;

    double result{collapse_metric(s)};

    DOUBLES_EQUAL(10.0, result, FLOAT_TOLERANCE);
}

TEST(CommonTests, CollapseMetricAllZero)
{
    MetricStats s{};

    double result{collapse_metric(s)};

    DOUBLES_EQUAL(0.0, result, FLOAT_TOLERANCE);
}

TEST(CommonTests, CollapseMetricNegativeValues)
{
    MetricStats s{};
    s.mean = -1.0;
    s.stddev = -2.0;
    s.min = -3.0;
    s.max = -4.0;

    double result{collapse_metric(s)};

    DOUBLES_EQUAL(-10.0, result, FLOAT_TOLERANCE);
}

TEST(CommonTests, SafeNormBasic)
{
    double result{safe_norm(5.0, 10.0)};
    DOUBLES_EQUAL(0.5, result, FLOAT_TOLERANCE);
}

TEST(CommonTests, SafeNormZeroMax)
{
    double result{safe_norm(5.0, 0.0)};
    DOUBLES_EQUAL(0.0, result, FLOAT_TOLERANCE);
}

TEST(CommonTests, SafeNormNearZeroMax)
{
    double result{safe_norm(5.0, 1e-8)};
    DOUBLES_EQUAL(0.0, result, FLOAT_TOLERANCE);
}

TEST(CommonTests, SafeNormZeroValue)
{
    double result{safe_norm(0.0, 10.0)};
    DOUBLES_EQUAL(0.0, result, FLOAT_TOLERANCE);
}

TEST(CommonTests, ComputeMetricScoreBasic)
{
    double result{compute_metric_score(2.0, 4.0)};
    /* norm = 0.5, /4 = 0.125 */
    DOUBLES_EQUAL(0.125, result, FLOAT_TOLERANCE);
}

TEST(CommonTests, ComputeMetricScoreZeroMax)
{
    double result{compute_metric_score(2.0, 0.0)};
    DOUBLES_EQUAL(0.0, result, FLOAT_TOLERANCE);
}

TEST(CommonTests, ComputeMetricScoreFullScale)
{
    double result{compute_metric_score(4.0, 4.0)};
    /* norm = 1, /4 = 0.25 */
    DOUBLES_EQUAL(0.25, result, FLOAT_TOLERANCE);
}