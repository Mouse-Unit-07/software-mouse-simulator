/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : front_wall_detection.hpp                              */
/*                                                                            */
/* Interface to functions to run micromouse front wall detection simulation   */ 
/* and associated config and results analysis helpers                         */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef FRONT_WALL_DETECTION_HPP_
#define FRONT_WALL_DETECTION_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace front_wall_detection
{

struct Config
{
    double ir_reading_scale;
    double mouse_angle;
    double horizontal_position_variance;
    double vertical_position_variance;

    uint32_t reading_threshold;
};

class ConfigSweeper
{
public:
    std::vector<double> ir_reading_scale;
    std::vector<double> mouse_angle;
    std::vector<double> horizontal_position_variance;
    std::vector<double> vertical_position_variance;

    std::vector<uint32_t> reading_threshold;

    bool next();
    Config value() const;
private:
    simulation_common::CommonConfigSweeper sweeper;
    bool initialized_{false};
};

struct Result
{
    bool identified_absent_wall;
    bool identified_present_wall;
};

struct Trial
{
    Config config;
    Result result;
};

struct ResultsMetrics
{
    double absent_wall_identification_rate{0.0};
    double present_wall_identification_rate{0.0};
};

struct CandidateKey
{
    uint32_t threshold;

    bool operator<(const CandidateKey& other) const {
        return threshold < other.threshold;
    }
};

struct Candidate
{
    CandidateKey key;
    ResultsMetrics results_metrics;
};

} /* front_wall_detection namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace front_wall_detection
{

Result run_simulation(const Config& cfg);

ResultsMetrics compute_results_metrics(const std::vector<Result>& results);
std::vector<Candidate> build_candidates(const std::vector<Trial>& trials);

} /* front_wall_detection namespace */

#endif /* FRONT_WALL_DETECTION_HPP_ */
