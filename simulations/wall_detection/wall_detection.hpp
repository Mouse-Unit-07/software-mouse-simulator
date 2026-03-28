/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : wall_detection.hpp                                    */
/*                                                                            */
/* Interface to functions to run micromouse wall detection simulation and     */ 
/* associated config and results analysis helpers                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef WALL_DETECTION_HPP_
#define WALL_DETECTION_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace wall_detection
{

struct Config
{
    double maze_size_scale;
    double ir_reading_scale;
    double mouse_angle;
    double horizontal_position_variance;
    double vertical_position_variance;
    int total_steps;

    uint32_t reading_threshold;
};

struct Result
{
    std::vector<bool> correct_detection_at_step;
};

struct Trial
{
    Config config;
    Result result;
};

struct ResultsMetrics
{
    int window_start {-1};
    int window_size {0};
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

} /* wall_detection namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace wall_detection
{

void enable_visualization(void);
void disable_visualization(void);

Config build_config(const std::vector<double>& v);
std::string config_to_string(const Config& cfg);
Result run_simulation(const Config& cfg);

ResultsMetrics compute_results_metrics(const std::vector<Result>& results);
std::vector<Candidate> build_candidates(const std::vector<Trial>& trials);
void sort_candidates_by_lowest_threshold(std::vector<Candidate>& candidates);

void write_analysis_to_file(const std::string& filename,
        const std::vector<Candidate>& sorted_candidates, size_t total_size);
void run_full_wall_detection_experiment(const std::string& filename,
        std::vector<simulation_common::SweepConfig> configs);

} /* wall_detection namespace */

#endif /* WALL_DETECTION_HPP_ */
