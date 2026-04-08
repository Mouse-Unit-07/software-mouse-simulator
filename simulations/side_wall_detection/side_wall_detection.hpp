/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : side_wall_detection.hpp                               */
/*                                                                            */
/* Interface to functions to run micromouse side wall detection simulation    */ 
/* and associated config and results analysis helpers                         */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef SIDE_WALL_DETECTION_HPP_
#define SIDE_WALL_DETECTION_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace side_wall_detection
{

struct DetectionWindow
{
    int window_start{-1};
    int window_size{0};
};

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

class ConfigSweeper
{
public:
    std::vector<double> maze_size_scale;
    std::vector<double> ir_reading_scale;
    std::vector<double> mouse_angle;
    std::vector<double> horizontal_position_variance;
    std::vector<double> vertical_position_variance;
    std::vector<int> total_steps;

    std::vector<uint32_t> reading_threshold;

    bool next();
    Config value() const;
private:
    simulation_common::CommonConfigSweeper sweeper;
    bool initialized_{false};
};

struct Result
{
    std::vector<bool> wall_absent_at_step;
    std::vector<bool> wall_present_at_step;
};

struct Trial
{
    Config config;
    Result result;
};

struct ResultsMetrics
{
    DetectionWindow detection_window;
    std::vector<int> correct_detection_count_at_step;
    int total_detection_counts_per_step;
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

} /* side_wall_detection namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace side_wall_detection
{

void enable_visualization(void);
void disable_visualization(void);

std::string config_to_string(const Config& cfg);
Result run_simulation(const Config& cfg);

ResultsMetrics compute_results_metrics(const std::vector<Result>& results);
std::vector<Candidate> build_candidates(const std::vector<Trial>& trials);
std::vector<Candidate> filter_candidates_by_rate(const std::vector<Candidate>& candidates,
        double required_rate);

void write_analysis_to_file(const std::string& filename, const std::vector<Candidate>& candidates,
        size_t total_size, double min_correct_rate);
void run_full_side_wall_detection_experiment(const std::string& filename,
        ConfigSweeper& sweeper, double min_correct_rate);

} /* side_wall_detection namespace */

#endif /* SIDE_WALL_DETECTION_HPP_ */
