#include <iostream>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/container/vector.hpp>

namespace ba = boost::accumulators;

int main() {
    // Example time series data
    boost::container::vector<double> time_series = {1.0, 2.5, 3.8, 4.2, 5.1};

    // Define an accumulator set to calculate mean and standard deviation
    ba::accumulator_set<double, ba::stats<ba::mean, ba::variance>> acc;

    // Add data to the accumulator
    for (const auto& value : time_series) {
        acc(value);
    }

    // Compute the standard deviation
    double variance = ba::variance(acc);
    double standard_deviation = std::sqrt(variance);

    // Output the results
    std::cout << "Mean: " << ba::mean(acc) << std::endl;
    std::cout << "Standard Deviation: " << standard_deviation << std::endl;

    return 0;
}
