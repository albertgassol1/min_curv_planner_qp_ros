#include <iostream>
#include <chrono>
#include <fstream>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <boost/program_options.hpp>

#include "cubic_b_spline.hpp"
#include "cubic_spline.hpp"
#include "curv_min.hpp"

// Add boost program options
namespace po = boost::program_options;

// Constants
double scale = 0.5; // Scale factor for the sine wave
std::size_t num_points_spline = 50;

// Function to generate a curve (left boundary)
std::vector<Eigen::Vector2d> generateLeftBoundary(int num_points) {
    std::vector<Eigen::Vector2d> left_boundary;

    // Let's assume a simple parametric curve: a sine wave
    for (std::size_t i = 0; i < num_points; ++i) {
        const double x = i * scale; // Parameter
        const double y = sin(x);  // y = sin(t), a sine wave
        left_boundary.push_back(Eigen::Vector2d(x, y));
    }
    return left_boundary;
}

// Function to translate the left boundary to create the right boundary
std::vector<Eigen::Vector2d> translateBoundary(const std::vector<Eigen::Vector2d>& boundary, 
                                               const Eigen::Vector2d& translation_vector) {
    std::vector<Eigen::Vector2d> translated_boundary;
    for (const auto& point : boundary) {
        translated_boundary.push_back(point + translation_vector); // Translate each point
    }
    return translated_boundary;
}

// Function to compute the center points (midpoints between left and right boundaries)
std::vector<Eigen::Vector2d> computeCenterPoints(const std::vector<Eigen::Vector2d>& left_boundary, 
                                                 const std::vector<Eigen::Vector2d>& right_boundary) {
    std::vector<Eigen::Vector2d> center_points;
    for (size_t i = 0; i < left_boundary.size(); ++i) {
        Eigen::Vector2d midpoint = 0.5 * (left_boundary[i] + right_boundary[i]);
        center_points.push_back(midpoint);
    }
    return center_points;
}

// Function to save points to a CSV file
void savePointsToFile(const std::string& filename, 
                      const std::vector<Eigen::Vector2d>& left_boundary, 
                      const std::vector<Eigen::Vector2d>& right_boundary, 
                      const std::vector<Eigen::Vector2d>& center_points, 
                      const std::vector<Eigen::Vector2d>& opt_spline,
                      const std::vector<double>& center_curv,
                      const std::vector<double>& opt_curv) {
    std::ofstream file(filename);
    if (file.is_open()) {
        file << "x_left,y_left,x_right,y_right,x_center,y_center,x_opt,y_opt,k_center,k_opt\n";
        for (size_t i = 0; i < left_boundary.size(); ++i) {
            file << left_boundary[i].x() << "," << left_boundary[i].y() << ","
                 << right_boundary[i].x() << "," << right_boundary[i].y() << ","
                 << center_points[i].x() << "," << center_points[i].y() << ","
                 << opt_spline[i].x() << "," << opt_spline[i].y() << "," 
                 << center_curv[i] << "," << opt_curv[i] << "\n";
        }
        file.close();
        std::cout << "Points saved to " << filename << std::endl;
    } else {
        std::cerr << "Error opening file!" << std::endl;
    }
}


void test_min_curvature_optimizer(const double first_opt_weight = 0.5, 
                                  const double last_point_shrink = 0.5,
                                  const std::string& filename = "optimized.csv",
                                  const bool verbose = false) {
    // Initialize some example control points for reference, left, and right boundary B-splines
    Eigen::Vector2d trans(0.0, 2.0); 
    std::vector<Eigen::Vector2d> left_boundary = generateLeftBoundary(num_points_spline);  
    std::vector<Eigen::Vector2d> right_boundary = translateBoundary(left_boundary, trans);
    std::vector<Eigen::Vector2d> ref_points = computeCenterPoints(left_boundary, right_boundary);

    // Create B-Spline objects
    std::shared_ptr<spline::BaseCubicSpline> ref_spline = std::make_shared<spline::ParametricCubicSpline>(ref_points);
    std::shared_ptr<spline::BaseCubicSpline> left_spline = std::make_shared<spline::ParametricCubicSpline>(left_boundary);
    std::shared_ptr<spline::BaseCubicSpline> right_spline = std::make_shared<spline::ParametricCubicSpline>(right_boundary);
    
    // Create and solve optimizer
    std::unique_ptr<spline::optimization::MinCurvatureOptimizer> optimizer = std::make_unique<spline::optimization::MinCurvatureOptimizer>(verbose);
    std::shared_ptr<spline::BaseCubicSpline> opt_traj = std::make_shared<spline::ParametricCubicSpline>(ref_points);
    // Solve twice to get a smoother trajectory
    // Compute solving time in milliseconds
    auto start_solver = std::chrono::high_resolution_clock::now();
    optimizer->setUp(ref_spline, left_spline, right_spline, last_point_shrink);
    optimizer->solve(opt_traj, first_opt_weight);
    optimizer->setUp(opt_traj, left_spline, right_spline, last_point_shrink);
    optimizer->solve(opt_traj, 1.0 - first_opt_weight);
    auto end_solver = std::chrono::high_resolution_clock::now();
    auto duration_solver = std::chrono::duration_cast<std::chrono::milliseconds>(end_solver - start_solver);
    std::cout << "Solving time: " << duration_solver.count() << "ms\n";
    opt_traj = std::make_shared<spline::CubicBSpline>(opt_traj->getControlPoints());
    ref_spline = std::make_shared<spline::CubicBSpline>(ref_spline->getControlPoints());

    std::vector<Eigen::Vector2d> left_points;
    std::vector<Eigen::Vector2d> right_points;
    std::vector<Eigen::Vector2d> opt_points;
    std::vector<Eigen::Vector2d> init_points; 
    std::vector<double> init_curv;
    std::vector<double> opt_curv;

    // Output optimized control points
    for (double u = 0.0; u <= 1.0; u += 0.01) {
        const auto point_ref = ref_spline->evaluateSpline(u, 0);
        const auto point_left = left_spline->evaluateSpline(u, 0);
        const auto point_right = right_spline->evaluateSpline(u, 0);
        const auto point_opt = opt_traj->evaluateSpline(u, 0);
        init_curv.emplace_back(ref_spline->computeCurvature(u));
        opt_curv.emplace_back(opt_traj->computeCurvature(u));

        // Add points to vectors
        left_points.emplace_back(point_left);
        right_points.emplace_back(point_right);
        opt_points.emplace_back(point_opt);
        init_points.emplace_back(point_ref);
    }
    savePointsToFile(filename, left_points, right_points, init_points, opt_points, init_curv, opt_curv);
}


int main(int argc, char* argv[]) {

    // Program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("scale", po::value<double>()->default_value(0.25), "scale factor for the sine wave")
        ("num_points", po::value<std::size_t>()->default_value(20), "number of points for the spline")
        ("first_opt_weight", po::value<double>()->default_value(0.5), "weight for the first optimization")
        ("last_point_shrink", po::value<double>()->default_value(0.5), "shrink factor for the last point")
        ("output", po::value<std::string>()->default_value("optimized.csv"), "output file name")
        ("verbose", po::value<bool>()->default_value(false), "verbose mode")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    scale = vm["scale"].as<double>();
    num_points_spline = vm["num_points"].as<std::size_t>();
    double first_opt_weight = vm["first_opt_weight"].as<double>();
    double last_point_shrink = vm["last_point_shrink"].as<double>();
    bool verbose = vm["verbose"].as<bool>();
    std::string output = vm["output"].as<std::string>();
    // Compute time
    auto start = std::chrono::high_resolution_clock::now();
    test_min_curvature_optimizer(first_opt_weight, last_point_shrink, output, verbose);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Total time: " << duration.count() << "ms\n";
    return 1;
}
