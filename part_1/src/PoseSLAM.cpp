#include <fstream>
#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {
  // Create a factor graph container and add factors to it
  NonlinearFactorGraph graph;

  // A prior factor consists of a mean and a noise model (covariance matrix)
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.3, 0.3, 0.1));
  // Add a prior on the first pose, setting it to the origin
  graph.add(PriorFactor<Pose2>(1, Pose2(0, 0, 0), priorNoise));

  // For simplicity, we will use the same noise model for odometry and loop closures
  noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas((Vector(3) << 0.2, 0.2, 0.1));

  // Create the data structure to hold the initial estimate to the solution
  // For illustrative purposes, these have been deliberately set to incorrect values
  Values initial;

  // Insert the following initial estimate values into the initial estimate datastructure
  // Use initial.insert()
  // 0.5, 0.0, 0.2
  // 2.3, 0.1, -0.2
  // 4.1, 0.1, M_PI_2
  // 4.0, 2.0, M_PI
  // 2.1, 2.1, -M_PI_2

  // Insert the following odometry values, graph edges, into the factor graph container
  // Use graph.add()
  // Describing the edge with a BetweenFactor<Pose2>() object
  // 2, 0, 0
  // 2, 0, M_PI_2
  // 2, 0, M_PI_2
  // 2, 0, M_PI_2
  // 2, 0, M_PI_2

  // Assign the non optimized values from the LevenbergMarquardtOptimizer to a Values type
  // Call Values type non_optimized_result
  // Output non optimized graph to dot graph file 
  ofstream non_optimized_dot_file("NonOptimizedGraph.dot");
  graph.saveGraph(non_optimized_dot_file, non_optimized_result);

  // Assign the optimized values from the LevenbergMarquardtOptimizer to a Values type
  // Call Values type optimized_result
  // Output optimized graph to dot graph file 
  ofstream optimized_dot_file("OptimizedGraph.dot");
  graph.saveGraph(optimized_dot_file, optimized_result);

  return 0;
}
