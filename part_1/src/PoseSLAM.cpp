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
  initial.insert(1, Pose2(0.5, 0.0, 0.2));
  initial.insert(2, Pose2(2.3, 0.1, -0.2));
  initial.insert(3, Pose2(4.1, 0.1, 1.6));
  initial.insert(4, Pose2(4.0, 2.0, 3.1));
  initial.insert(5, Pose2(2.1, 2.1, -1.5));
  // 0.5, 0.0, 0.2
  // 2.3, 0.1, -0.2
  // 4.1, 0.1, M_PI_2
  // 4.0, 2.0, M_PI
  // 2.1, 2.1, -M_PI_2

  // Insert the following odometry values, graph edges, into the factor graph container
  // Use graph.add()
  // Describing the edge with a BetweenFactor<Pose2>() object
  Pose2 odometry(2.0, 0.0, 0.0);
  graph.add(BetweenFactor<Pose2>(1, 2, odometry, model));
  //Pose2 odometry(2.0, 0.0, M_PI/2);
  graph.add(BetweenFactor<Pose2>(2, 3, Pose2(2.0, 0.0, M_PI_2), model));
  graph.add(BetweenFactor<Pose2>(3, 4, Pose2(2.0, 0.0, M_PI_2), model));
  graph.add(BetweenFactor<Pose2>(4, 5, Pose2(2.0, 0.0, M_PI_2), model));
  graph.add(BetweenFactor<Pose2>(5, 2, Pose2(1.98, -0.6, 1.6), model));
  // 2, 0, 0
  // 2, 0, M_PI_2
  // 2, 0, M_PI_2
  // 2, 0, M_PI_2
  // 2, 0, M_PI_2

  Values non_optimized_result = LevenbergMarquardtOptimizer(graph, initial).values();
  // Assign the non optimized values from the LevenbergMarquardtOptimizer to a Values type
  // Call Values type non_optimized_result
  // Output non optimized graph to dot graph file 
  ofstream non_optimized_dot_file("NonOptimizedGraph.dot");
  graph.saveGraph(non_optimized_dot_file, non_optimized_result);

  Values optimized_result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  // Assign the optimized values from the LevenbergMarquardtOptimizer to a Values type
  // Call Values type optimized_result
  // Output optimized graph to dot graph file 
  ofstream optimized_dot_file("OptimizedGraph.dot");
  graph.saveGraph(optimized_dot_file, optimized_result);

  //Marginal covariances
  cout.precision(2);
  Marginals marginals(graph, optimized_result);
  cout << "Node 1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  cout << "Node 2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  cout << "Node 3 covariance:\n" << marginals.marginalCovariance(3) << endl;
  cout << "Node 4 covariance:\n" << marginals.marginalCovariance(4) << endl;
  cout << "Node 5 covariance:\n" << marginals.marginalCovariance(5) << endl;

  return 0;
}
