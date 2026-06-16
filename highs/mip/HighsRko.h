/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                       */
/*    This file is part of the HiGHS linear optimization suite           */
/*                                                                       */
/*    Available as open-source under the MIT License                     */
/*                                                                       */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef HIGHS_RKO_H_
#define HIGHS_RKO_H_

#include "lp_data/HighsLp.h"

const HighsInt kRkoNumRandomKeyPasses = 64;
const HighsInt kRkoHashMultiplier = 1103515245;
const HighsInt kRkoHashIncrement = 12345;
const HighsInt kRkoHashMask = 0x7fffffff;
const HighsInt kRkoRandomKeyModulus = 10000;
const double kRkoRandomKeyBaseWeight = 0.75;
const double kRkoRandomKeyRangeWeight = 0.5;
const double kRkoFeasibilityTolerance = 1e-9;

struct RkoKnapsackData {
  HighsInt num_item = 0;
  double capacity = 0.0;
  std::vector<double> profit;
  std::vector<double> weight;
};

bool rkoHeuristic(const HighsLp* lp, std::vector<double>& solution);
bool setupKnapsackProblem(const HighsLp* lp, RkoKnapsackData& knapsack);
void searchKnapsackProblem(const RkoKnapsackData& knapsack,
                           std::vector<double>& solution);

#endif
