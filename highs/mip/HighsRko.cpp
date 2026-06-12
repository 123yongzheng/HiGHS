/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                       */
/*    This file is part of the HiGHS linear optimization suite           */
/*                                                                       */
/*    Available as open-source under the MIT License                     */
/*                                                                       */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "mip/HighsRko.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

struct RkoKnapsackData {
  HighsInt num_item = 0;
  double capacity = 0.0;
  std::vector<double> profit;
  std::vector<double> weight;
};

bool setupKnapsackProblem(const HighsLp* lp, RkoKnapsackData& knapsack) {
  if (lp->mip_type_ != kMipTypeKnapsack) return false;
  if (lp->num_col_ <= 0 || lp->num_row_ != 1) return false;

  const HighsSparseMatrix& matrix = lp->a_matrix_;
  assert(matrix.isColwise());
  assert(HighsInt(matrix.start_.size()) == lp->num_col_ + 1);

  knapsack.num_item = lp->num_col_;
  knapsack.capacity = lp->row_upper_[0];
  if (knapsack.capacity >= kHighsInf && lp->row_lower_[0] > -kHighsInf)
    knapsack.capacity = -lp->row_lower_[0];
  if (knapsack.capacity < 0 || knapsack.capacity >= kHighsInf) return false;

  knapsack.weight.assign(knapsack.num_item, 0.0);
  for (HighsInt iCol = 0; iCol < knapsack.num_item; ++iCol) {
    for (HighsInt iEl = matrix.start_[iCol]; iEl < matrix.start_[iCol + 1];
         ++iEl) {
      if (matrix.index_[iEl] == 0) knapsack.weight[iCol] = matrix.value_[iEl];
    }
  }

  HighsInt num_negative_cost = 0;
  for (HighsInt iCol = 0; iCol < knapsack.num_item; ++iCol) {
    if (lp->col_cost_[iCol] < 0) ++num_negative_cost;
  }

  const bool costs_are_negated = num_negative_cost > knapsack.num_item / 2;
  knapsack.profit.assign(knapsack.num_item, 0.0);
  for (HighsInt iCol = 0; iCol < knapsack.num_item; ++iCol) {
    knapsack.profit[iCol] =
        costs_are_negated ? -lp->col_cost_[iCol] : lp->col_cost_[iCol];
  }
  return true;
}

void searchKnapsackProblem(const RkoKnapsackData& knapsack,
                           std::vector<double>& solution) {
  const HighsInt num_item = knapsack.num_item;
  std::vector<double> best_solution(num_item, 0.0);
  double best_profit = 0.0;

  auto tryKeys = [&](const std::vector<double>& key) {
    std::vector<HighsInt> order(num_item);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(),
              [&](const HighsInt a, const HighsInt b) {
                if (key[a] == key[b]) return a < b;
                return key[a] > key[b];
              });

    std::vector<double> candidate_solution(num_item, 0.0);
    double used_capacity = 0.0;
    double candidate_profit = 0.0;
    for (const HighsInt iCol : order) {
      if (knapsack.profit[iCol] <= 0 || knapsack.weight[iCol] <= 0) continue;
      if (used_capacity + knapsack.weight[iCol] <=
          knapsack.capacity + kRkoFeasibilityTolerance) {
        candidate_solution[iCol] = 1.0;
        used_capacity += knapsack.weight[iCol];
        candidate_profit += knapsack.profit[iCol];
      }
    }
    if (candidate_profit > best_profit + kRkoFeasibilityTolerance) {
      best_profit = candidate_profit;
      best_solution = candidate_solution;
    }
  };

  std::vector<double> key(num_item, 0.0);

  for (HighsInt iCol = 0; iCol < num_item; ++iCol)
    key[iCol] = knapsack.weight[iCol] > 0
                    ? knapsack.profit[iCol] / knapsack.weight[iCol]
                    : 0.0;
  tryKeys(key);

  for (HighsInt iCol = 0; iCol < num_item; ++iCol)
    key[iCol] = knapsack.profit[iCol];
  tryKeys(key);

  for (HighsInt pass = 0; pass < kRkoNumRandomKeyPasses; ++pass) {
    for (HighsInt iCol = 0; iCol < num_item; ++iCol) {
      const HighsInt hash =
          (kRkoHashMultiplier * (iCol + 1) + kRkoHashIncrement * (pass + 1)) &
          kRkoHashMask;
      const double random_key =
          double(hash % kRkoRandomKeyModulus) / kRkoRandomKeyModulus;
      const double density = knapsack.weight[iCol] > 0
                                 ? knapsack.profit[iCol] / knapsack.weight[iCol]
                                 : 0.0;
      key[iCol] =
          density * (kRkoRandomKeyBaseWeight +
                     kRkoRandomKeyRangeWeight * random_key);
    }
    tryKeys(key);
  }

  solution = best_solution;
}

// This method should return true if an integer feasible solution
// (returned as solution) has been found
bool rkoHeuristic(const HighsLp* lp, std::vector<double>& solution) {
  if (lp->mip_type_ != kMipTypeKnapsack) return false;
  printf("Calling the RKO heuristic for a knapsack problem with %d items\n",
         int(lp->num_col_));

  RkoKnapsackData knapsack;
  if (!setupKnapsackProblem(lp, knapsack)) return false;
  searchKnapsackProblem(knapsack, solution);
  return true;
}
