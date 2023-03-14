//
// Copyright 2017-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_MATH_MODULAR_SIMULTANEOUS_CONGRUENCES_SOLVER_TEST_H_
#define CODELIBRARY_TEST_MATH_MODULAR_SIMULTANEOUS_CONGRUENCES_SOLVER_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/math/modular/simultaneous_congruences_solver.h"

namespace cl {
namespace test {

TEST(SimultaneousCongruencesSolverTest, Test) {
    SimultaneousCongruencesSolver<int> solver;
    solver.InsertCongruence(2, 1);
    ASSERT(solver.IsCorrect());
    solver.InsertCongruence(3, 2);
    ASSERT(solver.IsCorrect());
    solver.InsertCongruence(5, 4);
    ASSERT(solver.IsCorrect());
}

} // namespace test
} // namespace cl


#endif // CODELIBRARY_TEST_MATH_MODULAR_SIMULTANEOUS_CONGRUENCES_SOLVER_TEST_H_
