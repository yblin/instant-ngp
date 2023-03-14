//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef MATH_DERIVATIVE_H_
#define MATH_DERIVATIVE_H_

#include "codelibrary/base/array.h"
#include "codelibrary/math/number/dual_number.h"

namespace cl {

/**
 * Compute derivative of the function on the given paramters.
 *
 * ------------------------------------------------------
 * Sample usage [1]:
 * ------------------------------------------------------
 *
 *  struct Function {
 *  public:
 *      template <typename T>
 *      T operator()(const T& x, const T& y) const {
 *          return x * x + x * y;
 *      }
 *  };
 *
 *  Function function;
 *  LOG(INFO) << Derivative(&function, 1.0, 2.0)[0];
 *
 * ------------------------------------------------------
 * Sample usage [2]:
 * ------------------------------------------------------
 *
 *  struct Function {
 *      template <typename T>
 *      T operator()(const Array<T>& x) const {
 *          return x[0] * x[0] + x[0] * x[1];
 *      }
 *  };
 *
 *  Function function;
 *  Array<double> x = {1.0, 2.0};
 *  LOG(INFO) << Derivative(&function, x)[0];
 *
 * ------------------------------------------------------
 * Sample usage [3]:
 * ------------------------------------------------------
 *
 *  template <typename T>
 *  T Function(const T& x, const T& y) {
 *      return x * x + x * y;
 *  }
 *
 *  LOG(INFO) << Derivative(&Function<DualNumber<double>>, 1.0, 2.0)[0];
 *
 *
 * ----------------------------------------------------------------------
 * This class can also get the second-order derivatives, for example:
 *
 * struct CostFunction {
 *     template <typename T>
 *     T operator()(const T& x, const T& y) const {
 *         return x * x + x * y;
 *     }
 * };
 *
 * CostFunction function;
 *
 * The following code will get the Partial Derivative of d^2f / dxdy
 * Derivative(&function,
 *            DualNumber<double>(1.0, 1.0),
 *            DualNumber<double>(2.0, 0.0))[1].dual;
 *
 *
 * However, it is not encouraged to use this method to obtain the second
 * derivative, because the efficiency is too low. The best way is to manually
 * obtain the analytical solution.
 */
template <typename T, class Function, int D = 0>
class DerivativeSystem {
    static_assert(D >= 0, "");

    using Dual = DualNumber<T>;

    template <int N, class Func>
    struct Evaluator;

    template <class Func>
    struct Evaluator<0, Func> {
        Dual operator()(Func* f, const Array<Dual>& x) const {
            return (*f)(x);
        }
    };
    template <class Func>
    struct Evaluator<1, Func> {
        Dual operator()(Func* f, const Array<Dual>& x) const {
            return (*f)(x[0]);
        }
    };
    template <class Func>
    struct Evaluator<2, Func> {
        Dual operator()(Func* f, const Array<Dual>& x) const {
            return (*f)(x[0], x[1]);
        }
    };
    template <class Func>
    struct Evaluator<3, Func> {
        Dual operator()(Func* f, const Array<Dual>& x) const {
            return (*f)(x[0], x[1], x[2]);
        }
    };
    template <class Func>
    struct Evaluator<4, Func> {
        Dual operator()(Func* f, const Array<Dual>& x) const {
            return (*f)(x[0], x[1], x[2], x[3]);
        }
    };
    template <class Func>
    struct Evaluator<5, Func> {
        Dual operator()(Func* f, const Array<Dual>& x) const {
            return (*f)(x[0], x[1], x[2], x[3], x[4]);
        }
    };
    template <class Func>
    struct Evaluator<6, Func> {
        Dual operator()(Func* f, const Array<Dual>& x) const {
            return (*f)(x[0], x[1], x[2], x[3], x[4], x[5]);
        }
    };
    template <class Func>
    struct Evaluator<7, Func> {
        Dual operator()(Func* f, const Array<Dual>& x) const {
            return (*f)(x[0], x[1], x[2], x[3], x[4], x[5], x[6]);
        }
    };
    template <class Func>
    struct Evaluator<8, Func> {
        Dual operator()(Func* f, const Array<Dual>& x) const {
            return (*f)(x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7]);
        }
    };
    template <class Func>
    struct Evaluator<9, Func> {
        Dual operator()(Func* f, const Array<Dual>& x) const {
            return (*f)(x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8]);
        }
    };

public:
    DerivativeSystem(Function* function, const Array<T>& value)
        : function_(function), values_(value) {
        CHECK(function_);
        if (D > 0) CHECK(values_.size() == D);
    }

    /**
     * Return the first-order partial derivative value of the i-th paramter,
     * i.e., df/dx_i
     */
    T operator[] (int i) const {
        CHECK(i >= 0 && i < values_.size());

        Array<Dual> duals(values_.begin(), values_.end());
        duals[i].dual = T(1);
        Evaluator<D, Function> evaluator;
        return evaluator(function_, duals).dual;
    }

private:
    Function* function_ = nullptr;
    Array<T> values_;
};

template <typename T, class Func>
DerivativeSystem<T, Func, 0> Derivative(Func* function, const Array<T>& a) {
    return DerivativeSystem<T, Func, 0>(function, a);
}
template <typename T, class Func>
DerivativeSystem<T, Func, 1> Derivative(Func* function, T a) {
    return DerivativeSystem<T, Func, 1>(function, {a});
}
template <typename T, class Func>
DerivativeSystem<T, Func, 2> Derivative(Func* function, T a, T b) {
    return DerivativeSystem<T, Func, 2>(function, {a, b});
}
template <typename T, class Func>
DerivativeSystem<T, Func, 3> Derivative(Func* function, T a, T b, T c) {
    return DerivativeSystem<T, Func, 3>(function, {a, b, c});
}
template <typename T, class Func>
DerivativeSystem<T, Func, 4> Derivative(Func* function, T a, T b, T c, T d) {
    return DerivativeSystem<T, Func, 4>(function, {a, b, c, d});
}
template <typename T, class Func>
DerivativeSystem<T, Func, 5> Derivative(Func* function, T a, T b, T c, T d,
                                         T e) {
    return DerivativeSystem<T, Func, 5>(function, {a, b, c, d, e});
}
template <typename T, class Func>
DerivativeSystem<T, Func, 6> Derivative(Func* function, T a, T b, T c, T d,
                                         T e, T f) {
    return DerivativeSystem<T, Func, 6>(function, {a, b, c, d, e, f});
}
template <typename T, class Func>
DerivativeSystem<T, Func, 7> Derivative(Func* function, T a, T b, T c, T d,
                                         T e, T f, T g) {
    return DerivativeSystem<T, Func, 7>(function, {a, b, c, d, e, f, g});
}
template <typename T, class Func>
DerivativeSystem<T, Func, 8> Derivative(Func* function, T a, T b, T c, T d,
                                         T e, T f, T g, T h) {
    return DerivativeSystem<T, Func, 8>(function, {a, b, c, d, e, f, g, h});
}
template <typename T, class Func>
DerivativeSystem<T, Func, 9> Derivative(Func* function, T a, T b, T c, T d,
                                         T e, T f, T g, T h, T i) {
    return DerivativeSystem<T, Func, 9>(function, {a, b, c, d, e, f, g, h, i});
}

} // namespace cl

#endif // MATH_DERIVATIVE_H_
