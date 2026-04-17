#pragma once

#include "baseInclude.h"

using namespace std;

namespace optimization {
    using Vector = vector<double>;
    using Matrix = vector<Vector>;
    using Objective = function<double(const Vector &)>;

    struct Result {
        Vector point;
        double value = numeric_limits<double>::infinity();
        int iterations = 0;
        bool converged = false;
    };

    struct Options {
        int max_iterations = 80;
        double tolerance = 1e-5;
        double finite_difference_step = 1e-4;
        Vector initial_steps;
    };

    double norm(const Vector &values) {
        double accum = 0.0;
        for (double value : values) {
            accum += value * value;
        }
        return sqrt(accum);
    }

    Vector addScaled(const Vector &base, const Vector &delta, double scale) {
        Vector result = base;
        for (size_t i = 0; i < result.size(); ++i) {
            result[i] += delta[i] * scale;
        }
        return result;
    }

    Vector gradient(const Objective &objective, const Vector &point, double h) {
        Vector grad(point.size(), 0.0);
        for (size_t i = 0; i < point.size(); ++i) {
            Vector left = point;
            Vector right = point;
            left[i] -= h;
            right[i] += h;
            grad[i] = (objective(right) - objective(left)) / (2.0 * h);
        }
        return grad;
    }

    Matrix hessian(const Objective &objective, const Vector &point, double h) {
        const size_t n = point.size();
        Matrix hess(n, Vector(n, 0.0));
        const double f0 = objective(point);

        for (size_t i = 0; i < n; ++i) {
            Vector plus = point;
            Vector minus = point;
            plus[i] += h;
            minus[i] -= h;
            const double fPlus = objective(plus);
            const double fMinus = objective(minus);
            hess[i][i] = (fPlus - 2.0 * f0 + fMinus) / (h * h);

            for (size_t j = i + 1; j < n; ++j) {
                Vector pp = point;
                Vector pm = point;
                Vector mp = point;
                Vector mm = point;
                pp[i] += h;
                pp[j] += h;
                pm[i] += h;
                pm[j] -= h;
                mp[i] -= h;
                mp[j] += h;
                mm[i] -= h;
                mm[j] -= h;
                const double mixed = (objective(pp) - objective(pm) - objective(mp) + objective(mm)) / (4.0 * h * h);
                hess[i][j] = mixed;
                hess[j][i] = mixed;
            }
        }

        return hess;
    }

    bool solveLinearSystem(Matrix matrix, Vector rhs, Vector &solution) {
        const size_t n = rhs.size();
        solution.assign(n, 0.0);

        for (size_t col = 0; col < n; ++col) {
            size_t pivot = col;
            double pivotValue = abs(matrix[col][col]);
            for (size_t row = col + 1; row < n; ++row) {
                if (abs(matrix[row][col]) > pivotValue) {
                    pivotValue = abs(matrix[row][col]);
                    pivot = row;
                }
            }

            if (pivotValue < 1e-12) {
                return false;
            }

            if (pivot != col) {
                swap(matrix[pivot], matrix[col]);
                swap(rhs[pivot], rhs[col]);
            }

            const double diagonal = matrix[col][col];
            for (size_t row = col + 1; row < n; ++row) {
                const double factor = matrix[row][col] / diagonal;
                if (abs(factor) < 1e-12) {
                    continue;
                }
                for (size_t inner = col; inner < n; ++inner) {
                    matrix[row][inner] -= factor * matrix[col][inner];
                }
                rhs[row] -= factor * rhs[col];
            }
        }

        for (int row = static_cast<int>(n) - 1; row >= 0; --row) {
            double value = rhs[row];
            for (size_t col = static_cast<size_t>(row) + 1; col < n; ++col) {
                value -= matrix[row][col] * solution[col];
            }
            if (abs(matrix[row][row]) < 1e-12) {
                return false;
            }
            solution[row] = value / matrix[row][row];
        }

        return true;
    }

    Result minimizeNewton(const Objective &objective, const Vector &initial, const Options &options = Options()) {
        Result result;
        result.point = initial;
        result.value = objective(initial);

        for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
            result.iterations = iteration + 1;
            Vector grad = gradient(objective, result.point, options.finite_difference_step);
            if (norm(grad) < options.tolerance) {
                result.converged = true;
                break;
            }

            Matrix hess = hessian(objective, result.point, options.finite_difference_step);
            bool improved = false;

            for (double lambda : {1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1.0}) {
                Matrix regularized = hess;
                for (size_t i = 0; i < regularized.size(); ++i) {
                    regularized[i][i] += lambda;
                }

                Vector rhs = grad;
                for (double &value : rhs) {
                    value = -value;
                }

                Vector step;
                if (!solveLinearSystem(regularized, rhs, step)) {
                    continue;
                }

                double alpha = 1.0;
                while (alpha >= 1e-3) {
                    Vector candidate = addScaled(result.point, step, alpha);
                    const double value = objective(candidate);
                    if (value + 1e-9 < result.value) {
                        result.point = candidate;
                        result.value = value;
                        improved = true;
                        break;
                    }
                    alpha *= 0.5;
                }

                if (improved) {
                    break;
                }
            }

            if (!improved) {
                Vector step = grad;
                for (double &value : step) {
                    value = -value;
                }
                double alpha = 0.25 / max(1.0, norm(step));
                while (alpha >= 1e-4) {
                    Vector candidate = addScaled(result.point, step, alpha);
                    const double value = objective(candidate);
                    if (value + 1e-9 < result.value) {
                        result.point = candidate;
                        result.value = value;
                        improved = true;
                        break;
                    }
                    alpha *= 0.5;
                }
            }

            if (!improved) {
                break;
            }
        }

        if (!result.converged) {
            Vector grad = gradient(objective, result.point, options.finite_difference_step);
            result.converged = norm(grad) < options.tolerance * 5.0;
        }

        return result;
    }

    Result minimizeNelderMead(const Objective &objective, const Vector &initial, const Options &options = Options()) {
        const size_t n = initial.size();
        vector<Vector> simplex(n + 1, initial);
        vector<double> values(n + 1, numeric_limits<double>::infinity());

        for (size_t i = 0; i < n; ++i) {
            simplex[i + 1][i] += options.initial_steps.empty() ? 0.2 : options.initial_steps[i];
        }

        for (size_t i = 0; i < simplex.size(); ++i) {
            values[i] = objective(simplex[i]);
        }

        Result result;
        result.point = simplex[0];
        result.value = values[0];

        auto reorder = [&]() {
            vector<size_t> order(simplex.size());
            for (size_t i = 0; i < order.size(); ++i) {
                order[i] = i;
            }
            sort(order.begin(), order.end(), [&](size_t lhs, size_t rhs) {
                return values[lhs] < values[rhs];
            });

            vector<Vector> newSimplex(simplex.size());
            vector<double> newValues(values.size());
            for (size_t i = 0; i < order.size(); ++i) {
                newSimplex[i] = simplex[order[i]];
                newValues[i] = values[order[i]];
            }
            simplex = newSimplex;
            values = newValues;
            result.point = simplex.front();
            result.value = values.front();
        };

        reorder();

        for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
            result.iterations = iteration + 1;
            reorder();

            Vector centroid(n, 0.0);
            for (size_t vertex = 0; vertex < n; ++vertex) {
                for (size_t dimension = 0; dimension < n; ++dimension) {
                    centroid[dimension] += simplex[vertex][dimension];
                }
            }
            for (double &value : centroid) {
                value /= static_cast<double>(n);
            }

            const Vector &worst = simplex.back();
            Vector reflected(n, 0.0);
            for (size_t i = 0; i < n; ++i) {
                reflected[i] = centroid[i] + (centroid[i] - worst[i]);
            }
            double reflectedValue = objective(reflected);

            if (reflectedValue < values.front()) {
                Vector expanded(n, 0.0);
                for (size_t i = 0; i < n; ++i) {
                    expanded[i] = centroid[i] + 2.0 * (reflected[i] - centroid[i]);
                }
                const double expandedValue = objective(expanded);
                if (expandedValue < reflectedValue) {
                    simplex.back() = expanded;
                    values.back() = expandedValue;
                } else {
                    simplex.back() = reflected;
                    values.back() = reflectedValue;
                }
            } else if (reflectedValue < values[n - 1]) {
                simplex.back() = reflected;
                values.back() = reflectedValue;
            } else {
                Vector contracted(n, 0.0);
                if (reflectedValue < values.back()) {
                    for (size_t i = 0; i < n; ++i) {
                        contracted[i] = centroid[i] + 0.5 * (reflected[i] - centroid[i]);
                    }
                } else {
                    for (size_t i = 0; i < n; ++i) {
                        contracted[i] = centroid[i] + 0.5 * (worst[i] - centroid[i]);
                    }
                }
                const double contractedValue = objective(contracted);
                if (contractedValue < values.back()) {
                    simplex.back() = contracted;
                    values.back() = contractedValue;
                } else {
                    for (size_t vertex = 1; vertex < simplex.size(); ++vertex) {
                        for (size_t dimension = 0; dimension < n; ++dimension) {
                            simplex[vertex][dimension] =
                                simplex.front()[dimension] + 0.5 * (simplex[vertex][dimension] - simplex.front()[dimension]);
                        }
                        values[vertex] = objective(simplex[vertex]);
                    }
                }
            }

            reorder();

            double spread = 0.0;
            for (size_t vertex = 1; vertex < simplex.size(); ++vertex) {
                spread = max(spread, norm(addScaled(simplex[vertex], simplex.front(), -1.0)));
            }
            double valueSpread = 0.0;
            for (size_t vertex = 1; vertex < values.size(); ++vertex) {
                valueSpread = max(valueSpread, abs(values[vertex] - values.front()));
            }

            if (spread < options.tolerance && valueSpread < options.tolerance) {
                result.converged = true;
                break;
            }
        }

        if (!result.converged) {
            reorder();
            result.converged = true;
        }

        return result;
    }
}
