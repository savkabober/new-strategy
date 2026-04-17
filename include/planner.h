#pragma once

#include "Point.h"
#include "Timer.h"
#include "newton.h"
#include "numAux.h"
#include "quickrandom.h"

using namespace std;

namespace strategy {
    constexpr double kSceneHalfWidth = 4500.0;
    constexpr double kSceneHalfHeight = 3000.0;
    constexpr double kDefaultObstacleRadius = 320.0;
    constexpr int kDefaultSegmentCount = 4;
    constexpr int kDefaultMaxSegmentCount = 8;

    enum class OptimizerKind {
        Newton,
        NelderMead
    };

    string optimizerName(OptimizerKind optimizer) {
        return optimizer == OptimizerKind::Newton ? "newton" : "nelder-mead";
    }

    struct RobotState {
        Point position;
        Point velocity;
    };

    struct Obstacle {
        Point center;
        double radius = kDefaultObstacleRadius;
    };

    struct Scenario {
        string name = "demo";
        RobotState start;
        RobotState goal;
        vector<Obstacle> obstacles;
        int segmentCount = kDefaultSegmentCount;
        int maxSegmentCount = kDefaultMaxSegmentCount;
        bool adaptiveSegments = true;
        double obstacleRadius = kDefaultObstacleRadius;
    };

    struct ControlSegment {
        double angle = 0.0;
        double duration = 0.0;
    };

    struct SegmentTrace {
        RobotState begin;
        RobotState end;
        Point acceleration;
        double duration = 0.0;
    };

    struct TrajectoryMetrics {
        double positionError = 0.0;
        double velocityError = 0.0;
        double obstacleTime = 0.0;
        double totalTime = 0.0;
        double speedPenalty = 0.0;
        double objective = numeric_limits<double>::infinity();
    };

    struct Trajectory {
        vector<ControlSegment> controls;
        vector<SegmentTrace> segments;
        vector<Point> samples;
        RobotState finish;
        TrajectoryMetrics metrics;
    };

    struct PlannerConfig {
        int segmentCount = kDefaultSegmentCount;
        int maxSegmentCount = kDefaultMaxSegmentCount;
        bool adaptiveSegments = true;
        int samplesPerSegment = 20;
        double maxAcceleration = MAX_ACC;
        double maxSpeed = MAX_SPEED;
        double minSegmentDuration = 0.05;
        double maxSegmentDuration = 4.0;
        double positionTolerance = 150.0;
        double velocityTolerance = 120.0;
        double positionWeight = 40.0;
        double velocityWeight = 18.0;
        double timeWeight = 0.75;
        double obstacleWeight = 2200.0;
        double speedWeight = 12.0;
        double segmentImprovementThreshold = 0.03;
        optimization::Options newton;
        optimization::Options nelderMead;
    };

    struct SolveResult {
        Scenario scenario;
        OptimizerKind optimizer = OptimizerKind::NelderMead;
        Trajectory trajectory;
        vector<double> parameters;
        double optimizationSeconds = 0.0;
        int objectiveEvaluations = 0;
        bool converged = false;
        string renderPath;
    };

    struct TestSummary {
        vector<SolveResult> results;
        double averageOptimizationMs = 0.0;
        double maxOptimizationMs = 0.0;
        int underTenMs = 0;
    };

    struct ProgressSnapshot {
        Scenario scenario;
        OptimizerKind optimizer = OptimizerKind::NelderMead;
        Trajectory currentTrajectory;
        Trajectory bestTrajectory;
        int evaluations = 0;
        double elapsedSeconds = 0.0;
        bool improved = false;
        bool final = false;
        bool converged = false;
        string stage = "optimization";
    };

    using ProgressCallback = function<bool(const ProgressSnapshot &)>;

    class OptimizationAborted final : public exception {
        public:
        const char *what() const noexcept override {
            return "optimization aborted";
        }
    };

    Point directionFromAngle(double angle, double magnitude = 1.0) {
        return Point(cos(angle) * magnitude, sin(angle) * magnitude, 0.0);
    }

    double sigmoid(double value) {
        if (value >= 0.0) {
            const double expValue = exp(-value);
            return 1.0 / (1.0 + expValue);
        }
        const double expValue = exp(value);
        return expValue / (1.0 + expValue);
    }

    double encodeDuration(double duration, const PlannerConfig &config) {
        const double span = config.maxSegmentDuration - config.minSegmentDuration;
        const double clamped = clamp((duration - config.minSegmentDuration) / span, 1e-5, 1.0 - 1e-5);
        return log(clamped / (1.0 - clamped));
    }

    double decodeDuration(double raw, const PlannerConfig &config) {
        return config.minSegmentDuration +
               (config.maxSegmentDuration - config.minSegmentDuration) * sigmoid(raw);
    }

    double lineDistanceToPoint(const Point &start, const Point &finish, const Point &point) {
        const Point delta = finish - start;
        const double len2 = delta.mag2();
        if (len2 < 1e-9) {
            return (point - start).mag();
        }
        const double projection = clamp(((point - start) ^ delta) / len2, 0.0, 1.0);
        const Point closest = start + delta * projection;
        return (point - closest).mag();
    }

    const Obstacle *findBlockingObstacle(const Scenario &scenario) {
        const Point path = scenario.goal.position - scenario.start.position;
        if (path.mag2() < 1e-9) {
            return nullptr;
        }
        const Obstacle *best = nullptr;
        double bestDistance = numeric_limits<double>::infinity();
        for (const Obstacle &obstacle : scenario.obstacles) {
            const double distance = lineDistanceToPoint(scenario.start.position, scenario.goal.position, obstacle.center);
            if (distance < obstacle.radius * 1.4 && distance < bestDistance) {
                bestDistance = distance;
                best = &obstacle;
            }
        }
        return best;
    }

    vector<double> encodeControls(const vector<ControlSegment> &controls, const PlannerConfig &config) {
        vector<double> parameters;
        parameters.reserve(controls.size() * 2);
        for (const ControlSegment &control : controls) {
            parameters.push_back(control.angle);
        }
        for (const ControlSegment &control : controls) {
            parameters.push_back(encodeDuration(control.duration, config));
        }
        return parameters;
    }

    vector<ControlSegment> decodeControls(const vector<double> &parameters, const PlannerConfig &config) {
        vector<ControlSegment> controls(config.segmentCount);
        for (int i = 0; i < config.segmentCount; ++i) {
            controls[i].angle = numAux::windDownAngle(parameters[i]);
            controls[i].duration = decodeDuration(parameters[config.segmentCount + i], config);
        }
        return controls;
    }

    double estimateTravelTime(const Scenario &scenario, const PlannerConfig &config) {
        const double distance = (scenario.goal.position - scenario.start.position).mag();
        const double speedMismatch = (scenario.goal.velocity - scenario.start.velocity).mag();
        const double accelTime = sqrt(max(0.0, distance / max(1.0, config.maxAcceleration))) * 1.8;
        const double cruiseTime = distance / max(200.0, config.maxSpeed * 0.65);
        const double velocityTime = speedMismatch / max(1.0, config.maxAcceleration);
        return clamp(max({1.0, accelTime, cruiseTime, velocityTime}) + 0.35,
                     config.minSegmentDuration * config.segmentCount,
                     config.maxSegmentDuration * config.segmentCount * 0.9);
    }

    vector<vector<double>> makeInitialSeeds(const Scenario &scenario, const PlannerConfig &config) {
        const Point displacement = scenario.goal.position - scenario.start.position;
        const Point velocityDelta = scenario.goal.velocity - scenario.start.velocity;
        double primary = displacement.mag() > 1e-6 ? displacement.arg() : velocityDelta.arg();
        if (displacement.mag() <= 1e-6 && velocityDelta.mag() <= 1e-6) {
            primary = 0.0;
        }
        const double reverse = numAux::windDownAngle(primary + M_PI);
        const double velocityAngle = velocityDelta.mag() > 1e-6 ? velocityDelta.arg() : primary;
        double detourLeft = numAux::windDownAngle(primary + M_PI / 2.0);
        double detourRight = numAux::windDownAngle(primary - M_PI / 2.0);

        if (const Obstacle *blocking = findBlockingObstacle(scenario)) {
            const double cross = ((scenario.goal.position - scenario.start.position) *
                                  (blocking->center - scenario.start.position)).z;
            if (cross >= 0.0) {
                detourLeft = numAux::windDownAngle(primary - M_PI / 2.0);
                detourRight = numAux::windDownAngle(primary + M_PI / 2.0);
            }
        }

        const double totalTime = estimateTravelTime(scenario, config);
        const double baseDuration = totalTime / static_cast<double>(config.segmentCount);

        auto seedFromPattern = [&](const vector<double> &angles, const vector<double> &durations) {
            vector<ControlSegment> controls(config.segmentCount);
            for (int i = 0; i < config.segmentCount; ++i) {
                controls[i].angle = angles[i];
                controls[i].duration = clamp(durations[i], config.minSegmentDuration, config.maxSegmentDuration);
            }
            return encodeControls(controls, config);
        };

        vector<double> straightAngles(config.segmentCount, primary);
        vector<double> reverseAngles(config.segmentCount, reverse);
        vector<double> times(config.segmentCount, baseDuration);
        straightAngles.back() = reverse;
        reverseAngles.front() = primary;
        if (config.segmentCount > 2) {
            reverseAngles[config.segmentCount / 2] = velocityAngle;
        }

        vector<double> detourPattern = straightAngles;
        vector<double> detourPatternAlt = straightAngles;
        detourPattern[config.segmentCount / 2] = detourLeft;
        detourPatternAlt[config.segmentCount / 2] = detourRight;

        vector<double> velocityPattern = straightAngles;
        velocityPattern[config.segmentCount / 2] = velocityAngle;

        vector<double> aggressiveTimes = times;
        vector<double> detourTimes = times;
        aggressiveTimes.front() *= 1.15;
        aggressiveTimes.back() *= 0.85;
        detourTimes[config.segmentCount / 2] *= 1.3;
        if (config.segmentCount > 3) {
            detourPattern[config.segmentCount / 2 - 1] = primary;
            detourPatternAlt[config.segmentCount / 2 - 1] = primary;
            velocityPattern[config.segmentCount / 2 - 1] = primary;
        }

        vector<vector<double>> seeds;
        seeds.push_back(seedFromPattern(straightAngles, times));
        seeds.push_back(seedFromPattern(detourPattern, detourTimes));
        seeds.push_back(seedFromPattern(detourPatternAlt, detourTimes));
        seeds.push_back(seedFromPattern(velocityPattern, aggressiveTimes));
        seeds.push_back(seedFromPattern(reverseAngles, times));
        return seeds;
    }

    Point segmentPositionAt(const SegmentTrace &segment, double time) {
        return segment.begin.position + segment.begin.velocity * time + segment.acceleration * (0.5 * time * time);
    }

    Point segmentVelocityAt(const SegmentTrace &segment, double time) {
        return segment.begin.velocity + segment.acceleration * time;
    }

    double polynomialValue(const array<double, 5> &coeffs, double value) {
        double result = 0.0;
        for (double coefficient : coeffs) {
            result = result * value + coefficient;
        }
        return result;
    }

    double timeInsideObstacle(const SegmentTrace &segment, const Obstacle &obstacle) {
        const Point relative = segment.begin.position - obstacle.center;
        const Point velocity = segment.begin.velocity;
        const Point acceleration = segment.acceleration;

        array<double, 5> coeffs = {
            0.25 * (acceleration ^ acceleration),
            velocity ^ acceleration,
            (velocity ^ velocity) + (relative ^ acceleration),
            2.0 * (relative ^ velocity),
            (relative ^ relative) - obstacle.radius * obstacle.radius
        };

        double roots[4] = {};
        const int rootCount = numAux::solveFour(roots, coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4]);
        vector<double> cuts = {0.0, segment.duration};
        for (int i = 0; i < rootCount; ++i) {
            if (roots[i] > 1e-7 && roots[i] < segment.duration - 1e-7) {
                cuts.push_back(roots[i]);
            }
        }

        sort(cuts.begin(), cuts.end());
        cuts.erase(unique(cuts.begin(), cuts.end(), [](double lhs, double rhs) {
            return abs(lhs - rhs) < 1e-6;
        }), cuts.end());

        double total = 0.0;
        for (size_t i = 0; i + 1 < cuts.size(); ++i) {
            const double midpoint = 0.5 * (cuts[i] + cuts[i + 1]);
            if (polynomialValue(coeffs, midpoint) <= 0.0) {
                total += cuts[i + 1] - cuts[i];
            }
        }
        return total;
    }

    double speedPenalty(const SegmentTrace &segment, const PlannerConfig &config) {
        const double acceleration2 = segment.acceleration ^ segment.acceleration;
        vector<double> candidates = {0.0, segment.duration};
        if (acceleration2 > 1e-9) {
            const double extremum = -((segment.begin.velocity ^ segment.acceleration) / acceleration2);
            if (extremum > 0.0 && extremum < segment.duration) {
                candidates.push_back(extremum);
            }
        }

        double worstExcess = 0.0;
        for (double time : candidates) {
            const double speed = segmentVelocityAt(segment, time).mag();
            worstExcess = max(worstExcess, max(0.0, speed - config.maxSpeed));
        }
        return (worstExcess / 100.0) * (worstExcess / 100.0) * segment.duration;
    }

    Trajectory simulateTrajectory(const Scenario &scenario,
                                  const vector<ControlSegment> &controls,
                                  const PlannerConfig &config) {
        Trajectory trajectory;
        trajectory.controls = controls;
        trajectory.finish = scenario.start;
        trajectory.samples.push_back(scenario.start.position);

        Point position = scenario.start.position;
        Point velocity = scenario.start.velocity;

        for (const ControlSegment &control : controls) {
            SegmentTrace segment;
            segment.begin.position = position;
            segment.begin.velocity = velocity;
            segment.acceleration = directionFromAngle(control.angle, config.maxAcceleration);
            segment.duration = control.duration;
            segment.end.position = position + velocity * control.duration +
                                   segment.acceleration * (0.5 * control.duration * control.duration);
            segment.end.velocity = velocity + segment.acceleration * control.duration;
            trajectory.segments.push_back(segment);

            for (int sample = 1; sample <= config.samplesPerSegment; ++sample) {
                const double localTime = control.duration * static_cast<double>(sample) /
                                         static_cast<double>(config.samplesPerSegment);
                trajectory.samples.push_back(segmentPositionAt(segment, localTime));
            }

            for (const Obstacle &obstacle : scenario.obstacles) {
                trajectory.metrics.obstacleTime += timeInsideObstacle(segment, obstacle);
            }

            trajectory.metrics.speedPenalty += speedPenalty(segment, config);
            trajectory.metrics.totalTime += control.duration;
            position = segment.end.position;
            velocity = segment.end.velocity;
        }

        trajectory.finish.position = position;
        trajectory.finish.velocity = velocity;
        trajectory.metrics.positionError = (position - scenario.goal.position).mag();
        trajectory.metrics.velocityError = (velocity - scenario.goal.velocity).mag();
        trajectory.metrics.objective =
            config.positionWeight * (trajectory.metrics.positionError / config.positionTolerance) *
                (trajectory.metrics.positionError / config.positionTolerance) +
            config.velocityWeight * (trajectory.metrics.velocityError / config.velocityTolerance) *
                (trajectory.metrics.velocityError / config.velocityTolerance) +
            config.timeWeight * trajectory.metrics.totalTime +
            config.obstacleWeight * trajectory.metrics.obstacleTime +
            config.speedWeight * trajectory.metrics.speedPenalty;

        return trajectory;
    }

    vector<double> simplexSteps(const PlannerConfig &config) {
        vector<double> steps(config.segmentCount * 2, 0.0);
        for (int i = 0; i < config.segmentCount; ++i) {
            steps[i] = 0.35;
            steps[config.segmentCount + i] = 0.25;
        }
        return steps;
    }

    bool qualitySatisfied(const TrajectoryMetrics &metrics, const PlannerConfig &config) {
        return metrics.positionError <= config.positionTolerance &&
               metrics.velocityError <= config.velocityTolerance &&
               metrics.obstacleTime <= 1e-4 &&
               metrics.speedPenalty <= 1e-4;
    }

    vector<ControlSegment> splitLongestControl(const vector<ControlSegment> &controls) {
        vector<ControlSegment> expanded = controls;
        if (expanded.empty()) {
            return expanded;
        }

        size_t splitIndex = 0;
        double bestDuration = expanded.front().duration;
        for (size_t i = 1; i < expanded.size(); ++i) {
            if (expanded[i].duration > bestDuration) {
                bestDuration = expanded[i].duration;
                splitIndex = i;
            }
        }

        ControlSegment first = expanded[splitIndex];
        ControlSegment second = expanded[splitIndex];
        first.duration *= 0.5;
        second.duration -= first.duration;
        expanded[splitIndex] = first;
        expanded.insert(expanded.begin() + static_cast<long>(splitIndex) + 1, second);
        return expanded;
    }

    vector<vector<double>> makeAdaptiveSeeds(const vector<ControlSegment> &baseControls, const PlannerConfig &config) {
        vector<vector<double>> seeds;
        if (baseControls.size() != static_cast<size_t>(config.segmentCount)) {
            return seeds;
        }

        seeds.push_back(encodeControls(baseControls, config));
        for (size_t i = 0; i < baseControls.size(); ++i) {
            vector<ControlSegment> perturbed = baseControls;
            perturbed[i].angle = numAux::windDownAngle(perturbed[i].angle + 0.18);
            seeds.push_back(encodeControls(perturbed, config));
            perturbed[i].angle = numAux::windDownAngle(baseControls[i].angle - 0.18);
            seeds.push_back(encodeControls(perturbed, config));
        }
        return seeds;
    }

    SolveResult solveScenarioFixed(const Scenario &scenario,
                                   OptimizerKind optimizer,
                                   PlannerConfig config = PlannerConfig(),
                                   const vector<vector<double>> &extraSeeds = {},
                                   const ProgressCallback &progress = nullptr) {
        config.segmentCount = scenario.segmentCount;
        config.maxSegmentCount = max(config.segmentCount, scenario.maxSegmentCount);
        config.adaptiveSegments = scenario.adaptiveSegments;
        config.newton.max_iterations = 8;
        config.newton.tolerance = 1e-4;
        config.newton.finite_difference_step = 3e-3;
        config.nelderMead.max_iterations = 90;
        config.nelderMead.tolerance = 1e-4;
        config.nelderMead.finite_difference_step = 1e-4;
        config.nelderMead.initial_steps = simplexSteps(config);

        SolveResult result;
        result.scenario = scenario;
        result.optimizer = optimizer;

        vector<vector<double>> seeds = makeInitialSeeds(scenario, config);
        for (const vector<double> &seed : extraSeeds) {
            if (seed.size() == static_cast<size_t>(config.segmentCount * 2)) {
                seeds.push_back(seed);
            }
        }
        vector<double> bestSeed = seeds.front();
        double bestSeedValue = numeric_limits<double>::infinity();
        vector<double> bestParametersSeen = bestSeed;
        Trajectory bestTrajectorySeen;
        bool hasBestTrajectorySeen = false;
        Timer timer;

        int evaluations = 0;
        auto objectiveWithConfig = [&](const PlannerConfig &activeConfig, const vector<double> &parameters) {
            ++evaluations;
            Trajectory trajectory = simulateTrajectory(scenario, decodeControls(parameters, activeConfig), activeConfig);
            const double objective = trajectory.metrics.objective;
            const bool improved = !hasBestTrajectorySeen || objective + 1e-9 < bestTrajectorySeen.metrics.objective;
            if (improved) {
                bestTrajectorySeen = trajectory;
                bestParametersSeen = parameters;
                hasBestTrajectorySeen = true;
            }

            if (progress && (improved || evaluations % 3 == 0)) {
                ProgressSnapshot snapshot;
                snapshot.scenario = scenario;
                snapshot.scenario.segmentCount = activeConfig.segmentCount;
                snapshot.optimizer = optimizer;
                snapshot.currentTrajectory = trajectory;
                snapshot.bestTrajectory = hasBestTrajectorySeen ? bestTrajectorySeen : trajectory;
                snapshot.evaluations = evaluations;
                snapshot.elapsedSeconds = static_cast<double>(timer.time());
                snapshot.improved = improved;
                snapshot.final = false;
                snapshot.converged = false;
                snapshot.stage = "segment_count=" + to_string(activeConfig.segmentCount);
                if (!progress(snapshot)) {
                    throw OptimizationAborted();
                }
            }

            return objective;
        };

        PlannerConfig polishConfig = config;
        polishConfig.positionWeight *= 6.0;
        polishConfig.velocityWeight *= 5.0;
        polishConfig.timeWeight *= 0.5;
        polishConfig.obstacleWeight *= 1.15;
        polishConfig.nelderMead.max_iterations = 45;
        polishConfig.newton.max_iterations = 4;

        optimization::Result optimizationResult;
        try {
            for (const vector<double> &seed : seeds) {
                const double value = objectiveWithConfig(config, seed);
                if (value < bestSeedValue) {
                    bestSeedValue = value;
                    bestSeed = seed;
                }
            }

            if (optimizer == OptimizerKind::Newton) {
                auto polishObjective = [&](const vector<double> &parameters) {
                    return objectiveWithConfig(polishConfig, parameters);
                };
                optimization::Options warmupOptions = polishConfig.nelderMead;
                warmupOptions.max_iterations = 45;
                optimization::Result warmupResult = optimization::minimizeNelderMead(polishObjective, bestSeed, warmupOptions);
                optimization::Result polishResult = optimization::minimizeNewton(polishObjective, warmupResult.point, polishConfig.newton);
                const double seedPolishValue = polishObjective(bestSeed);
                const double warmupPolishValue = polishObjective(warmupResult.point);
                const double polishValue = polishObjective(polishResult.point);
                optimizationResult = polishValue < warmupPolishValue ? polishResult : warmupResult;
                if (warmupPolishValue < polishObjective(optimizationResult.point)) {
                    optimizationResult = warmupResult;
                    optimizationResult.value = warmupPolishValue;
                }
                if (seedPolishValue < polishObjective(optimizationResult.point)) {
                    optimizationResult.point = bestSeed;
                    optimizationResult.value = seedPolishValue;
                }
            } else {
                auto coarseObjective = [&](const vector<double> &parameters) {
                    return objectiveWithConfig(config, parameters);
                };
                auto polishObjective = [&](const vector<double> &parameters) {
                    return objectiveWithConfig(polishConfig, parameters);
                };
                optimization::Result coarseResult = optimization::minimizeNelderMead(coarseObjective, bestSeed, config.nelderMead);
                optimization::Result polishResult = optimization::minimizeNelderMead(polishObjective, coarseResult.point, polishConfig.nelderMead);
                const double seedPolishValue = polishObjective(bestSeed);
                const double coarsePolishValue = polishObjective(coarseResult.point);
                const double polishValue = polishObjective(polishResult.point);
                optimizationResult = polishValue < coarsePolishValue ? polishResult : coarseResult;
                if (seedPolishValue < polishObjective(optimizationResult.point)) {
                    optimizationResult.point = bestSeed;
                    optimizationResult.value = seedPolishValue;
                }
            }
        } catch (const OptimizationAborted &) {
            optimizationResult.point = bestParametersSeen;
            optimizationResult.value = hasBestTrajectorySeen ? bestTrajectorySeen.metrics.objective : bestSeedValue;
            optimizationResult.converged = false;
        }
        result.optimizationSeconds = static_cast<double>(timer.time());
        result.parameters = optimizationResult.point;
        result.objectiveEvaluations = evaluations;
        result.converged = optimizationResult.converged;
        result.trajectory = simulateTrajectory(scenario, decodeControls(result.parameters, polishConfig), polishConfig);

        if (progress) {
            ProgressSnapshot snapshot;
            snapshot.scenario = scenario;
            snapshot.scenario.segmentCount = polishConfig.segmentCount;
            snapshot.optimizer = optimizer;
            snapshot.currentTrajectory = result.trajectory;
            snapshot.bestTrajectory = result.trajectory;
            snapshot.evaluations = evaluations;
            snapshot.elapsedSeconds = result.optimizationSeconds;
            snapshot.improved = true;
            snapshot.final = true;
            snapshot.converged = result.converged;
            snapshot.stage = "final";
            progress(snapshot);
        }

        return result;
    }

    SolveResult solveScenario(const Scenario &scenario,
                             OptimizerKind optimizer,
                             PlannerConfig config = PlannerConfig(),
                             const ProgressCallback &progress = nullptr) {
        config.segmentCount = scenario.segmentCount;
        config.maxSegmentCount = max(scenario.segmentCount, scenario.maxSegmentCount);
        config.adaptiveSegments = scenario.adaptiveSegments;

        SolveResult bestResult = solveScenarioFixed(scenario, optimizer, config, {}, progress);
        double totalOptimizationSeconds = bestResult.optimizationSeconds;
        int totalObjectiveEvaluations = bestResult.objectiveEvaluations;
        if (!config.adaptiveSegments || config.maxSegmentCount <= config.segmentCount) {
            return bestResult;
        }

        for (int segmentCount = scenario.segmentCount + 1; segmentCount <= config.maxSegmentCount; ++segmentCount) {
            Scenario candidateScenario = scenario;
            candidateScenario.segmentCount = segmentCount;

            vector<ControlSegment> expandedControls = bestResult.trajectory.controls;
            while (expandedControls.size() < static_cast<size_t>(segmentCount)) {
                expandedControls = splitLongestControl(expandedControls);
            }

            PlannerConfig candidateConfig = config;
            candidateConfig.segmentCount = segmentCount;
            vector<vector<double>> adaptiveSeeds = makeAdaptiveSeeds(expandedControls, candidateConfig);
            SolveResult candidateResult = solveScenarioFixed(candidateScenario, optimizer, candidateConfig, adaptiveSeeds, progress);
            totalOptimizationSeconds += candidateResult.optimizationSeconds;
            totalObjectiveEvaluations += candidateResult.objectiveEvaluations;

            const double bestObjective = bestResult.trajectory.metrics.objective;
            const double candidateObjective = candidateResult.trajectory.metrics.objective;
            const bool improved = candidateObjective + 1e-6 < bestObjective;
            const double relativeImprovement = (bestObjective - candidateObjective) / max(1.0, bestObjective);

            if (improved) {
                bestResult = candidateResult;
            }

            if (qualitySatisfied(bestResult.trajectory.metrics, config) &&
                (!improved || relativeImprovement < config.segmentImprovementThreshold)) {
                break;
            }
        }

        bestResult.optimizationSeconds = totalOptimizationSeconds;
        bestResult.objectiveEvaluations = totalObjectiveEvaluations;
        if (progress) {
            ProgressSnapshot snapshot;
            snapshot.scenario = bestResult.scenario;
            snapshot.optimizer = bestResult.optimizer;
            snapshot.currentTrajectory = bestResult.trajectory;
            snapshot.bestTrajectory = bestResult.trajectory;
            snapshot.evaluations = bestResult.objectiveEvaluations;
            snapshot.elapsedSeconds = bestResult.optimizationSeconds;
            snapshot.improved = true;
            snapshot.final = true;
            snapshot.converged = bestResult.converged;
            snapshot.stage = "adaptive-final";
            progress(snapshot);
        }
        return bestResult;
    }

    bool isPointFarEnough(const Point &point, const vector<Obstacle> &obstacles, double clearance) {
        for (const Obstacle &obstacle : obstacles) {
            if ((point - obstacle.center).mag() < obstacle.radius + clearance) {
                return false;
            }
        }
        return true;
    }

    Point randomFreePoint(const vector<Obstacle> &obstacles, double clearance) {
        for (int attempt = 0; attempt < 500; ++attempt) {
            Point point(quickrandom::random_double(-kSceneHalfWidth, kSceneHalfWidth),
                        quickrandom::random_double(-kSceneHalfHeight, kSceneHalfHeight));
            if (isPointFarEnough(point, obstacles, clearance)) {
                return point;
            }
        }
        return Point();
    }

    Point randomVelocity(double maxSpeed) {
        const double angle = quickrandom::random_double(-M_PI, M_PI);
        const double speed = quickrandom::random_double(0.0, maxSpeed * 0.35);
        return directionFromAngle(angle, speed);
    }

    Scenario makeRandomScenario(int index, double obstacleRadius = kDefaultObstacleRadius) {
        Scenario scenario;
        scenario.name = "case_" + to_string(index);
        scenario.segmentCount = kDefaultSegmentCount;
        scenario.maxSegmentCount = kDefaultMaxSegmentCount;
        scenario.adaptiveSegments = true;
        scenario.obstacleRadius = obstacleRadius;

        const int obstacleCount = quickrandom::random_int(2, 5);
        for (int attempt = 0; attempt < 4000 && static_cast<int>(scenario.obstacles.size()) < obstacleCount; ++attempt) {
            Obstacle obstacle;
            obstacle.radius = obstacleRadius;
            obstacle.center = Point(quickrandom::random_double(-kSceneHalfWidth * 0.8, kSceneHalfWidth * 0.8),
                                    quickrandom::random_double(-kSceneHalfHeight * 0.8, kSceneHalfHeight * 0.8));

            bool overlap = false;
            for (const Obstacle &existing : scenario.obstacles) {
                if ((existing.center - obstacle.center).mag() < (existing.radius + obstacle.radius) * 1.35) {
                    overlap = true;
                    break;
                }
            }
            if (!overlap) {
                scenario.obstacles.push_back(obstacle);
            }
        }

        scenario.start.position = randomFreePoint(scenario.obstacles, obstacleRadius * 1.2);
        do {
            scenario.goal.position = randomFreePoint(scenario.obstacles, obstacleRadius * 1.2);
        } while ((scenario.goal.position - scenario.start.position).mag() < 2500.0);
        scenario.start.velocity = randomVelocity(MAX_SPEED);
        scenario.goal.velocity = randomVelocity(MAX_SPEED * 0.7);
        return scenario;
    }

    Scenario makeDefaultScenario() {
        Scenario scenario;
        scenario.name = "demo";
        scenario.segmentCount = kDefaultSegmentCount;
        scenario.maxSegmentCount = kDefaultMaxSegmentCount;
        scenario.adaptiveSegments = true;
        scenario.obstacleRadius = kDefaultObstacleRadius;
        scenario.start.position = Point(-3400.0, -1200.0);
        scenario.start.velocity = Point(250.0, 80.0);
        scenario.goal.position = Point(3350.0, 1250.0);
        scenario.goal.velocity = Point(-180.0, 0.0);
        scenario.obstacles = {
            {Point(-1200.0, -350.0), kDefaultObstacleRadius},
            {Point(-300.0, 950.0), kDefaultObstacleRadius},
            {Point(900.0, -850.0), kDefaultObstacleRadius},
            {Point(1750.0, 500.0), kDefaultObstacleRadius}
        };
        return scenario;
    }

    bool loadScenarioFromFile(const string &path, Scenario &scenario) {
        ifstream input(path);
        if (!input) {
            return false;
        }

        scenario = Scenario();
        string token;
        while (input >> token) {
            if (token[0] == '#') {
                string restOfLine;
                getline(input, restOfLine);
                continue;
            }
            if (token == "name") {
                input >> scenario.name;
            } else if (token == "segment_count") {
                input >> scenario.segmentCount;
            } else if (token == "max_segment_count") {
                input >> scenario.maxSegmentCount;
            } else if (token == "adaptive_segments") {
                int flag = 1;
                input >> flag;
                scenario.adaptiveSegments = flag != 0;
            } else if (token == "obstacle_radius") {
                input >> scenario.obstacleRadius;
            } else if (token == "start") {
                input >> scenario.start.position.x >> scenario.start.position.y
                      >> scenario.start.velocity.x >> scenario.start.velocity.y;
            } else if (token == "goal") {
                input >> scenario.goal.position.x >> scenario.goal.position.y
                      >> scenario.goal.velocity.x >> scenario.goal.velocity.y;
            } else if (token == "obstacles") {
                int count = 0;
                input >> count;
                scenario.obstacles.clear();
                for (int i = 0; i < count; ++i) {
                    Obstacle obstacle;
                    obstacle.radius = scenario.obstacleRadius;
                    input >> obstacle.center.x >> obstacle.center.y;
                    scenario.obstacles.push_back(obstacle);
                }
            }
        }

        if (scenario.segmentCount <= 0) {
            scenario.segmentCount = kDefaultSegmentCount;
        }
        if (scenario.maxSegmentCount < scenario.segmentCount) {
            scenario.maxSegmentCount = scenario.segmentCount;
        }
        for (Obstacle &obstacle : scenario.obstacles) {
            obstacle.radius = scenario.obstacleRadius;
        }
        return true;
    }
}
