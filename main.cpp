#include "include/planner.h"
#include "include/sfmlLiveRe        nderer.h"
#include "include/svgRenderer.h"

using namespace std;

struct CliOptions {
    string inputPath;
    string outputDir = "artifacts";
    string optimizer = "nelder-mead";
    int tests = 0;
    unsigned int seed = 42;
    bool render = true;
    bool sfmlLive = false;
    optional<int> segmentCount;
    optional<int> maxSegmentCount;
    bool adaptiveSegments = true;
};

void printUsage(const char *program) {
    cout << "Usage:\n";
    cout << "  " << program << " [--input path] [--optimizer newton|nelder-mead|both]\n";
    cout << "  " << program << " --tests N [--optimizer newton|nelder-mead|both] [--seed 42]\n";
    cout << "Options:\n";
    cout << "  --sfml-live         Show optimization progress in a real-time SFML window\n";
    cout << "  --segments N         Initial number of trajectory segments\n";
    cout << "  --max-segments N     Maximum number of segments when adaptive mode is on\n";
    cout << "  --fixed-segments     Disable automatic segment growth\n";
    cout << "  --output-dir PATH   Where to store SVG reports (default: artifacts)\n";
    cout << "  --no-render         Skip SVG generation\n";
    cout << "  --help              Show this message\n";
}

bool parseArguments(int argc, char **argv, CliOptions &options) {
    for (int i = 1; i < argc; ++i) {
        const string argument = argv[i];
        if (argument == "--input" && i + 1 < argc) {
            options.inputPath = argv[++i];
        } else if (argument == "--output-dir" && i + 1 < argc) {
            options.outputDir = argv[++i];
        } else if (argument == "--optimizer" && i + 1 < argc) {
            options.optimizer = argv[++i];
        } else if (argument == "--tests" && i + 1 < argc) {
            options.tests = stoi(argv[++i]);
        } else if (argument == "--seed" && i + 1 < argc) {
            options.seed = static_cast<unsigned int>(stoul(argv[++i]));
        } else if (argument == "--segments" && i + 1 < argc) {
            options.segmentCount = stoi(argv[++i]);
        } else if (argument == "--max-segments" && i + 1 < argc) {
            options.maxSegmentCount = stoi(argv[++i]);
        } else if (argument == "--fixed-segments") {
            options.adaptiveSegments = false;
        } else if (argument == "--sfml-live") {
            options.sfmlLive = true;
        } else if (argument == "--no-render") {
            options.render = false;
        } else if (argument == "--help") {
            printUsage(argv[0]);
            return false;
        } else {
            cerr << "Unknown argument: " << argument << "\n";
            printUsage(argv[0]);
            return false;
        }
    }
    return true;
}

vector<strategy::OptimizerKind> parseOptimizers(const string &name) {
    if (name == "newton") {
        return {strategy::OptimizerKind::Newton};
    }
    if (name == "nelder-mead" || name == "nelder_mead" || name == "nelder") {
        return {strategy::OptimizerKind::NelderMead};
    }
    if (name == "both") {
        return {strategy::OptimizerKind::Newton, strategy::OptimizerKind::NelderMead};
    }
    throw invalid_argument("optimizer must be one of: newton, nelder-mead, both");
}

bool approxEqual(double lhs, double rhs, double eps = 1e-4) {
    return abs(lhs - rhs) <= eps;
}

bool validateRoots(const vector<double> &actual, const vector<double> &expected, double eps = 1e-4) {
    if (actual.size() != expected.size()) {
        return false;
    }
    for (size_t i = 0; i < actual.size(); ++i) {
        if (!approxEqual(actual[i], expected[i], eps)) {
            return false;
        }
    }
    return true;
}

bool runPolynomialTests() {
    bool success = true;

    {
        double roots[4] = {};
        const int count = numAux::solveThree(roots, 1.0, -6.0, 11.0, -6.0);
        vector<double> actual(roots, roots + count);
        if (!validateRoots(actual, {1.0, 2.0, 3.0})) {
            cerr << "Cubic solver test failed\n";
            success = false;
        }
    }

    {
        double roots[4] = {};
        const int count = numAux::solveFour(roots, 1.0, -10.0, 35.0, -50.0, 24.0);
        vector<double> actual(roots, roots + count);
        if (!validateRoots(actual, {1.0, 2.0, 3.0, 4.0})) {
            cerr << "Quartic solver test failed\n";
            success = false;
        }
    }

    {
        double roots[4] = {};
        const int count = numAux::solveFour(roots, 1.0, 0.0, -5.0, 0.0, 4.0);
        vector<double> actual(roots, roots + count);
        if (!validateRoots(actual, {-2.0, -1.0, 1.0, 2.0})) {
            cerr << "Symmetric quartic solver test failed\n";
            success = false;
        }
    }

    cout << "Polynomial tests: " << (success ? "passed" : "failed") << "\n";
    return success;
}

void printResult(const strategy::SolveResult &result) {
    cout << "\nScenario: " << result.scenario.name
         << " | optimizer: " << strategy::optimizerName(result.optimizer) << "\n";
    cout << "  segments: " << result.trajectory.controls.size();
    if (result.scenario.adaptiveSegments) {
        cout << " (adaptive up to " << result.scenario.maxSegmentCount << ")";
    }
    cout << "\n";
    cout << "  optimization: " << fixed << setprecision(3) << result.optimizationSeconds * 1000.0
         << " ms";
    if (result.optimizationSeconds * 1000.0 <= 10.0) {
        cout << " (within target)";
    }
    cout << "\n";
    cout << "  objective: " << setprecision(4) << result.trajectory.metrics.objective
         << ", pos_err: " << result.trajectory.metrics.positionError
         << ", vel_err: " << result.trajectory.metrics.velocityError
         << ", obstacle_time: " << result.trajectory.metrics.obstacleTime
         << ", path_time: " << result.trajectory.metrics.totalTime << "\n";
    cout << "  controls:\n";
    for (size_t i = 0; i < result.trajectory.controls.size(); ++i) {
        const strategy::ControlSegment &control = result.trajectory.controls[i];
        cout << "    " << i + 1 << ". ("
             << setprecision(3) << control.angle * 180.0 / M_PI << " deg, "
             << setprecision(4) << control.duration << " s)\n";
    }
    if (!result.renderPath.empty()) {
        cout << "  render: " << result.renderPath << "\n";
    }
}

strategy::TestSummary runScenarioBatch(int count,
                                       strategy::OptimizerKind optimizer,
                                       const CliOptions &options) {
    strategy::TestSummary summary;

    for (int i = 0; i < count; ++i) {
        strategy::Scenario scenario = strategy::makeRandomScenario(i + 1);
        if (options.segmentCount.has_value()) {
            scenario.segmentCount = max(1, *options.segmentCount);
        }
        if (options.maxSegmentCount.has_value()) {
            scenario.maxSegmentCount = max(1, *options.maxSegmentCount);
        }
        scenario.adaptiveSegments = options.adaptiveSegments;
        scenario.maxSegmentCount = max(scenario.segmentCount, scenario.maxSegmentCount);
        strategy::SolveResult result = strategy::solveScenario(scenario, optimizer);
        if (options.render) {
            filesystem::path path = filesystem::path(options.outputDir) /
                                    (scenario.name + "_" + strategy::optimizerName(optimizer) + ".svg");
            if (svg::renderSvg(result, path.string())) {
                result.renderPath = path.string();
            }
        }

        summary.averageOptimizationMs += result.optimizationSeconds * 1000.0;
        summary.maxOptimizationMs = max(summary.maxOptimizationMs, result.optimizationSeconds * 1000.0);
        if (result.optimizationSeconds * 1000.0 <= 10.0) {
            ++summary.underTenMs;
        }

        printResult(result);
        summary.results.push_back(result);
    }

    if (!summary.results.empty()) {
        summary.averageOptimizationMs /= static_cast<double>(summary.results.size());
    }
    return summary;
}

int main(int argc, char **argv) {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    CliOptions options;
    if (!parseArguments(argc, argv, options)) {
        return 0;
    }

    if (options.sfmlLive && !sfml_live::available()) {
        cerr << sfml_live::unavailableReason() << "\n";
        return 1;
    }

    quickrandom::seed(options.seed);

    if (!runPolynomialTests()) {
        return 1;
    }

    vector<strategy::OptimizerKind> optimizers;
    try {
        optimizers = parseOptimizers(options.optimizer);
    } catch (const exception &error) {
        cerr << error.what() << "\n";
        return 1;
    }

    if (options.tests > 0) {
        if (options.sfmlLive) {
            cerr << "SFML live mode is supported only for a single scenario run\n";
            return 1;
        }
        for (strategy::OptimizerKind optimizer : optimizers) {
            cout << "\nRunning " << options.tests << " random scenarios with "
                 << strategy::optimizerName(optimizer) << "\n";
            strategy::TestSummary summary = runScenarioBatch(options.tests, optimizer, options);
            cout << "\nSummary for " << strategy::optimizerName(optimizer) << ":\n";
            cout << "  average optimization time: " << fixed << setprecision(3)
                 << summary.averageOptimizationMs << " ms\n";
            cout << "  max optimization time: " << summary.maxOptimizationMs << " ms\n";
            cout << "  under 10 ms: " << summary.underTenMs << "/" << summary.results.size() << "\n";
        }
        return 0;
    }

    strategy::Scenario scenario = strategy::makeDefaultScenario();
    if (!options.inputPath.empty() && !strategy::loadScenarioFromFile(options.inputPath, scenario)) {
        cerr << "Failed to read scenario file: " << options.inputPath << "\n";
        return 1;
    }

    if (options.segmentCount.has_value()) {
        scenario.segmentCount = max(1, *options.segmentCount);
    }
    if (options.maxSegmentCount.has_value()) {
        scenario.maxSegmentCount = max(1, *options.maxSegmentCount);
    }
    scenario.adaptiveSegments = options.adaptiveSegments;
    scenario.maxSegmentCount = max(scenario.segmentCount, scenario.maxSegmentCount);

    strategy::PlannerConfig plannerConfig;
    for (strategy::OptimizerKind optimizer : optimizers) {
        strategy::SolveResult result;
        if (options.sfmlLive) {
            string error;
            if (!sfml_live::runLiveSolve(scenario, optimizer, plannerConfig, result, error)) {
                cerr << error << "\n";
                return 1;
            }
        } else {
            result = strategy::solveScenario(scenario, optimizer, plannerConfig);
        }
        if (options.render) {
            filesystem::path path = filesystem::path(options.outputDir) /
                                    (scenario.name + "_" + strategy::optimizerName(optimizer) + ".svg");
            if (svg::renderSvg(result, path.string())) {
                result.renderPath = path.string();
            }
        }
        printResult(result);
    }

    return 0;
}
