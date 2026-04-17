#pragma once

#include "planner.h"

using namespace std;

namespace svg {
    struct Bounds {
        double minX = numeric_limits<double>::infinity();
        double maxX = -numeric_limits<double>::infinity();
        double minY = numeric_limits<double>::infinity();
        double maxY = -numeric_limits<double>::infinity();
    };

    void extend(Bounds &bounds, const Point &point, double padding = 0.0) {
        bounds.minX = min(bounds.minX, point.x - padding);
        bounds.maxX = max(bounds.maxX, point.x + padding);
        bounds.minY = min(bounds.minY, point.y - padding);
        bounds.maxY = max(bounds.maxY, point.y + padding);
    }

    string formatDouble(double value, int precision = 2) {
        ostringstream stream;
        stream << fixed << setprecision(precision) << value;
        return stream.str();
    }

    string escape(const string &text) {
        string escaped;
        escaped.reserve(text.size());
        for (char ch : text) {
            switch (ch) {
                case '&':
                    escaped += "&amp;";
                    break;
                case '<':
                    escaped += "&lt;";
                    break;
                case '>':
                    escaped += "&gt;";
                    break;
                case '"':
                    escaped += "&quot;";
                    break;
                default:
                    escaped += ch;
                    break;
            }
        }
        return escaped;
    }

    bool renderSvg(const strategy::SolveResult &result, const string &path) {
        namespace fs = filesystem;

        Bounds bounds;
        extend(bounds, result.scenario.start.position, 200.0);
        extend(bounds, result.scenario.goal.position, 200.0);
        for (const strategy::Obstacle &obstacle : result.scenario.obstacles) {
            extend(bounds, obstacle.center, obstacle.radius + 200.0);
        }
        for (const Point &sample : result.trajectory.samples) {
            extend(bounds, sample, 50.0);
        }

        if (!isfinite(bounds.minX) || !isfinite(bounds.minY)) {
            return false;
        }

        const double sidebarWidth = 420.0;
        const double canvasWidth = 1440.0;
        const double canvasHeight = 920.0;
        const double margin = 60.0;
        const double worldWidth = max(1.0, bounds.maxX - bounds.minX);
        const double worldHeight = max(1.0, bounds.maxY - bounds.minY);
        const double scale = min((canvasWidth - sidebarWidth - 2.0 * margin) / worldWidth,
                                 (canvasHeight - 2.0 * margin) / worldHeight);

        auto mapX = [&](double x) {
            return margin + (x - bounds.minX) * scale;
        };
        auto mapY = [&](double y) {
            return canvasHeight - margin - (y - bounds.minY) * scale;
        };

        fs::create_directories(fs::path(path).parent_path());
        ofstream out(path);
        if (!out) {
            return false;
        }

        out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << canvasWidth
            << "\" height=\"" << canvasHeight << "\" viewBox=\"0 0 " << canvasWidth << " "
            << canvasHeight << "\">\n";
        out << "<rect width=\"100%\" height=\"100%\" fill=\"#f7f7f2\"/>\n";
        out << "<rect x=\"0\" y=\"0\" width=\"" << canvasWidth - sidebarWidth
            << "\" height=\"" << canvasHeight << "\" fill=\"#fffdf7\" stroke=\"#d8d5c4\"/>\n";
        out << "<rect x=\"" << canvasWidth - sidebarWidth << "\" y=\"0\" width=\"" << sidebarWidth
            << "\" height=\"" << canvasHeight << "\" fill=\"#17313e\"/>\n";

        const double gridStep = 500.0;
        const double startGridX = floor(bounds.minX / gridStep) * gridStep;
        const double endGridX = ceil(bounds.maxX / gridStep) * gridStep;
        const double startGridY = floor(bounds.minY / gridStep) * gridStep;
        const double endGridY = ceil(bounds.maxY / gridStep) * gridStep;

        for (double x = startGridX; x <= endGridX; x += gridStep) {
            out << "<line x1=\"" << mapX(x) << "\" y1=\"" << margin << "\" x2=\"" << mapX(x)
                << "\" y2=\"" << canvasHeight - margin
                << "\" stroke=\"#ece8d8\" stroke-width=\"1\"/>\n";
        }
        for (double y = startGridY; y <= endGridY; y += gridStep) {
            out << "<line x1=\"" << margin << "\" y1=\"" << mapY(y) << "\" x2=\""
                << canvasWidth - sidebarWidth - margin << "\" y2=\"" << mapY(y)
                << "\" stroke=\"#ece8d8\" stroke-width=\"1\"/>\n";
        }

        for (const strategy::Obstacle &obstacle : result.scenario.obstacles) {
            out << "<circle cx=\"" << mapX(obstacle.center.x) << "\" cy=\"" << mapY(obstacle.center.y)
                << "\" r=\"" << obstacle.radius * scale
                << "\" fill=\"#ef6f6c\" fill-opacity=\"0.22\" stroke=\"#9f2f2a\" stroke-width=\"2\"/>\n";
        }

        ostringstream polyline;
        for (const Point &sample : result.trajectory.samples) {
            polyline << mapX(sample.x) << "," << mapY(sample.y) << " ";
        }
        out << "<polyline fill=\"none\" stroke=\"#1f6f8b\" stroke-width=\"4\" points=\""
            << polyline.str() << "\"/>\n";

        for (const strategy::SegmentTrace &segment : result.trajectory.segments) {
            out << "<circle cx=\"" << mapX(segment.begin.position.x) << "\" cy=\"" << mapY(segment.begin.position.y)
                << "\" r=\"5\" fill=\"#0f4c5c\"/>\n";
        }

        auto drawState = [&](const strategy::RobotState &state, const string &fill, const string &stroke) {
            const double cx = mapX(state.position.x);
            const double cy = mapY(state.position.y);
            out << "<circle cx=\"" << cx << "\" cy=\"" << cy << "\" r=\"10\" fill=\"" << fill
                << "\" stroke=\"" << stroke << "\" stroke-width=\"3\"/>\n";

            Point velocity = state.velocity;
            if (velocity.mag() > 1e-6) {
                const Point end = state.position + velocity * 0.35;
                out << "<line x1=\"" << cx << "\" y1=\"" << cy << "\" x2=\"" << mapX(end.x)
                    << "\" y2=\"" << mapY(end.y) << "\" stroke=\"" << stroke
                    << "\" stroke-width=\"3\" marker-end=\"url(#arrow)\"/>\n";
            }
        };

        out << "<defs>\n";
        out << "<marker id=\"arrow\" markerWidth=\"8\" markerHeight=\"8\" refX=\"7\" refY=\"4\" orient=\"auto\">\n";
        out << "<path d=\"M0,0 L8,4 L0,8 z\" fill=\"#17313e\"/>\n";
        out << "</marker>\n";
        out << "</defs>\n";

        drawState(result.scenario.start, "#2ec4b6", "#0b525b");
        drawState(result.scenario.goal, "#ffbf69", "#bc6c25");
        drawState(result.trajectory.finish, "#ffffff", "#1f6f8b");

        double textX = canvasWidth - sidebarWidth + 28.0;
        double textY = 48.0;
        auto addText = [&](const string &text, int fontSize = 20, const string &fill = "#f8faf8", bool bold = false) {
            out << "<text x=\"" << textX << "\" y=\"" << textY << "\" fill=\"" << fill
                << "\" font-family=\"'DejaVu Sans Mono', monospace\" font-size=\"" << fontSize << "\"";
            if (bold) {
                out << " font-weight=\"700\"";
            }
            out << ">" << escape(text) << "</text>\n";
            textY += fontSize + 10.0;
        };

        addText("Trajectory Report", 24, "#fefae0", true);
        addText("Scenario: " + result.scenario.name, 18);
        addText("Optimizer: " + strategy::optimizerName(result.optimizer), 18);
        addText("Segments: " + to_string(result.trajectory.controls.size()), 18);
        if (result.scenario.adaptiveSegments) {
            addText("Adaptive cap: " + to_string(result.scenario.maxSegmentCount), 18);
        }
        addText("Optimize time: " + formatDouble(result.optimizationSeconds * 1000.0) + " ms", 18);
        addText("Objective evals: " + to_string(result.objectiveEvaluations), 18);
        addText("Objective: " + formatDouble(result.trajectory.metrics.objective), 18);
        addText("Pos err: " + formatDouble(result.trajectory.metrics.positionError) + " mm", 18);
        addText("Vel err: " + formatDouble(result.trajectory.metrics.velocityError) + " mm/s", 18);
        addText("Obstacle time: " + formatDouble(result.trajectory.metrics.obstacleTime, 4) + " s", 18);
        addText("Path time: " + formatDouble(result.trajectory.metrics.totalTime, 4) + " s", 18);
        addText("Speed penalty: " + formatDouble(result.trajectory.metrics.speedPenalty, 4), 18);
        textY += 12.0;
        addText("Controls (angle deg, time s)", 18, "#fefae0", true);
        for (size_t i = 0; i < result.trajectory.controls.size(); ++i) {
            const strategy::ControlSegment &control = result.trajectory.controls[i];
            addText(to_string(i + 1) + ". (" +
                    formatDouble(control.angle * 180.0 / M_PI) + ", " +
                    formatDouble(control.duration, 4) + ")", 18);
        }

        out << "</svg>\n";
        return true;
    }
}
