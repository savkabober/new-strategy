#pragma once

#include "planner.h"

#if defined(__has_include)
#if __has_include(<SFML/Graphics.hpp>) && __has_include(<SFML/Window.hpp>)
#define STRATEGY_HAS_SFML 1
#include <SFML/Graphics.hpp>
#else
#define STRATEGY_HAS_SFML 0
#endif
#else
#define STRATEGY_HAS_SFML 0
#endif

using namespace std;

namespace sfml_live {
#if STRATEGY_HAS_SFML
    struct SharedState {
        mutex mtx;
        optional<strategy::ProgressSnapshot> snapshot;
        strategy::SolveResult result;
        bool done = false;
        bool success = false;
        bool cancel = false;
        string error;
    };

    struct CanvasLayout {
        float width = 1500.0f;
        float height = 920.0f;
        float sidebarWidth = 360.0f;
        float margin = 40.0f;

        float sceneWidth() const {
            return width - sidebarWidth;
        }
    };

    bool available() {
        return true;
    }

    string unavailableReason() {
        return "";
    }

    bool loadDefaultFont(sf::Font &font) {
        const vector<string> candidates = {
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/usr/share/fonts/truetype/liberation2/LiberationMono-Regular.ttf"
        };

        for (const string &path : candidates) {
            if (filesystem::exists(path) && font.loadFromFile(path)) {
                return true;
            }
        }
        return false;
    }

    sf::Vector2f mapPoint(const Point &point, const CanvasLayout &layout) {
        const float drawableWidth = layout.sceneWidth() - 2.0f * layout.margin;
        const float drawableHeight = layout.height - 2.0f * layout.margin;
        const float scale = min(drawableWidth / static_cast<float>(2.0 * strategy::kSceneHalfWidth),
                                drawableHeight / static_cast<float>(2.0 * strategy::kSceneHalfHeight));

        const float x = layout.margin +
                        static_cast<float>(point.x + strategy::kSceneHalfWidth) * scale;
        const float y = layout.height - layout.margin -
                        static_cast<float>(point.y + strategy::kSceneHalfHeight) * scale;
        return sf::Vector2f(x, y);
    }

    float radiusToPixels(double radius, const CanvasLayout &layout) {
        const float drawableWidth = layout.sceneWidth() - 2.0f * layout.margin;
        const float drawableHeight = layout.height - 2.0f * layout.margin;
        const float scale = min(drawableWidth / static_cast<float>(2.0 * strategy::kSceneHalfWidth),
                                drawableHeight / static_cast<float>(2.0 * strategy::kSceneHalfHeight));
        return static_cast<float>(radius) * scale;
    }

    void drawGrid(sf::RenderTarget &target, const CanvasLayout &layout) {
        sf::VertexArray lines(sf::Lines);
        const double step = 500.0;

        for (double x = -strategy::kSceneHalfWidth; x <= strategy::kSceneHalfWidth; x += step) {
            const sf::Vector2f top = mapPoint(Point(x, strategy::kSceneHalfHeight), layout);
            const sf::Vector2f bottom = mapPoint(Point(x, -strategy::kSceneHalfHeight), layout);
            lines.append(sf::Vertex(top, sf::Color(226, 230, 224)));
            lines.append(sf::Vertex(bottom, sf::Color(226, 230, 224)));
        }

        for (double y = -strategy::kSceneHalfHeight; y <= strategy::kSceneHalfHeight; y += step) {
            const sf::Vector2f left = mapPoint(Point(-strategy::kSceneHalfWidth, y), layout);
            const sf::Vector2f right = mapPoint(Point(strategy::kSceneHalfWidth, y), layout);
            lines.append(sf::Vertex(left, sf::Color(226, 230, 224)));
            lines.append(sf::Vertex(right, sf::Color(226, 230, 224)));
        }

        target.draw(lines);
    }

    void drawTrajectory(sf::RenderTarget &target,
                        const vector<Point> &samples,
                        const CanvasLayout &layout,
                        const sf::Color &color) {
        if (samples.size() < 2) {
            return;
        }

        sf::VertexArray strip(sf::LineStrip, samples.size());
        for (size_t i = 0; i < samples.size(); ++i) {
            strip[i].position = mapPoint(samples[i], layout);
            strip[i].color = color;
        }
        target.draw(strip);
    }

    void drawObstacle(sf::RenderTarget &target,
                      const strategy::Obstacle &obstacle,
                      const CanvasLayout &layout) {
        sf::CircleShape circle(radiusToPixels(obstacle.radius, layout));
        circle.setOrigin(circle.getRadius(), circle.getRadius());
        circle.setPosition(mapPoint(obstacle.center, layout));
        circle.setFillColor(sf::Color(235, 98, 77, 40));
        circle.setOutlineThickness(2.0f);
        circle.setOutlineColor(sf::Color(163, 48, 36));
        target.draw(circle);
    }

    void drawVelocity(sf::RenderTarget &target,
                      const strategy::RobotState &state,
                      const CanvasLayout &layout,
                      const sf::Color &color) {
        if (state.velocity.mag() < 1e-6) {
            return;
        }

        const Point velocityEnd = state.position + state.velocity * 0.35;
        const sf::Vector2f start = mapPoint(state.position, layout);
        const sf::Vector2f end = mapPoint(velocityEnd, layout);
        sf::VertexArray vector(sf::Lines, 2);
        vector[0].position = start;
        vector[0].color = color;
        vector[1].position = end;
        vector[1].color = color;
        target.draw(vector);
    }

    void drawRobotState(sf::RenderTarget &target,
                        const strategy::RobotState &state,
                        const CanvasLayout &layout,
                        const sf::Color &fill,
                        const sf::Color &stroke) {
        sf::CircleShape marker(9.0f);
        marker.setOrigin(9.0f, 9.0f);
        marker.setPosition(mapPoint(state.position, layout));
        marker.setFillColor(fill);
        marker.setOutlineThickness(2.0f);
        marker.setOutlineColor(stroke);
        target.draw(marker);
        drawVelocity(target, state, layout, stroke);
    }

    void drawPanelText(sf::RenderTarget &target,
                       const sf::Font *font,
                       float x,
                       float &y,
                       const string &text,
                       unsigned int size,
                       const sf::Color &color,
                       bool bold = false) {
        if (font == nullptr) {
            return;
        }

        sf::Text label;
        label.setFont(*font);
        label.setCharacterSize(size);
        label.setFillColor(color);
        label.setString(text);
        label.setPosition(x, y);
        label.setStyle(bold ? sf::Text::Bold : sf::Text::Regular);
        target.draw(label);
        y += static_cast<float>(size) + 8.0f;
    }

    string formatDouble(double value, int precision = 2) {
        ostringstream stream;
        stream << fixed << setprecision(precision) << value;
        return stream.str();
    }

    void drawSnapshot(sf::RenderWindow &window,
                      const optional<strategy::ProgressSnapshot> &snapshot,
                      const sf::Font *font) {
        const CanvasLayout layout;

        sf::RectangleShape bg(sf::Vector2f(layout.width, layout.height));
        bg.setFillColor(sf::Color(245, 247, 242));
        window.draw(bg);

        sf::RectangleShape sceneBg(sf::Vector2f(layout.sceneWidth(), layout.height));
        sceneBg.setFillColor(sf::Color(252, 251, 246));
        sceneBg.setOutlineThickness(1.0f);
        sceneBg.setOutlineColor(sf::Color(214, 214, 204));
        window.draw(sceneBg);

        sf::RectangleShape panel(sf::Vector2f(layout.sidebarWidth, layout.height));
        panel.setPosition(layout.sceneWidth(), 0.0f);
        panel.setFillColor(sf::Color(22, 44, 56));
        window.draw(panel);

        drawGrid(window, layout);

        float textX = layout.sceneWidth() + 22.0f;
        float textY = 24.0f;

        drawPanelText(window, font, textX, textY, "SFML Live Optimization", 22, sf::Color(248, 250, 232), true);

        if (!snapshot.has_value()) {
            drawPanelText(window, font, textX, textY, "Waiting for optimizer...", 18, sf::Color(236, 241, 235));
            return;
        }

        const strategy::ProgressSnapshot &frame = *snapshot;
        const strategy::Scenario &scenario = frame.scenario;

        for (const strategy::Obstacle &obstacle : scenario.obstacles) {
            drawObstacle(window, obstacle, layout);
        }

        drawTrajectory(window, frame.bestTrajectory.samples, layout, sf::Color(32, 144, 153));
        drawTrajectory(window, frame.currentTrajectory.samples, layout, sf::Color(244, 162, 97));

        drawRobotState(window, scenario.start, layout, sf::Color(46, 196, 182), sf::Color(12, 85, 91));
        drawRobotState(window, scenario.goal, layout, sf::Color(255, 191, 105), sf::Color(189, 108, 37));
        drawRobotState(window, frame.bestTrajectory.finish, layout, sf::Color(255, 255, 255), sf::Color(31, 111, 139));

        drawPanelText(window, font, textX, textY, "Scenario: " + scenario.name, 17, sf::Color(236, 241, 235));
        drawPanelText(window, font, textX, textY, "Optimizer: " + strategy::optimizerName(frame.optimizer), 17, sf::Color(236, 241, 235));
        drawPanelText(window, font, textX, textY, "Stage: " + frame.stage, 17, sf::Color(236, 241, 235));
        drawPanelText(window, font, textX, textY, "Evaluations: " + to_string(frame.evaluations), 17, sf::Color(236, 241, 235));
        drawPanelText(window, font, textX, textY, "Elapsed: " + formatDouble(frame.elapsedSeconds * 1000.0) + " ms", 17, sf::Color(236, 241, 235));
        drawPanelText(window, font, textX, textY, "Segments: " + to_string(frame.bestTrajectory.controls.size()), 17, sf::Color(236, 241, 235));
        drawPanelText(window, font, textX, textY, "Current obj: " + formatDouble(frame.currentTrajectory.metrics.objective, 3), 17, sf::Color(236, 241, 235));
        drawPanelText(window, font, textX, textY, "Best obj: " + formatDouble(frame.bestTrajectory.metrics.objective, 3), 17, sf::Color(236, 241, 235));
        drawPanelText(window, font, textX, textY, "Pos err: " + formatDouble(frame.bestTrajectory.metrics.positionError, 2), 17, sf::Color(236, 241, 235));
        drawPanelText(window, font, textX, textY, "Vel err: " + formatDouble(frame.bestTrajectory.metrics.velocityError, 2), 17, sf::Color(236, 241, 235));
        drawPanelText(window, font, textX, textY, "Obstacle time: " + formatDouble(frame.bestTrajectory.metrics.obstacleTime, 4), 17, sf::Color(236, 241, 235));

        textY += 10.0f;
        drawPanelText(window, font, textX, textY, "Best controls", 18, sf::Color(248, 250, 232), true);
        for (size_t i = 0; i < frame.bestTrajectory.controls.size(); ++i) {
            const strategy::ControlSegment &control = frame.bestTrajectory.controls[i];
            drawPanelText(window,
                          font,
                          textX,
                          textY,
                          to_string(i + 1) + ". (" + formatDouble(control.angle * 180.0 / M_PI, 1) +
                              " deg, " + formatDouble(control.duration, 3) + " s)",
                          16,
                          sf::Color(236, 241, 235));
        }

        if (frame.final) {
            textY += 8.0f;
            drawPanelText(window,
                          font,
                          textX,
                          textY,
                          frame.converged ? "Finished" : "Stopped",
                          18,
                          frame.converged ? sf::Color(140, 255, 180) : sf::Color(255, 210, 120),
                          true);
        }
    }

    bool runLiveSolve(const strategy::Scenario &scenario,
                      strategy::OptimizerKind optimizer,
                      const strategy::PlannerConfig &config,
                      strategy::SolveResult &result,
                      string &error) {
        SharedState shared;
        sf::RenderWindow window(sf::VideoMode(1500, 920), "Trajectory Optimizer", sf::Style::Titlebar | sf::Style::Close);
        window.setVerticalSyncEnabled(true);

        sf::Font font;
        sf::Font *fontPtr = loadDefaultFont(font) ? &font : nullptr;

        thread worker([&]() {
            try {
                strategy::ProgressCallback callback = [&](const strategy::ProgressSnapshot &snapshot) {
                    lock_guard<mutex> lock(shared.mtx);
                    shared.snapshot = snapshot;
                    return !shared.cancel;
                };

                strategy::SolveResult liveResult = strategy::solveScenario(scenario, optimizer, config, callback);
                lock_guard<mutex> lock(shared.mtx);
                shared.result = liveResult;
                shared.success = true;
                shared.done = true;
                if (!shared.snapshot.has_value() || !shared.snapshot->final) {
                    strategy::ProgressSnapshot finalSnapshot;
                    finalSnapshot.scenario = liveResult.scenario;
                    finalSnapshot.optimizer = liveResult.optimizer;
                    finalSnapshot.currentTrajectory = liveResult.trajectory;
                    finalSnapshot.bestTrajectory = liveResult.trajectory;
                    finalSnapshot.evaluations = liveResult.objectiveEvaluations;
                    finalSnapshot.elapsedSeconds = liveResult.optimizationSeconds;
                    finalSnapshot.improved = true;
                    finalSnapshot.final = true;
                    finalSnapshot.converged = liveResult.converged;
                    finalSnapshot.stage = "final";
                    shared.snapshot = finalSnapshot;
                }
            } catch (const exception &ex) {
                lock_guard<mutex> lock(shared.mtx);
                shared.error = ex.what();
                shared.done = true;
            }
        });

        while (window.isOpen()) {
            sf::Event event;
            while (window.pollEvent(event)) {
                if (event.type == sf::Event::Closed) {
                    {
                        lock_guard<mutex> lock(shared.mtx);
                        shared.cancel = true;
                    }
                    window.close();
                }
                if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape) {
                    {
                        lock_guard<mutex> lock(shared.mtx);
                        shared.cancel = true;
                    }
                    window.close();
                }
            }

            optional<strategy::ProgressSnapshot> snapshot;
            {
                lock_guard<mutex> lock(shared.mtx);
                snapshot = shared.snapshot;
            }

            window.clear();
            drawSnapshot(window, snapshot, fontPtr);
            window.display();
        }

        {
            lock_guard<mutex> lock(shared.mtx);
            shared.cancel = true;
        }
        if (worker.joinable()) {
            worker.join();
        }

        {
            lock_guard<mutex> lock(shared.mtx);
            if (!shared.success && !shared.error.empty()) {
                error = shared.error;
                return false;
            }
            result = shared.result;
        }

        return true;
    }
#else
    bool available() {
        return false;
    }

    string unavailableReason() {
        return "SFML is not available in this build. Install libsfml-dev and rebuild.";
    }

    bool runLiveSolve(const strategy::Scenario &,
                      strategy::OptimizerKind,
                      const strategy::PlannerConfig &,
                      strategy::SolveResult &,
                      string &error) {
        error = unavailableReason();
        return false;
    }
#endif
}
