#include "render/Renderer.hpp"

#include <SFML/Graphics/Text.hpp>

#include <cmath>
#include <iomanip>
#include <sstream>

#include "render/FontLoader.hpp"

namespace rf {

namespace {

const sf::Color kBackground{9, 11, 17};
const sf::Color kGridMinor{28, 36, 52};
const sf::Color kGridMajor{48, 62, 86};
const sf::Color kBoundsColor{120, 60, 60};
const sf::Color kHullFill{196, 202, 214};
const sf::Color kHullEdge{92, 190, 224};
const sf::Color kPlumeHot{255, 226, 150};
const sf::Color kVelocity{96, 220, 140};
const sf::Color kThrust{255, 150, 60};
const sf::Color kText{188, 200, 216};
const sf::Color kTextDim{112, 126, 148};
const sf::Color kAttractor{74, 84, 112};

void addLine(sf::VertexArray& va, sf::Vector2f a, sf::Vector2f b, sf::Color c) {
    va.append(sf::Vertex{a, c});
    va.append(sf::Vertex{b, c});
}

// Grid spacing on a 1-2-5 ladder, chosen so roughly eight divisions are visible
// at any zoom. Without a reference grid, motion through empty space is close to
// invisible: nothing on screen changes as the ship drifts.
Real gridSpacing(Real widthMeters) {
    const Real target = widthMeters / Real(8);
    const Real mag    = std::pow(Real(10), std::floor(std::log10(target)));
    const Real n      = target / mag;
    const Real step   = n < Real(1.5) ? Real(1) : n < Real(3.5) ? Real(2) : n < Real(7.5) ? Real(5)
                                                                                         : Real(10);
    return mag * step;
}

std::string formatMetres(Real m) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(m < Real(1000) ? 1 : 0);
    if (std::abs(m) >= Real(1000)) {
        os << std::setprecision(2) << (m / Real(1000)) << " km";
    } else {
        os << m << " m";
    }
    return os.str();
}

}  // namespace

Renderer::Renderer() : font_(loadSystemFont()) {
    hull_.setPointCount(5);
    plume_.setPointCount(3);
    disc_.setPointCount(64);
}

void Renderer::drawGrid(sf::RenderWindow& window, const Camera& camera) {
    const sf::View  view = camera.view();
    const sf::Vector2f size   = view.getSize();
    const sf::Vector2f centre = view.getCenter();

    const Real spacing = gridSpacing(camera.widthMeters());
    const Real left    = static_cast<Real>(centre.x - size.x / 2.f);
    const Real right   = static_cast<Real>(centre.x + size.x / 2.f);
    const Real top     = static_cast<Real>(centre.y - size.y / 2.f);
    const Real bottom  = static_cast<Real>(centre.y + size.y / 2.f);

    lines_.clear();

    const Real firstX = std::floor(left / spacing) * spacing;
    for (Real x = firstX; x <= right; x += spacing) {
        // Every fifth line brighter, so the eye can judge scale and speed
        // rather than just seeing texture slide past.
        const bool major = std::fmod(std::round(x / spacing), Real(5)) == Real(0);
        addLine(lines_, {static_cast<float>(x), static_cast<float>(top)},
                {static_cast<float>(x), static_cast<float>(bottom)},
                major ? kGridMajor : kGridMinor);
    }

    const Real firstY = std::floor(top / spacing) * spacing;
    for (Real y = firstY; y <= bottom; y += spacing) {
        const bool major = std::fmod(std::round(y / spacing), Real(5)) == Real(0);
        addLine(lines_, {static_cast<float>(left), static_cast<float>(y)},
                {static_cast<float>(right), static_cast<float>(y)},
                major ? kGridMajor : kGridMinor);
    }

    window.draw(lines_);
}

void Renderer::drawBounds(sf::RenderWindow& window, const Snapshot& snap) {
    if (snap.boundsRadius <= Real(0)) return;
    disc_.setRadius(static_cast<float>(snap.boundsRadius));
    disc_.setOrigin({static_cast<float>(snap.boundsRadius), static_cast<float>(snap.boundsRadius)});
    disc_.setPosition({0.f, 0.f});
    disc_.setFillColor(sf::Color::Transparent);
    disc_.setOutlineColor(kBoundsColor);
    disc_.setOutlineThickness(static_cast<float>(snap.boundsRadius * Real(0.002)));
    window.draw(disc_);
}

void Renderer::drawAttractors(sf::RenderWindow& window, const Snapshot& snap) {
    for (const Attractor& a : snap.attractors) {
        disc_.setRadius(static_cast<float>(a.radius));
        disc_.setOrigin({static_cast<float>(a.radius), static_cast<float>(a.radius)});
        disc_.setPosition(toView(a.pos));
        disc_.setFillColor(kAttractor);
        disc_.setOutlineColor(sf::Color{120, 134, 170});
        disc_.setOutlineThickness(static_cast<float>(a.radius * Real(0.02)));
        window.draw(disc_);
    }
}

// A 12 m rocket is two pixels wide on a view framed to show a 2 km orbit, and
// the interesting thing about an orbit is the whole orbit. So the ship is drawn
// at a floor of a dozen-odd pixels however far out the camera is: its position
// stays exact, only its icon stops shrinking.
Real Renderer::visualScale(const RocketView& r, const Camera& camera) {
    constexpr Real kMinPixels = 22;
    const Real     pixels     = r.length / camera.metersPerPixel();
    return pixels >= kMinPixels ? Real(1) : kMinPixels / pixels;
}

void Renderer::drawRocket(sf::RenderWindow& window, const RocketView& r, const Camera& camera) {
    const Real  scale   = visualScale(r, camera);
    const float halfLen = static_cast<float>(r.length * scale / Real(2));
    const float halfWid = static_cast<float>(r.width * scale / Real(2));
    const float pixel   = static_cast<float>(camera.metersPerPixel());

    // Local frame: nose at +x, tail at -x. Built in metres, because the view is
    // in metres -- there is no pixel-space geometry anywhere.
    hull_.setPoint(0, {halfLen, 0.f});
    hull_.setPoint(1, {halfLen * 0.3f, -halfWid});
    hull_.setPoint(2, {-halfLen, -halfWid});
    hull_.setPoint(3, {-halfLen, halfWid});
    hull_.setPoint(4, {halfLen * 0.3f, halfWid});

    hull_.setFillColor(kHullFill);
    hull_.setOutlineColor(kHullEdge);
    hull_.setOutlineThickness(pixel * 1.5f);
    hull_.setPosition(toView(r.pos));
    hull_.setRotation(sf::degrees(toViewDegrees(r.angle)));

    // Exhaust, drawn from the nozzle and deflected by the actual gimbal command,
    // so what the fly-by-wire is doing with the nozzle is visible rather than
    // inferred.
    if (r.throttle > Real(0.001)) {
        const Real gimbalAngle = clamp(r.gimbal, Real(-1), Real(1)) * r.maxGimbal;
        const Real plumeLen    = r.length * scale * (Real(0.3) + Real(1.5) * r.throttle);
        const float pw         = halfWid * 0.8f;

        plume_.setPoint(0, {0.f, pw});
        plume_.setPoint(1, {static_cast<float>(plumeLen), 0.f});
        plume_.setPoint(2, {0.f, -pw});

        const auto alpha = static_cast<std::uint8_t>(clamp(Real(90) + Real(140) * r.throttle,
                                                           Real(0), Real(255)));
        plume_.setFillColor(sf::Color{kPlumeHot.r, kPlumeHot.g, kPlumeHot.b, alpha});
        plume_.setOutlineThickness(0.f);

        const Vec2 nozzleWorld = r.pos + rotate({-r.length * scale / Real(2), Real(0)}, r.angle);
        plume_.setPosition(toView(nozzleWorld));
        // Thrust points along the deflected nozzle; the plume points the other
        // way, hence the half turn.
        plume_.setRotation(sf::degrees(toViewDegrees(r.angle + gimbalAngle + kPi)));
        window.draw(plume_);
    }

    window.draw(hull_);
}

void Renderer::drawVectors(sf::RenderWindow& window, const RocketView& r, const Camera& camera) {
    lines_.clear();
    const Real scale = visualScale(r, camera);

    // Velocity drawn as "where you will be in one second", so its length means
    // something rather than being an arbitrary arrow. It picks up the same
    // scale as the hull, or it would vanish at the zoom levels where the hull
    // needed help in the first place.
    addLine(lines_, toView(r.pos), toView(r.pos + r.vel * scale), kVelocity);

    if (r.throttle > Real(0.001)) {
        const Real gimbalAngle = clamp(r.gimbal, Real(-1), Real(1)) * r.maxGimbal;
        const Vec2 dir = fromAngle(r.angle + gimbalAngle);
        addLine(lines_, toView(r.pos),
                toView(r.pos + dir * (r.length * scale * Real(2) * r.throttle)), kThrust);
    }

    window.draw(lines_);
}

void Renderer::drawHud(sf::RenderWindow& window, const Snapshot& snap, const Camera& camera,
                       const Hud& hud) {
    if (!font_) return;

    window.setView(window.getDefaultView());

    std::ostringstream os;
    os << std::fixed << std::setprecision(1);

    os << (hud.flyByWire ? "FLY-BY-WIRE" : "DIRECT") << "   [Start/Tab] toggle\n";

    if (!snap.rockets.empty()) {
        const RocketView& r = snap.rockets.front();
        os << "speed    " << length(r.vel) << " m/s\n";
        os << "position " << formatMetres(r.pos.x) << ", " << formatMetres(r.pos.y) << "\n";
        os << "from origin " << formatMetres(length(r.pos)) << "\n";
        os << "heading  " << (r.angle * Real(180) / kPi) << " deg   spin "
           << (r.angVel * Real(180) / kPi) << " deg/s\n";
        os << std::setprecision(2);
        os << "throttle " << r.throttle << "  gimbal " << r.gimbal << "  rcs " << r.rcs << "\n";
        os << std::setprecision(1);
    }

    os << "\nsim   " << snap.stats.tickRate << " Hz   load "
       << (snap.stats.loadFactor * Real(100)) << "%";
    if (snap.stats.catchUpResyncs > 0) os << "   resyncs " << snap.stats.catchUpResyncs;
    os << "\ntick  " << snap.tick << "   t " << snap.time << " s";
    os << "\nsnapshot age " << std::setprecision(2) << hud.snapshotAgeMs << " ms";
    os << std::setprecision(1);
    os << "\nrender " << hud.frameRate << " fps   view " << formatMetres(camera.widthMeters());
    os << "\ninput  " << hud.inputDevice;

    sf::Text text(*font_, os.str(), 14);
    text.setFillColor(kText);
    text.setPosition({12.f, 10.f});
    window.draw(text);

    if (hud.showInputDebug && hud.input) {
        std::ostringstream dbg;
        dbg << std::fixed << std::setprecision(0);
        dbg << "raw axes  ";
        static constexpr const char* kNames[] = {"X", "Y", "Z", "R", "U", "V", "PovX", "PovY"};
        for (std::size_t i = 0; i < hud.input->rawAxes.size(); ++i) {
            dbg << kNames[i] << ":" << hud.input->rawAxes[i] << "  ";
        }
        dbg << "\nbuttons   ";
        for (std::size_t i = 0; i < hud.input->rawButtons.size(); ++i) {
            if (hud.input->rawButtons[i]) dbg << i << " ";
        }

        sf::Text dbgText(*font_, dbg.str(), 13);
        dbgText.setFillColor(kTextDim);
        dbgText.setPosition({12.f, static_cast<float>(window.getSize().y) - 44.f});
        window.draw(dbgText);
    }
}

void Renderer::draw(sf::RenderWindow& window, const Snapshot& snap, const Camera& camera,
                    const Hud& hud) {
    window.clear(kBackground);
    window.setView(camera.view());

    drawGrid(window, camera);
    drawBounds(window, snap);
    drawAttractors(window, snap);

    for (const RocketView& r : snap.rockets) {
        drawVectors(window, r, camera);
        drawRocket(window, r, camera);
    }

    drawHud(window, snap, camera, hud);
}

}  // namespace rf
