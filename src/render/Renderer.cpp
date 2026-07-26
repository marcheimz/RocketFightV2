#include "render/Renderer.hpp"

#include <SFML/Graphics/Text.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>

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

// Actuator panel. The ghost is the demand, the fill is the reality, and the
// ignition colour is deliberately cool: whatever it is, it must not read as a
// smaller amount of thrust, because during ignition there is no thrust at all.
const sf::Color kPanelBg{13, 16, 24, 216};
const sf::Color kPanelEdge{40, 48, 66};
const sf::Color kBarFrame{50, 60, 80};
const sf::Color kBarGhost{124, 144, 178};
const sf::Color kBarActual{255, 150, 60};
const sf::Color kBarIgnite{104, 198, 255};
const sf::Color kFloorTick{206, 96, 96};
// The nozzle gets its own hue rather than the thrust orange: the strip answers
// "which way is it pointing", not "how hard is it pushing", and two rows of the
// same colour would invite reading the second as more of the first. The demand
// stays kBarGhost, because a demand is a demand whichever row it is in.
const sf::Color kGimbalZero{72, 84, 108};
const sf::Color kGimbalActual{176, 146, 232};

void addLine(sf::VertexArray& va, sf::Vector2f a, sf::Vector2f b, sf::Color c) {
    va.append(sf::Vertex{a, c});
    va.append(sf::Vertex{b, c});
}

// A demand, in the same hue as the reality it is meant to be compared against.
// The panel already established that demand is a ghost and reality is filled;
// a line cannot be outlined, so here the ghost is a dash pattern. Giving it a
// third colour instead would read as a third quantity rather than as the same
// quantity asked for.
//
// `dash` arrives in world units computed from metres-per-pixel, so the pattern
// is the same size on screen at every zoom -- a dash defined in metres would
// turn into a solid line the moment the camera pulled back.
void addDashedLine(sf::VertexArray& va, sf::Vector2f a, sf::Vector2f b, sf::Color c, float dash) {
    const sf::Vector2f delta = b - a;
    const float        total = std::sqrt(delta.x * delta.x + delta.y * delta.y);
    if (total <= 0.f || dash <= 0.f) return;

    const sf::Vector2f unit = delta / total;
    for (float t = 0.f; t < total; t += dash * 2.f) {
        addLine(va, a + unit * t, a + unit * std::min(t + dash, total), c);
    }
}

// A cap across the tip of a demand vector. The dashes lie on top of the solid
// line whenever the vehicle is tracking well -- which is the good case and must
// still be readable -- so the endpoint gets a mark of its own.
void addTipCap(sf::VertexArray& va, sf::Vector2f a, sf::Vector2f b, sf::Color c, float half) {
    const sf::Vector2f delta = b - a;
    const float        total = std::sqrt(delta.x * delta.x + delta.y * delta.y);
    if (total <= 0.f) return;

    const sf::Vector2f n{-delta.y / total * half, delta.x / total * half};
    addLine(va, b - n, b + n, c);
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

std::string formatKilonewtons(Real newtons) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(newtons < Real(10000) ? 1 : 0) << (newtons / Real(1000))
       << " kN";
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
    const Real     pixels     = r.spec.length / camera.metersPerPixel();
    return pixels >= kMinPixels ? Real(1) : kMinPixels / pixels;
}

void Renderer::drawRocket(sf::RenderWindow& window, const RocketView& r, const Camera& camera) {
    const Real  scale   = visualScale(r, camera);
    const float halfLen = static_cast<float>(r.spec.length * scale / Real(2));
    const float halfWid = static_cast<float>(r.spec.width * scale / Real(2));
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

    // A plume per thruster, wherever this particular vehicle happens to have
    // them, drawn from the actual mount point and along the actual deflected
    // nozzle. Attitude thrusters firing in opposed pairs are then visible as
    // exactly that, rather than as an invisible torque appearing from nowhere.
    Real strongest = 0;
    for (std::size_t i = 0; i < r.spec.count(); ++i) {
        strongest = std::max(strongest, r.spec[i].maxThrust);
    }

    for (std::size_t i = 0; i < r.spec.count(); ++i) {
        const Thruster&      t = r.spec[i];
        const ThrusterState& a = r.actuators[i];

        // Drawn from what the thruster is *doing*, not what it was told to do.
        // An engine commanded on but still counting down its ignition delay
        // correctly shows nothing, and one ramping up visibly grows.
        if (a.thrust <= Real(1e-6)) continue;
        const Real level = t.maxThrust > Real(0) ? a.thrust / t.maxThrust : Real(0);

        // Scale each plume by how much thrust that nozzle actually has, so an
        // attitude puff does not look like a main engine burn.
        const Real relative = strongest > Real(0) ? t.maxThrust / strongest : Real(1);
        const Real plumeLen =
            r.spec.length * scale * relative * (Real(0.25) + Real(1.2) * level);
        const auto pw = static_cast<float>(r.spec.width * scale * relative * Real(0.3));

        plume_.setPoint(0, {0.f, pw});
        plume_.setPoint(1, {static_cast<float>(plumeLen), 0.f});
        plume_.setPoint(2, {0.f, -pw});

        const auto alpha =
            static_cast<std::uint8_t>(clamp(Real(90) + Real(140) * level, Real(0), Real(255)));
        plume_.setFillColor(sf::Color{kPlumeHot.r, kPlumeHot.g, kPlumeHot.b, alpha});
        plume_.setOutlineThickness(0.f);

        const Vec2 mount = r.pos + rotate(t.mountLocal * scale, r.angle);
        plume_.setPosition(toView(mount));

        // Exhaust leaves opposite the force the thruster applies to the hull,
        // along the nozzle's actual angle -- which lags the commanded one.
        plume_.setRotation(
            sf::degrees(toViewDegrees(r.angle + t.directionLocal + a.gimbalAngle + kPi)));
        window.draw(plume_);
    }

    window.draw(hull_);
}

void Renderer::drawVectors(sf::RenderWindow& window, const RocketView& r, const Camera& camera,
                           const CommandView& command) {
    lines_.clear();
    const Real scale = visualScale(r, camera);

    // Velocity drawn as "where you will be in one second", so its length means
    // something rather than being an arbitrary arrow. It picks up the same
    // scale as the hull, or it would vanish at the zoom levels where the hull
    // needed help in the first place.
    addLine(lines_, toView(r.pos), toView(r.pos + r.vel * scale), kVelocity);

    // Both acceleration vectors -- the one asked for and the one arriving -- go
    // through this. Sharing the mapping is the whole reason the pair is worth
    // drawing: two arrows on different scales would only say "some" and "some".
    const auto accelLength = [&](Real accel) {
        return r.spec.length * scale * Real(2) *
               clamp(accel / std::max(r.spec.maxForwardAccel(), Real(1e-6)), Real(0), Real(1));
    };

    // Net thrust: the vector sum of every firing nozzle, in world space. On a
    // symmetric attitude burn this is correctly zero -- the ship is rotating,
    // not accelerating, and the HUD should say so.
    Vec2 net{};
    for (std::size_t i = 0; i < r.spec.count(); ++i) {
        net += rotate(r.spec[i].forceLocal(r.actuators[i].thrust, r.actuators[i].gimbalAngle),
                      r.angle);
    }

    const Real netAccel = length(net) / r.spec.mass;
    if (netAccel > Real(0.01)) {
        const Vec2 dir = normalizedOr(net, Vec2{});
        addLine(lines_, toView(r.pos), toView(r.pos + dir * accelLength(netAccel)), kThrust);
    }

    // ...and what was commanded, drawn last so it survives the lines it is being
    // compared with. Only the one that matches the mode: the two carry different
    // units, and showing both would invite reading a length in one as a length
    // in the other.
    //
    // Nothing at all when the controller has no Intent to report. Direct manual
    // flight is not a mode with a zero command in it; it is a mode with no
    // command, and an arrow of length zero would be a claim rather than silence.
    if (command.valid) {
        const auto dash = static_cast<float>(camera.metersPerPixel() * Real(7));
        const auto cap  = static_cast<float>(camera.metersPerPixel() * Real(4));

        // Below a few pixels there is no direction to read, only a smudge on the
        // hull -- which is where a commanded stop (target velocity zero) sits,
        // and the solid velocity line running away from it already says that.
        const Real floorLen = camera.metersPerPixel() * Real(4);

        Vec2      end{};
        sf::Color hue = kVelocity;

        if (command.intent.mode == Intent::Mode::Velocity) {
            // The same "where you will be in one second" convention as the
            // actual velocity, so the gap between the two lines is the tracking
            // error in metres per second, read straight off the screen.
            end = r.pos + command.intent.vector * scale;
        } else {
            // Intent::vector is normalised as a fraction of the vehicle's
            // maximum, so this is the demanded acceleration in m/s^2 -- and it
            // goes through the same mapping the achieved one just did.
            const Vec2 want = command.intent.vector * r.spec.maxForwardAccel();
            end = r.pos + normalizedOr(want, Vec2{}) * accelLength(length(want));
            hue = kThrust;
        }

        if (length(end - r.pos) > floorLen) {
            addDashedLine(lines_, toView(r.pos), toView(end), hue, dash);
            addTipCap(lines_, toView(r.pos), toView(end), hue, cap);
        }
    }

    window.draw(lines_);
}

sf::View Renderer::screenView(const sf::RenderWindow& window) {
    return sf::View(sf::FloatRect({0.f, 0.f}, sf::Vector2f(window.getSize())));
}

// One column per thruster: the demand as an empty outline, what the hardware is
// actually producing as a filled bar inside it, and under it -- for the nozzles
// that can move -- the same pair for gimbal deflection.
//
// The gap between the two is the whole point. A ControlInput is a request, and
// between asking and receiving sit an ignition delay, a thrust floor, a ramp
// rate and a nozzle slew rate -- constraints that are invisible in the world
// view, where a thruster either shows a plume or does not. Here the outline
// jumps the instant the controller commits, the fill crawls up behind it, and
// the lag the fly-by-wire has to fly around is finally something you can watch.
void Renderer::drawActuatorPanel(sf::RenderWindow& window, const Snapshot& snap) {
    if (snap.rockets.empty()) return;
    const RocketView& r     = snap.rockets.front();
    const auto        count = static_cast<float>(r.spec.count());
    if (count <= 0.f) return;

    constexpr float kSlot   = 30.f;    // pitch between columns
    constexpr float kBar    = 20.f;    // width of one envelope
    constexpr float kBarH   = 156.f;
    constexpr float kPad    = 12.f;
    constexpr float kLabel  = 122.f;   // room for a name rotated on its side

    // The deflection strip. Wider than the bar it belongs to, because the useful
    // reading is a signed offset from centre and half of 20 px is not much of a
    // scale -- but still centred on that bar, so it groups with its own engine
    // and not with its neighbour.
    constexpr float kGimbalW   = kBar + 4.f;
    constexpr float kGimbalH   = 15.f;
    constexpr float kGimbalGap = 6.f;

    // Mixed layouts are the normal case: classic and lander gimbal one main and
    // fly four fixed puffers, interceptor gimbals two. So the row is reserved
    // only when something on board can actually steer with it -- a rocket with
    // no gimbal anywhere gets no empty band -- and within that row a fixed
    // nozzle simply has no strip rather than one pinned at zero, which would
    // read as "centred" instead of "cannot move".
    bool anyGimbal = false;
    for (std::size_t i = 0; i < r.spec.count(); ++i) anyGimbal |= r.spec[i].gimballed();
    const float gimbalRow = anyGimbal ? kGimbalGap + kGimbalH : 0.f;

    // The header is the widest fixed element, so the panel is sized to whichever
    // of it and the columns is larger. Measured rather than guessed: the font is
    // whatever the system offered, and a legend that runs off the edge of its
    // own panel is worse than no legend. The second line only exists when there
    // is a second row to explain, and costs height rather than width because the
    // panel is a tall column down the side of a wide screen.
    std::optional<sf::Text> header;
    float                   headerW = 0.f;
    const float             kHeader = anyGimbal ? 34.f : 20.f;
    if (font_) {
        header.emplace(*font_,
                       anyGimbal ? "ACTUATORS  demand / actual\nbars thrust, strips nozzle"
                                 : "ACTUATORS  demand / actual",
                       12);
        headerW = header->getLocalBounds().size.x;
    }

    const sf::Vector2f screen(window.getSize());
    const float        panelW = kPad * 2.f + std::max(count * kSlot, headerW);
    const float        panelH = kPad * 2.f + kHeader + kBarH + gimbalRow + kLabel;
    const float        panelX = screen.x - panelW - 12.f;
    const float        panelY = 12.f;
    const float        colsX  = panelX + (panelW - count * kSlot) / 2.f;

    box_.setSize({panelW, panelH});
    box_.setPosition({panelX, panelY});
    box_.setFillColor(kPanelBg);
    box_.setOutlineColor(kPanelEdge);
    box_.setOutlineThickness(1.f);
    window.draw(box_);
    box_.setOutlineThickness(0.f);

    const float barTop    = panelY + kPad + kHeader;
    const float barBottom = barTop + kBarH;
    const float stripTop  = barBottom + kGimbalGap;

    auto fillBox = [&](float x, float y, float w, float h, sf::Color c) {
        box_.setSize({w, h});
        box_.setPosition({x, y});
        box_.setFillColor(c);
        window.draw(box_);
    };

    for (std::size_t i = 0; i < r.spec.count(); ++i) {
        const Thruster&      t = r.spec[i];
        const ThrusterState& a = r.actuators[i];

        const float colX = colsX + static_cast<float>(i) * kSlot + (kSlot - kBar) / 2.f;

        // Commanded on, counting down, producing nothing yet. Worth its own
        // colour precisely because there is nothing else on screen to see: no
        // plume, no bar, no motion -- just a decision that has not landed.
        const bool igniting = a.ignitionTimer > Real(0) && !a.lit;

        const Real target = clamp(r.ctrl.level[i], Real(0), Real(1)) * t.maxThrust;
        const Real inv    = t.maxThrust > Real(0) ? Real(1) / t.maxThrust : Real(0);

        const auto targetH = static_cast<float>(clamp(target * inv, Real(0), Real(1))) * kBarH;
        const auto actualH = static_cast<float>(clamp(a.thrust * inv, Real(0), Real(1))) * kBarH;

        // The full envelope, so an idle thruster still occupies its column and
        // the bars do not appear and vanish as the ship manoeuvres.
        box_.setSize({kBar, kBarH});
        box_.setPosition({colX, barTop});
        box_.setFillColor(sf::Color{20, 25, 36, 200});
        box_.setOutlineColor(kBarFrame);
        box_.setOutlineThickness(1.f);
        window.draw(box_);
        box_.setOutlineThickness(0.f);

        // Demand: an outline the fill has to catch up with. During ignition it
        // is filled instead, since that is the only moment the demand exists and
        // the reality is still exactly zero.
        if (targetH > 0.f) {
            box_.setSize({kBar - 2.f, targetH});
            box_.setPosition({colX + 1.f, barBottom - targetH});
            box_.setFillColor(igniting ? sf::Color{kBarIgnite.r, kBarIgnite.g, kBarIgnite.b, 90}
                                       : sf::Color::Transparent);
            box_.setOutlineColor(igniting ? kBarIgnite : kBarGhost);
            box_.setOutlineThickness(-1.f);   // inward, so it stays inside the envelope
            window.draw(box_);
            box_.setOutlineThickness(0.f);
        }

        // Reality.
        if (actualH > 0.f) fillBox(colX + 4.f, barBottom - actualH, kBar - 8.f, actualH, kBarActual);

        // The floor. A lit thruster can be anywhere in [min, max] and nowhere
        // below, so a fill sitting on this line is not a controller being timid
        // -- it is the vehicle refusing to go any lower.
        if (t.minThrust > Real(0)) {
            const auto floorY =
                barBottom - static_cast<float>(clamp(t.minThrust * inv, Real(0), Real(1))) * kBarH;
            fillBox(colX, floorY - 1.f, kBar, 1.5f, kFloorTick);
        }

        // The nozzle, in the same grammar one row down: the commanded deflection
        // as an outline growing out of centre, the angle the nozzle has actually
        // reached as a fill inside it. Normalised to +-maxGimbal, so the strip
        // ends mean "hard over" on any vehicle.
        //
        // A gimbal slews, it does not snap -- norcs takes 1.6 s to swing from one
        // stop to the other, which is dead time its whole rate profile is built
        // around. So the fill trailing the outline is that travel happening, and
        // a fill still on the far side of centre from the outline is a reversal
        // in progress: the vehicle is being turned the wrong way *right now* by
        // a nozzle that was told to turn it the other way.
        //
        // Drawn whether or not the engine is lit, because a nozzle has an angle
        // either way and pre-positioning it before ignition is a real tactic.
        if (t.gimballed()) {
            const float stripX  = colX + (kBar - kGimbalW) / 2.f;
            const float centreX = stripX + kGimbalW / 2.f;
            const float half    = kGimbalW / 2.f - 2.f;   // hard over stays inside the frame

            box_.setSize({kGimbalW, kGimbalH});
            box_.setPosition({stripX, stripTop});
            box_.setFillColor(sf::Color{20, 25, 36, 200});
            box_.setOutlineColor(kBarFrame);
            box_.setOutlineThickness(1.f);
            window.draw(box_);
            box_.setOutlineThickness(0.f);

            const auto demand = static_cast<float>(clamp(r.ctrl.gimbal[i], Real(-1), Real(1)));
            const auto actual =
                static_cast<float>(clamp(a.gimbalAngle / t.maxGimbal, Real(-1), Real(1)));

            if (const float w = std::abs(demand) * half; w >= 1.f) {
                box_.setSize({w, kGimbalH - 2.f});
                box_.setPosition({demand < 0.f ? centreX - w : centreX, stripTop + 1.f});
                box_.setFillColor(sf::Color::Transparent);
                box_.setOutlineColor(kBarGhost);
                box_.setOutlineThickness(-1.f);   // inward, so it stays inside the strip
                window.draw(box_);
                box_.setOutlineThickness(0.f);
            }

            if (const float w = std::abs(actual) * half; w >= 1.f) {
                fillBox(actual < 0.f ? centreX - w : centreX, stripTop + 4.f, w, kGimbalH - 8.f,
                        kGimbalActual);
            }

            // Zero, drawn last so it survives both of the above. A centred nozzle
            // is where the thing spends most of its life, and a strip with
            // nothing in it at all would read as "no reading" rather than as
            // "pointing straight".
            fillBox(centreX - 0.5f, stripTop + 1.f, 1.f, kGimbalH - 2.f, kGimbalZero);
        }
    }

    if (!font_) return;   // bars without labels still beat no panel at all

    header->setFillColor(kTextDim);
    header->setPosition({panelX + kPad, panelY + kPad - 4.f});
    window.draw(*header);

    for (std::size_t i = 0; i < r.spec.count(); ++i) {
        const Thruster&      t = r.spec[i];
        const ThrusterState& a = r.actuators[i];

        const float colX = colsX + static_cast<float>(i) * kSlot + (kSlot - kBar) / 2.f;

        // Rotated onto its side: a 30 px column cannot hold "nose-right" any
        // other way, and a column too narrow to label is a column nobody reads.
        sf::Text label(*font_, std::string(t.name.view()) + "   " + formatKilonewtons(t.maxThrust),
                       10);
        label.setFillColor(a.thrust > Real(0) || a.ignitionTimer > Real(0) ? kText : kTextDim);
        label.setRotation(sf::degrees(-90.f));
        label.setPosition({colX + 4.f, panelY + panelH - kPad});
        window.draw(label);
    }
}

void Renderer::drawHud(sf::RenderWindow& window, const Snapshot& snap, const Camera& camera,
                       const Hud& hud) {
    if (!font_) return;

    std::ostringstream os;
    os << std::fixed << std::setprecision(1);

    os << (hud.flyByWire ? "FLY-BY-WIRE" : "DIRECT") << "   [Start/Tab] toggle\n";

    // Which layer 2 is flying, and whether it is one of yours. Named on every
    // frame rather than only when it changes, because "what is actually
    // controlling this ship" is the question the whole submission feature
    // exists to let you ask.
    if (!snap.flyByWire.name.empty()) {
        os << "control  " << std::string(snap.flyByWire.name.view());
        if (snap.flyByWire.submission) os << " (submission)";
        if (snap.flyByWire.loadFailed) os << "  -- FAILED TO LOAD, flying built-in";
        os << "   [D-pad / , .]\n";
    }
    os << (hud.benchmark.velocityMode ? "VELOCITY  stick = target m/s"
                                      : "ACCELERATION  stick = fraction of max")
       << "   [Y/V] toggle\n";

    if (!snap.rockets.empty()) {
        const RocketView& r = snap.rockets.front();
        os << std::string(r.spec.name.view()) << "   [LB/RB or [ ] ] vehicle\n";
        os << "speed    " << length(r.vel) << " m/s\n";
        os << "position " << formatMetres(r.pos.x) << ", " << formatMetres(r.pos.y) << "\n";
        os << "from origin " << formatMetres(length(r.pos)) << "\n";
        os << "heading  " << (r.angle * Real(180) / kPi) << " deg   spin "
           << (r.angVel * Real(180) / kPi) << " deg/s\n";

        os << std::setprecision(2);
        Real actual = 0;
        Real capacity = 0;
        for (std::size_t i = 0; i < r.spec.count(); ++i) {
            actual   += r.actuators[i].thrust;
            capacity += r.spec[i].maxThrust;
        }
        // Just the aggregate here. Per-thruster demand versus reality used to be
        // a row of '#' and '+' characters; the actuator panel now says the same
        // thing with a resolution that ASCII could not.
        os << "thrust   " << (capacity > Real(0) ? actual / capacity : Real(0)) << "\n";
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

    // The benchmark replaces the pilot outright while it runs, so this has to
    // be impossible to miss: a dead stick with no explanation looks exactly like
    // a controller that has crashed.
    if (hud.benchmark.active) {
        const Renderer::Benchmark& b = hud.benchmark;

        std::ostringstream bs;
        bs << std::fixed << std::setprecision(2);
        bs << "BENCHMARK RUNNING -- pilot input ignored   [B] stop\n";
        bs << std::setprecision(1) << "t " << b.seconds << " s   steps " << b.stepsSettled << "/"
           << b.steps << " settled\n";
        bs << std::setprecision(2);
        bs << "tracking err  " << b.trackingErrorMean
           << (hud.benchmark.velocityMode ? " m/s mean" : " m/s^2 mean") << "   now "
           << b.currentError << "\n";
        bs << "settling      " << b.settlingMean << " s mean\n";
        bs << std::setprecision(0);
        bs << "impulse       " << b.impulse << " N.s\n";
        bs << std::setprecision(2);
        bs << "att. wander   " << b.attitudeWander << " rad";

        sf::Text benchText(*font_, bs.str(), 14);
        // Warm, so it reads as "something else is flying" rather than as another
        // row of telemetry.
        benchText.setFillColor(sf::Color{255, 190, 90});
        // Sits above the input debug overlay when that is up, rather than
        // through it.
        const float bottom =
            static_cast<float>(window.getSize().y) - (hud.showInputDebug ? 54.f : 0.f);
        benchText.setPosition({12.f, bottom - 130.f});
        window.draw(benchText);
    }

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

    // The command belongs to the controlled rocket alone. Anything else in the
    // world is flying itself and has none to show here.
    static constexpr CommandView kNoCommand{};

    for (std::size_t i = 0; i < snap.rockets.size(); ++i) {
        drawVectors(window, snap.rockets[i], camera, i == 0 ? snap.command : kNoCommand);
        drawRocket(window, snap.rockets[i], camera);
    }

    // Everything from here is measured in pixels, not metres.
    window.setView(screenView(window));
    drawActuatorPanel(window, snap);
    drawHud(window, snap, camera, hud);
}

}  // namespace rf
