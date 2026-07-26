#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "control/Controller.hpp"

namespace rf {

// Name -> factory. This is the seam an agent writes against: add a file, add one
// registration line, and the harness can run the controller by name without any
// other part of the tree knowing it exists.
//
// Registration is explicit rather than done by static initialisers. Self-
// registration inside a static library is a well-known trap: the linker drops
// object files nothing references, and the controllers quietly vanish.
class ControllerRegistry {
public:
    using Factory = std::function<std::unique_ptr<Controller>()>;

    static ControllerRegistry& instance();

    void add(std::string name, Factory factory);

    // Returns nullptr for an unknown name; callers decide whether that is fatal.
    std::unique_ptr<Controller> create(const std::string& name) const;

    std::vector<std::string> names() const;

private:
    ControllerRegistry();

    std::vector<std::pair<std::string, Factory>> entries_;
};

}  // namespace rf
