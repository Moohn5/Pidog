#include "ActionDict.hpp"
#include <cmath>

Action ActionDict::get_action(const std::string& name) {
    if (name == "stand") return stand();
    if (name == "sit") return sit();
    if (name == "lie") return lie();
    if (name == "wag_tail") return wag_tail();
    return {{}, "none"};
}

Action ActionDict::stand() {
    // Traduction de : [[x, y], [x, y], [x+20, y-5], [x+20, y-5]]
    double x = (double)barycenter;
    double y = (double)height;
    return { {{x, y, x, y, x+20, y-5, x+20, y-5}}, "legs" };
}

Action ActionDict::sit() {
    return { {{30, 60, -30, -60, 80, -45, -80, 45}}, "legs" };
}

Action ActionDict::wag_tail() {
    return { {{-30}, {30}}, "tail" };
}
