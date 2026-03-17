#include "ActionDict.hpp"
#include "Pidog.hpp" // Pour accéder à legs_angle_calculation

Action ActionDict::get_action(const std::string& name) {
    if (name == "forward")    return forward();
    if (name == "backward")   return backward();
    if (name == "turn_left")  return turn_left();
    if (name == "turn_right") return turn_right();
    if (name == "trot")       return trot();
    if (name == "stand")      return stand();
    if (name == "sit")        return sit();
    if (name == "lie")        return lie();
    if (name == "wag_tail")   return wag_tail();
    
    return {{}, "none"};
}

// Exemple d'implémentation de forward
Action ActionDict::forward() {
    Action act;
    act.type = "legs";
    
    Walk walker(Walk::FORWARD, Walk::STRAIGHT);
    auto coords = walker.get_coords();
    
    for (auto& pose : coords) {
        // IMPORTANT : Ton Pidog::legs_angle_calculation doit être statique 
        // ou accessible ici pour transformer les (y,z) en angles servos.
        act.data.push_back(Pidog::legs_angle_calculation(pose));
    }
    return act;
}
