#pragma once
#include <vector>
#include <string>
#include <map>
#include "Gait.hpp" // <--- Indispensable pour utiliser Walk et Trot

struct Action {
    std::vector<std::vector<double>> data;
    std::string type;
};

class ActionDict {
public:
    ActionDict();
    Action get_action(const std::string& name);

private:
    // Méthodes de haut niveau (Calculées via Gait)
    Action forward();    // <--- Ajouté
    Action backward();   // <--- Ajouté
    Action turn_left();  // <--- Ajouté
    Action turn_right(); // <--- Ajouté
    Action trot();       // <--- Ajouté

    // Poses fixes (Hardcodées comme dans le Python original)
    Action stand();
    Action sit();
    Action lie();
    Action wag_tail();
    
    int height = 95;
    int barycenter = -15;
};
