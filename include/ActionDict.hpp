#pragma once
#include <vector>
#include <string>
#include <map>

struct Action {
    std::vector<std::vector<double>> data; // Liste de poses (angles ou coords)
    std::string type;                      // "legs", "head", ou "tail"
};

class ActionDict {
public:
    ActionDict();
    Action get_action(const std::string& name);

private:
    // Les fonctions qui génèrent les mouvements
    Action stand();
    Action sit();
    Action lie();
    Action wag_tail();
    
    int height = 95;
    int barycenter = -15;
};
