#include "Gait.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --- Fonctions Communes ---
double Gait::step_y_func(int leg, int step, int step_count, double leg_step_width, double leg_origin, int fb) {
    double theta = step * M_PI / (step_count - 1);
    double temp = (leg_step_width * (std::cos(theta) - fb) / 2.0 * fb);
    return leg_origin + temp;
}

double Gait::step_z_func(int step, int step_count, double z_origin, double leg_step_height) {
    return z_origin - (leg_step_height * step / (step_count - 1));
}

// --- Implémentation TROT ---
Trot::Trot(int _fb, int _lr) : fb(_fb), lr(_lr) {
    double center_of_gravity = -17.0;
    double leg_step_width_base = 100.0;
    double y_offset = center_of_gravity;

    // Logique de décalage selon direction (traduit du Python)
    if (fb == FORWARD) y_offset += (lr == STRAIGHT ? 0 : -2);
    else if (fb == BACKWARD) y_offset += (lr == STRAIGHT ? 8 : 1);

    double scales[3][4] = {{0.5, 1, 0.5, 1}, {1, 1, 1, 1}, {1, 0.5, 1, 0.5}};
    int lr_idx = lr + 1; // -1,0,1 -> 0,1,2
    double dirs[4] = {-1, -1, 1, 1};
    double stand_offset = 5.0;

    for (int i = 0; i < 4; ++i) {
        double width = leg_step_width_base * scales[lr_idx][i];
        leg_step_width.push_back(width);
        section_length.push_back(width / (2.0 - 1.0)); // SECTION_COUNT = 2
        step_down_length.push_back(section_length[i] / 3.0); // STEP_COUNT = 3
        leg_origin.push_back(width / 2.0 + y_offset + (stand_offset * dirs[i] * scales[lr_idx][i]));
    }
}

std::vector<Pose> Trot::get_coords() {
    std::vector<Pose> all_poses;
    int section_count = 2;
    int step_count = 3;
    int leg_raise_order[2][2] = {{1, 4}, {2, 3}};
    double leg_table[4] = {0, 1, 1, 0};

    Pose current_pose(4);
    for(int i=0; i<4; i++) {
        current_pose[i] = {leg_origin[i] - leg_table[i] * section_length[i], z_origin};
    }

    for (int s = 0; s < section_count; ++s) {
        for (int step = 0; step < step_count; ++step) {
            Pose step_pose(4);
            int s_idx = (fb == 1) ? s : (section_count - s - 1);
            
            for (int i = 0; i < 4; ++i) {
                bool raise = (i+1 == leg_raise_order[s_idx][0] || i+1 == leg_raise_order[s_idx][1]);
                if (raise) {
                    step_pose[i].y = step_y_func(i, step, step_count, leg_step_width[i], leg_origin[i], fb);
                    step_pose[i].z = step_z_func(step, step_count, z_origin, leg_step_height);
                } else {
                    step_pose[i].y = current_pose[i].y + step_down_length[i] * fb;
                    step_pose[i].z = z_origin;
                }
            }
            current_pose = step_pose;
            all_poses.push_back(step_pose);
        }
    }
    return all_poses;
}

// --- Implémentation WALK ---
// Note : La logique est quasi identique, seules les constantes (LEG_ORDER, SECTION_COUNT) changent.
// Pour gagner de la place, je résume la structure, c'est le même principe que Trot.
