#include "Pidog.hpp"
#include <iostream>

int main() {
    try {
        std::cout << "Initialisation du PiDog..." << std::endl;
        Pidog my_dog;

        // On démarre les threads de mouvement
        my_dog.start_action_threads();

        std::cout << "Le robot va se mettre en position 'Couché' dans 2 secondes..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Test d'un mouvement simple
        my_dog.stop_and_lie(80);

        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        my_dog.close();
        std::cout << "Test terminé avec succès !" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "ERREUR CRITIQUE : " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
