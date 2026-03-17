# 1. Version minimale de CMake
cmake_minimum_required(VERSION 3.10)

# 2. Nom de ton projet
project(PiDog_CPP)

# 3. Utiliser le standard C++ 17 (pour avoir accès aux outils modernes)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 4. Chercher la bibliothèque Eigen3 (pour les matrices)
find_package(Eigen3 REQUIRED)

# 5. Définir où sont tes fichiers d'en-tête (.hpp)
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

# 6. Lister tous tes fichiers source (.cpp)
set(SOURCES
    src/main.cpp
    src/Pidog.cpp
    src/Robot.cpp
    src/PWM.cpp
)

# 7. Créer l'exécutable final (nommé "pidog_run")
add_executable(pidog_run ${SOURCES})

# 8. Lier les bibliothèques nécessaires (i2c et threads)
# Le robot a besoin de "pthread" pour gérer les mouvements en parallèle
target_link_libraries(pidog_run i2c pthread)
