#include "okapi/api.hpp"

using namespace okapi;

/**
 * @brief
 *
 * @param drive Pointeur sur le mouvement du robot
 * @param ultraSonicFront Pointeur sur le senseur
 * @param ultraSonicLeft Pointeur sur le senseur
 * @param ultraSonicRight Pointeur sur le senseur
 * @param xGoal difference entre le robot et l'objectif x
 * @param yGoal difference entre le robot et l'objectif y
 * @param tetaGoal difference angulaire entre le robot et l'objectif
 */

// Ouverture: si on revient sur la meme position deux fois on tourne a gauche
// Ajouter en entree le gyroscope du robot pour mesure
// teta se calcule tout seul et on peut donc l'enlever
// check la dessus : https://www.cs.cmu.edu/~motionplanning/lecture/Chap2-Bug-Alg_howie.pdf
void bug_0(std::shared_ptr<ChassisController> drive,ADIUltrasonic ultraSonicFront,ADIUltrasonic ultraSonicLeft,ADIUltrasonic ultraSonicRight, int xGoal,int yGoal, int tetaGoal){
    int currentPosX = xGoal;
    int currentPosY = yGoal;
    int currentPosTeta = tetaGoal;
    // TODO - Implement this algo


    // Rotates towards goal
    // while not arrived
        // Forward
        // if obstacle encountered (< front threshold)
            // while no obstacle on left
                // while < front threshold or < left threshold
                    // rotate right
                // Forward
}


/**
 * @brief
 *
 * @param drive Pointeur sur le mouvement du robot
 * @param ultraSonicFront Pointeur sur le senseur
 * @param ultraSonicLeft Pointeur sur le senseur
 * @param ultraSonicRight Pointeur sur le senseur
 * @param xGoal difference entre le robot et l'objectif x
 * @param yGoal difference entre le robot et l'objectif y
 * @param tetaGoal difference angulaire entre le robot et l'objectif
 */
void bug_1(std::shared_ptr<ChassisController> drive,ADIUltrasonic ultraSonicFront,ADIUltrasonic ultraSonicLeft,ADIUltrasonic ultraSonicRight, int xGoal,int yGoal, int tetaGoal){
{
    // Rotates towards goal
}
