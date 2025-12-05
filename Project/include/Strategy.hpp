// Ce module définit la logique de "décision" du robot : comment réagir aux obstacles, quelles 
// consignes envoyer aux moteurs, etc.

// Ce que fait ce module :
// Centralise les règles de comportement (avancer, s’arrêter, contourner, etc.).
// Prend en entrée les mesures (vitesse, obstacle).
// Produit les consignes moteur sous forme d’un tableau [cmd1, cmd2].

#pragma once
class Strategy {
public:
  Strategy();
  void init();
  // returns target commands for motors based on sensors
  std::array<double,2> decide(double current1, double current2, bool obstacleDetected);
};
