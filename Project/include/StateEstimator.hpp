// Ce module fournit le squelette d’un observateur d’état, utilisé pour estimer position et vitesse 
// des moteurs à partir d’un modèle dynamique.
// Il sert plus tard si tu veux utiliser un contrôle par retour d’état au lieu d’un PID.

// Ce que fait ce module :
// Fournit une structure pour gérer un modèle linéaire simple.
// Propose deux fonctions : predict() (modèle) et correct() (mesure).
// Retourne un vecteur d’états estimés.

#pragma once
#include <array>
class StateEstimator {
public:
  StateEstimator();
  void init();
  // x: state vector pointer, y: measurement pointer
  void predict(double dt, const std::array<double,2>& u);
  void correct(const std::array<double,2>& y);
  std::array<double,4> getState() const; // example: [pos1, vel1, pos2, vel2]
};
