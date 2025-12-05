// Ce module gère un encodeur incrémental à quadrature (signaux A et B), utilisé pour mesurer la position et 
// la vitesse d’un moteur.
// Il fournit des méthodes pour compter les impulsions, convertir en révolutions et calculer une vitesse approximative.

// Ce que fait ce module :
// Configure les interruptions nécessaires pour lire les signaux A/B.
// Incrémente ou décrémente un compteur selon le sens de rotation.
// Fournit des fonctions pour accéder à la position, la vitesse et réinitialiser le compteur.
// Propose une interface minimale pour être utilisée avec un contrôleur ou une stratégie.

#pragma once
#include <cstdint>
class Encoder {
public:
  Encoder(int pinA, int pinB, int pulsesPerRev);
  void init();
  long getCount(); // atomic-ish
  void reset();
  double getRevolutions(); // count / pulsesPerRev
  double getAngularVelocity(); // RPM or rad/s (filter internally)
  void tickA(); // ISR-friendly
  void tickB();
private:
  volatile long count_;
  int ppr_;
  unsigned long lastTs_;
  long lastCount_;
  int pinA_;
  int pinB_;
  // optional: simple filter state
};
