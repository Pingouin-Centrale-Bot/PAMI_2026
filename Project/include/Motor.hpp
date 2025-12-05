// Ce module définit la classe Motor, qui représente un moteur à courant continu contrôlé via un driver (pont en H ou module PWM).
// La classe fournit une interface simple pour régler la puissance du moteur (via une commande PWM), 
// définir le sens de rotation et réaliser un arrêt d’urgence.

// Ce que fait ce module :

// Configure un canal PWM sur l’ESP32 (via ledc).
// Applique une consigne de puissance en gérant automatiquement le sens (avant/arrière).
// Limite les valeurs pour éviter les dépassements.
// Offre un moyen d’arrêter immédiatement le moteur en cas de problème.

#pragma once
#include <cstdint>
#include "DriverInterface.hpp"

class Motor {
public:
  Motor(IDriver& driver, int channel);
  void init();
  void setDuty(int pwm); // [-MAX_PWM, MAX_PWM]
  int getDuty() const;
  void emergencyStop();
private:
  IDriver& driver_;
  int channel_;
  int duty_;
};
