// Ce module fournit un moyen simple de transmettre des informations à un PC via la liaison série, pour débugger 
// les moteurs, les capteurs ou l’asservissement.

// Ce que fait ce module :
// Initialise une communication série.
// Envoie périodiquement l’état du robot (positions, vitesses, PWM, etc.).
// Simplifie l’affichage des valeurs pour faciliter les tests.

#pragma once
class Telemetry {
public:
  Telemetry();
  void init();
  void publishStatus(double t, const double* state, int n);
  void sendHeartbeat();
};
