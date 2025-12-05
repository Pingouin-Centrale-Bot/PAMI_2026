// Ce module fournit une première version d’un contrôleur simple (type PI ou PID) pour réguler la 
// vitesse ou la position d’un moteur.

// Ce que fait ce module :
// Implémente un contrôleur numérique discret (PI ou PID).
// Prend en entrée une consigne et une mesure.
// Retourne une commande PWM limitée.
// Stocke les intégrales et dérivées nécessaires au calcul.

#pragma once
struct ControlOutput { double pwm_m1; double pwm_m2; };
class Controller {
public:
  Controller();
  void init();
  // call at CONTROL_PERIOD_MS
  ControlOutput update(double target1, double target2, double meas1, double meas2);
  void setGains(double kp, double ki, double kd = 0.0);
  void reset();
private:
  double kp_, ki_, kd_;
  double integral1_, integral2_;
  double lastErr1_, lastErr2_;
};
