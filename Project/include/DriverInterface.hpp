// Ce fichier définit l’interface abstraite IDriver, permettant de séparer la logique du moteur de 
// l’implémentation réelle du driver matériel.
// Il assure que le code côté moteur reste portable, propre et testable.

// Ce que fait ce module :
// Définit trois fonctions virtuelles : setPWM(), enable(), disable().
// Permet d’implémenter différents drivers (L298N, BTS7960, TB6612FNG, etc.) sans modifier le reste du code.
// Sert de base pour que la classe Motor commande le matériel de manière uniforme.

#pragma once
class IDriver {
public:
  virtual ~IDriver() = default;
  virtual void setPWM(int channel, int pwmValue) = 0; // pwmValue in [-MAX_PWM, MAX_PWM]
  virtual void enable() = 0;
  virtual void disable() = 0;
};
