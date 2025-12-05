// Ce fichier centralise tous les paramètres matériels et constants du projet : numéros de pins, fréquences PWM, 
// résolution, période de contrôle, etc.

// Ce que fait ce module :
// Regroupe les valeurs critiques au même endroit.
// Facilite la modification du matériel sans casser le code.
// Rend le projet plus propre et plus lisible.

#pragma once
namespace cfg {
  constexpr unsigned long CONTROL_PERIOD_MS = 10; // 100 Hz
  constexpr int PWM_FREQUENCY = 20000; // 20 kHz
  constexpr int PWM_RESOLUTION = 8;    // 8-bit by default
  constexpr int MAX_PWM = (1<<PWM_RESOLUTION)-1;
  // pins (valeurs par défaut à changer)
  constexpr int M1_PWM_PIN = 18;
  constexpr int M1_DIR_PIN = 19;
  constexpr int M1_ENC_A = 34;
  constexpr int M1_ENC_B = 35;
  constexpr int M2_PWM_PIN = 21;
  constexpr int M2_DIR_PIN = 22;
  constexpr int M2_ENC_A = 32;
  constexpr int M2_ENC_B = 33;
  constexpr int OBSTACLE_PIN = 25; // ex: digitale pour capteur IR/trigger echo pour HC-SR04
}
