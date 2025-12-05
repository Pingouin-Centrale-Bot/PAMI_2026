#include "DriverInterface.hpp"
#include "Config.hpp"
#include <Arduino.h>

/**
 * Implémentation générique d’un driver pour moteurs DC
 * utilisant 1 pin PWM + 1 pin direction par moteur.
 *
 * Hypothèse : chaque moteur utilise un canal PWM ESP32 (ledc),
 * et le driver possède un pin ENABLE commun.
 *
 * Cette classe est un exemple concret qui peut être adaptée
 * selon ton hardware (TB6612, L298N, BTS7960, etc.).
 */

class BasicDriver : public IDriver {
public:
    BasicDriver(int pwmPin, int dirPin, int enablePin)
        : pwmPin_(pwmPin), dirPin_(dirPin), enablePin_(enablePin)
    {}

    void init(int channel) {
        channel_ = channel;

        pinMode(dirPin_, OUTPUT);
        pinMode(enablePin_, OUTPUT);

        // Configuration du PWM sur ESP32
        ledcSetup(channel_, cfg::PWM_FREQUENCY, cfg::PWM_RESOLUTION);
        ledcAttachPin(pwmPin_, channel_);

        disable();   // Par sécurité au boot
    }

    /**
     * @brief Applique le PWM signé.
     * pwmValue ∈ [-MAX_PWM ; +MAX_PWM]
     */
    void setPWM(int channel, int pwmValue) override {
        if (channel != channel_) return;  // sécurité

        // Sens en fonction du signe
        if (pwmValue >= 0) {
            digitalWrite(dirPin_, HIGH);
            ledcWrite(channel_, pwmValue);
        } else {
            digitalWrite(dirPin_, LOW);
            ledcWrite(channel_, -pwmValue);
        }
    }

    void enable() override {
        digitalWrite(enablePin_, HIGH);
    }

    void disable() override {
        digitalWrite(enablePin_, LOW);
        ledcWrite(channel_, 0);  // arrêt PWM
    }

private:
    int pwmPin_;
    int dirPin_;
    int enablePin_;
    int channel_;
};
