#include "Motor.hpp"
#include "Config.hpp"          // pour MAX_PWM
#include <Arduino.h>           // pour ledc functions si utilisé
#include <algorithm>           // pour std::clamp

Motor::Motor(IDriver& driver, int channel)
    : driver_(driver),
      channel_(channel),
      duty_(0)
{
}

/**
 * @brief Initialise le moteur.
 * Configure le canal PWM et active le driver.
 */
void Motor::init() {
    // Configuration du canal PWM via l'ESP32
    // (Hypothèse : le driver utilise ledc en interne ou s'appuie sur ces configs)
    ledcSetup(channel_, cfg::PWM_FREQUENCY, cfg::PWM_RESOLUTION);

    // Le driver doit savoir à quelle pin est attaché ce canal
    // (ex : dans ton driver, setPWM pourra appeler ledcWrite(channel, value))
    // Ici on n'attache pas de pin directement, c'est le driver qui le fait.
    
    driver_.enable();   // Active le driver (met EN pin à HIGH par ex.)
}

/**
 * @brief Applique une commande PWM au moteur.
 * @param pwm Valeur entre -MAX_PWM et +MAX_PWM
 */
void Motor::setDuty(int pwm) {
    // Clamp pour éviter les débordements
    pwm = std::clamp(pwm, -cfg::MAX_PWM, cfg::MAX_PWM);

    duty_ = pwm;

    // Appel au driver : il gère le sens + PWM
    driver_.setPWM(channel_, pwm);
}

/**
 * @brief Lit la dernière valeur PWM demandée.
 */
int Motor::getDuty() const {
    return duty_;
}

/**
 * @brief Arrêt immédiat du moteur.
 */
void Motor::emergencyStop() {
    duty_ = 0;
    driver_.setPWM(channel_, 0);
    driver_.disable();     // coupe totalement le moteur
}
