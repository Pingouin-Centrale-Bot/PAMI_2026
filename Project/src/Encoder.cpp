#include "Encoder.hpp"
#include <Arduino.h>

Encoder::Encoder(int pinA, int pinB, int pulsesPerRev)
    : pinA_(pinA),
      pinB_(pinB),
      count_(0),
      ppr_(pulsesPerRev),
      lastTs_(0),
      lastCount_(0)
{}

void Encoder::init() {
    pinMode(pinA_, INPUT_PULLUP);
    pinMode(pinB_, INPUT_PULLUP);

    // Enregistre les interruptions
    attachInterrupt(digitalPinToInterrupt(pinA_), [this]() { this->tickA(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinB_), [this]() { this->tickB(); }, CHANGE);

    lastTs_ = micros();
    lastCount_ = 0;
}

long Encoder::getCount() {
    noInterrupts();
    long c = count_;
    interrupts();
    return c;
}

void Encoder::reset() {
    noInterrupts();
    count_ = 0;
    lastCount_ = 0;
    lastTs_ = micros();
    interrupts();
}

double Encoder::getRevolutions() {
    return static_cast<double>(getCount()) / static_cast<double>(ppr_);
}

// -----------------------------------------------------------------------------
// Calcul de vitesse
// On calcule une vitesse filtrée (simple filtre passe-bas)
// Δθ = variation d'impulsions * (2π/ppr)
// Δt = micros()
// ω = Δθ / Δt  (rad/s)
//
// Le résultat final peut être renvoyé en rad/s ou RPM.
// Ici je renvoie **rad/s** car plus propre scientifiquement.
// -----------------------------------------------------------------------------
// Un filtre exponentiel simple est utilisé :
// ω_filtered = 0.7 * ω_filtered + 0.3 * ω_instant
// -----------------------------------------------------------------------------

double Encoder::getAngularVelocity() {
    unsigned long now = micros();
    unsigned long dt = now - lastTs_;  // µs

    if (dt < 1000) {
        return 0.0; // trop court pour mesurer
    }

    long currentCount = getCount();
    long dc = currentCount - lastCount_;

    // Conversion impulsions → rad
    double dtheta = (static_cast<double>(dc) * 2.0 * PI) / static_cast<double>(ppr_);

    // Δt en secondes
    double dt_s = static_cast<double>(dt) * 1e-6;

    double omega = dtheta / dt_s;  // rad/s

    // Filtrage
    static double omega_f = 0.0;
    omega_f = 0.7 * omega_f + 0.3 * omega;

    lastCount_ = currentCount;
    lastTs_ = now;

    return omega_f; // rad/s
}

// -----------------------------------------------------------------------------
// ISR quadrature logic
// Ces fonctions doivent être **extrêmement rapides**.
// On lit A et B, et on incrémente/décrémente selon l'état.
// -----------------------------------------------------------------------------

void Encoder::tickA() {
    bool a = digitalRead(pinA_);
    bool b = digitalRead(pinB_);

    if (a == b) {
        count_++;
    } else {
        count_--;
    }
}

void Encoder::tickB() {
    bool a = digitalRead(pinA_);
    bool b = digitalRead(pinB_);

    if (a != b) {
        count_++;
    } else {
        count_--;
    }
}
