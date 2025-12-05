// Ce module décrit une interface simple pour un capteur d’obstacle, qu’il soit infrarouge, ultrasonique ou 
// même un simple détecteur de contact.

// Ce que fait ce module :
// Propose une classe Sensor qui lit un pin d’entrée ou effectue une mesure.
// Permet d’abstraire la nature du capteur pour que la logique du robot reste la même.
// Peut être étendu pour gérer plusieurs types de capteurs.