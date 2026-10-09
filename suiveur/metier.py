import math


def minimum_distance_angle(ranges, angle_min, angle_increment):
    """Retourne l'angle de la distance minimale parmi les mesures valides."""
    valid_ranges = (
        (index, distance)
        for index, distance in enumerate(ranges)
        if math.isfinite(distance)
    )
    minimum = min(valid_ranges, key=lambda item: item[1], default=None)
    if minimum is None:
        return None
    return angle_min + minimum[0] * angle_increment


def minimum_distance(ranges):
    """Retourne la distance minimale parmi les mesures laser valides."""
    valid_ranges = (distance for distance in ranges if math.isfinite(distance))
    return min(valid_ranges, default=None)


class CalculateurCerveauOrientation:
    """Calcule une correction proportionnelle de l'orientation du robot."""

    def __init__(self, gain_proportionnel=0.5):
        """Initialise le calculateur avec son gain proportionnel."""
        self.erreur = 0.0
        self.gain_proportionnel = gain_proportionnel

    def calculer_correction(self):
        """Retourne la correction basée sur la dernière erreur calculée."""
        return self.gain_proportionnel * self.erreur

    def mettre_a_jour_erreur(self, ranges, angle_min, angle_increment):
        """Met à jour l'erreur avec l'angle de la mesure laser minimale."""
        angle = minimum_distance_angle(ranges, angle_min, angle_increment)
        if angle is None:
            return None

        self.erreur = angle - math.pi
        return angle


class CalculateurCerveauDistance:
    """Classe pour calculer la distance minimale à partir des données du laser."""

    def __init__(self, gain_proportionnel=0.5, distance_cible=0.5):
        """Initialise le calculateur avec une erreur initiale de 0."""
        self.erreur = 0.0
        self.gain_proportionnel = gain_proportionnel
        self.distance_cible = distance_cible

    def calculer_correction(self):
        """Retourne la correction basée sur la dernière erreur calculée."""
        return self.gain_proportionnel * self.erreur

    def mettre_a_jour_erreur(self, ranges):
        """Met à jour l'erreur avec la distance minimale mesurée par le laser."""
        distance = minimum_distance(ranges)
        if distance is None:
            return None

        self.erreur = self.distance_cible - distance
        return distance
