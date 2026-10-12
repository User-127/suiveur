import math




def to_polarcoord(ranges, angle_min, angle_increment):
    """Convertit les mesures laser en coordonnées polaires (angle, distance)."""
    polar_coords = []
    for index, distance in enumerate(ranges):
        if math.isfinite(distance):
            angle = angle_min + index * angle_increment
            polar_coords.append(PolarCoord(angle, distance))
    return polar_coords

class PolarCoord:
    def __init__(self, angle, distance):
        self.angle = angle
        self.distance = distance


def filter_valid_polars(polars: list[PolarCoord]):
    """Filtre les coordonnées polaires pour ne garder que celles avec des distances finies."""
    return [polar for polar in polars if math.isfinite(polar.distance)]


def minimum_distance_angle(polars: list[PolarCoord]):
    """Retourne l'angle de la distance minimale parmi les mesures valides."""
    valid_ranges = filter_valid_polars(polars)

    minimum = min(valid_ranges, key=lambda item: item.distance, default=None)
    if minimum is None:
        return None
    return minimum.angle


def minimum_distance(polars: list[PolarCoord]):
    """Retourne la distance minimale parmi les mesures laser valides."""
    valid_ranges = filter_valid_polars(polars)
    return min(valid_ranges, key=lambda item: item.distance, default=None)


class CalculateurCerveauOrientation:
    """Calcule une correction proportionnelle de l'orientation du robot."""

    def __init__(self, gain_proportionnel=0.5):
        """Initialise le calculateur avec son gain proportionnel."""
        self.erreur = 0.0
        self.gain_proportionnel = gain_proportionnel
        self.angle_vue = math.radians(60.0)  # Angle de vue par défaut en radians

    def calculer_correction(self):
        """Retourne la correction basée sur la dernière erreur calculée."""
        return self.gain_proportionnel * self.erreur

    def mettre_a_jour_erreur(self, ranges, angle_min, angle_increment):
        """Met à jour l'erreur avec l'angle de la mesure laser minimale."""
        polars = to_polarcoord(ranges, angle_min, angle_increment)
        angle = minimum_distance_angle(polars)
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

    def mettre_a_jour_erreur(self, ranges, angle_min, angle_increment):
        """Met à jour l'erreur avec la distance minimale mesurée par le laser."""
        polars = to_polarcoord(ranges, angle_min, angle_increment)
        distance = minimum_distance(polars)
        if distance is None:
            return None

        self.erreur = self.distance_cible - distance
        return distance
