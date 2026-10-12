import math


def wrap_angle_symetric(angle):
    """Retourne l'angle dans l'intervalle [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi

def to_polarcoord(ranges, angle_min, angle_increment):
    """Convertit les mesures laser en coordonnées polaires (angle, distance) avec des angles dans l'intervalle [-pi, pi]."""
    polar_coords: list[PolarCoord] = []
    for index, distance in enumerate(ranges):
        if math.isfinite(distance):
            angle = wrap_angle_symetric(angle_min + index * angle_increment)
            polar_coords.append(PolarCoord(angle, distance))
    return polar_coords

class PolarCoord:
    def __init__(self, angle, distance):
        self.angle = angle
        self.distance = distance


def filter_valid_polars(polars: list[PolarCoord]):
    """Filtre les coordonnées polaires pour ne garder que celles avec des distances finies."""
    return [polar for polar in polars if math.isfinite(polar.distance)]


def minimum_distance_angle(polars: list[PolarCoord]) -> float | None:
    """Retourne l'angle de la distance minimale parmi les mesures valides."""
    valid_ranges = filter_valid_polars(polars)

    minimum = min(valid_ranges, key=lambda item: item.distance, default=None)
    if minimum is None:
        return None
    return minimum.angle


def minimum_distance(polars: list[PolarCoord]) -> float | None:
    """Retourne la distance minimale parmi les mesures laser valides."""
    valid_ranges = filter_valid_polars(polars)
    minimum = min(valid_ranges, key=lambda item: item.distance, default=None)
    return minimum.distance if minimum is not None else None


def deform_distance(polars: list[PolarCoord], facteur_min: float, facteur_max: float, champ_vision: float) -> list[PolarCoord]:
    """Déforme les distances pour faire apparaitre les objets en face plus proches qu'ils ne le sont réellement."""
    for polar in polars:
        if abs(polar.angle) <= champ_vision / 2:
            # Le facteur varie tel que facteur_min est appliqué à l'angle 0
            # et facteur_max est appliqué aux extrémités du champ de vision.
            facteur = facteur_min + (facteur_max - facteur_min) * (abs(polar.angle) / (champ_vision / 2))
            polar.distance *= facteur
    return polars



class CalculateurCerveauOrientation:
    """Calcule une correction proportionnelle de l'orientation du robot."""

    def __init__(self, gain_proportionnel=0.5):
        """Initialise le calculateur avec son gain proportionnel."""
        self.erreur_angle = 0.0
        self.gain_proportionnel = gain_proportionnel
        self.angle_vue = math.radians(60.0)  # Angle de vue par défaut en radians

    def calculer_correction(self):
        """Retourne la correction basée sur la dernière erreur calculée."""
        return self.gain_proportionnel * self.erreur_angle

    def mettre_a_jour_erreur(self, ranges, angle_min, angle_increment):
        """Met à jour l'erreur avec l'angle de la mesure laser minimale."""
        polars = to_polarcoord(ranges, angle_min, angle_increment)

        # print([f"{polar.distance}, {polar.angle}" for polar in polars])
        
        # Filtrage des polars pour ne garder que ceux dans le champ de vision
        polars = [polar for polar in polars if abs(polar.angle) <= self.angle_vue / 2]

        angle = minimum_distance_angle(deform_distance(polars, facteur_min=0.5, facteur_max=1.0, champ_vision=self.angle_vue))
        if angle is None:
            self.erreur_angle = 0.0
            return None

        self.erreur_angle = angle # angle 0 signifie que le robot est bien orienté vers l'objet le plus proche
        return angle


class CalculateurCerveauDistance:
    """Classe pour calculer la distance minimale à partir des données du laser."""

    def __init__(self, gain_proportionnel=0.5, distance_cible=0.5):
        """Initialise le calculateur avec une erreur initiale de 0."""
        self.erreur_distance = 0.0
        self.gain_proportionnel = gain_proportionnel
        self.distance_cible = distance_cible
        self.angle_vue = math.radians(60.0)  # Angle de vue par défaut en radians

    def calculer_correction(self):
        """Retourne la correction basée sur la dernière erreur calculée."""
        return self.gain_proportionnel * self.erreur_distance

    def mettre_a_jour_erreur(self, ranges, angle_min, angle_increment):
        """Met à jour l'erreur avec la distance minimale mesurée par le laser."""
        polars = to_polarcoord(ranges, angle_min, angle_increment)

        polars = [polar for polar in polars if abs(polar.angle) <= self.angle_vue / 2]

        distance = minimum_distance(polars)
        if distance is None:
            self.erreur_distance = 0.0
            return None

        # Le robot doit se rapprocher de l'objet le plus proche, donc l'erreur est la différence entre la distance mesurée et la distance cible.
        # Car le robot avance vers l'objet si l'erreur est positive et recule si l'erreur est négative.
        self.erreur_distance = distance - self.distance_cible
        return distance
