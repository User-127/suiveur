import pytest

from suiveur.metier import CalculateurCerveauDistance


@pytest.mark.parametrize(
    ('distance', 'expected_correction'),
    [
        (0.25, -0.125),
        (0.5, 0.0),
        (0.75, 0.125),
    ],
)
def test_distance_correction_moves_towards_target(distance, expected_correction):
    calculateur = CalculateurCerveauDistance(distance_cible=0.5)

    calculateur.mettre_a_jour_erreur([distance])

    assert calculateur.calculer_correction() == pytest.approx(expected_correction)
