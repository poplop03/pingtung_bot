from pingtung_vision_interfaces.msg import (
    AnimalResult,
    FruitLineHit,
    PigShitSpatial,
)


def test_interface_constants_and_defaults():
    animal = AnimalResult()
    assert AnimalResult.UNKNOWN == 0
    assert AnimalResult.DOG == 1
    assert AnimalResult.MONKEY == 2
    assert AnimalResult.RABBIT == 3
    assert AnimalResult.TURTLE == 4
    assert animal.valid is False

    fruit = FruitLineHit()
    assert FruitLineHit.NO_HIT == 0
    assert FruitLineHit.HIT == 1
    assert fruit.valid is False

    spatial = PigShitSpatial()
    assert spatial.valid is False
    assert spatial.left_pig == 0.0
