import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent.absolute()))

import swervelib.mod as mod


# TODO: Write more tests
def test_place_in_scope():
    assert mod.place_in_proper_0_to_360_scope(500, 20) == 380
