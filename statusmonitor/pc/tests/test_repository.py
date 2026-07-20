from __future__ import annotations

from statusmonitor.models import SessionMetadata
from statusmonitor.repository import Repository
from statusmonitor.settings import Settings


def test_session_template_round_trip(isolated_home):
    repo = Repository(Settings.load())
    session = repo.create_session(
        SessionMetadata(
            team_number="74000M",
            operator="Alex",
            test_type="manual",
            surface="foam",
        )
    )
    repo.save_template("foam-manual", session)
    template = repo.list_templates()["foam-manual"]
    assert template.team_number == "74000M"
    assert template.surface == "foam"
    assert template.session_id == "TEMPLATE"
