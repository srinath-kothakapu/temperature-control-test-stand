"""Records the requirement IDs of each test in the JUnit XML, so tools/trace.py can build the
traceability matrix. Mark a test with @pytest.mark.req("REQ-15")."""

import pytest


def pytest_configure(config):
    config.addinivalue_line("markers", "req(*ids): requirement IDs (tests/requirements.csv) verified by this test")


@pytest.fixture(autouse=True)
def _record_requirements(request, record_property):
    marker = request.node.get_closest_marker("req")
    if marker:
        record_property("req", ",".join(marker.args))
