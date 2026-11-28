"""Every requirement in docs/requirements.md is tested, and every rule a test
cites is a requirement there."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import reqtrace as rq  # noqa: E402


def test_spec_has_requirements():
    assert len(rq.requirements()) >= 10


def test_every_requirement_has_a_test():
    reqs, cited = rq.requirements(), rq.citations()
    untested = [r for r in reqs if not cited.get(r)]
    assert not untested, f"requirements with no test citing them: {untested}"


def test_every_cited_rule_is_a_requirement():
    reqs, cited = rq.requirements(), rq.citations()
    unknown = {r: cited[r] for r in cited if r not in reqs}
    assert not unknown, f"tests cite rules not in docs/requirements.md: {unknown}"
