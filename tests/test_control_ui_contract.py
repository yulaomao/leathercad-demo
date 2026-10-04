
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
js=(ROOT/"static/app.js").read_text(encoding="utf8")
html=(ROOT/"templates/index.html").read_text(encoding="utf8")
def test_control_mode_and_reference_controls_exist():
    for x in ['id="controlMode"','id="refToggle"','id="refOpacity"','id="ghostToggle"']:
        assert x in html
def test_drag_control_api_used():
    assert "/api/drag_control/" in js
    assert "ctrlHandle" in js
def test_aligned_reference_uses_project_meta():
    assert "rectified_size" in js and "S.meta.origin" in js
