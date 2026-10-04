
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
js=(ROOT/"static/app.js").read_text(encoding="utf8")
html=(ROOT/"templates/index.html").read_text(encoding="utf8")
def test_double_click_dimension_editor_present():
    assert "ondblclick" in js and "/api/edit_entity/" in js
def test_dimension_types_present():
    for s in ["length_mm","angle_deg","radius_mm","diameter_mm"]: assert s in js
def test_editor_hint_present():
    assert "双击尺寸标注" in html
