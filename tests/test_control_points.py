
import sys,math
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from geometry_fit import move_fitted_control

def base():
    return {"outer":[
      {"type":"LINE","start":[0.,0.],"end":[100.,0.]},
      {"type":"LINE","start":[100.,0.],"end":[100.,80.]},
      {"type":"LINE","start":[100.,80.],"end":[0.,80.]},
      {"type":"LINE","start":[0.,80.],"end":[0.,0.]},
    ],"holes":[],"hole_kinds":[],"counts":{"LINE":4,"ARC":0,"CIRCLE":0,"SLOT":0}}

def test_line_junction_drag_keeps_closed_topology():
    f=move_fitted_control(base(),"outer",0,0,"end",[110,5])
    assert np.allclose(f["outer"][0]["end"],[110,5])
    assert np.allclose(f["outer"][1]["start"],[110,5])

def test_circle_center_and_radius_handles():
    f={"outer":[],"holes":[[{"type":"CIRCLE","center":[10.,20.],"radius":8.}]],"hole_kinds":["CIRCLE"],"counts":{"LINE":0,"ARC":0,"CIRCLE":1,"SLOT":0}}
    q=move_fitted_control(f,"holes",0,0,"center",[30,40])
    assert np.allclose(q["holes"][0][0]["center"],[30,40])
    q=move_fitted_control(q,"holes",0,0,"radius",[50,40])
    assert abs(q["holes"][0][0]["radius"]-20)<1e-6

def test_arc_radius_handle_preserves_arc_type():
    arc={"type":"ARC","center":[0.,0.],"radius":20.,"start":[20.,0.],"end":[0.,20.],
         "start_angle":0.,"end_angle":90.,"ccw":True,"span_deg":90.}
    f={"outer":[arc,{"type":"LINE","start":[0.,20.],"end":[-30.,20.]},
                {"type":"LINE","start":[-30.,20.],"end":[20.,0.]}],
       "holes":[],"hole_kinds":[],"counts":{"LINE":2,"ARC":1,"CIRCLE":0,"SLOT":0}}
    q=move_fitted_control(f,"outer",0,0,"radius",[35,0])
    assert q["outer"][0]["type"]=="ARC"
    assert abs(q["outer"][0]["radius"]-35)<1e-6
    assert np.allclose(q["outer"][1]["start"],q["outer"][0]["end"])

def test_arc_center_drag_translates_arc_and_reconnects_neighbors():
    arc={"type":"ARC","center":[0.,0.],"radius":20.,"start":[20.,0.],"end":[0.,20.],
         "start_angle":0.,"end_angle":90.,"ccw":True,"span_deg":90.}
    f={"outer":[{"type":"LINE","start":[-20.,0.],"end":[20.,0.]},arc,
                {"type":"LINE","start":[0.,20.],"end":[-20.,20.]}],
       "holes":[],"hole_kinds":[],"counts":{"LINE":2,"ARC":1,"CIRCLE":0,"SLOT":0}}
    q=move_fitted_control(f,"outer",0,1,"center",[5,5])
    assert np.allclose(q["outer"][1]["center"],[5,5])
    assert np.allclose(q["outer"][0]["end"],q["outer"][1]["start"])
    assert np.allclose(q["outer"][2]["start"],q["outer"][1]["end"])
