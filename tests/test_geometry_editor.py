
import math,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from geometry_fit import fit_geometry,edit_entity,entity_measurements

def rounded_rect(w=400,h=250,r=40,narc=20):
    pts=[]
    # clockwise in image-style coordinates
    pts += [[r,0],[w-r,0]]
    for a in np.linspace(-90,0,narc)[1:]: pts.append([w-r+r*math.cos(math.radians(a)),r+r*math.sin(math.radians(a))])
    pts += [[w,h-r]]
    for a in np.linspace(0,90,narc)[1:]: pts.append([w-r+r*math.cos(math.radians(a)),h-r+r*math.sin(math.radians(a))])
    pts += [[r,h]]
    for a in np.linspace(90,180,narc)[1:]: pts.append([r+r*math.cos(math.radians(a)),h-r+r*math.sin(math.radians(a))])
    pts += [[0,r]]
    for a in np.linspace(180,270,narc)[1:]: pts.append([r+r*math.cos(math.radians(a)),r+r*math.sin(math.radians(a))])
    return np.asarray(pts,float)

def test_rounded_rectangle_has_long_lines_and_arcs():
    f=fit_geometry(rounded_rect(),[],6,False)
    assert f["counts"]["LINE"]<=8
    assert f["counts"]["ARC"]>=2
    lengths=[entity_measurements(e).get("length_mm",0) for e in f["outer"] if e["type"]=="LINE"]
    assert max(lengths)>=240

def test_line_length_edit():
    e={"type":"LINE","start":[0.,0.],"end":[100.,0.]}
    q=edit_entity(e,"length_mm",160)
    assert abs(entity_measurements(q)["length_mm"]-160)<1e-6
    assert np.allclose((np.asarray(q["start"])+np.asarray(q["end"]))/2,[50,0])

def test_line_angle_edit():
    e={"type":"LINE","start":[0.,0.],"end":[100.,0.]}
    q=edit_entity(e,"angle_deg",30)
    assert abs(entity_measurements(q)["angle_deg"]-30)<1e-6
    assert abs(entity_measurements(q)["length_mm"]-100)<1e-6

def test_arc_radius_edit():
    e={"type":"ARC","center":[0.,0.],"radius":20.,"start":[20.,0.],"end":[0.,20.],"span_deg":90.,"ccw":True}
    q=edit_entity(e,"radius_mm",35)
    assert abs(q["radius"]-35)<1e-6
    assert abs(np.linalg.norm(q["start"])-35)<1e-6

def test_circle_diameter_edit():
    e={"type":"CIRCLE","center":[5.,5.],"radius":10.}
    q=edit_entity(e,"diameter_mm",50)
    assert q["radius"]==25

def test_no_fragmentation_on_noisy_long_rectangle():
    rng=np.random.default_rng(2)
    pts=[]
    for x in np.linspace(0,500,120): pts.append([x,rng.normal(0,1.8)])
    for y in np.linspace(0,250,60): pts.append([500+rng.normal(0,1.2),y])
    for x in np.linspace(500,0,120): pts.append([x,250+rng.normal(0,1.8)])
    for y in np.linspace(250,0,60): pts.append([rng.normal(0,1.2),y])
    f=fit_geometry(np.asarray(pts),[],10,False)
    assert f["counts"]["LINE"]<=8

def test_large_rounded_corner_must_be_arc_not_chord():
    # Closed rectangle with four true R60 corners.
    f=fit_geometry(rounded_rect(500,320,60,45),[],8,False)
    arcs=[e for e in f["outer"] if e["type"]=="ARC"]
    assert len(arcs)>=3, f
    assert sum(45 <= e["radius"] <= 80 and e["span_deg"]>=45 for e in arcs)>=3, f

def test_semicircular_notch_must_survive_as_arc():
    # U-like contour with an R50 semicircular recess.
    pts=[[0,0],[300,0],[300,250],[0,250],[0,180],[120,180]]
    c=np.array([120.,130.])
    for a in np.linspace(90,-90,70)[1:]:
        pts.append((c+50*np.array([math.cos(math.radians(a)),math.sin(math.radians(a))])).tolist())
    pts += [[0,80],[0,0]]
    f=fit_geometry(np.asarray(pts,float),[],8,False)
    arcs=[e for e in f["outer"] if e["type"]=="ARC"]
    assert any(e["span_deg"]>=100 for e in arcs), f

def test_small_burr_on_long_edge_is_not_a_design_feature():
    # A long manufactured edge with a 4 mm local textile/piping bump.
    pts=[]
    for x in np.linspace(0,500,160):
        y=4*np.exp(-((x-250)/9)**2)
        pts.append([x,y])
    pts += [[500,y] for y in np.linspace(0,250,60)]
    pts += [[x,250] for x in np.linspace(500,0,160)]
    pts += [[0,y] for y in np.linspace(250,0,60)]
    f=fit_geometry(np.asarray(pts,float),[],10,False)
    long_lines=[e for e in f["outer"] if e["type"]=="LINE"
                and np.linalg.norm(np.asarray(e["end"])-np.asarray(e["start"]))>350]
    assert long_lines, f
