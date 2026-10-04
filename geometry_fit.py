import math
import numpy as np

def _line_fit(p):
    p=np.asarray(p,float); c=p.mean(0)
    _,_,vh=np.linalg.svd(p-c,full_matrices=False); d=vh[0]; n=np.array([-d[1],d[0]])
    e=np.abs((p-c)@n)
    return float(np.sqrt(np.mean(e*e))),float(e.max())

def _circle_fit(p):
    p=np.asarray(p,float);x=p[:,0];y=p[:,1]
    A=np.c_[2*x,2*y,np.ones(len(p))];b=x*x+y*y
    try:q=np.linalg.lstsq(A,b,rcond=None)[0]
    except:return None
    c=np.array(q[:2]);r2=q[2]+c@c
    if r2<=0:return None
    r=math.sqrt(r2);e=np.abs(np.hypot(x-c[0],y-c[1])-r)
    return c,r,float(np.sqrt(np.mean(e*e))),float(e.max())

def _line(a,b):return {"type":"LINE","start":list(map(float,a)),"end":list(map(float,b))}
def _arc(seg,c,r):
    a=np.unwrap(np.arctan2(seg[:,1]-c[1],seg[:,0]-c[0])); span=math.degrees(a[-1]-a[0])
    return {"type":"ARC","center":c.tolist(),"radius":float(r),"start":seg[0].tolist(),"end":seg[-1].tolist(),
            "start_angle":float(math.degrees(a[0])),"end_angle":float(math.degrees(a[-1])),
            "ccw":bool(span>=0),"span_deg":float(abs(span))}

def _rdp(points,eps):
    p=np.asarray(points,np.float32).reshape(-1,1,2)
    return cv_approx(p,eps)

def cv_approx(p,eps):
    # local import keeps this module lightweight
    import cv2
    return cv2.approxPolyDP(p,float(eps),True)[:,0,:].astype(float)

def _turn_angle(a,b,c):
    u=a-b;v=c-b
    nu=np.linalg.norm(u);nv=np.linalg.norm(v)
    if nu<1e-8 or nv<1e-8:return 180.
    return math.degrees(math.acos(float(np.clip(np.dot(u,v)/(nu*nv),-1,1))))

def _corner_indices(p,corner_deg=145):
    n=len(p); out=[]
    for i in range(n):
        if _turn_angle(p[(i-1)%n],p[i],p[(i+1)%n])<corner_deg: out.append(i)
    return out

def _valid_line(seg,tol):
    if len(seg)<2:return False
    rms,mx=_line_fit(seg)
    return mx<=tol and rms<=tol*.55

def _valid_arc(seg,tol):
    if len(seg)<5:return None
    f=_circle_fit(seg)
    if not f:return None
    c,r,rms,mx=f
    if r<4 or r>20000 or mx>tol or rms>tol*.60:return None
    ang=np.unwrap(np.arctan2(seg[:,1]-c[1],seg[:,0]-c[0]))
    span=abs(math.degrees(ang[-1]-ang[0]))
    chord=np.linalg.norm(seg[-1]-seg[0])
    if span<10 or span>260 or chord<tol*3:return None
    # reject nearly-straight huge-radius arcs
    sag=r-math.sqrt(max(0,r*r-(chord*.5)**2)) if chord<2*r else r
    if sag<tol*.7:return None
    return c,r

def _fit_open_run(seg,tol):
    """Fit one run with as few primitives as possible. No artificial 28-point cap."""
    n=len(seg); ents=[];i=0
    while i<n-1:
        best_line=i+1; best_arc=None; fail_line=fail_arc=0
        # progressively extend. Once both models have failed repeatedly after a useful run,
        # stop; this prevents a later unrelated edge being swallowed.
        for j in range(i+1,n):
            s=seg[i:j+1]
            vl=_valid_line(s,tol)
            va=_valid_arc(s,tol)
            if vl: best_line=j;fail_line=0
            else: fail_line+=1
            if va is not None: best_arc=(j,va);fail_arc=0
            else: fail_arc+=1
            if j-i>8 and fail_line>=4 and fail_arc>=4: break
        line_span=best_line-i
        arc_span=(best_arc[0]-i) if best_arc else -1
        # prefer a long line unless arc explains substantially more contour samples.
        if best_arc and arc_span>=max(5,line_span+2):
            j,(c,r)=best_arc;ents.append(_arc(seg[i:j+1],c,r));i=j
        else:
            j=max(i+1,best_line);ents.append(_line(seg[i],seg[j]));i=j
    return ents

def _merge_lines(ents,angle_deg=3.0,dist_tol=2.5):
    if not ents:return []
    changed=True
    while changed:
        changed=False;out=[];i=0
        while i<len(ents):
            if i+1<len(ents) and ents[i]["type"]=="LINE" and ents[i+1]["type"]=="LINE":
                a=np.array(ents[i]["start"]);b=np.array(ents[i]["end"]);c=np.array(ents[i+1]["end"])
                u=b-a;v=c-b
                if np.linalg.norm(u)>0 and np.linalg.norm(v)>0:
                    ang=math.degrees(math.acos(float(np.clip(np.dot(u,v)/(np.linalg.norm(u)*np.linalg.norm(v)),-1,1))))
                    rms,mx=_line_fit(np.array([a,b,c]))
                    if ang<=angle_deg and mx<=dist_tol:
                        out.append(_line(a,c));i+=2;changed=True;continue
            out.append(ents[i]);i+=1
        ents=out
    return ents

def _arc_candidate(seg,tol):
    f=_circle_fit(seg)
    if not f:return None
    c,r,rms,mx=f
    if not (5<r<350):return None
    v=seg-c
    a=np.unwrap(np.arctan2(v[:,1],v[:,0]))
    da=np.diff(a)
    # A real contour arc must progress monotonically around the fitted centre.
    nz=da[np.abs(da)>1e-5]
    if len(nz)<2:return None
    same=max(np.mean(nz>0),np.mean(nz<0))
    if same<.88:return None
    span=abs(float(a[-1]-a[0]))
    span_deg=math.degrees(span)
    if not (12<=span_deg<=135):return None
    chord=float(np.linalg.norm(seg[-1]-seg[0]))
    if chord<tol*2.5 or mx>tol*1.05 or rms>tol*.58:return None
    # Compare actual contour length with the fitted-circle arc length.
    plen=float(np.sum(np.linalg.norm(np.diff(seg,axis=0),axis=1)))
    alen=float(r*span)
    if alen<=1e-6 or not (.72<=plen/alen<=1.35):return None
    # Reject almost-straight arcs; those should become one LINE.
    sag=r-math.sqrt(max(0,r*r-(chord*.5)**2)) if chord<2*r else r
    if sag<max(2.5,tol*.55):return None
    return c,r,rms,mx,span_deg

def fit_closed_polyline(poly,tol=8.0):
    """Global design-geometry fit.
    Optimizes for few primitives, prefers long straight edges, and only accepts
    arcs whose point order, arc length and residual are geometrically consistent."""
    p=np.asarray(poly,float)
    if len(p)<3:return []
    p=_rdp(p,max(3.0,tol*.48))
    n=len(p)
    if n<3:return []
    turns=[_turn_angle(p[(i-1)%n],p[i],p[(i+1)%n]) for i in range(n)]
    st=int(np.argmin(turns))
    q=np.vstack([p[st:],p[:st],p[st]])
    N=len(q); INF=1e18
    dp=[INF]*N; prev=[None]*N; dp[0]=0.0
    ENTITY=1000.0
    for j in range(1,N):
        for i in range(j):
            if dp[i]>=INF: continue
            seg=q[i:j+1]
            chord=float(np.linalg.norm(seg[-1]-seg[0]))
            if chord<1e-6: continue

            lr,lm=_line_fit(seg)
            if lm<=tol*1.20 and lr<=tol*.68:
                # Prefer a single long line whenever it explains the points well.
                score=dp[i]+ENTITY+(lr/max(tol,1e-6))*7.0
                if score<dp[j]:
                    dp[j]=score;prev[j]=(i,"LINE",None)

            if j-i>=4:
                ac=_arc_candidate(seg,tol)
                if ac:
                    c,r,rr,rm,span_deg=ac
                    # Slight arc penalty prevents noisy long edges turning into curves.
                    score=dp[i]+ENTITY+120.0+(rr/max(tol,1e-6))*9.0
                    if score<dp[j]:
                        dp[j]=score;prev[j]=(i,"ARC",(c,r))

    if prev[-1] is None:
        return [_line(q[i],q[i+1]) for i in range(N-1)]

    chunks=[];j=N-1
    while j>0:
        i,typ,param=prev[j];chunks.append((i,j,typ,param));j=i
    chunks.reverse()
    ents=[]
    for i,j,typ,param in chunks:
        if typ=="LINE": ents.append(_line(q[i],q[j]))
        else: ents.append(_arc(q[i:j+1],param[0],param[1]))
    ents=_merge_lines(ents,6.0,max(4.0,tol*.85))
    return ents

def _area(p):
    p=np.asarray(p,float);q=np.roll(p,-1,axis=0)
    return abs(.5*np.sum(p[:,0]*q[:,1]-q[:,0]*p[:,1]))

def classify_hole(poly,tol=3.0):
    p=np.asarray(poly,float);f=_circle_fit(p)
    if f:
        c,r,rms,mx=f
        if r>5.0 and mx<=tol*.85 and rms<=tol*.50:
            return [{"type":"CIRCLE","center":c.tolist(),"radius":float(r)}],"CIRCLE"
    c=p.mean(0);_,_,vh=np.linalg.svd(p-c,full_matrices=False);q=(p-c)@vh.T
    L=float(np.ptp(q[:,0]));W=float(np.ptp(q[:,1]))
    if W>L:L,W=W,L;vh=vh[::-1]
    area=_area(p);ideal=max(1,(L-W)*W+math.pi*(W/2)**2)
    if W>8 and 1.35<=L/W<=10 and abs(area-ideal)/max(area,1)<.16:
        axis=vh[0];normal=vh[1];r=W/2;half=max(0,(L-W)/2);c1=c-axis*half;c2=c+axis*half
        p1=c1+normal*r;p2=c2+normal*r;p3=c2-normal*r;p4=c1-normal*r
        return [_line(p1,p2),_arc(np.array([p2,c2+axis*r,p3]),c2,r),_line(p3,p4),_arc(np.array([p4,c1-axis*r,p1]),c1,r)],"SLOT"
    return fit_closed_polyline(p,tol),"MIXED"

def fit_geometry(outer,holes,tol=3.0):
    oe=fit_closed_polyline(outer,tol);he=[];kinds=[]
    for h in holes:
        e,k=classify_hole(h,tol);he.append(e);kinds.append(k)
    counts={"LINE":0,"ARC":0,"CIRCLE":0,"SLOT":sum(k=="SLOT" for k in kinds)}
    for e in oe:
        if e["type"] in counts:counts[e["type"]]+=1
    for g in he:
        for e in g:
            if e["type"] in counts:counts[e["type"]]+=1
    return {"outer":oe,"holes":he,"hole_kinds":kinds,"counts":counts,"tolerance_mm":tol}


def _angle_deg(a,b):
    v=np.asarray(b,float)-np.asarray(a,float)
    if np.linalg.norm(v)<1e-9:return 0.0
    return math.degrees(math.atan2(v[1],v[0]))

def _norm180(a):
    a=a%180.0
    return a

def _angdiff180(a,b):
    d=abs(_norm180(a)-_norm180(b))
    return min(d,180-d)

def _line_len(e):
    a=np.asarray(e["start"],float);b=np.asarray(e["end"],float)
    return float(np.linalg.norm(b-a))

def _intersection(p1,p2,p3,p4):
    x1,y1=p1;x2,y2=p2;x3,y3=p3;x4,y4=p4
    den=(x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)
    if abs(den)<1e-9:return None
    px=((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4))/den
    py=((x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4))/den
    return np.array([px,py],float)

def regularize_design_geometry(fitted, angle_snap_deg=4.0, orthogonal_snap_deg=4.0,
                               parallel_snap_deg=4.0, radius_snap_mm=2.0,
                               standard_radii=None):
    """
    Conservative design-geometry regularization.

    Goals:
    1. Collapse nearly identical line directions into shared dominant directions.
    2. Snap near-horizontal/vertical lines to exact 0/90 degrees.
    3. Preserve line endpoints as much as possible by rotating about segment midpoints.
    4. Snap similar ARC/CIRCLE radii to shared/standard radii only when already close.
    5. Reconnect adjacent LINE-LINE entities by their intersection when movement is small.

    It intentionally avoids aggressive geometry invention.
    """
    if standard_radii is None:
        standard_radii=[5,8,10,12,15,20,25,30,35,40,50,60,75,80,100]

    import copy
    out=copy.deepcopy(fitted)
    ents=out.get("outer",[])
    line_idx=[i for i,e in enumerate(ents) if e.get("type")=="LINE" and _line_len(e)>5]

    # --- dominant line direction clustering (orientation modulo 180°)
    angles=[]
    weights=[]
    for i in line_idx:
        e=ents[i]
        angles.append(_norm180(_angle_deg(e["start"],e["end"])))
        weights.append(max(1.0,_line_len(e)))

    clusters=[]
    for idx,a in enumerate(angles):
        placed=False
        for c in clusters:
            if _angdiff180(a,c["angle"])<=parallel_snap_deg:
                c["members"].append(idx)
                # weighted circular-ish mean for orientation modulo 180 using doubled angle
                vals=[angles[k] for k in c["members"]]
                ws=[weights[k] for k in c["members"]]
                sx=sum(w*math.cos(math.radians(2*v)) for w,v in zip(ws,vals))
                sy=sum(w*math.sin(math.radians(2*v)) for w,v in zip(ws,vals))
                c["angle"]=(math.degrees(math.atan2(sy,sx))/2)%180
                placed=True
                break
        if not placed:
            clusters.append({"angle":a,"members":[idx]})

    # Snap cluster directions to 0/90 if already close, and pair near-orthogonal clusters
    for c in clusters:
        if min(_angdiff180(c["angle"],0),_angdiff180(c["angle"],90))<=angle_snap_deg:
            c["angle"]=0.0 if _angdiff180(c["angle"],0)<=_angdiff180(c["angle"],90) else 90.0

    # near-orthogonal relationships: choose heavier cluster as anchor
    cluster_weight=[]
    for c in clusters:
        cluster_weight.append(sum(weights[k] for k in c["members"]))
    for i in range(len(clusters)):
        for j in range(i+1,len(clusters)):
            ai,aj=clusters[i]["angle"],clusters[j]["angle"]
            d=_angdiff180(ai,aj)
            if abs(d-90)<=orthogonal_snap_deg:
                if cluster_weight[i]>=cluster_weight[j]:
                    clusters[j]["angle"]=(ai+90)%180
                else:
                    clusters[i]["angle"]=(aj+90)%180

    # Apply direction snapping by rotating each line around its midpoint, preserving length
    line_to_cluster={}
    for ci,c in enumerate(clusters):
        for local_idx in c["members"]:
            line_to_cluster[line_idx[local_idx]]=ci

    for i in line_idx:
        e=ents[i]
        a=np.asarray(e["start"],float);b=np.asarray(e["end"],float)
        mid=(a+b)/2;L=np.linalg.norm(b-a)
        ang=math.radians(clusters[line_to_cluster[i]]["angle"])
        d=np.array([math.cos(ang),math.sin(ang)])
        e["start"]=(mid-d*L/2).tolist()
        e["end"]=(mid+d*L/2).tolist()
        e["regularized"]=True

    # Reconnect consecutive LINE-LINE if intersection is nearby.
    n=len(ents)
    for i in range(n):
        j=(i+1)%n
        e1,e2=ents[i],ents[j]
        if e1.get("type")!="LINE" or e2.get("type")!="LINE":
            continue
        p1=np.asarray(e1["start"],float);p2=np.asarray(e1["end"],float)
        p3=np.asarray(e2["start"],float);p4=np.asarray(e2["end"],float)
        q=_intersection(p1,p2,p3,p4)
        if q is None: continue
        if np.linalg.norm(q-p2)<=8.0 and np.linalg.norm(q-p3)<=8.0:
            e1["end"]=q.tolist();e2["start"]=q.tolist()

    # Radius snapping on outer arcs and hole circles/arcs
    all_groups=[out.get("outer",[])] + out.get("holes",[])
    radii=[]
    refs=[]
    for g in all_groups:
        for e in g:
            if e.get("type") in ("ARC","CIRCLE") and e.get("radius",0)>0:
                radii.append(float(e["radius"]));refs.append(e)

    # First snap to common measured-radius clusters
    used=[False]*len(radii)
    for i,r in enumerate(radii):
        if used[i]: continue
        grp=[i]
        for j in range(i+1,len(radii)):
            if not used[j] and abs(radii[j]-r)<=radius_snap_mm:
                grp.append(j)
        if len(grp)>=2:
            avg=sum(radii[k] for k in grp)/len(grp)
            for k in grp:
                refs[k]["radius"]=avg
                refs[k]["radius_grouped"]=True
                used[k]=True

    # Then snap to standard radii only if extremely close
    for e in refs:
        r=float(e["radius"])
        nearest=min(standard_radii,key=lambda x:abs(x-r))
        if abs(nearest-r)<=min(radius_snap_mm, max(.8,0.025*r)):
            e["radius_raw"]=r
            e["radius"]=float(nearest)
            e["radius_standardized"]=True

    # Restore exact topological continuity after line-direction snapping.
    # LINE endpoints may move during regularization; ARC endpoints are treated as
    # the geometric anchors so we never leave visible CAD gaps.
    n=len(ents)
    for i in range(n):
        j=(i+1)%n
        a,b=ents[i],ents[j]
        if a.get("type")=="LINE" and b.get("type")=="ARC":
            a["end"]=list(b["start"])
        elif a.get("type")=="ARC" and b.get("type")=="LINE":
            b["start"]=list(a["end"])
        elif a.get("type")=="LINE" and b.get("type")=="LINE":
            # If intersection was not accepted above, use midpoint only for a small residual gap.
            p=np.asarray(a["end"],float);q=np.asarray(b["start"],float)
            if np.linalg.norm(p-q)<=12.0:
                z=((p+q)/2).tolist();a["end"]=z;b["start"]=z

    # Refresh counts
    counts={"LINE":0,"ARC":0,"CIRCLE":0,"SLOT":out.get("counts",{}).get("SLOT",0)}
    for e in out.get("outer",[]):
        if e.get("type") in counts: counts[e["type"]]+=1
    for g in out.get("holes",[]):
        for e in g:
            if e.get("type") in counts: counts[e["type"]]+=1
    out["counts"]=counts
    out["regularized"]=True
    out["regularization"]={
        "angle_snap_deg":angle_snap_deg,
        "parallel_snap_deg":parallel_snap_deg,
        "orthogonal_snap_deg":orthogonal_snap_deg,
        "radius_snap_mm":radius_snap_mm,
    }
    return out

def fit_geometry(outer,holes,tol=8.0,regularize=True):
    oe=fit_closed_polyline(outer,tol);he=[];kinds=[]
    for h in holes:
        e,k=classify_hole(h,tol);he.append(e);kinds.append(k)
    counts={"LINE":0,"ARC":0,"CIRCLE":0,"SLOT":sum(k=="SLOT" for k in kinds)}
    for e in oe:
        if e["type"] in counts:counts[e["type"]]+=1
    for g in he:
        for e in g:
            if e["type"] in counts:counts[e["type"]]+=1
    result={"outer":oe,"holes":he,"hole_kinds":kinds,"counts":counts,"tolerance_mm":tol}
    if regularize:
        result=regularize_design_geometry(result)
    return result


# ========================= V0.16 design-geometry fitter =========================
def _resample_closed(poly, step=3.0):
    p=np.asarray(poly,float)
    if len(p)<3:return p
    q=np.vstack([p,p[0]])
    d=np.linalg.norm(np.diff(q,axis=0),axis=1)
    cum=np.r_[0,np.cumsum(d)]
    total=cum[-1]
    if total<step*3:return p
    ss=np.arange(0,total,step)
    out=[]
    for s in ss:
        i=min(len(d)-1,max(0,np.searchsorted(cum,s,side="right")-1))
        t=(s-cum[i])/max(d[i],1e-9)
        out.append(q[i]*(1-t)+q[i+1]*t)
    return np.asarray(out)

def _smooth_closed(p, win=5):
    p=np.asarray(p,float);n=len(p)
    if n<win:return p
    h=win//2
    ext=np.vstack([p[-h:],p,p[:h]])
    return np.asarray([ext[i:i+win].mean(0) for i in range(n)])

def _design_corners(poly, tol):
    """Detect SHARP design vertices only.
    Rounded corners create sustained curvature (a plateau), while a true polygon
    corner creates a localized curvature peak. Treating every high-curvature point
    as a corner was the reason V0.16.1 converted many radii into chords."""
    sample_step=max(2.0,min(4.0,tol*.30))
    p=_smooth_closed(_resample_closed(poly,sample_step),5);n=len(p)
    if n<8:return p,[]
    baseline=max(20.0,tol*2.4)
    step=max(2,int(round(baseline/sample_step)))
    score=np.zeros(n)
    for i in range(n):
        a=p[(i-step)%n]-p[i];b=p[(i+step)%n]-p[i]
        na=np.linalg.norm(a);nb=np.linalg.norm(b)
        if na<1e-6 or nb<1e-6:continue
        interior=math.degrees(math.acos(float(np.clip(np.dot(a,b)/(na*nb),-1,1))))
        score[i]=180.0-interior

    # Peak prominence discriminates a true vertex from a smooth circular arc.
    shoulder=max(2,int(round(max(16.0,tol*1.8)/sample_step)))
    cand=[]
    for i in range(n):
        if score[i]<38.0:continue
        local=[score[(i+j)%n] for j in range(-2,3)]
        if score[i]+1e-6<max(local):continue
        shoulders=[score[(i-shoulder)%n],score[(i+shoulder)%n]]
        prominence=score[i]-sum(shoulders)/2
        # Strong 80–100° vertices pass; a radius plateau normally does not.
        if prominence>=12.0 or score[i]>=78.0:
            cand.append(i)

    # Non-max suppression: one design vertex, one index.
    radius=max(2,int(round(max(18.0,tol*2.0)/sample_step)))
    chosen=[]
    for i in sorted(cand,key=lambda k:score[k],reverse=True):
        if all(min((i-j)%n,(j-i)%n)>radius for j in chosen):
            chosen.append(i)
    chosen.sort()
    return p,chosen

def _cyclic_run(p,a,b):
    if a<=b:return p[a:b+1]
    return np.vstack([p[a:],p[:b+1]])

def _fit_design_run(seg,tol):
    """Fit corner-to-corner contour with curvature preservation.
    Priority: whole LINE -> whole ARC -> find longest valid ARC sub-run -> recursive split."""
    seg=np.asarray(seg,float)
    if len(seg)<2:return []
    lr,lm=_line_fit(seg)
    if lm<=tol*.48 and lr<=tol*.24:
        return [_line(seg[0],seg[-1])]
    ac=_arc_candidate(seg,max(2.0,tol*.75))
    if ac:
        c,r,rr,rm,span=ac
        return [_arc(seg,c,r)]

    # Search longest substantial circular sub-run. This is what preserves a rounded
    # corner between two straight tangent portions.
    n=len(seg);best=None
    minpts=max(7,int(n*.12))
    for i in range(0,n-minpts):
        # coarse-to-fine longest-first end search
        for j in range(n-1,i+minpts-1,-1):
            if best and (j-i)<=best[0]:break
            sub=seg[i:j+1]
            ac=_arc_candidate(sub,max(2.0,tol*.72))
            if ac:
                c,r,rr,rm,span=ac
                chord=np.linalg.norm(sub[-1]-sub[0])
                if span>=22 and chord>=max(12.0,tol*1.8):
                    best=(j-i,i,j,c,r);break
    if best:
        _,i,j,c,r=best;out=[]
        if i>=2: out.extend(_fit_design_run(seg[:i+1],tol))
        elif i==1: out.append(_line(seg[0],seg[1]))
        out.append(_arc(seg[i:j+1],c,r))
        if j<=n-3: out.extend(_fit_design_run(seg[j:],tol))
        elif j==n-2: out.append(_line(seg[j],seg[-1]))
        return out

    # No stable circle: split only where the contour most strongly departs from its chord.
    a,b=seg[0],seg[-1];v=b-a;L=np.linalg.norm(v)
    if L<1e-8:return []
    dist=np.abs(np.cross(v,seg-a)/L)
    k=int(np.argmax(dist))
    if 2<=k<=len(seg)-3 and dist[k]>max(2.0,tol*.48):
        return _fit_design_run(seg[:k+1],tol)+_fit_design_run(seg[k:],tol)
    return [_line(seg[0],seg[-1])]

def _merge_collinear_closed(ents,tol):
    if len(ents)<2:return ents
    changed=True
    while changed and len(ents)>1:
        changed=False;out=[];i=0
        while i<len(ents):
            a=ents[i];b=ents[(i+1)%len(ents)]
            if i<len(ents)-1 and a["type"]=="LINE" and b["type"]=="LINE":
                pts=np.asarray([a["start"],a["end"],b["end"]],float)
                lr,lm=_line_fit(pts)
                u=np.asarray(a["end"])-np.asarray(a["start"]);v=np.asarray(b["end"])-np.asarray(b["start"])
                if np.linalg.norm(u)>0 and np.linalg.norm(v)>0:
                    ang=math.degrees(math.acos(float(np.clip(np.dot(u,v)/(np.linalg.norm(u)*np.linalg.norm(v)),-1,1))))
                    if ang<10 and lm<=max(3.0,tol*.65):
                        out.append(_line(a["start"],b["end"]));i+=2;changed=True;continue
            out.append(a);i+=1
        ents=out
    return ents

def _entity_dense_error(seg, typ, param=None):
    seg=np.asarray(seg,float)
    if typ=="LINE":
        a,b=seg[0],seg[-1];v=b-a;L=np.linalg.norm(v)
        if L<1e-9:return 1e9,1e9
        e=np.abs(np.cross(v,seg-a)/L)
    else:
        c,r=param
        e=np.abs(np.linalg.norm(seg-c,axis=1)-r)
    return float(np.sqrt(np.mean(e*e))),float(np.max(e))

def _dense_arc_candidate(seg, fit_tol):
    if len(seg)<7:return None
    f=_circle_fit(seg)
    if not f:return None
    c,r,rms,mx=f
    if not (10.0<=r<=800.0):return None
    a=np.unwrap(np.arctan2(seg[:,1]-c[1],seg[:,0]-c[0]))
    da=np.diff(a); nz=da[np.abs(da)>1e-5]
    if len(nz)<4:return None
    same=max(float(np.mean(nz>0)),float(np.mean(nz<0)))
    span=abs(math.degrees(a[-1]-a[0]))
    chord=float(np.linalg.norm(seg[-1]-seg[0]))
    plen=float(np.sum(np.linalg.norm(np.diff(seg,axis=0),axis=1)))
    alen=float(r*math.radians(span))
    if same<.93 or not (18<=span<=220) or chord<18:return None
    if rms>fit_tol*.45 or mx>fit_tol:return None
    if alen<=1e-6 or not (.88<=plen/alen<=1.14):return None
    sag=r-math.sqrt(max(0,r*r-(chord*.5)**2)) if chord<2*r else r
    if sag<max(2.0,fit_tol*.70):return None
    return c,r,rms,mx,span

def _choose_start(p):
    # Start at the strongest localized corner so a primitive does not wrap across
    # an obvious design vertex.
    n=len(p);step=max(3,min(10,n//35));best=(0,-1)
    for i in range(n):
        a=p[(i-step)%n]-p[i];b=p[(i+step)%n]-p[i]
        na=np.linalg.norm(a);nb=np.linalg.norm(b)
        if na<1e-6 or nb<1e-6:continue
        turn=180-math.degrees(math.acos(float(np.clip(np.dot(a,b)/(na*nb),-1,1))))
        if turn>best[1]:best=(i,turn)
    return best[0]

def _greedy_dense_fit(poly,tol):
    # Fit against the dense contour, not RDP vertices. This prevents a chord from
    # replacing a visible radius simply because both share the same endpoints.
    step=max(2.0,min(3.0,tol*.25))
    p=_smooth_closed(_resample_closed(poly,step),5)
    n=len(p)
    if n<8:return fit_closed_polyline(poly,max(3.0,tol*.5))
    st=_choose_start(p)
    q=np.vstack([p[st:],p[:st],p[st]])
    N=len(q); fit_tol=max(2.5,min(4.5,tol*.42))
    ents=[];i=0
    while i<N-1:
        best_line=None;best_arc=None
        # Maximum physical look-ahead keeps runtime bounded but allows large radii.
        maxj=min(N-1,i+180)
        line_failed=arc_failed=0
        for j in range(i+2,maxj+1):
            seg=q[i:j+1]
            lr,lm=_line_fit(seg)
            if lr<=fit_tol*.42 and lm<=fit_tol:
                best_line=(j,lr,lm);line_failed=0
            else: line_failed+=1
            ac=_dense_arc_candidate(seg,fit_tol)
            if ac is not None:
                best_arc=(j,ac);arc_failed=0
            else: arc_failed+=1
            if j-i>16 and line_failed>8 and arc_failed>12 and best_line and (not best_arc or j-best_arc[0]>12):
                break

        lspan=(best_line[0]-i) if best_line else 0
        aspan=(best_arc[0]-i) if best_arc else 0
        # Arc must explain materially more dense contour than a line, unless its
        # sagitta is unmistakable. This suppresses tiny "ear" arcs from mask noise.
        use_arc=False
        if best_arc:
            j,ac=best_arc;c,r,rr,rm,span=ac
            chord=np.linalg.norm(q[j]-q[i])
            sag=r-math.sqrt(max(0,r*r-(chord*.5)**2)) if chord<2*r else r
            use_arc=(aspan>=max(7,lspan+4) or (span>=35 and sag>=fit_tol*1.8 and aspan>=7))
        if use_arc:
            j,ac=best_arc;c,r,rr,rm,span=ac
            ents.append(_arc(q[i:j+1],c,r));i=j
        elif best_line and lspan>=2:
            j=best_line[0];ents.append(_line(q[i],q[j]));i=j
        else:
            # uncertain local shape: use a short faithful chord rather than inventing
            # a large circle. It can later be edited by the user.
            j=min(N-1,i+max(2,int(round(10.0/step))))
            ents.append(_line(q[i],q[j]));i=j

    ents=_merge_lines(ents,4.0,max(2.0,fit_tol*.75))
    return ents

def _sample_entity(e, ds=2.0):
    if e["type"]=="LINE":
        a=np.asarray(e["start"],float);b=np.asarray(e["end"],float)
        n=max(2,int(np.linalg.norm(b-a)/ds)+1)
        return np.linspace(a,b,n)
    if e["type"]=="ARC":
        c=np.asarray(e["center"],float);r=float(e["radius"])
        a0=math.atan2(e["start"][1]-c[1],e["start"][0]-c[0])
        sgn=1 if e.get("ccw",True) else -1
        span=math.radians(float(e.get("span_deg",0)))
        n=max(4,int(r*span/ds)+1)
        aa=a0+sgn*np.linspace(0,span,n)
        return c+np.c_[np.cos(aa),np.sin(aa)]*r
    return np.empty((0,2))

def fit_fidelity_report(poly,ents):
    # Symmetric nearest-neighbour contour error. No SciPy dependency.
    raw=_resample_closed(poly,2.5)
    fit=np.vstack([_sample_entity(e,2.5) for e in ents if e["type"] in ("LINE","ARC")])
    if len(raw)==0 or len(fit)==0:return {"rms_mm":999.,"max_mm":999.,"p95_mm":999.}
    def nearest(A,B):
        vals=[]
        for k in range(0,len(A),250):
            d=A[k:k+250,None,:]-B[None,:,:]
            vals.extend(np.sqrt(np.min(np.sum(d*d,axis=2),axis=1)).tolist())
        return np.asarray(vals)
    d=np.r_[nearest(raw,fit),nearest(fit,raw)]
    return {"rms_mm":float(np.sqrt(np.mean(d*d))),"max_mm":float(np.max(d)),
            "p95_mm":float(np.percentile(d,95))}

def _suppress_collinear_interruptions(ents,tol):
    if len(ents)<5:return ents
    out=list(ents);changed=True
    while changed:
        changed=False;n=len(out)
        for i in range(n):
            a=out[i]
            if a["type"]!="LINE":continue
            for gapn in (1,2,3):
                j=(i+gapn+1)%n
                if j<=i:continue  # keep implementation simple; seam is chosen at a strong corner
                b=out[j]
                if b["type"]!="LINE":continue
                ua=np.asarray(a["end"])-np.asarray(a["start"]);ub=np.asarray(b["end"])-np.asarray(b["start"])
                La=np.linalg.norm(ua);Lb=np.linalg.norm(ub)
                if min(La,Lb)<25:continue
                ua/=La;ub/=Lb
                ang=math.degrees(math.acos(float(np.clip(abs(np.dot(ua,ub)),-1,1))))
                if ang>5.0:continue
                # distance of B endpoints to A support line
                p0=np.asarray(a["start"]);normal=np.array([-ua[1],ua[0]])
                off=max(abs(np.dot(np.asarray(b["start"])-p0,normal)),abs(np.dot(np.asarray(b["end"])-p0,normal)))
                gap=float(np.linalg.norm(np.asarray(b["start"])-np.asarray(a["end"])))
                if off<=max(4.0,tol*.55) and gap<=max(35.0,tol*3.5):
                    merged=_line(a["start"],b["end"])
                    out=out[:i]+[merged]+out[j+1:]
                    changed=True;break
            if changed:break
    return out

def _line_intersection(e1,e2):
    p=np.asarray(e1["start"],float); r=np.asarray(e1["end"],float)-p
    q=np.asarray(e2["start"],float); s=np.asarray(e2["end"],float)-q
    cr=float(np.cross(r,s))
    if abs(cr)<1e-8:return None
    t=float(np.cross(q-p,s)/cr)
    return p+t*r

def _straighten_near_linear_chains(ents,tol):
    out=list(ents)
    changed=True
    while changed and len(out)>=2:
        changed=False
        # Prefer the longest explainable chain first.
        for win in (5,4,3,2):
            if len(out)<win:continue
            for i in range(0,len(out)-win+1):
                chunk=out[i:i+win]
                # Do not erase an obvious large design radius.
                if any(e["type"]=="ARC" and e.get("span_deg",0)>=45 and e.get("radius",0)>=18 for e in chunk):
                    continue
                pts=[]
                for e in chunk:
                    if e["type"] not in ("LINE","ARC"):pts=[];break
                    pts.append(_sample_entity(e,2.5))
                if not pts:continue
                p=np.vstack(pts);lr,lm=_line_fit(p)
                plen=float(np.sum(np.linalg.norm(np.diff(p,axis=0),axis=1)))
                chord=float(np.linalg.norm(p[-1]-p[0]))
                if chord<18 or plen/chord>1.08:continue
                if lr<=max(1.5,tol*.22) and lm<=max(3.2,tol*.38):
                    out=out[:i]+[_line(p[0],p[-1])]+out[i+win:]
                    changed=True;break
            if changed:break
    return out

def _manufactured_corner_cleanup(ents,tol):
    """Prefer a clean engineered corner over a tiny burr between two strong lines.
    Closed contours are rotated first so cleanup is not defeated by the list seam."""
    out=list(ents)
    if out:
        lens=[np.linalg.norm(np.asarray(e["end"])-np.asarray(e["start"])) if e["type"]=="LINE" else -1 for e in out]
        k=int(np.argmax(lens))
        shift=(k-len(out)//2)%len(out)
        out=out[shift:]+out[:shift]
    changed=True
    while changed and len(out)>=3:
        changed=False;n=len(out)
        for i in range(n-2):
            a=out[i]
            if a["type"]!="LINE":continue
            La=np.linalg.norm(np.asarray(a["end"])-np.asarray(a["start"]))
            if La<45:continue
            for gapn in (1,2):
                j=i+gapn+1
                if j>=len(out):continue
                b=out[j]
                if b["type"]!="LINE":continue
                Lb=np.linalg.norm(np.asarray(b["end"])-np.asarray(b["start"]))
                if Lb<45:continue
                middle=out[i+1:j]
                # Never delete a convincing design radius.
                convincing=False
                for e in middle:
                    if e["type"]=="ARC":
                        chord=np.linalg.norm(np.asarray(e["end"])-np.asarray(e["start"]))
                        if e["radius"]>=18 and e.get("span_deg",0)>=35 and chord>=25:
                            convincing=True
                if convincing:continue
                x=_line_intersection(a,b)
                if x is None:continue
                da=np.linalg.norm(x-np.asarray(a["end"]));db=np.linalg.norm(x-np.asarray(b["start"]))
                # local defect must be small enough to be a burr/contact artefact
                midlen=sum(np.linalg.norm(np.asarray(e["end"])-np.asarray(e["start"])) for e in middle)
                if da<=max(14.,tol*1.5) and db<=max(14.,tol*1.5) and midlen<=max(30.,tol*3.2):
                    aa=dict(a);bb=dict(b);aa["end"]=x.tolist();bb["start"]=x.tolist()
                    out=out[:i]+[aa,bb]+out[j+1:]
                    changed=True;break
            if changed:break
    return out

def fit_design_outline(poly,tol=8.0):
    e=_suppress_collinear_interruptions(_greedy_dense_fit(poly,tol),tol)
    e=_straighten_near_linear_chains(e,tol)
    return _manufactured_corner_cleanup(e,tol)

def entity_measurements(e):
    t=e.get("type")
    if t=="LINE":
        a=np.asarray(e["start"],float);b=np.asarray(e["end"],float);v=b-a
        return {"length_mm":float(np.linalg.norm(v)),
                "angle_deg":float(math.degrees(math.atan2(v[1],v[0])))}
    if t=="ARC":
        return {"radius_mm":float(e["radius"]),"angle_deg":float(e.get("span_deg",0)),
                "arc_length_mm":float(e["radius"]*math.radians(e.get("span_deg",0)))}
    if t=="CIRCLE":
        return {"radius_mm":float(e["radius"]),"diameter_mm":float(e["radius"]*2)}
    return {}

def edit_entity(e, field, value):
    """Parametric entity editing used by UI and tests."""
    import copy
    e=copy.deepcopy(e);v=float(value);t=e["type"]
    if t=="LINE":
        a=np.asarray(e["start"],float);b=np.asarray(e["end"],float)
        mid=(a+b)/2;d=b-a;L=np.linalg.norm(d)
        ang=math.atan2(d[1],d[0])
        if field=="length_mm":
            if v<=0:raise ValueError("长度必须大于0")
            L=v
        elif field=="angle_deg":
            ang=math.radians(v)
        else:raise ValueError("LINE仅支持长度/角度")
        u=np.array([math.cos(ang),math.sin(ang)])
        e["start"]=(mid-u*L/2).tolist();e["end"]=(mid+u*L/2).tolist()
    elif t=="ARC":
        if field!="radius_mm":raise ValueError("ARC当前支持修改半径")
        if v<=0:raise ValueError("半径必须大于0")
        c=np.asarray(e["center"],float)
        for key in ("start","end"):
            p=np.asarray(e[key],float);d=p-c;L=np.linalg.norm(d)
            if L>1e-9:e[key]=(c+d/L*v).tolist()
        e["radius"]=v
    elif t=="CIRCLE":
        if field not in ("radius_mm","diameter_mm"):raise ValueError("CIRCLE仅支持半径/直径")
        r=v if field=="radius_mm" else v/2
        if r<=0:raise ValueError("半径必须大于0")
        e["radius"]=r
    else:raise ValueError("不支持的实体类型")
    return e

# Final override used by app.py
def fit_geometry(outer,holes,tol=8.0,regularize=False):
    oe=fit_design_outline(outer,tol);he=[];kinds=[]
    for h in holes:
        e,k=classify_hole(h,max(2.0,tol*.55));he.append(e);kinds.append(k)
    counts={"LINE":0,"ARC":0,"CIRCLE":0,"SLOT":sum(k=="SLOT" for k in kinds)}
    for e in oe:
        if e["type"] in counts:counts[e["type"]]+=1
    for group in he:
        for e in group:
            if e["type"] in counts:counts[e["type"]]+=1
    result={"outer":oe,"holes":he,"hole_kinds":kinds,"counts":counts,
            "tolerance_mm":tol,"fitter":"dense_fidelity_v3",
            "fidelity":fit_fidelity_report(outer,oe)}
    if regularize: result=regularize_design_geometry(result)
    return result


# ========================= V0.19 control-point editing =========================
def _arc_midpoint(e):
    c=np.asarray(e["center"],float); r=float(e["radius"])
    a0=math.atan2(e["start"][1]-c[1],e["start"][0]-c[0])
    sgn=1 if e.get("ccw",True) else -1
    a=a0+sgn*math.radians(float(e.get("span_deg",0)))*0.5
    return c+r*np.array([math.cos(a),math.sin(a)])

def _circle_three_points(a,b,c):
    a=np.asarray(a,float);b=np.asarray(b,float);c=np.asarray(c,float)
    A=np.array([[2*(b[0]-a[0]),2*(b[1]-a[1])],
                [2*(c[0]-a[0]),2*(c[1]-a[1])]],float)
    B=np.array([b@b-a@a,c@c-a@a],float)
    if abs(np.linalg.det(A))<1e-9:return None
    ctr=np.linalg.solve(A,B); r=float(np.linalg.norm(a-ctr))
    return ctr,r

def _rebuild_arc(start,mid,end,ccw_hint=True):
    z=_circle_three_points(start,mid,end)
    if z is None:
        raise ValueError("拖拽后圆弧退化，请换一个控制点位置")
    ctr,r=z
    if not np.isfinite(r) or r<1e-3 or r>1e6:
        raise ValueError("圆弧半径无效")
    a0=math.atan2(start[1]-ctr[1],start[0]-ctr[0])
    a1=math.atan2(end[1]-ctr[1],end[0]-ctr[0])
    if ccw_hint: span=(a1-a0)%(2*math.pi)
    else: span=(a0-a1)%(2*math.pi)
    # choose the orientation that actually passes through the retained midpoint
    am=math.atan2(mid[1]-ctr[1],mid[0]-ctr[0])
    def on_ccw(x0,xm,x1):
        return ((xm-x0)%(2*math.pi)) <= ((x1-x0)%(2*math.pi))+1e-7
    ccw=bool(ccw_hint)
    if ccw and not on_ccw(a0,am,a1): ccw=False
    elif (not ccw) and on_ccw(a0,am,a1): ccw=True
    span=((a1-a0)%(2*math.pi)) if ccw else ((a0-a1)%(2*math.pi))
    return {"type":"ARC","center":ctr.tolist(),"radius":r,
            "start":np.asarray(start,float).tolist(),"end":np.asarray(end,float).tolist(),
            "start_angle":math.degrees(a0),"end_angle":math.degrees(a1),
            "ccw":ccw,"span_deg":math.degrees(span)}

def _move_entity_endpoint(e, which, target):
    import copy
    q=copy.deepcopy(e); target=np.asarray(target,float)
    if q["type"]=="LINE":
        q[which]=target.tolist(); return q
    if q["type"]=="ARC":
        mid=_arc_midpoint(q)
        if which=="start":
            return _rebuild_arc(target,mid,np.asarray(q["end"],float),q.get("ccw",True))
        return _rebuild_arc(np.asarray(q["start"],float),mid,target,q.get("ccw",True))
    raise ValueError("该实体没有端点控制柄")

def _sync_adjacent_endpoint(group, idx, which, point):
    n=len(group)
    if n<2:return
    if which=="start":
        j=(idx-1)%n
        if group[j].get("type") in ("LINE","ARC"):
            group[j]=_move_entity_endpoint(group[j],"end",point)
    else:
        j=(idx+1)%n
        if group[j].get("type") in ("LINE","ARC"):
            group[j]=_move_entity_endpoint(group[j],"start",point)

def move_fitted_control(fitted, group="outer", group_index=0, entity_index=0,
                        handle="end", target=(0,0)):
    """Move one CAD control point while preserving primitive type and closed topology.
    Handles:
      LINE: start, end
      ARC: start, end, center, radius
      CIRCLE: center, radius
    """
    import copy
    f=copy.deepcopy(fitted)
    arr=f["outer"] if group=="outer" else f["holes"][int(group_index)]
    i=int(entity_index)
    if i<0 or i>=len(arr): raise ValueError("实体索引无效")
    e=arr[i]; t=np.asarray(target,float)

    if e["type"]=="LINE":
        if handle not in ("start","end"): raise ValueError("LINE支持起点/终点控制柄")
        arr[i]=_move_entity_endpoint(e,handle,t)
        _sync_adjacent_endpoint(arr,i,handle,t)

    elif e["type"]=="ARC":
        if handle in ("start","end"):
            arr[i]=_move_entity_endpoint(e,handle,t)
            p=np.asarray(arr[i][handle],float)
            _sync_adjacent_endpoint(arr,i,handle,p)
        elif handle=="center":
            old=np.asarray(e["center"],float);d=t-old
            q=copy.deepcopy(e)
            for k in ("center","start","end"):
                q[k]=(np.asarray(q[k],float)+d).tolist()
            arr[i]=q
            _sync_adjacent_endpoint(arr,i,"start",np.asarray(q["start"]))
            _sync_adjacent_endpoint(arr,i,"end",np.asarray(q["end"]))
        elif handle=="radius":
            c=np.asarray(e["center"],float);r=float(np.linalg.norm(t-c))
            if r<=1e-3: raise ValueError("半径必须大于0")
            q=copy.deepcopy(e);q["radius"]=r
            for k in ("start","end"):
                v=np.asarray(q[k],float)-c;L=np.linalg.norm(v)
                if L>1e-9:q[k]=(c+v/L*r).tolist()
            arr[i]=q
            _sync_adjacent_endpoint(arr,i,"start",np.asarray(q["start"]))
            _sync_adjacent_endpoint(arr,i,"end",np.asarray(q["end"]))
        else: raise ValueError("ARC控制柄无效")

    elif e["type"]=="CIRCLE":
        q=copy.deepcopy(e)
        if handle=="center": q["center"]=t.tolist()
        elif handle=="radius":
            r=float(np.linalg.norm(t-np.asarray(q["center"],float)))
            if r<=1e-3: raise ValueError("半径必须大于0")
            q["radius"]=r
        else: raise ValueError("CIRCLE支持圆心/半径控制柄")
        arr[i]=q
    else:
        raise ValueError("当前实体不支持控制点拖拽")

    # refresh entity counts
    counts={"LINE":0,"ARC":0,"CIRCLE":0,"SLOT":sum(k=="SLOT" for k in f.get("hole_kinds",[]))}
    for x in f.get("outer",[]):
        if x.get("type") in counts:counts[x["type"]]+=1
    for hg in f.get("holes",[]):
        for x in hg:
            if x.get("type") in counts:counts[x["type"]]+=1
    f["counts"]=counts
    f["control_edited"]=True
    return f
