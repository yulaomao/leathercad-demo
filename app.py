from flask import Flask, render_template, request, jsonify, send_file, send_from_directory
from pathlib import Path
import uuid, json, math, os
import cv2, numpy as np, ezdxf
from geometry_fit import fit_geometry, edit_entity, entity_measurements, move_fitted_control

BASE=Path(__file__).resolve().parent
WORK=Path(os.environ.get("LEATHERCAD_WORKDIR", str(BASE/"workspace")))
WORK.mkdir(parents=True, exist_ok=True)
app=Flask(__name__); app.config["MAX_CONTENT_LENGTH"]=50*1024*1024

@app.get("/healthz")
def healthz():
    return {"ok": True, "service": "LeatherCAD"}, 200

DICT=cv2.aruco.DICT_4X4_50; MARKER_MM=70.0; PPM=1.5

def detect(img):
    d=cv2.aruco.getPredefinedDictionary(DICT)
    det=cv2.aruco.ArucoDetector(d,cv2.aruco.DetectorParameters())
    cs,ids,_=det.detectMarkers(cv2.cvtColor(img,cv2.COLOR_BGR2GRAY))
    m={} if ids is None else {int(i):c.reshape(4,2).astype(float) for c,i in zip(cs,ids.flatten())}
    return m,cs,ids

def marker_groups(markers):
    groups=[]
    for ids in [(1,2,3,4),(5,6,7,8),(9,10,11,12),(13,14,15,16)]:
        have=[i for i in ids if i in markers]
        if have: groups.append(have)
    return groups

def local_homographies(markers):
    # Each detected marker alone provides an exact square->image homography.
    # We use all marker inverse mappings to estimate local metric samples; free placement is supported.
    out=[]
    world=np.array([[0,0],[MARKER_MM,0],[MARKER_MM,MARKER_MM],[0,MARKER_MM]],np.float32)
    for mid,c in markers.items():
        H=cv2.getPerspectiveTransform(c.astype(np.float32),world)
        out.append((mid,H))
    return out

def _sheet_groups(markers):
    return [[i for i in ids if i in markers] for ids in
            [(1,2,3,4),(5,6,7,8),(9,10,11,12),(13,14,15,16)]]

def _sheet_page_points(ids):
    pos={1:(10,15),2:(130,15),3:(10,212),4:(130,212)}
    out=[]
    for mid in ids:
        local=(mid-1)%4+1
        x,y=pos[local]
        out.append((mid,np.array([[x,y],[x+70,y],[x+70,y+70],[x,y+70]],np.float32)))
    return out

def _sheet_homography_to_image(markers,ids):
    src=[];dst=[]
    for mid,p in _sheet_page_points(ids):
        src.extend(p.tolist());dst.extend(np.asarray(markers[mid],np.float32).tolist())
    H,_=cv2.findHomography(np.asarray(src,np.float32),np.asarray(dst,np.float32),cv2.RANSAC,2.5)
    return H

def _rigid_fit(src,dst):
    src=np.asarray(src,float);dst=np.asarray(dst,float)
    a=src-src.mean(0);b=dst-dst.mean(0)
    U,_,Vt=np.linalg.svd(a.T@b)
    R=Vt.T@U.T
    if np.linalg.det(R)<0:
        Vt[-1]*=-1;R=Vt.T@U.T
    t=dst.mean(0)-src.mean(0)@R.T
    th=math.atan2(R[1,0],R[0,0])
    return th,t

def _global_metric_homography(markers):
    """Joint robust calibration. Bad/non-coplanar A4 sheets are automatically rejected."""
    from scipy.optimize import least_squares
    all_groups=[g for g in _sheet_groups(markers) if len(g)>=3]
    if not all_groups: raise RuntimeError("至少需要一张完整/近完整A4标定页")

    def fit(groups):
        groups=list(groups)
        groups.sort(key=lambda ids:-sum(abs(cv2.contourArea(np.asarray(markers[i],np.float32))) for i in ids))
        anchor=groups[0]
        H0=_sheet_homography_to_image(markers,anchor)
        H0=H0/H0[2,2];invH=np.linalg.inv(H0)
        poses=[]
        for ids in groups[1:]:
            src=[];dst=[]
            for mid,p in _sheet_page_points(ids):
                g=cv2.perspectiveTransform(np.asarray(markers[mid],np.float32).reshape(-1,1,2),invH).reshape(-1,2)
                src.extend(p.tolist());dst.extend(g.tolist())
            th,t=_rigid_fit(src,dst);poses.extend([th,t[0],t[1]])
        x0=np.r_[H0.reshape(-1)[:8],np.asarray(poses,float)]

        def unpack(x):
            H=np.array([[x[0],x[1],x[2]],[x[3],x[4],x[5]],[x[6],x[7],1.]],float)
            ps=[(0.,np.zeros(2))];k=8
            for _ in groups[1:]:
                ps.append((x[k],np.array([x[k+1],x[k+2]])));k+=3
            return H,ps
        def residual(x,per_sheet=False):
            H,ps=unpack(x);allr=[];prs=[]
            for ids,(th,t) in zip(groups,ps):
                c,sn=math.cos(th),math.sin(th);R=np.array([[c,-sn],[sn,c]])
                rr=[]
                for mid,pag in _sheet_page_points(ids):
                    gp=pag@R.T+t
                    pred=cv2.perspectiveTransform(gp.astype(np.float32).reshape(-1,1,2),H).reshape(-1,2)
                    rr.extend((pred-np.asarray(markers[mid],float)).reshape(-1))
                rr=np.asarray(rr,float);allr.extend(rr);prs.append(float(np.sqrt(np.mean(rr*rr))))
            return prs if per_sheet else np.asarray(allr,float)
        sol=least_squares(lambda x:residual(x),x0,loss="soft_l1",f_scale=1.5,max_nfev=400)
        H,_=unpack(sol.x)
        return H,residual(sol.x,True),groups

    groups=all_groups
    rejected=[]
    while True:
        H,per,groups=fit(groups)
        worst=int(np.argmax(per))
        # A page with >4 px RMS is likely curled, lifted, or has a bad marker localization.
        # Keep at least two sheets when available; one full A4 is still a valid fallback.
        if per[worst] <= 4.0 or len(groups)<=2: break
        rejected.append(groups[worst])
        groups=[g for i,g in enumerate(groups) if i!=worst]

    rms=float(np.sqrt(np.mean(np.square(per))))
    if not np.isfinite(rms) or rms>10.0:
        raise RuntimeError(f"标定一致性不足（RMS={rms:.2f}px），请保证标定纸与工件共面")
    return np.linalg.inv(H),rms,groups,rejected

def rectify(img,markers,ppm=PPM):
    H_mm,rms,groups,rejected=_global_metric_homography(markers)
    h,w=img.shape[:2]
    ic=np.array([[0,0],[w,0],[w,h],[0,h]],np.float32).reshape(-1,1,2)
    q=cv2.perspectiveTransform(ic,H_mm).reshape(-1,2)
    mn=q.min(0)-30;mx=q.max(0)+30
    W=int(np.ceil((mx[0]-mn[0])*ppm));HH=int(np.ceil((mx[1]-mn[1])*ppm))
    if W<=0 or HH<=0 or W*HH>70_000_000:raise RuntimeError("照片倾斜过大，请更接近俯拍")
    T=np.array([[ppm,0,-mn[0]*ppm],[0,ppm,-mn[1]*ppm],[0,0,1]],float)
    Ho=T@H_mm
    rect=cv2.warpPerspective(img,Ho,(W,HH))
    return rect,Ho,mn,{"mode":"bundle_A4_robust","rms_px":rms,"used_sheets":len(groups),"rejected_sheets":len(rejected)}

def marker_mask_rectified(shape,markers,Hout):
    """Mask each actual A4 page polygon using its known 4-marker print layout."""
    mask=np.zeros(shape[:2],np.uint8)
    for ids in _sheet_groups(markers):
        if len(ids)>=3:
            Hpi=_sheet_homography_to_image(markers,ids)  # page mm -> original image
            if Hpi is not None:
                page=np.array([[0,0],[210,0],[210,297],[0,297]],np.float32).reshape(-1,1,2)
                orig=cv2.perspectiveTransform(page,Hpi)
                q=cv2.perspectiveTransform(orig,Hout).reshape(-1,2)
                cv2.fillConvexPoly(mask,np.round(q).astype(np.int32),255)
    # fallback for any isolated marker
    for c in markers.values():
        q=cv2.perspectiveTransform(np.asarray(c,np.float32).reshape(-1,1,2),Hout).reshape(-1,2)
        cv2.fillConvexPoly(mask,np.round(q).astype(np.int32),255)
    pad=max(3,int(round(1.0*PPM)))
    k=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(pad|1,pad|1))
    return cv2.dilate(mask,k,1)

def clean_binary(mask,exclude=None):
    if exclude is not None: mask[exclude>0]=0
    k=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(7,7))
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,k,iterations=2)
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,k,iterations=1)
    return mask

def component_candidates(binary):
    n,L,stats,cents=cv2.connectedComponentsWithStats(binary,8)
    H,W=binary.shape; center=np.array([W/2,H/2],float)
    arr=[]
    for i in range(1,n):
        x,y,w,h,a=stats[i]
        frac=a/max(1,W*H)
        if frac<0.002 or frac>0.82: continue
        d=np.linalg.norm((cents[i]-center)/np.array([W,H],float))
        touches=int(x<=2)+int(y<=2)+int(x+w>=W-2)+int(y+h>=H-2)
        fill=a/max(1,w*h)
        # Workpieces normally do not coincide with the rectified-image border.
        # Background components often touch 2-4 borders; punish them strongly.
        border_factor={0:1.0,1:.22,2:.05,3:.01,4:.003}[touches]
        # Reject huge border-connected slabs unless there is no better candidate.
        if touches>=2 and frac>.18: border_factor*=.05
        center_factor=max(.25,1.15-min(d,.9))
        fill_factor=.65+.35*min(1.0,fill/.55)
        score=a*border_factor*center_factor*fill_factor
        arr.append((score,i,L))
    return sorted(arr,key=lambda x:x[0],reverse=True)

def _valid_rectified_mask(rect):
    # warpPerspective fills outside-source pixels with exact/near black.
    # This mask prevents those artificial black wedges from becoming "background samples".
    return (np.max(rect,axis=2)>3).astype(np.uint8)*255

def _border_background_candidate(rect,exclude):
    """Learn the actual photographed background from the valid image perimeter.
    This is particularly effective for a controlled high-contrast capture station."""
    valid=_valid_rectified_mask(rect)
    valid=cv2.erode(valid,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(9,9)),1)
    dist=cv2.distanceTransform(valid,cv2.DIST_L2,5)
    # Sample a broad band along the real photographed perimeter, excluding A4 sheets.
    band=(valid>0)&(dist<max(30,int(40*PPM)))&(exclude==0)
    lab=cv2.cvtColor(rect,cv2.COLOR_BGR2LAB).astype(np.float32)
    X=lab[band]
    if len(X)<500:return None

    # Robustly fit the dominant background distribution.
    med=np.median(X,axis=0)
    dd=np.linalg.norm(X-med,axis=1)
    X=X[dd<=np.percentile(dd,82)]
    if len(X)<200:return None
    mu=X.mean(0)
    cov=np.cov(X.T)+np.diag([35.0,22.0,22.0])
    inv=np.linalg.pinv(cov)
    D=lab-mu
    md=np.sqrt(np.maximum(0,np.einsum("...i,ij,...j->...",D,inv,D)))
    score=np.clip(md*20,0,255).astype(np.uint8)

    sel=(valid>0)&(exclude==0)
    vals=score[sel]
    if len(vals)<100:return None
    thr,_=cv2.threshold(vals.reshape(-1,1),0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    mask=((score>float(thr))&sel).astype(np.uint8)*255
    mask=clean_binary(mask,None)
    return mask

def _best_component(mask):
    comps=component_candidates(mask)
    if not comps:return None
    _,i,L=comps[0]
    return np.where(L==i,255,0).astype(np.uint8)

def auto_segment(rect,exclude):
    """Automatic segmentation optimized for a controlled capture background.

    Priority:
      1) learn background appearance from the true photographed perimeter;
      2) color/chroma/grayscale ensemble;
      3) choose the most plausible non-border workpiece component.

    The first branch makes future high-contrast capture stations substantially
    more stable without requiring an AI model.
    """
    hsv=cv2.cvtColor(rect,cv2.COLOR_BGR2HSV)
    lab=cv2.cvtColor(rect,cv2.COLOR_BGR2LAB)
    gray=cv2.cvtColor(rect,cv2.COLOR_BGR2GRAY)
    valid=_valid_rectified_mask(rect)

    candidate_masks=[]
    bg=_border_background_candidate(rect,exclude)
    if bg is not None:
        candidate_masks.append(("BORDER_BG",bg,5.00))

    _,sat=cv2.threshold(hsv[:,:,1],0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    candidate_masks.append(("SAT",clean_binary(sat,None),1.35))

    chrom=np.sqrt((lab[:,:,1].astype(float)-128)**2+(lab[:,:,2].astype(float)-128)**2)
    chrom8=np.clip(chrom*4,0,255).astype(np.uint8)
    _,ch=cv2.threshold(chrom8,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    candidate_masks.append(("CHROMA",clean_binary(ch,None),1.45))

    _,gd=cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
    candidate_masks.append(("DARK",clean_binary(gd,None),.72))
    candidate_masks.append(("LIGHT",clean_binary(cv2.bitwise_not(gd),None),.62))

    H,W=gray.shape
    best=None
    for source,m,source_weight in candidate_masks:
        m=np.where(valid>0,m,0).astype(np.uint8)
        comps=component_candidates(m)
        for score,i,L in comps[:4]:
            cand=np.where(L==i,255,0).astype(np.uint8)
            area=np.count_nonzero(cand); frac=area/max(1,np.count_nonzero(valid))
            if frac<.008 or frac>.72: continue

            # Penalize shapes glued to the real image perimeter.
            border=cv2.morphologyEx(valid,cv2.MORPH_GRADIENT,np.ones((11,11),np.uint8))
            contact=np.count_nonzero((cand>0)&(border>0))
            perimeter=max(1,cv2.arcLength(max(cv2.findContours(cand,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0],key=cv2.contourArea),True))
            contact_penalty=1.0/(1.0+4.0*contact/perimeter)

            final=score*source_weight*contact_penalty
            if best is None or final>best[0]:
                best=(final,cand,source)

    if best is None:
        raise RuntimeError("自动分割失败，请使用点选增加/删除进行修正")

    mask=best[1]
    # Smooth only small texture defects; do not alter genuine slots/holes.
    # Remove thin segmentation whiskers / stitching protrusions, then close only tiny pinholes.
    # At 1.5 px/mm, 7 px is ~4.7 mm and remains well below the current 10 mm product tolerance.
    ko=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(11,11))
    kc=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(19,19))
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,ko,1)
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kc,1)
    mask=np.where(valid>0,mask,0).astype(np.uint8)
    return mask


def split_parts(mask,min_area_fraction=.003):
    """Separate disconnected or lightly-touching large workpieces.
    Sweep the erosion scale and accept a split only when it is stable at >=2
    consecutive scales and produces 2-4 substantial cores. This separates the
    two beige pieces at their narrow contacts, while the orange U-part and grey
    plate remain single workpieces."""
    from scipy.ndimage import distance_transform_edt
    fg=(mask>0).astype(np.uint8); H,W=fg.shape
    min_core=H*W*min_area_fraction*.55
    chosen=None;stable=0;prev_count=None

    # 21..91 px ~= 14..61 mm at 1.5 px/mm. A genuine wide arm survives as one core;
    # a narrow accidental contact disappears and reveals the two object cores.
    for ks in (21,31,41,51,61,71,81,91):
        core=cv2.erode(fg,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(ks,ks)),1)
        n,L,st,_=cv2.connectedComponentsWithStats(core,8)
        ids=[i for i in range(1,n) if st[i,cv2.CC_STAT_AREA]>=min_core]
        # require every proposed part to be substantial relative to the largest core
        if 2<=len(ids)<=4:
            aa=sorted([st[i,cv2.CC_STAT_AREA] for i in ids],reverse=True)
            good=(aa[-1]>=aa[0]*.18)
        else: good=False
        if good:
            if prev_count==len(ids): stable+=1
            else: stable=1
            prev_count=len(ids)
            if stable>=2:
                chosen=(L,ids,ks); break
        else:
            stable=0;prev_count=None

    if chosen is None:
        n,L,st,_=cv2.connectedComponentsWithStats(fg,8)
        ids=[i for i in range(1,n) if st[i,cv2.CC_STAT_AREA]>=H*W*min_area_fraction]
        labels=L; valid_ids=ids
    else:
        seed_labels,valid_ids,ks=chosen
        seedmask=np.isin(seed_labels,valid_ids)
        _,inds=distance_transform_edt(~seedmask,return_indices=True)
        labels=seed_labels[inds[0],inds[1]]
        labels=np.where(fg>0,labels,0)

    parts=[]
    for i in valid_ids:
        part=np.where(labels==i,255,0).astype(np.uint8)
        area=int(np.count_nonzero(part))
        if area<H*W*min_area_fraction: continue
        ys,xs=np.where(part>0);x0,x1=int(xs.min()),int(xs.max());y0,y1=int(ys.min()),int(ys.max())
        parts.append({"label":int(i),"area_px":area,"bbox":[x0,y0,x1-x0+1,y1-y0+1],
                      "centroid":[float(xs.mean()),float(ys.mean())],"mask":part})
    parts.sort(key=lambda z:z["area_px"],reverse=True)
    return parts

def select_part(mask,x,y,append=False,current=None):
    """Click a segmented component to select one physical workpiece."""
    parts=split_parts(mask)
    hit=None
    for p in parts:
        xx,yy,w,h=p["bbox"]
        if xx<=x<xx+w and yy<=y<yy+h and p["mask"][int(y),int(x)]>0:
            hit=p;break
    if hit is None:return current if current is not None else mask
    if append and current is not None:
        return cv2.bitwise_or(current,hit["mask"])
    return hit["mask"].copy()

def contours_from_mask(mask,ppm,origin,simplify_mm=3.0):
    # Robust boundary median: remove narrow textile/piping burrs before geometric fitting.
    # 15 px ~= 10 mm at the current 1.5 px/mm working scale; this is intentionally
    # aligned with the present product tolerance and is applied only to CAD extraction.
    mask=cv2.medianBlur(mask,15)
    # CAD-stage silhouette cleanup: suppress <~7 mm textile/piping whiskers while
    # preserving the much larger real slots/holes used in these parts.
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(29,29)),1)
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(19,19)),1)

    cs,h=cv2.findContours(mask,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_NONE)
    if not cs: raise RuntimeError("未找到有效轮廓")
    oi=int(np.argmax([abs(cv2.contourArea(c)) for c in cs])); eps=max(1,simplify_mm*ppm)
    def conv(c):
        p=cv2.approxPolyDP(c,eps,True)[:,0,:].astype(float)
        p[:,0]=p[:,0]/ppm+origin[0];p[:,1]=p[:,1]/ppm+origin[1];return p
    outer=conv(cs[oi]); holes=[]
    if h is not None:
        ch=h[0][oi][2]
        while ch!=-1:
            if abs(cv2.contourArea(cs[ch]))>(10*ppm)**2: holes.append(conv(cs[ch]))
            ch=h[0][ch][0]
    return outer,holes

def report_for(outer,holes,markers):
    mn=outer.min(0);mx=outer.max(0)
    return {"marker_count":len(markers),"marker_ids":sorted(markers),"sheet_count":len(marker_groups(markers)),
      "width_mm":round(float(mx[0]-mn[0]),1),"height_mm":round(float(mx[1]-mn[1]),1),
      "hole_count":len(holes),"vertex_count":len(outer),
      "quality":"高" if len(markers)>=12 else ("中" if len(markers)>=8 else "低")}

def save_state(p,outer,holes,report,fit_tol=10.0,regularize=False):
    fitted=fit_geometry(outer,holes,fit_tol,regularize=regularize)
    report["cad_entities"]=fitted["counts"]
    report["fit_tolerance_mm"]=fit_tol
    (p/"state.json").write_text(json.dumps({"outer":outer.tolist(),"holes":[x.tolist() for x in holes],
      "fitted":fitted,"report":report},ensure_ascii=False),encoding="utf8")

def _dxf_arc(m,e,layer):
    # Image/mm coordinates have +Y downward; DXF uses +Y upward.
    # Mirroring Y reverses orientation, so swap endpoints/angles to preserve the physical arc.
    cx,cy=e["center"]; r=e["radius"]
    s=np.array(e["start"],float); t=np.array(e["end"],float)
    def ang(pt): return math.degrees(math.atan2(-pt[1]+cy,pt[0]-cx))%360
    # After mirror, if original traversal was CCW in image coordinates it becomes CW in DXF.
    # DXF ARC is CCW, hence swap for original image-CCW.
    sa,ea=ang(s),ang(t)
    if e.get("ccw",True): sa,ea=ea,sa
    m.add_arc((float(cx),float(-cy)),float(r),float(sa),float(ea),dxfattribs={"layer":layer})

def _add_entity(m,e,layer):
    typ=e["type"]
    if typ=="LINE":
        a=e["start"];b=e["end"];m.add_line((a[0],-a[1]),(b[0],-b[1]),dxfattribs={"layer":layer})
    elif typ=="ARC": _dxf_arc(m,e,layer)
    elif typ=="CIRCLE":
        c=e["center"];m.add_circle((c[0],-c[1]),e["radius"],dxfattribs={"layer":layer})

def make_dxf(p):
    s=json.loads((p/"state.json").read_text(encoding="utf8"))
    doc=ezdxf.new("R2010");doc.units=ezdxf.units.MM
    for name in ["OUTER","HOLES","RAW_REFERENCE"]:doc.layers.add(name)
    m=doc.modelspace(); fitted=s.get("fitted")
    if fitted:
        for e in fitted["outer"]:_add_entity(m,e,"OUTER")
        for group in fitted["holes"]:
            for e in group:_add_entity(m,e,"HOLES")
    else:
        m.add_lwpolyline([(x,-y) for x,y in s["outer"]],close=True,dxfattribs={"layer":"OUTER"})
        for h in s["holes"]:m.add_lwpolyline([(x,-y) for x,y in h],close=True,dxfattribs={"layer":"HOLES"})
    doc.saveas(p/"result.dxf")


def _seed_features(img,x,y,r=7):
    lab=cv2.cvtColor(img,cv2.COLOR_BGR2LAB).astype(np.float32)
    hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV).astype(np.float32)
    g=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    y0,y1=max(0,y-r),min(img.shape[0],y+r+1);x0,x1=max(0,x-r),min(img.shape[1],x+r+1)
    lp=lab[y0:y1,x0:x1].reshape(-1,3)
    hp=hsv[y0:y1,x0:x1].reshape(-1,3)
    return lab,hsv,g,np.median(lp,0),np.median(hp,0),lp

def _click_region(img,x,y,base_mask=None,mode="add"):
    """One-click whole-region selection.
    Uses adaptive Lab/HSV similarity, local texture, edge barriers and connectivity.
    It is deliberately asymmetric:
      delete -> conservative region belonging to clicked background;
      add    -> more permissive object-region expansion.
    """
    H,W=img.shape[:2];x=int(np.clip(x,0,W-1));y=int(np.clip(y,0,H-1))

    # DELETE has an important semantic advantage: the user is clicking a region
    # that is ALREADY wrongly included in the current mask. First use mask topology,
    # not colour. If that false-positive blob is disconnected from the real object,
    # one click removes the whole blob exactly, regardless of texture/shadows.
    if mode=="delete" and base_mask is not None and base_mask[y,x]>0:
        bm=(base_mask>0).astype(np.uint8)
        n0,L0,st0,_=cv2.connectedComponentsWithStats(bm,8)
        lid0=int(L0[y,x])
        if lid0>0:
            comp=np.where(L0==lid0,255,0).astype(np.uint8)
            comp_area=int(st0[lid0,cv2.CC_STAT_AREA])
            total_area=max(1,int(np.count_nonzero(bm)))
            # Safe whole-component deletion when the clicked blob is not nearly
            # the entire current target mask. This is the common false-background case.
            if comp_area/total_area < .82:
                return comp,0.0

    lab,hsv,gray,seed,seedh,patch=_seed_features(img,x,y)
    # Robust local colour spread
    dd=np.linalg.norm(patch-seed,axis=1)
    spread=float(np.percentile(dd,85))
    tol=float(np.clip(12+2.0*spread,16,42))
    if mode=="add": tol=min(46,tol*1.12)

    # Colour distance. Chroma is slightly more important than L to tolerate illumination.
    dL=np.abs(lab[:,:,0]-seed[0])
    da=lab[:,:,1]-seed[1]; db=lab[:,:,2]-seed[2]
    dC=np.sqrt(da*da+db*db)
    dist=np.sqrt((.48*dL)**2+dC**2)

    # Edge barrier prevents leaking across a real material boundary.
    blur=cv2.GaussianBlur(gray,(5,5),0)
    gx=cv2.Sobel(blur,cv2.CV_32F,1,0,ksize=3); gy=cv2.Sobel(blur,cv2.CV_32F,0,1,ksize=3)
    grad=cv2.magnitude(gx,gy)
    edge_thr=float(np.percentile(grad,78))
    barrier=grad>max(18.0,edge_thr)

    candidate=(dist<=tol)
    # permit texture/stitch edges inside the same material; only strong edges block growth
    strong=grad>max(42.0,float(np.percentile(grad,93)))
    candidate &= ~strong

    # close small stitch/texture gaps, but not large object boundaries
    m=(candidate.astype(np.uint8)*255)
    m=cv2.morphologyEx(m,cv2.MORPH_CLOSE,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(9,9)),iterations=2)
    m=cv2.morphologyEx(m,cv2.MORPH_OPEN,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3)),iterations=1)

    # Guarantee the seed is represented, then choose only its connected component.
    cv2.circle(m,(x,y),3,255,-1)
    n,L,stats,_=cv2.connectedComponentsWithStats(m,8)
    lid=int(L[y,x])
    region=np.where(L==lid,255,0).astype(np.uint8) if lid>0 else np.zeros_like(m)

    # GrabCut refinement for a suspiciously tiny/huge selection.
    frac=np.count_nonzero(region)/max(1,H*W)
    if frac<.002 or frac>.72:
        gc=np.full((H,W),cv2.GC_PR_BGD,np.uint8)
        if base_mask is not None:
            if mode=="add": gc[base_mask>0]=cv2.GC_PR_FGD
            else: gc[base_mask==0]=cv2.GC_PR_FGD
        rr=max(10,int(min(H,W)*.018));cv2.circle(gc,(x,y),rr,cv2.GC_FGD,-1)
        bw=max(3,int(min(H,W)*.008))
        # only force border to background when the click is not near that border
        if y>3*bw: gc[:bw,:]=cv2.GC_BGD
        if y<H-3*bw: gc[-bw:,:]=cv2.GC_BGD
        if x>3*bw: gc[:,:bw]=cv2.GC_BGD
        if x<W-3*bw: gc[:,-bw:]=cv2.GC_BGD
        bg=np.zeros((1,65),np.float64);fg=np.zeros((1,65),np.float64)
        try:
            cv2.grabCut(img,gc,None,bg,fg,4,cv2.GC_INIT_WITH_MASK)
            gm=np.where((gc==cv2.GC_FGD)|(gc==cv2.GC_PR_FGD),255,0).astype(np.uint8)
            n2,L2,st2,_=cv2.connectedComponentsWithStats(gm,8);lid2=int(L2[y,x])
            if lid2>0:
                gr=np.where(L2==lid2,255,0).astype(np.uint8)
                f2=np.count_nonzero(gr)/max(1,H*W)
                if .001<=f2<=.82: region=gr
        except cv2.error: pass

    # For delete, never erase pixels that were not part of the current target mask.
    # This makes one-click deletion predictable and prevents "background selection"
    # from damaging unrelated image areas.
    if mode=="delete" and base_mask is not None:
        region=np.where((region>0)&(base_mask>0),255,0).astype(np.uint8)

    # Fill tiny internal pinholes created by surface texture.
    region=cv2.morphologyEx(region,cv2.MORPH_CLOSE,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(7,7)),1)
    return region,tol

def _flood_region_by_seed(img,x,y,base_mask=None,mode="add"):
    return _click_region(img,int(x),int(y),base_mask,mode)


@app.post("/api/select_part/<pid>")
def select_part_api(pid):
    p=WORK/pid
    if not p.exists(): return jsonify(ok=False,error="项目不存在"),404
    d=request.get_json(force=True);x=int(d.get("x",0));y=int(d.get("y",0));append=bool(d.get("append",False))
    source=p/"mask_all.png" if (p/"mask_all.png").exists() else p/"mask.png"
    mask=cv2.imread(str(source),cv2.IMREAD_GRAYSCALE)
    if mask is None:return jsonify(ok=False,error="缺少分割数据"),400
    chosen=select_part(mask,x,y,append,None)
    cv2.imwrite(str(p/"mask.png"),chosen)
    meta=json.loads((p/"meta.json").read_text(encoding="utf8"));origin=np.asarray(meta["origin"],float);ppm=float(meta["ppm"])
    outer,holes=contours_from_mask(chosen,ppm,origin)
    s=json.loads((p/"state.json").read_text(encoding="utf8"));rep=s["report"];mn=outer.min(0);mx=outer.max(0)
    rep.update(width_mm=round(float(mx[0]-mn[0]),1),height_mm=round(float(mx[1]-mn[1]),1),hole_count=len(holes),vertex_count=len(outer))
    save_state(p,outer,holes,rep,float(s.get("fitted",{}).get("tolerance_mm",10.0)));make_dxf(p)
    ns=json.loads((p/"state.json").read_text(encoding="utf8"))
    return jsonify(ok=True,report=ns["report"],geometry={"outer":ns["outer"],"holes":ns["holes"],"fitted":ns["fitted"]},
                   mask=f"/workspace/{pid}/mask.png?v={uuid.uuid4().hex[:6]}")

@app.post("/api/region_click/<pid>")
def region_click(pid):
    p=WORK/pid
    if not p.exists(): return jsonify(ok=False,error="项目不存在"),404
    d=request.get_json(force=True); x=d.get("x",0); y=d.get("y",0); mode=d.get("mode","add")
    rect=cv2.imread(str(p/"rectified.jpg")); mask=cv2.imread(str(p/"mask.png"),cv2.IMREAD_GRAYSCALE)
    if rect is None or mask is None:return jsonify(ok=False,error="缺少分割数据"),400
    region,tol=_flood_region_by_seed(rect,x,y,mask,mode)
    pixels=int(np.count_nonzero(region))
    if pixels<20:return jsonify(ok=False,error="未找到可扩展的连续区域，请改用画笔"),400
    if mode=="add": mask[region>0]=255
    else: mask[region>0]=0
    mask=clean_binary(mask);cv2.imwrite(str(p/"mask.png"),mask)
    meta=json.loads((p/"meta.json").read_text(encoding="utf8"));origin=np.asarray(meta["origin"],float);ppm=float(meta["ppm"])
    try: outer,holes=contours_from_mask(mask,ppm,origin)
    except Exception as e:return jsonify(ok=False,error=str(e)),400
    s=json.loads((p/"state.json").read_text(encoding="utf8"));rep=s["report"];mn=outer.min(0);mx=outer.max(0)
    rep.update(width_mm=round(float(mx[0]-mn[0]),1),height_mm=round(float(mx[1]-mn[1]),1),hole_count=len(holes),vertex_count=len(outer))
    save_state(p,outer,holes,rep,float(s.get("fitted",{}).get("tolerance_mm",10.0)));make_dxf(p)
    s=json.loads((p/"state.json").read_text(encoding="utf8"))
    return jsonify(ok=True,pixels=pixels,tolerance=round(tol,1),report=s["report"],
      geometry={"outer":s["outer"],"holes":s["holes"],"fitted":s["fitted"]},
      mask=f"/workspace/{pid}/mask.png?v={uuid.uuid4().hex[:6]}")

@app.get("/")
def index():return render_template("index.html")
@app.get("/workspace/<pid>/<name>")
def wf(pid,name):return send_from_directory(WORK/pid,name)

@app.post("/api/process")
def process():
    f=request.files.get("image")
    if not f:return jsonify(ok=False,error="请选择照片"),400
    pid=uuid.uuid4().hex[:12];p=WORK/pid;p.mkdir()
    f.save(p/"original.jpg");img=cv2.imread(str(p/"original.jpg"))
    markers,cs,ids=detect(img)
    if len(markers)<4:return jsonify(ok=False,error=f"仅识别到 {len(markers)} 个 ArUco；至少建议完整识别一张纸的4个码"),400
    vis=img.copy();cv2.aruco.drawDetectedMarkers(vis,cs,ids);cv2.imwrite(str(p/"markers.jpg"),vis)
    try:
        rect,Hout,origin,refid=rectify(img,markers);cv2.imwrite(str(p/"rectified.jpg"),rect)
        ex=marker_mask_rectified(rect.shape,markers,Hout)
        mask_all=auto_segment(rect,ex);cv2.imwrite(str(p/"mask_all.png"),mask_all)
        parts=split_parts(mask_all)
        mask=parts[0]["mask"] if parts else mask_all
        cv2.imwrite(str(p/"mask.png"),mask)
        outer,holes=contours_from_mask(mask,PPM,origin)
        ov=rect.copy()
        cs2,_=cv2.findContours(mask_all,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE);cv2.drawContours(ov,cs2,-1,(0,255,0),2)
        cv2.imwrite(str(p/"overlay.jpg"),ov)
        rep=report_for(outer,holes,markers);rep["reference_marker"]=refid;rep["part_count"]=len(parts)
        save_state(p,outer,holes,rep);make_dxf(p)
        fitted=json.loads((p/"state.json").read_text(encoding="utf8"))["fitted"]
        meta={"origin":origin.tolist(),"ppm":PPM,"markers":sorted(markers),
              "rectified_size":[int(rect.shape[1]),int(rect.shape[0])]}
        (p/"meta.json").write_text(json.dumps(meta,ensure_ascii=False),encoding="utf8")
        return jsonify(ok=True,project_id=pid,report=rep,meta=meta,images={k:f"/workspace/{pid}/{v}" for k,v in
          {"original":"original.jpg","markers":"markers.jpg","rectified":"rectified.jpg","mask":"mask.png","overlay":"overlay.jpg"}.items()},
          geometry={"outer":outer.tolist(),"holes":[h.tolist() for h in holes],"fitted":fitted})
    except Exception as e:return jsonify(ok=False,error=str(e)),500

@app.post("/api/mask_edit/<pid>")
def mask_edit(pid):
    p=WORK/pid
    if not p.exists():return jsonify(ok=False,error="项目不存在"),404
    data=request.get_json(force=True); strokes=data.get("strokes",[])
    mask=cv2.imread(str(p/"mask.png"),cv2.IMREAD_GRAYSCALE)
    rect=cv2.imread(str(p/"rectified.jpg"))
    meta=json.loads((p/"meta.json").read_text(encoding="utf8")); ppm=float(meta["ppm"]);origin=np.array(meta["origin"],float)
    for s in strokes:
        pts=np.asarray(s.get("points",[]),np.int32)
        if len(pts)<1:continue
        val=255 if s.get("mode")=="add" else 0; radius=max(2,int(s.get("radius",20)))
        if len(pts)==1:cv2.circle(mask,tuple(pts[0]),radius,val,-1)
        else:
            cv2.polylines(mask,[pts.reshape(-1,1,2)],False,val,radius*2,lineType=cv2.LINE_AA)
    mask=clean_binary(mask);cv2.imwrite(str(p/"mask.png"),mask)
    outer,holes=contours_from_mask(mask,ppm,origin)
    old=json.loads((p/"state.json").read_text(encoding="utf8"))
    rep=old["report"];mn=outer.min(0);mx=outer.max(0)
    rep.update(width_mm=round(float(mx[0]-mn[0]),1),height_mm=round(float(mx[1]-mn[1]),1),hole_count=len(holes),vertex_count=len(outer))
    save_state(p,outer,holes,rep);make_dxf(p)
    fitted=json.loads((p/"state.json").read_text(encoding="utf8"))["fitted"]
    ov=rect.copy();cc,_=cv2.findContours(mask,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE);cv2.drawContours(ov,cc,-1,(0,255,0),2);cv2.imwrite(str(p/"overlay.jpg"),ov)
    return jsonify(ok=True,report=rep,geometry={"outer":outer.tolist(),"holes":[h.tolist() for h in holes],"fitted":fitted},
                   mask=f"/workspace/{pid}/mask.png?v={uuid.uuid4().hex[:6]}",overlay=f"/workspace/{pid}/overlay.jpg?v={uuid.uuid4().hex[:6]}")

@app.post("/api/save_geometry/<pid>")
def save_geom(pid):
    p=WORK/pid;d=request.get_json(force=True);old=json.loads((p/"state.json").read_text(encoding="utf8"))
    outer=np.asarray(d.get("outer"),float);holes=[np.asarray(x,float) for x in d.get("holes",[])]
    rep=old["report"];mn=outer.min(0);mx=outer.max(0);rep.update(width_mm=round(float(mx[0]-mn[0]),1),height_mm=round(float(mx[1]-mn[1]),1),hole_count=len(holes),vertex_count=len(outer))
    save_state(p,outer,holes,rep);make_dxf(p);return jsonify(ok=True,report=rep)
@app.get("/api/export/<pid>")
def export(pid):p=WORK/pid;make_dxf(p);return send_file(p/"result.dxf",as_attachment=True,download_name="result.dxf")

@app.get("/api/health")
def health():
    return jsonify(ok=True, version="0.4")

@app.get("/api/state/<pid>")
def get_state(pid):
    p=WORK/pid
    if not (p/"state.json").exists(): return jsonify(ok=False,error="项目不存在"),404
    s=json.loads((p/"state.json").read_text(encoding="utf8"))
    return jsonify(ok=True,**s)

@app.post("/api/refit/<pid>")
def refit(pid):
    p=WORK/pid
    if not p.exists(): return jsonify(ok=False,error="项目不存在"),404
    d=request.get_json(silent=True) or {}
    tol=float(d.get("tolerance_mm",10.0)); tol=max(.3,min(15.0,tol))
    regularize=bool(d.get("regularize",False))
    s=json.loads((p/"state.json").read_text(encoding="utf8"))
    outer=np.asarray(s["outer"],float); holes=[np.asarray(x,float) for x in s["holes"]]
    rep=s["report"]; save_state(p,outer,holes,rep,tol,regularize=regularize); make_dxf(p)
    s=json.loads((p/"state.json").read_text(encoding="utf8"))
    return jsonify(ok=True,fitted=s["fitted"],report=s["report"])


@app.post("/api/edit_entity/<pid>")
def edit_entity_api(pid):
    p=WORK/pid
    if not p.exists(): return jsonify(ok=False,error="项目不存在"),404
    d=request.get_json(force=True)
    group=d.get("group","outer"); gi=int(d.get("group_index",0)); ei=int(d.get("entity_index",0))
    field=d.get("field"); value=d.get("value")
    s=json.loads((p/"state.json").read_text(encoding="utf8"))
    fitted=s["fitted"]
    try:
        if group=="outer":
            fitted["outer"][ei]=edit_entity(fitted["outer"][ei],field,value)
            edited=fitted["outer"][ei]
        else:
            fitted["holes"][gi][ei]=edit_entity(fitted["holes"][gi][ei],field,value)
            edited=fitted["holes"][gi][ei]
    except (IndexError,ValueError,TypeError) as ex:
        return jsonify(ok=False,error=str(ex)),400
    counts={"LINE":0,"ARC":0,"CIRCLE":0,"SLOT":sum(k=="SLOT" for k in fitted.get("hole_kinds",[]))}
    for e in fitted["outer"]:
        if e["type"] in counts: counts[e["type"]]+=1
    for hg in fitted["holes"]:
        for e in hg:
            if e["type"] in counts: counts[e["type"]]+=1
    fitted["counts"]=counts;s["fitted"]=fitted;s["report"]["cad_entities"]=counts
    (p/"state.json").write_text(json.dumps(s,ensure_ascii=False),encoding="utf8")
    make_dxf(p)
    return jsonify(ok=True,entity=edited,measurements=entity_measurements(edited),
                   fitted=fitted,report=s["report"])



@app.post("/api/save_fitted/<pid>")
def save_fitted_api(pid):
    p=WORK/pid
    if not p.exists(): return jsonify(ok=False,error="项目不存在"),404
    d=request.get_json(force=True); fitted=d.get("fitted")
    if not isinstance(fitted,dict) or "outer" not in fitted or "holes" not in fitted:
        return jsonify(ok=False,error="CAD数据格式无效"),400
    s=json.loads((p/"state.json").read_text(encoding="utf8"))
    s["fitted"]=fitted;s["report"]["cad_entities"]=fitted.get("counts",s["report"].get("cad_entities",{}))
    (p/"state.json").write_text(json.dumps(s,ensure_ascii=False),encoding="utf8");make_dxf(p)
    return jsonify(ok=True)

@app.post("/api/drag_control/<pid>")
def drag_control_api(pid):
    p=WORK/pid
    if not p.exists(): return jsonify(ok=False,error="项目不存在"),404
    d=request.get_json(force=True)
    s=json.loads((p/"state.json").read_text(encoding="utf8"))
    try:
        fitted=move_fitted_control(
            s["fitted"],
            group=d.get("group","outer"),
            group_index=int(d.get("group_index",0)),
            entity_index=int(d.get("entity_index",0)),
            handle=d.get("handle","end"),
            target=d.get("target",[0,0])
        )
    except (ValueError,TypeError,IndexError) as ex:
        return jsonify(ok=False,error=str(ex)),400
    s["fitted"]=fitted
    s["report"]["cad_entities"]=fitted.get("counts",{})
    (p/"state.json").write_text(json.dumps(s,ensure_ascii=False),encoding="utf8")
    make_dxf(p)
    return jsonify(ok=True,fitted=fitted,report=s["report"])

@app.get("/api/meta/<pid>")
def get_meta_api(pid):
    p=WORK/pid
    if not (p/"meta.json").exists(): return jsonify(ok=False,error="缺少项目元数据"),404
    return jsonify(ok=True,meta=json.loads((p/"meta.json").read_text(encoding="utf8")))

@app.post("/api/reset_mask/<pid>")
def reset_mask(pid):
    p=WORK/pid
    if not p.exists(): return jsonify(ok=False,error="项目不存在"),404
    # Re-run automatic segmentation from rectified image.
    rect=cv2.imread(str(p/"rectified.jpg"))
    # Marker exclusion is no longer available in rectified coordinates after reload;
    # automatic reset uses a blank exclusion mask, adequate as a user-requested reset.
    mask=auto_segment(rect,np.zeros(rect.shape[:2],np.uint8))
    cv2.imwrite(str(p/"mask.png"),mask)
    meta=json.loads((p/"meta.json").read_text(encoding="utf8")); origin=np.asarray(meta["origin"],float); ppm=float(meta["ppm"])
    outer,holes=contours_from_mask(mask,ppm,origin)
    s=json.loads((p/"state.json").read_text(encoding="utf8")); rep=s["report"]
    mn=outer.min(0);mx=outer.max(0);rep.update(width_mm=round(float(mx[0]-mn[0]),1),height_mm=round(float(mx[1]-mn[1]),1),hole_count=len(holes),vertex_count=len(outer))
    save_state(p,outer,holes,rep);make_dxf(p)
    s=json.loads((p/"state.json").read_text(encoding="utf8"))
    return jsonify(ok=True,geometry={"outer":s["outer"],"holes":s["holes"],"fitted":s["fitted"]},report=s["report"],
      mask=f"/workspace/{pid}/mask.png?v={uuid.uuid4().hex[:6]}")

if __name__=="__main__":
    app.run("0.0.0.0", int(os.environ.get("PORT", "5000")), debug=False)
