"""
Real-photo regression for LeatherCAD segmentation.
Run:
    .venv\\Scripts\\python.exe regression_real_photos.py <folder-containing-jpgs>

It tests:
- ArUco detection
- automatic segmentation
- photometric stability under brightness/contrast/blur changes
- one-click delete on injected false-positive blobs
Outputs masks/overlays and summary.json under regression_output/.
"""
from pathlib import Path
import sys,json,cv2,numpy as np
import app

ROOT=Path(sys.argv[1]) if len(sys.argv)>1 else Path(".")
OUT=Path("regression_output");OUT.mkdir(exist_ok=True)

def iou(a,b):
    a=a>0;b=b>0
    u=np.count_nonzero(a|b)
    return np.count_nonzero(a&b)/u if u else 1.0

def augment(img,kind):
    if kind=="bright": return cv2.convertScaleAbs(img,alpha=1.0,beta=22)
    if kind=="dark": return cv2.convertScaleAbs(img,alpha=.86,beta=-8)
    if kind=="contrast": return cv2.convertScaleAbs(img,alpha=1.18,beta=-18)
    if kind=="blur": return cv2.GaussianBlur(img,(5,5),1.0)
    return img

rows=[]
for fp in sorted(ROOT.glob("*.jpg")):
    im=cv2.imread(str(fp))
    markers,_,_=app.detect(im)
    if len(markers)<4: continue

    def process(src):
        mk,_,_=app.detect(src)
        if len(mk)<4:return None,None
        rect,H,o,_=app.rectify(src,mk)
        ex=app.marker_mask_rectified(rect.shape,mk,H)
        return app.auto_segment(rect,ex),rect

    base,rect=process(im)
    cv2.imwrite(str(OUT/(fp.stem+"_mask.png")),base)

    st=[]
    for kind in ["bright","dark","contrast","blur"]:
        aug=augment(im,kind)  # IMPORTANT: source-photo augmentation, before rectification
        mm,_=process(aug)
        if mm is None: continue
        if mm.shape!=base.shape:
            mm=cv2.resize(mm,(base.shape[1],base.shape[0]),interpolation=cv2.INTER_NEAREST)
        st.append(iou(base,mm))
        cv2.imwrite(str(OUT/(fp.stem+"_"+kind+"_mask.png")),mm)

    # Inject a disconnected false positive into the current mask and verify one-click deletion.
    test=base.copy()
    Hh,Ww=test.shape
    false=np.zeros_like(test)
    cv2.circle(false,(max(25,Ww//12),max(25,Hh//12)),max(12,min(Hh,Ww)//35),255,-1)
    false[base>0]=0
    test[false>0]=255
    ys,xs=np.where(false>0)
    delete_ok=None
    if len(xs):
        x=int(np.median(xs));y=int(np.median(ys))
        reg,_=app._click_region(rect,x,y,test,"delete")
        after=test.copy();after[reg>0]=0
        removed=np.count_nonzero((false>0)&(after==0))/max(1,np.count_nonzero(false))
        retained=np.count_nonzero((base>0)&(after>0))/max(1,np.count_nonzero(base))
        delete_ok={"false_removed":removed,"target_retained":retained}

    rows.append({
      "file":fp.name,"markers":len(markers),
      "mask_fraction":float(np.count_nonzero(base)/base.size),
      "photometric_iou_mean":float(np.mean(st)) if st else None,
      "photometric_iou_min":float(np.min(st)) if st else None,
      "click_delete":delete_ok
    })

(OUT/"summary.json").write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding="utf8")
print(json.dumps(rows,ensure_ascii=False,indent=2))
