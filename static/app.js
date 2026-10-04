const S={file:null,pid:null,images:{},geometry:null,report:null,meta:null,stage:"import",view:"original",maskMode:"regionAdd",strokes:[],drawing:null,base:null,mask:null,cadMode:"fitted",history:[],selected:null,drag:null,ghostFitted:null};
const $=q=>document.querySelector(q),$$=q=>[...document.querySelectorAll(q)];
const toast=t=>{let x=$("#toast");x.textContent=t;x.classList.add("show");setTimeout(()=>x.classList.remove("show"),1800)};
function status(t){$("#topStatus").textContent=t}
function steps(n){$$(".step").forEach((e,i)=>{e.classList.toggle("active",i===n);e.classList.toggle("done",i<n)})}
function setFile(f){if(!f)return;S.file=f;S.pid=null;S.images={original:URL.createObjectURL(f)};S.geometry=null;$("#emptyPage").classList.add("hidden");$("#workPage").classList.remove("hidden");status("照片已载入");processImage()}
$("#fileInput").onchange=e=>setFile(e.target.files[0]);$("#heroInput").onchange=e=>setFile(e.target.files[0]);
const dz=$("#dropZone");["dragenter","dragover"].forEach(x=>dz.addEventListener(x,e=>{e.preventDefault();dz.classList.add("drag")}));["dragleave","drop"].forEach(x=>dz.addEventListener(x,e=>{e.preventDefault();dz.classList.remove("drag")}));dz.ondrop=e=>setFile(e.dataTransfer.files[0]);
async function processImage(){steps(1);status("正在自动标定和分割…");view("original");let fd=new FormData();fd.append("image",S.file);try{let r=await fetch("/api/process",{method:"POST",body:fd}),j=await r.json();if(!j.ok)throw Error(j.error);S.pid=j.project_id;S.images={...S.images,...j.images};S.geometry=j.geometry;S.report=j.report;S.meta=j.meta||null;update();$("#exportBtn").disabled=false;$("#refitBtn").disabled=false;stage("calibration");toast("自动处理完成，请检查标定结果")}catch(e){status("处理失败");toast(e.message)}}
function update(){let r=S.report||{},c=r.cad_entities||{};if(r.part_count>1)status(`检测到 ${r.part_count} 个独立零件 · 当前显示一个，可用“选择零件”切换`);$("#qualityVal").textContent=r.quality?`${r.quality} · 可用`:"—";$("#markerVal").textContent=r.marker_count?`${r.sheet_count||1} 张标定纸 / ${r.marker_count} 个标记`:"等待处理";$("#widthVal").textContent=r.width_mm!=null?r.width_mm+" mm":"—";$("#heightVal").textContent=r.height_mm!=null?r.height_mm+" mm":"—";for(let [id,k] of [["lineCount","LINE"],["arcCount","ARC"],["circleCount","CIRCLE"],["slotCount","SLOT"]])$("#"+id).textContent=c[k]??"—";$("#fitTolerance").value=r.fit_tolerance_mm||8;setCheck("#checkClosed",!!S.geometry);setCheck("#checkCal",(r.marker_count||0)>=4);setCheck("#checkCad",!!S.geometry?.fitted)}
function setCheck(q,ok){let e=$(q);e.textContent=ok?"✓":"○";e.classList.toggle("ok",ok)}
const stageInfo={calibration:["标定检查","确认 ArUco 是否完整识别，并检查俯视校正是否合理"],mask:["分割确认","绿色区域应完整覆盖待切工件；必要时直接用画笔修正"],cad:["CAD 检查","比较原始轮廓与最终几何实体，确认尺寸和拟合结果"],export:["导出检查","确认闭合、标定、CAD 几何和单位后导出 DXF"]};
function stage(s){S.stage=s;let map={calibration:1,mask:2,cad:3,export:4};steps(map[s]??0);let d=stageInfo[s];if(d){$("#pageTitle").textContent=d[0];$("#pageDesc").textContent=d[1]}if(s==="calibration")view("markers");if(s==="mask")view("mask");if(s==="cad"||s==="export")view("cad");$("#backBtn").style.visibility=s==="calibration"?"hidden":"visible";$("#nextBtn").textContent=s==="export"?"导出 DXF":"确认，下一步 →";$("#nextHint").textContent=s==="mask"?"绿色区域正确即可继续":s==="cad"?"建议使用“叠加对比”检查拟合":"请检查当前结果"}
$("#nextBtn").onclick=async()=>{if(S.stage==="calibration")stage("mask");else if(S.stage==="mask"){if(S.strokes.length)await applyMask();stage("cad")}else if(S.stage==="cad")stage("export");else exportDXF()};$("#backBtn").onclick=()=>{if(S.stage==="mask")stage("calibration");else if(S.stage==="cad")stage("mask");else if(S.stage==="export")stage("cad")};$("#exportBtn").onclick=exportDXF;function exportDXF(){if(S.pid)location.href=`/api/export/${S.pid}`}
$$(".step").forEach((e,i)=>e.onclick=()=>{if(!S.pid&&i>0)return;i===0?$("#fileInput").click():stage(["","calibration","mask","cad","export"][i])});$$("#viewTabs button").forEach(b=>b.onclick=()=>view(b.dataset.view));
function view(v){S.view=v;$$("#viewTabs button").forEach(b=>b.classList.toggle("active",b.dataset.view===v));$("#imageStage").classList.add("hidden");$("#maskEditor").classList.add("hidden");$("#cadStage").classList.add("hidden");if(v==="mask"&&S.images.mask){$("#maskEditor").classList.remove("hidden");loadMask()}else if(v==="cad"&&S.geometry){$("#cadStage").classList.remove("hidden");renderCad()}else if(S.images[v]){$("#imageStage").classList.remove("hidden");$("#stageImage").src=S.images[v]}}
const img=u=>new Promise((ok,no)=>{let i=new Image;i.onload=()=>ok(i);i.onerror=no;i.src=u+(u.includes("?")?"&":"?")+"t="+Date.now()});
async function loadMask(){[S.base,S.mask]=await Promise.all([img(S.images.rectified),img(S.images.mask)]);let c=$("#maskCanvas");c.width=S.base.naturalWidth;c.height=S.base.naturalHeight;drawMask()}
function drawMask(){let c=$("#maskCanvas"),x=c.getContext("2d");x.clearRect(0,0,c.width,c.height);x.drawImage(S.base,0,0);if(S.mask){let t=document.createElement("canvas");t.width=c.width;t.height=c.height;let z=t.getContext("2d");z.drawImage(S.mask,0,0,t.width,t.height);let d=z.getImageData(0,0,t.width,t.height),o=z.createImageData(t.width,t.height);for(let i=0;i<d.data.length;i+=4)if(d.data[i]>80){o.data[i]=35;o.data[i+1]=220;o.data[i+2]=130;o.data[i+3]=95}z.putImageData(o,0,0);x.drawImage(t,0,0)}S.strokes.forEach(s=>drawStroke(s));if(S.drawing)drawStroke(S.drawing)}
function drawStroke(s){let x=$("#maskCanvas").getContext("2d");x.save();x.strokeStyle=s.mode==="add"?"rgba(20,230,115,.8)":"rgba(240,65,75,.8)";x.lineWidth=s.radius*2;x.lineCap="round";x.lineJoin="round";x.beginPath();x.moveTo(...s.points[0]);s.points.slice(1).forEach(p=>x.lineTo(...p));x.stroke();x.restore()}
function cp(e){let c=$("#maskCanvas"),r=c.getBoundingClientRect();return[Math.round((e.clientX-r.left)*c.width/r.width),Math.round((e.clientY-r.top)*c.height/r.height)]}
$("#maskCanvas").onpointerdown=async e=>{let p=cp(e);if(S.maskMode==="partSelect"){await selectPhysicalPart(p);return}if(S.maskMode==="regionAdd"||S.maskMode==="regionErase"){await regionClick(p,S.maskMode==="regionAdd"?"add":"erase");return}S.drawing={mode:S.maskMode,radius:+$("#brushSize").value,points:[p]};e.target.setPointerCapture?.(e.pointerId)};$("#maskCanvas").onpointermove=e=>{if(S.maskMode==="add"||S.maskMode==="erase")cursor(e);else $("#brushCursor").style.display="none";if(S.drawing){S.drawing.points.push(cp(e));drawMask()}};$("#maskCanvas").onpointerup=()=>{if(S.drawing){S.strokes.push(S.drawing);S.drawing=null;drawMask()}};$("#maskCanvas").onpointerleave=()=>$("#brushCursor").style.display="none";
function cursor(e){let c=$("#maskCanvas"),r=c.getBoundingClientRect(),u=$("#brushCursor"),rad=+$("#brushSize").value*(r.width/c.width);u.style.display="block";u.style.width=u.style.height=rad*2+"px";u.style.left=(e.clientX-r.left-rad+8)+"px";u.style.top=(e.clientY-r.top-rad+8)+"px"}
function maskMode(m){S.maskMode=m;for(let [id,v] of [["partSelect","partSelect"],["regionAdd","regionAdd"],["regionErase","regionErase"],["maskAdd","add"],["maskErase","erase"]])$("#"+id).classList.toggle("active",m===v);$("#brushCtl").style.opacity=(m==="add"||m==="erase")?"1":".35"}
$("#partSelect").onclick=()=>maskMode("partSelect");$("#regionAdd").onclick=()=>maskMode("regionAdd");$("#regionErase").onclick=()=>maskMode("regionErase");$("#maskAdd").onclick=()=>maskMode("add");$("#maskErase").onclick=()=>maskMode("erase");$("#brushSize").oninput=e=>$("#brushVal").textContent=e.target.value;$("#maskUndo").onclick=()=>{S.strokes.pop();drawMask()};
document.addEventListener("keydown",e=>{if(S.view!=="mask")return;if(e.key==="0")maskMode("partSelect");if(e.key==="1")maskMode("regionAdd");if(e.key==="2")maskMode("regionErase");if(e.key.toLowerCase()==="a")maskMode("add");if(e.key.toLowerCase()==="e")maskMode("erase");if((e.ctrlKey||e.metaKey)&&e.key==="z"){e.preventDefault();S.strokes.pop();drawMask()}});
async function selectPhysicalPart(p){status("正在切换独立零件…");let r=await fetch(`/api/select_part/${S.pid}`,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({x:p[0],y:p[1],append:false})}),j=await r.json();if(!j.ok){toast(j.error);return}S.geometry=j.geometry;S.report=j.report;S.images.mask=j.mask;update();await loadMask();toast("已切换到所选零件")}
async function regionClick(p,mode){status(mode==="add"?"正在识别整块目标区域…":"正在识别整块背景区域…");let r=await fetch(`/api/region_click/${S.pid}`,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({x:p[0],y:p[1],mode})}),j=await r.json();if(!j.ok){toast(j.error);status("点选未生效，可改用画笔");return}S.geometry=j.geometry;S.report=j.report;S.images.mask=j.mask;update();await loadMask();status("区域修正完成");toast(`${mode==="add"?"已增加":"已删除"}整块连续区域`)}
async function applyMask(){if(!S.strokes.length)return;status("应用分割修正…");let r=await fetch(`/api/mask_edit/${S.pid}`,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({strokes:S.strokes})}),j=await r.json();if(!j.ok)return toast(j.error);S.geometry=j.geometry;S.report=j.report;S.images.mask=j.mask;S.images.overlay=j.overlay;S.strokes=[];update();status("分割修正已保存")}
$("#maskReset").onclick=async()=>{if(!confirm("恢复自动分割结果？当前未应用的画笔修改会丢失。"))return;S.strokes=[];let r=await fetch(`/api/reset_mask/${S.pid}`,{method:"POST"}),j=await r.json();if(j.ok){S.geometry=j.geometry;S.report=j.report;S.images.mask=j.mask;update();loadMask();toast("已恢复自动分割")}};
function bounds(p){let xs=p.map(x=>x[0]),ys=p.map(x=>x[1]);return{minx:Math.min(...xs),maxx:Math.max(...xs),miny:Math.min(...ys),maxy:Math.max(...ys)}}function CT(){let b=bounds(S.geometry.outer),pad=90,s=Math.min((1200-2*pad)/Math.max(1,b.maxx-b.minx),(780-2*pad)/Math.max(1,b.maxy-b.miny));return{s,ox:pad-b.minx*s,oy:pad-b.miny*s,b}}function cv(p,T){return[p[0]*T.s+T.ox,p[1]*T.s+T.oy]}function inv(x,y,T){return[(x-T.ox)/T.s,(y-T.oy)/T.s]}function pd(poly,T){return poly.map((p,i)=>{let q=cv(p,T);return`${i?"L":"M"}${q[0]},${q[1]}`}).join(" ")+" Z"}
function svgEl(tag,attrs={}){let x=document.createElementNS("http://www.w3.org/2000/svg",tag);Object.entries(attrs).forEach(([k,v])=>x.setAttribute(k,v));return x}
function fittedEntityRefs(){
 let out=[];(S.geometry?.fitted?.outer||[]).forEach((e,i)=>out.push({e,group:"outer",gi:0,ei:i}));
 (S.geometry?.fitted?.holes||[]).forEach((hg,gi)=>hg.forEach((e,ei)=>out.push({e,group:"holes",gi,ei})));return out
}
function renderEntitySvg(layer,e,cl,T){
 let x;
 if(e.type==="LINE"){let a=cv(e.start,T),b=cv(e.end,T);x=svgEl("line",{x1:a[0],y1:a[1],x2:b[0],y2:b[1]})}
 else if(e.type==="CIRCLE"){let c=cv(e.center,T);x=svgEl("circle",{cx:c[0],cy:c[1],r:e.radius*T.s})}
 else if(e.type==="ARC"){let a=cv(e.start,T),b=cv(e.end,T),r=e.radius*T.s;x=svgEl("path",{d:`M${a[0]},${a[1]} A${r},${r} 0 ${e.span_deg>180?1:0} ${e.ccw?1:0} ${b[0]},${b[1]}`})}
 if(x){x.setAttribute("class",cl);layer.appendChild(x)} return x
}
function allEntityRefs(fitted=S.geometry?.fitted){
 let out=[];(fitted?.outer||[]).forEach((e,i)=>out.push({e,group:"outer",gi:0,ei:i}));
 (fitted?.holes||[]).forEach((hg,gi)=>hg.forEach((e,ei)=>out.push({e,group:"holes",gi,ei})));return out
}
function updateReferenceLayer(T){
 let g=$("#referenceLayer");g.innerHTML="";
 if(!$("#refToggle").checked||!S.meta)return;
 let src=$("#refSource").value==="mask"?S.images.mask:S.images.rectified;if(!src)return;
 let [wpx,hpx]=S.meta.rectified_size||[0,0],ppm=+S.meta.ppm,org=S.meta.origin;
 if(!wpx||!hpx||!ppm||!org)return;
 let q=cv(org,T),im=svgEl("image",{href:src,x:q[0],y:q[1],width:(wpx/ppm)*T.s,height:(hpx/ppm)*T.s,
   preserveAspectRatio:"none",opacity:(+$("#refOpacity").value/100),class:"referenceImage"});
 g.appendChild(im)
}
function renderGhost(T){
 let g=$("#ghostLayer");g.innerHTML="";
 if(!$("#ghostToggle").checked||!S.ghostFitted)return;
 allEntityRefs(S.ghostFitted).forEach(r=>renderEntitySvg(g,r.e,"ghostEntity",T))
}
function arcMid(e){
 let c=e.center,a0=Math.atan2(e.start[1]-c[1],e.start[0]-c[0]),sgn=e.ccw?1:-1,a=a0+sgn*(e.span_deg*Math.PI/180)/2;
 return[c[0]+e.radius*Math.cos(a),c[1]+e.radius*Math.sin(a)]
}
function addHandle(layer,T,ref,handle,p,cls=""){
 let q=cv(p,T),x=svgEl("circle",{cx:q[0],cy:q[1],r:6,class:`ctrlHandle ${cls}`,title:`${ref.e.type} ${handle}`});
 x.dataset.group=ref.group;x.dataset.gi=ref.gi;x.dataset.ei=ref.ei;x.dataset.handle=handle;layer.appendChild(x)
}
function drawControls(T){
 let g=$("#controlLayer");g.innerHTML="";if(S.cadMode!=="control")return;
 allEntityRefs().forEach(ref=>{
   let e=ref.e;
   if(e.type==="LINE"){addHandle(g,T,ref,"start",e.start);addHandle(g,T,ref,"end",e.end)}
   else if(e.type==="ARC"){
     addHandle(g,T,ref,"start",e.start);addHandle(g,T,ref,"end",e.end);addHandle(g,T,ref,"center",e.center,"arcCenter");
     let pm=arcMid(e),c=cv(e.center,T),r=cv(pm,T);g.appendChild(svgEl("line",{x1:c[0],y1:c[1],x2:r[0],y2:r[1],class:"controlGuide"}));
     addHandle(g,T,ref,"radius",pm,"radius")
   }else if(e.type==="CIRCLE"){
     addHandle(g,T,ref,"center",e.center,"arcCenter");addHandle(g,T,ref,"radius",[e.center[0]+e.radius,e.center[1]],"radius")
   }
 });
 let t=svgEl("text",{x:18,y:28,class:"controlLegend"});t.textContent="蓝=端点  黄=圆心  粉=半径；拖拽后自动同步 DXF";g.appendChild(t)
}
function renderCad(){
 let g=$("#cadLayer");g.innerHTML="";if(!S.geometry)return;let T=CT();S.T=T;updateReferenceLayer(T);renderGhost(T);
 let addPath=(poly,cl)=>{let x=svgEl("path",{d:pd(poly,T),class:cl});g.appendChild(x)};
 if(S.cadMode==="raw"||S.cadMode==="compare")addPath(S.geometry.outer,S.cadMode==="compare"?"rawCompare":"fitOuter");
 if(S.cadMode!=="raw"&&S.geometry.fitted){
   let drawEntity=(ref,cl)=>{
     let x=renderEntitySvg(g,ref.e,cl,T);if(x){x.onclick=()=>entityInfo(ref);if(S.cadMode!=="control")drawEntityDims(g,T,ref)}
   };
   S.geometry.fitted.outer.forEach((e,i)=>drawEntity({e,group:"outer",gi:0,ei:i},"fitOuter"));
   S.geometry.fitted.holes.forEach((hg,gi)=>hg.forEach((e,ei)=>drawEntity({e,group:"holes",gi,ei},"fitHole")));
 }
 if(S.cadMode==="edit")S.geometry.outer.forEach((p,i)=>{let q=cv(p,T),x=svgEl("circle",{cx:q[0],cy:q[1],r:5,class:"node"+(S.selected===i?" sel":"")});x.dataset.i=i;g.appendChild(x)});
 drawControls(T);dims(g,T)
}
function dimText(g,x,y,text,ref,field,value){
 let t=svgEl("text",{x,y,class:"entityDim"});t.textContent=text;t.title="双击修改";t.ondblclick=e=>{e.stopPropagation();editDimension(ref,field,value)};g.appendChild(t)
}
function drawEntityDims(g,T,ref){
 let e=ref.e;
 if(e.type==="LINE"){
   let a=cv(e.start,T),b=cv(e.end,T),mx=(a[0]+b[0])/2,my=(a[1]+b[1])/2;
   let L=Math.hypot(e.end[0]-e.start[0],e.end[1]-e.start[1]);
   let A=Math.atan2(e.end[1]-e.start[1],e.end[0]-e.start[0])*180/Math.PI;
   dimText(g,mx+8,my-8,`${L.toFixed(1)} mm`,ref,"length_mm",L);
   dimText(g,mx+8,my+13,`${A.toFixed(1)}°`,ref,"angle_deg",A)
 }else if(e.type==="ARC"){
   let c=cv(e.center,T),mid=[(e.start[0]+e.end[0])/2,(e.start[1]+e.end[1])/2],q=cv(mid,T);
   dimText(g,q[0]+8,q[1]-8,`R${e.radius.toFixed(1)}`,ref,"radius_mm",e.radius)
 }else if(e.type==="CIRCLE"){
   let c=cv(e.center,T);dimText(g,c[0]+e.radius*T.s+8,c[1],`Ø${(e.radius*2).toFixed(1)}`,ref,"diameter_mm",e.radius*2)
 }
}
async function editDimension(ref,field,current){
 let names={length_mm:"长度 (mm)",angle_deg:"角度 (°)",radius_mm:"半径 (mm)",diameter_mm:"直径 (mm)"};
 let v=prompt(`修改${names[field]}`,Number(current).toFixed(2));if(v===null)return;
 v=Number(v);if(!Number.isFinite(v))return toast("请输入有效数字");
 S.history.push(JSON.stringify(S.geometry));
 let r=await fetch(`/api/edit_entity/${S.pid}`,{method:"POST",headers:{"Content-Type":"application/json"},
   body:JSON.stringify({group:ref.group,group_index:ref.gi,entity_index:ref.ei,field,value:v})}),j=await r.json();
 if(!j.ok){S.history.pop();return toast(j.error||"修改失败")}
 S.geometry.fitted=j.fitted;S.report=j.report;update();renderCad();toast("尺寸已修改并同步 DXF")
}
function dims(g,T){let b=T.b,a=cv([b.minx,b.miny],T),z=cv([b.maxx,b.maxy],T);let tx=svgEl("text",{x:(a[0]+z[0])/2-45,y:a[1]-25,class:"dim"});tx.textContent=(b.maxx-b.minx).toFixed(1)+" mm";g.appendChild(tx);let ty=svgEl("text",{x:z[0]+18,y:(a[1]+z[1])/2,class:"dim"});ty.textContent=(b.maxy-b.miny).toFixed(1)+" mm";g.appendChild(ty)}
function entityInfo(ref){let e=ref.e,t=e.type;if(t==="LINE"){let a=e.start,b=e.end;t+=`<br>长度：${Math.hypot(b[0]-a[0],b[1]-a[1]).toFixed(2)} mm<br>角度：${(Math.atan2(b[1]-a[1],b[0]-a[0])*180/Math.PI).toFixed(2)}°`}else if(t==="ARC")t+=`<br>半径：${e.radius.toFixed(2)} mm<br>圆心：${e.center.map(x=>x.toFixed(1)).join(", ")}`;else if(t==="CIRCLE")t+=`<br>直径：${(e.radius*2).toFixed(2)} mm`;$("#entityInfo").innerHTML=t+"<br><b>双击图中尺寸标注即可修改</b>"}
function cadMode(m){S.cadMode=m;for(let [id,v] of [["showFitted","fitted"],["showCompare","compare"],["showRaw","raw"],["controlMode","control"],["selectMode","edit"]])$("#"+id).classList.toggle("active",m===v);renderCad()}
$("#showFitted").onclick=()=>cadMode("fitted");$("#showCompare").onclick=()=>cadMode("compare");$("#showRaw").onclick=()=>cadMode("raw");$("#controlMode").onclick=()=>cadMode("control");$("#selectMode").onclick=()=>cadMode("edit");$("#fitBtn").onclick=renderCad;
function svgPointMM(e){let p=$("#cadSvg").createSVGPoint();p.x=e.clientX;p.y=e.clientY;let l=p.matrixTransform($("#cadSvg").getScreenCTM().inverse());return inv(l.x,l.y,S.T)}
function previewControl(ref,handle,target){
 let e=ref.e,t=target;
 if(e.type==="LINE"&&(handle==="start"||handle==="end"))e[handle]=t;
 else if(e.type==="ARC"){
   if(handle==="center"){let d=[t[0]-e.center[0],t[1]-e.center[1]];e.center=t;e.start=[e.start[0]+d[0],e.start[1]+d[1]];e.end=[e.end[0]+d[0],e.end[1]+d[1]]}
   else if(handle==="radius"){let dx=t[0]-e.center[0],dy=t[1]-e.center[1],r=Math.hypot(dx,dy);if(r>0.1){for(let k of ["start","end"]){let vx=e[k][0]-e.center[0],vy=e[k][1]-e.center[1],L=Math.hypot(vx,vy);e[k]=[e.center[0]+vx/L*r,e.center[1]+vy/L*r]}e.radius=r}}
   else if(handle==="start"||handle==="end")e[handle]=t
 }else if(e.type==="CIRCLE"){
   if(handle==="center")e.center=t;else if(handle==="radius")e.radius=Math.max(.1,Math.hypot(t[0]-e.center[0],t[1]-e.center[1]))
 }
}
async function commitControlDrag(d){
 let r=await fetch(`/api/drag_control/${S.pid}`,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({
   group:d.group,group_index:d.gi,entity_index:d.ei,handle:d.handle,target:d.target
 })}),j=await r.json();
 if(!j.ok){S.geometry=JSON.parse(d.before);toast(j.error||"控制点修改失败");renderCad();return}
 S.geometry.fitted=j.fitted;S.report=j.report;update();renderCad();toast("控制点修改已保存并同步 DXF")
}
$("#cadSvg").onpointerdown=e=>{
 if(e.target.classList.contains("ctrlHandle")&&S.cadMode==="control"){
   let before=JSON.stringify(S.geometry),fittedBefore=JSON.parse(JSON.stringify(S.geometry.fitted));
   S.history.push(before);S.ghostFitted=fittedBefore;
   S.drag={kind:"control",group:e.target.dataset.group,gi:+e.target.dataset.gi,ei:+e.target.dataset.ei,handle:e.target.dataset.handle,before,target:null};
   $("#cadSvg").setPointerCapture?.(e.pointerId);return
 }
 if(!e.target.classList.contains("node"))return;
 S.selected=+e.target.dataset.i;S.history.push(JSON.stringify(S.geometry));S.drag={kind:"raw",i:S.selected};renderCad()
};
$("#cadSvg").onpointermove=e=>{
 if(!S.drag)return;
 let q=svgPointMM(e);
 if(S.drag.kind==="control"){
   S.drag.target=q;let arr=S.drag.group==="outer"?S.geometry.fitted.outer:S.geometry.fitted.holes[S.drag.gi],ref={e:arr[S.drag.ei],group:S.drag.group,gi:S.drag.gi,ei:S.drag.ei};
   previewControl(ref,S.drag.handle,q);renderCad()
 }else{S.geometry.outer[S.drag.i]=q;renderCad()}
};
$("#cadSvg").onpointerup=async e=>{if(!S.drag)return;let d=S.drag;S.drag=null;if(d.kind==="control"&&d.target)await commitControlDrag(d)};
$("#undoBtn").onclick=async()=>{let p=S.history.pop();if(!p)return;S.geometry=JSON.parse(p);S.ghostFitted=null;renderCad();if(S.pid&&S.geometry.fitted){await fetch(`/api/save_fitted/${S.pid}`,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({fitted:S.geometry.fitted})}).catch(()=>{})}};
$("#deleteNode").onclick=()=>{if(S.selected==null)return toast("先进入原始节点模式并选择节点");if(S.geometry.outer.length<=3)return;S.history.push(JSON.stringify(S.geometry));S.geometry.outer.splice(S.selected,1);S.selected=null;renderCad()};
$("#refToggle").onchange=renderCad;$("#refSource").onchange=renderCad;$("#refOpacity").oninput=e=>{$("#refOpacityVal").textContent=e.target.value+"%";renderCad()};$("#ghostToggle").onchange=renderCad;
$("#refitBtn").onclick=async()=>{let tol=+$("#fitTolerance").value;status("重新拟合 CAD…");let r=await fetch(`/api/refit/${S.pid}`,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({tolerance_mm:tol,regularize:$("#regularizeToggle").checked})}),j=await r.json();if(j.ok){S.geometry.fitted=j.fitted;S.report=j.report;update();renderCad();status("CAD 已重新拟合");toast("拟合完成")}};


document.querySelectorAll(".profile").forEach(b=>b.addEventListener("click",async()=>{
  document.querySelectorAll(".profile").forEach(x=>x.classList.remove("active"));
  b.classList.add("active");
  $("#fitTolerance").value=b.dataset.tol;
  if(S.pid) $("#refitBtn").click();
}));
$("#fitTolerance").addEventListener("input",()=>{
  document.querySelectorAll(".profile").forEach(x=>x.classList.remove("active"));
});

$("#regularizeToggle").addEventListener("change",()=>{
  if(S.pid) $("#refitBtn").click();
});
