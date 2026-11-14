var hm=Object.defineProperty;var um=(n,t,e)=>t in n?hm(n,t,{enumerable:!0,configurable:!0,writable:!0,value:e}):n[t]=e;var st=(n,t,e)=>um(n,typeof t!="symbol"?t+"":t,e);(function(){const t=document.createElement("link").relList;if(t&&t.supports&&t.supports("modulepreload"))return;for(const s of document.querySelectorAll('link[rel="modulepreload"]'))i(s);new MutationObserver(s=>{for(const r of s)if(r.type==="childList")for(const o of r.addedNodes)o.tagName==="LINK"&&o.rel==="modulepreload"&&i(o)}).observe(document,{childList:!0,subtree:!0});function e(s){const r={};return s.integrity&&(r.integrity=s.integrity),s.referrerPolicy&&(r.referrerPolicy=s.referrerPolicy),s.crossOrigin==="use-credentials"?r.credentials="include":s.crossOrigin==="anonymous"?r.credentials="omit":r.credentials="same-origin",r}function i(s){if(s.ep)return;s.ep=!0;const r=e(s);fetch(s.href,r)}})();const of=[{key:"start",name:"시작",desc:"Home 자세에서 시작 (집게 열림)",kind:"joint"},{key:"approach_a",name:"접근",desc:"A 위치 위(접근 높이)로 이동",kind:"joint"},{key:"descend_a",name:"하강",desc:"A 위치로 내려감",kind:"linear"},{key:"grip",name:"집기",desc:"제자리에서 집게를 닫아 물체를 잡음",kind:"gripper"},{key:"lift_a",name:"상승",desc:"물체를 들고 A 위로 올라감",kind:"linear"},{key:"move",name:"이동",desc:"B 위치 위로 이동",kind:"joint"},{key:"descend_b",name:"하강",desc:"B 위치로 내려감",kind:"linear"},{key:"release",name:"놓기",desc:"제자리에서 집게를 열어 물체를 놓음",kind:"gripper"},{key:"lift_b",name:"상승",desc:"B 위로 올라감",kind:"linear"},{key:"home",name:"복귀",desc:"Home 자세로 돌아감",kind:"joint"}];function ha(){return{links:{base_height:70,link1_length:80,link2_length:80,gripper_length:60},joints:[{name:"J1 Base",offset:90,direction:1},{name:"J2 Shoulder",offset:0,direction:1},{name:"J3 Elbow",offset:180,direction:1}],gripper:{open_angle:30,close_angle:90,max_gap_mm:40},home_angles:[90,90,90,30],max_speed_deg_s:120,pick_place:{pick:[130,60,10],place:[130,-60,10],approach_height:40,cartesian:!0,sample_ms:20,prefer:"auto",durations:[0,1500,800,500,800,1500,800,500,800,1500]}}}const af=["J1 Base","J2 Shoulder","J3 Elbow","J4 Gripper"],xh=["#e4572e","#29b6f6","#66bb6a","#ab47bc"],lf="robot_arm_4dof_settings_v1";function cf(n){try{localStorage.setItem(lf,JSON.stringify(n))}catch{}}function dm(){try{const n=localStorage.getItem(lf);return n?hf(ha(),JSON.parse(n)):null}catch{return null}}function Bn(n,t,e=-1/0,i=1/0){if(n===null||n===""||typeof n=="boolean")return t;const s=Number(n);return Number.isFinite(s)?Math.min(i,Math.max(e,s)):t}function Hr(n,t,e=-1/0,i=1/0){return!Array.isArray(n)||n.length!==t.length?t.slice():n.map((s,r)=>Bn(s,t[r],e,i))}const Xs=n=>!!n&&typeof n=="object"&&!Array.isArray(n);function hf(n,t){const e=structuredClone(n);if(!Xs(t))return e;if(Xs(t.links))for(const i of Object.keys(e.links))e.links[i]=Bn(t.links[i],e.links[i],1);if(Array.isArray(t.joints)&&t.joints.slice(0,3).forEach((i,s)=>{Xs(i)&&(e.joints[s].offset=Bn(i.offset,e.joints[s].offset),(Number(i.direction)===1||Number(i.direction)===-1)&&(e.joints[s].direction=Number(i.direction)))}),Xs(t.gripper)&&(e.gripper.open_angle=Bn(t.gripper.open_angle,e.gripper.open_angle,0,180),e.gripper.close_angle=Bn(t.gripper.close_angle,e.gripper.close_angle,0,180),e.gripper.max_gap_mm=Bn(t.gripper.max_gap_mm,e.gripper.max_gap_mm,1)),e.home_angles=Hr(t.home_angles,e.home_angles,0,180),e.max_speed_deg_s=Bn(t.max_speed_deg_s,e.max_speed_deg_s,10),Xs(t.pick_place)){const i=e.pick_place,s=t.pick_place;i.pick=Hr(s.pick,i.pick),i.place=Hr(s.place,i.place),i.approach_height=Bn(s.approach_height,i.approach_height),i.sample_ms=Bn(s.sample_ms,i.sample_ms,5),typeof s.cartesian=="boolean"&&(i.cartesian=s.cartesian),["auto","up","down"].includes(s.prefer)&&(i.prefer=s.prefer),i.durations=Hr(s.durations,i.durations,0)}return e}const uf=Math.PI/180,Pc=0,Dc=180,vh=.01,rr=n=>n*uf,Io=n=>n/uf;function Aa(n){let t=n%360;return t>180&&(t-=360),t<=-180&&(t+=360),t}function fm(n,t){return(n-t.offset)*t.direction}function pm(n,t){return n*t.direction+t.offset}function mm(n,t){const e=pm(n,t),i=[e,e-360,e+360],s=i.find(br);if(s!==void 0)return s;const r=(Pc+Dc)/2;return i.reduce((o,a)=>Math.abs(a-r)<Math.abs(o-r)?a:o)}function br(n){return n>=Pc-vh&&n<=Dc+vh}function Vi(n){return Math.min(Dc,Math.max(Pc,n))}function df(n,t){return[0,1,2].map(e=>fm(t[e],n.joints[e]))}function gm(n,t){return[{index:1,name:"Base",theta:t[0],d:n.base_height,a:0,alpha:90},{index:2,name:"Shoulder",theta:t[1],d:0,a:n.link1_length,alpha:0},{index:3,name:"Elbow",theta:t[2],d:0,a:n.link2_length+n.gripper_length,alpha:0}]}function _m(n,t,e,i){const s=Math.cos(rr(n)),r=Math.sin(rr(n)),o=Math.cos(rr(i)),a=Math.sin(rr(i));return[[s,-r*o,r*a,e*s],[r,s*o,-s*a,e*r],[0,a,o,t],[0,0,0,1]]}function xm(n,t){const e=[[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]];for(let i=0;i<4;i++)for(let s=0;s<4;s++){let r=0;for(let o=0;o<4;o++)r+=n[i][o]*t[o][s];e[i][s]=r}return e}const bh=()=>[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],yh=n=>[n[0][3],n[1][3],n[2][3]],vm=n=>[n[0][0],n[1][0],n[2][0]];function ff(n,t){const e=gm(n,t),i=[bh()];let s=bh();for(const g of e)s=xm(s,_m(g.theta,g.d,g.a,g.alpha)),i.push(s);const r=yh(i[3]),o=vm(i[3]),a=r.map((g,_)=>g-o[_]*n.gripper_length),[l,c,h]=t.map(rr),u=n.link2_length+n.gripper_length,d=n.link1_length*Math.cos(c)+u*Math.cos(c+h),f=n.base_height+n.link1_length*Math.sin(c)+u*Math.sin(c+h);return{dh_table:e,frames:i,joint_positions:i.map(yh),gripper_base:a,tcp:r,tool_axis:o,tool_pitch:Io(c+h),closed_form:{r:d,z:f,x:d*Math.cos(l),y:d*Math.sin(l),link23:u}}}function Pr(n,t){const e=df(n,t);return{joint_deg:e,...ff(n.links,e)}}function bm(n,t){const[e,i,s]=t,r=n.link1_length,o=n.link2_length+n.gripper_length,a=Math.hypot(e,i),l=s-n.base_height,c=a<1e-9?0:Io(Math.atan2(i,e)),h=Math.hypot(a,l),u=(a*a+l*l-r*r-o*o)/(2*r*o),d={x:e,y:i,z:s,rho:a,s:l,dist:h,d_value:u,theta1_front:c,l1:r,l23:o,reach_max:r+o,reach_min:Math.abs(r-o)};if(u>1+1e-9)return{ok:!1,reason:`도달 거리 초과: 어깨에서 목표까지 ${h.toFixed(1)}mm > 최대 ${(r+o).toFixed(1)}mm`,candidates:[],steps:d};if(u<-1-1e-9)return{ok:!1,reason:`너무 가까움: 어깨에서 목표까지 ${h.toFixed(1)}mm < 최소 ${Math.abs(r-o).toFixed(1)}mm`,candidates:[],steps:d};const f=Math.max(-1,Math.min(1,u)),g=Math.sqrt(1-f*f),_=[],m=[{base:"front",theta1:c,r:a},{base:"back",theta1:Aa(c+180),r:-a}];for(const p of m)for(const v of[-1,1]){const y=Math.atan2(v*g,f),x=Math.atan2(l,p.r)-Math.atan2(o*Math.sin(y),r+o*Math.cos(y)),A=(p.r>=0?v<0:v>0)?"up":"down";_.push({base:p.base,elbow:A,r:p.r,joint:[p.theta1,Aa(Io(x)),Aa(Io(y))]})}return{ok:!0,reason:"",candidates:_,steps:d}}function Ko(n,t,e={}){const i=e.prefer||"auto",s=bm(n.links,t);if(!s.ok)return{ok:!1,reason:s.reason,steps:s.steps,candidates:[]};const r=s.candidates.map(h=>{const u=h.joint.map((_,m)=>mm(_,n.joints[m])),d=u.map((_,m)=>br(_)?null:m).filter(_=>_!==null),f=ff(n.links,h.joint),g=Math.hypot(f.tcp[0]-t[0],f.tcp[1]-t[1],f.tcp[2]-t[2]);return{...h,servo:u.map(_=>br(_)?Vi(_):_),in_range:d.length===0,out_of_range:d,error_mm:g,tool_pitch:f.tool_pitch}});let o=r.filter(h=>h.in_range);if(o.length===0){const h=["J1(Base)","J2(Shoulder)","J3(Elbow)"];return{ok:!1,reason:`서보 범위(0~180°) 초과: ${r.map(d=>`${d.base==="front"?"정면":"뒤쪽"}/${d.elbow==="up"?"팔꿈치 위":"팔꿈치 아래"}: ${d.out_of_range.map(f=>`${h[f]}=${d.servo[f].toFixed(1)}°`).join(", ")}`).join(" | ")}`,steps:s.steps,candidates:r}}if(i==="up"||i==="down"){const h=o.filter(u=>u.elbow===i);h.length>0&&(o=h)}const a=e.reference_servo,l=h=>a?h.servo.reduce((u,d,f)=>u+Math.abs(d-a[f]),0):(h.base==="front"?0:1e3)+(h.elbow==="up"?0:100);o.sort((h,u)=>l(h)-l(u));const c=o[0];return{ok:!0,reason:"",servo:c.servo,joint:c.joint,chosen:c,steps:s.steps,candidates:r}}const pf=n=>3*n*n-2*n*n*n,mf=(n,t,e)=>n+(t-n)*e,yl=(n,t,e)=>n.map((i,s)=>mf(i,t[s],e));function gf(n,t){const e=[],i=Math.max(1,Math.ceil(n/t));for(let s=1;s<=i;s++)e.push(Math.min(n,s*t));return e}function ym(n,t){const e=t||n.pick_place,i=[],s=[],r=Math.max(5,Number(e.sample_ms)||20),o=n.gripper.open_angle,a=n.gripper.close_angle,l=p=>[p[0],p[1],p[2]+Number(e.approach_height)],c=[{pos:null,grip:o},{pos:l(e.pick),grip:o},{pos:e.pick,grip:o},{pos:e.pick,grip:a},{pos:l(e.pick),grip:a},{pos:l(e.place),grip:a},{pos:e.place,grip:a},{pos:e.place,grip:o},{pos:l(e.place),grip:o},{pos:null,grip:o}],h=[];let u=n.home_angles.slice(0,3);if(c.forEach((p,v)=>{const y=of[v];if(p.pos===null){const w=[...n.home_angles.slice(0,3),p.grip];h.push({...y,index:v,pos:null,servo:w,ik:null}),u=w.slice(0,3);return}const x=Ko(n,p.pos,{prefer:e.prefer,reference_servo:u});if(!x.ok){i.push(`[${v}. ${y.name}] 목표 (${p.pos.map(w=>Number(w).toFixed(0)).join(", ")}) → ${x.reason}`),h.push({...y,index:v,pos:p.pos,servo:[...u,p.grip],ik:x});return}const A=[...x.servo,p.grip];h.push({...y,index:v,pos:p.pos.map(Number),servo:A,ik:x}),u=x.servo}),i.length>0)return{ok:!1,errors:i,warnings:s,samples:[],steps:[],waypoints:h,total_ms:0,sample_ms:r};const d=[],f=[],g=(p,v,y)=>{const x=Pr(n,y);d.push({time_ms:p,step_index:v,servo:y,tcp:x.tcp,holding:v>=4&&v<=6})};g(0,0,h[0].servo.slice()),f.push({...h[0],start_ms:0,end_ms:0,duration_ms:0});let _=0;for(let p=1;p<h.length;p++){const v=h[p-1],y=h[p],x=Math.max(r,Number(e.durations[p])||0),A=e.cartesian&&y.kind==="linear"&&v.pos&&y.pos;let w=v.servo.slice(0,3),T=!1;for(const R of gf(x,r)){const S=pf(R/x),b=mf(v.servo[3],y.servo[3],S);let P;if(A){const N=yl(v.pos,y.pos,S),F=Ko(n,N,{prefer:e.prefer,reference_servo:w});F.ok?P=[...F.servo,b]:(T||i.push(`[${p}. ${y.name}] t=${(_+R).toFixed(0)}ms 부터 직선 경로 IK 실패 → ${F.reason}`),T=!0,P=[...w,b])}else P=yl(v.servo,y.servo,S);w=P.slice(0,3),g(_+R,p,P)}f.push({...y,start_ms:_,end_ms:_+x,duration_ms:x}),_+=x}let m=0;for(const p of d){if(!p.servo.every(br)){i.push(`t=${p.time_ms.toFixed(0)}ms 서보 범위 초과: ${p.servo.map(v=>v.toFixed(1)).join(", ")}`);break}p.tcp[2]<-.5&&m++}return m>0&&s.push(`집게 끝이 바닥(z<0) 아래로 내려가는 샘플이 ${m}개 있습니다. 경로나 높이를 확인하세요.`),{ok:i.length===0,errors:i,warnings:s,samples:d,steps:f,waypoints:h,total_ms:_,sample_ms:r,kind:"pick_place"}}function Mm(n,t,e,i=20){const s=[],r=[],o=[],a=(h,u,d)=>({time_ms:h,step_index:u,servo:d,tcp:Pr(n,d).tcp,holding:!1});s.push(a(0,0,t.slice())),r.push({index:0,name:"시작",desc:"현재 자세",servo:t.slice(),pos:null,start_ms:0,end_ms:0,duration_ms:0});let l=0,c=t.slice();return e.forEach((h,u)=>{h.servo.every(br)||o.push(`키프레임 ${u+1}: 서보 범위 초과`);const d=Math.max(i,Number(h.duration_ms)||1e3);for(const f of gf(d,i))s.push(a(l+f,u+1,yl(c,h.servo,pf(f/d))));r.push({index:u+1,name:h.name||`키프레임 ${u+1}`,desc:"키프레임으로 이동",servo:h.servo.slice(),pos:null,start_ms:l,end_ms:l+d,duration_ms:d}),l+=d,c=h.servo.slice()}),{ok:o.length===0,errors:o,warnings:[],samples:s,steps:r,total_ms:l,sample_ms:i,kind:"keyframe"}}function Sm(n){const t="time_ms,step,step_name,servo1,servo2,servo3,servo4,tcp_x,tcp_y,tcp_z,holding",e=n.samples.map(i=>{const s=n.steps[i.step_index];return[i.time_ms.toFixed(0),i.step_index,s?s.name:"",...i.servo.map(r=>r.toFixed(2)),...i.tcp.map(r=>r.toFixed(2)),i.holding?1:0].join(",")});return[t,...e].join(`
`)}/**
 * @license
 * Copyright 2010-2024 Three.js Authors
 * SPDX-License-Identifier: MIT
 */const Lc="170",Ts={ROTATE:0,DOLLY:1,PAN:2},ys={ROTATE:0,PAN:1,DOLLY_PAN:2,DOLLY_ROTATE:3},Em=0,Mh=1,Tm=2,_f=1,wm=2,zn=3,_i=0,qe=1,Tn=2,fi=0,ws=1,Sh=2,Eh=3,Th=4,Am=5,Fi=100,Cm=101,Rm=102,Pm=103,Dm=104,Lm=200,Im=201,Om=202,Um=203,Ml=204,Sl=205,Nm=206,Fm=207,km=208,Bm=209,zm=210,Hm=211,Vm=212,Gm=213,Wm=214,El=0,Tl=1,wl=2,Ds=3,Al=4,Cl=5,Rl=6,Pl=7,xf=0,Xm=1,jm=2,pi=0,$m=1,Ym=2,qm=3,Km=4,Zm=5,Jm=6,Qm=7,vf=300,Ls=301,Is=302,Dl=303,Ll=304,ua=306,Il=1e3,Bi=1001,Ol=1002,xn=1003,tg=1004,Vr=1005,Cn=1006,Ca=1007,zi=1008,Jn=1009,bf=1010,yf=1011,yr=1012,Ic=1013,Xi=1014,jn=1015,Dr=1016,Oc=1017,Uc=1018,Os=1020,Mf=35902,Sf=1021,Ef=1022,mn=1023,Tf=1024,wf=1025,As=1026,Us=1027,Af=1028,Nc=1029,Cf=1030,Fc=1031,kc=1033,Oo=33776,Uo=33777,No=33778,Fo=33779,Ul=35840,Nl=35841,Fl=35842,kl=35843,Bl=36196,zl=37492,Hl=37496,Vl=37808,Gl=37809,Wl=37810,Xl=37811,jl=37812,$l=37813,Yl=37814,ql=37815,Kl=37816,Zl=37817,Jl=37818,Ql=37819,tc=37820,ec=37821,ko=36492,nc=36494,ic=36495,Rf=36283,sc=36284,rc=36285,oc=36286,eg=3200,ng=3201,Pf=0,ig=1,li="",an="srgb",Hs="srgb-linear",da="linear",ie="srgb",ts=7680,wh=519,sg=512,rg=513,og=514,Df=515,ag=516,lg=517,cg=518,hg=519,ac=35044,Ah="300 es",$n=2e3,Zo=2001;class Ki{addEventListener(t,e){this._listeners===void 0&&(this._listeners={});const i=this._listeners;i[t]===void 0&&(i[t]=[]),i[t].indexOf(e)===-1&&i[t].push(e)}hasEventListener(t,e){if(this._listeners===void 0)return!1;const i=this._listeners;return i[t]!==void 0&&i[t].indexOf(e)!==-1}removeEventListener(t,e){if(this._listeners===void 0)return;const s=this._listeners[t];if(s!==void 0){const r=s.indexOf(e);r!==-1&&s.splice(r,1)}}dispatchEvent(t){if(this._listeners===void 0)return;const i=this._listeners[t.type];if(i!==void 0){t.target=this;const s=i.slice(0);for(let r=0,o=s.length;r<o;r++)s[r].call(this,t);t.target=null}}}const Ne=["00","01","02","03","04","05","06","07","08","09","0a","0b","0c","0d","0e","0f","10","11","12","13","14","15","16","17","18","19","1a","1b","1c","1d","1e","1f","20","21","22","23","24","25","26","27","28","29","2a","2b","2c","2d","2e","2f","30","31","32","33","34","35","36","37","38","39","3a","3b","3c","3d","3e","3f","40","41","42","43","44","45","46","47","48","49","4a","4b","4c","4d","4e","4f","50","51","52","53","54","55","56","57","58","59","5a","5b","5c","5d","5e","5f","60","61","62","63","64","65","66","67","68","69","6a","6b","6c","6d","6e","6f","70","71","72","73","74","75","76","77","78","79","7a","7b","7c","7d","7e","7f","80","81","82","83","84","85","86","87","88","89","8a","8b","8c","8d","8e","8f","90","91","92","93","94","95","96","97","98","99","9a","9b","9c","9d","9e","9f","a0","a1","a2","a3","a4","a5","a6","a7","a8","a9","aa","ab","ac","ad","ae","af","b0","b1","b2","b3","b4","b5","b6","b7","b8","b9","ba","bb","bc","bd","be","bf","c0","c1","c2","c3","c4","c5","c6","c7","c8","c9","ca","cb","cc","cd","ce","cf","d0","d1","d2","d3","d4","d5","d6","d7","d8","d9","da","db","dc","dd","de","df","e0","e1","e2","e3","e4","e5","e6","e7","e8","e9","ea","eb","ec","ed","ee","ef","f0","f1","f2","f3","f4","f5","f6","f7","f8","f9","fa","fb","fc","fd","fe","ff"],Bo=Math.PI/180,lc=180/Math.PI;function mi(){const n=Math.random()*4294967295|0,t=Math.random()*4294967295|0,e=Math.random()*4294967295|0,i=Math.random()*4294967295|0;return(Ne[n&255]+Ne[n>>8&255]+Ne[n>>16&255]+Ne[n>>24&255]+"-"+Ne[t&255]+Ne[t>>8&255]+"-"+Ne[t>>16&15|64]+Ne[t>>24&255]+"-"+Ne[e&63|128]+Ne[e>>8&255]+"-"+Ne[e>>16&255]+Ne[e>>24&255]+Ne[i&255]+Ne[i>>8&255]+Ne[i>>16&255]+Ne[i>>24&255]).toLowerCase()}function Ge(n,t,e){return Math.max(t,Math.min(e,n))}function ug(n,t){return(n%t+t)%t}function Ra(n,t,e){return(1-e)*n+e*t}function wn(n,t){switch(t.constructor){case Float32Array:return n;case Uint32Array:return n/4294967295;case Uint16Array:return n/65535;case Uint8Array:return n/255;case Int32Array:return Math.max(n/2147483647,-1);case Int16Array:return Math.max(n/32767,-1);case Int8Array:return Math.max(n/127,-1);default:throw new Error("Invalid component type.")}}function se(n,t){switch(t.constructor){case Float32Array:return n;case Uint32Array:return Math.round(n*4294967295);case Uint16Array:return Math.round(n*65535);case Uint8Array:return Math.round(n*255);case Int32Array:return Math.round(n*2147483647);case Int16Array:return Math.round(n*32767);case Int8Array:return Math.round(n*127);default:throw new Error("Invalid component type.")}}const dg={DEG2RAD:Bo};class Tt{constructor(t=0,e=0){Tt.prototype.isVector2=!0,this.x=t,this.y=e}get width(){return this.x}set width(t){this.x=t}get height(){return this.y}set height(t){this.y=t}set(t,e){return this.x=t,this.y=e,this}setScalar(t){return this.x=t,this.y=t,this}setX(t){return this.x=t,this}setY(t){return this.y=t,this}setComponent(t,e){switch(t){case 0:this.x=e;break;case 1:this.y=e;break;default:throw new Error("index is out of range: "+t)}return this}getComponent(t){switch(t){case 0:return this.x;case 1:return this.y;default:throw new Error("index is out of range: "+t)}}clone(){return new this.constructor(this.x,this.y)}copy(t){return this.x=t.x,this.y=t.y,this}add(t){return this.x+=t.x,this.y+=t.y,this}addScalar(t){return this.x+=t,this.y+=t,this}addVectors(t,e){return this.x=t.x+e.x,this.y=t.y+e.y,this}addScaledVector(t,e){return this.x+=t.x*e,this.y+=t.y*e,this}sub(t){return this.x-=t.x,this.y-=t.y,this}subScalar(t){return this.x-=t,this.y-=t,this}subVectors(t,e){return this.x=t.x-e.x,this.y=t.y-e.y,this}multiply(t){return this.x*=t.x,this.y*=t.y,this}multiplyScalar(t){return this.x*=t,this.y*=t,this}divide(t){return this.x/=t.x,this.y/=t.y,this}divideScalar(t){return this.multiplyScalar(1/t)}applyMatrix3(t){const e=this.x,i=this.y,s=t.elements;return this.x=s[0]*e+s[3]*i+s[6],this.y=s[1]*e+s[4]*i+s[7],this}min(t){return this.x=Math.min(this.x,t.x),this.y=Math.min(this.y,t.y),this}max(t){return this.x=Math.max(this.x,t.x),this.y=Math.max(this.y,t.y),this}clamp(t,e){return this.x=Math.max(t.x,Math.min(e.x,this.x)),this.y=Math.max(t.y,Math.min(e.y,this.y)),this}clampScalar(t,e){return this.x=Math.max(t,Math.min(e,this.x)),this.y=Math.max(t,Math.min(e,this.y)),this}clampLength(t,e){const i=this.length();return this.divideScalar(i||1).multiplyScalar(Math.max(t,Math.min(e,i)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this}negate(){return this.x=-this.x,this.y=-this.y,this}dot(t){return this.x*t.x+this.y*t.y}cross(t){return this.x*t.y-this.y*t.x}lengthSq(){return this.x*this.x+this.y*this.y}length(){return Math.sqrt(this.x*this.x+this.y*this.y)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)}normalize(){return this.divideScalar(this.length()||1)}angle(){return Math.atan2(-this.y,-this.x)+Math.PI}angleTo(t){const e=Math.sqrt(this.lengthSq()*t.lengthSq());if(e===0)return Math.PI/2;const i=this.dot(t)/e;return Math.acos(Ge(i,-1,1))}distanceTo(t){return Math.sqrt(this.distanceToSquared(t))}distanceToSquared(t){const e=this.x-t.x,i=this.y-t.y;return e*e+i*i}manhattanDistanceTo(t){return Math.abs(this.x-t.x)+Math.abs(this.y-t.y)}setLength(t){return this.normalize().multiplyScalar(t)}lerp(t,e){return this.x+=(t.x-this.x)*e,this.y+=(t.y-this.y)*e,this}lerpVectors(t,e,i){return this.x=t.x+(e.x-t.x)*i,this.y=t.y+(e.y-t.y)*i,this}equals(t){return t.x===this.x&&t.y===this.y}fromArray(t,e=0){return this.x=t[e],this.y=t[e+1],this}toArray(t=[],e=0){return t[e]=this.x,t[e+1]=this.y,t}fromBufferAttribute(t,e){return this.x=t.getX(e),this.y=t.getY(e),this}rotateAround(t,e){const i=Math.cos(e),s=Math.sin(e),r=this.x-t.x,o=this.y-t.y;return this.x=r*i-o*s+t.x,this.y=r*s+o*i+t.y,this}random(){return this.x=Math.random(),this.y=Math.random(),this}*[Symbol.iterator](){yield this.x,yield this.y}}class Ft{constructor(t,e,i,s,r,o,a,l,c){Ft.prototype.isMatrix3=!0,this.elements=[1,0,0,0,1,0,0,0,1],t!==void 0&&this.set(t,e,i,s,r,o,a,l,c)}set(t,e,i,s,r,o,a,l,c){const h=this.elements;return h[0]=t,h[1]=s,h[2]=a,h[3]=e,h[4]=r,h[5]=l,h[6]=i,h[7]=o,h[8]=c,this}identity(){return this.set(1,0,0,0,1,0,0,0,1),this}copy(t){const e=this.elements,i=t.elements;return e[0]=i[0],e[1]=i[1],e[2]=i[2],e[3]=i[3],e[4]=i[4],e[5]=i[5],e[6]=i[6],e[7]=i[7],e[8]=i[8],this}extractBasis(t,e,i){return t.setFromMatrix3Column(this,0),e.setFromMatrix3Column(this,1),i.setFromMatrix3Column(this,2),this}setFromMatrix4(t){const e=t.elements;return this.set(e[0],e[4],e[8],e[1],e[5],e[9],e[2],e[6],e[10]),this}multiply(t){return this.multiplyMatrices(this,t)}premultiply(t){return this.multiplyMatrices(t,this)}multiplyMatrices(t,e){const i=t.elements,s=e.elements,r=this.elements,o=i[0],a=i[3],l=i[6],c=i[1],h=i[4],u=i[7],d=i[2],f=i[5],g=i[8],_=s[0],m=s[3],p=s[6],v=s[1],y=s[4],x=s[7],A=s[2],w=s[5],T=s[8];return r[0]=o*_+a*v+l*A,r[3]=o*m+a*y+l*w,r[6]=o*p+a*x+l*T,r[1]=c*_+h*v+u*A,r[4]=c*m+h*y+u*w,r[7]=c*p+h*x+u*T,r[2]=d*_+f*v+g*A,r[5]=d*m+f*y+g*w,r[8]=d*p+f*x+g*T,this}multiplyScalar(t){const e=this.elements;return e[0]*=t,e[3]*=t,e[6]*=t,e[1]*=t,e[4]*=t,e[7]*=t,e[2]*=t,e[5]*=t,e[8]*=t,this}determinant(){const t=this.elements,e=t[0],i=t[1],s=t[2],r=t[3],o=t[4],a=t[5],l=t[6],c=t[7],h=t[8];return e*o*h-e*a*c-i*r*h+i*a*l+s*r*c-s*o*l}invert(){const t=this.elements,e=t[0],i=t[1],s=t[2],r=t[3],o=t[4],a=t[5],l=t[6],c=t[7],h=t[8],u=h*o-a*c,d=a*l-h*r,f=c*r-o*l,g=e*u+i*d+s*f;if(g===0)return this.set(0,0,0,0,0,0,0,0,0);const _=1/g;return t[0]=u*_,t[1]=(s*c-h*i)*_,t[2]=(a*i-s*o)*_,t[3]=d*_,t[4]=(h*e-s*l)*_,t[5]=(s*r-a*e)*_,t[6]=f*_,t[7]=(i*l-c*e)*_,t[8]=(o*e-i*r)*_,this}transpose(){let t;const e=this.elements;return t=e[1],e[1]=e[3],e[3]=t,t=e[2],e[2]=e[6],e[6]=t,t=e[5],e[5]=e[7],e[7]=t,this}getNormalMatrix(t){return this.setFromMatrix4(t).invert().transpose()}transposeIntoArray(t){const e=this.elements;return t[0]=e[0],t[1]=e[3],t[2]=e[6],t[3]=e[1],t[4]=e[4],t[5]=e[7],t[6]=e[2],t[7]=e[5],t[8]=e[8],this}setUvTransform(t,e,i,s,r,o,a){const l=Math.cos(r),c=Math.sin(r);return this.set(i*l,i*c,-i*(l*o+c*a)+o+t,-s*c,s*l,-s*(-c*o+l*a)+a+e,0,0,1),this}scale(t,e){return this.premultiply(Pa.makeScale(t,e)),this}rotate(t){return this.premultiply(Pa.makeRotation(-t)),this}translate(t,e){return this.premultiply(Pa.makeTranslation(t,e)),this}makeTranslation(t,e){return t.isVector2?this.set(1,0,t.x,0,1,t.y,0,0,1):this.set(1,0,t,0,1,e,0,0,1),this}makeRotation(t){const e=Math.cos(t),i=Math.sin(t);return this.set(e,-i,0,i,e,0,0,0,1),this}makeScale(t,e){return this.set(t,0,0,0,e,0,0,0,1),this}equals(t){const e=this.elements,i=t.elements;for(let s=0;s<9;s++)if(e[s]!==i[s])return!1;return!0}fromArray(t,e=0){for(let i=0;i<9;i++)this.elements[i]=t[i+e];return this}toArray(t=[],e=0){const i=this.elements;return t[e]=i[0],t[e+1]=i[1],t[e+2]=i[2],t[e+3]=i[3],t[e+4]=i[4],t[e+5]=i[5],t[e+6]=i[6],t[e+7]=i[7],t[e+8]=i[8],t}clone(){return new this.constructor().fromArray(this.elements)}}const Pa=new Ft;function Lf(n){for(let t=n.length-1;t>=0;--t)if(n[t]>=65535)return!0;return!1}function Jo(n){return document.createElementNS("http://www.w3.org/1999/xhtml",n)}function fg(){const n=Jo("canvas");return n.style.display="block",n}const Ch={};function or(n){n in Ch||(Ch[n]=!0,console.warn(n))}function pg(n,t,e){return new Promise(function(i,s){function r(){switch(n.clientWaitSync(t,n.SYNC_FLUSH_COMMANDS_BIT,0)){case n.WAIT_FAILED:s();break;case n.TIMEOUT_EXPIRED:setTimeout(r,e);break;default:i()}}setTimeout(r,e)})}function mg(n){const t=n.elements;t[2]=.5*t[2]+.5*t[3],t[6]=.5*t[6]+.5*t[7],t[10]=.5*t[10]+.5*t[11],t[14]=.5*t[14]+.5*t[15]}function gg(n){const t=n.elements;t[11]===-1?(t[10]=-t[10]-1,t[14]=-t[14]):(t[10]=-t[10],t[14]=-t[14]+1)}const Kt={enabled:!0,workingColorSpace:Hs,spaces:{},convert:function(n,t,e){return this.enabled===!1||t===e||!t||!e||(this.spaces[t].transfer===ie&&(n.r=Zn(n.r),n.g=Zn(n.g),n.b=Zn(n.b)),this.spaces[t].primaries!==this.spaces[e].primaries&&(n.applyMatrix3(this.spaces[t].toXYZ),n.applyMatrix3(this.spaces[e].fromXYZ)),this.spaces[e].transfer===ie&&(n.r=Cs(n.r),n.g=Cs(n.g),n.b=Cs(n.b))),n},fromWorkingColorSpace:function(n,t){return this.convert(n,this.workingColorSpace,t)},toWorkingColorSpace:function(n,t){return this.convert(n,t,this.workingColorSpace)},getPrimaries:function(n){return this.spaces[n].primaries},getTransfer:function(n){return n===li?da:this.spaces[n].transfer},getLuminanceCoefficients:function(n,t=this.workingColorSpace){return n.fromArray(this.spaces[t].luminanceCoefficients)},define:function(n){Object.assign(this.spaces,n)},_getMatrix:function(n,t,e){return n.copy(this.spaces[t].toXYZ).multiply(this.spaces[e].fromXYZ)},_getDrawingBufferColorSpace:function(n){return this.spaces[n].outputColorSpaceConfig.drawingBufferColorSpace},_getUnpackColorSpace:function(n=this.workingColorSpace){return this.spaces[n].workingColorSpaceConfig.unpackColorSpace}};function Zn(n){return n<.04045?n*.0773993808:Math.pow(n*.9478672986+.0521327014,2.4)}function Cs(n){return n<.0031308?n*12.92:1.055*Math.pow(n,.41666)-.055}const Rh=[.64,.33,.3,.6,.15,.06],Ph=[.2126,.7152,.0722],Dh=[.3127,.329],Lh=new Ft().set(.4123908,.3575843,.1804808,.212639,.7151687,.0721923,.0193308,.1191948,.9505322),Ih=new Ft().set(3.2409699,-1.5373832,-.4986108,-.9692436,1.8759675,.0415551,.0556301,-.203977,1.0569715);Kt.define({[Hs]:{primaries:Rh,whitePoint:Dh,transfer:da,toXYZ:Lh,fromXYZ:Ih,luminanceCoefficients:Ph,workingColorSpaceConfig:{unpackColorSpace:an},outputColorSpaceConfig:{drawingBufferColorSpace:an}},[an]:{primaries:Rh,whitePoint:Dh,transfer:ie,toXYZ:Lh,fromXYZ:Ih,luminanceCoefficients:Ph,outputColorSpaceConfig:{drawingBufferColorSpace:an}}});let es;class _g{static getDataURL(t){if(/^data:/i.test(t.src)||typeof HTMLCanvasElement>"u")return t.src;let e;if(t instanceof HTMLCanvasElement)e=t;else{es===void 0&&(es=Jo("canvas")),es.width=t.width,es.height=t.height;const i=es.getContext("2d");t instanceof ImageData?i.putImageData(t,0,0):i.drawImage(t,0,0,t.width,t.height),e=es}return e.width>2048||e.height>2048?(console.warn("THREE.ImageUtils.getDataURL: Image converted to jpg for performance reasons",t),e.toDataURL("image/jpeg",.6)):e.toDataURL("image/png")}static sRGBToLinear(t){if(typeof HTMLImageElement<"u"&&t instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&t instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&t instanceof ImageBitmap){const e=Jo("canvas");e.width=t.width,e.height=t.height;const i=e.getContext("2d");i.drawImage(t,0,0,t.width,t.height);const s=i.getImageData(0,0,t.width,t.height),r=s.data;for(let o=0;o<r.length;o++)r[o]=Zn(r[o]/255)*255;return i.putImageData(s,0,0),e}else if(t.data){const e=t.data.slice(0);for(let i=0;i<e.length;i++)e instanceof Uint8Array||e instanceof Uint8ClampedArray?e[i]=Math.floor(Zn(e[i]/255)*255):e[i]=Zn(e[i]);return{data:e,width:t.width,height:t.height}}else return console.warn("THREE.ImageUtils.sRGBToLinear(): Unsupported image type. No color space conversion applied."),t}}let xg=0;class If{constructor(t=null){this.isSource=!0,Object.defineProperty(this,"id",{value:xg++}),this.uuid=mi(),this.data=t,this.dataReady=!0,this.version=0}set needsUpdate(t){t===!0&&this.version++}toJSON(t){const e=t===void 0||typeof t=="string";if(!e&&t.images[this.uuid]!==void 0)return t.images[this.uuid];const i={uuid:this.uuid,url:""},s=this.data;if(s!==null){let r;if(Array.isArray(s)){r=[];for(let o=0,a=s.length;o<a;o++)s[o].isDataTexture?r.push(Da(s[o].image)):r.push(Da(s[o]))}else r=Da(s);i.url=r}return e||(t.images[this.uuid]=i),i}}function Da(n){return typeof HTMLImageElement<"u"&&n instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&n instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&n instanceof ImageBitmap?_g.getDataURL(n):n.data?{data:Array.from(n.data),width:n.width,height:n.height,type:n.data.constructor.name}:(console.warn("THREE.Texture: Unable to serialize Texture."),{})}let vg=0;class We extends Ki{constructor(t=We.DEFAULT_IMAGE,e=We.DEFAULT_MAPPING,i=Bi,s=Bi,r=Cn,o=zi,a=mn,l=Jn,c=We.DEFAULT_ANISOTROPY,h=li){super(),this.isTexture=!0,Object.defineProperty(this,"id",{value:vg++}),this.uuid=mi(),this.name="",this.source=new If(t),this.mipmaps=[],this.mapping=e,this.channel=0,this.wrapS=i,this.wrapT=s,this.magFilter=r,this.minFilter=o,this.anisotropy=c,this.format=a,this.internalFormat=null,this.type=l,this.offset=new Tt(0,0),this.repeat=new Tt(1,1),this.center=new Tt(0,0),this.rotation=0,this.matrixAutoUpdate=!0,this.matrix=new Ft,this.generateMipmaps=!0,this.premultiplyAlpha=!1,this.flipY=!0,this.unpackAlignment=4,this.colorSpace=h,this.userData={},this.version=0,this.onUpdate=null,this.isRenderTargetTexture=!1,this.pmremVersion=0}get image(){return this.source.data}set image(t=null){this.source.data=t}updateMatrix(){this.matrix.setUvTransform(this.offset.x,this.offset.y,this.repeat.x,this.repeat.y,this.rotation,this.center.x,this.center.y)}clone(){return new this.constructor().copy(this)}copy(t){return this.name=t.name,this.source=t.source,this.mipmaps=t.mipmaps.slice(0),this.mapping=t.mapping,this.channel=t.channel,this.wrapS=t.wrapS,this.wrapT=t.wrapT,this.magFilter=t.magFilter,this.minFilter=t.minFilter,this.anisotropy=t.anisotropy,this.format=t.format,this.internalFormat=t.internalFormat,this.type=t.type,this.offset.copy(t.offset),this.repeat.copy(t.repeat),this.center.copy(t.center),this.rotation=t.rotation,this.matrixAutoUpdate=t.matrixAutoUpdate,this.matrix.copy(t.matrix),this.generateMipmaps=t.generateMipmaps,this.premultiplyAlpha=t.premultiplyAlpha,this.flipY=t.flipY,this.unpackAlignment=t.unpackAlignment,this.colorSpace=t.colorSpace,this.userData=JSON.parse(JSON.stringify(t.userData)),this.needsUpdate=!0,this}toJSON(t){const e=t===void 0||typeof t=="string";if(!e&&t.textures[this.uuid]!==void 0)return t.textures[this.uuid];const i={metadata:{version:4.6,type:"Texture",generator:"Texture.toJSON"},uuid:this.uuid,name:this.name,image:this.source.toJSON(t).uuid,mapping:this.mapping,channel:this.channel,repeat:[this.repeat.x,this.repeat.y],offset:[this.offset.x,this.offset.y],center:[this.center.x,this.center.y],rotation:this.rotation,wrap:[this.wrapS,this.wrapT],format:this.format,internalFormat:this.internalFormat,type:this.type,colorSpace:this.colorSpace,minFilter:this.minFilter,magFilter:this.magFilter,anisotropy:this.anisotropy,flipY:this.flipY,generateMipmaps:this.generateMipmaps,premultiplyAlpha:this.premultiplyAlpha,unpackAlignment:this.unpackAlignment};return Object.keys(this.userData).length>0&&(i.userData=this.userData),e||(t.textures[this.uuid]=i),i}dispose(){this.dispatchEvent({type:"dispose"})}transformUv(t){if(this.mapping!==vf)return t;if(t.applyMatrix3(this.matrix),t.x<0||t.x>1)switch(this.wrapS){case Il:t.x=t.x-Math.floor(t.x);break;case Bi:t.x=t.x<0?0:1;break;case Ol:Math.abs(Math.floor(t.x)%2)===1?t.x=Math.ceil(t.x)-t.x:t.x=t.x-Math.floor(t.x);break}if(t.y<0||t.y>1)switch(this.wrapT){case Il:t.y=t.y-Math.floor(t.y);break;case Bi:t.y=t.y<0?0:1;break;case Ol:Math.abs(Math.floor(t.y)%2)===1?t.y=Math.ceil(t.y)-t.y:t.y=t.y-Math.floor(t.y);break}return this.flipY&&(t.y=1-t.y),t}set needsUpdate(t){t===!0&&(this.version++,this.source.needsUpdate=!0)}set needsPMREMUpdate(t){t===!0&&this.pmremVersion++}}We.DEFAULT_IMAGE=null;We.DEFAULT_MAPPING=vf;We.DEFAULT_ANISOTROPY=1;class be{constructor(t=0,e=0,i=0,s=1){be.prototype.isVector4=!0,this.x=t,this.y=e,this.z=i,this.w=s}get width(){return this.z}set width(t){this.z=t}get height(){return this.w}set height(t){this.w=t}set(t,e,i,s){return this.x=t,this.y=e,this.z=i,this.w=s,this}setScalar(t){return this.x=t,this.y=t,this.z=t,this.w=t,this}setX(t){return this.x=t,this}setY(t){return this.y=t,this}setZ(t){return this.z=t,this}setW(t){return this.w=t,this}setComponent(t,e){switch(t){case 0:this.x=e;break;case 1:this.y=e;break;case 2:this.z=e;break;case 3:this.w=e;break;default:throw new Error("index is out of range: "+t)}return this}getComponent(t){switch(t){case 0:return this.x;case 1:return this.y;case 2:return this.z;case 3:return this.w;default:throw new Error("index is out of range: "+t)}}clone(){return new this.constructor(this.x,this.y,this.z,this.w)}copy(t){return this.x=t.x,this.y=t.y,this.z=t.z,this.w=t.w!==void 0?t.w:1,this}add(t){return this.x+=t.x,this.y+=t.y,this.z+=t.z,this.w+=t.w,this}addScalar(t){return this.x+=t,this.y+=t,this.z+=t,this.w+=t,this}addVectors(t,e){return this.x=t.x+e.x,this.y=t.y+e.y,this.z=t.z+e.z,this.w=t.w+e.w,this}addScaledVector(t,e){return this.x+=t.x*e,this.y+=t.y*e,this.z+=t.z*e,this.w+=t.w*e,this}sub(t){return this.x-=t.x,this.y-=t.y,this.z-=t.z,this.w-=t.w,this}subScalar(t){return this.x-=t,this.y-=t,this.z-=t,this.w-=t,this}subVectors(t,e){return this.x=t.x-e.x,this.y=t.y-e.y,this.z=t.z-e.z,this.w=t.w-e.w,this}multiply(t){return this.x*=t.x,this.y*=t.y,this.z*=t.z,this.w*=t.w,this}multiplyScalar(t){return this.x*=t,this.y*=t,this.z*=t,this.w*=t,this}applyMatrix4(t){const e=this.x,i=this.y,s=this.z,r=this.w,o=t.elements;return this.x=o[0]*e+o[4]*i+o[8]*s+o[12]*r,this.y=o[1]*e+o[5]*i+o[9]*s+o[13]*r,this.z=o[2]*e+o[6]*i+o[10]*s+o[14]*r,this.w=o[3]*e+o[7]*i+o[11]*s+o[15]*r,this}divide(t){return this.x/=t.x,this.y/=t.y,this.z/=t.z,this.w/=t.w,this}divideScalar(t){return this.multiplyScalar(1/t)}setAxisAngleFromQuaternion(t){this.w=2*Math.acos(t.w);const e=Math.sqrt(1-t.w*t.w);return e<1e-4?(this.x=1,this.y=0,this.z=0):(this.x=t.x/e,this.y=t.y/e,this.z=t.z/e),this}setAxisAngleFromRotationMatrix(t){let e,i,s,r;const l=t.elements,c=l[0],h=l[4],u=l[8],d=l[1],f=l[5],g=l[9],_=l[2],m=l[6],p=l[10];if(Math.abs(h-d)<.01&&Math.abs(u-_)<.01&&Math.abs(g-m)<.01){if(Math.abs(h+d)<.1&&Math.abs(u+_)<.1&&Math.abs(g+m)<.1&&Math.abs(c+f+p-3)<.1)return this.set(1,0,0,0),this;e=Math.PI;const y=(c+1)/2,x=(f+1)/2,A=(p+1)/2,w=(h+d)/4,T=(u+_)/4,R=(g+m)/4;return y>x&&y>A?y<.01?(i=0,s=.707106781,r=.707106781):(i=Math.sqrt(y),s=w/i,r=T/i):x>A?x<.01?(i=.707106781,s=0,r=.707106781):(s=Math.sqrt(x),i=w/s,r=R/s):A<.01?(i=.707106781,s=.707106781,r=0):(r=Math.sqrt(A),i=T/r,s=R/r),this.set(i,s,r,e),this}let v=Math.sqrt((m-g)*(m-g)+(u-_)*(u-_)+(d-h)*(d-h));return Math.abs(v)<.001&&(v=1),this.x=(m-g)/v,this.y=(u-_)/v,this.z=(d-h)/v,this.w=Math.acos((c+f+p-1)/2),this}setFromMatrixPosition(t){const e=t.elements;return this.x=e[12],this.y=e[13],this.z=e[14],this.w=e[15],this}min(t){return this.x=Math.min(this.x,t.x),this.y=Math.min(this.y,t.y),this.z=Math.min(this.z,t.z),this.w=Math.min(this.w,t.w),this}max(t){return this.x=Math.max(this.x,t.x),this.y=Math.max(this.y,t.y),this.z=Math.max(this.z,t.z),this.w=Math.max(this.w,t.w),this}clamp(t,e){return this.x=Math.max(t.x,Math.min(e.x,this.x)),this.y=Math.max(t.y,Math.min(e.y,this.y)),this.z=Math.max(t.z,Math.min(e.z,this.z)),this.w=Math.max(t.w,Math.min(e.w,this.w)),this}clampScalar(t,e){return this.x=Math.max(t,Math.min(e,this.x)),this.y=Math.max(t,Math.min(e,this.y)),this.z=Math.max(t,Math.min(e,this.z)),this.w=Math.max(t,Math.min(e,this.w)),this}clampLength(t,e){const i=this.length();return this.divideScalar(i||1).multiplyScalar(Math.max(t,Math.min(e,i)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this.z=Math.floor(this.z),this.w=Math.floor(this.w),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this.z=Math.ceil(this.z),this.w=Math.ceil(this.w),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this.z=Math.round(this.z),this.w=Math.round(this.w),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this.z=Math.trunc(this.z),this.w=Math.trunc(this.w),this}negate(){return this.x=-this.x,this.y=-this.y,this.z=-this.z,this.w=-this.w,this}dot(t){return this.x*t.x+this.y*t.y+this.z*t.z+this.w*t.w}lengthSq(){return this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w}length(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)+Math.abs(this.z)+Math.abs(this.w)}normalize(){return this.divideScalar(this.length()||1)}setLength(t){return this.normalize().multiplyScalar(t)}lerp(t,e){return this.x+=(t.x-this.x)*e,this.y+=(t.y-this.y)*e,this.z+=(t.z-this.z)*e,this.w+=(t.w-this.w)*e,this}lerpVectors(t,e,i){return this.x=t.x+(e.x-t.x)*i,this.y=t.y+(e.y-t.y)*i,this.z=t.z+(e.z-t.z)*i,this.w=t.w+(e.w-t.w)*i,this}equals(t){return t.x===this.x&&t.y===this.y&&t.z===this.z&&t.w===this.w}fromArray(t,e=0){return this.x=t[e],this.y=t[e+1],this.z=t[e+2],this.w=t[e+3],this}toArray(t=[],e=0){return t[e]=this.x,t[e+1]=this.y,t[e+2]=this.z,t[e+3]=this.w,t}fromBufferAttribute(t,e){return this.x=t.getX(e),this.y=t.getY(e),this.z=t.getZ(e),this.w=t.getW(e),this}random(){return this.x=Math.random(),this.y=Math.random(),this.z=Math.random(),this.w=Math.random(),this}*[Symbol.iterator](){yield this.x,yield this.y,yield this.z,yield this.w}}class bg extends Ki{constructor(t=1,e=1,i={}){super(),this.isRenderTarget=!0,this.width=t,this.height=e,this.depth=1,this.scissor=new be(0,0,t,e),this.scissorTest=!1,this.viewport=new be(0,0,t,e);const s={width:t,height:e,depth:1};i=Object.assign({generateMipmaps:!1,internalFormat:null,minFilter:Cn,depthBuffer:!0,stencilBuffer:!1,resolveDepthBuffer:!0,resolveStencilBuffer:!0,depthTexture:null,samples:0,count:1},i);const r=new We(s,i.mapping,i.wrapS,i.wrapT,i.magFilter,i.minFilter,i.format,i.type,i.anisotropy,i.colorSpace);r.flipY=!1,r.generateMipmaps=i.generateMipmaps,r.internalFormat=i.internalFormat,this.textures=[];const o=i.count;for(let a=0;a<o;a++)this.textures[a]=r.clone(),this.textures[a].isRenderTargetTexture=!0;this.depthBuffer=i.depthBuffer,this.stencilBuffer=i.stencilBuffer,this.resolveDepthBuffer=i.resolveDepthBuffer,this.resolveStencilBuffer=i.resolveStencilBuffer,this.depthTexture=i.depthTexture,this.samples=i.samples}get texture(){return this.textures[0]}set texture(t){this.textures[0]=t}setSize(t,e,i=1){if(this.width!==t||this.height!==e||this.depth!==i){this.width=t,this.height=e,this.depth=i;for(let s=0,r=this.textures.length;s<r;s++)this.textures[s].image.width=t,this.textures[s].image.height=e,this.textures[s].image.depth=i;this.dispose()}this.viewport.set(0,0,t,e),this.scissor.set(0,0,t,e)}clone(){return new this.constructor().copy(this)}copy(t){this.width=t.width,this.height=t.height,this.depth=t.depth,this.scissor.copy(t.scissor),this.scissorTest=t.scissorTest,this.viewport.copy(t.viewport),this.textures.length=0;for(let i=0,s=t.textures.length;i<s;i++)this.textures[i]=t.textures[i].clone(),this.textures[i].isRenderTargetTexture=!0;const e=Object.assign({},t.texture.image);return this.texture.source=new If(e),this.depthBuffer=t.depthBuffer,this.stencilBuffer=t.stencilBuffer,this.resolveDepthBuffer=t.resolveDepthBuffer,this.resolveStencilBuffer=t.resolveStencilBuffer,t.depthTexture!==null&&(this.depthTexture=t.depthTexture.clone()),this.samples=t.samples,this}dispose(){this.dispatchEvent({type:"dispose"})}}class ji extends bg{constructor(t=1,e=1,i={}){super(t,e,i),this.isWebGLRenderTarget=!0}}class Of extends We{constructor(t=null,e=1,i=1,s=1){super(null),this.isDataArrayTexture=!0,this.image={data:t,width:e,height:i,depth:s},this.magFilter=xn,this.minFilter=xn,this.wrapR=Bi,this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1,this.layerUpdates=new Set}addLayerUpdate(t){this.layerUpdates.add(t)}clearLayerUpdates(){this.layerUpdates.clear()}}class yg extends We{constructor(t=null,e=1,i=1,s=1){super(null),this.isData3DTexture=!0,this.image={data:t,width:e,height:i,depth:s},this.magFilter=xn,this.minFilter=xn,this.wrapR=Bi,this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1}}class $i{constructor(t=0,e=0,i=0,s=1){this.isQuaternion=!0,this._x=t,this._y=e,this._z=i,this._w=s}static slerpFlat(t,e,i,s,r,o,a){let l=i[s+0],c=i[s+1],h=i[s+2],u=i[s+3];const d=r[o+0],f=r[o+1],g=r[o+2],_=r[o+3];if(a===0){t[e+0]=l,t[e+1]=c,t[e+2]=h,t[e+3]=u;return}if(a===1){t[e+0]=d,t[e+1]=f,t[e+2]=g,t[e+3]=_;return}if(u!==_||l!==d||c!==f||h!==g){let m=1-a;const p=l*d+c*f+h*g+u*_,v=p>=0?1:-1,y=1-p*p;if(y>Number.EPSILON){const A=Math.sqrt(y),w=Math.atan2(A,p*v);m=Math.sin(m*w)/A,a=Math.sin(a*w)/A}const x=a*v;if(l=l*m+d*x,c=c*m+f*x,h=h*m+g*x,u=u*m+_*x,m===1-a){const A=1/Math.sqrt(l*l+c*c+h*h+u*u);l*=A,c*=A,h*=A,u*=A}}t[e]=l,t[e+1]=c,t[e+2]=h,t[e+3]=u}static multiplyQuaternionsFlat(t,e,i,s,r,o){const a=i[s],l=i[s+1],c=i[s+2],h=i[s+3],u=r[o],d=r[o+1],f=r[o+2],g=r[o+3];return t[e]=a*g+h*u+l*f-c*d,t[e+1]=l*g+h*d+c*u-a*f,t[e+2]=c*g+h*f+a*d-l*u,t[e+3]=h*g-a*u-l*d-c*f,t}get x(){return this._x}set x(t){this._x=t,this._onChangeCallback()}get y(){return this._y}set y(t){this._y=t,this._onChangeCallback()}get z(){return this._z}set z(t){this._z=t,this._onChangeCallback()}get w(){return this._w}set w(t){this._w=t,this._onChangeCallback()}set(t,e,i,s){return this._x=t,this._y=e,this._z=i,this._w=s,this._onChangeCallback(),this}clone(){return new this.constructor(this._x,this._y,this._z,this._w)}copy(t){return this._x=t.x,this._y=t.y,this._z=t.z,this._w=t.w,this._onChangeCallback(),this}setFromEuler(t,e=!0){const i=t._x,s=t._y,r=t._z,o=t._order,a=Math.cos,l=Math.sin,c=a(i/2),h=a(s/2),u=a(r/2),d=l(i/2),f=l(s/2),g=l(r/2);switch(o){case"XYZ":this._x=d*h*u+c*f*g,this._y=c*f*u-d*h*g,this._z=c*h*g+d*f*u,this._w=c*h*u-d*f*g;break;case"YXZ":this._x=d*h*u+c*f*g,this._y=c*f*u-d*h*g,this._z=c*h*g-d*f*u,this._w=c*h*u+d*f*g;break;case"ZXY":this._x=d*h*u-c*f*g,this._y=c*f*u+d*h*g,this._z=c*h*g+d*f*u,this._w=c*h*u-d*f*g;break;case"ZYX":this._x=d*h*u-c*f*g,this._y=c*f*u+d*h*g,this._z=c*h*g-d*f*u,this._w=c*h*u+d*f*g;break;case"YZX":this._x=d*h*u+c*f*g,this._y=c*f*u+d*h*g,this._z=c*h*g-d*f*u,this._w=c*h*u-d*f*g;break;case"XZY":this._x=d*h*u-c*f*g,this._y=c*f*u-d*h*g,this._z=c*h*g+d*f*u,this._w=c*h*u+d*f*g;break;default:console.warn("THREE.Quaternion: .setFromEuler() encountered an unknown order: "+o)}return e===!0&&this._onChangeCallback(),this}setFromAxisAngle(t,e){const i=e/2,s=Math.sin(i);return this._x=t.x*s,this._y=t.y*s,this._z=t.z*s,this._w=Math.cos(i),this._onChangeCallback(),this}setFromRotationMatrix(t){const e=t.elements,i=e[0],s=e[4],r=e[8],o=e[1],a=e[5],l=e[9],c=e[2],h=e[6],u=e[10],d=i+a+u;if(d>0){const f=.5/Math.sqrt(d+1);this._w=.25/f,this._x=(h-l)*f,this._y=(r-c)*f,this._z=(o-s)*f}else if(i>a&&i>u){const f=2*Math.sqrt(1+i-a-u);this._w=(h-l)/f,this._x=.25*f,this._y=(s+o)/f,this._z=(r+c)/f}else if(a>u){const f=2*Math.sqrt(1+a-i-u);this._w=(r-c)/f,this._x=(s+o)/f,this._y=.25*f,this._z=(l+h)/f}else{const f=2*Math.sqrt(1+u-i-a);this._w=(o-s)/f,this._x=(r+c)/f,this._y=(l+h)/f,this._z=.25*f}return this._onChangeCallback(),this}setFromUnitVectors(t,e){let i=t.dot(e)+1;return i<Number.EPSILON?(i=0,Math.abs(t.x)>Math.abs(t.z)?(this._x=-t.y,this._y=t.x,this._z=0,this._w=i):(this._x=0,this._y=-t.z,this._z=t.y,this._w=i)):(this._x=t.y*e.z-t.z*e.y,this._y=t.z*e.x-t.x*e.z,this._z=t.x*e.y-t.y*e.x,this._w=i),this.normalize()}angleTo(t){return 2*Math.acos(Math.abs(Ge(this.dot(t),-1,1)))}rotateTowards(t,e){const i=this.angleTo(t);if(i===0)return this;const s=Math.min(1,e/i);return this.slerp(t,s),this}identity(){return this.set(0,0,0,1)}invert(){return this.conjugate()}conjugate(){return this._x*=-1,this._y*=-1,this._z*=-1,this._onChangeCallback(),this}dot(t){return this._x*t._x+this._y*t._y+this._z*t._z+this._w*t._w}lengthSq(){return this._x*this._x+this._y*this._y+this._z*this._z+this._w*this._w}length(){return Math.sqrt(this._x*this._x+this._y*this._y+this._z*this._z+this._w*this._w)}normalize(){let t=this.length();return t===0?(this._x=0,this._y=0,this._z=0,this._w=1):(t=1/t,this._x=this._x*t,this._y=this._y*t,this._z=this._z*t,this._w=this._w*t),this._onChangeCallback(),this}multiply(t){return this.multiplyQuaternions(this,t)}premultiply(t){return this.multiplyQuaternions(t,this)}multiplyQuaternions(t,e){const i=t._x,s=t._y,r=t._z,o=t._w,a=e._x,l=e._y,c=e._z,h=e._w;return this._x=i*h+o*a+s*c-r*l,this._y=s*h+o*l+r*a-i*c,this._z=r*h+o*c+i*l-s*a,this._w=o*h-i*a-s*l-r*c,this._onChangeCallback(),this}slerp(t,e){if(e===0)return this;if(e===1)return this.copy(t);const i=this._x,s=this._y,r=this._z,o=this._w;let a=o*t._w+i*t._x+s*t._y+r*t._z;if(a<0?(this._w=-t._w,this._x=-t._x,this._y=-t._y,this._z=-t._z,a=-a):this.copy(t),a>=1)return this._w=o,this._x=i,this._y=s,this._z=r,this;const l=1-a*a;if(l<=Number.EPSILON){const f=1-e;return this._w=f*o+e*this._w,this._x=f*i+e*this._x,this._y=f*s+e*this._y,this._z=f*r+e*this._z,this.normalize(),this}const c=Math.sqrt(l),h=Math.atan2(c,a),u=Math.sin((1-e)*h)/c,d=Math.sin(e*h)/c;return this._w=o*u+this._w*d,this._x=i*u+this._x*d,this._y=s*u+this._y*d,this._z=r*u+this._z*d,this._onChangeCallback(),this}slerpQuaternions(t,e,i){return this.copy(t).slerp(e,i)}random(){const t=2*Math.PI*Math.random(),e=2*Math.PI*Math.random(),i=Math.random(),s=Math.sqrt(1-i),r=Math.sqrt(i);return this.set(s*Math.sin(t),s*Math.cos(t),r*Math.sin(e),r*Math.cos(e))}equals(t){return t._x===this._x&&t._y===this._y&&t._z===this._z&&t._w===this._w}fromArray(t,e=0){return this._x=t[e],this._y=t[e+1],this._z=t[e+2],this._w=t[e+3],this._onChangeCallback(),this}toArray(t=[],e=0){return t[e]=this._x,t[e+1]=this._y,t[e+2]=this._z,t[e+3]=this._w,t}fromBufferAttribute(t,e){return this._x=t.getX(e),this._y=t.getY(e),this._z=t.getZ(e),this._w=t.getW(e),this._onChangeCallback(),this}toJSON(){return this.toArray()}_onChange(t){return this._onChangeCallback=t,this}_onChangeCallback(){}*[Symbol.iterator](){yield this._x,yield this._y,yield this._z,yield this._w}}class L{constructor(t=0,e=0,i=0){L.prototype.isVector3=!0,this.x=t,this.y=e,this.z=i}set(t,e,i){return i===void 0&&(i=this.z),this.x=t,this.y=e,this.z=i,this}setScalar(t){return this.x=t,this.y=t,this.z=t,this}setX(t){return this.x=t,this}setY(t){return this.y=t,this}setZ(t){return this.z=t,this}setComponent(t,e){switch(t){case 0:this.x=e;break;case 1:this.y=e;break;case 2:this.z=e;break;default:throw new Error("index is out of range: "+t)}return this}getComponent(t){switch(t){case 0:return this.x;case 1:return this.y;case 2:return this.z;default:throw new Error("index is out of range: "+t)}}clone(){return new this.constructor(this.x,this.y,this.z)}copy(t){return this.x=t.x,this.y=t.y,this.z=t.z,this}add(t){return this.x+=t.x,this.y+=t.y,this.z+=t.z,this}addScalar(t){return this.x+=t,this.y+=t,this.z+=t,this}addVectors(t,e){return this.x=t.x+e.x,this.y=t.y+e.y,this.z=t.z+e.z,this}addScaledVector(t,e){return this.x+=t.x*e,this.y+=t.y*e,this.z+=t.z*e,this}sub(t){return this.x-=t.x,this.y-=t.y,this.z-=t.z,this}subScalar(t){return this.x-=t,this.y-=t,this.z-=t,this}subVectors(t,e){return this.x=t.x-e.x,this.y=t.y-e.y,this.z=t.z-e.z,this}multiply(t){return this.x*=t.x,this.y*=t.y,this.z*=t.z,this}multiplyScalar(t){return this.x*=t,this.y*=t,this.z*=t,this}multiplyVectors(t,e){return this.x=t.x*e.x,this.y=t.y*e.y,this.z=t.z*e.z,this}applyEuler(t){return this.applyQuaternion(Oh.setFromEuler(t))}applyAxisAngle(t,e){return this.applyQuaternion(Oh.setFromAxisAngle(t,e))}applyMatrix3(t){const e=this.x,i=this.y,s=this.z,r=t.elements;return this.x=r[0]*e+r[3]*i+r[6]*s,this.y=r[1]*e+r[4]*i+r[7]*s,this.z=r[2]*e+r[5]*i+r[8]*s,this}applyNormalMatrix(t){return this.applyMatrix3(t).normalize()}applyMatrix4(t){const e=this.x,i=this.y,s=this.z,r=t.elements,o=1/(r[3]*e+r[7]*i+r[11]*s+r[15]);return this.x=(r[0]*e+r[4]*i+r[8]*s+r[12])*o,this.y=(r[1]*e+r[5]*i+r[9]*s+r[13])*o,this.z=(r[2]*e+r[6]*i+r[10]*s+r[14])*o,this}applyQuaternion(t){const e=this.x,i=this.y,s=this.z,r=t.x,o=t.y,a=t.z,l=t.w,c=2*(o*s-a*i),h=2*(a*e-r*s),u=2*(r*i-o*e);return this.x=e+l*c+o*u-a*h,this.y=i+l*h+a*c-r*u,this.z=s+l*u+r*h-o*c,this}project(t){return this.applyMatrix4(t.matrixWorldInverse).applyMatrix4(t.projectionMatrix)}unproject(t){return this.applyMatrix4(t.projectionMatrixInverse).applyMatrix4(t.matrixWorld)}transformDirection(t){const e=this.x,i=this.y,s=this.z,r=t.elements;return this.x=r[0]*e+r[4]*i+r[8]*s,this.y=r[1]*e+r[5]*i+r[9]*s,this.z=r[2]*e+r[6]*i+r[10]*s,this.normalize()}divide(t){return this.x/=t.x,this.y/=t.y,this.z/=t.z,this}divideScalar(t){return this.multiplyScalar(1/t)}min(t){return this.x=Math.min(this.x,t.x),this.y=Math.min(this.y,t.y),this.z=Math.min(this.z,t.z),this}max(t){return this.x=Math.max(this.x,t.x),this.y=Math.max(this.y,t.y),this.z=Math.max(this.z,t.z),this}clamp(t,e){return this.x=Math.max(t.x,Math.min(e.x,this.x)),this.y=Math.max(t.y,Math.min(e.y,this.y)),this.z=Math.max(t.z,Math.min(e.z,this.z)),this}clampScalar(t,e){return this.x=Math.max(t,Math.min(e,this.x)),this.y=Math.max(t,Math.min(e,this.y)),this.z=Math.max(t,Math.min(e,this.z)),this}clampLength(t,e){const i=this.length();return this.divideScalar(i||1).multiplyScalar(Math.max(t,Math.min(e,i)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this.z=Math.floor(this.z),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this.z=Math.ceil(this.z),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this.z=Math.round(this.z),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this.z=Math.trunc(this.z),this}negate(){return this.x=-this.x,this.y=-this.y,this.z=-this.z,this}dot(t){return this.x*t.x+this.y*t.y+this.z*t.z}lengthSq(){return this.x*this.x+this.y*this.y+this.z*this.z}length(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)+Math.abs(this.z)}normalize(){return this.divideScalar(this.length()||1)}setLength(t){return this.normalize().multiplyScalar(t)}lerp(t,e){return this.x+=(t.x-this.x)*e,this.y+=(t.y-this.y)*e,this.z+=(t.z-this.z)*e,this}lerpVectors(t,e,i){return this.x=t.x+(e.x-t.x)*i,this.y=t.y+(e.y-t.y)*i,this.z=t.z+(e.z-t.z)*i,this}cross(t){return this.crossVectors(this,t)}crossVectors(t,e){const i=t.x,s=t.y,r=t.z,o=e.x,a=e.y,l=e.z;return this.x=s*l-r*a,this.y=r*o-i*l,this.z=i*a-s*o,this}projectOnVector(t){const e=t.lengthSq();if(e===0)return this.set(0,0,0);const i=t.dot(this)/e;return this.copy(t).multiplyScalar(i)}projectOnPlane(t){return La.copy(this).projectOnVector(t),this.sub(La)}reflect(t){return this.sub(La.copy(t).multiplyScalar(2*this.dot(t)))}angleTo(t){const e=Math.sqrt(this.lengthSq()*t.lengthSq());if(e===0)return Math.PI/2;const i=this.dot(t)/e;return Math.acos(Ge(i,-1,1))}distanceTo(t){return Math.sqrt(this.distanceToSquared(t))}distanceToSquared(t){const e=this.x-t.x,i=this.y-t.y,s=this.z-t.z;return e*e+i*i+s*s}manhattanDistanceTo(t){return Math.abs(this.x-t.x)+Math.abs(this.y-t.y)+Math.abs(this.z-t.z)}setFromSpherical(t){return this.setFromSphericalCoords(t.radius,t.phi,t.theta)}setFromSphericalCoords(t,e,i){const s=Math.sin(e)*t;return this.x=s*Math.sin(i),this.y=Math.cos(e)*t,this.z=s*Math.cos(i),this}setFromCylindrical(t){return this.setFromCylindricalCoords(t.radius,t.theta,t.y)}setFromCylindricalCoords(t,e,i){return this.x=t*Math.sin(e),this.y=i,this.z=t*Math.cos(e),this}setFromMatrixPosition(t){const e=t.elements;return this.x=e[12],this.y=e[13],this.z=e[14],this}setFromMatrixScale(t){const e=this.setFromMatrixColumn(t,0).length(),i=this.setFromMatrixColumn(t,1).length(),s=this.setFromMatrixColumn(t,2).length();return this.x=e,this.y=i,this.z=s,this}setFromMatrixColumn(t,e){return this.fromArray(t.elements,e*4)}setFromMatrix3Column(t,e){return this.fromArray(t.elements,e*3)}setFromEuler(t){return this.x=t._x,this.y=t._y,this.z=t._z,this}setFromColor(t){return this.x=t.r,this.y=t.g,this.z=t.b,this}equals(t){return t.x===this.x&&t.y===this.y&&t.z===this.z}fromArray(t,e=0){return this.x=t[e],this.y=t[e+1],this.z=t[e+2],this}toArray(t=[],e=0){return t[e]=this.x,t[e+1]=this.y,t[e+2]=this.z,t}fromBufferAttribute(t,e){return this.x=t.getX(e),this.y=t.getY(e),this.z=t.getZ(e),this}random(){return this.x=Math.random(),this.y=Math.random(),this.z=Math.random(),this}randomDirection(){const t=Math.random()*Math.PI*2,e=Math.random()*2-1,i=Math.sqrt(1-e*e);return this.x=i*Math.cos(t),this.y=e,this.z=i*Math.sin(t),this}*[Symbol.iterator](){yield this.x,yield this.y,yield this.z}}const La=new L,Oh=new $i;class Lr{constructor(t=new L(1/0,1/0,1/0),e=new L(-1/0,-1/0,-1/0)){this.isBox3=!0,this.min=t,this.max=e}set(t,e){return this.min.copy(t),this.max.copy(e),this}setFromArray(t){this.makeEmpty();for(let e=0,i=t.length;e<i;e+=3)this.expandByPoint(dn.fromArray(t,e));return this}setFromBufferAttribute(t){this.makeEmpty();for(let e=0,i=t.count;e<i;e++)this.expandByPoint(dn.fromBufferAttribute(t,e));return this}setFromPoints(t){this.makeEmpty();for(let e=0,i=t.length;e<i;e++)this.expandByPoint(t[e]);return this}setFromCenterAndSize(t,e){const i=dn.copy(e).multiplyScalar(.5);return this.min.copy(t).sub(i),this.max.copy(t).add(i),this}setFromObject(t,e=!1){return this.makeEmpty(),this.expandByObject(t,e)}clone(){return new this.constructor().copy(this)}copy(t){return this.min.copy(t.min),this.max.copy(t.max),this}makeEmpty(){return this.min.x=this.min.y=this.min.z=1/0,this.max.x=this.max.y=this.max.z=-1/0,this}isEmpty(){return this.max.x<this.min.x||this.max.y<this.min.y||this.max.z<this.min.z}getCenter(t){return this.isEmpty()?t.set(0,0,0):t.addVectors(this.min,this.max).multiplyScalar(.5)}getSize(t){return this.isEmpty()?t.set(0,0,0):t.subVectors(this.max,this.min)}expandByPoint(t){return this.min.min(t),this.max.max(t),this}expandByVector(t){return this.min.sub(t),this.max.add(t),this}expandByScalar(t){return this.min.addScalar(-t),this.max.addScalar(t),this}expandByObject(t,e=!1){t.updateWorldMatrix(!1,!1);const i=t.geometry;if(i!==void 0){const r=i.getAttribute("position");if(e===!0&&r!==void 0&&t.isInstancedMesh!==!0)for(let o=0,a=r.count;o<a;o++)t.isMesh===!0?t.getVertexPosition(o,dn):dn.fromBufferAttribute(r,o),dn.applyMatrix4(t.matrixWorld),this.expandByPoint(dn);else t.boundingBox!==void 0?(t.boundingBox===null&&t.computeBoundingBox(),Gr.copy(t.boundingBox)):(i.boundingBox===null&&i.computeBoundingBox(),Gr.copy(i.boundingBox)),Gr.applyMatrix4(t.matrixWorld),this.union(Gr)}const s=t.children;for(let r=0,o=s.length;r<o;r++)this.expandByObject(s[r],e);return this}containsPoint(t){return t.x>=this.min.x&&t.x<=this.max.x&&t.y>=this.min.y&&t.y<=this.max.y&&t.z>=this.min.z&&t.z<=this.max.z}containsBox(t){return this.min.x<=t.min.x&&t.max.x<=this.max.x&&this.min.y<=t.min.y&&t.max.y<=this.max.y&&this.min.z<=t.min.z&&t.max.z<=this.max.z}getParameter(t,e){return e.set((t.x-this.min.x)/(this.max.x-this.min.x),(t.y-this.min.y)/(this.max.y-this.min.y),(t.z-this.min.z)/(this.max.z-this.min.z))}intersectsBox(t){return t.max.x>=this.min.x&&t.min.x<=this.max.x&&t.max.y>=this.min.y&&t.min.y<=this.max.y&&t.max.z>=this.min.z&&t.min.z<=this.max.z}intersectsSphere(t){return this.clampPoint(t.center,dn),dn.distanceToSquared(t.center)<=t.radius*t.radius}intersectsPlane(t){let e,i;return t.normal.x>0?(e=t.normal.x*this.min.x,i=t.normal.x*this.max.x):(e=t.normal.x*this.max.x,i=t.normal.x*this.min.x),t.normal.y>0?(e+=t.normal.y*this.min.y,i+=t.normal.y*this.max.y):(e+=t.normal.y*this.max.y,i+=t.normal.y*this.min.y),t.normal.z>0?(e+=t.normal.z*this.min.z,i+=t.normal.z*this.max.z):(e+=t.normal.z*this.max.z,i+=t.normal.z*this.min.z),e<=-t.constant&&i>=-t.constant}intersectsTriangle(t){if(this.isEmpty())return!1;this.getCenter(js),Wr.subVectors(this.max,js),ns.subVectors(t.a,js),is.subVectors(t.b,js),ss.subVectors(t.c,js),ei.subVectors(is,ns),ni.subVectors(ss,is),Ei.subVectors(ns,ss);let e=[0,-ei.z,ei.y,0,-ni.z,ni.y,0,-Ei.z,Ei.y,ei.z,0,-ei.x,ni.z,0,-ni.x,Ei.z,0,-Ei.x,-ei.y,ei.x,0,-ni.y,ni.x,0,-Ei.y,Ei.x,0];return!Ia(e,ns,is,ss,Wr)||(e=[1,0,0,0,1,0,0,0,1],!Ia(e,ns,is,ss,Wr))?!1:(Xr.crossVectors(ei,ni),e=[Xr.x,Xr.y,Xr.z],Ia(e,ns,is,ss,Wr))}clampPoint(t,e){return e.copy(t).clamp(this.min,this.max)}distanceToPoint(t){return this.clampPoint(t,dn).distanceTo(t)}getBoundingSphere(t){return this.isEmpty()?t.makeEmpty():(this.getCenter(t.center),t.radius=this.getSize(dn).length()*.5),t}intersect(t){return this.min.max(t.min),this.max.min(t.max),this.isEmpty()&&this.makeEmpty(),this}union(t){return this.min.min(t.min),this.max.max(t.max),this}applyMatrix4(t){return this.isEmpty()?this:(In[0].set(this.min.x,this.min.y,this.min.z).applyMatrix4(t),In[1].set(this.min.x,this.min.y,this.max.z).applyMatrix4(t),In[2].set(this.min.x,this.max.y,this.min.z).applyMatrix4(t),In[3].set(this.min.x,this.max.y,this.max.z).applyMatrix4(t),In[4].set(this.max.x,this.min.y,this.min.z).applyMatrix4(t),In[5].set(this.max.x,this.min.y,this.max.z).applyMatrix4(t),In[6].set(this.max.x,this.max.y,this.min.z).applyMatrix4(t),In[7].set(this.max.x,this.max.y,this.max.z).applyMatrix4(t),this.setFromPoints(In),this)}translate(t){return this.min.add(t),this.max.add(t),this}equals(t){return t.min.equals(this.min)&&t.max.equals(this.max)}}const In=[new L,new L,new L,new L,new L,new L,new L,new L],dn=new L,Gr=new Lr,ns=new L,is=new L,ss=new L,ei=new L,ni=new L,Ei=new L,js=new L,Wr=new L,Xr=new L,Ti=new L;function Ia(n,t,e,i,s){for(let r=0,o=n.length-3;r<=o;r+=3){Ti.fromArray(n,r);const a=s.x*Math.abs(Ti.x)+s.y*Math.abs(Ti.y)+s.z*Math.abs(Ti.z),l=t.dot(Ti),c=e.dot(Ti),h=i.dot(Ti);if(Math.max(-Math.max(l,c,h),Math.min(l,c,h))>a)return!1}return!0}const Mg=new Lr,$s=new L,Oa=new L;class Ir{constructor(t=new L,e=-1){this.isSphere=!0,this.center=t,this.radius=e}set(t,e){return this.center.copy(t),this.radius=e,this}setFromPoints(t,e){const i=this.center;e!==void 0?i.copy(e):Mg.setFromPoints(t).getCenter(i);let s=0;for(let r=0,o=t.length;r<o;r++)s=Math.max(s,i.distanceToSquared(t[r]));return this.radius=Math.sqrt(s),this}copy(t){return this.center.copy(t.center),this.radius=t.radius,this}isEmpty(){return this.radius<0}makeEmpty(){return this.center.set(0,0,0),this.radius=-1,this}containsPoint(t){return t.distanceToSquared(this.center)<=this.radius*this.radius}distanceToPoint(t){return t.distanceTo(this.center)-this.radius}intersectsSphere(t){const e=this.radius+t.radius;return t.center.distanceToSquared(this.center)<=e*e}intersectsBox(t){return t.intersectsSphere(this)}intersectsPlane(t){return Math.abs(t.distanceToPoint(this.center))<=this.radius}clampPoint(t,e){const i=this.center.distanceToSquared(t);return e.copy(t),i>this.radius*this.radius&&(e.sub(this.center).normalize(),e.multiplyScalar(this.radius).add(this.center)),e}getBoundingBox(t){return this.isEmpty()?(t.makeEmpty(),t):(t.set(this.center,this.center),t.expandByScalar(this.radius),t)}applyMatrix4(t){return this.center.applyMatrix4(t),this.radius=this.radius*t.getMaxScaleOnAxis(),this}translate(t){return this.center.add(t),this}expandByPoint(t){if(this.isEmpty())return this.center.copy(t),this.radius=0,this;$s.subVectors(t,this.center);const e=$s.lengthSq();if(e>this.radius*this.radius){const i=Math.sqrt(e),s=(i-this.radius)*.5;this.center.addScaledVector($s,s/i),this.radius+=s}return this}union(t){return t.isEmpty()?this:this.isEmpty()?(this.copy(t),this):(this.center.equals(t.center)===!0?this.radius=Math.max(this.radius,t.radius):(Oa.subVectors(t.center,this.center).setLength(t.radius),this.expandByPoint($s.copy(t.center).add(Oa)),this.expandByPoint($s.copy(t.center).sub(Oa))),this)}equals(t){return t.center.equals(this.center)&&t.radius===this.radius}clone(){return new this.constructor().copy(this)}}const On=new L,Ua=new L,jr=new L,ii=new L,Na=new L,$r=new L,Fa=new L;class Or{constructor(t=new L,e=new L(0,0,-1)){this.origin=t,this.direction=e}set(t,e){return this.origin.copy(t),this.direction.copy(e),this}copy(t){return this.origin.copy(t.origin),this.direction.copy(t.direction),this}at(t,e){return e.copy(this.origin).addScaledVector(this.direction,t)}lookAt(t){return this.direction.copy(t).sub(this.origin).normalize(),this}recast(t){return this.origin.copy(this.at(t,On)),this}closestPointToPoint(t,e){e.subVectors(t,this.origin);const i=e.dot(this.direction);return i<0?e.copy(this.origin):e.copy(this.origin).addScaledVector(this.direction,i)}distanceToPoint(t){return Math.sqrt(this.distanceSqToPoint(t))}distanceSqToPoint(t){const e=On.subVectors(t,this.origin).dot(this.direction);return e<0?this.origin.distanceToSquared(t):(On.copy(this.origin).addScaledVector(this.direction,e),On.distanceToSquared(t))}distanceSqToSegment(t,e,i,s){Ua.copy(t).add(e).multiplyScalar(.5),jr.copy(e).sub(t).normalize(),ii.copy(this.origin).sub(Ua);const r=t.distanceTo(e)*.5,o=-this.direction.dot(jr),a=ii.dot(this.direction),l=-ii.dot(jr),c=ii.lengthSq(),h=Math.abs(1-o*o);let u,d,f,g;if(h>0)if(u=o*l-a,d=o*a-l,g=r*h,u>=0)if(d>=-g)if(d<=g){const _=1/h;u*=_,d*=_,f=u*(u+o*d+2*a)+d*(o*u+d+2*l)+c}else d=r,u=Math.max(0,-(o*d+a)),f=-u*u+d*(d+2*l)+c;else d=-r,u=Math.max(0,-(o*d+a)),f=-u*u+d*(d+2*l)+c;else d<=-g?(u=Math.max(0,-(-o*r+a)),d=u>0?-r:Math.min(Math.max(-r,-l),r),f=-u*u+d*(d+2*l)+c):d<=g?(u=0,d=Math.min(Math.max(-r,-l),r),f=d*(d+2*l)+c):(u=Math.max(0,-(o*r+a)),d=u>0?r:Math.min(Math.max(-r,-l),r),f=-u*u+d*(d+2*l)+c);else d=o>0?-r:r,u=Math.max(0,-(o*d+a)),f=-u*u+d*(d+2*l)+c;return i&&i.copy(this.origin).addScaledVector(this.direction,u),s&&s.copy(Ua).addScaledVector(jr,d),f}intersectSphere(t,e){On.subVectors(t.center,this.origin);const i=On.dot(this.direction),s=On.dot(On)-i*i,r=t.radius*t.radius;if(s>r)return null;const o=Math.sqrt(r-s),a=i-o,l=i+o;return l<0?null:a<0?this.at(l,e):this.at(a,e)}intersectsSphere(t){return this.distanceSqToPoint(t.center)<=t.radius*t.radius}distanceToPlane(t){const e=t.normal.dot(this.direction);if(e===0)return t.distanceToPoint(this.origin)===0?0:null;const i=-(this.origin.dot(t.normal)+t.constant)/e;return i>=0?i:null}intersectPlane(t,e){const i=this.distanceToPlane(t);return i===null?null:this.at(i,e)}intersectsPlane(t){const e=t.distanceToPoint(this.origin);return e===0||t.normal.dot(this.direction)*e<0}intersectBox(t,e){let i,s,r,o,a,l;const c=1/this.direction.x,h=1/this.direction.y,u=1/this.direction.z,d=this.origin;return c>=0?(i=(t.min.x-d.x)*c,s=(t.max.x-d.x)*c):(i=(t.max.x-d.x)*c,s=(t.min.x-d.x)*c),h>=0?(r=(t.min.y-d.y)*h,o=(t.max.y-d.y)*h):(r=(t.max.y-d.y)*h,o=(t.min.y-d.y)*h),i>o||r>s||((r>i||isNaN(i))&&(i=r),(o<s||isNaN(s))&&(s=o),u>=0?(a=(t.min.z-d.z)*u,l=(t.max.z-d.z)*u):(a=(t.max.z-d.z)*u,l=(t.min.z-d.z)*u),i>l||a>s)||((a>i||i!==i)&&(i=a),(l<s||s!==s)&&(s=l),s<0)?null:this.at(i>=0?i:s,e)}intersectsBox(t){return this.intersectBox(t,On)!==null}intersectTriangle(t,e,i,s,r){Na.subVectors(e,t),$r.subVectors(i,t),Fa.crossVectors(Na,$r);let o=this.direction.dot(Fa),a;if(o>0){if(s)return null;a=1}else if(o<0)a=-1,o=-o;else return null;ii.subVectors(this.origin,t);const l=a*this.direction.dot($r.crossVectors(ii,$r));if(l<0)return null;const c=a*this.direction.dot(Na.cross(ii));if(c<0||l+c>o)return null;const h=-a*ii.dot(Fa);return h<0?null:this.at(h/o,r)}applyMatrix4(t){return this.origin.applyMatrix4(t),this.direction.transformDirection(t),this}equals(t){return t.origin.equals(this.origin)&&t.direction.equals(this.direction)}clone(){return new this.constructor().copy(this)}}class oe{constructor(t,e,i,s,r,o,a,l,c,h,u,d,f,g,_,m){oe.prototype.isMatrix4=!0,this.elements=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],t!==void 0&&this.set(t,e,i,s,r,o,a,l,c,h,u,d,f,g,_,m)}set(t,e,i,s,r,o,a,l,c,h,u,d,f,g,_,m){const p=this.elements;return p[0]=t,p[4]=e,p[8]=i,p[12]=s,p[1]=r,p[5]=o,p[9]=a,p[13]=l,p[2]=c,p[6]=h,p[10]=u,p[14]=d,p[3]=f,p[7]=g,p[11]=_,p[15]=m,this}identity(){return this.set(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1),this}clone(){return new oe().fromArray(this.elements)}copy(t){const e=this.elements,i=t.elements;return e[0]=i[0],e[1]=i[1],e[2]=i[2],e[3]=i[3],e[4]=i[4],e[5]=i[5],e[6]=i[6],e[7]=i[7],e[8]=i[8],e[9]=i[9],e[10]=i[10],e[11]=i[11],e[12]=i[12],e[13]=i[13],e[14]=i[14],e[15]=i[15],this}copyPosition(t){const e=this.elements,i=t.elements;return e[12]=i[12],e[13]=i[13],e[14]=i[14],this}setFromMatrix3(t){const e=t.elements;return this.set(e[0],e[3],e[6],0,e[1],e[4],e[7],0,e[2],e[5],e[8],0,0,0,0,1),this}extractBasis(t,e,i){return t.setFromMatrixColumn(this,0),e.setFromMatrixColumn(this,1),i.setFromMatrixColumn(this,2),this}makeBasis(t,e,i){return this.set(t.x,e.x,i.x,0,t.y,e.y,i.y,0,t.z,e.z,i.z,0,0,0,0,1),this}extractRotation(t){const e=this.elements,i=t.elements,s=1/rs.setFromMatrixColumn(t,0).length(),r=1/rs.setFromMatrixColumn(t,1).length(),o=1/rs.setFromMatrixColumn(t,2).length();return e[0]=i[0]*s,e[1]=i[1]*s,e[2]=i[2]*s,e[3]=0,e[4]=i[4]*r,e[5]=i[5]*r,e[6]=i[6]*r,e[7]=0,e[8]=i[8]*o,e[9]=i[9]*o,e[10]=i[10]*o,e[11]=0,e[12]=0,e[13]=0,e[14]=0,e[15]=1,this}makeRotationFromEuler(t){const e=this.elements,i=t.x,s=t.y,r=t.z,o=Math.cos(i),a=Math.sin(i),l=Math.cos(s),c=Math.sin(s),h=Math.cos(r),u=Math.sin(r);if(t.order==="XYZ"){const d=o*h,f=o*u,g=a*h,_=a*u;e[0]=l*h,e[4]=-l*u,e[8]=c,e[1]=f+g*c,e[5]=d-_*c,e[9]=-a*l,e[2]=_-d*c,e[6]=g+f*c,e[10]=o*l}else if(t.order==="YXZ"){const d=l*h,f=l*u,g=c*h,_=c*u;e[0]=d+_*a,e[4]=g*a-f,e[8]=o*c,e[1]=o*u,e[5]=o*h,e[9]=-a,e[2]=f*a-g,e[6]=_+d*a,e[10]=o*l}else if(t.order==="ZXY"){const d=l*h,f=l*u,g=c*h,_=c*u;e[0]=d-_*a,e[4]=-o*u,e[8]=g+f*a,e[1]=f+g*a,e[5]=o*h,e[9]=_-d*a,e[2]=-o*c,e[6]=a,e[10]=o*l}else if(t.order==="ZYX"){const d=o*h,f=o*u,g=a*h,_=a*u;e[0]=l*h,e[4]=g*c-f,e[8]=d*c+_,e[1]=l*u,e[5]=_*c+d,e[9]=f*c-g,e[2]=-c,e[6]=a*l,e[10]=o*l}else if(t.order==="YZX"){const d=o*l,f=o*c,g=a*l,_=a*c;e[0]=l*h,e[4]=_-d*u,e[8]=g*u+f,e[1]=u,e[5]=o*h,e[9]=-a*h,e[2]=-c*h,e[6]=f*u+g,e[10]=d-_*u}else if(t.order==="XZY"){const d=o*l,f=o*c,g=a*l,_=a*c;e[0]=l*h,e[4]=-u,e[8]=c*h,e[1]=d*u+_,e[5]=o*h,e[9]=f*u-g,e[2]=g*u-f,e[6]=a*h,e[10]=_*u+d}return e[3]=0,e[7]=0,e[11]=0,e[12]=0,e[13]=0,e[14]=0,e[15]=1,this}makeRotationFromQuaternion(t){return this.compose(Sg,t,Eg)}lookAt(t,e,i){const s=this.elements;return Ze.subVectors(t,e),Ze.lengthSq()===0&&(Ze.z=1),Ze.normalize(),si.crossVectors(i,Ze),si.lengthSq()===0&&(Math.abs(i.z)===1?Ze.x+=1e-4:Ze.z+=1e-4,Ze.normalize(),si.crossVectors(i,Ze)),si.normalize(),Yr.crossVectors(Ze,si),s[0]=si.x,s[4]=Yr.x,s[8]=Ze.x,s[1]=si.y,s[5]=Yr.y,s[9]=Ze.y,s[2]=si.z,s[6]=Yr.z,s[10]=Ze.z,this}multiply(t){return this.multiplyMatrices(this,t)}premultiply(t){return this.multiplyMatrices(t,this)}multiplyMatrices(t,e){const i=t.elements,s=e.elements,r=this.elements,o=i[0],a=i[4],l=i[8],c=i[12],h=i[1],u=i[5],d=i[9],f=i[13],g=i[2],_=i[6],m=i[10],p=i[14],v=i[3],y=i[7],x=i[11],A=i[15],w=s[0],T=s[4],R=s[8],S=s[12],b=s[1],P=s[5],N=s[9],F=s[13],W=s[2],G=s[6],H=s[10],$=s[14],V=s[3],et=s[7],at=s[11],_t=s[15];return r[0]=o*w+a*b+l*W+c*V,r[4]=o*T+a*P+l*G+c*et,r[8]=o*R+a*N+l*H+c*at,r[12]=o*S+a*F+l*$+c*_t,r[1]=h*w+u*b+d*W+f*V,r[5]=h*T+u*P+d*G+f*et,r[9]=h*R+u*N+d*H+f*at,r[13]=h*S+u*F+d*$+f*_t,r[2]=g*w+_*b+m*W+p*V,r[6]=g*T+_*P+m*G+p*et,r[10]=g*R+_*N+m*H+p*at,r[14]=g*S+_*F+m*$+p*_t,r[3]=v*w+y*b+x*W+A*V,r[7]=v*T+y*P+x*G+A*et,r[11]=v*R+y*N+x*H+A*at,r[15]=v*S+y*F+x*$+A*_t,this}multiplyScalar(t){const e=this.elements;return e[0]*=t,e[4]*=t,e[8]*=t,e[12]*=t,e[1]*=t,e[5]*=t,e[9]*=t,e[13]*=t,e[2]*=t,e[6]*=t,e[10]*=t,e[14]*=t,e[3]*=t,e[7]*=t,e[11]*=t,e[15]*=t,this}determinant(){const t=this.elements,e=t[0],i=t[4],s=t[8],r=t[12],o=t[1],a=t[5],l=t[9],c=t[13],h=t[2],u=t[6],d=t[10],f=t[14],g=t[3],_=t[7],m=t[11],p=t[15];return g*(+r*l*u-s*c*u-r*a*d+i*c*d+s*a*f-i*l*f)+_*(+e*l*f-e*c*d+r*o*d-s*o*f+s*c*h-r*l*h)+m*(+e*c*u-e*a*f-r*o*u+i*o*f+r*a*h-i*c*h)+p*(-s*a*h-e*l*u+e*a*d+s*o*u-i*o*d+i*l*h)}transpose(){const t=this.elements;let e;return e=t[1],t[1]=t[4],t[4]=e,e=t[2],t[2]=t[8],t[8]=e,e=t[6],t[6]=t[9],t[9]=e,e=t[3],t[3]=t[12],t[12]=e,e=t[7],t[7]=t[13],t[13]=e,e=t[11],t[11]=t[14],t[14]=e,this}setPosition(t,e,i){const s=this.elements;return t.isVector3?(s[12]=t.x,s[13]=t.y,s[14]=t.z):(s[12]=t,s[13]=e,s[14]=i),this}invert(){const t=this.elements,e=t[0],i=t[1],s=t[2],r=t[3],o=t[4],a=t[5],l=t[6],c=t[7],h=t[8],u=t[9],d=t[10],f=t[11],g=t[12],_=t[13],m=t[14],p=t[15],v=u*m*c-_*d*c+_*l*f-a*m*f-u*l*p+a*d*p,y=g*d*c-h*m*c-g*l*f+o*m*f+h*l*p-o*d*p,x=h*_*c-g*u*c+g*a*f-o*_*f-h*a*p+o*u*p,A=g*u*l-h*_*l-g*a*d+o*_*d+h*a*m-o*u*m,w=e*v+i*y+s*x+r*A;if(w===0)return this.set(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);const T=1/w;return t[0]=v*T,t[1]=(_*d*r-u*m*r-_*s*f+i*m*f+u*s*p-i*d*p)*T,t[2]=(a*m*r-_*l*r+_*s*c-i*m*c-a*s*p+i*l*p)*T,t[3]=(u*l*r-a*d*r-u*s*c+i*d*c+a*s*f-i*l*f)*T,t[4]=y*T,t[5]=(h*m*r-g*d*r+g*s*f-e*m*f-h*s*p+e*d*p)*T,t[6]=(g*l*r-o*m*r-g*s*c+e*m*c+o*s*p-e*l*p)*T,t[7]=(o*d*r-h*l*r+h*s*c-e*d*c-o*s*f+e*l*f)*T,t[8]=x*T,t[9]=(g*u*r-h*_*r-g*i*f+e*_*f+h*i*p-e*u*p)*T,t[10]=(o*_*r-g*a*r+g*i*c-e*_*c-o*i*p+e*a*p)*T,t[11]=(h*a*r-o*u*r-h*i*c+e*u*c+o*i*f-e*a*f)*T,t[12]=A*T,t[13]=(h*_*s-g*u*s+g*i*d-e*_*d-h*i*m+e*u*m)*T,t[14]=(g*a*s-o*_*s-g*i*l+e*_*l+o*i*m-e*a*m)*T,t[15]=(o*u*s-h*a*s+h*i*l-e*u*l-o*i*d+e*a*d)*T,this}scale(t){const e=this.elements,i=t.x,s=t.y,r=t.z;return e[0]*=i,e[4]*=s,e[8]*=r,e[1]*=i,e[5]*=s,e[9]*=r,e[2]*=i,e[6]*=s,e[10]*=r,e[3]*=i,e[7]*=s,e[11]*=r,this}getMaxScaleOnAxis(){const t=this.elements,e=t[0]*t[0]+t[1]*t[1]+t[2]*t[2],i=t[4]*t[4]+t[5]*t[5]+t[6]*t[6],s=t[8]*t[8]+t[9]*t[9]+t[10]*t[10];return Math.sqrt(Math.max(e,i,s))}makeTranslation(t,e,i){return t.isVector3?this.set(1,0,0,t.x,0,1,0,t.y,0,0,1,t.z,0,0,0,1):this.set(1,0,0,t,0,1,0,e,0,0,1,i,0,0,0,1),this}makeRotationX(t){const e=Math.cos(t),i=Math.sin(t);return this.set(1,0,0,0,0,e,-i,0,0,i,e,0,0,0,0,1),this}makeRotationY(t){const e=Math.cos(t),i=Math.sin(t);return this.set(e,0,i,0,0,1,0,0,-i,0,e,0,0,0,0,1),this}makeRotationZ(t){const e=Math.cos(t),i=Math.sin(t);return this.set(e,-i,0,0,i,e,0,0,0,0,1,0,0,0,0,1),this}makeRotationAxis(t,e){const i=Math.cos(e),s=Math.sin(e),r=1-i,o=t.x,a=t.y,l=t.z,c=r*o,h=r*a;return this.set(c*o+i,c*a-s*l,c*l+s*a,0,c*a+s*l,h*a+i,h*l-s*o,0,c*l-s*a,h*l+s*o,r*l*l+i,0,0,0,0,1),this}makeScale(t,e,i){return this.set(t,0,0,0,0,e,0,0,0,0,i,0,0,0,0,1),this}makeShear(t,e,i,s,r,o){return this.set(1,i,r,0,t,1,o,0,e,s,1,0,0,0,0,1),this}compose(t,e,i){const s=this.elements,r=e._x,o=e._y,a=e._z,l=e._w,c=r+r,h=o+o,u=a+a,d=r*c,f=r*h,g=r*u,_=o*h,m=o*u,p=a*u,v=l*c,y=l*h,x=l*u,A=i.x,w=i.y,T=i.z;return s[0]=(1-(_+p))*A,s[1]=(f+x)*A,s[2]=(g-y)*A,s[3]=0,s[4]=(f-x)*w,s[5]=(1-(d+p))*w,s[6]=(m+v)*w,s[7]=0,s[8]=(g+y)*T,s[9]=(m-v)*T,s[10]=(1-(d+_))*T,s[11]=0,s[12]=t.x,s[13]=t.y,s[14]=t.z,s[15]=1,this}decompose(t,e,i){const s=this.elements;let r=rs.set(s[0],s[1],s[2]).length();const o=rs.set(s[4],s[5],s[6]).length(),a=rs.set(s[8],s[9],s[10]).length();this.determinant()<0&&(r=-r),t.x=s[12],t.y=s[13],t.z=s[14],fn.copy(this);const c=1/r,h=1/o,u=1/a;return fn.elements[0]*=c,fn.elements[1]*=c,fn.elements[2]*=c,fn.elements[4]*=h,fn.elements[5]*=h,fn.elements[6]*=h,fn.elements[8]*=u,fn.elements[9]*=u,fn.elements[10]*=u,e.setFromRotationMatrix(fn),i.x=r,i.y=o,i.z=a,this}makePerspective(t,e,i,s,r,o,a=$n){const l=this.elements,c=2*r/(e-t),h=2*r/(i-s),u=(e+t)/(e-t),d=(i+s)/(i-s);let f,g;if(a===$n)f=-(o+r)/(o-r),g=-2*o*r/(o-r);else if(a===Zo)f=-o/(o-r),g=-o*r/(o-r);else throw new Error("THREE.Matrix4.makePerspective(): Invalid coordinate system: "+a);return l[0]=c,l[4]=0,l[8]=u,l[12]=0,l[1]=0,l[5]=h,l[9]=d,l[13]=0,l[2]=0,l[6]=0,l[10]=f,l[14]=g,l[3]=0,l[7]=0,l[11]=-1,l[15]=0,this}makeOrthographic(t,e,i,s,r,o,a=$n){const l=this.elements,c=1/(e-t),h=1/(i-s),u=1/(o-r),d=(e+t)*c,f=(i+s)*h;let g,_;if(a===$n)g=(o+r)*u,_=-2*u;else if(a===Zo)g=r*u,_=-1*u;else throw new Error("THREE.Matrix4.makeOrthographic(): Invalid coordinate system: "+a);return l[0]=2*c,l[4]=0,l[8]=0,l[12]=-d,l[1]=0,l[5]=2*h,l[9]=0,l[13]=-f,l[2]=0,l[6]=0,l[10]=_,l[14]=-g,l[3]=0,l[7]=0,l[11]=0,l[15]=1,this}equals(t){const e=this.elements,i=t.elements;for(let s=0;s<16;s++)if(e[s]!==i[s])return!1;return!0}fromArray(t,e=0){for(let i=0;i<16;i++)this.elements[i]=t[i+e];return this}toArray(t=[],e=0){const i=this.elements;return t[e]=i[0],t[e+1]=i[1],t[e+2]=i[2],t[e+3]=i[3],t[e+4]=i[4],t[e+5]=i[5],t[e+6]=i[6],t[e+7]=i[7],t[e+8]=i[8],t[e+9]=i[9],t[e+10]=i[10],t[e+11]=i[11],t[e+12]=i[12],t[e+13]=i[13],t[e+14]=i[14],t[e+15]=i[15],t}}const rs=new L,fn=new oe,Sg=new L(0,0,0),Eg=new L(1,1,1),si=new L,Yr=new L,Ze=new L,Uh=new oe,Nh=new $i;class Pn{constructor(t=0,e=0,i=0,s=Pn.DEFAULT_ORDER){this.isEuler=!0,this._x=t,this._y=e,this._z=i,this._order=s}get x(){return this._x}set x(t){this._x=t,this._onChangeCallback()}get y(){return this._y}set y(t){this._y=t,this._onChangeCallback()}get z(){return this._z}set z(t){this._z=t,this._onChangeCallback()}get order(){return this._order}set order(t){this._order=t,this._onChangeCallback()}set(t,e,i,s=this._order){return this._x=t,this._y=e,this._z=i,this._order=s,this._onChangeCallback(),this}clone(){return new this.constructor(this._x,this._y,this._z,this._order)}copy(t){return this._x=t._x,this._y=t._y,this._z=t._z,this._order=t._order,this._onChangeCallback(),this}setFromRotationMatrix(t,e=this._order,i=!0){const s=t.elements,r=s[0],o=s[4],a=s[8],l=s[1],c=s[5],h=s[9],u=s[2],d=s[6],f=s[10];switch(e){case"XYZ":this._y=Math.asin(Ge(a,-1,1)),Math.abs(a)<.9999999?(this._x=Math.atan2(-h,f),this._z=Math.atan2(-o,r)):(this._x=Math.atan2(d,c),this._z=0);break;case"YXZ":this._x=Math.asin(-Ge(h,-1,1)),Math.abs(h)<.9999999?(this._y=Math.atan2(a,f),this._z=Math.atan2(l,c)):(this._y=Math.atan2(-u,r),this._z=0);break;case"ZXY":this._x=Math.asin(Ge(d,-1,1)),Math.abs(d)<.9999999?(this._y=Math.atan2(-u,f),this._z=Math.atan2(-o,c)):(this._y=0,this._z=Math.atan2(l,r));break;case"ZYX":this._y=Math.asin(-Ge(u,-1,1)),Math.abs(u)<.9999999?(this._x=Math.atan2(d,f),this._z=Math.atan2(l,r)):(this._x=0,this._z=Math.atan2(-o,c));break;case"YZX":this._z=Math.asin(Ge(l,-1,1)),Math.abs(l)<.9999999?(this._x=Math.atan2(-h,c),this._y=Math.atan2(-u,r)):(this._x=0,this._y=Math.atan2(a,f));break;case"XZY":this._z=Math.asin(-Ge(o,-1,1)),Math.abs(o)<.9999999?(this._x=Math.atan2(d,c),this._y=Math.atan2(a,r)):(this._x=Math.atan2(-h,f),this._y=0);break;default:console.warn("THREE.Euler: .setFromRotationMatrix() encountered an unknown order: "+e)}return this._order=e,i===!0&&this._onChangeCallback(),this}setFromQuaternion(t,e,i){return Uh.makeRotationFromQuaternion(t),this.setFromRotationMatrix(Uh,e,i)}setFromVector3(t,e=this._order){return this.set(t.x,t.y,t.z,e)}reorder(t){return Nh.setFromEuler(this),this.setFromQuaternion(Nh,t)}equals(t){return t._x===this._x&&t._y===this._y&&t._z===this._z&&t._order===this._order}fromArray(t){return this._x=t[0],this._y=t[1],this._z=t[2],t[3]!==void 0&&(this._order=t[3]),this._onChangeCallback(),this}toArray(t=[],e=0){return t[e]=this._x,t[e+1]=this._y,t[e+2]=this._z,t[e+3]=this._order,t}_onChange(t){return this._onChangeCallback=t,this}_onChangeCallback(){}*[Symbol.iterator](){yield this._x,yield this._y,yield this._z,yield this._order}}Pn.DEFAULT_ORDER="XYZ";class Bc{constructor(){this.mask=1}set(t){this.mask=(1<<t|0)>>>0}enable(t){this.mask|=1<<t|0}enableAll(){this.mask=-1}toggle(t){this.mask^=1<<t|0}disable(t){this.mask&=~(1<<t|0)}disableAll(){this.mask=0}test(t){return(this.mask&t.mask)!==0}isEnabled(t){return(this.mask&(1<<t|0))!==0}}let Tg=0;const Fh=new L,os=new $i,Un=new oe,qr=new L,Ys=new L,wg=new L,Ag=new $i,kh=new L(1,0,0),Bh=new L(0,1,0),zh=new L(0,0,1),Hh={type:"added"},Cg={type:"removed"},as={type:"childadded",child:null},ka={type:"childremoved",child:null};class Ce extends Ki{constructor(){super(),this.isObject3D=!0,Object.defineProperty(this,"id",{value:Tg++}),this.uuid=mi(),this.name="",this.type="Object3D",this.parent=null,this.children=[],this.up=Ce.DEFAULT_UP.clone();const t=new L,e=new Pn,i=new $i,s=new L(1,1,1);function r(){i.setFromEuler(e,!1)}function o(){e.setFromQuaternion(i,void 0,!1)}e._onChange(r),i._onChange(o),Object.defineProperties(this,{position:{configurable:!0,enumerable:!0,value:t},rotation:{configurable:!0,enumerable:!0,value:e},quaternion:{configurable:!0,enumerable:!0,value:i},scale:{configurable:!0,enumerable:!0,value:s},modelViewMatrix:{value:new oe},normalMatrix:{value:new Ft}}),this.matrix=new oe,this.matrixWorld=new oe,this.matrixAutoUpdate=Ce.DEFAULT_MATRIX_AUTO_UPDATE,this.matrixWorldAutoUpdate=Ce.DEFAULT_MATRIX_WORLD_AUTO_UPDATE,this.matrixWorldNeedsUpdate=!1,this.layers=new Bc,this.visible=!0,this.castShadow=!1,this.receiveShadow=!1,this.frustumCulled=!0,this.renderOrder=0,this.animations=[],this.userData={}}onBeforeShadow(){}onAfterShadow(){}onBeforeRender(){}onAfterRender(){}applyMatrix4(t){this.matrixAutoUpdate&&this.updateMatrix(),this.matrix.premultiply(t),this.matrix.decompose(this.position,this.quaternion,this.scale)}applyQuaternion(t){return this.quaternion.premultiply(t),this}setRotationFromAxisAngle(t,e){this.quaternion.setFromAxisAngle(t,e)}setRotationFromEuler(t){this.quaternion.setFromEuler(t,!0)}setRotationFromMatrix(t){this.quaternion.setFromRotationMatrix(t)}setRotationFromQuaternion(t){this.quaternion.copy(t)}rotateOnAxis(t,e){return os.setFromAxisAngle(t,e),this.quaternion.multiply(os),this}rotateOnWorldAxis(t,e){return os.setFromAxisAngle(t,e),this.quaternion.premultiply(os),this}rotateX(t){return this.rotateOnAxis(kh,t)}rotateY(t){return this.rotateOnAxis(Bh,t)}rotateZ(t){return this.rotateOnAxis(zh,t)}translateOnAxis(t,e){return Fh.copy(t).applyQuaternion(this.quaternion),this.position.add(Fh.multiplyScalar(e)),this}translateX(t){return this.translateOnAxis(kh,t)}translateY(t){return this.translateOnAxis(Bh,t)}translateZ(t){return this.translateOnAxis(zh,t)}localToWorld(t){return this.updateWorldMatrix(!0,!1),t.applyMatrix4(this.matrixWorld)}worldToLocal(t){return this.updateWorldMatrix(!0,!1),t.applyMatrix4(Un.copy(this.matrixWorld).invert())}lookAt(t,e,i){t.isVector3?qr.copy(t):qr.set(t,e,i);const s=this.parent;this.updateWorldMatrix(!0,!1),Ys.setFromMatrixPosition(this.matrixWorld),this.isCamera||this.isLight?Un.lookAt(Ys,qr,this.up):Un.lookAt(qr,Ys,this.up),this.quaternion.setFromRotationMatrix(Un),s&&(Un.extractRotation(s.matrixWorld),os.setFromRotationMatrix(Un),this.quaternion.premultiply(os.invert()))}add(t){if(arguments.length>1){for(let e=0;e<arguments.length;e++)this.add(arguments[e]);return this}return t===this?(console.error("THREE.Object3D.add: object can't be added as a child of itself.",t),this):(t&&t.isObject3D?(t.removeFromParent(),t.parent=this,this.children.push(t),t.dispatchEvent(Hh),as.child=t,this.dispatchEvent(as),as.child=null):console.error("THREE.Object3D.add: object not an instance of THREE.Object3D.",t),this)}remove(t){if(arguments.length>1){for(let i=0;i<arguments.length;i++)this.remove(arguments[i]);return this}const e=this.children.indexOf(t);return e!==-1&&(t.parent=null,this.children.splice(e,1),t.dispatchEvent(Cg),ka.child=t,this.dispatchEvent(ka),ka.child=null),this}removeFromParent(){const t=this.parent;return t!==null&&t.remove(this),this}clear(){return this.remove(...this.children)}attach(t){return this.updateWorldMatrix(!0,!1),Un.copy(this.matrixWorld).invert(),t.parent!==null&&(t.parent.updateWorldMatrix(!0,!1),Un.multiply(t.parent.matrixWorld)),t.applyMatrix4(Un),t.removeFromParent(),t.parent=this,this.children.push(t),t.updateWorldMatrix(!1,!0),t.dispatchEvent(Hh),as.child=t,this.dispatchEvent(as),as.child=null,this}getObjectById(t){return this.getObjectByProperty("id",t)}getObjectByName(t){return this.getObjectByProperty("name",t)}getObjectByProperty(t,e){if(this[t]===e)return this;for(let i=0,s=this.children.length;i<s;i++){const o=this.children[i].getObjectByProperty(t,e);if(o!==void 0)return o}}getObjectsByProperty(t,e,i=[]){this[t]===e&&i.push(this);const s=this.children;for(let r=0,o=s.length;r<o;r++)s[r].getObjectsByProperty(t,e,i);return i}getWorldPosition(t){return this.updateWorldMatrix(!0,!1),t.setFromMatrixPosition(this.matrixWorld)}getWorldQuaternion(t){return this.updateWorldMatrix(!0,!1),this.matrixWorld.decompose(Ys,t,wg),t}getWorldScale(t){return this.updateWorldMatrix(!0,!1),this.matrixWorld.decompose(Ys,Ag,t),t}getWorldDirection(t){this.updateWorldMatrix(!0,!1);const e=this.matrixWorld.elements;return t.set(e[8],e[9],e[10]).normalize()}raycast(){}traverse(t){t(this);const e=this.children;for(let i=0,s=e.length;i<s;i++)e[i].traverse(t)}traverseVisible(t){if(this.visible===!1)return;t(this);const e=this.children;for(let i=0,s=e.length;i<s;i++)e[i].traverseVisible(t)}traverseAncestors(t){const e=this.parent;e!==null&&(t(e),e.traverseAncestors(t))}updateMatrix(){this.matrix.compose(this.position,this.quaternion,this.scale),this.matrixWorldNeedsUpdate=!0}updateMatrixWorld(t){this.matrixAutoUpdate&&this.updateMatrix(),(this.matrixWorldNeedsUpdate||t)&&(this.matrixWorldAutoUpdate===!0&&(this.parent===null?this.matrixWorld.copy(this.matrix):this.matrixWorld.multiplyMatrices(this.parent.matrixWorld,this.matrix)),this.matrixWorldNeedsUpdate=!1,t=!0);const e=this.children;for(let i=0,s=e.length;i<s;i++)e[i].updateMatrixWorld(t)}updateWorldMatrix(t,e){const i=this.parent;if(t===!0&&i!==null&&i.updateWorldMatrix(!0,!1),this.matrixAutoUpdate&&this.updateMatrix(),this.matrixWorldAutoUpdate===!0&&(this.parent===null?this.matrixWorld.copy(this.matrix):this.matrixWorld.multiplyMatrices(this.parent.matrixWorld,this.matrix)),e===!0){const s=this.children;for(let r=0,o=s.length;r<o;r++)s[r].updateWorldMatrix(!1,!0)}}toJSON(t){const e=t===void 0||typeof t=="string",i={};e&&(t={geometries:{},materials:{},textures:{},images:{},shapes:{},skeletons:{},animations:{},nodes:{}},i.metadata={version:4.6,type:"Object",generator:"Object3D.toJSON"});const s={};s.uuid=this.uuid,s.type=this.type,this.name!==""&&(s.name=this.name),this.castShadow===!0&&(s.castShadow=!0),this.receiveShadow===!0&&(s.receiveShadow=!0),this.visible===!1&&(s.visible=!1),this.frustumCulled===!1&&(s.frustumCulled=!1),this.renderOrder!==0&&(s.renderOrder=this.renderOrder),Object.keys(this.userData).length>0&&(s.userData=this.userData),s.layers=this.layers.mask,s.matrix=this.matrix.toArray(),s.up=this.up.toArray(),this.matrixAutoUpdate===!1&&(s.matrixAutoUpdate=!1),this.isInstancedMesh&&(s.type="InstancedMesh",s.count=this.count,s.instanceMatrix=this.instanceMatrix.toJSON(),this.instanceColor!==null&&(s.instanceColor=this.instanceColor.toJSON())),this.isBatchedMesh&&(s.type="BatchedMesh",s.perObjectFrustumCulled=this.perObjectFrustumCulled,s.sortObjects=this.sortObjects,s.drawRanges=this._drawRanges,s.reservedRanges=this._reservedRanges,s.visibility=this._visibility,s.active=this._active,s.bounds=this._bounds.map(a=>({boxInitialized:a.boxInitialized,boxMin:a.box.min.toArray(),boxMax:a.box.max.toArray(),sphereInitialized:a.sphereInitialized,sphereRadius:a.sphere.radius,sphereCenter:a.sphere.center.toArray()})),s.maxInstanceCount=this._maxInstanceCount,s.maxVertexCount=this._maxVertexCount,s.maxIndexCount=this._maxIndexCount,s.geometryInitialized=this._geometryInitialized,s.geometryCount=this._geometryCount,s.matricesTexture=this._matricesTexture.toJSON(t),this._colorsTexture!==null&&(s.colorsTexture=this._colorsTexture.toJSON(t)),this.boundingSphere!==null&&(s.boundingSphere={center:s.boundingSphere.center.toArray(),radius:s.boundingSphere.radius}),this.boundingBox!==null&&(s.boundingBox={min:s.boundingBox.min.toArray(),max:s.boundingBox.max.toArray()}));function r(a,l){return a[l.uuid]===void 0&&(a[l.uuid]=l.toJSON(t)),l.uuid}if(this.isScene)this.background&&(this.background.isColor?s.background=this.background.toJSON():this.background.isTexture&&(s.background=this.background.toJSON(t).uuid)),this.environment&&this.environment.isTexture&&this.environment.isRenderTargetTexture!==!0&&(s.environment=this.environment.toJSON(t).uuid);else if(this.isMesh||this.isLine||this.isPoints){s.geometry=r(t.geometries,this.geometry);const a=this.geometry.parameters;if(a!==void 0&&a.shapes!==void 0){const l=a.shapes;if(Array.isArray(l))for(let c=0,h=l.length;c<h;c++){const u=l[c];r(t.shapes,u)}else r(t.shapes,l)}}if(this.isSkinnedMesh&&(s.bindMode=this.bindMode,s.bindMatrix=this.bindMatrix.toArray(),this.skeleton!==void 0&&(r(t.skeletons,this.skeleton),s.skeleton=this.skeleton.uuid)),this.material!==void 0)if(Array.isArray(this.material)){const a=[];for(let l=0,c=this.material.length;l<c;l++)a.push(r(t.materials,this.material[l]));s.material=a}else s.material=r(t.materials,this.material);if(this.children.length>0){s.children=[];for(let a=0;a<this.children.length;a++)s.children.push(this.children[a].toJSON(t).object)}if(this.animations.length>0){s.animations=[];for(let a=0;a<this.animations.length;a++){const l=this.animations[a];s.animations.push(r(t.animations,l))}}if(e){const a=o(t.geometries),l=o(t.materials),c=o(t.textures),h=o(t.images),u=o(t.shapes),d=o(t.skeletons),f=o(t.animations),g=o(t.nodes);a.length>0&&(i.geometries=a),l.length>0&&(i.materials=l),c.length>0&&(i.textures=c),h.length>0&&(i.images=h),u.length>0&&(i.shapes=u),d.length>0&&(i.skeletons=d),f.length>0&&(i.animations=f),g.length>0&&(i.nodes=g)}return i.object=s,i;function o(a){const l=[];for(const c in a){const h=a[c];delete h.metadata,l.push(h)}return l}}clone(t){return new this.constructor().copy(this,t)}copy(t,e=!0){if(this.name=t.name,this.up.copy(t.up),this.position.copy(t.position),this.rotation.order=t.rotation.order,this.quaternion.copy(t.quaternion),this.scale.copy(t.scale),this.matrix.copy(t.matrix),this.matrixWorld.copy(t.matrixWorld),this.matrixAutoUpdate=t.matrixAutoUpdate,this.matrixWorldAutoUpdate=t.matrixWorldAutoUpdate,this.matrixWorldNeedsUpdate=t.matrixWorldNeedsUpdate,this.layers.mask=t.layers.mask,this.visible=t.visible,this.castShadow=t.castShadow,this.receiveShadow=t.receiveShadow,this.frustumCulled=t.frustumCulled,this.renderOrder=t.renderOrder,this.animations=t.animations.slice(),this.userData=JSON.parse(JSON.stringify(t.userData)),e===!0)for(let i=0;i<t.children.length;i++){const s=t.children[i];this.add(s.clone())}return this}}Ce.DEFAULT_UP=new L(0,1,0);Ce.DEFAULT_MATRIX_AUTO_UPDATE=!0;Ce.DEFAULT_MATRIX_WORLD_AUTO_UPDATE=!0;const pn=new L,Nn=new L,Ba=new L,Fn=new L,ls=new L,cs=new L,Vh=new L,za=new L,Ha=new L,Va=new L,Ga=new be,Wa=new be,Xa=new be;class cn{constructor(t=new L,e=new L,i=new L){this.a=t,this.b=e,this.c=i}static getNormal(t,e,i,s){s.subVectors(i,e),pn.subVectors(t,e),s.cross(pn);const r=s.lengthSq();return r>0?s.multiplyScalar(1/Math.sqrt(r)):s.set(0,0,0)}static getBarycoord(t,e,i,s,r){pn.subVectors(s,e),Nn.subVectors(i,e),Ba.subVectors(t,e);const o=pn.dot(pn),a=pn.dot(Nn),l=pn.dot(Ba),c=Nn.dot(Nn),h=Nn.dot(Ba),u=o*c-a*a;if(u===0)return r.set(0,0,0),null;const d=1/u,f=(c*l-a*h)*d,g=(o*h-a*l)*d;return r.set(1-f-g,g,f)}static containsPoint(t,e,i,s){return this.getBarycoord(t,e,i,s,Fn)===null?!1:Fn.x>=0&&Fn.y>=0&&Fn.x+Fn.y<=1}static getInterpolation(t,e,i,s,r,o,a,l){return this.getBarycoord(t,e,i,s,Fn)===null?(l.x=0,l.y=0,"z"in l&&(l.z=0),"w"in l&&(l.w=0),null):(l.setScalar(0),l.addScaledVector(r,Fn.x),l.addScaledVector(o,Fn.y),l.addScaledVector(a,Fn.z),l)}static getInterpolatedAttribute(t,e,i,s,r,o){return Ga.setScalar(0),Wa.setScalar(0),Xa.setScalar(0),Ga.fromBufferAttribute(t,e),Wa.fromBufferAttribute(t,i),Xa.fromBufferAttribute(t,s),o.setScalar(0),o.addScaledVector(Ga,r.x),o.addScaledVector(Wa,r.y),o.addScaledVector(Xa,r.z),o}static isFrontFacing(t,e,i,s){return pn.subVectors(i,e),Nn.subVectors(t,e),pn.cross(Nn).dot(s)<0}set(t,e,i){return this.a.copy(t),this.b.copy(e),this.c.copy(i),this}setFromPointsAndIndices(t,e,i,s){return this.a.copy(t[e]),this.b.copy(t[i]),this.c.copy(t[s]),this}setFromAttributeAndIndices(t,e,i,s){return this.a.fromBufferAttribute(t,e),this.b.fromBufferAttribute(t,i),this.c.fromBufferAttribute(t,s),this}clone(){return new this.constructor().copy(this)}copy(t){return this.a.copy(t.a),this.b.copy(t.b),this.c.copy(t.c),this}getArea(){return pn.subVectors(this.c,this.b),Nn.subVectors(this.a,this.b),pn.cross(Nn).length()*.5}getMidpoint(t){return t.addVectors(this.a,this.b).add(this.c).multiplyScalar(1/3)}getNormal(t){return cn.getNormal(this.a,this.b,this.c,t)}getPlane(t){return t.setFromCoplanarPoints(this.a,this.b,this.c)}getBarycoord(t,e){return cn.getBarycoord(t,this.a,this.b,this.c,e)}getInterpolation(t,e,i,s,r){return cn.getInterpolation(t,this.a,this.b,this.c,e,i,s,r)}containsPoint(t){return cn.containsPoint(t,this.a,this.b,this.c)}isFrontFacing(t){return cn.isFrontFacing(this.a,this.b,this.c,t)}intersectsBox(t){return t.intersectsTriangle(this)}closestPointToPoint(t,e){const i=this.a,s=this.b,r=this.c;let o,a;ls.subVectors(s,i),cs.subVectors(r,i),za.subVectors(t,i);const l=ls.dot(za),c=cs.dot(za);if(l<=0&&c<=0)return e.copy(i);Ha.subVectors(t,s);const h=ls.dot(Ha),u=cs.dot(Ha);if(h>=0&&u<=h)return e.copy(s);const d=l*u-h*c;if(d<=0&&l>=0&&h<=0)return o=l/(l-h),e.copy(i).addScaledVector(ls,o);Va.subVectors(t,r);const f=ls.dot(Va),g=cs.dot(Va);if(g>=0&&f<=g)return e.copy(r);const _=f*c-l*g;if(_<=0&&c>=0&&g<=0)return a=c/(c-g),e.copy(i).addScaledVector(cs,a);const m=h*g-f*u;if(m<=0&&u-h>=0&&f-g>=0)return Vh.subVectors(r,s),a=(u-h)/(u-h+(f-g)),e.copy(s).addScaledVector(Vh,a);const p=1/(m+_+d);return o=_*p,a=d*p,e.copy(i).addScaledVector(ls,o).addScaledVector(cs,a)}equals(t){return t.a.equals(this.a)&&t.b.equals(this.b)&&t.c.equals(this.c)}}const Uf={aliceblue:15792383,antiquewhite:16444375,aqua:65535,aquamarine:8388564,azure:15794175,beige:16119260,bisque:16770244,black:0,blanchedalmond:16772045,blue:255,blueviolet:9055202,brown:10824234,burlywood:14596231,cadetblue:6266528,chartreuse:8388352,chocolate:13789470,coral:16744272,cornflowerblue:6591981,cornsilk:16775388,crimson:14423100,cyan:65535,darkblue:139,darkcyan:35723,darkgoldenrod:12092939,darkgray:11119017,darkgreen:25600,darkgrey:11119017,darkkhaki:12433259,darkmagenta:9109643,darkolivegreen:5597999,darkorange:16747520,darkorchid:10040012,darkred:9109504,darksalmon:15308410,darkseagreen:9419919,darkslateblue:4734347,darkslategray:3100495,darkslategrey:3100495,darkturquoise:52945,darkviolet:9699539,deeppink:16716947,deepskyblue:49151,dimgray:6908265,dimgrey:6908265,dodgerblue:2003199,firebrick:11674146,floralwhite:16775920,forestgreen:2263842,fuchsia:16711935,gainsboro:14474460,ghostwhite:16316671,gold:16766720,goldenrod:14329120,gray:8421504,green:32768,greenyellow:11403055,grey:8421504,honeydew:15794160,hotpink:16738740,indianred:13458524,indigo:4915330,ivory:16777200,khaki:15787660,lavender:15132410,lavenderblush:16773365,lawngreen:8190976,lemonchiffon:16775885,lightblue:11393254,lightcoral:15761536,lightcyan:14745599,lightgoldenrodyellow:16448210,lightgray:13882323,lightgreen:9498256,lightgrey:13882323,lightpink:16758465,lightsalmon:16752762,lightseagreen:2142890,lightskyblue:8900346,lightslategray:7833753,lightslategrey:7833753,lightsteelblue:11584734,lightyellow:16777184,lime:65280,limegreen:3329330,linen:16445670,magenta:16711935,maroon:8388608,mediumaquamarine:6737322,mediumblue:205,mediumorchid:12211667,mediumpurple:9662683,mediumseagreen:3978097,mediumslateblue:8087790,mediumspringgreen:64154,mediumturquoise:4772300,mediumvioletred:13047173,midnightblue:1644912,mintcream:16121850,mistyrose:16770273,moccasin:16770229,navajowhite:16768685,navy:128,oldlace:16643558,olive:8421376,olivedrab:7048739,orange:16753920,orangered:16729344,orchid:14315734,palegoldenrod:15657130,palegreen:10025880,paleturquoise:11529966,palevioletred:14381203,papayawhip:16773077,peachpuff:16767673,peru:13468991,pink:16761035,plum:14524637,powderblue:11591910,purple:8388736,rebeccapurple:6697881,red:16711680,rosybrown:12357519,royalblue:4286945,saddlebrown:9127187,salmon:16416882,sandybrown:16032864,seagreen:3050327,seashell:16774638,sienna:10506797,silver:12632256,skyblue:8900331,slateblue:6970061,slategray:7372944,slategrey:7372944,snow:16775930,springgreen:65407,steelblue:4620980,tan:13808780,teal:32896,thistle:14204888,tomato:16737095,turquoise:4251856,violet:15631086,wheat:16113331,white:16777215,whitesmoke:16119285,yellow:16776960,yellowgreen:10145074},ri={h:0,s:0,l:0},Kr={h:0,s:0,l:0};function ja(n,t,e){return e<0&&(e+=1),e>1&&(e-=1),e<1/6?n+(t-n)*6*e:e<1/2?t:e<2/3?n+(t-n)*6*(2/3-e):n}let Ht=class{constructor(t,e,i){return this.isColor=!0,this.r=1,this.g=1,this.b=1,this.set(t,e,i)}set(t,e,i){if(e===void 0&&i===void 0){const s=t;s&&s.isColor?this.copy(s):typeof s=="number"?this.setHex(s):typeof s=="string"&&this.setStyle(s)}else this.setRGB(t,e,i);return this}setScalar(t){return this.r=t,this.g=t,this.b=t,this}setHex(t,e=an){return t=Math.floor(t),this.r=(t>>16&255)/255,this.g=(t>>8&255)/255,this.b=(t&255)/255,Kt.toWorkingColorSpace(this,e),this}setRGB(t,e,i,s=Kt.workingColorSpace){return this.r=t,this.g=e,this.b=i,Kt.toWorkingColorSpace(this,s),this}setHSL(t,e,i,s=Kt.workingColorSpace){if(t=ug(t,1),e=Ge(e,0,1),i=Ge(i,0,1),e===0)this.r=this.g=this.b=i;else{const r=i<=.5?i*(1+e):i+e-i*e,o=2*i-r;this.r=ja(o,r,t+1/3),this.g=ja(o,r,t),this.b=ja(o,r,t-1/3)}return Kt.toWorkingColorSpace(this,s),this}setStyle(t,e=an){function i(r){r!==void 0&&parseFloat(r)<1&&console.warn("THREE.Color: Alpha component of "+t+" will be ignored.")}let s;if(s=/^(\w+)\(([^\)]*)\)/.exec(t)){let r;const o=s[1],a=s[2];switch(o){case"rgb":case"rgba":if(r=/^\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(a))return i(r[4]),this.setRGB(Math.min(255,parseInt(r[1],10))/255,Math.min(255,parseInt(r[2],10))/255,Math.min(255,parseInt(r[3],10))/255,e);if(r=/^\s*(\d+)\%\s*,\s*(\d+)\%\s*,\s*(\d+)\%\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(a))return i(r[4]),this.setRGB(Math.min(100,parseInt(r[1],10))/100,Math.min(100,parseInt(r[2],10))/100,Math.min(100,parseInt(r[3],10))/100,e);break;case"hsl":case"hsla":if(r=/^\s*(\d*\.?\d+)\s*,\s*(\d*\.?\d+)\%\s*,\s*(\d*\.?\d+)\%\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(a))return i(r[4]),this.setHSL(parseFloat(r[1])/360,parseFloat(r[2])/100,parseFloat(r[3])/100,e);break;default:console.warn("THREE.Color: Unknown color model "+t)}}else if(s=/^\#([A-Fa-f\d]+)$/.exec(t)){const r=s[1],o=r.length;if(o===3)return this.setRGB(parseInt(r.charAt(0),16)/15,parseInt(r.charAt(1),16)/15,parseInt(r.charAt(2),16)/15,e);if(o===6)return this.setHex(parseInt(r,16),e);console.warn("THREE.Color: Invalid hex color "+t)}else if(t&&t.length>0)return this.setColorName(t,e);return this}setColorName(t,e=an){const i=Uf[t.toLowerCase()];return i!==void 0?this.setHex(i,e):console.warn("THREE.Color: Unknown color "+t),this}clone(){return new this.constructor(this.r,this.g,this.b)}copy(t){return this.r=t.r,this.g=t.g,this.b=t.b,this}copySRGBToLinear(t){return this.r=Zn(t.r),this.g=Zn(t.g),this.b=Zn(t.b),this}copyLinearToSRGB(t){return this.r=Cs(t.r),this.g=Cs(t.g),this.b=Cs(t.b),this}convertSRGBToLinear(){return this.copySRGBToLinear(this),this}convertLinearToSRGB(){return this.copyLinearToSRGB(this),this}getHex(t=an){return Kt.fromWorkingColorSpace(Fe.copy(this),t),Math.round(Ge(Fe.r*255,0,255))*65536+Math.round(Ge(Fe.g*255,0,255))*256+Math.round(Ge(Fe.b*255,0,255))}getHexString(t=an){return("000000"+this.getHex(t).toString(16)).slice(-6)}getHSL(t,e=Kt.workingColorSpace){Kt.fromWorkingColorSpace(Fe.copy(this),e);const i=Fe.r,s=Fe.g,r=Fe.b,o=Math.max(i,s,r),a=Math.min(i,s,r);let l,c;const h=(a+o)/2;if(a===o)l=0,c=0;else{const u=o-a;switch(c=h<=.5?u/(o+a):u/(2-o-a),o){case i:l=(s-r)/u+(s<r?6:0);break;case s:l=(r-i)/u+2;break;case r:l=(i-s)/u+4;break}l/=6}return t.h=l,t.s=c,t.l=h,t}getRGB(t,e=Kt.workingColorSpace){return Kt.fromWorkingColorSpace(Fe.copy(this),e),t.r=Fe.r,t.g=Fe.g,t.b=Fe.b,t}getStyle(t=an){Kt.fromWorkingColorSpace(Fe.copy(this),t);const e=Fe.r,i=Fe.g,s=Fe.b;return t!==an?`color(${t} ${e.toFixed(3)} ${i.toFixed(3)} ${s.toFixed(3)})`:`rgb(${Math.round(e*255)},${Math.round(i*255)},${Math.round(s*255)})`}offsetHSL(t,e,i){return this.getHSL(ri),this.setHSL(ri.h+t,ri.s+e,ri.l+i)}add(t){return this.r+=t.r,this.g+=t.g,this.b+=t.b,this}addColors(t,e){return this.r=t.r+e.r,this.g=t.g+e.g,this.b=t.b+e.b,this}addScalar(t){return this.r+=t,this.g+=t,this.b+=t,this}sub(t){return this.r=Math.max(0,this.r-t.r),this.g=Math.max(0,this.g-t.g),this.b=Math.max(0,this.b-t.b),this}multiply(t){return this.r*=t.r,this.g*=t.g,this.b*=t.b,this}multiplyScalar(t){return this.r*=t,this.g*=t,this.b*=t,this}lerp(t,e){return this.r+=(t.r-this.r)*e,this.g+=(t.g-this.g)*e,this.b+=(t.b-this.b)*e,this}lerpColors(t,e,i){return this.r=t.r+(e.r-t.r)*i,this.g=t.g+(e.g-t.g)*i,this.b=t.b+(e.b-t.b)*i,this}lerpHSL(t,e){this.getHSL(ri),t.getHSL(Kr);const i=Ra(ri.h,Kr.h,e),s=Ra(ri.s,Kr.s,e),r=Ra(ri.l,Kr.l,e);return this.setHSL(i,s,r),this}setFromVector3(t){return this.r=t.x,this.g=t.y,this.b=t.z,this}applyMatrix3(t){const e=this.r,i=this.g,s=this.b,r=t.elements;return this.r=r[0]*e+r[3]*i+r[6]*s,this.g=r[1]*e+r[4]*i+r[7]*s,this.b=r[2]*e+r[5]*i+r[8]*s,this}equals(t){return t.r===this.r&&t.g===this.g&&t.b===this.b}fromArray(t,e=0){return this.r=t[e],this.g=t[e+1],this.b=t[e+2],this}toArray(t=[],e=0){return t[e]=this.r,t[e+1]=this.g,t[e+2]=this.b,t}fromBufferAttribute(t,e){return this.r=t.getX(e),this.g=t.getY(e),this.b=t.getZ(e),this}toJSON(){return this.getHex()}*[Symbol.iterator](){yield this.r,yield this.g,yield this.b}};const Fe=new Ht;Ht.NAMES=Uf;let Rg=0;class yi extends Ki{static get type(){return"Material"}get type(){return this.constructor.type}set type(t){}constructor(){super(),this.isMaterial=!0,Object.defineProperty(this,"id",{value:Rg++}),this.uuid=mi(),this.name="",this.blending=ws,this.side=_i,this.vertexColors=!1,this.opacity=1,this.transparent=!1,this.alphaHash=!1,this.blendSrc=Ml,this.blendDst=Sl,this.blendEquation=Fi,this.blendSrcAlpha=null,this.blendDstAlpha=null,this.blendEquationAlpha=null,this.blendColor=new Ht(0,0,0),this.blendAlpha=0,this.depthFunc=Ds,this.depthTest=!0,this.depthWrite=!0,this.stencilWriteMask=255,this.stencilFunc=wh,this.stencilRef=0,this.stencilFuncMask=255,this.stencilFail=ts,this.stencilZFail=ts,this.stencilZPass=ts,this.stencilWrite=!1,this.clippingPlanes=null,this.clipIntersection=!1,this.clipShadows=!1,this.shadowSide=null,this.colorWrite=!0,this.precision=null,this.polygonOffset=!1,this.polygonOffsetFactor=0,this.polygonOffsetUnits=0,this.dithering=!1,this.alphaToCoverage=!1,this.premultipliedAlpha=!1,this.forceSinglePass=!1,this.visible=!0,this.toneMapped=!0,this.userData={},this.version=0,this._alphaTest=0}get alphaTest(){return this._alphaTest}set alphaTest(t){this._alphaTest>0!=t>0&&this.version++,this._alphaTest=t}onBeforeRender(){}onBeforeCompile(){}customProgramCacheKey(){return this.onBeforeCompile.toString()}setValues(t){if(t!==void 0)for(const e in t){const i=t[e];if(i===void 0){console.warn(`THREE.Material: parameter '${e}' has value of undefined.`);continue}const s=this[e];if(s===void 0){console.warn(`THREE.Material: '${e}' is not a property of THREE.${this.type}.`);continue}s&&s.isColor?s.set(i):s&&s.isVector3&&i&&i.isVector3?s.copy(i):this[e]=i}}toJSON(t){const e=t===void 0||typeof t=="string";e&&(t={textures:{},images:{}});const i={metadata:{version:4.6,type:"Material",generator:"Material.toJSON"}};i.uuid=this.uuid,i.type=this.type,this.name!==""&&(i.name=this.name),this.color&&this.color.isColor&&(i.color=this.color.getHex()),this.roughness!==void 0&&(i.roughness=this.roughness),this.metalness!==void 0&&(i.metalness=this.metalness),this.sheen!==void 0&&(i.sheen=this.sheen),this.sheenColor&&this.sheenColor.isColor&&(i.sheenColor=this.sheenColor.getHex()),this.sheenRoughness!==void 0&&(i.sheenRoughness=this.sheenRoughness),this.emissive&&this.emissive.isColor&&(i.emissive=this.emissive.getHex()),this.emissiveIntensity!==void 0&&this.emissiveIntensity!==1&&(i.emissiveIntensity=this.emissiveIntensity),this.specular&&this.specular.isColor&&(i.specular=this.specular.getHex()),this.specularIntensity!==void 0&&(i.specularIntensity=this.specularIntensity),this.specularColor&&this.specularColor.isColor&&(i.specularColor=this.specularColor.getHex()),this.shininess!==void 0&&(i.shininess=this.shininess),this.clearcoat!==void 0&&(i.clearcoat=this.clearcoat),this.clearcoatRoughness!==void 0&&(i.clearcoatRoughness=this.clearcoatRoughness),this.clearcoatMap&&this.clearcoatMap.isTexture&&(i.clearcoatMap=this.clearcoatMap.toJSON(t).uuid),this.clearcoatRoughnessMap&&this.clearcoatRoughnessMap.isTexture&&(i.clearcoatRoughnessMap=this.clearcoatRoughnessMap.toJSON(t).uuid),this.clearcoatNormalMap&&this.clearcoatNormalMap.isTexture&&(i.clearcoatNormalMap=this.clearcoatNormalMap.toJSON(t).uuid,i.clearcoatNormalScale=this.clearcoatNormalScale.toArray()),this.dispersion!==void 0&&(i.dispersion=this.dispersion),this.iridescence!==void 0&&(i.iridescence=this.iridescence),this.iridescenceIOR!==void 0&&(i.iridescenceIOR=this.iridescenceIOR),this.iridescenceThicknessRange!==void 0&&(i.iridescenceThicknessRange=this.iridescenceThicknessRange),this.iridescenceMap&&this.iridescenceMap.isTexture&&(i.iridescenceMap=this.iridescenceMap.toJSON(t).uuid),this.iridescenceThicknessMap&&this.iridescenceThicknessMap.isTexture&&(i.iridescenceThicknessMap=this.iridescenceThicknessMap.toJSON(t).uuid),this.anisotropy!==void 0&&(i.anisotropy=this.anisotropy),this.anisotropyRotation!==void 0&&(i.anisotropyRotation=this.anisotropyRotation),this.anisotropyMap&&this.anisotropyMap.isTexture&&(i.anisotropyMap=this.anisotropyMap.toJSON(t).uuid),this.map&&this.map.isTexture&&(i.map=this.map.toJSON(t).uuid),this.matcap&&this.matcap.isTexture&&(i.matcap=this.matcap.toJSON(t).uuid),this.alphaMap&&this.alphaMap.isTexture&&(i.alphaMap=this.alphaMap.toJSON(t).uuid),this.lightMap&&this.lightMap.isTexture&&(i.lightMap=this.lightMap.toJSON(t).uuid,i.lightMapIntensity=this.lightMapIntensity),this.aoMap&&this.aoMap.isTexture&&(i.aoMap=this.aoMap.toJSON(t).uuid,i.aoMapIntensity=this.aoMapIntensity),this.bumpMap&&this.bumpMap.isTexture&&(i.bumpMap=this.bumpMap.toJSON(t).uuid,i.bumpScale=this.bumpScale),this.normalMap&&this.normalMap.isTexture&&(i.normalMap=this.normalMap.toJSON(t).uuid,i.normalMapType=this.normalMapType,i.normalScale=this.normalScale.toArray()),this.displacementMap&&this.displacementMap.isTexture&&(i.displacementMap=this.displacementMap.toJSON(t).uuid,i.displacementScale=this.displacementScale,i.displacementBias=this.displacementBias),this.roughnessMap&&this.roughnessMap.isTexture&&(i.roughnessMap=this.roughnessMap.toJSON(t).uuid),this.metalnessMap&&this.metalnessMap.isTexture&&(i.metalnessMap=this.metalnessMap.toJSON(t).uuid),this.emissiveMap&&this.emissiveMap.isTexture&&(i.emissiveMap=this.emissiveMap.toJSON(t).uuid),this.specularMap&&this.specularMap.isTexture&&(i.specularMap=this.specularMap.toJSON(t).uuid),this.specularIntensityMap&&this.specularIntensityMap.isTexture&&(i.specularIntensityMap=this.specularIntensityMap.toJSON(t).uuid),this.specularColorMap&&this.specularColorMap.isTexture&&(i.specularColorMap=this.specularColorMap.toJSON(t).uuid),this.envMap&&this.envMap.isTexture&&(i.envMap=this.envMap.toJSON(t).uuid,this.combine!==void 0&&(i.combine=this.combine)),this.envMapRotation!==void 0&&(i.envMapRotation=this.envMapRotation.toArray()),this.envMapIntensity!==void 0&&(i.envMapIntensity=this.envMapIntensity),this.reflectivity!==void 0&&(i.reflectivity=this.reflectivity),this.refractionRatio!==void 0&&(i.refractionRatio=this.refractionRatio),this.gradientMap&&this.gradientMap.isTexture&&(i.gradientMap=this.gradientMap.toJSON(t).uuid),this.transmission!==void 0&&(i.transmission=this.transmission),this.transmissionMap&&this.transmissionMap.isTexture&&(i.transmissionMap=this.transmissionMap.toJSON(t).uuid),this.thickness!==void 0&&(i.thickness=this.thickness),this.thicknessMap&&this.thicknessMap.isTexture&&(i.thicknessMap=this.thicknessMap.toJSON(t).uuid),this.attenuationDistance!==void 0&&this.attenuationDistance!==1/0&&(i.attenuationDistance=this.attenuationDistance),this.attenuationColor!==void 0&&(i.attenuationColor=this.attenuationColor.getHex()),this.size!==void 0&&(i.size=this.size),this.shadowSide!==null&&(i.shadowSide=this.shadowSide),this.sizeAttenuation!==void 0&&(i.sizeAttenuation=this.sizeAttenuation),this.blending!==ws&&(i.blending=this.blending),this.side!==_i&&(i.side=this.side),this.vertexColors===!0&&(i.vertexColors=!0),this.opacity<1&&(i.opacity=this.opacity),this.transparent===!0&&(i.transparent=!0),this.blendSrc!==Ml&&(i.blendSrc=this.blendSrc),this.blendDst!==Sl&&(i.blendDst=this.blendDst),this.blendEquation!==Fi&&(i.blendEquation=this.blendEquation),this.blendSrcAlpha!==null&&(i.blendSrcAlpha=this.blendSrcAlpha),this.blendDstAlpha!==null&&(i.blendDstAlpha=this.blendDstAlpha),this.blendEquationAlpha!==null&&(i.blendEquationAlpha=this.blendEquationAlpha),this.blendColor&&this.blendColor.isColor&&(i.blendColor=this.blendColor.getHex()),this.blendAlpha!==0&&(i.blendAlpha=this.blendAlpha),this.depthFunc!==Ds&&(i.depthFunc=this.depthFunc),this.depthTest===!1&&(i.depthTest=this.depthTest),this.depthWrite===!1&&(i.depthWrite=this.depthWrite),this.colorWrite===!1&&(i.colorWrite=this.colorWrite),this.stencilWriteMask!==255&&(i.stencilWriteMask=this.stencilWriteMask),this.stencilFunc!==wh&&(i.stencilFunc=this.stencilFunc),this.stencilRef!==0&&(i.stencilRef=this.stencilRef),this.stencilFuncMask!==255&&(i.stencilFuncMask=this.stencilFuncMask),this.stencilFail!==ts&&(i.stencilFail=this.stencilFail),this.stencilZFail!==ts&&(i.stencilZFail=this.stencilZFail),this.stencilZPass!==ts&&(i.stencilZPass=this.stencilZPass),this.stencilWrite===!0&&(i.stencilWrite=this.stencilWrite),this.rotation!==void 0&&this.rotation!==0&&(i.rotation=this.rotation),this.polygonOffset===!0&&(i.polygonOffset=!0),this.polygonOffsetFactor!==0&&(i.polygonOffsetFactor=this.polygonOffsetFactor),this.polygonOffsetUnits!==0&&(i.polygonOffsetUnits=this.polygonOffsetUnits),this.linewidth!==void 0&&this.linewidth!==1&&(i.linewidth=this.linewidth),this.dashSize!==void 0&&(i.dashSize=this.dashSize),this.gapSize!==void 0&&(i.gapSize=this.gapSize),this.scale!==void 0&&(i.scale=this.scale),this.dithering===!0&&(i.dithering=!0),this.alphaTest>0&&(i.alphaTest=this.alphaTest),this.alphaHash===!0&&(i.alphaHash=!0),this.alphaToCoverage===!0&&(i.alphaToCoverage=!0),this.premultipliedAlpha===!0&&(i.premultipliedAlpha=!0),this.forceSinglePass===!0&&(i.forceSinglePass=!0),this.wireframe===!0&&(i.wireframe=!0),this.wireframeLinewidth>1&&(i.wireframeLinewidth=this.wireframeLinewidth),this.wireframeLinecap!=="round"&&(i.wireframeLinecap=this.wireframeLinecap),this.wireframeLinejoin!=="round"&&(i.wireframeLinejoin=this.wireframeLinejoin),this.flatShading===!0&&(i.flatShading=!0),this.visible===!1&&(i.visible=!1),this.toneMapped===!1&&(i.toneMapped=!1),this.fog===!1&&(i.fog=!1),Object.keys(this.userData).length>0&&(i.userData=this.userData);function s(r){const o=[];for(const a in r){const l=r[a];delete l.metadata,o.push(l)}return o}if(e){const r=s(t.textures),o=s(t.images);r.length>0&&(i.textures=r),o.length>0&&(i.images=o)}return i}clone(){return new this.constructor().copy(this)}copy(t){this.name=t.name,this.blending=t.blending,this.side=t.side,this.vertexColors=t.vertexColors,this.opacity=t.opacity,this.transparent=t.transparent,this.blendSrc=t.blendSrc,this.blendDst=t.blendDst,this.blendEquation=t.blendEquation,this.blendSrcAlpha=t.blendSrcAlpha,this.blendDstAlpha=t.blendDstAlpha,this.blendEquationAlpha=t.blendEquationAlpha,this.blendColor.copy(t.blendColor),this.blendAlpha=t.blendAlpha,this.depthFunc=t.depthFunc,this.depthTest=t.depthTest,this.depthWrite=t.depthWrite,this.stencilWriteMask=t.stencilWriteMask,this.stencilFunc=t.stencilFunc,this.stencilRef=t.stencilRef,this.stencilFuncMask=t.stencilFuncMask,this.stencilFail=t.stencilFail,this.stencilZFail=t.stencilZFail,this.stencilZPass=t.stencilZPass,this.stencilWrite=t.stencilWrite;const e=t.clippingPlanes;let i=null;if(e!==null){const s=e.length;i=new Array(s);for(let r=0;r!==s;++r)i[r]=e[r].clone()}return this.clippingPlanes=i,this.clipIntersection=t.clipIntersection,this.clipShadows=t.clipShadows,this.shadowSide=t.shadowSide,this.colorWrite=t.colorWrite,this.precision=t.precision,this.polygonOffset=t.polygonOffset,this.polygonOffsetFactor=t.polygonOffsetFactor,this.polygonOffsetUnits=t.polygonOffsetUnits,this.dithering=t.dithering,this.alphaTest=t.alphaTest,this.alphaHash=t.alphaHash,this.alphaToCoverage=t.alphaToCoverage,this.premultipliedAlpha=t.premultipliedAlpha,this.forceSinglePass=t.forceSinglePass,this.visible=t.visible,this.toneMapped=t.toneMapped,this.userData=JSON.parse(JSON.stringify(t.userData)),this}dispose(){this.dispatchEvent({type:"dispose"})}set needsUpdate(t){t===!0&&this.version++}onBuild(){console.warn("Material: onBuild() has been removed.")}}class Ms extends yi{static get type(){return"MeshBasicMaterial"}constructor(t){super(),this.isMeshBasicMaterial=!0,this.color=new Ht(16777215),this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.specularMap=null,this.alphaMap=null,this.envMap=null,this.envMapRotation=new Pn,this.combine=xf,this.reflectivity=1,this.refractionRatio=.98,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.color.copy(t.color),this.map=t.map,this.lightMap=t.lightMap,this.lightMapIntensity=t.lightMapIntensity,this.aoMap=t.aoMap,this.aoMapIntensity=t.aoMapIntensity,this.specularMap=t.specularMap,this.alphaMap=t.alphaMap,this.envMap=t.envMap,this.envMapRotation.copy(t.envMapRotation),this.combine=t.combine,this.reflectivity=t.reflectivity,this.refractionRatio=t.refractionRatio,this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this.wireframeLinecap=t.wireframeLinecap,this.wireframeLinejoin=t.wireframeLinejoin,this.fog=t.fog,this}}const Ae=new L,Zr=new Tt;class en{constructor(t,e,i=!1){if(Array.isArray(t))throw new TypeError("THREE.BufferAttribute: array should be a Typed Array.");this.isBufferAttribute=!0,this.name="",this.array=t,this.itemSize=e,this.count=t!==void 0?t.length/e:0,this.normalized=i,this.usage=ac,this.updateRanges=[],this.gpuType=jn,this.version=0}onUploadCallback(){}set needsUpdate(t){t===!0&&this.version++}setUsage(t){return this.usage=t,this}addUpdateRange(t,e){this.updateRanges.push({start:t,count:e})}clearUpdateRanges(){this.updateRanges.length=0}copy(t){return this.name=t.name,this.array=new t.array.constructor(t.array),this.itemSize=t.itemSize,this.count=t.count,this.normalized=t.normalized,this.usage=t.usage,this.gpuType=t.gpuType,this}copyAt(t,e,i){t*=this.itemSize,i*=e.itemSize;for(let s=0,r=this.itemSize;s<r;s++)this.array[t+s]=e.array[i+s];return this}copyArray(t){return this.array.set(t),this}applyMatrix3(t){if(this.itemSize===2)for(let e=0,i=this.count;e<i;e++)Zr.fromBufferAttribute(this,e),Zr.applyMatrix3(t),this.setXY(e,Zr.x,Zr.y);else if(this.itemSize===3)for(let e=0,i=this.count;e<i;e++)Ae.fromBufferAttribute(this,e),Ae.applyMatrix3(t),this.setXYZ(e,Ae.x,Ae.y,Ae.z);return this}applyMatrix4(t){for(let e=0,i=this.count;e<i;e++)Ae.fromBufferAttribute(this,e),Ae.applyMatrix4(t),this.setXYZ(e,Ae.x,Ae.y,Ae.z);return this}applyNormalMatrix(t){for(let e=0,i=this.count;e<i;e++)Ae.fromBufferAttribute(this,e),Ae.applyNormalMatrix(t),this.setXYZ(e,Ae.x,Ae.y,Ae.z);return this}transformDirection(t){for(let e=0,i=this.count;e<i;e++)Ae.fromBufferAttribute(this,e),Ae.transformDirection(t),this.setXYZ(e,Ae.x,Ae.y,Ae.z);return this}set(t,e=0){return this.array.set(t,e),this}getComponent(t,e){let i=this.array[t*this.itemSize+e];return this.normalized&&(i=wn(i,this.array)),i}setComponent(t,e,i){return this.normalized&&(i=se(i,this.array)),this.array[t*this.itemSize+e]=i,this}getX(t){let e=this.array[t*this.itemSize];return this.normalized&&(e=wn(e,this.array)),e}setX(t,e){return this.normalized&&(e=se(e,this.array)),this.array[t*this.itemSize]=e,this}getY(t){let e=this.array[t*this.itemSize+1];return this.normalized&&(e=wn(e,this.array)),e}setY(t,e){return this.normalized&&(e=se(e,this.array)),this.array[t*this.itemSize+1]=e,this}getZ(t){let e=this.array[t*this.itemSize+2];return this.normalized&&(e=wn(e,this.array)),e}setZ(t,e){return this.normalized&&(e=se(e,this.array)),this.array[t*this.itemSize+2]=e,this}getW(t){let e=this.array[t*this.itemSize+3];return this.normalized&&(e=wn(e,this.array)),e}setW(t,e){return this.normalized&&(e=se(e,this.array)),this.array[t*this.itemSize+3]=e,this}setXY(t,e,i){return t*=this.itemSize,this.normalized&&(e=se(e,this.array),i=se(i,this.array)),this.array[t+0]=e,this.array[t+1]=i,this}setXYZ(t,e,i,s){return t*=this.itemSize,this.normalized&&(e=se(e,this.array),i=se(i,this.array),s=se(s,this.array)),this.array[t+0]=e,this.array[t+1]=i,this.array[t+2]=s,this}setXYZW(t,e,i,s,r){return t*=this.itemSize,this.normalized&&(e=se(e,this.array),i=se(i,this.array),s=se(s,this.array),r=se(r,this.array)),this.array[t+0]=e,this.array[t+1]=i,this.array[t+2]=s,this.array[t+3]=r,this}onUpload(t){return this.onUploadCallback=t,this}clone(){return new this.constructor(this.array,this.itemSize).copy(this)}toJSON(){const t={itemSize:this.itemSize,type:this.array.constructor.name,array:Array.from(this.array),normalized:this.normalized};return this.name!==""&&(t.name=this.name),this.usage!==ac&&(t.usage=this.usage),t}}class Nf extends en{constructor(t,e,i){super(new Uint16Array(t),e,i)}}class Ff extends en{constructor(t,e,i){super(new Uint32Array(t),e,i)}}class Se extends en{constructor(t,e,i){super(new Float32Array(t),e,i)}}let Pg=0;const rn=new oe,$a=new Ce,hs=new L,Je=new Lr,qs=new Lr,Ie=new L;class Te extends Ki{constructor(){super(),this.isBufferGeometry=!0,Object.defineProperty(this,"id",{value:Pg++}),this.uuid=mi(),this.name="",this.type="BufferGeometry",this.index=null,this.indirect=null,this.attributes={},this.morphAttributes={},this.morphTargetsRelative=!1,this.groups=[],this.boundingBox=null,this.boundingSphere=null,this.drawRange={start:0,count:1/0},this.userData={}}getIndex(){return this.index}setIndex(t){return Array.isArray(t)?this.index=new(Lf(t)?Ff:Nf)(t,1):this.index=t,this}setIndirect(t){return this.indirect=t,this}getIndirect(){return this.indirect}getAttribute(t){return this.attributes[t]}setAttribute(t,e){return this.attributes[t]=e,this}deleteAttribute(t){return delete this.attributes[t],this}hasAttribute(t){return this.attributes[t]!==void 0}addGroup(t,e,i=0){this.groups.push({start:t,count:e,materialIndex:i})}clearGroups(){this.groups=[]}setDrawRange(t,e){this.drawRange.start=t,this.drawRange.count=e}applyMatrix4(t){const e=this.attributes.position;e!==void 0&&(e.applyMatrix4(t),e.needsUpdate=!0);const i=this.attributes.normal;if(i!==void 0){const r=new Ft().getNormalMatrix(t);i.applyNormalMatrix(r),i.needsUpdate=!0}const s=this.attributes.tangent;return s!==void 0&&(s.transformDirection(t),s.needsUpdate=!0),this.boundingBox!==null&&this.computeBoundingBox(),this.boundingSphere!==null&&this.computeBoundingSphere(),this}applyQuaternion(t){return rn.makeRotationFromQuaternion(t),this.applyMatrix4(rn),this}rotateX(t){return rn.makeRotationX(t),this.applyMatrix4(rn),this}rotateY(t){return rn.makeRotationY(t),this.applyMatrix4(rn),this}rotateZ(t){return rn.makeRotationZ(t),this.applyMatrix4(rn),this}translate(t,e,i){return rn.makeTranslation(t,e,i),this.applyMatrix4(rn),this}scale(t,e,i){return rn.makeScale(t,e,i),this.applyMatrix4(rn),this}lookAt(t){return $a.lookAt(t),$a.updateMatrix(),this.applyMatrix4($a.matrix),this}center(){return this.computeBoundingBox(),this.boundingBox.getCenter(hs).negate(),this.translate(hs.x,hs.y,hs.z),this}setFromPoints(t){const e=this.getAttribute("position");if(e===void 0){const i=[];for(let s=0,r=t.length;s<r;s++){const o=t[s];i.push(o.x,o.y,o.z||0)}this.setAttribute("position",new Se(i,3))}else{for(let i=0,s=e.count;i<s;i++){const r=t[i];e.setXYZ(i,r.x,r.y,r.z||0)}t.length>e.count&&console.warn("THREE.BufferGeometry: Buffer size too small for points data. Use .dispose() and create a new geometry."),e.needsUpdate=!0}return this}computeBoundingBox(){this.boundingBox===null&&(this.boundingBox=new Lr);const t=this.attributes.position,e=this.morphAttributes.position;if(t&&t.isGLBufferAttribute){console.error("THREE.BufferGeometry.computeBoundingBox(): GLBufferAttribute requires a manual bounding box.",this),this.boundingBox.set(new L(-1/0,-1/0,-1/0),new L(1/0,1/0,1/0));return}if(t!==void 0){if(this.boundingBox.setFromBufferAttribute(t),e)for(let i=0,s=e.length;i<s;i++){const r=e[i];Je.setFromBufferAttribute(r),this.morphTargetsRelative?(Ie.addVectors(this.boundingBox.min,Je.min),this.boundingBox.expandByPoint(Ie),Ie.addVectors(this.boundingBox.max,Je.max),this.boundingBox.expandByPoint(Ie)):(this.boundingBox.expandByPoint(Je.min),this.boundingBox.expandByPoint(Je.max))}}else this.boundingBox.makeEmpty();(isNaN(this.boundingBox.min.x)||isNaN(this.boundingBox.min.y)||isNaN(this.boundingBox.min.z))&&console.error('THREE.BufferGeometry.computeBoundingBox(): Computed min/max have NaN values. The "position" attribute is likely to have NaN values.',this)}computeBoundingSphere(){this.boundingSphere===null&&(this.boundingSphere=new Ir);const t=this.attributes.position,e=this.morphAttributes.position;if(t&&t.isGLBufferAttribute){console.error("THREE.BufferGeometry.computeBoundingSphere(): GLBufferAttribute requires a manual bounding sphere.",this),this.boundingSphere.set(new L,1/0);return}if(t){const i=this.boundingSphere.center;if(Je.setFromBufferAttribute(t),e)for(let r=0,o=e.length;r<o;r++){const a=e[r];qs.setFromBufferAttribute(a),this.morphTargetsRelative?(Ie.addVectors(Je.min,qs.min),Je.expandByPoint(Ie),Ie.addVectors(Je.max,qs.max),Je.expandByPoint(Ie)):(Je.expandByPoint(qs.min),Je.expandByPoint(qs.max))}Je.getCenter(i);let s=0;for(let r=0,o=t.count;r<o;r++)Ie.fromBufferAttribute(t,r),s=Math.max(s,i.distanceToSquared(Ie));if(e)for(let r=0,o=e.length;r<o;r++){const a=e[r],l=this.morphTargetsRelative;for(let c=0,h=a.count;c<h;c++)Ie.fromBufferAttribute(a,c),l&&(hs.fromBufferAttribute(t,c),Ie.add(hs)),s=Math.max(s,i.distanceToSquared(Ie))}this.boundingSphere.radius=Math.sqrt(s),isNaN(this.boundingSphere.radius)&&console.error('THREE.BufferGeometry.computeBoundingSphere(): Computed radius is NaN. The "position" attribute is likely to have NaN values.',this)}}computeTangents(){const t=this.index,e=this.attributes;if(t===null||e.position===void 0||e.normal===void 0||e.uv===void 0){console.error("THREE.BufferGeometry: .computeTangents() failed. Missing required attributes (index, position, normal or uv)");return}const i=e.position,s=e.normal,r=e.uv;this.hasAttribute("tangent")===!1&&this.setAttribute("tangent",new en(new Float32Array(4*i.count),4));const o=this.getAttribute("tangent"),a=[],l=[];for(let R=0;R<i.count;R++)a[R]=new L,l[R]=new L;const c=new L,h=new L,u=new L,d=new Tt,f=new Tt,g=new Tt,_=new L,m=new L;function p(R,S,b){c.fromBufferAttribute(i,R),h.fromBufferAttribute(i,S),u.fromBufferAttribute(i,b),d.fromBufferAttribute(r,R),f.fromBufferAttribute(r,S),g.fromBufferAttribute(r,b),h.sub(c),u.sub(c),f.sub(d),g.sub(d);const P=1/(f.x*g.y-g.x*f.y);isFinite(P)&&(_.copy(h).multiplyScalar(g.y).addScaledVector(u,-f.y).multiplyScalar(P),m.copy(u).multiplyScalar(f.x).addScaledVector(h,-g.x).multiplyScalar(P),a[R].add(_),a[S].add(_),a[b].add(_),l[R].add(m),l[S].add(m),l[b].add(m))}let v=this.groups;v.length===0&&(v=[{start:0,count:t.count}]);for(let R=0,S=v.length;R<S;++R){const b=v[R],P=b.start,N=b.count;for(let F=P,W=P+N;F<W;F+=3)p(t.getX(F+0),t.getX(F+1),t.getX(F+2))}const y=new L,x=new L,A=new L,w=new L;function T(R){A.fromBufferAttribute(s,R),w.copy(A);const S=a[R];y.copy(S),y.sub(A.multiplyScalar(A.dot(S))).normalize(),x.crossVectors(w,S);const P=x.dot(l[R])<0?-1:1;o.setXYZW(R,y.x,y.y,y.z,P)}for(let R=0,S=v.length;R<S;++R){const b=v[R],P=b.start,N=b.count;for(let F=P,W=P+N;F<W;F+=3)T(t.getX(F+0)),T(t.getX(F+1)),T(t.getX(F+2))}}computeVertexNormals(){const t=this.index,e=this.getAttribute("position");if(e!==void 0){let i=this.getAttribute("normal");if(i===void 0)i=new en(new Float32Array(e.count*3),3),this.setAttribute("normal",i);else for(let d=0,f=i.count;d<f;d++)i.setXYZ(d,0,0,0);const s=new L,r=new L,o=new L,a=new L,l=new L,c=new L,h=new L,u=new L;if(t)for(let d=0,f=t.count;d<f;d+=3){const g=t.getX(d+0),_=t.getX(d+1),m=t.getX(d+2);s.fromBufferAttribute(e,g),r.fromBufferAttribute(e,_),o.fromBufferAttribute(e,m),h.subVectors(o,r),u.subVectors(s,r),h.cross(u),a.fromBufferAttribute(i,g),l.fromBufferAttribute(i,_),c.fromBufferAttribute(i,m),a.add(h),l.add(h),c.add(h),i.setXYZ(g,a.x,a.y,a.z),i.setXYZ(_,l.x,l.y,l.z),i.setXYZ(m,c.x,c.y,c.z)}else for(let d=0,f=e.count;d<f;d+=3)s.fromBufferAttribute(e,d+0),r.fromBufferAttribute(e,d+1),o.fromBufferAttribute(e,d+2),h.subVectors(o,r),u.subVectors(s,r),h.cross(u),i.setXYZ(d+0,h.x,h.y,h.z),i.setXYZ(d+1,h.x,h.y,h.z),i.setXYZ(d+2,h.x,h.y,h.z);this.normalizeNormals(),i.needsUpdate=!0}}normalizeNormals(){const t=this.attributes.normal;for(let e=0,i=t.count;e<i;e++)Ie.fromBufferAttribute(t,e),Ie.normalize(),t.setXYZ(e,Ie.x,Ie.y,Ie.z)}toNonIndexed(){function t(a,l){const c=a.array,h=a.itemSize,u=a.normalized,d=new c.constructor(l.length*h);let f=0,g=0;for(let _=0,m=l.length;_<m;_++){a.isInterleavedBufferAttribute?f=l[_]*a.data.stride+a.offset:f=l[_]*h;for(let p=0;p<h;p++)d[g++]=c[f++]}return new en(d,h,u)}if(this.index===null)return console.warn("THREE.BufferGeometry.toNonIndexed(): BufferGeometry is already non-indexed."),this;const e=new Te,i=this.index.array,s=this.attributes;for(const a in s){const l=s[a],c=t(l,i);e.setAttribute(a,c)}const r=this.morphAttributes;for(const a in r){const l=[],c=r[a];for(let h=0,u=c.length;h<u;h++){const d=c[h],f=t(d,i);l.push(f)}e.morphAttributes[a]=l}e.morphTargetsRelative=this.morphTargetsRelative;const o=this.groups;for(let a=0,l=o.length;a<l;a++){const c=o[a];e.addGroup(c.start,c.count,c.materialIndex)}return e}toJSON(){const t={metadata:{version:4.6,type:"BufferGeometry",generator:"BufferGeometry.toJSON"}};if(t.uuid=this.uuid,t.type=this.type,this.name!==""&&(t.name=this.name),Object.keys(this.userData).length>0&&(t.userData=this.userData),this.parameters!==void 0){const l=this.parameters;for(const c in l)l[c]!==void 0&&(t[c]=l[c]);return t}t.data={attributes:{}};const e=this.index;e!==null&&(t.data.index={type:e.array.constructor.name,array:Array.prototype.slice.call(e.array)});const i=this.attributes;for(const l in i){const c=i[l];t.data.attributes[l]=c.toJSON(t.data)}const s={};let r=!1;for(const l in this.morphAttributes){const c=this.morphAttributes[l],h=[];for(let u=0,d=c.length;u<d;u++){const f=c[u];h.push(f.toJSON(t.data))}h.length>0&&(s[l]=h,r=!0)}r&&(t.data.morphAttributes=s,t.data.morphTargetsRelative=this.morphTargetsRelative);const o=this.groups;o.length>0&&(t.data.groups=JSON.parse(JSON.stringify(o)));const a=this.boundingSphere;return a!==null&&(t.data.boundingSphere={center:a.center.toArray(),radius:a.radius}),t}clone(){return new this.constructor().copy(this)}copy(t){this.index=null,this.attributes={},this.morphAttributes={},this.groups=[],this.boundingBox=null,this.boundingSphere=null;const e={};this.name=t.name;const i=t.index;i!==null&&this.setIndex(i.clone(e));const s=t.attributes;for(const c in s){const h=s[c];this.setAttribute(c,h.clone(e))}const r=t.morphAttributes;for(const c in r){const h=[],u=r[c];for(let d=0,f=u.length;d<f;d++)h.push(u[d].clone(e));this.morphAttributes[c]=h}this.morphTargetsRelative=t.morphTargetsRelative;const o=t.groups;for(let c=0,h=o.length;c<h;c++){const u=o[c];this.addGroup(u.start,u.count,u.materialIndex)}const a=t.boundingBox;a!==null&&(this.boundingBox=a.clone());const l=t.boundingSphere;return l!==null&&(this.boundingSphere=l.clone()),this.drawRange.start=t.drawRange.start,this.drawRange.count=t.drawRange.count,this.userData=t.userData,this}dispose(){this.dispatchEvent({type:"dispose"})}}const Gh=new oe,wi=new Or,Jr=new Ir,Wh=new L,Qr=new L,to=new L,eo=new L,Ya=new L,no=new L,Xh=new L,io=new L;class xe extends Ce{constructor(t=new Te,e=new Ms){super(),this.isMesh=!0,this.type="Mesh",this.geometry=t,this.material=e,this.updateMorphTargets()}copy(t,e){return super.copy(t,e),t.morphTargetInfluences!==void 0&&(this.morphTargetInfluences=t.morphTargetInfluences.slice()),t.morphTargetDictionary!==void 0&&(this.morphTargetDictionary=Object.assign({},t.morphTargetDictionary)),this.material=Array.isArray(t.material)?t.material.slice():t.material,this.geometry=t.geometry,this}updateMorphTargets(){const e=this.geometry.morphAttributes,i=Object.keys(e);if(i.length>0){const s=e[i[0]];if(s!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let r=0,o=s.length;r<o;r++){const a=s[r].name||String(r);this.morphTargetInfluences.push(0),this.morphTargetDictionary[a]=r}}}}getVertexPosition(t,e){const i=this.geometry,s=i.attributes.position,r=i.morphAttributes.position,o=i.morphTargetsRelative;e.fromBufferAttribute(s,t);const a=this.morphTargetInfluences;if(r&&a){no.set(0,0,0);for(let l=0,c=r.length;l<c;l++){const h=a[l],u=r[l];h!==0&&(Ya.fromBufferAttribute(u,t),o?no.addScaledVector(Ya,h):no.addScaledVector(Ya.sub(e),h))}e.add(no)}return e}raycast(t,e){const i=this.geometry,s=this.material,r=this.matrixWorld;s!==void 0&&(i.boundingSphere===null&&i.computeBoundingSphere(),Jr.copy(i.boundingSphere),Jr.applyMatrix4(r),wi.copy(t.ray).recast(t.near),!(Jr.containsPoint(wi.origin)===!1&&(wi.intersectSphere(Jr,Wh)===null||wi.origin.distanceToSquared(Wh)>(t.far-t.near)**2))&&(Gh.copy(r).invert(),wi.copy(t.ray).applyMatrix4(Gh),!(i.boundingBox!==null&&wi.intersectsBox(i.boundingBox)===!1)&&this._computeIntersections(t,e,wi)))}_computeIntersections(t,e,i){let s;const r=this.geometry,o=this.material,a=r.index,l=r.attributes.position,c=r.attributes.uv,h=r.attributes.uv1,u=r.attributes.normal,d=r.groups,f=r.drawRange;if(a!==null)if(Array.isArray(o))for(let g=0,_=d.length;g<_;g++){const m=d[g],p=o[m.materialIndex],v=Math.max(m.start,f.start),y=Math.min(a.count,Math.min(m.start+m.count,f.start+f.count));for(let x=v,A=y;x<A;x+=3){const w=a.getX(x),T=a.getX(x+1),R=a.getX(x+2);s=so(this,p,t,i,c,h,u,w,T,R),s&&(s.faceIndex=Math.floor(x/3),s.face.materialIndex=m.materialIndex,e.push(s))}}else{const g=Math.max(0,f.start),_=Math.min(a.count,f.start+f.count);for(let m=g,p=_;m<p;m+=3){const v=a.getX(m),y=a.getX(m+1),x=a.getX(m+2);s=so(this,o,t,i,c,h,u,v,y,x),s&&(s.faceIndex=Math.floor(m/3),e.push(s))}}else if(l!==void 0)if(Array.isArray(o))for(let g=0,_=d.length;g<_;g++){const m=d[g],p=o[m.materialIndex],v=Math.max(m.start,f.start),y=Math.min(l.count,Math.min(m.start+m.count,f.start+f.count));for(let x=v,A=y;x<A;x+=3){const w=x,T=x+1,R=x+2;s=so(this,p,t,i,c,h,u,w,T,R),s&&(s.faceIndex=Math.floor(x/3),s.face.materialIndex=m.materialIndex,e.push(s))}}else{const g=Math.max(0,f.start),_=Math.min(l.count,f.start+f.count);for(let m=g,p=_;m<p;m+=3){const v=m,y=m+1,x=m+2;s=so(this,o,t,i,c,h,u,v,y,x),s&&(s.faceIndex=Math.floor(m/3),e.push(s))}}}}function Dg(n,t,e,i,s,r,o,a){let l;if(t.side===qe?l=i.intersectTriangle(o,r,s,!0,a):l=i.intersectTriangle(s,r,o,t.side===_i,a),l===null)return null;io.copy(a),io.applyMatrix4(n.matrixWorld);const c=e.ray.origin.distanceTo(io);return c<e.near||c>e.far?null:{distance:c,point:io.clone(),object:n}}function so(n,t,e,i,s,r,o,a,l,c){n.getVertexPosition(a,Qr),n.getVertexPosition(l,to),n.getVertexPosition(c,eo);const h=Dg(n,t,e,i,Qr,to,eo,Xh);if(h){const u=new L;cn.getBarycoord(Xh,Qr,to,eo,u),s&&(h.uv=cn.getInterpolatedAttribute(s,a,l,c,u,new Tt)),r&&(h.uv1=cn.getInterpolatedAttribute(r,a,l,c,u,new Tt)),o&&(h.normal=cn.getInterpolatedAttribute(o,a,l,c,u,new L),h.normal.dot(i.direction)>0&&h.normal.multiplyScalar(-1));const d={a,b:l,c,normal:new L,materialIndex:0};cn.getNormal(Qr,to,eo,d.normal),h.face=d,h.barycoord=u}return h}class An extends Te{constructor(t=1,e=1,i=1,s=1,r=1,o=1){super(),this.type="BoxGeometry",this.parameters={width:t,height:e,depth:i,widthSegments:s,heightSegments:r,depthSegments:o};const a=this;s=Math.floor(s),r=Math.floor(r),o=Math.floor(o);const l=[],c=[],h=[],u=[];let d=0,f=0;g("z","y","x",-1,-1,i,e,t,o,r,0),g("z","y","x",1,-1,i,e,-t,o,r,1),g("x","z","y",1,1,t,i,e,s,o,2),g("x","z","y",1,-1,t,i,-e,s,o,3),g("x","y","z",1,-1,t,e,i,s,r,4),g("x","y","z",-1,-1,t,e,-i,s,r,5),this.setIndex(l),this.setAttribute("position",new Se(c,3)),this.setAttribute("normal",new Se(h,3)),this.setAttribute("uv",new Se(u,2));function g(_,m,p,v,y,x,A,w,T,R,S){const b=x/T,P=A/R,N=x/2,F=A/2,W=w/2,G=T+1,H=R+1;let $=0,V=0;const et=new L;for(let at=0;at<H;at++){const _t=at*P-F;for(let Dt=0;Dt<G;Dt++){const $t=Dt*b-N;et[_]=$t*v,et[m]=_t*y,et[p]=W,c.push(et.x,et.y,et.z),et[_]=0,et[m]=0,et[p]=w>0?1:-1,h.push(et.x,et.y,et.z),u.push(Dt/T),u.push(1-at/R),$+=1}}for(let at=0;at<R;at++)for(let _t=0;_t<T;_t++){const Dt=d+_t+G*at,$t=d+_t+G*(at+1),j=d+(_t+1)+G*(at+1),tt=d+(_t+1)+G*at;l.push(Dt,$t,tt),l.push($t,j,tt),V+=6}a.addGroup(f,V,S),f+=V,d+=$}}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new An(t.width,t.height,t.depth,t.widthSegments,t.heightSegments,t.depthSegments)}}function Ns(n){const t={};for(const e in n){t[e]={};for(const i in n[e]){const s=n[e][i];s&&(s.isColor||s.isMatrix3||s.isMatrix4||s.isVector2||s.isVector3||s.isVector4||s.isTexture||s.isQuaternion)?s.isRenderTargetTexture?(console.warn("UniformsUtils: Textures of render targets cannot be cloned via cloneUniforms() or mergeUniforms()."),t[e][i]=null):t[e][i]=s.clone():Array.isArray(s)?t[e][i]=s.slice():t[e][i]=s}}return t}function Ve(n){const t={};for(let e=0;e<n.length;e++){const i=Ns(n[e]);for(const s in i)t[s]=i[s]}return t}function Lg(n){const t=[];for(let e=0;e<n.length;e++)t.push(n[e].clone());return t}function kf(n){const t=n.getRenderTarget();return t===null?n.outputColorSpace:t.isXRRenderTarget===!0?t.texture.colorSpace:Kt.workingColorSpace}const Ig={clone:Ns,merge:Ve};var Og=`void main() {
	gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );
}`,Ug=`void main() {
	gl_FragColor = vec4( 1.0, 0.0, 0.0, 1.0 );
}`;class xi extends yi{static get type(){return"ShaderMaterial"}constructor(t){super(),this.isShaderMaterial=!0,this.defines={},this.uniforms={},this.uniformsGroups=[],this.vertexShader=Og,this.fragmentShader=Ug,this.linewidth=1,this.wireframe=!1,this.wireframeLinewidth=1,this.fog=!1,this.lights=!1,this.clipping=!1,this.forceSinglePass=!0,this.extensions={clipCullDistance:!1,multiDraw:!1},this.defaultAttributeValues={color:[1,1,1],uv:[0,0],uv1:[0,0]},this.index0AttributeName=void 0,this.uniformsNeedUpdate=!1,this.glslVersion=null,t!==void 0&&this.setValues(t)}copy(t){return super.copy(t),this.fragmentShader=t.fragmentShader,this.vertexShader=t.vertexShader,this.uniforms=Ns(t.uniforms),this.uniformsGroups=Lg(t.uniformsGroups),this.defines=Object.assign({},t.defines),this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this.fog=t.fog,this.lights=t.lights,this.clipping=t.clipping,this.extensions=Object.assign({},t.extensions),this.glslVersion=t.glslVersion,this}toJSON(t){const e=super.toJSON(t);e.glslVersion=this.glslVersion,e.uniforms={};for(const s in this.uniforms){const o=this.uniforms[s].value;o&&o.isTexture?e.uniforms[s]={type:"t",value:o.toJSON(t).uuid}:o&&o.isColor?e.uniforms[s]={type:"c",value:o.getHex()}:o&&o.isVector2?e.uniforms[s]={type:"v2",value:o.toArray()}:o&&o.isVector3?e.uniforms[s]={type:"v3",value:o.toArray()}:o&&o.isVector4?e.uniforms[s]={type:"v4",value:o.toArray()}:o&&o.isMatrix3?e.uniforms[s]={type:"m3",value:o.toArray()}:o&&o.isMatrix4?e.uniforms[s]={type:"m4",value:o.toArray()}:e.uniforms[s]={value:o}}Object.keys(this.defines).length>0&&(e.defines=this.defines),e.vertexShader=this.vertexShader,e.fragmentShader=this.fragmentShader,e.lights=this.lights,e.clipping=this.clipping;const i={};for(const s in this.extensions)this.extensions[s]===!0&&(i[s]=!0);return Object.keys(i).length>0&&(e.extensions=i),e}}class Bf extends Ce{constructor(){super(),this.isCamera=!0,this.type="Camera",this.matrixWorldInverse=new oe,this.projectionMatrix=new oe,this.projectionMatrixInverse=new oe,this.coordinateSystem=$n}copy(t,e){return super.copy(t,e),this.matrixWorldInverse.copy(t.matrixWorldInverse),this.projectionMatrix.copy(t.projectionMatrix),this.projectionMatrixInverse.copy(t.projectionMatrixInverse),this.coordinateSystem=t.coordinateSystem,this}getWorldDirection(t){return super.getWorldDirection(t).negate()}updateMatrixWorld(t){super.updateMatrixWorld(t),this.matrixWorldInverse.copy(this.matrixWorld).invert()}updateWorldMatrix(t,e){super.updateWorldMatrix(t,e),this.matrixWorldInverse.copy(this.matrixWorld).invert()}clone(){return new this.constructor().copy(this)}}const oi=new L,jh=new Tt,$h=new Tt;class ln extends Bf{constructor(t=50,e=1,i=.1,s=2e3){super(),this.isPerspectiveCamera=!0,this.type="PerspectiveCamera",this.fov=t,this.zoom=1,this.near=i,this.far=s,this.focus=10,this.aspect=e,this.view=null,this.filmGauge=35,this.filmOffset=0,this.updateProjectionMatrix()}copy(t,e){return super.copy(t,e),this.fov=t.fov,this.zoom=t.zoom,this.near=t.near,this.far=t.far,this.focus=t.focus,this.aspect=t.aspect,this.view=t.view===null?null:Object.assign({},t.view),this.filmGauge=t.filmGauge,this.filmOffset=t.filmOffset,this}setFocalLength(t){const e=.5*this.getFilmHeight()/t;this.fov=lc*2*Math.atan(e),this.updateProjectionMatrix()}getFocalLength(){const t=Math.tan(Bo*.5*this.fov);return .5*this.getFilmHeight()/t}getEffectiveFOV(){return lc*2*Math.atan(Math.tan(Bo*.5*this.fov)/this.zoom)}getFilmWidth(){return this.filmGauge*Math.min(this.aspect,1)}getFilmHeight(){return this.filmGauge/Math.max(this.aspect,1)}getViewBounds(t,e,i){oi.set(-1,-1,.5).applyMatrix4(this.projectionMatrixInverse),e.set(oi.x,oi.y).multiplyScalar(-t/oi.z),oi.set(1,1,.5).applyMatrix4(this.projectionMatrixInverse),i.set(oi.x,oi.y).multiplyScalar(-t/oi.z)}getViewSize(t,e){return this.getViewBounds(t,jh,$h),e.subVectors($h,jh)}setViewOffset(t,e,i,s,r,o){this.aspect=t/e,this.view===null&&(this.view={enabled:!0,fullWidth:1,fullHeight:1,offsetX:0,offsetY:0,width:1,height:1}),this.view.enabled=!0,this.view.fullWidth=t,this.view.fullHeight=e,this.view.offsetX=i,this.view.offsetY=s,this.view.width=r,this.view.height=o,this.updateProjectionMatrix()}clearViewOffset(){this.view!==null&&(this.view.enabled=!1),this.updateProjectionMatrix()}updateProjectionMatrix(){const t=this.near;let e=t*Math.tan(Bo*.5*this.fov)/this.zoom,i=2*e,s=this.aspect*i,r=-.5*s;const o=this.view;if(this.view!==null&&this.view.enabled){const l=o.fullWidth,c=o.fullHeight;r+=o.offsetX*s/l,e-=o.offsetY*i/c,s*=o.width/l,i*=o.height/c}const a=this.filmOffset;a!==0&&(r+=t*a/this.getFilmWidth()),this.projectionMatrix.makePerspective(r,r+s,e,e-i,t,this.far,this.coordinateSystem),this.projectionMatrixInverse.copy(this.projectionMatrix).invert()}toJSON(t){const e=super.toJSON(t);return e.object.fov=this.fov,e.object.zoom=this.zoom,e.object.near=this.near,e.object.far=this.far,e.object.focus=this.focus,e.object.aspect=this.aspect,this.view!==null&&(e.object.view=Object.assign({},this.view)),e.object.filmGauge=this.filmGauge,e.object.filmOffset=this.filmOffset,e}}const us=-90,ds=1;class Ng extends Ce{constructor(t,e,i){super(),this.type="CubeCamera",this.renderTarget=i,this.coordinateSystem=null,this.activeMipmapLevel=0;const s=new ln(us,ds,t,e);s.layers=this.layers,this.add(s);const r=new ln(us,ds,t,e);r.layers=this.layers,this.add(r);const o=new ln(us,ds,t,e);o.layers=this.layers,this.add(o);const a=new ln(us,ds,t,e);a.layers=this.layers,this.add(a);const l=new ln(us,ds,t,e);l.layers=this.layers,this.add(l);const c=new ln(us,ds,t,e);c.layers=this.layers,this.add(c)}updateCoordinateSystem(){const t=this.coordinateSystem,e=this.children.concat(),[i,s,r,o,a,l]=e;for(const c of e)this.remove(c);if(t===$n)i.up.set(0,1,0),i.lookAt(1,0,0),s.up.set(0,1,0),s.lookAt(-1,0,0),r.up.set(0,0,-1),r.lookAt(0,1,0),o.up.set(0,0,1),o.lookAt(0,-1,0),a.up.set(0,1,0),a.lookAt(0,0,1),l.up.set(0,1,0),l.lookAt(0,0,-1);else if(t===Zo)i.up.set(0,-1,0),i.lookAt(-1,0,0),s.up.set(0,-1,0),s.lookAt(1,0,0),r.up.set(0,0,1),r.lookAt(0,1,0),o.up.set(0,0,-1),o.lookAt(0,-1,0),a.up.set(0,-1,0),a.lookAt(0,0,1),l.up.set(0,-1,0),l.lookAt(0,0,-1);else throw new Error("THREE.CubeCamera.updateCoordinateSystem(): Invalid coordinate system: "+t);for(const c of e)this.add(c),c.updateMatrixWorld()}update(t,e){this.parent===null&&this.updateMatrixWorld();const{renderTarget:i,activeMipmapLevel:s}=this;this.coordinateSystem!==t.coordinateSystem&&(this.coordinateSystem=t.coordinateSystem,this.updateCoordinateSystem());const[r,o,a,l,c,h]=this.children,u=t.getRenderTarget(),d=t.getActiveCubeFace(),f=t.getActiveMipmapLevel(),g=t.xr.enabled;t.xr.enabled=!1;const _=i.texture.generateMipmaps;i.texture.generateMipmaps=!1,t.setRenderTarget(i,0,s),t.render(e,r),t.setRenderTarget(i,1,s),t.render(e,o),t.setRenderTarget(i,2,s),t.render(e,a),t.setRenderTarget(i,3,s),t.render(e,l),t.setRenderTarget(i,4,s),t.render(e,c),i.texture.generateMipmaps=_,t.setRenderTarget(i,5,s),t.render(e,h),t.setRenderTarget(u,d,f),t.xr.enabled=g,i.texture.needsPMREMUpdate=!0}}class zf extends We{constructor(t,e,i,s,r,o,a,l,c,h){t=t!==void 0?t:[],e=e!==void 0?e:Ls,super(t,e,i,s,r,o,a,l,c,h),this.isCubeTexture=!0,this.flipY=!1}get images(){return this.image}set images(t){this.image=t}}class Fg extends ji{constructor(t=1,e={}){super(t,t,e),this.isWebGLCubeRenderTarget=!0;const i={width:t,height:t,depth:1},s=[i,i,i,i,i,i];this.texture=new zf(s,e.mapping,e.wrapS,e.wrapT,e.magFilter,e.minFilter,e.format,e.type,e.anisotropy,e.colorSpace),this.texture.isRenderTargetTexture=!0,this.texture.generateMipmaps=e.generateMipmaps!==void 0?e.generateMipmaps:!1,this.texture.minFilter=e.minFilter!==void 0?e.minFilter:Cn}fromEquirectangularTexture(t,e){this.texture.type=e.type,this.texture.colorSpace=e.colorSpace,this.texture.generateMipmaps=e.generateMipmaps,this.texture.minFilter=e.minFilter,this.texture.magFilter=e.magFilter;const i={uniforms:{tEquirect:{value:null}},vertexShader:`

				varying vec3 vWorldDirection;

				vec3 transformDirection( in vec3 dir, in mat4 matrix ) {

					return normalize( ( matrix * vec4( dir, 0.0 ) ).xyz );

				}

				void main() {

					vWorldDirection = transformDirection( position, modelMatrix );

					#include <begin_vertex>
					#include <project_vertex>

				}
			`,fragmentShader:`

				uniform sampler2D tEquirect;

				varying vec3 vWorldDirection;

				#include <common>

				void main() {

					vec3 direction = normalize( vWorldDirection );

					vec2 sampleUV = equirectUv( direction );

					gl_FragColor = texture2D( tEquirect, sampleUV );

				}
			`},s=new An(5,5,5),r=new xi({name:"CubemapFromEquirect",uniforms:Ns(i.uniforms),vertexShader:i.vertexShader,fragmentShader:i.fragmentShader,side:qe,blending:fi});r.uniforms.tEquirect.value=e;const o=new xe(s,r),a=e.minFilter;return e.minFilter===zi&&(e.minFilter=Cn),new Ng(1,10,this).update(t,o),e.minFilter=a,o.geometry.dispose(),o.material.dispose(),this}clear(t,e,i,s){const r=t.getRenderTarget();for(let o=0;o<6;o++)t.setRenderTarget(this,o),t.clear(e,i,s);t.setRenderTarget(r)}}const qa=new L,kg=new L,Bg=new Ft;class ai{constructor(t=new L(1,0,0),e=0){this.isPlane=!0,this.normal=t,this.constant=e}set(t,e){return this.normal.copy(t),this.constant=e,this}setComponents(t,e,i,s){return this.normal.set(t,e,i),this.constant=s,this}setFromNormalAndCoplanarPoint(t,e){return this.normal.copy(t),this.constant=-e.dot(this.normal),this}setFromCoplanarPoints(t,e,i){const s=qa.subVectors(i,e).cross(kg.subVectors(t,e)).normalize();return this.setFromNormalAndCoplanarPoint(s,t),this}copy(t){return this.normal.copy(t.normal),this.constant=t.constant,this}normalize(){const t=1/this.normal.length();return this.normal.multiplyScalar(t),this.constant*=t,this}negate(){return this.constant*=-1,this.normal.negate(),this}distanceToPoint(t){return this.normal.dot(t)+this.constant}distanceToSphere(t){return this.distanceToPoint(t.center)-t.radius}projectPoint(t,e){return e.copy(t).addScaledVector(this.normal,-this.distanceToPoint(t))}intersectLine(t,e){const i=t.delta(qa),s=this.normal.dot(i);if(s===0)return this.distanceToPoint(t.start)===0?e.copy(t.start):null;const r=-(t.start.dot(this.normal)+this.constant)/s;return r<0||r>1?null:e.copy(t.start).addScaledVector(i,r)}intersectsLine(t){const e=this.distanceToPoint(t.start),i=this.distanceToPoint(t.end);return e<0&&i>0||i<0&&e>0}intersectsBox(t){return t.intersectsPlane(this)}intersectsSphere(t){return t.intersectsPlane(this)}coplanarPoint(t){return t.copy(this.normal).multiplyScalar(-this.constant)}applyMatrix4(t,e){const i=e||Bg.getNormalMatrix(t),s=this.coplanarPoint(qa).applyMatrix4(t),r=this.normal.applyMatrix3(i).normalize();return this.constant=-s.dot(r),this}translate(t){return this.constant-=t.dot(this.normal),this}equals(t){return t.normal.equals(this.normal)&&t.constant===this.constant}clone(){return new this.constructor().copy(this)}}const Ai=new Ir,ro=new L;class zc{constructor(t=new ai,e=new ai,i=new ai,s=new ai,r=new ai,o=new ai){this.planes=[t,e,i,s,r,o]}set(t,e,i,s,r,o){const a=this.planes;return a[0].copy(t),a[1].copy(e),a[2].copy(i),a[3].copy(s),a[4].copy(r),a[5].copy(o),this}copy(t){const e=this.planes;for(let i=0;i<6;i++)e[i].copy(t.planes[i]);return this}setFromProjectionMatrix(t,e=$n){const i=this.planes,s=t.elements,r=s[0],o=s[1],a=s[2],l=s[3],c=s[4],h=s[5],u=s[6],d=s[7],f=s[8],g=s[9],_=s[10],m=s[11],p=s[12],v=s[13],y=s[14],x=s[15];if(i[0].setComponents(l-r,d-c,m-f,x-p).normalize(),i[1].setComponents(l+r,d+c,m+f,x+p).normalize(),i[2].setComponents(l+o,d+h,m+g,x+v).normalize(),i[3].setComponents(l-o,d-h,m-g,x-v).normalize(),i[4].setComponents(l-a,d-u,m-_,x-y).normalize(),e===$n)i[5].setComponents(l+a,d+u,m+_,x+y).normalize();else if(e===Zo)i[5].setComponents(a,u,_,y).normalize();else throw new Error("THREE.Frustum.setFromProjectionMatrix(): Invalid coordinate system: "+e);return this}intersectsObject(t){if(t.boundingSphere!==void 0)t.boundingSphere===null&&t.computeBoundingSphere(),Ai.copy(t.boundingSphere).applyMatrix4(t.matrixWorld);else{const e=t.geometry;e.boundingSphere===null&&e.computeBoundingSphere(),Ai.copy(e.boundingSphere).applyMatrix4(t.matrixWorld)}return this.intersectsSphere(Ai)}intersectsSprite(t){return Ai.center.set(0,0,0),Ai.radius=.7071067811865476,Ai.applyMatrix4(t.matrixWorld),this.intersectsSphere(Ai)}intersectsSphere(t){const e=this.planes,i=t.center,s=-t.radius;for(let r=0;r<6;r++)if(e[r].distanceToPoint(i)<s)return!1;return!0}intersectsBox(t){const e=this.planes;for(let i=0;i<6;i++){const s=e[i];if(ro.x=s.normal.x>0?t.max.x:t.min.x,ro.y=s.normal.y>0?t.max.y:t.min.y,ro.z=s.normal.z>0?t.max.z:t.min.z,s.distanceToPoint(ro)<0)return!1}return!0}containsPoint(t){const e=this.planes;for(let i=0;i<6;i++)if(e[i].distanceToPoint(t)<0)return!1;return!0}clone(){return new this.constructor().copy(this)}}function Hf(){let n=null,t=!1,e=null,i=null;function s(r,o){e(r,o),i=n.requestAnimationFrame(s)}return{start:function(){t!==!0&&e!==null&&(i=n.requestAnimationFrame(s),t=!0)},stop:function(){n.cancelAnimationFrame(i),t=!1},setAnimationLoop:function(r){e=r},setContext:function(r){n=r}}}function zg(n){const t=new WeakMap;function e(a,l){const c=a.array,h=a.usage,u=c.byteLength,d=n.createBuffer();n.bindBuffer(l,d),n.bufferData(l,c,h),a.onUploadCallback();let f;if(c instanceof Float32Array)f=n.FLOAT;else if(c instanceof Uint16Array)a.isFloat16BufferAttribute?f=n.HALF_FLOAT:f=n.UNSIGNED_SHORT;else if(c instanceof Int16Array)f=n.SHORT;else if(c instanceof Uint32Array)f=n.UNSIGNED_INT;else if(c instanceof Int32Array)f=n.INT;else if(c instanceof Int8Array)f=n.BYTE;else if(c instanceof Uint8Array)f=n.UNSIGNED_BYTE;else if(c instanceof Uint8ClampedArray)f=n.UNSIGNED_BYTE;else throw new Error("THREE.WebGLAttributes: Unsupported buffer data format: "+c);return{buffer:d,type:f,bytesPerElement:c.BYTES_PER_ELEMENT,version:a.version,size:u}}function i(a,l,c){const h=l.array,u=l.updateRanges;if(n.bindBuffer(c,a),u.length===0)n.bufferSubData(c,0,h);else{u.sort((f,g)=>f.start-g.start);let d=0;for(let f=1;f<u.length;f++){const g=u[d],_=u[f];_.start<=g.start+g.count+1?g.count=Math.max(g.count,_.start+_.count-g.start):(++d,u[d]=_)}u.length=d+1;for(let f=0,g=u.length;f<g;f++){const _=u[f];n.bufferSubData(c,_.start*h.BYTES_PER_ELEMENT,h,_.start,_.count)}l.clearUpdateRanges()}l.onUploadCallback()}function s(a){return a.isInterleavedBufferAttribute&&(a=a.data),t.get(a)}function r(a){a.isInterleavedBufferAttribute&&(a=a.data);const l=t.get(a);l&&(n.deleteBuffer(l.buffer),t.delete(a))}function o(a,l){if(a.isInterleavedBufferAttribute&&(a=a.data),a.isGLBufferAttribute){const h=t.get(a);(!h||h.version<a.version)&&t.set(a,{buffer:a.buffer,type:a.type,bytesPerElement:a.elementSize,version:a.version});return}const c=t.get(a);if(c===void 0)t.set(a,e(a,l));else if(c.version<a.version){if(c.size!==a.array.byteLength)throw new Error("THREE.WebGLAttributes: The size of the buffer attribute's array buffer does not match the original size. Resizing buffer attributes is not supported.");i(c.buffer,a,l),c.version=a.version}}return{get:s,remove:r,update:o}}class Ur extends Te{constructor(t=1,e=1,i=1,s=1){super(),this.type="PlaneGeometry",this.parameters={width:t,height:e,widthSegments:i,heightSegments:s};const r=t/2,o=e/2,a=Math.floor(i),l=Math.floor(s),c=a+1,h=l+1,u=t/a,d=e/l,f=[],g=[],_=[],m=[];for(let p=0;p<h;p++){const v=p*d-o;for(let y=0;y<c;y++){const x=y*u-r;g.push(x,-v,0),_.push(0,0,1),m.push(y/a),m.push(1-p/l)}}for(let p=0;p<l;p++)for(let v=0;v<a;v++){const y=v+c*p,x=v+c*(p+1),A=v+1+c*(p+1),w=v+1+c*p;f.push(y,x,w),f.push(x,A,w)}this.setIndex(f),this.setAttribute("position",new Se(g,3)),this.setAttribute("normal",new Se(_,3)),this.setAttribute("uv",new Se(m,2))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new Ur(t.width,t.height,t.widthSegments,t.heightSegments)}}var Hg=`#ifdef USE_ALPHAHASH
	if ( diffuseColor.a < getAlphaHashThreshold( vPosition ) ) discard;
#endif`,Vg=`#ifdef USE_ALPHAHASH
	const float ALPHA_HASH_SCALE = 0.05;
	float hash2D( vec2 value ) {
		return fract( 1.0e4 * sin( 17.0 * value.x + 0.1 * value.y ) * ( 0.1 + abs( sin( 13.0 * value.y + value.x ) ) ) );
	}
	float hash3D( vec3 value ) {
		return hash2D( vec2( hash2D( value.xy ), value.z ) );
	}
	float getAlphaHashThreshold( vec3 position ) {
		float maxDeriv = max(
			length( dFdx( position.xyz ) ),
			length( dFdy( position.xyz ) )
		);
		float pixScale = 1.0 / ( ALPHA_HASH_SCALE * maxDeriv );
		vec2 pixScales = vec2(
			exp2( floor( log2( pixScale ) ) ),
			exp2( ceil( log2( pixScale ) ) )
		);
		vec2 alpha = vec2(
			hash3D( floor( pixScales.x * position.xyz ) ),
			hash3D( floor( pixScales.y * position.xyz ) )
		);
		float lerpFactor = fract( log2( pixScale ) );
		float x = ( 1.0 - lerpFactor ) * alpha.x + lerpFactor * alpha.y;
		float a = min( lerpFactor, 1.0 - lerpFactor );
		vec3 cases = vec3(
			x * x / ( 2.0 * a * ( 1.0 - a ) ),
			( x - 0.5 * a ) / ( 1.0 - a ),
			1.0 - ( ( 1.0 - x ) * ( 1.0 - x ) / ( 2.0 * a * ( 1.0 - a ) ) )
		);
		float threshold = ( x < ( 1.0 - a ) )
			? ( ( x < a ) ? cases.x : cases.y )
			: cases.z;
		return clamp( threshold , 1.0e-6, 1.0 );
	}
#endif`,Gg=`#ifdef USE_ALPHAMAP
	diffuseColor.a *= texture2D( alphaMap, vAlphaMapUv ).g;
#endif`,Wg=`#ifdef USE_ALPHAMAP
	uniform sampler2D alphaMap;
#endif`,Xg=`#ifdef USE_ALPHATEST
	#ifdef ALPHA_TO_COVERAGE
	diffuseColor.a = smoothstep( alphaTest, alphaTest + fwidth( diffuseColor.a ), diffuseColor.a );
	if ( diffuseColor.a == 0.0 ) discard;
	#else
	if ( diffuseColor.a < alphaTest ) discard;
	#endif
#endif`,jg=`#ifdef USE_ALPHATEST
	uniform float alphaTest;
#endif`,$g=`#ifdef USE_AOMAP
	float ambientOcclusion = ( texture2D( aoMap, vAoMapUv ).r - 1.0 ) * aoMapIntensity + 1.0;
	reflectedLight.indirectDiffuse *= ambientOcclusion;
	#if defined( USE_CLEARCOAT ) 
		clearcoatSpecularIndirect *= ambientOcclusion;
	#endif
	#if defined( USE_SHEEN ) 
		sheenSpecularIndirect *= ambientOcclusion;
	#endif
	#if defined( USE_ENVMAP ) && defined( STANDARD )
		float dotNV = saturate( dot( geometryNormal, geometryViewDir ) );
		reflectedLight.indirectSpecular *= computeSpecularOcclusion( dotNV, ambientOcclusion, material.roughness );
	#endif
#endif`,Yg=`#ifdef USE_AOMAP
	uniform sampler2D aoMap;
	uniform float aoMapIntensity;
#endif`,qg=`#ifdef USE_BATCHING
	#if ! defined( GL_ANGLE_multi_draw )
	#define gl_DrawID _gl_DrawID
	uniform int _gl_DrawID;
	#endif
	uniform highp sampler2D batchingTexture;
	uniform highp usampler2D batchingIdTexture;
	mat4 getBatchingMatrix( const in float i ) {
		int size = textureSize( batchingTexture, 0 ).x;
		int j = int( i ) * 4;
		int x = j % size;
		int y = j / size;
		vec4 v1 = texelFetch( batchingTexture, ivec2( x, y ), 0 );
		vec4 v2 = texelFetch( batchingTexture, ivec2( x + 1, y ), 0 );
		vec4 v3 = texelFetch( batchingTexture, ivec2( x + 2, y ), 0 );
		vec4 v4 = texelFetch( batchingTexture, ivec2( x + 3, y ), 0 );
		return mat4( v1, v2, v3, v4 );
	}
	float getIndirectIndex( const in int i ) {
		int size = textureSize( batchingIdTexture, 0 ).x;
		int x = i % size;
		int y = i / size;
		return float( texelFetch( batchingIdTexture, ivec2( x, y ), 0 ).r );
	}
#endif
#ifdef USE_BATCHING_COLOR
	uniform sampler2D batchingColorTexture;
	vec3 getBatchingColor( const in float i ) {
		int size = textureSize( batchingColorTexture, 0 ).x;
		int j = int( i );
		int x = j % size;
		int y = j / size;
		return texelFetch( batchingColorTexture, ivec2( x, y ), 0 ).rgb;
	}
#endif`,Kg=`#ifdef USE_BATCHING
	mat4 batchingMatrix = getBatchingMatrix( getIndirectIndex( gl_DrawID ) );
#endif`,Zg=`vec3 transformed = vec3( position );
#ifdef USE_ALPHAHASH
	vPosition = vec3( position );
#endif`,Jg=`vec3 objectNormal = vec3( normal );
#ifdef USE_TANGENT
	vec3 objectTangent = vec3( tangent.xyz );
#endif`,Qg=`float G_BlinnPhong_Implicit( ) {
	return 0.25;
}
float D_BlinnPhong( const in float shininess, const in float dotNH ) {
	return RECIPROCAL_PI * ( shininess * 0.5 + 1.0 ) * pow( dotNH, shininess );
}
vec3 BRDF_BlinnPhong( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, const in vec3 specularColor, const in float shininess ) {
	vec3 halfDir = normalize( lightDir + viewDir );
	float dotNH = saturate( dot( normal, halfDir ) );
	float dotVH = saturate( dot( viewDir, halfDir ) );
	vec3 F = F_Schlick( specularColor, 1.0, dotVH );
	float G = G_BlinnPhong_Implicit( );
	float D = D_BlinnPhong( shininess, dotNH );
	return F * ( G * D );
} // validated`,t_=`#ifdef USE_IRIDESCENCE
	const mat3 XYZ_TO_REC709 = mat3(
		 3.2404542, -0.9692660,  0.0556434,
		-1.5371385,  1.8760108, -0.2040259,
		-0.4985314,  0.0415560,  1.0572252
	);
	vec3 Fresnel0ToIor( vec3 fresnel0 ) {
		vec3 sqrtF0 = sqrt( fresnel0 );
		return ( vec3( 1.0 ) + sqrtF0 ) / ( vec3( 1.0 ) - sqrtF0 );
	}
	vec3 IorToFresnel0( vec3 transmittedIor, float incidentIor ) {
		return pow2( ( transmittedIor - vec3( incidentIor ) ) / ( transmittedIor + vec3( incidentIor ) ) );
	}
	float IorToFresnel0( float transmittedIor, float incidentIor ) {
		return pow2( ( transmittedIor - incidentIor ) / ( transmittedIor + incidentIor ));
	}
	vec3 evalSensitivity( float OPD, vec3 shift ) {
		float phase = 2.0 * PI * OPD * 1.0e-9;
		vec3 val = vec3( 5.4856e-13, 4.4201e-13, 5.2481e-13 );
		vec3 pos = vec3( 1.6810e+06, 1.7953e+06, 2.2084e+06 );
		vec3 var = vec3( 4.3278e+09, 9.3046e+09, 6.6121e+09 );
		vec3 xyz = val * sqrt( 2.0 * PI * var ) * cos( pos * phase + shift ) * exp( - pow2( phase ) * var );
		xyz.x += 9.7470e-14 * sqrt( 2.0 * PI * 4.5282e+09 ) * cos( 2.2399e+06 * phase + shift[ 0 ] ) * exp( - 4.5282e+09 * pow2( phase ) );
		xyz /= 1.0685e-7;
		vec3 rgb = XYZ_TO_REC709 * xyz;
		return rgb;
	}
	vec3 evalIridescence( float outsideIOR, float eta2, float cosTheta1, float thinFilmThickness, vec3 baseF0 ) {
		vec3 I;
		float iridescenceIOR = mix( outsideIOR, eta2, smoothstep( 0.0, 0.03, thinFilmThickness ) );
		float sinTheta2Sq = pow2( outsideIOR / iridescenceIOR ) * ( 1.0 - pow2( cosTheta1 ) );
		float cosTheta2Sq = 1.0 - sinTheta2Sq;
		if ( cosTheta2Sq < 0.0 ) {
			return vec3( 1.0 );
		}
		float cosTheta2 = sqrt( cosTheta2Sq );
		float R0 = IorToFresnel0( iridescenceIOR, outsideIOR );
		float R12 = F_Schlick( R0, 1.0, cosTheta1 );
		float T121 = 1.0 - R12;
		float phi12 = 0.0;
		if ( iridescenceIOR < outsideIOR ) phi12 = PI;
		float phi21 = PI - phi12;
		vec3 baseIOR = Fresnel0ToIor( clamp( baseF0, 0.0, 0.9999 ) );		vec3 R1 = IorToFresnel0( baseIOR, iridescenceIOR );
		vec3 R23 = F_Schlick( R1, 1.0, cosTheta2 );
		vec3 phi23 = vec3( 0.0 );
		if ( baseIOR[ 0 ] < iridescenceIOR ) phi23[ 0 ] = PI;
		if ( baseIOR[ 1 ] < iridescenceIOR ) phi23[ 1 ] = PI;
		if ( baseIOR[ 2 ] < iridescenceIOR ) phi23[ 2 ] = PI;
		float OPD = 2.0 * iridescenceIOR * thinFilmThickness * cosTheta2;
		vec3 phi = vec3( phi21 ) + phi23;
		vec3 R123 = clamp( R12 * R23, 1e-5, 0.9999 );
		vec3 r123 = sqrt( R123 );
		vec3 Rs = pow2( T121 ) * R23 / ( vec3( 1.0 ) - R123 );
		vec3 C0 = R12 + Rs;
		I = C0;
		vec3 Cm = Rs - T121;
		for ( int m = 1; m <= 2; ++ m ) {
			Cm *= r123;
			vec3 Sm = 2.0 * evalSensitivity( float( m ) * OPD, float( m ) * phi );
			I += Cm * Sm;
		}
		return max( I, vec3( 0.0 ) );
	}
#endif`,e_=`#ifdef USE_BUMPMAP
	uniform sampler2D bumpMap;
	uniform float bumpScale;
	vec2 dHdxy_fwd() {
		vec2 dSTdx = dFdx( vBumpMapUv );
		vec2 dSTdy = dFdy( vBumpMapUv );
		float Hll = bumpScale * texture2D( bumpMap, vBumpMapUv ).x;
		float dBx = bumpScale * texture2D( bumpMap, vBumpMapUv + dSTdx ).x - Hll;
		float dBy = bumpScale * texture2D( bumpMap, vBumpMapUv + dSTdy ).x - Hll;
		return vec2( dBx, dBy );
	}
	vec3 perturbNormalArb( vec3 surf_pos, vec3 surf_norm, vec2 dHdxy, float faceDirection ) {
		vec3 vSigmaX = normalize( dFdx( surf_pos.xyz ) );
		vec3 vSigmaY = normalize( dFdy( surf_pos.xyz ) );
		vec3 vN = surf_norm;
		vec3 R1 = cross( vSigmaY, vN );
		vec3 R2 = cross( vN, vSigmaX );
		float fDet = dot( vSigmaX, R1 ) * faceDirection;
		vec3 vGrad = sign( fDet ) * ( dHdxy.x * R1 + dHdxy.y * R2 );
		return normalize( abs( fDet ) * surf_norm - vGrad );
	}
#endif`,n_=`#if NUM_CLIPPING_PLANES > 0
	vec4 plane;
	#ifdef ALPHA_TO_COVERAGE
		float distanceToPlane, distanceGradient;
		float clipOpacity = 1.0;
		#pragma unroll_loop_start
		for ( int i = 0; i < UNION_CLIPPING_PLANES; i ++ ) {
			plane = clippingPlanes[ i ];
			distanceToPlane = - dot( vClipPosition, plane.xyz ) + plane.w;
			distanceGradient = fwidth( distanceToPlane ) / 2.0;
			clipOpacity *= smoothstep( - distanceGradient, distanceGradient, distanceToPlane );
			if ( clipOpacity == 0.0 ) discard;
		}
		#pragma unroll_loop_end
		#if UNION_CLIPPING_PLANES < NUM_CLIPPING_PLANES
			float unionClipOpacity = 1.0;
			#pragma unroll_loop_start
			for ( int i = UNION_CLIPPING_PLANES; i < NUM_CLIPPING_PLANES; i ++ ) {
				plane = clippingPlanes[ i ];
				distanceToPlane = - dot( vClipPosition, plane.xyz ) + plane.w;
				distanceGradient = fwidth( distanceToPlane ) / 2.0;
				unionClipOpacity *= 1.0 - smoothstep( - distanceGradient, distanceGradient, distanceToPlane );
			}
			#pragma unroll_loop_end
			clipOpacity *= 1.0 - unionClipOpacity;
		#endif
		diffuseColor.a *= clipOpacity;
		if ( diffuseColor.a == 0.0 ) discard;
	#else
		#pragma unroll_loop_start
		for ( int i = 0; i < UNION_CLIPPING_PLANES; i ++ ) {
			plane = clippingPlanes[ i ];
			if ( dot( vClipPosition, plane.xyz ) > plane.w ) discard;
		}
		#pragma unroll_loop_end
		#if UNION_CLIPPING_PLANES < NUM_CLIPPING_PLANES
			bool clipped = true;
			#pragma unroll_loop_start
			for ( int i = UNION_CLIPPING_PLANES; i < NUM_CLIPPING_PLANES; i ++ ) {
				plane = clippingPlanes[ i ];
				clipped = ( dot( vClipPosition, plane.xyz ) > plane.w ) && clipped;
			}
			#pragma unroll_loop_end
			if ( clipped ) discard;
		#endif
	#endif
#endif`,i_=`#if NUM_CLIPPING_PLANES > 0
	varying vec3 vClipPosition;
	uniform vec4 clippingPlanes[ NUM_CLIPPING_PLANES ];
#endif`,s_=`#if NUM_CLIPPING_PLANES > 0
	varying vec3 vClipPosition;
#endif`,r_=`#if NUM_CLIPPING_PLANES > 0
	vClipPosition = - mvPosition.xyz;
#endif`,o_=`#if defined( USE_COLOR_ALPHA )
	diffuseColor *= vColor;
#elif defined( USE_COLOR )
	diffuseColor.rgb *= vColor;
#endif`,a_=`#if defined( USE_COLOR_ALPHA )
	varying vec4 vColor;
#elif defined( USE_COLOR )
	varying vec3 vColor;
#endif`,l_=`#if defined( USE_COLOR_ALPHA )
	varying vec4 vColor;
#elif defined( USE_COLOR ) || defined( USE_INSTANCING_COLOR ) || defined( USE_BATCHING_COLOR )
	varying vec3 vColor;
#endif`,c_=`#if defined( USE_COLOR_ALPHA )
	vColor = vec4( 1.0 );
#elif defined( USE_COLOR ) || defined( USE_INSTANCING_COLOR ) || defined( USE_BATCHING_COLOR )
	vColor = vec3( 1.0 );
#endif
#ifdef USE_COLOR
	vColor *= color;
#endif
#ifdef USE_INSTANCING_COLOR
	vColor.xyz *= instanceColor.xyz;
#endif
#ifdef USE_BATCHING_COLOR
	vec3 batchingColor = getBatchingColor( getIndirectIndex( gl_DrawID ) );
	vColor.xyz *= batchingColor.xyz;
#endif`,h_=`#define PI 3.141592653589793
#define PI2 6.283185307179586
#define PI_HALF 1.5707963267948966
#define RECIPROCAL_PI 0.3183098861837907
#define RECIPROCAL_PI2 0.15915494309189535
#define EPSILON 1e-6
#ifndef saturate
#define saturate( a ) clamp( a, 0.0, 1.0 )
#endif
#define whiteComplement( a ) ( 1.0 - saturate( a ) )
float pow2( const in float x ) { return x*x; }
vec3 pow2( const in vec3 x ) { return x*x; }
float pow3( const in float x ) { return x*x*x; }
float pow4( const in float x ) { float x2 = x*x; return x2*x2; }
float max3( const in vec3 v ) { return max( max( v.x, v.y ), v.z ); }
float average( const in vec3 v ) { return dot( v, vec3( 0.3333333 ) ); }
highp float rand( const in vec2 uv ) {
	const highp float a = 12.9898, b = 78.233, c = 43758.5453;
	highp float dt = dot( uv.xy, vec2( a,b ) ), sn = mod( dt, PI );
	return fract( sin( sn ) * c );
}
#ifdef HIGH_PRECISION
	float precisionSafeLength( vec3 v ) { return length( v ); }
#else
	float precisionSafeLength( vec3 v ) {
		float maxComponent = max3( abs( v ) );
		return length( v / maxComponent ) * maxComponent;
	}
#endif
struct IncidentLight {
	vec3 color;
	vec3 direction;
	bool visible;
};
struct ReflectedLight {
	vec3 directDiffuse;
	vec3 directSpecular;
	vec3 indirectDiffuse;
	vec3 indirectSpecular;
};
#ifdef USE_ALPHAHASH
	varying vec3 vPosition;
#endif
vec3 transformDirection( in vec3 dir, in mat4 matrix ) {
	return normalize( ( matrix * vec4( dir, 0.0 ) ).xyz );
}
vec3 inverseTransformDirection( in vec3 dir, in mat4 matrix ) {
	return normalize( ( vec4( dir, 0.0 ) * matrix ).xyz );
}
mat3 transposeMat3( const in mat3 m ) {
	mat3 tmp;
	tmp[ 0 ] = vec3( m[ 0 ].x, m[ 1 ].x, m[ 2 ].x );
	tmp[ 1 ] = vec3( m[ 0 ].y, m[ 1 ].y, m[ 2 ].y );
	tmp[ 2 ] = vec3( m[ 0 ].z, m[ 1 ].z, m[ 2 ].z );
	return tmp;
}
bool isPerspectiveMatrix( mat4 m ) {
	return m[ 2 ][ 3 ] == - 1.0;
}
vec2 equirectUv( in vec3 dir ) {
	float u = atan( dir.z, dir.x ) * RECIPROCAL_PI2 + 0.5;
	float v = asin( clamp( dir.y, - 1.0, 1.0 ) ) * RECIPROCAL_PI + 0.5;
	return vec2( u, v );
}
vec3 BRDF_Lambert( const in vec3 diffuseColor ) {
	return RECIPROCAL_PI * diffuseColor;
}
vec3 F_Schlick( const in vec3 f0, const in float f90, const in float dotVH ) {
	float fresnel = exp2( ( - 5.55473 * dotVH - 6.98316 ) * dotVH );
	return f0 * ( 1.0 - fresnel ) + ( f90 * fresnel );
}
float F_Schlick( const in float f0, const in float f90, const in float dotVH ) {
	float fresnel = exp2( ( - 5.55473 * dotVH - 6.98316 ) * dotVH );
	return f0 * ( 1.0 - fresnel ) + ( f90 * fresnel );
} // validated`,u_=`#ifdef ENVMAP_TYPE_CUBE_UV
	#define cubeUV_minMipLevel 4.0
	#define cubeUV_minTileSize 16.0
	float getFace( vec3 direction ) {
		vec3 absDirection = abs( direction );
		float face = - 1.0;
		if ( absDirection.x > absDirection.z ) {
			if ( absDirection.x > absDirection.y )
				face = direction.x > 0.0 ? 0.0 : 3.0;
			else
				face = direction.y > 0.0 ? 1.0 : 4.0;
		} else {
			if ( absDirection.z > absDirection.y )
				face = direction.z > 0.0 ? 2.0 : 5.0;
			else
				face = direction.y > 0.0 ? 1.0 : 4.0;
		}
		return face;
	}
	vec2 getUV( vec3 direction, float face ) {
		vec2 uv;
		if ( face == 0.0 ) {
			uv = vec2( direction.z, direction.y ) / abs( direction.x );
		} else if ( face == 1.0 ) {
			uv = vec2( - direction.x, - direction.z ) / abs( direction.y );
		} else if ( face == 2.0 ) {
			uv = vec2( - direction.x, direction.y ) / abs( direction.z );
		} else if ( face == 3.0 ) {
			uv = vec2( - direction.z, direction.y ) / abs( direction.x );
		} else if ( face == 4.0 ) {
			uv = vec2( - direction.x, direction.z ) / abs( direction.y );
		} else {
			uv = vec2( direction.x, direction.y ) / abs( direction.z );
		}
		return 0.5 * ( uv + 1.0 );
	}
	vec3 bilinearCubeUV( sampler2D envMap, vec3 direction, float mipInt ) {
		float face = getFace( direction );
		float filterInt = max( cubeUV_minMipLevel - mipInt, 0.0 );
		mipInt = max( mipInt, cubeUV_minMipLevel );
		float faceSize = exp2( mipInt );
		highp vec2 uv = getUV( direction, face ) * ( faceSize - 2.0 ) + 1.0;
		if ( face > 2.0 ) {
			uv.y += faceSize;
			face -= 3.0;
		}
		uv.x += face * faceSize;
		uv.x += filterInt * 3.0 * cubeUV_minTileSize;
		uv.y += 4.0 * ( exp2( CUBEUV_MAX_MIP ) - faceSize );
		uv.x *= CUBEUV_TEXEL_WIDTH;
		uv.y *= CUBEUV_TEXEL_HEIGHT;
		#ifdef texture2DGradEXT
			return texture2DGradEXT( envMap, uv, vec2( 0.0 ), vec2( 0.0 ) ).rgb;
		#else
			return texture2D( envMap, uv ).rgb;
		#endif
	}
	#define cubeUV_r0 1.0
	#define cubeUV_m0 - 2.0
	#define cubeUV_r1 0.8
	#define cubeUV_m1 - 1.0
	#define cubeUV_r4 0.4
	#define cubeUV_m4 2.0
	#define cubeUV_r5 0.305
	#define cubeUV_m5 3.0
	#define cubeUV_r6 0.21
	#define cubeUV_m6 4.0
	float roughnessToMip( float roughness ) {
		float mip = 0.0;
		if ( roughness >= cubeUV_r1 ) {
			mip = ( cubeUV_r0 - roughness ) * ( cubeUV_m1 - cubeUV_m0 ) / ( cubeUV_r0 - cubeUV_r1 ) + cubeUV_m0;
		} else if ( roughness >= cubeUV_r4 ) {
			mip = ( cubeUV_r1 - roughness ) * ( cubeUV_m4 - cubeUV_m1 ) / ( cubeUV_r1 - cubeUV_r4 ) + cubeUV_m1;
		} else if ( roughness >= cubeUV_r5 ) {
			mip = ( cubeUV_r4 - roughness ) * ( cubeUV_m5 - cubeUV_m4 ) / ( cubeUV_r4 - cubeUV_r5 ) + cubeUV_m4;
		} else if ( roughness >= cubeUV_r6 ) {
			mip = ( cubeUV_r5 - roughness ) * ( cubeUV_m6 - cubeUV_m5 ) / ( cubeUV_r5 - cubeUV_r6 ) + cubeUV_m5;
		} else {
			mip = - 2.0 * log2( 1.16 * roughness );		}
		return mip;
	}
	vec4 textureCubeUV( sampler2D envMap, vec3 sampleDir, float roughness ) {
		float mip = clamp( roughnessToMip( roughness ), cubeUV_m0, CUBEUV_MAX_MIP );
		float mipF = fract( mip );
		float mipInt = floor( mip );
		vec3 color0 = bilinearCubeUV( envMap, sampleDir, mipInt );
		if ( mipF == 0.0 ) {
			return vec4( color0, 1.0 );
		} else {
			vec3 color1 = bilinearCubeUV( envMap, sampleDir, mipInt + 1.0 );
			return vec4( mix( color0, color1, mipF ), 1.0 );
		}
	}
#endif`,d_=`vec3 transformedNormal = objectNormal;
#ifdef USE_TANGENT
	vec3 transformedTangent = objectTangent;
#endif
#ifdef USE_BATCHING
	mat3 bm = mat3( batchingMatrix );
	transformedNormal /= vec3( dot( bm[ 0 ], bm[ 0 ] ), dot( bm[ 1 ], bm[ 1 ] ), dot( bm[ 2 ], bm[ 2 ] ) );
	transformedNormal = bm * transformedNormal;
	#ifdef USE_TANGENT
		transformedTangent = bm * transformedTangent;
	#endif
#endif
#ifdef USE_INSTANCING
	mat3 im = mat3( instanceMatrix );
	transformedNormal /= vec3( dot( im[ 0 ], im[ 0 ] ), dot( im[ 1 ], im[ 1 ] ), dot( im[ 2 ], im[ 2 ] ) );
	transformedNormal = im * transformedNormal;
	#ifdef USE_TANGENT
		transformedTangent = im * transformedTangent;
	#endif
#endif
transformedNormal = normalMatrix * transformedNormal;
#ifdef FLIP_SIDED
	transformedNormal = - transformedNormal;
#endif
#ifdef USE_TANGENT
	transformedTangent = ( modelViewMatrix * vec4( transformedTangent, 0.0 ) ).xyz;
	#ifdef FLIP_SIDED
		transformedTangent = - transformedTangent;
	#endif
#endif`,f_=`#ifdef USE_DISPLACEMENTMAP
	uniform sampler2D displacementMap;
	uniform float displacementScale;
	uniform float displacementBias;
#endif`,p_=`#ifdef USE_DISPLACEMENTMAP
	transformed += normalize( objectNormal ) * ( texture2D( displacementMap, vDisplacementMapUv ).x * displacementScale + displacementBias );
#endif`,m_=`#ifdef USE_EMISSIVEMAP
	vec4 emissiveColor = texture2D( emissiveMap, vEmissiveMapUv );
	#ifdef DECODE_VIDEO_TEXTURE_EMISSIVE
		emissiveColor = sRGBTransferEOTF( emissiveColor );
	#endif
	totalEmissiveRadiance *= emissiveColor.rgb;
#endif`,g_=`#ifdef USE_EMISSIVEMAP
	uniform sampler2D emissiveMap;
#endif`,__="gl_FragColor = linearToOutputTexel( gl_FragColor );",x_=`vec4 LinearTransferOETF( in vec4 value ) {
	return value;
}
vec4 sRGBTransferEOTF( in vec4 value ) {
	return vec4( mix( pow( value.rgb * 0.9478672986 + vec3( 0.0521327014 ), vec3( 2.4 ) ), value.rgb * 0.0773993808, vec3( lessThanEqual( value.rgb, vec3( 0.04045 ) ) ) ), value.a );
}
vec4 sRGBTransferOETF( in vec4 value ) {
	return vec4( mix( pow( value.rgb, vec3( 0.41666 ) ) * 1.055 - vec3( 0.055 ), value.rgb * 12.92, vec3( lessThanEqual( value.rgb, vec3( 0.0031308 ) ) ) ), value.a );
}`,v_=`#ifdef USE_ENVMAP
	#ifdef ENV_WORLDPOS
		vec3 cameraToFrag;
		if ( isOrthographic ) {
			cameraToFrag = normalize( vec3( - viewMatrix[ 0 ][ 2 ], - viewMatrix[ 1 ][ 2 ], - viewMatrix[ 2 ][ 2 ] ) );
		} else {
			cameraToFrag = normalize( vWorldPosition - cameraPosition );
		}
		vec3 worldNormal = inverseTransformDirection( normal, viewMatrix );
		#ifdef ENVMAP_MODE_REFLECTION
			vec3 reflectVec = reflect( cameraToFrag, worldNormal );
		#else
			vec3 reflectVec = refract( cameraToFrag, worldNormal, refractionRatio );
		#endif
	#else
		vec3 reflectVec = vReflect;
	#endif
	#ifdef ENVMAP_TYPE_CUBE
		vec4 envColor = textureCube( envMap, envMapRotation * vec3( flipEnvMap * reflectVec.x, reflectVec.yz ) );
	#else
		vec4 envColor = vec4( 0.0 );
	#endif
	#ifdef ENVMAP_BLENDING_MULTIPLY
		outgoingLight = mix( outgoingLight, outgoingLight * envColor.xyz, specularStrength * reflectivity );
	#elif defined( ENVMAP_BLENDING_MIX )
		outgoingLight = mix( outgoingLight, envColor.xyz, specularStrength * reflectivity );
	#elif defined( ENVMAP_BLENDING_ADD )
		outgoingLight += envColor.xyz * specularStrength * reflectivity;
	#endif
#endif`,b_=`#ifdef USE_ENVMAP
	uniform float envMapIntensity;
	uniform float flipEnvMap;
	uniform mat3 envMapRotation;
	#ifdef ENVMAP_TYPE_CUBE
		uniform samplerCube envMap;
	#else
		uniform sampler2D envMap;
	#endif
	
#endif`,y_=`#ifdef USE_ENVMAP
	uniform float reflectivity;
	#if defined( USE_BUMPMAP ) || defined( USE_NORMALMAP ) || defined( PHONG ) || defined( LAMBERT )
		#define ENV_WORLDPOS
	#endif
	#ifdef ENV_WORLDPOS
		varying vec3 vWorldPosition;
		uniform float refractionRatio;
	#else
		varying vec3 vReflect;
	#endif
#endif`,M_=`#ifdef USE_ENVMAP
	#if defined( USE_BUMPMAP ) || defined( USE_NORMALMAP ) || defined( PHONG ) || defined( LAMBERT )
		#define ENV_WORLDPOS
	#endif
	#ifdef ENV_WORLDPOS
		
		varying vec3 vWorldPosition;
	#else
		varying vec3 vReflect;
		uniform float refractionRatio;
	#endif
#endif`,S_=`#ifdef USE_ENVMAP
	#ifdef ENV_WORLDPOS
		vWorldPosition = worldPosition.xyz;
	#else
		vec3 cameraToVertex;
		if ( isOrthographic ) {
			cameraToVertex = normalize( vec3( - viewMatrix[ 0 ][ 2 ], - viewMatrix[ 1 ][ 2 ], - viewMatrix[ 2 ][ 2 ] ) );
		} else {
			cameraToVertex = normalize( worldPosition.xyz - cameraPosition );
		}
		vec3 worldNormal = inverseTransformDirection( transformedNormal, viewMatrix );
		#ifdef ENVMAP_MODE_REFLECTION
			vReflect = reflect( cameraToVertex, worldNormal );
		#else
			vReflect = refract( cameraToVertex, worldNormal, refractionRatio );
		#endif
	#endif
#endif`,E_=`#ifdef USE_FOG
	vFogDepth = - mvPosition.z;
#endif`,T_=`#ifdef USE_FOG
	varying float vFogDepth;
#endif`,w_=`#ifdef USE_FOG
	#ifdef FOG_EXP2
		float fogFactor = 1.0 - exp( - fogDensity * fogDensity * vFogDepth * vFogDepth );
	#else
		float fogFactor = smoothstep( fogNear, fogFar, vFogDepth );
	#endif
	gl_FragColor.rgb = mix( gl_FragColor.rgb, fogColor, fogFactor );
#endif`,A_=`#ifdef USE_FOG
	uniform vec3 fogColor;
	varying float vFogDepth;
	#ifdef FOG_EXP2
		uniform float fogDensity;
	#else
		uniform float fogNear;
		uniform float fogFar;
	#endif
#endif`,C_=`#ifdef USE_GRADIENTMAP
	uniform sampler2D gradientMap;
#endif
vec3 getGradientIrradiance( vec3 normal, vec3 lightDirection ) {
	float dotNL = dot( normal, lightDirection );
	vec2 coord = vec2( dotNL * 0.5 + 0.5, 0.0 );
	#ifdef USE_GRADIENTMAP
		return vec3( texture2D( gradientMap, coord ).r );
	#else
		vec2 fw = fwidth( coord ) * 0.5;
		return mix( vec3( 0.7 ), vec3( 1.0 ), smoothstep( 0.7 - fw.x, 0.7 + fw.x, coord.x ) );
	#endif
}`,R_=`#ifdef USE_LIGHTMAP
	uniform sampler2D lightMap;
	uniform float lightMapIntensity;
#endif`,P_=`LambertMaterial material;
material.diffuseColor = diffuseColor.rgb;
material.specularStrength = specularStrength;`,D_=`varying vec3 vViewPosition;
struct LambertMaterial {
	vec3 diffuseColor;
	float specularStrength;
};
void RE_Direct_Lambert( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in LambertMaterial material, inout ReflectedLight reflectedLight ) {
	float dotNL = saturate( dot( geometryNormal, directLight.direction ) );
	vec3 irradiance = dotNL * directLight.color;
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectDiffuse_Lambert( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in LambertMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
#define RE_Direct				RE_Direct_Lambert
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Lambert`,L_=`uniform bool receiveShadow;
uniform vec3 ambientLightColor;
#if defined( USE_LIGHT_PROBES )
	uniform vec3 lightProbe[ 9 ];
#endif
vec3 shGetIrradianceAt( in vec3 normal, in vec3 shCoefficients[ 9 ] ) {
	float x = normal.x, y = normal.y, z = normal.z;
	vec3 result = shCoefficients[ 0 ] * 0.886227;
	result += shCoefficients[ 1 ] * 2.0 * 0.511664 * y;
	result += shCoefficients[ 2 ] * 2.0 * 0.511664 * z;
	result += shCoefficients[ 3 ] * 2.0 * 0.511664 * x;
	result += shCoefficients[ 4 ] * 2.0 * 0.429043 * x * y;
	result += shCoefficients[ 5 ] * 2.0 * 0.429043 * y * z;
	result += shCoefficients[ 6 ] * ( 0.743125 * z * z - 0.247708 );
	result += shCoefficients[ 7 ] * 2.0 * 0.429043 * x * z;
	result += shCoefficients[ 8 ] * 0.429043 * ( x * x - y * y );
	return result;
}
vec3 getLightProbeIrradiance( const in vec3 lightProbe[ 9 ], const in vec3 normal ) {
	vec3 worldNormal = inverseTransformDirection( normal, viewMatrix );
	vec3 irradiance = shGetIrradianceAt( worldNormal, lightProbe );
	return irradiance;
}
vec3 getAmbientLightIrradiance( const in vec3 ambientLightColor ) {
	vec3 irradiance = ambientLightColor;
	return irradiance;
}
float getDistanceAttenuation( const in float lightDistance, const in float cutoffDistance, const in float decayExponent ) {
	float distanceFalloff = 1.0 / max( pow( lightDistance, decayExponent ), 0.01 );
	if ( cutoffDistance > 0.0 ) {
		distanceFalloff *= pow2( saturate( 1.0 - pow4( lightDistance / cutoffDistance ) ) );
	}
	return distanceFalloff;
}
float getSpotAttenuation( const in float coneCosine, const in float penumbraCosine, const in float angleCosine ) {
	return smoothstep( coneCosine, penumbraCosine, angleCosine );
}
#if NUM_DIR_LIGHTS > 0
	struct DirectionalLight {
		vec3 direction;
		vec3 color;
	};
	uniform DirectionalLight directionalLights[ NUM_DIR_LIGHTS ];
	void getDirectionalLightInfo( const in DirectionalLight directionalLight, out IncidentLight light ) {
		light.color = directionalLight.color;
		light.direction = directionalLight.direction;
		light.visible = true;
	}
#endif
#if NUM_POINT_LIGHTS > 0
	struct PointLight {
		vec3 position;
		vec3 color;
		float distance;
		float decay;
	};
	uniform PointLight pointLights[ NUM_POINT_LIGHTS ];
	void getPointLightInfo( const in PointLight pointLight, const in vec3 geometryPosition, out IncidentLight light ) {
		vec3 lVector = pointLight.position - geometryPosition;
		light.direction = normalize( lVector );
		float lightDistance = length( lVector );
		light.color = pointLight.color;
		light.color *= getDistanceAttenuation( lightDistance, pointLight.distance, pointLight.decay );
		light.visible = ( light.color != vec3( 0.0 ) );
	}
#endif
#if NUM_SPOT_LIGHTS > 0
	struct SpotLight {
		vec3 position;
		vec3 direction;
		vec3 color;
		float distance;
		float decay;
		float coneCos;
		float penumbraCos;
	};
	uniform SpotLight spotLights[ NUM_SPOT_LIGHTS ];
	void getSpotLightInfo( const in SpotLight spotLight, const in vec3 geometryPosition, out IncidentLight light ) {
		vec3 lVector = spotLight.position - geometryPosition;
		light.direction = normalize( lVector );
		float angleCos = dot( light.direction, spotLight.direction );
		float spotAttenuation = getSpotAttenuation( spotLight.coneCos, spotLight.penumbraCos, angleCos );
		if ( spotAttenuation > 0.0 ) {
			float lightDistance = length( lVector );
			light.color = spotLight.color * spotAttenuation;
			light.color *= getDistanceAttenuation( lightDistance, spotLight.distance, spotLight.decay );
			light.visible = ( light.color != vec3( 0.0 ) );
		} else {
			light.color = vec3( 0.0 );
			light.visible = false;
		}
	}
#endif
#if NUM_RECT_AREA_LIGHTS > 0
	struct RectAreaLight {
		vec3 color;
		vec3 position;
		vec3 halfWidth;
		vec3 halfHeight;
	};
	uniform sampler2D ltc_1;	uniform sampler2D ltc_2;
	uniform RectAreaLight rectAreaLights[ NUM_RECT_AREA_LIGHTS ];
#endif
#if NUM_HEMI_LIGHTS > 0
	struct HemisphereLight {
		vec3 direction;
		vec3 skyColor;
		vec3 groundColor;
	};
	uniform HemisphereLight hemisphereLights[ NUM_HEMI_LIGHTS ];
	vec3 getHemisphereLightIrradiance( const in HemisphereLight hemiLight, const in vec3 normal ) {
		float dotNL = dot( normal, hemiLight.direction );
		float hemiDiffuseWeight = 0.5 * dotNL + 0.5;
		vec3 irradiance = mix( hemiLight.groundColor, hemiLight.skyColor, hemiDiffuseWeight );
		return irradiance;
	}
#endif`,I_=`#ifdef USE_ENVMAP
	vec3 getIBLIrradiance( const in vec3 normal ) {
		#ifdef ENVMAP_TYPE_CUBE_UV
			vec3 worldNormal = inverseTransformDirection( normal, viewMatrix );
			vec4 envMapColor = textureCubeUV( envMap, envMapRotation * worldNormal, 1.0 );
			return PI * envMapColor.rgb * envMapIntensity;
		#else
			return vec3( 0.0 );
		#endif
	}
	vec3 getIBLRadiance( const in vec3 viewDir, const in vec3 normal, const in float roughness ) {
		#ifdef ENVMAP_TYPE_CUBE_UV
			vec3 reflectVec = reflect( - viewDir, normal );
			reflectVec = normalize( mix( reflectVec, normal, roughness * roughness) );
			reflectVec = inverseTransformDirection( reflectVec, viewMatrix );
			vec4 envMapColor = textureCubeUV( envMap, envMapRotation * reflectVec, roughness );
			return envMapColor.rgb * envMapIntensity;
		#else
			return vec3( 0.0 );
		#endif
	}
	#ifdef USE_ANISOTROPY
		vec3 getIBLAnisotropyRadiance( const in vec3 viewDir, const in vec3 normal, const in float roughness, const in vec3 bitangent, const in float anisotropy ) {
			#ifdef ENVMAP_TYPE_CUBE_UV
				vec3 bentNormal = cross( bitangent, viewDir );
				bentNormal = normalize( cross( bentNormal, bitangent ) );
				bentNormal = normalize( mix( bentNormal, normal, pow2( pow2( 1.0 - anisotropy * ( 1.0 - roughness ) ) ) ) );
				return getIBLRadiance( viewDir, bentNormal, roughness );
			#else
				return vec3( 0.0 );
			#endif
		}
	#endif
#endif`,O_=`ToonMaterial material;
material.diffuseColor = diffuseColor.rgb;`,U_=`varying vec3 vViewPosition;
struct ToonMaterial {
	vec3 diffuseColor;
};
void RE_Direct_Toon( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in ToonMaterial material, inout ReflectedLight reflectedLight ) {
	vec3 irradiance = getGradientIrradiance( geometryNormal, directLight.direction ) * directLight.color;
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectDiffuse_Toon( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in ToonMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
#define RE_Direct				RE_Direct_Toon
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Toon`,N_=`BlinnPhongMaterial material;
material.diffuseColor = diffuseColor.rgb;
material.specularColor = specular;
material.specularShininess = shininess;
material.specularStrength = specularStrength;`,F_=`varying vec3 vViewPosition;
struct BlinnPhongMaterial {
	vec3 diffuseColor;
	vec3 specularColor;
	float specularShininess;
	float specularStrength;
};
void RE_Direct_BlinnPhong( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in BlinnPhongMaterial material, inout ReflectedLight reflectedLight ) {
	float dotNL = saturate( dot( geometryNormal, directLight.direction ) );
	vec3 irradiance = dotNL * directLight.color;
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
	reflectedLight.directSpecular += irradiance * BRDF_BlinnPhong( directLight.direction, geometryViewDir, geometryNormal, material.specularColor, material.specularShininess ) * material.specularStrength;
}
void RE_IndirectDiffuse_BlinnPhong( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in BlinnPhongMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
#define RE_Direct				RE_Direct_BlinnPhong
#define RE_IndirectDiffuse		RE_IndirectDiffuse_BlinnPhong`,k_=`PhysicalMaterial material;
material.diffuseColor = diffuseColor.rgb * ( 1.0 - metalnessFactor );
vec3 dxy = max( abs( dFdx( nonPerturbedNormal ) ), abs( dFdy( nonPerturbedNormal ) ) );
float geometryRoughness = max( max( dxy.x, dxy.y ), dxy.z );
material.roughness = max( roughnessFactor, 0.0525 );material.roughness += geometryRoughness;
material.roughness = min( material.roughness, 1.0 );
#ifdef IOR
	material.ior = ior;
	#ifdef USE_SPECULAR
		float specularIntensityFactor = specularIntensity;
		vec3 specularColorFactor = specularColor;
		#ifdef USE_SPECULAR_COLORMAP
			specularColorFactor *= texture2D( specularColorMap, vSpecularColorMapUv ).rgb;
		#endif
		#ifdef USE_SPECULAR_INTENSITYMAP
			specularIntensityFactor *= texture2D( specularIntensityMap, vSpecularIntensityMapUv ).a;
		#endif
		material.specularF90 = mix( specularIntensityFactor, 1.0, metalnessFactor );
	#else
		float specularIntensityFactor = 1.0;
		vec3 specularColorFactor = vec3( 1.0 );
		material.specularF90 = 1.0;
	#endif
	material.specularColor = mix( min( pow2( ( material.ior - 1.0 ) / ( material.ior + 1.0 ) ) * specularColorFactor, vec3( 1.0 ) ) * specularIntensityFactor, diffuseColor.rgb, metalnessFactor );
#else
	material.specularColor = mix( vec3( 0.04 ), diffuseColor.rgb, metalnessFactor );
	material.specularF90 = 1.0;
#endif
#ifdef USE_CLEARCOAT
	material.clearcoat = clearcoat;
	material.clearcoatRoughness = clearcoatRoughness;
	material.clearcoatF0 = vec3( 0.04 );
	material.clearcoatF90 = 1.0;
	#ifdef USE_CLEARCOATMAP
		material.clearcoat *= texture2D( clearcoatMap, vClearcoatMapUv ).x;
	#endif
	#ifdef USE_CLEARCOAT_ROUGHNESSMAP
		material.clearcoatRoughness *= texture2D( clearcoatRoughnessMap, vClearcoatRoughnessMapUv ).y;
	#endif
	material.clearcoat = saturate( material.clearcoat );	material.clearcoatRoughness = max( material.clearcoatRoughness, 0.0525 );
	material.clearcoatRoughness += geometryRoughness;
	material.clearcoatRoughness = min( material.clearcoatRoughness, 1.0 );
#endif
#ifdef USE_DISPERSION
	material.dispersion = dispersion;
#endif
#ifdef USE_IRIDESCENCE
	material.iridescence = iridescence;
	material.iridescenceIOR = iridescenceIOR;
	#ifdef USE_IRIDESCENCEMAP
		material.iridescence *= texture2D( iridescenceMap, vIridescenceMapUv ).r;
	#endif
	#ifdef USE_IRIDESCENCE_THICKNESSMAP
		material.iridescenceThickness = (iridescenceThicknessMaximum - iridescenceThicknessMinimum) * texture2D( iridescenceThicknessMap, vIridescenceThicknessMapUv ).g + iridescenceThicknessMinimum;
	#else
		material.iridescenceThickness = iridescenceThicknessMaximum;
	#endif
#endif
#ifdef USE_SHEEN
	material.sheenColor = sheenColor;
	#ifdef USE_SHEEN_COLORMAP
		material.sheenColor *= texture2D( sheenColorMap, vSheenColorMapUv ).rgb;
	#endif
	material.sheenRoughness = clamp( sheenRoughness, 0.07, 1.0 );
	#ifdef USE_SHEEN_ROUGHNESSMAP
		material.sheenRoughness *= texture2D( sheenRoughnessMap, vSheenRoughnessMapUv ).a;
	#endif
#endif
#ifdef USE_ANISOTROPY
	#ifdef USE_ANISOTROPYMAP
		mat2 anisotropyMat = mat2( anisotropyVector.x, anisotropyVector.y, - anisotropyVector.y, anisotropyVector.x );
		vec3 anisotropyPolar = texture2D( anisotropyMap, vAnisotropyMapUv ).rgb;
		vec2 anisotropyV = anisotropyMat * normalize( 2.0 * anisotropyPolar.rg - vec2( 1.0 ) ) * anisotropyPolar.b;
	#else
		vec2 anisotropyV = anisotropyVector;
	#endif
	material.anisotropy = length( anisotropyV );
	if( material.anisotropy == 0.0 ) {
		anisotropyV = vec2( 1.0, 0.0 );
	} else {
		anisotropyV /= material.anisotropy;
		material.anisotropy = saturate( material.anisotropy );
	}
	material.alphaT = mix( pow2( material.roughness ), 1.0, pow2( material.anisotropy ) );
	material.anisotropyT = tbn[ 0 ] * anisotropyV.x + tbn[ 1 ] * anisotropyV.y;
	material.anisotropyB = tbn[ 1 ] * anisotropyV.x - tbn[ 0 ] * anisotropyV.y;
#endif`,B_=`struct PhysicalMaterial {
	vec3 diffuseColor;
	float roughness;
	vec3 specularColor;
	float specularF90;
	float dispersion;
	#ifdef USE_CLEARCOAT
		float clearcoat;
		float clearcoatRoughness;
		vec3 clearcoatF0;
		float clearcoatF90;
	#endif
	#ifdef USE_IRIDESCENCE
		float iridescence;
		float iridescenceIOR;
		float iridescenceThickness;
		vec3 iridescenceFresnel;
		vec3 iridescenceF0;
	#endif
	#ifdef USE_SHEEN
		vec3 sheenColor;
		float sheenRoughness;
	#endif
	#ifdef IOR
		float ior;
	#endif
	#ifdef USE_TRANSMISSION
		float transmission;
		float transmissionAlpha;
		float thickness;
		float attenuationDistance;
		vec3 attenuationColor;
	#endif
	#ifdef USE_ANISOTROPY
		float anisotropy;
		float alphaT;
		vec3 anisotropyT;
		vec3 anisotropyB;
	#endif
};
vec3 clearcoatSpecularDirect = vec3( 0.0 );
vec3 clearcoatSpecularIndirect = vec3( 0.0 );
vec3 sheenSpecularDirect = vec3( 0.0 );
vec3 sheenSpecularIndirect = vec3(0.0 );
vec3 Schlick_to_F0( const in vec3 f, const in float f90, const in float dotVH ) {
    float x = clamp( 1.0 - dotVH, 0.0, 1.0 );
    float x2 = x * x;
    float x5 = clamp( x * x2 * x2, 0.0, 0.9999 );
    return ( f - vec3( f90 ) * x5 ) / ( 1.0 - x5 );
}
float V_GGX_SmithCorrelated( const in float alpha, const in float dotNL, const in float dotNV ) {
	float a2 = pow2( alpha );
	float gv = dotNL * sqrt( a2 + ( 1.0 - a2 ) * pow2( dotNV ) );
	float gl = dotNV * sqrt( a2 + ( 1.0 - a2 ) * pow2( dotNL ) );
	return 0.5 / max( gv + gl, EPSILON );
}
float D_GGX( const in float alpha, const in float dotNH ) {
	float a2 = pow2( alpha );
	float denom = pow2( dotNH ) * ( a2 - 1.0 ) + 1.0;
	return RECIPROCAL_PI * a2 / pow2( denom );
}
#ifdef USE_ANISOTROPY
	float V_GGX_SmithCorrelated_Anisotropic( const in float alphaT, const in float alphaB, const in float dotTV, const in float dotBV, const in float dotTL, const in float dotBL, const in float dotNV, const in float dotNL ) {
		float gv = dotNL * length( vec3( alphaT * dotTV, alphaB * dotBV, dotNV ) );
		float gl = dotNV * length( vec3( alphaT * dotTL, alphaB * dotBL, dotNL ) );
		float v = 0.5 / ( gv + gl );
		return saturate(v);
	}
	float D_GGX_Anisotropic( const in float alphaT, const in float alphaB, const in float dotNH, const in float dotTH, const in float dotBH ) {
		float a2 = alphaT * alphaB;
		highp vec3 v = vec3( alphaB * dotTH, alphaT * dotBH, a2 * dotNH );
		highp float v2 = dot( v, v );
		float w2 = a2 / v2;
		return RECIPROCAL_PI * a2 * pow2 ( w2 );
	}
#endif
#ifdef USE_CLEARCOAT
	vec3 BRDF_GGX_Clearcoat( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, const in PhysicalMaterial material) {
		vec3 f0 = material.clearcoatF0;
		float f90 = material.clearcoatF90;
		float roughness = material.clearcoatRoughness;
		float alpha = pow2( roughness );
		vec3 halfDir = normalize( lightDir + viewDir );
		float dotNL = saturate( dot( normal, lightDir ) );
		float dotNV = saturate( dot( normal, viewDir ) );
		float dotNH = saturate( dot( normal, halfDir ) );
		float dotVH = saturate( dot( viewDir, halfDir ) );
		vec3 F = F_Schlick( f0, f90, dotVH );
		float V = V_GGX_SmithCorrelated( alpha, dotNL, dotNV );
		float D = D_GGX( alpha, dotNH );
		return F * ( V * D );
	}
#endif
vec3 BRDF_GGX( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, const in PhysicalMaterial material ) {
	vec3 f0 = material.specularColor;
	float f90 = material.specularF90;
	float roughness = material.roughness;
	float alpha = pow2( roughness );
	vec3 halfDir = normalize( lightDir + viewDir );
	float dotNL = saturate( dot( normal, lightDir ) );
	float dotNV = saturate( dot( normal, viewDir ) );
	float dotNH = saturate( dot( normal, halfDir ) );
	float dotVH = saturate( dot( viewDir, halfDir ) );
	vec3 F = F_Schlick( f0, f90, dotVH );
	#ifdef USE_IRIDESCENCE
		F = mix( F, material.iridescenceFresnel, material.iridescence );
	#endif
	#ifdef USE_ANISOTROPY
		float dotTL = dot( material.anisotropyT, lightDir );
		float dotTV = dot( material.anisotropyT, viewDir );
		float dotTH = dot( material.anisotropyT, halfDir );
		float dotBL = dot( material.anisotropyB, lightDir );
		float dotBV = dot( material.anisotropyB, viewDir );
		float dotBH = dot( material.anisotropyB, halfDir );
		float V = V_GGX_SmithCorrelated_Anisotropic( material.alphaT, alpha, dotTV, dotBV, dotTL, dotBL, dotNV, dotNL );
		float D = D_GGX_Anisotropic( material.alphaT, alpha, dotNH, dotTH, dotBH );
	#else
		float V = V_GGX_SmithCorrelated( alpha, dotNL, dotNV );
		float D = D_GGX( alpha, dotNH );
	#endif
	return F * ( V * D );
}
vec2 LTC_Uv( const in vec3 N, const in vec3 V, const in float roughness ) {
	const float LUT_SIZE = 64.0;
	const float LUT_SCALE = ( LUT_SIZE - 1.0 ) / LUT_SIZE;
	const float LUT_BIAS = 0.5 / LUT_SIZE;
	float dotNV = saturate( dot( N, V ) );
	vec2 uv = vec2( roughness, sqrt( 1.0 - dotNV ) );
	uv = uv * LUT_SCALE + LUT_BIAS;
	return uv;
}
float LTC_ClippedSphereFormFactor( const in vec3 f ) {
	float l = length( f );
	return max( ( l * l + f.z ) / ( l + 1.0 ), 0.0 );
}
vec3 LTC_EdgeVectorFormFactor( const in vec3 v1, const in vec3 v2 ) {
	float x = dot( v1, v2 );
	float y = abs( x );
	float a = 0.8543985 + ( 0.4965155 + 0.0145206 * y ) * y;
	float b = 3.4175940 + ( 4.1616724 + y ) * y;
	float v = a / b;
	float theta_sintheta = ( x > 0.0 ) ? v : 0.5 * inversesqrt( max( 1.0 - x * x, 1e-7 ) ) - v;
	return cross( v1, v2 ) * theta_sintheta;
}
vec3 LTC_Evaluate( const in vec3 N, const in vec3 V, const in vec3 P, const in mat3 mInv, const in vec3 rectCoords[ 4 ] ) {
	vec3 v1 = rectCoords[ 1 ] - rectCoords[ 0 ];
	vec3 v2 = rectCoords[ 3 ] - rectCoords[ 0 ];
	vec3 lightNormal = cross( v1, v2 );
	if( dot( lightNormal, P - rectCoords[ 0 ] ) < 0.0 ) return vec3( 0.0 );
	vec3 T1, T2;
	T1 = normalize( V - N * dot( V, N ) );
	T2 = - cross( N, T1 );
	mat3 mat = mInv * transposeMat3( mat3( T1, T2, N ) );
	vec3 coords[ 4 ];
	coords[ 0 ] = mat * ( rectCoords[ 0 ] - P );
	coords[ 1 ] = mat * ( rectCoords[ 1 ] - P );
	coords[ 2 ] = mat * ( rectCoords[ 2 ] - P );
	coords[ 3 ] = mat * ( rectCoords[ 3 ] - P );
	coords[ 0 ] = normalize( coords[ 0 ] );
	coords[ 1 ] = normalize( coords[ 1 ] );
	coords[ 2 ] = normalize( coords[ 2 ] );
	coords[ 3 ] = normalize( coords[ 3 ] );
	vec3 vectorFormFactor = vec3( 0.0 );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 0 ], coords[ 1 ] );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 1 ], coords[ 2 ] );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 2 ], coords[ 3 ] );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 3 ], coords[ 0 ] );
	float result = LTC_ClippedSphereFormFactor( vectorFormFactor );
	return vec3( result );
}
#if defined( USE_SHEEN )
float D_Charlie( float roughness, float dotNH ) {
	float alpha = pow2( roughness );
	float invAlpha = 1.0 / alpha;
	float cos2h = dotNH * dotNH;
	float sin2h = max( 1.0 - cos2h, 0.0078125 );
	return ( 2.0 + invAlpha ) * pow( sin2h, invAlpha * 0.5 ) / ( 2.0 * PI );
}
float V_Neubelt( float dotNV, float dotNL ) {
	return saturate( 1.0 / ( 4.0 * ( dotNL + dotNV - dotNL * dotNV ) ) );
}
vec3 BRDF_Sheen( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, vec3 sheenColor, const in float sheenRoughness ) {
	vec3 halfDir = normalize( lightDir + viewDir );
	float dotNL = saturate( dot( normal, lightDir ) );
	float dotNV = saturate( dot( normal, viewDir ) );
	float dotNH = saturate( dot( normal, halfDir ) );
	float D = D_Charlie( sheenRoughness, dotNH );
	float V = V_Neubelt( dotNV, dotNL );
	return sheenColor * ( D * V );
}
#endif
float IBLSheenBRDF( const in vec3 normal, const in vec3 viewDir, const in float roughness ) {
	float dotNV = saturate( dot( normal, viewDir ) );
	float r2 = roughness * roughness;
	float a = roughness < 0.25 ? -339.2 * r2 + 161.4 * roughness - 25.9 : -8.48 * r2 + 14.3 * roughness - 9.95;
	float b = roughness < 0.25 ? 44.0 * r2 - 23.7 * roughness + 3.26 : 1.97 * r2 - 3.27 * roughness + 0.72;
	float DG = exp( a * dotNV + b ) + ( roughness < 0.25 ? 0.0 : 0.1 * ( roughness - 0.25 ) );
	return saturate( DG * RECIPROCAL_PI );
}
vec2 DFGApprox( const in vec3 normal, const in vec3 viewDir, const in float roughness ) {
	float dotNV = saturate( dot( normal, viewDir ) );
	const vec4 c0 = vec4( - 1, - 0.0275, - 0.572, 0.022 );
	const vec4 c1 = vec4( 1, 0.0425, 1.04, - 0.04 );
	vec4 r = roughness * c0 + c1;
	float a004 = min( r.x * r.x, exp2( - 9.28 * dotNV ) ) * r.x + r.y;
	vec2 fab = vec2( - 1.04, 1.04 ) * a004 + r.zw;
	return fab;
}
vec3 EnvironmentBRDF( const in vec3 normal, const in vec3 viewDir, const in vec3 specularColor, const in float specularF90, const in float roughness ) {
	vec2 fab = DFGApprox( normal, viewDir, roughness );
	return specularColor * fab.x + specularF90 * fab.y;
}
#ifdef USE_IRIDESCENCE
void computeMultiscatteringIridescence( const in vec3 normal, const in vec3 viewDir, const in vec3 specularColor, const in float specularF90, const in float iridescence, const in vec3 iridescenceF0, const in float roughness, inout vec3 singleScatter, inout vec3 multiScatter ) {
#else
void computeMultiscattering( const in vec3 normal, const in vec3 viewDir, const in vec3 specularColor, const in float specularF90, const in float roughness, inout vec3 singleScatter, inout vec3 multiScatter ) {
#endif
	vec2 fab = DFGApprox( normal, viewDir, roughness );
	#ifdef USE_IRIDESCENCE
		vec3 Fr = mix( specularColor, iridescenceF0, iridescence );
	#else
		vec3 Fr = specularColor;
	#endif
	vec3 FssEss = Fr * fab.x + specularF90 * fab.y;
	float Ess = fab.x + fab.y;
	float Ems = 1.0 - Ess;
	vec3 Favg = Fr + ( 1.0 - Fr ) * 0.047619;	vec3 Fms = FssEss * Favg / ( 1.0 - Ems * Favg );
	singleScatter += FssEss;
	multiScatter += Fms * Ems;
}
#if NUM_RECT_AREA_LIGHTS > 0
	void RE_Direct_RectArea_Physical( const in RectAreaLight rectAreaLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight ) {
		vec3 normal = geometryNormal;
		vec3 viewDir = geometryViewDir;
		vec3 position = geometryPosition;
		vec3 lightPos = rectAreaLight.position;
		vec3 halfWidth = rectAreaLight.halfWidth;
		vec3 halfHeight = rectAreaLight.halfHeight;
		vec3 lightColor = rectAreaLight.color;
		float roughness = material.roughness;
		vec3 rectCoords[ 4 ];
		rectCoords[ 0 ] = lightPos + halfWidth - halfHeight;		rectCoords[ 1 ] = lightPos - halfWidth - halfHeight;
		rectCoords[ 2 ] = lightPos - halfWidth + halfHeight;
		rectCoords[ 3 ] = lightPos + halfWidth + halfHeight;
		vec2 uv = LTC_Uv( normal, viewDir, roughness );
		vec4 t1 = texture2D( ltc_1, uv );
		vec4 t2 = texture2D( ltc_2, uv );
		mat3 mInv = mat3(
			vec3( t1.x, 0, t1.y ),
			vec3(    0, 1,    0 ),
			vec3( t1.z, 0, t1.w )
		);
		vec3 fresnel = ( material.specularColor * t2.x + ( vec3( 1.0 ) - material.specularColor ) * t2.y );
		reflectedLight.directSpecular += lightColor * fresnel * LTC_Evaluate( normal, viewDir, position, mInv, rectCoords );
		reflectedLight.directDiffuse += lightColor * material.diffuseColor * LTC_Evaluate( normal, viewDir, position, mat3( 1.0 ), rectCoords );
	}
#endif
void RE_Direct_Physical( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight ) {
	float dotNL = saturate( dot( geometryNormal, directLight.direction ) );
	vec3 irradiance = dotNL * directLight.color;
	#ifdef USE_CLEARCOAT
		float dotNLcc = saturate( dot( geometryClearcoatNormal, directLight.direction ) );
		vec3 ccIrradiance = dotNLcc * directLight.color;
		clearcoatSpecularDirect += ccIrradiance * BRDF_GGX_Clearcoat( directLight.direction, geometryViewDir, geometryClearcoatNormal, material );
	#endif
	#ifdef USE_SHEEN
		sheenSpecularDirect += irradiance * BRDF_Sheen( directLight.direction, geometryViewDir, geometryNormal, material.sheenColor, material.sheenRoughness );
	#endif
	reflectedLight.directSpecular += irradiance * BRDF_GGX( directLight.direction, geometryViewDir, geometryNormal, material );
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectDiffuse_Physical( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectSpecular_Physical( const in vec3 radiance, const in vec3 irradiance, const in vec3 clearcoatRadiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight) {
	#ifdef USE_CLEARCOAT
		clearcoatSpecularIndirect += clearcoatRadiance * EnvironmentBRDF( geometryClearcoatNormal, geometryViewDir, material.clearcoatF0, material.clearcoatF90, material.clearcoatRoughness );
	#endif
	#ifdef USE_SHEEN
		sheenSpecularIndirect += irradiance * material.sheenColor * IBLSheenBRDF( geometryNormal, geometryViewDir, material.sheenRoughness );
	#endif
	vec3 singleScattering = vec3( 0.0 );
	vec3 multiScattering = vec3( 0.0 );
	vec3 cosineWeightedIrradiance = irradiance * RECIPROCAL_PI;
	#ifdef USE_IRIDESCENCE
		computeMultiscatteringIridescence( geometryNormal, geometryViewDir, material.specularColor, material.specularF90, material.iridescence, material.iridescenceFresnel, material.roughness, singleScattering, multiScattering );
	#else
		computeMultiscattering( geometryNormal, geometryViewDir, material.specularColor, material.specularF90, material.roughness, singleScattering, multiScattering );
	#endif
	vec3 totalScattering = singleScattering + multiScattering;
	vec3 diffuse = material.diffuseColor * ( 1.0 - max( max( totalScattering.r, totalScattering.g ), totalScattering.b ) );
	reflectedLight.indirectSpecular += radiance * singleScattering;
	reflectedLight.indirectSpecular += multiScattering * cosineWeightedIrradiance;
	reflectedLight.indirectDiffuse += diffuse * cosineWeightedIrradiance;
}
#define RE_Direct				RE_Direct_Physical
#define RE_Direct_RectArea		RE_Direct_RectArea_Physical
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Physical
#define RE_IndirectSpecular		RE_IndirectSpecular_Physical
float computeSpecularOcclusion( const in float dotNV, const in float ambientOcclusion, const in float roughness ) {
	return saturate( pow( dotNV + ambientOcclusion, exp2( - 16.0 * roughness - 1.0 ) ) - 1.0 + ambientOcclusion );
}`,z_=`
vec3 geometryPosition = - vViewPosition;
vec3 geometryNormal = normal;
vec3 geometryViewDir = ( isOrthographic ) ? vec3( 0, 0, 1 ) : normalize( vViewPosition );
vec3 geometryClearcoatNormal = vec3( 0.0 );
#ifdef USE_CLEARCOAT
	geometryClearcoatNormal = clearcoatNormal;
#endif
#ifdef USE_IRIDESCENCE
	float dotNVi = saturate( dot( normal, geometryViewDir ) );
	if ( material.iridescenceThickness == 0.0 ) {
		material.iridescence = 0.0;
	} else {
		material.iridescence = saturate( material.iridescence );
	}
	if ( material.iridescence > 0.0 ) {
		material.iridescenceFresnel = evalIridescence( 1.0, material.iridescenceIOR, dotNVi, material.iridescenceThickness, material.specularColor );
		material.iridescenceF0 = Schlick_to_F0( material.iridescenceFresnel, 1.0, dotNVi );
	}
#endif
IncidentLight directLight;
#if ( NUM_POINT_LIGHTS > 0 ) && defined( RE_Direct )
	PointLight pointLight;
	#if defined( USE_SHADOWMAP ) && NUM_POINT_LIGHT_SHADOWS > 0
	PointLightShadow pointLightShadow;
	#endif
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_POINT_LIGHTS; i ++ ) {
		pointLight = pointLights[ i ];
		getPointLightInfo( pointLight, geometryPosition, directLight );
		#if defined( USE_SHADOWMAP ) && ( UNROLLED_LOOP_INDEX < NUM_POINT_LIGHT_SHADOWS )
		pointLightShadow = pointLightShadows[ i ];
		directLight.color *= ( directLight.visible && receiveShadow ) ? getPointShadow( pointShadowMap[ i ], pointLightShadow.shadowMapSize, pointLightShadow.shadowIntensity, pointLightShadow.shadowBias, pointLightShadow.shadowRadius, vPointShadowCoord[ i ], pointLightShadow.shadowCameraNear, pointLightShadow.shadowCameraFar ) : 1.0;
		#endif
		RE_Direct( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if ( NUM_SPOT_LIGHTS > 0 ) && defined( RE_Direct )
	SpotLight spotLight;
	vec4 spotColor;
	vec3 spotLightCoord;
	bool inSpotLightMap;
	#if defined( USE_SHADOWMAP ) && NUM_SPOT_LIGHT_SHADOWS > 0
	SpotLightShadow spotLightShadow;
	#endif
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_SPOT_LIGHTS; i ++ ) {
		spotLight = spotLights[ i ];
		getSpotLightInfo( spotLight, geometryPosition, directLight );
		#if ( UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS )
		#define SPOT_LIGHT_MAP_INDEX UNROLLED_LOOP_INDEX
		#elif ( UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS )
		#define SPOT_LIGHT_MAP_INDEX NUM_SPOT_LIGHT_MAPS
		#else
		#define SPOT_LIGHT_MAP_INDEX ( UNROLLED_LOOP_INDEX - NUM_SPOT_LIGHT_SHADOWS + NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS )
		#endif
		#if ( SPOT_LIGHT_MAP_INDEX < NUM_SPOT_LIGHT_MAPS )
			spotLightCoord = vSpotLightCoord[ i ].xyz / vSpotLightCoord[ i ].w;
			inSpotLightMap = all( lessThan( abs( spotLightCoord * 2. - 1. ), vec3( 1.0 ) ) );
			spotColor = texture2D( spotLightMap[ SPOT_LIGHT_MAP_INDEX ], spotLightCoord.xy );
			directLight.color = inSpotLightMap ? directLight.color * spotColor.rgb : directLight.color;
		#endif
		#undef SPOT_LIGHT_MAP_INDEX
		#if defined( USE_SHADOWMAP ) && ( UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS )
		spotLightShadow = spotLightShadows[ i ];
		directLight.color *= ( directLight.visible && receiveShadow ) ? getShadow( spotShadowMap[ i ], spotLightShadow.shadowMapSize, spotLightShadow.shadowIntensity, spotLightShadow.shadowBias, spotLightShadow.shadowRadius, vSpotLightCoord[ i ] ) : 1.0;
		#endif
		RE_Direct( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if ( NUM_DIR_LIGHTS > 0 ) && defined( RE_Direct )
	DirectionalLight directionalLight;
	#if defined( USE_SHADOWMAP ) && NUM_DIR_LIGHT_SHADOWS > 0
	DirectionalLightShadow directionalLightShadow;
	#endif
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_DIR_LIGHTS; i ++ ) {
		directionalLight = directionalLights[ i ];
		getDirectionalLightInfo( directionalLight, directLight );
		#if defined( USE_SHADOWMAP ) && ( UNROLLED_LOOP_INDEX < NUM_DIR_LIGHT_SHADOWS )
		directionalLightShadow = directionalLightShadows[ i ];
		directLight.color *= ( directLight.visible && receiveShadow ) ? getShadow( directionalShadowMap[ i ], directionalLightShadow.shadowMapSize, directionalLightShadow.shadowIntensity, directionalLightShadow.shadowBias, directionalLightShadow.shadowRadius, vDirectionalShadowCoord[ i ] ) : 1.0;
		#endif
		RE_Direct( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if ( NUM_RECT_AREA_LIGHTS > 0 ) && defined( RE_Direct_RectArea )
	RectAreaLight rectAreaLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_RECT_AREA_LIGHTS; i ++ ) {
		rectAreaLight = rectAreaLights[ i ];
		RE_Direct_RectArea( rectAreaLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if defined( RE_IndirectDiffuse )
	vec3 iblIrradiance = vec3( 0.0 );
	vec3 irradiance = getAmbientLightIrradiance( ambientLightColor );
	#if defined( USE_LIGHT_PROBES )
		irradiance += getLightProbeIrradiance( lightProbe, geometryNormal );
	#endif
	#if ( NUM_HEMI_LIGHTS > 0 )
		#pragma unroll_loop_start
		for ( int i = 0; i < NUM_HEMI_LIGHTS; i ++ ) {
			irradiance += getHemisphereLightIrradiance( hemisphereLights[ i ], geometryNormal );
		}
		#pragma unroll_loop_end
	#endif
#endif
#if defined( RE_IndirectSpecular )
	vec3 radiance = vec3( 0.0 );
	vec3 clearcoatRadiance = vec3( 0.0 );
#endif`,H_=`#if defined( RE_IndirectDiffuse )
	#ifdef USE_LIGHTMAP
		vec4 lightMapTexel = texture2D( lightMap, vLightMapUv );
		vec3 lightMapIrradiance = lightMapTexel.rgb * lightMapIntensity;
		irradiance += lightMapIrradiance;
	#endif
	#if defined( USE_ENVMAP ) && defined( STANDARD ) && defined( ENVMAP_TYPE_CUBE_UV )
		iblIrradiance += getIBLIrradiance( geometryNormal );
	#endif
#endif
#if defined( USE_ENVMAP ) && defined( RE_IndirectSpecular )
	#ifdef USE_ANISOTROPY
		radiance += getIBLAnisotropyRadiance( geometryViewDir, geometryNormal, material.roughness, material.anisotropyB, material.anisotropy );
	#else
		radiance += getIBLRadiance( geometryViewDir, geometryNormal, material.roughness );
	#endif
	#ifdef USE_CLEARCOAT
		clearcoatRadiance += getIBLRadiance( geometryViewDir, geometryClearcoatNormal, material.clearcoatRoughness );
	#endif
#endif`,V_=`#if defined( RE_IndirectDiffuse )
	RE_IndirectDiffuse( irradiance, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
#endif
#if defined( RE_IndirectSpecular )
	RE_IndirectSpecular( radiance, iblIrradiance, clearcoatRadiance, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
#endif`,G_=`#if defined( USE_LOGDEPTHBUF )
	gl_FragDepth = vIsPerspective == 0.0 ? gl_FragCoord.z : log2( vFragDepth ) * logDepthBufFC * 0.5;
#endif`,W_=`#if defined( USE_LOGDEPTHBUF )
	uniform float logDepthBufFC;
	varying float vFragDepth;
	varying float vIsPerspective;
#endif`,X_=`#ifdef USE_LOGDEPTHBUF
	varying float vFragDepth;
	varying float vIsPerspective;
#endif`,j_=`#ifdef USE_LOGDEPTHBUF
	vFragDepth = 1.0 + gl_Position.w;
	vIsPerspective = float( isPerspectiveMatrix( projectionMatrix ) );
#endif`,$_=`#ifdef USE_MAP
	vec4 sampledDiffuseColor = texture2D( map, vMapUv );
	#ifdef DECODE_VIDEO_TEXTURE
		sampledDiffuseColor = sRGBTransferEOTF( sampledDiffuseColor );
	#endif
	diffuseColor *= sampledDiffuseColor;
#endif`,Y_=`#ifdef USE_MAP
	uniform sampler2D map;
#endif`,q_=`#if defined( USE_MAP ) || defined( USE_ALPHAMAP )
	#if defined( USE_POINTS_UV )
		vec2 uv = vUv;
	#else
		vec2 uv = ( uvTransform * vec3( gl_PointCoord.x, 1.0 - gl_PointCoord.y, 1 ) ).xy;
	#endif
#endif
#ifdef USE_MAP
	diffuseColor *= texture2D( map, uv );
#endif
#ifdef USE_ALPHAMAP
	diffuseColor.a *= texture2D( alphaMap, uv ).g;
#endif`,K_=`#if defined( USE_POINTS_UV )
	varying vec2 vUv;
#else
	#if defined( USE_MAP ) || defined( USE_ALPHAMAP )
		uniform mat3 uvTransform;
	#endif
#endif
#ifdef USE_MAP
	uniform sampler2D map;
#endif
#ifdef USE_ALPHAMAP
	uniform sampler2D alphaMap;
#endif`,Z_=`float metalnessFactor = metalness;
#ifdef USE_METALNESSMAP
	vec4 texelMetalness = texture2D( metalnessMap, vMetalnessMapUv );
	metalnessFactor *= texelMetalness.b;
#endif`,J_=`#ifdef USE_METALNESSMAP
	uniform sampler2D metalnessMap;
#endif`,Q_=`#ifdef USE_INSTANCING_MORPH
	float morphTargetInfluences[ MORPHTARGETS_COUNT ];
	float morphTargetBaseInfluence = texelFetch( morphTexture, ivec2( 0, gl_InstanceID ), 0 ).r;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		morphTargetInfluences[i] =  texelFetch( morphTexture, ivec2( i + 1, gl_InstanceID ), 0 ).r;
	}
#endif`,t0=`#if defined( USE_MORPHCOLORS )
	vColor *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		#if defined( USE_COLOR_ALPHA )
			if ( morphTargetInfluences[ i ] != 0.0 ) vColor += getMorph( gl_VertexID, i, 2 ) * morphTargetInfluences[ i ];
		#elif defined( USE_COLOR )
			if ( morphTargetInfluences[ i ] != 0.0 ) vColor += getMorph( gl_VertexID, i, 2 ).rgb * morphTargetInfluences[ i ];
		#endif
	}
#endif`,e0=`#ifdef USE_MORPHNORMALS
	objectNormal *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		if ( morphTargetInfluences[ i ] != 0.0 ) objectNormal += getMorph( gl_VertexID, i, 1 ).xyz * morphTargetInfluences[ i ];
	}
#endif`,n0=`#ifdef USE_MORPHTARGETS
	#ifndef USE_INSTANCING_MORPH
		uniform float morphTargetBaseInfluence;
		uniform float morphTargetInfluences[ MORPHTARGETS_COUNT ];
	#endif
	uniform sampler2DArray morphTargetsTexture;
	uniform ivec2 morphTargetsTextureSize;
	vec4 getMorph( const in int vertexIndex, const in int morphTargetIndex, const in int offset ) {
		int texelIndex = vertexIndex * MORPHTARGETS_TEXTURE_STRIDE + offset;
		int y = texelIndex / morphTargetsTextureSize.x;
		int x = texelIndex - y * morphTargetsTextureSize.x;
		ivec3 morphUV = ivec3( x, y, morphTargetIndex );
		return texelFetch( morphTargetsTexture, morphUV, 0 );
	}
#endif`,i0=`#ifdef USE_MORPHTARGETS
	transformed *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		if ( morphTargetInfluences[ i ] != 0.0 ) transformed += getMorph( gl_VertexID, i, 0 ).xyz * morphTargetInfluences[ i ];
	}
#endif`,s0=`float faceDirection = gl_FrontFacing ? 1.0 : - 1.0;
#ifdef FLAT_SHADED
	vec3 fdx = dFdx( vViewPosition );
	vec3 fdy = dFdy( vViewPosition );
	vec3 normal = normalize( cross( fdx, fdy ) );
#else
	vec3 normal = normalize( vNormal );
	#ifdef DOUBLE_SIDED
		normal *= faceDirection;
	#endif
#endif
#if defined( USE_NORMALMAP_TANGENTSPACE ) || defined( USE_CLEARCOAT_NORMALMAP ) || defined( USE_ANISOTROPY )
	#ifdef USE_TANGENT
		mat3 tbn = mat3( normalize( vTangent ), normalize( vBitangent ), normal );
	#else
		mat3 tbn = getTangentFrame( - vViewPosition, normal,
		#if defined( USE_NORMALMAP )
			vNormalMapUv
		#elif defined( USE_CLEARCOAT_NORMALMAP )
			vClearcoatNormalMapUv
		#else
			vUv
		#endif
		);
	#endif
	#if defined( DOUBLE_SIDED ) && ! defined( FLAT_SHADED )
		tbn[0] *= faceDirection;
		tbn[1] *= faceDirection;
	#endif
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	#ifdef USE_TANGENT
		mat3 tbn2 = mat3( normalize( vTangent ), normalize( vBitangent ), normal );
	#else
		mat3 tbn2 = getTangentFrame( - vViewPosition, normal, vClearcoatNormalMapUv );
	#endif
	#if defined( DOUBLE_SIDED ) && ! defined( FLAT_SHADED )
		tbn2[0] *= faceDirection;
		tbn2[1] *= faceDirection;
	#endif
#endif
vec3 nonPerturbedNormal = normal;`,r0=`#ifdef USE_NORMALMAP_OBJECTSPACE
	normal = texture2D( normalMap, vNormalMapUv ).xyz * 2.0 - 1.0;
	#ifdef FLIP_SIDED
		normal = - normal;
	#endif
	#ifdef DOUBLE_SIDED
		normal = normal * faceDirection;
	#endif
	normal = normalize( normalMatrix * normal );
#elif defined( USE_NORMALMAP_TANGENTSPACE )
	vec3 mapN = texture2D( normalMap, vNormalMapUv ).xyz * 2.0 - 1.0;
	mapN.xy *= normalScale;
	normal = normalize( tbn * mapN );
#elif defined( USE_BUMPMAP )
	normal = perturbNormalArb( - vViewPosition, normal, dHdxy_fwd(), faceDirection );
#endif`,o0=`#ifndef FLAT_SHADED
	varying vec3 vNormal;
	#ifdef USE_TANGENT
		varying vec3 vTangent;
		varying vec3 vBitangent;
	#endif
#endif`,a0=`#ifndef FLAT_SHADED
	varying vec3 vNormal;
	#ifdef USE_TANGENT
		varying vec3 vTangent;
		varying vec3 vBitangent;
	#endif
#endif`,l0=`#ifndef FLAT_SHADED
	vNormal = normalize( transformedNormal );
	#ifdef USE_TANGENT
		vTangent = normalize( transformedTangent );
		vBitangent = normalize( cross( vNormal, vTangent ) * tangent.w );
	#endif
#endif`,c0=`#ifdef USE_NORMALMAP
	uniform sampler2D normalMap;
	uniform vec2 normalScale;
#endif
#ifdef USE_NORMALMAP_OBJECTSPACE
	uniform mat3 normalMatrix;
#endif
#if ! defined ( USE_TANGENT ) && ( defined ( USE_NORMALMAP_TANGENTSPACE ) || defined ( USE_CLEARCOAT_NORMALMAP ) || defined( USE_ANISOTROPY ) )
	mat3 getTangentFrame( vec3 eye_pos, vec3 surf_norm, vec2 uv ) {
		vec3 q0 = dFdx( eye_pos.xyz );
		vec3 q1 = dFdy( eye_pos.xyz );
		vec2 st0 = dFdx( uv.st );
		vec2 st1 = dFdy( uv.st );
		vec3 N = surf_norm;
		vec3 q1perp = cross( q1, N );
		vec3 q0perp = cross( N, q0 );
		vec3 T = q1perp * st0.x + q0perp * st1.x;
		vec3 B = q1perp * st0.y + q0perp * st1.y;
		float det = max( dot( T, T ), dot( B, B ) );
		float scale = ( det == 0.0 ) ? 0.0 : inversesqrt( det );
		return mat3( T * scale, B * scale, N );
	}
#endif`,h0=`#ifdef USE_CLEARCOAT
	vec3 clearcoatNormal = nonPerturbedNormal;
#endif`,u0=`#ifdef USE_CLEARCOAT_NORMALMAP
	vec3 clearcoatMapN = texture2D( clearcoatNormalMap, vClearcoatNormalMapUv ).xyz * 2.0 - 1.0;
	clearcoatMapN.xy *= clearcoatNormalScale;
	clearcoatNormal = normalize( tbn2 * clearcoatMapN );
#endif`,d0=`#ifdef USE_CLEARCOATMAP
	uniform sampler2D clearcoatMap;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	uniform sampler2D clearcoatNormalMap;
	uniform vec2 clearcoatNormalScale;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	uniform sampler2D clearcoatRoughnessMap;
#endif`,f0=`#ifdef USE_IRIDESCENCEMAP
	uniform sampler2D iridescenceMap;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	uniform sampler2D iridescenceThicknessMap;
#endif`,p0=`#ifdef OPAQUE
diffuseColor.a = 1.0;
#endif
#ifdef USE_TRANSMISSION
diffuseColor.a *= material.transmissionAlpha;
#endif
gl_FragColor = vec4( outgoingLight, diffuseColor.a );`,m0=`vec3 packNormalToRGB( const in vec3 normal ) {
	return normalize( normal ) * 0.5 + 0.5;
}
vec3 unpackRGBToNormal( const in vec3 rgb ) {
	return 2.0 * rgb.xyz - 1.0;
}
const float PackUpscale = 256. / 255.;const float UnpackDownscale = 255. / 256.;const float ShiftRight8 = 1. / 256.;
const float Inv255 = 1. / 255.;
const vec4 PackFactors = vec4( 1.0, 256.0, 256.0 * 256.0, 256.0 * 256.0 * 256.0 );
const vec2 UnpackFactors2 = vec2( UnpackDownscale, 1.0 / PackFactors.g );
const vec3 UnpackFactors3 = vec3( UnpackDownscale / PackFactors.rg, 1.0 / PackFactors.b );
const vec4 UnpackFactors4 = vec4( UnpackDownscale / PackFactors.rgb, 1.0 / PackFactors.a );
vec4 packDepthToRGBA( const in float v ) {
	if( v <= 0.0 )
		return vec4( 0., 0., 0., 0. );
	if( v >= 1.0 )
		return vec4( 1., 1., 1., 1. );
	float vuf;
	float af = modf( v * PackFactors.a, vuf );
	float bf = modf( vuf * ShiftRight8, vuf );
	float gf = modf( vuf * ShiftRight8, vuf );
	return vec4( vuf * Inv255, gf * PackUpscale, bf * PackUpscale, af );
}
vec3 packDepthToRGB( const in float v ) {
	if( v <= 0.0 )
		return vec3( 0., 0., 0. );
	if( v >= 1.0 )
		return vec3( 1., 1., 1. );
	float vuf;
	float bf = modf( v * PackFactors.b, vuf );
	float gf = modf( vuf * ShiftRight8, vuf );
	return vec3( vuf * Inv255, gf * PackUpscale, bf );
}
vec2 packDepthToRG( const in float v ) {
	if( v <= 0.0 )
		return vec2( 0., 0. );
	if( v >= 1.0 )
		return vec2( 1., 1. );
	float vuf;
	float gf = modf( v * 256., vuf );
	return vec2( vuf * Inv255, gf );
}
float unpackRGBAToDepth( const in vec4 v ) {
	return dot( v, UnpackFactors4 );
}
float unpackRGBToDepth( const in vec3 v ) {
	return dot( v, UnpackFactors3 );
}
float unpackRGToDepth( const in vec2 v ) {
	return v.r * UnpackFactors2.r + v.g * UnpackFactors2.g;
}
vec4 pack2HalfToRGBA( const in vec2 v ) {
	vec4 r = vec4( v.x, fract( v.x * 255.0 ), v.y, fract( v.y * 255.0 ) );
	return vec4( r.x - r.y / 255.0, r.y, r.z - r.w / 255.0, r.w );
}
vec2 unpackRGBATo2Half( const in vec4 v ) {
	return vec2( v.x + ( v.y / 255.0 ), v.z + ( v.w / 255.0 ) );
}
float viewZToOrthographicDepth( const in float viewZ, const in float near, const in float far ) {
	return ( viewZ + near ) / ( near - far );
}
float orthographicDepthToViewZ( const in float depth, const in float near, const in float far ) {
	return depth * ( near - far ) - near;
}
float viewZToPerspectiveDepth( const in float viewZ, const in float near, const in float far ) {
	return ( ( near + viewZ ) * far ) / ( ( far - near ) * viewZ );
}
float perspectiveDepthToViewZ( const in float depth, const in float near, const in float far ) {
	return ( near * far ) / ( ( far - near ) * depth - far );
}`,g0=`#ifdef PREMULTIPLIED_ALPHA
	gl_FragColor.rgb *= gl_FragColor.a;
#endif`,_0=`vec4 mvPosition = vec4( transformed, 1.0 );
#ifdef USE_BATCHING
	mvPosition = batchingMatrix * mvPosition;
#endif
#ifdef USE_INSTANCING
	mvPosition = instanceMatrix * mvPosition;
#endif
mvPosition = modelViewMatrix * mvPosition;
gl_Position = projectionMatrix * mvPosition;`,x0=`#ifdef DITHERING
	gl_FragColor.rgb = dithering( gl_FragColor.rgb );
#endif`,v0=`#ifdef DITHERING
	vec3 dithering( vec3 color ) {
		float grid_position = rand( gl_FragCoord.xy );
		vec3 dither_shift_RGB = vec3( 0.25 / 255.0, -0.25 / 255.0, 0.25 / 255.0 );
		dither_shift_RGB = mix( 2.0 * dither_shift_RGB, -2.0 * dither_shift_RGB, grid_position );
		return color + dither_shift_RGB;
	}
#endif`,b0=`float roughnessFactor = roughness;
#ifdef USE_ROUGHNESSMAP
	vec4 texelRoughness = texture2D( roughnessMap, vRoughnessMapUv );
	roughnessFactor *= texelRoughness.g;
#endif`,y0=`#ifdef USE_ROUGHNESSMAP
	uniform sampler2D roughnessMap;
#endif`,M0=`#if NUM_SPOT_LIGHT_COORDS > 0
	varying vec4 vSpotLightCoord[ NUM_SPOT_LIGHT_COORDS ];
#endif
#if NUM_SPOT_LIGHT_MAPS > 0
	uniform sampler2D spotLightMap[ NUM_SPOT_LIGHT_MAPS ];
#endif
#ifdef USE_SHADOWMAP
	#if NUM_DIR_LIGHT_SHADOWS > 0
		uniform sampler2D directionalShadowMap[ NUM_DIR_LIGHT_SHADOWS ];
		varying vec4 vDirectionalShadowCoord[ NUM_DIR_LIGHT_SHADOWS ];
		struct DirectionalLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform DirectionalLightShadow directionalLightShadows[ NUM_DIR_LIGHT_SHADOWS ];
	#endif
	#if NUM_SPOT_LIGHT_SHADOWS > 0
		uniform sampler2D spotShadowMap[ NUM_SPOT_LIGHT_SHADOWS ];
		struct SpotLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform SpotLightShadow spotLightShadows[ NUM_SPOT_LIGHT_SHADOWS ];
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
		uniform sampler2D pointShadowMap[ NUM_POINT_LIGHT_SHADOWS ];
		varying vec4 vPointShadowCoord[ NUM_POINT_LIGHT_SHADOWS ];
		struct PointLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
			float shadowCameraNear;
			float shadowCameraFar;
		};
		uniform PointLightShadow pointLightShadows[ NUM_POINT_LIGHT_SHADOWS ];
	#endif
	float texture2DCompare( sampler2D depths, vec2 uv, float compare ) {
		return step( compare, unpackRGBAToDepth( texture2D( depths, uv ) ) );
	}
	vec2 texture2DDistribution( sampler2D shadow, vec2 uv ) {
		return unpackRGBATo2Half( texture2D( shadow, uv ) );
	}
	float VSMShadow (sampler2D shadow, vec2 uv, float compare ){
		float occlusion = 1.0;
		vec2 distribution = texture2DDistribution( shadow, uv );
		float hard_shadow = step( compare , distribution.x );
		if (hard_shadow != 1.0 ) {
			float distance = compare - distribution.x ;
			float variance = max( 0.00000, distribution.y * distribution.y );
			float softness_probability = variance / (variance + distance * distance );			softness_probability = clamp( ( softness_probability - 0.3 ) / ( 0.95 - 0.3 ), 0.0, 1.0 );			occlusion = clamp( max( hard_shadow, softness_probability ), 0.0, 1.0 );
		}
		return occlusion;
	}
	float getShadow( sampler2D shadowMap, vec2 shadowMapSize, float shadowIntensity, float shadowBias, float shadowRadius, vec4 shadowCoord ) {
		float shadow = 1.0;
		shadowCoord.xyz /= shadowCoord.w;
		shadowCoord.z += shadowBias;
		bool inFrustum = shadowCoord.x >= 0.0 && shadowCoord.x <= 1.0 && shadowCoord.y >= 0.0 && shadowCoord.y <= 1.0;
		bool frustumTest = inFrustum && shadowCoord.z <= 1.0;
		if ( frustumTest ) {
		#if defined( SHADOWMAP_TYPE_PCF )
			vec2 texelSize = vec2( 1.0 ) / shadowMapSize;
			float dx0 = - texelSize.x * shadowRadius;
			float dy0 = - texelSize.y * shadowRadius;
			float dx1 = + texelSize.x * shadowRadius;
			float dy1 = + texelSize.y * shadowRadius;
			float dx2 = dx0 / 2.0;
			float dy2 = dy0 / 2.0;
			float dx3 = dx1 / 2.0;
			float dy3 = dy1 / 2.0;
			shadow = (
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx0, dy0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx1, dy0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx2, dy2 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy2 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx3, dy2 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx0, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx2, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy, shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx3, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx1, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx2, dy3 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy3 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx3, dy3 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx0, dy1 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy1 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx1, dy1 ), shadowCoord.z )
			) * ( 1.0 / 17.0 );
		#elif defined( SHADOWMAP_TYPE_PCF_SOFT )
			vec2 texelSize = vec2( 1.0 ) / shadowMapSize;
			float dx = texelSize.x;
			float dy = texelSize.y;
			vec2 uv = shadowCoord.xy;
			vec2 f = fract( uv * shadowMapSize + 0.5 );
			uv -= f * texelSize;
			shadow = (
				texture2DCompare( shadowMap, uv, shadowCoord.z ) +
				texture2DCompare( shadowMap, uv + vec2( dx, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, uv + vec2( 0.0, dy ), shadowCoord.z ) +
				texture2DCompare( shadowMap, uv + texelSize, shadowCoord.z ) +
				mix( texture2DCompare( shadowMap, uv + vec2( -dx, 0.0 ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, 0.0 ), shadowCoord.z ),
					 f.x ) +
				mix( texture2DCompare( shadowMap, uv + vec2( -dx, dy ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, dy ), shadowCoord.z ),
					 f.x ) +
				mix( texture2DCompare( shadowMap, uv + vec2( 0.0, -dy ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( 0.0, 2.0 * dy ), shadowCoord.z ),
					 f.y ) +
				mix( texture2DCompare( shadowMap, uv + vec2( dx, -dy ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( dx, 2.0 * dy ), shadowCoord.z ),
					 f.y ) +
				mix( mix( texture2DCompare( shadowMap, uv + vec2( -dx, -dy ), shadowCoord.z ),
						  texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, -dy ), shadowCoord.z ),
						  f.x ),
					 mix( texture2DCompare( shadowMap, uv + vec2( -dx, 2.0 * dy ), shadowCoord.z ),
						  texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, 2.0 * dy ), shadowCoord.z ),
						  f.x ),
					 f.y )
			) * ( 1.0 / 9.0 );
		#elif defined( SHADOWMAP_TYPE_VSM )
			shadow = VSMShadow( shadowMap, shadowCoord.xy, shadowCoord.z );
		#else
			shadow = texture2DCompare( shadowMap, shadowCoord.xy, shadowCoord.z );
		#endif
		}
		return mix( 1.0, shadow, shadowIntensity );
	}
	vec2 cubeToUV( vec3 v, float texelSizeY ) {
		vec3 absV = abs( v );
		float scaleToCube = 1.0 / max( absV.x, max( absV.y, absV.z ) );
		absV *= scaleToCube;
		v *= scaleToCube * ( 1.0 - 2.0 * texelSizeY );
		vec2 planar = v.xy;
		float almostATexel = 1.5 * texelSizeY;
		float almostOne = 1.0 - almostATexel;
		if ( absV.z >= almostOne ) {
			if ( v.z > 0.0 )
				planar.x = 4.0 - v.x;
		} else if ( absV.x >= almostOne ) {
			float signX = sign( v.x );
			planar.x = v.z * signX + 2.0 * signX;
		} else if ( absV.y >= almostOne ) {
			float signY = sign( v.y );
			planar.x = v.x + 2.0 * signY + 2.0;
			planar.y = v.z * signY - 2.0;
		}
		return vec2( 0.125, 0.25 ) * planar + vec2( 0.375, 0.75 );
	}
	float getPointShadow( sampler2D shadowMap, vec2 shadowMapSize, float shadowIntensity, float shadowBias, float shadowRadius, vec4 shadowCoord, float shadowCameraNear, float shadowCameraFar ) {
		float shadow = 1.0;
		vec3 lightToPosition = shadowCoord.xyz;
		
		float lightToPositionLength = length( lightToPosition );
		if ( lightToPositionLength - shadowCameraFar <= 0.0 && lightToPositionLength - shadowCameraNear >= 0.0 ) {
			float dp = ( lightToPositionLength - shadowCameraNear ) / ( shadowCameraFar - shadowCameraNear );			dp += shadowBias;
			vec3 bd3D = normalize( lightToPosition );
			vec2 texelSize = vec2( 1.0 ) / ( shadowMapSize * vec2( 4.0, 2.0 ) );
			#if defined( SHADOWMAP_TYPE_PCF ) || defined( SHADOWMAP_TYPE_PCF_SOFT ) || defined( SHADOWMAP_TYPE_VSM )
				vec2 offset = vec2( - 1, 1 ) * shadowRadius * texelSize.y;
				shadow = (
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xyy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yyy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xyx, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yyx, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xxy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yxy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xxx, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yxx, texelSize.y ), dp )
				) * ( 1.0 / 9.0 );
			#else
				shadow = texture2DCompare( shadowMap, cubeToUV( bd3D, texelSize.y ), dp );
			#endif
		}
		return mix( 1.0, shadow, shadowIntensity );
	}
#endif`,S0=`#if NUM_SPOT_LIGHT_COORDS > 0
	uniform mat4 spotLightMatrix[ NUM_SPOT_LIGHT_COORDS ];
	varying vec4 vSpotLightCoord[ NUM_SPOT_LIGHT_COORDS ];
#endif
#ifdef USE_SHADOWMAP
	#if NUM_DIR_LIGHT_SHADOWS > 0
		uniform mat4 directionalShadowMatrix[ NUM_DIR_LIGHT_SHADOWS ];
		varying vec4 vDirectionalShadowCoord[ NUM_DIR_LIGHT_SHADOWS ];
		struct DirectionalLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform DirectionalLightShadow directionalLightShadows[ NUM_DIR_LIGHT_SHADOWS ];
	#endif
	#if NUM_SPOT_LIGHT_SHADOWS > 0
		struct SpotLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform SpotLightShadow spotLightShadows[ NUM_SPOT_LIGHT_SHADOWS ];
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
		uniform mat4 pointShadowMatrix[ NUM_POINT_LIGHT_SHADOWS ];
		varying vec4 vPointShadowCoord[ NUM_POINT_LIGHT_SHADOWS ];
		struct PointLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
			float shadowCameraNear;
			float shadowCameraFar;
		};
		uniform PointLightShadow pointLightShadows[ NUM_POINT_LIGHT_SHADOWS ];
	#endif
#endif`,E0=`#if ( defined( USE_SHADOWMAP ) && ( NUM_DIR_LIGHT_SHADOWS > 0 || NUM_POINT_LIGHT_SHADOWS > 0 ) ) || ( NUM_SPOT_LIGHT_COORDS > 0 )
	vec3 shadowWorldNormal = inverseTransformDirection( transformedNormal, viewMatrix );
	vec4 shadowWorldPosition;
#endif
#if defined( USE_SHADOWMAP )
	#if NUM_DIR_LIGHT_SHADOWS > 0
		#pragma unroll_loop_start
		for ( int i = 0; i < NUM_DIR_LIGHT_SHADOWS; i ++ ) {
			shadowWorldPosition = worldPosition + vec4( shadowWorldNormal * directionalLightShadows[ i ].shadowNormalBias, 0 );
			vDirectionalShadowCoord[ i ] = directionalShadowMatrix[ i ] * shadowWorldPosition;
		}
		#pragma unroll_loop_end
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
		#pragma unroll_loop_start
		for ( int i = 0; i < NUM_POINT_LIGHT_SHADOWS; i ++ ) {
			shadowWorldPosition = worldPosition + vec4( shadowWorldNormal * pointLightShadows[ i ].shadowNormalBias, 0 );
			vPointShadowCoord[ i ] = pointShadowMatrix[ i ] * shadowWorldPosition;
		}
		#pragma unroll_loop_end
	#endif
#endif
#if NUM_SPOT_LIGHT_COORDS > 0
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_SPOT_LIGHT_COORDS; i ++ ) {
		shadowWorldPosition = worldPosition;
		#if ( defined( USE_SHADOWMAP ) && UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS )
			shadowWorldPosition.xyz += shadowWorldNormal * spotLightShadows[ i ].shadowNormalBias;
		#endif
		vSpotLightCoord[ i ] = spotLightMatrix[ i ] * shadowWorldPosition;
	}
	#pragma unroll_loop_end
#endif`,T0=`float getShadowMask() {
	float shadow = 1.0;
	#ifdef USE_SHADOWMAP
	#if NUM_DIR_LIGHT_SHADOWS > 0
	DirectionalLightShadow directionalLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_DIR_LIGHT_SHADOWS; i ++ ) {
		directionalLight = directionalLightShadows[ i ];
		shadow *= receiveShadow ? getShadow( directionalShadowMap[ i ], directionalLight.shadowMapSize, directionalLight.shadowIntensity, directionalLight.shadowBias, directionalLight.shadowRadius, vDirectionalShadowCoord[ i ] ) : 1.0;
	}
	#pragma unroll_loop_end
	#endif
	#if NUM_SPOT_LIGHT_SHADOWS > 0
	SpotLightShadow spotLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_SPOT_LIGHT_SHADOWS; i ++ ) {
		spotLight = spotLightShadows[ i ];
		shadow *= receiveShadow ? getShadow( spotShadowMap[ i ], spotLight.shadowMapSize, spotLight.shadowIntensity, spotLight.shadowBias, spotLight.shadowRadius, vSpotLightCoord[ i ] ) : 1.0;
	}
	#pragma unroll_loop_end
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
	PointLightShadow pointLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_POINT_LIGHT_SHADOWS; i ++ ) {
		pointLight = pointLightShadows[ i ];
		shadow *= receiveShadow ? getPointShadow( pointShadowMap[ i ], pointLight.shadowMapSize, pointLight.shadowIntensity, pointLight.shadowBias, pointLight.shadowRadius, vPointShadowCoord[ i ], pointLight.shadowCameraNear, pointLight.shadowCameraFar ) : 1.0;
	}
	#pragma unroll_loop_end
	#endif
	#endif
	return shadow;
}`,w0=`#ifdef USE_SKINNING
	mat4 boneMatX = getBoneMatrix( skinIndex.x );
	mat4 boneMatY = getBoneMatrix( skinIndex.y );
	mat4 boneMatZ = getBoneMatrix( skinIndex.z );
	mat4 boneMatW = getBoneMatrix( skinIndex.w );
#endif`,A0=`#ifdef USE_SKINNING
	uniform mat4 bindMatrix;
	uniform mat4 bindMatrixInverse;
	uniform highp sampler2D boneTexture;
	mat4 getBoneMatrix( const in float i ) {
		int size = textureSize( boneTexture, 0 ).x;
		int j = int( i ) * 4;
		int x = j % size;
		int y = j / size;
		vec4 v1 = texelFetch( boneTexture, ivec2( x, y ), 0 );
		vec4 v2 = texelFetch( boneTexture, ivec2( x + 1, y ), 0 );
		vec4 v3 = texelFetch( boneTexture, ivec2( x + 2, y ), 0 );
		vec4 v4 = texelFetch( boneTexture, ivec2( x + 3, y ), 0 );
		return mat4( v1, v2, v3, v4 );
	}
#endif`,C0=`#ifdef USE_SKINNING
	vec4 skinVertex = bindMatrix * vec4( transformed, 1.0 );
	vec4 skinned = vec4( 0.0 );
	skinned += boneMatX * skinVertex * skinWeight.x;
	skinned += boneMatY * skinVertex * skinWeight.y;
	skinned += boneMatZ * skinVertex * skinWeight.z;
	skinned += boneMatW * skinVertex * skinWeight.w;
	transformed = ( bindMatrixInverse * skinned ).xyz;
#endif`,R0=`#ifdef USE_SKINNING
	mat4 skinMatrix = mat4( 0.0 );
	skinMatrix += skinWeight.x * boneMatX;
	skinMatrix += skinWeight.y * boneMatY;
	skinMatrix += skinWeight.z * boneMatZ;
	skinMatrix += skinWeight.w * boneMatW;
	skinMatrix = bindMatrixInverse * skinMatrix * bindMatrix;
	objectNormal = vec4( skinMatrix * vec4( objectNormal, 0.0 ) ).xyz;
	#ifdef USE_TANGENT
		objectTangent = vec4( skinMatrix * vec4( objectTangent, 0.0 ) ).xyz;
	#endif
#endif`,P0=`float specularStrength;
#ifdef USE_SPECULARMAP
	vec4 texelSpecular = texture2D( specularMap, vSpecularMapUv );
	specularStrength = texelSpecular.r;
#else
	specularStrength = 1.0;
#endif`,D0=`#ifdef USE_SPECULARMAP
	uniform sampler2D specularMap;
#endif`,L0=`#if defined( TONE_MAPPING )
	gl_FragColor.rgb = toneMapping( gl_FragColor.rgb );
#endif`,I0=`#ifndef saturate
#define saturate( a ) clamp( a, 0.0, 1.0 )
#endif
uniform float toneMappingExposure;
vec3 LinearToneMapping( vec3 color ) {
	return saturate( toneMappingExposure * color );
}
vec3 ReinhardToneMapping( vec3 color ) {
	color *= toneMappingExposure;
	return saturate( color / ( vec3( 1.0 ) + color ) );
}
vec3 CineonToneMapping( vec3 color ) {
	color *= toneMappingExposure;
	color = max( vec3( 0.0 ), color - 0.004 );
	return pow( ( color * ( 6.2 * color + 0.5 ) ) / ( color * ( 6.2 * color + 1.7 ) + 0.06 ), vec3( 2.2 ) );
}
vec3 RRTAndODTFit( vec3 v ) {
	vec3 a = v * ( v + 0.0245786 ) - 0.000090537;
	vec3 b = v * ( 0.983729 * v + 0.4329510 ) + 0.238081;
	return a / b;
}
vec3 ACESFilmicToneMapping( vec3 color ) {
	const mat3 ACESInputMat = mat3(
		vec3( 0.59719, 0.07600, 0.02840 ),		vec3( 0.35458, 0.90834, 0.13383 ),
		vec3( 0.04823, 0.01566, 0.83777 )
	);
	const mat3 ACESOutputMat = mat3(
		vec3(  1.60475, -0.10208, -0.00327 ),		vec3( -0.53108,  1.10813, -0.07276 ),
		vec3( -0.07367, -0.00605,  1.07602 )
	);
	color *= toneMappingExposure / 0.6;
	color = ACESInputMat * color;
	color = RRTAndODTFit( color );
	color = ACESOutputMat * color;
	return saturate( color );
}
const mat3 LINEAR_REC2020_TO_LINEAR_SRGB = mat3(
	vec3( 1.6605, - 0.1246, - 0.0182 ),
	vec3( - 0.5876, 1.1329, - 0.1006 ),
	vec3( - 0.0728, - 0.0083, 1.1187 )
);
const mat3 LINEAR_SRGB_TO_LINEAR_REC2020 = mat3(
	vec3( 0.6274, 0.0691, 0.0164 ),
	vec3( 0.3293, 0.9195, 0.0880 ),
	vec3( 0.0433, 0.0113, 0.8956 )
);
vec3 agxDefaultContrastApprox( vec3 x ) {
	vec3 x2 = x * x;
	vec3 x4 = x2 * x2;
	return + 15.5 * x4 * x2
		- 40.14 * x4 * x
		+ 31.96 * x4
		- 6.868 * x2 * x
		+ 0.4298 * x2
		+ 0.1191 * x
		- 0.00232;
}
vec3 AgXToneMapping( vec3 color ) {
	const mat3 AgXInsetMatrix = mat3(
		vec3( 0.856627153315983, 0.137318972929847, 0.11189821299995 ),
		vec3( 0.0951212405381588, 0.761241990602591, 0.0767994186031903 ),
		vec3( 0.0482516061458583, 0.101439036467562, 0.811302368396859 )
	);
	const mat3 AgXOutsetMatrix = mat3(
		vec3( 1.1271005818144368, - 0.1413297634984383, - 0.14132976349843826 ),
		vec3( - 0.11060664309660323, 1.157823702216272, - 0.11060664309660294 ),
		vec3( - 0.016493938717834573, - 0.016493938717834257, 1.2519364065950405 )
	);
	const float AgxMinEv = - 12.47393;	const float AgxMaxEv = 4.026069;
	color *= toneMappingExposure;
	color = LINEAR_SRGB_TO_LINEAR_REC2020 * color;
	color = AgXInsetMatrix * color;
	color = max( color, 1e-10 );	color = log2( color );
	color = ( color - AgxMinEv ) / ( AgxMaxEv - AgxMinEv );
	color = clamp( color, 0.0, 1.0 );
	color = agxDefaultContrastApprox( color );
	color = AgXOutsetMatrix * color;
	color = pow( max( vec3( 0.0 ), color ), vec3( 2.2 ) );
	color = LINEAR_REC2020_TO_LINEAR_SRGB * color;
	color = clamp( color, 0.0, 1.0 );
	return color;
}
vec3 NeutralToneMapping( vec3 color ) {
	const float StartCompression = 0.8 - 0.04;
	const float Desaturation = 0.15;
	color *= toneMappingExposure;
	float x = min( color.r, min( color.g, color.b ) );
	float offset = x < 0.08 ? x - 6.25 * x * x : 0.04;
	color -= offset;
	float peak = max( color.r, max( color.g, color.b ) );
	if ( peak < StartCompression ) return color;
	float d = 1. - StartCompression;
	float newPeak = 1. - d * d / ( peak + d - StartCompression );
	color *= newPeak / peak;
	float g = 1. - 1. / ( Desaturation * ( peak - newPeak ) + 1. );
	return mix( color, vec3( newPeak ), g );
}
vec3 CustomToneMapping( vec3 color ) { return color; }`,O0=`#ifdef USE_TRANSMISSION
	material.transmission = transmission;
	material.transmissionAlpha = 1.0;
	material.thickness = thickness;
	material.attenuationDistance = attenuationDistance;
	material.attenuationColor = attenuationColor;
	#ifdef USE_TRANSMISSIONMAP
		material.transmission *= texture2D( transmissionMap, vTransmissionMapUv ).r;
	#endif
	#ifdef USE_THICKNESSMAP
		material.thickness *= texture2D( thicknessMap, vThicknessMapUv ).g;
	#endif
	vec3 pos = vWorldPosition;
	vec3 v = normalize( cameraPosition - pos );
	vec3 n = inverseTransformDirection( normal, viewMatrix );
	vec4 transmitted = getIBLVolumeRefraction(
		n, v, material.roughness, material.diffuseColor, material.specularColor, material.specularF90,
		pos, modelMatrix, viewMatrix, projectionMatrix, material.dispersion, material.ior, material.thickness,
		material.attenuationColor, material.attenuationDistance );
	material.transmissionAlpha = mix( material.transmissionAlpha, transmitted.a, material.transmission );
	totalDiffuse = mix( totalDiffuse, transmitted.rgb, material.transmission );
#endif`,U0=`#ifdef USE_TRANSMISSION
	uniform float transmission;
	uniform float thickness;
	uniform float attenuationDistance;
	uniform vec3 attenuationColor;
	#ifdef USE_TRANSMISSIONMAP
		uniform sampler2D transmissionMap;
	#endif
	#ifdef USE_THICKNESSMAP
		uniform sampler2D thicknessMap;
	#endif
	uniform vec2 transmissionSamplerSize;
	uniform sampler2D transmissionSamplerMap;
	uniform mat4 modelMatrix;
	uniform mat4 projectionMatrix;
	varying vec3 vWorldPosition;
	float w0( float a ) {
		return ( 1.0 / 6.0 ) * ( a * ( a * ( - a + 3.0 ) - 3.0 ) + 1.0 );
	}
	float w1( float a ) {
		return ( 1.0 / 6.0 ) * ( a *  a * ( 3.0 * a - 6.0 ) + 4.0 );
	}
	float w2( float a ){
		return ( 1.0 / 6.0 ) * ( a * ( a * ( - 3.0 * a + 3.0 ) + 3.0 ) + 1.0 );
	}
	float w3( float a ) {
		return ( 1.0 / 6.0 ) * ( a * a * a );
	}
	float g0( float a ) {
		return w0( a ) + w1( a );
	}
	float g1( float a ) {
		return w2( a ) + w3( a );
	}
	float h0( float a ) {
		return - 1.0 + w1( a ) / ( w0( a ) + w1( a ) );
	}
	float h1( float a ) {
		return 1.0 + w3( a ) / ( w2( a ) + w3( a ) );
	}
	vec4 bicubic( sampler2D tex, vec2 uv, vec4 texelSize, float lod ) {
		uv = uv * texelSize.zw + 0.5;
		vec2 iuv = floor( uv );
		vec2 fuv = fract( uv );
		float g0x = g0( fuv.x );
		float g1x = g1( fuv.x );
		float h0x = h0( fuv.x );
		float h1x = h1( fuv.x );
		float h0y = h0( fuv.y );
		float h1y = h1( fuv.y );
		vec2 p0 = ( vec2( iuv.x + h0x, iuv.y + h0y ) - 0.5 ) * texelSize.xy;
		vec2 p1 = ( vec2( iuv.x + h1x, iuv.y + h0y ) - 0.5 ) * texelSize.xy;
		vec2 p2 = ( vec2( iuv.x + h0x, iuv.y + h1y ) - 0.5 ) * texelSize.xy;
		vec2 p3 = ( vec2( iuv.x + h1x, iuv.y + h1y ) - 0.5 ) * texelSize.xy;
		return g0( fuv.y ) * ( g0x * textureLod( tex, p0, lod ) + g1x * textureLod( tex, p1, lod ) ) +
			g1( fuv.y ) * ( g0x * textureLod( tex, p2, lod ) + g1x * textureLod( tex, p3, lod ) );
	}
	vec4 textureBicubic( sampler2D sampler, vec2 uv, float lod ) {
		vec2 fLodSize = vec2( textureSize( sampler, int( lod ) ) );
		vec2 cLodSize = vec2( textureSize( sampler, int( lod + 1.0 ) ) );
		vec2 fLodSizeInv = 1.0 / fLodSize;
		vec2 cLodSizeInv = 1.0 / cLodSize;
		vec4 fSample = bicubic( sampler, uv, vec4( fLodSizeInv, fLodSize ), floor( lod ) );
		vec4 cSample = bicubic( sampler, uv, vec4( cLodSizeInv, cLodSize ), ceil( lod ) );
		return mix( fSample, cSample, fract( lod ) );
	}
	vec3 getVolumeTransmissionRay( const in vec3 n, const in vec3 v, const in float thickness, const in float ior, const in mat4 modelMatrix ) {
		vec3 refractionVector = refract( - v, normalize( n ), 1.0 / ior );
		vec3 modelScale;
		modelScale.x = length( vec3( modelMatrix[ 0 ].xyz ) );
		modelScale.y = length( vec3( modelMatrix[ 1 ].xyz ) );
		modelScale.z = length( vec3( modelMatrix[ 2 ].xyz ) );
		return normalize( refractionVector ) * thickness * modelScale;
	}
	float applyIorToRoughness( const in float roughness, const in float ior ) {
		return roughness * clamp( ior * 2.0 - 2.0, 0.0, 1.0 );
	}
	vec4 getTransmissionSample( const in vec2 fragCoord, const in float roughness, const in float ior ) {
		float lod = log2( transmissionSamplerSize.x ) * applyIorToRoughness( roughness, ior );
		return textureBicubic( transmissionSamplerMap, fragCoord.xy, lod );
	}
	vec3 volumeAttenuation( const in float transmissionDistance, const in vec3 attenuationColor, const in float attenuationDistance ) {
		if ( isinf( attenuationDistance ) ) {
			return vec3( 1.0 );
		} else {
			vec3 attenuationCoefficient = -log( attenuationColor ) / attenuationDistance;
			vec3 transmittance = exp( - attenuationCoefficient * transmissionDistance );			return transmittance;
		}
	}
	vec4 getIBLVolumeRefraction( const in vec3 n, const in vec3 v, const in float roughness, const in vec3 diffuseColor,
		const in vec3 specularColor, const in float specularF90, const in vec3 position, const in mat4 modelMatrix,
		const in mat4 viewMatrix, const in mat4 projMatrix, const in float dispersion, const in float ior, const in float thickness,
		const in vec3 attenuationColor, const in float attenuationDistance ) {
		vec4 transmittedLight;
		vec3 transmittance;
		#ifdef USE_DISPERSION
			float halfSpread = ( ior - 1.0 ) * 0.025 * dispersion;
			vec3 iors = vec3( ior - halfSpread, ior, ior + halfSpread );
			for ( int i = 0; i < 3; i ++ ) {
				vec3 transmissionRay = getVolumeTransmissionRay( n, v, thickness, iors[ i ], modelMatrix );
				vec3 refractedRayExit = position + transmissionRay;
		
				vec4 ndcPos = projMatrix * viewMatrix * vec4( refractedRayExit, 1.0 );
				vec2 refractionCoords = ndcPos.xy / ndcPos.w;
				refractionCoords += 1.0;
				refractionCoords /= 2.0;
		
				vec4 transmissionSample = getTransmissionSample( refractionCoords, roughness, iors[ i ] );
				transmittedLight[ i ] = transmissionSample[ i ];
				transmittedLight.a += transmissionSample.a;
				transmittance[ i ] = diffuseColor[ i ] * volumeAttenuation( length( transmissionRay ), attenuationColor, attenuationDistance )[ i ];
			}
			transmittedLight.a /= 3.0;
		
		#else
		
			vec3 transmissionRay = getVolumeTransmissionRay( n, v, thickness, ior, modelMatrix );
			vec3 refractedRayExit = position + transmissionRay;
			vec4 ndcPos = projMatrix * viewMatrix * vec4( refractedRayExit, 1.0 );
			vec2 refractionCoords = ndcPos.xy / ndcPos.w;
			refractionCoords += 1.0;
			refractionCoords /= 2.0;
			transmittedLight = getTransmissionSample( refractionCoords, roughness, ior );
			transmittance = diffuseColor * volumeAttenuation( length( transmissionRay ), attenuationColor, attenuationDistance );
		
		#endif
		vec3 attenuatedColor = transmittance * transmittedLight.rgb;
		vec3 F = EnvironmentBRDF( n, v, specularColor, specularF90, roughness );
		float transmittanceFactor = ( transmittance.r + transmittance.g + transmittance.b ) / 3.0;
		return vec4( ( 1.0 - F ) * attenuatedColor, 1.0 - ( 1.0 - transmittedLight.a ) * transmittanceFactor );
	}
#endif`,N0=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
	varying vec2 vUv;
#endif
#ifdef USE_MAP
	varying vec2 vMapUv;
#endif
#ifdef USE_ALPHAMAP
	varying vec2 vAlphaMapUv;
#endif
#ifdef USE_LIGHTMAP
	varying vec2 vLightMapUv;
#endif
#ifdef USE_AOMAP
	varying vec2 vAoMapUv;
#endif
#ifdef USE_BUMPMAP
	varying vec2 vBumpMapUv;
#endif
#ifdef USE_NORMALMAP
	varying vec2 vNormalMapUv;
#endif
#ifdef USE_EMISSIVEMAP
	varying vec2 vEmissiveMapUv;
#endif
#ifdef USE_METALNESSMAP
	varying vec2 vMetalnessMapUv;
#endif
#ifdef USE_ROUGHNESSMAP
	varying vec2 vRoughnessMapUv;
#endif
#ifdef USE_ANISOTROPYMAP
	varying vec2 vAnisotropyMapUv;
#endif
#ifdef USE_CLEARCOATMAP
	varying vec2 vClearcoatMapUv;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	varying vec2 vClearcoatNormalMapUv;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	varying vec2 vClearcoatRoughnessMapUv;
#endif
#ifdef USE_IRIDESCENCEMAP
	varying vec2 vIridescenceMapUv;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	varying vec2 vIridescenceThicknessMapUv;
#endif
#ifdef USE_SHEEN_COLORMAP
	varying vec2 vSheenColorMapUv;
#endif
#ifdef USE_SHEEN_ROUGHNESSMAP
	varying vec2 vSheenRoughnessMapUv;
#endif
#ifdef USE_SPECULARMAP
	varying vec2 vSpecularMapUv;
#endif
#ifdef USE_SPECULAR_COLORMAP
	varying vec2 vSpecularColorMapUv;
#endif
#ifdef USE_SPECULAR_INTENSITYMAP
	varying vec2 vSpecularIntensityMapUv;
#endif
#ifdef USE_TRANSMISSIONMAP
	uniform mat3 transmissionMapTransform;
	varying vec2 vTransmissionMapUv;
#endif
#ifdef USE_THICKNESSMAP
	uniform mat3 thicknessMapTransform;
	varying vec2 vThicknessMapUv;
#endif`,F0=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
	varying vec2 vUv;
#endif
#ifdef USE_MAP
	uniform mat3 mapTransform;
	varying vec2 vMapUv;
#endif
#ifdef USE_ALPHAMAP
	uniform mat3 alphaMapTransform;
	varying vec2 vAlphaMapUv;
#endif
#ifdef USE_LIGHTMAP
	uniform mat3 lightMapTransform;
	varying vec2 vLightMapUv;
#endif
#ifdef USE_AOMAP
	uniform mat3 aoMapTransform;
	varying vec2 vAoMapUv;
#endif
#ifdef USE_BUMPMAP
	uniform mat3 bumpMapTransform;
	varying vec2 vBumpMapUv;
#endif
#ifdef USE_NORMALMAP
	uniform mat3 normalMapTransform;
	varying vec2 vNormalMapUv;
#endif
#ifdef USE_DISPLACEMENTMAP
	uniform mat3 displacementMapTransform;
	varying vec2 vDisplacementMapUv;
#endif
#ifdef USE_EMISSIVEMAP
	uniform mat3 emissiveMapTransform;
	varying vec2 vEmissiveMapUv;
#endif
#ifdef USE_METALNESSMAP
	uniform mat3 metalnessMapTransform;
	varying vec2 vMetalnessMapUv;
#endif
#ifdef USE_ROUGHNESSMAP
	uniform mat3 roughnessMapTransform;
	varying vec2 vRoughnessMapUv;
#endif
#ifdef USE_ANISOTROPYMAP
	uniform mat3 anisotropyMapTransform;
	varying vec2 vAnisotropyMapUv;
#endif
#ifdef USE_CLEARCOATMAP
	uniform mat3 clearcoatMapTransform;
	varying vec2 vClearcoatMapUv;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	uniform mat3 clearcoatNormalMapTransform;
	varying vec2 vClearcoatNormalMapUv;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	uniform mat3 clearcoatRoughnessMapTransform;
	varying vec2 vClearcoatRoughnessMapUv;
#endif
#ifdef USE_SHEEN_COLORMAP
	uniform mat3 sheenColorMapTransform;
	varying vec2 vSheenColorMapUv;
#endif
#ifdef USE_SHEEN_ROUGHNESSMAP
	uniform mat3 sheenRoughnessMapTransform;
	varying vec2 vSheenRoughnessMapUv;
#endif
#ifdef USE_IRIDESCENCEMAP
	uniform mat3 iridescenceMapTransform;
	varying vec2 vIridescenceMapUv;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	uniform mat3 iridescenceThicknessMapTransform;
	varying vec2 vIridescenceThicknessMapUv;
#endif
#ifdef USE_SPECULARMAP
	uniform mat3 specularMapTransform;
	varying vec2 vSpecularMapUv;
#endif
#ifdef USE_SPECULAR_COLORMAP
	uniform mat3 specularColorMapTransform;
	varying vec2 vSpecularColorMapUv;
#endif
#ifdef USE_SPECULAR_INTENSITYMAP
	uniform mat3 specularIntensityMapTransform;
	varying vec2 vSpecularIntensityMapUv;
#endif
#ifdef USE_TRANSMISSIONMAP
	uniform mat3 transmissionMapTransform;
	varying vec2 vTransmissionMapUv;
#endif
#ifdef USE_THICKNESSMAP
	uniform mat3 thicknessMapTransform;
	varying vec2 vThicknessMapUv;
#endif`,k0=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
	vUv = vec3( uv, 1 ).xy;
#endif
#ifdef USE_MAP
	vMapUv = ( mapTransform * vec3( MAP_UV, 1 ) ).xy;
#endif
#ifdef USE_ALPHAMAP
	vAlphaMapUv = ( alphaMapTransform * vec3( ALPHAMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_LIGHTMAP
	vLightMapUv = ( lightMapTransform * vec3( LIGHTMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_AOMAP
	vAoMapUv = ( aoMapTransform * vec3( AOMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_BUMPMAP
	vBumpMapUv = ( bumpMapTransform * vec3( BUMPMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_NORMALMAP
	vNormalMapUv = ( normalMapTransform * vec3( NORMALMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_DISPLACEMENTMAP
	vDisplacementMapUv = ( displacementMapTransform * vec3( DISPLACEMENTMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_EMISSIVEMAP
	vEmissiveMapUv = ( emissiveMapTransform * vec3( EMISSIVEMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_METALNESSMAP
	vMetalnessMapUv = ( metalnessMapTransform * vec3( METALNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_ROUGHNESSMAP
	vRoughnessMapUv = ( roughnessMapTransform * vec3( ROUGHNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_ANISOTROPYMAP
	vAnisotropyMapUv = ( anisotropyMapTransform * vec3( ANISOTROPYMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_CLEARCOATMAP
	vClearcoatMapUv = ( clearcoatMapTransform * vec3( CLEARCOATMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	vClearcoatNormalMapUv = ( clearcoatNormalMapTransform * vec3( CLEARCOAT_NORMALMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	vClearcoatRoughnessMapUv = ( clearcoatRoughnessMapTransform * vec3( CLEARCOAT_ROUGHNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_IRIDESCENCEMAP
	vIridescenceMapUv = ( iridescenceMapTransform * vec3( IRIDESCENCEMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	vIridescenceThicknessMapUv = ( iridescenceThicknessMapTransform * vec3( IRIDESCENCE_THICKNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SHEEN_COLORMAP
	vSheenColorMapUv = ( sheenColorMapTransform * vec3( SHEEN_COLORMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SHEEN_ROUGHNESSMAP
	vSheenRoughnessMapUv = ( sheenRoughnessMapTransform * vec3( SHEEN_ROUGHNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SPECULARMAP
	vSpecularMapUv = ( specularMapTransform * vec3( SPECULARMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SPECULAR_COLORMAP
	vSpecularColorMapUv = ( specularColorMapTransform * vec3( SPECULAR_COLORMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SPECULAR_INTENSITYMAP
	vSpecularIntensityMapUv = ( specularIntensityMapTransform * vec3( SPECULAR_INTENSITYMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_TRANSMISSIONMAP
	vTransmissionMapUv = ( transmissionMapTransform * vec3( TRANSMISSIONMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_THICKNESSMAP
	vThicknessMapUv = ( thicknessMapTransform * vec3( THICKNESSMAP_UV, 1 ) ).xy;
#endif`,B0=`#if defined( USE_ENVMAP ) || defined( DISTANCE ) || defined ( USE_SHADOWMAP ) || defined ( USE_TRANSMISSION ) || NUM_SPOT_LIGHT_COORDS > 0
	vec4 worldPosition = vec4( transformed, 1.0 );
	#ifdef USE_BATCHING
		worldPosition = batchingMatrix * worldPosition;
	#endif
	#ifdef USE_INSTANCING
		worldPosition = instanceMatrix * worldPosition;
	#endif
	worldPosition = modelMatrix * worldPosition;
#endif`;const z0=`varying vec2 vUv;
uniform mat3 uvTransform;
void main() {
	vUv = ( uvTransform * vec3( uv, 1 ) ).xy;
	gl_Position = vec4( position.xy, 1.0, 1.0 );
}`,H0=`uniform sampler2D t2D;
uniform float backgroundIntensity;
varying vec2 vUv;
void main() {
	vec4 texColor = texture2D( t2D, vUv );
	#ifdef DECODE_VIDEO_TEXTURE
		texColor = vec4( mix( pow( texColor.rgb * 0.9478672986 + vec3( 0.0521327014 ), vec3( 2.4 ) ), texColor.rgb * 0.0773993808, vec3( lessThanEqual( texColor.rgb, vec3( 0.04045 ) ) ) ), texColor.w );
	#endif
	texColor.rgb *= backgroundIntensity;
	gl_FragColor = texColor;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,V0=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
	gl_Position.z = gl_Position.w;
}`,G0=`#ifdef ENVMAP_TYPE_CUBE
	uniform samplerCube envMap;
#elif defined( ENVMAP_TYPE_CUBE_UV )
	uniform sampler2D envMap;
#endif
uniform float flipEnvMap;
uniform float backgroundBlurriness;
uniform float backgroundIntensity;
uniform mat3 backgroundRotation;
varying vec3 vWorldDirection;
#include <cube_uv_reflection_fragment>
void main() {
	#ifdef ENVMAP_TYPE_CUBE
		vec4 texColor = textureCube( envMap, backgroundRotation * vec3( flipEnvMap * vWorldDirection.x, vWorldDirection.yz ) );
	#elif defined( ENVMAP_TYPE_CUBE_UV )
		vec4 texColor = textureCubeUV( envMap, backgroundRotation * vWorldDirection, backgroundBlurriness );
	#else
		vec4 texColor = vec4( 0.0, 0.0, 0.0, 1.0 );
	#endif
	texColor.rgb *= backgroundIntensity;
	gl_FragColor = texColor;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,W0=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
	gl_Position.z = gl_Position.w;
}`,X0=`uniform samplerCube tCube;
uniform float tFlip;
uniform float opacity;
varying vec3 vWorldDirection;
void main() {
	vec4 texColor = textureCube( tCube, vec3( tFlip * vWorldDirection.x, vWorldDirection.yz ) );
	gl_FragColor = texColor;
	gl_FragColor.a *= opacity;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,j0=`#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
varying vec2 vHighPrecisionZW;
void main() {
	#include <uv_vertex>
	#include <batching_vertex>
	#include <skinbase_vertex>
	#include <morphinstance_vertex>
	#ifdef USE_DISPLACEMENTMAP
		#include <beginnormal_vertex>
		#include <morphnormal_vertex>
		#include <skinnormal_vertex>
	#endif
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vHighPrecisionZW = gl_Position.zw;
}`,$0=`#if DEPTH_PACKING == 3200
	uniform float opacity;
#endif
#include <common>
#include <packing>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
varying vec2 vHighPrecisionZW;
void main() {
	vec4 diffuseColor = vec4( 1.0 );
	#include <clipping_planes_fragment>
	#if DEPTH_PACKING == 3200
		diffuseColor.a = opacity;
	#endif
	#include <map_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <logdepthbuf_fragment>
	float fragCoordZ = 0.5 * vHighPrecisionZW[0] / vHighPrecisionZW[1] + 0.5;
	#if DEPTH_PACKING == 3200
		gl_FragColor = vec4( vec3( 1.0 - fragCoordZ ), opacity );
	#elif DEPTH_PACKING == 3201
		gl_FragColor = packDepthToRGBA( fragCoordZ );
	#elif DEPTH_PACKING == 3202
		gl_FragColor = vec4( packDepthToRGB( fragCoordZ ), 1.0 );
	#elif DEPTH_PACKING == 3203
		gl_FragColor = vec4( packDepthToRG( fragCoordZ ), 0.0, 1.0 );
	#endif
}`,Y0=`#define DISTANCE
varying vec3 vWorldPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <batching_vertex>
	#include <skinbase_vertex>
	#include <morphinstance_vertex>
	#ifdef USE_DISPLACEMENTMAP
		#include <beginnormal_vertex>
		#include <morphnormal_vertex>
		#include <skinnormal_vertex>
	#endif
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <worldpos_vertex>
	#include <clipping_planes_vertex>
	vWorldPosition = worldPosition.xyz;
}`,q0=`#define DISTANCE
uniform vec3 referencePosition;
uniform float nearDistance;
uniform float farDistance;
varying vec3 vWorldPosition;
#include <common>
#include <packing>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <clipping_planes_pars_fragment>
void main () {
	vec4 diffuseColor = vec4( 1.0 );
	#include <clipping_planes_fragment>
	#include <map_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	float dist = length( vWorldPosition - referencePosition );
	dist = ( dist - nearDistance ) / ( farDistance - nearDistance );
	dist = saturate( dist );
	gl_FragColor = packDepthToRGBA( dist );
}`,K0=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
}`,Z0=`uniform sampler2D tEquirect;
varying vec3 vWorldDirection;
#include <common>
void main() {
	vec3 direction = normalize( vWorldDirection );
	vec2 sampleUV = equirectUv( direction );
	gl_FragColor = texture2D( tEquirect, sampleUV );
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,J0=`uniform float scale;
attribute float lineDistance;
varying float vLineDistance;
#include <common>
#include <uv_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	vLineDistance = scale * lineDistance;
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <fog_vertex>
}`,Q0=`uniform vec3 diffuse;
uniform float opacity;
uniform float dashSize;
uniform float totalSize;
varying float vLineDistance;
#include <common>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <fog_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	if ( mod( vLineDistance, totalSize ) > dashSize ) {
		discard;
	}
	vec3 outgoingLight = vec3( 0.0 );
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	outgoingLight = diffuseColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
}`,tx=`#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <envmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#if defined ( USE_ENVMAP ) || defined ( USE_SKINNING )
		#include <beginnormal_vertex>
		#include <morphnormal_vertex>
		#include <skinbase_vertex>
		#include <skinnormal_vertex>
		#include <defaultnormal_vertex>
	#endif
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <worldpos_vertex>
	#include <envmap_vertex>
	#include <fog_vertex>
}`,ex=`uniform vec3 diffuse;
uniform float opacity;
#ifndef FLAT_SHADED
	varying vec3 vNormal;
#endif
#include <common>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_pars_fragment>
#include <fog_pars_fragment>
#include <specularmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <specularmap_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	#ifdef USE_LIGHTMAP
		vec4 lightMapTexel = texture2D( lightMap, vLightMapUv );
		reflectedLight.indirectDiffuse += lightMapTexel.rgb * lightMapIntensity * RECIPROCAL_PI;
	#else
		reflectedLight.indirectDiffuse += vec3( 1.0 );
	#endif
	#include <aomap_fragment>
	reflectedLight.indirectDiffuse *= diffuseColor.rgb;
	vec3 outgoingLight = reflectedLight.indirectDiffuse;
	#include <envmap_fragment>
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,nx=`#define LAMBERT
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <envmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <envmap_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,ix=`#define LAMBERT
uniform vec3 diffuse;
uniform vec3 emissive;
uniform float opacity;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_pars_fragment>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_lambert_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <specularmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <specularmap_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_lambert_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + totalEmissiveRadiance;
	#include <envmap_fragment>
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,sx=`#define MATCAP
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <color_pars_vertex>
#include <displacementmap_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <fog_vertex>
	vViewPosition = - mvPosition.xyz;
}`,rx=`#define MATCAP
uniform vec3 diffuse;
uniform float opacity;
uniform sampler2D matcap;
varying vec3 vViewPosition;
#include <common>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <fog_pars_fragment>
#include <normal_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	vec3 viewDir = normalize( vViewPosition );
	vec3 x = normalize( vec3( viewDir.z, 0.0, - viewDir.x ) );
	vec3 y = cross( viewDir, x );
	vec2 uv = vec2( dot( x, normal ), dot( y, normal ) ) * 0.495 + 0.5;
	#ifdef USE_MATCAP
		vec4 matcapColor = texture2D( matcap, uv );
	#else
		vec4 matcapColor = vec4( vec3( mix( 0.2, 0.8, uv.y ) ), 1.0 );
	#endif
	vec3 outgoingLight = diffuseColor.rgb * matcapColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,ox=`#define NORMAL
#if defined( FLAT_SHADED ) || defined( USE_BUMPMAP ) || defined( USE_NORMALMAP_TANGENTSPACE )
	varying vec3 vViewPosition;
#endif
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphinstance_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
#if defined( FLAT_SHADED ) || defined( USE_BUMPMAP ) || defined( USE_NORMALMAP_TANGENTSPACE )
	vViewPosition = - mvPosition.xyz;
#endif
}`,ax=`#define NORMAL
uniform float opacity;
#if defined( FLAT_SHADED ) || defined( USE_BUMPMAP ) || defined( USE_NORMALMAP_TANGENTSPACE )
	varying vec3 vViewPosition;
#endif
#include <packing>
#include <uv_pars_fragment>
#include <normal_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( 0.0, 0.0, 0.0, opacity );
	#include <clipping_planes_fragment>
	#include <logdepthbuf_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	gl_FragColor = vec4( packNormalToRGB( normal ), diffuseColor.a );
	#ifdef OPAQUE
		gl_FragColor.a = 1.0;
	#endif
}`,lx=`#define PHONG
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <envmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphinstance_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <envmap_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,cx=`#define PHONG
uniform vec3 diffuse;
uniform vec3 emissive;
uniform vec3 specular;
uniform float shininess;
uniform float opacity;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_pars_fragment>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_phong_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <specularmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <specularmap_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_phong_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + reflectedLight.directSpecular + reflectedLight.indirectSpecular + totalEmissiveRadiance;
	#include <envmap_fragment>
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,hx=`#define STANDARD
varying vec3 vViewPosition;
#ifdef USE_TRANSMISSION
	varying vec3 vWorldPosition;
#endif
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
#ifdef USE_TRANSMISSION
	vWorldPosition = worldPosition.xyz;
#endif
}`,ux=`#define STANDARD
#ifdef PHYSICAL
	#define IOR
	#define USE_SPECULAR
#endif
uniform vec3 diffuse;
uniform vec3 emissive;
uniform float roughness;
uniform float metalness;
uniform float opacity;
#ifdef IOR
	uniform float ior;
#endif
#ifdef USE_SPECULAR
	uniform float specularIntensity;
	uniform vec3 specularColor;
	#ifdef USE_SPECULAR_COLORMAP
		uniform sampler2D specularColorMap;
	#endif
	#ifdef USE_SPECULAR_INTENSITYMAP
		uniform sampler2D specularIntensityMap;
	#endif
#endif
#ifdef USE_CLEARCOAT
	uniform float clearcoat;
	uniform float clearcoatRoughness;
#endif
#ifdef USE_DISPERSION
	uniform float dispersion;
#endif
#ifdef USE_IRIDESCENCE
	uniform float iridescence;
	uniform float iridescenceIOR;
	uniform float iridescenceThicknessMinimum;
	uniform float iridescenceThicknessMaximum;
#endif
#ifdef USE_SHEEN
	uniform vec3 sheenColor;
	uniform float sheenRoughness;
	#ifdef USE_SHEEN_COLORMAP
		uniform sampler2D sheenColorMap;
	#endif
	#ifdef USE_SHEEN_ROUGHNESSMAP
		uniform sampler2D sheenRoughnessMap;
	#endif
#endif
#ifdef USE_ANISOTROPY
	uniform vec2 anisotropyVector;
	#ifdef USE_ANISOTROPYMAP
		uniform sampler2D anisotropyMap;
	#endif
#endif
varying vec3 vViewPosition;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <iridescence_fragment>
#include <cube_uv_reflection_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_physical_pars_fragment>
#include <fog_pars_fragment>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_physical_pars_fragment>
#include <transmission_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <clearcoat_pars_fragment>
#include <iridescence_pars_fragment>
#include <roughnessmap_pars_fragment>
#include <metalnessmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <roughnessmap_fragment>
	#include <metalnessmap_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <clearcoat_normal_fragment_begin>
	#include <clearcoat_normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_physical_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 totalDiffuse = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse;
	vec3 totalSpecular = reflectedLight.directSpecular + reflectedLight.indirectSpecular;
	#include <transmission_fragment>
	vec3 outgoingLight = totalDiffuse + totalSpecular + totalEmissiveRadiance;
	#ifdef USE_SHEEN
		float sheenEnergyComp = 1.0 - 0.157 * max3( material.sheenColor );
		outgoingLight = outgoingLight * sheenEnergyComp + sheenSpecularDirect + sheenSpecularIndirect;
	#endif
	#ifdef USE_CLEARCOAT
		float dotNVcc = saturate( dot( geometryClearcoatNormal, geometryViewDir ) );
		vec3 Fcc = F_Schlick( material.clearcoatF0, material.clearcoatF90, dotNVcc );
		outgoingLight = outgoingLight * ( 1.0 - material.clearcoat * Fcc ) + ( clearcoatSpecularDirect + clearcoatSpecularIndirect ) * material.clearcoat;
	#endif
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,dx=`#define TOON
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,fx=`#define TOON
uniform vec3 diffuse;
uniform vec3 emissive;
uniform float opacity;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <gradientmap_pars_fragment>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_toon_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_toon_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + totalEmissiveRadiance;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,px=`uniform float size;
uniform float scale;
#include <common>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
#ifdef USE_POINTS_UV
	varying vec2 vUv;
	uniform mat3 uvTransform;
#endif
void main() {
	#ifdef USE_POINTS_UV
		vUv = ( uvTransform * vec3( uv, 1 ) ).xy;
	#endif
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <project_vertex>
	gl_PointSize = size;
	#ifdef USE_SIZEATTENUATION
		bool isPerspective = isPerspectiveMatrix( projectionMatrix );
		if ( isPerspective ) gl_PointSize *= ( scale / - mvPosition.z );
	#endif
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <worldpos_vertex>
	#include <fog_vertex>
}`,mx=`uniform vec3 diffuse;
uniform float opacity;
#include <common>
#include <color_pars_fragment>
#include <map_particle_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <fog_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	vec3 outgoingLight = vec3( 0.0 );
	#include <logdepthbuf_fragment>
	#include <map_particle_fragment>
	#include <color_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	outgoingLight = diffuseColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
}`,gx=`#include <common>
#include <batching_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <shadowmap_pars_vertex>
void main() {
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphinstance_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <worldpos_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,_x=`uniform vec3 color;
uniform float opacity;
#include <common>
#include <packing>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <logdepthbuf_pars_fragment>
#include <shadowmap_pars_fragment>
#include <shadowmask_pars_fragment>
void main() {
	#include <logdepthbuf_fragment>
	gl_FragColor = vec4( color, opacity * ( 1.0 - getShadowMask() ) );
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
}`,xx=`uniform float rotation;
uniform vec2 center;
#include <common>
#include <uv_pars_vertex>
#include <fog_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	vec4 mvPosition = modelViewMatrix[ 3 ];
	vec2 scale = vec2( length( modelMatrix[ 0 ].xyz ), length( modelMatrix[ 1 ].xyz ) );
	#ifndef USE_SIZEATTENUATION
		bool isPerspective = isPerspectiveMatrix( projectionMatrix );
		if ( isPerspective ) scale *= - mvPosition.z;
	#endif
	vec2 alignedPosition = ( position.xy - ( center - vec2( 0.5 ) ) ) * scale;
	vec2 rotatedPosition;
	rotatedPosition.x = cos( rotation ) * alignedPosition.x - sin( rotation ) * alignedPosition.y;
	rotatedPosition.y = sin( rotation ) * alignedPosition.x + cos( rotation ) * alignedPosition.y;
	mvPosition.xy += rotatedPosition;
	gl_Position = projectionMatrix * mvPosition;
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <fog_vertex>
}`,vx=`uniform vec3 diffuse;
uniform float opacity;
#include <common>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <fog_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	vec3 outgoingLight = vec3( 0.0 );
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	outgoingLight = diffuseColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
}`,zt={alphahash_fragment:Hg,alphahash_pars_fragment:Vg,alphamap_fragment:Gg,alphamap_pars_fragment:Wg,alphatest_fragment:Xg,alphatest_pars_fragment:jg,aomap_fragment:$g,aomap_pars_fragment:Yg,batching_pars_vertex:qg,batching_vertex:Kg,begin_vertex:Zg,beginnormal_vertex:Jg,bsdfs:Qg,iridescence_fragment:t_,bumpmap_pars_fragment:e_,clipping_planes_fragment:n_,clipping_planes_pars_fragment:i_,clipping_planes_pars_vertex:s_,clipping_planes_vertex:r_,color_fragment:o_,color_pars_fragment:a_,color_pars_vertex:l_,color_vertex:c_,common:h_,cube_uv_reflection_fragment:u_,defaultnormal_vertex:d_,displacementmap_pars_vertex:f_,displacementmap_vertex:p_,emissivemap_fragment:m_,emissivemap_pars_fragment:g_,colorspace_fragment:__,colorspace_pars_fragment:x_,envmap_fragment:v_,envmap_common_pars_fragment:b_,envmap_pars_fragment:y_,envmap_pars_vertex:M_,envmap_physical_pars_fragment:I_,envmap_vertex:S_,fog_vertex:E_,fog_pars_vertex:T_,fog_fragment:w_,fog_pars_fragment:A_,gradientmap_pars_fragment:C_,lightmap_pars_fragment:R_,lights_lambert_fragment:P_,lights_lambert_pars_fragment:D_,lights_pars_begin:L_,lights_toon_fragment:O_,lights_toon_pars_fragment:U_,lights_phong_fragment:N_,lights_phong_pars_fragment:F_,lights_physical_fragment:k_,lights_physical_pars_fragment:B_,lights_fragment_begin:z_,lights_fragment_maps:H_,lights_fragment_end:V_,logdepthbuf_fragment:G_,logdepthbuf_pars_fragment:W_,logdepthbuf_pars_vertex:X_,logdepthbuf_vertex:j_,map_fragment:$_,map_pars_fragment:Y_,map_particle_fragment:q_,map_particle_pars_fragment:K_,metalnessmap_fragment:Z_,metalnessmap_pars_fragment:J_,morphinstance_vertex:Q_,morphcolor_vertex:t0,morphnormal_vertex:e0,morphtarget_pars_vertex:n0,morphtarget_vertex:i0,normal_fragment_begin:s0,normal_fragment_maps:r0,normal_pars_fragment:o0,normal_pars_vertex:a0,normal_vertex:l0,normalmap_pars_fragment:c0,clearcoat_normal_fragment_begin:h0,clearcoat_normal_fragment_maps:u0,clearcoat_pars_fragment:d0,iridescence_pars_fragment:f0,opaque_fragment:p0,packing:m0,premultiplied_alpha_fragment:g0,project_vertex:_0,dithering_fragment:x0,dithering_pars_fragment:v0,roughnessmap_fragment:b0,roughnessmap_pars_fragment:y0,shadowmap_pars_fragment:M0,shadowmap_pars_vertex:S0,shadowmap_vertex:E0,shadowmask_pars_fragment:T0,skinbase_vertex:w0,skinning_pars_vertex:A0,skinning_vertex:C0,skinnormal_vertex:R0,specularmap_fragment:P0,specularmap_pars_fragment:D0,tonemapping_fragment:L0,tonemapping_pars_fragment:I0,transmission_fragment:O0,transmission_pars_fragment:U0,uv_pars_fragment:N0,uv_pars_vertex:F0,uv_vertex:k0,worldpos_vertex:B0,background_vert:z0,background_frag:H0,backgroundCube_vert:V0,backgroundCube_frag:G0,cube_vert:W0,cube_frag:X0,depth_vert:j0,depth_frag:$0,distanceRGBA_vert:Y0,distanceRGBA_frag:q0,equirect_vert:K0,equirect_frag:Z0,linedashed_vert:J0,linedashed_frag:Q0,meshbasic_vert:tx,meshbasic_frag:ex,meshlambert_vert:nx,meshlambert_frag:ix,meshmatcap_vert:sx,meshmatcap_frag:rx,meshnormal_vert:ox,meshnormal_frag:ax,meshphong_vert:lx,meshphong_frag:cx,meshphysical_vert:hx,meshphysical_frag:ux,meshtoon_vert:dx,meshtoon_frag:fx,points_vert:px,points_frag:mx,shadow_vert:gx,shadow_frag:_x,sprite_vert:xx,sprite_frag:vx},rt={common:{diffuse:{value:new Ht(16777215)},opacity:{value:1},map:{value:null},mapTransform:{value:new Ft},alphaMap:{value:null},alphaMapTransform:{value:new Ft},alphaTest:{value:0}},specularmap:{specularMap:{value:null},specularMapTransform:{value:new Ft}},envmap:{envMap:{value:null},envMapRotation:{value:new Ft},flipEnvMap:{value:-1},reflectivity:{value:1},ior:{value:1.5},refractionRatio:{value:.98}},aomap:{aoMap:{value:null},aoMapIntensity:{value:1},aoMapTransform:{value:new Ft}},lightmap:{lightMap:{value:null},lightMapIntensity:{value:1},lightMapTransform:{value:new Ft}},bumpmap:{bumpMap:{value:null},bumpMapTransform:{value:new Ft},bumpScale:{value:1}},normalmap:{normalMap:{value:null},normalMapTransform:{value:new Ft},normalScale:{value:new Tt(1,1)}},displacementmap:{displacementMap:{value:null},displacementMapTransform:{value:new Ft},displacementScale:{value:1},displacementBias:{value:0}},emissivemap:{emissiveMap:{value:null},emissiveMapTransform:{value:new Ft}},metalnessmap:{metalnessMap:{value:null},metalnessMapTransform:{value:new Ft}},roughnessmap:{roughnessMap:{value:null},roughnessMapTransform:{value:new Ft}},gradientmap:{gradientMap:{value:null}},fog:{fogDensity:{value:25e-5},fogNear:{value:1},fogFar:{value:2e3},fogColor:{value:new Ht(16777215)}},lights:{ambientLightColor:{value:[]},lightProbe:{value:[]},directionalLights:{value:[],properties:{direction:{},color:{}}},directionalLightShadows:{value:[],properties:{shadowIntensity:1,shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{}}},directionalShadowMap:{value:[]},directionalShadowMatrix:{value:[]},spotLights:{value:[],properties:{color:{},position:{},direction:{},distance:{},coneCos:{},penumbraCos:{},decay:{}}},spotLightShadows:{value:[],properties:{shadowIntensity:1,shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{}}},spotLightMap:{value:[]},spotShadowMap:{value:[]},spotLightMatrix:{value:[]},pointLights:{value:[],properties:{color:{},position:{},decay:{},distance:{}}},pointLightShadows:{value:[],properties:{shadowIntensity:1,shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{},shadowCameraNear:{},shadowCameraFar:{}}},pointShadowMap:{value:[]},pointShadowMatrix:{value:[]},hemisphereLights:{value:[],properties:{direction:{},skyColor:{},groundColor:{}}},rectAreaLights:{value:[],properties:{color:{},position:{},width:{},height:{}}},ltc_1:{value:null},ltc_2:{value:null}},points:{diffuse:{value:new Ht(16777215)},opacity:{value:1},size:{value:1},scale:{value:1},map:{value:null},alphaMap:{value:null},alphaMapTransform:{value:new Ft},alphaTest:{value:0},uvTransform:{value:new Ft}},sprite:{diffuse:{value:new Ht(16777215)},opacity:{value:1},center:{value:new Tt(.5,.5)},rotation:{value:0},map:{value:null},mapTransform:{value:new Ft},alphaMap:{value:null},alphaMapTransform:{value:new Ft},alphaTest:{value:0}}},En={basic:{uniforms:Ve([rt.common,rt.specularmap,rt.envmap,rt.aomap,rt.lightmap,rt.fog]),vertexShader:zt.meshbasic_vert,fragmentShader:zt.meshbasic_frag},lambert:{uniforms:Ve([rt.common,rt.specularmap,rt.envmap,rt.aomap,rt.lightmap,rt.emissivemap,rt.bumpmap,rt.normalmap,rt.displacementmap,rt.fog,rt.lights,{emissive:{value:new Ht(0)}}]),vertexShader:zt.meshlambert_vert,fragmentShader:zt.meshlambert_frag},phong:{uniforms:Ve([rt.common,rt.specularmap,rt.envmap,rt.aomap,rt.lightmap,rt.emissivemap,rt.bumpmap,rt.normalmap,rt.displacementmap,rt.fog,rt.lights,{emissive:{value:new Ht(0)},specular:{value:new Ht(1118481)},shininess:{value:30}}]),vertexShader:zt.meshphong_vert,fragmentShader:zt.meshphong_frag},standard:{uniforms:Ve([rt.common,rt.envmap,rt.aomap,rt.lightmap,rt.emissivemap,rt.bumpmap,rt.normalmap,rt.displacementmap,rt.roughnessmap,rt.metalnessmap,rt.fog,rt.lights,{emissive:{value:new Ht(0)},roughness:{value:1},metalness:{value:0},envMapIntensity:{value:1}}]),vertexShader:zt.meshphysical_vert,fragmentShader:zt.meshphysical_frag},toon:{uniforms:Ve([rt.common,rt.aomap,rt.lightmap,rt.emissivemap,rt.bumpmap,rt.normalmap,rt.displacementmap,rt.gradientmap,rt.fog,rt.lights,{emissive:{value:new Ht(0)}}]),vertexShader:zt.meshtoon_vert,fragmentShader:zt.meshtoon_frag},matcap:{uniforms:Ve([rt.common,rt.bumpmap,rt.normalmap,rt.displacementmap,rt.fog,{matcap:{value:null}}]),vertexShader:zt.meshmatcap_vert,fragmentShader:zt.meshmatcap_frag},points:{uniforms:Ve([rt.points,rt.fog]),vertexShader:zt.points_vert,fragmentShader:zt.points_frag},dashed:{uniforms:Ve([rt.common,rt.fog,{scale:{value:1},dashSize:{value:1},totalSize:{value:2}}]),vertexShader:zt.linedashed_vert,fragmentShader:zt.linedashed_frag},depth:{uniforms:Ve([rt.common,rt.displacementmap]),vertexShader:zt.depth_vert,fragmentShader:zt.depth_frag},normal:{uniforms:Ve([rt.common,rt.bumpmap,rt.normalmap,rt.displacementmap,{opacity:{value:1}}]),vertexShader:zt.meshnormal_vert,fragmentShader:zt.meshnormal_frag},sprite:{uniforms:Ve([rt.sprite,rt.fog]),vertexShader:zt.sprite_vert,fragmentShader:zt.sprite_frag},background:{uniforms:{uvTransform:{value:new Ft},t2D:{value:null},backgroundIntensity:{value:1}},vertexShader:zt.background_vert,fragmentShader:zt.background_frag},backgroundCube:{uniforms:{envMap:{value:null},flipEnvMap:{value:-1},backgroundBlurriness:{value:0},backgroundIntensity:{value:1},backgroundRotation:{value:new Ft}},vertexShader:zt.backgroundCube_vert,fragmentShader:zt.backgroundCube_frag},cube:{uniforms:{tCube:{value:null},tFlip:{value:-1},opacity:{value:1}},vertexShader:zt.cube_vert,fragmentShader:zt.cube_frag},equirect:{uniforms:{tEquirect:{value:null}},vertexShader:zt.equirect_vert,fragmentShader:zt.equirect_frag},distanceRGBA:{uniforms:Ve([rt.common,rt.displacementmap,{referencePosition:{value:new L},nearDistance:{value:1},farDistance:{value:1e3}}]),vertexShader:zt.distanceRGBA_vert,fragmentShader:zt.distanceRGBA_frag},shadow:{uniforms:Ve([rt.lights,rt.fog,{color:{value:new Ht(0)},opacity:{value:1}}]),vertexShader:zt.shadow_vert,fragmentShader:zt.shadow_frag}};En.physical={uniforms:Ve([En.standard.uniforms,{clearcoat:{value:0},clearcoatMap:{value:null},clearcoatMapTransform:{value:new Ft},clearcoatNormalMap:{value:null},clearcoatNormalMapTransform:{value:new Ft},clearcoatNormalScale:{value:new Tt(1,1)},clearcoatRoughness:{value:0},clearcoatRoughnessMap:{value:null},clearcoatRoughnessMapTransform:{value:new Ft},dispersion:{value:0},iridescence:{value:0},iridescenceMap:{value:null},iridescenceMapTransform:{value:new Ft},iridescenceIOR:{value:1.3},iridescenceThicknessMinimum:{value:100},iridescenceThicknessMaximum:{value:400},iridescenceThicknessMap:{value:null},iridescenceThicknessMapTransform:{value:new Ft},sheen:{value:0},sheenColor:{value:new Ht(0)},sheenColorMap:{value:null},sheenColorMapTransform:{value:new Ft},sheenRoughness:{value:1},sheenRoughnessMap:{value:null},sheenRoughnessMapTransform:{value:new Ft},transmission:{value:0},transmissionMap:{value:null},transmissionMapTransform:{value:new Ft},transmissionSamplerSize:{value:new Tt},transmissionSamplerMap:{value:null},thickness:{value:0},thicknessMap:{value:null},thicknessMapTransform:{value:new Ft},attenuationDistance:{value:0},attenuationColor:{value:new Ht(0)},specularColor:{value:new Ht(1,1,1)},specularColorMap:{value:null},specularColorMapTransform:{value:new Ft},specularIntensity:{value:1},specularIntensityMap:{value:null},specularIntensityMapTransform:{value:new Ft},anisotropyVector:{value:new Tt},anisotropyMap:{value:null},anisotropyMapTransform:{value:new Ft}}]),vertexShader:zt.meshphysical_vert,fragmentShader:zt.meshphysical_frag};const oo={r:0,b:0,g:0},Ci=new Pn,bx=new oe;function yx(n,t,e,i,s,r,o){const a=new Ht(0);let l=r===!0?0:1,c,h,u=null,d=0,f=null;function g(v){let y=v.isScene===!0?v.background:null;return y&&y.isTexture&&(y=(v.backgroundBlurriness>0?e:t).get(y)),y}function _(v){let y=!1;const x=g(v);x===null?p(a,l):x&&x.isColor&&(p(x,1),y=!0);const A=n.xr.getEnvironmentBlendMode();A==="additive"?i.buffers.color.setClear(0,0,0,1,o):A==="alpha-blend"&&i.buffers.color.setClear(0,0,0,0,o),(n.autoClear||y)&&(i.buffers.depth.setTest(!0),i.buffers.depth.setMask(!0),i.buffers.color.setMask(!0),n.clear(n.autoClearColor,n.autoClearDepth,n.autoClearStencil))}function m(v,y){const x=g(y);x&&(x.isCubeTexture||x.mapping===ua)?(h===void 0&&(h=new xe(new An(1,1,1),new xi({name:"BackgroundCubeMaterial",uniforms:Ns(En.backgroundCube.uniforms),vertexShader:En.backgroundCube.vertexShader,fragmentShader:En.backgroundCube.fragmentShader,side:qe,depthTest:!1,depthWrite:!1,fog:!1})),h.geometry.deleteAttribute("normal"),h.geometry.deleteAttribute("uv"),h.onBeforeRender=function(A,w,T){this.matrixWorld.copyPosition(T.matrixWorld)},Object.defineProperty(h.material,"envMap",{get:function(){return this.uniforms.envMap.value}}),s.update(h)),Ci.copy(y.backgroundRotation),Ci.x*=-1,Ci.y*=-1,Ci.z*=-1,x.isCubeTexture&&x.isRenderTargetTexture===!1&&(Ci.y*=-1,Ci.z*=-1),h.material.uniforms.envMap.value=x,h.material.uniforms.flipEnvMap.value=x.isCubeTexture&&x.isRenderTargetTexture===!1?-1:1,h.material.uniforms.backgroundBlurriness.value=y.backgroundBlurriness,h.material.uniforms.backgroundIntensity.value=y.backgroundIntensity,h.material.uniforms.backgroundRotation.value.setFromMatrix4(bx.makeRotationFromEuler(Ci)),h.material.toneMapped=Kt.getTransfer(x.colorSpace)!==ie,(u!==x||d!==x.version||f!==n.toneMapping)&&(h.material.needsUpdate=!0,u=x,d=x.version,f=n.toneMapping),h.layers.enableAll(),v.unshift(h,h.geometry,h.material,0,0,null)):x&&x.isTexture&&(c===void 0&&(c=new xe(new Ur(2,2),new xi({name:"BackgroundMaterial",uniforms:Ns(En.background.uniforms),vertexShader:En.background.vertexShader,fragmentShader:En.background.fragmentShader,side:_i,depthTest:!1,depthWrite:!1,fog:!1})),c.geometry.deleteAttribute("normal"),Object.defineProperty(c.material,"map",{get:function(){return this.uniforms.t2D.value}}),s.update(c)),c.material.uniforms.t2D.value=x,c.material.uniforms.backgroundIntensity.value=y.backgroundIntensity,c.material.toneMapped=Kt.getTransfer(x.colorSpace)!==ie,x.matrixAutoUpdate===!0&&x.updateMatrix(),c.material.uniforms.uvTransform.value.copy(x.matrix),(u!==x||d!==x.version||f!==n.toneMapping)&&(c.material.needsUpdate=!0,u=x,d=x.version,f=n.toneMapping),c.layers.enableAll(),v.unshift(c,c.geometry,c.material,0,0,null))}function p(v,y){v.getRGB(oo,kf(n)),i.buffers.color.setClear(oo.r,oo.g,oo.b,y,o)}return{getClearColor:function(){return a},setClearColor:function(v,y=1){a.set(v),l=y,p(a,l)},getClearAlpha:function(){return l},setClearAlpha:function(v){l=v,p(a,l)},render:_,addToRenderList:m}}function Mx(n,t){const e=n.getParameter(n.MAX_VERTEX_ATTRIBS),i={},s=d(null);let r=s,o=!1;function a(b,P,N,F,W){let G=!1;const H=u(F,N,P);r!==H&&(r=H,c(r.object)),G=f(b,F,N,W),G&&g(b,F,N,W),W!==null&&t.update(W,n.ELEMENT_ARRAY_BUFFER),(G||o)&&(o=!1,x(b,P,N,F),W!==null&&n.bindBuffer(n.ELEMENT_ARRAY_BUFFER,t.get(W).buffer))}function l(){return n.createVertexArray()}function c(b){return n.bindVertexArray(b)}function h(b){return n.deleteVertexArray(b)}function u(b,P,N){const F=N.wireframe===!0;let W=i[b.id];W===void 0&&(W={},i[b.id]=W);let G=W[P.id];G===void 0&&(G={},W[P.id]=G);let H=G[F];return H===void 0&&(H=d(l()),G[F]=H),H}function d(b){const P=[],N=[],F=[];for(let W=0;W<e;W++)P[W]=0,N[W]=0,F[W]=0;return{geometry:null,program:null,wireframe:!1,newAttributes:P,enabledAttributes:N,attributeDivisors:F,object:b,attributes:{},index:null}}function f(b,P,N,F){const W=r.attributes,G=P.attributes;let H=0;const $=N.getAttributes();for(const V in $)if($[V].location>=0){const at=W[V];let _t=G[V];if(_t===void 0&&(V==="instanceMatrix"&&b.instanceMatrix&&(_t=b.instanceMatrix),V==="instanceColor"&&b.instanceColor&&(_t=b.instanceColor)),at===void 0||at.attribute!==_t||_t&&at.data!==_t.data)return!0;H++}return r.attributesNum!==H||r.index!==F}function g(b,P,N,F){const W={},G=P.attributes;let H=0;const $=N.getAttributes();for(const V in $)if($[V].location>=0){let at=G[V];at===void 0&&(V==="instanceMatrix"&&b.instanceMatrix&&(at=b.instanceMatrix),V==="instanceColor"&&b.instanceColor&&(at=b.instanceColor));const _t={};_t.attribute=at,at&&at.data&&(_t.data=at.data),W[V]=_t,H++}r.attributes=W,r.attributesNum=H,r.index=F}function _(){const b=r.newAttributes;for(let P=0,N=b.length;P<N;P++)b[P]=0}function m(b){p(b,0)}function p(b,P){const N=r.newAttributes,F=r.enabledAttributes,W=r.attributeDivisors;N[b]=1,F[b]===0&&(n.enableVertexAttribArray(b),F[b]=1),W[b]!==P&&(n.vertexAttribDivisor(b,P),W[b]=P)}function v(){const b=r.newAttributes,P=r.enabledAttributes;for(let N=0,F=P.length;N<F;N++)P[N]!==b[N]&&(n.disableVertexAttribArray(N),P[N]=0)}function y(b,P,N,F,W,G,H){H===!0?n.vertexAttribIPointer(b,P,N,W,G):n.vertexAttribPointer(b,P,N,F,W,G)}function x(b,P,N,F){_();const W=F.attributes,G=N.getAttributes(),H=P.defaultAttributeValues;for(const $ in G){const V=G[$];if(V.location>=0){let et=W[$];if(et===void 0&&($==="instanceMatrix"&&b.instanceMatrix&&(et=b.instanceMatrix),$==="instanceColor"&&b.instanceColor&&(et=b.instanceColor)),et!==void 0){const at=et.normalized,_t=et.itemSize,Dt=t.get(et);if(Dt===void 0)continue;const $t=Dt.buffer,j=Dt.type,tt=Dt.bytesPerElement,ft=j===n.INT||j===n.UNSIGNED_INT||et.gpuType===Ic;if(et.isInterleavedBufferAttribute){const lt=et.data,Rt=lt.stride,It=et.offset;if(lt.isInstancedInterleavedBuffer){for(let Vt=0;Vt<V.locationSize;Vt++)p(V.location+Vt,lt.meshPerAttribute);b.isInstancedMesh!==!0&&F._maxInstanceCount===void 0&&(F._maxInstanceCount=lt.meshPerAttribute*lt.count)}else for(let Vt=0;Vt<V.locationSize;Vt++)m(V.location+Vt);n.bindBuffer(n.ARRAY_BUFFER,$t);for(let Vt=0;Vt<V.locationSize;Vt++)y(V.location+Vt,_t/V.locationSize,j,at,Rt*tt,(It+_t/V.locationSize*Vt)*tt,ft)}else{if(et.isInstancedBufferAttribute){for(let lt=0;lt<V.locationSize;lt++)p(V.location+lt,et.meshPerAttribute);b.isInstancedMesh!==!0&&F._maxInstanceCount===void 0&&(F._maxInstanceCount=et.meshPerAttribute*et.count)}else for(let lt=0;lt<V.locationSize;lt++)m(V.location+lt);n.bindBuffer(n.ARRAY_BUFFER,$t);for(let lt=0;lt<V.locationSize;lt++)y(V.location+lt,_t/V.locationSize,j,at,_t*tt,_t/V.locationSize*lt*tt,ft)}}else if(H!==void 0){const at=H[$];if(at!==void 0)switch(at.length){case 2:n.vertexAttrib2fv(V.location,at);break;case 3:n.vertexAttrib3fv(V.location,at);break;case 4:n.vertexAttrib4fv(V.location,at);break;default:n.vertexAttrib1fv(V.location,at)}}}}v()}function A(){R();for(const b in i){const P=i[b];for(const N in P){const F=P[N];for(const W in F)h(F[W].object),delete F[W];delete P[N]}delete i[b]}}function w(b){if(i[b.id]===void 0)return;const P=i[b.id];for(const N in P){const F=P[N];for(const W in F)h(F[W].object),delete F[W];delete P[N]}delete i[b.id]}function T(b){for(const P in i){const N=i[P];if(N[b.id]===void 0)continue;const F=N[b.id];for(const W in F)h(F[W].object),delete F[W];delete N[b.id]}}function R(){S(),o=!0,r!==s&&(r=s,c(r.object))}function S(){s.geometry=null,s.program=null,s.wireframe=!1}return{setup:a,reset:R,resetDefaultState:S,dispose:A,releaseStatesOfGeometry:w,releaseStatesOfProgram:T,initAttributes:_,enableAttribute:m,disableUnusedAttributes:v}}function Sx(n,t,e){let i;function s(c){i=c}function r(c,h){n.drawArrays(i,c,h),e.update(h,i,1)}function o(c,h,u){u!==0&&(n.drawArraysInstanced(i,c,h,u),e.update(h,i,u))}function a(c,h,u){if(u===0)return;t.get("WEBGL_multi_draw").multiDrawArraysWEBGL(i,c,0,h,0,u);let f=0;for(let g=0;g<u;g++)f+=h[g];e.update(f,i,1)}function l(c,h,u,d){if(u===0)return;const f=t.get("WEBGL_multi_draw");if(f===null)for(let g=0;g<c.length;g++)o(c[g],h[g],d[g]);else{f.multiDrawArraysInstancedWEBGL(i,c,0,h,0,d,0,u);let g=0;for(let _=0;_<u;_++)g+=h[_]*d[_];e.update(g,i,1)}}this.setMode=s,this.render=r,this.renderInstances=o,this.renderMultiDraw=a,this.renderMultiDrawInstances=l}function Ex(n,t,e,i){let s;function r(){if(s!==void 0)return s;if(t.has("EXT_texture_filter_anisotropic")===!0){const T=t.get("EXT_texture_filter_anisotropic");s=n.getParameter(T.MAX_TEXTURE_MAX_ANISOTROPY_EXT)}else s=0;return s}function o(T){return!(T!==mn&&i.convert(T)!==n.getParameter(n.IMPLEMENTATION_COLOR_READ_FORMAT))}function a(T){const R=T===Dr&&(t.has("EXT_color_buffer_half_float")||t.has("EXT_color_buffer_float"));return!(T!==Jn&&i.convert(T)!==n.getParameter(n.IMPLEMENTATION_COLOR_READ_TYPE)&&T!==jn&&!R)}function l(T){if(T==="highp"){if(n.getShaderPrecisionFormat(n.VERTEX_SHADER,n.HIGH_FLOAT).precision>0&&n.getShaderPrecisionFormat(n.FRAGMENT_SHADER,n.HIGH_FLOAT).precision>0)return"highp";T="mediump"}return T==="mediump"&&n.getShaderPrecisionFormat(n.VERTEX_SHADER,n.MEDIUM_FLOAT).precision>0&&n.getShaderPrecisionFormat(n.FRAGMENT_SHADER,n.MEDIUM_FLOAT).precision>0?"mediump":"lowp"}let c=e.precision!==void 0?e.precision:"highp";const h=l(c);h!==c&&(console.warn("THREE.WebGLRenderer:",c,"not supported, using",h,"instead."),c=h);const u=e.logarithmicDepthBuffer===!0,d=e.reverseDepthBuffer===!0&&t.has("EXT_clip_control"),f=n.getParameter(n.MAX_TEXTURE_IMAGE_UNITS),g=n.getParameter(n.MAX_VERTEX_TEXTURE_IMAGE_UNITS),_=n.getParameter(n.MAX_TEXTURE_SIZE),m=n.getParameter(n.MAX_CUBE_MAP_TEXTURE_SIZE),p=n.getParameter(n.MAX_VERTEX_ATTRIBS),v=n.getParameter(n.MAX_VERTEX_UNIFORM_VECTORS),y=n.getParameter(n.MAX_VARYING_VECTORS),x=n.getParameter(n.MAX_FRAGMENT_UNIFORM_VECTORS),A=g>0,w=n.getParameter(n.MAX_SAMPLES);return{isWebGL2:!0,getMaxAnisotropy:r,getMaxPrecision:l,textureFormatReadable:o,textureTypeReadable:a,precision:c,logarithmicDepthBuffer:u,reverseDepthBuffer:d,maxTextures:f,maxVertexTextures:g,maxTextureSize:_,maxCubemapSize:m,maxAttributes:p,maxVertexUniforms:v,maxVaryings:y,maxFragmentUniforms:x,vertexTextures:A,maxSamples:w}}function Tx(n){const t=this;let e=null,i=0,s=!1,r=!1;const o=new ai,a=new Ft,l={value:null,needsUpdate:!1};this.uniform=l,this.numPlanes=0,this.numIntersection=0,this.init=function(u,d){const f=u.length!==0||d||i!==0||s;return s=d,i=u.length,f},this.beginShadows=function(){r=!0,h(null)},this.endShadows=function(){r=!1},this.setGlobalState=function(u,d){e=h(u,d,0)},this.setState=function(u,d,f){const g=u.clippingPlanes,_=u.clipIntersection,m=u.clipShadows,p=n.get(u);if(!s||g===null||g.length===0||r&&!m)r?h(null):c();else{const v=r?0:i,y=v*4;let x=p.clippingState||null;l.value=x,x=h(g,d,y,f);for(let A=0;A!==y;++A)x[A]=e[A];p.clippingState=x,this.numIntersection=_?this.numPlanes:0,this.numPlanes+=v}};function c(){l.value!==e&&(l.value=e,l.needsUpdate=i>0),t.numPlanes=i,t.numIntersection=0}function h(u,d,f,g){const _=u!==null?u.length:0;let m=null;if(_!==0){if(m=l.value,g!==!0||m===null){const p=f+_*4,v=d.matrixWorldInverse;a.getNormalMatrix(v),(m===null||m.length<p)&&(m=new Float32Array(p));for(let y=0,x=f;y!==_;++y,x+=4)o.copy(u[y]).applyMatrix4(v,a),o.normal.toArray(m,x),m[x+3]=o.constant}l.value=m,l.needsUpdate=!0}return t.numPlanes=_,t.numIntersection=0,m}}function wx(n){let t=new WeakMap;function e(o,a){return a===Dl?o.mapping=Ls:a===Ll&&(o.mapping=Is),o}function i(o){if(o&&o.isTexture){const a=o.mapping;if(a===Dl||a===Ll)if(t.has(o)){const l=t.get(o).texture;return e(l,o.mapping)}else{const l=o.image;if(l&&l.height>0){const c=new Fg(l.height);return c.fromEquirectangularTexture(n,o),t.set(o,c),o.addEventListener("dispose",s),e(c.texture,o.mapping)}else return null}}return o}function s(o){const a=o.target;a.removeEventListener("dispose",s);const l=t.get(a);l!==void 0&&(t.delete(a),l.dispose())}function r(){t=new WeakMap}return{get:i,dispose:r}}class Vf extends Bf{constructor(t=-1,e=1,i=1,s=-1,r=.1,o=2e3){super(),this.isOrthographicCamera=!0,this.type="OrthographicCamera",this.zoom=1,this.view=null,this.left=t,this.right=e,this.top=i,this.bottom=s,this.near=r,this.far=o,this.updateProjectionMatrix()}copy(t,e){return super.copy(t,e),this.left=t.left,this.right=t.right,this.top=t.top,this.bottom=t.bottom,this.near=t.near,this.far=t.far,this.zoom=t.zoom,this.view=t.view===null?null:Object.assign({},t.view),this}setViewOffset(t,e,i,s,r,o){this.view===null&&(this.view={enabled:!0,fullWidth:1,fullHeight:1,offsetX:0,offsetY:0,width:1,height:1}),this.view.enabled=!0,this.view.fullWidth=t,this.view.fullHeight=e,this.view.offsetX=i,this.view.offsetY=s,this.view.width=r,this.view.height=o,this.updateProjectionMatrix()}clearViewOffset(){this.view!==null&&(this.view.enabled=!1),this.updateProjectionMatrix()}updateProjectionMatrix(){const t=(this.right-this.left)/(2*this.zoom),e=(this.top-this.bottom)/(2*this.zoom),i=(this.right+this.left)/2,s=(this.top+this.bottom)/2;let r=i-t,o=i+t,a=s+e,l=s-e;if(this.view!==null&&this.view.enabled){const c=(this.right-this.left)/this.view.fullWidth/this.zoom,h=(this.top-this.bottom)/this.view.fullHeight/this.zoom;r+=c*this.view.offsetX,o=r+c*this.view.width,a-=h*this.view.offsetY,l=a-h*this.view.height}this.projectionMatrix.makeOrthographic(r,o,a,l,this.near,this.far,this.coordinateSystem),this.projectionMatrixInverse.copy(this.projectionMatrix).invert()}toJSON(t){const e=super.toJSON(t);return e.object.zoom=this.zoom,e.object.left=this.left,e.object.right=this.right,e.object.top=this.top,e.object.bottom=this.bottom,e.object.near=this.near,e.object.far=this.far,this.view!==null&&(e.object.view=Object.assign({},this.view)),e}}const Ss=4,Yh=[.125,.215,.35,.446,.526,.582],ki=20,Ka=new Vf,qh=new Ht;let Za=null,Ja=0,Qa=0,tl=!1;const Oi=(1+Math.sqrt(5))/2,fs=1/Oi,Kh=[new L(-Oi,fs,0),new L(Oi,fs,0),new L(-fs,0,Oi),new L(fs,0,Oi),new L(0,Oi,-fs),new L(0,Oi,fs),new L(-1,1,-1),new L(1,1,-1),new L(-1,1,1),new L(1,1,1)];class Zh{constructor(t){this._renderer=t,this._pingPongRenderTarget=null,this._lodMax=0,this._cubeSize=0,this._lodPlanes=[],this._sizeLods=[],this._sigmas=[],this._blurMaterial=null,this._cubemapMaterial=null,this._equirectMaterial=null,this._compileMaterial(this._blurMaterial)}fromScene(t,e=0,i=.1,s=100){Za=this._renderer.getRenderTarget(),Ja=this._renderer.getActiveCubeFace(),Qa=this._renderer.getActiveMipmapLevel(),tl=this._renderer.xr.enabled,this._renderer.xr.enabled=!1,this._setSize(256);const r=this._allocateTargets();return r.depthBuffer=!0,this._sceneToCubeUV(t,i,s,r),e>0&&this._blur(r,0,0,e),this._applyPMREM(r),this._cleanup(r),r}fromEquirectangular(t,e=null){return this._fromTexture(t,e)}fromCubemap(t,e=null){return this._fromTexture(t,e)}compileCubemapShader(){this._cubemapMaterial===null&&(this._cubemapMaterial=tu(),this._compileMaterial(this._cubemapMaterial))}compileEquirectangularShader(){this._equirectMaterial===null&&(this._equirectMaterial=Qh(),this._compileMaterial(this._equirectMaterial))}dispose(){this._dispose(),this._cubemapMaterial!==null&&this._cubemapMaterial.dispose(),this._equirectMaterial!==null&&this._equirectMaterial.dispose()}_setSize(t){this._lodMax=Math.floor(Math.log2(t)),this._cubeSize=Math.pow(2,this._lodMax)}_dispose(){this._blurMaterial!==null&&this._blurMaterial.dispose(),this._pingPongRenderTarget!==null&&this._pingPongRenderTarget.dispose();for(let t=0;t<this._lodPlanes.length;t++)this._lodPlanes[t].dispose()}_cleanup(t){this._renderer.setRenderTarget(Za,Ja,Qa),this._renderer.xr.enabled=tl,t.scissorTest=!1,ao(t,0,0,t.width,t.height)}_fromTexture(t,e){t.mapping===Ls||t.mapping===Is?this._setSize(t.image.length===0?16:t.image[0].width||t.image[0].image.width):this._setSize(t.image.width/4),Za=this._renderer.getRenderTarget(),Ja=this._renderer.getActiveCubeFace(),Qa=this._renderer.getActiveMipmapLevel(),tl=this._renderer.xr.enabled,this._renderer.xr.enabled=!1;const i=e||this._allocateTargets();return this._textureToCubeUV(t,i),this._applyPMREM(i),this._cleanup(i),i}_allocateTargets(){const t=3*Math.max(this._cubeSize,112),e=4*this._cubeSize,i={magFilter:Cn,minFilter:Cn,generateMipmaps:!1,type:Dr,format:mn,colorSpace:Hs,depthBuffer:!1},s=Jh(t,e,i);if(this._pingPongRenderTarget===null||this._pingPongRenderTarget.width!==t||this._pingPongRenderTarget.height!==e){this._pingPongRenderTarget!==null&&this._dispose(),this._pingPongRenderTarget=Jh(t,e,i);const{_lodMax:r}=this;({sizeLods:this._sizeLods,lodPlanes:this._lodPlanes,sigmas:this._sigmas}=Ax(r)),this._blurMaterial=Cx(r,t,e)}return s}_compileMaterial(t){const e=new xe(this._lodPlanes[0],t);this._renderer.compile(e,Ka)}_sceneToCubeUV(t,e,i,s){const a=new ln(90,1,e,i),l=[1,-1,1,1,1,1],c=[1,1,1,-1,-1,-1],h=this._renderer,u=h.autoClear,d=h.toneMapping;h.getClearColor(qh),h.toneMapping=pi,h.autoClear=!1;const f=new Ms({name:"PMREM.Background",side:qe,depthWrite:!1,depthTest:!1}),g=new xe(new An,f);let _=!1;const m=t.background;m?m.isColor&&(f.color.copy(m),t.background=null,_=!0):(f.color.copy(qh),_=!0);for(let p=0;p<6;p++){const v=p%3;v===0?(a.up.set(0,l[p],0),a.lookAt(c[p],0,0)):v===1?(a.up.set(0,0,l[p]),a.lookAt(0,c[p],0)):(a.up.set(0,l[p],0),a.lookAt(0,0,c[p]));const y=this._cubeSize;ao(s,v*y,p>2?y:0,y,y),h.setRenderTarget(s),_&&h.render(g,a),h.render(t,a)}g.geometry.dispose(),g.material.dispose(),h.toneMapping=d,h.autoClear=u,t.background=m}_textureToCubeUV(t,e){const i=this._renderer,s=t.mapping===Ls||t.mapping===Is;s?(this._cubemapMaterial===null&&(this._cubemapMaterial=tu()),this._cubemapMaterial.uniforms.flipEnvMap.value=t.isRenderTargetTexture===!1?-1:1):this._equirectMaterial===null&&(this._equirectMaterial=Qh());const r=s?this._cubemapMaterial:this._equirectMaterial,o=new xe(this._lodPlanes[0],r),a=r.uniforms;a.envMap.value=t;const l=this._cubeSize;ao(e,0,0,3*l,2*l),i.setRenderTarget(e),i.render(o,Ka)}_applyPMREM(t){const e=this._renderer,i=e.autoClear;e.autoClear=!1;const s=this._lodPlanes.length;for(let r=1;r<s;r++){const o=Math.sqrt(this._sigmas[r]*this._sigmas[r]-this._sigmas[r-1]*this._sigmas[r-1]),a=Kh[(s-r-1)%Kh.length];this._blur(t,r-1,r,o,a)}e.autoClear=i}_blur(t,e,i,s,r){const o=this._pingPongRenderTarget;this._halfBlur(t,o,e,i,s,"latitudinal",r),this._halfBlur(o,t,i,i,s,"longitudinal",r)}_halfBlur(t,e,i,s,r,o,a){const l=this._renderer,c=this._blurMaterial;o!=="latitudinal"&&o!=="longitudinal"&&console.error("blur direction must be either latitudinal or longitudinal!");const h=3,u=new xe(this._lodPlanes[s],c),d=c.uniforms,f=this._sizeLods[i]-1,g=isFinite(r)?Math.PI/(2*f):2*Math.PI/(2*ki-1),_=r/g,m=isFinite(r)?1+Math.floor(h*_):ki;m>ki&&console.warn(`sigmaRadians, ${r}, is too large and will clip, as it requested ${m} samples when the maximum is set to ${ki}`);const p=[];let v=0;for(let T=0;T<ki;++T){const R=T/_,S=Math.exp(-R*R/2);p.push(S),T===0?v+=S:T<m&&(v+=2*S)}for(let T=0;T<p.length;T++)p[T]=p[T]/v;d.envMap.value=t.texture,d.samples.value=m,d.weights.value=p,d.latitudinal.value=o==="latitudinal",a&&(d.poleAxis.value=a);const{_lodMax:y}=this;d.dTheta.value=g,d.mipInt.value=y-i;const x=this._sizeLods[s],A=3*x*(s>y-Ss?s-y+Ss:0),w=4*(this._cubeSize-x);ao(e,A,w,3*x,2*x),l.setRenderTarget(e),l.render(u,Ka)}}function Ax(n){const t=[],e=[],i=[];let s=n;const r=n-Ss+1+Yh.length;for(let o=0;o<r;o++){const a=Math.pow(2,s);e.push(a);let l=1/a;o>n-Ss?l=Yh[o-n+Ss-1]:o===0&&(l=0),i.push(l);const c=1/(a-2),h=-c,u=1+c,d=[h,h,u,h,u,u,h,h,u,u,h,u],f=6,g=6,_=3,m=2,p=1,v=new Float32Array(_*g*f),y=new Float32Array(m*g*f),x=new Float32Array(p*g*f);for(let w=0;w<f;w++){const T=w%3*2/3-1,R=w>2?0:-1,S=[T,R,0,T+2/3,R,0,T+2/3,R+1,0,T,R,0,T+2/3,R+1,0,T,R+1,0];v.set(S,_*g*w),y.set(d,m*g*w);const b=[w,w,w,w,w,w];x.set(b,p*g*w)}const A=new Te;A.setAttribute("position",new en(v,_)),A.setAttribute("uv",new en(y,m)),A.setAttribute("faceIndex",new en(x,p)),t.push(A),s>Ss&&s--}return{lodPlanes:t,sizeLods:e,sigmas:i}}function Jh(n,t,e){const i=new ji(n,t,e);return i.texture.mapping=ua,i.texture.name="PMREM.cubeUv",i.scissorTest=!0,i}function ao(n,t,e,i,s){n.viewport.set(t,e,i,s),n.scissor.set(t,e,i,s)}function Cx(n,t,e){const i=new Float32Array(ki),s=new L(0,1,0);return new xi({name:"SphericalGaussianBlur",defines:{n:ki,CUBEUV_TEXEL_WIDTH:1/t,CUBEUV_TEXEL_HEIGHT:1/e,CUBEUV_MAX_MIP:`${n}.0`},uniforms:{envMap:{value:null},samples:{value:1},weights:{value:i},latitudinal:{value:!1},dTheta:{value:0},mipInt:{value:0},poleAxis:{value:s}},vertexShader:Hc(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			varying vec3 vOutputDirection;

			uniform sampler2D envMap;
			uniform int samples;
			uniform float weights[ n ];
			uniform bool latitudinal;
			uniform float dTheta;
			uniform float mipInt;
			uniform vec3 poleAxis;

			#define ENVMAP_TYPE_CUBE_UV
			#include <cube_uv_reflection_fragment>

			vec3 getSample( float theta, vec3 axis ) {

				float cosTheta = cos( theta );
				// Rodrigues' axis-angle rotation
				vec3 sampleDirection = vOutputDirection * cosTheta
					+ cross( axis, vOutputDirection ) * sin( theta )
					+ axis * dot( axis, vOutputDirection ) * ( 1.0 - cosTheta );

				return bilinearCubeUV( envMap, sampleDirection, mipInt );

			}

			void main() {

				vec3 axis = latitudinal ? poleAxis : cross( poleAxis, vOutputDirection );

				if ( all( equal( axis, vec3( 0.0 ) ) ) ) {

					axis = vec3( vOutputDirection.z, 0.0, - vOutputDirection.x );

				}

				axis = normalize( axis );

				gl_FragColor = vec4( 0.0, 0.0, 0.0, 1.0 );
				gl_FragColor.rgb += weights[ 0 ] * getSample( 0.0, axis );

				for ( int i = 1; i < n; i++ ) {

					if ( i >= samples ) {

						break;

					}

					float theta = dTheta * float( i );
					gl_FragColor.rgb += weights[ i ] * getSample( -1.0 * theta, axis );
					gl_FragColor.rgb += weights[ i ] * getSample( theta, axis );

				}

			}
		`,blending:fi,depthTest:!1,depthWrite:!1})}function Qh(){return new xi({name:"EquirectangularToCubeUV",uniforms:{envMap:{value:null}},vertexShader:Hc(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			varying vec3 vOutputDirection;

			uniform sampler2D envMap;

			#include <common>

			void main() {

				vec3 outputDirection = normalize( vOutputDirection );
				vec2 uv = equirectUv( outputDirection );

				gl_FragColor = vec4( texture2D ( envMap, uv ).rgb, 1.0 );

			}
		`,blending:fi,depthTest:!1,depthWrite:!1})}function tu(){return new xi({name:"CubemapToCubeUV",uniforms:{envMap:{value:null},flipEnvMap:{value:-1}},vertexShader:Hc(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			uniform float flipEnvMap;

			varying vec3 vOutputDirection;

			uniform samplerCube envMap;

			void main() {

				gl_FragColor = textureCube( envMap, vec3( flipEnvMap * vOutputDirection.x, vOutputDirection.yz ) );

			}
		`,blending:fi,depthTest:!1,depthWrite:!1})}function Hc(){return`

		precision mediump float;
		precision mediump int;

		attribute float faceIndex;

		varying vec3 vOutputDirection;

		// RH coordinate system; PMREM face-indexing convention
		vec3 getDirection( vec2 uv, float face ) {

			uv = 2.0 * uv - 1.0;

			vec3 direction = vec3( uv, 1.0 );

			if ( face == 0.0 ) {

				direction = direction.zyx; // ( 1, v, u ) pos x

			} else if ( face == 1.0 ) {

				direction = direction.xzy;
				direction.xz *= -1.0; // ( -u, 1, -v ) pos y

			} else if ( face == 2.0 ) {

				direction.x *= -1.0; // ( -u, v, 1 ) pos z

			} else if ( face == 3.0 ) {

				direction = direction.zyx;
				direction.xz *= -1.0; // ( -1, v, -u ) neg x

			} else if ( face == 4.0 ) {

				direction = direction.xzy;
				direction.xy *= -1.0; // ( -u, -1, v ) neg y

			} else if ( face == 5.0 ) {

				direction.z *= -1.0; // ( u, v, -1 ) neg z

			}

			return direction;

		}

		void main() {

			vOutputDirection = getDirection( uv, faceIndex );
			gl_Position = vec4( position, 1.0 );

		}
	`}function Rx(n){let t=new WeakMap,e=null;function i(a){if(a&&a.isTexture){const l=a.mapping,c=l===Dl||l===Ll,h=l===Ls||l===Is;if(c||h){let u=t.get(a);const d=u!==void 0?u.texture.pmremVersion:0;if(a.isRenderTargetTexture&&a.pmremVersion!==d)return e===null&&(e=new Zh(n)),u=c?e.fromEquirectangular(a,u):e.fromCubemap(a,u),u.texture.pmremVersion=a.pmremVersion,t.set(a,u),u.texture;if(u!==void 0)return u.texture;{const f=a.image;return c&&f&&f.height>0||h&&f&&s(f)?(e===null&&(e=new Zh(n)),u=c?e.fromEquirectangular(a):e.fromCubemap(a),u.texture.pmremVersion=a.pmremVersion,t.set(a,u),a.addEventListener("dispose",r),u.texture):null}}}return a}function s(a){let l=0;const c=6;for(let h=0;h<c;h++)a[h]!==void 0&&l++;return l===c}function r(a){const l=a.target;l.removeEventListener("dispose",r);const c=t.get(l);c!==void 0&&(t.delete(l),c.dispose())}function o(){t=new WeakMap,e!==null&&(e.dispose(),e=null)}return{get:i,dispose:o}}function Px(n){const t={};function e(i){if(t[i]!==void 0)return t[i];let s;switch(i){case"WEBGL_depth_texture":s=n.getExtension("WEBGL_depth_texture")||n.getExtension("MOZ_WEBGL_depth_texture")||n.getExtension("WEBKIT_WEBGL_depth_texture");break;case"EXT_texture_filter_anisotropic":s=n.getExtension("EXT_texture_filter_anisotropic")||n.getExtension("MOZ_EXT_texture_filter_anisotropic")||n.getExtension("WEBKIT_EXT_texture_filter_anisotropic");break;case"WEBGL_compressed_texture_s3tc":s=n.getExtension("WEBGL_compressed_texture_s3tc")||n.getExtension("MOZ_WEBGL_compressed_texture_s3tc")||n.getExtension("WEBKIT_WEBGL_compressed_texture_s3tc");break;case"WEBGL_compressed_texture_pvrtc":s=n.getExtension("WEBGL_compressed_texture_pvrtc")||n.getExtension("WEBKIT_WEBGL_compressed_texture_pvrtc");break;default:s=n.getExtension(i)}return t[i]=s,s}return{has:function(i){return e(i)!==null},init:function(){e("EXT_color_buffer_float"),e("WEBGL_clip_cull_distance"),e("OES_texture_float_linear"),e("EXT_color_buffer_half_float"),e("WEBGL_multisampled_render_to_texture"),e("WEBGL_render_shared_exponent")},get:function(i){const s=e(i);return s===null&&or("THREE.WebGLRenderer: "+i+" extension not supported."),s}}}function Dx(n,t,e,i){const s={},r=new WeakMap;function o(u){const d=u.target;d.index!==null&&t.remove(d.index);for(const g in d.attributes)t.remove(d.attributes[g]);for(const g in d.morphAttributes){const _=d.morphAttributes[g];for(let m=0,p=_.length;m<p;m++)t.remove(_[m])}d.removeEventListener("dispose",o),delete s[d.id];const f=r.get(d);f&&(t.remove(f),r.delete(d)),i.releaseStatesOfGeometry(d),d.isInstancedBufferGeometry===!0&&delete d._maxInstanceCount,e.memory.geometries--}function a(u,d){return s[d.id]===!0||(d.addEventListener("dispose",o),s[d.id]=!0,e.memory.geometries++),d}function l(u){const d=u.attributes;for(const g in d)t.update(d[g],n.ARRAY_BUFFER);const f=u.morphAttributes;for(const g in f){const _=f[g];for(let m=0,p=_.length;m<p;m++)t.update(_[m],n.ARRAY_BUFFER)}}function c(u){const d=[],f=u.index,g=u.attributes.position;let _=0;if(f!==null){const v=f.array;_=f.version;for(let y=0,x=v.length;y<x;y+=3){const A=v[y+0],w=v[y+1],T=v[y+2];d.push(A,w,w,T,T,A)}}else if(g!==void 0){const v=g.array;_=g.version;for(let y=0,x=v.length/3-1;y<x;y+=3){const A=y+0,w=y+1,T=y+2;d.push(A,w,w,T,T,A)}}else return;const m=new(Lf(d)?Ff:Nf)(d,1);m.version=_;const p=r.get(u);p&&t.remove(p),r.set(u,m)}function h(u){const d=r.get(u);if(d){const f=u.index;f!==null&&d.version<f.version&&c(u)}else c(u);return r.get(u)}return{get:a,update:l,getWireframeAttribute:h}}function Lx(n,t,e){let i;function s(d){i=d}let r,o;function a(d){r=d.type,o=d.bytesPerElement}function l(d,f){n.drawElements(i,f,r,d*o),e.update(f,i,1)}function c(d,f,g){g!==0&&(n.drawElementsInstanced(i,f,r,d*o,g),e.update(f,i,g))}function h(d,f,g){if(g===0)return;t.get("WEBGL_multi_draw").multiDrawElementsWEBGL(i,f,0,r,d,0,g);let m=0;for(let p=0;p<g;p++)m+=f[p];e.update(m,i,1)}function u(d,f,g,_){if(g===0)return;const m=t.get("WEBGL_multi_draw");if(m===null)for(let p=0;p<d.length;p++)c(d[p]/o,f[p],_[p]);else{m.multiDrawElementsInstancedWEBGL(i,f,0,r,d,0,_,0,g);let p=0;for(let v=0;v<g;v++)p+=f[v]*_[v];e.update(p,i,1)}}this.setMode=s,this.setIndex=a,this.render=l,this.renderInstances=c,this.renderMultiDraw=h,this.renderMultiDrawInstances=u}function Ix(n){const t={geometries:0,textures:0},e={frame:0,calls:0,triangles:0,points:0,lines:0};function i(r,o,a){switch(e.calls++,o){case n.TRIANGLES:e.triangles+=a*(r/3);break;case n.LINES:e.lines+=a*(r/2);break;case n.LINE_STRIP:e.lines+=a*(r-1);break;case n.LINE_LOOP:e.lines+=a*r;break;case n.POINTS:e.points+=a*r;break;default:console.error("THREE.WebGLInfo: Unknown draw mode:",o);break}}function s(){e.calls=0,e.triangles=0,e.points=0,e.lines=0}return{memory:t,render:e,programs:null,autoReset:!0,reset:s,update:i}}function Ox(n,t,e){const i=new WeakMap,s=new be;function r(o,a,l){const c=o.morphTargetInfluences,h=a.morphAttributes.position||a.morphAttributes.normal||a.morphAttributes.color,u=h!==void 0?h.length:0;let d=i.get(a);if(d===void 0||d.count!==u){let b=function(){R.dispose(),i.delete(a),a.removeEventListener("dispose",b)};var f=b;d!==void 0&&d.texture.dispose();const g=a.morphAttributes.position!==void 0,_=a.morphAttributes.normal!==void 0,m=a.morphAttributes.color!==void 0,p=a.morphAttributes.position||[],v=a.morphAttributes.normal||[],y=a.morphAttributes.color||[];let x=0;g===!0&&(x=1),_===!0&&(x=2),m===!0&&(x=3);let A=a.attributes.position.count*x,w=1;A>t.maxTextureSize&&(w=Math.ceil(A/t.maxTextureSize),A=t.maxTextureSize);const T=new Float32Array(A*w*4*u),R=new Of(T,A,w,u);R.type=jn,R.needsUpdate=!0;const S=x*4;for(let P=0;P<u;P++){const N=p[P],F=v[P],W=y[P],G=A*w*4*P;for(let H=0;H<N.count;H++){const $=H*S;g===!0&&(s.fromBufferAttribute(N,H),T[G+$+0]=s.x,T[G+$+1]=s.y,T[G+$+2]=s.z,T[G+$+3]=0),_===!0&&(s.fromBufferAttribute(F,H),T[G+$+4]=s.x,T[G+$+5]=s.y,T[G+$+6]=s.z,T[G+$+7]=0),m===!0&&(s.fromBufferAttribute(W,H),T[G+$+8]=s.x,T[G+$+9]=s.y,T[G+$+10]=s.z,T[G+$+11]=W.itemSize===4?s.w:1)}}d={count:u,texture:R,size:new Tt(A,w)},i.set(a,d),a.addEventListener("dispose",b)}if(o.isInstancedMesh===!0&&o.morphTexture!==null)l.getUniforms().setValue(n,"morphTexture",o.morphTexture,e);else{let g=0;for(let m=0;m<c.length;m++)g+=c[m];const _=a.morphTargetsRelative?1:1-g;l.getUniforms().setValue(n,"morphTargetBaseInfluence",_),l.getUniforms().setValue(n,"morphTargetInfluences",c)}l.getUniforms().setValue(n,"morphTargetsTexture",d.texture,e),l.getUniforms().setValue(n,"morphTargetsTextureSize",d.size)}return{update:r}}function Ux(n,t,e,i){let s=new WeakMap;function r(l){const c=i.render.frame,h=l.geometry,u=t.get(l,h);if(s.get(u)!==c&&(t.update(u),s.set(u,c)),l.isInstancedMesh&&(l.hasEventListener("dispose",a)===!1&&l.addEventListener("dispose",a),s.get(l)!==c&&(e.update(l.instanceMatrix,n.ARRAY_BUFFER),l.instanceColor!==null&&e.update(l.instanceColor,n.ARRAY_BUFFER),s.set(l,c))),l.isSkinnedMesh){const d=l.skeleton;s.get(d)!==c&&(d.update(),s.set(d,c))}return u}function o(){s=new WeakMap}function a(l){const c=l.target;c.removeEventListener("dispose",a),e.remove(c.instanceMatrix),c.instanceColor!==null&&e.remove(c.instanceColor)}return{update:r,dispose:o}}class Gf extends We{constructor(t,e,i,s,r,o,a,l,c,h=As){if(h!==As&&h!==Us)throw new Error("DepthTexture format must be either THREE.DepthFormat or THREE.DepthStencilFormat");i===void 0&&h===As&&(i=Xi),i===void 0&&h===Us&&(i=Os),super(null,s,r,o,a,l,h,i,c),this.isDepthTexture=!0,this.image={width:t,height:e},this.magFilter=a!==void 0?a:xn,this.minFilter=l!==void 0?l:xn,this.flipY=!1,this.generateMipmaps=!1,this.compareFunction=null}copy(t){return super.copy(t),this.compareFunction=t.compareFunction,this}toJSON(t){const e=super.toJSON(t);return this.compareFunction!==null&&(e.compareFunction=this.compareFunction),e}}const Wf=new We,eu=new Gf(1,1),Xf=new Of,jf=new yg,$f=new zf,nu=[],iu=[],su=new Float32Array(16),ru=new Float32Array(9),ou=new Float32Array(4);function Vs(n,t,e){const i=n[0];if(i<=0||i>0)return n;const s=t*e;let r=nu[s];if(r===void 0&&(r=new Float32Array(s),nu[s]=r),t!==0){i.toArray(r,0);for(let o=1,a=0;o!==t;++o)a+=e,n[o].toArray(r,a)}return r}function De(n,t){if(n.length!==t.length)return!1;for(let e=0,i=n.length;e<i;e++)if(n[e]!==t[e])return!1;return!0}function Le(n,t){for(let e=0,i=t.length;e<i;e++)n[e]=t[e]}function fa(n,t){let e=iu[t];e===void 0&&(e=new Int32Array(t),iu[t]=e);for(let i=0;i!==t;++i)e[i]=n.allocateTextureUnit();return e}function Nx(n,t){const e=this.cache;e[0]!==t&&(n.uniform1f(this.addr,t),e[0]=t)}function Fx(n,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y)&&(n.uniform2f(this.addr,t.x,t.y),e[0]=t.x,e[1]=t.y);else{if(De(e,t))return;n.uniform2fv(this.addr,t),Le(e,t)}}function kx(n,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z)&&(n.uniform3f(this.addr,t.x,t.y,t.z),e[0]=t.x,e[1]=t.y,e[2]=t.z);else if(t.r!==void 0)(e[0]!==t.r||e[1]!==t.g||e[2]!==t.b)&&(n.uniform3f(this.addr,t.r,t.g,t.b),e[0]=t.r,e[1]=t.g,e[2]=t.b);else{if(De(e,t))return;n.uniform3fv(this.addr,t),Le(e,t)}}function Bx(n,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z||e[3]!==t.w)&&(n.uniform4f(this.addr,t.x,t.y,t.z,t.w),e[0]=t.x,e[1]=t.y,e[2]=t.z,e[3]=t.w);else{if(De(e,t))return;n.uniform4fv(this.addr,t),Le(e,t)}}function zx(n,t){const e=this.cache,i=t.elements;if(i===void 0){if(De(e,t))return;n.uniformMatrix2fv(this.addr,!1,t),Le(e,t)}else{if(De(e,i))return;ou.set(i),n.uniformMatrix2fv(this.addr,!1,ou),Le(e,i)}}function Hx(n,t){const e=this.cache,i=t.elements;if(i===void 0){if(De(e,t))return;n.uniformMatrix3fv(this.addr,!1,t),Le(e,t)}else{if(De(e,i))return;ru.set(i),n.uniformMatrix3fv(this.addr,!1,ru),Le(e,i)}}function Vx(n,t){const e=this.cache,i=t.elements;if(i===void 0){if(De(e,t))return;n.uniformMatrix4fv(this.addr,!1,t),Le(e,t)}else{if(De(e,i))return;su.set(i),n.uniformMatrix4fv(this.addr,!1,su),Le(e,i)}}function Gx(n,t){const e=this.cache;e[0]!==t&&(n.uniform1i(this.addr,t),e[0]=t)}function Wx(n,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y)&&(n.uniform2i(this.addr,t.x,t.y),e[0]=t.x,e[1]=t.y);else{if(De(e,t))return;n.uniform2iv(this.addr,t),Le(e,t)}}function Xx(n,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z)&&(n.uniform3i(this.addr,t.x,t.y,t.z),e[0]=t.x,e[1]=t.y,e[2]=t.z);else{if(De(e,t))return;n.uniform3iv(this.addr,t),Le(e,t)}}function jx(n,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z||e[3]!==t.w)&&(n.uniform4i(this.addr,t.x,t.y,t.z,t.w),e[0]=t.x,e[1]=t.y,e[2]=t.z,e[3]=t.w);else{if(De(e,t))return;n.uniform4iv(this.addr,t),Le(e,t)}}function $x(n,t){const e=this.cache;e[0]!==t&&(n.uniform1ui(this.addr,t),e[0]=t)}function Yx(n,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y)&&(n.uniform2ui(this.addr,t.x,t.y),e[0]=t.x,e[1]=t.y);else{if(De(e,t))return;n.uniform2uiv(this.addr,t),Le(e,t)}}function qx(n,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z)&&(n.uniform3ui(this.addr,t.x,t.y,t.z),e[0]=t.x,e[1]=t.y,e[2]=t.z);else{if(De(e,t))return;n.uniform3uiv(this.addr,t),Le(e,t)}}function Kx(n,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z||e[3]!==t.w)&&(n.uniform4ui(this.addr,t.x,t.y,t.z,t.w),e[0]=t.x,e[1]=t.y,e[2]=t.z,e[3]=t.w);else{if(De(e,t))return;n.uniform4uiv(this.addr,t),Le(e,t)}}function Zx(n,t,e){const i=this.cache,s=e.allocateTextureUnit();i[0]!==s&&(n.uniform1i(this.addr,s),i[0]=s);let r;this.type===n.SAMPLER_2D_SHADOW?(eu.compareFunction=Df,r=eu):r=Wf,e.setTexture2D(t||r,s)}function Jx(n,t,e){const i=this.cache,s=e.allocateTextureUnit();i[0]!==s&&(n.uniform1i(this.addr,s),i[0]=s),e.setTexture3D(t||jf,s)}function Qx(n,t,e){const i=this.cache,s=e.allocateTextureUnit();i[0]!==s&&(n.uniform1i(this.addr,s),i[0]=s),e.setTextureCube(t||$f,s)}function tv(n,t,e){const i=this.cache,s=e.allocateTextureUnit();i[0]!==s&&(n.uniform1i(this.addr,s),i[0]=s),e.setTexture2DArray(t||Xf,s)}function ev(n){switch(n){case 5126:return Nx;case 35664:return Fx;case 35665:return kx;case 35666:return Bx;case 35674:return zx;case 35675:return Hx;case 35676:return Vx;case 5124:case 35670:return Gx;case 35667:case 35671:return Wx;case 35668:case 35672:return Xx;case 35669:case 35673:return jx;case 5125:return $x;case 36294:return Yx;case 36295:return qx;case 36296:return Kx;case 35678:case 36198:case 36298:case 36306:case 35682:return Zx;case 35679:case 36299:case 36307:return Jx;case 35680:case 36300:case 36308:case 36293:return Qx;case 36289:case 36303:case 36311:case 36292:return tv}}function nv(n,t){n.uniform1fv(this.addr,t)}function iv(n,t){const e=Vs(t,this.size,2);n.uniform2fv(this.addr,e)}function sv(n,t){const e=Vs(t,this.size,3);n.uniform3fv(this.addr,e)}function rv(n,t){const e=Vs(t,this.size,4);n.uniform4fv(this.addr,e)}function ov(n,t){const e=Vs(t,this.size,4);n.uniformMatrix2fv(this.addr,!1,e)}function av(n,t){const e=Vs(t,this.size,9);n.uniformMatrix3fv(this.addr,!1,e)}function lv(n,t){const e=Vs(t,this.size,16);n.uniformMatrix4fv(this.addr,!1,e)}function cv(n,t){n.uniform1iv(this.addr,t)}function hv(n,t){n.uniform2iv(this.addr,t)}function uv(n,t){n.uniform3iv(this.addr,t)}function dv(n,t){n.uniform4iv(this.addr,t)}function fv(n,t){n.uniform1uiv(this.addr,t)}function pv(n,t){n.uniform2uiv(this.addr,t)}function mv(n,t){n.uniform3uiv(this.addr,t)}function gv(n,t){n.uniform4uiv(this.addr,t)}function _v(n,t,e){const i=this.cache,s=t.length,r=fa(e,s);De(i,r)||(n.uniform1iv(this.addr,r),Le(i,r));for(let o=0;o!==s;++o)e.setTexture2D(t[o]||Wf,r[o])}function xv(n,t,e){const i=this.cache,s=t.length,r=fa(e,s);De(i,r)||(n.uniform1iv(this.addr,r),Le(i,r));for(let o=0;o!==s;++o)e.setTexture3D(t[o]||jf,r[o])}function vv(n,t,e){const i=this.cache,s=t.length,r=fa(e,s);De(i,r)||(n.uniform1iv(this.addr,r),Le(i,r));for(let o=0;o!==s;++o)e.setTextureCube(t[o]||$f,r[o])}function bv(n,t,e){const i=this.cache,s=t.length,r=fa(e,s);De(i,r)||(n.uniform1iv(this.addr,r),Le(i,r));for(let o=0;o!==s;++o)e.setTexture2DArray(t[o]||Xf,r[o])}function yv(n){switch(n){case 5126:return nv;case 35664:return iv;case 35665:return sv;case 35666:return rv;case 35674:return ov;case 35675:return av;case 35676:return lv;case 5124:case 35670:return cv;case 35667:case 35671:return hv;case 35668:case 35672:return uv;case 35669:case 35673:return dv;case 5125:return fv;case 36294:return pv;case 36295:return mv;case 36296:return gv;case 35678:case 36198:case 36298:case 36306:case 35682:return _v;case 35679:case 36299:case 36307:return xv;case 35680:case 36300:case 36308:case 36293:return vv;case 36289:case 36303:case 36311:case 36292:return bv}}class Mv{constructor(t,e,i){this.id=t,this.addr=i,this.cache=[],this.type=e.type,this.setValue=ev(e.type)}}class Sv{constructor(t,e,i){this.id=t,this.addr=i,this.cache=[],this.type=e.type,this.size=e.size,this.setValue=yv(e.type)}}class Ev{constructor(t){this.id=t,this.seq=[],this.map={}}setValue(t,e,i){const s=this.seq;for(let r=0,o=s.length;r!==o;++r){const a=s[r];a.setValue(t,e[a.id],i)}}}const el=/(\w+)(\])?(\[|\.)?/g;function au(n,t){n.seq.push(t),n.map[t.id]=t}function Tv(n,t,e){const i=n.name,s=i.length;for(el.lastIndex=0;;){const r=el.exec(i),o=el.lastIndex;let a=r[1];const l=r[2]==="]",c=r[3];if(l&&(a=a|0),c===void 0||c==="["&&o+2===s){au(e,c===void 0?new Mv(a,n,t):new Sv(a,n,t));break}else{let u=e.map[a];u===void 0&&(u=new Ev(a),au(e,u)),e=u}}}class zo{constructor(t,e){this.seq=[],this.map={};const i=t.getProgramParameter(e,t.ACTIVE_UNIFORMS);for(let s=0;s<i;++s){const r=t.getActiveUniform(e,s),o=t.getUniformLocation(e,r.name);Tv(r,o,this)}}setValue(t,e,i,s){const r=this.map[e];r!==void 0&&r.setValue(t,i,s)}setOptional(t,e,i){const s=e[i];s!==void 0&&this.setValue(t,i,s)}static upload(t,e,i,s){for(let r=0,o=e.length;r!==o;++r){const a=e[r],l=i[a.id];l.needsUpdate!==!1&&a.setValue(t,l.value,s)}}static seqWithValue(t,e){const i=[];for(let s=0,r=t.length;s!==r;++s){const o=t[s];o.id in e&&i.push(o)}return i}}function lu(n,t,e){const i=n.createShader(t);return n.shaderSource(i,e),n.compileShader(i),i}const wv=37297;let Av=0;function Cv(n,t){const e=n.split(`
`),i=[],s=Math.max(t-6,0),r=Math.min(t+6,e.length);for(let o=s;o<r;o++){const a=o+1;i.push(`${a===t?">":" "} ${a}: ${e[o]}`)}return i.join(`
`)}const cu=new Ft;function Rv(n){Kt._getMatrix(cu,Kt.workingColorSpace,n);const t=`mat3( ${cu.elements.map(e=>e.toFixed(4))} )`;switch(Kt.getTransfer(n)){case da:return[t,"LinearTransferOETF"];case ie:return[t,"sRGBTransferOETF"];default:return console.warn("THREE.WebGLProgram: Unsupported color space: ",n),[t,"LinearTransferOETF"]}}function hu(n,t,e){const i=n.getShaderParameter(t,n.COMPILE_STATUS),s=n.getShaderInfoLog(t).trim();if(i&&s==="")return"";const r=/ERROR: 0:(\d+)/.exec(s);if(r){const o=parseInt(r[1]);return e.toUpperCase()+`

`+s+`

`+Cv(n.getShaderSource(t),o)}else return s}function Pv(n,t){const e=Rv(t);return[`vec4 ${n}( vec4 value ) {`,`	return ${e[1]}( vec4( value.rgb * ${e[0]}, value.a ) );`,"}"].join(`
`)}function Dv(n,t){let e;switch(t){case $m:e="Linear";break;case Ym:e="Reinhard";break;case qm:e="Cineon";break;case Km:e="ACESFilmic";break;case Jm:e="AgX";break;case Qm:e="Neutral";break;case Zm:e="Custom";break;default:console.warn("THREE.WebGLProgram: Unsupported toneMapping:",t),e="Linear"}return"vec3 "+n+"( vec3 color ) { return "+e+"ToneMapping( color ); }"}const lo=new L;function Lv(){Kt.getLuminanceCoefficients(lo);const n=lo.x.toFixed(4),t=lo.y.toFixed(4),e=lo.z.toFixed(4);return["float luminance( const in vec3 rgb ) {",`	const vec3 weights = vec3( ${n}, ${t}, ${e} );`,"	return dot( weights, rgb );","}"].join(`
`)}function Iv(n){return[n.extensionClipCullDistance?"#extension GL_ANGLE_clip_cull_distance : require":"",n.extensionMultiDraw?"#extension GL_ANGLE_multi_draw : require":""].filter(ar).join(`
`)}function Ov(n){const t=[];for(const e in n){const i=n[e];i!==!1&&t.push("#define "+e+" "+i)}return t.join(`
`)}function Uv(n,t){const e={},i=n.getProgramParameter(t,n.ACTIVE_ATTRIBUTES);for(let s=0;s<i;s++){const r=n.getActiveAttrib(t,s),o=r.name;let a=1;r.type===n.FLOAT_MAT2&&(a=2),r.type===n.FLOAT_MAT3&&(a=3),r.type===n.FLOAT_MAT4&&(a=4),e[o]={type:r.type,location:n.getAttribLocation(t,o),locationSize:a}}return e}function ar(n){return n!==""}function uu(n,t){const e=t.numSpotLightShadows+t.numSpotLightMaps-t.numSpotLightShadowsWithMaps;return n.replace(/NUM_DIR_LIGHTS/g,t.numDirLights).replace(/NUM_SPOT_LIGHTS/g,t.numSpotLights).replace(/NUM_SPOT_LIGHT_MAPS/g,t.numSpotLightMaps).replace(/NUM_SPOT_LIGHT_COORDS/g,e).replace(/NUM_RECT_AREA_LIGHTS/g,t.numRectAreaLights).replace(/NUM_POINT_LIGHTS/g,t.numPointLights).replace(/NUM_HEMI_LIGHTS/g,t.numHemiLights).replace(/NUM_DIR_LIGHT_SHADOWS/g,t.numDirLightShadows).replace(/NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS/g,t.numSpotLightShadowsWithMaps).replace(/NUM_SPOT_LIGHT_SHADOWS/g,t.numSpotLightShadows).replace(/NUM_POINT_LIGHT_SHADOWS/g,t.numPointLightShadows)}function du(n,t){return n.replace(/NUM_CLIPPING_PLANES/g,t.numClippingPlanes).replace(/UNION_CLIPPING_PLANES/g,t.numClippingPlanes-t.numClipIntersection)}const Nv=/^[ \t]*#include +<([\w\d./]+)>/gm;function cc(n){return n.replace(Nv,kv)}const Fv=new Map;function kv(n,t){let e=zt[t];if(e===void 0){const i=Fv.get(t);if(i!==void 0)e=zt[i],console.warn('THREE.WebGLRenderer: Shader chunk "%s" has been deprecated. Use "%s" instead.',t,i);else throw new Error("Can not resolve #include <"+t+">")}return cc(e)}const Bv=/#pragma unroll_loop_start\s+for\s*\(\s*int\s+i\s*=\s*(\d+)\s*;\s*i\s*<\s*(\d+)\s*;\s*i\s*\+\+\s*\)\s*{([\s\S]+?)}\s+#pragma unroll_loop_end/g;function fu(n){return n.replace(Bv,zv)}function zv(n,t,e,i){let s="";for(let r=parseInt(t);r<parseInt(e);r++)s+=i.replace(/\[\s*i\s*\]/g,"[ "+r+" ]").replace(/UNROLLED_LOOP_INDEX/g,r);return s}function pu(n){let t=`precision ${n.precision} float;
	precision ${n.precision} int;
	precision ${n.precision} sampler2D;
	precision ${n.precision} samplerCube;
	precision ${n.precision} sampler3D;
	precision ${n.precision} sampler2DArray;
	precision ${n.precision} sampler2DShadow;
	precision ${n.precision} samplerCubeShadow;
	precision ${n.precision} sampler2DArrayShadow;
	precision ${n.precision} isampler2D;
	precision ${n.precision} isampler3D;
	precision ${n.precision} isamplerCube;
	precision ${n.precision} isampler2DArray;
	precision ${n.precision} usampler2D;
	precision ${n.precision} usampler3D;
	precision ${n.precision} usamplerCube;
	precision ${n.precision} usampler2DArray;
	`;return n.precision==="highp"?t+=`
#define HIGH_PRECISION`:n.precision==="mediump"?t+=`
#define MEDIUM_PRECISION`:n.precision==="lowp"&&(t+=`
#define LOW_PRECISION`),t}function Hv(n){let t="SHADOWMAP_TYPE_BASIC";return n.shadowMapType===_f?t="SHADOWMAP_TYPE_PCF":n.shadowMapType===wm?t="SHADOWMAP_TYPE_PCF_SOFT":n.shadowMapType===zn&&(t="SHADOWMAP_TYPE_VSM"),t}function Vv(n){let t="ENVMAP_TYPE_CUBE";if(n.envMap)switch(n.envMapMode){case Ls:case Is:t="ENVMAP_TYPE_CUBE";break;case ua:t="ENVMAP_TYPE_CUBE_UV";break}return t}function Gv(n){let t="ENVMAP_MODE_REFLECTION";if(n.envMap)switch(n.envMapMode){case Is:t="ENVMAP_MODE_REFRACTION";break}return t}function Wv(n){let t="ENVMAP_BLENDING_NONE";if(n.envMap)switch(n.combine){case xf:t="ENVMAP_BLENDING_MULTIPLY";break;case Xm:t="ENVMAP_BLENDING_MIX";break;case jm:t="ENVMAP_BLENDING_ADD";break}return t}function Xv(n){const t=n.envMapCubeUVHeight;if(t===null)return null;const e=Math.log2(t)-2,i=1/t;return{texelWidth:1/(3*Math.max(Math.pow(2,e),7*16)),texelHeight:i,maxMip:e}}function jv(n,t,e,i){const s=n.getContext(),r=e.defines;let o=e.vertexShader,a=e.fragmentShader;const l=Hv(e),c=Vv(e),h=Gv(e),u=Wv(e),d=Xv(e),f=Iv(e),g=Ov(r),_=s.createProgram();let m,p,v=e.glslVersion?"#version "+e.glslVersion+`
`:"";e.isRawShaderMaterial?(m=["#define SHADER_TYPE "+e.shaderType,"#define SHADER_NAME "+e.shaderName,g].filter(ar).join(`
`),m.length>0&&(m+=`
`),p=["#define SHADER_TYPE "+e.shaderType,"#define SHADER_NAME "+e.shaderName,g].filter(ar).join(`
`),p.length>0&&(p+=`
`)):(m=[pu(e),"#define SHADER_TYPE "+e.shaderType,"#define SHADER_NAME "+e.shaderName,g,e.extensionClipCullDistance?"#define USE_CLIP_DISTANCE":"",e.batching?"#define USE_BATCHING":"",e.batchingColor?"#define USE_BATCHING_COLOR":"",e.instancing?"#define USE_INSTANCING":"",e.instancingColor?"#define USE_INSTANCING_COLOR":"",e.instancingMorph?"#define USE_INSTANCING_MORPH":"",e.useFog&&e.fog?"#define USE_FOG":"",e.useFog&&e.fogExp2?"#define FOG_EXP2":"",e.map?"#define USE_MAP":"",e.envMap?"#define USE_ENVMAP":"",e.envMap?"#define "+h:"",e.lightMap?"#define USE_LIGHTMAP":"",e.aoMap?"#define USE_AOMAP":"",e.bumpMap?"#define USE_BUMPMAP":"",e.normalMap?"#define USE_NORMALMAP":"",e.normalMapObjectSpace?"#define USE_NORMALMAP_OBJECTSPACE":"",e.normalMapTangentSpace?"#define USE_NORMALMAP_TANGENTSPACE":"",e.displacementMap?"#define USE_DISPLACEMENTMAP":"",e.emissiveMap?"#define USE_EMISSIVEMAP":"",e.anisotropy?"#define USE_ANISOTROPY":"",e.anisotropyMap?"#define USE_ANISOTROPYMAP":"",e.clearcoatMap?"#define USE_CLEARCOATMAP":"",e.clearcoatRoughnessMap?"#define USE_CLEARCOAT_ROUGHNESSMAP":"",e.clearcoatNormalMap?"#define USE_CLEARCOAT_NORMALMAP":"",e.iridescenceMap?"#define USE_IRIDESCENCEMAP":"",e.iridescenceThicknessMap?"#define USE_IRIDESCENCE_THICKNESSMAP":"",e.specularMap?"#define USE_SPECULARMAP":"",e.specularColorMap?"#define USE_SPECULAR_COLORMAP":"",e.specularIntensityMap?"#define USE_SPECULAR_INTENSITYMAP":"",e.roughnessMap?"#define USE_ROUGHNESSMAP":"",e.metalnessMap?"#define USE_METALNESSMAP":"",e.alphaMap?"#define USE_ALPHAMAP":"",e.alphaHash?"#define USE_ALPHAHASH":"",e.transmission?"#define USE_TRANSMISSION":"",e.transmissionMap?"#define USE_TRANSMISSIONMAP":"",e.thicknessMap?"#define USE_THICKNESSMAP":"",e.sheenColorMap?"#define USE_SHEEN_COLORMAP":"",e.sheenRoughnessMap?"#define USE_SHEEN_ROUGHNESSMAP":"",e.mapUv?"#define MAP_UV "+e.mapUv:"",e.alphaMapUv?"#define ALPHAMAP_UV "+e.alphaMapUv:"",e.lightMapUv?"#define LIGHTMAP_UV "+e.lightMapUv:"",e.aoMapUv?"#define AOMAP_UV "+e.aoMapUv:"",e.emissiveMapUv?"#define EMISSIVEMAP_UV "+e.emissiveMapUv:"",e.bumpMapUv?"#define BUMPMAP_UV "+e.bumpMapUv:"",e.normalMapUv?"#define NORMALMAP_UV "+e.normalMapUv:"",e.displacementMapUv?"#define DISPLACEMENTMAP_UV "+e.displacementMapUv:"",e.metalnessMapUv?"#define METALNESSMAP_UV "+e.metalnessMapUv:"",e.roughnessMapUv?"#define ROUGHNESSMAP_UV "+e.roughnessMapUv:"",e.anisotropyMapUv?"#define ANISOTROPYMAP_UV "+e.anisotropyMapUv:"",e.clearcoatMapUv?"#define CLEARCOATMAP_UV "+e.clearcoatMapUv:"",e.clearcoatNormalMapUv?"#define CLEARCOAT_NORMALMAP_UV "+e.clearcoatNormalMapUv:"",e.clearcoatRoughnessMapUv?"#define CLEARCOAT_ROUGHNESSMAP_UV "+e.clearcoatRoughnessMapUv:"",e.iridescenceMapUv?"#define IRIDESCENCEMAP_UV "+e.iridescenceMapUv:"",e.iridescenceThicknessMapUv?"#define IRIDESCENCE_THICKNESSMAP_UV "+e.iridescenceThicknessMapUv:"",e.sheenColorMapUv?"#define SHEEN_COLORMAP_UV "+e.sheenColorMapUv:"",e.sheenRoughnessMapUv?"#define SHEEN_ROUGHNESSMAP_UV "+e.sheenRoughnessMapUv:"",e.specularMapUv?"#define SPECULARMAP_UV "+e.specularMapUv:"",e.specularColorMapUv?"#define SPECULAR_COLORMAP_UV "+e.specularColorMapUv:"",e.specularIntensityMapUv?"#define SPECULAR_INTENSITYMAP_UV "+e.specularIntensityMapUv:"",e.transmissionMapUv?"#define TRANSMISSIONMAP_UV "+e.transmissionMapUv:"",e.thicknessMapUv?"#define THICKNESSMAP_UV "+e.thicknessMapUv:"",e.vertexTangents&&e.flatShading===!1?"#define USE_TANGENT":"",e.vertexColors?"#define USE_COLOR":"",e.vertexAlphas?"#define USE_COLOR_ALPHA":"",e.vertexUv1s?"#define USE_UV1":"",e.vertexUv2s?"#define USE_UV2":"",e.vertexUv3s?"#define USE_UV3":"",e.pointsUvs?"#define USE_POINTS_UV":"",e.flatShading?"#define FLAT_SHADED":"",e.skinning?"#define USE_SKINNING":"",e.morphTargets?"#define USE_MORPHTARGETS":"",e.morphNormals&&e.flatShading===!1?"#define USE_MORPHNORMALS":"",e.morphColors?"#define USE_MORPHCOLORS":"",e.morphTargetsCount>0?"#define MORPHTARGETS_TEXTURE_STRIDE "+e.morphTextureStride:"",e.morphTargetsCount>0?"#define MORPHTARGETS_COUNT "+e.morphTargetsCount:"",e.doubleSided?"#define DOUBLE_SIDED":"",e.flipSided?"#define FLIP_SIDED":"",e.shadowMapEnabled?"#define USE_SHADOWMAP":"",e.shadowMapEnabled?"#define "+l:"",e.sizeAttenuation?"#define USE_SIZEATTENUATION":"",e.numLightProbes>0?"#define USE_LIGHT_PROBES":"",e.logarithmicDepthBuffer?"#define USE_LOGDEPTHBUF":"",e.reverseDepthBuffer?"#define USE_REVERSEDEPTHBUF":"","uniform mat4 modelMatrix;","uniform mat4 modelViewMatrix;","uniform mat4 projectionMatrix;","uniform mat4 viewMatrix;","uniform mat3 normalMatrix;","uniform vec3 cameraPosition;","uniform bool isOrthographic;","#ifdef USE_INSTANCING","	attribute mat4 instanceMatrix;","#endif","#ifdef USE_INSTANCING_COLOR","	attribute vec3 instanceColor;","#endif","#ifdef USE_INSTANCING_MORPH","	uniform sampler2D morphTexture;","#endif","attribute vec3 position;","attribute vec3 normal;","attribute vec2 uv;","#ifdef USE_UV1","	attribute vec2 uv1;","#endif","#ifdef USE_UV2","	attribute vec2 uv2;","#endif","#ifdef USE_UV3","	attribute vec2 uv3;","#endif","#ifdef USE_TANGENT","	attribute vec4 tangent;","#endif","#if defined( USE_COLOR_ALPHA )","	attribute vec4 color;","#elif defined( USE_COLOR )","	attribute vec3 color;","#endif","#ifdef USE_SKINNING","	attribute vec4 skinIndex;","	attribute vec4 skinWeight;","#endif",`
`].filter(ar).join(`
`),p=[pu(e),"#define SHADER_TYPE "+e.shaderType,"#define SHADER_NAME "+e.shaderName,g,e.useFog&&e.fog?"#define USE_FOG":"",e.useFog&&e.fogExp2?"#define FOG_EXP2":"",e.alphaToCoverage?"#define ALPHA_TO_COVERAGE":"",e.map?"#define USE_MAP":"",e.matcap?"#define USE_MATCAP":"",e.envMap?"#define USE_ENVMAP":"",e.envMap?"#define "+c:"",e.envMap?"#define "+h:"",e.envMap?"#define "+u:"",d?"#define CUBEUV_TEXEL_WIDTH "+d.texelWidth:"",d?"#define CUBEUV_TEXEL_HEIGHT "+d.texelHeight:"",d?"#define CUBEUV_MAX_MIP "+d.maxMip+".0":"",e.lightMap?"#define USE_LIGHTMAP":"",e.aoMap?"#define USE_AOMAP":"",e.bumpMap?"#define USE_BUMPMAP":"",e.normalMap?"#define USE_NORMALMAP":"",e.normalMapObjectSpace?"#define USE_NORMALMAP_OBJECTSPACE":"",e.normalMapTangentSpace?"#define USE_NORMALMAP_TANGENTSPACE":"",e.emissiveMap?"#define USE_EMISSIVEMAP":"",e.anisotropy?"#define USE_ANISOTROPY":"",e.anisotropyMap?"#define USE_ANISOTROPYMAP":"",e.clearcoat?"#define USE_CLEARCOAT":"",e.clearcoatMap?"#define USE_CLEARCOATMAP":"",e.clearcoatRoughnessMap?"#define USE_CLEARCOAT_ROUGHNESSMAP":"",e.clearcoatNormalMap?"#define USE_CLEARCOAT_NORMALMAP":"",e.dispersion?"#define USE_DISPERSION":"",e.iridescence?"#define USE_IRIDESCENCE":"",e.iridescenceMap?"#define USE_IRIDESCENCEMAP":"",e.iridescenceThicknessMap?"#define USE_IRIDESCENCE_THICKNESSMAP":"",e.specularMap?"#define USE_SPECULARMAP":"",e.specularColorMap?"#define USE_SPECULAR_COLORMAP":"",e.specularIntensityMap?"#define USE_SPECULAR_INTENSITYMAP":"",e.roughnessMap?"#define USE_ROUGHNESSMAP":"",e.metalnessMap?"#define USE_METALNESSMAP":"",e.alphaMap?"#define USE_ALPHAMAP":"",e.alphaTest?"#define USE_ALPHATEST":"",e.alphaHash?"#define USE_ALPHAHASH":"",e.sheen?"#define USE_SHEEN":"",e.sheenColorMap?"#define USE_SHEEN_COLORMAP":"",e.sheenRoughnessMap?"#define USE_SHEEN_ROUGHNESSMAP":"",e.transmission?"#define USE_TRANSMISSION":"",e.transmissionMap?"#define USE_TRANSMISSIONMAP":"",e.thicknessMap?"#define USE_THICKNESSMAP":"",e.vertexTangents&&e.flatShading===!1?"#define USE_TANGENT":"",e.vertexColors||e.instancingColor||e.batchingColor?"#define USE_COLOR":"",e.vertexAlphas?"#define USE_COLOR_ALPHA":"",e.vertexUv1s?"#define USE_UV1":"",e.vertexUv2s?"#define USE_UV2":"",e.vertexUv3s?"#define USE_UV3":"",e.pointsUvs?"#define USE_POINTS_UV":"",e.gradientMap?"#define USE_GRADIENTMAP":"",e.flatShading?"#define FLAT_SHADED":"",e.doubleSided?"#define DOUBLE_SIDED":"",e.flipSided?"#define FLIP_SIDED":"",e.shadowMapEnabled?"#define USE_SHADOWMAP":"",e.shadowMapEnabled?"#define "+l:"",e.premultipliedAlpha?"#define PREMULTIPLIED_ALPHA":"",e.numLightProbes>0?"#define USE_LIGHT_PROBES":"",e.decodeVideoTexture?"#define DECODE_VIDEO_TEXTURE":"",e.decodeVideoTextureEmissive?"#define DECODE_VIDEO_TEXTURE_EMISSIVE":"",e.logarithmicDepthBuffer?"#define USE_LOGDEPTHBUF":"",e.reverseDepthBuffer?"#define USE_REVERSEDEPTHBUF":"","uniform mat4 viewMatrix;","uniform vec3 cameraPosition;","uniform bool isOrthographic;",e.toneMapping!==pi?"#define TONE_MAPPING":"",e.toneMapping!==pi?zt.tonemapping_pars_fragment:"",e.toneMapping!==pi?Dv("toneMapping",e.toneMapping):"",e.dithering?"#define DITHERING":"",e.opaque?"#define OPAQUE":"",zt.colorspace_pars_fragment,Pv("linearToOutputTexel",e.outputColorSpace),Lv(),e.useDepthPacking?"#define DEPTH_PACKING "+e.depthPacking:"",`
`].filter(ar).join(`
`)),o=cc(o),o=uu(o,e),o=du(o,e),a=cc(a),a=uu(a,e),a=du(a,e),o=fu(o),a=fu(a),e.isRawShaderMaterial!==!0&&(v=`#version 300 es
`,m=[f,"#define attribute in","#define varying out","#define texture2D texture"].join(`
`)+`
`+m,p=["#define varying in",e.glslVersion===Ah?"":"layout(location = 0) out highp vec4 pc_fragColor;",e.glslVersion===Ah?"":"#define gl_FragColor pc_fragColor","#define gl_FragDepthEXT gl_FragDepth","#define texture2D texture","#define textureCube texture","#define texture2DProj textureProj","#define texture2DLodEXT textureLod","#define texture2DProjLodEXT textureProjLod","#define textureCubeLodEXT textureLod","#define texture2DGradEXT textureGrad","#define texture2DProjGradEXT textureProjGrad","#define textureCubeGradEXT textureGrad"].join(`
`)+`
`+p);const y=v+m+o,x=v+p+a,A=lu(s,s.VERTEX_SHADER,y),w=lu(s,s.FRAGMENT_SHADER,x);s.attachShader(_,A),s.attachShader(_,w),e.index0AttributeName!==void 0?s.bindAttribLocation(_,0,e.index0AttributeName):e.morphTargets===!0&&s.bindAttribLocation(_,0,"position"),s.linkProgram(_);function T(P){if(n.debug.checkShaderErrors){const N=s.getProgramInfoLog(_).trim(),F=s.getShaderInfoLog(A).trim(),W=s.getShaderInfoLog(w).trim();let G=!0,H=!0;if(s.getProgramParameter(_,s.LINK_STATUS)===!1)if(G=!1,typeof n.debug.onShaderError=="function")n.debug.onShaderError(s,_,A,w);else{const $=hu(s,A,"vertex"),V=hu(s,w,"fragment");console.error("THREE.WebGLProgram: Shader Error "+s.getError()+" - VALIDATE_STATUS "+s.getProgramParameter(_,s.VALIDATE_STATUS)+`

Material Name: `+P.name+`
Material Type: `+P.type+`

Program Info Log: `+N+`
`+$+`
`+V)}else N!==""?console.warn("THREE.WebGLProgram: Program Info Log:",N):(F===""||W==="")&&(H=!1);H&&(P.diagnostics={runnable:G,programLog:N,vertexShader:{log:F,prefix:m},fragmentShader:{log:W,prefix:p}})}s.deleteShader(A),s.deleteShader(w),R=new zo(s,_),S=Uv(s,_)}let R;this.getUniforms=function(){return R===void 0&&T(this),R};let S;this.getAttributes=function(){return S===void 0&&T(this),S};let b=e.rendererExtensionParallelShaderCompile===!1;return this.isReady=function(){return b===!1&&(b=s.getProgramParameter(_,wv)),b},this.destroy=function(){i.releaseStatesOfProgram(this),s.deleteProgram(_),this.program=void 0},this.type=e.shaderType,this.name=e.shaderName,this.id=Av++,this.cacheKey=t,this.usedTimes=1,this.program=_,this.vertexShader=A,this.fragmentShader=w,this}let $v=0;class Yv{constructor(){this.shaderCache=new Map,this.materialCache=new Map}update(t){const e=t.vertexShader,i=t.fragmentShader,s=this._getShaderStage(e),r=this._getShaderStage(i),o=this._getShaderCacheForMaterial(t);return o.has(s)===!1&&(o.add(s),s.usedTimes++),o.has(r)===!1&&(o.add(r),r.usedTimes++),this}remove(t){const e=this.materialCache.get(t);for(const i of e)i.usedTimes--,i.usedTimes===0&&this.shaderCache.delete(i.code);return this.materialCache.delete(t),this}getVertexShaderID(t){return this._getShaderStage(t.vertexShader).id}getFragmentShaderID(t){return this._getShaderStage(t.fragmentShader).id}dispose(){this.shaderCache.clear(),this.materialCache.clear()}_getShaderCacheForMaterial(t){const e=this.materialCache;let i=e.get(t);return i===void 0&&(i=new Set,e.set(t,i)),i}_getShaderStage(t){const e=this.shaderCache;let i=e.get(t);return i===void 0&&(i=new qv(t),e.set(t,i)),i}}class qv{constructor(t){this.id=$v++,this.code=t,this.usedTimes=0}}function Kv(n,t,e,i,s,r,o){const a=new Bc,l=new Yv,c=new Set,h=[],u=s.logarithmicDepthBuffer,d=s.vertexTextures;let f=s.precision;const g={MeshDepthMaterial:"depth",MeshDistanceMaterial:"distanceRGBA",MeshNormalMaterial:"normal",MeshBasicMaterial:"basic",MeshLambertMaterial:"lambert",MeshPhongMaterial:"phong",MeshToonMaterial:"toon",MeshStandardMaterial:"physical",MeshPhysicalMaterial:"physical",MeshMatcapMaterial:"matcap",LineBasicMaterial:"basic",LineDashedMaterial:"dashed",PointsMaterial:"points",ShadowMaterial:"shadow",SpriteMaterial:"sprite"};function _(S){return c.add(S),S===0?"uv":`uv${S}`}function m(S,b,P,N,F){const W=N.fog,G=F.geometry,H=S.isMeshStandardMaterial?N.environment:null,$=(S.isMeshStandardMaterial?e:t).get(S.envMap||H),V=$&&$.mapping===ua?$.image.height:null,et=g[S.type];S.precision!==null&&(f=s.getMaxPrecision(S.precision),f!==S.precision&&console.warn("THREE.WebGLProgram.getParameters:",S.precision,"not supported, using",f,"instead."));const at=G.morphAttributes.position||G.morphAttributes.normal||G.morphAttributes.color,_t=at!==void 0?at.length:0;let Dt=0;G.morphAttributes.position!==void 0&&(Dt=1),G.morphAttributes.normal!==void 0&&(Dt=2),G.morphAttributes.color!==void 0&&(Dt=3);let $t,j,tt,ft;if(et){const ee=En[et];$t=ee.vertexShader,j=ee.fragmentShader}else $t=S.vertexShader,j=S.fragmentShader,l.update(S),tt=l.getVertexShaderID(S),ft=l.getFragmentShaderID(S);const lt=n.getRenderTarget(),Rt=n.state.buffers.depth.getReversed(),It=F.isInstancedMesh===!0,Vt=F.isBatchedMesh===!0,_e=!!S.map,Yt=!!S.matcap,Ee=!!$,U=!!S.aoMap,nn=!!S.lightMap,Wt=!!S.bumpMap,Xt=!!S.normalMap,At=!!S.displacementMap,he=!!S.emissiveMap,wt=!!S.metalnessMap,C=!!S.roughnessMap,M=S.anisotropy>0,k=S.clearcoat>0,K=S.dispersion>0,J=S.iridescence>0,Y=S.sheen>0,yt=S.transmission>0,ct=M&&!!S.anisotropyMap,pt=k&&!!S.clearcoatMap,qt=k&&!!S.clearcoatNormalMap,nt=k&&!!S.clearcoatRoughnessMap,mt=J&&!!S.iridescenceMap,Ct=J&&!!S.iridescenceThicknessMap,Pt=Y&&!!S.sheenColorMap,gt=Y&&!!S.sheenRoughnessMap,jt=!!S.specularMap,Bt=!!S.specularColorMap,ae=!!S.specularIntensityMap,D=yt&&!!S.transmissionMap,ot=yt&&!!S.thicknessMap,X=!!S.gradientMap,Z=!!S.alphaMap,dt=S.alphaTest>0,ht=!!S.alphaHash,Ut=!!S.extensions;let ve=pi;S.toneMapped&&(lt===null||lt.isXRRenderTarget===!0)&&(ve=n.toneMapping);const Ue={shaderID:et,shaderType:S.type,shaderName:S.name,vertexShader:$t,fragmentShader:j,defines:S.defines,customVertexShaderID:tt,customFragmentShaderID:ft,isRawShaderMaterial:S.isRawShaderMaterial===!0,glslVersion:S.glslVersion,precision:f,batching:Vt,batchingColor:Vt&&F._colorsTexture!==null,instancing:It,instancingColor:It&&F.instanceColor!==null,instancingMorph:It&&F.morphTexture!==null,supportsVertexTextures:d,outputColorSpace:lt===null?n.outputColorSpace:lt.isXRRenderTarget===!0?lt.texture.colorSpace:Hs,alphaToCoverage:!!S.alphaToCoverage,map:_e,matcap:Yt,envMap:Ee,envMapMode:Ee&&$.mapping,envMapCubeUVHeight:V,aoMap:U,lightMap:nn,bumpMap:Wt,normalMap:Xt,displacementMap:d&&At,emissiveMap:he,normalMapObjectSpace:Xt&&S.normalMapType===ig,normalMapTangentSpace:Xt&&S.normalMapType===Pf,metalnessMap:wt,roughnessMap:C,anisotropy:M,anisotropyMap:ct,clearcoat:k,clearcoatMap:pt,clearcoatNormalMap:qt,clearcoatRoughnessMap:nt,dispersion:K,iridescence:J,iridescenceMap:mt,iridescenceThicknessMap:Ct,sheen:Y,sheenColorMap:Pt,sheenRoughnessMap:gt,specularMap:jt,specularColorMap:Bt,specularIntensityMap:ae,transmission:yt,transmissionMap:D,thicknessMap:ot,gradientMap:X,opaque:S.transparent===!1&&S.blending===ws&&S.alphaToCoverage===!1,alphaMap:Z,alphaTest:dt,alphaHash:ht,combine:S.combine,mapUv:_e&&_(S.map.channel),aoMapUv:U&&_(S.aoMap.channel),lightMapUv:nn&&_(S.lightMap.channel),bumpMapUv:Wt&&_(S.bumpMap.channel),normalMapUv:Xt&&_(S.normalMap.channel),displacementMapUv:At&&_(S.displacementMap.channel),emissiveMapUv:he&&_(S.emissiveMap.channel),metalnessMapUv:wt&&_(S.metalnessMap.channel),roughnessMapUv:C&&_(S.roughnessMap.channel),anisotropyMapUv:ct&&_(S.anisotropyMap.channel),clearcoatMapUv:pt&&_(S.clearcoatMap.channel),clearcoatNormalMapUv:qt&&_(S.clearcoatNormalMap.channel),clearcoatRoughnessMapUv:nt&&_(S.clearcoatRoughnessMap.channel),iridescenceMapUv:mt&&_(S.iridescenceMap.channel),iridescenceThicknessMapUv:Ct&&_(S.iridescenceThicknessMap.channel),sheenColorMapUv:Pt&&_(S.sheenColorMap.channel),sheenRoughnessMapUv:gt&&_(S.sheenRoughnessMap.channel),specularMapUv:jt&&_(S.specularMap.channel),specularColorMapUv:Bt&&_(S.specularColorMap.channel),specularIntensityMapUv:ae&&_(S.specularIntensityMap.channel),transmissionMapUv:D&&_(S.transmissionMap.channel),thicknessMapUv:ot&&_(S.thicknessMap.channel),alphaMapUv:Z&&_(S.alphaMap.channel),vertexTangents:!!G.attributes.tangent&&(Xt||M),vertexColors:S.vertexColors,vertexAlphas:S.vertexColors===!0&&!!G.attributes.color&&G.attributes.color.itemSize===4,pointsUvs:F.isPoints===!0&&!!G.attributes.uv&&(_e||Z),fog:!!W,useFog:S.fog===!0,fogExp2:!!W&&W.isFogExp2,flatShading:S.flatShading===!0,sizeAttenuation:S.sizeAttenuation===!0,logarithmicDepthBuffer:u,reverseDepthBuffer:Rt,skinning:F.isSkinnedMesh===!0,morphTargets:G.morphAttributes.position!==void 0,morphNormals:G.morphAttributes.normal!==void 0,morphColors:G.morphAttributes.color!==void 0,morphTargetsCount:_t,morphTextureStride:Dt,numDirLights:b.directional.length,numPointLights:b.point.length,numSpotLights:b.spot.length,numSpotLightMaps:b.spotLightMap.length,numRectAreaLights:b.rectArea.length,numHemiLights:b.hemi.length,numDirLightShadows:b.directionalShadowMap.length,numPointLightShadows:b.pointShadowMap.length,numSpotLightShadows:b.spotShadowMap.length,numSpotLightShadowsWithMaps:b.numSpotLightShadowsWithMaps,numLightProbes:b.numLightProbes,numClippingPlanes:o.numPlanes,numClipIntersection:o.numIntersection,dithering:S.dithering,shadowMapEnabled:n.shadowMap.enabled&&P.length>0,shadowMapType:n.shadowMap.type,toneMapping:ve,decodeVideoTexture:_e&&S.map.isVideoTexture===!0&&Kt.getTransfer(S.map.colorSpace)===ie,decodeVideoTextureEmissive:he&&S.emissiveMap.isVideoTexture===!0&&Kt.getTransfer(S.emissiveMap.colorSpace)===ie,premultipliedAlpha:S.premultipliedAlpha,doubleSided:S.side===Tn,flipSided:S.side===qe,useDepthPacking:S.depthPacking>=0,depthPacking:S.depthPacking||0,index0AttributeName:S.index0AttributeName,extensionClipCullDistance:Ut&&S.extensions.clipCullDistance===!0&&i.has("WEBGL_clip_cull_distance"),extensionMultiDraw:(Ut&&S.extensions.multiDraw===!0||Vt)&&i.has("WEBGL_multi_draw"),rendererExtensionParallelShaderCompile:i.has("KHR_parallel_shader_compile"),customProgramCacheKey:S.customProgramCacheKey()};return Ue.vertexUv1s=c.has(1),Ue.vertexUv2s=c.has(2),Ue.vertexUv3s=c.has(3),c.clear(),Ue}function p(S){const b=[];if(S.shaderID?b.push(S.shaderID):(b.push(S.customVertexShaderID),b.push(S.customFragmentShaderID)),S.defines!==void 0)for(const P in S.defines)b.push(P),b.push(S.defines[P]);return S.isRawShaderMaterial===!1&&(v(b,S),y(b,S),b.push(n.outputColorSpace)),b.push(S.customProgramCacheKey),b.join()}function v(S,b){S.push(b.precision),S.push(b.outputColorSpace),S.push(b.envMapMode),S.push(b.envMapCubeUVHeight),S.push(b.mapUv),S.push(b.alphaMapUv),S.push(b.lightMapUv),S.push(b.aoMapUv),S.push(b.bumpMapUv),S.push(b.normalMapUv),S.push(b.displacementMapUv),S.push(b.emissiveMapUv),S.push(b.metalnessMapUv),S.push(b.roughnessMapUv),S.push(b.anisotropyMapUv),S.push(b.clearcoatMapUv),S.push(b.clearcoatNormalMapUv),S.push(b.clearcoatRoughnessMapUv),S.push(b.iridescenceMapUv),S.push(b.iridescenceThicknessMapUv),S.push(b.sheenColorMapUv),S.push(b.sheenRoughnessMapUv),S.push(b.specularMapUv),S.push(b.specularColorMapUv),S.push(b.specularIntensityMapUv),S.push(b.transmissionMapUv),S.push(b.thicknessMapUv),S.push(b.combine),S.push(b.fogExp2),S.push(b.sizeAttenuation),S.push(b.morphTargetsCount),S.push(b.morphAttributeCount),S.push(b.numDirLights),S.push(b.numPointLights),S.push(b.numSpotLights),S.push(b.numSpotLightMaps),S.push(b.numHemiLights),S.push(b.numRectAreaLights),S.push(b.numDirLightShadows),S.push(b.numPointLightShadows),S.push(b.numSpotLightShadows),S.push(b.numSpotLightShadowsWithMaps),S.push(b.numLightProbes),S.push(b.shadowMapType),S.push(b.toneMapping),S.push(b.numClippingPlanes),S.push(b.numClipIntersection),S.push(b.depthPacking)}function y(S,b){a.disableAll(),b.supportsVertexTextures&&a.enable(0),b.instancing&&a.enable(1),b.instancingColor&&a.enable(2),b.instancingMorph&&a.enable(3),b.matcap&&a.enable(4),b.envMap&&a.enable(5),b.normalMapObjectSpace&&a.enable(6),b.normalMapTangentSpace&&a.enable(7),b.clearcoat&&a.enable(8),b.iridescence&&a.enable(9),b.alphaTest&&a.enable(10),b.vertexColors&&a.enable(11),b.vertexAlphas&&a.enable(12),b.vertexUv1s&&a.enable(13),b.vertexUv2s&&a.enable(14),b.vertexUv3s&&a.enable(15),b.vertexTangents&&a.enable(16),b.anisotropy&&a.enable(17),b.alphaHash&&a.enable(18),b.batching&&a.enable(19),b.dispersion&&a.enable(20),b.batchingColor&&a.enable(21),S.push(a.mask),a.disableAll(),b.fog&&a.enable(0),b.useFog&&a.enable(1),b.flatShading&&a.enable(2),b.logarithmicDepthBuffer&&a.enable(3),b.reverseDepthBuffer&&a.enable(4),b.skinning&&a.enable(5),b.morphTargets&&a.enable(6),b.morphNormals&&a.enable(7),b.morphColors&&a.enable(8),b.premultipliedAlpha&&a.enable(9),b.shadowMapEnabled&&a.enable(10),b.doubleSided&&a.enable(11),b.flipSided&&a.enable(12),b.useDepthPacking&&a.enable(13),b.dithering&&a.enable(14),b.transmission&&a.enable(15),b.sheen&&a.enable(16),b.opaque&&a.enable(17),b.pointsUvs&&a.enable(18),b.decodeVideoTexture&&a.enable(19),b.decodeVideoTextureEmissive&&a.enable(20),b.alphaToCoverage&&a.enable(21),S.push(a.mask)}function x(S){const b=g[S.type];let P;if(b){const N=En[b];P=Ig.clone(N.uniforms)}else P=S.uniforms;return P}function A(S,b){let P;for(let N=0,F=h.length;N<F;N++){const W=h[N];if(W.cacheKey===b){P=W,++P.usedTimes;break}}return P===void 0&&(P=new jv(n,b,S,r),h.push(P)),P}function w(S){if(--S.usedTimes===0){const b=h.indexOf(S);h[b]=h[h.length-1],h.pop(),S.destroy()}}function T(S){l.remove(S)}function R(){l.dispose()}return{getParameters:m,getProgramCacheKey:p,getUniforms:x,acquireProgram:A,releaseProgram:w,releaseShaderCache:T,programs:h,dispose:R}}function Zv(){let n=new WeakMap;function t(o){return n.has(o)}function e(o){let a=n.get(o);return a===void 0&&(a={},n.set(o,a)),a}function i(o){n.delete(o)}function s(o,a,l){n.get(o)[a]=l}function r(){n=new WeakMap}return{has:t,get:e,remove:i,update:s,dispose:r}}function Jv(n,t){return n.groupOrder!==t.groupOrder?n.groupOrder-t.groupOrder:n.renderOrder!==t.renderOrder?n.renderOrder-t.renderOrder:n.material.id!==t.material.id?n.material.id-t.material.id:n.z!==t.z?n.z-t.z:n.id-t.id}function mu(n,t){return n.groupOrder!==t.groupOrder?n.groupOrder-t.groupOrder:n.renderOrder!==t.renderOrder?n.renderOrder-t.renderOrder:n.z!==t.z?t.z-n.z:n.id-t.id}function gu(){const n=[];let t=0;const e=[],i=[],s=[];function r(){t=0,e.length=0,i.length=0,s.length=0}function o(u,d,f,g,_,m){let p=n[t];return p===void 0?(p={id:u.id,object:u,geometry:d,material:f,groupOrder:g,renderOrder:u.renderOrder,z:_,group:m},n[t]=p):(p.id=u.id,p.object=u,p.geometry=d,p.material=f,p.groupOrder=g,p.renderOrder=u.renderOrder,p.z=_,p.group=m),t++,p}function a(u,d,f,g,_,m){const p=o(u,d,f,g,_,m);f.transmission>0?i.push(p):f.transparent===!0?s.push(p):e.push(p)}function l(u,d,f,g,_,m){const p=o(u,d,f,g,_,m);f.transmission>0?i.unshift(p):f.transparent===!0?s.unshift(p):e.unshift(p)}function c(u,d){e.length>1&&e.sort(u||Jv),i.length>1&&i.sort(d||mu),s.length>1&&s.sort(d||mu)}function h(){for(let u=t,d=n.length;u<d;u++){const f=n[u];if(f.id===null)break;f.id=null,f.object=null,f.geometry=null,f.material=null,f.group=null}}return{opaque:e,transmissive:i,transparent:s,init:r,push:a,unshift:l,finish:h,sort:c}}function Qv(){let n=new WeakMap;function t(i,s){const r=n.get(i);let o;return r===void 0?(o=new gu,n.set(i,[o])):s>=r.length?(o=new gu,r.push(o)):o=r[s],o}function e(){n=new WeakMap}return{get:t,dispose:e}}function tb(){const n={};return{get:function(t){if(n[t.id]!==void 0)return n[t.id];let e;switch(t.type){case"DirectionalLight":e={direction:new L,color:new Ht};break;case"SpotLight":e={position:new L,direction:new L,color:new Ht,distance:0,coneCos:0,penumbraCos:0,decay:0};break;case"PointLight":e={position:new L,color:new Ht,distance:0,decay:0};break;case"HemisphereLight":e={direction:new L,skyColor:new Ht,groundColor:new Ht};break;case"RectAreaLight":e={color:new Ht,position:new L,halfWidth:new L,halfHeight:new L};break}return n[t.id]=e,e}}}function eb(){const n={};return{get:function(t){if(n[t.id]!==void 0)return n[t.id];let e;switch(t.type){case"DirectionalLight":e={shadowIntensity:1,shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new Tt};break;case"SpotLight":e={shadowIntensity:1,shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new Tt};break;case"PointLight":e={shadowIntensity:1,shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new Tt,shadowCameraNear:1,shadowCameraFar:1e3};break}return n[t.id]=e,e}}}let nb=0;function ib(n,t){return(t.castShadow?2:0)-(n.castShadow?2:0)+(t.map?1:0)-(n.map?1:0)}function sb(n){const t=new tb,e=eb(),i={version:0,hash:{directionalLength:-1,pointLength:-1,spotLength:-1,rectAreaLength:-1,hemiLength:-1,numDirectionalShadows:-1,numPointShadows:-1,numSpotShadows:-1,numSpotMaps:-1,numLightProbes:-1},ambient:[0,0,0],probe:[],directional:[],directionalShadow:[],directionalShadowMap:[],directionalShadowMatrix:[],spot:[],spotLightMap:[],spotShadow:[],spotShadowMap:[],spotLightMatrix:[],rectArea:[],rectAreaLTC1:null,rectAreaLTC2:null,point:[],pointShadow:[],pointShadowMap:[],pointShadowMatrix:[],hemi:[],numSpotLightShadowsWithMaps:0,numLightProbes:0};for(let c=0;c<9;c++)i.probe.push(new L);const s=new L,r=new oe,o=new oe;function a(c){let h=0,u=0,d=0;for(let S=0;S<9;S++)i.probe[S].set(0,0,0);let f=0,g=0,_=0,m=0,p=0,v=0,y=0,x=0,A=0,w=0,T=0;c.sort(ib);for(let S=0,b=c.length;S<b;S++){const P=c[S],N=P.color,F=P.intensity,W=P.distance,G=P.shadow&&P.shadow.map?P.shadow.map.texture:null;if(P.isAmbientLight)h+=N.r*F,u+=N.g*F,d+=N.b*F;else if(P.isLightProbe){for(let H=0;H<9;H++)i.probe[H].addScaledVector(P.sh.coefficients[H],F);T++}else if(P.isDirectionalLight){const H=t.get(P);if(H.color.copy(P.color).multiplyScalar(P.intensity),P.castShadow){const $=P.shadow,V=e.get(P);V.shadowIntensity=$.intensity,V.shadowBias=$.bias,V.shadowNormalBias=$.normalBias,V.shadowRadius=$.radius,V.shadowMapSize=$.mapSize,i.directionalShadow[f]=V,i.directionalShadowMap[f]=G,i.directionalShadowMatrix[f]=P.shadow.matrix,v++}i.directional[f]=H,f++}else if(P.isSpotLight){const H=t.get(P);H.position.setFromMatrixPosition(P.matrixWorld),H.color.copy(N).multiplyScalar(F),H.distance=W,H.coneCos=Math.cos(P.angle),H.penumbraCos=Math.cos(P.angle*(1-P.penumbra)),H.decay=P.decay,i.spot[_]=H;const $=P.shadow;if(P.map&&(i.spotLightMap[A]=P.map,A++,$.updateMatrices(P),P.castShadow&&w++),i.spotLightMatrix[_]=$.matrix,P.castShadow){const V=e.get(P);V.shadowIntensity=$.intensity,V.shadowBias=$.bias,V.shadowNormalBias=$.normalBias,V.shadowRadius=$.radius,V.shadowMapSize=$.mapSize,i.spotShadow[_]=V,i.spotShadowMap[_]=G,x++}_++}else if(P.isRectAreaLight){const H=t.get(P);H.color.copy(N).multiplyScalar(F),H.halfWidth.set(P.width*.5,0,0),H.halfHeight.set(0,P.height*.5,0),i.rectArea[m]=H,m++}else if(P.isPointLight){const H=t.get(P);if(H.color.copy(P.color).multiplyScalar(P.intensity),H.distance=P.distance,H.decay=P.decay,P.castShadow){const $=P.shadow,V=e.get(P);V.shadowIntensity=$.intensity,V.shadowBias=$.bias,V.shadowNormalBias=$.normalBias,V.shadowRadius=$.radius,V.shadowMapSize=$.mapSize,V.shadowCameraNear=$.camera.near,V.shadowCameraFar=$.camera.far,i.pointShadow[g]=V,i.pointShadowMap[g]=G,i.pointShadowMatrix[g]=P.shadow.matrix,y++}i.point[g]=H,g++}else if(P.isHemisphereLight){const H=t.get(P);H.skyColor.copy(P.color).multiplyScalar(F),H.groundColor.copy(P.groundColor).multiplyScalar(F),i.hemi[p]=H,p++}}m>0&&(n.has("OES_texture_float_linear")===!0?(i.rectAreaLTC1=rt.LTC_FLOAT_1,i.rectAreaLTC2=rt.LTC_FLOAT_2):(i.rectAreaLTC1=rt.LTC_HALF_1,i.rectAreaLTC2=rt.LTC_HALF_2)),i.ambient[0]=h,i.ambient[1]=u,i.ambient[2]=d;const R=i.hash;(R.directionalLength!==f||R.pointLength!==g||R.spotLength!==_||R.rectAreaLength!==m||R.hemiLength!==p||R.numDirectionalShadows!==v||R.numPointShadows!==y||R.numSpotShadows!==x||R.numSpotMaps!==A||R.numLightProbes!==T)&&(i.directional.length=f,i.spot.length=_,i.rectArea.length=m,i.point.length=g,i.hemi.length=p,i.directionalShadow.length=v,i.directionalShadowMap.length=v,i.pointShadow.length=y,i.pointShadowMap.length=y,i.spotShadow.length=x,i.spotShadowMap.length=x,i.directionalShadowMatrix.length=v,i.pointShadowMatrix.length=y,i.spotLightMatrix.length=x+A-w,i.spotLightMap.length=A,i.numSpotLightShadowsWithMaps=w,i.numLightProbes=T,R.directionalLength=f,R.pointLength=g,R.spotLength=_,R.rectAreaLength=m,R.hemiLength=p,R.numDirectionalShadows=v,R.numPointShadows=y,R.numSpotShadows=x,R.numSpotMaps=A,R.numLightProbes=T,i.version=nb++)}function l(c,h){let u=0,d=0,f=0,g=0,_=0;const m=h.matrixWorldInverse;for(let p=0,v=c.length;p<v;p++){const y=c[p];if(y.isDirectionalLight){const x=i.directional[u];x.direction.setFromMatrixPosition(y.matrixWorld),s.setFromMatrixPosition(y.target.matrixWorld),x.direction.sub(s),x.direction.transformDirection(m),u++}else if(y.isSpotLight){const x=i.spot[f];x.position.setFromMatrixPosition(y.matrixWorld),x.position.applyMatrix4(m),x.direction.setFromMatrixPosition(y.matrixWorld),s.setFromMatrixPosition(y.target.matrixWorld),x.direction.sub(s),x.direction.transformDirection(m),f++}else if(y.isRectAreaLight){const x=i.rectArea[g];x.position.setFromMatrixPosition(y.matrixWorld),x.position.applyMatrix4(m),o.identity(),r.copy(y.matrixWorld),r.premultiply(m),o.extractRotation(r),x.halfWidth.set(y.width*.5,0,0),x.halfHeight.set(0,y.height*.5,0),x.halfWidth.applyMatrix4(o),x.halfHeight.applyMatrix4(o),g++}else if(y.isPointLight){const x=i.point[d];x.position.setFromMatrixPosition(y.matrixWorld),x.position.applyMatrix4(m),d++}else if(y.isHemisphereLight){const x=i.hemi[_];x.direction.setFromMatrixPosition(y.matrixWorld),x.direction.transformDirection(m),_++}}}return{setup:a,setupView:l,state:i}}function _u(n){const t=new sb(n),e=[],i=[];function s(h){c.camera=h,e.length=0,i.length=0}function r(h){e.push(h)}function o(h){i.push(h)}function a(){t.setup(e)}function l(h){t.setupView(e,h)}const c={lightsArray:e,shadowsArray:i,camera:null,lights:t,transmissionRenderTarget:{}};return{init:s,state:c,setupLights:a,setupLightsView:l,pushLight:r,pushShadow:o}}function rb(n){let t=new WeakMap;function e(s,r=0){const o=t.get(s);let a;return o===void 0?(a=new _u(n),t.set(s,[a])):r>=o.length?(a=new _u(n),o.push(a)):a=o[r],a}function i(){t=new WeakMap}return{get:e,dispose:i}}class ob extends yi{static get type(){return"MeshDepthMaterial"}constructor(t){super(),this.isMeshDepthMaterial=!0,this.depthPacking=eg,this.map=null,this.alphaMap=null,this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.wireframe=!1,this.wireframeLinewidth=1,this.setValues(t)}copy(t){return super.copy(t),this.depthPacking=t.depthPacking,this.map=t.map,this.alphaMap=t.alphaMap,this.displacementMap=t.displacementMap,this.displacementScale=t.displacementScale,this.displacementBias=t.displacementBias,this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this}}class ab extends yi{static get type(){return"MeshDistanceMaterial"}constructor(t){super(),this.isMeshDistanceMaterial=!0,this.map=null,this.alphaMap=null,this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.setValues(t)}copy(t){return super.copy(t),this.map=t.map,this.alphaMap=t.alphaMap,this.displacementMap=t.displacementMap,this.displacementScale=t.displacementScale,this.displacementBias=t.displacementBias,this}}const lb=`void main() {
	gl_Position = vec4( position, 1.0 );
}`,cb=`uniform sampler2D shadow_pass;
uniform vec2 resolution;
uniform float radius;
#include <packing>
void main() {
	const float samples = float( VSM_SAMPLES );
	float mean = 0.0;
	float squared_mean = 0.0;
	float uvStride = samples <= 1.0 ? 0.0 : 2.0 / ( samples - 1.0 );
	float uvStart = samples <= 1.0 ? 0.0 : - 1.0;
	for ( float i = 0.0; i < samples; i ++ ) {
		float uvOffset = uvStart + i * uvStride;
		#ifdef HORIZONTAL_PASS
			vec2 distribution = unpackRGBATo2Half( texture2D( shadow_pass, ( gl_FragCoord.xy + vec2( uvOffset, 0.0 ) * radius ) / resolution ) );
			mean += distribution.x;
			squared_mean += distribution.y * distribution.y + distribution.x * distribution.x;
		#else
			float depth = unpackRGBAToDepth( texture2D( shadow_pass, ( gl_FragCoord.xy + vec2( 0.0, uvOffset ) * radius ) / resolution ) );
			mean += depth;
			squared_mean += depth * depth;
		#endif
	}
	mean = mean / samples;
	squared_mean = squared_mean / samples;
	float std_dev = sqrt( squared_mean - mean * mean );
	gl_FragColor = pack2HalfToRGBA( vec2( mean, std_dev ) );
}`;function hb(n,t,e){let i=new zc;const s=new Tt,r=new Tt,o=new be,a=new ob({depthPacking:ng}),l=new ab,c={},h=e.maxTextureSize,u={[_i]:qe,[qe]:_i,[Tn]:Tn},d=new xi({defines:{VSM_SAMPLES:8},uniforms:{shadow_pass:{value:null},resolution:{value:new Tt},radius:{value:4}},vertexShader:lb,fragmentShader:cb}),f=d.clone();f.defines.HORIZONTAL_PASS=1;const g=new Te;g.setAttribute("position",new en(new Float32Array([-1,-1,.5,3,-1,.5,-1,3,.5]),3));const _=new xe(g,d),m=this;this.enabled=!1,this.autoUpdate=!0,this.needsUpdate=!1,this.type=_f;let p=this.type;this.render=function(w,T,R){if(m.enabled===!1||m.autoUpdate===!1&&m.needsUpdate===!1||w.length===0)return;const S=n.getRenderTarget(),b=n.getActiveCubeFace(),P=n.getActiveMipmapLevel(),N=n.state;N.setBlending(fi),N.buffers.color.setClear(1,1,1,1),N.buffers.depth.setTest(!0),N.setScissorTest(!1);const F=p!==zn&&this.type===zn,W=p===zn&&this.type!==zn;for(let G=0,H=w.length;G<H;G++){const $=w[G],V=$.shadow;if(V===void 0){console.warn("THREE.WebGLShadowMap:",$,"has no shadow.");continue}if(V.autoUpdate===!1&&V.needsUpdate===!1)continue;s.copy(V.mapSize);const et=V.getFrameExtents();if(s.multiply(et),r.copy(V.mapSize),(s.x>h||s.y>h)&&(s.x>h&&(r.x=Math.floor(h/et.x),s.x=r.x*et.x,V.mapSize.x=r.x),s.y>h&&(r.y=Math.floor(h/et.y),s.y=r.y*et.y,V.mapSize.y=r.y)),V.map===null||F===!0||W===!0){const _t=this.type!==zn?{minFilter:xn,magFilter:xn}:{};V.map!==null&&V.map.dispose(),V.map=new ji(s.x,s.y,_t),V.map.texture.name=$.name+".shadowMap",V.camera.updateProjectionMatrix()}n.setRenderTarget(V.map),n.clear();const at=V.getViewportCount();for(let _t=0;_t<at;_t++){const Dt=V.getViewport(_t);o.set(r.x*Dt.x,r.y*Dt.y,r.x*Dt.z,r.y*Dt.w),N.viewport(o),V.updateMatrices($,_t),i=V.getFrustum(),x(T,R,V.camera,$,this.type)}V.isPointLightShadow!==!0&&this.type===zn&&v(V,R),V.needsUpdate=!1}p=this.type,m.needsUpdate=!1,n.setRenderTarget(S,b,P)};function v(w,T){const R=t.update(_);d.defines.VSM_SAMPLES!==w.blurSamples&&(d.defines.VSM_SAMPLES=w.blurSamples,f.defines.VSM_SAMPLES=w.blurSamples,d.needsUpdate=!0,f.needsUpdate=!0),w.mapPass===null&&(w.mapPass=new ji(s.x,s.y)),d.uniforms.shadow_pass.value=w.map.texture,d.uniforms.resolution.value=w.mapSize,d.uniforms.radius.value=w.radius,n.setRenderTarget(w.mapPass),n.clear(),n.renderBufferDirect(T,null,R,d,_,null),f.uniforms.shadow_pass.value=w.mapPass.texture,f.uniforms.resolution.value=w.mapSize,f.uniforms.radius.value=w.radius,n.setRenderTarget(w.map),n.clear(),n.renderBufferDirect(T,null,R,f,_,null)}function y(w,T,R,S){let b=null;const P=R.isPointLight===!0?w.customDistanceMaterial:w.customDepthMaterial;if(P!==void 0)b=P;else if(b=R.isPointLight===!0?l:a,n.localClippingEnabled&&T.clipShadows===!0&&Array.isArray(T.clippingPlanes)&&T.clippingPlanes.length!==0||T.displacementMap&&T.displacementScale!==0||T.alphaMap&&T.alphaTest>0||T.map&&T.alphaTest>0){const N=b.uuid,F=T.uuid;let W=c[N];W===void 0&&(W={},c[N]=W);let G=W[F];G===void 0&&(G=b.clone(),W[F]=G,T.addEventListener("dispose",A)),b=G}if(b.visible=T.visible,b.wireframe=T.wireframe,S===zn?b.side=T.shadowSide!==null?T.shadowSide:T.side:b.side=T.shadowSide!==null?T.shadowSide:u[T.side],b.alphaMap=T.alphaMap,b.alphaTest=T.alphaTest,b.map=T.map,b.clipShadows=T.clipShadows,b.clippingPlanes=T.clippingPlanes,b.clipIntersection=T.clipIntersection,b.displacementMap=T.displacementMap,b.displacementScale=T.displacementScale,b.displacementBias=T.displacementBias,b.wireframeLinewidth=T.wireframeLinewidth,b.linewidth=T.linewidth,R.isPointLight===!0&&b.isMeshDistanceMaterial===!0){const N=n.properties.get(b);N.light=R}return b}function x(w,T,R,S,b){if(w.visible===!1)return;if(w.layers.test(T.layers)&&(w.isMesh||w.isLine||w.isPoints)&&(w.castShadow||w.receiveShadow&&b===zn)&&(!w.frustumCulled||i.intersectsObject(w))){w.modelViewMatrix.multiplyMatrices(R.matrixWorldInverse,w.matrixWorld);const F=t.update(w),W=w.material;if(Array.isArray(W)){const G=F.groups;for(let H=0,$=G.length;H<$;H++){const V=G[H],et=W[V.materialIndex];if(et&&et.visible){const at=y(w,et,S,b);w.onBeforeShadow(n,w,T,R,F,at,V),n.renderBufferDirect(R,null,F,at,w,V),w.onAfterShadow(n,w,T,R,F,at,V)}}}else if(W.visible){const G=y(w,W,S,b);w.onBeforeShadow(n,w,T,R,F,G,null),n.renderBufferDirect(R,null,F,G,w,null),w.onAfterShadow(n,w,T,R,F,G,null)}}const N=w.children;for(let F=0,W=N.length;F<W;F++)x(N[F],T,R,S,b)}function A(w){w.target.removeEventListener("dispose",A);for(const R in c){const S=c[R],b=w.target.uuid;b in S&&(S[b].dispose(),delete S[b])}}}const ub={[El]:Tl,[wl]:Rl,[Al]:Pl,[Ds]:Cl,[Tl]:El,[Rl]:wl,[Pl]:Al,[Cl]:Ds};function db(n,t){function e(){let D=!1;const ot=new be;let X=null;const Z=new be(0,0,0,0);return{setMask:function(dt){X!==dt&&!D&&(n.colorMask(dt,dt,dt,dt),X=dt)},setLocked:function(dt){D=dt},setClear:function(dt,ht,Ut,ve,Ue){Ue===!0&&(dt*=ve,ht*=ve,Ut*=ve),ot.set(dt,ht,Ut,ve),Z.equals(ot)===!1&&(n.clearColor(dt,ht,Ut,ve),Z.copy(ot))},reset:function(){D=!1,X=null,Z.set(-1,0,0,0)}}}function i(){let D=!1,ot=!1,X=null,Z=null,dt=null;return{setReversed:function(ht){if(ot!==ht){const Ut=t.get("EXT_clip_control");ot?Ut.clipControlEXT(Ut.LOWER_LEFT_EXT,Ut.ZERO_TO_ONE_EXT):Ut.clipControlEXT(Ut.LOWER_LEFT_EXT,Ut.NEGATIVE_ONE_TO_ONE_EXT);const ve=dt;dt=null,this.setClear(ve)}ot=ht},getReversed:function(){return ot},setTest:function(ht){ht?lt(n.DEPTH_TEST):Rt(n.DEPTH_TEST)},setMask:function(ht){X!==ht&&!D&&(n.depthMask(ht),X=ht)},setFunc:function(ht){if(ot&&(ht=ub[ht]),Z!==ht){switch(ht){case El:n.depthFunc(n.NEVER);break;case Tl:n.depthFunc(n.ALWAYS);break;case wl:n.depthFunc(n.LESS);break;case Ds:n.depthFunc(n.LEQUAL);break;case Al:n.depthFunc(n.EQUAL);break;case Cl:n.depthFunc(n.GEQUAL);break;case Rl:n.depthFunc(n.GREATER);break;case Pl:n.depthFunc(n.NOTEQUAL);break;default:n.depthFunc(n.LEQUAL)}Z=ht}},setLocked:function(ht){D=ht},setClear:function(ht){dt!==ht&&(ot&&(ht=1-ht),n.clearDepth(ht),dt=ht)},reset:function(){D=!1,X=null,Z=null,dt=null,ot=!1}}}function s(){let D=!1,ot=null,X=null,Z=null,dt=null,ht=null,Ut=null,ve=null,Ue=null;return{setTest:function(ee){D||(ee?lt(n.STENCIL_TEST):Rt(n.STENCIL_TEST))},setMask:function(ee){ot!==ee&&!D&&(n.stencilMask(ee),ot=ee)},setFunc:function(ee,hn,Dn){(X!==ee||Z!==hn||dt!==Dn)&&(n.stencilFunc(ee,hn,Dn),X=ee,Z=hn,dt=Dn)},setOp:function(ee,hn,Dn){(ht!==ee||Ut!==hn||ve!==Dn)&&(n.stencilOp(ee,hn,Dn),ht=ee,Ut=hn,ve=Dn)},setLocked:function(ee){D=ee},setClear:function(ee){Ue!==ee&&(n.clearStencil(ee),Ue=ee)},reset:function(){D=!1,ot=null,X=null,Z=null,dt=null,ht=null,Ut=null,ve=null,Ue=null}}}const r=new e,o=new i,a=new s,l=new WeakMap,c=new WeakMap;let h={},u={},d=new WeakMap,f=[],g=null,_=!1,m=null,p=null,v=null,y=null,x=null,A=null,w=null,T=new Ht(0,0,0),R=0,S=!1,b=null,P=null,N=null,F=null,W=null;const G=n.getParameter(n.MAX_COMBINED_TEXTURE_IMAGE_UNITS);let H=!1,$=0;const V=n.getParameter(n.VERSION);V.indexOf("WebGL")!==-1?($=parseFloat(/^WebGL (\d)/.exec(V)[1]),H=$>=1):V.indexOf("OpenGL ES")!==-1&&($=parseFloat(/^OpenGL ES (\d)/.exec(V)[1]),H=$>=2);let et=null,at={};const _t=n.getParameter(n.SCISSOR_BOX),Dt=n.getParameter(n.VIEWPORT),$t=new be().fromArray(_t),j=new be().fromArray(Dt);function tt(D,ot,X,Z){const dt=new Uint8Array(4),ht=n.createTexture();n.bindTexture(D,ht),n.texParameteri(D,n.TEXTURE_MIN_FILTER,n.NEAREST),n.texParameteri(D,n.TEXTURE_MAG_FILTER,n.NEAREST);for(let Ut=0;Ut<X;Ut++)D===n.TEXTURE_3D||D===n.TEXTURE_2D_ARRAY?n.texImage3D(ot,0,n.RGBA,1,1,Z,0,n.RGBA,n.UNSIGNED_BYTE,dt):n.texImage2D(ot+Ut,0,n.RGBA,1,1,0,n.RGBA,n.UNSIGNED_BYTE,dt);return ht}const ft={};ft[n.TEXTURE_2D]=tt(n.TEXTURE_2D,n.TEXTURE_2D,1),ft[n.TEXTURE_CUBE_MAP]=tt(n.TEXTURE_CUBE_MAP,n.TEXTURE_CUBE_MAP_POSITIVE_X,6),ft[n.TEXTURE_2D_ARRAY]=tt(n.TEXTURE_2D_ARRAY,n.TEXTURE_2D_ARRAY,1,1),ft[n.TEXTURE_3D]=tt(n.TEXTURE_3D,n.TEXTURE_3D,1,1),r.setClear(0,0,0,1),o.setClear(1),a.setClear(0),lt(n.DEPTH_TEST),o.setFunc(Ds),Wt(!1),Xt(Mh),lt(n.CULL_FACE),U(fi);function lt(D){h[D]!==!0&&(n.enable(D),h[D]=!0)}function Rt(D){h[D]!==!1&&(n.disable(D),h[D]=!1)}function It(D,ot){return u[D]!==ot?(n.bindFramebuffer(D,ot),u[D]=ot,D===n.DRAW_FRAMEBUFFER&&(u[n.FRAMEBUFFER]=ot),D===n.FRAMEBUFFER&&(u[n.DRAW_FRAMEBUFFER]=ot),!0):!1}function Vt(D,ot){let X=f,Z=!1;if(D){X=d.get(ot),X===void 0&&(X=[],d.set(ot,X));const dt=D.textures;if(X.length!==dt.length||X[0]!==n.COLOR_ATTACHMENT0){for(let ht=0,Ut=dt.length;ht<Ut;ht++)X[ht]=n.COLOR_ATTACHMENT0+ht;X.length=dt.length,Z=!0}}else X[0]!==n.BACK&&(X[0]=n.BACK,Z=!0);Z&&n.drawBuffers(X)}function _e(D){return g!==D?(n.useProgram(D),g=D,!0):!1}const Yt={[Fi]:n.FUNC_ADD,[Cm]:n.FUNC_SUBTRACT,[Rm]:n.FUNC_REVERSE_SUBTRACT};Yt[Pm]=n.MIN,Yt[Dm]=n.MAX;const Ee={[Lm]:n.ZERO,[Im]:n.ONE,[Om]:n.SRC_COLOR,[Ml]:n.SRC_ALPHA,[zm]:n.SRC_ALPHA_SATURATE,[km]:n.DST_COLOR,[Nm]:n.DST_ALPHA,[Um]:n.ONE_MINUS_SRC_COLOR,[Sl]:n.ONE_MINUS_SRC_ALPHA,[Bm]:n.ONE_MINUS_DST_COLOR,[Fm]:n.ONE_MINUS_DST_ALPHA,[Hm]:n.CONSTANT_COLOR,[Vm]:n.ONE_MINUS_CONSTANT_COLOR,[Gm]:n.CONSTANT_ALPHA,[Wm]:n.ONE_MINUS_CONSTANT_ALPHA};function U(D,ot,X,Z,dt,ht,Ut,ve,Ue,ee){if(D===fi){_===!0&&(Rt(n.BLEND),_=!1);return}if(_===!1&&(lt(n.BLEND),_=!0),D!==Am){if(D!==m||ee!==S){if((p!==Fi||x!==Fi)&&(n.blendEquation(n.FUNC_ADD),p=Fi,x=Fi),ee)switch(D){case ws:n.blendFuncSeparate(n.ONE,n.ONE_MINUS_SRC_ALPHA,n.ONE,n.ONE_MINUS_SRC_ALPHA);break;case Sh:n.blendFunc(n.ONE,n.ONE);break;case Eh:n.blendFuncSeparate(n.ZERO,n.ONE_MINUS_SRC_COLOR,n.ZERO,n.ONE);break;case Th:n.blendFuncSeparate(n.ZERO,n.SRC_COLOR,n.ZERO,n.SRC_ALPHA);break;default:console.error("THREE.WebGLState: Invalid blending: ",D);break}else switch(D){case ws:n.blendFuncSeparate(n.SRC_ALPHA,n.ONE_MINUS_SRC_ALPHA,n.ONE,n.ONE_MINUS_SRC_ALPHA);break;case Sh:n.blendFunc(n.SRC_ALPHA,n.ONE);break;case Eh:n.blendFuncSeparate(n.ZERO,n.ONE_MINUS_SRC_COLOR,n.ZERO,n.ONE);break;case Th:n.blendFunc(n.ZERO,n.SRC_COLOR);break;default:console.error("THREE.WebGLState: Invalid blending: ",D);break}v=null,y=null,A=null,w=null,T.set(0,0,0),R=0,m=D,S=ee}return}dt=dt||ot,ht=ht||X,Ut=Ut||Z,(ot!==p||dt!==x)&&(n.blendEquationSeparate(Yt[ot],Yt[dt]),p=ot,x=dt),(X!==v||Z!==y||ht!==A||Ut!==w)&&(n.blendFuncSeparate(Ee[X],Ee[Z],Ee[ht],Ee[Ut]),v=X,y=Z,A=ht,w=Ut),(ve.equals(T)===!1||Ue!==R)&&(n.blendColor(ve.r,ve.g,ve.b,Ue),T.copy(ve),R=Ue),m=D,S=!1}function nn(D,ot){D.side===Tn?Rt(n.CULL_FACE):lt(n.CULL_FACE);let X=D.side===qe;ot&&(X=!X),Wt(X),D.blending===ws&&D.transparent===!1?U(fi):U(D.blending,D.blendEquation,D.blendSrc,D.blendDst,D.blendEquationAlpha,D.blendSrcAlpha,D.blendDstAlpha,D.blendColor,D.blendAlpha,D.premultipliedAlpha),o.setFunc(D.depthFunc),o.setTest(D.depthTest),o.setMask(D.depthWrite),r.setMask(D.colorWrite);const Z=D.stencilWrite;a.setTest(Z),Z&&(a.setMask(D.stencilWriteMask),a.setFunc(D.stencilFunc,D.stencilRef,D.stencilFuncMask),a.setOp(D.stencilFail,D.stencilZFail,D.stencilZPass)),he(D.polygonOffset,D.polygonOffsetFactor,D.polygonOffsetUnits),D.alphaToCoverage===!0?lt(n.SAMPLE_ALPHA_TO_COVERAGE):Rt(n.SAMPLE_ALPHA_TO_COVERAGE)}function Wt(D){b!==D&&(D?n.frontFace(n.CW):n.frontFace(n.CCW),b=D)}function Xt(D){D!==Em?(lt(n.CULL_FACE),D!==P&&(D===Mh?n.cullFace(n.BACK):D===Tm?n.cullFace(n.FRONT):n.cullFace(n.FRONT_AND_BACK))):Rt(n.CULL_FACE),P=D}function At(D){D!==N&&(H&&n.lineWidth(D),N=D)}function he(D,ot,X){D?(lt(n.POLYGON_OFFSET_FILL),(F!==ot||W!==X)&&(n.polygonOffset(ot,X),F=ot,W=X)):Rt(n.POLYGON_OFFSET_FILL)}function wt(D){D?lt(n.SCISSOR_TEST):Rt(n.SCISSOR_TEST)}function C(D){D===void 0&&(D=n.TEXTURE0+G-1),et!==D&&(n.activeTexture(D),et=D)}function M(D,ot,X){X===void 0&&(et===null?X=n.TEXTURE0+G-1:X=et);let Z=at[X];Z===void 0&&(Z={type:void 0,texture:void 0},at[X]=Z),(Z.type!==D||Z.texture!==ot)&&(et!==X&&(n.activeTexture(X),et=X),n.bindTexture(D,ot||ft[D]),Z.type=D,Z.texture=ot)}function k(){const D=at[et];D!==void 0&&D.type!==void 0&&(n.bindTexture(D.type,null),D.type=void 0,D.texture=void 0)}function K(){try{n.compressedTexImage2D.apply(n,arguments)}catch(D){console.error("THREE.WebGLState:",D)}}function J(){try{n.compressedTexImage3D.apply(n,arguments)}catch(D){console.error("THREE.WebGLState:",D)}}function Y(){try{n.texSubImage2D.apply(n,arguments)}catch(D){console.error("THREE.WebGLState:",D)}}function yt(){try{n.texSubImage3D.apply(n,arguments)}catch(D){console.error("THREE.WebGLState:",D)}}function ct(){try{n.compressedTexSubImage2D.apply(n,arguments)}catch(D){console.error("THREE.WebGLState:",D)}}function pt(){try{n.compressedTexSubImage3D.apply(n,arguments)}catch(D){console.error("THREE.WebGLState:",D)}}function qt(){try{n.texStorage2D.apply(n,arguments)}catch(D){console.error("THREE.WebGLState:",D)}}function nt(){try{n.texStorage3D.apply(n,arguments)}catch(D){console.error("THREE.WebGLState:",D)}}function mt(){try{n.texImage2D.apply(n,arguments)}catch(D){console.error("THREE.WebGLState:",D)}}function Ct(){try{n.texImage3D.apply(n,arguments)}catch(D){console.error("THREE.WebGLState:",D)}}function Pt(D){$t.equals(D)===!1&&(n.scissor(D.x,D.y,D.z,D.w),$t.copy(D))}function gt(D){j.equals(D)===!1&&(n.viewport(D.x,D.y,D.z,D.w),j.copy(D))}function jt(D,ot){let X=c.get(ot);X===void 0&&(X=new WeakMap,c.set(ot,X));let Z=X.get(D);Z===void 0&&(Z=n.getUniformBlockIndex(ot,D.name),X.set(D,Z))}function Bt(D,ot){const Z=c.get(ot).get(D);l.get(ot)!==Z&&(n.uniformBlockBinding(ot,Z,D.__bindingPointIndex),l.set(ot,Z))}function ae(){n.disable(n.BLEND),n.disable(n.CULL_FACE),n.disable(n.DEPTH_TEST),n.disable(n.POLYGON_OFFSET_FILL),n.disable(n.SCISSOR_TEST),n.disable(n.STENCIL_TEST),n.disable(n.SAMPLE_ALPHA_TO_COVERAGE),n.blendEquation(n.FUNC_ADD),n.blendFunc(n.ONE,n.ZERO),n.blendFuncSeparate(n.ONE,n.ZERO,n.ONE,n.ZERO),n.blendColor(0,0,0,0),n.colorMask(!0,!0,!0,!0),n.clearColor(0,0,0,0),n.depthMask(!0),n.depthFunc(n.LESS),o.setReversed(!1),n.clearDepth(1),n.stencilMask(4294967295),n.stencilFunc(n.ALWAYS,0,4294967295),n.stencilOp(n.KEEP,n.KEEP,n.KEEP),n.clearStencil(0),n.cullFace(n.BACK),n.frontFace(n.CCW),n.polygonOffset(0,0),n.activeTexture(n.TEXTURE0),n.bindFramebuffer(n.FRAMEBUFFER,null),n.bindFramebuffer(n.DRAW_FRAMEBUFFER,null),n.bindFramebuffer(n.READ_FRAMEBUFFER,null),n.useProgram(null),n.lineWidth(1),n.scissor(0,0,n.canvas.width,n.canvas.height),n.viewport(0,0,n.canvas.width,n.canvas.height),h={},et=null,at={},u={},d=new WeakMap,f=[],g=null,_=!1,m=null,p=null,v=null,y=null,x=null,A=null,w=null,T=new Ht(0,0,0),R=0,S=!1,b=null,P=null,N=null,F=null,W=null,$t.set(0,0,n.canvas.width,n.canvas.height),j.set(0,0,n.canvas.width,n.canvas.height),r.reset(),o.reset(),a.reset()}return{buffers:{color:r,depth:o,stencil:a},enable:lt,disable:Rt,bindFramebuffer:It,drawBuffers:Vt,useProgram:_e,setBlending:U,setMaterial:nn,setFlipSided:Wt,setCullFace:Xt,setLineWidth:At,setPolygonOffset:he,setScissorTest:wt,activeTexture:C,bindTexture:M,unbindTexture:k,compressedTexImage2D:K,compressedTexImage3D:J,texImage2D:mt,texImage3D:Ct,updateUBOMapping:jt,uniformBlockBinding:Bt,texStorage2D:qt,texStorage3D:nt,texSubImage2D:Y,texSubImage3D:yt,compressedTexSubImage2D:ct,compressedTexSubImage3D:pt,scissor:Pt,viewport:gt,reset:ae}}function xu(n,t,e,i){const s=fb(i);switch(e){case Sf:return n*t;case Tf:return n*t;case wf:return n*t*2;case Af:return n*t/s.components*s.byteLength;case Nc:return n*t/s.components*s.byteLength;case Cf:return n*t*2/s.components*s.byteLength;case Fc:return n*t*2/s.components*s.byteLength;case Ef:return n*t*3/s.components*s.byteLength;case mn:return n*t*4/s.components*s.byteLength;case kc:return n*t*4/s.components*s.byteLength;case Oo:case Uo:return Math.floor((n+3)/4)*Math.floor((t+3)/4)*8;case No:case Fo:return Math.floor((n+3)/4)*Math.floor((t+3)/4)*16;case Nl:case kl:return Math.max(n,16)*Math.max(t,8)/4;case Ul:case Fl:return Math.max(n,8)*Math.max(t,8)/2;case Bl:case zl:return Math.floor((n+3)/4)*Math.floor((t+3)/4)*8;case Hl:return Math.floor((n+3)/4)*Math.floor((t+3)/4)*16;case Vl:return Math.floor((n+3)/4)*Math.floor((t+3)/4)*16;case Gl:return Math.floor((n+4)/5)*Math.floor((t+3)/4)*16;case Wl:return Math.floor((n+4)/5)*Math.floor((t+4)/5)*16;case Xl:return Math.floor((n+5)/6)*Math.floor((t+4)/5)*16;case jl:return Math.floor((n+5)/6)*Math.floor((t+5)/6)*16;case $l:return Math.floor((n+7)/8)*Math.floor((t+4)/5)*16;case Yl:return Math.floor((n+7)/8)*Math.floor((t+5)/6)*16;case ql:return Math.floor((n+7)/8)*Math.floor((t+7)/8)*16;case Kl:return Math.floor((n+9)/10)*Math.floor((t+4)/5)*16;case Zl:return Math.floor((n+9)/10)*Math.floor((t+5)/6)*16;case Jl:return Math.floor((n+9)/10)*Math.floor((t+7)/8)*16;case Ql:return Math.floor((n+9)/10)*Math.floor((t+9)/10)*16;case tc:return Math.floor((n+11)/12)*Math.floor((t+9)/10)*16;case ec:return Math.floor((n+11)/12)*Math.floor((t+11)/12)*16;case ko:case nc:case ic:return Math.ceil(n/4)*Math.ceil(t/4)*16;case Rf:case sc:return Math.ceil(n/4)*Math.ceil(t/4)*8;case rc:case oc:return Math.ceil(n/4)*Math.ceil(t/4)*16}throw new Error(`Unable to determine texture byte length for ${e} format.`)}function fb(n){switch(n){case Jn:case bf:return{byteLength:1,components:1};case yr:case yf:case Dr:return{byteLength:2,components:1};case Oc:case Uc:return{byteLength:2,components:4};case Xi:case Ic:case jn:return{byteLength:4,components:1};case Mf:return{byteLength:4,components:3}}throw new Error(`Unknown texture type ${n}.`)}function pb(n,t,e,i,s,r,o){const a=t.has("WEBGL_multisampled_render_to_texture")?t.get("WEBGL_multisampled_render_to_texture"):null,l=typeof navigator>"u"?!1:/OculusBrowser/g.test(navigator.userAgent),c=new Tt,h=new WeakMap;let u;const d=new WeakMap;let f=!1;try{f=typeof OffscreenCanvas<"u"&&new OffscreenCanvas(1,1).getContext("2d")!==null}catch{}function g(C,M){return f?new OffscreenCanvas(C,M):Jo("canvas")}function _(C,M,k){let K=1;const J=wt(C);if((J.width>k||J.height>k)&&(K=k/Math.max(J.width,J.height)),K<1)if(typeof HTMLImageElement<"u"&&C instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&C instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&C instanceof ImageBitmap||typeof VideoFrame<"u"&&C instanceof VideoFrame){const Y=Math.floor(K*J.width),yt=Math.floor(K*J.height);u===void 0&&(u=g(Y,yt));const ct=M?g(Y,yt):u;return ct.width=Y,ct.height=yt,ct.getContext("2d").drawImage(C,0,0,Y,yt),console.warn("THREE.WebGLRenderer: Texture has been resized from ("+J.width+"x"+J.height+") to ("+Y+"x"+yt+")."),ct}else return"data"in C&&console.warn("THREE.WebGLRenderer: Image in DataTexture is too big ("+J.width+"x"+J.height+")."),C;return C}function m(C){return C.generateMipmaps}function p(C){n.generateMipmap(C)}function v(C){return C.isWebGLCubeRenderTarget?n.TEXTURE_CUBE_MAP:C.isWebGL3DRenderTarget?n.TEXTURE_3D:C.isWebGLArrayRenderTarget||C.isCompressedArrayTexture?n.TEXTURE_2D_ARRAY:n.TEXTURE_2D}function y(C,M,k,K,J=!1){if(C!==null){if(n[C]!==void 0)return n[C];console.warn("THREE.WebGLRenderer: Attempt to use non-existing WebGL internal format '"+C+"'")}let Y=M;if(M===n.RED&&(k===n.FLOAT&&(Y=n.R32F),k===n.HALF_FLOAT&&(Y=n.R16F),k===n.UNSIGNED_BYTE&&(Y=n.R8)),M===n.RED_INTEGER&&(k===n.UNSIGNED_BYTE&&(Y=n.R8UI),k===n.UNSIGNED_SHORT&&(Y=n.R16UI),k===n.UNSIGNED_INT&&(Y=n.R32UI),k===n.BYTE&&(Y=n.R8I),k===n.SHORT&&(Y=n.R16I),k===n.INT&&(Y=n.R32I)),M===n.RG&&(k===n.FLOAT&&(Y=n.RG32F),k===n.HALF_FLOAT&&(Y=n.RG16F),k===n.UNSIGNED_BYTE&&(Y=n.RG8)),M===n.RG_INTEGER&&(k===n.UNSIGNED_BYTE&&(Y=n.RG8UI),k===n.UNSIGNED_SHORT&&(Y=n.RG16UI),k===n.UNSIGNED_INT&&(Y=n.RG32UI),k===n.BYTE&&(Y=n.RG8I),k===n.SHORT&&(Y=n.RG16I),k===n.INT&&(Y=n.RG32I)),M===n.RGB_INTEGER&&(k===n.UNSIGNED_BYTE&&(Y=n.RGB8UI),k===n.UNSIGNED_SHORT&&(Y=n.RGB16UI),k===n.UNSIGNED_INT&&(Y=n.RGB32UI),k===n.BYTE&&(Y=n.RGB8I),k===n.SHORT&&(Y=n.RGB16I),k===n.INT&&(Y=n.RGB32I)),M===n.RGBA_INTEGER&&(k===n.UNSIGNED_BYTE&&(Y=n.RGBA8UI),k===n.UNSIGNED_SHORT&&(Y=n.RGBA16UI),k===n.UNSIGNED_INT&&(Y=n.RGBA32UI),k===n.BYTE&&(Y=n.RGBA8I),k===n.SHORT&&(Y=n.RGBA16I),k===n.INT&&(Y=n.RGBA32I)),M===n.RGB&&k===n.UNSIGNED_INT_5_9_9_9_REV&&(Y=n.RGB9_E5),M===n.RGBA){const yt=J?da:Kt.getTransfer(K);k===n.FLOAT&&(Y=n.RGBA32F),k===n.HALF_FLOAT&&(Y=n.RGBA16F),k===n.UNSIGNED_BYTE&&(Y=yt===ie?n.SRGB8_ALPHA8:n.RGBA8),k===n.UNSIGNED_SHORT_4_4_4_4&&(Y=n.RGBA4),k===n.UNSIGNED_SHORT_5_5_5_1&&(Y=n.RGB5_A1)}return(Y===n.R16F||Y===n.R32F||Y===n.RG16F||Y===n.RG32F||Y===n.RGBA16F||Y===n.RGBA32F)&&t.get("EXT_color_buffer_float"),Y}function x(C,M){let k;return C?M===null||M===Xi||M===Os?k=n.DEPTH24_STENCIL8:M===jn?k=n.DEPTH32F_STENCIL8:M===yr&&(k=n.DEPTH24_STENCIL8,console.warn("DepthTexture: 16 bit depth attachment is not supported with stencil. Using 24-bit attachment.")):M===null||M===Xi||M===Os?k=n.DEPTH_COMPONENT24:M===jn?k=n.DEPTH_COMPONENT32F:M===yr&&(k=n.DEPTH_COMPONENT16),k}function A(C,M){return m(C)===!0||C.isFramebufferTexture&&C.minFilter!==xn&&C.minFilter!==Cn?Math.log2(Math.max(M.width,M.height))+1:C.mipmaps!==void 0&&C.mipmaps.length>0?C.mipmaps.length:C.isCompressedTexture&&Array.isArray(C.image)?M.mipmaps.length:1}function w(C){const M=C.target;M.removeEventListener("dispose",w),R(M),M.isVideoTexture&&h.delete(M)}function T(C){const M=C.target;M.removeEventListener("dispose",T),b(M)}function R(C){const M=i.get(C);if(M.__webglInit===void 0)return;const k=C.source,K=d.get(k);if(K){const J=K[M.__cacheKey];J.usedTimes--,J.usedTimes===0&&S(C),Object.keys(K).length===0&&d.delete(k)}i.remove(C)}function S(C){const M=i.get(C);n.deleteTexture(M.__webglTexture);const k=C.source,K=d.get(k);delete K[M.__cacheKey],o.memory.textures--}function b(C){const M=i.get(C);if(C.depthTexture&&(C.depthTexture.dispose(),i.remove(C.depthTexture)),C.isWebGLCubeRenderTarget)for(let K=0;K<6;K++){if(Array.isArray(M.__webglFramebuffer[K]))for(let J=0;J<M.__webglFramebuffer[K].length;J++)n.deleteFramebuffer(M.__webglFramebuffer[K][J]);else n.deleteFramebuffer(M.__webglFramebuffer[K]);M.__webglDepthbuffer&&n.deleteRenderbuffer(M.__webglDepthbuffer[K])}else{if(Array.isArray(M.__webglFramebuffer))for(let K=0;K<M.__webglFramebuffer.length;K++)n.deleteFramebuffer(M.__webglFramebuffer[K]);else n.deleteFramebuffer(M.__webglFramebuffer);if(M.__webglDepthbuffer&&n.deleteRenderbuffer(M.__webglDepthbuffer),M.__webglMultisampledFramebuffer&&n.deleteFramebuffer(M.__webglMultisampledFramebuffer),M.__webglColorRenderbuffer)for(let K=0;K<M.__webglColorRenderbuffer.length;K++)M.__webglColorRenderbuffer[K]&&n.deleteRenderbuffer(M.__webglColorRenderbuffer[K]);M.__webglDepthRenderbuffer&&n.deleteRenderbuffer(M.__webglDepthRenderbuffer)}const k=C.textures;for(let K=0,J=k.length;K<J;K++){const Y=i.get(k[K]);Y.__webglTexture&&(n.deleteTexture(Y.__webglTexture),o.memory.textures--),i.remove(k[K])}i.remove(C)}let P=0;function N(){P=0}function F(){const C=P;return C>=s.maxTextures&&console.warn("THREE.WebGLTextures: Trying to use "+C+" texture units while this GPU supports only "+s.maxTextures),P+=1,C}function W(C){const M=[];return M.push(C.wrapS),M.push(C.wrapT),M.push(C.wrapR||0),M.push(C.magFilter),M.push(C.minFilter),M.push(C.anisotropy),M.push(C.internalFormat),M.push(C.format),M.push(C.type),M.push(C.generateMipmaps),M.push(C.premultiplyAlpha),M.push(C.flipY),M.push(C.unpackAlignment),M.push(C.colorSpace),M.join()}function G(C,M){const k=i.get(C);if(C.isVideoTexture&&At(C),C.isRenderTargetTexture===!1&&C.version>0&&k.__version!==C.version){const K=C.image;if(K===null)console.warn("THREE.WebGLRenderer: Texture marked for update but no image data found.");else if(K.complete===!1)console.warn("THREE.WebGLRenderer: Texture marked for update but image is incomplete");else{j(k,C,M);return}}e.bindTexture(n.TEXTURE_2D,k.__webglTexture,n.TEXTURE0+M)}function H(C,M){const k=i.get(C);if(C.version>0&&k.__version!==C.version){j(k,C,M);return}e.bindTexture(n.TEXTURE_2D_ARRAY,k.__webglTexture,n.TEXTURE0+M)}function $(C,M){const k=i.get(C);if(C.version>0&&k.__version!==C.version){j(k,C,M);return}e.bindTexture(n.TEXTURE_3D,k.__webglTexture,n.TEXTURE0+M)}function V(C,M){const k=i.get(C);if(C.version>0&&k.__version!==C.version){tt(k,C,M);return}e.bindTexture(n.TEXTURE_CUBE_MAP,k.__webglTexture,n.TEXTURE0+M)}const et={[Il]:n.REPEAT,[Bi]:n.CLAMP_TO_EDGE,[Ol]:n.MIRRORED_REPEAT},at={[xn]:n.NEAREST,[tg]:n.NEAREST_MIPMAP_NEAREST,[Vr]:n.NEAREST_MIPMAP_LINEAR,[Cn]:n.LINEAR,[Ca]:n.LINEAR_MIPMAP_NEAREST,[zi]:n.LINEAR_MIPMAP_LINEAR},_t={[sg]:n.NEVER,[hg]:n.ALWAYS,[rg]:n.LESS,[Df]:n.LEQUAL,[og]:n.EQUAL,[cg]:n.GEQUAL,[ag]:n.GREATER,[lg]:n.NOTEQUAL};function Dt(C,M){if(M.type===jn&&t.has("OES_texture_float_linear")===!1&&(M.magFilter===Cn||M.magFilter===Ca||M.magFilter===Vr||M.magFilter===zi||M.minFilter===Cn||M.minFilter===Ca||M.minFilter===Vr||M.minFilter===zi)&&console.warn("THREE.WebGLRenderer: Unable to use linear filtering with floating point textures. OES_texture_float_linear not supported on this device."),n.texParameteri(C,n.TEXTURE_WRAP_S,et[M.wrapS]),n.texParameteri(C,n.TEXTURE_WRAP_T,et[M.wrapT]),(C===n.TEXTURE_3D||C===n.TEXTURE_2D_ARRAY)&&n.texParameteri(C,n.TEXTURE_WRAP_R,et[M.wrapR]),n.texParameteri(C,n.TEXTURE_MAG_FILTER,at[M.magFilter]),n.texParameteri(C,n.TEXTURE_MIN_FILTER,at[M.minFilter]),M.compareFunction&&(n.texParameteri(C,n.TEXTURE_COMPARE_MODE,n.COMPARE_REF_TO_TEXTURE),n.texParameteri(C,n.TEXTURE_COMPARE_FUNC,_t[M.compareFunction])),t.has("EXT_texture_filter_anisotropic")===!0){if(M.magFilter===xn||M.minFilter!==Vr&&M.minFilter!==zi||M.type===jn&&t.has("OES_texture_float_linear")===!1)return;if(M.anisotropy>1||i.get(M).__currentAnisotropy){const k=t.get("EXT_texture_filter_anisotropic");n.texParameterf(C,k.TEXTURE_MAX_ANISOTROPY_EXT,Math.min(M.anisotropy,s.getMaxAnisotropy())),i.get(M).__currentAnisotropy=M.anisotropy}}}function $t(C,M){let k=!1;C.__webglInit===void 0&&(C.__webglInit=!0,M.addEventListener("dispose",w));const K=M.source;let J=d.get(K);J===void 0&&(J={},d.set(K,J));const Y=W(M);if(Y!==C.__cacheKey){J[Y]===void 0&&(J[Y]={texture:n.createTexture(),usedTimes:0},o.memory.textures++,k=!0),J[Y].usedTimes++;const yt=J[C.__cacheKey];yt!==void 0&&(J[C.__cacheKey].usedTimes--,yt.usedTimes===0&&S(M)),C.__cacheKey=Y,C.__webglTexture=J[Y].texture}return k}function j(C,M,k){let K=n.TEXTURE_2D;(M.isDataArrayTexture||M.isCompressedArrayTexture)&&(K=n.TEXTURE_2D_ARRAY),M.isData3DTexture&&(K=n.TEXTURE_3D);const J=$t(C,M),Y=M.source;e.bindTexture(K,C.__webglTexture,n.TEXTURE0+k);const yt=i.get(Y);if(Y.version!==yt.__version||J===!0){e.activeTexture(n.TEXTURE0+k);const ct=Kt.getPrimaries(Kt.workingColorSpace),pt=M.colorSpace===li?null:Kt.getPrimaries(M.colorSpace),qt=M.colorSpace===li||ct===pt?n.NONE:n.BROWSER_DEFAULT_WEBGL;n.pixelStorei(n.UNPACK_FLIP_Y_WEBGL,M.flipY),n.pixelStorei(n.UNPACK_PREMULTIPLY_ALPHA_WEBGL,M.premultiplyAlpha),n.pixelStorei(n.UNPACK_ALIGNMENT,M.unpackAlignment),n.pixelStorei(n.UNPACK_COLORSPACE_CONVERSION_WEBGL,qt);let nt=_(M.image,!1,s.maxTextureSize);nt=he(M,nt);const mt=r.convert(M.format,M.colorSpace),Ct=r.convert(M.type);let Pt=y(M.internalFormat,mt,Ct,M.colorSpace,M.isVideoTexture);Dt(K,M);let gt;const jt=M.mipmaps,Bt=M.isVideoTexture!==!0,ae=yt.__version===void 0||J===!0,D=Y.dataReady,ot=A(M,nt);if(M.isDepthTexture)Pt=x(M.format===Us,M.type),ae&&(Bt?e.texStorage2D(n.TEXTURE_2D,1,Pt,nt.width,nt.height):e.texImage2D(n.TEXTURE_2D,0,Pt,nt.width,nt.height,0,mt,Ct,null));else if(M.isDataTexture)if(jt.length>0){Bt&&ae&&e.texStorage2D(n.TEXTURE_2D,ot,Pt,jt[0].width,jt[0].height);for(let X=0,Z=jt.length;X<Z;X++)gt=jt[X],Bt?D&&e.texSubImage2D(n.TEXTURE_2D,X,0,0,gt.width,gt.height,mt,Ct,gt.data):e.texImage2D(n.TEXTURE_2D,X,Pt,gt.width,gt.height,0,mt,Ct,gt.data);M.generateMipmaps=!1}else Bt?(ae&&e.texStorage2D(n.TEXTURE_2D,ot,Pt,nt.width,nt.height),D&&e.texSubImage2D(n.TEXTURE_2D,0,0,0,nt.width,nt.height,mt,Ct,nt.data)):e.texImage2D(n.TEXTURE_2D,0,Pt,nt.width,nt.height,0,mt,Ct,nt.data);else if(M.isCompressedTexture)if(M.isCompressedArrayTexture){Bt&&ae&&e.texStorage3D(n.TEXTURE_2D_ARRAY,ot,Pt,jt[0].width,jt[0].height,nt.depth);for(let X=0,Z=jt.length;X<Z;X++)if(gt=jt[X],M.format!==mn)if(mt!==null)if(Bt){if(D)if(M.layerUpdates.size>0){const dt=xu(gt.width,gt.height,M.format,M.type);for(const ht of M.layerUpdates){const Ut=gt.data.subarray(ht*dt/gt.data.BYTES_PER_ELEMENT,(ht+1)*dt/gt.data.BYTES_PER_ELEMENT);e.compressedTexSubImage3D(n.TEXTURE_2D_ARRAY,X,0,0,ht,gt.width,gt.height,1,mt,Ut)}M.clearLayerUpdates()}else e.compressedTexSubImage3D(n.TEXTURE_2D_ARRAY,X,0,0,0,gt.width,gt.height,nt.depth,mt,gt.data)}else e.compressedTexImage3D(n.TEXTURE_2D_ARRAY,X,Pt,gt.width,gt.height,nt.depth,0,gt.data,0,0);else console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .uploadTexture()");else Bt?D&&e.texSubImage3D(n.TEXTURE_2D_ARRAY,X,0,0,0,gt.width,gt.height,nt.depth,mt,Ct,gt.data):e.texImage3D(n.TEXTURE_2D_ARRAY,X,Pt,gt.width,gt.height,nt.depth,0,mt,Ct,gt.data)}else{Bt&&ae&&e.texStorage2D(n.TEXTURE_2D,ot,Pt,jt[0].width,jt[0].height);for(let X=0,Z=jt.length;X<Z;X++)gt=jt[X],M.format!==mn?mt!==null?Bt?D&&e.compressedTexSubImage2D(n.TEXTURE_2D,X,0,0,gt.width,gt.height,mt,gt.data):e.compressedTexImage2D(n.TEXTURE_2D,X,Pt,gt.width,gt.height,0,gt.data):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .uploadTexture()"):Bt?D&&e.texSubImage2D(n.TEXTURE_2D,X,0,0,gt.width,gt.height,mt,Ct,gt.data):e.texImage2D(n.TEXTURE_2D,X,Pt,gt.width,gt.height,0,mt,Ct,gt.data)}else if(M.isDataArrayTexture)if(Bt){if(ae&&e.texStorage3D(n.TEXTURE_2D_ARRAY,ot,Pt,nt.width,nt.height,nt.depth),D)if(M.layerUpdates.size>0){const X=xu(nt.width,nt.height,M.format,M.type);for(const Z of M.layerUpdates){const dt=nt.data.subarray(Z*X/nt.data.BYTES_PER_ELEMENT,(Z+1)*X/nt.data.BYTES_PER_ELEMENT);e.texSubImage3D(n.TEXTURE_2D_ARRAY,0,0,0,Z,nt.width,nt.height,1,mt,Ct,dt)}M.clearLayerUpdates()}else e.texSubImage3D(n.TEXTURE_2D_ARRAY,0,0,0,0,nt.width,nt.height,nt.depth,mt,Ct,nt.data)}else e.texImage3D(n.TEXTURE_2D_ARRAY,0,Pt,nt.width,nt.height,nt.depth,0,mt,Ct,nt.data);else if(M.isData3DTexture)Bt?(ae&&e.texStorage3D(n.TEXTURE_3D,ot,Pt,nt.width,nt.height,nt.depth),D&&e.texSubImage3D(n.TEXTURE_3D,0,0,0,0,nt.width,nt.height,nt.depth,mt,Ct,nt.data)):e.texImage3D(n.TEXTURE_3D,0,Pt,nt.width,nt.height,nt.depth,0,mt,Ct,nt.data);else if(M.isFramebufferTexture){if(ae)if(Bt)e.texStorage2D(n.TEXTURE_2D,ot,Pt,nt.width,nt.height);else{let X=nt.width,Z=nt.height;for(let dt=0;dt<ot;dt++)e.texImage2D(n.TEXTURE_2D,dt,Pt,X,Z,0,mt,Ct,null),X>>=1,Z>>=1}}else if(jt.length>0){if(Bt&&ae){const X=wt(jt[0]);e.texStorage2D(n.TEXTURE_2D,ot,Pt,X.width,X.height)}for(let X=0,Z=jt.length;X<Z;X++)gt=jt[X],Bt?D&&e.texSubImage2D(n.TEXTURE_2D,X,0,0,mt,Ct,gt):e.texImage2D(n.TEXTURE_2D,X,Pt,mt,Ct,gt);M.generateMipmaps=!1}else if(Bt){if(ae){const X=wt(nt);e.texStorage2D(n.TEXTURE_2D,ot,Pt,X.width,X.height)}D&&e.texSubImage2D(n.TEXTURE_2D,0,0,0,mt,Ct,nt)}else e.texImage2D(n.TEXTURE_2D,0,Pt,mt,Ct,nt);m(M)&&p(K),yt.__version=Y.version,M.onUpdate&&M.onUpdate(M)}C.__version=M.version}function tt(C,M,k){if(M.image.length!==6)return;const K=$t(C,M),J=M.source;e.bindTexture(n.TEXTURE_CUBE_MAP,C.__webglTexture,n.TEXTURE0+k);const Y=i.get(J);if(J.version!==Y.__version||K===!0){e.activeTexture(n.TEXTURE0+k);const yt=Kt.getPrimaries(Kt.workingColorSpace),ct=M.colorSpace===li?null:Kt.getPrimaries(M.colorSpace),pt=M.colorSpace===li||yt===ct?n.NONE:n.BROWSER_DEFAULT_WEBGL;n.pixelStorei(n.UNPACK_FLIP_Y_WEBGL,M.flipY),n.pixelStorei(n.UNPACK_PREMULTIPLY_ALPHA_WEBGL,M.premultiplyAlpha),n.pixelStorei(n.UNPACK_ALIGNMENT,M.unpackAlignment),n.pixelStorei(n.UNPACK_COLORSPACE_CONVERSION_WEBGL,pt);const qt=M.isCompressedTexture||M.image[0].isCompressedTexture,nt=M.image[0]&&M.image[0].isDataTexture,mt=[];for(let Z=0;Z<6;Z++)!qt&&!nt?mt[Z]=_(M.image[Z],!0,s.maxCubemapSize):mt[Z]=nt?M.image[Z].image:M.image[Z],mt[Z]=he(M,mt[Z]);const Ct=mt[0],Pt=r.convert(M.format,M.colorSpace),gt=r.convert(M.type),jt=y(M.internalFormat,Pt,gt,M.colorSpace),Bt=M.isVideoTexture!==!0,ae=Y.__version===void 0||K===!0,D=J.dataReady;let ot=A(M,Ct);Dt(n.TEXTURE_CUBE_MAP,M);let X;if(qt){Bt&&ae&&e.texStorage2D(n.TEXTURE_CUBE_MAP,ot,jt,Ct.width,Ct.height);for(let Z=0;Z<6;Z++){X=mt[Z].mipmaps;for(let dt=0;dt<X.length;dt++){const ht=X[dt];M.format!==mn?Pt!==null?Bt?D&&e.compressedTexSubImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+Z,dt,0,0,ht.width,ht.height,Pt,ht.data):e.compressedTexImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+Z,dt,jt,ht.width,ht.height,0,ht.data):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .setTextureCube()"):Bt?D&&e.texSubImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+Z,dt,0,0,ht.width,ht.height,Pt,gt,ht.data):e.texImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+Z,dt,jt,ht.width,ht.height,0,Pt,gt,ht.data)}}}else{if(X=M.mipmaps,Bt&&ae){X.length>0&&ot++;const Z=wt(mt[0]);e.texStorage2D(n.TEXTURE_CUBE_MAP,ot,jt,Z.width,Z.height)}for(let Z=0;Z<6;Z++)if(nt){Bt?D&&e.texSubImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+Z,0,0,0,mt[Z].width,mt[Z].height,Pt,gt,mt[Z].data):e.texImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+Z,0,jt,mt[Z].width,mt[Z].height,0,Pt,gt,mt[Z].data);for(let dt=0;dt<X.length;dt++){const Ut=X[dt].image[Z].image;Bt?D&&e.texSubImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+Z,dt+1,0,0,Ut.width,Ut.height,Pt,gt,Ut.data):e.texImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+Z,dt+1,jt,Ut.width,Ut.height,0,Pt,gt,Ut.data)}}else{Bt?D&&e.texSubImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+Z,0,0,0,Pt,gt,mt[Z]):e.texImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+Z,0,jt,Pt,gt,mt[Z]);for(let dt=0;dt<X.length;dt++){const ht=X[dt];Bt?D&&e.texSubImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+Z,dt+1,0,0,Pt,gt,ht.image[Z]):e.texImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+Z,dt+1,jt,Pt,gt,ht.image[Z])}}}m(M)&&p(n.TEXTURE_CUBE_MAP),Y.__version=J.version,M.onUpdate&&M.onUpdate(M)}C.__version=M.version}function ft(C,M,k,K,J,Y){const yt=r.convert(k.format,k.colorSpace),ct=r.convert(k.type),pt=y(k.internalFormat,yt,ct,k.colorSpace),qt=i.get(M),nt=i.get(k);if(nt.__renderTarget=M,!qt.__hasExternalTextures){const mt=Math.max(1,M.width>>Y),Ct=Math.max(1,M.height>>Y);J===n.TEXTURE_3D||J===n.TEXTURE_2D_ARRAY?e.texImage3D(J,Y,pt,mt,Ct,M.depth,0,yt,ct,null):e.texImage2D(J,Y,pt,mt,Ct,0,yt,ct,null)}e.bindFramebuffer(n.FRAMEBUFFER,C),Xt(M)?a.framebufferTexture2DMultisampleEXT(n.FRAMEBUFFER,K,J,nt.__webglTexture,0,Wt(M)):(J===n.TEXTURE_2D||J>=n.TEXTURE_CUBE_MAP_POSITIVE_X&&J<=n.TEXTURE_CUBE_MAP_NEGATIVE_Z)&&n.framebufferTexture2D(n.FRAMEBUFFER,K,J,nt.__webglTexture,Y),e.bindFramebuffer(n.FRAMEBUFFER,null)}function lt(C,M,k){if(n.bindRenderbuffer(n.RENDERBUFFER,C),M.depthBuffer){const K=M.depthTexture,J=K&&K.isDepthTexture?K.type:null,Y=x(M.stencilBuffer,J),yt=M.stencilBuffer?n.DEPTH_STENCIL_ATTACHMENT:n.DEPTH_ATTACHMENT,ct=Wt(M);Xt(M)?a.renderbufferStorageMultisampleEXT(n.RENDERBUFFER,ct,Y,M.width,M.height):k?n.renderbufferStorageMultisample(n.RENDERBUFFER,ct,Y,M.width,M.height):n.renderbufferStorage(n.RENDERBUFFER,Y,M.width,M.height),n.framebufferRenderbuffer(n.FRAMEBUFFER,yt,n.RENDERBUFFER,C)}else{const K=M.textures;for(let J=0;J<K.length;J++){const Y=K[J],yt=r.convert(Y.format,Y.colorSpace),ct=r.convert(Y.type),pt=y(Y.internalFormat,yt,ct,Y.colorSpace),qt=Wt(M);k&&Xt(M)===!1?n.renderbufferStorageMultisample(n.RENDERBUFFER,qt,pt,M.width,M.height):Xt(M)?a.renderbufferStorageMultisampleEXT(n.RENDERBUFFER,qt,pt,M.width,M.height):n.renderbufferStorage(n.RENDERBUFFER,pt,M.width,M.height)}}n.bindRenderbuffer(n.RENDERBUFFER,null)}function Rt(C,M){if(M&&M.isWebGLCubeRenderTarget)throw new Error("Depth Texture with cube render targets is not supported");if(e.bindFramebuffer(n.FRAMEBUFFER,C),!(M.depthTexture&&M.depthTexture.isDepthTexture))throw new Error("renderTarget.depthTexture must be an instance of THREE.DepthTexture");const K=i.get(M.depthTexture);K.__renderTarget=M,(!K.__webglTexture||M.depthTexture.image.width!==M.width||M.depthTexture.image.height!==M.height)&&(M.depthTexture.image.width=M.width,M.depthTexture.image.height=M.height,M.depthTexture.needsUpdate=!0),G(M.depthTexture,0);const J=K.__webglTexture,Y=Wt(M);if(M.depthTexture.format===As)Xt(M)?a.framebufferTexture2DMultisampleEXT(n.FRAMEBUFFER,n.DEPTH_ATTACHMENT,n.TEXTURE_2D,J,0,Y):n.framebufferTexture2D(n.FRAMEBUFFER,n.DEPTH_ATTACHMENT,n.TEXTURE_2D,J,0);else if(M.depthTexture.format===Us)Xt(M)?a.framebufferTexture2DMultisampleEXT(n.FRAMEBUFFER,n.DEPTH_STENCIL_ATTACHMENT,n.TEXTURE_2D,J,0,Y):n.framebufferTexture2D(n.FRAMEBUFFER,n.DEPTH_STENCIL_ATTACHMENT,n.TEXTURE_2D,J,0);else throw new Error("Unknown depthTexture format")}function It(C){const M=i.get(C),k=C.isWebGLCubeRenderTarget===!0;if(M.__boundDepthTexture!==C.depthTexture){const K=C.depthTexture;if(M.__depthDisposeCallback&&M.__depthDisposeCallback(),K){const J=()=>{delete M.__boundDepthTexture,delete M.__depthDisposeCallback,K.removeEventListener("dispose",J)};K.addEventListener("dispose",J),M.__depthDisposeCallback=J}M.__boundDepthTexture=K}if(C.depthTexture&&!M.__autoAllocateDepthBuffer){if(k)throw new Error("target.depthTexture not supported in Cube render targets");Rt(M.__webglFramebuffer,C)}else if(k){M.__webglDepthbuffer=[];for(let K=0;K<6;K++)if(e.bindFramebuffer(n.FRAMEBUFFER,M.__webglFramebuffer[K]),M.__webglDepthbuffer[K]===void 0)M.__webglDepthbuffer[K]=n.createRenderbuffer(),lt(M.__webglDepthbuffer[K],C,!1);else{const J=C.stencilBuffer?n.DEPTH_STENCIL_ATTACHMENT:n.DEPTH_ATTACHMENT,Y=M.__webglDepthbuffer[K];n.bindRenderbuffer(n.RENDERBUFFER,Y),n.framebufferRenderbuffer(n.FRAMEBUFFER,J,n.RENDERBUFFER,Y)}}else if(e.bindFramebuffer(n.FRAMEBUFFER,M.__webglFramebuffer),M.__webglDepthbuffer===void 0)M.__webglDepthbuffer=n.createRenderbuffer(),lt(M.__webglDepthbuffer,C,!1);else{const K=C.stencilBuffer?n.DEPTH_STENCIL_ATTACHMENT:n.DEPTH_ATTACHMENT,J=M.__webglDepthbuffer;n.bindRenderbuffer(n.RENDERBUFFER,J),n.framebufferRenderbuffer(n.FRAMEBUFFER,K,n.RENDERBUFFER,J)}e.bindFramebuffer(n.FRAMEBUFFER,null)}function Vt(C,M,k){const K=i.get(C);M!==void 0&&ft(K.__webglFramebuffer,C,C.texture,n.COLOR_ATTACHMENT0,n.TEXTURE_2D,0),k!==void 0&&It(C)}function _e(C){const M=C.texture,k=i.get(C),K=i.get(M);C.addEventListener("dispose",T);const J=C.textures,Y=C.isWebGLCubeRenderTarget===!0,yt=J.length>1;if(yt||(K.__webglTexture===void 0&&(K.__webglTexture=n.createTexture()),K.__version=M.version,o.memory.textures++),Y){k.__webglFramebuffer=[];for(let ct=0;ct<6;ct++)if(M.mipmaps&&M.mipmaps.length>0){k.__webglFramebuffer[ct]=[];for(let pt=0;pt<M.mipmaps.length;pt++)k.__webglFramebuffer[ct][pt]=n.createFramebuffer()}else k.__webglFramebuffer[ct]=n.createFramebuffer()}else{if(M.mipmaps&&M.mipmaps.length>0){k.__webglFramebuffer=[];for(let ct=0;ct<M.mipmaps.length;ct++)k.__webglFramebuffer[ct]=n.createFramebuffer()}else k.__webglFramebuffer=n.createFramebuffer();if(yt)for(let ct=0,pt=J.length;ct<pt;ct++){const qt=i.get(J[ct]);qt.__webglTexture===void 0&&(qt.__webglTexture=n.createTexture(),o.memory.textures++)}if(C.samples>0&&Xt(C)===!1){k.__webglMultisampledFramebuffer=n.createFramebuffer(),k.__webglColorRenderbuffer=[],e.bindFramebuffer(n.FRAMEBUFFER,k.__webglMultisampledFramebuffer);for(let ct=0;ct<J.length;ct++){const pt=J[ct];k.__webglColorRenderbuffer[ct]=n.createRenderbuffer(),n.bindRenderbuffer(n.RENDERBUFFER,k.__webglColorRenderbuffer[ct]);const qt=r.convert(pt.format,pt.colorSpace),nt=r.convert(pt.type),mt=y(pt.internalFormat,qt,nt,pt.colorSpace,C.isXRRenderTarget===!0),Ct=Wt(C);n.renderbufferStorageMultisample(n.RENDERBUFFER,Ct,mt,C.width,C.height),n.framebufferRenderbuffer(n.FRAMEBUFFER,n.COLOR_ATTACHMENT0+ct,n.RENDERBUFFER,k.__webglColorRenderbuffer[ct])}n.bindRenderbuffer(n.RENDERBUFFER,null),C.depthBuffer&&(k.__webglDepthRenderbuffer=n.createRenderbuffer(),lt(k.__webglDepthRenderbuffer,C,!0)),e.bindFramebuffer(n.FRAMEBUFFER,null)}}if(Y){e.bindTexture(n.TEXTURE_CUBE_MAP,K.__webglTexture),Dt(n.TEXTURE_CUBE_MAP,M);for(let ct=0;ct<6;ct++)if(M.mipmaps&&M.mipmaps.length>0)for(let pt=0;pt<M.mipmaps.length;pt++)ft(k.__webglFramebuffer[ct][pt],C,M,n.COLOR_ATTACHMENT0,n.TEXTURE_CUBE_MAP_POSITIVE_X+ct,pt);else ft(k.__webglFramebuffer[ct],C,M,n.COLOR_ATTACHMENT0,n.TEXTURE_CUBE_MAP_POSITIVE_X+ct,0);m(M)&&p(n.TEXTURE_CUBE_MAP),e.unbindTexture()}else if(yt){for(let ct=0,pt=J.length;ct<pt;ct++){const qt=J[ct],nt=i.get(qt);e.bindTexture(n.TEXTURE_2D,nt.__webglTexture),Dt(n.TEXTURE_2D,qt),ft(k.__webglFramebuffer,C,qt,n.COLOR_ATTACHMENT0+ct,n.TEXTURE_2D,0),m(qt)&&p(n.TEXTURE_2D)}e.unbindTexture()}else{let ct=n.TEXTURE_2D;if((C.isWebGL3DRenderTarget||C.isWebGLArrayRenderTarget)&&(ct=C.isWebGL3DRenderTarget?n.TEXTURE_3D:n.TEXTURE_2D_ARRAY),e.bindTexture(ct,K.__webglTexture),Dt(ct,M),M.mipmaps&&M.mipmaps.length>0)for(let pt=0;pt<M.mipmaps.length;pt++)ft(k.__webglFramebuffer[pt],C,M,n.COLOR_ATTACHMENT0,ct,pt);else ft(k.__webglFramebuffer,C,M,n.COLOR_ATTACHMENT0,ct,0);m(M)&&p(ct),e.unbindTexture()}C.depthBuffer&&It(C)}function Yt(C){const M=C.textures;for(let k=0,K=M.length;k<K;k++){const J=M[k];if(m(J)){const Y=v(C),yt=i.get(J).__webglTexture;e.bindTexture(Y,yt),p(Y),e.unbindTexture()}}}const Ee=[],U=[];function nn(C){if(C.samples>0){if(Xt(C)===!1){const M=C.textures,k=C.width,K=C.height;let J=n.COLOR_BUFFER_BIT;const Y=C.stencilBuffer?n.DEPTH_STENCIL_ATTACHMENT:n.DEPTH_ATTACHMENT,yt=i.get(C),ct=M.length>1;if(ct)for(let pt=0;pt<M.length;pt++)e.bindFramebuffer(n.FRAMEBUFFER,yt.__webglMultisampledFramebuffer),n.framebufferRenderbuffer(n.FRAMEBUFFER,n.COLOR_ATTACHMENT0+pt,n.RENDERBUFFER,null),e.bindFramebuffer(n.FRAMEBUFFER,yt.__webglFramebuffer),n.framebufferTexture2D(n.DRAW_FRAMEBUFFER,n.COLOR_ATTACHMENT0+pt,n.TEXTURE_2D,null,0);e.bindFramebuffer(n.READ_FRAMEBUFFER,yt.__webglMultisampledFramebuffer),e.bindFramebuffer(n.DRAW_FRAMEBUFFER,yt.__webglFramebuffer);for(let pt=0;pt<M.length;pt++){if(C.resolveDepthBuffer&&(C.depthBuffer&&(J|=n.DEPTH_BUFFER_BIT),C.stencilBuffer&&C.resolveStencilBuffer&&(J|=n.STENCIL_BUFFER_BIT)),ct){n.framebufferRenderbuffer(n.READ_FRAMEBUFFER,n.COLOR_ATTACHMENT0,n.RENDERBUFFER,yt.__webglColorRenderbuffer[pt]);const qt=i.get(M[pt]).__webglTexture;n.framebufferTexture2D(n.DRAW_FRAMEBUFFER,n.COLOR_ATTACHMENT0,n.TEXTURE_2D,qt,0)}n.blitFramebuffer(0,0,k,K,0,0,k,K,J,n.NEAREST),l===!0&&(Ee.length=0,U.length=0,Ee.push(n.COLOR_ATTACHMENT0+pt),C.depthBuffer&&C.resolveDepthBuffer===!1&&(Ee.push(Y),U.push(Y),n.invalidateFramebuffer(n.DRAW_FRAMEBUFFER,U)),n.invalidateFramebuffer(n.READ_FRAMEBUFFER,Ee))}if(e.bindFramebuffer(n.READ_FRAMEBUFFER,null),e.bindFramebuffer(n.DRAW_FRAMEBUFFER,null),ct)for(let pt=0;pt<M.length;pt++){e.bindFramebuffer(n.FRAMEBUFFER,yt.__webglMultisampledFramebuffer),n.framebufferRenderbuffer(n.FRAMEBUFFER,n.COLOR_ATTACHMENT0+pt,n.RENDERBUFFER,yt.__webglColorRenderbuffer[pt]);const qt=i.get(M[pt]).__webglTexture;e.bindFramebuffer(n.FRAMEBUFFER,yt.__webglFramebuffer),n.framebufferTexture2D(n.DRAW_FRAMEBUFFER,n.COLOR_ATTACHMENT0+pt,n.TEXTURE_2D,qt,0)}e.bindFramebuffer(n.DRAW_FRAMEBUFFER,yt.__webglMultisampledFramebuffer)}else if(C.depthBuffer&&C.resolveDepthBuffer===!1&&l){const M=C.stencilBuffer?n.DEPTH_STENCIL_ATTACHMENT:n.DEPTH_ATTACHMENT;n.invalidateFramebuffer(n.DRAW_FRAMEBUFFER,[M])}}}function Wt(C){return Math.min(s.maxSamples,C.samples)}function Xt(C){const M=i.get(C);return C.samples>0&&t.has("WEBGL_multisampled_render_to_texture")===!0&&M.__useRenderToTexture!==!1}function At(C){const M=o.render.frame;h.get(C)!==M&&(h.set(C,M),C.update())}function he(C,M){const k=C.colorSpace,K=C.format,J=C.type;return C.isCompressedTexture===!0||C.isVideoTexture===!0||k!==Hs&&k!==li&&(Kt.getTransfer(k)===ie?(K!==mn||J!==Jn)&&console.warn("THREE.WebGLTextures: sRGB encoded textures have to use RGBAFormat and UnsignedByteType."):console.error("THREE.WebGLTextures: Unsupported texture color space:",k)),M}function wt(C){return typeof HTMLImageElement<"u"&&C instanceof HTMLImageElement?(c.width=C.naturalWidth||C.width,c.height=C.naturalHeight||C.height):typeof VideoFrame<"u"&&C instanceof VideoFrame?(c.width=C.displayWidth,c.height=C.displayHeight):(c.width=C.width,c.height=C.height),c}this.allocateTextureUnit=F,this.resetTextureUnits=N,this.setTexture2D=G,this.setTexture2DArray=H,this.setTexture3D=$,this.setTextureCube=V,this.rebindTextures=Vt,this.setupRenderTarget=_e,this.updateRenderTargetMipmap=Yt,this.updateMultisampleRenderTarget=nn,this.setupDepthRenderbuffer=It,this.setupFrameBufferTexture=ft,this.useMultisampledRTT=Xt}function mb(n,t){function e(i,s=li){let r;const o=Kt.getTransfer(s);if(i===Jn)return n.UNSIGNED_BYTE;if(i===Oc)return n.UNSIGNED_SHORT_4_4_4_4;if(i===Uc)return n.UNSIGNED_SHORT_5_5_5_1;if(i===Mf)return n.UNSIGNED_INT_5_9_9_9_REV;if(i===bf)return n.BYTE;if(i===yf)return n.SHORT;if(i===yr)return n.UNSIGNED_SHORT;if(i===Ic)return n.INT;if(i===Xi)return n.UNSIGNED_INT;if(i===jn)return n.FLOAT;if(i===Dr)return n.HALF_FLOAT;if(i===Sf)return n.ALPHA;if(i===Ef)return n.RGB;if(i===mn)return n.RGBA;if(i===Tf)return n.LUMINANCE;if(i===wf)return n.LUMINANCE_ALPHA;if(i===As)return n.DEPTH_COMPONENT;if(i===Us)return n.DEPTH_STENCIL;if(i===Af)return n.RED;if(i===Nc)return n.RED_INTEGER;if(i===Cf)return n.RG;if(i===Fc)return n.RG_INTEGER;if(i===kc)return n.RGBA_INTEGER;if(i===Oo||i===Uo||i===No||i===Fo)if(o===ie)if(r=t.get("WEBGL_compressed_texture_s3tc_srgb"),r!==null){if(i===Oo)return r.COMPRESSED_SRGB_S3TC_DXT1_EXT;if(i===Uo)return r.COMPRESSED_SRGB_ALPHA_S3TC_DXT1_EXT;if(i===No)return r.COMPRESSED_SRGB_ALPHA_S3TC_DXT3_EXT;if(i===Fo)return r.COMPRESSED_SRGB_ALPHA_S3TC_DXT5_EXT}else return null;else if(r=t.get("WEBGL_compressed_texture_s3tc"),r!==null){if(i===Oo)return r.COMPRESSED_RGB_S3TC_DXT1_EXT;if(i===Uo)return r.COMPRESSED_RGBA_S3TC_DXT1_EXT;if(i===No)return r.COMPRESSED_RGBA_S3TC_DXT3_EXT;if(i===Fo)return r.COMPRESSED_RGBA_S3TC_DXT5_EXT}else return null;if(i===Ul||i===Nl||i===Fl||i===kl)if(r=t.get("WEBGL_compressed_texture_pvrtc"),r!==null){if(i===Ul)return r.COMPRESSED_RGB_PVRTC_4BPPV1_IMG;if(i===Nl)return r.COMPRESSED_RGB_PVRTC_2BPPV1_IMG;if(i===Fl)return r.COMPRESSED_RGBA_PVRTC_4BPPV1_IMG;if(i===kl)return r.COMPRESSED_RGBA_PVRTC_2BPPV1_IMG}else return null;if(i===Bl||i===zl||i===Hl)if(r=t.get("WEBGL_compressed_texture_etc"),r!==null){if(i===Bl||i===zl)return o===ie?r.COMPRESSED_SRGB8_ETC2:r.COMPRESSED_RGB8_ETC2;if(i===Hl)return o===ie?r.COMPRESSED_SRGB8_ALPHA8_ETC2_EAC:r.COMPRESSED_RGBA8_ETC2_EAC}else return null;if(i===Vl||i===Gl||i===Wl||i===Xl||i===jl||i===$l||i===Yl||i===ql||i===Kl||i===Zl||i===Jl||i===Ql||i===tc||i===ec)if(r=t.get("WEBGL_compressed_texture_astc"),r!==null){if(i===Vl)return o===ie?r.COMPRESSED_SRGB8_ALPHA8_ASTC_4x4_KHR:r.COMPRESSED_RGBA_ASTC_4x4_KHR;if(i===Gl)return o===ie?r.COMPRESSED_SRGB8_ALPHA8_ASTC_5x4_KHR:r.COMPRESSED_RGBA_ASTC_5x4_KHR;if(i===Wl)return o===ie?r.COMPRESSED_SRGB8_ALPHA8_ASTC_5x5_KHR:r.COMPRESSED_RGBA_ASTC_5x5_KHR;if(i===Xl)return o===ie?r.COMPRESSED_SRGB8_ALPHA8_ASTC_6x5_KHR:r.COMPRESSED_RGBA_ASTC_6x5_KHR;if(i===jl)return o===ie?r.COMPRESSED_SRGB8_ALPHA8_ASTC_6x6_KHR:r.COMPRESSED_RGBA_ASTC_6x6_KHR;if(i===$l)return o===ie?r.COMPRESSED_SRGB8_ALPHA8_ASTC_8x5_KHR:r.COMPRESSED_RGBA_ASTC_8x5_KHR;if(i===Yl)return o===ie?r.COMPRESSED_SRGB8_ALPHA8_ASTC_8x6_KHR:r.COMPRESSED_RGBA_ASTC_8x6_KHR;if(i===ql)return o===ie?r.COMPRESSED_SRGB8_ALPHA8_ASTC_8x8_KHR:r.COMPRESSED_RGBA_ASTC_8x8_KHR;if(i===Kl)return o===ie?r.COMPRESSED_SRGB8_ALPHA8_ASTC_10x5_KHR:r.COMPRESSED_RGBA_ASTC_10x5_KHR;if(i===Zl)return o===ie?r.COMPRESSED_SRGB8_ALPHA8_ASTC_10x6_KHR:r.COMPRESSED_RGBA_ASTC_10x6_KHR;if(i===Jl)return o===ie?r.COMPRESSED_SRGB8_ALPHA8_ASTC_10x8_KHR:r.COMPRESSED_RGBA_ASTC_10x8_KHR;if(i===Ql)return o===ie?r.COMPRESSED_SRGB8_ALPHA8_ASTC_10x10_KHR:r.COMPRESSED_RGBA_ASTC_10x10_KHR;if(i===tc)return o===ie?r.COMPRESSED_SRGB8_ALPHA8_ASTC_12x10_KHR:r.COMPRESSED_RGBA_ASTC_12x10_KHR;if(i===ec)return o===ie?r.COMPRESSED_SRGB8_ALPHA8_ASTC_12x12_KHR:r.COMPRESSED_RGBA_ASTC_12x12_KHR}else return null;if(i===ko||i===nc||i===ic)if(r=t.get("EXT_texture_compression_bptc"),r!==null){if(i===ko)return o===ie?r.COMPRESSED_SRGB_ALPHA_BPTC_UNORM_EXT:r.COMPRESSED_RGBA_BPTC_UNORM_EXT;if(i===nc)return r.COMPRESSED_RGB_BPTC_SIGNED_FLOAT_EXT;if(i===ic)return r.COMPRESSED_RGB_BPTC_UNSIGNED_FLOAT_EXT}else return null;if(i===Rf||i===sc||i===rc||i===oc)if(r=t.get("EXT_texture_compression_rgtc"),r!==null){if(i===ko)return r.COMPRESSED_RED_RGTC1_EXT;if(i===sc)return r.COMPRESSED_SIGNED_RED_RGTC1_EXT;if(i===rc)return r.COMPRESSED_RED_GREEN_RGTC2_EXT;if(i===oc)return r.COMPRESSED_SIGNED_RED_GREEN_RGTC2_EXT}else return null;return i===Os?n.UNSIGNED_INT_24_8:n[i]!==void 0?n[i]:null}return{convert:e}}class gb extends ln{constructor(t=[]){super(),this.isArrayCamera=!0,this.cameras=t}}class Xn extends Ce{constructor(){super(),this.isGroup=!0,this.type="Group"}}const _b={type:"move"};class nl{constructor(){this._targetRay=null,this._grip=null,this._hand=null}getHandSpace(){return this._hand===null&&(this._hand=new Xn,this._hand.matrixAutoUpdate=!1,this._hand.visible=!1,this._hand.joints={},this._hand.inputState={pinching:!1}),this._hand}getTargetRaySpace(){return this._targetRay===null&&(this._targetRay=new Xn,this._targetRay.matrixAutoUpdate=!1,this._targetRay.visible=!1,this._targetRay.hasLinearVelocity=!1,this._targetRay.linearVelocity=new L,this._targetRay.hasAngularVelocity=!1,this._targetRay.angularVelocity=new L),this._targetRay}getGripSpace(){return this._grip===null&&(this._grip=new Xn,this._grip.matrixAutoUpdate=!1,this._grip.visible=!1,this._grip.hasLinearVelocity=!1,this._grip.linearVelocity=new L,this._grip.hasAngularVelocity=!1,this._grip.angularVelocity=new L),this._grip}dispatchEvent(t){return this._targetRay!==null&&this._targetRay.dispatchEvent(t),this._grip!==null&&this._grip.dispatchEvent(t),this._hand!==null&&this._hand.dispatchEvent(t),this}connect(t){if(t&&t.hand){const e=this._hand;if(e)for(const i of t.hand.values())this._getHandJoint(e,i)}return this.dispatchEvent({type:"connected",data:t}),this}disconnect(t){return this.dispatchEvent({type:"disconnected",data:t}),this._targetRay!==null&&(this._targetRay.visible=!1),this._grip!==null&&(this._grip.visible=!1),this._hand!==null&&(this._hand.visible=!1),this}update(t,e,i){let s=null,r=null,o=null;const a=this._targetRay,l=this._grip,c=this._hand;if(t&&e.session.visibilityState!=="visible-blurred"){if(c&&t.hand){o=!0;for(const _ of t.hand.values()){const m=e.getJointPose(_,i),p=this._getHandJoint(c,_);m!==null&&(p.matrix.fromArray(m.transform.matrix),p.matrix.decompose(p.position,p.rotation,p.scale),p.matrixWorldNeedsUpdate=!0,p.jointRadius=m.radius),p.visible=m!==null}const h=c.joints["index-finger-tip"],u=c.joints["thumb-tip"],d=h.position.distanceTo(u.position),f=.02,g=.005;c.inputState.pinching&&d>f+g?(c.inputState.pinching=!1,this.dispatchEvent({type:"pinchend",handedness:t.handedness,target:this})):!c.inputState.pinching&&d<=f-g&&(c.inputState.pinching=!0,this.dispatchEvent({type:"pinchstart",handedness:t.handedness,target:this}))}else l!==null&&t.gripSpace&&(r=e.getPose(t.gripSpace,i),r!==null&&(l.matrix.fromArray(r.transform.matrix),l.matrix.decompose(l.position,l.rotation,l.scale),l.matrixWorldNeedsUpdate=!0,r.linearVelocity?(l.hasLinearVelocity=!0,l.linearVelocity.copy(r.linearVelocity)):l.hasLinearVelocity=!1,r.angularVelocity?(l.hasAngularVelocity=!0,l.angularVelocity.copy(r.angularVelocity)):l.hasAngularVelocity=!1));a!==null&&(s=e.getPose(t.targetRaySpace,i),s===null&&r!==null&&(s=r),s!==null&&(a.matrix.fromArray(s.transform.matrix),a.matrix.decompose(a.position,a.rotation,a.scale),a.matrixWorldNeedsUpdate=!0,s.linearVelocity?(a.hasLinearVelocity=!0,a.linearVelocity.copy(s.linearVelocity)):a.hasLinearVelocity=!1,s.angularVelocity?(a.hasAngularVelocity=!0,a.angularVelocity.copy(s.angularVelocity)):a.hasAngularVelocity=!1,this.dispatchEvent(_b)))}return a!==null&&(a.visible=s!==null),l!==null&&(l.visible=r!==null),c!==null&&(c.visible=o!==null),this}_getHandJoint(t,e){if(t.joints[e.jointName]===void 0){const i=new Xn;i.matrixAutoUpdate=!1,i.visible=!1,t.joints[e.jointName]=i,t.add(i)}return t.joints[e.jointName]}}const xb=`
void main() {

	gl_Position = vec4( position, 1.0 );

}`,vb=`
uniform sampler2DArray depthColor;
uniform float depthWidth;
uniform float depthHeight;

void main() {

	vec2 coord = vec2( gl_FragCoord.x / depthWidth, gl_FragCoord.y / depthHeight );

	if ( coord.x >= 1.0 ) {

		gl_FragDepth = texture( depthColor, vec3( coord.x - 1.0, coord.y, 1 ) ).r;

	} else {

		gl_FragDepth = texture( depthColor, vec3( coord.x, coord.y, 0 ) ).r;

	}

}`;class bb{constructor(){this.texture=null,this.mesh=null,this.depthNear=0,this.depthFar=0}init(t,e,i){if(this.texture===null){const s=new We,r=t.properties.get(s);r.__webglTexture=e.texture,(e.depthNear!=i.depthNear||e.depthFar!=i.depthFar)&&(this.depthNear=e.depthNear,this.depthFar=e.depthFar),this.texture=s}}getMesh(t){if(this.texture!==null&&this.mesh===null){const e=t.cameras[0].viewport,i=new xi({vertexShader:xb,fragmentShader:vb,uniforms:{depthColor:{value:this.texture},depthWidth:{value:e.z},depthHeight:{value:e.w}}});this.mesh=new xe(new Ur(20,20),i)}return this.mesh}reset(){this.texture=null,this.mesh=null}getDepthTexture(){return this.texture}}class yb extends Ki{constructor(t,e){super();const i=this;let s=null,r=1,o=null,a="local-floor",l=1,c=null,h=null,u=null,d=null,f=null,g=null;const _=new bb,m=e.getContextAttributes();let p=null,v=null;const y=[],x=[],A=new Tt;let w=null;const T=new ln;T.viewport=new be;const R=new ln;R.viewport=new be;const S=[T,R],b=new gb;let P=null,N=null;this.cameraAutoUpdate=!0,this.enabled=!1,this.isPresenting=!1,this.getController=function(j){let tt=y[j];return tt===void 0&&(tt=new nl,y[j]=tt),tt.getTargetRaySpace()},this.getControllerGrip=function(j){let tt=y[j];return tt===void 0&&(tt=new nl,y[j]=tt),tt.getGripSpace()},this.getHand=function(j){let tt=y[j];return tt===void 0&&(tt=new nl,y[j]=tt),tt.getHandSpace()};function F(j){const tt=x.indexOf(j.inputSource);if(tt===-1)return;const ft=y[tt];ft!==void 0&&(ft.update(j.inputSource,j.frame,c||o),ft.dispatchEvent({type:j.type,data:j.inputSource}))}function W(){s.removeEventListener("select",F),s.removeEventListener("selectstart",F),s.removeEventListener("selectend",F),s.removeEventListener("squeeze",F),s.removeEventListener("squeezestart",F),s.removeEventListener("squeezeend",F),s.removeEventListener("end",W),s.removeEventListener("inputsourceschange",G);for(let j=0;j<y.length;j++){const tt=x[j];tt!==null&&(x[j]=null,y[j].disconnect(tt))}P=null,N=null,_.reset(),t.setRenderTarget(p),f=null,d=null,u=null,s=null,v=null,$t.stop(),i.isPresenting=!1,t.setPixelRatio(w),t.setSize(A.width,A.height,!1),i.dispatchEvent({type:"sessionend"})}this.setFramebufferScaleFactor=function(j){r=j,i.isPresenting===!0&&console.warn("THREE.WebXRManager: Cannot change framebuffer scale while presenting.")},this.setReferenceSpaceType=function(j){a=j,i.isPresenting===!0&&console.warn("THREE.WebXRManager: Cannot change reference space type while presenting.")},this.getReferenceSpace=function(){return c||o},this.setReferenceSpace=function(j){c=j},this.getBaseLayer=function(){return d!==null?d:f},this.getBinding=function(){return u},this.getFrame=function(){return g},this.getSession=function(){return s},this.setSession=async function(j){if(s=j,s!==null){if(p=t.getRenderTarget(),s.addEventListener("select",F),s.addEventListener("selectstart",F),s.addEventListener("selectend",F),s.addEventListener("squeeze",F),s.addEventListener("squeezestart",F),s.addEventListener("squeezeend",F),s.addEventListener("end",W),s.addEventListener("inputsourceschange",G),m.xrCompatible!==!0&&await e.makeXRCompatible(),w=t.getPixelRatio(),t.getSize(A),s.renderState.layers===void 0){const tt={antialias:m.antialias,alpha:!0,depth:m.depth,stencil:m.stencil,framebufferScaleFactor:r};f=new XRWebGLLayer(s,e,tt),s.updateRenderState({baseLayer:f}),t.setPixelRatio(1),t.setSize(f.framebufferWidth,f.framebufferHeight,!1),v=new ji(f.framebufferWidth,f.framebufferHeight,{format:mn,type:Jn,colorSpace:t.outputColorSpace,stencilBuffer:m.stencil})}else{let tt=null,ft=null,lt=null;m.depth&&(lt=m.stencil?e.DEPTH24_STENCIL8:e.DEPTH_COMPONENT24,tt=m.stencil?Us:As,ft=m.stencil?Os:Xi);const Rt={colorFormat:e.RGBA8,depthFormat:lt,scaleFactor:r};u=new XRWebGLBinding(s,e),d=u.createProjectionLayer(Rt),s.updateRenderState({layers:[d]}),t.setPixelRatio(1),t.setSize(d.textureWidth,d.textureHeight,!1),v=new ji(d.textureWidth,d.textureHeight,{format:mn,type:Jn,depthTexture:new Gf(d.textureWidth,d.textureHeight,ft,void 0,void 0,void 0,void 0,void 0,void 0,tt),stencilBuffer:m.stencil,colorSpace:t.outputColorSpace,samples:m.antialias?4:0,resolveDepthBuffer:d.ignoreDepthValues===!1})}v.isXRRenderTarget=!0,this.setFoveation(l),c=null,o=await s.requestReferenceSpace(a),$t.setContext(s),$t.start(),i.isPresenting=!0,i.dispatchEvent({type:"sessionstart"})}},this.getEnvironmentBlendMode=function(){if(s!==null)return s.environmentBlendMode},this.getDepthTexture=function(){return _.getDepthTexture()};function G(j){for(let tt=0;tt<j.removed.length;tt++){const ft=j.removed[tt],lt=x.indexOf(ft);lt>=0&&(x[lt]=null,y[lt].disconnect(ft))}for(let tt=0;tt<j.added.length;tt++){const ft=j.added[tt];let lt=x.indexOf(ft);if(lt===-1){for(let It=0;It<y.length;It++)if(It>=x.length){x.push(ft),lt=It;break}else if(x[It]===null){x[It]=ft,lt=It;break}if(lt===-1)break}const Rt=y[lt];Rt&&Rt.connect(ft)}}const H=new L,$=new L;function V(j,tt,ft){H.setFromMatrixPosition(tt.matrixWorld),$.setFromMatrixPosition(ft.matrixWorld);const lt=H.distanceTo($),Rt=tt.projectionMatrix.elements,It=ft.projectionMatrix.elements,Vt=Rt[14]/(Rt[10]-1),_e=Rt[14]/(Rt[10]+1),Yt=(Rt[9]+1)/Rt[5],Ee=(Rt[9]-1)/Rt[5],U=(Rt[8]-1)/Rt[0],nn=(It[8]+1)/It[0],Wt=Vt*U,Xt=Vt*nn,At=lt/(-U+nn),he=At*-U;if(tt.matrixWorld.decompose(j.position,j.quaternion,j.scale),j.translateX(he),j.translateZ(At),j.matrixWorld.compose(j.position,j.quaternion,j.scale),j.matrixWorldInverse.copy(j.matrixWorld).invert(),Rt[10]===-1)j.projectionMatrix.copy(tt.projectionMatrix),j.projectionMatrixInverse.copy(tt.projectionMatrixInverse);else{const wt=Vt+At,C=_e+At,M=Wt-he,k=Xt+(lt-he),K=Yt*_e/C*wt,J=Ee*_e/C*wt;j.projectionMatrix.makePerspective(M,k,K,J,wt,C),j.projectionMatrixInverse.copy(j.projectionMatrix).invert()}}function et(j,tt){tt===null?j.matrixWorld.copy(j.matrix):j.matrixWorld.multiplyMatrices(tt.matrixWorld,j.matrix),j.matrixWorldInverse.copy(j.matrixWorld).invert()}this.updateCamera=function(j){if(s===null)return;let tt=j.near,ft=j.far;_.texture!==null&&(_.depthNear>0&&(tt=_.depthNear),_.depthFar>0&&(ft=_.depthFar)),b.near=R.near=T.near=tt,b.far=R.far=T.far=ft,(P!==b.near||N!==b.far)&&(s.updateRenderState({depthNear:b.near,depthFar:b.far}),P=b.near,N=b.far),T.layers.mask=j.layers.mask|2,R.layers.mask=j.layers.mask|4,b.layers.mask=T.layers.mask|R.layers.mask;const lt=j.parent,Rt=b.cameras;et(b,lt);for(let It=0;It<Rt.length;It++)et(Rt[It],lt);Rt.length===2?V(b,T,R):b.projectionMatrix.copy(T.projectionMatrix),at(j,b,lt)};function at(j,tt,ft){ft===null?j.matrix.copy(tt.matrixWorld):(j.matrix.copy(ft.matrixWorld),j.matrix.invert(),j.matrix.multiply(tt.matrixWorld)),j.matrix.decompose(j.position,j.quaternion,j.scale),j.updateMatrixWorld(!0),j.projectionMatrix.copy(tt.projectionMatrix),j.projectionMatrixInverse.copy(tt.projectionMatrixInverse),j.isPerspectiveCamera&&(j.fov=lc*2*Math.atan(1/j.projectionMatrix.elements[5]),j.zoom=1)}this.getCamera=function(){return b},this.getFoveation=function(){if(!(d===null&&f===null))return l},this.setFoveation=function(j){l=j,d!==null&&(d.fixedFoveation=j),f!==null&&f.fixedFoveation!==void 0&&(f.fixedFoveation=j)},this.hasDepthSensing=function(){return _.texture!==null},this.getDepthSensingMesh=function(){return _.getMesh(b)};let _t=null;function Dt(j,tt){if(h=tt.getViewerPose(c||o),g=tt,h!==null){const ft=h.views;f!==null&&(t.setRenderTargetFramebuffer(v,f.framebuffer),t.setRenderTarget(v));let lt=!1;ft.length!==b.cameras.length&&(b.cameras.length=0,lt=!0);for(let It=0;It<ft.length;It++){const Vt=ft[It];let _e=null;if(f!==null)_e=f.getViewport(Vt);else{const Ee=u.getViewSubImage(d,Vt);_e=Ee.viewport,It===0&&(t.setRenderTargetTextures(v,Ee.colorTexture,d.ignoreDepthValues?void 0:Ee.depthStencilTexture),t.setRenderTarget(v))}let Yt=S[It];Yt===void 0&&(Yt=new ln,Yt.layers.enable(It),Yt.viewport=new be,S[It]=Yt),Yt.matrix.fromArray(Vt.transform.matrix),Yt.matrix.decompose(Yt.position,Yt.quaternion,Yt.scale),Yt.projectionMatrix.fromArray(Vt.projectionMatrix),Yt.projectionMatrixInverse.copy(Yt.projectionMatrix).invert(),Yt.viewport.set(_e.x,_e.y,_e.width,_e.height),It===0&&(b.matrix.copy(Yt.matrix),b.matrix.decompose(b.position,b.quaternion,b.scale)),lt===!0&&b.cameras.push(Yt)}const Rt=s.enabledFeatures;if(Rt&&Rt.includes("depth-sensing")){const It=u.getDepthInformation(ft[0]);It&&It.isValid&&It.texture&&_.init(t,It,s.renderState)}}for(let ft=0;ft<y.length;ft++){const lt=x[ft],Rt=y[ft];lt!==null&&Rt!==void 0&&Rt.update(lt,tt,c||o)}_t&&_t(j,tt),tt.detectedPlanes&&i.dispatchEvent({type:"planesdetected",data:tt}),g=null}const $t=new Hf;$t.setAnimationLoop(Dt),this.setAnimationLoop=function(j){_t=j},this.dispose=function(){}}}const Ri=new Pn,Mb=new oe;function Sb(n,t){function e(m,p){m.matrixAutoUpdate===!0&&m.updateMatrix(),p.value.copy(m.matrix)}function i(m,p){p.color.getRGB(m.fogColor.value,kf(n)),p.isFog?(m.fogNear.value=p.near,m.fogFar.value=p.far):p.isFogExp2&&(m.fogDensity.value=p.density)}function s(m,p,v,y,x){p.isMeshBasicMaterial||p.isMeshLambertMaterial?r(m,p):p.isMeshToonMaterial?(r(m,p),u(m,p)):p.isMeshPhongMaterial?(r(m,p),h(m,p)):p.isMeshStandardMaterial?(r(m,p),d(m,p),p.isMeshPhysicalMaterial&&f(m,p,x)):p.isMeshMatcapMaterial?(r(m,p),g(m,p)):p.isMeshDepthMaterial?r(m,p):p.isMeshDistanceMaterial?(r(m,p),_(m,p)):p.isMeshNormalMaterial?r(m,p):p.isLineBasicMaterial?(o(m,p),p.isLineDashedMaterial&&a(m,p)):p.isPointsMaterial?l(m,p,v,y):p.isSpriteMaterial?c(m,p):p.isShadowMaterial?(m.color.value.copy(p.color),m.opacity.value=p.opacity):p.isShaderMaterial&&(p.uniformsNeedUpdate=!1)}function r(m,p){m.opacity.value=p.opacity,p.color&&m.diffuse.value.copy(p.color),p.emissive&&m.emissive.value.copy(p.emissive).multiplyScalar(p.emissiveIntensity),p.map&&(m.map.value=p.map,e(p.map,m.mapTransform)),p.alphaMap&&(m.alphaMap.value=p.alphaMap,e(p.alphaMap,m.alphaMapTransform)),p.bumpMap&&(m.bumpMap.value=p.bumpMap,e(p.bumpMap,m.bumpMapTransform),m.bumpScale.value=p.bumpScale,p.side===qe&&(m.bumpScale.value*=-1)),p.normalMap&&(m.normalMap.value=p.normalMap,e(p.normalMap,m.normalMapTransform),m.normalScale.value.copy(p.normalScale),p.side===qe&&m.normalScale.value.negate()),p.displacementMap&&(m.displacementMap.value=p.displacementMap,e(p.displacementMap,m.displacementMapTransform),m.displacementScale.value=p.displacementScale,m.displacementBias.value=p.displacementBias),p.emissiveMap&&(m.emissiveMap.value=p.emissiveMap,e(p.emissiveMap,m.emissiveMapTransform)),p.specularMap&&(m.specularMap.value=p.specularMap,e(p.specularMap,m.specularMapTransform)),p.alphaTest>0&&(m.alphaTest.value=p.alphaTest);const v=t.get(p),y=v.envMap,x=v.envMapRotation;y&&(m.envMap.value=y,Ri.copy(x),Ri.x*=-1,Ri.y*=-1,Ri.z*=-1,y.isCubeTexture&&y.isRenderTargetTexture===!1&&(Ri.y*=-1,Ri.z*=-1),m.envMapRotation.value.setFromMatrix4(Mb.makeRotationFromEuler(Ri)),m.flipEnvMap.value=y.isCubeTexture&&y.isRenderTargetTexture===!1?-1:1,m.reflectivity.value=p.reflectivity,m.ior.value=p.ior,m.refractionRatio.value=p.refractionRatio),p.lightMap&&(m.lightMap.value=p.lightMap,m.lightMapIntensity.value=p.lightMapIntensity,e(p.lightMap,m.lightMapTransform)),p.aoMap&&(m.aoMap.value=p.aoMap,m.aoMapIntensity.value=p.aoMapIntensity,e(p.aoMap,m.aoMapTransform))}function o(m,p){m.diffuse.value.copy(p.color),m.opacity.value=p.opacity,p.map&&(m.map.value=p.map,e(p.map,m.mapTransform))}function a(m,p){m.dashSize.value=p.dashSize,m.totalSize.value=p.dashSize+p.gapSize,m.scale.value=p.scale}function l(m,p,v,y){m.diffuse.value.copy(p.color),m.opacity.value=p.opacity,m.size.value=p.size*v,m.scale.value=y*.5,p.map&&(m.map.value=p.map,e(p.map,m.uvTransform)),p.alphaMap&&(m.alphaMap.value=p.alphaMap,e(p.alphaMap,m.alphaMapTransform)),p.alphaTest>0&&(m.alphaTest.value=p.alphaTest)}function c(m,p){m.diffuse.value.copy(p.color),m.opacity.value=p.opacity,m.rotation.value=p.rotation,p.map&&(m.map.value=p.map,e(p.map,m.mapTransform)),p.alphaMap&&(m.alphaMap.value=p.alphaMap,e(p.alphaMap,m.alphaMapTransform)),p.alphaTest>0&&(m.alphaTest.value=p.alphaTest)}function h(m,p){m.specular.value.copy(p.specular),m.shininess.value=Math.max(p.shininess,1e-4)}function u(m,p){p.gradientMap&&(m.gradientMap.value=p.gradientMap)}function d(m,p){m.metalness.value=p.metalness,p.metalnessMap&&(m.metalnessMap.value=p.metalnessMap,e(p.metalnessMap,m.metalnessMapTransform)),m.roughness.value=p.roughness,p.roughnessMap&&(m.roughnessMap.value=p.roughnessMap,e(p.roughnessMap,m.roughnessMapTransform)),p.envMap&&(m.envMapIntensity.value=p.envMapIntensity)}function f(m,p,v){m.ior.value=p.ior,p.sheen>0&&(m.sheenColor.value.copy(p.sheenColor).multiplyScalar(p.sheen),m.sheenRoughness.value=p.sheenRoughness,p.sheenColorMap&&(m.sheenColorMap.value=p.sheenColorMap,e(p.sheenColorMap,m.sheenColorMapTransform)),p.sheenRoughnessMap&&(m.sheenRoughnessMap.value=p.sheenRoughnessMap,e(p.sheenRoughnessMap,m.sheenRoughnessMapTransform))),p.clearcoat>0&&(m.clearcoat.value=p.clearcoat,m.clearcoatRoughness.value=p.clearcoatRoughness,p.clearcoatMap&&(m.clearcoatMap.value=p.clearcoatMap,e(p.clearcoatMap,m.clearcoatMapTransform)),p.clearcoatRoughnessMap&&(m.clearcoatRoughnessMap.value=p.clearcoatRoughnessMap,e(p.clearcoatRoughnessMap,m.clearcoatRoughnessMapTransform)),p.clearcoatNormalMap&&(m.clearcoatNormalMap.value=p.clearcoatNormalMap,e(p.clearcoatNormalMap,m.clearcoatNormalMapTransform),m.clearcoatNormalScale.value.copy(p.clearcoatNormalScale),p.side===qe&&m.clearcoatNormalScale.value.negate())),p.dispersion>0&&(m.dispersion.value=p.dispersion),p.iridescence>0&&(m.iridescence.value=p.iridescence,m.iridescenceIOR.value=p.iridescenceIOR,m.iridescenceThicknessMinimum.value=p.iridescenceThicknessRange[0],m.iridescenceThicknessMaximum.value=p.iridescenceThicknessRange[1],p.iridescenceMap&&(m.iridescenceMap.value=p.iridescenceMap,e(p.iridescenceMap,m.iridescenceMapTransform)),p.iridescenceThicknessMap&&(m.iridescenceThicknessMap.value=p.iridescenceThicknessMap,e(p.iridescenceThicknessMap,m.iridescenceThicknessMapTransform))),p.transmission>0&&(m.transmission.value=p.transmission,m.transmissionSamplerMap.value=v.texture,m.transmissionSamplerSize.value.set(v.width,v.height),p.transmissionMap&&(m.transmissionMap.value=p.transmissionMap,e(p.transmissionMap,m.transmissionMapTransform)),m.thickness.value=p.thickness,p.thicknessMap&&(m.thicknessMap.value=p.thicknessMap,e(p.thicknessMap,m.thicknessMapTransform)),m.attenuationDistance.value=p.attenuationDistance,m.attenuationColor.value.copy(p.attenuationColor)),p.anisotropy>0&&(m.anisotropyVector.value.set(p.anisotropy*Math.cos(p.anisotropyRotation),p.anisotropy*Math.sin(p.anisotropyRotation)),p.anisotropyMap&&(m.anisotropyMap.value=p.anisotropyMap,e(p.anisotropyMap,m.anisotropyMapTransform))),m.specularIntensity.value=p.specularIntensity,m.specularColor.value.copy(p.specularColor),p.specularColorMap&&(m.specularColorMap.value=p.specularColorMap,e(p.specularColorMap,m.specularColorMapTransform)),p.specularIntensityMap&&(m.specularIntensityMap.value=p.specularIntensityMap,e(p.specularIntensityMap,m.specularIntensityMapTransform))}function g(m,p){p.matcap&&(m.matcap.value=p.matcap)}function _(m,p){const v=t.get(p).light;m.referencePosition.value.setFromMatrixPosition(v.matrixWorld),m.nearDistance.value=v.shadow.camera.near,m.farDistance.value=v.shadow.camera.far}return{refreshFogUniforms:i,refreshMaterialUniforms:s}}function Eb(n,t,e,i){let s={},r={},o=[];const a=n.getParameter(n.MAX_UNIFORM_BUFFER_BINDINGS);function l(v,y){const x=y.program;i.uniformBlockBinding(v,x)}function c(v,y){let x=s[v.id];x===void 0&&(g(v),x=h(v),s[v.id]=x,v.addEventListener("dispose",m));const A=y.program;i.updateUBOMapping(v,A);const w=t.render.frame;r[v.id]!==w&&(d(v),r[v.id]=w)}function h(v){const y=u();v.__bindingPointIndex=y;const x=n.createBuffer(),A=v.__size,w=v.usage;return n.bindBuffer(n.UNIFORM_BUFFER,x),n.bufferData(n.UNIFORM_BUFFER,A,w),n.bindBuffer(n.UNIFORM_BUFFER,null),n.bindBufferBase(n.UNIFORM_BUFFER,y,x),x}function u(){for(let v=0;v<a;v++)if(o.indexOf(v)===-1)return o.push(v),v;return console.error("THREE.WebGLRenderer: Maximum number of simultaneously usable uniforms groups reached."),0}function d(v){const y=s[v.id],x=v.uniforms,A=v.__cache;n.bindBuffer(n.UNIFORM_BUFFER,y);for(let w=0,T=x.length;w<T;w++){const R=Array.isArray(x[w])?x[w]:[x[w]];for(let S=0,b=R.length;S<b;S++){const P=R[S];if(f(P,w,S,A)===!0){const N=P.__offset,F=Array.isArray(P.value)?P.value:[P.value];let W=0;for(let G=0;G<F.length;G++){const H=F[G],$=_(H);typeof H=="number"||typeof H=="boolean"?(P.__data[0]=H,n.bufferSubData(n.UNIFORM_BUFFER,N+W,P.__data)):H.isMatrix3?(P.__data[0]=H.elements[0],P.__data[1]=H.elements[1],P.__data[2]=H.elements[2],P.__data[3]=0,P.__data[4]=H.elements[3],P.__data[5]=H.elements[4],P.__data[6]=H.elements[5],P.__data[7]=0,P.__data[8]=H.elements[6],P.__data[9]=H.elements[7],P.__data[10]=H.elements[8],P.__data[11]=0):(H.toArray(P.__data,W),W+=$.storage/Float32Array.BYTES_PER_ELEMENT)}n.bufferSubData(n.UNIFORM_BUFFER,N,P.__data)}}}n.bindBuffer(n.UNIFORM_BUFFER,null)}function f(v,y,x,A){const w=v.value,T=y+"_"+x;if(A[T]===void 0)return typeof w=="number"||typeof w=="boolean"?A[T]=w:A[T]=w.clone(),!0;{const R=A[T];if(typeof w=="number"||typeof w=="boolean"){if(R!==w)return A[T]=w,!0}else if(R.equals(w)===!1)return R.copy(w),!0}return!1}function g(v){const y=v.uniforms;let x=0;const A=16;for(let T=0,R=y.length;T<R;T++){const S=Array.isArray(y[T])?y[T]:[y[T]];for(let b=0,P=S.length;b<P;b++){const N=S[b],F=Array.isArray(N.value)?N.value:[N.value];for(let W=0,G=F.length;W<G;W++){const H=F[W],$=_(H),V=x%A,et=V%$.boundary,at=V+et;x+=et,at!==0&&A-at<$.storage&&(x+=A-at),N.__data=new Float32Array($.storage/Float32Array.BYTES_PER_ELEMENT),N.__offset=x,x+=$.storage}}}const w=x%A;return w>0&&(x+=A-w),v.__size=x,v.__cache={},this}function _(v){const y={boundary:0,storage:0};return typeof v=="number"||typeof v=="boolean"?(y.boundary=4,y.storage=4):v.isVector2?(y.boundary=8,y.storage=8):v.isVector3||v.isColor?(y.boundary=16,y.storage=12):v.isVector4?(y.boundary=16,y.storage=16):v.isMatrix3?(y.boundary=48,y.storage=48):v.isMatrix4?(y.boundary=64,y.storage=64):v.isTexture?console.warn("THREE.WebGLRenderer: Texture samplers can not be part of an uniforms group."):console.warn("THREE.WebGLRenderer: Unsupported uniform value type.",v),y}function m(v){const y=v.target;y.removeEventListener("dispose",m);const x=o.indexOf(y.__bindingPointIndex);o.splice(x,1),n.deleteBuffer(s[y.id]),delete s[y.id],delete r[y.id]}function p(){for(const v in s)n.deleteBuffer(s[v]);o=[],s={},r={}}return{bind:l,update:c,dispose:p}}class Tb{constructor(t={}){const{canvas:e=fg(),context:i=null,depth:s=!0,stencil:r=!1,alpha:o=!1,antialias:a=!1,premultipliedAlpha:l=!0,preserveDrawingBuffer:c=!1,powerPreference:h="default",failIfMajorPerformanceCaveat:u=!1,reverseDepthBuffer:d=!1}=t;this.isWebGLRenderer=!0;let f;if(i!==null){if(typeof WebGLRenderingContext<"u"&&i instanceof WebGLRenderingContext)throw new Error("THREE.WebGLRenderer: WebGL 1 is not supported since r163.");f=i.getContextAttributes().alpha}else f=o;const g=new Uint32Array(4),_=new Int32Array(4);let m=null,p=null;const v=[],y=[];this.domElement=e,this.debug={checkShaderErrors:!0,onShaderError:null},this.autoClear=!0,this.autoClearColor=!0,this.autoClearDepth=!0,this.autoClearStencil=!0,this.sortObjects=!0,this.clippingPlanes=[],this.localClippingEnabled=!1,this._outputColorSpace=an,this.toneMapping=pi,this.toneMappingExposure=1;const x=this;let A=!1,w=0,T=0,R=null,S=-1,b=null;const P=new be,N=new be;let F=null;const W=new Ht(0);let G=0,H=e.width,$=e.height,V=1,et=null,at=null;const _t=new be(0,0,H,$),Dt=new be(0,0,H,$);let $t=!1;const j=new zc;let tt=!1,ft=!1;const lt=new oe,Rt=new oe,It=new L,Vt=new be,_e={background:null,fog:null,environment:null,overrideMaterial:null,isScene:!0};let Yt=!1;function Ee(){return R===null?V:1}let U=i;function nn(E,I){return e.getContext(E,I)}try{const E={alpha:!0,depth:s,stencil:r,antialias:a,premultipliedAlpha:l,preserveDrawingBuffer:c,powerPreference:h,failIfMajorPerformanceCaveat:u};if("setAttribute"in e&&e.setAttribute("data-engine",`three.js r${Lc}`),e.addEventListener("webglcontextlost",Z,!1),e.addEventListener("webglcontextrestored",dt,!1),e.addEventListener("webglcontextcreationerror",ht,!1),U===null){const I="webgl2";if(U=nn(I,E),U===null)throw nn(I)?new Error("Error creating WebGL context with your selected attributes."):new Error("Error creating WebGL context.")}}catch(E){throw console.error("THREE.WebGLRenderer: "+E.message),E}let Wt,Xt,At,he,wt,C,M,k,K,J,Y,yt,ct,pt,qt,nt,mt,Ct,Pt,gt,jt,Bt,ae,D;function ot(){Wt=new Px(U),Wt.init(),Bt=new mb(U,Wt),Xt=new Ex(U,Wt,t,Bt),At=new db(U,Wt),Xt.reverseDepthBuffer&&d&&At.buffers.depth.setReversed(!0),he=new Ix(U),wt=new Zv,C=new pb(U,Wt,At,wt,Xt,Bt,he),M=new wx(x),k=new Rx(x),K=new zg(U),ae=new Mx(U,K),J=new Dx(U,K,he,ae),Y=new Ux(U,J,K,he),Pt=new Ox(U,Xt,C),nt=new Tx(wt),yt=new Kv(x,M,k,Wt,Xt,ae,nt),ct=new Sb(x,wt),pt=new Qv,qt=new rb(Wt),Ct=new yx(x,M,k,At,Y,f,l),mt=new hb(x,Y,Xt),D=new Eb(U,he,Xt,At),gt=new Sx(U,Wt,he),jt=new Lx(U,Wt,he),he.programs=yt.programs,x.capabilities=Xt,x.extensions=Wt,x.properties=wt,x.renderLists=pt,x.shadowMap=mt,x.state=At,x.info=he}ot();const X=new yb(x,U);this.xr=X,this.getContext=function(){return U},this.getContextAttributes=function(){return U.getContextAttributes()},this.forceContextLoss=function(){const E=Wt.get("WEBGL_lose_context");E&&E.loseContext()},this.forceContextRestore=function(){const E=Wt.get("WEBGL_lose_context");E&&E.restoreContext()},this.getPixelRatio=function(){return V},this.setPixelRatio=function(E){E!==void 0&&(V=E,this.setSize(H,$,!1))},this.getSize=function(E){return E.set(H,$)},this.setSize=function(E,I,B=!0){if(X.isPresenting){console.warn("THREE.WebGLRenderer: Can't change size while VR device is presenting.");return}H=E,$=I,e.width=Math.floor(E*V),e.height=Math.floor(I*V),B===!0&&(e.style.width=E+"px",e.style.height=I+"px"),this.setViewport(0,0,E,I)},this.getDrawingBufferSize=function(E){return E.set(H*V,$*V).floor()},this.setDrawingBufferSize=function(E,I,B){H=E,$=I,V=B,e.width=Math.floor(E*B),e.height=Math.floor(I*B),this.setViewport(0,0,E,I)},this.getCurrentViewport=function(E){return E.copy(P)},this.getViewport=function(E){return E.copy(_t)},this.setViewport=function(E,I,B,z){E.isVector4?_t.set(E.x,E.y,E.z,E.w):_t.set(E,I,B,z),At.viewport(P.copy(_t).multiplyScalar(V).round())},this.getScissor=function(E){return E.copy(Dt)},this.setScissor=function(E,I,B,z){E.isVector4?Dt.set(E.x,E.y,E.z,E.w):Dt.set(E,I,B,z),At.scissor(N.copy(Dt).multiplyScalar(V).round())},this.getScissorTest=function(){return $t},this.setScissorTest=function(E){At.setScissorTest($t=E)},this.setOpaqueSort=function(E){et=E},this.setTransparentSort=function(E){at=E},this.getClearColor=function(E){return E.copy(Ct.getClearColor())},this.setClearColor=function(){Ct.setClearColor.apply(Ct,arguments)},this.getClearAlpha=function(){return Ct.getClearAlpha()},this.setClearAlpha=function(){Ct.setClearAlpha.apply(Ct,arguments)},this.clear=function(E=!0,I=!0,B=!0){let z=0;if(E){let O=!1;if(R!==null){const it=R.texture.format;O=it===kc||it===Fc||it===Nc}if(O){const it=R.texture.type,ut=it===Jn||it===Xi||it===yr||it===Os||it===Oc||it===Uc,xt=Ct.getClearColor(),vt=Ct.getClearAlpha(),Lt=xt.r,Nt=xt.g,bt=xt.b;ut?(g[0]=Lt,g[1]=Nt,g[2]=bt,g[3]=vt,U.clearBufferuiv(U.COLOR,0,g)):(_[0]=Lt,_[1]=Nt,_[2]=bt,_[3]=vt,U.clearBufferiv(U.COLOR,0,_))}else z|=U.COLOR_BUFFER_BIT}I&&(z|=U.DEPTH_BUFFER_BIT),B&&(z|=U.STENCIL_BUFFER_BIT,this.state.buffers.stencil.setMask(4294967295)),U.clear(z)},this.clearColor=function(){this.clear(!0,!1,!1)},this.clearDepth=function(){this.clear(!1,!0,!1)},this.clearStencil=function(){this.clear(!1,!1,!0)},this.dispose=function(){e.removeEventListener("webglcontextlost",Z,!1),e.removeEventListener("webglcontextrestored",dt,!1),e.removeEventListener("webglcontextcreationerror",ht,!1),pt.dispose(),qt.dispose(),wt.dispose(),M.dispose(),k.dispose(),Y.dispose(),ae.dispose(),D.dispose(),yt.dispose(),X.dispose(),X.removeEventListener("sessionstart",hh),X.removeEventListener("sessionend",uh),Si.stop()};function Z(E){E.preventDefault(),console.log("THREE.WebGLRenderer: Context Lost."),A=!0}function dt(){console.log("THREE.WebGLRenderer: Context Restored."),A=!1;const E=he.autoReset,I=mt.enabled,B=mt.autoUpdate,z=mt.needsUpdate,O=mt.type;ot(),he.autoReset=E,mt.enabled=I,mt.autoUpdate=B,mt.needsUpdate=z,mt.type=O}function ht(E){console.error("THREE.WebGLRenderer: A WebGL context could not be created. Reason: ",E.statusMessage)}function Ut(E){const I=E.target;I.removeEventListener("dispose",Ut),ve(I)}function ve(E){Ue(E),wt.remove(E)}function Ue(E){const I=wt.get(E).programs;I!==void 0&&(I.forEach(function(B){yt.releaseProgram(B)}),E.isShaderMaterial&&yt.releaseShaderCache(E))}this.renderBufferDirect=function(E,I,B,z,O,it){I===null&&(I=_e);const ut=O.isMesh&&O.matrixWorld.determinant()<0,xt=am(E,I,B,z,O);At.setMaterial(z,ut);let vt=B.index,Lt=1;if(z.wireframe===!0){if(vt=J.getWireframeAttribute(B),vt===void 0)return;Lt=2}const Nt=B.drawRange,bt=B.attributes.position;let Jt=Nt.start*Lt,le=(Nt.start+Nt.count)*Lt;it!==null&&(Jt=Math.max(Jt,it.start*Lt),le=Math.min(le,(it.start+it.count)*Lt)),vt!==null?(Jt=Math.max(Jt,0),le=Math.min(le,vt.count)):bt!=null&&(Jt=Math.max(Jt,0),le=Math.min(le,bt.count));const ue=le-Jt;if(ue<0||ue===1/0)return;ae.setup(O,z,xt,B,vt);let Xe,Qt=gt;if(vt!==null&&(Xe=K.get(vt),Qt=jt,Qt.setIndex(Xe)),O.isMesh)z.wireframe===!0?(At.setLineWidth(z.wireframeLinewidth*Ee()),Qt.setMode(U.LINES)):Qt.setMode(U.TRIANGLES);else if(O.isLine){let Mt=z.linewidth;Mt===void 0&&(Mt=1),At.setLineWidth(Mt*Ee()),O.isLineSegments?Qt.setMode(U.LINES):O.isLineLoop?Qt.setMode(U.LINE_LOOP):Qt.setMode(U.LINE_STRIP)}else O.isPoints?Qt.setMode(U.POINTS):O.isSprite&&Qt.setMode(U.TRIANGLES);if(O.isBatchedMesh)if(O._multiDrawInstances!==null)Qt.renderMultiDrawInstances(O._multiDrawStarts,O._multiDrawCounts,O._multiDrawCount,O._multiDrawInstances);else if(Wt.get("WEBGL_multi_draw"))Qt.renderMultiDraw(O._multiDrawStarts,O._multiDrawCounts,O._multiDrawCount);else{const Mt=O._multiDrawStarts,Ln=O._multiDrawCounts,te=O._multiDrawCount,un=vt?K.get(vt).bytesPerElement:1,Qi=wt.get(z).currentProgram.getUniforms();for(let Ke=0;Ke<te;Ke++)Qi.setValue(U,"_gl_DrawID",Ke),Qt.render(Mt[Ke]/un,Ln[Ke])}else if(O.isInstancedMesh)Qt.renderInstances(Jt,ue,O.count);else if(B.isInstancedBufferGeometry){const Mt=B._maxInstanceCount!==void 0?B._maxInstanceCount:1/0,Ln=Math.min(B.instanceCount,Mt);Qt.renderInstances(Jt,ue,Ln)}else Qt.render(Jt,ue)};function ee(E,I,B){E.transparent===!0&&E.side===Tn&&E.forceSinglePass===!1?(E.side=qe,E.needsUpdate=!0,zr(E,I,B),E.side=_i,E.needsUpdate=!0,zr(E,I,B),E.side=Tn):zr(E,I,B)}this.compile=function(E,I,B=null){B===null&&(B=E),p=qt.get(B),p.init(I),y.push(p),B.traverseVisible(function(O){O.isLight&&O.layers.test(I.layers)&&(p.pushLight(O),O.castShadow&&p.pushShadow(O))}),E!==B&&E.traverseVisible(function(O){O.isLight&&O.layers.test(I.layers)&&(p.pushLight(O),O.castShadow&&p.pushShadow(O))}),p.setupLights();const z=new Set;return E.traverse(function(O){if(!(O.isMesh||O.isPoints||O.isLine||O.isSprite))return;const it=O.material;if(it)if(Array.isArray(it))for(let ut=0;ut<it.length;ut++){const xt=it[ut];ee(xt,B,O),z.add(xt)}else ee(it,B,O),z.add(it)}),y.pop(),p=null,z},this.compileAsync=function(E,I,B=null){const z=this.compile(E,I,B);return new Promise(O=>{function it(){if(z.forEach(function(ut){wt.get(ut).currentProgram.isReady()&&z.delete(ut)}),z.size===0){O(E);return}setTimeout(it,10)}Wt.get("KHR_parallel_shader_compile")!==null?it():setTimeout(it,10)})};let hn=null;function Dn(E){hn&&hn(E)}function hh(){Si.stop()}function uh(){Si.start()}const Si=new Hf;Si.setAnimationLoop(Dn),typeof self<"u"&&Si.setContext(self),this.setAnimationLoop=function(E){hn=E,X.setAnimationLoop(E),E===null?Si.stop():Si.start()},X.addEventListener("sessionstart",hh),X.addEventListener("sessionend",uh),this.render=function(E,I){if(I!==void 0&&I.isCamera!==!0){console.error("THREE.WebGLRenderer.render: camera is not an instance of THREE.Camera.");return}if(A===!0)return;if(E.matrixWorldAutoUpdate===!0&&E.updateMatrixWorld(),I.parent===null&&I.matrixWorldAutoUpdate===!0&&I.updateMatrixWorld(),X.enabled===!0&&X.isPresenting===!0&&(X.cameraAutoUpdate===!0&&X.updateCamera(I),I=X.getCamera()),E.isScene===!0&&E.onBeforeRender(x,E,I,R),p=qt.get(E,y.length),p.init(I),y.push(p),Rt.multiplyMatrices(I.projectionMatrix,I.matrixWorldInverse),j.setFromProjectionMatrix(Rt),ft=this.localClippingEnabled,tt=nt.init(this.clippingPlanes,ft),m=pt.get(E,v.length),m.init(),v.push(m),X.enabled===!0&&X.isPresenting===!0){const it=x.xr.getDepthSensingMesh();it!==null&&wa(it,I,-1/0,x.sortObjects)}wa(E,I,0,x.sortObjects),m.finish(),x.sortObjects===!0&&m.sort(et,at),Yt=X.enabled===!1||X.isPresenting===!1||X.hasDepthSensing()===!1,Yt&&Ct.addToRenderList(m,E),this.info.render.frame++,tt===!0&&nt.beginShadows();const B=p.state.shadowsArray;mt.render(B,E,I),tt===!0&&nt.endShadows(),this.info.autoReset===!0&&this.info.reset();const z=m.opaque,O=m.transmissive;if(p.setupLights(),I.isArrayCamera){const it=I.cameras;if(O.length>0)for(let ut=0,xt=it.length;ut<xt;ut++){const vt=it[ut];fh(z,O,E,vt)}Yt&&Ct.render(E);for(let ut=0,xt=it.length;ut<xt;ut++){const vt=it[ut];dh(m,E,vt,vt.viewport)}}else O.length>0&&fh(z,O,E,I),Yt&&Ct.render(E),dh(m,E,I);R!==null&&(C.updateMultisampleRenderTarget(R),C.updateRenderTargetMipmap(R)),E.isScene===!0&&E.onAfterRender(x,E,I),ae.resetDefaultState(),S=-1,b=null,y.pop(),y.length>0?(p=y[y.length-1],tt===!0&&nt.setGlobalState(x.clippingPlanes,p.state.camera)):p=null,v.pop(),v.length>0?m=v[v.length-1]:m=null};function wa(E,I,B,z){if(E.visible===!1)return;if(E.layers.test(I.layers)){if(E.isGroup)B=E.renderOrder;else if(E.isLOD)E.autoUpdate===!0&&E.update(I);else if(E.isLight)p.pushLight(E),E.castShadow&&p.pushShadow(E);else if(E.isSprite){if(!E.frustumCulled||j.intersectsSprite(E)){z&&Vt.setFromMatrixPosition(E.matrixWorld).applyMatrix4(Rt);const ut=Y.update(E),xt=E.material;xt.visible&&m.push(E,ut,xt,B,Vt.z,null)}}else if((E.isMesh||E.isLine||E.isPoints)&&(!E.frustumCulled||j.intersectsObject(E))){const ut=Y.update(E),xt=E.material;if(z&&(E.boundingSphere!==void 0?(E.boundingSphere===null&&E.computeBoundingSphere(),Vt.copy(E.boundingSphere.center)):(ut.boundingSphere===null&&ut.computeBoundingSphere(),Vt.copy(ut.boundingSphere.center)),Vt.applyMatrix4(E.matrixWorld).applyMatrix4(Rt)),Array.isArray(xt)){const vt=ut.groups;for(let Lt=0,Nt=vt.length;Lt<Nt;Lt++){const bt=vt[Lt],Jt=xt[bt.materialIndex];Jt&&Jt.visible&&m.push(E,ut,Jt,B,Vt.z,bt)}}else xt.visible&&m.push(E,ut,xt,B,Vt.z,null)}}const it=E.children;for(let ut=0,xt=it.length;ut<xt;ut++)wa(it[ut],I,B,z)}function dh(E,I,B,z){const O=E.opaque,it=E.transmissive,ut=E.transparent;p.setupLightsView(B),tt===!0&&nt.setGlobalState(x.clippingPlanes,B),z&&At.viewport(P.copy(z)),O.length>0&&Br(O,I,B),it.length>0&&Br(it,I,B),ut.length>0&&Br(ut,I,B),At.buffers.depth.setTest(!0),At.buffers.depth.setMask(!0),At.buffers.color.setMask(!0),At.setPolygonOffset(!1)}function fh(E,I,B,z){if((B.isScene===!0?B.overrideMaterial:null)!==null)return;p.state.transmissionRenderTarget[z.id]===void 0&&(p.state.transmissionRenderTarget[z.id]=new ji(1,1,{generateMipmaps:!0,type:Wt.has("EXT_color_buffer_half_float")||Wt.has("EXT_color_buffer_float")?Dr:Jn,minFilter:zi,samples:4,stencilBuffer:r,resolveDepthBuffer:!1,resolveStencilBuffer:!1,colorSpace:Kt.workingColorSpace}));const it=p.state.transmissionRenderTarget[z.id],ut=z.viewport||P;it.setSize(ut.z,ut.w);const xt=x.getRenderTarget();x.setRenderTarget(it),x.getClearColor(W),G=x.getClearAlpha(),G<1&&x.setClearColor(16777215,.5),x.clear(),Yt&&Ct.render(B);const vt=x.toneMapping;x.toneMapping=pi;const Lt=z.viewport;if(z.viewport!==void 0&&(z.viewport=void 0),p.setupLightsView(z),tt===!0&&nt.setGlobalState(x.clippingPlanes,z),Br(E,B,z),C.updateMultisampleRenderTarget(it),C.updateRenderTargetMipmap(it),Wt.has("WEBGL_multisampled_render_to_texture")===!1){let Nt=!1;for(let bt=0,Jt=I.length;bt<Jt;bt++){const le=I[bt],ue=le.object,Xe=le.geometry,Qt=le.material,Mt=le.group;if(Qt.side===Tn&&ue.layers.test(z.layers)){const Ln=Qt.side;Qt.side=qe,Qt.needsUpdate=!0,ph(ue,B,z,Xe,Qt,Mt),Qt.side=Ln,Qt.needsUpdate=!0,Nt=!0}}Nt===!0&&(C.updateMultisampleRenderTarget(it),C.updateRenderTargetMipmap(it))}x.setRenderTarget(xt),x.setClearColor(W,G),Lt!==void 0&&(z.viewport=Lt),x.toneMapping=vt}function Br(E,I,B){const z=I.isScene===!0?I.overrideMaterial:null;for(let O=0,it=E.length;O<it;O++){const ut=E[O],xt=ut.object,vt=ut.geometry,Lt=z===null?ut.material:z,Nt=ut.group;xt.layers.test(B.layers)&&ph(xt,I,B,vt,Lt,Nt)}}function ph(E,I,B,z,O,it){E.onBeforeRender(x,I,B,z,O,it),E.modelViewMatrix.multiplyMatrices(B.matrixWorldInverse,E.matrixWorld),E.normalMatrix.getNormalMatrix(E.modelViewMatrix),O.onBeforeRender(x,I,B,z,E,it),O.transparent===!0&&O.side===Tn&&O.forceSinglePass===!1?(O.side=qe,O.needsUpdate=!0,x.renderBufferDirect(B,I,z,O,E,it),O.side=_i,O.needsUpdate=!0,x.renderBufferDirect(B,I,z,O,E,it),O.side=Tn):x.renderBufferDirect(B,I,z,O,E,it),E.onAfterRender(x,I,B,z,O,it)}function zr(E,I,B){I.isScene!==!0&&(I=_e);const z=wt.get(E),O=p.state.lights,it=p.state.shadowsArray,ut=O.state.version,xt=yt.getParameters(E,O.state,it,I,B),vt=yt.getProgramCacheKey(xt);let Lt=z.programs;z.environment=E.isMeshStandardMaterial?I.environment:null,z.fog=I.fog,z.envMap=(E.isMeshStandardMaterial?k:M).get(E.envMap||z.environment),z.envMapRotation=z.environment!==null&&E.envMap===null?I.environmentRotation:E.envMapRotation,Lt===void 0&&(E.addEventListener("dispose",Ut),Lt=new Map,z.programs=Lt);let Nt=Lt.get(vt);if(Nt!==void 0){if(z.currentProgram===Nt&&z.lightsStateVersion===ut)return gh(E,xt),Nt}else xt.uniforms=yt.getUniforms(E),E.onBeforeCompile(xt,x),Nt=yt.acquireProgram(xt,vt),Lt.set(vt,Nt),z.uniforms=xt.uniforms;const bt=z.uniforms;return(!E.isShaderMaterial&&!E.isRawShaderMaterial||E.clipping===!0)&&(bt.clippingPlanes=nt.uniform),gh(E,xt),z.needsLights=cm(E),z.lightsStateVersion=ut,z.needsLights&&(bt.ambientLightColor.value=O.state.ambient,bt.lightProbe.value=O.state.probe,bt.directionalLights.value=O.state.directional,bt.directionalLightShadows.value=O.state.directionalShadow,bt.spotLights.value=O.state.spot,bt.spotLightShadows.value=O.state.spotShadow,bt.rectAreaLights.value=O.state.rectArea,bt.ltc_1.value=O.state.rectAreaLTC1,bt.ltc_2.value=O.state.rectAreaLTC2,bt.pointLights.value=O.state.point,bt.pointLightShadows.value=O.state.pointShadow,bt.hemisphereLights.value=O.state.hemi,bt.directionalShadowMap.value=O.state.directionalShadowMap,bt.directionalShadowMatrix.value=O.state.directionalShadowMatrix,bt.spotShadowMap.value=O.state.spotShadowMap,bt.spotLightMatrix.value=O.state.spotLightMatrix,bt.spotLightMap.value=O.state.spotLightMap,bt.pointShadowMap.value=O.state.pointShadowMap,bt.pointShadowMatrix.value=O.state.pointShadowMatrix),z.currentProgram=Nt,z.uniformsList=null,Nt}function mh(E){if(E.uniformsList===null){const I=E.currentProgram.getUniforms();E.uniformsList=zo.seqWithValue(I.seq,E.uniforms)}return E.uniformsList}function gh(E,I){const B=wt.get(E);B.outputColorSpace=I.outputColorSpace,B.batching=I.batching,B.batchingColor=I.batchingColor,B.instancing=I.instancing,B.instancingColor=I.instancingColor,B.instancingMorph=I.instancingMorph,B.skinning=I.skinning,B.morphTargets=I.morphTargets,B.morphNormals=I.morphNormals,B.morphColors=I.morphColors,B.morphTargetsCount=I.morphTargetsCount,B.numClippingPlanes=I.numClippingPlanes,B.numIntersection=I.numClipIntersection,B.vertexAlphas=I.vertexAlphas,B.vertexTangents=I.vertexTangents,B.toneMapping=I.toneMapping}function am(E,I,B,z,O){I.isScene!==!0&&(I=_e),C.resetTextureUnits();const it=I.fog,ut=z.isMeshStandardMaterial?I.environment:null,xt=R===null?x.outputColorSpace:R.isXRRenderTarget===!0?R.texture.colorSpace:Hs,vt=(z.isMeshStandardMaterial?k:M).get(z.envMap||ut),Lt=z.vertexColors===!0&&!!B.attributes.color&&B.attributes.color.itemSize===4,Nt=!!B.attributes.tangent&&(!!z.normalMap||z.anisotropy>0),bt=!!B.morphAttributes.position,Jt=!!B.morphAttributes.normal,le=!!B.morphAttributes.color;let ue=pi;z.toneMapped&&(R===null||R.isXRRenderTarget===!0)&&(ue=x.toneMapping);const Xe=B.morphAttributes.position||B.morphAttributes.normal||B.morphAttributes.color,Qt=Xe!==void 0?Xe.length:0,Mt=wt.get(z),Ln=p.state.lights;if(tt===!0&&(ft===!0||E!==b)){const sn=E===b&&z.id===S;nt.setState(z,E,sn)}let te=!1;z.version===Mt.__version?(Mt.needsLights&&Mt.lightsStateVersion!==Ln.state.version||Mt.outputColorSpace!==xt||O.isBatchedMesh&&Mt.batching===!1||!O.isBatchedMesh&&Mt.batching===!0||O.isBatchedMesh&&Mt.batchingColor===!0&&O.colorTexture===null||O.isBatchedMesh&&Mt.batchingColor===!1&&O.colorTexture!==null||O.isInstancedMesh&&Mt.instancing===!1||!O.isInstancedMesh&&Mt.instancing===!0||O.isSkinnedMesh&&Mt.skinning===!1||!O.isSkinnedMesh&&Mt.skinning===!0||O.isInstancedMesh&&Mt.instancingColor===!0&&O.instanceColor===null||O.isInstancedMesh&&Mt.instancingColor===!1&&O.instanceColor!==null||O.isInstancedMesh&&Mt.instancingMorph===!0&&O.morphTexture===null||O.isInstancedMesh&&Mt.instancingMorph===!1&&O.morphTexture!==null||Mt.envMap!==vt||z.fog===!0&&Mt.fog!==it||Mt.numClippingPlanes!==void 0&&(Mt.numClippingPlanes!==nt.numPlanes||Mt.numIntersection!==nt.numIntersection)||Mt.vertexAlphas!==Lt||Mt.vertexTangents!==Nt||Mt.morphTargets!==bt||Mt.morphNormals!==Jt||Mt.morphColors!==le||Mt.toneMapping!==ue||Mt.morphTargetsCount!==Qt)&&(te=!0):(te=!0,Mt.__version=z.version);let un=Mt.currentProgram;te===!0&&(un=zr(z,I,O));let Qi=!1,Ke=!1,Gs=!1;const de=un.getUniforms(),yn=Mt.uniforms;if(At.useProgram(un.program)&&(Qi=!0,Ke=!0,Gs=!0),z.id!==S&&(S=z.id,Ke=!0),Qi||b!==E){At.buffers.depth.getReversed()?(lt.copy(E.projectionMatrix),mg(lt),gg(lt),de.setValue(U,"projectionMatrix",lt)):de.setValue(U,"projectionMatrix",E.projectionMatrix),de.setValue(U,"viewMatrix",E.matrixWorldInverse);const Qn=de.map.cameraPosition;Qn!==void 0&&Qn.setValue(U,It.setFromMatrixPosition(E.matrixWorld)),Xt.logarithmicDepthBuffer&&de.setValue(U,"logDepthBufFC",2/(Math.log(E.far+1)/Math.LN2)),(z.isMeshPhongMaterial||z.isMeshToonMaterial||z.isMeshLambertMaterial||z.isMeshBasicMaterial||z.isMeshStandardMaterial||z.isShaderMaterial)&&de.setValue(U,"isOrthographic",E.isOrthographicCamera===!0),b!==E&&(b=E,Ke=!0,Gs=!0)}if(O.isSkinnedMesh){de.setOptional(U,O,"bindMatrix"),de.setOptional(U,O,"bindMatrixInverse");const sn=O.skeleton;sn&&(sn.boneTexture===null&&sn.computeBoneTexture(),de.setValue(U,"boneTexture",sn.boneTexture,C))}O.isBatchedMesh&&(de.setOptional(U,O,"batchingTexture"),de.setValue(U,"batchingTexture",O._matricesTexture,C),de.setOptional(U,O,"batchingIdTexture"),de.setValue(U,"batchingIdTexture",O._indirectTexture,C),de.setOptional(U,O,"batchingColorTexture"),O._colorsTexture!==null&&de.setValue(U,"batchingColorTexture",O._colorsTexture,C));const Ws=B.morphAttributes;if((Ws.position!==void 0||Ws.normal!==void 0||Ws.color!==void 0)&&Pt.update(O,B,un),(Ke||Mt.receiveShadow!==O.receiveShadow)&&(Mt.receiveShadow=O.receiveShadow,de.setValue(U,"receiveShadow",O.receiveShadow)),z.isMeshGouraudMaterial&&z.envMap!==null&&(yn.envMap.value=vt,yn.flipEnvMap.value=vt.isCubeTexture&&vt.isRenderTargetTexture===!1?-1:1),z.isMeshStandardMaterial&&z.envMap===null&&I.environment!==null&&(yn.envMapIntensity.value=I.environmentIntensity),Ke&&(de.setValue(U,"toneMappingExposure",x.toneMappingExposure),Mt.needsLights&&lm(yn,Gs),it&&z.fog===!0&&ct.refreshFogUniforms(yn,it),ct.refreshMaterialUniforms(yn,z,V,$,p.state.transmissionRenderTarget[E.id]),zo.upload(U,mh(Mt),yn,C)),z.isShaderMaterial&&z.uniformsNeedUpdate===!0&&(zo.upload(U,mh(Mt),yn,C),z.uniformsNeedUpdate=!1),z.isSpriteMaterial&&de.setValue(U,"center",O.center),de.setValue(U,"modelViewMatrix",O.modelViewMatrix),de.setValue(U,"normalMatrix",O.normalMatrix),de.setValue(U,"modelMatrix",O.matrixWorld),z.isShaderMaterial||z.isRawShaderMaterial){const sn=z.uniformsGroups;for(let Qn=0,ti=sn.length;Qn<ti;Qn++){const _h=sn[Qn];D.update(_h,un),D.bind(_h,un)}}return un}function lm(E,I){E.ambientLightColor.needsUpdate=I,E.lightProbe.needsUpdate=I,E.directionalLights.needsUpdate=I,E.directionalLightShadows.needsUpdate=I,E.pointLights.needsUpdate=I,E.pointLightShadows.needsUpdate=I,E.spotLights.needsUpdate=I,E.spotLightShadows.needsUpdate=I,E.rectAreaLights.needsUpdate=I,E.hemisphereLights.needsUpdate=I}function cm(E){return E.isMeshLambertMaterial||E.isMeshToonMaterial||E.isMeshPhongMaterial||E.isMeshStandardMaterial||E.isShadowMaterial||E.isShaderMaterial&&E.lights===!0}this.getActiveCubeFace=function(){return w},this.getActiveMipmapLevel=function(){return T},this.getRenderTarget=function(){return R},this.setRenderTargetTextures=function(E,I,B){wt.get(E.texture).__webglTexture=I,wt.get(E.depthTexture).__webglTexture=B;const z=wt.get(E);z.__hasExternalTextures=!0,z.__autoAllocateDepthBuffer=B===void 0,z.__autoAllocateDepthBuffer||Wt.has("WEBGL_multisampled_render_to_texture")===!0&&(console.warn("THREE.WebGLRenderer: Render-to-texture extension was disabled because an external texture was provided"),z.__useRenderToTexture=!1)},this.setRenderTargetFramebuffer=function(E,I){const B=wt.get(E);B.__webglFramebuffer=I,B.__useDefaultFramebuffer=I===void 0},this.setRenderTarget=function(E,I=0,B=0){R=E,w=I,T=B;let z=!0,O=null,it=!1,ut=!1;if(E){const vt=wt.get(E);if(vt.__useDefaultFramebuffer!==void 0)At.bindFramebuffer(U.FRAMEBUFFER,null),z=!1;else if(vt.__webglFramebuffer===void 0)C.setupRenderTarget(E);else if(vt.__hasExternalTextures)C.rebindTextures(E,wt.get(E.texture).__webglTexture,wt.get(E.depthTexture).__webglTexture);else if(E.depthBuffer){const bt=E.depthTexture;if(vt.__boundDepthTexture!==bt){if(bt!==null&&wt.has(bt)&&(E.width!==bt.image.width||E.height!==bt.image.height))throw new Error("WebGLRenderTarget: Attached DepthTexture is initialized to the incorrect size.");C.setupDepthRenderbuffer(E)}}const Lt=E.texture;(Lt.isData3DTexture||Lt.isDataArrayTexture||Lt.isCompressedArrayTexture)&&(ut=!0);const Nt=wt.get(E).__webglFramebuffer;E.isWebGLCubeRenderTarget?(Array.isArray(Nt[I])?O=Nt[I][B]:O=Nt[I],it=!0):E.samples>0&&C.useMultisampledRTT(E)===!1?O=wt.get(E).__webglMultisampledFramebuffer:Array.isArray(Nt)?O=Nt[B]:O=Nt,P.copy(E.viewport),N.copy(E.scissor),F=E.scissorTest}else P.copy(_t).multiplyScalar(V).floor(),N.copy(Dt).multiplyScalar(V).floor(),F=$t;if(At.bindFramebuffer(U.FRAMEBUFFER,O)&&z&&At.drawBuffers(E,O),At.viewport(P),At.scissor(N),At.setScissorTest(F),it){const vt=wt.get(E.texture);U.framebufferTexture2D(U.FRAMEBUFFER,U.COLOR_ATTACHMENT0,U.TEXTURE_CUBE_MAP_POSITIVE_X+I,vt.__webglTexture,B)}else if(ut){const vt=wt.get(E.texture),Lt=I||0;U.framebufferTextureLayer(U.FRAMEBUFFER,U.COLOR_ATTACHMENT0,vt.__webglTexture,B||0,Lt)}S=-1},this.readRenderTargetPixels=function(E,I,B,z,O,it,ut){if(!(E&&E.isWebGLRenderTarget)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not THREE.WebGLRenderTarget.");return}let xt=wt.get(E).__webglFramebuffer;if(E.isWebGLCubeRenderTarget&&ut!==void 0&&(xt=xt[ut]),xt){At.bindFramebuffer(U.FRAMEBUFFER,xt);try{const vt=E.texture,Lt=vt.format,Nt=vt.type;if(!Xt.textureFormatReadable(Lt)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not in RGBA or implementation defined format.");return}if(!Xt.textureTypeReadable(Nt)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not in UnsignedByteType or implementation defined type.");return}I>=0&&I<=E.width-z&&B>=0&&B<=E.height-O&&U.readPixels(I,B,z,O,Bt.convert(Lt),Bt.convert(Nt),it)}finally{const vt=R!==null?wt.get(R).__webglFramebuffer:null;At.bindFramebuffer(U.FRAMEBUFFER,vt)}}},this.readRenderTargetPixelsAsync=async function(E,I,B,z,O,it,ut){if(!(E&&E.isWebGLRenderTarget))throw new Error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not THREE.WebGLRenderTarget.");let xt=wt.get(E).__webglFramebuffer;if(E.isWebGLCubeRenderTarget&&ut!==void 0&&(xt=xt[ut]),xt){const vt=E.texture,Lt=vt.format,Nt=vt.type;if(!Xt.textureFormatReadable(Lt))throw new Error("THREE.WebGLRenderer.readRenderTargetPixelsAsync: renderTarget is not in RGBA or implementation defined format.");if(!Xt.textureTypeReadable(Nt))throw new Error("THREE.WebGLRenderer.readRenderTargetPixelsAsync: renderTarget is not in UnsignedByteType or implementation defined type.");if(I>=0&&I<=E.width-z&&B>=0&&B<=E.height-O){At.bindFramebuffer(U.FRAMEBUFFER,xt);const bt=U.createBuffer();U.bindBuffer(U.PIXEL_PACK_BUFFER,bt),U.bufferData(U.PIXEL_PACK_BUFFER,it.byteLength,U.STREAM_READ),U.readPixels(I,B,z,O,Bt.convert(Lt),Bt.convert(Nt),0);const Jt=R!==null?wt.get(R).__webglFramebuffer:null;At.bindFramebuffer(U.FRAMEBUFFER,Jt);const le=U.fenceSync(U.SYNC_GPU_COMMANDS_COMPLETE,0);return U.flush(),await pg(U,le,4),U.bindBuffer(U.PIXEL_PACK_BUFFER,bt),U.getBufferSubData(U.PIXEL_PACK_BUFFER,0,it),U.deleteBuffer(bt),U.deleteSync(le),it}else throw new Error("THREE.WebGLRenderer.readRenderTargetPixelsAsync: requested read bounds are out of range.")}},this.copyFramebufferToTexture=function(E,I=null,B=0){E.isTexture!==!0&&(or("WebGLRenderer: copyFramebufferToTexture function signature has changed."),I=arguments[0]||null,E=arguments[1]);const z=Math.pow(2,-B),O=Math.floor(E.image.width*z),it=Math.floor(E.image.height*z),ut=I!==null?I.x:0,xt=I!==null?I.y:0;C.setTexture2D(E,0),U.copyTexSubImage2D(U.TEXTURE_2D,B,0,0,ut,xt,O,it),At.unbindTexture()},this.copyTextureToTexture=function(E,I,B=null,z=null,O=0){E.isTexture!==!0&&(or("WebGLRenderer: copyTextureToTexture function signature has changed."),z=arguments[0]||null,E=arguments[1],I=arguments[2],O=arguments[3]||0,B=null);let it,ut,xt,vt,Lt,Nt,bt,Jt,le;const ue=E.isCompressedTexture?E.mipmaps[O]:E.image;B!==null?(it=B.max.x-B.min.x,ut=B.max.y-B.min.y,xt=B.isBox3?B.max.z-B.min.z:1,vt=B.min.x,Lt=B.min.y,Nt=B.isBox3?B.min.z:0):(it=ue.width,ut=ue.height,xt=ue.depth||1,vt=0,Lt=0,Nt=0),z!==null?(bt=z.x,Jt=z.y,le=z.z):(bt=0,Jt=0,le=0);const Xe=Bt.convert(I.format),Qt=Bt.convert(I.type);let Mt;I.isData3DTexture?(C.setTexture3D(I,0),Mt=U.TEXTURE_3D):I.isDataArrayTexture||I.isCompressedArrayTexture?(C.setTexture2DArray(I,0),Mt=U.TEXTURE_2D_ARRAY):(C.setTexture2D(I,0),Mt=U.TEXTURE_2D),U.pixelStorei(U.UNPACK_FLIP_Y_WEBGL,I.flipY),U.pixelStorei(U.UNPACK_PREMULTIPLY_ALPHA_WEBGL,I.premultiplyAlpha),U.pixelStorei(U.UNPACK_ALIGNMENT,I.unpackAlignment);const Ln=U.getParameter(U.UNPACK_ROW_LENGTH),te=U.getParameter(U.UNPACK_IMAGE_HEIGHT),un=U.getParameter(U.UNPACK_SKIP_PIXELS),Qi=U.getParameter(U.UNPACK_SKIP_ROWS),Ke=U.getParameter(U.UNPACK_SKIP_IMAGES);U.pixelStorei(U.UNPACK_ROW_LENGTH,ue.width),U.pixelStorei(U.UNPACK_IMAGE_HEIGHT,ue.height),U.pixelStorei(U.UNPACK_SKIP_PIXELS,vt),U.pixelStorei(U.UNPACK_SKIP_ROWS,Lt),U.pixelStorei(U.UNPACK_SKIP_IMAGES,Nt);const Gs=E.isDataArrayTexture||E.isData3DTexture,de=I.isDataArrayTexture||I.isData3DTexture;if(E.isRenderTargetTexture||E.isDepthTexture){const yn=wt.get(E),Ws=wt.get(I),sn=wt.get(yn.__renderTarget),Qn=wt.get(Ws.__renderTarget);At.bindFramebuffer(U.READ_FRAMEBUFFER,sn.__webglFramebuffer),At.bindFramebuffer(U.DRAW_FRAMEBUFFER,Qn.__webglFramebuffer);for(let ti=0;ti<xt;ti++)Gs&&U.framebufferTextureLayer(U.READ_FRAMEBUFFER,U.COLOR_ATTACHMENT0,wt.get(E).__webglTexture,O,Nt+ti),E.isDepthTexture?(de&&U.framebufferTextureLayer(U.DRAW_FRAMEBUFFER,U.COLOR_ATTACHMENT0,wt.get(I).__webglTexture,O,le+ti),U.blitFramebuffer(vt,Lt,it,ut,bt,Jt,it,ut,U.DEPTH_BUFFER_BIT,U.NEAREST)):de?U.copyTexSubImage3D(Mt,O,bt,Jt,le+ti,vt,Lt,it,ut):U.copyTexSubImage2D(Mt,O,bt,Jt,le+ti,vt,Lt,it,ut);At.bindFramebuffer(U.READ_FRAMEBUFFER,null),At.bindFramebuffer(U.DRAW_FRAMEBUFFER,null)}else de?E.isDataTexture||E.isData3DTexture?U.texSubImage3D(Mt,O,bt,Jt,le,it,ut,xt,Xe,Qt,ue.data):I.isCompressedArrayTexture?U.compressedTexSubImage3D(Mt,O,bt,Jt,le,it,ut,xt,Xe,ue.data):U.texSubImage3D(Mt,O,bt,Jt,le,it,ut,xt,Xe,Qt,ue):E.isDataTexture?U.texSubImage2D(U.TEXTURE_2D,O,bt,Jt,it,ut,Xe,Qt,ue.data):E.isCompressedTexture?U.compressedTexSubImage2D(U.TEXTURE_2D,O,bt,Jt,ue.width,ue.height,Xe,ue.data):U.texSubImage2D(U.TEXTURE_2D,O,bt,Jt,it,ut,Xe,Qt,ue);U.pixelStorei(U.UNPACK_ROW_LENGTH,Ln),U.pixelStorei(U.UNPACK_IMAGE_HEIGHT,te),U.pixelStorei(U.UNPACK_SKIP_PIXELS,un),U.pixelStorei(U.UNPACK_SKIP_ROWS,Qi),U.pixelStorei(U.UNPACK_SKIP_IMAGES,Ke),O===0&&I.generateMipmaps&&U.generateMipmap(Mt),At.unbindTexture()},this.copyTextureToTexture3D=function(E,I,B=null,z=null,O=0){return E.isTexture!==!0&&(or("WebGLRenderer: copyTextureToTexture3D function signature has changed."),B=arguments[0]||null,z=arguments[1]||null,E=arguments[2],I=arguments[3],O=arguments[4]||0),or('WebGLRenderer: copyTextureToTexture3D function has been deprecated. Use "copyTextureToTexture" instead.'),this.copyTextureToTexture(E,I,B,z,O)},this.initRenderTarget=function(E){wt.get(E).__webglFramebuffer===void 0&&C.setupRenderTarget(E)},this.initTexture=function(E){E.isCubeTexture?C.setTextureCube(E,0):E.isData3DTexture?C.setTexture3D(E,0):E.isDataArrayTexture||E.isCompressedArrayTexture?C.setTexture2DArray(E,0):C.setTexture2D(E,0),At.unbindTexture()},this.resetState=function(){w=0,T=0,R=null,At.reset(),ae.reset()},typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("observe",{detail:this}))}get coordinateSystem(){return $n}get outputColorSpace(){return this._outputColorSpace}set outputColorSpace(t){this._outputColorSpace=t;const e=this.getContext();e.drawingBufferColorspace=Kt._getDrawingBufferColorSpace(t),e.unpackColorSpace=Kt._getUnpackColorSpace()}}class wb extends Ce{constructor(){super(),this.isScene=!0,this.type="Scene",this.background=null,this.environment=null,this.fog=null,this.backgroundBlurriness=0,this.backgroundIntensity=1,this.backgroundRotation=new Pn,this.environmentIntensity=1,this.environmentRotation=new Pn,this.overrideMaterial=null,typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("observe",{detail:this}))}copy(t,e){return super.copy(t,e),t.background!==null&&(this.background=t.background.clone()),t.environment!==null&&(this.environment=t.environment.clone()),t.fog!==null&&(this.fog=t.fog.clone()),this.backgroundBlurriness=t.backgroundBlurriness,this.backgroundIntensity=t.backgroundIntensity,this.backgroundRotation.copy(t.backgroundRotation),this.environmentIntensity=t.environmentIntensity,this.environmentRotation.copy(t.environmentRotation),t.overrideMaterial!==null&&(this.overrideMaterial=t.overrideMaterial.clone()),this.matrixAutoUpdate=t.matrixAutoUpdate,this}toJSON(t){const e=super.toJSON(t);return this.fog!==null&&(e.object.fog=this.fog.toJSON()),this.backgroundBlurriness>0&&(e.object.backgroundBlurriness=this.backgroundBlurriness),this.backgroundIntensity!==1&&(e.object.backgroundIntensity=this.backgroundIntensity),e.object.backgroundRotation=this.backgroundRotation.toArray(),this.environmentIntensity!==1&&(e.object.environmentIntensity=this.environmentIntensity),e.object.environmentRotation=this.environmentRotation.toArray(),e}}class Ab{constructor(t,e){this.isInterleavedBuffer=!0,this.array=t,this.stride=e,this.count=t!==void 0?t.length/e:0,this.usage=ac,this.updateRanges=[],this.version=0,this.uuid=mi()}onUploadCallback(){}set needsUpdate(t){t===!0&&this.version++}setUsage(t){return this.usage=t,this}addUpdateRange(t,e){this.updateRanges.push({start:t,count:e})}clearUpdateRanges(){this.updateRanges.length=0}copy(t){return this.array=new t.array.constructor(t.array),this.count=t.count,this.stride=t.stride,this.usage=t.usage,this}copyAt(t,e,i){t*=this.stride,i*=e.stride;for(let s=0,r=this.stride;s<r;s++)this.array[t+s]=e.array[i+s];return this}set(t,e=0){return this.array.set(t,e),this}clone(t){t.arrayBuffers===void 0&&(t.arrayBuffers={}),this.array.buffer._uuid===void 0&&(this.array.buffer._uuid=mi()),t.arrayBuffers[this.array.buffer._uuid]===void 0&&(t.arrayBuffers[this.array.buffer._uuid]=this.array.slice(0).buffer);const e=new this.array.constructor(t.arrayBuffers[this.array.buffer._uuid]),i=new this.constructor(e,this.stride);return i.setUsage(this.usage),i}onUpload(t){return this.onUploadCallback=t,this}toJSON(t){return t.arrayBuffers===void 0&&(t.arrayBuffers={}),this.array.buffer._uuid===void 0&&(this.array.buffer._uuid=mi()),t.arrayBuffers[this.array.buffer._uuid]===void 0&&(t.arrayBuffers[this.array.buffer._uuid]=Array.from(new Uint32Array(this.array.buffer))),{uuid:this.uuid,buffer:this.array.buffer._uuid,type:this.array.constructor.name,stride:this.stride}}}const He=new L;class Qo{constructor(t,e,i,s=!1){this.isInterleavedBufferAttribute=!0,this.name="",this.data=t,this.itemSize=e,this.offset=i,this.normalized=s}get count(){return this.data.count}get array(){return this.data.array}set needsUpdate(t){this.data.needsUpdate=t}applyMatrix4(t){for(let e=0,i=this.data.count;e<i;e++)He.fromBufferAttribute(this,e),He.applyMatrix4(t),this.setXYZ(e,He.x,He.y,He.z);return this}applyNormalMatrix(t){for(let e=0,i=this.count;e<i;e++)He.fromBufferAttribute(this,e),He.applyNormalMatrix(t),this.setXYZ(e,He.x,He.y,He.z);return this}transformDirection(t){for(let e=0,i=this.count;e<i;e++)He.fromBufferAttribute(this,e),He.transformDirection(t),this.setXYZ(e,He.x,He.y,He.z);return this}getComponent(t,e){let i=this.array[t*this.data.stride+this.offset+e];return this.normalized&&(i=wn(i,this.array)),i}setComponent(t,e,i){return this.normalized&&(i=se(i,this.array)),this.data.array[t*this.data.stride+this.offset+e]=i,this}setX(t,e){return this.normalized&&(e=se(e,this.array)),this.data.array[t*this.data.stride+this.offset]=e,this}setY(t,e){return this.normalized&&(e=se(e,this.array)),this.data.array[t*this.data.stride+this.offset+1]=e,this}setZ(t,e){return this.normalized&&(e=se(e,this.array)),this.data.array[t*this.data.stride+this.offset+2]=e,this}setW(t,e){return this.normalized&&(e=se(e,this.array)),this.data.array[t*this.data.stride+this.offset+3]=e,this}getX(t){let e=this.data.array[t*this.data.stride+this.offset];return this.normalized&&(e=wn(e,this.array)),e}getY(t){let e=this.data.array[t*this.data.stride+this.offset+1];return this.normalized&&(e=wn(e,this.array)),e}getZ(t){let e=this.data.array[t*this.data.stride+this.offset+2];return this.normalized&&(e=wn(e,this.array)),e}getW(t){let e=this.data.array[t*this.data.stride+this.offset+3];return this.normalized&&(e=wn(e,this.array)),e}setXY(t,e,i){return t=t*this.data.stride+this.offset,this.normalized&&(e=se(e,this.array),i=se(i,this.array)),this.data.array[t+0]=e,this.data.array[t+1]=i,this}setXYZ(t,e,i,s){return t=t*this.data.stride+this.offset,this.normalized&&(e=se(e,this.array),i=se(i,this.array),s=se(s,this.array)),this.data.array[t+0]=e,this.data.array[t+1]=i,this.data.array[t+2]=s,this}setXYZW(t,e,i,s,r){return t=t*this.data.stride+this.offset,this.normalized&&(e=se(e,this.array),i=se(i,this.array),s=se(s,this.array),r=se(r,this.array)),this.data.array[t+0]=e,this.data.array[t+1]=i,this.data.array[t+2]=s,this.data.array[t+3]=r,this}clone(t){if(t===void 0){console.log("THREE.InterleavedBufferAttribute.clone(): Cloning an interleaved buffer attribute will de-interleave buffer data.");const e=[];for(let i=0;i<this.count;i++){const s=i*this.data.stride+this.offset;for(let r=0;r<this.itemSize;r++)e.push(this.data.array[s+r])}return new en(new this.array.constructor(e),this.itemSize,this.normalized)}else return t.interleavedBuffers===void 0&&(t.interleavedBuffers={}),t.interleavedBuffers[this.data.uuid]===void 0&&(t.interleavedBuffers[this.data.uuid]=this.data.clone(t)),new Qo(t.interleavedBuffers[this.data.uuid],this.itemSize,this.offset,this.normalized)}toJSON(t){if(t===void 0){console.log("THREE.InterleavedBufferAttribute.toJSON(): Serializing an interleaved buffer attribute will de-interleave buffer data.");const e=[];for(let i=0;i<this.count;i++){const s=i*this.data.stride+this.offset;for(let r=0;r<this.itemSize;r++)e.push(this.data.array[s+r])}return{itemSize:this.itemSize,type:this.array.constructor.name,array:e,normalized:this.normalized}}else return t.interleavedBuffers===void 0&&(t.interleavedBuffers={}),t.interleavedBuffers[this.data.uuid]===void 0&&(t.interleavedBuffers[this.data.uuid]=this.data.toJSON(t)),{isInterleavedBufferAttribute:!0,itemSize:this.itemSize,data:this.data.uuid,offset:this.offset,normalized:this.normalized}}}class Yf extends yi{static get type(){return"SpriteMaterial"}constructor(t){super(),this.isSpriteMaterial=!0,this.color=new Ht(16777215),this.map=null,this.alphaMap=null,this.rotation=0,this.sizeAttenuation=!0,this.transparent=!0,this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.color.copy(t.color),this.map=t.map,this.alphaMap=t.alphaMap,this.rotation=t.rotation,this.sizeAttenuation=t.sizeAttenuation,this.fog=t.fog,this}}let ps;const Ks=new L,ms=new L,gs=new L,_s=new Tt,Zs=new Tt,qf=new oe,co=new L,Js=new L,ho=new L,vu=new Tt,il=new Tt,bu=new Tt;class Cb extends Ce{constructor(t=new Yf){if(super(),this.isSprite=!0,this.type="Sprite",ps===void 0){ps=new Te;const e=new Float32Array([-.5,-.5,0,0,0,.5,-.5,0,1,0,.5,.5,0,1,1,-.5,.5,0,0,1]),i=new Ab(e,5);ps.setIndex([0,1,2,0,2,3]),ps.setAttribute("position",new Qo(i,3,0,!1)),ps.setAttribute("uv",new Qo(i,2,3,!1))}this.geometry=ps,this.material=t,this.center=new Tt(.5,.5)}raycast(t,e){t.camera===null&&console.error('THREE.Sprite: "Raycaster.camera" needs to be set in order to raycast against sprites.'),ms.setFromMatrixScale(this.matrixWorld),qf.copy(t.camera.matrixWorld),this.modelViewMatrix.multiplyMatrices(t.camera.matrixWorldInverse,this.matrixWorld),gs.setFromMatrixPosition(this.modelViewMatrix),t.camera.isPerspectiveCamera&&this.material.sizeAttenuation===!1&&ms.multiplyScalar(-gs.z);const i=this.material.rotation;let s,r;i!==0&&(r=Math.cos(i),s=Math.sin(i));const o=this.center;uo(co.set(-.5,-.5,0),gs,o,ms,s,r),uo(Js.set(.5,-.5,0),gs,o,ms,s,r),uo(ho.set(.5,.5,0),gs,o,ms,s,r),vu.set(0,0),il.set(1,0),bu.set(1,1);let a=t.ray.intersectTriangle(co,Js,ho,!1,Ks);if(a===null&&(uo(Js.set(-.5,.5,0),gs,o,ms,s,r),il.set(0,1),a=t.ray.intersectTriangle(co,ho,Js,!1,Ks),a===null))return;const l=t.ray.origin.distanceTo(Ks);l<t.near||l>t.far||e.push({distance:l,point:Ks.clone(),uv:cn.getInterpolation(Ks,co,Js,ho,vu,il,bu,new Tt),face:null,object:this})}copy(t,e){return super.copy(t,e),t.center!==void 0&&this.center.copy(t.center),this.material=t.material,this}}function uo(n,t,e,i,s,r){_s.subVectors(n,e).addScalar(.5).multiply(i),s!==void 0?(Zs.x=r*_s.x-s*_s.y,Zs.y=s*_s.x+r*_s.y):Zs.copy(_s),n.copy(t),n.x+=Zs.x,n.y+=Zs.y,n.applyMatrix4(qf)}class Nr extends yi{static get type(){return"LineBasicMaterial"}constructor(t){super(),this.isLineBasicMaterial=!0,this.color=new Ht(16777215),this.map=null,this.linewidth=1,this.linecap="round",this.linejoin="round",this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.color.copy(t.color),this.map=t.map,this.linewidth=t.linewidth,this.linecap=t.linecap,this.linejoin=t.linejoin,this.fog=t.fog,this}}const ta=new L,ea=new L,yu=new oe,Qs=new Or,fo=new Ir,sl=new L,Mu=new L;class hc extends Ce{constructor(t=new Te,e=new Nr){super(),this.isLine=!0,this.type="Line",this.geometry=t,this.material=e,this.updateMorphTargets()}copy(t,e){return super.copy(t,e),this.material=Array.isArray(t.material)?t.material.slice():t.material,this.geometry=t.geometry,this}computeLineDistances(){const t=this.geometry;if(t.index===null){const e=t.attributes.position,i=[0];for(let s=1,r=e.count;s<r;s++)ta.fromBufferAttribute(e,s-1),ea.fromBufferAttribute(e,s),i[s]=i[s-1],i[s]+=ta.distanceTo(ea);t.setAttribute("lineDistance",new Se(i,1))}else console.warn("THREE.Line.computeLineDistances(): Computation only possible with non-indexed BufferGeometry.");return this}raycast(t,e){const i=this.geometry,s=this.matrixWorld,r=t.params.Line.threshold,o=i.drawRange;if(i.boundingSphere===null&&i.computeBoundingSphere(),fo.copy(i.boundingSphere),fo.applyMatrix4(s),fo.radius+=r,t.ray.intersectsSphere(fo)===!1)return;yu.copy(s).invert(),Qs.copy(t.ray).applyMatrix4(yu);const a=r/((this.scale.x+this.scale.y+this.scale.z)/3),l=a*a,c=this.isLineSegments?2:1,h=i.index,d=i.attributes.position;if(h!==null){const f=Math.max(0,o.start),g=Math.min(h.count,o.start+o.count);for(let _=f,m=g-1;_<m;_+=c){const p=h.getX(_),v=h.getX(_+1),y=po(this,t,Qs,l,p,v);y&&e.push(y)}if(this.isLineLoop){const _=h.getX(g-1),m=h.getX(f),p=po(this,t,Qs,l,_,m);p&&e.push(p)}}else{const f=Math.max(0,o.start),g=Math.min(d.count,o.start+o.count);for(let _=f,m=g-1;_<m;_+=c){const p=po(this,t,Qs,l,_,_+1);p&&e.push(p)}if(this.isLineLoop){const _=po(this,t,Qs,l,g-1,f);_&&e.push(_)}}}updateMorphTargets(){const e=this.geometry.morphAttributes,i=Object.keys(e);if(i.length>0){const s=e[i[0]];if(s!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let r=0,o=s.length;r<o;r++){const a=s[r].name||String(r);this.morphTargetInfluences.push(0),this.morphTargetDictionary[a]=r}}}}}function po(n,t,e,i,s,r){const o=n.geometry.attributes.position;if(ta.fromBufferAttribute(o,s),ea.fromBufferAttribute(o,r),e.distanceSqToSegment(ta,ea,sl,Mu)>i)return;sl.applyMatrix4(n.matrixWorld);const l=t.ray.origin.distanceTo(sl);if(!(l<t.near||l>t.far))return{distance:l,point:Mu.clone().applyMatrix4(n.matrixWorld),index:s,face:null,faceIndex:null,barycoord:null,object:n}}const Su=new L,Eu=new L;class Kf extends hc{constructor(t,e){super(t,e),this.isLineSegments=!0,this.type="LineSegments"}computeLineDistances(){const t=this.geometry;if(t.index===null){const e=t.attributes.position,i=[];for(let s=0,r=e.count;s<r;s+=2)Su.fromBufferAttribute(e,s),Eu.fromBufferAttribute(e,s+1),i[s]=s===0?0:i[s-1],i[s+1]=i[s]+Su.distanceTo(Eu);t.setAttribute("lineDistance",new Se(i,1))}else console.warn("THREE.LineSegments.computeLineDistances(): Computation only possible with non-indexed BufferGeometry.");return this}}class Zf extends yi{static get type(){return"PointsMaterial"}constructor(t){super(),this.isPointsMaterial=!0,this.color=new Ht(16777215),this.map=null,this.alphaMap=null,this.size=1,this.sizeAttenuation=!0,this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.color.copy(t.color),this.map=t.map,this.alphaMap=t.alphaMap,this.size=t.size,this.sizeAttenuation=t.sizeAttenuation,this.fog=t.fog,this}}const Tu=new oe,uc=new Or,mo=new Ir,go=new L;class Rb extends Ce{constructor(t=new Te,e=new Zf){super(),this.isPoints=!0,this.type="Points",this.geometry=t,this.material=e,this.updateMorphTargets()}copy(t,e){return super.copy(t,e),this.material=Array.isArray(t.material)?t.material.slice():t.material,this.geometry=t.geometry,this}raycast(t,e){const i=this.geometry,s=this.matrixWorld,r=t.params.Points.threshold,o=i.drawRange;if(i.boundingSphere===null&&i.computeBoundingSphere(),mo.copy(i.boundingSphere),mo.applyMatrix4(s),mo.radius+=r,t.ray.intersectsSphere(mo)===!1)return;Tu.copy(s).invert(),uc.copy(t.ray).applyMatrix4(Tu);const a=r/((this.scale.x+this.scale.y+this.scale.z)/3),l=a*a,c=i.index,u=i.attributes.position;if(c!==null){const d=Math.max(0,o.start),f=Math.min(c.count,o.start+o.count);for(let g=d,_=f;g<_;g++){const m=c.getX(g);go.fromBufferAttribute(u,m),wu(go,m,l,s,t,e,this)}}else{const d=Math.max(0,o.start),f=Math.min(u.count,o.start+o.count);for(let g=d,_=f;g<_;g++)go.fromBufferAttribute(u,g),wu(go,g,l,s,t,e,this)}}updateMorphTargets(){const e=this.geometry.morphAttributes,i=Object.keys(e);if(i.length>0){const s=e[i[0]];if(s!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let r=0,o=s.length;r<o;r++){const a=s[r].name||String(r);this.morphTargetInfluences.push(0),this.morphTargetDictionary[a]=r}}}}}function wu(n,t,e,i,s,r,o){const a=uc.distanceSqToPoint(n);if(a<e){const l=new L;uc.closestPointToPoint(n,l),l.applyMatrix4(i);const c=s.ray.origin.distanceTo(l);if(c<s.near||c>s.far)return;r.push({distance:c,distanceToRay:Math.sqrt(a),point:l,index:t,face:null,faceIndex:null,barycoord:null,object:o})}}class Pb extends We{constructor(t,e,i,s,r,o,a,l,c){super(t,e,i,s,r,o,a,l,c),this.isCanvasTexture=!0,this.needsUpdate=!0}}class Vc extends Te{constructor(t=1,e=32,i=0,s=Math.PI*2){super(),this.type="CircleGeometry",this.parameters={radius:t,segments:e,thetaStart:i,thetaLength:s},e=Math.max(3,e);const r=[],o=[],a=[],l=[],c=new L,h=new Tt;o.push(0,0,0),a.push(0,0,1),l.push(.5,.5);for(let u=0,d=3;u<=e;u++,d+=3){const f=i+u/e*s;c.x=t*Math.cos(f),c.y=t*Math.sin(f),o.push(c.x,c.y,c.z),a.push(0,0,1),h.x=(o[d]/t+1)/2,h.y=(o[d+1]/t+1)/2,l.push(h.x,h.y)}for(let u=1;u<=e;u++)r.push(u,u+1,0);this.setIndex(r),this.setAttribute("position",new Se(o,3)),this.setAttribute("normal",new Se(a,3)),this.setAttribute("uv",new Se(l,2))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new Vc(t.radius,t.segments,t.thetaStart,t.thetaLength)}}class Es extends Te{constructor(t=1,e=1,i=1,s=32,r=1,o=!1,a=0,l=Math.PI*2){super(),this.type="CylinderGeometry",this.parameters={radiusTop:t,radiusBottom:e,height:i,radialSegments:s,heightSegments:r,openEnded:o,thetaStart:a,thetaLength:l};const c=this;s=Math.floor(s),r=Math.floor(r);const h=[],u=[],d=[],f=[];let g=0;const _=[],m=i/2;let p=0;v(),o===!1&&(t>0&&y(!0),e>0&&y(!1)),this.setIndex(h),this.setAttribute("position",new Se(u,3)),this.setAttribute("normal",new Se(d,3)),this.setAttribute("uv",new Se(f,2));function v(){const x=new L,A=new L;let w=0;const T=(e-t)/i;for(let R=0;R<=r;R++){const S=[],b=R/r,P=b*(e-t)+t;for(let N=0;N<=s;N++){const F=N/s,W=F*l+a,G=Math.sin(W),H=Math.cos(W);A.x=P*G,A.y=-b*i+m,A.z=P*H,u.push(A.x,A.y,A.z),x.set(G,T,H).normalize(),d.push(x.x,x.y,x.z),f.push(F,1-b),S.push(g++)}_.push(S)}for(let R=0;R<s;R++)for(let S=0;S<r;S++){const b=_[S][R],P=_[S+1][R],N=_[S+1][R+1],F=_[S][R+1];(t>0||S!==0)&&(h.push(b,P,F),w+=3),(e>0||S!==r-1)&&(h.push(P,N,F),w+=3)}c.addGroup(p,w,0),p+=w}function y(x){const A=g,w=new Tt,T=new L;let R=0;const S=x===!0?t:e,b=x===!0?1:-1;for(let N=1;N<=s;N++)u.push(0,m*b,0),d.push(0,b,0),f.push(.5,.5),g++;const P=g;for(let N=0;N<=s;N++){const W=N/s*l+a,G=Math.cos(W),H=Math.sin(W);T.x=S*H,T.y=m*b,T.z=S*G,u.push(T.x,T.y,T.z),d.push(0,b,0),w.x=G*.5+.5,w.y=H*.5*b+.5,f.push(w.x,w.y),g++}for(let N=0;N<s;N++){const F=A+N,W=P+N;x===!0?h.push(W,W+1,F):h.push(W+1,W,F),R+=3}c.addGroup(p,R,x===!0?1:2),p+=R}}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new Es(t.radiusTop,t.radiusBottom,t.height,t.radialSegments,t.heightSegments,t.openEnded,t.thetaStart,t.thetaLength)}}class pr extends Te{constructor(t=1,e=32,i=16,s=0,r=Math.PI*2,o=0,a=Math.PI){super(),this.type="SphereGeometry",this.parameters={radius:t,widthSegments:e,heightSegments:i,phiStart:s,phiLength:r,thetaStart:o,thetaLength:a},e=Math.max(3,Math.floor(e)),i=Math.max(2,Math.floor(i));const l=Math.min(o+a,Math.PI);let c=0;const h=[],u=new L,d=new L,f=[],g=[],_=[],m=[];for(let p=0;p<=i;p++){const v=[],y=p/i;let x=0;p===0&&o===0?x=.5/e:p===i&&l===Math.PI&&(x=-.5/e);for(let A=0;A<=e;A++){const w=A/e;u.x=-t*Math.cos(s+w*r)*Math.sin(o+y*a),u.y=t*Math.cos(o+y*a),u.z=t*Math.sin(s+w*r)*Math.sin(o+y*a),g.push(u.x,u.y,u.z),d.copy(u).normalize(),_.push(d.x,d.y,d.z),m.push(w+x,1-y),v.push(c++)}h.push(v)}for(let p=0;p<i;p++)for(let v=0;v<e;v++){const y=h[p][v+1],x=h[p][v],A=h[p+1][v],w=h[p+1][v+1];(p!==0||o>0)&&f.push(y,x,w),(p!==i-1||l<Math.PI)&&f.push(x,A,w)}this.setIndex(f),this.setAttribute("position",new Se(g,3)),this.setAttribute("normal",new Se(_,3)),this.setAttribute("uv",new Se(m,2))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new pr(t.radius,t.widthSegments,t.heightSegments,t.phiStart,t.phiLength,t.thetaStart,t.thetaLength)}}class xs extends yi{static get type(){return"MeshStandardMaterial"}constructor(t){super(),this.isMeshStandardMaterial=!0,this.defines={STANDARD:""},this.color=new Ht(16777215),this.roughness=1,this.metalness=0,this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.emissive=new Ht(0),this.emissiveIntensity=1,this.emissiveMap=null,this.bumpMap=null,this.bumpScale=1,this.normalMap=null,this.normalMapType=Pf,this.normalScale=new Tt(1,1),this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.roughnessMap=null,this.metalnessMap=null,this.alphaMap=null,this.envMap=null,this.envMapRotation=new Pn,this.envMapIntensity=1,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.flatShading=!1,this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.defines={STANDARD:""},this.color.copy(t.color),this.roughness=t.roughness,this.metalness=t.metalness,this.map=t.map,this.lightMap=t.lightMap,this.lightMapIntensity=t.lightMapIntensity,this.aoMap=t.aoMap,this.aoMapIntensity=t.aoMapIntensity,this.emissive.copy(t.emissive),this.emissiveMap=t.emissiveMap,this.emissiveIntensity=t.emissiveIntensity,this.bumpMap=t.bumpMap,this.bumpScale=t.bumpScale,this.normalMap=t.normalMap,this.normalMapType=t.normalMapType,this.normalScale.copy(t.normalScale),this.displacementMap=t.displacementMap,this.displacementScale=t.displacementScale,this.displacementBias=t.displacementBias,this.roughnessMap=t.roughnessMap,this.metalnessMap=t.metalnessMap,this.alphaMap=t.alphaMap,this.envMap=t.envMap,this.envMapRotation.copy(t.envMapRotation),this.envMapIntensity=t.envMapIntensity,this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this.wireframeLinecap=t.wireframeLinecap,this.wireframeLinejoin=t.wireframeLinejoin,this.flatShading=t.flatShading,this.fog=t.fog,this}}class Db extends Nr{static get type(){return"LineDashedMaterial"}constructor(t){super(),this.isLineDashedMaterial=!0,this.scale=1,this.dashSize=3,this.gapSize=1,this.setValues(t)}copy(t){return super.copy(t),this.scale=t.scale,this.dashSize=t.dashSize,this.gapSize=t.gapSize,this}}class Jf extends Ce{constructor(t,e=1){super(),this.isLight=!0,this.type="Light",this.color=new Ht(t),this.intensity=e}dispose(){}copy(t,e){return super.copy(t,e),this.color.copy(t.color),this.intensity=t.intensity,this}toJSON(t){const e=super.toJSON(t);return e.object.color=this.color.getHex(),e.object.intensity=this.intensity,this.groundColor!==void 0&&(e.object.groundColor=this.groundColor.getHex()),this.distance!==void 0&&(e.object.distance=this.distance),this.angle!==void 0&&(e.object.angle=this.angle),this.decay!==void 0&&(e.object.decay=this.decay),this.penumbra!==void 0&&(e.object.penumbra=this.penumbra),this.shadow!==void 0&&(e.object.shadow=this.shadow.toJSON()),this.target!==void 0&&(e.object.target=this.target.uuid),e}}class Lb extends Jf{constructor(t,e,i){super(t,i),this.isHemisphereLight=!0,this.type="HemisphereLight",this.position.copy(Ce.DEFAULT_UP),this.updateMatrix(),this.groundColor=new Ht(e)}copy(t,e){return super.copy(t,e),this.groundColor.copy(t.groundColor),this}}const rl=new oe,Au=new L,Cu=new L;class Ib{constructor(t){this.camera=t,this.intensity=1,this.bias=0,this.normalBias=0,this.radius=1,this.blurSamples=8,this.mapSize=new Tt(512,512),this.map=null,this.mapPass=null,this.matrix=new oe,this.autoUpdate=!0,this.needsUpdate=!1,this._frustum=new zc,this._frameExtents=new Tt(1,1),this._viewportCount=1,this._viewports=[new be(0,0,1,1)]}getViewportCount(){return this._viewportCount}getFrustum(){return this._frustum}updateMatrices(t){const e=this.camera,i=this.matrix;Au.setFromMatrixPosition(t.matrixWorld),e.position.copy(Au),Cu.setFromMatrixPosition(t.target.matrixWorld),e.lookAt(Cu),e.updateMatrixWorld(),rl.multiplyMatrices(e.projectionMatrix,e.matrixWorldInverse),this._frustum.setFromProjectionMatrix(rl),i.set(.5,0,0,.5,0,.5,0,.5,0,0,.5,.5,0,0,0,1),i.multiply(rl)}getViewport(t){return this._viewports[t]}getFrameExtents(){return this._frameExtents}dispose(){this.map&&this.map.dispose(),this.mapPass&&this.mapPass.dispose()}copy(t){return this.camera=t.camera.clone(),this.intensity=t.intensity,this.bias=t.bias,this.radius=t.radius,this.mapSize.copy(t.mapSize),this}clone(){return new this.constructor().copy(this)}toJSON(){const t={};return this.intensity!==1&&(t.intensity=this.intensity),this.bias!==0&&(t.bias=this.bias),this.normalBias!==0&&(t.normalBias=this.normalBias),this.radius!==1&&(t.radius=this.radius),(this.mapSize.x!==512||this.mapSize.y!==512)&&(t.mapSize=this.mapSize.toArray()),t.camera=this.camera.toJSON(!1).object,delete t.camera.matrix,t}}class Ob extends Ib{constructor(){super(new Vf(-5,5,5,-5,.5,500)),this.isDirectionalLightShadow=!0}}class Ub extends Jf{constructor(t,e){super(t,e),this.isDirectionalLight=!0,this.type="DirectionalLight",this.position.copy(Ce.DEFAULT_UP),this.updateMatrix(),this.target=new Ce,this.shadow=new Ob}dispose(){this.shadow.dispose()}copy(t){return super.copy(t),this.target=t.target.clone(),this.shadow=t.shadow.clone(),this}}const Ru=new oe;class Nb{constructor(t,e,i=0,s=1/0){this.ray=new Or(t,e),this.near=i,this.far=s,this.camera=null,this.layers=new Bc,this.params={Mesh:{},Line:{threshold:1},LOD:{},Points:{threshold:1},Sprite:{}}}set(t,e){this.ray.set(t,e)}setFromCamera(t,e){e.isPerspectiveCamera?(this.ray.origin.setFromMatrixPosition(e.matrixWorld),this.ray.direction.set(t.x,t.y,.5).unproject(e).sub(this.ray.origin).normalize(),this.camera=e):e.isOrthographicCamera?(this.ray.origin.set(t.x,t.y,(e.near+e.far)/(e.near-e.far)).unproject(e),this.ray.direction.set(0,0,-1).transformDirection(e.matrixWorld),this.camera=e):console.error("THREE.Raycaster: Unsupported camera type: "+e.type)}setFromXRController(t){return Ru.identity().extractRotation(t.matrixWorld),this.ray.origin.setFromMatrixPosition(t.matrixWorld),this.ray.direction.set(0,0,-1).applyMatrix4(Ru),this}intersectObject(t,e=!0,i=[]){return dc(t,this,i,e),i.sort(Pu),i}intersectObjects(t,e=!0,i=[]){for(let s=0,r=t.length;s<r;s++)dc(t[s],this,i,e);return i.sort(Pu),i}}function Pu(n,t){return n.distance-t.distance}function dc(n,t,e,i){let s=!0;if(n.layers.test(t.layers)&&n.raycast(t,e)===!1&&(s=!1),s===!0&&i===!0){const r=n.children;for(let o=0,a=r.length;o<a;o++)dc(r[o],t,e,!0)}}class Du{constructor(t=1,e=0,i=0){return this.radius=t,this.phi=e,this.theta=i,this}set(t,e,i){return this.radius=t,this.phi=e,this.theta=i,this}copy(t){return this.radius=t.radius,this.phi=t.phi,this.theta=t.theta,this}makeSafe(){return this.phi=Math.max(1e-6,Math.min(Math.PI-1e-6,this.phi)),this}setFromVector3(t){return this.setFromCartesianCoords(t.x,t.y,t.z)}setFromCartesianCoords(t,e,i){return this.radius=Math.sqrt(t*t+e*e+i*i),this.radius===0?(this.theta=0,this.phi=0):(this.theta=Math.atan2(t,i),this.phi=Math.acos(Ge(e/this.radius,-1,1))),this}clone(){return new this.constructor().copy(this)}}class Fb extends Kf{constructor(t=10,e=10,i=4473924,s=8947848){i=new Ht(i),s=new Ht(s);const r=e/2,o=t/e,a=t/2,l=[],c=[];for(let d=0,f=0,g=-a;d<=e;d++,g+=o){l.push(-a,0,g,a,0,g),l.push(g,0,-a,g,0,a);const _=d===r?i:s;_.toArray(c,f),f+=3,_.toArray(c,f),f+=3,_.toArray(c,f),f+=3,_.toArray(c,f),f+=3}const h=new Te;h.setAttribute("position",new Se(l,3)),h.setAttribute("color",new Se(c,3));const u=new Nr({vertexColors:!0,toneMapped:!1});super(h,u),this.type="GridHelper"}dispose(){this.geometry.dispose(),this.material.dispose()}}class Lu extends Kf{constructor(t=1){const e=[0,0,0,t,0,0,0,0,0,0,t,0,0,0,0,0,0,t],i=[1,0,0,1,.6,0,0,1,0,.6,1,0,0,0,1,0,.6,1],s=new Te;s.setAttribute("position",new Se(e,3)),s.setAttribute("color",new Se(i,3));const r=new Nr({vertexColors:!0,toneMapped:!1});super(s,r),this.type="AxesHelper"}setColors(t,e,i){const s=new Ht,r=this.geometry.attributes.color.array;return s.set(t),s.toArray(r,0),s.toArray(r,3),s.set(e),s.toArray(r,6),s.toArray(r,9),s.set(i),s.toArray(r,12),s.toArray(r,15),this.geometry.attributes.color.needsUpdate=!0,this}dispose(){this.geometry.dispose(),this.material.dispose()}}class kb extends Ki{constructor(t,e=null){super(),this.object=t,this.domElement=e,this.enabled=!0,this.state=-1,this.keys={},this.mouseButtons={LEFT:null,MIDDLE:null,RIGHT:null},this.touches={ONE:null,TWO:null}}connect(){}disconnect(){}dispose(){}update(){}}typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("register",{detail:{revision:Lc}}));typeof window<"u"&&(window.__THREE__?console.warn("WARNING: Multiple instances of Three.js being imported."):window.__THREE__=Lc);const Iu={type:"change"},Gc={type:"start"},Qf={type:"end"},_o=new Or,Ou=new ai,Bb=Math.cos(70*dg.DEG2RAD),Re=new L,je=2*Math.PI,re={NONE:-1,ROTATE:0,DOLLY:1,PAN:2,TOUCH_ROTATE:3,TOUCH_PAN:4,TOUCH_DOLLY_PAN:5,TOUCH_DOLLY_ROTATE:6},ol=1e-6;class zb extends kb{constructor(t,e=null){super(t,e),this.state=re.NONE,this.enabled=!0,this.target=new L,this.cursor=new L,this.minDistance=0,this.maxDistance=1/0,this.minZoom=0,this.maxZoom=1/0,this.minTargetRadius=0,this.maxTargetRadius=1/0,this.minPolarAngle=0,this.maxPolarAngle=Math.PI,this.minAzimuthAngle=-1/0,this.maxAzimuthAngle=1/0,this.enableDamping=!1,this.dampingFactor=.05,this.enableZoom=!0,this.zoomSpeed=1,this.enableRotate=!0,this.rotateSpeed=1,this.enablePan=!0,this.panSpeed=1,this.screenSpacePanning=!0,this.keyPanSpeed=7,this.zoomToCursor=!1,this.autoRotate=!1,this.autoRotateSpeed=2,this.keys={LEFT:"ArrowLeft",UP:"ArrowUp",RIGHT:"ArrowRight",BOTTOM:"ArrowDown"},this.mouseButtons={LEFT:Ts.ROTATE,MIDDLE:Ts.DOLLY,RIGHT:Ts.PAN},this.touches={ONE:ys.ROTATE,TWO:ys.DOLLY_PAN},this.target0=this.target.clone(),this.position0=this.object.position.clone(),this.zoom0=this.object.zoom,this._domElementKeyEvents=null,this._lastPosition=new L,this._lastQuaternion=new $i,this._lastTargetPosition=new L,this._quat=new $i().setFromUnitVectors(t.up,new L(0,1,0)),this._quatInverse=this._quat.clone().invert(),this._spherical=new Du,this._sphericalDelta=new Du,this._scale=1,this._panOffset=new L,this._rotateStart=new Tt,this._rotateEnd=new Tt,this._rotateDelta=new Tt,this._panStart=new Tt,this._panEnd=new Tt,this._panDelta=new Tt,this._dollyStart=new Tt,this._dollyEnd=new Tt,this._dollyDelta=new Tt,this._dollyDirection=new L,this._mouse=new Tt,this._performCursorZoom=!1,this._pointers=[],this._pointerPositions={},this._controlActive=!1,this._onPointerMove=Vb.bind(this),this._onPointerDown=Hb.bind(this),this._onPointerUp=Gb.bind(this),this._onContextMenu=Kb.bind(this),this._onMouseWheel=jb.bind(this),this._onKeyDown=$b.bind(this),this._onTouchStart=Yb.bind(this),this._onTouchMove=qb.bind(this),this._onMouseDown=Wb.bind(this),this._onMouseMove=Xb.bind(this),this._interceptControlDown=Zb.bind(this),this._interceptControlUp=Jb.bind(this),this.domElement!==null&&this.connect(),this.update()}connect(){this.domElement.addEventListener("pointerdown",this._onPointerDown),this.domElement.addEventListener("pointercancel",this._onPointerUp),this.domElement.addEventListener("contextmenu",this._onContextMenu),this.domElement.addEventListener("wheel",this._onMouseWheel,{passive:!1}),this.domElement.getRootNode().addEventListener("keydown",this._interceptControlDown,{passive:!0,capture:!0}),this.domElement.style.touchAction="none"}disconnect(){this.domElement.removeEventListener("pointerdown",this._onPointerDown),this.domElement.removeEventListener("pointermove",this._onPointerMove),this.domElement.removeEventListener("pointerup",this._onPointerUp),this.domElement.removeEventListener("pointercancel",this._onPointerUp),this.domElement.removeEventListener("wheel",this._onMouseWheel),this.domElement.removeEventListener("contextmenu",this._onContextMenu),this.stopListenToKeyEvents(),this.domElement.getRootNode().removeEventListener("keydown",this._interceptControlDown,{capture:!0}),this.domElement.style.touchAction="auto"}dispose(){this.disconnect()}getPolarAngle(){return this._spherical.phi}getAzimuthalAngle(){return this._spherical.theta}getDistance(){return this.object.position.distanceTo(this.target)}listenToKeyEvents(t){t.addEventListener("keydown",this._onKeyDown),this._domElementKeyEvents=t}stopListenToKeyEvents(){this._domElementKeyEvents!==null&&(this._domElementKeyEvents.removeEventListener("keydown",this._onKeyDown),this._domElementKeyEvents=null)}saveState(){this.target0.copy(this.target),this.position0.copy(this.object.position),this.zoom0=this.object.zoom}reset(){this.target.copy(this.target0),this.object.position.copy(this.position0),this.object.zoom=this.zoom0,this.object.updateProjectionMatrix(),this.dispatchEvent(Iu),this.update(),this.state=re.NONE}update(t=null){const e=this.object.position;Re.copy(e).sub(this.target),Re.applyQuaternion(this._quat),this._spherical.setFromVector3(Re),this.autoRotate&&this.state===re.NONE&&this._rotateLeft(this._getAutoRotationAngle(t)),this.enableDamping?(this._spherical.theta+=this._sphericalDelta.theta*this.dampingFactor,this._spherical.phi+=this._sphericalDelta.phi*this.dampingFactor):(this._spherical.theta+=this._sphericalDelta.theta,this._spherical.phi+=this._sphericalDelta.phi);let i=this.minAzimuthAngle,s=this.maxAzimuthAngle;isFinite(i)&&isFinite(s)&&(i<-Math.PI?i+=je:i>Math.PI&&(i-=je),s<-Math.PI?s+=je:s>Math.PI&&(s-=je),i<=s?this._spherical.theta=Math.max(i,Math.min(s,this._spherical.theta)):this._spherical.theta=this._spherical.theta>(i+s)/2?Math.max(i,this._spherical.theta):Math.min(s,this._spherical.theta)),this._spherical.phi=Math.max(this.minPolarAngle,Math.min(this.maxPolarAngle,this._spherical.phi)),this._spherical.makeSafe(),this.enableDamping===!0?this.target.addScaledVector(this._panOffset,this.dampingFactor):this.target.add(this._panOffset),this.target.sub(this.cursor),this.target.clampLength(this.minTargetRadius,this.maxTargetRadius),this.target.add(this.cursor);let r=!1;if(this.zoomToCursor&&this._performCursorZoom||this.object.isOrthographicCamera)this._spherical.radius=this._clampDistance(this._spherical.radius);else{const o=this._spherical.radius;this._spherical.radius=this._clampDistance(this._spherical.radius*this._scale),r=o!=this._spherical.radius}if(Re.setFromSpherical(this._spherical),Re.applyQuaternion(this._quatInverse),e.copy(this.target).add(Re),this.object.lookAt(this.target),this.enableDamping===!0?(this._sphericalDelta.theta*=1-this.dampingFactor,this._sphericalDelta.phi*=1-this.dampingFactor,this._panOffset.multiplyScalar(1-this.dampingFactor)):(this._sphericalDelta.set(0,0,0),this._panOffset.set(0,0,0)),this.zoomToCursor&&this._performCursorZoom){let o=null;if(this.object.isPerspectiveCamera){const a=Re.length();o=this._clampDistance(a*this._scale);const l=a-o;this.object.position.addScaledVector(this._dollyDirection,l),this.object.updateMatrixWorld(),r=!!l}else if(this.object.isOrthographicCamera){const a=new L(this._mouse.x,this._mouse.y,0);a.unproject(this.object);const l=this.object.zoom;this.object.zoom=Math.max(this.minZoom,Math.min(this.maxZoom,this.object.zoom/this._scale)),this.object.updateProjectionMatrix(),r=l!==this.object.zoom;const c=new L(this._mouse.x,this._mouse.y,0);c.unproject(this.object),this.object.position.sub(c).add(a),this.object.updateMatrixWorld(),o=Re.length()}else console.warn("WARNING: OrbitControls.js encountered an unknown camera type - zoom to cursor disabled."),this.zoomToCursor=!1;o!==null&&(this.screenSpacePanning?this.target.set(0,0,-1).transformDirection(this.object.matrix).multiplyScalar(o).add(this.object.position):(_o.origin.copy(this.object.position),_o.direction.set(0,0,-1).transformDirection(this.object.matrix),Math.abs(this.object.up.dot(_o.direction))<Bb?this.object.lookAt(this.target):(Ou.setFromNormalAndCoplanarPoint(this.object.up,this.target),_o.intersectPlane(Ou,this.target))))}else if(this.object.isOrthographicCamera){const o=this.object.zoom;this.object.zoom=Math.max(this.minZoom,Math.min(this.maxZoom,this.object.zoom/this._scale)),o!==this.object.zoom&&(this.object.updateProjectionMatrix(),r=!0)}return this._scale=1,this._performCursorZoom=!1,r||this._lastPosition.distanceToSquared(this.object.position)>ol||8*(1-this._lastQuaternion.dot(this.object.quaternion))>ol||this._lastTargetPosition.distanceToSquared(this.target)>ol?(this.dispatchEvent(Iu),this._lastPosition.copy(this.object.position),this._lastQuaternion.copy(this.object.quaternion),this._lastTargetPosition.copy(this.target),!0):!1}_getAutoRotationAngle(t){return t!==null?je/60*this.autoRotateSpeed*t:je/60/60*this.autoRotateSpeed}_getZoomScale(t){const e=Math.abs(t*.01);return Math.pow(.95,this.zoomSpeed*e)}_rotateLeft(t){this._sphericalDelta.theta-=t}_rotateUp(t){this._sphericalDelta.phi-=t}_panLeft(t,e){Re.setFromMatrixColumn(e,0),Re.multiplyScalar(-t),this._panOffset.add(Re)}_panUp(t,e){this.screenSpacePanning===!0?Re.setFromMatrixColumn(e,1):(Re.setFromMatrixColumn(e,0),Re.crossVectors(this.object.up,Re)),Re.multiplyScalar(t),this._panOffset.add(Re)}_pan(t,e){const i=this.domElement;if(this.object.isPerspectiveCamera){const s=this.object.position;Re.copy(s).sub(this.target);let r=Re.length();r*=Math.tan(this.object.fov/2*Math.PI/180),this._panLeft(2*t*r/i.clientHeight,this.object.matrix),this._panUp(2*e*r/i.clientHeight,this.object.matrix)}else this.object.isOrthographicCamera?(this._panLeft(t*(this.object.right-this.object.left)/this.object.zoom/i.clientWidth,this.object.matrix),this._panUp(e*(this.object.top-this.object.bottom)/this.object.zoom/i.clientHeight,this.object.matrix)):(console.warn("WARNING: OrbitControls.js encountered an unknown camera type - pan disabled."),this.enablePan=!1)}_dollyOut(t){this.object.isPerspectiveCamera||this.object.isOrthographicCamera?this._scale/=t:(console.warn("WARNING: OrbitControls.js encountered an unknown camera type - dolly/zoom disabled."),this.enableZoom=!1)}_dollyIn(t){this.object.isPerspectiveCamera||this.object.isOrthographicCamera?this._scale*=t:(console.warn("WARNING: OrbitControls.js encountered an unknown camera type - dolly/zoom disabled."),this.enableZoom=!1)}_updateZoomParameters(t,e){if(!this.zoomToCursor)return;this._performCursorZoom=!0;const i=this.domElement.getBoundingClientRect(),s=t-i.left,r=e-i.top,o=i.width,a=i.height;this._mouse.x=s/o*2-1,this._mouse.y=-(r/a)*2+1,this._dollyDirection.set(this._mouse.x,this._mouse.y,1).unproject(this.object).sub(this.object.position).normalize()}_clampDistance(t){return Math.max(this.minDistance,Math.min(this.maxDistance,t))}_handleMouseDownRotate(t){this._rotateStart.set(t.clientX,t.clientY)}_handleMouseDownDolly(t){this._updateZoomParameters(t.clientX,t.clientX),this._dollyStart.set(t.clientX,t.clientY)}_handleMouseDownPan(t){this._panStart.set(t.clientX,t.clientY)}_handleMouseMoveRotate(t){this._rotateEnd.set(t.clientX,t.clientY),this._rotateDelta.subVectors(this._rotateEnd,this._rotateStart).multiplyScalar(this.rotateSpeed);const e=this.domElement;this._rotateLeft(je*this._rotateDelta.x/e.clientHeight),this._rotateUp(je*this._rotateDelta.y/e.clientHeight),this._rotateStart.copy(this._rotateEnd),this.update()}_handleMouseMoveDolly(t){this._dollyEnd.set(t.clientX,t.clientY),this._dollyDelta.subVectors(this._dollyEnd,this._dollyStart),this._dollyDelta.y>0?this._dollyOut(this._getZoomScale(this._dollyDelta.y)):this._dollyDelta.y<0&&this._dollyIn(this._getZoomScale(this._dollyDelta.y)),this._dollyStart.copy(this._dollyEnd),this.update()}_handleMouseMovePan(t){this._panEnd.set(t.clientX,t.clientY),this._panDelta.subVectors(this._panEnd,this._panStart).multiplyScalar(this.panSpeed),this._pan(this._panDelta.x,this._panDelta.y),this._panStart.copy(this._panEnd),this.update()}_handleMouseWheel(t){this._updateZoomParameters(t.clientX,t.clientY),t.deltaY<0?this._dollyIn(this._getZoomScale(t.deltaY)):t.deltaY>0&&this._dollyOut(this._getZoomScale(t.deltaY)),this.update()}_handleKeyDown(t){let e=!1;switch(t.code){case this.keys.UP:t.ctrlKey||t.metaKey||t.shiftKey?this._rotateUp(je*this.rotateSpeed/this.domElement.clientHeight):this._pan(0,this.keyPanSpeed),e=!0;break;case this.keys.BOTTOM:t.ctrlKey||t.metaKey||t.shiftKey?this._rotateUp(-je*this.rotateSpeed/this.domElement.clientHeight):this._pan(0,-this.keyPanSpeed),e=!0;break;case this.keys.LEFT:t.ctrlKey||t.metaKey||t.shiftKey?this._rotateLeft(je*this.rotateSpeed/this.domElement.clientHeight):this._pan(this.keyPanSpeed,0),e=!0;break;case this.keys.RIGHT:t.ctrlKey||t.metaKey||t.shiftKey?this._rotateLeft(-je*this.rotateSpeed/this.domElement.clientHeight):this._pan(-this.keyPanSpeed,0),e=!0;break}e&&(t.preventDefault(),this.update())}_handleTouchStartRotate(t){if(this._pointers.length===1)this._rotateStart.set(t.pageX,t.pageY);else{const e=this._getSecondPointerPosition(t),i=.5*(t.pageX+e.x),s=.5*(t.pageY+e.y);this._rotateStart.set(i,s)}}_handleTouchStartPan(t){if(this._pointers.length===1)this._panStart.set(t.pageX,t.pageY);else{const e=this._getSecondPointerPosition(t),i=.5*(t.pageX+e.x),s=.5*(t.pageY+e.y);this._panStart.set(i,s)}}_handleTouchStartDolly(t){const e=this._getSecondPointerPosition(t),i=t.pageX-e.x,s=t.pageY-e.y,r=Math.sqrt(i*i+s*s);this._dollyStart.set(0,r)}_handleTouchStartDollyPan(t){this.enableZoom&&this._handleTouchStartDolly(t),this.enablePan&&this._handleTouchStartPan(t)}_handleTouchStartDollyRotate(t){this.enableZoom&&this._handleTouchStartDolly(t),this.enableRotate&&this._handleTouchStartRotate(t)}_handleTouchMoveRotate(t){if(this._pointers.length==1)this._rotateEnd.set(t.pageX,t.pageY);else{const i=this._getSecondPointerPosition(t),s=.5*(t.pageX+i.x),r=.5*(t.pageY+i.y);this._rotateEnd.set(s,r)}this._rotateDelta.subVectors(this._rotateEnd,this._rotateStart).multiplyScalar(this.rotateSpeed);const e=this.domElement;this._rotateLeft(je*this._rotateDelta.x/e.clientHeight),this._rotateUp(je*this._rotateDelta.y/e.clientHeight),this._rotateStart.copy(this._rotateEnd)}_handleTouchMovePan(t){if(this._pointers.length===1)this._panEnd.set(t.pageX,t.pageY);else{const e=this._getSecondPointerPosition(t),i=.5*(t.pageX+e.x),s=.5*(t.pageY+e.y);this._panEnd.set(i,s)}this._panDelta.subVectors(this._panEnd,this._panStart).multiplyScalar(this.panSpeed),this._pan(this._panDelta.x,this._panDelta.y),this._panStart.copy(this._panEnd)}_handleTouchMoveDolly(t){const e=this._getSecondPointerPosition(t),i=t.pageX-e.x,s=t.pageY-e.y,r=Math.sqrt(i*i+s*s);this._dollyEnd.set(0,r),this._dollyDelta.set(0,Math.pow(this._dollyEnd.y/this._dollyStart.y,this.zoomSpeed)),this._dollyOut(this._dollyDelta.y),this._dollyStart.copy(this._dollyEnd);const o=(t.pageX+e.x)*.5,a=(t.pageY+e.y)*.5;this._updateZoomParameters(o,a)}_handleTouchMoveDollyPan(t){this.enableZoom&&this._handleTouchMoveDolly(t),this.enablePan&&this._handleTouchMovePan(t)}_handleTouchMoveDollyRotate(t){this.enableZoom&&this._handleTouchMoveDolly(t),this.enableRotate&&this._handleTouchMoveRotate(t)}_addPointer(t){this._pointers.push(t.pointerId)}_removePointer(t){delete this._pointerPositions[t.pointerId];for(let e=0;e<this._pointers.length;e++)if(this._pointers[e]==t.pointerId){this._pointers.splice(e,1);return}}_isTrackingPointer(t){for(let e=0;e<this._pointers.length;e++)if(this._pointers[e]==t.pointerId)return!0;return!1}_trackPointer(t){let e=this._pointerPositions[t.pointerId];e===void 0&&(e=new Tt,this._pointerPositions[t.pointerId]=e),e.set(t.pageX,t.pageY)}_getSecondPointerPosition(t){const e=t.pointerId===this._pointers[0]?this._pointers[1]:this._pointers[0];return this._pointerPositions[e]}_customWheelEvent(t){const e=t.deltaMode,i={clientX:t.clientX,clientY:t.clientY,deltaY:t.deltaY};switch(e){case 1:i.deltaY*=16;break;case 2:i.deltaY*=100;break}return t.ctrlKey&&!this._controlActive&&(i.deltaY*=10),i}}function Hb(n){this.enabled!==!1&&(this._pointers.length===0&&(this.domElement.setPointerCapture(n.pointerId),this.domElement.addEventListener("pointermove",this._onPointerMove),this.domElement.addEventListener("pointerup",this._onPointerUp)),!this._isTrackingPointer(n)&&(this._addPointer(n),n.pointerType==="touch"?this._onTouchStart(n):this._onMouseDown(n)))}function Vb(n){this.enabled!==!1&&(n.pointerType==="touch"?this._onTouchMove(n):this._onMouseMove(n))}function Gb(n){switch(this._removePointer(n),this._pointers.length){case 0:this.domElement.releasePointerCapture(n.pointerId),this.domElement.removeEventListener("pointermove",this._onPointerMove),this.domElement.removeEventListener("pointerup",this._onPointerUp),this.dispatchEvent(Qf),this.state=re.NONE;break;case 1:const t=this._pointers[0],e=this._pointerPositions[t];this._onTouchStart({pointerId:t,pageX:e.x,pageY:e.y});break}}function Wb(n){let t;switch(n.button){case 0:t=this.mouseButtons.LEFT;break;case 1:t=this.mouseButtons.MIDDLE;break;case 2:t=this.mouseButtons.RIGHT;break;default:t=-1}switch(t){case Ts.DOLLY:if(this.enableZoom===!1)return;this._handleMouseDownDolly(n),this.state=re.DOLLY;break;case Ts.ROTATE:if(n.ctrlKey||n.metaKey||n.shiftKey){if(this.enablePan===!1)return;this._handleMouseDownPan(n),this.state=re.PAN}else{if(this.enableRotate===!1)return;this._handleMouseDownRotate(n),this.state=re.ROTATE}break;case Ts.PAN:if(n.ctrlKey||n.metaKey||n.shiftKey){if(this.enableRotate===!1)return;this._handleMouseDownRotate(n),this.state=re.ROTATE}else{if(this.enablePan===!1)return;this._handleMouseDownPan(n),this.state=re.PAN}break;default:this.state=re.NONE}this.state!==re.NONE&&this.dispatchEvent(Gc)}function Xb(n){switch(this.state){case re.ROTATE:if(this.enableRotate===!1)return;this._handleMouseMoveRotate(n);break;case re.DOLLY:if(this.enableZoom===!1)return;this._handleMouseMoveDolly(n);break;case re.PAN:if(this.enablePan===!1)return;this._handleMouseMovePan(n);break}}function jb(n){this.enabled===!1||this.enableZoom===!1||this.state!==re.NONE||(n.preventDefault(),this.dispatchEvent(Gc),this._handleMouseWheel(this._customWheelEvent(n)),this.dispatchEvent(Qf))}function $b(n){this.enabled===!1||this.enablePan===!1||this._handleKeyDown(n)}function Yb(n){switch(this._trackPointer(n),this._pointers.length){case 1:switch(this.touches.ONE){case ys.ROTATE:if(this.enableRotate===!1)return;this._handleTouchStartRotate(n),this.state=re.TOUCH_ROTATE;break;case ys.PAN:if(this.enablePan===!1)return;this._handleTouchStartPan(n),this.state=re.TOUCH_PAN;break;default:this.state=re.NONE}break;case 2:switch(this.touches.TWO){case ys.DOLLY_PAN:if(this.enableZoom===!1&&this.enablePan===!1)return;this._handleTouchStartDollyPan(n),this.state=re.TOUCH_DOLLY_PAN;break;case ys.DOLLY_ROTATE:if(this.enableZoom===!1&&this.enableRotate===!1)return;this._handleTouchStartDollyRotate(n),this.state=re.TOUCH_DOLLY_ROTATE;break;default:this.state=re.NONE}break;default:this.state=re.NONE}this.state!==re.NONE&&this.dispatchEvent(Gc)}function qb(n){switch(this._trackPointer(n),this.state){case re.TOUCH_ROTATE:if(this.enableRotate===!1)return;this._handleTouchMoveRotate(n),this.update();break;case re.TOUCH_PAN:if(this.enablePan===!1)return;this._handleTouchMovePan(n),this.update();break;case re.TOUCH_DOLLY_PAN:if(this.enableZoom===!1&&this.enablePan===!1)return;this._handleTouchMoveDollyPan(n),this.update();break;case re.TOUCH_DOLLY_ROTATE:if(this.enableZoom===!1&&this.enableRotate===!1)return;this._handleTouchMoveDollyRotate(n),this.update();break;default:this.state=re.NONE}}function Kb(n){this.enabled!==!1&&n.preventDefault()}function Zb(n){n.key==="Control"&&(this._controlActive=!0,this.domElement.getRootNode().addEventListener("keyup",this._interceptControlUp,{passive:!0,capture:!0}))}function Jb(n){n.key==="Control"&&(this._controlActive=!1,this.domElement.getRootNode().removeEventListener("keyup",this._interceptControlUp,{passive:!0,capture:!0}))}const Qb=new L(0,1,0);function xo(n,t,e){const i=new L(...t),s=new L(...e),r=s.clone().sub(i),o=r.length();n.visible=o>1e-6,n.visible&&(n.position.copy(i).add(s).multiplyScalar(.5),n.quaternion.setFromUnitVectors(Qb,r.normalize()),n.scale.set(1,o,1))}function Uu(n){return new oe().set(n[0][0],n[0][1],n[0][2],n[0][3],n[1][0],n[1][1],n[1][2],n[1][3],n[2][0],n[2][1],n[2][2],n[2][3],n[3][0],n[3][1],n[3][2],n[3][3])}function al(n,t="#ffffff",e=28){const i=document.createElement("canvas");i.width=128,i.height=64;const s=i.getContext("2d");s.font="bold 40px sans-serif",s.textAlign="center",s.textBaseline="middle",s.lineWidth=6,s.strokeStyle="rgba(0,0,0,0.75)",s.strokeText(n,64,32),s.fillStyle=t,s.fillText(n,64,32);const r=new Cb(new Yf({map:new Pb(i),depthTest:!1}));return r.scale.set(e*2,e,1),r.renderOrder=10,r}class ty{constructor(t){this.container=t,this.renderer=new Tb({antialias:!0}),this.renderer.setPixelRatio(Math.min(window.devicePixelRatio,2)),t.appendChild(this.renderer.domElement),this.scene=new wb,this.scene.background=new Ht("#1b1f27"),this.camera=new ln(45,1,1,5e3),this.camera.position.set(380,320,420),this.controls=new zb(this.camera,this.renderer.domElement),this.controls.target.set(60,80,0),this.controls.enableDamping=!0,this.controls.update(),this.scene.add(new Lb("#ffffff","#445566",1.2));const e=new Ub("#ffffff",1.6);e.position.set(300,600,200),this.scene.add(e);const i=new Fb(600,30,"#5a6372","#323844");this.scene.add(i);const s=new xe(new Ur(600,600),new Ms({visible:!1}));s.rotation.x=-Math.PI/2,this.ground=s,this.scene.add(s),this.robot_root=new Xn,this.robot_root.rotation.x=-Math.PI/2,this.scene.add(this.robot_root);const r=new Lu(80);this.robot_root.add(r);for(const[o,a,l]of[["X",[92,0,0],"#ff6b6b"],["Y",[0,92,0],"#7bd88f"],["Z",[0,0,92],"#74a7ff"]]){const c=al(o,l,16);c.position.set(...a),this.robot_root.add(c)}this.build_robot(),this.build_helpers(),this.resize_observer=new ResizeObserver(()=>this.resize()),this.resize_observer.observe(t),this.resize(),this.on_ground_pick=null,this.raycaster=new Nb,this.renderer.domElement.addEventListener("dblclick",o=>this.handle_dblclick(o))}build_robot(){const t=new xs({color:"#3d4554",metalness:.3,roughness:.6}),e=new xs({color:"#f0a030",metalness:.2,roughness:.5}),i=new xs({color:"#f5c04a",metalness:.2,roughness:.5}),s=new xs({color:"#d0d6e0",metalness:.5,roughness:.3}),r=new xs({color:"#9aa4b5",metalness:.4,roughness:.4});this.materials={mat_link1:e,mat_link2:i},this.base_plate=new xe(new Es(50,55,1,40),t),xo(this.base_plate,[0,0,0],[0,0,10]),this.robot_root.add(this.base_plate),this.turret=new Xn,this.robot_root.add(this.turret),this.turret_column=new xe(new Es(18,22,1,32),t),this.turret.add(this.turret_column),this.turret_marker=new xe(new An(30,14,8),e),this.turret.add(this.turret_marker),this.link1_mesh=new xe(new Es(9,9,1,24),e),this.link2_mesh=new xe(new Es(7.5,7.5,1,24),i),this.robot_root.add(this.link1_mesh,this.link2_mesh),this.joint_meshes=[0,1].map(()=>new xe(new pr(13,24,16),s)),this.robot_root.add(...this.joint_meshes),this.gripper_group=new Xn,this.gripper_group.matrixAutoUpdate=!1,this.robot_root.add(this.gripper_group),this.gripper_palm=new xe(new An(1,1,1),r),this.finger_left=new xe(new An(1,1,1),r),this.finger_right=new xe(new An(1,1,1),r),this.gripper_group.add(this.gripper_palm,this.finger_left,this.finger_right),this.tcp_mesh=new xe(new pr(4,16,12),new Ms({color:"#ff3b3b",depthTest:!1})),this.tcp_mesh.renderOrder=5,this.robot_root.add(this.tcp_mesh),this.frame_axes=[0,1,2,3].map(o=>{const a=new Lu(o===0?0:45);a.matrixAutoUpdate=!1,a.visible=!1;const l=al(`F${o}`,"#ffe08a",12);return l.position.set(8,8,8),a.add(l),this.robot_root.add(a),a})}build_helpers(){this.target_marker=new Xn;const t=new xe(new pr(7,20,14),new Ms({color:"#ff9f1c",transparent:!0,opacity:.7}));this.target_marker.add(t),this.target_drop=new hc(new Te().setFromPoints([new L,new L]),new Db({color:"#ff9f1c",dashSize:4,gapSize:3})),this.target_marker.visible=!1,this.target_drop.visible=!1,this.robot_root.add(this.target_marker,this.target_drop);const e=(i,s)=>{const r=new Xn,o=new xe(new Vc(16,32),new Ms({color:i,transparent:!0,opacity:.45,side:Tn}));o.position.z=.5,r.add(o);const a=al(s,i,18);return a.position.z=30,r.add(a),r.visible=!1,this.robot_root.add(r),r};this.pad_a=e("#4cd97b","A"),this.pad_b=e("#4ca3ff","B"),this.object_mesh=new xe(new An(20,20,20),new xs({color:"#e84a5f",roughness:.5})),this.object_mesh.visible=!1,this.robot_root.add(this.object_mesh),this.trail_line=new hc(new Te,new Nr({color:"#00e5ff"})),this.trail_line.visible=!1,this.robot_root.add(this.trail_line),this.workspace_points=new Rb(new Te,new Zf({size:2.2,vertexColors:!0,transparent:!0,opacity:.55})),this.workspace_points.visible=!1,this.robot_root.add(this.workspace_points)}update_pose(t,e,i){const s=t.links,[r,o,a,l]=e.joint_positions;this.turret.rotation.z=e.joint_deg[0]*Math.PI/180,xo(this.turret_column,[0,0,10],[0,0,Math.max(10.1,s.base_height)]),this.turret_marker.position.set(22,0,Math.max(14,s.base_height*.5)),xo(this.link1_mesh,o,a),xo(this.link2_mesh,a,e.gripper_base),this.joint_meshes[0].position.set(...o),this.joint_meshes[1].position.set(...a),this.tcp_mesh.position.set(...l),this.gripper_group.matrix.copy(Uu(e.frames[3])),this.gripper_group.matrixWorldNeedsUpdate=!0;const c=Math.max(1,s.gripper_length),h=t.gripper,u=h.open_angle===h.close_angle?0:(i-h.close_angle)/(h.open_angle-h.close_angle),d=Math.min(1,Math.max(0,u))*h.max_gap_mm,f=5;this.gripper_palm.scale.set(8,14,d+f*2+6),this.gripper_palm.position.set(-c+4,0,0);for(const[g,_]of[[this.finger_left,1],[this.finger_right,-1]])g.scale.set(c-8,12,f),g.position.set(-(c-8)/2,0,_*(d/2+f/2));e.frames.forEach((g,_)=>{this.frame_axes[_].matrix.copy(Uu(g)),this.frame_axes[_].matrixWorldNeedsUpdate=!0})}set_show_frames(t){this.frame_axes.forEach((e,i)=>e.visible=t&&i>0)}set_target(t,e=!0){if(!t){this.target_marker.visible=!1,this.target_drop.visible=!1;return}this.target_marker.visible=!0,this.target_drop.visible=!0,this.target_marker.position.set(...t),this.target_marker.children[0].material.color.set(e?"#ff9f1c":"#ff3b3b"),this.target_drop.geometry.setFromPoints([new L(...t),new L(t[0],t[1],0)]),this.target_drop.computeLineDistances()}set_pick_place(t,e){if(!t||!e){this.pad_a.visible=!1,this.pad_b.visible=!1;return}this.pad_a.visible=!0,this.pad_b.visible=!0,this.pad_a.position.set(t[0],t[1],0),this.pad_b.position.set(e[0],e[1],0)}set_object(t){this.object_mesh.visible=!!t,t&&this.object_mesh.position.set(...t)}set_trail(t){if(!t||t.length<2){this.trail_line.geometry.dispose(),this.trail_line.geometry=new Te,this.trail_line.visible=!1;return}this.trail_line.geometry.dispose(),this.trail_line.geometry=new Te().setFromPoints(t.map(e=>new L(...e))),this.trail_line.visible=!0}set_trail_visible(t){this.trail_line.geometry.attributes.position&&(this.trail_line.visible=t)}set_workspace(t){const e=new Float32Array(t.length*3),i=new Float32Array(t.length*3);let s=1/0,r=-1/0;for(const l of t)s=Math.min(s,l[2]),r=Math.max(r,l[2]);const o=new Ht;t.forEach((l,c)=>{e.set(l,c*3);const h=(l[2]-s)/Math.max(1,r-s);o.setHSL(.62-h*.5,.8,.55),i.set([o.r,o.g,o.b],c*3)});const a=new Te;a.setAttribute("position",new en(e,3)),a.setAttribute("color",new en(i,3)),this.workspace_points.geometry.dispose(),this.workspace_points.geometry=a}set_workspace_visible(t){this.workspace_points.visible=t}handle_dblclick(t){if(!this.on_ground_pick)return;const e=this.renderer.domElement.getBoundingClientRect(),i=new Tt((t.clientX-e.left)/e.width*2-1,-((t.clientY-e.top)/e.height)*2+1);this.raycaster.setFromCamera(i,this.camera);const s=this.raycaster.intersectObject(this.ground)[0];s&&this.on_ground_pick([s.point.x,-s.point.z])}resize(){const t=this.container.clientWidth,e=this.container.clientHeight;t===0||e===0||(this.renderer.setSize(t,e),this.camera.aspect=t/e,this.camera.updateProjectionMatrix())}render(){this.controls.update(),this.renderer.render(this.scene,this.camera)}}/*!
 * @kurkle/color v0.3.4
 * https://github.com/kurkle/color#readme
 * (c) 2024 Jukka Kurkela
 * Released under the MIT License
 */function Fr(n){return n+.5|0}const ci=(n,t,e)=>Math.max(Math.min(n,e),t);function lr(n){return ci(Fr(n*2.55),0,255)}function gi(n){return ci(Fr(n*255),0,255)}function Gn(n){return ci(Fr(n/2.55)/100,0,1)}function Nu(n){return ci(Fr(n*100),0,100)}const on={0:0,1:1,2:2,3:3,4:4,5:5,6:6,7:7,8:8,9:9,A:10,B:11,C:12,D:13,E:14,F:15,a:10,b:11,c:12,d:13,e:14,f:15},fc=[..."0123456789ABCDEF"],ey=n=>fc[n&15],ny=n=>fc[(n&240)>>4]+fc[n&15],vo=n=>(n&240)>>4===(n&15),iy=n=>vo(n.r)&&vo(n.g)&&vo(n.b)&&vo(n.a);function sy(n){var t=n.length,e;return n[0]==="#"&&(t===4||t===5?e={r:255&on[n[1]]*17,g:255&on[n[2]]*17,b:255&on[n[3]]*17,a:t===5?on[n[4]]*17:255}:(t===7||t===9)&&(e={r:on[n[1]]<<4|on[n[2]],g:on[n[3]]<<4|on[n[4]],b:on[n[5]]<<4|on[n[6]],a:t===9?on[n[7]]<<4|on[n[8]]:255})),e}const ry=(n,t)=>n<255?t(n):"";function oy(n){var t=iy(n)?ey:ny;return n?"#"+t(n.r)+t(n.g)+t(n.b)+ry(n.a,t):void 0}const ay=/^(hsla?|hwb|hsv)\(\s*([-+.e\d]+)(?:deg)?[\s,]+([-+.e\d]+)%[\s,]+([-+.e\d]+)%(?:[\s,]+([-+.e\d]+)(%)?)?\s*\)$/;function tp(n,t,e){const i=t*Math.min(e,1-e),s=(r,o=(r+n/30)%12)=>e-i*Math.max(Math.min(o-3,9-o,1),-1);return[s(0),s(8),s(4)]}function ly(n,t,e){const i=(s,r=(s+n/60)%6)=>e-e*t*Math.max(Math.min(r,4-r,1),0);return[i(5),i(3),i(1)]}function cy(n,t,e){const i=tp(n,1,.5);let s;for(t+e>1&&(s=1/(t+e),t*=s,e*=s),s=0;s<3;s++)i[s]*=1-t-e,i[s]+=t;return i}function hy(n,t,e,i,s){return n===s?(t-e)/i+(t<e?6:0):t===s?(e-n)/i+2:(n-t)/i+4}function Wc(n){const e=n.r/255,i=n.g/255,s=n.b/255,r=Math.max(e,i,s),o=Math.min(e,i,s),a=(r+o)/2;let l,c,h;return r!==o&&(h=r-o,c=a>.5?h/(2-r-o):h/(r+o),l=hy(e,i,s,h,r),l=l*60+.5),[l|0,c||0,a]}function Xc(n,t,e,i){return(Array.isArray(t)?n(t[0],t[1],t[2]):n(t,e,i)).map(gi)}function jc(n,t,e){return Xc(tp,n,t,e)}function uy(n,t,e){return Xc(cy,n,t,e)}function dy(n,t,e){return Xc(ly,n,t,e)}function ep(n){return(n%360+360)%360}function fy(n){const t=ay.exec(n);let e=255,i;if(!t)return;t[5]!==i&&(e=t[6]?lr(+t[5]):gi(+t[5]));const s=ep(+t[2]),r=+t[3]/100,o=+t[4]/100;return t[1]==="hwb"?i=uy(s,r,o):t[1]==="hsv"?i=dy(s,r,o):i=jc(s,r,o),{r:i[0],g:i[1],b:i[2],a:e}}function py(n,t){var e=Wc(n);e[0]=ep(e[0]+t),e=jc(e),n.r=e[0],n.g=e[1],n.b=e[2]}function my(n){if(!n)return;const t=Wc(n),e=t[0],i=Nu(t[1]),s=Nu(t[2]);return n.a<255?`hsla(${e}, ${i}%, ${s}%, ${Gn(n.a)})`:`hsl(${e}, ${i}%, ${s}%)`}const Fu={x:"dark",Z:"light",Y:"re",X:"blu",W:"gr",V:"medium",U:"slate",A:"ee",T:"ol",S:"or",B:"ra",C:"lateg",D:"ights",R:"in",Q:"turquois",E:"hi",P:"ro",O:"al",N:"le",M:"de",L:"yello",F:"en",K:"ch",G:"arks",H:"ea",I:"ightg",J:"wh"},ku={OiceXe:"f0f8ff",antiquewEte:"faebd7",aqua:"ffff",aquamarRe:"7fffd4",azuY:"f0ffff",beige:"f5f5dc",bisque:"ffe4c4",black:"0",blanKedOmond:"ffebcd",Xe:"ff",XeviTet:"8a2be2",bPwn:"a52a2a",burlywood:"deb887",caMtXe:"5f9ea0",KartYuse:"7fff00",KocTate:"d2691e",cSO:"ff7f50",cSnflowerXe:"6495ed",cSnsilk:"fff8dc",crimson:"dc143c",cyan:"ffff",xXe:"8b",xcyan:"8b8b",xgTMnPd:"b8860b",xWay:"a9a9a9",xgYF:"6400",xgYy:"a9a9a9",xkhaki:"bdb76b",xmagFta:"8b008b",xTivegYF:"556b2f",xSange:"ff8c00",xScEd:"9932cc",xYd:"8b0000",xsOmon:"e9967a",xsHgYF:"8fbc8f",xUXe:"483d8b",xUWay:"2f4f4f",xUgYy:"2f4f4f",xQe:"ced1",xviTet:"9400d3",dAppRk:"ff1493",dApskyXe:"bfff",dimWay:"696969",dimgYy:"696969",dodgerXe:"1e90ff",fiYbrick:"b22222",flSOwEte:"fffaf0",foYstWAn:"228b22",fuKsia:"ff00ff",gaRsbSo:"dcdcdc",ghostwEte:"f8f8ff",gTd:"ffd700",gTMnPd:"daa520",Way:"808080",gYF:"8000",gYFLw:"adff2f",gYy:"808080",honeyMw:"f0fff0",hotpRk:"ff69b4",RdianYd:"cd5c5c",Rdigo:"4b0082",ivSy:"fffff0",khaki:"f0e68c",lavFMr:"e6e6fa",lavFMrXsh:"fff0f5",lawngYF:"7cfc00",NmoncEffon:"fffacd",ZXe:"add8e6",ZcSO:"f08080",Zcyan:"e0ffff",ZgTMnPdLw:"fafad2",ZWay:"d3d3d3",ZgYF:"90ee90",ZgYy:"d3d3d3",ZpRk:"ffb6c1",ZsOmon:"ffa07a",ZsHgYF:"20b2aa",ZskyXe:"87cefa",ZUWay:"778899",ZUgYy:"778899",ZstAlXe:"b0c4de",ZLw:"ffffe0",lime:"ff00",limegYF:"32cd32",lRF:"faf0e6",magFta:"ff00ff",maPon:"800000",VaquamarRe:"66cdaa",VXe:"cd",VScEd:"ba55d3",VpurpN:"9370db",VsHgYF:"3cb371",VUXe:"7b68ee",VsprRggYF:"fa9a",VQe:"48d1cc",VviTetYd:"c71585",midnightXe:"191970",mRtcYam:"f5fffa",mistyPse:"ffe4e1",moccasR:"ffe4b5",navajowEte:"ffdead",navy:"80",Tdlace:"fdf5e6",Tive:"808000",TivedBb:"6b8e23",Sange:"ffa500",SangeYd:"ff4500",ScEd:"da70d6",pOegTMnPd:"eee8aa",pOegYF:"98fb98",pOeQe:"afeeee",pOeviTetYd:"db7093",papayawEp:"ffefd5",pHKpuff:"ffdab9",peru:"cd853f",pRk:"ffc0cb",plum:"dda0dd",powMrXe:"b0e0e6",purpN:"800080",YbeccapurpN:"663399",Yd:"ff0000",Psybrown:"bc8f8f",PyOXe:"4169e1",saddNbPwn:"8b4513",sOmon:"fa8072",sandybPwn:"f4a460",sHgYF:"2e8b57",sHshell:"fff5ee",siFna:"a0522d",silver:"c0c0c0",skyXe:"87ceeb",UXe:"6a5acd",UWay:"708090",UgYy:"708090",snow:"fffafa",sprRggYF:"ff7f",stAlXe:"4682b4",tan:"d2b48c",teO:"8080",tEstN:"d8bfd8",tomato:"ff6347",Qe:"40e0d0",viTet:"ee82ee",JHt:"f5deb3",wEte:"ffffff",wEtesmoke:"f5f5f5",Lw:"ffff00",LwgYF:"9acd32"};function gy(){const n={},t=Object.keys(ku),e=Object.keys(Fu);let i,s,r,o,a;for(i=0;i<t.length;i++){for(o=a=t[i],s=0;s<e.length;s++)r=e[s],a=a.replace(r,Fu[r]);r=parseInt(ku[o],16),n[a]=[r>>16&255,r>>8&255,r&255]}return n}let bo;function _y(n){bo||(bo=gy(),bo.transparent=[0,0,0,0]);const t=bo[n.toLowerCase()];return t&&{r:t[0],g:t[1],b:t[2],a:t.length===4?t[3]:255}}const xy=/^rgba?\(\s*([-+.\d]+)(%)?[\s,]+([-+.e\d]+)(%)?[\s,]+([-+.e\d]+)(%)?(?:[\s,/]+([-+.e\d]+)(%)?)?\s*\)$/;function vy(n){const t=xy.exec(n);let e=255,i,s,r;if(t){if(t[7]!==i){const o=+t[7];e=t[8]?lr(o):ci(o*255,0,255)}return i=+t[1],s=+t[3],r=+t[5],i=255&(t[2]?lr(i):ci(i,0,255)),s=255&(t[4]?lr(s):ci(s,0,255)),r=255&(t[6]?lr(r):ci(r,0,255)),{r:i,g:s,b:r,a:e}}}function by(n){return n&&(n.a<255?`rgba(${n.r}, ${n.g}, ${n.b}, ${Gn(n.a)})`:`rgb(${n.r}, ${n.g}, ${n.b})`)}const ll=n=>n<=.0031308?n*12.92:Math.pow(n,1/2.4)*1.055-.055,vs=n=>n<=.04045?n/12.92:Math.pow((n+.055)/1.055,2.4);function yy(n,t,e){const i=vs(Gn(n.r)),s=vs(Gn(n.g)),r=vs(Gn(n.b));return{r:gi(ll(i+e*(vs(Gn(t.r))-i))),g:gi(ll(s+e*(vs(Gn(t.g))-s))),b:gi(ll(r+e*(vs(Gn(t.b))-r))),a:n.a+e*(t.a-n.a)}}function yo(n,t,e){if(n){let i=Wc(n);i[t]=Math.max(0,Math.min(i[t]+i[t]*e,t===0?360:1)),i=jc(i),n.r=i[0],n.g=i[1],n.b=i[2]}}function np(n,t){return n&&Object.assign(t||{},n)}function Bu(n){var t={r:0,g:0,b:0,a:255};return Array.isArray(n)?n.length>=3&&(t={r:n[0],g:n[1],b:n[2],a:255},n.length>3&&(t.a=gi(n[3]))):(t=np(n,{r:0,g:0,b:0,a:1}),t.a=gi(t.a)),t}function My(n){return n.charAt(0)==="r"?vy(n):fy(n)}class Mr{constructor(t){if(t instanceof Mr)return t;const e=typeof t;let i;e==="object"?i=Bu(t):e==="string"&&(i=sy(t)||_y(t)||My(t)),this._rgb=i,this._valid=!!i}get valid(){return this._valid}get rgb(){var t=np(this._rgb);return t&&(t.a=Gn(t.a)),t}set rgb(t){this._rgb=Bu(t)}rgbString(){return this._valid?by(this._rgb):void 0}hexString(){return this._valid?oy(this._rgb):void 0}hslString(){return this._valid?my(this._rgb):void 0}mix(t,e){if(t){const i=this.rgb,s=t.rgb;let r;const o=e===r?.5:e,a=2*o-1,l=i.a-s.a,c=((a*l===-1?a:(a+l)/(1+a*l))+1)/2;r=1-c,i.r=255&c*i.r+r*s.r+.5,i.g=255&c*i.g+r*s.g+.5,i.b=255&c*i.b+r*s.b+.5,i.a=o*i.a+(1-o)*s.a,this.rgb=i}return this}interpolate(t,e){return t&&(this._rgb=yy(this._rgb,t._rgb,e)),this}clone(){return new Mr(this.rgb)}alpha(t){return this._rgb.a=gi(t),this}clearer(t){const e=this._rgb;return e.a*=1-t,this}greyscale(){const t=this._rgb,e=Fr(t.r*.3+t.g*.59+t.b*.11);return t.r=t.g=t.b=e,this}opaquer(t){const e=this._rgb;return e.a*=1+t,this}negate(){const t=this._rgb;return t.r=255-t.r,t.g=255-t.g,t.b=255-t.b,this}lighten(t){return yo(this._rgb,2,t),this}darken(t){return yo(this._rgb,2,-t),this}saturate(t){return yo(this._rgb,1,t),this}desaturate(t){return yo(this._rgb,1,-t),this}rotate(t){return py(this._rgb,t),this}}/*!
 * Chart.js v4.4.7
 * https://www.chartjs.org
 * (c) 2024 Chart.js Contributors
 * Released under the MIT License
 */function kn(){}const Sy=(()=>{let n=0;return()=>n++})();function Zt(n){return n==null}function me(n){if(Array.isArray&&Array.isArray(n))return!0;const t=Object.prototype.toString.call(n);return t.slice(0,7)==="[object"&&t.slice(-6)==="Array]"}function Gt(n){return n!==null&&Object.prototype.toString.call(n)==="[object Object]"}function ye(n){return(typeof n=="number"||n instanceof Number)&&isFinite(+n)}function Qe(n,t){return ye(n)?n:t}function Ot(n,t){return typeof n>"u"?t:n}const Ey=(n,t)=>typeof n=="string"&&n.endsWith("%")?parseFloat(n)/100:+n/t,ip=(n,t)=>typeof n=="string"&&n.endsWith("%")?parseFloat(n)/100*t:+n;function ce(n,t,e){if(n&&typeof n.call=="function")return n.apply(e,t)}function ne(n,t,e,i){let s,r,o;if(me(n))for(r=n.length,s=0;s<r;s++)t.call(e,n[s],s);else if(Gt(n))for(o=Object.keys(n),r=o.length,s=0;s<r;s++)t.call(e,n[o[s]],o[s])}function na(n,t){let e,i,s,r;if(!n||!t||n.length!==t.length)return!1;for(e=0,i=n.length;e<i;++e)if(s=n[e],r=t[e],s.datasetIndex!==r.datasetIndex||s.index!==r.index)return!1;return!0}function ia(n){if(me(n))return n.map(ia);if(Gt(n)){const t=Object.create(null),e=Object.keys(n),i=e.length;let s=0;for(;s<i;++s)t[e[s]]=ia(n[e[s]]);return t}return n}function sp(n){return["__proto__","prototype","constructor"].indexOf(n)===-1}function Ty(n,t,e,i){if(!sp(n))return;const s=t[n],r=e[n];Gt(s)&&Gt(r)?Sr(s,r,i):t[n]=ia(r)}function Sr(n,t,e){const i=me(t)?t:[t],s=i.length;if(!Gt(n))return n;e=e||{};const r=e.merger||Ty;let o;for(let a=0;a<s;++a){if(o=i[a],!Gt(o))continue;const l=Object.keys(o);for(let c=0,h=l.length;c<h;++c)r(l[c],n,o,e)}return n}function mr(n,t){return Sr(n,t,{merger:wy})}function wy(n,t,e){if(!sp(n))return;const i=t[n],s=e[n];Gt(i)&&Gt(s)?mr(i,s):Object.prototype.hasOwnProperty.call(t,n)||(t[n]=ia(s))}const zu={"":n=>n,x:n=>n.x,y:n=>n.y};function Ay(n){const t=n.split("."),e=[];let i="";for(const s of t)i+=s,i.endsWith("\\")?i=i.slice(0,-1)+".":(e.push(i),i="");return e}function Cy(n){const t=Ay(n);return e=>{for(const i of t){if(i==="")break;e=e&&e[i]}return e}}function vi(n,t){return(zu[t]||(zu[t]=Cy(t)))(n)}function $c(n){return n.charAt(0).toUpperCase()+n.slice(1)}const Er=n=>typeof n<"u",bi=n=>typeof n=="function",Hu=(n,t)=>{if(n.size!==t.size)return!1;for(const e of n)if(!t.has(e))return!1;return!0};function Ry(n){return n.type==="mouseup"||n.type==="click"||n.type==="contextmenu"}const pe=Math.PI,fe=2*pe,Py=fe+pe,sa=Number.POSITIVE_INFINITY,Dy=pe/180,we=pe/2,Pi=pe/4,Vu=pe*2/3,hi=Math.log10,Rn=Math.sign;function gr(n,t,e){return Math.abs(n-t)<e}function Gu(n){const t=Math.round(n);n=gr(n,t,n/1e3)?t:n;const e=Math.pow(10,Math.floor(hi(n))),i=n/e;return(i<=1?1:i<=2?2:i<=5?5:10)*e}function Ly(n){const t=[],e=Math.sqrt(n);let i;for(i=1;i<e;i++)n%i===0&&(t.push(i),t.push(n/i));return e===(e|0)&&t.push(e),t.sort((s,r)=>s-r).pop(),t}function Fs(n){return!isNaN(parseFloat(n))&&isFinite(n)}function Iy(n,t){const e=Math.round(n);return e-t<=n&&e+t>=n}function rp(n,t,e){let i,s,r;for(i=0,s=n.length;i<s;i++)r=n[i][e],isNaN(r)||(t.min=Math.min(t.min,r),t.max=Math.max(t.max,r))}function gn(n){return n*(pe/180)}function Yc(n){return n*(180/pe)}function Wu(n){if(!ye(n))return;let t=1,e=0;for(;Math.round(n*t)/t!==n;)t*=10,e++;return e}function op(n,t){const e=t.x-n.x,i=t.y-n.y,s=Math.sqrt(e*e+i*i);let r=Math.atan2(i,e);return r<-.5*pe&&(r+=fe),{angle:r,distance:s}}function pc(n,t){return Math.sqrt(Math.pow(t.x-n.x,2)+Math.pow(t.y-n.y,2))}function Oy(n,t){return(n-t+Py)%fe-pe}function tn(n){return(n%fe+fe)%fe}function Tr(n,t,e,i){const s=tn(n),r=tn(t),o=tn(e),a=tn(r-s),l=tn(o-s),c=tn(s-r),h=tn(s-o);return s===r||s===o||i&&r===o||a>l&&c<h}function Oe(n,t,e){return Math.max(t,Math.min(e,n))}function Uy(n){return Oe(n,-32768,32767)}function Yn(n,t,e,i=1e-6){return n>=Math.min(t,e)-i&&n<=Math.max(t,e)+i}function qc(n,t,e){e=e||(o=>n[o]<t);let i=n.length-1,s=0,r;for(;i-s>1;)r=s+i>>1,e(r)?s=r:i=r;return{lo:s,hi:i}}const qn=(n,t,e,i)=>qc(n,e,i?s=>{const r=n[s][t];return r<e||r===e&&n[s+1][t]===e}:s=>n[s][t]<e),Ny=(n,t,e)=>qc(n,e,i=>n[i][t]>=e);function Fy(n,t,e){let i=0,s=n.length;for(;i<s&&n[i]<t;)i++;for(;s>i&&n[s-1]>e;)s--;return i>0||s<n.length?n.slice(i,s):n}const ap=["push","pop","shift","splice","unshift"];function ky(n,t){if(n._chartjs){n._chartjs.listeners.push(t);return}Object.defineProperty(n,"_chartjs",{configurable:!0,enumerable:!1,value:{listeners:[t]}}),ap.forEach(e=>{const i="_onData"+$c(e),s=n[e];Object.defineProperty(n,e,{configurable:!0,enumerable:!1,value(...r){const o=s.apply(this,r);return n._chartjs.listeners.forEach(a=>{typeof a[i]=="function"&&a[i](...r)}),o}})})}function Xu(n,t){const e=n._chartjs;if(!e)return;const i=e.listeners,s=i.indexOf(t);s!==-1&&i.splice(s,1),!(i.length>0)&&(ap.forEach(r=>{delete n[r]}),delete n._chartjs)}function lp(n){const t=new Set(n);return t.size===n.length?n:Array.from(t)}const cp=function(){return typeof window>"u"?function(n){return n()}:window.requestAnimationFrame}();function hp(n,t){let e=[],i=!1;return function(...s){e=s,i||(i=!0,cp.call(window,()=>{i=!1,n.apply(t,e)}))}}function By(n,t){let e;return function(...i){return t?(clearTimeout(e),e=setTimeout(n,t,i)):n.apply(this,i),t}}const Kc=n=>n==="start"?"left":n==="end"?"right":"center",ke=(n,t,e)=>n==="start"?t:n==="end"?e:(t+e)/2,zy=(n,t,e,i)=>n===(i?"left":"right")?e:n==="center"?(t+e)/2:t;function up(n,t,e){const i=t.length;let s=0,r=i;if(n._sorted){const{iScale:o,_parsed:a}=n,l=o.axis,{min:c,max:h,minDefined:u,maxDefined:d}=o.getUserBounds();u&&(s=Oe(Math.min(qn(a,l,c).lo,e?i:qn(t,l,o.getPixelForValue(c)).lo),0,i-1)),d?r=Oe(Math.max(qn(a,o.axis,h,!0).hi+1,e?0:qn(t,l,o.getPixelForValue(h),!0).hi+1),s,i)-s:r=i-s}return{start:s,count:r}}function dp(n){const{xScale:t,yScale:e,_scaleRanges:i}=n,s={xmin:t.min,xmax:t.max,ymin:e.min,ymax:e.max};if(!i)return n._scaleRanges=s,!0;const r=i.xmin!==t.min||i.xmax!==t.max||i.ymin!==e.min||i.ymax!==e.max;return Object.assign(i,s),r}const Mo=n=>n===0||n===1,ju=(n,t,e)=>-(Math.pow(2,10*(n-=1))*Math.sin((n-t)*fe/e)),$u=(n,t,e)=>Math.pow(2,-10*n)*Math.sin((n-t)*fe/e)+1,_r={linear:n=>n,easeInQuad:n=>n*n,easeOutQuad:n=>-n*(n-2),easeInOutQuad:n=>(n/=.5)<1?.5*n*n:-.5*(--n*(n-2)-1),easeInCubic:n=>n*n*n,easeOutCubic:n=>(n-=1)*n*n+1,easeInOutCubic:n=>(n/=.5)<1?.5*n*n*n:.5*((n-=2)*n*n+2),easeInQuart:n=>n*n*n*n,easeOutQuart:n=>-((n-=1)*n*n*n-1),easeInOutQuart:n=>(n/=.5)<1?.5*n*n*n*n:-.5*((n-=2)*n*n*n-2),easeInQuint:n=>n*n*n*n*n,easeOutQuint:n=>(n-=1)*n*n*n*n+1,easeInOutQuint:n=>(n/=.5)<1?.5*n*n*n*n*n:.5*((n-=2)*n*n*n*n+2),easeInSine:n=>-Math.cos(n*we)+1,easeOutSine:n=>Math.sin(n*we),easeInOutSine:n=>-.5*(Math.cos(pe*n)-1),easeInExpo:n=>n===0?0:Math.pow(2,10*(n-1)),easeOutExpo:n=>n===1?1:-Math.pow(2,-10*n)+1,easeInOutExpo:n=>Mo(n)?n:n<.5?.5*Math.pow(2,10*(n*2-1)):.5*(-Math.pow(2,-10*(n*2-1))+2),easeInCirc:n=>n>=1?n:-(Math.sqrt(1-n*n)-1),easeOutCirc:n=>Math.sqrt(1-(n-=1)*n),easeInOutCirc:n=>(n/=.5)<1?-.5*(Math.sqrt(1-n*n)-1):.5*(Math.sqrt(1-(n-=2)*n)+1),easeInElastic:n=>Mo(n)?n:ju(n,.075,.3),easeOutElastic:n=>Mo(n)?n:$u(n,.075,.3),easeInOutElastic(n){return Mo(n)?n:n<.5?.5*ju(n*2,.1125,.45):.5+.5*$u(n*2-1,.1125,.45)},easeInBack(n){return n*n*((1.70158+1)*n-1.70158)},easeOutBack(n){return(n-=1)*n*((1.70158+1)*n+1.70158)+1},easeInOutBack(n){let t=1.70158;return(n/=.5)<1?.5*(n*n*(((t*=1.525)+1)*n-t)):.5*((n-=2)*n*(((t*=1.525)+1)*n+t)+2)},easeInBounce:n=>1-_r.easeOutBounce(1-n),easeOutBounce(n){return n<1/2.75?7.5625*n*n:n<2/2.75?7.5625*(n-=1.5/2.75)*n+.75:n<2.5/2.75?7.5625*(n-=2.25/2.75)*n+.9375:7.5625*(n-=2.625/2.75)*n+.984375},easeInOutBounce:n=>n<.5?_r.easeInBounce(n*2)*.5:_r.easeOutBounce(n*2-1)*.5+.5};function Zc(n){if(n&&typeof n=="object"){const t=n.toString();return t==="[object CanvasPattern]"||t==="[object CanvasGradient]"}return!1}function Yu(n){return Zc(n)?n:new Mr(n)}function cl(n){return Zc(n)?n:new Mr(n).saturate(.5).darken(.1).hexString()}const Hy=["x","y","borderWidth","radius","tension"],Vy=["color","borderColor","backgroundColor"];function Gy(n){n.set("animation",{delay:void 0,duration:1e3,easing:"easeOutQuart",fn:void 0,from:void 0,loop:void 0,to:void 0,type:void 0}),n.describe("animation",{_fallback:!1,_indexable:!1,_scriptable:t=>t!=="onProgress"&&t!=="onComplete"&&t!=="fn"}),n.set("animations",{colors:{type:"color",properties:Vy},numbers:{type:"number",properties:Hy}}),n.describe("animations",{_fallback:"animation"}),n.set("transitions",{active:{animation:{duration:400}},resize:{animation:{duration:0}},show:{animations:{colors:{from:"transparent"},visible:{type:"boolean",duration:0}}},hide:{animations:{colors:{to:"transparent"},visible:{type:"boolean",easing:"linear",fn:t=>t|0}}}})}function Wy(n){n.set("layout",{autoPadding:!0,padding:{top:0,right:0,bottom:0,left:0}})}const qu=new Map;function Xy(n,t){t=t||{};const e=n+JSON.stringify(t);let i=qu.get(e);return i||(i=new Intl.NumberFormat(n,t),qu.set(e,i)),i}function kr(n,t,e){return Xy(t,e).format(n)}const fp={values(n){return me(n)?n:""+n},numeric(n,t,e){if(n===0)return"0";const i=this.chart.options.locale;let s,r=n;if(e.length>1){const c=Math.max(Math.abs(e[0].value),Math.abs(e[e.length-1].value));(c<1e-4||c>1e15)&&(s="scientific"),r=jy(n,e)}const o=hi(Math.abs(r)),a=isNaN(o)?1:Math.max(Math.min(-1*Math.floor(o),20),0),l={notation:s,minimumFractionDigits:a,maximumFractionDigits:a};return Object.assign(l,this.options.ticks.format),kr(n,i,l)},logarithmic(n,t,e){if(n===0)return"0";const i=e[t].significand||n/Math.pow(10,Math.floor(hi(n)));return[1,2,3,5,10,15].includes(i)||t>.8*e.length?fp.numeric.call(this,n,t,e):""}};function jy(n,t){let e=t.length>3?t[2].value-t[1].value:t[1].value-t[0].value;return Math.abs(e)>=1&&n!==Math.floor(n)&&(e=n-Math.floor(n)),e}var pa={formatters:fp};function $y(n){n.set("scale",{display:!0,offset:!1,reverse:!1,beginAtZero:!1,bounds:"ticks",clip:!0,grace:0,grid:{display:!0,lineWidth:1,drawOnChartArea:!0,drawTicks:!0,tickLength:8,tickWidth:(t,e)=>e.lineWidth,tickColor:(t,e)=>e.color,offset:!1},border:{display:!0,dash:[],dashOffset:0,width:1},title:{display:!1,text:"",padding:{top:4,bottom:4}},ticks:{minRotation:0,maxRotation:50,mirror:!1,textStrokeWidth:0,textStrokeColor:"",padding:3,display:!0,autoSkip:!0,autoSkipPadding:3,labelOffset:0,callback:pa.formatters.values,minor:{},major:{},align:"center",crossAlign:"near",showLabelBackdrop:!1,backdropColor:"rgba(255, 255, 255, 0.75)",backdropPadding:2}}),n.route("scale.ticks","color","","color"),n.route("scale.grid","color","","borderColor"),n.route("scale.border","color","","borderColor"),n.route("scale.title","color","","color"),n.describe("scale",{_fallback:!1,_scriptable:t=>!t.startsWith("before")&&!t.startsWith("after")&&t!=="callback"&&t!=="parser",_indexable:t=>t!=="borderDash"&&t!=="tickBorderDash"&&t!=="dash"}),n.describe("scales",{_fallback:"scale"}),n.describe("scale.ticks",{_scriptable:t=>t!=="backdropPadding"&&t!=="callback",_indexable:t=>t!=="backdropPadding"})}const Yi=Object.create(null),mc=Object.create(null);function xr(n,t){if(!t)return n;const e=t.split(".");for(let i=0,s=e.length;i<s;++i){const r=e[i];n=n[r]||(n[r]=Object.create(null))}return n}function hl(n,t,e){return typeof t=="string"?Sr(xr(n,t),e):Sr(xr(n,""),t)}class Yy{constructor(t,e){this.animation=void 0,this.backgroundColor="rgba(0,0,0,0.1)",this.borderColor="rgba(0,0,0,0.1)",this.color="#666",this.datasets={},this.devicePixelRatio=i=>i.chart.platform.getDevicePixelRatio(),this.elements={},this.events=["mousemove","mouseout","click","touchstart","touchmove"],this.font={family:"'Helvetica Neue', 'Helvetica', 'Arial', sans-serif",size:12,style:"normal",lineHeight:1.2,weight:null},this.hover={},this.hoverBackgroundColor=(i,s)=>cl(s.backgroundColor),this.hoverBorderColor=(i,s)=>cl(s.borderColor),this.hoverColor=(i,s)=>cl(s.color),this.indexAxis="x",this.interaction={mode:"nearest",intersect:!0,includeInvisible:!1},this.maintainAspectRatio=!0,this.onHover=null,this.onClick=null,this.parsing=!0,this.plugins={},this.responsive=!0,this.scale=void 0,this.scales={},this.showLine=!0,this.drawActiveElementsOnTop=!0,this.describe(t),this.apply(e)}set(t,e){return hl(this,t,e)}get(t){return xr(this,t)}describe(t,e){return hl(mc,t,e)}override(t,e){return hl(Yi,t,e)}route(t,e,i,s){const r=xr(this,t),o=xr(this,i),a="_"+e;Object.defineProperties(r,{[a]:{value:r[e],writable:!0},[e]:{enumerable:!0,get(){const l=this[a],c=o[s];return Gt(l)?Object.assign({},c,l):Ot(l,c)},set(l){this[a]=l}}})}apply(t){t.forEach(e=>e(this))}}var ge=new Yy({_scriptable:n=>!n.startsWith("on"),_indexable:n=>n!=="events",hover:{_fallback:"interaction"},interaction:{_scriptable:!1,_indexable:!1}},[Gy,Wy,$y]);function qy(n){return!n||Zt(n.size)||Zt(n.family)?null:(n.style?n.style+" ":"")+(n.weight?n.weight+" ":"")+n.size+"px "+n.family}function ra(n,t,e,i,s){let r=t[s];return r||(r=t[s]=n.measureText(s).width,e.push(s)),r>i&&(i=r),i}function Ky(n,t,e,i){i=i||{};let s=i.data=i.data||{},r=i.garbageCollect=i.garbageCollect||[];i.font!==t&&(s=i.data={},r=i.garbageCollect=[],i.font=t),n.save(),n.font=t;let o=0;const a=e.length;let l,c,h,u,d;for(l=0;l<a;l++)if(u=e[l],u!=null&&!me(u))o=ra(n,s,r,o,u);else if(me(u))for(c=0,h=u.length;c<h;c++)d=u[c],d!=null&&!me(d)&&(o=ra(n,s,r,o,d));n.restore();const f=r.length/2;if(f>e.length){for(l=0;l<f;l++)delete s[r[l]];r.splice(0,f)}return o}function Di(n,t,e){const i=n.currentDevicePixelRatio,s=e!==0?Math.max(e/2,.5):0;return Math.round((t-s)*i)/i+s}function Ku(n,t){!t&&!n||(t=t||n.getContext("2d"),t.save(),t.resetTransform(),t.clearRect(0,0,n.width,n.height),t.restore())}function gc(n,t,e,i){pp(n,t,e,i,null)}function pp(n,t,e,i,s){let r,o,a,l,c,h,u,d;const f=t.pointStyle,g=t.rotation,_=t.radius;let m=(g||0)*Dy;if(f&&typeof f=="object"&&(r=f.toString(),r==="[object HTMLImageElement]"||r==="[object HTMLCanvasElement]")){n.save(),n.translate(e,i),n.rotate(m),n.drawImage(f,-f.width/2,-f.height/2,f.width,f.height),n.restore();return}if(!(isNaN(_)||_<=0)){switch(n.beginPath(),f){default:s?n.ellipse(e,i,s/2,_,0,0,fe):n.arc(e,i,_,0,fe),n.closePath();break;case"triangle":h=s?s/2:_,n.moveTo(e+Math.sin(m)*h,i-Math.cos(m)*_),m+=Vu,n.lineTo(e+Math.sin(m)*h,i-Math.cos(m)*_),m+=Vu,n.lineTo(e+Math.sin(m)*h,i-Math.cos(m)*_),n.closePath();break;case"rectRounded":c=_*.516,l=_-c,o=Math.cos(m+Pi)*l,u=Math.cos(m+Pi)*(s?s/2-c:l),a=Math.sin(m+Pi)*l,d=Math.sin(m+Pi)*(s?s/2-c:l),n.arc(e-u,i-a,c,m-pe,m-we),n.arc(e+d,i-o,c,m-we,m),n.arc(e+u,i+a,c,m,m+we),n.arc(e-d,i+o,c,m+we,m+pe),n.closePath();break;case"rect":if(!g){l=Math.SQRT1_2*_,h=s?s/2:l,n.rect(e-h,i-l,2*h,2*l);break}m+=Pi;case"rectRot":u=Math.cos(m)*(s?s/2:_),o=Math.cos(m)*_,a=Math.sin(m)*_,d=Math.sin(m)*(s?s/2:_),n.moveTo(e-u,i-a),n.lineTo(e+d,i-o),n.lineTo(e+u,i+a),n.lineTo(e-d,i+o),n.closePath();break;case"crossRot":m+=Pi;case"cross":u=Math.cos(m)*(s?s/2:_),o=Math.cos(m)*_,a=Math.sin(m)*_,d=Math.sin(m)*(s?s/2:_),n.moveTo(e-u,i-a),n.lineTo(e+u,i+a),n.moveTo(e+d,i-o),n.lineTo(e-d,i+o);break;case"star":u=Math.cos(m)*(s?s/2:_),o=Math.cos(m)*_,a=Math.sin(m)*_,d=Math.sin(m)*(s?s/2:_),n.moveTo(e-u,i-a),n.lineTo(e+u,i+a),n.moveTo(e+d,i-o),n.lineTo(e-d,i+o),m+=Pi,u=Math.cos(m)*(s?s/2:_),o=Math.cos(m)*_,a=Math.sin(m)*_,d=Math.sin(m)*(s?s/2:_),n.moveTo(e-u,i-a),n.lineTo(e+u,i+a),n.moveTo(e+d,i-o),n.lineTo(e-d,i+o);break;case"line":o=s?s/2:Math.cos(m)*_,a=Math.sin(m)*_,n.moveTo(e-o,i-a),n.lineTo(e+o,i+a);break;case"dash":n.moveTo(e,i),n.lineTo(e+Math.cos(m)*(s?s/2:_),i+Math.sin(m)*_);break;case!1:n.closePath();break}n.fill(),t.borderWidth>0&&n.stroke()}}function Kn(n,t,e){return e=e||.5,!t||n&&n.x>t.left-e&&n.x<t.right+e&&n.y>t.top-e&&n.y<t.bottom+e}function ma(n,t){n.save(),n.beginPath(),n.rect(t.left,t.top,t.right-t.left,t.bottom-t.top),n.clip()}function ga(n){n.restore()}function Zy(n,t,e,i,s){if(!t)return n.lineTo(e.x,e.y);if(s==="middle"){const r=(t.x+e.x)/2;n.lineTo(r,t.y),n.lineTo(r,e.y)}else s==="after"!=!!i?n.lineTo(t.x,e.y):n.lineTo(e.x,t.y);n.lineTo(e.x,e.y)}function Jy(n,t,e,i){if(!t)return n.lineTo(e.x,e.y);n.bezierCurveTo(i?t.cp1x:t.cp2x,i?t.cp1y:t.cp2y,i?e.cp2x:e.cp1x,i?e.cp2y:e.cp1y,e.x,e.y)}function Qy(n,t){t.translation&&n.translate(t.translation[0],t.translation[1]),Zt(t.rotation)||n.rotate(t.rotation),t.color&&(n.fillStyle=t.color),t.textAlign&&(n.textAlign=t.textAlign),t.textBaseline&&(n.textBaseline=t.textBaseline)}function tM(n,t,e,i,s){if(s.strikethrough||s.underline){const r=n.measureText(i),o=t-r.actualBoundingBoxLeft,a=t+r.actualBoundingBoxRight,l=e-r.actualBoundingBoxAscent,c=e+r.actualBoundingBoxDescent,h=s.strikethrough?(l+c)/2:c;n.strokeStyle=n.fillStyle,n.beginPath(),n.lineWidth=s.decorationWidth||2,n.moveTo(o,h),n.lineTo(a,h),n.stroke()}}function eM(n,t){const e=n.fillStyle;n.fillStyle=t.color,n.fillRect(t.left,t.top,t.width,t.height),n.fillStyle=e}function qi(n,t,e,i,s,r={}){const o=me(t)?t:[t],a=r.strokeWidth>0&&r.strokeColor!=="";let l,c;for(n.save(),n.font=s.string,Qy(n,r),l=0;l<o.length;++l)c=o[l],r.backdrop&&eM(n,r.backdrop),a&&(r.strokeColor&&(n.strokeStyle=r.strokeColor),Zt(r.strokeWidth)||(n.lineWidth=r.strokeWidth),n.strokeText(c,e,i,r.maxWidth)),n.fillText(c,e,i,r.maxWidth),tM(n,e,i,c,r),i+=Number(s.lineHeight);n.restore()}function wr(n,t){const{x:e,y:i,w:s,h:r,radius:o}=t;n.arc(e+o.topLeft,i+o.topLeft,o.topLeft,1.5*pe,pe,!0),n.lineTo(e,i+r-o.bottomLeft),n.arc(e+o.bottomLeft,i+r-o.bottomLeft,o.bottomLeft,pe,we,!0),n.lineTo(e+s-o.bottomRight,i+r),n.arc(e+s-o.bottomRight,i+r-o.bottomRight,o.bottomRight,we,0,!0),n.lineTo(e+s,i+o.topRight),n.arc(e+s-o.topRight,i+o.topRight,o.topRight,0,-we,!0),n.lineTo(e+o.topLeft,i)}const nM=/^(normal|(\d+(?:\.\d+)?)(px|em|%)?)$/,iM=/^(normal|italic|initial|inherit|unset|(oblique( -?[0-9]?[0-9]deg)?))$/;function sM(n,t){const e=(""+n).match(nM);if(!e||e[1]==="normal")return t*1.2;switch(n=+e[2],e[3]){case"px":return n;case"%":n/=100;break}return t*n}const rM=n=>+n||0;function Jc(n,t){const e={},i=Gt(t),s=i?Object.keys(t):t,r=Gt(n)?i?o=>Ot(n[o],n[t[o]]):o=>n[o]:()=>n;for(const o of s)e[o]=rM(r(o));return e}function mp(n){return Jc(n,{top:"y",right:"x",bottom:"y",left:"x"})}function Gi(n){return Jc(n,["topLeft","topRight","bottomLeft","bottomRight"])}function ze(n){const t=mp(n);return t.width=t.left+t.right,t.height=t.top+t.bottom,t}function Pe(n,t){n=n||{},t=t||ge.font;let e=Ot(n.size,t.size);typeof e=="string"&&(e=parseInt(e,10));let i=Ot(n.style,t.style);i&&!(""+i).match(iM)&&(console.warn('Invalid font style specified: "'+i+'"'),i=void 0);const s={family:Ot(n.family,t.family),lineHeight:sM(Ot(n.lineHeight,t.lineHeight),e),size:e,style:i,weight:Ot(n.weight,t.weight),string:""};return s.string=qy(s),s}function cr(n,t,e,i){let s,r,o;for(s=0,r=n.length;s<r;++s)if(o=n[s],o!==void 0&&o!==void 0)return o}function oM(n,t,e){const{min:i,max:s}=n,r=ip(t,(s-i)/2),o=(a,l)=>e&&a===0?0:a+l;return{min:o(i,-Math.abs(r)),max:o(s,r)}}function Mi(n,t){return Object.assign(Object.create(n),t)}function Qc(n,t=[""],e,i,s=()=>n[0]){const r=e||n;typeof i>"u"&&(i=vp("_fallback",n));const o={[Symbol.toStringTag]:"Object",_cacheable:!0,_scopes:n,_rootScopes:r,_fallback:i,_getTarget:s,override:a=>Qc([a,...n],t,r,i)};return new Proxy(o,{deleteProperty(a,l){return delete a[l],delete a._keys,delete n[0][l],!0},get(a,l){return _p(a,l,()=>pM(l,t,n,a))},getOwnPropertyDescriptor(a,l){return Reflect.getOwnPropertyDescriptor(a._scopes[0],l)},getPrototypeOf(){return Reflect.getPrototypeOf(n[0])},has(a,l){return Ju(a).includes(l)},ownKeys(a){return Ju(a)},set(a,l,c){const h=a._storage||(a._storage=s());return a[l]=h[l]=c,delete a._keys,!0}})}function ks(n,t,e,i){const s={_cacheable:!1,_proxy:n,_context:t,_subProxy:e,_stack:new Set,_descriptors:gp(n,i),setContext:r=>ks(n,r,e,i),override:r=>ks(n.override(r),t,e,i)};return new Proxy(s,{deleteProperty(r,o){return delete r[o],delete n[o],!0},get(r,o,a){return _p(r,o,()=>lM(r,o,a))},getOwnPropertyDescriptor(r,o){return r._descriptors.allKeys?Reflect.has(n,o)?{enumerable:!0,configurable:!0}:void 0:Reflect.getOwnPropertyDescriptor(n,o)},getPrototypeOf(){return Reflect.getPrototypeOf(n)},has(r,o){return Reflect.has(n,o)},ownKeys(){return Reflect.ownKeys(n)},set(r,o,a){return n[o]=a,delete r[o],!0}})}function gp(n,t={scriptable:!0,indexable:!0}){const{_scriptable:e=t.scriptable,_indexable:i=t.indexable,_allKeys:s=t.allKeys}=n;return{allKeys:s,scriptable:e,indexable:i,isScriptable:bi(e)?e:()=>e,isIndexable:bi(i)?i:()=>i}}const aM=(n,t)=>n?n+$c(t):t,th=(n,t)=>Gt(t)&&n!=="adapters"&&(Object.getPrototypeOf(t)===null||t.constructor===Object);function _p(n,t,e){if(Object.prototype.hasOwnProperty.call(n,t)||t==="constructor")return n[t];const i=e();return n[t]=i,i}function lM(n,t,e){const{_proxy:i,_context:s,_subProxy:r,_descriptors:o}=n;let a=i[t];return bi(a)&&o.isScriptable(t)&&(a=cM(t,a,n,e)),me(a)&&a.length&&(a=hM(t,a,n,o.isIndexable)),th(t,a)&&(a=ks(a,s,r&&r[t],o)),a}function cM(n,t,e,i){const{_proxy:s,_context:r,_subProxy:o,_stack:a}=e;if(a.has(n))throw new Error("Recursion detected: "+Array.from(a).join("->")+"->"+n);a.add(n);let l=t(r,o||i);return a.delete(n),th(n,l)&&(l=eh(s._scopes,s,n,l)),l}function hM(n,t,e,i){const{_proxy:s,_context:r,_subProxy:o,_descriptors:a}=e;if(typeof r.index<"u"&&i(n))return t[r.index%t.length];if(Gt(t[0])){const l=t,c=s._scopes.filter(h=>h!==l);t=[];for(const h of l){const u=eh(c,s,n,h);t.push(ks(u,r,o&&o[n],a))}}return t}function xp(n,t,e){return bi(n)?n(t,e):n}const uM=(n,t)=>n===!0?t:typeof n=="string"?vi(t,n):void 0;function dM(n,t,e,i,s){for(const r of t){const o=uM(e,r);if(o){n.add(o);const a=xp(o._fallback,e,s);if(typeof a<"u"&&a!==e&&a!==i)return a}else if(o===!1&&typeof i<"u"&&e!==i)return null}return!1}function eh(n,t,e,i){const s=t._rootScopes,r=xp(t._fallback,e,i),o=[...n,...s],a=new Set;a.add(i);let l=Zu(a,o,e,r||e,i);return l===null||typeof r<"u"&&r!==e&&(l=Zu(a,o,r,l,i),l===null)?!1:Qc(Array.from(a),[""],s,r,()=>fM(t,e,i))}function Zu(n,t,e,i,s){for(;e;)e=dM(n,t,e,i,s);return e}function fM(n,t,e){const i=n._getTarget();t in i||(i[t]={});const s=i[t];return me(s)&&Gt(e)?e:s||{}}function pM(n,t,e,i){let s;for(const r of t)if(s=vp(aM(r,n),e),typeof s<"u")return th(n,s)?eh(e,i,n,s):s}function vp(n,t){for(const e of t){if(!e)continue;const i=e[n];if(typeof i<"u")return i}}function Ju(n){let t=n._keys;return t||(t=n._keys=mM(n._scopes)),t}function mM(n){const t=new Set;for(const e of n)for(const i of Object.keys(e).filter(s=>!s.startsWith("_")))t.add(i);return Array.from(t)}function bp(n,t,e,i){const{iScale:s}=n,{key:r="r"}=this._parsing,o=new Array(i);let a,l,c,h;for(a=0,l=i;a<l;++a)c=a+e,h=t[c],o[a]={r:s.parse(vi(h,r),c)};return o}const gM=Number.EPSILON||1e-14,Bs=(n,t)=>t<n.length&&!n[t].skip&&n[t],yp=n=>n==="x"?"y":"x";function _M(n,t,e,i){const s=n.skip?t:n,r=t,o=e.skip?t:e,a=pc(r,s),l=pc(o,r);let c=a/(a+l),h=l/(a+l);c=isNaN(c)?0:c,h=isNaN(h)?0:h;const u=i*c,d=i*h;return{previous:{x:r.x-u*(o.x-s.x),y:r.y-u*(o.y-s.y)},next:{x:r.x+d*(o.x-s.x),y:r.y+d*(o.y-s.y)}}}function xM(n,t,e){const i=n.length;let s,r,o,a,l,c=Bs(n,0);for(let h=0;h<i-1;++h)if(l=c,c=Bs(n,h+1),!(!l||!c)){if(gr(t[h],0,gM)){e[h]=e[h+1]=0;continue}s=e[h]/t[h],r=e[h+1]/t[h],a=Math.pow(s,2)+Math.pow(r,2),!(a<=9)&&(o=3/Math.sqrt(a),e[h]=s*o*t[h],e[h+1]=r*o*t[h])}}function vM(n,t,e="x"){const i=yp(e),s=n.length;let r,o,a,l=Bs(n,0);for(let c=0;c<s;++c){if(o=a,a=l,l=Bs(n,c+1),!a)continue;const h=a[e],u=a[i];o&&(r=(h-o[e])/3,a[`cp1${e}`]=h-r,a[`cp1${i}`]=u-r*t[c]),l&&(r=(l[e]-h)/3,a[`cp2${e}`]=h+r,a[`cp2${i}`]=u+r*t[c])}}function bM(n,t="x"){const e=yp(t),i=n.length,s=Array(i).fill(0),r=Array(i);let o,a,l,c=Bs(n,0);for(o=0;o<i;++o)if(a=l,l=c,c=Bs(n,o+1),!!l){if(c){const h=c[t]-l[t];s[o]=h!==0?(c[e]-l[e])/h:0}r[o]=a?c?Rn(s[o-1])!==Rn(s[o])?0:(s[o-1]+s[o])/2:s[o-1]:s[o]}xM(n,s,r),vM(n,r,t)}function So(n,t,e){return Math.max(Math.min(n,e),t)}function yM(n,t){let e,i,s,r,o,a=Kn(n[0],t);for(e=0,i=n.length;e<i;++e)o=r,r=a,a=e<i-1&&Kn(n[e+1],t),r&&(s=n[e],o&&(s.cp1x=So(s.cp1x,t.left,t.right),s.cp1y=So(s.cp1y,t.top,t.bottom)),a&&(s.cp2x=So(s.cp2x,t.left,t.right),s.cp2y=So(s.cp2y,t.top,t.bottom)))}function MM(n,t,e,i,s){let r,o,a,l;if(t.spanGaps&&(n=n.filter(c=>!c.skip)),t.cubicInterpolationMode==="monotone")bM(n,s);else{let c=i?n[n.length-1]:n[0];for(r=0,o=n.length;r<o;++r)a=n[r],l=_M(c,a,n[Math.min(r+1,o-(i?0:1))%o],t.tension),a.cp1x=l.previous.x,a.cp1y=l.previous.y,a.cp2x=l.next.x,a.cp2y=l.next.y,c=a}t.capBezierPoints&&yM(n,e)}function nh(){return typeof window<"u"&&typeof document<"u"}function ih(n){let t=n.parentNode;return t&&t.toString()==="[object ShadowRoot]"&&(t=t.host),t}function oa(n,t,e){let i;return typeof n=="string"?(i=parseInt(n,10),n.indexOf("%")!==-1&&(i=i/100*t.parentNode[e])):i=n,i}const _a=n=>n.ownerDocument.defaultView.getComputedStyle(n,null);function SM(n,t){return _a(n).getPropertyValue(t)}const EM=["top","right","bottom","left"];function Wi(n,t,e){const i={};e=e?"-"+e:"";for(let s=0;s<4;s++){const r=EM[s];i[r]=parseFloat(n[t+"-"+r+e])||0}return i.width=i.left+i.right,i.height=i.top+i.bottom,i}const TM=(n,t,e)=>(n>0||t>0)&&(!e||!e.shadowRoot);function wM(n,t){const e=n.touches,i=e&&e.length?e[0]:n,{offsetX:s,offsetY:r}=i;let o=!1,a,l;if(TM(s,r,n.target))a=s,l=r;else{const c=t.getBoundingClientRect();a=i.clientX-c.left,l=i.clientY-c.top,o=!0}return{x:a,y:l,box:o}}function Ui(n,t){if("native"in n)return n;const{canvas:e,currentDevicePixelRatio:i}=t,s=_a(e),r=s.boxSizing==="border-box",o=Wi(s,"padding"),a=Wi(s,"border","width"),{x:l,y:c,box:h}=wM(n,e),u=o.left+(h&&a.left),d=o.top+(h&&a.top);let{width:f,height:g}=t;return r&&(f-=o.width+a.width,g-=o.height+a.height),{x:Math.round((l-u)/f*e.width/i),y:Math.round((c-d)/g*e.height/i)}}function AM(n,t,e){let i,s;if(t===void 0||e===void 0){const r=n&&ih(n);if(!r)t=n.clientWidth,e=n.clientHeight;else{const o=r.getBoundingClientRect(),a=_a(r),l=Wi(a,"border","width"),c=Wi(a,"padding");t=o.width-c.width-l.width,e=o.height-c.height-l.height,i=oa(a.maxWidth,r,"clientWidth"),s=oa(a.maxHeight,r,"clientHeight")}}return{width:t,height:e,maxWidth:i||sa,maxHeight:s||sa}}const Eo=n=>Math.round(n*10)/10;function CM(n,t,e,i){const s=_a(n),r=Wi(s,"margin"),o=oa(s.maxWidth,n,"clientWidth")||sa,a=oa(s.maxHeight,n,"clientHeight")||sa,l=AM(n,t,e);let{width:c,height:h}=l;if(s.boxSizing==="content-box"){const d=Wi(s,"border","width"),f=Wi(s,"padding");c-=f.width+d.width,h-=f.height+d.height}return c=Math.max(0,c-r.width),h=Math.max(0,i?c/i:h-r.height),c=Eo(Math.min(c,o,l.maxWidth)),h=Eo(Math.min(h,a,l.maxHeight)),c&&!h&&(h=Eo(c/2)),(t!==void 0||e!==void 0)&&i&&l.height&&h>l.height&&(h=l.height,c=Eo(Math.floor(h*i))),{width:c,height:h}}function Qu(n,t,e){const i=t||1,s=Math.floor(n.height*i),r=Math.floor(n.width*i);n.height=Math.floor(n.height),n.width=Math.floor(n.width);const o=n.canvas;return o.style&&(e||!o.style.height&&!o.style.width)&&(o.style.height=`${n.height}px`,o.style.width=`${n.width}px`),n.currentDevicePixelRatio!==i||o.height!==s||o.width!==r?(n.currentDevicePixelRatio=i,o.height=s,o.width=r,n.ctx.setTransform(i,0,0,i,0,0),!0):!1}const RM=function(){let n=!1;try{const t={get passive(){return n=!0,!1}};nh()&&(window.addEventListener("test",null,t),window.removeEventListener("test",null,t))}catch{}return n}();function td(n,t){const e=SM(n,t),i=e&&e.match(/^(\d+)(\.\d+)?px$/);return i?+i[1]:void 0}function Ni(n,t,e,i){return{x:n.x+e*(t.x-n.x),y:n.y+e*(t.y-n.y)}}function PM(n,t,e,i){return{x:n.x+e*(t.x-n.x),y:i==="middle"?e<.5?n.y:t.y:i==="after"?e<1?n.y:t.y:e>0?t.y:n.y}}function DM(n,t,e,i){const s={x:n.cp2x,y:n.cp2y},r={x:t.cp1x,y:t.cp1y},o=Ni(n,s,e),a=Ni(s,r,e),l=Ni(r,t,e),c=Ni(o,a,e),h=Ni(a,l,e);return Ni(c,h,e)}const LM=function(n,t){return{x(e){return n+n+t-e},setWidth(e){t=e},textAlign(e){return e==="center"?e:e==="right"?"left":"right"},xPlus(e,i){return e-i},leftForLtr(e,i){return e-i}}},IM=function(){return{x(n){return n},setWidth(n){},textAlign(n){return n},xPlus(n,t){return n+t},leftForLtr(n,t){return n}}};function Rs(n,t,e){return n?LM(t,e):IM()}function Mp(n,t){let e,i;(t==="ltr"||t==="rtl")&&(e=n.canvas.style,i=[e.getPropertyValue("direction"),e.getPropertyPriority("direction")],e.setProperty("direction",t,"important"),n.prevTextDirection=i)}function Sp(n,t){t!==void 0&&(delete n.prevTextDirection,n.canvas.style.setProperty("direction",t[0],t[1]))}function Ep(n){return n==="angle"?{between:Tr,compare:Oy,normalize:tn}:{between:Yn,compare:(t,e)=>t-e,normalize:t=>t}}function ed({start:n,end:t,count:e,loop:i,style:s}){return{start:n%e,end:t%e,loop:i&&(t-n+1)%e===0,style:s}}function OM(n,t,e){const{property:i,start:s,end:r}=e,{between:o,normalize:a}=Ep(i),l=t.length;let{start:c,end:h,loop:u}=n,d,f;if(u){for(c+=l,h+=l,d=0,f=l;d<f&&o(a(t[c%l][i]),s,r);++d)c--,h--;c%=l,h%=l}return h<c&&(h+=l),{start:c,end:h,loop:u,style:n.style}}function Tp(n,t,e){if(!e)return[n];const{property:i,start:s,end:r}=e,o=t.length,{compare:a,between:l,normalize:c}=Ep(i),{start:h,end:u,loop:d,style:f}=OM(n,t,e),g=[];let _=!1,m=null,p,v,y;const x=()=>l(s,y,p)&&a(s,y)!==0,A=()=>a(r,p)===0||l(r,y,p),w=()=>_||x(),T=()=>!_||A();for(let R=h,S=h;R<=u;++R)v=t[R%o],!v.skip&&(p=c(v[i]),p!==y&&(_=l(p,s,r),m===null&&w()&&(m=a(p,s)===0?R:S),m!==null&&T()&&(g.push(ed({start:m,end:R,loop:d,count:o,style:f})),m=null),S=R,y=p));return m!==null&&g.push(ed({start:m,end:u,loop:d,count:o,style:f})),g}function wp(n,t){const e=[],i=n.segments;for(let s=0;s<i.length;s++){const r=Tp(i[s],n.points,t);r.length&&e.push(...r)}return e}function UM(n,t,e,i){let s=0,r=t-1;if(e&&!i)for(;s<t&&!n[s].skip;)s++;for(;s<t&&n[s].skip;)s++;for(s%=t,e&&(r+=s);r>s&&n[r%t].skip;)r--;return r%=t,{start:s,end:r}}function NM(n,t,e,i){const s=n.length,r=[];let o=t,a=n[t],l;for(l=t+1;l<=e;++l){const c=n[l%s];c.skip||c.stop?a.skip||(i=!1,r.push({start:t%s,end:(l-1)%s,loop:i}),t=o=c.stop?l:null):(o=l,a.skip&&(t=l)),a=c}return o!==null&&r.push({start:t%s,end:o%s,loop:i}),r}function FM(n,t){const e=n.points,i=n.options.spanGaps,s=e.length;if(!s)return[];const r=!!n._loop,{start:o,end:a}=UM(e,s,r,i);if(i===!0)return nd(n,[{start:o,end:a,loop:r}],e,t);const l=a<o?a+s:a,c=!!n._fullLoop&&o===0&&a===s-1;return nd(n,NM(e,o,l,c),e,t)}function nd(n,t,e,i){return!i||!i.setContext||!e?t:kM(n,t,e,i)}function kM(n,t,e,i){const s=n._chart.getContext(),r=id(n.options),{_datasetIndex:o,options:{spanGaps:a}}=n,l=e.length,c=[];let h=r,u=t[0].start,d=u;function f(g,_,m,p){const v=a?-1:1;if(g!==_){for(g+=l;e[g%l].skip;)g-=v;for(;e[_%l].skip;)_+=v;g%l!==_%l&&(c.push({start:g%l,end:_%l,loop:m,style:p}),h=p,u=_%l)}}for(const g of t){u=a?u:g.start;let _=e[u%l],m;for(d=u+1;d<=g.end;d++){const p=e[d%l];m=id(i.setContext(Mi(s,{type:"segment",p0:_,p1:p,p0DataIndex:(d-1)%l,p1DataIndex:d%l,datasetIndex:o}))),BM(m,h)&&f(u,d-1,g.loop,h),_=p,h=m}u<d-1&&f(u,d-1,g.loop,h)}return c}function id(n){return{backgroundColor:n.backgroundColor,borderCapStyle:n.borderCapStyle,borderDash:n.borderDash,borderDashOffset:n.borderDashOffset,borderJoinStyle:n.borderJoinStyle,borderWidth:n.borderWidth,borderColor:n.borderColor}}function BM(n,t){if(!t)return!1;const e=[],i=function(s,r){return Zc(r)?(e.includes(r)||e.push(r),e.indexOf(r)):r};return JSON.stringify(n,i)!==JSON.stringify(t,i)}/*!
 * Chart.js v4.4.7
 * https://www.chartjs.org
 * (c) 2024 Chart.js Contributors
 * Released under the MIT License
 */class zM{constructor(){this._request=null,this._charts=new Map,this._running=!1,this._lastDate=void 0}_notify(t,e,i,s){const r=e.listeners[s],o=e.duration;r.forEach(a=>a({chart:t,initial:e.initial,numSteps:o,currentStep:Math.min(i-e.start,o)}))}_refresh(){this._request||(this._running=!0,this._request=cp.call(window,()=>{this._update(),this._request=null,this._running&&this._refresh()}))}_update(t=Date.now()){let e=0;this._charts.forEach((i,s)=>{if(!i.running||!i.items.length)return;const r=i.items;let o=r.length-1,a=!1,l;for(;o>=0;--o)l=r[o],l._active?(l._total>i.duration&&(i.duration=l._total),l.tick(t),a=!0):(r[o]=r[r.length-1],r.pop());a&&(s.draw(),this._notify(s,i,t,"progress")),r.length||(i.running=!1,this._notify(s,i,t,"complete"),i.initial=!1),e+=r.length}),this._lastDate=t,e===0&&(this._running=!1)}_getAnims(t){const e=this._charts;let i=e.get(t);return i||(i={running:!1,initial:!0,items:[],listeners:{complete:[],progress:[]}},e.set(t,i)),i}listen(t,e,i){this._getAnims(t).listeners[e].push(i)}add(t,e){!e||!e.length||this._getAnims(t).items.push(...e)}has(t){return this._getAnims(t).items.length>0}start(t){const e=this._charts.get(t);e&&(e.running=!0,e.start=Date.now(),e.duration=e.items.reduce((i,s)=>Math.max(i,s._duration),0),this._refresh())}running(t){if(!this._running)return!1;const e=this._charts.get(t);return!(!e||!e.running||!e.items.length)}stop(t){const e=this._charts.get(t);if(!e||!e.items.length)return;const i=e.items;let s=i.length-1;for(;s>=0;--s)i[s].cancel();e.items=[],this._notify(t,e,Date.now(),"complete")}remove(t){return this._charts.delete(t)}}var Hn=new zM;const sd="transparent",HM={boolean(n,t,e){return e>.5?t:n},color(n,t,e){const i=Yu(n||sd),s=i.valid&&Yu(t||sd);return s&&s.valid?s.mix(i,e).hexString():t},number(n,t,e){return n+(t-n)*e}};class VM{constructor(t,e,i,s){const r=e[i];s=cr([t.to,s,r,t.from]);const o=cr([t.from,r,s]);this._active=!0,this._fn=t.fn||HM[t.type||typeof o],this._easing=_r[t.easing]||_r.linear,this._start=Math.floor(Date.now()+(t.delay||0)),this._duration=this._total=Math.floor(t.duration),this._loop=!!t.loop,this._target=e,this._prop=i,this._from=o,this._to=s,this._promises=void 0}active(){return this._active}update(t,e,i){if(this._active){this._notify(!1);const s=this._target[this._prop],r=i-this._start,o=this._duration-r;this._start=i,this._duration=Math.floor(Math.max(o,t.duration)),this._total+=r,this._loop=!!t.loop,this._to=cr([t.to,e,s,t.from]),this._from=cr([t.from,s,e])}}cancel(){this._active&&(this.tick(Date.now()),this._active=!1,this._notify(!1))}tick(t){const e=t-this._start,i=this._duration,s=this._prop,r=this._from,o=this._loop,a=this._to;let l;if(this._active=r!==a&&(o||e<i),!this._active){this._target[s]=a,this._notify(!0);return}if(e<0){this._target[s]=r;return}l=e/i%2,l=o&&l>1?2-l:l,l=this._easing(Math.min(1,Math.max(0,l))),this._target[s]=this._fn(r,a,l)}wait(){const t=this._promises||(this._promises=[]);return new Promise((e,i)=>{t.push({res:e,rej:i})})}_notify(t){const e=t?"res":"rej",i=this._promises||[];for(let s=0;s<i.length;s++)i[s][e]()}}class Ap{constructor(t,e){this._chart=t,this._properties=new Map,this.configure(e)}configure(t){if(!Gt(t))return;const e=Object.keys(ge.animation),i=this._properties;Object.getOwnPropertyNames(t).forEach(s=>{const r=t[s];if(!Gt(r))return;const o={};for(const a of e)o[a]=r[a];(me(r.properties)&&r.properties||[s]).forEach(a=>{(a===s||!i.has(a))&&i.set(a,o)})})}_animateOptions(t,e){const i=e.options,s=WM(t,i);if(!s)return[];const r=this._createAnimations(s,i);return i.$shared&&GM(t.options.$animations,i).then(()=>{t.options=i},()=>{}),r}_createAnimations(t,e){const i=this._properties,s=[],r=t.$animations||(t.$animations={}),o=Object.keys(e),a=Date.now();let l;for(l=o.length-1;l>=0;--l){const c=o[l];if(c.charAt(0)==="$")continue;if(c==="options"){s.push(...this._animateOptions(t,e));continue}const h=e[c];let u=r[c];const d=i.get(c);if(u)if(d&&u.active()){u.update(d,h,a);continue}else u.cancel();if(!d||!d.duration){t[c]=h;continue}r[c]=u=new VM(d,t,c,h),s.push(u)}return s}update(t,e){if(this._properties.size===0){Object.assign(t,e);return}const i=this._createAnimations(t,e);if(i.length)return Hn.add(this._chart,i),!0}}function GM(n,t){const e=[],i=Object.keys(t);for(let s=0;s<i.length;s++){const r=n[i[s]];r&&r.active()&&e.push(r.wait())}return Promise.all(e)}function WM(n,t){if(!t)return;let e=n.options;if(!e){n.options=t;return}return e.$shared&&(n.options=e=Object.assign({},e,{$shared:!1,$animations:{}})),e}function rd(n,t){const e=n&&n.options||{},i=e.reverse,s=e.min===void 0?t:0,r=e.max===void 0?t:0;return{start:i?r:s,end:i?s:r}}function XM(n,t,e){if(e===!1)return!1;const i=rd(n,e),s=rd(t,e);return{top:s.end,right:i.end,bottom:s.start,left:i.start}}function jM(n){let t,e,i,s;return Gt(n)?(t=n.top,e=n.right,i=n.bottom,s=n.left):t=e=i=s=n,{top:t,right:e,bottom:i,left:s,disabled:n===!1}}function Cp(n,t){const e=[],i=n._getSortedDatasetMetas(t);let s,r;for(s=0,r=i.length;s<r;++s)e.push(i[s].index);return e}function od(n,t,e,i={}){const s=n.keys,r=i.mode==="single";let o,a,l,c;if(t===null)return;let h=!1;for(o=0,a=s.length;o<a;++o){if(l=+s[o],l===e){if(h=!0,i.all)continue;break}c=n.values[l],ye(c)&&(r||t===0||Rn(t)===Rn(c))&&(t+=c)}return!h&&!i.all?0:t}function $M(n,t){const{iScale:e,vScale:i}=t,s=e.axis==="x"?"x":"y",r=i.axis==="x"?"x":"y",o=Object.keys(n),a=new Array(o.length);let l,c,h;for(l=0,c=o.length;l<c;++l)h=o[l],a[l]={[s]:h,[r]:n[h]};return a}function ul(n,t){const e=n&&n.options.stacked;return e||e===void 0&&t.stack!==void 0}function YM(n,t,e){return`${n.id}.${t.id}.${e.stack||e.type}`}function qM(n){const{min:t,max:e,minDefined:i,maxDefined:s}=n.getUserBounds();return{min:i?t:Number.NEGATIVE_INFINITY,max:s?e:Number.POSITIVE_INFINITY}}function KM(n,t,e){const i=n[t]||(n[t]={});return i[e]||(i[e]={})}function ad(n,t,e,i){for(const s of t.getMatchingVisibleMetas(i).reverse()){const r=n[s.index];if(e&&r>0||!e&&r<0)return s.index}return null}function ld(n,t){const{chart:e,_cachedMeta:i}=n,s=e._stacks||(e._stacks={}),{iScale:r,vScale:o,index:a}=i,l=r.axis,c=o.axis,h=YM(r,o,i),u=t.length;let d;for(let f=0;f<u;++f){const g=t[f],{[l]:_,[c]:m}=g,p=g._stacks||(g._stacks={});d=p[c]=KM(s,h,_),d[a]=m,d._top=ad(d,o,!0,i.type),d._bottom=ad(d,o,!1,i.type);const v=d._visualValues||(d._visualValues={});v[a]=m}}function dl(n,t){const e=n.scales;return Object.keys(e).filter(i=>e[i].axis===t).shift()}function ZM(n,t){return Mi(n,{active:!1,dataset:void 0,datasetIndex:t,index:t,mode:"default",type:"dataset"})}function JM(n,t,e){return Mi(n,{active:!1,dataIndex:t,parsed:void 0,raw:void 0,element:e,index:t,mode:"default",type:"data"})}function tr(n,t){const e=n.controller.index,i=n.vScale&&n.vScale.axis;if(i){t=t||n._parsed;for(const s of t){const r=s._stacks;if(!r||r[i]===void 0||r[i][e]===void 0)return;delete r[i][e],r[i]._visualValues!==void 0&&r[i]._visualValues[e]!==void 0&&delete r[i]._visualValues[e]}}}const fl=n=>n==="reset"||n==="none",cd=(n,t)=>t?n:Object.assign({},n),QM=(n,t,e)=>n&&!t.hidden&&t._stacked&&{keys:Cp(e,!0),values:null};class vn{constructor(t,e){this.chart=t,this._ctx=t.ctx,this.index=e,this._cachedDataOpts={},this._cachedMeta=this.getMeta(),this._type=this._cachedMeta.type,this.options=void 0,this._parsing=!1,this._data=void 0,this._objectData=void 0,this._sharedOptions=void 0,this._drawStart=void 0,this._drawCount=void 0,this.enableOptionSharing=!1,this.supportsDecimation=!1,this.$context=void 0,this._syncList=[],this.datasetElementType=new.target.datasetElementType,this.dataElementType=new.target.dataElementType,this.initialize()}initialize(){const t=this._cachedMeta;this.configure(),this.linkScales(),t._stacked=ul(t.vScale,t),this.addElements(),this.options.fill&&!this.chart.isPluginEnabled("filler")&&console.warn("Tried to use the 'fill' option without the 'Filler' plugin enabled. Please import and register the 'Filler' plugin and make sure it is not disabled in the options")}updateIndex(t){this.index!==t&&tr(this._cachedMeta),this.index=t}linkScales(){const t=this.chart,e=this._cachedMeta,i=this.getDataset(),s=(u,d,f,g)=>u==="x"?d:u==="r"?g:f,r=e.xAxisID=Ot(i.xAxisID,dl(t,"x")),o=e.yAxisID=Ot(i.yAxisID,dl(t,"y")),a=e.rAxisID=Ot(i.rAxisID,dl(t,"r")),l=e.indexAxis,c=e.iAxisID=s(l,r,o,a),h=e.vAxisID=s(l,o,r,a);e.xScale=this.getScaleForId(r),e.yScale=this.getScaleForId(o),e.rScale=this.getScaleForId(a),e.iScale=this.getScaleForId(c),e.vScale=this.getScaleForId(h)}getDataset(){return this.chart.data.datasets[this.index]}getMeta(){return this.chart.getDatasetMeta(this.index)}getScaleForId(t){return this.chart.scales[t]}_getOtherScale(t){const e=this._cachedMeta;return t===e.iScale?e.vScale:e.iScale}reset(){this._update("reset")}_destroy(){const t=this._cachedMeta;this._data&&Xu(this._data,this),t._stacked&&tr(t)}_dataCheck(){const t=this.getDataset(),e=t.data||(t.data=[]),i=this._data;if(Gt(e)){const s=this._cachedMeta;this._data=$M(e,s)}else if(i!==e){if(i){Xu(i,this);const s=this._cachedMeta;tr(s),s._parsed=[]}e&&Object.isExtensible(e)&&ky(e,this),this._syncList=[],this._data=e}}addElements(){const t=this._cachedMeta;this._dataCheck(),this.datasetElementType&&(t.dataset=new this.datasetElementType)}buildOrUpdateElements(t){const e=this._cachedMeta,i=this.getDataset();let s=!1;this._dataCheck();const r=e._stacked;e._stacked=ul(e.vScale,e),e.stack!==i.stack&&(s=!0,tr(e),e.stack=i.stack),this._resyncElements(t),(s||r!==e._stacked)&&(ld(this,e._parsed),e._stacked=ul(e.vScale,e))}configure(){const t=this.chart.config,e=t.datasetScopeKeys(this._type),i=t.getOptionScopes(this.getDataset(),e,!0);this.options=t.createResolver(i,this.getContext()),this._parsing=this.options.parsing,this._cachedDataOpts={}}parse(t,e){const{_cachedMeta:i,_data:s}=this,{iScale:r,_stacked:o}=i,a=r.axis;let l=t===0&&e===s.length?!0:i._sorted,c=t>0&&i._parsed[t-1],h,u,d;if(this._parsing===!1)i._parsed=s,i._sorted=!0,d=s;else{me(s[t])?d=this.parseArrayData(i,s,t,e):Gt(s[t])?d=this.parseObjectData(i,s,t,e):d=this.parsePrimitiveData(i,s,t,e);const f=()=>u[a]===null||c&&u[a]<c[a];for(h=0;h<e;++h)i._parsed[h+t]=u=d[h],l&&(f()&&(l=!1),c=u);i._sorted=l}o&&ld(this,d)}parsePrimitiveData(t,e,i,s){const{iScale:r,vScale:o}=t,a=r.axis,l=o.axis,c=r.getLabels(),h=r===o,u=new Array(s);let d,f,g;for(d=0,f=s;d<f;++d)g=d+i,u[d]={[a]:h||r.parse(c[g],g),[l]:o.parse(e[g],g)};return u}parseArrayData(t,e,i,s){const{xScale:r,yScale:o}=t,a=new Array(s);let l,c,h,u;for(l=0,c=s;l<c;++l)h=l+i,u=e[h],a[l]={x:r.parse(u[0],h),y:o.parse(u[1],h)};return a}parseObjectData(t,e,i,s){const{xScale:r,yScale:o}=t,{xAxisKey:a="x",yAxisKey:l="y"}=this._parsing,c=new Array(s);let h,u,d,f;for(h=0,u=s;h<u;++h)d=h+i,f=e[d],c[h]={x:r.parse(vi(f,a),d),y:o.parse(vi(f,l),d)};return c}getParsed(t){return this._cachedMeta._parsed[t]}getDataElement(t){return this._cachedMeta.data[t]}applyStack(t,e,i){const s=this.chart,r=this._cachedMeta,o=e[t.axis],a={keys:Cp(s,!0),values:e._stacks[t.axis]._visualValues};return od(a,o,r.index,{mode:i})}updateRangeFromParsed(t,e,i,s){const r=i[e.axis];let o=r===null?NaN:r;const a=s&&i._stacks[e.axis];s&&a&&(s.values=a,o=od(s,r,this._cachedMeta.index)),t.min=Math.min(t.min,o),t.max=Math.max(t.max,o)}getMinMax(t,e){const i=this._cachedMeta,s=i._parsed,r=i._sorted&&t===i.iScale,o=s.length,a=this._getOtherScale(t),l=QM(e,i,this.chart),c={min:Number.POSITIVE_INFINITY,max:Number.NEGATIVE_INFINITY},{min:h,max:u}=qM(a);let d,f;function g(){f=s[d];const _=f[a.axis];return!ye(f[t.axis])||h>_||u<_}for(d=0;d<o&&!(!g()&&(this.updateRangeFromParsed(c,t,f,l),r));++d);if(r){for(d=o-1;d>=0;--d)if(!g()){this.updateRangeFromParsed(c,t,f,l);break}}return c}getAllParsedValues(t){const e=this._cachedMeta._parsed,i=[];let s,r,o;for(s=0,r=e.length;s<r;++s)o=e[s][t.axis],ye(o)&&i.push(o);return i}getMaxOverflow(){return!1}getLabelAndValue(t){const e=this._cachedMeta,i=e.iScale,s=e.vScale,r=this.getParsed(t);return{label:i?""+i.getLabelForValue(r[i.axis]):"",value:s?""+s.getLabelForValue(r[s.axis]):""}}_update(t){const e=this._cachedMeta;this.update(t||"default"),e._clip=jM(Ot(this.options.clip,XM(e.xScale,e.yScale,this.getMaxOverflow())))}update(t){}draw(){const t=this._ctx,e=this.chart,i=this._cachedMeta,s=i.data||[],r=e.chartArea,o=[],a=this._drawStart||0,l=this._drawCount||s.length-a,c=this.options.drawActiveElementsOnTop;let h;for(i.dataset&&i.dataset.draw(t,r,a,l),h=a;h<a+l;++h){const u=s[h];u.hidden||(u.active&&c?o.push(u):u.draw(t,r))}for(h=0;h<o.length;++h)o[h].draw(t,r)}getStyle(t,e){const i=e?"active":"default";return t===void 0&&this._cachedMeta.dataset?this.resolveDatasetElementOptions(i):this.resolveDataElementOptions(t||0,i)}getContext(t,e,i){const s=this.getDataset();let r;if(t>=0&&t<this._cachedMeta.data.length){const o=this._cachedMeta.data[t];r=o.$context||(o.$context=JM(this.getContext(),t,o)),r.parsed=this.getParsed(t),r.raw=s.data[t],r.index=r.dataIndex=t}else r=this.$context||(this.$context=ZM(this.chart.getContext(),this.index)),r.dataset=s,r.index=r.datasetIndex=this.index;return r.active=!!e,r.mode=i,r}resolveDatasetElementOptions(t){return this._resolveElementOptions(this.datasetElementType.id,t)}resolveDataElementOptions(t,e){return this._resolveElementOptions(this.dataElementType.id,e,t)}_resolveElementOptions(t,e="default",i){const s=e==="active",r=this._cachedDataOpts,o=t+"-"+e,a=r[o],l=this.enableOptionSharing&&Er(i);if(a)return cd(a,l);const c=this.chart.config,h=c.datasetElementScopeKeys(this._type,t),u=s?[`${t}Hover`,"hover",t,""]:[t,""],d=c.getOptionScopes(this.getDataset(),h),f=Object.keys(ge.elements[t]),g=()=>this.getContext(i,s,e),_=c.resolveNamedOptions(d,f,g,u);return _.$shared&&(_.$shared=l,r[o]=Object.freeze(cd(_,l))),_}_resolveAnimations(t,e,i){const s=this.chart,r=this._cachedDataOpts,o=`animation-${e}`,a=r[o];if(a)return a;let l;if(s.options.animation!==!1){const h=this.chart.config,u=h.datasetAnimationScopeKeys(this._type,e),d=h.getOptionScopes(this.getDataset(),u);l=h.createResolver(d,this.getContext(t,i,e))}const c=new Ap(s,l&&l.animations);return l&&l._cacheable&&(r[o]=Object.freeze(c)),c}getSharedOptions(t){if(t.$shared)return this._sharedOptions||(this._sharedOptions=Object.assign({},t))}includeOptions(t,e){return!e||fl(t)||this.chart._animationsDisabled}_getSharedOptions(t,e){const i=this.resolveDataElementOptions(t,e),s=this._sharedOptions,r=this.getSharedOptions(i),o=this.includeOptions(e,r)||r!==s;return this.updateSharedOptions(r,e,i),{sharedOptions:r,includeOptions:o}}updateElement(t,e,i,s){fl(s)?Object.assign(t,i):this._resolveAnimations(e,s).update(t,i)}updateSharedOptions(t,e,i){t&&!fl(e)&&this._resolveAnimations(void 0,e).update(t,i)}_setStyle(t,e,i,s){t.active=s;const r=this.getStyle(e,s);this._resolveAnimations(e,i,s).update(t,{options:!s&&this.getSharedOptions(r)||r})}removeHoverStyle(t,e,i){this._setStyle(t,i,"active",!1)}setHoverStyle(t,e,i){this._setStyle(t,i,"active",!0)}_removeDatasetHoverStyle(){const t=this._cachedMeta.dataset;t&&this._setStyle(t,void 0,"active",!1)}_setDatasetHoverStyle(){const t=this._cachedMeta.dataset;t&&this._setStyle(t,void 0,"active",!0)}_resyncElements(t){const e=this._data,i=this._cachedMeta.data;for(const[a,l,c]of this._syncList)this[a](l,c);this._syncList=[];const s=i.length,r=e.length,o=Math.min(r,s);o&&this.parse(0,o),r>s?this._insertElements(s,r-s,t):r<s&&this._removeElements(r,s-r)}_insertElements(t,e,i=!0){const s=this._cachedMeta,r=s.data,o=t+e;let a;const l=c=>{for(c.length+=e,a=c.length-1;a>=o;a--)c[a]=c[a-e]};for(l(r),a=t;a<o;++a)r[a]=new this.dataElementType;this._parsing&&l(s._parsed),this.parse(t,e),i&&this.updateElements(r,t,e,"reset")}updateElements(t,e,i,s){}_removeElements(t,e){const i=this._cachedMeta;if(this._parsing){const s=i._parsed.splice(t,e);i._stacked&&tr(i,s)}i.data.splice(t,e)}_sync(t){if(this._parsing)this._syncList.push(t);else{const[e,i,s]=t;this[e](i,s)}this.chart._dataChanges.push([this.index,...t])}_onDataPush(){const t=arguments.length;this._sync(["_insertElements",this.getDataset().data.length-t,t])}_onDataPop(){this._sync(["_removeElements",this._cachedMeta.data.length-1,1])}_onDataShift(){this._sync(["_removeElements",0,1])}_onDataSplice(t,e){e&&this._sync(["_removeElements",t,e]);const i=arguments.length-2;i&&this._sync(["_insertElements",t,i])}_onDataUnshift(){this._sync(["_insertElements",0,arguments.length])}}st(vn,"defaults",{}),st(vn,"datasetElementType",null),st(vn,"dataElementType",null);function tS(n,t){if(!n._cache.$bar){const e=n.getMatchingVisibleMetas(t);let i=[];for(let s=0,r=e.length;s<r;s++)i=i.concat(e[s].controller.getAllParsedValues(n));n._cache.$bar=lp(i.sort((s,r)=>s-r))}return n._cache.$bar}function eS(n){const t=n.iScale,e=tS(t,n.type);let i=t._length,s,r,o,a;const l=()=>{o===32767||o===-32768||(Er(a)&&(i=Math.min(i,Math.abs(o-a)||i)),a=o)};for(s=0,r=e.length;s<r;++s)o=t.getPixelForValue(e[s]),l();for(a=void 0,s=0,r=t.ticks.length;s<r;++s)o=t.getPixelForTick(s),l();return i}function nS(n,t,e,i){const s=e.barThickness;let r,o;return Zt(s)?(r=t.min*e.categoryPercentage,o=e.barPercentage):(r=s*i,o=1),{chunk:r/i,ratio:o,start:t.pixels[n]-r/2}}function iS(n,t,e,i){const s=t.pixels,r=s[n];let o=n>0?s[n-1]:null,a=n<s.length-1?s[n+1]:null;const l=e.categoryPercentage;o===null&&(o=r-(a===null?t.end-t.start:a-r)),a===null&&(a=r+r-o);const c=r-(r-Math.min(o,a))/2*l;return{chunk:Math.abs(a-o)/2*l/i,ratio:e.barPercentage,start:c}}function sS(n,t,e,i){const s=e.parse(n[0],i),r=e.parse(n[1],i),o=Math.min(s,r),a=Math.max(s,r);let l=o,c=a;Math.abs(o)>Math.abs(a)&&(l=a,c=o),t[e.axis]=c,t._custom={barStart:l,barEnd:c,start:s,end:r,min:o,max:a}}function Rp(n,t,e,i){return me(n)?sS(n,t,e,i):t[e.axis]=e.parse(n,i),t}function hd(n,t,e,i){const s=n.iScale,r=n.vScale,o=s.getLabels(),a=s===r,l=[];let c,h,u,d;for(c=e,h=e+i;c<h;++c)d=t[c],u={},u[s.axis]=a||s.parse(o[c],c),l.push(Rp(d,u,r,c));return l}function pl(n){return n&&n.barStart!==void 0&&n.barEnd!==void 0}function rS(n,t,e){return n!==0?Rn(n):(t.isHorizontal()?1:-1)*(t.min>=e?1:-1)}function oS(n){let t,e,i,s,r;return n.horizontal?(t=n.base>n.x,e="left",i="right"):(t=n.base<n.y,e="bottom",i="top"),t?(s="end",r="start"):(s="start",r="end"),{start:e,end:i,reverse:t,top:s,bottom:r}}function aS(n,t,e,i){let s=t.borderSkipped;const r={};if(!s){n.borderSkipped=r;return}if(s===!0){n.borderSkipped={top:!0,right:!0,bottom:!0,left:!0};return}const{start:o,end:a,reverse:l,top:c,bottom:h}=oS(n);s==="middle"&&e&&(n.enableBorderRadius=!0,(e._top||0)===i?s=c:(e._bottom||0)===i?s=h:(r[ud(h,o,a,l)]=!0,s=c)),r[ud(s,o,a,l)]=!0,n.borderSkipped=r}function ud(n,t,e,i){return i?(n=lS(n,t,e),n=dd(n,e,t)):n=dd(n,t,e),n}function lS(n,t,e){return n===t?e:n===e?t:n}function dd(n,t,e){return n==="start"?t:n==="end"?e:n}function cS(n,{inflateAmount:t},e){n.inflateAmount=t==="auto"?e===1?.33:0:t}class Ho extends vn{parsePrimitiveData(t,e,i,s){return hd(t,e,i,s)}parseArrayData(t,e,i,s){return hd(t,e,i,s)}parseObjectData(t,e,i,s){const{iScale:r,vScale:o}=t,{xAxisKey:a="x",yAxisKey:l="y"}=this._parsing,c=r.axis==="x"?a:l,h=o.axis==="x"?a:l,u=[];let d,f,g,_;for(d=i,f=i+s;d<f;++d)_=e[d],g={},g[r.axis]=r.parse(vi(_,c),d),u.push(Rp(vi(_,h),g,o,d));return u}updateRangeFromParsed(t,e,i,s){super.updateRangeFromParsed(t,e,i,s);const r=i._custom;r&&e===this._cachedMeta.vScale&&(t.min=Math.min(t.min,r.min),t.max=Math.max(t.max,r.max))}getMaxOverflow(){return 0}getLabelAndValue(t){const e=this._cachedMeta,{iScale:i,vScale:s}=e,r=this.getParsed(t),o=r._custom,a=pl(o)?"["+o.start+", "+o.end+"]":""+s.getLabelForValue(r[s.axis]);return{label:""+i.getLabelForValue(r[i.axis]),value:a}}initialize(){this.enableOptionSharing=!0,super.initialize();const t=this._cachedMeta;t.stack=this.getDataset().stack}update(t){const e=this._cachedMeta;this.updateElements(e.data,0,e.data.length,t)}updateElements(t,e,i,s){const r=s==="reset",{index:o,_cachedMeta:{vScale:a}}=this,l=a.getBasePixel(),c=a.isHorizontal(),h=this._getRuler(),{sharedOptions:u,includeOptions:d}=this._getSharedOptions(e,s);for(let f=e;f<e+i;f++){const g=this.getParsed(f),_=r||Zt(g[a.axis])?{base:l,head:l}:this._calculateBarValuePixels(f),m=this._calculateBarIndexPixels(f,h),p=(g._stacks||{})[a.axis],v={horizontal:c,base:_.base,enableBorderRadius:!p||pl(g._custom)||o===p._top||o===p._bottom,x:c?_.head:m.center,y:c?m.center:_.head,height:c?m.size:Math.abs(_.size),width:c?Math.abs(_.size):m.size};d&&(v.options=u||this.resolveDataElementOptions(f,t[f].active?"active":s));const y=v.options||t[f].options;aS(v,y,p,o),cS(v,y,h.ratio),this.updateElement(t[f],f,v,s)}}_getStacks(t,e){const{iScale:i}=this._cachedMeta,s=i.getMatchingVisibleMetas(this._type).filter(h=>h.controller.options.grouped),r=i.options.stacked,o=[],a=this._cachedMeta.controller.getParsed(e),l=a&&a[i.axis],c=h=>{const u=h._parsed.find(f=>f[i.axis]===l),d=u&&u[h.vScale.axis];if(Zt(d)||isNaN(d))return!0};for(const h of s)if(!(e!==void 0&&c(h))&&((r===!1||o.indexOf(h.stack)===-1||r===void 0&&h.stack===void 0)&&o.push(h.stack),h.index===t))break;return o.length||o.push(void 0),o}_getStackCount(t){return this._getStacks(void 0,t).length}_getStackIndex(t,e,i){const s=this._getStacks(t,i),r=e!==void 0?s.indexOf(e):-1;return r===-1?s.length-1:r}_getRuler(){const t=this.options,e=this._cachedMeta,i=e.iScale,s=[];let r,o;for(r=0,o=e.data.length;r<o;++r)s.push(i.getPixelForValue(this.getParsed(r)[i.axis],r));const a=t.barThickness;return{min:a||eS(e),pixels:s,start:i._startPixel,end:i._endPixel,stackCount:this._getStackCount(),scale:i,grouped:t.grouped,ratio:a?1:t.categoryPercentage*t.barPercentage}}_calculateBarValuePixels(t){const{_cachedMeta:{vScale:e,_stacked:i,index:s},options:{base:r,minBarLength:o}}=this,a=r||0,l=this.getParsed(t),c=l._custom,h=pl(c);let u=l[e.axis],d=0,f=i?this.applyStack(e,l,i):u,g,_;f!==u&&(d=f-u,f=u),h&&(u=c.barStart,f=c.barEnd-c.barStart,u!==0&&Rn(u)!==Rn(c.barEnd)&&(d=0),d+=u);const m=!Zt(r)&&!h?r:d;let p=e.getPixelForValue(m);if(this.chart.getDataVisibility(t)?g=e.getPixelForValue(d+f):g=p,_=g-p,Math.abs(_)<o){_=rS(_,e,a)*o,u===a&&(p-=_/2);const v=e.getPixelForDecimal(0),y=e.getPixelForDecimal(1),x=Math.min(v,y),A=Math.max(v,y);p=Math.max(Math.min(p,A),x),g=p+_,i&&!h&&(l._stacks[e.axis]._visualValues[s]=e.getValueForPixel(g)-e.getValueForPixel(p))}if(p===e.getPixelForValue(a)){const v=Rn(_)*e.getLineWidthForValue(a)/2;p+=v,_-=v}return{size:_,base:p,head:g,center:g+_/2}}_calculateBarIndexPixels(t,e){const i=e.scale,s=this.options,r=s.skipNull,o=Ot(s.maxBarThickness,1/0);let a,l;if(e.grouped){const c=r?this._getStackCount(t):e.stackCount,h=s.barThickness==="flex"?iS(t,e,s,c):nS(t,e,s,c),u=this._getStackIndex(this.index,this._cachedMeta.stack,r?t:void 0);a=h.start+h.chunk*u+h.chunk/2,l=Math.min(o,h.chunk*h.ratio)}else a=i.getPixelForValue(this.getParsed(t)[i.axis],t),l=Math.min(o,e.min*e.ratio);return{base:a-l/2,head:a+l/2,center:a,size:l}}draw(){const t=this._cachedMeta,e=t.vScale,i=t.data,s=i.length;let r=0;for(;r<s;++r)this.getParsed(r)[e.axis]!==null&&!i[r].hidden&&i[r].draw(this._ctx)}}st(Ho,"id","bar"),st(Ho,"defaults",{datasetElementType:!1,dataElementType:"bar",categoryPercentage:.8,barPercentage:.9,grouped:!0,animations:{numbers:{type:"number",properties:["x","y","base","width","height"]}}}),st(Ho,"overrides",{scales:{_index_:{type:"category",offset:!0,grid:{offset:!0}},_value_:{type:"linear",beginAtZero:!0}}});class Vo extends vn{initialize(){this.enableOptionSharing=!0,super.initialize()}parsePrimitiveData(t,e,i,s){const r=super.parsePrimitiveData(t,e,i,s);for(let o=0;o<r.length;o++)r[o]._custom=this.resolveDataElementOptions(o+i).radius;return r}parseArrayData(t,e,i,s){const r=super.parseArrayData(t,e,i,s);for(let o=0;o<r.length;o++){const a=e[i+o];r[o]._custom=Ot(a[2],this.resolveDataElementOptions(o+i).radius)}return r}parseObjectData(t,e,i,s){const r=super.parseObjectData(t,e,i,s);for(let o=0;o<r.length;o++){const a=e[i+o];r[o]._custom=Ot(a&&a.r&&+a.r,this.resolveDataElementOptions(o+i).radius)}return r}getMaxOverflow(){const t=this._cachedMeta.data;let e=0;for(let i=t.length-1;i>=0;--i)e=Math.max(e,t[i].size(this.resolveDataElementOptions(i))/2);return e>0&&e}getLabelAndValue(t){const e=this._cachedMeta,i=this.chart.data.labels||[],{xScale:s,yScale:r}=e,o=this.getParsed(t),a=s.getLabelForValue(o.x),l=r.getLabelForValue(o.y),c=o._custom;return{label:i[t]||"",value:"("+a+", "+l+(c?", "+c:"")+")"}}update(t){const e=this._cachedMeta.data;this.updateElements(e,0,e.length,t)}updateElements(t,e,i,s){const r=s==="reset",{iScale:o,vScale:a}=this._cachedMeta,{sharedOptions:l,includeOptions:c}=this._getSharedOptions(e,s),h=o.axis,u=a.axis;for(let d=e;d<e+i;d++){const f=t[d],g=!r&&this.getParsed(d),_={},m=_[h]=r?o.getPixelForDecimal(.5):o.getPixelForValue(g[h]),p=_[u]=r?a.getBasePixel():a.getPixelForValue(g[u]);_.skip=isNaN(m)||isNaN(p),c&&(_.options=l||this.resolveDataElementOptions(d,f.active?"active":s),r&&(_.options.radius=0)),this.updateElement(f,d,_,s)}}resolveDataElementOptions(t,e){const i=this.getParsed(t);let s=super.resolveDataElementOptions(t,e);s.$shared&&(s=Object.assign({},s,{$shared:!1}));const r=s.radius;return e!=="active"&&(s.radius=0),s.radius+=Ot(i&&i._custom,r),s}}st(Vo,"id","bubble"),st(Vo,"defaults",{datasetElementType:!1,dataElementType:"point",animations:{numbers:{type:"number",properties:["x","y","borderWidth","radius"]}}}),st(Vo,"overrides",{scales:{x:{type:"linear"},y:{type:"linear"}}});function hS(n,t,e){let i=1,s=1,r=0,o=0;if(t<fe){const a=n,l=a+t,c=Math.cos(a),h=Math.sin(a),u=Math.cos(l),d=Math.sin(l),f=(y,x,A)=>Tr(y,a,l,!0)?1:Math.max(x,x*e,A,A*e),g=(y,x,A)=>Tr(y,a,l,!0)?-1:Math.min(x,x*e,A,A*e),_=f(0,c,u),m=f(we,h,d),p=g(pe,c,u),v=g(pe+we,h,d);i=(_-p)/2,s=(m-v)/2,r=-(_+p)/2,o=-(m+v)/2}return{ratioX:i,ratioY:s,offsetX:r,offsetY:o}}class Hi extends vn{constructor(t,e){super(t,e),this.enableOptionSharing=!0,this.innerRadius=void 0,this.outerRadius=void 0,this.offsetX=void 0,this.offsetY=void 0}linkScales(){}parse(t,e){const i=this.getDataset().data,s=this._cachedMeta;if(this._parsing===!1)s._parsed=i;else{let r=l=>+i[l];if(Gt(i[t])){const{key:l="value"}=this._parsing;r=c=>+vi(i[c],l)}let o,a;for(o=t,a=t+e;o<a;++o)s._parsed[o]=r(o)}}_getRotation(){return gn(this.options.rotation-90)}_getCircumference(){return gn(this.options.circumference)}_getRotationExtents(){let t=fe,e=-fe;for(let i=0;i<this.chart.data.datasets.length;++i)if(this.chart.isDatasetVisible(i)&&this.chart.getDatasetMeta(i).type===this._type){const s=this.chart.getDatasetMeta(i).controller,r=s._getRotation(),o=s._getCircumference();t=Math.min(t,r),e=Math.max(e,r+o)}return{rotation:t,circumference:e-t}}update(t){const e=this.chart,{chartArea:i}=e,s=this._cachedMeta,r=s.data,o=this.getMaxBorderWidth()+this.getMaxOffset(r)+this.options.spacing,a=Math.max((Math.min(i.width,i.height)-o)/2,0),l=Math.min(Ey(this.options.cutout,a),1),c=this._getRingWeight(this.index),{circumference:h,rotation:u}=this._getRotationExtents(),{ratioX:d,ratioY:f,offsetX:g,offsetY:_}=hS(u,h,l),m=(i.width-o)/d,p=(i.height-o)/f,v=Math.max(Math.min(m,p)/2,0),y=ip(this.options.radius,v),x=Math.max(y*l,0),A=(y-x)/this._getVisibleDatasetWeightTotal();this.offsetX=g*y,this.offsetY=_*y,s.total=this.calculateTotal(),this.outerRadius=y-A*this._getRingWeightOffset(this.index),this.innerRadius=Math.max(this.outerRadius-A*c,0),this.updateElements(r,0,r.length,t)}_circumference(t,e){const i=this.options,s=this._cachedMeta,r=this._getCircumference();return e&&i.animation.animateRotate||!this.chart.getDataVisibility(t)||s._parsed[t]===null||s.data[t].hidden?0:this.calculateCircumference(s._parsed[t]*r/fe)}updateElements(t,e,i,s){const r=s==="reset",o=this.chart,a=o.chartArea,c=o.options.animation,h=(a.left+a.right)/2,u=(a.top+a.bottom)/2,d=r&&c.animateScale,f=d?0:this.innerRadius,g=d?0:this.outerRadius,{sharedOptions:_,includeOptions:m}=this._getSharedOptions(e,s);let p=this._getRotation(),v;for(v=0;v<e;++v)p+=this._circumference(v,r);for(v=e;v<e+i;++v){const y=this._circumference(v,r),x=t[v],A={x:h+this.offsetX,y:u+this.offsetY,startAngle:p,endAngle:p+y,circumference:y,outerRadius:g,innerRadius:f};m&&(A.options=_||this.resolveDataElementOptions(v,x.active?"active":s)),p+=y,this.updateElement(x,v,A,s)}}calculateTotal(){const t=this._cachedMeta,e=t.data;let i=0,s;for(s=0;s<e.length;s++){const r=t._parsed[s];r!==null&&!isNaN(r)&&this.chart.getDataVisibility(s)&&!e[s].hidden&&(i+=Math.abs(r))}return i}calculateCircumference(t){const e=this._cachedMeta.total;return e>0&&!isNaN(t)?fe*(Math.abs(t)/e):0}getLabelAndValue(t){const e=this._cachedMeta,i=this.chart,s=i.data.labels||[],r=kr(e._parsed[t],i.options.locale);return{label:s[t]||"",value:r}}getMaxBorderWidth(t){let e=0;const i=this.chart;let s,r,o,a,l;if(!t){for(s=0,r=i.data.datasets.length;s<r;++s)if(i.isDatasetVisible(s)){o=i.getDatasetMeta(s),t=o.data,a=o.controller;break}}if(!t)return 0;for(s=0,r=t.length;s<r;++s)l=a.resolveDataElementOptions(s),l.borderAlign!=="inner"&&(e=Math.max(e,l.borderWidth||0,l.hoverBorderWidth||0));return e}getMaxOffset(t){let e=0;for(let i=0,s=t.length;i<s;++i){const r=this.resolveDataElementOptions(i);e=Math.max(e,r.offset||0,r.hoverOffset||0)}return e}_getRingWeightOffset(t){let e=0;for(let i=0;i<t;++i)this.chart.isDatasetVisible(i)&&(e+=this._getRingWeight(i));return e}_getRingWeight(t){return Math.max(Ot(this.chart.data.datasets[t].weight,1),0)}_getVisibleDatasetWeightTotal(){return this._getRingWeightOffset(this.chart.data.datasets.length)||1}}st(Hi,"id","doughnut"),st(Hi,"defaults",{datasetElementType:!1,dataElementType:"arc",animation:{animateRotate:!0,animateScale:!1},animations:{numbers:{type:"number",properties:["circumference","endAngle","innerRadius","outerRadius","startAngle","x","y","offset","borderWidth","spacing"]}},cutout:"50%",rotation:0,circumference:360,radius:"100%",spacing:0,indexAxis:"r"}),st(Hi,"descriptors",{_scriptable:t=>t!=="spacing",_indexable:t=>t!=="spacing"&&!t.startsWith("borderDash")&&!t.startsWith("hoverBorderDash")}),st(Hi,"overrides",{aspectRatio:1,plugins:{legend:{labels:{generateLabels(t){const e=t.data;if(e.labels.length&&e.datasets.length){const{labels:{pointStyle:i,color:s}}=t.legend.options;return e.labels.map((r,o)=>{const l=t.getDatasetMeta(0).controller.getStyle(o);return{text:r,fillStyle:l.backgroundColor,strokeStyle:l.borderColor,fontColor:s,lineWidth:l.borderWidth,pointStyle:i,hidden:!t.getDataVisibility(o),index:o}})}return[]}},onClick(t,e,i){i.chart.toggleDataVisibility(e.index),i.chart.update()}}}});class Go extends vn{initialize(){this.enableOptionSharing=!0,this.supportsDecimation=!0,super.initialize()}update(t){const e=this._cachedMeta,{dataset:i,data:s=[],_dataset:r}=e,o=this.chart._animationsDisabled;let{start:a,count:l}=up(e,s,o);this._drawStart=a,this._drawCount=l,dp(e)&&(a=0,l=s.length),i._chart=this.chart,i._datasetIndex=this.index,i._decimated=!!r._decimated,i.points=s;const c=this.resolveDatasetElementOptions(t);this.options.showLine||(c.borderWidth=0),c.segment=this.options.segment,this.updateElement(i,void 0,{animated:!o,options:c},t),this.updateElements(s,a,l,t)}updateElements(t,e,i,s){const r=s==="reset",{iScale:o,vScale:a,_stacked:l,_dataset:c}=this._cachedMeta,{sharedOptions:h,includeOptions:u}=this._getSharedOptions(e,s),d=o.axis,f=a.axis,{spanGaps:g,segment:_}=this.options,m=Fs(g)?g:Number.POSITIVE_INFINITY,p=this.chart._animationsDisabled||r||s==="none",v=e+i,y=t.length;let x=e>0&&this.getParsed(e-1);for(let A=0;A<y;++A){const w=t[A],T=p?w:{};if(A<e||A>=v){T.skip=!0;continue}const R=this.getParsed(A),S=Zt(R[f]),b=T[d]=o.getPixelForValue(R[d],A),P=T[f]=r||S?a.getBasePixel():a.getPixelForValue(l?this.applyStack(a,R,l):R[f],A);T.skip=isNaN(b)||isNaN(P)||S,T.stop=A>0&&Math.abs(R[d]-x[d])>m,_&&(T.parsed=R,T.raw=c.data[A]),u&&(T.options=h||this.resolveDataElementOptions(A,w.active?"active":s)),p||this.updateElement(w,A,T,s),x=R}}getMaxOverflow(){const t=this._cachedMeta,e=t.dataset,i=e.options&&e.options.borderWidth||0,s=t.data||[];if(!s.length)return i;const r=s[0].size(this.resolveDataElementOptions(0)),o=s[s.length-1].size(this.resolveDataElementOptions(s.length-1));return Math.max(i,r,o)/2}draw(){const t=this._cachedMeta;t.dataset.updateControlPoints(this.chart.chartArea,t.iScale.axis),super.draw()}}st(Go,"id","line"),st(Go,"defaults",{datasetElementType:"line",dataElementType:"point",showLine:!0,spanGaps:!1}),st(Go,"overrides",{scales:{_index_:{type:"category"},_value_:{type:"linear"}}});class vr extends vn{constructor(t,e){super(t,e),this.innerRadius=void 0,this.outerRadius=void 0}getLabelAndValue(t){const e=this._cachedMeta,i=this.chart,s=i.data.labels||[],r=kr(e._parsed[t].r,i.options.locale);return{label:s[t]||"",value:r}}parseObjectData(t,e,i,s){return bp.bind(this)(t,e,i,s)}update(t){const e=this._cachedMeta.data;this._updateRadius(),this.updateElements(e,0,e.length,t)}getMinMax(){const t=this._cachedMeta,e={min:Number.POSITIVE_INFINITY,max:Number.NEGATIVE_INFINITY};return t.data.forEach((i,s)=>{const r=this.getParsed(s).r;!isNaN(r)&&this.chart.getDataVisibility(s)&&(r<e.min&&(e.min=r),r>e.max&&(e.max=r))}),e}_updateRadius(){const t=this.chart,e=t.chartArea,i=t.options,s=Math.min(e.right-e.left,e.bottom-e.top),r=Math.max(s/2,0),o=Math.max(i.cutoutPercentage?r/100*i.cutoutPercentage:1,0),a=(r-o)/t.getVisibleDatasetCount();this.outerRadius=r-a*this.index,this.innerRadius=this.outerRadius-a}updateElements(t,e,i,s){const r=s==="reset",o=this.chart,l=o.options.animation,c=this._cachedMeta.rScale,h=c.xCenter,u=c.yCenter,d=c.getIndexAngle(0)-.5*pe;let f=d,g;const _=360/this.countVisibleElements();for(g=0;g<e;++g)f+=this._computeAngle(g,s,_);for(g=e;g<e+i;g++){const m=t[g];let p=f,v=f+this._computeAngle(g,s,_),y=o.getDataVisibility(g)?c.getDistanceFromCenterForValue(this.getParsed(g).r):0;f=v,r&&(l.animateScale&&(y=0),l.animateRotate&&(p=v=d));const x={x:h,y:u,innerRadius:0,outerRadius:y,startAngle:p,endAngle:v,options:this.resolveDataElementOptions(g,m.active?"active":s)};this.updateElement(m,g,x,s)}}countVisibleElements(){const t=this._cachedMeta;let e=0;return t.data.forEach((i,s)=>{!isNaN(this.getParsed(s).r)&&this.chart.getDataVisibility(s)&&e++}),e}_computeAngle(t,e,i){return this.chart.getDataVisibility(t)?gn(this.resolveDataElementOptions(t,e).angle||i):0}}st(vr,"id","polarArea"),st(vr,"defaults",{dataElementType:"arc",animation:{animateRotate:!0,animateScale:!0},animations:{numbers:{type:"number",properties:["x","y","startAngle","endAngle","innerRadius","outerRadius"]}},indexAxis:"r",startAngle:0}),st(vr,"overrides",{aspectRatio:1,plugins:{legend:{labels:{generateLabels(t){const e=t.data;if(e.labels.length&&e.datasets.length){const{labels:{pointStyle:i,color:s}}=t.legend.options;return e.labels.map((r,o)=>{const l=t.getDatasetMeta(0).controller.getStyle(o);return{text:r,fillStyle:l.backgroundColor,strokeStyle:l.borderColor,fontColor:s,lineWidth:l.borderWidth,pointStyle:i,hidden:!t.getDataVisibility(o),index:o}})}return[]}},onClick(t,e,i){i.chart.toggleDataVisibility(e.index),i.chart.update()}}},scales:{r:{type:"radialLinear",angleLines:{display:!1},beginAtZero:!0,grid:{circular:!0},pointLabels:{display:!1},startAngle:0}}});class _c extends Hi{}st(_c,"id","pie"),st(_c,"defaults",{cutout:0,rotation:0,circumference:360,radius:"100%"});class Wo extends vn{getLabelAndValue(t){const e=this._cachedMeta.vScale,i=this.getParsed(t);return{label:e.getLabels()[t],value:""+e.getLabelForValue(i[e.axis])}}parseObjectData(t,e,i,s){return bp.bind(this)(t,e,i,s)}update(t){const e=this._cachedMeta,i=e.dataset,s=e.data||[],r=e.iScale.getLabels();if(i.points=s,t!=="resize"){const o=this.resolveDatasetElementOptions(t);this.options.showLine||(o.borderWidth=0);const a={_loop:!0,_fullLoop:r.length===s.length,options:o};this.updateElement(i,void 0,a,t)}this.updateElements(s,0,s.length,t)}updateElements(t,e,i,s){const r=this._cachedMeta.rScale,o=s==="reset";for(let a=e;a<e+i;a++){const l=t[a],c=this.resolveDataElementOptions(a,l.active?"active":s),h=r.getPointPositionForValue(a,this.getParsed(a).r),u=o?r.xCenter:h.x,d=o?r.yCenter:h.y,f={x:u,y:d,angle:h.angle,skip:isNaN(u)||isNaN(d),options:c};this.updateElement(l,a,f,s)}}}st(Wo,"id","radar"),st(Wo,"defaults",{datasetElementType:"line",dataElementType:"point",indexAxis:"r",showLine:!0,elements:{line:{fill:"start"}}}),st(Wo,"overrides",{aspectRatio:1,scales:{r:{type:"radialLinear"}}});class Xo extends vn{getLabelAndValue(t){const e=this._cachedMeta,i=this.chart.data.labels||[],{xScale:s,yScale:r}=e,o=this.getParsed(t),a=s.getLabelForValue(o.x),l=r.getLabelForValue(o.y);return{label:i[t]||"",value:"("+a+", "+l+")"}}update(t){const e=this._cachedMeta,{data:i=[]}=e,s=this.chart._animationsDisabled;let{start:r,count:o}=up(e,i,s);if(this._drawStart=r,this._drawCount=o,dp(e)&&(r=0,o=i.length),this.options.showLine){this.datasetElementType||this.addElements();const{dataset:a,_dataset:l}=e;a._chart=this.chart,a._datasetIndex=this.index,a._decimated=!!l._decimated,a.points=i;const c=this.resolveDatasetElementOptions(t);c.segment=this.options.segment,this.updateElement(a,void 0,{animated:!s,options:c},t)}else this.datasetElementType&&(delete e.dataset,this.datasetElementType=!1);this.updateElements(i,r,o,t)}addElements(){const{showLine:t}=this.options;!this.datasetElementType&&t&&(this.datasetElementType=this.chart.registry.getElement("line")),super.addElements()}updateElements(t,e,i,s){const r=s==="reset",{iScale:o,vScale:a,_stacked:l,_dataset:c}=this._cachedMeta,h=this.resolveDataElementOptions(e,s),u=this.getSharedOptions(h),d=this.includeOptions(s,u),f=o.axis,g=a.axis,{spanGaps:_,segment:m}=this.options,p=Fs(_)?_:Number.POSITIVE_INFINITY,v=this.chart._animationsDisabled||r||s==="none";let y=e>0&&this.getParsed(e-1);for(let x=e;x<e+i;++x){const A=t[x],w=this.getParsed(x),T=v?A:{},R=Zt(w[g]),S=T[f]=o.getPixelForValue(w[f],x),b=T[g]=r||R?a.getBasePixel():a.getPixelForValue(l?this.applyStack(a,w,l):w[g],x);T.skip=isNaN(S)||isNaN(b)||R,T.stop=x>0&&Math.abs(w[f]-y[f])>p,m&&(T.parsed=w,T.raw=c.data[x]),d&&(T.options=u||this.resolveDataElementOptions(x,A.active?"active":s)),v||this.updateElement(A,x,T,s),y=w}this.updateSharedOptions(u,s,h)}getMaxOverflow(){const t=this._cachedMeta,e=t.data||[];if(!this.options.showLine){let a=0;for(let l=e.length-1;l>=0;--l)a=Math.max(a,e[l].size(this.resolveDataElementOptions(l))/2);return a>0&&a}const i=t.dataset,s=i.options&&i.options.borderWidth||0;if(!e.length)return s;const r=e[0].size(this.resolveDataElementOptions(0)),o=e[e.length-1].size(this.resolveDataElementOptions(e.length-1));return Math.max(s,r,o)/2}}st(Xo,"id","scatter"),st(Xo,"defaults",{datasetElementType:!1,dataElementType:"point",showLine:!1,fill:!1}),st(Xo,"overrides",{interaction:{mode:"point"},scales:{x:{type:"linear"},y:{type:"linear"}}});var uS=Object.freeze({__proto__:null,BarController:Ho,BubbleController:Vo,DoughnutController:Hi,LineController:Go,PieController:_c,PolarAreaController:vr,RadarController:Wo,ScatterController:Xo});function Li(){throw new Error("This method is not implemented: Check that a complete date adapter is provided.")}class sh{constructor(t){st(this,"options");this.options=t||{}}static override(t){Object.assign(sh.prototype,t)}init(){}formats(){return Li()}parse(){return Li()}format(){return Li()}add(){return Li()}diff(){return Li()}startOf(){return Li()}endOf(){return Li()}}var dS={_date:sh};function fS(n,t,e,i){const{controller:s,data:r,_sorted:o}=n,a=s._cachedMeta.iScale;if(a&&t===a.axis&&t!=="r"&&o&&r.length){const l=a._reversePixels?Ny:qn;if(i){if(s._sharedOptions){const c=r[0],h=typeof c.getRange=="function"&&c.getRange(t);if(h){const u=l(r,t,e-h),d=l(r,t,e+h);return{lo:u.lo,hi:d.hi}}}}else return l(r,t,e)}return{lo:0,hi:r.length-1}}function xa(n,t,e,i,s){const r=n.getSortedVisibleDatasetMetas(),o=e[t];for(let a=0,l=r.length;a<l;++a){const{index:c,data:h}=r[a],{lo:u,hi:d}=fS(r[a],t,o,s);for(let f=u;f<=d;++f){const g=h[f];g.skip||i(g,c,f)}}}function pS(n){const t=n.indexOf("x")!==-1,e=n.indexOf("y")!==-1;return function(i,s){const r=t?Math.abs(i.x-s.x):0,o=e?Math.abs(i.y-s.y):0;return Math.sqrt(Math.pow(r,2)+Math.pow(o,2))}}function ml(n,t,e,i,s){const r=[];return!s&&!n.isPointInArea(t)||xa(n,e,t,function(a,l,c){!s&&!Kn(a,n.chartArea,0)||a.inRange(t.x,t.y,i)&&r.push({element:a,datasetIndex:l,index:c})},!0),r}function mS(n,t,e,i){let s=[];function r(o,a,l){const{startAngle:c,endAngle:h}=o.getProps(["startAngle","endAngle"],i),{angle:u}=op(o,{x:t.x,y:t.y});Tr(u,c,h)&&s.push({element:o,datasetIndex:a,index:l})}return xa(n,e,t,r),s}function gS(n,t,e,i,s,r){let o=[];const a=pS(e);let l=Number.POSITIVE_INFINITY;function c(h,u,d){const f=h.inRange(t.x,t.y,s);if(i&&!f)return;const g=h.getCenterPoint(s);if(!(!!r||n.isPointInArea(g))&&!f)return;const m=a(t,g);m<l?(o=[{element:h,datasetIndex:u,index:d}],l=m):m===l&&o.push({element:h,datasetIndex:u,index:d})}return xa(n,e,t,c),o}function gl(n,t,e,i,s,r){return!r&&!n.isPointInArea(t)?[]:e==="r"&&!i?mS(n,t,e,s):gS(n,t,e,i,s,r)}function fd(n,t,e,i,s){const r=[],o=e==="x"?"inXRange":"inYRange";let a=!1;return xa(n,e,t,(l,c,h)=>{l[o]&&l[o](t[e],s)&&(r.push({element:l,datasetIndex:c,index:h}),a=a||l.inRange(t.x,t.y,s))}),i&&!a?[]:r}var _S={modes:{index(n,t,e,i){const s=Ui(t,n),r=e.axis||"x",o=e.includeInvisible||!1,a=e.intersect?ml(n,s,r,i,o):gl(n,s,r,!1,i,o),l=[];return a.length?(n.getSortedVisibleDatasetMetas().forEach(c=>{const h=a[0].index,u=c.data[h];u&&!u.skip&&l.push({element:u,datasetIndex:c.index,index:h})}),l):[]},dataset(n,t,e,i){const s=Ui(t,n),r=e.axis||"xy",o=e.includeInvisible||!1;let a=e.intersect?ml(n,s,r,i,o):gl(n,s,r,!1,i,o);if(a.length>0){const l=a[0].datasetIndex,c=n.getDatasetMeta(l).data;a=[];for(let h=0;h<c.length;++h)a.push({element:c[h],datasetIndex:l,index:h})}return a},point(n,t,e,i){const s=Ui(t,n),r=e.axis||"xy",o=e.includeInvisible||!1;return ml(n,s,r,i,o)},nearest(n,t,e,i){const s=Ui(t,n),r=e.axis||"xy",o=e.includeInvisible||!1;return gl(n,s,r,e.intersect,i,o)},x(n,t,e,i){const s=Ui(t,n);return fd(n,s,"x",e.intersect,i)},y(n,t,e,i){const s=Ui(t,n);return fd(n,s,"y",e.intersect,i)}}};const Pp=["left","top","right","bottom"];function er(n,t){return n.filter(e=>e.pos===t)}function pd(n,t){return n.filter(e=>Pp.indexOf(e.pos)===-1&&e.box.axis===t)}function nr(n,t){return n.sort((e,i)=>{const s=t?i:e,r=t?e:i;return s.weight===r.weight?s.index-r.index:s.weight-r.weight})}function xS(n){const t=[];let e,i,s,r,o,a;for(e=0,i=(n||[]).length;e<i;++e)s=n[e],{position:r,options:{stack:o,stackWeight:a=1}}=s,t.push({index:e,box:s,pos:r,horizontal:s.isHorizontal(),weight:s.weight,stack:o&&r+o,stackWeight:a});return t}function vS(n){const t={};for(const e of n){const{stack:i,pos:s,stackWeight:r}=e;if(!i||!Pp.includes(s))continue;const o=t[i]||(t[i]={count:0,placed:0,weight:0,size:0});o.count++,o.weight+=r}return t}function bS(n,t){const e=vS(n),{vBoxMaxWidth:i,hBoxMaxHeight:s}=t;let r,o,a;for(r=0,o=n.length;r<o;++r){a=n[r];const{fullSize:l}=a.box,c=e[a.stack],h=c&&a.stackWeight/c.weight;a.horizontal?(a.width=h?h*i:l&&t.availableWidth,a.height=s):(a.width=i,a.height=h?h*s:l&&t.availableHeight)}return e}function yS(n){const t=xS(n),e=nr(t.filter(c=>c.box.fullSize),!0),i=nr(er(t,"left"),!0),s=nr(er(t,"right")),r=nr(er(t,"top"),!0),o=nr(er(t,"bottom")),a=pd(t,"x"),l=pd(t,"y");return{fullSize:e,leftAndTop:i.concat(r),rightAndBottom:s.concat(l).concat(o).concat(a),chartArea:er(t,"chartArea"),vertical:i.concat(s).concat(l),horizontal:r.concat(o).concat(a)}}function md(n,t,e,i){return Math.max(n[e],t[e])+Math.max(n[i],t[i])}function Dp(n,t){n.top=Math.max(n.top,t.top),n.left=Math.max(n.left,t.left),n.bottom=Math.max(n.bottom,t.bottom),n.right=Math.max(n.right,t.right)}function MS(n,t,e,i){const{pos:s,box:r}=e,o=n.maxPadding;if(!Gt(s)){e.size&&(n[s]-=e.size);const u=i[e.stack]||{size:0,count:1};u.size=Math.max(u.size,e.horizontal?r.height:r.width),e.size=u.size/u.count,n[s]+=e.size}r.getPadding&&Dp(o,r.getPadding());const a=Math.max(0,t.outerWidth-md(o,n,"left","right")),l=Math.max(0,t.outerHeight-md(o,n,"top","bottom")),c=a!==n.w,h=l!==n.h;return n.w=a,n.h=l,e.horizontal?{same:c,other:h}:{same:h,other:c}}function SS(n){const t=n.maxPadding;function e(i){const s=Math.max(t[i]-n[i],0);return n[i]+=s,s}n.y+=e("top"),n.x+=e("left"),e("right"),e("bottom")}function ES(n,t){const e=t.maxPadding;function i(s){const r={left:0,top:0,right:0,bottom:0};return s.forEach(o=>{r[o]=Math.max(t[o],e[o])}),r}return i(n?["left","right"]:["top","bottom"])}function hr(n,t,e,i){const s=[];let r,o,a,l,c,h;for(r=0,o=n.length,c=0;r<o;++r){a=n[r],l=a.box,l.update(a.width||t.w,a.height||t.h,ES(a.horizontal,t));const{same:u,other:d}=MS(t,e,a,i);c|=u&&s.length,h=h||d,l.fullSize||s.push(a)}return c&&hr(s,t,e,i)||h}function To(n,t,e,i,s){n.top=e,n.left=t,n.right=t+i,n.bottom=e+s,n.width=i,n.height=s}function gd(n,t,e,i){const s=e.padding;let{x:r,y:o}=t;for(const a of n){const l=a.box,c=i[a.stack]||{placed:0,weight:1},h=a.stackWeight/c.weight||1;if(a.horizontal){const u=t.w*h,d=c.size||l.height;Er(c.start)&&(o=c.start),l.fullSize?To(l,s.left,o,e.outerWidth-s.right-s.left,d):To(l,t.left+c.placed,o,u,d),c.start=o,c.placed+=u,o=l.bottom}else{const u=t.h*h,d=c.size||l.width;Er(c.start)&&(r=c.start),l.fullSize?To(l,r,s.top,d,e.outerHeight-s.bottom-s.top):To(l,r,t.top+c.placed,d,u),c.start=r,c.placed+=u,r=l.right}}t.x=r,t.y=o}var Be={addBox(n,t){n.boxes||(n.boxes=[]),t.fullSize=t.fullSize||!1,t.position=t.position||"top",t.weight=t.weight||0,t._layers=t._layers||function(){return[{z:0,draw(e){t.draw(e)}}]},n.boxes.push(t)},removeBox(n,t){const e=n.boxes?n.boxes.indexOf(t):-1;e!==-1&&n.boxes.splice(e,1)},configure(n,t,e){t.fullSize=e.fullSize,t.position=e.position,t.weight=e.weight},update(n,t,e,i){if(!n)return;const s=ze(n.options.layout.padding),r=Math.max(t-s.width,0),o=Math.max(e-s.height,0),a=yS(n.boxes),l=a.vertical,c=a.horizontal;ne(n.boxes,_=>{typeof _.beforeLayout=="function"&&_.beforeLayout()});const h=l.reduce((_,m)=>m.box.options&&m.box.options.display===!1?_:_+1,0)||1,u=Object.freeze({outerWidth:t,outerHeight:e,padding:s,availableWidth:r,availableHeight:o,vBoxMaxWidth:r/2/h,hBoxMaxHeight:o/2}),d=Object.assign({},s);Dp(d,ze(i));const f=Object.assign({maxPadding:d,w:r,h:o,x:s.left,y:s.top},s),g=bS(l.concat(c),u);hr(a.fullSize,f,u,g),hr(l,f,u,g),hr(c,f,u,g)&&hr(l,f,u,g),SS(f),gd(a.leftAndTop,f,u,g),f.x+=f.w,f.y+=f.h,gd(a.rightAndBottom,f,u,g),n.chartArea={left:f.left,top:f.top,right:f.left+f.w,bottom:f.top+f.h,height:f.h,width:f.w},ne(a.chartArea,_=>{const m=_.box;Object.assign(m,n.chartArea),m.update(f.w,f.h,{left:0,top:0,right:0,bottom:0})})}};class Lp{acquireContext(t,e){}releaseContext(t){return!1}addEventListener(t,e,i){}removeEventListener(t,e,i){}getDevicePixelRatio(){return 1}getMaximumSize(t,e,i,s){return e=Math.max(0,e||t.width),i=i||t.height,{width:e,height:Math.max(0,s?Math.floor(e/s):i)}}isAttached(t){return!0}updateConfig(t){}}class TS extends Lp{acquireContext(t){return t&&t.getContext&&t.getContext("2d")||null}updateConfig(t){t.options.animation=!1}}const jo="$chartjs",wS={touchstart:"mousedown",touchmove:"mousemove",touchend:"mouseup",pointerenter:"mouseenter",pointerdown:"mousedown",pointermove:"mousemove",pointerup:"mouseup",pointerleave:"mouseout",pointerout:"mouseout"},_d=n=>n===null||n==="";function AS(n,t){const e=n.style,i=n.getAttribute("height"),s=n.getAttribute("width");if(n[jo]={initial:{height:i,width:s,style:{display:e.display,height:e.height,width:e.width}}},e.display=e.display||"block",e.boxSizing=e.boxSizing||"border-box",_d(s)){const r=td(n,"width");r!==void 0&&(n.width=r)}if(_d(i))if(n.style.height==="")n.height=n.width/(t||2);else{const r=td(n,"height");r!==void 0&&(n.height=r)}return n}const Ip=RM?{passive:!0}:!1;function CS(n,t,e){n&&n.addEventListener(t,e,Ip)}function RS(n,t,e){n&&n.canvas&&n.canvas.removeEventListener(t,e,Ip)}function PS(n,t){const e=wS[n.type]||n.type,{x:i,y:s}=Ui(n,t);return{type:e,chart:t,native:n,x:i!==void 0?i:null,y:s!==void 0?s:null}}function aa(n,t){for(const e of n)if(e===t||e.contains(t))return!0}function DS(n,t,e){const i=n.canvas,s=new MutationObserver(r=>{let o=!1;for(const a of r)o=o||aa(a.addedNodes,i),o=o&&!aa(a.removedNodes,i);o&&e()});return s.observe(document,{childList:!0,subtree:!0}),s}function LS(n,t,e){const i=n.canvas,s=new MutationObserver(r=>{let o=!1;for(const a of r)o=o||aa(a.removedNodes,i),o=o&&!aa(a.addedNodes,i);o&&e()});return s.observe(document,{childList:!0,subtree:!0}),s}const Ar=new Map;let xd=0;function Op(){const n=window.devicePixelRatio;n!==xd&&(xd=n,Ar.forEach((t,e)=>{e.currentDevicePixelRatio!==n&&t()}))}function IS(n,t){Ar.size||window.addEventListener("resize",Op),Ar.set(n,t)}function OS(n){Ar.delete(n),Ar.size||window.removeEventListener("resize",Op)}function US(n,t,e){const i=n.canvas,s=i&&ih(i);if(!s)return;const r=hp((a,l)=>{const c=s.clientWidth;e(a,l),c<s.clientWidth&&e()},window),o=new ResizeObserver(a=>{const l=a[0],c=l.contentRect.width,h=l.contentRect.height;c===0&&h===0||r(c,h)});return o.observe(s),IS(n,r),o}function _l(n,t,e){e&&e.disconnect(),t==="resize"&&OS(n)}function NS(n,t,e){const i=n.canvas,s=hp(r=>{n.ctx!==null&&e(PS(r,n))},n);return CS(i,t,s),s}class FS extends Lp{acquireContext(t,e){const i=t&&t.getContext&&t.getContext("2d");return i&&i.canvas===t?(AS(t,e),i):null}releaseContext(t){const e=t.canvas;if(!e[jo])return!1;const i=e[jo].initial;["height","width"].forEach(r=>{const o=i[r];Zt(o)?e.removeAttribute(r):e.setAttribute(r,o)});const s=i.style||{};return Object.keys(s).forEach(r=>{e.style[r]=s[r]}),e.width=e.width,delete e[jo],!0}addEventListener(t,e,i){this.removeEventListener(t,e);const s=t.$proxies||(t.$proxies={}),o={attach:DS,detach:LS,resize:US}[e]||NS;s[e]=o(t,e,i)}removeEventListener(t,e){const i=t.$proxies||(t.$proxies={}),s=i[e];if(!s)return;({attach:_l,detach:_l,resize:_l}[e]||RS)(t,e,s),i[e]=void 0}getDevicePixelRatio(){return window.devicePixelRatio}getMaximumSize(t,e,i,s){return CM(t,e,i,s)}isAttached(t){const e=t&&ih(t);return!!(e&&e.isConnected)}}function kS(n){return!nh()||typeof OffscreenCanvas<"u"&&n instanceof OffscreenCanvas?TS:FS}class bn{constructor(){st(this,"x");st(this,"y");st(this,"active",!1);st(this,"options");st(this,"$animations")}tooltipPosition(t){const{x:e,y:i}=this.getProps(["x","y"],t);return{x:e,y:i}}hasValue(){return Fs(this.x)&&Fs(this.y)}getProps(t,e){const i=this.$animations;if(!e||!i)return this;const s={};return t.forEach(r=>{s[r]=i[r]&&i[r].active()?i[r]._to:this[r]}),s}}st(bn,"defaults",{}),st(bn,"defaultRoutes");function BS(n,t){const e=n.options.ticks,i=zS(n),s=Math.min(e.maxTicksLimit||i,i),r=e.major.enabled?VS(t):[],o=r.length,a=r[0],l=r[o-1],c=[];if(o>s)return GS(t,c,r,o/s),c;const h=HS(r,t,s);if(o>0){let u,d;const f=o>1?Math.round((l-a)/(o-1)):null;for(wo(t,c,h,Zt(f)?0:a-f,a),u=0,d=o-1;u<d;u++)wo(t,c,h,r[u],r[u+1]);return wo(t,c,h,l,Zt(f)?t.length:l+f),c}return wo(t,c,h),c}function zS(n){const t=n.options.offset,e=n._tickSize(),i=n._length/e+(t?0:1),s=n._maxLength/e;return Math.floor(Math.min(i,s))}function HS(n,t,e){const i=WS(n),s=t.length/e;if(!i)return Math.max(s,1);const r=Ly(i);for(let o=0,a=r.length-1;o<a;o++){const l=r[o];if(l>s)return l}return Math.max(s,1)}function VS(n){const t=[];let e,i;for(e=0,i=n.length;e<i;e++)n[e].major&&t.push(e);return t}function GS(n,t,e,i){let s=0,r=e[0],o;for(i=Math.ceil(i),o=0;o<n.length;o++)o===r&&(t.push(n[o]),s++,r=e[s*i])}function wo(n,t,e,i,s){const r=Ot(i,0),o=Math.min(Ot(s,n.length),n.length);let a=0,l,c,h;for(e=Math.ceil(e),s&&(l=s-i,e=l/Math.floor(l/e)),h=r;h<0;)a++,h=Math.round(r+a*e);for(c=Math.max(r,0);c<o;c++)c===h&&(t.push(n[c]),a++,h=Math.round(r+a*e))}function WS(n){const t=n.length;let e,i;if(t<2)return!1;for(i=n[0],e=1;e<t;++e)if(n[e]-n[e-1]!==i)return!1;return i}const XS=n=>n==="left"?"right":n==="right"?"left":n,vd=(n,t,e)=>t==="top"||t==="left"?n[t]+e:n[t]-e,bd=(n,t)=>Math.min(t||n,n);function yd(n,t){const e=[],i=n.length/t,s=n.length;let r=0;for(;r<s;r+=i)e.push(n[Math.floor(r)]);return e}function jS(n,t,e){const i=n.ticks.length,s=Math.min(t,i-1),r=n._startPixel,o=n._endPixel,a=1e-6;let l=n.getPixelForTick(s),c;if(!(e&&(i===1?c=Math.max(l-r,o-l):t===0?c=(n.getPixelForTick(1)-l)/2:c=(l-n.getPixelForTick(s-1))/2,l+=s<t?c:-c,l<r-a||l>o+a)))return l}function $S(n,t){ne(n,e=>{const i=e.gc,s=i.length/2;let r;if(s>t){for(r=0;r<s;++r)delete e.data[i[r]];i.splice(0,s)}})}function ir(n){return n.drawTicks?n.tickLength:0}function Md(n,t){if(!n.display)return 0;const e=Pe(n.font,t),i=ze(n.padding);return(me(n.text)?n.text.length:1)*e.lineHeight+i.height}function YS(n,t){return Mi(n,{scale:t,type:"scale"})}function qS(n,t,e){return Mi(n,{tick:e,index:t,type:"tick"})}function KS(n,t,e){let i=Kc(n);return(e&&t!=="right"||!e&&t==="right")&&(i=XS(i)),i}function ZS(n,t,e,i){const{top:s,left:r,bottom:o,right:a,chart:l}=n,{chartArea:c,scales:h}=l;let u=0,d,f,g;const _=o-s,m=a-r;if(n.isHorizontal()){if(f=ke(i,r,a),Gt(e)){const p=Object.keys(e)[0],v=e[p];g=h[p].getPixelForValue(v)+_-t}else e==="center"?g=(c.bottom+c.top)/2+_-t:g=vd(n,e,t);d=a-r}else{if(Gt(e)){const p=Object.keys(e)[0],v=e[p];f=h[p].getPixelForValue(v)-m+t}else e==="center"?f=(c.left+c.right)/2-m+t:f=vd(n,e,t);g=ke(i,o,s),u=e==="left"?-we:we}return{titleX:f,titleY:g,maxWidth:d,rotation:u}}class Zi extends bn{constructor(t){super(),this.id=t.id,this.type=t.type,this.options=void 0,this.ctx=t.ctx,this.chart=t.chart,this.top=void 0,this.bottom=void 0,this.left=void 0,this.right=void 0,this.width=void 0,this.height=void 0,this._margins={left:0,right:0,top:0,bottom:0},this.maxWidth=void 0,this.maxHeight=void 0,this.paddingTop=void 0,this.paddingBottom=void 0,this.paddingLeft=void 0,this.paddingRight=void 0,this.axis=void 0,this.labelRotation=void 0,this.min=void 0,this.max=void 0,this._range=void 0,this.ticks=[],this._gridLineItems=null,this._labelItems=null,this._labelSizes=null,this._length=0,this._maxLength=0,this._longestTextCache={},this._startPixel=void 0,this._endPixel=void 0,this._reversePixels=!1,this._userMax=void 0,this._userMin=void 0,this._suggestedMax=void 0,this._suggestedMin=void 0,this._ticksLength=0,this._borderValue=0,this._cache={},this._dataLimitsCached=!1,this.$context=void 0}init(t){this.options=t.setContext(this.getContext()),this.axis=t.axis,this._userMin=this.parse(t.min),this._userMax=this.parse(t.max),this._suggestedMin=this.parse(t.suggestedMin),this._suggestedMax=this.parse(t.suggestedMax)}parse(t,e){return t}getUserBounds(){let{_userMin:t,_userMax:e,_suggestedMin:i,_suggestedMax:s}=this;return t=Qe(t,Number.POSITIVE_INFINITY),e=Qe(e,Number.NEGATIVE_INFINITY),i=Qe(i,Number.POSITIVE_INFINITY),s=Qe(s,Number.NEGATIVE_INFINITY),{min:Qe(t,i),max:Qe(e,s),minDefined:ye(t),maxDefined:ye(e)}}getMinMax(t){let{min:e,max:i,minDefined:s,maxDefined:r}=this.getUserBounds(),o;if(s&&r)return{min:e,max:i};const a=this.getMatchingVisibleMetas();for(let l=0,c=a.length;l<c;++l)o=a[l].controller.getMinMax(this,t),s||(e=Math.min(e,o.min)),r||(i=Math.max(i,o.max));return e=r&&e>i?i:e,i=s&&e>i?e:i,{min:Qe(e,Qe(i,e)),max:Qe(i,Qe(e,i))}}getPadding(){return{left:this.paddingLeft||0,top:this.paddingTop||0,right:this.paddingRight||0,bottom:this.paddingBottom||0}}getTicks(){return this.ticks}getLabels(){const t=this.chart.data;return this.options.labels||(this.isHorizontal()?t.xLabels:t.yLabels)||t.labels||[]}getLabelItems(t=this.chart.chartArea){return this._labelItems||(this._labelItems=this._computeLabelItems(t))}beforeLayout(){this._cache={},this._dataLimitsCached=!1}beforeUpdate(){ce(this.options.beforeUpdate,[this])}update(t,e,i){const{beginAtZero:s,grace:r,ticks:o}=this.options,a=o.sampleSize;this.beforeUpdate(),this.maxWidth=t,this.maxHeight=e,this._margins=i=Object.assign({left:0,right:0,top:0,bottom:0},i),this.ticks=null,this._labelSizes=null,this._gridLineItems=null,this._labelItems=null,this.beforeSetDimensions(),this.setDimensions(),this.afterSetDimensions(),this._maxLength=this.isHorizontal()?this.width+i.left+i.right:this.height+i.top+i.bottom,this._dataLimitsCached||(this.beforeDataLimits(),this.determineDataLimits(),this.afterDataLimits(),this._range=oM(this,r,s),this._dataLimitsCached=!0),this.beforeBuildTicks(),this.ticks=this.buildTicks()||[],this.afterBuildTicks();const l=a<this.ticks.length;this._convertTicksToLabels(l?yd(this.ticks,a):this.ticks),this.configure(),this.beforeCalculateLabelRotation(),this.calculateLabelRotation(),this.afterCalculateLabelRotation(),o.display&&(o.autoSkip||o.source==="auto")&&(this.ticks=BS(this,this.ticks),this._labelSizes=null,this.afterAutoSkip()),l&&this._convertTicksToLabels(this.ticks),this.beforeFit(),this.fit(),this.afterFit(),this.afterUpdate()}configure(){let t=this.options.reverse,e,i;this.isHorizontal()?(e=this.left,i=this.right):(e=this.top,i=this.bottom,t=!t),this._startPixel=e,this._endPixel=i,this._reversePixels=t,this._length=i-e,this._alignToPixels=this.options.alignToPixels}afterUpdate(){ce(this.options.afterUpdate,[this])}beforeSetDimensions(){ce(this.options.beforeSetDimensions,[this])}setDimensions(){this.isHorizontal()?(this.width=this.maxWidth,this.left=0,this.right=this.width):(this.height=this.maxHeight,this.top=0,this.bottom=this.height),this.paddingLeft=0,this.paddingTop=0,this.paddingRight=0,this.paddingBottom=0}afterSetDimensions(){ce(this.options.afterSetDimensions,[this])}_callHooks(t){this.chart.notifyPlugins(t,this.getContext()),ce(this.options[t],[this])}beforeDataLimits(){this._callHooks("beforeDataLimits")}determineDataLimits(){}afterDataLimits(){this._callHooks("afterDataLimits")}beforeBuildTicks(){this._callHooks("beforeBuildTicks")}buildTicks(){return[]}afterBuildTicks(){this._callHooks("afterBuildTicks")}beforeTickToLabelConversion(){ce(this.options.beforeTickToLabelConversion,[this])}generateTickLabels(t){const e=this.options.ticks;let i,s,r;for(i=0,s=t.length;i<s;i++)r=t[i],r.label=ce(e.callback,[r.value,i,t],this)}afterTickToLabelConversion(){ce(this.options.afterTickToLabelConversion,[this])}beforeCalculateLabelRotation(){ce(this.options.beforeCalculateLabelRotation,[this])}calculateLabelRotation(){const t=this.options,e=t.ticks,i=bd(this.ticks.length,t.ticks.maxTicksLimit),s=e.minRotation||0,r=e.maxRotation;let o=s,a,l,c;if(!this._isVisible()||!e.display||s>=r||i<=1||!this.isHorizontal()){this.labelRotation=s;return}const h=this._getLabelSizes(),u=h.widest.width,d=h.highest.height,f=Oe(this.chart.width-u,0,this.maxWidth);a=t.offset?this.maxWidth/i:f/(i-1),u+6>a&&(a=f/(i-(t.offset?.5:1)),l=this.maxHeight-ir(t.grid)-e.padding-Md(t.title,this.chart.options.font),c=Math.sqrt(u*u+d*d),o=Yc(Math.min(Math.asin(Oe((h.highest.height+6)/a,-1,1)),Math.asin(Oe(l/c,-1,1))-Math.asin(Oe(d/c,-1,1)))),o=Math.max(s,Math.min(r,o))),this.labelRotation=o}afterCalculateLabelRotation(){ce(this.options.afterCalculateLabelRotation,[this])}afterAutoSkip(){}beforeFit(){ce(this.options.beforeFit,[this])}fit(){const t={width:0,height:0},{chart:e,options:{ticks:i,title:s,grid:r}}=this,o=this._isVisible(),a=this.isHorizontal();if(o){const l=Md(s,e.options.font);if(a?(t.width=this.maxWidth,t.height=ir(r)+l):(t.height=this.maxHeight,t.width=ir(r)+l),i.display&&this.ticks.length){const{first:c,last:h,widest:u,highest:d}=this._getLabelSizes(),f=i.padding*2,g=gn(this.labelRotation),_=Math.cos(g),m=Math.sin(g);if(a){const p=i.mirror?0:m*u.width+_*d.height;t.height=Math.min(this.maxHeight,t.height+p+f)}else{const p=i.mirror?0:_*u.width+m*d.height;t.width=Math.min(this.maxWidth,t.width+p+f)}this._calculatePadding(c,h,m,_)}}this._handleMargins(),a?(this.width=this._length=e.width-this._margins.left-this._margins.right,this.height=t.height):(this.width=t.width,this.height=this._length=e.height-this._margins.top-this._margins.bottom)}_calculatePadding(t,e,i,s){const{ticks:{align:r,padding:o},position:a}=this.options,l=this.labelRotation!==0,c=a!=="top"&&this.axis==="x";if(this.isHorizontal()){const h=this.getPixelForTick(0)-this.left,u=this.right-this.getPixelForTick(this.ticks.length-1);let d=0,f=0;l?c?(d=s*t.width,f=i*e.height):(d=i*t.height,f=s*e.width):r==="start"?f=e.width:r==="end"?d=t.width:r!=="inner"&&(d=t.width/2,f=e.width/2),this.paddingLeft=Math.max((d-h+o)*this.width/(this.width-h),0),this.paddingRight=Math.max((f-u+o)*this.width/(this.width-u),0)}else{let h=e.height/2,u=t.height/2;r==="start"?(h=0,u=t.height):r==="end"&&(h=e.height,u=0),this.paddingTop=h+o,this.paddingBottom=u+o}}_handleMargins(){this._margins&&(this._margins.left=Math.max(this.paddingLeft,this._margins.left),this._margins.top=Math.max(this.paddingTop,this._margins.top),this._margins.right=Math.max(this.paddingRight,this._margins.right),this._margins.bottom=Math.max(this.paddingBottom,this._margins.bottom))}afterFit(){ce(this.options.afterFit,[this])}isHorizontal(){const{axis:t,position:e}=this.options;return e==="top"||e==="bottom"||t==="x"}isFullSize(){return this.options.fullSize}_convertTicksToLabels(t){this.beforeTickToLabelConversion(),this.generateTickLabels(t);let e,i;for(e=0,i=t.length;e<i;e++)Zt(t[e].label)&&(t.splice(e,1),i--,e--);this.afterTickToLabelConversion()}_getLabelSizes(){let t=this._labelSizes;if(!t){const e=this.options.ticks.sampleSize;let i=this.ticks;e<i.length&&(i=yd(i,e)),this._labelSizes=t=this._computeLabelSizes(i,i.length,this.options.ticks.maxTicksLimit)}return t}_computeLabelSizes(t,e,i){const{ctx:s,_longestTextCache:r}=this,o=[],a=[],l=Math.floor(e/bd(e,i));let c=0,h=0,u,d,f,g,_,m,p,v,y,x,A;for(u=0;u<e;u+=l){if(g=t[u].label,_=this._resolveTickFontOptions(u),s.font=m=_.string,p=r[m]=r[m]||{data:{},gc:[]},v=_.lineHeight,y=x=0,!Zt(g)&&!me(g))y=ra(s,p.data,p.gc,y,g),x=v;else if(me(g))for(d=0,f=g.length;d<f;++d)A=g[d],!Zt(A)&&!me(A)&&(y=ra(s,p.data,p.gc,y,A),x+=v);o.push(y),a.push(x),c=Math.max(y,c),h=Math.max(x,h)}$S(r,e);const w=o.indexOf(c),T=a.indexOf(h),R=S=>({width:o[S]||0,height:a[S]||0});return{first:R(0),last:R(e-1),widest:R(w),highest:R(T),widths:o,heights:a}}getLabelForValue(t){return t}getPixelForValue(t,e){return NaN}getValueForPixel(t){}getPixelForTick(t){const e=this.ticks;return t<0||t>e.length-1?null:this.getPixelForValue(e[t].value)}getPixelForDecimal(t){this._reversePixels&&(t=1-t);const e=this._startPixel+t*this._length;return Uy(this._alignToPixels?Di(this.chart,e,0):e)}getDecimalForPixel(t){const e=(t-this._startPixel)/this._length;return this._reversePixels?1-e:e}getBasePixel(){return this.getPixelForValue(this.getBaseValue())}getBaseValue(){const{min:t,max:e}=this;return t<0&&e<0?e:t>0&&e>0?t:0}getContext(t){const e=this.ticks||[];if(t>=0&&t<e.length){const i=e[t];return i.$context||(i.$context=qS(this.getContext(),t,i))}return this.$context||(this.$context=YS(this.chart.getContext(),this))}_tickSize(){const t=this.options.ticks,e=gn(this.labelRotation),i=Math.abs(Math.cos(e)),s=Math.abs(Math.sin(e)),r=this._getLabelSizes(),o=t.autoSkipPadding||0,a=r?r.widest.width+o:0,l=r?r.highest.height+o:0;return this.isHorizontal()?l*i>a*s?a/i:l/s:l*s<a*i?l/i:a/s}_isVisible(){const t=this.options.display;return t!=="auto"?!!t:this.getMatchingVisibleMetas().length>0}_computeGridLineItems(t){const e=this.axis,i=this.chart,s=this.options,{grid:r,position:o,border:a}=s,l=r.offset,c=this.isHorizontal(),u=this.ticks.length+(l?1:0),d=ir(r),f=[],g=a.setContext(this.getContext()),_=g.display?g.width:0,m=_/2,p=function(H){return Di(i,H,_)};let v,y,x,A,w,T,R,S,b,P,N,F;if(o==="top")v=p(this.bottom),T=this.bottom-d,S=v-m,P=p(t.top)+m,F=t.bottom;else if(o==="bottom")v=p(this.top),P=t.top,F=p(t.bottom)-m,T=v+m,S=this.top+d;else if(o==="left")v=p(this.right),w=this.right-d,R=v-m,b=p(t.left)+m,N=t.right;else if(o==="right")v=p(this.left),b=t.left,N=p(t.right)-m,w=v+m,R=this.left+d;else if(e==="x"){if(o==="center")v=p((t.top+t.bottom)/2+.5);else if(Gt(o)){const H=Object.keys(o)[0],$=o[H];v=p(this.chart.scales[H].getPixelForValue($))}P=t.top,F=t.bottom,T=v+m,S=T+d}else if(e==="y"){if(o==="center")v=p((t.left+t.right)/2);else if(Gt(o)){const H=Object.keys(o)[0],$=o[H];v=p(this.chart.scales[H].getPixelForValue($))}w=v-m,R=w-d,b=t.left,N=t.right}const W=Ot(s.ticks.maxTicksLimit,u),G=Math.max(1,Math.ceil(u/W));for(y=0;y<u;y+=G){const H=this.getContext(y),$=r.setContext(H),V=a.setContext(H),et=$.lineWidth,at=$.color,_t=V.dash||[],Dt=V.dashOffset,$t=$.tickWidth,j=$.tickColor,tt=$.tickBorderDash||[],ft=$.tickBorderDashOffset;x=jS(this,y,l),x!==void 0&&(A=Di(i,x,et),c?w=R=b=N=A:T=S=P=F=A,f.push({tx1:w,ty1:T,tx2:R,ty2:S,x1:b,y1:P,x2:N,y2:F,width:et,color:at,borderDash:_t,borderDashOffset:Dt,tickWidth:$t,tickColor:j,tickBorderDash:tt,tickBorderDashOffset:ft}))}return this._ticksLength=u,this._borderValue=v,f}_computeLabelItems(t){const e=this.axis,i=this.options,{position:s,ticks:r}=i,o=this.isHorizontal(),a=this.ticks,{align:l,crossAlign:c,padding:h,mirror:u}=r,d=ir(i.grid),f=d+h,g=u?-h:f,_=-gn(this.labelRotation),m=[];let p,v,y,x,A,w,T,R,S,b,P,N,F="middle";if(s==="top")w=this.bottom-g,T=this._getXAxisLabelAlignment();else if(s==="bottom")w=this.top+g,T=this._getXAxisLabelAlignment();else if(s==="left"){const G=this._getYAxisLabelAlignment(d);T=G.textAlign,A=G.x}else if(s==="right"){const G=this._getYAxisLabelAlignment(d);T=G.textAlign,A=G.x}else if(e==="x"){if(s==="center")w=(t.top+t.bottom)/2+f;else if(Gt(s)){const G=Object.keys(s)[0],H=s[G];w=this.chart.scales[G].getPixelForValue(H)+f}T=this._getXAxisLabelAlignment()}else if(e==="y"){if(s==="center")A=(t.left+t.right)/2-f;else if(Gt(s)){const G=Object.keys(s)[0],H=s[G];A=this.chart.scales[G].getPixelForValue(H)}T=this._getYAxisLabelAlignment(d).textAlign}e==="y"&&(l==="start"?F="top":l==="end"&&(F="bottom"));const W=this._getLabelSizes();for(p=0,v=a.length;p<v;++p){y=a[p],x=y.label;const G=r.setContext(this.getContext(p));R=this.getPixelForTick(p)+r.labelOffset,S=this._resolveTickFontOptions(p),b=S.lineHeight,P=me(x)?x.length:1;const H=P/2,$=G.color,V=G.textStrokeColor,et=G.textStrokeWidth;let at=T;o?(A=R,T==="inner"&&(p===v-1?at=this.options.reverse?"left":"right":p===0?at=this.options.reverse?"right":"left":at="center"),s==="top"?c==="near"||_!==0?N=-P*b+b/2:c==="center"?N=-W.highest.height/2-H*b+b:N=-W.highest.height+b/2:c==="near"||_!==0?N=b/2:c==="center"?N=W.highest.height/2-H*b:N=W.highest.height-P*b,u&&(N*=-1),_!==0&&!G.showLabelBackdrop&&(A+=b/2*Math.sin(_))):(w=R,N=(1-P)*b/2);let _t;if(G.showLabelBackdrop){const Dt=ze(G.backdropPadding),$t=W.heights[p],j=W.widths[p];let tt=N-Dt.top,ft=0-Dt.left;switch(F){case"middle":tt-=$t/2;break;case"bottom":tt-=$t;break}switch(T){case"center":ft-=j/2;break;case"right":ft-=j;break;case"inner":p===v-1?ft-=j:p>0&&(ft-=j/2);break}_t={left:ft,top:tt,width:j+Dt.width,height:$t+Dt.height,color:G.backdropColor}}m.push({label:x,font:S,textOffset:N,options:{rotation:_,color:$,strokeColor:V,strokeWidth:et,textAlign:at,textBaseline:F,translation:[A,w],backdrop:_t}})}return m}_getXAxisLabelAlignment(){const{position:t,ticks:e}=this.options;if(-gn(this.labelRotation))return t==="top"?"left":"right";let s="center";return e.align==="start"?s="left":e.align==="end"?s="right":e.align==="inner"&&(s="inner"),s}_getYAxisLabelAlignment(t){const{position:e,ticks:{crossAlign:i,mirror:s,padding:r}}=this.options,o=this._getLabelSizes(),a=t+r,l=o.widest.width;let c,h;return e==="left"?s?(h=this.right+r,i==="near"?c="left":i==="center"?(c="center",h+=l/2):(c="right",h+=l)):(h=this.right-a,i==="near"?c="right":i==="center"?(c="center",h-=l/2):(c="left",h=this.left)):e==="right"?s?(h=this.left+r,i==="near"?c="right":i==="center"?(c="center",h-=l/2):(c="left",h-=l)):(h=this.left+a,i==="near"?c="left":i==="center"?(c="center",h+=l/2):(c="right",h=this.right)):c="right",{textAlign:c,x:h}}_computeLabelArea(){if(this.options.ticks.mirror)return;const t=this.chart,e=this.options.position;if(e==="left"||e==="right")return{top:0,left:this.left,bottom:t.height,right:this.right};if(e==="top"||e==="bottom")return{top:this.top,left:0,bottom:this.bottom,right:t.width}}drawBackground(){const{ctx:t,options:{backgroundColor:e},left:i,top:s,width:r,height:o}=this;e&&(t.save(),t.fillStyle=e,t.fillRect(i,s,r,o),t.restore())}getLineWidthForValue(t){const e=this.options.grid;if(!this._isVisible()||!e.display)return 0;const s=this.ticks.findIndex(r=>r.value===t);return s>=0?e.setContext(this.getContext(s)).lineWidth:0}drawGrid(t){const e=this.options.grid,i=this.ctx,s=this._gridLineItems||(this._gridLineItems=this._computeGridLineItems(t));let r,o;const a=(l,c,h)=>{!h.width||!h.color||(i.save(),i.lineWidth=h.width,i.strokeStyle=h.color,i.setLineDash(h.borderDash||[]),i.lineDashOffset=h.borderDashOffset,i.beginPath(),i.moveTo(l.x,l.y),i.lineTo(c.x,c.y),i.stroke(),i.restore())};if(e.display)for(r=0,o=s.length;r<o;++r){const l=s[r];e.drawOnChartArea&&a({x:l.x1,y:l.y1},{x:l.x2,y:l.y2},l),e.drawTicks&&a({x:l.tx1,y:l.ty1},{x:l.tx2,y:l.ty2},{color:l.tickColor,width:l.tickWidth,borderDash:l.tickBorderDash,borderDashOffset:l.tickBorderDashOffset})}}drawBorder(){const{chart:t,ctx:e,options:{border:i,grid:s}}=this,r=i.setContext(this.getContext()),o=i.display?r.width:0;if(!o)return;const a=s.setContext(this.getContext(0)).lineWidth,l=this._borderValue;let c,h,u,d;this.isHorizontal()?(c=Di(t,this.left,o)-o/2,h=Di(t,this.right,a)+a/2,u=d=l):(u=Di(t,this.top,o)-o/2,d=Di(t,this.bottom,a)+a/2,c=h=l),e.save(),e.lineWidth=r.width,e.strokeStyle=r.color,e.beginPath(),e.moveTo(c,u),e.lineTo(h,d),e.stroke(),e.restore()}drawLabels(t){if(!this.options.ticks.display)return;const i=this.ctx,s=this._computeLabelArea();s&&ma(i,s);const r=this.getLabelItems(t);for(const o of r){const a=o.options,l=o.font,c=o.label,h=o.textOffset;qi(i,c,0,h,l,a)}s&&ga(i)}drawTitle(){const{ctx:t,options:{position:e,title:i,reverse:s}}=this;if(!i.display)return;const r=Pe(i.font),o=ze(i.padding),a=i.align;let l=r.lineHeight/2;e==="bottom"||e==="center"||Gt(e)?(l+=o.bottom,me(i.text)&&(l+=r.lineHeight*(i.text.length-1))):l+=o.top;const{titleX:c,titleY:h,maxWidth:u,rotation:d}=ZS(this,l,e,a);qi(t,i.text,0,0,r,{color:i.color,maxWidth:u,rotation:d,textAlign:KS(a,e,s),textBaseline:"middle",translation:[c,h]})}draw(t){this._isVisible()&&(this.drawBackground(),this.drawGrid(t),this.drawBorder(),this.drawTitle(),this.drawLabels(t))}_layers(){const t=this.options,e=t.ticks&&t.ticks.z||0,i=Ot(t.grid&&t.grid.z,-1),s=Ot(t.border&&t.border.z,0);return!this._isVisible()||this.draw!==Zi.prototype.draw?[{z:e,draw:r=>{this.draw(r)}}]:[{z:i,draw:r=>{this.drawBackground(),this.drawGrid(r),this.drawTitle()}},{z:s,draw:()=>{this.drawBorder()}},{z:e,draw:r=>{this.drawLabels(r)}}]}getMatchingVisibleMetas(t){const e=this.chart.getSortedVisibleDatasetMetas(),i=this.axis+"AxisID",s=[];let r,o;for(r=0,o=e.length;r<o;++r){const a=e[r];a[i]===this.id&&(!t||a.type===t)&&s.push(a)}return s}_resolveTickFontOptions(t){const e=this.options.ticks.setContext(this.getContext(t));return Pe(e.font)}_maxDigits(){const t=this._resolveTickFontOptions(0).lineHeight;return(this.isHorizontal()?this.width:this.height)/t}}class Ao{constructor(t,e,i){this.type=t,this.scope=e,this.override=i,this.items=Object.create(null)}isForType(t){return Object.prototype.isPrototypeOf.call(this.type.prototype,t.prototype)}register(t){const e=Object.getPrototypeOf(t);let i;tE(e)&&(i=this.register(e));const s=this.items,r=t.id,o=this.scope+"."+r;if(!r)throw new Error("class does not have id: "+t);return r in s||(s[r]=t,JS(t,o,i),this.override&&ge.override(t.id,t.overrides)),o}get(t){return this.items[t]}unregister(t){const e=this.items,i=t.id,s=this.scope;i in e&&delete e[i],s&&i in ge[s]&&(delete ge[s][i],this.override&&delete Yi[i])}}function JS(n,t,e){const i=Sr(Object.create(null),[e?ge.get(e):{},ge.get(t),n.defaults]);ge.set(t,i),n.defaultRoutes&&QS(t,n.defaultRoutes),n.descriptors&&ge.describe(t,n.descriptors)}function QS(n,t){Object.keys(t).forEach(e=>{const i=e.split("."),s=i.pop(),r=[n].concat(i).join("."),o=t[e].split("."),a=o.pop(),l=o.join(".");ge.route(r,s,l,a)})}function tE(n){return"id"in n&&"defaults"in n}class eE{constructor(){this.controllers=new Ao(vn,"datasets",!0),this.elements=new Ao(bn,"elements"),this.plugins=new Ao(Object,"plugins"),this.scales=new Ao(Zi,"scales"),this._typedRegistries=[this.controllers,this.scales,this.elements]}add(...t){this._each("register",t)}remove(...t){this._each("unregister",t)}addControllers(...t){this._each("register",t,this.controllers)}addElements(...t){this._each("register",t,this.elements)}addPlugins(...t){this._each("register",t,this.plugins)}addScales(...t){this._each("register",t,this.scales)}getController(t){return this._get(t,this.controllers,"controller")}getElement(t){return this._get(t,this.elements,"element")}getPlugin(t){return this._get(t,this.plugins,"plugin")}getScale(t){return this._get(t,this.scales,"scale")}removeControllers(...t){this._each("unregister",t,this.controllers)}removeElements(...t){this._each("unregister",t,this.elements)}removePlugins(...t){this._each("unregister",t,this.plugins)}removeScales(...t){this._each("unregister",t,this.scales)}_each(t,e,i){[...e].forEach(s=>{const r=i||this._getRegistryForType(s);i||r.isForType(s)||r===this.plugins&&s.id?this._exec(t,r,s):ne(s,o=>{const a=i||this._getRegistryForType(o);this._exec(t,a,o)})})}_exec(t,e,i){const s=$c(t);ce(i["before"+s],[],i),e[t](i),ce(i["after"+s],[],i)}_getRegistryForType(t){for(let e=0;e<this._typedRegistries.length;e++){const i=this._typedRegistries[e];if(i.isForType(t))return i}return this.plugins}_get(t,e,i){const s=e.get(t);if(s===void 0)throw new Error('"'+t+'" is not a registered '+i+".");return s}}var Sn=new eE;class nE{constructor(){this._init=[]}notify(t,e,i,s){e==="beforeInit"&&(this._init=this._createDescriptors(t,!0),this._notify(this._init,t,"install"));const r=s?this._descriptors(t).filter(s):this._descriptors(t),o=this._notify(r,t,e,i);return e==="afterDestroy"&&(this._notify(r,t,"stop"),this._notify(this._init,t,"uninstall")),o}_notify(t,e,i,s){s=s||{};for(const r of t){const o=r.plugin,a=o[i],l=[e,s,r.options];if(ce(a,l,o)===!1&&s.cancelable)return!1}return!0}invalidate(){Zt(this._cache)||(this._oldCache=this._cache,this._cache=void 0)}_descriptors(t){if(this._cache)return this._cache;const e=this._cache=this._createDescriptors(t);return this._notifyStateChanges(t),e}_createDescriptors(t,e){const i=t&&t.config,s=Ot(i.options&&i.options.plugins,{}),r=iE(i);return s===!1&&!e?[]:rE(t,r,s,e)}_notifyStateChanges(t){const e=this._oldCache||[],i=this._cache,s=(r,o)=>r.filter(a=>!o.some(l=>a.plugin.id===l.plugin.id));this._notify(s(e,i),t,"stop"),this._notify(s(i,e),t,"start")}}function iE(n){const t={},e=[],i=Object.keys(Sn.plugins.items);for(let r=0;r<i.length;r++)e.push(Sn.getPlugin(i[r]));const s=n.plugins||[];for(let r=0;r<s.length;r++){const o=s[r];e.indexOf(o)===-1&&(e.push(o),t[o.id]=!0)}return{plugins:e,localIds:t}}function sE(n,t){return!t&&n===!1?null:n===!0?{}:n}function rE(n,{plugins:t,localIds:e},i,s){const r=[],o=n.getContext();for(const a of t){const l=a.id,c=sE(i[l],s);c!==null&&r.push({plugin:a,options:oE(n.config,{plugin:a,local:e[l]},c,o)})}return r}function oE(n,{plugin:t,local:e},i,s){const r=n.pluginScopeKeys(t),o=n.getOptionScopes(i,r);return e&&t.defaults&&o.push(t.defaults),n.createResolver(o,s,[""],{scriptable:!1,indexable:!1,allKeys:!0})}function xc(n,t){const e=ge.datasets[n]||{};return((t.datasets||{})[n]||{}).indexAxis||t.indexAxis||e.indexAxis||"x"}function aE(n,t){let e=n;return n==="_index_"?e=t:n==="_value_"&&(e=t==="x"?"y":"x"),e}function lE(n,t){return n===t?"_index_":"_value_"}function Sd(n){if(n==="x"||n==="y"||n==="r")return n}function cE(n){if(n==="top"||n==="bottom")return"x";if(n==="left"||n==="right")return"y"}function vc(n,...t){if(Sd(n))return n;for(const e of t){const i=e.axis||cE(e.position)||n.length>1&&Sd(n[0].toLowerCase());if(i)return i}throw new Error(`Cannot determine type of '${n}' axis. Please provide 'axis' or 'position' option.`)}function Ed(n,t,e){if(e[t+"AxisID"]===n)return{axis:t}}function hE(n,t){if(t.data&&t.data.datasets){const e=t.data.datasets.filter(i=>i.xAxisID===n||i.yAxisID===n);if(e.length)return Ed(n,"x",e[0])||Ed(n,"y",e[0])}return{}}function uE(n,t){const e=Yi[n.type]||{scales:{}},i=t.scales||{},s=xc(n.type,t),r=Object.create(null);return Object.keys(i).forEach(o=>{const a=i[o];if(!Gt(a))return console.error(`Invalid scale configuration for scale: ${o}`);if(a._proxy)return console.warn(`Ignoring resolver passed as options for scale: ${o}`);const l=vc(o,a,hE(o,n),ge.scales[a.type]),c=lE(l,s),h=e.scales||{};r[o]=mr(Object.create(null),[{axis:l},a,h[l],h[c]])}),n.data.datasets.forEach(o=>{const a=o.type||n.type,l=o.indexAxis||xc(a,t),h=(Yi[a]||{}).scales||{};Object.keys(h).forEach(u=>{const d=aE(u,l),f=o[d+"AxisID"]||d;r[f]=r[f]||Object.create(null),mr(r[f],[{axis:d},i[f],h[u]])})}),Object.keys(r).forEach(o=>{const a=r[o];mr(a,[ge.scales[a.type],ge.scale])}),r}function Up(n){const t=n.options||(n.options={});t.plugins=Ot(t.plugins,{}),t.scales=uE(n,t)}function Np(n){return n=n||{},n.datasets=n.datasets||[],n.labels=n.labels||[],n}function dE(n){return n=n||{},n.data=Np(n.data),Up(n),n}const Td=new Map,Fp=new Set;function Co(n,t){let e=Td.get(n);return e||(e=t(),Td.set(n,e),Fp.add(e)),e}const sr=(n,t,e)=>{const i=vi(t,e);i!==void 0&&n.add(i)};class fE{constructor(t){this._config=dE(t),this._scopeCache=new Map,this._resolverCache=new Map}get platform(){return this._config.platform}get type(){return this._config.type}set type(t){this._config.type=t}get data(){return this._config.data}set data(t){this._config.data=Np(t)}get options(){return this._config.options}set options(t){this._config.options=t}get plugins(){return this._config.plugins}update(){const t=this._config;this.clearCache(),Up(t)}clearCache(){this._scopeCache.clear(),this._resolverCache.clear()}datasetScopeKeys(t){return Co(t,()=>[[`datasets.${t}`,""]])}datasetAnimationScopeKeys(t,e){return Co(`${t}.transition.${e}`,()=>[[`datasets.${t}.transitions.${e}`,`transitions.${e}`],[`datasets.${t}`,""]])}datasetElementScopeKeys(t,e){return Co(`${t}-${e}`,()=>[[`datasets.${t}.elements.${e}`,`datasets.${t}`,`elements.${e}`,""]])}pluginScopeKeys(t){const e=t.id,i=this.type;return Co(`${i}-plugin-${e}`,()=>[[`plugins.${e}`,...t.additionalOptionScopes||[]]])}_cachedScopes(t,e){const i=this._scopeCache;let s=i.get(t);return(!s||e)&&(s=new Map,i.set(t,s)),s}getOptionScopes(t,e,i){const{options:s,type:r}=this,o=this._cachedScopes(t,i),a=o.get(e);if(a)return a;const l=new Set;e.forEach(h=>{t&&(l.add(t),h.forEach(u=>sr(l,t,u))),h.forEach(u=>sr(l,s,u)),h.forEach(u=>sr(l,Yi[r]||{},u)),h.forEach(u=>sr(l,ge,u)),h.forEach(u=>sr(l,mc,u))});const c=Array.from(l);return c.length===0&&c.push(Object.create(null)),Fp.has(e)&&o.set(e,c),c}chartOptionScopes(){const{options:t,type:e}=this;return[t,Yi[e]||{},ge.datasets[e]||{},{type:e},ge,mc]}resolveNamedOptions(t,e,i,s=[""]){const r={$shared:!0},{resolver:o,subPrefixes:a}=wd(this._resolverCache,t,s);let l=o;if(mE(o,e)){r.$shared=!1,i=bi(i)?i():i;const c=this.createResolver(t,i,a);l=ks(o,i,c)}for(const c of e)r[c]=l[c];return r}createResolver(t,e,i=[""],s){const{resolver:r}=wd(this._resolverCache,t,i);return Gt(e)?ks(r,e,void 0,s):r}}function wd(n,t,e){let i=n.get(t);i||(i=new Map,n.set(t,i));const s=e.join();let r=i.get(s);return r||(r={resolver:Qc(t,e),subPrefixes:e.filter(a=>!a.toLowerCase().includes("hover"))},i.set(s,r)),r}const pE=n=>Gt(n)&&Object.getOwnPropertyNames(n).some(t=>bi(n[t]));function mE(n,t){const{isScriptable:e,isIndexable:i}=gp(n);for(const s of t){const r=e(s),o=i(s),a=(o||r)&&n[s];if(r&&(bi(a)||pE(a))||o&&me(a))return!0}return!1}var gE="4.4.7";const _E=["top","bottom","left","right","chartArea"];function Ad(n,t){return n==="top"||n==="bottom"||_E.indexOf(n)===-1&&t==="x"}function Cd(n,t){return function(e,i){return e[n]===i[n]?e[t]-i[t]:e[n]-i[n]}}function Rd(n){const t=n.chart,e=t.options.animation;t.notifyPlugins("afterRender"),ce(e&&e.onComplete,[n],t)}function xE(n){const t=n.chart,e=t.options.animation;ce(e&&e.onProgress,[n],t)}function kp(n){return nh()&&typeof n=="string"?n=document.getElementById(n):n&&n.length&&(n=n[0]),n&&n.canvas&&(n=n.canvas),n}const $o={},Pd=n=>{const t=kp(n);return Object.values($o).filter(e=>e.canvas===t).pop()};function vE(n,t,e){const i=Object.keys(n);for(const s of i){const r=+s;if(r>=t){const o=n[s];delete n[s],(e>0||r>t)&&(n[r+e]=o)}}}function bE(n,t,e,i){return!e||n.type==="mouseout"?null:i?t:n}function Ro(n,t,e){return n.options.clip?n[e]:t[e]}function yE(n,t){const{xScale:e,yScale:i}=n;return e&&i?{left:Ro(e,t,"left"),right:Ro(e,t,"right"),top:Ro(i,t,"top"),bottom:Ro(i,t,"bottom")}:t}class Wn{static register(...t){Sn.add(...t),Dd()}static unregister(...t){Sn.remove(...t),Dd()}constructor(t,e){const i=this.config=new fE(e),s=kp(t),r=Pd(s);if(r)throw new Error("Canvas is already in use. Chart with ID '"+r.id+"' must be destroyed before the canvas with ID '"+r.canvas.id+"' can be reused.");const o=i.createResolver(i.chartOptionScopes(),this.getContext());this.platform=new(i.platform||kS(s)),this.platform.updateConfig(i);const a=this.platform.acquireContext(s,o.aspectRatio),l=a&&a.canvas,c=l&&l.height,h=l&&l.width;if(this.id=Sy(),this.ctx=a,this.canvas=l,this.width=h,this.height=c,this._options=o,this._aspectRatio=this.aspectRatio,this._layers=[],this._metasets=[],this._stacks=void 0,this.boxes=[],this.currentDevicePixelRatio=void 0,this.chartArea=void 0,this._active=[],this._lastEvent=void 0,this._listeners={},this._responsiveListeners=void 0,this._sortedMetasets=[],this.scales={},this._plugins=new nE,this.$proxies={},this._hiddenIndices={},this.attached=!1,this._animationsDisabled=void 0,this.$context=void 0,this._doResize=By(u=>this.update(u),o.resizeDelay||0),this._dataChanges=[],$o[this.id]=this,!a||!l){console.error("Failed to create chart: can't acquire context from the given item");return}Hn.listen(this,"complete",Rd),Hn.listen(this,"progress",xE),this._initialize(),this.attached&&this.update()}get aspectRatio(){const{options:{aspectRatio:t,maintainAspectRatio:e},width:i,height:s,_aspectRatio:r}=this;return Zt(t)?e&&r?r:s?i/s:null:t}get data(){return this.config.data}set data(t){this.config.data=t}get options(){return this._options}set options(t){this.config.options=t}get registry(){return Sn}_initialize(){return this.notifyPlugins("beforeInit"),this.options.responsive?this.resize():Qu(this,this.options.devicePixelRatio),this.bindEvents(),this.notifyPlugins("afterInit"),this}clear(){return Ku(this.canvas,this.ctx),this}stop(){return Hn.stop(this),this}resize(t,e){Hn.running(this)?this._resizeBeforeDraw={width:t,height:e}:this._resize(t,e)}_resize(t,e){const i=this.options,s=this.canvas,r=i.maintainAspectRatio&&this.aspectRatio,o=this.platform.getMaximumSize(s,t,e,r),a=i.devicePixelRatio||this.platform.getDevicePixelRatio(),l=this.width?"resize":"attach";this.width=o.width,this.height=o.height,this._aspectRatio=this.aspectRatio,Qu(this,a,!0)&&(this.notifyPlugins("resize",{size:o}),ce(i.onResize,[this,o],this),this.attached&&this._doResize(l)&&this.render())}ensureScalesHaveIDs(){const e=this.options.scales||{};ne(e,(i,s)=>{i.id=s})}buildOrUpdateScales(){const t=this.options,e=t.scales,i=this.scales,s=Object.keys(i).reduce((o,a)=>(o[a]=!1,o),{});let r=[];e&&(r=r.concat(Object.keys(e).map(o=>{const a=e[o],l=vc(o,a),c=l==="r",h=l==="x";return{options:a,dposition:c?"chartArea":h?"bottom":"left",dtype:c?"radialLinear":h?"category":"linear"}}))),ne(r,o=>{const a=o.options,l=a.id,c=vc(l,a),h=Ot(a.type,o.dtype);(a.position===void 0||Ad(a.position,c)!==Ad(o.dposition))&&(a.position=o.dposition),s[l]=!0;let u=null;if(l in i&&i[l].type===h)u=i[l];else{const d=Sn.getScale(h);u=new d({id:l,type:h,ctx:this.ctx,chart:this}),i[u.id]=u}u.init(a,t)}),ne(s,(o,a)=>{o||delete i[a]}),ne(i,o=>{Be.configure(this,o,o.options),Be.addBox(this,o)})}_updateMetasets(){const t=this._metasets,e=this.data.datasets.length,i=t.length;if(t.sort((s,r)=>s.index-r.index),i>e){for(let s=e;s<i;++s)this._destroyDatasetMeta(s);t.splice(e,i-e)}this._sortedMetasets=t.slice(0).sort(Cd("order","index"))}_removeUnreferencedMetasets(){const{_metasets:t,data:{datasets:e}}=this;t.length>e.length&&delete this._stacks,t.forEach((i,s)=>{e.filter(r=>r===i._dataset).length===0&&this._destroyDatasetMeta(s)})}buildOrUpdateControllers(){const t=[],e=this.data.datasets;let i,s;for(this._removeUnreferencedMetasets(),i=0,s=e.length;i<s;i++){const r=e[i];let o=this.getDatasetMeta(i);const a=r.type||this.config.type;if(o.type&&o.type!==a&&(this._destroyDatasetMeta(i),o=this.getDatasetMeta(i)),o.type=a,o.indexAxis=r.indexAxis||xc(a,this.options),o.order=r.order||0,o.index=i,o.label=""+r.label,o.visible=this.isDatasetVisible(i),o.controller)o.controller.updateIndex(i),o.controller.linkScales();else{const l=Sn.getController(a),{datasetElementType:c,dataElementType:h}=ge.datasets[a];Object.assign(l,{dataElementType:Sn.getElement(h),datasetElementType:c&&Sn.getElement(c)}),o.controller=new l(this,i),t.push(o.controller)}}return this._updateMetasets(),t}_resetElements(){ne(this.data.datasets,(t,e)=>{this.getDatasetMeta(e).controller.reset()},this)}reset(){this._resetElements(),this.notifyPlugins("reset")}update(t){const e=this.config;e.update();const i=this._options=e.createResolver(e.chartOptionScopes(),this.getContext()),s=this._animationsDisabled=!i.animation;if(this._updateScales(),this._checkEventBindings(),this._updateHiddenIndices(),this._plugins.invalidate(),this.notifyPlugins("beforeUpdate",{mode:t,cancelable:!0})===!1)return;const r=this.buildOrUpdateControllers();this.notifyPlugins("beforeElementsUpdate");let o=0;for(let c=0,h=this.data.datasets.length;c<h;c++){const{controller:u}=this.getDatasetMeta(c),d=!s&&r.indexOf(u)===-1;u.buildOrUpdateElements(d),o=Math.max(+u.getMaxOverflow(),o)}o=this._minPadding=i.layout.autoPadding?o:0,this._updateLayout(o),s||ne(r,c=>{c.reset()}),this._updateDatasets(t),this.notifyPlugins("afterUpdate",{mode:t}),this._layers.sort(Cd("z","_idx"));const{_active:a,_lastEvent:l}=this;l?this._eventHandler(l,!0):a.length&&this._updateHoverStyles(a,a,!0),this.render()}_updateScales(){ne(this.scales,t=>{Be.removeBox(this,t)}),this.ensureScalesHaveIDs(),this.buildOrUpdateScales()}_checkEventBindings(){const t=this.options,e=new Set(Object.keys(this._listeners)),i=new Set(t.events);(!Hu(e,i)||!!this._responsiveListeners!==t.responsive)&&(this.unbindEvents(),this.bindEvents())}_updateHiddenIndices(){const{_hiddenIndices:t}=this,e=this._getUniformDataChanges()||[];for(const{method:i,start:s,count:r}of e){const o=i==="_removeElements"?-r:r;vE(t,s,o)}}_getUniformDataChanges(){const t=this._dataChanges;if(!t||!t.length)return;this._dataChanges=[];const e=this.data.datasets.length,i=r=>new Set(t.filter(o=>o[0]===r).map((o,a)=>a+","+o.splice(1).join(","))),s=i(0);for(let r=1;r<e;r++)if(!Hu(s,i(r)))return;return Array.from(s).map(r=>r.split(",")).map(r=>({method:r[1],start:+r[2],count:+r[3]}))}_updateLayout(t){if(this.notifyPlugins("beforeLayout",{cancelable:!0})===!1)return;Be.update(this,this.width,this.height,t);const e=this.chartArea,i=e.width<=0||e.height<=0;this._layers=[],ne(this.boxes,s=>{i&&s.position==="chartArea"||(s.configure&&s.configure(),this._layers.push(...s._layers()))},this),this._layers.forEach((s,r)=>{s._idx=r}),this.notifyPlugins("afterLayout")}_updateDatasets(t){if(this.notifyPlugins("beforeDatasetsUpdate",{mode:t,cancelable:!0})!==!1){for(let e=0,i=this.data.datasets.length;e<i;++e)this.getDatasetMeta(e).controller.configure();for(let e=0,i=this.data.datasets.length;e<i;++e)this._updateDataset(e,bi(t)?t({datasetIndex:e}):t);this.notifyPlugins("afterDatasetsUpdate",{mode:t})}}_updateDataset(t,e){const i=this.getDatasetMeta(t),s={meta:i,index:t,mode:e,cancelable:!0};this.notifyPlugins("beforeDatasetUpdate",s)!==!1&&(i.controller._update(e),s.cancelable=!1,this.notifyPlugins("afterDatasetUpdate",s))}render(){this.notifyPlugins("beforeRender",{cancelable:!0})!==!1&&(Hn.has(this)?this.attached&&!Hn.running(this)&&Hn.start(this):(this.draw(),Rd({chart:this})))}draw(){let t;if(this._resizeBeforeDraw){const{width:i,height:s}=this._resizeBeforeDraw;this._resizeBeforeDraw=null,this._resize(i,s)}if(this.clear(),this.width<=0||this.height<=0||this.notifyPlugins("beforeDraw",{cancelable:!0})===!1)return;const e=this._layers;for(t=0;t<e.length&&e[t].z<=0;++t)e[t].draw(this.chartArea);for(this._drawDatasets();t<e.length;++t)e[t].draw(this.chartArea);this.notifyPlugins("afterDraw")}_getSortedDatasetMetas(t){const e=this._sortedMetasets,i=[];let s,r;for(s=0,r=e.length;s<r;++s){const o=e[s];(!t||o.visible)&&i.push(o)}return i}getSortedVisibleDatasetMetas(){return this._getSortedDatasetMetas(!0)}_drawDatasets(){if(this.notifyPlugins("beforeDatasetsDraw",{cancelable:!0})===!1)return;const t=this.getSortedVisibleDatasetMetas();for(let e=t.length-1;e>=0;--e)this._drawDataset(t[e]);this.notifyPlugins("afterDatasetsDraw")}_drawDataset(t){const e=this.ctx,i=t._clip,s=!i.disabled,r=yE(t,this.chartArea),o={meta:t,index:t.index,cancelable:!0};this.notifyPlugins("beforeDatasetDraw",o)!==!1&&(s&&ma(e,{left:i.left===!1?0:r.left-i.left,right:i.right===!1?this.width:r.right+i.right,top:i.top===!1?0:r.top-i.top,bottom:i.bottom===!1?this.height:r.bottom+i.bottom}),t.controller.draw(),s&&ga(e),o.cancelable=!1,this.notifyPlugins("afterDatasetDraw",o))}isPointInArea(t){return Kn(t,this.chartArea,this._minPadding)}getElementsAtEventForMode(t,e,i,s){const r=_S.modes[e];return typeof r=="function"?r(this,t,i,s):[]}getDatasetMeta(t){const e=this.data.datasets[t],i=this._metasets;let s=i.filter(r=>r&&r._dataset===e).pop();return s||(s={type:null,data:[],dataset:null,controller:null,hidden:null,xAxisID:null,yAxisID:null,order:e&&e.order||0,index:t,_dataset:e,_parsed:[],_sorted:!1},i.push(s)),s}getContext(){return this.$context||(this.$context=Mi(null,{chart:this,type:"chart"}))}getVisibleDatasetCount(){return this.getSortedVisibleDatasetMetas().length}isDatasetVisible(t){const e=this.data.datasets[t];if(!e)return!1;const i=this.getDatasetMeta(t);return typeof i.hidden=="boolean"?!i.hidden:!e.hidden}setDatasetVisibility(t,e){const i=this.getDatasetMeta(t);i.hidden=!e}toggleDataVisibility(t){this._hiddenIndices[t]=!this._hiddenIndices[t]}getDataVisibility(t){return!this._hiddenIndices[t]}_updateVisibility(t,e,i){const s=i?"show":"hide",r=this.getDatasetMeta(t),o=r.controller._resolveAnimations(void 0,s);Er(e)?(r.data[e].hidden=!i,this.update()):(this.setDatasetVisibility(t,i),o.update(r,{visible:i}),this.update(a=>a.datasetIndex===t?s:void 0))}hide(t,e){this._updateVisibility(t,e,!1)}show(t,e){this._updateVisibility(t,e,!0)}_destroyDatasetMeta(t){const e=this._metasets[t];e&&e.controller&&e.controller._destroy(),delete this._metasets[t]}_stop(){let t,e;for(this.stop(),Hn.remove(this),t=0,e=this.data.datasets.length;t<e;++t)this._destroyDatasetMeta(t)}destroy(){this.notifyPlugins("beforeDestroy");const{canvas:t,ctx:e}=this;this._stop(),this.config.clearCache(),t&&(this.unbindEvents(),Ku(t,e),this.platform.releaseContext(e),this.canvas=null,this.ctx=null),delete $o[this.id],this.notifyPlugins("afterDestroy")}toBase64Image(...t){return this.canvas.toDataURL(...t)}bindEvents(){this.bindUserEvents(),this.options.responsive?this.bindResponsiveEvents():this.attached=!0}bindUserEvents(){const t=this._listeners,e=this.platform,i=(r,o)=>{e.addEventListener(this,r,o),t[r]=o},s=(r,o,a)=>{r.offsetX=o,r.offsetY=a,this._eventHandler(r)};ne(this.options.events,r=>i(r,s))}bindResponsiveEvents(){this._responsiveListeners||(this._responsiveListeners={});const t=this._responsiveListeners,e=this.platform,i=(l,c)=>{e.addEventListener(this,l,c),t[l]=c},s=(l,c)=>{t[l]&&(e.removeEventListener(this,l,c),delete t[l])},r=(l,c)=>{this.canvas&&this.resize(l,c)};let o;const a=()=>{s("attach",a),this.attached=!0,this.resize(),i("resize",r),i("detach",o)};o=()=>{this.attached=!1,s("resize",r),this._stop(),this._resize(0,0),i("attach",a)},e.isAttached(this.canvas)?a():o()}unbindEvents(){ne(this._listeners,(t,e)=>{this.platform.removeEventListener(this,e,t)}),this._listeners={},ne(this._responsiveListeners,(t,e)=>{this.platform.removeEventListener(this,e,t)}),this._responsiveListeners=void 0}updateHoverStyle(t,e,i){const s=i?"set":"remove";let r,o,a,l;for(e==="dataset"&&(r=this.getDatasetMeta(t[0].datasetIndex),r.controller["_"+s+"DatasetHoverStyle"]()),a=0,l=t.length;a<l;++a){o=t[a];const c=o&&this.getDatasetMeta(o.datasetIndex).controller;c&&c[s+"HoverStyle"](o.element,o.datasetIndex,o.index)}}getActiveElements(){return this._active||[]}setActiveElements(t){const e=this._active||[],i=t.map(({datasetIndex:r,index:o})=>{const a=this.getDatasetMeta(r);if(!a)throw new Error("No dataset found at index "+r);return{datasetIndex:r,element:a.data[o],index:o}});!na(i,e)&&(this._active=i,this._lastEvent=null,this._updateHoverStyles(i,e))}notifyPlugins(t,e,i){return this._plugins.notify(this,t,e,i)}isPluginEnabled(t){return this._plugins._cache.filter(e=>e.plugin.id===t).length===1}_updateHoverStyles(t,e,i){const s=this.options.hover,r=(l,c)=>l.filter(h=>!c.some(u=>h.datasetIndex===u.datasetIndex&&h.index===u.index)),o=r(e,t),a=i?t:r(t,e);o.length&&this.updateHoverStyle(o,s.mode,!1),a.length&&s.mode&&this.updateHoverStyle(a,s.mode,!0)}_eventHandler(t,e){const i={event:t,replay:e,cancelable:!0,inChartArea:this.isPointInArea(t)},s=o=>(o.options.events||this.options.events).includes(t.native.type);if(this.notifyPlugins("beforeEvent",i,s)===!1)return;const r=this._handleEvent(t,e,i.inChartArea);return i.cancelable=!1,this.notifyPlugins("afterEvent",i,s),(r||i.changed)&&this.render(),this}_handleEvent(t,e,i){const{_active:s=[],options:r}=this,o=e,a=this._getActiveElements(t,s,i,o),l=Ry(t),c=bE(t,this._lastEvent,i,l);i&&(this._lastEvent=null,ce(r.onHover,[t,a,this],this),l&&ce(r.onClick,[t,a,this],this));const h=!na(a,s);return(h||e)&&(this._active=a,this._updateHoverStyles(a,s,e)),this._lastEvent=c,h}_getActiveElements(t,e,i,s){if(t.type==="mouseout")return[];if(!i)return e;const r=this.options.hover;return this.getElementsAtEventForMode(t,r.mode,r,s)}}st(Wn,"defaults",ge),st(Wn,"instances",$o),st(Wn,"overrides",Yi),st(Wn,"registry",Sn),st(Wn,"version",gE),st(Wn,"getChart",Pd);function Dd(){return ne(Wn.instances,n=>n._plugins.invalidate())}function ME(n,t,e){const{startAngle:i,pixelMargin:s,x:r,y:o,outerRadius:a,innerRadius:l}=t;let c=s/a;n.beginPath(),n.arc(r,o,a,i-c,e+c),l>s?(c=s/l,n.arc(r,o,l,e+c,i-c,!0)):n.arc(r,o,s,e+we,i-we),n.closePath(),n.clip()}function SE(n){return Jc(n,["outerStart","outerEnd","innerStart","innerEnd"])}function EE(n,t,e,i){const s=SE(n.options.borderRadius),r=(e-t)/2,o=Math.min(r,i*t/2),a=l=>{const c=(e-Math.min(r,l))*i/2;return Oe(l,0,Math.min(r,c))};return{outerStart:a(s.outerStart),outerEnd:a(s.outerEnd),innerStart:Oe(s.innerStart,0,o),innerEnd:Oe(s.innerEnd,0,o)}}function bs(n,t,e,i){return{x:e+n*Math.cos(t),y:i+n*Math.sin(t)}}function la(n,t,e,i,s,r){const{x:o,y:a,startAngle:l,pixelMargin:c,innerRadius:h}=t,u=Math.max(t.outerRadius+i+e-c,0),d=h>0?h+i+e+c:0;let f=0;const g=s-l;if(i){const G=h>0?h-i:0,H=u>0?u-i:0,$=(G+H)/2,V=$!==0?g*$/($+i):g;f=(g-V)/2}const _=Math.max(.001,g*u-e/pe)/u,m=(g-_)/2,p=l+m+f,v=s-m-f,{outerStart:y,outerEnd:x,innerStart:A,innerEnd:w}=EE(t,d,u,v-p),T=u-y,R=u-x,S=p+y/T,b=v-x/R,P=d+A,N=d+w,F=p+A/P,W=v-w/N;if(n.beginPath(),r){const G=(S+b)/2;if(n.arc(o,a,u,S,G),n.arc(o,a,u,G,b),x>0){const et=bs(R,b,o,a);n.arc(et.x,et.y,x,b,v+we)}const H=bs(N,v,o,a);if(n.lineTo(H.x,H.y),w>0){const et=bs(N,W,o,a);n.arc(et.x,et.y,w,v+we,W+Math.PI)}const $=(v-w/d+(p+A/d))/2;if(n.arc(o,a,d,v-w/d,$,!0),n.arc(o,a,d,$,p+A/d,!0),A>0){const et=bs(P,F,o,a);n.arc(et.x,et.y,A,F+Math.PI,p-we)}const V=bs(T,p,o,a);if(n.lineTo(V.x,V.y),y>0){const et=bs(T,S,o,a);n.arc(et.x,et.y,y,p-we,S)}}else{n.moveTo(o,a);const G=Math.cos(S)*u+o,H=Math.sin(S)*u+a;n.lineTo(G,H);const $=Math.cos(b)*u+o,V=Math.sin(b)*u+a;n.lineTo($,V)}n.closePath()}function TE(n,t,e,i,s){const{fullCircles:r,startAngle:o,circumference:a}=t;let l=t.endAngle;if(r){la(n,t,e,i,l,s);for(let c=0;c<r;++c)n.fill();isNaN(a)||(l=o+(a%fe||fe))}return la(n,t,e,i,l,s),n.fill(),l}function wE(n,t,e,i,s){const{fullCircles:r,startAngle:o,circumference:a,options:l}=t,{borderWidth:c,borderJoinStyle:h,borderDash:u,borderDashOffset:d}=l,f=l.borderAlign==="inner";if(!c)return;n.setLineDash(u||[]),n.lineDashOffset=d,f?(n.lineWidth=c*2,n.lineJoin=h||"round"):(n.lineWidth=c,n.lineJoin=h||"bevel");let g=t.endAngle;if(r){la(n,t,e,i,g,s);for(let _=0;_<r;++_)n.stroke();isNaN(a)||(g=o+(a%fe||fe))}f&&ME(n,t,g),r||(la(n,t,e,i,g,s),n.stroke())}class ur extends bn{constructor(e){super();st(this,"circumference");st(this,"endAngle");st(this,"fullCircles");st(this,"innerRadius");st(this,"outerRadius");st(this,"pixelMargin");st(this,"startAngle");this.options=void 0,this.circumference=void 0,this.startAngle=void 0,this.endAngle=void 0,this.innerRadius=void 0,this.outerRadius=void 0,this.pixelMargin=0,this.fullCircles=0,e&&Object.assign(this,e)}inRange(e,i,s){const r=this.getProps(["x","y"],s),{angle:o,distance:a}=op(r,{x:e,y:i}),{startAngle:l,endAngle:c,innerRadius:h,outerRadius:u,circumference:d}=this.getProps(["startAngle","endAngle","innerRadius","outerRadius","circumference"],s),f=(this.options.spacing+this.options.borderWidth)/2,g=Ot(d,c-l),_=Tr(o,l,c)&&l!==c,m=g>=fe||_,p=Yn(a,h+f,u+f);return m&&p}getCenterPoint(e){const{x:i,y:s,startAngle:r,endAngle:o,innerRadius:a,outerRadius:l}=this.getProps(["x","y","startAngle","endAngle","innerRadius","outerRadius"],e),{offset:c,spacing:h}=this.options,u=(r+o)/2,d=(a+l+h+c)/2;return{x:i+Math.cos(u)*d,y:s+Math.sin(u)*d}}tooltipPosition(e){return this.getCenterPoint(e)}draw(e){const{options:i,circumference:s}=this,r=(i.offset||0)/4,o=(i.spacing||0)/2,a=i.circular;if(this.pixelMargin=i.borderAlign==="inner"?.33:0,this.fullCircles=s>fe?Math.floor(s/fe):0,s===0||this.innerRadius<0||this.outerRadius<0)return;e.save();const l=(this.startAngle+this.endAngle)/2;e.translate(Math.cos(l)*r,Math.sin(l)*r);const c=1-Math.sin(Math.min(pe,s||0)),h=r*c;e.fillStyle=i.backgroundColor,e.strokeStyle=i.borderColor,TE(e,this,h,o,a),wE(e,this,h,o,a),e.restore()}}st(ur,"id","arc"),st(ur,"defaults",{borderAlign:"center",borderColor:"#fff",borderDash:[],borderDashOffset:0,borderJoinStyle:void 0,borderRadius:0,borderWidth:2,offset:0,spacing:0,angle:void 0,circular:!0}),st(ur,"defaultRoutes",{backgroundColor:"backgroundColor"}),st(ur,"descriptors",{_scriptable:!0,_indexable:e=>e!=="borderDash"});function Bp(n,t,e=t){n.lineCap=Ot(e.borderCapStyle,t.borderCapStyle),n.setLineDash(Ot(e.borderDash,t.borderDash)),n.lineDashOffset=Ot(e.borderDashOffset,t.borderDashOffset),n.lineJoin=Ot(e.borderJoinStyle,t.borderJoinStyle),n.lineWidth=Ot(e.borderWidth,t.borderWidth),n.strokeStyle=Ot(e.borderColor,t.borderColor)}function AE(n,t,e){n.lineTo(e.x,e.y)}function CE(n){return n.stepped?Zy:n.tension||n.cubicInterpolationMode==="monotone"?Jy:AE}function zp(n,t,e={}){const i=n.length,{start:s=0,end:r=i-1}=e,{start:o,end:a}=t,l=Math.max(s,o),c=Math.min(r,a),h=s<o&&r<o||s>a&&r>a;return{count:i,start:l,loop:t.loop,ilen:c<l&&!h?i+c-l:c-l}}function RE(n,t,e,i){const{points:s,options:r}=t,{count:o,start:a,loop:l,ilen:c}=zp(s,e,i),h=CE(r);let{move:u=!0,reverse:d}=i||{},f,g,_;for(f=0;f<=c;++f)g=s[(a+(d?c-f:f))%o],!g.skip&&(u?(n.moveTo(g.x,g.y),u=!1):h(n,_,g,d,r.stepped),_=g);return l&&(g=s[(a+(d?c:0))%o],h(n,_,g,d,r.stepped)),!!l}function PE(n,t,e,i){const s=t.points,{count:r,start:o,ilen:a}=zp(s,e,i),{move:l=!0,reverse:c}=i||{};let h=0,u=0,d,f,g,_,m,p;const v=x=>(o+(c?a-x:x))%r,y=()=>{_!==m&&(n.lineTo(h,m),n.lineTo(h,_),n.lineTo(h,p))};for(l&&(f=s[v(0)],n.moveTo(f.x,f.y)),d=0;d<=a;++d){if(f=s[v(d)],f.skip)continue;const x=f.x,A=f.y,w=x|0;w===g?(A<_?_=A:A>m&&(m=A),h=(u*h+x)/++u):(y(),n.lineTo(x,A),g=w,u=0,_=m=A),p=A}y()}function bc(n){const t=n.options,e=t.borderDash&&t.borderDash.length;return!n._decimated&&!n._loop&&!t.tension&&t.cubicInterpolationMode!=="monotone"&&!t.stepped&&!e?PE:RE}function DE(n){return n.stepped?PM:n.tension||n.cubicInterpolationMode==="monotone"?DM:Ni}function LE(n,t,e,i){let s=t._path;s||(s=t._path=new Path2D,t.path(s,e,i)&&s.closePath()),Bp(n,t.options),n.stroke(s)}function IE(n,t,e,i){const{segments:s,options:r}=t,o=bc(t);for(const a of s)Bp(n,r,a.style),n.beginPath(),o(n,t,a,{start:e,end:e+i-1})&&n.closePath(),n.stroke()}const OE=typeof Path2D=="function";function UE(n,t,e,i){OE&&!t.options.segment?LE(n,t,e,i):IE(n,t,e,i)}class ui extends bn{constructor(t){super(),this.animated=!0,this.options=void 0,this._chart=void 0,this._loop=void 0,this._fullLoop=void 0,this._path=void 0,this._points=void 0,this._segments=void 0,this._decimated=!1,this._pointsUpdated=!1,this._datasetIndex=void 0,t&&Object.assign(this,t)}updateControlPoints(t,e){const i=this.options;if((i.tension||i.cubicInterpolationMode==="monotone")&&!i.stepped&&!this._pointsUpdated){const s=i.spanGaps?this._loop:this._fullLoop;MM(this._points,i,t,s,e),this._pointsUpdated=!0}}set points(t){this._points=t,delete this._segments,delete this._path,this._pointsUpdated=!1}get points(){return this._points}get segments(){return this._segments||(this._segments=FM(this,this.options.segment))}first(){const t=this.segments,e=this.points;return t.length&&e[t[0].start]}last(){const t=this.segments,e=this.points,i=t.length;return i&&e[t[i-1].end]}interpolate(t,e){const i=this.options,s=t[e],r=this.points,o=wp(this,{property:e,start:s,end:s});if(!o.length)return;const a=[],l=DE(i);let c,h;for(c=0,h=o.length;c<h;++c){const{start:u,end:d}=o[c],f=r[u],g=r[d];if(f===g){a.push(f);continue}const _=Math.abs((s-f[e])/(g[e]-f[e])),m=l(f,g,_,i.stepped);m[e]=t[e],a.push(m)}return a.length===1?a[0]:a}pathSegment(t,e,i){return bc(this)(t,this,e,i)}path(t,e,i){const s=this.segments,r=bc(this);let o=this._loop;e=e||0,i=i||this.points.length-e;for(const a of s)o&=r(t,this,a,{start:e,end:e+i-1});return!!o}draw(t,e,i,s){const r=this.options||{};(this.points||[]).length&&r.borderWidth&&(t.save(),UE(t,this,i,s),t.restore()),this.animated&&(this._pointsUpdated=!1,this._path=void 0)}}st(ui,"id","line"),st(ui,"defaults",{borderCapStyle:"butt",borderDash:[],borderDashOffset:0,borderJoinStyle:"miter",borderWidth:3,capBezierPoints:!0,cubicInterpolationMode:"default",fill:!1,spanGaps:!1,stepped:!1,tension:0}),st(ui,"defaultRoutes",{backgroundColor:"backgroundColor",borderColor:"borderColor"}),st(ui,"descriptors",{_scriptable:!0,_indexable:t=>t!=="borderDash"&&t!=="fill"});function Ld(n,t,e,i){const s=n.options,{[e]:r}=n.getProps([e],i);return Math.abs(t-r)<s.radius+s.hitRadius}class Yo extends bn{constructor(e){super();st(this,"parsed");st(this,"skip");st(this,"stop");this.options=void 0,this.parsed=void 0,this.skip=void 0,this.stop=void 0,e&&Object.assign(this,e)}inRange(e,i,s){const r=this.options,{x:o,y:a}=this.getProps(["x","y"],s);return Math.pow(e-o,2)+Math.pow(i-a,2)<Math.pow(r.hitRadius+r.radius,2)}inXRange(e,i){return Ld(this,e,"x",i)}inYRange(e,i){return Ld(this,e,"y",i)}getCenterPoint(e){const{x:i,y:s}=this.getProps(["x","y"],e);return{x:i,y:s}}size(e){e=e||this.options||{};let i=e.radius||0;i=Math.max(i,i&&e.hoverRadius||0);const s=i&&e.borderWidth||0;return(i+s)*2}draw(e,i){const s=this.options;this.skip||s.radius<.1||!Kn(this,i,this.size(s)/2)||(e.strokeStyle=s.borderColor,e.lineWidth=s.borderWidth,e.fillStyle=s.backgroundColor,gc(e,s,this.x,this.y))}getRange(){const e=this.options||{};return e.radius+e.hitRadius}}st(Yo,"id","point"),st(Yo,"defaults",{borderWidth:1,hitRadius:1,hoverBorderWidth:1,hoverRadius:4,pointStyle:"circle",radius:3,rotation:0}),st(Yo,"defaultRoutes",{backgroundColor:"backgroundColor",borderColor:"borderColor"});function Hp(n,t){const{x:e,y:i,base:s,width:r,height:o}=n.getProps(["x","y","base","width","height"],t);let a,l,c,h,u;return n.horizontal?(u=o/2,a=Math.min(e,s),l=Math.max(e,s),c=i-u,h=i+u):(u=r/2,a=e-u,l=e+u,c=Math.min(i,s),h=Math.max(i,s)),{left:a,top:c,right:l,bottom:h}}function di(n,t,e,i){return n?0:Oe(t,e,i)}function NE(n,t,e){const i=n.options.borderWidth,s=n.borderSkipped,r=mp(i);return{t:di(s.top,r.top,0,e),r:di(s.right,r.right,0,t),b:di(s.bottom,r.bottom,0,e),l:di(s.left,r.left,0,t)}}function FE(n,t,e){const{enableBorderRadius:i}=n.getProps(["enableBorderRadius"]),s=n.options.borderRadius,r=Gi(s),o=Math.min(t,e),a=n.borderSkipped,l=i||Gt(s);return{topLeft:di(!l||a.top||a.left,r.topLeft,0,o),topRight:di(!l||a.top||a.right,r.topRight,0,o),bottomLeft:di(!l||a.bottom||a.left,r.bottomLeft,0,o),bottomRight:di(!l||a.bottom||a.right,r.bottomRight,0,o)}}function kE(n){const t=Hp(n),e=t.right-t.left,i=t.bottom-t.top,s=NE(n,e/2,i/2),r=FE(n,e/2,i/2);return{outer:{x:t.left,y:t.top,w:e,h:i,radius:r},inner:{x:t.left+s.l,y:t.top+s.t,w:e-s.l-s.r,h:i-s.t-s.b,radius:{topLeft:Math.max(0,r.topLeft-Math.max(s.t,s.l)),topRight:Math.max(0,r.topRight-Math.max(s.t,s.r)),bottomLeft:Math.max(0,r.bottomLeft-Math.max(s.b,s.l)),bottomRight:Math.max(0,r.bottomRight-Math.max(s.b,s.r))}}}}function xl(n,t,e,i){const s=t===null,r=e===null,a=n&&!(s&&r)&&Hp(n,i);return a&&(s||Yn(t,a.left,a.right))&&(r||Yn(e,a.top,a.bottom))}function BE(n){return n.topLeft||n.topRight||n.bottomLeft||n.bottomRight}function zE(n,t){n.rect(t.x,t.y,t.w,t.h)}function vl(n,t,e={}){const i=n.x!==e.x?-t:0,s=n.y!==e.y?-t:0,r=(n.x+n.w!==e.x+e.w?t:0)-i,o=(n.y+n.h!==e.y+e.h?t:0)-s;return{x:n.x+i,y:n.y+s,w:n.w+r,h:n.h+o,radius:n.radius}}class qo extends bn{constructor(t){super(),this.options=void 0,this.horizontal=void 0,this.base=void 0,this.width=void 0,this.height=void 0,this.inflateAmount=void 0,t&&Object.assign(this,t)}draw(t){const{inflateAmount:e,options:{borderColor:i,backgroundColor:s}}=this,{inner:r,outer:o}=kE(this),a=BE(o.radius)?wr:zE;t.save(),(o.w!==r.w||o.h!==r.h)&&(t.beginPath(),a(t,vl(o,e,r)),t.clip(),a(t,vl(r,-e,o)),t.fillStyle=i,t.fill("evenodd")),t.beginPath(),a(t,vl(r,e)),t.fillStyle=s,t.fill(),t.restore()}inRange(t,e,i){return xl(this,t,e,i)}inXRange(t,e){return xl(this,t,null,e)}inYRange(t,e){return xl(this,null,t,e)}getCenterPoint(t){const{x:e,y:i,base:s,horizontal:r}=this.getProps(["x","y","base","horizontal"],t);return{x:r?(e+s)/2:e,y:r?i:(i+s)/2}}getRange(t){return t==="x"?this.width/2:this.height/2}}st(qo,"id","bar"),st(qo,"defaults",{borderSkipped:"start",borderWidth:0,borderRadius:0,inflateAmount:"auto",pointStyle:void 0}),st(qo,"defaultRoutes",{backgroundColor:"backgroundColor",borderColor:"borderColor"});var HE=Object.freeze({__proto__:null,ArcElement:ur,BarElement:qo,LineElement:ui,PointElement:Yo});const yc=["rgb(54, 162, 235)","rgb(255, 99, 132)","rgb(255, 159, 64)","rgb(255, 205, 86)","rgb(75, 192, 192)","rgb(153, 102, 255)","rgb(201, 203, 207)"],Id=yc.map(n=>n.replace("rgb(","rgba(").replace(")",", 0.5)"));function Vp(n){return yc[n%yc.length]}function Gp(n){return Id[n%Id.length]}function VE(n,t){return n.borderColor=Vp(t),n.backgroundColor=Gp(t),++t}function GE(n,t){return n.backgroundColor=n.data.map(()=>Vp(t++)),t}function WE(n,t){return n.backgroundColor=n.data.map(()=>Gp(t++)),t}function XE(n){let t=0;return(e,i)=>{const s=n.getDatasetMeta(i).controller;s instanceof Hi?t=GE(e,t):s instanceof vr?t=WE(e,t):s&&(t=VE(e,t))}}function Od(n){let t;for(t in n)if(n[t].borderColor||n[t].backgroundColor)return!0;return!1}function jE(n){return n&&(n.borderColor||n.backgroundColor)}function $E(){return ge.borderColor!=="rgba(0,0,0,0.1)"||ge.backgroundColor!=="rgba(0,0,0,0.1)"}var YE={id:"colors",defaults:{enabled:!0,forceOverride:!1},beforeLayout(n,t,e){if(!e.enabled)return;const{data:{datasets:i},options:s}=n.config,{elements:r}=s,o=Od(i)||jE(s)||r&&Od(r)||$E();if(!e.forceOverride&&o)return;const a=XE(n);i.forEach(a)}};function qE(n,t,e,i,s){const r=s.samples||i;if(r>=e)return n.slice(t,t+e);const o=[],a=(e-2)/(r-2);let l=0;const c=t+e-1;let h=t,u,d,f,g,_;for(o[l++]=n[h],u=0;u<r-2;u++){let m=0,p=0,v;const y=Math.floor((u+1)*a)+1+t,x=Math.min(Math.floor((u+2)*a)+1,e)+t,A=x-y;for(v=y;v<x;v++)m+=n[v].x,p+=n[v].y;m/=A,p/=A;const w=Math.floor(u*a)+1+t,T=Math.min(Math.floor((u+1)*a)+1,e)+t,{x:R,y:S}=n[h];for(f=g=-1,v=w;v<T;v++)g=.5*Math.abs((R-m)*(n[v].y-S)-(R-n[v].x)*(p-S)),g>f&&(f=g,d=n[v],_=v);o[l++]=d,h=_}return o[l++]=n[c],o}function KE(n,t,e,i){let s=0,r=0,o,a,l,c,h,u,d,f,g,_;const m=[],p=t+e-1,v=n[t].x,x=n[p].x-v;for(o=t;o<t+e;++o){a=n[o],l=(a.x-v)/x*i,c=a.y;const A=l|0;if(A===h)c<g?(g=c,u=o):c>_&&(_=c,d=o),s=(r*s+a.x)/++r;else{const w=o-1;if(!Zt(u)&&!Zt(d)){const T=Math.min(u,d),R=Math.max(u,d);T!==f&&T!==w&&m.push({...n[T],x:s}),R!==f&&R!==w&&m.push({...n[R],x:s})}o>0&&w!==f&&m.push(n[w]),m.push(a),h=A,r=0,g=_=c,u=d=f=o}}return m}function Wp(n){if(n._decimated){const t=n._data;delete n._decimated,delete n._data,Object.defineProperty(n,"data",{configurable:!0,enumerable:!0,writable:!0,value:t})}}function Ud(n){n.data.datasets.forEach(t=>{Wp(t)})}function ZE(n,t){const e=t.length;let i=0,s;const{iScale:r}=n,{min:o,max:a,minDefined:l,maxDefined:c}=r.getUserBounds();return l&&(i=Oe(qn(t,r.axis,o).lo,0,e-1)),c?s=Oe(qn(t,r.axis,a).hi+1,i,e)-i:s=e-i,{start:i,count:s}}var JE={id:"decimation",defaults:{algorithm:"min-max",enabled:!1},beforeElementsUpdate:(n,t,e)=>{if(!e.enabled){Ud(n);return}const i=n.width;n.data.datasets.forEach((s,r)=>{const{_data:o,indexAxis:a}=s,l=n.getDatasetMeta(r),c=o||s.data;if(cr([a,n.options.indexAxis])==="y"||!l.controller.supportsDecimation)return;const h=n.scales[l.xAxisID];if(h.type!=="linear"&&h.type!=="time"||n.options.parsing)return;let{start:u,count:d}=ZE(l,c);const f=e.threshold||4*i;if(d<=f){Wp(s);return}Zt(o)&&(s._data=c,delete s.data,Object.defineProperty(s,"data",{configurable:!0,enumerable:!0,get:function(){return this._decimated},set:function(_){this._data=_}}));let g;switch(e.algorithm){case"lttb":g=qE(c,u,d,i,e);break;case"min-max":g=KE(c,u,d,i);break;default:throw new Error(`Unsupported decimation algorithm '${e.algorithm}'`)}s._decimated=g})},destroy(n){Ud(n)}};function QE(n,t,e){const i=n.segments,s=n.points,r=t.points,o=[];for(const a of i){let{start:l,end:c}=a;c=rh(l,c,s);const h=Mc(e,s[l],s[c],a.loop);if(!t.segments){o.push({source:a,target:h,start:s[l],end:s[c]});continue}const u=wp(t,h);for(const d of u){const f=Mc(e,r[d.start],r[d.end],d.loop),g=Tp(a,s,f);for(const _ of g)o.push({source:_,target:d,start:{[e]:Nd(h,f,"start",Math.max)},end:{[e]:Nd(h,f,"end",Math.min)}})}}return o}function Mc(n,t,e,i){if(i)return;let s=t[n],r=e[n];return n==="angle"&&(s=tn(s),r=tn(r)),{property:n,start:s,end:r}}function tT(n,t){const{x:e=null,y:i=null}=n||{},s=t.points,r=[];return t.segments.forEach(({start:o,end:a})=>{a=rh(o,a,s);const l=s[o],c=s[a];i!==null?(r.push({x:l.x,y:i}),r.push({x:c.x,y:i})):e!==null&&(r.push({x:e,y:l.y}),r.push({x:e,y:c.y}))}),r}function rh(n,t,e){for(;t>n;t--){const i=e[t];if(!isNaN(i.x)&&!isNaN(i.y))break}return t}function Nd(n,t,e,i){return n&&t?i(n[e],t[e]):n?n[e]:t?t[e]:0}function Xp(n,t){let e=[],i=!1;return me(n)?(i=!0,e=n):e=tT(n,t),e.length?new ui({points:e,options:{tension:0},_loop:i,_fullLoop:i}):null}function Fd(n){return n&&n.fill!==!1}function eT(n,t,e){let s=n[t].fill;const r=[t];let o;if(!e)return s;for(;s!==!1&&r.indexOf(s)===-1;){if(!ye(s))return s;if(o=n[s],!o)return!1;if(o.visible)return s;r.push(s),s=o.fill}return!1}function nT(n,t,e){const i=oT(n);if(Gt(i))return isNaN(i.value)?!1:i;let s=parseFloat(i);return ye(s)&&Math.floor(s)===s?iT(i[0],t,s,e):["origin","start","end","stack","shape"].indexOf(i)>=0&&i}function iT(n,t,e,i){return(n==="-"||n==="+")&&(e=t+e),e===t||e<0||e>=i?!1:e}function sT(n,t){let e=null;return n==="start"?e=t.bottom:n==="end"?e=t.top:Gt(n)?e=t.getPixelForValue(n.value):t.getBasePixel&&(e=t.getBasePixel()),e}function rT(n,t,e){let i;return n==="start"?i=e:n==="end"?i=t.options.reverse?t.min:t.max:Gt(n)?i=n.value:i=t.getBaseValue(),i}function oT(n){const t=n.options,e=t.fill;let i=Ot(e&&e.target,e);return i===void 0&&(i=!!t.backgroundColor),i===!1||i===null?!1:i===!0?"origin":i}function aT(n){const{scale:t,index:e,line:i}=n,s=[],r=i.segments,o=i.points,a=lT(t,e);a.push(Xp({x:null,y:t.bottom},i));for(let l=0;l<r.length;l++){const c=r[l];for(let h=c.start;h<=c.end;h++)cT(s,o[h],a)}return new ui({points:s,options:{}})}function lT(n,t){const e=[],i=n.getMatchingVisibleMetas("line");for(let s=0;s<i.length;s++){const r=i[s];if(r.index===t)break;r.hidden||e.unshift(r.dataset)}return e}function cT(n,t,e){const i=[];for(let s=0;s<e.length;s++){const r=e[s],{first:o,last:a,point:l}=hT(r,t,"x");if(!(!l||o&&a)){if(o)i.unshift(l);else if(n.push(l),!a)break}}n.push(...i)}function hT(n,t,e){const i=n.interpolate(t,e);if(!i)return{};const s=i[e],r=n.segments,o=n.points;let a=!1,l=!1;for(let c=0;c<r.length;c++){const h=r[c],u=o[h.start][e],d=o[h.end][e];if(Yn(s,u,d)){a=s===u,l=s===d;break}}return{first:a,last:l,point:i}}class jp{constructor(t){this.x=t.x,this.y=t.y,this.radius=t.radius}pathSegment(t,e,i){const{x:s,y:r,radius:o}=this;return e=e||{start:0,end:fe},t.arc(s,r,o,e.end,e.start,!0),!i.bounds}interpolate(t){const{x:e,y:i,radius:s}=this,r=t.angle;return{x:e+Math.cos(r)*s,y:i+Math.sin(r)*s,angle:r}}}function uT(n){const{chart:t,fill:e,line:i}=n;if(ye(e))return dT(t,e);if(e==="stack")return aT(n);if(e==="shape")return!0;const s=fT(n);return s instanceof jp?s:Xp(s,i)}function dT(n,t){const e=n.getDatasetMeta(t);return e&&n.isDatasetVisible(t)?e.dataset:null}function fT(n){return(n.scale||{}).getPointPositionForValue?mT(n):pT(n)}function pT(n){const{scale:t={},fill:e}=n,i=sT(e,t);if(ye(i)){const s=t.isHorizontal();return{x:s?i:null,y:s?null:i}}return null}function mT(n){const{scale:t,fill:e}=n,i=t.options,s=t.getLabels().length,r=i.reverse?t.max:t.min,o=rT(e,t,r),a=[];if(i.grid.circular){const l=t.getPointPositionForValue(0,r);return new jp({x:l.x,y:l.y,radius:t.getDistanceFromCenterForValue(o)})}for(let l=0;l<s;++l)a.push(t.getPointPositionForValue(l,o));return a}function bl(n,t,e){const i=uT(t),{line:s,scale:r,axis:o}=t,a=s.options,l=a.fill,c=a.backgroundColor,{above:h=c,below:u=c}=l||{};i&&s.points.length&&(ma(n,e),gT(n,{line:s,target:i,above:h,below:u,area:e,scale:r,axis:o}),ga(n))}function gT(n,t){const{line:e,target:i,above:s,below:r,area:o,scale:a}=t,l=e._loop?"angle":t.axis;n.save(),l==="x"&&r!==s&&(kd(n,i,o.top),Bd(n,{line:e,target:i,color:s,scale:a,property:l}),n.restore(),n.save(),kd(n,i,o.bottom)),Bd(n,{line:e,target:i,color:r,scale:a,property:l}),n.restore()}function kd(n,t,e){const{segments:i,points:s}=t;let r=!0,o=!1;n.beginPath();for(const a of i){const{start:l,end:c}=a,h=s[l],u=s[rh(l,c,s)];r?(n.moveTo(h.x,h.y),r=!1):(n.lineTo(h.x,e),n.lineTo(h.x,h.y)),o=!!t.pathSegment(n,a,{move:o}),o?n.closePath():n.lineTo(u.x,e)}n.lineTo(t.first().x,e),n.closePath(),n.clip()}function Bd(n,t){const{line:e,target:i,property:s,color:r,scale:o}=t,a=QE(e,i,s);for(const{source:l,target:c,start:h,end:u}of a){const{style:{backgroundColor:d=r}={}}=l,f=i!==!0;n.save(),n.fillStyle=d,_T(n,o,f&&Mc(s,h,u)),n.beginPath();const g=!!e.pathSegment(n,l);let _;if(f){g?n.closePath():zd(n,i,u,s);const m=!!i.pathSegment(n,c,{move:g,reverse:!0});_=g&&m,_||zd(n,i,h,s)}n.closePath(),n.fill(_?"evenodd":"nonzero"),n.restore()}}function _T(n,t,e){const{top:i,bottom:s}=t.chart.chartArea,{property:r,start:o,end:a}=e||{};r==="x"&&(n.beginPath(),n.rect(o,i,a-o,s-i),n.clip())}function zd(n,t,e,i){const s=t.interpolate(e,i);s&&n.lineTo(s.x,s.y)}var xT={id:"filler",afterDatasetsUpdate(n,t,e){const i=(n.data.datasets||[]).length,s=[];let r,o,a,l;for(o=0;o<i;++o)r=n.getDatasetMeta(o),a=r.dataset,l=null,a&&a.options&&a instanceof ui&&(l={visible:n.isDatasetVisible(o),index:o,fill:nT(a,o,i),chart:n,axis:r.controller.options.indexAxis,scale:r.vScale,line:a}),r.$filler=l,s.push(l);for(o=0;o<i;++o)l=s[o],!(!l||l.fill===!1)&&(l.fill=eT(s,o,e.propagate))},beforeDraw(n,t,e){const i=e.drawTime==="beforeDraw",s=n.getSortedVisibleDatasetMetas(),r=n.chartArea;for(let o=s.length-1;o>=0;--o){const a=s[o].$filler;a&&(a.line.updateControlPoints(r,a.axis),i&&a.fill&&bl(n.ctx,a,r))}},beforeDatasetsDraw(n,t,e){if(e.drawTime!=="beforeDatasetsDraw")return;const i=n.getSortedVisibleDatasetMetas();for(let s=i.length-1;s>=0;--s){const r=i[s].$filler;Fd(r)&&bl(n.ctx,r,n.chartArea)}},beforeDatasetDraw(n,t,e){const i=t.meta.$filler;!Fd(i)||e.drawTime!=="beforeDatasetDraw"||bl(n.ctx,i,n.chartArea)},defaults:{propagate:!0,drawTime:"beforeDatasetDraw"}};const Hd=(n,t)=>{let{boxHeight:e=t,boxWidth:i=t}=n;return n.usePointStyle&&(e=Math.min(e,t),i=n.pointStyleWidth||Math.min(i,t)),{boxWidth:i,boxHeight:e,itemHeight:Math.max(t,e)}},vT=(n,t)=>n!==null&&t!==null&&n.datasetIndex===t.datasetIndex&&n.index===t.index;class Vd extends bn{constructor(t){super(),this._added=!1,this.legendHitBoxes=[],this._hoveredItem=null,this.doughnutMode=!1,this.chart=t.chart,this.options=t.options,this.ctx=t.ctx,this.legendItems=void 0,this.columnSizes=void 0,this.lineWidths=void 0,this.maxHeight=void 0,this.maxWidth=void 0,this.top=void 0,this.bottom=void 0,this.left=void 0,this.right=void 0,this.height=void 0,this.width=void 0,this._margins=void 0,this.position=void 0,this.weight=void 0,this.fullSize=void 0}update(t,e,i){this.maxWidth=t,this.maxHeight=e,this._margins=i,this.setDimensions(),this.buildLabels(),this.fit()}setDimensions(){this.isHorizontal()?(this.width=this.maxWidth,this.left=this._margins.left,this.right=this.width):(this.height=this.maxHeight,this.top=this._margins.top,this.bottom=this.height)}buildLabels(){const t=this.options.labels||{};let e=ce(t.generateLabels,[this.chart],this)||[];t.filter&&(e=e.filter(i=>t.filter(i,this.chart.data))),t.sort&&(e=e.sort((i,s)=>t.sort(i,s,this.chart.data))),this.options.reverse&&e.reverse(),this.legendItems=e}fit(){const{options:t,ctx:e}=this;if(!t.display){this.width=this.height=0;return}const i=t.labels,s=Pe(i.font),r=s.size,o=this._computeTitleHeight(),{boxWidth:a,itemHeight:l}=Hd(i,r);let c,h;e.font=s.string,this.isHorizontal()?(c=this.maxWidth,h=this._fitRows(o,r,a,l)+10):(h=this.maxHeight,c=this._fitCols(o,s,a,l)+10),this.width=Math.min(c,t.maxWidth||this.maxWidth),this.height=Math.min(h,t.maxHeight||this.maxHeight)}_fitRows(t,e,i,s){const{ctx:r,maxWidth:o,options:{labels:{padding:a}}}=this,l=this.legendHitBoxes=[],c=this.lineWidths=[0],h=s+a;let u=t;r.textAlign="left",r.textBaseline="middle";let d=-1,f=-h;return this.legendItems.forEach((g,_)=>{const m=i+e/2+r.measureText(g.text).width;(_===0||c[c.length-1]+m+2*a>o)&&(u+=h,c[c.length-(_>0?0:1)]=0,f+=h,d++),l[_]={left:0,top:f,row:d,width:m,height:s},c[c.length-1]+=m+a}),u}_fitCols(t,e,i,s){const{ctx:r,maxHeight:o,options:{labels:{padding:a}}}=this,l=this.legendHitBoxes=[],c=this.columnSizes=[],h=o-t;let u=a,d=0,f=0,g=0,_=0;return this.legendItems.forEach((m,p)=>{const{itemWidth:v,itemHeight:y}=bT(i,e,r,m,s);p>0&&f+y+2*a>h&&(u+=d+a,c.push({width:d,height:f}),g+=d+a,_++,d=f=0),l[p]={left:g,top:f,col:_,width:v,height:y},d=Math.max(d,v),f+=y+a}),u+=d,c.push({width:d,height:f}),u}adjustHitBoxes(){if(!this.options.display)return;const t=this._computeTitleHeight(),{legendHitBoxes:e,options:{align:i,labels:{padding:s},rtl:r}}=this,o=Rs(r,this.left,this.width);if(this.isHorizontal()){let a=0,l=ke(i,this.left+s,this.right-this.lineWidths[a]);for(const c of e)a!==c.row&&(a=c.row,l=ke(i,this.left+s,this.right-this.lineWidths[a])),c.top+=this.top+t+s,c.left=o.leftForLtr(o.x(l),c.width),l+=c.width+s}else{let a=0,l=ke(i,this.top+t+s,this.bottom-this.columnSizes[a].height);for(const c of e)c.col!==a&&(a=c.col,l=ke(i,this.top+t+s,this.bottom-this.columnSizes[a].height)),c.top=l,c.left+=this.left+s,c.left=o.leftForLtr(o.x(c.left),c.width),l+=c.height+s}}isHorizontal(){return this.options.position==="top"||this.options.position==="bottom"}draw(){if(this.options.display){const t=this.ctx;ma(t,this),this._draw(),ga(t)}}_draw(){const{options:t,columnSizes:e,lineWidths:i,ctx:s}=this,{align:r,labels:o}=t,a=ge.color,l=Rs(t.rtl,this.left,this.width),c=Pe(o.font),{padding:h}=o,u=c.size,d=u/2;let f;this.drawTitle(),s.textAlign=l.textAlign("left"),s.textBaseline="middle",s.lineWidth=.5,s.font=c.string;const{boxWidth:g,boxHeight:_,itemHeight:m}=Hd(o,u),p=function(w,T,R){if(isNaN(g)||g<=0||isNaN(_)||_<0)return;s.save();const S=Ot(R.lineWidth,1);if(s.fillStyle=Ot(R.fillStyle,a),s.lineCap=Ot(R.lineCap,"butt"),s.lineDashOffset=Ot(R.lineDashOffset,0),s.lineJoin=Ot(R.lineJoin,"miter"),s.lineWidth=S,s.strokeStyle=Ot(R.strokeStyle,a),s.setLineDash(Ot(R.lineDash,[])),o.usePointStyle){const b={radius:_*Math.SQRT2/2,pointStyle:R.pointStyle,rotation:R.rotation,borderWidth:S},P=l.xPlus(w,g/2),N=T+d;pp(s,b,P,N,o.pointStyleWidth&&g)}else{const b=T+Math.max((u-_)/2,0),P=l.leftForLtr(w,g),N=Gi(R.borderRadius);s.beginPath(),Object.values(N).some(F=>F!==0)?wr(s,{x:P,y:b,w:g,h:_,radius:N}):s.rect(P,b,g,_),s.fill(),S!==0&&s.stroke()}s.restore()},v=function(w,T,R){qi(s,R.text,w,T+m/2,c,{strikethrough:R.hidden,textAlign:l.textAlign(R.textAlign)})},y=this.isHorizontal(),x=this._computeTitleHeight();y?f={x:ke(r,this.left+h,this.right-i[0]),y:this.top+h+x,line:0}:f={x:this.left+h,y:ke(r,this.top+x+h,this.bottom-e[0].height),line:0},Mp(this.ctx,t.textDirection);const A=m+h;this.legendItems.forEach((w,T)=>{s.strokeStyle=w.fontColor,s.fillStyle=w.fontColor;const R=s.measureText(w.text).width,S=l.textAlign(w.textAlign||(w.textAlign=o.textAlign)),b=g+d+R;let P=f.x,N=f.y;l.setWidth(this.width),y?T>0&&P+b+h>this.right&&(N=f.y+=A,f.line++,P=f.x=ke(r,this.left+h,this.right-i[f.line])):T>0&&N+A>this.bottom&&(P=f.x=P+e[f.line].width+h,f.line++,N=f.y=ke(r,this.top+x+h,this.bottom-e[f.line].height));const F=l.x(P);if(p(F,N,w),P=zy(S,P+g+d,y?P+b:this.right,t.rtl),v(l.x(P),N,w),y)f.x+=b+h;else if(typeof w.text!="string"){const W=c.lineHeight;f.y+=$p(w,W)+h}else f.y+=A}),Sp(this.ctx,t.textDirection)}drawTitle(){const t=this.options,e=t.title,i=Pe(e.font),s=ze(e.padding);if(!e.display)return;const r=Rs(t.rtl,this.left,this.width),o=this.ctx,a=e.position,l=i.size/2,c=s.top+l;let h,u=this.left,d=this.width;if(this.isHorizontal())d=Math.max(...this.lineWidths),h=this.top+c,u=ke(t.align,u,this.right-d);else{const g=this.columnSizes.reduce((_,m)=>Math.max(_,m.height),0);h=c+ke(t.align,this.top,this.bottom-g-t.labels.padding-this._computeTitleHeight())}const f=ke(a,u,u+d);o.textAlign=r.textAlign(Kc(a)),o.textBaseline="middle",o.strokeStyle=e.color,o.fillStyle=e.color,o.font=i.string,qi(o,e.text,f,h,i)}_computeTitleHeight(){const t=this.options.title,e=Pe(t.font),i=ze(t.padding);return t.display?e.lineHeight+i.height:0}_getLegendItemAt(t,e){let i,s,r;if(Yn(t,this.left,this.right)&&Yn(e,this.top,this.bottom)){for(r=this.legendHitBoxes,i=0;i<r.length;++i)if(s=r[i],Yn(t,s.left,s.left+s.width)&&Yn(e,s.top,s.top+s.height))return this.legendItems[i]}return null}handleEvent(t){const e=this.options;if(!ST(t.type,e))return;const i=this._getLegendItemAt(t.x,t.y);if(t.type==="mousemove"||t.type==="mouseout"){const s=this._hoveredItem,r=vT(s,i);s&&!r&&ce(e.onLeave,[t,s,this],this),this._hoveredItem=i,i&&!r&&ce(e.onHover,[t,i,this],this)}else i&&ce(e.onClick,[t,i,this],this)}}function bT(n,t,e,i,s){const r=yT(i,n,t,e),o=MT(s,i,t.lineHeight);return{itemWidth:r,itemHeight:o}}function yT(n,t,e,i){let s=n.text;return s&&typeof s!="string"&&(s=s.reduce((r,o)=>r.length>o.length?r:o)),t+e.size/2+i.measureText(s).width}function MT(n,t,e){let i=n;return typeof t.text!="string"&&(i=$p(t,e)),i}function $p(n,t){const e=n.text?n.text.length:0;return t*e}function ST(n,t){return!!((n==="mousemove"||n==="mouseout")&&(t.onHover||t.onLeave)||t.onClick&&(n==="click"||n==="mouseup"))}var ET={id:"legend",_element:Vd,start(n,t,e){const i=n.legend=new Vd({ctx:n.ctx,options:e,chart:n});Be.configure(n,i,e),Be.addBox(n,i)},stop(n){Be.removeBox(n,n.legend),delete n.legend},beforeUpdate(n,t,e){const i=n.legend;Be.configure(n,i,e),i.options=e},afterUpdate(n){const t=n.legend;t.buildLabels(),t.adjustHitBoxes()},afterEvent(n,t){t.replay||n.legend.handleEvent(t.event)},defaults:{display:!0,position:"top",align:"center",fullSize:!0,reverse:!1,weight:1e3,onClick(n,t,e){const i=t.datasetIndex,s=e.chart;s.isDatasetVisible(i)?(s.hide(i),t.hidden=!0):(s.show(i),t.hidden=!1)},onHover:null,onLeave:null,labels:{color:n=>n.chart.options.color,boxWidth:40,padding:10,generateLabels(n){const t=n.data.datasets,{labels:{usePointStyle:e,pointStyle:i,textAlign:s,color:r,useBorderRadius:o,borderRadius:a}}=n.legend.options;return n._getSortedDatasetMetas().map(l=>{const c=l.controller.getStyle(e?0:void 0),h=ze(c.borderWidth);return{text:t[l.index].label,fillStyle:c.backgroundColor,fontColor:r,hidden:!l.visible,lineCap:c.borderCapStyle,lineDash:c.borderDash,lineDashOffset:c.borderDashOffset,lineJoin:c.borderJoinStyle,lineWidth:(h.width+h.height)/4,strokeStyle:c.borderColor,pointStyle:i||c.pointStyle,rotation:c.rotation,textAlign:s||c.textAlign,borderRadius:o&&(a||c.borderRadius),datasetIndex:l.index}},this)}},title:{color:n=>n.chart.options.color,display:!1,position:"center",text:""}},descriptors:{_scriptable:n=>!n.startsWith("on"),labels:{_scriptable:n=>!["generateLabels","filter","sort"].includes(n)}}};class oh extends bn{constructor(t){super(),this.chart=t.chart,this.options=t.options,this.ctx=t.ctx,this._padding=void 0,this.top=void 0,this.bottom=void 0,this.left=void 0,this.right=void 0,this.width=void 0,this.height=void 0,this.position=void 0,this.weight=void 0,this.fullSize=void 0}update(t,e){const i=this.options;if(this.left=0,this.top=0,!i.display){this.width=this.height=this.right=this.bottom=0;return}this.width=this.right=t,this.height=this.bottom=e;const s=me(i.text)?i.text.length:1;this._padding=ze(i.padding);const r=s*Pe(i.font).lineHeight+this._padding.height;this.isHorizontal()?this.height=r:this.width=r}isHorizontal(){const t=this.options.position;return t==="top"||t==="bottom"}_drawArgs(t){const{top:e,left:i,bottom:s,right:r,options:o}=this,a=o.align;let l=0,c,h,u;return this.isHorizontal()?(h=ke(a,i,r),u=e+t,c=r-i):(o.position==="left"?(h=i+t,u=ke(a,s,e),l=pe*-.5):(h=r-t,u=ke(a,e,s),l=pe*.5),c=s-e),{titleX:h,titleY:u,maxWidth:c,rotation:l}}draw(){const t=this.ctx,e=this.options;if(!e.display)return;const i=Pe(e.font),r=i.lineHeight/2+this._padding.top,{titleX:o,titleY:a,maxWidth:l,rotation:c}=this._drawArgs(r);qi(t,e.text,0,0,i,{color:e.color,maxWidth:l,rotation:c,textAlign:Kc(e.align),textBaseline:"middle",translation:[o,a]})}}function TT(n,t){const e=new oh({ctx:n.ctx,options:t,chart:n});Be.configure(n,e,t),Be.addBox(n,e),n.titleBlock=e}var wT={id:"title",_element:oh,start(n,t,e){TT(n,e)},stop(n){const t=n.titleBlock;Be.removeBox(n,t),delete n.titleBlock},beforeUpdate(n,t,e){const i=n.titleBlock;Be.configure(n,i,e),i.options=e},defaults:{align:"center",display:!1,font:{weight:"bold"},fullSize:!0,padding:10,position:"top",text:"",weight:2e3},defaultRoutes:{color:"color"},descriptors:{_scriptable:!0,_indexable:!1}};const Po=new WeakMap;var AT={id:"subtitle",start(n,t,e){const i=new oh({ctx:n.ctx,options:e,chart:n});Be.configure(n,i,e),Be.addBox(n,i),Po.set(n,i)},stop(n){Be.removeBox(n,Po.get(n)),Po.delete(n)},beforeUpdate(n,t,e){const i=Po.get(n);Be.configure(n,i,e),i.options=e},defaults:{align:"center",display:!1,font:{weight:"normal"},fullSize:!0,padding:0,position:"top",text:"",weight:1500},defaultRoutes:{color:"color"},descriptors:{_scriptable:!0,_indexable:!1}};const dr={average(n){if(!n.length)return!1;let t,e,i=new Set,s=0,r=0;for(t=0,e=n.length;t<e;++t){const a=n[t].element;if(a&&a.hasValue()){const l=a.tooltipPosition();i.add(l.x),s+=l.y,++r}}return r===0||i.size===0?!1:{x:[...i].reduce((a,l)=>a+l)/i.size,y:s/r}},nearest(n,t){if(!n.length)return!1;let e=t.x,i=t.y,s=Number.POSITIVE_INFINITY,r,o,a;for(r=0,o=n.length;r<o;++r){const l=n[r].element;if(l&&l.hasValue()){const c=l.getCenterPoint(),h=pc(t,c);h<s&&(s=h,a=l)}}if(a){const l=a.tooltipPosition();e=l.x,i=l.y}return{x:e,y:i}}};function Mn(n,t){return t&&(me(t)?Array.prototype.push.apply(n,t):n.push(t)),n}function Vn(n){return(typeof n=="string"||n instanceof String)&&n.indexOf(`
`)>-1?n.split(`
`):n}function CT(n,t){const{element:e,datasetIndex:i,index:s}=t,r=n.getDatasetMeta(i).controller,{label:o,value:a}=r.getLabelAndValue(s);return{chart:n,label:o,parsed:r.getParsed(s),raw:n.data.datasets[i].data[s],formattedValue:a,dataset:r.getDataset(),dataIndex:s,datasetIndex:i,element:e}}function Gd(n,t){const e=n.chart.ctx,{body:i,footer:s,title:r}=n,{boxWidth:o,boxHeight:a}=t,l=Pe(t.bodyFont),c=Pe(t.titleFont),h=Pe(t.footerFont),u=r.length,d=s.length,f=i.length,g=ze(t.padding);let _=g.height,m=0,p=i.reduce((x,A)=>x+A.before.length+A.lines.length+A.after.length,0);if(p+=n.beforeBody.length+n.afterBody.length,u&&(_+=u*c.lineHeight+(u-1)*t.titleSpacing+t.titleMarginBottom),p){const x=t.displayColors?Math.max(a,l.lineHeight):l.lineHeight;_+=f*x+(p-f)*l.lineHeight+(p-1)*t.bodySpacing}d&&(_+=t.footerMarginTop+d*h.lineHeight+(d-1)*t.footerSpacing);let v=0;const y=function(x){m=Math.max(m,e.measureText(x).width+v)};return e.save(),e.font=c.string,ne(n.title,y),e.font=l.string,ne(n.beforeBody.concat(n.afterBody),y),v=t.displayColors?o+2+t.boxPadding:0,ne(i,x=>{ne(x.before,y),ne(x.lines,y),ne(x.after,y)}),v=0,e.font=h.string,ne(n.footer,y),e.restore(),m+=g.width,{width:m,height:_}}function RT(n,t){const{y:e,height:i}=t;return e<i/2?"top":e>n.height-i/2?"bottom":"center"}function PT(n,t,e,i){const{x:s,width:r}=i,o=e.caretSize+e.caretPadding;if(n==="left"&&s+r+o>t.width||n==="right"&&s-r-o<0)return!0}function DT(n,t,e,i){const{x:s,width:r}=e,{width:o,chartArea:{left:a,right:l}}=n;let c="center";return i==="center"?c=s<=(a+l)/2?"left":"right":s<=r/2?c="left":s>=o-r/2&&(c="right"),PT(c,n,t,e)&&(c="center"),c}function Wd(n,t,e){const i=e.yAlign||t.yAlign||RT(n,e);return{xAlign:e.xAlign||t.xAlign||DT(n,t,e,i),yAlign:i}}function LT(n,t){let{x:e,width:i}=n;return t==="right"?e-=i:t==="center"&&(e-=i/2),e}function IT(n,t,e){let{y:i,height:s}=n;return t==="top"?i+=e:t==="bottom"?i-=s+e:i-=s/2,i}function Xd(n,t,e,i){const{caretSize:s,caretPadding:r,cornerRadius:o}=n,{xAlign:a,yAlign:l}=e,c=s+r,{topLeft:h,topRight:u,bottomLeft:d,bottomRight:f}=Gi(o);let g=LT(t,a);const _=IT(t,l,c);return l==="center"?a==="left"?g+=c:a==="right"&&(g-=c):a==="left"?g-=Math.max(h,d)+s:a==="right"&&(g+=Math.max(u,f)+s),{x:Oe(g,0,i.width-t.width),y:Oe(_,0,i.height-t.height)}}function Do(n,t,e){const i=ze(e.padding);return t==="center"?n.x+n.width/2:t==="right"?n.x+n.width-i.right:n.x+i.left}function jd(n){return Mn([],Vn(n))}function OT(n,t,e){return Mi(n,{tooltip:t,tooltipItems:e,type:"tooltip"})}function $d(n,t){const e=t&&t.dataset&&t.dataset.tooltip&&t.dataset.tooltip.callbacks;return e?n.override(e):n}const Yp={beforeTitle:kn,title(n){if(n.length>0){const t=n[0],e=t.chart.data.labels,i=e?e.length:0;if(this&&this.options&&this.options.mode==="dataset")return t.dataset.label||"";if(t.label)return t.label;if(i>0&&t.dataIndex<i)return e[t.dataIndex]}return""},afterTitle:kn,beforeBody:kn,beforeLabel:kn,label(n){if(this&&this.options&&this.options.mode==="dataset")return n.label+": "+n.formattedValue||n.formattedValue;let t=n.dataset.label||"";t&&(t+=": ");const e=n.formattedValue;return Zt(e)||(t+=e),t},labelColor(n){const e=n.chart.getDatasetMeta(n.datasetIndex).controller.getStyle(n.dataIndex);return{borderColor:e.borderColor,backgroundColor:e.backgroundColor,borderWidth:e.borderWidth,borderDash:e.borderDash,borderDashOffset:e.borderDashOffset,borderRadius:0}},labelTextColor(){return this.options.bodyColor},labelPointStyle(n){const e=n.chart.getDatasetMeta(n.datasetIndex).controller.getStyle(n.dataIndex);return{pointStyle:e.pointStyle,rotation:e.rotation}},afterLabel:kn,afterBody:kn,beforeFooter:kn,footer:kn,afterFooter:kn};function $e(n,t,e,i){const s=n[t].call(e,i);return typeof s>"u"?Yp[t].call(e,i):s}class Sc extends bn{constructor(t){super(),this.opacity=0,this._active=[],this._eventPosition=void 0,this._size=void 0,this._cachedAnimations=void 0,this._tooltipItems=[],this.$animations=void 0,this.$context=void 0,this.chart=t.chart,this.options=t.options,this.dataPoints=void 0,this.title=void 0,this.beforeBody=void 0,this.body=void 0,this.afterBody=void 0,this.footer=void 0,this.xAlign=void 0,this.yAlign=void 0,this.x=void 0,this.y=void 0,this.height=void 0,this.width=void 0,this.caretX=void 0,this.caretY=void 0,this.labelColors=void 0,this.labelPointStyles=void 0,this.labelTextColors=void 0}initialize(t){this.options=t,this._cachedAnimations=void 0,this.$context=void 0}_resolveAnimations(){const t=this._cachedAnimations;if(t)return t;const e=this.chart,i=this.options.setContext(this.getContext()),s=i.enabled&&e.options.animation&&i.animations,r=new Ap(this.chart,s);return s._cacheable&&(this._cachedAnimations=Object.freeze(r)),r}getContext(){return this.$context||(this.$context=OT(this.chart.getContext(),this,this._tooltipItems))}getTitle(t,e){const{callbacks:i}=e,s=$e(i,"beforeTitle",this,t),r=$e(i,"title",this,t),o=$e(i,"afterTitle",this,t);let a=[];return a=Mn(a,Vn(s)),a=Mn(a,Vn(r)),a=Mn(a,Vn(o)),a}getBeforeBody(t,e){return jd($e(e.callbacks,"beforeBody",this,t))}getBody(t,e){const{callbacks:i}=e,s=[];return ne(t,r=>{const o={before:[],lines:[],after:[]},a=$d(i,r);Mn(o.before,Vn($e(a,"beforeLabel",this,r))),Mn(o.lines,$e(a,"label",this,r)),Mn(o.after,Vn($e(a,"afterLabel",this,r))),s.push(o)}),s}getAfterBody(t,e){return jd($e(e.callbacks,"afterBody",this,t))}getFooter(t,e){const{callbacks:i}=e,s=$e(i,"beforeFooter",this,t),r=$e(i,"footer",this,t),o=$e(i,"afterFooter",this,t);let a=[];return a=Mn(a,Vn(s)),a=Mn(a,Vn(r)),a=Mn(a,Vn(o)),a}_createItems(t){const e=this._active,i=this.chart.data,s=[],r=[],o=[];let a=[],l,c;for(l=0,c=e.length;l<c;++l)a.push(CT(this.chart,e[l]));return t.filter&&(a=a.filter((h,u,d)=>t.filter(h,u,d,i))),t.itemSort&&(a=a.sort((h,u)=>t.itemSort(h,u,i))),ne(a,h=>{const u=$d(t.callbacks,h);s.push($e(u,"labelColor",this,h)),r.push($e(u,"labelPointStyle",this,h)),o.push($e(u,"labelTextColor",this,h))}),this.labelColors=s,this.labelPointStyles=r,this.labelTextColors=o,this.dataPoints=a,a}update(t,e){const i=this.options.setContext(this.getContext()),s=this._active;let r,o=[];if(!s.length)this.opacity!==0&&(r={opacity:0});else{const a=dr[i.position].call(this,s,this._eventPosition);o=this._createItems(i),this.title=this.getTitle(o,i),this.beforeBody=this.getBeforeBody(o,i),this.body=this.getBody(o,i),this.afterBody=this.getAfterBody(o,i),this.footer=this.getFooter(o,i);const l=this._size=Gd(this,i),c=Object.assign({},a,l),h=Wd(this.chart,i,c),u=Xd(i,c,h,this.chart);this.xAlign=h.xAlign,this.yAlign=h.yAlign,r={opacity:1,x:u.x,y:u.y,width:l.width,height:l.height,caretX:a.x,caretY:a.y}}this._tooltipItems=o,this.$context=void 0,r&&this._resolveAnimations().update(this,r),t&&i.external&&i.external.call(this,{chart:this.chart,tooltip:this,replay:e})}drawCaret(t,e,i,s){const r=this.getCaretPosition(t,i,s);e.lineTo(r.x1,r.y1),e.lineTo(r.x2,r.y2),e.lineTo(r.x3,r.y3)}getCaretPosition(t,e,i){const{xAlign:s,yAlign:r}=this,{caretSize:o,cornerRadius:a}=i,{topLeft:l,topRight:c,bottomLeft:h,bottomRight:u}=Gi(a),{x:d,y:f}=t,{width:g,height:_}=e;let m,p,v,y,x,A;return r==="center"?(x=f+_/2,s==="left"?(m=d,p=m-o,y=x+o,A=x-o):(m=d+g,p=m+o,y=x-o,A=x+o),v=m):(s==="left"?p=d+Math.max(l,h)+o:s==="right"?p=d+g-Math.max(c,u)-o:p=this.caretX,r==="top"?(y=f,x=y-o,m=p-o,v=p+o):(y=f+_,x=y+o,m=p+o,v=p-o),A=y),{x1:m,x2:p,x3:v,y1:y,y2:x,y3:A}}drawTitle(t,e,i){const s=this.title,r=s.length;let o,a,l;if(r){const c=Rs(i.rtl,this.x,this.width);for(t.x=Do(this,i.titleAlign,i),e.textAlign=c.textAlign(i.titleAlign),e.textBaseline="middle",o=Pe(i.titleFont),a=i.titleSpacing,e.fillStyle=i.titleColor,e.font=o.string,l=0;l<r;++l)e.fillText(s[l],c.x(t.x),t.y+o.lineHeight/2),t.y+=o.lineHeight+a,l+1===r&&(t.y+=i.titleMarginBottom-a)}}_drawColorBox(t,e,i,s,r){const o=this.labelColors[i],a=this.labelPointStyles[i],{boxHeight:l,boxWidth:c}=r,h=Pe(r.bodyFont),u=Do(this,"left",r),d=s.x(u),f=l<h.lineHeight?(h.lineHeight-l)/2:0,g=e.y+f;if(r.usePointStyle){const _={radius:Math.min(c,l)/2,pointStyle:a.pointStyle,rotation:a.rotation,borderWidth:1},m=s.leftForLtr(d,c)+c/2,p=g+l/2;t.strokeStyle=r.multiKeyBackground,t.fillStyle=r.multiKeyBackground,gc(t,_,m,p),t.strokeStyle=o.borderColor,t.fillStyle=o.backgroundColor,gc(t,_,m,p)}else{t.lineWidth=Gt(o.borderWidth)?Math.max(...Object.values(o.borderWidth)):o.borderWidth||1,t.strokeStyle=o.borderColor,t.setLineDash(o.borderDash||[]),t.lineDashOffset=o.borderDashOffset||0;const _=s.leftForLtr(d,c),m=s.leftForLtr(s.xPlus(d,1),c-2),p=Gi(o.borderRadius);Object.values(p).some(v=>v!==0)?(t.beginPath(),t.fillStyle=r.multiKeyBackground,wr(t,{x:_,y:g,w:c,h:l,radius:p}),t.fill(),t.stroke(),t.fillStyle=o.backgroundColor,t.beginPath(),wr(t,{x:m,y:g+1,w:c-2,h:l-2,radius:p}),t.fill()):(t.fillStyle=r.multiKeyBackground,t.fillRect(_,g,c,l),t.strokeRect(_,g,c,l),t.fillStyle=o.backgroundColor,t.fillRect(m,g+1,c-2,l-2))}t.fillStyle=this.labelTextColors[i]}drawBody(t,e,i){const{body:s}=this,{bodySpacing:r,bodyAlign:o,displayColors:a,boxHeight:l,boxWidth:c,boxPadding:h}=i,u=Pe(i.bodyFont);let d=u.lineHeight,f=0;const g=Rs(i.rtl,this.x,this.width),_=function(R){e.fillText(R,g.x(t.x+f),t.y+d/2),t.y+=d+r},m=g.textAlign(o);let p,v,y,x,A,w,T;for(e.textAlign=o,e.textBaseline="middle",e.font=u.string,t.x=Do(this,m,i),e.fillStyle=i.bodyColor,ne(this.beforeBody,_),f=a&&m!=="right"?o==="center"?c/2+h:c+2+h:0,x=0,w=s.length;x<w;++x){for(p=s[x],v=this.labelTextColors[x],e.fillStyle=v,ne(p.before,_),y=p.lines,a&&y.length&&(this._drawColorBox(e,t,x,g,i),d=Math.max(u.lineHeight,l)),A=0,T=y.length;A<T;++A)_(y[A]),d=u.lineHeight;ne(p.after,_)}f=0,d=u.lineHeight,ne(this.afterBody,_),t.y-=r}drawFooter(t,e,i){const s=this.footer,r=s.length;let o,a;if(r){const l=Rs(i.rtl,this.x,this.width);for(t.x=Do(this,i.footerAlign,i),t.y+=i.footerMarginTop,e.textAlign=l.textAlign(i.footerAlign),e.textBaseline="middle",o=Pe(i.footerFont),e.fillStyle=i.footerColor,e.font=o.string,a=0;a<r;++a)e.fillText(s[a],l.x(t.x),t.y+o.lineHeight/2),t.y+=o.lineHeight+i.footerSpacing}}drawBackground(t,e,i,s){const{xAlign:r,yAlign:o}=this,{x:a,y:l}=t,{width:c,height:h}=i,{topLeft:u,topRight:d,bottomLeft:f,bottomRight:g}=Gi(s.cornerRadius);e.fillStyle=s.backgroundColor,e.strokeStyle=s.borderColor,e.lineWidth=s.borderWidth,e.beginPath(),e.moveTo(a+u,l),o==="top"&&this.drawCaret(t,e,i,s),e.lineTo(a+c-d,l),e.quadraticCurveTo(a+c,l,a+c,l+d),o==="center"&&r==="right"&&this.drawCaret(t,e,i,s),e.lineTo(a+c,l+h-g),e.quadraticCurveTo(a+c,l+h,a+c-g,l+h),o==="bottom"&&this.drawCaret(t,e,i,s),e.lineTo(a+f,l+h),e.quadraticCurveTo(a,l+h,a,l+h-f),o==="center"&&r==="left"&&this.drawCaret(t,e,i,s),e.lineTo(a,l+u),e.quadraticCurveTo(a,l,a+u,l),e.closePath(),e.fill(),s.borderWidth>0&&e.stroke()}_updateAnimationTarget(t){const e=this.chart,i=this.$animations,s=i&&i.x,r=i&&i.y;if(s||r){const o=dr[t.position].call(this,this._active,this._eventPosition);if(!o)return;const a=this._size=Gd(this,t),l=Object.assign({},o,this._size),c=Wd(e,t,l),h=Xd(t,l,c,e);(s._to!==h.x||r._to!==h.y)&&(this.xAlign=c.xAlign,this.yAlign=c.yAlign,this.width=a.width,this.height=a.height,this.caretX=o.x,this.caretY=o.y,this._resolveAnimations().update(this,h))}}_willRender(){return!!this.opacity}draw(t){const e=this.options.setContext(this.getContext());let i=this.opacity;if(!i)return;this._updateAnimationTarget(e);const s={width:this.width,height:this.height},r={x:this.x,y:this.y};i=Math.abs(i)<.001?0:i;const o=ze(e.padding),a=this.title.length||this.beforeBody.length||this.body.length||this.afterBody.length||this.footer.length;e.enabled&&a&&(t.save(),t.globalAlpha=i,this.drawBackground(r,t,s,e),Mp(t,e.textDirection),r.y+=o.top,this.drawTitle(r,t,e),this.drawBody(r,t,e),this.drawFooter(r,t,e),Sp(t,e.textDirection),t.restore())}getActiveElements(){return this._active||[]}setActiveElements(t,e){const i=this._active,s=t.map(({datasetIndex:a,index:l})=>{const c=this.chart.getDatasetMeta(a);if(!c)throw new Error("Cannot find a dataset at index "+a);return{datasetIndex:a,element:c.data[l],index:l}}),r=!na(i,s),o=this._positionChanged(s,e);(r||o)&&(this._active=s,this._eventPosition=e,this._ignoreReplayEvents=!0,this.update(!0))}handleEvent(t,e,i=!0){if(e&&this._ignoreReplayEvents)return!1;this._ignoreReplayEvents=!1;const s=this.options,r=this._active||[],o=this._getActiveElements(t,r,e,i),a=this._positionChanged(o,t),l=e||!na(o,r)||a;return l&&(this._active=o,(s.enabled||s.external)&&(this._eventPosition={x:t.x,y:t.y},this.update(!0,e))),l}_getActiveElements(t,e,i,s){const r=this.options;if(t.type==="mouseout")return[];if(!s)return e.filter(a=>this.chart.data.datasets[a.datasetIndex]&&this.chart.getDatasetMeta(a.datasetIndex).controller.getParsed(a.index)!==void 0);const o=this.chart.getElementsAtEventForMode(t,r.mode,r,i);return r.reverse&&o.reverse(),o}_positionChanged(t,e){const{caretX:i,caretY:s,options:r}=this,o=dr[r.position].call(this,t,e);return o!==!1&&(i!==o.x||s!==o.y)}}st(Sc,"positioners",dr);var UT={id:"tooltip",_element:Sc,positioners:dr,afterInit(n,t,e){e&&(n.tooltip=new Sc({chart:n,options:e}))},beforeUpdate(n,t,e){n.tooltip&&n.tooltip.initialize(e)},reset(n,t,e){n.tooltip&&n.tooltip.initialize(e)},afterDraw(n){const t=n.tooltip;if(t&&t._willRender()){const e={tooltip:t};if(n.notifyPlugins("beforeTooltipDraw",{...e,cancelable:!0})===!1)return;t.draw(n.ctx),n.notifyPlugins("afterTooltipDraw",e)}},afterEvent(n,t){if(n.tooltip){const e=t.replay;n.tooltip.handleEvent(t.event,e,t.inChartArea)&&(t.changed=!0)}},defaults:{enabled:!0,external:null,position:"average",backgroundColor:"rgba(0,0,0,0.8)",titleColor:"#fff",titleFont:{weight:"bold"},titleSpacing:2,titleMarginBottom:6,titleAlign:"left",bodyColor:"#fff",bodySpacing:2,bodyFont:{},bodyAlign:"left",footerColor:"#fff",footerSpacing:2,footerMarginTop:6,footerFont:{weight:"bold"},footerAlign:"left",padding:6,caretPadding:2,caretSize:5,cornerRadius:6,boxHeight:(n,t)=>t.bodyFont.size,boxWidth:(n,t)=>t.bodyFont.size,multiKeyBackground:"#fff",displayColors:!0,boxPadding:0,borderColor:"rgba(0,0,0,0)",borderWidth:0,animation:{duration:400,easing:"easeOutQuart"},animations:{numbers:{type:"number",properties:["x","y","width","height","caretX","caretY"]},opacity:{easing:"linear",duration:200}},callbacks:Yp},defaultRoutes:{bodyFont:"font",footerFont:"font",titleFont:"font"},descriptors:{_scriptable:n=>n!=="filter"&&n!=="itemSort"&&n!=="external",_indexable:!1,callbacks:{_scriptable:!1,_indexable:!1},animation:{_fallback:!1},animations:{_fallback:"animation"}},additionalOptionScopes:["interaction"]},NT=Object.freeze({__proto__:null,Colors:YE,Decimation:JE,Filler:xT,Legend:ET,SubTitle:AT,Title:wT,Tooltip:UT});const FT=(n,t,e,i)=>(typeof t=="string"?(e=n.push(t)-1,i.unshift({index:e,label:t})):isNaN(t)&&(e=null),e);function kT(n,t,e,i){const s=n.indexOf(t);if(s===-1)return FT(n,t,e,i);const r=n.lastIndexOf(t);return s!==r?e:s}const BT=(n,t)=>n===null?null:Oe(Math.round(n),0,t);function Yd(n){const t=this.getLabels();return n>=0&&n<t.length?t[n]:n}class Ec extends Zi{constructor(t){super(t),this._startValue=void 0,this._valueRange=0,this._addedLabels=[]}init(t){const e=this._addedLabels;if(e.length){const i=this.getLabels();for(const{index:s,label:r}of e)i[s]===r&&i.splice(s,1);this._addedLabels=[]}super.init(t)}parse(t,e){if(Zt(t))return null;const i=this.getLabels();return e=isFinite(e)&&i[e]===t?e:kT(i,t,Ot(e,t),this._addedLabels),BT(e,i.length-1)}determineDataLimits(){const{minDefined:t,maxDefined:e}=this.getUserBounds();let{min:i,max:s}=this.getMinMax(!0);this.options.bounds==="ticks"&&(t||(i=0),e||(s=this.getLabels().length-1)),this.min=i,this.max=s}buildTicks(){const t=this.min,e=this.max,i=this.options.offset,s=[];let r=this.getLabels();r=t===0&&e===r.length-1?r:r.slice(t,e+1),this._valueRange=Math.max(r.length-(i?0:1),1),this._startValue=this.min-(i?.5:0);for(let o=t;o<=e;o++)s.push({value:o});return s}getLabelForValue(t){return Yd.call(this,t)}configure(){super.configure(),this.isHorizontal()||(this._reversePixels=!this._reversePixels)}getPixelForValue(t){return typeof t!="number"&&(t=this.parse(t)),t===null?NaN:this.getPixelForDecimal((t-this._startValue)/this._valueRange)}getPixelForTick(t){const e=this.ticks;return t<0||t>e.length-1?null:this.getPixelForValue(e[t].value)}getValueForPixel(t){return Math.round(this._startValue+this.getDecimalForPixel(t)*this._valueRange)}getBasePixel(){return this.bottom}}st(Ec,"id","category"),st(Ec,"defaults",{ticks:{callback:Yd}});function zT(n,t){const e=[],{bounds:s,step:r,min:o,max:a,precision:l,count:c,maxTicks:h,maxDigits:u,includeBounds:d}=n,f=r||1,g=h-1,{min:_,max:m}=t,p=!Zt(o),v=!Zt(a),y=!Zt(c),x=(m-_)/(u+1);let A=Gu((m-_)/g/f)*f,w,T,R,S;if(A<1e-14&&!p&&!v)return[{value:_},{value:m}];S=Math.ceil(m/A)-Math.floor(_/A),S>g&&(A=Gu(S*A/g/f)*f),Zt(l)||(w=Math.pow(10,l),A=Math.ceil(A*w)/w),s==="ticks"?(T=Math.floor(_/A)*A,R=Math.ceil(m/A)*A):(T=_,R=m),p&&v&&r&&Iy((a-o)/r,A/1e3)?(S=Math.round(Math.min((a-o)/A,h)),A=(a-o)/S,T=o,R=a):y?(T=p?o:T,R=v?a:R,S=c-1,A=(R-T)/S):(S=(R-T)/A,gr(S,Math.round(S),A/1e3)?S=Math.round(S):S=Math.ceil(S));const b=Math.max(Wu(A),Wu(T));w=Math.pow(10,Zt(l)?b:l),T=Math.round(T*w)/w,R=Math.round(R*w)/w;let P=0;for(p&&(d&&T!==o?(e.push({value:o}),T<o&&P++,gr(Math.round((T+P*A)*w)/w,o,qd(o,x,n))&&P++):T<o&&P++);P<S;++P){const N=Math.round((T+P*A)*w)/w;if(v&&N>a)break;e.push({value:N})}return v&&d&&R!==a?e.length&&gr(e[e.length-1].value,a,qd(a,x,n))?e[e.length-1].value=a:e.push({value:a}):(!v||R===a)&&e.push({value:R}),e}function qd(n,t,{horizontal:e,minRotation:i}){const s=gn(i),r=(e?Math.sin(s):Math.cos(s))||.001,o=.75*t*(""+n).length;return Math.min(t/r,o)}class ca extends Zi{constructor(t){super(t),this.start=void 0,this.end=void 0,this._startValue=void 0,this._endValue=void 0,this._valueRange=0}parse(t,e){return Zt(t)||(typeof t=="number"||t instanceof Number)&&!isFinite(+t)?null:+t}handleTickRangeOptions(){const{beginAtZero:t}=this.options,{minDefined:e,maxDefined:i}=this.getUserBounds();let{min:s,max:r}=this;const o=l=>s=e?s:l,a=l=>r=i?r:l;if(t){const l=Rn(s),c=Rn(r);l<0&&c<0?a(0):l>0&&c>0&&o(0)}if(s===r){let l=r===0?1:Math.abs(r*.05);a(r+l),t||o(s-l)}this.min=s,this.max=r}getTickLimit(){const t=this.options.ticks;let{maxTicksLimit:e,stepSize:i}=t,s;return i?(s=Math.ceil(this.max/i)-Math.floor(this.min/i)+1,s>1e3&&(console.warn(`scales.${this.id}.ticks.stepSize: ${i} would result generating up to ${s} ticks. Limiting to 1000.`),s=1e3)):(s=this.computeTickLimit(),e=e||11),e&&(s=Math.min(e,s)),s}computeTickLimit(){return Number.POSITIVE_INFINITY}buildTicks(){const t=this.options,e=t.ticks;let i=this.getTickLimit();i=Math.max(2,i);const s={maxTicks:i,bounds:t.bounds,min:t.min,max:t.max,precision:e.precision,step:e.stepSize,count:e.count,maxDigits:this._maxDigits(),horizontal:this.isHorizontal(),minRotation:e.minRotation||0,includeBounds:e.includeBounds!==!1},r=this._range||this,o=zT(s,r);return t.bounds==="ticks"&&rp(o,this,"value"),t.reverse?(o.reverse(),this.start=this.max,this.end=this.min):(this.start=this.min,this.end=this.max),o}configure(){const t=this.ticks;let e=this.min,i=this.max;if(super.configure(),this.options.offset&&t.length){const s=(i-e)/Math.max(t.length-1,1)/2;e-=s,i+=s}this._startValue=e,this._endValue=i,this._valueRange=i-e}getLabelForValue(t){return kr(t,this.chart.options.locale,this.options.ticks.format)}}class Tc extends ca{determineDataLimits(){const{min:t,max:e}=this.getMinMax(!0);this.min=ye(t)?t:0,this.max=ye(e)?e:1,this.handleTickRangeOptions()}computeTickLimit(){const t=this.isHorizontal(),e=t?this.width:this.height,i=gn(this.options.ticks.minRotation),s=(t?Math.sin(i):Math.cos(i))||.001,r=this._resolveTickFontOptions(0);return Math.ceil(e/Math.min(40,r.lineHeight/s))}getPixelForValue(t){return t===null?NaN:this.getPixelForDecimal((t-this._startValue)/this._valueRange)}getValueForPixel(t){return this._startValue+this.getDecimalForPixel(t)*this._valueRange}}st(Tc,"id","linear"),st(Tc,"defaults",{ticks:{callback:pa.formatters.numeric}});const Cr=n=>Math.floor(hi(n)),Ii=(n,t)=>Math.pow(10,Cr(n)+t);function Kd(n){return n/Math.pow(10,Cr(n))===1}function Zd(n,t,e){const i=Math.pow(10,e),s=Math.floor(n/i);return Math.ceil(t/i)-s}function HT(n,t){const e=t-n;let i=Cr(e);for(;Zd(n,t,i)>10;)i++;for(;Zd(n,t,i)<10;)i--;return Math.min(i,Cr(n))}function VT(n,{min:t,max:e}){t=Qe(n.min,t);const i=[],s=Cr(t);let r=HT(t,e),o=r<0?Math.pow(10,Math.abs(r)):1;const a=Math.pow(10,r),l=s>r?Math.pow(10,s):0,c=Math.round((t-l)*o)/o,h=Math.floor((t-l)/a/10)*a*10;let u=Math.floor((c-h)/Math.pow(10,r)),d=Qe(n.min,Math.round((l+h+u*Math.pow(10,r))*o)/o);for(;d<e;)i.push({value:d,major:Kd(d),significand:u}),u>=10?u=u<15?15:20:u++,u>=20&&(r++,u=2,o=r>=0?1:o),d=Math.round((l+h+u*Math.pow(10,r))*o)/o;const f=Qe(n.max,d);return i.push({value:f,major:Kd(f),significand:u}),i}class wc extends Zi{constructor(t){super(t),this.start=void 0,this.end=void 0,this._startValue=void 0,this._valueRange=0}parse(t,e){const i=ca.prototype.parse.apply(this,[t,e]);if(i===0){this._zero=!0;return}return ye(i)&&i>0?i:null}determineDataLimits(){const{min:t,max:e}=this.getMinMax(!0);this.min=ye(t)?Math.max(0,t):null,this.max=ye(e)?Math.max(0,e):null,this.options.beginAtZero&&(this._zero=!0),this._zero&&this.min!==this._suggestedMin&&!ye(this._userMin)&&(this.min=t===Ii(this.min,0)?Ii(this.min,-1):Ii(this.min,0)),this.handleTickRangeOptions()}handleTickRangeOptions(){const{minDefined:t,maxDefined:e}=this.getUserBounds();let i=this.min,s=this.max;const r=a=>i=t?i:a,o=a=>s=e?s:a;i===s&&(i<=0?(r(1),o(10)):(r(Ii(i,-1)),o(Ii(s,1)))),i<=0&&r(Ii(s,-1)),s<=0&&o(Ii(i,1)),this.min=i,this.max=s}buildTicks(){const t=this.options,e={min:this._userMin,max:this._userMax},i=VT(e,this);return t.bounds==="ticks"&&rp(i,this,"value"),t.reverse?(i.reverse(),this.start=this.max,this.end=this.min):(this.start=this.min,this.end=this.max),i}getLabelForValue(t){return t===void 0?"0":kr(t,this.chart.options.locale,this.options.ticks.format)}configure(){const t=this.min;super.configure(),this._startValue=hi(t),this._valueRange=hi(this.max)-hi(t)}getPixelForValue(t){return(t===void 0||t===0)&&(t=this.min),t===null||isNaN(t)?NaN:this.getPixelForDecimal(t===this.min?0:(hi(t)-this._startValue)/this._valueRange)}getValueForPixel(t){const e=this.getDecimalForPixel(t);return Math.pow(10,this._startValue+e*this._valueRange)}}st(wc,"id","logarithmic"),st(wc,"defaults",{ticks:{callback:pa.formatters.logarithmic,major:{enabled:!0}}});function Ac(n){const t=n.ticks;if(t.display&&n.display){const e=ze(t.backdropPadding);return Ot(t.font&&t.font.size,ge.font.size)+e.height}return 0}function GT(n,t,e){return e=me(e)?e:[e],{w:Ky(n,t.string,e),h:e.length*t.lineHeight}}function Jd(n,t,e,i,s){return n===i||n===s?{start:t-e/2,end:t+e/2}:n<i||n>s?{start:t-e,end:t}:{start:t,end:t+e}}function WT(n){const t={l:n.left+n._padding.left,r:n.right-n._padding.right,t:n.top+n._padding.top,b:n.bottom-n._padding.bottom},e=Object.assign({},t),i=[],s=[],r=n._pointLabels.length,o=n.options.pointLabels,a=o.centerPointLabels?pe/r:0;for(let l=0;l<r;l++){const c=o.setContext(n.getPointLabelContext(l));s[l]=c.padding;const h=n.getPointPosition(l,n.drawingArea+s[l],a),u=Pe(c.font),d=GT(n.ctx,u,n._pointLabels[l]);i[l]=d;const f=tn(n.getIndexAngle(l)+a),g=Math.round(Yc(f)),_=Jd(g,h.x,d.w,0,180),m=Jd(g,h.y,d.h,90,270);XT(e,t,f,_,m)}n.setCenterPoint(t.l-e.l,e.r-t.r,t.t-e.t,e.b-t.b),n._pointLabelItems=YT(n,i,s)}function XT(n,t,e,i,s){const r=Math.abs(Math.sin(e)),o=Math.abs(Math.cos(e));let a=0,l=0;i.start<t.l?(a=(t.l-i.start)/r,n.l=Math.min(n.l,t.l-a)):i.end>t.r&&(a=(i.end-t.r)/r,n.r=Math.max(n.r,t.r+a)),s.start<t.t?(l=(t.t-s.start)/o,n.t=Math.min(n.t,t.t-l)):s.end>t.b&&(l=(s.end-t.b)/o,n.b=Math.max(n.b,t.b+l))}function jT(n,t,e){const i=n.drawingArea,{extra:s,additionalAngle:r,padding:o,size:a}=e,l=n.getPointPosition(t,i+s+o,r),c=Math.round(Yc(tn(l.angle+we))),h=ZT(l.y,a.h,c),u=qT(c),d=KT(l.x,a.w,u);return{visible:!0,x:l.x,y:h,textAlign:u,left:d,top:h,right:d+a.w,bottom:h+a.h}}function $T(n,t){if(!t)return!0;const{left:e,top:i,right:s,bottom:r}=n;return!(Kn({x:e,y:i},t)||Kn({x:e,y:r},t)||Kn({x:s,y:i},t)||Kn({x:s,y:r},t))}function YT(n,t,e){const i=[],s=n._pointLabels.length,r=n.options,{centerPointLabels:o,display:a}=r.pointLabels,l={extra:Ac(r)/2,additionalAngle:o?pe/s:0};let c;for(let h=0;h<s;h++){l.padding=e[h],l.size=t[h];const u=jT(n,h,l);i.push(u),a==="auto"&&(u.visible=$T(u,c),u.visible&&(c=u))}return i}function qT(n){return n===0||n===180?"center":n<180?"left":"right"}function KT(n,t,e){return e==="right"?n-=t:e==="center"&&(n-=t/2),n}function ZT(n,t,e){return e===90||e===270?n-=t/2:(e>270||e<90)&&(n-=t),n}function JT(n,t,e){const{left:i,top:s,right:r,bottom:o}=e,{backdropColor:a}=t;if(!Zt(a)){const l=Gi(t.borderRadius),c=ze(t.backdropPadding);n.fillStyle=a;const h=i-c.left,u=s-c.top,d=r-i+c.width,f=o-s+c.height;Object.values(l).some(g=>g!==0)?(n.beginPath(),wr(n,{x:h,y:u,w:d,h:f,radius:l}),n.fill()):n.fillRect(h,u,d,f)}}function QT(n,t){const{ctx:e,options:{pointLabels:i}}=n;for(let s=t-1;s>=0;s--){const r=n._pointLabelItems[s];if(!r.visible)continue;const o=i.setContext(n.getPointLabelContext(s));JT(e,o,r);const a=Pe(o.font),{x:l,y:c,textAlign:h}=r;qi(e,n._pointLabels[s],l,c+a.lineHeight/2,a,{color:o.color,textAlign:h,textBaseline:"middle"})}}function qp(n,t,e,i){const{ctx:s}=n;if(e)s.arc(n.xCenter,n.yCenter,t,0,fe);else{let r=n.getPointPosition(0,t);s.moveTo(r.x,r.y);for(let o=1;o<i;o++)r=n.getPointPosition(o,t),s.lineTo(r.x,r.y)}}function tw(n,t,e,i,s){const r=n.ctx,o=t.circular,{color:a,lineWidth:l}=t;!o&&!i||!a||!l||e<0||(r.save(),r.strokeStyle=a,r.lineWidth=l,r.setLineDash(s.dash||[]),r.lineDashOffset=s.dashOffset,r.beginPath(),qp(n,e,o,i),r.closePath(),r.stroke(),r.restore())}function ew(n,t,e){return Mi(n,{label:e,index:t,type:"pointLabel"})}class fr extends ca{constructor(t){super(t),this.xCenter=void 0,this.yCenter=void 0,this.drawingArea=void 0,this._pointLabels=[],this._pointLabelItems=[]}setDimensions(){const t=this._padding=ze(Ac(this.options)/2),e=this.width=this.maxWidth-t.width,i=this.height=this.maxHeight-t.height;this.xCenter=Math.floor(this.left+e/2+t.left),this.yCenter=Math.floor(this.top+i/2+t.top),this.drawingArea=Math.floor(Math.min(e,i)/2)}determineDataLimits(){const{min:t,max:e}=this.getMinMax(!1);this.min=ye(t)&&!isNaN(t)?t:0,this.max=ye(e)&&!isNaN(e)?e:0,this.handleTickRangeOptions()}computeTickLimit(){return Math.ceil(this.drawingArea/Ac(this.options))}generateTickLabels(t){ca.prototype.generateTickLabels.call(this,t),this._pointLabels=this.getLabels().map((e,i)=>{const s=ce(this.options.pointLabels.callback,[e,i],this);return s||s===0?s:""}).filter((e,i)=>this.chart.getDataVisibility(i))}fit(){const t=this.options;t.display&&t.pointLabels.display?WT(this):this.setCenterPoint(0,0,0,0)}setCenterPoint(t,e,i,s){this.xCenter+=Math.floor((t-e)/2),this.yCenter+=Math.floor((i-s)/2),this.drawingArea-=Math.min(this.drawingArea/2,Math.max(t,e,i,s))}getIndexAngle(t){const e=fe/(this._pointLabels.length||1),i=this.options.startAngle||0;return tn(t*e+gn(i))}getDistanceFromCenterForValue(t){if(Zt(t))return NaN;const e=this.drawingArea/(this.max-this.min);return this.options.reverse?(this.max-t)*e:(t-this.min)*e}getValueForDistanceFromCenter(t){if(Zt(t))return NaN;const e=t/(this.drawingArea/(this.max-this.min));return this.options.reverse?this.max-e:this.min+e}getPointLabelContext(t){const e=this._pointLabels||[];if(t>=0&&t<e.length){const i=e[t];return ew(this.getContext(),t,i)}}getPointPosition(t,e,i=0){const s=this.getIndexAngle(t)-we+i;return{x:Math.cos(s)*e+this.xCenter,y:Math.sin(s)*e+this.yCenter,angle:s}}getPointPositionForValue(t,e){return this.getPointPosition(t,this.getDistanceFromCenterForValue(e))}getBasePosition(t){return this.getPointPositionForValue(t||0,this.getBaseValue())}getPointLabelPosition(t){const{left:e,top:i,right:s,bottom:r}=this._pointLabelItems[t];return{left:e,top:i,right:s,bottom:r}}drawBackground(){const{backgroundColor:t,grid:{circular:e}}=this.options;if(t){const i=this.ctx;i.save(),i.beginPath(),qp(this,this.getDistanceFromCenterForValue(this._endValue),e,this._pointLabels.length),i.closePath(),i.fillStyle=t,i.fill(),i.restore()}}drawGrid(){const t=this.ctx,e=this.options,{angleLines:i,grid:s,border:r}=e,o=this._pointLabels.length;let a,l,c;if(e.pointLabels.display&&QT(this,o),s.display&&this.ticks.forEach((h,u)=>{if(u!==0||u===0&&this.min<0){l=this.getDistanceFromCenterForValue(h.value);const d=this.getContext(u),f=s.setContext(d),g=r.setContext(d);tw(this,f,l,o,g)}}),i.display){for(t.save(),a=o-1;a>=0;a--){const h=i.setContext(this.getPointLabelContext(a)),{color:u,lineWidth:d}=h;!d||!u||(t.lineWidth=d,t.strokeStyle=u,t.setLineDash(h.borderDash),t.lineDashOffset=h.borderDashOffset,l=this.getDistanceFromCenterForValue(e.reverse?this.min:this.max),c=this.getPointPosition(a,l),t.beginPath(),t.moveTo(this.xCenter,this.yCenter),t.lineTo(c.x,c.y),t.stroke())}t.restore()}}drawBorder(){}drawLabels(){const t=this.ctx,e=this.options,i=e.ticks;if(!i.display)return;const s=this.getIndexAngle(0);let r,o;t.save(),t.translate(this.xCenter,this.yCenter),t.rotate(s),t.textAlign="center",t.textBaseline="middle",this.ticks.forEach((a,l)=>{if(l===0&&this.min>=0&&!e.reverse)return;const c=i.setContext(this.getContext(l)),h=Pe(c.font);if(r=this.getDistanceFromCenterForValue(this.ticks[l].value),c.showLabelBackdrop){t.font=h.string,o=t.measureText(a.label).width,t.fillStyle=c.backdropColor;const u=ze(c.backdropPadding);t.fillRect(-o/2-u.left,-r-h.size/2-u.top,o+u.width,h.size+u.height)}qi(t,a.label,0,-r,h,{color:c.color,strokeColor:c.textStrokeColor,strokeWidth:c.textStrokeWidth})}),t.restore()}drawTitle(){}}st(fr,"id","radialLinear"),st(fr,"defaults",{display:!0,animate:!0,position:"chartArea",angleLines:{display:!0,lineWidth:1,borderDash:[],borderDashOffset:0},grid:{circular:!1},startAngle:0,ticks:{showLabelBackdrop:!0,callback:pa.formatters.numeric},pointLabels:{backdropColor:void 0,backdropPadding:2,display:!0,font:{size:10},callback(t){return t},padding:5,centerPointLabels:!1}}),st(fr,"defaultRoutes",{"angleLines.color":"borderColor","pointLabels.color":"color","ticks.color":"color"}),st(fr,"descriptors",{angleLines:{_fallback:"grid"}});const va={millisecond:{common:!0,size:1,steps:1e3},second:{common:!0,size:1e3,steps:60},minute:{common:!0,size:6e4,steps:60},hour:{common:!0,size:36e5,steps:24},day:{common:!0,size:864e5,steps:30},week:{common:!1,size:6048e5,steps:4},month:{common:!0,size:2628e6,steps:12},quarter:{common:!1,size:7884e6,steps:4},year:{common:!0,size:3154e7}},Ye=Object.keys(va);function Qd(n,t){return n-t}function tf(n,t){if(Zt(t))return null;const e=n._adapter,{parser:i,round:s,isoWeekday:r}=n._parseOpts;let o=t;return typeof i=="function"&&(o=i(o)),ye(o)||(o=typeof i=="string"?e.parse(o,i):e.parse(o)),o===null?null:(s&&(o=s==="week"&&(Fs(r)||r===!0)?e.startOf(o,"isoWeek",r):e.startOf(o,s)),+o)}function ef(n,t,e,i){const s=Ye.length;for(let r=Ye.indexOf(n);r<s-1;++r){const o=va[Ye[r]],a=o.steps?o.steps:Number.MAX_SAFE_INTEGER;if(o.common&&Math.ceil((e-t)/(a*o.size))<=i)return Ye[r]}return Ye[s-1]}function nw(n,t,e,i,s){for(let r=Ye.length-1;r>=Ye.indexOf(e);r--){const o=Ye[r];if(va[o].common&&n._adapter.diff(s,i,o)>=t-1)return o}return Ye[e?Ye.indexOf(e):0]}function iw(n){for(let t=Ye.indexOf(n)+1,e=Ye.length;t<e;++t)if(va[Ye[t]].common)return Ye[t]}function nf(n,t,e){if(!e)n[t]=!0;else if(e.length){const{lo:i,hi:s}=qc(e,t),r=e[i]>=t?e[i]:e[s];n[r]=!0}}function sw(n,t,e,i){const s=n._adapter,r=+s.startOf(t[0].value,i),o=t[t.length-1].value;let a,l;for(a=r;a<=o;a=+s.add(a,1,i))l=e[a],l>=0&&(t[l].major=!0);return t}function sf(n,t,e){const i=[],s={},r=t.length;let o,a;for(o=0;o<r;++o)a=t[o],s[a]=o,i.push({value:a,major:!1});return r===0||!e?i:sw(n,i,s,e)}class Rr extends Zi{constructor(t){super(t),this._cache={data:[],labels:[],all:[]},this._unit="day",this._majorUnit=void 0,this._offsets={},this._normalized=!1,this._parseOpts=void 0}init(t,e={}){const i=t.time||(t.time={}),s=this._adapter=new dS._date(t.adapters.date);s.init(e),mr(i.displayFormats,s.formats()),this._parseOpts={parser:i.parser,round:i.round,isoWeekday:i.isoWeekday},super.init(t),this._normalized=e.normalized}parse(t,e){return t===void 0?null:tf(this,t)}beforeLayout(){super.beforeLayout(),this._cache={data:[],labels:[],all:[]}}determineDataLimits(){const t=this.options,e=this._adapter,i=t.time.unit||"day";let{min:s,max:r,minDefined:o,maxDefined:a}=this.getUserBounds();function l(c){!o&&!isNaN(c.min)&&(s=Math.min(s,c.min)),!a&&!isNaN(c.max)&&(r=Math.max(r,c.max))}(!o||!a)&&(l(this._getLabelBounds()),(t.bounds!=="ticks"||t.ticks.source!=="labels")&&l(this.getMinMax(!1))),s=ye(s)&&!isNaN(s)?s:+e.startOf(Date.now(),i),r=ye(r)&&!isNaN(r)?r:+e.endOf(Date.now(),i)+1,this.min=Math.min(s,r-1),this.max=Math.max(s+1,r)}_getLabelBounds(){const t=this.getLabelTimestamps();let e=Number.POSITIVE_INFINITY,i=Number.NEGATIVE_INFINITY;return t.length&&(e=t[0],i=t[t.length-1]),{min:e,max:i}}buildTicks(){const t=this.options,e=t.time,i=t.ticks,s=i.source==="labels"?this.getLabelTimestamps():this._generate();t.bounds==="ticks"&&s.length&&(this.min=this._userMin||s[0],this.max=this._userMax||s[s.length-1]);const r=this.min,o=this.max,a=Fy(s,r,o);return this._unit=e.unit||(i.autoSkip?ef(e.minUnit,this.min,this.max,this._getLabelCapacity(r)):nw(this,a.length,e.minUnit,this.min,this.max)),this._majorUnit=!i.major.enabled||this._unit==="year"?void 0:iw(this._unit),this.initOffsets(s),t.reverse&&a.reverse(),sf(this,a,this._majorUnit)}afterAutoSkip(){this.options.offsetAfterAutoskip&&this.initOffsets(this.ticks.map(t=>+t.value))}initOffsets(t=[]){let e=0,i=0,s,r;this.options.offset&&t.length&&(s=this.getDecimalForValue(t[0]),t.length===1?e=1-s:e=(this.getDecimalForValue(t[1])-s)/2,r=this.getDecimalForValue(t[t.length-1]),t.length===1?i=r:i=(r-this.getDecimalForValue(t[t.length-2]))/2);const o=t.length<3?.5:.25;e=Oe(e,0,o),i=Oe(i,0,o),this._offsets={start:e,end:i,factor:1/(e+1+i)}}_generate(){const t=this._adapter,e=this.min,i=this.max,s=this.options,r=s.time,o=r.unit||ef(r.minUnit,e,i,this._getLabelCapacity(e)),a=Ot(s.ticks.stepSize,1),l=o==="week"?r.isoWeekday:!1,c=Fs(l)||l===!0,h={};let u=e,d,f;if(c&&(u=+t.startOf(u,"isoWeek",l)),u=+t.startOf(u,c?"day":o),t.diff(i,e,o)>1e5*a)throw new Error(e+" and "+i+" are too far apart with stepSize of "+a+" "+o);const g=s.ticks.source==="data"&&this.getDataTimestamps();for(d=u,f=0;d<i;d=+t.add(d,a,o),f++)nf(h,d,g);return(d===i||s.bounds==="ticks"||f===1)&&nf(h,d,g),Object.keys(h).sort(Qd).map(_=>+_)}getLabelForValue(t){const e=this._adapter,i=this.options.time;return i.tooltipFormat?e.format(t,i.tooltipFormat):e.format(t,i.displayFormats.datetime)}format(t,e){const s=this.options.time.displayFormats,r=this._unit,o=e||s[r];return this._adapter.format(t,o)}_tickFormatFunction(t,e,i,s){const r=this.options,o=r.ticks.callback;if(o)return ce(o,[t,e,i],this);const a=r.time.displayFormats,l=this._unit,c=this._majorUnit,h=l&&a[l],u=c&&a[c],d=i[e],f=c&&u&&d&&d.major;return this._adapter.format(t,s||(f?u:h))}generateTickLabels(t){let e,i,s;for(e=0,i=t.length;e<i;++e)s=t[e],s.label=this._tickFormatFunction(s.value,e,t)}getDecimalForValue(t){return t===null?NaN:(t-this.min)/(this.max-this.min)}getPixelForValue(t){const e=this._offsets,i=this.getDecimalForValue(t);return this.getPixelForDecimal((e.start+i)*e.factor)}getValueForPixel(t){const e=this._offsets,i=this.getDecimalForPixel(t)/e.factor-e.end;return this.min+i*(this.max-this.min)}_getLabelSize(t){const e=this.options.ticks,i=this.ctx.measureText(t).width,s=gn(this.isHorizontal()?e.maxRotation:e.minRotation),r=Math.cos(s),o=Math.sin(s),a=this._resolveTickFontOptions(0).size;return{w:i*r+a*o,h:i*o+a*r}}_getLabelCapacity(t){const e=this.options.time,i=e.displayFormats,s=i[e.unit]||i.millisecond,r=this._tickFormatFunction(t,0,sf(this,[t],this._majorUnit),s),o=this._getLabelSize(r),a=Math.floor(this.isHorizontal()?this.width/o.w:this.height/o.h)-1;return a>0?a:1}getDataTimestamps(){let t=this._cache.data||[],e,i;if(t.length)return t;const s=this.getMatchingVisibleMetas();if(this._normalized&&s.length)return this._cache.data=s[0].controller.getAllParsedValues(this);for(e=0,i=s.length;e<i;++e)t=t.concat(s[e].controller.getAllParsedValues(this));return this._cache.data=this.normalize(t)}getLabelTimestamps(){const t=this._cache.labels||[];let e,i;if(t.length)return t;const s=this.getLabels();for(e=0,i=s.length;e<i;++e)t.push(tf(this,s[e]));return this._cache.labels=this._normalized?t:this.normalize(t)}normalize(t){return lp(t.sort(Qd))}}st(Rr,"id","time"),st(Rr,"defaults",{bounds:"data",adapters:{},time:{parser:!1,unit:!1,round:!1,isoWeekday:!1,minUnit:"millisecond",displayFormats:{}},ticks:{source:"auto",callback:!1,major:{enabled:!1}}});function Lo(n,t,e){let i=0,s=n.length-1,r,o,a,l;e?(t>=n[i].pos&&t<=n[s].pos&&({lo:i,hi:s}=qn(n,"pos",t)),{pos:r,time:a}=n[i],{pos:o,time:l}=n[s]):(t>=n[i].time&&t<=n[s].time&&({lo:i,hi:s}=qn(n,"time",t)),{time:r,pos:a}=n[i],{time:o,pos:l}=n[s]);const c=o-r;return c?a+(l-a)*(t-r)/c:a}class Cc extends Rr{constructor(t){super(t),this._table=[],this._minPos=void 0,this._tableRange=void 0}initOffsets(){const t=this._getTimestampsForTable(),e=this._table=this.buildLookupTable(t);this._minPos=Lo(e,this.min),this._tableRange=Lo(e,this.max)-this._minPos,super.initOffsets(t)}buildLookupTable(t){const{min:e,max:i}=this,s=[],r=[];let o,a,l,c,h;for(o=0,a=t.length;o<a;++o)c=t[o],c>=e&&c<=i&&s.push(c);if(s.length<2)return[{time:e,pos:0},{time:i,pos:1}];for(o=0,a=s.length;o<a;++o)h=s[o+1],l=s[o-1],c=s[o],Math.round((h+l)/2)!==c&&r.push({time:c,pos:o/(a-1)});return r}_generate(){const t=this.min,e=this.max;let i=super.getDataTimestamps();return(!i.includes(t)||!i.length)&&i.splice(0,0,t),(!i.includes(e)||i.length===1)&&i.push(e),i.sort((s,r)=>s-r)}_getTimestampsForTable(){let t=this._cache.all||[];if(t.length)return t;const e=this.getDataTimestamps(),i=this.getLabelTimestamps();return e.length&&i.length?t=this.normalize(e.concat(i)):t=e.length?e:i,t=this._cache.all=t,t}getDecimalForValue(t){return(Lo(this._table,t)-this._minPos)/this._tableRange}getValueForPixel(t){const e=this._offsets,i=this.getDecimalForPixel(t)/e.factor-e.end;return Lo(this._table,i*this._tableRange+this._minPos,!0)}}st(Cc,"id","timeseries"),st(Cc,"defaults",Rr.defaults);var rw=Object.freeze({__proto__:null,CategoryScale:Ec,LinearScale:Tc,LogarithmicScale:wc,RadialLinearScale:fr,TimeScale:Rr,TimeSeriesScale:Cc});const ow=[uS,HE,NT,rw];Wn.register(...ow);const aw={id:"timeline_overlay",afterDatasetsDraw(n){const{ctx:t,chartArea:e,scales:i}=n,s=n.$steps||[];if(t.save(),t.font="11px sans-serif",t.textAlign="center",s.forEach((r,o)=>{if(o===0)return;const a=i.x.getPixelForValue(r.end_ms/1e3),l=i.x.getPixelForValue(r.start_ms/1e3);t.strokeStyle="rgba(255,255,255,0.18)",t.setLineDash([4,4]),t.beginPath(),t.moveTo(a,e.top),t.lineTo(a,e.bottom),t.stroke(),t.fillStyle=o===n.$active_step?"#ffd166":"rgba(220,226,235,0.65)",t.fillText(`${o}.${r.name}`,(l+a)/2,e.top+11)}),typeof n.$playhead_ms=="number"){const r=i.x.getPixelForValue(n.$playhead_ms/1e3);t.setLineDash([]),t.strokeStyle="#ffd166",t.lineWidth=2,t.beginPath(),t.moveTo(r,e.top),t.lineTo(r,e.bottom),t.stroke()}t.restore()}};class lw{constructor(t){this.chart=new Wn(t,{type:"line",data:{datasets:af.map((e,i)=>({label:e,data:[],borderColor:xh[i],backgroundColor:xh[i],borderWidth:2,pointRadius:0,tension:0}))},options:{animation:!1,responsive:!0,maintainAspectRatio:!1,parsing:!1,normalized:!0,interaction:{mode:"index",intersect:!1},layout:{padding:{top:16}},scales:{x:{type:"linear",title:{display:!0,text:"시간 (s)",color:"#aab3c2"},ticks:{color:"#aab3c2"},grid:{color:"rgba(255,255,255,0.06)"}},y:{min:0,max:180,title:{display:!0,text:"서보각 (°)",color:"#aab3c2"},ticks:{color:"#aab3c2",stepSize:30},grid:{color:"rgba(255,255,255,0.08)"}}},plugins:{legend:{labels:{color:"#dfe5ee",boxWidth:12},position:"right"},tooltip:{callbacks:{title:e=>`t = ${e[0].parsed.x.toFixed(2)} s`,label:e=>`${e.dataset.label}: ${e.parsed.y.toFixed(1)}°`}}},onClick:(e,i,s)=>{if(!this.on_seek)return;const r=s.scales.x.getValueForPixel(e.x);this.on_seek(r*1e3)}},plugins:[aw]}),this.on_seek=null}set_timeline(t){const e=t?t.samples:[];this.chart.data.datasets.forEach((i,s)=>{i.data=e.map(r=>({x:r.time_ms/1e3,y:r.servo[s]}))}),this.chart.$steps=t?t.steps:[],this.chart.$playhead_ms=t?0:void 0,this.chart.options.scales.x.max=t?t.total_ms/1e3:void 0,this.chart.update()}set_playhead(t,e){this.chart.$playhead_ms===t&&this.chart.$active_step===e||(this.chart.$playhead_ms=t,this.chart.$active_step=e,this.chart.draw())}}class cw{constructor(){this.timeline=null,this.time_ms=0,this.playing=!1,this.speed=1}load(t){this.timeline=t,this.time_ms=0,this.playing=!1}unload(){this.timeline=null,this.playing=!1,this.time_ms=0}get loaded(){return!!(this.timeline&&this.timeline.samples.length>0)}get total_ms(){return this.timeline?this.timeline.total_ms:0}play(){this.loaded&&(this.time_ms>=this.total_ms&&(this.time_ms=0),this.playing=!0)}pause(){this.playing=!1}stop(){this.playing=!1,this.time_ms=0}seek(t){this.time_ms=Math.min(this.total_ms,Math.max(0,t))}current_step_index(){var t;return((t=this.current_sample())==null?void 0:t.step_index)??0}step_next(){if(!this.loaded)return;this.pause();const e=this.timeline.steps.find(i=>i.end_ms>this.time_ms+.5);this.seek(e?e.end_ms:this.total_ms)}step_prev(){if(!this.loaded)return;this.pause();const e=[...this.timeline.steps].reverse().find(i=>i.end_ms<this.time_ms-.5);this.seek(e?e.end_ms:0)}update(t){this.playing&&(this.time_ms+=t*this.speed,this.time_ms>=this.total_ms&&(this.time_ms=this.total_ms,this.playing=!1))}current_sample_index(){if(!this.loaded)return-1;const{samples:t,sample_ms:e}=this.timeline;let i=Math.min(t.length-1,Math.max(0,Math.round(this.time_ms/e)));for(;i>0&&t[i].time_ms>this.time_ms+1e-6;)i--;for(;i<t.length-1&&t[i+1].time_ms<=this.time_ms+1e-6;)i++;return i}current_sample(){const t=this.current_sample_index();return t>=0?this.timeline.samples[t]:null}}class hw{constructor(){this.name="sim",this.connected=!0}send(){}}class ba{constructor({on_log:t=()=>{},on_lost:e=()=>{}}={}){this.name="serial",this.connected=!1,this.port=null,this.writer=null,this.reader=null,this.on_log=t,this.on_lost=e,this.send_interval_ms=40,this.last_send_time=0,this.last_sent_line="",this.pending_line=null,this.is_writing=!1,this.ready_at=0,this.ok_count=0,this.encoder=new TextEncoder}static is_supported(){return typeof navigator<"u"&&"serial"in navigator}async connect(t=115200){if(!ba.is_supported())throw new Error("이 브라우저는 Web Serial API 를 지원하지 않습니다. Chrome 또는 Edge 를 사용하세요.");this.port=await navigator.serial.requestPort(),await this.port.open({baudRate:t}),this.writer=this.port.writable.getWriter(),this.connected=!0,this.last_sent_line="",this.ok_count=0,this.ready_at=performance.now()+2e3,this.on_log(`포트 열림 (${t}bps), 아두이노 부팅 대기 2초...`),this.handle_usb_disconnect=e=>{e.target===this.port&&this.handle_lost("USB 연결이 끊어졌습니다.")},navigator.serial.addEventListener("disconnect",this.handle_usb_disconnect),this.read_loop()}async read_loop(){var i;const t=new TextDecoder;let e="";try{for(this.reader=this.port.readable.getReader();this.connected;){const{value:s,done:r}=await this.reader.read();if(r)break;e+=t.decode(s,{stream:!0});let o;for(;(o=e.indexOf(`
`))>=0;){const a=e.slice(0,o).trim();e=e.slice(o+1),a&&(a==="OK"?this.ok_count++:this.on_log(`← ${a}`))}}}catch(s){this.connected&&this.handle_lost(`수신 오류: ${s.message}`)}finally{try{(i=this.reader)==null||i.releaseLock()}catch{}}}send(t){if(!this.connected)return;const e=performance.now();if(e<this.ready_at||e-this.last_send_time<this.send_interval_ms)return;const i=`A,${t.map(s=>Math.round(s)).join(",")}
`;i!==this.last_sent_line&&(this.last_send_time=e,this.last_sent_line=i,this.pending_line=i,this.flush())}async flush(){if(this.is_writing||!this.pending_line||!this.writer)return;this.is_writing=!0;const t=this.pending_line;this.pending_line=null;try{await this.writer.write(this.encoder.encode(t))}catch(e){this.handle_lost(`전송 오류: ${e.message}`)}finally{this.is_writing=!1}this.pending_line&&this.flush()}handle_lost(t){this.connected&&(this.on_log(t),this.disconnect().finally(()=>this.on_lost(t)))}async disconnect(){var t,e,i;this.connected=!1,this.handle_usb_disconnect&&navigator.serial.removeEventListener("disconnect",this.handle_usb_disconnect);try{await((t=this.reader)==null?void 0:t.cancel())}catch{}try{(e=this.writer)==null||e.releaseLock()}catch{}try{await((i=this.port)==null?void 0:i.close())}catch{}this.reader=null,this.writer=null,this.port=null,this.on_log("연결 해제")}}const q=n=>document.getElementById(n),Et=(n,t=1)=>Number.isFinite(n)?n.toFixed(t):"-",_n=(n,t=1)=>n.map(e=>Et(e,t)).join(", ");let St=dm()||ha();const Q={current_servo:St.home_angles.slice(),target_servo:St.home_angles.slice(),follow_timeline:!1,last_ik:null,ik_target:null,keyframes:[],pick_place_plan:null,show_frames:!1,show_workspace:!1,show_trail:!0,workspace_dirty:!0,last_readout_time:0,last_sample_index:-1},Me=new ty(q("viewport")),ya=new lw(q("chart_canvas")),kt=new cw,uw=new hw,Ps=new ba({on_log:n=>sm(n),on_lost:n=>{ch(),alert(`아두이노 연결이 끊어져 시뮬레이션 모드로 전환합니다.
${n}`)}}),dw=()=>Ps.connected?Ps:uw;document.querySelectorAll(".tab").forEach(n=>{n.addEventListener("click",()=>{document.querySelectorAll(".tab").forEach(t=>t.classList.toggle("active",t===n)),document.querySelectorAll(".tab-body").forEach(t=>t.classList.toggle("active",t.id===n.dataset.tab))})});const Kp=af.map((n,t)=>{const e=document.createElement("div");e.className="servo-row",e.innerHTML=`
    <span class="name">S${t+1} ${n.split(" ")[1]}</span>
    <input type="range" min="0" max="180" step="1" />
    <input type="number" min="0" max="180" step="1" />
    <span class="current" title="현재 화면 각도">-</span>`,q("servo_sliders").appendChild(e);const i=e.querySelector("input[type=range]"),s=e.querySelector("input[type=number]"),r=e.querySelector(".current"),o=a=>{const l=Vi(Number(a)||0);Zp(),Q.target_servo[t]=l,i.value=l,s.value=l};return i.addEventListener("input",()=>o(i.value)),s.addEventListener("change",()=>o(s.value)),{range:i,number:s,current:r}});function ah(n){Kp.forEach((t,e)=>{const i=Math.round(n[e]);document.activeElement!==t.number&&(t.number.value=i),document.activeElement!==t.range&&(t.range.value=i)})}function Zp(){Q.follow_timeline&&(kt.pause(),Q.follow_timeline=!1,Q.target_servo=Q.current_servo.slice())}function Ji(n){Zp(),Q.target_servo=n.map(Vi),ah(Q.target_servo)}q("btn_home").addEventListener("click",()=>Ji(St.home_angles));q("btn_grip_open").addEventListener("click",()=>Ji([...Q.target_servo.slice(0,3),St.gripper.open_angle]));q("btn_grip_close").addEventListener("click",()=>Ji([...Q.target_servo.slice(0,3),St.gripper.close_angle]));function fw(){return[Number(q("ik_x").value)||0,Number(q("ik_y").value)||0,Number(q("ik_z").value)||0]}function Ma(n){const t=fw(),e=Ko(St,t,{prefer:q("ik_prefer").value,reference_servo:q("ik_prefer").value==="auto"?Q.current_servo.slice(0,3):null});Q.last_ik={target:t,result:e},Q.ik_target=t,Me.set_target(t,e.ok),pw(),vw(),e.ok&&n&&Ji([...e.servo,Q.target_servo[3]])}function pw(){const n=q("ik_message"),t=q("ik_candidates");if(!Q.last_ik)return;const{result:e}=Q.last_ik;if(e.ok){const i=e.chosen;n.className="message ok",n.textContent=`도달 가능 → 서보 (${_n(e.servo)}) · ${i.base==="front"?"정면":"뒤쪽"} / ${i.elbow==="up"?"팔꿈치 위":"팔꿈치 아래"} · 집게 기울기 ${Et(i.tool_pitch)}°`}else n.className="message error",n.textContent=`도달 불가: ${e.reason}`;if(!e.candidates||e.candidates.length===0){t.innerHTML='<tr><td colspan="8" class="muted">해 없음</td></tr>';return}t.innerHTML=e.candidates.map(i=>{const s=e.ok&&i===e.chosen,r=`${s?"★ ":""}${i.base==="front"?"정면":"뒤쪽"}/${i.elbow==="up"?"위":"아래"}`,o=i.servo.map((a,l)=>`<td class="${i.out_of_range.includes(l)?"bad":""}">${Et(a)}</td>`).join("");return`<tr class="${s?"active":""}"><td>${r}</td>${i.joint.map(a=>`<td>${Et(a)}</td>`).join("")}${o}<td>${Et(i.error_mm,3)}</td></tr>`}).join("")}q("btn_ik_solve").addEventListener("click",()=>Ma(!1));q("btn_ik_move").addEventListener("click",()=>Ma(!0));q("btn_ik_clear").addEventListener("click",()=>{Q.ik_target=null,Me.set_target(null)});Me.on_ground_pick=([n,t])=>{q("ik_x").value=Math.round(n),q("ik_y").value=Math.round(t),Ma(!0)};const Jp=of.map((n,t)=>{if(t===0)return null;const e=document.createElement("label");return e.innerHTML=`${t}. ${n.name}<input type="number" min="20" step="50" />`,q("pp_durations").appendChild(e),e.querySelector("input")});function mw(){const n=St.pick_place;["x","y","z"].forEach((t,e)=>{q(`pp_a${t}`).value=n.pick[e],q(`pp_b${t}`).value=n.place[e]}),q("pp_approach").value=n.approach_height,q("pp_sample").value=n.sample_ms,q("pp_prefer").value=n.prefer,q("pp_cartesian").checked=n.cartesian,Jp.forEach((t,e)=>t&&(t.value=n.durations[e]))}function Qp(){const n=St.pick_place;n.pick=["x","y","z"].map(t=>Number(q(`pp_a${t}`).value)||0),n.place=["x","y","z"].map(t=>Number(q(`pp_b${t}`).value)||0),n.approach_height=Number(q("pp_approach").value)||0,n.sample_ms=Math.max(5,Number(q("pp_sample").value)||20),n.prefer=q("pp_prefer").value,n.cartesian=q("pp_cartesian").checked,n.durations=Jp.map(t=>t?Math.max(20,Number(t.value)||500):0),cf(St)}function tm(){var e;Qp();const n=ym(St,St.pick_place),t=q("pp_message");return Me.set_pick_place(St.pick_place.pick,St.pick_place.place),n.ok?(Q.pick_place_plan=n,t.className=n.warnings.length?"message warn":"message ok",t.textContent=`계획 생성 완료: ${n.steps.length}단계, 총 ${(n.total_ms/1e3).toFixed(2)}초, 샘플 ${n.samples.length}개`+(n.warnings.length?`
⚠ ${n.warnings.join(`
⚠ `)}`:""),gw(n),Sa(n),!0):(Q.pick_place_plan=null,t.className="message error",t.textContent=`계획 실패
${n.errors.join(`
`)}`,q("pp_steps").innerHTML='<tr><td colspan="8" class="muted">오류를 수정한 뒤 다시 생성하세요.</td></tr>',q("pp_samples").innerHTML="",((e=kt.timeline)==null?void 0:e.kind)==="pick_place"&&Rc(),Me.set_object(St.pick_place.pick),!1)}function gw(n){q("pp_steps").innerHTML=n.steps.map(e=>{const i=e.pos?`(${_n(e.pos,0)})`:"Home";return`<tr class="clickable" data-step="${e.index}" title="${e.desc}">
        <td>${e.index}</td><td class="left">${e.name}</td>
        <td>${Et(e.start_ms/1e3,2)}~${Et(e.end_ms/1e3,2)}</td>
        <td class="left">${i}</td>${e.servo.map(s=>`<td>${Et(s)}</td>`).join("")}</tr>`}).join(""),q("pp_steps").querySelectorAll("tr").forEach(e=>{e.addEventListener("click",()=>{const i=n.steps[Number(e.dataset.step)];kt.timeline!==n&&Sa(n),kt.pause(),kt.seek(i.end_ms),Q.follow_timeline=!0})});const t=n.samples.map(e=>`<tr><td>${e.time_ms.toFixed(0)}</td><td>${e.step_index}.${n.steps[e.step_index].name}</td>${e.servo.map(i=>`<td>${Et(i)}</td>`).join("")}${e.tcp.map(i=>`<td>${Et(i)}</td>`).join("")}</tr>`);q("pp_samples").innerHTML=t.join("")}q("btn_pp_plan").addEventListener("click",tm);q("btn_pp_play").addEventListener("click",()=>{!Q.pick_place_plan&&!tm()||(kt.timeline!==Q.pick_place_plan&&Sa(Q.pick_place_plan),lh())});q("btn_pp_csv").addEventListener("click",()=>{if(!Q.pick_place_plan)return alert("먼저 계획을 생성하세요.");rm("pick_place_timeline.csv",Sm(Q.pick_place_plan),"text/csv")});function em(){Q.keyframes.push({servo:Q.current_servo.map(n=>Math.round(n)),duration_ms:1e3}),zs()}function zs(){const n=q("kf_list");if(Q.keyframes.length===0){n.innerHTML='<tr><td colspan="7" class="muted">키프레임이 없습니다. 자세를 만든 뒤 [현재 자세 추가]를 누르세요.</td></tr>';return}n.innerHTML=Q.keyframes.map((t,e)=>`<tr>
        <td>${e+1}</td>${t.servo.map(i=>`<td>${i}</td>`).join("")}
        <td><input type="number" min="50" step="100" value="${t.duration_ms}" data-k="${e}" /></td>
        <td><button class="btn btn-small" data-go="${e}">이동</button>
            <button class="btn btn-small" data-up="${e}">▲</button>
            <button class="btn btn-small btn-danger" data-del="${e}">✕</button></td></tr>`).join(""),n.querySelectorAll("input[data-k]").forEach(t=>t.addEventListener("change",()=>Q.keyframes[Number(t.dataset.k)].duration_ms=Math.max(50,Number(t.value)||1e3))),n.querySelectorAll("[data-go]").forEach(t=>t.addEventListener("click",()=>Ji(Q.keyframes[Number(t.dataset.go)].servo))),n.querySelectorAll("[data-up]").forEach(t=>t.addEventListener("click",()=>{const e=Number(t.dataset.up);e>0&&([Q.keyframes[e-1],Q.keyframes[e]]=[Q.keyframes[e],Q.keyframes[e-1]]),zs()})),n.querySelectorAll("[data-del]").forEach(t=>t.addEventListener("click",()=>{Q.keyframes.splice(Number(t.dataset.del),1),zs()}))}q("btn_kf_add").addEventListener("click",em);q("btn_add_keyframe_fk").addEventListener("click",()=>{em(),q("btn_add_keyframe_fk").textContent=`키프레임 추가됨 (${Q.keyframes.length})`,setTimeout(()=>q("btn_add_keyframe_fk").textContent="현재 자세 → 키프레임",1200)});q("btn_kf_clear").addEventListener("click",()=>{Q.keyframes=[],zs()});q("btn_kf_play").addEventListener("click",()=>{if(Q.keyframes.length===0)return alert("키프레임을 먼저 추가하세요.");const n=Mm(St,Q.current_servo,Q.keyframes,20);if(!n.ok)return alert(n.errors.join(`
`));Sa(n),lh()});function Sa(n){kt.load(n),ya.set_timeline(n),q("rng_time").disabled=!1,q("rng_time").max=n.total_ms,Q.follow_timeline=!0,Q.last_sample_index=-1,Me.set_trail(n.samples.map(t=>t.tcp)),Me.set_trail_visible(Q.show_trail),Me.set_pick_place(St.pick_place.pick,St.pick_place.place)}function Rc(){kt.unload(),ya.set_timeline(null),q("rng_time").disabled=!0,Q.follow_timeline=!1,Me.set_trail(null)}function lh(){kt.loaded&&(Q.follow_timeline=!0,kt.play())}q("btn_play").addEventListener("click",()=>{kt.loaded&&(kt.playing?kt.pause():lh())});q("btn_stop").addEventListener("click",()=>{kt.loaded&&(kt.stop(),Q.follow_timeline=!0)});q("btn_step_next").addEventListener("click",()=>{kt.step_next(),Q.follow_timeline=kt.loaded});q("btn_step_prev").addEventListener("click",()=>{kt.step_prev(),Q.follow_timeline=kt.loaded});q("sel_speed").addEventListener("change",()=>kt.speed=Number(q("sel_speed").value));q("rng_time").addEventListener("input",()=>{kt.pause(),kt.seek(Number(q("rng_time").value)),Q.follow_timeline=!0});ya.on_seek=n=>{kt.loaded&&(kt.pause(),kt.seek(n),Q.follow_timeline=!0)};const nm=St.joints.map((n,t)=>{const e=document.createElement("tr");return e.innerHTML=`<td>${["J1 Base","J2 Shoulder","J3 Elbow"][t]}</td>
    <td><input type="number" step="1" /></td>
    <td><select><option value="1">+1</option><option value="-1">-1</option></select></td>`,q("joint_cfg_rows").appendChild(e),{offset:e.querySelector("input"),direction:e.querySelector("select")}});function Ea(){document.querySelectorAll("[data-link]").forEach(n=>n.value=St.links[n.dataset.link]),nm.forEach((n,t)=>{n.offset.value=St.joints[t].offset,n.direction.value=String(St.joints[t].direction)}),q("cfg_grip_open").value=St.gripper.open_angle,q("cfg_grip_close").value=St.gripper.close_angle,q("cfg_grip_gap").value=St.gripper.max_gap_mm,document.querySelectorAll("[data-home]").forEach(n=>n.value=St.home_angles[Number(n.dataset.home)]),q("cfg_speed").value=St.max_speed_deg_s,mw()}function _w(){document.querySelectorAll("[data-link]").forEach(n=>{St.links[n.dataset.link]=Math.max(1,Number(n.value)||1)}),nm.forEach((n,t)=>{St.joints[t].offset=Number(n.offset.value)||0,St.joints[t].direction=Number(n.direction.value)===-1?-1:1}),St.gripper.open_angle=Vi(Number(q("cfg_grip_open").value)||0),St.gripper.close_angle=Vi(Number(q("cfg_grip_close").value)||0),St.gripper.max_gap_mm=Math.max(1,Number(q("cfg_grip_gap").value)||40),document.querySelectorAll("[data-home]").forEach(n=>{St.home_angles[Number(n.dataset.home)]=Vi(Number(n.value)||0)}),St.max_speed_deg_s=Math.max(10,Number(q("cfg_speed").value)||120),Ta()}function Ta(){var n,t;if(cf(St),Q.workspace_dirty=!0,Q.show_workspace&&im(),Q.pick_place_plan&&(Q.pick_place_plan=null,((n=kt.timeline)==null?void 0:n.kind)==="pick_place"&&Rc(),q("pp_message").className="message warn",q("pp_message").textContent="설정이 바뀌었습니다. [계획 생성]을 다시 누르세요."),((t=kt.timeline)==null?void 0:t.kind)==="keyframe"&&Rc(),Q.last_ik){const e=Q.ik_target===null;Ma(!1),e&&(Q.ik_target=null,Me.set_target(null))}}document.querySelectorAll("#tab_settings input, #tab_settings select").forEach(n=>{n.id!=="file_json"&&n.addEventListener("change",_w)});q("btn_set_home").addEventListener("click",()=>{St.home_angles=Q.current_servo.map(n=>Math.round(n)),Ea(),Ta()});q("btn_save_json").addEventListener("click",()=>{Qp();const n={...St,keyframes:Q.keyframes};rm("robot_arm_settings.json",JSON.stringify(n,null,2),"application/json")});q("btn_load_json").addEventListener("click",()=>q("file_json").click());q("file_json").addEventListener("change",async()=>{const n=q("file_json").files[0];if(n){try{const t=JSON.parse(await n.text());St=hf(ha(),t),Array.isArray(t.keyframes)&&(Q.keyframes=t.keyframes.filter(e=>Array.isArray(e.servo)&&e.servo.length===4).map(e=>({servo:e.servo.map(i=>Vi(Number(i)||0)),duration_ms:Number(e.duration_ms)||1e3}))),Ea(),zs(),Ta(),Ji(St.home_angles)}catch(t){alert(`JSON 불러오기 실패: ${t.message}`)}q("file_json").value=""}});q("btn_reset").addEventListener("click",()=>{confirm("모든 설정을 기본값으로 되돌릴까요?")&&(St=ha(),Ea(),Ta(),Ji(St.home_angles))});q("chk_frames").addEventListener("change",()=>{Q.show_frames=q("chk_frames").checked,Me.set_show_frames(Q.show_frames)});q("chk_workspace").addEventListener("change",()=>{Q.show_workspace=q("chk_workspace").checked,Q.show_workspace&&im(),Me.set_workspace_visible(Q.show_workspace)});q("chk_trail").addEventListener("change",()=>{Q.show_trail=q("chk_trail").checked,Me.set_trail_visible(Q.show_trail)});function im(){if(!Q.workspace_dirty)return;const n=[];for(let t=0;t<=180;t+=10)for(let e=0;e<=180;e+=6)for(let i=0;i<=180;i+=6){const s=Pr(St,[t,e,i,0]).tcp;s[2]>=0&&n.push(s)}Me.set_workspace(n),Q.workspace_dirty=!1}function xw(n){const t=q("learn_dh");t.innerHTML=n.dh_table.map(a=>`<tr><td>${a.index}</td><td class="left">${a.name}</td><td>${Et(a.theta)}</td><td>${Et(a.d)}</td><td>${Et(a.a)}</td><td>${Et(a.alpha,0)}</td></tr>`).join("");const e=Q.current_servo,i=St.joints,s=St.links,r=n.closed_form,o=n.joint_deg;q("learn_fk").textContent=["1) 서보각 → 관절각 : θ = (S − offset) × direction",`   θ1 = (${Et(e[0])} − ${i[0].offset}) × ${i[0].direction} = ${Et(o[0])}°`,`   θ2 = (${Et(e[1])} − ${i[1].offset}) × ${i[1].direction} = ${Et(o[1])}°`,`   θ3 = (${Et(e[2])} − ${i[2].offset}) × ${i[2].direction} = ${Et(o[2])}°`,"2) 팔 평면에서의 위치 (L23 = L2 + L3)",`   L23 = ${s.link2_length} + ${s.gripper_length} = ${r.link23}`,`   r = L1·cosθ2 + L23·cos(θ2+θ3) = ${s.link1_length}·cos(${Et(o[1])}°) + ${r.link23}·cos(${Et(o[1]+o[2])}°) = ${Et(r.r)}`,`   z = L0 + L1·sinθ2 + L23·sin(θ2+θ3) = ${s.base_height} + ${s.link1_length}·sin(${Et(o[1])}°) + ${r.link23}·sin(${Et(o[1]+o[2])}°) = ${Et(r.z)}`,"3) 베이스 회전 적용",`   x = r·cosθ1 = ${Et(r.r)}·cos(${Et(o[0])}°) = ${Et(r.x)}`,`   y = r·sinθ1 = ${Et(r.r)}·sin(${Et(o[0])}°) = ${Et(r.y)}`,`4) DH 행렬 곱 결과 TCP = (${_n(n.tcp)})  ← 2)3) 과 같아야 함`].join(`
`)}function vw(){if(!Q.last_ik)return;const{target:n,result:t}=Q.last_ik,e=t.steps,i=[`목표 TCP = (${_n(n)})`,`1) θ1 = atan2(y, x) = atan2(${Et(e.y)}, ${Et(e.x)}) = ${Et(e.theta1_front)}°  (뒤쪽 해: θ1 ± 180°)`,`2) ρ = √(x² + y²) = ${Et(e.rho)},   s = z − L0 = ${Et(e.z)} − ${St.links.base_height} = ${Et(e.s)}`,`   어깨~목표 거리 = √(ρ² + s²) = ${Et(e.dist)}  (도달 범위 ${Et(e.reach_min)} ~ ${Et(e.reach_max)})`,"3) D = (r² + s² − L1² − L23²) / (2·L1·L23)",`     = (${Et(e.rho)}² + ${Et(e.s)}² − ${e.l1}² − ${e.l23}²) / (2·${e.l1}·${e.l23}) = ${Et(e.d_value,4)}`];if(!t.candidates||t.candidates.length===0)i.push("   |D| > 1 → 삼각형이 만들어지지 않아 도달 불가");else{i.push("4) θ3 = atan2(±√(1 − D²), D)"),i.push("5) θ2 = atan2(s, r) − atan2(L23·sinθ3, L1 + L23·cosθ3)"),i.push("6) 서보각 = θ × direction + offset, 0~180° 범위 검사");for(const s of t.candidates){const r=t.ok&&s===t.chosen?"★":" ";i.push(`  ${r} ${s.base==="front"?"정면":"뒤쪽"}/${s.elbow==="up"?"팔꿈치 위 ":"팔꿈치 아래"}: θ=(${_n(s.joint)}) → S=(${_n(s.servo)}) ${s.in_range?"OK":"범위 초과"}`)}}i.push(t.ok?`결과: 서보 (${_n(t.servo)})`:`결과: 실패 - ${t.reason}`),q("learn_ik").textContent=i.join(`
`)}function ch(){const n=Ps.connected;q("mode_badge").className=`badge ${n?"badge-live":"badge-sim"}`,q("mode_badge").textContent=n?"연동 모드 (아두이노)":"시뮬레이션 모드",q("btn_connect").textContent=n?"연결 해제":"아두이노 연결"}function sm(n){const t=q("serial_log"),e=new Date().toLocaleTimeString(),i=(t.textContent?t.textContent.split(`
`):[]).slice(-80);i.push(`[${e}] ${n}`),t.textContent=i.join(`
`),t.scrollTop=t.scrollHeight}ba.is_supported()||(q("btn_connect").disabled=!0,q("serial_hint").textContent="Web Serial 미지원 브라우저 (Chrome/Edge 필요) · 시뮬레이션만 가능");q("btn_connect").addEventListener("click",async()=>{try{Ps.connected?await Ps.disconnect():await Ps.connect(115200)}catch(n){n.name!=="NotFoundError"&&alert(`연결 실패: ${n.message}`),sm(`연결 실패: ${n.message}`)}ch()});function rm(n,t,e){const i=new Blob([e==="text/csv"?"\uFEFF"+t:t],{type:e}),s=URL.createObjectURL(i),r=document.createElement("a");r.href=s,r.download=n,r.click(),setTimeout(()=>URL.revokeObjectURL(s),1e3)}function bw(n){const t=St.max_speed_deg_s*n/1e3;Q.current_servo=Q.current_servo.map((e,i)=>{const s=Q.target_servo[i]-e;return Math.abs(s)<=t?Q.target_servo[i]:e+Math.sign(s)*t})}function yw(n){const t=kt.timeline;if(!t||t.kind!=="pick_place"){Me.set_object(St.pick_place.pick);return}const e=St.pick_place.pick,i=St.pick_place.place;!Q.follow_timeline||!n?Me.set_object(e):n.holding?Me.set_object(n.tcp):Me.set_object(n.step_index>=7?i:e)}function Mw(n){if(!kt.loaded){q("lbl_time").textContent="0.00 / 0.00 s",q("btn_play").textContent="▶";return}q("btn_play").textContent=kt.playing?"⏸":"▶",document.activeElement!==q("rng_time")&&(q("rng_time").value=kt.time_ms),q("lbl_time").textContent=`${(kt.time_ms/1e3).toFixed(2)} / ${(kt.total_ms/1e3).toFixed(2)} s`;const t=kt.timeline.steps[n.step_index],e=kt.timeline.kind==="pick_place"?"Pick & Place":"시퀀스";q("lbl_step").textContent=Q.follow_timeline?`[${e}] ${n.step_index}. ${t.name} — ${t.desc} · 서보 (${_n(n.servo)})`:`[${e}] 수동 조작 중 (▶ 를 누르면 타임라인으로 돌아갑니다)`,ya.set_playhead(kt.time_ms,n.step_index),kt.timeline===Q.pick_place_plan&&q("pp_steps").querySelectorAll("tr").forEach(i=>i.classList.toggle("active",Number(i.dataset.step)===n.step_index))}function Sw(n){const t=Q.current_servo;q("hud_tcp").textContent=`TCP: (${_n(n.tcp)}) mm`,q("hud_servo").textContent=`서보: S1 ${Et(t[0])}° S2 ${Et(t[1])}° S3 ${Et(t[2])}° S4 ${Et(t[3])}°`,q("fk_joint").textContent=`${_n(n.joint_deg)} °`,q("fk_tcp").textContent=`${_n(n.tcp)} mm`,q("fk_r").textContent=`${Et(n.closed_form.r)} mm`,q("fk_pitch").textContent=`${Et(n.tool_pitch)} °`,Kp.forEach((e,i)=>e.current.textContent=`${Et(t[i])}°`),q("tab_learn").classList.contains("active")&&xw(n)}let rf=performance.now();function om(n){const t=Math.min(100,n-rf);rf=n;let e=null;kt.loaded&&(kt.update(t),e=kt.current_sample()),Q.follow_timeline&&e?(Q.current_servo=e.servo.slice(),Q.target_servo=e.servo.slice(),ah(Q.current_servo)):bw(t);const i=Pr(St,Q.current_servo);Me.update_pose(St,i,Q.current_servo[3]),yw(e),dw().send(Q.current_servo),n-Q.last_readout_time>50&&(Q.last_readout_time=n,Sw(i),Mw(e)),Me.render(),requestAnimationFrame(om)}Ea();ah(Q.target_servo);zs();ch();Me.set_pick_place(St.pick_place.pick,St.pick_place.place);Me.set_object(St.pick_place.pick);requestAnimationFrame(om);window.robot_sim={settings:()=>St,state:Q,player:kt,forward_from_servo:Pr,solve_ik_servo:Ko,servo_list_to_joint:df};
