/*
  ui_web_s3_ap.ino — KeyGrid UI for ESP32-S3 (Arduino-ESP32 3.x)

  What it does
  • Starts a Wi-Fi SoftAP:  SSID: KeyGrid_UI   PASS: keygrid123
  • Hosts a single-page web app at "/"
  • Reads UART2 from SMOKE tail (pins below) and parses:
      - "UI: <room>.<tag>@<str>, ..." snapshots
      - "HELLO,mac=AA:BB:...,room=N,tail=..."
      - "LOG: ICE UNMAPPED <RAW>" (optional; shows in “Manage” for aliasing)
  • Lets you:
      - Rename rooms (persist /rooms.txt)
      - Map RAW→tag (persist /alias.csv) — also sends MAP line back over UART2
      - Set/clear registration strings per tag (persist /regs.csv)
      - Move/forget tags (UI-local only; smoke keeps doing its job)

  Hardware
  • Board: ESP32-S3 Dev Module
  • UART2: RX=16 (from SMOKE TX), TX=17 (to SMOKE RX; used to send MAP lines)
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <FS.h>

#if __has_include(<LittleFS.h>)
  #include <LittleFS.h>
  #define FSYS LittleFS
#else
  #include <SPIFFS.h>
  #define FSYS SPIFFS
#endif

// -------- Pins (match SMOKE tail UART1 pins) --------
#define UI_RX2 16
#define UI_TX2 17

// -------- AP creds --------
static const char* AP_SSID = "KeyGrid_UI";
static const char* AP_PASS = "keygrid123";

// -------- Limits --------
static const int MAX_ROOMS  = 25;
static const int MAX_TAG_ID = 1024;
static const int LOG_RING   = 200;
static const int MAX_PEERS  = 40;
static const int ALIAS_MAX  = 1024;
static const int SEEN_MAX   = 128;

// -------- Files --------
static const char* ROOMS_PATH = "/rooms.txt";
static const char* ALIAS_PATH = "/alias.csv"; // raw,tag
static const char* REGS_PATH  = "/regs.csv";  // tag,reg

// -------- State --------
String   roomNames[MAX_ROOMS];
uint16_t tagRoom[MAX_TAG_ID + 1];   // 0 = not shown
uint8_t  tagStr [MAX_TAG_ID + 1];   // 0..100
String   regByTag[MAX_TAG_ID + 1];  // "" if none

struct PeerInfo { String mac; int room; uint32_t lastMs; };
PeerInfo peers[MAX_PEERS]; int peerCount=0;

struct AliasEntry { String raw; uint16_t tag; bool used; };
AliasEntry aliasMap[ALIAS_MAX];

String seenRaw[SEEN_MAX]; int seenCount=0;

String logBuf[LOG_RING]; int logPtr=0;
inline void pushLog(const String& s){ logBuf[logPtr]=s; logPtr=(logPtr+1)%LOG_RING; Serial.println(s); }

// -------- Utils --------
static inline String jsonEscape(const String& s){
  String o; o.reserve(s.length()+4);
  for(size_t i=0;i<s.length();++i){
    char c=s[i];
    if(c=='"'||c=='\\'){ o+='\\'; o+=c; }
    else if(c=='\n'){ o+="\\n"; }
    else if(c!='\r'){ o+=c; }
  }
  return o;
}
static void setDefaultRooms(){
  static const char* d[10]={"Workshop","Parts","Spray room","Showroom","Accounts","Valeting","Frontline","Sideline","Yard 1","Yard 2"};
  for(int i=0;i<MAX_ROOMS;i++) roomNames[i] = (i<10? String(d[i]) : String());
}

// -------- FS helpers --------
static void roomsLoad(){
  for(int i=0;i<MAX_ROOMS;i++) roomNames[i]="";
  File f=FSYS.open(ROOMS_PATH,"r");
  if(!f){ setDefaultRooms(); return; }
  int i=0; while(f.available() && i<MAX_ROOMS){ String l=f.readStringUntil('\n'); l.trim(); roomNames[i++]=l; }
  f.close();
}
static void roomsSave(){
  File w=FSYS.open(ROOMS_PATH,"w");
  if(!w){ pushLog("[FS] rooms save fail"); return; }
  int limit=0; for(int i=MAX_ROOMS-1;i>=0;i--){ if(roomNames[i].length()){ limit=i+1; break; } }
  if(limit<5) limit=5;
  for(int i=0;i<limit;i++) w.println(roomNames[i].length()?roomNames[i]:String("Room ")+String(i+1));
  w.close(); pushLog("[FS] rooms saved");
}
static void aliasClear(){ for(int i=0;i<ALIAS_MAX;i++) aliasMap[i].used=false; }
static void aliasLoad(){
  aliasClear();
  File f=FSYS.open(ALIAS_PATH,"r");
  if(!f){ pushLog("[FS] no alias.csv"); return; }
  while(f.available()){
    String line=f.readStringUntil('\n'); line.trim(); if(!line.length()) continue;
    int c=line.indexOf(','); if(c<0) continue;
    String raw=line.substring(0,c); raw.trim();
    uint16_t tag=(uint16_t)line.substring(c+1).toInt();
    for(int i=0;i<ALIAS_MAX;i++) if(!aliasMap[i].used){ aliasMap[i].used=true; aliasMap[i].raw=raw; aliasMap[i].tag=tag; break; }
  }
  f.close(); pushLog("[FS] alias loaded");
}
static void aliasSave(){
  File w=FSYS.open(ALIAS_PATH,"w");
  if(!w){ pushLog("[FS] alias save fail"); return; }
  for(int i=0;i<ALIAS_MAX;i++) if(aliasMap[i].used){ w.print(aliasMap[i].raw); w.print(","); w.println(aliasMap[i].tag); }
  w.close(); pushLog("[FS] alias saved");
}
static int aliasFindRaw(const String& raw){ for(int i=0;i<ALIAS_MAX;i++) if(aliasMap[i].used && aliasMap[i].raw==raw) return i; return -1; }
static void aliasSet(const String& raw, uint16_t tag){
  int i=aliasFindRaw(raw);
  if(i>=0){ aliasMap[i].tag=tag; }
  else for(int k=0;k<ALIAS_MAX;k++) if(!aliasMap[k].used){ aliasMap[k].used=true; aliasMap[k].raw=raw; aliasMap[k].tag=tag; break; }
  aliasSave();
  // Optional hint back to SMOKE chain (future use)
  String line = "MAP,name=" + raw + ",id=" + String(tag);
  Serial2.println(line);
  pushLog("[MAP→SMOKE] " + line);
}
static void aliasDel(const String& raw){ int i=aliasFindRaw(raw); if(i>=0){ aliasMap[i].used=false; aliasSave(); } }

static void regsLoad(){
  for(int i=1;i<=MAX_TAG_ID;i++) regByTag[i]="";
  File f=FSYS.open(REGS_PATH,"r");
  if(!f){ pushLog("[FS] no regs.csv"); return; }
  while(f.available()){
    String line=f.readStringUntil('\n'); line.trim(); if(!line.length()) continue;
    int c=line.indexOf(','); if(c<0) continue;
    int id=line.substring(0,c).toInt();
    String reg=line.substring(c+1); reg.trim();
    if(id>=1 && id<=MAX_TAG_ID) regByTag[id]=reg;
  }
  f.close(); pushLog("[FS] regs loaded");
}
static void regsSave(){
  File w=FSYS.open(REGS_PATH,"w");
  if(!w){ pushLog("[FS] regs save fail"); return; }
  for(int i=1;i<=MAX_TAG_ID;i++) if(regByTag[i].length()){ w.print(i); w.print(","); w.println(regByTag[i]); }
  w.close(); pushLog("[FS] regs saved");
}

// -------- Web --------
WebServer server(80);

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang=en><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Key Grid — WiFi UI</title>
<style>
:root{--bg:#0b0f14;--panel:rgba(19,24,31,.72);--text:#e8eef6;--muted:#9fb0c4;--shadow:0 10px 30px rgba(0,0,0,.35), inset 0 1px 0 rgba(255,255,255,.05)}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}html,body{height:100%;margin:0;background:radial-gradient(1200px 700px at -10% -10%,#19202a 0,transparent 40%),radial-gradient(1000px 600px at 110% -20%,#0e1620 0,transparent 40%),radial-gradient(1200px 700px at 50% 120%,#0c131b 0,transparent 50%),var(--bg);color:var(--text);font:15px/1.35 system-ui,-apple-system,Segoe UI,Roboto,Inter,Arial}
.top{position:sticky;top:0;z-index:3;display:flex;gap:.6rem;align-items:center;padding:.7rem .9rem;background:linear-gradient(to bottom,rgba(12,16,22,.92),rgba(12,16,22,.7));backdrop-filter:blur(12px) saturate(115%);border-bottom:1px solid rgba(255,255,255,.06)}
.brand{font-weight:800;letter-spacing:.2px}
.tabs{display:flex;gap:.5rem;margin-left:.4rem}
.tab{padding:.5rem .8rem;border:1px solid rgba(255,255,255,.06);border-radius:12px;background:rgba(23,29,37,.7);cursor:pointer;user-select:none}
.tab.active{outline:2px solid rgba(102,214,255,.15)}
.status{margin-left:auto;color:var(--muted);font-weight:600}
.controls{display:grid;gap:.55rem;grid-template-columns:1fr auto;padding:.5rem .9rem .25rem}
.input{display:flex;align-items:center;gap:.6rem;padding:.7rem .9rem;border-radius:14px;background:linear-gradient(180deg,rgba(19,26,34,.9),rgba(14,20,28,.85));border:1px solid rgba(255,255,255,.06)}
.input input{all:unset;flex:1;font-size:15px;color:var(--text)}
.btn{padding:.7rem 1rem;border-radius:14px;border:1px solid rgba(255,255,255,.08);background:linear-gradient(180deg,#1a2836,#16222f);box-shadow:var(--shadow);font-weight:700;cursor:pointer}
.wrap{padding:.35rem .9rem 1rem}
.grid{display:grid;gap:.9rem;grid-template-columns:repeat(auto-fill,minmax(280px,1fr))}
.card{position:relative;padding:1rem .9rem .9rem;border-radius:16px;background:linear-gradient(180deg,rgba(19,26,34,.88),rgba(11,16,23,.86));border:1px solid rgba(255,255,255,.07);box-shadow:var(--shadow)}
.card h3{margin:.1rem 0 .6rem;font-size:18px;cursor:pointer}
.badge{position:absolute;top:.6rem;right:.6rem;font-size:13px;min-width:28px;text-align:center;background:linear-gradient(180deg,#2b3949,#1b2836);color:#e9f6ff;padding:.35rem .55rem;border-radius:999px;border:1px solid rgba(255,255,255,.07);box-shadow:var(--shadow)}
.chips{display:flex;flex-wrap:wrap;gap:.6rem}
.chip{padding:.55rem .7rem;border-radius:999px;background:linear-gradient(180deg,#0f1620,#0c141d);color:#cfe6ff;border:1px solid rgba(255,255,255,.07);box-shadow:inset 0 0 0 1px rgba(99,215,255,.15),var(--shadow);cursor:pointer}
.list{padding:.5rem .9rem 1.2rem;display:grid;gap:.6rem}
.row{display:flex;gap:.8rem;align-items:center;padding:.7rem .85rem;border-radius:12px;background:var(--panel);border:1px solid rgba(255,255,255,.06);box-shadow:var(--shadow);color:#d3e1f0}
.mono{font-family:ui-monospace,Menlo,Consolas,monospace}
.tag{padding:.2rem .45rem;border:1px solid rgba(255,255,255,.1);border-radius:8px;color:#d6f3ff;background:#0e1a24}
.drawer{position:fixed;inset:auto 0 0 0;transform:translateY(105%);background:linear-gradient(180deg,rgba(17,23,31,.98),rgba(13,19,27,.98));border-top-left-radius:18px;border-top-right-radius:18px;border-top:1px solid rgba(255,255,255,.08);box-shadow:0 -20px 50px rgba(0,0,0,.6);backdrop-filter:blur(12px);transition:transform .28s ease;z-index:40;padding:1rem .9rem}
.drawer.open{transform:translateY(0)}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:.55rem}
.muted{color:#9fb0c4}
.small{font-size:12px;opacity:.85}
</style>

<div class=top>
  <div class=brand>Key Grid</div>
  <div class=tabs>
    <div class="tab active" data-tab=grid>Grid</div>
    <div class=tab data-tab=peers>Peers</div>
    <div class=tab data-tab=logs>Logs</div>
    <div class=tab data-tab=manage>Manage</div>
  </div>
  <div class=status id=status>AP mode | waiting…</div>
</div>

<div class=controls id=gridControls>
  <label class=input><input id=q placeholder="Search reg or tag ID…"></label>
  <button class=btn id=btnSearch>Search</button>
</div>

<div class=wrap id=wrapGrid><div class=grid id=grid></div></div>
<div class=list id=wrapPeers hidden></div>
<div class=list id=wrapLogs hidden></div>

<!-- Manage -->
<div class=list id=wrapManage hidden>
  <div class=row style="display:block">
    <div class=muted style="margin-bottom:.35rem">Alias (map RAW → Tag)</div>
    <div class=grid2>
      <label class=input><input id=rawIn placeholder="RAW ID…"></label>
      <label class=input><input id=rawTag type=number placeholder="Tag #"></label>
    </div>
    <div style="display:flex;gap:.6rem;margin-top:.5rem">
      <button class=btn onclick="aliasSet()">Save Alias</button>
      <button class=btn onclick="aliasDel()">Delete Alias</button>
      <div class="small" id=aliasMsg></div>
    </div>
    <div class=muted style="margin:.7rem 0 .3rem">Seen RAW (tap to fill)</div>
    <div id=seenWrap class="row" style="flex-wrap:wrap"></div>
  </div>

  <div class=row style="display:block">
    <div class=muted style="margin-bottom:.35rem">Rooms</div>
    <div id=roomsWrap style="display:grid;grid-template-columns:repeat(auto-fill,minmax(220px,1fr));gap:.5rem"></div>
    <div style="display:flex;gap:.6rem;margin-top:.5rem">
      <button class=btn onclick="roomsSave()">Save Rooms</button>
      <div class="small" id=roomsMsg></div>
    </div>
  </div>
</div>

<!-- Drawer -->
<div class=drawer id=drawer>
  <div style="display:flex;align-items:center;gap:.6rem;margin-bottom:.5rem">
    <h4 style="margin:0;margin-right:auto">Tag details</h4>
    <button class=btn id=closeDrawer>Close</button>
  </div>
  <div class=row>
    <div style=flex:1><div class=muted>Tag ID</div><div id=dTag class=mono></div></div>
    <div style=flex:1><div class=muted>Room</div><div id=dRoom class=mono></div></div>
    <div style=flex:1><div class=muted>Strength</div><div id=dStr class=mono></div></div>
  </div>
  <div class=row style="display:block">
    <div class=muted style="margin-bottom:.35rem">Registration</div>
    <div class=grid2>
      <label class=input><input id=dReg placeholder="e.g., 152MH12345"></label>
      <button class="btn" id=saveReg>Save</button>
    </div>
  </div>
  <div class=row style="display:block">
    <div class=muted style="margin-bottom:.35rem">Move to room</div>
    <div class=grid2>
      <label class=input><input id=dMove type=number min=1 max=25 placeholder="Room #"></label>
      <button class=btn id=doMove>Move</button>
    </div>
  </div>
  <div class=row style="display:flex;justify-content:space-between">
    <button class=btn id=clearReg>Clear reg</button>
    <button class=btn id=forgetTag>Forget tag</button>
  </div>
</div>

<script>
let STATE={roomNames:[],tags:[],peers:[],logs:[],seenRaw:[]}; // tags: [{id,room,str,reg}]
let FILTER="";

// Tabs
document.querySelectorAll(".tab").forEach(t=>t.onclick=()=>{
  document.querySelectorAll(".tab").forEach(x=>x.classList.remove("active"));
  t.classList.add("active");
  const id=t.dataset.tab;
  wrap("wrapGrid", id==="grid");
  wrap("gridControls", id==="grid");
  wrap("wrapPeers", id==="peers");
  wrap("wrapLogs", id==="logs");
  wrap("wrapManage", id==="manage");
  if(id==="manage") renderManage();
});
function wrap(id,show){ document.getElementById(id).hidden=!show; }

// Search
document.getElementById("btnSearch").onclick=()=>{FILTER=Q().toUpperCase(); renderGrid();};
document.getElementById("q").oninput=()=>{FILTER=Q().toUpperCase(); renderGrid();};
const Q=()=>document.getElementById("q").value.trim();

// Grid
const gridEl=document.getElementById("grid");
function renderGrid(){
  gridEl.innerHTML="";
  const rooms = STATE.roomNames.length? STATE.roomNames : Array.from({length:5},(_,i)=>"Room "+(i+1));
  for(let r=1;r<=rooms.length;r++){
    const card=div("card");
    const h=document.createElement("h3");
    h.textContent=rooms[r-1]||("Room "+r);
    h.title="Rename room"; h.onclick=async()=>{
      const v=prompt("Rename room "+r, h.textContent);
      if(!v) return;
      await post("/setRoom",{idx:r,name:v});
      pollOnce();
    };
    card.appendChild(h);
    card.appendChild(div("badge", STATE.tags.filter(t=>t.room===r).length));
    const chips=div("chips");
    const here=STATE.tags.filter(t=>t.room===r).sort((a,b)=> String(a.reg||a.id).localeCompare(String(b.reg||b.id)));
    for(const t of here){
      const label=t.reg||t.id;
      if(FILTER && !String(label).toUpperCase().includes(FILTER) && !String(t.id).includes(FILTER)) continue;
      const ch=div("chip", label); ch.title="S="+t.str; ch.onclick=()=>openDrawer(t);
      chips.appendChild(ch);
    }
    card.appendChild(chips); gridEl.appendChild(card);
  }
}

// Peers & Logs
function renderPeers(){
  const wrap=document.getElementById("wrapPeers"); wrap.innerHTML="";
  if(!STATE.peers.length){ wrap.innerHTML='<div class=row>No peers yet.</div>'; return; }
  for(const p of STATE.peers){
    wrap.appendChild(rowHTML(`<div class="mono tag">${p.mac}</div><div>room <b>${p.room||"?"}</b></div><div class=muted>seen ${p.ago||"?"} ago</div>`));
  }
}
function renderLogs(){
  const wrap=document.getElementById("wrapLogs"); wrap.innerHTML="";
  for(const L of STATE.logs){ wrap.appendChild(rowText(L,"mono")); }
}

// Manage
function renderManage(){
  const RW=document.getElementById("roomsWrap"); RW.innerHTML="";
  const rooms=STATE.roomNames.length?STATE.roomNames:Array.from({length:5},(_,i)=>"Room "+(i+1));
  rooms.forEach((nm,i)=>{
    const w=div("",`<label class="input"><input id="rname_${i+1}" value="${nm||("Room "+(i+1))}"></label>`);
    RW.appendChild(w);
  });
  const SW=document.getElementById("seenWrap"); SW.innerHTML="";
  for(const raw of STATE.seenRaw||[]){
    const b=div("chip",raw); b.onclick=()=>{document.getElementById("rawIn").value=raw;};
    SW.appendChild(b);
  }
}
async function roomsSave(){
  const rooms=[];
  const nodes=[...document.querySelectorAll('[id^="rname_"]')];
  nodes.forEach(n=>rooms.push(n.value.trim()));
  await post("/roomsSave",{json:JSON.stringify(rooms)});
  document.getElementById("roomsMsg").textContent="Saved.";
  setTimeout(()=>document.getElementById("roomsMsg").textContent="",1500);
}
async function aliasSet(){
  const raw=document.getElementById("rawIn").value.trim();
  const tag=parseInt(document.getElementById("rawTag").value||"0");
  if(!raw||!tag) return;
  await post("/aliasSet",{raw,tag});
  document.getElementById("aliasMsg").textContent="Alias saved.";
  setTimeout(()=>document.getElementById("aliasMsg").textContent="",1500);
  pollOnce();
}
async function aliasDel(){
  const raw=document.getElementById("rawIn").value.trim(); if(!raw) return;
  await post("/aliasDel",{raw});
  document.getElementById("aliasMsg").textContent="Alias deleted.";
  setTimeout(()=>document.getElementById("aliasMsg").textContent="",1500);
  pollOnce();
}

// Drawer
let curTag=null;
const drawer=document.getElementById("drawer");
const dTag=document.getElementById("dTag"), dRoom=document.getElementById("dRoom"), dStr=document.getElementById("dStr");
const dReg=document.getElementById("dReg"), dMove=document.getElementById("dMove");
function openDrawer(t){curTag=t.id; dTag.textContent=t.id; dRoom.textContent=t.room; dStr.textContent=t.str; dReg.value=t.reg||""; dMove.value=""; drawer.classList.add("open");}
document.getElementById("closeDrawer").onclick=()=>drawer.classList.remove("open");
document.getElementById("saveReg").onclick=async()=>{ if(!curTag) return; await post("/setReg",{tag:curTag,reg:dReg.value.trim()}); drawer.classList.remove("open"); pollOnce(); };
document.getElementById("clearReg").onclick=async()=>{ if(!curTag) return; await post("/setReg",{tag:curTag,reg:""}); drawer.classList.remove("open"); pollOnce(); };
document.getElementById("doMove").onclick=async()=>{ if(!curTag) return; const r=parseInt(dMove.value||"0"); if(r>0) await post("/moveTag",{tag:curTag,room:r}); drawer.classList.remove("open"); pollOnce(); };
document.getElementById("forgetTag").onclick=async()=>{ if(!curTag) return; await post("/forgetTag",{tag:curTag}); drawer.classList.remove("open"); pollOnce(); };

// Fetch helpers
async function post(path, obj){
  const b=new URLSearchParams(); for(const k in obj) b.append(k,obj[k]);
  await fetch(path,{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:b.toString()});
}
function div(c,html){ const d=document.createElement("div"); if(c) d.className=c; if(html!==undefined) d.innerHTML=html; return d; }
function rowHTML(html){ const d=div("row"); d.innerHTML=html; return d; }
function rowText(t,extra){ const d=div("row"+(extra?(" "+extra):"")); d.textContent=t; return d; }

// Poll
async function poll(){
  try{
    const r=await fetch("/state.json",{cache:"no-store"});
    if(r.ok){ STATE=await r.json(); document.getElementById("status").textContent=STATE.status||"online"; renderGrid(); renderPeers(); renderLogs(); }
  }catch(e){}
  setTimeout(poll,500);
}
async function pollOnce(){ try{ const r=await fetch("/state.json",{cache:"no-store"}); if(r.ok){ STATE=await r.json(); renderGrid(); renderPeers(); renderLogs(); renderManage(); } }catch(e){} }

poll();
</script>
)HTML";

// -------- small helpers --------
static String formValue(const String& body, const char* key){
  String k=String(key)+'=';
  int p=body.indexOf(k); if(p<0) return "";
  int e=body.indexOf('&', p+k.length()); if(e<0) e=body.length();
  String v = body.substring(p+k.length(), e); v.replace("+"," ");
  String o; o.reserve(v.length());
  for(size_t i=0;i<v.length();++i){
    if(v[i]=='%' && i+2<v.length()){ int v8=strtol(v.substring(i+1,i+3).c_str(),nullptr,16); o+=(char)v8; i+=2; }
    else o+=v[i];
  }
  return o;
}
static void sendStatus(){
  pushLog(String("[AP] ")+AP_SSID+"  IP: "+WiFi.softAPIP().toString());
}

// -------- HTTP handlers --------
WebServer server(80);

void handleStateJson(){
  WiFiClient client = server.client();
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");

  String s = "{\"status\":\"AP "+String(AP_SSID)+" | UART feed\",\"roomNames\":[";
  for(int i=0;i<MAX_ROOMS;i++){ if(i) s+=','; s+='\"'+jsonEscape(roomNames[i])+'\"'; }
  s += "],\"tags\":[";
  bool first=true;
  for(int id=1; id<=MAX_TAG_ID; id++){
    if(tagRoom[id]){
      if(!first) s+=','; first=false;
      s += "{\"id\":"+String(id)+",\"room\":"+String(tagRoom[id])+",\"str\":"+String(tagStr[id])+",\"reg\":\""+jsonEscape(regByTag[id])+"\"}";
    }
  }
  s += "],\"peers\":[";
  for(int i=0;i<peerCount;i++){ if(i) s+=','; uint32_t ago=(millis()-peers[i].lastMs)/1000;
    s += "{\"mac\":\""+jsonEscape(peers[i].mac)+"\",\"room\":"+String(peers[i].room)+",\"ago\":\""+String(ago)+"s\"}";
  }
  s += "],\"logs\":[";
  bool f2=true; for(int i=0;i<LOG_RING;i++){ int idx=(logPtr+i)%LOG_RING; if(logBuf[idx].length()){
    if(!f2) s+=','; f2=false; s+='\"'+jsonEscape(logBuf[idx])+'\"'; } }
  s += "],\"seenRaw\":[";
  for(int i=0;i<seenCount;i++){ if(i) s+=','; s+='\"'+jsonEscape(seenRaw[i])+'\"'; }
  s += "]}";
  client.print(s);
}
void handleSetReg(){
  String body = server.hasArg("plain")? server.arg("plain") : "";
  int id  = formValue(body,"tag").toInt();
  String reg = formValue(body,"reg");
  if(id>=1 && id<=MAX_TAG_ID){ regByTag[id]=reg; regsSave(); pushLog(String("[REG] tag ")+id+" = "+reg); }
  server.send(200,"text/plain","OK");
}
void handleMoveTag(){
  String body = server.hasArg("plain")? server.arg("plain") : "";
  int id=formValue(body,"tag").toInt();
  int room=formValue(body,"room").toInt();
  if(id>=1 && id<=MAX_TAG_ID && room>=1 && room<=MAX_ROOMS){ tagRoom[id]=room; pushLog(String("[MOVE] tag ")+id+" -> room "+room); }
  server.send(200,"text/plain","OK");
}
void handleForgetTag(){
  String body = server.hasArg("plain")? server.arg("plain") : "";
  int id=formValue(body,"tag").toInt();
  if(id>=1 && id<=MAX_TAG_ID){ tagRoom[id]=0; tagStr[id]=0; pushLog(String("[FORGET] tag ")+id); }
  server.send(200,"text/plain","OK");
}
void handleSetRoom(){
  String body = server.hasArg("plain")? server.arg("plain") : "";
  int idx=formValue(body,"idx").toInt();
  String nm=formValue(body,"name");
  if(idx>=1 && idx<=MAX_ROOMS){ roomNames[idx-1]=nm; pushLog(String("[ROOM] ")+idx+" = "+nm); }
  server.send(200,"text/plain","OK");
}
void handleRoomsSave(){
  String body = server.hasArg("plain")? server.arg("plain") : "";
  String j = formValue(body,"json");
  if(j.length()){
    int i=0, r=0; while(i<(int)j.length() && r<MAX_ROOMS){
      int q1=j.indexOf('"', i); if(q1<0) break; int q2=j.indexOf('"', q1+1); if(q2<0) break;
      roomNames[r++] = j.substring(q1+1,q2);
      i = q2+1;
    }
  }
  roomsSave();
  server.send(200,"text/plain","OK");
}
void handleAliasSet(){
  String body = server.hasArg("plain")? server.arg("plain") : "";
  String raw=formValue(body,"raw"); uint16_t tag=(uint16_t)formValue(body,"tag").toInt();
  if(raw.length() && tag>0){ aliasSet(raw, tag); }
  server.send(200,"text/plain","OK");
}
void handleAliasDel(){
  String body = server.hasArg("plain")? server.arg("plain") : "";
  String raw=formValue(body,"raw"); if(raw.length()) aliasDel(raw);
  server.send(200,"text/plain","OK");
}

// -------- UART2 parsing --------
volatile bool u2Flag=false; String u2Q;

static void parseLine(String s){
  s.trim(); if(!s.length()) return;

  if(s.startsWith("HELLO")){
    String mac, roomS; int mi=s.indexOf("mac="), ri=s.indexOf("room=");
    if(mi>=0){ mac=s.substring(mi+4); int c=mac.indexOf(','); if(c>0) mac=mac.substring(0,c); }
    if(ri>=0){ roomS=s.substring(ri+5); int c=roomS.indexOf(','); if(c>0) roomS=roomS.substring(0,c); }
    int room=roomS.toInt();
    int idx=-1; for(int i=0;i<peerCount;i++) if(peers[i].mac==mac){ idx=i; break; }
    if(idx<0 && peerCount<MAX_PEERS){ idx=peerCount++; peers[idx].mac=mac; peers[idx].room=0; peers[idx].lastMs=0; }
    if(idx>=0){ peers[idx].lastMs=millis(); if(room>0) peers[idx].room=room; }
    pushLog(s); return;
  }

  if(s.startsWith("LOG: ICE UNMAPPED ")){
    String raw = s.substring(strlen("LOG: ICE UNMAPPED ")); raw.trim();
    if(raw.length()){
      bool have=false; for(int i=0;i<seenCount;i++) if(seenRaw[i]==raw){ have=true; break; }
      if(!have && seenCount<SEEN_MAX){ seenRaw[seenCount++]=raw; }
    }
    pushLog(s); return;
  }

  // Accept "UI:" (or any prefix ending with ':')
  int colon=s.indexOf(':'); if(colon>=0) s=s.substring(colon+1);

  static bool seen[MAX_TAG_ID+1]; memset(seen,0,sizeof(seen));
  int p=0;
  while(p<s.length()){
    int comma=s.indexOf(',',p); if(comma<0) comma=s.length();
    String tok=s.substring(p,comma); tok.trim();
    int dot=tok.indexOf('.'); int at=tok.indexOf('@');
    if(dot>0 && at>dot){
      int roomId=tok.substring(0,dot).toInt();
      int tagId =tok.substring(dot+1,at).toInt();
      int str   =tok.substring(at+1).toInt();
      if(roomId>=1 && roomId<=MAX_ROOMS && tagId>=1 && tagId<=MAX_TAG_ID){
        if(str<0) str=0; if(str>100) str=100;
        tagRoom[tagId]=roomId; tagStr[tagId]=str; seen[tagId]=true;
      }
    }
    p=comma+1;
  }
  pushLog("[UI] update received");
}
static void pumpUART2(){
  while(Serial2.available()){
    char c=(char)Serial2.read();
    if(c=='\n') u2Flag=true;
    if(c!='\r') u2Q+=c;
    if(u2Q.length()>4096) u2Q.remove(0,1024);
  }
  if(u2Flag){
    u2Flag=false;
    while(true){
      int nl=u2Q.indexOf('\n'); if(nl<0) break;
      String line=u2Q.substring(0,nl); u2Q.remove(0,nl+1);
      parseLine(line);
    }
  }
}

// -------- Setup / Loop --------
void setup(){
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, UI_RX2, UI_TX2);

  FSYS.begin(true);
  roomsLoad(); aliasLoad(); regsLoad();

  for(int i=1;i<=MAX_TAG_ID;i++){ tagRoom[i]=0; tagStr[i]=0; }

  WiFi.mode(WIFI_AP);
  bool ok=WiFi.softAP(AP_SSID, AP_PASS);
  (void)ok;
  pushLog(String("[UI] AP ready: http://")+WiFi.softAPIP().toString()+"/");

  server.on("/", HTTP_GET, [](){ server.send_P(200,"text/html; charset=utf-8", INDEX_HTML); });
  server.on("/state.json",  HTTP_GET, handleStateJson);
  server.on("/setReg",      HTTP_POST, handleSetReg);
  server.on("/moveTag",     HTTP_POST, handleMoveTag);
  server.on("/forgetTag",   HTTP_POST, handleForgetTag);
  server.on("/setRoom",     HTTP_POST, handleSetRoom);
  server.on("/roomsSave",   HTTP_POST, handleRoomsSave);
  server.on("/aliasSet",    HTTP_POST, handleAliasSet);
  server.on("/aliasDel",    HTTP_POST, handleAliasDel);
  server.onNotFound([](){ server.send(404,"text/plain","404"); });
  server.begin();
}

void loop(){
  pumpUART2();
  server.handleClient();
}
