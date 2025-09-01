/*
  ui_web_s3_ap.ino
  ESP32-S3 “UI Brain” (headless) with Wi-Fi Web UI + UART feed from tail SMOKE.

  - Starts AP:  SSID: KeyGrid_UI    PASS: keygrid123
  - Serves a single page at "/" (from PROGMEM)
  - Browser polls /state.json (500 ms)
  - Reads UART lines from tail SMOKE:
      "UI: <room>.<tag>@<strength>, ..."   (presence snapshot)
      "HELLO,mac=AA:BB:CC:DD:EE:FF,room=N"

  Hardware: ESP32-S3 Dev Module
  UART (from SMOKE tail) on Serial2 (RX2=16, TX2=17 default; only RX2 used)
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// ------------ Pins (set to your wiring) ------------
#define UI_RX2 16
#define UI_TX2 17   // not used, OK to leave unconnected

// ------------ Wi-Fi AP ------------
static const char* AP_SSID = "KeyGrid_UI";
static const char* AP_PASS = "keygrid123";

// ------------ Limits ------------
static const int MAX_ROOMS  = 25;
static const int MAX_TAG_ID = 1024;
static const int LOG_RING   = 200;
static const int MAX_PEERS  = 40;

// ------------ State ------------
String   roomNames[MAX_ROOMS];
uint16_t tagRoom[MAX_TAG_ID + 1];
uint8_t  tagStr [MAX_TAG_ID + 1];
String   regByTag[MAX_TAG_ID + 1];

struct PeerInfo { String mac; int room; uint32_t lastMs; };
PeerInfo peers[MAX_PEERS]; int peerCount=0;

String logBuf[LOG_RING]; int logPtr=0;
inline void pushLog(const String& s){ logBuf[logPtr]=s; logPtr=(logPtr+1)%LOG_RING; Serial.println(s); }

// ------------ Utils ------------
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

// ------------ WebServer ------------
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
.card h3{margin:.1rem 0 .6rem;font-size:18px}
.badge{position:absolute;top:.6rem;right:.6rem;font-size:13px;min-width:28px;text-align:center;background:linear-gradient(180deg,#2b3949,#1b2836);color:#e9f6ff;padding:.35rem .55rem;border-radius:999px;border:1px solid rgba(255,255,255,.07);box-shadow:var(--shadow)}
.chips{display:flex;flex-wrap:wrap;gap:.6rem}
.chip{padding:.55rem .7rem;border-radius:999px;background:linear-gradient(180deg,#0f1620,#0c141d);color:#cfe6ff;border:1px solid rgba(255,255,255,.07);box-shadow:inset 0 0 0 1px rgba(99,215,255,.15),var(--shadow);cursor:pointer}
.list{padding:.5rem .9rem 1.2rem;display:grid;gap:.6rem}
.row{display:flex;gap:.8rem;align-items:center;padding:.7rem .85rem;border-radius:12px;background:var(--panel);border:1px solid rgba(255,255,255,.06);box-shadow:var(--shadow);color:#d3e1f0}
.mono{font-family:ui-monospace,Menlo,Consolas,monospace}
.tag{padding:.2rem .45rem;border:1px solid rgba(255,255,255,.1);border-radius:8px;color:#d6f3ff;background:#0e1a24}
</style>
<div class=top>
  <div class=brand>Key Grid</div>
  <div class=tabs>
    <div class="tab active" data-tab=grid>Grid</div>
    <div class=tab data-tab=peers>Peers</div>
    <div class=tab data-tab=logs>Logs</div>
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

<script>
let STATE={roomNames:[],tags:[],peers:[],logs:[]}; // tags: [{id,room,str,reg}]
let FILTER="";
const tabs=document.querySelectorAll(".tab");
tabs.forEach(t=>t.onclick=()=>{
  tabs.forEach(x=>x.classList.remove("active"));
  t.classList.add("active");
  const id=t.dataset.tab;
  document.getElementById("wrapGrid").hidden=(id!=="grid");
  document.getElementById("gridControls").hidden=(id!=="grid");
  document.getElementById("wrapPeers").hidden=(id!=="peers");
  document.getElementById("wrapLogs").hidden=(id!=="logs");
});
document.getElementById("btnSearch").onclick=()=>{FILTER=document.getElementById("q").value.trim().toUpperCase(); renderGrid();};
document.getElementById("q").oninput=()=>{FILTER=document.getElementById("q").value.trim().toUpperCase(); renderGrid();};

const gridEl=document.getElementById("grid");
function renderGrid(){
  gridEl.innerHTML="";
  const rooms = STATE.roomNames.length? STATE.roomNames : Array.from({length:5},(_,i)=>"Room "+(i+1));
  for (let r=1;r<=rooms.length;r++){
    const card=document.createElement("div"); card.className="card";
    const h=document.createElement("h3"); h.textContent=rooms[r-1]||("Room "+r); card.appendChild(h);
    const b=document.createElement("div"); b.className="badge"; b.textContent=STATE.tags.filter(t=>t.room===r).length; card.appendChild(b);
    const chips=document.createElement("div"); chips.className="chips";
    const here=STATE.tags.filter(t=>t.room===r).sort((a,b)=> String(a.reg||a.id).localeCompare(String(b.reg||b.id)));
    for(const t of here){
      const label = t.reg || t.id;
      if(FILTER && !String(label).toUpperCase().includes(FILTER) && !String(t.id).includes(FILTER)) continue;
      const ch=document.createElement("div"); ch.className="chip"; ch.textContent=label; ch.title="S="+t.str;
      ch.onclick=()=>alert('Tag '+t.id+'  Room '+t.room+'  S='+t.str);
      chips.appendChild(ch);
    }
    card.appendChild(chips); gridEl.appendChild(card);
  }
}
function renderPeers(){
  const wrap=document.getElementById("wrapPeers"); wrap.innerHTML="";
  if(!STATE.peers.length){ wrap.innerHTML='<div class=row>No peers yet.</div>'; return; }
  for(const p of STATE.peers){
    const row=document.createElement("div"); row.className="row";
    row.innerHTML=`<div class="mono tag">${p.mac}</div><div>room <b>${p.room||"?"}</b></div><div class=muted>seen ${p.ago||"?"} ago</div>`;
    wrap.appendChild(row);
  }
}
function renderLogs(){
  const wrap=document.getElementById("wrapLogs"); wrap.innerHTML="";
  for(const L of STATE.logs){ const row=document.createElement("div"); row.className="row mono"; row.textContent=L; wrap.appendChild(row); }
}

async function poll(){
  try{
    const r=await fetch("/state.json",{cache:"no-store"});
    if(r.ok){ STATE=await r.json(); document.getElementById("status").textContent=STATE.status||"online"; renderGrid(); renderPeers(); renderLogs(); }
  }catch(e){}
  setTimeout(poll,500);
}
poll();
</script>
)HTML";

WebServer::THandlerFunction notFound = [](){ server.send(404,"text/plain","404"); };

// /state.json — snapshot
void handleStateJson(){
  WiFiClient client = server.client();
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");

  String s = "{\"status\":\"AP ";
  s += AP_SSID; s += " | UART feed\",\"roomNames\":[";
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
  s += "]}";
  client.print(s);
}

// simple x-www-form-urlencoded parser (used later if you add POST endpoints)
static String urlValue(const String& body, const char* key){
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

// ------------ UART parsing ------------
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

  // "UI:" or "DATA:UI:"
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

// ------------ Setup / Loop ------------
void setup(){
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, UI_RX2, UI_TX2);

  setDefaultRooms();
  for(int i=1;i<=MAX_TAG_ID;i++){ tagRoom[i]=0; tagStr[i]=0; regByTag[i]=""; }

  WiFi.mode(WIFI_AP);
  bool ok=WiFi.softAP(AP_SSID, AP_PASS);
  pushLog(String("[AP] ")+(ok?"started ":"FAILED ") + String(AP_SSID) + "  IP: " + WiFi.softAPIP().toString());

  server.on("/", HTTP_GET, [](){ server.send_P(200,"text/html; charset=utf-8", INDEX_HTML); });
  server.on("/state.json", HTTP_GET, handleStateJson);
  server.onNotFound(notFound);
  server.begin();
  pushLog("[UI] ready → http://"+WiFi.softAPIP().toString()+"/");
}

void loop(){
  pumpUART2();
  server.handleClient();
}
