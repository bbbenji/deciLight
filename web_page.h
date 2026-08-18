/*
 * deciLight - the control page served to the browser
 *
 * Kept as one self-contained document with no external requests, so it loads
 * over the unit's own access point with no internet connection available.
 * Stored in flash rather than RAM.
 */

#ifndef DECILIGHT_WEB_PAGE_H
#define DECILIGHT_WEB_PAGE_H

#include <Arduino.h>

const char WEB_PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#12141a">
<title>deciLight</title>
<style>
:root{--bg:#12141a;--card:#1c1f28;--line:#2c3140;--text:#e8eaf0;--dim:#8b92a6;
--quiet:#00c853;--warn:#ffd600;--loud:#ff3d3d}
*{box-sizing:border-box}
body{margin:0;padding:16px;background:var(--bg);color:var(--text);
font:16px/1.5 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
max-width:520px;margin-inline:auto;-webkit-text-size-adjust:100%}
h1{font-size:15px;font-weight:600;letter-spacing:.14em;text-transform:uppercase;
color:var(--dim);margin:4px 0 16px}
.card{background:var(--card);border:1px solid var(--line);border-radius:14px;
padding:18px;margin-bottom:14px}
.level{display:flex;align-items:baseline;gap:12px}
#dot{width:16px;height:16px;border-radius:50%;background:var(--dim);flex:none;
align-self:center;transition:background .25s}
#db{font-size:56px;font-weight:650;font-variant-numeric:tabular-nums;line-height:1}
#units{color:var(--dim);font-size:15px}
#note{color:var(--dim);font-size:13px;min-height:1.5em;margin-top:6px}
.meter{--a:12%;--b:38%;position:relative;height:8px;border-radius:4px;margin-top:16px;
background:linear-gradient(90deg,var(--quiet) 0 var(--a),var(--warn) var(--a) var(--b),var(--loud) var(--b) 100%)}
#needle{position:absolute;top:-4px;width:3px;height:16px;border-radius:2px;
background:var(--text);box-shadow:0 0 0 2px var(--card);transition:left .3s}
.scale{display:flex;justify-content:space-between;color:var(--dim);
font-size:12px;margin-top:6px}
label{display:block;font-size:13px;color:var(--dim);margin:16px 0 6px}
label:first-child{margin-top:0}
label b{color:var(--text);font-weight:600;font-variant-numeric:tabular-nums}
input[type=range]{width:100%;margin:0;accent-color:#5b8cff;height:28px}
.row{display:flex;gap:8px}
button{flex:1;padding:11px 8px;border-radius:10px;border:1px solid var(--line);
background:#242835;color:var(--text);font:inherit;font-size:14px;cursor:pointer;
-webkit-tap-highlight-color:transparent}
button:active{transform:scale(.97)}
button.on{background:#5b8cff;border-color:#5b8cff;color:#0d1017;font-weight:600}
.sw{display:grid;grid-template-columns:repeat(8,1fr);gap:8px;margin-top:12px}
.sw i{aspect-ratio:1;border-radius:8px;border:1px solid #0006;cursor:pointer}
.sw i:active{transform:scale(.9)}
details{margin-top:2px}
summary{color:var(--dim);font-size:13px;cursor:pointer;padding:4px 0}
input[type=text],input[type=password]{width:100%;padding:10px;margin-top:8px;
border-radius:10px;border:1px solid var(--line);background:#242835;
color:var(--text);font:inherit;font-size:15px}
.net{color:var(--dim);font-size:12px;text-align:center;margin:18px 0 4px}
input[type=file]{width:100%;margin-top:8px;color:var(--dim);font-size:13px}
.bar{height:6px;border-radius:3px;background:#242835;margin-top:12px;overflow:hidden}
.bar i{display:block;height:100%;width:0;background:#5b8cff;transition:width .2s}
#otanote{color:var(--dim);font-size:13px;margin-top:8px;min-height:1.2em}
</style>
</head>
<body>
<h1>deciLight</h1>

<div class="card">
  <div class="level"><span id="dot"></span><span id="db">--</span><span id="units">dBA</span></div>
  <div id="note"></div>
  <div class="meter" id="meter"><div id="needle"></div></div>
  <div class="scale"><span>30</span><span id="lo"></span><span id="hi"></span><span>110</span></div>
</div>

<div class="card">
  <label>Quiet below <b><span id="vmin"></span> dB</b></label>
  <input type="range" id="smin" min="30" max="110">
  <label>Too loud above <b><span id="vmax"></span> dB</b></label>
  <input type="range" id="smax" min="30" max="110">
  <label>Brightness <b><span id="vbri"></span>%</b></label>
  <input type="range" id="sbri" min="10" max="255">
</div>

<div class="card">
  <div class="row">
    <button id="mauto">Auto</button>
    <button id="moff">Off</button>
  </div>
  <div class="sw" id="sw"></div>
</div>

<div class="card">
  <details>
    <summary>Join a WiFi network</summary>
    <input type="text" id="ssid" placeholder="Network name" autocapitalize="off" autocorrect="off">
    <input type="password" id="pass" placeholder="Password">
    <div class="row" style="margin-top:12px"><button id="wsave">Save and restart</button></div>
  </details>
</div>

<div class="card" id="otacard" hidden>
  <details>
    <summary>Update firmware</summary>
    <input type="password" id="otapass" placeholder="Update password">
    <input type="file" id="otafile" accept=".bin">
    <div class="row" style="margin-top:12px"><button id="otago">Upload and restart</button></div>
    <div class="bar" id="otabar" hidden><i id="otafill"></i></div>
    <div id="otanote"></div>
  </details>
</div>

<div class="net" id="net"></div>

<script>
var COLORS=["ff0000","ff4500","ff6347","ffa500","ffff00","00ff00","90ee90","008b8b",
            "00ffff","87ceeb","0000ff","9370db","800080","dda0dd","40e0d0","ffffff"];
var ZONE={quiet:"--quiet",warn:"--warn",loud:"--loud"};
var $=function(i){return document.getElementById(i)};
var busy=0, dragging=null;

var sw=$("sw");
COLORS.forEach(function(c){
  var e=document.createElement("i");
  e.style.background="#"+c;
  e.onclick=function(){post("/api/mode","mode=manual&color="+c)};
  sw.appendChild(e);
});
$("mauto").onclick=function(){post("/api/mode","mode=auto")};
$("moff").onclick=function(){post("/api/mode","mode=off")};
$("wsave").onclick=function(){
  post("/api/wifi","ssid="+encodeURIComponent($("ssid").value)+
                   "&pass="+encodeURIComponent($("pass").value));
  $("net").textContent="Restarting...";
};

[["smin","dbMin"],["smax","dbMax"],["sbri","brightness"]].forEach(function(p){
  var el=$(p[0]);
  el.addEventListener("input",function(){dragging=p[0];paint()});
  el.addEventListener("change",function(){
    dragging=null;
    post("/api/set",p[1]+"="+el.value);
  });
});

function post(url,body){
  busy=1;
  var x=new XMLHttpRequest();
  x.open("POST",url,true);
  x.setRequestHeader("Content-Type","application/x-www-form-urlencoded");
  x.onload=function(){busy=0; if(x.status==200) render(JSON.parse(x.responseText))};
  x.onerror=function(){busy=0};
  x.send(body);
}

function paint(){
  var lo=+$("smin").value, hi=+$("smax").value;
  var a=(lo-30)/80*100, b=(hi-30)/80*100;
  $("meter").style.setProperty("--a",a+"%");
  $("meter").style.setProperty("--b",b+"%");
  $("vmin").textContent=lo; $("vmax").textContent=hi;
  $("lo").textContent=lo; $("hi").textContent=hi;
  $("vbri").textContent=Math.round($("sbri").value/255*100);
}

function render(s){
  $("db").textContent=s.db.toFixed(1);
  $("units").textContent=s.units;
  $("dot").style.background=s.mode=="auto"?"var("+(ZONE[s.zone]||"--dim")+")"
          :(s.mode=="manual"?"#"+s.color:"var(--dim)");
  $("note").textContent=s.quality=="overload"?"Above the microphone's range"
          :s.quality=="quiet"?"Below the microphone's noise floor"
          :s.mode=="manual"?"Holding a fixed colour"
          :s.mode=="off"?"Light is off, still measuring":"";
  $("needle").style.left="calc("+Math.max(0,Math.min(100,(s.db-30)/80*100))+"% - 1.5px)";
  $("mauto").className=s.mode=="auto"?"on":"";
  $("moff").className=s.mode=="off"?"on":"";
  if(dragging!="smin") $("smin").value=s.dbMin;
  if(dragging!="smax") $("smax").value=s.dbMax;
  if(dragging!="sbri") $("sbri").value=s.brightness;
  paint();
  $("otacard").hidden=!s.ota;
  $("net").textContent=s.net=="ap"
    ? "Access point "+s.ssid+" \u00b7 "+s.ip
    : "Connected to "+s.ssid+" \u00b7 "+s.ip;
}

$("otago").onclick=function(){
  var f=$("otafile").files[0];
  if(!f){ $("otanote").textContent="Choose a .bin file first."; return; }
  var pass=$("otapass").value;
  if(!pass){ $("otanote").textContent="Enter the update password."; return; }

  var fd=new FormData(); fd.append("firmware",f,f.name);
  var x=new XMLHttpRequest();
  // Credentials go on open() rather than waiting for a 401 challenge, which
  // browsers do not reliably surface for XHR.
  x.open("POST","/api/update",true,"decilight",pass);
  busy=1;
  $("otabar").hidden=false;
  x.upload.onprogress=function(e){
    if(e.lengthComputable) $("otafill").style.width=(e.loaded/e.total*100)+"%";
  };
  x.onload=function(){
    busy=0;
    if(x.status==200){
      $("otanote").textContent="Uploaded. The light is restarting.";
    }else{
      var msg="Update failed.";
      try{ msg=JSON.parse(x.responseText).error||msg; }catch(_){}
      $("otanote").textContent=msg;
      $("otafill").style.width="0";
    }
  };
  x.onerror=function(){
    busy=0;
    // The unit resets as soon as it has the image, so the connection dropping
    // at the very end is the expected ending, not a failure.
    $("otanote").textContent="Connection closed - if the upload completed, the light is restarting.";
  };
  x.send(fd);
};

function poll(){
  if(busy||dragging) return;
  var x=new XMLHttpRequest();
  x.open("GET","/api/state",true);
  x.onload=function(){if(x.status==200) render(JSON.parse(x.responseText))};
  x.send();
}
poll(); setInterval(poll,500);
</script>
</body>
</html>)HTML";

#endif  // DECILIGHT_WEB_PAGE_H
