#!/usr/bin/env python3
"""
STM32 Terminal — Web-based UART interface for STM32F407 robot
Run: python3 stm32_terminal.py [PORT] [BAUD]
Then open: http://localhost:8765
"""

import sys
import serial
import serial.tools.list_ports
import threading
import time
import json
import argparse
from http.server import HTTPServer, BaseHTTPRequestHandler
from collections import deque

# ─── Config ───────────────────────────────────────────────────────────────────
DEFAULT_BAUD = 9600 #change in main.c as well
WEB_PORT     = 8765
LOG_MAXLEN   = 500

# ─── Global State ─────────────────────────────────────────────────────────────
ser           = None
ser_lock      = threading.Lock()
log_buf       = deque(maxlen=LOG_MAXLEN)
log_lock      = threading.Lock()
last_fetch_id = 0

# Quick-command presets matching the STM32 protocol
QUICK_CMDS = [
    ("GO",   "GO",   "Start walking"),
    ("STOP", "STOP", "Stop & disable servos"),
    ("FWD",  "FWD",  "Forward direction"),
    ("BACK", "BACK", "Backward direction"),
    ("HELP", "HELP", "List commands"),
]

# ─── Serial helpers ───────────────────────────────────────────────────────────
def list_ports():
    return [p.device for p in serial.tools.list_ports.comports()]

def open_serial(port, baud):
    global ser
    with ser_lock:
        if ser and ser.is_open:
            ser.close()
        ser = serial.Serial(port, baud, timeout=0.05)

def send_command(cmd):
    with ser_lock:
        if ser and ser.is_open:
            line = cmd.strip() + "\n"
            ser.write(line.encode())
            _log("TX", cmd.strip())
            return True
    return False

def _log(direction, text):
    with log_lock:
        global last_fetch_id
        last_fetch_id += 1
        log_buf.append({
            "id":   last_fetch_id,
            "ts":   time.strftime("%H:%M:%S"),
            "dir":  direction,   # "TX", "RX", "SYS"
            "text": text.rstrip()
        })

# ─── Serial reader thread ─────────────────────────────────────────────────────
def serial_reader():
    while True:
        try:
            with ser_lock:
                s = ser
            if s and s.is_open:
                line = s.readline()
                if line:
                    _log("RX", line.decode(errors="replace").rstrip())
            else:
                time.sleep(0.2)
        except Exception as e:
            _log("SYS", f"Read error: {e}")
            time.sleep(1)

# ─── HTTP handler ─────────────────────────────────────────────────────────────
HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>STM32 Terminal</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Orbitron:wght@400;700;900&display=swap" rel="stylesheet">
<style>
  :root {
    --bg:       #080c0a;
    --panel:    #0d1410;
    --border:   #1a3028;
    --green:    #00ff88;
    --green-dim:#00c868;
    --amber:    #ffb700;
    --red:      #ff3c3c;
    --cyan:     #00e5ff;
    --text:     #b0d8c0;
    --text-dim: #4a7058;
    --glow:     0 0 8px #00ff8866;
    --glow-hard:0 0 16px #00ff88aa;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }

  body {
    background: var(--bg);
    color: var(--text);
    font-family: 'Share Tech Mono', monospace;
    font-size: 13px;
    height: 100vh;
    display: flex;
    flex-direction: column;
    overflow: hidden;
  }

  /* ── Header ── */
  header {
    display: flex;
    align-items: center;
    gap: 18px;
    padding: 10px 18px;
    border-bottom: 1px solid var(--border);
    background: var(--panel);
    flex-shrink: 0;
  }
  header h1 {
    font-family: 'Orbitron', monospace;
    font-size: 14px;
    font-weight: 900;
    letter-spacing: 3px;
    color: var(--green);
    text-shadow: var(--glow);
    white-space: nowrap;
  }
  .blink { animation: blink 1.1s step-end infinite; }
  @keyframes blink { 0%,100%{opacity:1} 50%{opacity:0} }

  .status-dot {
    width: 8px; height: 8px; border-radius: 50%;
    background: var(--red);
    box-shadow: 0 0 6px var(--red);
    flex-shrink: 0;
    transition: background 0.3s, box-shadow 0.3s;
  }
  .status-dot.connected {
    background: var(--green);
    box-shadow: 0 0 8px var(--green);
    animation: pulse 2s ease-in-out infinite;
  }
  @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:.5} }

  .status-label {
    font-size: 11px;
    color: var(--text-dim);
    letter-spacing: 1px;
  }
  .status-label span { color: var(--green); }

  header .spacer { flex: 1; }

  /* ── Connection bar ── */
  #conn-bar {
    display: flex;
    align-items: center;
    gap: 8px;
    padding: 8px 18px;
    border-bottom: 1px solid var(--border);
    background: #0a120e;
    flex-shrink: 0;
    flex-wrap: wrap;
  }
  #conn-bar label { color: var(--text-dim); font-size: 11px; letter-spacing:1px; }
  select, input[type=text], input[type=number] {
    background: var(--bg);
    border: 1px solid var(--border);
    color: var(--green);
    font-family: 'Share Tech Mono', monospace;
    font-size: 12px;
    padding: 4px 8px;
    outline: none;
    border-radius: 2px;
  }
  select:focus, input:focus { border-color: var(--green); box-shadow: var(--glow); }
  #port-select { min-width: 160px; }
  #baud-select { width: 100px; }

  button {
    font-family: 'Share Tech Mono', monospace;
    font-size: 12px;
    letter-spacing: 1px;
    padding: 5px 14px;
    border: 1px solid var(--green-dim);
    background: transparent;
    color: var(--green);
    cursor: pointer;
    border-radius: 2px;
    transition: all 0.15s;
  }
  button:hover { background: #00ff8820; box-shadow: var(--glow); }
  button:active { background: #00ff8840; }
  button.danger { border-color: var(--red); color: var(--red); }
  button.danger:hover { background: #ff3c3c20; box-shadow: 0 0 8px #ff3c3c66; }
  button.amber { border-color: var(--amber); color: var(--amber); }
  button.amber:hover { background: #ffb70020; }

  /* ── Main layout ── */
  main {
    display: flex;
    flex: 1;
    overflow: hidden;
    gap: 0;
  }

  /* ── Terminal panel ── */
  #terminal-wrap {
    flex: 1;
    display: flex;
    flex-direction: column;
    border-right: 1px solid var(--border);
    overflow: hidden;
  }
  #terminal-header {
    padding: 6px 14px;
    font-size: 10px;
    letter-spacing: 2px;
    color: var(--text-dim);
    border-bottom: 1px solid var(--border);
    display: flex;
    justify-content: space-between;
    align-items: center;
    flex-shrink: 0;
  }
  #log {
    flex: 1;
    overflow-y: auto;
    padding: 10px 14px;
    display: flex;
    flex-direction: column;
    gap: 1px;
    scrollbar-width: thin;
    scrollbar-color: var(--border) transparent;
  }
  #log::-webkit-scrollbar { width: 4px; }
  #log::-webkit-scrollbar-thumb { background: var(--border); }

  .log-line {
    display: flex;
    gap: 10px;
    align-items: baseline;
    line-height: 1.6;
    animation: fadein 0.15s ease;
  }
  @keyframes fadein { from{opacity:0;transform:translateY(2px)} to{opacity:1;transform:none} }

  .log-ts   { color: var(--text-dim); font-size: 11px; flex-shrink: 0; }
  .log-dir  { font-size: 10px; font-weight: bold; flex-shrink:0; width: 28px; text-align:center; letter-spacing:1px; }
  .log-text { word-break: break-all; }

  .dir-rx { color: var(--green); }
  .dir-tx { color: var(--amber); }
  .dir-sys{ color: var(--cyan); }

  .text-rx { color: var(--text); }
  .text-tx { color: var(--amber); }
  .text-sys{ color: var(--cyan); }

  /* OK/FAIL highlights */
  .hl-ok   { color: var(--green); text-shadow: var(--glow); }
  .hl-fail { color: var(--red);   text-shadow: 0 0 8px #ff3c3c88; }
  .hl-ready{ color: var(--cyan);  text-shadow: 0 0 6px #00e5ff88; }

  /* ── Input bar ── */
  #input-bar {
    display: flex;
    align-items: center;
    gap: 8px;
    padding: 8px 14px;
    border-top: 1px solid var(--border);
    background: var(--panel);
    flex-shrink: 0;
  }
  #prompt { color: var(--green); font-size: 14px; text-shadow: var(--glow); }
  #cmd-input {
    flex: 1;
    background: transparent;
    border: none;
    border-bottom: 1px solid var(--border);
    color: var(--green);
    font-family: 'Share Tech Mono', monospace;
    font-size: 13px;
    padding: 2px 4px;
    outline: none;
    caret-color: var(--green);
  }
  #cmd-input:focus { border-bottom-color: var(--green); box-shadow: 0 1px 0 var(--green); }
  #cmd-input::placeholder { color: var(--text-dim); }

  /* ── Right sidebar ── */
  #sidebar {
    width: 200px;
    display: flex;
    flex-direction: column;
    flex-shrink: 0;
    background: var(--panel);
    overflow-y: auto;
    scrollbar-width: thin;
    scrollbar-color: var(--border) transparent;
  }

  .sidebar-section {
    padding: 10px 12px;
    border-bottom: 1px solid var(--border);
  }
  .sidebar-section h3 {
    font-family: 'Orbitron', monospace;
    font-size: 9px;
    letter-spacing: 2px;
    color: var(--text-dim);
    margin-bottom: 8px;
  }

  .quick-btn {
    display: block;
    width: 100%;
    margin-bottom: 5px;
    padding: 6px 10px;
    text-align: left;
    font-size: 12px;
  }
  .quick-btn .qb-cmd  { color: var(--green); }
  .quick-btn .qb-desc { display:block; font-size:10px; color: var(--text-dim); margin-top:1px; }

  .quick-btn.go-btn   { border-color: var(--green); }
  .quick-btn.stop-btn { border-color: var(--red); color: var(--red); }
  .quick-btn.stop-btn .qb-cmd { color: var(--red); }

  /* stats */
  .stat-row { display:flex; justify-content:space-between; margin-bottom:4px; font-size:11px; }
  .stat-row .sk { color: var(--text-dim); }
  .stat-row .sv { color: var(--green); }

  /* custom cmd */
  #custom-form { display:flex; flex-direction:column; gap:5px; }
  #custom-input {
    background: var(--bg);
    border: 1px solid var(--border);
    color: var(--green);
    font-family: 'Share Tech Mono', monospace;
    font-size: 12px;
    padding: 5px 8px;
    outline: none;
    border-radius: 2px;
    width: 100%;
  }
  #custom-input:focus { border-color: var(--green); }
  #custom-send { width: 100%; }

  /* scan / refresh */
  .icon-btn {
    background: transparent;
    border: none;
    color: var(--text-dim);
    cursor: pointer;
    font-size: 13px;
    padding: 2px 5px;
  }
  .icon-btn:hover { color: var(--green); }

  /* scanline overlay */
  body::after {
    content: '';
    position: fixed; inset: 0;
    background: repeating-linear-gradient(
      0deg,
      transparent,
      transparent 2px,
      rgba(0,0,0,0.04) 2px,
      rgba(0,0,0,0.04) 4px
    );
    pointer-events: none;
    z-index: 9999;
  }
</style>
</head>
<body>

<header>
  <div class="status-dot" id="status-dot"></div>
  <h1>STM32<span class="blink">_</span>TERMINAL</h1>
  <div class="status-label" id="status-label">DISCONNECTED</div>
  <div class="spacer"></div>
  <div class="status-label" style="font-size:10px; color:var(--text-dim)">STM32F407 · PCA9685 · 4-LEG WALKER</div>
</header>

<div id="conn-bar">
  <label>PORT</label>
  <select id="port-select">
    <option value="">— select —</option>
  </select>
  <button class="icon-btn" onclick="refreshPorts()" title="Refresh ports">⟳</button>
  <label>BAUD</label>
  <select id="baud-select">
    <option>9600</option>
    <option>19200</option>
    <option>38400</option>
    <option selected>115200</option>
    <option>230400</option>
    <option>460800</option>
    <option>921600</option>
  </select>
  <button id="conn-btn" onclick="toggleConnect()">CONNECT</button>
  <button class="danger" onclick="clearLog()">CLEAR</button>
</div>

<main>
  <div id="terminal-wrap">
    <div id="terminal-header">
      <span>UART MONITOR</span>
      <span id="line-count" style="color:var(--text-dim)">0 lines</span>
    </div>
    <div id="log"></div>
    <div id="input-bar">
      <span id="prompt">&gt;</span>
      <input id="cmd-input" type="text" placeholder="type command + enter..."
             autocomplete="off" autocorrect="off" spellcheck="false"
             onkeydown="onInputKey(event)">
      <button onclick="sendFromInput()">SEND</button>
    </div>
  </div>

  <div id="sidebar">
    <div class="sidebar-section">
      <h3>QUICK COMMANDS</h3>
      <button class="quick-btn go-btn" onclick="quickSend('GO')">
        <span class="qb-cmd">GO</span>
        <span class="qb-desc">Start walking</span>
      </button>
      <button class="quick-btn stop-btn danger" onclick="quickSend('STOP')">
        <span class="qb-cmd">STOP</span>
        <span class="qb-desc">Disable servos</span>
      </button>
      <button class="quick-btn" onclick="quickSend('FWD')">
        <span class="qb-cmd">FWD</span>
        <span class="qb-desc">Forward direction</span>
      </button>
      <button class="quick-btn" onclick="quickSend('BACK')">
        <span class="qb-cmd">BACK</span>
        <span class="qb-desc">Backward direction</span>
      </button>
      <button class="quick-btn amber" onclick="quickSend('HELP')">
        <span class="qb-cmd">HELP</span>
        <span class="qb-desc">List commands</span>
      </button>
    </div>

    <div class="sidebar-section">
      <h3>CUSTOM COMMAND</h3>
      <div id="custom-form">
        <input id="custom-input" type="text" placeholder="e.g. STATUS"
               onkeydown="if(event.key==='Enter') sendCustom()">
        <button id="custom-send" onclick="sendCustom()">SEND</button>
      </div>
    </div>

    <div class="sidebar-section">
      <h3>SESSION STATS</h3>
      <div class="stat-row"><span class="sk">TX</span><span class="sv" id="stat-tx">0</span></div>
      <div class="stat-row"><span class="sk">RX</span><span class="sv" id="stat-rx">0</span></div>
      <div class="stat-row"><span class="sk">UPTIME</span><span class="sv" id="stat-uptime">—</span></div>
    </div>

    <div class="sidebar-section">
      <h3>CHANNEL MAP</h3>
      <div style="font-size:10px; color:var(--text-dim); line-height:1.9;">
        <div>FR: CH0 hip / CH2 knee</div>
        <div>FL: CH3 hip / CH5 knee</div>
        <div>BR: CH6 hip / CH7 knee</div>
        <div>BL: CH8 hip / CH9 knee</div>
      </div>
    </div>
  </div>
</main>

<script>
let connected = false;
let pollInterval = null;
let lastId = 0;
let txCount = 0, rxCount = 0;
let connectTime = null;
let cmdHistory = [];
let histIdx = -1;

// ─── Port management ───────────────────────────────────────────────────────
async function refreshPorts() {
  try {
    const r = await fetch('/api/ports');
    const d = await r.json();
    const sel = document.getElementById('port-select');
    const cur = sel.value;
    sel.innerHTML = '<option value="">— select —</option>';
    d.ports.forEach(p => {
      const o = document.createElement('option');
      o.value = o.textContent = p;
      if (p === cur) o.selected = true;
      sel.appendChild(o);
    });
  } catch(e) { sysLog('Port refresh failed: ' + e); }
}

async function toggleConnect() {
  if (!connected) {
    const port = document.getElementById('port-select').value;
    const baud = document.getElementById('baud-select').value;
    if (!port) { sysLog('Select a port first.'); return; }
    try {
      const r = await fetch('/api/connect', {
        method: 'POST',
        headers: {'Content-Type':'application/json'},
        body: JSON.stringify({port, baud: parseInt(baud)})
      });
      const d = await r.json();
      if (d.ok) {
        setConnected(true);
        startPoll();
        sysLog(`Connected: ${port} @ ${baud} baud`);
      } else {
        sysLog('Connect failed: ' + d.error);
      }
    } catch(e) { sysLog('Connect error: ' + e); }
  } else {
    await fetch('/api/disconnect', {method:'POST'});
    setConnected(false);
    stopPoll();
    sysLog('Disconnected.');
  }
}

function setConnected(v) {
  connected = v;
  document.getElementById('status-dot').className = 'status-dot' + (v ? ' connected' : '');
  document.getElementById('status-label').innerHTML = v
    ? 'CONNECTED <span>' + document.getElementById('port-select').value + '</span>'
    : 'DISCONNECTED';
  document.getElementById('conn-btn').textContent = v ? 'DISCONNECT' : 'CONNECT';
  if (v) { connectTime = Date.now(); updateUptime(); setInterval(updateUptime, 1000); }
  else connectTime = null;
}

function updateUptime() {
  if (!connectTime) return;
  const s = Math.floor((Date.now() - connectTime) / 1000);
  const h = Math.floor(s/3600), m = Math.floor((s%3600)/60), sec = s%60;
  document.getElementById('stat-uptime').textContent =
    (h?h+'h ':'') + (m?m+'m ':'') + sec+'s';
}

// ─── Sending ───────────────────────────────────────────────────────────────
async function sendCmd(cmd) {
  if (!connected) { sysLog('Not connected.'); return; }
  try {
    const r = await fetch('/api/send', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify({cmd})
    });
    const d = await r.json();
    if (d.ok) {
      txCount++;
      document.getElementById('stat-tx').textContent = txCount;
      cmdHistory.unshift(cmd);
      if (cmdHistory.length > 50) cmdHistory.pop();
      histIdx = -1;
    } else {
      sysLog('Send failed: ' + d.error);
    }
  } catch(e) { sysLog('Send error: ' + e); }
}

function quickSend(cmd) { sendCmd(cmd); }

function sendFromInput() {
  const inp = document.getElementById('cmd-input');
  const v = inp.value.trim();
  if (v) { sendCmd(v); inp.value = ''; }
}

function sendCustom() {
  const inp = document.getElementById('custom-input');
  const v = inp.value.trim();
  if (v) { sendCmd(v); inp.value = ''; }
}

function onInputKey(e) {
  if (e.key === 'Enter') { sendFromInput(); return; }
  if (e.key === 'ArrowUp') {
    histIdx = Math.min(histIdx + 1, cmdHistory.length - 1);
    document.getElementById('cmd-input').value = cmdHistory[histIdx] || '';
    e.preventDefault();
  }
  if (e.key === 'ArrowDown') {
    histIdx = Math.max(histIdx - 1, -1);
    document.getElementById('cmd-input').value = histIdx >= 0 ? cmdHistory[histIdx] : '';
    e.preventDefault();
  }
}

// ─── Polling ───────────────────────────────────────────────────────────────
function startPoll() { pollInterval = setInterval(poll, 100); }
function stopPoll()  { clearInterval(pollInterval); }

async function poll() {
  try {
    const r = await fetch('/api/log?since=' + lastId);
    const d = await r.json();
    if (d.lines && d.lines.length) {
      d.lines.forEach(renderLine);
      lastId = d.lines[d.lines.length-1].id;
    }
  } catch(e) {}
}

// ─── Rendering ─────────────────────────────────────────────────────────────
function renderLine(entry) {
  const log = document.getElementById('log');
  const div = document.createElement('div');
  div.className = 'log-line';

  const ts   = document.createElement('span');
  ts.className = 'log-ts';
  ts.textContent = entry.ts;

  const dir  = document.createElement('span');
  dir.className = 'log-dir dir-' + entry.dir.toLowerCase();
  dir.textContent = entry.dir;

  const txt  = document.createElement('span');
  txt.className = 'log-text text-' + entry.dir.toLowerCase();
  txt.innerHTML = highlight(escHtml(entry.text));

  if (entry.dir === 'RX') { rxCount++; document.getElementById('stat-rx').textContent = rxCount; }

  div.appendChild(ts); div.appendChild(dir); div.appendChild(txt);
  log.appendChild(div);

  // auto-scroll
  log.scrollTop = log.scrollHeight;
  document.getElementById('line-count').textContent = log.children.length + ' lines';
}

function escHtml(s) {
  return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}

function highlight(s) {
  // [OK] green, [FAIL] red, READY/CONFIRMED/STARTING cyan
  s = s.replace(/\[OK\]/g, '<span class="hl-ok">[OK]</span>');
  s = s.replace(/\[FAIL\]/g, '<span class="hl-fail">[FAIL]</span>');
  s = s.replace(/(READY|CONFIRMED|STARTING|CONNECTED|ALL SYSTEMS)/g,
      '<span class="hl-ready">$1</span>');
  s = s.replace(/(ERROR|FAIL|NOT RESPONDING)/g,
      '<span class="hl-fail">$1</span>');
  return s;
}

function sysLog(msg) {
  renderLine({id: --lastId, ts: new Date().toLocaleTimeString('en',{hour12:false}), dir:'SYS', text: msg});
}

function clearLog() {
  document.getElementById('log').innerHTML = '';
  document.getElementById('line-count').textContent = '0 lines';
  txCount = rxCount = 0;
  document.getElementById('stat-tx').textContent = '0';
  document.getElementById('stat-rx').textContent = '0';
}

// ─── Init ──────────────────────────────────────────────────────────────────
refreshPorts();
document.getElementById('cmd-input').focus();
sysLog('STM32 Terminal ready. Select a port and click CONNECT.');
</script>
</body>
</html>
"""

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args): pass  # silence HTTP logs

    def _json(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            body = HTML.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        elif self.path == "/api/ports":
            self._json(200, {"ports": list_ports()})

        elif self.path.startswith("/api/log"):
            since = 0
            if "since=" in self.path:
                try: since = int(self.path.split("since=")[1])
                except: pass
            with log_lock:
                lines = [e for e in log_buf if e["id"] > since]
            self._json(200, {"lines": lines})

        else:
            self._json(404, {"error": "not found"})

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length)
        try: body = json.loads(raw) if raw else {}
        except: body = {}

        if self.path == "/api/connect":
            port = body.get("port", "")
            baud = int(body.get("baud", DEFAULT_BAUD))
            try:
                open_serial(port, baud)
                _log("SYS", f"Serial opened: {port} @ {baud}")
                self._json(200, {"ok": True})
            except Exception as e:
                self._json(200, {"ok": False, "error": str(e)})

        elif self.path == "/api/disconnect":
            with ser_lock:
                global ser
                if ser and ser.is_open:
                    ser.close()
            _log("SYS", "Serial closed.")
            self._json(200, {"ok": True})

        elif self.path == "/api/send":
            cmd = body.get("cmd", "").strip()
            if not cmd:
                self._json(200, {"ok": False, "error": "empty command"})
                return
            ok = send_command(cmd)
            if ok:
                self._json(200, {"ok": True})
            else:
                self._json(200, {"ok": False, "error": "port not open"})

        else:
            self._json(404, {"error": "not found"})


# ─── Entry point ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="STM32 Web Terminal")
    parser.add_argument("port", nargs="?", default=None, help="Serial port (e.g. /dev/ttyUSB0 or COM3)")
    parser.add_argument("baud", nargs="?", type=int, default=DEFAULT_BAUD)
    args = parser.parse_args()

    # Auto-connect if port given on CLI
    if args.port:
        try:
            open_serial(args.port, args.baud)
            _log("SYS", f"Auto-connected: {args.port} @ {args.baud}")
            print(f"[+] Auto-connected to {args.port} @ {args.baud}")
        except Exception as e:
            print(f"[!] Could not open {args.port}: {e}")

    # Start reader thread
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()

    # Start web server
    server = HTTPServer(("127.0.0.1", WEB_PORT), Handler)
    url = f"http://localhost:{WEB_PORT}"
    print(f"[+] STM32 Terminal running at {url}")
    print(f"[+] Open your browser → {url}")
    print(f"[+] Press Ctrl+C to quit\n")

    try:
        import webbrowser
        webbrowser.open(url)
    except: pass

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[+] Shutting down.")
