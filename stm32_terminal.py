#!/usr/bin/env python3
"""
STM32 Control Center — Web-based UART interface for STM32F407 robot

Run:
    python main_improved.py
or:
    python main_improved.py COM4 9600

Then open:
    http://localhost:8765
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
DEFAULT_PORT   = "COM4"
DEFAULT_BAUD   = 9600   # Match STM32 main.c UART baud
WEB_PORT       = 8765
LOG_MAXLEN     = 800
READ_TIMEOUT   = 0.05

# ─── Global State ─────────────────────────────────────────────────────────────
ser           = None
ser_lock      = threading.Lock()
log_buf       = deque(maxlen=LOG_MAXLEN)
log_lock      = threading.Lock()
last_fetch_id = 0

QUICK_CMDS = [
    ("GO",   "GO",   "Start walking"),
    ("STOP", "STOP", "Stop walking"),
    ("FWD",  "FWD",  "Move forward"),
    ("BACK", "BACK", "Move backward"),
    ("HELP", "HELP", "List commands"),
]

# ─── Serial helpers ───────────────────────────────────────────────────────────
def list_ports():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    ports.sort(key=lambda p: (p != DEFAULT_PORT, p))
    return ports

def port_exists(port):
    return port in list_ports()

def open_serial(port, baud):
    global ser
    with ser_lock:
        if ser and ser.is_open:
            ser.close()
        ser = serial.Serial(port, baud, timeout=READ_TIMEOUT)
        # Give USB-UART a moment to settle
        time.sleep(0.2)

def close_serial():
    global ser
    with ser_lock:
        if ser and ser.is_open:
            ser.close()

def is_connected():
    with ser_lock:
        return bool(ser and ser.is_open)

def send_command(cmd):
    with ser_lock:
        if ser and ser.is_open:
            line = cmd.strip() + "\n"
            ser.write(line.encode("utf-8", errors="replace"))
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
            "dir":  direction,
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
                time.sleep(0.15)
        except Exception as e:
            _log("SYS", f"Read error: {e}")
            time.sleep(0.8)

# ─── UI HTML ──────────────────────────────────────────────────────────────────
HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>STM32 Control Center</title>
<style>
  :root{
    --bg:#0b1020;
    --bg-2:#121933;
    --panel:#151d3a;
    --panel-2:#1b2548;
    --card:#1a2344;
    --border:#2d3a6c;
    --text:#eef3ff;
    --muted:#9fb0da;
    --accent:#6ea8fe;
    --accent-2:#7cf5c9;
    --success:#52d273;
    --warn:#ffca5c;
    --danger:#ff6b7a;
    --shadow:0 12px 30px rgba(0,0,0,.28);
    --radius:18px;
  }

  *{box-sizing:border-box}
  html,body{height:100%}
  body{
    margin:0;
    font-family:Inter,Segoe UI,Arial,sans-serif;
    color:var(--text);
    background:
      radial-gradient(circle at top left, #223468 0%, transparent 28%),
      radial-gradient(circle at top right, #173253 0%, transparent 22%),
      linear-gradient(180deg, #0a1020 0%, #0f1730 100%);
    overflow:hidden;
  }

  .app{
    height:100%;
    display:grid;
    grid-template-rows:auto auto 1fr;
    gap:14px;
    padding:16px;
  }

  .topbar, .toolbar, .panel, .sidebar-card{
    background:rgba(21,29,58,.92);
    backdrop-filter:blur(10px);
    border:1px solid rgba(114,143,216,.18);
    box-shadow:var(--shadow);
  }

  .topbar{
    border-radius:20px;
    display:flex;
    align-items:center;
    gap:14px;
    padding:16px 18px;
  }

  .brand{
    display:flex;
    align-items:center;
    gap:14px;
    min-width:0;
  }

  .logo{
    width:44px;height:44px;border-radius:14px;
    display:grid;place-items:center;
    background:linear-gradient(135deg, var(--accent), var(--accent-2));
    color:#08101f;
    font-weight:800;
    font-size:18px;
  }

  .title-wrap h1{
    margin:0;
    font-size:1.25rem;
    font-weight:800;
    letter-spacing:.02em;
  }
  .title-wrap p{
    margin:4px 0 0 0;
    color:var(--muted);
    font-size:.92rem;
  }

  .spacer{flex:1}

  .status{
    display:flex;
    align-items:center;
    gap:10px;
    padding:10px 14px;
    border-radius:999px;
    background:rgba(255,255,255,.03);
    border:1px solid rgba(255,255,255,.06);
    white-space:nowrap;
  }

  .dot{
    width:10px;height:10px;border-radius:50%;
    background:var(--danger);
    box-shadow:0 0 0 4px rgba(255,107,122,.15);
  }
  .dot.connected{
    background:var(--success);
    box-shadow:0 0 0 4px rgba(82,210,115,.15);
  }

  .toolbar{
    border-radius:20px;
    padding:14px 16px;
    display:flex;
    flex-wrap:wrap;
    gap:12px;
    align-items:end;
  }

  .field{
    display:flex;
    flex-direction:column;
    gap:6px;
    min-width:180px;
  }
  .field.small{min-width:120px}
  .field label{
    color:var(--muted);
    font-size:.8rem;
    font-weight:700;
    letter-spacing:.03em;
  }

  select,input[type=text]{
    width:100%;
    background:var(--bg-2);
    color:var(--text);
    border:1px solid var(--border);
    border-radius:14px;
    padding:12px 14px;
    font-size:.95rem;
    outline:none;
  }
  select:focus,input[type=text]:focus{
    border-color:var(--accent);
    box-shadow:0 0 0 3px rgba(110,168,254,.18);
  }

  button{
    border:none;
    border-radius:14px;
    padding:12px 16px;
    font-size:.95rem;
    font-weight:700;
    cursor:pointer;
    transition:.18s transform,.18s opacity,.18s box-shadow;
  }
  button:hover{transform:translateY(-1px)}
  button:active{transform:translateY(0)}
  button.primary{
    background:linear-gradient(135deg, var(--accent), #8e96ff);
    color:white;
  }
  button.secondary{
    background:#25325f;
    color:var(--text);
  }
  button.success{
    background:linear-gradient(135deg, #3dbb67, #69dd87);
    color:white;
  }
  button.warning{
    background:linear-gradient(135deg, #f3b23e, #ffd36f);
    color:#312200;
  }
  button.danger{
    background:linear-gradient(135deg, #ef5e71, #ff8594);
    color:white;
  }

  .layout{
    min-height:0;
    display:grid;
    grid-template-columns:1.55fr .9fr;
    gap:14px;
  }

  .panel{
    border-radius:22px;
    display:grid;
    grid-template-rows:auto 1fr auto;
    min-height:0;
    overflow:hidden;
  }

  .panel-head{
    display:flex;
    justify-content:space-between;
    align-items:center;
    padding:16px 18px;
    border-bottom:1px solid rgba(255,255,255,.08);
  }
  .panel-head h2{
    margin:0;
    font-size:1rem;
  }
  .panel-head .muted{
    color:var(--muted);
    font-size:.9rem;
  }

  #log{
    min-height:0;
    overflow:auto;
    padding:14px 16px;
    background:
      linear-gradient(180deg, rgba(255,255,255,.015), rgba(255,255,255,.01)),
      #0f1630;
  }

  .log-line{
    display:grid;
    grid-template-columns:76px 56px 1fr;
    gap:10px;
    align-items:start;
    padding:10px 12px;
    margin-bottom:8px;
    border-radius:14px;
    background:rgba(255,255,255,.025);
    border:1px solid rgba(255,255,255,.04);
  }

  .log-ts{color:var(--muted); font-variant-numeric:tabular-nums;}
  .log-dir{
    font-weight:800;
    text-align:center;
    border-radius:999px;
    padding:5px 8px;
    font-size:.82rem;
  }
  .dir-rx{background:rgba(82,210,115,.14); color:#87f2a3;}
  .dir-tx{background:rgba(255,202,92,.14); color:#ffd97d;}
  .dir-sys{background:rgba(110,168,254,.14); color:#9ec3ff;}
  .log-text{word-break:break-word; line-height:1.45;}

  .hl-ok{color:#87f2a3;font-weight:700;}
  .hl-fail{color:#ff96a1;font-weight:700;}
  .hl-ready{color:#9ec3ff;font-weight:700;}

  .inputbar{
    display:flex;
    gap:10px;
    align-items:center;
    padding:14px 16px 16px 16px;
    border-top:1px solid rgba(255,255,255,.08);
    background:rgba(255,255,255,.02);
  }
  .inputbar input{
    flex:1;
  }

  .sidebar{
    min-height:0;
    display:grid;
    gap:14px;
    grid-template-rows:auto auto auto 1fr;
  }

  .sidebar-card{
    border-radius:22px;
    padding:16px;
  }
  .sidebar-card h3{
    margin:0 0 12px 0;
    font-size:.96rem;
  }

  .quick-grid{
    display:grid;
    grid-template-columns:1fr 1fr;
    gap:10px;
  }
  .quick-grid button{
    text-align:left;
    min-height:70px;
  }
  .quick-grid small{
    display:block;
    margin-top:6px;
    opacity:.88;
    font-weight:600;
  }

  .stats{
    display:grid;
    grid-template-columns:repeat(3, 1fr);
    gap:10px;
  }
  .stat{
    border-radius:16px;
    background:rgba(255,255,255,.03);
    border:1px solid rgba(255,255,255,.05);
    padding:12px;
  }
  .stat .k{
    color:var(--muted);
    font-size:.8rem;
    margin-bottom:4px;
  }
  .stat .v{
    font-size:1.15rem;
    font-weight:800;
  }

  .map{
    color:var(--muted);
    line-height:1.7;
    font-size:.92rem;
  }

  .hint{
    color:var(--muted);
    font-size:.88rem;
    line-height:1.55;
  }

  @media (max-width: 980px){
    body{overflow:auto}
    .layout{grid-template-columns:1fr}
    .app{height:auto; min-height:100%}
  }
</style>
</head>
<body>
<div class="app">
  <div class="topbar">
    <div class="brand">
      <div class="logo">S</div>
      <div class="title-wrap">
        <h1>STM32 Control Center</h1>
        <p>Cleaner UART terminal for your STM32F407 robot</p>
      </div>
    </div>
    <div class="spacer"></div>
    <div class="status">
      <div class="dot" id="status-dot"></div>
      <div id="status-text">Disconnected</div>
    </div>
  </div>

  <div class="toolbar">
    <div class="field">
      <label>Serial Port</label>
      <select id="port-select"></select>
    </div>

    <div class="field small">
      <label>Baud Rate</label>
      <select id="baud-select">
        <option selected>9600</option>
        <option>19200</option>
        <option>38400</option>
        <option>57600</option>
        <option>115200</option>
      </select>
    </div>

    <button class="secondary" onclick="refreshPorts()">Refresh Ports</button>
    <button class="primary" id="conn-btn" onclick="toggleConnect()">Connect</button>
    <button class="secondary" onclick="sendHandshake()">Send HELLO</button>
    <button class="danger" onclick="clearLog()">Clear Log</button>
  </div>

  <div class="layout">
    <div class="panel">
      <div class="panel-head">
        <h2>UART Monitor</h2>
        <div class="muted" id="line-count">0 lines</div>
      </div>

      <div id="log"></div>

      <div class="inputbar">
        <input id="cmd-input" type="text" placeholder="Type a command and press Enter" autocomplete="off"
               onkeydown="onInputKey(event)">
        <button class="primary" onclick="sendFromInput()">Send</button>
      </div>
    </div>

    <div class="sidebar">
      <div class="sidebar-card">
        <h3>Quick Commands</h3>
        <div class="quick-grid">
          <button class="success" onclick="quickSend('GO')">GO<small>Start walking</small></button>
          <button class="danger" onclick="quickSend('STOP')">STOP<small>Stop walking</small></button>
          <button class="primary" onclick="quickSend('FWD')">FWD<small>Move forward</small></button>
          <button class="warning" onclick="quickSend('BACK')">BACK<small>Move backward</small></button>
        </div>
      </div>

      <div class="sidebar-card">
        <h3>Custom Command</h3>
        <div style="display:flex; gap:10px;">
          <input id="custom-input" type="text" placeholder="Example: HELP"
                 onkeydown="if(event.key==='Enter') sendCustom()">
          <button class="primary" onclick="sendCustom()">Send</button>
        </div>
      </div>

      <div class="sidebar-card">
        <h3>Session Stats</h3>
        <div class="stats">
          <div class="stat"><div class="k">TX</div><div class="v" id="stat-tx">0</div></div>
          <div class="stat"><div class="k">RX</div><div class="v" id="stat-rx">0</div></div>
          <div class="stat"><div class="k">Uptime</div><div class="v" id="stat-uptime">—</div></div>
        </div>
      </div>

      <div class="sidebar-card">
        <h3>Servo Channel Map</h3>
        <div class="map">
          FR: CH0 hip / CH2 knee<br>
          FL: CH3 hip / CH5 knee<br>
          BR: CH6 hip / CH7 knee<br>
          BL: CH8 hip / CH9 knee
        </div>
        <div class="hint" style="margin-top:12px;">
          Tip: COM4 is auto-selected when available so you do not need to choose it every time.
        </div>
      </div>
    </div>
  </div>
</div>

<script>
let connected = false;
let pollInterval = null;
let uptimeInterval = null;
let lastId = 0;
let txCount = 0;
let rxCount = 0;
let connectTime = null;
let cmdHistory = [];
let histIdx = -1;

function savePrefs() {
  localStorage.setItem('stm32_port', document.getElementById('port-select').value);
  localStorage.setItem('stm32_baud', document.getElementById('baud-select').value);
}

function loadPrefs() {
  const baud = localStorage.getItem('stm32_baud');
  if (baud) document.getElementById('baud-select').value = baud;
}

async function refreshPorts() {
  try {
    const res = await fetch('/api/ports');
    const data = await res.json();
    const sel = document.getElementById('port-select');
    const saved = localStorage.getItem('stm32_port');
    const current = sel.value;

    sel.innerHTML = '';
    if (!data.ports.length) {
      const opt = document.createElement('option');
      opt.value = '';
      opt.textContent = 'No serial ports found';
      sel.appendChild(opt);
      return;
    }

    data.ports.forEach((p) => {
      const o = document.createElement('option');
      o.value = p;
      o.textContent = p + (p === 'COM4' ? ' (default)' : '');
      sel.appendChild(o);
    });

    if (data.ports.includes(saved)) sel.value = saved;
    else if (data.ports.includes(current)) sel.value = current;
    else if (data.ports.includes('COM4')) sel.value = 'COM4';
    else sel.value = data.ports[0];

    savePrefs();
  } catch (e) {
    sysLog('Port refresh failed: ' + e);
  }
}

async function toggleConnect() {
  if (!connected) {
    const port = document.getElementById('port-select').value;
    const baud = parseInt(document.getElementById('baud-select').value);
    if (!port) {
      sysLog('No serial port selected.');
      return;
    }
    try {
      const res = await fetch('/api/connect', {
        method: 'POST',
        headers: {'Content-Type':'application/json'},
        body: JSON.stringify({port, baud})
      });
      const data = await res.json();
      if (data.ok) {
        setConnected(true, port);
        savePrefs();
        startPoll();
        sysLog(`Connected to ${port} @ ${baud} baud`);
      } else {
        sysLog('Connect failed: ' + data.error);
      }
    } catch (e) {
      sysLog('Connect error: ' + e);
    }
  } else {
    try {
      await fetch('/api/disconnect', {method:'POST'});
    } catch (_) {}
    setConnected(false);
    stopPoll();
    sysLog('Disconnected.');
  }
}

function setConnected(v, port='') {
  connected = v;
  const dot = document.getElementById('status-dot');
  const label = document.getElementById('status-text');
  const btn = document.getElementById('conn-btn');

  dot.className = 'dot' + (v ? ' connected' : '');
  label.textContent = v ? `Connected: ${port}` : 'Disconnected';
  btn.textContent = v ? 'Disconnect' : 'Connect';

  if (uptimeInterval) clearInterval(uptimeInterval);
  if (v) {
    connectTime = Date.now();
    updateUptime();
    uptimeInterval = setInterval(updateUptime, 1000);
  } else {
    connectTime = null;
    document.getElementById('stat-uptime').textContent = '—';
  }
}

function updateUptime() {
  if (!connectTime) return;
  const s = Math.floor((Date.now() - connectTime) / 1000);
  const m = Math.floor(s / 60);
  const sec = s % 60;
  const h = Math.floor(m / 60);
  const mm = m % 60;
  document.getElementById('stat-uptime').textContent =
    h ? `${h}h ${mm}m` : `${mm}m ${sec}s`;
}

async function sendCmd(cmd) {
  if (!connected) {
    sysLog('Not connected.');
    return;
  }
  try {
    const res = await fetch('/api/send', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify({cmd})
    });
    const data = await res.json();
    if (data.ok) {
      txCount++;
      document.getElementById('stat-tx').textContent = txCount;
      cmdHistory.unshift(cmd);
      if (cmdHistory.length > 50) cmdHistory.pop();
      histIdx = -1;
    } else {
      sysLog('Send failed: ' + data.error);
    }
  } catch (e) {
    sysLog('Send error: ' + e);
  }
}

function sendHandshake() {
  sendCmd('HELLO');
}

function quickSend(cmd) {
  sendCmd(cmd);
}

function sendFromInput() {
  const inp = document.getElementById('cmd-input');
  const value = inp.value.trim();
  if (!value) return;
  sendCmd(value);
  inp.value = '';
}

function sendCustom() {
  const inp = document.getElementById('custom-input');
  const value = inp.value.trim();
  if (!value) return;
  sendCmd(value);
  inp.value = '';
}

function onInputKey(e) {
  if (e.key === 'Enter') {
    sendFromInput();
    return;
  }
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

function startPoll() {
  stopPoll();
  pollInterval = setInterval(poll, 120);
}

function stopPoll() {
  if (pollInterval) clearInterval(pollInterval);
  pollInterval = null;
}

async function poll() {
  try {
    const res = await fetch('/api/log?since=' + lastId);
    const data = await res.json();
    if (data.lines && data.lines.length) {
      data.lines.forEach(renderLine);
      lastId = data.lines[data.lines.length - 1].id;
    }
  } catch (_) {}
}

function renderLine(entry) {
  const log = document.getElementById('log');
  const row = document.createElement('div');
  row.className = 'log-line';

  const ts = document.createElement('div');
  ts.className = 'log-ts';
  ts.textContent = entry.ts;

  const dir = document.createElement('div');
  dir.className = 'log-dir dir-' + entry.dir.toLowerCase();
  dir.textContent = entry.dir;

  const txt = document.createElement('div');
  txt.className = 'log-text';
  txt.innerHTML = highlight(escapeHtml(entry.text));

  if (entry.dir === 'RX') {
    rxCount++;
    document.getElementById('stat-rx').textContent = rxCount;
  }

  row.appendChild(ts);
  row.appendChild(dir);
  row.appendChild(txt);
  log.appendChild(row);

  log.scrollTop = log.scrollHeight;
  document.getElementById('line-count').textContent = `${log.children.length} lines`;
}

function escapeHtml(s) {
  return s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
}

function highlight(s) {
  s = s.replace(/\[OK\]/g, '<span class="hl-ok">[OK]</span>');
  s = s.replace(/\[FAIL\]/g, '<span class="hl-fail">[FAIL]</span>');
  s = s.replace(/\b(READY|CONFIRMED|STARTING|CONNECTED|ALL SYSTEMS)\b/g, '<span class="hl-ready">$1</span>');
  s = s.replace(/\b(ERROR|FAIL|NOT RESPONDING)\b/g, '<span class="hl-fail">$1</span>');
  return s;
}

function sysLog(msg) {
  const now = new Date();
  const ts = now.toLocaleTimeString('en-GB', {hour12:false});
  renderLine({id: --lastId, ts, dir:'SYS', text: msg});
}

function clearLog() {
  document.getElementById('log').innerHTML = '';
  document.getElementById('line-count').textContent = '0 lines';
  txCount = 0;
  rxCount = 0;
  document.getElementById('stat-tx').textContent = '0';
  document.getElementById('stat-rx').textContent = '0';
}

document.getElementById('baud-select').addEventListener('change', savePrefs);
document.getElementById('port-select').addEventListener('change', savePrefs);

loadPrefs();
refreshPorts();
setInterval(refreshPorts, 3000);
document.getElementById('cmd-input').focus();
sysLog('Ready. COM4 will be selected automatically when available.');
</script>
</body>
</html>
"""

# ─── HTTP handler ─────────────────────────────────────────────────────────────
class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def _json(self, code, obj):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/api/ports":
            ports = list_ports()
            self._json(200, {
                "ports": ports,
                "default_port": DEFAULT_PORT,
                "default_available": DEFAULT_PORT in ports,
                "connected": is_connected(),
            })
            return

        if self.path.startswith("/api/log"):
            since = 0
            if "since=" in self.path:
                try:
                    since = int(self.path.split("since=")[1])
                except Exception:
                    since = 0
            with log_lock:
                lines = [e for e in log_buf if e["id"] > since]
            self._json(200, {"lines": lines})
            return

        self._json(404, {"error": "not found"})

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length)
        try:
            body = json.loads(raw) if raw else {}
        except Exception:
            body = {}

        if self.path == "/api/connect":
            port = body.get("port", DEFAULT_PORT)
            baud = int(body.get("baud", DEFAULT_BAUD))
            try:
                open_serial(port, baud)
                _log("SYS", f"Serial opened: {port} @ {baud}")
                self._json(200, {"ok": True, "port": port, "baud": baud})
            except Exception as e:
                self._json(200, {"ok": False, "error": str(e)})
            return

        if self.path == "/api/disconnect":
            close_serial()
            _log("SYS", "Serial closed.")
            self._json(200, {"ok": True})
            return

        if self.path == "/api/send":
            cmd = body.get("cmd", "").strip()
            if not cmd:
                self._json(200, {"ok": False, "error": "empty command"})
                return
            if send_command(cmd):
                self._json(200, {"ok": True})
            else:
                self._json(200, {"ok": False, "error": "port not open"})
            return

        self._json(404, {"error": "not found"})

# ─── Entry point ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="STM32 Web Terminal")
    parser.add_argument("port", nargs="?", default=DEFAULT_PORT, help="Serial port (default: COM4)")
    parser.add_argument("baud", nargs="?", type=int, default=DEFAULT_BAUD, help="Baud rate (default: 9600)")
    args = parser.parse_args()

    # Auto-connect only if the requested port exists
    if args.port and port_exists(args.port):
        try:
            open_serial(args.port, args.baud)
            _log("SYS", f"Auto-connected: {args.port} @ {args.baud}")
            print(f"[+] Auto-connected to {args.port} @ {args.baud}")
        except Exception as e:
            print(f"[!] Could not open {args.port}: {e}")
    else:
        if args.port:
            print(f"[!] {args.port} not found. Open the UI and choose an available port.")

    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()

    server = HTTPServer(("127.0.0.1", WEB_PORT), Handler)
    url = f"http://localhost:{WEB_PORT}"
    print(f"[+] STM32 Control Center running at {url}")
    print(f"[+] Default port: {DEFAULT_PORT}")
    print(f"[+] Default baud: {DEFAULT_BAUD}")
    print(f"[+] Press Ctrl+C to quit\n")

    try:
        import webbrowser
        webbrowser.open(url)
    except Exception:
        pass

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[+] Shutting down.")
        close_serial()
