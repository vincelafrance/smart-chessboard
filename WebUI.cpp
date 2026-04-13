#include "WebUI.h"
#include "Commands.h"

// =======================================================
// HTML (unchanged)
// =======================================================
const char PAGE_INDEX[] PROGMEM = R"HTML(
<!doctype html>
<html lang="fr">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width,initial-scale=1"/>
  <title>Smart Chessboard</title>
  <style>
    body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial;margin:0;background:#0b0f14;color:#e8eef6}
    .wrap{max-width:760px;margin:0 auto;padding:18px}
    .card{background:#111826;border:1px solid #1f2a3a;border-radius:18px;padding:16px;box-shadow:0 10px 30px rgba(0,0,0,.25); margin-bottom:14px}
    h1{font-size:18px;margin:0 0 10px 0}
    .big{font-size:54px;font-weight:800;letter-spacing:-1px;margin:8px 0}
    .row{display:flex;gap:12px;margin-top:12px;flex-wrap:wrap}
    .pill{flex:1;background:#0e1522;border:1px solid #1f2a3a;border-radius:14px;padding:12px;min-width:160px}
    .label{opacity:.7;font-size:12px}
    .val{font-size:20px;font-weight:700;margin-top:4px}
    .bar{height:14px;border-radius:999px;background:#0e1522;border:1px solid #1f2a3a;overflow:hidden;margin-top:12px}
    .fill{height:100%;width:0%;background:linear-gradient(90deg,#1dd1a1,#54a0ff);transition:width .12s ease}
    .status{margin-top:10px;font-size:12px;opacity:.75}
    .warn{color:#ff6b6b}
    .ok{color:#1dd1a1}
    .controls{display:flex;flex-direction:column;align-items:center;gap:10px;margin-top:6px}
    .controls .mid{display:flex;gap:12px}
    button{
      font-size:26px;width:72px;height:72px;border-radius:16px;
      border:1px solid #1f2a3a;background:#0e1522;color:#e8eef6;
      touch-action: none;
    }
    button:active{background:#1dd1a1;color:#000}
    .mini{font-size:14px;width:auto;height:auto;padding:10px 12px}
    .seg{display:flex;gap:10px;flex-wrap:wrap;margin-top:10px}
    .seg button{font-size:13px;padding:10px 12px;width:auto;height:auto;border-radius:14px}
    .seg button.active{background:#1dd1a1;color:#000;border-color:#1dd1a1}
    .pos{display:flex;gap:12px;flex-wrap:wrap;margin-top:8px;opacity:.85;font-size:13px}
    code{background:#0e1522;border:1px solid #1f2a3a;border-radius:10px;padding:2px 8px}
    #magnetBtn.active{background:#1dd1a1;color:#000;border-color:#1dd1a1}
    #calibBtn.active{background:#1dd1a1;color:#000;border-color:#1dd1a1}
    .dot{width:14px;height:14px;border-radius:999px;display:inline-block;background:#ff4d4d;box-shadow:0 0 0 3px rgba(255,77,77,.15);margin-right:8px;vertical-align:middle;}
    .dot.on{background:#1dd1a1;box-shadow:0 0 0 3px rgba(29,209,161,.15);}
    .inline{display:flex;align-items:center;gap:10px;margin-top:10px;flex-wrap:wrap;font-size:13px;opacity:.9;}
    .badge{background:#0e1522;border:1px solid #1f2a3a;border-radius:999px;padding:8px 10px;display:inline-flex;align-items:center;gap:8px;}

    .gridWrap{display:flex;gap:14px;flex-wrap:wrap;margin-top:10px;align-items:flex-start}
    .board{display:grid;grid-template-columns:repeat(8,38px);grid-template-rows:repeat(8,38px);gap:6px}
    .sq{width:38px;height:38px;border-radius:10px;border:1px solid #1f2a3a;background:#0e1522;color:#e8eef6;
        font-size:12px;display:flex;align-items:center;justify-content:center;user-select:none}

    .sq{position:relative}
    .sq .piece{font-size:22px;line-height:1}
    .sq.illegalFlash{
  box-shadow: inset 0 0 0 3px rgba(255,0,0,0.85);
  animation: illegalPulse 220ms ease-in-out 0s 2;
}
@keyframes illegalPulse{
  0%{ transform: scale(1); }
  50%{ transform: scale(0.98); }
  100%{ transform: scale(1); }
}

.sq .coord{position:absolute;bottom:4px;right:6px;font-size:9px;opacity:.55}

    .sq.light{background:#101b2a}
    .sq.selFrom{background:#54a0ff;color:#001018;border-color:#54a0ff}
    .sq.selTo{background:#1dd1a1;color:#001018;border-color:#1dd1a1}
    .legend{font-size:12px;opacity:.8;margin-top:6px}
  
.boardWrap{
  display:flex;
  flex-direction:column;
  gap:8px;
  align-items:flex-start;
  position:relative;
}

.boardLoading{
  display:none;
  position:absolute;
  inset:34px 0 0 0;
  align-items:center;
  justify-content:center;
  font-size:14px;
  font-weight:700;
  color:#e8eef6;
  background:rgba(11,15,20,0.65);
  border-radius:12px;
  z-index:25;
}

.boardWrap.calibrating .board{opacity:.25; pointer-events:none;}
.boardWrap.calibrating .boardLoading{display:flex;}


.turnOverlay{
  position:static;
  z-index:20;
  padding:6px 10px;
  border-radius:10px;
  font-weight:700;
  font-size:14px;
  letter-spacing:0.3px;
  background:rgba(0,0,0,0.55);
  backdrop-filter: blur(6px);
  border:1px solid rgba(255,255,255,0.12);
}

.turnOverlay.white{ color: rgba(255,255,255,0.95); }
.turnOverlay.black{ color: rgba(255,255,255,0.95); }


.piece.white{
  color: rgba(255,255,255,0.95);
  text-shadow: 0 1px 2px rgba(0,0,0,0.6);
}
.piece.black{
  color: rgba(0,0,0,0.9);
  text-shadow: 0 1px 2px rgba(255,255,255,0.25);
}

.sq.check{box-shadow:inset 0 0 0 3px rgba(255,80,80,0.95);}
</style>
</head>
<body>
  <div class="wrap">
    <div class="card">
      <h1>🔋 Batterie (temps réel)</h1>
      <div class="big"><span id="pct">--</span>%</div>
      <div class="bar"><div class="fill" id="fill"></div></div>

      <div class="row">
        <div class="pill"><div class="label">Voltage</div><div class="val"><span id="v">--</span> V</div></div>
        <div class="pill"><div class="label">Courant</div><div class="val"><span id="i">--</span> A</div></div>
      </div>

      <div class="inline">
        <div class="badge">
          <span class="dot" id="hallYDot"></span>
          <span>Hall Y:</span>
          <b id="hallYText">No magnet</b>
        </div>
        <div class="badge">
          <span class="dot" id="hallXDot"></span>
          <span>Hall X:</span>
          <b id="hallXText">No magnet</b>
        </div>
        <div class="badge">
          <span>Calib:</span>
          <b id="calibText">OFF</b>
        </div>
      </div>

      <div class="status" id="status">Connexion...</div>
    </div>

    <div class="card">
      <h1>♟️ Chariot (CoreXY)</h1>

      <div class="seg">
        <button class="mini" id="spSlow">🐢 Lent</button>
        <button class="mini active" id="spNorm">🚶 Normal</button>
        <button class="mini" id="spFast">🚀 Rapide</button>
      </div>

      <div class="controls">
        <button id="up">⬆️</button>
        <div class="mid">
          <button id="left">⬅️</button>
          <button id="stop">⏹</button>
          <button id="right">➡️</button>
        </div>
        <button id="down">⬇️</button>

        <div style="margin-top:6px; display:flex; gap:10px; flex-wrap:wrap;">
          <button class="mini" id="center">Recentrer</button>
          <button class="mini" id="calibBtn">🎯 Calibration</button>
          <button class="mini" id="diag4Btn">🧭 Test 1-2-3-4</button>
          <button class="mini" id="magnetBtn">🧲 Electroaimant: OFF</button>
          <button class="mini" id="tuneBtn">⚙️ Auto Tune</button>
          <button class="mini" id="tuneStopBtn" style="display:none;">🛑 Stop Tune</button>
          <button class="mini" id="testRunBtn">♟️ Test Run</button>
          <button class="mini" id="testRunStopBtn" style="display:none;">🛑 Stop Test</button>
        </div>

        <div id="testRunCard" style="display:none;margin-top:10px;padding:10px;background:#0e1522;border:1px solid #1f2a3a;border-radius:14px;">
          <div style="display:flex;align-items:center;gap:10px;flex-wrap:wrap;">
            <span style="font-weight:700;font-size:13px;">♟️ Test Run</span>
            <span id="testRunStepTxt" style="font-size:12px;opacity:.8;">—</span>
            <span id="testRunPctTxt" style="font-size:12px;font-weight:700;color:#54a0ff;">0%</span>
          </div>
          <div class="bar" style="margin-top:8px;">
            <div class="fill" id="testRunFill" style="background:linear-gradient(90deg,#f9ca24,#f0932b);"></div>
          </div>
        </div>

        <div id="tuneCard" style="display:none;margin-top:10px;padding:10px;background:#0e1522;border:1px solid #1f2a3a;border-radius:14px;">
          <div style="display:flex;align-items:center;gap:10px;flex-wrap:wrap;">
            <span style="font-weight:700;font-size:13px;">⚙️ Auto Tune</span>
            <span id="tunePhaseTxt" style="font-size:12px;opacity:.8;">—</span>
            <span id="tunePctTxt" style="font-size:12px;font-weight:700;color:#54a0ff;">0%</span>
          </div>
          <div class="bar" style="margin-top:8px;">
            <div class="fill" id="tuneFill" style="background:linear-gradient(90deg,#54a0ff,#1dd1a1);"></div>
          </div>
          <div id="tuneLog" style="margin-top:8px;font-size:11px;font-family:monospace;opacity:.85;max-height:100px;overflow-y:auto;white-space:pre-wrap;background:#060a10;padding:6px 8px;border-radius:8px;border:1px solid #1f2a3a;"></div>
        </div>

        <div id="tuneResultCard" style="margin-top:10px;padding:10px;background:#0e1522;border:1px solid #1f2a3a;border-radius:14px;">
          <div style="display:flex;align-items:center;gap:8px;margin-bottom:8px;flex-wrap:wrap;">
            <span style="font-weight:700;font-size:13px;">💾 Réglages enregistrés</span>
            <span id="tuneValidBadge" style="font-size:11px;padding:3px 10px;border-radius:999px;background:#1f2a3a;color:#888;border:1px solid #2a3a4a;">Non calibré</span>
          </div>
          <div style="display:flex;align-items:center;gap:8px;margin-bottom:8px;flex-wrap:wrap;">
            <span style="font-weight:700;font-size:12px;">🧪 Réglages en cours de test</span>
            <span id="tuneLiveBadge" style="font-size:11px;padding:3px 10px;border-radius:999px;background:#1f2a3a;color:#888;border:1px solid #2a3a4a;">Inactif</span>
          </div>
          <div style="display:flex;gap:8px;flex-wrap:wrap;margin-bottom:10px;">
            <div class="pill" style="min-width:90px;">
              <div class="label">Vit. axe</div>
              <div class="val" id="liveTuneSpdVal" style="font-size:16px;">—</div>
              <div class="label" style="margin-top:2px;">steps/s</div>
            </div>
            <div class="pill" style="min-width:90px;">
              <div class="label">Vit. diag</div>
              <div class="val" id="liveTuneSpdDiagVal" style="font-size:16px;">—</div>
              <div class="label" style="margin-top:2px;">steps/s</div>
            </div>
            <div class="pill" style="min-width:90px;">
              <div class="label">Courant</div>
              <div class="val" id="liveTuneCurrVal" style="font-size:16px;">—</div>
              <div class="label" style="margin-top:2px;">mA</div>
            </div>
          </div>
          <div style="display:flex;gap:8px;flex-wrap:wrap;">
            <div class="pill" style="min-width:90px;">
              <div class="label">Vit. lignes droites</div>
              <div class="val" id="tuneSpdVal" style="font-size:16px;">—</div>
              <div class="label" style="margin-top:2px;">steps/s</div>
            </div>
            <div class="pill" style="min-width:90px;">
              <div class="label">Vit. diagonales</div>
              <div class="val" id="tuneSpdDiagVal" style="font-size:16px;">—</div>
              <div class="label" style="margin-top:2px;">steps/s</div>
            </div>
            <div class="pill" style="min-width:90px;">
              <div class="label">Courant moteur</div>
              <div class="val" id="tuneCurrVal" style="font-size:16px;">—</div>
              <div class="label" style="margin-top:2px;">mA</div>
            </div>
          </div>
        </div>

        <div class="pos">
          Position: <code>X=<span id="x">--</span></code> <code>Y=<span id="y">--</span></code>
        </div>
      </div>

      <div class="gridWrap">
        <div>
          <div style="font-weight:700;margin-bottom:6px;">Sélection des cases</div>
          <div class="legend">Tap sur une pièce = FROM (bleu). Puis tap destination = TO (vert) pour envoyer le coup.</div>
<div class="boardWrap" id="boardWrap">
        <div id="turnOverlay" class="turnOverlay white">Tour: Blancs</div>

        <div id="board" class="board"></div>
  <div id="boardLoading" class="boardLoading">Calibration en cours…</div>
      </div>
        </div>
        <div style="min-width:220px;">
          <div class="pill">
            <div class="label">FROM</div>
            <div class="val"><span id="fromTxt">--</span></div>
          </div>
          <div class="pill" style="margin-top:10px;">
            <div class="label">TO</div>
            <div class="val"><span id="toTxt">--</span></div>
          </div>
          <button class="mini" id="resetBoard" style="margin-top:10px;width:100%;">♻️ Reset Board</button>
          <button class="mini" id="hardResetUi" style="margin-top:10px;width:100%;">🧹 Hard UI Reset</button>
          <button class="mini" id="swapMove" style="margin-top:10px;width:100%;">🔁 Swap</button>
        </div>
      </div>
    </div>

    <div class="card">
      <h1>📡 WiFi</h1>
      <div style="font-size:13px;opacity:.75;margin-bottom:14px;">
        Connecte l'ESP32 à ton réseau maison pour accéder au WebUI depuis n'importe quel appareil sans changer de WiFi.
        Une fois connecté, utilise <b>http://smartchessboard.local</b> ou l'IP affichée dans le port série.
      </div>
      <div style="display:flex;flex-direction:column;gap:10px;">
        <div>
          <div class="label" style="margin-bottom:5px;">Nom du réseau (SSID)</div>
          <input id="wifiSsid" type="text" placeholder="MonWiFi" autocomplete="off" autocorrect="off" spellcheck="false"
            style="width:100%;box-sizing:border-box;background:#0e1522;border:1px solid #1f2a3a;border-radius:10px;padding:10px 12px;color:#e8eef6;font-size:14px;outline:none;"/>
        </div>
        <div>
          <div class="label" style="margin-bottom:5px;">Mot de passe</div>
          <input id="wifiPass" type="password" placeholder="••••••••" autocomplete="new-password"
            style="width:100%;box-sizing:border-box;background:#0e1522;border:1px solid #1f2a3a;border-radius:10px;padding:10px 12px;color:#e8eef6;font-size:14px;outline:none;"/>
        </div>
        <button class="mini" id="wifiSaveBtn" style="margin-top:2px;width:100%;">💾 Sauvegarder et redémarrer</button>
        <div id="wifiMsg" style="font-size:12px;min-height:16px;"></div>
      </div>
    </div>
  </div>

<script>
  const statusEl = document.getElementById("status");
  const pctEl = document.getElementById("pct");
  const fillEl = document.getElementById("fill");
  const vEl = document.getElementById("v");
  const iEl = document.getElementById("i");
  const xEl = document.getElementById("x");
  const yEl = document.getElementById("y");

  const hallYDot = document.getElementById("hallYDot");
  const hallYText = document.getElementById("hallYText");
  const hallXDot = document.getElementById("hallXDot");
  const hallXText = document.getElementById("hallXText");
  const calibText = document.getElementById("calibText");

  const bSlow = document.getElementById("spSlow");
  const bNorm = document.getElementById("spNorm");
  const bFast = document.getElementById("spFast");

  const magnetBtn       = document.getElementById("magnetBtn");
  const calibBtn        = document.getElementById("calibBtn");
  const diag4Btn        = document.getElementById("diag4Btn");
  const tuneBtn         = document.getElementById("tuneBtn");
  const tuneStopBtn     = document.getElementById("tuneStopBtn");
  const testRunBtn      = document.getElementById("testRunBtn");
  const testRunStopBtn  = document.getElementById("testRunStopBtn");
  const testRunCard     = document.getElementById("testRunCard");
  const testRunStepTxt  = document.getElementById("testRunStepTxt");
  const testRunPctTxt   = document.getElementById("testRunPctTxt");
  const testRunFill     = document.getElementById("testRunFill");
  let testRunBusy = false;
  const tuneCard      = document.getElementById("tuneCard");
  const tunePhaseTxt  = document.getElementById("tunePhaseTxt");
  const tunePctTxt    = document.getElementById("tunePctTxt");
  const tuneFill      = document.getElementById("tuneFill");
  const tuneLogEl     = document.getElementById("tuneLog");
  const boardWrapEl   = document.getElementById("boardWrap");
  const tuneValidBadge = document.getElementById("tuneValidBadge");
  const tuneLiveBadge  = document.getElementById("tuneLiveBadge");
  const tuneSpdVal     = document.getElementById("tuneSpdVal");
  const tuneSpdDiagVal = document.getElementById("tuneSpdDiagVal");
  const tuneCurrVal    = document.getElementById("tuneCurrVal");
  const liveTuneSpdVal     = document.getElementById("liveTuneSpdVal");
  const liveTuneSpdDiagVal = document.getElementById("liveTuneSpdDiagVal");
  const liveTuneCurrVal    = document.getElementById("liveTuneCurrVal");
  let magnetOn   = false;
  let calibBusy  = false;
  let tuneBusy   = false;
  let tuneStartMs = null;

  const TUNE_PHASE_NAMES = [
    "Idle","Référence","Répétabilité","Rampe vitesse",
    "Comparaison efficacité","Courant moteur","Vérification finale",
    "Terminé ✅","Annulé","Erreur ❌"
  ];

  // [startPct, weightPct] for each TunePhase enum value (weights sum to 100)
  const PHASE_SPANS = [
    [0,   0],   // 0: IDLE
    [0,   5],   // 1: REFERENCE
    [5,   0],   // 2: REPEATABILITY (unused)
    [5,  55],   // 3: SPEED_RAMP (diag + axis)
    [60, 10],   // 4: efficiency comparison pass
    [70, 15],   // 5: CURRENT
    [85, 15],   // 6: FINAL CHARACTERIZE
    [100, 0],   // 7: DONE
    [100, 0],   // 8: ABORTED
    [100, 0],   // 9: ERROR
  ];

  function computeOverallPct(phase, phasePct) {
    if (phase >= PHASE_SPANS.length) return 100;
    const [start, weight] = PHASE_SPANS[phase];
    return Math.round(start + weight * phasePct / 100);
  }

  function updateTuneUI(tuning, phase, pct) {
    tuneBusy = !!tuning;

    tuneCard.style.display = tuneBusy ? "block" : "none";
    tuneBtn.style.display     = tuneBusy ? "none" : "";
    tuneStopBtn.style.display = tuneBusy ? "" : "none";
    if (tuneBusy || phase >= 7) {  // also show card for done/abort/error
      tuneCard.style.display = "block";
    }
    const name = TUNE_PHASE_NAMES[phase] || "—";
    tunePhaseTxt.textContent = name;

    // Affiche le pourcentage d'avancement, mais pas d'ETA/temps
    const overallPct = computeOverallPct(phase, pct);
    tunePctTxt.textContent = overallPct + "%";
    tuneFill.style.width   = overallPct + "%";

    tuneBtn.classList.toggle("active", false);
  }

  function appendTuneLog(msg) {
    if (!tuneLogEl) return;
    tuneLogEl.textContent += msg + "\n";
    tuneLogEl.scrollTop = tuneLogEl.scrollHeight;
  }

  function setStatus(msg, ok=true){
    statusEl.textContent = msg;
    statusEl.className = "status " + (ok ? "ok" : "warn");
  }
  function updateMagnetUI(on){
    magnetBtn.textContent = on ? "🧲 Electroaimant: ON (100%)" : "🧲 Electroaimant: OFF";
    magnetBtn.classList.toggle("active", on);
  }
  function updateHallUI(dotEl, textEl, detected){
    dotEl.classList.toggle("on", detected);
    textEl.textContent = detected ? "MAGNET DETECTED" : "No magnet";
  }
  function updateCalibUI(active){
    calibBusy = !!active;
    calibText.textContent = active ? "ON" : "OFF";
    calibBtn.classList.toggle("active", active);
    calibBtn.textContent = active ? "🎯 Calibration..." : "🎯 Calibration";
    if(boardWrapEl) boardWrapEl.classList.toggle("calibrating", calibBusy);
  }

  function updateTestRunUI(active, step, idx, total) {
    testRunBusy = !!active;
    testRunBtn.style.display     = testRunBusy ? "none" : "";
    testRunStopBtn.style.display = testRunBusy ? "" : "none";
    testRunCard.style.display    = testRunBusy ? "block" : "none";
    if (testRunBusy) {
      testRunStepTxt.textContent = step || "—";
      const pct = (total > 0) ? Math.round(100 * (idx + 1) / total) : 0;
      testRunPctTxt.textContent = pct + "%";
      testRunFill.style.width   = pct + "%";
    } else {
      testRunStepTxt.textContent = "—";
      testRunPctTxt.textContent  = "0%";
      testRunFill.style.width    = "0%";
    }
    // Disable test run button while auto tune is running
    testRunBtn.disabled = tuneBusy;
    tuneBtn.disabled    = testRunBusy;
  }

  let savedTuneCache = null;

  function renderTuneResultCard(valid, spd, spdDiag, curr) {
    if (valid) {
      tuneValidBadge.textContent = "✅ Valide — NVS";
      tuneValidBadge.style.background = "rgba(29,209,161,0.15)";
      tuneValidBadge.style.color      = "#1dd1a1";
      tuneValidBadge.style.border     = "1px solid rgba(29,209,161,0.4)";
      tuneSpdVal.textContent     = Math.round(spd);
      tuneSpdDiagVal.textContent = Math.round(spdDiag);
      tuneCurrVal.textContent    = curr;
    } else {
      tuneValidBadge.textContent = "Non calibré";
      tuneValidBadge.style.background = "#1f2a3a";
      tuneValidBadge.style.color      = "#888";
      tuneValidBadge.style.border     = "1px solid #2a3a4a";
      tuneSpdVal.textContent     = "—";
      tuneSpdDiagVal.textContent = "—";
      tuneCurrVal.textContent    = "—";
    }
  }

  function updateTuneResultCard(valid, spd, spdDiag, curr) {
    if (valid) {
      savedTuneCache = { spd, spdDiag, curr };
      renderTuneResultCard(true, spd, spdDiag, curr);
      return;
    }

    if (tuneBusy && savedTuneCache) {
      renderTuneResultCard(true,
        savedTuneCache.spd,
        savedTuneCache.spdDiag,
        savedTuneCache.curr);
      return;
    }

    renderTuneResultCard(false, spd, spdDiag, curr);
  }

  function updateLiveTuneCard(active, spd, spdDiag, curr) {
    if (active) {
      tuneLiveBadge.textContent = "Actif";
      tuneLiveBadge.style.background = "rgba(84,160,255,0.16)";
      tuneLiveBadge.style.color      = "#54a0ff";
      tuneLiveBadge.style.border     = "1px solid rgba(84,160,255,0.45)";
      liveTuneSpdVal.textContent     = Math.round(spd);
      liveTuneSpdDiagVal.textContent = Math.round(spdDiag);
      liveTuneCurrVal.textContent    = curr > 0 ? curr : "—";
    } else {
      tuneLiveBadge.textContent = "Inactif";
      tuneLiveBadge.style.background = "#1f2a3a";
      tuneLiveBadge.style.color      = "#888";
      tuneLiveBadge.style.border     = "1px solid #2a3a4a";
      liveTuneSpdVal.textContent     = "—";
      liveTuneSpdDiagVal.textContent = "—";
      liveTuneCurrVal.textContent    = "—";
    }
  }

  const wsUrl = `ws://${location.hostname}:81/`;
  let ws;

  function send(obj){
    if(!ws || ws.readyState !== WebSocket.OPEN) return;
    ws.send(JSON.stringify(obj));
  }
  function setActive(btn){
    [bSlow,bNorm,bFast].forEach(b=>b.classList.remove("active"));
    btn.classList.add("active");
  }

  let holdTimer = null;
  let holdDir = null;

  function startHold(dir){
    holdDir = dir;
    send({cmd:"move", dir: holdDir});
    if (holdTimer) clearInterval(holdTimer);
    holdTimer = setInterval(() => send({cmd:"move", dir: holdDir}), 80);
  }
  function stopHold(){
    holdDir = null;
    if (holdTimer) { clearInterval(holdTimer); holdTimer = null; }
    send({cmd:"move", dir:"stop"});
  }
  function bindHold(btnId, dir){
    const el = document.getElementById(btnId);
    el.addEventListener("mousedown", ()=>startHold(dir));
    el.addEventListener("mouseup", stopHold);
    el.addEventListener("mouseleave", stopHold);

    el.addEventListener("touchstart", (e)=>{ e.preventDefault(); startHold(dir); }, {passive:false});
    el.addEventListener("touchend", (e)=>{ e.preventDefault(); stopHold(); }, {passive:false});
    el.addEventListener("touchcancel", (e)=>{ e.preventDefault(); stopHold(); }, {passive:false});
  }

  const boardEl = document.getElementById("board");
  
  const sqEls = {}; // key -> div
  function flashIllegal(key){
    const el = sqEls[key];
    if(!el) return;
    el.classList.remove("illegalFlash");
    // reflow to restart animation
    void el.offsetWidth;
    el.classList.add("illegalFlash");
    setTimeout(()=>el.classList.remove("illegalFlash"), 600);
  }
const fromTxt = document.getElementById("fromTxt");
  const toTxt = document.getElementById("toTxt");

  let from = null;
  let to   = null;

  function labelOf(s){
    if(!s) return "--";
    return String.fromCharCode(97+s.f) + String(s.r);
  }

// ================================
// Board state (UI side)
// ================================
const pieceSym = {
  wK:"♔", wQ:"♕", wR:"♖", wB:"♗", wN:"♘", wP:"♙",
  bK:"♚", bQ:"♛", bR:"♜", bB:"♝", bN:"♞", bP:"♟"
};

// boardState maps "e4" -> pieceId like "wP5", "bQ", etc.
let boardState = {};
let deadZones = { L:new Array(16).fill(null), R:new Array(16).fill(null), nextL:15, nextR:15 };
let pathBusy = false;
let resetInProgress = false;
const UI_STATE_KEY = "smartchess.ui.state.v1";
let fwBusy = false;
let fwPending = 0;
let resetPlan = [];
let resetPlanIndex = 0;
let resetTargetBoard = null;

  // ======================================================
  // Chess rules (basic legality + king safety)
  // Note: no castling and no en-passant yet.
  // ======================================================
  let turn = "w"; // 'w' or 'b'
  let promoCountW = 0, promoCountB = 0;
  // Special moves + check indicator
  let castling = { wK:true, wQ:true, bK:true, bQ:true };
  let epSquare = null;      // e.g. "e3"
  let checkSquare = null;   // king square of side-to-move if in check

  function resetSpecialRights(){
    castling = { wK:true, wQ:true, bK:true, bQ:true };
    epSquare = null;
    checkSquare = null;
  }

  function recomputeDeadZonePointers(){
    deadZones.nextL = deadZones.L.length - 1;
    while(deadZones.nextL >= 0 && deadZones.L[deadZones.nextL]) deadZones.nextL--;

    deadZones.nextR = deadZones.R.length - 1;
    while(deadZones.nextR >= 0 && deadZones.R[deadZones.nextR]) deadZones.nextR--;
  }

  function saveUiState(){
    try {
      const payload = {
        boardState,
        deadZones,
        turn,
        promoCountW,
        promoCountB,
        castling,
        epSquare,
      };
      localStorage.setItem(UI_STATE_KEY, JSON.stringify(payload));
    } catch(e) {}
  }

  function loadUiState(){
    try {
      const raw = localStorage.getItem(UI_STATE_KEY);
      if(!raw) return false;

      const saved = JSON.parse(raw);
      if(!saved || typeof saved !== "object") return false;

      if(!saved.boardState || typeof saved.boardState !== "object") return false;
      boardState = saved.boardState;

      if(saved.deadZones && Array.isArray(saved.deadZones.L) && Array.isArray(saved.deadZones.R)) {
        deadZones = {
          L: saved.deadZones.L.slice(0, 16),
          R: saved.deadZones.R.slice(0, 16),
          nextL: 15,
          nextR: 15,
        };
        while(deadZones.L.length < 16) deadZones.L.push(null);
        while(deadZones.R.length < 16) deadZones.R.push(null);
      } else {
        deadZones = { L:new Array(16).fill(null), R:new Array(16).fill(null), nextL:15, nextR:15 };
      }
      recomputeDeadZonePointers();

      turn = (saved.turn === "b") ? "b" : "w";
      promoCountW = Number.isFinite(saved.promoCountW) ? Math.max(0, saved.promoCountW|0) : 0;
      promoCountB = Number.isFinite(saved.promoCountB) ? Math.max(0, saved.promoCountB|0) : 0;

      const c = saved.castling || {};
      castling = {
        wK: !!c.wK,
        wQ: !!c.wQ,
        bK: !!c.bK,
        bQ: !!c.bQ,
      };
      epSquare = (typeof saved.epSquare === "string" && saved.epSquare.length === 2) ? saved.epSquare : null;
      checkSquare = null;

      from = null;
      to = null;
      selectedPieceId = null;
      if(typeof fromTxt!=="undefined") fromTxt.textContent = "--";
      if(typeof toTxt!=="undefined") toTxt.textContent = "--";
      return true;
    } catch(e) {
      return false;
    }
  }

  function updateCheckState(){
    const kingSq = findKing(boardState, turn);
    const opp = (turn === "w") ? "b" : "w";
    checkSquare = (kingSq && isSquareAttacked(boardState, kingSq, opp)) ? kingSq : null;
  }


  function pieceGlyph(pid){
    if(!pid) return "";
    const c = pieceColor(pid); // 'w' or 'b'
    const t = pieceType(pid);
    const W = {K:"♔", Q:"♕", R:"♖", B:"♗", N:"♘", P:"♙"};
    const B = {K:"♚", Q:"♛", R:"♜", B:"♝", N:"♞", P:"♟"};
    return (c==="w" ? (W[t]||"") : (B[t]||""));
  }

  function updateTurnOverlay(){
    const el = document.getElementById("turnOverlay");
    if(!el) return;
    const isW = (turn === "w");
    el.textContent = "Tour: " + (isW ? "Blancs" : "Noirs");
    el.classList.toggle("white", isW);
    el.classList.toggle("black", !isW);
  }


  function pieceColor(pid){ return pid ? pid[0] : null; }
  function pieceType(pid){ return pid ? pid[1] : null; } // P,R,N,B,Q,K

  function sqToXY(sq){
    const f = sq.charCodeAt(0) - 97; // a=0
    const r = parseInt(sq[1],10);    // '1'..'8' (1-based, matches sqKey)
    return {f,r};
  }
  function xyToSq(f,r){
    // r is 1..8 (1-based, matches sqKey)
    return String.fromCharCode(97+f) + String(r);
  }

  function getPieceAt(board, f, r){ return board[xyToSq(f,r)] || null; }
  function setPieceAt(board, f, r, pid){
    const k = xyToSq(f,r);
    if(pid) board[k] = pid; else delete board[k];
  }
  function cloneBoard(board){ return Object.assign({}, board); }

  function findKing(board, color){
    for(const sq in board){
      if(board[sq] === color + "K") return sq;
    }
    return null;
  }

  function rayClear(board, f0, r0, f1, r1){
    const df = Math.sign(f1 - f0);
    const dr = Math.sign(r1 - r0);
    let f = f0 + df, r = r0 + dr;
    while(f !== f1 || r !== r1){
      if(getPieceAt(board,f,r)) return false;
      f += df; r += dr;
    }
    return true;
  }

  function isSlidingPiece(pid){
    const t = pieceType(pid);
    return (t === "B" || t === "R" || t === "Q");
  }

  function isPathClear(board, from, to, pid){
    if(!pid) return true;
    if(!isSlidingPiece(pid)) return true;
    return rayClear(board, from.f, from.r, to.f, to.r);
  }

  const MAX_WP = 12;

  function isBlocked(board, f, r, from, to){
    if(from && f === from.f && r === from.r) return false;
    if(to && f === to.f && r === to.r) return false;
    return !!getPieceAt(board, f, r);
  }

  function canStepDiagonal(board, cur, nf, nr, from, to){
    const df = nf - cur.f;
    const dr = nr - cur.r;
    if(Math.abs(df) !== 1 || Math.abs(dr) !== 1) return true;
    const blockH = isBlocked(board, cur.f + df, cur.r, from, to);
    const blockV = isBlocked(board, cur.f, cur.r + dr, from, to);
    return !(blockH && blockV);
  }

  function findShortestPath(board, from, to){
    if(!from || !to) return null;
    const startKey = from.f + "," + from.r;
    const goalKey = to.f + "," + to.r;

    const queue = [from];
    const visited = {};
    const prev = {};
    visited[startKey] = true;

    const dirs = [
      {df:1, dr:0},
      {df:-1, dr:0},
      {df:0, dr:1},
      {df:0, dr:-1},
      {df:1, dr:1},
      {df:1, dr:-1},
      {df:-1, dr:1},
      {df:-1, dr:-1}
    ];

    while(queue.length){
      const cur = queue.shift();
      const curKey = cur.f + "," + cur.r;
      if(curKey === goalKey) break;

      for(const d of dirs){
        const nf = cur.f + d.df;
        const nr = cur.r + d.dr;
        if(nf < 0 || nf > 7 || nr < 1 || nr > 8) continue;
        if(isBlocked(board, nf, nr, from, to)) continue;
        if(!canStepDiagonal(board, cur, nf, nr, from, to)) continue;
        const nk = nf + "," + nr;
        if(visited[nk]) continue;
        visited[nk] = true;
        prev[nk] = curKey;
        queue.push({f:nf, r:nr});
      }
    }

    if(!visited[goalKey]) return null;

    const path = [];
    let k = goalKey;
    while(k){
      const parts = k.split(",");
      path.push({f:parseInt(parts[0],10), r:parseInt(parts[1],10)});
      k = prev[k];
    }
    path.reverse();
    return path;
  }

  function attacksSquare(board, fromSq, pid, targetSq){
    const c = pieceColor(pid);
    const t = pieceType(pid);
    const a = sqToXY(fromSq), b = sqToXY(targetSq);
    const df = b.f - a.f;
    const dr = b.r - a.r;
    const adf = Math.abs(df), adr = Math.abs(dr);

    if(t === "P"){
      const dir = (c === "w") ? 1 : -1;
      return (adr === 1 && adf === 1 && dr === dir);
    }
    if(t === "N"){
      return (adf === 1 && adr === 2) || (adf === 2 && adr === 1);
    }
    if(t === "B"){
      if(adf !== adr) return false;
      return rayClear(board, a.f,a.r,b.f,b.r);
    }
    if(t === "R"){
      if(!(df === 0 || dr === 0)) return false;
      return rayClear(board, a.f,a.r,b.f,b.r);
    }
    if(t === "Q"){
      if(adf === adr || df === 0 || dr === 0) return rayClear(board, a.f,a.r,b.f,b.r);
      return false;
    }
    if(t === "K"){
      return adf <= 1 && adr <= 1 && !(adf === 0 && adr === 0);
    }
    return false;
  }

  function isSquareAttacked(board, targetSq, byColor){
    for(const sq in board){
      const pid = board[sq];
      if(pieceColor(pid) !== byColor) continue;
      if(attacksSquare(board, sq, pid, targetSq)) return true;
    }
    return false;
  }

  function validateMove(board, fromSq, toSq){
    const pid = board[fromSq];
    let epCaptureSq = null;
    let castle = null;
    let epNext = null;
    let rightsUpdate = null;
    if(!pid) return {ok:false, reason:"Aucune pièce sur la case FROM."};

    const c = pieceColor(pid);
    const t = pieceType(pid);

    if(c !== turn) return {ok:false, reason:`Ce n'est pas le tour des ${turn==="w"?"blancs":"noirs"}.`};

    const dest = board[toSq] || null;
    if(dest && pieceColor(dest) === c) return {ok:false, reason:"Tu ne peux pas capturer ta propre pièce."};

    const a = sqToXY(fromSq), b = sqToXY(toSq);
    const df = b.f - a.f;
    const dr = b.r - a.r;
    const adf = Math.abs(df), adr = Math.abs(dr);

    if(t === "P"){
      const dir = (c === "w") ? 1 : -1;
      const startRank = (c === "w") ? 2 : 7;

      if(adf === 1 && dr === dir){
        if(!dest){
          if(epSquare && toSq === epSquare){
            const behindSq = xyToSq(b.f, b.r - dir);
            const behindPid = board[behindSq] || null;
            if(behindPid && pieceType(behindPid)==="P" && pieceColor(behindPid)!==c){
              epCaptureSq = behindSq;
            } else {
              return {ok:false, reason:"En passant impossible (pas de pion à capturer)."};
            }
          } else {
            return {ok:false, reason:"Un pion capture seulement en diagonale s'il y a une pièce (ou en passant)."};
          }
        }
      } else if(df === 0 && dr === dir){
        if(dest) return {ok:false, reason:"Un pion ne peut pas avancer sur une case occupée."};
      } else if(df === 0 && dr === 2*dir && a.r === startRank){
        epNext = xyToSq(a.f, a.r + dir);
        const midSq = xyToSq(a.f, a.r + dir);
        if(dest) return {ok:false, reason:"Un pion ne peut pas avancer sur une case occupée."};
        if(board[midSq]) return {ok:false, reason:"Le pion ne peut pas sauter par-dessus une pièce."};
      } else {
        return {ok:false, reason:"Déplacement de pion illégal."};
      }
    } else if(t === "N"){
      if(!((adf === 1 && adr === 2) || (adf === 2 && adr === 1)))
        return {ok:false, reason:"Déplacement de cavalier illégal."};
    } else if(t === "B"){
      if(adf !== adr) return {ok:false, reason:"Déplacement de fou illégal."};
      if(!rayClear(board, a.f,a.r,b.f,b.r)) return {ok:false, reason:"Chemin bloqué."};
    } else if(t === "R"){
      if(!(df === 0 || dr === 0)) return {ok:false, reason:"Déplacement de tour illégal."};
      if(!rayClear(board, a.f,a.r,b.f,b.r)) return {ok:false, reason:"Chemin bloqué."};
    } else if(t === "Q"){
      if(!(adf === adr || df === 0 || dr === 0)) return {ok:false, reason:"Déplacement de reine illégal."};
      if(!rayClear(board, a.f,a.r,b.f,b.r)) return {ok:false, reason:"Chemin bloqué."};
    } else if(t === "K"){
      if(adr === 0 && adf === 2){
        const opp = (c === "w") ? "b" : "w";
        if(isSquareAttacked(board, fromSq, opp)){
          return {ok:false, reason:"Roque interdit : le roi est en échec."};
        }

        if(c==="w" && fromSq==="e1" && toSq==="g1"){
          if(!castling.wK) return {ok:false, reason:"Roque côté roi non disponible."};
          if(board["h1"]!=="wR") return {ok:false, reason:"Roque impossible : tour manquante en h1."};
          if(board["f1"]||board["g1"]) return {ok:false, reason:"Roque impossible : cases entre le roi et la tour occupées."};
          if(isSquareAttacked(board, "f1", opp) || isSquareAttacked(board, "g1", opp)){
            return {ok:false, reason:"Roque interdit : le roi passerait par une case attaquée."};
          }
          castle = { rookFrom:"h1", rookTo:"f1" };
          rightsUpdate = { wK:false, wQ:false };
        } else if(c==="w" && fromSq==="e1" && toSq==="c1"){
          if(!castling.wQ) return {ok:false, reason:"Roque côté reine non disponible."};
          if(board["a1"]!=="wR") return {ok:false, reason:"Roque impossible : tour manquante en a1."};
          if(board["b1"]||board["c1"]||board["d1"]) return {ok:false, reason:"Roque impossible : cases entre le roi et la tour occupées."};
          if(isSquareAttacked(board, "d1", opp) || isSquareAttacked(board, "c1", opp)){
            return {ok:false, reason:"Roque interdit : le roi passerait par une case attaquée."};
          }
          castle = { rookFrom:"a1", rookTo:"d1" };
          rightsUpdate = { wK:false, wQ:false };
        } else if(c==="b" && fromSq==="e8" && toSq==="g8"){
          if(!castling.bK) return {ok:false, reason:"Roque côté roi non disponible."};
          if(board["h8"]!=="bR") return {ok:false, reason:"Roque impossible : tour manquante en h8."};
          if(board["f8"]||board["g8"]) return {ok:false, reason:"Roque impossible : cases entre le roi et la tour occupées."};
          if(isSquareAttacked(board, "f8", opp) || isSquareAttacked(board, "g8", opp)){
            return {ok:false, reason:"Roque interdit : le roi passerait par une case attaquée."};
          }
          castle = { rookFrom:"h8", rookTo:"f8" };
          rightsUpdate = { bK:false, bQ:false };
        } else if(c==="b" && fromSq==="e8" && toSq==="c8"){
          if(!castling.bQ) return {ok:false, reason:"Roque côté reine non disponible."};
          if(board["a8"]!=="bR") return {ok:false, reason:"Roque impossible : tour manquante en a8."};
          if(board["b8"]||board["c8"]||board["d8"]) return {ok:false, reason:"Roque impossible : cases entre le roi et la tour occupées."};
          if(isSquareAttacked(board, "d8", opp) || isSquareAttacked(board, "c8", opp)){
            return {ok:false, reason:"Roque interdit : le roi passerait par une case attaquée."};
          }
          castle = { rookFrom:"a8", rookTo:"d8" };
          rightsUpdate = { bK:false, bQ:false };
        } else {
          return {ok:false, reason:"Roque non valide depuis cette position."};
        }
      } else {
        if(adf > 1 || adr > 1) return {ok:false, reason:"Déplacement de roi illégal."};
        rightsUpdate = (c==="w") ? { wK:false, wQ:false } : { bK:false, bQ:false };
      }
    }

    
    // Castling rights updates (rook moved / captured on start square)
    if(t === "R"){
      if(c==="w"){
        if(fromSq==="h1") rightsUpdate = Object.assign({}, rightsUpdate||{}, {wK:false});
        if(fromSq==="a1") rightsUpdate = Object.assign({}, rightsUpdate||{}, {wQ:false});
      } else {
        if(fromSq==="h8") rightsUpdate = Object.assign({}, rightsUpdate||{}, {bK:false});
        if(fromSq==="a8") rightsUpdate = Object.assign({}, rightsUpdate||{}, {bQ:false});
      }
    }
    if(dest){
      const dc = pieceColor(dest);
      const dt = pieceType(dest);
      if(dt==="R"){
        if(dc==="w"){
          if(toSq==="h1") rightsUpdate = Object.assign({}, rightsUpdate||{}, {wK:false});
          if(toSq==="a1") rightsUpdate = Object.assign({}, rightsUpdate||{}, {wQ:false});
        } else {
          if(toSq==="h8") rightsUpdate = Object.assign({}, rightsUpdate||{}, {bK:false});
          if(toSq==="a8") rightsUpdate = Object.assign({}, rightsUpdate||{}, {bQ:false});
        }
      }
    }

// King safety
    const next = cloneBoard(board);
    setPieceAt(next, a.f,a.r, null);
    setPieceAt(next, b.f,b.r, pid);

    if(epCaptureSq){
      const ep = sqToXY(epCaptureSq);
      setPieceAt(next, ep.f, ep.r, null);
    }

    if(castle){
      const rf = sqToXY(castle.rookFrom);
      const rt = sqToXY(castle.rookTo);
      const rookPid = board[castle.rookFrom];
      setPieceAt(next, rf.f, rf.r, null);
      setPieceAt(next, rt.f, rt.r, rookPid);
    }

    // Promotion (auto queen)
    if(t === "P"){
      const promoteRank = (c === "w") ? 8 : 1;
      if(b.r === promoteRank){
        if(c === "w"){ promoCountW++; setPieceAt(next, b.f,b.r, "wQp"+promoCountW); }
        else { promoCountB++; setPieceAt(next, b.f,b.r, "bQp"+promoCountB); }
      }
    }

    const kingSq = findKing(next, c);
    if(!kingSq) return {ok:false, reason:"Roi introuvable (state UI incohérent)."};
    const opp = (c === "w") ? "b" : "w";
    if(isSquareAttacked(next, kingSq, opp)){
      return {ok:false, reason:"Coup illégal : ton roi serait en échec."};
    }

    const opp2 = (c === 'w') ? 'b' : 'w';
    const oppKing = findKing(next, opp2);
    const givesCheck = (oppKing && isSquareAttacked(next, oppKing, c)) ? true : false;
    return {ok:true, nextBoard: next, givesCheck, epNext, rightsUpdate};
  }


let selectedPieceId = null;

function sqKey(f,r){ return String.fromCharCode(97+f) + String(r); }
function pieceBase(id){ return id ? id.slice(0,2) : null; }

function addToCapturedZone(pid, side){
  // side: 'L' or 'R'
  if(!pid) return;
  const zone = deadZones[side];
  const key = side === 'L' ? 'nextL' : 'nextR';
  if(deadZones[key] >= 0){
    zone[deadZones[key]] = pid;
    deadZones[key]--;
  }
}

function removeCaptured(side, idx){
  const zone = deadZones[side];
  if(zone[idx]){
    zone[idx] = null;
  }
}

function getCapturedDisplay(side){
  const zone = deadZones[side];
  const captured = [];
  for(let i = 0; i < zone.length; i++){
    if(zone[i]){
      const base = pieceBase(zone[i]);
      if(pieceSym[base]){
        captured.push({idx:i, sym:pieceSym[base], pid:zone[i], color:zone[i][0]});
      }
    }
  }
  return captured;
}

function resetStandardPosition(){
  turn = "w";
  promoCountW = 0; promoCountB = 0;
  resetSpecialRights();
  boardState = {};

  // --- White pieces (bottom: ranks 1-2)
  boardState["a1"]="wR"; boardState["b1"]="wN"; boardState["c1"]="wB"; boardState["d1"]="wQ";
  boardState["e1"]="wK"; boardState["f1"]="wB"; boardState["g1"]="wN"; boardState["h1"]="wR";
  for(const f of ["a","b","c","d","e","f","g","h"]){ boardState[f+"2"]="wP"; }

  // --- Black pieces (top: ranks 8-7)
  boardState["a8"]="bR"; boardState["b8"]="bN"; boardState["c8"]="bB"; boardState["d8"]="bQ";
  boardState["e8"]="bK"; boardState["f8"]="bB"; boardState["g8"]="bN"; boardState["h8"]="bR";
  for(const f of ["a","b","c","d","e","f","g","h"]){ boardState[f+"7"]="bP"; }

  // clear selection UI
  from = null; to = null; selectedPieceId = null;
  if(typeof fromTxt!=="undefined") fromTxt.textContent="--";
  if(typeof toTxt!=="undefined") toTxt.textContent="--";
  saveUiState();
}

function applyLocalMove(ff,fr,tf,tr){
  const kFrom = sqKey(ff,fr);
  const kTo   = sqKey(tf,tr);
  const pid = boardState[kFrom];
  if(!pid) return false;
  // capture if occupied
  delete boardState[kFrom];
  boardState[kTo] = pid;
  return true;
}

function pieceBaseId(pid){ return pid ? pid.slice(0,2) : null; }

function buildStandardBoardState(){
  const target = {};
  target["a1"]="wR"; target["b1"]="wN"; target["c1"]="wB"; target["d1"]="wQ";
  target["e1"]="wK"; target["f1"]="wB"; target["g1"]="wN"; target["h1"]="wR";
  for(const f of ["a","b","c","d","e","f","g","h"]) target[f+"2"] = "wP";

  target["a8"]="bR"; target["b8"]="bN"; target["c8"]="bB"; target["d8"]="bQ";
  target["e8"]="bK"; target["f8"]="bB"; target["g8"]="bN"; target["h8"]="bR";
  for(const f of ["a","b","c","d","e","f","g","h"]) target[f+"7"] = "bP";
  return target;
}

function allSquares(){
  const out = [];
  for(let r=1;r<=8;r++) for(let f=0;f<8;f++) out.push(xyToSq(f,r));
  return out;
}

function findSquareWithBase(work, base, avoidSq){
  for(const sq of allSquares()){
    if(sq === avoidSq) continue;
    const p = work[sq];
    if(pieceBaseId(p) === base) return sq;
  }
  return null;
}

function findTempSquare(work, target, blocked){
  for(const sq of allSquares()){
    if(blocked && blocked[sq]) continue;
    if(work[sq]) continue;
    if(!target[sq]) return sq;
  }
  for(const sq of allSquares()){
    if(blocked && blocked[sq]) continue;
    if(!work[sq]) return sq;
  }
  return null;
}

function addPlannedMove(work, moves, fromSq, toSq){
  const pid = work[fromSq];
  if(!pid) return false;
  delete work[fromSq];
  work[toSq] = pid;
  moves.push({fromSq, toSq});
  return true;
}

function buildPhysicalResetPlan(currentBoard){
  const target = buildStandardBoardState();
  const work = cloneBoard(currentBoard);
  const moves = [];

  for(const targetSq of allSquares()){
    const wantBase = pieceBaseId(target[targetSq]);
    if(!wantBase) continue;

    const curBase = pieceBaseId(work[targetSq]);
    if(curBase === wantBase) continue;

    const srcSq = findSquareWithBase(work, wantBase, targetSq);
    if(!srcSq) continue;

    if(work[targetSq]) {
      const blocked = {};
      blocked[targetSq] = true;
      blocked[srcSq] = true;
      const tmpSq = findTempSquare(work, target, blocked);
      if(!tmpSq) continue;
      addPlannedMove(work, moves, targetSq, tmpSq);
    }

    addPlannedMove(work, moves, srcSq, targetSq);
  }

  return {moves, target};
}

function dispatchResetMove(fromSq, toSq){
  const from = sqToXY(fromSq);
  const to = sqToXY(toSq);
  const pid = getPieceAt(boardState, from.f, from.r);
  const path = findShortestPath(boardState, from, to);

  if(path && path.length >= 2 && path.length <= MAX_WP){
    send({cmd:"pathMove", path});
  } else {
    // Prefer planner-based motion when BFS path is not available,
    // to avoid straight-line drag-through across other pieces.
    const isKing = pid && pieceType(pid) === "K";
    const isCastleLike = isKing && (from.f === 4) && (Math.abs(to.f - from.f) === 2) && (from.r === 1 || from.r === 8) && (from.r === to.r);

    if (isCastleLike) {
      // Avoid castling special-case in squareMove during reset shuffling.
      send({cmd:"squareMoveDirect", ff:from.f, fr:from.r, tf:to.f, tr:to.r});
    } else {
      send({cmd:"squareMove", ff:from.f, fr:from.r, tf:to.f, tr:to.r});
    }
  }

  applyLocalMove(from.f, from.r, to.f, to.r);
}

function finishPhysicalReset(ok){
  if(ok && resetTargetBoard){
    boardState = cloneBoard(resetTargetBoard);
    turn = "w";
    promoCountW = 0;
    promoCountB = 0;
    resetSpecialRights();
    deadZones = { L:new Array(16).fill(null), R:new Array(16).fill(null), nextL:15, nextR:15 };
  }

  from = null;
  to = null;
  selectedPieceId = null;
  fromTxt.textContent = "--";
  toTxt.textContent = "--";
  updateCheckState();
  updateTurnOverlay();
  saveUiState();
  renderBoard();

  resetPlan = [];
  resetPlanIndex = 0;
  resetTargetBoard = null;
  resetInProgress = false;
  setStatus(ok ? "Pièces réinitialisées physiquement" : "Reset interrompu", ok);
}

function runPhysicalResetPump(){
  if(!resetInProgress) return;
  if(fwBusy || fwPending > 0) return;

  while(resetPlanIndex < resetPlan.length){
    const step = resetPlan[resetPlanIndex];
    const pid = boardState[step.fromSq];
    if(!pid){
      resetPlanIndex++;
      continue;
    }

    dispatchResetMove(step.fromSq, step.toSq);
    resetPlanIndex++;
    setStatus("Reset physique: " + resetPlanIndex + "/" + resetPlan.length, true);
    saveUiState();
    renderBoard();
    return;
  }

  finishPhysicalReset(true);
}

function startPhysicalReset(){
  if(resetInProgress){
    setStatus("Reset déjà en cours...", false);
    return;
  }

  const planData = buildPhysicalResetPlan(boardState);
  resetPlan = planData.moves;
  resetPlanIndex = 0;
  resetTargetBoard = planData.target;

  if(resetPlan.length === 0){
    finishPhysicalReset(true);
    return;
  }

  resetInProgress = true;
  send({cmd:"resetPieces"});
  setStatus("Préparation du reset physique...", true);
  runPhysicalResetPump();
}

  function renderBoard(){
  boardEl.innerHTML = "";
  for(let r=8; r>=1; r--){
    for(let f=0; f<8; f++){
      const div = document.createElement("div");
      const light = ((f + r) % 2) === 0;
      div.className = "sq " + (light ? "light" : "");
      const key = sqKey(f,r);
      
      sqEls[key] = div;

      if(checkSquare && key===checkSquare){ div.classList.add("check"); }
const pid = boardState[key];

      // Content: piece + coord
      const coord = `<span class="coord">${key}</span>`;
      if(pid){
        const base = pieceBase(pid);
        const sym = pieceSym[base] || "•";
        div.innerHTML = `<span class="piece ${pid[0]==="w" ? "white" : "black"}">${sym}</span>` + coord;
        div.classList.add("occupied");
      }else{
        div.innerHTML = coord;
      }

      if(from && from.f===f && from.r===r) div.classList.add("selFrom");
      if(to   && to.f===f   && to.r===r)   div.classList.add("selTo");

      div.addEventListener("pointerup", (e)=>{
        if(resetInProgress){
          setStatus("Reset en cours, attends la fin des déplacements.", false);
          return;
        }

        // ----------------------------
        // Step 0: no FROM yet -> pick a piece with a normal tap/click
        // ----------------------------
        if(!from){
          if(!pid){
            setStatus("Sélectionne d'abord une pièce (case occupée).", false);
            return;
          }
          from = {f,r};
          selectedPieceId = pid;
          fromTxt.textContent = key;
          to = null;
          toTxt.textContent = "--";
          renderBoard();
          return;
        }

        // Tapping the same FROM square toggles off.
        if(from && from.f===f && from.r===r){
          from = null;
          selectedPieceId = null;
          fromTxt.textContent = "--";
          to = null;
          toTxt.textContent = "--";
          renderBoard();
          return;
        }

        // Clicking another friendly piece switches FROM.
        const fromPid = selectedPieceId || getPieceAt(boardState, from.f, from.r);
        if(pid && fromPid && pieceColor(pid) === pieceColor(fromPid)){
          from = {f,r};
          selectedPieceId = pid;
          fromTxt.textContent = key;
          to = null;
          toTxt.textContent = "--";
          renderBoard();
          return;
        }

        // Otherwise this click is TO, then send move immediately.
        to = {f,r};
        toTxt.textContent = key;
        renderBoard();
        executeSelectedMove();
      });

      boardEl.appendChild(div);
    }
  }
}

  function executeSelectedMove(){
    if(resetInProgress){
      setStatus("Reset en cours, mouvement manuel temporairement bloqué.", false);
      return;
    }

    if(!from || !to){
      setStatus("Sélectionne une pièce (FROM), puis une destination (TO).", false);
      return;
    }
    const fromSq = xyToSq(from.f, from.r);
    const toSq = xyToSq(to.f, to.r);

    const res = validateMove(boardState, fromSq, toSq);
    if(!res.ok){
      setStatus(res.reason, false);
      if(to){ flashIllegal(xyToSq(to.f,to.r)); }
      else if(from){ flashIllegal(xyToSq(from.f,from.r)); }
      return;
    }

    const pid = getPieceAt(boardState, from.f, from.r);
    const t = pieceType(pid);

    if (t === "K" && Math.abs(to.f - from.f) === 2) {
      send({cmd:"squareMove", ff:from.f, fr:from.r, tf:to.f, tr:to.r});
    } else if (t === "N") {
      const path = findShortestPath(boardState, from, to);
      if(!path || path.length < 2){
        // Check for capture BEFORE applying move
        const destPid = getPieceAt(boardState, to.f, to.r);
        if(destPid){
          const destSide = pieceColor(destPid) === 'w' ? 'L' : 'R';
          addToCapturedZone(destPid, destSide);
        }
        send({cmd:"squareMove", ff:from.f, fr:from.r, tf:to.f, tr:to.r});
        boardState = res.nextBoard;
        if(res.rightsUpdate){
          castling = Object.assign({}, castling, res.rightsUpdate);
        }
        epSquare = res.epNext ? res.epNext : null;

        from = null; to = null; selectedPieceId = null;
        fromTxt.textContent = "--";
        toTxt.textContent = "--";

        turn = (turn === "w") ? "b" : "w";
        updateTurnOverlay();

        updateCheckState();
        if(checkSquare){
          setStatus("ÉCHEC ! Tour: " + (turn==="w" ? "Blancs" : "Noirs"), false);
        } else if(res.givesCheck){
          setStatus("ÉCHEC ! Tour: " + (turn==="w" ? "Blancs" : "Noirs"), false);
        } else {
          setStatus("OK. Tour: " + (turn==="w" ? "Blancs" : "Noirs"), true);
        }

        renderBoard();
        saveUiState();
        return;
      }
      if(path.length > MAX_WP){
        // Check for capture BEFORE applying move
        const destPid = getPieceAt(boardState, to.f, to.r);
        if(destPid){
          const destSide = pieceColor(destPid) === 'w' ? 'L' : 'R';
          addToCapturedZone(destPid, destSide);
        }
        send({cmd:"squareMove", ff:from.f, fr:from.r, tf:to.f, tr:to.r});
        boardState = res.nextBoard;
        if(res.rightsUpdate){
          castling = Object.assign({}, castling, res.rightsUpdate);
        }
        epSquare = res.epNext ? res.epNext : null;

        from = null; to = null; selectedPieceId = null;
        fromTxt.textContent = "--";
        toTxt.textContent = "--";

        turn = (turn === "w") ? "b" : "w";
        updateTurnOverlay();

        updateCheckState();
        if(checkSquare){
          setStatus("ÉCHEC ! Tour: " + (turn==="w" ? "Blancs" : "Noirs"), false);
        } else if(res.givesCheck){
          setStatus("ÉCHEC ! Tour: " + (turn==="w" ? "Blancs" : "Noirs"), false);
        } else {
          setStatus("OK. Tour: " + (turn==="w" ? "Blancs" : "Noirs"), true);
        }

        renderBoard();
        saveUiState();
        return;
      }
      send({cmd:"pathMove", path});
    } else if (isSlidingPiece(pid)) {
      const clear = isPathClear(boardState, from, to, pid);
      if (clear) {
        // Check for capture BEFORE applying move
        const destPid = getPieceAt(boardState, to.f, to.r);
        if(destPid){
          const destSide = pieceColor(destPid) === 'w' ? 'L' : 'R';
          addToCapturedZone(destPid, destSide);
        }
        send({cmd:"squareMoveDirect", ff:from.f, fr:from.r, tf:to.f, tr:to.r});
      } else {
        const path = findShortestPath(boardState, from, to);
        if(!path || path.length < 2){
          setStatus("Chemin physique bloqué pour cette pièce.", false);
          return;
        }
        if(path.length > MAX_WP){
          setStatus("Chemin trop long pour le déplacement physique.", false);
          return;
        }
        // Check for capture BEFORE applying move
        const destPidPath = getPieceAt(boardState, to.f, to.r);
        if(destPidPath){
          const destSide = pieceColor(destPidPath) === 'w' ? 'L' : 'R';
          addToCapturedZone(destPidPath, destSide);
        }
        send({cmd:"pathMove", path});
      }
    } else {
      // Check for capture BEFORE applying move
      const destPid = getPieceAt(boardState, to.f, to.r);
      if(destPid){
        const destSide = pieceColor(destPid) === 'w' ? 'L' : 'R';
        addToCapturedZone(destPid, destSide);
      }
      send({cmd:"squareMoveDirect", ff:from.f, fr:from.r, tf:to.f, tr:to.r});
    }

    boardState = res.nextBoard;

    if(res.rightsUpdate){
      castling = Object.assign({}, castling, res.rightsUpdate);
    }
    epSquare = res.epNext ? res.epNext : null;

    from = null; to = null; selectedPieceId = null;
    fromTxt.textContent = "--";
    toTxt.textContent = "--";

    turn = (turn === "w") ? "b" : "w";
    updateTurnOverlay();

    updateCheckState();
    if(checkSquare){
      setStatus("ÉCHEC ! Tour: " + (turn==="w" ? "Blancs" : "Noirs"), false);
    } else if(res.givesCheck){
      setStatus("ÉCHEC ! Tour: " + (turn==="w" ? "Blancs" : "Noirs"), false);
    } else {
      setStatus("OK. Tour: " + (turn==="w" ? "Blancs" : "Noirs"), true);
    }

    renderBoard();
    saveUiState();
  }

  document.getElementById("swapMove").addEventListener("click", ()=>{
    const tmp = from; from = to; to = tmp;
    fromTxt.textContent = labelOf(from);
    toTxt.textContent = labelOf(to);
    renderBoard();
  });

  function connect(){
    ws = new WebSocket(wsUrl);
    ws.onopen = () => setStatus("Connecté ✅");
    ws.onclose = () => { setStatus("Déconnecté… reconnexion", false); setTimeout(connect, 800); };
    ws.onerror = () => setStatus("Erreur WebSocket", false);

    ws.onmessage = (ev) => {
      try {
        const d = JSON.parse(ev.data);
        if (typeof d.pct !== "undefined") {
          const pct = Math.round(d.pct);
          pctEl.textContent = pct;
          fillEl.style.width = pct + "%";
          vEl.textContent = (d.v).toFixed(3);
          iEl.textContent = (d.i).toFixed(3);
        }
        if (typeof d.x !== "undefined") xEl.textContent = d.x;
        if (typeof d.y !== "undefined") yEl.textContent = d.y;
        if (typeof d.sp !== "undefined") {
          if (d.sp === 0) setActive(bSlow);
          if (d.sp === 1) setActive(bNorm);
          if (d.sp === 2) setActive(bFast);
        }
        if (typeof d.mag !== "undefined") {
          magnetOn = !!d.mag;
          updateMagnetUI(magnetOn);
        }
        if (typeof d.hallY !== "undefined") updateHallUI(hallYDot, hallYText, !!d.hallY);
        if (typeof d.hallX !== "undefined") updateHallUI(hallXDot, hallXText, !!d.hallX);
        if (typeof d.calib !== "undefined") updateCalibUI(!!d.calib);
        if (typeof d.busy  !== "undefined") fwBusy    = !!d.busy;
        if (typeof d.pending !== "undefined") fwPending = (d.pending|0);
        // AutoTune telemetry
        if (typeof d.tuning !== "undefined") {
          const nowTuning = !!d.tuning;
          if (nowTuning && !tuneBusy) tuneStartMs = Date.now();  // capture start time
          if (!nowTuning) tuneStartMs = null;
          updateTuneUI(nowTuning,
                       typeof d.tunePhase === "number" ? d.tunePhase : 0,
                       typeof d.tunePct   === "number" ? d.tunePct   : 0);
          updateLiveTuneCard(
            nowTuning,
            typeof d.liveTuneSpd     === "number" ? d.liveTuneSpd     : 0,
            typeof d.liveTuneSpdDiag === "number" ? d.liveTuneSpdDiag : 0,
            typeof d.liveTuneCurr    === "number" ? d.liveTuneCurr    : 0
          );
        }
        // Saved tune settings card
        if (typeof d.tuneValid !== "undefined") {
          updateTuneResultCard(
            !!d.tuneValid,
            typeof d.tuneSpd     === "number" ? d.tuneSpd     : 0,
            typeof d.tuneSpdDiag === "number" ? d.tuneSpdDiag : 0,
            typeof d.tuneCurr    === "number" ? d.tuneCurr    : 0
          );
        }
        // Tune log messages arrive as a separate frame type
        if (d.type === "tuneLog" && typeof d.msg === "string") {
          appendTuneLog(d.msg);
        }
        // Chess Test Run telemetry
        if (typeof d.testRun !== "undefined") {
          updateTestRunUI(
            !!d.testRun,
            typeof d.trStep  === "string" ? d.trStep  : "—",
            typeof d.trIdx   === "number" ? d.trIdx   : 0,
            typeof d.trTotal === "number" ? d.trTotal : 0
          );
        }
        runPhysicalResetPump();
      } catch(e) {}
    };
  }

  bSlow.addEventListener("click", ()=>{ setActive(bSlow); send({cmd:"speed", sp:0}); });
  bNorm.addEventListener("click", ()=>{ setActive(bNorm); send({cmd:"speed", sp:1}); });
  bFast.addEventListener("click", ()=>{ setActive(bFast); send({cmd:"speed", sp:2}); });

  bindHold("up","up");
  bindHold("down","down");
  bindHold("left","left");
  bindHold("right","right");

  document.getElementById("stop").addEventListener("click", ()=>stopHold());
  document.getElementById("center").addEventListener("click", ()=>{ stopHold(); send({cmd:"move", dir:"center"}); });

  calibBtn.addEventListener("click", ()=>{
    stopHold();
    send({cmd:"calibAll"});
  });

  tuneBtn.addEventListener("click", ()=>{
    stopHold();
    if(tuneBusy){ return; }
    tuneLogEl.textContent = "";
    send({cmd:"start_tuning"});
  });

  tuneStopBtn.addEventListener("click", ()=>{
    send({cmd:"stop"});
  });

  magnetBtn.addEventListener("click", ()=>{
    magnetOn = !magnetOn;
    send({cmd:"magnet", on: magnetOn});
    updateMagnetUI(magnetOn);
  });

  diag4Btn.addEventListener("click", ()=>{
    stopHold();
    send({cmd:"diag4"});
    setStatus("Test 1-2-3-4 envoyé", true);
  });

  testRunBtn.addEventListener("click", ()=>{
    stopHold();
    if(testRunBusy || tuneBusy) return;
    send({cmd:"start_test_run"});
    setStatus("Test Run démarré…", true);
  });

  testRunStopBtn.addEventListener("click", ()=>{
    send({cmd:"stop"});
  });

  document.getElementById("resetBoard").addEventListener("click", ()=>{
    startPhysicalReset();
  });

  document.getElementById("hardResetUi").addEventListener("click", ()=>{
    if(confirm("Êtes-vous sûr de vouloir faire un Hard Reset? Cela reinitalisera tout.")){
      try { localStorage.removeItem(UI_STATE_KEY); } catch(e) {}
      location.reload();
    }
  });

  updateHallUI(hallYDot, hallYText, false);
  updateHallUI(hallXDot, hallXText, false);
  updateCalibUI(false);
  fromTxt.textContent = "--";
  toTxt.textContent = "--";
  if(!loadUiState()) resetStandardPosition();
  updateCheckState();
  setStatus('Tour: ' + (turn === "w" ? "Blancs" : "Noirs"), true);
  updateTurnOverlay();
  renderBoard();

  document.getElementById("wifiSaveBtn").addEventListener("click", () => {
    const ssid = document.getElementById("wifiSsid").value.trim();
    const pass = document.getElementById("wifiPass").value;
    const msg  = document.getElementById("wifiMsg");
    if (!ssid) { msg.textContent = "⚠️ Entre le nom de ton réseau WiFi."; msg.style.color="#ff6b6b"; return; }
    msg.textContent = "⏳ Sauvegarde en cours, l'ESP32 va redémarrer…";
    msg.style.color = "#1dd1a1";
    document.getElementById("wifiSaveBtn").disabled = true;
    send({cmd:"wifiConfig", ssid, pass});
  });

  connect();
</script>
</body>
</html>
)HTML";

static void handleRoot() { server.send(200, "text/html", PAGE_INDEX); }

void webInit() {
  server.on("/", handleRoot);
  server.begin();

  webSocket.begin();
  webSocket.onEvent(onWsEvent);
}

void webLoop() {
  server.handleClient();
  webSocket.loop();
}

void webPushTelemetry() {
  float v, i, pct;
  long xAbs, yAbs;
  long cxAbs, cyAbs;
  uint8_t sp;
  bool magOn, hallY, hallX, calib;
  bool busy;
  uint8_t pending;

  portENTER_CRITICAL(&gMux);
  v     = g_battV;
  i     = g_battI;
  pct   = g_battPct;
  xAbs  = g_xAbs;
  yAbs  = g_yAbs;
  cxAbs = g_xCenterTarget;
  cyAbs = g_yCenterTarget;
  sp    = (uint8_t)g_speedMode;
  magOn = g_magnetOn;
  hallY = g_hallYDetected;
  hallX = g_hallXDetected;
  calib = (g_calibState != CALIB_IDLE);
  portEXIT_CRITICAL(&gMux);

  // AutoTune state snapshot.
  bool         tuning;
  uint8_t      tunePh;
  int          tunePct;
  uint8_t      sysState;
  TuneSettings tuneSnapshot;
  float        liveTuneSpd;
  float        liveTuneSpdDiag;
  float        liveTuneAcc;
  float        liveTuneDecel;
  uint16_t     liveTuneCurr;

  // ChessTestRun snapshot
  bool    testRunActive;
  char    trStepSnap[64];
  uint8_t trIdxSnap;
  uint8_t trTotalSnap;

  portENTER_CRITICAL(&gMux);
  tuning         = g_tuneActive;
  tunePh         = (uint8_t)g_tunePhase;
  tunePct        = g_tuneProgress;
  sysState       = (uint8_t)g_systemState;
  tuneSnapshot   = g_tuneSettings;
  liveTuneSpd    = g_tuneLiveAxisSpeed;
  liveTuneSpdDiag= g_tuneLiveDiagSpeed;
  liveTuneAcc    = g_tuneLiveAccel;
  liveTuneDecel  = g_tuneLiveDecel;
  liveTuneCurr   = g_tuneCurrentMa;
  testRunActive  = g_testRunActive;
  strncpy(trStepSnap, (const char*)g_trStepName, 63); trStepSnap[63] = '\0';
  trIdxSnap      = g_trStepIdx;
  trTotalSnap    = g_trStepTotal;
  portEXIT_CRITICAL(&gMux);

  bool     tuneValid    = tuneSnapshot.tuningValid;
  float    tuneSpd      = tuneSnapshot.safeSpeed;
  float    tuneSpdDiag  = tuneSnapshot.safeSpeedDiag;
  float    tuneAcc      = tuneSnapshot.safeAccel;
  float    tuneDecel    = tuneSnapshot.safeDecel;
  uint16_t tuneCurr     = tuneSnapshot.motorCurrent;

  long xDisp = xAbs - cxAbs;
  long yDisp = yAbs - cyAbs;
  busy    = commandsIsBusy();
  pending = commandsPendingCount();

  char msg[800];
  snprintf(msg, sizeof(msg),
           "{\"pct\":%.2f,\"v\":%.3f,\"i\":%.3f,\"x\":%ld,\"y\":%ld,\"sp\":%u,"
           "\"mag\":%s,\"hallY\":%s,\"hallX\":%s,\"calib\":%s,"
           "\"busy\":%s,\"pending\":%u,"
           "\"sysState\":%u,\"tuning\":%s,\"tunePhase\":%u,\"tunePct\":%d,"
           "\"liveTuneSpd\":%.0f,\"liveTuneSpdDiag\":%.0f,\"liveTuneAcc\":%.0f,\"liveTuneDecel\":%.0f,\"liveTuneCurr\":%u,"
           "\"tuneValid\":%s,\"tuneSpd\":%.0f,\"tuneSpdDiag\":%.0f,\"tuneAcc\":%.0f,\"tuneDecel\":%.0f,\"tuneCurr\":%u,"
           "\"testRun\":%s,\"trStep\":\"%s\",\"trIdx\":%u,\"trTotal\":%u}",
           pct, v, i, xDisp, yDisp, (unsigned)sp,
           (magOn     ? "true" : "false"),
           (hallY     ? "true" : "false"),
           (hallX     ? "true" : "false"),
           (calib     ? "true" : "false"),
           (busy      ? "true" : "false"),
           (unsigned)pending,
           (unsigned)sysState,
           (tuning    ? "true" : "false"),
           (unsigned)tunePh,
           tunePct,
           liveTuneSpd, liveTuneSpdDiag, liveTuneAcc, liveTuneDecel, (unsigned)liveTuneCurr,
           (tuneValid ? "true" : "false"),
           tuneSpd, tuneSpdDiag, tuneAcc, tuneDecel,
           (unsigned)tuneCurr,
           (testRunActive ? "true" : "false"),
           trStepSnap,
           (unsigned)trIdxSnap,
           (unsigned)trTotalSnap);

  webSocket.broadcastTXT(msg);
}
