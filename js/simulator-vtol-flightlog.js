/* js/simulator-vtol-flightlog.js
   Enkel, alltid-på flightlogg for feilsøking (høyde, fart, modus, RTL-fase, pinneinput, ...) - brukeren ba
   eksplisitt om dette som en VARIG funksjon for fremtidig debugging, ikke bare den ene krasj-serien som
   utløste det: "legg til en flylogg... kan være en fast funksjon for fremtidig debugging også." Siste 30
   sekunder holdes i en ring-buffer, samplet med et FAST intervall (IKKE hver fysikk-tick - 120Hz i 30 sek
   ville vært 3600 rader, langt mer enn "vil ikke ha for mye data der heller" tilsier) - se
   FLIGHTLOG_SAMPLE_INTERVAL_S.

   Lastes ETTER både simulator-vtol.js OG simulator-vtol-rtl.js (samme globale scope-mønster som
   simulator-vtol-rtl.js sin egen toppkommentar beskriver) - leser planeState/inputState/rtlState/
   lastAirspeed/MODE_LABELS direkte som globale identifikatorer, og logFlightSample(dt) kalles fra
   stepPhysics i simulator-vtol.js (helt til slutt i funksjonen, se der) - samme "forover-referanse,
   løses ved kall-tidspunkt, ikke definisjons-tidspunkt"-mønster som rtlState/updateRtlAutopilot allerede
   er referert fra samme sted. */

const FLIGHTLOG_SAMPLE_INTERVAL_S = 0.15;
const FLIGHTLOG_DURATION_S = 30;
const FLIGHTLOG_MAX_SAMPLES = Math.ceil(FLIGHTLOG_DURATION_S / FLIGHTLOG_SAMPLE_INTERVAL_S);

const flightLogState = { buffer: [], sampleTimer: 0 };

// Kalt fra stepPhysics hver fysikk-tick (120Hz) - selve NEDSAMPLINGEN til FLIGHTLOG_SAMPLE_INTERVAL_S
// skjer her via en enkel akkumulator, samme idiom som FIXED_DT-akkumulatoren i animate() selv.
function logFlightSample(dt) {
    // Fryser loggen ved krasj i stedet for å fortsette å skrive over krasjøyeblikket med "ligger i ro på
    // bakken etterpå"-samples - akkurat krasj-sekundene er det mest interessante for feilsøking. Tines opp
    // igjen automatisk idet planeState.crashed blir false igjen (resetPlane), og gamle samples fra FØR
    // krasjen ruller naturlig ut av ring-bufferet etter 30 nye sekunder uten at noe eksplisitt trengs her.
    if (planeState.crashed) return;
    flightLogState.sampleTimer += dt;
    if (flightLogState.sampleTimer < FLIGHTLOG_SAMPLE_INTERVAL_S) return;
    flightLogState.sampleTimer = 0;

    const euler = new THREE.Euler().setFromQuaternion(planeState.quaternion, "YXZ");
    const pitchDeg = -THREE.MathUtils.radToDeg(euler.x);
    const bankDeg = -THREE.MathUtils.radToDeg(euler.z);
    const rtlPhase = planeState.flightMode === "qrtl" ? rtlState.phase : "-";
    // Avstand til hjem (brukeren spurte eksplisitt om dette) - KUN meningsfull når hjem faktisk er satt
    // (rtlState.homeSet, se captureHome i simulator-vtol-rtl.js); -1 ellers. Horisontal avstand (samme
    // definisjon som horizDist i updateRtlAutopilot), IKKE 3D-avstand - det ER avstanden fase-/
    // overgangslogikken faktisk styrer etter.
    const homeDistM = rtlState.homeSet
        ? Math.hypot(planeState.position.x - rtlState.home.x, planeState.position.z - rtlState.home.z)
        : -1;

    flightLogState.buffer.push({
        t: performance.now() / 1000,
        mode: planeState.flightMode,
        phase: rtlPhase,
        altM: Math.max(0, planeState.position.y),
        airspeedMs: lastAirspeed,
        vSpeedMs: planeState.velocity.y,
        bankDeg: bankDeg,
        pitchDeg: pitchDeg,
        mcAuthPct: planeState.lastMcAuthority * 100,
        homeDistM: homeDistM,
        stickP: inputState.stick.pitch,
        stickR: inputState.stick.roll,
        stickY: inputState.stick.yaw,
        stickT: inputState.stick.throttle,
        onGround: planeState.onGround,
        crashed: planeState.crashed
    });
    if (flightLogState.buffer.length > FLIGHTLOG_MAX_SAMPLES) flightLogState.buffer.shift();
}

// Tab-separert (limes rett inn i et regneark om ønskelig) - t er sekunder RELATIVT til bufferets første
// rad (ikke performance.now() sin absolutte, meningsløse epoke), så en limt-inn logg alltid starter på 0.00.
function flightLogToText() {
    if (flightLogState.buffer.length === 0) return "(ingen data ennå - fly litt først)";
    const t0 = flightLogState.buffer[0].t;
    const header = "t(s)\tmodus\tfase\thøyde(m)\tluftfart(m/s)\tvfart(m/s)\tbank(deg)\tpitch(deg)\tmcAuth(%)\tavstand-hjem(m)\tpinneP\tpinneR\tpinneY\tpinneT\tbakke\tkrasj";
    const rows = flightLogState.buffer.map(function (s) {
        return [
            (s.t - t0).toFixed(2), s.mode, s.phase, s.altM.toFixed(1), s.airspeedMs.toFixed(1),
            s.vSpeedMs.toFixed(2), s.bankDeg.toFixed(1), s.pitchDeg.toFixed(1), s.mcAuthPct.toFixed(0),
            s.homeDistM < 0 ? "-" : s.homeDistM.toFixed(0),
            s.stickP.toFixed(2), s.stickR.toFixed(2), s.stickY.toFixed(2), s.stickT.toFixed(2),
            s.onGround ? "1" : "0", s.crashed ? "1" : "0"
        ].join("\t");
    });
    return header + "\n" + rows.join("\n");
}

function resetFlightLog() {
    flightLogState.buffer.length = 0;
    flightLogState.sampleTimer = 0;
}

/* ---------- Panel (samme mønster som initRtlPanel i simulator-vtol-rtl.js) ---------- */
function initFlightLogPanel() {
    const textEl = document.getElementById("flightLogText");
    const refreshBtn = document.getElementById("flightLogRefreshBtn");
    const copyBtn = document.getElementById("flightLogCopyBtn");
    const clearBtn = document.getElementById("flightLogClearBtn");
    const statusEl = document.getElementById("flightLogStatus");

    function refresh() { textEl.value = flightLogToText(); }
    function showStatus(msg) {
        if (!statusEl) return;
        statusEl.textContent = msg;
        setTimeout(function () { statusEl.textContent = ""; }, 2500);
    }

    refresh();
    refreshBtn.addEventListener("click", refresh);
    clearBtn.addEventListener("click", function () { resetFlightLog(); refresh(); });
    copyBtn.addEventListener("click", function () {
        refresh();
        const text = textEl.value;
        function fallbackCopy() {
            // navigator.clipboard krever HTTPS/localhost i mange nettlesere - execCommand("copy") via en
            // markert <textarea> er en trygg reserveløsning som funker overalt uten den begrensningen.
            textEl.focus();
            textEl.select();
            try {
                document.execCommand("copy");
                showStatus("Kopiert");
            } catch (e) {
                showStatus("Kunne ikke kopiere automatisk - marker teksten og Ctrl+C manuelt");
            }
        }
        if (navigator.clipboard && navigator.clipboard.writeText) {
            navigator.clipboard.writeText(text).then(function () { showStatus("Kopiert til utklippstavle"); }, fallbackCopy);
        } else {
            fallbackCopy();
        }
    });
}
