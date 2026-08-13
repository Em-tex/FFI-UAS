/* js/simulator-flightlog.js
   Enkel, alltid-på flightlogg for feilsøking (høyde, fart, modus, Loiter-fase/holdepunkt, pinneinput, ...) -
   samme mønster/motivasjon som VTOL-simmens egen js/simulator-vtol-flightlog.js (brukeren ba opprinnelig om
   DEN som en varig funksjon for fremtidig debugging, ikke bare én krasj-serie). Lagt til her spesifikt for å
   kunne feilsøke Loiter-tuning ut fra TALL i stedet for verbale beskrivelser ("wobbler", "flyter") - loggen
   har derfor egne Loiter-felt (fase, avstand til holdepunkt, filtrert fart, kommandert lenevinkel) i tillegg
   til de generelle feltene. Siste 30 sekunder holdes i en ring-buffer, samplet med et FAST intervall (IKKE
   hver fysikk-tick - 120Hz i 30 sek ville vært 3600 rader) - se FLIGHTLOG_SAMPLE_INTERVAL_S.

   Lastes ETTER simulator.js (samme globale scope-mønster som simulator-vtol-flightlog.js sin egen topp-
   kommentar beskriver) - leser droneState/inputState/MODE_LABELS/currentWindVector direkte som globale
   identifikatorer, og logFlightSample(dt) kalles fra stepPhysics i simulator.js (helt til slutt i
   funksjonen, se der) - samme "forover-referanse, løses ved kall-tidspunkt, ikke definisjons-tidspunkt"-
   mønster som VTOL-simmen sin rtlState/updateRtlAutopilot allerede er referert fra samme sted. */

const FLIGHTLOG_SAMPLE_INTERVAL_S = 0.15;
const FLIGHTLOG_DURATION_S = 30;
const FLIGHTLOG_MAX_SAMPLES = Math.ceil(FLIGHTLOG_DURATION_S / FLIGHTLOG_SAMPLE_INTERVAL_S);

const flightLogState = { buffer: [], sampleTimer: 0 };

// Kalt fra stepPhysics hver fysikk-tick (120Hz) - selve NEDSAMPLINGEN til FLIGHTLOG_SAMPLE_INTERVAL_S
// skjer her via en enkel akkumulator, samme idiom som FIXED_DT-akkumulatoren i animate() selv.
function logFlightSample(dt) {
    // Fryser loggen ved krasj i stedet for å fortsette å skrive over krasjøyeblikket med "ligger i ro på
    // bakken etterpå"-samples - akkurat krasj-sekundene er det mest interessante for feilsøking. Tines opp
    // igjen automatisk idet droneState.crashed blir false igjen (resetDrone), og gamle samples fra FØR
    // krasjen ruller naturlig ut av ring-bufferet etter 30 nye sekunder uten at noe eksplisitt trengs her.
    if (droneState.crashed) return;
    flightLogState.sampleTimer += dt;
    if (flightLogState.sampleTimer < FLIGHTLOG_SAMPLE_INTERVAL_S) return;
    flightLogState.sampleTimer = 0;

    const euler = new THREE.Euler().setFromQuaternion(droneState.quaternion, "YXZ");
    const pitchDeg = -THREE.MathUtils.radToDeg(euler.x);
    const bankDeg = -THREE.MathUtils.radToDeg(euler.z);
    const groundSpeedMs = Math.hypot(droneState.velocity.x, droneState.velocity.z);
    // Avstand til Loiter sitt holdepunkt (horisontal) - KUN meningsfull i Loiter (droneState.loiterPhase
    // er ellers "-", se droneState-kommentaren i simulator.js); -1 ellers.
    const loiterTargetDistM = droneState.flightMode === "loiter"
        ? Math.hypot(droneState.position.x - droneState.loiterTargetPos.x, droneState.position.z - droneState.loiterTargetPos.z)
        : -1;

    flightLogState.buffer.push({
        t: performance.now() / 1000,
        mode: droneState.flightMode,
        loiterPhase: droneState.loiterPhase,
        altM: Math.max(0, droneState.position.y),
        groundSpeedMs: groundSpeedMs,
        vSpeedMs: droneState.velocity.y,
        bankDeg: bankDeg,
        pitchDeg: pitchDeg,
        cmdBankDeg: droneState.loiterCmdRollAngle === null ? 0 : droneState.loiterCmdRollAngle,
        cmdPitchDeg: droneState.loiterCmdPitchAngle === null ? 0 : droneState.loiterCmdPitchAngle,
        loiterTargetDistM: loiterTargetDistM,
        windMs: currentWindVector.length(),
        stickP: inputState.stick.pitch,
        stickR: inputState.stick.roll,
        stickY: inputState.stick.yaw,
        stickT: inputState.stick.throttle,
        armed: droneState.armed,
        crashed: droneState.crashed
    });
    if (flightLogState.buffer.length > FLIGHTLOG_MAX_SAMPLES) flightLogState.buffer.shift();
}

// Tab-separert (limes rett inn i et regneark om ønskelig) - t er sekunder RELATIVT til bufferets første
// rad (ikke performance.now() sin absolutte, meningsløse epoke), så en limt-inn logg alltid starter på 0.00.
function flightLogToText() {
    if (flightLogState.buffer.length === 0) return "(ingen data ennå - fly litt først)";
    const t0 = flightLogState.buffer[0].t;
    const header = "t(s)\tmodus\tloiter-fase\thøyde(m)\tbakkefart(m/s)\tvfart(m/s)\tbank(deg)\tpitch(deg)\t"
        + "kmd-bank(deg)\tkmd-pitch(deg)\tavstand-holdepkt(m)\tvind(m/s)\tpinneP\tpinneR\tpinneY\tpinneT\tarmed\tkrasj";
    const rows = flightLogState.buffer.map(function (s) {
        return [
            (s.t - t0).toFixed(2), s.mode, s.loiterPhase, s.altM.toFixed(1), s.groundSpeedMs.toFixed(2),
            s.vSpeedMs.toFixed(2), s.bankDeg.toFixed(1), s.pitchDeg.toFixed(1),
            s.cmdBankDeg.toFixed(1), s.cmdPitchDeg.toFixed(1),
            s.loiterTargetDistM < 0 ? "-" : s.loiterTargetDistM.toFixed(2), s.windMs.toFixed(1),
            s.stickP.toFixed(2), s.stickR.toFixed(2), s.stickY.toFixed(2), s.stickT.toFixed(2),
            s.armed ? "1" : "0", s.crashed ? "1" : "0"
        ].join("\t");
    });
    return header + "\n" + rows.join("\n");
}

function resetFlightLog() {
    flightLogState.buffer.length = 0;
    flightLogState.sampleTimer = 0;
}

/* ---------- Panel (samme mønster som initRtlPanel/initFlightLogPanel i VTOL-simmen) ---------- */
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
