/* js/simulator-common.js
   Delt kode for FFI UAS-simulatorene (quadcopter + fixed-wing): matte/kontroll-hjelpere,
   lagring, rate-kurve-UI, fjernkontroll(gamepad)-rammeverk, scene-elementer (himmel/bakke/
   landingsplass/vindpølse/tre), vind, FPV-HUD-tegning og panel/dropdown-UI. Eksponeres som
   window.Sim - hver simulator-side har sin egen tilstand (dronestate/planestate) og kaller
   inn i disse generiske funksjonene. */
(function (global) {
    "use strict";

    const STICK_RAMP_TIME = 0.22; // sekunder til full utslag (tastatur)
    const MIN_GAMEPAD_CHANNELS = 8; // RC-sendere har typisk minst 8 kanaler i USB-joystick-modus

    /* ---------- Matte/kontroll-hjelpere ---------- */
    function clamp(v, min, max) { return Math.min(max, Math.max(min, v)); }

    function rampStick(current, target, dt, rampTime) {
        const maxDelta = dt / (rampTime || STICK_RAMP_TIME);
        if (Math.abs(target - current) <= maxDelta) return target;
        return current + Math.sign(target - current) * maxDelta;
    }

    // Betaflight "Actual Rates"-tilnærming: stick i [-1,1] -> vinkelhastighet i grader/s
    function computeRate(stick, axisRates) {
        const expo = axisRates.expo;
        const stickWithExpo = stick * (1 - expo) + expo * Math.pow(stick, 3);
        const cs = axisRates.centerSensitivity;
        const mr = axisRates.maxRate;
        return cs * stickWithExpo + (mr - cs) * Math.pow(Math.abs(stickWithExpo), 3) * Math.sign(stickWithExpo);
    }

    // Enkel throttle-kurve (samme kubiske expo-formel som rate-kurvene, men på gass 0..1 i stedet for grader/s).
    function computeThrottleCurve(stick01, expo) {
        const centered = stick01 * 2 - 1;
        const shaped = centered * (1 - expo) + expo * Math.pow(centered, 3);
        return clamp((shaped + 1) / 2, 0, 1);
    }

    // Integrerer orientering fra body-frame vinkelhastighet (rad/s): qdot = 0.5 * q (x) omega
    function integrateOrientation(q, angVelVec3, dt) {
        const omegaQuat = new THREE.Quaternion(angVelVec3.x, angVelVec3.y, angVelVec3.z, 0);
        const qDot = q.clone().multiply(omegaQuat);
        q.x += qDot.x * 0.5 * dt;
        q.y += qDot.y * 0.5 * dt;
        q.z += qDot.z * 0.5 * dt;
        q.w += qDot.w * 0.5 * dt;
        q.normalize();
    }

    /* ---------- Lagring (localStorage) ---------- */
    // Én nivå dyp merge: for nøkler der default-verdien er et objekt slås lagrede felter sammen inn i
    // en klone av defaults (samme mønster som de tidligere separate loadRates/loadGamepadMap/loadSettings).
    function loadJSON(key, defaults) {
        const result = JSON.parse(JSON.stringify(defaults));
        try {
            const raw = localStorage.getItem(key);
            if (raw) {
                const parsed = JSON.parse(raw);
                Object.keys(defaults).forEach(function (k) {
                    if (parsed[k] === undefined) return;
                    const def = defaults[k];
                    if (def && typeof def === "object" && !Array.isArray(def)) {
                        Object.assign(result[k], parsed[k]);
                    } else {
                        result[k] = parsed[k];
                    }
                });
            }
        } catch (e) {}
        return result;
    }

    function saveJSON(key, obj) {
        localStorage.setItem(key, JSON.stringify(obj));
    }

    /* ---------- Rate-kurve: visualisering + drakontroll ---------- */
    const RATE_CURVE_W = 260;
    const RATE_CURVE_H = 130;
    const RATE_CURVE_CENTER_STICK = 0.2; // referansepunkt for "center sensitivity"-håndtaket

    function rateCurveScale(maxRate) {
        return Math.max(50, maxRate * 1.15);
    }
    function stickToCanvasX(stick) { return (stick + 1) / 2 * RATE_CURVE_W; }
    function canvasYToRate(y, scale) { return clamp(-(y - RATE_CURVE_H / 2) / (RATE_CURVE_H / 2) * scale, -scale, scale); }
    function rateToCanvasY(rate, scale) { return RATE_CURVE_H / 2 - (rate / scale) * (RATE_CURVE_H / 2); }

    function drawRateCurve(ctx, axisRates) {
        const scale = rateCurveScale(axisRates.maxRate);
        ctx.clearRect(0, 0, RATE_CURVE_W, RATE_CURVE_H);
        ctx.strokeStyle = "#e0e0e0";
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(RATE_CURVE_W / 2, 0);
        ctx.lineTo(RATE_CURVE_W / 2, RATE_CURVE_H);
        ctx.moveTo(0, RATE_CURVE_H / 2);
        ctx.lineTo(RATE_CURVE_W, RATE_CURVE_H / 2);
        ctx.stroke();

        ctx.strokeStyle = "#03477F";
        ctx.lineWidth = 2;
        ctx.beginPath();
        for (let i = 0; i <= 40; i++) {
            const stick = -1 + (i / 40) * 2;
            const x = stickToCanvasX(stick);
            const y = rateToCanvasY(computeRate(stick, axisRates), scale);
            if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        }
        ctx.stroke();

        const maxX = stickToCanvasX(1);
        const maxY = rateToCanvasY(axisRates.maxRate, scale);
        ctx.fillStyle = "#03477F";
        ctx.beginPath();
        ctx.arc(maxX, maxY, 5, 0, Math.PI * 2);
        ctx.fill();

        const csRate = computeRate(RATE_CURVE_CENTER_STICK, axisRates);
        const csX = stickToCanvasX(RATE_CURVE_CENTER_STICK);
        const csY = rateToCanvasY(csRate, scale);
        ctx.fillStyle = "#c0392b";
        ctx.beginPath();
        ctx.arc(csX, csY, 5, 0, Math.PI * 2);
        ctx.fill();
    }

    // Løser centerSensitivity slik at kurven treffer et gitt punkt ved RATE_CURVE_CENTER_STICK.
    function solveCenterSensitivityForRate(targetRate, axisRates) {
        const expo = axisRates.expo;
        const s = RATE_CURVE_CENTER_STICK;
        const swe = s * (1 - expo) + expo * Math.pow(s, 3);
        const swe3 = Math.pow(Math.abs(swe), 3) * Math.sign(swe);
        const denom = swe - swe3;
        if (Math.abs(denom) < 1e-6) return axisRates.centerSensitivity;
        return clamp((targetRate - axisRates.maxRate * swe3) / denom, 0, 300);
    }

    // Bygger en komplett rate-akse-boks (tittel + center/max/expo-slidere + draggbar kurve) for én akse
    // (roll/pitch/yaw for quad, aileron/elevator/rudder for fixed-wing). Kaller onChange() etter hver endring.
    function buildRateAxisBox(axisRates, axisLabel, onChange) {
        const box = document.createElement("div");
        box.className = "sim-rate-axis";
        const title = document.createElement("div");
        title.className = "sim-rate-axis-title";
        title.textContent = axisLabel;
        box.appendChild(title);

        const inputs = {};
        const spans = {};
        [
            { key: "centerSensitivity", label: "Center sens.", min: 0, max: 300, step: 5 },
            { key: "maxRate", label: "Max rate", min: 100, max: 1200, step: 10 },
            { key: "expo", label: "Expo", min: 0, max: 1, step: 0.05 }
        ].forEach(function (param) {
            const row = document.createElement("div");
            row.className = "sim-rate-row";
            const label = document.createElement("label");
            label.textContent = param.label;
            const input = document.createElement("input");
            input.type = "range";
            input.min = param.min;
            input.max = param.max;
            input.step = param.step;
            input.value = axisRates[param.key];
            const valueSpan = document.createElement("span");
            valueSpan.className = "sim-rate-value";
            valueSpan.textContent = axisRates[param.key];
            input.addEventListener("input", function () {
                axisRates[param.key] = parseFloat(input.value);
                valueSpan.textContent = input.value;
                if (onChange) onChange();
                redraw();
            });
            row.appendChild(label);
            row.appendChild(input);
            row.appendChild(valueSpan);
            box.appendChild(row);
            inputs[param.key] = input;
            spans[param.key] = valueSpan;
        });

        const canvas = document.createElement("canvas");
        canvas.width = RATE_CURVE_W;
        canvas.height = RATE_CURVE_H;
        canvas.className = "sim-rate-curve";
        box.appendChild(canvas);
        const hint = document.createElement("p");
        hint.className = "sim-panel-hint";
        hint.style.margin = "6px 0 0 0";
        hint.textContent = "Dra blått punkt (max rate) eller rødt punkt (center sensitivity) direkte på kurven.";
        box.appendChild(hint);

        const ctx = canvas.getContext("2d");
        function redraw() { drawRateCurve(ctx, axisRates); }

        let dragging = null;
        canvas.addEventListener("pointerdown", function (e) {
            const rect = canvas.getBoundingClientRect();
            const px = (e.clientX - rect.left) * (RATE_CURVE_W / rect.width);
            const py = (e.clientY - rect.top) * (RATE_CURVE_H / rect.height);
            const scale = rateCurveScale(axisRates.maxRate);
            const maxHandle = { x: stickToCanvasX(1), y: rateToCanvasY(axisRates.maxRate, scale) };
            const csHandle = { x: stickToCanvasX(RATE_CURVE_CENTER_STICK), y: rateToCanvasY(computeRate(RATE_CURVE_CENTER_STICK, axisRates), scale) };
            const distMax = Math.hypot(px - maxHandle.x, py - maxHandle.y);
            const distCs = Math.hypot(px - csHandle.x, py - csHandle.y);
            if (distMax <= 10 || distCs <= 10) {
                dragging = distMax <= distCs ? "max" : "center";
                canvas.setPointerCapture(e.pointerId);
                e.preventDefault();
            }
        });
        canvas.addEventListener("pointermove", function (e) {
            if (!dragging) return;
            const rect = canvas.getBoundingClientRect();
            const py = (e.clientY - rect.top) * (RATE_CURVE_H / rect.height);
            const scale = rateCurveScale(axisRates.maxRate);
            const targetRate = canvasYToRate(py, scale);
            if (dragging === "max") {
                axisRates.maxRate = clamp(Math.round(targetRate / 10) * 10, 100, 1200);
                inputs.maxRate.value = axisRates.maxRate;
                spans.maxRate.textContent = axisRates.maxRate;
            } else {
                axisRates.centerSensitivity = Math.round(solveCenterSensitivityForRate(targetRate, axisRates) / 5) * 5;
                inputs.centerSensitivity.value = axisRates.centerSensitivity;
                spans.centerSensitivity.textContent = axisRates.centerSensitivity;
            }
            if (onChange) onChange();
            redraw();
        });
        function stopDrag() { dragging = null; }
        canvas.addEventListener("pointerup", stopDrag);
        canvas.addEventListener("pointercancel", stopDrag);

        redraw();
        return box;
    }

    // Tilsvarende boks for gass-expo (ingen kurve å dra i, kun én expo-slider).
    function buildThrottleExpoBox(throttleRates, title, hintText, onChange) {
        const box = document.createElement("div");
        box.className = "sim-rate-axis";
        const titleEl = document.createElement("div");
        titleEl.className = "sim-rate-axis-title";
        titleEl.textContent = title;
        box.appendChild(titleEl);

        const row = document.createElement("div");
        row.className = "sim-rate-row";
        const label = document.createElement("label");
        label.textContent = "Expo";
        const input = document.createElement("input");
        input.type = "range";
        input.min = 0;
        input.max = 1;
        input.step = 0.05;
        input.value = throttleRates.expo;
        const valueSpan = document.createElement("span");
        valueSpan.className = "sim-rate-value";
        valueSpan.textContent = throttleRates.expo;
        input.addEventListener("input", function () {
            throttleRates.expo = parseFloat(input.value);
            valueSpan.textContent = input.value;
            if (onChange) onChange();
        });
        row.appendChild(label);
        row.appendChild(input);
        row.appendChild(valueSpan);
        box.appendChild(row);

        const hint = document.createElement("p");
        hint.className = "sim-panel-hint";
        hint.style.margin = "6px 0 0 0";
        hint.textContent = hintText;
        box.appendChild(hint);

        return box;
    }

    /* ---------- Fjernkontroll (gamepad) ---------- */
    function rawFirstGamepad() {
        const pads = navigator.getGamepads ? navigator.getGamepads() : [];
        for (let i = 0; i < pads.length; i++) {
            if (pads[i]) return pads[i];
        }
        return null;
    }

    function getActiveGamepad(inputSource) {
        if (inputSource === "keyboard") return null;
        const pads = navigator.getGamepads ? navigator.getGamepads() : [];
        if (inputSource !== "auto") {
            const idx = parseInt(inputSource, 10);
            if (pads[idx]) return pads[idx];
        }
        return rawFirstGamepad();
    }

    // scale (default 1, se DEFAULT_GAMEPAD_MAP/createAxisCalibrationManager) - noen sendere rapporterer
    // ikke ±1.0 ved fysisk fullt utslag (f.eks. bare ±0.61), som uten denne ville gitt permanent
    // redusert kontrollmyndighet uansett hvor langt spaken faktisk føres. Ganges inn FØR reverse/clamp.
    function readStickAxis(gp, channelMap) {
        const raw = (gp.axes[channelMap.axis] || 0) * (channelMap.scale || 1);
        return clamp(raw * (channelMap.reverse ? -1 : 1), -1, 1);
    }

    function readThrottleAxis(gp, channelMap) {
        const raw = (gp.axes[channelMap.axis] || 0) * (channelMap.scale || 1);
        const signed = raw * (channelMap.reverse ? -1 : 1);
        return clamp((signed + 1) / 2, 0, 1);
    }

    // Full-utslag-kalibrering: fanger opp maks observert |akseverdi| PER KANAL over et kort tidsvindu
    // mens brukeren beveger alle spakene til ytterpunktene, og lagrer 1/maxObservert som skalering (se
    // readStickAxis/readThrottleAxis over) - IKKE en full min/max/senter-kalibrering (det hadde også
    // krevd å fange senterverdi/dødsone), bare den vanligste feilen (sendere som skalerer utslaget).
    function createAxisCalibrationManager(channelMap, channels, onDone, durationMs) {
        let active = false;
        let endAt = 0;
        let observed = {}; // akse-indeks -> maks |verdi| sett i vinduet
        function start() {
            active = true;
            endAt = performance.now() + (durationMs || 4000);
            observed = {};
        }
        function isActive() { return active; }
        function remainingMs() { return active ? Math.max(0, endAt - performance.now()) : 0; }
        function poll(gp) {
            if (!active || !gp) return;
            for (let i = 0; i < gp.axes.length; i++) {
                observed[i] = Math.max(observed[i] || 0, Math.abs(gp.axes[i]));
            }
            if (performance.now() >= endAt) {
                active = false;
                channels.forEach(function (ch) {
                    const maxSeen = observed[channelMap[ch].axis] || 0;
                    // Justerer kun hvis spaken faktisk ble beveget markert (>0.2, ellers vet vi ikke noe
                    // reelt om denne kanalen) OG ikke allerede er nesten helt ute (>0.97) - unngår både
                    // å dele på nesten-null (urimelig stor skalering) og å "forsterke" en sender som
                    // allerede er fint kalibrert (ren støy i input-lesingen ville da blitt skalert opp).
                    if (maxSeen > 0.2 && maxSeen < 0.97) channelMap[ch].scale = clamp(1 / maxSeen, 1, 3);
                });
                if (onDone) onDone();
            }
        }
        return { start: start, isActive: isActive, remainingMs: remainingMs, poll: poll };
    }

    // Sjekker om en lagret binding ({type:"button",index} eller {type:"axis",index,onValue,offValue})
    // er aktiv AKKURAT NÅ (holdt inne) - eksponert på Sim slik at kontinuerlige "hold inne"-handlinger
    // (f.eks. trim opp/ned) kan lese knappestatus direkte hver frame, uten å gå via
    // createButtonBindingManagers stigende-kant-varsling (som er ment for enkelt-trigge handlinger).
    function isBindingActive(gp, binding) {
        if (!binding) return false;
        if (binding.type === "axis") {
            const v = gp.axes[binding.index];
            if (v === undefined) return false;
            return Math.abs(v - binding.onValue) < Math.abs(v - binding.offValue);
        }
        const btn = gp.buttons[binding.index];
        if (!btn) return false;
        return btn.pressed || btn.value > 0.5;
    }

    // Fungerer med enhver sender i USB-joystick-modus - gimbaler som akser, brytere som knapper via
    // standard HTML5 Gamepad API. Bindinger lagres som { type:"button", index } eller
    // { type:"axis", index, onValue, offValue }. bindingsObj er f.eks. gamepadMap.buttons; actionsMap
    // er { actionNavn: fn } - fn kalles på stigende kant (bryter aktivert).
    function createButtonBindingManager(bindingsObj, actionsMap, onBindingChanged) {
        let listeningForAction = null;
        let learnIgnoreButtons = new Set();
        let learnAxisBaseline = [];
        const prevActive = {};

        function startListening(action, gp) {
            listeningForAction = action;
            learnIgnoreButtons = new Set();
            learnAxisBaseline = gp ? gp.axes.slice() : [];
            if (gp) {
                for (let i = 0; i < gp.buttons.length; i++) {
                    if (gp.buttons[i].pressed || gp.buttons[i].value > 0.5) learnIgnoreButtons.add(i);
                }
            }
        }

        function poll(gp) {
            if (!gp) return;
            if (listeningForAction) {
                let captured = false;
                for (let i = 0; i < gp.buttons.length; i++) {
                    const pressed = gp.buttons[i].pressed || gp.buttons[i].value > 0.5;
                    if (pressed && !learnIgnoreButtons.has(i)) {
                        bindingsObj[listeningForAction] = { type: "button", index: i };
                        captured = true;
                        break;
                    }
                }
                if (!captured) {
                    for (let i = 0; i < gp.axes.length; i++) {
                        const baseline = learnAxisBaseline[i] || 0;
                        if (Math.abs(gp.axes[i] - baseline) > 0.25) {
                            bindingsObj[listeningForAction] = { type: "axis", index: i, onValue: gp.axes[i], offValue: baseline };
                            captured = true;
                            break;
                        }
                    }
                }
                if (captured) {
                    if (onBindingChanged) onBindingChanged();
                    listeningForAction = null;
                }
            }

            Object.keys(actionsMap).forEach(function (action) {
                const active = isBindingActive(gp, bindingsObj[action]);
                if (active && !prevActive[action]) actionsMap[action]();
                prevActive[action] = active;
            });
        }

        return {
            startListening: startListening,
            poll: poll,
            isListening: function () { return listeningForAction; }
        };
    }

    // Fyller <select> med Automatisk/Tastatur/tilkoblede gamepads. Returnerer den faktiske valgte
    // kilden (kan avvike fra currentSource hvis den lagrede enheten ikke lenger er tilkoblet -
    // kalleren bør da lagre den returnerte verdien som ny innstilling).
    function populateInputSourceSelect(selectEl, currentSource) {
        const pads = navigator.getGamepads ? navigator.getGamepads() : [];
        selectEl.innerHTML = "";

        const autoOpt = document.createElement("option");
        autoOpt.value = "auto";
        autoOpt.textContent = "Automatisk (første tilkoblede)";
        selectEl.appendChild(autoOpt);

        const kbOpt = document.createElement("option");
        kbOpt.value = "keyboard";
        kbOpt.textContent = "Tastatur";
        selectEl.appendChild(kbOpt);

        for (let i = 0; i < pads.length; i++) {
            if (!pads[i]) continue;
            const opt = document.createElement("option");
            opt.value = String(i);
            opt.textContent = "Gamepad " + i + ": " + (pads[i].id || "ukjent enhet");
            selectEl.appendChild(opt);
        }

        selectEl.value = currentSource;
        if (selectEl.value !== currentSource) {
            selectEl.value = "auto";
            return "auto";
        }
        return currentSource;
    }

    // Bygger kanal-mapping-rader (akse-valg + reverser) for et sett kanaler, f.eks.
    // { throttle:"Gass", roll:"Roll", pitch:"Pitch", yaw:"Yaw" } eller aileron/elevator/rudder-varianten.
    function buildGamepadChannelsGrid(gridEl, channelMap, channelLabels, axisCount, onChange) {
        gridEl.innerHTML = "";
        Object.keys(channelLabels).forEach(function (channel) {
            const row = document.createElement("div");
            row.className = "sim-rate-row";
            const label = document.createElement("label");
            label.textContent = channelLabels[channel];
            const select = document.createElement("select");
            for (let i = 0; i < axisCount; i++) {
                const opt = document.createElement("option");
                opt.value = i;
                opt.textContent = "Kanal " + (i + 1);
                if (channelMap[channel].axis === i) opt.selected = true;
                select.appendChild(opt);
            }
            select.addEventListener("change", function () {
                channelMap[channel].axis = parseInt(select.value, 10);
                if (onChange) onChange();
            });

            const reverseLabel = document.createElement("label");
            reverseLabel.style.cssText = "display:flex; align-items:center; gap:4px; flex:0 0 60px; font-size:0.75rem;";
            const reverseInput = document.createElement("input");
            reverseInput.type = "checkbox";
            reverseInput.checked = channelMap[channel].reverse;
            reverseInput.addEventListener("change", function () {
                channelMap[channel].reverse = reverseInput.checked;
                if (onChange) onChange();
            });
            reverseLabel.appendChild(reverseInput);
            reverseLabel.appendChild(document.createTextNode("Rev."));

            row.appendChild(label);
            row.appendChild(select);
            row.appendChild(reverseLabel);
            gridEl.appendChild(row);
        });
    }

    // Bygger knappe-mapping-rader (Sett/Fjern) for et sett actions, f.eks.
    // { kill:"Kill/Arm", modeAcro:"Modus: Acro", ... }.
    function buildGamepadButtonsGrid(containerEl, bindingsObj, actionLabels, buttonManager, getGamepadFn, onChange) {
        containerEl.innerHTML = "";
        Object.keys(actionLabels).forEach(function (action) {
            const row = document.createElement("div");
            row.className = "sim-rate-row";
            const label = document.createElement("label");
            label.textContent = actionLabels[action];
            const statusSpan = document.createElement("span");
            statusSpan.className = "sim-rate-value";
            statusSpan.style.cssText = "flex:1; text-align:left;";
            function refreshStatus() {
                const b = bindingsObj[action];
                if (!b) statusSpan.textContent = "Ikke satt";
                else if (b.type === "axis") statusSpan.textContent = "Kanal " + (b.index + 1) + " (bryter)";
                else statusSpan.textContent = "Knapp " + b.index;
            }
            refreshStatus();

            const setBtn = document.createElement("button");
            setBtn.type = "button";
            setBtn.className = "sim-btn";
            setBtn.textContent = "Sett";
            setBtn.addEventListener("click", function () {
                setBtn.textContent = "Trykk knapp...";
                buttonManager.startListening(action, getGamepadFn());
                const checkDone = setInterval(function () {
                    if (buttonManager.isListening() !== action) {
                        setBtn.textContent = "Sett";
                        refreshStatus();
                        if (onChange) onChange();
                        clearInterval(checkDone);
                    }
                }, 150);
            });

            const clearBtn = document.createElement("button");
            clearBtn.type = "button";
            clearBtn.className = "sim-btn";
            clearBtn.textContent = "Fjern";
            clearBtn.addEventListener("click", function () {
                bindingsObj[action] = null;
                if (onChange) onChange();
                refreshStatus();
            });

            row.appendChild(label);
            row.appendChild(statusSpan);
            row.appendChild(setBtn);
            row.appendChild(clearBtn);
            containerEl.appendChild(row);
        });
    }

    // Viser BÅDE akser ("Kanal") og knapper - noen sendere/USB-adaptere legger enkelte brytere/kanaler
    // på gamepad-API-ets buttons[] i stedet for axes[] (typisk høyere kanalnumre på større sendere), og
    // uten knappe-listen her ville de brytene vært helt usynlige i denne visningen selv om de faktisk
    // registreres av nettleseren - så det så ut som "ingenting skjer" når man trykket dem.
    // outputByAxis (valgfri, kun quad-simulatoren bruker den p.t. - fixed-wing kaller uten): { akse-
    // indeks: {label, value} }, en ekstra kolonne som viser hva den kanalen faktisk gir som utgangsverdi
    // TIL SPILLET (etter reverse/skalering, se readStickAxis/readThrottleAxis) for kanaler som er mappet
    // til roll/pitch/yaw/gass - uten denne kunne man se rå kanalverdien endre seg, men ikke om/hvordan
    // f.eks. "Kalibrer fullt utslag" faktisk endret det programmet mottar.
    function updateGamepadAxesReadout(readoutEl, gp, minChannels, outputByAxis) {
        if (!gp) {
            readoutEl.textContent = "Ingen fjernkontroll/gamepad tilkoblet.";
            return;
        }
        readoutEl.innerHTML = "";
        const channelCount = Math.max(gp.axes.length, minChannels || MIN_GAMEPAD_CHANNELS);
        for (let i = 0; i < channelCount; i++) {
            const v = gp.axes[i];
            const line = document.createElement("div");
            line.style.cssText = "display:flex; gap:8px; align-items:baseline;";
            const chSpan = document.createElement("span");
            chSpan.style.cssText = "flex:0 0 62px;";
            chSpan.textContent = "Kanal " + (i + 1) + ":";
            const rawSpan = document.createElement("span");
            rawSpan.style.cssText = "flex:0 0 48px;";
            rawSpan.textContent = v === undefined ? "–" : v.toFixed(2);
            line.appendChild(chSpan);
            line.appendChild(rawSpan);
            const mapped = outputByAxis && outputByAxis[i];
            if (mapped) {
                const outSpan = document.createElement("span");
                outSpan.style.cssText = "color:var(--ffi-blue); font-weight:700;";
                outSpan.textContent = "→ " + mapped.label + ": " + mapped.value.toFixed(2);
                line.appendChild(outSpan);
            }
            readoutEl.appendChild(line);
        }
        if (gp.buttons.length > 0) {
            const heading = document.createElement("div");
            heading.style.cssText = "margin-top:6px; font-weight:700;";
            heading.textContent = "Knapper";
            readoutEl.appendChild(heading);
            for (let i = 0; i < gp.buttons.length; i++) {
                const btn = gp.buttons[i];
                const active = btn.pressed || btn.value > 0.05;
                const line = document.createElement("div");
                if (active) line.style.color = "#ffd76b";
                line.textContent = "Knapp " + i + ": " + btn.value.toFixed(2) + (active ? " (aktiv)" : "");
                readoutEl.appendChild(line);
            }
        }
    }

    /* ---------- Vind (stabil + kast) ---------- */
    // Beregner gjeldende vindvektor (verdensrom, m/s): stabil komponent + jevnt glattet
    // (ikke hakkete) tilfeldig kast-element. gustOffsetVec muteres (holder tilstand mellom kall).
    function computeWind(dt, windSettings, gustOffsetVec, outVec) {
        outVec = outVec || new THREE.Vector3();
        if (!windSettings.enabled) {
            outVec.set(0, 0, 0);
            return outVec;
        }
        const dirRad = THREE.MathUtils.degToRad(windSettings.directionDeg);
        const steady = new THREE.Vector3(Math.sin(dirRad), 0, Math.cos(dirRad)).multiplyScalar(windSettings.speed);
        if (windSettings.gust > 0) {
            const gustTarget = new THREE.Vector3(Math.random() * 2 - 1, 0, Math.random() * 2 - 1)
                .multiplyScalar(windSettings.gust * windSettings.speed);
            gustOffsetVec.lerp(gustTarget, Math.min(1, dt * 0.6));
        } else {
            gustOffsetVec.lerp(new THREE.Vector3(), Math.min(1, dt * 2));
        }
        outVec.copy(steady).add(gustOffsetVec);
        return outVec;
    }

    /* ---------- Scene-elementer ---------- */
    function buildGradientSky() {
        const skyGeo = new THREE.SphereGeometry(800, 32, 15);
        const skyMat = new THREE.ShaderMaterial({
            uniforms: {
                topColor: { value: new THREE.Color(0x4a90d9) },
                bottomColor: { value: new THREE.Color(0xdfefff) },
                offset: { value: 20 },
                exponent: { value: 0.6 }
            },
            vertexShader: [
                "varying vec3 vWorldPosition;",
                "void main() {",
                "  vec4 worldPosition = modelMatrix * vec4(position, 1.0);",
                "  vWorldPosition = worldPosition.xyz;",
                "  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);",
                "}"
            ].join("\n"),
            fragmentShader: [
                "uniform vec3 topColor;",
                "uniform vec3 bottomColor;",
                "uniform float offset;",
                "uniform float exponent;",
                "varying vec3 vWorldPosition;",
                "void main() {",
                "  float h = normalize(vWorldPosition + vec3(0.0, offset, 0.0)).y;",
                "  gl_FragColor = vec4(mix(bottomColor, topColor, max(pow(max(h, 0.0), exponent), 0.0)), 1.0);",
                "}"
            ].join("\n"),
            side: THREE.BackSide
        });
        return new THREE.Mesh(skyGeo, skyMat);
    }

    // Prosedural sjakkbrett-tekstur (ingen ekstern bildefil) - gir avstands-/høydereferanse mot bakken.
    function buildGroundTexture() {
        const size = 256;
        const canvas = document.createElement("canvas");
        canvas.width = size;
        canvas.height = size;
        const ctx = canvas.getContext("2d");
        const tiles = 8;
        const tileSize = size / tiles;
        for (let y = 0; y < tiles; y++) {
            for (let x = 0; x < tiles; x++) {
                const even = (x + y) % 2 === 0;
                ctx.fillStyle = even ? "#3a5f3a" : "#32502f";
                ctx.fillRect(x * tileSize, y * tileSize, tileSize, tileSize);
            }
        }
        const texture = new THREE.CanvasTexture(canvas);
        texture.wrapS = THREE.RepeatWrapping;
        texture.wrapT = THREE.RepeatWrapping;
        texture.repeat.set(125, 125); // ~2m per rute
        return texture;
    }

    // Prosedural "H"-landingsplass-tekstur (hvit sirkel, gul kant, sort H) - ingen ekstern bildefil.
    function buildLandingPadTexture() {
        const size = 256;
        const canvas = document.createElement("canvas");
        canvas.width = size;
        canvas.height = size;
        const ctx = canvas.getContext("2d");
        ctx.clearRect(0, 0, size, size);
        ctx.fillStyle = "#f2b100";
        ctx.beginPath();
        ctx.arc(size / 2, size / 2, size / 2 - 2, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = "#ffffff";
        ctx.beginPath();
        ctx.arc(size / 2, size / 2, size / 2 - 14, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = "#151515";
        ctx.font = "bold " + Math.round(size * 0.56) + "px sans-serif";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText("H", size / 2, size / 2 + size * 0.02);
        return new THREE.CanvasTexture(canvas);
    }

    function buildLandingPad(diameter) {
        const mesh = new THREE.Mesh(
            new THREE.CircleGeometry(diameter / 2, 32),
            new THREE.MeshStandardMaterial({ map: buildLandingPadTexture() })
        );
        mesh.rotation.x = -Math.PI / 2;
        return mesh;
    }

    // Rød/hvit stripetekstur (langs pølsens lengdeakse) - ingen ekstern bildefil.
    function buildWindsockStripeTexture() {
        const w = 32, h = 256;
        const canvas = document.createElement("canvas");
        canvas.width = w;
        canvas.height = h;
        const ctx = canvas.getContext("2d");
        const bands = 5; // oddetall slik at både innerste og ytterste bånd blir rødt (som en ekte vindpølse)
        const bandH = h / bands;
        for (let i = 0; i < bands; i++) {
            ctx.fillStyle = (i % 2 === 0) ? "#c62828" : "#f2f2f2";
            ctx.fillRect(0, Math.floor(i * bandH), w, Math.ceil(bandH) + 1);
        }
        return new THREE.CanvasTexture(canvas);
    }

    // Vindpølse på stolpen: mount-gruppen roteres (yaw) for å peke nedvinds, henger rett ned ved 0 vind
    // og reiser seg mot vannrett med økende styrke. Returnerer et handle-objekt (i stedet for å sette
    // module-globale variabler) slik at flere simulatorer kan ha hver sin vindpølse-instans.
    function buildWindsockPole() {
        const group = new THREE.Group();
        const poleMat = new THREE.MeshStandardMaterial({ color: 0xff5533 });
        const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.15, 7, 8), poleMat);
        pole.position.y = 3.5;
        pole.castShadow = true;
        group.add(pole);

        const yawGroup = new THREE.Group();
        yawGroup.position.y = 7;
        group.add(yawGroup);

        const droopPivot = new THREE.Group();
        yawGroup.add(droopPivot);

        const ringMat = new THREE.MeshStandardMaterial({ color: 0x888888 });
        const ring = new THREE.Mesh(new THREE.TorusGeometry(0.16, 0.02, 6, 12), ringMat);
        ring.rotation.y = Math.PI / 2;
        droopPivot.add(ring);

        const sockLength = 2.4; // realistisk vindpølse-lengde
        // radiusTop (lokal +Y) havner nærmest festeringen etter rotasjonen under, radiusBottom (lokal -Y)
        // havner ytterst - derfor radiusTop=bred (munning) og radiusBottom=smal (hale).
        const sockGeometry = new THREE.CylinderGeometry(0.35, 0.08, sockLength, 16, 10, true);
        const basePositions = Float32Array.from(sockGeometry.attributes.position.array);
        const sockMat = new THREE.MeshStandardMaterial({ map: buildWindsockStripeTexture(), side: THREE.DoubleSide });
        const sock = new THREE.Mesh(sockGeometry, sockMat);
        sock.rotation.z = Math.PI / 2;
        sock.position.x = sockLength / 2;
        sock.castShadow = true;
        droopPivot.add(sock);

        return { group: group, yawGroup: yawGroup, droopPivot: droopPivot, sockGeometry: sockGeometry, basePositions: basePositions, sockLength: sockLength };
    }

    // Oppdaterer vindpølsens retning (peker nedvinds), "slapphet" og en myk bølge langs geometrien.
    function updateWindsockVisual(handle, now, windVector) {
        if (!handle) return;
        const speed = windVector.length();
        if (speed > 0.05) {
            handle.yawGroup.rotation.y = Math.atan2(-windVector.z, windVector.x);
        }
        const strength = clamp(speed / 12, 0, 1);
        handle.droopPivot.rotation.z = -Math.PI / 2 * (1 - strength);

        const t = now * 0.001;
        const amp = 0.01 + strength * 0.06;
        const posAttr = handle.sockGeometry.attributes.position;
        const halfLen = handle.sockLength / 2;
        for (let i = 0; i < posAttr.count; i++) {
            const baseY = handle.basePositions[i * 3 + 1];
            const baseZ = handle.basePositions[i * 3 + 2];
            const alongTail = clamp((baseY + halfLen) / handle.sockLength, 0, 1); // 0=munning, 1=hale
            const wave = Math.sin(t * 3.2 + alongTail * 7) * amp * alongTail * alongTail;
            posAttr.setZ(i, baseZ + wave);
        }
        posAttr.needsUpdate = true;
        handle.sockGeometry.computeVertexNormals();
    }

    // Enkel prosedural pilot-/spotter-figur (~1.75 m) i hi-vis vest - brukes til å markere hvor VLOS-
    // observatøren står. Kalleren er ansvarlig for å sette figuren på et eget layer og skjule det laget
    // for VLOS-kameraet (personen skal ikke se seg selv), f.eks.:
    //   person.traverse(o => o.layers.set(1)); chaseCamera.layers.enable(1); fpvCamera.layers.enable(1);
    function buildPersonFigure(opts) {
        const holdingController = !!(opts && opts.holdingController);
        // vestColor: lar kalleren variere klesfargen (f.eks. en folkemengde med ulike farger i stedet
        // for uniform hi-vis-vest) - default uendret fra originalen (VLOS-observatørens oransje vest).
        const vestColor = (opts && opts.vestColor) || 0xff7a1a;
        const group = new THREE.Group();
        const skinMat = new THREE.MeshStandardMaterial({ color: 0xe0b088 });
        const vestMat = new THREE.MeshStandardMaterial({ color: vestColor });
        const pantsMat = new THREE.MeshStandardMaterial({ color: 0x2a3a4a });
        const shoeMat = new THREE.MeshStandardMaterial({ color: 0x1a1a1a });

        const legHeight = 0.75;
        [-1, 1].forEach(function (side) {
            const leg = new THREE.Mesh(new THREE.BoxGeometry(0.13, legHeight, 0.15), pantsMat);
            leg.position.set(side * 0.09, legHeight / 2, 0);
            leg.castShadow = true;
            group.add(leg);
            const shoe = new THREE.Mesh(new THREE.BoxGeometry(0.14, 0.06, 0.22), shoeMat);
            shoe.position.set(side * 0.09, 0.03, 0.03);
            shoe.castShadow = true;
            group.add(shoe);
        });

        const torsoHeight = 0.5;
        const torso = new THREE.Mesh(new THREE.BoxGeometry(0.34, torsoHeight, 0.2), vestMat);
        torso.position.y = legHeight + torsoHeight / 2;
        torso.castShadow = true;
        group.add(torso);

        const headRadius = 0.11;
        const head = new THREE.Mesh(new THREE.SphereGeometry(headRadius, 10, 8), skinMat);
        head.position.y = legHeight + torsoHeight + headRadius + 0.02;
        head.castShadow = true;
        group.add(head);

        [-1, 1].forEach(function (side) {
            let arm;
            if (holdingController) {
                // Armene bøyd frem og inn slik at hendene møtes på fjernkontrollen foran magen
                // (kalleren fester selve kontrolleren ved ca. (0, 1.05, 0.28) i figurens ramme).
                const shoulder = new THREE.Vector3(side * 0.21, 1.17, 0.02);
                const hand = new THREE.Vector3(side * 0.07, 1.06, 0.24);
                const dir = new THREE.Vector3().subVectors(shoulder, hand);
                arm = new THREE.Mesh(new THREE.BoxGeometry(0.09, dir.length() + 0.06, 0.09), vestMat);
                arm.position.copy(hand).addScaledVector(dir, 0.5);
                arm.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize());
            } else {
                arm = new THREE.Mesh(new THREE.BoxGeometry(0.09, 0.42, 0.09), vestMat);
                arm.position.set(side * 0.21, legHeight + torsoHeight - 0.18, 0);
                arm.rotation.z = side * 0.12;
            }
            arm.castShadow = true;
            group.add(arm);
        });

        return group;
    }

    // To norske treslag i stedet for én generisk trekjegle - begge bygges rundt samme origo-konvensjon
    // (basen på bakken, y=0), slik at createTreeSwayManager sin pivot-rundt-basen fungerer identisk
    // for begge. Delt mellom quad- og fixed-wing-simulatoren (begge hadde hver sin nesten identiske
    // trebygger før - flyttet hit for én kilde til sannhet).
    // Bjørk: lys, tynn stamme med noen mørke "bjørkeflekker", og en rundere, fyldigere krone bygget av
    // flere overlappende klumper (i stedet for én kjegle) - gir et løvtre-preg.
    function buildBirch(height) {
        const group = new THREE.Group();
        const trunkMat = new THREE.MeshStandardMaterial({ color: 0xd8d0c0 });
        const barkMat = new THREE.MeshStandardMaterial({ color: 0x2a2a26 });
        const canopyMat = new THREE.MeshStandardMaterial({ color: 0x7ba050 });
        const trunkHeight = height * 0.55;
        const trunk = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.14, trunkHeight, 8), trunkMat);
        trunk.position.y = trunkHeight / 2;
        trunk.castShadow = true;
        group.add(trunk);
        for (let i = 0; i < 4; i++) {
            const mark = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.05, 0.02), barkMat);
            mark.position.set(0.09, trunkHeight * (0.15 + i * 0.2), 0.06);
            mark.rotation.y = i * 1.3;
            group.add(mark);
        }
        const canopyBaseY = trunkHeight * 0.85;
        [
            { dx: 0, dz: 0, dy: 0, r: height * 0.22 },
            { dx: height * 0.13, dz: height * 0.05, dy: height * 0.08, r: height * 0.16 },
            { dx: -height * 0.11, dz: -height * 0.07, dy: height * 0.14, r: height * 0.15 }
        ].forEach(function (c) {
            const cluster = new THREE.Mesh(new THREE.IcosahedronGeometry(c.r, 1), canopyMat);
            cluster.position.set(c.dx, canopyBaseY + c.dy, c.dz);
            cluster.castShadow = true;
            group.add(cluster);
        });
        return group;
    }
    // Furu: mørk, tykkere stamme og en lagvis krone av avtagende kjeglesegmenter (typisk bartre-silhuett)
    // i stedet for én stor, jevn kjegle.
    function buildPine(height) {
        const group = new THREE.Group();
        const trunkMat = new THREE.MeshStandardMaterial({ color: 0x4a3320 });
        const canopyMat = new THREE.MeshStandardMaterial({ color: 0x264a2e });
        const trunkHeight = height * 0.3;
        const trunk = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.15, trunkHeight, 8), trunkMat);
        trunk.position.y = trunkHeight / 2;
        trunk.castShadow = true;
        group.add(trunk);
        const layerCount = 4;
        const canopyTotalHeight = height - trunkHeight;
        let y = trunkHeight;
        for (let i = 0; i < layerCount; i++) {
            const progress = i / (layerCount - 1);
            const layerH = canopyTotalHeight * 0.42 * (1 - progress * 0.35);
            const layerR = height * 0.24 * (1 - progress * 0.55);
            const layer = new THREE.Mesh(new THREE.ConeGeometry(layerR, layerH, 9), canopyMat);
            layer.position.y = y + layerH * 0.45;
            layer.castShadow = true;
            group.add(layer);
            y += layerH * 0.62;
        }
        return group;
    }
    // Tilfeldig bjørk eller furu, med litt tilfeldig høyde-/skala-variasjon for et mindre ensartet utseende.
    function buildRandomTree(height) {
        const h = height * (0.9 + Math.random() * 0.2);
        return Math.random() < 0.5 ? buildBirch(h) : buildPine(h);
    }

    // Trær som vaier i vinden - hvert tre lagres med sin egen tilfeldige fase/frekvens slik at de ikke
    // svaier helt synkront (som ville sett kunstig/robotisk ut). Egen factory (i stedet for delt modul-
    // tilstand) slik at quad- og fixed-wing-simulatoren har hver sin uavhengige treliste selv om begge
    // laster samme fil. Trærne bøyer seg i selve VINDRETNINGEN (ikke bare en generisk, retningsløs
    // oscillasjon), pluss en raskere, fase-forskjøvet "kast"-rist oppå den jevne bøyningen. Roterer
    // tre-gruppen rundt sin egen base (y=0, se buildBirch/buildPine) - en liten-vinkel-tilnærming
    // (separate X/Z-rotasjoner) til å tilte toppen mot vindretningen.
    function createTreeSwayManager() {
        let treeHandles = [];
        function addSwayingTree(group) {
            treeHandles.push({ group: group, phase: Math.random() * Math.PI * 2, freq: 0.7 + Math.random() * 0.5 });
            return group;
        }
        function update(now, windVector) {
            const windSpeed = windVector.length();
            if (windSpeed < 0.05) {
                treeHandles.forEach(function (t) { t.group.rotation.set(0, 0, 0); });
                return;
            }
            const t = now / 1000;
            const windDirX = windVector.x / windSpeed;
            const windDirZ = windVector.z / windSpeed;
            const leanAngle = Math.min(0.16, windSpeed * 0.015);
            treeHandles.forEach(function (tree) {
                const gustWiggle = 1 + Math.sin(t * tree.freq * 2.3 + tree.phase) * 0.3;
                const lean = leanAngle * gustWiggle;
                tree.group.rotation.x = windDirZ * lean;
                tree.group.rotation.z = -windDirX * lean;
            });
        }
        return { addSwayingTree: addSwayingTree, update: update };
    }

    /* ---------- Renderer/kamera ---------- */
    function resizeRenderer(renderer, wrapEl, cameras) {
        const w = wrapEl.clientWidth, h = wrapEl.clientHeight;
        renderer.setSize(w, h, false);
        cameras.forEach(function (cam) {
            cam.aspect = w / h;
            cam.updateProjectionMatrix();
        });
    }

    // Chase-kamera med manuell orbit - delt mellom quad- og fixed-wing-simulatoren (begge hadde tidligere
    // hver sin nesten identiske kopi av både tilstanden og de fem event-lytterne). Hold høyreklikk og dra
    // for å se rundt kjøretøyet, scroll for å zoome. Vinkel/avstand er en offset OVENPÅ kjøretøyets egen
    // heading - kameraet henger fortsatt bak og følger med rundt svinger, men piloten kan se seg rundt
    // (f.eks. for skjermbilder) uten at det påvirker selve styringen.
    // opts: { defaultPitch, zoomMin, zoomMax, initialZoom, smoothingBase, lookAtOffsetY }
    function createChaseCameraController(camera, canvasEl, opts) {
        let orbitYaw = 0;
        let orbitPitch = opts.defaultPitch;
        let zoomDistance = opts.initialZoom;
        let isOrbiting = false;
        let lastPointerX = 0, lastPointerY = 0;

        canvasEl.addEventListener("contextmenu", function (e) { e.preventDefault(); });
        canvasEl.addEventListener("mousedown", function (e) {
            if (e.button !== 2) return;
            isOrbiting = true;
            lastPointerX = e.clientX;
            lastPointerY = e.clientY;
        });
        global.addEventListener("mouseup", function (e) {
            if (e.button === 2) isOrbiting = false;
        });
        global.addEventListener("mousemove", function (e) {
            if (!isOrbiting) return;
            const dx = e.clientX - lastPointerX;
            const dy = e.clientY - lastPointerY;
            lastPointerX = e.clientX;
            lastPointerY = e.clientY;
            orbitYaw -= dx * 0.006;
            orbitPitch = clamp(orbitPitch + dy * 0.006, 0.02, 1.4);
        });
        canvasEl.addEventListener("wheel", function (e) {
            e.preventDefault();
            zoomDistance = clamp(zoomDistance + e.deltaY * 0.02, opts.zoomMin, opts.zoomMax);
        }, { passive: false });

        function update(dt, targetPosition, targetQuaternion) {
            // Følger kun med på yaw (heading), ikke full roll/pitch - gir en stabil, brukbar chase-cam
            // fremfor en kamera-rigg som er stivt låst til kjøretøyets fulle rotasjon.
            const euler = new THREE.Euler().setFromQuaternion(targetQuaternion, "YXZ");
            const headingQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, euler.y + orbitYaw, 0));
            const horizontalDist = zoomDistance * Math.cos(orbitPitch);
            const verticalDist = zoomDistance * Math.sin(orbitPitch);
            const behindOffset = new THREE.Vector3(0, verticalDist, horizontalDist).applyQuaternion(headingQuat);
            const desiredPos = targetPosition.clone().add(behindOffset);
            const smoothing = 1 - Math.pow(opts.smoothingBase, dt);
            camera.position.lerp(desiredPos, smoothing);
            camera.lookAt(targetPosition.clone().add(new THREE.Vector3(0, opts.lookAtOffsetY || 0, 0)));
        }
        return { update: update };
    }

    /* ---------- FPV HUD/OSD (crosshair / kunstig horisont) ---------- */
    // Betaflight-lignende OSD-crosshair: hvit, liten runding i midten med korte streker ut til hver side.
    function drawFpvCrosshair(ctx, w, h) {
        ctx.strokeStyle = "#ffffff";
        ctx.lineWidth = 1.5;
        const cx = w / 2, cy = h / 2;
        ctx.beginPath();
        ctx.arc(cx, cy, 3.5, 0, Math.PI * 2);
        ctx.stroke();
        const gap = 9, wingLen = 16;
        ctx.beginPath();
        ctx.moveTo(cx - gap - wingLen, cy); ctx.lineTo(cx - gap, cy);
        ctx.moveTo(cx + gap, cy); ctx.lineTo(cx + gap + wingLen, cy);
        ctx.stroke();
    }

    // Betaflight-lignende kunstig horisont: ingen himmel-/bakkefarge, kun noen korte hvite streker som
    // alltid ligger vannrett i forhold til den ekte horisonten (roterer med rull, flyttes med pitch).
    // Tar ferdig utregnede grader (pitchDeg/rollDeg) - kalleren avgjør aksekonvensjon selv.
    function drawFpvHorizonFromAngles(ctx, w, h, pitchDeg, rollDeg) {
        const pxPerDeg = 3;
        ctx.save();
        ctx.translate(w / 2, h / 2 + pitchDeg * pxPerDeg);
        ctx.rotate(THREE.MathUtils.degToRad(-rollDeg));
        ctx.strokeStyle = "#ffffff";
        ctx.lineWidth = 1.5;
        const dashLen = 26, gap = 13;
        ctx.beginPath();
        ctx.moveTo(-gap - dashLen, 0); ctx.lineTo(-gap, 0);
        ctx.moveTo(gap, 0); ctx.lineTo(gap + dashLen, 0);
        ctx.moveTo(0, -5); ctx.lineTo(0, 5);
        ctx.stroke();
        ctx.restore();
    }

    /* ---------- Panel/dropdown-UI ---------- */
    function togglePanel(panel, allPanels) {
        const wasOpen = panel.style.display !== "none";
        allPanels.forEach(function (p) { p.style.display = "none"; });
        panel.style.display = wasOpen ? "none" : "block";
    }

    // Kobler opp "tilbake" (lukk panelet, åpne Settings-menyen igjen) og "X" (bare lukk) - knapper i
    // panel-headerne (.sim-panel-back / .sim-panel-close). Kalles én gang ved oppstart; matcher enhver
    // .sim-panel som inneholder disse knappene, uavhengig av hvilken side som kaller det.
    function wirePanelCloseButtons(settingsMenuEl) {
        document.querySelectorAll(".sim-panel-close").forEach(function (btn) {
            btn.addEventListener("click", function () {
                const panel = btn.closest(".sim-panel");
                if (panel) panel.style.display = "none";
            });
        });
        document.querySelectorAll(".sim-panel-back").forEach(function (btn) {
            btn.addEventListener("click", function (e) {
                // stopPropagation er nødvendig: uten den bobler klikket videre opp til document, der
                // setupDropdown sin "lukk ved klikk utenfor"-lytter ser at klikket ikke var INNI selve
                // dropdown-menyen (knappen sitter jo i panelet, ikke i menyen) og fjerner "open"-klassen
                // igjen rett etter at denne håndtereren nettopp la den til - menyen så ut til å lukke seg
                // i stedet for å gå tilbake til den.
                e.stopPropagation();
                const panel = btn.closest(".sim-panel");
                if (panel) panel.style.display = "none";
                if (settingsMenuEl) settingsMenuEl.classList.add("open");
            });
        });
    }

    // Kobler en verktøylinje-"Settings"-knapp til en dropdown-meny: klikk åpner/lukker,
    // klikk utenfor lukker. Flere dropdowns på samme side lukker hverandre automatisk.
    // panelsToCloseOnOpen (valgfri): sim-panel-elementer som skal lukkes når selve dropdown-menyen åpnes -
    // ellers kan et undermeny-panel (Rates/Vind/osv.) fra en tidligere åpning fortsatt henge synlig
    // samtidig som Settings-menyen vises oppå, som ser rotete/dobbelt ut.
    function setupDropdown(toggleBtnEl, menuEl, panelsToCloseOnOpen) {
        toggleBtnEl.addEventListener("click", function (e) {
            e.stopPropagation();
            const isOpen = menuEl.classList.contains("open");
            document.querySelectorAll(".sim-dropdown-menu.open").forEach(function (m) {
                if (m !== menuEl) m.classList.remove("open");
            });
            const willOpen = !isOpen;
            menuEl.classList.toggle("open", willOpen);
            if (willOpen && panelsToCloseOnOpen) {
                panelsToCloseOnOpen.forEach(function (p) { if (p) p.style.display = "none"; });
            }
        });
        document.addEventListener("click", function (e) {
            if (menuEl.contains(e.target) || toggleBtnEl.contains(e.target)) return;
            menuEl.classList.remove("open");
        });
    }

    global.Sim = {
        STICK_RAMP_TIME: STICK_RAMP_TIME,
        MIN_GAMEPAD_CHANNELS: MIN_GAMEPAD_CHANNELS,
        clamp: clamp,
        rampStick: rampStick,
        computeRate: computeRate,
        computeThrottleCurve: computeThrottleCurve,
        integrateOrientation: integrateOrientation,
        loadJSON: loadJSON,
        saveJSON: saveJSON,
        drawRateCurve: drawRateCurve,
        solveCenterSensitivityForRate: solveCenterSensitivityForRate,
        buildRateAxisBox: buildRateAxisBox,
        buildThrottleExpoBox: buildThrottleExpoBox,
        rawFirstGamepad: rawFirstGamepad,
        getActiveGamepad: getActiveGamepad,
        readStickAxis: readStickAxis,
        readThrottleAxis: readThrottleAxis,
        createAxisCalibrationManager: createAxisCalibrationManager,
        isBindingActive: isBindingActive,
        createButtonBindingManager: createButtonBindingManager,
        populateInputSourceSelect: populateInputSourceSelect,
        buildGamepadChannelsGrid: buildGamepadChannelsGrid,
        buildGamepadButtonsGrid: buildGamepadButtonsGrid,
        updateGamepadAxesReadout: updateGamepadAxesReadout,
        computeWind: computeWind,
        buildGradientSky: buildGradientSky,
        buildGroundTexture: buildGroundTexture,
        buildLandingPadTexture: buildLandingPadTexture,
        buildLandingPad: buildLandingPad,
        buildWindsockPole: buildWindsockPole,
        updateWindsockVisual: updateWindsockVisual,
        buildPersonFigure: buildPersonFigure,
        buildBirch: buildBirch,
        buildPine: buildPine,
        buildRandomTree: buildRandomTree,
        createTreeSwayManager: createTreeSwayManager,
        resizeRenderer: resizeRenderer,
        createChaseCameraController: createChaseCameraController,
        drawFpvCrosshair: drawFpvCrosshair,
        drawFpvHorizonFromAngles: drawFpvHorizonFromAngles,
        togglePanel: togglePanel,
        wirePanelCloseButtons: wirePanelCloseButtons,
        setupDropdown: setupDropdown
    };
})(window);
