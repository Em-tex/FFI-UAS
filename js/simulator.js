/* js/simulator.js */

/* ---------- Konstanter: fysikk ---------- */
const GRAVITY = 9.81;               // m/s^2

// Dreiemoment-forsterkning for det indre rate-lookket (felles for alle drone-typer).
// Faktisk vinkelakselerasjon = TORQUE_GAIN * (ønsket_rate - faktisk_rate) / treghetsmoment(akse).
// Tyngre/større droner har høyere treghetsmoment -> tregere, mer "moment"-preget respons,
// og yaw har alltid høyere treghet enn roll/pitch (kun reaktivt motor-moment gir yaw-kraft på en ekte quad).
// (Verdiene under er dempet noe ift. en ren "snap-to-rate"-følelse, slik at treghet/moment merkes tydeligere -
// både i rotasjon og i hvor lenge droneen "seiler" videre lineært før luftmotstanden bremser den.)
const TORQUE_GAIN = 0.30;
const DRONE_CLASSES = {
    racing: {
        label: "Racing (rask, lett)",
        mass: 0.5, maxThrust: 18,
        inertiaRollPitch: 0.025, inertiaYaw: 0.05,
        linearDrag: 0.35, visualScale: 1.0
    },
    mid: {
        label: "Middels",
        mass: 1.2, maxThrust: 24,
        inertiaRollPitch: 0.07, inertiaYaw: 0.14,
        linearDrag: 0.55, visualScale: 1.3
    },
    cinematic: {
        label: "Cinematic (stor, treg)",
        mass: 2.6, maxThrust: 35,
        inertiaRollPitch: 0.16, inertiaYaw: 0.32,
        linearDrag: 0.9, visualScale: 2.3
    }
};
const DEFAULT_DRONE_CLASS = "racing";

const PASSIVE_ANGULAR_DAMPING = 0.995; // demping av rotasjon når disarmet (fritt fall/tumling)
const ANGLE_P_GAIN = 6;             // ytre selvnivellerings-lookk (Stabilized/Alt Hold), 1/s
const MAX_SELF_LEVEL_ANGLE = 35;    // grader
const MAX_CLIMB_RATE = 4;           // m/s, Alt Hold
const ALT_GAIN = 6;                 // N per (m/s) avvik i Alt Hold
const ALT_HOLD_DEADBAND = 0.12;     // ±12% rundt 50% gass regnes som "hold høyde"
const DEFAULT_FPV_TILT_DEG = -15;   // typisk oppovervinklet FPV-kamera-montering
const GROUND_CLEARANCE = 0.08;      // m, bakkekontakt
const FIXED_DT = 1 / 120;           // fysikk-tidssteg
const STICK_RAMP_TIME = 0.22;       // sekunder til full utslag (tastatur)
const THROTTLE_RATE = 0.7;          // gass endring per sekund (tastatur)

// Realistisk modus: batteri og linkkvalitet (kun aktivt når settings.realisticMode er på).
const LAUNCH_POINT = new THREE.Vector3(0, 1, 0);
const BATTERY_DRAIN_IDLE = 0.15;    // %/s ved 0% gass
const BATTERY_DRAIN_FULL = 1.4;     // %/s ved 100% gass
const BATTERY_LOW_THRESHOLD = 20;   // % - under dette svekkes makstrekk (spenningsfall)
const LINK_RANGE_FULL = 60;         // m - full linkkvalitet innenfor denne avstanden
const LINK_RANGE_ZERO = 150;        // m - linken er helt død her (uten hindring)
const LINK_OBSTRUCTION_PENALTY = 0.12; // multiplikator når siktlinjen til bygget er blokkert

const MODE_LABELS = { acro: "Acro", stabilized: "Stabilized", althold: "Alt Hold" };
const AXIS_LABELS = { roll: "Roll", pitch: "Pitch", yaw: "Yaw" };
const CHANNEL_LABELS = { roll: "Roll", pitch: "Pitch", yaw: "Yaw", throttle: "Gass" };
const MIN_GAMEPAD_CHANNELS = 8; // RC-sendere har typisk minst 8 kanaler i USB-joystick-modus

const RATE_STORAGE_KEY = "ffi-uas:simulator-rates";
const GAMEPAD_STORAGE_KEY = "ffi-uas:simulator-gamepad-map";
const SETTINGS_STORAGE_KEY = "ffi-uas:simulator-settings";

const DEFAULT_RATES = {
    roll: { centerSensitivity: 100, maxRate: 620, expo: 0.3 },
    pitch: { centerSensitivity: 100, maxRate: 620, expo: 0.3 },
    yaw: { centerSensitivity: 100, maxRate: 400, expo: 0.3 },
    throttle: { expo: 0 }
};

// TAER-rekkefølge (Throttle/Aileron/Elevator/Rudder) - standard kanalrekkefølge på TBS Crossfire/Tango-
// sendere i USB-joystick-modus. Justerbart i kalibreringspanelet for andre sendere/rekkefølger.
const DEFAULT_GAMEPAD_MAP = {
    throttle: { axis: 0, reverse: false },
    roll: { axis: 1, reverse: false },
    pitch: { axis: 2, reverse: false },
    yaw: { axis: 3, reverse: false },
    buttons: { kill: null, modeAcro: null, modeStabilized: null, modeAltHold: null }
};
const BUTTON_ACTION_LABELS = { kill: "Kill/Arm", modeAcro: "Modus: Acro", modeStabilized: "Modus: Stabilized", modeAltHold: "Modus: Alt Hold" };

const DEFAULT_WIND = { enabled: false, speed: 5, directionDeg: 0, gust: 0.3 };

const DEFAULT_SETTINGS = {
    fpvTiltDeg: DEFAULT_FPV_TILT_DEG,
    droneClass: DEFAULT_DRONE_CLASS,
    realisticMode: false,
    inputSource: "auto", // "auto" | "keyboard" | gamepad-indeks som streng ("0", "1", ...)
    wind: DEFAULT_WIND,
    fpvHudMode: "crosshair" // "crosshair" | "horizon" | "none"
};

const FPV_HUD_MODES = ["crosshair", "horizon", "none"];
const FPV_HUD_MODE_LABELS = { crosshair: "Crosshair", horizon: "Kunstig horisont", none: "Ingen" };

/* ---------- Hjelpefunksjoner ---------- */
function clamp(v, min, max) { return Math.min(max, Math.max(min, v)); }

function rampStick(current, target, dt) {
    const maxDelta = dt / STICK_RAMP_TIME;
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

function loadRates() {
    const result = {
        roll: Object.assign({}, DEFAULT_RATES.roll),
        pitch: Object.assign({}, DEFAULT_RATES.pitch),
        yaw: Object.assign({}, DEFAULT_RATES.yaw),
        throttle: Object.assign({}, DEFAULT_RATES.throttle)
    };
    try {
        const raw = localStorage.getItem(RATE_STORAGE_KEY);
        if (raw) {
            const parsed = JSON.parse(raw);
            ["roll", "pitch", "yaw", "throttle"].forEach(function (axis) {
                if (parsed[axis]) Object.assign(result[axis], parsed[axis]);
            });
        }
    } catch (e) {}
    return result;
}

// Enkel throttle-kurve (samme kubiske expo-formel som rate-kurvene, men på gass 0..1 i stedet for grader/s).
function computeThrottleCurve(stick01, expo) {
    const centered = stick01 * 2 - 1;
    const shaped = centered * (1 - expo) + expo * Math.pow(centered, 3);
    return clamp((shaped + 1) / 2, 0, 1);
}

function saveRates() {
    localStorage.setItem(RATE_STORAGE_KEY, JSON.stringify(rates));
}

function loadGamepadMap() {
    const result = {
        roll: Object.assign({}, DEFAULT_GAMEPAD_MAP.roll),
        pitch: Object.assign({}, DEFAULT_GAMEPAD_MAP.pitch),
        yaw: Object.assign({}, DEFAULT_GAMEPAD_MAP.yaw),
        throttle: Object.assign({}, DEFAULT_GAMEPAD_MAP.throttle),
        buttons: Object.assign({}, DEFAULT_GAMEPAD_MAP.buttons)
    };
    try {
        const raw = localStorage.getItem(GAMEPAD_STORAGE_KEY);
        if (raw) {
            const parsed = JSON.parse(raw);
            ["roll", "pitch", "yaw", "throttle"].forEach(function (ch) {
                if (parsed[ch]) Object.assign(result[ch], parsed[ch]);
            });
            if (parsed.buttons) Object.assign(result.buttons, parsed.buttons);
        }
    } catch (e) {}
    return result;
}

function saveGamepadMap() {
    localStorage.setItem(GAMEPAD_STORAGE_KEY, JSON.stringify(gamepadMap));
}

function loadSettings() {
    const result = Object.assign({}, DEFAULT_SETTINGS, { wind: Object.assign({}, DEFAULT_WIND) });
    try {
        const raw = localStorage.getItem(SETTINGS_STORAGE_KEY);
        if (raw) {
            const parsed = JSON.parse(raw);
            Object.assign(result, parsed);
            result.wind = Object.assign({}, DEFAULT_WIND, parsed.wind);
        }
    } catch (e) {}
    // Vind skal alltid være av ved sideinnlasting, uansett hva som var lagret fra en tidligere økt -
    // styrke/retning/kast huskes fortsatt, men man må aktivere vinden på nytt hver gang siden lastes.
    result.wind.enabled = false;
    return result;
}

function saveSettings() {
    localStorage.setItem(SETTINGS_STORAGE_KEY, JSON.stringify(settings));
}

/* ---------- Tilstand ---------- */
const rates = loadRates();
const gamepadMap = loadGamepadMap();
const settings = loadSettings();

const droneState = {
    position: new THREE.Vector3(0, 1.0, 0),
    velocity: new THREE.Vector3(0, 0, 0),
    quaternion: new THREE.Quaternion(),
    angularVelocity: { pitch: 0, yaw: 0, roll: 0 }, // rad/s, body-akser
    armed: true,
    flightMode: "stabilized",
    droneClass: DRONE_CLASSES[settings.droneClass] ? settings.droneClass : DEFAULT_DRONE_CLASS,
    batteryPercent: 100,
    grounded: false // i bakkekontakt denne fysikk-ticken - vind skal ikke drifte den mens den står
};

let linkQuality = 1;

/* ---------- Vind (stabil + kast) ---------- */
const currentWindVector = new THREE.Vector3();
const windGustOffset = new THREE.Vector3();

// Beregner gjeldende vindvektor (verdensrom, m/s): en stabil komponent pluss et jevnt glattet
// (ikke hakkete) tilfeldig kast-element, slik at kastene føles naturlige og ikke som støy.
function updateWind(dt) {
    const wind = settings.wind;
    if (!wind.enabled) {
        currentWindVector.set(0, 0, 0);
        return;
    }
    const dirRad = THREE.MathUtils.degToRad(wind.directionDeg);
    const steady = new THREE.Vector3(Math.sin(dirRad), 0, Math.cos(dirRad)).multiplyScalar(wind.speed);
    if (wind.gust > 0) {
        const gustTarget = new THREE.Vector3(Math.random() * 2 - 1, 0, Math.random() * 2 - 1)
            .multiplyScalar(wind.gust * wind.speed);
        windGustOffset.lerp(gustTarget, Math.min(1, dt * 0.6));
    } else {
        windGustOffset.lerp(new THREE.Vector3(), Math.min(1, dt * 2));
    }
    currentWindVector.copy(steady).add(windGustOffset);
}

function currentDroneSpec() {
    return DRONE_CLASSES[droneState.droneClass];
}

function setDroneClass(className) {
    if (!DRONE_CLASSES[className]) return;
    droneState.droneClass = className;
    settings.droneClass = className;
    saveSettings();
    // Drone-typene har ulik geometri (ben, canopy, propell-blad) - ikke bare skala - så modellen
    // må bygges på nytt, ikke bare skaleres, når typen endres.
    if (scene) rebuildDroneMesh();
}

const inputState = {
    source: "keyboard",
    stick: { roll: 0, pitch: 0, yaw: 0, throttle: 0 }
};

const keys = new Set();

let renderer, scene, chaseCamera, fpvCamera, vlosCamera, activeCamera;
let droneGroup, dronePropellers;
const CAMERA_MODES = ["chase", "fpv", "vlos"];
const CAMERA_MODE_LABELS = { chase: "Chase", fpv: "FPV", vlos: "VLOS" };
let cameraModeIndex = 0;

/* ---------- Three.js: scene, drone, kameraer ---------- */
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
// og reiser seg mot vannrett med økende styrke. Én sammenhengende, jevnt avsmalnende form (ingen
// hakkete skjøter) - "blafringen" gjøres i stedet med en myk bølge i selve geometrien.
let windsockYawGroup = null;
let windsockDroopPivot = null;
let windsockSockGeometry = null;
let windsockBasePositions = null;
let windsockSockLength = 2.4;

function buildWindsockPole() {
    const group = new THREE.Group();
    const poleMat = new THREE.MeshStandardMaterial({ color: 0xff5533 });
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.15, 7, 8), poleMat);
    pole.position.y = 3.5;
    pole.castShadow = true;
    group.add(pole);

    windsockYawGroup = new THREE.Group();
    windsockYawGroup.position.y = 7;
    group.add(windsockYawGroup);

    windsockDroopPivot = new THREE.Group();
    windsockYawGroup.add(windsockDroopPivot);

    const ringMat = new THREE.MeshStandardMaterial({ color: 0x888888 });
    const ring = new THREE.Mesh(new THREE.TorusGeometry(0.16, 0.02, 6, 12), ringMat);
    ring.rotation.y = Math.PI / 2;
    windsockDroopPivot.add(ring);

    windsockSockLength = 2.4; // realistisk vindpølse-lengde
    // radiusTop (lokal +Y) havner nærmest festeringen etter rotasjonen under, radiusBottom (lokal -Y)
    // havner ytterst - derfor radiusTop=bred (munning) og radiusBottom=smal (hale).
    windsockSockGeometry = new THREE.CylinderGeometry(0.35, 0.08, windsockSockLength, 16, 10, true);
    windsockBasePositions = Float32Array.from(windsockSockGeometry.attributes.position.array);
    const sockMat = new THREE.MeshStandardMaterial({ map: buildWindsockStripeTexture(), side: THREE.DoubleSide });
    const sock = new THREE.Mesh(windsockSockGeometry, sockMat);
    sock.rotation.z = Math.PI / 2;
    sock.position.x = windsockSockLength / 2;
    sock.castShadow = true;
    windsockDroopPivot.add(sock);

    return group;
}

function buildGround() {
    const group = new THREE.Group();
    const groundMat = new THREE.MeshStandardMaterial({ map: buildGroundTexture() });
    const ground = new THREE.Mesh(new THREE.PlaneGeometry(2000, 2000), groundMat);
    ground.rotation.x = -Math.PI / 2;
    ground.receiveShadow = true;
    group.add(ground);
    group.add(new THREE.GridHelper(2000, 200, 0x1f3d1f, 0x2d4d2d));

    const poleMat = new THREE.MeshStandardMaterial({ color: 0xff5533 });
    for (let i = 0; i < 8; i++) {
        const angle = (i / 8) * Math.PI * 2;
        const r = 30;
        if (i === 6) {
            // Rett foran (-Z) avgangsplassen sett fra spawn - stolpe med vindpølse i stedet for vanlig stolpe.
            const windsockPole = buildWindsockPole();
            windsockPole.position.set(Math.cos(angle) * r, 0, Math.sin(angle) * r);
            group.add(windsockPole);
            continue;
        }
        const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.15, 6, 8), poleMat);
        pole.position.set(Math.cos(angle) * r, 3, Math.sin(angle) * r);
        pole.castShadow = true;
        group.add(pole);
    }
    return group;
}

// Enkel bil, bygg med flatt tak og trær - prosedurale former i realistisk skala mot droneen.
function buildCar() {
    const group = new THREE.Group();
    const bodyMat = new THREE.MeshStandardMaterial({ color: 0xb33a3a });
    const cabinMat = new THREE.MeshStandardMaterial({ color: 0x223344 });
    const wheelMat = new THREE.MeshStandardMaterial({ color: 0x111111 });

    const wheelRadius = 0.32;
    const bodyHeight = 0.8;
    const bodyCenterY = wheelRadius + bodyHeight / 2;
    const body = new THREE.Mesh(new THREE.BoxGeometry(4.3, bodyHeight, 1.8), bodyMat);
    body.position.y = bodyCenterY;
    body.castShadow = true;
    body.receiveShadow = true;
    group.add(body);

    const cabinHeight = 0.55;
    const cabin = new THREE.Mesh(new THREE.BoxGeometry(2.2, cabinHeight, 1.6), cabinMat);
    cabin.position.set(-0.2, bodyCenterY + bodyHeight / 2 + cabinHeight / 2, 0);
    cabin.castShadow = true;
    group.add(cabin);

    [[1.4, 0.95], [1.4, -0.95], [-1.4, 0.95], [-1.4, -0.95]].forEach(function (p) {
        const wheel = new THREE.Mesh(new THREE.CylinderGeometry(wheelRadius, wheelRadius, 0.25, 16), wheelMat);
        wheel.rotation.z = Math.PI / 2;
        wheel.position.set(p[0], wheelRadius, p[1]);
        wheel.castShadow = true;
        group.add(wheel);
    });

    return group;
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

// Verdenskonstanter for bygget - gjenbrukt av linje-i-sikt-sjekken for Realistisk modus (lenger ned).
const BUILDING_POSITION = new THREE.Vector3(-35, 0, -35);
const BUILDING_SIZE = { width: 10, height: 14.3, depth: 8 };

function isLineBlockedByBuilding(from, to) {
    const box = new THREE.Box3(
        new THREE.Vector3(BUILDING_POSITION.x - BUILDING_SIZE.width / 2, 0, BUILDING_POSITION.z - BUILDING_SIZE.depth / 2),
        new THREE.Vector3(BUILDING_POSITION.x + BUILDING_SIZE.width / 2, BUILDING_SIZE.height, BUILDING_POSITION.z + BUILDING_SIZE.depth / 2)
    );
    const dir = new THREE.Vector3().subVectors(to, from);
    const dist = dir.length();
    if (dist < 1e-6) return false;
    dir.normalize();
    const hit = new THREE.Ray(from, dir).intersectBox(box, new THREE.Vector3());
    return !!hit && from.distanceTo(hit) < dist;
}

// Faste objekter droneen kan lande oppå (i stedet for å falle gjennom): topp-flate per boks,
// oppgitt akse-rettet (bilens rotasjon tilnærmes med en litt større boks for enkelhets skyld).
const SOLID_COLLIDERS = [
    {
        minX: BUILDING_POSITION.x - BUILDING_SIZE.width / 2, maxX: BUILDING_POSITION.x + BUILDING_SIZE.width / 2,
        minZ: BUILDING_POSITION.z - BUILDING_SIZE.depth / 2, maxZ: BUILDING_POSITION.z + BUILDING_SIZE.depth / 2,
        topY: BUILDING_SIZE.height
    },
    { minX: 10 - 2.33, maxX: 10 + 2.33, minZ: 7 - 1.58, maxZ: 7 + 1.58, topY: 1.7 }
];

function solidSurfaceHeightAt(x, z) {
    let top = 0;
    SOLID_COLLIDERS.forEach(function (c) {
        if (x >= c.minX && x <= c.maxX && z >= c.minZ && z <= c.maxZ) {
            top = Math.max(top, c.topY);
        }
    });
    return top;
}

// solidSurfaceHeightAt() gir kun en topp-flate å lande på ovenfra - uten en egen sidevegg-sjekk vil
// droneen fly rett gjennom veggen/kanten på et objekt når den nærmer seg fra siden i stedet for ovenfra
// (kun Y ble klemt opp mot toppen, X/Z var alltid fri). Dytter punktet ut til nærmeste vegg-side når det
// befinner seg innenfor fotavtrykket men klart under toppen - dvs. det er ikke i ferd med å lande, det
// prøver å fly gjennom veggen. Returnerer true hvis punktet var "innfelt" i en veggside (kalleren skal da
// IKKE også tolke det som en landing på toppen samme steg - det ga tidligere et brått oppløft opp på taket
// idet punktet stanset akkurat ved kanten).
function pushOutOfSolidWalls(point, velocity) {
    let embedded = false;
    SOLID_COLLIDERS.forEach(function (c) {
        if (point.x < c.minX || point.x > c.maxX || point.z < c.minZ || point.z > c.maxZ) return;
        if (point.y >= c.topY - GROUND_CLEARANCE) return;
        embedded = true;
        const distMinX = point.x - c.minX;
        const distMaxX = c.maxX - point.x;
        const distMinZ = point.z - c.minZ;
        const distMaxZ = c.maxZ - point.z;
        const minDist = Math.min(distMinX, distMaxX, distMinZ, distMaxZ);
        if (minDist === distMinX) { point.x = c.minX; if (velocity.x > 0) velocity.x = 0; }
        else if (minDist === distMaxX) { point.x = c.maxX; if (velocity.x < 0) velocity.x = 0; }
        else if (minDist === distMinZ) { point.z = c.minZ; if (velocity.z > 0) velocity.z = 0; }
        else { point.z = c.maxZ; if (velocity.z < 0) velocity.z = 0; }
    });
    return embedded;
}

function buildBuilding() {
    const group = new THREE.Group();
    const wallMat = new THREE.MeshStandardMaterial({ color: 0x9a9488 });
    const roofMat = new THREE.MeshStandardMaterial({ color: 0x55504a });
    const width = 10, depth = 8, height = 14;
    const walls = new THREE.Mesh(new THREE.BoxGeometry(width, height, depth), wallMat);
    walls.position.y = height / 2;
    walls.castShadow = true;
    walls.receiveShadow = true;
    group.add(walls);
    const roof = new THREE.Mesh(new THREE.BoxGeometry(width + 0.4, 0.3, depth + 0.4), roofMat);
    roof.position.y = height + 0.15;
    roof.castShadow = true;
    roof.receiveShadow = true;
    group.add(roof);

    const roofPad = buildLandingPad(Math.min(width, depth) - 1);
    roofPad.position.y = height + 0.32;
    roofPad.receiveShadow = true;
    group.add(roofPad);

    return group;
}

// Firkantet racing-gate med sjakkrutet ramme (oransje/hvit), montert på to bein over bakken.
function buildGate(size, groundGap) {
    const group = new THREE.Group();
    const barThickness = 0.18;
    const matA = new THREE.MeshStandardMaterial({ color: 0xff6a00 });
    const matB = new THREE.MeshStandardMaterial({ color: 0xffffff });
    const legMat = new THREE.MeshStandardMaterial({ color: 0x333333 });

    function checkeredBar(length, horizontal) {
        const segs = 6;
        const segLen = length / segs;
        const barGroup = new THREE.Group();
        for (let i = 0; i < segs; i++) {
            const mat = (i % 2 === 0) ? matA : matB;
            const geo = horizontal
                ? new THREE.BoxGeometry(segLen, barThickness, barThickness)
                : new THREE.BoxGeometry(barThickness, segLen, barThickness);
            const seg = new THREE.Mesh(geo, mat);
            const offset = -length / 2 + segLen / 2 + i * segLen;
            if (horizontal) seg.position.x = offset; else seg.position.y = offset;
            barGroup.add(seg);
        }
        return barGroup;
    }

    const top = checkeredBar(size, true);
    top.position.y = groundGap + size;
    group.add(top);

    const bottom = checkeredBar(size, true);
    bottom.position.y = groundGap;
    group.add(bottom);

    const left = checkeredBar(size, false);
    left.position.set(-size / 2, groundGap + size / 2, 0);
    group.add(left);

    const right = checkeredBar(size, false);
    right.position.set(size / 2, groundGap + size / 2, 0);
    group.add(right);

    [-size / 2, size / 2].forEach(function (x) {
        const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.08, groundGap, 8), legMat);
        leg.position.set(x, groundGap / 2, 0);
        group.add(leg);
    });

    return group;
}

// En låve med to vindusåpninger på motstående vegger - flyr inn det ene, gjennom, og ut det andre.
// Sideveggene er hele; front-/bakvegg bygges av fire paneler rundt et hull (ingen geometri i selve åpningen).
function buildBarn(width, height, depth, windowW, windowH, sillY) {
    const group = new THREE.Group();
    const wallMat = new THREE.MeshStandardMaterial({ color: 0xa1352b });
    const roofMat = new THREE.MeshStandardMaterial({ color: 0x3a3a3a });
    const wallThickness = 0.3;

    const leftWall = new THREE.Mesh(new THREE.BoxGeometry(wallThickness, height, depth), wallMat);
    leftWall.position.set(-width / 2, height / 2, 0);
    leftWall.castShadow = true;
    leftWall.receiveShadow = true;
    group.add(leftWall);
    const rightWall = leftWall.clone();
    rightWall.position.x = width / 2;
    group.add(rightWall);

    function buildWindowWall(zPos) {
        const wallGroup = new THREE.Group();
        const topY = sillY + windowH;
        const bottom = new THREE.Mesh(new THREE.BoxGeometry(width, sillY, wallThickness), wallMat);
        bottom.position.y = sillY / 2;
        wallGroup.add(bottom);
        const top = new THREE.Mesh(new THREE.BoxGeometry(width, height - topY, wallThickness), wallMat);
        top.position.y = topY + (height - topY) / 2;
        wallGroup.add(top);
        const sidePanelWidth = (width - windowW) / 2;
        const leftPanel = new THREE.Mesh(new THREE.BoxGeometry(sidePanelWidth, windowH, wallThickness), wallMat);
        leftPanel.position.set(-width / 2 + sidePanelWidth / 2, sillY + windowH / 2, 0);
        wallGroup.add(leftPanel);
        const rightPanel = leftPanel.clone();
        rightPanel.position.x = width / 2 - sidePanelWidth / 2;
        wallGroup.add(rightPanel);
        wallGroup.children.forEach(function (m) { m.castShadow = true; m.receiveShadow = true; });
        wallGroup.position.z = zPos;
        return wallGroup;
    }
    group.add(buildWindowWall(-depth / 2));
    group.add(buildWindowWall(depth / 2));

    const roof = new THREE.Mesh(new THREE.BoxGeometry(width + 0.6, 0.3, depth + 0.6), roofMat);
    roof.position.y = height + 0.15;
    roof.castShadow = true;
    roof.receiveShadow = true;
    group.add(roof);

    return group;
}

function buildTree(height) {
    const group = new THREE.Group();
    const trunkMat = new THREE.MeshStandardMaterial({ color: 0x5b3d24 });
    const canopyMat = new THREE.MeshStandardMaterial({ color: 0x2f5d34 });
    const trunkHeight = height * 0.45;
    const trunk = new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.24, trunkHeight, 8), trunkMat);
    trunk.position.y = trunkHeight / 2;
    trunk.castShadow = true;
    group.add(trunk);
    const canopyHeight = height - trunkHeight;
    const canopy = new THREE.Mesh(new THREE.ConeGeometry(height * 0.28, canopyHeight, 10), canopyMat);
    canopy.position.y = trunkHeight + canopyHeight / 2;
    canopy.castShadow = true;
    group.add(canopy);
    return group;
}

const GATE_COURSE_CENTER = new THREE.Vector3(0, 0, -110);
// Håndplasserte veipunkter (relativt til senter) gir en avlang bane (lange rettstrekk, trange svinger i
// hver ende - som en ekte racerbane, ikke bare en stor sirkel) med én låve du flyr gjennom (inn det ene
// vinduet, ut det andre) på det ene rettstrekket. På det andre rettstrekket er det lagt inn en trang
// chikane som svinger andre veien enn resten av banen, i stedet for at hele løypa svinger samme vei
// rundt. Nærsiden (mot avgangsplassen) holdes moderat for å unngå trærne der.
const GATE_WAYPOINTS = [
    { type: "gate", dx: 0, dz: 45, gap: 1.5, size: 3.0 },
    { type: "gate", dx: 24, dz: 42, gap: 2.4, size: 3.2 },
    { type: "gate", dx: 36, dz: 18, gap: 1.3, size: 2.8 },
    { type: "gate", dx: 36, dz: -15, gap: 2.2, size: 3.2 },
    { type: "barn", dx: 34, dz: -50 },
    { type: "gate", dx: 32, dz: -85, gap: 1.6, size: 3.0 },
    { type: "gate", dx: 18, dz: -112, gap: 2.6, size: 3.4 },
    { type: "gate", dx: 0, dz: -126, gap: 1.4, size: 2.8 },
    { type: "gate", dx: -18, dz: -112, gap: 2.4, size: 3.2 },
    { type: "gate", dx: -32, dz: -85, gap: 1.6, size: 2.8 },
    // Chikane: trangt sikksakk (skarpt kick "feil vei" midt i) i stedet for en jevn bue som resten av banen.
    { type: "gate", dx: -40, dz: -60, gap: 1.3, size: 2.6 },
    { type: "gate", dx: -16, dz: -40, gap: 1.2, size: 2.6 },
    { type: "gate", dx: -38, dz: -18, gap: 1.4, size: 2.6 },
    { type: "gate", dx: -30, dz: 8, gap: 2.0, size: 3.0 },
    { type: "gate", dx: -24, dz: 32, gap: 2.2, size: 3.2 }
];
const BARN_DIMENSIONS = { width: 8, height: 7, depth: 10, windowW: 3.2, windowH: 3.2, sillY: 1.8 };

function buildGateCourse() {
    const group = new THREE.Group();
    const n = GATE_WAYPOINTS.length;
    for (let i = 0; i < n; i++) {
        const wp = GATE_WAYPOINTS[i];
        const next = GATE_WAYPOINTS[(i + 1) % n];
        const obstacle = (wp.type === "barn")
            ? buildBarn(BARN_DIMENSIONS.width, BARN_DIMENSIONS.height, BARN_DIMENSIONS.depth,
                BARN_DIMENSIONS.windowW, BARN_DIMENSIONS.windowH, BARN_DIMENSIONS.sillY)
            : buildGate(wp.size, wp.gap);
        obstacle.position.set(GATE_COURSE_CENTER.x + wp.dx, 0, GATE_COURSE_CENTER.z + wp.dz);
        // Retter mot neste veipunkt, slik at man flyr gjennom (port eller låvevindu) langs den slyngede løypa.
        obstacle.rotation.y = Math.atan2(next.dx - wp.dx, next.dz - wp.dz);
        group.add(obstacle);
    }
    return group;
}

function buildWorldObjects() {
    const group = new THREE.Group();

    const spawnPad = buildLandingPad(2.4);
    spawnPad.position.y = 0.02; // løftet litt over bakkeplanet for å unngå z-fighting/flimring
    spawnPad.receiveShadow = true;
    group.add(spawnPad); // ved avgangsplassen (0,0,0)

    const car = buildCar();
    car.position.set(10, 0, 7);
    car.rotation.y = THREE.MathUtils.degToRad(20);
    group.add(car);

    const building = buildBuilding();
    building.position.set(-35, 0, -35);
    group.add(building);

    [
        { x: 45, z: -20, h: 7 }, { x: 55, z: 5, h: 8 }, { x: 40, z: 30, h: 6.5 },
        { x: -50, z: 20, h: 7.5 }, { x: -20, z: -55, h: 8.5 }, { x: 15, z: -60, h: 6 },
        { x: 70, z: -40, h: 7.2 }, { x: -60, z: -10, h: 6.8 }
    ].forEach(function (t) {
        const tree = buildTree(t.h);
        tree.position.set(t.x, 0, t.z);
        group.add(tree);
    });

    group.add(buildGateCourse());

    return group;
}

// Propell med 2 blad (én bjelke gjennom hub) eller 3+ blad (individuelle blad radielt fordelt).
function buildPropeller(bladeCount, bladeLength) {
    const group = new THREE.Group();
    const mat = new THREE.MeshStandardMaterial({ color: 0x111111, transparent: true, opacity: 0.8 });
    if (bladeCount <= 2) {
        group.add(new THREE.Mesh(new THREE.BoxGeometry(bladeLength, 0.005, 0.02), mat));
    } else {
        for (let i = 0; i < bladeCount; i++) {
            const pivot = new THREE.Group();
            pivot.rotation.y = (i / bladeCount) * Math.PI * 2;
            const blade = new THREE.Mesh(new THREE.BoxGeometry(bladeLength * 0.5, 0.005, 0.018), mat);
            blade.position.x = bladeLength * 0.25;
            pivot.add(blade);
            group.add(pivot);
        }
    }
    return group;
}

// Bygger en sylinder som går nøyaktig mellom to punkter - unngår feilaktig/"skjev" vinkling
// som lett oppstår ved å kombinere rotation.x/rotation.z manuelt på en forskjøvet posisjon.
function buildStrutBetween(p1, p2, radius, material) {
    const dir = new THREE.Vector3().subVectors(p2, p1);
    const length = dir.length();
    const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, length, 6), material);
    mesh.position.copy(p1).addScaledVector(dir, 0.5);
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize());
    return mesh;
}

// Delt kilde for bein-geometrien, brukt både av det visuelle nettet og av fysikken (bakkekontakt
// per fot) - slik at de to alltid stemmer overens.
const DRONE_ARM_LENGTH = 0.22;
function legLengthForClass(classKey) {
    return classKey === "cinematic" ? 0.24 : 0.12;
}
function getLegTopLocalPositions(armLength) {
    return [
        { x: armLength, z: -armLength },
        { x: -armLength, z: -armLength },
        { x: armLength, z: armLength },
        { x: -armLength, z: armLength }
    ].map(function (p) { return new THREE.Vector3(p.x, 0, p.z); });
}
function getLegFootLocalPositions(legLength, armLength) {
    return getLegTopLocalPositions(armLength).map(function (top) {
        const outward = new THREE.Vector2(top.x, top.z).normalize();
        return new THREE.Vector3(
            top.x + outward.x * legLength * 0.35,
            -legLength,
            top.z + outward.y * legLength * 0.35
        );
    });
}

// Landingsben festet ved hver arm/motor og ned til en fot lengre ute - ikke brukt på Racing
// (ekte racing-quader lander på understellet/motorene, uten egne ben).
function buildLandingLegs(legLength, armLength) {
    const group = new THREE.Group();
    const legMat = new THREE.MeshStandardMaterial({ color: 0x2a2a2a });
    const tops = getLegTopLocalPositions(armLength);
    const feet = getLegFootLocalPositions(legLength, armLength);
    tops.forEach(function (top, i) {
        const leg = buildStrutBetween(top, feet[i], 0.012, legMat);
        leg.castShadow = true;
        group.add(leg);
    });
    return group;
}

// Bygger droneen prosedyralt (X-ramme + 4 motorer/propeller) - ingen eksterne modellfiler.
// Utseendet varierer med drone-type: Racing får en spiss "canopy" og 3-bladet propell for et tøffere
// preg, Cinematic får en hengende gimbal-kule og lange, godt synlige landingsben.
function buildDrone(classKey) {
    const isRacing = classKey === "racing";
    const isCinematic = classKey === "cinematic";

    const group = new THREE.Group();
    const bodyMat = new THREE.MeshStandardMaterial({ color: 0x222222 });
    const armMat = new THREE.MeshStandardMaterial({ color: 0x444444 });
    const frontArmMat = new THREE.MeshStandardMaterial({ color: 0x992222 });

    const body = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.06, 0.16), bodyMat);
    body.castShadow = true;
    group.add(body);

    if (isRacing) {
        const canopy = new THREE.Mesh(new THREE.ConeGeometry(0.055, 0.09, 4), frontArmMat);
        canopy.rotation.x = -Math.PI / 2;
        canopy.rotation.y = Math.PI / 4;
        canopy.position.set(0, 0.04, -0.07);
        canopy.castShadow = true;
        group.add(canopy);
    }
    if (isCinematic) {
        const gimbal = new THREE.Mesh(new THREE.SphereGeometry(0.045, 10, 8), new THREE.MeshStandardMaterial({ color: 0x111111 }));
        gimbal.position.set(0, -0.06, -0.02);
        gimbal.castShadow = true;
        group.add(gimbal);
    }

    const armLength = DRONE_ARM_LENGTH;
    const armSpan = armLength * Math.SQRT2 * 2;
    const arm1 = new THREE.Mesh(new THREE.BoxGeometry(armSpan, 0.02, 0.03), armMat);
    arm1.rotation.y = Math.PI / 4;
    arm1.castShadow = true;
    const arm2 = new THREE.Mesh(new THREE.BoxGeometry(armSpan, 0.02, 0.03), armMat);
    arm2.rotation.y = -Math.PI / 4;
    arm2.castShadow = true;
    group.add(arm1, arm2);

    if (!isRacing) {
        group.add(buildLandingLegs(legLengthForClass(classKey), armLength));
    }

    // Forover er lokal -Z. Fremre motorer (z < 0) farges rødlig, som på en ekte FPV-quad.
    const motorOffsets = [
        { x: armLength, z: -armLength, dir: 1 },
        { x: -armLength, z: -armLength, dir: -1 },
        { x: armLength, z: armLength, dir: -1 },
        { x: -armLength, z: armLength, dir: 1 }
    ];
    const bladeCount = isRacing ? 3 : 2;
    const bladeLength = isCinematic ? 0.28 : 0.2;
    const props = [];
    motorOffsets.forEach(function (m) {
        const isFront = m.z < 0;
        const motor = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 0.04, 8), isFront ? frontArmMat : armMat);
        motor.position.set(m.x, 0.02, m.z);
        motor.castShadow = true;
        group.add(motor);

        const prop = buildPropeller(bladeCount, bladeLength);
        prop.position.set(m.x, 0.05, m.z);
        group.add(prop);
        props.push({ mesh: prop, spinDir: m.dir });
    });

    return { group: group, props: props };
}

function initScene() {
    const canvas = document.getElementById("simCanvas");
    renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    scene = new THREE.Scene();
    scene.add(buildGradientSky());
    scene.add(buildGround());
    scene.add(buildWorldObjects());

    scene.add(new THREE.AmbientLight(0xffffff, 0.55));
    const sun = new THREE.DirectionalLight(0xffffff, 0.9);
    sun.position.set(60, 90, 40);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    sun.shadow.camera.left = -80;
    sun.shadow.camera.right = 80;
    sun.shadow.camera.top = 80;
    sun.shadow.camera.bottom = -80;
    sun.shadow.camera.near = 1;
    sun.shadow.camera.far = 250;
    sun.shadow.bias = -0.0015;
    sun.target.position.set(0, 0, -40);
    scene.add(sun.target);
    scene.add(sun);

    const aspect = window.innerWidth / Math.max(1, window.innerHeight - 70);
    chaseCamera = new THREE.PerspectiveCamera(60, aspect, 0.1, 2000);
    fpvCamera = new THREE.PerspectiveCamera(95, aspect, 0.05, 2000);
    // Onboard FPV-kamera, montert litt foran/over senter. Vinkel er justerbar (Rates-panelet).
    fpvCamera.position.set(0, 0.06, -0.12);
    fpvCamera.rotation.x = THREE.MathUtils.degToRad(settings.fpvTiltDeg);

    rebuildDroneMesh();

    // VLOS: fast kamera litt bak avgangsplassen (som en pilot som står og ser etter droneen), følger ikke droneens bevegelse.
    vlosCamera = new THREE.PerspectiveCamera(50, aspect, 0.1, 2000);
    vlosCamera.position.set(0, 1.6, 5);
    scene.add(vlosCamera);

    activeCamera = chaseCamera;
    resizeRenderer();
}

// Drone-typene har ulik geometri (ben, canopy, propell-blad), ikke bare størrelse - bygger derfor
// hele modellen på nytt (i stedet for å bare skalere) hver gang typen velges/endres.
function rebuildDroneMesh() {
    if (droneGroup) scene.remove(droneGroup);
    const droneMesh = buildDrone(droneState.droneClass);
    droneGroup = droneMesh.group;
    dronePropellers = droneMesh.props;
    droneGroup.scale.setScalar(currentDroneSpec().visualScale);
    droneGroup.add(fpvCamera);
    scene.add(droneGroup);
}

function resizeRenderer() {
    const wrap = document.querySelector(".sim-page");
    const w = wrap.clientWidth, h = wrap.clientHeight;
    renderer.setSize(w, h, false);
    [chaseCamera, fpvCamera, vlosCamera].forEach(function (cam) {
        cam.aspect = w / h;
        cam.updateProjectionMatrix();
    });
}

function updateChaseCamera(dt) {
    // Følger kun med på yaw (heading), ikke full roll/pitch - gir en stabil, brukbar chase-cam
    // fremfor en kamera-rigg som er stivt låst til droneens fulle rotasjon.
    const euler = new THREE.Euler().setFromQuaternion(droneState.quaternion, "YXZ");
    const headingQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, euler.y, 0));
    const behindOffset = new THREE.Vector3(0, 1.2, 3.5).applyQuaternion(headingQuat);
    const desiredPos = droneState.position.clone().add(behindOffset);
    const smoothing = 1 - Math.pow(0.001, dt);
    chaseCamera.position.lerp(desiredPos, smoothing);
    chaseCamera.lookAt(droneState.position.clone().add(new THREE.Vector3(0, 0.3, 0)));
}

function updateVlosCamera() {
    // Fast plassert ved avgangsplassen - dreier kun for å følge droneen med blikket, som en pilot på bakken.
    vlosCamera.lookAt(droneState.position);
}

/* ---------- Input ---------- */
// Rå tilkoblingssjekk (uavhengig av inputkilde-valget) - brukes til å vise/skjule
// "Fjernkontroll"-knappen og for å fylle kalibreringspanelet, uansett hva brukeren har valgt å bruke.
function rawFirstGamepad() {
    const pads = navigator.getGamepads ? navigator.getGamepads() : [];
    for (let i = 0; i < pads.length; i++) {
        if (pads[i]) return pads[i];
    }
    return null;
}

// Inputkilde-bevisst valg - brukes for faktisk styring (updateInput/pollGamepadButtons).
function getActiveGamepad() {
    if (settings.inputSource === "keyboard") return null;
    const pads = navigator.getGamepads ? navigator.getGamepads() : [];
    if (settings.inputSource !== "auto") {
        const idx = parseInt(settings.inputSource, 10);
        if (pads[idx]) return pads[idx];
        // Valgt enhet er ikke lenger tilkoblet - fall tilbake til automatisk valg under.
    }
    return rawFirstGamepad();
}

function readStickAxis(gp, channelMap) {
    const raw = gp.axes[channelMap.axis] || 0;
    return clamp(raw * (channelMap.reverse ? -1 : 1), -1, 1);
}

function readThrottleAxis(gp, channelMap) {
    const raw = gp.axes[channelMap.axis] || 0;
    const signed = raw * (channelMap.reverse ? -1 : 1);
    return clamp((signed + 1) / 2, 0, 1);
}

/* ---------- Gamepad knappemapping (kill/arm + flymodus-brytere) ---------- */
// Fungerer med enhver sender i USB-joystick-modus (f.eks. Taranis/RadioMaster med EdgeTX/OpenTX) -
// disse eksponerer gimbaler som akser og brytere som knapper via standard HTML5 Gamepad API,
// helt uavhengig av merke/modell. Antall akser/knapper leses dynamisk fra enheten (se buildGamepadPanel).
// Bindinger lagres som { type:"button", index } eller { type:"axis", index, onValue, offValue } -
// mange sender-brytere (f.eks. en arm-bryter på kanal 6) kommer som en akse, ikke en HID-knapp,
// så læringsflyten sjekker begge deler og lagrer det som faktisk beveget seg.
let listeningForButtonAction = null;
let learnIgnoreButtons = new Set();
let learnAxisBaseline = [];
const prevActionActive = {};

function startListeningForButton(action) {
    listeningForButtonAction = action;
    learnIgnoreButtons = new Set();
    const gp = getActiveGamepad();
    learnAxisBaseline = gp ? gp.axes.slice() : [];
    if (gp) {
        for (let i = 0; i < gp.buttons.length; i++) {
            if (gp.buttons[i].pressed || gp.buttons[i].value > 0.5) learnIgnoreButtons.add(i);
        }
    }
}

const BUTTON_ACTIONS = {
    kill: toggleKill,
    modeAcro: function () { droneState.flightMode = "acro"; },
    modeStabilized: function () { droneState.flightMode = "stabilized"; },
    modeAltHold: function () { droneState.flightMode = "althold"; }
};

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

function pollGamepadButtons(gp) {
    if (listeningForButtonAction) {
        let captured = false;
        for (let i = 0; i < gp.buttons.length; i++) {
            const pressed = gp.buttons[i].pressed || gp.buttons[i].value > 0.5;
            if (pressed && !learnIgnoreButtons.has(i)) {
                gamepadMap.buttons[listeningForButtonAction] = { type: "button", index: i };
                captured = true;
                break;
            }
        }
        if (!captured) {
            for (let i = 0; i < gp.axes.length; i++) {
                const baseline = learnAxisBaseline[i] || 0;
                if (Math.abs(gp.axes[i] - baseline) > 0.25) {
                    gamepadMap.buttons[listeningForButtonAction] = { type: "axis", index: i, onValue: gp.axes[i], offValue: baseline };
                    captured = true;
                    break;
                }
            }
        }
        if (captured) {
            saveGamepadMap();
            listeningForButtonAction = null;
        }
    }

    Object.keys(BUTTON_ACTIONS).forEach(function (action) {
        const active = isBindingActive(gp, gamepadMap.buttons[action]);
        if (active && !prevActionActive[action]) BUTTON_ACTIONS[action]();
        prevActionActive[action] = active;
    });
}

function updateInput(dt) {
    updateLinkAndBattery(dt);

    const gp = getActiveGamepad();
    if (gp) pollGamepadButtons(gp);

    const dropChance = settings.realisticMode ? (1 - linkQuality) : 0;
    if (dropChance > 0 && Math.random() < dropChance) {
        // Simulerer tapt kontrollpakke pga svak/tapt link - pinnene beholder forrige verdi.
        updateGamepadAxesReadout(gp);
        return;
    }

    if (gp) {
        inputState.source = "gamepad";
        inputState.stick.roll = readStickAxis(gp, gamepadMap.roll);
        inputState.stick.pitch = readStickAxis(gp, gamepadMap.pitch);
        inputState.stick.yaw = readStickAxis(gp, gamepadMap.yaw);
        inputState.stick.throttle = readThrottleAxis(gp, gamepadMap.throttle);
        updateGamepadAxesReadout(gp);
        return;
    }
    inputState.source = "keyboard";
    const rollTarget = (keys.has("KeyD") ? 1 : 0) - (keys.has("KeyA") ? 1 : 0);
    const pitchTarget = (keys.has("KeyW") ? 1 : 0) - (keys.has("KeyS") ? 1 : 0);
    const yawTarget = (keys.has("KeyE") ? 1 : 0) - (keys.has("KeyQ") ? 1 : 0);
    inputState.stick.roll = rampStick(inputState.stick.roll, rollTarget, dt);
    inputState.stick.pitch = rampStick(inputState.stick.pitch, pitchTarget, dt);
    inputState.stick.yaw = rampStick(inputState.stick.yaw, yawTarget, dt);

    let throttle = inputState.stick.throttle;
    if (keys.has("ShiftLeft") || keys.has("ShiftRight")) throttle += THROTTLE_RATE * dt;
    if (keys.has("ControlLeft") || keys.has("ControlRight")) throttle -= THROTTLE_RATE * dt;
    inputState.stick.throttle = clamp(throttle, 0, 1);
    updateGamepadAxesReadout(null);
}

// Batteri + linkkvalitet (avstand fra avgangsplassen + siktlinje mot bygget). Kun aktivt i Realistisk modus.
function updateLinkAndBattery(dt) {
    if (!settings.realisticMode) {
        droneState.batteryPercent = 100;
        linkQuality = 1;
        return;
    }
    if (droneState.armed) {
        const drain = BATTERY_DRAIN_IDLE + (BATTERY_DRAIN_FULL - BATTERY_DRAIN_IDLE) * inputState.stick.throttle;
        droneState.batteryPercent = clamp(droneState.batteryPercent - drain * dt, 0, 100);
    }

    const dist = droneState.position.distanceTo(LAUNCH_POINT);
    let quality = 1;
    if (dist > LINK_RANGE_FULL) {
        quality = clamp(1 - (dist - LINK_RANGE_FULL) / (LINK_RANGE_ZERO - LINK_RANGE_FULL), 0, 1);
    }
    if (isLineBlockedByBuilding(LAUNCH_POINT, droneState.position)) {
        quality *= LINK_OBSTRUCTION_PENALTY;
    }
    linkQuality = quality;
}

/* ---------- Fysikk ---------- */
function stepPhysics(dt) {
    const spec = currentDroneSpec();
    const stick = inputState.stick;
    let thrustForce = 0;
    const desiredRateDeg = { roll: 0, pitch: 0, yaw: 0 };
    const wasGrounded = droneState.grounded; // fra forrige fysikk-tick - avgjør om vind skal virke nå

    const throttleShaped = computeThrottleCurve(stick.throttle, rates.throttle.expo);

    if (droneState.armed) {
        if (droneState.flightMode === "acro") {
            desiredRateDeg.roll = computeRate(stick.roll, rates.roll);
            desiredRateDeg.pitch = computeRate(stick.pitch, rates.pitch);
            desiredRateDeg.yaw = computeRate(stick.yaw, rates.yaw);
            thrustForce = throttleShaped * spec.maxThrust;
        } else {
            // Stabilized / Alt Hold: selvnivellerende ytre lookk for roll/pitch.
            // (euler.x/euler.z negeres - se merknad om aksekonvensjon lenger ned.)
            const euler = new THREE.Euler().setFromQuaternion(droneState.quaternion, "YXZ");
            const currentPitchDeg = -THREE.MathUtils.radToDeg(euler.x);
            const currentRollDeg = -THREE.MathUtils.radToDeg(euler.z);
            const desiredPitchAngle = stick.pitch * MAX_SELF_LEVEL_ANGLE;
            const desiredRollAngle = stick.roll * MAX_SELF_LEVEL_ANGLE;
            desiredRateDeg.pitch = ANGLE_P_GAIN * (desiredPitchAngle - currentPitchDeg);
            desiredRateDeg.roll = ANGLE_P_GAIN * (desiredRollAngle - currentRollDeg);
            desiredRateDeg.yaw = computeRate(stick.yaw, rates.yaw);

            if (droneState.flightMode === "althold") {
                // Gass rundt 50% (innenfor dødsone) holder høyden; utenfor justeres ønsket stigefart proporsjonalt.
                const centered = stick.throttle - 0.5;
                const magnitude = Math.abs(centered);
                let climbInput = 0;
                if (magnitude > ALT_HOLD_DEADBAND) {
                    climbInput = Math.sign(centered) * (magnitude - ALT_HOLD_DEADBAND) / (0.5 - ALT_HOLD_DEADBAND);
                }
                const desiredClimbRate = climbInput * MAX_CLIMB_RATE;
                const climbError = desiredClimbRate - droneState.velocity.y;
                thrustForce = spec.mass * GRAVITY + ALT_GAIN * climbError;
            } else {
                thrustForce = throttleShaped * spec.maxThrust;
            }
        }
        if (settings.realisticMode && droneState.batteryPercent < BATTERY_LOW_THRESHOLD) {
            // Spenningsfall ved lavt batteri: makstrekk svekkes lineært ned mot 50 % ved 0 %.
            thrustForce *= 0.5 + 0.5 * (droneState.batteryPercent / BATTERY_LOW_THRESHOLD);
        }
        thrustForce = clamp(thrustForce, 0, spec.maxThrust);

        const desiredRateRad = {
            roll: THREE.MathUtils.degToRad(desiredRateDeg.roll),
            pitch: THREE.MathUtils.degToRad(desiredRateDeg.pitch),
            yaw: THREE.MathUtils.degToRad(desiredRateDeg.yaw)
        };
        // Three.js' aksekonvensjon (forward = -Z) gir motsatt rotasjonsfortegn av "pinne-intuisjonen"
        // (pinne+ = nese ned / rull høyre / sving høyre) for alle tre aksene - derfor negeres her.
        // Vinkelakselerasjon = moment / treghet: tyngre/større droner (høyere treghet) responderer
        // tregere per akse, og yaw har alltid høyere treghet enn roll/pitch (som på en ekte quad).
        const pitchAccel = TORQUE_GAIN * (-desiredRateRad.pitch - droneState.angularVelocity.pitch) / spec.inertiaRollPitch;
        const rollAccel = TORQUE_GAIN * (-desiredRateRad.roll - droneState.angularVelocity.roll) / spec.inertiaRollPitch;
        const yawAccel = TORQUE_GAIN * (-desiredRateRad.yaw - droneState.angularVelocity.yaw) / spec.inertiaYaw;
        droneState.angularVelocity.pitch += pitchAccel * dt;
        droneState.angularVelocity.roll += rollAccel * dt;
        droneState.angularVelocity.yaw += yawAccel * dt;
    } else {
        // Killswitch: ingen aktiv kontroll - fritt fall og tumling med passiv demping.
        thrustForce = 0;
        droneState.angularVelocity.pitch *= PASSIVE_ANGULAR_DAMPING;
        droneState.angularVelocity.roll *= PASSIVE_ANGULAR_DAMPING;
        droneState.angularVelocity.yaw *= PASSIVE_ANGULAR_DAMPING;
    }

    const up = new THREE.Vector3(0, 1, 0).applyQuaternion(droneState.quaternion);
    const thrustVec = up.multiplyScalar(thrustForce);
    const gravityVec = new THREE.Vector3(0, -spec.mass * GRAVITY, 0);
    // Luftmotstand virker mot hastigheten relativt til luften (vind), ikke bakken - en drone driver
    // derfor nedover vinden over tid, akkurat som i virkeligheten, og må styres imot for å holde posisjon.
    // Når den står på bakken skal den derimot IKKE drifte med vinden (beina/friksjonen holder den i ro).
    const airRelativeVelocity = wasGrounded
        ? droneState.velocity.clone()
        : droneState.velocity.clone().sub(currentWindVector);
    const dragVec = airRelativeVelocity.multiplyScalar(-spec.linearDrag);
    const accel = new THREE.Vector3().add(thrustVec).add(gravityVec).add(dragVec).multiplyScalar(1 / spec.mass);

    droneState.velocity.add(accel.clone().multiplyScalar(dt));
    droneState.position.add(droneState.velocity.clone().multiplyScalar(dt));
    const pushedFromWall = pushOutOfSolidWalls(droneState.position, droneState.velocity);

    const angVelVec = new THREE.Vector3(droneState.angularVelocity.pitch, droneState.angularVelocity.yaw, droneState.angularVelocity.roll);
    integrateOrientation(droneState.quaternion, angVelVec, dt);

    droneState.grounded = false;
    if (droneHasLandingLegs(droneState.droneClass)) {
        resolveLegGroundContact(dt);
    } else {
        // Nettopp dyttet ut av en veggside denne rammen (dvs. ikke i ferd med å lande ovenfra) - se bort fra
        // objektets topp-flate for bakkesjekken under, ellers ville den blitt tolket som en landing på taket
        // og loftet brått opp dit. Den flate bakken (0) skal likevel fanges opp som vanlig.
        const surfaceY = (pushedFromWall ? 0 : solidSurfaceHeightAt(droneState.position.x, droneState.position.z)) + GROUND_CLEARANCE;
        if (droneState.position.y <= surfaceY) {
            droneState.grounded = true;
            droneState.position.y = surfaceY;
            if (droneState.velocity.y < 0) droneState.velocity.y = 0;
            droneState.velocity.x *= 0.9;
            droneState.velocity.z *= 0.9;
            droneState.angularVelocity.pitch *= 0.8;
            droneState.angularVelocity.roll *= 0.8;
            droneState.angularVelocity.yaw *= 0.8;
        }
    }
}

function droneHasLandingLegs(classKey) {
    return classKey !== "racing";
}

function getFootWorldPositions() {
    const spec = currentDroneSpec();
    const legLength = legLengthForClass(droneState.droneClass);
    const feet = getLegFootLocalPositions(legLength, DRONE_ARM_LENGTH);
    return feet.map(function (foot) {
        return foot.clone().multiplyScalar(spec.visualScale)
            .applyQuaternion(droneState.quaternion)
            .add(droneState.position);
    });
}

// Bakkekontakt via bena (i stedet for kun ett punkt i senter): kan lande stabilt på bena innenfor
// LEG_TIP_RECOVERY_ANGLE_DEG (fjæring/demping retter den opp), eller fortsette å tippe over ved en
// hardere/skjevere landing eller hvis massesenteret ikke er støttet på alle sider (f.eks. ben som
// henger utenfor kanten av et tak) - da lar vi tyngdekraften velte den mot den usupporterte siden
// i stedet for å late som den står stødig.
const LEG_TIP_RECOVERY_ANGLE_DEG = 25;
const LEG_CONTACT_RIGHTING_RATE = 6;
const LEG_CONTACT_TOLERANCE = 0.06; // m - hvor nær bakken en fot må være for å regnes som støttet
const LEG_TIP_TORQUE = 3.0; // vinkelakselerasjon som velter droneen mot usupportert side

function resolveLegGroundContact(dt) {
    // Rekkefølge fra getLegTopLocalPositions: 0=fremre-høyre, 1=fremre-venstre, 2=bakre-høyre, 3=bakre-venstre.
    const feet = getFootWorldPositions();
    let maxPenetration = 0;
    // Når droneen har tippet langt nok kan et ben svinge horisontalt innunder kanten av et tak (f.eks.)
    // i stedet for å henge fritt utenfor - solidSurfaceHeightAt() ser da bare en "dyp landing" der (fordi
    // den kun kjenner en topp-flate per søyle, ingen ekte veggside), og bena ville blitt lest som støttet
    // med et voldsomt oppløft som resultat. Fanger opp dette per fot og dytter kroppen ut av veggen i
    // stedet, slik at foten IKKE telles som støttet.
    let wallPush = null;
    const grounded = feet.map(function (f) {
        let embeddedInWall = false;
        SOLID_COLLIDERS.forEach(function (c) {
            if (f.x < c.minX || f.x > c.maxX || f.z < c.minZ || f.z > c.maxZ) return;
            if (f.y >= c.topY - GROUND_CLEARANCE) return;
            embeddedInWall = true;
            const distMinX = f.x - c.minX, distMaxX = c.maxX - f.x, distMinZ = f.z - c.minZ, distMaxZ = c.maxZ - f.z;
            const minDist = Math.min(distMinX, distMaxX, distMinZ, distMaxZ);
            if (!wallPush || minDist < wallPush.dist) {
                if (minDist === distMinX) wallPush = { dist: minDist, axis: "x", delta: c.minX - f.x };
                else if (minDist === distMaxX) wallPush = { dist: minDist, axis: "x", delta: c.maxX - f.x };
                else if (minDist === distMinZ) wallPush = { dist: minDist, axis: "z", delta: c.minZ - f.z };
                else wallPush = { dist: minDist, axis: "z", delta: c.maxZ - f.z };
            }
        });
        if (embeddedInWall) return false;
        const groundY = solidSurfaceHeightAt(f.x, f.z);
        const penetration = groundY - f.y;
        if (penetration > maxPenetration) maxPenetration = penetration;
        return penetration > -LEG_CONTACT_TOLERANCE;
    });

    if (wallPush) {
        if (wallPush.axis === "x") {
            droneState.position.x += wallPush.delta;
            if ((wallPush.delta > 0 && droneState.velocity.x < 0) || (wallPush.delta < 0 && droneState.velocity.x > 0)) droneState.velocity.x = 0;
        } else {
            droneState.position.z += wallPush.delta;
            if ((wallPush.delta > 0 && droneState.velocity.z < 0) || (wallPush.delta < 0 && droneState.velocity.z > 0)) droneState.velocity.z = 0;
        }
    }

    if (maxPenetration <= 0) return;

    const rightSupported = grounded[0] || grounded[2];
    const leftSupported = grounded[1] || grounded[3];
    const frontSupported = grounded[0] || grounded[1];
    const backSupported = grounded[2] || grounded[3];
    const wellSupported = rightSupported && leftSupported && frontSupported && backSupported;

    const euler = new THREE.Euler().setFromQuaternion(droneState.quaternion, "YXZ");
    const pitchDeg = Math.abs(-THREE.MathUtils.radToDeg(euler.x));
    const rollDeg = Math.abs(-THREE.MathUtils.radToDeg(euler.z));
    const tippedPastRecovery = Math.max(pitchDeg, rollDeg) >= LEG_TIP_RECOVERY_ANGLE_DEG;

    if (!wellSupported && tippedPastRecovery) {
        // For tippet til at bena skal "fange den opp" igjen (f.eks. rett før den velter av en takkant) -
        // la tyngdekraften fullføre velten og droneen falle fritt videre, i stedet for at
        // posisjonskorreksjonen under kunstig løfter kroppen opp på nytt hver eneste frame
        // (det ga tidligere en evig vippende/oscillerende drone som aldri faktisk falt av kanten).
        if (!rightSupported) droneState.angularVelocity.roll -= LEG_TIP_TORQUE * dt;
        if (!leftSupported) droneState.angularVelocity.roll += LEG_TIP_TORQUE * dt;
        if (!frontSupported) droneState.angularVelocity.pitch -= LEG_TIP_TORQUE * dt;
        if (!backSupported) droneState.angularVelocity.pitch += LEG_TIP_TORQUE * dt;
        return;
    }

    droneState.grounded = true;
    droneState.position.y += maxPenetration;
    if (droneState.velocity.y < 0) droneState.velocity.y *= 0.2; // myk/dempet landing (fjæring i bena)
    droneState.velocity.x *= 0.9;
    droneState.velocity.z *= 0.9;

    if (!wellSupported) {
        // Massesenteret mangler støtte på én eller flere sider, men innenfor gjenopprettbar helning ennå -
        // tyngdekraften begynner å velte den den veien (f.eks. når noen av bena lander utenfor kanten av et tak).
        if (!rightSupported) droneState.angularVelocity.roll -= LEG_TIP_TORQUE * dt;
        if (!leftSupported) droneState.angularVelocity.roll += LEG_TIP_TORQUE * dt;
        if (!frontSupported) droneState.angularVelocity.pitch -= LEG_TIP_TORQUE * dt;
        if (!backSupported) droneState.angularVelocity.pitch += LEG_TIP_TORQUE * dt;
    } else if (!tippedPastRecovery) {
        // Godt støttet og innenfor gjenopprettbar helning - bena "retter opp" droneen igjen.
        droneState.angularVelocity.pitch *= 0.5;
        droneState.angularVelocity.roll *= 0.5;
        const uprightQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, euler.y, 0, "YXZ"));
        droneState.quaternion.slerp(uprightQuat, Math.min(1, LEG_CONTACT_RIGHTING_RATE * dt));
    } else {
        // Godt støttet, men for skjev vinkel til å bli "rettet opp" - la den fortsette å tippe.
        droneState.angularVelocity.pitch *= 0.85;
        droneState.angularVelocity.roll *= 0.85;
    }
    droneState.angularVelocity.yaw *= 0.8;
}

function resetDrone() {
    droneState.position.set(0, 1.0, 0);
    droneState.velocity.set(0, 0, 0);
    droneState.quaternion.identity();
    droneState.angularVelocity.pitch = 0;
    droneState.angularVelocity.roll = 0;
    droneState.angularVelocity.yaw = 0;
    droneState.armed = true;
    // Gass er en ikke-selvsentrerende "holdt" verdi på tastatur - må nullstilles eksplisitt ved reset,
    // ellers tar droneen av igjen umiddelbart. (Gamepad overskriver denne uansett neste frame.)
    inputState.stick.throttle = 0;
    droneState.batteryPercent = 100;
}

function toggleKill() {
    droneState.armed = !droneState.armed;
}

function toggleCamera() {
    cameraModeIndex = (cameraModeIndex + 1) % CAMERA_MODES.length;
    const mode = CAMERA_MODES[cameraModeIndex];
    activeCamera = (mode === "chase") ? chaseCamera : (mode === "fpv") ? fpvCamera : vlosCamera;
}

/* ---------- Visuell oppdatering + HUD ---------- */
function updateDroneVisual(dt) {
    droneGroup.position.copy(droneState.position);
    droneGroup.quaternion.copy(droneState.quaternion);
    const spinSpeed = droneState.armed ? (4 + inputState.stick.throttle * 60) : 0;
    dronePropellers.forEach(function (p) {
        p.mesh.rotation.y += p.spinDir * spinSpeed * dt;
    });
}

const hudMode = document.getElementById("hudMode");
const hudArmed = document.getElementById("hudArmed");
const hudInput = document.getElementById("hudInput");
const hudCamera = document.getElementById("hudCamera");
const hudDroneClass = document.getElementById("hudDroneClass");
const hudAltitude = document.getElementById("hudAltitude");
const hudThrottle = document.getElementById("hudThrottle");
const armToggleBtn = document.getElementById("armToggleBtn");
const hudBatteryItem = document.getElementById("hudBatteryItem");
const hudLinkItem = document.getElementById("hudLinkItem");
const hudBattery = document.getElementById("hudBattery");
const hudLink = document.getElementById("hudLink");

function updateHud() {
    hudMode.textContent = MODE_LABELS[droneState.flightMode];
    hudArmed.textContent = droneState.armed ? "Armed" : "Killed";
    hudArmed.className = "sim-status-value " + (droneState.armed ? "sim-armed" : "sim-killed");
    hudInput.textContent = inputState.source === "gamepad" ? "Gamepad" : "Tastatur";
    hudCamera.textContent = CAMERA_MODE_LABELS[CAMERA_MODES[cameraModeIndex]];
    hudDroneClass.textContent = currentDroneSpec().label.split(" ")[0];
    const altitude = droneState.position.y;
    hudAltitude.textContent = (altitude >= 10 ? Math.round(altitude) : altitude.toFixed(1)) + " m";
    hudThrottle.textContent = Math.round(inputState.stick.throttle * 100) + " %";
    armToggleBtn.innerHTML = '<i class="fa-solid fa-power-off"></i> ' + (droneState.armed ? "Disarm (K)" : "Arm (K)");

    if (settings.realisticMode) {
        hudBatteryItem.style.display = "";
        hudLinkItem.style.display = "";
        hudBattery.textContent = Math.round(droneState.batteryPercent) + " %";
        hudLink.textContent = Math.round(linkQuality * 100) + " %";
        const linkClass = linkQuality < 0.15 ? "sim-killed" : (linkQuality < 0.5 ? "" : "sim-armed");
        hudLink.className = "sim-status-value " + linkClass;
        hudBattery.className = "sim-status-value " + (droneState.batteryPercent < BATTERY_LOW_THRESHOLD ? "sim-killed" : "");
    } else {
        hudBatteryItem.style.display = "none";
        hudLinkItem.style.display = "none";
    }
}

/* ---------- Paneler (rates / gamepad / hjelp) ---------- */
function togglePanel(panel) {
    const wasOpen = panel.style.display !== "none";
    ["ratesPanel", "gamepadPanel", "helpPanel"].forEach(function (id) {
        document.getElementById(id).style.display = "none";
    });
    panel.style.display = wasOpen ? "none" : "block";
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

function buildRatesPanel() {
    const grid = document.getElementById("ratesGrid");
    grid.innerHTML = "";
    ["roll", "pitch", "yaw"].forEach(function (axis) {
        const axisRates = rates[axis];
        const box = document.createElement("div");
        box.className = "sim-rate-axis";
        const title = document.createElement("div");
        title.className = "sim-rate-axis-title";
        title.textContent = AXIS_LABELS[axis];
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
                saveRates();
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
            saveRates();
            redraw();
        });
        function stopDrag() { dragging = null; }
        canvas.addEventListener("pointerup", stopDrag);
        canvas.addEventListener("pointercancel", stopDrag);

        redraw();
        grid.appendChild(box);
    });

    const throttleBox = document.createElement("div");
    throttleBox.className = "sim-rate-axis";
    const throttleTitle = document.createElement("div");
    throttleTitle.className = "sim-rate-axis-title";
    throttleTitle.textContent = "Gass";
    throttleBox.appendChild(throttleTitle);

    const throttleRow = document.createElement("div");
    throttleRow.className = "sim-rate-row";
    const throttleLabel = document.createElement("label");
    throttleLabel.textContent = "Expo";
    const throttleInput = document.createElement("input");
    throttleInput.type = "range";
    throttleInput.min = 0;
    throttleInput.max = 1;
    throttleInput.step = 0.05;
    throttleInput.value = rates.throttle.expo;
    const throttleValueSpan = document.createElement("span");
    throttleValueSpan.className = "sim-rate-value";
    throttleValueSpan.textContent = rates.throttle.expo;
    throttleInput.addEventListener("input", function () {
        rates.throttle.expo = parseFloat(throttleInput.value);
        throttleValueSpan.textContent = throttleInput.value;
        saveRates();
    });
    throttleRow.appendChild(throttleLabel);
    throttleRow.appendChild(throttleInput);
    throttleRow.appendChild(throttleValueSpan);
    throttleBox.appendChild(throttleRow);

    const throttleHint = document.createElement("p");
    throttleHint.className = "sim-panel-hint";
    throttleHint.style.margin = "6px 0 0 0";
    throttleHint.textContent = "0 = lineær gass. Høyere verdi gir finere kontroll nær midten (rundt hover), mer kraftfull respons ved fullt utslag.";
    throttleBox.appendChild(throttleHint);

    grid.appendChild(throttleBox);
}

function populateInputSourceSelect() {
    const select = document.getElementById("inputSourceSelect");
    if (!select) return;
    const pads = navigator.getGamepads ? navigator.getGamepads() : [];
    select.innerHTML = "";

    const autoOpt = document.createElement("option");
    autoOpt.value = "auto";
    autoOpt.textContent = "Automatisk (første tilkoblede)";
    select.appendChild(autoOpt);

    const kbOpt = document.createElement("option");
    kbOpt.value = "keyboard";
    kbOpt.textContent = "Tastatur";
    select.appendChild(kbOpt);

    for (let i = 0; i < pads.length; i++) {
        if (!pads[i]) continue;
        const opt = document.createElement("option");
        opt.value = String(i);
        opt.textContent = "Gamepad " + i + ": " + (pads[i].id || "ukjent enhet");
        select.appendChild(opt);
    }

    select.value = settings.inputSource;
    if (select.value !== settings.inputSource) {
        // Lagret valg finnes ikke lenger (enheten er frakoblet) - fall tilbake til automatisk.
        settings.inputSource = "auto";
        saveSettings();
        select.value = "auto";
    }
}

function buildGamepadPanel(pad) {
    populateInputSourceSelect();
    const grid = document.getElementById("gamepadGrid");
    grid.innerHTML = "";
    // RC-sendere har typisk 8 kanaler selv om enheten (ennå) ikke rapporterer full akse-lengde -
    // vis alltid minst 8 valg, i tillegg til flere hvis enheten faktisk har det.
    const axisCount = Math.max(pad.axes.length, MIN_GAMEPAD_CHANNELS);
    ["roll", "pitch", "yaw", "throttle"].forEach(function (channel) {
        const row = document.createElement("div");
        row.className = "sim-rate-row";
        const label = document.createElement("label");
        label.textContent = CHANNEL_LABELS[channel];
        const select = document.createElement("select");
        for (let i = 0; i < axisCount; i++) {
            const opt = document.createElement("option");
            opt.value = i;
            opt.textContent = "Kanal " + (i + 1);
            if (gamepadMap[channel].axis === i) opt.selected = true;
            select.appendChild(opt);
        }
        select.addEventListener("change", function () {
            gamepadMap[channel].axis = parseInt(select.value, 10);
            saveGamepadMap();
        });

        const reverseLabel = document.createElement("label");
        reverseLabel.style.cssText = "display:flex; align-items:center; gap:4px; flex:0 0 60px; font-size:0.75rem;";
        const reverseInput = document.createElement("input");
        reverseInput.type = "checkbox";
        reverseInput.checked = gamepadMap[channel].reverse;
        reverseInput.addEventListener("change", function () {
            gamepadMap[channel].reverse = reverseInput.checked;
            saveGamepadMap();
        });
        reverseLabel.appendChild(reverseInput);
        reverseLabel.appendChild(document.createTextNode("Rev."));

        row.appendChild(label);
        row.appendChild(select);
        row.appendChild(reverseLabel);
        grid.appendChild(row);
    });

    buildGamepadButtonsPanel();
}

function buildGamepadButtonsPanel() {
    const container = document.getElementById("gamepadButtonsGrid");
    container.innerHTML = "";
    Object.keys(BUTTON_ACTION_LABELS).forEach(function (action) {
        const row = document.createElement("div");
        row.className = "sim-rate-row";
        const label = document.createElement("label");
        label.textContent = BUTTON_ACTION_LABELS[action];
        const statusSpan = document.createElement("span");
        statusSpan.className = "sim-rate-value";
        statusSpan.style.cssText = "flex:1; text-align:left;";
        function refreshStatus() {
            const b = gamepadMap.buttons[action];
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
            startListeningForButton(action);
            const checkDone = setInterval(function () {
                if (listeningForButtonAction !== action) {
                    setBtn.textContent = "Sett";
                    refreshStatus();
                    clearInterval(checkDone);
                }
            }, 150);
        });

        const clearBtn = document.createElement("button");
        clearBtn.type = "button";
        clearBtn.className = "sim-btn";
        clearBtn.textContent = "Fjern";
        clearBtn.addEventListener("click", function () {
            gamepadMap.buttons[action] = null;
            saveGamepadMap();
            refreshStatus();
        });

        row.appendChild(label);
        row.appendChild(statusSpan);
        row.appendChild(setBtn);
        row.appendChild(clearBtn);
        container.appendChild(row);
    });
}

function updateGamepadAxesReadout(gp) {
    const panel = document.getElementById("gamepadPanel");
    if (panel.style.display === "none") return;
    const readout = document.getElementById("gamepadAxesReadout");
    const activeGp = gp || getActiveGamepad();
    if (!activeGp) {
        readout.textContent = "Ingen fjernkontroll/gamepad tilkoblet.";
        return;
    }
    readout.innerHTML = "";
    const channelCount = Math.max(activeGp.axes.length, MIN_GAMEPAD_CHANNELS);
    for (let i = 0; i < channelCount; i++) {
        const v = activeGp.axes[i];
        const line = document.createElement("div");
        line.textContent = "Kanal " + (i + 1) + ": " + (v === undefined ? "–" : v.toFixed(2));
        readout.appendChild(line);
    }
}

function setGamepadButtonVisible(visible) {
    document.getElementById("toggleGamepadBtn").style.display = visible ? "" : "none";
    if (!visible) document.getElementById("gamepadPanel").style.display = "none";
}

/* ---------- Videoforstyrrelse (FPV-signal i Realistisk modus) ---------- */
let noiseCtx = null;
let lastNoiseUpdate = 0;

function initNoiseCanvas() {
    const canvas = document.getElementById("signalNoiseCanvas");
    canvas.width = 96;
    canvas.height = 96;
    noiseCtx = canvas.getContext("2d");
}

function drawNoiseFrame() {
    const imgData = noiseCtx.createImageData(96, 96);
    for (let i = 0; i < imgData.data.length; i += 4) {
        const v = Math.random() * 255;
        imgData.data[i] = v;
        imgData.data[i + 1] = v;
        imgData.data[i + 2] = v;
        imgData.data[i + 3] = 255;
    }
    noiseCtx.putImageData(imgData, 0, 0);
}

function updateSignalOverlay(now) {
    const overlay = document.getElementById("signalNoiseCanvas");
    if (!settings.realisticMode || activeCamera !== fpvCamera || linkQuality > 0.9) {
        overlay.style.opacity = 0;
        overlay.style.background = "";
        return;
    }
    const badness = 1 - linkQuality;
    if (now - lastNoiseUpdate > 80) {
        drawNoiseFrame();
        lastNoiseUpdate = now;
    }
    overlay.style.opacity = Math.min(0.85, badness * 1.1);
    overlay.style.background = (linkQuality < 0.08 && Math.random() < 0.6) ? "#000" : "";
}

/* ---------- FPV HUD/OSD (crosshair / kunstig horisont) ---------- */
let fpvHudCtx = null;
let fpvHudModeIndex = 0;

function initFpvHudCanvas() {
    const canvas = document.getElementById("fpvHudCanvas");
    canvas.width = 400;
    canvas.height = 300;
    fpvHudCtx = canvas.getContext("2d");
    fpvHudModeIndex = Math.max(0, FPV_HUD_MODES.indexOf(settings.fpvHudMode));
}

function toggleFpvHud() {
    fpvHudModeIndex = (fpvHudModeIndex + 1) % FPV_HUD_MODES.length;
    settings.fpvHudMode = FPV_HUD_MODES[fpvHudModeIndex];
    saveSettings();
    const btn = document.getElementById("fpvHudBtn");
    btn.innerHTML = '<i class="fa-solid fa-crosshairs"></i> OSD: ' + FPV_HUD_MODE_LABELS[settings.fpvHudMode] + " (O)";
}

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
function drawFpvHorizon(ctx, w, h) {
    const euler = new THREE.Euler().setFromQuaternion(droneState.quaternion, "YXZ");
    // Samme aksekonvensjon-negasjon som resten av fysikken (se merknad i stepPhysics).
    const pitchDeg = -THREE.MathUtils.radToDeg(euler.x);
    const rollDeg = -THREE.MathUtils.radToDeg(euler.z);
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

function updateFpvHud() {
    const canvas = document.getElementById("fpvHudCanvas");
    const mode = FPV_HUD_MODES[fpvHudModeIndex];
    if (activeCamera !== fpvCamera || mode === "none") {
        canvas.style.display = "none";
        return;
    }
    canvas.style.display = "block";
    const w = canvas.width, h = canvas.height;
    fpvHudCtx.clearRect(0, 0, w, h);
    if (mode === "horizon") drawFpvHorizon(fpvHudCtx, w, h);
    drawFpvCrosshair(fpvHudCtx, w, h);
}

// Oppdaterer vindpølsens retning (peker nedvinds), "slapphet" (henger rett ned ved 0 vind, reiser seg
// mot vannrett med økende styrke) og en myk bølge langs selve geometrien for et levende, blafrende
// uttrykk - uten å bryte opp den jevnt avsmalnende formen i separate (hakkete) segmenter.
function updateWindsockVisual(now) {
    if (!windsockYawGroup || !windsockDroopPivot) return;
    const speed = currentWindVector.length();
    if (speed > 0.05) {
        windsockYawGroup.rotation.y = Math.atan2(-currentWindVector.z, currentWindVector.x);
    }
    const strength = clamp(speed / 12, 0, 1);
    windsockDroopPivot.rotation.z = -Math.PI / 2 * (1 - strength);

    if (windsockSockGeometry) {
        const t = now * 0.001;
        const amp = 0.01 + strength * 0.06;
        const posAttr = windsockSockGeometry.attributes.position;
        const halfLen = windsockSockLength / 2;
        for (let i = 0; i < posAttr.count; i++) {
            const baseY = windsockBasePositions[i * 3 + 1];
            const baseZ = windsockBasePositions[i * 3 + 2];
            const alongTail = clamp((baseY + halfLen) / windsockSockLength, 0, 1); // 0=munning, 1=hale
            const wave = Math.sin(t * 3.2 + alongTail * 7) * amp * alongTail * alongTail;
            posAttr.setZ(i, baseZ + wave);
        }
        posAttr.needsUpdate = true;
        windsockSockGeometry.computeVertexNormals();
    }
}

/* ---------- Hovedløkke ---------- */
let lastTime = performance.now();
let accumulator = 0;

function animate(now) {
    requestAnimationFrame(animate);
    const frameDt = Math.min((now - lastTime) / 1000, 0.1);
    lastTime = now;
    accumulator += frameDt;

    updateWind(frameDt);
    updateInput(frameDt);
    while (accumulator >= FIXED_DT) {
        stepPhysics(FIXED_DT);
        accumulator -= FIXED_DT;
    }

    updateDroneVisual(frameDt);
    updateChaseCamera(frameDt);
    updateVlosCamera();
    updateWindsockVisual(now);
    updateHud();
    updateSignalOverlay(now);
    updateFpvHud();
    renderer.render(scene, activeCamera);
}

/* ---------- Oppstart ---------- */
document.addEventListener("DOMContentLoaded", function () {
    initScene();
    initNoiseCanvas();
    initFpvHudCanvas();
    document.getElementById("fpvHudBtn").innerHTML =
        '<i class="fa-solid fa-crosshairs"></i> OSD: ' + FPV_HUD_MODE_LABELS[settings.fpvHudMode] + " (O)";
    buildRatesPanel();

    document.getElementById("resetBtn").addEventListener("click", resetDrone);
    document.getElementById("armToggleBtn").addEventListener("click", toggleKill);
    document.getElementById("toggleRatesBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("ratesPanel"));
    });
    document.getElementById("toggleHelpBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("helpPanel"));
    });
    document.getElementById("toggleGamepadBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("gamepadPanel"));
    });
    document.getElementById("fpvHudBtn").addEventListener("click", toggleFpvHud);

    const droneClassSelect = document.getElementById("droneClassSelect");
    Object.keys(DRONE_CLASSES).forEach(function (key) {
        const opt = document.createElement("option");
        opt.value = key;
        opt.textContent = DRONE_CLASSES[key].label;
        if (key === droneState.droneClass) opt.selected = true;
        droneClassSelect.appendChild(opt);
    });
    droneClassSelect.addEventListener("change", function () {
        setDroneClass(droneClassSelect.value);
    });

    const fpvTiltInput = document.getElementById("fpvTiltInput");
    const fpvTiltValue = document.getElementById("fpvTiltValue");
    fpvTiltInput.value = settings.fpvTiltDeg;
    fpvTiltValue.textContent = settings.fpvTiltDeg;
    fpvTiltInput.addEventListener("input", function () {
        settings.fpvTiltDeg = parseFloat(fpvTiltInput.value);
        fpvTiltValue.textContent = fpvTiltInput.value;
        fpvCamera.rotation.x = THREE.MathUtils.degToRad(settings.fpvTiltDeg);
        saveSettings();
    });

    const realisticModeInput = document.getElementById("realisticModeInput");
    realisticModeInput.checked = settings.realisticMode;
    realisticModeInput.addEventListener("change", function () {
        settings.realisticMode = realisticModeInput.checked;
        saveSettings();
    });

    const windEnabledInput = document.getElementById("windEnabledInput");
    const windSpeedInput = document.getElementById("windSpeedInput");
    const windSpeedValue = document.getElementById("windSpeedValue");
    const windDirectionInput = document.getElementById("windDirectionInput");
    const windDirectionValue = document.getElementById("windDirectionValue");
    const windGustInput = document.getElementById("windGustInput");
    const windGustValue = document.getElementById("windGustValue");
    windEnabledInput.checked = settings.wind.enabled;
    windSpeedInput.value = settings.wind.speed;
    windSpeedValue.textContent = settings.wind.speed;
    windDirectionInput.value = settings.wind.directionDeg;
    windDirectionValue.textContent = settings.wind.directionDeg + "°";
    windGustInput.value = settings.wind.gust;
    windGustValue.textContent = settings.wind.gust;
    windEnabledInput.addEventListener("change", function () {
        settings.wind.enabled = windEnabledInput.checked;
        saveSettings();
    });
    windSpeedInput.addEventListener("input", function () {
        settings.wind.speed = parseFloat(windSpeedInput.value);
        windSpeedValue.textContent = windSpeedInput.value;
        saveSettings();
    });
    windDirectionInput.addEventListener("input", function () {
        settings.wind.directionDeg = parseFloat(windDirectionInput.value);
        windDirectionValue.textContent = windDirectionInput.value + "°";
        saveSettings();
    });
    windGustInput.addEventListener("input", function () {
        settings.wind.gust = parseFloat(windGustInput.value);
        windGustValue.textContent = windGustInput.value;
        saveSettings();
    });

    window.addEventListener("resize", resizeRenderer);

    document.getElementById("inputSourceSelect").addEventListener("change", function (e) {
        settings.inputSource = e.target.value;
        saveSettings();
    });

    window.addEventListener("gamepadconnected", function (e) {
        setGamepadButtonVisible(true);
        buildGamepadPanel(e.gamepad);
    });
    window.addEventListener("gamepaddisconnected", function () {
        if (!rawFirstGamepad()) setGamepadButtonVisible(false);
        else populateInputSourceSelect();
    });
    const existingGamepad = rawFirstGamepad();
    if (existingGamepad) {
        setGamepadButtonVisible(true);
        buildGamepadPanel(existingGamepad);
    }

    window.addEventListener("keydown", function (e) {
        if (["ShiftLeft", "ShiftRight", "ControlLeft", "ControlRight", "Space"].indexOf(e.code) !== -1) {
            e.preventDefault();
        }
        keys.add(e.code);
        if (e.repeat) return;
        switch (e.code) {
            case "Digit1": droneState.flightMode = "stabilized"; break;
            case "Digit2": droneState.flightMode = "althold"; break;
            case "Digit3": droneState.flightMode = "acro"; break;
            case "KeyK": toggleKill(); break;
            case "KeyR": resetDrone(); break;
            case "KeyC": toggleCamera(); break;
            case "KeyT": togglePanel(document.getElementById("ratesPanel")); break;
            case "KeyH": togglePanel(document.getElementById("helpPanel")); break;
            case "KeyO": toggleFpvHud(); break;
        }
    });
    window.addEventListener("keyup", function (e) {
        keys.delete(e.code);
    });

    requestAnimationFrame(animate);
});
