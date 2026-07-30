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
// Propellskivene virker som "fallskjermer" ved vertikal bevegelse (mye mer luftmotstand opp/ned enn
// horisontalt gjennom lufta) - uten dette føles droneen glatt/"såpete" når gassen slippes.
const VERTICAL_DRAG_MULTIPLIER = 1.6;
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
const CRASH_SINK_RATE = 6;          // m/s - synkefart ved bakkeberøring som regnes som en hard krasj
const FIXED_DT = 1 / 120;           // fysikk-tidssteg
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

/* ---------- Hjelpefunksjoner (delt kode, se js/simulator-common.js) ---------- */
const clamp = Sim.clamp;
const rampStick = Sim.rampStick;
const computeRate = Sim.computeRate;
const computeThrottleCurve = Sim.computeThrottleCurve;
const integrateOrientation = Sim.integrateOrientation;

function loadRates() {
    return Sim.loadJSON(RATE_STORAGE_KEY, DEFAULT_RATES);
}
function saveRates() {
    Sim.saveJSON(RATE_STORAGE_KEY, rates);
}
function loadGamepadMap() {
    return Sim.loadJSON(GAMEPAD_STORAGE_KEY, DEFAULT_GAMEPAD_MAP);
}
function saveGamepadMap() {
    Sim.saveJSON(GAMEPAD_STORAGE_KEY, gamepadMap);
}
function loadSettings() {
    const result = Sim.loadJSON(SETTINGS_STORAGE_KEY, DEFAULT_SETTINGS);
    // Vind skal alltid være av ved sideinnlasting, uansett hva som var lagret fra en tidligere økt -
    // styrke/retning/kast huskes fortsatt, men man må aktivere vinden på nytt hver gang siden lastes.
    result.wind.enabled = false;
    return result;
}
function saveSettings() {
    Sim.saveJSON(SETTINGS_STORAGE_KEY, settings);
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
    grounded: false, // i bakkekontakt denne fysikk-ticken - vind skal ikke drifte den mens den står
    crashed: false // hard landing (se CRASH_SINK_RATE) - killswitch slår automatisk inn, varsel i HUD
};

let linkQuality = 1;

/* ---------- Vind (stabil + kast, se Sim.computeWind i simulator-common.js) ---------- */
const currentWindVector = new THREE.Vector3();
const windGustOffset = new THREE.Vector3();

function updateWind(dt) {
    Sim.computeWind(dt, settings.wind, windGustOffset, currentWindVector);
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
// buildGradientSky/buildGroundTexture/buildWindsockPole/updateWindsockVisual: se
// js/simulator-common.js (Sim.*) - delt med fixed-wing-simulatoren.
let windsockHandle = null;

function buildGround() {
    const group = new THREE.Group();
    const groundMat = new THREE.MeshStandardMaterial({ map: Sim.buildGroundTexture() });
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
            windsockHandle = Sim.buildWindsockPole();
            windsockHandle.group.position.set(Math.cos(angle) * r, 0, Math.sin(angle) * r);
            group.add(windsockHandle.group);
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

// buildLandingPad: se Sim.buildLandingPad i simulator-common.js.

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

    const roofPad = Sim.buildLandingPad(Math.min(width, depth) - 1);
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

// buildTree: se Sim.buildTree i simulator-common.js.

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

    const spawnPad = Sim.buildLandingPad(2.4);
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
        const tree = Sim.buildTree(t.h);
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
    scene.add(Sim.buildGradientSky());
    scene.add(buildGround());
    scene.add(buildWorldObjects());

    scene.add(new THREE.AmbientLight(0xffffff, 0.42)); // litt lavere enn før - gir tydeligere kontrast i den ekte skyggekart-skyggen
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

    // Pilot-figur akkurat der VLOS-kameraet står - synlig fra Chase/FPV (eget layer 1), men ikke fra
    // VLOS selv (den kameraet IKKE aktiverer layer 1, ser derfor kun standard layer 0 - personen "ser ikke seg selv").
    const vlosPerson = Sim.buildPersonFigure();
    vlosPerson.position.copy(vlosCamera.position);
    vlosPerson.position.y = 0;
    vlosPerson.traverse(function (obj) { obj.layers.set(1); });
    scene.add(vlosPerson);
    chaseCamera.layers.enable(1);
    fpvCamera.layers.enable(1);

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
    Sim.resizeRenderer(renderer, wrap, [chaseCamera, fpvCamera, vlosCamera]);
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
const rawFirstGamepad = Sim.rawFirstGamepad;
function getActiveGamepad() {
    return Sim.getActiveGamepad(settings.inputSource);
}
const readStickAxis = Sim.readStickAxis;
const readThrottleAxis = Sim.readThrottleAxis;

/* ---------- Gamepad knappemapping (kill/arm + flymodus-brytere) ---------- */
// Se Sim.createButtonBindingManager i simulator-common.js for læringsflyten (bryter kan komme
// som HID-knapp eller som en akse - fungerer med enhver sender i USB-joystick-modus).
const BUTTON_ACTIONS = {
    kill: toggleKill,
    modeAcro: function () { droneState.flightMode = "acro"; },
    modeStabilized: function () { droneState.flightMode = "stabilized"; },
    modeAltHold: function () { droneState.flightMode = "althold"; }
};
const buttonManager = Sim.createButtonBindingManager(gamepadMap.buttons, BUTTON_ACTIONS, saveGamepadMap);

function updateInput(dt) {
    updateLinkAndBattery(dt);

    const gp = getActiveGamepad();
    if (gp) buttonManager.poll(gp);

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
    // Anisotropisk drag: vertikal (verdens-Y) bevegelse møter mer motstand enn horisontal (se
    // VERTICAL_DRAG_MULTIPLIER) - propellskivene bremser opp-/nedgang mye kraftigere enn linjefart.
    const dragVec = new THREE.Vector3(
        airRelativeVelocity.x * -spec.linearDrag,
        airRelativeVelocity.y * -spec.linearDrag * VERTICAL_DRAG_MULTIPLIER,
        airRelativeVelocity.z * -spec.linearDrag
    );
    const accel = new THREE.Vector3().add(thrustVec).add(gravityVec).add(dragVec).multiplyScalar(1 / spec.mass);

    droneState.velocity.add(accel.clone().multiplyScalar(dt));
    droneState.position.add(droneState.velocity.clone().multiplyScalar(dt));
    const pushedFromWall = pushOutOfSolidWalls(droneState.position, droneState.velocity);

    const angVelVec = new THREE.Vector3(droneState.angularVelocity.pitch, droneState.angularVelocity.yaw, droneState.angularVelocity.roll);
    integrateOrientation(droneState.quaternion, angVelVec, dt);

    droneState.grounded = false;
    if (droneHasLandingLegs(droneState.droneClass)) {
        resolveLegGroundContact(dt, wasGrounded);
    } else {
        // Nettopp dyttet ut av en veggside denne rammen (dvs. ikke i ferd med å lande ovenfra) - se bort fra
        // objektets topp-flate for bakkesjekken under, ellers ville den blitt tolket som en landing på taket
        // og loftet brått opp dit. Den flate bakken (0) skal likevel fanges opp som vanlig.
        const surfaceY = (pushedFromWall ? 0 : solidSurfaceHeightAt(droneState.position.x, droneState.position.z)) + GROUND_CLEARANCE;
        if (droneState.position.y <= surfaceY) {
            if (!wasGrounded && droneState.velocity.y < -CRASH_SINK_RATE) {
                droneState.crashed = true;
                droneState.armed = false;
            }
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

function resolveLegGroundContact(dt, wasGrounded) {
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

    if (!wasGrounded && droneState.velocity.y < -CRASH_SINK_RATE) {
        droneState.crashed = true;
        droneState.armed = false;
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
    droneState.crashed = false;
    // Gass er en ikke-selvsentrerende "holdt" verdi på tastatur - må nullstilles eksplisitt ved reset,
    // ellers tar droneen av igjen umiddelbart. (Gamepad overskriver denne uansett neste frame.)
    inputState.stick.throttle = 0;
    droneState.batteryPercent = 100;
}

function toggleKill() {
    if (droneState.crashed) return; // må resettes (R) etter en krasj, ikke bare re-armes
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
const crashBanner = document.getElementById("crashBanner");

function updateHud() {
    hudMode.textContent = MODE_LABELS[droneState.flightMode];
    hudArmed.textContent = droneState.crashed ? "Krasjet" : (droneState.armed ? "Armed" : "Killed");
    hudArmed.className = "sim-status-value " + ((droneState.armed && !droneState.crashed) ? "sim-armed" : "sim-killed");
    crashBanner.classList.toggle("show", droneState.crashed);
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

/* ---------- Paneler (rates / drone-kamera / vind / gamepad / hjelp) ---------- */
const ALL_PANEL_IDS = ["ratesPanel", "droneCameraPanel", "windPanel", "gamepadPanel", "helpPanel"];
function togglePanel(panel) {
    Sim.togglePanel(panel, ALL_PANEL_IDS.map(function (id) { return document.getElementById(id); }));
}

/* ---------- Rates-panel (rate-kurver + gass-expo, se Sim.buildRateAxisBox/buildThrottleExpoBox) ---------- */
function buildRatesPanel() {
    const grid = document.getElementById("ratesGrid");
    grid.innerHTML = "";
    ["roll", "pitch", "yaw"].forEach(function (axis) {
        grid.appendChild(Sim.buildRateAxisBox(rates[axis], AXIS_LABELS[axis], saveRates));
    });
    grid.appendChild(Sim.buildThrottleExpoBox(
        rates.throttle,
        "Gass",
        "0 = lineær gass. Høyere verdi gir finere kontroll nær midten (rundt hover), mer kraftfull respons ved fullt utslag.",
        saveRates
    ));
}

function populateInputSourceSelect() {
    const select = document.getElementById("inputSourceSelect");
    if (!select) return;
    settings.inputSource = Sim.populateInputSourceSelect(select, settings.inputSource);
    saveSettings();
}

function buildGamepadPanel(pad) {
    populateInputSourceSelect();
    const grid = document.getElementById("gamepadGrid");
    // RC-sendere har typisk 8 kanaler selv om enheten (ennå) ikke rapporterer full akse-lengde -
    // vis alltid minst 8 valg, i tillegg til flere hvis enheten faktisk har det.
    const axisCount = Math.max(pad.axes.length, Sim.MIN_GAMEPAD_CHANNELS);
    Sim.buildGamepadChannelsGrid(grid, gamepadMap, CHANNEL_LABELS, axisCount, saveGamepadMap);
    buildGamepadButtonsPanel();
}

function buildGamepadButtonsPanel() {
    const container = document.getElementById("gamepadButtonsGrid");
    Sim.buildGamepadButtonsGrid(container, gamepadMap.buttons, BUTTON_ACTION_LABELS, buttonManager, getActiveGamepad, saveGamepadMap);
}

function updateGamepadAxesReadout(gp) {
    const panel = document.getElementById("gamepadPanel");
    if (panel.style.display === "none") return;
    const readout = document.getElementById("gamepadAxesReadout");
    const activeGp = gp || getActiveGamepad();
    Sim.updateGamepadAxesReadout(readout, activeGp, Sim.MIN_GAMEPAD_CHANNELS);
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

const drawFpvCrosshair = Sim.drawFpvCrosshair;

// Samme aksekonvensjon-negasjon som resten av fysikken (se merknad i stepPhysics).
function drawFpvHorizon(ctx, w, h) {
    const euler = new THREE.Euler().setFromQuaternion(droneState.quaternion, "YXZ");
    const pitchDeg = -THREE.MathUtils.radToDeg(euler.x);
    const rollDeg = -THREE.MathUtils.radToDeg(euler.z);
    Sim.drawFpvHorizonFromAngles(ctx, w, h, pitchDeg, rollDeg);
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

function updateWindsockVisual(now) {
    Sim.updateWindsockVisual(windsockHandle, now, currentWindVector);
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

    const settingsMenuEl = document.getElementById("settingsMenu");
    Sim.setupDropdown(document.getElementById("settingsToggleBtn"), settingsMenuEl,
        ["ratesPanel", "droneCameraPanel", "windPanel", "gamepadPanel"].map(function (id) { return document.getElementById(id); }));
    Sim.wirePanelCloseButtons(settingsMenuEl);
    function closeSettingsMenu() { settingsMenuEl.classList.remove("open"); }

    document.getElementById("toggleRatesBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("ratesPanel"));
        closeSettingsMenu();
    });
    document.getElementById("toggleDroneCameraBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("droneCameraPanel"));
        closeSettingsMenu();
    });
    document.getElementById("toggleWindBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("windPanel"));
        closeSettingsMenu();
    });
    document.getElementById("toggleHelpBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("helpPanel"));
    });
    document.getElementById("toggleGamepadBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("gamepadPanel"));
        closeSettingsMenu();
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
