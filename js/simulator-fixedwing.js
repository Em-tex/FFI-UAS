/* js/simulator-fixedwing.js
   Fixed-wing simulator - gjenbruker matte/kontroll/gamepad/scene-hjelpere fra js/simulator-common.js
   (window.Sim). Fysikkmodellen her er en forenklet, men AoA/steile-basert aerodynamikk-modell
   (løft/drag per vinge, venstre/høyre beregnet separat for realistisk vingedypp/spinn-tendens ved
   steiling i sving), ikke bare en kraft-vektor slik quadcopter-simulatoren bruker. */

/* ---------- Konstanter: fysikk ---------- */
const GRAVITY = 9.81;
const AIR_DENSITY = 1.225; // kg/m^3, havnivå

// I motsetning til quad-simulatorens Acro-modus (som setter ønsket vinkelhastighet direkte) styres et
// ekte fly ALDRI ved å kommandere en rate - pinnen avbøyer en rorflate, og rorflaten skaper et
// dreiemoment proporsjonalt med dynamisk trykk (0.5*rho*V^2), dempet av en motsatt rettet
// dreiemoment fra flyets egen vinkelhastighet (aerodynamisk demping - uten den ville flyet spunnet
// evig som i verdensrommet). Dette gir automatisk de to viktigste treningsegenskapene til et ekte fly:
// nesten ingen kontroll ved lav fart (qDyn liten -> begge momentene er små -> treg respons/"mushy" ror),
// og fast/hurtig respons ved høy fart - uten noen egen "svekk kontroll ved lav fart"-multiplikator.
// (Effektivitet og demping skalert opp ~3x fra første forsøk - responstiden ved gitt fart er
// proporsjonal med treghet/(fart^2 * demping), så den første tuningen ga en merkbart treg/"gummi-aktig"
// respons selv ved marsjfart. Forholdet effektivitet/demping - og dermed rotasjonshastigheten ved full
// utslag - er uendret, kun hvor RASKT den nås.)
const ROLL_CONTROL_EFFECTIVENESS = 0.15;
const ROLL_DAMPING = 0.03;
// Pitch fikk samme 3x-oppjustering som roll først, men det - kombinert med lavere pitch-treghet på
// Liten-klassen - gjorde høyderoret altfor nervøst (lett å overrotere/tailstrike). Dempet ned til 1.5x
// original i stedet for 3x - fortsatt raskere enn første forsøk, men ikke like "hårtrigger".
const PITCH_CONTROL_EFFECTIVENESS = 0.045;
const PITCH_DAMPING = 0.0165;
// Naturlig (utrimmet) stigemoment fra vinge/hale - vokser med dynamisk trykk (V^2) akkurat som
// kontroll-/dempeleddene over, men er UAVHENGIG av rorutslag. Uten dette leddet er null utslag ved
// null trim i likevekt ved ENHVER fart (ingen grunn til å trimme om ved fartsendring) - med det må
// piloten aktivt trimme om i Manual når farten endrer seg, akkurat som i et ekte fly ("trim følger
// fart/pådrag"). Holdt godt under PITCH_CONTROL_EFFECTIVENESS slik at fullt rorutslag/trim alltid kan
// overvinne det, selv i høy fart.
const SPEED_PITCH_MOMENT_COEFF = 0.014;
const YAW_CONTROL_EFFECTIVENESS = 0.036;
const YAW_DAMPING = 0.027;
const PASSIVE_ANGULAR_DAMPING = 0.995;

const MAX_BANK_ANGLE = 50;      // grader, Stabilized: pinne-utslag -> ønsket krengevinkel
// Økt videre (10 -> 12 -> 15) - brukeren rapporterte at tailstrike-geometrien (halelengden), ikke
// steilevinkelen, er den reelle begrensningen på rotasjon ved avgang, så "godt under laveste
// steilevinkel"-marginen (opprinnelig hensikt her) er ikke lenger den bindende bekymringen. Ved lav
// fart vil steiling naturlig gi redusert løft/nese-fall lenge før Stabilized når helt fram til et
// kommandert 15°-mål uansett (P-loopen bruker tid på å nå målet), så risikoen holdes akseptabel.
const MAX_PITCH_ANGLE = 15;     // grader, Stabilized: pinne-utslag -> ønsket stigningsvinkel
// Stabilized er en enkel autopilot som styrer AKKURAT DE SAMME rorflatene som Manual (samme
// torque-modell) - den kommanderer bare en ror-avbøyning proporsjonal med vinkelavviket i stedet for
// direkte fra pinnen. Disse gradtallene er "full ror-avbøyning oppnås ved dette vinkelavviket".
const STABILIZED_BANK_AUTHORITY_DEG = 25;
const STABILIZED_PITCH_AUTHORITY_DEG = 12;
const RUDDER_COORDINATION_GAIN = 0.35; // Stabilized: automatisk sideror proporsjonalt med kommandert krengning

const TRIM_RANGE_DEG = 15;      // maks høyderor-trim, begge retninger
const TRIM_STEP_PER_SEC = 6;    // grader/s trim-endring når [ / ] holdes inne (tastatur)
const STALL_POST_RANGE_DEG = 6; // bredde på overgangssonen rett etter kritisk vinkel før dyp steiling

const ROLLING_FRICTION = 0.045; // rullemotstand (hjul mot asfalt), ~typisk for lette luftfartøy
const GROUND_CLEARANCE_FW = 0.05;
const CRASH_SINK_RATE = 6;      // m/s synkefart ved berøring som teller som hard landing
const CRASH_BANK_DEG = 45;      // krengevinkel ved berøring som teller som hard landing

const FIXED_DT = 1 / 120;       // fysikk-tidssteg, samme substep-mønster (akkumulator) som quad-simulatoren
const STICK_RAMP_TIME = 0.22;
const THROTTLE_RATE = 0.7;

const RUNWAY_LENGTH = 360;
const RUNWAY_WIDTH = 14;
const RUNWAY_NEAR_Z = 20;   // verdens-Z for nærmeste terskel (nærmest spawn)
const RUNWAY_SPAWN_Z = 8;   // spawn litt bak terskelen, klar for avgang nedover -Z

/* ---------- Flystørrelser ---------- */
// Kontrolleffektivitet/-demping (se ROLL/PITCH/YAW_CONTROL_EFFECTIVENESS/_DAMPING) er felles for alle
// størrelser, akkurat som TORQUE_GAIN er felles i quad-simulatoren - det er inertia/masse under som gir
// et større fly en tregere, mer "sluggish" respons ved samme fart, ikke en egen per-klasse ro-styrke.
const PLANE_CLASSES = {
    small: {
        label: "Liten (trener)",
        // Lett og godt motorisert (typisk for en liten elektrisk skoleflymaskin) - skal føles nimbel og
        // lettflydd, ikke tung/undermotorisert. Trekkraft/vekt ≈ 0.83.
        // wingArea redusert (strekkforhold ~8.5 i stedet for ~5.9) - forrige vinge var for bred/kort
        // ("låvedør") relativt til skroget. Øker steilefarten noe (~7.6 -> ~9.2 m/s), en akseptabel
        // avveining mot et fly som faktisk ser ut som et fly.
        mass: 2.2, wingArea: 0.38, wingSpan: 1.8,
        maxThrust: 18, cd0: 0.045, inducedDragK: 0.9, clSlope: 0.11, stallAngleDeg: 14,
        // inertiaRoll redusert (0.25 -> 0.17) - en liten skoleflymaskin skal kunne rulles raskt/nimbelt,
        // følte seg for treg på roll (ROLL_CONTROL_EFFECTIVENESS/-DAMPING er delt mellom alle klasser,
        // se merknad over - lavere inertia her er den korrekte, klassespesifikke måten å gjøre AKKURAT
        // dette flyet rappere på roll uten å påvirke Middels/Stor).
        inertiaRoll: 0.17, inertiaPitch: 0.42, inertiaYaw: 0.42, yawStability: 0.4,
        // Økt (fra -0.14) sammen med lengre understell i buildPlane - propellen var lengre enn
        // hjulklaringen og stakk ned i rullebanen.
        gearOffsetY: -0.22, visualScale: 1.0
    },
    medium: {
        label: "Middels",
        mass: 8, wingArea: 0.65, wingSpan: 2.4,
        maxThrust: 22, cd0: 0.04, inducedDragK: 1.0, clSlope: 0.105, stallAngleDeg: 13,
        inertiaRoll: 0.9, inertiaPitch: 1.3, inertiaYaw: 1.5, yawStability: 0.35,
        gearOffsetY: -0.28, visualScale: 1.4
    },
    large: {
        label: "Stor",
        mass: 22, wingArea: 1.2, wingSpan: 3.4,
        maxThrust: 42, cd0: 0.035, inducedDragK: 1.1, clSlope: 0.1, stallAngleDeg: 12,
        inertiaRoll: 2.6, inertiaPitch: 3.6, inertiaYaw: 4.0, yawStability: 0.3,
        gearOffsetY: -0.35, visualScale: 2.0
    }
};
const DEFAULT_PLANE_CLASS = "small";

const MODE_LABELS = { stabilized: "Stabilized", manual: "Manual" };
const AXIS_LABELS = { aileron: "Aileron", elevator: "Elevator", rudder: "Rudder" };
const CHANNEL_LABELS = { aileron: "Aileron", elevator: "Elevator", rudder: "Rudder", throttle: "Gass" };

const RATE_STORAGE_KEY = "ffi-uas:fixedwing-rates";
const GAMEPAD_STORAGE_KEY = "ffi-uas:fixedwing-gamepad-map";
const SETTINGS_STORAGE_KEY = "ffi-uas:fixedwing-settings";

const DEFAULT_RATES = {
    aileron: { centerSensitivity: 80, maxRate: 260, expo: 0.25 },
    // maxRate redusert (200 -> 150) - full utslag ga for lettvint over-rotasjon/tailstrike i Manual under
    // avgang. Kun standardverdien - endres du rate-kurven selv i panelet, lagres ditt eget valg som før.
    elevator: { centerSensitivity: 60, maxRate: 150, expo: 0.25 },
    rudder: { centerSensitivity: 60, maxRate: 150, expo: 0.3 },
    throttle: { expo: 0 }
};

// TAER-rekkefølge (Throttle/Aileron/Elevator/Rudder) - samme standard som quad-simulatoren.
const DEFAULT_GAMEPAD_MAP = {
    throttle: { axis: 0, reverse: false },
    aileron: { axis: 1, reverse: false },
    elevator: { axis: 2, reverse: false },
    rudder: { axis: 3, reverse: false },
    buttons: { kill: null, modeStabilized: null, modeManual: null, trimUp: null, trimDown: null }
};
// trimUp/trimDown er IKKE med i BUTTON_ACTIONS/buttonManager under - de er kontinuerlige "hold inne"-
// handlinger (se adjustTrim), ikke enkelt-trigget på stigende kant slik kill/modus-bryterne er.
const BUTTON_ACTION_LABELS = {
    kill: "Motor av/på", modeStabilized: "Modus: Stabilized", modeManual: "Modus: Manual",
    trimUp: "Trim opp", trimDown: "Trim ned"
};

const DEFAULT_WIND = { enabled: false, speed: 5, directionDeg: 0, gust: 0.3 };

const DEFAULT_SETTINGS = {
    fpvTiltDeg: 0,
    planeClass: DEFAULT_PLANE_CLASS,
    inputSource: "auto",
    wind: DEFAULT_WIND,
    fpvHudMode: "crosshair"
};

const FPV_HUD_MODES = ["crosshair", "horizon", "none"];
const FPV_HUD_MODE_LABELS = { crosshair: "Crosshair", horizon: "Kunstig horisont", none: "Ingen" };
const CAMERA_MODES = ["chase", "fpv", "vlos"];
const CAMERA_MODE_LABELS = { chase: "Chase", fpv: "FPV", vlos: "VLOS" };

/* ---------- Hjelpefunksjoner (delt kode, se js/simulator-common.js) ---------- */
const clamp = Sim.clamp;
const rampStick = Sim.rampStick;
const computeRate = Sim.computeRate;
const computeThrottleCurve = Sim.computeThrottleCurve;
const integrateOrientation = Sim.integrateOrientation;

function loadRates() { return Sim.loadJSON(RATE_STORAGE_KEY, DEFAULT_RATES); }
function saveRates() { Sim.saveJSON(RATE_STORAGE_KEY, rates); }
function loadGamepadMap() { return Sim.loadJSON(GAMEPAD_STORAGE_KEY, DEFAULT_GAMEPAD_MAP); }
function saveGamepadMap() { Sim.saveJSON(GAMEPAD_STORAGE_KEY, gamepadMap); }
function loadSettings() {
    const result = Sim.loadJSON(SETTINGS_STORAGE_KEY, DEFAULT_SETTINGS);
    result.wind.enabled = false;
    // Standard kameravinkel skal alltid være 0 grader - tvinges her (ikke bare i DEFAULT_SETTINGS) slik
    // at en tidligere lagret vinkel fra før denne endringen ikke overlever et reload.
    result.fpvTiltDeg = 0;
    return result;
}
function saveSettings() { Sim.saveJSON(SETTINGS_STORAGE_KEY, settings); }

/* ---------- Tilstand ---------- */
const rates = loadRates();
const gamepadMap = loadGamepadMap();
const settings = loadSettings();

const planeState = {
    position: new THREE.Vector3(0, 0.3, RUNWAY_SPAWN_Z),
    velocity: new THREE.Vector3(0, 0, 0),
    quaternion: new THREE.Quaternion(),
    angularVelocity: { pitch: 0, yaw: 0, roll: 0 },
    engineOn: true,
    crashed: false,
    onGround: true,
    flightMode: "stabilized",
    planeClass: PLANE_CLASSES[settings.planeClass] ? settings.planeClass : DEFAULT_PLANE_CLASS,
    elevatorTrimDeg: 0,
    lastRollDeflection: 0, lastPitchDeflection: 0, lastYawDeflection: 0
};

let lastAirspeed = 0;

function currentPlaneSpec() {
    return PLANE_CLASSES[planeState.planeClass];
}

function setPlaneClass(className) {
    if (!PLANE_CLASSES[className]) return;
    planeState.planeClass = className;
    settings.planeClass = className;
    saveSettings();
    if (scene) rebuildPlaneMesh();
}

/* ---------- Vind ---------- */
const currentWindVector = new THREE.Vector3();
const windGustOffset = new THREE.Vector3();
function updateWind(dt) {
    Sim.computeWind(dt, settings.wind, windGustOffset, currentWindVector);
}

const inputState = {
    source: "keyboard",
    stick: { roll: 0, pitch: 0, yaw: 0, throttle: 0 }
};
const keys = new Set();

let renderer, scene, chaseCamera, fpvCamera, vlosCamera, activeCamera;
let planeGroup, planePropeller;
let planeAileronLeft, planeAileronRight, planeElevator, planeRudder;
let propSpinSpeed = 0;
let cameraModeIndex = 0;
let windsockHandles = [];

/* ---------- Three.js: scene, rullebane, fly, kameraer ---------- */
function buildGround() {
    const group = new THREE.Group();
    const groundMat = new THREE.MeshStandardMaterial({ map: Sim.buildGroundTexture() });
    const ground = new THREE.Mesh(new THREE.PlaneGeometry(3000, 3000), groundMat);
    ground.rotation.x = -Math.PI / 2;
    ground.receiveShadow = true;
    group.add(ground);
    // Rutenettet lå nøyaktig i samme plan (y=0) som bakken under - to sammenfallende flater flimrer
    // (z-fighting) i dybdebufferet. Løftet en anelse over bakken for å skille dem tydelig.
    const grid = new THREE.GridHelper(3000, 300, 0x1f3d1f, 0x2d4d2d);
    grid.position.y = 0.01;
    group.add(grid);
    return group;
}

// Prosedural rullebane-tekstur: asfalt, kantlinjer, stiplet midtlinje og terskelstriper i begge ender.
function buildRunwayTexture() {
    const texW = 128, texH = 1024;
    const canvas = document.createElement("canvas");
    canvas.width = texW;
    canvas.height = texH;
    const ctx = canvas.getContext("2d");
    ctx.fillStyle = "#48484a";
    ctx.fillRect(0, 0, texW, texH);

    ctx.fillStyle = "#e8e8e8";
    ctx.fillRect(texW * 0.04, 0, texW * 0.03, texH);
    ctx.fillRect(texW * 0.93, 0, texW * 0.03, texH);

    const dashLen = texH * 0.02, gapLen = texH * 0.015;
    let y = texH * 0.1;
    while (y < texH * 0.9) {
        ctx.fillRect(texW / 2 - texW * 0.01, y, texW * 0.02, dashLen);
        y += dashLen + gapLen;
    }

    // Terskelstriper: smale, korte felt på tvers av banen med tydelig avstand mellom seg - ikke lange,
    // sammenhengende felt langs banen (som ga et feilaktig "solid blokk"-utseende).
    function threshold(yStart) {
        const barCount = 6, barW = texW * 0.05, gap = texW * 0.032;
        const stripeLen = texH * 0.013; // ~4.6 m langs banen ved full RUNWAY_LENGTH
        const totalW = barCount * barW + (barCount - 1) * gap;
        let x = texW / 2 - totalW / 2;
        for (let i = 0; i < barCount; i++) {
            ctx.fillRect(x, yStart, barW, stripeLen);
            x += barW + gap;
        }
    }
    threshold(texH * 0.02);
    threshold(texH * 0.965);

    return new THREE.CanvasTexture(canvas);
}

function buildRunway() {
    const geo = new THREE.PlaneGeometry(RUNWAY_WIDTH, RUNWAY_LENGTH);
    const mat = new THREE.MeshStandardMaterial({ map: buildRunwayTexture() });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.rotation.x = -Math.PI / 2;
    mesh.position.set(0, 0.04, RUNWAY_NEAR_Z - RUNWAY_LENGTH / 2);
    mesh.receiveShadow = true;
    return mesh;
}

// Firkantet gate med sjakkrutet ramme (oransje/hvit) - samme prinsipp som quad-simulatorens racing-
// porter, men i god margin til flyets vingespenn (se GATE_SIZE) siden et fly er langt mindre nimbelt.
function buildGate(size, groundGap) {
    const group = new THREE.Group();
    const barThickness = 0.22;
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
            seg.castShadow = true;
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
        const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.12, groundGap, 8), legMat);
        leg.position.set(x, groundGap / 2, 0);
        group.add(leg);
    });
    return group;
}

// Låve/hus med én vindusåpning på hver av to motstående vegger - flyr inn den ene, gjennom, og ut den
// andre, akkurat som låven i quad-simulatorens bane. Sideveggene er hele; front-/bakvegg bygges av
// fire paneler rundt et hull (ingen geometri i selve åpningen).
function buildOpenBuilding(width, height, depth, windowW, windowH, sillY, wallColor, roofColor) {
    const group = new THREE.Group();
    const wallMat = new THREE.MeshStandardMaterial({ color: wallColor });
    const roofMat = new THREE.MeshStandardMaterial({ color: roofColor });
    const wallThickness = 0.3;

    const sideWall = new THREE.Mesh(new THREE.BoxGeometry(wallThickness, height, depth), wallMat);
    sideWall.position.set(-width / 2, height / 2, 0);
    sideWall.castShadow = true;
    sideWall.receiveShadow = true;
    group.add(sideWall);
    const otherWall = sideWall.clone();
    otherWall.position.x = width / 2;
    group.add(otherWall);

    function windowWall(zPos) {
        const wg = new THREE.Group();
        const topY = sillY + windowH;
        const bottom = new THREE.Mesh(new THREE.BoxGeometry(width, sillY, wallThickness), wallMat);
        bottom.position.y = sillY / 2;
        wg.add(bottom);
        const top = new THREE.Mesh(new THREE.BoxGeometry(width, height - topY, wallThickness), wallMat);
        top.position.y = topY + (height - topY) / 2;
        wg.add(top);
        const sideW = (width - windowW) / 2;
        const leftPanel = new THREE.Mesh(new THREE.BoxGeometry(sideW, windowH, wallThickness), wallMat);
        leftPanel.position.set(-width / 2 + sideW / 2, sillY + windowH / 2, 0);
        wg.add(leftPanel);
        const rightPanel = leftPanel.clone();
        rightPanel.position.x = width / 2 - sideW / 2;
        wg.add(rightPanel);
        wg.children.forEach(function (m) { m.castShadow = true; m.receiveShadow = true; });
        wg.position.z = zPos;
        return wg;
    }
    group.add(windowWall(-depth / 2));
    group.add(windowWall(depth / 2));

    const roof = new THREE.Mesh(new THREE.BoxGeometry(width + 0.6, 0.3, depth + 0.6), roofMat);
    roof.position.y = height + 0.15;
    roof.castShadow = true;
    roof.receiveShadow = true;
    group.add(roof);

    return group;
}

// Gate-området: en rekke porter et godt stykke vestover for rullebanen, som en enkel slalåm-løype å
// fly gjennom. GATE_SIZE gir god klaring selv for det største flyet (vingespenn 3.4 m).
const GATE_SIZE = 9, GATE_GROUND_GAP = 3;
const GATE_AREA_X = -(RUNWAY_WIDTH / 2 + 60);
const GATE_LINE = [
    { x: GATE_AREA_X, z: RUNWAY_NEAR_Z - 20 },
    { x: GATE_AREA_X - 8, z: RUNWAY_NEAR_Z - 80 },
    { x: GATE_AREA_X + 6, z: RUNWAY_NEAR_Z - 140 },
    { x: GATE_AREA_X - 4, z: RUNWAY_NEAR_Z - 200 }
];

function buildGateArea() {
    const group = new THREE.Group();
    const n = GATE_LINE.length;
    for (let i = 0; i < n; i++) {
        const wp = GATE_LINE[i];
        const next = GATE_LINE[(i + 1) % n];
        const gate = buildGate(GATE_SIZE, GATE_GROUND_GAP);
        gate.position.set(wp.x, 0, wp.z);
        gate.rotation.y = Math.atan2(next.x - wp.x, next.z - wp.z);
        group.add(gate);
    }
    return group;
}

// Hus- og låve-området: øst for rullebanen, med vindusåpninger store nok til at selv Stor-flyet
// (vingespenn 3.4 m) skal ha reell klaring gjennom hver bygning.
const BUILDING_AREA_X = RUNWAY_WIDTH / 2 + 60;

function buildBuildingArea() {
    const group = new THREE.Group();
    const barn1 = buildOpenBuilding(9, 8, 12, 6, 6, 1.6, 0xa1352b, 0x3a3a3a);
    barn1.position.set(BUILDING_AREA_X, 0, RUNWAY_NEAR_Z - 40);
    barn1.rotation.y = THREE.MathUtils.degToRad(15);
    group.add(barn1);

    const house1 = buildOpenBuilding(8, 6.5, 9, 5.5, 5.5, 1.3, 0xd8c9a0, 0x5a3a2a);
    house1.position.set(BUILDING_AREA_X + 8, 0, RUNWAY_NEAR_Z - 120);
    house1.rotation.y = THREE.MathUtils.degToRad(-12);
    group.add(house1);

    const barn2 = buildOpenBuilding(9, 8, 12, 6, 6, 1.6, 0xa1352b, 0x3a3a3a);
    barn2.position.set(BUILDING_AREA_X - 4, 0, RUNWAY_NEAR_Z - 200);
    barn2.rotation.y = THREE.MathUtils.degToRad(8);
    group.add(barn2);

    return group;
}

// Enkelt hus (solid, ikke gjennomflybart som barn/hus-området) med saltak - brukt til å fylle den
// lille byen med variasjon uten å trenge vindusåpninger.
function buildSimpleHouse(width, height, depth, wallColor, roofColor) {
    const group = new THREE.Group();
    const wallMat = new THREE.MeshStandardMaterial({ color: wallColor });
    const roofMat = new THREE.MeshStandardMaterial({ color: roofColor });
    const walls = new THREE.Mesh(new THREE.BoxGeometry(width, height, depth), wallMat);
    walls.position.y = height / 2;
    walls.castShadow = true;
    walls.receiveShadow = true;
    group.add(walls);
    const roof = new THREE.Mesh(new THREE.ConeGeometry(Math.max(width, depth) * 0.72, height * 0.55, 4), roofMat);
    roof.rotation.y = Math.PI / 4;
    roof.position.y = height + height * 0.55 * 0.5;
    roof.castShadow = true;
    group.add(roof);
    return group;
}

// Vanntårn - enkelt landemerke for byen (sylinderbein + stor sylindertank).
function buildWaterTower() {
    const group = new THREE.Group();
    const legMat = new THREE.MeshStandardMaterial({ color: 0x555555 });
    const tankMat = new THREE.MeshStandardMaterial({ color: 0xb8c4cc });
    [-1, 1].forEach(function (sx) {
        [-1, 1].forEach(function (sz) {
            const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.15, 9, 8), legMat);
            leg.position.set(sx * 1.4, 4.5, sz * 1.4);
            leg.castShadow = true;
            group.add(leg);
        });
    });
    const tank = new THREE.Mesh(new THREE.CylinderGeometry(2.6, 2.2, 3.4, 16), tankMat);
    tank.position.y = 10.7;
    tank.castShadow = true;
    group.add(tank);
    const roof = new THREE.Mesh(new THREE.ConeGeometry(2.7, 1.2, 16), legMat);
    roof.position.y = 13;
    roof.castShadow = true;
    group.add(roof);
    return group;
}

// Liten by et godt stykke øst for hus-/låve-området (utenfor rekkevidde for gjennomflyging) - gir
// simulatoren mye mer å se på i overflyging/navigasjonstrening, ikke bare rullebanen og noen få trær.
const TOWN_CENTER_X = BUILDING_AREA_X + 70;
const TOWN_CENTER_Z = RUNWAY_NEAR_Z - 150;
const TOWN_HOUSES = [
    { dx: -22, dz: -22, w: 6, h: 4, d: 6, wall: 0xd8c9a0, roof: 0x7a3a2a, ry: 0.3 },
    { dx: 4, dz: -28, w: 5, h: 3.6, d: 5, wall: 0xc9d0d8, roof: 0x4a4a52, ry: 1.1 },
    { dx: 26, dz: -16, w: 7, h: 4.5, d: 6, wall: 0xe0d8c0, roof: 0x6a3a3a, ry: -0.4 },
    { dx: -26, dz: 4, w: 5, h: 3.8, d: 5, wall: 0xd0c8b8, roof: 0x5a4a3a, ry: 0.8 },
    { dx: 20, dz: 10, w: 6, h: 4, d: 5, wall: 0xc8d0c8, roof: 0x4a3a3a, ry: -1.0 },
    { dx: -12, dz: 26, w: 5, h: 3.5, d: 6, wall: 0xd8d0c0, roof: 0x6a4a2a, ry: 0.2 },
    { dx: 15, dz: 30, w: 6, h: 4, d: 5, wall: 0xc0c8d0, roof: 0x3a4a4a, ry: 1.4 },
    { dx: -30, dz: -42, w: 5, h: 3.8, d: 5, wall: 0xd0d8c8, roof: 0x5a3a3a, ry: -0.6 },
    { dx: 32, dz: -38, w: 6, h: 4.2, d: 6, wall: 0xe0d0c8, roof: 0x4a3a2a, ry: 0.9 },
    { dx: 0, dz: -46, w: 6, h: 4, d: 5, wall: 0xd0c8c0, roof: 0x5a4a4a, ry: 0 }
];

function buildTown() {
    const group = new THREE.Group();
    TOWN_HOUSES.forEach(function (h) {
        const house = buildSimpleHouse(h.w, h.h, h.d, h.wall, h.roof);
        house.position.set(TOWN_CENTER_X + h.dx, 0, TOWN_CENTER_Z + h.dz);
        house.rotation.y = h.ry;
        group.add(house);
    });

    // Litt større "rådhus"-aktig bygg midt i byen som et landemerke å navigere etter.
    const townHall = buildSimpleHouse(9, 5.5, 8, 0xe8e0d0, 0x3a3a3a);
    townHall.position.set(TOWN_CENTER_X, 0, TOWN_CENTER_Z);
    group.add(townHall);

    const tower = buildWaterTower();
    tower.position.set(TOWN_CENTER_X - 42, 0, TOWN_CENTER_Z + 6);
    group.add(tower);

    // Liten dam ved kanten av byen - rent visuelt landemerke.
    // Løftet fra 0.02 til 0.05 over bakken - samme prinsipp som rullebanen/rutenettet (se buildGround) -
    // 0.02 var for tynn en margin og flimret (z-fighting) mot bakkeplanet under.
    const pond = new THREE.Mesh(new THREE.CircleGeometry(8, 24), new THREE.MeshStandardMaterial({ color: 0x2a5a78 }));
    pond.rotation.x = -Math.PI / 2;
    pond.position.set(TOWN_CENTER_X + 40, 0.05, TOWN_CENTER_Z - 10);
    group.add(pond);

    // Trekrans rundt byen.
    [
        { dx: -45, dz: -55, h: 7 }, { dx: 45, dz: -55, h: 7.5 }, { dx: -50, dz: 20, h: 6.5 },
        { dx: 50, dz: 25, h: 7 }, { dx: -20, dz: 50, h: 6.8 }, { dx: 25, dz: 52, h: 7.2 },
        { dx: 0, dz: -60, h: 8 }, { dx: 55, dz: -10, h: 6.5 }
    ].forEach(function (t) {
        const tree = Sim.buildTree(t.h);
        tree.position.set(TOWN_CENTER_X + t.dx, 0, TOWN_CENTER_Z + t.dz);
        group.add(tree);
    });

    return group;
}

function buildWorldObjects() {
    const group = new THREE.Group();
    group.add(buildRunway());

    // Vindpølse i begge ender av rullebanen - viser vindretning/styrke uansett hvilken vei du lander.
    const windsockNear = Sim.buildWindsockPole();
    windsockNear.group.position.set(RUNWAY_WIDTH / 2 + 8, 0, RUNWAY_NEAR_Z - 10);
    group.add(windsockNear.group);
    windsockHandles.push(windsockNear);

    const windsockFar = Sim.buildWindsockPole();
    windsockFar.group.position.set(RUNWAY_WIDTH / 2 + 8, 0, RUNWAY_NEAR_Z - RUNWAY_LENGTH + 10);
    group.add(windsockFar.group);
    windsockHandles.push(windsockFar);

    group.add(buildGateArea());
    group.add(buildBuildingArea());
    group.add(buildTown());

    // God del flere trær spredt bredere rundt hele kartet enn før - gjør overflyging/navigasjon mer
    // interessant å se på, ikke bare rullebanen og de nære flyv-gjennom-områdene.
    [
        { x: RUNWAY_WIDTH / 2 + 15, z: RUNWAY_NEAR_Z - 40, h: 7 },
        { x: -(RUNWAY_WIDTH / 2 + 15), z: RUNWAY_NEAR_Z - 90, h: 8 },
        { x: RUNWAY_WIDTH / 2 + 20, z: RUNWAY_NEAR_Z - 160, h: 7.5 },
        { x: -(RUNWAY_WIDTH / 2 + 18), z: RUNWAY_NEAR_Z - 220, h: 6.5 },
        { x: RUNWAY_WIDTH / 2 + 16, z: RUNWAY_NEAR_Z - 280, h: 7 },
        { x: RUNWAY_WIDTH / 2 + 5, z: RUNWAY_NEAR_Z + 15, h: 6 },
        { x: -(RUNWAY_WIDTH / 2 + 6), z: RUNWAY_NEAR_Z + 20, h: 6.5 },
        { x: -(RUNWAY_WIDTH / 2 + 30), z: RUNWAY_NEAR_Z - 30, h: 7.2 },
        { x: -(RUNWAY_WIDTH / 2 + 25), z: RUNWAY_NEAR_Z - 250, h: 7.8 },
        { x: RUNWAY_WIDTH / 2 + 35, z: RUNWAY_NEAR_Z - 240, h: 6.4 },
        { x: RUNWAY_WIDTH / 2 + 30, z: RUNWAY_NEAR_Z + 30, h: 5.8 },
        { x: -(RUNWAY_WIDTH / 2 + 35), z: RUNWAY_NEAR_Z + 35, h: 7 },
        { x: 30, z: RUNWAY_NEAR_Z - RUNWAY_LENGTH - 5, h: 6.8 },
        { x: -30, z: RUNWAY_NEAR_Z - RUNWAY_LENGTH - 15, h: 7.3 }
    ].forEach(function (t) {
        const tree = Sim.buildTree(t.h);
        tree.position.set(t.x, 0, t.z);
        group.add(tree);
    });

    return group;
}

// Vingen bygges i to deler langs korden (fremre del over HELE spennvidden, bakre del kun i midt-
// seksjonen) i stedet for én sammenhengende boks - det gir et "hakk" i bakkanten ute på tuppene der
// balanseroret felles inn på riktig hengslelinje, i stedet for å henge utenpå bakkanten som en løs
// klaff (som så rart ut - roret sto synlig utenfor selve vingeprofilen).
function buildWing(spec, wingMat, darkMat) {
    const group = new THREE.Group();
    const wingChord = spec.wingArea / spec.wingSpan;
    const thickness = 0.038;

    const frontChord = wingChord * 0.65;
    const rearChord = wingChord * 0.35;
    const aileronSpan = spec.wingSpan * 0.22;
    const centerSpan = spec.wingSpan - aileronSpan * 2;

    // Fremre hoveddel - går over hele spennvidden, uavbrutt (bærer hele vingens strukturelle silhuett).
    const wingFront = new THREE.Mesh(new THREE.BoxGeometry(spec.wingSpan, thickness, frontChord), wingMat);
    wingFront.position.z = -wingChord / 2 + frontChord / 2;
    wingFront.castShadow = true;
    group.add(wingFront);

    // Bakre midtdel - stopper FØR vingetuppene, som gir nettopp den utsparingen balanserorene skal fylle.
    const wingRear = new THREE.Mesh(new THREE.BoxGeometry(centerSpan, thickness, rearChord), wingMat);
    wingRear.position.z = wingChord / 2 - rearChord / 2;
    wingRear.castShadow = true;
    group.add(wingRear);

    [-1, 1].forEach(function (side) {
        // Aileron (skråror) - hengslet i utsparingen på vingetuppen, langs samme bakkant-linje som
        // wingRear. Roteres om lokal X-akse i updatePlaneVisual() ut fra faktisk avbøyning.
        const aileronPivot = new THREE.Group();
        aileronPivot.position.set(side * (spec.wingSpan / 2 - aileronSpan / 2), 0, wingFront.position.z + frontChord / 2);
        const aileronMesh = new THREE.Mesh(new THREE.BoxGeometry(aileronSpan * 0.95, thickness * 0.85, rearChord * 0.92), darkMat);
        aileronMesh.position.z = (rearChord * 0.92) / 2;
        aileronMesh.castShadow = true;
        aileronPivot.add(aileronMesh);
        group.add(aileronPivot);
        group.userData["aileron" + side] = aileronPivot;

        // Navigasjonslys i vingetuppen - rødt til venstre, grønt til høyre (ekte flykonvensjon).
        const navLight = new THREE.Mesh(new THREE.SphereGeometry(thickness * 0.7, 8, 6),
            new THREE.MeshStandardMaterial({
                color: side < 0 ? 0xff2a2a : 0x2aff5a,
                emissive: side < 0 ? 0xff2a2a : 0x2aff5a,
                emissiveIntensity: 0.6
            }));
        navLight.position.set(side * (spec.wingSpan / 2 - 0.02), 0, wingFront.position.z);
        group.add(navLight);

        // Vingestrebe ned til skroget (typisk for en høyvinget trener) - tykk nok til faktisk å synes.
        const strut = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 1, 8), darkMat);
        strut.position.set(side * spec.wingSpan * 0.22, 0, 0);
        group.add(strut);
        group.userData["strut" + side] = strut;
    });

    return group;
}

// Bygger flyet prosedyralt (avrundet, konisk skrog, høyvinge, hale, understell, spinner/propell) -
// ingen eksterne modellfiler. Vingespenn/-areal skaleres etter PLANE_CLASSES.
function buildPlane(classKey) {
    const spec = PLANE_CLASSES[classKey];
    const group = new THREE.Group();
    const bodyMat = new THREE.MeshStandardMaterial({ color: 0xe8e8e8 });
    const accentMat = new THREE.MeshStandardMaterial({ color: 0xd23c3c });
    const wingMat = new THREE.MeshStandardMaterial({ color: 0xf2f2f2 });
    const darkMat = new THREE.MeshStandardMaterial({ color: 0x222222 });
    // Lysere/mer transparent enn første forsøk - den mørke fargen (0x1c3a4a @ 0.75 opacity) leste som
    // et sort hull i skroget i skjermbildet, ikke en glassboble.
    const glassMat = new THREE.MeshStandardMaterial({ color: 0x8fb8d0, transparent: true, opacity: 0.55, metalness: 0.1, roughness: 0.1 });

    // Skrog: nesekjegle -> kabin (bredest) -> jevnt avsmalnende bakkropp mot halen, tre sylinderseksjoner
    // med matchende radius i skjøtene i stedet for én enkelt sylinder - gir et langt mer fly-aktig silhuett.
    // cabinRadius redusert ytterligere (0.095 -> 0.07) - skroget leste fortsatt som en "ubåt/luftskip"
    // (for tykk midje i forhold til lengden) i skjermbildene selv etter forrige runde.
    const fuselageLength = 1.35, cabinRadius = 0.07;
    // tailLen kortet inn (0.5 -> 0.38) - halekjeglen stakk ut god margin bak BÅDE høyderorets og
    // sideorets bakkant (tuppen endte over 20% av skroglengden bak rorflatene, som et bart "brodd" uten
    // noen haleflate rundt seg). tailFrontZ/rorflatenes plassering (se lenger ned) er UAVHENGIG av
    // tailLen (avledes av fuselageLength), så rorflatene beholder samme posisjon - kun kjeglen kortes inn.
    const noseLen = fuselageLength * 0.18, cabinLen = fuselageLength * 0.32, tailLen = fuselageLength * 0.38;
    // (radiusTop/radiusBottom var byttet om - nesen buttet ut ved tuppen og snørte seg inn mot kabinen
    // i stedet for å tapre til et punkt, pga. hvordan rotation.x=90° flytter lokal +Y til verdens +Z.)
    const noseSection = new THREE.Mesh(new THREE.CylinderGeometry(cabinRadius, cabinRadius * 0.35, noseLen, 14), bodyMat);
    noseSection.rotation.x = Math.PI / 2;
    noseSection.position.z = -(cabinLen / 2 + noseLen / 2);
    noseSection.castShadow = true;
    group.add(noseSection);

    const cabinSection = new THREE.Mesh(new THREE.CylinderGeometry(cabinRadius, cabinRadius, cabinLen, 14), bodyMat);
    cabinSection.rotation.x = Math.PI / 2;
    cabinSection.castShadow = true;
    group.add(cabinSection);

    const tailSection = new THREE.Mesh(new THREE.CylinderGeometry(cabinRadius * 0.16, cabinRadius, tailLen, 14), bodyMat);
    tailSection.rotation.x = Math.PI / 2;
    tailSection.position.z = cabinLen / 2 + tailLen / 2;
    tailSection.castShadow = true;
    group.add(tailSection);

    // Motorpanser + spinner - markerer overgangen mellom nesekjegle og propell tydeligere enn en bar spiss.
    const cowl = new THREE.Mesh(new THREE.CylinderGeometry(cabinRadius * 0.42, cabinRadius * 0.3, 0.05, 14), accentMat);
    cowl.rotation.x = Math.PI / 2;
    cowl.position.z = -(cabinLen / 2 + noseLen + 0.02);
    cowl.castShadow = true;
    group.add(cowl);
    const spinner = new THREE.Mesh(new THREE.ConeGeometry(cabinRadius * 0.28, 0.09, 12), darkMat);
    spinner.rotation.x = -Math.PI / 2;
    spinner.position.z = -(cabinLen / 2 + noseLen + 0.09);
    spinner.castShadow = true;
    group.add(spinner);

    // Kabinvindu (cockpit) - avlang glassboble over kabinseksjonen. Mindre og flatere enn første forsøk
    // (som leste som en stor sort ball/hull oppå skroget).
    const canopy = new THREE.Mesh(new THREE.SphereGeometry(cabinRadius * 0.85, 10, 8), glassMat);
    canopy.scale.set(1, 0.5, 1.6);
    canopy.position.set(0, cabinRadius * 0.75, -cabinLen * 0.12);
    group.add(canopy);

    // Høyvinge (over skroget, som en typisk skoleflymaskin) - se buildWing.
    const wingChord = spec.wingArea / spec.wingSpan;
    const wingMountY = cabinRadius * 1.3;
    const wing = buildWing(spec, wingMat, darkMat);
    wing.position.set(0, wingMountY, 0.02);
    group.add(wing);
    [-1, 1].forEach(function (side) {
        const strut = wing.userData["strut" + side];
        const strutLen = wingMountY - cabinRadius * 0.25;
        strut.scale.y = strutLen;
        strut.position.y = -strutLen / 2;
    });

    // Hale: fast stabilisator/finne + hengslede rorflater (høyderor/sideror) bakerst - se
    // updatePlaneVisual() for animasjonen. tailFrontZ er forkanten av hele haleflaten.
    const tailSpan = spec.wingSpan * 0.28, tailChord = wingChord * 0.7;
    const tailFrontZ = fuselageLength / 2 - tailChord;

    const stabChord = tailChord * 0.6, elevatorChord = tailChord * 0.45;
    const hStab = new THREE.Mesh(new THREE.BoxGeometry(tailSpan, 0.032, stabChord), wingMat);
    hStab.position.set(0, cabinRadius * 0.3, tailFrontZ + stabChord / 2);
    hStab.castShadow = true;
    group.add(hStab);

    const elevatorPivot = new THREE.Group();
    elevatorPivot.position.set(0, cabinRadius * 0.3, tailFrontZ + stabChord);
    const elevatorMesh = new THREE.Mesh(new THREE.BoxGeometry(tailSpan * 0.9, 0.028, elevatorChord), darkMat);
    elevatorMesh.position.z = elevatorChord / 2;
    elevatorMesh.castShadow = true;
    elevatorPivot.add(elevatorMesh);
    group.add(elevatorPivot);

    // Finnen er hvit/nøytral (samme som resten av skroget) - først forsøkt i aksentrødt, men det leste
    // som en løsrevet, skarpt fargede kloss i skjermbildet i stedet for et naturlig halefinne.
    const finHeight = cabinRadius * 2.3, finChord = tailChord * 0.65, rudderChord = tailChord * 0.5;
    const finBaseY = cabinRadius * 0.55;
    const finFixed = new THREE.Mesh(new THREE.BoxGeometry(0.032, finHeight, finChord), wingMat);
    finFixed.position.set(0, finBaseY + finHeight / 2, tailFrontZ + finChord / 2);
    finFixed.castShadow = true;
    group.add(finFixed);

    const rudderPivot = new THREE.Group();
    rudderPivot.position.set(0, finBaseY + finHeight / 2, tailFrontZ + finChord);
    const rudderMesh = new THREE.Mesh(new THREE.BoxGeometry(0.03, finHeight * 0.85, rudderChord), darkMat);
    rudderMesh.position.z = rudderChord / 2;
    rudderMesh.castShadow = true;
    rudderPivot.add(rudderMesh);
    group.add(rudderPivot);

    // Understellet må gi nok bakkeklaring til hele propellsveipet (se bladeLen under) - ellers stikker
    // propellen ned i rullebanen. gearHeight avledes direkte av spec.gearOffsetY (fysikkens faktiske
    // bakkekontaktpunkt), delt på visualScale siden hele modellen skaleres uniformt til slutt - dermed
    // stemmer det visuelle understellet nøyaktig med fysikkens bakkekontakt for alle tre flystørrelsene,
    // ikke bare den ene klassen en fast konstant tilfeldigvis passet for.
    // Hjulets BUNNPUNKT (senter minus radius), ikke hjulsenteret, må lande nøyaktig på -gearHeight -
    // ellers stikker hjulets underkant synlig ned i rullebanen selv om fysikken sier "på bakken" (dette
    // var årsaken til at hjulene så ut til å "glitche" delvis gjennom asfalten). Derfor stopper strebene
    // ved hjulaksling-høyde (gearHeight - hjulradius), ikke ved selve bakkekontaktpunktet.
    const wheelRadius = 0.04, noseWheelRadius = 0.03;
    const gearHeight = -spec.gearOffsetY / spec.visualScale;
    const gearTrack = spec.wingSpan * 0.22;
    const strutLenMain = gearHeight - wheelRadius;
    [-1, 1].forEach(function (side) {
        const strut = new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.008, strutLenMain, 6), darkMat);
        strut.position.set(side * gearTrack / 2, -strutLenMain / 2, 0.02);
        group.add(strut);
        const wheel = new THREE.Mesh(new THREE.CylinderGeometry(wheelRadius, wheelRadius, 0.025, 14), darkMat);
        wheel.rotation.z = Math.PI / 2;
        wheel.position.set(side * gearTrack / 2, -strutLenMain, 0.02);
        wheel.castShadow = true;
        group.add(wheel);
    });
    const noseGearHeight = gearHeight * 0.85;
    const strutLenNose = noseGearHeight - noseWheelRadius;
    const noseStrut = new THREE.Mesh(new THREE.CylinderGeometry(0.007, 0.007, strutLenNose, 6), darkMat);
    noseStrut.position.set(0, -strutLenNose / 2, -fuselageLength / 2 + 0.1);
    group.add(noseStrut);
    const noseWheel = new THREE.Mesh(new THREE.CylinderGeometry(noseWheelRadius, noseWheelRadius, 0.02, 12), darkMat);
    noseWheel.rotation.z = Math.PI / 2;
    noseWheel.position.set(0, -strutLenNose, -fuselageLength / 2 + 0.1);
    group.add(noseWheel);

    // Propell: to enkle, slanke blad (den forrige "twist"-varianten med et bredere tupp-segment leste
    // som to store, klossete firkanter ytterst i skjermbildene - fjernet til fordel for én enkel, tynn
    // bjelke per blad). bladeLen er verifisert mot minste gearHeight (Stor-klassen) med god klaring.
    const propGroup = new THREE.Group();
    const propMat = new THREE.MeshStandardMaterial({ color: 0x151515 });
    const bladeLen = cabinRadius * 2.0;
    [-1, 1].forEach(function (dir) {
        const blade = new THREE.Mesh(new THREE.BoxGeometry(bladeLen, 0.012, 0.032), propMat);
        blade.position.x = dir * bladeLen / 2;
        propGroup.add(blade);
    });
    propGroup.position.z = -(cabinLen / 2 + noseLen + 0.115);
    group.add(propGroup);

    group.scale.setScalar(spec.visualScale);
    return {
        group: group, propeller: propGroup,
        aileronLeft: wing.userData["aileron-1"], aileronRight: wing.userData["aileron1"],
        elevator: elevatorPivot, rudder: rudderPivot
    };
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

    scene.add(new THREE.AmbientLight(0xffffff, 0.45)); // litt lavere - gir tydeligere kontrast i den ekte skyggekart-skyggen
    const sun = new THREE.DirectionalLight(0xffffff, 0.9);
    sun.position.set(60, 90, 40);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    sun.shadow.camera.left = -100;
    sun.shadow.camera.right = 100;
    sun.shadow.camera.top = 200;
    sun.shadow.camera.bottom = -200;
    sun.shadow.camera.near = 1;
    sun.shadow.camera.far = 400;
    sun.shadow.bias = -0.0015;
    sun.target.position.set(0, 0, RUNWAY_NEAR_Z - RUNWAY_LENGTH / 2);
    scene.add(sun.target);
    scene.add(sun);

    const aspect = window.innerWidth / Math.max(1, window.innerHeight - 70);
    // Nærmere far-plan (1500 er rikelig - himmelkulen har radius 800) gir bedre dybdebuffer-presisjon enn
    // 3000, som reduserer flimring/z-fighting mellom rullebanen og bakkeplanet under den.
    chaseCamera = new THREE.PerspectiveCamera(60, aspect, 0.1, 1500);
    fpvCamera = new THREE.PerspectiveCamera(90, aspect, 0.05, 1500);
    fpvCamera.position.set(0, 0.08, -0.55);
    fpvCamera.rotation.x = THREE.MathUtils.degToRad(settings.fpvTiltDeg);

    rebuildPlaneMesh();

    // VLOS-observatøren står rett ved siden av rullebanen (ikke på den) og ser nedover mot avgangsenden.
    vlosCamera = new THREE.PerspectiveCamera(50, aspect, 0.1, 1500);
    vlosCamera.position.set(RUNWAY_WIDTH / 2 + 4, 1.6, RUNWAY_SPAWN_Z);
    scene.add(vlosCamera);

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

function rebuildPlaneMesh() {
    if (planeGroup) scene.remove(planeGroup);
    const built = buildPlane(planeState.planeClass);
    planeGroup = built.group;
    planePropeller = built.propeller;
    planeAileronLeft = built.aileronLeft;
    planeAileronRight = built.aileronRight;
    planeElevator = built.elevator;
    planeRudder = built.rudder;
    planeGroup.add(fpvCamera);
    scene.add(planeGroup);
}

function resizeRenderer() {
    const wrap = document.querySelector(".sim-page");
    Sim.resizeRenderer(renderer, wrap, [chaseCamera, fpvCamera, vlosCamera]);
}

// Chase-kamera med manuell orbit: hold høyreklikk og dra for å se rundt flyet, scroll for å zoome.
// Vinkel/avstand er en offset OVENPÅ flyets egen heading - dvs. kameraet henger fortsatt bak flyet
// og følger det rundt svinger, men piloten kan se seg rundt (f.eks. for å ta skjermbilder) uten at det
// påvirker selve styringen av flyet.
const CHASE_DEFAULT_DIST = Math.hypot(3.2, 15);
const CHASE_DEFAULT_PITCH = Math.atan2(3.2, 15);
let chaseOrbitYaw = 0;
let chaseOrbitPitch = CHASE_DEFAULT_PITCH;
let chaseZoomDistance = CHASE_DEFAULT_DIST;
let isOrbitingChase = false;
let lastPointerX = 0, lastPointerY = 0;

function updateChaseCamera(dt) {
    const euler = new THREE.Euler().setFromQuaternion(planeState.quaternion, "YXZ");
    const headingQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, euler.y + chaseOrbitYaw, 0));
    const horizontalDist = chaseZoomDistance * Math.cos(chaseOrbitPitch);
    const verticalDist = chaseZoomDistance * Math.sin(chaseOrbitPitch);
    const behindOffset = new THREE.Vector3(0, verticalDist, horizontalDist).applyQuaternion(headingQuat);
    const desiredPos = planeState.position.clone().add(behindOffset);
    const smoothing = 1 - Math.pow(0.0015, dt);
    chaseCamera.position.lerp(desiredPos, smoothing);
    chaseCamera.lookAt(planeState.position.clone().add(new THREE.Vector3(0, 1, 0)));
}

function updateVlosCamera() {
    vlosCamera.lookAt(planeState.position);
}

/* ---------- Input ---------- */
const rawFirstGamepad = Sim.rawFirstGamepad;
function getActiveGamepad() {
    return Sim.getActiveGamepad(settings.inputSource);
}
const readStickAxis = Sim.readStickAxis;
const readThrottleAxis = Sim.readThrottleAxis;

const BUTTON_ACTIONS = {
    kill: toggleEngine,
    modeStabilized: function () { planeState.flightMode = "stabilized"; },
    modeManual: function () { planeState.flightMode = "manual"; }
};
const buttonManager = Sim.createButtonBindingManager(gamepadMap.buttons, BUTTON_ACTIONS, saveGamepadMap);

function adjustTrim(dt, gp) {
    let trim = planeState.elevatorTrimDeg;
    if (keys.has("BracketRight")) trim += TRIM_STEP_PER_SEC * dt;
    if (keys.has("BracketLeft")) trim -= TRIM_STEP_PER_SEC * dt;
    // Kontinuerlig "hold inne"-justering fra fjernkontroll-bryter (ikke stigende-kant som kill/modus).
    if (gp) {
        if (Sim.isBindingActive(gp, gamepadMap.buttons.trimUp)) trim += TRIM_STEP_PER_SEC * dt;
        if (Sim.isBindingActive(gp, gamepadMap.buttons.trimDown)) trim -= TRIM_STEP_PER_SEC * dt;
    }
    planeState.elevatorTrimDeg = clamp(trim, -TRIM_RANGE_DEG, TRIM_RANGE_DEG);
}

function updateInput(dt) {
    const gp = getActiveGamepad();
    adjustTrim(dt, gp);
    if (gp) buttonManager.poll(gp);

    if (gp) {
        inputState.source = "gamepad";
        inputState.stick.roll = readStickAxis(gp, gamepadMap.aileron);
        inputState.stick.pitch = readStickAxis(gp, gamepadMap.elevator);
        inputState.stick.yaw = readStickAxis(gp, gamepadMap.rudder);
        inputState.stick.throttle = readThrottleAxis(gp, gamepadMap.throttle);
        updateGamepadAxesReadout(gp);
        return;
    }
    inputState.source = "keyboard";
    const rollTarget = (keys.has("KeyD") ? 1 : 0) - (keys.has("KeyA") ? 1 : 0);
    const pitchTarget = (keys.has("KeyW") ? 1 : 0) - (keys.has("KeyS") ? 1 : 0);
    const yawTarget = (keys.has("KeyE") ? 1 : 0) - (keys.has("KeyQ") ? 1 : 0);
    inputState.stick.roll = rampStick(inputState.stick.roll, rollTarget, dt, STICK_RAMP_TIME);
    inputState.stick.pitch = rampStick(inputState.stick.pitch, pitchTarget, dt, STICK_RAMP_TIME);
    inputState.stick.yaw = rampStick(inputState.stick.yaw, yawTarget, dt, STICK_RAMP_TIME);

    let throttle = inputState.stick.throttle;
    if (keys.has("ShiftLeft") || keys.has("ShiftRight")) throttle += THROTTLE_RATE * dt;
    if (keys.has("ControlLeft") || keys.has("ControlRight")) throttle -= THROTTLE_RATE * dt;
    inputState.stick.throttle = clamp(throttle, 0, 1);
    updateGamepadAxesReadout(null);
}

/* ---------- Aerodynamikk: løft/drag-koeffisienter med steiling ---------- */
// Lineær region opp til kritisk vinkel, så et raskt kollapsende overgangssjikt (klassisk steile-oppførsel),
// og til slutt en grov "flat plate"-tilnærming for dyp steiling (sin(2*AoA), typisk flate-plate-normalkraft).
function liftCoefficient(aoaDeg, spec) {
    const stall = spec.stallAngleDeg;
    const absA = Math.abs(aoaDeg);
    const sign = aoaDeg < 0 ? -1 : 1;
    const peak = spec.clSlope * stall;
    if (absA < stall) return spec.clSlope * aoaDeg;
    if (absA < stall + STALL_POST_RANGE_DEG) {
        const progress = (absA - stall) / STALL_POST_RANGE_DEG;
        return sign * (peak * (1 - progress) + peak * 0.3 * progress);
    }
    return sign * peak * 0.6 * Math.abs(Math.sin(2 * THREE.MathUtils.degToRad(absA)));
}

function dragCoefficient(aoaDeg, spec) {
    const aoaRad = THREE.MathUtils.degToRad(aoaDeg);
    let cd = spec.cd0 + spec.inducedDragK * aoaRad * aoaRad;
    if (Math.abs(aoaDeg) > spec.stallAngleDeg) cd += 0.3 * Math.abs(Math.sin(aoaRad));
    return cd;
}

// Lokal luftfart/AoA for én vingetupp: legger til hastighetsbidraget fra rull/gir-rotasjon (omega x r) -
// dette er selve mekanismen som lar innervingen i en hard, sakte sving steile før den ytre og få flyet
// til å falle/rulle brått mot den siden (ekte "tip stall"/spinn-tendens), i stedet for at hele flyet
// alltid steiler symmetrisk og jevnt.
function wingLocalAirspeedAoa(localAirVelCG, rotContribLocal) {
    const v = localAirVelCG.clone().add(rotContribLocal);
    const airspeed = v.length();
    const aoaDeg = airspeed > 0.3 ? THREE.MathUtils.radToDeg(Math.atan2(-v.y, -v.z)) : 0;
    return { airspeed: airspeed, aoaDeg: aoaDeg };
}

/* ---------- Fysikk ---------- */
function stepPhysics(dt) {
    const spec = currentPlaneSpec();

    // Sikkerhetsnett mot enhver uforklarlig dreining i stillstand (parkert/rett etter reset/spawn, før
    // flyet har fart): tvinger orienteringen HELT til identitet (rett ned rullebanen, nivå) i stedet for
    // å bare bevare "gjeldende" gir-retning som forrige versjon gjorde - hvis gir-verdien allerede var
    // feil av en eller annen grunn, ble den feilen bare "låst fast" i stedet for rettet opp. Kjøres
    // FØRST i hver fysikk-tick, uavhengig av bakkekontakt-grenen.
    if (planeState.velocity.length() < 0.4) {
        planeState.quaternion.identity();
        planeState.angularVelocity.roll = 0;
        planeState.angularVelocity.pitch = 0;
        planeState.angularVelocity.yaw = 0;
    }

    if (planeState.crashed) {
        planeState.angularVelocity.pitch *= PASSIVE_ANGULAR_DAMPING;
        planeState.angularVelocity.roll *= PASSIVE_ANGULAR_DAMPING;
        planeState.angularVelocity.yaw *= PASSIVE_ANGULAR_DAMPING;
        const angVelVec = new THREE.Vector3(planeState.angularVelocity.pitch, planeState.angularVelocity.yaw, planeState.angularVelocity.roll);
        integrateOrientation(planeState.quaternion, angVelVec, dt);
        planeState.velocity.y -= GRAVITY * dt;
        planeState.position.add(planeState.velocity.clone().multiplyScalar(dt));
        resolveGroundContact(dt);
        return;
    }

    const stick = inputState.stick;
    const throttleShaped = computeThrottleCurve(stick.throttle, rates.throttle.expo);
    const thrustForce = planeState.engineOn ? throttleShaped * spec.maxThrust : 0;

    const q = planeState.quaternion;
    const invQ = q.clone().invert();
    const airVelWorld = planeState.velocity.clone().sub(currentWindVector);
    const localAirVel = airVelWorld.clone().applyQuaternion(invQ);
    const airspeed = airVelWorld.length();
    lastAirspeed = airspeed;
    const aoaDeg = airspeed > 0.3 ? THREE.MathUtils.radToDeg(Math.atan2(-localAirVel.y, -localAirVel.z)) : 0;
    const sideslipDeg = airspeed > 0.3 ? THREE.MathUtils.radToDeg(Math.atan2(localAirVel.x, -localAirVel.z)) : 0;

    // Venstre/høyre vinge separat (se wingLocalAirspeedAoa) - gir differensiallift som blir til rulldreiemoment.
    const halfSpan = spec.wingSpan / 2;
    const rollRate = planeState.angularVelocity.roll;
    const yawRate = planeState.angularVelocity.yaw;
    const rightRot = new THREE.Vector3(0, rollRate * halfSpan, -yawRate * halfSpan);
    const leftRot = new THREE.Vector3(0, -rollRate * halfSpan, yawRate * halfSpan);
    const rightWing = wingLocalAirspeedAoa(localAirVel, rightRot);
    const leftWing = wingLocalAirspeedAoa(localAirVel, leftRot);

    const halfWingArea = spec.wingArea / 2;
    const liftRight = 0.5 * AIR_DENSITY * rightWing.airspeed * rightWing.airspeed * halfWingArea * liftCoefficient(rightWing.aoaDeg, spec);
    const liftLeft = 0.5 * AIR_DENSITY * leftWing.airspeed * leftWing.airspeed * halfWingArea * liftCoefficient(leftWing.aoaDeg, spec);
    const totalLiftMag = liftRight + liftLeft;
    const rollTorqueFromLift = (liftRight - liftLeft) * halfSpan;

    const qDynTotal = 0.5 * AIR_DENSITY * airspeed * airspeed * spec.wingArea;
    const dragMag = qDynTotal * dragCoefficient(aoaDeg, spec);
    const yawTorqueFromSideslip = -spec.yawStability * sideslipDeg;

    // Retninger i verdensrom via vektor-projeksjon (ikke lokal-akse trigonometri): løft står alltid
    // vinkelrett på luftfarten, i planet som inneholder kroppens "opp"-akse - robust mot fortegnsfeil.
    const airVelDirWorld = airspeed > 0.3 ? airVelWorld.clone().normalize() : new THREE.Vector3(0, 0, -1).applyQuaternion(q);
    const bodyUpWorld = new THREE.Vector3(0, 1, 0).applyQuaternion(q);
    const liftDirWorld = bodyUpWorld.clone().sub(airVelDirWorld.clone().multiplyScalar(bodyUpWorld.dot(airVelDirWorld)));
    if (liftDirWorld.lengthSq() > 1e-6) liftDirWorld.normalize(); else liftDirWorld.set(0, 1, 0);
    const dragDirWorld = airVelDirWorld.clone().multiplyScalar(-1);

    const forwardWorld = new THREE.Vector3(0, 0, -1).applyQuaternion(q);
    const thrustVec = forwardWorld.clone().multiplyScalar(thrustForce);
    const liftVec = liftDirWorld.multiplyScalar(totalLiftMag);
    const dragVec = dragDirWorld.multiplyScalar(dragMag);
    const gravityVec = new THREE.Vector3(0, -spec.mass * GRAVITY, 0);
    const accel = new THREE.Vector3().add(thrustVec).add(liftVec).add(dragVec).add(gravityVec).multiplyScalar(1 / spec.mass);

    // Rorenes avbøyning (-1..1): Manual = direkte fra pinnen, formet av rate-kurven (her brukt som en
    // avbøynings-/følsomhetskurve, normalisert mot maxRate - beholder samme rate-panel-UI som quaden,
    // men uten å late som pinnen kommanderer en rotasjonshastighet direkte). Stabilized er en enkel
    // autopilot som avbøyer AKKURAT DE SAMME rorflatene proporsjonalt med vinkelavviket i stedet for
    // direkte fra pinnen - ikke en egen "selvnivelleringskraft".
    let rollDeflection, pitchDeflection, yawDeflection;
    if (planeState.flightMode === "manual") {
        // Trim TREKKES fra (ikke legges til) - currentPitchDeg/effElevStick-konvensjonen her er positiv=
        // nese NED (se merknad ved drawFpvHorizon), mens elevatorTrimDeg selv skal bety positiv=nese OPP
        // (matcher HUD-visningen "+x.x°" og "Trim opp"-knappen/bryteren) - derav minustegnet.
        const effElevStick = clamp(stick.pitch - planeState.elevatorTrimDeg / TRIM_RANGE_DEG, -1, 1);
        rollDeflection = clamp(computeRate(stick.roll, rates.aileron) / rates.aileron.maxRate, -1, 1);
        pitchDeflection = clamp(computeRate(effElevStick, rates.elevator) / rates.elevator.maxRate, -1, 1);
        yawDeflection = clamp(computeRate(stick.yaw, rates.rudder) / rates.rudder.maxRate, -1, 1);
    } else {
        const euler = new THREE.Euler().setFromQuaternion(q, "YXZ");
        const currentPitchDeg = -THREE.MathUtils.radToDeg(euler.x);
        const currentBankDeg = -THREE.MathUtils.radToDeg(euler.z);
        const targetBankDeg = stick.roll * MAX_BANK_ANGLE;
        // Samme fortegnskorreksjon som i Manual over - trim trekkes fra slik at positiv elevatorTrimDeg
        // konsekvent betyr "nese opp" i begge flymodus, ikke bare i én av dem.
        const targetPitchDeg = stick.pitch * MAX_PITCH_ANGLE - planeState.elevatorTrimDeg;
        rollDeflection = clamp((targetBankDeg - currentBankDeg) / STABILIZED_BANK_AUTHORITY_DEG, -1, 1);
        pitchDeflection = clamp((targetPitchDeg - currentPitchDeg) / STABILIZED_PITCH_AUTHORITY_DEG, -1, 1);
        const manualYaw = computeRate(stick.yaw, rates.rudder) / rates.rudder.maxRate;
        yawDeflection = clamp(manualYaw + RUDDER_COORDINATION_GAIN * targetBankDeg / MAX_BANK_ANGLE, -1, 1);
    }
    // Lagres for den visuelle animasjonen av rorflatene (se updatePlaneVisual) - kjøres i rendring-loopet,
    // ikke i det faste fysikk-tidssteget, så den leser siste beregnede verdi herfra.
    planeState.lastRollDeflection = rollDeflection;
    planeState.lastPitchDeflection = pitchDeflection;
    planeState.lastYawDeflection = yawDeflection;

    // Dreiemoment = avbøyning * dynamisk trykk * kontrolleffektivitet, dempet av et moment proporsjonalt
    // med flyets egen vinkelhastighet (aerodynamisk demping). Begge ledd skalerer med V^2 - svaret blir
    // derfor naturlig trått/"mushy" ved lav fart og fast ved høy fart, uten noen kunstig svekkingsfaktor.
    // Samme aksekonvensjon-negasjon (forward=-Z) som resten av fysikken/quad-simulatoren.
    const qDynControl = 0.5 * AIR_DENSITY * airspeed * airspeed;

    // Ikke-dempings-momenter (kontroll + løft-/sideslip-/fartsmoment-bidrag) integreres eksplisitt,
    // som før.
    const rollTorqueNoDamp = -rollDeflection * qDynControl * ROLL_CONTROL_EFFECTIVENESS + rollTorqueFromLift;
    // Nese-opp-tendens som vokser med farten - se SPEED_PITCH_MOMENT_COEFF - krever aktiv
    // om-trimming/rorkorreksjon i Manual når farten endres, i stedet for at flyet er "for alltid i
    // vater" med samme utslag uansett hvor fort det flyr.
    const pitchTorqueNoDamp = -pitchDeflection * qDynControl * PITCH_CONTROL_EFFECTIVENESS - SPEED_PITCH_MOMENT_COEFF * qDynControl;
    const yawTorqueNoDamp = -yawDeflection * qDynControl * YAW_CONTROL_EFFECTIVENESS + yawTorqueFromSideslip;

    planeState.angularVelocity.roll += (rollTorqueNoDamp / spec.inertiaRoll) * dt;
    planeState.angularVelocity.pitch += (pitchTorqueNoDamp / spec.inertiaPitch) * dt;
    planeState.angularVelocity.yaw += (yawTorqueNoDamp / spec.inertiaYaw) * dt;

    // Aerodynamisk demping integreres IMPLISITT (eksponentiell nedgang: v *= e^(-k*dt)) i stedet for
    // eksplisitt Euler (v += -k*v*dt). Ved høy fart (qDyn stor, spesielt kombinert med lav treghet på
    // Liten-klassen) blir k*dt/treghet > 1 - eksplisitt Euler overskyter da null og SNUR fortegn hver
    // eneste fysikk-tick, som viser seg som en synlig hakking/jitter fram og tilbake i rulleaksen ved
    // høy fart. Eksponentiell demping er ubetinget stabil (nærmer seg alltid null, snur aldri fortegn)
    // uansett hvor stort k*dt/treghet blir, og gir SAMME steady-state-oppførsel som før.
    const rollDampDecay = Math.exp(-(qDynControl * ROLL_DAMPING / spec.inertiaRoll) * dt);
    const pitchDampDecay = Math.exp(-(qDynControl * PITCH_DAMPING / spec.inertiaPitch) * dt);
    const yawDampDecay = Math.exp(-(qDynControl * YAW_DAMPING / spec.inertiaYaw) * dt);
    planeState.angularVelocity.roll *= rollDampDecay;
    planeState.angularVelocity.pitch *= pitchDampDecay;
    planeState.angularVelocity.yaw *= yawDampDecay;

    planeState.velocity.add(accel.clone().multiplyScalar(dt));
    planeState.position.add(planeState.velocity.clone().multiplyScalar(dt));

    const angVelVec = new THREE.Vector3(planeState.angularVelocity.pitch, planeState.angularVelocity.yaw, planeState.angularVelocity.roll);
    integrateOrientation(planeState.quaternion, angVelVec, dt);

    resolveGroundContact(dt);
    checkTailStrike(spec);
}

// Halepunktet - litt bak og litt høyere enn hovedhjulene (se resolveGroundContact) - er det som først
// treffer bakken ved overrotasjon på avgang eller for hard "flare" ved landing. Viser et varsel (se
// tailstrikeWarningUntil/updateHud) - dette er KUN et varsel, ikke en automatisk krasj, siden brukeren
// ba spesifikt om et varsel og ikke nødvendigvis at flyet skal bli ukontrollerbart av det.
let tailStruckCurrently = false;
let tailstrikeWarningUntil = 0;
function checkTailStrike(spec) {
    const tailLocal = new THREE.Vector3(0, spec.gearOffsetY * 0.5, spec.wingSpan * 0.32);
    const tailWorld = tailLocal.applyQuaternion(planeState.quaternion).add(planeState.position);
    if (tailWorld.y <= 0.01) {
        if (!tailStruckCurrently) {
            tailstrikeWarningUntil = performance.now() + 2500;
        }
        tailStruckCurrently = true;
    } else if (tailWorld.y > GROUND_CLEARANCE_FW * 3) {
        tailStruckCurrently = false;
    }
}

// Ett understellspunkt per side (hovedhjul) - flat rullebane, ingen kant-/veggkollisjon nødvendig her
// (i motsetning til quad-banen). Hard landing (høy synkefart eller stor krengevinkel ved berøring) gir
// "krasjet"-tilstand; ellers demper kontakten synk/krengning og bremser med rullemotstand.
function resolveGroundContact(dt) {
    const spec = currentPlaneSpec();
    const gearTrack = spec.wingSpan * 0.25;
    const localPoints = [
        new THREE.Vector3(gearTrack / 2, spec.gearOffsetY, 0.05),
        new THREE.Vector3(-gearTrack / 2, spec.gearOffsetY, 0.05)
    ];
    const worldPoints = localPoints.map(function (p) {
        return p.clone().applyQuaternion(planeState.quaternion).add(planeState.position);
    });
    let maxPenetration = -Infinity;
    worldPoints.forEach(function (p) { maxPenetration = Math.max(maxPenetration, -p.y); });

    // VIKTIG: sjekker mot 0 (faktisk gjennomtrengning), ikke mot -GROUND_CLEARANCE_FW. Det siste var en
    // reell bug - det behandlet ALT innenfor en 5 cm klaringssone som "på bakken" og trakk flyet ned mot
    // bakken igjen (se position.y += maxPenetration under) SELV når hjulene allerede var et par cm klar.
    // I praksis "sugde" dette flyet ned igjen gjentatte ganger under selve avgangen, helt til det klarte
    // å hoppe HELE 5 cm-sonen i ett eneste fysikk-tidssteg - noe som gjorde en helt normal, gradvis
    // lettelse (som fungerer fint i luften et par sekunder senere) unaturlig vanskelig under selve
    // rulling/rotasjon på rullebanen.
    if (maxPenetration <= 0) {
        planeState.onGround = false;
        return;
    }

    if (!planeState.onGround) {
        const euler = new THREE.Euler().setFromQuaternion(planeState.quaternion, "YXZ");
        const bankDeg = Math.abs(-THREE.MathUtils.radToDeg(euler.z));
        if (planeState.velocity.y < -CRASH_SINK_RATE || bankDeg > CRASH_BANK_DEG) {
            planeState.crashed = true;
        }
    }
    planeState.onGround = true;
    planeState.position.y += maxPenetration;
    if (planeState.velocity.y < 0) planeState.velocity.y *= 0.15;

    // Retter kun opp KRENGNING (roll) mot null mens hjulene er på bakken - pitch skal IKKE tvinges
    // til null her, ellers kan flyet aldri rotere nesen opp for å lette (eller flare ved landing).
    const euler = new THREE.Euler().setFromQuaternion(planeState.quaternion, "YXZ");
    const levelRollQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(euler.x, euler.y, 0, "YXZ"));
    planeState.quaternion.slerp(levelRollQuat, Math.min(1, 4 * dt));
    planeState.angularVelocity.roll *= 0.5;
    // Hjulene har rotasjonsfriksjon mot underlaget - uten denne demperen ville en gir-rotasjon (yaw)
    // fortsette for evig i stillstand med motoren av, siden det aerodynamiske dempeleddet i stepPhysics
    // (som skalerer med farten i andre potens) blir null når flyet står stille.
    planeState.angularVelocity.yaw *= 0.85;

    // Nesehjulet hindrer fysisk at flyet "tipper frem" (roterer nesa ned) mens hovedhjulene er i
    // bakken - i et ekte trehjulsunderstell er dette geometrisk umulig siden nesehjulet ville truffet
    // bakken først. Egen kontaktpunkt foran CG, samme dybde (gearOffsetY) som hovedhjulene - løftes opp
    // akkurat nok til at det aldri synker under bakken. Kjøres KUN inne i denne (allerede onGround-
    // bekreftede) grenen, så et bratt stup høyt over bakken kan aldri feilaktig utløse dette.
    const noseLocal = new THREE.Vector3(0, spec.gearOffsetY, -spec.wingSpan * 0.3);
    const noseWorld = noseLocal.clone().applyQuaternion(planeState.quaternion).add(planeState.position);
    if (noseWorld.y < 0) {
        planeState.position.y -= noseWorld.y;
    }

    const forwardWorld = new THREE.Vector3(0, 0, -1).applyQuaternion(planeState.quaternion);
    const forwardSpeed = planeState.velocity.dot(forwardWorld);
    if (forwardSpeed > 0.05) {
        planeState.velocity.addScaledVector(forwardWorld, -ROLLING_FRICTION * GRAVITY * dt);
    }
    const rightWorld = new THREE.Vector3(1, 0, 0).applyQuaternion(planeState.quaternion);
    const lateralSpeed = planeState.velocity.dot(rightWorld);
    planeState.velocity.addScaledVector(rightWorld, -lateralSpeed * Math.min(1, 8 * dt));
}

function resetPlane() {
    planeState.position.set(0, 0.3, RUNWAY_SPAWN_Z);
    planeState.velocity.set(0, 0, 0);
    planeState.quaternion.identity();
    planeState.angularVelocity.pitch = 0;
    planeState.angularVelocity.roll = 0;
    planeState.angularVelocity.yaw = 0;
    planeState.engineOn = true;
    planeState.crashed = false;
    planeState.onGround = true;
    planeState.elevatorTrimDeg = 0;
    inputState.stick.throttle = 0;
}

function toggleEngine() {
    if (planeState.crashed) return;
    planeState.engineOn = !planeState.engineOn;
}

function toggleCamera() {
    cameraModeIndex = (cameraModeIndex + 1) % CAMERA_MODES.length;
    const mode = CAMERA_MODES[cameraModeIndex];
    activeCamera = (mode === "chase") ? chaseCamera : (mode === "fpv") ? fpvCamera : vlosCamera;
}

/* ---------- Visuell oppdatering + HUD ---------- */
const SURFACE_MAX_DEFLECTION_RAD = THREE.MathUtils.degToRad(22);

function updatePlaneVisual(dt) {
    planeGroup.position.copy(planeState.position);
    planeGroup.quaternion.copy(planeState.quaternion);

    // Propell-treghet: en elektrisk motor gir nesten momentant turtallsøkning med gasspådrag, men
    // propellen (+ motorens egen rotasjonstreghet) coaster ned over ca. 1 sekund når motoren kuttes -
    // ikke et brått stopp slik det var før.
    const targetSpin = planeState.engineOn ? (6 + inputState.stick.throttle * 90) : (planeState.onGround ? 0 : 2);
    const spinSmoothing = planeState.engineOn ? (1 - Math.pow(0.0005, dt)) : (1 - Math.pow(0.05, dt));
    propSpinSpeed += (targetSpin - propSpinSpeed) * spinSmoothing;
    planePropeller.rotation.z += propSpinSpeed * dt;

    // Synlige, bevegelige rorflater - viser faktisk pinne-/autopilot-utslag (se stepPhysics, som lagrer
    // siste avbøyning på planeState hver fysikk-tick).
    // (Fortegn snudd - rorene beveget seg motsatt vei av forventet.)
    planeAileronLeft.rotation.x = planeState.lastRollDeflection * SURFACE_MAX_DEFLECTION_RAD;
    planeAileronRight.rotation.x = -planeState.lastRollDeflection * SURFACE_MAX_DEFLECTION_RAD;
    planeElevator.rotation.x = planeState.lastPitchDeflection * SURFACE_MAX_DEFLECTION_RAD;
    planeRudder.rotation.y = planeState.lastYawDeflection * SURFACE_MAX_DEFLECTION_RAD;
}

const hudMode = document.getElementById("hudMode");
const hudArmed = document.getElementById("hudArmed");
const hudInput = document.getElementById("hudInput");
const hudCamera = document.getElementById("hudCamera");
const hudPlaneClass = document.getElementById("hudPlaneClass");
const hudAltitude = document.getElementById("hudAltitude");
const hudAirspeed = document.getElementById("hudAirspeed");
const hudThrottle = document.getElementById("hudThrottle");
const hudTrim = document.getElementById("hudTrim");
const armToggleBtn = document.getElementById("armToggleBtn");
const crashBanner = document.getElementById("crashBanner");
const tailstrikeBanner = document.getElementById("tailstrikeBanner");

function updateHud() {
    hudMode.textContent = MODE_LABELS[planeState.flightMode];
    const engineLabel = planeState.crashed ? "Krasjet" : (planeState.engineOn ? "På" : "Av");
    hudArmed.textContent = engineLabel;
    hudArmed.className = "sim-status-value " + ((!planeState.crashed && planeState.engineOn) ? "sim-armed" : "sim-killed");
    crashBanner.classList.toggle("show", planeState.crashed);
    tailstrikeBanner.classList.toggle("show", performance.now() < tailstrikeWarningUntil);
    hudInput.textContent = inputState.source === "gamepad" ? "Gamepad" : "Tastatur";
    hudCamera.textContent = CAMERA_MODE_LABELS[CAMERA_MODES[cameraModeIndex]];
    hudPlaneClass.textContent = currentPlaneSpec().label.split(" ")[0];
    const altitude = Math.max(0, planeState.position.y);
    hudAltitude.textContent = (altitude >= 10 ? Math.round(altitude) : altitude.toFixed(1)) + " m";
    hudAirspeed.textContent = lastAirspeed.toFixed(1) + " m/s";
    hudThrottle.textContent = Math.round(inputState.stick.throttle * 100) + " %";
    const trimText = (planeState.elevatorTrimDeg >= 0 ? "+" : "") + planeState.elevatorTrimDeg.toFixed(1) + "°";
    hudTrim.textContent = trimText;
    armToggleBtn.innerHTML = '<i class="fa-solid fa-power-off"></i> ' + (planeState.engineOn ? "Motor av (K)" : "Motor på (K)");

    const trimInputEl = document.getElementById("trimInput");
    if (trimInputEl && document.activeElement !== trimInputEl) {
        trimInputEl.value = planeState.elevatorTrimDeg;
    }
    const trimValueEl = document.getElementById("trimValue");
    if (trimValueEl) trimValueEl.textContent = trimText;
}

/* ---------- Paneler (rates / fly-kamera / vind / gamepad / hjelp) ---------- */
const ALL_PANEL_IDS = ["ratesPanel", "flyCameraPanel", "windPanel", "gamepadPanel", "helpPanel"];
function togglePanel(panel) {
    Sim.togglePanel(panel, ALL_PANEL_IDS.map(function (id) { return document.getElementById(id); }));
}

function buildRatesPanel() {
    const grid = document.getElementById("ratesGrid");
    grid.innerHTML = "";
    ["aileron", "elevator", "rudder"].forEach(function (axis) {
        grid.appendChild(Sim.buildRateAxisBox(rates[axis], AXIS_LABELS[axis], saveRates));
    });
    grid.appendChild(Sim.buildThrottleExpoBox(
        rates.throttle,
        "Gass",
        "0 = lineær gass. Høyere verdi gir finere kontroll nær midten, mer kraftfull respons ved fullt utslag.",
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

function drawFpvHorizon(ctx, w, h) {
    const euler = new THREE.Euler().setFromQuaternion(planeState.quaternion, "YXZ");
    // MERK: pitch-fortegnet her er IKKE negert (i motsetning til currentPitchDeg i stepPhysics, som
    // styrer Stabilized-loopen og er verifisert riktig gjennom mye testing). Denne funksjonen tegner
    // KUN den kunstige horisonten, og med samme negasjon som kontrolloven viste den seg å gå motsatt
    // vei av faktisk stigning/duving - derfor eget (uavhengig) fortegn kun for denne tegningen.
    const pitchDeg = THREE.MathUtils.radToDeg(euler.x);
    const rollDeg = -THREE.MathUtils.radToDeg(euler.z);
    Sim.drawFpvHorizonFromAngles(ctx, w, h, pitchDeg, rollDeg);
}

// Krysshåret skal alltid vise flyets EKTE 0-graders retning (nesen rett frem), uansett hvilken vinkel
// FPV-kameraet selv er montert i (fpvTiltDeg). Et kamera montert f.eks. 10° ned ser "lavere" enn nesen
// peker - da må krysshåret flyttes OPP på skjermen for fortsatt å vise den ekte null-referansen, ikke
// bare sitte fastlåst i bildesenteret (som i praksis ville vist kameraets EGEN retning, ikke flyets).
function fpvCrosshairOffsetPx(h) {
    const tiltRad = THREE.MathUtils.degToRad(settings.fpvTiltDeg);
    const halfFovRad = THREE.MathUtils.degToRad(fpvCamera.fov / 2);
    return (h / 2) * (Math.tan(tiltRad) / Math.tan(halfFovRad));
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
    fpvHudCtx.save();
    fpvHudCtx.translate(0, -fpvCrosshairOffsetPx(h));
    drawFpvCrosshair(fpvHudCtx, w, h);
    fpvHudCtx.restore();
}

function updateWindsockVisual(now) {
    windsockHandles.forEach(function (h) {
        Sim.updateWindsockVisual(h, now, currentWindVector);
    });
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

    updatePlaneVisual(frameDt);
    updateChaseCamera(frameDt);
    updateVlosCamera();
    updateWindsockVisual(now);
    updateHud();
    updateFpvHud();
    renderer.render(scene, activeCamera);
}

/* ---------- Oppstart ---------- */
document.addEventListener("DOMContentLoaded", function () {
    initScene();
    initFpvHudCanvas();
    document.getElementById("fpvHudBtn").innerHTML =
        '<i class="fa-solid fa-crosshairs"></i> OSD: ' + FPV_HUD_MODE_LABELS[settings.fpvHudMode] + " (O)";
    buildRatesPanel();

    document.getElementById("resetBtn").addEventListener("click", resetPlane);
    document.getElementById("armToggleBtn").addEventListener("click", toggleEngine);

    const settingsMenuEl = document.getElementById("settingsMenu");
    Sim.setupDropdown(document.getElementById("settingsToggleBtn"), settingsMenuEl,
        ["ratesPanel", "flyCameraPanel", "windPanel", "gamepadPanel"].map(function (id) { return document.getElementById(id); }));
    Sim.wirePanelCloseButtons(settingsMenuEl);
    function closeSettingsMenu() { settingsMenuEl.classList.remove("open"); }

    document.getElementById("toggleRatesBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("ratesPanel"));
        closeSettingsMenu();
    });
    document.getElementById("toggleFlyCameraBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("flyCameraPanel"));
        closeSettingsMenu();
    });
    document.getElementById("toggleWindBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("windPanel"));
        closeSettingsMenu();
    });
    document.getElementById("toggleGamepadBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("gamepadPanel"));
        closeSettingsMenu();
    });
    document.getElementById("toggleHelpBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("helpPanel"));
    });
    document.getElementById("fpvHudBtn").addEventListener("click", toggleFpvHud);

    const planeClassSelect = document.getElementById("planeClassSelect");
    Object.keys(PLANE_CLASSES).forEach(function (key) {
        const opt = document.createElement("option");
        opt.value = key;
        opt.textContent = PLANE_CLASSES[key].label;
        if (key === planeState.planeClass) opt.selected = true;
        planeClassSelect.appendChild(opt);
    });
    planeClassSelect.addEventListener("change", function () {
        setPlaneClass(planeClassSelect.value);
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

    const trimInput = document.getElementById("trimInput");
    trimInput.value = planeState.elevatorTrimDeg;
    trimInput.addEventListener("input", function () {
        planeState.elevatorTrimDeg = clamp(parseFloat(trimInput.value), -TRIM_RANGE_DEG, TRIM_RANGE_DEG);
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

    // Chase-kamera orbit: høyreklikk+dra ser rundt flyet, scroll zoomer. Kun aktiv over selve
    // canvas-en (ikke HUD-knappene), og hindrer nettleserens høyreklikk-kontekstmeny på canvas.
    const simCanvas = document.getElementById("simCanvas");
    simCanvas.addEventListener("contextmenu", function (e) { e.preventDefault(); });
    simCanvas.addEventListener("mousedown", function (e) {
        if (e.button !== 2) return;
        isOrbitingChase = true;
        lastPointerX = e.clientX;
        lastPointerY = e.clientY;
    });
    window.addEventListener("mouseup", function (e) {
        if (e.button === 2) isOrbitingChase = false;
    });
    window.addEventListener("mousemove", function (e) {
        if (!isOrbitingChase) return;
        const dx = e.clientX - lastPointerX;
        const dy = e.clientY - lastPointerY;
        lastPointerX = e.clientX;
        lastPointerY = e.clientY;
        chaseOrbitYaw -= dx * 0.006;
        chaseOrbitPitch = clamp(chaseOrbitPitch + dy * 0.006, 0.02, 1.4);
    });
    simCanvas.addEventListener("wheel", function (e) {
        e.preventDefault();
        chaseZoomDistance = clamp(chaseZoomDistance + e.deltaY * 0.02, 4, 60);
    }, { passive: false });

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
            case "Digit1": planeState.flightMode = "stabilized"; break;
            case "Digit2": planeState.flightMode = "manual"; break;
            case "KeyK": toggleEngine(); break;
            case "KeyR": resetPlane(); break;
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
