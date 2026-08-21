/* js/simulator-fixedwing.js
   Fixed-wing simulator - gjenbruker matte/kontroll/gamepad/scene-hjelpere fra js/simulator-common.js
   (window.Sim). Fysikkmodellen her er en forenklet, men AoA/steile-basert aerodynamikk-modell
   (løft/drag per vinge, venstre/høyre beregnet separat for realistisk vingedypp/spinn-tendens ved
   steiling i sving), ikke bare en kraft-vektor slik quadcopter-simulatoren bruker. Stigemomentet (pitch)
   er en ekte vekt-og-balanse-modell (se tailArm/tailLift i stepPhysics): halen har sin egen moment-arm
   bak tyngdepunktet og produserer stigemoment fra sitt EGET løft, i stedet for et flatt, håndjustert
   kontrolleffektivitet-tall - dette gir høyderor-effekt, pitch-demping OG fart-avhengig trimbehov som
   alle emergerer fra samme, ene geometriske modell. */

/* ---------- Konstanter: fysikk ---------- */
const GRAVITY = 9.81;
const AIR_DENSITY = 1.225; // kg/m^3, havnivå

// Skrog-mål (bygge-enheter, FØR visualScale) - DELT mellom buildPlane (visuell mesh) og fysikk-sjekker
// som checkTailStrike, slik at de aldri kan drifte ut av synk med hverandre (endres skroget, oppdateres
// tailstrike-punktet automatisk med).
const FUSELAGE_LENGTH_BUILD = 1.35;
const CABIN_RADIUS_BUILD = 0.07;
const CABIN_LEN_RATIO = 0.32;
const TAIL_LEN_RATIO = 0.33; // halekjeglens tupp skal lande omtrent der høyderor/sideror sitter, ikke lenger bak
const TAIL_TIP_RADIUS_RATIO = 0.16;
// Vingens monteringshøyde over CG (andel av cabinRadius) - DELT mellom buildPlane og bakkeeffekt-
// beregningen i stepPhysics, siden bakkeeffekt virker fra VINGEN, ikke fra CG/planeState.position.y
// direkte (denne er høyvinget - vingen sitter tydelig høyere enn CG).
const WING_MOUNT_HEIGHT_RATIO = 1.3;

// I motsetning til quad-simulatorens Acro-modus (som setter ønsket vinkelhastighet direkte) styres et
// ekte fly ALDRI ved å kommandere en rate - pinnen avbøyer en rorflate, og rorflaten skaper et
// dreiemoment proporsjonalt med dynamisk trykk (0.5*rho*V^2), dempet av et motsatt rettet moment fra
// flyets egen vinkelhastighet (aerodynamisk demping). Gir automatisk lav kontrollautoritet ved lav fart
// og fast/hurtig respons ved høy fart, uten noen egen "svekk kontroll"-multiplikator. Steady-state
// rotasjonsrate ved fullt utslag styres av forholdet EFFECTIVENESS/DAMPING (uavhengig av inertia -
// inertia styrer kun hvor RASKT den nås), så disse to må tunes sammen, ikke hver for seg.
const ROLL_CONTROL_EFFECTIVENESS = 0.24;
const ROLL_DAMPING = 0.03;
// Skråror (aileron) endrer nå OGSÅ vingens EFFEKTIVE angrepsvinkel direkte (se rightWing/leftWing i
// stepPhysics), i tillegg til den flate ROLL_CONTROL_EFFECTIVENESS over. Gir to realistiske effekter
// gratis: (1) "adverse yaw" - vingen med nedadgående klaff får mer løft OG mer indusert drag, som girer
// MOTSATT rullretningen (derfor trenger koordinerte svinger sideror), og (2) mer realistisk spinn/
// autorotasjon - samme differensial-drag-mekanisme oppstår fra selve ROTASJONEN, ikke bare rorutslag.
// AILERON_MAX_AOA_DEG (ikke ROLL_CONTROL_EFFECTIVENESS) er den reelle flaskehalsen for rull-autoritet:
// vinge-modellens egen rate-demping (rollWingDampCoeff) dominerer over den flate ROLL_DAMPING-termen med
// >5x ved marsjfart, så bare denne (som virker gjennom SAMME vinge-mekanisme) monner.
const AILERON_MAX_AOA_DEG = 22;
// Dihedral-effekt (Clβ - rull fra SIDESLIP-VINKEL, ikke gir-rate) - se rollTorqueFromDihedral i
// stepPhysics. VEDVARENDE (ikke rate-avhengig som Clr/Cnp over): et sideror holdt inne gir et sideslip
// som varer så lenge roret holdes, og dermed et rullmoment som også varer - dette er det som gjør at
// kryssede ror kan holde flyet i en rett, sideslippende "forward slip"-bane. Skalert med qDynControl
// (rekalibrert fra en tidligere u-skalert 0.02 til samme effekt ved ~12 m/s - se stepPhysics).
const DIHEDRAL_EFFECT = 0.02 / 88.2;
// Skrogets sidekraft/sidedrag i sideslip - se fuselageCrossflowDrag i stepPhysics. Skroget er i praksis
// en butt kropp som møter luften på TVERS under sideslip (en "forward slip"/kryssede ror, som dihedral-
// effekten over allerede lar flyet holde), og et flatt legeme på tvers av strømmen gir betydelig
// motstand - det er selve POENGET med manøveren (brattere synkefart uten å øke farten). Uten dette leste
// finnen alene ALL sidekraft/-drag i sideslip, som ga en for lett/svak slip-effekt.
const FUSELAGE_SIDE_AREA_RATIO = 0.3;  // skrogets sideprofil-areal, som andel av vingearealet
const FUSELAGE_SIDE_CD = 1.2;          // typisk normalkraft-koeffisient for et butt/flatt legeme på tvers
// Stigemomentet (pitch) er IKKE lenger et flatt, håndjustert "kontrolleffektivitet"-tall - se
// beregningen av tailArm/tailLift i stepPhysics, som modellerer et ekte vekt-og-balanse-prinsipp:
// halen henger et stykke bak tyngdepunktet og produserer selve stigemomentet fra sitt EGET løft, akkurat
// som i et ekte fly. Dette gir automatisk høyderor-effekt, pitch-demping OG fart-avhengig trimbehov uten
// noen egne håndjusterte konstanter for de tre tingene hver for seg (se kommentaren ved tailArm).
const TAIL_ARM_RATIO = 0.55;      // halens moment-arm fra CG, som andel av vingespennet
const TAIL_AREA_RATIO = 0.22;     // halens areal, som andel av vingearealet (typisk for lette fly)
const TAIL_CL_SLOPE_RATIO = 0.85; // halens løftekurve-helning relativt til hovedvingens
const ELEVATOR_MAX_AOA_DEG = 16;  // grader endring i halens angrepsvinkel ved fullt rorutslag
// Nedvask (downwash) fra vingen treffer halen og reduserer dens EFFEKTIVE AoA-endring når vingens egen
// AoA endres - halen ser i praksis IKKE fri luftstrøm, den sitter i vingens avbøyde nedvask. Uten dette
// var flyet mer stigestabilt enn selve geometrien skulle tilsi (halen "så" hele vinge-AoA-endringen
// direkte). Typisk 30-50% av vingens AoA-endring for et lavt/normalt halehøyde-forhold.
const DOWNWASH_RATIO = 0.4;
// Gir-momentet (yaw) brukte tidligere et flatt, håndjustert par (YAW_CONTROL_EFFECTIVENESS/YAW_DAMPING +
// en egen yawStability-konstant per klasse) - i motsetning til stigemomentet, som er en ekte vekt-og-
// balanse-modell. Erstattet med akkurat samme prinsipp som halen: finnen henger på samme moment-arm
// (TAIL_ARM_RATIO - samme skrogstasjon som halen) og produserer et sideveis "løft" fra sin EGEN lokale
// sideslip-vinkel (som inkluderer gir-ratens rotasjonsbidrag). Dette gir sideror-effekt, gir-demping
// (Cnr) OG retningsstabilitet (Cnβ) fra ÉN geometrisk modell, akkurat som halen ga tre pitch-egenskaper
// fra én modell - i stedet for tre separate, håndjusterte tall.
const FIN_AREA_RATIO = 0.13;      // finnens areal, som andel av vingearealet
const FIN_CL_SLOPE_RATIO = 0.75;  // finnens løftekurve-helning relativt til hovedvingens
// Den naturlige (urettede) sideslip-VINKELEN finnen ser er en ren GEOMETRISK vinkel, uavhengig av selve
// vind-STYRKEN - nærmer seg 90° når fartsvektoren er nesten null (stillestående/utflating), selv i svak
// vind. Fullt utslag må derfor kunne kansellere en naturlig sideslip opp mot 25-35° (klemt til ±35° i
// finAoaDeg), noe et lavere gradtall (18°, tidligere verdi) ikke klarte - påvirker både bakke-
// vindkantring og sideslip-landing likt, siden begge momenter skalerer med samme dynamiske trykk.
const RUDDER_MAX_AOA_DEG = 32;    // grader endring i finnens angrepsvinkel ved fullt sideror-utslag
// Se yawDampCoeff i stepPhysics for utregningen (dempingsforhold ζ≈0.29 uten denne, en tydelig
// underdempet "Dutch roll"-lignende gir-oscillasjon) - en ekte yaw-damper-boost, ikke en fiktiv fiks.
const YAW_DAMPER_GAIN = 3;

// Propellstrøm (propwash/slipstream): halen/finnen sitter rett i propellens luftstrøm, som akselereres
// UAVHENGIG av flyets EGEN bakkefart/luftfart - gir aksielt (0.5*rho*A*v²-momentum-teori, se
// propwashDeltaV i stepPhysics) reell rorautoritet (høyderor/sideror) selv ved lav fart så lenge gassen
// står på, noe som manglet helt før (rorautoriteten kom KUN fra flyets egen luftfart, null der propwash
// trengs mest). Arealet under er IKKE selve propellskiven, men hvor mye slipstrømmen har spredd seg idet
// den når halen/finnen - et for lite areal (f.eks. skivens fysiske ~6%) gir en urealistisk stor statisk
// fartsøkning (over 35 m/s), siden momentum-teoriens v_i går som sqrt(T/A). Kalibrert til ~5 m/s statisk
// fartsøkning for alle klasser (se v_i-formelen i stepPhysics).
const PROPWASH_EFFECTIVE_AREA_RATIO = 3.0;

// Bakkeeffekt: reduserer indusert motstand når VINGEN (ikke CG - se wingHeightAboveGround i stepPhysics)
// er lavere enn ca. ett vingespenn over bakken, og er tydelig merkbar (halvveis til fullt bortfalt) rundt
// et HALVT vingespenn (bakken hindrer vingetupp-virvlene/nedvasken i å utvikle seg fullt ut) - se
// groundEffectFactor i stepPhysics. Dette er det som gir "flyting" (redusert synkefart/lengre utrulling i
// luften rett før touchdown) ved landing, og mindre motstand/kortere rulling rett etter avgang mens flyet
// ennå er lavt. HEIGHT_FACTOR=2 gir nøyaktig halvveis-bortfall ved h/b=0.5 og ~20% effekt igjen ved h/b=1.
const GROUND_EFFECT_HEIGHT_FACTOR = 2; // høyere tall = bakkeeffekten forsvinner raskere med høyde
// Bakkeeffekten gir også en liten LØFT-økning nær bakken (ikke bare redusert drag, se over) - typisk
// 5-10% ved bakkenivå, forsvinner sammen med resten av bakkeeffekten idet høyden nærmer seg ett
// vingespenn. Dette er selve kilden til den karakteristiske "flyter forbi setepunktet"-følelsen ved
// landing (flyet nekter å synke helt til sist selv med gass i tomgang) - kun redusert drag alene gir
// ikke denne følelsen like tydelig.
const GROUND_EFFECT_LIFT_BOOST_MAX = 0.08;
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
// Stabilized var en REN P-regulator (utslag kun proporsjonalt med vinkelavviket) - selv med aerodynamisk
// demping i selve flykroppen kaster en ren P-regulator roret hardt ut helt fram til målet er nådd, noe
// som lett oppleves som "robotisk"/rykkete idet den treffer målet og må reversere brått. Lagt til et
// D-ledd (dempet mot RATEN vinkelen selv endrer seg med, målt direkte via forrige tick sin vinkel - ikke
// via angularVelocity sin egen aksekonvensjon, for å unngå nok en fortegns-utledning) - en vanlig,
// standard PID-forbedring (PD i stedet for ren P) som bremser tilnærmingen FØR den treffer målet.
const STABILIZED_BANK_D_GAIN = 0.02;
const STABILIZED_PITCH_D_GAIN = 0.02;
// D-leddets rate måles via finite-difference over ETT fysikk-tick (1/120s) - uten filter forsterkes enhver
// liten forstyrrelse (bakkekontaktens nivelleringskorreksjon, vinge-modellens ikke-lineære respons) med en
// faktor på 120 idet den differensieres ("derivative kick", et kjent digital-kontroll-fenomen, ikke en
// reell ustabilitet i selve flyfysikken). Lavpassfiltrert (samme idiom som AUTO_TRIM_FILTER_TAU) før bruk -
// kort nok til at ekte demping fortsatt skjer omtrent like raskt, lang nok til å dempe enkelttick-støy.
const STABILIZED_D_FILTER_TAU = 0.12;

const TRIM_RANGE_DEG = 15;      // maks høyderor-trim, begge retninger
const TRIM_STEP_PER_SEC = 6;    // grader/s trim-endring når [ / ] holdes inne (tastatur)
// Bevisst mye saktere enn TRIM_STEP_PER_SEC (manuell trim-tast) - skal føles som et trim-hjul som sakte
// "følger etter" i bakgrunnen, ikke en rask korreksjon. For rask her ville fått auto-trim til å slåss
// mot/overstyre pilotens egne pinne-kommandoer i Stabilized i stedet for kun å avlaste steady-state.
const AUTO_TRIM_RATE_DEG_PER_SEC = 3;
// Auto-trim kjører på et LAVPASSFILTRERT utslag, ikke det momentane pitchDeflection-tallet direkte -
// ellers ville trimmen "jaget" hvert lite, forbigående korreksjons-/pinneutslag (f.eks. midt i en sving)
// i stedet for kun å ta over den VEDVARENDE/faste delen av utslaget. TAU er tidskonstanten (sekunder)
// for hvor fort det filtrerte signalet følger faktisk utslag - typisk RC-autopilot-oppførsel.
const AUTO_TRIM_FILTER_TAU = 2.5;
// Bred nok overgangssone (11°) til at løftet ikke faller brått/rykkete idet flyet krysser inn i steiling.
const STALL_POST_RANGE_DEG = 11; // bredde på overgangssonen rett etter kritisk vinkel før dyp steiling

const ROLLING_FRICTION = 0.045; // rullemotstand (hjul mot asfalt) ved normal rulling/taxi
const BRAKE_DECELERATION = 2.5; // m/s^2 ekstra oppbremsing fra hjulbrems - kun ved gass i tomgang
const GROUND_YAW_FRICTION = 3;  // eksponentiell demping (1/s) av gir-rotasjon fra hjul mot bakken
// Statisk (Coulomb-lignende) motstand mot vindkantring mens flyet står/ruller på bakken - se
// yawTorqueF0 i stepPhysics. Skalert med vekt (mass*GRAVITY), som normalkraften på dekkene i
// virkeligheten - et tyngre fly har mer dekk-grep og er dermed mer motstandsdyktig mot vindkantring.
const GROUND_YAW_FRICTION_TORQUE_COEFF = 0.15;
// Samme statiske-friksjon-prinsipp som over, men for LATERAL (sideveis) kraft i stedet for gir-moment -
// resolveGroundContact sin egen lateralSpeed-demping er kun RATE-basert (bremser en eksisterende
// sidebevegelse) og motstår aldri en VEDVARENDE sidekraft (f.eks. vind-drag i kryssvind), som ellers gir
// en nullforskjellig likevektsfart der flyet sakte men uendelig driver sidelengs. Kansellerer små/
// moderate sidekrefter helt, reduserer bare (ikke fjerner) sterkere kast.
const GROUND_LATERAL_FRICTION_COEFF = 0.25;
const GROUND_CLEARANCE_FW = 0.05;
const CRASH_SINK_RATE = 6;      // m/s synkefart ved berøring som teller som hard landing
const CRASH_BANK_DEG = 45;      // krengevinkel ved berøring som teller som hard landing

const FIXED_DT = 1 / 120;       // fysikk-tidssteg, samme substep-mønster (akkumulator) som quad-simulatoren
// Samme verdi som Sim.rampStick sin egen interne default (simulator-common.js) - en referanse i stedet
// for en egen hardkodet kopi, så en fremtidig tuning ikke kan endre den ene og glemme den andre. Holdt
// som et eget, navngitt lokalt konstantnavn likevel (i stedet for å bare utelate 4. parameteren til
// rampStick) for å kunne kontrastere eksplisitt mot GAMEPAD_STICK_RAMP_TIME rett under.
const STICK_RAMP_TIME = Sim.STICK_RAMP_TIME;
// Kortere enn tastaturets STICK_RAMP_TIME - en ekte RC-sender-gimbal er raskere/mer presis enn en
// syntetisk tastatur-rampe, men et rått, ufiltrert gamepad-akse-signal (uten NOEN glatting) kan gi et
// momentant, fullt utslag i én eneste fysikk-tick - en umulig-i-virkeligheten dreiemoment-spike.
const GAMEPAD_STICK_RAMP_TIME = 0.06;
const THROTTLE_RATE = 0.7;

const RUNWAY_LENGTH = 360;
const RUNWAY_WIDTH = 14;
const RUNWAY_NEAR_Z = 20;   // verdens-Z for nærmeste terskel (nærmest spawn)
const RUNWAY_SPAWN_Z = 8;   // spawn litt bak terskelen, klar for avgang nedover -Z

/* ---------- Flystørrelser ---------- */
// Kontrolleffektivitet/-demping (ROLL_CONTROL_EFFECTIVENESS/-DAMPING, samt hale-/finne-geometrien som gir
// stigning og gir sin effektivitet/demping) er felles for alle størrelser, akkurat som TORQUE_GAIN er
// felles i quad-simulatoren - det er inertia/masse under som gir et større fly en tregere, mer
// "sluggish" respons ved samme fart, ikke en egen per-klasse ro-styrke.
const PLANE_CLASSES = {
    small: {
        label: "Liten (trener)",
        // Lett og godt motorisert (trekkraft/vekt ≈ 0.83) - skal føles nimbel og lettflydd. wingArea gir
        // strekkforhold ~8.5 (et fly-aktig silhuett, ikke en "låvedør"-vinge), på bekostning av noe høyere
        // steilefart (~9.2 m/s).
        mass: 2.2, wingArea: 0.38, wingSpan: 1.8,
        // propPitchSpeed: farten der en fastpitch-propell teoretisk gir null trekkraft (se thrustForce i
        // stepPhysics) - godt over normal marsjfart så cruise ikke strupes, men lavt nok til å begrense
        // den gamle, drag-begrensede toppfarten (~41 m/s) til noe mer troverdig for en liten trener.
        maxThrust: 18, cd0: 0.045, inducedDragK: 0.9, clSlope: 0.11, stallAngleDeg: 14, propPitchSpeed: 32,
        // Lav inertiaRoll gir kun RASKERE respons, ikke sterkere - selve toppraten styres av det delte
        // ROLL_CONTROL_EFFECTIVENESS/-DAMPING-forholdet over, uavhengig av inertia.
        inertiaRoll: 0.12, inertiaPitch: 0.42, inertiaYaw: 0.42,
        gearOffsetY: -0.22, visualScale: 1.0
    },
    medium: {
        label: "Middels",
        mass: 8, wingArea: 0.65, wingSpan: 2.4,
        // maxThrust økt (fra 22, trekkraft/vekt ≈0.28) - brukeren rapporterte Middels/Stor som "veldig
        // undermotorisert" sammenlignet med Liten sin egen ≈0.83-margin (se kommentaren der). 36N gir
        // ≈0.46 - fortsatt lavere enn Liten (realistisk at en tyngre maskin har noe mindre marginal
        // trekkraft/vekt), men en tydelig mindre "slapp" akselerasjon/stigeevne enn før.
        maxThrust: 36, cd0: 0.04, inducedDragK: 1.0, clSlope: 0.105, stallAngleDeg: 13, propPitchSpeed: 34,
        inertiaRoll: 0.9, inertiaPitch: 1.3, inertiaYaw: 1.5,
        gearOffsetY: -0.28, visualScale: 1.4
    },
    large: {
        label: "Stor",
        mass: 22, wingArea: 1.2, wingSpan: 3.4,
        // Se Middels sin egen maxThrust-kommentar over - samme begrunnelse. 80N gir ≈0.37 trekkraft/vekt
        // (opp fra ≈0.195).
        maxThrust: 80, cd0: 0.035, inducedDragK: 1.1, clSlope: 0.1, stallAngleDeg: 12, propPitchSpeed: 36,
        inertiaRoll: 2.6, inertiaPitch: 3.6, inertiaYaw: 4.0,
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
    hasBeenAirborne: false,
    flightMode: "stabilized",
    planeClass: PLANE_CLASSES[settings.planeClass] ? settings.planeClass : DEFAULT_PLANE_CLASS,
    elevatorTrimDeg: 0,
    autoTrimFilteredDeflection: 0,
    lastRollDeflection: 0, lastPitchDeflection: 0, lastYawDeflection: 0,
    prevBankDeg: 0, prevPitchDegForD: 0,
    filteredBankRateDeg: 0, filteredPitchRateDeg: 0
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
let viewportWatcher; // se Sim.createViewportWatcher - fanger opp DPI-/vindusstørrelse-endringer ved skjermbytte som en enkelt resize-event ikke er pålitelig for
let planeGroup, planePropeller;
let planeAileronLeft, planeAileronRight, planeElevator, planeRudder;
let propSpinSpeed = 0;
let cameraModeIndex = 0;
let windsockHandles = [];
// Trebygging (bjørk/furu) og vind-svai er delt med quad-simulatoren - se Sim.buildBirch/buildPine/
// buildRandomTree/createTreeSwayManager i simulator-common.js (begge simulatorene hadde tidligere hver
// sin nesten identiske kopi).
const treeSwayManager = Sim.createTreeSwayManager();
const buildRandomTree = Sim.buildRandomTree;

// Løv/rusk som driver langs bakken i vindretningen - synlig, retningsvisende vindtegn nær rullebanen (der
// piloten uansett ser under taksing/avgang/landing), i tillegg til vindpølsene. Kun synlig når vind er
// aktivert OG merkbar - resirkuleres ("wrappes") til motsatt kant av regionen når de driver ut av syne,
// slik at det ser ut som en kontinuerlig strøm i stedet for at partiklene tar slutt.
const WIND_LEAF_COUNT = 28;
const WIND_LEAF_REGION = {
    xHalf: RUNWAY_WIDTH / 2 + 20,
    zNear: RUNWAY_NEAR_Z + 20,
    zFar: RUNWAY_NEAR_Z - RUNWAY_LENGTH - 20
};
let windLeaves = [];
function buildWindLeaves() {
    const group = new THREE.Group();
    const leafColors = [0x8a5a2a, 0xa06a2a, 0x6a7a2a, 0xb0742a];
    for (let i = 0; i < WIND_LEAF_COUNT; i++) {
        const mat = new THREE.MeshStandardMaterial({ color: leafColors[i % leafColors.length], side: THREE.DoubleSide });
        const leaf = new THREE.Mesh(new THREE.PlaneGeometry(0.12, 0.08), mat);
        leaf.rotation.x = -Math.PI / 2;
        leaf.position.set(
            (Math.random() * 2 - 1) * WIND_LEAF_REGION.xHalf,
            0.04,
            WIND_LEAF_REGION.zNear + Math.random() * (WIND_LEAF_REGION.zFar - WIND_LEAF_REGION.zNear)
        );
        leaf.visible = false;
        windLeaves.push({ mesh: leaf, spin: (Math.random() * 2 - 1) * 4, bobPhase: Math.random() * Math.PI * 2 });
        group.add(leaf);
    }
    return group;
}
function updateWindLeaves(dt, now) {
    const windSpeed = currentWindVector.length();
    const active = settings.wind.enabled && windSpeed > 0.3;
    windLeaves.forEach(function (leaf) {
        leaf.mesh.visible = active;
        if (!active) return;
        leaf.mesh.position.x += currentWindVector.x * 0.4 * dt;
        leaf.mesh.position.z += currentWindVector.z * 0.4 * dt;
        leaf.mesh.position.y = 0.04 + Math.sin(now / 1000 * 3 + leaf.bobPhase) * 0.02;
        leaf.mesh.rotation.z += leaf.spin * dt;
        if (leaf.mesh.position.x > WIND_LEAF_REGION.xHalf) leaf.mesh.position.x = -WIND_LEAF_REGION.xHalf;
        if (leaf.mesh.position.x < -WIND_LEAF_REGION.xHalf) leaf.mesh.position.x = WIND_LEAF_REGION.xHalf;
        if (leaf.mesh.position.z > WIND_LEAF_REGION.zNear) leaf.mesh.position.z = WIND_LEAF_REGION.zFar;
        if (leaf.mesh.position.z < WIND_LEAF_REGION.zFar) leaf.mesh.position.z = WIND_LEAF_REGION.zNear;
    });
}

// Røyk fra en skorstein (by-huset og fabrikkpipa - se buildTown/buildFactory) - stiger og driver med
// vinden, retningsuavhengig av vindpølsene/løvet (alltid synlig, ikke bare når vind er aktivert - en ekte
// skorstein røyker uansett, selve DRIFTEN er det som viser vindretning/-styrke). Resirkulerer via life
// (0..1) i stedet for å opprette/fjerne objekter - unngår allokering i animasjonsløkken. opts lar
// fabrikkpipa få en tydelig større/tettere/høyere-stigende røyksky enn den vanlige husskorsteinen.
const WIND_SMOKE_COUNT = 10;
const WIND_SMOKE_LIFETIME = 4.5;
let windSmoke = [];
function buildWindSmoke(originLocal, opts) {
    opts = opts || {};
    const count = opts.count || WIND_SMOKE_COUNT;
    const lifetime = opts.lifetime || WIND_SMOKE_LIFETIME;
    const riseHeight = opts.riseHeight || 2.4;
    const maxScale = opts.maxScale || 1.8;
    const baseOpacity = opts.baseOpacity || 0.45;
    const driftSpeed = opts.driftSpeed || 1.5;
    const puffRadius = opts.puffRadius || 0.16;
    const color = opts.color || 0xaaaaaa;
    const fadeStart = opts.fadeStart || SMOKE_FADE_START;
    const group = new THREE.Group();
    for (let i = 0; i < count; i++) {
        const mat = new THREE.MeshStandardMaterial({ color: color, transparent: true, opacity: baseOpacity });
        const puff = new THREE.Mesh(new THREE.SphereGeometry(puffRadius, 8, 6), mat);
        windSmoke.push({
            mesh: puff, life: i / count, origin: originLocal.clone(), phase: Math.random() * Math.PI * 2,
            lifetime: lifetime, riseHeight: riseHeight, maxScale: maxScale, baseOpacity: baseOpacity,
            driftSpeed: driftSpeed, fadeStart: fadeStart
        });
        group.add(puff);
    }
    return group;
}
// Hver puff vandrer litt sidelengs (fase-forskjøvet sinus, økende med alder) i tillegg til å drive med
// vinden - en ren rett linje av voksende kuler så ut som en formfast "kjegle" i stedet for en bølgende
// røyksky. Opasiteten holdes oppe til fadeStart av levetiden og trappes først ned deretter, så røyken
// ikke visuelt dør ut rett over pipa (tidligere falmet den lineært fra dag én, altså mest synlig lengst nede).
const SMOKE_FADE_START = 0.55;
function updateWindSmoke(dt) {
    windSmoke.forEach(function (p) {
        p.life += dt / p.lifetime;
        if (p.life >= 1) p.life -= 1;
        const wander = p.life * 0.6;
        p.mesh.position.set(
            p.origin.x + currentWindVector.x * p.life * p.driftSpeed + Math.sin(p.life * 6 + p.phase) * wander,
            p.origin.y + p.life * p.riseHeight,
            p.origin.z + currentWindVector.z * p.life * p.driftSpeed + Math.cos(p.life * 5 + p.phase) * wander
        );
        p.mesh.scale.setScalar(0.5 + p.life * (p.maxScale - 0.5));
        const fade = p.life < p.fadeStart ? 1 : 1 - (p.life - p.fadeStart) / (1 - p.fadeStart);
        p.mesh.material.opacity = p.baseOpacity * fade;
    });
}

/* ---------- Three.js: scene, rullebane, fly, kameraer ---------- */
// Flate "dekaler" som ligger noen cm over bakkeplanet (rullebane, veier, plasser, plener, dam) flimrer
// (z-fighting) sett fra lufta: på flere hundre meters avstand i slak vinkel er dybdebufferets oppløsning
// langt grovere enn 4-5 cm, og å løfte flatene mer ville sett svevende ut på nært hold. polygonOffset
// biaser dybdeverdien under rastrering i stedet - factor-leddet skaleres med dybde-gradienten, som er
// nøyaktig kompensasjonen slake innsynsvinkler trenger. Brukes på ALLE flate bakke-dekaler.
function groundDecalProps(opts) {
    const result = opts || {};
    result.polygonOffset = true;
    result.polygonOffsetFactor = -2;
    result.polygonOffsetUnits = -2;
    return result;
}

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
    const mat = new THREE.MeshStandardMaterial(groundDecalProps({ map: buildRunwayTexture() }));
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
// lille byen med variasjon uten å trenge vindusåpninger. Vinduer (svakt selvlysende, leser som opplyst
// interiør uansett tid på døgnet/lysvinkel) og dør på fasaden (lokal +Z) gir husene liv på nært hold.
// Saltak (møne-tak) - to skråstilte plater fra møne til takutstikk på hver side, pluss trekantede
// gavlfelt (veggfarge, ikke takfarge) i endene som fyller hullet mellom flat vegg-topp og skrått tak.
// Dette leser som et ekte, gjenkjennelig norsk saltak i stedet for den forrige firkantede "pyramide"-
// takformen (en 4-kant kjegle), som var en vesentlig grunn til at husene virket blokkete/kunstige.
function buildGableRoof(width, depth, roofHeight, overhang, roofColor) {
    const group = new THREE.Group();
    const roofMat = new THREE.MeshStandardMaterial({ color: roofColor });
    const halfSpan = width / 2 + overhang;
    const slopeLen = Math.hypot(halfSpan, roofHeight);
    const angle = Math.atan2(roofHeight, halfSpan);
    const thickness = 0.12;
    const panelDepth = depth + overhang * 1.6;
    [-1, 1].forEach(function (side) {
        const panel = new THREE.Mesh(new THREE.BoxGeometry(slopeLen, thickness, panelDepth), roofMat);
        panel.rotation.z = side > 0 ? -angle : angle;
        panel.position.set(side * halfSpan / 2, roofHeight / 2, 0);
        panel.castShadow = true;
        panel.receiveShadow = true;
        group.add(panel);
    });
    return group;
}
function buildGableEndFill(width, peakHeight, wallColor, zPos, wallHeight) {
    const shape = new THREE.Shape();
    shape.moveTo(-width / 2, 0);
    shape.lineTo(width / 2, 0);
    shape.lineTo(0, peakHeight);
    shape.lineTo(-width / 2, 0);
    const thickness = 0.1;
    const geo = new THREE.ExtrudeGeometry(shape, { depth: thickness, bevelEnabled: false });
    geo.translate(0, 0, -thickness / 2);
    const mesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: wallColor }));
    // Trekantens Y=0 er "under mønet" (toppen av veggen), IKKE bakken - må løftes opp til veggtoppen,
    // ellers havner gavlfeltet nede ved bakken og etterlater et stort hull oppunder taket.
    mesh.position.set(0, wallHeight, zPos);
    mesh.castShadow = true;
    return mesh;
}

function buildSimpleHouse(width, height, depth, wallColor, roofColor, decorativeChimney) {
    if (decorativeChimney === undefined) decorativeChimney = true;
    const group = new THREE.Group();
    const wallMat = new THREE.MeshStandardMaterial({ color: wallColor });
    const walls = new THREE.Mesh(new THREE.BoxGeometry(width, height, depth), wallMat);
    walls.position.y = height / 2;
    walls.castShadow = true;
    walls.receiveShadow = true;
    group.add(walls);

    // Mørkere grunnmur-stripe nederst - bryter opp den ellers ensfargede veggflaten.
    const foundation = new THREE.Mesh(new THREE.BoxGeometry(width + 0.06, height * 0.12, depth + 0.06), new THREE.MeshStandardMaterial({ color: 0x555550 }));
    foundation.position.y = height * 0.06;
    group.add(foundation);

    const winMat = new THREE.MeshStandardMaterial({ color: 0xbfe0e8, emissive: 0x3a5560, emissiveIntensity: 0.4 });
    const doorMat = new THREE.MeshStandardMaterial({ color: 0x4a3322 });
    const winW = width * 0.18, winH = height * 0.3, winY = height * 0.58, winZ = depth / 2 + 0.02;
    [-1, 1].forEach(function (side) {
        const win = new THREE.Mesh(new THREE.BoxGeometry(winW, winH, 0.05), winMat);
        win.position.set(side * width * 0.25, winY, winZ);
        group.add(win);
    });
    const door = new THREE.Mesh(new THREE.BoxGeometry(width * 0.16, height * 0.5, 0.06), doorMat);
    door.position.set(0, height * 0.25, winZ);
    group.add(door);

    const roofHeight = height * 0.5, overhang = 0.35;
    const roof = buildGableRoof(width, depth, roofHeight, overhang, roofColor);
    roof.position.y = height;
    group.add(roof);
    [-1, 1].forEach(function (side) {
        group.add(buildGableEndFill(width, roofHeight, wallColor, side * depth / 2, height));
    });

    // Enkel pipe på taket - et lite, men effektivt detaljbrudd i den ellers rette mønelinjen. Rådhuset
    // styrer sin egen pipe+røyk manuelt (se buildTown) og skrur denne av for å unngå to piper på ett tak.
    if (decorativeChimney) {
        const chimney = new THREE.Mesh(new THREE.BoxGeometry(width * 0.1, roofHeight * 0.7, width * 0.1), new THREE.MeshStandardMaterial({ color: 0x5a4a44 }));
        chimney.position.set(width * 0.2, height + roofHeight * 0.55, depth * 0.15);
        chimney.castShadow = true;
        group.add(chimney);
    }
    return group;
}

// Lavt gjerde rundt husets tomt (stolper + to gjennomgående rekkverk per side) - rent visuelt, ingen
// kollisjon. plotW/plotD er noe større enn selve husets fotavtrykk. Siden mot veien (lokal +Z, samme
// side som døra - se buildSimpleHouse) får en portåpning (gateWidth) midt på, slik at oppkjørselen fra
// veien kan møte tunet uten å gå tvers gjennom gjerdet.
function buildFence(plotW, plotD, gateWidth) {
    const group = new THREE.Group();
    const postMat = new THREE.MeshStandardMaterial({ color: 0x6b4a30 });
    const railMat = new THREE.MeshStandardMaterial({ color: 0x7a5a3a });
    const postH = 0.9, postR = 0.05;
    const railYs = [0.35, 0.75];

    function side(length, isXAxis, offset, gap) {
        const halfGap = gap ? gap / 2 : 0;
        const postCount = Math.max(2, Math.round(length / 2.2) + 1);
        for (let i = 0; i < postCount; i++) {
            const t = (i / (postCount - 1) - 0.5) * length;
            if (gap && Math.abs(t) < halfGap) continue;
            const post = new THREE.Mesh(new THREE.CylinderGeometry(postR, postR, postH, 6), postMat);
            post.position.set(isXAxis ? t : offset, postH / 2, isXAxis ? offset : t);
            group.add(post);
        }
        const spans = gap ? [[-length / 2, -halfGap], [halfGap, length / 2]] : [[-length / 2, length / 2]];
        railYs.forEach(function (railY) {
            spans.forEach(function (span) {
                const segLen = span[1] - span[0];
                if (segLen <= 0.05) return;
                const mid = (span[0] + span[1]) / 2;
                const rail = new THREE.Mesh(
                    isXAxis ? new THREE.BoxGeometry(segLen, 0.06, 0.06) : new THREE.BoxGeometry(0.06, 0.06, segLen),
                    railMat
                );
                rail.position.set(isXAxis ? mid : offset, railY, isXAxis ? offset : mid);
                group.add(rail);
            });
        });
    }
    side(plotW, true, -plotD / 2, 0);
    side(plotW, true, plotD / 2, gateWidth || 0);
    side(plotD, false, -plotW / 2, 0);
    side(plotD, false, plotW / 2, 0);
    return group;
}

// Prosedural veitekstur (asfalt + stiplet midtlinje) - RepeatWrapping langs lengderetningen slik at
// stripemønsteret ser jevnt ut uansett hvor lang den enkelte veistrekningen er.
let roadTextureBase = null;
function buildRoadTexture() {
    if (roadTextureBase) return roadTextureBase;
    const texW = 64, texH = 64;
    const canvas = document.createElement("canvas");
    canvas.width = texW;
    canvas.height = texH;
    const ctx = canvas.getContext("2d");
    ctx.fillStyle = "#48453f";
    ctx.fillRect(0, 0, texW, texH);
    ctx.fillStyle = "#c8c0a8";
    ctx.fillRect(texW / 2 - 2, texH * 0.15, 4, texH * 0.7);
    roadTextureBase = new THREE.CanvasTexture(canvas);
    return roadTextureBase;
}

// Rett veistrekning mellom to punkter - brukt til å binde husene i den lille byen sammen med
// rådhuset/sentrum (se buildTown), i stedet for at de bare står spredt i gresset.
function buildRoadSegment(length, width) {
    const group = new THREE.Group();
    const tex = buildRoadTexture().clone();
    tex.wrapS = THREE.RepeatWrapping;
    tex.wrapT = THREE.RepeatWrapping;
    tex.repeat.set(1, Math.max(1, length / 3));
    const mat = new THREE.MeshStandardMaterial(groundDecalProps({ map: tex }));
    const mesh = new THREE.Mesh(new THREE.PlaneGeometry(width, length), mat);
    mesh.rotation.x = -Math.PI / 2;
    mesh.receiveShadow = true;
    group.add(mesh);
    return group;
}

// Enkel asfaltert/betong flate (plass, forplass, parkeringsplass) - rund, så den ikke krever at man
// vet hvilken vei et (eventuelt rotert) bygg vender for at den skal se riktig ut.
function buildPavedCircle(radius, color) {
    const mesh = new THREE.Mesh(new THREE.CircleGeometry(radius, 20), new THREE.MeshStandardMaterial(groundDecalProps({ color: color })));
    mesh.rotation.x = -Math.PI / 2;
    // 0.05, ikke 0.02 - samme erfaring som dammen (se buildTown): 0.02 er for tynn margin mot bakkeplanet
    // under og flimrer (z-fighting), særlig på en stor flate som denne.
    mesh.position.y = 0.05;
    mesh.receiveShadow = true;
    return mesh;
}

// Enkel parkert bil - grov boks-tilnærming, nok til å lese som "det står biler her" på avstand.
function buildParkedCar(color) {
    const group = new THREE.Group();
    const bodyMat = new THREE.MeshStandardMaterial({ color: color });
    const body = new THREE.Mesh(new THREE.BoxGeometry(1.8, 0.55, 4), bodyMat);
    body.position.y = 0.45;
    body.castShadow = true;
    group.add(body);
    const cabin = new THREE.Mesh(new THREE.BoxGeometry(1.5, 0.4, 2), bodyMat);
    cabin.position.set(0, 0.92, -0.2);
    cabin.castShadow = true;
    group.add(cabin);
    return group;
}

// Fraktcontainer (ISO-proporsjoner, grov tilnærming) - noen stablet ved fabrikken gir et "aktivt
// industriområde"-preg i stedet for en bar bygning midt i gresset.
function buildContainer(color) {
    const box = new THREE.Mesh(new THREE.BoxGeometry(6, 2.6, 2.4), new THREE.MeshStandardMaterial({ color: color }));
    box.position.y = 1.3;
    box.castShadow = true;
    box.receiveShadow = true;
    return box;
}

// Plentekstur (klippestriper, mørkere/mer ensartet grønt enn den ville bakketeksturen utenfor gjerdet -
// se Sim.buildGroundTexture) - brukt på inngjerdede hageflater (se buildTown) slik at man kan SE at det
// er en stelt hage, ikke bare umerket gress.
let lawnTextureBase = null;
function buildLawnTexture() {
    if (lawnTextureBase) return lawnTextureBase;
    const size = 64;
    const canvas = document.createElement("canvas");
    canvas.width = size;
    canvas.height = size;
    const ctx = canvas.getContext("2d");
    const stripes = 8, stripeW = size / stripes;
    for (let i = 0; i < stripes; i++) {
        ctx.fillStyle = (i % 2 === 0) ? "#4a7a3a" : "#427030";
        ctx.fillRect(i * stripeW, 0, stripeW, size);
    }
    lawnTextureBase = new THREE.CanvasTexture(canvas);
    return lawnTextureBase;
}
function buildLawnPatch(width, depth) {
    const tex = buildLawnTexture().clone();
    tex.needsUpdate = true;
    tex.wrapS = THREE.RepeatWrapping;
    tex.wrapT = THREE.RepeatWrapping;
    tex.repeat.set(Math.max(1, Math.round(width / 2)), Math.max(1, Math.round(depth / 2)));
    const mesh = new THREE.Mesh(new THREE.PlaneGeometry(width, depth), new THREE.MeshStandardMaterial(groundDecalProps({ map: tex })));
    mesh.rotation.x = -Math.PI / 2;
    mesh.position.y = 0.05; // se buildPavedCircle - 0.02 flimret (z-fighting) mot bakkeplanet under
    mesh.receiveShadow = true;
    // Pakket i en egen gruppe (uroterte) slik at buildTown trygt kan sette group.rotation.y for å
    // orientere hagen etter huset - å sette .rotation.y direkte på selve meshet (som allerede har
    // .rotation.x satt) hadde kombinert begge aksene i Euler-rekkefølge og vridd flaten skjevt/på kant,
    // i stedet for bare å dreie den flate flaten rundt vertikalaksen (samme mønster som buildRoadSegment).
    const group = new THREE.Group();
    group.add(mesh);
    return group;
}

// Enkel busk (lav, flattrykt ikosaeder) og et lite blomsterbed - hagedetaljer som bryter opp den
// ellers tomme plenflaten mellom hus og gjerde.
function buildBush() {
    const bush = new THREE.Mesh(new THREE.IcosahedronGeometry(0.35, 0), new THREE.MeshStandardMaterial({ color: 0x3a5a2a }));
    bush.position.y = 0.28;
    bush.scale.y = 0.75;
    bush.castShadow = true;
    return bush;
}
function buildFlowerPatch(color) {
    const group = new THREE.Group();
    const flowerMat = new THREE.MeshStandardMaterial({ color: color });
    for (let i = 0; i < 5; i++) {
        const flower = new THREE.Mesh(new THREE.SphereGeometry(0.08, 6, 6), flowerMat);
        flower.position.set((Math.random() - 0.5) * 0.6, 0.16, (Math.random() - 0.5) * 0.6);
        group.add(flower);
    }
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

// Fabrikkens posisjon relativt TOWN_CENTER_X/Z - delt konstant slik at buildWorldObjects (plassering)
// og buildTownRoads (adkomstvei fra ringveien, se under) alltid er i synk.
const FACTORY_DX = 65, FACTORY_DZ = -75;

// Fabrikk med en høy pipe - et tredje, tydelig kraftigere vindtegn (stor, tett røyksky) i tillegg til
// vindpølsene og husskorsteinen, plassert et stykke utenfor selve byen (ikke i boligklyngen). Hovedbygg
// + lavere sidefløy gir et større, mer sammensatt fotavtrykk enn én enkel boks.
function buildFactory() {
    const group = new THREE.Group();
    const baseMat = new THREE.MeshStandardMaterial({ color: 0x6b6b66 });
    const stackMat = new THREE.MeshStandardMaterial({ color: 0x8a3a2e });
    const trimMat = new THREE.MeshStandardMaterial({ color: 0x3a3a38 });
    const winMat = new THREE.MeshStandardMaterial({ color: 0x2a3a42, emissive: 0x18262c, emissiveIntensity: 0.35 });
    const doorMat = new THREE.MeshStandardMaterial({ color: 0x33302a });

    const base = new THREE.Mesh(new THREE.BoxGeometry(11, 7, 9), baseMat);
    base.position.y = 3.5;
    base.castShadow = true;
    base.receiveShadow = true;
    group.add(base);
    const wing = new THREE.Mesh(new THREE.BoxGeometry(6, 4.5, 7), baseMat);
    wing.position.set(-7.5, 2.25, 0.5);
    wing.castShadow = true;
    wing.receiveShadow = true;
    group.add(wing);

    // Vindusrekke på hovedhallens fasade (lokal +Z), personaldør, og en bred lasteport med kai/rampe -
    // gir bygget et gjenkjennelig "industrihall"-uttrykk i stedet for en ren, blank boks.
    for (let i = -3; i <= 3; i++) {
        if (i === 0) continue;
        const win = new THREE.Mesh(new THREE.BoxGeometry(0.9, 1.2, 0.06), winMat);
        win.position.set(i * 1.25, 4.6, 4.52);
        group.add(win);
    }
    const personnelDoor = new THREE.Mesh(new THREE.BoxGeometry(1.1, 2.1, 0.08), doorMat);
    personnelDoor.position.set(0, 1.05, 4.53);
    group.add(personnelDoor);
    const dockDoor = new THREE.Mesh(new THREE.BoxGeometry(3.2, 3.1, 0.1), trimMat);
    dockDoor.position.set(-3.6, 1.85, 4.55);
    group.add(dockDoor);
    const dockPlatform = new THREE.Mesh(new THREE.BoxGeometry(4, 0.5, 1.5), baseMat);
    dockPlatform.position.set(-3.6, 0.25, 5.25);
    dockPlatform.castShadow = true;
    dockPlatform.receiveShadow = true;
    group.add(dockPlatform);

    // Takventiler
    [-2.5, 2.5].forEach(function (x) {
        const vent = new THREE.Mesh(new THREE.BoxGeometry(0.8, 0.5, 0.8), trimMat);
        vent.position.set(x, 7.25, -1.5);
        vent.castShadow = true;
        group.add(vent);
    });

    // Lagringstank ved siden av hovedhallen, godt utenfor bygningskroppen.
    const tankMat = new THREE.MeshStandardMaterial({ color: 0xb0b8bc });
    const tank = new THREE.Mesh(new THREE.CylinderGeometry(1.4, 1.4, 4.5, 16), tankMat);
    tank.position.set(7.2, 2.25, -2);
    tank.castShadow = true;
    group.add(tank);
    const tankCap = new THREE.Mesh(new THREE.SphereGeometry(1.4, 16, 8, 0, Math.PI * 2, 0, Math.PI / 2), tankMat);
    tankCap.position.set(7.2, 4.5, -2);
    group.add(tankCap);

    const stackHeight = 20;
    const stack = new THREE.Mesh(new THREE.CylinderGeometry(1.3, 1.9, stackHeight, 14), stackMat);
    stack.position.set(3, stackHeight / 2, 0);
    stack.castShadow = true;
    group.add(stack);
    const rim = new THREE.Mesh(new THREE.CylinderGeometry(1.45, 1.3, 0.4, 14), trimMat);
    rim.position.set(3, stackHeight + 0.2, 0);
    group.add(rim);
    const smokeOrigin = new THREE.Vector3(3, stackHeight + 0.6, 0);
    group.add(buildWindSmoke(smokeOrigin, {
        count: 34, lifetime: 13, riseHeight: 15, maxScale: 6.5, driftSpeed: 1.5, baseOpacity: 0.6,
        puffRadius: 0.36, color: 0x8f8f8a, fadeStart: 0.72
    }));
    return group;
}

// Liten by et godt stykke øst for hus-/låve-området (utenfor rekkevidde for gjennomflyging) - gir
// simulatoren mye mer å se på i overflyging/navigasjonstrening, ikke bare rullebanen og noen få trær.
//
// Nabolaget er lagt opp som en ringvei med husene langs utsiden - ikke tilfeldig spredte punkt bundet
// sammen med en stjerne av veier til ett sentrum (så helt urealistisk ut, og veiene gikk rett gjennom
// gjerder/husvegger). Husene ligger jevnt fordelt i vinkel rundt sentrum (med en liten radius- og
// vinkel-variasjon per hus for et naturlig, ikke-perfekt-sirkulært preg), med fasaden (dør/vinduer,
// se buildSimpleHouse) vendt inn mot ringveien. Selve veien ligger et godt stykke innenfor husene
// (TOWN_ROAD_SETBACK) - hvert hus har en kort oppkjørsel fra veien inn til en portåpning i eget gjerde,
// slik at veien aldri går gjennom gjerdet eller rett inn i husveggen.
const TOWN_CENTER_X = BUILDING_AREA_X + 70;
const TOWN_CENTER_Z = RUNWAY_NEAR_Z - 150;
const TOWN_HOUSE_DEFS = [
    { w: 6, h: 4, d: 6, wall: 0xd8c9a0, roof: 0x7a3a2a },
    { w: 5, h: 3.6, d: 5, wall: 0xc9d0d8, roof: 0x4a4a52 },
    { w: 7, h: 4.5, d: 6, wall: 0xe0d8c0, roof: 0x6a3a3a },
    { w: 5, h: 3.8, d: 5, wall: 0xd0c8b8, roof: 0x5a4a3a },
    { w: 6, h: 4, d: 5, wall: 0xc8d0c8, roof: 0x4a3a3a },
    { w: 5, h: 3.5, d: 6, wall: 0xd8d0c0, roof: 0x6a4a2a },
    { w: 6, h: 4, d: 5, wall: 0xc0c8d0, roof: 0x3a4a4a },
    { w: 5, h: 3.8, d: 5, wall: 0xd0d8c8, roof: 0x5a3a3a },
    { w: 6, h: 4.2, d: 6, wall: 0xe0d0c8, roof: 0x4a3a2a },
    { w: 6, h: 4, d: 5, wall: 0xd0c8c0, roof: 0x5a4a4a }
];
const TOWN_HOUSE_RADIUS = 34;
const TOWNHALL_CLEARANCE = 8; // rådhusets halve diagonal (~6 m) + margin
const FACTORY_CLEARANCE = 14; // fabrikkens halve diagonal inkl. sidefløy (~11.4 m) + margin
const TOWN_ROAD_SETBACK = 12; // avstand fra ringveien inn til hvert hus (> halve tunbredden - se plotD)
const TOWN_GATE_WIDTH = 3.2; // > oppkjørselens bredde (2.4, se buildTown) så den ikke klipper gjerdestolpene
const TOWN_HOUSES = TOWN_HOUSE_DEFS.map(function (def, i) {
    const angle = (i / TOWN_HOUSE_DEFS.length) * Math.PI * 2 + ((i % 2 === 0) ? 0 : 0.12);
    const radius = TOWN_HOUSE_RADIUS + ((i % 3) - 1) * 4;
    const sin = Math.sin(angle), cos = Math.cos(angle);
    return {
        dx: sin * radius, dz: cos * radius,
        roadDx: sin * (radius - TOWN_ROAD_SETBACK), roadDz: cos * (radius - TOWN_ROAD_SETBACK),
        angle: angle,
        ry: angle + Math.PI, // fasaden vender inn mot ringveien/sentrum
        fenced: i % 2 === 0, // bare annethvert hus gjerdes inn (se buildTown og buildTownRoads)
        w: def.w, h: def.h, d: def.d, wall: def.wall, roof: def.roof
    };
});

// Ringvei gjennom byen (se kommentaren over TOWN_HOUSES) - koblingene følger vinkelrekkefølgen husene
// allerede er generert i, så hver strekning binder sammen to faktiske romlige naboer. Hvert hus får i
// tillegg en kort oppkjørsel rett inn til sin egen portåpning (se buildTown), og rådhuset (som ligger
// for seg selv i sentrum, uten gjerde) får en enkel stikkvei til nærmeste ringvei-punkt.
function buildTownRoads(group) {
    const n = TOWN_HOUSES.length;
    for (let i = 0; i < n; i++) {
        const a = TOWN_HOUSES[i], b = TOWN_HOUSES[(i + 1) % n];
        const dx = b.roadDx - a.roadDx, dz = b.roadDz - a.roadDz;
        const len = Math.hypot(dx, dz);
        const road = buildRoadSegment(len, 3);
        road.position.set(TOWN_CENTER_X + (a.roadDx + b.roadDx) / 2, 0.03, TOWN_CENTER_Z + (a.roadDz + b.roadDz) / 2);
        road.rotation.y = Math.atan2(dx, dz);
        group.add(road);
    }
    // Stikkvei til rådhuset: stopper TOWNHALL_CLEARANCE unna sentrum (rådhusets egen halve diagonal er
    // ca 6 m) i stedet for å gå helt til (0,0) - ellers kjørte veien rett gjennom bygningskroppen. Den
    // åpne flaten mellom der veien slutter og veggen blir en enkel forplass/plass (se buildTown).
    let nearest = TOWN_HOUSES[0], nearestDist = Infinity;
    TOWN_HOUSES.forEach(function (h) {
        const d = Math.hypot(h.roadDx, h.roadDz);
        if (d < nearestDist) { nearestDist = d; nearest = h; }
    });
    const ux = nearest.roadDx / nearestDist, uz = nearest.roadDz / nearestDist;
    const startX = ux * TOWNHALL_CLEARANCE, startZ = uz * TOWNHALL_CLEARANCE;
    const spurLen = Math.max(1, nearestDist - TOWNHALL_CLEARANCE);
    const spur = buildRoadSegment(spurLen, 3);
    spur.position.set(TOWN_CENTER_X + (startX + nearest.roadDx) / 2, 0.03, TOWN_CENTER_Z + (startZ + nearest.roadDz) / 2);
    spur.rotation.y = Math.atan2(nearest.roadDx - startX, nearest.roadDz - startZ);
    group.add(spur);

    // Adkomstvei til fabrikken (se buildWorldObjects) - grener av fra ringveien et sted MELLOM to
    // nabohus (den vinkelrette "gapet" mellom dem, ikke en ren rett linje fra sentrum til fabrikken,
    // som i praksis skar rett gjennom et av hus-tunene på veien ut). Første strekning går radielt ut
    // langs dette gapet til den er utenfor hele boligringen (inkludert største hage), andre strekning
    // går derfra rett til fabrikken - så veien aldri kutter gjennom et tun den passerer.
    const factoryAngle = Math.atan2(FACTORY_DX, FACTORY_DZ);
    let branchA = TOWN_HOUSES[0], branchB = TOWN_HOUSES[1], bestDiff = Infinity;
    for (let i = 0; i < n; i++) {
        const a = TOWN_HOUSES[i], b = TOWN_HOUSES[(i + 1) % n];
        const midAngle = Math.atan2(a.roadDx + b.roadDx, a.roadDz + b.roadDz);
        let diff = Math.abs(midAngle - factoryAngle);
        if (diff > Math.PI) diff = Math.PI * 2 - diff;
        if (diff < bestDiff) { bestDiff = diff; branchA = a; branchB = b; }
    }
    // T-krysset (branchX/branchZ) er midt mellom de to nabohusenes veinoder - det er trygt uansett,
    // siden ringveinodene alltid ligger godt innenfor tunkantene. Selve UTKJØRINGSVINKELEN derimot må
    // vris bort fra et eventuelt inngjerdet nabohus (større tun = mindre klaring) og heller mot det
    // ugjerdede - ellers kunne den rette strekningen ut av ringen skrape borti et gjerde.
    const branchX = (branchA.roadDx + branchB.roadDx) / 2, branchZ = (branchA.roadDz + branchB.roadDz) / 2;
    const biasT = branchA.fenced ? 0.6 : (branchB.fenced ? 0.4 : 0.5);
    const gapX = branchA.roadDx * (1 - biasT) + branchB.roadDx * biasT;
    const gapZ = branchA.roadDz * (1 - biasT) + branchB.roadDz * biasT;
    const gapAngle = Math.atan2(gapX, gapZ);
    const clearRadius = TOWN_HOUSE_RADIUS + 4 + 10; // størst mulig husradius + størst mulig hageutstrekning + margin
    const wpX = Math.sin(gapAngle) * clearRadius, wpZ = Math.cos(gapAngle) * clearRadius;

    const seg1dx = wpX - branchX, seg1dz = wpZ - branchZ;
    const seg1 = buildRoadSegment(Math.hypot(seg1dx, seg1dz), 3.4);
    seg1.position.set(TOWN_CENTER_X + (branchX + wpX) / 2, 0.03, TOWN_CENTER_Z + (branchZ + wpZ) / 2);
    seg1.rotation.y = Math.atan2(seg1dx, seg1dz);
    group.add(seg1);

    // Stopper FACTORY_CLEARANCE unna fabrikkens senter - ellers kjørte veien rett inn i bygningskroppen.
    // Åpen flate mellom veistopp og bygg blir en enkel oppstillings-/parkeringsplass (se buildWorldObjects).
    const seg2dx = FACTORY_DX - wpX, seg2dz = FACTORY_DZ - wpZ;
    const seg2FullLen = Math.hypot(seg2dx, seg2dz);
    const seg2Len = Math.max(1, seg2FullLen - FACTORY_CLEARANCE);
    const ux2 = seg2dx / seg2FullLen, uz2 = seg2dz / seg2FullLen;
    const seg2 = buildRoadSegment(seg2Len, 3.4);
    seg2.position.set(TOWN_CENTER_X + wpX + ux2 * seg2Len / 2, 0.03, TOWN_CENTER_Z + wpZ + uz2 * seg2Len / 2);
    seg2.rotation.y = Math.atan2(seg2dx, seg2dz);
    group.add(seg2);
}

// Murstein-tekstur - samme prosedurale prinsipp som bakke-/rullebane-teksturene (Sim.buildGroundTexture,
// buildRunwayTexture): ett lite, tilbart mønster med RepeatWrapping, ikke én stor fastmalt flate.
let brickTextureBase = null;
function buildBrickTexture() {
    if (brickTextureBase) return brickTextureBase;
    const texW = 64, texH = 64;
    const canvas = document.createElement("canvas");
    canvas.width = texW;
    canvas.height = texH;
    const ctx = canvas.getContext("2d");
    ctx.fillStyle = "#8a4a3a";
    ctx.fillRect(0, 0, texW, texH);
    ctx.strokeStyle = "#5a2e22";
    ctx.lineWidth = 2;
    const rows = 8, brickH = texH / rows, cols = 4;
    for (let r = 0; r <= rows; r++) {
        const y = r * brickH;
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(texW, y);
        ctx.stroke();
    }
    for (let r = 0; r < rows; r++) {
        const offset = (r % 2) * (texW / cols / 2);
        for (let c = 0; c <= cols; c++) {
            const x = ((c * (texW / cols) + offset) % texW + texW) % texW;
            ctx.beginPath();
            ctx.moveTo(x, r * brickH);
            ctx.lineTo(x, (r + 1) * brickH);
            ctx.stroke();
        }
    }
    brickTextureBase = new THREE.CanvasTexture(canvas);
    return brickTextureBase;
}
function buildBrickMaterial(repeatX, repeatY) {
    const tex = buildBrickTexture().clone();
    tex.needsUpdate = true;
    tex.wrapS = THREE.RepeatWrapping;
    tex.wrapT = THREE.RepeatWrapping;
    tex.repeat.set(repeatX, repeatY);
    return new THREE.MeshStandardMaterial({ map: tex });
}

// Urskive-tekstur UTEN visere (bare bunnplate + rand + time-streker) - viserne er egne mesh-"armer" på
// separate dreiepunkt-grupper (se buildClockFace) som roteres fra PC-ens klokke i updateClockTowers,
// i stedet for å tegnes fast inn i selve teksturen.
let clockTextureBase = null;
function buildClockTexture() {
    if (clockTextureBase) return clockTextureBase;
    const size = 128;
    const canvas = document.createElement("canvas");
    canvas.width = size;
    canvas.height = size;
    const ctx = canvas.getContext("2d");
    ctx.fillStyle = "#3a3a38";
    ctx.fillRect(0, 0, size, size);
    ctx.fillStyle = "#e8e2c8";
    ctx.beginPath();
    ctx.arc(size / 2, size / 2, size * 0.42, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "#2a2a28";
    ctx.lineWidth = 3;
    ctx.stroke();
    ctx.lineWidth = 2;
    for (let i = 0; i < 12; i++) {
        const a = (i / 12) * Math.PI * 2;
        const r1 = size * 0.42, r2 = size * (i % 3 === 0 ? 0.34 : 0.38);
        ctx.beginPath();
        ctx.moveTo(size / 2 + Math.sin(a) * r1, size / 2 - Math.cos(a) * r1);
        ctx.lineTo(size / 2 + Math.sin(a) * r2, size / 2 - Math.cos(a) * r2);
        ctx.stroke();
    }
    clockTextureBase = new THREE.CanvasTexture(canvas);
    return clockTextureBase;
}

// Klokkehendene registreres her (dreiepunkt-grupper, ikke selve viser-meshene) slik at
// updateClockTowers kan rotere dem fra PC-ens klokke hvert bilde - samme "handles"-mønster som
// windsockHandles/treeHandles.
let clockHandles = [];
function buildClockFace(radius) {
    const group = new THREE.Group();
    const face = new THREE.Mesh(new THREE.CircleGeometry(radius, 24), new THREE.MeshStandardMaterial({ map: buildClockTexture() }));
    group.add(face);

    const handMat = new THREE.MeshStandardMaterial({ color: 0x2a2a28 });
    const hourPivot = new THREE.Group();
    const hourHand = new THREE.Mesh(new THREE.BoxGeometry(radius * 0.1, radius * 0.5, 0.02), handMat);
    hourHand.position.set(0, radius * 0.25, 0.015);
    hourPivot.add(hourHand);
    group.add(hourPivot);

    const minutePivot = new THREE.Group();
    const minuteHand = new THREE.Mesh(new THREE.BoxGeometry(radius * 0.07, radius * 0.78, 0.02), handMat);
    minuteHand.position.set(0, radius * 0.39, 0.02);
    minutePivot.add(minuteHand);
    group.add(minutePivot);

    clockHandles.push({ hour: hourPivot, minute: minutePivot });
    return group;
}
// Setter viserne til faktisk PC-klokkeslett - kalt en gang ved bygging og hvert bilde fra animate()
// (se updateClockTowers), akkurat som treeSwayManager.update/updateWindsockVisual leser andre live-verdier.
// Kalt hvert bilde (60 Hz), men viserbevegelsen er umerkelig raskere enn ca. ett sekund av gangen
// (minuttviseren flytter seg ~0,1°/sek) - kastet ny Date() og satt rotasjonene på nytt uansett var ren
// bortkastet arbeid resten av tiden. Strupet til maks én reell oppdatering i sekundet.
let lastClockTowerUpdateMs = 0;
function updateClockTowers() {
    const nowMs = Date.now();
    if (nowMs - lastClockTowerUpdateMs < 1000) return;
    lastClockTowerUpdateMs = nowMs;
    const now = new Date(nowMs);
    const hourFrac = (now.getHours() % 12) / 12 + now.getMinutes() / 720;
    const minuteFrac = now.getMinutes() / 60 + now.getSeconds() / 3600;
    const hourAngle = hourFrac * Math.PI * 2;
    const minuteAngle = minuteFrac * Math.PI * 2;
    clockHandles.forEach(function (c) {
        c.hour.rotation.z = -hourAngle;
        c.minute.rotation.z = -minuteAngle;
    });
}

// Klokketårn - en forenklet nikk til Oslo rådhus' karakteristiske tårn (ikke en kopi, bare samme idé:
// en smal, høy tårnkropp med urskiver og en spiss topp, som skiller rådhuset visuelt fra alle
// bolighusene rundt det). Urskive på alle fire sider, ikke bare front/side.
function buildClockTower(width, towerHeight) {
    const group = new THREE.Group();
    const tower = new THREE.Mesh(new THREE.BoxGeometry(width, towerHeight, width), buildBrickMaterial(1, towerHeight / width));
    tower.position.y = towerHeight / 2;
    tower.castShadow = true;
    tower.receiveShadow = true;
    group.add(tower);

    const clockY = towerHeight * 0.82, clockR = width * 0.32, faceOffset = width / 2 + 0.02;
    [
        { pos: [0, clockY, faceOffset], ry: 0 },
        { pos: [faceOffset, clockY, 0], ry: Math.PI / 2 },
        { pos: [0, clockY, -faceOffset], ry: Math.PI },
        { pos: [-faceOffset, clockY, 0], ry: -Math.PI / 2 }
    ].forEach(function (side) {
        const clock = buildClockFace(clockR);
        clock.position.set(side.pos[0], side.pos[1], side.pos[2]);
        clock.rotation.y = side.ry;
        group.add(clock);
    });

    const cap = new THREE.Mesh(new THREE.ConeGeometry(width * 0.75, towerHeight * 0.35, 4), new THREE.MeshStandardMaterial({ color: 0x2a2a28 }));
    cap.rotation.y = Math.PI / 4;
    cap.position.y = towerHeight + towerHeight * 0.35 * 0.5;
    cap.castShadow = true;
    group.add(cap);
    return group;
}

// Rådhuset - et tydelig OFFENTLIG bygg (murstein, flatt tak, store vinduer, søyleinngang, klokketårn
// og flaggstang), ikke bare et stort bolighus i samme stil som resten av byen.
function buildTownHall(width, height, depth) {
    const group = new THREE.Group();
    const walls = new THREE.Mesh(new THREE.BoxGeometry(width, height, depth), buildBrickMaterial(width / 2.2, height / 2.2));
    walls.position.y = height / 2;
    walls.castShadow = true;
    walls.receiveShadow = true;
    group.add(walls);

    const roofMat = new THREE.MeshStandardMaterial({ color: 0x3a3a38 });
    const roof = new THREE.Mesh(new THREE.BoxGeometry(width + 0.3, 0.4, depth + 0.3), roofMat);
    roof.position.y = height + 0.2;
    roof.castShadow = true;
    group.add(roof);

    const winMat = new THREE.MeshStandardMaterial({ color: 0xbfe0e8, emissive: 0x3a5560, emissiveIntensity: 0.35 });
    const winCount = 4;
    for (let i = 0; i < winCount; i++) {
        const t = (i / (winCount - 1) - 0.5) * (width * 0.72);
        const win = new THREE.Mesh(new THREE.BoxGeometry(width * 0.11, height * 0.42, 0.06), winMat);
        win.position.set(t, height * 0.56, depth / 2 + 0.03);
        group.add(win);
    }

    const doorMat = new THREE.MeshStandardMaterial({ color: 0x3a2a1a });
    const door = new THREE.Mesh(new THREE.BoxGeometry(width * 0.2, height * 0.55, 0.08), doorMat);
    door.position.set(0, height * 0.28, depth / 2 + 0.04);
    group.add(door);

    const columnMat = new THREE.MeshStandardMaterial({ color: 0xf0ece0 });
    const porticoWidth = width * 0.42, porticoHeight = height * 0.85, porticoDepth = 1.6;
    [-1, 1].forEach(function (side) {
        const col = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.24, porticoHeight, 10), columnMat);
        col.position.set(side * porticoWidth / 2, porticoHeight / 2, depth / 2 + porticoDepth * 0.85);
        col.castShadow = true;
        group.add(col);
    });
    const porticoRoof = new THREE.Mesh(new THREE.BoxGeometry(porticoWidth + 0.8, 0.3, porticoDepth + 0.6), roofMat);
    porticoRoof.position.set(0, porticoHeight + 0.15, depth / 2 + porticoDepth * 0.55);
    porticoRoof.castShadow = true;
    group.add(porticoRoof);

    const tower = buildClockTower(width * 0.32, height * 1.6);
    tower.position.set(0, height + 0.4, -depth * 0.15);
    group.add(tower);

    const poleMat = new THREE.MeshStandardMaterial({ color: 0xcccccc });
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, height * 1.1, 8), poleMat);
    pole.position.set(width * 0.4, height * 0.55, depth / 2 + 0.5);
    group.add(pole);
    const flag = new THREE.Mesh(new THREE.PlaneGeometry(0.6, 0.4), new THREE.MeshStandardMaterial({ color: 0xba1522, side: THREE.DoubleSide }));
    flag.position.set(width * 0.4 + 0.3, height * 1.0, depth / 2 + 0.5);
    group.add(flag);

    return group;
}

function buildTown() {
    const group = new THREE.Group();
    buildTownRoads(group);
    TOWN_HOUSES.forEach(function (h, i) {
        const house = buildSimpleHouse(h.w, h.h, h.d, h.wall, h.roof);
        house.position.set(TOWN_CENTER_X + h.dx, 0, TOWN_CENTER_Z + h.dz);
        house.rotation.y = h.ry;
        group.add(house);

        // Bare annethvert hus gjerdes inn (ikke gjerder overalt) - med en beskjeden hageflate som
        // holder god avstand til nabotomtenes gjerder (husene ligger ~36-40° fra hverandre i ringen).
        // Gjerdet har en portåpning mot veien (samme side som inngangsdøra) der oppkjørselen munner ut.
        const plotW = h.w + 6, plotD = h.d + 6;
        if (h.fenced) {
            const fence = buildFence(plotW, plotD, TOWN_GATE_WIDTH);
            fence.position.set(TOWN_CENTER_X + h.dx, 0, TOWN_CENTER_Z + h.dz);
            fence.rotation.y = h.ry;
            group.add(fence);

            // Hageflate (egen plentekstur, se buildLawnPatch) pluss et par busker og et blomsterbed -
            // gir de inngjerdede tunene et faktisk "hage"-preg i stedet for bare vanlig bakketekstur.
            const lawn = buildLawnPatch(plotW, plotD);
            lawn.position.set(TOWN_CENTER_X + h.dx, 0, TOWN_CENTER_Z + h.dz);
            lawn.rotation.y = h.ry;
            group.add(lawn);

            const yardGroup = new THREE.Group();
            yardGroup.position.set(TOWN_CENTER_X + h.dx, 0, TOWN_CENTER_Z + h.dz);
            yardGroup.rotation.y = h.ry;
            [[-h.w / 2 - 0.6, -h.d / 2 - 0.6], [h.w / 2 + 0.6, -h.d / 2 - 0.6]].forEach(function (p) {
                const bush = buildBush();
                bush.position.set(p[0], 0, p[1]);
                yardGroup.add(bush);
            });
            const flowers = buildFlowerPatch(0xd8b23a);
            flowers.position.set(h.w * 0.28, 0, h.d / 2 + 0.6);
            yardGroup.add(flowers);
            group.add(yardGroup);
        }

        // Oppkjørsel: rett strekning fra ringveien til tunkanten - begge ligger på samme radielle
        // linje fra sentrum som selve huset, siden fasaden vender rett inn mot sentrum.
        const gateRadius = Math.hypot(h.dx, h.dz) - plotD / 2;
        const sin = Math.sin(h.angle), cos = Math.cos(h.angle);
        const gateX = sin * gateRadius, gateZ = cos * gateRadius;
        const dx = gateX - h.roadDx, dz = gateZ - h.roadDz;
        const driveway = buildRoadSegment(Math.hypot(dx, dz), 2.4);
        driveway.position.set(TOWN_CENTER_X + (h.roadDx + gateX) / 2, 0.03, TOWN_CENTER_Z + (h.roadDz + gateZ) / 2);
        driveway.rotation.y = Math.atan2(dx, dz);
        group.add(driveway);
    });

    // Litt større "rådhus"-aktig bygg midt i byen som et landemerke å navigere etter, med en enkel
    // brolagt plass foran inngangen (mellom veggen og der stikkveien inn til sentrum stopper - se
    // buildTownRoads/TOWNHALL_CLEARANCE) og et par benker - en liten "møteplass"/park-følelse.
    const townHall = buildTownHall(9, 5.5, 8);
    townHall.position.set(TOWN_CENTER_X, 0, TOWN_CENTER_Z);
    group.add(townHall);

    const plaza = buildPavedCircle(TOWNHALL_CLEARANCE, 0x9a9484);
    plaza.position.x = TOWN_CENTER_X;
    plaza.position.z = TOWN_CENTER_Z;
    group.add(plaza);
    const benchMat = new THREE.MeshStandardMaterial({ color: 0x5a4530 });
    [-2.4, 2.4].forEach(function (bx) {
        const bench = new THREE.Mesh(new THREE.BoxGeometry(1.4, 0.4, 0.5), benchMat);
        bench.position.set(TOWN_CENTER_X + bx, 0.2, TOWN_CENTER_Z + TOWNHALL_CLEARANCE - 1.2);
        bench.castShadow = true;
        group.add(bench);
    });

    // Skorstein + røyk - et retningsuavhengig vindtegn (røyken driver med vinden) synlig fra lang avstand,
    // i tillegg til vindpølsene ved rullebanen og de vaiende trærne (se treeSwayManager.update).
    const chimneyMat = new THREE.MeshStandardMaterial({ color: 0x555550 });
    const chimney = new THREE.Mesh(new THREE.BoxGeometry(0.5, 1.3, 0.5), chimneyMat);
    const chimneyLocalPos = new THREE.Vector3(2.8, 5.5 + 0.65, 1.6);
    chimney.position.copy(chimneyLocalPos);
    chimney.castShadow = true;
    townHall.add(chimney);
    townHall.add(buildWindSmoke(chimneyLocalPos.clone().add(new THREE.Vector3(0, 0.8, 0))));

    // Plassert godt utenfor selv den største hagen (maks husradius + maks hageutstrekning ~44) - sto
    // tidligere nesten oppi et hus-tun.
    const tower = buildWaterTower();
    tower.position.set(TOWN_CENTER_X - 49, 0, TOWN_CENTER_Z + 7);
    group.add(tower);

    // Liten dam ved kanten av byen - rent visuelt landemerke.
    // Løftet fra 0.02 til 0.05 over bakken - samme prinsipp som rullebanen/rutenettet (se buildGround) -
    // 0.02 var for tynn en margin og flimret (z-fighting) mot bakkeplanet under.
    const pond = new THREE.Mesh(new THREE.CircleGeometry(8, 24), new THREE.MeshStandardMaterial(groundDecalProps({ color: 0x2a5a78 })));
    pond.rotation.x = -Math.PI / 2;
    pond.position.set(TOWN_CENTER_X + 40, 0.05, TOWN_CENTER_Z - 10);
    group.add(pond);

    // Trekrans rundt byen.
    [
        { dx: -45, dz: -55, h: 7 }, { dx: 45, dz: -55, h: 7.5 }, { dx: -50, dz: 20, h: 6.5 },
        { dx: 50, dz: 25, h: 7 }, { dx: -20, dz: 50, h: 6.8 }, { dx: 25, dz: 52, h: 7.2 },
        { dx: 0, dz: -60, h: 8 }, { dx: 55, dz: -10, h: 6.5 }
    ].forEach(function (t) {
        const tree = buildRandomTree(t.h);
        tree.position.set(TOWN_CENTER_X + t.dx, 0, TOWN_CENTER_Z + t.dz);
        group.add(treeSwayManager.addSwayingTree(tree));
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
    group.add(buildWindLeaves());

    const factory = buildFactory();
    factory.position.set(TOWN_CENTER_X + FACTORY_DX, 0, TOWN_CENTER_Z + FACTORY_DZ);
    factory.rotation.y = THREE.MathUtils.degToRad(20);
    group.add(factory);

    // Asfaltert industritomt rundt fabrikken (ikke gress helt inntil bygget) - stor nok til å romme
    // hele bygningskroppen uansett rotasjon (FACTORY_CLEARANCE), pluss noen parkerte biler og
    // fraktcontainere for et "aktivt anlegg"-preg. Sirkulær flate - trenger ikke vite hvilken vei det
    // (roterte) bygget vender for at kantene skal se riktige ut.
    const factoryYard = buildPavedCircle(FACTORY_CLEARANCE + 4, 0x3d3a36);
    factoryYard.position.x = TOWN_CENTER_X + FACTORY_DX;
    factoryYard.position.z = TOWN_CENTER_Z + FACTORY_DZ;
    group.add(factoryYard);
    [
        { dx: 10, dz: 8, ry: 0.3, color: 0x445566 },
        { dx: 11.5, dz: -6, ry: -0.6, color: 0x883333 },
        { dx: -9, dz: 10.5, ry: 1.1, color: 0x336644 }
    ].forEach(function (c) {
        const car = buildParkedCar(c.color);
        car.position.set(TOWN_CENTER_X + FACTORY_DX + c.dx, 0, TOWN_CENTER_Z + FACTORY_DZ + c.dz);
        car.rotation.y = c.ry;
        group.add(car);
    });
    [
        { dx: -12.5, dz: -8, color: 0xb03a2a }, { dx: -12.5, dz: -4.6, color: 0x2a5a8a },
        { dx: -6, dz: -12, color: 0x8a7a2a }
    ].forEach(function (c) {
        const container = buildContainer(c.color);
        container.position.set(TOWN_CENTER_X + FACTORY_DX + c.dx, 0, TOWN_CENTER_Z + FACTORY_DZ + c.dz);
        group.add(container);
    });

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
        const tree = buildRandomTree(t.h);
        tree.position.set(t.x, 0, t.z);
        group.add(treeSwayManager.addSwayingTree(tree));
    });

    return group;
}

// Genererer en realistisk vingeprofil-kontur (avrundet forkant, buet overside, flat underside, tilspisset
// bakkant) i stedet for et rent rektangel. Bruker en NACA-00xx-lignende symmetrisk tykkelsesfordeling
// (5*t*(0.2969*sqrt(x) - 0.126*x - 0.3516*x² + 0.2843*x³ - 0.1015*x⁴)) som gir BÅDE den avrundede nesen
// (sqrt(x)-leddet har uendelig stigning ved x=0) OG den tilspissede bakkanten (går mot 0 ved x=1) av seg
// selv. Undersiden bruker samme kurve nær nesen (glatt overgang) men blandes raskt til flat innen ~15%
// korde - en klassisk "flat-bunn"-profil (Clark-Y-lignende).
// xStart/xEnd (0..1, andel av FULL korde chordLen) lar oss bygge "fremre hoveddel" og "bakre del/
// balanseror" som to separate, men konturmessig sammenhengende biter (balanserorets utsparing på
// vingetuppen). flatBottom (default true) styrer kambret (vinge) vs. symmetrisk (hale/finne, som må
// virke like godt begge veier) profil.
function buildAirfoilProfileShape(chordLen, xStart, xEnd, thicknessRatio, flatBottom) {
    const SAMPLES = 16;
    function halfThickness(x) {
        return thicknessRatio * chordLen * 5 * (0.2969 * Math.sqrt(x) - 0.126 * x - 0.3516 * x * x + 0.2843 * x * x * x - 0.1015 * x * x * x * x);
    }
    function upperY(x) { return halfThickness(x) * (flatBottom ? 1.15 : 1); }
    function lowerY(x) {
        if (!flatBottom) return -halfThickness(x);
        const noseBlend = Math.max(0, 1 - x / 0.15);
        const eased = noseBlend * noseBlend * (3 - 2 * noseBlend);
        return -halfThickness(x) * eased;
    }
    function toLocalX(x) { return x * chordLen - chordLen / 2; }

    const upperPts = [], lowerPts = [];
    for (let i = 0; i <= SAMPLES; i++) {
        // Flere samplepunkter nær xStart enn xEnd (t^1.5) - fanger opp nesens raske krumning bedre når
        // xStart=0 (fremre del); harmløst (bare litt tettere punkter nær kutt-linjen) for bakre del.
        const t = Math.pow(i / SAMPLES, 1.5);
        const x = xStart + (xEnd - xStart) * t;
        upperPts.push(new THREE.Vector2(toLocalX(x), upperY(x)));
        lowerPts.push(new THREE.Vector2(toLocalX(x), lowerY(x)));
    }
    const shape = new THREE.Shape();
    shape.moveTo(upperPts[0].x, upperPts[0].y);
    for (let i = 1; i < upperPts.length; i++) shape.lineTo(upperPts[i].x, upperPts[i].y);
    for (let i = lowerPts.length - 1; i >= 0; i--) shape.lineTo(lowerPts[i].x, lowerPts[i].y);
    shape.lineTo(upperPts[0].x, upperPts[0].y);
    return shape;
}

// Ekstruderer profilkonturen langs spennet. ExtrudeGeometry ekstruderer i sitt eget lokale Z - vi
// sentrerer denne dybden FØR rotasjon (så spennet blir symmetrisk om 0), roterer så -90° om Y for å
// bytte om lokal Z (spenn) og lokal X (korde) til hhv. verdens X (spenn) og Z (korde, med riktig fortegn
// - forkant havner på NEGATIV z, i tråd med "forover = -Z"-konvensjonen resten av fysikken bruker).
// chordZOffset (valgfri) forskyver konturen langs korde-aksen ETTERPÅ - brukes kun av balanserorets/
// høyderorets/sideorets hengslede del, som trenger hengslelinjen (ikke midtkorden) som sitt eget lokale
// origo. verticalFin (valgfri): en ekstra 90°-rotasjon om Z-aksen ETTER spenn-rotasjonen, som bytter om
// spennaksen (normalt X, vannrett - riktig for vinge/høyderor) til Y (loddrett - riktig for en stående
// finne/sideror, der "spennet" er finnens HØYDE og tykkelsen ligger langs X i stedet for Y).
function buildWingProfileMesh(chordLen, xStart, xEnd, thicknessRatio, spanLen, mat, chordZOffset, flatBottom, verticalFin) {
    const shape = buildAirfoilProfileShape(chordLen, xStart, xEnd, thicknessRatio, flatBottom !== false);
    const geometry = new THREE.ExtrudeGeometry(shape, { depth: spanLen, bevelEnabled: false, curveSegments: 1 });
    geometry.translate(0, 0, -spanLen / 2);
    geometry.rotateY(-Math.PI / 2);
    if (verticalFin) geometry.rotateZ(Math.PI / 2);
    if (chordZOffset) geometry.translate(0, 0, chordZOffset);
    const mesh = new THREE.Mesh(geometry, mat);
    mesh.castShadow = true;
    return mesh;
}

const WING_THICKNESS_RATIO = 0.1; // maks tykkelse som andel av korde (typisk trener-vingeprofil)
const WING_MAIN_FRAC = 0.65;      // andel av korden som er den faste hoveddelen (resten er balanseror)

// Vingen bygges i to deler langs korden (fremre hoveddel over HELE spennvidden, med den ekte avrundede
// nesen - bakre del kun i midt-seksjonen, med den ekte tilspissede bakkanten) i stedet for én
// sammenhengende boks - det gir et "hakk" i bakkanten ute på tuppene der balanseroret felles inn på
// riktig hengslelinje, i stedet for å henge utenpå bakkanten som en løs klaff (som så rart ut - roret
// sto synlig utenfor selve vingeprofilen).
function buildWing(spec, wingMat, darkMat) {
    const group = new THREE.Group();
    const wingChord = spec.wingArea / spec.wingSpan;

    const rearChord = wingChord * (1 - WING_MAIN_FRAC);
    const aileronSpan = spec.wingSpan * 0.22;
    const centerSpan = spec.wingSpan - aileronSpan * 2;
    const hingeChordZ = WING_MAIN_FRAC * wingChord - wingChord / 2;

    // Fremre hoveddel - går over hele spennvidden, uavbrutt (bærer hele vingens strukturelle silhuett
    // OG den avrundede forkanten).
    const wingFront = buildWingProfileMesh(wingChord, 0, WING_MAIN_FRAC, WING_THICKNESS_RATIO, spec.wingSpan, wingMat);
    group.add(wingFront);

    // Bakre midtdel - stopper FØR vingetuppene (den ekte tilspissede bakkanten), som gir nettopp den
    // utsparingen balanserorene skal fylle.
    const wingRear = buildWingProfileMesh(wingChord, WING_MAIN_FRAC, 1, WING_THICKNESS_RATIO, centerSpan, wingMat);
    group.add(wingRear);

    [-1, 1].forEach(function (side) {
        // Aileron (skråror) - hengslet i utsparingen på vingetuppen, langs samme bakkant-profil/
        // tilspissing som wingRear (chordZOffset flytter hengslelinjen til pivotens eget lokale origo).
        // Roteres om lokal X-akse i updatePlaneVisual() ut fra faktisk avbøyning.
        const aileronPivot = new THREE.Group();
        aileronPivot.position.set(side * (spec.wingSpan / 2 - aileronSpan / 2), 0, hingeChordZ);
        const aileronMesh = buildWingProfileMesh(wingChord, WING_MAIN_FRAC, 1, WING_THICKNESS_RATIO, aileronSpan * 0.95, darkMat, -hingeChordZ);
        aileronPivot.add(aileronMesh);
        group.add(aileronPivot);
        group.userData["aileron" + side] = aileronPivot;

        // Navigasjonslys i vingetuppen - rødt til venstre, grønt til høyre (ekte flykonvensjon).
        const navLight = new THREE.Mesh(new THREE.SphereGeometry(wingChord * WING_THICKNESS_RATIO * 0.7, 8, 6),
            new THREE.MeshStandardMaterial({
                color: side < 0 ? 0xff2a2a : 0x2aff5a,
                emissive: side < 0 ? 0xff2a2a : 0x2aff5a,
                emissiveIntensity: 0.6
            }));
        navLight.position.set(side * (spec.wingSpan / 2 - 0.02), 0, -wingChord / 2);
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
    // fuselageLength/cabinRadius/CABIN_LEN_RATIO/TAIL_LEN_RATIO er DELT med checkTailStrike (se konstantene
    // øverst i filen) - IKKE gjør disse til lokale, uavhengige tall, det holder tailstrike-varselet i synk.
    const fuselageLength = FUSELAGE_LENGTH_BUILD, cabinRadius = CABIN_RADIUS_BUILD;
    const noseLen = fuselageLength * 0.18, cabinLen = fuselageLength * CABIN_LEN_RATIO, tailLen = fuselageLength * TAIL_LEN_RATIO;
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

    const tailSection = new THREE.Mesh(new THREE.CylinderGeometry(cabinRadius * TAIL_TIP_RADIUS_RATIO, cabinRadius, tailLen, 14), bodyMat);
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
    // spec.wingSpan/wingArea er EKTE verdensrom-mål (brukt direkte i fysikken, som ikke bruker visualScale
    // i det hele tatt) - men HELE flygruppen skaleres uniformt med spec.visualScale til slutt
    // (group.scale.setScalar under), så all geometri bygget FØR den skaleringen må være i "bygge-rom",
    // akkurat som fuselageLength/cabinRadius allerede er. Derfor deles wingSpan/wingArea på visualScale
    // (areal på visualScale², siden areal skalerer med lengde i annen potens) FØR de brukes til å bygge
    // geometri her - samme prinsipp som gearHeight bruker for understellets høyde (se lenger ned).
    const buildWingSpan = spec.wingSpan / spec.visualScale;
    const buildWingArea = spec.wingArea / (spec.visualScale * spec.visualScale);
    const wingChord = buildWingArea / buildWingSpan;
    const wingMountY = cabinRadius * WING_MOUNT_HEIGHT_RATIO;
    const wing = buildWing({ wingArea: buildWingArea, wingSpan: buildWingSpan }, wingMat, darkMat);
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
    const tailSpan = buildWingSpan * 0.28, tailChord = wingChord * 0.7;
    const tailFrontZ = fuselageLength / 2 - tailChord;

    // Hale-/finneflater bruker samme profil-generator som vingen (buildAirfoilProfileShape/
    // buildWingProfileMesh), men med flatBottom=false - ekte hale-/finneflater er nesten alltid
    // SYMMETRISKE profiler (må virke like godt begge veier: høyderor opp/ned, sideror høyre/venstre).
    // "chordLen" her er den fast+hengslede delens SAMLEDE lengde (stab+elevator, hhv. fin+rudder) -
    // xStart/xEnd deler denne i to konturmessig sammenhengende biter akkurat som vingens
    // hoveddel/balanseror, med hengslelinjen (chordZOffset) beregnet av samme grunn.
    const stabChord = tailChord * 0.6, elevatorChord = tailChord * 0.45;
    const tailCombinedChord = stabChord + elevatorChord;
    const tailMainFrac = stabChord / tailCombinedChord;
    const tailHingeChordZ = tailMainFrac * tailCombinedChord - tailCombinedChord / 2;
    const TAIL_SURFACE_THICKNESS_RATIO = 0.09;

    const hStab = buildWingProfileMesh(tailCombinedChord, 0, tailMainFrac, TAIL_SURFACE_THICKNESS_RATIO, tailSpan, wingMat, 0, false);
    hStab.position.set(0, cabinRadius * 0.3, tailFrontZ + tailCombinedChord / 2);
    group.add(hStab);

    const elevatorPivot = new THREE.Group();
    elevatorPivot.position.set(0, cabinRadius * 0.3, tailFrontZ + stabChord);
    const elevatorMesh = buildWingProfileMesh(tailCombinedChord, tailMainFrac, 1, TAIL_SURFACE_THICKNESS_RATIO, tailSpan * 0.9, darkMat, -tailHingeChordZ, false);
    elevatorPivot.add(elevatorMesh);
    group.add(elevatorPivot);

    // Finnen er hvit/nøytral (samme som resten av skroget) - først forsøkt i aksentrødt, men det leste
    // som en løsrevet, skarpt fargede kloss i skjermbildet i stedet for et naturlig halefinne.
    const finHeight = cabinRadius * 2.3, finChord = tailChord * 0.65, rudderChord = tailChord * 0.5;
    const finBaseY = cabinRadius * 0.55;
    const finCombinedChord = finChord + rudderChord;
    const finMainFrac = finChord / finCombinedChord;
    const finHingeChordZ = finMainFrac * finCombinedChord - finCombinedChord / 2;

    const finFixed = buildWingProfileMesh(finCombinedChord, 0, finMainFrac, TAIL_SURFACE_THICKNESS_RATIO, finHeight, wingMat, 0, false, true);
    finFixed.position.set(0, finBaseY + finHeight / 2, tailFrontZ + finCombinedChord / 2);
    group.add(finFixed);

    const rudderPivot = new THREE.Group();
    rudderPivot.position.set(0, finBaseY + finHeight / 2, tailFrontZ + finChord);
    const rudderMesh = buildWingProfileMesh(finCombinedChord, finMainFrac, 1, TAIL_SURFACE_THICKNESS_RATIO, finHeight * 0.85, darkMat, -finHingeChordZ, false, true);
    rudderPivot.add(rudderMesh);
    group.add(rudderPivot);

    // Understellet må gi nok bakkeklaring til propellsveipet (se bladeLen under). gearHeight avledes
    // direkte av spec.gearOffsetY (fysikkens faktiske bakkekontaktpunkt) delt på visualScale, slik at det
    // visuelle understellet stemmer nøyaktig med fysikkens bakkekontakt for alle tre flystørrelsene.
    // Hjulets BUNNPUNKT (senter minus radius), ikke hjulsenteret, må lande på -gearHeight - derfor stopper
    // strebene ved hjulaksling-høyde (gearHeight - hjulradius), ikke ved selve bakkekontaktpunktet.
    const wheelRadius = 0.04, noseWheelRadius = 0.03;
    const gearHeight = -spec.gearOffsetY / spec.visualScale;
    // Samme bygge-rom-prinsipp som vingen (se buildWingSpan over) - koeffisienten (0.25) er også identisk
    // med resolveGroundContact sin egen gearTrack-formel, slik at de visuelle hjulene sitter nøyaktig der
    // fysikken faktisk registrerer bakkekontakt.
    const gearTrack = buildWingSpan * 0.25;
    const strutLenMain = gearHeight - wheelRadius;
    // Strebene går DIAGONALT fra et festepunkt på skrogets buk og skrår utover til hjulet, som en ekte
    // "cantilever"-fjærbein (typisk høyvinget trener, f.eks. Cessna 152/172-stil) - ikke loddrett rett
    // under hjulsporet, som ville latt streben sveve fritt utenfor det smale skroget uten synlig feste.
    // setFromUnitVectors orienterer sylinderen langs den faktiske retningen mellom de to punktene.
    const gearAttachPoint = new THREE.Vector3(0, -cabinRadius * 0.9, 0.02);
    [-1, 1].forEach(function (side) {
        const wheelPoint = new THREE.Vector3(side * gearTrack / 2, -strutLenMain, 0.02);
        const strutVec = wheelPoint.clone().sub(gearAttachPoint);
        const strutLen = strutVec.length();
        const strut = new THREE.Mesh(new THREE.CylinderGeometry(0.009, 0.009, strutLen, 6), darkMat);
        strut.position.copy(gearAttachPoint).addScaledVector(strutVec, 0.5);
        strut.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), strutVec.clone().normalize());
        group.add(strut);
        const wheel = new THREE.Mesh(new THREE.CylinderGeometry(wheelRadius, wheelRadius, 0.025, 14), darkMat);
        wheel.rotation.z = Math.PI / 2;
        wheel.position.copy(wheelPoint);
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

/* ---------- Fjell i det fjerne (bakgrunn) ----------
   Gjenbrukt fra quad-simulatoren (js/simulator.js: MOUNTAIN_DEFS/buildMountainRange) - samme
   ring-layout (posisjon/høyde/radius/kurve) rundt origo, samme buede (ikke-lineære) radiusprofil
   og per-vertex fargeovergang fra bakkefarge (matcher Sim.buildGroundTexture) via oliven-fjellfot
   og gråstein til ev. snø. Kun det rent DEKORATIVE er tatt med her - ingen kollisjon
   (fastvinget-simmen har ingen terrenghøyde-modell i det hele tatt fra før, kun et flatt
   bakkeplan på y=0, og fjellene her er bevisst langt utenfor der man faktisk flyr) og ingen av
   quad-simmens påskeegg (menneske-skala detaljer ingen ser på cruisehøyde/-avstand herfra). dist+
   radius holder samme trygge margin til himmelkulen (radius 800, se Sim.buildGradientSky) som i
   quad-simmen. */
const FW_MOUNTAIN_DEFS = [
    { angle: 0, dist: 620, height: 69, radius: 110, snow: false, curvePower: 1, jaggedness: 0.7, noiseFreqMul: 0.8,
        subPeaks: [{ f: 0.35, off: 0.3, dirOffset: 2.0 }] },
    { angle: 45, dist: 560, height: 103, radius: 165, snow: true, curvePower: 1.7, jaggedness: 1.1, noiseFreqMul: 1,
        subPeaks: [{ f: 0.55, off: 0.32, dirOffset: 1.3 }, { f: 0.4, off: -0.38, dirOffset: 3.6 }, { f: 0.3, off: 0.45, dirOffset: 5.0 }] },
    { angle: 90, dist: 600, height: 81, radius: 130, snow: false, curvePower: 0.6, jaggedness: 1.3, noiseFreqMul: 1.3,
        subPeaks: [{ f: 0.3, off: 0.28, dirOffset: 1.6 }, { f: 0.25, off: -0.3, dirOffset: 4.2 }] },
    { angle: 135, dist: 540, height: 116, radius: 185, snow: true, curvePower: 0.75, jaggedness: 1.4, noiseFreqMul: 1.4,
        subPeaks: [{ f: 0.6, off: 0.35, dirOffset: 1.1 }, { f: 0.45, off: -0.4, dirOffset: 3.3 }, { f: 0.35, off: 0.4, dirOffset: 5.4 }] },
    { angle: 180, dist: 610, height: 75, radius: 120, snow: false, curvePower: 1.4, jaggedness: 0.65, noiseFreqMul: 0.7,
        subPeaks: [{ f: 0.25, off: 0.25, dirOffset: 2.6 }] },
    { angle: 225, dist: 570, height: 97, radius: 155, snow: true, curvePower: 0.55, jaggedness: 1.2, noiseFreqMul: 1.2,
        subPeaks: [{ f: 0.32, off: 0.3, dirOffset: 1.8 }, { f: 0.28, off: -0.32, dirOffset: 4.5 }] },
    { angle: 270, dist: 590, height: 88, radius: 140, snow: false, curvePower: 1, jaggedness: 1, noiseFreqMul: 1,
        subPeaks: [{ f: 0.55, off: 0.32, dirOffset: 1.3 }, { f: 0.4, off: -0.38, dirOffset: 3.6 }] },
    { angle: 315, dist: 550, height: 109, radius: 175, snow: true, curvePower: 0.8, jaggedness: 1.15, noiseFreqMul: 1.05,
        subPeaks: [{ f: 0.5, off: 0.3, dirOffset: 1.0 }, { f: 0.42, off: -0.35, dirOffset: 3.0 }, { f: 0.3, off: 0.42, dirOffset: 5.2 }] }
];
const FW_MOUNTAIN_PEAKS = (function () {
    const peaks = [];
    FW_MOUNTAIN_DEFS.forEach(function (m, i) {
        const rad = m.angle * Math.PI / 180;
        const x = Math.sin(rad) * m.dist, z = Math.cos(rad) * m.dist;
        const curvePower = m.curvePower || 1;
        const jaggedness = m.jaggedness || 1;
        const noiseFreqMul = m.noiseFreqMul || 1;
        peaks.push({
            x: x, z: z, radius: m.radius, height: m.height, topRadiusFrac: 0.18, curvePower: curvePower,
            jaggedness: jaggedness, noiseFreqMul: noiseFreqMul,
            angle: rad, seed: i * 1.7 + 1, snow: m.snow, isMain: true
        });
        (m.subPeaks || []).forEach(function (sub, si) {
            const subHeight = m.height * sub.f;
            const subRadius = m.radius * (0.5 + sub.f * 0.2);
            const subDir = rad + sub.dirOffset;
            const subDist = m.radius * sub.off;
            peaks.push({
                x: x + Math.sin(subDir) * subDist, z: z + Math.cos(subDir) * subDist,
                radius: subRadius, height: subHeight, topRadiusFrac: 0.2, curvePower: curvePower,
                jaggedness: jaggedness, noiseFreqMul: noiseFreqMul,
                angle: subDir, seed: i * 1.7 + 2 + si * 5.3, snow: false, isMain: false
            });
        });
    });
    return peaks;
})();
const FW_MOUNTAIN_GROUND_COLOR = new THREE.Color(0x3a5f3a);
const FW_MOUNTAIN_FOOTHILL_COLOR = new THREE.Color(0x6e7a4d);
const FW_MOUNTAIN_ROCK_COLOR = new THREE.Color(0x5b6472);
const FW_MOUNTAIN_ROCK_LIGHT_COLOR = new THREE.Color(0x7c8794);
const FW_MOUNTAIN_SNOW_COLOR = new THREE.Color(0xf0f4f8);

function fwMountainProfileRadiusFrac(heightFrac, topRadiusFrac, curvePower) {
    return topRadiusFrac + (1 - topRadiusFrac) * Math.pow(1 - heightFrac, curvePower);
}

function buildFwGradientPeakGeometry(radius, height, seed, colorStops, jaggedness, topRadiusFrac, curvePower, noiseFreqMul) {
    const geo = new THREE.CylinderGeometry(radius, radius, height, 14, 7);
    const pos = geo.attributes.position;
    const colors = new Float32Array(pos.count * 3);
    const tmp = new THREE.Color();
    const p = curvePower || 1;
    const fm = noiseFreqMul || 1;
    for (let i = 0; i < pos.count; i++) {
        const x = pos.getX(i), y = pos.getY(i), z = pos.getZ(i);
        const heightFrac = clamp((y + height / 2) / height, 0, 1);
        const angle = Math.atan2(z, x);
        let radialScale = fwMountainProfileRadiusFrac(heightFrac, topRadiusFrac || 0, p);
        if (jaggedness > 0) {
            const topDamp = 1 - Math.pow(heightFrac, 3) * 0.7;
            const baseDamp = clamp(heightFrac / 0.04, 0, 1);
            const jitter = Math.sin(angle * 5 * fm + seed * 3.1) * 0.18 + Math.sin(angle * 11 * fm + seed * 7.7) * 0.1;
            radialScale *= 1 + jitter * jaggedness * topDamp;
            pos.setY(i, y + Math.sin(angle * 7 * fm + seed * 4.3) * height * 0.03 * jaggedness * topDamp * baseDamp);
        }
        pos.setX(i, x * radialScale);
        pos.setZ(i, z * radialScale);
        let c0 = colorStops[0], c1 = colorStops[colorStops.length - 1];
        for (let s = 0; s < colorStops.length - 1; s++) {
            if (heightFrac >= colorStops[s].frac && heightFrac <= colorStops[s + 1].frac) {
                c0 = colorStops[s]; c1 = colorStops[s + 1];
                break;
            }
        }
        const span = Math.max(1e-6, c1.frac - c0.frac);
        const t = clamp((heightFrac - c0.frac) / span, 0, 1);
        tmp.copy(c0.color).lerp(c1.color, t);
        colors[i * 3] = tmp.r; colors[i * 3 + 1] = tmp.g; colors[i * 3 + 2] = tmp.b;
    }
    geo.setAttribute("color", new THREE.BufferAttribute(colors, 3));
    geo.computeVertexNormals();
    return geo;
}

function buildMountainRange() {
    const group = new THREE.Group();
    const peakMat = new THREE.MeshStandardMaterial({ vertexColors: true, flatShading: true, roughness: 1 });
    FW_MOUNTAIN_PEAKS.forEach(function (peak) {
        const colorStops = (peak.isMain && peak.snow)
            ? [
                { frac: 0, color: FW_MOUNTAIN_GROUND_COLOR },
                { frac: 0.16, color: FW_MOUNTAIN_FOOTHILL_COLOR },
                { frac: 0.42, color: FW_MOUNTAIN_ROCK_COLOR },
                { frac: 0.82, color: FW_MOUNTAIN_ROCK_LIGHT_COLOR },
                { frac: 1, color: FW_MOUNTAIN_SNOW_COLOR }
            ]
            : [
                { frac: 0, color: FW_MOUNTAIN_GROUND_COLOR },
                { frac: peak.isMain ? 0.18 : 0.2, color: FW_MOUNTAIN_FOOTHILL_COLOR },
                { frac: 0.5, color: FW_MOUNTAIN_ROCK_COLOR },
                { frac: 1, color: FW_MOUNTAIN_ROCK_LIGHT_COLOR }
            ];
        const mesh = new THREE.Mesh(
            buildFwGradientPeakGeometry(
                peak.radius, peak.height, peak.seed, colorStops,
                peak.jaggedness, peak.topRadiusFrac, peak.curvePower, peak.noiseFreqMul
            ),
            peakMat
        );
        mesh.position.set(peak.x, peak.height / 2, peak.z);
        mesh.rotation.y = peak.angle;
        group.add(mesh);
    });
    return group;
}

function initScene() {
    const canvas = document.getElementById("simCanvas");
    renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    // Se identisk kommentar i simulator.js/initScene - WebGL-kontekst kan tapes ved GPU-bytte når
    // vinduet flyttes mellom skjermer (spesielt maskiner med to grafikkort), og uten disse fortsetter
    // animate() å tegne mot en død kontekst helt til siden lastes på nytt manuelt.
    canvas.addEventListener("webglcontextlost", function (e) {
        e.preventDefault();
        console.warn("[FFI-UAS] WebGL-kontekst tapt (f.eks. GPU-bytte ved flytting mellom skjermer) - laster siden på nytt...");
    }, false);
    canvas.addEventListener("webglcontextrestored", function () {
        location.reload();
    }, false);

    scene = new THREE.Scene();
    scene.add(Sim.buildGradientSky());
    scene.add(buildGround());
    scene.add(buildMountainRange());
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
    // Near-planene er løftet (0.1/0.05 -> 0.3/0.1): dybdebufferets oppløsning er proporsjonal med
    // near-verdien, og de gamle verdiene ga synlig z-fighting på bakke-dekalene sett fra lufta
    // (sammen med polygonOffset-biasen i groundDecalProps, som er hovedgrepet).
    chaseCamera = new THREE.PerspectiveCamera(60, aspect, 0.3, 1500);
    chaseCameraController = Sim.createChaseCameraController(chaseCamera, document.getElementById("simCanvas"), {
        defaultPitch: Math.atan2(3.2, 15),
        zoomMin: 4, zoomMax: 60,
        initialZoom: 4, // starter nærmest mulig ved innlasting av siden
        smoothingBase: 0.0015,
        lookAtOffsetY: 1
    });
    fpvCamera = new THREE.PerspectiveCamera(90, aspect, 0.1, 1500);
    fpvCamera.position.set(0, 0.08, -0.55);
    fpvCamera.rotation.x = THREE.MathUtils.degToRad(settings.fpvTiltDeg);

    rebuildPlaneMesh();

    // VLOS-observatøren står rett ved siden av rullebanen (ikke på den) og ser nedover mot avgangsenden.
    vlosCamera = new THREE.PerspectiveCamera(50, aspect, 0.5, 1500); // høy near = bedre dybdepresisjon på avstand (se chaseCamera)
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
    viewportWatcher = Sim.createViewportWatcher(renderer, document.querySelector(".sim-page"), resizeRenderer);
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

// Chase-kamera med manuell orbit - delt logikk med quad-simulatoren, se Sim.createChaseCameraController
// i simulator-common.js. Instansieres i initScene() (må vente til chaseCamera faktisk finnes),
// oppdateres fra animate().
let chaseCameraController;

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
        // Rått gamepad-signal glattes nå litt (i motsetning til før, som satte disse direkte) - en
        // fysisk RC-sender har uansett en viss reaksjonstid i selve gimbalen/fingerbevegelsen, mens et
        // rått, ufiltrert lesernummer kan gi et momentant, fullt utslag i én eneste fysikk-tick (en
        // umiddelbar dreiemoment-spike) på en måte som ikke er fysisk mulig med en ekte sender. Kortere
        // tidskonstant enn tastaturets STICK_RAMP_TIME siden en ekte gimbal normalt er raskere/mer
        // presis enn en syntetisk tastatur-rampe, men fortsatt ikke null.
        inputState.stick.roll = rampStick(inputState.stick.roll, readStickAxis(gp, gamepadMap.aileron), dt, GAMEPAD_STICK_RAMP_TIME);
        inputState.stick.pitch = rampStick(inputState.stick.pitch, readStickAxis(gp, gamepadMap.elevator), dt, GAMEPAD_STICK_RAMP_TIME);
        inputState.stick.yaw = rampStick(inputState.stick.yaw, readStickAxis(gp, gamepadMap.rudder), dt, GAMEPAD_STICK_RAMP_TIME);
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
// WING_ALPHA0_DEG: nullløft-vinkelen til en kambret vingeprofil (buet overside/flat underside, se
// buildAirfoilProfileShape) - en SYMMETRISK profil har løft=0 ved AoA=0, men en kambret profil gir
// fortsatt litt løft ved AoA=0 (og trenger en NEGATIV AoA, ~-2 til -4°, for å gi null løft). Delt/lik for
// alle klasser (samme profilfamilie), ikke per-klasse - konsistent med hvordan f.eks. DIHEDRAL_EFFECT
// også er delt. Gir tre realistiske ting gratis: lavere marsjfart-AoA (mindre indusert drag i cruise),
// et reelt vinge-stigemoment (Cm0, via at drag/løft nå er asymmetrisk fordelt om AoA=0 - se
// dragCoefficient), og at rygg-flyging (invertert) krever et tydelig dytt/negativt utslag for å holde
// høyden i stedet for å "falle" naturlig oppover.
const WING_ALPHA0_DEG = -3;

function liftCoefficient(aoaDeg, spec) {
    const stall = spec.stallAngleDeg;
    const absA = Math.abs(aoaDeg);
    const sign = aoaDeg < 0 ? -1 : 1;
    if (absA < stall) return spec.clSlope * (aoaDeg - WING_ALPHA0_DEG);
    // signedPeak: verdien den lineære formelen over FAKTISK gir ved grensevinkelen for DENNE siden (+/-
    // stall) - en kambret profil gir ulik CL-magnitude for +stall og -stall (se WING_ALPHA0_DEG), så dette
    // kan ikke lenger være ett delt, symmetrisk tall slik "peak" var før kamber ble lagt til. Selve
    // steilevinkelen (stall) er fortsatt den RÅ, geometriske AoA'en, HELT uendret av kamber-skiftet - kun
    // formen/magnituden på løftkurven som ruller av den er påvirket. Garantert kontinuerlig med
    // linjeformelen over per konstruksjon (samme uttrykk, evaluert nøyaktig ved grensen).
    const signedPeak = spec.clSlope * (sign * stall - WING_ALPHA0_DEG);
    if (absA < stall + STALL_POST_RANGE_DEG) {
        const progress = (absA - stall) / STALL_POST_RANGE_DEG;
        return signedPeak * (1 - progress) + signedPeak * 0.3 * progress;
    }
    // Flate-plate-formelen for dyp steiling skaleres slik at den er KONTINUERLIG med overgangssonens
    // sluttverdi (0.3*signedPeak) akkurat ved grensevinkelen, for enhver klasses stallAngleDeg - en fast
    // skalafaktor uavhengig av grensevinkelen ga tidligere et diskontinuerlig CL-sprang der (~49% for
    // Middels), som viste seg som rykkete, retningsvekslende rulling når AoA naturlig svinget over grensen
    // under en steiling.
    const boundaryDeg = stall + STALL_POST_RANGE_DEG;
    const boundaryRaw = Math.abs(Math.sin(2 * THREE.MathUtils.degToRad(boundaryDeg)));
    const flatPlateScale = boundaryRaw > 0.05 ? 0.3 / boundaryRaw : 0.6;
    return sign * Math.abs(signedPeak) * flatPlateScale * Math.abs(Math.sin(2 * THREE.MathUtils.degToRad(absA)));
}

// groundEffectFactor (0..1, default 1 = ingen effekt) skalerer KUN den induserte motstanden - se
// GROUND_EFFECT_WINGSPAN_FACTOR-merknaden ved beregningen i stepPhysics for hvorfor kun indusert drag
// (ikke løft) er modellert: bakkeeffekten kommer av at bakken hindrer vingens nedvask-virvler i å
// utvikle seg fullt ut, som reduserer indusert motstand - det er den effekten som gir "flyting"/økt
// rekkevidde i bakkeeffekt ved landing, og er den vanligste, mest merkbare av de to.
function dragCoefficient(aoaDeg, spec, groundEffectFactor) {
    const aoaRad = THREE.MathUtils.degToRad(aoaDeg);
    // Indusert drag måles fra samme nullløft-vinkel som liftCoefficient nå bruker (WING_ALPHA0_DEG) -
    // indusert drag er reelt sett en funksjon av LØFTET (~CL²), ikke av den rå geometriske AoA² alene.
    // Uten dette skiftet ville vingen ha reelt løft ved AoA=0 (fra kamberet) men modellen ville likevel
    // late som om indusert drag var omtrent null der - inkonsistent med selve løftkurven over.
    const shiftedRad = THREE.MathUtils.degToRad(aoaDeg - WING_ALPHA0_DEG);
    let cd = spec.cd0 + spec.inducedDragK * shiftedRad * shiftedRad * (groundEffectFactor === undefined ? 1 : groundEffectFactor);
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

    // Sikkerhetsnett mot uforklarlig dreining i stillstand: tvinger orienteringen til identitet (rett ned
    // rullebanen, nivå), men KUN før flyet noensinne har vært i luften (hasBeenAirborne) - ellers ville
    // dette teleportert et fly som lander og bremser ned under 0.4 m/s tilbake til spawn-retningen, uansett
    // hvilken vei det faktisk landet. Vinkelhastighetene nullstilles alltid uansett (forhindrer restspinn).
    if (planeState.velocity.length() < 0.4) {
        if (!planeState.hasBeenAirborne) planeState.quaternion.identity();
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

    const q = planeState.quaternion;
    const invQ = q.clone().invert();
    const airVelWorld = planeState.velocity.clone().sub(currentWindVector);
    const localAirVel = airVelWorld.clone().applyQuaternion(invQ);
    const airspeed = airVelWorld.length();
    lastAirspeed = airspeed;
    const aoaDeg = airspeed > 0.3 ? THREE.MathUtils.radToDeg(Math.atan2(-localAirVel.y, -localAirVel.z)) : 0;
    const sideslipDeg = airspeed > 0.3 ? THREE.MathUtils.radToDeg(Math.atan2(localAirVel.x, -localAirVel.z)) : 0;

    const forwardAirspeedIntoProp = Math.max(-localAirVel.z, 0);
    // En ekte (særlig fastpitch) propell mister trekkraft omtrent lineært med farten, fra full statisk
    // trekkraft ved V=0 til ~null idet flyet nærmer seg propellens "pitch speed" (spec.propPitchSpeed) -
    // uten dette var toppfarten satt av drag alene (urealistisk høy for en liten trener), og gass av i
    // høy fart ga ikke den brattere glidebanen en vindmøllende propell faktisk gir.
    const thrustForce = planeState.engineOn
        ? throttleShaped * spec.maxThrust * Math.max(0, 1 - forwardAirspeedIntoProp / spec.propPitchSpeed)
        : 0;

    // Propellstrøm (propwash) - se PROP_DISK_AREA_RATIO-merknaden ved konstanten. Momentum-teori for en
    // aktuator-skive: skiven induserer en hastighet v_i som løser T = 2*rho*A*v_i*(V0+v_i), der V0 er
    // flyets EGEN forover-luftfart inn i skiven (klemt til >=0 - motoren "suger" uansett, selv i revers-
    // eller null-fart). Full slipstrøm-hastighetsøkning langt bak skiven (der halen/finnen sitter) er
    // 2*v_i = sqrt(V0² + 2T/(rho*A)) - V0 (løsning av annengradslikningen over). Denne ekstra farten
    // legges KUN til halens/finnens EGEN lokale luftstrøm (se tailTorqueAtPitchRate/finTorqueAtYawRate),
    // ikke vingens - en enkeltmotors nese-propell vasker i praksis kun skrog/hale, ikke hele vingespennet.
    const propwashEffectiveArea = spec.wingArea * PROPWASH_EFFECTIVE_AREA_RATIO;
    const propwashDeltaV = thrustForce > 0.01
        ? Math.sqrt(forwardAirspeedIntoProp * forwardAirspeedIntoProp + (2 * thrustForce) / (AIR_DENSITY * propwashEffectiveArea)) - forwardAirspeedIntoProp
        : 0;

    // Bakkeeffekt - se GROUND_EFFECT_HEIGHT_FACTOR-merknaden ved konstanten. h/b = VINGENS høyde over
    // bakken (IKKE CG/planeState.position.y direkte - dette er høyvinget, så vingen sitter et stykke over
    // CG, se WING_MOUNT_HEIGHT_RATIO-merknaden) delt på vingespennet. Faktoren går mot 0 (ingen indusert
    // drag) helt ved bakken, og mot 1 (ingen effekt) idet høyden nærmer seg/overstiger ett vingespenn - en
    // vanlig, enkel empirisk tilnærming (Wieselsberger-lignende form).
    const wingHeightAboveGround = Math.max(planeState.position.y + CABIN_RADIUS_BUILD * WING_MOUNT_HEIGHT_RATIO * spec.visualScale, 0);
    const groundEffectRatio = GROUND_EFFECT_HEIGHT_FACTOR * wingHeightAboveGround / spec.wingSpan;
    const groundEffectFactor = (groundEffectRatio * groundEffectRatio) / (1 + groundEffectRatio * groundEffectRatio);
    // Løft-boost nær bakken (se GROUND_EFFECT_LIFT_BOOST_MAX-merknaden ved konstanten) - størst (1+MAX)
    // helt ved bakken (groundEffectFactor=0), forsvinner til nøyaktig 1 (ingen effekt) idet
    // groundEffectFactor->1 (over ca. ett vingespenn høyde). Brukes som en enkel multiplikator på
    // vingens løft der den beregnes (liftRight/liftLeft og wingTorqueForce), IKKE inni selve
    // liftCoefficient()-kurven - holder steile-/kambermatematikken der uberørt.
    const groundEffectLiftFactor = 1 + GROUND_EFFECT_LIFT_BOOST_MAX * (1 - groundEffectFactor);

    // Rorenes avbøyning (-1..1) beregnes FØR vinge-/hale-fysikken under, siden skråror (aileron) nå også
    // påvirker vingens egen angrepsvinkel (se rightWing/leftWing) og trenger rollDeflection klar til det.
    // Manual = direkte fra pinnen, formet av rate-kurven (her brukt som en avbøynings-/følsomhetskurve,
    // normalisert mot maxRate - beholder samme rate-panel-UI som quaden, men uten å late som pinnen
    // kommanderer en rotasjonshastighet direkte). Stabilized er en enkel autopilot som avbøyer AKKURAT DE
    // SAMME rorflatene proporsjonalt med vinkelavviket i stedet for direkte fra pinnen.
    let rollDeflection, pitchDeflection, yawDeflection;
    if (planeState.flightMode === "manual") {
        // Trim er IKKE lenger blandet inn i selve pinne-avbøyningen her - se tailAoaDeg under, der
        // elevatorTrimDeg påvirker halens angrepsvinkel DIREKTE (som en ekte trim-fane), uavhengig av
        // rorutslaget. pitchDeflection er dermed et rent uttrykk for PINNENS egen posisjon.
        rollDeflection = clamp(computeRate(stick.roll, rates.aileron) / rates.aileron.maxRate, -1, 1);
        pitchDeflection = clamp(computeRate(stick.pitch, rates.elevator) / rates.elevator.maxRate, -1, 1);
        yawDeflection = clamp(computeRate(stick.yaw, rates.rudder) / rates.rudder.maxRate, -1, 1);
    } else {
        const euler = new THREE.Euler().setFromQuaternion(q, "YXZ");
        const currentPitchDeg = -THREE.MathUtils.radToDeg(euler.x);
        const currentBankDeg = -THREE.MathUtils.radToDeg(euler.z);
        const targetBankDeg = stick.roll * MAX_BANK_ANGLE;
        // Trim er bevisst IKKE med her - Stabilized er en selvnivellerende autopilot og skal returnere
        // til det KOMMANDERTE stigningsmålet uansett trim (auto-trim under kompenserer i stedet for at
        // trim flytter selve nivelleringsmålet).
        const targetPitchDeg = stick.pitch * MAX_PITCH_ANGLE;
        // D-ledd: dempet mot raten vinkelen FAKTISK endret seg med forrige tick (finite-difference på
        // currentBankDeg/currentPitchDeg selv, ikke via angularVelocity.roll/pitch sin egen
        // aksekonvensjon - unngår dermed enda en risikabel fortegns-utledning; denne er alltid riktig
        // signert per konstruksjon siden den måler nøyaktig den samme vinkelen som P-leddet bruker).
        const bankRateDeg = (currentBankDeg - planeState.prevBankDeg) / dt;
        const pitchRateDegForD = (currentPitchDeg - planeState.prevPitchDegForD) / dt;
        planeState.prevBankDeg = currentBankDeg;
        planeState.prevPitchDegForD = currentPitchDeg;
        // Lavpassfiltrert FØR bruk i D-leddet - se STABILIZED_D_FILTER_TAU-merknaden ved konstanten
        // (fjerner "derivative kick"-hakking uten å gjøre selve stabiliseringen merkbart tregere).
        const filterBlend = Math.min(1, dt / STABILIZED_D_FILTER_TAU);
        planeState.filteredBankRateDeg += (bankRateDeg - planeState.filteredBankRateDeg) * filterBlend;
        planeState.filteredPitchRateDeg += (pitchRateDegForD - planeState.filteredPitchRateDeg) * filterBlend;

        rollDeflection = clamp((targetBankDeg - currentBankDeg) / STABILIZED_BANK_AUTHORITY_DEG - planeState.filteredBankRateDeg * STABILIZED_BANK_D_GAIN, -1, 1);
        pitchDeflection = clamp((targetPitchDeg - currentPitchDeg) / STABILIZED_PITCH_AUTHORITY_DEG - planeState.filteredPitchRateDeg * STABILIZED_PITCH_D_GAIN, -1, 1);
        // Ingen automatisk sideror-koordinering her - en tidligere åpen-løkke-versjon (proporsjonal med
        // kommandert krengning) fightet mot finnens ekte aerodynamiske respons (retningsstabilitet+demping)
        // og vingenes adverse yaw, som ga periodisk sideror-oscillering i svinger. Sideroret i Stabilized
        // er derfor rent pinnestyrt (som Manual) - koordineringen kommer naturlig fra aerodynamikken.
        yawDeflection = clamp(computeRate(stick.yaw, rates.rudder) / rates.rudder.maxRate, -1, 1);

        // Auto-trim: trim-hjulet "følger etter" sakte og avlaster roret mot null utslag, akkurat som en
        // ekte koblet autopilot med trim-servo. Kjører på et LAVPASSFILTRERT utslag (autoTrimFilteredDeflection),
        // ikke det momentane pitchDeflection-tallet direkte - ellers ville trimmen jaget hvert forbigående
        // pinneutslag (f.eks. midt i en sving) i stedet for kun å ta over den VEDVARENDE delen. Gir tre
        // ting gratis: (1) Stabilized nivellerer nå nøyaktig i steady state (en ren P-regulator alene har
        // et lite, fartsavhengig etterslep), (2) trim-verdien er allerede riktig innstilt idet du bytter
        // til Manual, i stedet for et brått jerk ved bytte, og (3) trimmer ikke bort en forbigående
        // manøver, kun en vedvarende ubalanse.
        planeState.autoTrimFilteredDeflection += (pitchDeflection - planeState.autoTrimFilteredDeflection) * Math.min(1, dt / AUTO_TRIM_FILTER_TAU);
        // Fortegn: pitchDeflection>0 betyr halen for øyeblikket dyttes mot MER nese-ned (se tailAoaDeg
        // under) - trim skal da beveges i MOTSATT retning av utslaget for å overta den samme jobben og la
        // utslaget slappe av mot null (verifisert med likevektsanalyse: trim_dot = -k*deflection er den
        // eneste av de to fortegnene som faktisk konvergerer, ikke bare tilsynelatende riktig retning).
        planeState.elevatorTrimDeg = clamp(
            planeState.elevatorTrimDeg - planeState.autoTrimFilteredDeflection * AUTO_TRIM_RATE_DEG_PER_SEC * dt,
            -TRIM_RANGE_DEG, TRIM_RANGE_DEG
        );
    }
    // Lagres for den visuelle animasjonen av rorflatene (se updatePlaneVisual) - kjøres i rendring-loopet,
    // ikke i det faste fysikk-tidssteget, så den leser siste beregnede verdi herfra.
    planeState.lastRollDeflection = rollDeflection;
    planeState.lastPitchDeflection = pitchDeflection;
    planeState.lastYawDeflection = yawDeflection;

    // Venstre/høyre vinge separat (se wingLocalAirspeedAoa) - gir differensiallift som blir til rulldreiemoment.
    const halfSpan = spec.wingSpan / 2;
    const rollRate = planeState.angularVelocity.roll;
    const yawRate = planeState.angularVelocity.yaw;
    const rightRot = new THREE.Vector3(0, rollRate * halfSpan, -yawRate * halfSpan);
    const leftRot = new THREE.Vector3(0, -rollRate * halfSpan, yawRate * halfSpan);
    const rightWingBase = wingLocalAirspeedAoa(localAirVel, rightRot);
    const leftWingBase = wingLocalAirspeedAoa(localAirVel, leftRot);
    // Skråror (aileron) endrer hver vinges EFFEKTIVE angrepsvinkel motsatt av hverandre (nedadgående
    // klaff = mer AoA, oppadgående = mindre), OVENPÅ den rotasjonsbaserte differensialen - begge kilder
    // til vingedypp/spinn-tendens (rorutslag OG ren rotasjon) bruker samme mekanisme.
    // controlAoaDeg holdes ADSKILT fra aoaDeg og legges til som et eget, lineært CL-tillegg (se
    // liftRight/liftLeft under) i stedet for inni liftCoefficient()'s steilekurve: et fullt rorutslag
    // (opptil 11°/vinge) presset tidligere den nedadgående vingen inn i steilingens flate område ved
    // nettopp lav fart/høy AoA (der rull faktisk trengs mest, f.eks. under takeoff-rullingen) og kuttet
    // mesteparten av differensialløftet - mens høyderor/sideror aldri gikk gjennom noen steilekurve i
    // det hele tatt. Dette var den reelle årsaken til at rull konsekvent føltes tregt.
    const rightWing = { airspeed: rightWingBase.airspeed, aoaDeg: rightWingBase.aoaDeg, controlAoaDeg: -rollDeflection * AILERON_MAX_AOA_DEG * 0.5 };
    const leftWing = { airspeed: leftWingBase.airspeed, aoaDeg: leftWingBase.aoaDeg, controlAoaDeg: rollDeflection * AILERON_MAX_AOA_DEG * 0.5 };

    const halfWingArea = spec.wingArea / 2;
    const liftRight = 0.5 * AIR_DENSITY * rightWing.airspeed * rightWing.airspeed * halfWingArea * (liftCoefficient(rightWing.aoaDeg, spec) + spec.clSlope * rightWing.controlAoaDeg) * groundEffectLiftFactor;
    const liftLeft = 0.5 * AIR_DENSITY * leftWing.airspeed * leftWing.airspeed * halfWingArea * (liftCoefficient(leftWing.aoaDeg, spec) + spec.clSlope * leftWing.controlAoaDeg) * groundEffectLiftFactor;
    const totalLiftMag = liftRight + liftLeft;

    // Rull-/gir-koblingen beregnes fra vingenes EGEN lokale luftstrøm-RETNING (ikke bare fart/AoA-
    // størrelse som liftRight/Left over) - når en vinge har en lokal Y-komponent i luftstrømmen (fra
    // rotasjon, se rightRot/leftRot), tipper den vingens løft-/drag-vektor litt forover/bakover i
    // kroppen, ikke bare opp/ned. Dette ER de ekte aerodynamiske derivatene Clr (rull fra gir-rate) og
    // Cnp (gir fra rull-rate): en vinge som beveger seg NEDOVER under en rulling får en forover-tippet
    // løftvektor, som girer nesen MOTSATT rullretningen - men KUN mens rotasjonsraten faktisk er ulik
    // null. Ved en etablert, konstant krengning (rollRate=0, yawRate=0) er rightRot/leftRot begge null,
    // vingene har identisk lokal luftstrøm, og denne koblingen forsvinner helt av seg selv - flyet
    // "slipper" ALDRI en etablert krengning på grunn av dette, akkurat som i et ekte fly.
    function wingTorqueForce(rot, extraAoaDeg) {
        const vy = localAirVel.y + rot.y, vz = localAirVel.z + rot.z;
        const speed = Math.sqrt(vy * vy + vz * vz);
        if (speed < 0.3) return { fy: 0, fz: 0 };
        const baseAoaDeg = THREE.MathUtils.radToDeg(Math.atan2(-vy, -vz));
        const wingAoaDeg = baseAoaDeg + extraAoaDeg;
        const qDynWing = 0.5 * AIR_DENSITY * speed * speed * halfWingArea;
        // Løft: skrårorets bidrag (extraAoaDeg) holdes UTENFOR liftCoefficient()'s steilekurve - se
        // merknaden ved rightWing/leftWing over for hvorfor (samme fiks, samme begrunnelse, brukt her
        // for selve rull-/gir-dreiemoment-beregningen). Drag bruker fortsatt DEN KOMBINERTE vinkelen -
        // ror-utslag skal fortsatt gi ekstra motstand/adverse yaw, det er kun løft-steilingen som ikke
        // skal "se" roret som en del av vingens egen angrepsvinkel.
        const liftMag = qDynWing * (liftCoefficient(baseAoaDeg, spec) + spec.clSlope * extraAoaDeg) * groundEffectLiftFactor;
        const dragMag = qDynWing * dragCoefficient(wingAoaDeg, spec, groundEffectFactor);
        const invSpeed = 1 / speed;
        return {
            fy: liftMag * (-vz * invSpeed) + dragMag * (-vy * invSpeed),
            fz: liftMag * (vy * invSpeed) + dragMag * (-vz * invSpeed)
        };
    }
    // Vinge-differensialens bidrag til BÅDE rull (løft-differansen) og gir (drag-differansen, Cnp/
    // "adverse yaw" - transient, OG samme mekanisme under en spinn: en steilet vinge har mye mer drag -
    // se dragCoefficient - som girer nesen mot den steilede siden og FORSTERKER rotasjonen), som funksjon
    // av ANTATT rull-/gir-rate (ikke bare gjeldende verdi) - se merknaden ved rollWingDampCoeff for
    // hvorfor dette trengs (samme numeriske-ustabilitet-fiks som pitch/yaw fikk for halen/finnen, se der).
    function wingCoupledTorques(rollRateAssumed, yawRateAssumed) {
        const rRot = new THREE.Vector3(0, rollRateAssumed * halfSpan, -yawRateAssumed * halfSpan);
        const lRot = new THREE.Vector3(0, -rollRateAssumed * halfSpan, yawRateAssumed * halfSpan);
        const rF = wingTorqueForce(rRot, -rollDeflection * AILERON_MAX_AOA_DEG * 0.5);
        const lF = wingTorqueForce(lRot, rollDeflection * AILERON_MAX_AOA_DEG * 0.5);
        return { roll: (rF.fy - lF.fy) * halfSpan, yaw: (lF.fz - rF.fz) * halfSpan };
    }
    // Bevisst INGEN separat "yaw induserer rull"-ledd her utover rollWingF0 sin egen yawRate-avhengighet -
    // et tidligere forsøk på et slikt ekstra-ledd reagerte på ALL yawRate uansett kilde, inkludert adverse
    // yaw fra egne skråror, som skapte en selvforsterkende tilbakekobling (mer aileron -> mer adverse yaw
    // -> mer kunstig rullmotstand) og kvalte rull-autoriteten. rollWingF0 alene gir samme fysiske effekt,
    // riktig skalert til den faktiske kilden til yaw.

    const qDynTotal = 0.5 * AIR_DENSITY * airspeed * airspeed * spec.wingArea;
    // Skrogets sidekraft/-drag i sideslip (se FUSELAGE_SIDE_AREA_RATIO/FUSELAGE_SIDE_CD-merknaden) - et
    // flatplate-ledd proporsjonalt med sin²(sideslip), lagt OVENPÅ vingens/halens egen AoA-baserte drag
    // (dragCoefficient over ser kun AoA, ikke sideslip). Dette er det som gjør en "forward slip" (kryssede
    // ror) til en reell, kraftig synkefart-økende manøver i stedet for bare en kosmetisk gir-vinkel - uten
    // dette bar KUN finnen sidekraften/-draget i sideslip, som ga en for svak/lett slip-effekt.
    const fuselageSideArea = spec.wingArea * FUSELAGE_SIDE_AREA_RATIO;
    const sideslipRad = THREE.MathUtils.degToRad(sideslipDeg);
    const fuselageCrossflowDrag = 0.5 * AIR_DENSITY * airspeed * airspeed * fuselageSideArea * FUSELAGE_SIDE_CD
        * Math.sin(sideslipRad) * Math.sin(sideslipRad);
    const dragMag = qDynTotal * dragCoefficient(aoaDeg, spec, groundEffectFactor) + fuselageCrossflowDrag;
    // Dreiemoment (roll/yaw) = avbøyning * dynamisk trykk * kontrolleffektivitet, dempet av et moment
    // proporsjonalt med flyets egen vinkelhastighet (aerodynamisk demping). Begge ledd skalerer med V^2 -
    // svaret blir derfor naturlig trått/"mushy" ved lav fart og fast ved høy fart, uten noen kunstig
    // svekkingsfaktor. Samme aksekonvensjon-negasjon (forward=-Z) som resten av fysikken/quad-simulatoren.
    const qDynControl = 0.5 * AIR_DENSITY * airspeed * airspeed;
    // Dihedral-effekt: sideslip (IKKE gir-RATE, men selve sideslip-VINKELEN) gir et VEDVARENDE rullmoment
    // som består så lenge sideslippet varer, ikke bare et forbigående napp - det er dette som lar sideror
    // alene etablere OG holde en krengning, og "kryssede ror" holde flyet rett fram i en sideslip. Skalert
    // med qDynControl som alle andre aerodynamiske momenter i modellen (konstanten er rekalibrert til å gi
    // samme effekt som før ved Liten sin marsjfart, ~12 m/s) - uten denne skaleringen forble momentet
    // fullt til stede selv i stillstand, siden sideslipDeg går mot ±90° ved lav/null fart uansett hvor svak
    // vinden faktisk er (samme geometriske artefakt som RUDDER_MAX_AOA_DEG-fiksen adresserte), mens
    // roll-dempingen (som OGSÅ skalerer med V²) nesten forsvant - flyet krenget/veltet i selv svak vind.
    const rollTorqueFromDihedral = -DIHEDRAL_EFFECT * qDynControl * sideslipDeg;

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

    // Statisk lateral bakkefriksjon (se GROUND_LATERAL_FRICTION_COEFF-merknaden ved konstanten) - motstår
    // selve SIDEKRAFTEN (f.eks. vind-drag i kryssvind) mens flyet er på bakken, ikke bare farten som
    // resolveGroundContact sin egen lateralSpeed-demping gjør. Samme "kanseller helt/reduser bare"-
    // struktur som yaw-motstykket (maxGroundYawTorque under) - kansellerer små/moderate sidekrefter helt
    // (accel er allerede per masseenhet, så GRAVITY*koeffisient er selve terskelen direkte, uten å måtte
    // gange/dele med spec.mass).
    if (planeState.onGround) {
        const rightWorldGround = new THREE.Vector3(1, 0, 0).applyQuaternion(q);
        const lateralAccel = accel.dot(rightWorldGround);
        const maxStaticLateralAccel = GROUND_LATERAL_FRICTION_COEFF * GRAVITY;
        if (Math.abs(lateralAccel) <= maxStaticLateralAccel) {
            accel.addScaledVector(rightWorldGround, -lateralAccel);
        } else {
            accel.addScaledVector(rightWorldGround, -Math.sign(lateralAccel) * maxStaticLateralAccel);
        }
    }

    // Stigemoment: EKTE vekt-og-balanse-modell i stedet for et flatt kontrolleffektivitet-tall. Vingens
    // egen AC er forenklet plassert rett i CG (bidrar ikke med moment - en vanlig, standard forenkling;
    // halens arm er uansett den dominerende bidragsyteren i et ekte fly), så ALT stigemoment kommer fra
    // halens EGET løft, som virker halens arm bak CG (samme "vippe"-prinsipp som en huske: løft bak CG
    // gir nese ned, og omvendt).
    const tailArm = spec.wingSpan * TAIL_ARM_RATIO;
    const tailArea = spec.wingArea * TAIL_AREA_RATIO;
    const tailClSlope = spec.clSlope * TAIL_CL_SLOPE_RATIO;
    // Halens EGEN lokale luftfart pga rotasjon (omega x arm) - nøyaktig samme prinsipp som venstre/høyre
    // vinge-differensialen lenger opp. Dette ER selve pitch-dempingen (Cmq): roterer nesen opp, øker
    // halens angrepsvinkel (og dermed løft), som vipper nesen tilbake ned - ekte aerodynamisk demping,
    // ikke en håndjustert konstant. Parameterisert over antatt pitchRate (ikke bare gjeldende verdi) -
    // se merknaden ved pitchDampCoeff for hvorfor.
    function tailTorqueAtPitchRate(pitchRateAssumed) {
        const rot = new THREE.Vector3(yawRate * tailArm, -pitchRateAssumed * tailArm, 0);
        const v = localAirVel.clone().add(rot);
        // Propwash: gjør luftstrømmen ved halen sterkere (mer negativ z, samme retning som "forover-
        // relativ" luft allerede peker) UAVHENGIG av flyets egen fart - se propwashDeltaV over.
        v.z -= propwashDeltaV;
        const speedSq = v.lengthSq();
        // Nedvask fra vingen (se DOWNWASH_RATIO) - trekkes fra halens EGEN baseAoa FØR ror/trim legges
        // til, siden nedvasken påvirker den naturlige luftstrøm-vinkelen halen ser, ikke pilotens utslag.
        const baseAoaFreestream = speedSq > 0.09 ? THREE.MathUtils.radToDeg(Math.atan2(-v.y, -v.z)) : 0;
        const baseAoa = baseAoaFreestream - DOWNWASH_RATIO * aoaDeg;
        // Høyderor (ELEVATOR_MAX_AOA_DEG) og trim endrer begge halens EFFEKTIVE angrepsvinkel - roret er
        // pilotens direkte utslag; trim gjelder nå i BEGGE modus (i Manual satt av piloten selv, i
        // Stabilized satt av auto-trim-integratoren over).
        const aoa = clamp(baseAoa + pitchDeflection * ELEVATOR_MAX_AOA_DEG - planeState.elevatorTrimDeg, -35, 35);
        const lift = 0.5 * AIR_DENSITY * speedSq * tailArea * (tailClSlope * aoa);
        return -tailArm * lift;
    }

    // Gir-momentet (yaw): NØYAKTIG samme "vekt-og-balanse"-prinsipp som halen over, ikke lenger en flat
    // YAW_CONTROL_EFFECTIVENESS/YAW_DAMPING/yawStability-trio. Finnen henger på SAMME moment-arm som
    // halen (samme skrogstasjon), og produserer et sideveis "løft" fra sin EGEN lokale sideslip-vinkel.
    // Parameterisert over antatt yawRate av samme grunn som tailTorqueAtPitchRate.
    const finArm = tailArm;
    const finArea = spec.wingArea * FIN_AREA_RATIO;
    const finClSlope = spec.clSlope * FIN_CL_SLOPE_RATIO;
    function finTorqueAtYawRate(yawRateAssumed) {
        // Finnens EGEN lokale luftstrøm pga rotasjon: X-komponenten (sideveis) får bidrag fra gir-raten
        // (yawRate*finArm) - dette ER både retningsstabiliteten (Cnβ, via sideslipDeg sitt bidrag til
        // localAirVel.x) OG gir-dempingen (Cnr, via selve rotasjonsbidraget her) fra ÉN og samme geometri.
        const vx = localAirVel.x + yawRateAssumed * finArm;
        // Propwash - se merknaden ved tailTorqueAtPitchRate (samme mekanisme, samme moment-arm/skrogstasjon).
        const vz = localAirVel.z - propwashDeltaV;
        const speedSq = vx * vx + vz * vz;
        const baseSlip = speedSq > 0.09 ? THREE.MathUtils.radToDeg(Math.atan2(vx, -vz)) : 0;
        // Sideror (RUDDER_MAX_AOA_DEG) endrer finnens EFFEKTIVE angrepsvinkel - pilotens direkte utslag.
        const aoa = clamp(baseSlip + yawDeflection * RUDDER_MAX_AOA_DEG, -35, 35);
        const sideForce = 0.5 * AIR_DENSITY * speedSq * finArea * (finClSlope * aoa);
        // MINUS her (i motsetning til tail-torque over) - matcher det etablerte, allerede riktige
        // fortegnet for retningsstabilitet (positiv sideslip skal gi et RETTENDE, ikke forsterkende, gir-moment).
        return -finArm * sideForce;
    }

    // Lineariserer rundt GJELDENDE rate (ikke rundt 0) - viktig ved store rater (f.eks. en etablert spinn)
    // hvor modellen er dypt inne i ikke-lineære soner (AoA-klemming, steilekurvens knekkpunkt): å alltid
    // linearisere ved rate=0 lot hele det ikke-lineære bidraget mellom 0 og faktisk rate forsvinne, som ga
    // kvalitativt feil oppførsel (ikke bare unøyaktig) ved store rater. F0 rekonstrueres her slik at
    // eksplisitt-steg + eksponentiell nedgang reproduserer nøyaktig riktig moment for gjeldende tidssteg,
    // samtidig som det forblir ubetinget stabilt for enhver k*dt/treghet.
    const RATE_DERIV_EPS = 0.02;
    function linearizeDamping(torqueFn, currentRate) {
        const torqueNow = torqueFn(currentRate);
        const k = Math.max(0, -(torqueFn(currentRate + RATE_DERIV_EPS) - torqueNow) / RATE_DERIV_EPS);
        return { f0: torqueNow + k * currentRate, k: k };
    }

    const rollWingLin = linearizeDamping(function (r) { return wingCoupledTorques(r, yawRate).roll; }, rollRate);
    const rollWingF0 = rollWingLin.f0;
    const rollWingDampCoeff = rollWingLin.k;
    const yawWingLin = linearizeDamping(function (r) { return wingCoupledTorques(rollRate, r).yaw; }, yawRate);
    const yawWingF0 = yawWingLin.f0;
    const yawWingDampCoeff = yawWingLin.k;

    // Ikke-dempings-momenter (kontroll + løft-/sideslip-/dragdifferensial-bidrag) integreres eksplisitt, som før.
    const rollTorqueNoDamp = -rollDeflection * qDynControl * ROLL_CONTROL_EFFECTIVENESS + rollWingF0 + rollTorqueFromDihedral;

    const pitchLin = linearizeDamping(tailTorqueAtPitchRate, planeState.angularVelocity.pitch);
    const pitchTorqueF0 = pitchLin.f0;
    const pitchDampCoeff = pitchLin.k;

    const finLin = linearizeDamping(finTorqueAtYawRate, yawRate);
    let yawTorqueF0 = finLin.f0 + yawWingF0;
    // Hjul-/understellsfriksjon mot bakken motstår også VINDKANTRING (weathervaning) mens flyet er på
    // bakken - ikke bare lateral gliding (se lengre ned i resolveGroundContact) eller gir-RATE-demping
    // (GROUND_YAW_FRICTION, som kun bremser en rotasjon som allerede er i gang). Uten en STATISK
    // (Coulomb-lignende) motstand her ville selv et lite sideprodukt fra vind rotere flyet fritt mens det
    // står stille - i et ekte fly holder dekkenes grep imot helt til vindkraften overstiger en terskel
    // proporsjonal med normalkraften (vekten), først da "glipper" den og flyet kantrer i vinden.
    if (planeState.onGround) {
        const maxGroundYawTorque = GROUND_YAW_FRICTION_TORQUE_COEFF * spec.mass * GRAVITY;
        if (Math.abs(yawTorqueF0) <= maxGroundYawTorque) {
            yawTorqueF0 = 0;
        } else {
            yawTorqueF0 -= Math.sign(yawTorqueF0) * maxGroundYawTorque;
        }
    }
    const finDampCoeff = finLin.k;
    // Total gir-demping = finnens EGEN (Cnr) + vinge-differensialens (over) - to reelle, uavhengige
    // dempende bidrag til SAMME akse, lagt sammen. YAW_DAMPER_GAIN booster dette: uten den er Liten sitt
    // dempingsforhold ζ≈0.29 ved 15 m/s (kritisk dempet er ζ=1) - en reell, underdempet "Dutch roll"-
    // modus (mange ekte fly trenger en faktisk yaw-damper-boks av samme grunn) - boostet til ζ≈0.87.
    const yawDampCoeff = (finDampCoeff + yawWingDampCoeff) * YAW_DAMPER_GAIN;

    planeState.angularVelocity.roll += (rollTorqueNoDamp / spec.inertiaRoll) * dt;
    planeState.angularVelocity.pitch += (pitchTorqueF0 / spec.inertiaPitch) * dt;
    planeState.angularVelocity.yaw += (yawTorqueF0 / spec.inertiaYaw) * dt;

    // Aerodynamisk demping (roll/pitch/yaw) integreres IMPLISITT (eksponentiell nedgang: v *= e^(-k*dt))
    // i stedet for eksplisitt Euler (v += -k*v*dt). Ved høy fart (qDyn stor, spesielt kombinert med lav
    // treghet) blir k*dt/treghet > 1 - eksplisitt Euler overskyter da null og SNUR fortegn hver eneste
    // fysikk-tick, som viser seg som en synlig hakking/jitter fram og tilbake ved høy fart. Eksponentiell
    // demping er ubetinget stabil uansett hvor stort k*dt/treghet blir, og gir SAMME steady-state-
    // oppførsel som før. Roll sin totale k er nå BÅDE den opprinnelige flate ROLL_DAMPING-konstanten OG
    // vinge-differensialens egen rate-avhengige del (rollWingDampCoeff) - se merknaden over.
    const rollDampDecay = Math.exp(-((qDynControl * ROLL_DAMPING + rollWingDampCoeff) / spec.inertiaRoll) * dt);
    const pitchDampDecay = Math.exp(-(pitchDampCoeff / spec.inertiaPitch) * dt);
    const yawDampDecay = Math.exp(-(yawDampCoeff / spec.inertiaYaw) * dt);
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
// Gjenbrukte "scratch"-objekter for checkTailStrike/resolveGroundContact (kalles hver fysikk-tick, 120
// ganger/sekund, uansett om flyet er på bakken eller ikke for denne ene) - se merknaden ved
// _groundContactLocalPts i resolveGroundContact for hvorfor: unngår å opprette nye Vector3/Euler/
// Quaternion-objekter hver eneste tick (som ellers gir jevn "søppel"-produksjon og kan vise seg som
// periodiske GC-pauser/hakking i lange flighter), ved å MUTERE de samme objektene på plass i stedet.
const _tailStrikeScratch = new THREE.Vector3();
function checkTailStrike(spec) {
    // Halepunktet er den FAKTISKE halekjegle-tuppen (samme delte konstanter som buildPlane, se merknaden
    // øverst i filen) - ikke en uavhengig tilnærming som kan drifte ut av synk med skroget.
    const cabinLen = FUSELAGE_LENGTH_BUILD * CABIN_LEN_RATIO;
    const tailLen = FUSELAGE_LENGTH_BUILD * TAIL_LEN_RATIO;
    const tailTipZ = (cabinLen / 2 + tailLen) * spec.visualScale;
    const tailTipRadius = CABIN_RADIUS_BUILD * TAIL_TIP_RADIUS_RATIO * spec.visualScale;
    const tailWorld = _tailStrikeScratch.set(0, -tailTipRadius, tailTipZ)
        .applyQuaternion(planeState.quaternion).add(planeState.position);
    if (tailWorld.y <= 0.01) {
        if (!tailStruckCurrently) {
            tailstrikeWarningUntil = performance.now() + 2500;
        }
        tailStruckCurrently = true;
    } else if (tailWorld.y > GROUND_CLEARANCE_FW * 3) {
        tailStruckCurrently = false;
    }
}

// Gjenbrukte "scratch"-objekter for resolveGroundContact - samme begrunnelse som _tailStrikeScratch over.
const _groundContactLocalPts = [new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()];
const _groundContactWorldPts = [new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()];
const _groundContactEuler = new THREE.Euler();
const _groundContactQuat = new THREE.Quaternion();

// Ett understellspunkt per side (hovedhjul) - flat rullebane, ingen kant-/veggkollisjon nødvendig her
// (i motsetning til quad-banen). Hard landing (høy synkefart eller stor krengevinkel ved berøring) gir
// "krasjet"-tilstand; ellers demper kontakten synk/krengning og bremser med rullemotstand.
function resolveGroundContact(dt) {
    const spec = currentPlaneSpec();
    const gearTrack = spec.wingSpan * 0.25;
    // Nesehjulet er med i SAMME liste som hovedhjulene (ikke en separat, senere korreksjon lenger) - to
    // uavhengige posisjonskorreksjoner i samme tick (én for hovedhjul, én for nesehjul) kunne motarbeide
    // hverandre (den ene løftet flyet, den andre senket det rett etterpå, avhengig av hvilket punkt som
    // "vant") - det ga en synlig glitching/tipping selv under rolig, jevn taksing. Én kombinert
    // gjennomtrengnings-sjekk over alle tre punktene unngår denne konflikten helt.
    _groundContactLocalPts[0].set(gearTrack / 2, spec.gearOffsetY, 0.05);
    _groundContactLocalPts[1].set(-gearTrack / 2, spec.gearOffsetY, 0.05);
    _groundContactLocalPts[2].set(0, spec.gearOffsetY, -spec.wingSpan * 0.3);
    let maxPenetration = -Infinity;
    for (let i = 0; i < 3; i++) {
        _groundContactWorldPts[i].copy(_groundContactLocalPts[i])
            .applyQuaternion(planeState.quaternion).add(planeState.position);
        maxPenetration = Math.max(maxPenetration, -_groundContactWorldPts[i].y);
    }

    // Sjekker mot 0 (faktisk gjennomtrengning), ikke mot -GROUND_CLEARANCE_FW - sistnevnte ville behandlet
    // hele klaringssonen som "på bakken" og trukket flyet ned igjen selv når hjulene allerede var et par
    // cm klar, som gjorde en helt normal, gradvis lettelse under selve avgangsrullingen unaturlig vanskelig.
    if (maxPenetration <= 0) {
        planeState.onGround = false;
        planeState.hasBeenAirborne = true;
        return;
    }

    if (!planeState.onGround) {
        _groundContactEuler.setFromQuaternion(planeState.quaternion, "YXZ");
        const bankDeg = Math.abs(-THREE.MathUtils.radToDeg(_groundContactEuler.z));
        if (planeState.velocity.y < -CRASH_SINK_RATE || bankDeg > CRASH_BANK_DEG) {
            planeState.crashed = true;
        }
    }
    planeState.onGround = true;
    planeState.position.y += maxPenetration;
    if (planeState.velocity.y < 0) planeState.velocity.y *= 0.15;

    // Svak, kontinuerlig slerp mot null krengning mens flyet er på bakken - forhindrer at et parkert fly
    // blir stående og krenge urealistisk, uten å overstyre pilotens eget skråror-utslag under rulling
    // (en tidligere, mye sterkere versjon av denne - 4*dt i stedet for 0.3*dt, pluss en direkte halvering
    // av vinkelhastigheten hver tick - fightet aktivt mot skråror under hele takeoff-rullingen og ga
    // både treg og oscillerende rull).
    _groundContactEuler.setFromQuaternion(planeState.quaternion, "YXZ");
    _groundContactEuler.set(_groundContactEuler.x, _groundContactEuler.y, 0, "YXZ");
    _groundContactQuat.setFromEuler(_groundContactEuler);
    planeState.quaternion.slerp(_groundContactQuat, Math.min(1, 0.3 * dt));
    // Hjulenes rotasjonsfriksjon mot underlaget - uten denne ville en gir-rotasjon fortsette for evig i
    // stillstand med motoren av, siden det aerodynamiske dempeleddet (som skalerer med fart²) blir null
    // når flyet står stille. Eksponentiell, tidssteg-uavhengig demping (samme prinsipp som rollDampDecay/
    // yawDampDecay i stepPhysics) - en ren "*=0.85 per tick" ville tilsvart 0.85^120≈3e-9 PER SEKUND ved
    // dette faste 120Hz-tidssteget, altså en nesten momentan stopp i stedet for en gradvis oppbremsing.
    planeState.angularVelocity.yaw *= Math.exp(-GROUND_YAW_FRICTION * dt);
    // (Nesehjulets "kan ikke tippe frem"-fysikk er nå del av maxPenetration-punktene over, ikke en egen
    // etterfølgende korreksjon her - se merknaden ved localPoints.)

    const forwardWorld = new THREE.Vector3(0, 0, -1).applyQuaternion(planeState.quaternion);
    const forwardSpeed = planeState.velocity.dot(forwardWorld);
    if (forwardSpeed > 0.05) {
        // Hjulbrems: piloten bruker normalt hjulbremser rett etter landing (gass i tomgang) i tillegg til
        // ren rullemotstand - uten dette rullet flyet urealistisk lenge etter landing/taxi med gass av.
        const brakeDecel = inputState.stick.throttle < 0.02 ? BRAKE_DECELERATION : 0;
        planeState.velocity.addScaledVector(forwardWorld, -(ROLLING_FRICTION * GRAVITY + brakeDecel) * dt);
    }
    const rightWorld = new THREE.Vector3(1, 0, 0).applyQuaternion(planeState.quaternion);
    const lateralSpeed = planeState.velocity.dot(rightWorld);
    planeState.velocity.addScaledVector(rightWorld, -lateralSpeed * Math.min(1, 8 * dt));
}

/* ---------- Fly-kontroller (reset/motor/kamera) ---------- */
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
    planeState.hasBeenAirborne = false;
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
const trimInputEl = document.getElementById("trimInput");
const trimValueEl = document.getElementById("trimValue");

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

    if (trimInputEl && document.activeElement !== trimInputEl) {
        trimInputEl.value = planeState.elevatorTrimDeg;
    }
    if (trimValueEl) trimValueEl.textContent = trimText;
}

/* ---------- Paneler (rates / fly-kamera / vind / gamepad / hjelp) ---------- */
// Sim.togglePanel lukker selv alt annet meny-UI (andre paneler OG en åpen Settings-meny, se
// closeAllMenus i simulator-common.js) - ingen egen panel-ID-liste å vedlikeholde her.
function togglePanel(panel) {
    Sim.togglePanel(panel);
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

const gamepadPanelEl = document.getElementById("gamepadPanel");
const gamepadAxesReadoutEl = document.getElementById("gamepadAxesReadout");
function updateGamepadAxesReadout(gp) {
    if (gamepadPanelEl.style.display === "none") return;
    const activeGp = gp || getActiveGamepad();
    Sim.updateGamepadAxesReadout(gamepadAxesReadoutEl, activeGp, Sim.MIN_GAMEPAD_CHANNELS);
}

function setGamepadButtonVisible(visible) {
    document.getElementById("toggleGamepadBtn").style.display = visible ? "" : "none";
    if (!visible) document.getElementById("gamepadPanel").style.display = "none";
}

/* ---------- FPV HUD/OSD (crosshair / kunstig horisont) ---------- */
let fpvHudCtx = null;
let fpvHudModeIndex = 0;

const fpvHudCanvasEl = document.getElementById("fpvHudCanvas");
function initFpvHudCanvas() {
    fpvHudCanvasEl.width = 400;
    fpvHudCanvasEl.height = 300;
    fpvHudCtx = fpvHudCanvasEl.getContext("2d");
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
    const mode = FPV_HUD_MODES[fpvHudModeIndex];
    if (activeCamera !== fpvCamera || mode === "none") {
        fpvHudCanvasEl.style.display = "none";
        return;
    }
    fpvHudCanvasEl.style.display = "block";
    const w = fpvHudCanvasEl.width, h = fpvHudCanvasEl.height;
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
    viewportWatcher();

    updateWind(frameDt);
    updateInput(frameDt);
    while (accumulator >= FIXED_DT) {
        stepPhysics(FIXED_DT);
        accumulator -= FIXED_DT;
    }

    updatePlaneVisual(frameDt);
    chaseCameraController.update(frameDt, planeState.position, planeState.quaternion);
    updateVlosCamera();
    updateWindsockVisual(now);
    treeSwayManager.update(now, currentWindVector);
    updateWindLeaves(frameDt, now);
    updateWindSmoke(frameDt);
    updateClockTowers();
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
    Sim.setupDropdown(document.getElementById("settingsToggleBtn"), settingsMenuEl);
    Sim.wirePanelCloseButtons(settingsMenuEl);

    // togglePanel (se Sim.togglePanel/closeAllMenus i simulator-common.js) lukker selv Settings-menyen som
    // en del av "kun ÉN meny av gangen" - ingen eget closeSettingsMenu()-kall trengs lenger her.
    document.getElementById("toggleRatesBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("ratesPanel"));
    });
    document.getElementById("toggleFlyCameraBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("flyCameraPanel"));
    });
    document.getElementById("toggleWindBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("windPanel"));
    });
    document.getElementById("toggleGamepadBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("gamepadPanel"));
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
    windSpeedValue.textContent = settings.wind.speed + " m/s";
    windDirectionInput.value = settings.wind.directionDeg;
    windDirectionValue.textContent = settings.wind.directionDeg + "°";
    windGustInput.value = settings.wind.gust;
    windGustValue.textContent = Math.round(settings.wind.gust * 100) + "%";
    windEnabledInput.addEventListener("change", function () {
        settings.wind.enabled = windEnabledInput.checked;
        saveSettings();
    });
    windSpeedInput.addEventListener("input", function () {
        settings.wind.speed = parseFloat(windSpeedInput.value);
        windSpeedValue.textContent = windSpeedInput.value + " m/s";
        saveSettings();
    });
    windDirectionInput.addEventListener("input", function () {
        settings.wind.directionDeg = parseFloat(windDirectionInput.value);
        windDirectionValue.textContent = windDirectionInput.value + "°";
        saveSettings();
    });
    windGustInput.addEventListener("input", function () {
        settings.wind.gust = parseFloat(windGustInput.value);
        windGustValue.textContent = Math.round(settings.wind.gust * 100) + "%";
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
