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
// Luftmotstand i to ledd per klasse: dragLinear dominerer i lav fart (indusert drag/momentum-drag fra
// propellstrømmen - det som faktisk bremser en quad som "seiler" sakte avgårde etter en dytt), dragQuad
// dominerer i høy fart (v² - "veggen" nær toppfart). Kun kvadratisk ledd ga null bremsing i lav fart
// og en drone som fløt evig videre etter et lite puff.
const DRONE_CLASSES = {
    racing: {
        label: "Racing (rask, lett)",
        mass: 0.5, maxThrust: 18,
        inertiaRollPitch: 0.025, inertiaYaw: 0.05,
        dragLinear: 0.2, dragQuad: 0.006, visualScale: 1.0
    },
    mid: {
        label: "Middels",
        mass: 1.2, maxThrust: 24,
        inertiaRollPitch: 0.07, inertiaYaw: 0.14,
        dragLinear: 0.35, dragQuad: 0.02, visualScale: 1.3
    },
    cinematic: {
        label: "Cinematic (stor, treg)",
        mass: 2.6, maxThrust: 35,
        inertiaRollPitch: 0.16, inertiaYaw: 0.32,
        dragLinear: 0.55, dragQuad: 0.07, visualScale: 2.3
    }
};
const DEFAULT_DRONE_CLASS = "racing";

/* ---------- Motor-mikser ----------
   Fire virtuelle motorer i X-oppsett i stedet for å sette gass/rull/pitch/yaw uavhengig av hverandre.
   Hver motor_i = grunngass + MIX_AUTHORITY*(rullkommando*rullfortegn_i + pitchkommando*pitchfortegn_i +
   yawkommando*yawfortegn_i), klemt til [gulv, 1]. Total trekkraft og faktisk oppnådde rull/pitch/yaw-
   momenter beregnes deretter FRA de klemte motorverdiene (se mixMotors/extractMixedAxis), ikke fra de
   ønskede verdiene direkte - det er dette som gir ekte kobling mellom gass og kontrollautoritet:
   nær 0% gass er det nesten ikke plass til å senke "motsatt" motorpar (mindre rull/pitch/yaw-autoritet),
   og nær 100% gass er det nesten ikke plass til å øke dem (harde manøvre stjeler trekkraft).
   Indeksrekkefølgen er fysisk: 0=fremre-høyre, 1=fremre-venstre, 2=bakre-høyre, 3=bakre-venstre -
   samme rekkefølge som motorOffsets i buildDrone() og getLegTopLocalPositions(), slik at propellskade
   (se propDamage) på motor i rammer riktig hjørne. Fortegnene er verifisert mot velte-konvensjonen i
   resolveGroundContact (negativ pitch-vinkelfart = tipper forover, negativ roll = tipper mot høyre):
   mister motor i trekkraft, gir ekstraksjonen et moment som tipper droneen mot nettopp det hjørnet.
   Yaw-fortegnene matcher spinnretningene (dir) i buildDrone - diagonale motorpar spinner samme vei.
*/
const MOTOR_MIX = [
    { roll: +1, pitch: +1, yaw: +1 }, // fremre-høyre
    { roll: -1, pitch: +1, yaw: -1 }, // fremre-venstre
    { roll: +1, pitch: -1, yaw: -1 }, // bakre-høyre
    { roll: -1, pitch: -1, yaw: +1 }  // bakre-venstre
];
// Hvor stor andel av hver motors kommandoområde rull/pitch/yaw-differensialen får bruke - resten av
// "budsjettet" er grunngassen. Kommandoene inn er allerede normalisert til [-1, 1] via axisTorqueNorm()
// under, så denne metter i praksis kun ved gass-ytterpunktene, ikke ved vanlige stick-utslag midt i
// gassområdet.
const MIX_AUTHORITY = 0.5;
// Verste normale tilfelle (full rate i én retning som momentant hopper til full rate motsatt vei,
// f.eks. et hurtig flikk fra fullt utslag venstre til fullt utslag høyre) gir en rate-feil på inntil
// ~2*maxRate - se axisTorqueNorm(), som normaliserer "ønsket moment" til en rull/pitch/yaw-kommando i
// [-1, 1] relativt til NÅVÆRENDE rate-innstilling (maxRate er brukerjusterbar, 100-1200 grader/s).
// TORQUE_CMD_HEADROOM gir litt margin over dette slik at normaliseringen i praksis er transparent
// (lik gammel oppførsel) for vanlig pinnebruk uansett rate-innstilling - kun mikserens motor-metning
// (se mixMotors) skal gi den følbare autoritetsreduksjonen ved gass-ytterpunktene.
const TORQUE_CMD_HEADROOM = 1.3;
function axisTorqueNorm(axisRateSettings) {
    return TORQUE_GAIN * THREE.MathUtils.degToRad(2 * axisRateSettings.maxRate) * TORQUE_CMD_HEADROOM;
}
// Airmode er ikke en egen flygemodus, men en egenskap ved stabiliserings-/rate-looppene: uten airmode
// er stabiliseringen i praksis borte på tomgangs-gass (motorene ligger på idle og kan ikke senkes -
// det finnes ingen differensial å ta av, så et gass-kutt midt i en manøver gir en ustabilisert drone).
// Med airmode beholdes full stabilisering på null gass. Mekanismen her er Betaflight-varianten:
// "Airmode av" (standard): motorene klemmes individuelt til [0, 1] - differensialen (og dermed
// stabiliseringen) spises opp nær gass-ytterpunktene.
// "Airmode på": hele motorsettet løftes samlet slik at den laveste motoren akkurat når AIRMODE_MIN_IDLE
// før klemming - differensialen (og kontrollautoriteten) bevares uavhengig av gassnivå, på bekostning
// av at total trekkraft kan bli høyere enn det gassen isolert sett tilsier.
const AIRMODE_MIN_IDLE = 0.05;

/* ---------- Vortex ring state (VRS) ----------
   Rask vertikal nedstigning med lav horisontalfart lar propellene synke ned i sin egen nedvask og
   miste effektiv trekkraft - mer gass hjelper ikke, kun å fly sideveis ut av det. Modellert som en
   gradvis trekkraft-reduksjon basert på synkefart, dempet av horisontalfart (å fly ut av det stopper
   effekten raskt).
*/
const VRS_DESCENT_ONSET = 2.0;       // m/s synkefart før noen VRS-effekt starter
const VRS_DESCENT_FULL = 5.0;        // m/s synkefart der effekten er helt mettet
const VRS_MAX_THRUST_LOSS = 0.45;    // maks andel av trekkraften som kan mistes
const VRS_HORIZ_SPEED_ESCAPE = 3.0;  // m/s horisontalfart som "flyr deg ut av" VRS

/* ---------- Propellskade ----------
   En propell som treffer bakken, en vegg eller en racing-port blir skadet og motoren mister trekkraft
   tilsvarende (propDamage 0..1 per motor, multiplisert inn på mikserens motorverdi - det gir automatisk
   både trekkraft-tap OG et skjevt moment som tipper droneen mot det ødelagte hjørnet). Skadegraden
   avhenger av treffarten: et hardt treff (>= PROP_DESTROY_SPEED) ødelegger propellen helt, et lett
   "så vidt borti" gir minst PROP_MIN_STRIKE_DAMAGE - flere lette berøringer akkumulerer. En spinnende
   propell som blir liggende an mot noe (f.eks. armert drone opp ned på bakken) slipes i tillegg
   gradvis ned (PROP_GRIND_RATE). Skaden vises på modellen (avbrukne blad + skjev/bøyd propell) og
   repareres kun ved reset (R).
*/
const PROP_DESTROY_SPEED = 8;        // m/s treff-fart som ødelegger propellen fullstendig i ett treff
const PROP_MIN_STRIKE_DAMAGE = 0.25; // minste skade per berøring - fire "så vidt borti" ødelegger propellen
const PROP_GRIND_RATE = 0.4;         // skade per sekund mens en spinnende propell ligger an mot noe
const PROP_BROKEN_STUB_SCALE = 0.3;  // lengde-andel som står igjen av et avbrukket propellblad
const PROP_GROUND_STRIKE_EPS = 0.02; // m - hvor nær en flate et propellpunkt må være for å regnes som treff

/* ---------- Bakkekontakt: velting og redusert kontroll ----------
   En drone som ligger på siden eller opp ned skal IKKE kunne "stikke-rulles" opp igjen - propellene
   jobber rett mot bakken og har nesten ingen effektiv momentarm. Kontrollmyndigheten reduseres derfor
   kraftig når droneen er i bakkekontakt og kraftig tippet (kun da - stor tilt i fri flukt er normal
   acro-flyging). I tillegg ruller tyngdekraften en tippet drone videre ned til nærmeste stabile side
   (flatt riktig vei eller flatt på ryggen - balansepunktet er på høykant, 90°), som en flat plate som
   velter over kanten sin, i stedet for at den blir stående og balansere på en armtupp.
*/
const GROUNDED_TIPPED_AUTHORITY = 0.08; // andel kontrollmyndighet igjen når den ligger veltet på bakken
const GROUNDED_AUTHORITY_TILT_START = Math.cos(THREE.MathUtils.degToRad(30)); // full kontroll inntil 30° tilt
const GROUNDED_AUTHORITY_TILT_END = Math.cos(THREE.MathUtils.degToRad(70));   // minimum fra 70° tilt
const GROUND_SETTLE_TORQUE = 15;     // rad/s^2 ved 90° tilt - tyngdekraft-velting ned mot flat
const EDGE_PIVOT_MAX_PENETRATION = 0.03; // m - over dette er det en ekte kollisjon, ikke kant-vipping
// Skraping: å dra understellet langs bakken i fart skal IKKE være en stabil flygestil - bena hekter
// seg i underlaget og drar nesa over. Over GROUND_SCRAPE_SPEED slås opprettingen på bakken av,
// snuble-momentet (GROUND_TIP_TORQUE_GAIN) og friksjonen får virke uimotsagt, og kontrollmyndigheten
// reduseres gradvis - ender med velt. Veltet forbi OVERTURN_CRASH_TILT_DEG på bakken regnes som krasj.
const GROUND_SCRAPE_SPEED = 1.5;     // m/s horisontalfart i bakkekontakt før det regnes som skraping
const OVERTURN_CRASH_TILT_DEG = 100; // tilt på bakken forbi dette = veltet = krasj (disarm + reset)

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

// VLOS-piloten står her (samme punkt som VLOS-kameraet, se initScene) - treffer droneen personen er
// det ikke et vanlig krasj, men en personskade med eget varsel (se updatePilotCollision/injuryBanner).
const PILOT_POSITION = new THREE.Vector3(0, 0, 5);
const PILOT_HIT_RADIUS = 0.45; // m - kroppssylinder rundt piloten
const PILOT_HEIGHT = 1.85;     // m
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
    airmodeEnabled: false,
    inputSource: "auto", // "auto" | "keyboard" | gamepad-indeks som streng ("0", "1", ...)
    wind: DEFAULT_WIND,
    cloudsEnabled: true,
    cloudCoverage: 0.6, // andel av skyklyngene som vises (0-1), se buildClouds/updateClouds
    fpvHudMode: "crosshair" // "crosshair" | "horizon" | "none"
};

const FPV_HUD_MODES = ["crosshair", "horizon", "none"];
const FPV_HUD_MODE_LABELS = { crosshair: "Crosshair", horizon: "Kunstig horisont", none: "Ingen" };

/* ---------- Øvelser: data/konstanter (se egen "Øvelser: kjøretøy for øvelser"-seksjon lenger ned
   for selve kjørelogikken) ---------- */
// v2: øvelsene delt opp i ni separate øvelser (hover, firkant, høydeforandring, sirkel, åttetall,
// vind) - gammel v1-lagring (én samleøvelse) er ikke kompatibel og skal ikke arves.
const EXERCISE_STORAGE_KEY = "ffi-uas:simulator-exercises-v2";

// Nær og lavt i forhold til VLOS-piloten på (0,1.6,5), slik at figurene er lette å se med det blotte
// øye. Flyttet nærmere (13 -> 11 m) etter tilbakemelding om at dybde/bane var vanskelig å bedømme på
// avstand fra VLOS - gjelder firkant, høydeforandring, sirkel og åttetall (delt senter for alle fire).
const EXERCISE_CENTER = new THREE.Vector3(0, 0, -6);
// Firkant/sirkel/åttetall senket (3 -> 2 m) - lavere gir bedre dybdereferanse mot bakken fra VLOS.
// Hover beholder sin egen, høyere HOVER_ALTITUDE - brukeren ba kun om de tre løype-øvelsene.
const EXERCISE_ALTITUDE = 1;
const HOVER_ALTITUDE = 2.5;
const ALTITUDE_TOLERANCE = 2;
const SQUARE_HALF_SIDE = 5;
const CIRCLE_RADIUS = 5;
const CIRCLE_SEGMENTS = 16;
const EIGHT_HALF_WIDTH = 8;      // åttetallets halve bredde (på tvers foran piloten)
const EIGHT_DEPTH = 7;           // dybde-amplitude på lemniskaten (før *0.5 fra sin*cos)
const EIGHT_SEGMENTS = 24;
const ZIGZAG_HALF_WIDTH = 5;     // høydeforandrings-mønsterets halve bredde
const ZIGZAG_LOW_Y = 2;
const ZIGZAG_HIGH_Y = 6.5;
const WAYPOINT_CAPTURE_RADIUS = 2.5;
// Sikksakken (3D-fangst) trenger strammere radius: benene er bare ~4,5 m lange, og med 2,5 m radius
// ble topp-/bunnpunktene "tatt" lenge før dronen faktisk var fremme.
const WAYPOINT_CAPTURE_RADIUS_3D = 1.4;
// Utvidet fra 20 - VLOS-dybdeoppfatning på avstand gjør det vanskelig å bedømme nøyaktig når man er
// "innenfor" et strengt vindu, og nese-frem-sjekken (fartsretning vs. nese) er i sin natur støyete.
const HEADING_TOLERANCE_DEG = 28;
const REQUIRED_CLEAN_LAPS = 3;
const REQUIRED_RETURN_REPS = 3; // "Returner hjem" må gjennomføres 3 ganger - totaltiden for alle tre lagres
const HOVER_HOLD_SEC = 10;
const HOVER_POS_TOLERANCE = 2.5; // m horisontal radius rundt hover-punktet
// Hover-øvelsen har sitt eget punkt, nærmere piloten enn løype-figurene (10 m unna i stedet for 11) -
// i ro på kort hold er det lettere å lese nese-retningen på droneen.
const HOVER_CENTER = new THREE.Vector3(0, 0, -5);
// Landingsplassen (H) ved avgangspunktet - øvelsene avsluttes automatisk ved landing her.
const LANDING_PAD_RADIUS = 2.4;
// (Løype-øvelsene aktiverer reglene først når første veipunkt er nådd - se updateExercise - så
// transportetappen fra avgangsplassen og ut til figuren er alltid fri flyging.)
// Nesa "ut" er en fast retning bort fra VLOS-piloten (0,1.6,5) - siden både piloten og øvelses-
// figurene ligger på Z-aksen, faller dette sammen med verdens -Z (samme retning droneen alt har
// rett etter resetDrone(), siden identitets-quaternionen tilsvarer gir 0 i yaw).
// Hover-øvelsens fire retninger bruker samme yaw-konvensjon (targetYaw = atan2(-dx,-dz)):
// venstre (nese mot verdens -X) = +90°, høyre (+X) = -90°, inn (mot piloten, +Z) = 180°.
const LOCKED_HEADING = 0;
const HEADING_LEFT = Math.PI / 2;
const HEADING_RIGHT = -Math.PI / 2;
const HEADING_IN = Math.PI;
// Under denne farten sjekkes ikke nese-mot-fartsretning: dels er retningen støyete, dels er lav fart
// nettopp hjørne-/korreksjons-manøveren i "nese frem" (bremse opp, yawe, rygge/justere, akselerere ut) -
// hevet til 2.5 fordi selv 1.5 fortsatt utløste avvik under små hastighets-korreksjoner nær et veipunkt.
const MIN_SPEED_FOR_HEADING_CHECK = 2.5;

function buildSquareWaypoints(center, halfSide) {
    return [
        { x: center.x - halfSide, z: center.z - halfSide },
        { x: center.x + halfSide, z: center.z - halfSide },
        { x: center.x + halfSide, z: center.z + halfSide },
        { x: center.x - halfSide, z: center.z + halfSide }
    ];
}
function buildCircleWaypoints(center, radius, segments) {
    const points = [];
    for (let i = 0; i < segments; i++) {
        const a = (i / segments) * Math.PI * 2;
        points.push({ x: center.x + Math.sin(a) * radius, z: center.z + Math.cos(a) * radius });
    }
    return points;
}
// Åttetall (Gerono-lemniskate) liggende på tvers foran piloten - løkkene til venstre og høyre,
// kryssingen midt foran. x = sin(t)*halvbredde, z = sin(t)*cos(t)*dybde.
function buildFigureEightWaypoints(center, halfWidth, depth, segments) {
    const points = [];
    for (let i = 0; i < segments; i++) {
        const t = (i / segments) * Math.PI * 2;
        points.push({ x: center.x + Math.sin(t) * halfWidth, z: center.z + Math.sin(t) * Math.cos(t) * depth });
    }
    return points;
}
// Høydeforandrings-mønster i et vertikalt plan på tvers foran piloten (veipunkter MED y-koordinat):
// A_lav -> A_høy (loddrett opp) -> B_lav (skrått ned til siden) -> B_høy (loddrett opp) -> tilbake
// til A_lav (skrått ned) via løkke-wrappingen. Reversert rekkefølge gir "loddrett ned, skrått opp".
function buildVerticalZigzagWaypoints(center, halfWidth, lowY, highY) {
    return [
        { x: center.x - halfWidth, y: lowY, z: center.z },
        { x: center.x - halfWidth, y: highY, z: center.z },
        { x: center.x + halfWidth, y: lowY, z: center.z },
        { x: center.x + halfWidth, y: highY, z: center.z }
    ];
}
const SQUARE_WAYPOINTS = buildSquareWaypoints(EXERCISE_CENTER, SQUARE_HALF_SIDE);
const CIRCLE_WAYPOINTS = buildCircleWaypoints(EXERCISE_CENTER, CIRCLE_RADIUS, CIRCLE_SEGMENTS);
const EIGHT_WAYPOINTS = buildFigureEightWaypoints(EXERCISE_CENTER, EIGHT_HALF_WIDTH, EIGHT_DEPTH, EIGHT_SEGMENTS);
const ZIGZAG_WAYPOINTS = buildVerticalZigzagWaypoints(EXERCISE_CENTER, ZIGZAG_HALF_WIDTH, ZIGZAG_LOW_Y, ZIGZAG_HIGH_Y);
const ZIGZAG_WAYPOINTS_REVERSED = ZIGZAG_WAYPOINTS.slice().reverse();

// Felles beskrivelsestekst-suffiks: reglene er like for alle øvelsene.
const EXERCISE_RULES_TEXT = " Første avvik gir bare en advarsel; skjer det igjen nullstilles steget " +
    "og dronen settes tilbake på avgangsplassen (klokka går videre). Droneklassen settes automatisk " +
    "til Middels, og du flyr fra VLOS-posisjonen. R restarter hele øvelsen med nullstilt klokke.";

const EXERCISES = {
    ex1: {
        id: "ex1",
        icon: "fa-crosshairs",
        label: "1. Hover",
        shortDescription: "Hold posisjonen med nesa ut, til sidene og inn - " + HOVER_HOLD_SEC + " s per retning.",
        fullDescription: "Hold dronen i ro over det markerte punktet på " + HOVER_ALTITUDE + " m høyde - " +
            "først med nesa bort fra deg, så mot venstre, så mot høyre, og til slutt med nesa mot deg. " +
            "Hver retning må holdes i minst " + HOVER_HOLD_SEC + " sekunder innenfor omtrent samme posisjon " +
            "og høyde - driver du ut av området eller mister retningen, nullstilles tiden for gjeldende " +
            "retning. Pila på bakken viser retningen nesa skal peke. Droneklassen settes automatisk til " +
            "Middels, og du flyr fra VLOS-posisjonen. R restarter hele øvelsen med nullstilt klokke.",
        stages: [
            { id: "hover-out", label: "Hover - nese ut", type: "hover", headingYaw: LOCKED_HEADING, holdSec: HOVER_HOLD_SEC },
            { id: "hover-left", label: "Hover - nese mot venstre", type: "hover", headingYaw: HEADING_LEFT, holdSec: HOVER_HOLD_SEC },
            { id: "hover-right", label: "Hover - nese mot høyre", type: "hover", headingYaw: HEADING_RIGHT, holdSec: HOVER_HOLD_SEC },
            { id: "hover-in", label: "Hover - nese inn", type: "hover", headingYaw: HEADING_IN, holdSec: HOVER_HOLD_SEC }
        ]
    },
    ex2: {
        id: "ex2",
        icon: "fa-vector-square",
        label: "2. Firkant - nese ut",
        shortDescription: "Fly firkanten med nesa fast bort fra deg - " + REQUIRED_CLEAN_LAPS + " rene runder.",
        fullDescription: "Fly langs den markerte firkanten med nesa hele tiden fast rettet bort fra deg " +
            "(ren pinnestyring - sidelengs og baklengs inngår). Hold høyden på " + EXERCISE_ALTITUDE + " m. " +
            "Du må fullføre " + REQUIRED_CLEAN_LAPS + " runder uten avvik." + EXERCISE_RULES_TEXT,
        // captureRadius: 1.5 var for stramt gitt hvor vanskelig dybde er å bedømme fra VLOS på avstand
        // ("føles nær uten at det registreres") - 2.0 er fortsatt tydelig trangere enn sirkelens
        // punktavstand (standardradiusen 2.5 var satt for DEN og ga "godkjent hjørne" for tidlig her).
        stages: [{ id: "square-out", label: "Firkant - nese ut", waypoints: SQUARE_WAYPOINTS, noseMode: "out", captureRadius: 2.0 }]
    },
    ex3: {
        id: "ex3",
        icon: "fa-vector-square",
        label: "3. Firkant - nese fremover",
        shortDescription: "Firkanten med nesa i fartsretningen - 2 rene runder.",
        fullDescription: "Samme firkant, men nå skal nesa peke fremover i fartsretningen hele veien " +
            "(som en bil - yaw i hjørnene). Hold høyden på " + EXERCISE_ALTITUDE + " m. " +
            "2 runder uten avvik kreves - noe færre enn de andre øvelsene, siden nese-styringen her " +
            "i seg selv er den krevende delen." + EXERCISE_RULES_TEXT,
        // cornerGraceSec: nese-sjekken hviler noen sekunder etter hvert hjørne - der skal man
        // bremse, yawe 90° og akselerere ut, og fartsretningen henger naturlig etter nesa en stund.
        // requiredCleanLaps: 2 er nok her (se shortDescription) - default er REQUIRED_CLEAN_LAPS (3).
        stages: [{ id: "square-forward", label: "Firkant - nese frem", waypoints: SQUARE_WAYPOINTS, noseMode: "forward", cornerGraceSec: 4.5, captureRadius: 2.0, requiredCleanLaps: 2 }]
    },
    ex4: {
        id: "ex4",
        icon: "fa-arrows-up-down",
        label: "4. Høydeforandring",
        shortDescription: "Loddrett opp, skrått ned - sikksakk i vertikalplanet, begge veier.",
        fullDescription: "Følg det vertikale sikksakk-mønsteret med nesa bort fra deg: loddrett opp, " +
            "skrått ned til siden, loddrett opp igjen og skrått ned tilbake - " + REQUIRED_CLEAN_LAPS +
            " runder. Deretter flys det samme mønsteret motsatt vei (loddrett ned, skrått opp) i " +
            REQUIRED_CLEAN_LAPS + " runder til. Her er det formen som styrer høyden - fly gjennom " +
            "veipunktene i riktig rekkefølge." + EXERCISE_RULES_TEXT,
        stages: [
            { id: "zigzag-up", label: "Opp/skrått ned", waypoints: ZIGZAG_WAYPOINTS, noseMode: "out" },
            { id: "zigzag-down", label: "Ned/skrått opp", waypoints: ZIGZAG_WAYPOINTS_REVERSED, noseMode: "out" }
        ]
    },
    ex5: {
        id: "ex5",
        icon: "fa-circle-notch",
        label: "5. Sirkel - nese ut",
        shortDescription: "Sirkelen med nesa fast bort fra deg - " + REQUIRED_CLEAN_LAPS + " rene runder.",
        fullDescription: "Fly den markerte sirkelen med nesa hele tiden fast rettet bort fra deg. " +
            "Hold høyden på " + EXERCISE_ALTITUDE + " m. " + REQUIRED_CLEAN_LAPS + " runder uten avvik." +
            EXERCISE_RULES_TEXT,
        // captureRadius: sirkelpunktene ligger ~2,3 m fra hverandre - standardradiusen (2.5) tok
        // neste punkt nesten umiddelbart og lot en slurvete bane telle som ren runde.
        stages: [{ id: "circle-out", label: "Sirkel - nese ut", waypoints: CIRCLE_WAYPOINTS, noseMode: "out", captureRadius: 1.3 }]
    },
    ex6: {
        id: "ex6",
        icon: "fa-circle-notch",
        label: "6. Sirkel - nese fremover",
        shortDescription: "Sirkelen med nesa i fartsretningen - " + REQUIRED_CLEAN_LAPS + " rene runder.",
        fullDescription: "Samme sirkel, med nesa fremover i fartsretningen - jevn, koordinert yaw " +
            "gjennom hele svingen. Hold høyden på " + EXERCISE_ALTITUDE + " m. " + REQUIRED_CLEAN_LAPS +
            " runder uten avvik." + EXERCISE_RULES_TEXT,
        stages: [{ id: "circle-forward", label: "Sirkel - nese frem", waypoints: CIRCLE_WAYPOINTS, noseMode: "forward", captureRadius: 1.3 }]
    },
    ex7: {
        id: "ex7",
        icon: "fa-infinity",
        label: "7. Åttetall - nese ut",
        shortDescription: "Åttetall med nesa fast bort fra deg - " + REQUIRED_CLEAN_LAPS + " rene runder.",
        fullDescription: "Fly åttetallet med nesa hele tiden fast rettet bort fra deg - svingene bytter " +
            "retning midtveis, så pinneføringen speiles i hver løkke. Hold høyden på " + EXERCISE_ALTITUDE +
            " m. " + REQUIRED_CLEAN_LAPS + " runder uten avvik." + EXERCISE_RULES_TEXT,
        // captureRadius: åttetallets 24 punkter ligger ~1,5 m fra hverandre - standardradiusen (2.5)
        // "tok" punktene lenge før dronen var fremme og kunne hoppe over dem.
        stages: [{ id: "eight-out", label: "Åttetall - nese ut", waypoints: EIGHT_WAYPOINTS, noseMode: "out", captureRadius: 1.2 }]
    },
    ex8: {
        id: "ex8",
        icon: "fa-infinity",
        label: "8. Åttetall - nese fremover",
        shortDescription: "Åttetall med nesa i fartsretningen - 2 rene runder.",
        fullDescription: "Åttetallet med nesa fremover i fartsretningen - koordinert yaw som bytter " +
            "svingretning i kryssingen. Hold høyden på " + EXERCISE_ALTITUDE + " m. 2 runder uten avvik." +
            EXERCISE_RULES_TEXT,
        stages: [{ id: "eight-forward", label: "Åttetall - nese frem", waypoints: EIGHT_WAYPOINTS, noseMode: "forward", captureRadius: 1.2, requiredCleanLaps: 2 }]
    },
    ex9: {
        id: "ex9",
        icon: "fa-wind",
        label: "9. Åttetall i vind",
        shortDescription: "Åttetall i 3 m/s sidevind, nesa ut - " + REQUIRED_CLEAN_LAPS + " rene runder.",
        fullDescription: "Samme åttetall, men nå med 3 m/s vind aktivert automatisk - du må korrigere " +
            "kontinuerlig for avdrift for å holde formen. Flys anbefalt med nesa ut, men nese-retningen " +
            "sjekkes ikke her - fokuset er vindkorreksjon og formen, ikke nesestilling. Hold høyden på " +
            EXERCISE_ALTITUDE + " m. " + REQUIRED_CLEAN_LAPS + " runder uten avvik. Vinden settes tilbake " +
            "til dine egne innstillinger når øvelsen avsluttes." + EXERCISE_RULES_TEXT,
        wind: { speed: 3, directionDeg: 90, gust: 0.2 },
        // noseMode "free": ingen nese-sjekk i det hele tatt (se updateExercise) - nese ut er anbefalt
        // stil, men i praksis fritt frem siden vindkorreksjon alene er nok utfordring her.
        stages: [{ id: "eight-wind", label: "Åttetall i vind - nese ut", waypoints: EIGHT_WAYPOINTS, noseMode: "free", captureRadius: 1.2 }]
    },
    ex10: {
        id: "ex10",
        icon: "fa-house",
        label: "10. Returner hjem i vind",
        shortDescription: "Ta over en drone langt hjemmefra og fly den trygt hjem - " + REQUIRED_RETURN_REPS + " ganger, 6 m/s vind fra tilfeldig retning.",
        fullDescription: "Dronen henger i lufta langt hjemmefra - bare synlig som en prikk - med " +
            "tilfeldig nese-retning og 6 m/s vind fra en ny, tilfeldig retning hver gang. Etter " +
            "nedtellingen overtar du kontrollen, men først når du har lagt gassen rundt 50 % (hover) - " +
            "akkurat som en ekte overtakelse. Finn ut hvilken vei nesa peker og hvor vinden kommer fra, " +
            "fly dronen trygt hjem og land på landingsplassen (H). Du må gjennomføre dette " +
            REQUIRED_RETURN_REPS + " ganger (ny tilfeldig posisjon, nese-retning og vindretning hver " +
            "gang) for å bestå - totaltiden for alle " + REQUIRED_RETURN_REPS + " forsøkene lagres i " +
            "menyen. R gir nytt forsøk fra runde 1 med nullstilt klokke.",
        wind: { speed: 6, directionDeg: 0, gust: 0.3 },
        randomizeWindDirection: true, // ny tilfeldig retning ved hver spawn (start/runde/R) - se spawnForExercise
        randomizeCloudCoverage: true, // tilfeldig skydekke per runde, men minst én av de tre garantert 100% - se spawnForExercise
        spawn: "far", // spesialspawn: høyt og langt unna med tilfeldig yaw (se spawnForExercise)
        stages: [{ id: "return-home", label: "Returner hjem", type: "return" }]
    }
};
const EXERCISE_ORDER = ["ex1", "ex2", "ex3", "ex4", "ex5", "ex6", "ex7", "ex8", "ex9", "ex10"];

// Kun sluttresultat (bestått + beste tid) lagres - all fremdrift underveis (steg/runde/tid/varsler)
// lever kun i minnet og forsvinner ved sideinnlasting, se exerciseState lenger ned.
const DEFAULT_EXERCISE_PROGRESS = {};
EXERCISE_ORDER.forEach(function (id) {
    DEFAULT_EXERCISE_PROGRESS[id] = { passed: false, bestTimeSec: null };
});

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
function loadExerciseProgress() {
    return Sim.loadJSON(EXERCISE_STORAGE_KEY, DEFAULT_EXERCISE_PROGRESS);
}
function saveExerciseProgress() {
    Sim.saveJSON(EXERCISE_STORAGE_KEY, exerciseProgress);
}

/* ---------- Tilstand ---------- */
const rates = loadRates();
const gamepadMap = loadGamepadMap();
const settings = loadSettings();
const exerciseProgress = loadExerciseProgress();

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
    crashed: false, // hard landing (se CRASH_SINK_RATE) - killswitch slår automatisk inn, varsel i HUD
    injured: false // droneen har truffet VLOS-piloten - eget varsel (legevakt/ambulanse), R for restart
};

let linkQuality = 1;

// Skade per propell/motor, 0 (hel) .. 1 (helt ødelagt) - indeks matcher MOTOR_MIX/motorOffsets
// (0=fremre-høyre, 1=fremre-venstre, 2=bakre-høyre, 3=bakre-venstre). Se "Propellskade"-konstantene.
const propDamage = [0, 0, 0, 0];
// Stigende-flanke-flagg per propell: treff-skade gis én gang per sammenhengende berøring, ikke per tick.
const propContactActive = [false, false, false, false];

// "Klebrig" bakkekontakt (1 ved kontakt, dør ut over GROUND_CONTACT_BLEND_DECAY sekunder) - brukes av
// autoritets-reduksjonen slik at intermitterende bein-berøringer under skimming ikke lar stabiliseringen
// redde droneen i de frie tickene mellom berøringene.
let groundContactBlend = 0;

/* ---------- Vind (stabil + kast, se Sim.computeWind i simulator-common.js) ---------- */
const currentWindVector = new THREE.Vector3();
const windGustOffset = new THREE.Vector3();

function updateWind(dt) {
    Sim.computeWind(dt, settings.wind, windGustOffset, currentWindVector);
}

function currentDroneSpec() {
    return DRONE_CLASSES[droneState.droneClass];
}

// Etter klassebytte mens droneen står på bakken: den nye modellen har annen benhøyde/skala, så
// senterhøyden fra den gamle klassen lar den enten sveve (faller og "lander på nytt") eller stå
// nede i bakken (dyttes opp - ser ut som et hopp). Legg den direkte i ro på underlaget i stedet.
function settleDroneOnGround() {
    if (!droneState.grounded) return;
    const spec = currentDroneSpec();
    const pts = getContactLocalPoints(droneState.droneClass);
    let minY = Infinity;
    for (let i = 0; i < 4; i++) minY = Math.min(minY, pts[i].y); // underside-punktene (0-3)
    const surfaceY = solidSurfaceHeightAt(droneState.position.x, droneState.position.z);
    droneState.position.y = surfaceY - minY * spec.visualScale;
    droneState.velocity.set(0, 0, 0);
}

function setDroneClass(className) {
    if (!DRONE_CLASSES[className]) return;
    droneState.droneClass = className;
    settings.droneClass = className;
    saveSettings();
    // Drone-typene har ulik geometri (ben, canopy, propell-blad) - ikke bare skala - så modellen
    // må bygges på nytt, ikke bare skaleres, når typen endres.
    if (scene) rebuildDroneMesh();
    settleDroneOnGround();
}

// Samme som setDroneClass, men rører aldri settings/localStorage - brukt av øvelsene til å midlertidig
// tvinge Middels-droneen mens en øvelse er aktiv, uten å overskrive brukerens vanlige valg permanent.
function setDroneClassEphemeral(className) {
    if (!DRONE_CLASSES[className]) return;
    droneState.droneClass = className;
    if (scene) rebuildDroneMesh();
    settleDroneOnGround();
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

/* ---------- Fjern bakgrunn: fjellkjede, skogsområde, skyer ----------
   Rent visuelt "varierende bakgrunn mot droneen" - ingen kollisjon, ligger godt innenfor himmel-kulen
   (radius 800, se Sim.buildGradientSky) og godt innenfor kameraenes far-plane (2000). Faste, hånd-
   plasserte/deterministiske posisjoner (ingen Math.random() ved verdensbygging) - verden skal se lik
   ut mellom sideinnlastinger, akkurat som trærne/portene ellers i filen.
*/
// Færre enn før (14 -> 8) og skjøvet lenger ut mot ytterkanten av kartet (dist ~540-620, opp fra
// ~440-580) - god klaring til alt som faktisk kan flys til (racing-løypa når maks ~131 fra origo,
// "Returner hjem" spawner maks 170 unna). radius er fortsatt ~1.6x høyden for en naturtro, slak
// silhuett; dist+radius holder fortsatt trygg margin til himmelkulen (radius 800).
const MOUNTAIN_DEFS = [
    { angle: 0, dist: 620, height: 69, radius: 110, snow: false },
    { angle: 45, dist: 560, height: 103, radius: 165, snow: true },
    { angle: 90, dist: 600, height: 81, radius: 130, snow: false },
    { angle: 135, dist: 540, height: 116, radius: 185, snow: true },
    { angle: 180, dist: 610, height: 75, radius: 120, snow: false },
    { angle: 225, dist: 570, height: 97, radius: 155, snow: true },
    { angle: 270, dist: 590, height: 88, radius: 140, snow: false },
    { angle: 315, dist: 550, height: 109, radius: 175, snow: true }
];
// Fargestopp brukt av buildGradientPeakGeometry - se der.
const MOUNTAIN_GROUND_COLOR = new THREE.Color(0x3a5f3a);   // matcher Sim.buildGroundTexture
const MOUNTAIN_FOOTHILL_COLOR = new THREE.Color(0x6e7a4d); // oliven - bro mellom bakke og stein
const MOUNTAIN_ROCK_COLOR = new THREE.Color(0x5b6472);
const MOUNTAIN_ROCK_LIGHT_COLOR = new THREE.Color(0x7c8794);
const MOUNTAIN_SNOW_COLOR = new THREE.Color(0xf0f4f8);

// Bygger en CylinderGeometry (med et lite, butt topp-radius i stedet for ConeGeometrys matematisk
// skarpe spiss - se topRadiusFrac) med (1) uregelmessig, ru silhuett - vinkelavhengig sinus-støy
// forskyver hver vertekes radius og litt av høyden, styrt av "jaggedness" (0 = helt glatt/konisk, brukt
// for den slake foten) - og (2) en jevn per-vertex fargeovergang mellom flere høydebaserte fargestopp i
// stedet for separate meshes med harde fargegrenser (det ga tidligere en synlig skarp overgang der
// snø-/stein-meshene møttes). Jitteren dempes (ikke fjernes helt) nær toppen, for en avrundet/erodert
// topp i stedet for enten en skarp spiss eller en unaturlig helt glatt/flat platå-sirkel.
function buildGradientPeakGeometry(radius, height, seed, colorStops, jaggedness, topRadiusFrac) {
    const topRadius = radius * (topRadiusFrac || 0);
    const geo = new THREE.CylinderGeometry(topRadius, radius, height, 14, 7);
    const pos = geo.attributes.position;
    const colors = new Float32Array(pos.count * 3);
    const tmp = new THREE.Color();
    for (let i = 0; i < pos.count; i++) {
        const x = pos.getX(i), y = pos.getY(i), z = pos.getZ(i);
        const heightFrac = clamp((y + height / 2) / height, 0, 1);
        const angle = Math.atan2(z, x);
        if (jaggedness > 0) {
            const topDamp = 1 - Math.pow(heightFrac, 3) * 0.7; // roligere silhuett mot toppen, ikke null
            const jitter = Math.sin(angle * 5 + seed * 3.1) * 0.18 + Math.sin(angle * 11 + seed * 7.7) * 0.1;
            const radialScale = 1 + jitter * jaggedness * topDamp;
            pos.setX(i, x * radialScale);
            pos.setZ(i, z * radialScale);
            pos.setY(i, y + Math.sin(angle * 7 + seed * 4.3) * height * 0.03 * jaggedness * topDamp);
        }
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
    // Delt for alt (hovedtopp, bi-topper) - fargen kommer utelukkende fra per-vertex-attributtet over,
    // ikke fra materialet, så ett shared material er nok.
    const peakMat = new THREE.MeshStandardMaterial({ vertexColors: true, flatShading: true, roughness: 1 });

    MOUNTAIN_DEFS.forEach(function (m, i) {
        const rad = THREE.MathUtils.degToRad(m.angle);
        const x = Math.sin(rad) * m.dist, z = Math.cos(rad) * m.dist;

        // Én sammenhengende geometri fra bakkeplanet (frac 0) helt til toppen (frac 1) - ingen egen
        // "fot"-mesh lenger. Det fjerner den synlige skjøten/ringen der en separat fot- og topp-mesh
        // møttes, OG gir en jevn fargeovergang fra selveste bakkefargen (matcher Sim.buildGroundTexture)
        // via oliven-fjellfot og gråstein til ev. snø, i stedet for et brått fargehopp ved bakken.
        const peakColorStops = m.snow
            ? [
                { frac: 0, color: MOUNTAIN_GROUND_COLOR },
                { frac: 0.16, color: MOUNTAIN_FOOTHILL_COLOR },
                { frac: 0.42, color: MOUNTAIN_ROCK_COLOR },
                { frac: 0.82, color: MOUNTAIN_ROCK_LIGHT_COLOR },
                { frac: 1, color: MOUNTAIN_SNOW_COLOR }
            ]
            : [
                { frac: 0, color: MOUNTAIN_GROUND_COLOR },
                { frac: 0.18, color: MOUNTAIN_FOOTHILL_COLOR },
                { frac: 0.5, color: MOUNTAIN_ROCK_COLOR },
                { frac: 1, color: MOUNTAIN_ROCK_LIGHT_COLOR }
            ];
        const peak = new THREE.Mesh(buildGradientPeakGeometry(m.radius, m.height, i * 1.7 + 1, peakColorStops, 1, 0.18), peakMat);
        peak.position.set(x, m.height / 2, z);
        peak.rotation.y = rad;
        group.add(peak);

        // Et par mindre, forskjøvne bi-topper klumpet inntil hovedtoppen - et enkelt fjell blir til et
        // massiv med flere rygger i stedet for én ensom, symmetrisk pyramide. Egne, mindre forskyvninger
        // (subDist) nå som radiene er mye større - ellers ville de stukket unødvendig langt ut fra massivet.
        [{ f: 0.55, off: 0.32, dirOffset: 1.3 }, { f: 0.4, off: -0.38, dirOffset: 3.6 }].forEach(function (sub, si) {
            const subHeight = m.height * sub.f;
            const subRadius = m.radius * (0.5 + sub.f * 0.2);
            const subDir = rad + sub.dirOffset;
            const subDist = m.radius * sub.off;
            const subColorStops = [
                { frac: 0, color: MOUNTAIN_GROUND_COLOR },
                { frac: 0.2, color: MOUNTAIN_FOOTHILL_COLOR },
                { frac: 0.5, color: MOUNTAIN_ROCK_COLOR },
                { frac: 1, color: MOUNTAIN_ROCK_LIGHT_COLOR }
            ];
            const subPeak = new THREE.Mesh(
                buildGradientPeakGeometry(subRadius, subHeight, i * 1.7 + 2 + si * 5.3, subColorStops, 1, 0.2),
                peakMat
            );
            subPeak.position.set(x + Math.sin(subDir) * subDist, subHeight / 2, z + Math.cos(subDir) * subDist);
            subPeak.rotation.y = subDir;
            group.add(subPeak);
        });
    });
    return group;
}

// Skogsområde med høyere trær (10-16 m) enn de spredte dekorasjonstrærne i buildWorldObjects (6-8,5 m) -
// en egen bakgrunnsflekk et godt stykke unna bane/bygninger/øvelser. Jitter fra sin/cos av indeksen i
// stedet for Math.random(), samme determinisme-prinsipp som resten av verdensbyggingen.
function buildForestArea() {
    const group = new THREE.Group();
    const centerX = 140, centerZ = 90;
    const rows = 6, cols = 6, spacing = 11;
    let i = 0;
    for (let r = 0; r < rows; r++) {
        for (let c = 0; c < cols; c++) {
            const jitterX = Math.sin(i * 12.9) * 4;
            const jitterZ = Math.cos(i * 7.3) * 4;
            const height = 10 + Math.abs(Math.sin(i * 3.7)) * 6;
            const tree = Sim.buildTree(height);
            tree.position.set(centerX + (c - cols / 2) * spacing + jitterX, 0, centerZ + (r - rows / 2) * spacing + jitterZ);
            group.add(tree);
            i++;
        }
    }
    return group;
}

// Skydomene (wrap-grense) og minimums-driftfart - se updateClouds().
const CLOUD_DOMAIN = 700; // skyene wrapper innenfor ±350 på både X og Z
const CLOUD_MIN_SPEED = 0.6; // m/s - alltid litt bevegelse i himmelen, selv med vindstyrke 0
const cloudClusters = []; // fylt av buildClouds() - hver er en THREE.Group, flyttes av updateClouds()

// Delte materialer (én per gråtone-variant) i stedet for ett nytt material per skyklynge - med mange
// klynger (se CLOUD_COUNT) sparer det en god del unødvendige shader-programmer/draw calls.
// Glatt skyggelegging (ikke flatShading, i motsetning til fjellene) - myke, avrundede puffer leser som
// "fluffy" bomullsklumper, mens flate fasetter ville sett steinete/lavpoly ut for noe som skal virke mykt.
const CLOUD_SHADE_MATERIALS = [0xff, 0xeb, 0xd7, 0xc3].map(function (shade) {
    return new THREE.MeshStandardMaterial({ color: (shade << 16) | (shade << 8) | shade, roughness: 0.9 });
});
// Ekte cumulus har en tydelig FLAT underside (kondensasjonsnivået) og en rundet, kupert overside - en
// vanlig kule er rund begge veier og leser derfor feil. Drar alle vertekser under en gitt høyde opp til
// samme flate (i stedet for en skulptert kule), som gir en ekte flat bunn-fasett på selve geometrien.
function buildFlatBottomedPuffGeometry(radius, widthSegments, heightSegments, flattenFrac) {
    const geo = new THREE.SphereGeometry(radius, widthSegments, heightSegments);
    const pos = geo.attributes.position;
    const flattenY = -radius * flattenFrac;
    for (let i = 0; i < pos.count; i++) {
        if (pos.getY(i) < flattenY) pos.setY(i, flattenY);
    }
    geo.computeVertexNormals();
    return geo;
}
function buildCloudCluster(seed) {
    // Små klynger med 4-6 mykt skyggelagte, opake puffer - MANGE små fluffy skyer ser bedre ut enn få
    // store (som fort blir en boksete/klumpete masse når mange av dem er synlige samtidig, se
    // updateClouds sin dekningsgrad). Opakt for å unngå sorterings-glitch mellom overlappende puffer.
    // Én liten kjerne-puff pluss noen få rundt/oppå - samme cumulus-oppbygning, bare i miniatyr.
    const group = new THREE.Group();
    const puffCount = 4 + (seed % 3);
    const mat = CLOUD_SHADE_MATERIALS[seed % CLOUD_SHADE_MATERIALS.length]; // gråtone-variasjon ("varierte skyer")

    const coreRadius = 3.5 + Math.abs(Math.sin(seed * 1.3)) * 1.3;
    const core = new THREE.Mesh(new THREE.SphereGeometry(coreRadius, 8, 6), mat);
    core.scale.y = 0.7;
    group.add(core);

    for (let i = 0; i < puffCount; i++) {
        const r = 1.8 + Math.abs(Math.sin(seed * 1.7 + i)) * 2;
        const puff = new THREE.Mesh(new THREE.SphereGeometry(r, 7, 5), mat);
        // Puffene klumper seg tett inntil/oppå kjernen (liten radius fra senter) i stedet for spredt
        // utover - det er tettheten/overlappet som gir det sammenhengende, fluffy inntrykket.
        const spread = coreRadius * 0.65;
        puff.position.set(
            Math.sin(seed + i * 2.1) * spread,
            Math.abs(Math.cos(seed * 0.8 + i)) * coreRadius * 0.5, // mest oppover - kupert topp, flat bunn
            Math.cos(seed + i * 1.6) * spread
        );
        puff.scale.y = 0.7;
        group.add(puff);
    }
    return group;
}
function buildClouds() {
    const group = new THREE.Group();
    // Høyt nok til at 100% skydekke faktisk ser fullt overskyet ut, ikke bare noen spredte klynger -
    // se coverageStep-fordelingen i updateClouds(), som viser en jevnt fordelt DEL av disse.
    const CLOUD_COUNT = 60;
    for (let i = 0; i < CLOUD_COUNT; i++) {
        const cluster = buildCloudCluster(i);
        const angle = (i / CLOUD_COUNT) * Math.PI * 2 + Math.sin(i) * 0.3;
        const dist = 110 + Math.abs(Math.sin(i * 2.3)) * 230; // maks 340 - godt innenfor CLOUD_DOMAIN/2
        cluster.position.set(Math.sin(angle) * dist, 140 + Math.abs(Math.cos(i * 1.3)) * 90, Math.cos(angle) * dist);
        // baseScale huskes (userData) - updateClouds() skalerer OPPÅ denne ved høy dekning, slik at
        // klyngene vokser seg sammen til et sammenhengende, virkelig overskyet dekke ved 100% i stedet
        // for bare "mange spredte puffer med luft mellom" (som fortsatt ville sett ut som glimt av himmel).
        const baseScale = 1.4 + Math.abs(Math.sin(i * 4.1)) * 1.4;
        cluster.userData.baseScale = baseScale;
        cluster.scale.setScalar(baseScale);
        group.add(cluster);
        cloudClusters.push(cluster);
    }
    return group;
}
// Wrapper v inn i [-domain/2, domain/2) - holder skyene i evig sirkulasjon uansett vindretning, i
// stedet for at de driver ut av verden for godt etter noen minutters flyging.
function cloudWrapCoord(v, domain) {
    const half = domain / 2;
    return ((v + half) % domain + domain) % domain - half;
}
// Skyene driver etter samme retning/styrke som er satt i Vær-panelet, men KUN når "Aktiver vind" er
// slått på (stille luft = ingen skybevegelse, samme intuisjon som resten av vind-effekten) - pluss en
// liten minimumsfart mens vind er på, slik at himmelen ikke står helt stille selv ved lav styrke.
// Dekning (0-1) styrer hvor mange av de forhåndsbygde klyngene som er synlige, jevnt fordelt (moduloen
// nedenfor sprer dem utover settet i stedet for å bare vise de N første, som ville gitt en skjev,
// samlet dekning) - uavhengig av om vinden er på, slik at man kan se/justere skydekket i stille vær også.
function updateClouds(dt) {
    if (settings.wind.enabled) {
        const dirRad = THREE.MathUtils.degToRad(settings.wind.directionDeg);
        const speed = Math.max(CLOUD_MIN_SPEED, settings.wind.speed * 0.4); // litt saktere enn selve vindstyrken
        const dx = Math.sin(dirRad) * speed * dt;
        const dz = Math.cos(dirRad) * speed * dt;
        cloudClusters.forEach(function (cluster) {
            cluster.position.x = cloudWrapCoord(cluster.position.x + dx, CLOUD_DOMAIN);
            cluster.position.z = cloudWrapCoord(cluster.position.z + dz, CLOUD_DOMAIN);
        });
    }
    const coverage = settings.cloudsEnabled ? clamp(settings.cloudCoverage, 0, 1) : 0;
    const coverageStep = coverage > 0 ? Math.round(1 / clamp(coverage, 0.05, 1)) : 0;
    // Vokser klyngene noe større jo høyere dekningen er (opptil 2.2x ved 100%) - ved full dekning skal
    // de faktisk smelte sammen til et sammenhengende lag, ikke bare stå som mange separate puffer.
    const growth = 1 + coverage * 1.2;
    cloudClusters.forEach(function (cluster, i) {
        cluster.visible = coverageStep > 0 && (i % coverageStep) === 0;
        cluster.scale.setScalar(cluster.userData.baseScale * growth);
    });
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

// Posisjon + retning per baneelement, delt kilde for både den visuelle byggingen (buildGateCourse) og
// propell-treff-boksene (PROP_HAZARDS) - slik at kollisjonsgeometrien alltid stemmer med det man ser.
// Retningen peker mot neste veipunkt, slik at man flyr gjennom (port eller låvevindu) langs løypa.
const GATE_PLACEMENTS = GATE_WAYPOINTS.map(function (wp, i) {
    const next = GATE_WAYPOINTS[(i + 1) % GATE_WAYPOINTS.length];
    return {
        wp: wp,
        x: GATE_COURSE_CENTER.x + wp.dx,
        z: GATE_COURSE_CENTER.z + wp.dz,
        yaw: Math.atan2(next.dx - wp.dx, next.dz - wp.dz)
    };
});

function buildGateCourse() {
    const group = new THREE.Group();
    GATE_PLACEMENTS.forEach(function (placement) {
        const wp = placement.wp;
        const obstacle = (wp.type === "barn")
            ? buildBarn(BARN_DIMENSIONS.width, BARN_DIMENSIONS.height, BARN_DIMENSIONS.depth,
                BARN_DIMENSIONS.windowW, BARN_DIMENSIONS.windowH, BARN_DIMENSIONS.sillY)
            : buildGate(wp.size, wp.gap);
        obstacle.position.set(placement.x, 0, placement.z);
        obstacle.rotation.y = placement.yaw;
        group.add(obstacle);
    });
    return group;
}

/* ---------- Propell-treffbokser for port-rammer og låvevegger ----------
   Portene/låven har ingen "solid" kollisjon for selve dronekroppen (bevisst - å fly gjennom skal være
   flytende), men propellene skal kunne treffe rammene. Boksene under er akse-rettede i hvert elements
   LOKALE ramme (punkter roteres inn med -yaw før testen). Ben-sylindrene er slått sammen med side-
   stolpene til én boks fra bakken og opp. Speiler geometrien i buildGate()/buildBarn().
*/
const PROP_HAZARDS = GATE_PLACEMENTS.map(function (placement) {
    const wp = placement.wp;
    const boxes = [];
    let boundR, maxY;
    if (wp.type === "barn") {
        const b = BARN_DIMENSIONS, t = 0.15; // halv veggtykkelse (0.3/2)
        const hw = b.width / 2, hd = b.depth / 2;
        boxes.push({ minX: -hw - t, maxX: -hw + t, minY: 0, maxY: b.height, minZ: -hd, maxZ: hd }); // venstre vegg
        boxes.push({ minX: hw - t, maxX: hw + t, minY: 0, maxY: b.height, minZ: -hd, maxZ: hd });   // høyre vegg
        [-hd, hd].forEach(function (zPos) { // vindusvegger: fire paneler rundt åpningen
            const topY = b.sillY + b.windowH, panelW = (b.width - b.windowW) / 2;
            boxes.push({ minX: -hw, maxX: hw, minY: 0, maxY: b.sillY, minZ: zPos - t, maxZ: zPos + t });
            boxes.push({ minX: -hw, maxX: hw, minY: topY, maxY: b.height, minZ: zPos - t, maxZ: zPos + t });
            boxes.push({ minX: -hw, maxX: -hw + panelW, minY: b.sillY, maxY: topY, minZ: zPos - t, maxZ: zPos + t });
            boxes.push({ minX: hw - panelW, maxX: hw, minY: b.sillY, maxY: topY, minZ: zPos - t, maxZ: zPos + t });
        });
        boxes.push({ minX: -hw - 0.3, maxX: hw + 0.3, minY: b.height, maxY: b.height + 0.3, minZ: -hd - 0.3, maxZ: hd + 0.3 }); // tak
        boundR = Math.hypot(hw + 0.3, hd + 0.3);
        maxY = b.height + 0.3;
    } else {
        const s = wp.size, gap = wp.gap, t = 0.09; // halv bar-tykkelse (0.18/2)
        const hs = s / 2;
        boxes.push({ minX: -hs, maxX: hs, minY: gap - t, maxY: gap + t, minZ: -t, maxZ: t });           // nedre bar
        boxes.push({ minX: -hs, maxX: hs, minY: gap + s - t, maxY: gap + s + t, minZ: -t, maxZ: t });   // øvre bar
        boxes.push({ minX: -hs - t, maxX: -hs + t, minY: 0, maxY: gap + s, minZ: -t, maxZ: t });        // venstre stolpe + ben
        boxes.push({ minX: hs - t, maxX: hs + t, minY: 0, maxY: gap + s, minZ: -t, maxZ: t });          // høyre stolpe + ben
        boundR = hs + 0.2;
        maxY = gap + s + 0.2;
    }
    return {
        x: placement.x, z: placement.z,
        cosYaw: Math.cos(placement.yaw), sinYaw: Math.sin(placement.yaw),
        boundRSq: (boundR + 1.5) * (boundR + 1.5), // margin for arm + propellrekkevidde
        maxY: maxY + 0.5,
        boxes: boxes
    };
});

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

// Propell bygget av individuelle blad-pivoter (hub i origo, bladet stikker ut langs pivotens +X) -
// også for 2-blads, slik at propellskade kan brekke av ett og ett blad (pivot.scale.x skalerer
// bladet fra huben og utover, se updatePropDamageVisual). Pivotene ligger i group.userData.blades.
function buildPropeller(bladeCount, bladeLength) {
    const group = new THREE.Group();
    const mat = new THREE.MeshStandardMaterial({ color: 0x111111, transparent: true, opacity: 0.8 });
    const blades = [];
    for (let i = 0; i < bladeCount; i++) {
        const pivot = new THREE.Group();
        pivot.rotation.y = (i / bladeCount) * Math.PI * 2;
        const blade = new THREE.Mesh(new THREE.BoxGeometry(bladeLength * 0.5, 0.005, bladeCount <= 2 ? 0.02 : 0.018), mat);
        blade.position.x = bladeLength * 0.25;
        pivot.add(blade);
        group.add(pivot);
        blades.push(pivot);
    }
    group.userData.blades = blades;
    return group;
}

// Fjernkontroll holdt i begge hender foran magen på VLOS-piloten - kasse med to pinner og antenne.
function buildRemoteController() {
    const group = new THREE.Group();
    const bodyMat = new THREE.MeshStandardMaterial({ color: 0x22262a });
    const body = new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.05, 0.12), bodyMat);
    body.castShadow = true;
    group.add(body);
    const stickMat = new THREE.MeshStandardMaterial({ color: 0x999999 });
    [-1, 1].forEach(function (side) {
        const stick = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, 0.05, 6), stickMat);
        stick.position.set(side * 0.06, 0.04, 0);
        group.add(stick);
    });
    const antenna = new THREE.Mesh(new THREE.CylinderGeometry(0.005, 0.005, 0.2, 6), bodyMat);
    antenna.position.set(0, 0.09, -0.06);
    antenna.rotation.x = -0.6; // peker skrått opp/bakover mot piloten
    group.add(antenna);
    group.rotation.x = 0.35; // vippet litt mot piloten, slik en sender faktisk holdes
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
// Delt mellom det visuelle (buildDrone) og propell-treffdeteksjonen (updatePropStrikes) - propellens
// tuppradius er halve bladlengden i begge blad-variantene.
// Økt fra 0.2/0.28 - ekte quad-propeller er typisk ~50-55% av (diagonal) motor-til-motor-avstand
// (f.eks. 5" prop = 127mm på en ~230mm wheelbase); med DRONE_ARM_LENGTH=0.22 gir motorene en diagonal
// avstand på 0.44, og de gamle bladlengdene (radius 0.1/0.14) var langt kortere/stumpere enn det -
// disse verdiene gir fortsatt trygg klaring mellom nabopropeller (se motorOffsets).
function bladeLengthForClass(classKey) {
    return classKey === "cinematic" ? 0.38 : 0.3;
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
    // Rekkefølgen (fremre-høyre, fremre-venstre, bakre-høyre, bakre-venstre) MÅ matche MOTOR_MIX og
    // propDamage - dronePropellers[i] og motor i i mikseren er samme fysiske hjørne.
    const motorOffsets = [
        { x: armLength, z: -armLength, dir: 1 },
        { x: -armLength, z: -armLength, dir: -1 },
        { x: armLength, z: armLength, dir: -1 },
        { x: -armLength, z: armLength, dir: 1 }
    ];
    const bladeCount = isRacing ? 3 : 2;
    const bladeLength = bladeLengthForClass(classKey);
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
    scene.add(buildMountainRange());
    scene.add(buildForestArea());
    scene.add(buildClouds());

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
    const vlosPerson = Sim.buildPersonFigure({ holdingController: true });
    vlosPerson.position.copy(vlosCamera.position);
    vlosPerson.position.y = 0;
    vlosPerson.rotation.y = Math.PI; // vendt mot flyfeltet (-Z) - figuren bygges med tærne mot +Z
    const controller = buildRemoteController();
    controller.position.set(0, 1.05, 0.28); // holdt foran magen i begge hender (lokal +Z = mot feltet etter snuingen)
    vlosPerson.add(controller);
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
    // Nybygde propeller er visuelt hele - påfør gjeldende skade på nytt så modellen og fysikken
    // (propDamage kuttes fortsatt i mikseren) aldri forteller to ulike historier.
    for (let i = 0; i < propDamage.length; i++) updatePropDamageVisual(i);
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

/* ---------- Fysikk: motor-mikser og VRS (se konstant-kommentarene lenger oppe) ---------- */
function mixMotors(baseCmd, rollCmd, pitchCmd, yawCmd, airmodeOn) {
    const raw = MOTOR_MIX.map(m => baseCmd + MIX_AUTHORITY * (m.roll * rollCmd + m.pitch * pitchCmd + m.yaw * yawCmd));
    if (airmodeOn) {
        // Løft hele motorsettet samlet slik at laveste motor akkurat når gulvet - bevarer differensialen
        // (kontrollautoriteten) i stedet for å klemme hver motor uavhengig og miste den.
        const minVal = Math.min(raw[0], raw[1], raw[2], raw[3]);
        if (minVal < AIRMODE_MIN_IDLE) {
            const offset = AIRMODE_MIN_IDLE - minVal;
            for (let i = 0; i < raw.length; i++) raw[i] += offset;
        }
        return raw.map(v => clamp(v, AIRMODE_MIN_IDLE, 1));
    }
    // Uten airmode klemmes hver motor uavhengig - differensialen mistes nær gass-ytterpunktene.
    return raw.map(v => clamp(v, 0, 1));
}
function extractMixedBase(motorValues) {
    return (motorValues[0] + motorValues[1] + motorValues[2] + motorValues[3]) / motorValues.length;
}
function extractMixedAxis(motorValues, axisKey) {
    let sum = 0;
    for (let i = 0; i < MOTOR_MIX.length; i++) sum += motorValues[i] * MOTOR_MIX[i][axisKey];
    return (sum / MOTOR_MIX.length) / MIX_AUTHORITY;
}
function computeVrsThrustFactor(velocity) {
    const descentRate = -velocity.y; // positiv = synker
    if (descentRate <= VRS_DESCENT_ONSET) return 1;
    const horizSpeed = Math.hypot(velocity.x, velocity.z);
    const horizEscape = clamp(1 - horizSpeed / VRS_HORIZ_SPEED_ESCAPE, 0, 1); // 0 = fløy ut av det
    const descentSeverity = clamp((descentRate - VRS_DESCENT_ONSET) / (VRS_DESCENT_FULL - VRS_DESCENT_ONSET), 0, 1);
    return 1 - VRS_MAX_THRUST_LOSS * descentSeverity * horizEscape;
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
        // Dette er ØNSKET gass, før motor-mikseren under eventuelt må kutte i den pga. metning.
        const baseCmd = clamp(thrustForce / spec.maxThrust, 0, 1);

        const desiredRateRad = {
            roll: THREE.MathUtils.degToRad(desiredRateDeg.roll),
            pitch: THREE.MathUtils.degToRad(desiredRateDeg.pitch),
            yaw: THREE.MathUtils.degToRad(desiredRateDeg.yaw)
        };
        // Three.js' aksekonvensjon (forward = -Z) gir motsatt rotasjonsfortegn av "pinne-intuisjonen"
        // (pinne+ = nese ned / rull høyre / sving høyre) for alle tre aksene - derfor negeres her.
        // "Ønsket moment" (torque-lignende størrelse) før mikser-metning - normalisert via axisTorqueNorm()
        // til en rull/pitch/yaw-kommando i [-1, 1] som sendes inn i motor-mikseren sammen med baseCmd
        // over, i stedet for å sette vinkelakselerasjon direkte og uavhengig av gassnivå.
        const desiredTorqueCmd = {
            pitch: TORQUE_GAIN * (-desiredRateRad.pitch - droneState.angularVelocity.pitch),
            roll: TORQUE_GAIN * (-desiredRateRad.roll - droneState.angularVelocity.roll),
            yaw: TORQUE_GAIN * (-desiredRateRad.yaw - droneState.angularVelocity.yaw)
        };
        const rollNorm = axisTorqueNorm(rates.roll);
        const pitchNorm = axisTorqueNorm(rates.pitch);
        const yawNorm = axisTorqueNorm(rates.yaw);
        const rollCmd = clamp(desiredTorqueCmd.roll / rollNorm, -1, 1);
        const pitchCmd = clamp(desiredTorqueCmd.pitch / pitchNorm, -1, 1);
        const yawCmd = clamp(desiredTorqueCmd.yaw / yawNorm, -1, 1);

        // Fire virtuelle motorer, klemt til [gulv, 1] - se mixMotors. Faktisk oppnådd gass/moment
        // beregnes FRA de klemte motorverdiene, ikke fra det som ble ønsket over: nær 0 % gass uten
        // airmode spises rull/pitch/yaw-autoriteten opp, og nær 100 % gass stjeler harde manøvre trekkraft.
        const motorValues = mixMotors(baseCmd, rollCmd, pitchCmd, yawCmd, settings.airmodeEnabled);
        // Propellskade kutter den enkelte motorens FAKTISKE output (etter mikser/airmode) - det gir
        // automatisk både trekkraft-tap og et skjevt moment som tipper mot det skadde hjørnet.
        for (let i = 0; i < motorValues.length; i++) motorValues[i] *= (1 - propDamage[i]);
        thrustForce = extractMixedBase(motorValues) * spec.maxThrust;

        // Ligger droneen veltet PÅ BAKKEN mister pinnene nesten all effekt (propellene jobber rett mot
        // bakken) - man skal ikke kunne "stikke-rulle" en veltet drone opp igjen. Kun i bakkekontakt;
        // stor tilt i fri flukt er normal acro-flyging. Glatt overgang mellom 30° og 70° tilt.
        // Skraping (understellet dras langs bakken i fart) struper autoriteten på samme måte - bena
        // hekter og propellene kan ikke redde stabiliteten, se GROUND_SCRAPE_SPEED.
        let controlAuthority = 1;
        if (groundContactBlend > 0) {
            const bodyUpY = new THREE.Vector3(0, 1, 0).applyQuaternion(droneState.quaternion).y; // cos(tilt)
            const tippedT = clamp((GROUNDED_AUTHORITY_TILT_START - bodyUpY) /
                (GROUNDED_AUTHORITY_TILT_START - GROUNDED_AUTHORITY_TILT_END), 0, 1);
            const groundSpeed = Math.hypot(droneState.velocity.x, droneState.velocity.z);
            const scrapeT = clamp((groundSpeed - GROUND_SCRAPE_SPEED) / 3.5, 0, 1);
            const worstT = Math.max(tippedT, scrapeT) * groundContactBlend;
            controlAuthority = 1 - worstT * (1 - GROUNDED_TIPPED_AUTHORITY);
        }

        // Vinkelakselerasjon = moment / treghet: tyngre/større droner (høyere treghet) responderer
        // tregere per akse, og yaw har alltid høyere treghet enn roll/pitch (som på en ekte quad).
        const pitchAccel = extractMixedAxis(motorValues, "pitch") * pitchNorm / spec.inertiaRollPitch;
        const rollAccel = extractMixedAxis(motorValues, "roll") * rollNorm / spec.inertiaRollPitch;
        const yawAccel = extractMixedAxis(motorValues, "yaw") * yawNorm / spec.inertiaYaw;
        droneState.angularVelocity.pitch += pitchAccel * controlAuthority * dt;
        droneState.angularVelocity.roll += rollAccel * controlAuthority * dt;
        droneState.angularVelocity.yaw += yawAccel * controlAuthority * dt;

        // Vortex ring state: rask nedstigning med lav horisontalfart lar propellene synke ned i sin
        // egen nedvask og miste effektiv trekkraft - se VRS-konstantene lenger oppe.
        thrustForce *= computeVrsThrustFactor(droneState.velocity);
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
    // Luftmotstand i to ledd (se kommentaren ved DRONE_CLASSES): lineært ledd bremser lavfarts-drift,
    // kvadratisk ledd gir "veggen" nær toppfart. Anisotropisk: vertikal (verdens-Y) bevegelse møter mer
    // motstand enn horisontal (VERTICAL_DRAG_MULTIPLIER) - propellskivene bremser opp-/nedgang kraftigst.
    const dragCoeffX = spec.dragLinear + spec.dragQuad * Math.abs(airRelativeVelocity.x);
    const dragCoeffY = (spec.dragLinear + spec.dragQuad * Math.abs(airRelativeVelocity.y)) * VERTICAL_DRAG_MULTIPLIER;
    const dragCoeffZ = spec.dragLinear + spec.dragQuad * Math.abs(airRelativeVelocity.z);
    const dragVec = new THREE.Vector3(
        -dragCoeffX * airRelativeVelocity.x,
        -dragCoeffY * airRelativeVelocity.y,
        -dragCoeffZ * airRelativeVelocity.z
    );
    const accel = new THREE.Vector3().add(thrustVec).add(gravityVec).add(dragVec).multiplyScalar(1 / spec.mass);

    droneState.velocity.add(accel.clone().multiplyScalar(dt));
    droneState.position.add(droneState.velocity.clone().multiplyScalar(dt));
    // Farten FØR kollisjonshåndteringen nuller komponenter - brukes som treffart for propellskade.
    const impactVelocity = droneState.velocity.clone();
    pushOutOfSolidWalls(droneState.position, droneState.velocity);

    const angVelVec = new THREE.Vector3(droneState.angularVelocity.pitch, droneState.angularVelocity.yaw, droneState.angularVelocity.roll);
    integrateOrientation(droneState.quaternion, angVelVec, dt);

    droneState.grounded = false;
    // Felles flerpunkts-bakkekontakt for alle droneklasser (ben-føtter eller ramme-undersider PLUSS
    // motor-topper) - se resolveGroundContact. Tidligere brukte racing kun ett senterpunkt og de
    // andre kun føttene, som lot kroppen synke gjennom bakken når droneen lå på siden/opp ned.
    resolveGroundContact(dt, wasGrounded);
    groundContactBlend = droneState.grounded
        ? 1
        : Math.max(0, groundContactBlend - dt / GROUND_CONTACT_BLEND_DECAY);

    updatePropStrikes(dt, impactVelocity);
    updatePilotCollision();
}

function droneHasLandingLegs(classKey) {
    return classKey !== "racing";
}

// Kontaktpunkter i drone-lokale koordinater: indeks 0-3 er UNDERSIDEN (ben-føtter for klasser med ben,
// ramme-/motor-undersider for racing), indeks 4-7 er motor-TOPPENE. Toppene gjør at droneen hviler på
// noe reelt også når den ligger på siden eller opp ned - uten dem sank kroppen gjennom bakken til
// føttene (som da peker til værs) til slutt tok imot den.
// Rekkefølge innen hver firergruppe: 0=fremre-høyre, 1=fremre-venstre, 2=bakre-høyre, 3=bakre-venstre.
function getContactLocalPoints(classKey) {
    const armTops = getLegTopLocalPositions(DRONE_ARM_LENGTH);
    const pts = droneHasLandingLegs(classKey)
        ? getLegFootLocalPositions(legLengthForClass(classKey), DRONE_ARM_LENGTH)
        : armTops.map(function (t) { return new THREE.Vector3(t.x, -0.03, t.z); });
    armTops.forEach(function (t) { pts.push(new THREE.Vector3(t.x, 0.06, t.z)); });
    return pts;
}

function getContactWorldPoints() {
    const spec = currentDroneSpec();
    return getContactLocalPoints(droneState.droneClass).map(function (p) {
        return p.clone().multiplyScalar(spec.visualScale)
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
// Hvor raskt "bakkekontakt-tilstanden" dør ut etter siste berøring. Kontakten under skimming er
// intermittent (oppdyttet frigjør benet noen ticks av gangen) - uten denne utfasingen flimret
// autoritets-reduksjonen av/på og stabiliseringen rakk å redde droneen i de frie tickene.
const GROUND_CONTACT_BLEND_DECAY = 0.3; // s

// Friksjon mot bakken/bena: konstant retardasjon (Coulomb-friksjon, ikke en hastighetsavhengig
// prosentandel per tick) - gir en naturlig, gradvis oppbremsing i stedet for at horisontalfarten
// forsvinner nesten momentant ved berøring.
const GROUND_FRICTION_DECEL = 9; // m/s^2
// Friksjonen ved bena virker et stykke under massesenteret - flyr droneen sidelengs/forlengs inn i
// bakken bremses "føttene" opp av friksjonen mens kroppen fortsetter på momentum, akkurat som en
// koffert med hjul som stopper brått og tipper forover. Dette gir et velte-moment i bevegelsesretningen
// (proporsjonalt med farten ved berøring), IKKE bare et fartstap - i tillegg til den eksisterende
// unsupported-side-veltingen. Er farten/momentet stort nok dytter dette helningen forbi
// LEG_TIP_RECOVERY_ANGLE_DEG før "oppretting" under rekker å motvirke det, og droneen flipper faktisk
// over i den retningen den fløy, i stedet for å bare stoppe brått.
const GROUND_TIP_TORQUE_GAIN = 2.5; // vinkelakselerasjon (rad/s^2) per (m/s) horisontal fart ved bakkekontakt

// Bremser horisontalfarten med konstant retardasjon (uavhengig av droneklasse - land-med-ben/skids gir
// omtrent samme friksjon), brukt av både den bein-baserte og den enkle (racing) bakkekontakten.
function applyGroundFriction(dt) {
    const horizSpeed = Math.hypot(droneState.velocity.x, droneState.velocity.z);
    if (horizSpeed <= 0) return;
    const decelFraction = Math.min(horizSpeed, GROUND_FRICTION_DECEL * dt) / horizSpeed;
    droneState.velocity.x *= (1 - decelFraction);
    droneState.velocity.z *= (1 - decelFraction);
}

// Velte-moment fra horisontalt momentum ved bakkekontakt - se GROUND_TIP_TORQUE_GAIN over. Bruker kun
// gir (yaw) for å finne kropps-fram/høyre-retning, ikke gjeldende helning - en bevisst forenkling som
// matcher resten av filens Euler-baserte tilnærming.
function applyGroundTipTorque(dt) {
    const horizSpeed = Math.hypot(droneState.velocity.x, droneState.velocity.z);
    if (horizSpeed <= 0) return;
    const yaw = new THREE.Euler().setFromQuaternion(droneState.quaternion, "YXZ").y;
    const forwardX = -Math.sin(yaw), forwardZ = -Math.cos(yaw); // kropps-forover (lokal -Z), se yaw-notatet andre steder i filen
    const rightX = Math.cos(yaw), rightZ = -Math.sin(yaw);      // kropps-høyre (lokal +X)
    const forwardSpeed = droneState.velocity.x * forwardX + droneState.velocity.z * forwardZ;
    const lateralSpeed = droneState.velocity.x * rightX + droneState.velocity.z * rightZ;
    // Samme fortegnskonvensjon som unsupported-side-veltingen over: fart fremover/til høyre skal velte
    // droneen samme vei som "front/høyre usupportert" ville gjort (nesa/høyresiden dukker først).
    droneState.angularVelocity.pitch += -GROUND_TIP_TORQUE_GAIN * forwardSpeed * dt;
    droneState.angularVelocity.roll += -GROUND_TIP_TORQUE_GAIN * lateralSpeed * dt;
}

// Tyngdekraft-velting for en drone som ligger tippet på bakken: som en flat plate på kanten sin ruller
// den ned til nærmeste stabile side - balansepunktet er på høykant (90° tilt), og momentet er størst
// der (sin-formen er den faktiske tyngdekraft-momentkurven for en plate som pivoterer på kanten).
// Verdensaksen som ØKER tilt ved positiv rotasjon er (verdens-opp x kropps-opp); den projiseres på
// kroppens X-/Z-akser for å gi pitch-/roll-rater (angularVelocity er i kroppsakser).
function applyGroundSettleTorque(dt, bodyUp, tiltDeg, horizSpeed) {
    const axis = new THREE.Vector3(0, 1, 0).cross(bodyUp);
    if (axis.lengthSq() < 1e-8) return; // helt flat (riktig vei eller på ryggen) - stabil, ingenting å rulle
    axis.normalize();
    const direction = tiltDeg > 90 ? 1 : -1; // forbi balansepunktet ruller den over på ryggen, ellers tilbake
    let mag = GROUND_SETTLE_TORQUE * Math.sin(THREE.MathUtils.degToRad(tiltDeg));
    // Glir den fortsatt i fart skal tyngdekraften IKKE "redde" den tilbake mot beina - snuble-momentet
    // fra farten skal få fullføre velten. Redningen fases inn først når glidningen har stanset.
    if (tiltDeg <= 90) mag *= clamp(1 - horizSpeed / 4, 0, 1);
    const bodyX = new THREE.Vector3(1, 0, 0).applyQuaternion(droneState.quaternion);
    const bodyZ = new THREE.Vector3(0, 0, 1).applyQuaternion(droneState.quaternion);
    droneState.angularVelocity.pitch += direction * mag * axis.dot(bodyX) * dt;
    droneState.angularVelocity.roll += direction * mag * axis.dot(bodyZ) * dt;
}

// Tyngdekraft-moment om støttepunktet: bakken bærer droneen ved kontaktpunktene, og henger ikke
// massesenteret rett over støtten, tipper tyngdekraften den rundt kontakten - som å balansere en
// blyant på tuppen. Alle bærende punkter midles til et støttesentroid, og momentet vokser med den
// horisontale avstanden h fra massesenteret dit (m·g·h, treghet om pivot via parallellakse).
// Dette er grunnen til at droneen IKKE kan stå og balansere på ett ben: for mid-klassen gir h=0.25 m
// ~20 rad/s² - langt mer enn selvnivelleringen klarer å holde imot. Godt understøttet (sentroid rett
// under massesenteret) gir h~0 og null moment - helt stabil. Erstatter den gamle konstante
// "LEG_TIP_TORQUE"-tilnærmingen, som var ~14x for svak og lot stabiliseringen vinne.
function applyGravityPivotTorque(dt, points, groundedFlags) {
    let cx = 0, cz = 0, n = 0;
    for (let i = 0; i < points.length; i++) {
        if (!groundedFlags[i]) continue;
        cx += points[i].x; cz += points[i].z; n++;
    }
    if (n === 0) return;
    const hx = droneState.position.x - cx / n;
    const hz = droneState.position.z - cz / n;
    const h = Math.hypot(hx, hz);
    if (h < 1e-4) return;
    const spec = currentDroneSpec();
    const alpha = (spec.mass * GRAVITY * h) / (spec.inertiaRollPitch + spec.mass * h * h);
    // Verdens-momentakse for tyngdekraft om pivotet: r x F med r=(hx,·,hz), F=(0,-mg,0) gir (hz,0,-hx).
    const axis = new THREE.Vector3(hz, 0, -hx).normalize();
    const bodyX = new THREE.Vector3(1, 0, 0).applyQuaternion(droneState.quaternion);
    const bodyZ = new THREE.Vector3(0, 0, 1).applyQuaternion(droneState.quaternion);
    droneState.angularVelocity.pitch += alpha * axis.dot(bodyX) * dt;
    droneState.angularVelocity.roll += alpha * axis.dot(bodyZ) * dt;
}

function resolveGroundContact(dt, wasGrounded) {
    const points = getContactWorldPoints();
    let maxPenetration = 0;
    // Når droneen har tippet langt nok kan et kontaktpunkt svinge horisontalt innunder kanten av et tak
    // (f.eks.) i stedet for å henge fritt utenfor - solidSurfaceHeightAt() ser da bare en "dyp landing"
    // der (fordi den kun kjenner en topp-flate per søyle, ingen ekte veggside), og punktet ville blitt
    // lest som støttet med et voldsomt oppløft som resultat. Fanger opp dette per punkt og dytter
    // kroppen ut av veggen i stedet, slik at punktet IKKE telles som støttet.
    let wallPush = null;
    const grounded = points.map(function (f) {
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

    // Støtte-mønster fra UNDERSIDE-punktene (0-3) - styrer kant-vipping og oppretting nær vater.
    const rightSupported = grounded[0] || grounded[2];
    const leftSupported = grounded[1] || grounded[3];
    const frontSupported = grounded[0] || grounded[1];
    const backSupported = grounded[2] || grounded[3];
    const wellSupported = rightSupported && leftSupported && frontSupported && backSupported;

    const bodyUp = new THREE.Vector3(0, 1, 0).applyQuaternion(droneState.quaternion);
    const tiltDeg = THREE.MathUtils.radToDeg(Math.acos(clamp(bodyUp.y, -1, 1)));
    const tippedPastRecovery = tiltDeg >= LEG_TIP_RECOVERY_ANGLE_DEG;

    if (!wellSupported && tippedPastRecovery && tiltDeg < 90 && maxPenetration < EDGE_PIVOT_MAX_PENETRATION) {
        // Kant-vipping: i ferd med å velte av en takkant - la tyngdekraften fullføre velten og droneen
        // falle fritt videre, i stedet for at posisjonskorreksjonen under kunstig løfter kroppen opp på
        // nytt hver eneste frame (det ga tidligere en evig vippende drone som aldri falt av kanten).
        // Betingelsen på GRUNN penetrasjon er viktig: en drone som ligger på siden/opp ned på flat bakke
        // har også "usupporterte føtter og stor tilt", men trenger posisjonskorreksjonen - uten den sank
        // kroppen gjennom bakken. Ved kant-vipping er penetrasjonen alltid nær null (punktene pivoterer
        // på selve flaten), ved en ekte kollisjon er den stor.
        applyGravityPivotTorque(dt, points, grounded);
        return;
    }

    if (!wasGrounded && droneState.velocity.y < -CRASH_SINK_RATE) {
        droneState.crashed = true;
        droneState.armed = false;
    }
    droneState.grounded = true;
    droneState.position.y += maxPenetration;
    if (droneState.velocity.y < 0) droneState.velocity.y *= 0.2; // myk/dempet landing (fjæring/struktur)
    // Horisontalfarten MÅ leses før friksjonen bremser den - styrer både snuble-momentet og skraping.
    const horizSpeed = Math.hypot(droneState.velocity.x, droneState.velocity.z);
    const scraping = horizSpeed > GROUND_SCRAPE_SPEED;
    // Momentum fra sidelengs/forlengs fart velter droneen (se GROUND_TIP_TORQUE_GAIN).
    applyGroundTipTorque(dt);
    applyGroundFriction(dt);
    // Tyngdekraft om støttesentroidet - null når godt understøttet, kraftig velting ved delvis støtte
    // (ett ben i bakken). Se applyGravityPivotTorque.
    applyGravityPivotTorque(dt, points, grounded);

    // Veltet helt rundt på bakken = krasj, uansett hvordan den kom dit (skraping som endte i velt,
    // hard skjev landing, manuell flip i lav høyde). Motor kuttes; R for å resette.
    if (tiltDeg >= OVERTURN_CRASH_TILT_DEG) {
        droneState.crashed = true;
        droneState.armed = false;
    }

    if (!tippedPastRecovery && !scraping) {
        if (wellSupported) {
            // Godt støttet, sakte og innenfor gjenopprettbar helning - understellet "retter opp" droneen.
            droneState.angularVelocity.pitch *= 0.5;
            droneState.angularVelocity.roll *= 0.5;
            const yawOnly = new THREE.Euler().setFromQuaternion(droneState.quaternion, "YXZ").y;
            const uprightQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, yawOnly, 0, "YXZ"));
            droneState.quaternion.slerp(uprightQuat, Math.min(1, LEG_CONTACT_RIGHTING_RATE * dt));
        }
        // Delvis støtte håndteres av applyGravityPivotTorque over - tyngdekraften velter den rundt
        // støttepunktet med realistisk styrke, ingen ekstra hjelpe-moment trengs.
    } else if (scraping && !tippedPastRecovery) {
        // Skraping: understellet dras langs bakken i fart. INGEN oppretting her - snuble-momentet over
        // og friksjonen får virke uimotsagt (kun lett demping), så nesa graver seg ned og den velter
        // til slutt, i stedet for at "fly langs bakken på bena" er en stabil flygestil.
        droneState.angularVelocity.pitch *= 0.95;
        droneState.angularVelocity.roll *= 0.95;
    } else {
        // Veltet forbi gjenopprettbar helning: tyngdekraften ruller den videre ned til nærmeste stabile
        // side (flatt riktig vei eller flatt på ryggen) - se applyGroundSettleTorque. Ingen pinne-redning
        // herfra (kontrollmyndigheten er også strupet, se GROUNDED_TIPPED_AUTHORITY) - disarm og reset.
        applyGroundSettleTorque(dt, bodyUp, tiltDeg, horizSpeed);
        droneState.angularVelocity.pitch *= 0.9;
        droneState.angularVelocity.roll *= 0.9;
    }
    droneState.angularVelocity.yaw *= 0.8;
}

/* ---------- Propellskade: treffdeteksjon og visuell oppdatering (se konstant-blokken øverst) ---------- */
function propPointHitsObstacle(p, nearbyHazards) {
    // Solide objekter (bakke, tak, vegger, bil): et punkt innenfor fotavtrykket under topp-flaten er
    // "inne i" objektet - én og samme test dekker både vegg-treff fra siden og bakke-/tak-treff.
    if (p.y <= solidSurfaceHeightAt(p.x, p.z) + PROP_GROUND_STRIKE_EPS) return true;
    for (let i = 0; i < nearbyHazards.length; i++) {
        const hz = nearbyHazards[i];
        const dx = p.x - hz.x, dz = p.z - hz.z;
        const localX = dx * hz.cosYaw - dz * hz.sinYaw;
        const localZ = dx * hz.sinYaw + dz * hz.cosYaw;
        for (let j = 0; j < hz.boxes.length; j++) {
            const b = hz.boxes[j];
            if (localX >= b.minX && localX <= b.maxX && p.y >= b.minY && p.y <= b.maxY &&
                localZ >= b.minZ && localZ <= b.maxZ) return true;
        }
    }
    return false;
}

function updatePropStrikes(dt, impactVelocity) {
    const spec = currentDroneSpec();
    const scale = spec.visualScale;
    const propRadius = (bladeLengthForClass(droneState.droneClass) / 2) * scale;
    // Grovsil: kun baneelementer nær droneen testes per punkt - vanligvis 0-1 stykker.
    const nearbyHazards = [];
    for (let i = 0; i < PROP_HAZARDS.length; i++) {
        const hz = PROP_HAZARDS[i];
        const dx = droneState.position.x - hz.x, dz = droneState.position.z - hz.z;
        if (dx * dx + dz * dz <= hz.boundRSq && droneState.position.y <= hz.maxY) nearbyHazards.push(hz);
    }
    // Propellskiven tilnærmes med senterpunktet + fire kantpunkter i skivens plan.
    const diskDirs = [
        new THREE.Vector3(1, 0, 0), new THREE.Vector3(-1, 0, 0),
        new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, -1)
    ].map(function (d) { return d.applyQuaternion(droneState.quaternion); });
    const armTops = getLegTopLocalPositions(DRONE_ARM_LENGTH);
    for (let i = 0; i < 4; i++) {
        const center = new THREE.Vector3(armTops[i].x, 0.05, armTops[i].z)
            .multiplyScalar(scale).applyQuaternion(droneState.quaternion).add(droneState.position);
        let hit = propPointHitsObstacle(center, nearbyHazards);
        for (let d = 0; !hit && d < diskDirs.length; d++) {
            hit = propPointHitsObstacle(center.clone().addScaledVector(diskDirs[d], propRadius), nearbyHazards);
        }
        if (hit) {
            if (!propContactActive[i]) {
                // Ny berøring: skade én gang, gradert etter treffart - hardt treff ødelegger helt.
                propContactActive[i] = true;
                addPropDamage(i, clamp(impactVelocity.length() / PROP_DESTROY_SPEED, PROP_MIN_STRIKE_DAMAGE, 1));
            } else if (droneState.armed) {
                // Vedvarende kontakt med spinnende propell (f.eks. armert drone opp ned) - slipes ned.
                addPropDamage(i, PROP_GRIND_RATE * dt);
            }
        } else {
            propContactActive[i] = false;
        }
    }
}

function addPropDamage(index, amount) {
    const before = propDamage[index];
    propDamage[index] = clamp(before + amount, 0, 1);
    if (propDamage[index] !== before) updatePropDamageVisual(index);
}

// Synlig skade: blad brekker av trinnvis med økende skade (kort stubbe står igjen), og hele propellen
// stilles skjevt/bøyd - gir en godt synlig slingring når den spinner.
function updatePropDamageVisual(index) {
    if (!dronePropellers || !dronePropellers[index]) return;
    const prop = dronePropellers[index];
    const blades = prop.mesh.userData.blades || [];
    const brokenCount = Math.min(blades.length, Math.round(propDamage[index] * blades.length));
    blades.forEach(function (pivot, j) {
        pivot.scale.x = j < brokenCount ? PROP_BROKEN_STUB_SCALE : 1;
    });
    prop.mesh.rotation.x = propDamage[index] * 0.3;
}

function repairAllProps() {
    for (let i = 0; i < propDamage.length; i++) {
        propDamage[i] = 0;
        propContactActive[i] = false;
        updatePropDamageVisual(i);
    }
}

// Treffer droneen VLOS-piloten (kroppssylinder) er det personskade, ikke et vanlig krasj: motor kuttes
// og et eget varsel med legevakt-/ambulansenummer vises. Rekkevidden inkluderer armer + propell.
function updatePilotCollision() {
    if (droneState.injured) return;
    const spec = currentDroneSpec();
    const reach = (DRONE_ARM_LENGTH + bladeLengthForClass(droneState.droneClass) / 2) * spec.visualScale;
    if (droneState.position.y > PILOT_HEIGHT + reach) return;
    const dx = droneState.position.x - PILOT_POSITION.x;
    const dz = droneState.position.z - PILOT_POSITION.z;
    if (Math.hypot(dx, dz) > PILOT_HIT_RADIUS + reach) return;
    droneState.injured = true;
    droneState.armed = false;
}

function resetDrone() {
    droneState.position.set(0, 0, 0);
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
    droneState.injured = false;
    groundContactBlend = 0;
    repairAllProps(); // reset er også "propellbytte"
    // Sett direkte i ro på avgangsplassen (samme utregning som settleDroneOnGround) - tidligere ble
    // den plassert 1 m over bakken og falt ned dit den skulle stå, synlig som et lite "hopp" ved hver
    // (re)start/øvelsesbytte i stedet for å stå klar med en gang.
    droneState.grounded = true;
    settleDroneOnGround();
}

function toggleKill() {
    if (droneState.crashed || droneState.injured) return; // må resettes (R) etter krasj/personskade
    if (exerciseState.awaitingNext) return; // fryst etter bestått øvelse - se completeExercise
    droneState.armed = !droneState.armed;
}

function toggleCamera() {
    // Øvelsene flys per definisjon fra VLOS - kamerabytte er låst mens en øvelse er aktiv.
    if (exerciseState.active) {
        exerciseState.warningMessage = "Kamera er låst til VLOS under øvelser.";
        exerciseState.warningUntil = performance.now() + 2000;
        exerciseState.warningIsSuccess = false;
        return;
    }
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
const injuryBanner = document.getElementById("injuryBanner");

function updateHud() {
    hudMode.textContent = MODE_LABELS[droneState.flightMode];
    hudArmed.textContent = droneState.injured ? "Skadet" : (droneState.crashed ? "Krasjet" : (droneState.armed ? "Armed" : "Killed"));
    hudArmed.className = "sim-status-value " + ((droneState.armed && !droneState.crashed && !droneState.injured) ? "sim-armed" : "sim-killed");
    // Personskade-varselet vinner over det vanlige krasj-varselet (droneen kan godt hard-lande ETTER
    // at den har truffet piloten - da er det fortsatt personskaden som er poenget).
    injuryBanner.classList.toggle("show", droneState.injured);
    crashBanner.classList.toggle("show", droneState.crashed && !droneState.injured);
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
const ALL_PANEL_IDS = ["ratesPanel", "droneCameraPanel", "windPanel", "gamepadPanel", "helpPanel", "exercisesPanel"];
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

/* ---------- Øvelser: kjøretøy for øvelser (tilstand, validering, veiledningsgeometri, UI) ----------
   Se EXERCISES/EXERCISE_ORDER og øvrige konstanter lenger oppe for dataene dette kjører på. */
const exerciseState = {
    active: false,
    exerciseId: null,
    stageIndex: 0,
    wpIndex: 0,
    lapsCleanCount: 0,
    lapHasViolation: false,
    attemptViolationCount: 0,
    violationActive: false, // debounce - teller kun en ny overtredelse på stigende flanke
    engaged: false, // reglene er aktive - settes når første veipunkt nås (løyper) / alltid av for hover
    headingGraceUntil: 0, // nese-sjekken hviler til dette tidspunktet (hjørne-frist, se cornerGraceSec)
    // Løpende avvik i grader mellom nesa og den sjekkede retningen (mål-yaw i "nese ut"/hover, farts-
    // retning i "nese frem") - null når ikke relevant (for lav fart i nese-frem, landing osv.). Rent
    // informativt HUD-tall, uavhengig av grace/violation-logikken - se updateExercise/updateExerciseHud.
    headingErrorDeg: null,
    landingPhase: false, // alle deløvelser fullført - venter på landing på H-plassen (se completeExercise)
    awaitingNext: false, // bestått og landet - fryst i ro (disarmet) til "Neste"/"Lukk" i oppsummeringskortet
    returnPhase: null, // "Returner hjem": null | "countdown" | "awaitThrottle" (se updateExercise)
    returnCountdownEnd: 0,
    returnRepsCompleted: 0, // "Returner hjem": antall vellykkede hjemkomster denne økten - se REQUIRED_RETURN_REPS
    returnSpawnPos: new THREE.Vector3(),
    returnSpawnQuat: new THREE.Quaternion(),
    hoverHoldSec: 0, // akkumulert tid innenfor toleransene for gjeldende hover-steg
    warningUntil: 0,
    warningMessage: "",
    warningIsSuccess: false, // styrer bannerfargen: grønn for fullført deløvelse/bestått, oransje for avvik
    startTime: 0,
    savedDroneClass: null,
    savedCameraModeIndex: 0,
    savedWind: null, // vind-innstillingene slik de var før en vind-øvelse tvang sine egne (ex9)
    savedClouds: null, // sky-innstillingene slik de var før en øvelse tvang sitt eget skydekke (ex10)
    returnFullCloudRep: 0 // "Returner hjem": hvilken runde (0-basert) som garantert får 100% skydekke
};
let exerciseGuideHandle = null;

function angularDiffDeg(aRad, bRad) {
    return THREE.MathUtils.radToDeg(Math.atan2(Math.sin(aRad - bRad), Math.cos(aRad - bRad)));
}

// VLOS-piloten står på verdens-Z-aksen og ser mot figurene - avstand LANGS synslinjen (dybde, altså
// world-Z) er dermed mye vanskeligere å bedømme presist enn sidelengs forskyvning (world-X, tvers på
// synslinjen). Treff-sonen rundt et punkt er derfor en ellipse, ikke en sirkel: samme radius sidelengs,
// men romsligere i dybden - deler dz på multiplikatoren FØR avstanden regnes, så et større faktisk
// dybdeavvik fortsatt registreres som "innenfor". Brukes kun for de flate løype-/hover-sjekkene (ikke
// sikksakkens 3D-fangst, der world-Z er konstant og dermed ikke noe dybdeproblem i utgangspunktet).
const DEPTH_TOLERANCE_MULTIPLIER = 1.8;
function horizontalCaptureDistance(dx, dz) {
    return Math.hypot(dx, dz / DEPTH_TOLERANCE_MULTIPLIER);
}
function getExerciseStage() {
    return EXERCISES[exerciseState.exerciseId].stages[exerciseState.stageIndex];
}
function formatExerciseTime(sec) {
    if (sec === null || sec === undefined) return "-";
    const mm = Math.floor(sec / 60);
    const ss = (sec % 60).toFixed(1);
    return mm + ":" + (Number(ss) < 10 ? "0" : "") + ss;
}

// Rette "stag" mellom to påfølgende veipunkt (i en lukket løkke) - samme idé som gear-strut-teknikken
// i fixed-wing-simulatoren: orienter en sylinder mellom to punkt med setFromUnitVectors. Én funksjon
// dekker alle formene (firkant, sirkel, åttetall og det vertikale sikksakk-mønsteret): veipunkt uten
// egen y-koordinat legges på fallbackY, veipunkt MED y (sikksakken) bruker sin egen.
function buildLoopStruts(waypoints, fallbackY, radius, material) {
    const group = new THREE.Group();
    const n = waypoints.length;
    for (let i = 0; i < n; i++) {
        const a = waypoints[i], b = waypoints[(i + 1) % n];
        const pA = new THREE.Vector3(a.x, a.y !== undefined ? a.y : fallbackY, a.z);
        const pB = new THREE.Vector3(b.x, b.y !== undefined ? b.y : fallbackY, b.z);
        const dir = new THREE.Vector3().subVectors(pB, pA);
        const len = dir.length();
        if (len < 1e-6) continue;
        const strut = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, len, 8), material);
        strut.position.copy(pA).addScaledVector(dir, 0.5);
        strut.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize());
        group.add(strut);
    }
    return group;
}
// Sikksakken skal ha 3D-form i lufta, men flat projeksjon på bakken - stripp y for bakkeversjonen.
function stripWaypointY(waypoints) {
    return waypoints.map(function (wp) { return { x: wp.x, z: wp.z }; });
}

// Bygger veiledningsgeometrien for ett steg: en gjennomsiktig 3D-løkke i flyhøyde, en flat løkke på
// bakken rett under (y>=0.05 - se lærdommen om z-fighting fra fixed-wing-simulatoren, 0.02 er for
// tynn margin), en statisk pil på bakken som viser den låste nese-retningen for "nese ut"-steg (uten
// en referanse har piloten ingenting konkret å rette nesa etter fra VLOS), og en pulserende markør på
// neste veipunkt (oppdatert i updateExerciseGuideVisual).
function buildExerciseGuide(stage) {
    const group = new THREE.Group();
    const guideMat = new THREE.MeshStandardMaterial({ color: 0x2ee6a6, transparent: true, opacity: 0.35 });
    const groundMat = new THREE.MeshStandardMaterial({ color: 0x2ee6a6, transparent: true, opacity: 0.55 });

    if (stage.type === "hover") {
        // Hover: ingen løype - en flat sirkel på bakken viser posisjonstoleransen, markøren (under)
        // pulserer på selve hover-punktet i flyhøyde.
        const ring = new THREE.Mesh(
            new THREE.RingGeometry(HOVER_POS_TOLERANCE - 0.15, HOVER_POS_TOLERANCE, 32),
            groundMat
        );
        ring.rotation.x = -Math.PI / 2;
        ring.position.set(HOVER_CENTER.x, 0.05, HOVER_CENTER.z);
        group.add(ring);
    } else if (stage.type !== "return") {
        // "Returner hjem" har ingen løype/veiledning - hele poenget er å finne hjem selv; markøren
        // (under) pulserer over H-plassen som mål.
        group.add(buildLoopStruts(stage.waypoints, EXERCISE_ALTITUDE, 0.08, guideMat));
        group.add(buildLoopStruts(stripWaypointY(stage.waypoints), 0.05, 0.06, groundMat));
    }

    // (Ingen retningspil på bakken for låst nese-retning - HUD-feltet "Nese-feil" (se updateExercise/
    // updateExerciseHud) viser det numeriske avviket direkte og gjorde pila overflødig.)

    const markerMat = new THREE.MeshStandardMaterial({ color: 0xffee55, transparent: true, opacity: 0.85 });
    const nextWaypointMarker = new THREE.Mesh(new THREE.SphereGeometry(0.4, 12, 10), markerMat);
    group.add(nextWaypointMarker);

    return { group: group, nextWaypointMarker: nextWaypointMarker };
}
function rebuildExerciseGuide() {
    if (exerciseGuideHandle) scene.remove(exerciseGuideHandle.group);
    exerciseGuideHandle = buildExerciseGuide(getExerciseStage());
    scene.add(exerciseGuideHandle.group);
}
// Kun kosmetisk per-bilde-animasjon (pulserende neste-veipunkt-markør) - selve formen bygges kun på
// nytt ved steg-/forsøksbytte (rebuildExerciseGuide), ikke hvert bilde, siden formen ikke endrer seg.
function updateExerciseGuideVisual(now) {
    if (!exerciseState.active || !exerciseGuideHandle) return;
    const stage = getExerciseStage();
    if (stage.type === "hover") {
        exerciseGuideHandle.nextWaypointMarker.position.set(HOVER_CENTER.x, HOVER_ALTITUDE, HOVER_CENTER.z);
    } else if (stage.type === "return") {
        exerciseGuideHandle.nextWaypointMarker.position.set(0, 1.0, 0); // målet: H-plassen hjemme
    } else {
        const wp = stage.waypoints[exerciseState.wpIndex];
        exerciseGuideHandle.nextWaypointMarker.position.set(wp.x, wp.y !== undefined ? wp.y : EXERCISE_ALTITUDE, wp.z);
    }
    exerciseGuideHandle.nextWaypointMarker.scale.setScalar(0.85 + Math.sin(now / 200) * 0.15);
}

function resetStageProgress() {
    exerciseState.wpIndex = 0;
    exerciseState.lapsCleanCount = 0;
    exerciseState.attemptViolationCount = 0;
    exerciseState.lapHasViolation = false;
    exerciseState.violationActive = false;
    exerciseState.engaged = false;
    exerciseState.hoverHoldSec = 0;
    exerciseState.headingGraceUntil = 0;
    exerciseState.returnRepsCompleted = 0;
    exerciseState.headingErrorDeg = null;
}

// Alle deløvelser fullført: fjern veiledningen og gå i landingsfase - selve fullføringen (tid,
// lagring, oppsummering) skjer først når droneen har landet på H-plassen (se updateExercise).
function enterLandingPhase(message) {
    exerciseState.landingPhase = true;
    // Meldingen vises i noen sekunder og forsvinner så - den skal ikke sperre sikten under hele
    // hjemflygingen (spesielt "Returner hjem", der turen hjem tar god tid). HUD-linja "Land på H"
    // står igjen som varig påminnelse.
    exerciseState.warningMessage = message || "Alle deløvelser fullført! Land på landingsplassen (H).";
    exerciseState.warningUntil = performance.now() + 6000;
    exerciseState.warningIsSuccess = true;
    if (exerciseGuideHandle) {
        scene.remove(exerciseGuideHandle.group);
        exerciseGuideHandle = null;
    }
}

function allExercisesPassed() {
    return EXERCISE_ORDER.every(function (id) {
        return exerciseProgress[id] && exerciseProgress[id].passed;
    });
}

// Kalles ved landing på H-plassen i landingsfasen: stopp klokka, lagre bestått/bestetid (synlig i
// øvelsesmenyen), og vis oppsummeringskortet med tid, ros og valg om å gå videre til neste øvelse.
function completeExercise() {
    const elapsedSec = (performance.now() - exerciseState.startTime) / 1000;
    const exerciseId = exerciseState.exerciseId;
    const progress = exerciseProgress[exerciseId];
    const wasAllPassedBefore = allExercisesPassed();
    const isNewBest = progress.bestTimeSec === null || elapsedSec < progress.bestTimeSec;
    const isNearBest = !isNewBest && elapsedSec < progress.bestTimeSec * 1.2;
    progress.passed = true;
    if (isNewBest) progress.bestTimeSec = elapsedSec;
    saveExerciseProgress();
    // Fryser droneen akkurat der den landet, i samme klasse/kamera den ble flydd i - stopExercise()
    // (som ville byttet klasse/kamera tilbake til brukerens vanlige valg) kalles bevisst IKKE her,
    // først når "Neste"/"Lukk" trykkes (se showExerciseSummary). Disarmet: ikke mulig å fly videre.
    exerciseState.landingPhase = false;
    exerciseState.awaitingNext = true;
    droneState.armed = false;
    renderExerciseList();
    // Diplomet dukker automatisk opp første gang ALLE øvelser er bestått (overgangen fra "ikke alle" -
    // ikke ved hver etterfølgende omkjøring av en allerede bestått øvelse - se showExerciseSummary).
    const justCompletedAll = !wasAllPassedBefore && allExercisesPassed();
    showExerciseSummary(exerciseId, elapsedSec, isNewBest, isNearBest, justCompletedAll);
}

function showExerciseSummary(exerciseId, elapsedSec, isNewBest, isNearBest, justCompletedAll) {
    const summary = document.getElementById("exerciseSummary");
    const title = document.getElementById("exerciseSummaryTitle");
    const text = document.getElementById("exerciseSummaryText");
    const nextBtn = document.getElementById("exerciseNextBtn");
    const closeBtn = document.getElementById("exerciseSummaryCloseBtn");

    title.textContent = justCompletedAll ? "Gratulerer - alle øvelser bestått!"
        : (isNewBest ? "Gratulerer - ny bestetid!" : (isNearBest ? "Bra jobba!" : "Øvelse fullført"));
    let line = EXERCISES[exerciseId].label + " bestått på " + formatExerciseTime(elapsedSec) + ".";
    if (!isNewBest) line += " Beste tid: " + formatExerciseTime(exerciseProgress[exerciseId].bestTimeSec) + ".";
    if (justCompletedAll) line += " Du har nå fullført samtlige øvelser - diplomet ditt venter!";
    text.textContent = line;

    if (justCompletedAll) {
        // Ferdig med alt - "Neste" gir ikke lenger mening som hovedvalg, lede rett til diplomet i stedet.
        nextBtn.style.display = "none";
        closeBtn.textContent = "Se diplomet";
        closeBtn.onclick = function () {
            summary.style.display = "none";
            stopExercise();
            openDiploma();
        };
    } else {
        closeBtn.textContent = "Lukk";
        const nextId = EXERCISE_ORDER[EXERCISE_ORDER.indexOf(exerciseId) + 1];
        if (nextId) {
            nextBtn.style.display = "";
            nextBtn.textContent = "Neste: " + EXERCISES[nextId].label;
            nextBtn.onclick = function () {
                summary.style.display = "none";
                // startExercise() kaller stopExercise() (gjenoppretter klasse/kamera) internt før den nye
                // øvelsen settes opp - droneen spawnes på avgangsplassen, ikke der forrige ble stående.
                startExercise(nextId);
            };
        } else {
            nextBtn.style.display = "none";
        }
        closeBtn.onclick = function () {
            summary.style.display = "none";
            stopExercise(); // gjenoppretter brukerens vanlige droneklasse/kamera og re-armer
        };
    }
    summary.style.display = "";
}

function advanceExerciseStage() {
    exerciseState.stageIndex++;
    resetStageProgress();
    const exercise = EXERCISES[exerciseState.exerciseId];
    if (exerciseState.stageIndex >= exercise.stages.length) {
        enterLandingPhase();
        return;
    }
    // Popup i 5 sekunder: kvitter for fullført deløvelse og forteller hva som er neste oppgave.
    exerciseState.warningMessage = "Fullført! Neste: " + exercise.stages[exerciseState.stageIndex].label;
    exerciseState.warningUntil = performance.now() + 5000;
    exerciseState.warningIsSuccess = true;
    rebuildExerciseGuide();
}

// Kalles rett etter den faste fysikk-løkka i animate() (se lenger ned) - droneState.position/
// quaternion reflekterer da nettopp integrert fysikk for dette bildet.
function updateExercise(dt, now) {
    if (!exerciseState.active || exerciseState.awaitingNext) return; // fryst etter bestått - se completeExercise

    // Landingsfase: deløvelsene er unnagjort - klokka går til droneen står trygt på H-plassen.
    // (Banner-meldingen ble satt én gang i enterLandingPhase og fases ut av seg selv - kun
    // landings-deteksjonen kjører her.)
    if (exerciseState.landingPhase) {
        exerciseState.headingErrorDeg = null; // ingen nese-krav under landing
        const horizSpeed = Math.hypot(droneState.velocity.x, droneState.velocity.z);
        const onPad = droneState.grounded && !droneState.crashed && horizSpeed < 0.5 &&
            Math.hypot(droneState.position.x, droneState.position.z) <= LANDING_PAD_RADIUS;
        if (onPad) {
            const exercise = EXERCISES[exerciseState.exerciseId];
            const isReturnExercise = exercise.stages[0].type === "return";
            if (isReturnExercise) exerciseState.returnRepsCompleted++;
            if (isReturnExercise && exerciseState.returnRepsCompleted < REQUIRED_RETURN_REPS) {
                // Ikke siste gjennomføring ennå - respawn med ny tilfeldig posisjon/retning og fortsett.
                // Klokka fortsetter å gå (samme startTime) - totaltiden for alle rundene lagres til slutt.
                exerciseState.landingPhase = false;
                spawnForExercise(exercise);
                exerciseState.warningMessage = "Runde " + exerciseState.returnRepsCompleted + "/" +
                    REQUIRED_RETURN_REPS + " fullført! Ny posisjon klargjort...";
                exerciseState.warningUntil = now + 3000;
                exerciseState.warningIsSuccess = true;
            } else {
                completeExercise();
            }
        }
        return;
    }

    const stage = getExerciseStage();

    // "Returner hjem": droneen holdes fastfrosset i lufta gjennom nedtelling + gass-matching, og
    // slippes først når piloten har lagt gassen rundt hover - deretter er det ren landingsfase.
    if (stage.type === "return") {
        exerciseState.headingErrorDeg = null; // ingen nese-krav mens dronen henger/venter på overtakelse
        if (exerciseState.returnPhase) {
            droneState.position.copy(exerciseState.returnSpawnPos);
            droneState.velocity.set(0, 0, 0);
            droneState.quaternion.copy(exerciseState.returnSpawnQuat);
            droneState.angularVelocity.pitch = 0;
            droneState.angularVelocity.yaw = 0;
            droneState.angularVelocity.roll = 0;
            if (exerciseState.returnPhase === "countdown") {
                const remainingSec = Math.ceil((exerciseState.returnCountdownEnd - now) / 1000);
                if (remainingSec > 0) {
                    exerciseState.warningMessage = "Dronen er langt hjemmefra! Du overtar om " + remainingSec + "...";
                    exerciseState.warningUntil = now + 500;
                    exerciseState.warningIsSuccess = false;
                } else {
                    exerciseState.returnPhase = "awaitThrottle";
                }
            }
            if (exerciseState.returnPhase === "awaitThrottle") {
                const throttle = inputState.stick.throttle;
                if (throttle >= 0.4 && throttle <= 0.65) {
                    exerciseState.returnPhase = null;
                    enterLandingPhase("Du har kontrollen! Fly dronen trygt hjem og land på landingsplassen (H).");
                } else {
                    exerciseState.warningMessage = "Ta kontroll: legg gassen rundt 50 % (nå " +
                        Math.round(throttle * 100) + " %)";
                    exerciseState.warningUntil = now + 500;
                    exerciseState.warningIsSuccess = false;
                }
            }
        }
        return;
    }

    const euler = new THREE.Euler().setFromQuaternion(droneState.quaternion, "YXZ");
    const currentYaw = euler.y;

    // Hover-steg: ingen løype/runder - hold posisjon, høyde og retning sammenhengende i holdSec
    // sekunder. Drift ut av toleransene nullstiller bare holde-tiden (mildere enn avviks-maskineriet
    // for løypene - hover er en ren teknikkovelse, ikke en avviksdrill). Ingen egen engage-gate:
    // logikken er selv-gatende (tiden teller kun innenfor toleransene).
    if (stage.type === "hover") {
        const posOk = horizontalCaptureDistance(droneState.position.x - HOVER_CENTER.x,
            droneState.position.z - HOVER_CENTER.z) <= HOVER_POS_TOLERANCE;
        const altOk = Math.abs(droneState.position.y - HOVER_ALTITUDE) <= ALTITUDE_TOLERANCE;
        exerciseState.headingErrorDeg = Math.abs(angularDiffDeg(currentYaw, stage.headingYaw));
        const headingOk = exerciseState.headingErrorDeg <= HEADING_TOLERANCE_DEG;
        if (posOk && altOk && headingOk) {
            exerciseState.hoverHoldSec += dt;
            if (exerciseState.hoverHoldSec >= stage.holdSec) advanceExerciseStage();
        } else {
            exerciseState.hoverHoldSec = 0;
        }
        return;
    }

    // Sikksakk-steget har veipunkt med egne y-koordinater: der styrer selve formen høyden (fanges i
    // 3D under), og den faste høydesjekken skrus av.
    const is3d = stage.waypoints[0].y !== undefined;

    // Løype-steg: reglene (høyde/nese) gjelder først fra det øyeblikket FØRSTE veipunkt er nådd -
    // hele transitten fra avgangsplassen frem til startpunktet (den pulserende markøren) er fri
    // flyging. Å nå startpunktet krever riktig høyde, så runden alltid begynner "innenfor reglene".
    // Fangst-radius: per steg (firkant-hjørner er trange mål), ellers 3D- eller standardradius.
    const captureRadius = stage.captureRadius || (is3d ? WAYPOINT_CAPTURE_RADIUS_3D : WAYPOINT_CAPTURE_RADIUS);

    if (!exerciseState.engaged) {
        const wp0 = stage.waypoints[0];
        const atStart = is3d
            ? Math.hypot(droneState.position.x - wp0.x, droneState.position.y - wp0.y, droneState.position.z - wp0.z) < captureRadius
            : (horizontalCaptureDistance(droneState.position.x - wp0.x, droneState.position.z - wp0.z) < captureRadius &&
                Math.abs(droneState.position.y - EXERCISE_ALTITUDE) <= ALTITUDE_TOLERANCE);
        if (!atStart) return;
        exerciseState.engaged = true;
        exerciseState.wpIndex = 1; // startpunktet er tatt - runden er i gang, neste mål er punkt 2
        // Startpunktet er også et "hjørne" - gi samme frist til å svinge inn på første kant.
        if (stage.cornerGraceSec) exerciseState.headingGraceUntil = now + stage.cornerGraceSec * 1000;
    }

    // headingErrorDeg oppdateres uavhengig av grace/violation-logikken under - rent informativt HUD-tall
    // (se "Nese-feil" i HUD-baren) slik at piloten kan se nøyaktig hvor mange grader unna man er i
    // stedet for bare et binært "avvik"-varsel etterpå.
    let headingViolation = false;
    if (stage.noseMode === "free") {
        // Ingen nese-krav i det hele tatt (se ex9 "Åttetall i vind") - headingViolation forblir false
        // og HUD-feltet "Nese-feil" viser "-" (ikke relevant her).
        exerciseState.headingErrorDeg = null;
    } else if (stage.noseMode === "out") {
        exerciseState.headingErrorDeg = Math.abs(angularDiffDeg(currentYaw, LOCKED_HEADING));
        headingViolation = exerciseState.headingErrorDeg > HEADING_TOLERANCE_DEG;
    } else {
        const horizSpeed = Math.hypot(droneState.velocity.x, droneState.velocity.z);
        if (horizSpeed > MIN_SPEED_FOR_HEADING_CHECK) {
            // -Z er forover (se stepPhysics/updateChaseCamera) - samme targetYaw-formel som nesa "ut"
            // ellers ville brukt, bare med fartsretningen i stedet for en fast retning.
            const targetYaw = Math.atan2(-droneState.velocity.x, -droneState.velocity.z);
            exerciseState.headingErrorDeg = Math.abs(angularDiffDeg(currentYaw, targetYaw));
            if (now >= exerciseState.headingGraceUntil) headingViolation = exerciseState.headingErrorDeg > HEADING_TOLERANCE_DEG;
        } else {
            exerciseState.headingErrorDeg = null; // for lav fart til at fartsretningen er meningsfull
        }
    }
    const altitudeViolation = !is3d && Math.abs(droneState.position.y - EXERCISE_ALTITUDE) > ALTITUDE_TOLERANCE;
    const violationNow = headingViolation || altitudeViolation;

    if (violationNow) {
        if (!exerciseState.violationActive) {
            exerciseState.violationActive = true;
            exerciseState.attemptViolationCount++;
            exerciseState.lapHasViolation = true;
            if (exerciseState.attemptViolationCount === 1) {
                exerciseState.warningMessage = altitudeViolation ? "Høydeavvik! Advarsel." : "Feil nese-retning! Advarsel.";
                exerciseState.warningUntil = now + 2500;
                exerciseState.warningIsSuccess = false;
            } else {
                exerciseState.warningMessage = "For mange avvik - steget nullstilt.";
                exerciseState.warningUntil = now + 2500;
                exerciseState.warningIsSuccess = false;
                resetStageProgress();
                resetDrone();
                return; // droneen ble nettopp resatt - hopp over vegpunkt-sjekk for dette bildet
            }
        }
    } else {
        exerciseState.violationActive = false;
    }

    const wp = stage.waypoints[exerciseState.wpIndex];
    const captured = is3d
        ? Math.hypot(droneState.position.x - wp.x, droneState.position.y - wp.y, droneState.position.z - wp.z) < captureRadius
        : horizontalCaptureDistance(droneState.position.x - wp.x, droneState.position.z - wp.z) < captureRadius;
    if (captured) {
        // Hvert veipunkt-treff starter hjørne-fristen på nytt (kun satt for steg med cornerGraceSec).
        if (stage.cornerGraceSec) exerciseState.headingGraceUntil = now + stage.cornerGraceSec * 1000;
        exerciseState.wpIndex++;
        if (exerciseState.wpIndex >= stage.waypoints.length) {
            exerciseState.wpIndex = 0;
            if (!exerciseState.lapHasViolation) exerciseState.lapsCleanCount++;
            exerciseState.lapHasViolation = false;
            if (exerciseState.lapsCleanCount >= (stage.requiredCleanLaps || REQUIRED_CLEAN_LAPS)) advanceExerciseStage();
        }
    }
}

function updateExerciseHud() {
    const banner = document.getElementById("exerciseWarningBanner");
    const bannerText = document.getElementById("exerciseWarningText");
    const showBanner = performance.now() < exerciseState.warningUntil;
    banner.classList.toggle("show", showBanner);
    banner.classList.toggle("sim-banner-success", exerciseState.warningIsSuccess);
    if (showBanner) bannerText.textContent = exerciseState.warningMessage;

    const hudBar = document.getElementById("exerciseHudBar");
    if (!exerciseState.active) {
        hudBar.style.display = "none";
        return;
    }
    hudBar.style.display = "";
    // "Returner hjem" har ett fast steg (stageIndex alltid 0) - stegobjektet er gyldig gjennom hele
    // landingsfasen der også, i motsetning til vanlige flerstegs-øvelser (der landingPhase betyr
    // stageIndex har passert siste steg og getExerciseStage() ville returnert undefined). awaitingNext
    // (bestått og fryst, se completeExercise) betyr ALLTID at stegobjektet ikke lenger er gyldig å lese.
    const isReturnExercise = EXERCISES[exerciseState.exerciseId].stages[0].type === "return";
    const stage = (exerciseState.awaitingNext || (exerciseState.landingPhase && !isReturnExercise))
        ? null : getExerciseStage();
    document.getElementById("exerciseHudStage").textContent =
        stage ? stage.label : (exerciseState.awaitingNext ? "Fullført!" : "Landing");
    // Merk runden som "teller ikke" så snart et avvik har skjedd i den - synlig konsekvens med en gang.
    const lapSuffix = (stage && exerciseState.lapHasViolation) ? " (runden teller ikke)" : "";
    const returnSuffix = exerciseState.landingPhase ? " - land på H" : "";
    document.getElementById("exerciseHudLaps").textContent = !stage
        ? (exerciseState.awaitingNext ? "Se oppsummering" : "Land på H")
        : (stage.type === "hover"
            ? exerciseState.hoverHoldSec.toFixed(1) + "/" + stage.holdSec + " s"
            : (stage.type === "return"
                ? exerciseState.returnRepsCompleted + "/" + REQUIRED_RETURN_REPS + returnSuffix
                : exerciseState.lapsCleanCount + "/" + (stage.requiredCleanLaps || REQUIRED_CLEAN_LAPS) + lapSuffix));

    // Avviks-status: tydelig, vedvarende indikator på hvor nær steget er å bli nullstilt - banneret
    // alene forsvinner etter noen sekunder og etterlot ingen synlig "du har brukt opp advarselen".
    const violationsEl = document.getElementById("exerciseHudViolations");
    if (!stage || stage.type === "hover" || stage.type === "return") {
        violationsEl.textContent = "-";
        violationsEl.className = "sim-status-value";
    } else if (exerciseState.attemptViolationCount === 0) {
        violationsEl.textContent = "Ingen";
        violationsEl.className = "sim-status-value sim-armed";
    } else {
        violationsEl.textContent = "1 - neste nullstiller!";
        violationsEl.className = "sim-status-value sim-killed";
    }

    // Løpende nese-avvik i grader - se headingErrorDeg-kommentaren i updateExercise. Svarer direkte på
    // "hvilken retning sjekkes egentlig akkurat nå" i stedet for å bare oppdage det som et varsel etterpå.
    const headingErrEl = document.getElementById("exerciseHudHeadingError");
    if (exerciseState.headingErrorDeg === null) {
        headingErrEl.textContent = "-";
        headingErrEl.className = "sim-status-value";
    } else {
        headingErrEl.textContent = Math.round(exerciseState.headingErrorDeg) + "°";
        headingErrEl.className = "sim-status-value " +
            (exerciseState.headingErrorDeg <= HEADING_TOLERANCE_DEG ? "sim-armed" : "sim-killed");
    }

    const elapsed = (performance.now() - exerciseState.startTime) / 1000;
    const mm = Math.floor(elapsed / 60);
    const ss = Math.floor(elapsed % 60);
    document.getElementById("exerciseHudTimer").textContent = mm + ":" + (ss < 10 ? "0" : "") + ss;
}

// Plasserer droneen for (om)start av en øvelse: vanlige øvelser starter på avgangsplassen (via
// resetDrone), "Returner hjem" spawner høyt og langt unna med tilfeldig yaw og starter nedtellingen.
function spawnForExercise(exercise) {
    resetDrone();
    exerciseState.returnPhase = null;
    if (exercise.spawn === "far") {
        const bearing = (Math.random() * 2 - 1) * (Math.PI / 3); // innenfor ±60° av rett frem (-Z)
        const dist = 130 + Math.random() * 40;
        exerciseState.returnSpawnPos.set(Math.sin(bearing) * dist, 35 + Math.random() * 10, -Math.cos(bearing) * dist);
        exerciseState.returnSpawnQuat.setFromEuler(new THREE.Euler(0, Math.random() * Math.PI * 2, 0, "YXZ"));
        droneState.position.copy(exerciseState.returnSpawnPos);
        droneState.quaternion.copy(exerciseState.returnSpawnQuat);
        exerciseState.returnPhase = "countdown";
        exerciseState.returnCountdownEnd = performance.now() + 4000;
    }
    // Ny tilfeldig vindretning ved HVER spawn (første start, hver runde og hver R-restart) - ikke bare
    // ved øvelsesstart. Kjøres etter startExercise sin faste vind-tvang (se der), så denne vinner.
    if (exercise.wind && exercise.randomizeWindDirection) {
        settings.wind.directionDeg = Math.floor(Math.random() * 360);
    }
    // Tilfeldig skydekke per runde, men minst én av de REQUIRED_RETURN_REPS rundene garantert 100% -
    // hvilken runde det blir avgjøres på nytt for hver ferske økt (returnRepsCompleted === 0, altså
    // helt i starten av startExercise ELLER rett etter en R-restart, ikke ved vanlig runde-fremgang).
    if (exercise.randomizeCloudCoverage) {
        if (exerciseState.returnRepsCompleted === 0) {
            exerciseState.returnFullCloudRep = Math.floor(Math.random() * REQUIRED_RETURN_REPS);
        }
        settings.cloudsEnabled = true;
        settings.cloudCoverage = (exerciseState.returnRepsCompleted === exerciseState.returnFullCloudRep)
            ? 1
            : 0.3 + Math.random() * 0.6;
    }
}

function startExercise(id) {
    stopExercise();
    const exercise = EXERCISES[id];
    if (!exercise) return;
    exerciseState.savedDroneClass = droneState.droneClass;
    exerciseState.savedCameraModeIndex = cameraModeIndex;
    setDroneClassEphemeral("mid");
    cameraModeIndex = CAMERA_MODES.indexOf("vlos");
    activeCamera = vlosCamera;

    // Vind-øvelser tvinger sin egen vind mens øvelsen pågår - brukerens innstillinger huskes og
    // settes tilbake i stopExercise. Ingen saveSettings her: tvangen skal aldri lekke til localStorage.
    // MÅ settes FØR spawnForExercise (rett under) - for øvelser med randomizeWindDirection er det
    // spawnForExercise sin oppgave å gi den faktiske (tilfeldige) retningen, denne blokka setter bare
    // resten av vind-parametrene (styrke/kast/på) og en forhåndsvalgt retning for de øvrige.
    exerciseState.savedWind = {
        enabled: settings.wind.enabled, speed: settings.wind.speed,
        directionDeg: settings.wind.directionDeg, gust: settings.wind.gust
    };
    if (exercise.wind) {
        settings.wind.enabled = true;
        settings.wind.speed = exercise.wind.speed;
        settings.wind.directionDeg = exercise.wind.directionDeg;
        settings.wind.gust = exercise.wind.gust;
    } else {
        settings.wind.enabled = false; // øvrige øvelser flys i stille vær uansett hva brukeren hadde på
    }

    // Enkelte øvelser (ex10) tvinger sitt eget skydekke mens de pågår - samme lagre/gjenopprett-
    // mønster som vinden over. Statisk verdi settes her; PER RUNDE-randomisering (randomizeCloudCoverage)
    // håndteres i stedet i spawnForExercise, som kjøres rett under og til slutt vinner.
    exerciseState.savedClouds = { enabled: settings.cloudsEnabled, coverage: settings.cloudCoverage };
    if (exercise.cloudCoverage !== undefined && !exercise.randomizeCloudCoverage) {
        settings.cloudsEnabled = true;
        settings.cloudCoverage = exercise.cloudCoverage;
    }

    // MÅ nullstilles FØR spawnForExercise: den leser exerciseState.returnRepsCompleted for å avgjøre om
    // dette er en helt fersk økt (og i så fall trekke en ny "garantert 100% skydekke"-runde, ex10) -
    // uten denne rekkefølgen ville den første spawnen i en ny økt sett den GAMLE verdien fra forrige gang.
    exerciseState.stageIndex = 0;
    resetStageProgress();
    spawnForExercise(exercise);

    exerciseState.active = true;
    exerciseState.exerciseId = id;
    exerciseState.warningUntil = 0;
    exerciseState.startTime = performance.now();
    rebuildExerciseGuide();
    // Lukk menyen - piloten skal rett i gang, og panelet skygger for sikten mot treningsområdet.
    document.getElementById("exercisesPanel").style.display = "none";
}

// Idempotent opprydning - kalles både fra "Avbryt" og fra starten av startExercise (dekker "bytt til
// en annen øvelse" og "start øvelsen på nytt" med én kodesti). Teleporterer IKKE droneen tilbake til
// plattformen - respekterer det brukeren holder på med akkurat da.
function stopExercise() {
    if (!exerciseState.active) return;
    setDroneClassEphemeral(exerciseState.savedDroneClass);
    cameraModeIndex = exerciseState.savedCameraModeIndex;
    const mode = CAMERA_MODES[cameraModeIndex];
    activeCamera = (mode === "chase") ? chaseCamera : (mode === "fpv") ? fpvCamera : vlosCamera;
    if (exerciseState.savedClouds) {
        settings.cloudsEnabled = exerciseState.savedClouds.enabled;
        settings.cloudCoverage = exerciseState.savedClouds.coverage;
        exerciseState.savedClouds = null;
    }
    if (exerciseState.savedWind) {
        settings.wind.enabled = exerciseState.savedWind.enabled;
        settings.wind.speed = exerciseState.savedWind.speed;
        settings.wind.directionDeg = exerciseState.savedWind.directionDeg;
        settings.wind.gust = exerciseState.savedWind.gust;
        exerciseState.savedWind = null;
    }
    if (exerciseGuideHandle) {
        scene.remove(exerciseGuideHandle.group);
        exerciseGuideHandle = null;
    }
    // Re-armer: den eneste grunnen droneState.armed kan stå false her er completeExercise()s frys
    // (awaitingNext) - i alle andre tilfeller (Avbryt midt i flukt) er den alt armert, så dette er en
    // no-op da og en nødvendig gjenoppretting her.
    droneState.armed = true;
    exerciseState.active = false;
    exerciseState.exerciseId = null;
    exerciseState.landingPhase = false;
    exerciseState.awaitingNext = false;
}

// Manuell R/reset-knapp: starter HELE øvelsen på nytt fra steg 0 og nullstiller stoppeklokken - i
// motsetning til det automatiske steg-nullstillet ved 2. avvik (som beholder tidligere bestått steg-
// fremgang OG lar klokken gå videre, som en implisitt tidsstraff for restarts).
function handleResetRequest() {
    if (!exerciseState.active) {
        resetDrone();
        return;
    }
    // Gyldig også rett etter en bestått (awaitingNext) - full omkamp i stedet for å måtte gå via
    // oppsummeringskortet. Skjul kortet hvis det står åpent (fra en tidligere fullføring i samme økt
    // ville det ikke være det, men det koster ingenting å være trygg).
    document.getElementById("exerciseSummary").style.display = "none";
    exerciseState.awaitingNext = false;
    exerciseState.stageIndex = 0;
    exerciseState.landingPhase = false;
    resetStageProgress();
    exerciseState.startTime = performance.now();
    spawnForExercise(EXERCISES[exerciseState.exerciseId]); // "Returner hjem" får ny tilfeldig posisjon/retning
    rebuildExerciseGuide();
}

/* ---------- Øvelser: panel-UI (liste + detaljvisning) ---------- */
function renderExerciseList() {
    const container = document.getElementById("exerciseListItems");
    container.innerHTML = "";
    EXERCISE_ORDER.forEach(function (id) {
        const exercise = EXERCISES[id];
        const progress = exerciseProgress[id] || { passed: false, bestTimeSec: null };
        const row = document.createElement("button");
        row.type = "button";
        row.className = "sim-exercise-row";
        row.innerHTML =
            '<span class="sim-exercise-row-icon"><i class="fa-solid ' + exercise.icon + '"></i></span>' +
            '<span class="sim-exercise-row-main">' +
            '<span class="sim-exercise-row-title">' + exercise.label + "</span>" +
            '<span class="sim-exercise-row-desc">' + exercise.shortDescription + "</span>" +
            "</span>" +
            (progress.passed
                ? '<span class="sim-exercise-check"><i class="fa-solid fa-circle-check"></i> ' + formatExerciseTime(progress.bestTimeSec) + "</span>"
                : "");
        row.addEventListener("click", function () { showExerciseDetail(id); });
        container.appendChild(row);
    });

    // Alle øvelser bestått: diplom-rad nederst - klikk for å fylle ut navn og skrive ut/lagre som PDF.
    // (Diplomet dukker i tillegg automatisk opp første gang dette blir sant, se completeExercise.)
    if (allExercisesPassed()) {
        const diplomaRow = document.createElement("button");
        diplomaRow.type = "button";
        diplomaRow.className = "sim-exercise-row sim-exercise-row-diploma";
        diplomaRow.innerHTML =
            '<span class="sim-exercise-row-icon"><i class="fa-solid fa-award"></i></span>' +
            '<span class="sim-exercise-row-main">' +
            '<span class="sim-exercise-row-title">Diplom</span>' +
            '<span class="sim-exercise-row-desc">Alle øvelser bestått! Fyll ut navnet ditt og skriv ut beviset.</span>' +
            "</span>";
        diplomaRow.addEventListener("click", openDiploma);
        container.appendChild(diplomaRow);
    }
}

function openDiploma() {
    const overlay = document.getElementById("diplomaOverlay");
    // Bestetidene (fra localStorage) printes på diplomet - konkurranse-elementet: fly øvelsene så
    // mange ganger man vil for å forbedre tidene, diplomet viser alltid de gjeldende bestetidene.
    const timesEl = document.getElementById("diplomaTimes");
    timesEl.innerHTML = "";
    EXERCISE_ORDER.forEach(function (id) {
        const rowEl = document.createElement("div");
        rowEl.className = "sim-diploma-time-row";
        const nameEl = document.createElement("span");
        nameEl.textContent = EXERCISES[id].label;
        const timeEl = document.createElement("span");
        timeEl.textContent = formatExerciseTime(exerciseProgress[id].bestTimeSec);
        rowEl.appendChild(nameEl);
        rowEl.appendChild(timeEl);
        timesEl.appendChild(rowEl);
    });
    document.getElementById("diplomaDate").textContent =
        "Dato: " + new Date().toLocaleDateString("nb-NO", { year: "numeric", month: "long", day: "numeric" });
    document.getElementById("diplomaPrintBtn").onclick = function () {
        // Print-CSS-en (body.printing-diploma) skjuler alt annet enn diplom-arket under utskriften -
        // "Lagre som PDF" i nettleserens print-dialog gir PDF-beviset.
        document.body.classList.add("printing-diploma");
        window.print();
        document.body.classList.remove("printing-diploma");
    };
    document.getElementById("diplomaCloseBtn").onclick = function () {
        overlay.style.display = "none";
    };
    overlay.style.display = "";
}
function showExerciseListView() {
    document.getElementById("exerciseListView").style.display = "";
    document.getElementById("exerciseDetailView").style.display = "none";
}
function showExerciseDetail(id) {
    const exercise = EXERCISES[id];
    if (!exercise) return;
    document.getElementById("exerciseListView").style.display = "none";
    document.getElementById("exerciseDetailView").style.display = "";
    document.getElementById("exerciseDetailTitle").innerHTML =
        '<i class="fa-solid ' + exercise.icon + ' sim-exercise-icon"></i>' + exercise.label;
    document.getElementById("exerciseDetailDescription").textContent = exercise.fullDescription;

    const progressEl = document.getElementById("exerciseDetailProgress");
    const startBtn = document.getElementById("exerciseStartBtn");
    const cancelBtn = document.getElementById("exerciseCancelBtn");
    if (exerciseState.active && exerciseState.exerciseId === id) {
        progressEl.style.display = "";
        const isReturnExercise = exercise.stages[0].type === "return";
        if (exerciseState.awaitingNext) {
            // stageIndex kan stå forbi siste steg her (vanlige flerstegs-øvelser) - stegobjektet er
            // IKKE gyldig å lese, se samme resonnement i updateExerciseHud.
            progressEl.textContent = "Bestått! Se oppsummeringskortet for å gå videre.";
        } else if (exerciseState.landingPhase && !isReturnExercise) {
            progressEl.textContent = "Pågår: landing på H-plassen";
        } else if (isReturnExercise) {
            progressEl.textContent = "Pågår: runde " + (exerciseState.returnRepsCompleted + 1) + "/" +
                REQUIRED_RETURN_REPS + (exerciseState.landingPhase ? " - land på H" : "");
        } else {
            const stage = getExerciseStage();
            progressEl.textContent = "Pågår: " + stage.label + (stage.type === "hover"
                ? " - " + exerciseState.hoverHoldSec.toFixed(0) + "/" + stage.holdSec + " s"
                : " - runde " + exerciseState.lapsCleanCount + "/" + (stage.requiredCleanLaps || REQUIRED_CLEAN_LAPS));
        }
        startBtn.style.display = "none";
        cancelBtn.style.display = "";
    } else {
        const progress = exerciseProgress[id] || { passed: false, bestTimeSec: null };
        progressEl.style.display = progress.passed ? "" : "none";
        if (progress.passed) progressEl.textContent = "Bestått - beste tid: " + formatExerciseTime(progress.bestTimeSec);
        startBtn.style.display = "";
        cancelBtn.style.display = "none";
    }
    startBtn.onclick = function () { startExercise(id); };
    cancelBtn.onclick = function () { stopExercise(); showExerciseDetail(id); };
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
    updateClouds(frameDt);
    updateInput(frameDt);
    while (accumulator >= FIXED_DT) {
        stepPhysics(FIXED_DT);
        accumulator -= FIXED_DT;
    }
    updateExercise(frameDt, now);

    updateDroneVisual(frameDt);
    updateChaseCamera(frameDt);
    updateVlosCamera();
    updateWindsockVisual(now);
    updateExerciseGuideVisual(now);
    updateHud();
    updateExerciseHud();
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
    renderExerciseList();

    document.getElementById("resetBtn").addEventListener("click", handleResetRequest);
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
    document.getElementById("clearExerciseTimesBtn").addEventListener("click", function () {
        closeSettingsMenu();
        if (!window.confirm("Nullstille alle øvelsestider og bestått-status? Dette kan ikke angres.")) return;
        EXERCISE_ORDER.forEach(function (id) {
            exerciseProgress[id].passed = false;
            exerciseProgress[id].bestTimeSec = null;
        });
        saveExerciseProgress();
        renderExerciseList();
        document.getElementById("diplomaOverlay").style.display = "none";
    });
    document.getElementById("toggleHelpBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("helpPanel"));
    });
    document.getElementById("toggleExercisesBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("exercisesPanel"));
        // Vis fremgangen for en øvelse som allerede er i gang, ellers listen.
        if (exerciseState.active) showExerciseDetail(exerciseState.exerciseId);
        else showExerciseListView();
    });
    document.getElementById("exerciseBackToListBtn").addEventListener("click", showExerciseListView);
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

    const airmodeInput = document.getElementById("airmodeInput");
    airmodeInput.checked = settings.airmodeEnabled;
    airmodeInput.addEventListener("change", function () {
        settings.airmodeEnabled = airmodeInput.checked;
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

    const cloudsEnabledInput = document.getElementById("cloudsEnabledInput");
    const cloudCoverageInput = document.getElementById("cloudCoverageInput");
    const cloudCoverageValue = document.getElementById("cloudCoverageValue");
    cloudsEnabledInput.checked = settings.cloudsEnabled;
    cloudCoverageInput.value = Math.round(settings.cloudCoverage * 100);
    cloudCoverageValue.textContent = Math.round(settings.cloudCoverage * 100) + "%";
    cloudsEnabledInput.addEventListener("change", function () {
        settings.cloudsEnabled = cloudsEnabledInput.checked;
        saveSettings();
    });
    cloudCoverageInput.addEventListener("input", function () {
        settings.cloudCoverage = parseFloat(cloudCoverageInput.value) / 100;
        cloudCoverageValue.textContent = cloudCoverageInput.value + "%";
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
        // Skriving i et tekstfelt (f.eks. navnet på diplomet) skal aldri tolkes som flysimulator-
        // taster - uten denne sjekken ble mellomrom/R/H/M/1-3 osv. kapret av hurtigtastene under, og
        // mellomrom ble i tillegg preventDefault()-et og kunne dermed ALDRI skrives i et tekstfelt.
        const active = document.activeElement;
        if (active && (active.tagName === "INPUT" || active.tagName === "TEXTAREA" || active.isContentEditable)) {
            return;
        }
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
            case "KeyR": handleResetRequest(); break;
            case "KeyC": toggleCamera(); break;
            case "KeyT": togglePanel(document.getElementById("ratesPanel")); break;
            case "KeyH": togglePanel(document.getElementById("helpPanel")); break;
            case "KeyO": toggleFpvHud(); break;
            case "KeyM":
                togglePanel(document.getElementById("exercisesPanel"));
                if (exerciseState.active) showExerciseDetail(exerciseState.exerciseId);
                else showExerciseListView();
                break;
        }
    });
    window.addEventListener("keyup", function (e) {
        keys.delete(e.code);
    });

    requestAnimationFrame(animate);
});
