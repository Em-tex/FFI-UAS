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
        dragLinear: 0.2, dragQuad: 0.006, visualScale: 0.72 // kun visuell størrelse - massen er uendret
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
// Litt trangere enn PILOT_HIT_RADIUS for folkemengden/fotgjengerne spesifikt - PILOT_HIT_RADIUS sjekkes
// individuelt mot HVER av de ~7 sprikende personene i folkemengden (se CROWD_MEMBER_OFFSETS), så den
// EFFEKTIVE faresonen (unionen av alle sju sirklene) ble merkbart større/mindre forutsigbar enn å bare
// unngå den ene VLOS-piloten - opplevdes for strengt ("krasjet" med noen i utkanten man ikke så).
// Senket videre fra 0.35 - selv da trigget skade for tidlig (radiusen legges til propell-rekkevidden
// under, så totalen ble fortsatt større enn den føltes ut som å skulle være).
const BYSTANDER_HIT_RADIUS = 0.15;
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
const HOVER_POS_TOLERANCE = 1.5; // m horisontal radius rundt hover-punktet - strammere enn løype-figurenes captureRadius
const HOVER_ALTITUDE_TOLERANCE = 1; // m - egen, strengere høydetoleranse enn de øvrige øvelsenes ALTITUDE_TOLERANCE
// Hover-øvelsen har sitt eget punkt, nærmere piloten enn løype-figurene (10 m unna i stedet for 11) -
// i ro på kort hold er det lettere å lese nese-retningen på droneen.
const HOVER_CENTER = new THREE.Vector3(0, 0, -5);
// Landingsplassen (H) ved avgangspunktet - øvelsene avsluttes automatisk ved landing her.
const LANDING_PAD_RADIUS = 2.4;
// Droneen må stå i ro på padden sammenhengende i denne varigheten før øvelsen faktisk avsluttes -
// uten denne følies det brått ut idet bena så vidt rører bakken nær padden.
const LANDING_CONFIRM_SEC = 1.5;
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
// "Nese frem" avledes fra fartsvektoren, som glipper til korte, forbigående utslag i raske, kontinuerlige
// svinger (sirkel/åttetall) selv ved korrekt yaw-styring - naturlig sideveis "slip" i banen, ikke en reell
// feil. Avviket må derfor overstige toleransen sammenhengende i denne varigheten før det telles som avvik.
const HEADING_FORWARD_SLIP_GRACE_MS = 400;

/* ---------- Øvelser: uforutsette hendelser (ex11) ----------
   Fire uavhengige scenarioer - se egen "Øvelser: killswitch-tilstandsmaskin"-seksjon lenger ned for selve
   kjørelogikken. Kjernepoenget: riktig respons er IKKE alltid killswitch. "crowd" og "traffic" er ekte
   styringstap (dronen kan ikke reddes ved å styre unna) der kutt er eneste riktige svar. "heli" og
   "pedestrians" er derimot situasjoner der DRONEN fortsatt er fullt kontrollerbar - der er riktig respons
   å vike unna og/eller lande, akkurat som i en reell nærsituasjon med bemannet luftfart eller mennesker i
   flygeområdet; å kutte motorene der (og falle rett ned, ukontrollert) er feil respons.
   Verken hvilket scenario som kommer eller hva slags respons det krever varsles på forhånd - INGEN
   melding vises når faren/situasjonen inntreffer (kun EasyReport-lignende tilbakemelding i etterkant ved
   feil respons, se KILLSWITCH_MESSAGES) - brukeren må selv oppdage og vurdere situasjonen, akkurat som i
   en reell uforutsett hendelse.
   "crowd" og "traffic" er kinematisk styrt via en falsk, "spøkelses"-pinneinput som mates inn i den
   ORDINÆRE fysikken (se applyKillswitchInputOverride, kalt fra updateInput) - dronen ser dermed ut til
   faktisk å motta (helt gale) pinneutslag og kan ikke korrigeres av spillerens egen input, i stedet for å
   teleportere langs en fast bane. "heli" og "pedestrians" lar brukeren beholde full kontroll - kun selve
   faremomentet (helikopteret/fotgjengerne) er scriptet.
*/
const KILLSWITCH_TRIGGER_MIN_SEC = 4; // tilfeldig ventetid før noe inntreffer, MENS "liksom"-runden flys
const KILLSWITCH_TRIGGER_MAX_SEC = 9;
const KILLSWITCH_FAIL_PAUSE_MS = 2800; // hvor lenge feilmeldingen står før et nytt forsøk klargjøres
// Etter et vellykket kutt: la fysikken (fritt fall) spille seg ut og vise HVOR droneen faktisk lander/
// krasjer, i stedet for å hoppe rett til neste scenario midt i fallet.
const KILLSWITCH_SUCCESS_WATCH_SEC = 4;
// Kutt SENERE enn denne andelen av rømningsvarigheten (men fortsatt før selve deadline) teller ikke
// lenger som en trygg/tidsnok respons - uten denne kunne man kutte i aller siste liten, praktisk talt
// allerede inni faresonen, og fortsatt få det godkjent (se updateKillswitchStage).
const KILLSWITCH_SAFE_CUTOFF_FRACTION = 0.75;
// Vises i noen sekunder når en "vente"-fase starter (øvelsesstart, nytt steg, eller nytt forsøk etter
// feil) - forteller hva brukeren skal gjøre AKKURAT NÅ (fly den vanlige runden), uten å røpe noe om at
// noe kommer til å skje. Se spawnForExercise/advanceExerciseStage/updateKillswitchStage.
const KILLSWITCH_PATROL_HINT = "Følg ringen med nesa fremover.";

// Scenario 1: dronen mister styringen og "flyr av seg selv" mot folkemengden ved bilen (CROWD_CENTER).
const CROWD_RUNAWAY_DURATION_SEC = 3.2;
const CROWD_TARGET_ALTITUDE = 1.4; // ca. hodehøyde i mengden

// Scenario 2: et helikopter kommer plutselig lavt gjennom området mens brukeren beholder full kontroll -
// riktig respons er å øke horisontal avstand til flygebanen (vike unna) eller lande, IKKE killswitch
// (se updateHeliDangerPhase). HELI_SAFE_HORIZ_DISTANCE er bevisst horisontal, ikke 3D-avstand - å bare
// stå stille og stole på høydeforskjellen til helikopteret skal ikke telle som en gyldig unnamanøver -
// derfor må HELI_ALTITUDE ligge nær høyden "liksom"-runden FAKTISK flys i for akkurat dette steget
// (ks-heli.patrolAltitude, høyere enn standard EXERCISE_ALTITUDE=1m - se buildExerciseGuide/
// updateExerciseGuideVisual), ellers er det aldri reell nærhet i utgangspunktet uansett hva spilleren
// gjør. Realistisk løst ved å heve "liksom"-runden opp mot en troverdig helikopterhøyde, IKKE ved å
// dra helikopteret urealistisk lavt ned mot bakkenivå. Verdien er en avveining: for høyt (var 15) og
// VLOS-kameraet (som følger dronen, se updateVlosCamera) må vippe så bratt oppover for å holde den
// nære, høytflyvende droneen i bildet at et fjernt, lavere innkommende helikopter havner utenfor
// synsfeltet - for lavt og det er ingen reell høydeoverlapp i det hele tatt (opprinnelig feil). 6 m
// holder kameravinkelen nær horisontal (fortsatt synlig langt unna) samtidig som det er tydelig hevet
// over standardhøyden.
const HELI_ALTITUDE = 6;
const HELI_FLIGHT_HALF_LENGTH = 75;
const HELI_FLIGHT_DURATION_SEC = 5.5;
const HELI_SAFE_HORIZ_DISTANCE = 10;

// Scenario 3: dronen stikker av oppover mot høyden der et fly krysser (AIRWAY_ALTITUDE, godt over
// TRAFFIC_DANGER_ALTITUDE - selve terskelen rømningen stoppes ved dersom motorene ikke kuttes i tide).
const TRAFFIC_DANGER_ALTITUDE = 70;
const TRAFFIC_CLIMB_DURATION_SEC = 3.4;
const AIRWAY_ALTITUDE = 95;
const AIRPLANE_FLIGHT_HALF_LENGTH = 100;
const AIRPLANE_FLIGHT_DURATION_SEC = 6;

// Scenario 4: noen kommer gående rett mot flygeområdet mens brukeren beholder full kontroll - riktig
// respons er (som helikopteret) å vike unna og/eller lande, IKKE killswitch. Rolig gangfart (realistisk,
// ikke noe tidspress). Avgjøres FØRST når fotgjengerne faktisk er ferdige med å gå forbi (samme prinsipp
// som helikopteret) - IKKE fortløpende, for de starter allerede utenfor sikkerhetsavstanden og en
// fortløpende sjekk ville da bestått med det samme, før noe reelt har skjedd (se updatePedestrianDangerPhase).
// Kommer inn langs Z (dypt i feltet -> mot senteret), samme retning/prinsipp som helikopteret - ikke fra
// siden, som var usynlig helt til de plutselig sto rett ved siden av dronen.
const PEDESTRIAN_WALK_START_OFFSET = 28; // m - utenfor det typiske kamerabildet
const PEDESTRIAN_WALK_END_OFFSET = 6;    // m - fortsetter et stykke forbi senteret
const PEDESTRIAN_WALK_DURATION_SEC = 12;
const PEDESTRIAN_SAFE_DISTANCE = 10; // m, horisontal - samme resonnement som HELI_SAFE_HORIZ_DISTANCE
const PEDESTRIAN_SAFE_MAX_SPEED = 2; // m/s - "i kontrollert sveveflukt", ikke bare i ferd med å styrte forbi

// Ingen av meldingene under vises mens noe FAKTISK skjer (se seksjons-kommentaren over) - kun i etterkant,
// som forklaring når et forsøk mislykkes.
const KILLSWITCH_MESSAGES = {
    crowd: {
        fail: "For sent - dronen traff folkemengden. Nytt forsøk om et øyeblikk...", // aldri kuttet i tide (droneen NÅDDE faktisk frem)
        failLate: "Du kuttet motorene for sent til at det regnes som en trygg respons. Nytt forsøk om et øyeblikk..." // kuttet, men for nær innpå til å telle - se KILLSWITCH_SAFE_CUTOFF_FRACTION
    },
    heli: {
        failKilled: "Feil respons - dronen var fullt kontrollerbar. Et innflyvende helikopter unngås ved " +
            "å vike unna eller lande, ikke ved å kutte motorene. Nytt forsøk om et øyeblikk...",
        failTooClose: "For nær helikopteret! Vik tydelig unna eller land tidligere neste gang. Nytt forsøk om et øyeblikk..."
    },
    traffic: {
        fail: "For sent - konflikt med lufttrafikken. Nytt forsøk om et øyeblikk...",
        failLate: "Du kuttet motorene for sent til at det regnes som en trygg respons. Nytt forsøk om et øyeblikk..."
    },
    pedestrians: {
        failKilled: "Feil respons - dronen var fullt kontrollerbar. Fotgjengere i området unngås ved å " +
            "vike unna eller lande, ikke ved å kutte motorene. Nytt forsøk om et øyeblikk...",
        failTooClose: "For nære fotgjengerne! Vik tydelig unna eller land tidligere neste gang. Nytt forsøk om et øyeblikk..."
    }
};

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
// "Vente"-fasen i killswitch-øvelsen (ex11) gjenbruker sirkel-runden (samme CIRCLE_WAYPOINTS som ex5/
// ex6) som en "liksom"-øvelse å faktisk fly mens man venter på at noe skal skje - kjent fra tidligere
// øvelser, nese-frem-stil (se KILLSWITCH_PATROL_CAPTURE_RADIUS). Rent kosmetisk/uten avviksregler (se
// updateKillswitchPatrol) - looper bare videre uansett hvor lenge ventetiden varer.
const KILLSWITCH_PATROL_CAPTURE_RADIUS = 1.3; // samme som ex5/ex6 (se captureRadius-kommentaren der)
// ks-heli flyr sin "liksom"-runde et godt stykke lenger unna avgangsplassen enn standard-sirkelen (ikke
// bare høyere, se HELI_ALTITUDE) - med den flydd rett ved avgangsplassen måtte VLOS-kameraet (som følger
// dronen, se updateVlosCamera) fortsatt vinkle unødvendig bratt oppover selv ved moderat høyde, siden
// vertikal og horisontal avstand til dronen da var i samme størrelsesorden. Lenger unna gir en flatere,
// mer horisontal kameravinkel ved samme høyde - viktig for at det innkommende helikopteret (se
// spawnHelicopterFlight, som bruker samme senter) faktisk skal være synlig idet det nærmer seg.
const HELI_PATROL_CENTER = new THREE.Vector3(EXERCISE_CENTER.x, 0, EXERCISE_CENTER.z - 20);
const HELI_PATROL_WAYPOINTS = buildCircleWaypoints(HELI_PATROL_CENTER, CIRCLE_RADIUS, CIRCLE_SEGMENTS);

// Felles beskrivelsestekst-suffiks: reglene er like for alle øvelsene.
const EXERCISE_RULES_TEXT = "\n\nFørste avvik gir bare en advarsel; skjer det igjen nullstilles steget " +
    "og dronen settes tilbake på avgangsplassen (klokka går videre).\n\nDroneklassen settes automatisk " +
    "til Middels, og du flyr fra VLOS-posisjonen. R restarter hele øvelsen med nullstilt klokke.";

const EXERCISES = {
    ex1: {
        id: "ex1",
        icon: "fa-crosshairs",
        label: "1. Hover",
        shortDescription: "Hold posisjonen med nesa ut, til sidene og inn - " + HOVER_HOLD_SEC + " s per retning.",
        startHint: "Hold posisjonen i den gule indikatoren. Pila på bakken viser hvilken retning nesa skal peke.",
        fullDescription: "Hold dronen i ro over det markerte punktet på " + HOVER_ALTITUDE + " m høyde - " +
            "først med nesa bort fra deg, så mot venstre, så mot høyre, og til slutt med nesa mot deg. " +
            "Hver retning må holdes i minst " + HOVER_HOLD_SEC + " sekunder innenfor omtrent samme posisjon " +
            "og høyde - driver du ut av området eller mister retningen, nullstilles tiden for gjeldende " +
            "retning.\n\nPila på bakken viser retningen nesa skal peke.\n\nDroneklassen settes automatisk til " +
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
        startHint: "Fly firkanten. Følg indikatoren. Nesa skal peke bort fra deg hele tiden.",
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
        startHint: "Fly firkanten. Følg indikatoren. Nesa skal peke i fartsretningen hele veien.",
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
        startHint: "Følg sikksakk-mønsteret loddrett opp og skrått ned. Nesa peker bort fra deg.",
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
        startHint: "Fly sirkelen. Følg indikatoren. Nesa skal peke bort fra deg hele tiden.",
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
        startHint: "Fly sirkelen. Følg indikatoren. Nesa skal peke i fartsretningen hele veien.",
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
        startHint: "Fly åttetallet. Følg indikatoren. Nesa skal peke bort fra deg hele tiden.",
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
        startHint: "Fly åttetallet. Følg indikatoren. Nesa skal peke i fartsretningen hele veien.",
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
        startHint: "Fly åttetallet i sidevind. Korriger jevnlig for avdrift for å holde formen.",
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
        startHint: "Vent til nedtellingen er ferdig og gassen er ved ca. 50 %. Finn nese- og vindretning, fly trygt hjem og land.",
        fullDescription: "Dronen henger i lufta langt hjemmefra - bare synlig som en prikk - med " +
            "tilfeldig nese-retning og 6 m/s vind fra en ny, tilfeldig retning hver gang. Etter " +
            "nedtellingen overtar du kontrollen, men først når du har lagt gassen rundt 50 % (hover) - " +
            "akkurat som en ekte overtakelse.\n\nFinn ut hvilken vei nesa peker og hvor vinden kommer fra, " +
            "fly dronen trygt hjem og land på landingsplassen (H). Du må gjennomføre dette " +
            REQUIRED_RETURN_REPS + " ganger (ny tilfeldig posisjon, nese-retning og vindretning hver " +
            "gang) for å bestå - totaltiden for alle " + REQUIRED_RETURN_REPS + " forsøkene lagres i " +
            "menyen.\n\nR gir nytt forsøk fra runde 1 med nullstilt klokke.",
        wind: { speed: 6, directionDeg: 0, gust: 0.3 },
        randomizeWindDirection: true, // ny tilfeldig retning ved hver spawn (start/runde/R) - se spawnForExercise
        randomizeCloudCoverage: true, // tilfeldig skydekke per runde, men minst én av de tre garantert 100% - se spawnForExercise
        spawn: "far", // spesialspawn: høyt og langt unna med tilfeldig yaw (se spawnForExercise)
        stages: [{ id: "return-home", label: "Returner hjem", type: "return" }]
    },
    ex11: {
        id: "ex11",
        icon: "fa-triangle-exclamation",
        label: "11. Uforutsette hendelser",
        shortDescription: "Fly noen øvelser - underveis kan det skje noe uforutsett. 4 scenarier må bestås.",
        fullDescription: "Du skal fly noen øvelser - underveis kan det skje noe uforutsett. Riktig " +
            "reaksjon kan være å fly unna, lande, eller stoppe motorene i lufta. Det er 4 scenarier som " +
            "må bestås, ett om gangen.\n\nHusk å sette killswitchen på fjernkontrollen din i " +
            "Fjernkontroll-kalibrering (Innstillinger) - tastatur og skjermknappen virker ikke i denne " +
            "øvelsen, det er den ekte bryteren som skal trenes.",
        requiresGamepadKill: true,
        skipLanding: true, // hver deløvelse ender med motorene kuttet, landet eller trygt unna - ingen vits i å kreve landing på H
        noTiming: true, // handler om å reagere RIKTIG, ikke raskt - ingen stoppeklokke/bestetid her
        stages: [
            { id: "ks-crowd", label: "Rømning mot folkemengden", type: "killswitch", variant: "crowd", runawaySec: CROWD_RUNAWAY_DURATION_SEC },
            // patrolAltitude: "liksom"-runden flys mye høyere enn vanlig for akkurat dette steget - se
            // HELI_ALTITUDE-kommentaren (må overlappe reell helikopterhøyde for at nærhet skal bety noe).
            { id: "ks-heli", label: "Helikopter i lav innflyging", type: "killswitch", variant: "heli", patrolAltitude: HELI_ALTITUDE, patrolWaypoints: HELI_PATROL_WAYPOINTS },
            { id: "ks-traffic", label: "Rømning mot lufttrafikk", type: "killswitch", variant: "traffic", runawaySec: TRAFFIC_CLIMB_DURATION_SEC },
            { id: "ks-pedestrians", label: "Fotgjengere nærmer seg", type: "killswitch", variant: "pedestrians" }
        ]
    },
    race1: {
        id: "race1",
        icon: "fa-flag-checkered",
        label: "Racingbane",
        droneClass: "racing",
        forceCameraMode: "fpv",
        forceFlightMode: "acro",
        forceFpvTiltDeg: 30,
        freeCameraToggle: true, // C (kamerabytte) er IKKE låst til VLOS her, se toggleCamera
        shortDescription: "Fly gjennom alle portene på racingbanen så fort du kan - klokken starter når du krysser start/mål.",
        startHint: "Fly gjennom porten for å starte tiden.",
        fullDescription: "En lengre racingbane et stykke unna avgangsplassen, med porter, en låve du flyr " +
            "gjennom, og en egen gate på taket av rådhuset.\n\nKlokken starter automatisk idet du krysser " +
            "start/mål-porten (svart/hvitt rutemønster med en gul pil som viser flyretningen), og stopper " +
            "når du har fløyet gjennom alle de andre portene i rekkefølge og kommer tilbake til samme " +
            "port. Du kan fly så mange runder du vil - hver fullførte runde havner i ledertavlen (lagres " +
            "lokalt i nettleseren), med beste tid øverst. Endre navnet ditt øverst i ledertavlen for å " +
            "merke dine egne tider.\n\nSpawner i Racing-klasse, Acro-modus og FPV-kamera.",
        // Ikke noTiming (ex11 sin variant) - racing har en helt egen, løpende klokke (se
        // updateExerciseHud/raceStartTime), bare vist annerledes enn de vanlige øvelsenes tidtaking.
        stages: [{ id: "race-lap", label: "Racingbane", type: "racing" }]
    }
};
const EXERCISE_ORDER = ["ex1", "ex2", "ex3", "ex4", "ex5", "ex6", "ex7", "ex8", "ex9", "ex10", "ex11"];
// Kategorisering for øvelsesmenyens undermenyer (Stabilized/Acro) - se showExerciseCategoryView. Racing
// (race1) er bevisst IKKE i EXERCISE_ORDER (som styrer bekreftelsen/diplomet) - det er et åpent
// tidsforsøk med egen ledertavle, ikke en engangs bestått/ikke-bestått-øvelse.
const ACRO_EXERCISE_ORDER = ["race1"];

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
    injured: false, // droneen har truffet en person (VLOS-pilot eller forbipasserende) - eget varsel (legevakt/ambulanse), R for restart
    injuredTarget: null // "pilot" | "bystander" - styrer kun bannerteksten, se updateHud
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

// Trebygging (bjørk/furu) og vind-svai er delt med fixed-wing-simulatoren - se Sim.buildRandomTree/
// createTreeSwayManager i simulator-common.js.
const treeSwayManager = Sim.createTreeSwayManager();

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
// Alle taster spillet selv lytter på (styring + hurtigtaster) - se keydown-lytteren lenger ned, som
// preventDefault()-er alle disse for å hindre at nettleseren kaprer kombinasjoner (Ctrl+D, Ctrl+R, ...).
const GAME_KEY_CODES = new Set([
    "KeyW", "KeyA", "KeyS", "KeyD", "KeyQ", "KeyE",
    "ShiftLeft", "ShiftRight", "ControlLeft", "ControlRight", "Space",
    "Digit1", "Digit2", "Digit3",
    "KeyK", "KeyR", "KeyC", "KeyT", "KeyH", "KeyO", "KeyM"
]);

let renderer, scene, chaseCamera, fpvCamera, vlosCamera, activeCamera;
let droneGroup, dronePropellers;
let heliHandle, airplaneHandle, pedestrianHandle; // se buildHelicopter/buildAirplane/buildPedestrianGroup - kun brukt av ex11
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
    // Tidligere lå det en THREE.GridHelper (sort/grønne linjer over hele bakken) her i tillegg til
    // sjakkbrett-teksturen. Den fungerte dårlig sammen med elva - linjene stakk gjennom vannoverflaten
    // og "blinket" (z-fighting) når kameraet beveget seg, og ga generelt et unaturlig rutenett-utseende
    // over hele kartet. Sjakkbrett-teksturen (Sim.buildGroundTexture over) gir fortsatt avstandsreferanse
    // uten dette problemet, så griden er fjernet i stedet for justert.

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
// curvePower styrer stigningsprofilen fra fot til topp (se mountainProfileRadiusFrac) - 1 er den
// opprinnelige rette skråstreken (kjegle), >1 gir en avrundet/bred topp med brattere nedre flanker,
// <1 gir en bred/slak fot med en brattere, spissere topp (klassisk alpin silhuett). jaggedness styrer
// hvor ru/steinete overflaten er (0,65 = glatt/erodert kolle, 1,4 = kraftig taggete massiv - se
// buildGradientPeakGeometry), noiseFreqMul hvor MANGE bulker/rygger silhuetten har (lav = få, brede
// former, høy = mange små tagger). subPeaks er en egen, håndlaget bi-topp-liste PER fjell (1-3 stk,
// ulik plassering/størrelse) i stedet for samme faste tvilling-rygg-oppskrift på alle åtte - ellers
// er alle fjell strukturelt identiske massiv, bare skalert. Bevisst blandet variasjon, ikke alle like.
const MOUNTAIN_DEFS = [
    { angle: 0, dist: 620, height: 69, radius: 110, snow: false, curvePower: 1, jaggedness: 0.7, noiseFreqMul: 0.8,
        subPeaks: [{ f: 0.35, off: 0.3, dirOffset: 2.0 }] }, // lav, avrundet ås
    { angle: 45, dist: 560, height: 103, radius: 165, snow: true, curvePower: 1.7, jaggedness: 1.1, noiseFreqMul: 1,
        subPeaks: [{ f: 0.55, off: 0.32, dirOffset: 1.3 }, { f: 0.4, off: -0.38, dirOffset: 3.6 }, { f: 0.3, off: 0.45, dirOffset: 5.0 }] }, // bredt massiv
    { angle: 90, dist: 600, height: 81, radius: 130, snow: false, curvePower: 0.6, jaggedness: 1.3, noiseFreqMul: 1.3,
        subPeaks: [{ f: 0.3, off: 0.28, dirOffset: 1.6 }, { f: 0.25, off: -0.3, dirOffset: 4.2 }] }, // spiss, steinete alpintopp
    { angle: 135, dist: 540, height: 116, radius: 185, snow: true, curvePower: 0.75, jaggedness: 1.4, noiseFreqMul: 1.4,
        subPeaks: [{ f: 0.6, off: 0.35, dirOffset: 1.1 }, { f: 0.45, off: -0.4, dirOffset: 3.3 }, { f: 0.35, off: 0.4, dirOffset: 5.4 }] }, // dramatisk, taggete massiv (trollet)
    { angle: 180, dist: 610, height: 75, radius: 120, snow: false, curvePower: 1.4, jaggedness: 0.65, noiseFreqMul: 0.7,
        subPeaks: [{ f: 0.25, off: 0.25, dirOffset: 2.6 }] }, // rund, slak kolle
    { angle: 225, dist: 570, height: 97, radius: 155, snow: true, curvePower: 0.55, jaggedness: 1.2, noiseFreqMul: 1.2,
        subPeaks: [{ f: 0.32, off: 0.3, dirOffset: 1.8 }, { f: 0.28, off: -0.32, dirOffset: 4.5 }] }, // spiss snøtopp
    { angle: 270, dist: 590, height: 88, radius: 140, snow: false, curvePower: 1, jaggedness: 1, noiseFreqMul: 1,
        subPeaks: [{ f: 0.55, off: 0.32, dirOffset: 1.3 }, { f: 0.4, off: -0.38, dirOffset: 3.6 }] }, // klassisk, balansert (varden)
    { angle: 315, dist: 550, height: 109, radius: 175, snow: true, curvePower: 0.8, jaggedness: 1.15, noiseFreqMul: 1.05,
        subPeaks: [{ f: 0.5, off: 0.3, dirOffset: 1.0 }, { f: 0.42, off: -0.35, dirOffset: 3.0 }, { f: 0.3, off: 0.42, dirOffset: 5.2 }] } // stort massiv (hytta)
];
// Flat liste over ALLE fjell-koner (hovedtopper + bi-topper) - bygget ÉN gang og delt mellom
// geometribyggingen (buildMountainRange) og kollisjonsdeteksjon (mountainHeightAt). Med dette som
// eneste kilde til sannhet kan ikke kollisjonsflaten komme ut av synk med det som faktisk vises -
// samme posisjon/radius/høyde/kurve/seed brukes begge steder. topRadiusFrac/seed-formlene matcher
// nøyaktig det buildMountainRange brukte før denne ble innført. jaggedness/noiseFreqMul er rent
// visuelle (kollisjonen bruker kun den glatte profilkurven, se mountainHeightAt), men lagres her
// likevel for at MOUNTAIN_PEAKS skal forbli eneste kilde til sannhet for ALT ved hvert fjell.
const MOUNTAIN_PEAKS = (function () {
    const peaks = [];
    MOUNTAIN_DEFS.forEach(function (m, i) {
        const rad = m.angle * Math.PI / 180;
        const x = Math.sin(rad) * m.dist, z = Math.cos(rad) * m.dist;
        const curvePower = m.curvePower || 1;
        const jaggedness = m.jaggedness || 1;
        const noiseFreqMul = m.noiseFreqMul || 1;
        peaks.push({
            x: x, z: z, radius: m.radius, height: m.height, topRadiusFrac: 0.18, curvePower: curvePower,
            jaggedness: jaggedness, noiseFreqMul: noiseFreqMul,
            angle: rad, seed: i * 1.7 + 1, snow: m.snow, isMain: true, mainIndex: i
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
// Fargestopp brukt av buildGradientPeakGeometry - se der.
const MOUNTAIN_GROUND_COLOR = new THREE.Color(0x3a5f3a);   // matcher Sim.buildGroundTexture
const MOUNTAIN_FOOTHILL_COLOR = new THREE.Color(0x6e7a4d); // oliven - bro mellom bakke og stein
const MOUNTAIN_ROCK_COLOR = new THREE.Color(0x5b6472);
const MOUNTAIN_ROCK_LIGHT_COLOR = new THREE.Color(0x7c8794);
const MOUNTAIN_SNOW_COLOR = new THREE.Color(0xf0f4f8);

// Buet (ikke lineær) radius-profil fra fot (heightFrac 0) til topp (heightFrac 1) - se curvePower-
// kommentaren ved MOUNTAIN_DEFS. curvePower=1 gir nøyaktig samme rette skråstrek som en ren kjegle
// (1-heightFrac, lineær). Invertert i mountainProfileHeightFrac under, brukt av mountainHeightAt for
// kollisjon - de to funksjonene MÅ holdes i sync (én er den matematiske inversen av den andre).
function mountainProfileRadiusFrac(heightFrac, topRadiusFrac, curvePower) {
    return topRadiusFrac + (1 - topRadiusFrac) * Math.pow(1 - heightFrac, curvePower);
}
// Inversen: gitt hvor stor brøkdel av grunnradiusen et punkt ligger unna sentrum (distFrac, 0=sentrum,
// 1=foten), hvilken høydebrøkdel tilsvarer fjelloverflaten akkurat der. Brukt av mountainHeightAt.
function mountainProfileHeightFrac(distFrac, topRadiusFrac, curvePower) {
    if (distFrac <= topRadiusFrac) return 1; // innenfor den flate toppflaten
    if (distFrac >= 1) return 0;
    const t = (distFrac - topRadiusFrac) / (1 - topRadiusFrac);
    return 1 - Math.pow(t, 1 / curvePower);
}

// Bygger en CylinderGeometry MED ENSARTET RADIUS (selve innsnevringen mot toppen styres heretter per
// vertex av mountainProfileRadiusFrac over, ikke geometriens egen lineære topp/bunn-interpolasjon - det
// er det som gjør buede/ikke-lineære profiler mulig) med (1) uregelmessig, ru silhuett - vinkelavhengig
// sinus-støy forskyver hver vertekes radius og litt av høyden, styrt av "jaggedness" (0 = helt glatt,
// brukt for den slake foten) - og (2) en jevn per-vertex fargeovergang mellom flere høydebaserte
// fargestopp i stedet for separate meshes med harde fargegrenser (det ga tidligere en synlig skarp
// overgang der snø-/stein-meshene møttes). Jitteren dempes (ikke fjernes helt) nær toppen, for en
// avrundet/erodert topp i stedet for enten en skarp spiss eller en unaturlig helt glatt/flat platå-sirkel.
function buildGradientPeakGeometry(radius, height, seed, colorStops, jaggedness, topRadiusFrac, curvePower, noiseFreqMul) {
    const geo = new THREE.CylinderGeometry(radius, radius, height, 14, 7);
    const pos = geo.attributes.position;
    const colors = new Float32Array(pos.count * 3);
    const tmp = new THREE.Color();
    const p = curvePower || 1;
    const fm = noiseFreqMul || 1; // antall bulker/rygger rundt silhuetten - se MOUNTAIN_DEFS-kommentaren
    for (let i = 0; i < pos.count; i++) {
        const x = pos.getX(i), y = pos.getY(i), z = pos.getZ(i);
        const heightFrac = clamp((y + height / 2) / height, 0, 1);
        const angle = Math.atan2(z, x);
        let radialScale = mountainProfileRadiusFrac(heightFrac, topRadiusFrac || 0, p);
        if (jaggedness > 0) {
            const topDamp = 1 - Math.pow(heightFrac, 3) * 0.7; // roligere silhuett mot toppen, ikke null
            // baseDamp dempet Y-jitteret til null de aller nederste par prosentene av høyden - UTEN denne
            // fikk fotringen (heightFrac 0) FULL jitterstyrke (topDamp er 1 der, ikke dempet), som kunne
            // løfte deler av foten flere meter over selve bakkeplanet og etterlate synlige "svevende"
            // hull der fjellet ikke lenger har kontakt med bakken. Rampes raskt opp (innen 4% av høyden)
            // så overgangen er umerkelig og resten av silhuetten er uendret.
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

// Nøyaktig samme Y-jitter som senter-vertexen i toppdekket over (angle=0 siden x=z=0 der - upåvirket av
// noiseFreqMul siden 0*fm alltid er 0 - heightFrac=1 gir topDamp=0.3 og baseDamp=1 fast) - uten denne
// fikk påskeeggene en fast antatt topp-høyde og svevde over/sank ned i den faktisk ujevne toppflaten
// (spesielt synlig på de høyeste og mest taggete fjellene, der jitteret er størst).
function peakApexYOffset(height, seed, jaggedness) {
    return Math.sin(seed * 4.3) * height * 0.03 * (jaggedness || 1) * 0.3;
}

// ---------- Påskeegg på fjelltoppene ----------
// Små, human-skala overraskelser for den som gidder å fly de 500+ meterne ut dit - synlige/morsomme
// bare på nært hold, usynlige detaljer på avstand. Ekte meter-skala (ikke skalert med fjellets egen
// størrelse), akkurat som resten av verden. Én, distinkt påskeegg per topp - se MOUNTAIN_EASTER_EGGS
// i buildMountainRange for hvilken builder som hører til hvilken toppindeks.
function buildNorwegianFlag() {
    const group = new THREE.Group();
    const poleMat = new THREE.MeshStandardMaterial({ color: 0x8a8a8a });
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.03, 2.2, 6), poleMat);
    pole.position.y = 1.1;
    group.add(pole);

    const fw = 0.9, fh = 0.65;
    const redMat = new THREE.MeshStandardMaterial({ color: 0xba0c2f, side: THREE.DoubleSide });
    const whiteMat = new THREE.MeshStandardMaterial({ color: 0xffffff, side: THREE.DoubleSide });
    const blueMat = new THREE.MeshStandardMaterial({ color: 0x00205b, side: THREE.DoubleSide });
    const flag = new THREE.Group();
    flag.add(new THREE.Mesh(new THREE.PlaneGeometry(fw, fh), redMat));
    const crossX = -fw * 0.18; // korset nærmere stangen - norsk flagg-proporsjon (ikke midtstilt)
    const whiteV = new THREE.Mesh(new THREE.PlaneGeometry(fw * 0.22, fh), whiteMat);
    whiteV.position.set(crossX, 0, 0.001);
    flag.add(whiteV);
    const whiteH = new THREE.Mesh(new THREE.PlaneGeometry(fw, fh * 0.28), whiteMat);
    whiteH.position.z = 0.001;
    flag.add(whiteH);
    const blueV = new THREE.Mesh(new THREE.PlaneGeometry(fw * 0.11, fh), blueMat);
    blueV.position.set(crossX, 0, 0.002);
    flag.add(blueV);
    const blueH = new THREE.Mesh(new THREE.PlaneGeometry(fw, fh * 0.14), blueMat);
    blueH.position.z = 0.002;
    flag.add(blueH);
    flag.position.set(fw / 2, 1.9, 0);
    group.add(flag);
    return group;
}

// Varde - stabel av uregelmessige steiner, norsk tradisjon for å markere en topp.
function buildSummitCairn() {
    const group = new THREE.Group();
    const rockMat = new THREE.MeshStandardMaterial({ color: 0x746b60, flatShading: true });
    [{ y: 0.06, r: 0.32 }, { y: 0.16, r: 0.26 }, { y: 0.25, r: 0.19 }, { y: 0.33, r: 0.12 }].forEach(function (l, i) {
        const rock = new THREE.Mesh(new THREE.DodecahedronGeometry(l.r, 0), rockMat);
        rock.position.set(Math.sin(i * 2.1) * 0.04, l.y, Math.cos(i * 2.1) * 0.04);
        rock.rotation.set(i * 0.7, i * 1.3, i * 0.4);
        group.add(rock);
    });
    return group;
}

// Et godmodig fjelltroll som titter ut over kanten - norsk folklore-referanse. Litt større og med et
// fullt ansikt (øyne, øyenbryn, munn, tenner, vorter) i stedet for bare et blankt hode med nese og ører -
// den detaljen er det som faktisk gir det karakter når man flyr helt inntil for å se på det.
function buildMountainTroll() {
    const group = new THREE.Group();
    const skinMat = new THREE.MeshStandardMaterial({ color: 0x6b7a5e, flatShading: true, roughness: 1 });
    const hairMat = new THREE.MeshStandardMaterial({ color: 0x3a2f1f, flatShading: true });
    const wartMat = new THREE.MeshStandardMaterial({ color: 0x545e46, flatShading: true, roughness: 1 });

    const body = new THREE.Mesh(new THREE.SphereGeometry(0.72, 9, 7), skinMat);
    body.scale.set(1, 0.85, 1);
    body.position.y = 0.62;
    group.add(body);
    const head = new THREE.Mesh(new THREE.SphereGeometry(0.44, 10, 8), skinMat);
    head.position.y = 1.28;
    group.add(head);
    // Underkjeve - gir hodet en tydeligere ansiktsform i stedet for én ensom kule.
    const jaw = new THREE.Mesh(new THREE.SphereGeometry(0.3, 8, 6), skinMat);
    jaw.scale.set(0.95, 0.7, 0.85);
    jaw.position.set(0, 1.06, 0.1);
    group.add(jaw);
    const nose = new THREE.Mesh(new THREE.ConeGeometry(0.13, 0.58, 6), skinMat);
    nose.rotation.x = Math.PI / 2;
    nose.position.set(0, 1.24, 0.48);
    group.add(nose);
    [-1, 1].forEach(function (side) {
        const ear = new THREE.Mesh(new THREE.ConeGeometry(0.12, 0.28, 5), skinMat);
        ear.position.set(side * 0.4, 1.44, 0);
        ear.rotation.z = side * 0.5;
        group.add(ear);
    });
    const hairTuft = new THREE.Mesh(new THREE.ConeGeometry(0.18, 0.38, 5), hairMat);
    hairTuft.position.y = 1.72;
    group.add(hairTuft);

    // Øyne under buskete, sammenknepne øyenbryn - det som gjør et hode om til et ANSIKT.
    const eyeWhiteMat = new THREE.MeshStandardMaterial({ color: 0xe8dfa0 });
    const pupilMat = new THREE.MeshStandardMaterial({ color: 0x1c1c1c });
    [-1, 1].forEach(function (side) {
        const eyeWhite = new THREE.Mesh(new THREE.SphereGeometry(0.075, 8, 6), eyeWhiteMat);
        eyeWhite.position.set(side * 0.17, 1.33, 0.37);
        group.add(eyeWhite);
        const pupil = new THREE.Mesh(new THREE.SphereGeometry(0.035, 6, 6), pupilMat);
        pupil.position.set(side * 0.17, 1.33, 0.43);
        group.add(pupil);
        const brow = new THREE.Mesh(new THREE.BoxGeometry(0.19, 0.045, 0.06), hairMat);
        brow.position.set(side * 0.17, 1.4, 0.4);
        brow.rotation.z = side * -0.35; // sammenknepet, litt sint/lurt uttrykk
        group.add(brow);
    });

    // Munn med et par tenner/hoggtenner som stikker opp - klassisk trolltrekk.
    const mouth = new THREE.Mesh(new THREE.BoxGeometry(0.26, 0.05, 0.04), pupilMat);
    mouth.position.set(0, 1.09, 0.44);
    group.add(mouth);
    const toothMat = new THREE.MeshStandardMaterial({ color: 0xe8e0c8 });
    [-1, 1].forEach(function (side) {
        const tooth = new THREE.Mesh(new THREE.ConeGeometry(0.03, 0.1, 4), toothMat);
        tooth.rotation.x = Math.PI;
        tooth.position.set(side * 0.09, 1.12, 0.44);
        group.add(tooth);
    });

    // Vorter på nese og kinn - litt asymmetriske for å ikke se maskinelt plasserte ut.
    [{ x: 0.09, y: 1.15, z: 0.53, r: 0.045 }, { x: -0.16, y: 1.27, z: 0.36, r: 0.04 }, { x: 0.2, y: 1.3, z: 0.28, r: 0.035 }]
        .forEach(function (w) {
            const wart = new THREE.Mesh(new THREE.SphereGeometry(w.r, 6, 5), wartMat);
            wart.position.set(w.x, w.y, w.z);
            group.add(wart);
        });

    // Armer/never som griper kanten - forsterker inntrykket av at det faktisk sitter og titter ut.
    [-1, 1].forEach(function (side) {
        const arm = new THREE.Mesh(new THREE.SphereGeometry(0.14, 7, 6), skinMat);
        arm.scale.set(0.8, 1.3, 0.8);
        arm.position.set(side * 0.62, 0.75, 0.28);
        arm.rotation.z = side * -0.3;
        group.add(arm);
        const hand = new THREE.Mesh(new THREE.SphereGeometry(0.13, 7, 6), skinMat);
        hand.position.set(side * 0.68, 0.52, 0.42);
        group.add(hand);
    });
    return group;
}

// Ensom turgåer som har nådd toppen - gjenbruker samme figur-bygger som folkemengden/fotgjengerne
// (buildPersonFigure), som allerede vender mot lokal +Z (samme konvensjon som resten av påskeeggene).
function buildLoneHiker() {
    const group = new THREE.Group();
    group.add(Sim.buildPersonFigure({ vestColor: 0xd97a2b }));
    // Sekk med synlige stropper - skiller figuren tydelig fra en helt vanlig folkemengde-person.
    const packMat = new THREE.MeshStandardMaterial({ color: 0x3a4a63 });
    const pack = new THREE.Mesh(new THREE.BoxGeometry(0.28, 0.4, 0.16), packMat);
    pack.position.set(0, 1.05, -0.16); // på ryggen - motsatt av ansiktsretningen (+Z)
    group.add(pack);
    const strapMat = new THREE.MeshStandardMaterial({ color: 0x22293a });
    [-1, 1].forEach(function (side) {
        const strap = new THREE.Mesh(new THREE.BoxGeometry(0.035, 0.42, 0.03), strapMat);
        strap.position.set(side * 0.13, 1.05, -0.02);
        strap.rotation.x = -0.15;
        group.add(strap);
    });
    // Tursekk-stav
    const poleMat = new THREE.MeshStandardMaterial({ color: 0x888888 });
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.015, 0.015, 1.1, 6), poleMat);
    pole.position.set(0.3, 0.55, 0.15);
    pole.rotation.z = 0.15;
    group.add(pole);
    // Rødrutete turlue - gjenkjennelig norsk turgåer-silhuett, og gjør hodet lesbart på avstand.
    const hatMat = new THREE.MeshStandardMaterial({ color: 0xba2c2c });
    const hat = new THREE.Mesh(new THREE.SphereGeometry(0.115, 10, 8, 0, Math.PI * 2, 0, Math.PI * 0.6), hatMat);
    hat.position.y = 1.4;
    group.add(hat);
    const pompomMat = new THREE.MeshStandardMaterial({ color: 0xf4f0e6 });
    const pompom = new THREE.Mesh(new THREE.SphereGeometry(0.035, 8, 6), pompomMat);
    pompom.position.y = 1.47;
    group.add(pompom);
    return group;
}

// Geocache - liten "skattekiste" på en stein, referanse til friluftslivs-hobbyen.
function buildGeocache() {
    const group = new THREE.Group();
    const rockMat = new THREE.MeshStandardMaterial({ color: 0x6e6a63, flatShading: true });
    const rock = new THREE.Mesh(new THREE.DodecahedronGeometry(0.28, 0), rockMat);
    rock.scale.set(1.3, 0.55, 1.1);
    rock.position.y = 0.12;
    group.add(rock);
    // Boksen står åpen med et hevet, hengslet lokk og en gul loggbok-lapp som stikker opp av den - gjør
    // det tydelig at det faktisk ER en (funnet/åpnet) geocache, ikke bare en tilfeldig grønn kloss.
    const boxGroup = new THREE.Group();
    boxGroup.position.set(0.05, 0.28, 0.08);
    boxGroup.rotation.y = 0.3;
    group.add(boxGroup);
    const boxMat = new THREE.MeshStandardMaterial({ color: 0x4a5c33 });
    const boxBase = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.08, 0.13), boxMat);
    boxBase.position.y = 0.04;
    boxGroup.add(boxBase);
    const lid = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.025, 0.13), boxMat);
    lid.position.set(0, 0.09, -0.075);
    lid.rotation.x = -1.1; // vippet åpent bakover på et "hengsel" langs bakkanten
    boxGroup.add(lid);
    const paperMat = new THREE.MeshStandardMaterial({ color: 0xe8d879, side: THREE.DoubleSide });
    const paper = new THREE.Mesh(new THREE.PlaneGeometry(0.1, 0.07), paperMat);
    paper.position.set(0, 0.1, 0.01);
    paper.rotation.x = -0.3;
    boxGroup.add(paper);
    return group;
}

// Turskilt med noen retningsarmer - ingen lesbar tekst (samme prinsipp som varden: detaljen leses som
// "et skilt", ikke bokstaver, på den avstanden dette faktisk sees fra).
function buildTrailSignpost() {
    const group = new THREE.Group();
    const woodMat = new THREE.MeshStandardMaterial({ color: 0x6b4a2f });
    const post = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.06, 1.8, 6), woodMat);
    post.position.y = 0.9;
    group.add(post);
    // Rød "T"-merking - Den Norske Turistforenings klassiske stimerking, malt rett på stolpen. Gjør
    // stolpen umiddelbart lesbar som et TURSKILT og ikke bare en bar påle.
    const paintMat = new THREE.MeshStandardMaterial({ color: 0xba0c2f });
    const tVert = new THREE.Mesh(new THREE.BoxGeometry(0.032, 0.16, 0.01), paintMat);
    tVert.position.set(0, 1.68, 0.061);
    group.add(tVert);
    const tHoriz = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.032, 0.01), paintMat);
    tHoriz.position.set(0, 1.75, 0.061);
    group.add(tHoriz);
    // Skiltarmer med tilspisset pilspiss (i stedet for en blank planke) - leses tydelig som "retningsskilt".
    const signMat = new THREE.MeshStandardMaterial({ color: 0xd8c9a3 });
    const tipMat = new THREE.MeshStandardMaterial({ color: 0xd8c9a3, side: THREE.DoubleSide });
    [{ y: 1.5, len: 0.55, rot: 0.35 }, { y: 1.35, len: 0.45, rot: -0.6 }, { y: 1.2, len: 0.5, rot: 2.4 }]
        .forEach(function (s) {
            const signGroup = new THREE.Group();
            signGroup.position.set(Math.sin(s.rot) * s.len * 0.5, s.y, Math.cos(s.rot) * s.len * 0.5);
            signGroup.rotation.y = s.rot;
            group.add(signGroup);
            const sign = new THREE.Mesh(new THREE.BoxGeometry(s.len * 0.8, 0.12, 0.02), signMat);
            signGroup.add(sign);
            const tip = new THREE.Mesh(new THREE.ConeGeometry(0.085, 0.14, 3), tipMat);
            tip.rotation.z = -Math.PI / 2;
            tip.position.x = s.len * 0.47;
            signGroup.add(tip);
        });
    return group;
}

// Snømann med skjerf og hatt - naturlig hjemmehørende på de snødekte toppene.
function buildSnowman() {
    const group = new THREE.Group();
    const snowMat = new THREE.MeshStandardMaterial({ color: 0xf4f7fb });
    const bottom = new THREE.Mesh(new THREE.SphereGeometry(0.34, 10, 8), snowMat);
    bottom.position.y = 0.34;
    group.add(bottom);
    const mid = new THREE.Mesh(new THREE.SphereGeometry(0.24, 10, 8), snowMat);
    mid.position.y = 0.76;
    group.add(mid);
    const head = new THREE.Mesh(new THREE.SphereGeometry(0.16, 10, 8), snowMat);
    head.position.y = 1.06;
    group.add(head);
    const noseMat = new THREE.MeshStandardMaterial({ color: 0xd9782b });
    const nose = new THREE.Mesh(new THREE.ConeGeometry(0.035, 0.22, 6), noseMat);
    nose.rotation.x = Math.PI / 2;
    nose.position.set(0, 1.05, 0.18);
    group.add(nose);
    const darkMat = new THREE.MeshStandardMaterial({ color: 0x232323 });
    [0.09, -0.09].forEach(function (dx) {
        const eye = new THREE.Mesh(new THREE.SphereGeometry(0.02, 6, 6), darkMat);
        eye.position.set(dx, 1.1, 0.14);
        group.add(eye);
    });
    [0.62, 0.78, 0.92].forEach(function (y) {
        const button = new THREE.Mesh(new THREE.SphereGeometry(0.025, 6, 6), darkMat);
        button.position.set(0, y, 0.23);
        group.add(button);
    });
    // Kvist-armer - uten disse leser figuren som tre snøballer, ikke en snømann.
    const twigMat = new THREE.MeshStandardMaterial({ color: 0x4a3420 });
    [-1, 1].forEach(function (side) {
        const arm = new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.012, 0.5, 5), twigMat);
        arm.position.set(side * 0.32, 0.85, 0);
        arm.rotation.z = side * -0.9;
        group.add(arm);
        const twig = new THREE.Mesh(new THREE.CylinderGeometry(0.01, 0.005, 0.14, 4), twigMat);
        twig.position.set(side * 0.52, 0.98, 0);
        twig.rotation.z = side * -1.6;
        group.add(twig);
    });
    const scarfMat = new THREE.MeshStandardMaterial({ color: 0xba0c2f });
    const scarf = new THREE.Mesh(new THREE.TorusGeometry(0.17, 0.035, 6, 12), scarfMat);
    scarf.rotation.x = Math.PI / 2;
    scarf.position.y = 0.9;
    group.add(scarf);
    const hatMat = new THREE.MeshStandardMaterial({ color: 0x1c1c1c });
    const hat = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.12, 0.16, 10), hatMat);
    hat.position.y = 1.28;
    group.add(hat);
    const brim = new THREE.Mesh(new THREE.CylinderGeometry(0.17, 0.17, 0.02, 10), hatMat);
    brim.position.y = 1.2;
    group.add(brim);
    return group;
}

// Liten, falurød fjellhytte med gavltak - norsk turhytte-referanse for den høyeste toppen.
function buildMountainCabin() {
    const group = new THREE.Group();
    // Mørk grunnmur-list rundt foten - forankrer hytta visuelt til bakken i stedet for at
    // den falurøde veggen bare stopper brått rett over terrenget.
    const foundationMat = new THREE.MeshStandardMaterial({ color: 0x3a3733, flatShading: true });
    const foundation = new THREE.Mesh(new THREE.BoxGeometry(1.48, 0.14, 1.68), foundationMat);
    foundation.position.y = 0.07;
    group.add(foundation);
    const wallMat = new THREE.MeshStandardMaterial({ color: 0xa33a2c });
    const wall = new THREE.Mesh(new THREE.BoxGeometry(1.4, 1.1, 1.6), wallMat);
    wall.position.y = 0.14 + 0.55;
    group.add(wall);
    const roofMat = new THREE.MeshStandardMaterial({ color: 0x2b2b2b });
    const roof = new THREE.Mesh(new THREE.ConeGeometry(1.25, 0.7, 4), roofMat);
    roof.rotation.y = Math.PI / 4;
    roof.position.y = 0.14 + 1.45;
    group.add(roof);
    // Mønekam langs takryggen - et enkelt strøk øker leseligheten av "gavltak" betraktelig på nært hold.
    const ridge = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.06, 1.35), roofMat);
    ridge.position.y = 0.14 + 1.72;
    group.add(ridge);
    const trimMat = new THREE.MeshStandardMaterial({ color: 0xf4f0e6, side: THREE.DoubleSide });
    const door = new THREE.Mesh(new THREE.PlaneGeometry(0.32, 0.6), trimMat);
    door.position.set(0, 0.14 + 0.35, 0.81);
    group.add(door);
    const winPane = new THREE.Mesh(new THREE.PlaneGeometry(0.26, 0.26), trimMat);
    winPane.position.set(0.45, 0.14 + 0.65, 0.81);
    group.add(winPane);
    // Vindusluker - typisk detalj på norske fjellhytter, og bryter opp den ellers flate falurøde veggen.
    const shutterMat = new THREE.MeshStandardMaterial({ color: 0xf4f0e6 });
    [-1, 1].forEach(function (side) {
        const shutter = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.32, 0.03), shutterMat);
        shutter.position.set(0.45 + side * 0.16, 0.14 + 0.65, 0.8);
        group.add(shutter);
    });
    // Inngangstrapp - en liten kloss foran døra, så terskelen ikke bare henger i lufta over bakken.
    const stepMat = new THREE.MeshStandardMaterial({ color: 0x5b5650 });
    const step = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.12, 0.24), stepMat);
    step.position.set(0, 0.06, 0.95);
    group.add(step);
    const chimneyMat = new THREE.MeshStandardMaterial({ color: 0x6b6b6b });
    const chimney = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.4, 0.16), chimneyMat);
    chimney.position.set(-0.4, 0.14 + 1.65, -0.2);
    group.add(chimney);
    return group;
}

function buildMountainRange() {
    const group = new THREE.Group();
    // Delt for alt (hovedtopp, bi-topper) - fargen kommer utelukkende fra per-vertex-attributtet over,
    // ikke fra materialet, så ett shared material er nok.
    const peakMat = new THREE.MeshStandardMaterial({ vertexColors: true, flatShading: true, roughness: 1 });
    // Påskeegg på ALLE åtte toppene nå (indeks i MOUNTAIN_DEFS) - se builderne over.
    const MOUNTAIN_EASTER_EGGS = {
        0: buildLoneHiker,
        1: buildNorwegianFlag,
        2: buildGeocache,
        3: buildMountainTroll,
        4: buildTrailSignpost,
        5: buildSnowman,
        6: buildSummitCairn,
        7: buildMountainCabin
    };

    // Bygget fra MOUNTAIN_PEAKS (samme datasett som mountainHeightAt bruker til kollisjon) - én
    // sammenhengende geometri fra bakkeplanet (frac 0) helt til toppen (frac 1) per kjegle, ingen egen
    // "fot"-mesh. Det fjerner den synlige skjøten/ringen der en separat fot- og topp-mesh møttes, OG gir
    // en jevn fargeovergang fra selveste bakkefargen (matcher Sim.buildGroundTexture) via oliven-
    // fjellfot og gråstein til ev. snø, i stedet for et brått fargehopp ved bakken.
    MOUNTAIN_PEAKS.forEach(function (peak) {
        const colorStops = (peak.isMain && peak.snow)
            ? [
                { frac: 0, color: MOUNTAIN_GROUND_COLOR },
                { frac: 0.16, color: MOUNTAIN_FOOTHILL_COLOR },
                { frac: 0.42, color: MOUNTAIN_ROCK_COLOR },
                { frac: 0.82, color: MOUNTAIN_ROCK_LIGHT_COLOR },
                { frac: 1, color: MOUNTAIN_SNOW_COLOR }
            ]
            : [
                { frac: 0, color: MOUNTAIN_GROUND_COLOR },
                { frac: peak.isMain ? 0.18 : 0.2, color: MOUNTAIN_FOOTHILL_COLOR },
                { frac: 0.5, color: MOUNTAIN_ROCK_COLOR },
                { frac: 1, color: MOUNTAIN_ROCK_LIGHT_COLOR }
            ];
        const mesh = new THREE.Mesh(
            buildGradientPeakGeometry(
                peak.radius, peak.height, peak.seed, colorStops,
                peak.jaggedness, peak.topRadiusFrac, peak.curvePower, peak.noiseFreqMul
            ),
            peakMat
        );
        mesh.position.set(peak.x, peak.height / 2, peak.z);
        mesh.rotation.y = peak.angle;
        group.add(mesh);

        // Påskeegg rett på toppunktet (kun hovedtopper) - snudd til å vende mot spillområdet (origo),
        // som om det venter på en nysgjerrig pilot. Y beregnes eksakt via peakApexYOffset (samme seed
        // og jaggedness som toppdekket over), pluss en liten klaring for å unngå z-fighting mot selve
        // fjelloverflaten.
        const eggBuilder = peak.isMain && MOUNTAIN_EASTER_EGGS[peak.mainIndex];
        if (eggBuilder) {
            const egg = eggBuilder();
            const apexY = peak.height + peakApexYOffset(peak.height, peak.seed, peak.jaggedness) + 0.04;
            egg.position.set(peak.x, apexY, peak.z);
            egg.rotation.y = peak.angle + Math.PI;
            group.add(egg);
        }
    });
    return group;
}

// Fjellenes kollisjonsflate - gjenbruker den glatte profilkurven fra geometrien (mountainProfileHeightFrac
// er inversen av mountainProfileRadiusFrac som selve meshen bygges med), MEN uten finjitteret vertex for
// vertex (samme forenkling som SOLID_COLLIDERS - en glatt tilnærming er mer enn god nok). Radiusen blåses
// derfor opp litt utover selve profilkurven (JITTER_SAFETY_MARGIN * jaggedness) - den synlige overflaten
// buler utover med opptil ~28 % i steinete partier pga. vinkelavhengig jitter (se buildGradientPeakGeometry),
// og uten denne marginen kunne droneen synke synlig gjennom fjellsiden før den glatte, litt INNENFOR
// liggende profilen faktisk stanset den. Et par ekstra centimeter "usynlig vegg" et stykke fra en
// bergknaus er et mye mindre problem enn å fly rett gjennom synlig stein.
const JITTER_SAFETY_MARGIN = 0.25;
function mountainHeightAt(x, z) {
    let top = 0;
    for (let i = 0; i < MOUNTAIN_PEAKS.length; i++) {
        const peak = MOUNTAIN_PEAKS[i];
        const collisionRadius = peak.radius * (1 + JITTER_SAFETY_MARGIN * peak.jaggedness);
        const dist = Math.hypot(x - peak.x, z - peak.z);
        if (dist >= collisionRadius) continue;
        const h = peak.height * mountainProfileHeightFrac(dist / collisionRadius, peak.topRadiusFrac, peak.curvePower);
        if (h > top) top = h;
    }
    return top;
}

// Skogsområde med høyere trær (10-16 m) enn de spredte dekorasjonstrærne i buildWorldObjects (6-8,5 m) -
// en egen bakgrunnsflekk et godt stykke unna bane/bygninger/øvelser. Jitter fra sin/cos av indeksen i
// stedet for Math.random(), samme determinisme-prinsipp som resten av verdensbyggingen.
function buildForestArea() {
    const group = new THREE.Group();
    FOREST_TREES.forEach(function (t) {
        const tree = Sim.buildRandomTree(t.h);
        tree.position.set(t.x, 0, t.z);
        group.add(treeSwayManager.addSwayingTree(tree));
    });
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
// transparent+opacity gir en lett, dis-aktig kant i stedet for en helt solid, hard silhuett -
// depthWrite er fortsatt PÅ som standard (selv med transparent:true), som holder overlappende puffer
// noenlunde riktig sortert i stedet for å bli fullt gjennomsiktighets-sortert (det ga synlige glitch).
const CLOUD_SHADE_MATERIALS = [0xff, 0xeb, 0xd7, 0xc3].map(function (shade) {
    return new THREE.MeshStandardMaterial({
        color: (shade << 16) | (shade << 8) | shade, roughness: 0.9,
        transparent: true, opacity: 0.95
    });
});
// ÉN sammenhengende, myk kule i stedet for flere separate, overlappende sfærer - uansett hvor tett
// puffer overlapper vil to separate kule-meshes alltid lage en synlig fure der overflatene møtes
// (det ble tydelig som mange små "furer"/skiller mellom ballene i forrige forsøk). Ekte cumulus har
// en kontinuerlig, mykt bulkete overflate uten harde skiller - derfor perturberes radiusen på plass,
// akkurat som fjellenes uregelmessighet (buildGradientPeakGeometry), men med mye LAVERE amplitude
// (myke bulker, ikke en taggete silhuett) og lav frekvens (noen få brede, myke lober, ikke mange
// spisse tagger). Flat, dempet bunn (kondensasjonsnivået) er baket inn på samme måte som fjellenes fot.
function buildCloudBlobGeometry(radius, seed) {
    const geo = new THREE.SphereGeometry(radius, 14, 10);
    const pos = geo.attributes.position;
    const flattenY = -radius * 0.22;
    for (let i = 0; i < pos.count; i++) {
        const x = pos.getX(i), y = pos.getY(i), z = pos.getZ(i);
        const angle = Math.atan2(z, x);
        const heightFrac = clamp((y + radius) / (radius * 2), 0, 1);
        // Et par brede lober (lav frekvens) pluss ørlite finstruktur (høyere frekvens, lav amplitude) -
        // roligere nederst (nær den flate bunnen) enn mot toppen, for en kupert, ikke helt jevn topp.
        const lobes = Math.sin(angle * 3 + seed) * 0.14 + Math.sin(angle * 2 + seed * 2.7) * 0.09;
        const texture = Math.sin(angle * 9 + seed * 3.1) * 0.035 + Math.sin(y * 2.2 + seed * 5.3) * 0.03;
        const bump = 1 + (lobes + texture) * (0.4 + heightFrac * 0.6);
        pos.setX(i, x * bump);
        pos.setZ(i, z * bump);
        if (y < flattenY) pos.setY(i, flattenY);
    }
    geo.computeVertexNormals();
    return geo;
}
function buildCloudCluster(seed) {
    const mat = CLOUD_SHADE_MATERIALS[seed % CLOUD_SHADE_MATERIALS.length]; // gråtone-variasjon ("varierte skyer")
    const radius = 3.2 + Math.abs(Math.sin(seed * 1.3)) * 1.6;
    const mesh = new THREE.Mesh(buildCloudBlobGeometry(radius, seed), mat);
    // Bredere enn høy (typisk cumulus-silhuett), og litt avlang i planet (ikke perfekt sirkulær sett
    // ovenfra) for et mer organisk, variert utseende mellom klyngene.
    mesh.scale.set(1 + Math.abs(Math.sin(seed * 2.1)) * 0.4, 0.6, 1 + Math.abs(Math.cos(seed * 1.8)) * 0.35);
    const group = new THREE.Group();
    group.add(mesh);
    return group;
}
function buildClouds() {
    const group = new THREE.Group();
    // Høyt nok til at 100% skydekke faktisk ser fullt overskyet ut, ikke bare noen spredte klynger -
    // se coverageStep-fordelingen i updateClouds(), som viser en jevnt fordelt DEL av disse.
    const CLOUD_COUNT = 60;
    // Jevnt rutenett over hele skydomenet (IKKE en sirkel rundt origo, som var forrige oppsett) - med
    // litt tilfeldig forskyvning per rute for naturlig variasjon uten at selve rutenettet blir synlig.
    // Gir omtrent lik skytetthet over hele kartet (bare små, lokale avvik) i stedet for at skyene
    // klumper seg rundt sentrum og etterlater hjørnene/kantene av kartet skyfrie.
    const gridCols = 8;
    const gridRows = Math.ceil(CLOUD_COUNT / gridCols);
    const cellW = CLOUD_DOMAIN / gridCols, cellH = CLOUD_DOMAIN / gridRows;
    for (let i = 0; i < CLOUD_COUNT; i++) {
        const cluster = buildCloudCluster(i);
        const gx = i % gridCols, gz = Math.floor(i / gridCols);
        const x = -CLOUD_DOMAIN / 2 + (gx + 0.5) * cellW + (Math.random() - 0.5) * cellW * 0.8;
        const z = -CLOUD_DOMAIN / 2 + (gz + 0.5) * cellH + (Math.random() - 0.5) * cellH * 0.8;
        cluster.position.set(x, 140 + Math.abs(Math.cos(i * 1.3)) * 90, z);
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

// Folkemengde ved bilen - brukt som "publikum"-faresonen i killswitch-øvelsen (ex11). Gjenbruker
// Sim.buildPersonFigure (samme figur som VLOS-observatøren) med ulike klesfarger for variasjon i
// stedet for en egen modell - jitteret er deterministisk (sin/cos av indeksen), samme prinsipp som
// resten av verdensbyggingen (skog, fjell).
const CROWD_SHIRT_COLORS = [0x3f6fb0, 0xb0473f, 0x4fae6a, 0xd0a83a, 0x7a4fae, 0xd0703a, 0x3aa8c0];
const CROWD_CENTER = new THREE.Vector3(15.5, 0, 9.5); // foran bilen (24,0,14), sett fra avgangsplassen - se buildWorldObjects
// Delt kilde for BÅDE det visuelle jitteret (buildCrowd) og kollisjonssjekken (updateBystanderCollision) -
// slik at "hvor personene faktisk står" og "hvor de kan bli truffet" aldri kan komme ut av synk.
const CROWD_MEMBER_OFFSETS = CROWD_SHIRT_COLORS.map(function (_, i) {
    return { x: Math.sin(i * 12.9) * 1.7, z: Math.cos(i * 7.3) * 1.7 };
});
function buildCrowd() {
    const group = new THREE.Group();
    CROWD_SHIRT_COLORS.forEach(function (color, i) {
        const person = Sim.buildPersonFigure({ vestColor: color });
        const off = CROWD_MEMBER_OFFSETS[i];
        person.position.set(off.x, 0, off.z);
        person.rotation.y = (Math.sin(i * 5.1) * 0.5 + 0.5) * Math.PI * 2;
        group.add(person);
    });
    return group;
}

// Retter et objekt slik at dets lokale -Z-akse (samme "forover er -Z"-konvensjon som droneen selv og
// buildPlane i fixed-wing-simulatoren) peker langs reisevektoren from->to. Brukt til å snu helikopteret
// og flyet langs sin egen bane, samme idé som buildStrutBetween sin setFromUnitVectors-teknikk.
function orientTowardTravel(object3d, from, to) {
    const dir = new THREE.Vector3().subVectors(to, from).normalize();
    if (dir.lengthSq() < 1e-8) return;
    object3d.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, -1), dir);
}

// Enkelt prosedural helikopter - kun brukt som lavtflyvende luftfarts-farescenario i killswitch-øvelsen
// (ex11), ikke en flybar enhet. Skala/detaljnivå er tilpasset å leses tydelig fra bakken på ~15 m høyde,
// ikke et nærbilde-objekt. rotor/tailRotor-håndtakene brukes til å spinne rotorene i updateKillswitchVisuals.
function buildHelicopter() {
    const group = new THREE.Group();
    const bodyMat = new THREE.MeshStandardMaterial({ color: 0xcc4422 });
    const glassMat = new THREE.MeshStandardMaterial({ color: 0x223040, transparent: true, opacity: 0.75 });
    const darkMat = new THREE.MeshStandardMaterial({ color: 0x1a1a1a });

    const fuselage = new THREE.Mesh(new THREE.SphereGeometry(1, 10, 8), bodyMat);
    fuselage.scale.set(1, 0.75, 1.7);
    group.add(fuselage);

    const canopy = new THREE.Mesh(new THREE.SphereGeometry(0.55, 8, 6), glassMat);
    canopy.scale.set(1, 0.8, 1.1);
    canopy.position.set(0, 0.15, -0.9); // nese/cockpit fremover, lokal -Z
    group.add(canopy);

    const tailBoom = new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.32, 3.4, 8), bodyMat);
    tailBoom.rotation.x = Math.PI / 2;
    tailBoom.position.set(0, 0.15, 2.6); // hale bakover, lokal +Z
    group.add(tailBoom);

    const tailFin = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.9, 0.7), bodyMat);
    tailFin.position.set(0, 0.75, 4.1);
    group.add(tailFin);
    // Halerotor - flat, halvgjennomsiktig skive i stedet for animerte blad (for langt unna til at
    // enkeltblad ville vært synlige uansett - antyder bevegelsesuskarphet uten animasjonskostnad).
    const tailRotorDisc = new THREE.Mesh(new THREE.CircleGeometry(0.5, 12), new THREE.MeshStandardMaterial({ color: 0x1a1a1a, transparent: true, opacity: 0.35 }));
    tailRotorDisc.rotation.y = Math.PI / 2;
    tailRotorDisc.position.set(0.1, 0.75, 4.15);
    group.add(tailRotorDisc);

    [-0.75, 0.75].forEach(function (side) {
        const skid = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 3.4, 6), darkMat);
        skid.rotation.x = Math.PI / 2;
        skid.position.set(side, -0.75, 0);
        group.add(skid);
    });

    const rotorHub = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.08, 0.15, 8), darkMat);
    rotorHub.position.set(0, 0.85, -0.2);
    group.add(rotorHub);
    const mainRotor = new THREE.Group();
    mainRotor.position.copy(rotorHub.position);
    [0, Math.PI / 2].forEach(function (a) {
        const blade = new THREE.Mesh(new THREE.BoxGeometry(5.4, 0.03, 0.18), darkMat);
        blade.rotation.y = a;
        mainRotor.add(blade);
    });
    group.add(mainRotor);

    group.traverse(function (obj) { if (obj.isMesh) obj.castShadow = true; });
    group.visible = false;
    return { group: group, rotor: mainRotor };
}

// Enkelt prosedural fly (liten rutefly-/transportsilhuett) - samme "kun leselig som fjern silhuett"-
// ambisjon som buildHelicopter, brukt som lufttrafikk-farescenario høyt oppe i ex11.
function buildAirplane() {
    const group = new THREE.Group();
    const bodyMat = new THREE.MeshStandardMaterial({ color: 0xe8e8ec });
    const accentMat = new THREE.MeshStandardMaterial({ color: 0x2255aa });

    const fuselage = new THREE.Mesh(new THREE.CylinderGeometry(0.5, 0.35, 8, 10), bodyMat);
    fuselage.rotation.x = Math.PI / 2;
    group.add(fuselage);

    const nose = new THREE.Mesh(new THREE.ConeGeometry(0.5, 1.2, 10), bodyMat);
    nose.rotation.x = Math.PI / 2;
    nose.position.z = -4.6; // nese fremover, lokal -Z
    group.add(nose);

    const wing = new THREE.Mesh(new THREE.BoxGeometry(11, 0.15, 1.6), accentMat);
    wing.position.set(0, -0.1, -0.3);
    group.add(wing);

    const tailWing = new THREE.Mesh(new THREE.BoxGeometry(4, 0.12, 1), accentMat);
    tailWing.position.set(0, 0.3, 3.7);
    group.add(tailWing);

    const fin = new THREE.Mesh(new THREE.BoxGeometry(0.12, 1.4, 1.4), accentMat);
    fin.position.set(0, 0.9, 3.9);
    group.add(fin);

    group.traverse(function (obj) { if (obj.isMesh) obj.castShadow = true; });
    group.visible = false;
    return group;
}

// Et par fotgjengere som går rett mot flygeområdet - scenario 4 i killswitch-øvelsen (ex11). Gjenbruker
// Sim.buildPersonFigure (samme figur som folkemengden/VLOS-observatøren), to farger for litt variasjon.
function buildPedestrianGroup() {
    const group = new THREE.Group();
    [-1, 1].forEach(function (side) {
        const person = Sim.buildPersonFigure({ vestColor: side < 0 ? 0x3f6fb0 : 0xb0473f });
        person.position.x = side * 0.9;
        group.add(person);
    });
    group.visible = false;
    return group;
}

// Samme idé som orientTowardTravel, men for Sim.buildPersonFigure - den er bygget med front mot LOKAL
// +Z (se kommentaren ved figurens VLOS-bruk i initScene: "figuren bygges med tærne mot +Z"), motsatt av
// kjøretøy-konvensjonen (-Z) orientTowardTravel selv bruker. Snur derfor et halvt ekstra hjørne.
function orientPersonGroupTowardTravel(object3d, from, to) {
    orientTowardTravel(object3d, from, to);
    object3d.rotateY(Math.PI);
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

// Delt tredata - samme prinsipp som MOUNTAIN_PEAKS: bygget én gang, brukt til BÅDE rendering
// (buildWorldObjects/buildForestArea) og kollisjon (TREE_COLLIDERS under), så de aldri kan komme ut
// av synk. Alle godt utenfor øvelsesområdet (se koordinatene) - ingen risiko for å forstyrre eksisterende
// baner/øvelser.
const DECORATIVE_TREES = [
    { x: 45, z: -20, h: 7 }, { x: 55, z: 5, h: 8 }, { x: 40, z: 30, h: 6.5 },
    { x: -50, z: 20, h: 7.5 }, { x: -20, z: -55, h: 8.5 }, { x: 15, z: -60, h: 6 },
    { x: 70, z: -40, h: 7.2 }, { x: -60, z: -10, h: 6.8 }
];
const FOREST_TREES = (function () {
    const trees = [];
    const centerX = 140, centerZ = 90, rows = 6, cols = 6, spacing = 11;
    let i = 0;
    for (let r = 0; r < rows; r++) {
        for (let c = 0; c < cols; c++) {
            const jitterX = Math.sin(i * 12.9) * 4;
            const jitterZ = Math.cos(i * 7.3) * 4;
            const height = 10 + Math.abs(Math.sin(i * 3.7)) * 6;
            trees.push({ x: centerX + (c - cols / 2) * spacing + jitterX, z: centerZ + (r - rows / 2) * spacing + jitterZ, h: height });
            i++;
        }
    }
    return trees;
})();
// Trekollidere - TRE bokser per tre (stamme + to kroneband) i stedet for én flat søyle fra bakken opp
// til toppen (som ga en merkbart FOR BRED kollider nede ved stammen uansett hvor smal radiusen ble satt,
// siden samme bredde da gjaldt helt fra bakken og opp). Se minY - støttet av
// solidSurfaceHeightAt/pushOutOfSolidWalls/resolveGroundContact, se orientedBoxLocalXZ-kommentaren for
// samme type utvidelse tidligere.
// trunkR er satt til den faktiske maks. stammeradiusen (0.15, se CylinderGeometry i
// Sim.buildBirch/buildPine), ingen ekstra sikkerhetsmargin oppå (all margin der ble opplevd som "usynlig
// kollisjon"). Selve krona er IKKE en jevn søyle heller - den er bygget av runde/koniske klynger (se
// buildBirch/buildPine) som er SMALEST akkurat der de starter (bunnen av en kule/kjegle) og videst et
// stykke lenger opp, ikke fullbredde med det samme. Ett rett hopp fra stamme til full kronebredde ga
// akkurat DEN feilen andre veien - "kollisjon i løse lufta under krona", altså full bredde et stykke FØR
// selve løvverket faktisk er der. To kroneband (et smalere overgangsband, så det fulle) er en grovere,
// men mye bedre tilnærming til den avrundede formen enn ett enkelt hopp.
function treeToColliders(t) {
    const trunkTopY = t.h * 0.48;
    const canopyTaperTopY = trunkTopY + (t.h - trunkTopY) * 0.35;
    const trunkR = 0.15;
    const canopyR = t.h * 0.22;
    const taperR = canopyR * 0.5;
    return [
        { minX: t.x - trunkR, maxX: t.x + trunkR, minZ: t.z - trunkR, maxZ: t.z + trunkR, minY: 0, topY: trunkTopY },
        { minX: t.x - taperR, maxX: t.x + taperR, minZ: t.z - taperR, maxZ: t.z + taperR, minY: trunkTopY, topY: canopyTaperTopY },
        { minX: t.x - canopyR, maxX: t.x + canopyR, minZ: t.z - canopyR, maxZ: t.z + canopyR, minY: canopyTaperTopY, topY: t.h }
    ];
}
const TREE_COLLIDERS = DECORATIVE_TREES.concat(FOREST_TREES).reduce(function (acc, t) {
    return acc.concat(treeToColliders(t));
}, []);

// Faste objekter droneen kan lande oppå (i stedet for å falle gjennom): topp-flate per boks,
// oppgitt akse-rettet (bilens rotasjon tilnærmes med en litt større boks for enkelhets skyld).
// To former støttes: akse-rettet (minX/maxX/minZ/maxZ, som over) for ting som uansett aldri roterer, og
// ORIENTERT (cx/cz/halfW/halfD/yaw - se orientedBoxLocalXZ) for roterte bygninger som rådhuset/husene i
// racingbane 2 (pushet inn senere, se buildGateCourse2) - der ville en akse-rettet boks stor nok til å
// garantert dekke hele det roterte footprinten (f.eks. halve DIAGONALEN i begge retninger) blitt merkbart
// større enn selve bygningen, og kollisjonen ville trigget lenge før droneen faktisk nådde veggen.
const SOLID_COLLIDERS = TREE_COLLIDERS.concat([
    {
        minX: BUILDING_POSITION.x - BUILDING_SIZE.width / 2, maxX: BUILDING_POSITION.x + BUILDING_SIZE.width / 2,
        minZ: BUILDING_POSITION.z - BUILDING_SIZE.depth / 2, maxZ: BUILDING_POSITION.z + BUILDING_SIZE.depth / 2,
        topY: BUILDING_SIZE.height
    },
    { minX: 24 - 2.33, maxX: 24 + 2.33, minZ: 14 - 1.58, maxZ: 14 + 1.58, topY: 1.7 } // bilen, se buildWorldObjects
]);

// Verdenspunkt (x,z) -> punktets koordinater i boksens LOKALE, urotere rom (senter = origo, lokal +Z =
// boksens dybde-akse) - samme yaw-konvensjon som resten av banen (se GATE_PLACEMENTS-kommentaren: lokal
// +Z er "forover" etter rotasjon rundt Y med vinkelen yaw). Utledet ved å invertere (= transponere, siden
// rotasjonsmatrisen er ortogonal) fremover-transformen world = senter + R(yaw)*lokal.
function orientedBoxLocalXZ(x, z, c) {
    const dx = x - c.cx, dz = z - c.cz;
    const cosA = Math.cos(c.yaw), sinA = Math.sin(c.yaw);
    return { lx: dx * cosA - dz * sinA, lz: dx * sinA + dz * cosA };
}
// Motsatt vei av over - et lokalt (lx,lz)-punkt tilbake til verdensrommet, brukt til å plassere et
// dyttet punkt (se pushOutOfSolidWalls) tilbake på riktig sted etter at det er klemt til boksens kant i
// det lokale rommet.
function orientedBoxWorldFromLocal(lx, lz, c) {
    const cosA = Math.cos(c.yaw), sinA = Math.sin(c.yaw);
    return { x: c.cx + lx * cosA + lz * sinA, z: c.cz - lx * sinA + lz * cosA };
}

function solidSurfaceHeightAt(x, z) {
    let top = mountainHeightAt(x, z); // fjellene er nå del av bakkehøyden - se mountainHeightAt
    SOLID_COLLIDERS.forEach(function (c) {
        if (c.yaw !== undefined) {
            const p = orientedBoxLocalXZ(x, z, c);
            if (Math.abs(p.lx) <= c.halfW && Math.abs(p.lz) <= c.halfD) top = Math.max(top, c.topY);
        } else if (x >= c.minX && x <= c.maxX && z >= c.minZ && z <= c.maxZ) {
            top = Math.max(top, c.topY);
        }
    });
    return top;
}
// Er (x,y,z) fysisk INNI en solid kollider - IKKE bare "under toppen et sted i søylen", slik
// solidSurfaceHeightAt over ignorerer med vilje (den svarer "hva ville jeg landet på om jeg falt her fra
// himmelen", uavhengig av hvor jeg faktisk befinner meg akkurat nå). Denne respekterer minY (se
// treeToColliders) - brukt av propPointHitsObstacle for propellskade: uten minY-sjekk her ville en
// trekrone som starter langt over bakken (bredere enn stammen, se treeToColliders) telt som "solid" helt
// ned til bakken for EN PROPELL som passerer i lav høyde langt fra selve stammen, siden
// solidSurfaceHeightAt bare bryr seg om XZ-fotavtrykket, ikke hvor høyt akkurat DENNE bestemte boksen
// faktisk starter. Det var den egentlige årsaken til "propellene ødelegges lenge før stammen".
function pointInsideAnySolidCollider(x, y, z) {
    for (let i = 0; i < SOLID_COLLIDERS.length; i++) {
        const c = SOLID_COLLIDERS[i];
        if (y < (c.minY || 0) || y > c.topY) continue;
        if (c.yaw !== undefined) {
            const p = orientedBoxLocalXZ(x, z, c);
            if (Math.abs(p.lx) <= c.halfW && Math.abs(p.lz) <= c.halfD) return true;
        } else if (x >= c.minX && x <= c.maxX && z >= c.minZ && z <= c.maxZ) {
            return true;
        }
    }
    return false;
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
        if (c.yaw !== undefined) {
            const loc = orientedBoxLocalXZ(point.x, point.z, c);
            if (Math.abs(loc.lx) > c.halfW || Math.abs(loc.lz) > c.halfD) return;
            if (point.y < (c.minY || 0) || point.y >= c.topY - GROUND_CLEARANCE) return;
            embedded = true;
            const distMinX = loc.lx + c.halfW, distMaxX = c.halfW - loc.lx;
            const distMinZ = loc.lz + c.halfD, distMaxZ = c.halfD - loc.lz;
            const minDist = Math.min(distMinX, distMaxX, distMinZ, distMaxZ);
            let newLx = loc.lx, newLz = loc.lz, pushDirLocalX = 0, pushDirLocalZ = 0;
            if (minDist === distMinX) { newLx = -c.halfW; pushDirLocalX = -1; }
            else if (minDist === distMaxX) { newLx = c.halfW; pushDirLocalX = 1; }
            else if (minDist === distMinZ) { newLz = -c.halfD; pushDirLocalZ = -1; }
            else { newLz = c.halfD; pushDirLocalZ = 1; }
            const w = orientedBoxWorldFromLocal(newLx, newLz, c);
            point.x = w.x;
            point.z = w.z;
            // Konverter lokal push-retning til en verdens-enhetsvektor (samme fremover-rotasjon som
            // orientedBoxWorldFromLocal, uten senterforskyvningen - kun retningen er interessant her) -
            // nullstiller kun hastighetskomponenten INN i veggen, resten av bevegelsen langs veggen
            // beholdes (samme "gli langs veggen"-følelse som den akse-rettede varianten under).
            const cosA = Math.cos(c.yaw), sinA = Math.sin(c.yaw);
            const worldDirX = pushDirLocalX * cosA + pushDirLocalZ * sinA;
            const worldDirZ = -pushDirLocalX * sinA + pushDirLocalZ * cosA;
            const vDot = velocity.x * worldDirX + velocity.z * worldDirZ;
            if (vDot < 0) {
                velocity.x -= vDot * worldDirX;
                velocity.z -= vDot * worldDirZ;
            }
            return;
        }
        if (point.x < c.minX || point.x > c.maxX || point.z < c.minZ || point.z > c.maxZ) return;
        // minY (satt for trekollidere - se treeToColliders) - en boks som starter over bakken (kronen)
        // skal IKKE regnes som "innfelt i" så snart droneen er et sted langt under den (nær bakken/
        // stammen); uten denne nedre grensen ville krone-boksen (som er bredere enn stamme-boksen)
        // fortsatt trigget helt ned til bakken, og gjeninnført akkurat problemet stamme-/krone-delingen
        // over skulle løse.
        if (point.y < (c.minY || 0) || point.y >= c.topY - GROUND_CLEARANCE) return;
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

// Sjakkrutet stolpe/bar bygget av vekslende fargede segmenter - delt mellom buildGate (oransje/hvit)
// og buildStartFinishGate (svart/hvit, tettere rutemønster). horizontal styrer om segmentene forskyves
// langs lokal X (topp-/bunnbar) eller Y (side-stolper).
function buildCheckeredBar(length, horizontal, matA, matB, barThickness, segs) {
    const segLen = length / segs;
    const barGroup = new THREE.Group();
    for (let i = 0; i < segs; i++) {
        const mat = (i % 2 === 0) ? matA : matB;
        const geo = horizontal
            ? new THREE.BoxGeometry(segLen, barThickness, barThickness)
            : new THREE.BoxGeometry(barThickness, segLen, barThickness);
        const seg = new THREE.Mesh(geo, mat);
        seg.castShadow = true;
        seg.receiveShadow = true;
        const offset = -length / 2 + segLen / 2 + i * segLen;
        if (horizontal) seg.position.x = offset; else seg.position.y = offset;
        barGroup.add(seg);
    }
    return barGroup;
}
const GATE_BAR_THICKNESS = 0.18;
function buildGateFrame(size, groundGap, matA, matB, segs) {
    const group = new THREE.Group();
    const legMat = new THREE.MeshStandardMaterial({ color: 0x333333 });

    const top = buildCheckeredBar(size, true, matA, matB, GATE_BAR_THICKNESS, segs);
    top.position.y = groundGap + size;
    group.add(top);

    const bottom = buildCheckeredBar(size, true, matA, matB, GATE_BAR_THICKNESS, segs);
    bottom.position.y = groundGap;
    group.add(bottom);

    const left = buildCheckeredBar(size, false, matA, matB, GATE_BAR_THICKNESS, segs);
    left.position.set(-size / 2, groundGap + size / 2, 0);
    group.add(left);

    const right = buildCheckeredBar(size, false, matA, matB, GATE_BAR_THICKNESS, segs);
    right.position.set(size / 2, groundGap + size / 2, 0);
    group.add(right);

    [-size / 2, size / 2].forEach(function (x) {
        const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.08, groundGap, 8), legMat);
        leg.position.set(x, groundGap / 2, 0);
        leg.castShadow = true;
        leg.receiveShadow = true;
        group.add(leg);
    });

    return group;
}
const GATE_MAT_ORANGE = new THREE.MeshStandardMaterial({ color: 0xff6a00 });
const GATE_MAT_WHITE = new THREE.MeshStandardMaterial({ color: 0xffffff });
// Firkantet racing-gate med sjakkrutet ramme (oransje/hvit), montert på to bein over bakken.
function buildGate(size, groundGap) {
    return buildGateFrame(size, groundGap, GATE_MAT_ORANGE, GATE_MAT_WHITE, 6);
}

const GATE_MAT_BLACK = new THREE.MeshStandardMaterial({ color: 0x111111 });
// Start/mål-gate for racingbanen - svart/hvitt rutemønster (tettere enn de vanlige oransje/hvite
// portene, for å ligne et ekte målflagg) pluss en gul pil montert over toppbaren som peker i
// flyretningen (lokal +Z - se yaw-formelen ved GATE_PLACEMENTS: samme akse som resten av banen flys mot).
function buildStartFinishGate(size, groundGap) {
    const group = buildGateFrame(size, groundGap, GATE_MAT_BLACK, GATE_MAT_WHITE, 8);
    const arrowMat = new THREE.MeshStandardMaterial({ color: 0xffee55 });
    const arrowY = groundGap + size + 0.55;
    const shaftLen = size * 0.5;
    const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, shaftLen, 6), arrowMat);
    shaft.rotation.x = Math.PI / 2; // sylinderens lengdeakse (normalt +Y) vippes til å peke langs +Z
    shaft.position.set(0, arrowY, shaftLen / 2);
    shaft.castShadow = true;
    group.add(shaft);
    const tip = new THREE.Mesh(new THREE.ConeGeometry(0.16, 0.5, 8), arrowMat);
    tip.rotation.x = Math.PI / 2; // samme vipp - konusspissen peker videre langs +Z
    tip.position.set(0, arrowY, shaftLen + 0.25);
    tip.castShadow = true;
    group.add(tip);
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

/* ---------- Rådhus med klokketårn (racingbane 2 - se GATE_COURSE_2_CENTER) ----------
   Samme idé som "rådhuset" i fixed-wing-simulatoren (bygget på nytt her, ikke delt via
   simulator-common.js - de to simulatorene har egne verdener og ingen andre felles bygningstyper):
   murstein-tekstur, flatt tak (til racing-gaten som står der, se GATE_WAYPOINTS_2), urskiver som følger
   PC-ens faktiske klokkeslett på alle fire sider av et smalt tårn. */
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
// Norsk flagg (forhold 22:16, korset forskjøvet mot stangsiden) - prosedural canvas-tekstur, samme
// prinsipp som buildBrickTexture/buildClockTexture. Bygget én gang og gjenbrukt (som de andre) - kun ett
// flagg i verden akkurat nå, men samme cache-mønster uansett.
let norwayFlagTextureBase = null;
function buildNorwayFlagTexture() {
    if (norwayFlagTextureBase) return norwayFlagTextureBase;
    const w = 220, h = 160;
    const canvas = document.createElement("canvas");
    canvas.width = w;
    canvas.height = h;
    const ctx = canvas.getContext("2d");
    ctx.fillStyle = "#ba0c2f";
    ctx.fillRect(0, 0, w, h);
    ctx.fillStyle = "#ffffff";
    ctx.fillRect(0, 60, w, 40);  // hvit horisontal stripe
    ctx.fillRect(40, 0, 40, h); // hvit vertikal stripe (nær stangsiden, venstre kant)
    ctx.fillStyle = "#00205b";
    ctx.fillRect(0, 70, w, 20);  // blå horisontal stripe
    ctx.fillRect(50, 0, 20, h); // blå vertikal stripe
    norwayFlagTextureBase = new THREE.CanvasTexture(canvas);
    return norwayFlagTextureBase;
}
// Vindpåvirkede flaggduk-håndtak (kun rotasjonspivoten, se updateFlags) - fylt av buildTownHallFlag.
const flagHandles = [];
// Flaggstang + duk, ment for TAKET (ikke veggfasaden som før) - se plasseringen i buildTownHall under.
// "pivot" (returnert som andre del) er den delen som roterer med vindretningen (se updateFlags) - selve
// stangen står fast, kun duken svinger, som på ekte.
function buildTownHallFlag(poleHeight, flagWidth, flagHeight) {
    const group = new THREE.Group();
    const poleMat = new THREE.MeshStandardMaterial({ color: 0xcccccc });
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.09, poleHeight, 8), poleMat);
    pole.position.y = poleHeight / 2;
    pole.castShadow = true;
    group.add(pole);

    const pivot = new THREE.Group();
    pivot.position.y = poleHeight - flagHeight * 0.6;
    group.add(pivot);

    // To duk-plan (ikke ett dobbeltsidig) - et enkelt DoubleSide-plan ville vist korset SPEILVENDT (altså
    // feil side av stangen) fra baksiden, siden korset ikke er symmetrisk (forskjøvet mot stangsiden).
    // Baksiden roteres 180° OG speilvendes (scale.x=-1) - rotasjonen alene ville speilvendt mønsteret,
    // scale.x kompenserer akkurat det tilbake, slik at begge sider viser korrekt, uspeilet flagg.
    const flagMat = new THREE.MeshStandardMaterial({ map: buildNorwayFlagTexture() });
    const front = new THREE.Mesh(new THREE.PlaneGeometry(flagWidth, flagHeight), flagMat);
    front.position.x = flagWidth / 2 + 0.05;
    front.castShadow = true;
    pivot.add(front);
    const back = new THREE.Mesh(new THREE.PlaneGeometry(flagWidth, flagHeight), flagMat);
    back.position.x = flagWidth / 2 + 0.05;
    back.rotation.y = Math.PI;
    back.scale.x = -1;
    pivot.add(back);

    flagHandles.push(pivot);
    return group;
}
// Strupet oppdatering (se lastFlagUpdateMs) - flagget skal se levende ut (litt raskere enn
// klokketårnet/treet), men trenger ikke oppdateres hvert eneste bilde. IKKE ekte duk-simulering - en
// "værhane"-rotasjon mot vindretningen (som vindpølsen, se Sim.updateWindsockVisual) pluss en liten
// sinus-"flagre" oppå, som leser fint som vindpåvirket på avstand uten kostnaden ved ekte klut-fysikk.
let lastFlagUpdateMs = 0;
function updateFlags(now) {
    if (flagHandles.length === 0 || now - lastFlagUpdateMs < 60) return;
    lastFlagUpdateMs = now;
    const windSpeed = currentWindVector.length();
    const baseYaw = windSpeed > 0.05 ? Math.atan2(currentWindVector.x, currentWindVector.z) : 0;
    const flutter = Math.sin(now / 180) * 0.08 * Math.min(1, windSpeed / 3 + 0.15);
    flagHandles.forEach(function (pivot) {
        pivot.rotation.y = baseYaw + flutter;
    });
}
function buildBrickMaterial(repeatX, repeatY) {
    const tex = buildBrickTexture().clone();
    tex.needsUpdate = true;
    tex.wrapS = THREE.RepeatWrapping;
    tex.wrapT = THREE.RepeatWrapping;
    tex.repeat.set(repeatX, repeatY);
    return new THREE.MeshStandardMaterial({ map: tex });
}

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

// Klokkehendene (dreiepunkt-grupper, ikke selve viser-meshene) registreres her slik at
// updateClockTower kan rotere dem fra PC-ens klokke hvert bilde - samme "handles"-mønster som
// windsockHandle/treeSwayManager.
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
// Strupet til maks én reell oppdatering i sekundet - viserbevegelsen er umerkelig raskere enn det
// (samme resonnement som treet/vindpølsen sine egne throttlede oppdateringer).
let lastClockTowerUpdateMs = 0;
function updateClockTower(nowMs) {
    if (clockHandles.length === 0 || nowMs - lastClockTowerUpdateMs < 1000) return;
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

// Rådhuset - murstein, flatt tak (racing-gaten fra GATE_WAYPOINTS_2 lander midt på det), store vinduer,
// søyleinngang, klokketårn og flaggstang.
// Bredere/dypere enn "ekte" rådhus-proporsjoner ville tilsagt - taket må ha god nok plass til racing-
// gaten (se GATE_WAYPOINTS_2 "roofgate") PLUSS innflygings-/utflygingsrom rundt den, ikke bare så vidt.
// Økt videre fra 14x12 - gaten (midt på taket) og klokketårnet (se tower.position.set under) overlappet
// før denne økningen, siden tårnet bare sto rett bak sentrum. Tårnet er nå flyttet ut til et bakre hjørne
// i stedet (se lenger ned) - denne ekstra bredden gir god klaring der også.
const TOWNHALL_WIDTH = 18, TOWNHALL_HEIGHT = 10, TOWNHALL_DEPTH = 14;
// Roof-boksen under er sentrert på height+0.2 med 0.4 tykkelse (se roof.position.y/BoxGeometry) - selve
// OVERFLATEN (der droneen faktisk skal lande) er dermed height+0.4, IKKE height+0.2 (senteret). Brukte
// feilaktig senterhøyden her tidligere, som lot droneen synke ~0.2 m ned i selve takplaten før
// kollisjonen fanget den opp - retter det opp til å matche den visuelle toppflaten nøyaktig.
const TOWNHALL_ROOF_Y = TOWNHALL_HEIGHT + 0.4;
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
    roof.receiveShadow = true;
    group.add(roof);

    const winMat = new THREE.MeshStandardMaterial({ color: 0xbfe0e8, emissive: 0x3a5560, emissiveIntensity: 0.35 });
    const winCount = 4;
    for (let i = 0; i < winCount; i++) {
        const t = (i / (winCount - 1) - 0.5) * (width * 0.72);
        const win = new THREE.Mesh(new THREE.BoxGeometry(width * 0.11, height * 0.42, 0.06), winMat);
        win.position.set(t, height * 0.56, depth / 2 + 0.03);
        win.receiveShadow = true;
        group.add(win);
    }

    const doorMat = new THREE.MeshStandardMaterial({ color: 0x3a2a1a });
    const door = new THREE.Mesh(new THREE.BoxGeometry(width * 0.2, height * 0.55, 0.08), doorMat);
    door.position.set(0, height * 0.28, depth / 2 + 0.04);
    door.receiveShadow = true;
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

    // Flyttet ut til et bakre hjørne (i stedet for nær sentrum) - racing-gaten på taket (se
    // GATE_WAYPOINTS_2 "roofgate") flys gjennom midten av taket, og tårnet stod tidligere i veien for
    // den. Sentrum (lokal X=0) holdes dermed fritt for selve gaten.
    const tower = buildClockTower(width * 0.32, height * 1.6);
    tower.position.set(width * 0.3, height + 0.4, -depth * 0.32);
    group.add(tower);

    // Flaggstang på TAKET (ikke veggfasaden som før) - motsatt hjørne av tårnet, unna både gaten midt på
    // taket og tårnets eget footprint. Se buildTownHallFlag for selve duken (norsk flagg, vindpåvirket).
    const flagPole = buildTownHallFlag(height * 0.9, width * 0.16, width * 0.11);
    flagPole.position.set(-width * 0.32, height + 0.4, -depth * 0.3);
    group.add(flagPole);

    return group;
}

// Enkelt dekorativt bolighus (boks + pyramidetak) langs racingbane 2 - samme lavpoly-prinsipp som
// fjellhytta (buildMountainCabin) og barnet/rådhuset over, for et par "ledelinjer" til uten den fulle
// gavltak-detaljen fixed-wing-simulatorens husmodeller har (holdt enklere med vilje her).
function buildSimpleHouse2(width, height, depth, wallColor, roofColor) {
    const group = new THREE.Group();
    const wall = new THREE.Mesh(new THREE.BoxGeometry(width, height, depth), new THREE.MeshStandardMaterial({ color: wallColor }));
    wall.position.y = height / 2;
    wall.castShadow = true;
    wall.receiveShadow = true;
    group.add(wall);
    const roofHeight = height * 0.55;
    const roof = new THREE.Mesh(new THREE.ConeGeometry(Math.hypot(width, depth) * 0.5, roofHeight, 4), new THREE.MeshStandardMaterial({ color: roofColor }));
    roof.rotation.y = Math.PI / 4;
    roof.position.y = height + roofHeight / 2;
    roof.castShadow = true;
    roof.receiveShadow = true;
    group.add(roof);
    const winMat = new THREE.MeshStandardMaterial({ color: 0xbfe0e8, emissive: 0x3a5560, emissiveIntensity: 0.35 });
    const win = new THREE.Mesh(new THREE.BoxGeometry(width * 0.22, height * 0.28, 0.05), winMat);
    win.position.set(0, height * 0.55, depth / 2 + 0.03);
    win.receiveShadow = true;
    group.add(win);
    return group;
}

// En liten bro å fly under - rent landskapselement (ingen SOLID_COLLIDERS-registrering, se kommentaren
// ved plasseringen i buildGateCourse2) langs returleggen av racingbane 2, for variasjon i en ellers
// nokså rett strekning. "deckClearance" er høyden fra bakken til undersiden av dekket - selve
// gjennomflygingshøyden. Bygget flatt langs lokal X (spennet) med gjennomflyging langs lokal Z, akkurat
// som gate-rammene - roter hele gruppen med rotation.y for å rette den på tvers av banens retning.
function buildBridge(spanWidth, deckClearance, deckDepth) {
    const group = new THREE.Group();
    const deckMat = new THREE.MeshStandardMaterial({ color: 0x8a7a68, roughness: 0.85 });
    const pierMat = new THREE.MeshStandardMaterial({ color: 0x6e6e6e, roughness: 0.9 });
    const railMat = new THREE.MeshStandardMaterial({ color: 0x4a4a48 });
    const deckThickness = 0.7;

    const deck = new THREE.Mesh(new THREE.BoxGeometry(spanWidth, deckThickness, deckDepth), deckMat);
    deck.position.y = deckClearance + deckThickness / 2;
    deck.castShadow = true;
    deck.receiveShadow = true;
    group.add(deck);

    [-1, 1].forEach(function (side) {
        const rail = new THREE.Mesh(new THREE.BoxGeometry(spanWidth, 0.35, 0.1), railMat);
        rail.position.set(0, deckClearance + deckThickness + 0.2, side * deckDepth / 2);
        rail.castShadow = true;
        group.add(rail);
    });

    [-1, 1].forEach(function (side) {
        const pier = new THREE.Mesh(new THREE.CylinderGeometry(0.55, 0.7, deckClearance, 10), pierMat);
        pier.position.set(side * (spanWidth / 2 - 1.2), deckClearance / 2, 0);
        pier.castShadow = true;
        pier.receiveShadow = true;
        group.add(pier);
    });

    return group;
}

/* ---------- Elv (kun dekorativ - ingen kollisjon, samme prinsipp som bakketeksturen) ----------
   Ugjennomsiktig med vilje (IKKE transparent:true) - en halvgjennomsiktig overflate lot bakkens
   sjakkrutemønster (Sim.buildGroundTexture, normalt et subtilt gress-triks) skinne tydelig gjennom som
   synlige "rutenett-linjer" i elva, spesielt ved kontrasten mot blåfargen.
   Bygget som ETT sammenhengende bånd langs en Catmull-Rom-kurve gjennom RIVER_POINTS (i stedet for rette
   trapesformede segmenter mellom hvert punkt, med runde "lapper" limt på i svingene) - det gamle
   oppsettet var fortsatt synlig kantete siden SELVE BANEN aldri var buet, bare skjøtene mellom de rette
   bitene var avrundet. Én kurve gir naturlig buede svinger over hele lengden, og siden det er ett eneste
   mesh (ingen overlappende segmentkanter i det hele tatt) forsvinner også all skjøt-relatert flimring. */
// widths: én bredde per punkt i points (ikke én bredde totalt) - bredden interpoleres jevnt mellom
// naboverdiene langs kurven, se widthAtParam under.
const RIVER_Y = 0.12; // høyere enn det opprinnelige (0.03) - samme absolutte klaring blir en mye mindre
// ANDEL av dybdebufferets presisjon sett fra høyde/avstand, som ga synlig flimring (z-fighting) mot
// bakken derfra selv om det så helt stabilt ut på nært hold.
const RIVER_SAMPLES_PER_SEGMENT = 14;
function buildRiver(points, widths, pondRadius) {
    const group = new THREE.Group();
    const mat = new THREE.MeshStandardMaterial({ color: 0x3a72a8, roughness: 0.35, side: THREE.DoubleSide });

    const curvePoints = points.map(function (p) { return new THREE.Vector3(p.x, 0, p.z); });
    // "centripetal" (i stedet for standard uniform catmullrom) unngår løkker/overshoot ved skarpe
    // vinkler mellom punktene - viktig her siden RIVER_POINTS ikke er jevnt fordelt langs banen.
    const curve = new THREE.CatmullRomCurve3(curvePoints, false, "centripetal");
    const sampleCount = (points.length - 1) * RIVER_SAMPLES_PER_SEGMENT;
    const sampled = curve.getPoints(sampleCount);

    // Bredden ved kurveparameter t (0..1) - widths[i] er definert til å gjelde nøyaktig ved originalpunkt
    // i, altså t = i/(points.length-1) (slik CatmullRomCurve3 selv fordeler t jevnt over inputpunktene,
    // uavhengig av faktisk buelengde mellom dem) - resten lerpes rett fram mellom naboene.
    function widthAtParam(t) {
        const f = t * (widths.length - 1);
        const i0 = Math.min(Math.floor(f), widths.length - 2);
        return widths[i0] + (widths[i0 + 1] - widths[i0]) * (f - i0);
    }

    const positions = [];
    const indices = [];
    for (let i = 0; i <= sampleCount; i++) {
        const t = i / sampleCount;
        const p = sampled[i];
        const tangent = curve.getTangent(clamp(t, 0.0001, 0.9999));
        const normal = new THREE.Vector3(-tangent.z, 0, tangent.x).normalize(); // 90° om Y - peker tvers på elveløpet
        const halfW = widthAtParam(t) / 2;
        positions.push(p.x - normal.x * halfW, RIVER_Y, p.z - normal.z * halfW);
        positions.push(p.x + normal.x * halfW, RIVER_Y, p.z + normal.z * halfW);
        if (i > 0) {
            const a = (i - 1) * 2, b = a + 1, c = i * 2, d = c + 1;
            indices.push(a, c, b, b, c, d);
        }
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    geo.setIndex(indices);
    geo.computeVertexNormals();
    const mesh = new THREE.Mesh(geo, mat);
    mesh.receiveShadow = true; // elva skal ta imot skygge fra trær/gates/bygninger som krysser over den
    group.add(mesh);

    if (pondRadius) {
        // Liten dam/innsjø i enden av elva - egen (litt dypere) blåtone for å lese som stillestående
        // vann i motsetning til den rennende elva. Litt høyere enn selve elva (unngår samme
        // z-fighting-flimring mot elvebåndets ende som det opprinnelige oppsettet hadde).
        const pondMat = new THREE.MeshStandardMaterial({ color: 0x2f5f8f, roughness: 0.3 });
        const last = points[points.length - 1];
        const pond = new THREE.Mesh(new THREE.CircleGeometry(pondRadius, 24), pondMat);
        pond.receiveShadow = true;
        pond.rotation.x = -Math.PI / 2;
        pond.position.set(last.x, RIVER_Y + 0.02, last.z);
        group.add(pond);
    }
    return group;
}
// Bygger OG registrerer et racingbane-2-tre i ett kall - både det visuelle treet (med vind-svaiing, se
// treeSwayManager) og kolliderne (se treeToColliders, delt med DECORATIVE_TREES/FOREST_TREES). De
// statiske listene TREE_COLLIDERS bygges fra er ferdig regnet ut ved MODUL-lasting, lenge før course-2s
// trær i det hele tatt eksisterer (de bygges her, ved scene-oppbygging via buildGateCourse2) - uten
// dette hjelpefunksjon-kallet ville ALLE course-2-trærne vært usynlige for kollisjonssystemet, og
// droneen ville flydd rett gjennom dem alle sammen. Samme "push rett inn i den allerede-levende
// SOLID_COLLIDERS-arrayen"-mønster som rådhuset/husene bruker.
function addCourse2Tree(group, x, z, height) {
    const tree = Sim.buildRandomTree(height);
    tree.position.set(x, 0, z);
    group.add(treeSwayManager.addSwayingTree(tree));
    // treeToColliders returnerer to bokser (stamme + krone, se kommentaren der) - begge må registreres.
    Array.prototype.push.apply(SOLID_COLLIDERS, treeToColliders({ x: x, z: z, h: height }));
}
// Langs banen (se GATE_COURSE_2_CENTER) - rent landskapselement for variasjon, ligger klart utenfor
// selve portrekka. Starter som en liten bekk oppe ved foten av fjellet nord for banen (se
// MOUNTAIN_PEAKS index 0 - det tryggeste/minste fjellet, samme retning som selve banen er flyttet til),
// brer seg gradvis ut til en elv (se RIVER_WIDTHS), og ender i en liten dam/innsjø (se pondRadius).
// De tre siste punktene bøyer vestover (mot lavere X) sammenlignet med et rett løp ned fra punkt 4 -
// et rett løp ville tatt dammen/elvemunningen rett inn i FOREST_TREES (definert langt tidligere i
// filen, sentrert på x=140/z=90 - opprinnelig KUN en del av bane 1) - flere av de trærne lå bokstavelig
// innenfor dammens radius før denne justeringen.
const RIVER_POINTS = [
    { x: 64, z: 507 }, { x: 115, z: 415 }, { x: 125, z: 360 }, { x: 120, z: 290 },
    { x: 130, z: 230 }, { x: 90, z: 165 }, { x: 65, z: 120 }, { x: 45, z: 70 }
];
const RIVER_WIDTHS = [1.5, 3, 5, 7, 8, 9, 9.5, 10];
const RIVER_POND_RADIUS = 15;

/* ---------- Racingbane 2 - lengre og lenger unna enn den opprinnelige (GATE_COURSE_CENTER) ----------
   Samme håndplasserte-veipunkt-prinsipp som GATE_WAYPOINTS, men mer variert: en låve å fly gjennom
   (som før), pluss en egen racing-gate midt på taket av rådhuset (elementet "roofgate" - droneen må
   klatre opp, gjennom, og ned igjen), og en tydelig trang chikane. Element 0 er start/mål (svart/hvitt
   rutemønster + retningspil, se buildStartFinishGate) - løypa er en lukket løkke som alle de andre.
   Senteret er rett NORD for avgangsplassen (samme retning som den TRYGGESTE fjellsektoren, se
   mountainHeightAt-kommentaren - fjellet i denne retningen har desidert minst "rekkevidde" av alle
   åtte, se MOUNTAIN_DEFS index 0) i stedet for sørøst, der banens sørøstre hjørne (rådhus-/chikane-
   området) viste seg å ligge INNENFOR kollisjonsmarginen til det sørøstlige fjellet (index 3) - usynlig
   "krasj i løse lufta" et godt stykke fra selve fjellsiden, se JITTER_SAFETY_MARGIN.
*/
const GATE_COURSE_2_CENTER = new THREE.Vector3(0, 0, 320);
const TOWNHALL_OFFSET = { dx: 0, dz: -165 };
const GATE_WAYPOINTS_2 = [
    { type: "start", dx: 0, dz: 85, gap: 1.6, size: 3.4 },
    { type: "gate", dx: 40, dz: 100, gap: 2.2, size: 3.2 },
    { type: "gate", dx: 75, dz: 80, gap: 1.4, size: 2.8 },
    { type: "gate", dx: 95, dz: 40, gap: 2.4, size: 3.2 },
    { type: "barn", dx: 90, dz: -10 },
    { type: "gate", dx: 70, dz: -55, gap: 1.6, size: 3.0 },
    // Trang chikane - kicker andre veien enn resten av banen, samme idé som GATE_WAYPOINTS.
    { type: "gate", dx: 75, dz: -100, gap: 1.3, size: 2.6 },
    { type: "gate", dx: 35, dz: -90, gap: 1.2, size: 2.6 },
    // God avstand før/etter rådhustak-gaten (se roofgate under) - en lang, tydelig klatring opp og en
    // like lang, tydelig glidning ned igjen, i stedet for en brå (og forvirrende) rekkefølge tett inntil
    // selve bygget.
    { type: "gate", dx: 50, dz: -135, gap: 1.6, size: 2.8 },
    // Gate på rådhustaket - samme dx/dz som TOWNHALL_OFFSET, men elevation løfter HELE gaten opp på
    // taket i stedet for bakkeplanet (se GATE_PLACEMENTS_2/buildGateCourse2). gap er her kun høyden på
    // gatens egne (korte) bein OPPÅ taket, ikke avstanden fra bakken. Sentrert midt på det nå bredere
    // taket (se TOWNHALL_WIDTH/DEPTH) med god klaring til alle kanter.
    { type: "roofgate", dx: TOWNHALL_OFFSET.dx, dz: TOWNHALL_OFFSET.dz, gap: 0.4, size: 3.0, elevation: TOWNHALL_ROOF_Y },
    { type: "gate", dx: -50, dz: -135, gap: 1.6, size: 2.8 },
    { type: "gate", dx: -95, dz: -80, gap: 1.8, size: 3.0 },
    { type: "gate", dx: -100, dz: -30, gap: 1.8, size: 3.0 },
    { type: "gate", dx: -85, dz: 15, gap: 1.4, size: 2.6 },
    { type: "gate", dx: -55, dz: 55, gap: 2.0, size: 3.0 },
    { type: "gate", dx: -25, dz: 75, gap: 2.2, size: 3.2 }
];
const GATE_PLACEMENTS_2 = GATE_WAYPOINTS_2.map(function (wp, i) {
    const next = GATE_WAYPOINTS_2[(i + 1) % GATE_WAYPOINTS_2.length];
    return {
        wp: wp,
        x: GATE_COURSE_2_CENTER.x + wp.dx,
        z: GATE_COURSE_2_CENTER.z + wp.dz,
        y: wp.elevation || 0,
        yaw: Math.atan2(next.dx - wp.dx, next.dz - wp.dz)
    };
});
function buildGateCourse2() {
    const group = new THREE.Group();
    GATE_PLACEMENTS_2.forEach(function (placement) {
        const wp = placement.wp;
        const obstacle = (wp.type === "barn")
            ? buildBarn(BARN_DIMENSIONS.width, BARN_DIMENSIONS.height, BARN_DIMENSIONS.depth,
                BARN_DIMENSIONS.windowW, BARN_DIMENSIONS.windowH, BARN_DIMENSIONS.sillY)
            : (wp.type === "start")
                ? buildStartFinishGate(wp.size, wp.gap)
                : buildGate(wp.size, wp.gap);
        obstacle.position.set(placement.x, placement.y, placement.z);
        obstacle.rotation.y = placement.yaw;
        group.add(obstacle);
    });

    const townHall = buildTownHall(TOWNHALL_WIDTH, TOWNHALL_HEIGHT, TOWNHALL_DEPTH);
    const roofGatePlacement = GATE_PLACEMENTS_2.filter(function (p) { return p.wp.type === "roofgate"; })[0];
    const townHallX = GATE_COURSE_2_CENTER.x + TOWNHALL_OFFSET.dx;
    const townHallZ = GATE_COURSE_2_CENTER.z + TOWNHALL_OFFSET.dz;
    townHall.position.set(townHallX, 0, townHallZ);
    townHall.rotation.y = roofGatePlacement.yaw; // rådhuset vender samme vei som gaten på taket flys
    group.add(townHall);
    // Registreres i SOLID_COLLIDERS (definert lenger opp i filen, men fortsatt bare en vanlig mutert
    // array på dette tidspunktet - buildGateCourse2 kjøres først ved scene-oppbygging, godt etter at
    // hele filen er lest inn) slik at droneen faktisk kolliderer med veggene og kan lande på taket, i
    // stedet for å fly rett gjennom. ORIENTERT boks (cx/cz/halfW/halfD/yaw), ikke akse-rettet - rådhuset
    // er rotert (roofGatePlacement.yaw over), og en akse-rettet boks stor nok til å garantert dekke hele
    // det roterte footprinten (halve diagonalen i begge retninger) hadde trigget kollisjon lenge før
    // droneen faktisk nådde veggene. Se orientedBoxLocalXZ/pushOutOfSolidWalls for selve testen.
    SOLID_COLLIDERS.push({
        cx: townHallX, cz: townHallZ, halfW: TOWNHALL_WIDTH / 2, halfD: TOWNHALL_DEPTH / 2,
        yaw: roofGatePlacement.yaw, topY: TOWNHALL_ROOF_Y
    });
    // Klokketårnet (se tower.position.set/buildClockTower i buildTownHall) stikker godt over selve taket
    // - kollideren over dekker kun opp til takhøyden, så uten en EGEN, mindre kollider for tårnet ville
    // droneen fløyet rett gjennom hele spiret. Samme lokale offset/formel som buildTownHall bruker for
    // selve tårnet (width*0.3, -depth*0.32), transformert til verdensrom med samme rotasjon/senter som
    // rådhuset over - de to kolliderne overlapper med vilje der tårnet står (Math.max av topY-verdiene
    // i solidSurfaceHeightAt/resolveGroundContact velger automatisk den høyeste, altså tårnets tak, kun
    // innenfor tårnets eget, mindre footprint).
    const towerLocal = { x: TOWNHALL_WIDTH * 0.3, z: -TOWNHALL_DEPTH * 0.32 };
    const towerWorld = orientedBoxWorldFromLocal(towerLocal.x, towerLocal.z, {
        cx: townHallX, cz: townHallZ, yaw: roofGatePlacement.yaw
    });
    const towerBoxSize = TOWNHALL_WIDTH * 0.32; // se buildClockTower(width*0.32, ...) i buildTownHall
    const towerHeight = TOWNHALL_HEIGHT * 1.6;
    const towerCapHeight = towerHeight * 0.35;
    SOLID_COLLIDERS.push({
        cx: towerWorld.x, cz: towerWorld.z, halfW: towerBoxSize / 2, halfD: towerBoxSize / 2,
        yaw: roofGatePlacement.yaw, topY: TOWNHALL_ROOF_Y + towerHeight + towerCapHeight
    });

    group.add(buildRiver(RIVER_POINTS, RIVER_WIDTHS, RIVER_POND_RADIUS));

    // Liten bro å fly under, midt i den ellers nokså rette/kjedelige returleggen (mellom gate-elementene
    // like etter chikanen og rådhustaket) - ren landskapsvariasjon, samme prinsipp som fjellene i
    // bakgrunnen: dekorativ, ikke en obligatorisk del av løypa, og uten SOLID_COLLIDERS-registrering (den
    // eksisterende kollider-modellen antar en solid søyle fra bakken og opp til toppen - en bro med åpning
    // UNDER dekket hadde krevd en egen "hul boks"-variant resten av banen ikke bruker).
    const bridgeOffset = { x: -97, z: -55 };
    const bridge = buildBridge(16, 6, 3.5);
    bridge.position.set(GATE_COURSE_2_CENTER.x + bridgeOffset.x, 0, GATE_COURSE_2_CENTER.z + bridgeOffset.z);
    bridge.rotation.y = Math.atan2(-5, 50); // samme retning som banen flyr her (fra gate -95/-80 mot -100/-30)
    group.add(bridge);

    // "Ledelinjer" langs banen - referansepunkter for å bedømme fart/høyde/retning i høy fart, samme
    // begrunnelse som selve porthjørnene. Alle punktene holder minst ~20 m klaring til nærmeste port
    // (se GATE_WAYPOINTS_2) OG til elva (se RIVER_POINTS, dx ~95-130 langs denne siden av banen -
    // de to første punktene sto opprinnelig nesten oppå elva, flyttet vestover/innover her).
    // Tettere rundt chikanen og rådhustak-gaten (elementene 6-9, som svinger brått) - der trengs
    // ledelinjene mest.
    [
        { x: 80, z: 100 }, { x: 80, z: -25 }, { x: 100, z: -115 }, { x: -120, z: -55 },
        { x: -105, z: 45 }, { x: 35, z: 130 }, { x: 20, z: -60 }, { x: 10, z: -95 },
        { x: 60, z: -70 }, { x: 15, z: -130 }, { x: -25, z: -145 }
    ].forEach(function (t) {
        addCourse2Tree(group, GATE_COURSE_2_CENTER.x + t.x, GATE_COURSE_2_CENTER.z + t.z, 6 + Math.random() * 3);
    });
    // Liten skog midt i "infield"-området, godt unna alle portene i begge retninger (ut langs
    // østsiden, tilbake langs vestsiden) - gir banen et landskapsinnslag midt i, ikke bare i kantene.
    for (let r = 0; r < 3; r++) {
        for (let c = 0; c < 3; c++) {
            const idx = r * 3 + c;
            const jitterX = Math.sin(idx * 12.9) * 3;
            const jitterZ = Math.cos(idx * 7.3) * 3;
            addCourse2Tree(
                group,
                GATE_COURSE_2_CENTER.x + (c - 1) * 9 + jitterX,
                GATE_COURSE_2_CENTER.z - 40 + (r - 1) * 9 + jitterZ,
                7 + Math.abs(Math.sin(idx * 3.7)) * 4
            );
        }
    }
    [
        { x: 162, z: -20, w: 6, h: 5, d: 6, ry: 0.6, wall: 0xc9b896, roof: 0x6b4a3a }, // flyttet lenger unna elva (se RIVER_POINTS) - sto for tett på svingen der
        { x: -130, z: -100, w: 5.5, h: 4.5, d: 5.5, ry: 2.1, wall: 0xb8c9b0, roof: 0x4a3a2e }
    ].forEach(function (h) {
        const house = buildSimpleHouse2(h.w, h.h, h.d, h.wall, h.roof);
        const houseX = GATE_COURSE_2_CENTER.x + h.x, houseZ = GATE_COURSE_2_CENTER.z + h.z;
        house.position.set(houseX, 0, houseZ);
        house.rotation.y = h.ry;
        group.add(house);
        // Samme orienterte registrering som rådhuset over - ellers ville droneen flydd rett gjennom
        // husene også, og en akse-rettet tilnærming hadde igjen gitt en for løs kollisjonsboks siden
        // husene også er rotert (h.ry).
        SOLID_COLLIDERS.push({ cx: houseX, cz: houseZ, halfW: h.w / 2, halfD: h.d / 2, yaw: h.ry, topY: h.h });
    });

    // Tett skogstripe langs vestsiden av siste strekket (etter brua, fram til mål) - godt UTENFOR selve
    // porten-linjen (gate-elementene her ligger på x mellom -100 og -25, se GATE_WAYPOINTS_2), rent
    // bakgrunnslandskap for å gjøre den ellers ganske stille avslutningen mindre tom, ikke noe droneen
    // skal fly nær. Jitret rutenett, samme prinsipp som "infield"-skogen over.
    for (let r = 0; r < 7; r++) {
        for (let c = 0; c < 4; c++) {
            const idx = r * 4 + c;
            const jitterX = Math.sin(idx * 9.1) * 4;
            const jitterZ = Math.cos(idx * 5.7) * 4;
            addCourse2Tree(
                group,
                GATE_COURSE_2_CENTER.x - 125 - c * 14 + jitterX,
                GATE_COURSE_2_CENTER.z - 20 + r * 18 + jitterZ,
                6 + Math.abs(Math.sin(idx * 2.9)) * 5
            );
        }
    }

    // Tettere trær nær selve svingene på siste strekket (etter brua, fram til mål) - på BEGGE sider av
    // banen her (både innsiden/øst mot senter og utsiden/vest mot skogstripen over), i motsetning til
    // skogstripen over som bevisst holdt god avstand som ren bakgrunn. Disse skal oppleves som en del av
    // selve svingen, ikke bare landskap - fortsatt trygg klaring til gate-fangstradiusene (se
    // RACE_GATE_CENTERS_2, ~1-1.5 m), 10+ m er rikelig.
    // OBS: de fire siste (nær gatene (-55,55)/(-25,75), rett før mål) sto opprinnelig som
    // {-40,60}/{-40,68}/{-10,80} - regnet ut på nytt med faktisk vinkelrett avstand fra flylinja mellom
    // nabogatene, det viste seg å være så lite som 0.9-4 m til rett-linja mellom portene (praktisk talt
    // MIDT i banen) i stedet for den tiltenkte 10+ m klaringen. Erstattet med punkter 14 m vinkelrett ut
    // fra midtpunktet på hvert av de to siste segmentene (gate->gate->mål).
    [
        { x: -113, z: -35 }, { x: -88, z: -22 }, // rundt gate (-100,-30)
        { x: -100, z: 8 }, { x: -70, z: 20 },     // rundt gate (-85,15)
        { x: -70, z: 48 }, { x: -32, z: 53 },     // rundt gate (-55,55)
        { x: -48, z: 77 }, { x: -7, z: 67 }, { x: -18, z: 93 }, // rundt gate (-25,75) og siste strekk til mål
        { x: -108, z: -58 }, { x: -85, z: -52 }   // rundt brua
    ].forEach(function (t, i) {
        addCourse2Tree(group, GATE_COURSE_2_CENTER.x + t.x, GATE_COURSE_2_CENTER.z + t.z, 6 + Math.abs(Math.sin(i * 3.3)) * 4);
    });

    return group;
}

/* ---------- Propell-treffbokser for racingbane 2 - se PROP_HAZARDS-kommentaren for prinsippet.
   Identisk oppbygging, bare med et Y-elevasjonsledd (placement.y) lagt til hvor relevant, for
   rådhustak-gaten (den eneste som ikke står med bunnbein rett på bakkeplanet). */
const PROP_HAZARDS_2 = GATE_PLACEMENTS_2.map(function (placement) {
    const wp = placement.wp;
    const boxes = [];
    let boundR, maxY;
    if (wp.type === "barn") {
        const b = BARN_DIMENSIONS, t = 0.15;
        const hw = b.width / 2, hd = b.depth / 2;
        boxes.push({ minX: -hw - t, maxX: -hw + t, minY: 0, maxY: b.height, minZ: -hd, maxZ: hd });
        boxes.push({ minX: hw - t, maxX: hw + t, minY: 0, maxY: b.height, minZ: -hd, maxZ: hd });
        [-hd, hd].forEach(function (zPos) {
            const topY = b.sillY + b.windowH, panelW = (b.width - b.windowW) / 2;
            boxes.push({ minX: -hw, maxX: hw, minY: 0, maxY: b.sillY, minZ: zPos - t, maxZ: zPos + t });
            boxes.push({ minX: -hw, maxX: hw, minY: topY, maxY: b.height, minZ: zPos - t, maxZ: zPos + t });
            boxes.push({ minX: -hw, maxX: -hw + panelW, minY: b.sillY, maxY: topY, minZ: zPos - t, maxZ: zPos + t });
            boxes.push({ minX: hw - panelW, maxX: hw, minY: b.sillY, maxY: topY, minZ: zPos - t, maxZ: zPos + t });
        });
        boxes.push({ minX: -hw - 0.3, maxX: hw + 0.3, minY: b.height, maxY: b.height + 0.3, minZ: -hd - 0.3, maxZ: hd + 0.3 });
        boundR = Math.hypot(hw + 0.3, hd + 0.3);
        maxY = b.height + 0.3;
    } else {
        const s = wp.size, gap = wp.gap, t = 0.09;
        const hs = s / 2;
        boxes.push({ minX: -hs, maxX: hs, minY: gap - t, maxY: gap + t, minZ: -t, maxZ: t });
        boxes.push({ minX: -hs, maxX: hs, minY: gap + s - t, maxY: gap + s + t, minZ: -t, maxZ: t });
        boxes.push({ minX: -hs - t, maxX: -hs + t, minY: 0, maxY: gap + s, minZ: -t, maxZ: t });
        boxes.push({ minX: hs - t, maxX: hs + t, minY: 0, maxY: gap + s, minZ: -t, maxZ: t });
        boundR = hs + 0.2;
        maxY = gap + s + 0.2;
    }
    // Y-boksene over er alle relative til gatens/låvens EGEN bunn (0) - løft dem opp med
    // placement.y for elementer som ikke står rett på bakken (rådhustak-gaten).
    const yOffset = placement.y || 0;
    boxes.forEach(function (b) { b.minY += yOffset; b.maxY += yOffset; });
    return {
        x: placement.x, z: placement.z,
        cosYaw: Math.cos(placement.yaw), sinYaw: Math.sin(placement.yaw),
        boundRSq: (boundR + 1.5) * (boundR + 1.5),
        maxY: maxY + yOffset + 0.5,
        boxes: boxes
    };
});
const ALL_PROP_HAZARDS = PROP_HAZARDS.concat(PROP_HAZARDS_2);

function buildWorldObjects() {
    const group = new THREE.Group();

    const spawnPad = Sim.buildLandingPad(2.4);
    spawnPad.position.y = 0.02; // løftet litt over bakkeplanet for å unngå z-fighting/flimring
    spawnPad.receiveShadow = true;
    group.add(spawnPad); // ved avgangsplassen (0,0,0)

    const car = buildCar();
    // Flyttet lenger unna langs samme retning som folkemengden (CROWD_CENTER), i stedet for å stå
    // nærmere kameraet/avgangsplassen enn den - da havnet bilen omtrent i siktlinjen og skjulte
    // folkemengden bak seg. Nå står bilen bak mengden sett fra avgangsplassen: folkemengden er alltid
    // det nærmeste/synlige elementet, bilen bare et landemerke lenger bak.
    car.position.set(24, 0, 14);
    car.rotation.y = THREE.MathUtils.degToRad(20);
    group.add(car);

    const crowd = buildCrowd();
    crowd.position.copy(CROWD_CENTER);
    group.add(crowd);

    const building = buildBuilding();
    building.position.set(-35, 0, -35);
    group.add(building);

    DECORATIVE_TREES.forEach(function (t) {
        const tree = Sim.buildRandomTree(t.h);
        tree.position.set(t.x, 0, t.z);
        group.add(treeSwayManager.addSwayingTree(tree));
    });

    group.add(buildGateCourse());
    group.add(buildGateCourse2());

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
// Alle faktiske kallere i fysikk-/kollisjons-hot-pathen (getContactLocalPoints, updatePropStrikes)
// bruker alltid DRONE_ARM_LENGTH - cachet én gang her i stedet for å bygge 4 nye Vector3-er på nytt
// flere ganger per fysikk-tick (120 Hz). Kun lesing (map/x/z) noensinne av disse - trygt å dele.
const LEG_TOP_LOCAL = getLegTopLocalPositions(DRONE_ARM_LENGTH);
function getLegFootLocalPositions(legLength, armLength) {
    const tops = armLength === DRONE_ARM_LENGTH ? LEG_TOP_LOCAL : getLegTopLocalPositions(armLength);
    return tops.map(function (top) {
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
    // Hver arm-bom går fra et fremre til et bakre motorhjørne. Fremre halvdel farges rødlig
    // (samme som frontmotorene), bakre halvdel mørk grå, slik at nese-retningen er tydelig.
    function buildArmBoom(rotationY, frontLocalXSign) {
        const halfLen = armSpan / 2;
        const armGroup = new THREE.Group();
        const frontHalf = new THREE.Mesh(new THREE.BoxGeometry(halfLen, 0.02, 0.03), frontArmMat);
        frontHalf.position.x = frontLocalXSign * halfLen / 2;
        frontHalf.castShadow = true;
        const backHalf = new THREE.Mesh(new THREE.BoxGeometry(halfLen, 0.02, 0.03), armMat);
        backHalf.position.x = -frontLocalXSign * halfLen / 2;
        backHalf.castShadow = true;
        armGroup.add(frontHalf, backHalf);
        armGroup.rotation.y = rotationY;
        return armGroup;
    }
    const arm1 = buildArmBoom(Math.PI / 4, 1);
    const arm2 = buildArmBoom(-Math.PI / 4, -1);
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
    sunLight = new THREE.DirectionalLight(0xffffff, 0.9);
    sunLight.position.set(
        SHADOW_CENTER.x + SHADOW_LIGHT_OFFSET.x, SHADOW_LIGHT_OFFSET.y, SHADOW_CENTER.z + SHADOW_LIGHT_OFFSET.z
    );
    sunLight.castShadow = true;
    sunLight.shadow.mapSize.set(2048, 2048);
    sunLight.shadow.camera.left = -80;
    sunLight.shadow.camera.right = 80;
    sunLight.shadow.camera.top = 80;
    sunLight.shadow.camera.bottom = -80;
    sunLight.shadow.camera.near = 1;
    sunLight.shadow.camera.far = 250;
    sunLight.shadow.bias = -0.0015;
    sunLight.target.position.copy(SHADOW_CENTER);
    scene.add(sunLight.target);
    scene.add(sunLight);

    const aspect = window.innerWidth / Math.max(1, window.innerHeight - 70);
    chaseCamera = new THREE.PerspectiveCamera(60, aspect, 0.1, 2000);
    // Startverdiene matcher nøyaktig den gamle faste (0, 1.2, 3.5)-offseten, så standardutsikten er
    // uendret - orbit/zoom (høyreklikk+dra / scroll) er bare lagt oppå den, se createChaseCameraController.
    chaseCameraController = Sim.createChaseCameraController(chaseCamera, document.getElementById("simCanvas"), {
        defaultPitch: Math.atan2(1.2, 3.5),
        zoomMin: 1.5, zoomMax: 40,
        initialZoom: Math.hypot(1.2, 3.5),
        smoothingBase: 0.001,
        lookAtOffsetY: 0.3
    });
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

    // Helikopter/fly for killswitch-øvelsen (ex11) - bygget én gang og gjenbrukt, skjult (visible=false)
    // helt til en fare-sekvens faktisk starter (se spawnHelicopterFlight/spawnAirplaneFlight).
    heliHandle = buildHelicopter();
    scene.add(heliHandle.group);
    airplaneHandle = buildAirplane();
    scene.add(airplaneHandle);
    pedestrianHandle = buildPedestrianGroup();
    scene.add(pedestrianHandle);

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

// Chase-kamera med manuell orbit - delt logikk med fixed-wing-simulatoren, se
// Sim.createChaseCameraController i simulator-common.js. Instansieres i initScene() (må vente til
// chaseCamera faktisk finnes), oppdateres fra animate().
let chaseCameraController;

// Sollyset (skygge-castende DirectionalLight, se initScene) - modul-scope slik at skyggekamerarets
// senter kan følge droneen (se updateShadowCamera). Skyggekartet dekker kun en ±SHADOW_CAMERA_HALF_SIZE
// boks (fast oppløsning, se sun.shadow.mapSize) - med to racingbaner spredt over flere hundre meter ville
// én statisk boks stor nok til å dekke begge enten gitt store, grovpikslede skygger, eller (som det var
// før denne fiksen) rett og slett ingen skygger i det hele tatt utenfor boksen. Løsningen er å flytte
// boksen med droneen i stedet for å gjøre den større.
let sunLight;
const SHADOW_CENTER = new THREE.Vector3(0, 0, -40);
const SHADOW_LIGHT_OFFSET = { x: 60, y: 90, z: 80 }; // sun.position - SHADOW_CENTER ved init (se initScene)
const SHADOW_RECENTER_DIST = 40; // m - reposisjoner først når droneen har beveget seg dette langt fra
// forrige senter, ikke hvert bilde - hvert bilde ville fått skyggekantene til å "svømme" merkbart.

function updateVlosCamera() {
    // Fast plassert ved avgangsplassen - dreier kun for å følge droneen med blikket, som en pilot på bakken.
    vlosCamera.lookAt(droneState.position);
}

// Flytter skyggekamera-boksen til droneens nåværende område når den har beveget seg langt nok unna
// forrige senter (se SHADOW_RECENTER_DIST) - se kommentaren ved SHADOW_CENTER for hvorfor. Lysretningen
// (offset fra senter til sun.position) holdes konstant, kun SELVE boksen flyttes, så solvinkelen ser
// alltid lik ut uansett hvor på kartet droneen befinner seg.
function updateShadowCamera() {
    const dx = droneState.position.x - SHADOW_CENTER.x;
    const dz = droneState.position.z - SHADOW_CENTER.z;
    if (dx * dx + dz * dz < SHADOW_RECENTER_DIST * SHADOW_RECENTER_DIST) return;
    SHADOW_CENTER.set(droneState.position.x, 0, droneState.position.z);
    sunLight.position.set(
        SHADOW_CENTER.x + SHADOW_LIGHT_OFFSET.x, SHADOW_LIGHT_OFFSET.y, SHADOW_CENTER.z + SHADOW_LIGHT_OFFSET.z
    );
    sunLight.target.position.copy(SHADOW_CENTER);
    sunLight.target.updateMatrixWorld();
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
    kill: function () { toggleKill("gamepad"); },
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
    const packetDropped = dropChance > 0 && Math.random() < dropChance;

    if (packetDropped) {
        // Simulerer tapt kontrollpakke pga svak/tapt link - pinnene beholder forrige verdi.
    } else if (gp) {
        inputState.source = "gamepad";
        inputState.stick.roll = readStickAxis(gp, gamepadMap.roll);
        inputState.stick.pitch = readStickAxis(gp, gamepadMap.pitch);
        inputState.stick.yaw = readStickAxis(gp, gamepadMap.yaw);
        inputState.stick.throttle = readThrottleAxis(gp, gamepadMap.throttle);
    } else {
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
    }

    // gp er null/undefined i tastatur-grenen, akkurat som det eksplisitte null-kallet det erstatter -
    // updateGamepadAxesReadout faller selv tilbake på getActiveGamepad() da.
    updateGamepadAxesReadout(gp);
    // Kontrolltap (ex11, crowd/traffic): overstyrer stick-verdiene satt over med en falsk kommando mot
    // faresonen HVIS en slik rømning faktisk pågår akkurat nå - se applyKillswitchInputOverride. Må stå
    // sist i updateInput (etter ekte tastatur/gamepad-lesing over) for faktisk å nå frem til fysikken.
    applyKillswitchInputOverride();
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
    updateBystanderCollision();
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
    const armTops = LEG_TOP_LOCAL;
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
    // Ett eneste SOLID_COLLIDERS-gjennomløp per punkt (i stedet for to - dette gjennomløpet OG et til
    // inne i solidSurfaceHeightAt) - avgjør innfelt-i-vegg-status og samler samtidig opp den høyeste
    // gyldige topp-flaten (colliderTop), akkurat som solidSurfaceHeightAt() ellers ville gjort separat.
    const grounded = points.map(function (f) {
        let embeddedInWall = false;
        let colliderTop = 0;
        SOLID_COLLIDERS.forEach(function (c) {
            // Orienterte (roterte) kollidere - se orientedBoxLocalXZ-kommentaren lenger opp. MÅ ha egen
            // gren her: uten den ville c.minX/maxX/minZ/maxZ vært undefined for disse, og
            // "f.x < undefined"-sjekkene under ville alltid vært false - dvs. HELE kartet ville lest som
            // "innenfor" boksen (aldri hoppet ut via early return), med NaN-dytting som resultat overalt
            // under takhøyden. Det viste seg som usynlig "kollisjon i løse lufta" langt fra selve bygget.
            if (c.yaw !== undefined) {
                const loc = orientedBoxLocalXZ(f.x, f.z, c);
                if (Math.abs(loc.lx) > c.halfW || Math.abs(loc.lz) > c.halfD) return;
                if (f.y < (c.minY || 0)) return; // under boksens nedre grense - se minY-kommentaren under
                if (f.y >= c.topY - GROUND_CLEARANCE) {
                    colliderTop = Math.max(colliderTop, c.topY);
                    return;
                }
                embeddedInWall = true;
                const distMinX = loc.lx + c.halfW, distMaxX = c.halfW - loc.lx;
                const distMinZ = loc.lz + c.halfD, distMaxZ = c.halfD - loc.lz;
                const minDist = Math.min(distMinX, distMaxX, distMinZ, distMaxZ);
                if (!wallPush || minDist < wallPush.dist) {
                    let newLx = loc.lx, newLz = loc.lz;
                    if (minDist === distMinX) newLx = -c.halfW;
                    else if (minDist === distMaxX) newLx = c.halfW;
                    else if (minDist === distMinZ) newLz = -c.halfD;
                    else newLz = c.halfD;
                    const w = orientedBoxWorldFromLocal(newLx, newLz, c);
                    wallPush = { dist: minDist, deltaX: w.x - f.x, deltaZ: w.z - f.z };
                }
                return;
            }
            if (f.x < c.minX || f.x > c.maxX || f.z < c.minZ || f.z > c.maxZ) return;
            // minY (satt for trekollidere - se treeToColliders): en krone-boks som starter høyt over
            // bakken skal ikke gjelde i det hele tatt nede ved stamme-/bakkenivå - uten denne sjekken
            // ville den bredere krone-boksen likevel trigget helt ned til bakken, og gjeninnført akkurat
            // "for bred kollisjon ved stammen"-problemet den smalere stamme-boksen skulle løse.
            if (f.y < (c.minY || 0)) return;
            if (f.y >= c.topY - GROUND_CLEARANCE) {
                colliderTop = Math.max(colliderTop, c.topY);
                return;
            }
            embeddedInWall = true;
            const distMinX = f.x - c.minX, distMaxX = c.maxX - f.x, distMinZ = f.z - c.minZ, distMaxZ = c.maxZ - f.z;
            const minDist = Math.min(distMinX, distMaxX, distMinZ, distMaxZ);
            if (!wallPush || minDist < wallPush.dist) {
                if (minDist === distMinX) wallPush = { dist: minDist, deltaX: c.minX - f.x, deltaZ: 0 };
                else if (minDist === distMaxX) wallPush = { dist: minDist, deltaX: c.maxX - f.x, deltaZ: 0 };
                else if (minDist === distMinZ) wallPush = { dist: minDist, deltaX: 0, deltaZ: c.minZ - f.z };
                else wallPush = { dist: minDist, deltaX: 0, deltaZ: c.maxZ - f.z };
            }
        });
        if (embeddedInWall) return false;
        const groundY = Math.max(mountainHeightAt(f.x, f.z), colliderTop);
        const penetration = groundY - f.y;
        if (penetration > maxPenetration) maxPenetration = penetration;
        return penetration > -LEG_CONTACT_TOLERANCE;
    });

    if (wallPush) {
        // Generalisert (verdens-vektor, ikke "x eller z") for å dekke orienterte kollidere, som kan
        // trenge en dytt-retning som ikke er akse-rettet. For de gamle akse-rettede kolliderne er
        // deltaX/deltaZ alltid rene enkelt-akse-verdier, så dette reduserer til nøyaktig samme oppførsel
        // som før (kun nullstiller hastighetskomponenten som driver PUNKTET dypere inn i veggen).
        droneState.position.x += wallPush.deltaX;
        droneState.position.z += wallPush.deltaZ;
        const len = Math.hypot(wallPush.deltaX, wallPush.deltaZ);
        if (len > 1e-6) {
            const dirX = wallPush.deltaX / len, dirZ = wallPush.deltaZ / len;
            const vAlong = droneState.velocity.x * dirX + droneState.velocity.z * dirZ;
            if (vAlong < 0) {
                droneState.velocity.x -= vAlong * dirX;
                droneState.velocity.z -= vAlong * dirZ;
            }
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
        // Demping av vinkelhastigheten MÅ skje uansett støttemønster her - den gjaldt tidligere kun
        // wellSupported-grenen, så en drone som bare hviler delvis (f.eks. racing-klassen etter en krasj
        // med ødelagte propeller/ben, balansert på to motstående armunder-punkter) fikk ALDRI dempet
        // vinkelfarten. applyGravityPivotTorque under LEGGER TIL vinkelfart hver tick (tyngdekraft-momentet
        // om støttepunktet) men fjerner aldri noe - uten demping her hadde det ingen steder å ta veien, og
        // droneen vippet fram og tilbake for alltid i stedet for gradvis å miste energi og falle til ro.
        const dampFactor = wellSupported ? 0.5 : 0.9;
        droneState.angularVelocity.pitch *= dampFactor;
        droneState.angularVelocity.roll *= dampFactor;
        if (wellSupported) {
            // Godt støttet, sakte og innenfor gjenopprettbar helning - understellet "retter opp" droneen.
            const yawOnly = new THREE.Euler().setFromQuaternion(droneState.quaternion, "YXZ").y;
            const uprightQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, yawOnly, 0, "YXZ"));
            droneState.quaternion.slerp(uprightQuat, Math.min(1, LEG_CONTACT_RIGHTING_RATE * dt));
        }
        // Delvis støtte: tyngdekraften vipper den rundt støttepunktet (se applyGravityPivotTorque over) -
        // dempingen over sørger nå for at svingningen faktisk dør ut, i stedet for en aktiv oppretting
        // (som ville sett urealistisk ut for en drone som bare hviler på to hjørner, ikke fire ben).
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
    // Bakken/fjellene: alltid solid ved sin egen beregnede høyde, uavhengig av minY-konseptet under
    // (terreng har ingen "starter høyt oppe"-kollidere).
    if (p.y <= mountainHeightAt(p.x, p.z) + PROP_GROUND_STRIKE_EPS) return true;
    // Andre solide objekter (tak, vegger, bil, trestammer/-kroner): pointInsideAnySolidCollider
    // respekterer minY (se kommentaren der) - IKKE solidSurfaceHeightAt, som bevisst ignorerer minY (den
    // svarer "hva ville jeg landet på herfra", ikke "er jeg fysisk inni noe akkurat nå") og dermed ville
    // latt en trekrone telle som solid helt ned til bakken for en propell langt fra selve stammen.
    if (pointInsideAnySolidCollider(p.x, p.y, p.z)) return true;
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
    // Grovsil: kun baneelementer nær droneen testes per punkt - vanligvis 0-1 stykker. ALL_PROP_HAZARDS
    // dekker begge racingbanene (se PROP_HAZARDS/PROP_HAZARDS_2).
    const nearbyHazards = [];
    for (let i = 0; i < ALL_PROP_HAZARDS.length; i++) {
        const hz = ALL_PROP_HAZARDS[i];
        const dx = droneState.position.x - hz.x, dz = droneState.position.z - hz.z;
        if (dx * dx + dz * dz <= hz.boundRSq && droneState.position.y <= hz.maxY) nearbyHazards.push(hz);
    }
    // Propellskiven tilnærmes med senterpunktet + fire kantpunkter i skivens plan.
    const diskDirs = [
        new THREE.Vector3(1, 0, 0), new THREE.Vector3(-1, 0, 0),
        new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, -1)
    ].map(function (d) { return d.applyQuaternion(droneState.quaternion); });
    const armTops = LEG_TOP_LOCAL;
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
    droneState.injuredTarget = "pilot";
    droneState.armed = false;
}

// Samme personskade-mekanikk som updatePilotCollision, men for folkemengden ved bilen (alltid i verden,
// se buildWorldObjects) og fotgjengerne i "Uforutsette hendelser" (ex11, kun til stede mens de faktisk
// går - se pedestrianHandle). injuredTarget styrer kun BANNERTEKSTEN (se updateHud) - selve konsekvensen
// (disarm + varsel + R for restart) er identisk med å treffe VLOS-piloten.
function updateBystanderCollision() {
    if (droneState.injured) return;
    const spec = currentDroneSpec();
    const reach = (DRONE_ARM_LENGTH + bladeLengthForClass(droneState.droneClass) / 2) * spec.visualScale;
    if (droneState.position.y > PILOT_HEIGHT + reach) return;
    for (let i = 0; i < CROWD_MEMBER_OFFSETS.length; i++) {
        const off = CROWD_MEMBER_OFFSETS[i];
        const dx = droneState.position.x - (CROWD_CENTER.x + off.x);
        const dz = droneState.position.z - (CROWD_CENTER.z + off.z);
        if (Math.hypot(dx, dz) <= BYSTANDER_HIT_RADIUS + reach) {
            droneState.injured = true;
            droneState.injuredTarget = "bystander";
            droneState.armed = false;
            return;
        }
    }
    if (pedestrianHandle && pedestrianHandle.visible) {
        const dx = droneState.position.x - pedestrianHandle.position.x;
        const dz = droneState.position.z - pedestrianHandle.position.z;
        if (Math.hypot(dx, dz) <= BYSTANDER_HIT_RADIUS + reach) {
            droneState.injured = true;
            droneState.injuredTarget = "bystander";
            droneState.armed = false;
        }
    }
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
    droneState.injuredTarget = null;
    groundContactBlend = 0;
    repairAllProps(); // reset er også "propellbytte"
    // Sett direkte i ro på avgangsplassen (samme utregning som settleDroneOnGround) - tidligere ble
    // den plassert 1 m over bakken og falt ned dit den skulle stå, synlig som et lite "hopp" ved hver
    // (re)start/øvelsesbytte i stedet for å stå klar med en gang.
    droneState.grounded = true;
    settleDroneOnGround();
}

// source: "keyboard" | "button" (skjermknappen) | "gamepad" - brukt til å håndheve requiresGamepadKill
// (ex11): den øvelsen skal trene bruken av den FYSISKE bryteren på senderen, så tastatur/skjermknapp
// blokkeres bevisst mens den er aktiv (se BUTTON_ACTIONS.kill/keydown/armToggleBtn for kallerne).
function toggleKill(source) {
    if (droneState.crashed || droneState.injured) return; // må resettes (R) etter krasj/personskade
    if (exerciseState.awaitingNext) return; // fryst etter bestått øvelse - se completeExercise
    if (exerciseState.active && source !== "gamepad") {
        const exercise = EXERCISES[exerciseState.exerciseId];
        if (exercise && exercise.requiresGamepadKill) {
            exerciseState.warningMessage = "Bruk den fysiske kill-knappen på fjernkontrollen for denne øvelsen.";
            exerciseState.warningUntil = performance.now() + 2000;
            exerciseState.warningIsSuccess = false;
            return;
        }
    }
    droneState.armed = !droneState.armed;
}

function isGamepadKillBound() {
    return getActiveGamepad() != null && gamepadMap.buttons.kill !== null;
}

// Kort statustekst for gjeldende killswitch-fase - delt mellom øvelsesdetaljvisningen (showExerciseDetail)
// og HUD-linjen (updateExerciseHud).
function killswitchStatusText() {
    // Bevisst nøytral, uansett fase (se seksjonskommentaren over EXERCISES.ex11) - "danger" nevnes ikke
    // med ord her heller, det skal oppdages i selve 3D-scenen, ikke leses av HUD-en.
    if (exerciseState.ksPhase === "pending-respawn") return "Klargjør nytt forsøk...";
    if (exerciseState.ksPhase === "resolved") return "Følger konsekvensen...";
    return "Flyr...";
}

// Stegnavnet ("Rømning mot folkemengden" osv.) ville spoilet hvilket scenario som kommer/pågår - vises
// derfor kun som et nøytralt løpenummer i HUD/menyen (stage.label brukes fortsatt internt til logikk/
// feilmeldinger, bare ikke vist frem før/mens det skjer).
function killswitchDisplayLabel() {
    return "Scenario " + (exerciseState.stageIndex + 1);
}

function toggleCamera() {
    // De fleste øvelsene flys per definisjon fra VLOS - kamerabytte er låst mens de er aktive.
    // Racingbanen er unntaket (se EXERCISES.race1.freeCameraToggle): den krever ikke VLOS for at
    // øvelseslogikken skal gi mening, og piloten skal fritt kunne bytte mellom FPV/chase/VLOS i farta.
    const exercise = exerciseState.active ? EXERCISES[exerciseState.exerciseId] : null;
    if (exercise && !exercise.freeCameraToggle) {
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
const injuryBannerTitle = document.getElementById("injuryBannerTitle");
const INJURY_TITLES = {
    pilot: "AU AU! DU HAR SKADET DEG SELV!",
    bystander: "DU HAR SKADET EN PERSON I PUBLIKUM!"
};

function updateHud() {
    hudMode.textContent = MODE_LABELS[droneState.flightMode];
    hudArmed.textContent = droneState.injured ? "Skadet" : (droneState.crashed ? "Krasjet" : (droneState.armed ? "Armed" : "Killed"));
    hudArmed.className = "sim-status-value " + ((droneState.armed && !droneState.crashed && !droneState.injured) ? "sim-armed" : "sim-killed");
    // Personskade-varselet vinner over det vanlige krasj-varselet (droneen kan godt hard-lande ETTER
    // at den har truffet noen - da er det fortsatt personskaden som er poenget). Tittelen varierer med
    // HVEM som ble truffet (se updatePilotCollision/updateBystanderCollision) - å treffe en forbipasserende
    // er ikke "du har skadet deg selv".
    if (droneState.injured) injuryBannerTitle.textContent = INJURY_TITLES[droneState.injuredTarget] || INJURY_TITLES.pilot;
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

/* ---------- Videoforstyrrelse (FPV-signal i Realistisk modus) ---------- */
let noiseCtx = null;
let lastNoiseUpdate = 0;

const signalNoiseCanvasEl = document.getElementById("signalNoiseCanvas");
function initNoiseCanvas() {
    signalNoiseCanvasEl.width = 96;
    signalNoiseCanvasEl.height = 96;
    noiseCtx = signalNoiseCanvasEl.getContext("2d");
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
    if (!settings.realisticMode || activeCamera !== fpvCamera || linkQuality > 0.9) {
        signalNoiseCanvasEl.style.opacity = 0;
        signalNoiseCanvasEl.style.background = "";
        return;
    }
    const badness = 1 - linkQuality;
    if (now - lastNoiseUpdate > 80) {
        drawNoiseFrame();
        lastNoiseUpdate = now;
    }
    signalNoiseCanvasEl.style.opacity = Math.min(0.85, badness * 1.1);
    signalNoiseCanvasEl.style.background = (linkQuality < 0.08 && Math.random() < 0.6) ? "#000" : "";
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

// Samme aksekonvensjon-negasjon som resten av fysikken (se merknad i stepPhysics).
function drawFpvHorizon(ctx, w, h) {
    const euler = new THREE.Euler().setFromQuaternion(droneState.quaternion, "YXZ");
    const pitchDeg = -THREE.MathUtils.radToDeg(euler.x);
    const rollDeg = -THREE.MathUtils.radToDeg(euler.z);
    Sim.drawFpvHorizonFromAngles(ctx, w, h, pitchDeg, rollDeg);
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
    headingBadSinceMs: null, // tidspunktet nese-frem-avviket sist begynte å overstige toleransen (se HEADING_FORWARD_SLIP_GRACE_MS)
    // Løpende avvik i grader mellom nesa og den sjekkede retningen (mål-yaw i "nese ut"/hover, farts-
    // retning i "nese frem") - null når ikke relevant (for lav fart i nese-frem, landing osv.). Rent
    // informativt HUD-tall, uavhengig av grace/violation-logikken - se updateExercise/updateExerciseHud.
    headingErrorDeg: null,
    landingPhase: false, // alle deløvelser fullført - venter på landing på H-plassen (se completeExercise)
    landedSinceMs: null, // tidspunktet droneen sist begynte å stå trygt på landingsplassen (se LANDING_CONFIRM_SEC)
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
    returnFullCloudRep: 0, // "Returner hjem": hvilken runde (0-basert) som garantert får 100% skydekke

    // Killswitch-scenarioene (ex11) - se "Øvelser: killswitch-tilstandsmaskin" lenger ned.
    ksPhase: null, // null | "wait" | "danger" | "resolved" | "pending-respawn" - se killswitch-tilstandsmaskinen
    ksEngaged: false, // "wait": har nådd "liksom"-runden minst én gang - se updateKillswitchPatrol
    ksTriggerAt: 0, // "wait": 0 (ikke rullet) eller tidspunkt hendelsen inntreffer. "danger": tidspunkt den INNTRAFF (start for baneutregning)
    ksDeadlineAt: 0, // "danger" (crowd/traffic): absolutt siste tidspunkt motorene må kuttes innen
    ksSafeCutoffAt: 0, // "danger" (crowd/traffic): kutt ETTER dette teller ikke lenger som tidsnok - se KILLSWITCH_SAFE_CUTOFF_FRACTION
    ksRespawnAt: 0, // "pending-respawn"/"resolved": tidspunkt et nytt forsøk/neste steg klargjøres
    ksRunawayFrom: new THREE.Vector3(),
    ksRunawayTo: new THREE.Vector3(),
    ksHeliFrom: new THREE.Vector3(),
    ksHeliTo: new THREE.Vector3(),
    ksHeliStartTime: 0,
    ksAirplaneFrom: new THREE.Vector3(),
    ksAirplaneTo: new THREE.Vector3(),
    ksAirplaneStartTime: 0,
    ksPedestrianFrom: new THREE.Vector3(),
    ksPedestrianTo: new THREE.Vector3(),
    ksPedestrianStartTime: 0,
    ksPatrolIndex: 0, // "vente"-fasens "liksom"-runde (gjenbruker CIRCLE_WAYPOINTS) - rent kosmetisk
    ksSavedFlightMode: null, // flightMode midlertidig tvunget til Stabilized under crowd/traffic-rømning

    // Racingbanen (ex-race1) - se "Øvelser: racing-tilstandsmaskin". Gjenbruker wpIndex/engaged (samme
    // felt som løype-øvelsene) for hvilken port som er neste/om start/mål er krysset minst én gang.
    raceStartTime: 0, // tidspunkt inneværende runde startet (0 = klokken går ikke ennå)
    savedFlightMode: null, // flightMode slik den var før en øvelse tvang sin egen (racingbanen: Acro)
    savedFpvTiltDeg: null // FPV-kameravinkelen slik den var før en øvelse tvang sin egen (racingbanen: 30°)
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
// decimals: antall desimaler på sekund-delen (default 1 - tideler, som før, brukt av de fleste
// øvelsene). Racingbanens tider bruker 2 (hundredeler) i stedet - der er jevne tider (samme tidel)
// ganske sannsynlig med mange forsøk, og en ledertavle trenger en reell tiebreaker.
function formatExerciseTime(sec, decimals) {
    if (sec === null || sec === undefined) return "-";
    const d = decimals || 1;
    const mm = Math.floor(sec / 60);
    const ss = (sec % 60).toFixed(d);
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
// Flat pil på bakken som viser ønsket nese-retning for et hover-steg. Pila bygges liggende og pekende
// mot lokal -Z (samme "nese ut ved yaw 0"-konvensjon som droneen selv, se currentYaw/angularDiffDeg),
// og legges i en egen gruppe som roteres om verdens Y-akse med selve yaw-vinkelen - det unngår enhver
// tvetydighet rundt Euler-rekkefølge (se orientTowardTravel for samme "child peker -Z, parent roterer"-triks).
function buildGroundArrow(yaw, color) {
    const shaftW = 0.35, shaftLen = 1.0, headW = 0.9, headLen = 0.8;
    const shape = new THREE.Shape();
    shape.moveTo(-shaftW / 2, 0);
    shape.lineTo(-shaftW / 2, shaftLen);
    shape.lineTo(-headW / 2, shaftLen);
    shape.lineTo(0, shaftLen + headLen);
    shape.lineTo(headW / 2, shaftLen);
    shape.lineTo(shaftW / 2, shaftLen);
    shape.lineTo(shaftW / 2, 0);
    shape.closePath();
    const mesh = new THREE.Mesh(
        new THREE.ExtrudeGeometry(shape, { depth: 0.04, bevelEnabled: false }),
        new THREE.MeshStandardMaterial({ color: color, transparent: true, opacity: 0.9 })
    );
    mesh.rotation.x = -Math.PI / 2; // ligger flatt, pil-tuppen peker mot -Z (yaw 0) før parent-rotasjonen under
    const group = new THREE.Group();
    group.add(mesh);
    group.rotation.y = yaw;
    return group;
}

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
        const arrow = buildGroundArrow(stage.headingYaw, 0xffee55);
        arrow.position.set(HOVER_CENTER.x, 0.06, HOVER_CENTER.z);
        group.add(arrow);
    } else if (stage.type === "killswitch") {
        // "Vente"-fasens "liksom"-øvelse gjenbruker sirkel-runden (se updateKillswitchPatrol) - samme
        // veiledningsløkke som ex5/ex6, så det faktisk ser ut som en ordentlig øvelse å fly. Enkelte steg
        // (ks-heli) flyr denne mye høyere OG lenger unna enn standard - se patrolAltitude/patrolWaypoints
        // og HELI_ALTITUDE/HELI_PATROL_CENTER-kommentarene.
        const patrolAlt = stage.patrolAltitude || EXERCISE_ALTITUDE;
        const patrolWp = stage.patrolWaypoints || CIRCLE_WAYPOINTS;
        group.add(buildLoopStruts(patrolWp, patrolAlt, 0.08, guideMat));
        group.add(buildLoopStruts(stripWaypointY(patrolWp), 0.05, 0.06, groundMat));
    } else if (stage.type === "racing") {
        // Racingbanens porter er allerede tydelig markert i verden selv (sjakkrutete start/mål,
        // oransje/hvite portrammer) - ingen ekstra grønn løype-visualisering trengs her. Merk: denne
        // grenen MÅ finnes - uten den ville stage.waypoints under vært undefined for racing-steget og
        // kastet en feil midt i startExercise (som stopper den før menyen rekker å lukkes/ledertavlen
        // vises, se rebuildExerciseGuide-kallet i startExercise).
    } else if (stage.type !== "return") {
        // "Returner hjem" har ingen løype/veiledning - hele poenget er å finne hjem selv; markøren
        // (under) pulserer over H-plassen som mål.
        group.add(buildLoopStruts(stage.waypoints, EXERCISE_ALTITUDE, 0.08, guideMat));
        group.add(buildLoopStruts(stripWaypointY(stage.waypoints), 0.05, 0.06, groundMat));
    }

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
    if (stage.type === "killswitch") {
        // Markøren vises/animeres UAVHENGIG av ksPhase (aldri skjult/vist basert på fase) - å slå den av
        // akkurat idet noe inntreffer var selv et tydelig "nå skjer det"-signal (markøren forsvant et
        // øyeblikk før hendelsen ble synlig), stikk i strid med at brukeren skal oppdage alt selv. Den
        // fryser ganske enkelt på siste patruljepunkt når updateKillswitchPatrol slutter å oppdatere
        // ksPatrolIndex (utenfor "wait") - det gir ikke bort noe i seg selv.
        const wp = (stage.patrolWaypoints || CIRCLE_WAYPOINTS)[exerciseState.ksPatrolIndex];
        exerciseGuideHandle.nextWaypointMarker.position.set(wp.x, stage.patrolAltitude || EXERCISE_ALTITUDE, wp.z);
        exerciseGuideHandle.nextWaypointMarker.scale.setScalar(0.85 + Math.sin(now / 200) * 0.15);
        return;
    }
    if (stage.type === "hover") {
        exerciseGuideHandle.nextWaypointMarker.position.set(HOVER_CENTER.x, HOVER_ALTITUDE, HOVER_CENTER.z);
    } else if (stage.type === "return") {
        exerciseGuideHandle.nextWaypointMarker.position.set(0, 1.0, 0); // målet: H-plassen hjemme
    } else if (stage.type === "racing") {
        const wp = RACE_GATE_CENTERS_2[exerciseState.wpIndex];
        exerciseGuideHandle.nextWaypointMarker.position.set(wp.x, wp.y, wp.z);
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
    exerciseState.headingBadSinceMs = null;
    exerciseState.returnRepsCompleted = 0;
    exerciseState.headingErrorDeg = null;
    exerciseState.landedSinceMs = null;
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
    const exerciseId = exerciseState.exerciseId;
    const noTiming = !!EXERCISES[exerciseId].noTiming;
    const elapsedSec = (performance.now() - exerciseState.startTime) / 1000;
    const progress = exerciseProgress[exerciseId];
    const wasAllPassedBefore = allExercisesPassed();
    // "Uforutsette hendelser" (ex11) handler om riktig respons, ikke fart - ingen bestetid å slå der.
    const isNewBest = !noTiming && (progress.bestTimeSec === null || elapsedSec < progress.bestTimeSec);
    const isNearBest = !noTiming && !isNewBest && elapsedSec < progress.bestTimeSec * 1.2;
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

    const noTiming = !!EXERCISES[exerciseId].noTiming;
    title.textContent = justCompletedAll ? "Gratulerer - alle øvelser bestått!"
        : (isNewBest ? "Gratulerer - ny bestetid!" : (isNearBest ? "Bra jobba!" : "Øvelse fullført"));
    let line = noTiming
        ? EXERCISES[exerciseId].label + " bestått."
        : EXERCISES[exerciseId].label + " bestått på " + formatExerciseTime(elapsedSec) + ".";
    if (!noTiming && !isNewBest) line += " Beste tid: " + formatExerciseTime(exerciseProgress[exerciseId].bestTimeSec) + ".";
    if (justCompletedAll) line += " Du har nå fullført samtlige øvelser - bekreftelsen din venter!";
    text.textContent = line;

    if (justCompletedAll) {
        // Ferdig med alt - "Neste" gir ikke lenger mening som hovedvalg, lede rett til diplomet i stedet.
        nextBtn.style.display = "none";
        closeBtn.textContent = "Se bekreftelsen";
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
        // Killswitch-scenarioene (ex11) ender alle med motorene kuttet - ingen vits i å kreve at
        // brukeren flyr (den nå disarmede) droneen hjem til H-plassen, i motsetning til de andre
        // øvelsene. Fullfør rett ut i stedet for å gå via enterLandingPhase().
        if (exercise.skipLanding) { completeExercise(); return; }
        enterLandingPhase();
        return;
    }
    const nextStage = exercise.stages[exerciseState.stageIndex];
    // Popup i 5 sekunder: kvitter for fullført deløvelse og forteller hva som er neste oppgave. Bruker
    // et nøytralt løpenummer for killswitch-steg (stage.label ville spoilet neste scenario).
    exerciseState.warningMessage = "Fullført! Neste: " +
        (nextStage.type === "killswitch" ? killswitchDisplayLabel() : nextStage.label) +
        (nextStage.type === "killswitch" ? " - " + KILLSWITCH_PATROL_HINT : "");
    exerciseState.warningUntil = performance.now() + 5000;
    exerciseState.warningIsSuccess = true;
    rebuildExerciseGuide();
    if (nextStage.type === "killswitch") spawnKillswitchStage(nextStage);
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
            if (exerciseState.landedSinceMs === null) exerciseState.landedSinceMs = now;
            if (now - exerciseState.landedSinceMs >= LANDING_CONFIRM_SEC * 1000) {
                const exercise = EXERCISES[exerciseState.exerciseId];
                const isReturnExercise = exercise.stages[0].type === "return";
                if (isReturnExercise) exerciseState.returnRepsCompleted++;
                if (isReturnExercise && exerciseState.returnRepsCompleted < REQUIRED_RETURN_REPS) {
                    // Ikke siste gjennomføring ennå - respawn med ny tilfeldig posisjon/retning og fortsett.
                    // Klokka fortsetter å gå (samme startTime) - totaltiden for alle rundene lagres til slutt.
                    exerciseState.landingPhase = false;
                    exerciseState.landedSinceMs = null;
                    spawnForExercise(exercise);
                    exerciseState.warningMessage = "Runde " + exerciseState.returnRepsCompleted + "/" +
                        REQUIRED_RETURN_REPS + " fullført! Ny posisjon klargjort...";
                    exerciseState.warningUntil = now + 3000;
                    exerciseState.warningIsSuccess = true;
                } else {
                    completeExercise();
                }
            }
        } else {
            exerciseState.landedSinceMs = null;
        }
        return;
    }

    const stage = getExerciseStage();

    if (stage.type === "killswitch") {
        exerciseState.headingErrorDeg = null; // ingen nese-krav i killswitch-scenarioene
        updateKillswitchStage(stage, dt, now);
        return;
    }

    if (stage.type === "racing") {
        updateRacingStage(stage, dt, now);
        return;
    }

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
        const altOk = Math.abs(droneState.position.y - HOVER_ALTITUDE) <= HOVER_ALTITUDE_TOLERANCE;
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
            // -Z er forover (se stepPhysics/createChaseCameraController) - samme targetYaw-formel som
            // nesa "ut" ellers ville brukt, bare med fartsretningen i stedet for en fast retning.
            const targetYaw = Math.atan2(-droneState.velocity.x, -droneState.velocity.z);
            exerciseState.headingErrorDeg = Math.abs(angularDiffDeg(currentYaw, targetYaw));
            const overTolerance = exerciseState.headingErrorDeg > HEADING_TOLERANCE_DEG;
            if (overTolerance) {
                if (exerciseState.headingBadSinceMs === null) exerciseState.headingBadSinceMs = now;
                headingViolation = now >= exerciseState.headingGraceUntil &&
                    (now - exerciseState.headingBadSinceMs) >= HEADING_FORWARD_SLIP_GRACE_MS;
            } else {
                exerciseState.headingBadSinceMs = null;
            }
        } else {
            exerciseState.headingErrorDeg = null; // for lav fart til at fartsretningen er meningsfull
            exerciseState.headingBadSinceMs = null;
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

// Cachet én gang (samme mønster som hudMode/hudArmed osv. over updateHud) i stedet for et nytt
// document.getElementById-oppslag per felt hvert eneste bilde - denne kalles uvilkårlig fra animate().
const exerciseWarningBannerEl = document.getElementById("exerciseWarningBanner");
const exerciseWarningTextEl = document.getElementById("exerciseWarningText");
const exerciseHudBarEl = document.getElementById("exerciseHudBar");
const exerciseProgressBoxEl = document.getElementById("exerciseProgressBox");
const exerciseHudStageEl = document.getElementById("exerciseHudStage");
const exerciseHudLapsEl = document.getElementById("exerciseHudLaps");
const exerciseHudViolationsEl = document.getElementById("exerciseHudViolations");
const exerciseHudViolationsItemEl = document.getElementById("exerciseHudViolationsItem");
const exerciseHudHeadingErrorEl = document.getElementById("exerciseHudHeadingError");
const exerciseHudHeadingErrorItemEl = document.getElementById("exerciseHudHeadingErrorItem");
const exerciseHudTimerItemEl = document.getElementById("exerciseHudTimerItem");
const exerciseHudTimerEl = document.getElementById("exerciseHudTimer");

function updateExerciseHud() {
    const showBanner = performance.now() < exerciseState.warningUntil;
    exerciseWarningBannerEl.classList.toggle("show", showBanner);
    exerciseWarningBannerEl.classList.toggle("sim-banner-success", exerciseState.warningIsSuccess);
    if (showBanner) exerciseWarningTextEl.textContent = exerciseState.warningMessage;

    if (!exerciseState.active) {
        exerciseHudBarEl.style.display = "none";
        exerciseProgressBoxEl.style.display = "none";
        return;
    }
    exerciseHudBarEl.style.display = "";
    exerciseProgressBoxEl.style.display = "";
    // "Returner hjem" har ett fast steg (stageIndex alltid 0) - stegobjektet er gyldig gjennom hele
    // landingsfasen der også, i motsetning til vanlige flerstegs-øvelser (der landingPhase betyr
    // stageIndex har passert siste steg og getExerciseStage() ville returnert undefined). awaitingNext
    // (bestått og fryst, se completeExercise) betyr ALLTID at stegobjektet ikke lenger er gyldig å lese.
    const isReturnExercise = EXERCISES[exerciseState.exerciseId].stages[0].type === "return";
    const stage = (exerciseState.awaitingNext || (exerciseState.landingPhase && !isReturnExercise))
        ? null : getExerciseStage();
    // Killswitch-stegnavnet (stage.label) ville spoilet hvilket scenario som kommer/pågår - vises som
    // et nøytralt løpenummer i stedet, se killswitchDisplayLabel.
    exerciseHudStageEl.textContent = stage
        ? (stage.type === "killswitch" ? killswitchDisplayLabel() : stage.label)
        : (exerciseState.awaitingNext ? "Fullført!" : "Landing");
    // Merk runden som "teller ikke" så snart et avvik har skjedd i den - synlig konsekvens med en gang.
    const lapSuffix = (stage && exerciseState.lapHasViolation) ? " (runden teller ikke)" : "";
    const returnSuffix = exerciseState.landingPhase ? " - land på H" : "";
    exerciseHudLapsEl.textContent = !stage
        ? (exerciseState.awaitingNext ? "Se oppsummering" : "Land på H")
        : (stage.type === "hover"
            ? exerciseState.hoverHoldSec.toFixed(1) + "/" + stage.holdSec + " s"
            : (stage.type === "return"
                ? exerciseState.returnRepsCompleted + "/" + REQUIRED_RETURN_REPS + returnSuffix
                : stage.type === "killswitch"
                    ? killswitchStatusText()
                    : stage.type === "racing"
                        ? (exerciseState.engaged ? "Port " + exerciseState.wpIndex + "/" + RACE_GATE_CENTERS_2.length : "Kryss start/mål")
                        : exerciseState.lapsCleanCount + "/" + (stage.requiredCleanLaps || REQUIRED_CLEAN_LAPS) + lapSuffix));

    // Racing har verken avviks-telling eller nese-krav (fri stil) - begge feltene er bare støy der,
    // se stage.type === "racing" i updateRacingStage.
    const isRacing = stage && stage.type === "racing";
    exerciseHudViolationsItemEl.style.display = isRacing ? "none" : "";
    exerciseHudHeadingErrorItemEl.style.display = isRacing ? "none" : "";

    // Avviks-status: tydelig, vedvarende indikator på hvor nær steget er å bli nullstilt - banneret
    // alene forsvinner etter noen sekunder og etterlot ingen synlig "du har brukt opp advarselen".
    if (!stage || stage.type === "hover" || stage.type === "return" || stage.type === "killswitch") {
        exerciseHudViolationsEl.textContent = "-";
        exerciseHudViolationsEl.className = "sim-status-value";
    } else if (exerciseState.attemptViolationCount === 0) {
        exerciseHudViolationsEl.textContent = "Ingen";
        exerciseHudViolationsEl.className = "sim-status-value sim-armed";
    } else {
        exerciseHudViolationsEl.textContent = "1 - neste nullstiller!";
        exerciseHudViolationsEl.className = "sim-status-value sim-killed";
    }

    // Løpende nese-avvik i grader - se headingErrorDeg-kommentaren i updateExercise. Svarer direkte på
    // "hvilken retning sjekkes egentlig akkurat nå" i stedet for å bare oppdage det som et varsel etterpå.
    if (exerciseState.headingErrorDeg === null) {
        exerciseHudHeadingErrorEl.textContent = "-";
        exerciseHudHeadingErrorEl.className = "sim-status-value";
    } else {
        exerciseHudHeadingErrorEl.textContent = Math.round(exerciseState.headingErrorDeg) + "°";
        exerciseHudHeadingErrorEl.className = "sim-status-value " +
            (exerciseState.headingErrorDeg <= HEADING_TOLERANCE_DEG ? "sim-armed" : "sim-killed");
    }

    // "Uforutsette hendelser" (ex11) handler om riktig respons, ikke fart - se noTiming/completeExercise.
    if (EXERCISES[exerciseState.exerciseId].noTiming) {
        exerciseHudTimerItemEl.style.display = "none";
    } else if (stage && stage.type === "racing") {
        // Racingbanens klokke er en helt egen løpende rundetid (se updateRacingStage/raceStartTime),
        // IKKE den vanlige exerciseState.startTime (som bare markerer når selve øvelsen ble åpnet) -
        // står på "0:00" til klokken faktisk starter idet start/mål-porten krysses første gang.
        exerciseHudTimerItemEl.style.display = "";
        const elapsedSec = exerciseState.engaged ? (performance.now() - exerciseState.raceStartTime) / 1000 : 0;
        exerciseHudTimerEl.textContent = formatExerciseTime(elapsedSec, 2);
    } else {
        exerciseHudTimerItemEl.style.display = "";
        const elapsed = (performance.now() - exerciseState.startTime) / 1000;
        const mm = Math.floor(elapsed / 60);
        const ss = Math.floor(elapsed % 60);
        exerciseHudTimerEl.textContent = mm + ":" + (ss < 10 ? "0" : "") + ss;
    }
}

// Plasserer droneen for (om)start av en øvelse: vanlige øvelser starter på avgangsplassen (via
// resetDrone), "Returner hjem" spawner høyt og langt unna med tilfeldig yaw og starter nedtellingen.
function spawnForExercise(exercise) {
    resetDrone();
    exerciseState.returnPhase = null;
    // Killswitch-øvelsen (ex11): egen spawn/tilstand for steg 0 - se spawnKillswitchStage. Kalles kun
    // herfra ved førstegangsstart og fullt R-restart (begge nullstiller stageIndex til 0 først), så
    // exercise.stages[0] er alltid riktig steg å klargjøre.
    if (exercise.stages[0].type === "killswitch") {
        spawnKillswitchStage(exercise.stages[0]);
        exerciseState.warningMessage = KILLSWITCH_PATROL_HINT;
        exerciseState.warningUntil = performance.now() + 4000;
        exerciseState.warningIsSuccess = true;
    } else {
        // Racingbanen (ex-race1): egen spawn ved den nye banens start/mål-port i stedet for
        // avgangsplassen - se spawnRacingStage/RACE_SPAWN_POINT.
        if (exercise.stages[0].type === "racing") spawnRacingStage(exercise.stages[0]);
        if (exercise.startHint) {
            exerciseState.warningMessage = exercise.startHint;
            exerciseState.warningUntil = performance.now() + 4500;
            exerciseState.warningIsSuccess = true;
        }
    }
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
    const exercise = EXERCISES[id];
    if (!exercise) return;
    // Gaten håndheves også her (ikke bare i UI-en, se showExerciseDetail) - avviser før stopExercise()
    // slik at et eventuelt PÅGÅENDE forsøk på en annen øvelse ikke avbrytes for et forsøk som uansett blir avvist.
    if (exercise.requiresGamepadKill && !isGamepadKillBound()) return;
    stopExercise();
    exerciseState.savedDroneClass = droneState.droneClass;
    exerciseState.savedCameraModeIndex = cameraModeIndex;
    exerciseState.savedFlightMode = droneState.flightMode;
    exerciseState.savedFpvTiltDeg = settings.fpvTiltDeg;
    // De aller fleste øvelsene flys i Middels/VLOS - racingbanen (ex-race1) er unntaket: Racing-klasse,
    // Acro-modus, FPV-kamera med et lavere standard kameravinkel (se exercise.forceFpvTiltDeg) egnet for
    // racing. Alt lagres her og gjenopprettes i stopExercise, akkurat som vind/skydekke under.
    setDroneClassEphemeral(exercise.droneClass || "mid");
    const forcedCameraMode = exercise.forceCameraMode || "vlos";
    cameraModeIndex = CAMERA_MODES.indexOf(forcedCameraMode);
    activeCamera = (forcedCameraMode === "chase") ? chaseCamera : (forcedCameraMode === "fpv") ? fpvCamera : vlosCamera;
    if (exercise.forceFlightMode) droneState.flightMode = exercise.forceFlightMode;
    if (exercise.forceFpvTiltDeg !== undefined) {
        settings.fpvTiltDeg = exercise.forceFpvTiltDeg;
        fpvCamera.rotation.x = THREE.MathUtils.degToRad(settings.fpvTiltDeg);
    }

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
    exerciseState.warningUntil = 0; // rydd unna en ev. gjenværende banner fra forrige økt FØR spawnForExercise setter start-hintet
    spawnForExercise(exercise);

    exerciseState.active = true;
    exerciseState.exerciseId = id;
    exerciseState.startTime = performance.now();
    rebuildExerciseGuide();
    // Lukk menyen - piloten skal rett i gang, og panelet skygger for sikten mot treningsområdet.
    document.getElementById("exercisesPanel").style.display = "none";
    // Racingbanens ledertavle vises i stedet for menyen mens den flys (se stopExercise for skjuling) -
    // delvis gjennomsiktig, se CSS, så den ikke sperrer sikten helt slik selve menyen ville gjort.
    document.getElementById("racingLeaderboardOverlay").style.display = (id === "race1") ? "" : "none";
}

// Idempotent opprydning - kalles både fra "Avbryt" og fra starten av startExercise (dekker "bytt til
// en annen øvelse" og "start øvelsen på nytt" med én kodesti). Teleporterer IKKE droneen tilbake til
// plattformen - respekterer det brukeren holder på med akkurat da.
function stopExercise() {
    if (!exerciseState.active) return;
    document.getElementById("racingLeaderboardOverlay").style.display = "none";
    setDroneClassEphemeral(exerciseState.savedDroneClass);
    cameraModeIndex = exerciseState.savedCameraModeIndex;
    const mode = CAMERA_MODES[cameraModeIndex];
    activeCamera = (mode === "chase") ? chaseCamera : (mode === "fpv") ? fpvCamera : vlosCamera;
    if (exerciseState.savedFlightMode) {
        droneState.flightMode = exerciseState.savedFlightMode;
        exerciseState.savedFlightMode = null;
    }
    if (exerciseState.savedFpvTiltDeg !== undefined && exerciseState.savedFpvTiltDeg !== null) {
        settings.fpvTiltDeg = exerciseState.savedFpvTiltDeg;
        fpvCamera.rotation.x = THREE.MathUtils.degToRad(settings.fpvTiltDeg);
        exerciseState.savedFpvTiltDeg = null;
    }
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

/* ---------- Øvelser: killswitch-tilstandsmaskin (ex11) ----------
   Hvert steg går gjennom exerciseState.ksPhase: "wait" (tilfeldig ventetid mens patruljeruten flys,
   se updateKillswitchPatrol) -> "danger" (noe har inntruffet) -> enten "resolved" (riktig respons -
   kort pause så konsekvensen (fritt fall/landing) rekker å vises, se KILLSWITCH_SUCCESS_WATCH_SEC, før
   advanceExerciseStage) eller "pending-respawn" (feil respons/for sent - kort feilmelding, så et helt
   nytt forsøk på SAMME steg via spawnKillswitchStage).
   "crowd"/"traffic": ekte styringstap. Posisjonen er kinematisk scriptet (samme frys-teknikk som
   "Returner hjem"), MEN spillerens stick-input overstyres i tillegg med en falsk "spøkelses"-input inn i
   den ORDINÆRE fysikk-loopen (se applyKillswitchInputOverride, kalt fra updateInput) - dronen tipper/
   akselererer synlig som om den faktisk (feil-)styres, og spillerens egen pinne har null effekt. Kun
   kill (armed=false) stopper det.
   "heli"/"pedestrians": dronen er IKKE scriptet - brukeren beholder full kontroll og skal selv vike
   unna/lande. Kutt av motorene her er feil respons (se updateHeliDangerPhase/updatePedestrianDangerPhase).
*/
function spawnKillswitchStage(stage) {
    // Vanlig bakkespawn (som alle andre øvelser) - resetDrone() setter den armert og i ro på
    // avgangsplassen. Spilleren tar av selv, akkurat som ellers - ingen airborne-teleportering her.
    resetDrone();
    exerciseState.ksPhase = "wait";
    exerciseState.ksPatrolIndex = 0;
    exerciseState.ksSavedFlightMode = null;
    // Rulles IKKE inn her - ventetiden starter først når runden faktisk er nådd (ksEngaged, se
    // updateKillswitchPatrol/updateKillswitchStage). Uten dette kunne en hendelse utløses mens
    // spilleren fortsatt sto på bakken/nettopp hadde lettet, lenge før de var i nærheten av runden.
    exerciseState.ksEngaged = false;
    exerciseState.ksTriggerAt = 0;
    if (heliHandle) heliHandle.group.visible = false;
    if (airplaneHandle) airplaneHandle.visible = false;
    if (pedestrianHandle) pedestrianHandle.visible = false;
}

function respawnKillswitchAttempt(now, message) {
    exerciseState.ksPhase = "pending-respawn";
    exerciseState.warningMessage = message;
    exerciseState.warningUntil = now + KILLSWITCH_FAIL_PAUSE_MS;
    exerciseState.warningIsSuccess = false;
    exerciseState.ksRespawnAt = now + KILLSWITCH_FAIL_PAUSE_MS;
}

function restoreKillswitchFlightMode() {
    if (exerciseState.ksSavedFlightMode) {
        droneState.flightMode = exerciseState.ksSavedFlightMode;
        exerciseState.ksSavedFlightMode = null;
    }
}

// "Vente"-fasen: gjenbruker sirkel-runden (CIRCLE_WAYPOINTS, samme som ex5/ex6) som en "liksom"-øvelse
// å faktisk fly mens man venter - rent kosmetisk (ingen avvik/runder telles), looper i det uendelige.
// Første fangst setter ksEngaged (se updateKillswitchStage) - ventetiden til selve hendelsen starter
// ikke å telle før spilleren faktisk har nådd runden. Krever både at droneen er i lufta OG i riktig
// høyde (ikke bare horisontal avstand) - spawnpunktet (avgangsplassen, 0,0,0) ligger i seg selv bare
// ~1 m unna sirkelens nærmeste punkt horisontalt, så uten høydekravet kunne "nådd runden" trigges mens
// droneen fortsatt sto stille på bakken (eller rett etter lettoff, lenge før den nådde f.eks. den høye
// ks-heli-ringen på 15 m).
function updateKillswitchPatrol(stage) {
    if (droneState.grounded) return;
    const targetAlt = stage.patrolAltitude || EXERCISE_ALTITUDE;
    if (Math.abs(droneState.position.y - targetAlt) > 3) return;
    const patrolWp = stage.patrolWaypoints || CIRCLE_WAYPOINTS;
    const wp = patrolWp[exerciseState.ksPatrolIndex];
    const dx = droneState.position.x - wp.x, dz = droneState.position.z - wp.z;
    if (Math.hypot(dx, dz) < KILLSWITCH_PATROL_CAPTURE_RADIUS) {
        exerciseState.ksEngaged = true;
        exerciseState.ksPatrolIndex = (exerciseState.ksPatrolIndex + 1) % patrolWp.length;
    }
}

function spawnHelicopterFlight() {
    // Alltid FRA dypt i feltet (samme retning kameraet peker, se konstant-kommentaren) og INN mot
    // piloten - ikke tilfeldig retning som før (som halve gangene lot den dukke opp rett ved siden av
    // spilleren i stedet for synlig langt unna). Litt sidevariasjon (X) for at det ikke skal se
    // identisk ut hver gang.
    // Sentrert på HELI_PATROL_CENTER (samme senter som ringen spilleren faktisk flyr, se
    // HELI_PATROL_WAYPOINTS) - ikke EXERCISE_CENTER, ellers ville innflygingen passert et helt annet
    // sted enn der spilleren praktisk talt befinner seg.
    const x = HELI_PATROL_CENTER.x + (Math.random() * 2 - 1) * 8;
    exerciseState.ksHeliFrom.set(x, HELI_ALTITUDE, HELI_PATROL_CENTER.z - HELI_FLIGHT_HALF_LENGTH);
    exerciseState.ksHeliTo.set(x, HELI_ALTITUDE, HELI_PATROL_CENTER.z + 15);
    exerciseState.ksHeliStartTime = performance.now();
    heliHandle.group.position.copy(exerciseState.ksHeliFrom);
    orientTowardTravel(heliHandle.group, exerciseState.ksHeliFrom, exerciseState.ksHeliTo);
    heliHandle.group.visible = true;
}

function spawnAirplaneFlight() {
    const dir = Math.random() < 0.5 ? 1 : -1;
    // Litt forskjøvet i Z fra EXERCISE_CENTER (ikke rett over) - leser tydeligere som "krysser i
    // området" enn som rett over selve øvingsfeltet.
    const z = EXERCISE_CENTER.z - 20;
    exerciseState.ksAirplaneFrom.set(-dir * AIRPLANE_FLIGHT_HALF_LENGTH, AIRWAY_ALTITUDE, z);
    exerciseState.ksAirplaneTo.set(dir * AIRPLANE_FLIGHT_HALF_LENGTH, AIRWAY_ALTITUDE, z);
    exerciseState.ksAirplaneStartTime = performance.now();
    airplaneHandle.position.copy(exerciseState.ksAirplaneFrom);
    orientTowardTravel(airplaneHandle, exerciseState.ksAirplaneFrom, exerciseState.ksAirplaneTo);
    airplaneHandle.visible = true;
}

function spawnPedestrianWalk() {
    // Samme prinsipp som helikopteret (spawnHelicopterFlight) - inn langs Z, fra dypt i feltet (samme
    // retning kameraet peker) i stedet for sidelengs fra utsiden av bildet.
    const x = EXERCISE_CENTER.x + (Math.random() * 2 - 1) * 4;
    exerciseState.ksPedestrianFrom.set(x, 0, EXERCISE_CENTER.z - PEDESTRIAN_WALK_START_OFFSET);
    exerciseState.ksPedestrianTo.set(x, 0, EXERCISE_CENTER.z + PEDESTRIAN_WALK_END_OFFSET);
    exerciseState.ksPedestrianStartTime = performance.now();
    pedestrianHandle.position.copy(exerciseState.ksPedestrianFrom);
    orientPersonGroupTowardTravel(pedestrianHandle, exerciseState.ksPedestrianFrom, exerciseState.ksPedestrianTo);
    pedestrianHandle.visible = true;
}

// Vellykket respons på heli/pedestrians (vike unna/lande) - samme "vent litt før neste steg"-pause som
// crowd/traffic sin post-kill-visning (KILLSWITCH_SUCCESS_WATCH_SEC), slik at brukeren rekker å se
// utfallet (helikopteret som passerer trygt, fotgjengerne som går forbi) før neste hendelse starter.
function markKillswitchStageResolved(now) {
    exerciseState.ksPhase = "resolved";
    exerciseState.ksRespawnAt = now + KILLSWITCH_SUCCESS_WATCH_SEC * 1000;
}

// Utløser selve hendelsen - bevisst HELT STILLE (ingen banner/melding, se seksjonskommentaren over
// EXERCISES.ex11): brukeren skal oppdage og vurdere situasjonen selv, ikke bli varslet om den.
function startKillswitchDanger(stage, now) {
    exerciseState.ksPhase = "danger";
    exerciseState.ksTriggerAt = now;

    if (stage.variant === "crowd" || stage.variant === "traffic") {
        exerciseState.ksSavedFlightMode = droneState.flightMode;
        // Tvinger Stabilized under selve rømningen - se applyKillswitchInputOverride: en falsk stick-
        // input tolket som RATE (Acro) ville gitt en kontinuerlig rotasjon/tumling i stedet for en
        // troverdig, avgrenset "lener seg mot faren"-vinkel.
        droneState.flightMode = "stabilized";
        exerciseState.ksRunawayFrom.copy(droneState.position);
        if (stage.variant === "crowd") {
            exerciseState.ksRunawayTo.set(CROWD_CENTER.x, CROWD_TARGET_ALTITUDE, CROWD_CENTER.z);
        } else {
            exerciseState.ksRunawayTo.set(droneState.position.x, TRAFFIC_DANGER_ALTITUDE, droneState.position.z);
            spawnAirplaneFlight();
        }
        exerciseState.ksDeadlineAt = now + stage.runawaySec * 1000;
        // Kutt ETTER dette (men fortsatt før selve deadline) regnes IKKE lenger som en tidsnok respons -
        // uten margin var "kutt i aller siste liten, praktisk talt inni faresonen" fortsatt bestått, se
        // KILLSWITCH_SAFE_CUTOFF_FRACTION.
        exerciseState.ksSafeCutoffAt = now + stage.runawaySec * 1000 * KILLSWITCH_SAFE_CUTOFF_FRACTION;
    } else if (stage.variant === "heli") {
        spawnHelicopterFlight();
    } else {
        spawnPedestrianWalk();
    }
}

// Kontrolltap (crowd/traffic): spillerens EKTE pinne overstyres fullstendig med en falsk kommando mot
// faresonen, matet inn i den ORDINÆRE fysikken (stepPhysics leser inputState.stick) - dronen ser dermed
// ut til faktisk å motta (helt gale) pinneutslag i stedet for å bare gli/teleportere. MÅ kalles etter
// den ekte tastatur/gamepad-lesingen i updateInput (ellers blir denne overskrevet FØR fysikken bruker
// den) - selve posisjonen/timingen styres likevel deterministisk av updateKillswitchStage rett under,
// dette er kun det VISUELLE (tilt/bank) laget oppå.
function applyKillswitchInputOverride() {
    if (!exerciseState.active || exerciseState.ksPhase !== "danger") return;
    const stage = getExerciseStage();
    if (!stage || stage.type !== "killswitch" || (stage.variant !== "crowd" && stage.variant !== "traffic")) return;
    if (!droneState.armed) return; // nettopp kuttet - ikke fortsett å mate falsk input inn i et fritt fall
    const toTarget = new THREE.Vector3().subVectors(exerciseState.ksRunawayTo, droneState.position);
    const localTarget = toTarget.lengthSq() > 1e-6
        ? toTarget.normalize().applyQuaternion(droneState.quaternion.clone().invert())
        : new THREE.Vector3(0, 0, -1);
    inputState.stick.pitch = clamp(-localTarget.z * 3, -1, 1); // lokal -Z er forover (se stepPhysics)
    inputState.stick.roll = clamp(localTarget.x * 3, -1, 1);
    inputState.stick.yaw = 0;
    inputState.stick.throttle = 0.85;
}

// Riktig respons her er å vike unna/lande, IKKE kill - se seksjonskommentaren over EXERCISES.ex11.
// HELI_SAFE_HORIZ_DISTANCE er horisontal med vilje (se konstant-kommentaren) - kun å stå stille og
// stole på høydeforskjellen skal ikke telle som en unnamanøver.
function updateHeliDangerPhase(now) {
    if (!droneState.armed) {
        respawnKillswitchAttempt(now, KILLSWITCH_MESSAGES.heli.failKilled);
        return;
    }
    if (heliHandle.group.visible) {
        if (!droneState.grounded) {
            const horizDist = Math.hypot(
                droneState.position.x - heliHandle.group.position.x,
                droneState.position.z - heliHandle.group.position.z
            );
            if (horizDist < HELI_SAFE_HORIZ_DISTANCE) {
                respawnKillswitchAttempt(now, KILLSWITCH_MESSAGES.heli.failTooClose);
            }
        }
        return; // helikopteret flyr fortsatt - vent og se om avstanden holder seg trygg
    }
    markKillswitchStageResolved(now); // ferdig overflydd uten at avstanden noen gang ble kritisk - bestått
}

// Samme respons-logikk som helikopteret (vike unna/lande, ikke kill) OG samme "vent til det er ferdig"-
// prinsipp: fotgjengerne starter allerede utenfor PEDESTRIAN_SAFE_DISTANCE (se konstant-kommentaren), så
// avgjørelsen tas først når de faktisk er ferdige med å gå forbi (pedestrianHandle.visible blir false),
// basert på hvor droneen befant seg DA - ikke fortløpende fra første bilde.
function updatePedestrianDangerPhase(now) {
    if (!droneState.armed) {
        respawnKillswitchAttempt(now, KILLSWITCH_MESSAGES.pedestrians.failKilled);
        return;
    }
    if (pedestrianHandle.visible) return; // fortsatt på vei - ingen avgjørelse ennå
    const horizDist = Math.hypot(
        droneState.position.x - exerciseState.ksPedestrianTo.x,
        droneState.position.z - exerciseState.ksPedestrianTo.z
    );
    const controlled = droneState.grounded || droneState.velocity.length() < PEDESTRIAN_SAFE_MAX_SPEED;
    if (horizDist >= PEDESTRIAN_SAFE_DISTANCE && controlled) {
        markKillswitchStageResolved(now);
    } else {
        respawnKillswitchAttempt(now, KILLSWITCH_MESSAGES.pedestrians.failTooClose);
    }
}

function updateKillswitchStage(stage, dt, now) {
    if (exerciseState.ksPhase === "wait") {
        if (!droneState.armed) {
            // Motorene kuttet (eller krasjet) før noe faktisk inntraff - ikke en del av drillen ennå,
            // bare klargjør et nytt forsøk i stedet for å telle det som noe reelt.
            respawnKillswitchAttempt(now, droneState.crashed
                ? "Du krasjet før noe inntraff. Klargjør nytt forsøk..."
                : "Motorene ble kuttet uten grunn. Klargjør nytt forsøk...");
            return;
        }
        updateKillswitchPatrol(stage);
        // Ventetiden starter ikke å telle før runden faktisk er nådd - se ksEngaged-kommentaren i
        // spawnKillswitchStage/updateKillswitchPatrol.
        if (!exerciseState.ksEngaged) return;
        if (exerciseState.ksTriggerAt === 0) {
            exerciseState.ksTriggerAt = now +
                (KILLSWITCH_TRIGGER_MIN_SEC + Math.random() * (KILLSWITCH_TRIGGER_MAX_SEC - KILLSWITCH_TRIGGER_MIN_SEC)) * 1000;
            return;
        }
        if (now >= exerciseState.ksTriggerAt) startKillswitchDanger(stage, now);
        return;
    }
    if (exerciseState.ksPhase === "danger") {
        if (stage.variant === "heli") { updateHeliDangerPhase(now); return; }
        if (stage.variant === "pedestrians") { updatePedestrianDangerPhase(now); return; }
        // crowd/traffic: se applyKillswitchInputOverride (kalt fra updateInput) for det visuelle
        // "kontrolltapet" - posisjonen under er den deterministiske, alltid-reagerbare fasiten.
        if (!droneState.armed) {
            restoreKillswitchFlightMode();
            // Kuttet for sent (etter ksSafeCutoffAt, men fortsatt før selve deadline) er IKKE en
            // vellykket respons - se KILLSWITCH_SAFE_CUTOFF_FRACTION.
            if (now < exerciseState.ksSafeCutoffAt) {
                markKillswitchStageResolved(now);
            } else {
                // Kuttet, men for sent til å telle som trygt - IKKE samme melding som "aldri kuttet i
                // tide" (droneen har her ikke nødvendigvis faktisk nådd frem, bare reagert for seint).
                respawnKillswitchAttempt(now, KILLSWITCH_MESSAGES[stage.variant].failLate);
            }
            return;
        }
        const t = clamp((now - exerciseState.ksTriggerAt) / (stage.runawaySec * 1000), 0, 1);
        const oldPos = droneState.position.clone();
        droneState.position.lerpVectors(exerciseState.ksRunawayFrom, exerciseState.ksRunawayTo, t * t); // t² - akselererende, som et reelt kontrolltap
        // Fartsvektoren settes ut fra selve forflytningen (i stedet for å nullstilles) - gir realistisk
        // bevart moment inn i det frie fallet i det øyeblikket motorene kuttes, i stedet for at den
        // stopper brått og deretter faller fra stillstand.
        if (dt > 1e-4) droneState.velocity.copy(droneState.position).sub(oldPos).divideScalar(dt);
        if (now >= exerciseState.ksDeadlineAt) {
            restoreKillswitchFlightMode();
            droneState.armed = false;
            respawnKillswitchAttempt(now, KILLSWITCH_MESSAGES[stage.variant].fail);
        }
        return;
    }
    if (exerciseState.ksPhase === "resolved" && now >= exerciseState.ksRespawnAt) {
        advanceExerciseStage();
        return;
    }
    if (exerciseState.ksPhase === "pending-respawn" && now >= exerciseState.ksRespawnAt) {
        spawnKillswitchStage(stage);
        exerciseState.warningMessage = KILLSWITCH_PATROL_HINT;
        exerciseState.warningUntil = now + 4000;
        exerciseState.warningIsSuccess = true;
    }
}

// Helikopter/fly/fotgjenger-animasjon (posisjon + rotorspinn) - egen, alltid-kjørende oppdatering
// (uavhengig av exerciseState.active) slik at en påbegynt bevegelse fullføres visuelt selv om steget
// allerede er avgjort (bestått/feilet) eller øvelsen avbrytes midt i - se kallet i animate().
function updateKillswitchVisuals(now, dt) {
    if (heliHandle.group.visible) {
        const t = (now - exerciseState.ksHeliStartTime) / (HELI_FLIGHT_DURATION_SEC * 1000);
        if (t >= 1) {
            heliHandle.group.visible = false;
        } else {
            heliHandle.group.position.lerpVectors(exerciseState.ksHeliFrom, exerciseState.ksHeliTo, t);
        }
        heliHandle.rotor.rotation.y += dt * 45;
    }
    if (airplaneHandle.visible) {
        const t = (now - exerciseState.ksAirplaneStartTime) / (AIRPLANE_FLIGHT_DURATION_SEC * 1000);
        if (t >= 1) {
            airplaneHandle.visible = false;
        } else {
            airplaneHandle.position.lerpVectors(exerciseState.ksAirplaneFrom, exerciseState.ksAirplaneTo, t);
        }
    }
    if (pedestrianHandle.visible) {
        const t = (now - exerciseState.ksPedestrianStartTime) / (PEDESTRIAN_WALK_DURATION_SEC * 1000);
        if (t >= 1) {
            pedestrianHandle.visible = false;
        } else {
            pedestrianHandle.position.lerpVectors(exerciseState.ksPedestrianFrom, exerciseState.ksPedestrianTo, t);
        }
    }
}

/* ---------- Øvelser: racing-tilstandsmaskin (racingbane 2, se GATE_COURSE_2_CENTER) ----------
   Fritt løpende tidsforsøk, ikke en engangs-øvelse: klokken starter idet start/mål-porten (element 0 i
   GATE_WAYPOINTS_2) krysses FØRSTE gang, og hver fullførte runde (alle porter i rekkefølge, tilbake til
   samme port) logges i den lokale ledertavlen (se addRacingLapResult) - deretter starter neste runde
   umiddelbart uten å måtte krysse start på nytt. wpIndex/engaged gjenbrukes fra det generelle
   øvelsessystemet (samme felt som løype-øvelsene bruker); racing-spesifikt er kun raceStartTime.
*/
// captureRadius: romslig nok til å dekke det meste av selve åpningen (ikke bare et lite punkt midt i)
// - en drone som flyr gjennom nær kanten i høy fart skal fortsatt registreres, ikke bare et perfekt
// sentrert gjennomtreff. 0.55 * størrelsen dekker godt over halve åpningsbredden på hver kant.
const RACE_GATE_CENTERS_2 = GATE_PLACEMENTS_2.map(function (placement) {
    const wp = placement.wp;
    if (wp.type === "barn") {
        return {
            x: placement.x, y: BARN_DIMENSIONS.sillY + BARN_DIMENSIONS.windowH / 2, z: placement.z,
            captureRadius: Math.min(BARN_DIMENSIONS.windowW, BARN_DIMENSIONS.windowH) * 0.55
        };
    }
    return {
        x: placement.x, y: placement.y + wp.gap + wp.size / 2, z: placement.z,
        captureRadius: wp.size * 0.55
    };
});
const RACE_START_PLACEMENT = GATE_PLACEMENTS_2[0]; // start/mål-porten, se GATE_WAYPOINTS_2
const RACE_SPAWN_BACK_DIST = 10; // meter "bak" start/mål-porten (mot ankomstretningen) droneen spawner
const RACE_SPAWN_POINT = new THREE.Vector3(
    RACE_START_PLACEMENT.x - Math.sin(RACE_START_PLACEMENT.yaw) * RACE_SPAWN_BACK_DIST,
    0,
    RACE_START_PLACEMENT.z - Math.cos(RACE_START_PLACEMENT.yaw) * RACE_SPAWN_BACK_DIST
);
// +PI: droneens EGEN "forover er lokal -Z"-konvensjon er motsatt av portenes (lokal +Z er flyretningen
// gjennom en port, se GATE_PLACEMENTS-kommentaren) - nesa skal peke MOT porten, altså langs portens +Z.
const RACE_SPAWN_YAW = RACE_START_PLACEMENT.yaw + Math.PI;

function spawnRacingStage(stage) {
    resetDrone();
    droneState.position.x = RACE_SPAWN_POINT.x;
    droneState.position.z = RACE_SPAWN_POINT.z;
    droneState.quaternion.setFromEuler(new THREE.Euler(0, RACE_SPAWN_YAW, 0, "YXZ"));
    settleDroneOnGround();
    exerciseState.wpIndex = 0;
    exerciseState.engaged = false;
    exerciseState.raceStartTime = 0;
}

function updateRacingStage(stage, dt, now) {
    exerciseState.headingErrorDeg = null; // ingen nese-krav i racing - fri stil, bare gjennom portene
    const gates = RACE_GATE_CENTERS_2;
    const wp = gates[exerciseState.wpIndex];
    const dist = Math.hypot(droneState.position.x - wp.x, droneState.position.y - wp.y, droneState.position.z - wp.z);
    if (dist >= wp.captureRadius) return;

    if (exerciseState.wpIndex === 0) {
        if (!exerciseState.engaged) {
            // Første kryssing av start/mål - klokken starter. Bevisst ingen forhåndsvarsel om NÅR dette
            // skjer (kun det generelle exercise.startHint ved spawn) - selve klokkestarten skal
            // oppleves akkurat idet streken krysses, som i ekte racing.
            exerciseState.engaged = true;
            exerciseState.raceStartTime = now;
            exerciseState.warningUntil = 0; // fjern start-hintet med det samme - ikke la det henge igjen
        } else {
            // Runde fullført - alle porter truffet i rekkefølge for å komme hit igjen.
            const elapsedSec = (now - exerciseState.raceStartTime) / 1000;
            addRacingLapResult(elapsedSec);
            exerciseState.warningMessage = "Runde fullført: " + formatExerciseTime(elapsedSec, 2) + "!";
            exerciseState.warningUntil = now + 3500;
            exerciseState.warningIsSuccess = true;
            exerciseState.raceStartTime = now; // ny runde starter umiddelbart - løpende tidsforsøk
        }
    }
    exerciseState.wpIndex = (exerciseState.wpIndex + 1) % gates.length;
}

/* ---------- Racingbanens ledertavle - lagres lokalt i nettleseren (samme Sim.loadJSON/saveJSON-mønster
   som exerciseProgress/settings), helt separat fra det vanlige øvelses-fremgangssystemet siden dette
   er et åpent tidsforsøk (så mange runder man vil), ikke en engangs bestått/ikke-bestått-sjekk. ---------- */
// Bumpet til v2 (fra v1) idet racingbanen ble bygget om - de gamle v1-tidene ble målt på en helt annen
// bane (kortere, andre porter) og er ikke lenger sammenlignbare. En ny nøkkel gir automatisk en tom
// ledertavle ved denne oppdateringen (Sim.loadJSON returnerer DEFAULT_RACING_LEADERBOARD for en nøkkel
// som aldri er lagret før), uten noen egen migrerings-/versjonssjekk-logikk - de gamle v1-tidene blir
// liggende urørt (og ubrukt) i nettleserens localStorage, rett og slett aldri lest igjen. Fremtidige
// baneendringer som gjør gamle tider usammenlignbare kan bumpe denne på nytt samme måte.
const RACING_LEADERBOARD_KEY = "ffi-uas:racing-leaderboard-v2";
const RACING_LEADERBOARD_MAX_ENTRIES = 20;
const DEFAULT_RACING_LEADERBOARD = { playerName: "Pilot", entries: [] };
function loadRacingLeaderboard() {
    return Sim.loadJSON(RACING_LEADERBOARD_KEY, DEFAULT_RACING_LEADERBOARD);
}
function saveRacingLeaderboard() {
    Sim.saveJSON(RACING_LEADERBOARD_KEY, racingLeaderboard);
}
const racingLeaderboard = loadRacingLeaderboard();

// Manuell nullstilling av HELE ledertavlen (navnet i feltet øverst beholdes - kun tidene fjernes) - egen
// "er du sikker"-bekreftelse siden dette er permanent og rammer ALLE tider, ikke bare én. Samme mønster
// som renameRacingLeaderboardEntry.
function resetRacingLeaderboard() {
    if (racingLeaderboard.entries.length === 0) return;
    const ok = window.confirm(
        "Nullstille HELE ledertavlen? Dette sletter alle " + racingLeaderboard.entries.length +
        " lagrede tid(er) permanent - dette kan ikke angres."
    );
    if (!ok) return;
    racingLeaderboard.entries = [];
    saveRacingLeaderboard();
    renderRacingLeaderboard();
}

function addRacingLapResult(timeSec) {
    racingLeaderboard.entries.push({
        name: racingLeaderboard.playerName || "Pilot", timeSec: timeSec, dateISO: new Date().toISOString()
    });
    // Beste tid øverst - kappes til RACING_LEADERBOARD_MAX_ENTRIES for at listen ikke skal vokse i det
    // uendelige (bare de beste rundene er interessante uansett).
    racingLeaderboard.entries.sort(function (a, b) { return a.timeSec - b.timeSec; });
    if (racingLeaderboard.entries.length > RACING_LEADERBOARD_MAX_ENTRIES) {
        racingLeaderboard.entries.length = RACING_LEADERBOARD_MAX_ENTRIES;
    }
    saveRacingLeaderboard();
    renderRacingLeaderboard();
}
function renderRacingLeaderboard() {
    const listEl = document.getElementById("racingLeaderboardList");
    if (!listEl) return;
    listEl.innerHTML = "";
    if (racingLeaderboard.entries.length === 0) {
        const empty = document.createElement("div");
        empty.className = "sim-racing-lb-empty";
        empty.textContent = "Ingen tider ennå - fullfør en runde!";
        listEl.appendChild(empty);
        return;
    }
    racingLeaderboard.entries.forEach(function (entry, i) {
        const row = document.createElement("div");
        row.className = "sim-racing-lb-row" + (i === 0 ? " sim-racing-lb-best" : "");
        const rank = document.createElement("span");
        rank.className = "sim-racing-lb-rank";
        rank.textContent = (i + 1) + ".";
        const name = document.createElement("span");
        name.className = "sim-racing-lb-name";
        name.textContent = entry.name;
        const time = document.createElement("span");
        time.className = "sim-racing-lb-time";
        time.textContent = formatExerciseTime(entry.timeSec, 2);
        // Endre-knapp for en allerede lagret tid (ikke feltet øverst - det setter kun navnet på
        // FREMTIDIGE runder) - for å rette et feilskrevet navn i etterkant, se renameRacingLeaderboardEntry.
        const editBtn = document.createElement("button");
        editBtn.type = "button";
        editBtn.className = "sim-racing-lb-edit";
        editBtn.title = "Endre navn på denne tiden";
        editBtn.innerHTML = '<i class="fa-solid fa-pen"></i>';
        editBtn.addEventListener("click", function () { renameRacingLeaderboardEntry(i); });
        row.appendChild(rank);
        row.appendChild(name);
        row.appendChild(time);
        row.appendChild(editBtn);
        listEl.appendChild(row);
    });
}
// Endrer navnet på en ALLEREDE lagret tid (i motsetning til navnefeltet øverst, som kun gjelder
// runder man fullfører etter at man har endret det) - typisk for å rette et feilskrevet navn.
// Bevisst en "er du sikker"-bekreftelse først (ikke bare et rett-frem endre-felt): listen er felles for
// alle som bruker denne nettleseren/maskinen, og uten friksjon her hadde det vært for lett å stille
// endre navnet på ANDRES tid og "jukse" til seg æren for en runde man ikke selv fløy.
function renameRacingLeaderboardEntry(i) {
    const entry = racingLeaderboard.entries[i];
    if (!entry) return;
    const ok = window.confirm(
        "Endre navnet på denne tiden (" + formatExerciseTime(entry.timeSec, 2) + ", satt av \"" + entry.name +
        "\")?\n\nBruk dette kun til å rette DITT EGET feilskrevne navn - ikke for å jukse til deg æren for andres runder!"
    );
    if (!ok) return;
    const newName = window.prompt("Nytt navn:", entry.name);
    if (newName === null) return; // avbrutt
    const trimmed = newName.trim().slice(0, 16);
    if (!trimmed) return;
    entry.name = trimmed;
    saveRacingLeaderboard();
    renderRacingLeaderboard();
}
function racingBestTimeSec() {
    return racingLeaderboard.entries.length > 0 ? racingLeaderboard.entries[0].timeSec : null;
}

/* ---------- Øvelser: panel-UI (liste + detaljvisning) ---------- */
function renderExerciseList(category) {
    const container = document.getElementById("exerciseListItems");
    container.innerHTML = "";

    if (category === "acro") {
        ACRO_EXERCISE_ORDER.forEach(function (id) {
            const exercise = EXERCISES[id];
            const bestSec = racingBestTimeSec();
            const row = document.createElement("button");
            row.type = "button";
            row.className = "sim-exercise-row";
            row.innerHTML =
                '<span class="sim-exercise-row-icon"><i class="fa-solid ' + exercise.icon + '"></i></span>' +
                '<span class="sim-exercise-row-main">' +
                '<span class="sim-exercise-row-title">' + exercise.label + "</span>" +
                '<span class="sim-exercise-row-desc">' + exercise.shortDescription + "</span>" +
                "</span>" +
                (bestSec !== null
                    ? '<span class="sim-exercise-check"><i class="fa-solid fa-trophy"></i> ' + formatExerciseTime(bestSec, 2) + "</span>"
                    : "");
            row.addEventListener("click", function () { showExerciseDetail(id); });
            container.appendChild(row);
        });
        return;
    }

    // Alle øvelser bestått: diplom-rad ØVERST (mest fremtredende plassen - se etterspurt om å flytte
    // den opp fra bunnen) - klikk for å fylle ut navn og skrive ut/lagre som PDF. (Diplomet dukker i
    // tillegg automatisk opp første gang dette blir sant, se completeExercise.)
    if (allExercisesPassed()) {
        const diplomaRow = document.createElement("button");
        diplomaRow.type = "button";
        diplomaRow.className = "sim-exercise-row sim-exercise-row-diploma";
        diplomaRow.innerHTML =
            '<span class="sim-exercise-row-icon"><i class="fa-solid fa-award"></i></span>' +
            '<span class="sim-exercise-row-main">' +
            '<span class="sim-exercise-row-title">Bekreftelse</span>' +
            '<span class="sim-exercise-row-desc">Alle øvelser bestått! Fyll ut navnet ditt og skriv ut bekreftelsen.</span>' +
            "</span>";
        diplomaRow.addEventListener("click", openDiploma);
        container.appendChild(diplomaRow);
    }

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
                ? '<span class="sim-exercise-check"><i class="fa-solid fa-circle-check"></i>' +
                    (exercise.noTiming ? "" : " " + formatExerciseTime(progress.bestTimeSec)) + "</span>"
                : "");
        row.addEventListener("click", function () { showExerciseDetail(id); });
        container.appendChild(row);
    });
}

function openDiploma() {
    const overlay = document.getElementById("diplomaOverlay");
    // Bestetidene (fra localStorage) printes på diplomet - konkurranse-elementet: fly øvelsene så
    // mange ganger man vil for å forbedre tidene, diplomet viser alltid de gjeldende bestetidene.
    const timesEl = document.getElementById("diplomaTimes");
    timesEl.innerHTML = "";
    let totalTimedSec = 0;
    EXERCISE_ORDER.forEach(function (id) {
        const rowEl = document.createElement("div");
        rowEl.className = "sim-diploma-time-row";
        const nameEl = document.createElement("span");
        nameEl.textContent = EXERCISES[id].label;
        const timeEl = document.createElement("span");
        // "Uforutsette hendelser" (ex11) har ingen stoppeklokke (se noTiming) - bare bestått/ikke,
        // og telles heller ikke med i totaltiden nederst.
        if (EXERCISES[id].noTiming) {
            timeEl.textContent = "Bestått";
        } else {
            timeEl.textContent = formatExerciseTime(exerciseProgress[id].bestTimeSec);
            totalTimedSec += exerciseProgress[id].bestTimeSec || 0;
        }
        rowEl.appendChild(nameEl);
        rowEl.appendChild(timeEl);
        timesEl.appendChild(rowEl);
    });
    const totalRowEl = document.createElement("div");
    totalRowEl.className = "sim-diploma-time-row sim-diploma-time-total";
    const totalNameEl = document.createElement("span");
    totalNameEl.textContent = "Total tid";
    const totalTimeEl = document.createElement("span");
    totalTimeEl.textContent = formatExerciseTime(totalTimedSec);
    totalRowEl.appendChild(totalNameEl);
    totalRowEl.appendChild(totalTimeEl);
    timesEl.appendChild(totalRowEl);
    document.getElementById("diplomaDate").textContent =
        "Dato: " + new Date().toLocaleDateString("nb-NO", { year: "numeric", month: "2-digit", day: "2-digit" });
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
// Hvilken undermeny (Stabilized/Acro) som sist var åpen - husker valget slik at "Tilbake" fra
// detaljvisningen og gjenåpning av panelet (M/knapp) havner samme sted, se showExerciseDetail/
// toggleExercisesBtn-lytteren.
let currentExerciseCategory = "stabilized";
function showExerciseCategoryView() {
    document.getElementById("exerciseCategoryView").style.display = "";
    document.getElementById("exerciseListView").style.display = "none";
    document.getElementById("exerciseDetailView").style.display = "none";
}
function showExerciseListView(category) {
    if (category) currentExerciseCategory = category;
    renderExerciseList(currentExerciseCategory);
    document.getElementById("exerciseCategoryView").style.display = "none";
    document.getElementById("exerciseListView").style.display = "";
    document.getElementById("exerciseDetailView").style.display = "none";
}
function showExerciseDetail(id) {
    const exercise = EXERCISES[id];
    if (!exercise) return;
    document.getElementById("exerciseCategoryView").style.display = "none";
    document.getElementById("exerciseListView").style.display = "none";
    document.getElementById("exerciseDetailView").style.display = "";
    document.getElementById("exerciseDetailTitle").innerHTML =
        '<i class="fa-solid ' + exercise.icon + ' sim-exercise-icon"></i>' + exercise.label;
    document.getElementById("exerciseDetailDescription").textContent = exercise.fullDescription;

    const progressEl = document.getElementById("exerciseDetailProgress");
    const startBtn = document.getElementById("exerciseStartBtn");
    const cancelBtn = document.getElementById("exerciseCancelBtn");
    progressEl.classList.remove("sim-exercise-gate-warning");
    startBtn.disabled = false;
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
            const stageLabel = stage.type === "killswitch" ? killswitchDisplayLabel() : stage.label;
            progressEl.textContent = "Pågår: " + stageLabel + (stage.type === "killswitch"
                ? " - " + killswitchStatusText()
                : stage.type === "hover"
                    ? " - " + exerciseState.hoverHoldSec.toFixed(0) + "/" + stage.holdSec + " s"
                    : stage.type === "racing"
                        ? (exerciseState.engaged ? " - port " + exerciseState.wpIndex + "/" + RACE_GATE_CENTERS_2.length : " - kryss start/mål")
                        : " - runde " + exerciseState.lapsCleanCount + "/" + (stage.requiredCleanLaps || REQUIRED_CLEAN_LAPS));
        }
        startBtn.style.display = "none";
        cancelBtn.style.display = "";
    } else {
        const progress = exerciseProgress[id] || { passed: false, bestTimeSec: null };
        const gateBlocked = exercise.requiresGamepadKill && !isGamepadKillBound();
        if (gateBlocked) {
            progressEl.style.display = "";
            progressEl.classList.add("sim-exercise-gate-warning");
            progressEl.textContent = "Krever at Kill/Arm-knappen er bundet til en fysisk fjernkontroll i " +
                "Fjernkontroll-kalibrering (Innstillinger) - tastatur og skjermknappen virker ikke i denne øvelsen.";
        } else if (exercise.stages[0].type === "racing") {
            const bestSec = racingBestTimeSec();
            progressEl.style.display = bestSec !== null ? "" : "none";
            if (bestSec !== null) progressEl.textContent = "Beste rundetid: " + formatExerciseTime(bestSec, 2);
        } else if (progress.passed) {
            progressEl.style.display = "";
            progressEl.textContent = exercise.noTiming ? "Bestått" : "Bestått - beste tid: " + formatExerciseTime(progress.bestTimeSec);
        } else {
            progressEl.style.display = "none";
        }
        startBtn.style.display = "";
        startBtn.disabled = gateBlocked;
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
    updateKillswitchVisuals(now, frameDt);

    updateDroneVisual(frameDt);
    chaseCameraController.update(frameDt, droneState.position, droneState.quaternion);
    updateVlosCamera();
    updateWindsockVisual(now);
    treeSwayManager.update(now, currentWindVector);
    updateClockTower(Date.now());
    updateFlags(now);
    updateShadowCamera();
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
    document.getElementById("armToggleBtn").addEventListener("click", function () { toggleKill("button"); });

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
        // Vis fremgangen for en øvelse som allerede er i gang, ellers øverste nivå (kategorivalget) -
        // samme oppførsel som M-snarveien under (var tidligere showExerciseListView() her, som hoppet
        // rett forbi kategorivalget og inn i den sist viste listen).
        if (exerciseState.active) showExerciseDetail(exerciseState.exerciseId);
        else showExerciseCategoryView();
    });
    document.getElementById("exerciseBackToListBtn").addEventListener("click", function () {
        showExerciseListView(currentExerciseCategory);
    });
    document.getElementById("exerciseBackToCategoryBtn").addEventListener("click", showExerciseCategoryView);
    document.getElementById("categoryStabilizedBtn").addEventListener("click", function () {
        showExerciseListView("stabilized");
    });
    document.getElementById("categoryAcroBtn").addEventListener("click", function () {
        showExerciseListView("acro");
    });

    // Ledertavlens navnefelt - lagres fortløpende (samme Sim.saveJSON-mønster som resten), brukes som
    // "name" på HVER runde som logges fra og med nå (se addRacingLapResult). Gamle rader i listen
    // beholder navnet de ble lagret med.
    const racingPlayerNameInputEl = document.getElementById("racingPlayerNameInput");
    racingPlayerNameInputEl.value = racingLeaderboard.playerName || "";
    racingPlayerNameInputEl.addEventListener("input", function () {
        racingLeaderboard.playerName = racingPlayerNameInputEl.value.trim() || "Pilot";
        saveRacingLeaderboard();
    });
    document.getElementById("racingLeaderboardResetBtn").addEventListener("click", resetRacingLeaderboard);
    renderRacingLeaderboard();
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
        // MÅ preventDefault() alle spillets taster (ikke bare Shift/Ctrl/Space) - ellers kan
        // nettleseren tolke kombinasjonen som sin egen hurtigtast midt i flyging, f.eks.
        // Ctrl (gass ned) + D (roll høyre) = "Legg til bokmerke", Ctrl+R = last siden på nytt,
        // Ctrl+W = lukk fanen, Ctrl+T = ny fane osv.
        if (GAME_KEY_CODES.has(e.code)) {
            e.preventDefault();
        }
        keys.add(e.code);
        if (e.repeat) return;
        switch (e.code) {
            case "Digit1": droneState.flightMode = "stabilized"; break;
            case "Digit2": droneState.flightMode = "althold"; break;
            case "Digit3": droneState.flightMode = "acro"; break;
            case "KeyK": toggleKill("keyboard"); break;
            case "KeyR": handleResetRequest(); break;
            case "KeyC": toggleCamera(); break;
            case "KeyT": togglePanel(document.getElementById("ratesPanel")); break;
            case "KeyH": togglePanel(document.getElementById("helpPanel")); break;
            case "KeyO": toggleFpvHud(); break;
            case "KeyM":
                togglePanel(document.getElementById("exercisesPanel"));
                if (exerciseState.active) showExerciseDetail(exerciseState.exerciseId);
                else showExerciseCategoryView();
                break;
        }
    });
    window.addEventListener("keyup", function (e) {
        keys.delete(e.code);
    });
    // Mister vinduet fokus mens en tast holdes nede (alt-tab, klikk i et annet program, DevTools osv.),
    // kommer keyup ALDRI - tasten ble sittende "fastlåst" nede i settet for alltid, og droneen fortsatte
    // å bevege seg på det gamle input etter at man kom tilbake. Tøm alt ved fokustap/skjult fane.
    window.addEventListener("blur", function () { keys.clear(); });
    document.addEventListener("visibilitychange", function () {
        if (document.hidden) keys.clear();
    });

    requestAnimationFrame(animate);
});
