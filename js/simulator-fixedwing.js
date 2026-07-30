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
// som checkTailStrike, slik at de aldri kan drifte ut av synk med hverandre. De pleide å være separate,
// uavhengige tall, og etter flere runder med modell-justering (skroglengde/hale kortet inn osv.) hadde
// checkTailStrike sin egen, grove tilnærming falt bakpå - det ga et falskt tailstrike-varsel lenge FØR
// halen faktisk var i nærheten av bakken visuelt.
const FUSELAGE_LENGTH_BUILD = 1.35;
const CABIN_RADIUS_BUILD = 0.07;
const CABIN_LEN_RATIO = 0.32;
// Kortet inn (0.38 -> 0.33) - halekjeglen stakk synlig lenger bak enn høyderoret/sideroret (kjeglens
// tupp endte ved z≈0.73, mot høyderorets bakkant på kun z≈0.68, uavhengig av flyklasse siden forskjellen
// domineres av de DELTE, klasse-uavhengige skrog-konstantene) - så kjeglen stakk ut bak hele haleflaten
// i stedet for å avsluttes omtrent der halen sitter. Siden denne er DELT med checkTailStrike (se
// merknaden over), flytter denne fiksen automatisk varselpunktet tilsvarende fremover også - riktig,
// ikke en bieffekt: en kortere kjegle skal treffe bakken ved en litt mindre rotasjonsvinkel i
// virkeligheten også.
const TAIL_LEN_RATIO = 0.33;
const TAIL_TIP_RADIUS_RATIO = 0.16;

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
// Økt (0.15 -> 0.24) - Liten sin rulling var fortsatt treg selv etter å ha kuttet inertiaRoll kraftig
// (0.25 -> 0.12). Det er en viktig forskjell mellom de to: inertia styrer kun hvor RASKT toppfarten nås,
// mens EFFECTIVENESS/DAMPING-FORHOLDET styrer selve toppfarten (steady-state rate = effectiveness/damping
// * qDyn/qDyn, uavhengig av inertia). Å bare kutte inertia mer og mer hjelper ikke hvis toppfarten selv
// er for lav - dette økte selve forholdet (0.15/0.03=5 -> 0.24/0.03=8), som øker toppfarten for ALLE
// klasser (delt konstant), ikke bare responstiden for Liten.
const ROLL_CONTROL_EFFECTIVENESS = 0.24;
const ROLL_DAMPING = 0.03;
// Skråror (aileron) endrer nå OGSÅ vingens EFFEKTIVE angrepsvinkel direkte (i tillegg til den
// eksisterende flate ROLL_CONTROL_EFFECTIVENESS over, beholdt uendret for ikke å rokke ved en nylig
// stabilisert rullrespons) - se rightWing/leftWing i stepPhysics. Dette gir to realistiske effekter
// "gratis" fra samme mekanisme: (1) "adverse yaw" - vingen med nedadgående klaff får mer løft OG mer
// indusert drag, som gir et girmoment MOTSATT rullretningen (grunnen til at koordinerte svinger trenger
// sideror), og (2) mer realistisk spinn/autorotasjon - samme differensial-drag-mekanisme oppstår også
// fra selve ROTASJONEN (ikke bare rorutslag), slik at en steilet/indre vinge i en spinn gir et
// gir-moment som FORSTERKER rotasjonen, akkurat som i et ekte fly.
// Økt (10 -> 22) - dette var det egentlige treghetspunktet, ikke ROLL_CONTROL_EFFECTIVENESS (som bare
// øker TELLEREN i steady-state-forholdet control/demping). Regnet ut faktisk NEVNER for Liten ved
// marsjfart (12 m/s, ~5.85° cruise-AoA for å bære vekten): rollWingDampCoeff (vinge-modellens EGEN,
// nå korrekt lineariserte rate-demping) ≈ 13.9, mot den flate qDynControl*ROLL_DAMPING ≈ 2.6 - vinge-
// dempingen dominerer med over 5x! Å øke ROLL_CONTROL_EFFECTIVENESS (som kun virker på den FLATE, lille
// delen av telleren) kunne derfor aldri monne. AILERON_MAX_AOA_DEG virker derimot gjennom SAMME
// vinge-mekanisme som denne dominerende dempingen kommer fra, og treffer dermed den faktiske flaskehalsen.
const AILERON_MAX_AOA_DEG = 22;
// (To tidligere forsøk på å forsterke "yaw induserer rull" (Clr) - først ved å multiplisere gir-rate-
// bidraget inni rightRot/leftRot direkte (kvalte gir-responsen ved utilsiktet å forsterke gir-dempingen
// like mye), så et eget ekstra-ledd proporsjonalt med rå yawRate (kvalte i stedet rull-autoriteten, siden
// det reagerte likt på YAW FRA ADVERSE YAW også - se historikk i git/samtalelogg). Begge fjernet - nå som
// vinge-modellens egen Clr-kobling (rollWingF0) har riktig linearisert demping, gir den denne effekten
// selv, korrekt skalert til akkurat den kilden til yaw som faktisk er til stede.)
// Dihedral-effekt (Clβ - rull fra SIDESLIP-VINKEL, ikke gir-rate) - se rollTorqueFromDihedral i
// stepPhysics. Denne er VEDVARENDE (ikke rate-avhengig som Clr/Cnp-koblingen over): et sideror holdt
// inne gir et sideslip som varer så lenge roret holdes, og dermed et rullmoment som OGSÅ varer så lenge
// - ikke bare et kortvarig napp. Dette er det som gjør at kryssede ror (skråror én vei, sideror andre
// veien) kan holde flyet i en rett, sideslippende bane, akkurat som en ekte "forward slip".
const DIHEDRAL_EFFECT = 0.02;
// Stigemomentet (pitch) er IKKE lenger et flatt, håndjustert "kontrolleffektivitet"-tall - se
// beregningen av tailArm/tailLift i stepPhysics, som modellerer et ekte vekt-og-balanse-prinsipp:
// halen henger et stykke bak tyngdepunktet og produserer selve stigemomentet fra sitt EGET løft, akkurat
// som i et ekte fly. Dette gir automatisk høyderor-effekt, pitch-demping OG fart-avhengig trimbehov uten
// noen egne håndjusterte konstanter for de tre tingene hver for seg (se kommentaren ved tailArm).
const TAIL_ARM_RATIO = 0.55;      // halens moment-arm fra CG, som andel av vingespennet
const TAIL_AREA_RATIO = 0.22;     // halens areal, som andel av vingearealet (typisk for lette fly)
const TAIL_CL_SLOPE_RATIO = 0.85; // halens løftekurve-helning relativt til hovedvingens
const ELEVATOR_MAX_AOA_DEG = 16;  // grader endring i halens angrepsvinkel ved fullt rorutslag
// Gir-momentet (yaw) brukte tidligere et flatt, håndjustert par (YAW_CONTROL_EFFECTIVENESS/YAW_DAMPING +
// en egen yawStability-konstant per klasse) - i motsetning til stigemomentet, som er en ekte vekt-og-
// balanse-modell. Erstattet med akkurat samme prinsipp som halen: finnen henger på samme moment-arm
// (TAIL_ARM_RATIO - samme skrogstasjon som halen) og produserer et sideveis "løft" fra sin EGEN lokale
// sideslip-vinkel (som inkluderer gir-ratens rotasjonsbidrag). Dette gir sideror-effekt, gir-demping
// (Cnr) OG retningsstabilitet (Cnβ) fra ÉN geometrisk modell, akkurat som halen ga tre pitch-egenskaper
// fra én modell - i stedet for tre separate, håndjusterte tall.
const FIN_AREA_RATIO = 0.13;      // finnens areal, som andel av vingearealet
const FIN_CL_SLOPE_RATIO = 0.75;  // finnens løftekurve-helning relativt til hovedvingens
// Økt (18 -> 32) - dette er en RATIO-bug, ikke en fartsavhengig en: den naturlige (urettede) sideslip-
// VINKELEN finnen ser (baseSlip = atan2(vx,-vz)) er en ren GEOMETRISK vinkel som ikke avhenger av selve
// vind-STYRKEN - selv "litt" vind gir en stor vinkel når fartsvektoren nesten er null (stillestående på
// bakken, eller i utflating/berøring ved landing), siden luften da kommer nesten rett fra siden uansett
// hvor svak vinden er. Ved 18° kunne fullt sideror aldri kansellere en naturlig sideslip på 25-35°
// (vanlig ved lav fart i vind - se finAoaDeg sin egen klemming til ±35°) - piloten hadde rett og slett
// ikke nok rorutslag til å holde nesen rett, uansett hvor mye sideror som ble brukt. Siden BÅDE det
// naturlige momentet og rorets moment skalerer med SAMME dynamiske trykk, er dette en ren gradtall-/
// autoritets-ubalanse, ikke noe som endrer seg med fart - derfor samme fiks for både bakke-vindkantring
// og sideslip-landing.
const RUDDER_MAX_AOA_DEG = 32;    // grader endring i finnens angrepsvinkel ved fullt sideror-utslag
// Se yawDampCoeff i stepPhysics for utregningen (dempingsforhold ζ≈0.29 uten denne, en tydelig
// underdempet "Dutch roll"-lignende gir-oscillasjon) - en ekte yaw-damper-boost, ikke en fiktiv fiks.
const YAW_DAMPER_GAIN = 3;

// Propellstrøm (propwash/slipstream): halen/finnen sitter rett i propellens luftstrøm, som akselereres
// UAVHENGIG av flyets EGEN bakkefart/luftfart - gir aksielt (0.5*rho*A*v²-momentum-teori, se
// propwashDeltaV i stepPhysics) reell rorautoritet (høyderor/sideror) selv ved lav fart så lenge gassen
// står på (f.eks. haleroterende taksing med full gass, eller å "fange" nesen med gass rett før stall/ved
// avgang) - noe som manglet helt før (rorautoriteten der kom KUN fra flyets egen luftfart, som er akkurat
// null der propwash trengs mest).
// VIKTIG (fikset bug - propwash ble helt urealistisk voldsom, kunne gire flyet rundt midt i luften):
// PROPWASH_EFFECTIVE_AREA_RATIO ble først satt til 0.06 (tolket som selve propellskivens fysiske areal,
// ~6% av vingearealet) - momentum-teori-farten v_i går som sqrt(T/A), så et SÅ lite areal ga en statisk
// (V=0) slipstrøm-økning på over 35 m/s for Liten (18N/0.0228m²) - langt mer enn selv marsjfarten (12
// m/s). Arealet her representerer i praksis IKKE selve propellskiven, men hvor mye slipstrømmen har
// SPREDD/blandet seg med omgivelsene idet den når halen/finnen et stykke bak - derfor et langt større
// "effektivt" areal enn selve skiven. Kalibrert til å gi en statisk (stillestående, full gass) fart-økning
// på ca. 5 m/s for alle klasser (tydelig og nyttig ved lav fart/taksing, men ikke urealistisk voldsom) -
// se v_i-formelen i stepPhysics.
const PROPWASH_EFFECTIVE_AREA_RATIO = 3.0;

// Bakkeeffekt: reduserer indusert motstand når vingen er lavere enn ca. ett vingespenn over bakken
// (bakken hindrer vingetupp-virvlene/nedvasken i å utvikle seg fullt ut) - se groundEffectFactor i
// stepPhysics. Dette er det som gir "flyting" (redusert synkefart/lengre utrulling i luften rett før
// touchdown) ved landing, og mindre motstand/kortere rulling rett etter avgang mens flyet ennå er lavt.
// Kun indusert DRAG er modellert (ikke en egen løft-økning) - det er den klart mest merkbare av de to i
// praksis, og unngår å innføre en ekstra, vanskeligere-å-verifisere løft-korreksjon.
const GROUND_EFFECT_HEIGHT_FACTOR = 16; // høyere tall = bakkeeffekten forsvinner raskere med høyde
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
// D-leddet over målte RÅ vinkelrate (finite-difference over ETT tick, 1/120s) uten noe filter - enhver
// liten, forbigående forstyrrelse i vinkelen mellom to fysikk-tick (f.eks. bakkekontaktens svake
// nivelleringskorreksjon, eller bare selve turbulensen i vinge-/hale-modellens ikke-lineære respons)
// blir da FORSTERKET med en faktor på 120 (1/dt) idet den differensieres, og gir seg utslag som synlig
// "hakking" i rorutslaget/rullet - klassisk digital-kontroll-fenomen ("derivative kick"/støyforsterkning
// fra et ufiltrert D-ledd), ikke en reell ustabilitet i selve flyfysikken. Fikset med et lavpassfilter på
// selve raten (samme prinsipp/idiom som AUTO_TRIM_FILTER_TAU under) FØR den ganges med D_GAIN - kort nok
// tidskonstant til at ekte, tilsiktet demping av tilnærmingen mot målvinkelen fortsatt skjer omtrent
// like raskt (den ekte kommanderte raten endrer seg over titalls-hundretalls ms), men lang nok til å
// dempe vekk enkelttick-støy.
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
// Utvidet (6 -> 11) - løftet falt fra topp til kun 30% i løpet av bare 6 grader, et brått nok fall/
// gjenvinning av 70% av vingens løft til å oppleves som et rykk/kast hvis flyet manøvreres nær
// steilegrensen, i stedet for en mer gradvis inntreden.
const STALL_POST_RANGE_DEG = 11; // bredde på overgangssonen rett etter kritisk vinkel før dyp steiling

const ROLLING_FRICTION = 0.045; // rullemotstand (hjul mot asfalt) ved normal rulling/taxi
const BRAKE_DECELERATION = 2.5; // m/s^2 ekstra oppbremsing fra hjulbrems - kun ved gass i tomgang
const GROUND_YAW_FRICTION = 3;  // eksponentiell demping (1/s) av gir-rotasjon fra hjul mot bakken
// Statisk (Coulomb-lignende) motstand mot vindkantring mens flyet står/ruller på bakken - se
// yawTorqueF0 i stepPhysics. Skalert med vekt (mass*GRAVITY), som normalkraften på dekkene i
// virkeligheten - et tyngre fly har mer dekk-grep og er dermed mer motstandsdyktig mot vindkantring.
const GROUND_YAW_FRICTION_TORQUE_COEFF = 0.15;
const GROUND_CLEARANCE_FW = 0.05;
const CRASH_SINK_RATE = 6;      // m/s synkefart ved berøring som teller som hard landing
const CRASH_BANK_DEG = 45;      // krengevinkel ved berøring som teller som hard landing

const FIXED_DT = 1 / 120;       // fysikk-tidssteg, samme substep-mønster (akkumulator) som quad-simulatoren
const STICK_RAMP_TIME = 0.22;
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
        // Lett og godt motorisert (typisk for en liten elektrisk skoleflymaskin) - skal føles nimbel og
        // lettflydd, ikke tung/undermotorisert. Trekkraft/vekt ≈ 0.83.
        // wingArea redusert (strekkforhold ~8.5 i stedet for ~5.9) - forrige vinge var for bred/kort
        // ("låvedør") relativt til skroget. Øker steilefarten noe (~7.6 -> ~9.2 m/s), en akseptabel
        // avveining mot et fly som faktisk ser ut som et fly.
        mass: 2.2, wingArea: 0.38, wingSpan: 1.8,
        maxThrust: 18, cd0: 0.045, inducedDragK: 0.9, clSlope: 0.11, stallAngleDeg: 14,
        // inertiaRoll redusert videre (0.25 -> 0.17 -> 0.12) - fortsatt for treg respons for en liten
        // skoleflymaskin. Merk: siden ROLL_CONTROL_EFFECTIVENESS/-DAMPING er delt mellom alle klasser (se
        // merknad over), endrer lavere inertia KUN hvor RASKT toppfarten nås, ikke selve toppfarten
        // (som styres av forholdet effektivitet/demping, uendret) - trygt å skru ned uten å gjøre roll
        // urealistisk kraftig i steady state, bare raskere å komme dit.
        inertiaRoll: 0.12, inertiaPitch: 0.42, inertiaYaw: 0.42,
        // Økt (fra -0.14) sammen med lengre understell i buildPlane - propellen var lengre enn
        // hjulklaringen og stakk ned i rullebanen.
        gearOffsetY: -0.22, visualScale: 1.0
    },
    medium: {
        label: "Middels",
        mass: 8, wingArea: 0.65, wingSpan: 2.4,
        maxThrust: 22, cd0: 0.04, inducedDragK: 1.0, clSlope: 0.105, stallAngleDeg: 13,
        inertiaRoll: 0.9, inertiaPitch: 1.3, inertiaYaw: 1.5,
        gearOffsetY: -0.28, visualScale: 1.4
    },
    large: {
        label: "Stor",
        mass: 22, wingArea: 1.2, wingSpan: 3.4,
        maxThrust: 42, cd0: 0.035, inducedDragK: 1.1, clSlope: 0.1, stallAngleDeg: 12,
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

// Genererer en enkel, realistisk vingeprofil-kontur (i stedet for et rent rektangel): avrundet forkant,
// buet overside, flat underside, jevnt tilspisset mot bakkant. Bruker en NACA-00xx-lignende symmetrisk
// tykkelsesfordeling (5*t*(0.2969*sqrt(x) - 0.126*x - 0.3516*x² + 0.2843*x³ - 0.1015*x⁴)) - denne
// formelen gir BÅDE den avrundede nesen (sqrt(x)-leddet har uendelig stigning ved x=0, som geometrisk
// gir en butt, avrundet spiss i stedet for et skarpt hjørne) OG den jevnt tilspissede bakkanten (går mot
// 0 ved x=1) helt av seg selv, uten noen egen håndkodet avrunding. Oversiden bruker denne tykkelsen
// direkte (litt forsterket for en tydelig buet/kambret følelse), undersiden bruker SAMME kurve nær
// nesen (for en glatt, avrundet overgang) men blandes raskt over til flat (y=0) innen ~15% korde -
// altså en klassisk "flat-bunn"-profil (Clark-Y-lignende), som brukeren spesifikt ba om.
// xStart/xEnd (0..1, andel av FULL korde chordLen) lar oss bygge "fremre hoveddel" (xStart=0, med den
// ekte avrundede nesen) og "bakre del/balanseror" (xEnd=1, med den ekte tilspissede bakkanten) som to
// separate, men konturmessig SAMMENHENGENDE biter - akkurat som boks-versjonen hadde et fremre/bakre
// skille for balanserorets utsparing på vingetuppen.
// flatBottom (default true): true = kambret vingeprofil (buet overside/flat underside, som brukeren ba
// om for selve vingen); false = SYMMETRISK profil (samme kurve begge veier) - brukt for hale-/finne-
// flater, som i et ekte fly nesten alltid er symmetriske (må gi løft/sideveis kraft like godt begge
// veier - nese opp OG ned, sideror høyre OG venstre - noe en kambret profil ville gjort dårligere den
// ene veien).
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
    // cabinRadius redusert ytterligere (0.095 -> 0.07) - skroget leste fortsatt som en "ubåt/luftskip"
    // (for tykk midje i forhold til lengden) i skjermbildene selv etter forrige runde.
    // fuselageLength/cabinRadius/CABIN_LEN_RATIO/TAIL_LEN_RATIO er DELT med checkTailStrike (se disse
    // konstantene øverst i filen) - IKKE gjør disse til lokale, uavhengige tall igjen, det var nettopp
    // det som fikk tailstrike-varselet til å drifte ut av synk med skroget sist.
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
    // VIKTIG (fikset bug - vingen "fløt" synlig over hele skroget, verst på Middels/Stor): spec.wingSpan/
    // wingArea er EKTE, fysiske verdensrom-mål (brukt direkte i fysikken - se stepPhysics/
    // resolveGroundContact, som IKKE bruker visualScale i det hele tatt, siden fysikken allerede opererer
    // i sanne verdensrom-enheter). Men HELE flygruppen skaleres uniformt med spec.visualScale helt til
    // slutt (group.scale.setScalar under) - så all geometri bygget FØR den skaleringen må være i
    // "bygge-rom" (FØR visualScale), akkurat som fuselageLength/cabinRadius allerede er (se merknaden
    // ved FUSELAGE_LENGTH_BUILD øverst i filen). Vinge/hale/finne/understells-spor ble tidligere bygget
    // DIREKTE fra spec.wingSpan/wingArea (de sanne verdensrom-tallene), og fikk dermed visualScale
    // påført EN GANG FOR MYE - Middels sitt vingespenn endte 1.4x for stort I TILLEGG TIL at det
    // allerede var 1.33x større enn Liten sitt (2.4 vs 1.8), altså faktisk 1.87x - som fikk vingen til å
    // se ut som en overdimensjonert flate som dekket hele skroget, ikke en proporsjonalt montert vinge.
    // Fikset ved å dele wingSpan/wingArea på visualScale (areal på visualScale² - areal skalerer med
    // LENGDE i annen potens) FØR de brukes til å bygge geometri her, akkurat som gearHeight allerede
    // gjorde riktig for understellets HØYDE (se gearHeight-merknaden lenger ned) - resten av denne
    // fila brukte samme fiks kun for det ene tallet, ikke konsekvent for alt som bruker wingSpan/wingArea.
    const buildWingSpan = spec.wingSpan / spec.visualScale;
    const buildWingArea = spec.wingArea / (spec.visualScale * spec.visualScale);
    const wingChord = buildWingArea / buildWingSpan;
    const wingMountY = cabinRadius * 1.3;
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
    // Samme dobbel-skalering-fiks som vingen (se merknaden ved buildWingSpan over) - understellssporet
    // ble bygget fra RÅ spec.wingSpan, som fikk visualScale påført en ekstra gang og ga hjul som satt
    // synlig FEIL i forhold til fysikkens faktiske bakkekontaktpunkter. Koeffisienten er også endret
    // fra 0.22 til 0.25 slik at den er IDENTISK med resolveGroundContact sin egen gearTrack-formel
    // (spec.wingSpan*0.25) - de visuelle hjulene sitter dermed nå NØYAKTIG der fysikken faktisk
    // registrerer bakkekontakt, i stedet for en tilnærmet (og, pga. dobbel-skaleringen, økende feil for
    // større klasser) plassering.
    const gearTrack = buildWingSpan * 0.25;
    const strutLenMain = gearHeight - wheelRadius;
    // VIKTIG (fikset bug - "understellet henger i løse lufta, hjulene har kontakt med bakken men ikke
    // med flyet"): strebene stod tidligere LODDRETT, rett under hjulsporet (side*gearTrack/2) - men
    // hjulsporet er langt bredere enn selve skroget (cabinRadius, kun 0.07 bygge-enheter), så streben
    // sin egen TOPP endte godt UTENFOR skrogsylinderen i X-retning, med et synlig gap mellom skrogets
    // buk og strebens toppunkt (se skjermbildet - bekreftet). Fikset ved å la hver strebe gå DIAGONALT
    // fra et festepunkt PÅ skrogets buk (rett under senterlinjen) og skrå utover til hjulet - akkurat
    // som en ekte "cantilever"-fjærbein (typisk på en høyvinget trener, f.eks. Cessna 152/172-stil),
    // ikke to separate, uavhengige loddrette bein. attachPoint/wheelPoint + setFromUnitVectors orienterer
    // sylinderen langs den faktiske retningen mellom de to punktene i stedet for å anta loddrett.
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
    // VIKTIG (fikset bug - den reelle årsaken til "rullingen hopper fram og tilbake, virker unaturlig"
    // under en steiling): den faste 0.6-faktoren under var UAVHENGIG av selve grensevinkelen
    // (stall+STALL_POST_RANGE_DEG), og falt så å si ALDRI sammen med overgangssonens sluttverdi
    // (peak*0.3) rett over - for f.eks. Middels-klassen (stall=13°) hoppet CL diskontinuerlig fra
    // 0.3*peak til 0.6*sin(48°)*peak ≈ 0.446*peak (et ~49% sprang) I SAMME TICK som AoA krysset 24°.
    // Siden vingens AoA naturlig svinger litt fram og tilbake rundt akkurat denne grensen under en
    // steiling (fra egen rotasjon/turbulens i selve steilingen), hoppet løftet - og dermed rull-/
    // gir-dreiemomentet - diskontinuerlig hver gang grensen ble krysset, som ga nettopp en rykkete,
    // retningsvekslende rulling i stedet for en jevn vingedypp. Fikset ved å skalere flate-plate-
    // formelen slik at den er KONTINUERLIG med overgangssonen akkurat ved grensevinkelen, uansett
    // klassens stallAngleDeg (peak*0.3 forblir uendret - kun HVORDAN den fortsetter videre forbi
    // grensen er glattet ut).
    const boundaryDeg = stall + STALL_POST_RANGE_DEG;
    const boundaryRaw = Math.abs(Math.sin(2 * THREE.MathUtils.degToRad(boundaryDeg)));
    const flatPlateScale = boundaryRaw > 0.05 ? 0.3 / boundaryRaw : 0.6;
    return sign * peak * flatPlateScale * Math.abs(Math.sin(2 * THREE.MathUtils.degToRad(absA)));
}

// groundEffectFactor (0..1, default 1 = ingen effekt) skalerer KUN den induserte motstanden - se
// GROUND_EFFECT_WINGSPAN_FACTOR-merknaden ved beregningen i stepPhysics for hvorfor kun indusert drag
// (ikke løft) er modellert: bakkeeffekten kommer av at bakken hindrer vingens nedvask-virvler i å
// utvikle seg fullt ut, som reduserer indusert motstand - det er den effekten som gir "flyting"/økt
// rekkevidde i bakkeeffekt ved landing, og er den vanligste, mest merkbare av de to.
function dragCoefficient(aoaDeg, spec, groundEffectFactor) {
    const aoaRad = THREE.MathUtils.degToRad(aoaDeg);
    let cd = spec.cd0 + spec.inducedDragK * aoaRad * aoaRad * (groundEffectFactor === undefined ? 1 : groundEffectFactor);
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
    // flyet har fart): tvinger orienteringen HELT til identitet (rett ned rullebanen, nivå). VIKTIG: kun
    // FØR flyet noensinne har vært i luften (hasBeenAirborne) - denne fikk opprinnelig lov til å kjøre
    // ubetinget for ENHVER lav fart, som var en reell bug: et fly som lander og bremser ned til under
    // 0.4 m/s (helt normalt ved full stopp etter landing) ble da teleportert 180°/til vilkårlig retning
    // TILBAKE til spawn-retningen, uansett hvilken vei det faktisk pekte etter en helt normal landing og
    // utrulling. Nå gjelder identitets-tvangen kun i det smale vinduet "har aldri lettet ennå" (parkert
    // rett etter reset/spawn), der det opprinnelige problemet (uforklarlig dreining) faktisk oppsto -
    // vinkelhastighetene nullstilles fortsatt uansett (forhindrer restspinn ved full stopp, uavhengig av
    // retning - det er alltid riktig).
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
    const thrustForce = planeState.engineOn ? throttleShaped * spec.maxThrust : 0;

    const q = planeState.quaternion;
    const invQ = q.clone().invert();
    const airVelWorld = planeState.velocity.clone().sub(currentWindVector);
    const localAirVel = airVelWorld.clone().applyQuaternion(invQ);
    const airspeed = airVelWorld.length();
    lastAirspeed = airspeed;
    const aoaDeg = airspeed > 0.3 ? THREE.MathUtils.radToDeg(Math.atan2(-localAirVel.y, -localAirVel.z)) : 0;
    const sideslipDeg = airspeed > 0.3 ? THREE.MathUtils.radToDeg(Math.atan2(localAirVel.x, -localAirVel.z)) : 0;

    // Propellstrøm (propwash) - se PROP_DISK_AREA_RATIO-merknaden ved konstanten. Momentum-teori for en
    // aktuator-skive: skiven induserer en hastighet v_i som løser T = 2*rho*A*v_i*(V0+v_i), der V0 er
    // flyets EGEN forover-luftfart inn i skiven (klemt til >=0 - motoren "suger" uansett, selv i revers-
    // eller null-fart). Full slipstrøm-hastighetsøkning langt bak skiven (der halen/finnen sitter) er
    // 2*v_i = sqrt(V0² + 2T/(rho*A)) - V0 (løsning av annengradslikningen over). Denne ekstra farten
    // legges KUN til halens/finnens EGEN lokale luftstrøm (se tailTorqueAtPitchRate/finTorqueAtYawRate),
    // ikke vingens - en enkeltmotors nese-propell vasker i praksis kun skrog/hale, ikke hele vingespennet.
    const propwashEffectiveArea = spec.wingArea * PROPWASH_EFFECTIVE_AREA_RATIO;
    const forwardAirspeedIntoProp = Math.max(-localAirVel.z, 0);
    const propwashDeltaV = thrustForce > 0.01
        ? Math.sqrt(forwardAirspeedIntoProp * forwardAirspeedIntoProp + (2 * thrustForce) / (AIR_DENSITY * propwashEffectiveArea)) - forwardAirspeedIntoProp
        : 0;

    // Bakkeeffekt - se GROUND_EFFECT_HEIGHT_FACTOR-merknaden ved konstanten. h/b = høyde over bakken
    // (her brukt direkte som posisjonens Y - vingen sitter uansett nær CG-høyden på disse småflyene)
    // delt på vingespennet. Faktoren går mot 0 (ingen indusert drag) helt ved bakken, og mot 1 (ingen
    // effekt) idet høyden nærmer seg/overstiger ett vingespenn - en vanlig, enkel empirisk tilnærming
    // (Wieselsberger-lignende form).
    const groundEffectRatio = GROUND_EFFECT_HEIGHT_FACTOR * Math.max(planeState.position.y, 0) / spec.wingSpan;
    const groundEffectFactor = (groundEffectRatio * groundEffectRatio) / (1 + groundEffectRatio * groundEffectRatio);

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
        // Automatisk sideror-koordinering (proporsjonal med KOMMANDERT krengning) fjernet - den var en
        // åpen løkke uten tilbakekobling på flyets faktiske gir-tilstand, og fungerte mot en gammel, svak,
        // flat gir-modell. Nå som finnen har en ekte, langt sterkere aerodynamisk respons (egen
        // retningsstabilitet OG demping, se finTorqueAtYawRate) OG vingene gir ekte adverse yaw fra
        // skråror (yawTorqueFromDragDiff), fightet den statiske koordineringen mot den nye, realistiske
        // fysikken i stedet for å hjelpe den - det viste seg som periodisk sideror-oscillering i svinger.
        // Sideroret i Stabilized er nå rent pinnestyrt (samme som Manual) - selve retningsstabiliteten og
        // koordineringen kommer naturlig fra aerodynamikken, ikke fra en kunstig autopilot-hjelp.
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
    // klaff = mer AoA, oppadgående = mindre) - se AILERON_MAX_AOA_DEG-merknaden ved konstanten. Dette
    // legges OVENPÅ den eksisterende rotasjonsbaserte differensialen, så begge kilder til vingedypp/
    // spinn-tendens (rorutslag OG ren rotasjon) bruker nå samme mekanisme.
    // VIKTIG (fikset strukturell bug - den EGENTLIGE årsaken til at rull konsekvent føltes "treg" mens
    // høyderor/sideror var raske, til tross for fire tuning-forsøk): rorutslaget ble tidligere lagt
    // OVENPÅ vingens egen base-AoA FØR liftCoefficient()'s steilekurve - altså akkurat samme ikke-
    // lineære kurve som gir tip-stall/spinn-tendens ved ren rotasjon. Ved lav fart (nettopp der rull ble
    // testet - under/rett etter takeoff-rullingen, hvor vingen uansett trenger høy AoA for å bære
    // vekten) presset et fullt rorutslag (opptil 11° per vinge) den nedadgående vingen godt inn i
    // steilingens avtagende/flate område, som kuttet en stor del av differensialløftet nettopp når det
    // trengtes mest - mens ELEVATOR_MAX_AOA_DEG/RUDDER_MAX_AOA_DEG ALDRI går gjennom noen steilekurve
    // (se tailTorqueAtPitchRate/finTorqueAtYawRate - rent lineær CL = slope*AoA). Fikset ved å gi
    // skrårorets bidrag samme behandling som høyderor/sideror: et EGET, tilnærmet lineært CL-tillegg
    // (clSlope*rorutslag) lagt til ETTER at vingens EGEN base-AoA har gått gjennom steilekurven, ikke
    // FØR - vingens egen steiling (fra reell AoA/rotasjon) er fortsatt fullt intakt og upåvirket.
    const rightWing = { airspeed: rightWingBase.airspeed, aoaDeg: rightWingBase.aoaDeg, controlAoaDeg: -rollDeflection * AILERON_MAX_AOA_DEG * 0.5 };
    const leftWing = { airspeed: leftWingBase.airspeed, aoaDeg: leftWingBase.aoaDeg, controlAoaDeg: rollDeflection * AILERON_MAX_AOA_DEG * 0.5 };

    const halfWingArea = spec.wingArea / 2;
    const liftRight = 0.5 * AIR_DENSITY * rightWing.airspeed * rightWing.airspeed * halfWingArea * (liftCoefficient(rightWing.aoaDeg, spec) + spec.clSlope * rightWing.controlAoaDeg);
    const liftLeft = 0.5 * AIR_DENSITY * leftWing.airspeed * leftWing.airspeed * halfWingArea * (liftCoefficient(leftWing.aoaDeg, spec) + spec.clSlope * leftWing.controlAoaDeg);
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
        const liftMag = qDynWing * (liftCoefficient(baseAoaDeg, spec) + spec.clSlope * extraAoaDeg);
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
    // FJERNET (var rollTorqueFromYawRate): dette kunstige ekstra-leddet ("gir induserer rull", et fast
    // forsterket bidrag proporsjonalt med RÅ yawRate) ble lagt til fordi vinge-modellens EGEN Clr-kobling
    // (rollWingF0 sin yawRate-avhengighet) den gang var for svak til å merkes. Nå som vinge-modellens
    // demping er ordentlig fikset (linearisert rundt gjeldende rate, se linearizeDamping), gir den ekte
    // fysikken denne koblingen selv, riktig skalert. Verre: leddet reagerte på ALL yawRate, uansett
    // KILDE - inkludert ADVERSE YAW fra egne skråror (en reell, tilsiktet effekt: å rulle høyre skaper
    // gir venstre). Siden leddet kommanderte rull i "samme retning som yaw", men adverse yaw fra en
    // HØYRE-rulling peker VENSTRE, ga det en rullmotstand som VOKSTE med skrårorets eget utslag - en
    // selvforsterkende tilbakekobling som kvalte rull-autoriteten, og som IKKE ble bedre av å øke
    // ROLL_CONTROL_EFFECTIVENESS eller kutte inertiaRoll (begge gjorde tilbakekoblingen tilsvarende
    // sterkere). Dette var den reelle årsaken til vedvarende treg rulling til tross for to
    // tuning-forsøk.

    const qDynTotal = 0.5 * AIR_DENSITY * airspeed * airspeed * spec.wingArea;
    const dragMag = qDynTotal * dragCoefficient(aoaDeg, spec, groundEffectFactor);
    // Dihedral-effekt: sideslip (IKKE gir-RATE, men selve sideslip-VINKELEN) gir et VEDVARENDE
    // rullmoment som består så lenge sideslippet varer - ikke bare en forbigående ting under selve
    // rotasjonen (det er yawTorqueFromDragDiff over). Dette gjør at sideror alene etablerer OG holder en
    // krengning (til den motvirkes med skråror), og at "kryssede ror" (skråror én vei, sideror andre
    // veien) kan holde flyet rett fram i en ekte sideslip/forward slip, akkurat som i et ekte fly.
    const rollTorqueFromDihedral = -DIHEDRAL_EFFECT * sideslipDeg;

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

    // Dreiemoment (roll/yaw) = avbøyning * dynamisk trykk * kontrolleffektivitet, dempet av et moment
    // proporsjonalt med flyets egen vinkelhastighet (aerodynamisk demping). Begge ledd skalerer med V^2 -
    // svaret blir derfor naturlig trått/"mushy" ved lav fart og fast ved høy fart, uten noen kunstig
    // svekkingsfaktor. Samme aksekonvensjon-negasjon (forward=-Z) som resten av fysikken/quad-simulatoren.
    const qDynControl = 0.5 * AIR_DENSITY * airspeed * airspeed;

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
        const baseAoa = speedSq > 0.09 ? THREE.MathUtils.radToDeg(Math.atan2(-v.y, -v.z)) : 0;
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

    // VIKTIG (fikset bug - fysisk FEIL retning ved store rater, f.eks. en fullt utviklet spinn, ikke bare
    // en stabilitets-/støyfiks): den forrige semi-implisitte behandlingen lineariserte ALLTID rundt
    // rate=0 (F0 = moment NÅR raten er null, k = stigningstall NÆR null). Det er en fin tilnærming for
    // små svingninger nær null rate, men en DÅRLIG tilnærming langt unna - f.eks. midt i en etablert
    // spinn, hvor den faktiske raten er stor og modellen er dypt inne i ikke-lineære soner (AoA-klemming,
    // steilekurvens knekkpunkt). Siden F0 alltid ble hentet ved rate=0 og BARE den lineære k-delen fikk
    // "se" den faktiske raten (via eksponentiell nedgang), gikk hele det ikke-lineære bidraget FORBI
    // rate=0 og opp til faktisk rate tapt - modellen kunne dermed oppføre seg kvalitativt galt (ikke bare
    // unøyaktig) ved store rater. Fikset ved å linearisere RUNDT GJELDENDE RATE i stedet for null (se
    // linearizeDamping): F0 rekonstrueres slik at eksplisitt-steget + eksponentiell-nedgang-steget
    // reproduserer NØYAKTIG riktig moment akkurat i dette tidssteget (verifisert med
    // førsteordens-utvidelse), mens fortsatt å være ubetinget stabil for enhver k*dt/treghet.
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
    // Total gir-demping = finnens EGEN (Cnr) + vinge-differensialens (over) - begge er reelle,
    // uavhengige dempende bidrag til SAMME akse og skal legges sammen, ikke velges mellom. Så multiplisert
    // med YAW_DAMPER_GAIN - regnet ut (finArm²*0.5*rho*V*finArea*finClSlope*(180/π)) mot finnens egen
    // stivhet (Cnβ) for Liten ved 15 m/s: dempingsforhold ζ ≈ 0.29 (kritisk dempet er ζ=1) - en klart
    // underdempet "Dutch roll"-lignende modus, som stemmer nøyaktig med "nesa hopper ut, dyttes tilbake,
    // oscillerer" som ble rapportert. Dette er et EKTE fenomen (mange virkelige fly trenger en faktisk
    // yaw-damper-boks nettopp av denne grunn) - boost gir ζ ≈ 0.87, komfortabelt dempet uten å bli sløvt.
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
    // Halepunktet er nå den FAKTISKE halekjegle-tuppen (samme FUSELAGE_LENGTH_BUILD/CABIN_LEN_RATIO/
    // TAIL_LEN_RATIO/TAIL_TIP_RADIUS_RATIO-konstanter som buildPlane, skalert til verdensrom med
    // visualScale) - ikke en grov, uavhengig tilnærming som kunne drifte ut av synk med skroget etter
    // modell-justeringer (se konstantene øverst i filen for historikken - dette ga tidligere et falskt
    // varsel lenge før halen faktisk var i nærheten av bakken).
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

// Gjenbrukte "scratch"-objekter for resolveGroundContact - se merknaden ved _tailStrikeScratch over
// (samme begrunnelse: denne kjører hver eneste fysikk-tick mens flyet er på bakken, dvs. under HELE
// taksing/takeoff-rulling/landing-utrulling - tett nok til at repeterte Vector3/Euler/Quaternion-
// allokeringer her var en reell, målbar kandidat for periodisk GC-hakking under lengre flighter).
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

    // VIKTIG: sjekker mot 0 (faktisk gjennomtrengning), ikke mot -GROUND_CLEARANCE_FW. Det siste var en
    // reell bug - det behandlet ALT innenfor en 5 cm klaringssone som "på bakken" og trakk flyet ned mot
    // bakken igjen (se position.y += maxPenetration under) SELV når hjulene allerede var et par cm klar.
    // I praksis "sugde" dette flyet ned igjen gjentatte ganger under selve avgangen, helt til det klarte
    // å hoppe HELE 5 cm-sonen i ett eneste fysikk-tidssteg - noe som gjorde en helt normal, gradvis
    // lettelse (som fungerer fint i luften et par sekunder senere) unaturlig vanskelig under selve
    // rulling/rotasjon på rullebanen.
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

    // VIKTIG (fikset bug - kvalte all rull-respons og ga oscillasjoner under takeoff-rulling): denne
    // slerpet ALLTID mot NØYAKTIG null krengning (4*dt ~3.3% korreksjon PER TICK, som kombinert med
    // *=0.5 på vinkelhastigheten HVER TICK er en svært aggressiv, kunstig "lås til vater" - den kjørte
    // UBETINGET hele tiden flyet var på bakken, og fightet dermed AKTIVT ethvert skråror-utslag under
    // hele takeoff-rullingen. To ting som begge prøver å bestemme krengevinkelen samtidig (denne harde
    // slerpen MOT nøyaktig 0°, og den aerodynamiske rull-modellen som svarer på pinnen) ga nettopp en
    // dra-kamp/oscillasjon, i tillegg til å gjøre rull "treg" siden halvparten av vinkelhastigheten ble
    // visket bort hver eneste tick uansett årsak. Betydelig dempet (0.3*dt i stedet for 4*dt, ingen
    // direkte vinkelhastighets-halvering lenger) - gir fortsatt en svak tendens mot vater (hindrer at
    // flyet blir stående og krenge helt urealistisk mens det er parkert), uten å overstyre pilotens
    // egen skråror-kommando under rulling.
    _groundContactEuler.setFromQuaternion(planeState.quaternion, "YXZ");
    _groundContactEuler.set(_groundContactEuler.x, _groundContactEuler.y, 0, "YXZ");
    _groundContactQuat.setFromEuler(_groundContactEuler);
    planeState.quaternion.slerp(_groundContactQuat, Math.min(1, 0.3 * dt));
    // Hjulene har rotasjonsfriksjon mot underlaget - uten denne demperen ville en gir-rotasjon (yaw)
    // fortsette for evig i stillstand med motoren av, siden det aerodynamiske dempeleddet i stepPhysics
    // (som skalerer med farten i andre potens) blir null når flyet står stille.
    // VIKTIG (fikset bug): dette kjøres i et FAST fysikk-tidssteg (FIXED_DT, 120 ganger/sekund) - en ren
    // "*= 0.85" per tick tilsvarer 0.85^120 ≈ 3*10^-9 PER SEKUND, som knuser all gir-rotasjon på bakken
    // nesten momentant (innen 0.1-0.2s), ikke en gradvis oppbremsing. Byttet til eksponentiell,
    // tidssteg-uavhengig demping (samme prinsipp som rollDampDecay/yawDampDecay i stepPhysics).
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
