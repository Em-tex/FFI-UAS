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
// maxYawRateDeg: øvre TAK på yaw-rate uansett hva brukeren selv setter i Rates-panelet (se
// effectiveYawRates lenger ned) - inertiaYaw ALENE (over) gir riktig RETNING (tyngre klasser er tregere å
// spinne opp/ned i yaw), men uten et eget tak ville de likevel til slutt nådd akkurat samme TOPPFART som
// Racing gitt nok tid, siden selve rates.yaw.maxRate er én delt verdi for alle klasser. Ekte cinematic-
// rigger konfigureres typisk til en MYE lavere maks yaw-rate enn racing-droner (gjerne 60-150°/s, uavhengig
// av selve treg-/rakskheten) - både fordi rask yaw på en tung rigg er upraktisk/ukontrollert, og fordi jevne,
// rolige panoreringer er selve poenget med cinematic-opptak.
const DRONE_CLASSES = {
    racing: {
        label: "Racing (rask, lett)",
        mass: 0.5, maxThrust: 18,
        inertiaRollPitch: 0.025, inertiaYaw: 0.05, maxYawRateDeg: 800, // høyt nok til i praksis ikke begrense noen vanlig Rates-instilling
        dragLinear: 0.2, dragQuad: 0.006, visualScale: 0.72 // kun visuell størrelse - massen er uendret
    },
    mid: {
        label: "Middels",
        mass: 1.2, maxThrust: 24,
        inertiaRollPitch: 0.07, inertiaYaw: 0.14, maxYawRateDeg: 250,
        dragLinear: 0.35, dragQuad: 0.02, visualScale: 1.3
    },
    cinematic: {
        label: "Cinematic (stor, treg)",
        mass: 2.6, maxThrust: 35,
        inertiaRollPitch: 0.16, inertiaYaw: 0.32, maxYawRateDeg: 120,
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

// Yaw-spesifikk bakkefriksjon (BUG rapportert av brukeren: "quaden kan yawe rundt mens den står på
// bakken - nå kan den det med null throttle og yaw input"): en ekte quad står med hele vekten sin på
// beina/rammeunderkanten mens den er i ro på bakken - normalkraften der gir en friksjon (mu*N*benavstand)
// som er LANGT sterkere enn det puslete reaksjonsmomentet fire propeller kan yawe imot (derfor har
// inertiaYaw alltid høyere treghet enn rull/pitch i utgangspunktet, se DRONE_CLASSES-kommentaren - selv i
// FRI luft er yaw den svakeste aksen). Piloten skal derfor ikke merke NOE av yaw-pinnen før nok av vekten
// er lettet av beina at friksjonen slipper taket - se groundYawAuthority i stepPhysics.
// Bruker PILOTENS ØNSKEDE gass (baseCmd, FØR motor-mikseren) i stedet for den faktisk oppnådde
// trekkraften etter mikser/airmode - uten mixMotors kan et fullt yaw-utslag alene løfte to av fire
// motorer til 50% (se MOTOR_MIX/mixMotors-kommentaren), som ellers ville gitt en falsk egen-forsterkende
// løkke der "yawe hardt nok" i seg selv later som beina lettes og låser opp MER yaw-autoritet.
const GROUND_YAW_UNLOCK_THRUST_FRAC_START = 0.5;  // andel av hover-trekkraft (vekt) før friksjonen begynner å slippe
const GROUND_YAW_UNLOCK_THRUST_FRAC_FULL = 0.95;  // andel av hover-trekkraft der beina er tilnærmet vektløse -> full yaw-autoritet

const PASSIVE_ANGULAR_DAMPING = 0.995; // demping av rotasjon når disarmet (fritt fall/tumling)
const ANGLE_P_GAIN = 6;             // ytre selvnivellerings-lookk (Stabilized/Alt Hold), 1/s
const MAX_SELF_LEVEL_ANGLE = 35;    // grader
const MAX_CLIMB_RATE = 4;           // m/s, Alt Hold
const ALT_GAIN = 6;                 // N per (m/s) avvik i Alt Hold
const ALT_HOLD_DEADBAND = 0.12;     // ±12% rundt 50% gass regnes som "hold høyde"
// Loiter (posisjonsholding, se stepPhysics) - TRE faser, matcher ArduCopters egen todeling mellom BREMSING
// og POSISJONSHOLDING (brukeren delte skjermbilder av Mission Planner: "Position XY (Dist to Speed)" ->
// "Velocity XY (Vel to Accel)", OG Loiter-wikisiden sine egne LOIT_BRK_*-parametre for selve bremsingen):
//  1) FLYGING (pinnen utslått, se LOITER_STICK_DEADBAND): pinnen kommanderer farten direkte, som før.
//     Holdepunktet (loiterTargetPos) flyttes kontinuerlig med droneen.
//  2) BREMSING (pinnen sluppet, men farten fortsatt over LOITER_TARGET_LOCK_SPEED): ØNSKET fart er rett og
//     slett 0 - INGEN posisjonsledd ennå. Holdepunktet fortsetter å flyttes med droneen. BUG rettet: forrige
//     versjon brukte posisjons-P-leddet (fase 3) med samme LØSE holdepunkt som var "frosset" akkurat idet
//     pinnen ble sluppet, mens droneen fortsatt hadde full fart - det tvang en aktiv "flyging TILBAKE" mot
//     et gammelt punkt langt bak, i stedet for bare å bremse rett ned, og kunne bygge opp fart i MOTSATT
//     retning idet den for-aggressivt jaget punktet (bruker rapporterte nettopp dette).
//  3) HOLDING (farten har falt under LOITER_TARGET_LOCK_SPEED - droneen har reelt stanset): HER, og først
//     HER, låses holdepunktet til der den faktisk endte, og posisjons-P-leddet (LOITER_POS_P_GAIN) tar over
//     for å rette opp SMÅ, vedvarende avvik (vind) - ikke store transportetapper.
// 2) og 3) sin fart-feil mates uansett inn i samme fart-P-I-D-lookk (LOITER_VEL_TO_LEAN_DEG/-WIND_I_GAIN/
// -VEL_D_GAIN) som omsetter den til krengevinkel.
const LOITER_MAX_SPEED = 12;        // m/s, maks kommandert horisontal fart ved fullt pinneutslag (ArduCopter
// sin egen LOIT_SPEED_MS er ofte tunet opp mot 12-13 m/s på raske droner, se skjermbildet - langt over
// default på 5 m/s).
// Dødsone rundt sentrert pinne (0..1, samme skala som stick.pitch/roll) - under denne regnes pinnen som
// "sluppet" og bremse-/holdefasen (over) tar over. Litt strammere enn f.eks. FBWB_STICK_DEADBAND i
// VTOL-simmen (0.05) siden roll/pitch her - i motsetning til der - skal føles direkte responsivt for enhver
// reell utslag, ikke bare store/bevisste bevegelser.
const LOITER_STICK_DEADBAND = 0.03;
// Fartsterskel (m/s) som utløser overgangen fra BREMSING (fase 2) til HOLDING (fase 3) - se
// droneState.loiterHolding i stepPhysics. Lav nok til at overgangen selv ikke er merkbar, høy nok til at
// den faktisk nås i praksis i stedet for å henge like under toppfart i lang tid.
// BUG rettet: forrige versjon sjekket denne terskelen PÅ NYTT hver eneste tick (ingen egen "har vi allerede
// låst?"-tilstand) - selv en LETT vindkast-lerp fikk farten til å krysse frem og tilbake over 0.5 m/s, og
// hver gang den krysset OPP falt simmen tilbake til fase 2 (som IKKE korrigerer posisjon, bare farten) og
// FLYTTET holdepunktet til der den akkurat da befant seg - i praksis kunne droneen dermed aldri "låse seg"
// ordentlig i vind, og drev umerkelig fra tick til tick uten at noe FAKTISK dro den tilbake ("flyter" -
// rapportert av brukeren). droneState.loiterHolding er nå en ekte TILSTAND (ikke en terskel som sjekkes på
// nytt hver tick): når den først blir true, blir den TIL PILOTEN FLYR AKTIVT IGJEN (fase 1) - en forbigående
// fartsøkning fra et vindkast midt i holdefasen slår den ALTSÅ ikke tilbake til fase 2 lenger, posisjons-
// P-leddet fortsetter i stedet å jobbe mot det samme, faste holdepunktet gjennom hele kastet.
const LOITER_TARGET_LOCK_SPEED = 0.5;
// ArduCopters egen PSC_NE_POS_P ("Position XY (Dist to Speed)") er nettopp P=1.0 i sine egne cm/cm-per-s-
// enheter - samme forholdstall (1 m/s ønsket fart per meter avvik) er en velprøvd verdi å gjenbruke direkte
// her. Klemmes uansett til LOITER_MAX_SPEED (se stepPhysics), så et stort avvik gir aldri et urealistisk
// fartshopp - i praksis er avvikene som når fram til DENNE loopen nå uansett små (kun fase 3, se over), så
// det sjeldent er relevant.
const LOITER_POS_P_GAIN = 0.9;      // (m/s) per m avvik
// BUG rettet (AVGJØRENDE runde - se flightlogg brukeren limte inn): 7 var rett og slett for MYE lukket-
// lookk-forsterkning oppå de allerede tunede vinkel-/rate-lookkene (ANGLE_P_GAIN/TORQUE_GAIN) - loggen viste
// en VEDVARENDE, IKKE-avtagende svingning i BÅDE bank og pitch, med ren pinneP=1.00/pinneR≈0.00 (ingen
// gir-input involvert i det hele tatt store deler av loggen), periode ~1-1.5s, som gjentatte ganger slo i
// taket ±45°. Det er en ekte ustabilitet (for lite faseskudd-margin i den sammensatte lookken), IKKE støy -
// et lavpassfilter hjelper ALDRI mot dette (svingningen ligger godt UNDER filterets grensefrekvens), og et
// D-ledd på et FILTRERT (dermed faseforsinket) signal kan faktisk FORVERRE stabiliteten ved akkurat denne
// frekvensen i stedet for å dempe den. Redusert kraftig i stedet for å filtrere/dempe mer.
const LOITER_VEL_TO_LEAN_DEG = 3.5; // grader krengevinkel per m/s fartsavvik (P-ledd), klemmes til LOITER_MAX_LEAN_ANGLE
// Egen (høyere) krengevinkel-takk enn MAX_SELF_LEVEL_ANGLE - Loiter skal kunne kaste inn mer krengning enn
// Stabilized/Alt Hold for å faktisk klare å holde posisjonen i sterk vind (se LOITER_MAX_WIND_SPEED) og
// følge et pinneutslag helt til LOITER_MAX_SPEED. 45° gir god margin: nødvendig vinkel for å stå stille i
// LOITER_MAX_WIND_SPEED vind (utregnet fra drag-modellen per droneklasse) ligger på 29-32°, og for å FLY
// LOITER_MAX_SPEED i marsjfart på 37-42° - begge godt innenfor taket, uten å måtte gå til et urealistisk
// ekstremt "racing"-aktig lenevinkel-tak.
const LOITER_MAX_LEAN_ANGLE = 45;   // grader
// I-ledd (se loiterIntegralFwd/-Right i stepPhysics) - et RENT P-ledd krever en VEDVARENDE fartsFEIL for
// å holde en gitt krengevinkel oppe, så mot en konstant vind ville droneen aldri stoppe helt opp (den
// måtte stadig drifte litt for at feilen skal holde korreksjonsvinkelen aktiv) - I-leddet bygger sakte opp
// en egen vinkel-bias som til slutt bærer HELE motvirkningen alene, slik at fartsfeilen (og dermed driften)
// kan gå mot null selv i vedvarende vind. Klemt til LOITER_WIND_I_MAX_DEG for å unngå "integral windup" -
// BUG rettet: klemt til kun 15° tidligere, mens opptil ~32° faktisk trengs for å stå stille i sterk vind,
// så I-leddet kunne ALDRI ta over hele jobben og en liten, vedvarende drift ble stående igjen uansett hvor
// lenge man ventet. LOITER_WIND_I_MAX_DEG matcher derfor nå LOITER_MAX_LEAN_ANGLE - samme prinsipp som
// ArduCopters egen IMAX (satt langt over det som normalt trengs, ikke stramt rundt et anslått behov).
const LOITER_WIND_I_GAIN = 1.0;     // grader/s opphopet bias per m/s vedvarende fartsfeil
const LOITER_WIND_I_MAX_DEG = LOITER_MAX_LEAN_ANGLE;
// D-ledd (se loiterVelFwdFilt/-RightFilt i stepPhysics) - dempning på selve FARTSENDRINGEN (akselerasjonen),
// ikke feilen: bremser INN mot null idet farten nærmer seg ønsket verdi, slik at en rask oppbremsing (f.eks.
// rett etter pinnen slippes i fart) stopper PRESIST i stedet for å suse forbi og måtte hentes tilbake.
// BUG rettet (AVGJØRENDE runde - se LOITER_VEL_TO_LEAN_DEG-kommentaren): et D-ledd som virker på et FILTRERT
// (dermed faseFORSINKET) signal legger til akkurat den galt-tidede korreksjonen som kan FORVERRE en
// svingning i stedet for å dempe den, ved frekvenser nær filterets grensefrekvens - stikk i strid med D sin
// vanlige jobb. Holdt lav med VILJE av denne grunnen (i tillegg til at det - som notert tidligere - uansett
// bidrar 0 til selve steady-state-holde-presisjonen, den bæres av P+I).
const LOITER_VEL_D_GAIN = 0.4;      // grader per m/s² (filtrert) fartsendring
// Løsnet fra 0.08 (som la til betydelig faseforsinkelse, se LOITER_VEL_D_GAIN-kommentaren og hoved-BUG-
// notatet ved LOITER_VEL_TO_LEAN_DEG) - fanger fortsatt opp ekte tick-til-tick-måle-jitter, uten å forsinke
// D-leddet nok til å bli en destabiliserende faktor i seg selv ved svingefrekvensen som faktisk ble observert.
const LOITER_VEL_FILTER_ALPHA = 0.25; // eksponensiell glatting, samme mønster som windGustOffset sin egen
// Sim.computeWind-lerp - ved 120Hz gir dette en tidskonstant på ~40ms (grensefrekvens ~4 Hz).
// Fartsbegrensning (grader/s) på selve DEN KOMMANDERTE lenevinkelen (desiredPitchAngle/-RollAngle over) - et
// sikkerhetstak mot at én enkelt tick kan hoppe til FULL krengevinkel momentant. Dempet noe tilbake fra 150
// (som i praksis knapt begrenset noe, og dermed ikke bidro med noen ekstra dempings-margin mot ustabiliteten
// beskrevet ved LOITER_VEL_TO_LEAN_DEG) - fortsatt raskere enn de opprinnelige, "sluggish"-rapporterte 40°/s.
const LOITER_MAX_ANGLE_RATE = 60;   // grader/s
// Vindstyrke Loiter er dimensjonert for å holde posisjon i (se vindwarning i updateHud) - ved sterkere
// vind enn dette kan I-leddet/krengevinkeltaket over bli utilstrekkelige, og piloten varsles.
const LOITER_MAX_WIND_SPEED = 10;   // m/s
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

// Rekkefølge følger tastesnarveiene (1/2/3/4, se Digit1-4-håndteringen lenger ned) - avgjør også
// rekkefølgen i modus-popoveren (buildModePopover), siden den bare looper Object.keys(MODE_LABELS).
const MODE_LABELS = { stabilized: "Stabilized", althold: "Alt Hold", loiter: "Loiter", acro: "Acro" };
// Tastesnarveier og forklaringstekst til modus-popoveren (klikk på "Modus" i HUD-en, se buildModePopover
// lenger ned) - samme mønster/tekst-kilde som helpPanel sitt "1 / 2 / 3 / 4"-punkt, bare brutt ut per modus.
const MODE_KEY_LABELS = { acro: "4", stabilized: "1", althold: "2", loiter: "3" };
const MODE_DESCRIPTIONS = {
    acro: "Rate-styrt, ingen selvnivellering - stikken styrer rotasjonsraten direkte.",
    stabilized: "Selvnivellerende krengning/stigning - slipp stikken og droneen retter seg selv opp. Gasspaken styrer trekkraft direkte.",
    althold: "Som Stabilized + Alt Hold - gasspak rundt 50 % holder høyden, utenfor justeres ønsket stigefart.",
    loiter: "Som Alt Hold + posisjonsholding (GPS) - slipp stikkene for å bremse opp og holde posisjonen, korrigerer selv for vind opp til ca. " + LOITER_MAX_WIND_SPEED + " m/s (varsel ved mer)."
};
const AXIS_LABELS = { roll: "Roll", pitch: "Pitch", yaw: "Yaw" };
const CHANNEL_LABELS = { roll: "Roll", pitch: "Pitch", yaw: "Yaw", throttle: "Throttle" };

const RATE_STORAGE_KEY = "ffi-uas:simulator-rates";
const GAMEPAD_STORAGE_KEY = "ffi-uas:simulator-gamepad-map";
// Har brukeren allerede fått (og lukket, med Lagre ELLER Avbryt) fjernkontroll-oppsett-veiviseren én gang
// på denne siden - se Sim.buildGamepadCalibrationWizard/maybeAutoOpen. Egen nøkkel PER simulator (ikke
// delt) - kanal-semantikken (roll/pitch/yaw her, aileron/elevator/rudder på fixed-wing/VTOL) er ulik nok
// til at "sett opp" på én side ikke bør undertrykke veiviseren på en annen.
const GAMEPAD_WIZARD_STORAGE_KEY = GAMEPAD_STORAGE_KEY + ":wizard-seen";
const SETTINGS_STORAGE_KEY = "ffi-uas:simulator-settings";

const DEFAULT_RATES = {
    roll: { centerSensitivity: 100, maxRate: 620, expo: 0.3 },
    pitch: { centerSensitivity: 100, maxRate: 620, expo: 0.3 },
    yaw: { centerSensitivity: 100, maxRate: 400, expo: 0.3 },
    throttle: { expo: 0 }
};

// TAER-rekkefølge (Throttle/Aileron/Elevator/Rudder) - standard kanalrekkefølge på TBS Crossfire/Tango-
// sendere i USB-joystick-modus. Justerbart i kalibreringspanelet for andre sendere/rekkefølger.
// scale: 1 er default (ingen justering) - satt av "Kalibrer fullt utslag" (se
// Sim.createAxisCalibrationManager) for sendere som ikke rapporterer ±1.0 ved fysisk fullt utslag.
const DEFAULT_GAMEPAD_MAP = {
    throttle: { axis: 0, reverse: false, scale: 1 },
    roll: { axis: 1, reverse: false, scale: 1 },
    pitch: { axis: 2, reverse: false, scale: 1 },
    yaw: { axis: 3, reverse: false, scale: 1 },
    buttons: { kill: null, modeAcro: null, modeStabilized: null, modeAltHold: null, modeLoiter: null, reset: null }
};
// kill er IKKE med her - den har sin egen dedikerte builder (Sim.buildGamepadKillGrid, se
// buildGamepadButtonsPanel) fordi den (i motsetning til alle disse) kan bestå av FLERE kombinerte
// brytere, se KILL_ACTION_LABEL/DEFAULT_GAMEPAD_MAP.buttons.kill.
const BUTTON_ACTION_LABELS = {
    modeAcro: "Modus: Acro", modeStabilized: "Modus: Stabilized", modeAltHold: "Modus: Alt Hold",
    modeLoiter: "Modus: Loiter", reset: "Reset (R)"
};
const KILL_ACTION_LABEL = "Kill/Arm";

const DEFAULT_WIND = { enabled: false, speed: 5, directionDeg: 0, gust: 0.3 };

const DEFAULT_SETTINGS = {
    fpvTiltDeg: DEFAULT_FPV_TILT_DEG,
    droneClass: DEFAULT_DRONE_CLASS,
    realisticMode: false,
    // Standard PÅ (brukervalg, se mixMotors-fiksen rett over) - matcher hvordan en ekte freestyle-/
    // racing-drone faktisk settes opp (aldri av for den typen flyging), og gir DEFAULT-opplevelsen full,
    // gassnivå-uavhengig rull/pitch/yaw-autoritet uansett hvor pinnen/gassen står - nærmest mulig
    // følelsen FØR selve motor-mikseren (med sin gass-avhengige metning) ble innført 31. juli.
    airmodeEnabled: true,
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
const REQUIRED_HOVER_WIND_REPS = 3; // "Hover i vind" må gjennomføres 3 ganger, samme mønster som "Returner hjem"
const HOVER_WIND_MIN_SPEED = 5; // m/s - brukeren ba eksplisitt om minst denne styrken
const HOVER_WIND_MAX_SPEED = 8; // m/s - fortsatt moderat, ikke ekstrem vind for en ren posisjonsholdings-øvelse
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
    // Ny øvelse (v3): ren posisjonsholding mot vind, uten en bane som delvis "skjuler" avdrift slik
    // åttetall i vind (ex9) gjør - komplementær ferdighet, ikke en erstatning. Nøkkelen "exHoverWind"
    // (ikke "ex10") er bevisst IKKE en omnummerering av de påfølgende øvelsenes lagrings-nøkler
    // (ex10/ex11 beholder sine - kun label-teksten deres endres til "11."/"12." under) - en
    // omnummerering av selve nøklene ville mistet allerede lagret fremgang/bestetid for dem.
    exHoverWind: {
        id: "exHoverWind",
        icon: "fa-wind",
        label: "10. Hover i vind",
        shortDescription: "Hold posisjonen i variabel vind (" + HOVER_WIND_MIN_SPEED + "-" + HOVER_WIND_MAX_SPEED +
            " m/s) - ta av, hold i " + HOVER_HOLD_SEC + " s og land, " + REQUIRED_HOVER_WIND_REPS + " ganger.",
        startHint: "Hold posisjonen i den gule indikatoren og korriger jevnlig for vinden.",
        fullDescription: "Ta av, hold posisjonen i den markerte hover-sonen på " + HOVER_ALTITUDE + " m høyde i " +
            HOVER_HOLD_SEC + " sekunder mens vinden dytter på deg, og land deretter på landingsplassen (H). " +
            "Nese-retningen sjekkes ikke her - fokuset er ren posisjonsholding mot vind, som i øvelse 9. " +
            "Vinden er " + HOVER_WIND_MIN_SPEED + "-" + HOVER_WIND_MAX_SPEED + " m/s fra en ny, tilfeldig " +
            "retning HVER runde. Du må gjennomføre dette " + REQUIRED_HOVER_WIND_REPS + " ganger for å bestå " +
            "- totaltiden for alle " + REQUIRED_HOVER_WIND_REPS + " rundene lagres i menyen.\n\nDroneklassen " +
            "settes automatisk til Middels, og du flyr fra VLOS-posisjonen. R gir nytt forsøk fra runde 1 " +
            "med nullstilt klokke.\n\nVinden settes tilbake til dine egne innstillinger når øvelsen avsluttes.",
        wind: { speed: HOVER_WIND_MIN_SPEED, directionDeg: 0, gust: 0.3 },
        randomizeWindDirection: true, // ny tilfeldig retning ved hver spawn (start/runde/R) - se spawnForExercise
        randomizeWindSpeed: { min: HOVER_WIND_MIN_SPEED, max: HOVER_WIND_MAX_SPEED }, // ny tilfeldig styrke, samme prinsipp
        stages: [{ id: "hover-wind", label: "Hover i vind", type: "hoverWind", holdSec: HOVER_HOLD_SEC }]
    },
    ex10: {
        id: "ex10",
        icon: "fa-house",
        label: "11. Returner hjem i vind",
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
        label: "12. Uforutsette hendelser",
        shortDescription: "Fly noen øvelser - underveis kan det skje noe uforutsett. 4 scenarier må bestås.",
        fullDescription: "Du skal fly noen øvelser - underveis kan det skje noe uforutsett. Riktig " +
            "reaksjon kan være å fly unna, lande, eller stoppe motorene i lufta. Det er 4 scenarier som " +
            "må bestås, ett om gangen.\n\nHusk å sette killswitchen på fjernkontrollen din i " +
            "Fjernkontroll-kalibrering.",
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
    // "1."/"2." lagt til (matcher nummereringen på det stabiliserte programmet, ex1..ex11) idet disse fire
    // ble en gradert "utsjekk" for Acro (se ACRO_EXERCISE_ORDER/acroMedalProgress) - en klar rekkefølge/
    // helhet i stedet for fire løsrevne tidsforsøk.
    race1: {
        id: "race1",
        icon: "fa-flag-checkered",
        label: "1. Racingbane - enkeltrunde",
        droneClass: "racing",
        forceCameraMode: "fpv",
        forceFlightMode: "acro",
        freeCameraToggle: true, // C (kamerabytte) er IKKE låst til VLOS her, se toggleCamera
        shortDescription: "Fly gjennom alle portene på racingbanen så fort du kan - klokken starter når du krysser start/mål.",
        startHint: "Fly gjennom porten for å starte tiden.",
        // "Racingbane enkeltrunde beskrivelse må kortes ned" (brukeren) - videre kortet ned i flere
        // oppfølgingsrunder: (1) selve strekliste-graderingen erstattet av en egen ikon-LISTE
        // (#exerciseDetailMedalRow, se medalThresholdRowHtml/showExerciseDetail - "Legg til ikoner med
        // gull, sølv og bronsje medalje/pokaler", "de medaljene må stå på liste") i stedet for tekst. (2)
        // åpningssetningen om banelayouten kuttet helt. (3) platinum-teksten flyttet til exercise.medalNote
        // (EGEN felt, IKKE en del av selve fullDescription) - vises av showExerciseDetail rett UNDER
        // ikon-listen (brukeren: "over teksten '...' - må stå OVER medaljesetningen, ikke i samme
        // avsnitt lenger, se den nye #exerciseDetailMedalNote-plasseringen i simulator.html). (4)
        // "Spawner i Racing-klasse, Acro-modus og FPV-kamera." kuttet helt (brukeren) - fra ALLE fire
        // Acro-tidsaktivitetenes fullDescription (race1/race3/raceTunnel/targetStrike), ikke bare denne.
        fullDescription: "Klokken starter automatisk idet du krysser start/mål-porten (svart/hvitt " +
            "rutemønster med en gul pil som viser flyretningen), og stopper når du har fløyet gjennom " +
            "alle de andre portene i rekkefølge og kommer tilbake til samme port. Du kan fly så mange " +
            "runder du vil - hver fullførte runde havner i ledertavlen (lagres lokalt i nettleseren), " +
            "med beste tid øverst.",
        medalNote: "Sammen med de tre andre tidsaktivitetene avgjør den dårligste medaljen din samlede " +
            "Acro-bekreftelse. Slår du 0:26.12 får du i tillegg platinum - ny rekord, ta skjermbilde og " +
            "send det til rpas@ffi.no.",
        // Ikke noTiming (ex11 sin variant) - racing har en helt egen, løpende klokke (se
        // updateExerciseHud/raceStartTime), bare vist annerledes enn de vanlige øvelsenes tidtaking.
        stages: [{ id: "race-lap", label: "Racingbane", type: "racing", lapsRequired: 1 }]
    },
    // Samme bane/spawn/porter som race1, men totaltiden for 3 SAMMENHENGENDE runder telles i stedet for
    // én runde av gangen - se lapsRequired i updateRacingStage. Egen ledertavle (racingLeaderboard.entries3,
    // ikke sammenlignbar med enkeltrunde-tidene) der hver oppføring også lagrer de tre enkeltrundetidene
    // (se finishTimedAcroRun/renderRacingLeaderboard - klikk på tiden for å se dem). Dronen resettes
    // automatisk til start/mål idet tredje runde fullføres (se updateRacingStage), i stedet for å fortsette
    // en løpende runde-etter-runde-klokke slik race1 gjør.
    race3: {
        id: "race3",
        icon: "fa-flag-checkered",
        label: "2. Racingbane - 3 runder",
        droneClass: "racing",
        forceCameraMode: "fpv",
        forceFlightMode: "acro",
        freeCameraToggle: true,
        shortDescription: "Fullfør 3 sammenhengende runder så fort du kan - totaltiden (og hver rundetid) telles.",
        startHint: "Fly gjennom porten for å starte tiden. 3 runder på rad.",
        fullDescription: "Samme racingbane som enkeltrunde-øvelsen, men her teller totaltiden for TRE " +
            "sammenhengende runder i stedet for én.\n\nBeste tid graderes til en medalje, samme system som " +
            "enkeltrunde-banen.",
        stages: [{ id: "race-3lap", label: "Racingbane - 3 runder", type: "racing", lapsRequired: 3 }]
    },
    // Punkt-til-punkt (A-til-B), IKKE en lukket løkke som de to over - se GATE_WAYPOINTS_TUNNEL/
    // buildSummitMountain for selve banen/fjellet/tunnelene, og stage.pointToPoint i updateRacingStage for
    // mål-logikken (siste port i lista, ikke tilbake til gate 0). Bygget inn i en håndbygd erstatning for
    // bakgrunnsfjellet på samme sted (IKKE et eget fjell et annet sted - se buildSummitMountain) - rett
    // gjennom foten, en spiral av porter opp den ekte fjellsiden (ekte kollisjon hele veien, ikke løse
    // porter i tomrommet), en ny liten tunnel nær toppen, og mål på selve fjelltoppen.
    raceTunnel: {
        id: "raceTunnel",
        icon: "fa-mountain",
        label: "3. Til topps",
        droneClass: "racing",
        forceCameraMode: "fpv",
        forceFlightMode: "acro",
        freeCameraToggle: true,
        shortDescription: "Banen starter nede og går til toppen av fjellet.",
        startHint: "Fly gjennom porten for å starte tiden.",
        fullDescription: "Banen starter nede og går til toppen av fjellet.\n\nKlokken starter idet du " +
            "krysser startporten og stopper idet du når målporten på toppen. Beste tid havner i en egen " +
            "ledertavle og graderes til en medalje - én av de fire tidsaktivitetene den samlede " +
            "Acro-bekreftelsen bygger på.",
        stages: [{ id: "race-tunnel", label: "Til topps", type: "racing", pointToPoint: true, assetsKey: "raceTunnel" }]
    },
    // Én sammenhengende, tidtatt økt - IKKE tre separate ledertavler (brukerens eget valg, se
    // spørsmålet/svaret bak denne funksjonen) - tre stages ("type + variant"-mønster fra ex11 sine
    // killswitch-stages) flydd på rad, alle mot samme løpende klokke. Se
    // "Øvelser: mål-i-bevegelse (targetStrike)"-seksjonen for selve bevegelses-/treff-logikken.
    targetStrike: {
        id: "targetStrike",
        icon: "fa-bullseye",
        label: "4. Krasj i bevegelige mål",
        droneClass: "racing",
        forceCameraMode: "fpv",
        forceFlightMode: "acro",
        freeCameraToggle: true,
        shortDescription: "Kollider med fire bevegelige mål - en drone, en grønn bil, en løpende person i skogen og et modellfly - så fort du kan.",
        startHint: "Klokken går allerede - finn og kollider med dronen som flyr.",
        fullDescription: "Fire bevegelige mål skal treffes på rad, mot klokken:\n" +
            "- en drone i lufta\n" +
            "- en grønn bil\n" +
            "- en løpende person i skogen\n" +
            "- et modellfly\n\n" +
            "Du må kollidere med hvert mål TO ganger før du går videre til det neste.",
        // timedLoop (IKKE skipLanding - se advanceExerciseStage): fullført idet siste mål treffes, løkker
        // rett tilbake til steg 0 for et nytt forsøk (finishTimedLoopRun) - ingen landing på H, og ingen
        // vei innom det ENGANGS bestått/ikke-bestått-systemet (completeExercise/EXERCISE_ORDER) i det hele
        // tatt, samme prinsipp som racingbanenes egen selv-resettende løkke.
        timedLoop: true,
        stages: [
            { id: "hit-drone", label: "Treff dronen", type: "targetHit", variant: "drone" },
            { id: "hit-car", label: "Treff bilen", type: "targetHit", variant: "car" },
            { id: "hit-person", label: "Treff personen i skogen", type: "targetHit", variant: "person" },
            // Fjerde mål (brukerens krav: "Legg til enda en oppgave om å krasje i en fixed wing drone...
            // 2 runder med krasj i den også") - samme TARGET_HITS_REQUIRED (2) som de tre andre, ingen
            // egen unntakslogikk trengs.
            { id: "hit-fixedwing", label: "Treff modellflyet", type: "targetHit", variant: "fixedwing" }
        ]
    }
};
const EXERCISE_ORDER = ["ex1", "ex2", "ex3", "ex4", "ex5", "ex6", "ex7", "ex8", "ex9", "exHoverWind", "ex10", "ex11"];
// Kategorisering for øvelsesmenyens undermenyer (Stabilized/Acro) - se showExerciseCategoryView. Disse
// fire er bevisst IKKE i EXERCISE_ORDER (som styrer den STABILISERTE bekreftelsen/diplomet) - de er åpne
// tidsforsøk med egne ledertavler, ikke engangs bestått/ikke-bestått-øvelser. I stedet får de sin egen,
// parallelle gradering (medalje per aktivitet, se ACRO_MEDAL_THRESHOLDS/acroMedalProgress lenger ned) og
// sitt eget Acro-diplom (openAcroDiploma) - "utsjekk" for Acro-kategorien, bygget rundt disse fire
// tidsaktivitetene i stedet for nye hover/firkant-drills (brukeren: "'øvelsene' er de racingbanene vi har
// fra før + den nye + den øvelsen med å treffe bevegelige mål").
const ACRO_EXERCISE_ORDER = ["race1", "race3", "raceTunnel", "targetStrike"];

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
    injuredTarget: null, // "pilot" | "bystander" - styrer kun bannerteksten, se updateHud
    // Loiter sitt vindkorreksjon-I-ledd (se LOITER_WIND_I_GAIN-kommentaren) - nullstilles i stepPhysics
    // hver tick simmen IKKE er i Loiter (anti-windup, samme mønster som VTOL-simmens fbwbClimbIntegral).
    loiterIntegralFwd: 0,
    loiterIntegralRight: 0,
    // Lavpassfiltrert fwd-/right-fart (se LOITER_VEL_FILTER_ALPHA) - DETTE, ikke de rå fwdSpeed/rightSpeed-
    // målingene, er hva fwdError/rightError (P- og I-leddet) OG D-leddets deriverte faktisk regnes fra. null
    // betyr "ingen gyldig verdi ennå" (akkurat gått inn i Loiter, eller nettopp forlatt den) - stepPhysics
    // tolker det som "initialiser filteret til nåværende fart i stedet for å rulle dit fra en gammel/
    // urelatert verdi (eller 0)".
    loiterVelFwdFilt: null,
    loiterVelRightFilt: null,
    // Holdepunktet (verdens-XZ) Loiter faktisk navigerer TILBAKE til når pinnen er sluppet (se
    // LOITER_POS_P_GAIN) - satt til gjeldende posisjon idet Loiter velges, og flyttet kontinuerlig med
    // droneen mens piloten aktivt flyr (pinnen utenfor LOITER_STICK_DEADBAND), se stepPhysics.
    loiterTargetPos: new THREE.Vector3(),
    // Har droneen faktisk LÅST SEG til loiterTargetPos (fase 3 - se LOITER_TARGET_LOCK_SPEED-kommentaren)?
    // Ekte, vedvarende tilstand (ikke en terskel som sjekkes på nytt hver tick) - forblir true gjennom
    // forbigående fartsøkninger fra vindkast, nullstilles kun når piloten flyr aktivt igjen (fase 1).
    loiterHolding: false,
    // Selve DEN KOMMANDERTE lenevinkelen akkurat nå (se LOITER_MAX_ANGLE_RATE) - null betyr "ingen gyldig
    // forrige verdi ennå" (samme null-mønster som loiterVelFwdFilt over), som stepPhysics tolker som
    // "hopp rett til det utregnede målet denne ticken" i stedet for å rulle dit fra en gammel/urelatert verdi.
    loiterCmdPitchAngle: null,
    loiterCmdRollAngle: null,
    // Kun til observasjon/feilsøking (js/simulator-flightlog.js) - hvilken av de tre Loiter-fasene
    // (flying/braking/holding) stepPhysics faktisk brukte SIST tick. Påvirker ingen fysikk selv.
    loiterPhase: "-"
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

// Klasse-taket (spec.maxYawRateDeg, se DRONE_CLASSES-kommentaren) på TOPP av brukerens egne rates.yaw -
// begge parter skal begrense: en Racing-pilot som setter yaw-raten lavt i Rates-panelet skal fortsatt få
// akkurat DEN lave raten, og en Cinematic-pilot som (feilaktig) setter den høyt skal likevel klemmes til
// klassens eget tak. Skalerer centerSensitivity proporsjonalt ned også når taket faktisk klipper noe, slik
// at selve KURVEFORMEN (senterfølsomhet relativt til toppfart) bevares i stedet for at bare enden hugges av.
function effectiveYawRates() {
    const capDeg = currentDroneSpec().maxYawRateDeg;
    if (rates.yaw.maxRate <= capDeg) return rates.yaw;
    const scale = capDeg / rates.yaw.maxRate;
    return { expo: rates.yaw.expo, centerSensitivity: rates.yaw.centerSensitivity * scale, maxRate: capDeg };
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
    const surfaceY = solidSurfaceHeightAt(droneState.position.x, droneState.position.z, droneState.position.y);
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
    "Digit1", "Digit2", "Digit3", "Digit4",
    "KeyK", "KeyR", "KeyC", "KeyT", "KeyH", "KeyO", "KeyM"
]);

let renderer, scene, chaseCamera, fpvCamera, vlosCamera, activeCamera;
// Gjenbrukt vektor for skyMesh-rekentrering (se animate) - unngår en ny THREE.Vector3-allokering hvert bilde.
const skyRecenterPos = new THREE.Vector3();
let skyMesh; // se updateInCloudFog - skjules mens droneen er inni en sky, ellers ville den blå himmelen skint gjennom eventuelle hull i den (nå dobbeltsidige) sky-meshen
let viewportWatcher; // se Sim.createViewportWatcher - fanger opp DPI-/vindusstørrelse-endringer ved skjermbytte som en enkelt resize-event ikke er pålitelig for
let droneGroup, dronePropellers;
let heliHandle, airplaneHandle, pedestrianHandle; // se buildHelicopter/buildAirplane/buildPedestrianGroup - kun brukt av ex11
// Egne, NYE håndtak for targetStrike (drone/bil/person) - IKKE heliHandle/pedestrianHandle over, bevisst:
// se begrunnelsen ved "Øvelser: mål-i-bevegelse"-seksjonen lenger ned (updateBystanderCollision sjekker
// eksplisitt pedestrianHandle og disarmer/skader dronen ved nærkontakt - stikk i strid med denne
// øvelsens mål, som er å faktisk kollidere).
let targetDroneHandle, targetCarHandle, targetRunnerHandle, targetFixedWingHandle;
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
   (radius 1400, se Sim.buildGradientSky-kallet i initScene) og godt innenfor kameraenes far-plane (2000).
   Faste, hånd-plasserte/deterministiske posisjoner (ingen Math.random() ved verdensbygging) - verden skal
   se lik ut mellom sideinnlastinger, akkurat som trærne/portene ellers i filen.
*/
// Færre enn før (14 -> 8) og skjøvet lenger ut mot ytterkanten av kartet (dist ~540-620, opp fra
// ~440-580) - god klaring til alt som faktisk kan flys til (racing-løypa når maks ~131 fra origo,
// "Returner hjem" spawner maks 170 unna). radius er fortsatt ~1.6x høyden for en naturtro, slak
// silhuett; dist+radius når opp mot ~730 i verste fall (se f.eks. MOUNTAIN_DEFS[0]/summit-fjellet,
// dist620+radius110) - FOR NÆRT den opprinnelige himmelkule-radiusen (800) til komfortabel margin for et
// kamera som faktisk flyr dit (brukerens rapport: "flimring/svarte hull... på himmelen på avstand" ved
// nettopp fjellene - et kamera nær/utenfor selve kule-radiusen ser BackSide-geometrien fra feil side,
// altså ingenting). Himmelkulen er derfor hevet til radius 1400 (se initScene) - god margin til alt som
// faktisk kan nås, fortsatt trygt innenfor far-plane (2000).
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
            angle: rad, seed: i * 1.7 + 1, snow: m.snow, isMain: true, mainIndex: i, familyIndex: i
        });
        (m.subPeaks || []).forEach(function (sub, si) {
            const subHeight = m.height * sub.f;
            const subRadius = m.radius * (0.5 + sub.f * 0.2);
            const subDir = rad + sub.dirOffset;
            const subDist = m.radius * sub.off;
            peaks.push({
                familyIndex: i,
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
// BUG (rapportert av brukeren: "kolliderer i løse luften nært fjellsider") - mountainHeightAt (se der)
// regner kollisjonsflaten fra den NØYAKTIGE, kontinuerlige jitter-formelen ved enhver vinkel, men denne
// meshen tegnet bare 14 radiale segmenter (~25,7° mellom hver vertex). Jitteret har to sinusledd, opp
// til sin(vinkel*11*noiseFreqMul) - med noiseFreqMul opptil 1.4 (se MOUNTAIN_DEFS) svinger det leddet
// >15 ganger rundt fjellet, LANGT mer enn 14 vertekser i det hele tatt kan gjengi (Nyquist krever minst
// 2x så mange sampler som svingninger). Mellom to nabovertekser tegnes bare en RETT strek (korde) - der
// den sanne, glatte kollisjonsflaten kollisjonen faktisk bruker buer utover forbi den korden ser
// spilleren åpen luft mens hitboksen fortsatt stikker ut dit. RADIAL_SEGMENTS økt godt forbi Nyquist-
// grensa (2*15,4≈31) med en trygg margin, slik at silhuetten faktisk gjengir samme kurve som
// mountainHeightAt regner på - ikke bare en grovere tilnærming av den, som lett kunne avvike usynlig i
// akkurat de taggete forsenkningene brukeren fløy inn i.
const MOUNTAIN_RADIAL_SEGMENTS = 48;
function buildGradientPeakGeometry(radius, height, seed, colorStops, jaggedness, topRadiusFrac, curvePower, noiseFreqMul) {
    const geo = new THREE.CylinderGeometry(radius, radius, height, MOUNTAIN_RADIAL_SEGMENTS, 7);
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
        // familyIndex 0 (MOUNTAIN_DEFS[0] - hovedtopp OG dens bi-topp) er nå bygget som en HELT egen,
        // håndbygd struktur med ekte tunneler (se buildSummitMountain, kalt fra buildGateCourseTunnel) -
        // IKKE denne generiske, jitrede kjeglemodellen. Se familyIndex-kommentaren ved MOUNTAIN_PEAKS.
        if (peak.familyIndex === 0) return;
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
// er inversen av mountainProfileRadiusFrac som selve meshen bygges med) OG selve den vinkelavhengige
// jitteren (nøyaktig samme formel som buildGradientPeakGeometry, se der), IKKE bare en jevnt oppblåst
// sikkerhetsmargin (se tidligere JITTER_SAFETY_MARGIN-forsøk, fjernet - den hindret riktignok klipping
// gjennom synlig fjellside der overflaten BULER utover (+jitter), men gjorde SAMTIDIG kollisjonen for
// stor akkurat der overflaten TREKKER seg innover (-jitter) - en usynlig vegg midt i det som ser ut som
// åpen luft ved en taggete fjellside/nedskjæring (rapportert: krasjet i løs luft nær et fjell). Ved å
// regne ut nøyaktig samme vinkelavhengige jitter her følger kollisjonen den faktiske, synlige, ujevne
// overflaten presist - verken for stor eller for liten - i stedet for en glattet tilnærming.
//
// heightFrac inngår i selve jitter-formelen (via topDamp), men er samtidig det vi egentlig løser for -
// ett fikspunkt-steg holder mer enn godt nok for et så lavfrekvent/mykt jitter: bruk den U-justerte
// profilens heightFrac til å anslå topDamp/jitter, juster distFrac med det anslaget, og les av endelig
// høyde. localAngle må regnes FØR meshets egen Y-rotasjon (se mesh.rotation.y = peak.angle i
// buildMountainRange) - ellers ville jitteret her og det man faktisk SER være forskjøvet i forhold til
// hverandre og fortsatt ikke stemme overens vinkel for vinkel.
// MOUNTAIN_TUNNEL_VOIDS/summitMountainHeightAt (definert lenger ned i filen, ved "Til topps"-banen) - se
// buildSummitMountain-kommentaren for hele resonnementet: familyIndex 0 (MOUNTAIN_DEFS[0]) er IKKE lenger
// del av denne løkken (se skip-sjekken under) - den erstattes helt av en egen, håndbygd struktur med en
// GLATT (ujitret) kollisjonsformel som er nøyaktig, matematisk identisk med dens egen (også ujitrede)
// synlige geometri - ingen tilnærming, ingen fare for usynlig kollisjon/hull (brukerens krav: "kollisjon
// som samsvarer helt med sin 3D mesh"). MOUNTAIN_TUNNEL_VOIDS er de to ekte, smale korridor-rektanglene
// boret inn i DEN strukturen - sjekket FØR alt annet, se orientedBoxLocalXZ (definert lenger ned i filen -
// trygt å kalle herfra siden mountainHeightAt sin FUNKSJONSKROPP ikke kjører før den faktisk kalles, lenge
// etter hele filen er lastet).
// atY (valgfri): høyden spørringen faktisk gjelder for. Et tunnel-hulrom (MOUNTAIN_TUNNEL_VOIDS) er KUN
// gyldig å svare "gulvhøyden der" på når man faktisk befinner seg PÅ ELLER UNDER korridorens eget tak
// (tv.ceilY, pluss en liten margin) - default (atY utelatt => Infinity, "fritt fall fra himmelen") svarer
// fortsatt korridorens gulv, siden et fall FRA UTENFOR/OVENFOR ville truffet det ekte fjelltaket først i
// virkeligheten, men INGEN annen kode her stoler på det uten en ekte referanse (se de fem kallerne).
// Uten denne sjekken ble ETHVERT punkt innenfor korridorens smale XZ-fotavtrykk lest som "gulvet der er 40
// m", UANSETT hvor høyt spørringen faktisk gjaldt - inkludert oppe på selve toppflaten (69 m), der
// påskeegget/steinene står (se buildSummitRocks), siden både korridoren OG toppflaten ligger nær
// fjellsenteret. Droneen falt dermed gjennom det den skulle lande på og ned til korridorgulvet i stedet
// (brukerens rapport: "nå glithcer dronen gjennom grunnen som personen og steinene ligger på før den
// treffer en annen grunn like under").
function mountainHeightAt(x, z, atY) {
    const ref = atY === undefined ? Infinity : atY;
    for (let v = 0; v < MOUNTAIN_TUNNEL_VOIDS.length; v++) {
        const tv = MOUNTAIN_TUNNEL_VOIDS[v];
        if (ref > tv.ceilY + 1) continue; // over korridorens eget tak - IKKE et gyldig "fall ned i hulrommet" her
        const p = orientedBoxLocalXZ(x, z, tv);
        if (Math.abs(p.lx) <= tv.halfW && Math.abs(p.lz) <= tv.halfD) return tv.floorY;
    }
    let top = summitMountainHeightAt(x, z);
    // Grov, billig avstands-avvisning FØR vinkel-/jitterberegningen: maks jitter-amplitude
    // (|0.18|+|0.10|=0.28) * maks jaggedness (1.4) * maks topDamp (1) ≈ 0.39 - god margin over dette.
    const MAX_JITTER_BULGE = 0.4;
    for (let i = 0; i < MOUNTAIN_PEAKS.length; i++) {
        const peak = MOUNTAIN_PEAKS[i];
        if (peak.familyIndex === 0) continue; // erstattet av summitMountainHeightAt over - se buildSummitMountain
        const dx = x - peak.x, dz = z - peak.z;
        const dist = Math.hypot(dx, dz);
        if (dist >= peak.radius * (1 + MAX_JITTER_BULGE)) continue;
        const rawDistFrac = dist / peak.radius;
        // Kun brukt som fikspunkt-ANSLAG for topDamp under - klippet til 1 er trygt her (se
        // MAX_JITTER_BULGE-kommentaren over: uten klipping ville et punkt godt utenfor den u-justerte
        // profilen fått en kunstig lav heightFrac0, men det er nettopp det anslaget SKAL fange opp -
        // et punkt der ute er nesten uansett nær bakken/foten uansett faktisk jitter).
        const heightFrac0 = mountainProfileHeightFrac(Math.min(rawDistFrac, 1), peak.topRadiusFrac, peak.curvePower);
        const localAngle = Math.atan2(dz, dx) - peak.angle;
        const topDamp = 1 - Math.pow(heightFrac0, 3) * 0.7; // nøyaktig samme formel som buildGradientPeakGeometry
        const jitter = Math.sin(localAngle * 5 * peak.noiseFreqMul + peak.seed * 3.1) * 0.18 +
            Math.sin(localAngle * 11 * peak.noiseFreqMul + peak.seed * 7.7) * 0.1;
        const radialJitterScale = 1 + jitter * peak.jaggedness * topDamp;
        const adjustedDistFrac = rawDistFrac / radialJitterScale;
        const h = peak.height * mountainProfileHeightFrac(adjustedDistFrac, peak.topRadiusFrac, peak.curvePower);
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
        // Seed fra treets egen (faste) posisjon - se buildRandomTree sin egen kommentar for hvorfor typen
        // (bjørk/furu) må være deterministisk i stedet for Math.random().
        const tree = Sim.buildRandomTree(t.h, t.x * 7.13 + t.z * 3.71);
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
// side: DoubleSide (BUG rapportert av brukeren: "flyr man inn i skyen så bare forsvinner skyen man er
// inni") - standard (FrontSide) tegner bare utsiden av kule-meshen; flyr kameraet INN i den, peker
// samtlige trekanter man da er omgitt av bort fra kameraet og blir bak-flate-kuttet (culling), så hele
// skyen forsvinner akkurat idet man er inni den. DoubleSide tegner innsiden også, slik at man fortsatt
// ser de myke, hvite skyveggene rundt seg. Se updateInCloudFog for selve "tåke"-effekten (redusert sikt)
// mens man faktisk er inni en - denne innsiden alene ville bare vist en tom, hul hvit ballong.
const CLOUD_SHADE_MATERIALS = [0xff, 0xeb, 0xd7, 0xc3].map(function (shade) {
    return new THREE.MeshStandardMaterial({
        color: (shade << 16) | (shade << 8) | shade, roughness: 0.9,
        transparent: true, opacity: 0.95, side: THREE.DoubleSide
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
    // Husket for updateInCloudFog sin ellipsoide "er droneen inni denne klyngen"-test - meshScale er
    // FAST (satt rett over, aldri endret senere), mens selve klyngens egen (uniforme) scale endres
    // dynamisk hvert bilde av updateClouds (skydekke-vekst) - begge trengs for å regne dagens faktiske
    // verdens-radius langs hver akse.
    group.userData.radius = radius;
    group.userData.meshScale = mesh.scale;
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

// "Tåkete inni skyen" (BUG rapportert av brukeren, se DoubleSide-kommentaren ved CLOUD_SHADE_MATERIALS
// for den andre halvparten av samme feilrapport): en ekte cumulus-sky er ikke bare et hult skall man kan
// se rett gjennom innsiden av - den er en diffus vanndråpe-tåke som stryker sikten kraftig ned uansett
// hvilken retning man ser. Testes med en grov, akse-rettet ELLIPSOIDE per klynge (samme radius/meshScale
// som selve visnings-meshen, se userData i buildCloudCluster - "kilde til sannhet" for kollisjon/tåke
// matcher det man faktisk SER, samme prinsipp som mountainHeightAt/MOUNTAIN_PEAKS). Kun én sky trenger
// å treffe (dvs. IKKE avstandssortert/nærmeste-først) - droneen er uansett aldri inni mer enn én
// klynge om gangen i praksis, klyngene overlapper ikke vesentlig.
// BUG (rapportert av brukeren: "skyene er litt gjennomsiktige fra utsiden men blir plutselig mye
// tettere når man flyr inn i dem") - forrige versjon slo fog-en helt AV/PÅ som en TERSKEL akkurat idet
// man krysset kloden sin overflate, rett fra "ingen fog i det hele tatt" til et tett near=1/far=16-
// hvitt-ut på én eneste frame - en hard, synlig "vegg" i tetthet som ikke hadde noe med den myke,
// halvgjennomsiktige overflaten man nettopp så utenfra å gjøre. Gradert nå etter PENETRASJONSDYBDE
// (0 rett ved overflaten, økende inn mot senteret - samme ellipsoide-avstand som før, bare IKKE
// klippet til en boolsk inni/utenfor) - fog-en er myk/lett akkurat idet man krysser overflaten (matcher
// hvor "litt gjennomsiktig" den så ut utenfra) og strammer seg gradvis til et tett hvitt-ut først et
// stykke lenger inn, i stedet for å hoppe dit momentant.
const CLOUD_FOG_COLOR = 0xe7e7ec; // gjennomsnittlig, litt kjølig hvit-grå - matcher CLOUD_SHADE_MATERIALS
const CLOUD_FOG_EDGE_NEAR = 20, CLOUD_FOG_EDGE_FAR = 70;  // rett innenfor overflaten - fortsatt lett disete
const CLOUD_FOG_CORE_NEAR = 1, CLOUD_FOG_CORE_FAR = 16;   // godt inne i klyngen - tett "IMC"-hvitt-ut
let insideCloud = false; // kun av/på-tilstanden for sky-skjuling/bakgrunn under, IKKE selve tettheten
function updateInCloudFog() {
    let maxPenetration = 0;
    if (settings.cloudsEnabled) {
        const p = droneState.position;
        for (let i = 0; i < cloudClusters.length; i++) {
            const cluster = cloudClusters[i];
            if (!cluster.visible) continue;
            // cluster.scale er alltid uniform (satt via setScalar i updateClouds over) - .x holder for alle tre.
            const s = cluster.userData.radius * cluster.scale.x;
            const ms = cluster.userData.meshScale;
            const dx = (p.x - cluster.position.x) / (s * ms.x);
            const dy = (p.y - cluster.position.y) / (s * ms.y);
            const dz = (p.z - cluster.position.z) / (s * ms.z);
            // 0 = akkurat på overflaten, 1 = i sentrum - se BUG-kommentaren over. Flere klynger kan i
            // prinsippet overlappe: bruker den DYPESTE (mest tåkete) av dem, ikke bare den første truffet.
            const penetration = clamp(1 - Math.sqrt(dx * dx + dy * dy + dz * dz), 0, 1);
            if (penetration > maxPenetration) maxPenetration = penetration;
        }
    }
    const inside = maxPenetration > 0;
    if (inside) {
        const near = THREE.MathUtils.lerp(CLOUD_FOG_EDGE_NEAR, CLOUD_FOG_CORE_NEAR, maxPenetration);
        const far = THREE.MathUtils.lerp(CLOUD_FOG_EDGE_FAR, CLOUD_FOG_CORE_FAR, maxPenetration);
        // Justerer near/far på et EKSISTERENDE Fog-objekt (i stedet for å lage et nytt hvert bilde) når
        // man allerede er inni - unngår at scene.fog sin referanse endres kontinuerlig for ingenting.
        if (scene.fog) { scene.fog.near = near; scene.fog.far = far; }
        else scene.fog = new THREE.Fog(CLOUD_FOG_COLOR, near, far);
    } else if (scene.fog) {
        scene.fog = null;
    }
    if (inside === insideCloud) return;
    insideCloud = inside;
    // Himmelkulen (Sim.buildGradientSky) har sin egen håndskrevne shader UTEN fog-uniformen scene.fog
    // ellers injiserer automatisk i standardmaterialer - den ville IKKE blitt hvit av fog-en over, og
    // ville skint gjennom som en synlig blå flekk overalt der (nå dobbeltsidige, men fortsatt en
    // lavpoly-kule med hull mellom bulkene) sky-meshens innside ikke akkurat dekker synsfeltet. Enklest
    // robuste fiks: bare skjul himmelen fullstendig mens man er inni en sky - man skal uansett ikke se
    // "ekte" himmel gjennom tett skydis. Dette er fortsatt en TERSKEL (av/på idet man krysser
    // overflaten, ikke gradert) - det er en synlighets-KORREKTHET, ikke en tetthets-FORNEMMELSE, så den
    // samme brå av/på-logikken som før er fortsatt riktig her.
    // scene.background settes til samme farge som fog-en (renderer/scene har ellers ingen bakgrunn satt
    // - standard clear-farge er SORT) slik at et ev. hull i den (fortsatt lavpoly) sky-meshen viser mer
    // tåke i stedet for et synlig svart glimt.
    skyMesh.visible = !inside;
    scene.background = inside ? new THREE.Color(CLOUD_FOG_COLOR) : null;
}

// Enkel bil, bygg med flatt tak og trær - prosedurale former i realistisk skala mot droneen.
// Lengdeaksen er lokal Z (IKKE X, som en tidligere versjon hadde) - samme "forover er lokal Z, bredde er
// lokal X"-konvensjon som orientTowardTravel/buildHelicopter/buildAirplane forutsetter. Målets bil
// (targetCarHandle, se targetHitHandleFor) orienteres med nettopp orientTowardTravel, som kun roterer
// OBJEKTETS EGEN akse til å peke langs kjøreretningen - da bilens lengdeakse tidligere var X i stedet for
// Z, ble hele bilen dermed alltid stilt 90° feil vei i forhold til selve bevegelsesretningen (brukerens
// rapport: "nå har bilen 90 grader stilte hjul noen ganger" - "noen ganger" fordi tilfeldig retning ved
// hver spawn, se spawnTargetHitStage, gjorde at feilen noen ganger så mer riktig ut fra kameraets vinkel
// enn andre). Den STATISKE bilen (se buildWorldObjects) bruker aldri orientTowardTravel og påvirkes ikke
// av denne konvensjonen i seg selv - dens egen rotation.y er justert +90° der for å bevare samme visuelle
// vinkel som før denne ombyggingen.
// bodyColor/cabinColor (valgfrie) - overstyrer standardfargene (rød karosseri/mørkeblå kabin) for den
// STATISKE, dekorative bilen ved avgangsplassen. targetCarHandle (målet i "Krasj i bevegelige mål") sender
// inn egne, grønne/kamuflasjefargede verdier i stedet (brukerens krav: "bilen som kjører rundt må være
// grønn/kamuflasjefarget") - se buildWorldObjects for den upåvirkede, fortsatt røde standardbilen.
function buildCar(bodyColor, cabinColor) {
    const group = new THREE.Group();
    const bodyMat = new THREE.MeshStandardMaterial({ color: bodyColor !== undefined ? bodyColor : 0xb33a3a });
    const cabinMat = new THREE.MeshStandardMaterial({ color: cabinColor !== undefined ? cabinColor : 0x223344 });
    const wheelMat = new THREE.MeshStandardMaterial({ color: 0x111111 });

    const wheelRadius = 0.32;
    const bodyHeight = 0.8;
    const bodyCenterY = wheelRadius + bodyHeight / 2;
    const body = new THREE.Mesh(new THREE.BoxGeometry(1.8, bodyHeight, 4.3), bodyMat);
    body.position.y = bodyCenterY;
    body.castShadow = true;
    body.receiveShadow = true;
    group.add(body);

    const cabinHeight = 0.55;
    const cabin = new THREE.Mesh(new THREE.BoxGeometry(1.6, cabinHeight, 2.2), cabinMat);
    cabin.position.set(0, bodyCenterY + bodyHeight / 2 + cabinHeight / 2, -0.2);
    cabin.castShadow = true;
    group.add(cabin);

    [[0.95, 1.4], [-0.95, 1.4], [0.95, -1.4], [-0.95, -1.4]].forEach(function (p) {
        const wheel = new THREE.Mesh(new THREE.CylinderGeometry(wheelRadius, wheelRadius, 0.25, 16), wheelMat);
        wheel.rotation.z = Math.PI / 2;
        wheel.position.set(p[0], wheelRadius, p[1]);
        wheel.castShadow = true;
        group.add(wheel);
    });

    return group;
}

// Enkelt, gjenkjennelig "mål-drone" for targetStrike (Krasj i bevegelige mål) - IKKE spillerdronens egen,
// mye mer kompliserte klasseavhengige modell (rebuildDroneMesh) - en frittstående, forenklet silhuett
// bygget kun som scenario-rekvisitt, samme "kun brukt her, ikke en flybar enhet"-idé som buildHelicopter.
function buildTargetDrone() {
    const group = new THREE.Group();
    const bodyMat = new THREE.MeshStandardMaterial({ color: 0x2a2a2a });
    const armMat = new THREE.MeshStandardMaterial({ color: 0x444444 });
    const bladeMat = new THREE.MeshStandardMaterial({ color: 0xdd3333, transparent: true, opacity: 0.85 });

    const body = new THREE.Mesh(new THREE.BoxGeometry(0.34, 0.12, 0.34), bodyMat);
    group.add(body);

    const armLen = 0.55;
    [[1, 1], [1, -1], [-1, 1], [-1, -1]].forEach(function (p) {
        const dir = Math.atan2(p[1], p[0]);
        const arm = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, armLen, 6), armMat);
        arm.rotation.z = Math.PI / 2;
        arm.rotation.y = -dir;
        arm.position.set(p[0] * armLen / 2, 0, p[1] * armLen / 2);
        group.add(arm);
        const blade = new THREE.Mesh(new THREE.CylinderGeometry(0.16, 0.16, 0.015, 14), bladeMat);
        blade.position.set(p[0] * armLen, 0.06, p[1] * armLen);
        group.add(blade);
    });
    // "Dronen man skal krasje i kan være litt større" (brukeren) - hele modellen skalert opp 40% i ett,
    // i stedet for å regne om hver enkelt dimensjon over - TARGET_HIT_RADIUS.drone er økt tilsvarende
    // (se der), så selve treffsonen fortsatt matcher den nå litt større, synlige modellen.
    group.scale.setScalar(1.4);
    group.visible = false;
    return group;
}

/* ---------- Fastvinge-mål ("Krasj i bevegelige mål", 4. mål) - EKTE Heewing-geometri ----------
   Brukeren, etter en første, enklere stand-in-modell: "Flyet man skal krasje i MÅ ha samme utseende som
   heewing fra VTOL simmen. tilpass fysikken da hvis det er vanskelig." Dette er en DIREKTE, verbatim
   PORTERING av selve geometribyggingen fra js/simulator-vtol.js sin buildHeewingPlane() (+ hjelpefunksjonene
   den bruker: buildHeewingWing/buildWingTaperLoft/buildRoundedFuselageSegment/unitRoundedRectPoints/
   buildPropBlade/buildHeewingFoamTexture) - IKKE en cross-file import/deling av selve VTOL-filen (som
   fortsatt er live, ferdig kalibrert flysimulator-kode ingen tester kan verifisere uendret oppførsel for
   her) - en lokal, selvstendig kopi i quad-simulatoren i stedet, risikofri for VTOL-siden.
   "Tilpass fysikken" (brukerens eget forbehold): buildHeewingPlane() sin geometri er i seg selv REN
   VISUELL bygging (tar kun inn wingSpan/wingArea/visualScale som tall) - selve VTOL-FYSIKKEN (aerodynamikk,
   bakkekontaktpunkter, tiltbare nacellers hover-vinkel) er en HELT SEPARAT del av VTOL-filen som ALDRI
   kalles herfra. Det eneste som faktisk er "tilpasset": ingen animert tilt/ror-utslag (target-flyet flyr
   alltid i fastvinget cruise-stilling, nacellene tiltGroup.rotation.x=0 - se buildTargetFixedWing) og ingen
   ekte bakkekontakt (målet berører aldri bakken, samme "rene, ukolliderende visuelle prop"-prinsipp som de
   tre andre målene, se targetStrike-seksjonens "HELT NYE håndtak"-kommentar).
   HEEWING_TARGET_SPEC (wingArea/wingSpan/visualScale) er kopiert direkte fra VTOL_CLASSES.heewing (samme
   fil) - de tre eneste tallene selve geometribyggingen faktisk trenger. */
const HW_FUSELAGE_LENGTH_BUILD = 1.35, HW_CABIN_RADIUS_BUILD = 0.07;
const HW_NOSE_LEN_RATIO = 0.18, HW_CABIN_LEN_RATIO = 0.32, HW_TAIL_LEN_RATIO = 0.33;
const HW_WING_MOUNT_HEIGHT_RATIO = 1.05, HW_WING_THICKNESS_RATIO = 0.1;
const HW_GEAR_BOOM_X_FRAC = 0.22, HW_BOOM_CENTER_Z_BUILD = 0.02;
const HEEWING_TARGET_SPEC = { wingArea: 0.27, wingSpan: 1.2, visualScale: 0.75 };

let hwFoamTextureBase = null;
function buildHwFoamTexture() {
    if (hwFoamTextureBase) return hwFoamTextureBase;
    const texW = 96, texH = 96;
    const canvas = document.createElement("canvas");
    canvas.width = texW;
    canvas.height = texH;
    const ctx = canvas.getContext("2d");
    ctx.fillStyle = "#828288";
    ctx.fillRect(0, 0, texW, texH);
    const grainCount = 2600;
    for (let i = 0; i < grainCount; i++) {
        const gx = Math.random() * texW, gy = Math.random() * texH;
        const lighter = Math.random() < 0.5;
        const shade = lighter ? (150 + Math.random() * 45) : (95 + Math.random() * 35);
        const alpha = 0.10 + Math.random() * 0.22;
        ctx.fillStyle = "rgba(" + shade + "," + shade + "," + (shade + 2) + "," + alpha + ")";
        ctx.beginPath();
        ctx.arc(gx, gy, 0.4 + Math.random() * 0.7, 0, Math.PI * 2);
        ctx.fill();
    }
    hwFoamTextureBase = new THREE.CanvasTexture(canvas);
    hwFoamTextureBase.wrapS = THREE.RepeatWrapping;
    hwFoamTextureBase.wrapT = THREE.RepeatWrapping;
    hwFoamTextureBase.repeat.set(6, 6);
    return hwFoamTextureBase;
}

// Enhets-omriss for ett "avrundet rektangel"-tverrsnitt (cornerFrac=1 -> ellipse/sirkel, nær 0 -> skarpt
// firkantet) - se buildHwRoundedFuselageSegment.
function hwUnitRoundedRectPoints(cornerFrac, segsPerCorner) {
    const cr = clamp(cornerFrac, 0.02, 0.98);
    const inner = 1 - cr;
    const corners = [
        { cx: inner, cy: inner, a0: 0 },
        { cx: -inner, cy: inner, a0: Math.PI / 2 },
        { cx: -inner, cy: -inner, a0: Math.PI },
        { cx: inner, cy: -inner, a0: 1.5 * Math.PI }
    ];
    const pts = [];
    corners.forEach(function (c) {
        for (let i = 0; i <= segsPerCorner; i++) {
            const a = c.a0 + (i / segsPerCorner) * (Math.PI / 2);
            pts.push({ x: c.cx + Math.cos(a) * cr, y: c.cy + Math.sin(a) * cr });
        }
    });
    return pts;
}
// Ett "loftet" avrundet-rektangel skrog-segment (front-/bakring med ulik størrelse/cornerFrac, forbundet
// med sideflater) - håndbygget BufferGeometry, ingen ferdig THREE.js-primitiv dekker formen.
function buildHwRoundedFuselageSegment(halfWFront, halfHFront, halfWBack, halfHBack, length, cornerFracFront, cornerFracBack, mat) {
    const segsPerCorner = 4;
    const frontPts = hwUnitRoundedRectPoints(cornerFracFront, segsPerCorner);
    const backPts = hwUnitRoundedRectPoints(cornerFracBack, segsPerCorner);
    const n = frontPts.length;
    const positions = [];
    for (let i = 0; i < n; i++) positions.push(frontPts[i].x * halfWFront, frontPts[i].y * halfHFront, -length / 2);
    for (let i = 0; i < n; i++) positions.push(backPts[i].x * halfWBack, backPts[i].y * halfHBack, length / 2);
    const indices = [];
    for (let i = 0; i < n; i++) {
        const a = i, b = (i + 1) % n, aBack = n + i, bBack = n + ((i + 1) % n);
        indices.push(a, aBack, b);
        indices.push(b, aBack, bBack);
    }
    const frontCenterIdx = positions.length / 3;
    positions.push(0, 0, -length / 2);
    for (let i = 0; i < n; i++) indices.push(frontCenterIdx, (i + 1) % n, i);
    const backCenterIdx = positions.length / 3;
    positions.push(0, 0, length / 2);
    for (let i = 0; i < n; i++) indices.push(backCenterIdx, n + i, n + ((i + 1) % n));
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    geo.setIndex(indices);
    geo.computeVertexNormals();
    return new THREE.Mesh(geo, mat);
}
// Tapret vinge-/hale-loft (NACA-aktig tverrsnitt via halfThickness) - kontinuerlig lineær interpolasjon
// av korde OG forkant-posisjon mellom rot og tupp i én sammenhengende geometri (ikke fasetterte segmenter).
function buildHwWingTaperLoft(rootChord, rootLEz, tipChord, tipLEz, halfSpan, spanFrac0, spanFrac1, xStart, xEnd, thicknessRatio, mat, flatBottom) {
    const SAMPLES = 10;
    function halfThickness(chord, x) {
        return thicknessRatio * chord * 5 * (0.2969 * Math.sqrt(x) - 0.126 * x - 0.3516 * x * x + 0.2843 * x * x * x - 0.1015 * x * x * x * x);
    }
    function upperY(chord, x) { return halfThickness(chord, x) * (flatBottom !== false ? 1.15 : 1); }
    function lowerY(chord, x) {
        if (flatBottom === false) return -halfThickness(chord, x);
        const noseBlend = Math.max(0, 1 - x / 0.15);
        const eased = noseBlend * noseBlend * (3 - 2 * noseBlend);
        return -halfThickness(chord, x) * eased;
    }
    function ringPoints(spanFrac) {
        const chord = THREE.MathUtils.lerp(rootChord, tipChord, spanFrac);
        const LEz = THREE.MathUtils.lerp(rootLEz, tipLEz, spanFrac);
        const upper = [], lower = [];
        for (let i = 0; i <= SAMPLES; i++) {
            const t = Math.pow(i / SAMPLES, 1.5);
            const x = xStart + (xEnd - xStart) * t;
            const z = LEz + x * chord;
            upper.push({ y: upperY(chord, x), z: z });
            lower.push({ y: lowerY(chord, x), z: z });
        }
        return upper.concat(lower.slice().reverse());
    }
    const ring0 = ringPoints(spanFrac0), ring1 = ringPoints(spanFrac1);
    const n = ring0.length;
    const x0 = spanFrac0 * halfSpan, x1 = spanFrac1 * halfSpan;
    const positions = [];
    for (let i = 0; i < n; i++) positions.push(x0, ring0[i].y, ring0[i].z);
    for (let i = 0; i < n; i++) positions.push(x1, ring1[i].y, ring1[i].z);
    const indices = [];
    for (let i = 0; i < n; i++) {
        const a = i, b = (i + 1) % n, aT = n + i, bT = n + ((i + 1) % n);
        indices.push(a, b, aT);
        indices.push(b, bT, aT);
    }
    const midIdx = Math.floor(n / 4);
    const c0 = positions.length / 3;
    positions.push(x0, 0, (ring0[0].z + ring0[midIdx].z) / 2);
    for (let i = 0; i < n; i++) indices.push(c0, i, (i + 1) % n);
    const c1 = positions.length / 3;
    positions.push(x1, 0, (ring1[0].z + ring1[midIdx].z) / 2);
    for (let i = 0; i < n; i++) indices.push(c1, n + ((i + 1) % n), n + i);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    geo.setIndex(indices);
    geo.computeVertexNormals();
    const mesh = new THREE.Mesh(geo, mat);
    mesh.castShadow = true;
    return mesh;
}
// Heewing T2 Cruza sin egen, taprede vinge (rot-/tuppkorde + forkant-/bakkant-sveip hentet fra ekte
// STL-mål, se den opprinnelige kommentaren i simulator-vtol.js) - inkl. innfelte balanseror.
function buildHwWing(spec, wingMat, darkMat) {
    const group = new THREE.Group();
    const wingChordAvg = spec.wingArea / spec.wingSpan;
    const rootChord = wingChordAvg * 1.2;
    const tipChord = wingChordAvg * 0.8;
    const rootLEz = -rootChord / 2;
    const LE_SWEEP_AFT = rootChord * 0.104;
    const tipLEz = rootLEz + LE_SWEEP_AFT;
    const aileronSpan = spec.wingSpan * 0.28;
    const AILERON_CHORD_FRAC = 0.72;
    const halfSpan = spec.wingSpan / 2;
    const centerSpanFrac = 1 - aileronSpan / halfSpan;

    [-1, 1].forEach(function (side) {
        const wingInner = buildHwWingTaperLoft(rootChord, rootLEz, tipChord, tipLEz, halfSpan, 0, centerSpanFrac, 0, 1, HW_WING_THICKNESS_RATIO, wingMat, true);
        if (side < 0) wingInner.scale.x = -1;
        group.add(wingInner);

        const fixedFront = buildHwWingTaperLoft(rootChord, rootLEz, tipChord, tipLEz, halfSpan, centerSpanFrac, 1, 0, AILERON_CHORD_FRAC, HW_WING_THICKNESS_RATIO, wingMat, true);
        if (side < 0) fixedFront.scale.x = -1;
        group.add(fixedFront);

        const AILERON_SPAN_FRAC = 0.7;
        const aileronMarginFrac = (1 - AILERON_SPAN_FRAC) / 2 * (1 - centerSpanFrac);
        const aileronInnerFrac = centerSpanFrac + aileronMarginFrac;
        const aileronOuterFrac = 1 - aileronMarginFrac;
        const aileronMidFrac = (aileronInnerFrac + aileronOuterFrac) / 2;
        [0, 1].forEach(function (edge) {
            const frame = buildHwWingTaperLoft(
                rootChord, rootLEz, tipChord, tipLEz, halfSpan,
                edge ? 1 - aileronMarginFrac : centerSpanFrac,
                edge ? 1 : centerSpanFrac + aileronMarginFrac,
                AILERON_CHORD_FRAC, 1, HW_WING_THICKNESS_RATIO, wingMat, true
            );
            if (side < 0) frame.scale.x = -1;
            group.add(frame);
        });

        const aileronPivot = new THREE.Group();
        const aileronHingeZAbs = THREE.MathUtils.lerp(rootLEz, tipLEz, aileronMidFrac) + AILERON_CHORD_FRAC * THREE.MathUtils.lerp(rootChord, tipChord, aileronMidFrac);
        aileronPivot.position.set(0, 0, aileronHingeZAbs);
        if (side < 0) aileronPivot.scale.x = -1;
        const aileronMesh = buildHwWingTaperLoft(
            rootChord, rootLEz - aileronHingeZAbs, tipChord, tipLEz - aileronHingeZAbs,
            halfSpan, aileronInnerFrac, aileronOuterFrac, AILERON_CHORD_FRAC, 1, HW_WING_THICKNESS_RATIO, darkMat, true
        );
        aileronPivot.add(aileronMesh);
        group.add(aileronPivot);
        group.userData["aileron" + side] = aileronPivot;

        const navLight = new THREE.Mesh(new THREE.SphereGeometry(tipChord * HW_WING_THICKNESS_RATIO * 0.7, 8, 6),
            new THREE.MeshStandardMaterial({
                color: side < 0 ? 0xff2a2a : 0x2aff5a,
                emissive: side < 0 ? 0xff2a2a : 0x2aff5a,
                emissiveIntensity: 0.6
            }));
        navLight.position.set(side * (halfSpan - 0.02), 0, tipLEz);
        group.add(navLight);
    });

    return group;
}
// Ett propellblad med tapret silhuett (bredest nær navet, avsmalnende mot en avrundet tupp).
function buildHwPropBlade(length, rootChord, tipChord, thickness, mat) {
    const shape = new THREE.Shape();
    const tipTaperStart = length * 0.78;
    shape.moveTo(0, -rootChord / 2);
    shape.quadraticCurveTo(length * 0.4, -rootChord * 0.5, tipTaperStart, -tipChord * 0.6);
    shape.quadraticCurveTo(length, -tipChord * 0.25, length, 0);
    shape.quadraticCurveTo(length, tipChord * 0.25, tipTaperStart, tipChord * 0.6);
    shape.quadraticCurveTo(length * 0.4, rootChord * 0.5, 0, rootChord / 2);
    shape.closePath();
    const geo = new THREE.ExtrudeGeometry(shape, { depth: thickness, bevelEnabled: false, curveSegments: 6 });
    geo.translate(0, 0, -thickness / 2);
    const mesh = new THREE.Mesh(geo, mat);
    mesh.castShadow = true;
    return mesh;
}
// Selve flykroppen: avrundet, konisk skrog -> høyvinge -> opp-ned T-hale -> to fremre traktormotorer +
// én bakre, fast vertikal motor på halebommen. Ren geometribygging (se toppkommentaren for grensene mot
// den ekte VTOL-fysikken denne IKKE tar med).
function buildHwPlane(spec) {
    const group = new THREE.Group();
    const foamTex = buildHwFoamTexture();
    const bodyMat = new THREE.MeshStandardMaterial({ color: 0x8c8c92, roughness: 0.92, map: foamTex, side: THREE.DoubleSide });
    const wingMat = new THREE.MeshStandardMaterial({ color: 0x8c8c92, roughness: 0.92, map: foamTex, side: THREE.DoubleSide });
    const darkMat = new THREE.MeshStandardMaterial({ color: 0x2a2a2d, roughness: 0.7, side: THREE.DoubleSide });
    const propMat = new THREE.MeshStandardMaterial({ color: 0x141414, roughness: 0.6 });

    const fuselageLength = HW_FUSELAGE_LENGTH_BUILD, cabinRadius = HW_CABIN_RADIUS_BUILD;
    const noseLen = fuselageLength * HW_NOSE_LEN_RATIO, cabinLen = fuselageLength * HW_CABIN_LEN_RATIO, tailLen = fuselageLength * HW_TAIL_LEN_RATIO;

    const CABIN_CORNER_FRAC = 0.4;
    const noseTipRadius = cabinRadius * 0.8;
    const cabinFrontHalfW = cabinRadius * 1.7, cabinFrontHalfH = cabinRadius * 0.9;
    const cabinRearHalfW = cabinRadius * 1.35, cabinRearHalfH = cabinRadius * 0.75;
    // Nese mot LOKAL -Z (samme konvensjon som orientTowardTravel forventer - se buildTargetFixedWing).
    const noseSection = buildHwRoundedFuselageSegment(noseTipRadius, noseTipRadius, cabinFrontHalfW, cabinFrontHalfH, noseLen, 1, CABIN_CORNER_FRAC, bodyMat);
    noseSection.position.z = -(cabinLen / 2 + noseLen / 2);
    noseSection.castShadow = true;
    group.add(noseSection);
    const noseTip = new THREE.Mesh(new THREE.SphereGeometry(noseTipRadius, 14, 10), bodyMat);
    noseTip.position.z = -(cabinLen / 2 + noseLen);
    noseTip.castShadow = true;
    group.add(noseTip);

    const cabinSection = buildHwRoundedFuselageSegment(cabinFrontHalfW, cabinFrontHalfH, cabinRearHalfW, cabinRearHalfH, cabinLen, CABIN_CORNER_FRAC, CABIN_CORNER_FRAC, bodyMat);
    cabinSection.castShadow = true;
    group.add(cabinSection);

    const shoulderLen = tailLen * 0.18;
    const tailTipZ = cabinLen / 2 + tailLen;
    const boomRadius = cabinRadius * 0.22;
    const shoulderSection = buildHwRoundedFuselageSegment(cabinRearHalfW, cabinRearHalfH, boomRadius, boomRadius, shoulderLen, CABIN_CORNER_FRAC, 1, bodyMat);
    shoulderSection.position.z = cabinLen / 2 + shoulderLen / 2;
    shoulderSection.castShadow = true;
    group.add(shoulderSection);

    const BOOM_TAIL_OVERLAP = boomRadius * 1.5;
    const boomLen = (tailLen - shoulderLen) + BOOM_TAIL_OVERLAP;
    const boom = new THREE.Mesh(new THREE.CylinderGeometry(boomRadius * 0.9, boomRadius, boomLen, 10), darkMat);
    boom.rotation.x = Math.PI / 2;
    boom.position.z = cabinLen / 2 + shoulderLen + boomLen / 2;
    boom.castShadow = true;
    group.add(boom);

    const buildWingSpan = spec.wingSpan / spec.visualScale;
    const buildWingArea = spec.wingArea / (spec.visualScale * spec.visualScale);
    const wingChord = buildWingArea / buildWingSpan;
    const wingMountYVisual = (cabinFrontHalfH + cabinRearHalfH) / 2 * 0.9;

    const wing = buildHwWing({ wingArea: buildWingArea, wingSpan: buildWingSpan }, wingMat, darkMat);
    wing.position.set(0, wingMountYVisual, 0.02);
    group.add(wing);

    const tailChord = wingChord * 0.6;
    const TAIL_SURFACE_THICKNESS_RATIO = 0.08;
    const finHeight = cabinRadius * 2.6, finChord = tailChord * 0.6, rudderChord = tailChord * 0.45;
    const finCombinedChord = finChord + rudderChord;
    const finBaseY = boomRadius * 0.8;

    const FIN_RAKE_AFT = finCombinedChord * 0.55;
    const finRootLEz = tailTipZ, finTipLEz = finRootLEz + FIN_RAKE_AFT;
    function finLEzAtHeightFrac(f) { return THREE.MathUtils.lerp(finRootLEz, finTipLEz, f); }

    const FIN_PIVOT_FRAC = 0.28;
    const finPivotRefLEz = finLEzAtHeightFrac(0.5);
    const finPivotZAbs = finPivotRefLEz + FIN_PIVOT_FRAC * finCombinedChord;
    const finPivot = new THREE.Group();
    finPivot.position.set(0, finBaseY, finPivotZAbs);
    group.add(finPivot);
    const finMesh = buildHwWingTaperLoft(
        finCombinedChord, finRootLEz - finPivotZAbs, finCombinedChord, finTipLEz - finPivotZAbs,
        finHeight, 0, 1, 0, 1, TAIL_SURFACE_THICKNESS_RATIO, darkMat, false
    );
    finMesh.rotation.z = Math.PI / 2;
    finPivot.add(finMesh);

    const stabSpan = buildWingSpan * 0.22, stabChord = tailChord * 0.65, elevatorChord = tailChord * 0.4;
    const stabCombinedChord = stabChord + elevatorChord;
    const stabMainFrac = stabChord / stabCombinedChord;
    const stabY = finBaseY;
    const STAB_TIP_CHORD_RATIO = 0.10 / 0.13;
    const stabRootChord = stabCombinedChord;
    const stabTipChord = stabCombinedChord * STAB_TIP_CHORD_RATIO;
    const stabHalfSpan = stabSpan / 2;
    const stabSweepAft = stabRootChord - stabTipChord;
    const stabRootLEzAbs = tailTipZ, stabTipLEzAbs = tailTipZ + stabSweepAft;

    const ELEVATOR_SPAN_FRAC = 0.72;
    const elevatorPivot = new THREE.Group();
    elevatorPivot.position.set(0, stabY, tailTipZ + stabChord);
    group.add(elevatorPivot);

    [-1, 1].forEach(function (side) {
        const fixedFront = buildHwWingTaperLoft(
            stabRootChord, stabRootLEzAbs, stabTipChord, stabTipLEzAbs,
            stabHalfSpan, 0, 1, 0, stabMainFrac, TAIL_SURFACE_THICKNESS_RATIO, wingMat, false
        );
        fixedFront.position.y = stabY;
        if (side < 0) fixedFront.scale.x = -1;
        group.add(fixedFront);

        const frame = buildHwWingTaperLoft(
            stabRootChord, stabRootLEzAbs, stabTipChord, stabTipLEzAbs,
            stabHalfSpan, ELEVATOR_SPAN_FRAC, 1, stabMainFrac, 1, TAIL_SURFACE_THICKNESS_RATIO, wingMat, false
        );
        frame.position.y = stabY;
        if (side < 0) frame.scale.x = -1;
        group.add(frame);

        const elevatorMesh = buildHwWingTaperLoft(
            stabRootChord, stabRootLEzAbs - elevatorPivot.position.z,
            stabTipChord, stabTipLEzAbs - elevatorPivot.position.z,
            stabHalfSpan, 0, ELEVATOR_SPAN_FRAC, stabMainFrac, 1, TAIL_SURFACE_THICKNESS_RATIO, darkMat, false
        );
        if (side < 0) elevatorMesh.scale.x = -1;
        elevatorPivot.add(elevatorMesh);
    });

    const rearMotorZ = cabinLen / 2 + tailLen * 0.72;
    const rearMotorPod = new THREE.Mesh(new THREE.CylinderGeometry(cabinRadius * 0.3, cabinRadius * 0.34, 0.05, 10), darkMat);
    rearMotorPod.position.set(0, boomRadius + 0.02, rearMotorZ);
    rearMotorPod.castShadow = true;
    group.add(rearMotorPod);
    const rearLiftProp = new THREE.Group();
    rearLiftProp.position.set(0, boomRadius + 0.045, rearMotorZ);
    const rearBladeLen = 0.15;
    [-1, 1].forEach(function (dir) {
        const blade = buildHwPropBlade(rearBladeLen, rearBladeLen * 0.22, rearBladeLen * 0.1, 0.006, propMat);
        blade.rotation.x = Math.PI / 2;
        blade.rotation.y = dir > 0 ? 0 : Math.PI;
        rearLiftProp.add(blade);
    });
    group.add(rearLiftProp);

    const boomX = buildWingSpan * HW_GEAR_BOOM_X_FRAC;
    const armMat = new THREE.MeshStandardMaterial({ color: 0x1a1a1a });
    const propGroups = [rearLiftProp];
    [-1, 1].forEach(function (side) {
        const boomXPos = side * boomX;
        const nacelleY = wingMountYVisual, nacelleZ = HW_BOOM_CENTER_Z_BUILD - wingChord * 0.6;
        const pylonTop = new THREE.Vector3(boomXPos, wingMountYVisual, HW_BOOM_CENTER_Z_BUILD);
        const pylonBottom = new THREE.Vector3(boomXPos, nacelleY, nacelleZ);
        const pylonVec = pylonBottom.clone().sub(pylonTop);
        const pylon = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.014, pylonVec.length(), 8), armMat);
        pylon.position.copy(pylonTop).addScaledVector(pylonVec, 0.5);
        pylon.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), pylonVec.clone().normalize());
        pylon.castShadow = true;
        group.add(pylon);

        // tiltGroup i fast, vannrett fastvinget cruise-stilling (rotation.x=0) - target-flyet animerer
        // ALDRI tilt mot hover (se toppkommentaren: "ingen animert tilt/ror-utslag").
        const nacelleGroup = new THREE.Group();
        nacelleGroup.position.set(boomXPos, nacelleY, nacelleZ);
        const nacelleBody = new THREE.Mesh(new THREE.CylinderGeometry(cabinRadius * 0.32, cabinRadius * 0.32, cabinRadius * 1.6, 10), darkMat);
        nacelleBody.rotation.x = Math.PI / 2;
        nacelleBody.castShadow = true;
        nacelleGroup.add(nacelleBody);
        const propGroup = new THREE.Group();
        propGroup.position.z = -cabinRadius * 1.0;
        const bladeLen = 0.16;
        [-1, 1].forEach(function (dir) {
            const blade = buildHwPropBlade(bladeLen, bladeLen * 0.22, bladeLen * 0.1, 0.006, propMat);
            blade.rotation.z = dir > 0 ? 0 : Math.PI;
            propGroup.add(blade);
        });
        nacelleGroup.add(propGroup);
        group.add(nacelleGroup);
        propGroups.push(propGroup);
    });

    group.scale.setScalar(spec.visualScale);
    group.userData.propGroups = propGroups; // spinnes i updateTargetHitVisuals - ren kosmetikk
    return group;
}
// Selve target-håndtaket - tynn wrapper rundt buildHwPlane(HEEWING_TARGET_SPEC), samme "usynlig til
// spawnTargetHitStage viser det"-mønster som de tre andre målene.
function buildTargetFixedWing() {
    const group = buildHwPlane(HEEWING_TARGET_SPEC);
    group.visible = false;
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
// crowdMembers - hvert medlems EGEN THREE.Group beholdes her (i stedet for å bare kastes inn i den
// samlede folkemengde-gruppen og glemmes), slik at updateBystanderCollision/knockPersonOver senere kan
// velte NØYAKTIG den personen som faktisk ble truffet - samme mønster som VTOL-simulatorens
// vtolCrowdMembers (js/simulator-vtol.js), gjenbrukt her (brukerens krav: "personen må falle over i
// naturlig retning. som på VTOL simulatoren").
let crowdMembers = [];
function buildCrowd() {
    crowdMembers = [];
    const group = new THREE.Group();
    CROWD_SHIRT_COLORS.forEach(function (color, i) {
        const person = Sim.buildPersonFigure({ vestColor: color });
        const off = CROWD_MEMBER_OFFSETS[i];
        person.position.set(off.x, 0, off.z);
        person.rotation.y = (Math.sin(i * 5.1) * 0.5 + 0.5) * Math.PI * 2;
        group.add(person);
        crowdMembers.push(person);
    });
    return group;
}
// "Krasjer man i en person eller en i publikum må personen falle over i naturlig retning. som på VTOL
// simulatoren" - samme knockPersonOver/updatePersonFalls/resetPersonFalls-mønster som
// js/simulator-vtol.js (se der for den opprinnelige versjonen/kommentaren) - portert hit uendret.
// Tilfeldig akse (x/z) og fortegn per fall - ingen grunn til at alle skal falle nøyaktig samme vei.
// Roterer figuren rundt sin EGEN base (0,0,0 lokalt - Sim.buildPersonFigure plasserer alle kroppsdeler
// relativt til føttene på bakken), altså et ekte "falle over ende"-velt, ikke en forskyvning i rommet.
// Brukt for BÅDE folkemengden (buildCrowd), ex11 sin fotgjenger (pedestrianHandle) OG målpersonen i
// "Krasj i bevegelige mål" (targetRunnerHandle, se updateTargetHitStage) - de tre eneste synlige,
// stående personfigurene i quad-simulatoren en drone faktisk kan kollidere med (VLOS-piloten,
// PILOT_POSITION, har ingen egen synlig figur her, i motsetning til VTOL-simulatorens vlosPersonGroup -
// ingenting å velte der).
const PERSON_FALL_SEC = 0.4;
function knockPersonOver(group) {
    if (!group || group.userData.fallen) return;
    group.userData.fallen = true;
    group.userData.fallAxis = Math.random() < 0.5 ? "x" : "z";
    group.userData.fallSign = Math.random() < 0.5 ? 1 : -1;
    group.userData.fallProgress = 0;
}
// Kalt UBETINGET hvert bilde fra animate() (samme mønster som VTOL-simulatoren) - ikke bare mens en
// bestemt øvelse er aktiv, siden folkemengden alltid er til stede i verden.
function updatePersonFalls(dt) {
    const groups = crowdMembers.concat(
        pedestrianHandle ? [pedestrianHandle] : [],
        targetRunnerHandle ? [targetRunnerHandle] : []
    );
    groups.forEach(function (g) {
        if (!g.userData.fallen || g.userData.fallProgress >= 1) return;
        g.userData.fallProgress = Math.min(1, g.userData.fallProgress + dt / PERSON_FALL_SEC);
        const angle = g.userData.fallProgress * (Math.PI / 2) * g.userData.fallSign;
        if (g.userData.fallAxis === "x") g.rotation.x = angle; else g.rotation.z = angle;
    });
}
// Reiser alle falne personer opp igjen - kalt fra resetDrone() (se der), samme "reis dem opp igjen ved
// reset"-prinsipp som VTOL-simulatorens resetPersonFalls (kalt fra resetPlane).
function resetPersonFalls() {
    const groups = crowdMembers.concat(
        pedestrianHandle ? [pedestrianHandle] : [],
        targetRunnerHandle ? [targetRunnerHandle] : []
    );
    groups.forEach(function (g) {
        g.userData.fallen = false;
        g.userData.fallProgress = 0;
        g.rotation.x = 0;
        g.rotation.z = 0;
    });
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
// vestColors (valgfritt par [venstre, høyre]) - overstyrer standardfargene (blå/rød, godt synlige som
// "publikum i fare"-farger for killswitch-øvelsen, se pedestrianHandle).
// IKKE lenger brukt for targetRunnerHandle ("Krasj i bevegelige mål") - den ga FEILAKTIG to personer i
// skogen i stedet for én, se buildTargetRunner rett under.
function buildPedestrianGroup(vestColors) {
    const group = new THREE.Group();
    const colors = vestColors || [0x3f6fb0, 0xb0473f];
    [-1, 1].forEach(function (side, i) {
        const person = Sim.buildPersonFigure({ vestColor: colors[i] });
        person.position.x = side * 0.9;
        group.add(person);
    });
    group.visible = false;
    return group;
}
// ÉN løpende person, målet i "Krasj i bevegelige mål" - EGEN builder (IKKE buildPedestrianGroup over, som
// bygger et PAR side ved side - riktig for killswitch sitt "to fotgjengere i fare"-scenario, men ga
// feilaktig TO personer i skogen her, brukerens rapport: "Personen i skogen er nå to personer. Det skal
// bare være en."). IKKE lenger en ren Sim.buildPersonFigure-wrapper - den figuren har beina bygget som
// FASTE, ikke-pivoterte bokser (rett i hovedgruppen, ingen hofteledd), så de kan ikke svinges. Bena/armene
// her er i stedet EGNE, pivoterte undergrupper (samme proporsjoner/farger som Sim.buildPersonFigure, for
// et konsistent utseende) festet i hofte-/skulderhøyde, slik at updateTargetHitVisuals kan svinge dem for
// en løpende bevegelse (brukerens krav: "personen i skogen kan bevege seg litt mer naturlig gjerne med
// beveglser i bena") - se group.userData.legs/arms. Selve "faller over ende ved treff"-animasjonen
// (updateTargetHitVisuals/updateTargetHitStage) roterer fortsatt HELE gruppen samlet (rotation.x/z), helt
// uavhengig av løpe-svingingen på de enkelte lemmene (multipliseres sammen via Object3D-hierarkiet).
function buildTargetRunner(vestColor) {
    const group = new THREE.Group();
    const skinMat = new THREE.MeshStandardMaterial({ color: 0xe0b088 });
    const vestMat = new THREE.MeshStandardMaterial({ color: vestColor });
    const pantsMat = new THREE.MeshStandardMaterial({ color: 0x2a3a4a });
    const shoeMat = new THREE.MeshStandardMaterial({ color: 0x1a1a1a });
    const legHeight = 0.75, torsoHeight = 0.5, headRadius = 0.11;

    // Bena - hofteledd (pivot-gruppe) plassert ved TOPPEN av benet (legHeight, samme høyde som
    // Sim.buildPersonFigure sin faste beinboks starter fra), selve benet/skoen henger NED fra pivoten
    // (lokal Y negativ) slik at en rotasjon om pivoten svinger benet fremover/bakover fra hoften, ikke
    // fra bakken.
    const legs = [];
    [-1, 1].forEach(function (side) {
        const hip = new THREE.Group();
        hip.position.set(side * 0.09, legHeight, 0);
        const leg = new THREE.Mesh(new THREE.BoxGeometry(0.13, legHeight, 0.15), pantsMat);
        leg.position.y = -legHeight / 2;
        leg.castShadow = true;
        hip.add(leg);
        const shoe = new THREE.Mesh(new THREE.BoxGeometry(0.14, 0.06, 0.22), shoeMat);
        shoe.position.set(0, -legHeight + 0.03, 0.03);
        shoe.castShadow = true;
        hip.add(shoe);
        group.add(hip);
        legs.push(hip);
    });

    const torso = new THREE.Mesh(new THREE.BoxGeometry(0.34, torsoHeight, 0.2), vestMat);
    torso.position.y = legHeight + torsoHeight / 2;
    torso.castShadow = true;
    group.add(torso);

    const head = new THREE.Mesh(new THREE.SphereGeometry(headRadius, 10, 8), skinMat);
    head.position.y = legHeight + torsoHeight + headRadius + 0.02;
    head.castShadow = true;
    group.add(head);

    // Armene - samme pivot-ved-toppen-idé som bena (skulderledd), motsatt fase av bena i løpesvingen
    // (venstre arm frem når høyre ben er frem, som et ekte løpesteg).
    const arms = [];
    [-1, 1].forEach(function (side) {
        const shoulder = new THREE.Group();
        shoulder.position.set(side * 0.21, legHeight + torsoHeight - 0.02, 0);
        const arm = new THREE.Mesh(new THREE.BoxGeometry(0.09, 0.42, 0.09), vestMat);
        arm.position.y = -0.19;
        arm.castShadow = true;
        shoulder.add(arm);
        group.add(shoulder);
        arms.push(shoulder);
    });

    group.userData.legs = legs; // [venstre, høyre] - se løpeanimasjonen i updateTargetHitVisuals
    group.userData.arms = arms; // [venstre, høyre]
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
    // rows opp fra 6 til 10 - brukerens ønske om flere trær ("kan ha flere trær i skogen"). KUN rows, ikke
    // cols: (r - rows/2) gir da nøyaktig SAMME opprinnelige radposisjoner (bare med to nye rader lagt til
    // i +-Z-retning, langt fra selve banens gjennomflygingskorridor - se GATE_WAYPOINTS_TUNNEL). cols er
    // bevisst IKKE utvidet - en ny, ytre kolonne ville havnet påfallende nær "Til topps" sin exit-port
    // fra skogen (dx 178, dz 105), rett i selve gjennomflygingslinjen.
    const centerX = 140, centerZ = 90, rows = 10, cols = 6, spacing = 11;
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
// t.h er den NOMINELLE høyden som sendes inn til Sim.buildRandomTree - men den funksjonen randomiserer
// selv høyden videre ±10% internt (0.9-1.1x, se buildRandomTree i simulator-common.js) for visuell
// variasjon, UTEN å gi den faktiske, endelige høyden tilbake til kalleren. Kollideren her bygges dermed
// alltid fra den GARANTERTE MINSTE mulige faktiske høyden (t.h*0.9) i stedet for selve t.h - ellers ville
// ethvert tre som tilfeldigvis ble trukket kortere enn nominell høyde fått en usynlig "hitboks langt over
// selve treet" (nøyaktig det som ble rapportert). Prisen er at kollideren kan bli opptil ~20% kortere enn
// et tre som ble trukket i den høye enden - ufarlig (man kan så vidt fly gjennom aller ytterste tuppen),
// stikk motsatt av å krasje i tomme luft over et kortere tre.
function treeToColliders(t) {
    // Ekstra margin lagt til her (0.9->0.85 på høyden, 0.22->0.19 på kroneradiusen) - høydematten alene
    // (t.h*0.9, den matematisk garanterte laveste mulige faktiske høyden fra Sim.buildRandomTrees egen
    // ±10%-randomisering) burde i teorien være vanntett mot "kollisjon over selve treet", men ble
    // fortsatt rapportert. Strammer inn ytterligere et hakk på begge akser i stedet for å anta at
    // matematikken var feil - koster litt mer "kan så vidt fly gjennom ytterste kant", tjener mindre
    // "usynlig vegg" i bytte.
    const h = t.h * 0.85;
    const trunkTopY = h * 0.48;
    const canopyTaperTopY = trunkTopY + (h - trunkTopY) * 0.35;
    const trunkR = 0.15;
    const canopyR = h * 0.19;
    const taperR = canopyR * 0.4;
    return [
        { minX: t.x - trunkR, maxX: t.x + trunkR, minZ: t.z - trunkR, maxZ: t.z + trunkR, minY: 0, topY: trunkTopY },
        { minX: t.x - taperR, maxX: t.x + taperR, minZ: t.z - taperR, maxZ: t.z + taperR, minY: trunkTopY, topY: canopyTaperTopY },
        { minX: t.x - canopyR, maxX: t.x + canopyR, minZ: t.z - canopyR, maxZ: t.z + canopyR, minY: canopyTaperTopY, topY: h }
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

// atY (valgfri): referansehøyden du faktisk befinner deg på/ved akkurat nå. En kollider med minY>0
// (f.eks. et trekronedekke, se treeToColliders, eller et tunnel-tak-segment, se
// buildMountainTunnelSegment - taket i toppunnelen henger fritt fra ~46 til ~66 m over selve
// korridorgulvet) er KUN en gyldig "flate å lande på" om referansen allerede er PÅ ELLER OVER selve
// underkanten (man kommer ovenfra og lander OPPÅ den) - default (atY utelatt => Infinity) beholder den
// gamle "fritt fall fra himmelen"-oppførselen for kall som ikke har noen fornuftig referanse.
// Uten denne sjekken leste funksjonen "bakken her" som toppen av tunneltaket langt over hodet selv når
// man faktisk sto/fløy trygt NEDE i korridoren (under taket, ikke oppå det) - kollideren har jo ingen
// fysisk "vegg" ned til bakken, bare den løse XZ-fotavtrykk-sjekken under, som ignorerer høyde helt.
// To konkrete symptomer: en stor, mørk skyggedekal svevende oppe ved taket i stedet for på gulvet under
// droneen (brukerens rapport, med skjermbilde: "stor svart sirkel på himmelen der nesa peker" -
// updateDroneShadowDecal), og i verste fall droneen satt fast oppe i selve taket ved en (re)spawn midt i
// korridoren (settleDroneOnGround).
function solidSurfaceHeightAt(x, z, atY) {
    const ref = atY === undefined ? Infinity : atY;
    let top = mountainHeightAt(x, z, atY); // fjellene er nå del av bakkehøyden - se mountainHeightAt
    SOLID_COLLIDERS.forEach(function (c) {
        if ((c.minY || 0) > ref + 0.5) return; // henger over referansepunktet - ikke en flate du star PÅ nå
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
// Samme rammeoppbygging som buildGateFrame, men med UAVHENGIG beregnet beinlengde per side (legLenLeft/
// Right) i stedet for en fast groundGap-lengde på begge - buildGateFrame antar flatt terreng rett under
// selve porten (elevation 0), noe som IKKE stemmer for porter på en skråning eller i klatrende luft over
// stigende terreng (se buildGroundedGate) - brukeren: "portene i fjellisden må ha begge bena i bakken. så
// et ben må kanskje gjøres lengre. eller begge." Ringen (topp/bunn/sidebjelker) er UENDRET - kun beina
// strekkes ned fra ringens bunnbjelke (lokal y=groundGap) til der de faktisk skal ende.
function buildGateFrameGrounded(size, groundGap, matA, matB, segs, legLenLeft, legLenRight) {
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

    [{ x: -size / 2, len: legLenLeft }, { x: size / 2, len: legLenRight }].forEach(function (side) {
        // Klippet til minst 0.3 m - kun en defensiv sikkerhet mot en null-/negativ lengde sylinder om
        // terrenget der (mot formodning) skulle vise seg å ligge HØYERE enn selve ringens bunnbjelke.
        const len = Math.max(0.3, side.len);
        const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.08, len, 8), legMat);
        // Beinets TOPP er alltid ved ringens bunnbjelke (lokal y=groundGap, uendret fra buildGateFrame) -
        // det er BUNNEN (groundGap-len) som varierer med hvor langt unna ekte bakke/fjellside faktisk er.
        leg.position.set(side.x, groundGap - len / 2, 0);
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

// Pil-dekoren start/mål-gaten bruker (se buildStartFinishGate) - egen funksjon slik at
// buildGroundedGate (under) kan gjenbruke den ordrett for SINE start/mål-porter, i stedet for å duplisere
// koden. Lagt oppå den ferdige rammen (group) - IKKE del av selve buildGateFrame(Grounded).
function addStartFinishArrow(group, size, groundGap) {
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
}
const GATE_MAT_BLACK = new THREE.MeshStandardMaterial({ color: 0x111111 });
// Start/mål-gate for racingbanen - svart/hvitt rutemønster (tettere enn de vanlige oransje/hvite
// portene, for å ligne et ekte målflagg) pluss en gul pil montert over toppbaren som peker i
// flyretningen (lokal +Z - se yaw-formelen ved GATE_PLACEMENTS: samme akse som resten av banen flys mot).
function buildStartFinishGate(size, groundGap) {
    const group = buildGateFrame(size, groundGap, GATE_MAT_BLACK, GATE_MAT_WHITE, 8);
    addStartFinishArrow(group, size, groundGap);
    return group;
}
// Bygger en gate (evt. start/mål-type) der BEGGE ben strekkes ned til EKTE terrenghøyde under hver av de
// to bunnpunktene (mountainHeightAt - samme funksjon resten av kollisjonen på "Til topps" bruker), i
// stedet for buildGate/buildStartFinishGate sin faste beinlengde (som antar flatt terreng rett under selve
// porten - riktig for course 1/2, men IKKE for en bane som klatrer i åpen luft over stigende terreng eller
// står tett inntil en fjellside). Beina kan bli ULIKE lange på en skråning - se buildGateFrameGrounded.
// wp/placement: samme objekter som GATE_PLACEMENTS_TUNNEL bruker (wp.size/gap, placement.x/y/z/yaw).
function buildGroundedGate(wp, placement) {
    const isStartFinish = wp.type === "start";
    // "goal" (brukeren: "siste gate på fjellet må være svart og hvit stripet for å indikere mål") - samme
    // sjakkrutede svart/hvitt-mønster som start/mål-porten på de lukkede løkkebanene bruker, gjenbrukt her
    // for å signalisere "dette er MÅLET" på en punkt-til-punkt-bane (se GATE_WAYPOINTS_TUNNEL sitt siste
    // element). IKKE addStartFinishArrow under - den peker i FLYRETNINGEN, som ikke gir mening for et
    // endepunkt man allerede har nådd.
    const isCheckered = isStartFinish || wp.type === "goal";
    const matA = isCheckered ? GATE_MAT_BLACK : GATE_MAT_ORANGE;
    const segs = isCheckered ? 8 : 6;
    const c = { cx: placement.x, cz: placement.z, yaw: placement.yaw };
    const legWorldLeft = orientedBoxWorldFromLocal(-wp.size / 2, 0, c);
    const legWorldRight = orientedBoxWorldFromLocal(wp.size / 2, 0, c);
    // placement.y som atY (se mountainHeightAt sin egen kommentar): denne porten er MENT å stå omtrent på
    // DENNE høyden - relevant for om et evt. tunnel-hulrom under den faktisk skal telle som gulv her.
    const terrainLeft = mountainHeightAt(legWorldLeft.x, legWorldLeft.z, placement.y);
    const terrainRight = mountainHeightAt(legWorldRight.x, legWorldRight.z, placement.y);
    // Beinlengde = gap (den opprinnelige, flate-terreng-antagelsen) + hvor mye HØYERE enn ekte terreng
    // ringens bunnbjelke faktisk står (placement.y - terrainUnderDetteBeinet) - blir 0 på flatt terreng
    // (placement.y===terrain), akkurat som buildGateFrame allerede antok der.
    const legLenLeft = wp.gap + (placement.y - terrainLeft);
    const legLenRight = wp.gap + (placement.y - terrainRight);
    const group = buildGateFrameGrounded(wp.size, wp.gap, matA, GATE_MAT_WHITE, segs, legLenLeft, legLenRight);
    if (isStartFinish) addStartFinishArrow(group, wp.size, wp.gap);
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

// Låvens SOLIDE vegger (alt unntatt selve vindusåpningen) som ekte kroppsblokkerende SOLID_COLLIDERS -
// før dette fantes bare PROP_HAZARDS-boksene for låven (se GATE_PLACEMENTS/PROP_HAZARDS-løkken lenger
// ned), som kun skader PROPELLENE, ikke kroppen (med vilje for de tynne gate-rammene, se
// ALL_PROP_HAZARDS-kommentaren - en drone skal kunne klippe en propell mot en tynn stolpe uten at hele
// kroppen stopper). For en låve med ordentlige, tykke vegger er det feil oppførsel - selve KROPPEN skal
// ikke fly rett gjennom veggene, bare gjennom vindusåpningen. Samme paneloppdeling (vegg minus
// vindusåpning = seks separate panelbokser + tak) som PROP_HAZARDS-versjonen, men registrert som ekte
// orienterte SOLID_COLLIDERS-bokser (se orientedBoxLocalXZ/-WorldFromLocal) i stedet.
function addBarnWallColliders(placement) {
    const b = BARN_DIMENSIONS, t = 0.15; // halv veggtykkelse (0.3/2), samme som PROP_HAZARDS-versjonen
    const hw = b.width / 2, hd = b.depth / 2;
    const panels = [
        { cx: -hw, cz: 0, halfW: t, halfD: hd, minY: 0, topY: b.height },  // venstre vegg (full lengde)
        { cx: hw, cz: 0, halfW: t, halfD: hd, minY: 0, topY: b.height },   // høyre vegg (full lengde)
        { cx: 0, cz: 0, halfW: hw + 0.3, halfD: hd + 0.3, minY: b.height, topY: b.height + 0.3 } // tak
    ];
    [-hd, hd].forEach(function (zPos) {
        const winTopY = b.sillY + b.windowH, panelW = (b.width - b.windowW) / 2;
        panels.push({ cx: 0, cz: zPos, halfW: hw, halfD: t, minY: 0, topY: b.sillY });               // under vinduet
        panels.push({ cx: 0, cz: zPos, halfW: hw, halfD: t, minY: winTopY, topY: b.height });         // over vinduet
        panels.push({ cx: -hw + panelW / 2, cz: zPos, halfW: panelW / 2, halfD: t, minY: b.sillY, topY: winTopY }); // venstre side av vinduet
        panels.push({ cx: hw - panelW / 2, cz: zPos, halfW: panelW / 2, halfD: t, minY: b.sillY, topY: winTopY });  // høyre side av vinduet
    });
    panels.forEach(function (p) {
        const w = orientedBoxWorldFromLocal(p.cx, p.cz, { cx: placement.x, cz: placement.z, yaw: placement.yaw });
        SOLID_COLLIDERS.push({ cx: w.x, cz: w.z, halfW: p.halfW, halfD: p.halfD, yaw: placement.yaw, minY: p.minY, topY: p.topY });
    });
}

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
        if (wp.type === "barn") addBarnWallColliders(placement);
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
    // BUG (rapportert av brukeren: "klokken på klokketårnet flimrer på avstand") - klokkeskiven henger
    // kun 0.02 m foran tårnveggen (se faceOffset i buildClockTower). På kort hold er det god nok klaring,
    // men depth-bufferet her spenner fra kamera-near (0.05-0.1, se initScene) til far=2000 - en
    // standard (ikke-logaritmisk) dybdebuffer bruker mesteparten av presisjonen sin nær kameraet, så
    // langt unna blir 2 cm mindre enn ÉN dybde-bufferverdi og de to flatene bytter tilfeldig på å vinne
    // depth-testen bilde for bilde (klassisk z-fighting). polygonOffset dytter klokkeskivens EGNE
    // dybdeverdier konsekvent nærmere kameraet FØR depth-testen, uavhengig av avstand - løser flimringen
    // på avstand uten å røre selve geometrien eller det globale depth-bufferet.
    const faceMat = new THREE.MeshStandardMaterial({
        map: buildClockTexture(),
        polygonOffset: true, polygonOffsetFactor: -4, polygonOffsetUnits: -4
    });
    const face = new THREE.Mesh(new THREE.CircleGeometry(radius, 24), faceMat);
    group.add(face);

    // BUG (rapportert av brukeren: "urviserne er ikke synlig før man kommer helt inntill klokka") - SAMME
    // grunnleggende z-fighting-problem som klokkeskiven selv hadde mot tårnveggen (se kommentaren ved
    // faceMat/polygonOffset over), bare ett nivå dypere: viserne satt kun 0.015-0.02 m foran selve
    // SKIVEN (som allerede fikk sin egen polygonOffset for å vinne mot VEGGEN) - et enda mindre mellomrom
    // enn det som allerede var identifisert som for lite til å overleve på avstand i en standard
    // (ikke-logaritmisk) dybdebuffer. Viserne tapte dermed depth-testen mot skiven fra et godt stykke
    // unna og forsvant sporløst - polygonOffset her (dobbelt så aggressiv som skivens -4/-4, siden viserne
    // må vinne mot BÅDE skiven og indirekte veggen bak den) løser det samme problemet på samme måte.
    const handMat = new THREE.MeshStandardMaterial({
        color: 0x2a2a28,
        polygonOffset: true, polygonOffsetFactor: -8, polygonOffsetUnits: -8
    });
    const hourPivot = new THREE.Group();
    const hourHand = new THREE.Mesh(new THREE.BoxGeometry(radius * 0.1, radius * 0.5, 0.02), handMat);
    hourHand.position.set(0, radius * 0.25, 0.03);
    hourPivot.add(hourHand);
    group.add(hourPivot);

    const minutePivot = new THREE.Group();
    const minuteHand = new THREE.Mesh(new THREE.BoxGeometry(radius * 0.07, radius * 0.78, 0.02), handMat);
    minuteHand.position.set(0, radius * 0.39, 0.05);
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
    // Seed fra treets egen (faste) posisjon - se buildRandomTree sin egen kommentar.
    const tree = Sim.buildRandomTree(height, x * 7.13 + z * 3.71);
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
   området) viste seg å ligge INNENFOR kollisjonsradiusen til det sørøstlige fjellet (index 3, det mest
   taggete av alle - jaggedness 1.4) - usynlig "krasj i løse lufta" et godt stykke fra selve fjellsiden.
   Dette var den samme uniformt oppblåste sikkerhetsmarginen som senere ble byttet ut med en presis,
   vinkelavhengig jitterberegning i mountainHeightAt (se der) - kursen ble likevel aldri flyttet tilbake,
   ingen grunn til å risikere en ny runde med akkurat dette problemet et sted den allerede er kjent unna.
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
        if (wp.type === "barn") addBarnWallColliders(placement);
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

/* ---------- Racingbane 3 "Til topps" - punkt-til-punkt (A-til-B), IKKE en lukket løkke som bane 1/2
   ---------- Gjennom skogen ved dammen, opp langs elva (samme innledning som brukerens opprinnelige
   ønske), og videre til en HELT EGEN, håndbygd erstatning for bakgrunnsfjellet MOUNTAIN_PEAKS[0] (samme
   posisjon/radius/høyde som det opprinnelige - (0,620), radius 110, height 69 - se familyIndex-skipene i
   buildMountainRange/mountainHeightAt) med to ekte, gjennomflybare tunneler boret inn i den.
   IKKE et fjell et annet sted (brukeren: "skal ikke ha nye fjell") og IKKE et hull skåret inn i det delte,
   jitrede fjell-nettet (forrige forsøk - fjernet, ga glitchete geometri og usynlig kollisjon der hullet og
   kollisjonsboksen ikke stemte nøyaktig overens). I stedet: fjell #0 fjernes fra den vanlige,
   per-vertex-jitrede modellen (som de syv andre bruker uendret) og bygges på nytt fra bunnen som en egen,
   GLATT (ujitret) struktur - ETT sammenhengende, håndbygd BufferGeometry (buildSummitMountainGeometry,
   IKKE flere separate CylinderGeometry-"høydebånd" stablet oppå hverandre - et tidligere forsøk, som ga
   synlige skjøter/sprekker der båndene møttes, se buildSummitMountainGeometry sin egen kommentar) - der
   akkurat de to tunnelmunning-parene utelates som ekte kvad-hull i selve indeksbufferet, og kollisjonen
   (summitMountainHeightAt) bruker NØYAKTIG samme glatte profilformel som geometrien - null jitter å holde
   synkronisert, null tilnærming, ingen usynlig kollisjon (brukerens krav: "kollisjon som samsvarer helt
   med sin 3D mesh"). Se buildSummitMountain.
   Traséen etter fjellet: rett gjennom fjellfoten på bakkenivå (bakketunnelen), ut på andre siden, en
   spiral av porter som klatrer opp den ekte fjellsiden (samme glatte summitMountainHeightAt-formel gir
   ekte kollisjon der også - en bommet port betyr fjellside, ikke tomrom), en ny, kortere tunnel nærmere
   toppen, og til slutt mål på selve fjelltoppen.
   Alle vinkler/avstander for selve fjellet er regnet relativt (0,620) med samme x=dist*sin(θ),
   z=620+dist*cos(θ)-konvensjon som selve MOUNTAIN_DEFS-plasseringen (θ=0 er nord/+Z, 90 er øst/+X, 180 er
   sør/-Z - vendt mot resten av banen/spillområdet - 270 er vest) - og MERK: three.js sin egen
   CylinderGeometry-thetaStart bruker (x=r·sinθ, z=r·cosθ) internt, altså NØYAKTIG samme konvensjon -
   ingen omregning nødvendig mellom "hvor porten står" og "hvilken thetaStart-vinkel skjærer hullet der". */
const SUMMIT_MOUNTAIN_CENTER_X = 0, SUMMIT_MOUNTAIN_CENTER_Z = 620;
const SUMMIT_MOUNTAIN_RADIUS = 110, SUMMIT_MOUNTAIN_HEIGHT = 69, SUMMIT_MOUNTAIN_TOP_RADIUS_FRAC = 0.18;
// Glatt (ujitret) radius ved en gitt heightFrac (0-1) - gjenbruker NØYAKTIG samme profilformel
// (mountainProfileRadiusFrac, curvePower 1 - lineær, matcher MOUNTAIN_DEFS[0]) som de syv andre fjellene,
// bare uten selve jitter-leddet. Brukt til å bygge hver ring i selve geometrien (buildSummitMountainGeometry)
// - ETT sted for både geometri og (i summitMountainHeightAt under) kollisjon.
function summitMountainRadiusAtHeightFrac(heightFrac) {
    return mountainProfileRadiusFrac(clamp(heightFrac, 0, 1), SUMMIT_MOUNTAIN_TOP_RADIUS_FRAC, 1) * SUMMIT_MOUNTAIN_RADIUS;
}
// Fjellets kollisjonsflate - den glatte inversen (mountainProfileHeightFrac) av formelen over, NØYAKTIG
// samme matematiske profil som selve den synlige geometrien (buildSummitMountainGeometry) bruker - i motsetning
// til de syv andre fjellene (som må gjenskape en jitter-formel for å matche sin egen taggete overflate) er
// det her INGEN jitter å synkronisere i det hele tatt, så INGEN fare for at kollisjonen avviker fra det som
// faktisk vises (brukerens krav om nøyaktig samsvar). Selve tunnelhulrommene (MOUNTAIN_TUNNEL_VOIDS)
// sjekkes separat, FØR denne funksjonen i det hele tatt kalles - se mountainHeightAt.
function summitMountainHeightAt(x, z) {
    const dx = x - SUMMIT_MOUNTAIN_CENTER_X, dz = z - SUMMIT_MOUNTAIN_CENTER_Z;
    const dist = Math.hypot(dx, dz);
    if (dist >= SUMMIT_MOUNTAIN_RADIUS) return 0;
    const distFrac = dist / SUMMIT_MOUNTAIN_RADIUS;
    return SUMMIT_MOUNTAIN_HEIGHT * mountainProfileHeightFrac(distFrac, SUMMIT_MOUNTAIN_TOP_RADIUS_FRAC, 1);
}
// Samme fargestopp-prinsipp (bakke->fot->stein->lys stein, ingen snø - matcher MOUNTAIN_DEFS[0].snow:false)
// som buildMountainRange bruker for de ikke-snødekte fjellene, gjenbrukt her for et konsistent utseende.
const SUMMIT_MOUNTAIN_COLOR_STOPS = [
    { frac: 0, color: MOUNTAIN_GROUND_COLOR },
    { frac: 0.18, color: MOUNTAIN_FOOTHILL_COLOR },
    { frac: 0.5, color: MOUNTAIN_ROCK_COLOR },
    { frac: 1, color: MOUNTAIN_ROCK_LIGHT_COLOR }
];
function summitMountainColorAt(heightFrac) {
    const stops = SUMMIT_MOUNTAIN_COLOR_STOPS;
    let c0 = stops[0], c1 = stops[stops.length - 1];
    for (let s = 0; s < stops.length - 1; s++) {
        if (heightFrac >= stops[s].frac && heightFrac <= stops[s + 1].frac) { c0 = stops[s]; c1 = stops[s + 1]; break; }
    }
    const span = Math.max(1e-6, c1.frac - c0.frac);
    const t = clamp((heightFrac - c0.frac) / span, 0, 1);
    return new THREE.Color().copy(c0.color).lerp(c1.color, t);
}
/* Erstatningsfjellets skallgeometri - ETT sammenhengende BufferGeometry (IKKE flere separate
   CylinderGeometry-"høydebånd" stablet oppå hverandre, som forrige tilnærming brukte, se
   buildSummitMountainBand i git-historikken). Brukeren, etter flere runder med "sprekker og glipper" som
   ikke ble borte: "må settes opp helt på nytt". Rotårsaken til at forrige tilnærming aldri ble helt sømløs:
   hvert bånd fikk sitt EGET, uavhengig beregnede segmentantall (skalert etter båndets thetaLength, se den
   gamle segs-utregningen) - selv om radiene ved en delt høydegrense var matematisk IDENTISKE (samme formel,
   samme input), hadde de to tilstøtende båndene ULIKT ANTALL hjørner langs den grensen, så polygonkantene
   matchet aldri helt - synlige skjøter/sprekker der bånd møttes, uansett hvor nøyaktig selve radius-tallene
   stemte. Med ETT mesh og SAMME segmentantall (SUMMIT_MESH_SEGS) i HELE høyden finnes det ingen skjøter i
   det hele tatt å mismatche - kun de bevisste hullene ved tunnelmunningene (se SUMMIT_TUNNEL_HOLE_RANGES),
   som nå skjæres som eksplisitt utelatte kvadranter i selve indeksbufferet i stedet for en egen geometri-
   instans. Ringene ligger KUN ved høydene noe faktisk skjer (bakke, tunnel-gulv/-tak, topp) - curvePower=1
   (se MOUNTAIN_DEFS[0]) gjør radius-vs-høyde-profilen eksakt LINEÆR, så en rett linje mellom to ringer ER
   den sanne profilen der - ingen mellomliggende ringer trengs for nøyaktighet utenom der noe (et hull)
   faktisk starter/slutter. */
const SUMMIT_MESH_SEGS = 360; // 1° per segment - fin nok oppløsning til at tunnelhullene (se under) kan
// skjæres presist mot korridorenes egen fysiske bredde, se de to buildMountainTunnelSegment-kallene sin
// egen kommentar for selve gradberegningen bak SUMMIT_TUNNEL_HOLE_RANGES.
const SUMMIT_RING_HEIGHTS = [0, 2, 12, 40, 45, 69];
// [startDeg,endDegEksklusiv)-par per bånd (identifisert ved båndets EGEN bunnhøyde, en nøkkel i dette
// objektet) der IKKE noe kvad bygges - selve tunnelmunningenes hull. Nord-munningen (θ0) wrapper rundt
// 0°/360°, derfor to par for det bakketunnel-hullet.
// Halvvidden (grader) for HVERT hull er utledet direkte fra korridorens egen fysiske halvbredde
// (halfWidth+veggtykkelse, se buildMountainTunnelSegment - begge tunnelene bruker halfWidth 6, veggtykkelse
// 2, altså 8 m) via korde-formelen chordHalfWidth = radius*sin(halvvinkel), avrundet NEDOVER til nærmeste
// hele grad (1° pr. segment, se SUMMIT_MESH_SEGS) - ALDRI oppover: et hull som er en anelse SMALERE enn
// korridoren lar tunnelkonstruksjonen dekke hele hullet med en hårfin margin inn i massivt fjell i kantene
// (usynlig, ufarlig); et hull som er en anelse BREDERE etterlater en ekte, gjennomsiktig glipe med ingenting
// bak (brukerens gjentatte rapport - "sprekker og glipper").
//
// VIKTIG: radiusen i denne formelen er RINGENS EGEN radius ved den høyden (summitMountainRadiusAtHeightFrac),
// IKKE korridorens (ax,az)/(bx,bz)-plasseringsradius (48 for toppunnelen, 90 for bakketunnelen) - de to er
// IKKE det samme, siden fjellprofilen (curvePower 1, se mountainProfileRadiusFrac) gjør at ringen ved
// korridorens GULV-høyde alltid ligger LENGER UTE enn selve korridorplasseringen (korridoren er jo bevisst
// plassert et stykke INNENFOR fjellets ekte overflate der, for å ha fjell å bore gjennom). En tidligere
// runde brukte feilaktig 48/90 direkte i asin()-regnestykket - det ga et hull SMALERE/mindre langt ute enn
// selve fjelloverflaten der ringen faktisk sitter, med en synlig, ukledd glipe mellom korridorboksens ende
// og selve hullet i skallet som resultat (brukerens rapport, med skjermbilde: fjellstruktur som stakk opp
// av bakken med gress synlig gjennom/rundt). Riktig utregning bruker den STØRSTE (verste, altså widest)
// av de to ringradiene i båndet - ring-parets BUNN (nærmest fjellfoten, alltid videre enn toppen av
// samme bånd i en innsnevrende profil) - både for hullbredden (avrundet ned, trygt smalere ved den andre,
// trangere ringen også) OG for korridorboksens egen halfD (se buildMountainTunnelSegment-kallene under):
// den må rekke minst like langt ut som denne radiusen for faktisk å dekke hullet, ikke bare til selve
// (ax,az)/(bx,bz)-punktet.
//   Bakketunnelen (gulvring y=2): radius = summitMountainRadiusAtHeightFrac(2/69)*110 ≈ 107.4 m. Krevd
//   halvvinkel asin(8/107.4)≈4.27° → 4° (chordHalfWidth 107.4*sin4°≈7.49 m, trygt under 8). Korridorens
//   overhang (25, se buildMountainTunnelSegment-kallet) gir halfD=115, komfortabel margin til 107.4.
//   Toppunnelen (gulvring y=40): radius = summitMountainRadiusAtHeightFrac(40/69)*110 ≈ 57.7 m - MYE lenger
//   ute enn korridorens egen 48. Krevd halvvinkel asin(8/57.7)≈7.97° → 7° (chordHalfWidth 57.7*sin7°≈7.03 m,
//   trygt under 8). Overhang økt fra standard (3) til 13 (halfD=61) nettopp for å faktisk NÅ denne radiusen
//   - brukerens krav: "øverste tunell må strekkes lengre og ikke ha åpninger i fjellet".
// Toppunnelens to hull (θ200/θ20) ligger fortsatt EKSAKT 180° fra hverandre - en ekte diametrisk boring RETT
// GJENNOM fjellets senterakse i denne høyden, IKKE en kort korde nær overflaten (brukeren, etter flere
// runder: "rett gjennom midten av fjellet") - se buildMountainTunnelSegment-kallet for hele begrunnelsen.
const SUMMIT_TUNNEL_HOLE_RANGES = {
    2: [[176, 184], [356, 360], [0, 4]], // bakketunnelen (2-12 m): sørmunning θ180±4, nordmunning θ0±4
    40: [[193, 207], [13, 27]] // toppunnelen (40-45 m): inngang θ200±7, utgang θ20±7 (nøyaktig motsatt side)
};
// Bygger ETT kvad-nett-lag PER ring-par (yBottom->yTop), med SAMME segmentantall (SUMMIT_MESH_SEGS) i
// HELE geometrien - selve grunnen til at dette ikke lenger er separate CylinderGeometry-instanser (se
// toppkommentaren): identisk segmentantall/vinkelsteg overalt betyr at det ikke finnes NOEN skjøt mellom
// ring-par å mismatche i utgangspunktet, kun de bevisste hullene.
function buildSummitMountainGeometry() {
    const positions = [], colors = [], indices = [];
    for (let r = 0; r < SUMMIT_RING_HEIGHTS.length; r++) {
        const y = SUMMIT_RING_HEIGHTS[r];
        const heightFrac = y / SUMMIT_MOUNTAIN_HEIGHT;
        const radius = summitMountainRadiusAtHeightFrac(heightFrac);
        const col = summitMountainColorAt(heightFrac);
        for (let s = 0; s < SUMMIT_MESH_SEGS; s++) {
            const rad = THREE.MathUtils.degToRad(s); // SUMMIT_MESH_SEGS=360 => 1 segment per grad, s selv ER graden
            positions.push(
                SUMMIT_MOUNTAIN_CENTER_X + Math.sin(rad) * radius, y, SUMMIT_MOUNTAIN_CENTER_Z + Math.cos(rad) * radius
            );
            colors.push(col.r, col.g, col.b);
        }
    }
    // Vertex-indeks for (ring r, segment s) - s wrappes (modulo) slik at siste segment kobler sømløst
    // tilbake til segment 0 uten en duplisert sømvertex.
    function vIdx(r, s) { return r * SUMMIT_MESH_SEGS + ((s % SUMMIT_MESH_SEGS) + SUMMIT_MESH_SEGS) % SUMMIT_MESH_SEGS; }
    function inHole(deg, ranges) {
        for (let i = 0; i < ranges.length; i++) { if (deg >= ranges[i][0] && deg < ranges[i][1]) return true; }
        return false;
    }
    for (let r = 0; r < SUMMIT_RING_HEIGHTS.length - 1; r++) {
        const holes = SUMMIT_TUNNEL_HOLE_RANGES[SUMMIT_RING_HEIGHTS[r]] || [];
        for (let s = 0; s < SUMMIT_MESH_SEGS; s++) {
            if (inHole(s, holes)) continue; // tunnelmunning her - ingen kvad, ekte hull i geometrien
            const a = vIdx(r, s), b = vIdx(r, s + 1), c = vIdx(r + 1, s), d = vIdx(r + 1, s + 1);
            // Vindingsrekkefølge (a,b,c)/(b,d,c) - IKKE (a,c,b)/(b,c,d), en tidligere, feil rekkefølge her
            // ga UTOVERVENDTE trekanter regnet BAKVENDT (bekreftet med håndregning: kryssproduktet pekte
            // INN mot fjellsenteret i stedet for UT), som trolig var rotårsaken til at hele fjellet rendret
            // nesten helt svart (feil normal-fortegn for flatShading sin skjerm-derivert normalberegning,
            // som IKKE nødvendigvis reddes av materialets side:DoubleSide slik man skulle tro).
            indices.push(a, b, c, b, d, c); // to trekanter per kvad
        }
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    geo.setAttribute("color", new THREE.Float32BufferAttribute(colors, 3));
    geo.setIndex(indices);
    geo.computeVertexNormals();
    return geo;
}
// Korridorenes "hulrom" for KOLLISJON (mountainHeightAt - se der) - fylt av buildMountainTunnelSegment
// (kalt fra buildSummitMountain, ved scene-oppbygging) LENGE før mountainHeightAt noensinne faktisk KALLES
// (det skjer først under selve spillingen) - ingen rekkefølge-risiko, se samme mønster ved SOLID_COLLIDERS.
const MOUNTAIN_TUNNEL_VOIDS = [];
// Prosedural canvas-tekstur for tunnelveggene/-taket (brukerens ønske: "ha litt mer tekstur på innsiden av
// tunellen. kanskje noe fin streetart på innsiden?") - samme cache-mønster som buildBrickTexture/
// buildNorwayFlagTexture over (bygget én gang, gjenbrukt av BEGGE tunnelene sin rockMat, se
// buildMountainTunnelSegment). Grov steinstøy (mange små, tilfeldig fargede flekker i grå/brune nyanser)
// pluss noen få enkle, flate graffiti-merker (sirkel-tag, pil, sikksakk) i lyse farger - ren dekorasjon,
// ingen kollisjonspåvirkning.
let tunnelWallTextureBase = null;
function buildTunnelWallTexture() {
    if (tunnelWallTextureBase) return tunnelWallTextureBase;
    const w = 512, h = 512;
    const canvas = document.createElement("canvas");
    canvas.width = w; canvas.height = h;
    const ctx = canvas.getContext("2d");
    // Lysere, mer gråaktig bunnfarge (brukeren: "tunellen kan være mer gråaktig" - forrige "#57524c" var en
    // mørk, brunlig grå som sammen med det dempede lyset inni tunnelen rendret nesten helt svart).
    ctx.fillStyle = "#7d7f82";
    ctx.fillRect(0, 0, w, h);
    // Grov steinstøy - mange små, litt lysere/mørkere flekker i varierende størrelse for et ru, naturlig
    // fjellvegg-utseende i stedet for en helt jevn flate. Shade-området løftet (var 60-115) for å matche den
    // lysere bunnfargen over.
    for (let i = 0; i < 900; i++) {
        const x = Math.random() * w, y = Math.random() * h, r = 2 + Math.random() * 10;
        const shade = 95 + Math.floor(Math.random() * 60);
        ctx.fillStyle = "rgba(" + shade + "," + (shade - 2) + "," + (shade - 4) + "," + (0.2 + Math.random() * 0.35) + ")";
        ctx.beginPath();
        ctx.arc(x, y, r, 0, Math.PI * 2);
        ctx.fill();
    }
    // Noen få mørke "sprekker" - tynne, uregelmessige linjer.
    ctx.strokeStyle = "rgba(20,18,16,0.45)";
    ctx.lineWidth = 1.5;
    for (let i = 0; i < 14; i++) {
        let x = Math.random() * w, y = Math.random() * h;
        ctx.beginPath();
        ctx.moveTo(x, y);
        for (let s = 0; s < 5; s++) {
            x += (Math.random() - 0.5) * 60;
            y += (Math.random() - 0.5) * 60;
            ctx.lineTo(x, y);
        }
        ctx.stroke();
    }
    // Litt "fin streetart" (brukeren) - noen få enkle, flate graffiti-merker i lyse farger. Rene
    // canvas-former (ingen tekst/font-avhengighet) - en tagget sirkel, en pil, en sikksakk-strek.
    function graffitiCircle(cx, cy, r, color) {
        ctx.strokeStyle = color;
        ctx.lineWidth = 5;
        ctx.beginPath();
        ctx.arc(cx, cy, r, 0.4, Math.PI * 1.7);
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(cx, cy, r * 0.55, 0, Math.PI * 2);
        ctx.fillStyle = color;
        ctx.globalAlpha = 0.5;
        ctx.fill();
        ctx.globalAlpha = 1;
    }
    function graffitiArrow(x, y, len, angle, color) {
        ctx.save();
        ctx.translate(x, y);
        ctx.rotate(angle);
        ctx.strokeStyle = color;
        ctx.lineWidth = 6;
        ctx.lineCap = "round";
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(len, 0);
        ctx.lineTo(len - 14, -12);
        ctx.moveTo(len, 0);
        ctx.lineTo(len - 14, 12);
        ctx.stroke();
        ctx.restore();
    }
    function graffitiZigzag(x, y, w2, color) {
        ctx.strokeStyle = color;
        ctx.lineWidth = 5;
        ctx.beginPath();
        ctx.moveTo(x, y);
        for (let i = 1; i <= 5; i++) ctx.lineTo(x + (w2 / 5) * i, y + (i % 2 ? -16 : 16));
        ctx.stroke();
    }
    graffitiCircle(120, 150, 45, "#e0703a");
    graffitiArrow(300, 380, 90, -0.3, "#4fb0c9");
    graffitiZigzag(340, 120, 130, "#d9c53a");
    graffitiCircle(400, 300, 30, "#7bc96f");
    tunnelWallTextureBase = new THREE.CanvasTexture(canvas);
    tunnelWallTextureBase.wrapS = THREE.RepeatWrapping;
    tunnelWallTextureBase.wrapT = THREE.RepeatWrapping;
    return tunnelWallTextureBase;
}
// Myk, varm "lyspøl"-dekal på gulvet under hver taklampe (se lyskilde-løkken i buildMountainTunnelSegment)
// - SAMME radial-gradient-canvas-triks som droneens bakkeskygge (buildDroneShadowTexture), bare lys/varm i
// stedet for mørk. IKKE et ekte THREE.PointLight: det finnes ingen dynamiske punktlys noe annet sted i hele
// filen (kun sunLight+AmbientLight, se initScene) - mange små punktlys ville lagt en kontinuerlig kostnad
// på HVER pixel av HVERT lyspåvirket objekt i HELE scenen (ikke bare inni tunnelen) i three.js sin vanlige
// forward-rendering, mens en gjennomsiktig dekal er praktisk talt gratis og gir samme "her er det lyst"-
// inntrykk. Bygget/cachet én gang og delt av ALLE lampene i BEGGE tunnelene (ingen per-instans repeat/
// offset å style om, i motsetning til buildTunnelWallTexture over).
let tunnelLightPoolTextureBase = null;
function buildTunnelLightPoolTexture() {
    if (tunnelLightPoolTextureBase) return tunnelLightPoolTextureBase;
    const size = 128;
    const canvas = document.createElement("canvas");
    canvas.width = size;
    canvas.height = size;
    const ctx = canvas.getContext("2d");
    const grad = ctx.createRadialGradient(size / 2, size / 2, 0, size / 2, size / 2, size / 2);
    grad.addColorStop(0, "rgba(255,240,190,0.55)");
    grad.addColorStop(0.5, "rgba(255,235,180,0.28)");
    grad.addColorStop(1, "rgba(255,235,180,0)");
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, size, size);
    tunnelLightPoolTextureBase = new THREE.CanvasTexture(canvas);
    return tunnelLightPoolTextureBase;
}
// Bygger vegger/gulv/tak (synlig OG kollisjon, ALLTID nøyaktig samme boks brukt til begge deler - se
// placeBox under - null fare for at det ene avviker fra det andre) for ETT tunnelsegment mellom to
// munninger. ax,az/bx,bz: munningenes senterkoordinater. halfWidth: korridorens halve bredde. floorY/ceilY:
// korridorens gulv-/takhøyde, flat (kollisjonsmodellen støtter kun yaw, ikke pitch - en reelt SKRÅ korridor
// er ikke mulig uten å utvide selve boksmodellen).
// Taket er IKKE én ensartet høyde hele korridoren igjennom - fjellets ekte profilhøyde varierer sterkt
// (lavt ved munningene, opp mot 69 m nærmere sentrum) - delt i ROOF_SEGMENTS biter, hver satt til den EKTE
// profilhøyden (summitMountainHeightAt, samme glatte formel som selve fjellet er bygget av) minus en liten
// margin - aldri høyere enn fjellet faktisk er der, aldri lavere enn nødvendig for å dekke korridoren.
// overhang (valgfri, default 3): hvor mye korridorens beskyttede sone (MOUNTAIN_TUNNEL_VOIDS, se der)
// strekker seg forbi selve munningene på HVER side. Bakketunnelens sørside trenger en MYE større verdi
// (se buildSummitMountain) - fjellets ekte, glatte profilhøyde stiger BRATT rett utenfor munningen (fra 0
// m ved selve fjellfoten, radius 110, til 15+ m allerede ved munningen 20 m lenger inn) - tilnærmingsruta
// dit (se GATE_WAYPOINTS_TUNNEL) flyr i åpen luft over dette stigende terrenget, og trenger nok beskyttet
// sone til å faktisk NÅ munningen før den må dykke ned til korridorens lave gulvhøyde, ellers krasjer den
// i den ekte (usynlige, siden intensjonen var åpen luft) fjellsiden et lite stykke FØR selve åpningen -
// nøyaktig bugen brukeren rapporterte ("krasjer i løse lufta når jeg skal inn i tunnelen").
// voidOverhang (valgfri): styrer SÆRSKILT hvor langt MOUNTAIN_TUNNEL_VOIDS (selve "hulrommet" i
// høydefelt-kollisjonen, se push-kallet nederst) strekker seg - IKKE nødvendigvis det samme som overhang
// (som styrer selve vegg-/takSTRUKTURENS lengde). Default (utelatt) = samme verdi som overhang, altså
// UENDRET oppførsel for bakketunnelen (som fortsatt trenger en stor, delt verdi - se overhang-kommentaren
// over). Toppunnelen sender nå inn en EGEN, mindre voidOverhang (se buildSummitMountain-kallet) - da
// overhang ble økt fra 3 til 13 (se SUMMIT_TUNNEL_HOLE_RANGES-kommentaren om hvorfor) delte hulrommet
// tidligere SAMME (nå mye lengre) rekkevidde som selve veggene, og strakk seg dermed langt nok til å
// dekke store deler av selve toppflaten - der påskeegget/steinene står (se buildSummitRocks/buildSummitMountain).
// MOUNTAIN_TUNNEL_VOIDS-sjekken i mountainHeightAt er en BLIND XZ-only override (returnerer floorY, 40 m,
// for ALT innenfor rektangelet, uansett hvilken høyde spørringen egentlig gjelder) - for steiner/påskeegg
// som faktisk står oppe på ekte fjellhøyde (69 m), men som TILFELDIGVIS lå innenfor det utvidede
// hulrommets smale stripe (siden både korridoren OG toppflaten ligger nær fjellsenteret), ga dette en
// FALSK bakkehøyde på 40 m der - droneen falt rett gjennom den synlige toppflaten og landet 29 m lenger
// ned i stedet (brukerens rapport: "nå glithcer dronen gjennom grunnen som personen og steinene ligger på
// før den treffer en annen grunn like under"). Ved å holde selve hulrommet ved den opprinnelige, trygge
// standardverdien (3) mens bare vegg-/takstrukturen er den lengre (13), unngår hulrommet å nå så langt ut
// i utgangspunktet.
// roofCap (valgfri): høyeste tillatte y for "fyll-stein"-taket over selve korridoren (se ROOF_SEGMENTS-
// løkken under) - default (utelatt) = ingen grense utover den vanlige realHeight-ROOF_MARGIN-beregningen.
// KUN bakketunnelen (den nederste) trenger denne: begge tunnelene bores gjennom EKSAKT samme fjellsenter
// (0,620) - se de to buildMountainTunnelSegment-kallene i buildSummitMountain - så bakketunnelens egen
// "fyll opp til ekte fjellhøyde"-logikk (som nær senteret vil si helt opp mot 69 m, siden fjellet er
// høyest akkurat der) endte med å fylle solid stein rett gjennom TOPPUNNELENS EGEN luftrom (40-45 m) på
// midten av dens strekning, der de to korridorenes baner krysser samme (x,z)-punkt nær senteret - en
// usynlig (kun vanlig fjellstein å se på) vegg midt inne i toppunnelen (brukerens rapport: "kan fly et
// lite stykke inn, men så er det et skarpt hjørne og bom stopp"). Satt til rett under toppunnelens eget
// gulv (40) ved kallet i buildSummitMountain - bakketunnelens fyll-tak stopper der i stedet for å fortsette
// helt opp til ekte fjellhøyde, uansett hvor nær senteret man kommer.
function buildMountainTunnelSegment(ax, az, bx, bz, halfWidth, floorY, ceilY, overhang, voidOverhang, roofCap) {
    const group = new THREE.Group();
    const dx = bx - ax, dz = bz - az;
    const len = Math.hypot(dx, dz);
    const yaw = Math.atan2(dx, dz); // samme "lokal +Z er flyretningen"-konvensjon som resten av banen
    const cx = (ax + bx) / 2, cz = (az + bz) / 2;
    const halfD = len / 2 + (overhang === undefined ? 3 : overhang);
    const voidHalfD = len / 2 + (voidOverhang === undefined ? (overhang === undefined ? 3 : overhang) : voidOverhang);
    const c = { cx: cx, cz: cz, yaw: yaw };
    // map (se buildTunnelWallTexture) - EGEN THREE.Texture-instans per tunnel (samme delte canvas-bilde,
    // men repeat justert etter DENNE korridorens egen lengde, avhenger av halfD/overhang over) - en delt
    // Texture-instans ville latt de to tunnelenes ulike repeat-verdier overskrive hverandre.
    const wallTexture = buildTunnelWallTexture().clone();
    wallTexture.needsUpdate = true;
    wallTexture.wrapS = THREE.RepeatWrapping;
    wallTexture.wrapT = THREE.RepeatWrapping;
    wallTexture.repeat.set(Math.max(1, Math.round((halfD * 2) / 8)), 2);
    // MOUNTAIN_ROCK_LIGHT_COLOR (allerede definert, brukt av fjellenes egne topp-partier) - IKKE den mørkere
    // MOUNTAIN_ROCK_COLOR som først ble brukt her - inni tunnelen er belysningen svakere enn ute i solen, så
    // en farge som ser passe grå ute på fjellsiden blir nesten svart der inne (brukeren: "tunellen kan være
    // mer gråaktig").
    const rockMat = new THREE.MeshStandardMaterial({ color: MOUNTAIN_ROCK_LIGHT_COLOR, roughness: 0.95, flatShading: true, map: wallTexture });
    const floorMat = new THREE.MeshStandardMaterial({ color: 0x464034, roughness: 1 });

    function placeBox(w, h, d, lx, ly, lz, mat) {
        const mesh = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat);
        const wp = orientedBoxWorldFromLocal(lx, lz, c);
        mesh.position.set(wp.x, ly, wp.z);
        mesh.rotation.y = yaw;
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        group.add(mesh);
    }

    const wallThick = 2, floorThick = 0.3;
    placeBox(halfWidth * 2, floorThick, halfD * 2, 0, floorY - floorThick / 2, 0, floorMat); // synlig gulvflate
    placeBox(wallThick, ceilY - floorY, halfD * 2, -(halfWidth + wallThick / 2), (floorY + ceilY) / 2, 0, rockMat);
    placeBox(wallThick, ceilY - floorY, halfD * 2, halfWidth + wallThick / 2, (floorY + ceilY) / 2, 0, rockMat);

    const wallW = orientedBoxWorldFromLocal(-(halfWidth + wallThick / 2), 0, c);
    const wallE = orientedBoxWorldFromLocal(halfWidth + wallThick / 2, 0, c);
    SOLID_COLLIDERS.push(
        { cx: wallW.x, cz: wallW.z, halfW: wallThick / 2, halfD: halfD, yaw: yaw, minY: floorY, topY: ceilY },
        { cx: wallE.x, cz: wallE.z, halfW: wallThick / 2, halfD: halfD, yaw: yaw, minY: floorY, topY: ceilY }
    );

    // Opp fra 8 - færre segmenter ga tydelige "trappetrinn" i takhøyden (brukeren: "rare horisontale
    // seksjoner/linjer i fjellsiden") synlige gjennom selve munningsåpningen. Flere, tynnere segmenter gir
    // en mye jevnere (om enn fortsatt trinnvis, se ROOF_MARGIN-kommentaren) tilnærming til den faktiske,
    // glatte profilkurven taket følger.
    const ROOF_SEGMENTS = 24, ROOF_MARGIN = 3;
    for (let s = 0; s < ROOF_SEGMENTS; s++) {
        const lz0 = -halfD + (2 * halfD) * (s / ROOF_SEGMENTS);
        const lz1 = -halfD + (2 * halfD) * ((s + 1) / ROOF_SEGMENTS);
        // Ekte profilhøyde i alle FIRE hjørner av dette segmentet (begge sider av korridoren, begge
        // endene) - bruker den LAVESTE (mest konservative), slik at taket ALDRI stikker opp gjennom
        // fjellets faktiske overflate noe sted i segmentet.
        const corners = [
            orientedBoxWorldFromLocal(halfWidth, lz0, c), orientedBoxWorldFromLocal(halfWidth, lz1, c),
            orientedBoxWorldFromLocal(-halfWidth, lz0, c), orientedBoxWorldFromLocal(-halfWidth, lz1, c)
        ];
        let realHeight = Infinity;
        corners.forEach(function (p) { realHeight = Math.min(realHeight, summitMountainHeightAt(p.x, p.z)); });
        let segTop = Math.max(ceilY + 1, realHeight - ROOF_MARGIN);
        // roofCap (se funksjonens egen kommentar) - hindrer fyll-taket i å stikke opp i EN ANNEN korridors
        // eget luftrom nær fjellsenteret, uansett hvor høyt den ekte fjelloverflaten er akkurat der.
        if (roofCap !== undefined) segTop = Math.min(segTop, roofCap);
        const segCenterLz = (lz0 + lz1) / 2;
        placeBox(halfWidth * 2, segTop - ceilY, lz1 - lz0, 0, (ceilY + segTop) / 2, segCenterLz, rockMat);
        const segW = orientedBoxWorldFromLocal(0, segCenterLz, c);
        // SAMME tall (segTop) brukt til den synlige boksen over OG kollideren her - garantert samsvar.
        SOLID_COLLIDERS.push({ cx: segW.x, cz: segW.z, halfW: halfWidth, halfD: (lz1 - lz0) / 2, yaw: yaw, minY: ceilY, topY: segTop });
    }

    // Taklys, jevnt fordelt langs hele korridoren (brukerens krav: "må ha lys i tunellene. kanskje noen
    // lys i taket?") - begge tunnelene er kun ambient-belyst inni (solens DirectionalLight når ikke inn
    // under taket), så uten dette blir de ganske flate/mørke å fly gjennom sammenlignet med resten av
    // banen. Armaturet er en liten mørk "husk"-boks med en lysende "pære"-flate rett under - pæren bruker
    // MeshBasicMaterial (IKKE MeshStandardMaterial+emissive, som fortsatt skyggelegges av scenens lys og
    // kan se svakt/dødt ut i den ellers ganske jevnt ambient-belyste tunnelen) for et garantert synlig,
    // skyggeuavhengig lys, samme grunnprinsipp som de glødende vindusflatene på husene (se buildBuilding).
    // PLUSS en myk "lyspøl"-dekal på gulvet under hver lampe (se buildTunnelLightPoolTexture - IKKE et
    // ekte THREE.PointLight, se den funksjonens egen kommentar for hvorfor).
    const LIGHT_SPACING = 22; // meter mellom hver lampe langs korridoren
    const lightCount = Math.max(1, Math.floor((halfD * 2) / LIGHT_SPACING) + 1);
    const lampHousingMat = new THREE.MeshStandardMaterial({ color: 0x262626, roughness: 0.7, metalness: 0.2 });
    const lampBulbMat = new THREE.MeshBasicMaterial({ color: 0xfff0c0 });
    const lightPoolMat = new THREE.MeshBasicMaterial({
        map: buildTunnelLightPoolTexture(), transparent: true, depthWrite: false
    });
    for (let i = 0; i < lightCount; i++) {
        const lz = lightCount === 1 ? 0 : -halfD + (2 * halfD) * (i / (lightCount - 1));
        const wpHousing = orientedBoxWorldFromLocal(0, lz, c);
        const housing = new THREE.Mesh(new THREE.BoxGeometry(2.2, 0.16, 1.0), lampHousingMat);
        housing.position.set(wpHousing.x, ceilY - 0.08, wpHousing.z);
        housing.rotation.y = yaw;
        housing.castShadow = true;
        group.add(housing);
        const bulb = new THREE.Mesh(new THREE.BoxGeometry(1.7, 0.04, 0.7), lampBulbMat);
        bulb.position.set(wpHousing.x, ceilY - 0.19, wpHousing.z);
        bulb.rotation.y = yaw;
        group.add(bulb);
        // renderOrder 1 (etter selve gulvflaten, som er default renderOrder 0) - samme depthWrite:false/
        // renderOrder-kombinasjon som droneens bakkeskygge-dekal bruker mot bakken/plattformen den ligger
        // over, unngår z-fighting-flimring mot gulvet rett under.
        // CircleGeometry (IKKE PlaneGeometry, som forrige versjon brukte) - en firkantet flate rotert flat
        // KUN om X (rotation.x=-PI/2, uten en tilsvarende Y-rotasjon for selve korridorens yaw) beholder
        // sin kvadratiske footprint LANGS VERDENS X/Z-AKSENE, uavhengig av korridorens faktiske retning -
        // i toppunnelen (yaw ca. 20°, IKKE akse-rettet slik bakketunnelen tilfeldigvis er) stakk hjørnene
        // på den 10+ m brede firkanten dermed synlig UT gjennom sideveggene på skrå, og så ut som et fast,
        // lysende objekt som blokkerte korridoren (brukerens rapport: "den øverste tunellen har fortsatt
        // et objekt inne i seg som blokkerer tunellen"). En sirkel er rotasjonssymmetrisk om sin egen
        // normal, så den samme bugen er umulig uansett korridorretning - trenger ingen egen
        // yaw-korrigering i det hele tatt. Også tydelig mindre (radius halfWidth*0.7=4.2 m, diameter 8.4 m
        // - komfortabelt innenfor korridorens 12 m bredde) enn den forrige, nesten vegg-til-vegg firkanten.
        const pool = new THREE.Mesh(new THREE.CircleGeometry(halfWidth * 0.7, 20), lightPoolMat);
        pool.rotation.x = -Math.PI / 2;
        pool.position.set(wpHousing.x, floorY + 0.02, wpHousing.z);
        pool.renderOrder = 1;
        group.add(pool);
    }

    // Selve "hullet" i den delte fjell-høydefelt-modellen (mountainHeightAt) - gulvet i korridoren, i
    // stedet for den vanlige fjellprofilen, kun innenfor dette rektangelet (samme cx/cz/yaw/halfW/halfD som
    // kollisjonsboksene over - én kilde til sannhet for hvor korridoren faktisk er).
    MOUNTAIN_TUNNEL_VOIDS.push({ cx: cx, cz: cz, halfW: halfWidth, halfD: voidHalfD, yaw: yaw, floorY: floorY, ceilY: ceilY });

    return group;
}
// Noen få lavpoly "steiner" spredt på selve toppflaten - brukerens ønske om mer tekstur/naturlige detaljer
// deroppe. IcosahedronGeometry(r,0) (flatShading via materialet) gir en billig, uregelmessig steinform uten
// egen vertex-jitter-logikk - per-instans skalering på alle tre akser (ulik per akse) bryter opp den
// ellers for regelmessige polyeder-silhuetten videre. Rene dekorasjonsobjekter (ingen egen kollisjon,
// samme "for lite/lavt til å trenge egen SOLID_COLLIDERS-oppføring"-prinsipp som PROP_HAZARDS-unntakene
// andre steder i filen) - plassert (vinkel/avstand) for bevisst å unngå både påskeegget (offset ~10,5 fra
// senter, θ≈63°) og målporten (offset -4,8/13,2, θ≈340°, dist≈14), se de valgte vinklene under. Nærmeste
// stein til hver av dem (θ15/dist9 mot egget, θ310/dist11 mot målporten) har fortsatt 7+ m klaring.
function buildSummitRocks() {
    const group = new THREE.Group();
    const mat = new THREE.MeshStandardMaterial({ color: MOUNTAIN_ROCK_COLOR, flatShading: true, roughness: 1 });
    // { angleDeg, dist, r } - håndplassert, IKKE en loop-generert vifte, nettopp for å kunne styre unna
    // påskeegget/målporten presist i stedet for å stole på en filtrerings-sjekk.
    const ROCKS = [
        { angleDeg: 15, dist: 9, r: 1.1 },
        { angleDeg: 100, dist: 12, r: 0.8 },
        { angleDeg: 130, dist: 8, r: 1.4 },
        { angleDeg: 160, dist: 14, r: 0.9 },
        { angleDeg: 190, dist: 10, r: 1.2 },
        { angleDeg: 310, dist: 11, r: 0.7 }
    ];
    ROCKS.forEach(function (spec, i) {
        const ang = THREE.MathUtils.degToRad(spec.angleDeg);
        const rock = new THREE.Mesh(new THREE.IcosahedronGeometry(spec.r, 0), mat);
        rock.scale.set(1 + Math.sin(i * 2.2) * 0.35, 0.55 + Math.abs(Math.cos(i * 3.9)) * 0.35, 1 + Math.cos(i * 1.7) * 0.35);
        rock.rotation.set(Math.sin(i) * 2, Math.cos(i * 1.3) * 3, Math.sin(i * 0.7) * 2);
        rock.position.set(
            SUMMIT_MOUNTAIN_CENTER_X + Math.sin(ang) * spec.dist,
            69 + spec.r * 0.25,
            SUMMIT_MOUNTAIN_CENTER_Z + Math.cos(ang) * spec.dist
        );
        rock.castShadow = true;
        rock.receiveShadow = true;
        group.add(rock);
    });
    return group;
}
function buildSummitMountain() {
    const group = new THREE.Group();
    // Selve fjellskallet - ETT sammenhengende, sømløst mesh (se buildSummitMountainGeometry sin egen
    // kommentar for hele begrunnelsen bak omskrivingen fra det forrige, sømfulle bånd-baserte forsøket).
    const mat = new THREE.MeshStandardMaterial({ vertexColors: true, flatShading: true, roughness: 1, side: THREE.DoubleSide });
    const shell = new THREE.Mesh(buildSummitMountainGeometry(), mat);
    shell.castShadow = true;
    shell.receiveShadow = true;
    group.add(shell);
    // Toppflaten - flat disk (radius = summitMountainRadiusAtHeightFrac(1) = topRadiusFrac*radius ≈ 19.8).
    // EGEN, vanlig heltone-material (IKKE den delte vertexColors-`mat` båndene bruker) - CircleGeometry
    // har ingen egen "color"-attributt satt, og et MeshStandardMaterial med vertexColors:true uten noen
    // slik attributt på geometrien rendrer helt SVART (brukeren: "toppen av fjellet er helt svart?").
    const capMat = new THREE.MeshStandardMaterial({ color: MOUNTAIN_ROCK_LIGHT_COLOR, flatShading: true, roughness: 1 });
    const capRadius = summitMountainRadiusAtHeightFrac(1);
    // SUMMIT_MESH_SEGS (IKKE MOUNTAIN_RADIAL_SEGMENTS) - matcher selve skallets toppring segment-for-segment
    // langs samme sirkel, samme "ingen skjøt å mismatche"-prinsipp som resten av buildSummitMountainGeometry.
    const cap = new THREE.Mesh(new THREE.CircleGeometry(capRadius, SUMMIT_MESH_SEGS), capMat);
    cap.rotation.x = -Math.PI / 2;
    cap.position.set(SUMMIT_MOUNTAIN_CENTER_X, 69, SUMMIT_MOUNTAIN_CENTER_Z);
    cap.receiveShadow = true;
    group.add(cap);

    // Påskeegget som tidligere satt på MOUNTAIN_PEAKS[0] (buildLoneHiker - se MOUNTAIN_EASTER_EGGS i
    // buildMountainRange, som nå hopper over familyIndex 0) - flyttet hit, litt unna selve målporten
    // (dx -4.8, dz 633.2, se GATE_WAYPOINTS_TUNNEL) så de to ikke overlapper på det samme lille platået.
    const egg = buildLoneHiker();
    egg.position.set(SUMMIT_MOUNTAIN_CENTER_X + 10, 69.04, SUMMIT_MOUNTAIN_CENTER_Z + 5);
    egg.rotation.y = Math.PI;
    group.add(egg);

    // "kan ha mer tekstur på fjelltoppen også. kanskje noen naturlige steiner også?" (brukeren) - noen få
    // lavpoly steiner spredt på selve toppflaten, se buildSummitRocks.
    group.add(buildSummitRocks());

    // Bakketunnelen - rett gjennom senteret (0,620) langs Z, mellom sør- og nordmunningen. overhang=25 (opp
    // fra standard 3) - se buildMountainTunnelSegment sin egen kommentar for hvorfor: den ekte profilhøyden
    // stiger BRATT rett utenfor sørmunningen (0 m ved selve fjellfoten, radius 110/dist 90+25=115 fra
    // senter, til 15+ m allerede ved dist 90) - tilnærmingsruta inn (se GATE_WAYPOINTS_TUNNEL) trenger nok
    // beskyttet sone til faktisk å NÅ munningen i åpen luft før den dykker ned til gulvhøyden, uten å
    // krasje i den ekte fjellsiden et lite stykke FØR selve åpningen.
    // roofCap=39 (SISTE argument, voidOverhang utelatt/undefined = uendret standardoppførsel der) - se
    // buildMountainTunnelSegment sin egen roofCap-kommentar: uten denne fylte bakketunnelens "stein opp til
    // ekte fjellhøyde"-tak rett gjennom toppunnelens eget luftrom (40-45 m) nær fjellsenteret, der begge
    // korridorenes baner krysser samme punkt (begge er boret gjennom eksakt (0,620)).
    group.add(buildMountainTunnelSegment(0, 530, 0, 710, 6, 2, 12, 25, undefined, 39));
    // Toppunnelen - IKKE lenger en kort korde nær overflaten (θ210/θ250, så θ198/θ255 - flere tidligere
    // runder), men en EKTE DIAMETRISK boring: inngang θ200, utgang θ20, NØYAKTIG 180° fra hverandre, begge
    // på radius 48 fra fjellsenteret - korridoren går dermed i en RETT linje GJENNOM selve senterpunktet
    // (0,620) i denne høyden, akkurat som bakketunnelen under (θ180 til θ0) allerede gjør lenger nede.
    // Brukeren, etter flere runder med kortere korder som fortsatt så "skrå"/overflatiske ut: "rett gjennom
    // midten av fjellet". En diametrisk linje har INGEN "korde buer innover fra sirkelen"-komplikasjon i det
    // hele tatt (se tidligere versjoners lange resonnement om sagitta/mismatch, nå overflødig) - midtpunktet
    // ER selve fjellsenteret (dist 0, ekte høyde helt opp til 69 m, se summitMountainHeightAt), så klaringen
    // over taket er STØRST nøyaktig midtveis i stedet for å variere langs korridoren.
    // Gulv/tak (40-45 m, uendret) er fortsatt trygt: den TRANGESTE flaskehalsen langs hele korridoren er
    // (uforandret av selve vinkelen - kun radiusen 48 avgjør) EKTE fjellhøyde ved munningene selv (dist 48 →
    // høyde ≈47,4 m) - taket (45 m) ligger 2,4 m under denne, og ellers langs korridoren (helt opp til
    // ≈69 m midtveis) er klaringen enda større. ROOF_SEGMENTS fyller som før opp til ekte overflate langs
    // hele den nå ≈96 m lange korridoren (full diameter).
    // Selve hullet i fjellskallet (θ200±7/θ20±7) er utledet fra korridorens fysiske bredde OG ringens EGEN
    // (videre) radius ved gulvhøyden - se SUMMIT_TUNNEL_HOLE_RANGES sin egen kommentar for hele
    // regnestykket, inkludert hvorfor 48 (korridorplasseringen) IKKE er samme tall som ringradiusen der.
    // overhang økt fra standard (3) til 13 (halfD=48+13=61) - MÅ rekke minst ut til gulvringens egen radius
    // (≈57.7, se over) for at korridorboksens vegger faktisk skal dekke det utskårne hullet i skallet; med
    // gammel overhang (3, halfD=51) stoppet veggene 6-7 m for tidlig, og selve fjellåpningen (som fysisk
    // sitter ved radius ≈51.2-57.7, IKKE ved 48) sto delvis udekket - synlig som en boksstruktur med gress/
    // bakke synlig rundt/under i stedet for solid fjell (brukerens rapport, med skjermbilde: "øverste
    // tunell må strekkes lengre og ikke ha åpninger i fjellet"). Med halfD=61 stikker selve tunnelrøret nå
    // godt synlig ut av fjellsiden ved begge munninger (13 m forbi de fysiske portene på θ200/θ20, se
    // GATE_WAYPOINTS_TUNNEL) - en ekte, solid tunnelportal i stedet for en for kort boks gjemt for langt inne.
    // voidOverhang=13 (SISTE argument - nå SAMME som selve vegg-/takstrukturens overhang, IKKE lenger en
    // egen, mindre verdi). Den forrige, mindre verdien (3) var en overkorrigering: den ga en FOR SMAL
    // beskyttet gulvsone helt inntil selve munningen (kun siste ~3 m før hullet) - en tilnærming som ikke
    // traff eksakt midt på tunnelaksen (normalt for en ekte pilot) traff dermed den ekte, bratte
    // fjellsiden rett før selve åpningen, umulig å skille fra vanlig fjellstein (brukerens rapport:
    // "krasjer i noe usynlig på vei inn... det er fortsatt en synlig blokkering inne i tunellen. kan ikke
    // se gjennom den eller fly gjennom"). Den opprinnelige bekymringen som ga voidOverhang=3 (et STORT
    // hulrom ville dekke toppflaten der påskeegget/steinene står, se mountainHeightAt) er nå løst på riktig
    // sted - selve høyde-bevisstheten (atY/ceilY-sjekken i mountainHeightAt, se den funksjonens kommentar)
    // hindrer korridorens gulvhøyde-override fra å gjelde for spørringer langt over korridorens eget tak,
    // UANSETT hvor langt hulrommets XZ-fotavtrykk strekker seg - trygt å gjøre hulrommet like langt som
    // selve vegg-/takstrukturen igjen uten å gjeninnføre den opprinnelige "glitcher gjennom bakken"-bugen.
    group.add(buildMountainTunnelSegment(-16.4, 574.9, 16.4, 665.1, 6, 40, 45, 13, 13));

    return group;
}

const GATE_COURSE_TUNNEL_CENTER = new THREE.Vector3(0, 0, 0);
// Portenes verdenskoordinater (dx/dz - GATE_COURSE_TUNNEL_CENTER er (0,0,0)). De seks første (skog/dam)
// og de fire neste (langs elva, klatring begynner) er samme trasé som brukerens opprinnelige ønske
// ("gjennom skogen ved vannet, opp langs elva") - se RIVER_POINTS/FOREST_TREES. Deretter svinger banen
// vestover og glir ned mot fjellets sørmunning (world x≈0) i stedet for å fortsette til et eget fjell.
const GATE_WAYPOINTS_TUNNEL = [
    { type: "start", dx: 20, dz: 40, gap: 1.8, size: 3.2 },                    // sør for dammen (RIVER_POND_RADIUS ved 45,70)
    { type: "gate", dx: 65, dz: 85, gap: 1.8, size: 3.0 },                     // forbi dammens østkant
    { type: "gate", dx: 118, dz: 95, gap: 1.8, size: 3.0 },                    // inn i skogen (FOREST_TREES, senter 140,90)
    { type: "gate", dx: 151, dz: 95, gap: 1.8, size: 3.0 },                    // gjennom skogens rad-mellomrom
    { type: "gate", dx: 178, dz: 105, gap: 1.8, size: 3.0 },                   // ut av skogen, mot elva
    // To NYE overgangsporter (brukeren: "banen etter skogen må ikke snu så brått tilbake mot elva. heller
    // svinge slakere lenger opp mot elva og fjellet" - forrige versjon hoppet rett fra skogutgangen (dx178)
    // til en enkelt elve-port langt mot vest (dx85), en brå ~140° helomvending i én eneste sving). Fortsetter
    // nå stort sett rett fram et lite stykke først (dx200 - fortsatt utenfor selve skogen, som slutter rundt
    // dx177, se FOREST_TREES), så en slakere, videre sving (dx175→videre) HØYERE OPP (større dz) enn før,
    // fordelt over TO svinger i stedet for én brå - før den kobler seg på elve-følgingen ved dx120/dz200.
    { type: "gate", dx: 200, dz: 118, gap: 1.8, size: 3.0, elevation: 2 },
    { type: "gate", dx: 175, dz: 160, gap: 1.8, size: 3.0, elevation: 2 },
    // Langs elva (RIVER_POINTS) - dx-verdiene er OFFSET ut til østbredden (halve elvebredden - RIVER_WIDTHS,
    // interpolert ved samme z - PLUSS en liten margin) fra selve elvens senterlinje, IKKE midt i selve
    // elveløpet (brukerens rapport: "ingen gates midt i elva" - forrige versjon brukte senterlinjens EGNE
    // dx-verdier direkte, altså bokstavelig midt i vannet). Holdes lavt (elevation 2-6, ekte bakkekontakt
    // et lite stykke over bredden) HELE veien til fjellfoten - ingen kunstig høy "sikkerhetsstrekning" i
    // åpen luft over flatt mark lenger - korte, lave ben (brukerens rapport: "de skal ikke ha så lange
    // ben.. skal være nært bakken. kun lengre ben i fjellsiden der det trengs").
    // Senket enda et hakk (brukeren, gjentatt: "Gatene i til topps banen langs elva må senkes mye nærmere
    // bakken") - omtrent halvert fra forrige runde (2/2/3/4/6 -> 1/1/1.5/2/3). Fortsatt stigende mot
    // fjellfoten (der terrenget selv begynner å stige mot den ekte fjellsiden, se buildGroundedGate sin
    // egen ekte terrengfølging) - IKKE 0 hele veien, som ville gitt for lite klaring over selve elvebredden/
    // ujevnheter i bakken den siste strekningen.
    { type: "gate", dx: 120, dz: 200, gap: 1.8, size: 3.0, elevation: 1 },
    { type: "gate", dx: 131, dz: 270, gap: 1.8, size: 3.0, elevation: 1 },
    { type: "gate", dx: 130, dz: 340, gap: 1.8, size: 3.0, elevation: 1.5 },
    { type: "gate", dx: 123, dz: 400, gap: 1.8, size: 3.0, elevation: 2 },
    { type: "gate", dx: 95, dz: 460, gap: 1.8, size: 3.0, elevation: 3 },
    // IKKE lenger en egen liten port ved dz500 rett før selve munningsporten (brukeren: "nederste tunell.
    // trenger ikke ha den lille gaten inne i tunellen rett før den store" - de to lå bare 30 m fra
    // hverandre, unødvendig tett/overflødig). Går nå rett fra elve-følgingen til selve munningsporten -
    // fortsatt trygt: den ekte fjellsiden stiger først BRATT innenfor radius 110 fra senter (0,620) (se
    // buildMountainTunnelSegment sin overhang-kommentar), og ved x≈0 er MOUNTAIN_TUNNEL_VOIDS sin
    // beskyttede korridor (halfW 6) allerede aktiv fra z>=505 - hele strekningen hit inn er derfor enten
    // reelt flatt terreng (dist>110) eller inne i den beskyttede, flate korridorgulv-sonen.
    { type: "gate", dx: 0, dz: 530, gap: 2.0, size: 6.0, elevation: 3 },    // bakketunnelen inn (sørmunning, θ180,dist90)
    // yawFromPrev (se GATE_PLACEMENTS_TUNNEL) - denne portens "neste" i selve rekkefølgen er spiral-
    // klatringen (svinger kraftig av mot θ45), IKKE rett frem gjennom tunnelen - uten dette pekte porten
    // feil vei (mot neste banedel i stedet for langs tunnelaksen), og beina havnet utenfor den beskyttede
    // korridoren, opp mot den ekte, bratte fjellsiden - porten endte synlig "oppå" munningen i stedet for i
    // selve åpningen (brukerens rapport, med skjermbilde).
    { type: "gate", dx: 0, dz: 710, gap: 2.0, size: 6.0, elevation: 3, yawFromPrev: true }, // bakketunnelen ut (nordmunning, θ0,dist90)
    { type: "gate", dx: 55.2, dz: 675.2, gap: 1.8, size: 3.0, elevation: 26 }, // spiral opp østsiden (θ45,dist78)
    { type: "gate", dx: 64.0, dz: 620.0, gap: 1.8, size: 3.0, elevation: 37 }, // (θ90,dist64)
    { type: "gate", dx: 32.1, dz: 581.7, gap: 1.8, size: 3.0, elevation: 48 }, // (θ140,dist50)
    { type: "gate", dx: -10.4, dz: 581.4, gap: 1.8, size: 3.0, elevation: 55 }, // (θ195,dist40)
    // Munningsportene (bredere/lavere gap enn standard, se gate-kommentaren over) - gjør at ringen fyller
    // det meste av selve tunneltverrsnittet (halfWidth 6, gulv/tak 40-45) i stedet for å henge som en liten
    // frittstående ring midt i en mye større åpning (brukeren: "portene kan kanskje gå nesten flush med
    // tunnelen inni der?"). Koordinatene (radius 48, θ200/θ20 - ekte diametrisk boring, se buildSummitMountain
    // sin kommentar ved buildMountainTunnelSegment-kallet) og selve gulv-/takhøyden (40-45) matcher
    // buildSummitMountain sin egen toppunnel-geometri - elevation sitter fortsatt nær korridorens eget gulv
    // (40.2).
    { type: "gate", dx: -16.4, dz: 574.9, gap: 0.6, size: 3.8, elevation: 40.2 }, // toppunnelen inn (θ200,dist48)
    // yawFromPrev - samme begrunnelse som nordmunningen over: denne portens "neste" er klatreporten videre
    // mot toppen (svinger kraftig av fra tunnelaksen, se under), ikke rett frem langs toppunnelens akse.
    { type: "gate", dx: 16.4, dz: 665.1, gap: 0.6, size: 3.8, elevation: 40.2, yawFromPrev: true }, // toppunnelen ut (θ20,dist48)
    // Kort klatring fra munningen (θ20,dist48,høyde40.2) opp mot toppflaten (θ~10,dist30,høyde~60 - ekte
    // fjellhøyde der er ≈61.2 m, se summitMountainRadiusAtHeightFrac/mountainProfileHeightFrac, så 60 gir
    // trygg klaring). Munningen ligger nå på MOTSATT side av fjellet fra den forrige (θ255-baserte) utgangen
    // - målporten flyttet tilsvarende (se under) i stedet for å beholde en lang, unaturlig omvei tilbake
    // rundt fjellet til den gamle plasseringen.
    { type: "gate", dx: 5.2, dz: 649.5, gap: 1.8, size: 3.0, elevation: 60 },
    // Mål - nær toppflaten (θ~340,dist14), god klaring til både påskeegget (offset ~10,5 fra senter, θ≈63°)
    // og steinene ved θ15/dist9 (buildSummitRocks) - se koordinatvalget i git-historikken for regnestykket.
    // type "goal" (ikke "gate") - svart/hvitt sjakkrutet i stedet for oransje/hvit, se buildGroundedGate.
    { type: "goal", dx: -4.8, dz: 633.2, gap: 1.8, size: 3.0, elevation: 70 }
];
const GATE_PLACEMENTS_TUNNEL = GATE_WAYPOINTS_TUNNEL.map(function (wp, i) {
    const isLast = i === GATE_WAYPOINTS_TUNNEL.length - 1;
    // Ingen "neste port" å peke mot for den SISTE (i motsetning til GATE_PLACEMENTS_2 sin lukkede løkke,
    // der (i+1)%length alltid finnes) - fortsetter i stedet samme retning som selve ankomsten dit.
    // wp.yawFromPrev (valgfritt, satt på de to munnings-UTGANGENE - se GATE_WAYPOINTS_TUNNEL) - samme
    // "fortsett ankomstretningen" som isLast, men brukt MIDT i lista: en munning-UTGANG sin "neste port" i
    // selve rekkefølgen svinger av i en helt annen retning (inn i neste del av banen), mens porten selv
    // fysisk MÅ stå på linje med selve tunnelaksen (samme retning som ankomsten FRA munning-INNGANGEN) -
    // uten dette pekte utgangsporten mot neste bane-del i stedet for langs tunnelen, som ga en helt feil
    // beinretning (buildGroundedGate sine ben endte da UTENFOR MOUNTAIN_TUNNEL_VOIDS sin smale, beskyttede
    // korridor - der er ekte, bratt fjellside - i stedet for den flate tunnelgulv-høyden) - portens ben
    // strakk seg da helt opp til den ekte fjelloverflaten, og porten endte synlig "oppå" munningen i
    // stedet for å stå i selve åpningen (brukerens rapport, med skjermbilde).
    const useIncoming = isLast || wp.yawFromPrev;
    const reference = useIncoming ? GATE_WAYPOINTS_TUNNEL[i - 1] : GATE_WAYPOINTS_TUNNEL[i + 1];
    const yaw = useIncoming
        ? Math.atan2(wp.dx - reference.dx, wp.dz - reference.dz)
        : Math.atan2(reference.dx - wp.dx, reference.dz - wp.dz);
    return {
        wp: wp,
        x: GATE_COURSE_TUNNEL_CENTER.x + wp.dx,
        z: GATE_COURSE_TUNNEL_CENTER.z + wp.dz,
        y: wp.elevation || 0,
        yaw: yaw
    };
});
function buildGateCourseTunnel() {
    const group = new THREE.Group();
    GATE_PLACEMENTS_TUNNEL.forEach(function (placement) {
        const wp = placement.wp;
        // buildGroundedGate (IKKE buildGate/buildStartFinishGate) - hele denne banen klatrer i åpen luft
        // over stigende terreng eller står tett inntil fjellsiden, så en fast beinlengde (som antar flatt
        // terreng under selve porten) ville latt porten sveve med et synlig gap ned til bakken/fjellsiden -
        // se buildGroundedGate for hvordan beina strekkes til EKTE terrenghøyde i stedet.
        const obstacle = buildGroundedGate(wp, placement);
        obstacle.position.set(placement.x, placement.y, placement.z);
        obstacle.rotation.y = placement.yaw;
        group.add(obstacle);
    });
    group.add(buildSummitMountain());
    return group;
}
// Portenes fangstsoner for selve tidtakingen (se isDroneInGateOpening/updateRacingStage) - samme
// 0.92-marginprinsipp/RACE_GATE_DEPTH_TOLERANCE som RACE_GATE_CENTERS_2. Ingen barn-/roofgate-type her
// (kun "start"/"gate"), så oppslaget er enklere enn course 2 sitt.
const RACE_GATE_CENTERS_TUNNEL = GATE_PLACEMENTS_TUNNEL.map(function (placement) {
    const wp = placement.wp;
    return {
        x: placement.x, y: placement.y + wp.gap + wp.size / 2, z: placement.z,
        yaw: placement.yaw,
        halfW: wp.size / 2 * 0.92, halfH: wp.size / 2 * 0.92
    };
});
const RACE_TUNNEL_START_PLACEMENT = GATE_PLACEMENTS_TUNNEL[0];
const RACE_SPAWN_BACK_DIST_TUNNEL = 20; // samme avstand/begrunnelse som RACE_SPAWN_BACK_DIST (course 2)
const RACE_SPAWN_POINT_TUNNEL = new THREE.Vector3(
    RACE_TUNNEL_START_PLACEMENT.x - Math.sin(RACE_TUNNEL_START_PLACEMENT.yaw) * RACE_SPAWN_BACK_DIST_TUNNEL,
    0,
    RACE_TUNNEL_START_PLACEMENT.z - Math.cos(RACE_TUNNEL_START_PLACEMENT.yaw) * RACE_SPAWN_BACK_DIST_TUNNEL
);
const RACE_SPAWN_YAW_TUNNEL = RACE_TUNNEL_START_PLACEMENT.yaw + Math.PI;

// Per-bane "ressurser" (portliste + spawn) for racing-stagene, slått opp på stage.assetsKey ved selve
// KALLTIDSPUNKTET (inne i spawnRacingStage/updateRacingStage/racingGatesForStage, som kjører lenge etter
// hele filen er lastet) - IKKE lagret direkte på EXERCISES-objektets stage-definisjoner. EXERCISES
// evalueres STRAKS når det selv defineres, tidlig i filen, lenge FØR RACE_GATE_CENTERS_TUNNEL/
// RACE_SPAWN_POINT_TUNNEL over i det hele tatt eksisterer - en direkte referanse fra en stage-definisjon
// til dem ville feilet med en "kan ikke aksessere før initialisering"-feil. race1/race3 har ingen
// assetsKey og faller derfor tilbake til RACE_GATE_CENTERS_2/RACE_SPAWN_POINT (se disse funksjonenes egne
// fallback-verdier når oppslaget her gir undefined).
const RACING_STAGE_ASSETS = {
    raceTunnel: { gates: RACE_GATE_CENTERS_TUNNEL, spawnPoint: RACE_SPAWN_POINT_TUNNEL, spawnYaw: RACE_SPAWN_YAW_TUNNEL }
};
function racingGatesForStage(stage) {
    const assets = RACING_STAGE_ASSETS[stage.assetsKey];
    return (assets && assets.gates) || RACE_GATE_CENTERS_2;
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
/* ---------- Propell-treffbokser for "Til topps" (tunnelbanen) - se PROP_HAZARDS-kommentaren for
   prinsippet. Samme ring-geometri som PROP_HAZARDS/PROP_HAZARDS_2 sin egen (ikke-låve) gren - ingen
   "barn"-type porter her - men med placement.y-løftet PROP_HAZARDS_2 allerede innførte for rådhustak-
   gaten, siden STORE deler av denne banen (klatringen i fjellsiden, munningsportene) ikke står på
   bakkeplan. Manglet helt før nå - brukerens krav: "pass på att alle gates har kollisjonsdeteksjon" -
   tunnelbanens porter hadde ingen treffbokser i det hele tatt, verken for propell eller noe annet. */
const PROP_HAZARDS_TUNNEL = GATE_PLACEMENTS_TUNNEL.map(function (placement) {
    const wp = placement.wp;
    const s = wp.size, gap = wp.gap, t = 0.09; // halv bar-tykkelse (0.18/2)
    const hs = s / 2;
    const boxes = [
        { minX: -hs, maxX: hs, minY: gap - t, maxY: gap + t, minZ: -t, maxZ: t },           // nedre bar
        { minX: -hs, maxX: hs, minY: gap + s - t, maxY: gap + s + t, minZ: -t, maxZ: t },   // øvre bar
        { minX: -hs - t, maxX: -hs + t, minY: 0, maxY: gap + s, minZ: -t, maxZ: t },        // venstre stolpe + ben
        { minX: hs - t, maxX: hs + t, minY: 0, maxY: gap + s, minZ: -t, maxZ: t }           // høyre stolpe + ben
    ];
    const yOffset = placement.y || 0;
    boxes.forEach(function (b) { b.minY += yOffset; b.maxY += yOffset; });
    return {
        x: placement.x, z: placement.z,
        cosYaw: Math.cos(placement.yaw), sinYaw: Math.sin(placement.yaw),
        boundRSq: (hs + 0.2 + 1.5) * (hs + 0.2 + 1.5),
        maxY: gap + s + 0.2 + yOffset + 0.5,
        boxes: boxes
    };
});
const ALL_PROP_HAZARDS = PROP_HAZARDS.concat(PROP_HAZARDS_2, PROP_HAZARDS_TUNNEL);

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
    // +90° fra før - buildCar sin lengdeakse ble snudd fra lokal X til lokal Z (se kommentaren der), denne
    // justeringen holder akkurat samme synlige verdensvinkel som før ombyggingen.
    car.rotation.y = THREE.MathUtils.degToRad(20 + 90);
    group.add(car);

    const crowd = buildCrowd();
    crowd.position.copy(CROWD_CENTER);
    group.add(crowd);

    const building = buildBuilding();
    building.position.set(-35, 0, -35);
    group.add(building);

    DECORATIVE_TREES.forEach(function (t) {
        // Seed fra treets egen (faste) posisjon - se buildRandomTree sin egen kommentar.
        const tree = Sim.buildRandomTree(t.h, t.x * 7.13 + t.z * 3.71);
        tree.position.set(t.x, 0, t.z);
        group.add(treeSwayManager.addSwayingTree(tree));
    });

    group.add(buildGateCourse());
    group.add(buildGateCourse2());
    group.add(buildGateCourseTunnel());

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

// buildRemoteController flyttet til js/simulator-common.js (Sim.buildRemoteController) - VTOL-
// simulatoren trenger nøyaktig samme figur, se kallstedet under (vlosPerson).

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
    // logarithmicDepthBuffer: brukeren rapporterte flimring i dybdebufferet ("flimring når jeg ser elva på
    // avstand") - med near/far så langt fra hverandre (0.1/2000, se kameraene under) er en vanlig,
    // lineær dybdebuffer svært unøyaktig langt unna kameraet (mesteparten av presisjonen brukes opp nær
    // kameraet), så to flater som er tydelig atskilt på nært hold (f.eks. elva, RIVER_Y=0.12 over bakken)
    // kan likevel z-fighte synlig på avstand. Logaritmisk dybdebuffer fordeler presisjonen jevnere over
    // hele near-far-spennet i stedet - standard motorfiks for nettopp denne typen avstandsflimring.
    renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true, logarithmicDepthBuffer: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    // Å flytte vinduet mellom skjermer kan (spesielt på maskiner med to grafikkort, der skjermene
    // drives av ulik GPU) trigge et WebGL-kontekst-tap midt i rendring - uten disse to lytterne
    // fortsetter animate()-løkken å kjøre og TEGNE, men mot en død/tom kontekst, som viser seg som et
    // fastfrosset eller helt feil bilde (rapportert: ser "zoomet inn" ut) helt til siden lastes på
    // nytt. preventDefault() på "lost" ber nettleseren faktisk FORSØKE gjenoppretting i stedet for å
    // gi opp permanent. Et gjenopprettet kontekst har mistet ALLE GPU-ressurser (shaders/buffere/
    // teksturer) - i stedet for å prøve å bygge hele scenen på nytt for hånd (stor flate å bomme på),
    // reloader vi rett og slett siden, som starter helt rent - akkurat det brukeren uansett må gjøre
    // manuelt i dag.
    canvas.addEventListener("webglcontextlost", function (e) {
        e.preventDefault();
        console.warn("[FFI-UAS] WebGL-kontekst tapt (f.eks. GPU-bytte ved flytting mellom skjermer) - laster siden på nytt...");
    }, false);
    canvas.addEventListener("webglcontextrestored", function () {
        location.reload();
    }, false);

    scene = new THREE.Scene();
    // radius 1400 (opp fra Sim.buildGradientSky sin egen default 800, se dens kommentar) - kun for
    // quad-simulatoren (fixedwing/vtol kaller uten argument, uendret) - se fjellkjede-kommentaren ved
    // MOUNTAIN_DEFS for hele begrunnelsen (brukerens rapport om svarte hull i himmelen ved fjellene).
    skyMesh = Sim.buildGradientSky(1400);
    scene.add(skyMesh);
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

    droneShadowDecal = buildDroneShadowDecal();
    scene.add(droneShadowDecal);

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
    const controller = Sim.buildRemoteController();
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

    // Mål for targetStrike - egne håndtak, bygget/skjult på samme måte, se buildTargetDrone/buildCar/
    // Sim.buildPersonFigure-bruken i "Øvelser: mål-i-bevegelse"-seksjonen. buildTargetDrone() setter
    // group.visible=false selv (i motsetning til buildCar/buildPedestrianGroup, som ikke gjør det - derfor
    // eksplisitt satt for de to under) - denne linja er dermed strengt tatt overflødig, men holdt for
    // symmetri/lesbarhet med de to andre håndtakene rett under.
    targetDroneHandle = buildTargetDrone();
    targetDroneHandle.visible = false;
    scene.add(targetDroneHandle);
    // Grønn/kamuflasjefarget (brukerens krav) - IKKE standardbilens røde karosseri/mørkeblå kabin (se
    // buildCar-kommentaren), skiller også målet visuelt fra den statiske, røde bilen ved avgangsplassen.
    targetCarHandle = buildCar(0x4b5320, 0x39431c);
    targetCarHandle.visible = false;
    scene.add(targetCarHandle);
    // buildTargetRunner (IKKE buildPedestrianGroup, som bygger et PAR - riktig for killswitch sitt
    // "to fotgjengere"-scenario, men ga FEILAKTIG to personer her, se funksjonens egen kommentar,
    // brukerens rapport: "Personen i skogen er nå to personer. Det skal bare være en."). Grønn/
    // kamuflasjefarget vest (brukerens krav) - IKKE de blå/røde standardfargene, godt synlige "publikum i
    // fare"-farger som ville vært misvisende for et mål som faktisk SKAL være vanskelig å få øye på i skogen.
    targetRunnerHandle = buildTargetRunner(0x4b5320);
    scene.add(targetRunnerHandle);
    // Fjerde mål - fastvinge-fly (brukerens krav: "Legg til enda en oppgave om å krasje i en fixed wing
    // drone"), se buildTargetFixedWing.
    targetFixedWingHandle = buildTargetFixedWing();
    targetFixedWingHandle.visible = false;
    scene.add(targetFixedWingHandle);

    activeCamera = chaseCamera;
    resizeRenderer();
    viewportWatcher = Sim.createViewportWatcher(renderer, document.querySelector(".sim-page"), resizeRenderer);
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

// modeFlashUntil: brukt av updateHud til å legge på ".mode-flash"-CSS-klassen (kort gult blink, se
// style.css) på hudMode et lite øyeblikk hver gang modusen FAKTISK endres via setFlightMode - samme
// mønster som VTOL-simmen (js/simulator-vtol.js sin trySetFlightMode). Kun satt ved et EKTE bytte
// (mode !== droneState.flightMode), ikke ved f.eks. gjentatte trykk på samme tast/knapp.
let modeFlashUntil = 0;
function setFlightMode(mode) {
    // Øvelser med exercise.forceFlightMode (Acro-øvelsene: race1/race3/raceTunnel/targetStrike) tvinger
    // EN bestemt modus for hele øvelsens varighet (satt ved spawn, se spawnForExercise) - uten denne
    // vakten kunne piloten likevel bytte vekk fra Acro midt i et forsøk (tastatur 1-4/gamepad/HUD-
    // nedtrekket går alle via denne ene funksjonen, se BUTTON_ACTIONS/keydown-håndteringen), noe som ville
    // gitt en helt annen (og lettere) flyoppførsel enn øvelsen faktisk er kalibrert/tidtatt for. Brukerens
    // krav: "pass på at det ikke er mulig å bytte modus vekk fra acro så lenge en øvelse er aktiv."
    const exercise = exerciseState.active ? EXERCISES[exerciseState.exerciseId] : null;
    if (exercise && exercise.forceFlightMode && mode !== exercise.forceFlightMode) {
        exerciseState.warningMessage = "Denne øvelsen krever " + MODE_LABELS[exercise.forceFlightMode] + "-modus.";
        exerciseState.warningUntil = performance.now() + 2000;
        exerciseState.warningIsSuccess = false;
        return;
    }
    if (mode !== droneState.flightMode) modeFlashUntil = performance.now() + 400;
    droneState.flightMode = mode;
}

/* ---------- Gamepad knappemapping (kill/arm + flymodus-brytere) ---------- */
// Se Sim.createButtonBindingManager i simulator-common.js for læringsflyten (bryter kan komme
// som HID-knapp eller som en akse - fungerer med enhver sender i USB-joystick-modus).
// kill er IKKE med her (i motsetning til før) - den skal følge selve BRYTERPOSISJONEN kontinuerlig
// (se kill-håndteringen i updateInput), ikke trigges som et engangs-toggle på stigende kant slik
// resten av actionsMap gjør. Gjelder både én enkelt bryter og en kombinasjon av flere - se
// gamepadMap.buttons.kill/isBindingActive.
const BUTTON_ACTIONS = {
    modeAcro: function () { setFlightMode("acro"); },
    modeStabilized: function () { setFlightMode("stabilized"); },
    modeAltHold: function () { setFlightMode("althold"); },
    modeLoiter: function () { setFlightMode("loiter"); },
    // handleResetRequest er en function-DECLARATION lenger ned i filen - fullt hoistet, så referansen
    // her (i en funksjonskropp som først kjører når bryteren faktisk trigges) er trygg selv om den
    // tekstuelt står før definisjonen. Samme funksjon som R-tasten kaller.
    reset: function () { handleResetRequest(); }
};
const buttonManager = Sim.createButtonBindingManager(gamepadMap.buttons, BUTTON_ACTIONS, saveGamepadMap);
// Se Sim.createAxisCalibrationManager - fanger opp maks utslag per kanal over et kort tidsvindu
// ("Kalibrer fullt utslag" i fjernkontroll-panelet, se buildGamepadPanel) for sendere som ikke
// rapporterer ±1.0 ved fysisk fullt utslag.
const axisCalibrationManager = Sim.createAxisCalibrationManager(gamepadMap, ["throttle", "roll", "pitch", "yaw"], saveGamepadMap);

function updateInput(dt) {
    updateLinkAndBattery(dt);

    const gp = getActiveGamepad();
    if (gp) buttonManager.poll(gp);
    if (gp) axisCalibrationManager.poll(gp);
    // Kill følger BRYTERPOSISJONEN direkte og kontinuerlig - IKKE et toggle du trigger med et trykk
    // (se BUTTON_ACTIONS-kommentaren over). armed speiler ganske enkelt "er bryteren/kombinasjonen i
    // PÅ-stillingen akkurat nå" hvert eneste bilde, begge veier: står den i AV, er droneen armert;
    // vippes den til PÅ, killes den momentant - og vippes den tilbake til AV, armeres den momentant
    // igjen, akkurat som en ekte fysisk bryter. Gjelder likt for én enkelt bryter og en kombinasjon av
    // flere (se isBindingActive sin "combo"-håndtering - AND av alle delene, må alle stå i PÅ samtidig).
    // Samme unntak som toggleKill (krasjet/skadet/nettopp bestått øvelse) - se der.
    const killBinding = gamepadMap.buttons.kill;
    if (gp && killBinding && !droneState.crashed && !droneState.injured && !exerciseState.awaitingNext) {
        droneState.armed = !Sim.isBindingActive(gp, killBinding);
    }

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
    // Live "output"-prikk i Rates-panelet (se buildRatesPanel/Sim.buildRateAxisBox) - leser samme
    // inputState.stick som fysikken selv bruker, rett FØR et ev. killswitch-override under.
    updateRatesPanelLive();
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
        // BUG (rapportert av brukeren: "helt forskjellig respons i acro med samme rates" - viste seg i
        // flightloggen som harde, ukontrollerte kast akkurat mens pinneT sto i 100%) - Airmode sitt eget
        // panel-hint lover UTTRYKKELIG "beholder kontroll uansett gassnivå ... nær 0%/100% gass", men
        // koden løftet hele settet OPP mot gulvet (over) og glemte det HELT symmetriske tilfellet: nær
        // 100% gass er det den samme "positive" motoren (fremre-høyre/bakre-venstre) som klipper mot
        // TAKET på 1.0 i stedet for gulvet, og differensialen forsvinner akkurat like mye der - bare i
        // motsatt retning. Uten denne andre halvparten var Airmode reelt sett VIRKNINGSLØS nær full gass
        // (samme klipping som HELT UTEN Airmode), som er nøyaktig scenarioet harde acro-manøvre
        // (punch-outs, flips) skjer i. Speiler løftet over: senk hele settet samlet så høyeste motor
        // akkurat treffer taket, IKKE klipp hver for seg.
        const maxVal = Math.max(raw[0], raw[1], raw[2], raw[3]);
        if (maxVal > 1) {
            const offset = maxVal - 1;
            for (let i = 0; i < raw.length; i++) raw[i] -= offset;
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
            desiredRateDeg.yaw = computeRate(stick.yaw, effectiveYawRates());
            thrustForce = throttleShaped * spec.maxThrust;
        } else {
            // Stabilized / Alt Hold / Loiter: selvnivellerende ytre lookk for roll/pitch.
            // (euler.x/euler.z negeres - se merknad om aksekonvensjon lenger ned.)
            const euler = new THREE.Euler().setFromQuaternion(droneState.quaternion, "YXZ");
            const currentPitchDeg = -THREE.MathUtils.radToDeg(euler.x);
            const currentRollDeg = -THREE.MathUtils.radToDeg(euler.z);
            let desiredPitchAngle, desiredRollAngle;
            if (droneState.flightMode === "loiter") {
                // Loiter: pinnen kommanderer en ØNSKET horisontal fart (kropp-relativt forover/sideveis)
                // i stedet for en krengevinkel direkte - se LOITER_MAX_SPEED-kommentaren ved konstanten.
                // bodyForwardFlat/bodyRightFlat er nesens/høyre-siden sin retning FLATET til
                // horisontalplanet (ikke en full 3D-projeksjon via quaternion-invertering, som ville
                // blandet inn gjeldende krengning/stigning i selve fartsMÅLINGEN).
                const bodyForwardFlat = new THREE.Vector3(0, 0, -1).applyQuaternion(droneState.quaternion);
                bodyForwardFlat.y = 0;
                if (bodyForwardFlat.lengthSq() < 1e-6) bodyForwardFlat.set(0, 0, -1); else bodyForwardFlat.normalize();
                const bodyRightFlat = new THREE.Vector3(1, 0, 0).applyQuaternion(droneState.quaternion);
                bodyRightFlat.y = 0;
                if (bodyRightFlat.lengthSq() < 1e-6) bodyRightFlat.set(1, 0, 0); else bodyRightFlat.normalize();

                // droneState.velocity er den FAKTISKE bakkefarten (inkluderer allerede vindavdrift, se
                // vind-håndteringen lenger ned i denne funksjonen) - fartsFEILEN loopen regulerer på
                // inneholder derfor automatisk en vindkorreksjon, uten noen egen vind-spesifikk term.
                const groundVelFlat = new THREE.Vector3(droneState.velocity.x, 0, droneState.velocity.z);
                const fwdSpeed = groundVelFlat.dot(bodyForwardFlat);
                const rightSpeed = groundVelFlat.dot(bodyRightFlat);

                // "Nettopp entret Loiter" (samme flagg som fartsfilteret under bruker) - holdepunktet OG
                // fartsfilteret må initialiseres til NÅVÆRENDE posisjon/fart her, første tick, ellers ville
                // de stått igjen på (0,0,0)/null fra objekt-opprettelsen og droneen prøvd å kjøre dit,
                // eller filteret rukket å "rulle inn" fra 0 i stedet for å starte på faktisk fart.
                if (droneState.loiterVelFwdFilt === null) {
                    droneState.loiterTargetPos.set(droneState.position.x, 0, droneState.position.z);
                    droneState.loiterVelFwdFilt = fwdSpeed;
                    droneState.loiterVelRightFilt = rightSpeed;
                }

                let desiredFwdSpeed, desiredRightSpeed;
                if (Math.hypot(stick.pitch, stick.roll) > LOITER_STICK_DEADBAND) {
                    // FASE 1 - Flyging: piloten styrer aktivt, direkte fartskommando fra pinnen. Holdepunktet
                    // flyttes KONTINUERLIG med droneen mens dette skjer, og HOLDING-tilstanden nullstilles -
                    // et senere slipp starter alltid med en frisk bremsefase (fase 2), ikke et gammelt lås.
                    desiredFwdSpeed = stick.pitch * LOITER_MAX_SPEED;
                    desiredRightSpeed = stick.roll * LOITER_MAX_SPEED;
                    droneState.loiterTargetPos.set(droneState.position.x, 0, droneState.position.z);
                    droneState.loiterHolding = false;
                    droneState.loiterPhase = "flying";
                    // BUG rettet (se flightlogg brukeren limte inn - "litt treg å bremse opp?"): I-leddet er
                    // KUN ment som vindkorreksjon mens droneen faktisk står stille (fase 3, se kommentaren
                    // der) - under aktiv flyging (her) holder fwdError seg gjerne POSITIV i flere sekunder
                    // mens farten bygger seg opp mot pinnens kommanderte marsjfart, og I-leddet hopet seg
                    // derfor opp med en stor, positiv "fremover"-bias. Ble den STÅENDE ved et senere slipp
                    // (fase 2), kjempet den stale bias-en MOT den nye, sterkt NEGATIVE bremsekommandoen i
                    // flere sekunder før den rakk å "vikle seg ut" - selve bremsingen føltes treg selv om
                    // P-leddet i seg selv kommanderte en bratt vinkel med det samme. Nullstilles derfor her
                    // OGSÅ (ikke bare når Loiter forlates helt), slik at bremsingen alltid starter fra en
                    // ren, umiddelbart korrekt I-tilstand.
                    droneState.loiterIntegralFwd = 0;
                    droneState.loiterIntegralRight = 0;
                } else if (!droneState.loiterHolding && groundVelFlat.length() > LOITER_TARGET_LOCK_SPEED) {
                    // FASE 2 - Bremsing: pinnen sluppet, men farten fra flygingen henger fortsatt igjen -
                    // ØNSKET fart er rett og slett 0 (ren oppbremsing, se fart-P-I-D-loopen under). INGEN
                    // posisjonsledd her ennå, og holdepunktet fortsetter å flyttes med droneen (samme linje
                    // som fase 1) - droneen bygger derfor ALDRI opp fart i motsatt retning for å jage et
                    // gammelt punkt bak seg, den bare bremser rett ned der den er. I-leddet holdes OGSÅ 0 her
                    // (se fase 1-kommentaren over) - selve oppbremsingen skal være P(+D)-leddets jobb alene;
                    // I-leddet kobles først inn når droneen faktisk har stanset (fase 3).
                    desiredFwdSpeed = 0;
                    desiredRightSpeed = 0;
                    droneState.loiterTargetPos.set(droneState.position.x, 0, droneState.position.z);
                    droneState.loiterPhase = "braking";
                    droneState.loiterIntegralFwd = 0;
                    droneState.loiterIntegralRight = 0;
                } else {
                    // FASE 3 - Holding: droneen har reelt stanset (under LOITER_TARGET_LOCK_SPEED) - HER, og
                    // først her, får holdepunktet lov til å stå FAST (oppdateres ikke lenger over). Selve
                    // OVERGANGEN inn i denne fasen (loiterHolding blir true) skjer bare ÉN gang; den blir
                    // værende true til piloten flyr aktivt igjen (fase 1) - en forbigående fartsøkning fra et
                    // vindkast MIDT i holdefasen slår den altså ikke tilbake til fase 2 og "glemmer"
                    // holdepunktet, se LOITER_TARGET_LOCK_SPEED-kommentaren. Posisjons-P-leddet ("Position XY
                    // (Dist to Speed)" i ArduCopter) retter opp avviket fra akkurat DER den stanset.
                    droneState.loiterHolding = true;
                    droneState.loiterPhase = "holding";
                    const toTarget = new THREE.Vector3().subVectors(droneState.loiterTargetPos,
                        new THREE.Vector3(droneState.position.x, 0, droneState.position.z));
                    const desiredWorldSpeed = toTarget.multiplyScalar(LOITER_POS_P_GAIN);
                    if (desiredWorldSpeed.length() > LOITER_MAX_SPEED) desiredWorldSpeed.setLength(LOITER_MAX_SPEED);
                    desiredFwdSpeed = desiredWorldSpeed.dot(bodyForwardFlat);
                    desiredRightSpeed = desiredWorldSpeed.dot(bodyRightFlat);
                }
                // Lavpassfiltrer selve fartsMÅLINGEN (se LOITER_VEL_FILTER_ALPHA-kommentaren) FØR den brukes
                // noe sted under - fjerner tick-til-tick-støy ved KILDEN i stedet for å lappe hvert enkelt
                // ledd som bruker den. prevFiltered fanges FØR selve oppdateringen, til bruk i D-leddets
                // deriverte lenger ned.
                const prevFilteredFwd = droneState.loiterVelFwdFilt;
                const prevFilteredRight = droneState.loiterVelRightFilt;
                droneState.loiterVelFwdFilt += (fwdSpeed - droneState.loiterVelFwdFilt) * LOITER_VEL_FILTER_ALPHA;
                droneState.loiterVelRightFilt += (rightSpeed - droneState.loiterVelRightFilt) * LOITER_VEL_FILTER_ALPHA;
                // Samme fortegn som den direkte pinne->vinkel-kommandoen under (stick.pitch/roll ->
                // desiredPitchAngle/-RollAngle direkte) - dette ER akkurat den kommandoen, bare med et
                // fartsAVVIK (nå av den FILTRERTE farten) i stedet for selve pinneposisjonen som P-leddets
                // inngang.
                const fwdError = desiredFwdSpeed - droneState.loiterVelFwdFilt;
                const rightError = desiredRightSpeed - droneState.loiterVelRightFilt;
                // I-ledd (se LOITER_WIND_I_GAIN-kommentaren ved konstanten): bygger sakte opp en egen
                // vinkel-bias fra den VEDVARENDE fartsfeilen, slik at en konstant vind til slutt kan
                // motvirkes helt uten at posisjonen selv trenger å drifte for å holde P-leddet aktivt.
                droneState.loiterIntegralFwd = clamp(
                    droneState.loiterIntegralFwd + fwdError * LOITER_WIND_I_GAIN * dt,
                    -LOITER_WIND_I_MAX_DEG, LOITER_WIND_I_MAX_DEG
                );
                droneState.loiterIntegralRight = clamp(
                    droneState.loiterIntegralRight + rightError * LOITER_WIND_I_GAIN * dt,
                    -LOITER_WIND_I_MAX_DEG, LOITER_WIND_I_MAX_DEG
                );
                // D-ledd (se LOITER_VEL_D_GAIN-kommentaren): derivert på den FILTRERTE farten (ikke feilen,
                // som ville gitt et "derivative kick" hver gang stick.pitch/roll hopper, og ikke den rå
                // fwdSpeed/rightSpeed - se LOITER_VEL_FILTER_ALPHA-kommentaren for hvorfor).
                const fwdAccel = (droneState.loiterVelFwdFilt - prevFilteredFwd) / dt;
                const rightAccel = (droneState.loiterVelRightFilt - prevFilteredRight) / dt;
                const rawPitchTarget = clamp(fwdError * LOITER_VEL_TO_LEAN_DEG + droneState.loiterIntegralFwd - fwdAccel * LOITER_VEL_D_GAIN, -LOITER_MAX_LEAN_ANGLE, LOITER_MAX_LEAN_ANGLE);
                const rawRollTarget = clamp(rightError * LOITER_VEL_TO_LEAN_DEG + droneState.loiterIntegralRight - rightAccel * LOITER_VEL_D_GAIN, -LOITER_MAX_LEAN_ANGLE, LOITER_MAX_LEAN_ANGLE);
                // Fartsbegrens selve UTSLAGET (se LOITER_MAX_ANGLE_RATE) - P+I+D over kan fortsatt regne ut
                // et stort momentant behov, men den kommanderte vinkelen ruller jevnt dit i stedet for å
                // hoppe på én tick. loiterCmdPitchAngle === null (nettopp entret Loiter) hopper rett til
                // målet første tick i stedet for å rulle fra en gammel/urelatert verdi.
                const maxAngleStep = LOITER_MAX_ANGLE_RATE * dt;
                droneState.loiterCmdPitchAngle = droneState.loiterCmdPitchAngle === null ? rawPitchTarget :
                    droneState.loiterCmdPitchAngle + clamp(rawPitchTarget - droneState.loiterCmdPitchAngle, -maxAngleStep, maxAngleStep);
                droneState.loiterCmdRollAngle = droneState.loiterCmdRollAngle === null ? rawRollTarget :
                    droneState.loiterCmdRollAngle + clamp(rawRollTarget - droneState.loiterCmdRollAngle, -maxAngleStep, maxAngleStep);
                desiredPitchAngle = droneState.loiterCmdPitchAngle;
                desiredRollAngle = droneState.loiterCmdRollAngle;
            } else {
                desiredPitchAngle = stick.pitch * MAX_SELF_LEVEL_ANGLE;
                desiredRollAngle = stick.roll * MAX_SELF_LEVEL_ANGLE;
                // Nullstill Loiter sitt I-ledd og D-leddets fart-historikk så lenge vi IKKE er i Loiter
                // (anti-windup - unngår at et gammelt, opphopet bidrag fra en TIDLIGERE Loiter-økt påvirker
                // de aller første tickene neste gang Loiter velges igjen), se samme mønster i VTOL-simmens
                // fbwbClimbIntegral.
                droneState.loiterIntegralFwd = 0;
                droneState.loiterIntegralRight = 0;
                droneState.loiterVelFwdFilt = null;
                droneState.loiterVelRightFilt = null;
                droneState.loiterHolding = false;
                droneState.loiterCmdPitchAngle = null;
                droneState.loiterCmdRollAngle = null;
                droneState.loiterPhase = "-";
            }
            desiredRateDeg.pitch = ANGLE_P_GAIN * (desiredPitchAngle - currentPitchDeg);
            desiredRateDeg.roll = ANGLE_P_GAIN * (desiredRollAngle - currentRollDeg);
            desiredRateDeg.yaw = computeRate(stick.yaw, effectiveYawRates());

            if (droneState.flightMode === "althold" || droneState.flightMode === "loiter") {
                // Gass rundt 50% (innenfor dødsone) holder høyden; utenfor justeres ønsket stigefart
                // proporsjonalt - samme Alt Hold-kollektiv som Alt Hold-modus (Loiter er Alt Hold +
                // posisjonsholding, se desiredPitchAngle/-RollAngle over).
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
        const yawNorm = axisTorqueNorm(effectiveYawRates());
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

        // Yaw-spesifikk bakkefriksjon (se GROUND_YAW_UNLOCK_THRUST_FRAC_*-kommentaren): så lenge vekten
        // hviler på beina er friksjonen der langt sterkere enn yaw-reaksjonsmomentet, uansett hvor hardt
        // yaw-pinnen holdes inne. baseCmd (pilotens ØNSKEDE gass, FØR mikseren fordeler den ujevnt på
        // motorene) avgjør hvor mye av vekten som faktisk er lettet av beina - IKKE motorValues/den
        // oppnådde thrustForce under, som en aggressiv yaw-kommando alene kan blåse opp uten at piloten
        // har bedt om noe løft (se mixMotors: to av fire motorer kan nå 50% fra yaw alene ved 0% gass).
        let groundYawAuthority = 1;
        if (groundContactBlend > 0) {
            const desiredThrustFrac = (baseCmd * spec.maxThrust) / (spec.mass * GRAVITY);
            const unlockT = clamp(
                (desiredThrustFrac - GROUND_YAW_UNLOCK_THRUST_FRAC_START) /
                (GROUND_YAW_UNLOCK_THRUST_FRAC_FULL - GROUND_YAW_UNLOCK_THRUST_FRAC_START), 0, 1);
            groundYawAuthority = 1 - groundContactBlend * (1 - unlockT);
        }

        // Vinkelakselerasjon = moment / treghet: tyngre/større droner (høyere treghet) responderer
        // tregere per akse, og yaw har alltid høyere treghet enn roll/pitch (som på en ekte quad).
        const pitchAccel = extractMixedAxis(motorValues, "pitch") * pitchNorm / spec.inertiaRollPitch;
        const rollAccel = extractMixedAxis(motorValues, "roll") * rollNorm / spec.inertiaRollPitch;
        const yawAccel = extractMixedAxis(motorValues, "yaw") * yawNorm / spec.inertiaYaw;
        droneState.angularVelocity.pitch += pitchAccel * controlAuthority * dt;
        droneState.angularVelocity.roll += rollAccel * controlAuthority * dt;
        droneState.angularVelocity.yaw += yawAccel * controlAuthority * groundYawAuthority * dt;

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
    // logFlightSample defineres i js/simulator-flightlog.js, lastet ETTER denne filen - samme "forover-
    // referanse, løses ved kall-tidspunkt"-mønster som VTOL-simmens egen flightlogg-integrasjon.
    logFlightSample(dt);
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
        // f.y som atY (se mountainHeightAt sin egen kommentar) - dette ER kontaktpunktets faktiske,
        // nåværende høyde, akkurat den referansen tunnel-hulrom-sjekken trenger for ikke å blindt svare
        // "korridorgulvet" for et punkt som egentlig befinner seg langt over selve taket (f.eks. oppe på
        // toppflaten, se buildSummitRocks).
        const groundY = Math.max(mountainHeightAt(f.x, f.z, f.y), colliderTop);
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
    if (p.y <= mountainHeightAt(p.x, p.z, p.y) + PROP_GROUND_STRIKE_EPS) return true;
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
            knockPersonOver(crowdMembers[i]); // "personen må falle over i naturlig retning" (brukeren) - se knockPersonOver
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
            knockPersonOver(pedestrianHandle);
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
    droneState.loiterIntegralFwd = 0;
    droneState.loiterIntegralRight = 0;
    droneState.loiterVelFwdFilt = null;
    droneState.loiterVelRightFilt = null;
    droneState.loiterHolding = false;
    droneState.loiterCmdPitchAngle = null;
    droneState.loiterCmdRollAngle = null;
    groundContactBlend = 0;
    resetPersonFalls(); // "de kan falle over ende og bli liggende" - reis dem opp igjen ved reset, samme prinsipp som VTOL-simulatorens resetPlane()
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

// Myk, alltid synlig "blob"-skygge rett under droneen - reell shadow-mapping (sunLight.shadow, se
// initScene) alene var for svak/vanskelig å se i lav høyde og på selve landingsplattformen (brukeren:
// "quadens skygge mot bakken må være tydeligere i lav høyde. skyggen må være synlig på landingspadden
// også"). Tegnes ALLTID rett under droneen på den FAKTISKE overflaten der (solidSurfaceHeightAt - bakke,
// fjell ELLER landingsplattformen/tak/bro, samme funksjon fysikken selv bruker til bakkekontakt),
// uavhengig av shadow map-oppløsning/bias/frustum - garantert synlig posisjons-/høydereferanse i stedet
// for et supplement som kan svikte akkurat der man trenger det mest.
let droneShadowTextureBase = null;
function buildDroneShadowTexture() {
    if (droneShadowTextureBase) return droneShadowTextureBase;
    const size = 128;
    const canvas = document.createElement("canvas");
    canvas.width = size;
    canvas.height = size;
    const ctx = canvas.getContext("2d");
    // Myk radial gradient (mørkest i sentrum, helt gjennomsiktig i ytterkanten) - IKKE en hardkantet
    // sirkel, som ville sett ut som en klistrelapp klistret på bakken i stedet for en skygge.
    const grad = ctx.createRadialGradient(size / 2, size / 2, 0, size / 2, size / 2, size / 2);
    grad.addColorStop(0, "rgba(0,0,0,0.9)");
    grad.addColorStop(0.55, "rgba(0,0,0,0.6)");
    grad.addColorStop(1, "rgba(0,0,0,0)");
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, size, size);
    droneShadowTextureBase = new THREE.CanvasTexture(canvas);
    return droneShadowTextureBase;
}
let droneShadowDecal;
function buildDroneShadowDecal() {
    // MeshBasicMaterial (upåvirket av scenens lys) - en forutsigbar, jevnt mørk skygge uansett solvinkel/
    // skydekke, i stedet for å arve variasjonen den ekte shadow map-en allerede har (og som var problemet).
    // depthWrite:false + renderOrder ETTER bakken/plattformen den ligger 3 cm over unngår z-fighting-
    // flimring mot akkurat DEN flaten (samme grunnleggende problem som klokkeskivens polygonOffset-fiks,
    // se buildClockFace - her løst med render-rekkefølge i stedet siden dette er en helt separat,
    // frittstående flate, ikke geometri limt tett inntil en annen mesh).
    const mat = new THREE.MeshBasicMaterial({ map: buildDroneShadowTexture(), transparent: true, depthWrite: false });
    const mesh = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), mat);
    mesh.rotation.x = -Math.PI / 2;
    mesh.renderOrder = 1;
    return mesh;
}
// Høyde (m over bakken/plattformen der droneen står) skyggen er helt falmet bort - over dette gir den
// ingen posisjonsinformasjon uansett (droneen er for langt unna til at "rett under" er nyttig å vite).
// Senket fra 45 (brukeren: "skyggen ... henger litt for lenge igjen når drone øker høyden. må forsvinne
// litt tidligere med høyden") - falmer nå helt bort innen 26 m i stedet for 45.
const DRONE_SHADOW_MAX_ALT = 26;
const DRONE_SHADOW_MIN_OPACITY = 0, DRONE_SHADOW_MAX_OPACITY = 0.82;
function updateDroneShadowDecal() {
    // Klemt til ALDRI å overstige droneens egen, faktiske høyde - en skygge kan per definisjon ikke ligge
    // OVER det den kastes fra. Rent forsvarsverk (brukerens gjentatte rapport: "stor svart sirkel på
    // himmelen der nesa peker", som IKKE forsvant etter forrige runde sin minY-fiks i
    // solidSurfaceHeightAt - trolig enda en, ikke identifisert, kilde til en for høy "bakke"-avlesning et
    // sted i verden): uansett HVILKEN kollider/formel som måtte returnere en for høy verdi et sted, kan
    // dekalen nå strukturelt aldri havne synlig oppe i himmelen foran droneen igjen - den flater i verste
    // fall bare ut på droneens egen høyde (fullt synlig/opak, men RETT VED droneen, ikke langt unna og
    // oppe i løse lufta).
    const groundY = Math.min(
        solidSurfaceHeightAt(droneState.position.x, droneState.position.z, droneState.position.y),
        droneState.position.y
    );
    const altitude = Math.max(0, droneState.position.y - groundY);
    droneShadowDecal.position.set(droneState.position.x, groundY + 0.03, droneState.position.z);
    const reach = (DRONE_ARM_LENGTH + bladeLengthForClass(droneState.droneClass) / 2) * currentDroneSpec().visualScale;
    const linT = Math.min(altitude / DRONE_SHADOW_MAX_ALT, 1);
    // sqrt i stedet for lineær t - falmer BRATT med det samme i lav høyde og flater ut mot slutten, i
    // stedet for en jevn lineær rampe helt til DRONE_SHADOW_MAX_ALT. Det var selve den lineære rampen som
    // ga inntrykket av at skyggen "hang igjen" langt oppe (fortsatt godt synlig ved 30-40 m, langt forbi
    // høyden den er til noen nytte) - med sqrt er skyggen allerede godt uttynnet et godt stykke under
    // DRONE_SHADOW_MAX_ALT, ikke bare akkurat idet den kuttes helt.
    const t = Math.sqrt(linT);
    // Litt STØRRE og svakere jo høyere man er (naturlig penumbra-vekst), tydelig mindre og mørkere rett
    // over bakken/plattformen - akkurat den kontrasten i lav høyde brukeren etterspurte.
    const size = reach * 2.6 * (1 + t * 0.6);
    droneShadowDecal.scale.set(size, size, 1);
    droneShadowDecal.material.opacity = DRONE_SHADOW_MAX_OPACITY - t * (DRONE_SHADOW_MAX_OPACITY - DRONE_SHADOW_MIN_OPACITY);
    droneShadowDecal.visible = linT < 1; // usynlig fra og med DRONE_SHADOW_MAX_ALT - ingen "henger igjen"-margin utover det
}

/* ---------- Visuell oppdatering + HUD ---------- */
function updateDroneVisual(dt) {
    droneGroup.position.copy(droneState.position);
    droneGroup.quaternion.copy(droneState.quaternion);
    updateDroneShadowDecal();
    const spinSpeed = droneState.armed ? (4 + inputState.stick.throttle * 60) : 0;
    dronePropellers.forEach(function (p) {
        p.mesh.rotation.y += p.spinDir * spinSpeed * dt;
    });
}

const hudMode = document.getElementById("hudMode");
// Selve klikkeflaten for modus-popoveren (buildModePopover) er HELE HUD-cellen (label + verdi), ikke bare
// hudMode-teksten - en usynlig "knapp" over hele #modeToggle, større og lettere å treffe enn kun teksten.
const modeToggle = document.getElementById("modeToggle");
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
const loiterWindBanner = document.getElementById("loiterWindBanner");
const loiterWindBannerText = document.getElementById("loiterWindBannerText");
// Livslager-ikonene ("Krasj i bevegelige mål") - se updateTargetLivesHud lenger ned i filen (samme
// målseksjon som TARGET_STRIKE_DRONE_LIVES).
const targetLivesHud = document.getElementById("targetLivesHud");
const targetLivesIcons = document.getElementById("targetLivesIcons");
const INJURY_TITLES = {
    pilot: "AU AU! DU HAR SKADET DEG SELV!",
    bystander: "DU HAR SKADET EN PERSON I PUBLIKUM!"
};

function updateHud() {
    hudMode.textContent = MODE_LABELS[droneState.flightMode];
    hudMode.classList.toggle("mode-flash", performance.now() < modeFlashUntil);
    hudArmed.textContent = droneState.injured ? "Skadet" : (droneState.crashed ? "Krasjet" : (droneState.armed ? "Armed" : "Killed"));
    hudArmed.className = "sim-status-value " + ((droneState.armed && !droneState.crashed && !droneState.injured) ? "sim-armed" : "sim-killed");
    // Personskade-varselet vinner over det vanlige krasj-varselet (droneen kan godt hard-lande ETTER
    // at den har truffet noen - da er det fortsatt personskaden som er poenget). Tittelen varierer med
    // HVEM som ble truffet (se updatePilotCollision/updateBystanderCollision) - å treffe en forbipasserende
    // er ikke "du har skadet deg selv".
    if (droneState.injured) injuryBannerTitle.textContent = INJURY_TITLES[droneState.injuredTarget] || INJURY_TITLES.pilot;
    injuryBanner.classList.toggle("show", droneState.injured);
    // targetHitPendingUntil (se updateTargetHitStage) - et vellykket treff i "Krasj i bevegelige mål" er
    // et krasj som ALLEREDE er på vei til å resette seg selv om et par sekunder (se
    // TARGET_HIT_POPUP_DELAY_MS), og popup-en om selve treffet vises allerede (exerciseWarningBanner).
    // "Trykk R for å resette"-hintet på det vanlige krasj-banneret er da misvisende - man trenger ikke
    // gjøre noe selv (brukerens krav: "ved suksessfull krasj trenger man ikke ha med beskjeden om R for
    // å resette"). Skjuler hele banneret (ikke bare hintet) i dette vinduet - to samtidige, delvis
    // motstridende meldinger ("KRASJ! Trykk R" og "Truffet! X/2") er unødvendig støy når treffet uansett
    // var meningen.
    crashBanner.classList.toggle("show", droneState.crashed && !droneState.injured && !exerciseState.targetHitPendingUntil);
    // Loiter er dimensjonert (I-ledd + LOITER_MAX_LEAN_ANGLE, se konstantene) for å holde posisjonen i
    // opptil LOITER_MAX_WIND_SPEED - over det kan krengevinkeltaket bli utilstrekkelig til å motvirke
    // vinden helt, og piloten varsles i stedet for å bare drifte uforklarlig.
    const currentWindSpeed = currentWindVector.length();
    const showWindWarning = droneState.flightMode === "loiter" && droneState.armed && !droneState.crashed
        && !droneState.injured && currentWindSpeed > LOITER_MAX_WIND_SPEED;
    if (showWindWarning) {
        loiterWindBannerText.textContent = currentWindSpeed.toFixed(1) + " m/s vind - Loiter er dimensjonert for opptil "
            + LOITER_MAX_WIND_SPEED + " m/s og kan miste posisjonen";
    }
    loiterWindBanner.classList.toggle("show", showWindWarning);
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
// Sim.togglePanel lukker selv alt annet meny-UI (andre paneler OG åpne dropdowns som Settings/modus-
// popoveren, se closeAllMenus i simulator-common.js) - ingen egen panel-ID-liste å vedlikeholde her.
function togglePanel(panel) {
    Sim.togglePanel(panel);
}

/* ---------- Modus-popover (klikk på "Modus" i HUD-en) ----------
   Samme tastatursnarveier/rekkefølge som Digit1-4-håndteringen over, og samme forklaringstekst som
   helpPanel sitt "1/2/3/4"-punkt - ETT sted å oppdatere om en modus' oppførsel endres. Samme mekanikk som
   VTOL-simmen sin buildModePopover (js/simulator-vtol.js) og som Settings-menyen (Sim.setupDropdown):
   åpne/lukke, lukk ved klikk utenfor, lukk andre åpne dropdowns/paneler.*/
function buildModePopover() {
    const popover = document.getElementById("modePopover");
    popover.innerHTML = "";
    Object.keys(MODE_LABELS).forEach(function (mode) {
        const btn = document.createElement("button");
        btn.type = "button";
        btn.className = "sim-dropdown-item";
        btn.title = MODE_DESCRIPTIONS[mode] || "";
        btn.innerHTML = '<span style="opacity:0.6; min-width:14px; display:inline-block;">' + MODE_KEY_LABELS[mode] + '</span> ' + MODE_LABELS[mode];
        btn.addEventListener("click", function () {
            setFlightMode(mode);
            popover.classList.remove("open");
        });
        popover.appendChild(btn);
    });
    Sim.setupDropdown(modeToggle, popover);
}

/* ---------- Rates-panel (rate-kurver + gass-expo, se Sim.buildRateAxisBox/buildThrottleExpoBox) ---------- */
// Boks-referansene beholdes (i stedet for kun å hive dem inn i grid og glemme dem) slik at
// updateRatesPanelLive under kan mate faktisk pinneposisjon inn i DENNE frame'ns .setLiveStick()
// hver eneste tick - se Sim.buildRateAxisBox-kommentaren for selve den grønne "live output"-prikken.
let rateAxisBoxes = {};
function buildRatesPanel() {
    const grid = document.getElementById("ratesGrid");
    grid.innerHTML = "";
    rateAxisBoxes = {};
    ["roll", "pitch", "yaw"].forEach(function (axis) {
        const box = Sim.buildRateAxisBox(rates[axis], AXIS_LABELS[axis], saveRates);
        rateAxisBoxes[axis] = box;
        grid.appendChild(box);
    });
    grid.appendChild(Sim.buildThrottleExpoBox(
        rates.throttle,
        "Throttle",
        "0 = lineær gass. Høyere verdi gir finere kontroll nær midten (rundt hover), mer kraftfull respons ved fullt utslag.",
        saveRates
    ));
}

// Kalt fra updateInput() hvert bilde (se der) - kun mens Rates-panelet faktisk er synlig, akkurat som
// updateGamepadAxesReadout sitt mainVisible-gate, for ikke å tegne på kanvaser ingen ser på.
function updateRatesPanelLive() {
    const panel = document.getElementById("ratesPanel");
    if (!panel || panel.style.display === "none") return;
    ["roll", "pitch", "yaw"].forEach(function (axis) {
        if (rateAxisBoxes[axis]) rateAxisBoxes[axis].setLiveStick(inputState.stick[axis]);
    });
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

// Håndtaket buildGamepadKillGrid returnerer (updateLiveStatus) - drives fra samme per-bilde-lytter
// som resten av panelets live-visning, se updateGamepadAxesReadout under.
let gamepadKillGridHandle = null;
function buildGamepadButtonsPanel() {
    const killContainer = document.getElementById("gamepadKillGrid");
    gamepadKillGridHandle = Sim.buildGamepadKillGrid(killContainer, gamepadMap.buttons, "kill", KILL_ACTION_LABEL, buttonManager, getActiveGamepad, saveGamepadMap);
    const container = document.getElementById("gamepadButtonsGrid");
    Sim.buildGamepadButtonsGrid(container, gamepadMap.buttons, BUTTON_ACTION_LABELS, buttonManager, getActiveGamepad, saveGamepadMap);
}

const gamepadPanelEl = document.getElementById("gamepadPanel");
const gamepadAxesReadoutEl = document.getElementById("gamepadAxesReadout");
// Egen "Knapper"-avlesning ADSKILT fra kanal-avlesningen over (se HTML-kommentaren ved
// gamepadButtonsReadout) - lar kalibreringsknappene stå MELLOM de to, i stedet for at hele
// knappelisten henger som en hale nederst i gamepadAxesReadoutEl under knappene.
const gamepadButtonsReadoutEl = document.getElementById("gamepadButtonsReadout");
// Fjernkontroll-oppsett-veiviser (se Sim.buildGamepadCalibrationWizard i simulator-common.js) - dukker
// opp AUTOMATISK første gang en gamepad oppdages på siden (maybeAutoOpen, kalt fra
// "gamepadconnected"/oppstartssjekket lenger ned) - i tillegg til, ikke i stedet for, gamepadPanelEl over.
const gamepadWizard = Sim.buildGamepadCalibrationWizard({
    storageKey: GAMEPAD_WIZARD_STORAGE_KEY,
    backdropEl: document.getElementById("gamepadWizardOverlay"),
    gridEl: document.getElementById("gamepadWizardGrid"),
    readoutEl: document.getElementById("gamepadWizardReadout"),
    buttonsReadoutEl: document.getElementById("gamepadWizardButtonsReadout"),
    calibrateBtnEl: document.getElementById("gamepadWizardCalibrateBtn"),
    resetBtnEl: document.getElementById("gamepadWizardResetCalibrationBtn"),
    calibrateStatusEl: document.getElementById("gamepadWizardCalibrateStatus"),
    saveBtnEl: document.getElementById("gamepadWizardSaveBtn"),
    cancelBtnEl: document.getElementById("gamepadWizardCancelBtn"),
    gamepadMap: gamepadMap,
    channelLabels: CHANNEL_LABELS,
    calibrationChannels: ["throttle", "roll", "pitch", "yaw"],
    axisCalibrationManager: axisCalibrationManager,
    getActiveGamepad: getActiveGamepad,
    saveGamepadMap: saveGamepadMap,
    minChannels: Sim.MIN_GAMEPAD_CHANNELS,
    // Får Settings sitt eget gamepadPanel til å reflektere ev. "Avbryt"-tilbakerulling (eller bare en
    // fersk kalibrering) med det samme, i tilfelle brukeren åpner det rett etter veiviseren.
    onClose: function () {
        const pad = getActiveGamepad() || rawFirstGamepad();
        if (pad) buildGamepadPanel(pad);
    }
});
function updateGamepadAxesReadout(gp) {
    const mainVisible = gamepadPanelEl.style.display !== "none";
    const wizardVisible = gamepadWizard.isOpen();
    if (!mainVisible && !wizardVisible) return;
    const activeGp = gp || getActiveGamepad();
    let outputByAxis = null;
    if (activeGp) {
        // Hvilken FYSISK kanal-indeks som er mappet til hver av de fire spillkanalene, med det den
        // faktisk gir spillet AKKURAT NÅ (etter reverse/skalering) - se kommentaren ved
        // Sim.updateGamepadAxesReadout.
        outputByAxis = {};
        outputByAxis[gamepadMap.throttle.axis] = { label: "Throttle", value: readThrottleAxis(activeGp, gamepadMap.throttle) };
        outputByAxis[gamepadMap.roll.axis] = { label: "Roll", value: readStickAxis(activeGp, gamepadMap.roll) };
        outputByAxis[gamepadMap.pitch.axis] = { label: "Pitch", value: readStickAxis(activeGp, gamepadMap.pitch) };
        outputByAxis[gamepadMap.yaw.axis] = { label: "Yaw", value: readStickAxis(activeGp, gamepadMap.yaw) };
    }
    if (mainVisible) {
        // includeButtons=false: "Knapper"-listen rendres i sitt eget element (gamepadButtonsReadoutEl)
        // rett under kalibreringsknappene i stedet for som en hale her - se HTML-kommentaren ved
        // gamepadButtonsReadout (brukeren: "vi skal flytte knappen opp, så de er rett under kanalene og
        // rett over 'Knapper'"). Samme mønster som fjernkontroll-veiviseren allerede brukte.
        Sim.updateGamepadAxesReadout(gamepadAxesReadoutEl, activeGp, Sim.MIN_GAMEPAD_CHANNELS, outputByAxis, false);
        Sim.updateGamepadButtonsReadout(gamepadButtonsReadoutEl, activeGp);
        if (gamepadKillGridHandle) gamepadKillGridHandle.updateLiveStatus(activeGp);
    }
    gamepadWizard.updateReadout(activeGp, outputByAxis);
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
    // raceFinishPendingUntil: 0 = intet fullført forsøk venter på resett. Ellers: tidspunktet resultat-
    // popup-vinduet (se showRaceResultPopup) er over og dronen faktisk skal resettes til start - brukerens
    // krav: "etter fullført må det ikke resettes så brått. må være noen sekunder først." Samme
    // pending-før-handling-mønster som targetHitPendingUntil (se der). KUN satt for punkt-til-punkt-baner
    // og "flere sammenhengende runder"-baner (raceTunnel/race3) - race1 resetter aldri (se updateRacingStage).
    raceFinishPendingUntil: 0,

    // Mål-i-bevegelse (targetStrike) - se "Øvelser: mål-i-bevegelse"-seksjonen. targetActiveVariant er
    // null når intet mål er underveis (mellom stages, eller øvelsen ikke aktiv) - ellers "drone"/"car"/
    // "person", samme streng som stage.variant.
    targetActiveVariant: null,
    // targetPatrolStartTime: tidspunktet MÅLETS inneværende patrulje (se TARGET_PATH_POINTS) startet - IKKE
    // en engangs A->B-strekning lenger (brukeren: "målene skal ikke resettes før man har krasjet i de") -
    // målet patruljerer frem og tilbake langs en flerpunkts-rute i det UENDELIGE fra dette tidspunktet, helt
    // til spilleren faktisk treffer det. Se updateTargetHitVisuals.
    targetPatrolStartTime: 0,
    targetRunStartTime: 0, // tidspunkt HELE forsøket (alle tre mål) startet - se finishTimedLoopRun
    // Antall treff på INNEVÆRENDE mål (0/1) - må nå TARGET_HITS_REQUIRED (2) før neste måltype (brukerens
    // krav: "husk to krasj i hvert mål før neste måltype"). Nullstilles KUN når et NYTT mål starter (ikke
    // ved hvert enkelt treff-respawn på samme mål) - se spawnTargetHitStage.
    targetHitCount: 0,
    // Antall PILOT-krasj brukt opp GJENNOM HELE forsøket (alle mål) - brukerens krav: "brukeren i denne
    // krasj-øvelsen har 9 droner og kan altså resette posisjon 8 ganger før hele øvelsen resetter fra
    // start og tiden nullstilles". Nullstilles KUN ved et helt nytt forsøk (steg 0, ikke-retry) eller når
    // TARGET_STRIKE_DRONE_LIVES faktisk nås - se updateTargetHitStage/spawnTargetHitStage.
    targetCrashesUsed: 0,
    targetHitPendingUntil: 0, // 0 = intet treff venter. Ellers: tidspunktet popup-vinduet etter et treff er
    // over og selve steget faktisk skal avansere/målet respawne - se updateTargetHitStage/
    // TARGET_HIT_POPUP_DELAY_MS.
    // targetRunFinishPendingUntil: 0 = intet fullført FORSØK (alle fire mål) venter. Ellers: tidspunktet
    // resultatpopup-en (se finishTimedLoopRun) er over og et helt nytt forsøk faktisk skal klargjøres -
    // brukerens krav: "når alle øvelsene er fullført så skal det stoppe opp, ikke resette. få en popup med
    // resultatet." Egen fra targetHitPendingUntil over (ETT enkelt treff sin kortere ventetid) - denne
    // gjelder kun selve SLUTTEN av et helt forsøk.
    targetRunFinishPendingUntil: 0
    // NB: FPV-kameravinkelen (settings.fpvTiltDeg) har bevisst INGEN tilsvarende saved/tving-mekanisme -
    // den skal alltid være brukerens egen, lagrede innstilling, uansett hvilken øvelse som pågår (se
    // kommentaren i startExercise). Ingen exercise setter forceFpvTiltDeg akkurat nå.
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

    if (stage.type === "hover" || stage.type === "hoverWind") {
        // Hover/hoverWind: ingen løype - en flat sirkel på bakken viser posisjonstoleransen, markøren
        // (under) pulserer på selve hover-punktet i flyhøyde. hoverWind har ingen nese-krav (fri stil,
        // som øvelse 9 - se updateExercise) og har derfor ingen retningspil, i motsetning til "hover".
        const ring = new THREE.Mesh(
            new THREE.RingGeometry(HOVER_POS_TOLERANCE - 0.15, HOVER_POS_TOLERANCE, 32),
            groundMat
        );
        ring.rotation.x = -Math.PI / 2;
        ring.position.set(HOVER_CENTER.x, 0.05, HOVER_CENTER.z);
        group.add(ring);
        if (stage.type === "hover") {
            const arrow = buildGroundArrow(stage.headingYaw, 0xffee55);
            arrow.position.set(HOVER_CENTER.x, 0.06, HOVER_CENTER.z);
            group.add(arrow);
        }
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
    } else if (stage.type === "targetHit") {
        // Mål-i-bevegelse (targetStrike): ingen løype å tegne - selve det bevegelige målet ER veiledningen
        // (markøren under følger det live, se updateExerciseGuideVisual). Samme "denne grenen MÅ finnes"-
        // begrunnelse som racing over - uten den ville stage.waypoints vært undefined her og kastet en feil.
    } else if (stage.type !== "return") {
        // "Returner hjem" har ingen løype/veiledning - hele poenget er å finne hjem selv; markøren
        // (under) pulserer over H-plassen som mål. ("hoverWind" er allerede fanget opp i grenen over,
        // sammen med "hover" - havner aldri her.)
        group.add(buildLoopStruts(stage.waypoints, EXERCISE_ALTITUDE, 0.08, guideMat));
        group.add(buildLoopStruts(stripWaypointY(stage.waypoints), 0.05, 0.06, groundMat));
    }

    const markerMat = new THREE.MeshStandardMaterial({ color: 0xffee55, transparent: true, opacity: 0.85 });
    const nextWaypointMarker = new THREE.Mesh(new THREE.SphereGeometry(0.4, 12, 10), markerMat);
    // targetHit (Krasj i bevegelige mål): selve målet ER veiledningen (se grenen over) - denne gule kula
    // (0.8 m i diameter) sitter da rett oppi/rundt selve målmodellen, mest synlig på den vesle dronen (0.34
    // m kropp) - brukeren: "Dronemålet trenger ikke den enorme kule kula inni seg". Skjult (IKKE utelatt
    // fra group - updateExerciseGuideVisual kaller fortsatt .position/.scale på den ubetinget hver bilde,
    // uansett stage.type) for targetHit-steg.
    nextWaypointMarker.visible = stage.type !== "targetHit";
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
    if (stage.type === "hover" || stage.type === "hoverWind") {
        exerciseGuideHandle.nextWaypointMarker.position.set(HOVER_CENTER.x, HOVER_ALTITUDE, HOVER_CENTER.z);
    } else if (stage.type === "return") {
        exerciseGuideHandle.nextWaypointMarker.position.set(0, 1.0, 0); // målet: H-plassen hjemme
    } else if (stage.type === "racing") {
        const wp = racingGatesForStage(stage)[exerciseState.wpIndex];
        exerciseGuideHandle.nextWaypointMarker.position.set(wp.x, wp.y, wp.z);
    } else if (stage.type === "targetHit") {
        // Følger målets EGEN, live posisjon (samme håndtak som selve treff-sjekken, se
        // targetHitHandleFor/updateTargetHitStage) - i motsetning til de andre grenene, som peker mot et
        // FAST punkt, beveger denne markøren seg med målet.
        const handle = targetHitHandleFor(stage.variant);
        exerciseGuideHandle.nextWaypointMarker.position.copy(handle.position);
    } else {
        const wp = stage.waypoints[exerciseState.wpIndex];
        exerciseGuideHandle.nextWaypointMarker.position.set(wp.x, wp.y !== undefined ? wp.y : EXERCISE_ALTITUDE, wp.z);
    }
    exerciseGuideHandle.nextWaypointMarker.scale.setScalar(0.85 + Math.sin(now / 200) * 0.15);
}

// "Returner hjem" (ex10) og "Hover i vind" (ny øvelse) deler samme "land, tell runden, respawn med
// ny tilfeldig vind, gjenta N ganger"-mønster (se landingsfase-håndteringen i updateExercise) - disse
// to er ETT sted å slå opp hvilke steg-typer som faktisk er repeterte på den måten og hvor mange
// runder DE krever, i stedet for å gjenta typesjekken flere steder.
function isRepeatedLandingStage(stage) {
    return stage.type === "return" || stage.type === "hoverWind";
}
function requiredRepsFor(stage) {
    return stage.type === "return" ? REQUIRED_RETURN_REPS : REQUIRED_HOVER_WIND_REPS;
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
    // returnRepsCompleted: navnet er historisk (fra "Returner hjem") - telleren er nå delt med
    // "Hover i vind" også, se isRepeatedLandingStage/requiredRepsFor over.
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
        // Tidsaktiviteter som løkker tilbake til steg 0 for et nytt forsøk i stedet for å avslutte
        // (targetStrike, se EXERCISES.targetStrike.timedLoop) - samme "logg tid, klargjør et nytt
        // forsøk"-idé som racingbanenes egen selv-resettende løkke, IKKE completeExercise/enterLandingPhase
        // (de hører til det ENGANGS bestått/ikke-bestått-systemet, EXERCISE_ORDER - se finishTimedLoopRun).
        if (exercise.timedLoop) { finishTimedLoopRun(exercise); return; }
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
    // spawnTargetHitStage posisjonerer målet med en gang (t=0, se der) - kallet MÅ derfor komme FØR
    // targetClockHint leser handle.position, ellers hadde hintet pekt mot forrige måls (gamle) plassering.
    // spawnTargetHitStage overskriver selv IKKE warningMessage her (stageIndex>0, isRetry usann - se dens
    // egen betingelse), så "Fullført! Neste: ..."-meldingen satt over står fortsatt og kan trygt utvides.
    if (nextStage.type === "targetHit") {
        spawnTargetHitStage(nextStage);
        exerciseState.warningMessage += targetClockHint(nextStage.variant);
    }
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
                const isRepeated = isRepeatedLandingStage(exercise.stages[0]);
                const requiredReps = isRepeated ? requiredRepsFor(exercise.stages[0]) : 0;
                if (isRepeated) exerciseState.returnRepsCompleted++;
                if (isRepeated && exerciseState.returnRepsCompleted < requiredReps) {
                    // Ikke siste gjennomføring ennå - respawn med ny tilfeldig posisjon/vind og fortsett.
                    // Klokka fortsetter å gå (samme startTime) - totaltiden for alle rundene lagres til slutt.
                    exerciseState.landingPhase = false;
                    exerciseState.landedSinceMs = null;
                    exerciseState.hoverHoldSec = 0; // "Hover i vind" sin holde-tid gjelder KUN inneværende runde
                    spawnForExercise(exercise);
                    // enterLandingPhase() (kalt for å komme hit) fjerner veiledningen (se der) - uten å
                    // bygge den på nytt her ville runde 2/3 stått igjen UTEN hover-ringen ("Hover i
                    // vind") eller den pulserende H-markøren ("Returner hjem") resten av økten, siden
                    // rebuildExerciseGuide() ellers kun kalles ved selve øvelsesstarten.
                    rebuildExerciseGuide();
                    exerciseState.warningMessage = "Runde " + exerciseState.returnRepsCompleted + "/" +
                        requiredReps + " fullført! Ny runde klargjort...";
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
        updateRacingCrashAutoReset(now);
        updateRacingStage(stage, dt, now);
        return;
    }

    if (stage.type === "targetHit") {
        exerciseState.headingErrorDeg = null; // ingen nese-krav - bare kollider med målet
        updateTargetHitStage(stage, dt, now);
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

    // "Hover i vind": samme posisjons-/høydehold som "hover" over, men UTEN nese-krav (fri stil, som
    // øvelse 9 - vindkorreksjon er utfordring nok i seg selv) OG uten advanceExerciseStage() - denne
    // øvelsen har bare ETT steg som gjentas REQUIRED_HOVER_WIND_REPS ganger med landing+ny tilfeldig
    // vind mellom hver runde (se isRepeatedLandingStage/landingsfase-håndteringen over), akkurat som
    // "return" sitt rep-mønster - derfor enterLandingPhase() direkte her i stedet.
    if (stage.type === "hoverWind") {
        exerciseState.headingErrorDeg = null;
        const posOk = horizontalCaptureDistance(droneState.position.x - HOVER_CENTER.x,
            droneState.position.z - HOVER_CENTER.z) <= HOVER_POS_TOLERANCE;
        const altOk = Math.abs(droneState.position.y - HOVER_ALTITUDE) <= HOVER_ALTITUDE_TOLERANCE;
        if (posOk && altOk) {
            exerciseState.hoverHoldSec += dt;
            if (exerciseState.hoverHoldSec >= stage.holdSec) {
                enterLandingPhase("Holdt! Land på landingsplassen (H).");
            }
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
const exerciseHudLapTimesItemEl = document.getElementById("exerciseHudLapTimesItem");
const exerciseHudLapTimesEl = document.getElementById("exerciseHudLapTimes");

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
    // "Returner hjem" og "Hover i vind" har hver ett fast steg (stageIndex alltid 0) som gjentas -
    // stegobjektet er gyldig gjennom hele landingsfasen der også, i motsetning til vanlige
    // flerstegs-øvelser (der landingPhase betyr stageIndex har passert siste steg og
    // getExerciseStage() ville returnert undefined). awaitingNext (bestått og fryst, se
    // completeExercise) betyr ALLTID at stegobjektet ikke lenger er gyldig å lese.
    const isRepeatedExercise = isRepeatedLandingStage(EXERCISES[exerciseState.exerciseId].stages[0]);
    const stage = (exerciseState.awaitingNext || (exerciseState.landingPhase && !isRepeatedExercise))
        ? null : getExerciseStage();
    // Killswitch-stegnavnet (stage.label) ville spoilet hvilket scenario som kommer/pågår - vises som
    // et nøytralt løpenummer i stedet, se killswitchDisplayLabel.
    exerciseHudStageEl.textContent = stage
        ? (stage.type === "killswitch" ? killswitchDisplayLabel() : stage.label)
        : (exerciseState.awaitingNext ? "Fullført!" : "Landing");
    // Merk runden som "teller ikke" så snart et avvik har skjedd i den - synlig konsekvens med en gang.
    const lapSuffix = (stage && exerciseState.lapHasViolation) ? " (runden teller ikke)" : "";
    const returnSuffix = exerciseState.landingPhase ? " - land på H" : "";
    // Racingbanen med flere runder (race3): "fremdrift" skal bare si HVILKEN runde man er på akkurat
    // nå (ikke portnummeret - det er for detaljert til å følge med på i farta) - se exerciseHudLapTimes
    // like under for selve rundetidene underveis. Enkeltrunde (race1, lapsRequired 1) beholder den gamle
    // port-/kryssingsteksten, som fortsatt er nyttig der siden det ikke finnes noe "runde X av Y" å vise.
    const raceLapsRequired = (stage && stage.type === "racing") ? (stage.lapsRequired || 1) : 1;
    const isMultiLapRacing = stage && stage.type === "racing" && raceLapsRequired > 1;
    exerciseHudLapsEl.textContent = !stage
        ? (exerciseState.awaitingNext ? "Se oppsummering" : "Land på H")
        : (stage.type === "hover"
            ? exerciseState.hoverHoldSec.toFixed(1) + "/" + stage.holdSec + " s"
            : (stage.type === "hoverWind"
                // Før landing: hold-nedtelling som "hover" over. Under landing (rep unnagjort, på vei
                // til H): rundetall som "return" - stegobjektet forblir gyldig (se isRepeatedExercise).
                ? (exerciseState.landingPhase
                    ? exerciseState.returnRepsCompleted + "/" + requiredRepsFor(stage) + returnSuffix
                    : exerciseState.hoverHoldSec.toFixed(1) + "/" + stage.holdSec + " s")
                : stage.type === "return"
                ? exerciseState.returnRepsCompleted + "/" + REQUIRED_RETURN_REPS + returnSuffix
                : stage.type === "killswitch"
                    ? killswitchStatusText()
                    : stage.type === "racing"
                        ? (isMultiLapRacing
                            ? "Runde " + Math.min(exerciseState.raceLapSplits.length + 1, raceLapsRequired) + " av " + raceLapsRequired
                            : (exerciseState.engaged ? "Port " + exerciseState.wpIndex + "/" + racingGatesForStage(stage).length : "Kryss start/mål"))
                        : stage.type === "targetHit"
                            ? "Mål " + (exerciseState.stageIndex + 1) + "/" + EXERCISES.targetStrike.stages.length
                            : exerciseState.lapsCleanCount + "/" + (stage.requiredCleanLaps || REQUIRED_CLEAN_LAPS) + lapSuffix));

    // Rundetidene som allerede er i boks denne økten (race3) - vises kompakt oppe til venstre sammen
    // med totaltiden (samme HUD-bar som exerciseHudTimerItem), ikke i den store Fremdrift-boksen (som
    // nå bare viser "Runde X av Y", se over) - da hadde den store boksen blitt for full/rotete.
    if (isMultiLapRacing && exerciseState.raceLapSplits.length > 0) {
        exerciseHudLapTimesItemEl.style.display = "";
        exerciseHudLapTimesEl.textContent = exerciseState.raceLapSplits
            .map(function (s) { return formatExerciseTime(s, 2); })
            .join(" · ");
    } else {
        exerciseHudLapTimesItemEl.style.display = "none";
    }

    // Racing har verken avviks-telling eller nese-krav (fri stil) - begge feltene er bare støy der,
    // se stage.type === "racing" i updateRacingStage.
    const isRacing = stage && stage.type === "racing";
    exerciseHudViolationsItemEl.style.display = isRacing ? "none" : "";
    exerciseHudHeadingErrorItemEl.style.display = isRacing ? "none" : "";

    // Avviks-status: tydelig, vedvarende indikator på hvor nær steget er å bli nullstilt - banneret
    // alene forsvinner etter noen sekunder og etterlot ingen synlig "du har brukt opp advarselen".
    if (!stage || stage.type === "hover" || stage.type === "hoverWind" || stage.type === "return" || stage.type === "killswitch") {
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
    } else if (stage && stage.type === "targetHit") {
        // Samme idé som racingbanens egen klokke over, men fra targetRunStartTime (satt idet det første
        // målet spawnes, se spawnTargetHitStage) - IKKE exerciseState.startTime, som ellers ville fortsatt
        // å telle fra ØVELSENS åpning gjennom flere fullførte forsøk på rad (samme selv-resettende løkke
        // som racingbanene, se finishTimedLoopRun).
        exerciseHudTimerItemEl.style.display = "";
        const elapsedSec = (performance.now() - exerciseState.targetRunStartTime) / 1000;
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
        // Mål-i-bevegelse (targetStrike): vanlig avgangsplass-spawn (resetDrone() over holder) - kun det
        // FØRSTE målet (dronen) trenger å klargjøres/vises her, se spawnTargetHitStage.
        if (exercise.stages[0].type === "targetHit") spawnTargetHitStage(exercise.stages[0]);
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
    // Ny tilfeldig vindSTYRKE ved hver spawn ("Hover i vind") - samme prinsipp/rekkefølge som
    // retningen over, bare for speed i stedet for directionDeg. Ingen eksisterende øvelse har trengt
    // dette før nå (ex9/ex10 har fast styrke, bare retningen varierer for ex10).
    if (exercise.wind && exercise.randomizeWindSpeed) {
        const range = exercise.randomizeWindSpeed;
        settings.wind.speed = range.min + Math.random() * (range.max - range.min);
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
    // De aller fleste øvelsene flys i Middels/VLOS - racingbanen (ex-race1) er unntaket: Racing-klasse,
    // Acro-modus og FPV-kamera. FPV-vinkelen (settings.fpvTiltDeg) tvinges bevisst ALDRI her, i motsetning
    // til klasse/kameramodus/flight mode under - den skal alltid følge brukerens egen lagrede innstilling,
    // uansett hvilken øvelse som pågår (kameraet leser den live, se animate). Alt annet lagres her og
    // gjenopprettes i stopExercise, akkurat som vind/skydekke under.
    setDroneClassEphemeral(exercise.droneClass || "mid");
    const forcedCameraMode = exercise.forceCameraMode || "vlos";
    cameraModeIndex = CAMERA_MODES.indexOf(forcedCameraMode);
    activeCamera = (forcedCameraMode === "chase") ? chaseCamera : (forcedCameraMode === "fpv") ? fpvCamera : vlosCamera;
    if (exercise.forceFlightMode) droneState.flightMode = exercise.forceFlightMode;

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
    // Ledertavlen vises i stedet for menyen mens en gradert tidsaktivitet flys (se stopExercise for
    // skjuling) - delvis gjennomsiktig, se CSS, så den ikke sperrer sikten helt slik selve menyen ville
    // gjort. Generalisert fra det opprinnelige "id === race1 || id === race3" til ALLE fire aktivitetene
    // i ACRO_EXERCISE_ORDER (raceTunnel/targetStrike lagt til, se acroMedalProgress-seksjonen) - hver har
    // sin egen liste (racingEntriesFor/RACING_ENTRIES_FIELD), ikke sammenlignbare med hverandre.
    const isTimedAcroActivity = ACRO_EXERCISE_ORDER.indexOf(id) !== -1;
    document.getElementById("racingLeaderboardOverlay").style.display = isTimedAcroActivity ? "" : "none";
    // Må tegnes på nytt her, ikke bare når en ny tid legges til, ellers ville man se forrige øvelses liste
    // et øyeblikk før man i det hele tatt har fullført et forsøk i DENNE.
    if (isTimedAcroActivity) {
        document.getElementById("racingLeaderboardTitle").textContent = "Ledertavle - " + exercise.label;
        renderRacingLeaderboard();
    }
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
    // Skjul et ev. aktivt targetStrike-mål og nullstill patruljetilstanden - uten dette kunne målet (som
    // updateTargetHitVisuals flytter UBETINGET, uavhengig av exerciseState.active) fortsette å patruljere
    // synlig i verden en liten stund etter at man har avbrutt/forlatt øvelsen, siden movement-logikken bare
    // sjekker handle.visible/targetActiveVariant, ikke selve exerciseState.active.
    if (exerciseState.targetActiveVariant) {
        targetHitHandleFor(exerciseState.targetActiveVariant).visible = false;
        exerciseState.targetActiveVariant = null;
        exerciseState.targetHitPendingUntil = 0;
    }
}

// Manuell R/reset-knapp: starter HELE øvelsen på nytt fra steg 0 og nullstiller stoppeklokken - i
// motsetning til det automatiske steg-nullstillet ved 2. avvik (som beholder tidligere bestått steg-
// fremgang OG lar klokken gå videre, som en implisitt tidsstraff for restarts).
function handleResetRequest() {
    // En manuell R midt i popup-/ventevinduet etter et nettopp fullført racingforsøk (se
    // raceFinishPendingUntil/showRaceResultPopup) skal resette med det samme (forvente umiddelbar respons
    // på en eksplisitt tastetrykk), IKKE etterlate en foreldet, senere avfyrt automatisk reset OG la
    // resultat-kortet henge synlig igjen på skjermen etter at dronen alt har flyttet seg.
    exerciseState.raceFinishPendingUntil = 0;
    if (raceResultPopupUntil) { raceResultPopupUntil = 0; raceResultPopupEl.classList.remove("show"); }
    if (!exerciseState.active) {
        resetDrone();
        return;
    }
    // targetHit (Krasj i bevegelige mål): R skal oppføre seg som et SELVPÅFØRT krasj - bruke opp én
    // drone/liv på SAMME måte som et ekte treff (se updateTargetHitStage), IKKE resette hele forsøket
    // tilbake til mål 1 (brukerens krav: "reset knapp skal fjerne et liv, ikke resette hele øvelsen, med
    // mindre det ikke vil være nok liv igjen til å fullføre øvelsen"). "Ikke nok liv" er nå PROAKTIV, ikke
    // bare "nøyaktig 0 igjen" - se targetStrikeLivesInsufficient (brukeren, presisert senere: "hvis man
    // resetter så mye at man får for lite liv til å fullføre resten av kollisjonene må hele øvelsen
    // resettes" - en spiller som R-resetter gjentatte ganger UTEN faktisk å treffe noe kunne ellers endt
    // opp med færre liv enn gjenstående påkrevde treff lenge før telleren faktisk nådde 0, og fortsatt
    // fått lov til å fly videre mot en matematisk umulig fullføring). targetHitPendingUntil-vakten hindrer
    // at en R rett etter et EKTE treff (mens popup-vinduet/den automatiske resetten fra DET treffet
    // fortsatt teller ned) stjeler en ekstra drone på toppen.
    const currentStage = getExerciseStage();
    if (!exerciseState.landingPhase && currentStage.type === "targetHit" && !exerciseState.targetHitPendingUntil) {
        exerciseState.targetCrashesUsed++;
        resetDrone();
        if (targetStrikeLivesInsufficient()) {
            restartTargetStrikeRun(EXERCISES[exerciseState.exerciseId]);
        } else {
            spawnTargetHitStage(currentStage, true); // samme mål på nytt - fremdriften (X/2) beholdes
        }
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
    spawnForExercise(EXERCISES[exerciseState.exerciseId]); // "Returner hjem"/"Hover i vind" får ny tilfeldig posisjon/vind, og runde-telleren er allerede nullstilt av resetStageProgress() over
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

/* ---------- Øvelser: mål-i-bevegelse (targetStrike, "Krasj i bevegelige mål") ----------
   Tre bevegelige mål (drone/bil/person) flydd på rad, én tidtatt økt - se EXERCISES.targetStrike. Egne,
   HELT NYE håndtak (targetDroneHandle/targetCarHandle/targetRunnerHandle, se initScene) i stedet for å
   gjenbruke heliHandle/pedestrianHandle: det generiske sikkerhetssystemet
   (updateBystanderCollision/updatePilotCollision) sjekker eksplisitt AVSTAND TIL pedestrianHandle og
   disarmer/skader dronen ("droneState.injured") ved nærkontakt - stikk i strid med denne øvelsens mål, som
   er å faktisk KOLLIDERE. Egne, nye props unngår enhver kobling til den logikken uten å måtte røre den i
   det hele tatt (den ser bare på kjente, faste håndtak - ikke vilkårlige nye objekter). SOLID_COLLIDERS/
   PROP_HAZARDS brukes av samme grunn IKKE for målene her - de er bygget for å blokkere/skade (se
   pushOutOfSolidWalls/updatePropStrikes), ikke registrere et scorbart treff. Målene er derfor rene,
   ukolliderende visuelle props - selve "treffet" er en enkel 3D-avstandssjekk, se updateTargetHitStage.
   Bevegelsen (se TARGET_PATH_POINTS/getTargetPatrol) er en PATRULJE langs en myk Catmull-Rom-kurve gjennom
   noen få rutepunkter, ping-ponget frem og tilbake i det UENDELIGE (aldri et tidsavbrudd som gjør at målet
   forsvinner/resettes av seg selv - brukerens krav: "målene skal ikke resettes før man har krasjet i de") -
   erstatter den forrige, enkle rette A->B-strekningen som talte som "bom" og respawnet ved tidsavbrudd.
   Både kurven (i stedet for rette linjestykker mellom rutepunktene) OG selve ping-pong-easingen (i stedet
   for en rett trekantbølge, se updateTargetHitVisuals) er der for å unngå brå fart-/retningsendringer
   (brukerens rapport: "Farts og retningsendring kan ikke være så plutselige. må være en mer naturlig
   bevegelse"). Den gule "neste veipunkt"-markørkula (buildExerciseGuide) er skjult for targetHit-steg -
   den ville sittet midt oppi selve målmodellen (mest synlig på den vesle dronen, brukerens rapport: "Dronemålet
   trenger ikke den enorme kule kula inni seg") og er uansett overflødig siden målet selv ER veiledningen. */
const TARGET_DRONE_CENTER = new THREE.Vector3(EXERCISE_CENTER.x, 6, EXERCISE_CENTER.z - 40); // åpen luft over øvelsesområdet
const TARGET_CAR_CENTER = new THREE.Vector3(90, 0, -140); // åpen mark øst for racingbane 1 (GATE_COURSE_CENTER, dx -40..36) - ingen overlapp
const TARGET_RUNNER_CENTER = new THREE.Vector3(140, 0, 95); // inni FOREST_TREES (senter 140,90) - "en løpende person i skogen" (brukeren)
// Fjerde mål - fastvinge-flyet (brukerens krav: "2 runder med krasj i den også"). Åpen luft vest for de
// andre tre målene/banene (ingen overlapp med GATE_COURSE_CENTER (0,-110)/GATE_COURSE_2_CENTER (0,320)/
// TARGET_CAR_CENTER/skogen), og en god del høyere (32 m, mot dronemålets 6 m) - en naturlig marsjhøyde for
// et fastvinget fly i FBWA, ikke en drone-aktig lav sveiv.
const TARGET_FIXEDWING_CENTER = new THREE.Vector3(-90, 32, -140);
// Den STATISKE, røde pynt-bilen ved avgangsplassen (posisjon satt i buildWorldObjects, se buildCar()-
// kallet der - kopiert hit som en ren konstant siden den aldri beveger seg, ingen live håndtak-referanse
// trengs). Lett å forveksle med targetCarHandle (den ekte, GRØNNE, bevegelige målbilen) under "Treff
// bilen"-steget, siden den STÅR akkurat der man tar av fra - brukerens rapport: kolliderer i denne (rød,
// parkert) uten noen tilbakemelding om at det var feil bil. Se checkWrongCarConfusion.
const DECORATIVE_CAR_POSITION = new THREE.Vector3(24, 0, 14);
const WRONG_CAR_HINT_RADIUS = 4;
const WRONG_CAR_HINT_COOLDOWN_MS = 5000; // hindrer at hintet spammer på nytt hvert bilde mens man dveler nær bilen
let wrongCarHintShownAt = 0;
// Patruljerute PER mål - punkter RELATIVT til eget senter (TARGET_CENTER under), fast y (unntatt dronen/
// fastvinge-flyet, se høydegynging/-vandring i updateTargetHitVisuals). Minst 3 segmenter (4 punkter) hver,
// altså minst to reelle retningsendringer per full gjennomgang - IKKE bare én rett strek frem og tilbake
// som før. Vinklene er bevisst holdt moderate (ingen skarpe brekk) for fortsatt naturlig bevegelse -
// dronen "veiver" mykt i lufta, bilen svinger slakt over åpen mark, personen jukser lett mellom noen
// trerader (se FOREST_TREES: spacing 11, senter 140,90 - TARGET_RUNNER_CENTER ligger ~5 m nord for raden
// ved z=90, så en moderat z-amplitude på ±4-8 m her holder seg trygt unna selve trestammene), og
// fastvinge-flyet svinger i en romslig, sveipende bane (et fly trenger mye større svingradius enn en
// quadcopter).
const TARGET_PATH_POINTS = {
    // Strukket ut fra det opprinnelige (±30/±8..10) - brukerens krav: "Dronen man skal krasje i kan...
    // fly litt lengre avgårde før den snur" - samme rutefasong, bare ca. 35% lenger i alle retninger.
    // Strukket ut enda et hakk (brukeren: "Dronen man skal krasje i kan fly på et litt større område") -
    // ca. 35% lenger igjen i alle retninger enn forrige runde. Første punktets z SÆRSKILT trukket lenger
    // sør (-15 -> -30, world z -61 -> -76) - med det opprinnelige, kortere hoppet svingte ruta fra dette
    // punktet innom hjørnet av BUILDING_POSITION (-35,-35, se BUILDING_SIZE) med under 2 m klaring i en
    // grov rett-linje-sjekk, og en ekte Catmull-Rom-kurve kan bue enda nærmere enn det - god margin nå i
    // stedet (brukerens krav: "pass på at den ikke får fly/glitche gjennom bygget" - se
    // updateTargetHitVisuals, som ikke har noen kollisjonssjekk mot SOLID_COLLIDERS i det hele tatt for
    // disse rene, patruljerende målene, så eneste reelle fiks er å holde selve RUTA unna).
    drone: [{ x: -54, z: -30 }, { x: -15, z: 18 }, { x: 26, z: -18 }, { x: 54, z: 11 }],
    car: [{ x: -40, z: 5 }, { x: -14, z: -12 }, { x: 12, z: 10 }, { x: 40, z: -8 }],
    person: [{ x: -33, z: -4 }, { x: -11, z: 4 }, { x: 11, z: -4 }, { x: 33, z: 4 }],
    // LUKKET løkke (se TARGET_PATROL_CLOSED/getTargetPatrol/updateTargetHitVisuals) - IKKE en åpen frem-
    // og-tilbake-strekning som de tre andre målene. Seks punkter i en avlang "racerbane"-oval (to lange
    // rette strekk + en bred, buet sving i hver ende) i stedet for bare fire hjørner - gir svingene god
    // radius, ikke en skarp knekk, akkurat som et ekte fastvinget fly trenger (brukerens rapport: "kan
    // ikke stoppe opp i lufta og endre retning 180 grader plutselig. må oppføre seg om et ekkte
    // modellfly").
    fixedwing: [
        { x: -55, z: -18 }, { x: 0, z: -26 }, { x: 55, z: -18 },
        { x: 55, z: 18 }, { x: 0, z: 26 }, { x: -55, z: 18 }
    ]
};
// Fart i m/s - samme troverdige tempo som før (drone/bil ~13.3 m/s ≈ 48 km/t, person ~6 m/s ≈ 21.6 km/t,
// en solid joggefart, IKKE de opprinnelige urealistiske ~59 km/t - brukerens tidligere rapport "sjekk at
// objektene beveger seg naturlig"), nå brukt til å bevege målet langs HELE patruljelengden (se
// updateTargetHitVisuals) i stedet for en fast "strekk/tid"-varighet. fastvinge-flyet krysser 17 m/s
// (≈61 km/t) - en troverdig FBWA-marsjfart for en liten skum-modellfly, raskere enn quadcopter-målet men
// ikke urealistisk.
const TARGET_SPEED = { drone: 13.3, car: 13.3, person: 6, fixedwing: 17 };
// Dronemålet: lett høydegynging + krengning inn i svingene - brukeren: "måldronen er veldig flat og flyr
// helt stabil på høyde. Må være mer realistisk" (orientTowardTravel gir i seg selv KUN gir - null krengning/
// stigning, siden patruljepunktene har konstant y). Se bruken i updateTargetHitVisuals. Bil/person holder
// seg bevisst flate/ukrengte (ingen tilsvarende rapport for dem - en bil/løper som "krenger" ville sett rart
// ut, i motsetning til en drone).
const TARGET_DRONE_BOB_FREQ = 1.3; // rad/s i sin() - rolig, syklisk høydesveiv, IKKE et fast, dødt nivå
const TARGET_DRONE_BOB_AMPLITUDE = 0.85; // meter - økt fra 0.45 (brukeren: "ha litt mer høydeendringer")
// Andre, saktere/mindre sinusledd OPPÅ det første (samme "sum av to sinusledd i stedet for ett"-prinsipp
// som fastvinge-flyets TARGET_FW_ALT_*) - gir mer variert, mindre perfekt-periodisk høydegynging enn ett
// rent sinusledd alene ville gjort, i stedet for bare å skru opp amplituden på samme, ensformige svev.
const TARGET_DRONE_BOB_FREQ2 = 0.47, TARGET_DRONE_BOB_AMPLITUDE2 = 0.35;
// Løpepersonens beinsving (buildTargetRunner/updateTargetHitVisuals) - rad/s i sin() og maks utslag i
// radianer per lem. En enkel, fast sinussving er nok her (brukeren: "beveger seg litt mer naturlig
// gjerne med bevegelser i bena" - ikke en biomekanisk gangesimulator).
const TARGET_RUNNER_STRIDE_FREQ = 7.5, TARGET_RUNNER_STRIDE_AMPLITUDE = 0.85;
const TARGET_DRONE_BANK_GAIN = 9; // enhetsløs forsterkning (radian krengning per radian retningsendring i den korte kurve-prøven, se bruken)
const TARGET_DRONE_MAX_BANK = THREE.MathUtils.degToRad(28); // klemmetak i radianer, samme enhet som selve krengningsutregningen
// Fastvinge-flyet: "litt tilfeldige men naturlige endringer i høyde og bank" (brukeren, om FBWA-modusen) -
// IKKE ett rent, periodisk sinussvev som dronens bob (som ville sett for regelmessig/mekanisk ut for et
// fly), men SUMMEN av to sinusledd med ulik (og innbyrdes urelatert) frekvens/fase - fortsatt en helt
// deterministisk, glatt funksjon av tiden (ingen Math.random()), men uten det opplagt periodiske "opp-ned-
// opp-ned i takt"-mønsteret et enkelt sinusledd ville gitt. Krenger også kraftigere enn dronemålet i
// svinger (fly krenger mer synlig enn en quadcopter i en sving) og litt tregere/roligere respons (lavere
// FREQ2 enn dronens BOB_FREQ) for en tyngre, mer "luftfartøy"-aktig følelse.
const TARGET_FW_ALT_FREQ1 = 0.22, TARGET_FW_ALT_AMP1 = 1.3;
const TARGET_FW_ALT_FREQ2 = 0.53, TARGET_FW_ALT_AMP2 = 0.6;
const TARGET_FW_BANK_GAIN = 13;
const TARGET_FW_MAX_BANK = THREE.MathUtils.degToRad(38);
// TARGET_HIT_RADIUS.drone økt fra 2.2 til 3.0 - matcher modellen som nå er 40% større (se buildTargetDrone,
// brukerens krav: "Dronen man skal krasje i kan være litt større").
// fixedwing: 3.0 var satt ut fra DEN OPPRINNELIGE, enkle stand-in-modellens vingespenn (2.2 m, se
// git-historikken) - men den ble senere byttet ut med den ekte, portable Heewing-geometrien (buildHwPlane),
// som er MYE mindre (spec.wingSpan=1.2 m, se HEEWING_TARGET_SPEC) - 3.0 sto uendret igjen etter byttet,
// og ga dermed en treffsone godt over dobbelt så stor som selve den synlige modellen (brukerens rapport:
// "hadde en krasj som ble detektert litt for langt fra flyet"). Satt ned til 1.5 - fortsatt en komfortabel
// margin over selve kroppen (~1 m lang, 1.2 m vingespenn), men ikke lenger grovt overdrevet.
const TARGET_HIT_RADIUS = { drone: 3.0, car: 2.6, person: 1.6, fixedwing: 1.5 };
const TARGET_CENTER = { drone: TARGET_DRONE_CENTER, car: TARGET_CAR_CENTER, person: TARGET_RUNNER_CENTER, fixedwing: TARGET_FIXEDWING_CENTER };
const TARGET_HIT_LABEL = { drone: "dronen", car: "bilen", person: "personen", fixedwing: "modellflyet" };
// Antall treff PÅ SAMME mål som kreves før banen går videre til neste måltype (brukerens krav: "husk to
// krasj i hvert mål før neste måltype") - IKKE "antall forsøk" som i den forrige (nå fjernede) bom/miss-
// logikken. Målet resettes (ny patrulje fra start) etter HVERT treff, også det første (brukerens krav:
// "og resett etter hver krasj") - se updateTargetHitStage.
const TARGET_HITS_REQUIRED = 2;
// Antall "droner" spilleren har totalt gjennom ETT forsøk (alle mål) - opprinnelig 9 (brukerens krav: "9
// droner og kan altså resette posisjon 8 ganger før hele øvelsen resetter"), senere hevet med tre til 12
// (brukerens krav, ordrett: "Legg til tre liv til") - trolig fordi et fjerde mål (fastvinge-flyet, se
// TARGET_CENTER.fixedwing) ble lagt til samtidig, og 9 liv ble knappe med fire mål å komme seg gjennom.
// Hvert treff (uansett mål) bruker opp én drone og teleporterer piloten tilbake til avgangsplassen ("man
// har jo krasjet og trenger en 'ny' drone") - se updateTargetHitStage. Med 12 droner i lager brukes 11 til
// å erstatte etter de 11 første krasjene; det TOLVTE krasjet har ingen drone igjen å erstatte med og
// trigger i stedet restartTargetStrikeRun (helt forfra, ingen tid logges) - MED ÉTT UNNTAK: er akkurat
// DETTE krasjet også selve det avgjørende, siste treffet som fullfører hele forsøket (siste mål, andre
// treff), fullføres økten likevel i stedet for å restarte (brukerens krav: "ved siste krasj i person gjør
// det ikke noe om det siste livet blir brukt opp heller") - se den egne isFinalHitOfRun-sjekken i
// updateTargetHitStage.
const TARGET_STRIKE_DRONE_LIVES = 12;
// Sjekker om det GJENVÆRENDE antallet droner faktisk holder til å fullføre RESTEN av de påkrevde treffene
// - IKKE bare "er det nøyaktig 0 droner igjen ennå" (den opprinnelige, rent REAKTIVE sjekken). Brukerens
// krav, ordrett: "hvis man resetter så mye at man får for lite liv til å fullføre resten av kollisjonene
// må hele øvelsen resettes." Manuelle R-resetter bruker OGSÅ opp en drone UTEN å telle som fremgang (se
// handleResetRequest sin egen targetHit-gren) - denne funksjonen brukes derfor BÅDE der og i
// updateTargetHitStage sin pending-gren (ekte treff), slik at et forsøk som resettes nok ganger til å bli
// matematisk umulig å fullføre, restarter med det samme i stedet for å vente til man faktisk når 0.
function targetStrikeLivesInsufficient() {
    const exercise = EXERCISES[exerciseState.exerciseId];
    const hitsStillNeeded = (TARGET_HITS_REQUIRED - exerciseState.targetHitCount) +
        TARGET_HITS_REQUIRED * (exercise.stages.length - 1 - exerciseState.stageIndex);
    const livesRemaining = TARGET_STRIKE_DRONE_LIVES - exerciseState.targetCrashesUsed;
    return livesRemaining < hitsStillNeeded;
}
// Livslager-ikonene nede i venstre hjørne (targetLivesHud/targetLivesIcons, se DOM-referansene lenger opp
// i filen) - brukerens krav: "ikke noe 'x droner igjen' i popupene. ha heller noen ikoner nede i venstre
// hjørne med antall droner/liv" (i stedet for tekstlinjen forrige runde brukte i selve treff-popupen, se
// updateTargetHitStage). Bygges lat (kun første gang øvelsen faktisk er aktiv, se buildTargetLivesIconsIfNeeded)
// i stedet for ved modul-lasting - TARGET_STRIKE_DRONE_LIVES-konstanten over finnes riktignok allerede da,
// men det er ingen vits i å bygge DOM-noder for en HUD-del som nesten aldri vises.
function buildTargetLivesIconsIfNeeded() {
    if (targetLivesIcons.children.length) return;
    for (let i = 0; i < TARGET_STRIKE_DRONE_LIVES; i++) {
        const icon = document.createElement("i");
        // fa-helicopter (IKKE et eget "drone"-ikon - Font Awesome sitt gratis sett har ingen quadcopter-
        // variant) - samme ikon som allerede brukes for "Drone og kamera"-menyvalget (simulator.html),
        // gjenkjennelig nok som en generisk luftfartøy-stand-in for "én drone i lager".
        icon.className = "fa-solid fa-helicopter";
        targetLivesIcons.appendChild(icon);
    }
}
// Kalt fra animate() hvert bilde (samme mønster som updateHud) - viser/skjuler hele raden basert på om
// targetStrike-øvelsen faktisk er aktiv akkurat nå, og dimmer ett ikon per drone som er brukt opp (se
// exerciseState.targetCrashesUsed, satt i updateTargetHitStage). De ubrukte ikonene (indeks >= used) lyser
// normalt - "antall droner/liv" er dermed alltid avlesbart med ett blikk, uten en tekstlinje som forstyrrer
// selve treff-popupen.
function updateTargetLivesHud() {
    const active = exerciseState.active && exerciseState.exerciseId === "targetStrike";
    targetLivesHud.classList.toggle("show", active);
    if (!active) return;
    buildTargetLivesIconsIfNeeded();
    const used = exerciseState.targetCrashesUsed;
    for (let i = 0; i < targetLivesIcons.children.length; i++) {
        targetLivesIcons.children[i].classList.toggle("sim-target-life-used", i < used);
    }
}
// Hvor lenge "Truffet!"-popupen (og selve ventetiden før målet faktisk respawner/steget avanserer) varer
// etter et treff - brukerens krav: "når man har truffet må man få opp en liten popup om treff. vent i 2
// sekunder før resett." Se updateTargetHitStage/exerciseState.targetHitPendingUntil.
const TARGET_HIT_POPUP_DELAY_MS = 2000;

function targetHitHandleFor(variant) {
    return variant === "drone" ? targetDroneHandle : variant === "car" ? targetCarHandle :
        variant === "fixedwing" ? targetFixedWingHandle : targetRunnerHandle;
}
// Klokkeretning fra dronens NÅVÆRENDE posisjon/nese til et gitt punkt, sett fra cockpiten (FPV,
// tvunget for hele targetStrike-øvelsen - se EXERCISES.targetStrike.forceCameraMode) - 12 = rett fram
// (langs droneState.quaternion sin lokale -Z, samme "nesa"-konvensjon som resten av fila, f.eks.
// applyKillswitchInputOverride), 3 = rett til høyre, 6 = rett bak, 9 = rett til venstre, medurs derimellom.
// Brukerens krav: "gi brukeren et lite hint i popupen om ca. hvor flyet er (retning kl.11?)" -
// modellflyet (fixedwing) er MYE fortere og går i en stor, åpen løkke langt unna de tre andre målene
// (se TARGET_PATH_POINTS.fixedwing), og er dermed betydelig vanskeligere å få øye på i tide enn de tre
// andre - derfor kun brukt for den varianten (targetClockHint under), ikke drone/bil/person.
function clockHourFromDrone(point) {
    const toTarget = new THREE.Vector3().subVectors(point, droneState.position);
    toTarget.y = 0;
    if (toTarget.lengthSq() < 1e-6) return 12;
    const local = toTarget.normalize().applyQuaternion(droneState.quaternion.clone().invert());
    const angleDeg = Math.atan2(local.x, -local.z) * (180 / Math.PI); // lokal -Z er forover
    let hour = Math.round(angleDeg / 30) % 12;
    if (hour <= 0) hour += 12;
    return hour;
}
// Kun et GROVT, ett-blikks anslag idet popupen vises (målets posisjon i selve trefføyeblikket, ikke
// live-oppdatert mens popupen står) - modellflyet rekker uansett å bevege seg videre langs løkka i
// mellomtiden, hintet er ment som "se omtrent DEN veien", ikke en presis peker.
function targetClockHint(variant) {
    if (variant !== "fixedwing") return "";
    const handle = targetHitHandleFor(variant);
    if (!handle.visible) return "";
    return " Omtrent i retning kl. " + clockHourFromDrone(handle.position) + ".";
}
// Klargjør/(re)starter ett måls patrulje - kalt fra spawnForExercise (steg 0, øvelsesstart/full R-restart),
// advanceExerciseStage (neste måltype etter to treff) OG updateTargetHitStage (samme mål på nytt etter
// treff 1 av 2). isRetry (default false): true KUN når SAMME mål respawnes etter et treff som IKKE var det
// avgjørende (se updateTargetHitStage) - styrer om targetHitCount (TARGET_HITS_REQUIRED-telleren)
// nullstilles eller beholdes.
function spawnTargetHitStage(stage, isRetry) {
    const variant = stage.variant;
    exerciseState.targetActiveVariant = variant;
    if (!isRetry) exerciseState.targetHitCount = 0;
    exerciseState.targetPatrolStartTime = performance.now();
    // Etter et treff (isRetry - samme mål på nytt for runde 2/2) - start patruljen et STYKKE ute på ruta i
    // stedet for alltid nøyaktig samme startpunkt (brukerens krav, ordrett: "alle krasj øvelsene - pass på
    // at ting spawner et litt annet sted etter første krasj med tingen"). Trekker et tilfeldig tidsintervall
    // FRA startklokken i stedet for å flytte selve patruljepunktene - det gir en negativ "elapsedSec" ved
    // t=0 i updateTargetHitVisuals, som i praksis flytter startFASEN et tilfeldig stykke langs SAMME,
    // etablerte patruljerute (fortsatt et sted MÅLET faktisk patruljerer langs) i stedet for å teleportere
    // det til et helt urelatert sted i verden. IKKE på et helt NYTT måltype (isRetry usann) - der er
    // "Kollider med [mål]"-hintet allerede knyttet til et kjent, forutsigbart startpunkt.
    if (isRetry) {
        const patrol = getTargetPatrol(variant);
        const cycleSec = (2 * patrol.length) / TARGET_SPEED[variant];
        exerciseState.targetPatrolStartTime -= Math.random() * cycleSec * 1000;
    }
    const handle = targetHitHandleFor(variant);
    handle.visible = true;
    // Posisjonerer/orienterer målet med en gang (t=0 på patruljen) i stedet for å vente til neste
    // animate()-bilde - uten dette kunne målet blinke synlig ett bilde på sin GAMLE (forrige patruljes)
    // posisjon før updateTargetHitVisuals først rakk å flytte det.
    updateTargetHitVisuals(exerciseState.targetPatrolStartTime);
    // Totaltiden for HELE forsøket (alle tre mål) regnes fra det aller første målet spawnes - kun sant
    // for steg 0 (øvelsesstart/restart), IKKE når neste mål klargjøres etter to treff (steg 1/2) eller et
    // mål respawnes etter treff nr. 1 (samme steg på nytt) - se finishTimedLoopRun. Samme betingelse for
    // dronetallet (targetCrashesUsed) - det skal telle krasj GJENNOM HELE forsøket (alle tre mål), IKKE
    // nullstilles hver gang man går videre til neste måltype (restartTargetStrikeRun nullstiller den
    // separat for sitt eget tilfelle - "tom for droner" - se der).
    if (exerciseState.stageIndex === 0 && !isRetry) {
        exerciseState.targetRunStartTime = performance.now();
        exerciseState.targetCrashesUsed = 0;
    }
    // Liten "hva skal jeg gjøre nå?"-tooltip etter en posisjonsreset (ekte treff ELLER manuell R, se
    // handleResetRequest) - brukerens krav: "etter hver posisjonsresett må det komme opp et lite tooltip
    // om hva man skal gjøre. for når f.eks. man skal krasje i bil for andre gang vet ikke brukeren
    // nødvendigvis det." KUN ved isRetry (samme mål på nytt) eller helt i starten av øvelsen - IKKE når
    // man akkurat har rykket videre til et NYTT måltype (isRetry false, stageIndex>0): der har
    // advanceExerciseStage allerede satt sin egen "Fullført! Neste: ..."-melding (5 sek), som denne ellers
    // ville kuttet ned til nesten ingenting ved å overskrive den med det samme.
    if (isRetry || exerciseState.stageIndex === 0) {
        // stage.label (IKKE "Kollider med " + TARGET_HIT_LABEL[variant]) - samme "Treff ..."-ordlyd som
        // advanceExerciseStage sin "Fullført! Neste: ..."-melding bruker (nextStage.label) for førstegangs-
        // ankomst til et måltype. Brukerens rapport: runde 2 på samme mål (isRetry) viste plutselig
        // "Kollider med personen" der runde 1 hadde vist "Treff personen i skogen" - inkonsekvent ordlyd
        // for samme handling. "Kollider med" (uendret) brukes fortsatt bevisst andre steder i teksten som
        // faktisk beskriver selve KOLLISJONEN som handling (f.eks. exercise.shortDescription,
        // "feil bil"-hintet) - kun DENNE popup-meldingen (som duplisererte stage.label med andre ord)
        // endres.
        exerciseState.warningMessage = stage.label + " (" +
            (exerciseState.targetHitCount + 1) + "/" + TARGET_HITS_REQUIRED + ")" + targetClockHint(variant);
        exerciseState.warningUntil = performance.now() + 3000;
        exerciseState.warningIsSuccess = true;
    }
}
// Glatt kurve gjennom TARGET_PATH_POINTS - bygget/cachet KUN én gang per mål (rutepunktene endrer seg
// aldri, ingen vits i å bygge kurven på nytt hvert bilde). CatmullRomCurve3 (IKKE rette linjestykker
// mellom punktene, som forrige versjon brukte) runder svingpunktene mykt av - brukeren: "Farts og
// retningsendring kan ikke være så plutselige. må være en mer naturlig bevegelse" - en rett polyline gir
// en brå knekk i retningen nøyaktig i hvert punkt, en Catmull-Rom-spline glir jevnt gjennom dem i stedet.
// curve.getLength()/getPointAt/getTangentAt bruker kurvens egen bueleng­de-oppslagstabell - konstant fart
// LANGS SELVE KURVEN (ikke bare langs de rette kontrollpunkt-segmentene), se updateTargetHitVisuals.
const targetPatrolCurves = {};
// closed (kun fixedwing) - se getTargetPatrol/updateTargetHitVisuals sin egen kommentar for hvorfor:
// et ekte modellfly kan ikke stoppe midt i lufta og snu 180° momentant, i motsetning til
// drone-/bil-/personmålet (brukerens rapport: "kan ikke stoppe opp i lufta og endre retning 180 grader
// plutselig. må oppføre seg om et ekkte modellfly").
const TARGET_PATROL_CLOSED = { fixedwing: true };
function getTargetPatrol(variant) {
    if (targetPatrolCurves[variant]) return targetPatrolCurves[variant];
    const center = TARGET_CENTER[variant];
    const points3d = TARGET_PATH_POINTS[variant].map(function (p) {
        return new THREE.Vector3(center.x + p.x, center.y, center.z + p.z);
    });
    const curve = new THREE.CatmullRomCurve3(points3d, !!TARGET_PATROL_CLOSED[variant], "catmullrom", 0.5);
    const entry = { curve: curve, length: curve.getLength() };
    targetPatrolCurves[variant] = entry;
    return entry;
}
// Animerer det AKTIVE målet langs patruljekurven si hvert bilde - kalt fra animate() UBETINGET, samme
// mønster som updateKillswitchVisuals. Ping-ponger frem og tilbake langs HELE kurven i det uendelige - i
// MOTSETNING til den forrige versjonen finnes det ingen "fullført uten treff"-gren her lenger: målet bare
// fortsetter å patruljere til et faktisk treff (updateTargetHitStage) gjør det usynlig og ber om et nytt
// forsøk (se der) - brukerens krav: "målene skal ikke resettes før man har krasjet i de".
// "Heiset cosinus"-ping-pong (IKKE en rett trekantbølge, som forrige versjon brukte) - fasen 0..2 mappes
// via (1-cos(πfase))/2 i stedet for rett frem/tilbake - farten (den deriverte) er NØYAKTIG null akkurat i
// svingpunktene ved hver ende av ruta, så målet bremser mykt ned og snur i stedet for å momentant reversere
// full fart (samme brukerrapport som over - gjaldt både retning OG fart).
function updateTargetHitVisuals(now) {
    const variant = exerciseState.targetActiveVariant;
    if (!variant) return;
    const handle = targetHitHandleFor(variant);
    if (!handle.visible) return;
    // Personen ligger nede etter et treff (knockPersonOver, kalt fra updateTargetHitStage) - fryser
    // patruljebevegelsen mens han faller/blir liggende, i stedet for å fortsette å gli videre langs banen
    // med beina fortsatt i gang under (brukerens krav: "han skal falle over ende når man krasjer i han").
    // updatePersonFalls (kalt UBETINGET fra animate()) styrer selve fallanimasjonen uavhengig av dette.
    if (handle.userData.fallen) return;
    const patrol = getTargetPatrol(variant);
    const elapsedSec = (now - exerciseState.targetPatrolStartTime) / 1000;
    let u, forward;
    if (variant === "fixedwing") {
        // LUKKET løkke, alltid samme retning - IKKE ping-pong (se TARGET_PATROL_CLOSED/
        // TARGET_PATH_POINTS.fixedwing sin egen kommentar). Et ekte modellfly kan ikke stoppe midt i lufta
        // og reversere 180° momentant, selv med en mykt easet fart mot 0 slik ping-pongen under bruker for
        // de tre andre målene - brukerens rapport: "kan ikke stoppe opp i lufta og endre retning 180
        // grader plutselig. må oppføre seg om et ekkte modellfly". u øker jevnt og vikler seg rundt (%1) -
        // farten er alltid konstant og alltid fremover, aldri null.
        u = (elapsedSec * TARGET_SPEED.fixedwing / patrol.length) % 1;
        forward = true;
    } else {
        const cycleSec = (2 * patrol.length) / TARGET_SPEED[variant];
        const phase = ((elapsedSec % cycleSec) / cycleSec) * 2; // 0..2 (0/2 = rutestart, 1 = ruteslutt)
        u = clamp((1 - Math.cos(Math.PI * phase)) / 2, 0, 1); // 0..1, easet kurveposisjon
        forward = phase < 1;
    }
    const pos = patrol.curve.getPointAt(u);
    const tangent = patrol.curve.getTangentAt(u); // enhetsvektor, alltid i kurvens EGEN (økende u) retning
    if (variant === "drone") {
        pos.y += Math.sin(elapsedSec * TARGET_DRONE_BOB_FREQ) * TARGET_DRONE_BOB_AMPLITUDE +
            Math.sin(elapsedSec * TARGET_DRONE_BOB_FREQ2 + 2.3) * TARGET_DRONE_BOB_AMPLITUDE2;
    }
    // Fastvinge-flyets "litt tilfeldige men naturlige" høydevandring (brukeren, om FBWA) - se
    // TARGET_FW_ALT_*-konstantenes egen kommentar for hvorfor summen av to sinusledd i stedet for ett.
    if (variant === "fixedwing") {
        pos.y += Math.sin(elapsedSec * TARGET_FW_ALT_FREQ1) * TARGET_FW_ALT_AMP1 +
            Math.sin(elapsedSec * TARGET_FW_ALT_FREQ2 + 1.7) * TARGET_FW_ALT_AMP2;
        // Ren kosmetikk - propellene (se buildHwPlane sin propGroups-liste) spinner jevnt mens flyet
        // patruljerer, i stedet for å stå helt stille. propGroups[0] er den bakre, VERTIKALE motoren
        // (bladene bygget for en Y-akse-spinn, se buildHwPlane) - de to fremre traktormotorene (resten av
        // lista) spinner om lokal Z i stedet (nese-pekende akse), samme fordeling som selve bladenes egen
        // forhåndsrotasjon.
        const propGroups = handle.userData.propGroups;
        if (propGroups) {
            const spin = elapsedSec * 40;
            propGroups[0].rotation.y = spin;
            for (let i = 1; i < propGroups.length; i++) propGroups[i].rotation.z = spin;
        }
    }
    handle.position.copy(pos);
    const aheadPoint = pos.clone().addScaledVector(tangent, forward ? 1 : -1); // snur peikeretningen på returturen
    if (variant === "person") {
        orientPersonGroupTowardTravel(handle, pos, aheadPoint);
        // Løpesteg - bena/armene svinger motfase (venstre ben frem = høyre arm frem, som et ekte steg),
        // se buildTargetRunner sin egen kommentar. TARGET_RUNNER_STRIDE_FREQ er tunet til å se ut som et
        // troverdig jogge-steg ved TARGET_SPEED.person sin fart, IKKE koblet til selve fartsberegningen -
        // en enkel, fast frekvens er nok her (brukeren ba om "litt mer naturlig", ikke en biomekanisk
        // korrekt gange-simulator).
        const legs = handle.userData.legs, arms = handle.userData.arms;
        if (legs && arms) {
            const strideSwing = Math.sin(elapsedSec * TARGET_RUNNER_STRIDE_FREQ) * TARGET_RUNNER_STRIDE_AMPLITUDE;
            legs[0].rotation.x = strideSwing;
            legs[1].rotation.x = -strideSwing;
            arms[0].rotation.x = -strideSwing;
            arms[1].rotation.x = strideSwing;
        }
        return;
    }
    orientTowardTravel(handle, pos, aheadPoint);
    if (variant !== "drone" && variant !== "fixedwing") return;
    // Krengning: sammenligner retningen NÅ med retningen et lite stykke lenger frem på kurven - jo mer
    // den lokale kurvaturen svinger der, desto mer krengning (klemt til maxBank). uAhead flyttes i SAMME
    // retning som forward (fremover på turen, bakover på returen) - ellers ville krengningen pekt feil vei
    // på halve patruljen. Fastvinge-flyet bruker EGNE, kraftigere verdier (TARGET_FW_BANK_GAIN/MAX_BANK) -
    // et fly krenger synlig mer i svinger enn en liten quadcopter, se konstantenes egen kommentar.
    const bankGain = variant === "drone" ? TARGET_DRONE_BANK_GAIN : TARGET_FW_BANK_GAIN;
    const maxBank = variant === "drone" ? TARGET_DRONE_MAX_BANK : TARGET_FW_MAX_BANK;
    const uAhead = clamp(u + (forward ? 0.01 : -0.01), 0, 1);
    const tangentAhead = patrol.curve.getTangentAt(uAhead);
    const turnAmount = tangent.angleTo(tangentAhead);
    if (turnAmount > 1e-4) {
        const turnCross = new THREE.Vector3().crossVectors(tangent, tangentAhead);
        const bankSign = (turnCross.y >= 0 ? 1 : -1) * (forward ? 1 : -1);
        const bank = clamp(turnAmount * bankGain, 0, maxBank) * bankSign;
        handle.quaternion.multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), bank));
    }
}
// Selve treffsjekken - ren 3D-avstand (i motsetning til de fleste andre steg-typenes horisontale sjekker,
// se f.eks. HELI_SAFE_HORIZ_DISTANCE-kommentaren for hvorfor DE bevisst er horisontale - her skal en
// treffende kollisjon telle uansett retning/vinkel, akkurat som et ekte sammenstøt ville gjort).
function updateTargetHitStage(stage, dt, now) {
    // Hele forsøket (alle fire mål) er akkurat fullført og venter på resultatpopup-en sin egen ventetid
    // (se finishTimedLoopRun) - IKKE start et nytt forsøk med det samme (brukerens krav: "når alle
    // øvelsene er fullført så skal det stoppe opp, ikke resette. få en popup med resultatet"). Sjekkes
    // FØR targetHitPendingUntil under, siden begge aldri er satt samtidig (finishTimedLoopRun nullstiller
    // targetActiveVariant, så selve treffsjekken lenger ned aldri ville funnet noe uansett).
    if (exerciseState.targetRunFinishPendingUntil) {
        if (now >= exerciseState.targetRunFinishPendingUntil) {
            exerciseState.targetRunFinishPendingUntil = 0;
            const exercise = EXERCISES[exerciseState.exerciseId];
            exerciseState.stageIndex = 0;
            resetStageProgress();
            resetDrone();
            rebuildExerciseGuide();
            spawnTargetHitStage(exercise.stages[0]); // nytt forsøk, tilbake til mål 1 (dronen)
        }
        return;
    }
    // Et treff venter allerede på popup-vinduet sitt (se under) - vent til TARGET_HIT_POPUP_DELAY_MS er
    // over før selve steget faktisk avanserer/målet respawner (brukerens krav: "vent i 2 sekunder før
    // resett"), i stedet for å hoppe rett videre idet drona berører målet.
    if (exerciseState.targetHitPendingUntil) {
        if (now >= exerciseState.targetHitPendingUntil) {
            exerciseState.targetHitPendingUntil = 0;
            // Piloten sin EGEN "drone" er brukt opp ved dette krasjet - tilbake til avgangsplassen med en
            // frisk en (brukerens krav: "man har jo krasjet og trenger en 'ny' drone"), FØR selve
            // mål-/steg-avgjørelsen under. Gjelder uansett hvilken av de to grenene under som følger.
            // resetDrone() reiser også en eventuelt falt person opp igjen (se resetPersonFalls) - personen
            // skjules derfor FØRST rett under, mens han fortsatt (midlertidig) står oppreist igjen.
            resetDrone();
            // Personen har ligget nede og blitt vist frem helt til nå (se knockPersonOver/
            // updateTargetHitVisuals sin fallen-sjekk) - skjules først idet selve respawnen/steg-
            // avgjørelsen faktisk skjer, IKKE i samme øyeblikk han ble truffet (brukerens krav: "han skal
            // falle over ende når man krasjer i han"). De tre andre målene ble allerede skjult med det
            // samme treffet ble registrert (se treffsjekken under).
            if (stage.variant === "person") targetHitHandleFor("person").visible = false;
            const exercise = EXERCISES[exerciseState.exerciseId];
            // Er DETTE hitet også selve det siste, avgjørende treffet i hele forsøket (siste mål i
            // rekkefølgen, andre av to påkrevde treff der) - fullfør uansett, selv om det brukte opp den
            // siste dronen (brukerens krav, ordrett: "ved siste krasj i person gjør det ikke noe om det
            // siste livet blir brukt opp heller" - opprinnelig sagt mens personen var siste mål; gjelder nå
            // generelt siste mål i REKKEFØLGEN, siden fastvinge-flyet er lagt til etter personen og selv
            // har overtatt den plassen, se EXERCISES.targetStrike.stages).
            const isFinalHitOfRun = exerciseState.targetHitCount >= TARGET_HITS_REQUIRED &&
                exerciseState.stageIndex >= exercise.stages.length - 1;
            // targetStrikeLivesInsufficient (IKKE bare "targetCrashesUsed >= TARGET_STRIKE_DRONE_LIVES",
            // den opprinnelige, rent reaktive sjekken) - restarter så snart det er matematisk umulig å
            // fullføre resten av de påkrevde treffene med de gjenværende dronene, ikke først når telleren
            // faktisk når 0 (brukerens krav: "hvis man resetter så mye at man får for lite liv til å
            // fullføre resten av kollisjonene må hele øvelsen resettes").
            if (!isFinalHitOfRun && targetStrikeLivesInsufficient()) {
                // Ikke nok droner igjen til å fullføre resten - hele forsøket starter forfra, ingen tid
                // logges (brukerens krav: "hele øvelsen resetter fra start og tiden nullstilles").
                restartTargetStrikeRun(exercise);
            } else if (exerciseState.targetHitCount < TARGET_HITS_REQUIRED) {
                spawnTargetHitStage(stage, true); // treff 1 av 2 - samme mål på nytt, fra patruljestart
            } else {
                advanceExerciseStage(); // to av to treff unnagjort - videre til neste måltype (eller fullført)
            }
        }
        return;
    }
    // Forveksling med den STATISKE, røde pynt-bilen ved avgangsplassen (se DECORATIVE_CAR_POSITION) - kun
    // relevant under selve bil-steget. Egen nedkjølingstid (WRONG_CAR_HINT_COOLDOWN_MS) hindrer meldingen
    // i å spamme på nytt hvert bilde mens man dveler i nærheten av den (brukerens rapport: kolliderer i
    // den røde, parkerte bilen uten noen tilbakemelding om at det var feil bil).
    if (stage.variant === "car" && now - wrongCarHintShownAt > WRONG_CAR_HINT_COOLDOWN_MS &&
        droneState.position.distanceTo(DECORATIVE_CAR_POSITION) <= WRONG_CAR_HINT_RADIUS) {
        wrongCarHintShownAt = now;
        exerciseState.warningMessage = "Feil bil! Den røde bilen her er bare pynt - du skal kollidere " +
            "med den GRØNNE bilen som kjører rundt et sted i nærheten.";
        exerciseState.warningUntil = now + 3500;
        exerciseState.warningIsSuccess = false;
    }
    const handle = targetHitHandleFor(stage.variant);
    if (!handle.visible) return; // målet venter på neste spawn (nettopp truffet) - ingenting å sjekke akkurat nå
    if (droneState.position.distanceTo(handle.position) <= TARGET_HIT_RADIUS[stage.variant]) {
        exerciseState.targetHitCount++;
        exerciseState.targetCrashesUsed++;
        // Ekte krasj (brukerens krav: "må ha kollisjonsdeteksjon på objektene så det blir et krasj") - SAMME
        // flagg en hard landing/velt bruker (se updatePhysics), IKKE bare en stille avstandssjekk med
        // umiddelbar teleport. Utløser det vanlige, sentrerte KRASJ-banneret ("Krasjet"-status i HUD-en,
        // motoren kuttes - droneen faller fritt der den er) akkurat som et ekte sammenstøt ville gjort.
        // Selve resetten under (se targetHitPendingUntil-grenen over) skjer likevel AUTOMATISK et par
        // sekunder senere - i motsetning til et vanlig krasj (som krever manuell R) trenger man ikke gjøre
        // noe selv her, se TARGET_HIT_POPUP_DELAY_MS.
        droneState.crashed = true;
        droneState.armed = false;
        // "Legg til ordentlig kollisjonsdeteksjon i objektene det skal krasjes inn i slik at man ikke bare
        // glitcher gjennom" (brukerens krav) - nullstiller farten UMIDDELBART ved selve treffet, i stedet
        // for å la den (nå motorløse, men fortsatt i fart) droneen seile videre gjennom/forbi målets
        // synlige modell i det korte øyeblikket før tyngdekraften alene tar over. Samme prinsipp som en
        // ekte, solid veggkollisjon (se pushOutOfSolidWalls, som nullstiller hastighetskomponenten INN i
        // veggen) - her nullstilles hele farten, siden et sammenstøt med et bevegelig mål ikke har noen
        // entydig, fast "veggretning" å dempe langs.
        droneState.velocity.set(0, 0, 0);
        droneState.angularVelocity.pitch = 0;
        droneState.angularVelocity.roll = 0;
        droneState.angularVelocity.yaw = 0;
        if (stage.variant === "person") {
            // Faller over ende i stedet for å bare forsvinne med det samme (brukerens krav: "han skal
            // falle over ende når man krasjer i han") - forblir SYNLIG (IKKE handle.visible=false her, i
            // motsetning til de tre andre målene under) helt til selve respawnen i pending-grenen over.
            // Samme knockPersonOver-mekanikk som updateBystanderCollision/VTOL-simulatoren.
            knockPersonOver(handle);
        } else {
            handle.visible = false;
        }
        // Liten popup med selve treff-fremdriften (X/2) - egen fra advanceExerciseStage sin "Fullført!
        // Neste: ..."-melding (som først vises idet steget FAKTISK avanserer, se over). IKKE lenger noe om
        // hvor mange droner/liv som er igjen her (brukerens krav: "ikke noe 'x droner igjen' i popupene" -
        // det tallet vises nå i stedet som ikonrekken nede i venstre hjørne, se updateTargetLivesHud).
        exerciseState.warningMessage = "Truffet " + TARGET_HIT_LABEL[stage.variant] + "! (" +
            exerciseState.targetHitCount + "/" + TARGET_HITS_REQUIRED + ")";
        exerciseState.warningUntil = now + TARGET_HIT_POPUP_DELAY_MS;
        exerciseState.warningIsSuccess = true;
        exerciseState.targetHitPendingUntil = now + TARGET_HIT_POPUP_DELAY_MS;
    }
}
// Alle dronene brukt opp (se exerciseState.targetCrashesUsed/TARGET_STRIKE_DRONE_LIVES) - hele forsøket
// starter helt forfra: tilbake til mål 1, klokken nullstilt (targetRunStartTime, satt av spawnTargetHitStage
// siden stageIndex===0 og isRetry er usann her) - i MOTSETNING til finishTimedLoopRun (som logger et
// FULLFØRT forsøk) logges INGEN tid her, siden dette er et mislykket forsøk, ikke et fullført ett. MERK:
// dette kalles ALDRI på selve det avgjørende, siste treffet i forsøket - se isFinalHitOfRun-sjekken i
// updateTargetHitStage (brukerens krav: "ved siste krasj i person gjør det ikke noe om det siste livet
// blir brukt opp heller").
function restartTargetStrikeRun(exercise) {
    exerciseState.warningMessage = "Tom for droner! Hele øvelsen starter på nytt fra mål 1.";
    exerciseState.warningUntil = performance.now() + 4000;
    exerciseState.warningIsSuccess = false;
    exerciseState.targetCrashesUsed = 0;
    exerciseState.stageIndex = 0;
    resetStageProgress();
    rebuildExerciseGuide();
    spawnTargetHitStage(exercise.stages[0]);
}
// Alle fire mål truffet - logg totaltiden (finishTimedAcroRun/recordAcroMedal, se medaljeseksjonen) og
// vis resultatpopup-en (samme showRaceResultPopup som racingbanene). IKKE lenger et umiddelbart hopp
// tilbake til steg 0 for et nytt forsøk (brukerens krav, ordrett: "når alle øvelsene er fullført så skal
// det stoppe opp, ikke resette. få en popup med resultatet") - selve stoppet/nullstillingen håndteres nå
// av targetRunFinishPendingUntil (sjekket øverst i updateTargetHitStage), samme "vis resultat FØRST, vent
// noen sekunder, resett SÅ"-mønster som raceFinishPendingUntil (updateRacingStage) allerede bruker for
// racingbanene. IKKE completeExercise/enterLandingPhase, som hører til det ENGANGS bestått/ikke-bestått-
// systemet (EXERCISE_ORDER), se exercise.timedLoop i advanceExerciseStage.
function finishTimedLoopRun(exercise) {
    const totalSec = (performance.now() - exerciseState.targetRunStartTime) / 1000;
    const medal = recordAcroMedal(exercise.id, totalSec);
    finishTimedAcroRun(exercise.id, totalSec, null);
    const isNewRecord = medal === "platinum" && ACRO_RECORD_SEC[exercise.id] !== undefined; // ingen kjent rekord for targetStrike ennå - alltid usann i dag
    showRaceResultPopup("Alle mål", totalSec, medal, isNewRecord, RACE_RESULT_POPUP_MS);
    exerciseState.targetActiveVariant = null; // ingenting patruljerer lenger mens vi venter
    // stageIndex klemmes TILBAKE til siste gyldige steg (IKKE latt stå på stages.length, utenfor selve
    // tabellen) - getExerciseStage() i updateExercise()s vanlige dispatch leser stage.type UBETINGET hvert
    // bilde (unntatt i landingPhase, som targetStrike aldri bruker), og ville krasjet på et undefined-
    // objekt allerede neste bilde ellers. Hvilket steg-objekt som "offisielt" står igjen spiller uansett
    // ingen rolle her - targetActiveVariant er nullstilt over, så updateTargetHitVisuals gjør ingenting,
    // og selve treffsjekken i updateTargetHitStage kjøres aldri: targetRunFinishPendingUntil-grenen der
    // fanger opp og returnerer FØR treffsjekken i det hele tatt nås.
    exerciseState.stageIndex = exercise.stages.length - 1;
    exerciseState.targetRunFinishPendingUntil = performance.now() + RACE_RESULT_POPUP_MS;
    rebuildExerciseGuide();
}

/* ---------- Øvelser: racing-tilstandsmaskin (racingbane 2, se GATE_COURSE_2_CENTER) ----------
   Fritt løpende tidsforsøk, ikke en engangs-øvelse: klokken starter idet start/mål-porten (element 0 i
   GATE_WAYPOINTS_2) krysses FØRSTE gang, og hver fullførte runde (alle porter i rekkefølge, tilbake til
   samme port) logges i den lokale ledertavlen (se finishTimedAcroRun) - deretter starter neste runde
   umiddelbart uten å måtte krysse start på nytt. wpIndex/engaged gjenbrukes fra det generelle
   øvelsessystemet (samme felt som løype-øvelsene bruker); racing-spesifikt er kun raceStartTime.
*/
// Åpningen registreres som en boks (halfW/halfH, lokalt X/Y i gatens eget rotererte plan - se
// isDroneInGateOpening) i stedet for en KULE rundt sentrum (den gamle captureRadius-modellen) - en kule
// kunne trigges ved å passere et stykke UTENFOR selve rammen også, så lenge man var nær nok senterpunktet
// i luftlinje (f.eks. rett over eller til siden av portsøylene). 0.92 av halve størrelsen gir fortsatt
// god klaring nær kantene for en drone i høy fart, uten å telle en tydelig utenfor-passering.
const RACE_GATE_CENTERS_2 = GATE_PLACEMENTS_2.map(function (placement) {
    const wp = placement.wp;
    if (wp.type === "barn") {
        return {
            x: placement.x, y: BARN_DIMENSIONS.sillY + BARN_DIMENSIONS.windowH / 2, z: placement.z,
            yaw: placement.yaw,
            halfW: BARN_DIMENSIONS.windowW / 2 * 0.92, halfH: BARN_DIMENSIONS.windowH / 2 * 0.92
        };
    }
    return {
        x: placement.x, y: placement.y + wp.gap + wp.size / 2, z: placement.z,
        yaw: placement.yaw,
        halfW: wp.size / 2 * 0.92, halfH: wp.size / 2 * 0.92
    };
});
// Hvor nær selve gate-/vindusplanet (lokal Z, langs flyretningen gjennom åpningen) droneen må være for
// at en kryssing skal telle - romslig nok til å ikke miste registreringer mellom to bilder ved høy fart
// (updateRacingStage kjører én gang per RENDRET bilde, ikke per fysikk-tikk), men fortsatt tett nok til
// at det faktisk betyr "var akkurat her", ikke "et stykke unna langs banen".
const RACE_GATE_DEPTH_TOLERANCE = 1.2;
// Krever at droneen er INNENFOR selve åpningen (lokal X/Y, se over) OG nær planet (lokal Z) SAMTIDIG -
// se kommentaren ved RACE_GATE_CENTERS_2 for hvorfor dette erstattet den gamle kule-modellen. Samme
// lokal-transform-prinsipp som orientedBoxLocalXZ (rotasjon om Y med gate.yaw), for ett enkelt punkt
// (droneposisjonen) i stedet for et bokshjørne.
function isDroneInGateOpening(gate) {
    const dx = droneState.position.x - gate.x, dz = droneState.position.z - gate.z;
    const cosA = Math.cos(gate.yaw), sinA = Math.sin(gate.yaw);
    const localX = dx * cosA - dz * sinA;
    const localZ = dx * sinA + dz * cosA;
    const localY = droneState.position.y - gate.y;
    return Math.abs(localX) <= gate.halfW && Math.abs(localY) <= gate.halfH && Math.abs(localZ) <= RACE_GATE_DEPTH_TOLERANCE;
}
const RACE_START_PLACEMENT = GATE_PLACEMENTS_2[0]; // start/mål-porten, se GATE_WAYPOINTS_2
// meter "bak" start/mål-porten (mot ankomstretningen) droneen spawner. Økt videre fra 18 til 20 for enda
// mer fartsoppbygging-strekk før start/mål (brukeren ba om +10, men se regnestykket under for hvorfor det
// ikke var trygt) - klaringen mot forrige gate i løypa (den siste før man runder tilbake til start - se
// GATE_WAYPOINTS_2 sitt SISTE element, dx:-25/dz:-135->faktisk dx:-25/dz:75 - IKKE nest siste, det
// elementet ligger langt unna) krymper RASKT jo lenger tilbake man spawner: regnet ut fra
// GATE_COURSE_2_CENTER+GATE_WAYPOINTS_2 sine faktiske dx/dz-verdier er klaringen ved 18 m tilbake ~8.9 m,
// og ved 28 m tilbake (18+10, det opprinnelig forespurte hoppet) kun ~1.2 m - praktisk talt oppå selve
// porten. 20 er derfor satt som et forsiktig, trygt maksimum (~7 m klaring igjen) - IKKE øk dette videre
// uten å regne ut den avstanden på nytt (se distansen fra RACE_SPAWN_POINT til GATE_PLACEMENTS_2 sitt
// siste element - dx:-25, dz:75 relativt GATE_COURSE_2_CENTER).
const RACE_SPAWN_BACK_DIST = 20;
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
    // stage.assetsKey -> RACING_STAGE_ASSETS (definert lenger ned, ved RACE_SPAWN_YAW_TUNNEL) slår opp
    // per-bane portliste/spawn - IKKE en direkte referanse til f.eks. RACE_GATE_CENTERS_TUNNEL lagt rett
    // på stage-objektet: EXERCISES (der stage-objektene lever) evalueres tidlig i filen, lenge FØR
    // tunnelbanens konstanter i det hele tatt eksisterer, og en slik direkte referanse ville feilet med en
    // "kan ikke aksessere før initialisering"-feil. Et rent streng-nøkkel-oppslag her, gjort ved selve
    // KALLET (lenge etter hele filen er lastet), unngår problemet. race1/race3 har ingen assetsKey og
    // faller derfor tilbake til den opprinnelige banens spawn (course 2).
    const assets = RACING_STAGE_ASSETS[stage.assetsKey];
    const spawnPoint = (assets && assets.spawnPoint) || RACE_SPAWN_POINT;
    const spawnYaw = (assets && assets.spawnYaw !== undefined) ? assets.spawnYaw : RACE_SPAWN_YAW;
    droneState.position.x = spawnPoint.x;
    droneState.position.z = spawnPoint.z;
    droneState.quaternion.setFromEuler(new THREE.Euler(0, spawnYaw, 0, "YXZ"));
    settleDroneOnGround();
    exerciseState.wpIndex = 0;
    exerciseState.engaged = false;
    exerciseState.raceStartTime = 0;
    exerciseState.raceLapStartTime = 0;
    exerciseState.raceLapSplits = [];
}

// Racing-spesifikk: automatisk omstart et gitt antall sekunder etter et krasj (droneState.crashed) -
// vanlige øvelser krever fortsatt R manuelt (der er et krasj noe man skal reflektere over/lære av før
// man prøver igjen), men i racing er poenget å kjøre mange raske forsøk på rad, så et krasj skal ikke
// kreve en manuell tastetrykk-pause hver gang. IKKE for personskade (droneState.injured, treffer
// publikum/pilot) - det er bevisst utenfor scope her (bare "krasj", se brukerens ordlyd), og en slik
// hendelse bør uansett kreve et bevisst R-trykk, ikke stille forsvinne etter 2 sekunder.
const RACE_CRASH_AUTO_RESET_SEC = 1.5;
function updateRacingCrashAutoReset(now) {
    if (!droneState.crashed) {
        exerciseState.raceCrashDetectedAt = null;
        return;
    }
    if (exerciseState.raceCrashDetectedAt === null || exerciseState.raceCrashDetectedAt === undefined) {
        exerciseState.raceCrashDetectedAt = now; // første bilde krasjet oppdages - start nedtellingen
        return;
    }
    if (now - exerciseState.raceCrashDetectedAt >= RACE_CRASH_AUTO_RESET_SEC * 1000) {
        exerciseState.raceCrashDetectedAt = null;
        handleResetRequest(); // samme kodesti som å trykke R selv - respawner på start/mål (spawnRacingStage)
    }
}

// Bygger banner-teksten for et fullført forsøk på en av de fire tidsaktivitetene (kalt herfra OG fra
// finishTargetHitRun) - medaljenavn når tiden var god nok (se acroMedalForTime/recordAcroMedal), pluss -
// KUN når en ekte, kjent rekord finnes å slå (se ACRO_RECORD_SEC, i dag bare race1/race3) - en ekstra
// oppfordring om å dokumentere en ny rekord. "kanskje en liten premie" (ikke et fast løfte) - brukerens
// egen, bevisst forbeholdne ordlyd ("Slår man denne får man kanskje en liten premie"), gjentatt her
// ordrett i stedet for gjort om til noe skråsikkert.
function acroRunResultMessage(id, timeSec, medal, resultLabel) {
    let msg = resultLabel + " fullført: " + formatExerciseTime(timeSec, 2) + "!";
    if (medal) msg += " (" + acroMedalLabel(medal) + ")";
    if (medal === "platinum" && ACRO_RECORD_SEC[id] !== undefined) {
        msg += " Ny rekord! Dette kan gi en liten premie - ta skjermbilde og send det til rpas@ffi.no for dokumentasjon.";
    }
    return msg;
}

// Racingbane-resultatpopup (tid+medalje ved fullført forsøk) - EGEN fra exerciseWarningBanner (det lille
// hint-banneret øverst, brukt til korte statusmeldinger) og fra selve resett-tidspunktet (se
// raceFinishPendingUntil/updateRacingStage). Brukerens krav, ordrett: "etter fullført må det ikke
// resettes så brått. må være noen sekunder først. og en popup med tid og medalje. hvis det ble gull må
// det være ekstra stas, platinum må det være veldig ekstra stas." Bronse/sølv får et vanlig, rolig kort;
// gull et glødende/pulserende kort (tier-gold, se CSS); platinum samme pluss synlige "sparkles" og en
// egen "NY REKORD!"-linje (tier-platinum) - tydelig mer staffasje jo bedre resultatet er, ikke bare en
// tekstforskjell mellom nivåene.
const RACE_RESULT_POPUP_MS = 4500;
const raceResultPopupEl = document.getElementById("raceResultPopup");
const raceResultMedalIconEl = document.getElementById("raceResultMedalIcon");
const raceResultLabelEl = document.getElementById("raceResultLabel");
const raceResultTimeEl = document.getElementById("raceResultTime");
const raceResultMedalTextEl = document.getElementById("raceResultMedalText");
const raceResultRecordTextEl = document.getElementById("raceResultRecordText");
let raceResultPopupUntil = 0; // 0 = ingen popup vises - ellers tidspunktet den skal skjules igjen
function showRaceResultPopup(resultLabel, timeSec, medal, isNewRecord, displayMs) {
    raceResultLabelEl.textContent = resultLabel + " fullført!";
    raceResultTimeEl.textContent = formatExerciseTime(timeSec, 2);
    raceResultMedalTextEl.textContent = medal ? acroMedalLabel(medal).toUpperCase() : "";
    raceResultMedalTextEl.style.display = medal ? "" : "none";
    raceResultMedalIconEl.style.display = medal ? "" : "none";
    // sim-medal-* (bronse/sølv/gull/platinum) - samme fire fargeklasser som Acro-diplomets medaljeikoner
    // (se openAcroDiploma/renderExerciseList) - IKKE en egen fargedefinisjon her, ett sted å endre nyanser.
    raceResultMedalIconEl.className = "fa-solid fa-medal sim-race-result-medal-icon" + (medal ? " sim-medal-" + medal : "");
    ["tier-bronze", "tier-silver", "tier-gold", "tier-platinum"].forEach(function (c) { raceResultPopupEl.classList.remove(c); });
    if (medal) raceResultPopupEl.classList.add("tier-" + medal);
    raceResultRecordTextEl.textContent = isNewRecord ?
        "NY REKORD! Ta skjermbilde og send til rpas@ffi.no for dokumentasjon." : "";
    raceResultPopupEl.classList.add("show");
    raceResultPopupUntil = performance.now() + displayMs;
}
// Kalt fra animate() (samme "sjekk hvert bilde"-mønster som exerciseWarningBanner) - skjuler popup-kortet
// igjen når displayMs er over. Selve dronresetten (raceFinishPendingUntil) er en HELT separat tidtaking i
// updateRacingStage - de to har bevisst samme varighet (RACE_RESULT_POPUP_MS) i dag, men er ikke koblet
// sammen, så en fremtidig justering av den ene ikke ved et uhell endrer den andre.
function updateRaceResultPopup(now) {
    if (raceResultPopupUntil && now >= raceResultPopupUntil) {
        raceResultPopupUntil = 0;
        raceResultPopupEl.classList.remove("show");
    }
}

function updateRacingStage(stage, dt, now) {
    // Et fullført forsøk venter allerede på popup-vinduet sitt (se showRaceResultPopup) - vent til
    // raceFinishPendingUntil er over før dronen faktisk resettes til start, i stedet for å teleportere den
    // dit i SAMME øyeblikk målporten krysses (brukerens krav: "etter fullført må det ikke resettes så
    // brått. må være noen sekunder først"). Samme pending-før-handling-mønster som targetHitPendingUntil
    // (se updateTargetHitStage).
    if (exerciseState.raceFinishPendingUntil) {
        if (now >= exerciseState.raceFinishPendingUntil) {
            exerciseState.raceFinishPendingUntil = 0;
            handleResetRequest();
        }
        return;
    }
    exerciseState.headingErrorDeg = null; // ingen nese-krav i racing - fri stil, bare gjennom portene
    const gates = racingGatesForStage(stage);
    const wp = gates[exerciseState.wpIndex];
    if (!isDroneInGateOpening(wp)) return;

    const isStartGate = exerciseState.wpIndex === 0;
    // Punkt-til-punkt-baner (stage.pointToPoint, se raceTunnel): målporten er den SISTE i lista, ikke en
    // retur til gate 0 - en lineær A-til-B-bane har ingen løkke å runde. Lukkede løyper (race1/race3)
    // beholder den opprinnelige "tilbake til gate 0 = både forrige runde fullført OG neste runde startet"-
    // oppførselen (samme port er både start og mål).
    const isFinishGate = stage.pointToPoint ? exerciseState.wpIndex === gates.length - 1 : isStartGate;

    if (isStartGate && !exerciseState.engaged) {
        // Første kryssing av start(/mål)-porten - klokken starter (for HELE forsøket). Bevisst ingen
        // forhåndsvarsel om NÅR dette skjer (kun det generelle exercise.startHint ved spawn) - selve
        // klokkestarten skal oppleves akkurat idet streken krysses, som i ekte racing.
        exerciseState.engaged = true;
        exerciseState.raceStartTime = now;
        exerciseState.raceLapStartTime = now; // starten på DENNE ene runden/etappen (splitt-tid, se under)
        exerciseState.raceLapSplits = [];
        exerciseState.warningUntil = 0; // fjern start-hintet med det samme - ikke la det henge igjen
    } else if (isFinishGate && exerciseState.engaged) {
        // Én runde/etappe fullført - alle porter truffet i rekkefølge for å komme hit.
        const lapsRequired = stage.lapsRequired || 1;
        const lapSec = (now - exerciseState.raceLapStartTime) / 1000;
        exerciseState.raceLapSplits.push(lapSec);
        exerciseState.raceLapStartTime = now;
        // Punkt-til-punkt-baner har ALLTID nok "runder" ved denne (eneste) målkryssingen - de har ingen
        // løkke å runde flere ganger i samme forstand som race1/race3 (se lapsRequired-sjekken under).
        if (stage.pointToPoint || exerciseState.raceLapSplits.length >= lapsRequired) {
            // Nok runder for DETTE forsøket - lagre totaltiden (+ rundetidene hvis mer enn én, se
            // finishTimedAcroRun) og grader den (recordAcroMedal). race1 (lapsRequired=1, ikke
            // pointToPoint) treffer denne grenen på HVER kryssing (siden 1 alltid er >= 1).
            const totalSec = (now - exerciseState.raceStartTime) / 1000;
            const medal = recordAcroMedal(exerciseState.exerciseId, totalSec);
            const lapSplits = (!stage.pointToPoint && lapsRequired > 1) ? exerciseState.raceLapSplits : null;
            finishTimedAcroRun(exerciseState.exerciseId, totalSec, lapSplits);
            const resultLabel = stage.pointToPoint ? "Løypa" : (lapsRequired > 1 ? "Løp" : "Runde");
            // Egen popup (se showRaceResultPopup) i stedet for exerciseWarningBanner - brukerens krav om
            // "en popup med tid og medalje", med ekstra staffasje for gull/platinum (se CSS-tier-klassene).
            const isNewRecord = medal === "platinum" && ACRO_RECORD_SEC[exerciseState.exerciseId] !== undefined;
            showRaceResultPopup(resultLabel, totalSec, medal, isNewRecord, RACE_RESULT_POPUP_MS);
            exerciseState.engaged = false;
            exerciseState.raceLapSplits = [];
            // Punkt-til-punkt-baner og "N sammenhengende runder"-baner (race3) resetter dronen til start
            // for et nytt forsøk - men IKKE med det samme (se raceFinishPendingUntil-sjekken øverst i
            // funksjonen og RACE_RESULT_POPUP_MS) - popup-vinduet skal rekke å vises et par sekunder først
            // (brukerens krav: "etter fullført må det ikke resettes så brått. må være noen sekunder
            // først"). Enkeltrunde-banen (race1) gjør IKKE dette (se kommentaren ved lapsRequired===1-
            // grenen under) - piloten flyr videre uavbrutt, og en ny runde armes med det samme neste gang
            // gate 0 nås igjen. MÅ returnere med det samme her - ellers ville linjen under (som ellers
            // alltid kjører) overskrevet wpIndex feil mens vi venter på selve resetten.
            if (stage.pointToPoint || lapsRequired > 1) {
                exerciseState.raceFinishPendingUntil = now + RACE_RESULT_POPUP_MS;
                return;
            }
        } else {
            exerciseState.warningMessage = "Runde " + exerciseState.raceLapSplits.length + "/" + lapsRequired +
                ": " + formatExerciseTime(lapSec, 2);
            exerciseState.warningUntil = now + 2000;
            exerciseState.warningIsSuccess = true;
        }
    }
    exerciseState.wpIndex = (exerciseState.wpIndex + 1) % gates.length;
}

/* ---------- Acro-utsjekk: medaljegradering på de fire tidsaktivitetene i ACRO_EXERCISE_ORDER ----------
   Helt separat fra det stabiliserte utsjekk-programmet (EXERCISE_ORDER/exerciseProgress/allExercisesPassed
   over) - disse fire er åpne tidsforsøk (flys så mange ganger man vil, se ledertavle-seksjonen rett under),
   ikke engangs bestått/ikke-bestått-øvelser. I stedet graderes BESTE oppnådde tid på hver aktivitet til en
   medalje, og selve "utsjekken" (Acro-diplomet, se openAcroDiploma) er den DÅRLIGSTE av de fire medaljene -
   "3 gull og 1 bronse gir bronse" (brukeren). */
// Kort læringsmål-setning under hver aktivitet på selve Acro-diplomet (openAcroDiploma) - KUN
// diplomteksten, IKKE exercise.shortDescription/fullDescription (de dekker fortsatt selve øvelseslisten/
// detaljvisningen som før, uendret). Brukerens krav, ordrett: "Bør det under hver øvelse stå litt mer om
// innhold og hva som egentlig blir øvet? ... skriv det oversiktlig og elegant/passende for et diplom", senere
// presisert: "husk at det skal være dokumentasjon på hva piloten HA LÆRT. læringsmål aktig", og til slutt
// justert bort fra "mestrer"-ordlyden igjen: "ikke mestrer kanskje, men hva som kreves?" - formulert som
// KRAVET øvelsen stiller (dokumentert oppfylt ved at aktiviteten står fullført/gradert på selve diplomet,
// se overskriftsteksten "har fullført ... med en samlet gradering på ...") i stedet for et eksplisitt
// mestrings-utsagn på hver enkelt rad. Samme "kort, presist, diplom-verdig" idé som VTOL_DIPLOMA_GOALS
// (js/simulator-vtol-exercises.js) bruker for det andre diplomet.
const ACRO_DIPLOMA_SKILLS = {
    race1: "Krever varierte manøvre i høy fart - krappe svinger og høydevariasjon.",
    race3: "Krever utholdenhet og jevn presisjon gjennom tre sammenhengende runder.",
    raceTunnel: "Krever klatrende svinger, trange passasjer og krappe retningsskift.",
    targetStrike: "Krever baneberegning mot mål i bevegelse og rask reaksjon på plutselige endringer i " +
        "retning og fart, i luft og på bakken."
};
// Rene sekundtall, uavhengig av alt annet i filen, så disse kan justeres fritt uten å røre resten av
// medalje-/ledertavle-logikken. race1 er brukerens EGNE, eksakte grenser ("under 30 sek på en runde er
// gull. tregere er sølv. tregere enn 45 sek er bronse. tregere enn 1:15 er ingen gradering") - de tre
// andre er fortsatt førsteutkast ("litt usikker på passende tidsgrenser for de forskjellige. ta noe
// passende til å begynne med"), justér fritt etter faktisk spilltesting.
const ACRO_MEDAL_THRESHOLDS = {
    race1: { gold: 30, silver: 45, bronze: 75 },
    race3: { gold: 105, silver: 130, bronze: 180 },
    raceTunnel: { gold: 55, silver: 75, bronze: 110 },
    // Gull justert til brukerens eget, eksplisitte tall ("krasj i bevegelige mål kan gi gull under 1:30")
    // - opp fra 80 (1:20.0). Silver/bronse uendret (fortsatt god avstand oppover fra det nye gull-taket).
    targetStrike: { gold: 90, silver: 120, bronze: 200 }
};
// Platinum: KUN for de to eksisterende banene, der en ekte, kjent rekord finnes å slå - operativ leder
// UAS sine egne rekorder (0:26.12 på enkeltrundebanen, 1:24.05 på tre runder-banen), IKKE en vanlig
// spillers personlige beste (brukeren, presisert: "det skal ikke bli platinum. Det er kun hvis man slår
// operativ leder UAS sine rekorder som blir platinum" - se også acroDiplomaOverlay sin note i
// simulator.html, som tidligere feilaktig omtalte dette som "dine egne rekorder"). raceTunnel/targetStrike
// er nye baner uten noen etablert rekord ennå - de får ikke et platinum-nivå før en reell referansetid
// finnes (legg til flere nøkler her den dagen det er aktuelt).
const ACRO_RECORD_SEC = { race1: 26.12, race3: 84.05 };
const ACRO_MEDAL_RANK = { bronze: 1, silver: 2, gold: 3, platinum: 4 };
const ACRO_MEDAL_LABELS_NB = { bronze: "bronse", silver: "sølv", gold: "gull", platinum: "platinum" };
function acroMedalLabel(medal) { return ACRO_MEDAL_LABELS_NB[medal] || ""; }
// "Legg til ikoner med gull, sølv og bronsje medalje/pokaler" (brukeren) - egen liten HTML-rad
// (#exerciseDetailMedalRow, se showExerciseDetail) med tre fa-award-ikoner farget via de delte
// .sim-medal-*-klassene (css/style.css) - IKKE en del av selve exercise.fullDescription (som settes via
// textContent, ikke innerHTML, se showExerciseDetail - ingen HTML tillatt der). Generisk over
// ACRO_MEDAL_THRESHOLDS, så alle fire gradert tidsaktiviteter (ikke bare race1) får raden automatisk.
function medalThresholdRowHtml(id) {
    const t = ACRO_MEDAL_THRESHOLDS[id];
    if (!t) return "";
    function item(medal, text) {
        return '<span class="sim-medal-threshold-item"><i class="fa-solid fa-award sim-medal-' + medal + '"></i> ' + text + '</span>';
    }
    return item("gold", "under " + formatExerciseTime(t.gold, 0)) +
        item("silver", formatExerciseTime(t.gold, 0) + "–" + formatExerciseTime(t.silver, 0)) +
        item("bronze", formatExerciseTime(t.silver, 0) + "–" + formatExerciseTime(t.bronze, 0));
}
// Under bronsegrensen fullføres aktiviteten fortsatt (tiden logges i ledertavlen som vanlig, se
// finishTimedAcroRun), men den teller ikke som gradert ennå i utsjekk-sammenheng - returnerer null.
function acroMedalForTime(id, timeSec) {
    if (timeSec === null || timeSec === undefined) return null;
    const record = ACRO_RECORD_SEC[id];
    if (record !== undefined && timeSec < record) return "platinum";
    const t = ACRO_MEDAL_THRESHOLDS[id];
    if (!t) return null;
    if (timeSec <= t.gold) return "gold";
    if (timeSec <= t.silver) return "silver";
    if (timeSec <= t.bronze) return "bronze";
    return null;
}
// Beste medalje NOENSINNE oppnådd per aktivitet - kan bare forbedres, aldri forverres (samme "behold
// beste"-prinsipp som exerciseProgress.bestTimeSec/racingLeaderboard). Egen lagringsnøkkel (ikke en del av
// racingLeaderboard) siden dette er en avledet gradering, ikke selve rådataene (tidene) - rådataene ligger
// fortsatt kun i racingLeaderboard/racingEntriesFor.
const ACRO_MEDAL_STORAGE_KEY = "ffi-uas:acro-medals-v1";
const acroMedalProgress = Sim.loadJSON(ACRO_MEDAL_STORAGE_KEY, {});
function saveAcroMedalProgress() { Sim.saveJSON(ACRO_MEDAL_STORAGE_KEY, acroMedalProgress); }
// Oppdaterer acroMedalProgress[id] KUN hvis den nye medaljen er bedre enn den som eventuelt alt står der -
// returnerer medaljen for DENNE konkrete økten uansett (brukes til banner-/rekord-meldingen i
// finishTimedAcroRun, uavhengig av om den faktisk var en forbedring av den lagrede fremgangen).
function recordAcroMedal(id, timeSec) {
    const medal = acroMedalForTime(id, timeSec);
    if (!medal) return null;
    const prevRank = ACRO_MEDAL_RANK[acroMedalProgress[id]] || 0;
    if (ACRO_MEDAL_RANK[medal] > prevRank) {
        acroMedalProgress[id] = medal;
        saveAcroMedalProgress();
    }
    return medal;
}
// Medaljen som FAKTISK gjelder akkurat nå for en aktivitet - regnet direkte fra beste tid i ledertavlen
// (racingBestTimeSec) mot DE GJELDENDE terskelverdiene (acroMedalForTime), IKKE lest fra den lagrede
// acroMedalProgress-ratsjen over. Brukt i ALLE visnings-sammenhenger (øvelseslisten/detaljsiden/diplomet)
// - acroMedalProgress[id] brukes fortsatt til å AVGJØRE om en akkurat fullført økt var en ny personlig
// forbedring (recordAcroMedal, for selve popup-meldingen), men er IKKE en pålitelig kilde for HVA som
// faktisk vises: (1) tider logget FØR medaljesystemet fantes hadde aldri fått noen oppføring der i det
// hele tatt (viste seg som et nøytralt, "ingen medalje ennå"-ikon i menyen selv med en klart medaljeverdig
// tid), og (2) en senere justering av selve terskelverdiene (ACRO_MEDAL_THRESHOLDS, som har skjedd flere
// ganger denne økten) endrer aldri en allerede lagret medalje retroaktivt. Å regne medaljen fersk fra
// selve tiden hver gang løser begge, uten noen egen migrerings-/tilbakefyllings-logikk (brukerens rapport:
// "har noen tider fra før av, men vises som sølv i menyen etter oppdateringen?").
function currentAcroMedal(id) {
    return acroMedalForTime(id, racingBestTimeSec(id));
}
function allAcroActivitiesGraded() {
    return ACRO_EXERCISE_ORDER.every(function (id) { return !!currentAcroMedal(id); });
}
// null til alle fire har minst bronse (se allAcroActivitiesGraded) - Acro-diplomet (openAcroDiploma) skal
// ikke vise en "samlet gradering" før den faktisk betyr noe.
function overallAcroGrade() {
    if (!allAcroActivitiesGraded()) return null;
    let worst = "platinum";
    ACRO_EXERCISE_ORDER.forEach(function (id) {
        const medal = currentAcroMedal(id);
        if (ACRO_MEDAL_RANK[medal] < ACRO_MEDAL_RANK[worst]) worst = medal;
    });
    return worst;
}

/* ---------- Ledertavler for de fire tidsaktivitetene - lagres lokalt i nettleseren (samme
   Sim.loadJSON/saveJSON-mønster som exerciseProgress/settings), helt separat fra det vanlige
   øvelses-fremgangssystemet siden dette er åpne tidsforsøk (så mange runder man vil), ikke engangs
   bestått/ikke-bestått-sjekker. ---------- */
// Bumpet til v2 (fra v1) idet racingbanen ble bygget om - de gamle v1-tidene ble målt på en helt annen
// bane (kortere, andre porter) og er ikke lenger sammenlignbare. En ny nøkkel gir automatisk en tom
// ledertavle ved denne oppdateringen (Sim.loadJSON returnerer DEFAULT_RACING_LEADERBOARD for en nøkkel
// som aldri er lagret før), uten noen egen migrerings-/versjonssjekk-logikk - de gamle v1-tidene blir
// liggende urørt (og ubrukt) i nettleserens localStorage, rett og slett aldri lest igjen. Fremtidige
// baneendringer som gjør gamle tider usammenlignbare kan bumpe denne på nytt samme måte.
const RACING_LEADERBOARD_KEY = "ffi-uas:racing-leaderboard-v2";
const RACING_LEADERBOARD_MAX_ENTRIES = 20;
// Hvor lenge etter at en rekord ble satt navnet kan endres - se renderRacingLeaderboard (skjuler
// endre-ikonet etter dette) og renameRacingLeaderboardEntry (sperrer selve endringen som ekstra sikring).
const RACING_LEADERBOARD_RENAME_WINDOW_MS = 60 * 60 * 1000;
// entriesTunnel/entriesTargets lagt til her (samme lagringsnøkkel/versjon fortsatt - Sim.loadJSON fyller
// inn manglende felt fra denne default-strukturen, se defaults-merge-mønsteret, samme prinsipp som da
// entries3 selv ble lagt til - et EKSISTERENDE v2-lagret objekt uten disse to fra før raceTunnel/
// targetStrike fantes får dem tomme i stedet for undefined, uten behov for en ny nøkkel/versjon).
const DEFAULT_RACING_LEADERBOARD = { playerName: "Pilot", entries: [], entries3: [], entriesTunnel: [], entriesTargets: [] };
function loadRacingLeaderboard() {
    return Sim.loadJSON(RACING_LEADERBOARD_KEY, DEFAULT_RACING_LEADERBOARD);
}
function saveRacingLeaderboard() {
    Sim.saveJSON(RACING_LEADERBOARD_KEY, racingLeaderboard);
}
const racingLeaderboard = loadRacingLeaderboard();

// Hver av de fire tidsaktivitetene har SIN EGEN liste - tidene er ikke sammenlignbare på tvers (ulik bane/
// oppgave). id (default: den aktive øvelsen) avgjør hvilken.
const RACING_ENTRIES_FIELD = { race1: "entries", race3: "entries3", raceTunnel: "entriesTunnel", targetStrike: "entriesTargets" };
function racingEntriesFor(id) {
    const field = RACING_ENTRIES_FIELD[id || exerciseState.exerciseId] || "entries";
    return racingLeaderboard[field];
}

// Manuell nullstilling av HELE (aktive) ledertavlen (navnet i feltet øverst beholdes - kun tidene
// fjernes) - egen "er du sikker"-bekreftelse siden dette er permanent og rammer ALLE tider, ikke bare
// én. Samme mønster som renameRacingLeaderboardEntry. Rammer kun listen for øvelsen som faktisk vises
// akkurat nå (se racingEntriesFor) - ikke de andre listene på én gang.
function resetRacingLeaderboard() {
    const entries = racingEntriesFor();
    if (entries.length === 0) return;
    const ok = window.confirm(
        "Nullstille HELE ledertavlen for " + EXERCISES[exerciseState.exerciseId].label + "? Dette sletter alle " +
        entries.length + " lagrede tid(er) permanent - dette kan ikke angres."
    );
    if (!ok) return;
    entries.length = 0;
    saveRacingLeaderboard();
    renderRacingLeaderboard();
}

// Registrerer et fullført forsøk på en av de fire tidsaktivitetene: lagrer tiden i aktivitetens egen
// ledertavle (samme "beste tid øverst"-oppførsel som den gamle addRacingLapResult), OG grader den mot
// acroMedalForTime/recordAcroMedal (se medaljeseksjonen over) - én, felles funksjon for BÅDE racingbanene
// (kalt fra updateRacingStage) og mål-i-bevegelse-øvelsen (kalt fra finishTargetHitRun), i stedet for at
// hver aktivitet skulle duplisert samme lagre+grader-logikk hver for seg.
// lapSplits: kun for race3 (de tre enkeltrundetidene bak totaltiden, se updateRacingStage) - udefinert/
// utelatt ellers (de andre tre aktivitetene har kun én samlet tid, ingen egen splitt-liste å vise).
// Returnerer medaljen denne økten oppnådde (for banner-/rekord-meldingen hos kalleren), eller null hvis
// tiden ikke var god nok til å telle mot utsjekken ennå.
function finishTimedAcroRun(id, timeSec, lapSplits) {
    const entries = racingEntriesFor(id);
    const entry = { name: racingLeaderboard.playerName || "Pilot", timeSec: timeSec, dateISO: new Date().toISOString() };
    if (lapSplits && lapSplits.length > 0) entry.lapSplits = lapSplits;
    entries.push(entry);
    // Beste tid øverst - kappes til RACING_LEADERBOARD_MAX_ENTRIES for at listen ikke skal vokse i det
    // uendelige (bare de beste rundene er interessante uansett).
    entries.sort(function (a, b) { return a.timeSec - b.timeSec; });
    if (entries.length > RACING_LEADERBOARD_MAX_ENTRIES) entries.length = RACING_LEADERBOARD_MAX_ENTRIES;
    saveRacingLeaderboard();
    renderRacingLeaderboard();
}
function renderRacingLeaderboard() {
    const listEl = document.getElementById("racingLeaderboardList");
    if (!listEl) return;
    listEl.innerHTML = "";
    const entries = racingEntriesFor();
    if (entries.length === 0) {
        const empty = document.createElement("div");
        empty.className = "sim-racing-lb-empty";
        empty.textContent = "Ingen tider ennå - fullfør en runde!";
        listEl.appendChild(empty);
        return;
    }
    entries.forEach(function (entry, i) {
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
        const hasSplits = entry.lapSplits && entry.lapSplits.length > 0;
        // Klikk på TIDEN (ikke hele raden - navnet/rangen skal ikke reagere) åpner en rullegardin med
        // hver enkelt rundetid som utgjør totaltiden (kun race3 - race1-oppføringer har ingen splitt-
        // liste å vise, se finishTimedAcroRun).
        let splitsEl = null;
        if (hasSplits) {
            time.classList.add("sim-racing-lb-time-expandable");
            time.title = "Vis rundetider";
            splitsEl = document.createElement("div");
            splitsEl.className = "sim-racing-lb-splits";
            splitsEl.style.display = "none";
            entry.lapSplits.forEach(function (lapSec, li) {
                const line = document.createElement("div");
                line.textContent = "Runde " + (li + 1) + ": " + formatExerciseTime(lapSec, 2);
                splitsEl.appendChild(line);
            });
            time.addEventListener("click", function () {
                splitsEl.style.display = splitsEl.style.display === "none" ? "" : "none";
            });
        }
        // Endre-knapp for en allerede lagret tid (ikke feltet øverst - det setter kun navnet på
        // FREMTIDIGE runder) - for å rette et feilskrevet navn i etterkant, se renameRacingLeaderboardEntry.
        // Kun tilgjengelig i RACING_LEADERBOARD_RENAME_WINDOW_MS etter at rekorden ble satt - deretter
        // forsvinner ikonet, slik at man ikke kan gå tilbake og endre navn på gamle/andres tider.
        const ageMs = entry.dateISO ? (Date.now() - new Date(entry.dateISO).getTime()) : Infinity;
        row.appendChild(rank);
        row.appendChild(name);
        row.appendChild(time);
        if (ageMs <= RACING_LEADERBOARD_RENAME_WINDOW_MS) {
            const editBtn = document.createElement("button");
            editBtn.type = "button";
            editBtn.className = "sim-racing-lb-edit";
            editBtn.title = "Endre navn på denne tiden";
            editBtn.innerHTML = '<i class="fa-solid fa-pen"></i>';
            editBtn.addEventListener("click", function () { renameRacingLeaderboardEntry(i); });
            row.appendChild(editBtn);
        }
        listEl.appendChild(row);
        if (splitsEl) listEl.appendChild(splitsEl);
    });
}
// Endrer navnet på en ALLEREDE lagret tid (i motsetning til navnefeltet øverst, som kun gjelder
// runder man fullfører etter at man har endret det) - typisk for å rette et feilskrevet navn.
// Bevisst en "er du sikker"-bekreftelse først (ikke bare et rett-frem endre-felt): listen er felles for
// alle som bruker denne nettleseren/maskinen, og uten friksjon her hadde det vært for lett å stille
// endre navnet på ANDRES tid og "jukse" til seg æren for en runde man ikke selv fløy.
function renameRacingLeaderboardEntry(i) {
    const entry = racingEntriesFor()[i];
    if (!entry) return;
    const ageMs = entry.dateISO ? (Date.now() - new Date(entry.dateISO).getTime()) : Infinity;
    if (ageMs > RACING_LEADERBOARD_RENAME_WINDOW_MS) return; // fristen for å endre navn er ute
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
function racingBestTimeSec(id) {
    const entries = racingEntriesFor(id);
    return entries.length > 0 ? entries[0].timeSec : null;
}

/* ---------- Øvelser: panel-UI (liste + detaljvisning) ---------- */
function renderExerciseList(category) {
    const container = document.getElementById("exerciseListItems");
    container.innerHTML = "";

    if (category === "acro") {
        // Acro-bekreftelse-rad ØVERST, samme "mest fremtredende plassen"-begrunnelse som den stabiliserte
        // diplom-raden under - vises først når alle fire aktivitetene har nådd minst bronse (se
        // allAcroActivitiesGraded/acroMedalProgress, medaljeseksjonen).
        if (allAcroActivitiesGraded()) {
            const acroDiplomaRow = document.createElement("button");
            acroDiplomaRow.type = "button";
            acroDiplomaRow.className = "sim-exercise-row sim-exercise-row-diploma";
            acroDiplomaRow.innerHTML =
                '<span class="sim-exercise-row-icon"><i class="fa-solid fa-award"></i></span>' +
                '<span class="sim-exercise-row-main">' +
                '<span class="sim-exercise-row-title">Acro-bekreftelse</span>' +
                '<span class="sim-exercise-row-desc">Alle fire aktivitetene graderte! Samlet gradering: ' +
                acroMedalLabel(overallAcroGrade()) + ".</span>" +
                "</span>";
            acroDiplomaRow.addEventListener("click", openAcroDiploma);
            container.appendChild(acroDiplomaRow);
        } else {
            // Hint ØVERST før bekreftelsen faktisk er innen rekkevidde - brukerens krav: "Før man får en
            // bekreftelse kan det stå øverst. Fullfør alle øvelsene med minimum bronsegradering for å få
            // bekreftelse på simulatorutsjekk for acro." Samme diskré .sim-panel-hint-stil som resten av
            // panelet bruker for korte hint (se f.eks. exerciseDetailProgress), IKKE en klikkbar rad -
            // ingenting å trykke på her ennå.
            const hint = document.createElement("p");
            hint.className = "sim-panel-hint sim-exercise-list-hint";
            hint.textContent = "Fullfør alle øvelsene med minimum bronsegradering for å få bekreftelse på simulatorutsjekk for acro.";
            container.appendChild(hint);
        }
        ACRO_EXERCISE_ORDER.forEach(function (id) {
            const exercise = EXERCISES[id];
            const bestSec = racingBestTimeSec(id);
            const medal = currentAcroMedal(id);
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
                    // Medaljen (IKKE en generisk grønn pokal, brukerens krav: "nå er det en grønn pokal ved
                    // siden av tiden som vises der. kan heller medaljen vises? og tiden i svart skrift.") -
                    // sim-medal-X sitter nå på selve IKONET (ikke den ytre spannen, som tidligere lot
                    // fargen "lekke" ned til tidsteksten også via CSS-arv) - se .sim-exercise-check-icon i
                    // sim-medal-X (css/style.css). Uten medalje (logget tid, men ikke raskt nok for bronse
                    // ennå) - nøytralt grå pokal-ikon i stedet for den misvisende "suksess"-grønne, siden
                    // ingenting faktisk er oppnådd ennå. Tiden i egen span med eksplisitt svart farge -
                    // uavhengig av ikonets farge uansett.
                    ? '<span class="sim-exercise-check"><i class="fa-solid ' +
                        (medal ? "fa-award sim-exercise-check-icon sim-medal-" + medal : "fa-trophy sim-exercise-check-icon") +
                        '"></i> <span class="sim-exercise-check-time">' + formatExerciseTime(bestSec, 2) + "</span></span>"
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

// Acro-bekreftelsen - HELT separat fra openDiploma over (den stabiliserte, EXERCISE_ORDER-baserte
// bekreftelsen) - se acroMedalProgress/overallAcroGrade/ACRO_EXERCISE_ORDER (medaljeseksjonen) og
// #acroDiplomaOverlay i simulator.html. Medaljer i stedet for et bestått-kryss/tid per rad, og en samlet
// gradering (den DÅRLIGSTE av de fire) øverst i stedet for en totaltid.
function openAcroDiploma() {
    const overlay = document.getElementById("acroDiplomaOverlay");
    const timesEl = document.getElementById("acroDiplomaTimes");
    timesEl.innerHTML = "";
    ACRO_EXERCISE_ORDER.forEach(function (id) {
        const rowEl = document.createElement("div");
        // sim-diploma-time-row-acro (IKKE bare sim-diploma-time-row - se css) - egen modifikator scopet
        // KUN til denne løkka, slik at den delte klassen (fortsatt brukt uendret av openDiploma sin egen
        // diplomTimes-liste) ikke også får den nye, stablede topp+beskrivelse-layouten ved et uhell.
        rowEl.className = "sim-diploma-time-row sim-diploma-time-row-acro";
        const bestSec = racingBestTimeSec(id);
        const medal = currentAcroMedal(id);
        const timeText = (bestSec !== null ? formatExerciseTime(bestSec, 2) : "-") +
            (medal ? " (" + acroMedalLabel(medal) + ")" : "");
        // Beskrivelsen er et <p>, IKKE et <span> (i motsetning til de to over) - unngår at den eksisterende
        // ".sim-diploma-time-row span:last-child"-regelen (blå/fet skrift, ment for selve TIDEN) også
        // treffer beskrivelsen bare fordi den tilfeldigvis også er siste barn i raden. Se css.
        rowEl.innerHTML =
            '<span class="sim-diploma-time-row-top"><span>' + EXERCISES[id].label + "</span><span>" +
            timeText + "</span></span>" +
            '<p class="sim-diploma-time-row-desc">' + ACRO_DIPLOMA_SKILLS[id] + "</p>";
        timesEl.appendChild(rowEl);
    });
    // Utstyrsspesifikasjon - brukerens krav: "få med hva slags drone som er brukt? vekt og thrust to
    // weight ratio? så dokumentasjonen blir skikkelig", senere presisert bort fra DRONE_CLASSES.racing sin
    // egen menyvalg-tekst ("Racing (rask, lett) er vel ikke så proft? kanskje quad - størrelse - vekt -
    // TWR?") - DRONE_CLASSES.racing.label er ment for innstillinger-nedtrekksmenyen (kort/uformell), IKKE
    // en spesifikasjonslinje på selve diplomet, derfor en egen, mer "proff" fast tekst ("Racing-quadkopter")
    // her i stedet. Wheelbase (motor-til-motor, X-oppsett - se getLegTopLocalPositions) og TWR er fortsatt
    // regnet direkte ut fra kildekonstantene (IKKE hardkodet) slik at begge automatisk følger med om
    // DRONE_ARM_LENGTH/DRONE_CLASSES.racing justeres siden - se buildDrone (armLength=DRONE_ARM_LENGTH,
    // deretter droneGroup.scale.setScalar(spec.visualScale)) for hvorfor visualScale hører med i selve
    // avstandsberegningen: den skalerer HELE den bygde modellen (motorposisjoner inkludert), ikke bare et
    // rent kosmetisk overlegg.
    // Propellerstørrelse i tommer ("hvor mange inch blir det?... er det propellerstørrelsen de referer
    // til når de sier f.eks. 10 inch quad?" - brukeren) - JA, "X inch/tommer quad" i FPV-miljøet er
    // ALLTID selve propelldiameteren (ikke wheelbase), derfor tatt med som EGEN, primær størrelses-
    // betegnelse foran wheelbase-tallet (som fortsatt står med som et sekundært, mer presist detaljmål i
    // parentes). bladeLengthForClass returnerer selve propelldiameteren (unskalert, se dens egen
    // kommentar - "tuppradius er halve bladlengden") - skaleres med visualScale på samme måte som
    // wheelbase over, og konverteres fra meter til tommer (1" = 0.0254 m).
    // "kan du sette opp spesifikasjonene linje for linje? er det ryddigere?" (brukeren) - fem separate
    // .sim-diploma-spec-line-elementer i stedet for én kommaseparert setning.
    const racingSpec = DRONE_CLASSES.racing;
    const twRatio = racingSpec.maxThrust / (racingSpec.mass * GRAVITY);
    const wheelbaseMm = Math.round(2 * DRONE_ARM_LENGTH * racingSpec.visualScale * Math.SQRT2 * 1000);
    const propDiameterIn = bladeLengthForClass("racing") * racingSpec.visualScale / 0.0254;
    const specLines = [
        "Drone: Racing-quadkopter",
        "Propeller: " + propDiameterIn.toFixed(1) + "\"",
        "Wheelbase: " + wheelbaseMm + " mm",
        "Vekt: " + racingSpec.mass.toFixed(1) + " kg",
        "TWR: " + twRatio.toFixed(1) + ":1"
    ];
    document.getElementById("acroDiplomaDroneSpec").innerHTML = specLines.map(function (line) {
        return '<span class="sim-diploma-spec-line">' + line + "</span>";
    }).join("");
    // overallAcroGrade() returnerer null kun hvis IKKE alle fire har minst bronse ennå - raden som åpner
    // dette diplomet (renderExerciseList) vises selv først når allAcroActivitiesGraded() er sann, så dette
    // burde alltid være en gyldig medalje her, men fallback til "bronse"-fargen/teksten uansett for
    // robusthet (f.eks. et direkte kall utenfra, eller en fremtidig kodesti som glemmer å sjekke gaten).
    const overallGrade = overallAcroGrade() || "bronze";
    document.getElementById("acroDiplomaOverallGradeText").textContent = acroMedalLabel(overallGrade);
    const medalIcon = document.getElementById("acroDiplomaOverallMedalIcon");
    medalIcon.className = "fa-solid fa-award sim-diploma-medal sim-medal-" + overallGrade;
    document.getElementById("acroDiplomaDate").textContent =
        "Dato: " + new Date().toLocaleDateString("nb-NO", { year: "numeric", month: "2-digit", day: "2-digit" });
    document.getElementById("acroDiplomaPrintBtn").onclick = function () {
        document.body.classList.add("printing-diploma");
        window.print();
        document.body.classList.remove("printing-diploma");
    };
    document.getElementById("acroDiplomaCloseBtn").onclick = function () {
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
    // Medalje-/tidsgrense-ikonraden (se medalThresholdRowHtml) - kun for de fire gradert tidsaktivitetene
    // (ACRO_MEDAL_THRESHOLDS har en oppføring for dem, ingen andre) - skjult for resten av øvelsene.
    const medalRowEl = document.getElementById("exerciseDetailMedalRow");
    const medalRowHtml = medalThresholdRowHtml(id);
    medalRowEl.innerHTML = medalRowHtml;
    medalRowEl.style.display = medalRowHtml ? "" : "none";
    // exercise.medalNote (valgfritt, se race1) - egen kort tekst (platinum-rekorden osv.) vist rett UNDER
    // selve medalje-ikon-listen over, IKKE som en del av selve fullDescription-avsnittet lenger.
    const medalNoteEl = document.getElementById("exerciseDetailMedalNote");
    medalNoteEl.textContent = exercise.medalNote || "";
    medalNoteEl.style.display = exercise.medalNote ? "" : "none";
    // Direkte snarvei til kalibreringspanelet for øvelser som krever en fysisk kill-bryter (kun ex11 pt.
    // nå) - vises UANSETT om bryteren allerede er bundet eller ikke (i motsetning til gateBlocked-
    // varselet under, som kun vises når den IKKE er det) - praktisk å ha lett tilgjengelig for å
    // dobbeltsjekke/endre bindingen uten å måtte huske hvor Innstillinger-menyen faktisk ligger.
    document.getElementById("exerciseDetailCalibrationRow").style.display = exercise.requiresGamepadKill ? "" : "none";

    const progressEl = document.getElementById("exerciseDetailProgress");
    const startBtn = document.getElementById("exerciseStartBtn");
    const cancelBtn = document.getElementById("exerciseCancelBtn");
    progressEl.classList.remove("sim-exercise-gate-warning");
    startBtn.disabled = false;
    if (exerciseState.active && exerciseState.exerciseId === id) {
        progressEl.style.display = "";
        const isRepeatedExercise = isRepeatedLandingStage(exercise.stages[0]);
        if (exerciseState.awaitingNext) {
            // stageIndex kan stå forbi siste steg her (vanlige flerstegs-øvelser) - stegobjektet er
            // IKKE gyldig å lese, se samme resonnement i updateExerciseHud.
            progressEl.textContent = "Bestått! Se oppsummeringskortet for å gå videre.";
        } else if (exerciseState.landingPhase && !isRepeatedExercise) {
            progressEl.textContent = "Pågår: landing på H-plassen";
        } else if (isRepeatedExercise) {
            const stage0 = exercise.stages[0];
            const reqReps = requiredRepsFor(stage0);
            // "Hover i vind" viser hold-nedtelling mens man faktisk hover (mer nyttig enn bare
            // rundetall der) - "Returner hjem" har ingen tilsvarende deltilstand å vise fram.
            progressEl.textContent = "Pågår: runde " + (exerciseState.returnRepsCompleted + 1) + "/" + reqReps +
                (stage0.type === "hoverWind" && !exerciseState.landingPhase
                    ? " - hold " + exerciseState.hoverHoldSec.toFixed(0) + "/" + stage0.holdSec + " s"
                    : (exerciseState.landingPhase ? " - land på H" : ""));
        } else {
            const stage = getExerciseStage();
            const stageLabel = stage.type === "killswitch" ? killswitchDisplayLabel() : stage.label;
            progressEl.textContent = "Pågår: " + stageLabel + (stage.type === "killswitch"
                ? " - " + killswitchStatusText()
                : stage.type === "hover"
                    ? " - " + exerciseState.hoverHoldSec.toFixed(0) + "/" + stage.holdSec + " s"
                    : stage.type === "racing"
                        ? (exerciseState.engaged ? " - port " + exerciseState.wpIndex + "/" + racingGatesForStage(stage).length : " - kryss start/mål")
                        : stage.type === "targetHit"
                            ? " - mål " + (exerciseState.stageIndex + 1) + "/" + EXERCISES.targetStrike.stages.length
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
            progressEl.innerHTML = '<i class="fa-solid fa-triangle-exclamation"></i> Krever at Kill/Arm-knappen er bundet til en fysisk fjernkontroll. ' +
                '<button type="button" id="gateOpenCalibrationBtn">Åpne kalibrering</button>';
            document.getElementById("gateOpenCalibrationBtn").addEventListener("click", function () {
                togglePanel(document.getElementById("gamepadPanel"));
            });
        } else if (ACRO_EXERCISE_ORDER.indexOf(id) !== -1) {
            // Gradert tidsaktivitet (racingbanene + targetStrike, se ACRO_EXERCISE_ORDER/acroMedalProgress)
            // - generalisert fra det opprinnelige "exercise.stages[0].type === 'racing'" (som ikke fanget
            // targetStrike, som ikke er type "racing") til selve medlemskapet i listen.
            const bestSec = racingBestTimeSec(id);
            progressEl.style.display = bestSec !== null ? "" : "none";
            if (bestSec !== null) {
                const medal = currentAcroMedal(id);
                progressEl.textContent = "Beste tid: " + formatExerciseTime(bestSec, 2) +
                    (medal ? " (" + acroMedalLabel(medal) + ")" : "");
            }
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
    viewportWatcher();

    updateWind(frameDt);
    updateClouds(frameDt);
    updateInput(frameDt);
    updatePersonFalls(frameDt); // ubetinget, samme mønster som VTOL-simulatoren - folkemengden er alltid til stede
    while (accumulator >= FIXED_DT) {
        stepPhysics(FIXED_DT);
        accumulator -= FIXED_DT;
    }
    // Etter fysikk-løkken - trenger droneState.position slik DENNE frame'n faktisk endte, ikke forrige.
    updateInCloudFog();
    updateExercise(frameDt, now);
    updateKillswitchVisuals(now, frameDt);
    updateTargetHitVisuals(now);

    updateDroneVisual(frameDt);
    // Sikrer at FPV-kameraet faktisk står i den lagrede/husk­ede vinkelen kontinuerlig (ikke bare når
    // den ble satt via initScene/øvelsesstart/slideren) - uten denne kunne kameraet vise 0° etter en
    // reload helt til man dro i fpvTiltInput-slideren selv, selv om innstillingen viste riktig verdi.
    fpvCamera.rotation.x = THREE.MathUtils.degToRad(settings.fpvTiltDeg);
    chaseCameraController.update(frameDt, droneState.position, droneState.quaternion);
    updateVlosCamera();
    updateWindsockVisual(now);
    treeSwayManager.update(now, currentWindVector);
    updateClockTower(Date.now());
    updateFlags(now);
    updateShadowCamera();
    updateExerciseGuideVisual(now);
    updateHud();
    updateTargetLivesHud();
    updateRaceResultPopup(now);
    updateExerciseHud();
    updateSignalOverlay(now);
    updateFpvHud();
    // Himmelkulen følger kameraet (IKKE verdensorigo) hvert bilde - den ble tidligere lagt til FAST ved
    // (0,0,0) og aldri flyttet (se initScene). Et BackSide-material rendrer bare INNSIDEN av kulen, altså
    // kun mens kameraet faktisk befinner seg INNI den (radius 1400) - Til topps-banen strekker seg alene
    // til z≈620-730, og med litt høyde/kamerabevegelse oppå det er man fort langt nok fra origo til å ende
    // UTENFOR selve kule-meshen i egen synsretning. Der kuller BackSide forsiden nærmest kameraet, og
    // WebGL sin tomme clear-color (svart) skinner gjennom nøyaktig i det hullet - IKKE andre steder i bildet
    // (brukerens rapport, presist: "svart sirkel... alltid omtrent sentrert der jeg peker nesa... vokser
    // jo lengre unna jeg peker nesa" = et hull som vokser jo lenger UTENFOR kula kameraet befinner seg, i
    // nøyaktig den retningen man ser). Ved å sentrere selve kule-meshen på kameraet hvert bilde er kameraet
    // GARANTERT alltid inni den, uansett hvor langt fra origo man flyr - standard "uendelig himmelkule"-
    // teknikk, ingen synlig endring i utseende (kula har ingen posisjonsbunden detaljer, bare en jevn
    // gradient) siden radiusen (1400) er langt større enn noen reell flyavstand fra kameraet i én frame.
    // getWorldPosition (IKKE activeCamera.position direkte) - fpvCamera er et BARN av droneGroup (se
    // initScene: droneGroup.add(fpvCamera)), så .position der er LOKAL (offset fra droneen), ikke faktisk
    // verdensposisjon. Å kopiere den lokale offseten direkte ga en himmelkule som satt fast nesten ved
    // origo (kun forskjøvet med den vesle FPV-monteringsoffseten) i stedet for å følge droneen rundt i
    // verden - akkurat like langt fra riktig som å ikke flytte den i det hele tatt så snart man faktisk
    // flyr i FPV (brukerens rapport: "himmelbugen er der fortsatt" - forrige forsøk løste det aldri for
    // den mest brukte kameramodusen). chaseCamera/vlosCamera er ikke barn av noe (world space allerede),
    // så getWorldPosition er trygt/korrekt for alle tre kameraene uansett.
    activeCamera.getWorldPosition(skyRecenterPos);
    skyMesh.position.copy(skyRecenterPos);
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
    buildModePopover();
    initFlightLogPanel();
    renderExerciseList();

    document.getElementById("resetRatesBtn").addEventListener("click", function () {
        ["roll", "pitch", "yaw"].forEach(function (axis) { rates[axis] = Object.assign({}, DEFAULT_RATES[axis]); });
        rates.throttle = Object.assign({}, DEFAULT_RATES.throttle);
        saveRates();
        buildRatesPanel();
    });

    const calibrateAxesStatusEl = document.getElementById("calibrateAxesStatus");
    document.getElementById("calibrateAxesBtn").addEventListener("click", function () {
        const gp = getActiveGamepad();
        if (!gp) { calibrateAxesStatusEl.textContent = "Ingen fjernkontroll tilkoblet."; return; }
        const btn = document.getElementById("calibrateAxesBtn");
        btn.disabled = true;
        axisCalibrationManager.start();
        const tick = setInterval(function () {
            if (axisCalibrationManager.isActive()) {
                calibrateAxesStatusEl.textContent =
                    "Beveg alle spakene helt ut til ytterpunktene... " + Math.ceil(axisCalibrationManager.remainingMs() / 1000) + " s";
            } else {
                calibrateAxesStatusEl.textContent = "Kalibrert!";
                btn.disabled = false;
                clearInterval(tick);
            }
        }, 150);
    });
    // Nullstiller KUN skaleringen (se axisCalibrationManager.resetScale-kommentaren i simulator-
    // common.js) - IKKE et fullt fabrikk-reset av kanal-/reverstilordning/knappemapping, se
    // resetGamepadMapBtn rett under for det. Egen knapp rett ved siden av selve kalibrerings-knappen
    // (brukeren: "kaliberingsknappen med reset må være under kanalliste og over knappene i menyen") -
    // den enkle veien ut av en mislykket kalibrering (spakene ikke ført helt til ytterpunktene).
    document.getElementById("resetAxisCalibrationBtn").addEventListener("click", function () {
        axisCalibrationManager.resetScale();
        calibrateAxesStatusEl.textContent = "Kalibrering nullstilt.";
    });
    document.getElementById("resetGamepadMapBtn").addEventListener("click", function () {
        ["throttle", "roll", "pitch", "yaw"].forEach(function (ch) { gamepadMap[ch] = Object.assign({}, DEFAULT_GAMEPAD_MAP[ch]); });
        gamepadMap.buttons = Object.assign({}, DEFAULT_GAMEPAD_MAP.buttons);
        saveGamepadMap();
        const pad = getActiveGamepad() || rawFirstGamepad();
        if (pad) buildGamepadPanel(pad); else buildGamepadButtonsPanel();
        calibrateAxesStatusEl.textContent = "";
    });

    document.getElementById("resetBtn").addEventListener("click", handleResetRequest);
    document.getElementById("armToggleBtn").addEventListener("click", function () { toggleKill("button"); });

    const settingsMenuEl = document.getElementById("settingsMenu");
    Sim.setupDropdown(document.getElementById("settingsToggleBtn"), settingsMenuEl);
    Sim.wirePanelCloseButtons(settingsMenuEl);
    // Fortsatt i bruk under (clearExerciseTimesBtn) - den knappen åpner ikke et panel (så togglePanel sin
    // egen closeAllMenus-lukking trigges aldri), bare kjører en direkte handling og må derfor lukke
    // Settings-menyen selv. toggleRatesBtn/-DroneCameraBtn/-WindBtn/-GamepadBtn trengte tidligere samme
    // manuelle kall, men togglePanel (se over) lukker nå selv Settings-menyen som en del av closeAllMenus.
    function closeSettingsMenu() { settingsMenuEl.classList.remove("open"); }

    document.getElementById("toggleRatesBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("ratesPanel"));
    });
    document.getElementById("toggleDroneCameraBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("droneCameraPanel"));
    });
    document.getElementById("toggleWindBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("windPanel"));
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
    // "Fri flyging" - eneste veien ut av en aktiv øvelse/racingbane tilbake til vanlig flyging UTEN å gå
    // via en spesifikk øvelses "Avbryt"-knapp (som krever at man først vet/husker hvilken øvelse som er
    // aktiv og åpner nettopp DEN sin detaljvisning) - viktigst for racing, som overstyrer kamera/
    // dronemodus/spawn helt til man aktivt avslutter den.
    document.getElementById("categoryFreeFlightBtn").addEventListener("click", function () {
        if (exerciseState.active) stopExercise();
        // stopExercise() teleporterer bevisst IKKE droneen tilbake (se kommentaren der - respekterer det
        // man holder på med midt i en øvelse) - men "Fri flyging" skal oppleves som en ren retur til
        // vanlig flyging, ikke bare "stopp der du er" et sted langt inne på racingbanen i Racing-klasse.
        resetDrone();
        document.getElementById("exercisesPanel").style.display = "none";
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
    });
    document.getElementById("exerciseDetailCalibrationBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("gamepadPanel"));
    });
    document.getElementById("toggleFlightLogBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("flightLogPanel"));
    });
    document.getElementById("fpvHudBtn").addEventListener("click", toggleFpvHud);

    const droneClassSelect = document.getElementById("droneClassSelect");
    Object.keys(DRONE_CLASSES).forEach(function (key) {
        const opt = document.createElement("option");
        opt.value = key;
        // Vekten (DRONE_CLASSES sin mass, kg) tatt med i selve valg-teksten - brukeren ba om at den skal
        // være synlig der man faktisk velger droneklasse, ikke bare implisitt i fysikken.
        opt.textContent = DRONE_CLASSES[key].label + " - " + DRONE_CLASSES[key].mass.toFixed(1) + " kg";
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
        gamepadWizard.maybeAutoOpen(e.gamepad);
    });
    window.addEventListener("gamepaddisconnected", function () {
        if (!rawFirstGamepad()) setGamepadButtonVisible(false);
        else populateInputSourceSelect();
    });
    const existingGamepad = rawFirstGamepad();
    if (existingGamepad) {
        setGamepadButtonVisible(true);
        buildGamepadPanel(existingGamepad);
        gamepadWizard.maybeAutoOpen(existingGamepad);
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
            case "Digit1": setFlightMode("stabilized"); break;
            case "Digit2": setFlightMode("althold"); break;
            case "Digit3": setFlightMode("loiter"); break;
            case "Digit4": setFlightMode("acro"); break;
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
