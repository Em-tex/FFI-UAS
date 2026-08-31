/* js/simulator-vtol-exercises.js
   Heewing T2 introprogram ("øvelsen for Heewing. ikke 'Utsjekksprogram'... 'Heewing T2 introprogram' er
   bedre?" - brukeren, omdøpt fra "Utsjekksprogram" - se HTML-panelet/-tastatursnarveien og
   showVtolExerciseSummary under for selve UI-teksten) - øvelser + diplom for VTOL-simulatoren, Heewing T2
   Cruza VTOL. Rent VLOS - ingen
   BVLOS-opplegg (brukeren: "Vi kan kalle programmet VLOS - Heewing T2 Cruza VTOL. Siden dette er utsjekk
   for å fly VLOS. Trenger ingen BVLOS opplegg"). QLOITER for VTOL-sveving, FBWA for fastvinget
   marsjflyging, QRTL for automatisk retur (Heewing er satt opp med nettopp QLOITER/FBWA/RTL som
   standardmoduser på fjernkontrollen, se ex1 sin "sett opp fjernkontrollen"-note), KILL for
   nødsituasjoner. Ex5 ("Manuell retur i vind") drilles på å finne veien hjem og lande for egen maskin uten
   RTL. Ex6 ("Uforutsette hendelser") drilles på to uvarslede feilsituasjoner - feilkalibrert kompass under
   hover, og styringstap mot publikum - se kommentaren ved hvert scenario i VTOL_SCENARIOS under. Ex0
   ("Sett opp fjernkontrollen") og ex7 ("Teoriprøve") er "special" (wizard/quiz) - rene tekst-/spørsmål-
   gjennomganger uten 3D-flyging i det hele tatt, se "Veiviser/quiz"-seksjonen lenger ned.

   Lastes ETTER simulator-vtol.js OG simulator-vtol-rtl.js (samme globale scope-mønster som disse to
   allerede bruker seg imellom - se toppkommentaren i simulator-vtol-rtl.js) - planeState, inputState,
   rtlState, rtlParams, vtolParams, trySetFlightMode, isQMode, setEngine, resetPlane, captureHome,
   lastAirspeed, RUNWAY_SPAWN_Z, RUNWAY_WIDTH, TOWN_CENTER_X/Z, MODE_LABELS, clamp, scene, THREE, Sim,
   performance ... er alle direkte tilgjengelige globale identifikatorer her, akkurat som i
   simulator-vtol-rtl.js.

   Karakteren på grading er BEVISST lettere enn quad-simulatorens øvelsessystem (js/simulator.js) - ingen
   streng bane-/avviksregistrering med rundetelling og steg-reset. Det som teller er å unngå tap av
   kontroll, "fly-away" og krasj på uegnede steder - en nødlanding/krasjlanding på et egnet, åpent område
   regnes som en akseptabel utfall, ikke et nederlag (se ex6 "flyawayCrowd"-scenarioet). */

/* ==================== Konstanter ==================== */
const VTOL_EXERCISE_STORAGE_KEY = "ffi-uas:vtol-exercises-v1";

// "Hoverøvelsene kan kortes ned til 15 sek hver. og de kan være lavere." (brukeren) - senket fra 8 til 5 m
// og fra 20 til 15 sekunders holdetid (ex1/ex2 sine fire runder).
const VTOL_HOVER_ALT = 5;           // m - hover et par meters høyde
const VTOL_HOVER_HOLD_SEC = 15;     // minst 15 sekunder stillestående hold
const VTOL_HOVER_RADIUS_M = 8;      // hvor langt unna hover-punktet man kan drive før holdetiden nullstilles
const VTOL_HOVER_PITCH_WARN_DEG = 30; // "Om denne oppleves som stor (>30 grader) - avbryt og land"
const VTOL_TRANSITION_MIN_ALT = 20; // "stig til minst 20 meters høyde - skift til en Fixed wing mode"
const VTOL_CRUISE_ALT = 40;         // m - øvelseshøyde for den fastvingede ruten
const VTOL_LANDING_RADIUS_M = 15;   // m fra hjempunktet, for godkjent manuell landing
// "ikke avslutt øvelsen med en gang drona berører bakken. må få lov til å lande skikkelig og stå der noen
// sekunder" (brukeren) - både updateLandManualStage og updateHoldModeUntilLandedStage besto tidligere i
// samme frame som onGround først ble sann, uansett hvor hardt/ustøtt landingen så ut - eleven fikk aldri
// se en faktisk stillestående landing. Nå må flyet stå uavbrutt i bakkekontakt (uten å krasje) i minst
// dette antall sekunder før steget telles som bestått.
const VTOL_LANDING_SETTLE_SEC = 2.5;
// Radius senket fra 10 til 3 m sammen med firkant-omplasseringen over (VTOL_HOVER_SQUARE_WAYPOINTS) - det
// nye 10x10 m firkantsporet har kun 10 m mellom hvert hjørne, så den gamle 10 m-radiusen ville gjort
// nesten hele firkantens indre til én sammenhengende fangstsone (hjørne 2 allerede "truffet" bare ved å
// nærme seg hjørne 1) i stedet for å faktisk kreve at eleven flyr formen. 3 m er fortsatt romsligere enn
// quad-simmens tilsvarende captureRadius=2.0 (js/simulator.js) - Heewing er en tyngre/tregere farkost å
// posisjonere presist i hover enn en quad.
const VTOL_WAYPOINT_RADIUS_HOVER = 3;
// "kan tillate større avvik her" (brukeren, ex3-landingsrunden) - økt fra 45 til 70 m: i motsetning til
// ex2 sin trange firkant er ex3 sin runde en fastvinget, hundretalls-meter-lang landingsrunde (se
// VTOL_PATTERN_LEG_M/-WIDTH_M) flydd i god fart - langt mindre presisjon å forvente/kreve enn i en
// sakteflygende hover-firkant, og en videre fangstsone gir mer albuerom for en jevn, rolig sving i stedet
// for å måtte treffe et smalt nåløye midt i svingen.
const VTOL_WAYPOINT_RADIUS_CRUISE = 70;
const VTOL_MIN_SAFE_IAS = 12;       // m/s - varselgrense i fastvinget flyging (stallfart er ca. 10 m/s)
const VTOL_MANUAL_RETURN_MIN_DIST_M = 300; // m fra hjem, ex5 - "må fly til 300 m" (brukeren)
const VTOL_FENCE_RADIUS_M = 300;    // m - typisk VLOS-geofence, horisontalt
const VTOL_FENCE_ALT_M = 120;       // "120m (vertikalt)"
// ex6 (se VTOL_SCENARIOS.toiletbowlLand/flyawayCrowd): "ikke start og avslutt øvelsen så brått. må ha noen
// sekunder på å komme inn i hva som skjer med flyet" (brukeren) - antall sekunder med normal, uforstyrret
// hover ETTER at farkosten faktisk er luftbåren (ikke fra selve øvelsesstarten - eleven bruker ulik tid på
// å ta av) før selve feilen/hendelsen inntreffer.
const VTOL_SCENARIO_SETTLE_SEC = 6;
// "øvelse 6. må være landet i 3 sekunder før reset for neste gjennomføring" (brukeren) - toiletbowlLand sin
// pass-sjekk (se update() der) godkjente tidligere i samme øyeblikk hjulene traff bakken, uten noen
// holdetid - samme prinsipp som VTOL_LANDING_SETTLE_SEC bruker for de vanlige land-manual-stegene, men et
// eget tall (brukeren ba spesifikt om 3 sek her, ikke de 2,5 sek VTOL_LANDING_SETTLE_SEC allerede bruker).
const VTOL_SCENARIO_LANDED_HOLD_SEC = 3;
// Kollisjon mot pilot/publikum (VTOL_PILOT_POSITION/VTOL_CROWD_CENTER/VTOL_BYSTANDER_HIT_RADIUS_M m.fl.) er
// flyttet til js/simulator-vtol.js - piloten/tilskuerne er nå PERMANENTE verdensobjekter og kollisjonen
// (checkVtolPersonCollision) kjøres UBETINGET hver frame, ikke bare inne i ex6 sitt flyawayCrowd-scenario
// (se kommentaren ved buildVtolCrowd der for hvorfor). flyawayCrowd (under) leser bare planeState.injured.
const VTOL_NOFIGHT_SEC = 6;         // hvor lenge en failsafe-reaksjon skal få stå urørt i sikkerhetsøvelsene

// "legg til en øvelse med motorfeil... noen sekunder på å forberede seg, ta kontroll, så stopper
// motorene" (brukeren) - ny øvelse (ex6b, se VTOL_EXERCISES/VTOL_SCENARIOS.engineFailureGlide). I
// MOTSETNING til ex6 sine to (bevisst uvarslede) scenarioer er selve varselet her en uttalt del av
// oppgaven, ikke noe å skjule.
const ENGINE_FAILURE_PREP_SEC = 6;
// "krasjen ikke er for hard" (brukeren) - CRASH_SINK_RATE (simulator-vtol.js) er allerede terskelen for
// at det i det hele tatt TELLER som et krasj (6 m/s) - denne er en STRENGERE, egen terskel for når et
// allerede-inntruffet krasj i TILLEGG regnes som "for hardt" for akkurat denne øvelsen.
const ENGINE_FAILURE_TOO_HARD_SINK_MS = 10;
// "Øvelsen er fullført etter glidelanding på rullebanen" (brukeren) - egen, sjenerøs "på/nær rullebanen"-
// sone (bredde-/lengdemargin utover selve asfalten) i stedet for kun VTOL_LANDING_RADIUS_M fra hjempunktet
// (som er et PUNKT, ikke selve rullebanestripen) - langt innenfor (se GATE_AREA_X/BUILDING_AREA_X i
// simulator-vtol.js, begge 60+ m unna senterlinjen) den "trygg avstand fra folk og hus"-marginen brukeren
// ba om, uten å måtte regne ut avstand til hvert enkelt hus/port separat.
const RUNWAY_LANDING_MARGIN_M = 20;
function isNearRunwayXZ(x, z) {
    return Math.abs(x) <= RUNWAY_WIDTH / 2 + RUNWAY_LANDING_MARGIN_M &&
        z <= RUNWAY_NEAR_Z + RUNWAY_LANDING_MARGIN_M &&
        z >= RUNWAY_NEAR_Z - RUNWAY_LENGTH - RUNWAY_LANDING_MARGIN_M;
}

const VTOL_FIXED_WING_MODES = ["manual", "fbwa", "fbwb"];
function isFixedWingMode(mode) { return VTOL_FIXED_WING_MODES.indexOf(mode) !== -1; }

function horizDistFromHome() {
    return Math.hypot(planeState.position.x - rtlState.home.x, planeState.position.z - rtlState.home.z);
}
function currentAltitude() { return Math.max(0, planeState.position.y); }
// "øvelse 4. må ha noe mer indikatorer eller meldinger til brukeren hva/hvor hen skal fly?" (brukeren) -
// ex4 flys med chase-kamera (ikke VLOS-overlegget de andre øvelsene har), og selve banemarkørene kan være
// langt unna/ute av syne - denne gir en løpende avstand+retning til NESTE veipunkt i HUD-status-feltet
// (se updateExerciseHud), relativt til flyets EGEN nese (høyre/venstre/rett fram/bak), ikke et abstrakt
// verdenskompass, siden det er det en pilot faktisk trenger å vite ("dreie til høyre" er handlingsbart,
// "312°" er det ikke uten et kompass å sammenligne mot). Utleder flyets egen yaw fra SAMME akse-rekkefølge
// (Euler "YXZ") som quaternionen selv ble bygget med (se GROUND_SPAWN_YAW_RAD/teleportAirborne i
// simulator-vtol.js: forward=(0,0,-1) rotert yawRad rundt Y), så euler.y her ER planets yaw, uendret/
// unegert - i motsetning til currentPitchBankDeg sin pitch/bank, som negeres av HELT andre, urelaterte
// grunner (se kommentaren der).
function relativeBearingText(target) {
    const dx = target.x - planeState.position.x, dz = target.z - planeState.position.z;
    const dist = Math.hypot(dx, dz);
    if (dist < 1) return "0 m";
    const targetYawRad = Math.atan2(-dx, -dz);
    const planeYawRad = new THREE.Euler().setFromQuaternion(planeState.quaternion, "YXZ").y;
    let relDeg = THREE.MathUtils.radToDeg(targetYawRad - planeYawRad);
    relDeg = ((relDeg + 180) % 360 + 360) % 360 - 180; // normalisert til (-180, 180]
    const distText = Math.round(dist) + " m";
    if (Math.abs(relDeg) < 8) return distText + " rett fram";
    if (Math.abs(relDeg) > 172) return distText + " rett bak";
    return distText + ", " + Math.round(Math.abs(relDeg)) + "° " + (relDeg < 0 ? "høyre" : "venstre");
}
function currentPitchBankDeg() {
    const euler = new THREE.Euler().setFromQuaternion(planeState.quaternion, "YXZ");
    return { pitchDeg: -THREE.MathUtils.radToDeg(euler.x), bankDeg: -THREE.MathUtils.radToDeg(euler.z) };
}
function formatMMSS(sec) {
    sec = Math.max(0, Math.round(sec));
    return Math.floor(sec / 60) + ":" + String(sec % 60).padStart(2, "0");
}

/* ==================== Veipunkter ====================
   Plassert trygt unna rullebane, portløype (vest) og bebyggelse (øst) - se konstantene i
   simulator-vtol.js (RUNWAY_WIDTH/GATE_AREA_X/BUILDING_AREA_X). */
// "firkant øvelse. firkantbane som kan følges... ha banen rett ved avgangspunktet og ikke så stor"
// (brukeren) - krympet fra en 30x30 m firkant 15-45 m unna til en 16x16 m firkant rett utenfor
// rullebanekanten. "banen er jo helt feilplassert... banen/firkanten kan være rett over avgangsplass og
// lavere. som øvelsen i quad/stabilize simmen" (brukeren) - den 16x16 m firkanten (x:10-26) lå fortsatt
// forskjøvet vekk fra selve spawn-/avgangspunktet (0, RUNWAY_SPAWN_Z), delvis oppå VLOS-pilotens egen
// posisjon (VLOS_PILOT_X=9, se simulator-vtol.js), ikke RUNDT avgangspunktet. Sentrert nå direkte over
// spawn-punktet, samme idé/størrelse som quad-simmens EXERCISE_CENTER+SQUARE_HALF_SIDE (js/simulator.js)
// - en 10x10 m firkant rett ved avgang i stedet for et sted lenger unna. Høyden senket til
// VTOL_SQUARE_ALT (under ren hover-høyde VTOL_HOVER_ALT), samme "lavere gir bedre dybdereferanse mot
// bakken"-begrunnelse som quad-simmens EXERCISE_ALTITUDE.
const VTOL_SQUARE_ALT = 3;        // m - lavere enn VTOL_HOVER_ALT, bedre dybdereferanse mot bakken fra VLOS
const VTOL_SQUARE_HALF_SIDE = 5;  // m - samme størrelse som quad-simmens SQUARE_HALF_SIDE
const VTOL_HOVER_SQUARE_WAYPOINTS = [
    { x: -VTOL_SQUARE_HALF_SIDE, z: RUNWAY_SPAWN_Z - VTOL_SQUARE_HALF_SIDE },
    { x: VTOL_SQUARE_HALF_SIDE, z: RUNWAY_SPAWN_Z - VTOL_SQUARE_HALF_SIDE },
    { x: VTOL_SQUARE_HALF_SIDE, z: RUNWAY_SPAWN_Z + VTOL_SQUARE_HALF_SIDE },
    { x: -VTOL_SQUARE_HALF_SIDE, z: RUNWAY_SPAWN_Z + VTOL_SQUARE_HALF_SIDE }
].map(function (p) { return new THREE.Vector3(p.x, VTOL_SQUARE_ALT, p.z); });

// "fastvinget rute. trenger kanskje ikke den rutesjekken? bare enkel ta av fly fixed wing en landingsrunde
// (utflyging - crosswind - downwind - base - final og land i Q mode?" (brukeren) - erstatter den gamle,
// abstrakte 3-punkts "rutesjekken" med et EKTE rektangulært trafikkmønster rundt rullebanen: de samme fire
// benene en ekte landingsrunde flys i (utflyging langs senterlinjen - kryssvind, 90° - medvind, parallelt
// med rullebanen tilbake forbi terskelen - base, 90° inn mot senterlinjen igjen). Fjerde/siste punkt
// (finale) ligger på senterlinjen rett ved hjemområdet, klar for transition-back+land-manual (se ex3 sine
// to runder under) - IKKE en lukket løkke (se closeLoop:false på selve stage-definisjonen).
// "Legg landingsrunden på andre siden av rullebanen. lettere å se fra pilotposisjonen da. og ruten som
// skal flys må være videre. alt for trangt nå" (brukeren) - VLOS-piloten står på ØST-siden (VLOS_PILOT_X,
// positiv X, se simulator-vtol.js) rett ved rullebanekanten, så en runde med kryssvind-/medvind-benet PÅ
// SAMME (øst) side som før tvang piloten til å se rett forbi/bak sin egen posisjon for å følge den lengste
// legen. Flyttet til VEST-siden (negativ X, samme side som portløypa - annen høyde, ingen kollisjon) -
// rett foran piloten i stedet for bak. Bredden økt fra 50 til 90 m (trygt utenfor GATE_AREA_X=-67 m) og
// benlengden fra 220 til 280 m - "alt for trangt" ga lite margin for en jevn, rolig sving i hver retning.
// "kule 3 må fortsatt flyttes mye nærmere kule 2 langs banen" (brukeren, 1. oppfølging) - selve punkt 3
// (base-svingen) IKKE flyttet direkte (det ville forkortet finalestrekningen VTOL_PATTERN_FINAL_LEG_M rett
// under nettopp fikk forlenget) - i stedet krympet DENNE (utflygings-/medvind-benet) fra 280 til 130 m.
// "kule 3 må fortsatt flyttes mye tidligere i banen. man må kunne se neste kule fra kule nr 2 og det gjør
// man ikke nå" (brukeren, 2. oppfølging) - fortsatt for langt (130+150=280 m totalt mellom kule 2 og 3).
// Denne gangen et konkret, etterprøvbart mål ("synlig FRA kule 2") i stedet for et abstrakt "nærmere" -
// krympet derfor BEGGE (til 60/60) i stedet for kun én av dem, ned til en total avstand på 120 m - godt
// innenfor synsvidde for en 5 m-radius markør (se addWaypointMarkers/markerRadius:5 på ex3-stagene).
// "første kule må plasseres et godt stykke unna igjen. lengre opp rullebanen" (brukeren, 3. oppfølging) -
// direkte MOTSATT av forrige runde for kule 1 spesifikt, men uten å reversere kule 2/3-synligheten
// (samme underliggende VTOL_PATTERN_LEG_M styrte FØR begge - kule 1 OG 2 delte samme, nære avstand fra
// hjem, se den forrige varianten). Splittet derfor i to UAVHENGIGE avstander: kule 1 (kun DENNE) bruker nå
// sin egen, lange VTOL_PATTERN_DEPARTURE_DIST_M, mens kule 2 beholder den nære, allerede synlighets-
// justerte VTOL_PATTERN_CROSSWIND_DIST_M - benet MELLOM kule 1 og 2 blir dermed en lengre, litt
// diagonal "klatre rett ut, sving så inn i runden"-strekning (i stedet for et strengt 90°-hjørne), mens
// selve den synlighets-sensitive kule 2->3->4-delen av runden er UENDRET fra forrige runde.
const VTOL_PATTERN_DEPARTURE_DIST_M = 220;  // m fra hjem til kule 1 - "et godt stykke unna", langt oppe langs rullebanen
// "kule nr 2 må flyttes litt tidligere i banen sin, så den er synlig når kameraet kikker mot kule nr 1. men
// samtidig ikke for nært heller" (brukeren, 4. oppfølging) - vlosCamera.lookAt(planeState.position) sporer
// PLANET kontinuerlig, så "kameraet kikker mot kule 1" betyr her retningen fra VLOS-piloten (fast posisjon,
// se VLOS_PILOT_X) til selve kule 1. Regnet ut retningsvinkelen fra piloten til hvert punkt (bearing) med
// forrige verdi (60 m): kule 1 lå på ~-177° (nesten rett ned senterlinjen, ubetydelig påvirket av egen
// avstand siden pilotens sideveis offset er lite i forhold), kule 2 på ~-116° - et gap på ~62°, godt
// utenfor halve horisontal-FOV-en (~35-38° for vlosCamera sin 50° vertikale FOV). Økt til 110 m flytter
// kule 2 sin retning til ~-138° (gap ~40°, en klar forbedring) - MERK: siden kule 2/3 nødvendigvis ligger
// på hver sin side av pilotens egen Z-posisjon (RUNWAY_SPAWN_Z), er en tilsvarende innstramning av
// kule 2->3-gapet en direkte MOTSATT dragkamp (denne økningen forverrer faktisk DET gapet noe, fra ~62° til
// ~84° - geometrisk uunngåelig med denne rutens grunnform uten en større omforming). Prioritert den
// EKSPLISITT navngitte kule 1->2-overgangen. Selve benlengden kule 1->2 er fortsatt ~130 m (ikke "for
// nært").
const VTOL_PATTERN_CROSSWIND_DIST_M = 110;  // m fra hjem til kule 2 (kryssvind-/medvind-svingen)
const VTOL_PATTERN_WIDTH_M = 90;  // avstand fra senterlinjen (kryssvind-/base-benet) - trygt utenfor GATE_AREA_X (-67 m)
// "finalen må få lov til å være mye lengre. trenger god plass til å svinge og line seg opp" (brukeren) -
// punkt 3/4 (base-svingen/finale) lå OPPRINNELIG kun "+20" forbi hjemposisjonen (RUNWAY_NEAR_Z) - ingen
// reell finalestrekning å rette opp/stabilisere seg på. Økt først til 150 m, siden krympet til 60 m for
// synlighet fra kule 2 (se VTOL_PATTERN_CROSSWIND_DIST_M over) - fortsatt tre ganger så lang som den
// opprinnelige "+20".
const VTOL_PATTERN_FINAL_LEG_M = 60;
// "siste kule kan ha litt lavere høyde og trenger heller ikke være så stor" (brukeren) - kule 4 (finale)
// markerer inngangen til transition-back+landing rett etter, så en synlig LAVERE høyde her (fortsatt godt
// over VTOL_TRANSITION_MIN_ALT=20 m) gir en naturlig visuell "på vei ned nå"-antydning i stedet for samme
// faste VTOL_CRUISE_ALT som resten av runden. markerRadius satt DIREKTE på selve Vector3-punktet (leses av
// addWaypointMarkers som et valgfritt per-punkt-overstyrer, se der) - kun denne ENE kulen, IKKE hele
// stagets markerRadius:5 (som fortsatt gjelder de tre andre).
const VTOL_PATTERN_FINALE_ALT = 25;
const VTOL_PATTERN_FINALE_MARKER_RADIUS = 3;
const VTOL_PATTERN_WAYPOINTS = [
    new THREE.Vector3(0, VTOL_CRUISE_ALT, RUNWAY_NEAR_Z - VTOL_PATTERN_DEPARTURE_DIST_M),                    // 1) avgang/utflyging
    new THREE.Vector3(-VTOL_PATTERN_WIDTH_M, VTOL_CRUISE_ALT, RUNWAY_NEAR_Z - VTOL_PATTERN_CROSSWIND_DIST_M), // 2) kryssvind-/medvind-sving
    new THREE.Vector3(-VTOL_PATTERN_WIDTH_M, VTOL_CRUISE_ALT, RUNWAY_NEAR_Z + VTOL_PATTERN_FINAL_LEG_M),      // 3) base-sving
    Object.assign(
        new THREE.Vector3(0, VTOL_PATTERN_FINALE_ALT, RUNWAY_NEAR_Z + VTOL_PATTERN_FINAL_LEG_M),              // 4) finale
        { markerRadius: VTOL_PATTERN_FINALE_MARKER_RADIUS }
    )
];

// "Ny øvelse 4 ... En runde i manuell modus med chase kamera bare for å få følelsen med flyet? fly gjennom
// portene og låvene og en runde forbi klokketårnet og fabrikken? så tilbake og lande i Qhover? ha litt
// vind?" (brukeren) - erstatter den gamle QRTL-visning-øvelsen (se ex4 sin egen kommentar lenger ned) med
// en sammenhengende, manuelt flydd rundflyging gjennom de FAKTISKE verdensobjektene (buildGateArea/
// buildBuildingArea/buildTownHall+buildFactory i simulator-vtol.js), ikke abstrakte øvingspunkter.
//
// Portene (GATE_LINE) og bygningenes vindusåpninger ligger på en fast lokal høyde OVER selve
// gruppe-origoen (Y=0, se buildGate/buildOpenBuilding), og begge grupper roteres KUN om verdens-Y-aksen
// rundt sitt eget (lokale X=0, Z=0) senter - en Y-akse-rotasjon flytter ALDRI et punkt som allerede ligger
// PÅ rotasjonsaksen, så world-XZ for hvert gjennomflygingspunkt er ganske enkelt selve gruppens egen
// posisjon, uavhengig av dens rotation.y. Kun world-Y (høyden) må regnes ut separat per objekt:
// - Port: groundGap + size/2 = 3 + 9/2 = 7.5 m (se buildGate/GATE_SIZE/GATE_GROUND_GAP)
// - Garasje 1/3: sillY + windowH/2 = 1.6 + 6/2 = 4.6 m (se buildBuildingArea sitt garage1/garage3-kall)
// - Garasje 2: sillY + windowH/2 = 1.3 + 5.5/2 = 4.05 m (se buildBuildingArea sitt garage2-kall)
// "Banen for å fly gjennom portene kan settes opp motsatt veg" (brukeren) - GATE_LINE (simulator-vtol.js,
// selve de fysiske portobjektene) er UENDRET, kun REKKEFØLGEN eleven flyr dem i her er snudd (.reverse()) -
// gir en mer naturlig fortsettelse fra klatre-/transisjonspunktet inn i banen, i stedet for original-
// rekkefølgen (nærmeste port først).
const VTOL_TOUR_GATE_WAYPOINTS = GATE_LINE.slice().reverse().map(function (g) { return new THREE.Vector3(g.x, 7.5, g.z); });
const VTOL_TOUR_BUILDING_WAYPOINTS = [
    new THREE.Vector3(BUILDING_AREA_X, 4.6, RUNWAY_NEAR_Z - 40),       // garage1
    new THREE.Vector3(BUILDING_AREA_X + 8, 4.05, RUNWAY_NEAR_Z - 120), // garage2
    new THREE.Vector3(BUILDING_AREA_X - 4, 4.6, RUNWAY_NEAR_Z - 200)   // garage3
];
// Klokketårnet/rådhuset og fabrikken er landemerker å fly FORBI, ikke gjennom (ingen åpning å sikte på) -
// derfor en langt mer sjenerøs radius (VTOL_LANDMARK_WAYPOINT_RADIUS under) enn selve port-/vindus-
// gjennomflygingene. TOWN_CENTER_X/Z og FACTORY_DX/DZ er delt med simulator-vtol.js (samme konvensjon som
// RUNWAY_NEAR_Z/GATE_AREA_X/BUILDING_AREA_X over).
// "legg runden som skal flys lavere. under tårnhøyde" (brukeren) - klokketårnet (buildClockTower, kalt fra
// buildTownHall) er ca. 5.9 m (tårnets egen sokkel over taket) + 8.8 m (selve tårnet) + 3.1 m (toppspiret)
// ≈ 17.8 m totalt fra bakken, fabrikkpipa (stackHeight) er 20 m - senket fra 25 til 14 m, trygt UNDER
// begge (og fortsatt godt over husenes egne mønehøyder ~6-8 m) for en lav "under tårnet"-forbiflyging i
// stedet for en høy oversikts-passering.
// "gul indikatorer ved klokketårn og fabrikk kan være litt større. pass på at de ikke er inni bygningene"
// (brukeren) - punktene lå tidligere REM PÅ selve klokketårnets/fabrikkpipas eget senter-XZ (kun høyden
// var senket, se merknaden over): klokketårnets murkropp (buildClockTower, kalt fra buildTownHall) spenner
// lokalt X ±1.44 m, Z -2.64..0.24 m ved 14 m høyde - waypointet (lokal X=0, Z=0) lå altså BOKSTAVELIG inni
// selve tårnkroppen, usett bak murveggen. Fabrikkpipa (radius ~1.5 m ved 14 m høyde, sentrert 3 m unna i
// lokal X) hadde et trangt ~1.5 m klaring - nok til den gamle, vesle 0.45 m-markøren, men ikke til en
// tydelig større en. Begge punkt flyttet 12 m sidelengs vekk fra selve tårnet/pipa (fortsatt "ved" samme
// landemerke, samme høyde) - gir >10 m klaring til nærmeste murvegg/pipevegg, rikelig plass til en mye
// større markør (se VTOL_LANDMARK_MARKER_RADIUS) uten at den skjærer inn i bygget. Andre strukturer i byen
// (hus, benker, flaggstang) er alle lavere enn husenes egne mønehøyder (~6-8 m, se merknaden over) - godt
// under 14 m uansett sideveis posisjon, så kun tårnet/pipa trengte denne sjekken.
const VTOL_TOUR_LANDMARK_WAYPOINTS = [
    new THREE.Vector3(TOWN_CENTER_X + 12, 14, TOWN_CENTER_Z),                          // klokketårnet/rådhuset
    new THREE.Vector3(TOWN_CENTER_X + FACTORY_DX - 12, 14, TOWN_CENTER_Z + FACTORY_DZ) // fabrikken
];
// "fløy gjennom en port i ytterkanten uten at passering ble registrert" (brukeren) - fangst måles som
// AVSTAND TIL PORTENS SENTERPUNKT (se updateWaypointsStage), men en 9 m bred åpning (GATE_SIZE) har et
// HJØRNE opptil 4.5*sqrt(2)=~6.4 m unna senteret - en passering nær kanten/hjørnet av selve åpningen kunne
// dermed ligge utenfor en radius på kun 4 m, selv om flyet tydelig passerte gjennom selve porten (den
// separate kollisjonssjekken mot selve RAMMEN, checkVtolBuildingCollision-lignende logikk for porter
// finnes ikke ennå - kun waypoint-fangsten gjelder her). Økt til 6.5 m - dekker nå selve hjørnet av
// åpningen, ikke bare midten av den.
const VTOL_GATE_WAYPOINT_RADIUS = 6.5;    // m - dekker hele portåpningen (9 m, halv diagonal ~6.4 m), ikke bare senteret
const VTOL_BUILDING_WAYPOINT_RADIUS = 3;  // m - vindusåpningene er trangere enn portene (5.5-6 m brede)
const VTOL_LANDMARK_WAYPOINT_RADIUS = 30; // m - en forbiflyging, ikke en presisjonsport
// "gul indikatorer... kan være litt større" (brukeren) - opp fra default 0.45 m (for trangt/nært, se
// markerRadius-merknaden ved addWaypointMarkers) til en tydelig synlig kule på lang avstand, trygt innenfor
// klaringen den nye VTOL_TOUR_LANDMARK_WAYPOINTS-posisjonen gir (se dens egen merknad, >10 m til nærmeste
// murvegg/pipevegg).
const VTOL_LANDMARK_MARKER_RADIUS = 3;    // m
// "ha litt vind" (brukeren) - samme "jevn vind"-styrke som ex1/ex2 sitt tredje trinn, brukt gjennom hele
// rundflygingen (se wind-feltet på hver "waypoints"-stage i VTOL_EXERCISES.ex4 under).
const VTOL_TOUR_WIND = { speed: 4, directionDeg: 50, gust: 0.2 };

/* ==================== Øvelsesdata ==================== */
// "første leksjon bør være en veileder som tar brukeren gjennom konfigurering av fjernkontrollen så alt
// blir klart satt opp til øvelsene. Og en liten gjennomgang av de forskjellige modusene" (brukeren) -
// special:"wizard" (se startVtolSpecialExercise) - REN tekst-gjennomgang, ingen 3D-flyging/fysikk
// involvert, egen overlegg (#specialExerciseOverlay i simulator-vtol.html) i stedet for HUD-baren resten
// av øvelsene bruker. Plassert FØRST i VTOL_EXERCISE_ORDER (under), IKKE nummerert "1." som resten - den
// er et forarbeid FØR selve øvelsesnummereringen, ikke selve første flyøvelse.
const ex0WizardSteps = [
    {
        title: "Velkommen",
        icon: "fa-hand-sparkles",
        body: "Før du begynner på selve flyøvelsene, sett opp fjernkontrollen riktig og bli kjent med " +
            "flymodusene du kommer til å bruke. Dette tar bare noen minutter, og sparer deg for forvirring " +
            "midt i en øvelse."
    },
    {
        title: "1) Modusbryter - tre posisjoner",
        icon: "fa-toggle-on",
        body: "Sett opp én 3-posisjonsbryter på senderen med disse tre modusene, i denne rekkefølgen (så " +
            "bryteren går fra \"sikrest\" til \"mest automatisert\" den ene veien):\n\n" +
            "• QLOITER - VTOL-hover med GPS-posisjonshold (start-/landingsmodus)\n" +
            "• FBWA - fastvinget marsjflyging med stall-/krengebegrensning\n" +
            "• QRTL - automatisk VTOL-retur til hjempunktet\n\n" +
            "Dette er Heewing T2 Cruza sitt fabrikkoppsett.\n\n" +
            "Trykk \"Sett\" under, og trykk deretter posisjonen på senderen som skal gi hver modus - " +
            "ingen behov for å lete i en egen meny.",
        bindActions: ["modeQLoiter", "modeFbwa", "modeQrtl"]
    },
    {
        title: "2) Motorstopp / KILL - egen bryter",
        icon: "fa-power-off",
        body: "Bind en EGEN, lett tilgjengelig bryter (eller knapp) til Motor Emergency Stop (AUX-funksjon " +
            "\"Motor Emergency Stop\", eller ARM/DISARM). Dette er nødstoppen din - den skal kunne nås " +
            "UTEN å lete, med tommelen fortsatt på stikkene.\n\n" +
            "Motor Emergency Stop stanser motorene momentant, men flyet forblir armert. Nytt hjempunkt " +
            "settes ikke. Vanlig disarm med venstre stikke ned og til venstre er noe annet - det " +
            "nullstiller hjempunktet og krever en ny arming før neste flytur.\n\n" +
            "Trykk \"Sett\" under for AV og PÅ, og trykk deretter bryteren på senderen i hver posisjon.",
        bindActions: ["engineOff", "engineOn"],
        // "det er jo forskjell på motor emergency stop og disarm... her skal vi bare binde emergency stop.
        // disarm er jo fast på stikkene" (brukeren) - se bindNote-kommentaren i renderSpecialExerciseStep.
        bindNote: "\"Motor AV\"/\"Motor PÅ\" under ER Motor Emergency Stop-bryteren din (de to posisjonene på samme fysiske bryter) - IKKE disarm. Disarm er kun pinne-gesten, aldri en bryter du binder her."
    },
    {
        title: "3) QLOITER - hva den gjør",
        icon: "fa-location-crosshairs",
        body: "VTOL-hover MED GPS-posisjonshold. Slipper du stikkene, bremser og holder farkosten seg i ro " +
            "over ett punkt av seg selv - den beste modusen for kontrollert take-off og landing.\n\n" +
            "Krever en god GPS- og KOMPASS-løsning for å faktisk holde posisjonen - en dårlig " +
            "kompasskalibrering gir ustø drift eller sirkling (\"toilet bowling\") selv i denne modusen, " +
            "se teoriprøven til slutt i introprogrammet for mer om dette."
    },
    {
        title: "4) QHOVER - egen bryter",
        icon: "fa-arrows-up-down",
        body: "Som QLOITER, men UTEN GPS-posisjonshold - kun høyden holdes automatisk (Alt Hold), " +
            "posisjonen må du styre selv med stikkene. Brukes i noen av øvelsene for å trene ren " +
            "svevekontroll uten at autopiloten hjelper til med posisjonen.\n\n" +
            "Bind den på en egen, ledig bryterposisjon (utenom 3-posisjonsbryteren over) - trykk \"Sett\" " +
            "under, og trykk deretter bryteren.",
        bindActions: ["modeQHover"]
    },
    {
        title: "5) FBWA - hva den gjør",
        icon: "fa-plane",
        body: "Fastvinget marsjflyging (Fly By Wire A) - du styrer med vanlige fly-stikker (krengning/" +
            "stigning), men autopiloten begrenser bank- og angrepsvinkelen automatisk slik at flyet ikke " +
            "kan steile eller krenge for hardt, uansett hvor hardt du drar i stikkene.\n\n" +
            "Løftemotorene assisterer automatisk hvis luftfarten blir for lav (Q_ASSIST_SPEED) - de kobler " +
            "seg inn igjen for å hindre steiling i stedet for å la flyet falle."
    },
    {
        title: "6) QRTL - hva den gjør",
        icon: "fa-house",
        body: "Automatisk VTOL-retur til hjempunktet - flyr fastvinget tilbake mot hjem, transiterer til " +
            "VTOL-hover når den kommer innenfor RTL_RADIUS fra hjem (se RTL-panelet), og lander av seg " +
            "selv. Er avstanden til hjem allerede mindre enn RTL_RADIUS når QRTL aktiveres, hopper den " +
            "rett over den fastvingede returen og flyr rett i VTOL-modus.\n\n" +
            "Krever et gyldig hjempunkt."
    },
    {
        title: "7) Før hver flytur",
        icon: "fa-clipboard-check",
        body: "Kalibrer kompasset på nytt (eller sjekk at det er riktig) hver gang du flytter deg til et " +
            "nytt sted, eller etter en lengre pause - metall i bakken/bilen/utstyret ditt kan forstyrre " +
            "det.\n\nVent på et solid GPS-lås før du tar av i en Q-modus - uten " +
            "det kan ikke QLOITER/QRTL holde posisjonen eller finne veien hjem."
    },
    {
        title: "Klar!",
        icon: "fa-flag-checkered",
        body: "Modusbryteren er satt opp, motorstoppen er bundet, og du vet hva hver modus gjør. Du er " +
            "klar for øvelse 1 - hover-trening."
    }
];
const VTOL_EXERCISES = {
    ex0: {
        id: "ex0", icon: "fa-sliders", label: "Sett opp fjernkontrollen",
        special: "wizard",
        shortDescription: "Veiledning: sett opp modus-/motorstopp-bryterne på senderen og lær hva hver modus gjør, før du starter øvelsene.",
        fullDescription: "En kort tekst-gjennomgang (ingen flyging) av hvordan fjernkontrollen bør settes opp for Heewing T2 Cruza, og hva hver flymodus faktisk gjør.",
        wizardSteps: ex0WizardSteps
    },
    ex1: {
        id: "ex1", icon: "fa-crosshairs", label: "1. Hover-trening",
        shortDescription: "Arm motorene selv, 4 runder hover (økende vanskelighetsgrad), land og disarm.",
        // "Be brukeren sette opp modusene på fjernkontrollen. Motorstopp må også være bindet." (brukeren) -
        // denne (den FØRSTE øvelsen alle møter) er stedet det nevnes, én gang, i stedet for gjentatt i hver
        // eneste øvelse. "Kutt ned på svada-teksten... hold det kort og konsist" (brukeren) - resten trimmet
        // til kun hva som skal gjøres og hvorfor.
        // "Første øvelse kan starte med motorene av, så får man trent på armering" (brukeren) - se
        // startDisarmed-bruken i startVtolExercise (simulator-vtol-exercises.js) og den nye
        // "await-arm"-stagen (0) under, i stedet for at resetPlane() sin vanlige "motor PÅ"-standard skal
        // gjelde ukritisk for akkurat DENNE øvelsen.
        startDisarmed: true,
        fullDescription: "Fire runder à minst " + VTOL_HOVER_HOLD_SEC + " sek stillestående hover på ca. " + VTOL_HOVER_ALT +
            " m, økende vanskelighetsgrad: QLOITER (GPS-posisjonshold) -> QHOVER (kun Alt Hold, du holder " +
            "posisjonen selv) -> QHOVER i vind -> QSTABILIZE i vind.\n\n" +
            "Land manuelt i QLOITER, og disarm igjen til slutt.",
        // "gjelder alle øvelsene. første tooltip må være det man skal gjøre, og ikke svada/dobbel
        // informasjon i tooltips. kort og konsist. trenger ikke tallnummerering" (brukeren) - "N)"-prefikset
        // kuttet fra alle stegene under (se samme kutt i resten av øvelsene).
        // "siste der kan heller være QSTABILIZE i jevn vind. og kutt 'jevn'/'skiftende' fra all info. holder
        // å bare si 'vind'" (brukeren) - runde 4 byttet fra QHOVER+skiftende vind til QSTABILIZE (enda
        // mindre automatikk enn QHOVER - verken posisjons- eller høydeholding) + SAMME faste vindstyrke som
        // runde 3 (ikke windVariable lenger) - vanskelighetsgraden øker fortsatt, nå via selve modusen i
        // stedet for vindtypen. "jevn"/"skiftende" fjernet fra begge vind-labelene, kun "vind" igjen.
        // "Gi litt mer autoritet i QLOITER?... Kan kanskje gi et lite tips om det på starten av øvelsen." ->
        // "nei det var HOVER øvelsene der vind kunne være et problem med autoritet" -> "ikke tips i øvelse
        // 5, men i 1-2 da" (brukeren) - tipset sitter derfor her, på FØRSTE vind-runde (ikke gjentatt på
        // runde 4 - samme "ikke dobbel informasjon"-prinsipp resten av stegene under følger).
        stages: [
            { type: "await-arm", label: "Arm motorene" },
            { type: "hover", label: "Hover i QLOITER", requireMode: "qloiter", holdSec: VTOL_HOVER_HOLD_SEC, targetAlt: VTOL_HOVER_ALT },
            { type: "hover", label: "Hover i QHOVER (uten posisjonshold)", requireMode: "qhover", holdSec: VTOL_HOVER_HOLD_SEC, targetAlt: VTOL_HOVER_ALT },
            {
                type: "hover", label: "Hover i QHOVER med vind", requireMode: "qhover", holdSec: VTOL_HOVER_HOLD_SEC, targetAlt: VTOL_HOVER_ALT,
                wind: { speed: 4, directionDeg: 0, gust: 0.25 },
                hint: "Tips: sett nesa inn mot vinden - flyet trenger da mindre krengevinkel for å holde posisjonen."
            },
            { type: "hover", label: "Hover i QSTABILIZE med vind", requireMode: "qstabilize", holdSec: VTOL_HOVER_HOLD_SEC, targetAlt: VTOL_HOVER_ALT, wind: { speed: 4, directionDeg: 0, gust: 0.25 } },
            { type: "land-manual", label: "Landing" }
        ]
    },
    ex2: {
        id: "ex2", icon: "fa-vector-square", label: "2. Svevemanøvrering - firkant",
        shortDescription: "4 runder firkant i hover-modus, samme vanskelighetsstige som øvelse 1.",
        fullDescription: "Fly gjennom de fire markørene i firkanten, i rekkefølge, fire runder à økende " +
            "vanskelighetsgrad (samme stige som hover-treningen): QLOITER -> QHOVER -> QHOVER i vind " +
            "-> QSTABILIZE i vind.\n\nLand manuelt i QLOITER når alle fire rundene er fullført.",
        // "endre siste gjennomføringen til stabilize i vind. fjerne 'jevn' og 'skiftende' fra tekst"
        // (brukeren) - samme endring som ex1 (se dens egen kommentar): runde 4 QSTABILIZE + samme faste
        // vindstyrke som runde 3 i stedet for QHOVER + skiftende vind.
        // Samme vind-tips som ex1 (se dens kommentar for bakgrunnen) - på FØRSTE vind-runde her også.
        stages: [
            { type: "waypoints", label: "Firkant i QLOITER", waypoints: VTOL_HOVER_SQUARE_WAYPOINTS, radius: VTOL_WAYPOINT_RADIUS_HOVER, requireMode: "qloiter" },
            { type: "waypoints", label: "Firkant i QHOVER", waypoints: VTOL_HOVER_SQUARE_WAYPOINTS, radius: VTOL_WAYPOINT_RADIUS_HOVER, requireMode: "qhover" },
            {
                type: "waypoints", label: "Firkant i QHOVER med vind", waypoints: VTOL_HOVER_SQUARE_WAYPOINTS, radius: VTOL_WAYPOINT_RADIUS_HOVER, requireMode: "qhover",
                wind: { speed: 4, directionDeg: 0, gust: 0.25 },
                hint: "Tips: sett nesa inn mot vinden - flyet trenger da mindre krengevinkel for å holde posisjonen."
            },
            { type: "waypoints", label: "Firkant i QSTABILIZE med vind", waypoints: VTOL_HOVER_SQUARE_WAYPOINTS, radius: VTOL_WAYPOINT_RADIUS_HOVER, requireMode: "qstabilize", wind: { speed: 4, directionDeg: 0, gust: 0.25 } },
            { type: "land-manual", label: "Landing" }
        ]
    },
    ex3: {
        // "kan heller hete landingsrunde" (brukeren) - "VTOL - fastvinget - VTOL" beskrev MEKANIKKEN
        // (moduskjeden), "Landingsrunde" beskriver selve ØVELSEN (en fastvinget trafikkrunde) - mer
        // gjenkjennelig tittel for eleven.
        id: "ex3", icon: "fa-plane-up", label: "3. Landingsrunde",
        // "kan starte med nesa i baneretningen" (brukeren) - VLOS-standarden ellers (GROUND_SPAWN_YAW_RAD,
        // "hale mot piloten") gir liten mening for en øvelse som starter med en fastvinget landingsrunde
        // langs selve senterlinjen (se VTOL_PATTERN_WAYPOINTS, punkt 1/2 ligger rett ned rullebanens egen
        // Z-akse) - samme spawnYawRad:0-unntak som ex4 (se dens egen kommentar), nesa rett ned rullebanen
        // fra start i stedet for på tvers.
        spawnYawRad: 0,
        // Heewing T2 Cruza er satt opp med QLOITER/FBWA/RTL som standardmoduser på fjernkontrollen
        // (brukeren) - denne øvelsen brukte tidligere FBWB, en modus flykontrolleren faktisk IKKE er
        // konfigurert med. Byttet til FBWA - stage-logikken (requireFixedWing:true, se stages under)
        // godtok uansett enhver fastvinget modus (manual/fbwa/fbwb) fra før, så dette er kun en
        // tekst-/instruksjonsrettelse, ingen endring i selve godkjenningslogikken.
        //
        // "fastvinget rute. trenger kanskje ikke den rutesjekken? bare enkel ta av fly fixed wing en
        // landingsrunde (utflyging - crosswind - downwind - base - final og land i Q mode? en runde uten
        // vind og en med vind?" (brukeren) - erstattet den gamle, abstrakte 3-punkts ruten med en ekte
        // rektangulær landingsrunde (se VTOL_PATTERN_WAYPOINTS), flydd to hele ganger (ta av - rute - land)
        // - først uten vind, så med vind - i stedet for kun én runde.
        // "kule nr 3 kan flyttes lengre opp på downwind, nærmere kule nr 2" (brukeren) - først forsøkt løst
        // med kun en større markør (markerRadius 8->14, siden kule 3 ikke fremheves stort/gult før den
        // faktisk BLIR gjeldende mål, se hide/vis-logikken i updateWaypointsStage) i stedet for å flytte
        // selve svingpunktet, siden det ville forkortet finalestrekningen forrige fiks nettopp forlenget.
        // "kule 3 må fortsatt flyttes mye nærmere kule 2 langs banen. og kulene er alt for store" (brukeren,
        // 2. oppfølging) - begge deler rettet: markerRadius senket igjen til 5 m (14 var for stor), og selve
        // avstanden mellom kule 2 og 3 krympet ved å korte ned VTOL_PATTERN_LEG_M (se konstantens egen
        // kommentar over) - IKKE ved å flytte kule 3 sin posisjon direkte, for ikke å spise av finalen.
        // "kule 3 må fortsatt flyttes mye tidligere i banen. man må kunne se neste kule fra kule nr 2 og det
        // gjør man ikke nå" (brukeren, 3. oppfølging) - fortsatt for langt (280 m totalt). Krympet denne
        // gangen BÅDE VTOL_PATTERN_LEG_M og VTOL_PATTERN_FINAL_LEG_M ned til 60 m hver (se begges egne
        // kommentarer) - total avstand mellom kule 2 og 3 nå 120 m, innenfor synsvidde for en 5 m-radius
        // markør.
        shortDescription: "Ta av, fly en fastvinget landingsrunde (utflyging-kryssvind-medvind-base-finale), land i Q-modus. 2 runder: uten og med vind.",
        fullDescription: "Ta av i QLOITER, klatre til minst " + VTOL_TRANSITION_MIN_ALT + " m, skift til " +
            "FBWA og fly en full landingsrunde rundt rullebanen (utflyging - kryssvind - medvind - base - " +
            "finale, se de gule markørene). Transiter tilbake til QLOITER på finalen og land manuelt.\n\n" +
            "To runder: én uten vind, én med jevn vind.",
        // "gjelder alle øvelsene. første tooltip må være det man skal gjøre, og ikke svada/dobbel
        // informasjon i tooltips. kort og konsist. trenger ikke tallnummerering" (brukeren) - "N)"-prefikset
        // kuttet fra alle stegene under. Klatre-steget sin egen hint ("Klatre til minst 20 m.") fjernet OGSÅ
        // - den fantes kun som en omvei for at den FØRSTE meldingen skulle nevne klatring i det hele tatt
        // (se den forrige, nå fjernede kommentaren her), men startVtolExercise viser nå selve steg-labelen
        // direkte som første melding (se dens egen kommentar) - "Klatre til minst 20 m" i BÅDE label og hint
        // samtidig ville nå vært akkurat den slags dobbel/duplisert tekst brukeren ba om å fjerne.
        stages: [
            { type: "climb", label: "Klatre til minst " + VTOL_TRANSITION_MIN_ALT + " m", minAlt: VTOL_TRANSITION_MIN_ALT },
            { type: "transition-out", label: "Overgang til FBWA" },
            { type: "waypoints", label: "Landingsrunde (uten vind)", waypoints: VTOL_PATTERN_WAYPOINTS, closeLoop: false, radius: VTOL_WAYPOINT_RADIUS_CRUISE, markerRadius: 5, requireFixedWing: true, warnLowIas: true },
            { type: "transition-back", label: "Fortsett innflygning og land i QLOITER" },
            { type: "land-manual", label: "Landing" },
            { type: "climb", label: "Klatre til minst " + VTOL_TRANSITION_MIN_ALT + " m", minAlt: VTOL_TRANSITION_MIN_ALT },
            { type: "transition-out", label: "Overgang til FBWA" },
            { type: "waypoints", label: "Landingsrunde (med vind)", waypoints: VTOL_PATTERN_WAYPOINTS, closeLoop: false, radius: VTOL_WAYPOINT_RADIUS_CRUISE, markerRadius: 5, requireFixedWing: true, warnLowIas: true, wind: { speed: 5, directionDeg: 60, gust: 0.3 } },
            { type: "transition-back", label: "Fortsett innflygning og land i QLOITER" },
            { type: "land-manual", label: "Landing" }
        ]
    },
    // Erstattet fra bunnen av - BRUKEREN: "4. QRTL - automatisk retur. øvelsen går bare ut på å bruke RTL?
    // lite givende å bare kikke på. noe annet vi kan prøve? En runde i manuell modus med chase kamera bare
    // for å få følelsen med flyet? fly gjennom portene og låvene og en runde forbi klokketårnet og
    // fabrikken? så tilbake og lande i Qhover? ha litt vind?" - den gamle øvelsen (fly 100 m unna, aktiver
    // QRTL, se på at den lander selv) droppet HELT (se avklaringen med brukeren - ren manuell rundflyging
    // i stedet, ingen automatikk igjen i denne øvelsen). Se allowFreeCamera-kommentaren (startVtolExercise/
    // toggleCamera i simulator-vtol.js) for chase-kamera-unntaket, og VTOL_TOUR_*-konstantene over for selve
    // ruta gjennom portene/bygningene/landemerkene.
    ex4: {
        id: "ex4", icon: "fa-route", label: "4. Rundflyging - bli kjent med flyet",
        allowFreeCamera: true,
        spawnYawRad: 0, // "nese med rullebaneretningen" (brukeren) - se spawnYawRad-kommentaren i startVtolExercise
        shortDescription: "Rundflyging med chase-kamera - portene i MANUAL, resten i valgfri modus - land selv i en Q-modus.",
        fullDescription: "Ta av i QLOITER, klatre til minst " + VTOL_TRANSITION_MIN_ALT + " m og transiter til " +
            "fastvinget modus. Fly deretter én sammenhengende rundflyging for å bli kjent med hvordan " +
            "flyet oppfører seg i fastvinget flyging - med chase-kamera i stedet for VLOS denne gangen:\n\n" +
            "• Gjennom de fire portene vest for rullebanen i MANUAL (obligatorisk - hele denne delen må " +
            "flys manuelt)\n" +
            "• Gjennom garasjene og forbi klokketårnet/fabrikken øst for rullebanen, i valgfri modus\n\n" +
            "Vindstille gjennom hele rundflygingen - litt vind kommer først når du returnerer hjem og skal " +
            "lande. Transiter til slutt tilbake til en Q-modus og land selv.",
        // "man blir bedt om overgang til manuel modus etter 20 m på øvelse 4. men man kan jo ikke gå direkte
        // til manual. først fbwa for å få fart" (brukeren) - transition-out (steg 2) godtar generelt enhver
        // fastvinget modus (isFixedWingMode, se updateTransitionOutStage) - lot seg dermed "juksefullføre"
        // rett til MANUAL uten om FBWA i det hele tatt, selv om selve LABELEN sa "til MANUAL" (misvisende).
        // Splittet i to: transiter FØRST til FBWA (fortsatt quad-assistert, samme mcAuthority-glidning som
        // resten av programmet - bygger fart trygt).
        // "kanskje gjennom portene er i fbwa? og etter portene er det manuel modus" (brukeren, oppfølging) -
        // await-mode-steget (opprinnelig FØR selve portene) flyttet til å komme ETTER dem i stedet: portene
        // (trangest av gjennomflygingene, rett etter selve overgangen) flys nå i FBWA - fortsatt
        // selvnivellerende/steilingssikret, lettere å treffe presist rett etter en fersk overgang - før
        // eleven bytter til rent manuell kontroll for resten av rundflygingen (bygningene/landemerkene).
        // "øvelse 5 tooltips. alt for lange... mente øvelse 4 rundflyging her" (brukeren) - feilen var
        // faktisk her hele tiden, ikke i øvelse 5 (se dens egen ex5-kommentar, reversert igjen). To
        // separate problemer, begge rettet: (1) hver label hadde et unødvendig "N)"-rundetall-prefiks -
        // kuttet på alle steg. (2) fire av stegene hadde en EGEN hint som bare gjentok labelen med andre ord
        // ("Gjennom portene" -> "Fly gjennom portene", "Forbi klokketårnet og fabrikken" -> "Fly forbi
        // klokketårnet og fabrikken" osv, se stageStartMessage - samme dupliserings-mønster som ble rettet i
        // landingsrunden/ex3) - disse hintene fjernet, selve handlingen skrevet direkte inn i labelen i
        // stedet (kun ÉN, kort setning per steg igjen). Beholdt KUN MANUAL-bytte-hintet (tastatursnarveien),
        // som faktisk gir NY informasjon, ikke bare en omskriving av labelen.
        // "Tooltippet etter flyby av klokketårnet/fabrikken skal bare være returner og land. alt for tidlig
        // å sette Q-modus ved fabrikken. skal lande ved avgangsplassen" (brukeren) - "Overgang tilbake til
        // Q-modus" instruerte reelt et umiddelbart modusbytte midt ved fabrikken, langt fra rullebanen -
        // misvisende siden selve landingen skjer ved avgangsplassen. Endret til en enkel "Returner og
        // land"-instruks i stedet (selve transition-back-steget/logikken er UENDRET, kun teksten).
        // "Uten annen trening for manuell flyving var målet om å fly gjennom trange vinduer i sterk vind en
        // ganske krevende utfordring... Jeg synes fortsatt manuell bør øves. Kanskje ta første del i
        // manuell modus gjennom de portene og resten i valgfri modus? sjekk at det ikke er lov å jukse med
        // å fjerne manuell på første delen." (brukeren) - snudd fra "porter i FBWA, garasjer+landemerker i
        // obligatorisk MANUAL" til "porter i obligatorisk MANUAL, garasjer+landemerker i valgfri modus"
        // (INGEN requireMode på de to siste waypoints-stegene lenger, se stageModeOk - godtar dermed enhver
        // modus): portene (VTOL_GATE_WAYPOINT_RADIUS, den ROMSLIGSTE åpningen av de tre) er nå stedet
        // manuell flyging faktisk øves, mens de trange vindusåpningene (VTOL_BUILDING_WAYPOINT_RADIUS,
        // trangest av alle) ikke lenger TVINGER ren manuell kontroll i vind - eleven kan velge FBWA der om
        // ønskelig. stage.requireModeThroughout (se updateWaypointsStage) er selve anti-juks-sjekken:
        // MANUAL må holdes UNAVBRUTT gjennom HELE port-steget, ikke bare i det ene øyeblikket flyet er
        // innenfor fangstradiusen til hvert enkelt punkt - et avvik nullstiller fremgangen i steget.
        stages: [
            { type: "climb", label: "Klatre til minst " + VTOL_TRANSITION_MIN_ALT + " m", minAlt: VTOL_TRANSITION_MIN_ALT },
            { type: "transition-out", label: "Overgang til fastvinget modus" },
            { type: "await-mode", label: "Bytt til MANUAL", mode: "manual", hint: "Tast 5 på tastaturet, eller bryteren på senderen." },
            {
                type: "waypoints", label: "Fly gjennom portene", waypoints: VTOL_TOUR_GATE_WAYPOINTS, closeLoop: false,
                radius: VTOL_GATE_WAYPOINT_RADIUS, requireMode: "manual", requireModeThroughout: true
            },
            {
                type: "waypoints", label: "Fly gjennom garasjene", waypoints: VTOL_TOUR_BUILDING_WAYPOINTS, closeLoop: false,
                radius: VTOL_BUILDING_WAYPOINT_RADIUS
            },
            {
                type: "waypoints", label: "Fly forbi klokketårnet og fabrikken", waypoints: VTOL_TOUR_LANDMARK_WAYPOINTS, closeLoop: false,
                radius: VTOL_LANDMARK_WAYPOINT_RADIUS,
                markerRadius: VTOL_LANDMARK_MARKER_RADIUS
            },
            // "ingen vind før man skal returnere hjem for å lande. landing med vind" (brukeren) - de tre
            // waypoints-stegene over har IKKE lenger noe wind-felt (vindstille rundflyging), vinden skrus
            // på her i stedet (se enterStageVisuals - "transition-back" kaller nå applyStageWind) og blir
            // værende gjennom resten av returen og selve landingen (land-manual under rører den ikke).
            { type: "transition-back", label: "Returner og land", wind: VTOL_TOUR_WIND },
            // "Øvelse 4 landingen MÅ ikke være i QHOVER. kan være fri Q-modus for landing" (brukeren) -
            // reverserer den tidligere "må ikke godkjenne QLOITER"-innsnevringen (se den nå utdaterte
            // kommentaren ved updateLandManualStage/stageModeOk). requireQMode (allerede støttet av
            // stageModeOk, se der) godtar enhver Q-modus i stedet for kun requireMode:"qhover".
            { type: "land-manual", label: "Landing", requireQMode: true }
        ]
    },
    // Erstattet fra bunnen av - BRUKEREN: "øvelse 5 er en vits. fjern de scenarione. denne kan heller være
    // manuell returnering hjem i vind. må fly til 300 m og så returnere hjem å lande uten bruk av RTL. 3
    // runder med forskjellig vind." De gamle sikkerhetsscenarioene (linkloss/gpsloss/fence/battery/traffic)
    // er fjernet fra VTOL_SCENARIOS - "kill" og "toiletbowl" er BEHOLDT der (flyttet inn i ex6 i stedet, se
    // der), ikke slettet.
    ex5: {
        id: "ex5", icon: "fa-wind", label: "5. Manuell retur i vind",
        shortDescription: "Fly ut " + VTOL_MANUAL_RETURN_MIN_DIST_M + " m, returner og land manuelt - uten QRTL. 3 runder, økende vind.",
        fullDescription: "Fly ut minst " + VTOL_MANUAL_RETURN_MIN_DIST_M + " m fra hjempunktet, snu og fly " +
            "hjem igjen, og land manuelt nær hjempunktet - IKKE bruk QRTL. Tre runder med økende vind.\n\n" +
            "Poenget er å øve på å finne veien hjem og lande for egen maskin når GPS ikke er tilgjengelig - " +
            "QRTL under et forsøk teller som ikke bestått for den runden.",
        // "første tooltip må jo være hva man skal gjøre. altså ta av og fly i FBWA ut til 300 m... gjelder
        // alle øvelsene, kort og konsist, trenger ikke tallnummerering" (brukeren) - "N)"-prefikset kuttet,
        // og depart-distance-labelen skriver nå ut hele den faktiske handlingen (ta av + transiter til FBWA
        // + fly ut til avstanden), ikke bare selve avstandsmålet - dette er nå ordrett det FØRSTE eleven ser
        // (se startVtolExercise, viser steg 0 sin egen label direkte).
        // "Gi litt mer autoritet i QLOITER? ... Kan kanskje gi et lite tips om det på starten av øvelsen."
        // (brukeren) -> "nei det var HOVER øvelsene der vind kunne være et problem med autoritet" ->
        // "ikke tips i øvelse 5, men i 1-2 da" (brukeren, oppfølging/korrigering) - selve
        // nese-inn-mot-vinden-tipset FLYTTET til VTOL_EXERCISES.ex1/ex2 sine egne "med vind"-runder i
        // stedet (se deres egne kommentarer) - IKKE her. Selve autoritets-økningen (MC_MAX_LEAN_ANGLE,
        // js/simulator-vtol.js) er uendret og gjelder uansett alle Q-moduser overalt, ex5 inkludert.
        stages: [
            {
                type: "depart-distance", label: "Ta av, transiter til FBWA og fly ut ≥" + VTOL_MANUAL_RETURN_MIN_DIST_M + " m (lett vind)",
                minDist: VTOL_MANUAL_RETURN_MIN_DIST_M, wind: { speed: 3, directionDeg: 45, gust: 0.2 }
            },
            { type: "return-manual", label: "Returner og land manuelt", wind: { speed: 3, directionDeg: 45, gust: 0.2 } },
            { type: "depart-distance", label: "Ta av, transiter til FBWA og fly ut ≥" + VTOL_MANUAL_RETURN_MIN_DIST_M + " m (frisk vind)", minDist: VTOL_MANUAL_RETURN_MIN_DIST_M, wind: { speed: 6, directionDeg: 160, gust: 0.3 } },
            { type: "return-manual", label: "Returner og land manuelt", wind: { speed: 6, directionDeg: 160, gust: 0.3 } },
            { type: "depart-distance", label: "Ta av, transiter til FBWA og fly ut ≥" + VTOL_MANUAL_RETURN_MIN_DIST_M + " m (kraftig kastevind)", minDist: VTOL_MANUAL_RETURN_MIN_DIST_M, wind: { speed: 9, directionDeg: 260, gust: 0.5 } },
            { type: "return-manual", label: "Returner og land manuelt", wind: { speed: 9, directionDeg: 260, gust: 0.5 } }
        ]
    },
    // Erstattet fra bunnen av - BRUKEREN: "øvelse 6. her kan vi ha toilet bowl og fly away. ikke noe auto
    // mission som skal overvåkes... Først ta av hovre og lande. underveis får man toilet bowl. skal
    // helst bytte til QHOVER og lande da... Neste runde ta av og hovre, så får man fly away mot publikum
    // og må kille." To runder, begge med en "innkomst"-forsinkelse (se VTOL_SCENARIO_SETTLE_SEC) før
    // selve feilen inntreffer - "ikke start og avslutt øvelsen så brått. må ha noen sekunder på å komme
    // inn i hva som skjer med flyet."
    ex6: {
        id: "ex6", icon: "fa-triangle-exclamation", label: "6. Uforutsette hendelser",
        // "Det må jo ikke stå o teksten nøyaktig hva som skal skje da. Kanskje bare et hint om at det er
        // lurt å vite hvor QHOVER-knappen og emergency stop-knappen er bindet" (brukeren) - short-/
        // fullDescription (og stage-labelen, se stages under - vises LØPENDE i HUD-en mens øvelsen kjører,
        // altså en enda mer direkte spoiler enn selve beskrivelsen) avslørte tidligere nøyaktig hvilken feil
        // som ville inntreffe og i hvilken rekkefølge, noe som gjorde selve poenget ("uforutsett") meningsløst
        // - eleven visste akkurat hva som kom og når. Nå kun et generelt hint (bind QHOVER/motorstopp på
        // forhånd), ikke selve scenarioinnholdet - se VTOL_SCENARIOS.toiletbowlLand/flyawayCrowd for hva som
        // FAKTISK skjer (kun avslørt der, live, via setWarning() idet feilen faktisk inntreffer).
        shortDescription: "2 korte, uvarslede hendelser du må oppdage og reagere raskt på selv.",
        fullDescription: "To hendelser. Ta av og hold en stabil hover i QLOITER i begge rundene, og vær klar til å " +
            "oppdage og reagere på det som skjer.\n\n" +
            "Lurt å vite akkurat hvor QHOVER-bryteren og Motor Emergency Stop-bryteren er bundet på " +
            "senderen FØR du starter - se øvelse 0 om du er usikker.",
        noTiming: true,
        // "Øvelse 6. trenger tooltip på starten om hva som skal gjøres" (brukeren) - selve
        // scenario-setup()-funksjonene (VTOL_SCENARIOS.toiletbowlLand/flyawayCrowd) viser allerede en egen,
        // mer beskrivende melding ("Ta av og hold en stabil hover i QLOITER.") et par sekund idet runden
        // starter, MEN kun midlertidig (4 sek) - den VEDVARENDE steg-labelen (vises resten av runden i
        // HUD-en, se exerciseHudStage) sa bare det vage "Hold stabil hover", uten selve ta av/QLOITER-
        // instruksjonen. Utvidet begge til å si det samme som setup()-meldingen, slik den vedvarende
        // labelen fortsatt er nyttig etter at den midlertidige popup-en har forsvunnet.
        stages: [
            { type: "scenario", key: "toiletbowlLand", label: "Ta av og hold stabil hover i QLOITER", showHoverMarker: true, targetAlt: VTOL_HOVER_ALT },
            { type: "scenario", key: "flyawayCrowd", label: "Ta av og hold stabil hover i QLOITER", showHoverMarker: true, targetAlt: VTOL_HOVER_ALT }
        ]
    },
    // Ny øvelse - BRUKEREN: "legg til en øvelse med motorfeil. VLOS kamera drona er i fwba oppe i lufta.
    // noen sekunder på å forberede seg, ta kontroll, så stopper motorene. Øvelsen er fullført etter
    // glidelanding på rullebanen. krasj er ok så lenge det er i trygg avstand fra folk og hus, og krasjen
    // ikke er for hard. Skal helst glidelande uten krasj." Egen id "ex6b" (IKKE omdøpt til "ex8" eller
    // ex7/ex8 forskjøvet) - unngår å måtte endre teoriprøvens (ex7) egen ID og dermed risikere å miste
    // elevers allerede lagrede fremgang der (vtolExerciseProgress, se VTOL_EXERCISE_STORAGE_KEY, er nøklet
    // på nettopp denne ID-strengen) - kun selve LABEL-tallet er justert (se ex7 sin egen label under).
    ex6b: {
        id: "ex6b", icon: "fa-bolt", label: "7. Motorfeil - nødlanding",
        shortDescription: "Motoren stopper i FBWA oppe i lufta - glid til rullebanen og land. 3 runder: én uten vind, to med vind fra hver sin kant.",
        fullDescription: "Du blir teleportert opp i FBWA. Bruk de første " + ENGINE_FAILURE_PREP_SEC +
            " sekundene til å bli komfortabel med kontrollen - deretter stopper motoren.\n\n" +
            "Glid til rullebanen og land. Tre runder: én uten vind, to med vind - fra hver sin kant.",
        noTiming: true,
        // "Ta tre runder en uten vind, og de to neste med vind. men bytt på vindretningene på de to siste"
        // (brukeren) - utvidet fra 2 til 3 runder. Samme scenario-nøkkel (engineFailureGlide) alle tre
        // (updateScenarioStage nullstiller exerciseState.scenario ved fullført/feilet forsøk, og
        // re-initialiserer via def.setup() neste tick uansett om nøkkelen er uendret) - kun stage.wind
        // skiller dem. De to siste bruker MOTSATT vindretning av hverandre (30°/210°, nøyaktig 180° fra
        // hverandre) - ikke bare "litt ulik", en reelt forskjellig glidesituasjon (med-/motvind-komponenten
        // snur helt) fra samme vindstyrke.
        stages: [
            { type: "scenario", key: "engineFailureGlide", label: "Glid til rullebanen etter motorfeil" },
            { type: "scenario", key: "engineFailureGlide", label: "Glid til rullebanen etter motorfeil (med vind)", wind: { speed: 4, directionDeg: 30, gust: 0.25 } },
            { type: "scenario", key: "engineFailureGlide", label: "Glid til rullebanen etter motorfeil (med vind fra motsatt kant)", wind: { speed: 4, directionDeg: 210, gust: 0.25 } }
        ]
    },
    // "Siste leksjon bør være en teoretisk quiz (multipple choice?) med spørsmål spesifikt for Heewing og
    // Ardupilot... Pass på at gode svaralternativer, så det ikke er åpenbart hva som er riktig. Søk på
    // nettet så alt blir riktig og bra" (brukeren) - special:"quiz" (se startVtolSpecialExercise), samme
    // ingen-3D-flyging-overlegg som ex0. Innholdet er faktasjekket mot ardupilot.org/plane sin egen
    // dokumentasjon (kalibrering/toilet-bowling, QuadPlane-modusene, Q_ASSIST_SPEED, pre-arm-sjekker) og
    // mot Heewing-spesifikasjonene/ArduPilot-arming-sitatet brukeren selv oppga tidligere i denne økten
    // (motorlayout, pinne-arm/disarm-gesten, Motor Emergency Stop vs. disarm) - IKKE gjettet fritt.
    ex7: {
        id: "ex7", icon: "fa-graduation-cap", label: "8. Teoriprøve",
        special: "quiz",
        shortDescription: "Multiple choice om Heewing T2 Cruza og ArduPilot - moduser, kalibrering og sikkerhet.",
        fullDescription: "18 spørsmål om det viktigste å kunne før du flyr Heewing T2 Cruza med ArduPilot: " +
            "flymodusene, hvorfor kalibrering betyr noe, og forskjellen på nødstopp og disarm. Minst 15 av " +
            "18 riktige for å bestå - du kan ta den om igjen så mange ganger du vil.",
        quizPassFraction: 0.8,
        // Selve spørsmålene/svaralternativene ligger i js/simulator-vtol-quiz.js (egen fil, se
        // toppkommentaren der - "enklere redigering") - lastet FØR denne filen, se simulator-vtol.html.
        quizQuestions: VTOL_QUIZ_QUESTIONS
    }
};
const VTOL_EXERCISE_ORDER = ["ex0", "ex1", "ex2", "ex3", "ex4", "ex5", "ex6", "ex6b", "ex7"];

// "Alle øvelsene kan kreve disarm etter landing. Det tenker jeg er en grei vane å få inn" (brukeren) - lagt
// til ETT sted (i stedet for håndskrevet i alle seks stage-listene, med risiko for at én blir glemt/ulik) -
// et siste steg (type "await-disarm", se updateAwaitDisarmStage) som venter på selve disarm-gesten (gass i
// bunn + fullt sideror VENSTRE) FØR øvelsen faktisk telles som fullført. Gjelder alle de vanlige,
// fly-baserte øvelsene (ex1-ex6, ex6b) - IKKE ex0/ex7 (special:"wizard"/"quiz", ingen 3D-flyging/planeState
// i det hele tatt der). Egen objekt-instans per øvelse (ikke én delt referanse) - ingen kjent kode muterer et
// stage-objekt i farten i dag, men å dele én referanse ville vært en unødvendig, skjør avhengighet å stole
// på i fremtiden.
VTOL_EXERCISE_ORDER.forEach(function (id) {
    const exercise = VTOL_EXERCISES[id];
    if (!exercise.special) exercise.stages.push({ type: "await-disarm", label: "Disarm motorene" });
});

const VTOL_DEFAULT_EXERCISE_PROGRESS = {};
VTOL_EXERCISE_ORDER.forEach(function (id) { VTOL_DEFAULT_EXERCISE_PROGRESS[id] = { passed: false }; });
function loadVtolExerciseProgress() { return Sim.loadJSON(VTOL_EXERCISE_STORAGE_KEY, VTOL_DEFAULT_EXERCISE_PROGRESS); }
function saveVtolExerciseProgress() { Sim.saveJSON(VTOL_EXERCISE_STORAGE_KEY, vtolExerciseProgress); }
const vtolExerciseProgress = loadVtolExerciseProgress();
function allVtolExercisesPassed() {
    return VTOL_EXERCISE_ORDER.every(function (id) { return vtolExerciseProgress[id] && vtolExerciseProgress[id].passed; });
}
// "legg til mulighet for å resette hele øvelsesprogresjonen. i tilfelle noen andre skal gjennom
// programmet" (brukeren) - se knappen (exerciseResetProgressBtn, simulator-vtol.html) i selve
// øvelseslisten, wired i initVtolExercisePanel under. vtolExerciseProgress er en `const` (delt referanse
// alle andre steder i filen holder på) - MUTERER derfor hver egenskap i stedet for å forsøke å bytte ut
// hele objektet, som ville etterlatt de andre stedene pekende på det gamle, nå utdaterte objektet.
function resetVtolExerciseProgress() {
    VTOL_EXERCISE_ORDER.forEach(function (id) { vtolExerciseProgress[id] = { passed: false }; });
    saveVtolExerciseProgress();
}

/* ==================== Tilstand ==================== */
const exerciseState = {
    active: false,
    exerciseId: null,
    stageIndex: 0,
    startTime: 0,
    wasAirborne: false,
    awaitingNext: false,
    warningMessage: "",
    warningUntil: 0,
    warningIsSuccess: false,
    wpIndex: 0,
    hoverHoldSec: 0,
    groundedHoldSec: 0, // se LANDING_SETTLE_SEC - hvor lenge flyet har stått urørt på bakken i inneværende landingssteg
    pitchWarnUntil: 0,
    modeStrayWarnUntil: 0, // se stage.requireModeThroughout i updateWaypointsStage - advarsel-throttle
    armHintUntil: 0, // se updateAwaitArmStage/updateAwaitDisarmStage - repeterende gest-påminnelse
    scenario: null, // per-scenario arbeidsdata, satt av scenarioets egen setup()
    originalWind: null, // elevens egne Vind-panel-innstillinger, tatt vare på mens et vind-steg låner dem
    previousPlaneClass: null // elevens egen Fly-størrelse, tatt vare på mens øvelsen låner "heewing"
};
let exerciseMarkers = [];
// "Firkant sveveøvelse må ha en indikator å følge firkanten. som på quad simmen" (brukeren) - de statiske
// kulene under (addWaypointMarkers) viser kun FASONGEN, ikke hvor i runden eleven faktisk er. Denne
// pulserende, gule kulen følger LIVE etter exerciseState.wpIndex (samme idé som quad-simulatorens egen
// exerciseGuideHandle.nextWaypointMarker, se buildLoopStruts-området i js/simulator.js) - egen variabel,
// IKKE en del av exerciseMarkers (ville forskjøvet markWaypointReached sine indekser).
let nextWaypointMarker = null;
let exerciseAutopilotFn = null; // satt av scenarioer/AUTO-oppdraget som midlertidig skal styre pinnen selv

// Ett rett "stag" (sylinder) mellom to punkt - samme setFromUnitVectors-teknikk som buildLoopStruts i
// js/simulator.js (se addWaypointMarkers over). Lagt til (og fjernet igjen) via exerciseMarkers/
// clearExerciseMarkers, samme livssyklus som de andre banemarkørene.
function buildWaypointStrut(a, b, radius, material) {
    const dir = new THREE.Vector3().subVectors(b, a);
    const len = dir.length();
    const strut = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, Math.max(len, 0.01), 8), material);
    strut.position.copy(a).addScaledVector(dir, 0.5);
    if (len > 1e-6) strut.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize());
    scene.add(strut);
    return strut;
}

function clearExerciseMarkers() {
    exerciseMarkers.forEach(function (m) { scene.remove(m); });
    exerciseMarkers = [];
    if (nextWaypointMarker) { scene.remove(nextWaypointMarker); nextWaypointMarker = null; }
}
// closeLoop (default true) - ex2 sin firkant er en EKTE lukket løkke (siste punkt -> første igjen), mens
// ex3 sin landingsrunde (utflyging-kryssvind-medvind-base-finale) er en ÅPEN rute som ender nær
// rullebanen, ikke tilbake ved sitt eget startpunkt - en lukket forbindelseslinje der ville tegnet en
// vilkårlig ekstra strek tvers over runden.
// markerRadius (default 0.45) - "kuleindikatorene trenger ikke være så store når de er så nærme"
// (brukeren, ex1/ex2): radius senket fra 2 til 1, til 0.6, og igjen ("litt mindre kuler siden vi er så
// nært her") til 0.45 m der banen er trang og nære spawn-punktet (se VTOL_HOVER_SQUARE_WAYPOINTS).
// "kulene må være mye større for her er det mer avstand" (brukeren, ex3-landingsrunden) - MOTSATT problem
// på en fastvinget rute som spenner hundretalls meter: samme 0.45 m ville vært usynlig på avstand. Derfor
// nå en per-stage-justerbar radius (stage.markerRadius, se enterStageVisuals-kallet) i stedet for én fast
// verdi for alle øvelser.
function addWaypointMarkers(waypoints, closeLoop, markerRadius) {
    clearExerciseMarkers();
    const r = markerRadius || 0.45;
    // Første punkt fikk tidligere en egen oransje farge (uansett hvor eleven faktisk var i runden) -
    // droppet til fordel for ÉN ensartet blåfarge for alle utestående hjørner, siden nextWaypointMarker
    // (se rett under) nå er den ENESTE, tydelige "hit skal du"-indikatoren.
    // "siste kule kan... ikke være så stor" (brukeren, ex3-finalepunktet) - wp.markerRadius (valgfritt, satt
    // direkte på ENKELTPUNKTER, se VTOL_PATTERN_WAYPOINTS sitt finale-punkt) overstyrer stagets felles
    // radius (r) for akkurat DET punktet, i stedet for å kreve en helt egen markerRadius-parameter per
    // waypoints-stage for én enkelt kule sin skyld.
    waypoints.forEach(function (wp) {
        const mesh = new THREE.Mesh(
            new THREE.SphereGeometry(wp.markerRadius || r, 12, 12),
            new THREE.MeshBasicMaterial({ color: 0x2a7fd6, transparent: true, opacity: 0.55 })
        );
        mesh.position.copy(wp);
        scene.add(mesh);
        exerciseMarkers.push(mesh);
    });
    // "gjør det mer som i quad simmen" (brukeren, ex2-firkanten) - quad-simulatoren (buildLoopStruts i
    // js/simulator.js) tegner selve BANEN som synlige, solide "stag" (sylindre) mellom punktene i stedet
    // for en tynn 1px linje. "det holder med den blå streken langs bakken og de kulene i lufta. trenger
    // ikke den streken i lufta" (brukeren) - droppet flyhøyde-laget igjen (hadde det først i tillegg til
    // bakkeprojeksjonen) - kun bakke-projeksjonen (en tydelig "skygge av ruta") pluss selve kule-
    // markørene i flyhøyde nå. linePoints/closeLoop-logikken er UENDRET fra den opprinnelige Line-
    // varianten (ex3 sin åpne landingsrunde skal fortsatt IKKE lukkes med en vilkårlig strek tvers over
    // runden, se closeLoop-kommentaren ved funksjonskallet).
    const linePoints = closeLoop === false ? waypoints : waypoints.concat([waypoints[0]]);
    const groundMat = new THREE.MeshBasicMaterial({ color: 0x2a7fd6, transparent: true, opacity: 0.65 });
    for (let i = 0; i < linePoints.length - 1; i++) {
        const a = linePoints[i], b = linePoints[i + 1];
        exerciseMarkers.push(buildWaypointStrut(
            new THREE.Vector3(a.x, 0.08, a.z), new THREE.Vector3(b.x, 0.08, b.z), 0.12, groundMat
        ));
    }
    // "det må jo være en indikasjon på neste kule som skal flys til" (brukeren) - denne fantes allerede
    // (se kommentaren ved nextWaypointMarker-deklarasjonen), men var en LITEN, heltrukket kule plassert
    // NØYAKTIG oppå/inni den statiske hjørne-kulen på samme punkt - praktisk talt usynlig. Første forsøk
    // (et større WIREFRAME-"bur" rundt den blå hjørnekulen) så bare rart ut ("rar gul ball laget av
    // streker rundt seg... må vel være en hel kule? uten den blå inni?", brukeren) - en gjennomsiktig
    // strekkule rundt en annen, ulikefarget kule leser ikke som "målet", bare som visuell støy. Løst
    // riktig nå ved heller å SKJULE selve hjørne-kulen mens den er gjeldende mål (se skjul-/vis-logikken i
    // updateWaypointsStage/markWaypointReached under), slik at kun ÉN heltrukket, pulserende gul kule vises
    // på det punktet - ingen blå kule igjen inni/bak den å blande seg med.
    nextWaypointMarker = new THREE.Mesh(
        new THREE.SphereGeometry(r * 1.33, 12, 12),
        new THREE.MeshBasicMaterial({ color: 0xffee55, transparent: true, opacity: 0.85 })
    );
    scene.add(nextWaypointMarker);
}
function markWaypointReached(index) {
    // visible=true angrer skjul-logikken i updateWaypointsStage under (mens et hjørne var GJELDENDE mål) -
    // hjørne-kulen skal vises igjen (nå grønn) idet det faktisk er nådd.
    if (exerciseMarkers[index]) { exerciseMarkers[index].visible = true; exerciseMarkers[index].material.color.setHex(0x2ecc71); }
}

// Hover-øvelsene (ex1) hadde ingen visuell indikator i det hele tatt - kun selve tallene i HUD-en. En gul
// "beacon" på selve holdepunktet (målhøyde) pluss en flat ring på bakken (VTOL_HOVER_RADIUS_M) gir eleven
// noe konkret å sikte inn mot, samme idé som firkantens punktmarkører over.
function addHoverMarker(stage) {
    clearExerciseMarkers();
    const home = rtlState.home;
    // "kuleindikatorene trenger ikke være så store når de er så nærme. gjelder øvelse 1 også" (brukeren) -
    // radius senket fra 1.2 til 0.8 m, samme begrunnelse som addWaypointMarkers over.
    const beacon = new THREE.Mesh(
        new THREE.SphereGeometry(0.8, 12, 12),
        new THREE.MeshBasicMaterial({ color: 0xffcc33, transparent: true, opacity: 0.6 })
    );
    beacon.position.set(home.x, stage.targetAlt, home.z);
    scene.add(beacon);
    exerciseMarkers.push(beacon);

    const ring = new THREE.Mesh(
        new THREE.RingGeometry(Math.max(0.3, VTOL_HOVER_RADIUS_M - 0.4), VTOL_HOVER_RADIUS_M, 32),
        new THREE.MeshBasicMaterial({ color: 0xffcc33, transparent: true, opacity: 0.35, side: THREE.DoubleSide })
    );
    ring.rotation.x = -Math.PI / 2;
    // "indikatorene glitcher noen ganger gjennom rullebanen" (brukeren) - z-fighting: rullebane-meshen
    // (buildRunway i simulator-vtol.js) ligger på y=0.04, ringen lå kun 0.02 m over den - for lite
    // klaring til å unngå periodisk dybdebuffer-flimring på avstand/ved streifende vinkler, spesielt for
    // en så stor (VTOL_HOVER_RADIUS_M=8 m radius) flate. Hevet til god klaring over BÅDE rullebanen (0.04)
    // og bakkens eget rutenett (0.01, se buildGround) - samme prinsipp som forskjellen MELLOM disse to
    // allerede bruker, bare en klarere margin.
    ring.position.set(home.x, 0.15, home.z);
    scene.add(ring);
    exerciseMarkers.push(ring);
}

function setWarning(message, isSuccess, durationMs) {
    exerciseState.warningMessage = message;
    exerciseState.warningIsSuccess = !!isSuccess;
    exerciseState.warningUntil = performance.now() + (durationMs || 3500);
}

function getExercise() { return VTOL_EXERCISES[exerciseState.exerciseId]; }
function getStage() { return getExercise().stages[exerciseState.stageIndex]; }

// Vind-rundene i ex1/ex2 (stage.wind) låner Vind-panelets EGNE innstillinger midlertidig - restoreWind()
// gir dem tilbake når øvelsen avsluttes eller avbrytes, samme prinsipp quad-simulatorens ex9/exHoverWind
// bruker ("Vinden settes tilbake til dine egne innstillinger når øvelsen avsluttes").
function applyStageWind(stage) {
    if (!exerciseState.originalWind) exerciseState.originalWind = Object.assign({}, settings.wind);
    if (stage.wind) {
        settings.wind.enabled = true;
        if (stage.windVariable) {
            settings.wind.speed = 3 + Math.random() * 5;   // 3-8 m/s
            settings.wind.directionDeg = Math.random() * 360;
            settings.wind.gust = 0.4;
        } else {
            settings.wind.speed = stage.wind.speed;
            settings.wind.directionDeg = stage.wind.directionDeg;
            settings.wind.gust = stage.wind.gust;
        }
    } else {
        Object.assign(settings.wind, exerciseState.originalWind);
    }
}
function restoreWind() {
    if (!exerciseState.originalWind) return;
    Object.assign(settings.wind, exerciseState.originalWind);
    saveSettings();
    exerciseState.originalWind = null;
}

/* ==================== Start / stopp / fremdrift ==================== */
function startVtolExercise(id) {
    if (exerciseState.active) stopVtolExercise();
    // Introprogrammet er FOR Heewing T2 Cruza spesifikt - bytt til DEN modellen for varigheten av
    // øvelsen, men husk hva eleven hadde valgt selv (Fly-størrelse i Fly og kamera-panelet) for å
    // gjenopprette det når øvelsen avsluttes (se stopVtolExercise/exerciseSummaryCloseBtn) - IKKE la
    // øvelsesstart trakassere brukerens eget frie-flygings-valg permanent (setPlaneClass lagrer til
    // settings/localStorage).
    if (!exerciseState.previousPlaneClass) exerciseState.previousPlaneClass = planeState.planeClass;
    if (planeState.planeClass !== "heewing") setPlaneClass("heewing");
    // GROUND_SPAWN_YAW_RAD eksplisitt her (IKKE resetPlane() sin egen standard, se dens kommentar) -
    // øvelsene skal som hovedregel fortsatt spawne med halen mot piloten (VLOS-standarden), kun det frie
    // flyging-resettet (resetBtn/R-tasten) bruker standarden (nesa ned rullebanen), se BUG-kommentaren i
    // stepPhysics/simulator-vtol.js.
    // UNNTAK: "på øvelse 4 kan vel flyet starte med nese med rullebaneretningen (dreie nesa 90 grader mot
    // høyre)" (brukeren) - ex4 er IKKE en VLOS-øvelse (se allowFreeCamera over), "hale mot piloten" gir
    // ingen mening der - exercise.spawnYawRad (se VTOL_EXERCISES.ex4) lar akkurat den ene øvelsen starte
    // med nesa ned rullebanen (0 rad) i stedet, samme standard-retning som resetPlane() sin egen (se
    // kommentaren der).
    const exercise = VTOL_EXERCISES[id];
    resetPlane(exercise.spawnYawRad !== undefined ? exercise.spawnYawRad : GROUND_SPAWN_YAW_RAD); // rullebane, motor på, hjempunkt fanget på nytt her
    // "Første øvelse kan starte med motorene av, så får man trent på armering" (brukeren) - resetPlane over
    // armerer alltid (dens egen, vanlige "motor PÅ er arming-øyeblikket"-standard) - disarmPlane() rett
    // etter overstyrer det for de FÅ øvelsene (se VTOL_EXERCISES.ex1.startDisarmed) som eksplisitt skal
    // starte disarmert, med hjempunktet ugyldig helt til eleven selv fullfører arme-gesten (se den nye
    // "await-arm"-stagen/updateAwaitArmStage).
    if (exercise.startDisarmed) disarmPlane();
    exerciseState.active = true;
    exerciseState.exerciseId = id;
    exerciseState.stageIndex = 0;
    exerciseState.startTime = performance.now();
    exerciseState.wasAirborne = false;
    exerciseState.awaitingNext = false;
    exerciseState.hoverHoldSec = 0;
    exerciseState.groundedHoldSec = 0;
    exerciseState.wpIndex = 0;
    exerciseState.scenario = null;
    exerciseAutopilotFn = null;
    trySetFlightMode("qloiter");
    // VLOS - den realistiske bakkepilot-visningen (fast ved avgangsplassen) - se låsen i toggleCamera()
    // i simulator-vtol.js. Øvingsprogrammet er et VLOS-treningsprogram, ikke en FPV/chase-simulator.
    // UNNTAK: "runde i manuell modus med chase kamera bare for å få følelsen med flyet" (brukeren, ex4) -
    // exercise.allowFreeCamera (se VTOL_EXERCISES.ex4) lar ETT enkelt, bevisst valgt øvelses-formål bryte
    // VLOS-standarden, samme unntaksmønster som startDisarmed over.
    if (exercise.allowFreeCamera) {
        cameraModeIndex = CAMERA_MODES.indexOf("chase");
        activeCamera = chaseCamera;
    } else {
        cameraModeIndex = CAMERA_MODES.indexOf("vlos");
        activeCamera = vlosCamera;
    }
    enterStageVisuals();
    document.getElementById("exercisesPanel").style.display = "none";
    document.getElementById("exerciseHudBar").style.display = "flex";
    document.getElementById("gcsScreen").style.display = "block";
    // "første tooltip må jo være hva man skal gjøre... og ikke svada/dobbel informasjon i tooltips. kort og
    // konsist hva som skal gjøres. trenger ikke tallnummerering" (brukeren, gjelder ALLE øvelsene) - denne
    // meldingen brukte tidligere "Øvelse startet: " + ØVELSENS EGEN navn (f.eks. "3. Landingsrunde") som
    // det aller første eleven ser, ikke selve HANDLINGEN (steg 0 sin egen label) - eksercisnavnet er allerede
    // synlig i selve panelet/HUD-en før øvelsen starter, ikke ny informasjon her. Viser nå direkte det
    // FØRSTE steget sin egen (nå nummererings-/svada-frie, se hver øvelses egen stages-kommentar) label i
    // stedet - samme mønster som "Fullført! Neste: ..." allerede bruker for hvert påfølgende steg.
    setWarning(stageStartMessage(getStage().label), true, getStage().hint ? 6000 : 3000);
}
// "øvelse 4. må ha noe mer indikatorer eller meldinger til brukeren hva/hvor hen skal fly?" (brukeren) -
// stage.hint (valgfritt, se VTOL_EXERCISES.ex4 sine "waypoints"-steg) gir et konkret, i ord beskrevet mål
// ("fly mot X") i TILLEGG til selve steg-labelen, vist sammen med "Fullført! Neste: ..."/"Øvelse startet:
// ..."-meldingen - IKKE en egen setWarning()-kall, som ville overskrevet nettopp den meldingen (samme
// kall-rekkefølge-problem som armHintUntil-mønsteret andre steder i denne filen er bygget rundt).
function stageStartMessage(prefix) {
    const stage = getStage();
    return stage.hint ? prefix + " - " + stage.hint : prefix;
}

function enterStageVisuals() {
    clearExerciseMarkers();
    const stage = getStage();
    if (stage.type === "waypoints") addWaypointMarkers(stage.waypoints, stage.closeLoop, stage.markerRadius);
    else if (stage.type === "hover") addHoverMarker(stage);
    // "øvelse 6. må være satt opp som at det ser ut som en hover øvelse som øvelse 1 med indikator. så
    // brukeren har et utgangspunkt for hva som skal gjøres" (brukeren) - ex6 sine to "scenario"-steg
    // (VTOL_SCENARIOS.toiletbowlLand/flyawayCrowd) er begge reelt sett en hover-i-QLOITER-oppgave inntil
    // feilen inntreffer, men hadde INGEN visuell indikator i det hele tatt - eleven måtte gjette hvor/hvor
    // høyt uten noe å sikte inn mot. addHoverMarker (samme gule beacon+ring som ex1) gjenbrukes direkte -
    // stage.showHoverMarker (satt kun på ex6 sine to steg, se VTOL_EXERCISES.ex6) skiller dette fra ex6b sitt
    // scenario (engineFailureGlide, en luftbåren glidefeil, ikke en hover-oppgave - skal IKKE ha markøren).
    else if (stage.type === "scenario" && stage.showHoverMarker) addHoverMarker(stage);
    // "3 runder med forskjellig vind" (ex5, se VTOL_EXERCISES.ex5) - depart-distance/return-manual må også
    // kunne bære et stage.wind, ikke bare hover/waypoints (ellers ville vinden nullstilt seg selv midt i
    // hver runde, i overgangen fra utflygingen til returen).
    // "to runder på øvelse [motorfeil]. andre runde med litt vind" (brukeren) - "scenario" lagt til her
    // (var ikke med fra før - ex6 sine to scenarioer bruker ikke vind) slik at ex6b sin runde 2 kan bære
    // et stage.wind på samme måte som resten av programmet allerede gjør.
    // "ingen vind før man skal returnere hjem for å lande. landing med vind" (brukeren, ex4) -
    // "transition-back" lagt til her slik at ex4 kan holde selve rundflygingen (portene/garasjene/
    // landemerkene, alle type "waypoints") vindstille og heller legge vinden på FØRST når returen hjem
    // starter (se VTOL_EXERCISES.ex4 sitt transition-back-steg) - i stedet for at "waypoints" alltid var
    // stedet vinden skrudde på, som ga vind gjennom hele rundflygingen og (siden "transition-back"/
    // "land-manual" aldri rørte den) ufortjent ULIK vind videre inn i selve landingen.
    if (stage.type === "hover" || stage.type === "waypoints" || stage.type === "depart-distance" || stage.type === "return-manual" || stage.type === "scenario" || stage.type === "transition-back") applyStageWind(stage);
    exerciseState.hoverHoldSec = 0;
    exerciseState.wpIndex = 0;
    // wasAirborne MÅ nullstilles her (ikke bare i startVtolExercise) - ellers vil R midt i et
    // "land-manual"-steg la et fly som nettopp ble satt tilbake på bakken (rett ved hjempunktet, se
    // resetPlane) telle som "landet" øyeblikkelig, siden flagget fortsatt sto igjen som true fra FØR R
    // ble trykket. groundedHoldSec (VTOL_LANDING_SETTLE_SEC) nullstilles av samme grunn.
    exerciseState.wasAirborne = false;
    exerciseState.groundedHoldSec = 0;
    exerciseState.armHintUntil = 0; // se updateAwaitArmStage/updateAwaitDisarmStage - fersk stage, fersk påminnelse
    // Samme sikkerhetsnett som stopVtolExercise - et R-trykk MIDT i toiletbowl-scenarioet skal ikke la den
    // aktive kompass-feilen henge igjen inn i det (nylig resatte) flyets videre fysikk.
    qloiterHeadingErrorRad = 0;
}

function stopVtolExercise() {
    exerciseState.active = false;
    exerciseState.exerciseId = null;
    exerciseState.awaitingNext = false;
    exerciseAutopilotFn = null;
    // Sikkerhetsnett - se toiletbowl-scenarioets egen kommentar: garanterer at en feilkalibrert-kompass-
    // feil aldri kan overleve inn i fri flyging eller en annen øvelse, selv om update() skulle slutte å
    // kjøres midt i scenarioet (f.eks. R-reset eller at eleven forlater øvelsen brått).
    qloiterHeadingErrorRad = 0;
    clearExerciseMarkers();
    removeTrafficPlaneVisual();
    restoreWind();
    document.getElementById("exerciseHudBar").style.display = "none";
    document.getElementById("exerciseWarningBanner").classList.remove("show");
    closeGcsScreen();
    document.getElementById("gcsScreen").style.display = "none";
    restorePreviousPlaneClass();
}

function restorePreviousPlaneClass() {
    if (exerciseState.previousPlaneClass && exerciseState.previousPlaneClass !== planeState.planeClass) {
        setPlaneClass(exerciseState.previousPlaneClass);
    }
    exerciseState.previousPlaneClass = null;
}

function advanceStage() {
    const exercise = getExercise();
    exerciseState.stageIndex++;
    if (exerciseState.stageIndex >= exercise.stages.length) {
        completeExercise();
        return;
    }
    enterStageVisuals();
    setWarning(stageStartMessage("Fullført! Neste: " + getStage().label), true, getStage().hint ? 6000 : 3500);
}

function completeExercise() {
    const id = exerciseState.exerciseId;
    vtolExerciseProgress[id] = { passed: true };
    saveVtolExerciseProgress();
    clearExerciseMarkers();
    exerciseAutopilotFn = null;
    restoreWind();
    exerciseState.awaitingNext = true;
    document.getElementById("exerciseHudBar").style.display = "none";
    closeGcsScreen();
    document.getElementById("gcsScreen").style.display = "none";
    showVtolExerciseSummary(id);
}

function openGcsScreen() { document.getElementById("gcsScreen").classList.add("open"); }
function closeGcsScreen() { document.getElementById("gcsScreen").classList.remove("open"); }
function failStage(reason) {
    setWarning("Ikke bestått: " + reason + ".", false, 6000);
    exerciseState.hoverHoldSec = 0;
}

/* ==================== Steg-logikk (kalt hver frame fra updateVtolExercise) ==================== */
function stageModeOk(stage) {
    if (stage.requireMode) return planeState.flightMode === stage.requireMode;
    if (stage.requireQMode) return isQMode(planeState.flightMode);
    if (stage.requireFixedWing) return isFixedWingMode(planeState.flightMode);
    return true;
}
// "Gi noen gule flash i HUDEN på MODUS knappen Frem til modus er byttet. så ser brukeren at det kan
// trykkes der også" (brukeren) - se .exercise-mode-hint (css/style.css) og #hudModeItem-toggling i
// updateExerciseHud. Dekker alle stegtypene som venter på ETT spesifikt modusbytte fra eleven -
// "return-manual" er bevisst UTELATT (det venter på at eleven UNNGÅR qrtl, ikke bytter TIL en bestemt
// modus - et blink der ville gitt feil signal om hvilken modus som faktisk skal velges).
function stageNeedsModeSwitch(stage) {
    if (stage.type === "await-mode") return planeState.flightMode !== stage.mode;
    if (stage.type === "hold-mode-until-landed") return planeState.flightMode !== stage.mode;
    if (stage.type === "transition-out") return !isFixedWingMode(planeState.flightMode);
    if (stage.type === "transition-back") return !isQMode(planeState.flightMode);
    if (stage.type === "hover" || stage.type === "waypoints") return !stageModeOk(stage);
    if (stage.type === "land-manual" && (stage.requireMode || stage.requireQMode)) return planeState.onGround && !stageModeOk(stage);
    return false;
}
function stageModeWarning(stage) {
    // "'Vær i QHOVER for denne runden.' skal heller være 'Bytt til QHOVER'" (brukeren) - en mer direkte,
    // handlingsrettet formulering (samme endring for alle moduser, ikke bare QHOVER spesifikt).
    // "tooltip om bytte til qstabilize. få med hvilken tast på tastaturet det er" (brukeren) - samme
    // MODE_KEY_LABELS-oppslag som ex4 sitt eget MANUAL-bytte-hint (se await-mode-statusText i
    // updateExerciseHud) allerede bruker, her generelt for ALLE stage.requireMode-varsler (ikke bare
    // QSTABILIZE spesifikt) - gjelder dermed automatisk enhver hover-/waypoints-/land-manual-runde som
    // krever en bestemt modus, ikke bare det ene stedet brukeren nevnte.
    if (stage.requireMode) {
        return "Bytt til " + (MODE_LABELS[stage.requireMode] || stage.requireMode) +
            (MODE_KEY_LABELS[stage.requireMode] ? " (tast " + MODE_KEY_LABELS[stage.requireMode] + " på tastaturet)" : "") + ".";
    }
    if (stage.requireQMode) return "Vær i en Q-modus for at dette skal telle.";
    if (stage.requireFixedWing) return "Vær i en fastvinget modus for at dette skal telle.";
    return "";
}

function updateHoverStage(stage, dt) {
    const alt = currentAltitude();
    const dist = horizDistFromHome();
    const inBox = stageModeOk(stage) && dist < VTOL_HOVER_RADIUS_M &&
        Math.abs(alt - stage.targetAlt) < 6 && !planeState.crashed;
    if (!stageModeOk(stage) && !planeState.onGround && exerciseState.hoverHoldSec === 0) {
        setWarning(stageModeWarning(stage), false, 2500);
    }
    if (planeState.onGround) exerciseState.hoverHoldSec = 0;
    else if (inBox) exerciseState.hoverHoldSec += dt;
    else if (exerciseState.hoverHoldSec > 0) {
        exerciseState.hoverHoldSec = 0;
        setWarning("Driftet ut av hover-sonen - holdetiden er nullstilt.", false, 2500);
    }
    const pitchBank = currentPitchBankDeg();
    // "Kan kutte de meldingene. kanskje heller tipse om å sette nesa inn mot vinden?" (brukeren) - den
    // gamle meldingen ("stor pitch-vinkel - vurder å avbryte og lande") lød mer alarmerende enn selve
    // situasjonen tilsier (stor pitch i hover er normal, forventet flyteknikk i vind, ikke en
    // krise/feil-tilstand) - erstattet med et praktisk teknikk-tips i stedet for en avbryt-anbefaling.
    if (Math.abs(pitchBank.pitchDeg) > VTOL_HOVER_PITCH_WARN_DEG && performance.now() > exerciseState.pitchWarnUntil) {
        setWarning("Stor pitch-vinkel i hover (" + Math.abs(pitchBank.pitchDeg).toFixed(0) + "°) - prøv å snu nesa mer rett inn mot vinden, det reduserer hvor mye pitch som trengs for å holde posisjonen.", false, 4500);
        exerciseState.pitchWarnUntil = performance.now() + 6000;
    }
    if (exerciseState.hoverHoldSec >= stage.holdSec) advanceStage();
}

// stage.requireMode/requireQMode (begge valgfrie, se stageModeOk) gjenbruker SAMME mønster som
// hover-/waypoints-stegene allerede bruker for et land-manual-steg som krever en bestemt modus å lande i.
// "Landing i QHOVER må ikke godkjenne QLOITER på øvelse 4" (brukeren, TIDLIGERE) -> "Øvelse 4 landingen MÅ
// ikke være i QHOVER. kan være fri Q-modus" (brukeren, REVERSERT) - ex4 sitt siste steg satte først
// requireMode:"qhover" (kun QHOVER godkjent), nå requireQMode:true (enhver Q-modus godkjent) i stedet - se
// VTOL_EXERCISES.ex4. De andre øvelsenes land-manual-steg setter INGEN av delene - uendret oppførsel
// (godtar landing i enhver modus) for dem.
function updateLandManualStage(stage, dt) {
    if (currentAltitude() > 2) exerciseState.wasAirborne = true;
    if (exerciseState.wasAirborne && planeState.onGround && !planeState.crashed) {
        if ((stage.requireMode || stage.requireQMode) && !stageModeOk(stage)) {
            exerciseState.groundedHoldSec = 0;
            setWarning(stageModeWarning(stage), false, 2500);
            return;
        }
        if (horizDistFromHome() < VTOL_LANDING_RADIUS_M) {
            // Krev noen sekunders uavbrutt, stillestående bakkekontakt før steget telles som bestått - se
            // VTOL_LANDING_SETTLE_SEC-merknaden. groundedHoldSec telles KUN opp mens vilkårene over
            // (luftbåren tidligere + fortsatt på bakken + ikke krasjet + innenfor landingsradiusen) holder.
            exerciseState.groundedHoldSec += dt;
            if (exerciseState.groundedHoldSec >= VTOL_LANDING_SETTLE_SEC) advanceStage();
        } else {
            exerciseState.groundedHoldSec = 0;
            setWarning("Landet for langt fra hjempunktet - ta av igjen og prøv på nytt.", false, 3500);
        }
    } else {
        exerciseState.groundedHoldSec = 0;
    }
}

// Ny øvelse 5 - "manuell retur hjem i vind" (brukeren: "øvelse 5 er en vits. fjern de scenarione. denne kan
// heller være manuell returnering hjem i vind... uten bruk av RTL"). Samme landings-/oppsett-logikk som
// updateLandManualStage, MED ett ekstra krav: QRTL skal IKKE brukes i det hele tatt - hele poenget med
// øvelsen er å øve på å FINNE VEIEN HJEM OG LANDE SELV i vind, ikke å la autopiloten gjøre det. Bytter
// eleven til QRTL når som helst i steget, telles det som ikke bestått (samme "ikke lov å jukse seg unna
// selve poenget"-prinsipp som killswitch-scenarioene i js/simulator.js sin ex11 bruker for crowd/traffic).
function updateReturnManualStage(dt) {
    if (planeState.flightMode === "qrtl") {
        failStage("QRTL ble brukt - denne runden krever manuell retur");
        trySetFlightMode("qloiter"); // tving tilbake til manuell styring - la eleven fullføre selv
        return;
    }
    if (currentAltitude() > 2) exerciseState.wasAirborne = true;
    if (exerciseState.wasAirborne && planeState.onGround && !planeState.crashed) {
        if (horizDistFromHome() < VTOL_LANDING_RADIUS_M) {
            exerciseState.groundedHoldSec += dt;
            if (exerciseState.groundedHoldSec >= VTOL_LANDING_SETTLE_SEC) advanceStage();
        } else {
            exerciseState.groundedHoldSec = 0;
            setWarning("Landet for langt fra hjempunktet - ta av igjen og prøv på nytt.", false, 3500);
        }
    } else {
        exerciseState.groundedHoldSec = 0;
    }
}

function updateWaypointsStage(stage, dt) {
    const target = stage.waypoints[exerciseState.wpIndex];
    if (!target) { advanceStage(); return; }
    // stage.requireModeThroughout (se VTOL_EXERCISES.ex4 sin egen kommentar for bakgrunnen) - krever
    // UNAVBRUTT modus gjennom HELE steget, ikke bare i det ene øyeblikket flyet er innenfor
    // fangstradiusen til hvert punkt (ellers kunne eleven bytte til en enklere modus MELLOM punktene og
    // bare hoppe tilbake til den krevde modusen rett før hver fangst - "sjekk at det ikke er lov å jukse
    // med å fjerne manuell på første delen", brukerens egen bekymring). Et avvik nullstiller fremgangen i
    // steget og tegner markørene på nytt (alle uunnagjorte hjørner igjen) - har eleven ikke kommet forbi
    // første punkt ennå (wpIndex 0), er det ingenting å nullstille, kun selve advarselen.
    if (stage.requireModeThroughout && !planeState.onGround && planeState.flightMode !== stage.requireMode) {
        if (performance.now() > exerciseState.modeStrayWarnUntil) {
            if (exerciseState.wpIndex > 0) {
                setWarning("Byttet ut av " + (MODE_LABELS[stage.requireMode] || stage.requireMode) +
                    " midt i sekvensen - denne delen må flys på nytt fra start.", false, 3500);
                exerciseState.wpIndex = 0;
                addWaypointMarkers(stage.waypoints, stage.closeLoop, stage.markerRadius);
            } else {
                setWarning(stageModeWarning(stage), false, 2500);
            }
            exerciseState.modeStrayWarnUntil = performance.now() + 3500;
        }
        return;
    }
    // "en indikator å følge firkanten. som på quad simmen" (brukeren) - se nextWaypointMarker-kommentaren
    // ved deklarasjonen. Oppdatert hver tick uansett resten av steget under, slik at markøren følger
    // wpIndex live selv om eleven aldri kommer innenfor radius.
    // "uten den blå inni?" (brukeren) - skjuler selve hjørne-kulen for GJELDENDE mål mens den pulserende
    // gule kulen (nextWaypointMarker) står på nøyaktig samme punkt, slik at bare ÉN kule er synlig der -
    // markWaypointReached gjør den synlig igjen (grønn) idet eleven faktisk når den.
    if (exerciseMarkers[exerciseState.wpIndex]) exerciseMarkers[exerciseState.wpIndex].visible = false;
    if (nextWaypointMarker) {
        nextWaypointMarker.position.copy(target);
        nextWaypointMarker.scale.setScalar(0.85 + Math.sin(performance.now() / 200) * 0.15);
    }
    const dist = planeState.position.distanceTo(target);
    if (dist > stage.radius) return;
    if (!stageModeOk(stage)) {
        setWarning(stageModeWarning(stage), false, 2500);
        return;
    }
    if (stage.warnLowIas && lastAirspeed < VTOL_MIN_SAFE_IAS && lastAirspeed > 0.5) {
        setWarning("Lav luftfart (" + lastAirspeed.toFixed(1) + " m/s) - hold god margin til stallfart.", false, 2500);
    }
    markWaypointReached(exerciseState.wpIndex);
    exerciseState.wpIndex++;
    if (exerciseState.wpIndex >= stage.waypoints.length) advanceStage();
}

function updateClimbStage(stage) {
    if (currentAltitude() >= stage.minAlt) advanceStage();
}

function updateTransitionOutStage() {
    // AIRSPEED_MIN (js/simulator-vtol.js) - "Kritisk hastighet før overgangen til Fixed-Wing anses som
    // fullført" (brukeren) - egen, FAST terskel her, ikke lenger den pilot-justerbare Q_ASSIST_SPEED-
    // sliderverdien (vtolParams.assistSpeed), se AIRSPEED_MIN_TRANSITION-kommentaren ved deklarasjonen.
    if (isFixedWingMode(planeState.flightMode)) {
        if (planeState.lastMcAuthority < 0.15 || lastAirspeed > AIRSPEED_MIN_TRANSITION) advanceStage();
    }
}
function updateTransitionBackStage() {
    if (isQMode(planeState.flightMode) && planeState.lastMcAuthority > 0.85) advanceStage();
}

function updateDepartDistanceStage(stage) {
    if (isFixedWingMode(planeState.flightMode) && horizDistFromHome() >= stage.minDist) advanceStage();
}
function updateAwaitModeStage(stage) {
    if (planeState.flightMode === stage.mode) advanceStage();
}
// "Første øvelse kan starte med motorene av, så får man trent på armering" (brukeren) - se
// VTOL_EXERCISES.ex1.startDisarmed/startVtolExercise. planeState.armed settes av selve pinne-gesten
// (updateStickArming/armPlane i simulator-vtol.js, kjører UBETINGET hver frame - se dens egen kommentar),
// denne stagen venter bare på og OPPDAGER overgangen, akkurat som updateAwaitModeStage over.
function updateAwaitArmStage() {
    if (planeState.armed) { advanceStage(); return; }
    if (performance.now() > exerciseState.armHintUntil) {
        setWarning("Arm motorene: gass i bunn + fullt sideror til HØYRE i noen sekunder.", true, 4000);
        exerciseState.armHintUntil = performance.now() + 6000;
    }
}
// "Alle øvelsene kan kreve disarm etter landing. Det tenker jeg er en grei vane å få inn" (brukeren) - se
// VTOL_AWAIT_DISARM_STAGE, lagt til som SISTE steg i samtlige vanlige (ikke-special) øvelser under.
function updateAwaitDisarmStage() {
    if (!planeState.armed) { advanceStage(); return; }
    if (performance.now() > exerciseState.armHintUntil) {
        setWarning("Disarm motorene: gass i bunn + fullt sideror til VENSTRE i noen sekunder.", true, 4000);
        exerciseState.armHintUntil = performance.now() + 6000;
    }
}
function updateHoldModeUntilLandedStage(stage, dt) {
    if (currentAltitude() > 2) exerciseState.wasAirborne = true;
    if (planeState.flightMode !== stage.mode) {
        failStage("byttet bort fra " + MODE_LABELS[stage.mode] + " før landing");
        // Ikke stopp øvelsen - la eleven prøve igjen fra samme steg (trykk 8 på nytt).
        exerciseState.groundedHoldSec = 0;
        return;
    }
    // wasAirborne-kravet hindrer at et R-trykk midt i steget (som setter flyet rett tilbake på bakken
    // ved hjempunktet, se resetPlane) lar en umiddelbar re-aktivering av QRTL fra bakken telle som en
    // fullført retur - QRTL skal faktisk ha FLØYET hjem, ikke bare blitt slått på i ro på rullebanen.
    // Krever i tillegg noen sekunders uavbrutt bakkekontakt (VTOL_LANDING_SETTLE_SEC) før steget telles
    // som bestått - se merknaden ved konstanten.
    if (exerciseState.wasAirborne && planeState.onGround && !planeState.crashed) {
        exerciseState.groundedHoldSec += dt;
        if (exerciseState.groundedHoldSec >= VTOL_LANDING_SETTLE_SEC) advanceStage();
    } else {
        exerciseState.groundedHoldSec = 0;
    }
}

/* ==================== Scenarioer (ex6) ==================== */
function teleportAirborne(x, alt, z, yawDeg, mode, speed) {
    resetPlane();
    planeState.position.set(x, alt, z);
    const yawRad = THREE.MathUtils.degToRad(yawDeg);
    planeState.quaternion.setFromEuler(new THREE.Euler(0, yawRad, 0, "YXZ"));
    const fwd = new THREE.Vector3(0, 0, -1).applyQuaternion(planeState.quaternion);
    planeState.velocity.copy(fwd).multiplyScalar(speed || 0);
    planeState.onGround = false;
    planeState.hasBeenAirborne = true;
    planeState.angularVelocity.pitch = 0;
    planeState.angularVelocity.roll = 0;
    planeState.angularVelocity.yaw = 0;
    trySetFlightMode(mode);
    rtlState.phase = "idle";
    rtlState.failsafeTriggered = false;
}

/* ---------- Publikum/pilot (ex6 flyawayCrowd) ----------
   Piloten/tilskuerne (VTOL_PILOT_POSITION/VTOL_CROWD_CENTER m.fl.) og selve kollisjonen
   (checkVtolPersonCollision -> planeState.injured/injuredTarget) er nå PERMANENTE, UBETINGEDE ting i
   js/simulator-vtol.js - se buildVtolCrowd/checkVtolPersonCollision der. Dette scenarioet trenger derfor
   bare å LESE planeState.injured (under, i update()), ikke lenger bygge/fjerne noe selv. */
// Samme "spøkelses-pinne"-teknikk som den tidligere kill-scenarioets killScenarioAutopilot (styringstap mot
// et fast mål), her mot VTOL_CROWD_CENTER (simulator-vtol.js) i stedet for TOWN_CENTER.
function flyawayCrowdAutopilot() {
    const q = planeState.quaternion;
    const fwd = new THREE.Vector3(0, 0, -1).applyQuaternion(q); fwd.y = 0; fwd.normalize();
    const right = new THREE.Vector3(1, 0, 0).applyQuaternion(q); right.y = 0; right.normalize();
    const toX = VTOL_CROWD_CENTER.x - planeState.position.x, toZ = VTOL_CROWD_CENTER.z - planeState.position.z;
    const dist = Math.hypot(toX, toZ) || 1;
    const dirX = toX / dist, dirZ = toZ / dist;
    const headingErrorRad = Math.atan2(right.x * dirX + right.z * dirZ, fwd.x * dirX + fwd.z * dirZ);
    inputState.stick.roll = clamp(headingErrorRad / THREE.MathUtils.degToRad(60), -1, 1);
    inputState.stick.pitch = 0;
    inputState.stick.yaw = 0;
    inputState.stick.throttle = 0.65;
}

const VTOL_SCENARIOS = {
    // Runde 1 (ex6) - "Først ta av hovre og lande. underveis får man toilet bowl. skal helst bytte til
    // QHOVER og lande da." Eleven tar av OG etablerer hover SELV (ikke teleportert luftbåren, se
    // resetPlane() i setup) - feilen (samme fysiske mekanisme som før, se qloiterHeadingErrorRad-
    // forklaringen i stepPhysics) inntreffer først VTOL_SCENARIO_SETTLE_SEC sekunder ETTER at farkosten
    // faktisk er luftbåren, ikke fra selve øvelsesstarten - "ikke start og avslutt øvelsen så brått. må ha
    // noen sekunder på å komme inn i hva som skjer med flyet."
    toiletbowlLand: {
        setup: function () {
            resetPlane(GROUND_SPAWN_YAW_RAD); // se resetPlane-kommentaren i simulator-vtol.js
            setWarning("Ta av og hold en stabil hover i QLOITER.", true, 4000);
            return { settleUntil: null, headingErrorRad: 0, startX: 0, startZ: 0, recognizedAt: null, groundedHoldSec: 0 };
        },
        update: function (scenario, dt) {
            // "hvis man skader seg selv eller publikum er øvelsen feilet og man får ikke godkjent. må
            // restarte da" (brukeren) - flyawayCrowd (under)/engineFailureGlide (ex6b) sjekket allerede
            // planeState.injured (checkVtolPersonCollision kjører UBETINGET hver frame uansett scenario, se
            // toppkommentaren over VTOL_SCENARIOS), men DENNE runden gjorde det ALDRI - en toilet-bowling-
            // drift kan fint drive innom piloten sin egen posisjon, og traff den ville rett og slett bli
            // oversett/ikke telle som feil. Samme sjekk lagt til her.
            if (planeState.injured) {
                scenario.result = "fail";
                scenario.failReason = planeState.injuredTarget === "pilot" ? "traff piloten" : "traff en person i publikum";
                return;
            }
            if (planeState.crashed) { scenario.result = "fail"; scenario.failReason = "krasjet"; return; }
            if (!scenario.settleUntil) {
                if (currentAltitude() > 2) scenario.settleUntil = performance.now() + VTOL_SCENARIO_SETTLE_SEC * 1000;
                return; // ennå ikke luftbåren, eller fortsatt i "kom deg inn i det"-vinduet
            }
            if (performance.now() < scenario.settleUntil) return;
            if (!scenario.headingErrorRad) {
                // Se toiletbowl sin opprinnelige BUG-merknad (fysisk ekte drift, ikke en scriptet
                // posisjons-teleport) - samme mekanisme, kun forsinket til ETTER innkomst-vinduet nå.
                const errDeg = (100 + Math.random() * 40) * (Math.random() < 0.5 ? 1 : -1);
                scenario.headingErrorRad = THREE.MathUtils.degToRad(errDeg);
                scenario.startX = planeState.position.x; scenario.startZ = planeState.position.z;
                setWarning("Kompasset er feilkalibrert - farkosten driver sakte i QLOITER. Bytt til QHOVER og land.", false, 5000);
            }
            const TOILETBOWL_MAX_DRIFT_M = 35;
            const active = planeState.flightMode === "qloiter" && !planeState.onGround;
            qloiterHeadingErrorRad = active ? scenario.headingErrorRad : 0;
            if (active) {
                const driftM = Math.hypot(planeState.position.x - scenario.startX, planeState.position.z - scenario.startZ);
                if (driftM >= TOILETBOWL_MAX_DRIFT_M) {
                    scenario.result = "fail";
                    scenario.failReason = "for lenge i QLOITER med feilkalibrert kompass - drev for langt unna";
                    return;
                }
            } else if (!scenario.recognizedAt) {
                scenario.recognizedAt = performance.now();
                setWarning("Bra - driften stanser når QLOITER forlates. Land i QHOVER.", true, 3500);
            }
            // "må være landet i 3 sekunder før reset for neste gjennomføring" (brukeren) - godkjente
            // tidligere i SAMME frame hjulene traff bakken, uten noen holdetid (i motsetning til de vanlige
            // land-manual-stegene, se VTOL_LANDING_SETTLE_SEC-bruken der). Samme mønster her nå, med sitt
            // eget tall (VTOL_SCENARIO_LANDED_HOLD_SEC).
            if (scenario.recognizedAt && planeState.onGround) {
                scenario.groundedHoldSec += dt;
                if (scenario.groundedHoldSec >= VTOL_SCENARIO_LANDED_HOLD_SEC) scenario.result = "pass";
            } else {
                scenario.groundedHoldSec = 0;
            }
        }
    },
    // Runde 2 (ex6) - "ta av og hovre, så får man fly away mot publikum og må kille." Samme
    // innkomst-forsinkelse som runde 1, deretter styringstap (samme "spøkelses-pinne"-mekanisme som den
    // tidligere kill-scenarioet) mot den PERMANENTE publikumsgruppen (VTOL_CROWD_CENTER, simulator-vtol.js)
    // - ekte avstands-kollisjon mot BÅDE piloten og publikum kjøres nå UBETINGET hver frame
    // (checkVtolPersonCollision der), denne update()-funksjonen leser bare planeState.injured.
    flyawayCrowd: {
        setup: function () {
            resetPlane(GROUND_SPAWN_YAW_RAD); // nullstiller også planeState.injured/injuredTarget - se resetPlane
            setWarning("Ta av og hold en stabil hover i QLOITER.", true, 4000);
            return { settleUntil: null, faultTriggered: false };
        },
        update: function (scenario) {
            if (planeState.injured) {
                scenario.result = "fail";
                scenario.failReason = planeState.injuredTarget === "pilot" ? "traff piloten" : "traff en person i publikum";
                exerciseAutopilotFn = null;
                return;
            }
            if (planeState.crashed) { scenario.result = "fail"; scenario.failReason = "krasjet"; return; }
            if (!scenario.settleUntil) {
                if (currentAltitude() > 2) scenario.settleUntil = performance.now() + VTOL_SCENARIO_SETTLE_SEC * 1000;
                return;
            }
            if (!scenario.faultTriggered) {
                if (performance.now() < scenario.settleUntil) return;
                scenario.faultTriggered = true;
                exerciseAutopilotFn = flyawayCrowdAutopilot;
                setWarning("Farkosten mister styringen og flyr mot publikum! Aktiver KILL umiddelbart (K).", false, 6000);
                return;
            }
            if (!planeState.engineOn) {
                scenario.result = "pass";
                exerciseAutopilotFn = null;
            }
        }
    },
    // Ny øvelse (ex6b) - "VLOS kamera drona er i fwba oppe i lufta. noen sekunder på å forberede seg, ta
    // kontroll, så stopper motorene. Øvelsen er fullført etter glidelanding på rullebanen. krasj er ok så
    // lenge det er i trygg avstand fra folk og hus, og krasjen ikke er for hard. Skal helst glidelande uten
    // krasj" (brukeren) - se ENGINE_FAILURE_PREP_SEC/-TOO_HARD_SINK_MS/isNearRunwayXZ over.
    engineFailureGlide: {
        setup: function () {
            // 200 m ut langs rullebanens senterlinje, 70 m høyde, FBWA, pekende hjemover - "på finale" idet
            // motoren stopper, ikke en umulig snuoperasjon under press på selve førsteforsøket.
            teleportAirborne(0, 70, RUNWAY_NEAR_Z - 200, 180, "fbwa", 16);
            setWarning("Gjør deg klar - motorfeil om " + ENGINE_FAILURE_PREP_SEC + " sekunder. Ta kontroll i FBWA.", true, ENGINE_FAILURE_PREP_SEC * 1000);
            return { failAtMs: performance.now() + ENGINE_FAILURE_PREP_SEC * 1000, failed: false, lastVy: 0, crashSeverityChecked: false, hadCrash: false, wasAirborneAfterFail: false, groundedHoldSec: 0 };
        },
        update: function (scenario, dt) {
            if (planeState.injured) {
                scenario.result = "fail";
                scenario.failReason = planeState.injuredTarget === "pilot" ? "traff piloten" : "traff en person i publikum";
                return;
            }
            if (!scenario.failed) {
                if (performance.now() < scenario.failAtMs) return;
                scenario.failed = true;
                setEngine(false);
                setWarning("MOTORSTOPP! Glid til rullebanen og land.", false, 6000);
            }
            // Håndhevet UBETINGET hver tick ETTER selve feilen - et K-trykk/kill-bryter-omslag skal ikke
            // kunne "starte motoren igjen" etter en ekte motorfeil (samme "ikke lov å jukse seg unna selve
            // poenget"-prinsipp som updateReturnManualStage bruker mot QRTL, se der).
            setEngine(false);

            // "krasjen ikke er for hard" - et krengevinkel-/pitch-utløst krasj regnes alltid som for hardt
            // (ingen god måte å gradere DET på), et rent synkefart-utløst krasj kun om synkefarten var
            // markant over selve krasje-terskelen (CRASH_SINK_RATE, simulator-vtol.js - sjekket her via
            // FORRIGE ticks vertikalfart, FØR triggerCrash() sin egen CRASH_ENERGY_LOSS_FRAC-demping rakk å
            // skalere den ned inneværende tick).
            const priorVy = scenario.lastVy;
            scenario.lastVy = planeState.velocity.y;
            if (planeState.crashed && !scenario.crashSeverityChecked) {
                scenario.crashSeverityChecked = true;
                scenario.hadCrash = true;
                const bankPitch = currentPitchBankDeg();
                const steepAttitude = Math.abs(bankPitch.bankDeg) >= CRASH_BANK_DEG || Math.abs(bankPitch.pitchDeg) >= CRASH_PITCH_DEG;
                if (steepAttitude || priorVy < -ENGINE_FAILURE_TOO_HARD_SINK_MS) {
                    scenario.result = "fail";
                    scenario.failReason = "krasjlandet for hardt";
                    return;
                }
            }

            if (currentAltitude() > 2) scenario.wasAirborneAfterFail = true;
            if (scenario.wasAirborneAfterFail && planeState.onGround) {
                if (isNearRunwayXZ(planeState.position.x, planeState.position.z)) {
                    scenario.groundedHoldSec += dt;
                    if (scenario.groundedHoldSec >= VTOL_LANDING_SETTLE_SEC) scenario.result = "pass";
                } else {
                    scenario.groundedHoldSec = 0;
                    setWarning("Endte for langt fra rullebanen - prøv igjen.", false, 3500);
                }
            } else {
                scenario.groundedHoldSec = 0;
            }
        }
    }
};

/* Enkel, rent visuell "bemannet luftfartøy" - tre bokser (skrog + vinge + hale) som glir over rullebanen. */
let trafficPlaneGroup = null, trafficPlaneStartMs = 0;
const TRAFFIC_PLANE_DURATION_MS = 9000;
function buildTrafficPlaneVisual() {
    removeTrafficPlaneVisual();
    const group = new THREE.Group();
    const mat = new THREE.MeshStandardMaterial({ color: 0xe8e8e8 });
    const fuselage = new THREE.Mesh(new THREE.BoxGeometry(1.2, 1, 8), mat);
    const wing = new THREE.Mesh(new THREE.BoxGeometry(10, 0.3, 1.6), mat);
    wing.position.z = 0.5;
    const tail = new THREE.Mesh(new THREE.BoxGeometry(3.5, 0.25, 1), mat);
    tail.position.z = -3.5;
    group.add(fuselage, wing, tail);
    group.position.set(-220, 90, RUNWAY_SPAWN_Z - 100);
    scene.add(group);
    trafficPlaneGroup = group;
    trafficPlaneStartMs = performance.now();
}
function updateTrafficPlaneVisual() {
    if (!trafficPlaneGroup) return;
    const t = clamp((performance.now() - trafficPlaneStartMs) / TRAFFIC_PLANE_DURATION_MS, 0, 1);
    trafficPlaneGroup.position.x = -220 + t * 440;
    if (t >= 1) removeTrafficPlaneVisual();
}
function removeTrafficPlaneVisual() {
    if (trafficPlaneGroup) { scene.remove(trafficPlaneGroup); trafficPlaneGroup = null; }
}

function updateScenarioStage(stage, dt) {
    if (!exerciseState.scenario || exerciseState.scenario.key !== stage.key) {
        const def = VTOL_SCENARIOS[stage.key];
        exerciseState.scenario = Object.assign({ key: stage.key, result: null }, def.setup());
    }
    const scenario = exerciseState.scenario;
    if (scenario.result === "pass") {
        exerciseState.scenario = null;
        removeTrafficPlaneVisual();
        advanceStage();
        return;
    }
    if (scenario.result === "fail") {
        failStage(scenario.failReason || "feil reaksjon");
        exerciseState.scenario = null; // gir nytt forsøk på samme scenario ved neste tick
        removeTrafficPlaneVisual();
        // Publikums-/skade-scenarioet (flyawayCrowd) rydder IKKE opp skadebanneret her ved en
        // "injured"-feil - eleven skal få se konsekvensen (skadebanner + farkost i bakken/stanset) et lite
        // øyeblikk før neste forsøk starter, akkurat som quad-simulatorens egen injured-visning. planeState.
        // injured nullstilles i stedet av resetPlane() ved selve NESTE forsøkets setup().
        exerciseAutopilotFn = null;
        return;
    }
    VTOL_SCENARIOS[stage.key].update(scenario, dt);
}

/* ==================== Hoved-oppdatering (kalt fra animate() i simulator-vtol.js) ====================
   To separate innkoblingspunkter, IKKE ett - se BEGRUNNELSE:
   - applyVtolExerciseAutopilot(): MÅ kjøres FØR fysikk-løkken (rett etter updateInput(), samme sted som
     RTL-autopiloten selv skrives inn fra INNI stepPhysics, se js/simulator-vtol-rtl.js). updateInput()
     kalles ÉN gang per renderet frame og overskriver inputState.stick fra ekte tastatur/gamepad-input -
     hadde de syntetiske pinneverdiene her blitt skrevet ETTER fysikk-løkken (slik oppdaterings-/HUD-delen
     under gjør), ville de blitt overskrevet av NESTE frames updateInput() FØR fysikken noensinne fikk
     lest dem - autopiloten ville dermed aldri hatt noen reell effekt.
   - updateVtolExercise(): kjøres ETTER fysikk-løkken (samme sted som updateHud/updateRtlHud) - grading og
     HUD skal lese planeState/rtlState SLIK DE ER ETTER at fysikken for dette frameet har kjørt. */
function applyVtolExerciseAutopilot() {
    if (exerciseAutopilotFn) exerciseAutopilotFn();
}

function updateVtolExercise(dt, now) {
    updateExerciseHud();
    if (!exerciseState.active || exerciseState.awaitingNext) return;

    const stage = getStage();
    switch (stage.type) {
        case "hover": updateHoverStage(stage, dt); break;
        case "land-manual": updateLandManualStage(stage, dt); break;
        case "return-manual": updateReturnManualStage(dt); break;
        case "waypoints": updateWaypointsStage(stage, dt); break;
        case "climb": updateClimbStage(stage); break;
        case "transition-out": updateTransitionOutStage(); break;
        case "transition-back": updateTransitionBackStage(); break;
        case "depart-distance": updateDepartDistanceStage(stage); break;
        case "await-mode": updateAwaitModeStage(stage); break;
        case "hold-mode-until-landed": updateHoldModeUntilLandedStage(stage, dt); break;
        case "scenario": updateScenarioStage(stage, dt); break;
        case "await-arm": updateAwaitArmStage(); break;
        case "await-disarm": updateAwaitDisarmStage(); break;
    }
}

function updateExerciseHud() {
    const bar = document.getElementById("exerciseHudBar");
    const banner = document.getElementById("exerciseWarningBanner");
    if (exerciseState.active && !exerciseState.awaitingNext) {
        bar.style.display = "flex";
        document.getElementById("exerciseHudStage").textContent = getStage().label;
        const stage = getStage();
        // "øvelsen avslutter ikke?" (brukeren, ex1) - denne feltet viste bare et hardkodet "OK" for de
        // fleste steg-typene, uansett hva som faktisk blokkerte fremgang (avstand til hjem, holdetid,
        // armert/disarmert) - umulig å se HVA som manglet fra HUD-en alene. Nå viser den faktisk
        // fremgang for hver stegtype, ikke bare hover/waypoints.
        let statusText = "OK";
        if (stage.type === "hover") statusText = formatMMSS(exerciseState.hoverHoldSec) + " / " + formatMMSS(stage.holdSec);
        else if (stage.type === "waypoints") {
            const target = stage.waypoints[exerciseState.wpIndex];
            // "trenger ikke måle antall grader på nesa eller den avstanden. er bare distraherende med
            // tallene i HUD-en" (brukeren, først om ex2-firkanten, senere samme rapport om ex4: "trenger
            // ikke den avstanden og vinkelen der. veldig distraherende og unødvendig info") -
            // relativeBearingText ble opprinnelig lagt til for ex4 (chase-kamera, veipunktene langt
            // unna/ute av syne, se relativeBearingText sin egen kommentar), men viste seg unødvendig/
            // distraherende der også. Ingen steg setter lenger stage.showBearing (se ex4 sine steg over) -
            // funksjonen/feltet er beholdt urørt (billig, ufarlig dødt alternativ) i tilfelle en fremtidig
            // øvelse faktisk trenger det, men vises altså ikke noe sted akkurat nå.
            statusText = exerciseState.wpIndex + " / " + stage.waypoints.length +
                (stage.showBearing && target ? " · " + relativeBearingText(target) : "");
        }
        else if (stage.type === "land-manual" || stage.type === "return-manual" || stage.type === "hold-mode-until-landed") {
            if ((stage.requireMode || stage.requireQMode) && planeState.onGround && !stageModeOk(stage)) {
                statusText = "feil modus (" + (MODE_LABELS[planeState.flightMode] || planeState.flightMode) + ")";
            } else {
                statusText = planeState.onGround
                    ? formatMMSS(exerciseState.groundedHoldSec) + " / " + formatMMSS(VTOL_LANDING_SETTLE_SEC) +
                      (horizDistFromHome() < VTOL_LANDING_RADIUS_M ? "" : " (for langt fra hjem)")
                    : "i luften";
            }
        }
        else if (stage.type === "await-arm") statusText = planeState.armed ? "armert" : "venter";
        else if (stage.type === "await-disarm") statusText = planeState.armed ? "venter" : "disarmert";
        else if (stage.type === "depart-distance") statusText = Math.round(horizDistFromHome()) + " / " + stage.minDist + " m";
        // "øvelse 4 tooltips. må komme opp en melding om å klatre til 20 m" (brukeren) - "climb"-steget
        // (ex3/ex4/ex6b sin egen klatring til VTOL_TRANSITION_MIN_ALT FØR overgang) viste tidligere bare
        // det uforanderlige, hardkodede "OK"-standardtallet uansett faktisk høyde - ingen løpende
        // tilbakemelding på hvor langt unna målet eleven faktisk er, i motsetning til de andre stegtypene.
        else if (stage.type === "climb") statusText = Math.round(currentAltitude()) + " / " + stage.minAlt + " m";
        // "ved bytte til manuell. ta med at det er 5 på tastaturet på tooltipet" (brukeren) - MODE_KEY_LABELS
        // (simulator-vtol.js, lastet FØR denne filen) har den samme tastatursnarveien help-panelet/
        // moduspopoveren selv viser - IKKE en egen, duplisert "manual=5"-oppslagstabell her.
        else if (stage.type === "await-mode") {
            statusText = "nåværende: " + (MODE_LABELS[planeState.flightMode] || planeState.flightMode) +
                (MODE_KEY_LABELS[stage.mode] ? " (tast " + MODE_KEY_LABELS[stage.mode] + " for " + (MODE_LABELS[stage.mode] || stage.mode) + ")" : "");
        }
        document.getElementById("exerciseHudStatus").textContent = statusText;
        document.getElementById("exerciseHudTimer").textContent = formatMMSS((performance.now() - exerciseState.startTime) / 1000);
        updateGcsScreenFields();
        document.getElementById("hudModeItem").classList.toggle("exercise-mode-hint", stageNeedsModeSwitch(stage));
    } else {
        bar.style.display = "none";
        document.getElementById("hudModeItem").classList.remove("exercise-mode-hint");
    }
    const text = document.getElementById("exerciseWarningText");
    const show = performance.now() < exerciseState.warningUntil;
    banner.classList.toggle("show", show);
    banner.classList.toggle("sim-banner-success", exerciseState.warningIsSuccess);
    if (show) text.textContent = exerciseState.warningMessage;
}

// GCS-skjermens telemetrifelter - se HTML-widgeten (#gcsScreen) og openGcsScreen/closeGcsScreen.
function updateGcsScreenFields() {
    document.getElementById("gcsMode").textContent = MODE_LABELS[planeState.flightMode] || "-";
    document.getElementById("gcsAlt").textContent = currentAltitude().toFixed(1) + " m";
    document.getElementById("gcsAirspeed").textContent = lastAirspeed.toFixed(1) + " m/s";
    document.getElementById("gcsDistance").textContent = Math.round(horizDistFromHome()) + " m";
    document.getElementById("gcsBattery").textContent = Math.round(rtlState.batteryPct) + " %";
}

/* ==================== Veiviser/quiz (ex0/ex7 - "special", ingen 3D-flyging) ====================
   Egen, enkel tilstand+UI-motor for de to ikke-fly-baserte leksjonene (se special:"wizard"/"quiz" på
   selve øvelsesdataene over) - deler #specialExerciseOverlay (simulator-vtol.html) mellom begge. Bevisst
   IKKE bygget på exerciseState/startVtolExercise (som forutsetter en faktisk flytur - planeClass-bytte,
   resetPlane, exerciseHudBar osv.) - disse to rører aldri planeState i det hele tatt. */
let specialExerciseState = null; // { id, exercise, stepIndex, quizScore, quizAnswered }
// "når diplomet er åpent. pass på at tastatur og fjernkontroll ikke gir kommandoer til simulatoren i
// bakgrunnen" (brukeren) - diplomet (openVtolDiploma under) kan åpnes MENS en øvelse/flytur fortsatt
// pågår (fra "Se diplom"-raden i selve øvelseslisten), i motsetning til veiviseren/quizen (som alltid
// nullstiller flyet FØR den vises, se startVtolSpecialExercise). Egen boolsk flagg (ikke gjenbruk av
// specialExerciseState, som har en annen FORM - {id, exercise, stepIndex, ...} - og andre konsumenter, se
// renderSpecialExerciseStep) - lest av backgroundControlBlocked() i simulator-vtol.js (lastet FØR denne
// filen, samme cross-file-global-mønster som resten av øvelsesintegrasjonen, se specialExerciseState).
let vtolDiplomaOpen = false;

// "under fjernkontroll oppsett og quiz pass på at man ikke kan fly i bakgrunnen. virker distraherende at
// flyet plutselig rører på seg i bakgrunnen" (brukeren) - updateInput() blokkerer allerede FERSK
// styreinnput mens specialExerciseState er aktiv (se den tidligere fiksen i simulator-vtol.js), men
// FYSIKKEN fortsatte uendret å kjøre i bakgrunnen - et fly som tilfeldigvis var luftbårent/i bevegelse idet
// eleven åpnet veiviseren/quizen (f.eks. fra Øvelser-panelet midt i en økt) fortsatte da å falle/drifte/
// holde en Q-modus synlig bak selve overlegget. Veiviseren/quizen er eksplisitt "ingen 3D-flyging i det
// hele tatt" (se toppkommentaren i filen) - resetPlane() her fjerner enhver bevegelse ved å sette flyet
// trygt til ro på bakken FØR overlegget vises, samme GROUND_SPAWN_YAW_RAD-konvensjon som resten av
// øvelsesprogrammet (se startVtolExercise).
function startVtolSpecialExercise(id) {
    const exercise = VTOL_EXERCISES[id];
    resetPlane(GROUND_SPAWN_YAW_RAD);
    specialExerciseState = { id: id, exercise: exercise, stepIndex: 0, quizScore: 0, quizAnswered: false };
    document.getElementById("exercisesPanel").style.display = "none";
    document.getElementById("specialExerciseOverlay").style.display = "flex";
    renderSpecialExerciseStep();
}

function closeVtolSpecialExercise() {
    specialExerciseState = null;
    document.getElementById("specialExerciseOverlay").style.display = "none";
    showVtolExercisePanel();
    showVtolExerciseList();
}

function completeVtolSpecialExercise() {
    vtolExerciseProgress[specialExerciseState.id] = { passed: true };
    saveVtolExerciseProgress();
    closeVtolSpecialExercise();
}

function renderSpecialExerciseStep() {
    const s = specialExerciseState;
    const optionsEl = document.getElementById("wizardOptions");
    const explanationEl = document.getElementById("wizardExplanation");
    const backBtn = document.getElementById("wizardBackBtn");
    const nextBtn = document.getElementById("wizardNextBtn");
    const closeBtn = document.getElementById("wizardCloseBtn");
    const bindGrid = document.getElementById("wizardBindGrid");
    // "litt forstyrrende at vindusstørrelsen endrer seg hele tiden [i quizen]" (brukeren) - se
    // .sim-wizard-card-quiz i css/style.css - låser kortets høyde KUN for quiz-visningen (spørsmål OG
    // resultatskjermen), ikke ex0-veiviseren (se klasse-kommentaren der for hvorfor).
    document.getElementById("wizardCard").classList.toggle("sim-wizard-card-quiz", s.exercise.special === "quiz");
    optionsEl.innerHTML = "";
    explanationEl.style.display = "none";
    closeBtn.style.display = "none";
    nextBtn.style.display = "";
    backBtn.style.display = "";
    bindGrid.style.display = "none";
    bindGrid.innerHTML = "";

    if (s.exercise.special === "wizard") {
        const step = s.exercise.wizardSteps[s.stepIndex];
        document.getElementById("wizardStepLabel").textContent = "Fjernkontroll-oppsett";
        // "Se om det er mulig å få med noen passende illustrasjoner på fjerknotroll oppsett kortene og på
        // quizen" (brukeren) - step.icon (valgfritt FontAwesome-klassenavn, se ex0WizardSteps) som et lite,
        // tematisk passende ikon foran selve steg-tittelen - samme "<i class=...></i> tekst"-mønster som
        // resten av programmet allerede bruker for øvelses-/quiz-spørsmål-ikoner andre steder.
        document.getElementById("wizardTitle").innerHTML = step.icon
            ? '<i class="fa-solid ' + step.icon + ' sim-exercise-icon"></i> ' + step.title
            : step.title;
        document.getElementById("wizardBody").textContent = step.body;
        // "det var jo meningen at brukeren får sette opp fjernkontrollen her. Klikke på knapp der inne og
        // så trykke på fjernkontrollen for å binde knappene. Så slipper man å knote i menyen." (brukeren) -
        // gjenbruker den ALLEREDE eksisterende bindingsmotoren (buttonManager/BUTTON_ACTIONS/gamepadMap,
        // se simulator-vtol.js) og samme rad-bygger (Sim.buildGamepadButtonsGrid) som Gamepad-panelet ellers
        // bruker - ekte "Sett"/"Fjern"-binding rett i selve veiviseren, ikke bare en instruks om å gå gjøre
        // det et annet sted. step.bindActions er en delmengde av BUTTON_ACTION_LABELS-nøklene (se
        // ex0WizardSteps).
        if (step.bindActions && step.bindActions.length) {
            const labels = {};
            step.bindActions.forEach(function (action) { labels[action] = BUTTON_ACTION_LABELS[action]; });
            bindGrid.style.display = "";
            // "det er jo forskjell på motor emergency stop og disarm. og så kommer binding motor AV?? her
            // skal vi bare binde emergency stop. disarm er jo fast på stikkene" (brukeren) - "Motor AV"/
            // "Motor PÅ" (BUTTON_ACTION_LABELS.engineOff/-On) er de SAMME generiske knapp-handling-navnene
            // Bindinger-panelet ellers bruker for Motor Emergency Stop sine to diskrete triggere (se
            // DEFAULT_GAMEPAD_MAP-kommentaren, simulator-vtol.js) - riktige i seg selv, men rett etter en
            // tekst som nettopp kontrasterer "Motor Emergency Stop" mot "disarm" kan de lett mistolkes som
            // noe ANNET enn selve nødstopp-bryteren (disarm er jo bevisst IKKE bindbar - kun pinne-gesten).
            // step.bindNote (valgfritt) gir et lite, steg-spesifikt presiseringsnotat rett under selve
            // knapperaden - ikke en generell endring av BUTTON_ACTION_LABELS (som Bindinger-panelet ellers
            // bruker uendret, utenfor denne konteksten). Lagt til ETTER buildGamepadButtonsGrid-kallet -
            // den funksjonen nullstiller containerEl.innerHTML selv ved hvert kall, så et notat satt inn
            // FØR ville blitt visket bort igjen med det samme.
            Sim.buildGamepadButtonsGrid(bindGrid, gamepadMap.buttons, labels, buttonManager, getActiveGamepad, saveGamepadMap);
            if (step.bindNote) {
                const note = document.createElement("p");
                note.className = "sim-panel-hint";
                note.textContent = step.bindNote;
                bindGrid.appendChild(note);
            }
        }
        document.getElementById("wizardProgress").textContent = "Steg " + (s.stepIndex + 1) + " av " + s.exercise.wizardSteps.length;
        backBtn.disabled = s.stepIndex === 0;
        nextBtn.textContent = s.stepIndex === s.exercise.wizardSteps.length - 1 ? "Fullfør" : "Neste";
        return;
    }

    // Quiz - siste "steg" (index === quizQuestions.length) er en syntetisk resultat-skjerm, ikke et
    // spørsmål (se completeVtolSpecialExercise-kallet der/quizPassFraction-sjekken).
    const questions = s.exercise.quizQuestions;
    if (s.stepIndex >= questions.length) {
        const fraction = s.quizScore / questions.length;
        const passed = fraction >= (s.exercise.quizPassFraction || 0.8);
        document.getElementById("wizardStepLabel").textContent = "Resultat";
        document.getElementById("wizardTitle").textContent = passed ? "Bestått!" : "Ikke bestått ennå";
        document.getElementById("wizardBody").textContent = s.quizScore + " av " + questions.length + " riktige (krever minst " +
            Math.ceil((s.exercise.quizPassFraction || 0.8) * questions.length) + ").";
        document.getElementById("wizardProgress").textContent = "";
        backBtn.style.display = "none";
        if (passed) {
            nextBtn.textContent = "Fullfør";
            nextBtn.onclick = completeVtolSpecialExercise;
        } else {
            nextBtn.textContent = "Prøv igjen";
            nextBtn.onclick = function () {
                s.stepIndex = 0; s.quizScore = 0; s.quizAnswered = false;
                renderSpecialExerciseStep();
            };
        }
        closeBtn.style.display = "";
        return;
    }
    nextBtn.onclick = defaultWizardNextHandler;
    const q = questions[s.stepIndex];
    document.getElementById("wizardStepLabel").textContent = "Teoriprøve";
    document.getElementById("wizardTitle").textContent = "Spørsmål " + (s.stepIndex + 1) + " av " + questions.length;
    // q.icon (valgfritt, se VTOL_QUIZ_QUESTIONS) - samme "<i>...</i> tekst"-mønster som wizardsteg-tittelen
    // over, her på selve spørsmålsteksten (wizardTitle er bare den generiske "Spørsmål N av M"-telleren,
    // ikke tema-spesifikk nok til å bære et ikon meningsfullt).
    document.getElementById("wizardBody").innerHTML = q.icon
        ? '<i class="fa-solid ' + q.icon + ' sim-exercise-icon"></i> ' + q.question
        : q.question;
    document.getElementById("wizardProgress").textContent = "Poeng så langt: " + s.quizScore + " / " + s.stepIndex;
    backBtn.style.display = "none";
    s.quizAnswered = false;
    nextBtn.style.display = "none";
    q.options.forEach(function (optionText, i) {
        const btn = document.createElement("button");
        btn.type = "button";
        btn.className = "sim-wizard-option";
        btn.textContent = optionText;
        btn.addEventListener("click", function () { answerQuizQuestion(q, i, btn); });
        optionsEl.appendChild(btn);
    });
}

// Standard "Neste"-håndtering for veiviser-steg (quiz-spørsmål setter sin egen nextBtn.onclick i
// answerQuizQuestion under, siden den må vente til eleven faktisk har svart).
function defaultWizardNextHandler() {
    const s = specialExerciseState;
    if (s.exercise.special === "wizard") {
        if (s.stepIndex >= s.exercise.wizardSteps.length - 1) { completeVtolSpecialExercise(); return; }
        s.stepIndex++;
        renderSpecialExerciseStep();
    }
}
// "Pass på at gode svaralternativer, så det ikke er åpenbart hva som er riktig" (brukeren) - umiddelbar
// fasit+forklaring vises med en gang eleven svarer (ikke først til slutt), samme pedagogiske prinsipp som
// resten av introprogrammet (øyeblikkelig tilbakemelding, se setWarning-bruken andre steder).
function answerQuizQuestion(question, chosenIndex, chosenBtn) {
    const s = specialExerciseState;
    if (s.quizAnswered) return;
    s.quizAnswered = true;
    const correct = chosenIndex === question.correctIndex;
    if (correct) s.quizScore++;
    const optionsEl = document.getElementById("wizardOptions");
    Array.prototype.forEach.call(optionsEl.children, function (btn, i) {
        btn.disabled = true;
        if (i === question.correctIndex) btn.classList.add("sim-wizard-option-correct");
        else if (btn === chosenBtn) btn.classList.add("sim-wizard-option-wrong");
    });
    const explanationEl = document.getElementById("wizardExplanation");
    explanationEl.textContent = (correct ? "Riktig! " : "Feil. ") + question.explanation;
    explanationEl.style.display = "block";
    const nextBtn = document.getElementById("wizardNextBtn");
    nextBtn.style.display = "";
    const isLastQuestion = s.stepIndex === s.exercise.quizQuestions.length - 1;
    nextBtn.textContent = isLastQuestion ? "Se resultat" : "Neste spørsmål";
    nextBtn.onclick = function () {
        s.stepIndex++;
        renderSpecialExerciseStep();
    };
}

/* ==================== Panel-UI (liste + detalj) ==================== */
function renderVtolExerciseList() {
    const container = document.getElementById("exerciseListItems");
    container.innerHTML = "";
    // "Diplom knappen i menyen. når diplomet er tilgjengelig skal den knappen være øverste valg i menyen"
    // (brukeren) - bygget/lagt til FØRST nå (var tidligere lagt til SIST, etter hele forEach-løkken under,
    // som satte den nederst i listen).
    if (allVtolExercisesPassed()) {
        const diplomaRow = document.createElement("button");
        diplomaRow.type = "button";
        diplomaRow.className = "sim-exercise-row sim-exercise-row-diploma";
        diplomaRow.innerHTML =
            '<span class="sim-exercise-row-icon"><i class="fa-solid fa-award"></i></span>' +
            '<span class="sim-exercise-row-main">' +
            '<span class="sim-exercise-row-title">Alle øvelser bestått!</span>' +
            '<span class="sim-exercise-row-desc">Se diplomet ditt</span></span>';
        diplomaRow.addEventListener("click", openVtolDiploma);
        container.appendChild(diplomaRow);
    }
    VTOL_EXERCISE_ORDER.forEach(function (id) {
        const exercise = VTOL_EXERCISES[id];
        const progress = vtolExerciseProgress[id] || { passed: false };
        const row = document.createElement("button");
        row.type = "button";
        row.className = "sim-exercise-row";
        row.innerHTML =
            '<span class="sim-exercise-row-icon"><i class="fa-solid ' + exercise.icon + '"></i></span>' +
            '<span class="sim-exercise-row-main">' +
            '<span class="sim-exercise-row-title">' + exercise.label + "</span>" +
            '<span class="sim-exercise-row-desc">' + exercise.shortDescription + "</span>" +
            "</span>" +
            (progress.passed ? '<span class="sim-exercise-check"><i class="fa-solid fa-circle-check"></i></span>' : "");
        row.addEventListener("click", function () { showVtolExerciseDetail(id); });
        container.appendChild(row);
    });
}
// Kategorivisning ("hvilket fartøy") FØR selve øvelseslisten - i dag kun ett program (Heewing T2 Cruza
// VTOL), men lagt opp med samme tre-nivås struktur som quad-simulatorens øvelsespanel (kategori -> liste
// -> detalj, se js/simulator.js) slik at flere fartøy/øvelsesprogrammer kan legges til som egne knapper
// i exerciseCategoryView senere uten å bygge om selve liste-/detaljvisningen.
function showVtolExerciseCategoryView() {
    document.getElementById("exerciseCategoryView").style.display = "block";
    document.getElementById("exerciseListView").style.display = "none";
    document.getElementById("exerciseDetailView").style.display = "none";
}
function showVtolExerciseList() {
    document.getElementById("exerciseCategoryView").style.display = "none";
    document.getElementById("exerciseListView").style.display = "block";
    document.getElementById("exerciseDetailView").style.display = "none";
    renderVtolExerciseList();
}
let vtolExerciseDetailId = null;
function showVtolExerciseDetail(id) {
    vtolExerciseDetailId = id;
    const exercise = VTOL_EXERCISES[id];
    document.getElementById("exerciseCategoryView").style.display = "none";
    document.getElementById("exerciseListView").style.display = "none";
    document.getElementById("exerciseDetailView").style.display = "block";
    document.getElementById("exerciseDetailTitle").innerHTML =
        '<i class="fa-solid ' + exercise.icon + ' sim-exercise-icon"></i> ' + exercise.label;
    document.getElementById("exerciseDetailDescription").textContent = exercise.fullDescription;
    const progressEl = document.getElementById("exerciseDetailProgress");
    const progress = vtolExerciseProgress[id];
    if (progress && progress.passed) {
        progressEl.style.display = "block";
        progressEl.textContent = "Bestått.";
    } else {
        progressEl.style.display = "none";
    }
    const isRunning = exerciseState.active && exerciseState.exerciseId === id && !exerciseState.awaitingNext;
    document.getElementById("exerciseStartBtn").style.display = isRunning ? "none" : "";
    document.getElementById("exerciseCancelBtn").style.display = isRunning ? "" : "none";
}

function showVtolExerciseSummary(id) {
    const exercise = VTOL_EXERCISES[id];
    const overlay = document.getElementById("exerciseSummary");
    document.getElementById("exerciseSummaryTitle").textContent = "Bestått: " + exercise.label;
    const justCompletedAll = allVtolExercisesPassed();
    document.getElementById("exerciseSummaryText").textContent = justCompletedAll
        ? "Alle øvelsene i Heewing T2 introprogrammet er nå bestått!"
        : "Bra jobbet. Klar for neste øvelse?";
    const nextBtn = document.getElementById("exerciseNextBtn");
    const nextIndex = VTOL_EXERCISE_ORDER.indexOf(id) + 1;
    const nextId = VTOL_EXERCISE_ORDER[nextIndex];
    // "Bra jobbet-meldingen i øvelsen. Må få med der hva neste øvelse heter og hva den går ut på så
    // brukeren er forberedt" (brukeren) - "Klar for neste øvelse?" over nevnte aldri HVILKEN øvelse eller
    // hva den faktisk innebar, kun at det FANTES en neste. Legger nå til navn+kortbeskrivelse (samme
    // shortDescription-felt selve øvelseslisten viser, se renderVtolExerciseList) i et eget avsnitt.
    const nextInfoEl = document.getElementById("exerciseSummaryNextInfo");
    if (!justCompletedAll && nextId) {
        const nextExercise = VTOL_EXERCISES[nextId];
        nextInfoEl.textContent = "Neste: " + nextExercise.label + " - " + nextExercise.shortDescription;
        nextInfoEl.style.display = "";
    } else {
        nextInfoEl.style.display = "none";
    }
    if (justCompletedAll) {
        nextBtn.textContent = "Se diplom";
        nextBtn.onclick = function () { overlay.style.display = "none"; openVtolDiploma(); };
    } else if (nextId) {
        nextBtn.textContent = "Neste øvelse";
        // ex7 (teoriprøve) er "special" (ingen 3D-flyging) - MÅ gjennom startVtolSpecialExercise, ikke
        // startVtolExercise (som forutsetter exercise.stages, og ville feilet på ex7 sin manglende det).
        nextBtn.onclick = function () {
            overlay.style.display = "none";
            const nextExercise = VTOL_EXERCISES[nextId];
            if (nextExercise.special) startVtolSpecialExercise(nextId);
            else startVtolExercise(nextId);
        };
    } else {
        nextBtn.style.display = "none";
    }
    document.getElementById("exerciseSummaryCloseBtn").onclick = function () {
        overlay.style.display = "none";
        exerciseState.awaitingNext = false;
        exerciseState.active = false;
        restorePreviousPlaneClass();
        showVtolExercisePanel();
        showVtolExerciseList();
    };
    overlay.style.display = "";
}

// "Diplom øvelsesbeskrivelse... er for knotete beskrivelse. Heller f.eks '1. Hover / Holde fartøyet i
// stabil hover i forskjellige Q-moduser'" (brukeren, med konkrete forslag for flere øvelser) - egne, korte
// tittel+læringsmål-par KUN for diplomet, IKKE en endring av exercise.label/shortDescription selv (de
// brukes fortsatt slik de er i selve øvelseslisten/detaljvisningen - se renderVtolExerciseList - som har
// et annet formål og trenger mer presise, fullstendige beskrivelser enn diplomets korte oppsummering).
const VTOL_DIPLOMA_GOALS = {
    ex0: { title: "Oppsett", desc: "Sette opp fjernkontroll og modusvalg." },
    ex1: { title: "1. Hover", desc: "Holde fartøyet i stabil hover i forskjellige Q-moduser." },
    ex2: { title: "2. Firkant", desc: "Fly firkant i forskjellige Q-moduser." },
    ex3: { title: "3. Landingsrunde", desc: "Med og uten vind. Landing i Q-modus." },
    ex4: { title: "4. Rundflyging", desc: "Manøvrering i manuell modus." },
    ex5: { title: "5. Manuell retur", desc: "Finne veien hjem og lande manuelt i vind." },
    ex6: { title: "6. Uforutsette hendelser", desc: "Bytte til QHOVER ved kompassfeil. Bruk av killswitch ved kontrolltap." },
    ex6b: { title: "7. Motorfeil", desc: "Motorstopp med glidelandinger på rullebanen." },
    ex7: { title: "8. Teoriprøve", desc: "Bestått skriftlig prøve om Heewing T2 og ArduPilot." }
};
// "Få med total flytid i simulator på diplomet" (brukeren) - getVtolTotalFlightSec (simulator-vtol.js) er
// akkumulert luftbåren tid, IKKE bare inneværende økt (se dens egen kommentar). Format: t/min for alt over
// et minutt (en total over flere økter blir fort for stort for MM:SS, se formatMMSS-kommentaren), rene
// sekunder for de aller første minuttene av trening.
function formatVtolFlightTime(sec) {
    sec = Math.max(0, Math.round(sec));
    if (sec < 60) return sec + " sek";
    const totalMin = Math.floor(sec / 60);
    const h = Math.floor(totalMin / 60), m = totalMin % 60;
    return h > 0 ? h + "t " + m + "min" : m + " min";
}
function openVtolDiploma() {
    const overlay = document.getElementById("diplomaOverlay");
    // "liste opp alt som er gjennomgått nedover... de viktigste 'læringsmålene' under hver øvelse. Kort og
    // konsist" (brukeren) - samme "én rad per øvelse"-idé som quad-simulatorens eget diplom (openDiploma,
    // js/simulator.js), men uten en TID (de fleste VTOL-øvelsene her er ikke tidtatt, se noTiming-bruken).
    // "Få med ikonene til hver øvelse på diplomet" (brukeren) - exercise.icon (samme Font Awesome-glyf
    // øvelseslisten selv viser) gjenbrukt direkte.
    const goalsEl = document.getElementById("diplomaGoals");
    goalsEl.innerHTML = "";
    VTOL_EXERCISE_ORDER.forEach(function (id) {
        const exercise = VTOL_EXERCISES[id];
        const goal = VTOL_DIPLOMA_GOALS[id];
        const row = document.createElement("div");
        row.className = "sim-diploma-goal-row";
        row.innerHTML =
            '<span class="sim-diploma-goal-icon"><i class="fa-solid ' + exercise.icon + '"></i></span>' +
            '<span class="sim-diploma-goal-main">' +
            '<span class="sim-diploma-goal-title">' + goal.title + "</span>" +
            '<span class="sim-diploma-goal-desc">' + goal.desc + "</span></span>";
        goalsEl.appendChild(row);
    });
    document.getElementById("diplomaFlightTime").textContent = "Total flytid i simulator: " + formatVtolFlightTime(getVtolTotalFlightSec());
    document.getElementById("diplomaDate").textContent =
        "Dato: " + new Date().toLocaleDateString("nb-NO", { year: "numeric", month: "2-digit", day: "2-digit" });
    document.getElementById("diplomaPrintBtn").onclick = function () {
        // "må ikke være mulig å lagre uten navn. lys svakt rødt opp navnefeltet hvis man prøver å lagre
        // uten navn" (brukeren) - trim() slik at et navnefelt med KUN mellomrom (nå gyldig å skrive inn,
        // se diplomaNameInput-fiksen i initVtolExercisePanel) fortsatt telles som tomt.
        const nameInput = document.getElementById("diplomaNameInput");
        if (!nameInput.value.trim()) {
            nameInput.classList.remove("sim-diploma-name-error");
            void nameInput.offsetWidth; // tving reflow - restarter CSS-animasjonen selv ved gjentatte klikk på rad
            nameInput.classList.add("sim-diploma-name-error");
            nameInput.focus();
            return;
        }
        document.body.classList.add("printing-diploma");
        window.print();
        document.body.classList.remove("printing-diploma");
    };
    document.getElementById("diplomaCloseBtn").onclick = function () { vtolDiplomaOpen = false; overlay.style.display = "none"; };
    vtolDiplomaOpen = true;
    overlay.style.display = "";
}

function showVtolExercisePanel() { document.getElementById("exercisesPanel").style.display = "block"; }
function toggleVtolExercisesPanel() {
    const panel = document.getElementById("exercisesPanel");
    const wasOpen = panel.style.display !== "none";
    Sim.closeAllMenus(panel);
    if (wasOpen) {
        panel.style.display = "none";
    } else {
        panel.style.display = "block";
        // Har eleven en øvelse i gang, gå rett til dens detaljvisning (med "Avbryt" synlig) i stedet for
        // kategorivalget - reflekterer den faktiske tilstanden fremfor å late som ingenting kjører.
        if (exerciseState.active && !exerciseState.awaitingNext) showVtolExerciseDetail(exerciseState.exerciseId);
        else showVtolExerciseCategoryView();
    }
}

/* ==================== Oppstart ==================== */
function initVtolExercisePanel() {
    document.getElementById("toggleExercisesBtn").addEventListener("click", toggleVtolExercisesPanel);
    document.getElementById("categoryHeewingT2Btn").addEventListener("click", showVtolExerciseList);
    document.getElementById("exerciseBackToCategoryBtn").addEventListener("click", showVtolExerciseCategoryView);
    document.getElementById("exerciseBackToListBtn").addEventListener("click", showVtolExerciseList);
    // "legg til mulighet for å resette hele øvelsesprogresjonen. i tilfelle noen andre skal gjennom
    // programmet" (brukeren) - se resetVtolExerciseProgress-kommentaren. Bekreftelsesdialog (window.confirm)
    // siden dette sletter ALLE bestått-merkene på én gang, uten angre-mulighet - ingen tilsvarende
    // bekreftelse finnes for flightLogClearBtn (kun en ren feilsøkingslogg, langt lavere innsats å miste).
    document.getElementById("exerciseResetProgressBtn").addEventListener("click", function () {
        if (window.confirm("Nullstille all øvelsesfremgang? Alle bestått-merker fjernes, og kan ikke angres.")) {
            resetVtolExerciseProgress();
            renderVtolExerciseList();
        }
    });
    document.getElementById("exerciseStartBtn").addEventListener("click", function () {
        // ex0 (veiviser) / ex7 (quiz) - "special", ingen 3D-flyging - se startVtolSpecialExercise.
        const exercise = VTOL_EXERCISES[vtolExerciseDetailId];
        if (exercise.special) startVtolSpecialExercise(vtolExerciseDetailId);
        else startVtolExercise(vtolExerciseDetailId);
    });
    document.getElementById("exerciseCancelBtn").addEventListener("click", function () {
        stopVtolExercise();
        showVtolExerciseList();
    });
    document.getElementById("wizardBackBtn").addEventListener("click", function () {
        const s = specialExerciseState;
        if (!s || s.stepIndex === 0) return;
        s.stepIndex--;
        renderSpecialExerciseStep();
    });
    document.getElementById("wizardNextBtn").addEventListener("click", function () { defaultWizardNextHandler(); });
    document.getElementById("wizardCloseBtn").addEventListener("click", closeVtolSpecialExercise);
    // "utsjekksprogram meny. må ha en free flight knapp for å kunne forlate øvelsesmodus" (brukeren) -
    // exerciseCancelBtn ("Avbryt") fantes fra før, men lever INNI exercisesPanel, som startVtolExercise selv
    // skjuler (display:none) i det øvelsen faktisk starter - eneste vei tilbake dit var å taste M på nytt
    // for å åpne panelet midt i flygingen, lite oppdagbart. Denne knappen lever i stedet i selve
    // exerciseHudBar-en (alltid synlig gjennom HELE øvelsen, se startVtolExercise), for en direkte, synlig
    // vei ut til fri flyging når som helst.
    document.getElementById("exerciseHudExitBtn").addEventListener("click", function () {
        stopVtolExercise();
    });
    document.getElementById("gcsScreenTab").addEventListener("click", openGcsScreen);
    document.getElementById("gcsScreenCloseBtn").addEventListener("click", closeGcsScreen);
    document.getElementById("gcsScreenBackdrop").addEventListener("click", closeGcsScreen);
    // "'Skriv navnet ditt her' hjelpeteksten må forsvinne når brukeren klikker i navnefeltet for å skrive"
    // (brukeren) - et vanlig placeholder-attributt forsvinner først når man begynner å SKRIVE, ikke ved
    // selve fokuset. Tømmer/gjenoppretter derfor placeholder-attributtet manuelt på focus/blur i stedet.
    // Wired ÉN gang her (ikke i openVtolDiploma, som kjøres på nytt hver gang diplomet åpnes) - elementet
    // finnes allerede i det statiske HTML-et, ingen grunn til å risikere flere stablede lyttere.
    // sim-diploma-name-error (se diplomaPrintBtn over) fjernes idet animasjonen er ferdig, av samme grunn.
    (function () {
        const nameInput = document.getElementById("diplomaNameInput");
        const originalPlaceholder = nameInput.placeholder;
        nameInput.addEventListener("focus", function () { nameInput.placeholder = ""; });
        nameInput.addEventListener("blur", function () { nameInput.placeholder = originalPlaceholder; });
        nameInput.addEventListener("animationend", function () { nameInput.classList.remove("sim-diploma-name-error"); });
    })();
    window.addEventListener("keydown", function (e) {
        if (e.repeat) return;
        if (e.code === "KeyM") toggleVtolExercisesPanel();
        // R (tilbakestill fly) skal også nullstille selve øvelsesfremdriften i gjeldende steg, ikke bare
        // flyets fysiske tilstand - resetPlane() selv (kjøres av simulator-vtol.js sin egen R-håndtering)
        // rører ingen av disse feltene.
        if (e.code === "KeyR" && exerciseState.active && !exerciseState.awaitingNext) {
            exerciseState.hoverHoldSec = 0;
            exerciseState.wpIndex = 0;
            exerciseState.scenario = null;
            exerciseAutopilotFn = null;
            removeTrafficPlaneVisual();
            exerciseState.startTime = performance.now();
            enterStageVisuals();
        }
    });
    // "mulig å ha en lenke som tar brukeren til vtol siden med Heewing programmet (menyen) åpent?"
    // (brukeren) - ?exercises=1 i URL-en (f.eks. simulator-vtol.html?exercises=1) åpner
    // øvelses-/introprogram-panelet direkte på selve LISTEN over øvelser (hopper over kategorivalget - kun
    // én kategori finnes uansett, se exerciseCategoryView/categoryHeewingT2Btn) i stedet for å kreve at
    // eleven trykker M/øvelser-knappen selv først. Ren query-param, ingen ruting/historikk-endring - deles
    // som en helt vanlig lenke.
    if (new URLSearchParams(location.search).has("exercises")) {
        showVtolExercisePanel();
        showVtolExerciseList();
    }
}
