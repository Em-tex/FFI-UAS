/* js/simulator-vtol-rtl.js
   QRTL (QuadPlane Return-To-Launch) - autopilot-lag oppå kjernefysikken i simulator-vtol.js. Lastes
   ETTER den filen (se simulator-vtol.html) og kjører i SAMME globale scope - simulator-vtol.js er selv
   ikke IIFE-pakket, så alt herfra (planeState, inputState, vtolParams, isQMode, trySetFlightMode,
   computeMcAuthority, QLOITER_MAX_SPEED, MC_MAX_CLIMB_RATE, MC_ALT_HOLD_DEADBAND, RUNWAY_SPAWN_Z, clamp,
   scene, THREE, Sim ...) er direkte tilgjengelig som globale identifikatorer, akkurat som
   simulator-common.js sin Sim-namespace allerede er det der. (lastAirspeed er samme mønster - global `let`
   fra simulator-vtol.js, oppdatert FØR denne filens updateRtlAutopilot kalles hver tick, se stepPhysics -
   så verdien brukt her er ETT tick gammel, akkurat som trySetFlightMode sin egen MANUAL-fartssjekk
   allerede gjør med samme variabel - uvesentlig etterslep ved 120Hz.)

   Bakgrunn: den ekte QuadPlane QRTL-modusen (se ArduPilot-dokumentasjonen, sitert av brukeren) er en
   HYBRID - ikke en ren VTOL-retur: "If closer than 1.5X the larger of either RTL_RADIUS or
   WP_LOITER_RAD, then the vehicle will proceed toward home in VTOL mode and land. If greater, it will
   transition to fixed wing, climbing toward RTL_ALTITUDE ... The further away, the higher the climb."
   DEFAULT_RTL_PARAMS.transitionRadiusM er nettopp denne (1.5×max(RTL_RADIUS,WP_LOITER_RAD))-terskelen,
   forenklet til ÉN brukerjusterbar radius i stedet for to separate ArduPilot-parametre + en faktor -
   dette ER "samme logikk som bestemmer om flyet går i fastvingemodus eller hovermodus" som brukeren
   spesifikt ba om.

   Tilstandsmaskinen har bevisst KUN tre faser (cruise/vtol_return/land_final), ikke en egen "klatre
   først, transisjoner så"-fase slik ArduPilot sin tekst kan leses isolert: siden computeMcAuthority
   (gjenbrukt UENDRET, se updateRtlAutopilot) allerede gir FULL løftemotor-assistanse helt til
   luftfarten når vtolParams.assistSpeed, får en QRTL som starter fra lav fart i hover automatisk akkurat
   den oppførselen ArduPilot selv beskriver ("the quad motors will provide assistance with lift and
   attitude while the forward motor starts to pull the aircraft forward" - se "I switch into RTL mode
   while hovering", sitert av brukeren) UTEN noen egen klatre-fase i det hele tatt - assist-nedtrappingen
   ER klatrefasen. */

/* ---------- Parametre (Q_*-navngivning, se rtlPanel-tooltipsene) ---------- */
const RTL_PARAMS_STORAGE_KEY = "ffi-uas:vtol-rtl-params";
const DEFAULT_RTL_PARAMS = {
    // Q_RTL_ALT: høyde (m) å holde/klatre mot mens QRTL navigerer hjem (både i cruise- og
    // VTOL-retur-fasen, se updateRtlAutopilot).
    rtlAltM: 15,
    // Q_WP_SPD: ønsket marsjfart (m/s) i VTOL-retur-fasen (rampes ned mot 0 idet flyet nærmer seg hjem,
    // se RTL_APPROACH_SLOWDOWN-kommentaren).
    qWpSpeedMs: 5,
    // Q_WP_SPD_DN: synkefart (m/s) mot landFinalAltM.
    wpSpeedDnMs: 1,
    // Q_LAND_FINAL_ALT: høyde AGL (m) der finalnedstigningen begynner.
    landFinalAltM: 6,
    // Q_LAND_FINAL_SPD: synkefart (m/s) under landFinalAltM.
    landFinalSpeedMs: 0.5,
    // Vist i panelet som "RTL_RADIUS" (brukeren ba eksplisitt om å kunne sette denne under det navnet) -
    // tilsvarer ekte ArduPilot sin 1.5×max(RTL_RADIUS,WP_LOITER_RAD), se toppkommentaren, forenklet til ÉN
    // brukerjusterbar terskel i stedet for to separate parametre + en faktor. Innenfor denne avstanden fra
    // hjem flyr QuadPlanet rett hjem i VTOL-modus; utenfor går den over til fastvinget marsjflyging først.
    transitionRadiusM: 40,
    // Pusher-gass (0-1) i cruise-fasen (fastvinget marsjflyging mot hjem). Satt til FULL gass som
    // standard, ikke en moderat marsjfart-verdi - dette er en retur (evt. en batteri-failsafe), ikke en
    // vanlig tur, og "still undermotorisert" var en direkte tilbakemelding (se også
    // mcAuthority-skaleringen på stick.pitch over, som var hovedårsaken, men full gass hjelper i tillegg
    // med å bygge fart fortest mulig gjennom den assisterte overgangen).
    cruiseThrottleFrac: 1.0,
    // Q_OPTIONS bit 16 ("disable fixed wing RTL and approach"): tving REN VTOL-retur uansett avstand.
    pureVtolOnly: false,
    // Q_RTL_MODE (brukeren ba eksplisitt om denne, med hele ArduPilot-dokumentasjonsteksten limt inn) -
    // hvilken overordnet RTL-STRATEGI som brukes, se rtlMode-bruken i updateRtlAutopilot for hvordan hver
    // verdi bygger på (ikke duplikerer) de samme cruise-/loiter-/vtol_return-grenene:
    //   0 = "Fixed Wing RTL": sirkler som fastvinget fly rundt hjem FOR ALLTID (loiter, se
    //       LOITER_RADIUS_M-bruken) - transisjonerer ALDRI til VTOL-landing (ekte ArduPilot: "VTOL motors
    //       will not be used unless airspeed drops below Q_ASSIST_SPEED" - selve løftemotor-ASSISTANSEN
    //       kan uansett kicke inn via den vanlige computeMcAuthority-faden, men det er IKKE en landing).
    //   1 = "Switch to QRTL at radius": rett fastvinget innflyging (L1-styring) helt til
    //       transitionRadiusM/RTL_RADIUS, INGEN loiter-fase - transisjonerer direkte til vtol_return idet
    //       avstand+fart-vilkårene er oppfylt.
    //   2 = "Loiter to altitude, switch to QRTL": fastvinget retur, så en EKTE sirkel-nedstigning (se
    //       LOITER_RADIUS_M-grenen) ned mot rtlAltM idet flyet nærmer seg, FØR overgang til vtol_return.
    //   3 = "Approach, switch to QRTL" (ArduPilot sin EGNE standard for selve QRTL-modusen - dokumentasjonen
    //       er eksplisitt: "switching to QRTL mode will act exactly as Q_RTL_MODE = 3" UANSETT hva selve
    //       Q_RTL_MODE-parameteren er satt til, siden QRTL er en egen modus, ikke bare "RTL" - derfor er
    //       DETTE standardverdien her). Ekte Q_RTL_MODE=3 har en egen "airbraking"/lineær-nedstigningsslope-
    //       finmekanikk (Q_TRANS_DECEL, "VTOL Position1/2") - forenklet her til NØYAKTIG samme
    //       loiter-til-høyde-oppførsel som modus 2, siden den finmekanikken ligger under denne
    //       simulatorens fidelitetsnivå (treningsverdien av selve STRATEGIEN - fastvinget retur, kontrollert
    //       nedstigning, så VTOL - er den samme uansett).
    rtlMode: 3,
    // Forenklet batteri-/utholdenhetsproxy (ikke ekte mAh/spenning) - se updateBattery. Utholdenhet i ren
    // hover vs. ren fastvinget cruise ved full gass.
    batteryHoverMinutes: 12,
    batteryCruiseMinutes: 30,
    // Simulert lavspenning-failsafe - trigger automatisk QRTL når batteriet krysser terskelen.
    batteryFailsafeEnabled: true,
    batteryFailsafePercent: 20
};
function loadRtlParams() { return Sim.loadJSON(RTL_PARAMS_STORAGE_KEY, DEFAULT_RTL_PARAMS); }
function saveRtlParams() { Sim.saveJSON(RTL_PARAMS_STORAGE_KEY, rtlParams); }
const rtlParams = loadRtlParams();

// Interne regulator-forsterkninger - IKKE eksponert i panelet (samme prinsipp som QLOITER_VEL_TO_LEAN_DEG
// eller STABILIZED_BANK_D_GAIN i simulator-vtol.js: dette er implementasjonsdetaljer i kontrolloven, ikke
// noe en elev skal trenge å stille på - selve RTL-*oppførselen* styres av DEFAULT_RTL_PARAMS over).
const RTL_CRUISE_ALT_P_M = 15;          // høydeavvik (m) for fullt stigningsror-utslag i cruise-fasen
const RTL_APPROACH_SLOWDOWN_FACTOR = 2; // VTOL-retur begynner å bremse ved (qWpSpeedMs*denne) meters avstand
const RTL_LANDED_SPEED_MS = 0.3;        // fart under hvilken flyet regnes som "i ro" på bakken
const RTL_LANDED_DWELL_S = 1.5;         // hvor lenge den må stå i ro (på bakken) før motor kuttes automatisk
// RTL_DECEL_ZONE_FACTOR/RTL_DECEL_ZONE_MIN_M - se decelZoneM-bruken i updateRtlAutopilot: hvor langt FØR
// transitionRadiusM cruise-fasen begynner å kutte gassen for å bremse ned mot en trygg overgangsfart.
const RTL_DECEL_ZONE_FACTOR = 4;   // decelerasjonssonen starter ved transitionRadiusM * denne faktoren...
const RTL_DECEL_ZONE_MIN_M = 60;   // ...men aldri kortere enn dette (m), selv med en liten transitionRadiusM

/* ---------- Hjem + batteri + fase-tilstand ---------- */
const rtlState = {
    home: new THREE.Vector3(0, 0, RUNWAY_SPAWN_Z),
    homeSet: false,
    phase: "idle",       // "idle" | "cruise" | "vtol_return" | "land_final" | "landed"
    failsafeTriggered: false,
    batteryPct: 100,
    landedTimer: 0,
    failsafeBannerUntil: 0,
    // Startposisjonen for gjeldende cruise-BEN (rekaptureres hver gang fasen går INN i "cruise" - se
    // wasNotCruise i updateRtlAutopilot) - brukt av L1-lignende siktepunkt-styringen der, IKKE bare et
    // vilkårlig navn; dette ER selve "banen" (rett linje fra der cruise startet, til hjem) L1-styringen
    // sikter et stykke foran på, se lang BUG-kommentar der.
    cruiseLegStart: new THREE.Vector3(),
    // Fast krengeretning (+1/-1) for loiter-sirkelen rundt hjem, 0 = ikke satt ennå (settes én gang idet
    // loiteren først entres, nullstilles idet cruise forlates - se loiterSign-bruken i updateRtlAutopilot).
    loiterSign: 0
};

// Kalles fra simulator-vtol.js på STIGENDE flanke av motor PÅ (setEngine/toggleEngine/resetPlane) - se
// integrasjonspunktet der. Ekte ArduPilot: "home position is initially established at the time the plane
// acquires its GPS lock ... updated as long as the autopilot is disarmed" - denne simulatoren har ikke
// noe eget arm/disarm-begrep utover motor PÅ/AV, så "motor PÅ" ER arming-øyeblikket her.
// setEngine/toggleEngine gater selve KALLET på planeState.onGround (brukertilbakemelding: "nytt RTL punkt
// settes når motorene restartes i lufta. det skal ikke være mulig. kun lov å sette nytt RTL punkt etter
// at dronen har landet trygt") - en restart midt i lufta skal ALDRI overskrive et allerede etablert
// hjem-punkt med det midlertidige luft-punktet. resetPlane derimot tvinger onGround=true FØR den kaller
// hit (fullstendig simulator-reset til bakken), så den treffer alltid denne samme, ubetingede funksjonen.
function captureHome() {
    rtlState.home.copy(planeState.position);
    rtlState.homeSet = true;
    rtlState.failsafeTriggered = false;
    rtlState.batteryPct = 100;
    rtlState.phase = "idle";
    rtlState.landedTimer = 0;
    rtlState.loiterSign = 0;
    if (homeMarkerMesh) {
        homeMarkerMesh.visible = true;
        // Y=0.05 (IKKE 0.03) - BUG (rapportert av brukeren: "H på bakken som indikerer hjempunkt. blir
        // ikke synlig på rullebanen. blir sikkert liggende under teksturen der") - rullebane-mesh'et (se
        // buildRunway i simulator-vtol.js) ligger på Y=0.04, altså FYSISK OVER markøren her (0.03) i
        // verdensrom - polygonOffset (se groundDecalProps) biaser kun selve DYBDEBUFFER-presisjonen ved
        // rastrering, det endrer ikke hvilken av to FAKTISK ulike Y-høyder som er nærmest kameraet, så
        // rullebanens ugjennomsiktige asfalt-teksturt tegnet uforanderlig OVER (skjulte) H-en når hjemmet
        // ble satt på selve rullebanen. 0.05 matcher samme "trygt over rullebanen"-konvensjon andre hevede
        // bakke-dekaler i simulator-vtol.js allerede bruker (se f.eks. buildPavedCircle/veidekk-kommentaren
        // der: "0.02 flimret (z-fighting) mot bakkeplanet under").
        homeMarkerMesh.position.set(rtlState.home.x, 0.05, rtlState.home.z);
    }
}

/* ---------- Hjem-markør i scenen ----------
   En flat, delvis gjennomsiktig "H" (helipad-oppmerking) malt rett på bakken - en flaggstang viste seg
   "litt rar" (brukeren rapporterte det direkte) sammenlignet med en flat bakke-dekal, som er den vanlige,
   gjenkjennelige måten å markere et landingspunkt på (samme prinsipp som groundDecalProps allerede bruker
   for rullebane-/plen-/dam-teksturene, se buildGround-området). */
function buildHomeMarkerTexture() {
    const size = 256;
    const canvas = document.createElement("canvas");
    canvas.width = size;
    canvas.height = size;
    const ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, size, size);
    ctx.strokeStyle = "rgba(255,150,20,0.7)";
    ctx.lineWidth = size * 0.035;
    ctx.beginPath();
    ctx.arc(size / 2, size / 2, size * 0.46, 0, Math.PI * 2);
    ctx.stroke();
    ctx.fillStyle = "rgba(255,150,20,0.7)";
    const barW = size * 0.12, barH = size * 0.5, crossH = size * 0.11;
    ctx.fillRect(size * 0.28, size * 0.25, barW, barH);
    ctx.fillRect(size * 0.60, size * 0.25, barW, barH);
    ctx.fillRect(size * 0.28, size * 0.445, size * 0.44, crossH);
    return new THREE.CanvasTexture(canvas);
}
let homeMarkerMesh = null;
function initRtlHomeMarker() {
    const mat = new THREE.MeshStandardMaterial(groundDecalProps({
        map: buildHomeMarkerTexture(), transparent: true, depthWrite: false
    }));
    homeMarkerMesh = new THREE.Mesh(new THREE.PlaneGeometry(6, 6), mat);
    homeMarkerMesh.rotation.x = -Math.PI / 2;
    // 0.05 - se captureHome sin egen Y-kommentar for hvorfor (må ligge OVER rullebanens Y=0.04, ikke
    // under den). Denne startverdien overskrives uansett av captureHome() ved første motor-PÅ, men satt
    // riktig fra start for konsistens.
    homeMarkerMesh.position.y = 0.05;
    homeMarkerMesh.visible = false; // skjult til første captureHome()
    scene.add(homeMarkerMesh);
}

/* ---------- Batteri (forenklet utholdenhetsproxy, IKKE ekte mAh/spenning) ---------- */
// Forbruket interpoleres mellom ren cruise- og ren hover-tapping via mcAuthority (0=fastvinget cruise,
// 1=full svevemyndighet) - samme tall stepPhysics allerede regner ut hver tick (se
// computeMcAuthority-kallet), gjenbrukt direkte i stedet for å anslå motorpådrag på nytt. Ett tick
// etterslep (forrige ticks mcAuthority) - uvesentlig for en størrelse som endrer seg over minutter.
function updateBattery(dt, hoverWeight) {
    if (!planeState.engineOn || planeState.crashed) return;
    const hoverDrainPerSec = 100 / Math.max(1, rtlParams.batteryHoverMinutes * 60);
    const cruiseDrainPerSec = 100 / Math.max(1, rtlParams.batteryCruiseMinutes * 60);
    const drainPerSec = THREE.MathUtils.lerp(cruiseDrainPerSec, hoverDrainPerSec, clamp(hoverWeight, 0, 1));
    rtlState.batteryPct = Math.max(0, rtlState.batteryPct - drainPerSec * dt);

    if (rtlParams.batteryFailsafeEnabled && !rtlState.failsafeTriggered
        && rtlState.batteryPct <= rtlParams.batteryFailsafePercent
        && planeState.flightMode !== "qrtl") {
        rtlState.failsafeTriggered = true;
        trySetFlightMode("qrtl");
        rtlState.failsafeBannerUntil = performance.now() + 5000;
    }
}

/* ---------- QRTL-autopilot ----------
   Kalt KUN når planeState.flightMode === "qrtl" (se stepPhysics-integrasjonen). Skriver syntetiske
   verdier RETT INN i inputState.stick (samme objekt ekte pilotinput bruker) FØR resten av stepPhysics
   leser det, og returnerer en "effektiv modus" ("fbwa" i cruise-fasen, "qloiter" ellers) som
   stepPhysics bruker i stedet for det bokstavelige "qrtl" alle stedene kontrolloven i dag leser
   planeState.flightMode (mcAuthority, QLOITER/FBWA-valg, kollektiv/pusher-valg osv.) - se
   controlMode-kommentaren i stepPhysics. planeState.flightMode selv rører denne funksjonen ALDRI - HUD,
   lagring og tastatur-guard skal fortsatt se "qrtl". */
function updateRtlAutopilot(dt) {
    const stick = inputState.stick;
    const pos = planeState.position;
    const altitude = Math.max(0, pos.y);
    const toHome = new THREE.Vector3(rtlState.home.x - pos.x, 0, rtlState.home.z - pos.z);
    const horizDist = toHome.length();
    const toHomeDir = horizDist > 0.05 ? toHome.clone().multiplyScalar(1 / horizDist) : new THREE.Vector3(0, 0, -1);
    // Brukt BÅDE til fartsrampen (se desiredSpeed under) og til å GATE land_final-overgangen (se under) -
    // uten distanse-sjekket der kunne QRTL trigges lavt og langt unna hjem (f.eks. rett etter en
    // lav avgang) og bli tolket som "allerede i finalnedstigning" bare fordi altitude<=landFinalAltM,
    // selv om den fortsatt hadde hele reisen mot hjem igjen.
    const slowdownStartM = Math.max(3, rtlParams.qWpSpeedMs * RTL_APPROACH_SLOWDOWN_FACTOR);
    const nearHome = horizDist < slowdownStartM;

    // Fastvinget cruise vs. VTOL-retur - selve "samme logikk som bestemmer fastvinge- eller hovermodus"
    // brukeren ba om (se toppkommentaren for hvordan transitionRadiusM svarer til ekte ArduPilot).
    // "land_final"/"landed" er terminale for denne QRTL-armingen - reevalueres ALDRI tilbake til cruise
    // (unngår at flyet begynner å fly fastvinget igjen midt i en finalnedstigning pga. vind-drift).
    // BUG (rapportert av brukeren, BEKREFTET via flightlogg: "Går i VTOL RTL selv om jeg er langt unna.
    // svinger ned og krasjer" - loggen viste modus->vtol_return på ALLERFØRSTE tick etter QRTL-engasjement,
    // fra 17.8m høyde og 26 m/s, langt fra hjem) - roten var et tidligere "belowRtlAlt"-ledd her som tvang
    // UMIDDELBART vtol_return (Q-modus, full QLOITER-fartsregulator) hver gang høyden var under rtlAltM -
    // UANSETT fart eller avstand. Dette var i sin tid en bevisst fiks for et TIDLIGERE problem ("QRTL
    // krasjer i bakken selv om RTL-høyde er satt mye høyere" - flyet klatret aldri fordi fbwa sin
    // kollektiv-gren den gangen kun holdt NÅVÆRENDE synkefart, ikke noe høydemål), men den la samtidig inn
    // NØYAKTIG den samme høyfarts-transisjonsrisikoen (se BUG-historikken for wantVtolBySpeed/decelZoneM
    // under) via en HELT ANNEN inngangsdør: å engasjere QRTL i vanlig fastvinget marsjfart (20-30 m/s) et
    // godt stykke UNDER rtlAltM - som er et helt normalt, forventet scenario (lavtflyging, eller en
    // failsafe som trigger mens flyet allerede er lavt) - er akkurat like farlig som å bytte til
    // vtol_return for tidlig ved transitionRadiusM, og var IKKE beskyttet av samme fartsport.
    // DEN EGENTLIGE FIKSEN: fbwa-kollektiv-grenen i stepPhysics bruker NÅ stick.pitch til å style en EKTE
    // klatrerate (se BUG-kommentaren der, "pull back = climb" fra ArduPilot-dokumentasjonen), og cruise-
    // grenen under setter nå stick.pitch til en EKTE (om enn forsiktig, kun-klatre) høydekommando mot
    // rtlAltM - se stick.pitch-kommentaren lenger ned. Det betyr at "klatre til RTL-høyde" IKKE lenger
    // krever et Q-modus-bytte i det hele tatt: cruise klatrer selv, i fastvinget flukt, akkurat slik ekte
    // ArduPilot QRTL faktisk gjør det ("climbing toward RTL_ALTITUDE" skjer i FASTVINGET modus, med
    // løftemotor-assistanse KUN mens luftfarten ennå er under assistSpeed - Q-modus/VTOL kommer først idet
    // den faktisk nærmer seg hjem, se QRTL-dokumentasjonen sitert av brukeren). belowRtlAlt-leddet er
    // dermed fjernet herfra - vtol_return trigges NÅ UTELUKKENDE av wantVtolBySpeed (avstand OG fart) eller
    // pureVtolOnly, ALDRI av høyde alene, uansett hvor lavt flyet måtte være.
    //
    // BUG (rapportert av brukeren: "bytter til VTOL et stykke før hjempunktet ... la seg i synkende sving
    // og krasjet") - cruise-fasen fløy i konstant gass helt til transitionRadiusM, og hoppet RETT over i
    // vtol_return sin QLOITER-fartsregulator (kommanderer lenevinkel fra fartsAVVIK) ved full cruisefart -
    // et enormt avvik, klemt til MC_MAX_LEAN_ANGLE, som induserer stort drag/delvis steiling. Fikset ved å
    // GATE selve avstandsbaserte overgangen på at luftfarten (lastAirspeed) faktisk er under
    // vtolParams.assistSpeed FØRST - samme terskel som allerede styrer FBWA sin egen løftemotor-
    // autoritetsfade, så mcAuthority er per konstruksjon allerede ~1 idet overgangen skjer.
    //
    // BUG (rapportert av brukeren: "hopper inn i cruise, så VTOL frem og tilbake mens den svinger rundt")
    // - fasevalget ble reevaluert fra bunnen hver tick, også mens allerede i vtol_return: momentum kunne
    // seile flyet forbi hjem og ut igjen forbi transitionRadiusM, hoppende TILBAKE til cruise. Ekte
    // ArduPilot reverserer aldri et slikt engangs-beslutningspunkt. Fikset: cruise/vtol_return-valget under
    // kjøres nå KUN mens fasen fortsatt er "cruise" (eller "idle" ved første engasjement), ALDRI mens
    // allerede i vtol_return (som land_final/landed allerede var unntatt).
    const wasNotCruise = rtlState.phase !== "cruise";
    if (rtlState.phase === "cruise" || rtlState.phase === "idle") {
        // rtlMode===0 ("Fixed Wing RTL") sirkler for alltid - se DEFAULT_RTL_PARAMS.rtlMode-kommentaren -
        // og skal derfor ALDRI selv-transisjonere basert på avstand/fart, uansett hvor sakte den blir.
        // pureVtolOnly (Q_OPTIONS bit 16) er en eksplisitt brukerkommandert override og tvinger fortsatt
        // VTOL-retur UANSETT rtlMode - de to er ortogonale brytere i denne forenklingen.
        // BUG/DESIGN-INNSIKT (rapportert av brukeren, direkte fra ArduPilot-dokumentasjonen de nettopp limte
        // inn: "skal den ikke bytte til VTOL og lande når den er så nært home?" - loggen viste flyet
        // sirkle/steile i cruise i 30 sekunder rett ved hjem UTEN å noensinne transisjonere, fordi
        // luftfarten aldri KRYSSET assistSpeed før den steilet - selv om det var trygt nært hjem hele
        // tiden). Sitatet er eksplisitt: "if the approach is entered less than 1.5X MAXRAD, it will
        // IMMEDIATELY move to VTOL Position1 state... whether entered from fixed wing or VTOL" - ekte
        // QRTL krever ALTSÅ IKKE at farten først kommer under assistSpeed før den transisjonerer innenfor
        // radiusen - i stedet bruker den løftemotorene sine EGNE, fulle myndighet til å bremse ("AIRBRAKING
        // phase... spinning up the VTOL motors to create additional braking") ETTER transisjonen, ikke FØR.
        // Fjernet derfor fartsbetingelsen her - loiteren (se LOITER_RADIUS_M-grenen) gjør fortsatt sitt
        // beste for å bremse ned OG holde høyden mens den venter på å komme innenfor selve radiusen, men
        // selve TRANSISJONEN er nå (som ekte QRTL) et rent AVSTANDS-vilkår - vtol_return sin egen QLOITER-
        // fartsregulator (se under) HAR allerede full, umiddelbar løftemotor-autoritet (mcAuthority=1 i
        // Q-modus, se computeMcAuthority) og en velprøvd, konvergerende posisjonsholder - det ER den ekte
        // "airbraking"-mekanismen, ikke noe som mangler.
        const wantVtolByDistance = rtlParams.rtlMode !== 0 && horizDist < rtlParams.transitionRadiusM;
        rtlState.phase = (rtlParams.pureVtolOnly || wantVtolByDistance) ? "vtol_return" : "cruise";
    }
    // Fersk cruise-ben-startposisjon FANGET IDET FASEN GÅR INN I "cruise" (ikke hver tick) - se
    // L1_LOOKAHEAD_M-bruken lenger ned for hvorfor: dette definerer selve LINJA cruise-styringen sikter
    // et stykke foran på.
    if (rtlState.phase === "cruise" && wasNotCruise) rtlState.cruiseLegStart.copy(pos);
    if (rtlState.phase === "vtol_return" && nearHome && altitude <= rtlParams.landFinalAltM) {
        rtlState.phase = "land_final";
    }

    // Samme metode (applyQuaternion direkte, IKKE en algebraisk "roter forward 90°"-utledning) som
    // QLOITER-grenen i stepPhysics selv bruker for bodyForwardFlat/bodyRightFlat - denne kodebasen har
    // gjentatte ganger måttet rette opp fortegnsfeil i akkurat denne typen utledning (se f.eks.
    // currentBankDeg-negasjonen i stepPhysics), så et eget uavhengig "høyre = forward rotert 90°"-forsøk
    // her ville vært unødig risiko. Kun trygt å gjenbruke UENDRET siden den ER nøyaktig samme beregning.
    const q = planeState.quaternion;
    const bodyForwardFlat = new THREE.Vector3(0, 0, -1).applyQuaternion(q);
    bodyForwardFlat.y = 0;
    if (bodyForwardFlat.lengthSq() < 1e-6) bodyForwardFlat.set(0, 0, -1); else bodyForwardFlat.normalize();
    const bodyRightFlat = new THREE.Vector3(1, 0, 0).applyQuaternion(q);
    bodyRightFlat.y = 0;
    if (bodyRightFlat.lengthSq() < 1e-6) bodyRightFlat.set(1, 0, 0); else bodyRightFlat.normalize();

    if (rtlState.phase === "cruise") {
        // DESIGN-AVKLARING FRA BRUKEREN ("men den skal vel loitre over hjempunktet, gå ned på høyde og så
        // bytte til VTOL?") etter to runder med ustabil styring nær hjem (se BUG-historikk lenger ned): en
        // ren "sikt MOT et punkt"-lov (pursuit, evt. L1-lookahead) er PRINSIPPIELT ustabil/oscillerende når
        // flyet blir for nært punktet til å rette ut kursen i tide - uansett hvor godt siktepunktet er
        // konstruert, blir problemet bare flyttet, ikke løst (se den andre BUG-kommentaren). Riktig løsning,
        // bekreftet av brukeren: bytt STYRINGSFORM helt idet flyet er nær hjem - fra "naviger mot et punkt"
        // til en EKTE, FAST-krengende sirkel (loiter) rundt hjem, som synker gradvis mens den venter på at
        // fart/avstand-vilkårene (wantVtolBySpeed) tillater overgang til vtol_return. En konstant krengevinkel
        // er TRIVIELT stabil (ingen bæringsberegning mot et bevegelig/nært punkt i det hele tatt), og en
        // loiter-nedstigning er dessuten en helt vanlig, reell fastvinget prosedyre - ikke en hastverksløsning.
        // rtlMode===1 ("Switch to QRTL at radius") har INGEN loiter-fase i det hele tatt - se
        // DEFAULT_RTL_PARAMS.rtlMode-kommentaren: rett fastvinget innflyging (L1-grenen under) helt til
        // selve transisjonsterskelen. LOITER_RADIUS_M satt til 0 for den modusen sørger for at
        // horizDist<LOITER_RADIUS_M aldri blir sann (horizDist er alltid >=0).
        const LOITER_RADIUS_M = rtlParams.rtlMode === 1 ? 0 : Math.max(rtlParams.transitionRadiusM * 2, 80);
        if (horizDist < LOITER_RADIUS_M) {
            // Loiter: FAST, moderat krengevinkel (LOITER_BANK_DEG) - IKKE styrt av avstand/retning til hjem
            // i det hele tatt, kun retningen (med eller mot klokka, se loiterSign) er fast per QRTL-arming.
            // rtlState.loiterSign settes én gang idet loiter først entres (se under loiterSign-init) - unngår
            // at fortegnet flipper fram og tilbake tick for tick, som ville gitt en S-sving i stedet for en
            // sirkel.
            if (rtlState.loiterSign === 0) {
                // Sirkler i den retningen flyet ALLEREDE svinger svakt mot idet det entrer loiteren (samme
                // fortegn som gjeldende bank, eller med klokka som nøytral standard ved ~0 bank) - unngår et
                // unødig, brått motsatt rorutslag akkurat idet loiteren starter.
                const q = planeState.quaternion;
                const currentBankSign = Math.sign(-THREE.MathUtils.radToDeg(new THREE.Euler().setFromQuaternion(q, "YXZ").z)) || 1;
                rtlState.loiterSign = currentBankSign;
            }
            // BUG (rapportert av brukeren: "innflygningen og oppbremsningen kan beregnes bedre? nå slakker
            // den ned veldig sent. og krasjet igjen" - loggen viste mcAuth FORBLI 0% i alle 30 sekundene
            // (luftfarten kom aldri under assistSpeed), MENS høyden likevel sakte, men stadig raskere,
            // sank hele tiden (endte i et reelt steilfall de siste sekundene, pinneP klemt i bunn -1.00).
            // Roten er FYSISK, ikke bare en autoritets-klemme: ved FAST 22° krengning krever en gitt
            // vekt/løft-balanse MER angrepsvinkel jo LANGSOMMERE flyet blir (løft ∝ fart² ved gitt AoA) -
            // en selvforsterkende felle idet farten faller mot assistSpeed, siden nettopp DA trengs mest
            // løft, men vingen har minst å gi. Å bare øke pinne-autoriteten videre (var allerede hevet til
            // ±0.85, se BUG-historikk) hjelper ikke – det presser bare AoA nærmere steilegrensen, som er
            // nøyaktig det steilfallet på slutten av loggen viser. Ekte løsning (standard flygeteknikk):
            // FLATE UT krengningen etter hvert som farten nærmer seg assistSpeed - en grunnere sving
            // krever mindre løft å holde oppe, akkurat som en pilot ville redusert krengning i en
            // langsom nedstigningssving. loiterBankDeg glir fra LOITER_BANK_DEG_MAX (langt over
            // assistSpeed) ned mot LOITER_BANK_DEG_MIN (nær/under assistSpeed) - samme glidende prinsipp
            // som gasslovens nearAssistBlend under, men på selve krengevinkelen i stedet for gasspaken.
            const LOITER_BANK_DEG_MAX = 22, LOITER_BANK_DEG_MIN = 8;
            const bankSpeedBlend = clamp((lastAirspeed - vtolParams.assistSpeed) / 8, 0, 1); // 0 nær assistSpeed, 1 godt over
            const loiterBankDeg = THREE.MathUtils.lerp(LOITER_BANK_DEG_MIN, LOITER_BANK_DEG_MAX, bankSpeedBlend);
            stick.roll = clamp(rtlState.loiterSign * loiterBankDeg / MAX_BANK_ANGLE, -1, 1);
            // Høyde: TOVEIS her (i motsetning til L1-transitt-grenen under) - loiteren er en stabil,
            // kontrollert sirkel, så en vanlig P-regulator mot rtlAltM er trygt (ingen samtidig kamp om
            // retningskontrollen, se BUG-historikken for hvorfor toveis var farlig i TRANSITT-fasen).
            // "gå ned på høyde" - brukerens eget ønske - er nøyaktig dette: en jevn spiral-nedstigning mot
            // rtlAltM mens den venter på VTOL-overgangen.
            // BUG (rapportert av brukeren, BEKREFTET via flightlogg: "ville krasjet. klarer ikke lande på
            // rtl punktet" - loggen viste en JEVN, UAVBRUTT synkefart fra ~20m helt ned til under 1m over
            // HELE loiteren, med mcAuth=0% - dvs. INGEN løftemotor-hjelp - og gasspaken samtidig kuttet
            // helt til 0 av bremsesonen under (se decelZoneM), mens denne klemmen begrenset elevator-
            // autoriteten til kun ±0.5) - vingen ALENE, uten trekkraft OG med halvert autoritet, hadde
            // rett og slett ikke nok løft til å holde både den faste 22°-krengningen OG rtlAltM samtidig.
            // Flyet ble reddet i siste liten kun fordi farten til slutt falt under assistSpeed og
            // løftemotorene endelig fikk myndighet (se mcAuth-hoppet til 100% helt til slutt i loggen).
            // Fikset TO steder: (1) klemmen her økt til ±0.85 (mer elevator-autoritet å holde høyden med),
            // (2) gasspaken i loiteren styres nå av LUFTFART (se speedErrorMs under), IKKE avstand - en
            // ren avstandsbasert bremsesone (decelZoneM) gir ingen mening i en SIRKEL uansett (avstanden
            // til hjem er ikke monotont minkende rundt en loiter) - og gir et lite, men ALDRI NULL, gulv
            // med trekkraft, nok til å holde krengningen/høyden noenlunde stabilt mens farten likevel
            // synker gradvis (økt indusert drag i svingen) mot assistSpeed.
            const altErrorM = rtlParams.rtlAltM - altitude;
            stick.pitch = clamp(-altErrorM / RTL_CRUISE_ALT_P_M, -0.85, 0.85);
            // BUG (rapportert av brukeren, BEKREFTET via flightlogg: "den klarer aldri å lande. bare flyr
            // rundt i cruise" - loggen viste luftfarten flate ut på 20-24 m/s i over 100 SEKUNDER, aldri i
            // nærheten av å konvergere mot assistSpeed) - formelen under var STIKK BAKVENDT: den klemte
            // gasspaken til MAKSIMUM (speedErrorMs/10 mettet raskt over cruiseThrottleFrac) nettopp mens
            // farten var HØYEST over assistSpeed - altså mest gass akkurat når flyet trengte MINST (og
            // burde bremset via drag i stedet) - som låste flyet i en selvopprettholdende høyfarts-likevekt
            // (gass akkurat nok til å kompensere draget den samtidig kommanderte via 22°-krengningen) som
            // ALDRI konvergerte nedover. Fikset ved å SNU forholdet: LAV gass (kun gulvet) mens det er MYE
            // fart å bremse av - la indusert drag fra selve svingen gjøre jobben, akkurat som en ekte
            // motorredusert spiral-nedstigning - og kun en MODERAT (ikke maks) økning idet farten faktisk
            // nærmer seg assistSpeed, der steilerisikoen begynner å bli reell og løftemotorene ennå ikke
            // har trådt til.
            // BUG (rapportert av brukeren: "ny krasj. bytter aldri til VTOL" - loggen viste farten flate
            // helt ut på 14.2-14.3 m/s i over TOLV SAMMENHENGENDE sekunder - rett over assistSpeed (12
            // m/s), ALDRI under, mcAuth forble 0% hele veien - før den til slutt steilet og krasjet uansett
            // etter 20+ sekunder med pinneP fastlåst nær -0.85 (nesten maks klatrekommando, brukt bare for
            // å holde HØYDEN, ikke faktisk klatre - flyet var marginalt/nær steilegrensen hele denne tiden).
            // Roten: NEAR_ASSIST-gulvet (0.35) var HØYERE enn FLOOR-et (0.15) - altså MER gass akkurat idet
            // farten nærmet seg assistSpeed - kombinert med at krengningen SAMTIDIG flates ut der (mindre
            // indusert drag, se bankSpeedBlend over) traff akkurat en likevekt (trekkraft=drag) et lite
            // knapt stykke OVER assistSpeed - flyet satt fast der for alltid, siden ingenting lenger dro
            // farten videre ned. Fikset ved å SNU denne også (samme idé som forrige gasslov-fiks, bare
            // anvendt på "nær assistSpeed"-enden i stedet for "langt over"-enden): gjenbruker
            // bankSpeedBlend DIREKTE (samme glidende variabel som krengningen - unngår to separate,
            // potensielt usynkroniserte blend-beregninger) - gassen faller nå MOT NESTEN INGENTING idet
            // farten nærmer seg assistSpeed, i stedet for å stige. Trygt fordi krengningen SAMTIDIG flates
            // ut (mindre løftbehov akkurat når det er minst løft å gi) - ingen grunn til å HOLDE IGJEN med
            // ekstra gass der lenger; selve overgangen til vtol_return (full Q-modus-autoritet) er den
            // egentlige sikkerheten idet terskelen faktisk krysses.
            const LOITER_THROTTLE_FAR = 0.30;
            const LOITER_THROTTLE_NEAR_ASSIST = 0.05;
            stick.throttle = THREE.MathUtils.lerp(LOITER_THROTTLE_NEAR_ASSIST, LOITER_THROTTLE_FAR, bankSpeedBlend);
            stick.yaw = 0;
            return "fbwa";
        } else {
            rtlState.loiterSign = 0; // nullstilt til neste gang loiteren entres - se init over
            // L1-lignende siktepunkt (se opprinnelig BUG-kommentar i git-historikken: ren pursuit mot selve
            // hjem-punktet låser seg i en STABIL SIRKEL når målets avstand er mindre enn flyets egen minste
            // svingradius). legStart (fanget idet fasen gikk inn i cruise) -> hjem definerer linja;
            // siktepunktet er flyets egen projeksjon på linja PLUSS L1_LOOKAHEAD_M meter videre - UTEN
            // øvre klemme mot selve hjem (en tidligere versjon klemte til legLen, som lot siktepunktet
            // KOLLAPPE sammen med hjem og reintrodusere nøyaktig samme ustabilitet idet flyet nærmet seg -
            // se den andre BUG-kommentaren). Denne grenen kjører nå UANSETT kun UTENFOR LOITER_RADIUS_M,
            // så siktepunktet rekker aldri å bli farlig nært uansett.
            const legStart = rtlState.cruiseLegStart;
            const legVec = new THREE.Vector3(rtlState.home.x - legStart.x, 0, rtlState.home.z - legStart.z);
            const legLen = legVec.length();
            let aimX = rtlState.home.x, aimZ = rtlState.home.z; // degenerert fallback (legStart ~ hjem)
            if (legLen > 1) {
                const legDirX = legVec.x / legLen, legDirZ = legVec.z / legLen;
                const alongLeg = clamp((pos.x - legStart.x) * legDirX + (pos.z - legStart.z) * legDirZ, 0, legLen);
                // BUG (rapportert av brukeren, med flightlogg: "roll noen ganger kan være ekstremt
                // hakkete/høyfrekvent hakking" - loggen viste pinneR hoppe mellom -1.00 og +1.00 NESTEN
                // HVER ENESTE TICK i flere sekunder, mens banken samtidig hang fast oppunder 45-50°) - en
                // FAST L1_LOOKAHEAD_M (60, uavhengig av fart) er nøyaktig den samme typen ustabilitet som
                // toppkommentaren i denne filen allerede advarer mot for RENT pursuit-mot-hjem ("låser seg
                // i en STABIL SIRKEL når målets avstand er mindre enn flyets egen minste svingradius") -
                // bare flyttet til SIKTEPUNKTET i stedet for selve hjem-punktet. Flyets EGEN minste
                // svingradius (R=v²/(g·tanΦ)) vokser med KVADRATET av farten - ved cruisefart etter
                // "motor boost runde 4" (pusherMaxThrust, se VTOL_CLASSES) på 25-29 m/s og fullt utslått
                // MAX_BANK_ANGLE (50°), er R allerede 70-90+ meter - GODT over den faste 60m-lookaheaden.
                // Idet flyet ikke lenger klarer å svinge trangt nok til å nå et siktepunkt bare 60m foran
                // langs linja, går akkurat samme pursuit-instabilitet i gang: flyet begynner å SIRKLE rundt
                // det (nå relativt sett "for nære") siktepunktet i stedet for å konvergere mot linja, og
                // peilingen til et punkt man sirkler tett rundt endrer seg EKSTREMT raskt (i prinsippet mot
                // uendelig vinkelhastighet helt inntil punktet) - nøyaktig den hakkingen loggen viser.
                // Fikset ved å la lookahead-avstanden SKALERE MED FAKTISK LUFTFART (samme prinsipp som ekte
                // L1-styring for øvrig alltid bruker - L1-avstand er normalt et par sekunders flygetid, ikke
                // et fast metertall), garantert godt over svingradiusen uansett hvor fort flyet flyr.
                const L1_LOOKAHEAD_TIME_S = 4;
                const L1_LOOKAHEAD_M = Math.max(rtlParams.transitionRadiusM * 1.5, 60, lastAirspeed * L1_LOOKAHEAD_TIME_S);
                const aimAlong = alongLeg + L1_LOOKAHEAD_M;
                aimX = legStart.x + legDirX * aimAlong;
                aimZ = legStart.z + legDirZ * aimAlong;
            }
            const toAimX = aimX - pos.x, toAimZ = aimZ - pos.z;
            const aimDist = Math.hypot(toAimX, toAimZ);
            const aimDirX = aimDist > 0.05 ? toAimX / aimDist : toHomeDir.x;
            const aimDirZ = aimDist > 0.05 ? toAimZ / aimDist : toHomeDir.z;

            // Retningsstyring: samme bodyForwardFlat/bodyRightFlat-projeksjon som VTOL-retur-grenen under
            // bruker - unngår risiko for å style svingen FEIL vei (ArduPilot: "a right roll at low speed
            // will cause the aircraft to move to the right...").
            const fwdDot = bodyForwardFlat.x * aimDirX + bodyForwardFlat.z * aimDirZ;
            const rightDot = bodyRightFlat.x * aimDirX + bodyRightFlat.z * aimDirZ;
            // BUG (rapportert av brukeren: "rollen er veldig hakkete noen ganger. og ofte overskyter den
            // svingen og må rolle tilbake") - den GAMLE loven ("fwdDot>=0 ? clamp(rightDot*2,-1,1) :
            // (rightDot>=0?1:-1)") brukte rightDot (~sin(peilingsavvik)) DIREKTE som et proporsjonalt ledd
            // med forsterkning 2 - det METTET dermed til FULLT rorutslag ved kun ~30° peilingsavvik
            // (sin(30°)*2=1), og forble på FULL kommandert bank (opptil MAX_BANK_ANGLE=50° i cruise, se
            // stepPhysics) helt til svingen var nesten fullført. Idet flyet endelig nærmet seg riktig kurs
            // falt den kommanderte banken BRÅTT fra "full" til "liten" over et smalt vindu, mens flyets
            // FAKTISKE bank (med egen treghet/moment fra å ha stått i maks bank) ikke rakk å følge like
            // raskt - resultatet var et systematisk overskudd i svingen som måtte rettes opp igjen i etterkant,
            // akkurat det brukeren beskriver. Byttet til en EKTE peilingsVINKEL (atan2, samme prinsipp som
            // computeWeathervaneYawRateRad - robust også langt forbi ±90°, IKKE en rå sinus-tilnærming som
            // mister presisjon/metter kunstig tidlig) direkte fra de allerede riktig signerte rightDot/
            // fwdDot-verdiene, atan2(rightDot,fwdDot) - IKKE weathervane sin egen cross/dot-formel, som har
            // en annen, ikke-kompatibel fortegnskonvensjon kalibrert for GIRRATE, ikke rull. Selve
            // metnings-VINKELEN er dessuten hevet fra de effektive ~30° over til 75° - kommandert bank
            // avtar dermed GLATT og forutsigbart gjennom hele siste del av svingen i stedet for å henge i
            // taket til siste øyeblikk, som er selve roten til overskytingen. Eliminerer også det gamle,
            // separate "siktepunkt bak flyet"-spesialtilfellet helt - atan2 håndterer allerede HELE
            // ±180°-området glatt og kontinuerlig, uten noen egen gren.
            const headingErrorRad = Math.atan2(rightDot, fwdDot);
            const RTL_ROLL_FULL_DEFLECTION_RAD = THREE.MathUtils.degToRad(75);
            stick.roll = clamp(headingErrorRad / RTL_ROLL_FULL_DEFLECTION_RAD, -1, 1);
            // Høyde: KUN-KLATRE, ALDRI dykk, mens langt unna (se BUG-historikk: en toveis regulator her
            // bygde opp farlig synkefart FØR selve overgangen til loiter/vtol_return). Nivåflukt hvis for
            // høyt (brukerens eget forslag), ekte klatring hvis for lavt (siden belowRtlAlt-tvangen til
            // Q-modus er fjernet - cruise må nå selv nå rtlAltM i fastvinget flukt).
            const altErrorM = rtlParams.rtlAltM - altitude;
            stick.pitch = clamp(-altErrorM / RTL_CRUISE_ALT_P_M, -1, 0);
        }
        // Bremsesone (KUN transitt-grenen over - loiter-grenen returnerer tidlig med sin egen, luftfart-
        // baserte gasslov, se BUG-kommentaren der for hvorfor en avstandsbasert sone ikke gir mening i en
        // sirkel) - gassen trappes LINEÆRT ned fra cruiseThrottleFrac til 0 mellom decelZoneM og
        // transitionRadiusM, og FORBLIR 0 (idle) innenfor.
        const decelZoneM = Math.max(rtlParams.transitionRadiusM * RTL_DECEL_ZONE_FACTOR, rtlParams.transitionRadiusM + RTL_DECEL_ZONE_MIN_M);
        const decelProgress = clamp((decelZoneM - horizDist) / Math.max(1, decelZoneM - rtlParams.transitionRadiusM), 0, 1);
        stick.throttle = rtlParams.cruiseThrottleFrac * (1 - decelProgress);
        stick.yaw = 0;
        return "fbwa";
    }

    // "vtol_return" og "land_final": posisjonsholding mot hjem, samme mekanisme som QLOITER selv bruker
    // (se QLOITER-grenen i stepPhysics) - ønsket VERDENS-fart mot hjem dekomponeres til kropp-relativ
    // forover-/sideveis-fart og mates inn som SYNTETISK stick.pitch/roll (QLOITER-grenen multipliserer
    // stick.pitch*QLOITER_MAX_SPEED for å få ønsket fart - her mates ønsketFart/QLOITER_MAX_SPEED inn
    // igjen, så ingen endring av selve QLOITER-koden trengs). Farten rampes ned nær hjem for å unngå å
    // overskyte/oscillere rundt hjem-punktet - se RTL_APPROACH_SLOWDOWN_FACTOR. slowdownStartM/nearHome
    // er begge beregnet lenger OPPE i funksjonen nå (gjenbrukt av land_final-overgangen der også).
    const desiredSpeed = clamp(horizDist / slowdownStartM, 0, 1) * rtlParams.qWpSpeedMs;
    const desiredVel = toHomeDir.clone().multiplyScalar(desiredSpeed);
    const desiredFwdSpeed = desiredVel.dot(bodyForwardFlat);
    const desiredRightSpeed = desiredVel.dot(bodyRightFlat);
    stick.pitch = clamp(desiredFwdSpeed / QLOITER_MAX_SPEED, -1, 1);
    stick.roll = clamp(desiredRightSpeed / QLOITER_MAX_SPEED, -1, 1);
    stick.yaw = 0; // heading er fri - overlates til vindkantring (weathervaning), akkurat som i ekte QLOITER

    // Høyde: hold rtlAltM MENS den navigerer mot hjem, men begynn nedstigningen mot landFinalAltM så
    // snart den er nær nok hjem horisontalt (samme "nearHome"-terskel som farten allerede rampes ned
    // innenfor, se slowdownStartM over) - matcher ArduPilot sin egen beskrivelse direkte ("Once arriving
    // within Q_WP_RADIUS_M distance of home, it will begin descending at Q_WP_SPD_DN rate...until it
    // reaches Q_LAND_FINAL_ALT...then Q_LAND_FINAL_SPD").
    // BUG (rapportert av brukeren: "hovrer over homepoint men lander aldri") - en tidligere versjon hadde
    // KUN "synk hvis over rtlAltM+1, klatre hvis under rtlAltM-1, ellers hold" - det fantes ingen logikk
    // som noensinne kommanderte den NED FORBI rtlAltM i utgangspunktet, så den sirklet i ro over hjem på
    // rtlAltM (typisk 15 m) for alltid. nearHome-grenen under er selve fiksen.
    let desiredClimbRate;
    if (rtlState.phase === "land_final") desiredClimbRate = -rtlParams.landFinalSpeedMs;
    else if (nearHome) desiredClimbRate = altitude > rtlParams.landFinalAltM ? -rtlParams.wpSpeedDnMs : -rtlParams.landFinalSpeedMs;
    else if (altitude > rtlParams.rtlAltM + 1) desiredClimbRate = -rtlParams.wpSpeedDnMs;
    else if (altitude < rtlParams.rtlAltM - 1) desiredClimbRate = Math.min(MC_MAX_CLIMB_RATE, rtlParams.rtlAltM - altitude);
    else desiredClimbRate = 0;
    // Sikkerhetsnett (brukeren rapporterte "den bryr seg ikke om RTL-høyde, bare klatrer og klatrer") -
    // uansett hvilken gren over som traff, tillat ALDRI en netto klatrekommando når høyden allerede er PÅ
    // eller OVER rtlAltM. Grenene over SKAL allerede garantere dette hver for seg, men denne klemmen gjør
    // en evt. glipe i den logikken (eller i selve fysikkens momentum/overshoot ved rask klatring inn mot
    // grensen) strukturelt umulig i stedet for kun sannsynlig unngått.
    if (altitude >= rtlParams.rtlAltM && desiredClimbRate > 0) desiredClimbRate = 0;
    stick.throttle = climbRateToStickThrottle(desiredClimbRate);

    // Landingsdeteksjon: en pragmatisk erstatning for ArduPilot sin egen "motor ved minimum i 5s + høyde
    // uendret i 4s"-sjekk (se dokumentasjonen sitert av brukeren) - denne simulatoren har ingen egen
    // motor-PWM å inspisere, så den bruker i stedet den samme observerbare KONSEKVENSEN (stille i ro på
    // bakken i finalfasen) direkte, med en tilsvarende dwell-tid før motoren kuttes automatisk.
    if (rtlState.phase === "land_final" && planeState.onGround) {
        rtlState.landedTimer = (planeState.velocity.length() < RTL_LANDED_SPEED_MS) ? rtlState.landedTimer + dt : 0;
        if (rtlState.landedTimer >= RTL_LANDED_DWELL_S) {
            rtlState.phase = "landed";
            setEngine(false);
        }
    } else {
        rtlState.landedTimer = 0;
    }
    return "qloiter";
}

// Inverterer Alt Hold-formelen fra stepPhysics (se MC_ALT_HOLD_DEADBAND-bruken der) - gitt en ønsket
// klatrerate (m/s, negativ = synk), regn ut hvilken syntetisk stick.throttle (0.5=hold) som får DEN
// UENDREDE alt-hold-koden i stepPhysics til å kommandere nøyaktig den raten.
function climbRateToStickThrottle(desiredClimbMs) {
    const frac = clamp(desiredClimbMs / MC_MAX_CLIMB_RATE, -1, 1);
    if (Math.abs(frac) < 1e-4) return 0.5;
    const magnitude = Math.abs(frac) * (0.5 - MC_ALT_HOLD_DEADBAND) + MC_ALT_HOLD_DEADBAND;
    return 0.5 + Math.sign(frac) * magnitude;
}

const RTL_PHASE_LABELS = {
    idle: "", cruise: "· cruise", vtol_return: "· VTOL-retur", land_final: "· landing", landed: "· landet"
};

/* ---------- HUD (batteri, failsafe-banner, modus-undertekst) ---------- */
const hudBattery = document.getElementById("hudBattery");
const rtlFailsafeBanner = document.getElementById("rtlFailsafeBanner");
// Kalt RETT ETTER simulator-vtol.js sin egen updateHud() i animate() - bygger videre PÅ (ikke erstatter)
// det den allerede satte, slik at simulator-vtol.js sin updateHud() selv forblir totalt uendret.
function updateRtlHud() {
    if (hudBattery) {
        hudBattery.textContent = Math.round(rtlState.batteryPct) + " %";
        hudBattery.className = "sim-status-value" + (rtlState.batteryPct <= rtlParams.batteryFailsafePercent ? " sim-killed" : "");
    }
    if (planeState.flightMode === "qrtl") {
        hudMode.textContent = MODE_LABELS.qrtl + " " + (RTL_PHASE_LABELS[rtlState.phase] || "");
    }
    if (rtlFailsafeBanner) rtlFailsafeBanner.classList.toggle("show", performance.now() < rtlState.failsafeBannerUntil);
}

/* ---------- RTL-panel (samme mønster som initVtolPanel i simulator-vtol.js) ---------- */
function initRtlPanel() {
    const rtlModeSelect = document.getElementById("rtlModeSelect");
    const rtlAltInput = document.getElementById("rtlAltInput"), rtlAltValue = document.getElementById("rtlAltValue");
    const qWpSpeedInput = document.getElementById("qWpSpeedInput"), qWpSpeedValue = document.getElementById("qWpSpeedValue");
    const wpSpeedDnInput = document.getElementById("wpSpeedDnInput"), wpSpeedDnValue = document.getElementById("wpSpeedDnValue");
    const landFinalAltInput = document.getElementById("landFinalAltInput"), landFinalAltValue = document.getElementById("landFinalAltValue");
    const landFinalSpeedInput = document.getElementById("landFinalSpeedInput"), landFinalSpeedValue = document.getElementById("landFinalSpeedValue");
    const transitionRadiusInput = document.getElementById("transitionRadiusInput"), transitionRadiusValue = document.getElementById("transitionRadiusValue");
    const cruiseThrottleInput = document.getElementById("cruiseThrottleInput"), cruiseThrottleValue = document.getElementById("cruiseThrottleValue");
    const pureVtolOnlyInput = document.getElementById("pureVtolOnlyInput");
    const batteryHoverInput = document.getElementById("batteryHoverInput"), batteryHoverValue = document.getElementById("batteryHoverValue");
    const batteryCruiseInput = document.getElementById("batteryCruiseInput"), batteryCruiseValue = document.getElementById("batteryCruiseValue");
    const batteryFailsafeEnabledInput = document.getElementById("batteryFailsafeEnabledInput");
    const batteryFailsafePercentInput = document.getElementById("batteryFailsafePercentInput"), batteryFailsafePercentValue = document.getElementById("batteryFailsafePercentValue");

    function refresh() {
        rtlModeSelect.value = String(rtlParams.rtlMode);
        rtlAltInput.value = rtlParams.rtlAltM; rtlAltValue.textContent = rtlParams.rtlAltM + " m";
        qWpSpeedInput.value = rtlParams.qWpSpeedMs; qWpSpeedValue.textContent = rtlParams.qWpSpeedMs + " m/s";
        wpSpeedDnInput.value = rtlParams.wpSpeedDnMs; wpSpeedDnValue.textContent = rtlParams.wpSpeedDnMs + " m/s";
        landFinalAltInput.value = rtlParams.landFinalAltM; landFinalAltValue.textContent = rtlParams.landFinalAltM + " m";
        landFinalSpeedInput.value = rtlParams.landFinalSpeedMs; landFinalSpeedValue.textContent = rtlParams.landFinalSpeedMs + " m/s";
        transitionRadiusInput.value = rtlParams.transitionRadiusM; transitionRadiusValue.textContent = rtlParams.transitionRadiusM + " m";
        cruiseThrottleInput.value = Math.round(rtlParams.cruiseThrottleFrac * 100); cruiseThrottleValue.textContent = Math.round(rtlParams.cruiseThrottleFrac * 100) + " %";
        pureVtolOnlyInput.checked = rtlParams.pureVtolOnly;
        batteryHoverInput.value = rtlParams.batteryHoverMinutes; batteryHoverValue.textContent = rtlParams.batteryHoverMinutes + " min";
        batteryCruiseInput.value = rtlParams.batteryCruiseMinutes; batteryCruiseValue.textContent = rtlParams.batteryCruiseMinutes + " min";
        batteryFailsafeEnabledInput.checked = rtlParams.batteryFailsafeEnabled;
        batteryFailsafePercentInput.value = rtlParams.batteryFailsafePercent; batteryFailsafePercentValue.textContent = rtlParams.batteryFailsafePercent + " %";
    }
    refresh();

    rtlModeSelect.addEventListener("change", function () { rtlParams.rtlMode = parseInt(rtlModeSelect.value, 10); saveRtlParams(); });
    rtlAltInput.addEventListener("input", function () { rtlParams.rtlAltM = parseFloat(rtlAltInput.value); rtlAltValue.textContent = rtlAltInput.value + " m"; saveRtlParams(); });
    qWpSpeedInput.addEventListener("input", function () { rtlParams.qWpSpeedMs = parseFloat(qWpSpeedInput.value); qWpSpeedValue.textContent = qWpSpeedInput.value + " m/s"; saveRtlParams(); });
    wpSpeedDnInput.addEventListener("input", function () { rtlParams.wpSpeedDnMs = parseFloat(wpSpeedDnInput.value); wpSpeedDnValue.textContent = wpSpeedDnInput.value + " m/s"; saveRtlParams(); });
    landFinalAltInput.addEventListener("input", function () { rtlParams.landFinalAltM = parseFloat(landFinalAltInput.value); landFinalAltValue.textContent = landFinalAltInput.value + " m"; saveRtlParams(); });
    landFinalSpeedInput.addEventListener("input", function () { rtlParams.landFinalSpeedMs = parseFloat(landFinalSpeedInput.value); landFinalSpeedValue.textContent = landFinalSpeedInput.value + " m/s"; saveRtlParams(); });
    transitionRadiusInput.addEventListener("input", function () { rtlParams.transitionRadiusM = parseFloat(transitionRadiusInput.value); transitionRadiusValue.textContent = transitionRadiusInput.value + " m"; saveRtlParams(); });
    cruiseThrottleInput.addEventListener("input", function () { rtlParams.cruiseThrottleFrac = parseFloat(cruiseThrottleInput.value) / 100; cruiseThrottleValue.textContent = cruiseThrottleInput.value + " %"; saveRtlParams(); });
    pureVtolOnlyInput.addEventListener("change", function () { rtlParams.pureVtolOnly = pureVtolOnlyInput.checked; saveRtlParams(); });
    batteryHoverInput.addEventListener("input", function () { rtlParams.batteryHoverMinutes = parseFloat(batteryHoverInput.value); batteryHoverValue.textContent = batteryHoverInput.value + " min"; saveRtlParams(); });
    batteryCruiseInput.addEventListener("input", function () { rtlParams.batteryCruiseMinutes = parseFloat(batteryCruiseInput.value); batteryCruiseValue.textContent = batteryCruiseInput.value + " min"; saveRtlParams(); });
    batteryFailsafeEnabledInput.addEventListener("change", function () { rtlParams.batteryFailsafeEnabled = batteryFailsafeEnabledInput.checked; saveRtlParams(); });
    batteryFailsafePercentInput.addEventListener("input", function () { rtlParams.batteryFailsafePercent = parseFloat(batteryFailsafePercentInput.value); batteryFailsafePercentValue.textContent = batteryFailsafePercentInput.value + " %"; saveRtlParams(); });

    document.getElementById("resetRtlParamsBtn").addEventListener("click", function () {
        Object.assign(rtlParams, DEFAULT_RTL_PARAMS);
        saveRtlParams();
        refresh();
    });
}
