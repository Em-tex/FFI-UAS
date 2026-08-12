/* js/simulator-vtol.js
   VTOL (QuadPlane) simulator - startet som en kopi av js/simulator-fixedwing.js og bygger videre PÅ
   nøyaktig samme AoA/steile-baserte vinge-/hale-aerodynamikk (se den filens toppkommentar for
   begrunnelsen for selve fastvinge-modellen - uendret her). Flyet er en fastvinget kropp med FIRE faste
   (IKKE vippbare) løftemotorer (quad-oppsett, som quadcopter-simulatoren) for vertikal/svevende flyging,
   PLUSS en egen trekkmotor ("pusher") bak V-halen for forover-flyging - en "QuadPlane" i ArduPilot/
   Mission Planner-terminologi (se MODE_LABELS-kommentaren for hele modus-oversikten: QSTABILIZE/QHOVER/
   QLOITER/QACRO for svevemodus, MANUAL/FBWA for fastvinget marsjflyging, QRTL for automatisk hjemflyging
   (se js/simulator-vtol-rtl.js) - AUTO/QLAND er fortsatt bevisst utelatt, se samme kommentar). To
   flyregimer (multirotor/svevende og fastvinget/marsjerende) deler den
   samme skrog-/vinge-/hale-fysikken fra fastvinge-simmen, men har HVER SIN kilde til dreiemoment:
   fastvinge-modellens rorflater (skalerer med dynamisk trykk, ~null ved lav fart) og de fire
   løftemotorene (fast autoritet i Q-moduser, trappet ned/av i FBWA/MANUAL) - se computeMcAuthority i
   stepPhysics for selve overgangslogikken (ArduPilot sine Q_*-parametre, se DEFAULT_VTOL_PARAMS), og
   computeWeathervaneYawRateRad for vindkantrings-kompensasjonen i QLOITER (se ArduPilot sin "Active
   Weathervaning" - flyet dreier nesen inn mot vinden mens det holder posisjon). */

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
// Nesekjeglens lengde/tupp-radius (andel av hhv. fuselageLength/cabinRadius) - DELT mellom buildPlane og
// resolveGroundContact (se NOSE_TIP-bruken der), samme "aldri la disse drifte ut av synk"-prinsipp som
// resten av denne konstant-gruppen (se toppkommentaren for hele filen).
const NOSE_LEN_RATIO = 0.18;
const NOSE_TIP_RADIUS_RATIO = 0.35;
// Vingens monteringshøyde over CG (andel av cabinRadius) - DELT mellom buildPlane og bakkeeffekt-
// beregningen i stepPhysics, siden bakkeeffekt virker fra VINGEN, ikke fra CG/planeState.position.y
// direkte. Senket fra 1.3 (brukeren: "vingen må sitte fast i flykroppen" - en tydelig høyvinge-plassering
// etterlot et synlig gap opp til skroget, se buildPlane-kommentaren ved wingMountY for hele
// bug-historikken/den forkastede fairing-klossen). 1.05 holder vingeroten rett på/en anelse embedded i
// selve sylinderoverflaten (cabinRadius=1.0) for alle tre VTOL_CLASSES sine faktiske vingetykkelser (se
// WING_THICKNESS_RATIO-bruken i buildWing) i stedet for å sveve over den.
const WING_MOUNT_HEIGHT_RATIO = 1.05;
// Understellet er FIRE BEN som henger ned fra løftemotor-bommene (IKKE hjul, se referansebildene
// brukeren la ved) - DELT mellom buildPlane (visuell mesh, se addLiftBoom-kommentaren der) og
// resolveGroundContact (fysikkens bakkekontaktpunkter), slik at de aldri kan drifte ut av synk (samme
// prinsipp som FUSELAGE_LENGTH_BUILD-gruppen over). GEAR_BOOM_X_FRAC (andel av vingespennet) MÅ holde
// seg innenfor vingens faste midtseksjon (under 0.28 - se aileronSpan/centerSpan i buildWing), ellers
// stikker bom/ben rett gjennom balanserorets utsparing på vingetuppen.
const GEAR_BOOM_X_FRAC = 0.22;
const BOOM_CENTER_Z_BUILD = 0.02; // samme Z som vingefestet i buildPlane (bygge-rom)
// Selve BENA henger RETT NED fra bom-posisjonen (GEAR_BOOM_X_FRAC) i X - altså rett under vingen, ikke
// vinklet sideveis innover (brukeren rapporterte at et sideveis-skrått ben "så feil ut" / ikke "under
// vingen"). Eneste vinkling er en LITEN forover-lene (-Z, se GEAR_LEG_FORWARD_LEAN_BUILD) ned mot
// bakkeplanet, jf. referansebildene.
const GEAR_LEG_FORWARD_LEAN_BUILD = 0.05; // bygge-rom - hvor mye benas bunn er forskjøvet forover (-Z)
// Benas TOPP-feste langs selve bommen (Z) er IKKE rett under fremre motor (m.z) - brukeren påpekte at
// det så feil ut ("landingsbena foran skal ikke være under motorene foran"). I stedet interpoleres
// festepunktet et stykke TILBAKE langs bommen, mot bom-/vingefestets senter (BOOM_CENTER_Z_BUILD, se
// merknaden over - "litt lengre bak langs det staget. mer under vingene"). 0 = rett under fremre motor,
// 1 = rett i bom-/vingefestets senter.
const GEAR_LEG_BOOM_Z_FRAC = 0.55;

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
// VINGENS eget sideveis-drag i sideslip/kryssvind (se wingCrossflowDrag i stepPhysics) - brukeren påpekte
// at en stor, flat flate (HELE vingen, ikke bare skroget) legger seg opp mot vinden i sidevind, spesielt
// tydelig i QLOITER-hover der nesen ikke nødvendigvis peker inn mot vinden: vingens EGET plan-areal
// (spec.wingArea, ALLEREDE hele arealet - ingen ratio-konstant trengs slik skroget sin egen, mindre
// sideprofil trenger) møter da vinden nesten flatt-på, akkurat som skroget sin FUSELAGE_SIDE_CD over
// (samme størrelsesorden Cd for et butt/flatt legeme på tvers av strømmen - vingen er tynn, men BRED).
// UAVHENGIG av vingens EGEN løft-/AoA-modell (liftCoefficient/aoaDeg), som kun ser vertikal+forover-fart
// (localAirVel.y/.z) og dermed er blind for ren LATERAL (sideveis) luftstrøm.
const WING_CROSSFLOW_CD = 1.15;
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
// Engangs energitap i selve krasjøyeblikket - se triggerCrash()-kommentaren for hele bakgrunnen
// (brukeren: "vingene blir som et hjul. ruller og ruller... knekke av vingen som tar hardt nedi og
// bremse flyet"). Andelen av fart/vinkelfart som BEHOLDES rett etter selve sammenstøtet.
// JUSTERT NED (var 0.15 = 85% borte MOMENTANT) - brukeren rapporterte at flyet "stopper opp veldig bratt
// når det krasjer i høy hastighet med vingen. virker ikke realistisk" - et ekte, hardt sammenstøt taper
// ganske mye bevegelsesenergi momentant til selve strukturbristen (derfor fortsatt et reelt, øyeblikkelig
// tap her, ikke 0), men IKKE 85% av det i ett eneste fysikk-tick - resten av oppbremsingen skal skje
// GRADVIS, over en reell utrullings-/sklidistanse, akkurat som en ekte krasjlanding. Selve den fortsatte
// oppbremsingen kommer nå fra CRASH_FRICTION_MULTIPLIER (se konstanten/bruken i resolveGroundContact) i
// stedet for et nytt, kunstig engangskutt - fysisk begrunnet (et skadet, gnagende/gravende skrog har MYE
// høyere friksjon enn et intakt understell), kontinuerlig over tid/distanse i stedet for øyeblikkelig, og
// praktisk talt gratis ytelsesmessig (samme per-punkt-friksjonsberegning som allerede kjører hver tick,
// kun én ekstra multiplikasjon - ingen ny simulering).
const CRASH_ENERGY_LOSS_FRAC = 0.6;
// Se CRASH_ENERGY_LOSS_FRAC-kommentaren over - selve den KONTINUERLIGE, realistiske utrullings-
// oppbremsingen etter selve sammenstøtet. Et skadet skrog som sklir/tumler (bøyd metall, avrevne
// deler, ingen lenger glatte/runde landingsben) graver seg ned i underlaget og har derfor MYE høyere
// effektiv friksjon enn et intakt, uskadd fly - modellert som en rein multiplikator på den allerede
// eksisterende per-punkt Coulomb-friksjonen (se bruken i resolveGroundContact), IKKE en egen,
// separat mekanisme. 4x er en grov, ikke live-testet tilnærming - juster om utrullingen fortsatt føles
// for lang/kort.
const CRASH_FRICTION_MULTIPLIER = 4;

const MAX_BANK_ANGLE = 50;      // grader, FBWA (fastvinget marsjfart): pinne-utslag -> ønsket krengevinkel
// Økt videre (10 -> 12 -> 15) - brukeren rapporterte at tailstrike-geometrien (halelengden), ikke
// steilevinkelen, er den reelle begrensningen på rotasjon ved avgang, så "godt under laveste
// steilevinkel"-marginen (opprinnelig hensikt her) er ikke lenger den bindende bekymringen. Ved lav
// fart vil steiling naturlig gi redusert løft/nese-fall lenge før Stabilized når helt fram til et
// kommandert 15°-mål uansett (P-loopen bruker tid på å nå målet), så risikoen holdes akseptabel.
const MAX_PITCH_ANGLE = 15;     // grader, FBWA (fastvinget marsjfart): pinne-utslag -> ønsket stigningsvinkel
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

/* ---------- FBWB (Fly By Wire B) ----------
   Ekte ArduPilot (sitert direkte av brukeren): "similar to FLY BY WIRE A (FBWA), but Plane will try to
   hold altitude as well. Roll control is the same as FBWA, and altitude is controlled using the
   elevator. The target airspeed is controlled using the throttle... the elevator stick does not directly
   control pitch, it controls target climb or descent rate." Gjenbruker den SAMME selvnivellerende
   stigningsvinkel-P(D)-kontrolloven FBWA/Q-modusene allerede har (STABILIZED_PITCH_AUTHORITY_DEG/-D_GAIN
   over) som "indre løkke" - denne filens forenkling av ekte TECS er kun en YTRE klatrerate-P-regulator
   (se stepPhysics) som regner ut MÅLVINKELEN (targetPitchDeg) den indre løkken allerede vet hvordan den
   skal nå, i stedet for en helt separat kontrollstruktur. Rull er UENDRET fra FBWA (samme
   targetBankDeg-linje, se stepPhysics), og sideroret er (som i FBWA) rent pinnestyrt med naturlig
   aerodynamisk svingkoordinering. */
const FBWB_STICK_DEADBAND = 0.05;  // liten dødsone på selve elevator-pinnen (-1..1) rundt senter
// Q_FBWB_CLIMB_RATE-ekvivalent - ArduPilot sin egen standardverdi er 2 m/s, men dokumentasjonen brukeren
// limte inn påpeker selv at "many users will want to raise FBWB_CLIMB_RATE to a higher value to make the
// altitude change more responsive" - satt høyere her fra start (samme resonnement som
// MC_MAX_CLIMB_RATE/-SINK_RATE: treg respons oppleves som en svakhet i en treningssimulator, ikke realisme).
// ØKT VIDERE fra 4 (brukeren, etter forrige runde: "FBWB virker som fortsatt har alt for lav pitch
// autoritet") - ROTEN var IKKE selve P/D-loopens forsterkning (den mettet allerede momentant til fullt
// utslag på et hvilket som helst merkbart pinneutslag), men at 4 m/s er en så BESKJEDEN kommandert rate
// at flyet raskt (et par sekunder) NÅDDE den, og targetPitchDeg falt derfor raskt tilbake fra fullt
// utslag til bare noen få grader - akkurat nok til å HOLDE 4 m/s, ikke mer. Pinnen "ga seg" dermed
// merkbart etter et par sekunder, selv om den holdt fullt utslag - stikk i strid med FBWA sin egen
// direkte, VEDVARENDE fulle vinkelkommando (som ikke har noen slik "nådd målet"-avslapning). Med en MYE
// høyere kommandert rate (10, godt over hva denne klassens faktiske stigeevne trolig strekker til - se
// "runde 4"-motorøkningen ved VTOL_CLASSES) FORBLIR climbError (og dermed targetPitchDeg) mettet på
// full autoritet så lenge flyet fortsatt henger etter den urealistisk høye kommanderte raten - som i
// praksis er hele tiden ved fullt pinneutslag, akkurat den vedvarende, "alltid dra i"-følelsen FBWA gir.
const FBWB_CLIMB_RATE = 10;
// Grader mål-stigningsvinkel per (m/s) avvik mellom kommandert og faktisk klatrerate - samme "avvik ->
// vinkel"-idé som QLOITER_VEL_TO_LEAN_DEG (fart->krengevinkel), bare for klatrerate->stigningsvinkel.
// Kalibrert til å nå MAX_PITCH_ANGLE/MC_MAX_LEAN_ANGLE-grensen (se fbwbMaxPitchDeg i stepPhysics) godt før
// hele FBWB_CLIMB_RATE-avviket er brukt opp, slik at pinnen fortsatt føles responsiv fra midtstilling.
// Økt fra 3 (brukertilbakemelding: "veldig liten autoritet på høyderoret/pitch") - 3 ga kun ±3° mål-vinkel
// per m/s avvik, altfor svakt til å merkbart svare på en vanlig, moderat synk/klatre-kommando.
const FBWB_PITCH_GAIN_PER_MS = 7;
// I-ledd (rent P-ledd, se over, har PER DEFINISJON en vedvarende steady-state-feil under en KONSTANT
// forstyrrelse - brukertilbakemelding: "den mister gradvis høyde i cruise", nøyaktig det symptomet en
// manglende integrator gir). Akkumulerer climbError over tid til et eget, separat gradbidrag - ekte
// TECS/altitude-hold bruker akkurat denne PI-strukturen (proporsjonal for rask respons, integral for å
// presse selve den VEDVARENDE feilen mot null over tid), ikke bare et rent P-ledd. Klemt til
// FBWB_PITCH_I_MAX_DEG for å unngå "integral windup" (et vedvarende, urealistisk stort opphopet bidrag)
// og nullstilt hver gang FBWB forlates (se fbwbClimbIntegral-resetten i stepPhysics).
const FBWB_PITCH_I_GAIN_PER_MS = 1.2;
const FBWB_PITCH_I_MAX_DEG = 8;
// Q_AIRSPEED_MIN/-MAX-ekvivalent - denne simulatoren har ingen egen luftfartssensor-av/på-bryter (se
// ArduPilot-dokumentasjonens "with an airspeed sensor"-gren, som er den mest nyttige å modellere for en
// treningssimulator - piloten skal lære å styre FART med gasspaken, ikke bare et rått pådrag-tall). Delt
// likt mellom alle tre VTOL_CLASSES (samme "delt kontrolleffektivitet, kun inertia skiller klassene"-
// filosofi som resten av filen, se VTOL_CLASSES-toppkommentaren) - godt innenfor propPitchSpeed-taket
// (40-45 m/s) for alle tre klasser.
const FBWB_MIN_AIRSPEED = 14;
const FBWB_MAX_AIRSPEED = 28;
// Fartsregulatorens P-forsterkning (gasspak-fraksjon per m/s fartsavvik) og "marsj-trim" (baseline-gass
// feilen justerer RUNDT) - grov, men fungerende autothrottle uten en egen integrator-/TECS-modell.
const FBWB_SPEED_GAIN = 0.05;
const FBWB_THROTTLE_TRIM = 0.6;

/* ---------- VTOL: multirotor-kontroll (løftemotorer) og weathervane ---------- */
// Maks kommandert krengning/stigning i Q-modus (QSTABILIZE/QHOVER/QLOITER, MED multirotor-myndighet) -
// vesentlig lavere enn fastvinge-FBWA sin egen MAX_BANK_ANGLE/MAX_PITCH_ANGLE (50°/15° under, ment for
// marsjflyging). targetBankDeg/-PitchDeg i stepPhysics blander lineært mellom denne og fastvinge-
// vinklene etter mcAuthority (se computeMcAuthority), slik at selve MÅLVINKELEN glir naturlig fra
// "hover-aktig" til "fly-aktig" akkurat som selve kontrollmyndigheten gjør.
const MC_MAX_LEAN_ANGLE = 30;
// Gulv for girets EGEN aerodynamiske autoritet i Q-modus (se yawAeroAuthority-kommentaren i stepPhysics) -
// lar finnen/halepartiet fortsatt værhane merkbart inn mot en kryssvind selv med løftemotorene i full
// rull-/stigningskontroll, i motsetning til rull/stigning som fortsatt kuttes helt (aeroAuthority).
const YAW_AERO_MIN_AUTHORITY_QMODE = 0.5;
// Tak (IKKE gulv - motsatt av yaw-konstanten over) for rull/stigning sin EGEN aerodynamiske autoritet i en
// Q-modus, se aeroAuthority-bruken i stepPhysics. BUG (rapportert av brukeren, med flightlogg fra QHOVER
// ved 15-22 m/s forover-fart, full pinneP: "wobbler noe voldsomt i roll") - da aeroAuthority ble gjort
// uavhengig av mcAuthority (se BUG-kommentaren ved selve aeroAuthority-linjen, den forrige fiksen for
// "urealistisk lav roll autoritet"), fikk en Q-modus ved høy fart plutselig BEGGE momentkildene i FULL
// styrke SAMTIDIG - mcAuthority er jo per definisjon ALLTID 1 i en Q-modus (uansett fart), og
// aeroAuthority kunne nå OGSÅ nå helt opp til 1 ved nok forover-fart - to UAVHENGIGE, hver for seg fullt
// autoritative kontrollere som begge reagerer på nøyaktig samme rollDeflection-kommando, summert rått,
// er reelt sett en DOBLET total forsterkning i forhold til det verken systemet alene er tunet/dempet for
// - en klassisk "to regulatorer på samme feil"-ustabilitet, som viser seg som nettopp en voldsom,
// dårlig dempet svingning. I FBWA/FBWB oppstår ALDRI dette, siden mcAuthority der FAKTISK går mot 0 idet
// aeroAuthority stiger (ekte, glidende overgang, ikke to samtidige fulle kilder). Denne konstanten
// begrenser derfor HVOR MYE aeroAuthority får legge OVENPÅ den allerede garantert fulle quad-autoriteten
// i en Q-modus spesifikt - fortsatt en STOR forbedring fra den forrige, harde 0%-grensen (dette var selve
// poenget med fiksen), men uten å la den totale kombinerte autoriteten løpe løpsk ved høy fart.
const Q_MODE_AERO_AUTHORITY_CEILING = 0.4;
// mcXxxTorqueGain/-DampGain: samme "torque = kommando*autoritet - demping*rate"-struktur som
// fastvinge-rorflatenes egne ROLL_CONTROL_EFFECTIVENESS/-DAMPING (se lenger ned), men UAVHENGIG av
// dynamisk trykk - løftemotorenes dreiemoment kommer fra differensial-TREKKRAFT, ikke luftstrøm mot en
// rorflate, og virker derfor like godt ved null fart som i marsjfart (dette ER selve grunnen til at en
// VTOL trenger dem - fastvinge-rorflatene alene har ~ingen myndighet ved lav/null luftfart siden
// qDynControl går mot null der). Skalert med spec.inertiaRoll/-Pitch/-Yaw i stepPhysics akkurat som
// fastvinge-leddene - selve GAIN-tallet er dermed en ren vinkelakselerasjon (rad/s²), uavhengig av
// flyklasse (kun responsHASTIGHET, ikke -styrke, skal variere med inertia - samme designfilosofi som
// resten av filen).
const MC_ROLL_TORQUE_GAIN = 14;
const MC_ROLL_DAMP_GAIN = 5;
const MC_PITCH_TORQUE_GAIN = 14;
const MC_PITCH_DAMP_GAIN = 5;
// Yaw i Q-modus er en RATE-kommando (som QACRO/quad-simulatorens egen Acro), ikke en vinkel-P(D) slik
// rull/stigning er over - en multirotor har ingen naturlig "nullpunkt" å nivellere nesen mot i seg selv,
// kun en ønsket dreiehastighet fra spak (pluss et eventuelt weathervane-bidrag i QLOITER, se
// computeWeathervaneYawRateRad - de to legges sammen til én mål-girrate før samme rate-feil-struktur).
const MC_MAX_YAW_RATE_DEG = 90;
const MC_YAW_RATE_GAIN = 6;
// Q_LOIT_SPEED_MS: maks horisontal fart (m/s) i QLOITER - spaken kommanderer horisontal FART (kropp-
// relativt forover/sideveis) i stedet for en vinkel direkte, se stepPhysics. 5 m/s er ArduPilot sin
// egen standardverdi (sitert av brukeren).
const QLOITER_MAX_SPEED = 5;
// Hvor mange grader kommandert lenevinkel 1 m/s fartsavvik oversettes til i QLOITER (se stepPhysics) -
// klemmes uansett til MC_MAX_LEAN_ANGLE (tilsvarer Q_LOIT_ANG_MAX). Økt fra 3.5 til 6 - ved 3.5 nådde et
// fullt pinneutslag fra stillstand (QLOITER_MAX_SPEED*3.5 = 5*3.5 = 17.5°) ALDRI det faktiske
// MC_MAX_LEAN_ANGLE-taket (30°) - flyet kunne rett og slett ikke tilte mer enn 17.5° uansett hvor hardt
// piloten dyttet pinnen, og den vinkelen krympet raskt videre mot 0° idet farten nærmet seg 5 m/s (en
// ren P-kontroller mot mål-FART, ikke mål-vinkel - se targetPitchDeg/targetBankDeg i stepPhysics).
// Brukeren rapporterte nettopp dette ("nesten umulig å tilte noe særlig fremover"). 6 gir nøyaktig 30°
// (=taket) ved fullt utslag fra stillstand, uten å endre selve QLOITER_MAX_SPEED (fortsatt ArduPilot sin
// egen 5 m/s-standardverdi, kun selve OVERSETTELSEN fart->vinkel er gjort mer aggressiv/responsiv).
const QLOITER_VEL_TO_LEAN_DEG = 6;
// QHOVER/QLOITER/FBWA(assistert) sin Alt Hold - se stepPhysics. Samme "sentrert gasspak holder høyde,
// avvik gir ønsket klatrerate"-prinsipp som ArduPilot (og quad-simulatorens egen Alt Hold-modus, se
// der), men per-kilo (ikke et flatt Newton-tall) siden VTOL_CLASSES spenner et mye bredere masseområde.
// Dødsonen matcher ArduPilot sin egen RCn_DZ-standardverdi (±6%, sitert av brukeren).
const MC_ALT_HOLD_DEADBAND = 0.06;
const MC_MAX_CLIMB_RATE = 3.5;      // m/s ved fullt gassutslag OPPOVER fra midten (Q_PILOT_SPD_UP)
// Q_PILOT_SPD_DN - bevisst STØRRE enn klatretaket over (brukertilbakemelding: "jeg tror en ekte VTOL
// tillater lavere motorsetting når throttle er i idle, altså større synkehastighet" - en ekte multirotor
// faller raskere enn den klatrer, siden tyngdekraften selv gjør mye av jobben nedover, mens klatring
// krever at motorene alene overvinner både vekt OG ønsket akselerasjon oppover). Kun brukt i QHOVER/
// QLOITER sin Alt Hold-kollektiv-gren (se climbRateCommand der) - FBWA sin egen pitch-styrte klatre-/
// synkerate (climbInputFromPitch) er UENDRET symmetrisk, siden brukerens tilbakemelding gjaldt eksplisitt
// hover/loiter-modus, ikke FBWA-overgangen.
const MC_MAX_SINK_RATE = 6.5;
const MC_ALT_GAIN_PER_KG = 6;       // N per kg per (m/s) avvik i klatrerate (Q_P_POSZ_P-lignende)

// Denne VTOL-en har INGEN hjul (se GEAR_BOOM_X_FRAC-kommentaren og referansebildene brukeren la ved) -
// tre faste ben/en kufert mot bakken, ikke rullende hjul. Et fast ben har mye høyere friksjon enn et
// hjul (som er designet nettopp for å rulle med lite motstand) - en typisk μ for metall/kompositt mot
// asfalt/gress, IKKE en hjul-rullemotstandsverdi. Brukt som ren Coulomb-friksjon (konstant
// retardasjon inntil stopp, se GROUND_SKID_FRICTION_COEFF-bruken i resolveGroundContact) i stedet for
// det gamle hjul-rullemotstand+eget-bremse-konseptet - et fast ben har ingen egen bremse, kun friksjon.
// Økt fra 0.4 til 0.55 (brukeren understreket "kraftig friksjon" flere ganger) - nærmere en reell,
// grov/urban μ for metall eller kompositt mot asfalt/betong (0.5-0.6) enn et glatt/polert underlag.
const GROUND_SKID_FRICTION_COEFF = 0.55;
const GROUND_YAW_FRICTION = 3;  // eksponentiell demping (1/s) av gir-rotasjon fra bena mot bakken
// Statisk (Coulomb-lignende) motstand mot vindkantring mens flyet står/ruller på bakken - se
// yawTorqueF0 i stepPhysics. Skalert med vekt (mass*GRAVITY), som normalkraften på bena i
// virkeligheten - et tyngre fly har mer grep og er dermed mer motstandsdyktig mot vindkantring.
const GROUND_YAW_FRICTION_TORQUE_COEFF = 0.15;
// Samme statiske-friksjon-prinsipp som over, men for LATERAL (sideveis) kraft i stedet for gir-moment -
// resolveGroundContact sin egen lateralSpeed-demping er kun RATE-basert (bremser en eksisterende
// sidebevegelse) og motstår aldri en VEDVARENDE sidekraft (f.eks. vind-drag i kryssvind), som ellers gir
// en nullforskjellig likevektsfart der flyet sakte men uendelig driver sidelengs. Kansellerer små/
// moderate sidekrefter helt, reduserer bare (ikke fjerner) sterkere kast. Samme økning (0.4->0.55) og
// begrunnelse som GROUND_SKID_FRICTION_COEFF over.
const GROUND_LATERAL_FRICTION_COEFF = 0.55;
// (Tidligere fantes en GROUND_ROLL_PITCH_RESISTANCE_COEFF her - et forsøk på å klippe SUMMEN av
// aerodynamisk+løftemotor rull-/stigningsmoment mens flyet stod på bena, for å hindre at det "lett tiltet
// rundt". Fjernet igjen: den klippet OGSÅ selve den korrigerende selvnivellerings-torque'en, som lot små
// avvik bygge seg helt ukorrigert opp mens flyet stod på bakken - de "løste seg ut" i et brått,
// ukontrollert tipp idet flyet endelig løftet fra bakken. Se targetBankDeg/targetPitchDeg-kommentaren i
// stepPhysics for den riktige fiksen: MÅLET tvinges til null mens flyet er på bakken i en Q-modus, slik
// at selvnivelleringen har FULL, ukuttet autoritet til å faktisk holde flyet flatt hele bakke-fasen.)
const GROUND_CLEARANCE_FW = 0.05;
const CRASH_SINK_RATE = 6;      // m/s synkefart ved berøring som teller som hard landing
const CRASH_BANK_DEG = 45;      // krengevinkel ved berøring som teller som hard landing
// Stigningsvinkel (uansett fortegn - nese ned ELLER nese opp) ved berøring som teller som hard landing -
// samme prinsipp/terskel-verdi som CRASH_BANK_DEG over, se resolveGroundContact. Lagt til sammen med
// nesetupp-kontaktpunktet (se GROUND_CONTACT_POINT_COUNT-kommentaren) - UTEN denne kunne en nesetung
// berøring (brukerens skjermbilde: nesa ~45° ned i bakken) telle som en helt vanlig, ukrasjet landing bare
// fordi selve SYNKEFARTEN var lav, akkurat like farlig/urealistisk som en hard skjev krengning ville vært.
const CRASH_PITCH_DEG = 45;
// Vingetupp-krasj (rapportert av brukeren, med skjermbilde: "det er mulig å fly full gass i FBWA
// skrapende langs bakken... med vinge og fot nedi bakken. det skulle i virkeligheten ha ført til kraftig
// friksjon, blitt kastet rundt og krasjet") - CRASH_BANK_DEG alene fanger IKKE dette scenariet: hvor
// BRATT krengevinkel som faktisk får en vingetupp til å treffe bakken avhenger av GEOMETRI (vingespenn
// og bakkeklaring), ikke bare attityde - et fly som allerede flyr lavt kan dra en vingetupp langs bakken
// ved en krengevinkel langt under 45°, og dermed aldri krysse selve bank-terskelen i det hele tatt, mens
// det fortsatt flyr videre "normalt". En EKTE vingetupp i bakken ved reell flygefart er derimot praktisk
// talt ALLTID katastrofalt i virkeligheten (voldsom, asymmetrisk friksjon kaster flyet momentant rundt) -
// sjekkes derfor DIREKTE i resolveGroundContact ut fra selve KONTAKTEN (nok penetrasjon PÅ EN VINGETUPP,
// samtidig som flyet fortsatt har reell fart), uavhengig av hvilken krengevinkel som tilfeldigvis gjaldt
// akkurat da. PEN-terskelen er satt godt over den mikroskopiske likevekts-gjeninntrengningen en normal,
// flat landing gir (se frictionNormalForceMag-kommentaren: typisk brøkdeler av en millimeter) - kun en
// vingetupp som faktisk graver seg tydelig ned skal telle.
const WING_STRIKE_CRASH_PEN_M = 0.05;
const WING_STRIKE_CRASH_SPEED_MS = 3;
// Terskel for "sett seg helt til ro"-smellen i den krasjede grenen av stepPhysics - se BUG-kommentaren
// der ("sklir sakte unaturlig etter krasj... burde ha stoppet opp raskere").
const CRASH_SETTLE_SPEED_MS = 0.3;
// Robust, direkte eksponentiell drag på CG-fart OG vinkelfart mens et krasjet fly er i bakkekontakt - se
// BUG-kommentarene ved bruken i stepPhysics ("sklir med vingen nedi bakken. stopper aldri å skli" og
// oppfølgingen "krasjer med nesa først. så ruller flyet unaturlig mye fremover også").
// JUSTERT NED (var 3/3, under et kvart sekunds halveringstid) - brukeren rapporterte tilbake at DET i
// stedet var for BRÅTT ("må jo ha en mellomting. nå bare stopper flyet opp med en gang. og blir stående i
// en unaturlig stilling") - en for rask, ubetinget vinkelfart-drag kvelte den EKTE, fysiske
// veltemomentet (fra nesetupp-/vingetupp-/bein-fjærkraften, se resolveGroundContact) FØR det noensinne
// rakk å fullføre en naturlig velting ned mot en flat, stabil hvilestilling - flyet frøs i stedet fast
// midt i selve velte-bevegelsen. Lavere rate her gir en lengre, mer synlig (men fortsatt garantert
// begrenset) utrullings-/veltefase - noen sekunder, ikke under ett - som gir tyngdekraften/
// kontaktmomentet nok tid til faktisk å fullføre en troverdig velting, samtidig som flyet fortsatt
// garantert stanser innen rimelig tid i stedet for å rulle for alltid (det opprinnelige problemet).
// Vinkelfart-raten er satt LAVERE enn selve fart-raten spesifikt for å prioritere at VELTINGEN får tid
// til å fullføres.
const CRASH_DRAG_RATE_PER_SEC = 0.8;
const CRASH_GROUND_ANGULAR_DRAG_PER_SEC = 0.5;

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

/* ---------- VTOL-klasser ---------- */
// Samme prinsipp som fastvinge-simmens PLANE_CLASSES (delt kontrolleffektivitet/-demping, kun
// inertia/masse skiller klassene i respons-TREGHET, ikke -STYRKE) - se den filens tilsvarende
// kommentar. Tyngre enn en ren glider av samme vingeareal (motorarmer + fire løftemotorer + ekstra
// batterikapasitet for hover) - massen er derfor bevisst økt (~+55%) i forhold til de opprinnelige
// PLANE_CLASSES-tallene den er avledet fra, IKKE bare kopiert rett over. pusherMaxThrust erstatter
// den gamle nesepropellens maxThrust (samme rolle, se thrustForce i stepPhysics - kun trekkraft for
// FORWARD flight, ikke løft). Satt LIK den tilsvarende klassens maxThrust i fastvinge-simmen (samme
// trekkraft/vekt-forhold) - et tidligere forsøk på å gi pusheren LAVERE trekkraft enn en ren fastvinge
// (begrunnet med at den "kun trenger å akselerere INN i marsjfart etter en vertikal takeoff, ikke selv
// dra flyet opp fra stillstand på hjul") viste seg fortsatt "undermotorisert" i praksis - en VTOL i
// FBWA/MANUAL må jo fortsatt klatre/akselerere/holde fart mot medvind akkurat som en ren fastvinge gjør,
// uavhengig av HVORDAN den kom seg opp i luften i utgangspunktet.
// liftThrustTotal er den SAMLEDE (alle fire motorer) maks vertikale trekkraften - satt til en typisk
// multirotor-thrust/vekt-margin (~1.8-2.0x egenvekt i Newton) slik at flyet kan svelve OG fortsatt
// klatre/manøvrere med margin til overs, akkurat som en ekte quadcopter.
// pusherMaxThrust/propPitchSpeed økt IGJEN her (runde 3 - se "motor boost"-git-historikken for runde 2,
// som satte pusherMaxThrust LIK fastvinge-simmens egen tilsvarende klasse) - brukeren rapporterte at selv
// FULL gass på Liten bare så vidt kom over 20 m/s i level fastvinget flukt. Regnet etter: likevektsfarten
// (thrust(V)=drag(V)) med de gamle tallene (18N/30 m/s pitch-fart) løser til ~21 m/s for Liten - stemmer
// nøyaktig med rapporten, altså IKKE en feil, bare reell undertuning. Grunnen til at ren
// "match fastvinge-simmens tall"-runde 2 ikke var nok: en ekte QuadPlane har MER motstand i fastvinget
// flukt enn en ren fastvinge med samme skrog/vinge, fra de fire faste løftemotorene/bommene som henger i
// luftstrømmen (IKKE modellert som egen drag-økning her - cd0 er uendret fra fastvinge-simmens egne tall)
// - samme trekkraft holder da ikke til samme fart. Løftet derfor BÅDE trekkraften (mer margin over
// motstanden ved marsjfart) OG propPitchSpeed (flatere trekkraft-vs-fart-kurve, se thrustForce i
// stepPhysics - kraften faller null idet farten når propPitchSpeed) for alle tre klasser, proporsjonalt
// skalert (samme ~1.67x-faktor på trekkraft, ~1.33x på pitch-fart, gjennom alle tre - konsistent med
// hvordan runde 2 også skalerte klassene sammen).
// RUNDE 4 (brukeren, denne gangen om KLATRING spesifikt: "pusher propellen må ha sterke motor for
// klatring. virker litt undermotorisert") - runde 3 løste toppfart i NIVÅFLUKT, men ga fortsatt liten
// margin til overs for faktisk å KLATRE i FBWA/FBWB (der nettopp differansen mellom trekkraft og
// marsjfart-drag er det som setter klatreraten - se targetPitchDeg/FBWB_PITCH_GAIN_PER_MS-loopen i
// stepPhysics, som uansett trenger ekte overskuddskraft å konvertere til stigning). Liten sin
// statiske trekkraft/vekt var fortsatt under 1.0 (30N/33.3N) - et fastvinget fly klarer seg fint med det
// (vingen bærer vekten, ikke motoren), men gir lite margin igjen til klatring etter at marsjfart-draget
// er trukket fra. Løftet igjen, samme proporsjonale ~1.33x/~1.15x-mønster som runde 3.
const VTOL_CLASSES = {
    small: {
        label: "Liten (trener-VTOL)",
        mass: 3.4, wingArea: 0.4, wingSpan: 1.9,
        pusherMaxThrust: 40, cd0: 0.05, inducedDragK: 0.95, clSlope: 0.11, stallAngleDeg: 14, propPitchSpeed: 46,
        liftThrustTotal: 3.4 * GRAVITY * 1.8,
        inertiaRoll: 0.16, inertiaPitch: 0.5, inertiaYaw: 0.5,
        gearOffsetY: -0.22, visualScale: 1.0, armLen: 0.62
    },
    medium: {
        label: "Middels",
        mass: 12, wingArea: 0.68, wingSpan: 2.5,
        pusherMaxThrust: 80, cd0: 0.045, inducedDragK: 1.05, clSlope: 0.105, stallAngleDeg: 13, propPitchSpeed: 49,
        liftThrustTotal: 12 * GRAVITY * 2.0,
        inertiaRoll: 1.1, inertiaPitch: 1.6, inertiaYaw: 1.8,
        gearOffsetY: -0.28, visualScale: 1.4, armLen: 0.85
    },
    large: {
        label: "Stor",
        mass: 32, wingArea: 1.25, wingSpan: 3.5,
        pusherMaxThrust: 175, cd0: 0.04, inducedDragK: 1.15, clSlope: 0.1, stallAngleDeg: 12, propPitchSpeed: 52,
        liftThrustTotal: 32 * GRAVITY * 1.9,
        inertiaRoll: 3.2, inertiaPitch: 4.4, inertiaYaw: 4.8,
        gearOffsetY: -0.35, visualScale: 2.0, armLen: 1.15
    }
};
const DEFAULT_PLANE_CLASS = "small";

// Flightmode-navn og -sett følger ArduPilot/Mission Planner sin QuadPlane-terminologi direkte (brukeren
// ba spesifikt om dette) i stedet for et eget oppfunnet navnesett. QSTABILIZE/QHOVER/QLOITER/QACRO er
// Q-moduser (svevemodus, alltid full myndighet på løftemotorene, se computeMcAuthority) - MANUAL/FBWA/
// FBWB er "Plane"-moduser (fastvinget marsjflyging, kun FBWA/FBWB får assistanse av løftemotorene ved lav
// fart, se samme funksjon). Det finnes IKKE en egen "transisjonsbryter" som i PX4 - selve MODUSVALGET er
// transisjonen (bytt til en Q-modus for umiddelbar svevemyndighet, bytt til MANUAL/FBWA/FBWB for å fly
// videre som fastvinget), akkurat som i ekte ArduPilot. AUTO/QLAND (oppdragsbaserte moduser) er fortsatt
// bevisst UTELATT - denne simmen har ingen oppdrags-/rutepunkt-system. QRTL er lagt til som ETT eget
// unntak (se js/simulator-vtol-rtl.js): den trenger ikke et generelt rutepunktsystem, kun ett enkelt fast
// mål (hjem).
const MODE_LABELS = {
    qstabilize: "QSTABILIZE", qhover: "QHOVER", qloiter: "QLOITER", qacro: "QACRO",
    manual: "MANUAL", fbwa: "FBWA",
    // FBWB (brukeren limte inn hele ArduPilot-dokumentasjonsteksten) - "similar to FBWA, but Plane will
    // try to hold altitude as well... altitude is controlled using the elevator [as a climb RATE command,
    // not a direct pitch angle]... target airspeed is controlled using the throttle". Se egen
    // FBWB-konstantblokk (FBWB_CLIMB_RATE m.fl.) og stepPhysics for selve kontrolloven.
    fbwb: "FBWB",
    // QRTL - IKKE lenger utelatt (se js/simulator-vtol-rtl.js for hele autopilot-laget, lastet inn som en
    // egen fil rett etter denne). Bevisst IKKE med i isQMode()/DEFAULT_VTOL_PARAMS-familien: QRTL har sin
    // egen, dynamiske "effektiv modus" (fbwa/qloiter om hverandre avhengig av fase) i stedet for én fast
    // myndighet, se controlMode i stepPhysics.
    qrtl: "QRTL"
};
const AXIS_LABELS = { aileron: "Aileron", elevator: "Elevator", rudder: "Rudder" };
const CHANNEL_LABELS = { aileron: "Aileron", elevator: "Elevator", rudder: "Rudder", throttle: "Gass" };

const RATE_STORAGE_KEY = "ffi-uas:vtol-rates";
const GAMEPAD_STORAGE_KEY = "ffi-uas:vtol-gamepad-map";
const SETTINGS_STORAGE_KEY = "ffi-uas:vtol-settings";
const VTOL_PARAMS_STORAGE_KEY = "ffi-uas:vtol-transition-params";

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
    buttons: {
        // To DISKRÉ knapper (ikke én toggle, se setEngine-kommentaren) - én som alltid slår motoren PÅ,
        // én som alltid slår den AV, entydig uansett gjeldende tilstand.
        engineOn: null, engineOff: null, modeQStabilize: null, modeQHover: null, modeQLoiter: null, modeQAcro: null,
        modeManual: null, modeFbwa: null, modeFbwb: null, modeQrtl: null, trimUp: null, trimDown: null
    }
};
// trimUp/trimDown er IKKE med i BUTTON_ACTIONS/buttonManager under - de er kontinuerlige "hold inne"-
// handlinger (se adjustTrim), ikke enkelt-trigget på stigende kant slik engineOn/-Off/modus-knappene er.
// Ingen egen "transisjonsbryter" her (se MODE_LABELS-kommentaren) - modusknappene ER transisjonen.
const BUTTON_ACTION_LABELS = {
    engineOn: "Motor PÅ", engineOff: "Motor AV",
    modeQStabilize: "Modus: QSTABILIZE", modeQHover: "Modus: QHOVER", modeQLoiter: "Modus: QLOITER",
    modeQAcro: "Modus: QACRO", modeManual: "Modus: MANUAL", modeFbwa: "Modus: FBWA", modeFbwb: "Modus: FBWB",
    modeQrtl: "Modus: QRTL", trimUp: "Trim opp", trimDown: "Trim ned"
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
    flightMode: "qhover",
    planeClass: VTOL_CLASSES[settings.planeClass] ? settings.planeClass : DEFAULT_PLANE_CLASS,
    elevatorTrimDeg: 0,
    autoTrimFilteredDeflection: 0,
    lastRollDeflection: 0, lastPitchDeflection: 0, lastYawDeflection: 0,
    prevBankDeg: 0, prevPitchDegForD: 0,
    filteredBankRateDeg: 0, filteredPitchRateDeg: 0,
    // FBWB sin egen klatrerate-P-regulator sitt I-ledd (se fbwbClimbIntegral-bruken i stepPhysics) -
    // nullstilles hver gang FBWB forlates (anti-windup mellom økter i modusen).
    fbwbClimbIntegral: 0,
    // VTOL-spesifikke - lagres i stepPhysics for bruk i updatePlaneVisual (propellanimasjon) og
    // updateHud (samme "siste beregnede verdi fra fysikk-tick"-mønster som lastRollDeflection over).
    lastPusherThrottle: 0, lastCollectiveFrac: 0, lastMcAuthority: 1,
    // IAS (Indicated Airspeed) - se lastIndicatedAirspeed-utregningen i stepPhysics for hele
    // begrunnelsen (posisjonsfeil fra pitotrørets egen, forover-pekende monteringsakse). KUN til HUD, se
    // samme "siste beregnede verdi"-mønster som resten av denne gruppen.
    lastIndicatedAirspeed: 0,
    // Hvilken vingetupp som "brakk" ved selve krasj-øyeblikket (-1 venstre, 0 ingen/ikke krasjet, 1 høyre)
    // - se triggerCrash() og den visuelle bruken i updatePlaneVisual (brukertilbakemelding: "vingene blir
    // som et hjul. ruller og ruller. Kan vi knekke av vingen som tar hardt nedi og bremse flyet?").
    brokenWingSide: 0,
    // Halepropellen (pusher) "brekker" ALLTID ved et krasj (brukeren: "kanskje propellene knekker også
    // hvis de tar nedi ved krasj") - se triggerCrash()-kommentaren for hvorfor akkurat DENNE alltid
    // regnes som truffet, i motsetning til løftemotorene (kun brokenWingSide-siden, se over).
    brokenPusherProp: false,
    // Effektiv modus kontrolloven FAKTISK brukte forrige tick - lik flightMode for alle vanlige moduser,
    // men i "qrtl" er den enten "fbwa" (cruise-fase) eller "qloiter" (VTOL-fasene), se
    // js/simulator-vtol-rtl.js. Brukt av updatePlaneVisual/updateHud, som ikke selv har tilgang til
    // stepPhysics sin lokale controlMode-konstant.
    lastControlMode: "qhover"
};

let lastAirspeed = 0;

function currentPlaneSpec() {
    return VTOL_CLASSES[planeState.planeClass];
}

function setPlaneClass(className) {
    if (!VTOL_CLASSES[className]) return;
    planeState.planeClass = className;
    settings.planeClass = className;
    saveSettings();
    if (scene) rebuildPlaneMesh();
}

/* ---------- VTOL: Q-modus/Plane-modus-myndighet og weathervane ----------
   Navngivningen på DEFAULT_VTOL_PARAMS følger ArduPilot/Mission Planner sine ekte Q_*-parameternavn
   (se kommentarene på hvert felt) - brukeren ba spesifikt om ArduPilot-terminologi siden det er det som
   faktisk skal brukes, IKKE PX4 (som en tidligere versjon av denne filen brukte - se git-historikk).
   ArduPilot har INGEN egen "transisjonsbryter": selve FLIGHTMODE-valget ER transisjonen - å bytte til en
   Q-modus gir øyeblikkelig full svevemyndighet ("quad motors will immediately engage"), å bytte til
   MANUAL/FBWA går tilbake til fastvinget flyging (med eller uten assistanse, se computeMcAuthority) -
   se MODE_LABELS-kommentaren for hele modus-oversikten. */
const DEFAULT_VTOL_PARAMS = {
    // Q_ASSIST_SPEED / AIRSPEED_MIN: luftfart (m/s) flyet må nå i FBWA før løftemotor-assistansen
    // begynner å trappes ned (se computeMcAuthority) - under denne farten er assistansen alltid full.
    assistSpeed: 12,
    // Antall sekunder assistansen bruker på å trappes helt ned ETTER at assistSpeed nås første gang (se
    // ArduPilot-dokumentasjonen sitert av brukeren: "the amount of assistance...will decrease over 5
    // seconds. After that time the aircraft will be flying purely as a fixed wing").
    assistFadeSec: 5,
    // Q_WVANE_ENABLE: aktiver weathervaning (nesen dreier inn mot vinden) i QLOITER - ArduPilot støtter
    // fem retnings-varianter (nese/hale/side inn mot vind); forenklet til av/på her, kun standardverdien
    // "Nese inn mot vind" (se computeWeathervaneYawRateRad).
    wvaneEnabled: true,
    // Q_WVANE_GAIN: konverterer kommandert lenevinkel (grader) til girrate (grader/sekund) - "1 grad
    // krengning gir 1 grad/s gir" ved standardverdien 1.
    wvaneGain: 1,
    // Q_WVANE_ANG_MIN: minste kommanderte lenevinkel (grader) før weathervaning begynner å virke - unngår
    // unødig dreiing i (nesten) vindstille/med litt trim.
    wvaneAngMin: 1
};
function loadVtolParams() { return Sim.loadJSON(VTOL_PARAMS_STORAGE_KEY, DEFAULT_VTOL_PARAMS); }
function saveVtolParams() { Sim.saveJSON(VTOL_PARAMS_STORAGE_KEY, vtolParams); }
const vtolParams = loadVtolParams();

// assistFadeStartTime: tidspunktet FBWA-assistansen begynte å trappes ned (satt av computeMcAuthority
// første gang luftfarten når assistSpeed i FBWA, nullstilt idet farten faller under den igjen eller
// flightMode forlater FBWA) - eneste stykke VTOL-spesifikk TILSTAND simmen trenger nå (i motsetning til
// den forrige PX4-modellens egne switchOn/phase/pusherThrottle - se toppkommentaren).
const vtolState = { assistFadeStartTime: null };
function resetVtolState() { vtolState.assistFadeStartTime = null; }
function isQMode(mode) { return mode === "qstabilize" || mode === "qhover" || mode === "qloiter" || mode === "qacro"; }

// Kalt ved enhver motor-restart PÅ BAKKEN (resetPlane/toggleEngine/setEngine - se disse, samme
// onGround-vilkår som captureHome), brukertilbakemelding: "Ved restart må standard modus på bakken være
// en Q-modus. hover eller loiter. kan ikke starte i fixed wing modus på bakken" - en fastvinget modus
// (MANUAL/FBWA/FBWB) eller QRTL har ingen fornuftig "start rett opp fra stillstand"-oppførsel (FBWA/FBWB
// krever rulletakeoff, QRTL er en autopilot-modus, ikke en piloteringsmodus), så et fly som havner der ved
// et restart-øyeblikk (f.eks. etter en krasj i FBWA, eller motor slått av midt i en fastvinget tur) ville
// stå kraftløst uten å kunne lette rett opp igjen. Endrer INGENTING dersom flyet allerede står i en
// Q-modus (bevarer et bevisst QLOITER/QSTABILIZE-valg i stedet for alltid å tvinge QHOVER spesifikt).
function ensureGroundStartMode() {
    if (!isQMode(planeState.flightMode)) planeState.flightMode = "qhover";
}

// Multirotor-kontrollmyndighet (0-1) - se toppkommentaren for selve modellen. Q-moduser: ALLTID full
// myndighet, uansett fart (ArduPilot: "the quad motors will immediately engage"). MANUAL: ALDRI noen
// myndighet, uansett fart (ArduPilot: "the quad motors are disabled [i MANUAL mode]"). FBWA: full
// myndighet til assistSpeed nås, deretter en tidsbasert nedtrapping over assistFadeSec sekunder (se
// DEFAULT_VTOL_PARAMS) - IKKE en ren funksjon av øyeblikkelig fart slik en glattere modell ville gitt,
// siden ArduPilot selv beskriver dette som en tids-rampe som først STARTER idet terskelen krysses.
function computeMcAuthority(mode, airspeed, now) {
    if (mode === "manual") return 0;
    if (isQMode(mode)) {
        vtolState.assistFadeStartTime = null; // neste gang FBWA krysser terskelen skal telle fra 0 igjen
        return 1;
    }
    const p = vtolParams;
    if (airspeed < p.assistSpeed) {
        vtolState.assistFadeStartTime = null;
        return 1;
    }
    if (vtolState.assistFadeStartTime === null) vtolState.assistFadeStartTime = now;
    const elapsedSec = (now - vtolState.assistFadeStartTime) / 1000;
    return clamp(1 - elapsedSec / Math.max(0.1, p.assistFadeSec), 0, 1);
}

// Failsafe mot å hoppe rett i MANUAL uten nok luftfart: MANUAL gir (se computeMcAuthority over) ALLTID
// 0% løftemotor-myndighet MOMENTANT, uansett fart - i motsetning til FBWA, som alltid beholder full
// myndighet under assistSpeed (se samme funksjon), og derfor allerede er trygg å bytte til når som helst.
// Et ekte QuadPlane har ingen tilsvarende automatisk beskyttelse INNI selve MANUAL-modusen (motoren er
// og blir av der - derfor advarer ArduPilot pilotene selv mot å bruke MANUAL/ACRO for svevemotor-assistert
// flukt), men en ekte pilot/GCS ville heller aldri KOMMANDERT det bryteren uten fart i utgangspunktet -
// det er nettopp DEN beskyttelsen (mot selve bryteromslaget, ikke mot fysikken etter omslaget) som
// simuleres her, ved å AVVISE selve modusbyttet i stedet for å endre hva MANUAL faktisk gjør når du er i
// den. Bruker samme terskel (vtolParams.assistSpeed) som FBWA sin egen overgang - én kilde til sannhet,
// justerbar i samme VTOL-panel.
let modeBlockedUntil = 0;
// modeFlashUntil: brukt av updateHud til å legge på ".mode-flash"-CSS-klassen (kort gult blink, se
// style.css) på hudMode et lite øyeblikk hver gang selve modusen FAKTISK endres - brukeren ba om en
// tydelig visuell bekreftelse på modusbytte, siden HUD-teksten alene lett kan overses midt i en manøver.
// Kun satt ved et EKTE bytte (mode !== planeState.flightMode), ikke ved f.eks. gjentatte trykk på samme
// modusknapp eller et AVVIST bytte (se manual-sjekken over, som returnerer FØR dette punktet).
let modeFlashUntil = 0;
function trySetFlightMode(mode) {
    if (mode === "manual" && lastAirspeed < vtolParams.assistSpeed) {
        modeBlockedUntil = performance.now() + 3000;
        if (modeBlockedReasonEl) {
            modeBlockedReasonEl.textContent = "For lav luftfart (" + lastAirspeed.toFixed(1) + " < "
                + vtolParams.assistSpeed.toFixed(0) + " m/s) - løftemotorene ville kuttet momentant i MANUAL";
        }
        return false;
    }
    if (mode !== planeState.flightMode) modeFlashUntil = performance.now() + 400;
    planeState.flightMode = mode;
    return true;
}

// Se ArduPilot "Active Weathervaning" (sitert av brukeren): flyet dreier nesen inn MOT vinden mens det
// posisjonsholder i QLOITER (IKKE i QSTABILIZE/QHOVER - "not active in QSTABILIZE and QHOVER modes as
// those are not position controlled modes"), som reduserer vingens angrepsflate mot vinden. Bruker her
// den EKTE, simulerte vindvektoren direkte (currentWindVector) i stedet for å estimere den fra egen
// krengevinkel slik en ekte ArduPilot-kontroller må (den har ingen fasit å sammenligne mot) - "yaw inn i
// roll/pitch"-PRINSIPPET (se Q_WVANE_GAIN-kommentaren) er likevel det samme. leanMagDeg er gjeldende
// (allerede beregnede) kommandert lenevinkel i QLOITER, brukt til Q_WVANE_ANG_MIN-terskelen.
function computeWeathervaneYawRateRad(forwardWorld, leanMagDeg) {
    if (!vtolParams.wvaneEnabled) return 0;
    const windSpeed = currentWindVector.length();
    if (windSpeed < 0.5 || leanMagDeg < vtolParams.wvaneAngMin) return 0;
    // Nesen skal peke MOT der vinden kommer FRA, altså motsatt av vindens egen bevegelsesretning.
    const targetX = -currentWindVector.x / windSpeed, targetZ = -currentWindVector.z / windSpeed;
    // BUG (rapportert av brukeren: "weathervaner motsatt med halen inn i vinden") - cross var opprinnelig
    // forwardWorld.x*targetZ - forwardWorld.z*targetX, som beregner signert vinkel med MOTSATT fortegns-
    // konvensjon av motorens egen positive-gir-retning. Verifisert konkret: med forward=(0,0,-1) og en
    // vind som blåser i +x (target=(-1,0,0), altså nesen SKAL ende opp pekende mot -x), krever selve
    // rotasjonsintegratoren (se integrateOrientation/angVelVec, y-komponenten) en positiv gir på +90° for
    // å nå dit (positiv gir dreier nesen mot -x, se currentBankDeg/-PitchDeg-kommentaren over for samme
    // "forward rotert via applyQuaternion, ikke algebraisk"-forsiktighet). Den GAMLE formelen ga i stedet
    // -90° for akkurat dette tilfellet - stikk motsatt fortegn - som betydde at weathervaneYawRateRad
    // (lagt RETT INN i desiredMcYawRateRad) systematisk kommanderte gir i FEIL retning, og konvergerte mot
    // det ustabile motsatte likevektspunktet (halen inn i vinden) i stedet for det stabile (nesen inn i
    // vinden). Byttet komponentrekkefølgen (fz*tx - fx*tz i stedet for fx*tz - fz*tx) retter fortegnet -
    // samme type fiks som currentBankDeg-negasjonen andre steder i denne filen.
    const cross = forwardWorld.z * targetX - forwardWorld.x * targetZ;
    const dot = forwardWorld.x * targetX + forwardWorld.z * targetZ;
    const headingErrorRad = Math.atan2(cross, dot);
    // Q_WVANE_GAIN er offisielt "grader lenevinkel -> grader/s girrate" (et ÅPENT, ikke-konvergerende
    // ledd - se ArduPilot-dokumentasjonen: "yaw into the roll/pitch"). Brukt her i stedet som selve
    // P-FORSTERKNINGEN (rad/s girrate per rad retningsAVVIK, samme tallverdi/standard 1) - et rent
    // lenevinkel-proporsjonalt (ikke retnings-tilbakekoblet) moment ville aldri KONVERGERT mot faktisk å
    // peke inn i vinden, kun dreie proporsjonalt med hvor mye flyet lener. headingErrorRad går derimot
    // naturlig mot 0 idet nesen når målretningen - en ekte, konvergerende P-kontroller med samme
    // brukerjusterbare gain-tall.
    const maxRateRad = THREE.MathUtils.degToRad(MC_MAX_YAW_RATE_DEG);
    return clamp(headingErrorRad * vtolParams.wvaneGain, -maxRateRad, maxRateRad);
}

/* ---------- Vind ---------- */
const currentWindVector = new THREE.Vector3();
const windGustOffset = new THREE.Vector3();
function updateWind(dt) {
    Sim.computeWind(dt, settings.wind, windGustOffset, currentWindVector);
}

const inputState = {
    source: "keyboard",
    // throttle starter SENTRERT (0.5), IKKE i bunn (0) som fastvinge-simmens tomgangs-konvensjon - default
    // flightMode er qhover (se planeState under), et Alt Hold-regime der 0.5 betyr "hold nåværende høyde/
    // sveve i ro" (se MC_ALT_HOLD_DEADBAND-bruken i stepPhysics). Med pinnen i bunn ville Alt Hold i stedet
    // tolket det som et kommandert MAKS synk, som klemmer kollektiv trekkraft til 0 - løftemotorene ville
    // dermed aldri synlig spinne opp ved oppstart, og flyet ville stå der helt kraftløst på bakken (ingen
    // reell sveve-trekkraft til å holde det stødig) i stedet for å hvile rolig på hover-trekkraft.
    stick: { roll: 0, pitch: 0, yaw: 0, throttle: 0.5 }
};
const keys = new Set();

let renderer, scene, chaseCamera, fpvCamera, vlosCamera, activeCamera;
let planeGroup, planePropeller, planeLiftProps = [];
let planeAileronLeft, planeAileronRight, planeVtailLeft, planeVtailRight;
let propSpinSpeed = 0;
let liftPropSpinSpeed = 0;
// Rent visuelt differensial-tillegg (rad/s per full deflection) på løftemotorenes spinn ved kommandert
// rull/stigning - se bruken i updatePlaneVisual.
const LIFT_PROP_DIFFERENTIAL_GAIN = 60;
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

    // Asfalt-korn: et tett strøy av små, tilfeldig graderte flekker (lys/mørk rundt grunnfargen) over HELE
    // banen - gir rullebanen en synlig kornstruktur/skyggeanelse fra luften i stedet for én helt jevn,
    // flat gråtone (brukeren rapporterte direkte: "ha litt fin tekstur på rullebanen. så man bedre kan se
    // dybde. veldig flatt nå"). Tegnes FØR kantlinjene/midtlinjen/terskelstripene under, slik at DE
    // fortsatt står rene og skarpe oppå kornet, ikke selv sprutet til.
    const grainCount = 14000;
    for (let i = 0; i < grainCount; i++) {
        const gx = Math.random() * texW;
        const gy = Math.random() * texH;
        const shade = 58 + Math.random() * 46; // spenner rundt grunnfargens egen lysstyrke (~0x48=72)
        const alpha = 0.12 + Math.random() * 0.28;
        ctx.fillStyle = "rgba(" + shade + "," + shade + "," + (shade + 2) + "," + alpha + ")";
        ctx.beginPath();
        ctx.arc(gx, gy, 0.4 + Math.random() * 1.1, 0, Math.PI * 2);
        ctx.fill();
    }

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

        // INGEN vingestrebe her (i motsetning til fastvinge-simmens skoletrener-vinge denne ble kopiert
        // fra) - en VTOL-vinge av denne typen er selvbærende, uten diagonale strebepinner ned til skroget
        // (se referansebildene brukeren la ved - løftemotor-bommene, ikke strebepinner, er det synlige
        // under vingen, se addLiftBoom i buildPlane).
    });

    return group;
}

// Ett propellblad med en tapret silhuett (bredest nær navet, avsmalnende mot en avrundet tupp) i stedet
// for et rett rektangel (BoxGeometry), som brukeren rapporterte så ut som "flate rektangler". Bladet
// bygges FLATT i lokal XY (span langs X fra navet, korde langs Y, sentrert om 0), tynt i Z - riktig
// orientering for EN rotor som spinner om sin egen lokale Z-akse (se buildPlane sin pusherGroup, som
// roterer med rotation.z i updatePlaneVisual). For løftemotor-rotorene (som spinner om lokal Y i stedet,
// se liftPropGroup) roterer buildPlane hvert blad -90° om X etter kallet, som bytter om Y/Z og dermed gir
// riktig orientering for DEN rotasjonsaksen uten en egen kopi av selve bladformen.
function buildPropBlade(length, rootChord, tipChord, thickness, mat) {
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

// Bygger flyet prosedyralt (avrundet, konisk skrog, høyvinge, hale, understell, spinner/propell) -
// ingen eksterne modellfiler. Vingespenn/-areal skaleres etter VTOL_CLASSES.
function buildPlane(classKey) {
    const spec = VTOL_CLASSES[classKey];
    const group = new THREE.Group();
    const bodyMat = new THREE.MeshStandardMaterial({ color: 0xe8e8e8 });
    const accentMat = new THREE.MeshStandardMaterial({ color: 0xd23c3c });
    const wingMat = new THREE.MeshStandardMaterial({ color: 0xf2f2f2 });
    const darkMat = new THREE.MeshStandardMaterial({ color: 0x222222 });

    // Skrog: nesekjegle -> kabin (bredest) -> jevnt avsmalnende bakkropp mot halen, tre sylinderseksjoner
    // med matchende radius i skjøtene i stedet for én enkelt sylinder - gir et langt mer fly-aktig silhuett.
    // fuselageLength/cabinRadius/CABIN_LEN_RATIO/TAIL_LEN_RATIO er DELT med checkTailStrike (se konstantene
    // øverst i filen) - IKKE gjør disse til lokale, uavhengige tall, det holder tailstrike-varselet i synk.
    const fuselageLength = FUSELAGE_LENGTH_BUILD, cabinRadius = CABIN_RADIUS_BUILD;
    const noseLen = fuselageLength * NOSE_LEN_RATIO, cabinLen = fuselageLength * CABIN_LEN_RATIO, tailLen = fuselageLength * TAIL_LEN_RATIO;
    // (radiusTop/radiusBottom var byttet om - nesen buttet ut ved tuppen og snørte seg inn mot kabinen
    // i stedet for å tapre til et punkt, pga. hvordan rotation.x=90° flytter lokal +Y til verdens +Z.)
    const noseSection = new THREE.Mesh(new THREE.CylinderGeometry(cabinRadius, cabinRadius * NOSE_TIP_RADIUS_RATIO, noseLen, 14), bodyMat);
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

    // Ingen nesemotor på en pusher-VTOL (trekkraften kommer fra halepropellen, se pusherGroup lenger
    // ned) - nesen bærer i stedet en enkel sensor-/kamerakule, typisk for denne typen kartleggings-VTOL
    // (se referansebildene brukeren la ved).
    const sensorBall = new THREE.Mesh(new THREE.SphereGeometry(cabinRadius * 0.3, 10, 8), darkMat);
    sensorBall.position.z = -(cabinLen / 2 + noseLen + 0.02);
    sensorBall.castShadow = true;
    group.add(sensorBall);

    // INGEN cockpit-boble (brukeren: "Trenger ikke 'cockpit'. kanskje bare en sprekk/søm som indikerer
    // en lasteluke på toppen" - dette er en ubemannet kartleggings-VTOL, ikke et bemannet skolefly, en
    // glassboble ga derfor feil signal). Erstattet med en tynn, mørk rektangulær SØM-ramme (fire smale
    // bjelker, ikke en full, hevet lastelukestruktur - "bare en sprekk/søm som INDIKERER" en luke) lagt
    // rett over kabinseksjonens senterlinje. hatchY er regnet ut FRA selve sylinderens krumning (ikke en
    // hånd-justert konstant) slik at rammen garantert forblir embedded i skroget langs HELE sin bredde,
    // uansett cabinRadius/hatchHalfWidth-kombinasjon - se GEAR_LEG_FORWARD_LEAN_BUILD/
    // tailConeRadiusAtFrontRoot-kommentarene andre steder i filen for samme "overlapp i stedet for kun å
    // tangere"-prinsipp.
    const hatchHalfWidth = cabinRadius * 0.32, hatchLen = cabinLen * 0.5;
    const hatchLineW = cabinRadius * 0.05, hatchThickness = cabinRadius * 0.035;
    const hatchY = Math.sqrt(Math.max(0, cabinRadius * cabinRadius - hatchHalfWidth * hatchHalfWidth)) * 0.9;
    function hatchBar(w, l) {
        const bar = new THREE.Mesh(new THREE.BoxGeometry(w, hatchThickness, l), darkMat);
        bar.castShadow = true;
        return bar;
    }
    const hatchFront = hatchBar(hatchHalfWidth * 2, hatchLineW);
    hatchFront.position.set(0, hatchY, -cabinLen * 0.12 - hatchLen / 2);
    group.add(hatchFront);
    const hatchBack = hatchBar(hatchHalfWidth * 2, hatchLineW);
    hatchBack.position.set(0, hatchY, -cabinLen * 0.12 + hatchLen / 2);
    group.add(hatchBack);
    const hatchLeft = hatchBar(hatchLineW, hatchLen);
    hatchLeft.position.set(-hatchHalfWidth, hatchY, -cabinLen * 0.12);
    group.add(hatchLeft);
    const hatchRight = hatchBar(hatchLineW, hatchLen);
    hatchRight.position.set(hatchHalfWidth, hatchY, -cabinLen * 0.12);
    group.add(hatchRight);

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
    // BUG (rapportert av brukeren, etter en tidligere fairing-boks-fiks: "Nå er det en firkantig kloss der
    // vingen er festet mot kroppen. heller feste vingen lavere mot kroppen i stedet for å ha en rar kloss
    // på kroppen") - den forrige fiksen tettet gapet med en synlig, boksete fairing-kloss i stedet for å
    // fjerne selve GAPET. Fjernet fairingen igjen og senket WING_MOUNT_HEIGHT_RATIO (se konstanten øverst
    // i filen) til rett over/på selve sylinderoverflaten i stedet - vingen sitter dermed direkte MOT
    // skroget (en lav-/skulderplassert vinge, ikke lenger en tydelig høyvinge), uten noen egen synlig
    // festedel i det hele tatt.
    const wingMountY = cabinRadius * WING_MOUNT_HEIGHT_RATIO;
    const wing = buildWing({ wingArea: buildWingArea, wingSpan: buildWingSpan }, wingMat, darkMat);
    wing.position.set(0, wingMountY, 0.02);
    group.add(wing);

    // Hale: V-hale (to vinklede flater, hver med en hengslet "ruddervator" - en kombinert høyderor-/
    // sideror-flate, se bildene brukeren la ved) + en liten, fast bukfinne rett ned (KUN retningsstabilitet
    // - ingen egen rorflate, se ventralFin under). tailFrontZ er forkanten av haleflatene.
    const tailChord = wingChord * 0.7;
    const tailFrontZ = fuselageLength / 2 - tailChord;
    // Tynnere enn fastvinge-simmens opprinnelige haleflater (0.09) - en V-hale/bukfinne av denne typen
    // (se referansebildene) er slankere/mer aerodynamisk formet enn en tradisjonell trener-hale.
    const TAIL_SURFACE_THICKNESS_RATIO = 0.06;

    // V-halens vinkel fra vannrett (se referansebildene) - hver flate bygges FØRST som et rett,
    // loddrett panel (samme "verticalFin"-profilgenerator som en konvensjonell finne, se
    // buildWingProfileMesh), og VIPPES SÅ utover til riktig vinkel av tiltGroup sin egen rotation.z -
    // nøyaktig samme "bygg flatt, roter gruppen etterpå"-idiom som resten av filen allerede bruker
    // (se f.eks. streberne/aileronPivot i buildWing). Fortegnet på rotation.z (-side*vinkel) er
    // verifisert til å vippe høyre panel (side=1) mot +X og venstre (side=-1) mot -X.
    const VTAIL_ANGLE_DEG = 48; // brukeren ba om litt mer vinkel mellom flatene (opp fra 38)
    // Bakoverpil ("sweep"): en EKSTRA rotasjon om lokal X, lagt OVENPÅ V-vinkelen (rotation.z) på samme
    // tiltGroup, slik at hele panelet (rot OG tupp) vipper likt, ikke bare tuppen alene - gir et mer
    // aerodynamisk, bakovervinklet silhuett i stedet for et rett, loddrett panel (se referansebildene).
    // Samme fortegn for BEGGE sider (i motsetning til V-vinkelen, som speiles - pilvinkelen skal peke
    // samme vei, bakover, uansett hvilken side). ventralFin under bruker MOTSATT fortegn på nøyaktig
    // samme vinkel - se kommentaren der for hvorfor (finnen henger NED i stedet for opp).
    const TAIL_SWEEP_DEG = 18;
    const vtailHeight = cabinRadius * 2.1;
    const vFixedChord = tailChord * 0.62, vMovChord = tailChord * 0.46;
    const vCombinedChord = vFixedChord + vMovChord;
    const vMainFrac = vFixedChord / vCombinedChord;
    const vHingeChordZ = vMainFrac * vCombinedChord - vCombinedChord / 2;
    // Festepunktets høyde MÅ følge halekjeglens EGEN (avsmalnende) radius der panelet faktisk fester seg
    // (tailFrontZ, forkanten av selve rot-korden - den BREDESTE, mest konservative Z-posisjonen langs
    // panelets kordeutstrekning, siden kjeglen smalner videre bakover mot halespissen) - en FAST brøkdel
    // av cabinRadius (som før) stemte kun tilfeldigvis for ÉN vinge-/haleklasse, og etterlot et synlig
    // mellomrom mellom panelroten og skroget for andre kombinasjoner av VTAIL_ANGLE_DEG/TAIL_SWEEP_DEG/
    // tailChord (brukeren rapporterte "rart vinklet og festet... ser noe mellomrom"). 0.85 (ikke 1.0) gir
    // et lite, bevisst overlapp inn i kjeglen i stedet for kun å tangere overflaten - unngår en synlig
    // sprekk fra avrundingsfeil, samme prinsipp som en ekte påsveiset finne stikker litt inn i skroget.
    const tailConeRadiusAtFrontRoot = cabinRadius * THREE.MathUtils.lerp(
        1, TAIL_TIP_RADIUS_RATIO, clamp((tailFrontZ - cabinLen / 2) / tailLen, 0, 1)
    );
    const vtailBaseY = tailConeRadiusAtFrontRoot * 0.85;
    // Rundet hjørne der forkanten møter tuppen (se referansebildene) - en liten kule i samme materiale
    // som panelet, midt i tykkelsen, plassert nøyaktig i det hjørnet (se fixedPanel sitt eget Z-område:
    // forkanten ligger ved lokal Z=0, se pivot-kommentaren ved tiltGroup.position under for hvorfor).
    const tailTipFilletRadius = TAIL_SURFACE_THICKNESS_RATIO * vCombinedChord * 0.55;
    // Ruddervatorens span som andel av HELE panelets høyde ("innfelt ror" - se ruddervatorMesh- OG
    // vtailFrameMarginSpan-kommentarene under, som BEGGE MÅ bruke akkurat denne samme brøken for at
    // ramme-fyllet skal møte rorflaten helt uten synlig sprekk).
    const VTAIL_RUDDERVATOR_SPAN_FRAC = 0.72;
    const vtailPivotBySide = {};
    [-1, 1].forEach(function (side) {
        const tiltGroup = new THREE.Group();
        // Pivoten sitter på FORKANTEN av korden (lokal Z=0 her = tailFrontZ i verdensrom), IKKE korde-
        // SENTERET som tidligere (BUG rapportert av brukeren: "V halefinnene er litt tiltet bakover så de
        // har ikke helt kontakt med flykroppen") - tailConeRadiusAtFrontRoot/vtailBaseY over er begge
        // regnet ut NETTOPP for dette forkant-punktet (se kommentaren ved tailConeRadiusAtFrontRoot: "den
        // BREDESTE, mest konservative Z-posisjonen"), så det er kun DETTE punktet som er garantert å
        // tangere/overlappe halekjeglens overflate. Med pivoten i korde-SENTERET (som før) roterte
        // TAIL_SWEEP_DEG (rotation.x under) forkanten av panelet vekk fra akkurat det punktet - resten av
        // panelet fulgte fortsatt korrekt med, men selve roten (der flaten skal møte skroget) løftet seg
        // synlig fra kjeglen. Med pivoten på forkanten i stedet er DETTE punktet (lokal Y=0,Z=0) per
        // definisjon uberørt av enhver rotasjon om selve pivoten - roten forblir forankret i skroget
        // uansett sweep-/V-vinkel. Alle barn under er derfor flyttet +vCombinedChord/2 i lokal Z for å
        // kompensere (samme verdensposisjon som før ved rotation.x/z=0, kun selve pivot-punktet er flyttet).
        tiltGroup.position.set(0, vtailBaseY, tailFrontZ);
        tiltGroup.rotation.z = -side * THREE.MathUtils.degToRad(VTAIL_ANGLE_DEG);
        tiltGroup.rotation.x = THREE.MathUtils.degToRad(TAIL_SWEEP_DEG);
        group.add(tiltGroup);

        const fixedPanel = buildWingProfileMesh(vCombinedChord, 0, vMainFrac, TAIL_SURFACE_THICKNESS_RATIO, vtailHeight, wingMat, 0, false, true);
        fixedPanel.position.set(0, vtailHeight / 2, vCombinedChord / 2);
        tiltGroup.add(fixedPanel);

        // "Innfelt ror"-rammen (se ruddervatorMesh-kommentaren under - spennet der er bevisst mindre enn
        // fixedPanel sitt eget) trengte to små, FASTE fyllpaneler i BAKRE kordeområde (samme kordeområde
        // som selve ruddervatoren, vMainFrac til 1) ved topp- OG bunnmarginen - uten dem var det bakre
        // kordeområdet der ruddervatoren IKKE dekker (marginene over/under den) helt tomt/åpent (fixedPanel
        // dekker kun det FREMRE kordeområdet, over hele spennet) - så ut som et hakk/hull i halen i stedet
        // for en pent innfelt rorflate (se referansebildene brukeren la ved).
        const vtailFrameMarginSpan = vtailHeight * (1 - VTAIL_RUDDERVATOR_SPAN_FRAC) / 2;
        [0, 1].forEach(function (edge) {
            const frame = buildWingProfileMesh(vCombinedChord, vMainFrac, 1, TAIL_SURFACE_THICKNESS_RATIO, vtailFrameMarginSpan, wingMat, 0, false, true);
            frame.position.set(0, edge ? vtailHeight - vtailFrameMarginSpan / 2 : vtailFrameMarginSpan / 2, vCombinedChord / 2);
            tiltGroup.add(frame);
        });

        const tipFillet = new THREE.Mesh(new THREE.SphereGeometry(tailTipFilletRadius, 8, 6), wingMat);
        tipFillet.position.set(0, vtailHeight, tailTipFilletRadius * 0.6);
        tiltGroup.add(tipFillet);

        // Ruddervator-pivoten: rotation.x av DENNE (i sin egen, allerede vippede lokale ramme - se
        // updatePlaneVisual) gir samtidig et pitch- OG et yaw-bidrag i verdensrom, akkurat som en ekte
        // V-hale - fysikken (tailTorqueAtPitchRate/finTorqueAtYawRate) modellerer denne mikset
        // abstrakt/aerodynamisk og er UAVHENGIG av denne rent visuelle geometrien.
        // MERK: posisjonen her er RELATIV TIL tiltGroup, IKKE en absolutt fuselage-Z slik den originale
        // (ikke-vippede) hale-/finne-koden i fastvinge-simmen brukte direkte (tailFrontZ+stabChord) -
        // tiltGroup sitter nå på korde-FORKANTEN (se pivot-kommentaren over), så denne pivoten trenger
        // INGEN -vCombinedChord/2-korreksjon lenger (kun den rene hengslelinje-avstanden fra forkanten,
        // vFixedChord) for at hengslelinjen faktisk skal møte fixedPanel sin bakkant i stedet for å hoppe
        // et helt halvt kordemål bakover i løse luften (den opprinnelige bugen brukeren rapporterte som
        // "ser ut som to V-haler").
        const ruddervatorPivot = new THREE.Group();
        ruddervatorPivot.position.set(0, vtailHeight / 2, vFixedChord);
        // Innfelt: spennet (VTAIL_RUDDERVATOR_SPAN_FRAC, ned fra 0.9 opprinnelig) er bevisst mindre enn
        // fixedPanel sitt eget, slik at det står igjen en synlig kant/ramme av fast panel øverst OG
        // nederst rundt selve rorflaten - "innfelt ror" (se referansebildene), ikke en rorflate som dekker
        // hele panelets høyde kant til kant. Selve rammefyllet (vtailFrameMarginSpan) som lukker igjen
        // dette området i BAKRE kordeområde bygges over, sammen med fixedPanel.
        const ruddervatorMesh = buildWingProfileMesh(vCombinedChord, vMainFrac, 1, TAIL_SURFACE_THICKNESS_RATIO, vtailHeight * VTAIL_RUDDERVATOR_SPAN_FRAC, darkMat, -vHingeChordZ, false, true);
        ruddervatorPivot.add(ruddervatorMesh);
        tiltGroup.add(ruddervatorPivot);
        vtailPivotBySide[side] = ruddervatorPivot;
    });

    // INGEN hjul (verken hovedhjul eller nesehjul) - se referansebildene brukeren la ved: dette
    // understellet er fire rette BEN som henger ned fra løftemotor-bommene (se addLiftBoom under) pluss
    // BUKFINNEN som en femte, bakre landingsmeie/-ski - ikke et hjulunderstell i det hele tatt.
    // gearHeight er (som før) avledet direkte av spec.gearOffsetY (fysikkens faktiske bakkekontakthøyde)
    // delt på visualScale, slik at ALLE ben+bukfinne stemmer nøyaktig med fysikkens bakkekontaktplan
    // (se resolveGroundContact) for alle tre flystørrelsene - de skal alle nå ned til AKKURAT samme høyde,
    // slik at flyet står plant på bakken.
    const gearHeight = -spec.gearOffsetY / spec.visualScale;

    // Bukfinne/landingsmeie: FAST (ingen hengslet rorflate) finne rett ned fra halepartiet - gir ekstra
    // retningsstabilitet/-demping i luften (se ventralFinTorqueAtYawRate i stepPhysics) OG er samtidig
    // halens landingspunkt på bakken (se referansebildene) - derfor strekker den seg HELT ned til
    // gearHeight (samme høyde som beina under), ikke bare et lite stykke ned fra skroget.
    const ventralFinHeight = gearHeight, ventralFinChord = tailChord * 0.5;
    const ventralFin = buildWingProfileMesh(ventralFinChord, 0, 1, TAIL_SURFACE_THICKNESS_RATIO, ventralFinHeight, wingMat, 0, false, true);
    ventralFin.position.set(0, -ventralFinHeight / 2, tailFrontZ + ventralFinChord / 2);
    group.add(ventralFin);
    // Rundet hjørne der forkanten møter den nedre tuppen (se referansebildene og samme tipFillet-idé ved
    // V-halen over) - IKKE tilført pilvinkel/sweep her som V-halen fikk: bunnen av denne finnen ER
    // samtidig et av landingsbeina (se resolveGroundContact sin tailSkidZWorld), og en sweep ville flyttet
    // akkurat det kontaktpunktet ut av synk med fysikken - se GEAR_LEG_FORWARD_LEAN_BUILD-kommentaren for
    // hvorfor de FRIE beina trygt kan vinkles, mens dette faste punktet ikke kan det uten videre.
    const ventralTipFilletRadius = TAIL_SURFACE_THICKNESS_RATIO * ventralFinChord * 0.55;
    const ventralTipFillet = new THREE.Mesh(new THREE.SphereGeometry(ventralTipFilletRadius, 8, 6), wingMat);
    ventralTipFillet.position.set(0, -ventralFinHeight, tailFrontZ + ventralTipFilletRadius * 0.6);
    group.add(ventralTipFillet);

    // Trekkpropell (pusher) - montert bakerst på halekjeglen, IKKE på nesen (se sensorBall over) - to
    // enkle, slanke blad, samme byggemønster som fastvinge-simmens nesepropell hadde. tailTipZ er
    // halekjeglens FAKTISKE spiss (cabinLen/2+tailLen, IKKE den kun tilnærmede fuselageLength/2 - se
    // samme utregning i checkTailStrike) - en motorgondol bygger bro mellom spissen og selve propellen,
    // akkurat som cowl/spinner gjorde ved nesa, så propellen ikke henger synlig løsrevet i luften bak flyet.
    const tailTipZ = cabinLen / 2 + tailLen;
    const pusherMountLen = 0.09;
    // Flush overgang: pusherMount sin FRONTRADIUS (radiusBottom - se samme "-Y/radiusBottom mot
    // skroget"-mønster som tailSection over) matcher halekjeglens FAKTISKE tupp-radius nøyaktig
    // (cabinRadius*TAIL_TIP_RADIUS_RATIO), i stedet for en mye BREDERE, løsrevet krage (0.3/0.42*
    // cabinRadius, godt over tuppens egen ~0.16*cabinRadius) - det var derfor den "ikke var flush med
    // resten av kroppen". bodyMat (ikke rød accentMat) lar den lese som en naturlig FORLENGELSE av
    // skroget, ikke en påklistret del.
    const pusherMount = new THREE.Mesh(
        new THREE.CylinderGeometry(cabinRadius * TAIL_TIP_RADIUS_RATIO * 0.55, cabinRadius * TAIL_TIP_RADIUS_RATIO, pusherMountLen, 14),
        bodyMat
    );
    pusherMount.rotation.x = Math.PI / 2;
    pusherMount.position.z = tailTipZ + pusherMountLen / 2;
    pusherMount.castShadow = true;
    group.add(pusherMount);

    const pusherGroup = new THREE.Group();
    const propMat = new THREE.MeshStandardMaterial({ color: 0x151515 });
    const pusherBladeLen = cabinRadius * 2.1;
    [-1, 1].forEach(function (dir) {
        const blade = buildPropBlade(pusherBladeLen, pusherBladeLen * 0.24, pusherBladeLen * 0.11, 0.008, propMat);
        blade.rotation.z = dir > 0 ? 0 : Math.PI;
        pusherGroup.add(blade);
    });
    pusherGroup.position.z = tailTipZ + pusherMountLen + 0.02;
    group.add(pusherGroup);

    // Fire faste løftemotorer, montert på RETTE bommer som går GJENNOM vingen langs vingens EGEN
    // spennretning - IKKE en diagonal "X"-layout radiert ut fra skroget slik en ren quadcopter har (se
    // referansebildene brukeren la ved: én bom per vingeside, festet TIL vingen, med én motor fremst og
    // én bakerst på hver bom - fire motorer totalt, null skrog-monterte armer). boomX/boomHalfLen er (som
    // buildWingSpan over) i BYGGE-rom, siden hele gruppen skaleres med spec.visualScale til slutt.
    // boomX MÅ holde seg innenfor vingens FASTE midtseksjon (centerSpan i buildWing, dvs. innenfor
    // wingSpan*0.28 fra senterlinjen - se aileronSpan/centerSpan der) - ellers stikker bommen rett
    // gjennom balanserorets utsparing på vingetuppen, som brukeren rapporterte ("pinnene til
    // quadpropellene går rett gjennom ailerons"). 0.22 gir trygg klaring til aileron-grensen (0.28), og
    // MÅ matche GEAR_BOOM_X_FRAC i resolveGroundContact (se der - beina henger fra nøyaktig disse
    // bom-posisjonene, se referansebildene: "pinner som stikker ned under vingen fra rørene som
    // quadmotorene er montert på").
    const boomX = buildWingSpan * GEAR_BOOM_X_FRAC;
    const boomHalfLen = spec.armLen / spec.visualScale * 0.5;
    const armMat = new THREE.MeshStandardMaterial({ color: 0x2a2a2a });
    const liftMotorMat = new THREE.MeshStandardMaterial({ color: 0x151515 });
    const liftPropGroups = [];
    [-1, 1].forEach(function (side) {
        const boomXPos = side * boomX;
        // Sylinderens egen lengdeakse (lokal Y) roteres 90° om X til å peke langs lokal Z (fram/bak) -
        // samme "bygg langs Y, roter til riktig akse etterpå"-idiom som resten av filen (se f.eks.
        // buildWingProfileMesh).
        const boom = new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.018, boomHalfLen * 2, 8), armMat);
        boom.rotation.x = Math.PI / 2;
        boom.position.set(boomXPos, wingMountY, BOOM_CENTER_Z_BUILD);
        boom.castShadow = true;
        group.add(boom);

        // Motor + propell i hver ende av bommen - fremre farges rødlig (accentMat, samme konvensjon som
        // quad-simulatorens frontArmMat) for å gjøre nese-retningen tydelig på avstand.
        [
            { z: BOOM_CENTER_Z_BUILD - boomHalfLen, front: true },
            { z: BOOM_CENTER_Z_BUILD + boomHalfLen, front: false }
        ].forEach(function (m) {
            const motorPod = new THREE.Mesh(new THREE.CylinderGeometry(0.028, 0.032, 0.05, 10), m.front ? accentMat : liftMotorMat);
            motorPod.position.set(boomXPos, wingMountY, m.z);
            motorPod.castShadow = true;
            group.add(motorPod);

            // Landingsben: KUN på det FREMRE motorparet (se referansebildene brukeren la ved - "kun tre
            // landingsben, to på midten og et bak der det bakerste er den vertikale stabben") - rett ned
            // fra motoren til bakkeplanet (gearHeight, se over), IKKE hjul. legBottomY er ABSOLUTT
            // (fuselage-Y, ikke relativt til motoren), siden gearHeight allerede er målt fra CG (Y=0),
            // akkurat som bukfinnen (tredje/bakerste "ben") over.
            if (m.front) {
                // Benet festes IKKE rett under selve motoren (m.z) - se GEAR_LEG_BOOM_Z_FRAC-kommentaren -
                // festepunktet flyttes et stykke TILBAKE langs bommen, mot bom-/vingefestets senter, slik
                // at benet sitter mer under selve vingen enn under fremre motor. Henger ellers rett ned i
                // X (samme boomXPos - IKKE vinklet sideveis innover mot senterlinjen) med kun en liten
                // forover-lene (-Z) ned mot bakkeplanet (se GEAR_LEG_FORWARD_LEAN_BUILD-kommentaren og
                // referansebildene brukeren la ved). Samme "punkt-til-punkt-sylinder via
                // setFromUnitVectors"-mønster som understellet i fastvinge-simmen opprinnelig brukte for
                // de diagonale hjulstrebene.
                const legAttachZ = THREE.MathUtils.lerp(m.z, BOOM_CENTER_Z_BUILD, GEAR_LEG_BOOM_Z_FRAC);
                const legTop = new THREE.Vector3(boomXPos, wingMountY, legAttachZ);
                const legBottom = new THREE.Vector3(
                    boomXPos,
                    -gearHeight,
                    legAttachZ - GEAR_LEG_FORWARD_LEAN_BUILD
                );
                const legVec = legBottom.clone().sub(legTop);
                const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, legVec.length(), 8), darkMat);
                leg.position.copy(legTop).addScaledVector(legVec, 0.5);
                leg.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), legVec.clone().normalize());
                leg.castShadow = true;
                group.add(leg);
            }

            const liftPropGroup = new THREE.Group();
            liftPropGroup.position.set(boomXPos, wingMountY + 0.03, m.z);
            const liftBladeLen = 0.17;
            [-1, 1].forEach(function (dir) {
                const blade = buildPropBlade(liftBladeLen, liftBladeLen * 0.22, liftBladeLen * 0.1, 0.006, liftMotorMat);
                blade.rotation.x = Math.PI / 2;
                blade.rotation.y = dir > 0 ? 0 : Math.PI;
                liftPropGroup.add(blade);
            });
            group.add(liftPropGroup);
            // side/front lagres sammen med selve gruppen (ikke bare gruppen alene) - updatePlaneVisual
            // trenger dem til å spinne motorene ULIKT etter kommandert rull/stigning (se
            // LIFT_PROP_DIFFERENTIAL_GAIN-kommentaren der), ikke bare identisk kollektiv spinn for alle
            // fire slik det var før (brukeren rapporterte at det så rart ut at flyet tiltet uten at NOEN
            // motor så ut til å gå raskere).
            liftPropGroups.push({ group: liftPropGroup, side: side, front: m.front });
        });
    });

    // Pitot-rør: tynt, langt rør under høyre vinge, pekende forover inn i luftstrømmen (rent visuelt -
    // ingen egen fysikk/luftfartsmåling knyttet til det, se referansebildene brukeren la ved).
    const pitotLen = wingChord * 0.9;
    const pitotTube = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, pitotLen, 8), darkMat);
    pitotTube.rotation.x = Math.PI / 2;
    pitotTube.position.set(boomX * 0.55, wingMountY - 0.05, -pitotLen / 2 + wingChord * 0.15);
    pitotTube.castShadow = true;
    group.add(pitotTube);
    const pitotMount = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, 0.06, 6), darkMat);
    pitotMount.position.set(boomX * 0.55, wingMountY - 0.03, wingChord * 0.15);
    group.add(pitotMount);

    group.scale.setScalar(spec.visualScale);
    return {
        group: group, propeller: pusherGroup, liftProps: liftPropGroups,
        aileronLeft: wing.userData["aileron-1"], aileronRight: wing.userData["aileron1"],
        vtailLeft: vtailPivotBySide[-1], vtailRight: vtailPivotBySide[1]
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
}

function rebuildPlaneMesh() {
    if (planeGroup) scene.remove(planeGroup);
    const built = buildPlane(planeState.planeClass);
    planeGroup = built.group;
    planePropeller = built.propeller;
    planeLiftProps = built.liftProps;
    planeAileronLeft = built.aileronLeft;
    planeAileronRight = built.aileronRight;
    planeVtailLeft = built.vtailLeft;
    planeVtailRight = built.vtailRight;
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
    engineOn: function () { setEngine(true); },
    engineOff: function () { setEngine(false); },
    modeQStabilize: function () { trySetFlightMode("qstabilize"); },
    modeQHover: function () { trySetFlightMode("qhover"); },
    modeQLoiter: function () { trySetFlightMode("qloiter"); },
    modeQAcro: function () { trySetFlightMode("qacro"); },
    modeManual: function () { trySetFlightMode("manual"); },
    modeFbwa: function () { trySetFlightMode("fbwa"); },
    modeFbwb: function () { trySetFlightMode("fbwb"); },
    modeQrtl: function () { trySetFlightMode("qrtl"); }
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

// Skråror (aileron) mister effektivitet når VINGEN DEN SITTER PÅ allerede er (nær) steilet på EGEN HÅND -
// en ekte, separert luftstrøm kan ikke levere den ekstra, proporsjonale løft-økningen et rorutslag ber
// om. Brukeren påpekte at vingen "tipper over veldig lett" og etterlyste at et ekte treningsfly i steiling
// krever SIDEROR (ikke skråror) for å holde vingene rette - se finTorqueAtYawRate i stepPhysics, som er
// UPÅVIRKET av vingens steiling (sideroret sitter på halen, ikke på selve vingen, og mister derfor ikke
// autoritet her) og dermed forblir det virkemiddelet som faktisk virker, akkurat som i et ekte fly.
//
// VIKTIG: evaluert på VINGENS EGEN, RÅ AoA (baseAoaDeg - FØR ailerons eget bidrag legges til), IKKE den
// KOMBINERTE vinkelen (base+aileron) - se rightWing/leftWing- og wingTorqueForce-kommentarene der
// controlAoaDeg/extraAoaDeg brukes, som eksplisitt dokumenterer HVORFOR en tidligere versjon som lot
// AILERON-UTSLAGET SELV telle med i steile-vurderingen ble fjernet ("et fullt rorutslag presset tidligere
// den nedadgående vingen inn i steilingens flate område ved nettopp lav fart/høy AoA... Dette var den
// reelle årsaken til at rull konsekvent føltes tregt", f.eks. under avgangsrotasjon). Denne funksjonen
// gjentar IKKE den feilen: den ser KUN på om vingen ALLEREDE er steilet uavhengig av roret, så normal
// rull-autoritet ved moderat AoA (takeoff-rotasjon, brattere svinger osv.) er helt uendret - kun i en
// EKTE, etablert steiling (vingens egen AoA forbi stallAngleDeg) svekkes skråroret.
function aileronLiftEffectiveness(baseAoaDeg, spec) {
    const absA = Math.abs(baseAoaDeg);
    const stall = spec.stallAngleDeg;
    if (absA < stall) return 1;
    if (absA < stall + STALL_POST_RANGE_DEG) {
        const progress = (absA - stall) / STALL_POST_RANGE_DEG;
        return 1 - progress * 0.85; // 1.0 -> 0.15 gjennom overgangssonen
    }
    return 0.15; // aldri helt null - et halvveis nedadgående skråror gir fortsatt LITT ekstra motstand/
    // vridning selv dypt inne i steiling, kun mye mindre proporsjonal løft enn ved fri luftstrøm.
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
    // BUG (rapportert av brukeren, med skjermbilde: flyet ble stående BALANSERT PÅ NESA i en fysisk umulig
    // positur etter et krasj - "flyet kan bli hengende i unaturlige posisjoner etter krasj og det har
    // stoppet opp... skal ikke være sånn") - denne nullstillingen kjørte UBETINGET, ogSÅ mens
    // planeState.crashed er sann. Et krasjet fly kan fullt lovlig ha lav LINEÆR fart (mesteparten alt tapt
    // til CRASH_ENERGY_LOSS_FRAC/bakkefriksjonen) mens det FORTSATT egentlig burde falle videre om -
    // nesetupp-fjærkraften over gir et ekte, fysisk VELTENDE dreiemoment (se resolveGroundContact) hver
    // eneste tick et fly balanserer nesetungt, akkurat som en blyant balansert på spissen. Denne
    // nullstillingen slettet den oppbygde vinkelfarten FØR den noensinne rakk å velte flyet videre, og
    // fanget det dermed for alltid i akkurat den (fysisk ustabile, men numerisk "fastfrosne") balansen.
    // KUN gjelder derfor nå UTENFOR krasjet tilstand - et krasjet fly får lov til å fullføre sin egen,
    // ekte velte-/settefase uansett hvor lav farten momentant er.
    if (planeState.velocity.length() < 0.4 && !planeState.crashed) {
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
        // BUG (rapportert av brukeren TO ganger nå, med skjermbilder: "sklir med vingen nedi bakken.
        // stopper aldri å skli") - selv med CRASH_FRICTION_MULTIPLIER og det brukne vingepunktet fjernet
        // (se resolveGroundContact) kan et krasjet skrog fortsatt "rulle" seg fremover UTEN reell
        // GLIDNING ved noe kontaktpunkt til enhver tid - akkurat som et hjul translaterer fremover uten at
        // NOE punkt på hjulkanten faktisk glir mot bakken idet det ruller, kan et tumlende vrak flytte
        // CG-en fremover mens et STADIG NYTT punkt (ben, hale, gjenværende vingetupp, nese) hver for seg
        // har tilnærmet null EGEN kontaktpunkt-hastighet i det korte øyeblikket det er i bakken - den
        // rene, per-punkt Coulomb-friksjonsmodellen (se resolveGroundContact) er STRUKTURELT blind for
        // nettopp denne typen bevegelse, uansett hvor høy friksjonskoeffisienten settes (se
        // BUG-kommentaren ved det fjernede vingepunktet for samme "null glidningsfart = null
        // friksjonskraft"-prinsipp). I stedet for å prøve å modellere denne subtile rotasjons-/
        // glidningskoblingen fysisk korrekt (dyrt og skjørt), legges det her på en enkel, ROBUST, direkte
        // eksponentiell luftmotstand/slepe-drag på selve CG-fartens horisontale komponent, UAVHENGIG av
        // per-punkt-kinematikken - fysisk begrunnet (et faktisk ødelagt, tumlende vrak drar med seg
        // avrevne deler / graver kontinuerlig i bakken / møter luftmotstand fra sin egen uregelmessige,
        // ikke lenger aerodynamiske form) og praktisk talt gratis (to multiplikasjoner per tick) - MEN
        // viktigst: den er GARANTERT å stoppe farten innen kort, forutsigbar tid, uansett hvilken (evt.
        // "hjul-lignende") rotasjon skroget måtte ha, i stedet for å stole utelukkende på en
        // kontaktmodell som kan feile i akkurat denne situasjonen.
        // BUG (rapportert av brukeren, oppfølging: "krasjer med nesa først. så ruller flyet unaturlig mye
        // fremover også") - NØYAKTIG samme "null glidningsfart under ren rulling"-hull som over, bare på
        // VINKELFARTEN (stigning) i stedet for CG-farten - et fly som somersaulter forover om nesetuppen
        // kan ha tilnærmet null lineær kontaktpunkt-hastighet DER akkurat i det det fortsatt roterer, så
        // PASSIVE_ANGULAR_DAMPING alene (uendret, svak, 0.995/tick - ment for en fortsatt LUFTBÅREN
        // ettertumling, ikke en som allerede henger fast mot bakken) rekker ikke å stanse den. Samme
        // robuste, direkte eksponentielle løsning som over, nå på selve vinkelfarten - KUN mens flyet
        // faktisk er i bakkekontakt (ren luftbåren tumling etter selve sammenstøtet skal fortsatt dempes
        // av den opprinnelige, slakere PASSIVE_ANGULAR_DAMPING alene).
        if (planeState.onGround) {
            const crashDragDecay = Math.exp(-CRASH_DRAG_RATE_PER_SEC * dt);
            planeState.velocity.x *= crashDragDecay;
            planeState.velocity.z *= crashDragDecay;
            const crashAngularDragDecay = Math.exp(-CRASH_GROUND_ANGULAR_DRAG_PER_SEC * dt);
            planeState.angularVelocity.pitch *= crashAngularDragDecay;
            planeState.angularVelocity.roll *= crashAngularDragDecay;
            planeState.angularVelocity.yaw *= crashAngularDragDecay;
        }
        // Statisk "sett seg til ro" for resten - se BUG-kommentaren over. Fanger den lille resten
        // eksponentiell drag alene aldri helt når frem til (asymptotisk mot 0, aldri EKSAKT 0).
        if (planeState.onGround) {
            const horizSpeedSq = planeState.velocity.x * planeState.velocity.x + planeState.velocity.z * planeState.velocity.z;
            if (horizSpeedSq < CRASH_SETTLE_SPEED_MS * CRASH_SETTLE_SPEED_MS) {
                planeState.velocity.x = 0;
                planeState.velocity.z = 0;
            }
        }
        return;
    }

    // QRTL (autopilot-lag, se js/simulator-vtol-rtl.js) - MÅ kjøre FØR stick leses under, siden
    // updateRtlAutopilot skriver syntetiske pinneverdier RETT INN i inputState.stick (samme objekt) i
    // stedet for ekte pilotinput mens "qrtl" er aktiv modus. Returnerer en "effektiv modus" som resten av
    // denne funksjonen bruker i STEDET for planeState.flightMode - se controlMode rett under - siden QRTL
    // sjonglerer mellom en FBWA-lignende (fastvinget cruise) og en QLOITER-lignende (VTOL-retur/landing)
    // myndighet avhengig av hvilken fase den er i. planeState.flightMode selv rører verken denne eller
    // updateRtlAutopilot - HUD/lagring/tastatur-guard skal fortsatt se det bokstavelige "qrtl".
    const rtlEffectiveMode = planeState.flightMode === "qrtl" ? updateRtlAutopilot(dt) : null;
    if (!rtlEffectiveMode) { rtlState.phase = "idle"; rtlState.landedTimer = 0; }
    const controlMode = rtlEffectiveMode || planeState.flightMode;
    planeState.lastControlMode = controlMode;

    const stick = inputState.stick;
    const throttleShaped = computeThrottleCurve(stick.throttle, rates.throttle.expo);

    const q = planeState.quaternion;
    const invQ = q.clone().invert();
    const airVelWorld = planeState.velocity.clone().sub(currentWindVector);
    const localAirVel = airVelWorld.clone().applyQuaternion(invQ);
    const airspeed = airVelWorld.length();
    lastAirspeed = airspeed;
    // IAS (Indicated Airspeed) - BUG/ønske (rapportert av brukeren: "'Fart' kan kanskje hete 'Airspeed'
    // eller 'IAS'? og sjekk at dette er faktisk airspeeden som pitot røret ville målt. gjerne ta høyde
    // for posisjonsfeil også") - HUD-en viste FØR lastAirspeed direkte, altså flyets SANNE 3D-luftfart
    // (magnituden til hele airVelWorld-vektoren). Et EKTE pitotrør måler ikke det - det måler kun
    // DYNAMISK TRYKK fra luftstrømmens komponent LANGS RØRETS EGEN akse (se buildPlane: pitotTube er
    // montert forover-pekende, rotation.x=90° roterer den til å peke langs lokal Z, altså rett frem).
    // -localAirVel.z ER akkurat den forover-rettede komponenten (samme uttrykk som
    // forwardAirspeedIntoProp lenger ned bruker for propellen, gjenbrukt her for konsistens). Ved AoA/
    // sideslip ulik null er den lokale luftstrømmen IKKE lenger på linje med røret, så komponenten langs
    // røret er per definisjon MINDRE enn hele 3D-farten (Pytagoras: én komponent kan aldri overstige
    // vektorens egen lengde) - dette ER selve "posisjonsfeilen" brukeren spurte om: et EKTE fly viser
    // ofte en litt LAVERE ASI-avlesning enn den sanne farten nettopp ved høy AoA (f.eks. nær steiling),
    // av akkurat denne fysiske grunnen. Klemt til >=0 - en pitot gir ingen meningsfull avlesning ved
    // reversert/rent sidelengs gjennomstrømning. Lagret på planeState (samme "siste beregnede verdi"-
    // mønster som lastPusherThrottle m.fl.) og brukt KUN i HUD (se updateHud) - selve
    // FYSIKKEN/autopiloten (assistSpeed-terskler, L1-styring, steilemodell osv.) bruker fortsatt den
    // SANNE luftfarten (lastAirspeed/airspeed) UENDRET, akkurat som en ekte autopilot/steilevarslingssystem
    // helst ville brukt en pålitelig sann-fart-kilde i stedet for kun det (upålitelige) pitot-tallet.
    planeState.lastIndicatedAirspeed = Math.max(-localAirVel.z, 0);
    const aoaDeg = airspeed > 0.3 ? THREE.MathUtils.radToDeg(Math.atan2(-localAirVel.y, -localAirVel.z)) : 0;
    const sideslipDeg = airspeed > 0.3 ? THREE.MathUtils.radToDeg(Math.atan2(localAirVel.x, -localAirVel.z)) : 0;

    // Q-modus/Plane-modus-myndighet (0-1) - se computeMcAuthority-kommentaren. Beregnes tidlig siden BÅDE
    // trekkmotor- og løftemotor-thrust, OG selve vinkel-P(D)-målet (targetBankDeg/-PitchDeg under),
    // trenger den.
    const mcAuthority = computeMcAuthority(controlMode, airspeed, performance.now());
    planeState.lastMcAuthority = mcAuthority;
    const liftMotorsActive = mcAuthority > 0.001;
    // Batteri (simulert utholdenhetsproxy, se js/simulator-vtol-rtl.js) - oppdateres HVER tick uansett
    // modus, ikke bare i "qrtl", siden det er selve forbruket (og en evt. lavspenning-failsafe TRIGGER av
    // qrtl) som skal skje kontinuerlig. Gjenbruker mcAuthority direkte som "hvor mye i hover-regime"
    // (0=ren fastvinget cruise, 1=full svevemyndighet) i stedet for å anslå motorpådrag på nytt.
    updateBattery(dt, mcAuthority);

    const forwardAirspeedIntoProp = Math.max(-localAirVel.z, 0);
    // Trekkpropellen (pusher) styres av gasspaken KUN i MANUAL/FBWA/FBWB (fastvinget-regimet) - i en
    // Q-modus er den alltid av (ArduPilot har en egen, valgfri "manuell forover-gass i VTOL-moduser"-RC-
    // kanal, RCx_OPTION 209, som vi ikke har implementert her - se toppkommentaren). FBWB er et unntak
    // INNENFOR fastvinge-regimet: gasspaken styrer der ikke pådraget direkte, men en ØNSKET LUFTFART (se
    // FBWB-konstantblokken over) - en enkel P-regulator på fartsavviket erstatter throttleShaped sin
    // ellers direkte passthrough for nettopp denne modusen (ArduPilot: "the throttle will control the
    // target airspeed... If throttle is minimum then the plane will try to fly at AIRSPEED_MIN. If it is
    // maximum it will try to fly at AIRSPEED_MAX").
    let pusherThrottleEff;
    if (controlMode === "fbwb") {
        const desiredAirspeed = THREE.MathUtils.lerp(FBWB_MIN_AIRSPEED, FBWB_MAX_AIRSPEED, stick.throttle);
        const speedError = desiredAirspeed - airspeed;
        pusherThrottleEff = clamp(FBWB_THROTTLE_TRIM + speedError * FBWB_SPEED_GAIN, 0, 1);
    } else {
        pusherThrottleEff = isQMode(controlMode) ? 0 : throttleShaped;
    }
    planeState.lastPusherThrottle = pusherThrottleEff;
    // En ekte (særlig fastpitch) propell mister trekkraft omtrent lineært med farten, fra full statisk
    // trekkraft ved V=0 til ~null idet flyet nærmer seg propellens "pitch speed" (spec.propPitchSpeed) -
    // uten dette var toppfarten satt av drag alene (urealistisk høy for en liten trener), og gass av i
    // høy fart ga ikke den brattere glidebanen en vindmøllende propell faktisk gir.
    const thrustForce = planeState.engineOn
        ? pusherThrottleEff * spec.pusherMaxThrust * Math.max(0, 1 - forwardAirspeedIntoProp / spec.propPitchSpeed)
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
    let weathervaneYawRateRad = 0;
    if (controlMode === "manual" || controlMode === "qacro") {
        // Direkte fra pinnen (ingen selvnivellering) - MANUAL er fastvinget-ekvivalenten (ArduPilot:
        // "Please use FBWA mode instead of STABILIZE for manual flight" - ren pinnestyring er kun ment
        // for trim-sjekk/taksing, se MODE_LABELS-kommentaren). QACRO er samme prinsipp i Q-modus
        // (rate-styrt svevemodus, som QACRO/quad-simulatorens Acro).
        // Trim er IKKE blandet inn i selve pinne-avbøyningen her - se tailAoaDeg under, der
        // elevatorTrimDeg påvirker halens angrepsvinkel DIREKTE (som en ekte trim-fane), uavhengig av
        // rorutslaget. pitchDeflection er dermed et rent uttrykk for PINNENS egen posisjon.
        rollDeflection = clamp(computeRate(stick.roll, rates.aileron) / rates.aileron.maxRate, -1, 1);
        pitchDeflection = clamp(computeRate(stick.pitch, rates.elevator) / rates.elevator.maxRate, -1, 1);
        yawDeflection = clamp(computeRate(stick.yaw, rates.rudder) / rates.rudder.maxRate, -1, 1);
    } else {
        // fbwa/fbwb/qstabilize/qhover/qloiter: alle fem er selvnivellerende vinkel-P(D)-moduser - SAMME
        // kontrollov (uendret nedenfor). Kun selve MÅLVINKELEN (targetBankDeg/-PitchDeg) beregnes ulikt:
        // direkte fra pinnen for fbwa/qstabilize/qhover (skalert mellom fastvinget- og hover-vinkelmaks
        // etter mcAuthority, se MC_MAX_LEAN_ANGLE-kommentaren), fra en klatrerate-P-regulator i FBWB (se
        // FBWB-konstantblokken/-grenen under), eller - i QLOITER - fra et horisontalt fartsavvik
        // (posisjonsholding).
        const euler = new THREE.Euler().setFromQuaternion(q, "YXZ");
        const currentPitchDeg = -THREE.MathUtils.radToDeg(euler.x);
        const currentBankDeg = -THREE.MathUtils.radToDeg(euler.z);

        let targetBankDeg, targetPitchDeg;
        if (controlMode === "qloiter" && mcAuthority > 0.01) {
            // QLOITER: pinnen kommanderer horisontal FART (kropp-relativt forover/sideveis, se
            // QLOITER_MAX_SPEED - ArduPilot: "Horizontal location can be adjusted with the Roll and
            // Pitch control sticks... When the pilot releases the sticks the QuadPlane will slow to a
            // stop") i stedet for en vinkel direkte. bodyForwardFlat/bodyRightFlat er nesens/høyre-siden
            // sin retning FLATET til horisontalplanet (ikke en full 3D-projeksjon via quaternion-
            // invertering, som ville blandet inn gjeldende krengning/stigning i selve fartsMÅLINGEN).
            const bodyForwardFlat = new THREE.Vector3(0, 0, -1).applyQuaternion(q);
            bodyForwardFlat.y = 0;
            if (bodyForwardFlat.lengthSq() < 1e-6) bodyForwardFlat.set(0, 0, -1); else bodyForwardFlat.normalize();
            const bodyRightFlat = new THREE.Vector3(1, 0, 0).applyQuaternion(q);
            bodyRightFlat.y = 0;
            if (bodyRightFlat.lengthSq() < 1e-6) bodyRightFlat.set(1, 0, 0); else bodyRightFlat.normalize();

            const groundVelFlat = new THREE.Vector3(planeState.velocity.x, 0, planeState.velocity.z);
            const fwdSpeed = groundVelFlat.dot(bodyForwardFlat);
            const rightSpeed = groundVelFlat.dot(bodyRightFlat);
            const desiredFwdSpeed = stick.pitch * QLOITER_MAX_SPEED;
            const desiredRightSpeed = stick.roll * QLOITER_MAX_SPEED;
            // Samme FORTEGN som den direkte pinne->vinkel-kommandoen under (stick.pitch/roll -> target-
            // Deg direkte) - dette ER akkurat den kommandoen, bare med et fartsAVVIK i stedet for selve
            // pinneposisjonen som P-leddets inngang. Klemmes til MC_MAX_LEAN_ANGLE (Q_LOIT_ANG_MAX).
            targetPitchDeg = clamp((desiredFwdSpeed - fwdSpeed) * QLOITER_VEL_TO_LEAN_DEG, -MC_MAX_LEAN_ANGLE, MC_MAX_LEAN_ANGLE);
            targetBankDeg = clamp((desiredRightSpeed - rightSpeed) * QLOITER_VEL_TO_LEAN_DEG, -MC_MAX_LEAN_ANGLE, MC_MAX_LEAN_ANGLE);

            // Weathervane (se computeWeathervaneYawRateRad) - KUN aktiv i QLOITER, matcher ArduPilot:
            // "not active in QSTABILIZE and QHOVER modes as those are not position controlled modes...
            // active in QLOITER".
            const bodyForwardFull = new THREE.Vector3(0, 0, -1).applyQuaternion(q);
            const leanMagDeg = Math.sqrt(targetBankDeg * targetBankDeg + targetPitchDeg * targetPitchDeg);
            weathervaneYawRateRad = computeWeathervaneYawRateRad(bodyForwardFull, leanMagDeg);
        } else {
            // Rull er UENDRET mellom FBWA og FBWB (ArduPilot, sitert av brukeren: "Roll control is the
            // same as FBWA") - én felles linje for begge, ikke en egen fbwb-gren.
            targetBankDeg = stick.roll * THREE.MathUtils.lerp(MAX_BANK_ANGLE, MC_MAX_LEAN_ANGLE, mcAuthority);
            if (controlMode === "fbwb") {
                // FBWB (se FBWB-konstantblokken over for hele sitatet/resonnementet): elevator-pinnen
                // kommanderer en ØNSKET KLATRE-/SYNKERATE, ikke en vinkel direkte - denne ytre P-loopen
                // regner ut hvilken MÅLVINKEL (targetPitchDeg) som gir akkurat den raten, og mater den inn
                // i akkurat samme selvnivellerende vinkel-P(D)-kontrollov (rollDeflection/pitchDeflection
                // under) som FBWA/Q-modusene allerede bruker.
                const pitchStickMag = Math.abs(stick.pitch);
                let climbInput = 0;
                if (pitchStickMag > FBWB_STICK_DEADBAND) {
                    // Samme fortegnskonvensjon som FBWA sin egen quad-assist-klatrekommando (se
                    // climbInputFromPitch i den kollektive kraft-grenen lenger ned: negativ stick.pitch,
                    // dvs. pinnen trukket TILBAKE, betyr klatring) - POSITIV stick.pitch er "pinnen
                    // fremover" i denne kodebasens konvensjon.
                    climbInput = Math.sign(-stick.pitch) * (pitchStickMag - FBWB_STICK_DEADBAND) / (1 - FBWB_STICK_DEADBAND);
                }
                const climbRateCommand = climbInput * FBWB_CLIMB_RATE;
                const climbError = climbRateCommand - planeState.velocity.y;
                // Samme mcAuthority-blend som FBWA sin egen vinkelklemme - lar FBWB kommandere et brattere
                // (Q-assistert) stigningsmål ved lav fart (fortsatt under assistSpeed), akkurat som FBWA.
                const fbwbMaxPitchDeg = THREE.MathUtils.lerp(MAX_PITCH_ANGLE, MC_MAX_LEAN_ANGLE, mcAuthority);
                // I-ledd (se FBWB_PITCH_I_GAIN_PER_MS-kommentaren ved konstanten: "den mister gradvis
                // høyde i cruise" er nettopp symptomet et RENT P-ledd gir under en vedvarende
                // forstyrrelse) - akkumuleres KUN mens faktisk i FBWB (nullstilt andre steder, se
                // fbwbClimbIntegral-resetten under).
                planeState.fbwbClimbIntegral = clamp(
                    planeState.fbwbClimbIntegral + climbError * FBWB_PITCH_I_GAIN_PER_MS * dt,
                    -FBWB_PITCH_I_MAX_DEG, FBWB_PITCH_I_MAX_DEG
                );
                // FORTEGN-BUG (rapportert av brukeren: "motsatt fortegn? kommandert pitch opp gir pitch
                // ned? og høyden holdes ikke slik den skal?") - targetPitchDeg i DENNE kodebasen er POSITIV
                // for NESE NED (verifisert via QLOITER-grenen over: positiv stick.pitch -> positiv
                // desiredFwdSpeed -> "som for en multirotor krever NESE NED", som DIREKTE blir positiv
                // targetPitchDeg der). Et positivt climbError (ønsker MER klatring enn nåværende rate)
                // skal derfor gi et NEGATIVT targetPitchDeg (nese opp) - ikke positivt, slik den sto FØR
                // denne fiksen. Med feil fortegn var hele høyde-loopen i tillegg POSITIV TILBAKEKOBLING i
                // stedet for negativ: en begynnende synk (climbError>0 for å motvirke den) kommanderte MER
                // nese-ned i stedet for mindre, som forklarer "høyden holdes ikke" - selv ved sentrert
                // pinne (climbRateCommand=0) forsterket loopen enhver liten synk i stedet for å rette den
                // opp.
                targetPitchDeg = clamp(
                    -(climbError * FBWB_PITCH_GAIN_PER_MS + planeState.fbwbClimbIntegral),
                    -fbwbMaxPitchDeg, fbwbMaxPitchDeg
                );
            } else {
                // Trim er bevisst IKKE med her - dette er en selvnivellerende autopilot og skal returnere
                // til det KOMMANDERTE stigningsmålet uansett trim (auto-trim under kompenserer i stedet
                // for at trim flytter selve nivelleringsmålet).
                targetPitchDeg = stick.pitch * THREE.MathUtils.lerp(MAX_PITCH_ANGLE, MC_MAX_LEAN_ANGLE, mcAuthority);
                // Nullstill FBWB sitt I-ledd så lenge vi IKKE er i FBWB (anti-windup - unngår at et gammelt,
                // opphopet bidrag fra en TIDLIGERE FBWB-økt påvirker de aller første tickene neste gang
                // FBWB velges igjen).
                planeState.fbwbClimbIntegral = 0;
            }
        }
        // Ignorerer pinnens eget krengings-/stigningsutslag mens flyet fortsatt hviler på bena, når
        // løftemotorene faktisk har reell myndighet (samme "landed"-idé som ArduPilot sin egen
        // landingsdetektor, som sperrer krengings-/stigningskommandoer mens WoW/"weight on wheels" er
        // sann) - MÅLET er alltid null (helt flatt) mens flyet er på bakken OG løftemotorene kan
        // faktisk kommandere en attityde, uansett hva piloten kommanderer. Erstatter en tidligere
        // GROUND_ROLL_PITCH_RESISTANCE_COEFF-basert dreiemoment-klipping (nå fjernet) som viste seg å
        // ha feil effekt: den klippet BÅDE selve den korrigerende selvnivellerings-torque'en OG en
        // eventuell forstyrrelse, altså nettopp den mekanismen som skulle HOLDE flyet flatt mens det stod
        // - små avvik fikk dermed bygge seg opp helt ukorrigert mens flyet var på bakken, for så å
        // "løses ut" i et brått, ukontrollert tipp idet det endelig løftet (brukeren rapporterte at
        // VTOL-en "bikker over frem eller tilbake med en gang" rett etter avgang i QHOVER). Med MÅLET
        // tvunget til null i stedet er selvnivellerings-loopen isteden AKTIVT og KONTINUERLIG med på å
        // holde flyet flatt hele bakke-fasen (full P(D)-autoritet, ingen klipping), slik at det allerede
        // er flatt i det øyeblikket det faktisk letter - pinnen får krengings-/stigningsautoritet
        // tilbake umiddelbart idet onGround blir false.
        // BUG (rapportert av brukeren, med flightlogg: "kan fortsatt skli langs bakken med rare
        // nesestillinger" - loggen viste bank/pitch svinge vilt, opp mot ±30-34°, MENS "bakke"-kolonnen
        // sto på 1 (onGround) gjennom praktisk talt hele den 30 sekunder lange testen, i FBWB) - denne
        // klemmen sjekket FØR kun isQMode(controlMode), altså KUN QSTABILIZE/QHOVER/QLOITER/QACRO - men
        // FBWA/FBWB får AKKURAT SAMME fulle løftemotor-myndighet (mcAuthority) som en ekte Q-modus så
        // lenge luftfarten er under assistSpeed (se computeMcAuthority - IKKE mode-navnet som avgjør
        // dette, kun farten), og loggens tilfelle satt uansett FAST under assistSpeed (mcAuth=100% hele
        // veien). FBWA/FBWB sin egen elevator-/aileron-styrte målvinkel sto dermed FRITT til å kommandere
        // fulle ±MC_MAX_LEAN_ANGLE-utslag med FULL quad-torque-autoritet mens bena fortsatt sto i
        // bakkekontakt - selve mekanismen denne klemmen var ment å forhindre, bare via en modus klemmen
        // ikke gjenkjente. Byttet derfor betingelsen fra "er dette en Q-modus" til "har løftemotorene
        // faktisk reell myndighet akkurat nå" (mcAuthority>0.01, samme terskel som qloiter-grenen over
        // bruker) - den EGENTLIGE årsaken til at klemmen trengs i utgangspunktet, uavhengig av hvilket
        // modusnavn som tilfeldigvis er aktivt. FBWA/FBWB beholder fortsatt FULL aerodynamisk rorflate-
        // autoritet mens de ruller (se pitchDeflection/rollDeflection under, upåvirket av denne klemmen) -
        // kun den EKSTRA quad-torque-autoriteten dempes mens flyet står på bena, akkurat som i en Q-modus.
        if (planeState.onGround && mcAuthority > 0.01) {
            targetBankDeg = 0;
            targetPitchDeg = 0;
        }
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

        // BUG (rapportert av brukeren: "QHOVER - ved level høydehold og over til full kommandert pitch så
        // stopper pitch bevegelsen på en måte litt opp underveis noen ganger, før den fortsetter til full
        // pitch") - D-leddet over ble FØR brukt med FULL STABILIZED_BANK_/-PITCH_D_GAIN i ALLE moduser,
        // ogSÅ i ren Q-modus der mcRollTorque/mcPitchTorque (se lenger ned) ALLEREDE demper mot rå
        // vinkelhastighet sitt EGET ledd (MC_ROLL_/MC_PITCH_DAMP_GAIN). De to demperne virker på samme
        // rate, men uavhengig av hverandre - i Q-modus kan et fullt pinneutslag drive vinkelfarten opp mot
        // en likevekt på et par hundre grader/s (se MC_PITCH_TORQUE_GAIN/-DAMP_GAIN-forholdet), en rate der
        // dette D-leddet ALENE (0.02/(°/s)) blir like stort eller større enn selve P-leddets utslag - det
        // klemmer da rollDeflection/pitchDeflection tilbake mot/forbi 0 midt i en fortsatt stor
        // vinkelfeil, som bremser rotasjonen brått (dette ER "stopper litt opp") FØR filteredPitchRateDeg
        // rekker å falle nok til at P-leddet igjen dominerer og fullfører svingen mot målet. D-leddet
        // trengs fortsatt fullt ut i REN fastvinget flyging (mcAuthority=0), der aerodynamikkens egen
        // Cmq-demping er den ENESTE andre dempingskilden - blendes derfor ned mot 0 etter hvert som
        // mcAuthority stiger, i stedet for en fast verdi som dobbeltdemper Q-modusenes allerede
        // selvstendig dempede rate.
        const selfLevelDGainBlend = 1 - mcAuthority;
        rollDeflection = clamp((targetBankDeg - currentBankDeg) / STABILIZED_BANK_AUTHORITY_DEG - planeState.filteredBankRateDeg * STABILIZED_BANK_D_GAIN * selfLevelDGainBlend, -1, 1);
        pitchDeflection = clamp((targetPitchDeg - currentPitchDeg) / STABILIZED_PITCH_AUTHORITY_DEG - planeState.filteredPitchRateDeg * STABILIZED_PITCH_D_GAIN * selfLevelDGainBlend, -1, 1);
        // Ingen automatisk sideror-koordinering her - en tidligere åpen-løkke-versjon (proporsjonal med
        // kommandert krengning) fightet mot finnens ekte aerodynamiske respons (retningsstabilitet+demping)
        // og vingenes adverse yaw, som ga periodisk sideror-oscillering i svinger. Sideroret er derfor
        // rent pinnestyrt (som Manual) - koordineringen kommer naturlig fra aerodynamikken i fbwa/FW-fart.
        yawDeflection = clamp(computeRate(stick.yaw, rates.rudder) / rates.rudder.maxRate, -1, 1);

        // Auto-trim: trim-hjulet "følger etter" sakte og avlaster roret mot null utslag, akkurat som en
        // ekte koblet autopilot med trim-servo. Kjører på et LAVPASSFILTRERT utslag (autoTrimFilteredDeflection),
        // ikke det momentane pitchDeflection-tallet direkte - ellers ville trimmen jaget hvert forbigående
        // pinneutslag (f.eks. midt i en sving) i stedet for kun å ta over den VEDVARENDE delen. Gir tre
        // ting gratis: (1) selvnivellerende moduser nivellerer nøyaktig i steady state (en ren P-regulator
        // alene har et lite, fartsavhengig etterslep), (2) trim-verdien er allerede riktig innstilt idet du
        // bytter til MANUAL/QACRO, i stedet for et brått jerk ved bytte, og (3) trimmer ikke bort en
        // forbigående manøver, kun en vedvarende ubalanse.
        // BUG (rapportert av brukeren: "autotrim bør vel ikke være med i Q modusene? i manuel blir jo
        // trimmen helt ute å kjøre da") - punkt (2) over stemmer KUN når pitchDeflection faktisk
        // representerer en AERODYNAMISK trim-tilstand (FBWA/FBWB, ekte marsjflyging). I en Q-modus er
        // pitchDeflection derimot i stor grad et HOVER-/manøver-signal (kun opp til
        // Q_MODE_AERO_AUTHORITY_CEILING av det ender faktisk opp som ekte rorflate-utslag, se
        // aeroAuthority-kommentaren over - resten går til løftemotorenes mcPitchTorque) - å la autotrim
        // jage DETTE bygger opp en elevatorTrimDeg som passer for å holde et QHOVER-manøver, ikke for
        // trimmet marsjflyging. Byttet du så til MANUAL (som bruker elevatorTrimDeg DIREKTE som en fast
        // halevinkel-offset, se tailAoaDeg-bruken i manual/qacro-grenen) arvet du en helt feil, "ute å
        // kjøre" trim. Auto-trimmen oppdateres derfor nå KUN utenfor Q-moduser (isQMode) - i en Q-modus
        // fryses BÅDE filteret og selve trimverdien akkurat der de sist stod fra ekte FBWA/FBWB-flyging
        // (eller 0, om ingen har skjedd ennå), i stedet for å drive videre basert på hover-dynamikk.
        if (!isQMode(controlMode)) {
            planeState.autoTrimFilteredDeflection += (pitchDeflection - planeState.autoTrimFilteredDeflection) * Math.min(1, dt / AUTO_TRIM_FILTER_TAU);
            // Fortegn: pitchDeflection>0 betyr halen for øyeblikket dyttes mot MER nese-ned (se tailAoaDeg
            // under) - trim skal da beveges i MOTSATT retning av utslaget for å overta den samme jobben og
            // la utslaget slappe av mot null (verifisert med likevektsanalyse: trim_dot = -k*deflection er
            // den eneste av de to fortegnene som faktisk konvergerer, ikke bare tilsynelatende riktig
            // retning).
            planeState.elevatorTrimDeg = clamp(
                planeState.elevatorTrimDeg - planeState.autoTrimFilteredDeflection * AUTO_TRIM_RATE_DEG_PER_SEC * dt,
                -TRIM_RANGE_DEG, TRIM_RANGE_DEG
            );
        }
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
    // aileronLiftEffectiveness (se funksjonen over) evaluert på hver vinges EGEN, RÅ aoaDeg - IKKE
    // rørt av controlAoaDeg selv (se funksjonens egen kommentar for hvorfor det skillet er kritisk).
    const liftRight = 0.5 * AIR_DENSITY * rightWing.airspeed * rightWing.airspeed * halfWingArea * (liftCoefficient(rightWing.aoaDeg, spec) + spec.clSlope * rightWing.controlAoaDeg * aileronLiftEffectiveness(rightWing.aoaDeg, spec)) * groundEffectLiftFactor;
    const liftLeft = 0.5 * AIR_DENSITY * leftWing.airspeed * leftWing.airspeed * halfWingArea * (liftCoefficient(leftWing.aoaDeg, spec) + spec.clSlope * leftWing.controlAoaDeg * aileronLiftEffectiveness(leftWing.aoaDeg, spec)) * groundEffectLiftFactor;
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
        const liftMag = qDynWing * (liftCoefficient(baseAoaDeg, spec) + spec.clSlope * extraAoaDeg * aileronLiftEffectiveness(baseAoaDeg, spec)) * groundEffectLiftFactor;
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
    const sideslipSinSq = Math.sin(sideslipRad) * Math.sin(sideslipRad);
    const fuselageCrossflowDrag = 0.5 * AIR_DENSITY * airspeed * airspeed * fuselageSideArea * FUSELAGE_SIDE_CD * sideslipSinSq;
    // Vingens eget sideveis-drag (se WING_CROSSFLOW_CD-kommentaren) - IKKE skalert av aeroAuthority (denne
    // er en ren TRANSLASJONS-kraft, som resten av dragVec/liftVec, ikke et dreiemoment - se
    // aeroAuthority-merknaden lenger ned for hvorfor akkurat dreiemomentene måtte dempes i Q-modus, mens
    // selve luftmotstands-KREFTENE alltid har vært upåvirket av det). sideslipSinSq går mot 1 (maks drag)
    // idet vinden står tvers på nesen (ren sidevind i hover) og mot 0 (ingen ekstra drag utover vingens
    // egen normale profildrag) idet nesen peker rett inn mot vinden - akkurat den "større flate/mer
    // vindtak i sidevind"-oppførselen brukeren etterlyste.
    const wingCrossflowDrag = 0.5 * AIR_DENSITY * airspeed * airspeed * spec.wingArea * WING_CROSSFLOW_CD * sideslipSinSq;
    // bodyUpWorld flyttet hit (brukes videre nede ved liftDirWorld også, se der) - trengs allerede her av
    // bankWindExposure rett under.
    const bodyUpWorld = new THREE.Vector3(0, 1, 0).applyQuaternion(q);
    // Krengevinkelens EGEN bidrag til vindeksponering (uavhengig av sideslipSinSq over, som kun ser NESE-
    // RETNING mot vinden) - brukeren påpekte at en BANKET vinge (rull vekk fra flatt) presenterer mer av
    // sin store, flate underside/overside mot en HORISONTAL vind enn en flat, nivellert vinge gjør, uansett
    // hvilken vei nesen peker. Et flatt, nivellert vingeplan (bodyUpWorld ~ verdens-Y) står nesten
    // VINKELRETT på enhver horisontal vind (eksponerer kun kant-profilet, se WING_CROSSFLOW_CD-kommentaren)
    // - jo mer flyet krenger, jo mer dreier vingeplanets EGEN normalvektor (bodyUpWorld) inn mot den
    // horisontale vindretningen, og jo mer av selve FLATEN treffes. bankWindExposureSq (0 flatt/uberørt,
    // opp mot 1 ved fullt sideveis-eksponert flate) er derfor et EKSTRA, additivt bidrag oppå
    // sideslipSinSq - de to kombineres naturlig (mest drag når BÅDE broadside OG kraftig krenget).
    const windHorizDirWorld = new THREE.Vector3(airVelWorld.x, 0, airVelWorld.z);
    if (windHorizDirWorld.lengthSq() > 1e-6) windHorizDirWorld.normalize();
    const bankWindExposure = Math.abs(bodyUpWorld.dot(windHorizDirWorld));
    const wingBankExposureDrag = 0.5 * AIR_DENSITY * airspeed * airspeed * spec.wingArea * WING_CROSSFLOW_CD
        * bankWindExposure * bankWindExposure;
    const dragMag = qDynTotal * dragCoefficient(aoaDeg, spec, groundEffectFactor) + fuselageCrossflowDrag + wingCrossflowDrag + wingBankExposureDrag;
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
    const liftDirWorld = bodyUpWorld.clone().sub(airVelDirWorld.clone().multiplyScalar(bodyUpWorld.dot(airVelDirWorld)));
    if (liftDirWorld.lengthSq() > 1e-6) liftDirWorld.normalize(); else liftDirWorld.set(0, 1, 0);
    const dragDirWorld = airVelDirWorld.clone().multiplyScalar(-1);

    const forwardWorld = new THREE.Vector3(0, 0, -1).applyQuaternion(q);
    const thrustVec = forwardWorld.clone().multiplyScalar(thrustForce);
    const liftVec = liftDirWorld.multiplyScalar(totalLiftMag);
    const dragVec = dragDirWorld.multiplyScalar(dragMag);
    const gravityVec = new THREE.Vector3(0, -spec.mass * GRAVITY, 0);

    // Løftemotorenes samlede (kollektive) trekkraft - virker langs KROPPENS opp-akse (bodyUpWorld,
    // beregnet over), ikke world-Y direkte, akkurat som en ekte multirotor: en krenget VTOL i svevemodus
    // mister litt vertikal løft OG akselererer sidelengs, nøyaktig som den skal. liftMotorsActive (0/1,
    // se mcAuthority-kommentaren over) - motorene stopper HELT idet assistansen faller til 0 i FBWA
    // (ArduPilot: "After that time the aircraft will be flying purely as a fixed wing"), eller er alltid
    // av i MANUAL.
    let collectiveThrustMag = 0;
    if (planeState.engineOn && liftMotorsActive) {
        if (controlMode === "qhover" || controlMode === "qloiter") {
            // Alt Hold (Q_PILOT_SPD_UP/-DN/Q_P_POSZ_P-lignende) - sentrert gasspak (innenfor dødsonen)
            // holder høyden, avvik gir en ønsket klatre-/synkerate (se MC_ALT_HOLD_DEADBAND-kommentaren).
            const centered = stick.throttle - 0.5;
            const magnitude = Math.abs(centered);
            let climbInput = 0;
            if (magnitude > MC_ALT_HOLD_DEADBAND) climbInput = Math.sign(centered) * (magnitude - MC_ALT_HOLD_DEADBAND) / (0.5 - MC_ALT_HOLD_DEADBAND);
            // Asymmetrisk klatre-/synketak (se MC_MAX_SINK_RATE-kommentaren ved konstanten) - idet pinnen
            // kommanderer NEDOVER (climbInput<0) brukes det STØRRE synketaket, ikke det samme, mindre
            // klataretaket som oppover.
            const climbRateCommand = climbInput >= 0 ? climbInput * MC_MAX_CLIMB_RATE : climbInput * MC_MAX_SINK_RATE;
            const climbError = climbRateCommand - planeState.velocity.y;
            collectiveThrustMag = clamp(spec.mass * GRAVITY + spec.mass * MC_ALT_GAIN_PER_KG * climbError, 0, spec.liftThrustTotal);
        } else if (controlMode === "fbwa" || controlMode === "fbwb") {
            // Delt med FBWB (samme lavnivå quad-assist-mekanisme uansett HVILKEN fastvinget modus som
            // trigger den - kun FBWB sin egen AERODYNAMISKE elevator-målvinkel, se targetPitchDeg-grenen
            // over, bruker en annen, klatreratebasert kontrollov enn denne). ArduPilot (sitert direkte av
            // brukeren): "When you use the pitch stick (elevator) that will
            // affect the climb rate of the quad motors. If you pull back on the elevator the quad motors
            // will assist with the aircraft climb. If you push forward on the pitch stick the power to the
            // quad motors will decrease and the aircraft will descend." - stigningspinnen styrer altså EKTE
            // quad-klatrerate direkte, IKKE bare en ren rate-demping uavhengig av pinnen (se BUG under).
            // Ved stick.pitch=0 er dette identisk med den GAMLE formelen (climbError=-velocity.y) - "hold
            // nåværende høyde" ved nøytral pinne er dermed UENDRET oppførsel, kun aktiv klatre-/synke-
            // kommando via pinnen er nytt.
            // BUG (rapportert av brukeren: "QRTL cruise klatrer fortsatt mens den flyr tilbake, selv om den
            // er langt over RTL høyde") - den gamle, rene rate-dempingen (climbError=-velocity.y, uansett
            // stick.pitch) konvergerer KUN mot null RATE, aldri mot noen bestemt høyde: en allerede
            // etablert klatrerate (f.eks. fra en tidligere Q-modus-klatring) ble dermed FASTHOLDT, ikke
            // dempet mot noen referanse, så lenge løftemotor-myndigheten forble høy - noe QRTL sin
            // bremsesone (lav fart nær hjem, se simulator-vtol-rtl.js) nå kan gjøre langvarig. Siden
            // updateRtlAutopilot allerede setter stick.pitch fra et EKTE høydeavvik mot rtlAltM i cruise-
            // fasen (se der), gir denne fiksen den kommandoen et FAKTISK grep på klatreraten igjen -
            // akkurat som en ekte pilot/QRTL-kontroller ville brukt elevator til å style høyden mens
            // løftemotorene fortsatt assisterer.
            // FORTEGN-RETTELSE (rapportert av brukeren: "over RTL høyden ... klatrer den videre") - POSITIV
            // stick.pitch er i DENNE kodebasen "trekk pinnen FREMOVER" (se QLOITER: stick.pitch>0 ->
            // desiredFwdSpeed>0 -> forover bevegelse, som for en multirotor krever NESE NED), altså DYKK -
            // motsatt av fastvinget "pull back = climb"-intuisjonen jeg opprinnelig antok denne kommentaren
            // (og RTL-cruisens egen høyde-P-regulator, se updateRtlAutopilot) skulle følge. NEGERT her slik
            // at POSITIV stick.pitch (dykk) reduserer kollektiv (synk), NEGATIV (klatre) øker den - matcher
            // nå både den siterte ArduPilot-teksten OG denne kodebasens egen, allerede etablerte fortegn.
            const climbInputFromPitch = clamp(-stick.pitch, -1, 1);
            const climbError = climbInputFromPitch * MC_MAX_CLIMB_RATE - planeState.velocity.y;
            collectiveThrustMag = clamp(spec.mass * GRAVITY + spec.mass * MC_ALT_GAIN_PER_KG * climbError, 0, spec.liftThrustTotal);
        } else {
            // qstabilize/qacro: direkte, manuell kollektiv gass (ArduPilot: "the pilot's throttle input
            // controls the average motor speed... constant adjustment of the throttle is required to
            // maintain altitude").
            collectiveThrustMag = throttleShaped * spec.liftThrustTotal;
        }
    }
    planeState.lastCollectiveFrac = collectiveThrustMag / spec.liftThrustTotal;
    const liftMotorThrustVec = bodyUpWorld.clone().multiplyScalar(collectiveThrustMag);

    const accel = new THREE.Vector3().add(thrustVec).add(liftVec).add(dragVec).add(gravityVec).add(liftMotorThrustVec).multiplyScalar(1 / spec.mass);

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
        // BUG (rapportert av brukeren: "flyet sklir langs bakken i lav hastighet. selv uten vind eller
        // throtle") - kun SIDEVEIS (over) hadde denne FØR-integrering "kanseller-hele-den-lille-kraften"-
        // behandlingen; forover/bakover hadde KUN resolveGroundContact sin egen per-punkt Coulomb-friksjon
        // (se der), som riktig nok motstår EKSISTERENDE fart, men ikke kan hindre at en liten, VEDVARENDE
        // forover-kraft (f.eks. fra en ørliten utilsiktet tilt i kollektiv løftemotor-trekkraft i en
        // Q-modus, se liftMotorThrustVec/bodyUpWorld over - trekkraften virker langs kroppens EGEN
        // opp-akse, som aldri er MATEMATISK perfekt loddrett) sakte akselererer flyet fra ro, tick for
        // tick, siden en REN kinetisk (fart-reaktiv) friksjon per definisjon er null akkurat i det
        // øyeblikket farten faktisk ER null. Et ekte fast understell (i motsetning til et hjul) har
        // STATISK friksjon (ofte HØYERE enn den glidende/kinetiske verdien) som holder flyet HELT i ro mot
        // nettopp denne typen små, vedvarende krefter, i stedet for å bare bremse en fart som allerede har
        // fått lov å etablere seg - samme prinsipp/struktur som sidekraft-kanselleringen over, nå også
        // langs FOROVER-aksen.
        const forwardWorldGround = new THREE.Vector3(0, 0, -1).applyQuaternion(q);
        const forwardAccel = accel.dot(forwardWorldGround);
        const maxStaticForwardAccel = GROUND_SKID_FRICTION_COEFF * GRAVITY;
        if (Math.abs(forwardAccel) <= maxStaticForwardAccel) {
            accel.addScaledVector(forwardWorldGround, -forwardAccel);
        } else {
            accel.addScaledVector(forwardWorldGround, -Math.sign(forwardAccel) * maxStaticForwardAccel);
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

    // Løftemotorenes dreiemoment (roll/pitch/yaw) - se MC_ROLL_TORQUE_GAIN-kommentaren for hele
    // "torque = kommando*autoritet - demping*rate"-strukturen (samme idiom som fastvinge-rorflatenes
    // egen kontroll+demping over, men UAVHENGIG av qDynControl - se der for hvorfor). rollDeflection/
    // pitchDeflection GJENBRUKES direkte som ønsket kommando i BEGGE effektor-kanaler (rorflater OG
    // løftemotorer) - de representerer allerede "hvor mye/hvilken retning" piloten/autopiloten ønsker,
    // uavhengig av HVILKEN fysisk mekanisme som til slutt leverer momentet. Yaw er en ren
    // RATE-kommando (se MC_MAX_YAW_RATE_DEG-kommentaren) + et eventuelt weathervane-bidrag
    // (weathervaneYawRateRad, kun satt i QLOITER - se over), summert til én mål-girrate.
    // NEGERT fortegn på selve deflection-leddet i alle tre akser: rorflatenes AERODYNAMISKE moment over
    // (rollTorqueNoDamp="-rollDeflection*...", tailTorqueAtPitchRate/finTorqueAtYawRate="-arm*lift" der
    // lift OKER med hhv. +pitchDeflection/+yawDeflection) er ALLE negativt koblet til sin egen
    // deflection-variabel - løftemotor-momentet brukte tidligere POSITIVT fortegn på samme variabel, altså
    // stikk MOTSATT av den etablerte konvensjonen. Siden aeroAuthority/mcAuthority er komplementære
    // (fader glatt mellom de to effektor-kanalene), betydde dette at selvnivellerings-loopens P-ledd
    // (rollDeflection/pitchDeflection, beregnet for å DRIVE currentBankDeg/-PitchDeg MOT målet via den
    // aerodynamiske konvensjonen) i realiteten fikk løftemotorene til å dreie flyet i STIKK MOTSATT
    // retning i en Q-modus (der KUN løftemotor-momentet er aktivt, aeroAuthority=0) - en ekte, udempet
    // POSITIV tilbakekobling. Dette var den reelle årsaken til at flyet "tilter over av seg selv" selv
    // med målet tvunget til null (se targetBankDeg/-PitchDeg-kommentaren over) - loopen dyttet det stadig
    // LENGER unna null, ikke tilbake mot det - og til at auto-trimmen "løp av sted" (elevatorTrimDeg
    // jaget et pitchDeflection-avvik som aldri konvergerte, siden selve mekanismen som skulle rette det
    // opp virket baklengs).
    const mcRollTorque = (-rollDeflection * MC_ROLL_TORQUE_GAIN - planeState.angularVelocity.roll * MC_ROLL_DAMP_GAIN) * spec.inertiaRoll * mcAuthority;
    const mcPitchTorque = (-pitchDeflection * MC_PITCH_TORQUE_GAIN - planeState.angularVelocity.pitch * MC_PITCH_DAMP_GAIN) * spec.inertiaPitch * mcAuthority;
    const desiredMcYawRateRad = -yawDeflection * THREE.MathUtils.degToRad(MC_MAX_YAW_RATE_DEG) + weathervaneYawRateRad;
    const mcYawTorque = (desiredMcYawRateRad - planeState.angularVelocity.yaw) * MC_YAW_RATE_GAIN * spec.inertiaYaw * mcAuthority;

    // aeroAuthority (komplementet til mcAuthority) skalerer ned de AERODYNAMISKE (vinge/hale/finne)
    // dreiemomentene i takt med at løftemotorene tar over - IKKE bare additivt, slik en ren "legg til
    // MC-moment"-modell ville gjort. Nødvendig fordi vinge-/hale-modellen (bygget for FORWARD flight)
    // regner AoA fra HELE luftfarten, inkludert ren VERTIKAL klatring/synk (localAirVel.y) - i ren
    // hover (nesten null forover-fart, men opptil flere m/s klatrefart i QHOVER) gir dette en AoA nær
    // ±90° (en aerodynamisk degenerert, i praksis meningsløs vinkel som modellen aldri var tunet for),
    // som kunne gi et stort, ukommandert stigemoment rett etter avgang i Q-modus - brukeren rapporterte
    // dette som "VTOL-en begynner å tilte framover helt av seg selv, må sette den i acro for å stoppe"
    // (QACRO omgår problemet KUN fordi det hopper over selvnivellerings-loopen, ikke fordi det fjerner
    // selve den feilaktige aero-momentet - denne fiksen fjerner selve KILDEN). Fysisk begrunnet: i ekte
    // hover sitter hale/finne i grov, vertikal propellstrøm/turbulens fra løftemotorene, IKKE en ren,
    // sammenhengende horisontal luftstrøm - de bidrar ikke med meningsfullt dreiemoment der uansett.
    // BUG (rapportert av brukeren, med flightlogg fra QSTABILIZE ved 21 m/s luftfart, pinneR fastholdt på
    // -1.00 i flere sekunder: "urealistisk lav roll autoritet selv om balanserorene gir utslag? Gjelder
    // kanskje andre Q moduser også?") - aeroAuthority var FØR rett og slett komplementet til mcAuthority
    // (1-mcAuthority). I FBWA/FBWB er det riktig NOK (mcAuthority FADER faktisk mot 0 med økende fart der,
    // se computeMcAuthority) - men i EN Q-MODUS er mcAuthority ALLTID nøyaktig 1 (ArduPilot: "the quad
    // motors will immediately engage", uansett fart) - 1-mcAuthority LÅSTE dermed aeroAuthority til
    // nøyaktig 0 for alltid i enhver Q-modus, UANSETT hvor fort flyet faktisk fløy. Rorflatene beveget seg
    // synlig (rollDeflection/pitchDeflection er felles for alle moduser, se selvnivellerings-loopen over),
    // men bidro aldri med noe REELT dreiemoment i en Q-modus - en ekte QuadPlane har derimot BEGGE
    // systemer aktive SAMTIDIG når den har luftfart OG løftemotorene er på; de to er ikke gjensidig
    // utelukkende slik "1-mcAuthority" antok. aeroAuthority er derfor nå en HELT EGEN rampe basert på
    // FAKTISK FOROVER-luftfart (forwardAirspeedIntoProp, beregnet lenger opp til pusher-trekkraften) - IKKE
    // total airspeed, og IKKE mcAuthority i det hele tatt. FOROVER-komponenten spesifikt (ikke total) er
    // bevisst valgt: den ORIGINALE degenererte-AoA-bugen denne komplement-formelen opprinnelig løste (se
    // "VTOL-en begynner å tilte framover helt av seg selv"-historikken i git) oppsto NETTOPP fra en RÅ
    // VERTIKAL klatre-/synkefart i nær-hover (forover-fart ~0), som ga en degenerert AoA nær ±90° i vinge-/
    // hale-modellen - forwardAirspeedIntoProp forblir ~0 i AKKURAT den situasjonen (siden den eksplisitt
    // ekskluderer vertikal-/sidefart), så denne fiksen introduserer IKKE den gamle bugen på nytt. Skalert
    // mot samme assistSpeed-terskel som resten av overgangslogikken (Q_ASSIST_SPEED) allerede bruker.
    // I EN Q-MODUS (der mcAuthority ALLTID er 1, se over) klemmes rampen i tillegg til
    // Q_MODE_AERO_AUTHORITY_CEILING - se den egne BUG-kommentaren ved konstanten for hvorfor: uten dette
    // taket kunne aeroAuthority OGSÅ nå 1.0 ved nok fart, og legge en HEL, uavkortet ekstra
    // momentkilde OVENPÅ den allerede fulle quad-autoriteten i stedet for bare å avlaste/supplere den.
    const aeroAuthority = clamp(forwardAirspeedIntoProp / Math.max(0.1, vtolParams.assistSpeed), 0, 1)
        * (isQMode(controlMode) ? Q_MODE_AERO_AUTHORITY_CEILING : 1);
    // GIR (yaw) får sin EGEN autoritet, med et gulv i Q-modus - i MOTSETNING TIL rull/stigning over. Den
    // degenererte AoA-feilen over oppstår fra VERTIKAL fart (localAirVel.y) som dominerer over forover-
    // fart i rull-/stigningsmodellen - et modellsvakhet, ikke reell fysikk. Girmodellen (finTorqueAtYawRate)
    // sin egen degenererte vinkel oppstår derimot fra LATERAL fart (kryssvind) som dominerer over forover-
    // fart, men DER er den store, ukommanderte girvinkelen faktisk den ØNSKEDE, ekte oppførselen: en
    // ordentlig hale-/finneflate på et VTOL som dette VIL naturlig værhane inn mot vinden når den ikke
    // aktivt motvirkes, selv i hover - brukeren påpekte nettopp dette ("ustabilisert vil jo vinden
    // naturlig prøve å dytte nesa inn i vinden"). Gulvet er en forenkling av at halepartiet (bak vingen/
    // løftemotorene) uansett sitter noe friere fra selve propellstrømmen enn rull-/stigningsflatene rett
    // under rotorene - derfor ikke FULL autoritet (1.0) som i ren fastvinget flukt, kun et gulv.
    const yawAeroAuthority = isQMode(controlMode) ? Math.max(aeroAuthority, YAW_AERO_MIN_AUTHORITY_QMODE) : aeroAuthority;

    planeState.angularVelocity.roll += ((rollTorqueNoDamp * aeroAuthority + mcRollTorque) / spec.inertiaRoll) * dt;
    planeState.angularVelocity.pitch += ((pitchTorqueF0 * aeroAuthority + mcPitchTorque) / spec.inertiaPitch) * dt;
    planeState.angularVelocity.yaw += ((yawTorqueF0 * yawAeroAuthority + mcYawTorque) / spec.inertiaYaw) * dt;

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
    // Flightlogg (se js/simulator-vtol-flightlog.js) - kalt HELT til slutt slik at onGround/crashed er
    // ferdig oppdatert for denne ticken før den samples.
    logFlightSample(dt);
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
const GROUND_CONTACT_POINT_COUNT = 8;
const _groundContactLocalPts = Array.from({ length: GROUND_CONTACT_POINT_COUNT }, function () { return new THREE.Vector3(); });
const _groundContactWorldPts = Array.from({ length: GROUND_CONTACT_POINT_COUNT }, function () { return new THREE.Vector3(); });
const _groundContactEuler = new THREE.Euler();
const _groundContactForceScratch = new THREE.Vector3();
const _groundContactOffsetScratch = new THREE.Vector3();
const _groundContactTorqueAccum = new THREE.Vector3();
const _groundContactInvQuat = new THREE.Quaternion();
// NYE scratch-objekter for per-punkt Coulomb-friksjon (se BUG-kommentaren ved friksjons-loopen under) -
// samme "gjenbruk i stedet for ny allokering hver tick"-begrunnelse som resten av gruppen.
const _groundContactOmegaWorld = new THREE.Vector3();
const _groundContactPointVel = new THREE.Vector3();
const _groundContactCrossScratch = new THREE.Vector3();
const _groundContactLinearAccum = new THREE.Vector3();
// Fjærstivhet (N per kg per meter penetrasjon) for det EKTE per-punkt reaksjonsmomentet - se bruken i
// resolveGroundContact. Tunet til å gi en akselerasjon i SAMME størrelsesorden som MC_ROLL_TORQUE_GAIN
// sin egen (~10-15 rad/s² ved full kommandert avbøyning), IKKE en fysisk korrekt fjærstivhet - inertiaRoll/
// -Pitch/-Yaw er (som resten av filens dreiemoment-formler) tunede spillverdier, ikke ekte kg·m²-treghet,
// så en "fysisk riktig" fjærkonstant ville gitt en helt urealistisk voldsom respons (se utregningen i
// kommentaren ved bruken). Økt fra 25 til 35 - brukeren gjentok ønsket om en tydelig, merkbar
// velteeffekt ved en skjev/ujevn landing, ikke bare en teoretisk mulighet. Fortsatt uten live-testing -
// juster videre om landinger føles for slappe/voldsomme.
const GROUND_CONTACT_SPRING_GAIN = 35;

// Kalt fra BEGGE krasj-utløserne i resolveGroundContact under (synkefart OG bank/pitch) - se
// planeState.crashed sin egen "return"-gren tidlig i stepPhysics, som deretter bare lar farten dø
// gradvis ut via PASSIVE_ANGULAR_DAMPING mens resolveGroundContact fortsetter å kjøre hver tick. Med KUN
// den jevne dempingen kunne en solid rullefart RETT VED selve krasjøyeblikket (f.eks. fra en hard, skjev
// landing der én vingetupp/ett ben treffer først - se den per-punkt-baserte friksjonen/fjærkraften under)
// fortsette å rulle svært lenge, ekstra "matet" av vingetuppenes egne fjærkraft-sprett hver gang de traff
// bakken på nytt underveis i rullingen - brukeren rapporterte dette direkte ("vingene blir som et hjul.
// ruller og ruller"). triggerCrash() kjøres KUN på selve OVERGANGEN til krasjet tilstand (se
// !planeState.crashed-vakten under) og gir et ENGANGS, kraftig energitap - en forenklet, men fysisk
// rimelig historie om at vingen som traff hardest RETT OG SLETT KNEKKER AV i selve sammenstøtet (brukerens
// eget forslag: "Kan vi knekke av vingen som tar hardt nedi og bremse flyet? Kun ved krasj melding da for
// å holde det enkelt") - en strukturell brist absorberer bevegelsesenergi omtrent momentant, i motsetning
// til en jevn, vedvarende demping. Bevisst IKKE en full, kontinuerlig simulert løs vingedel (som ønsket,
// "for å holde det enkelt") - kun dette ENE engangsutslaget pluss en fast, varig visuell "hengende"
// vingetupp etterpå (se brokenWingSide-bruken i updatePlaneVisual).
function triggerCrash() {
    if (planeState.crashed) return;
    planeState.crashed = true;
    // Hvilken side traff hardest: sammenligner de to vingetupp-kontaktpunktenes EGEN gjennomtrengning
    // akkurat NÅ (samme _groundContactWorldPts-array/indekser 4/5 som friksjons-/fjærkraftløkken lenger
    // ned i resolveGroundContact selv bruker for vingetuppene) - dypest (mest negativ Y) er siden som
    // traff hardest. Begge kallstedene under kjører ETTER at world-punktene er fylt denne tick, så
    // arrayen er alltid gyldig her.
    planeState.brokenWingSide = _groundContactWorldPts[4].y <= _groundContactWorldPts[5].y ? 1 : -1;
    // Halepropellen (pusher) - brukeren: "kanskje propellene knekker også hvis de tar nedi ved krasj".
    // ALLTID (ikke betinget av hvilken side/vinkel) - i motsetning til vingetuppene, som kun brekker på
    // den siden som faktisk traff hardest, sitter pusher-propellen ALENE midt bak på halen (se
    // pusherGroup i buildPlane), rett bak/over halefinnen/bukfinnen - enhver hard nok landing/krasj til å
    // utløse triggerCrash() i det hele tatt (hardt synk ELLER en for bratt bank-/pitch-attityde ved
    // bakkekontakt) involverer så godt som alltid at denne enslige, lavtsittende propellen også treffer,
    // så en enkel, ubetinget "den brekker også" er en rimelig forenkling ("for å holde det enkelt").
    planeState.brokenPusherProp = true;
    planeState.angularVelocity.pitch *= CRASH_ENERGY_LOSS_FRAC;
    planeState.angularVelocity.roll *= CRASH_ENERGY_LOSS_FRAC;
    planeState.angularVelocity.yaw *= CRASH_ENERGY_LOSS_FRAC;
    planeState.velocity.multiplyScalar(CRASH_ENERGY_LOSS_FRAC);
}

// Understellet er TRE "ben" (se referansebildene brukeren la ved) - to landingsben på det FREMRE
// motorparet (ett per side) pluss bukfinnen som et tredje, bakre ben - IKKE hjul, og IKKE ben på det
// bakre motorparet. Én kombinert gjennomtrengnings-sjekk over ALLE punktene i samme tick unngår at
// uavhengige per-punkt-korreksjoner motarbeider hverandre (samme prinsipp som det opprinnelige
// hjul-/nesehjul-understellet hadde).
function resolveGroundContact(dt) {
    const spec = currentPlaneSpec();
    // Verdensrom-ekvivalentene av buildPlane sin boomHalfLen/legX/tailFrontZ+ventralFinChord/2 (se
    // kommentarene der) - MÅ holdes i synk med den visuelle modellen, akkurat som gearTrack/tailTipZ
    // gjorde for det gamle hjulunderstellet.
    const boomHalfLenWorld = spec.armLen * 0.5;
    const boomCenterZWorld = BOOM_CENTER_Z_BUILD * spec.visualScale;
    const frontZWorld = boomCenterZWorld - boomHalfLenWorld;
    // Selve BENAS bunnpunkt - rett under bom-/motorposisjonen i X (GEAR_BOOM_X_FRAC, se buildPlane som
    // bruker nøyaktig samme boomXPos for den visuelle beinplasseringen - ikke vinklet sideveis), men i Z
    // interpolert TILBAKE fra fremre motor mot bom-/vingefestets senter (GEAR_LEG_BOOM_Z_FRAC, se
    // kommentaren der og buildPlane sin legAttachZ - MÅ bruke nøyaktig samme formel).
    const legXWorld = spec.wingSpan * GEAR_BOOM_X_FRAC;
    const legAttachZWorld = THREE.MathUtils.lerp(frontZWorld, boomCenterZWorld, GEAR_LEG_BOOM_Z_FRAC);
    const legZWorld = legAttachZWorld - GEAR_LEG_FORWARD_LEAN_BUILD * spec.visualScale;
    const wingChordWorld = spec.wingArea / spec.wingSpan;
    const tailChordWorld = wingChordWorld * 0.7;
    const tailFrontZWorld = FUSELAGE_LENGTH_BUILD * spec.visualScale / 2 - tailChordWorld;
    const tailSkidZWorld = tailFrontZWorld + tailChordWorld * 0.5 * 0.5;
    _groundContactLocalPts[0].set(legXWorld, spec.gearOffsetY, legZWorld);
    _groundContactLocalPts[1].set(-legXWorld, spec.gearOffsetY, legZWorld);
    _groundContactLocalPts[2].set(0, spec.gearOffsetY, tailSkidZWorld);
    // 4. punkt (NYTT): nesetuppen - BUG (rapportert av brukeren, med skjermbilde: "FBWA lar flyet seg
    // balansere på nese i ro. det er også mulig å kjøre fremover langs bakken med nesa 45 grader ned i
    // bakken") - ingen av de tre bena/bukfinnen over ligger i nærheten av selve nesa (de sitter alle ved
    // bom-/hale-partiet), så en nesetung vinkel kunne tidligere la nesa penetrere bakkeplanet UTEN at
    // NOEN av de sporede punktene registrerte det - maxPenetration under forble negativ (alle FIRE gamle
    // punkter fortsatt "i lufta"), så onGround ble ALDRI satt, og ingen fjærkraft/friksjon virket i det
    // hele tatt. Flyet så dermed ut til å "hvile" nesetungt på bakken mens det i virkeligheten bare
    // hang/svevde helt fritt (holdt oppe av løftemotorenes Alt Hold, se MR-assist i skjermbildet) med
    // skrogmeshet visuelt overlappende bakkeplanet - IKKE en fysisk kontakt i det hele tatt. Radius/lengde
    // er DELT med buildPlane sin faktiske nesekjegle-geometri (NOSE_LEN_RATIO/NOSE_TIP_RADIUS_RATIO, se
    // konstantene øverst i filen) - samme "aldri la det visuelle og fysiske drifte fra hverandre"-prinsipp
    // som resten av understellet.
    const noseLenWorld = FUSELAGE_LENGTH_BUILD * NOSE_LEN_RATIO * spec.visualScale;
    const cabinLenWorld = FUSELAGE_LENGTH_BUILD * CABIN_LEN_RATIO * spec.visualScale;
    const noseTipZWorld = -(cabinLenWorld / 2 + noseLenWorld);
    const noseTipRadiusWorld = CABIN_RADIUS_BUILD * NOSE_TIP_RADIUS_RATIO * spec.visualScale;
    _groundContactLocalPts[3].set(0, -noseTipRadiusWorld, noseTipZWorld);
    // 5.-6. punkt (NYTT): vingetuppene - BUG (rapportert av brukeren: "vingetuppen skal ikke glitche
    // gjennom bakken. men ta nedi og simuleres de også") - ingen av de andre punktene ligger i nærheten av
    // vingespennet, så en hard/skjev landing med krengning kunne tidligere la en vingetupp visuelt
    // penetrere bakken helt uten fysisk reaksjon. wingMountYWorld/wingRootZWorld MÅ matche buildPlane sin
    // faktiske vingeplassering (WING_MOUNT_HEIGHT_RATIO delt via CABIN_RADIUS_BUILD, samme "0.02"
    // Z-forskyvning som wing.position der) - X er allerede ekte verdensrom (spec.wingSpan/2, samme
    // konvensjon som legXWorld over).
    const wingMountYWorld = CABIN_RADIUS_BUILD * WING_MOUNT_HEIGHT_RATIO * spec.visualScale;
    const wingRootZWorld = 0.02 * spec.visualScale;
    _groundContactLocalPts[4].set(spec.wingSpan / 2, wingMountYWorld, wingRootZWorld);
    _groundContactLocalPts[5].set(-spec.wingSpan / 2, wingMountYWorld, wingRootZWorld);
    // Brukket, hengende vingetupp (se triggerCrash()/brokenWingSide og BROKEN_WING_DROOP_RAD i
    // updatePlaneVisual) - BUG (rapportert av brukeren: "kollisjon og friksjon må detekteres med
    // vingetuppen mot bakken også") - den visuelle aileron-hengselen droopet ned til en fast, overdrevet
    // vinkel ved brudd, MEN kontaktpunktet over lå fortsatt fast ved den ORIGINALE, uknekte
    // vingehøyden - en brukket, hengende vingetupp kunne dermed synke synlig NED I bakken uten at
    // fysikken noensinne registrerte kontakt der, siden den fortsatt bare sjekket det gamle,
    // høyereliggende punktet. Senker kontaktpunktet på DEN skadde siden med en enkel
    // sin(droop)*wingChordWorld-tilnærming (wingChordWorld som en representativ lengde for hvor langt
    // rorflaten stikker ut - "for å holde det enkelt", ikke en eksakt geometrisk utledning av selve
    // hengselplasseringen/spennet).
    if (planeState.crashed && planeState.brokenWingSide !== 0) {
        const droopDropY = wingChordWorld * Math.sin(BROKEN_WING_DROOP_RAD);
        if (planeState.brokenWingSide > 0) _groundContactLocalPts[4].y -= droopDropY;
        else _groundContactLocalPts[5].y -= droopDropY;
    }
    // 7. punkt (NYTT): halespissen - BUG (rapportert av brukeren: "halen glitcher ned gjennom rullebanen
    // når man krasher og havner med halen ned") - tailSkidZWorld (punkt 2, brukt av bena/bukfinnen over)
    // sitter et stykke FORAN selve halekjeglens faktiske spiss (den er beregnet fra V-halens korde, ikke
    // skrogets bakerste punkt) - i en BRATT hale-ned/nese-opp attityde (f.eks. rett etter et krasj som
    // ender opp med halen ned, se CRASH_PITCH_DEG-fiksen) er det den EKTE, bakerste spissen (der
    // pusher-propellen er montert, se pusherMount/tailTipZ i buildPlane) som blir skrogets faktisk
    // LAVESTE punkt - ikke tailSkidZWorld. checkTailStrike lenger ned beregner nøyaktig dette punktet
    // allerede (samme tailTipZ/tailTipRadius-formel), MEN den funksjonen gir kun en visuell ADVARSEL -
    // ingen fysisk reaksjon i det hele tatt. Lagt til her som et EKTE, sjette kontaktpunkt (delt formel
    // med checkTailStrike, samme "aldri la punktene drifte fra hverandre"-prinsipp som resten av
    // understellet).
    const tailTipZWorld = (cabinLenWorld / 2 + FUSELAGE_LENGTH_BUILD * TAIL_LEN_RATIO * spec.visualScale);
    const tailTipRadiusWorld = CABIN_RADIUS_BUILD * TAIL_TIP_RADIUS_RATIO * spec.visualScale;
    _groundContactLocalPts[6].set(0, -tailTipRadiusWorld, tailTipZWorld);
    // 8. punkt: skrogbuken (kabinseksjonens underside, midt på skroget) - punktene over er IKKE
    // nødvendigvis skrogets faktisk LAVESTE punkt ved en brå/ekstrem stigning-/krengevinkel (f.eks. rett
    // etter en for hard rotasjon i Q-modus) - uten dette kunne selve skrogbuken svinge ned GJENNOM
    // bakkeplanet mens de andre kontaktpunktene (plassert lenger ut mot vingene/halen/nesa) fortsatt
    // teknisk sto klar, som brukeren rapporterte tidligere ("kroppen glitcher gjennom bakken"). Fortsatt
    // et rent sikkerhetsnett (IKKE med i fjærkraft-/friksjons-loopen under, se kommentaren der).
    _groundContactLocalPts[7].set(0, -CABIN_RADIUS_BUILD * spec.visualScale, BOOM_CENTER_Z_BUILD * spec.visualScale);
    let maxPenetration = -Infinity;
    for (let i = 0; i < GROUND_CONTACT_POINT_COUNT; i++) {
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

    // Synkefart (selve BERØRINGS-/anslagsstyrken) sjekkes KUN på selve landingsøyeblikket (den stigende
    // flanken fra luft til bakke) - gir ingen mening å re-sjekke kontinuerlig mens flyet allerede står i
    // ro (velocity.y er da uansett ~0). Krengevinkel/stigningsvinkel derimot (se rett under) sjekkes
    // UBETINGET, IKKE bare på selve landingsflanken - se BUG-kommentaren der.
    if (!planeState.onGround && planeState.velocity.y < -CRASH_SINK_RATE) {
        triggerCrash();
    }
    // BUG (rapportert av brukeren, med skjermbilde: flyet lå med nesa dypt begravd i bakken OG krenget,
    // men fortsatte likevel å akselerere fremover for FULL gass i FBWA i stedet for å telle som en
    // ødelagt/krasjet landing) - CRASH_BANK_DEG/-PITCH_DEG-sjekken sto FØR kun i "!planeState.onGround"-
    // grenen over, altså KUN idet flyet akkurat landet. Et fly som ALLEREDE stod på bakken (onGround
    // forble sann kontinuerlig) og deretter ble presset inn i en for ekstrem attityde (f.eks. av selve
    // nesetupp-fjærkraften over, eller pinneutslag mens det sto stille) rakk ALDRI gjennom en ny
    // luft->bakke-flanke, og ble derfor ALDRI sjekket på nytt - flyet kunne dermed sitte fast i en
    // fysisk umulig attityde på ubestemt tid, fortsatt fullt flyvbart (motor/gass upåvirket). Flyttet
    // derfor UT av flanke-sjekken - evaluert HVER tick flyet er på bakken (uansett om det nettopp landet
    // eller har stått der en stund), slik at en for bratt attityde krasjer i samme øyeblikk den oppstår,
    // ikke bare idet den oppstår VED en fersk landing.
    _groundContactEuler.setFromQuaternion(planeState.quaternion, "YXZ");
    const bankDeg = Math.abs(-THREE.MathUtils.radToDeg(_groundContactEuler.z));
    // Samme euler-uttrekk/fortegnskonvensjon som currentPitchDeg i stepPhysics (-radToDeg(euler.x)), men
    // her er kun MAGNITUDEN relevant (nese-ned OG nese-opp er begge en hard/ødeleggende attityde).
    const pitchDeg = Math.abs(-THREE.MathUtils.radToDeg(_groundContactEuler.x));
    if (bankDeg > CRASH_BANK_DEG || pitchDeg > CRASH_PITCH_DEG) {
        triggerCrash();
    }
    planeState.onGround = true;
    planeState.position.y += maxPenetration;
    if (planeState.velocity.y < 0) planeState.velocity.y *= 0.15;

    // EKTE per-punkt fjærkraft- OG friksjonsbasert reaksjon - erstatter en tidligere "kunstig, alltid-
    // nivellér-krengning"-slerp som ubetinget slerpet banken mot 0 uansett HVORDAN flyet faktisk landet.
    // Den slerpen hindret aktivt flyet i noensinne å tippe over ved en ujevn/skjev horisontal landing
    // (brukeren spurte eksplisitt: "får kraftig friksjon på bena. kanskje vil den tippe over?" - med kun
    // ETT globalt korreksjonspunkt og en garantert nivellering var svaret hardkodet til "aldri", uansett
    // hvor skjevt landingen faktisk skjedde). Hvert PENETRERENDE punkt (KUN de syv ekte kontaktpunktene
    // 0-6 - bena/bukfinnen/nesetuppen/vingetuppene/halespissen - IKKE sikkerhetsnett-punktet på skrogbuken,
    // punkt 7, som ikke skal bidra med en egen fysisk reaksjon) dytter nå tilbake med en kraft proporsjonal med sin
    // EGEN penetrasjonsdybde (som en fjær), akkurat der DET punktet faktisk er i verdensrom - via et EKTE
    // dreiemoment (r×F, transformert til kroppsrom via quaternion-invertering - samme "vektor i stedet for
    // håndplukket fortegn"-prinsipp som lift-/drag-retningene i stepPhysics bruker, "robust mot
    // fortegnsfeil", i stedet for en enkelt global korreksjon). En jevn, flat landing gir naturlig like
    // store krefter på alle punkter (null netto moment - flyet blir liggende flatt av seg selv, akkurat
    // som før), mens en skjev landing (kun ett ben/vingetupp/nesetupp treffer først) nå gir et EKTE
    // rettende ELLER veltende moment avhengig av hvor ille skjevheten faktisk er - fysikken avgjør, ikke en
    // hardkodet garanti.
    //
    // BUG/ønske (rapportert av brukeren: "mer realistisk friksjon og fysikk i kontakt med bakken? nær et
    // landingsben tar nedi i full fart vil det jo bremse kraftig og kanskje tippe flyet eller trekke
    // kraftig en retning? vingetuppen skal ikke glitche gjennom bakken. men ta nedi og simuleres de også")
    // - den GAMLE friksjonen (fjernet, se git-historikk) virket på HELE flyets GJENNOMSNITTSFART, helt
    // uavhengig av HVILKE punkter som faktisk var i bakken, og ga ALDRI noe dreiemoment i det hele tatt -
    // ett ben (eller nå en vingetupp) som tar i bakken alene kunne dermed umulig bremse/svinge/tippe flyet
    // slik brukeren ba om, kun en jevn, retningsløs oppbremsing av HELE flyet uansett skjevhet. Friksjonen
    // under er i stedet EKTE Coulomb PER PUNKT - proporsjonal med DETTE punktets EGEN normalkraft
    // (fjærkraften rett over, IKKE hele flyets vekt) og motsetter DETTE punktets EGEN verdensrom-fart
    // (CG-fart PLUSS rotasjonsbidraget ω×r - et punkt langt fra CG, som en vingetupp under en rulling eller
    // et ben under en gir-rotasjon, kan ha en helt annen fart enn CG selv om CG-farten er lav). Virker
    // derfor naturlig BÅDE som lineær oppbremsing (akkumulert i _groundContactLinearAccum, lagt til
    // planeState.velocity ETTER loopen) OG - via r×F inn i SAMME dreiemoment-akkumulator som fjærkraften -
    // som et EKTE gir-/rull-/stigningsmoment: ett ben som tar i bakken alene i full fart bremser/svinger nå
    // flyet akkurat DEN veien (et ekte "ground loop"-potensial), og en vingetupp som tar i bakken under en
    // krengning gir et ekte, fysisk motstående moment i stedet for å gli fritt gjennom.
    _groundContactOmegaWorld.set(planeState.angularVelocity.pitch, planeState.angularVelocity.yaw, planeState.angularVelocity.roll)
        .applyQuaternion(planeState.quaternion);
    _groundContactTorqueAccum.set(0, 0, 0);
    _groundContactLinearAccum.set(0, 0, 0);
    // BUG (rapportert av brukeren, med skjermbilde: flyet ble hengende diagonalt i lufta etter et krasj,
    // med halen tydelig hevet over bakken - "sklir litt lengre nå. men blir fortsatt hengende i lufta") -
    // løkkegrensen sto fortsatt på 6 (indeks 0-5: bena/bukfinnen/nesetuppen/vingetuppene) fra FØR
    // halespiss-kontaktpunktet (indeks 6, se _groundContactLocalPts[6]-oppsettet over) ble lagt til. Selve
    // halespissen var dermed KUN med i maxPenetration-sjekken (nok til å hindre den fra å synke synlig NED
    // I bakken) og bank/pitch-krasjsjekken, men ga ALDRI sin egen fjærkraft/friksjon/dreiemoment tilbake -
    // et fly som skulle hvilt flatt på f.eks. nese+hale kunne dermed finne en falsk likevekt der KUN nesa
    // (eller et ben/en vingetupp) faktisk dyttet tilbake, mens halepartiet forble "hengende" understøttet av
    // ingenting. Utvidet til 7 (indeks 0-6) - halespissen deltar nå i akkurat samme fjærkraft-/
    // friksjonsreaksjon som resten av punktene, se _groundContactLocalPts[6] og bevisst IKKE
    // punkt 7 (skrogbuken, fortsatt kun sikkerhetsnett).
    for (let i = 0; i < 7; i++) {
        // Brukket vingetupp (se triggerCrash()/brokenWingSide) - IKKE lenger et kontaktpunkt i det hele
        // tatt. BUG (rapportert av brukeren, med skjermbilde: "i noen krasj ruller flyet unaturlig lenge
        // rundt. vingetippen fungerer som hjul... vingen som slås i bakken må jo knekke av") - roten til
        // "hjul"-oppførselen er STRUKTURELL, ikke bare svak friksjon: en REN rotasjon rundt et fast
        // kontaktpunkt (rulling UTEN glidning, akkurat som et ekte hjul) har per definisjon NULL
        // kontaktpunkt-hastighet - og dermed NULL Coulomb-friksjonskraft, uansett hvor høy
        // friksjonskoeffisienten er (se pointSpeed-bruken under - en glidnings-basert friksjonsmodell kan
        // aldri stanse en ekte "rull uten å gli"-rotasjon, akkurat slik et virkelig hjul ikke TRENGER
        // friksjon for å fortsette å spinne). En EKTE vinge er derimot ikke et glatt, sirkulært hjul - den
        // ville revet seg løs/deformert strukturelt og dermed FYSISK MISTET selve dreiepunktet nesten
        // momentant, ikke fortsatt rotert pent rundt et perfekt, uendret pivot-punkt. Fjerner derfor selve
        // KONTAKTPUNKTET på den brukne siden helt (i stedet for bare å style friksjonen der) - akkurat som
        // brukeren selv foreslo ("vingen ... må jo knekke av") - flyet mister rett og slett
        // vingespennet/dreiearmen som holdt rullingen i gang, og synker/glir i stedet naturlig ned mot en
        // flatere, mer stabil hvilestilling (buken/nesen/halen/det gjenværende benet/vingetuppen) styrt av
        // de RESTERENDE kontaktpunktene og flyets egen vekt - den "vekt og balanse"-oppførselen brukeren
        // etterlyste, uten å måtte bygge en egen separat masse-/balansemodell.
        if (planeState.crashed && ((i === 4 && planeState.brokenWingSide > 0) || (i === 5 && planeState.brokenWingSide < 0))) continue;
        const pen = -_groundContactWorldPts[i].y;
        if (pen <= 0) continue;
        // Vingetupp-krasj (se WING_STRIKE_CRASH_PEN_M-kommentaren ved konstanten) - punkt 4/5 ER
        // vingetuppene (se _groundContactLocalPts-oppsettet over). Sjekket FØR selve fjærkraft-/
        // friksjonsreaksjonen under, slik at et krasj utløst her fortsatt rekker å bli behandlet av
        // triggerCrash() (og dermed CRASH_FRICTION_MULTIPLIER-oppbremsingen) i AKKURAT denne samme tick.
        if ((i === 4 || i === 5) && pen > WING_STRIKE_CRASH_PEN_M && planeState.velocity.length() > WING_STRIKE_CRASH_SPEED_MS) {
            triggerCrash();
        }
        _groundContactOffsetScratch.copy(_groundContactWorldPts[i]).sub(planeState.position);

        // Vertikal fjærkraft (uendret prinsipp fra før) - selve REAKSJONS-kraften som holder flyet oppe.
        const normalForceMag = pen * spec.mass * GROUND_CONTACT_SPRING_GAIN;
        _groundContactForceScratch.set(0, normalForceMag, 0);
        _groundContactCrossScratch.copy(_groundContactOffsetScratch).cross(_groundContactForceScratch);
        _groundContactTorqueAccum.add(_groundContactCrossScratch);

        // BUG (rapportert av brukeren, med flightlogg: "skjer fortsatt. når jeg lander med bare litt
        // horisontal hastighet. sklir og sklir i evigheten" - loggen viste luftfarten synke fra 2.3 til
        // 1.4 m/s over 46 SEKUNDER, en bremsing på under 0.02 m/s² - i praksis null) - ROTEN var at
        // penetrasjonsdybden (pen over) IKKE er et pålitelig mål på faktisk normalkraft i DENNE
        // fjærmodellen: "planeState.position.y += maxPenetration" NULLSTILLER penetrasjonen hver eneste
        // tick, så "pen" ved likevekt (flyet i ro, rett før neste korreksjon) er bare den MIKROSKOPISKE
        // gjeninntrengningen ett enkelt 1/120s-tidssteg med tyngdekraft rekker å gi (~0.3 mm) - IKKE en
        // reell, vedvarende fjærkompresjon som balanserer flyets fulle vekt (slik en ekte fjær/demper UTEN
        // en slik posisjons-snap ville gjort). normalForceMag (og dermed Coulomb-friksjonstaket,
        // GROUND_SKID_FRICTION_COEFF*normalForceMag) ble dermed også mikroskopisk i ro - mange
        // størrelsesordener for lite til å bremse selv en sakte glidning, uansett μ. Egen, friksjons-
        // SPESIFIKK normalkraft-referanse under (frictionNormalForceMag) - IKKE brukt til selve
        // fjærkraften/-momentet over, kun til friksjonstaket - med et gulv på en rimelig andel av flyets
        // faktiske vekt, slik at et punkt som faktisk berører bakken (pen>0, uansett hvor grunt) alltid har
        // NOK friksjonskapasitet til faktisk å stanse en treg glidning, akkurat som et ekte, vektbærende
        // ben ville hatt. Fjærkraften i seg selv (normalForceMag) kan fortsatt DOMINERE gulvet ved en
        // reell, hard/rask nedslag-penetrasjon (pen momentant mye større enn likevekts-mikroskopet), som
        // gir enda kraftigere bremsing akkurat da - se GROUND_CONTACT_SPRING_GAIN-kommentaren.
        const frictionNormalForceMag = Math.max(normalForceMag, spec.mass * GRAVITY * 0.5);

        // Per-punkt Coulomb-friksjon (horisontal - se pointVel.y=0 under, motstår GLIDNING langs
        // bakkeplanet, ikke selve gjennomtrengningen fjæra over allerede håndterer).
        _groundContactPointVel.copy(_groundContactOmegaWorld).cross(_groundContactOffsetScratch).add(planeState.velocity);
        _groundContactPointVel.y = 0;
        const pointSpeed = _groundContactPointVel.length();
        if (pointSpeed > 1e-4) {
            // CRASH_FRICTION_MULTIPLIER (se konstanten) - et skadet, gravende/gnagende skrog har mye
            // høyere friksjon enn et intakt understell, se BUG-kommentaren ved konstanten.
            const crashFrictionScale = planeState.crashed ? CRASH_FRICTION_MULTIPLIER : 1;
            const maxFrictionForceMag = GROUND_SKID_FRICTION_COEFF * crashFrictionScale * frictionNormalForceMag;
            // Klemt til den kraften som AKKURAT ville stanset DETTE punktets EGEN fart i løpet av ETT
            // tidssteg (samme "aldri overskyt/snu fortegn"-prinsipp den gamle globale friksjonen hadde,
            // nå anvendt per punkt i stedet for på hele flyets gjennomsnittsfart).
            const stoppingForceMag = pointSpeed * spec.mass / dt;
            const frictionForceMag = Math.min(maxFrictionForceMag, stoppingForceMag);
            _groundContactForceScratch.copy(_groundContactPointVel).multiplyScalar(-frictionForceMag / pointSpeed);
            _groundContactCrossScratch.copy(_groundContactOffsetScratch).cross(_groundContactForceScratch);
            _groundContactTorqueAccum.add(_groundContactCrossScratch);
            _groundContactLinearAccum.addScaledVector(_groundContactForceScratch, dt / spec.mass);
        }
    }
    planeState.velocity.add(_groundContactLinearAccum);
    _groundContactInvQuat.copy(planeState.quaternion).invert();
    _groundContactTorqueAccum.applyQuaternion(_groundContactInvQuat);
    // pitch=X, yaw=Y, roll=Z i kroppsrom - samme konvensjon som angVelVec i stepPhysics (se
    // integrateOrientation-kallet der), IKKE en ny, egen håndplukket akse-tilordning her.
    planeState.angularVelocity.pitch += (_groundContactTorqueAccum.x / spec.inertiaPitch) * dt;
    planeState.angularVelocity.yaw += (_groundContactTorqueAccum.y / spec.inertiaYaw) * dt;
    planeState.angularVelocity.roll += (_groundContactTorqueAccum.z / spec.inertiaRoll) * dt;
    // Ekstra, generell gir-demping mot underlaget (UAVHENGIG av om noe punkt faktisk glir horisontalt akkurat
    // nå) - uten denne ville en gir-rotasjon i prinsippet fortsette lenge i stillstand med motoren av, siden
    // det aerodynamiske dempeleddet (som skalerer med fart²) blir null når flyet står stille, og den nye
    // per-punkt-friksjonen over kun virker mens et punkt FAKTISK har en egen, ikke-null horisontalfart.
    // Eksponentiell, tidssteg-uavhengig demping (samme prinsipp som rollDampDecay/yawDampDecay i
    // stepPhysics).
    planeState.angularVelocity.yaw *= Math.exp(-GROUND_YAW_FRICTION * dt);
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
    planeState.brokenWingSide = 0;
    planeState.brokenPusherProp = false;
    planeState.onGround = true;
    planeState.hasBeenAirborne = false;
    planeState.elevatorTrimDeg = 0;
    // Sentrert (0.5), ikke i bunn - se inputState-kommentaren over for begrunnelsen (gjelder likt her,
    // siden R-tasten kan resette midt i en flytur der flightMode fortsatt er en Q-modus).
    inputState.stick.throttle = 0.5;
    resetVtolState();
    // resetPlane setter alltid onGround=true rett over - et restart-øyeblikk på bakken, derfor
    // ubetinget (se ensureGroundStartMode-kommentaren for hvorfor).
    ensureGroundStartMode();
    // "Motor PÅ" ER arming-øyeblikket i denne simmen (se captureHome-kommentaren i
    // js/simulator-vtol-rtl.js) - resetPlane tvinger alltid engineOn=true, så QRTL sitt hjem-punkt skal
    // alltid fanges på nytt her også, ikke bare i setEngine/toggleEngine.
    captureHome();
}

function toggleEngine() {
    // BUG (rapportert av brukeren: "og må fortsatt være mulig å styre/stoppe motorene. for treningen sin
    // del") - denne funksjonen returnerte FØR ubetinget med det samme flyet var krasjet, og LÅSTE dermed
    // engineOn-bryteren fast i whatever tilstand den hadde idet krasjet inntraff - piloten kunne verken
    // kutte eller restarte motorene etterpå. For et treningsverktøy er nettopp "kutt motorene umiddelbart
    // etter et krasj/en hard landing" en reell, viktig prosedyre å kunne øve på - denne skal ALDRI være
    // utilgjengelig. Ingen krasjet-vakt her lenger: stepPhysics() sin egen "if (planeState.crashed) return"
    // (se toppen av funksjonen) sørger uansett for at engineOn ikke lenger har NOEN fysisk effekt (gass/
    // rotorer) mens flyet er krasjet - denne bryteren styrer nå kun selve AV/PÅ-tilstanden/HUD-en, ikke om
    // det fortsatt er trygt eller mulig å fly videre.
    const wasOff = !planeState.engineOn;
    planeState.engineOn = !planeState.engineOn;
    // KUN på bakken (se onGround-vilkåret) - en motorrestart i LUFTA (brukertilbakemelding: "nytt RTL
    // punkt settes når motorene restartes i lufta. det skal ikke være mulig. kun lov å sette nytt RTL
    // punkt etter at dronen har landet trygt") skal ALDRI flytte hjem-punktet til det midlertidige
    // luft-punktet - QRTL sitt eksisterende hjem (satt sist gang flyet trygt sto på bakken) beholdes
    // uendret gjennom en slik restart, akkurat som ekte ArduPilot ("updated as long as the autopilot is
    // disarmed" - dvs. ALDRI mens den er armert/i lufta). Samme onGround-vilkår tvinger nå ALLTID en
    // Q-modus ved et bakke-restart også (se ensureGroundStartMode).
    if (wasOff && planeState.engineOn && planeState.onGround) {
        ensureGroundStartMode();
        captureHome();
    }
}
// Diskré av/på (i motsetning til toggleEngine over, som fortsatt brukes av K-tasten/HUD-knappen) - til
// gamepad-knappekartet, se BUTTON_ACTIONS/engineOn/engineOff under. Brukeren påpekte at ÉN knapp for
// av/på (en toggle) er dårlig gamepad-ergonomi: uten å se HUD-en vet piloten ikke hvilken tilstand
// motoren faktisk er i, så samme knapp kan enten starte ELLER stoppe motoren avhengig av gjeldende
// tilstand - to separate, diskré knapper (én som ALLTID slår PÅ, én som ALLTID slår AV) er entydige
// uansett gjeldende tilstand, akkurat som et ekte fly har separate start-/stopp-prosedyrer.
function setEngine(on) {
    // Se BUG-kommentaren i toggleEngine over - samme krasjet-vakt fjernet, samme begrunnelse (piloten må
    // fortsatt kunne kutte/restarte motorene etter et krasj, for treningens skyld).
    // Stigende flanke (AV->PÅ) = QRTL sitt hjem-/batterifangst-øyeblikk, se captureHome i
    // js/simulator-vtol-rtl.js - fanges FØR selve tilstandsendringen under (ren lesbarhet, ingen
    // funksjonell forskjell siden captureHome ikke leser engineOn selv). KUN på bakken - se
    // onGround-vilkåret og kommentaren i toggleEngine over for hvorfor (en restart i lufta skal ALDRI
    // flytte hjem-punktet).
    const wasOff = !planeState.engineOn;
    planeState.engineOn = on;
    if (wasOff && on && planeState.onGround) {
        ensureGroundStartMode();
        captureHome();
    }
}

function toggleCamera() {
    cameraModeIndex = (cameraModeIndex + 1) % CAMERA_MODES.length;
    const mode = CAMERA_MODES[cameraModeIndex];
    activeCamera = (mode === "chase") ? chaseCamera : (mode === "fpv") ? fpvCamera : vlosCamera;
}

/* ---------- Visuell oppdatering + HUD ---------- */
const SURFACE_MAX_DEFLECTION_RAD = THREE.MathUtils.degToRad(22);
// Krasj-skade, rent visuelt (se triggerCrash()-kommentaren for hele bakgrunnen/brukerønsket) - EN fast,
// overdrevet vinkel per skadet del, godt utenfor normalt bevegelsesspenn, så den tydelig leser som
// "brukket/hengende" og ikke en vanlig kontrollflate-/propellbevegelse. Ikke live-testet - juster om
// retningen ser feil ut visuelt.
const BROKEN_WING_DROOP_RAD = THREE.MathUtils.degToRad(75);
const BROKEN_PROP_BEND_RAD = THREE.MathUtils.degToRad(50);

function updatePlaneVisual(dt) {
    planeGroup.position.copy(planeState.position);
    planeGroup.quaternion.copy(planeState.quaternion);

    // Propell-treghet (trekkpropell/pusher): en elektrisk motor gir nesten momentant turtallsøkning med
    // gasspådrag, men propellen (+ motorens egen rotasjonstreghet) coaster ned over ca. 1 sekund når
    // motoren kuttes - ikke et brått stopp slik det var før. Bruker lastPusherThrottle (faktisk
    // trekkmotor-pådrag, se stepPhysics - IKKE inputState.stick.throttle direkte, som i Q-modus i
    // stedet styrer løftemotorene, se lastCollectiveFrac under).
    // I en Q-modus er pusheren fysisk avslått (pusherThrottleEff=0 uansett gasspak, se stepPhysics) - den
    // skal derfor stå HELT stille (ikke engang en "tomgangs"-spinn), ikke først rulle i gang idet
    // trekkmotor-modusene (MANUAL/FBWA) tar over under overgangen til fastvinget flukt.
    // Brukket i krasj (se triggerCrash() - brokenPusherProp settes ALLTID på et krasj) - propellen skal
    // da stå HELT stille (ikke coaste videre ned over ~1s som en vanlig motorkutt) og henge synlig
    // bøyd/skjev i stedet for å fortsette å rotere pent.
    if (planeState.crashed && planeState.brokenPusherProp) {
        propSpinSpeed = 0;
        planePropeller.rotation.x = BROKEN_PROP_BEND_RAD;
    } else {
        // BUG (rapportert av brukeren etter en krasj+reset): rotation.x ble KUN satt i den brukne grenen
        // over, aldri nullstilt her - propellen ble derfor stående bøyd for alltid etter en krasj, selv
        // etter reset (planeState.crashed/brokenPusherProp nullstilles i resetPlane(), men selve
        // mesh-rotasjonen ble aldri fulgt opp). Satt eksplisitt hver tick i normal drift i stedet for å
        // stole på en engangs-reset et annet sted.
        planePropeller.rotation.x = 0;
        const pusherIdleSpin = isQMode(planeState.lastControlMode) ? 0 : 6;
        const targetSpin = planeState.engineOn ? (pusherIdleSpin + planeState.lastPusherThrottle * 90) : (planeState.onGround ? 0 : 2);
        const spinSmoothing = planeState.engineOn ? (1 - Math.pow(0.0005, dt)) : (1 - Math.pow(0.05, dt));
        propSpinSpeed += (targetSpin - propSpinSpeed) * spinSmoothing;
        planePropeller.rotation.z += propSpinSpeed * dt;
    }

    // Løftemotorene (fire stk, samme treghetsprinsipp) - FELLES grunnfart styrt av samlet kollektiv gass
    // (lastCollectiveFrac), pluss et PER-MOTOR differensial-tillegg fra kommandert rull/stigning
    // (lastRollDeflection/lastPitchDeflection) - uten dette så det ut som flyet tiltet "av seg selv" uten
    // at noen motor synlig endret turtall (brukeren rapporterte nettopp dette). Rent kosmetisk - fysikkens
    // egen rull-/stigningsmoment (mcRollTorque/mcPitchTorque i stepPhysics) er en abstrakt "samlet"
    // modell, ikke fire uavhengige motor-krefter - denne mixeren later bare AS OM det er fire uavhengige
    // motorer, med samme fortegn-konvensjon som det korrigerte mcRollTorque/mcPitchTorque (se
    // kommentaren der): side*(-rollDeflection) og (front?1:-1)*(-pitchDeflection). Girmomentets egne
    // motsatt-roterende par (Cnp/reactive torque) er fortsatt ikke modellert visuelt.
    // liftMotorsActive-terskelen (0.001) er DEN SAMME som stepPhysics selv bruker til å avgjøre om
    // løftemotorene fysisk får noe kollektiv trekkraft i det hele tatt (se liftMotorsActive-bruken der) -
    // gjenbrukt her via lastMcAuthority (satt hver fysikk-tick, se computeMcAuthority-kallet) i stedet for
    // bare engineOn alene. FØR denne fiksen fikk løftepropellene en "tomgangs"-grunnfart (6 rad/s) så lenge
    // motoren i det hele tatt var PÅ, ogSÅ i ren FBWA-marsjflyging langt over assistSpeed der
    // løftemotorene fysisk står HELT stille (se mcAuthority-kommentaren lenger opp: "motorene stopper HELT
    // idet assistansen faller til 0 i FBWA") - propellene så dermed ut til å snurre svakt selv når
    // MR-assistansen (HUD-en) viste 0%, noe brukeren rapporterte direkte ("quad propellene går litt rundt
    // selv om MR-assist står på 0%. de skal vel stå helt i ro da?").
    const liftAuthorityActive = planeState.lastMcAuthority > 0.001;
    const liftTargetSpin = (planeState.engineOn && liftAuthorityActive) ? (6 + planeState.lastCollectiveFrac * 140) : 0;
    const liftSpinSmoothing = (planeState.engineOn && liftAuthorityActive) ? (1 - Math.pow(0.0005, dt)) : (1 - Math.pow(0.05, dt));
    liftPropSpinSpeed += (liftTargetSpin - liftPropSpinSpeed) * liftSpinSmoothing;
    // BUG (rapportert av brukeren: "quad propellene spinner noe selv om MR-assist skal være 0%. gjelder
    // FBWB også") - liftRollMix/-PitchMix ble FØR dette regnet ut fra lastRollDeflection/-PitchDeflection
    // UANSETT liftAuthorityActive. I FBWA/FBWB er aileron/elevator-utslagene (den AERODYNAMISKE
    // selvnivelleringen) i konstant, normal bruk langt over assistSpeed - diff ble dermed ofte ulik null
    // for minst én av de fire motorene selv lenge etter at liftPropSpinSpeed (selve grunnfarten) hadde
    // falmet helt til 0, siden spin=max(0, 0+diff) fortsatt kunne bli positiv. Nullet nå ut SAMTIDIG med
    // grunnfarten (samme liftAuthorityActive-vilkår) - differensial-mixeren later som fire uavhengige
    // motorer for et kollektiv som faktisk ER aktivt, ikke for restutslag på rorflater som uansett ikke
    // har noen løftemotor å mikse ut på.
    const liftRollMix = liftAuthorityActive ? -planeState.lastRollDeflection * LIFT_PROP_DIFFERENTIAL_GAIN : 0;
    const liftPitchMix = liftAuthorityActive ? -planeState.lastPitchDeflection * LIFT_PROP_DIFFERENTIAL_GAIN : 0;
    planeLiftProps.forEach(function (m) {
        // Brukket i krasj (se triggerCrash()/brokenWingSide - BEGGE løftemotorene på bommen som traff
        // hardest, ikke bare vingetuppens egen aileron) - stopp helt og heng synlig bøyd, samme prinsipp
        // som pusher-propellen over.
        if (planeState.crashed && m.side === planeState.brokenWingSide) {
            m.group.rotation.x = BROKEN_PROP_BEND_RAD;
            return;
        }
        // BUG (rapportert av brukeren: "de høyre quad propellene er montert skjevt nå? noe som blir
        // hengende igjen etter en krasj og reset kanskje?") - samme rotation.x-nullstillings-bug som
        // pusher-propellen over: grenen over satte rotation.x, men INGEN gren nullstilte den igjen etter
        // at planeState.crashed gikk tilbake til false (resetPlane() nullstiller kun selve
        // tilstands-flaggene, ikke denne mesh-rotasjonen) - løftemotorene på siden som sist var brukket
        // sto derfor værende synlig skjeve for alltid, selv etter et rent reset.
        m.group.rotation.x = 0;
        const diff = m.side * liftRollMix + (m.front ? 1 : -1) * liftPitchMix;
        const spin = Math.max(0, liftPropSpinSpeed + diff);
        if (spin > 0.05) {
            m.group.rotation.y += spin * dt;
        } else {
            // Stanset (spin ~0): gli mot nærmeste stilling der bladene peker langs kroppens fram/bak-akse
            // i stedet for å fryse i en vilkårlig vinkel der spinnet tilfeldigvis stoppet - brukeren ba
            // eksplisitt om dette ("helst rette seg inn pekende med spiss mot fartsretningen"), akkurat som
            // et ekte, felt/fjæret stoppet rotorblad minimerer luftmotstand ved å legge seg langs
            // luftstrømmen. Bladparet (dir=+1/-1 i buildPlane) ligger langs GRUPPENS lokale ±X ved
            // rotation.y=0/π (vingespenn-retning) og langs ±Z (fram/bak, fartsretningen) ved rotation.y=
            // ±π/2 - målvinkelen er derfor nærmeste ODDETALLS multiplum av π/2, ikke av π selv.
            const nearestAligned = Math.round((m.group.rotation.y - Math.PI / 2) / Math.PI) * Math.PI + Math.PI / 2;
            m.group.rotation.y += (nearestAligned - m.group.rotation.y) * Math.min(1, dt * 4);
        }
    });

    // Synlige, bevegelige rorflater - viser faktisk pinne-/autopilot-utslag (se stepPhysics, som lagrer
    // siste avbøyning på planeState hver fysikk-tick).
    // (Fortegn snudd - rorene beveget seg motsatt vei av forventet.)
    // Brukket vingetupp i krasj (se triggerCrash()/brokenWingSide) - gjenbruker aileron-hengselet (samme
    // pivot som normal rullstyring under) i stedet for et helt nytt vinge-rot-hengsel, "for å holde det
    // enkelt" som brukeren selv ba om. Kun DEN skadde siden fryses i en fast, overdrevet "hengende"
    // vinkel - den andre (uskadde) siden flyr videre helt normalt.
    planeAileronLeft.rotation.x = (planeState.crashed && planeState.brokenWingSide < 0)
        ? BROKEN_WING_DROOP_RAD : planeState.lastRollDeflection * SURFACE_MAX_DEFLECTION_RAD;
    planeAileronRight.rotation.x = (planeState.crashed && planeState.brokenWingSide > 0)
        ? BROKEN_WING_DROOP_RAD : -planeState.lastRollDeflection * SURFACE_MAX_DEFLECTION_RAD;
    // V-halens to ruddervatorer: rent visuell mix av stigning+gir (se buildPlane-kommentaren ved
    // vtailPivotBySide - fysikken bruker en egen, abstrakt aerodynamisk modell, uavhengig av denne
    // geometrien) - hver pivot ligger allerede i sin egen VIPPEDE lokale ramme, så EN rotasjon her gir
    // riktig kombinert utslag i verdensrom helt av seg selv - MEN hengselaksen må være rotation.Y, IKKE
    // rotation.x: buildWingProfileMesh sitt verticalFin=true-spor (se kommentaren der) legger spennet
    // langs lokal Y og TYKKELSEN (panelets egen flate-normal) langs lokal X for en vertikalt orientert
    // flate som denne - å rotere om X vrir da panelet om sin EGEN flate-normal (usynlig/nesten usynlig
    // for en tynn, flat plate), i stedet for å hengsle det om spennaksen slik en ekte rorflate skal (se
    // den allerede riktige fastvinge-sideroret, som nettopp derfor bruker rotation.Y - samme flate-
    // orientering, samme aksekonvensjon). Dette var den reelle årsaken til at rorene "ikke beveget seg".
    //
    // Hvilken av de to (felles-modus vs. differensial-modus mellom venstre/høyre) som faktisk gir PITCH
    // vs. GIR i verdensrom, er IKKE intuitivt for en tilted (V-vinklet) hengselakse - måtte regnes ut for
    // hånd (rotasjon om hver panels EGEN, allerede Rz(-side*VTAIL_ANGLE_DEG)-tiltede lokale Y-akse, projisert
    // til verdensrom): felles-modus (samme fortegn på begge sider) gir et bakkant-utslag som er SPEIL-
    // symmetrisk i world-Y (stigning kansellerer, sideveis/gir-komponenten adderer) - altså et GIR-utslag,
    // IKKE stigning. Differensial-modus (motsatt fortegn) gir omvendt: sideveis-komponenten kansellerer,
    // stigningskomponenten adderer - altså PITCH. Den opprinnelige koden hadde disse BYTTET OM
    // (pitchCmd som felles-modus, yawCmd som differensial-modus) - brukeren rapporterte nettopp dette
    // ("ser ut som yaw styrer høyde og omvendt"). Rettet: yawCmd er nå felles-modus, pitchCmd differensial.
    const pitchCmd = planeState.lastPitchDeflection, yawCmd = planeState.lastYawDeflection;
    planeVtailLeft.rotation.y = (yawCmd - pitchCmd) * SURFACE_MAX_DEFLECTION_RAD * 0.6;
    planeVtailRight.rotation.y = (yawCmd + pitchCmd) * SURFACE_MAX_DEFLECTION_RAD * 0.6;
}

const hudMode = document.getElementById("hudMode");
const hudAssist = document.getElementById("hudAssist");
const hudThrottleLabel = document.getElementById("hudThrottleLabel");
const hudArmed = document.getElementById("hudArmed");
const hudInput = document.getElementById("hudInput");
const hudCamera = document.getElementById("hudCamera");
const hudPlaneClass = document.getElementById("hudPlaneClass");
const hudAltitude = document.getElementById("hudAltitude");
const hudAirspeed = document.getElementById("hudAirspeed");
const hudGroundSpeed = document.getElementById("hudGroundSpeed");
const hudThrottle = document.getElementById("hudThrottle");
const hudTrim = document.getElementById("hudTrim");
const armToggleBtn = document.getElementById("armToggleBtn");
const crashBanner = document.getElementById("crashBanner");
const tailstrikeBanner = document.getElementById("tailstrikeBanner");
const modeBlockedBanner = document.getElementById("modeBlockedBanner");
const modeBlockedReasonEl = document.getElementById("modeBlockedReason");
const trimInputEl = document.getElementById("trimInput");
const trimValueEl = document.getElementById("trimValue");

function updateHud() {
    hudMode.textContent = MODE_LABELS[planeState.flightMode];
    // Kort gult blink ved modusbytte (se modeFlashUntil-kommentaren ved trySetFlightMode) - className
    // (ikke bare textContent) settes her HVER tick, siden updateRtlHud (kalt rett etter denne i animate())
    // kun overskriver hudMode.textContent i qrtl-fasetekst, aldri className - trygt å style uavhengig.
    hudMode.classList.toggle("mode-flash", performance.now() < modeFlashUntil);
    hudAssist.textContent = Math.round(planeState.lastMcAuthority * 100) + " %";
    const engineLabel = planeState.crashed ? "Krasjet" : (planeState.engineOn ? "På" : "Av");
    hudArmed.textContent = engineLabel;
    hudArmed.className = "sim-status-value " + ((!planeState.crashed && planeState.engineOn) ? "sim-armed" : "sim-killed");
    crashBanner.classList.toggle("show", planeState.crashed);
    tailstrikeBanner.classList.toggle("show", performance.now() < tailstrikeWarningUntil);
    modeBlockedBanner.classList.toggle("show", performance.now() < modeBlockedUntil);
    hudInput.textContent = inputState.source === "gamepad" ? "Gamepad" : "Tastatur";
    hudCamera.textContent = CAMERA_MODE_LABELS[CAMERA_MODES[cameraModeIndex]];
    hudPlaneClass.textContent = currentPlaneSpec().label.split(" ")[0];
    const altitude = Math.max(0, planeState.position.y);
    hudAltitude.textContent = (altitude >= 10 ? Math.round(altitude) : altitude.toFixed(1)) + " m";
    // IAS, ikke sann luftfart - se lastIndicatedAirspeed-utregningen i stepPhysics (posisjonsfeil fra
    // pitotrørets egen, forover-pekende monteringsakse).
    hudAirspeed.textContent = planeState.lastIndicatedAirspeed.toFixed(1) + " m/s";
    // GS (Ground Speed) - brukeren ba om en egen, GNSS-basert fartsmåling ved siden av IAS. Rent
    // posisjons-/hastighetsderivert (planeState.velocity er allerede VERDENS-hastigheten, ETT skritt
    // nærmere en ekte GNSS-mottakers egen fartsberegning enn IAS er - en GNSS regner fart fra faktisk
    // BEVEGELSE over bakken, ikke fra luftstrøm/dynamisk trykk) - derfor UTEN vind trukket fra
    // (currentWindVector inngår IKKE her, i motsetning til airVelWorld/localAirVel i stepPhysics), og KUN
    // den horisontale komponenten (X/Z) - en ekte GNSS/GPS-fartsmåling rapporterer normalt bakkefart og
    // klatre-/synkefart (VS) separat, ikke slått sammen til én 3D-magnitude slik IAS-beregningen gjør.
    const groundSpeed = Math.hypot(planeState.velocity.x, planeState.velocity.z);
    hudGroundSpeed.textContent = groundSpeed.toFixed(1) + " m/s";
    // Gasspaken styrer ULIKE ting avhengig av modus (kollektiv i Q-moduser, trekkmotor i MANUAL/FBWA) -
    // se stepPhysics - HUD-etiketten OG selve tallet følger derfor hvilken av de to som faktisk er
    // aktiv, ikke bare den rå pinneposisjonen.
    if (isQMode(planeState.lastControlMode)) {
        hudThrottleLabel.textContent = "Kollektiv";
        hudThrottle.textContent = Math.round(planeState.lastCollectiveFrac * 100) + " %";
    } else {
        hudThrottleLabel.textContent = "Gass";
        hudThrottle.textContent = Math.round(planeState.lastPusherThrottle * 100) + " %";
    }
    const trimText = (planeState.elevatorTrimDeg >= 0 ? "+" : "") + planeState.elevatorTrimDeg.toFixed(1) + "°";
    hudTrim.textContent = trimText;
    armToggleBtn.innerHTML = '<i class="fa-solid fa-power-off"></i> ' + (planeState.engineOn ? "Motor av (K)" : "Motor på (K)");

    if (trimInputEl && document.activeElement !== trimInputEl) {
        trimInputEl.value = planeState.elevatorTrimDeg;
    }
    if (trimValueEl) trimValueEl.textContent = trimText;
}

/* ---------- Modus-popover (klikk på "Modus" i HUD-en) ----------
   Samme tastatursnarveier/rekkefølge som Digit1-7-håndteringen under, og samme forklaringstekst som
   helpPanel sine egne <li>-punkter (se der) - ETT sted å oppdatere om en modus' oppførsel endres, ikke to
   separate tekster som kan gli fra hverandre. Bruker Sim.setupDropdown UENDRET (samme mekanikk som
   Settings-menyen: åpne/lukke, lukk ved klikk utenfor, lukk andre åpne dropdowns) - se
   .sim-dropdown/.sim-dropdown-menu-klassene i CSS-en, ingen ny styling trengt. */
const MODE_KEY_LABELS = {
    qstabilize: "1", qhover: "2", qloiter: "3", qacro: "4", manual: "5", fbwa: "6", fbwb: "7", qrtl: "8"
};
const MODE_DESCRIPTIONS = {
    qstabilize: "Selvnivellerende krengning/stigning, manuell (direkte) kollektiv gass.",
    qhover: "Som QSTABILIZE + Alt Hold (sentrert gasspak holder høyden).",
    qloiter: "Som QHOVER + posisjonsholding (slipp pinnen for å stoppe/holde) + weathervaning.",
    qacro: "Rate-styrt svevemodus, ingen selvnivellering (som Acro).",
    manual: "Direkte rorstyring, løftemotorene er alltid AV - kun for trim-sjekk/taksing.",
    fbwa: "Selvnivellerende fastvinget marsjflyging - løftemotorene assisterer automatisk til assist-farten nås.",
    fbwb: "Som FBWA, men elevator styrer ønsket klatre-/synkerate (ikke vinkel direkte) og gasspaken styrer ønsket luftfart - autopiloten holder høyden/farten selv.",
    qrtl: "Naviger automatisk hjem (punktet motoren sist ble slått PÅ) og land. Flyr fastvinget hvis langt unna, går over til VTOL-modus nær hjem - se RTL/failsafe-panelet."
};
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
            trySetFlightMode(mode);
            popover.classList.remove("open");
        });
        popover.appendChild(btn);
    });
    Sim.setupDropdown(hudMode, popover, ALL_PANEL_IDS.map(function (id) { return document.getElementById(id); }));
}

/* ---------- Paneler (rates / fly-kamera / vind / gamepad / hjelp) ---------- */
const ALL_PANEL_IDS = ["ratesPanel", "flyCameraPanel", "windPanel", "vtolPanel", "rtlPanel", "flightLogPanel", "gamepadPanel", "helpPanel"];
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
    updateRtlHud(); // bygger videre på (ikke erstatter) updateHud() over - se js/simulator-vtol-rtl.js
    updateFpvHud();
    renderer.render(scene, activeCamera);
}

/* ---------- Oppstart ---------- */
document.addEventListener("DOMContentLoaded", function () {
    initScene();
    initRtlHomeMarker();
    initRtlPanel();
    initFlightLogPanel();
    captureHome(); // fanger startposisjonen (motoren starter PÅ, se planeState.engineOn) som første hjem
    initFpvHudCanvas();
    document.getElementById("fpvHudBtn").innerHTML =
        '<i class="fa-solid fa-crosshairs"></i> OSD: ' + FPV_HUD_MODE_LABELS[settings.fpvHudMode] + " (O)";
    buildRatesPanel();
    buildModePopover();

    document.getElementById("resetBtn").addEventListener("click", resetPlane);
    document.getElementById("armToggleBtn").addEventListener("click", toggleEngine);

    const settingsMenuEl = document.getElementById("settingsMenu");
    Sim.setupDropdown(document.getElementById("settingsToggleBtn"), settingsMenuEl,
        ["ratesPanel", "flyCameraPanel", "windPanel", "vtolPanel", "rtlPanel", "flightLogPanel", "gamepadPanel"].map(function (id) { return document.getElementById(id); }));
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
    document.getElementById("toggleVtolBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("vtolPanel"));
        closeSettingsMenu();
    });
    document.getElementById("toggleRtlBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("rtlPanel"));
        closeSettingsMenu();
    });
    document.getElementById("toggleFlightLogBtn").addEventListener("click", function () {
        togglePanel(document.getElementById("flightLogPanel"));
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
    Object.keys(VTOL_CLASSES).forEach(function (key) {
        const opt = document.createElement("option");
        opt.value = key;
        opt.textContent = VTOL_CLASSES[key].label;
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

    const assistSpeedInput = document.getElementById("assistSpeedInput");
    const assistSpeedValue = document.getElementById("assistSpeedValue");
    const assistFadeInput = document.getElementById("assistFadeInput");
    const assistFadeValue = document.getElementById("assistFadeValue");
    const wvaneEnabledInput = document.getElementById("wvaneEnabledInput");
    const wvaneGainInput = document.getElementById("wvaneGainInput");
    const wvaneGainValue = document.getElementById("wvaneGainValue");
    const wvaneAngMinInput = document.getElementById("wvaneAngMinInput");
    const wvaneAngMinValue = document.getElementById("wvaneAngMinValue");
    function refreshVtolParamsPanel() {
        assistSpeedInput.value = vtolParams.assistSpeed;
        assistSpeedValue.textContent = vtolParams.assistSpeed + " m/s";
        assistFadeInput.value = vtolParams.assistFadeSec;
        assistFadeValue.textContent = vtolParams.assistFadeSec + " s";
        wvaneEnabledInput.checked = vtolParams.wvaneEnabled;
        wvaneGainInput.value = vtolParams.wvaneGain;
        wvaneGainValue.textContent = vtolParams.wvaneGain.toFixed(1);
        wvaneAngMinInput.value = vtolParams.wvaneAngMin;
        wvaneAngMinValue.textContent = vtolParams.wvaneAngMin.toFixed(1) + "°";
    }
    refreshVtolParamsPanel();
    assistSpeedInput.addEventListener("input", function () {
        vtolParams.assistSpeed = parseFloat(assistSpeedInput.value);
        assistSpeedValue.textContent = assistSpeedInput.value + " m/s";
        saveVtolParams();
    });
    assistFadeInput.addEventListener("input", function () {
        vtolParams.assistFadeSec = parseFloat(assistFadeInput.value);
        assistFadeValue.textContent = assistFadeInput.value + " s";
        saveVtolParams();
    });
    wvaneEnabledInput.addEventListener("change", function () {
        vtolParams.wvaneEnabled = wvaneEnabledInput.checked;
        saveVtolParams();
    });
    wvaneGainInput.addEventListener("input", function () {
        vtolParams.wvaneGain = parseFloat(wvaneGainInput.value);
        wvaneGainValue.textContent = wvaneGainInput.value;
        saveVtolParams();
    });
    wvaneAngMinInput.addEventListener("input", function () {
        vtolParams.wvaneAngMin = parseFloat(wvaneAngMinInput.value);
        wvaneAngMinValue.textContent = parseFloat(wvaneAngMinInput.value).toFixed(1) + "°";
        saveVtolParams();
    });
    document.getElementById("resetVtolParamsBtn").addEventListener("click", function () {
        Object.assign(vtolParams, DEFAULT_VTOL_PARAMS);
        saveVtolParams();
        refreshVtolParamsPanel();
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
            case "Digit1": trySetFlightMode("qstabilize"); break;
            case "Digit2": trySetFlightMode("qhover"); break;
            case "Digit3": trySetFlightMode("qloiter"); break;
            case "Digit4": trySetFlightMode("qacro"); break;
            case "Digit5": trySetFlightMode("manual"); break;
            case "Digit6": trySetFlightMode("fbwa"); break;
            case "Digit7": trySetFlightMode("fbwb"); break;
            case "Digit8": trySetFlightMode("qrtl"); break;
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
