/* js/simulator-vtol-quiz.js
   Spørsmålsbanken for ex7 "Teoriprøve" (js/simulator-vtol-exercises.js) - egen fil ("Kan quiz spørsmål og
   svaralternativ være i en egen fil for enklere redigering?", brukeren) utelukkende for å gjøre selve
   INNHOLDET raskt å redigere/lese, uten å måtte lete gjennom hele øvelses-/scenario-logikken. Lastes ETTER
   simulator-vtol-flightlog.js og FØR simulator-vtol-exercises.js (se simulator-vtol.html) - exercises.js
   bruker VTOL_QUIZ_QUESTIONS-konstanten under direkte som VTOL_EXERCISES.ex7.quizQuestions, samme
   delt-globalt-scope-mønster som resten av VTOL-filene seg imellom (se toppkommentaren i
   simulator-vtol-rtl.js for hele begrunnelsen).

   Faktasjekket mot ardupilot.org sin egen dokumentasjon (compass/kalibrering, QuadPlane-moduser,
   Q_ASSIST_SPEED/assistert luftfartsestimering, Motor Emergency Stop-varianter, pre-arm-sjekker) og mot
   Heewing-spesifikasjonene/ArduPilot-sitatene brukeren selv oppga tidligere i denne økten - IKKE gjettet
   fritt. "Pass på at gode svaralternativer, så det ikke er åpenbart hva som er riktig" (brukeren) - feil
   alternativ er bevisst holdt omtrent samme lengde/detaljnivå som det riktige, ikke åpenbart korte/urimelige
   "fyll"-svar. */
const VTOL_QUIZ_QUESTIONS = [
    {
        icon: "fa-compass",
        question: "Hva skjer typisk hvis kompasset er dårlig kalibrert og du flyr i QLOITER eller en annen GPS-avhengig modus?",
        options: [
            "Farkosten kan drifte eller sirkle ukontrollert (\"toilet bowling\"), i verste fall fly av gårde av seg selv",
            "Ingenting å bekymre seg for - kompasset styrer kun retningsvisningen på HUD-en, ikke selve flygingen",
            "Motorene nekter å starte i det hele tatt, uansett hvilken flymodus som er valgt på forhånd",
            "Batteriet tømmes merkbart raskere, fordi autopiloten da kompenserer automatisk med mer gasspådrag"
        ],
        correctIndex: 0,
        explanation: "Et unøyaktig kompass gir feil heading-referanse - alle GPS-avhengige moduser (QLOITER, AUTO, RTL) styrer da mot feil retning, som typisk vises som sirkling eller drift."
    },
    // "noe feil i svaret der?" (brukeren, med detaljert egen research om deklinasjon/lokale
    // anomalier/inklinasjon) - faktasjekket direkte mot ardupilot.org: magnetisk MISVISNING (deklinasjon)
    // slås automatisk opp fra GPS-posisjon (COMPASS_AUTODEC, standard PÅ) - ArduPilot sin egen dokumentasjon
    // er eksplisitt: "a locations magnetic declination might need to [be] manually entered... This is not
    // necessary now."
    // "Enig i at vi kan endre til dette?" (brukeren, med et konkret forslag til nytt spørsmål/svar/
    // forklaring) - spørsmålet omformulert fra "hvorfor etter flytting" til det bredere "når bør du
    // kalibrere" brukeren foreslo (dekker samme deklinasjon/lokal-interferens-poeng, PLUSS den praktiske
    // "sjekk visuelt først"-vanen). Fact-sjekket selve PÅSTANDENE i forslaget mot ardupilot.org (WebSearch/
    // WebFetch denne økten) før de ble tatt inn:
    // - BEKREFTET: "PreArm: Compass Variance"/magnetfelt-sjekken (ardupilot.org/copter/docs/common-prearm-
    //   safety-checks) er en ekte, dokumentert kalibrerings-trigger - enten kalibreringen ikke ga gode
    //   offset, ELLER farkosten står nær en stor metallisk/magnetisk forstyrrelse.
    // - IKKE FUNNET i offisiell dokumentasjon: brukerens forslag nevnte spesifikt "etter lang tids lagring
    //   (30+ dager) grunnet intern magnetisk drift" fra komponenter/skruer - et konkret, plausibelt-lydende
    //   tall/mekanisme jeg ikke fant STØTTE for i selve ardupilot.org-dokumentasjonen (kun GPS/kompass-bytte
    //   eller ny FC-plassering er eksplisitt nevnt som en ekte maskinvare-trigger). Droppet den spesifikke
    //   "30 dager/intern drift"-påstanden fra forklaringen under i stedet for å ta den inn ubekreftet - se
    //   også "Selve kalibreringen («drone-dansen») må aldri utføres nær metall/armeringsjern"-poenget
    //   brukeren la til til slutt, som ER bekreftet (PreArm magnetfelt-sjekken over) og nå flettet INN i
    //   forklaringen i stedet for lagt til som et eget, delvis overlappende ekstra avsnitt.
    {
        icon: "fa-map-location-dot",
        question: "Når er det anbefalt å utføre en ny kompasskalibrering?",
        options: [
            "Du må kalibrere kompasset fast hver 30. dag eller hver gang du flytter deg over 50 km, uavhengig av om kompassretningen og kartet på skjermen stemmer overens eller ikke",
            "Siden ArduPilot har en innebygd kartdatabase (COMPASS_AUTODEC), er det aldri nødvendig å sjekke eller kalibrere kompasset manuelt etter at farkosten har stått ubrukt over tid",
            "Kompasset skal alltid sjekkes visuelt mot faktisk retning før avgang, og kalibreres kun ved en faktisk feilmelding, en maskinvareendring eller mistanke om lokal magnetisk forstyrrelse",
            "Så lenge du kalibrerer kompasset regelmessig etter tid (hver 30. dag), trenger du ikke gjøre noen visuell sjekk av retningen på kartet før du tar av"
        ],
        correctIndex: 2,
        explanation: "Kompasset bør sjekkes visuelt (vri farkosten og se at kartretningen følger synkront med) før hver avgang - stemmer det, er kalibrering unødvendig, og faktisk en risiko: kalibrerer du unødig nær skjult armeringsjern, kjøretøy eller annet metall, bygger du de lokale forstyrrelsene rett inn i sensoren.\n\nKalibrer i stedet kun ved en faktisk PreArm-feilmelding om kompasset, etter å ha byttet eller flyttet GPS-/kompassmodulen, eller ved mistanke om en lokal magnetisk forstyrrelse - ikke bare fordi det har gått en fast tid. Misvisning (deklinasjon) endrer seg riktignok også med posisjon, men slås automatisk opp fra GPS (COMPASS_AUTODEC) - heller ikke det er en grunn til å kalibrere på nytt."
    },
    // "'navneforskjellen er bare historisk betinget' er jo helt merkelig ting å si og åpenbart feil. ha noe
    // mer plausibelt med bra språk" (brukeren) - byttet til en OMVENDT/byttet-om variant av selve det
    // korrekte svaret (QHOVER<->QLOITER sine roller speilvendt) i stedet - en langt mer plausibel felle:
    // fanger opp eleven som husker BEGGE fakta men blander sammen HVILKEN modus som gjør hva, ikke bare
    // noen som ikke leste spørsmålet. "'(du styrer posisjonen selv)' kan endres til '(du må korrigere
    // posisjon selv)'" (brukeren) - samme ordlyd i begge svaralternativene nå, for konsistens.
    {
        icon: "fa-code-compare",
        question: "Hva er hovedforskjellen mellom QHOVER og QLOITER?",
        options: [
            "QHOVER er ment kun for landing, mens QLOITER kun brukes til selve avgangen fra bakken",
            "QLOITER er egentlig en fastvinget marsjmodus, mens QHOVER er en ren VTOL-hovermodus",
            "QHOVER holder kun høyden automatisk (du må korrigere posisjon selv) - QLOITER holder BÅDE høyde og GPS-posisjon",
            "QLOITER holder kun høyden automatisk (du må korrigere posisjon selv) - QHOVER holder BÅDE høyde og GPS-posisjon"
        ],
        correctIndex: 2,
        explanation: "QHOVER gir kun Alt Hold - horisontal posisjon må du holde selv med stikkene. QLOITER legger GPS-posisjonshold oppå det, og bremser til stillstand når du slipper stikkene."
    },
    {
        icon: "fa-gauge-high",
        question: "Hva gjør ArduPilot-parameteren Q_ASSIST_SPEED?",
        options: [
            "Angir luftfarten flyet må ha i marsjflyging før overgangen til fastvinget modus regnes som fullført",
            "Bestemmer maksimal hastighet farkosten kan ha i QLOITER før den automatisk begynner å bremse",
            "Angir en luftfartsterskel der løftemotorene assisterer automatisk mot steiling hvis farten faller under den",
            "Styrer hvor fort de tiltbare motorene roterer fysisk under selve overgangsmanøveren"
        ],
        correctIndex: 2,
        explanation: "Q_ASSIST_SPEED er en sikkerhetsterskel: faller luftfarten under den i fastvinget flyging, kobler løftemotorene seg automatisk inn igjen for å hindre steiling/tap av kontroll."
    },
    {
        // Heewing T2 Cruza leveres normalt UTEN pitotrør (kun GPS/FC i PNP-pakken) - Q_ASSIST_SPEED
        // fungerer da fortsatt, men luftfarten den sammenlignes mot er en ESTIMERT verdi (GPS-fart +
        // vindestimering), ikke en fysisk målt en. Faktasjekket mot ArduPilot sin egen
        // "Assisted Fixed-Wing Flight"-dokumentasjon (airspeed-estimering uten sensor).
        icon: "fa-wind",
        question: "Heewing T2 Cruza leveres normalt UTEN fysisk luftfartsmåler (pitotrør). Hva betyr det for Q_ASSIST_SPEED i sterk medvind?",
        options: [
            "Det påvirker ikke Q_ASSIST_SPEED i det hele tatt - terskelen leses alltid fra en fysisk sensor",
            "Uten pitotrør estimeres luftfarten fra GPS-fart og vindestimering - i medvind kan estimatet se greit ut selv om reell fart er farlig lav",
            "GPS-farten blir da alltid null, så Q_ASSIST_SPEED utløses automatisk i samme øyeblikk motoren armeres",
            "Flykontrolleren nekter rett og slett å arme motorene uten en fysisk luftfartsmåler montert på flyet"
        ],
        correctIndex: 1,
        // "er vel ikke helt riktig? mer riktig med..." (brukeren, med et forslag som bl.a. nevnte TECS -
        // "Men kan kutte TECS") - omskrevet uten TECS-referansen (utenfor pensum her), men beholder
        // kjernepresiseringen brukeren etterlyste: estimatet er et "syntetisk" GPS+vind-anslag, ikke bare
        // "en feil GPS-fart", og problemet er spesifikt i RETT LINJE i medvind (der EKF-vindestimatet ikke
        // får observert forskjellen mellom GPS-fart og luftfart like godt som gjennom en sving).
        explanation: "Uten fysisk luftfartsmåler bruker ArduPilot et \"syntetisk\" luftfartsestimat som kombinerer GPS-fart med et vindestimat (best observert gjennom svinger). I rett linje i sterk medvind kan estimatet bli upresist og overvurdere farten gjennom luften. Q_ASSIST_SPEED stoler blindt på dette estimatet - assistansen risikerer da å slå inn for sent, eller ikke i det hele tatt."
    },
    {
        icon: "fa-plane",
        question: "Hva skjer med løftemotorene i MANUAL-modus (ren fastvinget flyging, ingen assistanse)?",
        options: [
            "De går alltid på lav, konstant tomgang gjennom hele flyturen for ekstra sikkerhetsmargin",
            "De er helt deaktivert - all løft og styring kommer fra vinger og rorflater, som et vanlig fly",
            "De kobler seg automatisk inn igjen dersom luftfarten skulle bli for lav under flygingen",
            "De brukes fortsatt, men da kun til å styre gir/retning, ikke til selve løftet"
        ],
        correctIndex: 1,
        // "'attityde' kan være 'attitude/nesestilling'" (brukeren) - ordet byttet med noe mer entydig norsk.
        explanation: "MANUAL er ren fastvinget flyging uten noen VTOL-assistanse i det hele tatt - løftemotorene er helt uten myndighet, uansett fart eller nesestilling (attitude)."
    },
    // "Her er det feil. Riktig svar blir vel..." (brukeren, med et konkret forslag) - den forrige "korrekte"
    // teksten var ikke direkte FEIL (den matcher fortsatt selve simulatorens Motor Emergency Stop-oppførsel,
    // se forrige versjon av denne kommentaren), men traff ikke den mest operasjonelt VIKTIGE forskjellen: AT
    // en vanlig disarm normalt ikke engang LAR SEG GJØRE midt i luften, mens nødstopp er ment å virke akkurat
    // der. Fact-sjekket brukerens forslag mot ardupilot.org (WebSearch/WebFetch denne økten) før det ble tatt
    // inn:
    // - BEKREFTET: ArmingPlane-dokumentasjonen ("held left rudder can disarm the vehicle in any mode IF THE
    //   AUTOPILOT JUDGES THAT THE VEHICLE IS NOT FLYING") - en flyr-sjekk sperrer normalt stikke-disarm i
    //   luften, nettopp for å hindre et utilsiktet feiltrykk. Nødstopp har intet slikt vilkår.
    // - BEKREFTET: hjempunktet oppdateres normalt til farkostens posisjon ved HVER arming (HOME_RESET_ALT,
    //   standard 0 = "continuously reset it" mens disarmert).
    // - IKKE BEKREFTET/droppet: brukerens forslag til forklaring nevnte at farkosten "vil etter 5 sekunder
    //   automatisk disarme på bakken" etter en nødstopp i luften - fant ingen slik 5-sekunders-regel i
    //   dokumentasjonen (nærmeste ekte parameter, LAND_DISARMDELAY, har 20 sek som standard, og gjelder for
    //   ØVRIG kun en normal landing, ikke spesifikt en nødstopp-hendelse) - droppet dette konkrete tallet
    //   fra forklaringen i stedet for å ta det inn ubekreftet.
    {
        icon: "fa-ban",
        question: "Hva er forskjellen på å bruke Motor Emergency Stop (\"kill\"/nødstopp) og å disarme farkosten?",
        options: [
            "Det er ingen reell forskjell mellom dem - begge gjør nøyaktig samme ting i praksis",
            "Disarming er den raskeste av de to å aktivere i en akutt nødsituasjon i luften",
            "Motor Emergency Stop krever et gyldig GPS-lås for å virke, disarming gjør ikke det",
            "Motor Emergency Stop kutter strømmen til motorene umiddelbart i alle situasjoner (også i luften) - en vanlig disarm blokkeres normalt av autopiloten under flyvning for å hindre utilsiktet motorstopp"
        ],
        correctIndex: 3,
        explanation: "En dedikert nødstopp-bryter (Motor Emergency Stop) er ment å virke i ENHVER situasjon, også midt i luften - signalet til motorene kuttes umiddelbart, uten unntak.\n\nEn vanlig disarm-kommando er derimot normalt sperret av autopiloten mens den vurderer farkosten som luftbåren, nettopp for å hindre at et utilsiktet feiltrykk stanser motorene midt i flyging - nødstopp overstyrer denne sperren med hensikt.\n\nHusk hjempunktet: i ArduPilot oppdateres hjempunktet normalt til farkostens posisjon ved hver ny arming. Lander du normalt og disarmer vanlig, flytter en ny arming et annet sted hjempunktet dit. Etter en nødstopp i luften beholdes derimot det gamle hjempunktet helt til en eventuell ny arming faktisk skjer."
    },
    {
        icon: "fa-fan",
        question: "Heewing T2 Cruza har to tiltbare motorer foran og én fast motor bak. Hva skjer med den BAKRE motoren når overgangen til fastvinget flyging er fullført?",
        options: [
            "Den fortsetter å gå på lav, konstant gass for ekstra stabilitet i marsjflyging",
            "Den kutter helt - fremdriften kommer da kun fra de to fremre, nå horisontalt tiltede motorene",
            "Den bytter rolle og blir i stedet hovedmotoren for selve fremdriften forover",
            "Den slås ikke av før farkosten faktisk har landet helt og motorene stanses"
        ],
        correctIndex: 1,
        explanation: "Den bakre, faste vertikale motoren har KUN en løfte-/hover-rolle - den kutter helt i fastvinget flyging, mens de to fremre motorene (nå tiltet forover) driver fremdriften."
    },
    {
        icon: "fa-gamepad",
        question: "Ekte ArduPilot-arming via stikken, uten en egen fysisk arm-bryter: hvilken kombinasjon armerer motorene?",
        options: [
            "Full gass + sideror nøytralt, holdt i noen sekunder",
            "Gass i bunn + fullt sideror til HØYRE, holdt i noen sekunder",
            "Gass i bunn + fullt sideror til VENSTRE, holdt i noen sekunder",
            "Gass midtstilt + begge rorene i bunn samtidig"
        ],
        correctIndex: 1,
        explanation: "Gass i bunn + fullt HØYRE sideror i noen sekunder armerer. Disarming er samme gass-posisjon, men med VENSTRE sideror i stedet."
    },
    {
        icon: "fa-clipboard-check",
        question: "Hvorfor kjører ArduPilot en rekke \"pre-arm\"-sjekker (GPS-presisjon, sensortilstand, kalibrering osv.) før den i det hele tatt tillater arming?",
        options: [
            "Kun en treg formalitet - erfarne piloter bør helst deaktivere dem for raskere oppstart",
            "De sjekker utelukkende gjenværende batterinivå og ingenting annet ved farkosten",
            "For å fange opp feil (svak GPS, ukalibrerte sensorer, dårlig tilstandsestimering) før avgang",
            "De er kun relevante for automatiske oppdrag, ikke i det hele tatt for manuell flyging"
        ],
        correctIndex: 2,
        explanation: "Pre-arm-sjekkene fanger opp feil MENS farkosten fortsatt står trygt på bakken - langt billigere (og tryggere) enn å oppdage samme feil midt i luften."
    },
    // "Endre dette til..." + "husk å få med gode alternativ med omtrent samme lengde" (brukeren) - omskrevet
    // fra ett enkelt "alltid kill umiddelbart"-svar til et mer presist, to-lags svar (avstand/tid avgjør):
    // god avstand igjen -> aktiv styring unna er ofte tryggere enn en nødstopp (som bare fjerner ALL
    // styreevne og lar farkosten falle/gli videre ukontrollert i en tilfeldig retning); uunngåelig/nært
    // forestående sammenstøt -> nødstopp for i det minste å stanse de roterende propellene før treff.
    {
        icon: "fa-triangle-exclamation",
        question: "Hva er riktig prioritet dersom en ukontrollert farkost beveger seg mot en person i et VLOS-scenario?",
        options: [
            "Avhenger av avstanden: god avstand igjen - styr aktivt unna. Uunngåelig og nære - nødstopp for å stanse propellene",
            "Alltid nødstopp umiddelbart uansett avstand - aktiv styring bør aldri forsøkes når farkosten er ukontrollert",
            "Alltid prøv å styre unna manuelt helt til siste øyeblikk - en nødstopp endrer uansett ikke utfallet",
            "Bytt i stedet til QRTL-modus og stol på at autopiloten selv navigerer trygt unna personen"
        ],
        correctIndex: 0,
        explanation: "Med god avstand/tid igjen er aktiv styring unna (f.eks. MANUAL) ofte tryggere enn en nødstopp, som bare fjerner all styreevne og lar farkosten falle/gli videre ukontrollert. Er sammenstøtet uunngåelig og nært forestående, stanser en umiddelbar nødstopp i det minste de roterende propellene og reduserer skadeomfanget."
    },
    // "Et til spørsmål" (brukeren) - Q_TILT_TYPE=2 "Vectored Yaw" (se ex0-veiviseren/kodekommentarene i
    // simulator-vtol.js sin updateHeewingPlaneVisual) bruker delvis DIFFERENSIELL NACELLE-TILT (ikke bare
    // differensiell turtall) for gir-autoritet i hover - Q_TILT_YAW_ANGLE (standard 16°) er den ekte
    // ArduPilot-parameteren som styrer hvor mye nacellene vinkles ulikt.
    {
        icon: "fa-rotate",
        // "'Gir-pinnen' kan fjernes siden det er elendig norsk. kan heller si 'full utslag i yaw'" (brukeren).
        question: "Du er i QLOITER i sterk vind og gir fullt utslag i yaw (sideror) for å snu nesa 180 grader. Hvilken mekanisk risiko utsetter du Heewing T2 for akkurat under denne rotasjonen?",
        options: [
            "Ingen reell risiko - gir i QLOITER styres utelukkende av sideroret, akkurat som i fastvinget flyging",
            "Farkosten kan miste retningsstabiliteten helt og begynne å rotere ukontrollert rundt gir-aksen",
            "Farkosten kan synke merkbart - giret skjer delvis ved å tilte frontmotorene ulikt, som reduserer vertikal løftkraft",
            "Batteriet tømmes betydelig raskere, fordi begge frontmotorene da må kjøre med maksimalt turtall"
        ],
        correctIndex: 2,
        explanation: "Heewing sin Q_TILT_TYPE=2 (\"Vectored Yaw\") bruker delvis differensiell nacelle-TILT (Q_TILT_YAW_ANGLE, typisk 16° som standard) for gir-autoritet i hover, ikke bare differensiell turtall. Når frontmotorene vinkles ulikt for å gire, peker de en liten stund mindre rett oppover - det reduserer samlet vertikal løftekraft midlertidig og kan gi et merkbart synk, spesielt ved fullt girutslag i vind."
    },
    // "Ta med et spørsmål eller to om batterispenning for 6s pakker" (brukeren) - to spørsmål: full-/tom-
    // spenning, og spenningsfall ("sag") ved overgang til hover på lavt batteri.
    {
        icon: "fa-battery-full",
        question: "Heewing T2 Cruza bruker en 6S LiPo-batteripakke. Hva er omtrent fulladet spenning, og hva regnes som \"tom\" (bør ikke synke lavere under flyging)?",
        options: [
            "Ca. 22,2 V fulladet (3,7 V/celle nominell), bør ikke synke under ca. 18,0 V (3,0 V/celle) under flyging",
            "Ca. 25,2 V fulladet (4,2 V/celle), bør ikke synke under ca. 19,8-21,0 V (3,3-3,5 V/celle) under flyging",
            "Ca. 25,2 V fulladet, men spenningen har ingenting å si før den treffer 0 V - da er den helt tom",
            "Ca. 21,0 V fulladet (3,5 V/celle), bør ikke synke under ca. 25,2 V under flyging"
        ],
        correctIndex: 1,
        explanation: "En 6S LiPo har 6 celler i serie. Fulladet er 4,2 V/celle = 25,2 V totalt. Nominell spenning (3,7 V/celle = 22,2 V) er IKKE det samme som fulladet - det er et gjennomsnitt gjennom utladingen. Under ca. 3,3 V/celle (19,8 V totalt) bør du allerede være i ferd med å lande - dypere utlading skader cellene permanent."
    },
    {
        icon: "fa-battery-quarter",
        question: "Du flyr en lang fastvinget tur på Heewing T2 og batteriet er nesten tomt idet du skal lande. Hva bør du forvente når du tilter motorene opp og går inn i QLOITER for landing?",
        options: [
            "Ingenting spesielt å forvente - strømforbruket er stort sett identisk i hover og fastvinget marsjflyging",
            "Spenningen kan falle drastisk momentant (et kraftig \"spenningssig\") - hover krever langt mer strøm enn å gli på vingene",
            "Spenningen stiger midlertidig, fordi løftemotorene da avlaster trekkmotoren og reduserer totalforbruket",
            "Batteriet varmes umiddelbart så mye opp at det bør kobles fra og byttes før videre bruk"
        ],
        correctIndex: 1,
        explanation: "Å holde farkosten svevende som en ren multirotor krever mye høyere strøm enn å gli fremover støttet av vingenes eget løft - denne plutselige strømøkningen gir et momentant spenningsfall (\"sag\") som kan vise en skremmende lav spenning selv om batteriet egentlig hadde nok kapasitet igjen til en fastvinget innflyging. Planlegg landingen med dette i tankene når batteriet allerede er lavt."
    },
    // "Legg til et spørsmål om CG - center of Gravity. ekstra kritisk på tricopter" (brukeren).
    {
        icon: "fa-scale-balanced",
        question: "Hvorfor er riktig tyngdepunkt (CG) ekstra kritisk på en tricopter-konfigurasjon som Heewing T2 (to fremre motorer, én bakre), sammenlignet med en vanlig firemotors quadplane?",
        options: [
            "Det er faktisk mindre kritisk enn på en quadplane - færre motorer gir færre steder det kan gå galt",
            "Med kun tre motorer er marginen til å kompensere et feil tyngdepunkt med differensiell trekkraft mye mindre enn på fire",
            "CG spiller ingen rolle for løftemotorene - kun relevant for vingens aerodynamikk i fastvinget marsjflyging",
            "Et feil tyngdepunkt påvirker kun toppfarten i fastvinget flyging, ikke selve hover-stabiliteten i det hele tatt"
        ],
        correctIndex: 1,
        explanation: "En tricopter/Heewing-oppsett har bare tre uavhengige løftepunkter i en asymmetrisk trekant (ikke en symmetrisk firkant som en vanlig quadplane) - det gir vesentlig mindre \"slakk\" i differensiell trekkraft til å kompensere for et tyngdepunkt som ligger feil langs nese-hale- eller sideaksen. Et for langt fremme/bak plassert batteri kan gi en hover-trim autopiloten ikke klarer å styre bort med gjenværende kontrollmyndighet."
    },
    // "ta med på quizen spørsmål om hva som er viktig å sjekke før avgang. riktig svar at roroflatene
    // reagerer korrekt på stikkeinputene" + "spørsmål om hva som er viktig å sjekke etter avgang, i hover.
    // riktig svar å sjekke at yaw pitch og roll reagerer riktig på stikkeinput" (brukeren) - to nye,
    // beslektede spørsmål: én bakke-sjekk (kontrolloverflatenes bevegelsesretning FØR motoren i det hele
    // tatt går fra bakken), én luft-sjekk (selve RESPONSEN i alle tre akser, rett etter avgang, mens man
    // fortsatt kan lande trygt om noe er galt) - to distinkte, kronologisk ordnede vaner, ikke samme
    // spørsmål to ganger.
    // "'pinne' og 'pinneutslag' skal være 'stikke' og 'stikkeutslag' på fjernkontrollen" (brukeren) - rettet.
    // "Det feile alternativet om GPS bør endres. for det er også ganske viktig. Kanskje heller noe som 'At
    // GPS-en viser at farkosten peker nøyaktig mot geografisk nord'? eller det var kanskje dumt? finn noe
    // plausibelt" (brukeren) - enig i at "full GPS-lås/hjempunkt registrert" er en genuint god vane (uheldig
    // som "feil"-svar), men selve HEADING-forslaget er faktisk et treffende, presist feil svar av en annen
    // grunn enn brukeren kanskje tenkte: en GPS-mottaker kan IKKE alene bestemme hvilken vei nesa peker mens
    // farkosten står helt i ro (den avleder kurs fra faktisk BEVEGELSE, "course over ground" - det er
    // KOMPASSET, ikke GPS-en, som gir en statisk retningsavlesning). Byttet til akkurat DEN presiseringen -
    // plausibel (høres ut som en fornuftig sjekk), men konkret feil om hva GPS-en faktisk kan måle i ro.
    {
        icon: "fa-magnifying-glass",
        question: "Hva er en av de viktigste tingene å sjekke rett FØR avgang, mens farkosten fortsatt står på bakken?",
        options: [
            "At rorflatene (høyderor, sideror, skråror) beveger seg tydelig og i RIKTIG retning ved stikkeutslag",
            "At GPS-mottakeren alene viser nøyaktig hvilken retning nesa peker, selv mens farkosten står helt i ro",
            "At batteriet viser nøyaktig 100% ladning - alt lavere bør alltid avbrytes før avgang",
            "At kompasset kalibreres på nytt før HVER eneste avgang, selv uten flytting eller lengre pause"
        ],
        correctIndex: 0,
        explanation: "En kontrollflate-sjekk (\"range check\") - beveg stikkene og se at høyderor/sideror/skråror faktisk beveger seg riktig vei - er den mest grunnleggende, universelle sjekken før enhver RC-/UAV-avgang. En reversert eller frakoblet kanal oppdages langt billigere på bakken enn i luften.\n\nEn GPS-mottaker kan ikke alene bestemme retning mens farkosten står i ro - den avleder kurs fra faktisk bevegelse. Det er kompasset (se spørsmålene om det over) som gir en pålitelig statisk retningsavlesning. 100%-krav er unødvendig strengt, og kompasset trenger kun rekalibreres ved en faktisk grunn til det - ikke rutinemessig hver eneste gang."
    },
    {
        icon: "fa-helicopter",
        question: "Hva er en av de viktigste tingene å sjekke RETT ETTER avgang, mens du fortsatt hovrer lavt og nær hjempunktet?",
        options: [
            "At gir, stigning og rulling reagerer riktig - både retning og styrke - på stikkeutslagene, før du flyr videre eller høyere",
            "At GCS-skjermen viser korrekt klokkeslett og dato for den aktuelle flygingen",
            "At luftfarten allerede har nådd normal marsjfart, selv om farkosten bare hovrer i ro",
            "At batteriprosenten har falt under 90%, som bekreftelse på at motorene faktisk trekker strøm"
        ],
        correctIndex: 0,
        explanation: "Rett etter avgang, mens du fortsatt er lavt og nær hjempunktet, er det siste sjanse til å oppdage en kontrollfeil (feil retning, for svak/sterk respons, en akse som ikke reagerer i det hele tatt) mens en trygg landing fortsatt er billig og enkel - før du flyr videre eller høyere opp. Klokkeslett på GCS-en, luftfart i hover, eller batteriprosent i seg selv sier ingenting om hvorvidt farkosten faktisk styres riktig."
    },
    // "ta med på quizen om man har lov til å modifisere dronen eller montere på annen nyttelast. Riktig
    // svar at det er spesifisert i godkjenningen til aktiviteten" (brukeren) - matcher praksisen i norsk/
    // EASA-regelverk: en operatørs godkjenning/driftstillatelse gjelder for en SPESIFIKK konfigurasjon
    // (vekt, ytelse, nyttelast) - endringer utenfor det den faktisk dekker krever normalt en ny vurdering,
    // ikke en fri sak å avgjøre selv på flyplassen.
    {
        icon: "fa-file-signature",
        question: "Har du lov til å modifisere farkosten (f.eks. montere ekstra nyttelast) i forhold til hvordan den er registrert/godkjent?",
        options: [
            "Ja, fritt fram - så lenge du fortsatt holder deg innenfor produsentens maksimale tillatte avgangsvekt",
            "Det avhenger av hva som faktisk er spesifisert i godkjenningen for den aktuelle aktiviteten",
            "Nei, aldri under noen omstendighet - enhver fysisk endring er alltid forbudt, uansett formål",
            "Kun produsenten selv kan gjøre slike endringer - operatøren/piloten har aldri denne muligheten"
        ],
        correctIndex: 1,
        explanation: "Godkjenningen gjelder for en spesifikk konfigurasjon av farkosten, ikke et fritt vekt-tak alene - hva som er tillatt av modifikasjoner/ekstra nyttelast avhenger av hva som konkret er spesifisert der for akkurat den aktiviteten. En endring utenfor det den faktisk dekker krever normalt en ny vurdering/godkjenning, ikke en vurdering du tar selv i felt."
    }
];
