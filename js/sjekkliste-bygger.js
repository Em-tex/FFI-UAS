/* js/sjekkliste-bygger.js */

const STORAGE_KEY = "ffi-uas:sjekkliste-generator";
// Bumpes hver gang DEFAULTS-innholdet endres vesentlig. Lagret tilstand fra en eldre versjon blir da
// IKKE lastet inn (se loadState) - i stedet lastes den ferske standardmalen, slik at gammelt
// mellomlagret innhold i nettleseren (fra før en oppdatering av sjekkpunktene) ikke stille overskygger
// nytt innhold. Egne redigeringer forsvinner riktignok samtidig, men det er en villet avveining så lenge
// malen fortsatt er under aktiv utvikling.
// Bumpet til 20 selv om selve DEFAULTS-INNHOLDET er uendret siden 19 - mistanke om at "Oppvisning for
// publikum"-raden (rapportert tom i skjermbilde, til tross for at DEFAULTS fortsatt har riktig verdi, se
// linje 40) faktisk er GAMMEL, lagret tilstand fra en tidligere (siden fikset) versjon av
// begrensninger-koden, ikke en fersk bug i gjeldende kode - en versjonsbump her tvinger uansett en frisk
// innlasting fra DEFAULTS igjen, som et sikkert "reset" uansett hva den egentlige årsaken var.
// Bumpet til 23: la til "UAS SITS" og "UAS Autorisasjonstelefon" i ERP-kontaktlisten (se DEFAULTS.erp.limits)
// - HUSK å bumpe dette tallet igjen ved enhver fremtidig DEFAULTS-endring, ellers overskygger brukernes
// egen mellomlagrede localStorage-tilstand stille den nye standardmalen (se loadState) helt til de selv
// nullstiller fanen manuelt.
// Bumpet til 24: FX-10-mal lagt til (AIRCRAFT_CONTENT/TEMPLATE_CONFIG), "UAS Autorisasjonstelefon" ->
// "UAS Aut. telefon", ERP sin "Fly-away / mistet kontakt" -> "Fly-away".
// Bumpet til 25: ERP sin "Operativ leder" -> "FFI Operativ leder UAS".
// Bumpet til 26: "Uventet oppførsel" (QLOITER/QRTL) flyttet fra normal-fanens FX-10-mal til en ny,
// fartøytype-filtrert (aircraft:"fx10") Contingency-del, se setActiveAircraft/TEMPLATE_CONFIG. Fjernet
// et par forklarende parenteser fra FX-10-sjekkpunkter (hører hjemme i kommende Expanded checklist).
// Bumpet til 27: FX-10 sin "Arming"/"Motorer (arm)" slått sammen til ett punkt. Ny "Etter avgang"-del
// (avgang/hovring/pitchvinkel/stigning-modusbytte) skilt ut fra "Overvåking under flyging", som nå kun
// har de generiske, løpende overvåkingspunktene.
// Bumpet til 28: "Bekledning - Passende" lagt til i "Før avreise" (både generisk og FX-10), rett etter
// værsjekken.
// Bumpet til 29: FX-10 sin "Arming" korttet ytterligere ned til "Arm" / "Motorer idle, lavt turtall".
// Bumpet til 30: FX-10 sin "Etter avgang" barbert ned - "Avgang" fjernet, hovring/ro/pitchvinkel slått
// sammen til "Hover - 20 sek ok", "Stigning/modusbytte" forenklet til "Transisjonshøyde - Over 50 m AGL".
// Bumpet til 31: FX-10 sin "Maks rekkevidde" korttet ned til bare "VLOS".
// Bumpet til 32: FX-10 sin "Telemetri" korttet ned fra "Mottatt OK på PC og RC-sender" til "Mottas på PC og RC".
// Bumpet til 33: FX-10 sin "Servoer" korttet ned fra "Liv i servoer - kjenn motstand" til "Testet".
// Bumpet til 34: FX-10 sin "Heading/roll/pitch" korttet ned fra "Riktig ved tilting av farkosten" til
// "Korrekte indikasjoner". "Lavt batterinivå varsel" på contingency-fanen merket aircraft:"generisk" og
// vises dermed ikke lenger for FX-10-malene.
// Bumpet til 35: begrensningen "Oppvisning for publikum" omdøpt til "Oppvisning foran folkemengder"
// (brukerønske) i både generisk og FX-10 normal-mal.
const SCHEMA_VERSION = 35;

// Kolonneoverskrifter i sjekkpunkt-tabellen er ulike for normal- vs. contingency/emergency/erp-
// sjekklister: en normal preflight-sjekk sammenligner mot en forventet status, mens de andre beskriver
// hvilket tiltak som skal iverksettes for en gitt situasjon. Ingen av fanene bruker sjekkboks lenger -
// ingen av disse punktene er noe som fysisk skal tikkes av på papiret.
// showHeader: false på alle fire nå (brukerønske: "Fjern 'Situasjon Tiltak' overskriftene") - text/target
// brukes fortsatt internt (bl.a. som placeholder-tekst på selve input-feltene, se createItemRow), men
// selve <thead>-raden med disse to kolonnenavnene vises ikke lenger i sjekklistebyggeren. Utskriften
// hadde forøvrig ALDRI en slik overskriftsrad i utgangspunktet (se buildSectionBox), så dette er en
// ren skjerm-endring.
const ITEM_LABELS = {
    normal: { text: "Sjekkpunkt", target: "Status / grense", checkbox: false, showHeader: false },
    contingency: { text: "Situasjon", target: "Tiltak", checkbox: false, showHeader: false },
    emergency: { text: "Situasjon", target: "Tiltak", checkbox: false, showHeader: false },
    erp: { text: "Situasjon", target: "Tiltak", checkbox: false, showHeader: false }
};

// Normal-fanens sjekklisteINNHOLD per fartøytype - "generisk" (passer enhver drone) og "fx10" (FX-10
// VTOL-spesifikk, se AIRCRAFT_CONTENT.fx10 under). Egen struktur (ikke bare inni DEFAULTS.normal) fordi
// normal-fanen egentlig har TO uavhengige akser (fartøytype og reguleringskategori MÅK/spesifikk), selv
// om de nå velges via ÉN kombinert nedtrekksmeny (brukerønske: "vil ikke ha to nedtrekksmenyer") - se
// TEMPLATE_CONFIG, som slår sammen de to aksene til fem konkrete menyvalg (mak/spesifikk/fx10-mak/
// fx10-spesifikk/custom).
const DEFAULT_AIRCRAFT = "generisk";
// De FEM offisielle malvalgene i normal-fanens nedtrekksmeny - hver kobler ett fartøytype-innhold
// (AIRCRAFT_CONTENT) til én reguleringskategori-filtrering (setActiveCategory). "custom" (egendefinert)
// har bevisst INGEN oppføring her - den representerer brukerens EGNE endringer, ikke en forhåndsdefinert
// mal å laste inn, se reloadNormalFromDefaults/setNormalTemplate.
const TEMPLATE_CONFIG = {
    mak: { aircraft: "generisk", category: "mak", label: "MÅK" },
    spesifikk: { aircraft: "generisk", category: "spesifikk", label: "Spesifikk" },
    "fx10-mak": { aircraft: "fx10", category: "mak", label: "FX-10 MÅK" },
    "fx10-spesifikk": { aircraft: "fx10", category: "spesifikk", label: "FX-10 spesifikk" }
};
const AIRCRAFT_CONTENT = {
    generisk: {
        equipment: ["Drone", "Fjernkontroll", "Batterier"],
        limits: [
            { key: "Maks vind", value: "10 m/s" },
            { key: "Maks høyde", value: "120 m" },
            { key: "Maks hastighet", value: "" },
            { key: "Maks rekkevidde", value: "VLOS" },
            { key: "Vær", value: "Ingen nedbør" },
            // "RTH-batterinivå" fjernet - brukerønske ("kan variere med operasjonen").
            { key: "Oppvisning foran folkemengder", value: "Forbudt" }
        ],
        sections: [
            {
                title: "Før avreise", items: [
                    { text: "FFI-godkjenning", target: "Gyldig" },
                    { text: "MÅK underkategori", target: "Definert", variant: "mak" },
                    { text: "ATO skjema", target: "Meldt inn", variant: "spesifikk" },
                    { text: "Vær", target: "Innenfor begrensninger" },
                    { text: "Bekledning", target: "Passende" },
                    { text: "Grunneier", target: "Tillatt" },
                    { text: "Dronesoner.no", target: "Sjekket" },
                    { text: "NOTAM", target: "Ingen annen aktivitet" },
                    { text: "Batteri", target: "Ladet" },
                    { text: "Drone", target: "Flygedyktig" },
                    { text: "Utstyrsliste", target: "Gjennomgått" }
                ]
            },
            {
                title: "På operasjonsområdet", items: [
                    { text: "Vær", target: "Innenfor begrensninger" },
                    { text: "HemsWX", target: "Registrert operasjon", variant: "mak" },
                    { text: "Luftrom", target: "Koordinert", variant: "spesifikk" },
                    { text: "Synlige faremomenter", target: "Vurdert" }
                ]
            },
            {
                title: "Før avgang", items: [
                    { text: "Drone og propeller", target: "Ingen skade" },
                    { text: "Batteri", target: "Over 80 %" },
                    { text: "System", target: "GPS grønn, ingen feil" },
                    { text: "Failsafe", target: "Riktig innstilling" },
                    { text: "Tillatelse", target: "Gitt / ikke relevant" }
                ]
            },
            {
                title: "Overvåking under flyging", items: [
                    { text: "Omgivelser", target: "Klart" },
                    { text: "Batteri og radiolink", target: "Tilstrekkelig" }
                ]
            },
            {
                title: "Etter landing", items: [
                    { text: "Motorer", target: "Avslått" },
                    { text: "Tillatelse", target: "Rapportert landet / ikke relevant" },
                    { text: "Drone og propeller", target: "Sjekk ok" },
                    { text: "Logg", target: "Ført" }
                ]
            }
        ]
    },
    // FX-10 (VTOL, fastvinge m/rotorer) - de fire administrative/regulatoriske delene ("Før avreise",
    // "På operasjonsområdet") er UENDRET fra generisk (de gjelder uansett fartøytype), mens "Før avgang"
    // og "Overvåking under flyging" er byttet ut med farkostspesifikke sjekkpunkter fra brukerens egen
    // FX-10-prosedyre (opprinnelig strukturert som "2.1 Farkost preflight" / "2.2 Power up" /
    // "2.3 Flyging" - fordelt inn i de eksisterende delene der de faktisk hører hjemme i flyt-
    // rekkefølgen, ikke som egne nye seksjoner, se brukerønske "få de inne der de passer"):
    //   - 2.1 (fysisk farkost-inspeksjon: vinger/propeller/rorflater/pitotrør) og 2.2 (power up/
    //     telemetri/FENCE/flight modes/batteri/satellitter) hører naturlig hjemme i "Før avgang" -
    //     samme plass som den generiske malens tilsvarende (men langt grovere) sjekkpunkter.
    //   - 2.3 sin arm/avgang/hover-sekvens deles ved selve avgangen: klargjøring og arming (fortsatt på
    //     bakken) blir siste steg i "Før avgang", mens hovring, stigning, modusbytte og selve
    //     operasjonen blir "Overvåking under flyging" (droneen er nå i lufta).
    fx10: {
        equipment: ["Farkost (FX-10)", "Fjernkontroll", "Batterier", "Bakkestasjon (PC)"],
        limits: [
            { key: "Maks vind", value: "10 m/s" },
            { key: "Maks høyde", value: "120 m" },
            { key: "Maks hastighet", value: "" },
            { key: "Maks rekkevidde", value: "VLOS" },
            { key: "Vær", value: "Ingen nedbør" },
            { key: "Oppvisning foran folkemengder", value: "Forbudt" }
        ],
        sections: [
            {
                title: "Før avreise", items: [
                    { text: "FFI-godkjenning", target: "Gyldig" },
                    { text: "MÅK underkategori", target: "Definert", variant: "mak" },
                    { text: "ATO skjema", target: "Meldt inn", variant: "spesifikk" },
                    { text: "Vær", target: "Innenfor begrensninger" },
                    { text: "Bekledning", target: "Passende" },
                    { text: "Grunneier", target: "Tillatt" },
                    { text: "Dronesoner.no", target: "Sjekket" },
                    { text: "NOTAM", target: "Ingen annen aktivitet" },
                    { text: "Batteri", target: "Ladet" },
                    { text: "Farkost", target: "Flygedyktig" },
                    { text: "Utstyrsliste", target: "Gjennomgått" }
                ]
            },
            {
                title: "På operasjonsområdet", items: [
                    { text: "Vær", target: "Innenfor begrensninger" },
                    { text: "HemsWX", target: "Registrert operasjon", variant: "mak" },
                    { text: "Luftrom", target: "Koordinert", variant: "spesifikk" },
                    { text: "Synlige faremomenter", target: "Vurdert" }
                ]
            },
            {
                // Farkost preflight (2.1) + Power up (2.2) + arm/klargjøring (starten av 2.3) - se
                // seksjonskommentaren over.
                title: "Før avgang", items: [
                    { text: "Vinger", target: "Låst posisjon" },
                    { text: "Propellere", target: "Riktig montert, uskadet, dratt til" },
                    // Korte, nowrap-vennlige labels (se print-label-situation i css/style.css) - lange,
                    // ubrutte labels tvinger verdikolonnen unødvendig smal og radhøyden opp, se
                    // splitSectionBoxToFit-kommentaren ved layoutNormalColumns.
                    { text: "Rorflater/servo", target: "Ingen skader, slarkfrie" },
                    { text: "Pitotrør", target: "Uskadet" },
                    { text: "Pitotrørhette", target: "Montert" },
                    { text: "Batteri/bakkeutstyr", target: "Koblet til - RC-sender og bakkeutstyr på" },
                    { text: "Telemetri", target: "Mottas på PC og RC" },
                    { text: "Heading/roll/pitch", target: "Korrekte indikasjoner" },
                    { text: "FENCE-oppsett", target: "VLOS: ca. 300 m / 120 m" },
                    { text: "Flight modes (RC)", target: "Korrekt oppsett (sjekket i telemetri)" },
                    { text: "Batteri", target: "Fulladet (ca. 50,4 V)" },
                    { text: "Satellitter", target: "Mer enn 10" },
                    { text: "Failsafe", target: "Riktig innstilling" },
                    { text: "Tillatelse", target: "Gitt / ikke relevant" },
                    { text: "Flight mode", target: "QLOITER" },
                    { text: "Vindretning", target: "Nesa inn i vinden" },
                    { text: "Servoer", target: "Testet" },
                    // Arming + motorstart slått sammen til ett punkt (brukerønske: "Kanskje slå disse
                    // sammen?") - var to separate punkter (selve håndgrepet / den påfølgende
                    // motortilstanden), naturlig ETT sjekkpunkt.
                    { text: "Arm", target: "Motorer idle, lavt turtall" }
                ]
            },
            {
                // Selve avgangen og den umiddelbare verifiseringssekvensen rett etter (2.3) - EGEN del,
                // atskilt fra den løpende overvåkingen under selve operasjonen (se "Overvåking under
                // flyging" under) - brukerønske: "Dette hører vel til en 'ETTER AVGANG' del?".
                title: "Etter avgang", items: [
                    // "Avgang" (øk throttle) fjernet - brukerønske, selvsagt/unødvendig som eget punkt.
                    // Hovring/ro i hovring/pitchvinkel slått sammen til ett punkt (brukerønske: "Kan vel
                    // slås sammen?").
                    { text: "Hover", target: "20 sek ok" },
                    { text: "Transisjonshøyde", target: "Over 50 m AGL" }
                ]
            },
            {
                // Løpende overvåking gjennom resten av operasjonen - de samme, generiske punktene som
                // den generiske malen bruker her (se AIRCRAFT_CONTENT.generisk).
                title: "Overvåking under flyging", items: [
                    { text: "Omgivelser", target: "Klart" },
                    { text: "Batteri og radiolink", target: "Tilstrekkelig" }
                    // "Uventet oppførsel" (QLOITER/QRTL) flyttet til Contingency-fanen (brukerønske: "det
                    // hører vel hjemme på contingency siden?") - se DEFAULTS.contingency, samme seksjon
                    // "Uventet oppførsel".
                ]
            },
            {
                title: "Etter landing", items: [
                    { text: "Motorer", target: "Avslått" },
                    { text: "Tillatelse", target: "Rapportert landet / ikke relevant" },
                    { text: "Farkost og propeller", target: "Sjekk ok" },
                    { text: "Logg", target: "Ført" }
                ]
            }
        ]
    }
};

const DEFAULTS = {
    normal: Object.assign(
        { drone: "", approvalNumber: "", template: "mak", aircraft: DEFAULT_AIRCRAFT },
        AIRCRAFT_CONTENT[DEFAULT_AIRCRAFT]
    ),
    contingency: {
        title: "Contingency-sjekkliste",
        sections: [
            {
                // "Tap av C2 link" (tidligere "Tap av fjernkontroll-lenke") - brukerønske. To sett
                // sjekkpunkter under SAMME overskrift, merket med variant "mak"/"spesifikk" (samme
                // mekanisme som MÅK/spesifikk-filtreringen på normal-fanen, se ITEM_LABELS/setActiveCategory
                // -kommentaren) - kun settet som matcher gjeldende kategori vises om gangen, se
                // .tab-panel[data-category=...] i css/style.css.
                title: "Tap av C2 link", items: [
                    { text: "Mannskap", target: "Informert om \"lost link\"", variant: "mak" },
                    { text: "C2-link", target: "Forsøk å gjenopprette", variant: "mak" },
                    { text: "UAS", target: "Monitorer", variant: "mak" },
                    { text: "RTH-punkt", target: "Klart for landing", variant: "mak" },
                    { text: "Mannskap", target: "Informer om \"lost link\"", variant: "spesifikk" },
                    { text: "C2-link", target: "Forsøk å gjenopprette", variant: "spesifikk" },
                    { text: "Siste posisjon, høyde og retning", target: "Noter", variant: "spesifikk" },
                    { text: "Radio", target: "Varslet", variant: "spesifikk" },
                    { text: "RTH-punkt", target: "Klart for landing", variant: "spesifikk" }
                ]
            },
            {
                title: "Tap av GNSS", items: [
                    { text: "Mannskap", target: "Informer om \"lost GPS\"" },
                    { text: "Operasjon", target: "Vurder å avbryte" }
                ]
            },
            {
                // aircraft:"generisk" på begge punktene (brukerønske: "For FX-10 på contingency kan lavt
                // batterinivå varsel fjernes") - denne delen gjelder derfor nå KUN de generiske malene,
                // og faller helt bort (hele boksen, ikke bare tomme punkter - se originalItemCount-
                // sjekken i buildSectionBoxesForPrint) når FX-10-malen er valgt, samme prinsipp som
                // "Uventet oppførsel" rett under gjør motsatt vei (KUN for FX-10).
                title: "Lavt batterinivå varsel", items: [
                    { text: "Auto RTH", target: "Avbryt", aircraft: "generisk" },
                    { text: "Drone", target: "Fly hjem og land", aircraft: "generisk" }
                ]
            },
            {
                // Flyttet hit fra normal-fanens FX-10-mal (brukerønske: "Dette hører vel hjemme på
                // contingency siden?") - kun relevant for FX-10 (QLOITER/QRTL er ArduPilot VTOL-
                // terminologi, gir ikke mening for en generisk sjekkliste), se aircraft-filtreringen
                // (setActiveAircraft/TEMPLATE_CONFIG) - vises derfor KUN når FX-10-malen er valgt på
                // normal-fanen, akkurat som MÅK/spesifikk-merkede punkter kun vises for sin kategori.
                title: "Uventet oppførsel", items: [
                    { text: "Uventet oppførsel", target: "Transiter til QLOITER, evt. QRTL", aircraft: "fx10" }
                ]
            }
            // "Annet luftfartøy / trafikk i området", "Kraftig endring i vær/vind" og "Personer i
            // bakkeområdet" fjernet - brukerønske ("Er vel overkill med sånne ting? bruke fornuften og
            // styre unna eller lande. trenger vel ikke sjekkliste for det?") - dette er situasjoner som
            // krever pilotens generelle dømmekraft/luftfartsforståelse der og da, ikke et fast, forhånds-
            // definert tiltak slik de andre contingency-punktene (tap av lenke/GNSS/batteri) faktisk har -
            // en sjekkliste for "bruk sunn fornuft" gir ikke reell verdi.
        ]
    },
    // Emergency: kun det som må håndteres UMIDDELBART mens man er i lufta, for å hindre en ulykke -
    // holdes bevisst enkelt (killswitch, velg område uten personer). Det som skjer ETTER en ulykke
    // (krasj, personskade, rapportering) hører hjemme på ERP-fanen, ikke her.
    emergency: {
        title: "Emergency-sjekkliste",
        sections: [
            {
                title: "Fly-away / kontrolltap", items: [
                    { text: "Drone", target: "Forsøk nødlanding" },
                    { text: "Killswitch", target: "Aktiver" }
                ]
            }
            // "Motorfeil i lufta" og "Batteribrann i lufta" fjernet - brukerønske.
        ]
    },
    // ERP (Emergency Response Plan): det som gjøres ETTER en ulykke - sikring, førstehjelp, varsling og
    // rapportering på bakken. Motstykket til emergency, som kun dekker det umiddelbare i lufta.
    erp: {
        title: "Emergency Response Plan (ERP)",
        // Nødnumrene (Brann/Politi/Ambulanse - se ERP_EMERGENCY_CONTACTS) FØRST, deretter de øvrige,
        // ikke-akutte kontaktene - brukerønske ("Ha nødnumrene øverst"). Selve nød-fremhevingen (ikon +
        // fet/farget navn) er likevel ikke rekkefølge-avhengig i seg selv (den treffer på navnet, se
        // createLimitRow/buildLimitsBox), men ordenen her er fortsatt det brukeren faktisk ser med mindre
        // de endrer den manuelt i sjekklistebyggeren.
        limits: [
            { key: "Brann", value: "110" },
            { key: "Politi", value: "112" },
            { key: "Ambulanse", value: "113" },
            { key: "Legevakt", value: "116 117" },
            { key: "Politi (ikke nød)", value: "02800" },
            { key: "FFI Operativ leder UAS", value: "" },
            { key: "UAS SITS", value: "520 7563 / 692 37 563 (sivilt innvalg)" },
            { key: "UAS Aut. telefon", value: "458 72 017" }
        ],
        sections: [
            {
                // "Ulykke" (tidligere "Ulykke / hendelse") - brukerønske: rene, nummererte prosedyretrinn i
                // ÉN kolonne (se singleColumn/createItemRow-kommentaren) i stedet for en situasjon+tiltak-
                // sammenligning - dette ER selve tiltaket, punkt for punkt, ikke noe å sammenligne mot en
                // situasjonsbeskrivelse. Trinn 3 sine understreker ("Ikke nødvendigvis i rekkefølge: / -
                // Livreddende førstehjelp / ...") er bevisst manuelle linjeskift (\n) INNI ett og samme
                // item.target-felt, ikke egne rader - se print-value-wide/white-space:pre-line i
                // css/style.css for hvorfor de likevel vises på egne linjer i utskriften.
                title: "Ulykke", singleColumn: true, items: [
                    { target: "Disarm dronen" },
                    { target: "Ta ledelsen, få oversikt" },
                    { target: "Ikke nødvendigvis i rekkefølge:\n- Livreddende førstehjelp\n- Slukke brann\n- Sikre skadestedet\n- Varsle nødetatene" },
                    { target: "Varsle operativ leder når du har kapasitet" }
                ]
            },
            {
                // "Fly-away" - samme singleColumn-behandling som "Ulykke" over, se kommentaren der.
                title: "Fly-away", singleColumn: true, items: [
                    { target: "Samle informasjon om siste kjente posisjon, fart, retning, høyde, batteritid og dronetype." },
                    { target: "Vurder varsling av:\n- Lufttrafikktjenesten ved fly-away i kontrollert luftrom\n- Brannvesenet ved skogbrannfare\n- Politiet ved fly-away over bebygd område" },
                    { target: "Vurder om du skal søke etter dronen." },
                    { target: "Varsle operativ leder når du har kapasitet" }
                ]
            }
        ]
    }
};

const TAB_KEYS = ["normal", "contingency", "emergency", "erp"];

/* ---------- Bygging av rader/seksjoner ---------- */

// Vokser en textarea vertikalt etter innholdet (rows=1 i utgangspunktet) - samme teknikk som
// .ex-comment i leksjonsskjema.js. Må kalles etter at elementet er satt inn i DOM-et (scrollHeight er
// upålitelig på løsrevne elementer), derfor kalles den av den som setter inn raden, ikke inni
// createItemRow selv.
function autoGrowTextarea(el) {
    el.style.height = "auto";
    el.style.height = el.scrollHeight + "px";
}

// Spør om bekreftelse før en rad fjernes, men kun hvis raden faktisk har innhold - en tom rad (nettopp
// lagt til, ingenting skrevet inn ennå) fjernes uten mas, siden det ikke er noe å miste.
function confirmRowRemoval(text) {
    if (!text || !text.trim()) return true;
    return confirm("Fjerne dette punktet? Det du har skrevet inn her går tapt.");
}

// Utstyrslisten finnes kun på normal-fanen, så enhver redigering her gjør malen egendefinert - se
// markNormalCustom.
function createEquipmentRow(text) {
    const li = document.createElement("li");
    const input = document.createElement("input");
    input.type = "text";
    input.className = "equipment-text";
    input.placeholder = "Utstyr";
    input.value = text || "";
    input.addEventListener("input", function () { saveState(); markNormalCustom(); });

    const removeBtn = document.createElement("button");
    removeBtn.type = "button";
    removeBtn.className = "remove-row-btn no-print";
    removeBtn.title = "Fjern utstyr";
    removeBtn.innerHTML = '<i class="fa-solid fa-trash"></i>';
    removeBtn.addEventListener("click", function () {
        if (confirmRowRemoval(input.value)) {
            li.remove();
            saveState();
            markNormalCustom();
        }
    });

    li.appendChild(input);
    li.appendChild(removeBtn);
    return li;
}

// Samme rad-type brukes til to ting: Begrensninger (normal) og Kontaktliste (erp) - derfor
// parametriserte placeholder-tekster i stedet for hardkodet "Maks vind"-eksempel begge steder.
// isErpContacts (nytt) - KUN sann for selve ERP-kontaktlisten (ikke normal-fanens Begrensninger, som
// gjenbruker akkurat samme radtype) - se ERP_EMERGENCY_CONTACTS-kommentaren for hvorfor nødnumrene der
// skal skille seg visuelt ut (ikon + fet/farget navn), mens en vanlig begrensningsrad aldri skal det.
function createLimitRow(key, value, keyPlaceholder, valuePlaceholder, isNormal, isErpContacts) {
    const tr = document.createElement("tr");
    const onEdit = isNormal ? function () { saveState(); markNormalCustom(); } : saveState;

    const tdKey = document.createElement("td");
    tdKey.className = "limit-key";
    const keyIcon = document.createElement("i");
    keyIcon.className = "limit-key-icon";
    keyIcon.setAttribute("aria-hidden", "true");
    // BUG-runde 3 (rapportert av brukeren med skjermbilde: "skjeve linjer. og for smal kolonne nr. 2") -
    // de to foregående rundene (fast prosentbredde, så ch-bredde) begge prøvde å GJETTE riktig fast bredde
    // for parameter-kolonnen, men den faktiske containeren viste seg smalere enn antatt begge ganger, så
    // enten gikk verdikolonnen tom for plass, eller (ved en for grådig kolonnebredde) ble bitteliten.
    // Strukturell fiks i stedet for et nytt gjettet tall: parameter-feltet er nå en auto-voksende
    // textarea (rows=1), akkurat samme teknikk som item-text/item-target allerede bruker i
    // createItemRow/autoGrowTextarea - en uvanlig lang parametertekst ("Oppvisning foran folkemengder") får
    // dermed lov til å BREKKE TIL TO LINJER i stedet for enten å kreve en bred, fast kolonne (som stjeler
    // plass fra verdikolonnen) eller å klippes. Kolonnebredden (se .limit-key i css/style.css) kan dermed
    // settes betydelig smalere/mer rimelig uten risiko for at teksten forsvinner.
    const keyInput = document.createElement("textarea");
    keyInput.rows = 1;
    keyInput.className = "limit-key-input";
    keyInput.placeholder = keyPlaceholder || "F.eks. Maks vind";
    keyInput.value = key || "";
    // Nødnummer-fremheving (ikon + fet/farget navn, se ERP_EMERGENCY_CONTACTS) oppdateres LIVE mens
    // brukeren skriver/redigerer selve navnet - ikke bare satt én gang ved oppretting - slik at f.eks. å
    // skrive inn "Brann" i en tom rad umiddelbart viser flamme-ikonet, i stedet for å kreve en reload.
    function updateEmergencyEmphasis() {
        if (!isErpContacts) return;
        const icon = ERP_EMERGENCY_CONTACTS[keyInput.value.trim()];
        tr.classList.toggle("limit-row-emergency", !!icon);
        // BUG (rapportert av brukeren, TO runder på rad med skjermbilder som så identiske ut til tross for
        // en økt padding-fiks - "ikonen ikke hhelt bra justert") - roten var IKKE selve avstanden (som
        // faktisk ble justert riktig), men at Font Awesome-glyfer UTEN "fa-fw" (fixed-width) rendres i sin
        // egen, per-ikon NATURLIGE bredde, ikke den 14px .limit-key-icon-boksen jeg satte - en bred glyf
        // som "fa-truck-medical" (ambulanse) kan rett og slett stikke UTENFOR den boksen og likevel treffe
        // teksten, uansett hvor mye padding jeg la til rundt selve boksen. "fa-fw" er FontAwesome sin egen,
        // formålsbygde klasse for akkurat dette (tvinger ALLE ikoner til samme faste bredde, uansett
        // glyfens egen naturlige form) - riktig fiks fremfor å fortsette å gjette på px-verdier rundt et
        // problem som satt i selve ikon-rendringen, ikke i avstanden.
        keyIcon.className = "limit-key-icon" + (icon ? " fa-solid fa-fw " + icon : "");
    }
    keyInput.addEventListener("input", function () { autoGrowTextarea(keyInput); updateEmergencyEmphasis(); onEdit(); });
    updateEmergencyEmphasis();
    tdKey.appendChild(keyIcon);
    tdKey.appendChild(keyInput);

    const tdValue = document.createElement("td");
    tdValue.className = "limit-value";
    const valueInput = document.createElement("input");
    valueInput.type = "text";
    valueInput.className = "limit-value-input";
    valueInput.placeholder = valuePlaceholder || "F.eks. 15 m/s";
    valueInput.value = value || "";
    valueInput.addEventListener("input", onEdit);
    tdValue.appendChild(valueInput);

    const tdRemove = document.createElement("td");
    tdRemove.className = "col-remove";
    const removeBtn = document.createElement("button");
    removeBtn.type = "button";
    removeBtn.className = "remove-row-btn no-print";
    removeBtn.title = "Fjern begrensning";
    removeBtn.innerHTML = '<i class="fa-solid fa-trash"></i>';
    removeBtn.addEventListener("click", function () {
        if (confirmRowRemoval(keyInput.value || valueInput.value)) {
            tr.remove();
            onEdit();
        }
    });
    tdRemove.appendChild(removeBtn);

    tr.appendChild(tdKey);
    tr.appendChild(tdValue);
    tr.appendChild(tdRemove);
    return tr;
}

// singleColumn (brukerønske: "Punktene skal kun ha en kolonne" - ERP-delene "Ulykke" og "Fly-away") -
// noen sjekklistedeler er rene, frittstående prosedyretrinn (evt. med innrykkede understreker som egne
// linjer inni ETT trinn, f.eks. "Ikke nødvendigvis i rekkefølge: / - Livreddende førstehjelp / - ..."),
// ikke en situasjon+tiltak-sammenligning slik resten av contingency/emergency/erp er bygget opp - en egen
// (tom) situasjon-kolonne ved siden av gir ingen mening der. I stedet for en helt egen datamodell for
// dette (ville brutt "hold det enkelt"-prinsippet resten av filen følger) gjenbrukes samme item.target-
// felt som resten av tabellen allerede har (fritekst, flerlinjes via autoGrowTextarea, akkurat som
// "Tiltak" ellers) - text-cellen utelates bare helt fra selve raden når singleColumn er satt, se
// getState() sin "tr.querySelector('.item-text') finnes kanskje ikke her"-guard og
// buildSectionBoxesForPrint/buildSectionBox for tilsvarende utskrifts-håndtering.
function createItemRow(item, showCheckbox, isNormal, singleColumn) {
    item = item || { text: "", target: "", checked: false, variant: "", aircraft: "" };
    const tr = document.createElement("tr");
    const onEdit = isNormal ? function () { saveState(); markNormalCustom(); } : saveState;
    // Malvalget (MÅK/spesifikk) et sjekkpunkt hører til er satt av standardmalen, ikke redigerbart per
    // rad - kun nedtrekksmenyen øverst på fanen styrer hvilke rader som vises, se setNormalTemplate og
    // CSS-filtreringen på tr[data-variant] i style.css.
    tr.dataset.variant = item.variant || "";
    // Fartøytype (Generisk/FX-10) et punkt hører til - samme prinsipp som data-variant over, men for
    // fartøytype-aksen (se setActiveAircraft/TEMPLATE_CONFIG). Tomt (default) betyr "gjelder alle
    // fartøytyper" - kun eksplisitt merkede punkter (f.eks. et FX-10-spesifikt contingency-punkt om
    // QLOITER/QRTL) filtreres bort når en ANNEN fartøytype er aktiv.
    tr.dataset.aircraft = item.aircraft || "";

    if (showCheckbox) {
        const tdCheck = document.createElement("td");
        tdCheck.className = "col-check";
        const checkbox = document.createElement("input");
        checkbox.type = "checkbox";
        checkbox.className = "item-checked";
        checkbox.checked = !!item.checked;
        checkbox.addEventListener("change", onEdit);
        tdCheck.appendChild(checkbox);
        tr.appendChild(tdCheck);
    }

    // Radnummer (1., 2., 3. ...) - ren CSS-teller på tbody/tr (se .col-number i style.css), ikke lagret
    // i data-modellen. Nummereringen starter dermed automatisk på nytt fra 1 for hver sjekklistedel/
    // tabell, og følger med uten ekstra JS-bokføring når rader legges til, fjernes eller flyttes.
    const tdNumber = document.createElement("td");
    tdNumber.className = "col-number";
    tr.appendChild(tdNumber);

    // Textarea (ikke input) - vokser vertikalt med innholdet i stedet for å klippe/scrolle lang tekst
    // (typisk "Tiltak" på contingency/emergency/erp) inni et enlinjes felt, se autoGrowTextarea. Utelates
    // helt (ingen tom kolonne) når singleColumn er satt, se kommentaren over createItemRow.
    // "let" i funksjonsomfang (ikke "const" inni if-blokken) - removeBtn-lytteren lenger ned må kunne
    // referere textInput uansett singleColumn eller ikke (den leser kun VERDIEN når singleColumn er
    // usann, se der, men selve variabelen må fortsatt være i scope for at det uttrykket skal parse).
    let textInput = null;
    if (!singleColumn) {
        const tdText = document.createElement("td");
        textInput = document.createElement("textarea");
        textInput.rows = 1;
        textInput.className = "item-text";
        textInput.placeholder = "Sjekkpunkt";
        textInput.value = item.text || "";
        textInput.addEventListener("input", function () { autoGrowTextarea(textInput); onEdit(); });
        tdText.appendChild(textInput);
        tr.appendChild(tdText);
    }

    const tdTarget = document.createElement("td");
    const targetInput = document.createElement("textarea");
    targetInput.rows = 1;
    targetInput.className = "item-target";
    targetInput.placeholder = singleColumn ? "Prosedyretrinn" : "Status / tiltak";
    targetInput.value = item.target || "";
    targetInput.addEventListener("input", function () { autoGrowTextarea(targetInput); onEdit(); });
    tdTarget.appendChild(targetInput);
    tr.appendChild(tdTarget);

    const tdRemove = document.createElement("td");
    tdRemove.className = "col-remove";
    const removeBtn = document.createElement("button");
    removeBtn.type = "button";
    removeBtn.className = "remove-row-btn no-print";
    removeBtn.title = "Fjern sjekkpunkt";
    removeBtn.innerHTML = '<i class="fa-solid fa-trash"></i>';
    removeBtn.addEventListener("click", function () {
        // singleColumn-rader har ingen textInput (se over) - bruk targetInput (eneste feltet) da i stedet,
        // samme "spør kun hvis raden faktisk har innhold"-prinsipp som før.
        if (confirmRowRemoval((singleColumn ? targetInput : textInput).value)) {
            tr.remove();
            onEdit();
        }
    });
    tdRemove.appendChild(removeBtn);

    tr.appendChild(tdRemove);
    return tr;
}

function createSectionEl(tabKey, section) {
    section = section || { title: "", items: [] };
    const labels = ITEM_LABELS[tabKey];
    const isNormal = tabKey === "normal";
    const onEdit = isNormal ? function () { saveState(); markNormalCustom(); } : saveState;

    const box = document.createElement("div");
    box.className = "section checklist-section";
    // Må overleve en lagre/last-runde (se getState() sin sections-mapping under) - uten dette ville en
    // singleColumn-del (f.eks. "Ulykke"/"Fly-away" på ERP-fanen) mistet selve singleColumn-egenskapen sin
    // ved neste sideinnlasting fra localStorage, og falt tilbake til vanlig to-kolonners visning.
    if (section.singleColumn) box.dataset.singleColumn = "1";
    // Ikonvalg (se ICON_PICKER_CHOICES/buildIconPicker under) - samme "må overleve en lagre/last-runde
    // via dataset"-mønster som singleColumn over. En lagret/importert del beholder sitt eget valg
    // (section.icon); en HELT NY del (ingen lagret verdi ennå) starter på det vanlige ikonet for akkurat
    // dette navnet (BOX_ICONS-oppslag, samme som utskriften ville brukt automatisk) - ikke noe eget
    // "automatisk"-modus å holde styr på i etterkant (brukerønske: "Automatisk ble ikke intuitivt. Bare
    // ha de standard ikonene der som default"), bare en vanlig startverdi brukeren kan endre som ethvert
    // annet felt.
    box.dataset.icon = section.icon || BOX_ICONS[section.title] || DEFAULT_BOX_ICON;

    // Dra-og-slipp for å endre rekkefølgen på boksene (brukerønske) - box.draggable slås KUN på mens
    // brukeren faktisk har musen nedtrykt på selve grep-håndtaket (dragHandle under), ikke permanent på
    // hele boksen. Uten denne begrensningen ville draggable="true" på hele boksen kapret vanlig
    // tekstmarkering/dra-og-slipp INNI section-title-input og textareaene under (mousedown der ville
    // startet en boks-drag i stedet for tekstmarkering) - se dragHandle sin mousedown/mouseup-håndtering.
    // Selve flyttingen skjer live mens man drar (se getDragTargetSection/dragover-lytteren på
    // [data-sections]-containeren i DOMContentLoaded), IKKE først ved drop - drop trenger derfor ikke
    // gjøre noe utover å hindre nettleserens standard drop-handling.
    box.draggable = false;
    box.addEventListener("dragstart", function (e) {
        box.classList.add("dragging");
        e.dataTransfer.effectAllowed = "move";
        // Uten setData nekter i alle fall Firefox å starte selve draget.
        e.dataTransfer.setData("text/plain", "");
    });
    box.addEventListener("dragend", function () {
        box.classList.remove("dragging");
        box.draggable = false;
        onEdit();
    });

    const header = document.createElement("div");
    header.className = "section-header";

    const dragHandle = document.createElement("span");
    dragHandle.className = "drag-section-handle no-print";
    dragHandle.title = "Dra for å endre rekkefølge på denne delen";
    dragHandle.setAttribute("aria-hidden", "true");
    dragHandle.innerHTML = '<i class="fa-solid fa-grip-vertical"></i>';
    // "mouseup" (ikke bare dragend over) rydder opp igjen for det vanlige tilfellet der brukeren klikker
    // håndtaket uten å faktisk dra noe sted - da fyres aldri dragstart/dragend i det hele tatt.
    dragHandle.addEventListener("mousedown", function () { box.draggable = true; });
    dragHandle.addEventListener("mouseup", function () { box.draggable = false; });

    const titleInput = document.createElement("input");
    titleInput.type = "text";
    titleInput.className = "section-title-input";
    titleInput.placeholder = "Navn på del (f.eks. Før avgang)";
    titleInput.value = section.title || "";
    titleInput.addEventListener("input", onEdit);

    const removeSectionBtn = document.createElement("button");
    removeSectionBtn.type = "button";
    removeSectionBtn.className = "remove-section-btn no-print";
    removeSectionBtn.title = "Fjern del";
    removeSectionBtn.innerHTML = '<i class="fa-solid fa-trash"></i>';
    removeSectionBtn.addEventListener("click", function () {
        if (confirm("Fjerne denne sjekklistedelen med alle sjekkpunkter?")) {
            box.remove();
            onEdit();
        }
    });

    header.appendChild(dragHandle);
    header.appendChild(buildIconPicker(box, onEdit));
    header.appendChild(titleInput);
    header.appendChild(removeSectionBtn);

    const table = document.createElement("table");
    table.className = "builder-item-table";
    if (labels.showHeader) {
        table.innerHTML =
            '<thead><tr>' +
            (labels.checkbox ? '<th class="col-check"></th>' : '') +
            '<th class="col-number"></th>' +
            '<th>' + labels.text + '</th>' +
            '<th>' + labels.target + '</th>' +
            '<th class="col-remove"></th>' +
            '</tr></thead>';
    }
    const tbody = document.createElement("tbody");
    table.appendChild(tbody);

    (section.items || []).forEach(function (item) {
        tbody.appendChild(createItemRow(item, labels.checkbox, isNormal, section.singleColumn));
    });

    const footer = document.createElement("div");
    footer.className = "no-print";
    footer.style.padding = "12px 20px";
    const addItemBtn = document.createElement("button");
    addItemBtn.type = "button";
    addItemBtn.className = "btn btn-secondary add-row-btn";
    addItemBtn.innerHTML = '<i class="fa-solid fa-plus"></i> Legg til sjekkpunkt';
    addItemBtn.addEventListener("click", function () {
        const tr = createItemRow(null, labels.checkbox, isNormal, section.singleColumn);
        tbody.appendChild(tr);
        tr.querySelectorAll("textarea").forEach(autoGrowTextarea);
        onEdit();
    });
    footer.appendChild(addItemBtn);

    box.appendChild(header);
    box.appendChild(table);
    box.appendChild(footer);
    return box;
}

// Finner hvilken av de ANDRE sjekklistedel-boksene i containeren musepekeren er nærmest, og om den
// dragede boksen skal settes FØR eller ETTER den boksen - brukt live under selve draget (se
// dragover-lytteren på [data-sections] i DOMContentLoaded). Avstand måles til boksens MIDTPUNKT (ikke
// bare vertikal posisjon), slik at dette fungerer noe fornuftig BÅDE for normal/erp sin rene
// én-kolonne-liste (.builder-col) OG for contingency/emergency sin to-kolonners "aviskolonne"-flyt
// (.builder-columns, se css/style.css) - ren y-sammenligning alene ville stokket boksene tilfeldig når
// de ligger side ved side i to kolonner.
function getDragTargetSection(container, x, y, dragging) {
    let closest = null;
    let closestDist = Infinity;
    Array.from(container.children).forEach(function (el) {
        if (el === dragging || !el.classList.contains("checklist-section")) return;
        const rect = el.getBoundingClientRect();
        const dx = x - (rect.left + rect.width / 2);
        const dy = y - (rect.top + rect.height / 2);
        const dist = dx * dx + dy * dy;
        if (dist < closestDist) {
            closestDist = dist;
            closest = el;
        }
    });
    if (!closest) return null;
    const rect = closest.getBoundingClientRect();
    return { element: closest, before: y < rect.top + rect.height / 2 };
}

// Kobler på selve dra-og-slipp-omorganiseringen for én [data-sections]-container - kalt én gang per
// fane-container i DOMContentLoaded (containerne finnes fast i HTML-en, opprettes ikke på nytt), ikke
// per boks (boksene selv har bare dragstart/dragend, se createSectionEl).
function initSectionDragAndDrop(container) {
    container.addEventListener("dragover", function (e) {
        const dragging = container.querySelector(".checklist-section.dragging");
        if (!dragging) return;
        // Må avbryte nettleserens standard dragover-håndtering for at containeren skal kunne motta en
        // drop i det hele tatt.
        e.preventDefault();
        const target = getDragTargetSection(container, e.clientX, e.clientY, dragging);
        if (!target) {
            if (container.lastElementChild !== dragging) container.appendChild(dragging);
            return;
        }
        if (target.before) {
            if (dragging.nextSibling !== target.element) container.insertBefore(dragging, target.element);
        } else {
            const after = target.element.nextSibling;
            if (dragging.nextSibling !== after) container.insertBefore(dragging, after);
        }
    });
    // Selve flyttingen skjer allerede live i dragover over - drop trenger bare å hindre nettleserens
    // standard håndtering (f.eks. å åpne teksten som en "fil").
    container.addEventListener("drop", function (e) { e.preventDefault(); });
}

/* ---------- Legg til-knapper ---------- */

function addEquipment(listKey, text) {
    const ul = document.querySelector('[data-equipment="' + listKey + '"]');
    if (!ul) return;
    ul.appendChild(createEquipmentRow(text));
}

// BUG (rapportert av brukeren: "'Maks hastighet' eksempelteksten i høyre kolonne skrives utenfor boksen
// sin. og eksempelet skal helst være i m/s") - de gamle placeholderne ("Parameter (f.eks. Maks vind)",
// "Verdi (f.eks. 20 knop)") var lenger enn selve INNHOLDET de skulle vise et eksempel på (verdikolonnen
// har typisk KORTE reelle verdier, se DEFAULTS.normal.limits - "10 m/s", "120 m", "20 %" ...), og "knop"
// stemte heller ikke med resten av appen som konsekvent bruker m/s (se "Maks vind"). Korte, rene "F.eks.
// ..."-placeholdere i stedet for det lengre "Parameter (f.eks. ...)"/"Verdi (f.eks. ...)"-omslaget -
// kolonnen VISER allerede at det er et parameter/verdi-par, den doble forklaringen er unødvendig og var
// selve årsaken til at teksten ikke fikk plass. Se også .limits-table td.limit-key/-value i
// css/style.css (bredde-fordelingen mellom kolonnene, samme bug).
const LIMIT_PLACEHOLDERS = {
    normal: { key: "F.eks. Maks vind", value: "F.eks. 15 m/s" },
    erp: { key: "F.eks. Operativ leder", value: "Telefonnummer" }
};

function addLimit(tabKey, key, value) {
    const tbody = document.querySelector('[data-limits="' + tabKey + '"]');
    if (!tbody) return;
    const placeholders = LIMIT_PLACEHOLDERS[tabKey] || {};
    const tr = createLimitRow(key, value, placeholders.key, placeholders.value, tabKey === "normal", tabKey === "erp");
    tbody.appendChild(tr);
    // Må gjøres ETTER innsetting i DOM-et - se samme begrunnelse ved addSection/addItemBtn
    // (scrollHeight, som autoGrowTextarea leser, er upålitelig på løsrevne elementer).
    tr.querySelectorAll("textarea").forEach(autoGrowTextarea);
}

function addSection(tabKey, section) {
    const container = document.querySelector('[data-sections="' + tabKey + '"]');
    if (!container) return;
    const box = createSectionEl(tabKey, section);
    container.appendChild(box);
    // Må gjøres etter innsetting i DOM-et - scrollHeight (som autoGrowTextarea leser) er upålitelig på
    // løsrevne elementer, så textareaene kan ikke vokses til riktig høyde inni createSectionEl selv.
    box.querySelectorAll("textarea").forEach(autoGrowTextarea);
}

/* ---------- Tilstand (les fra DOM / skriv til DOM) ---------- */

function getState() {
    const state = {};
    // "category" (MÅK/spesifikk, ALDRI "custom") leses fra normal-panelets data-category-attributt (se
    // setActiveCategory) og lagres på HVER fane sin tabState - dette er det buildSectionBoxesForPrint
    // faktisk filtrerer variant-merkede punkter etter (f.eks. "Tap av C2 link" på contingency-fanen), helt
    // uavhengig av om normal-fanens EGET innhold har blitt tilpasset (se BUG-kommentaren ved
    // setNormalTemplate for hvorfor de to måtte skilles fra hverandre).
    const normalPanel = document.getElementById("tabPanel-normal");
    const activeCategory = (normalPanel && normalPanel.getAttribute("data-category")) || "mak";
    // aircraft (Generisk/FX-10) - samme delt-attributt-prinsipp som category over, se
    // setActiveAircraft-kommentaren.
    const activeAircraft = (normalPanel && normalPanel.getAttribute("data-aircraft")) || DEFAULT_AIRCRAFT;
    TAB_KEYS.forEach(function (tabKey) {
        const tabState = {
            title: (document.getElementById(tabKey + "-title") || {}).value || "",
            category: activeCategory,
            aircraft: activeAircraft
        };

        if (tabKey === "normal") {
            // "template" (kan også være "custom") er derimot KUN normal-fanens egen nedtrekksmeny-
            // tilstand (se setNormalTemplate) - ikke lenger noe filtreringen leser, kun brukt til å
            // gjenopprette riktig valg i selve <select>-elementet ved innlasting (se renderTab).
            tabState.template = (normalPanel && normalPanel.getAttribute("data-template")) || "mak";
            tabState.drone = (document.getElementById("normal-drone") || {}).value || "";
            tabState.approvalNumber = (document.getElementById("normal-approval-number") || {}).value || "";
        }

        const eqList = document.querySelector('[data-equipment="' + tabKey + '"]');
        if (eqList) {
            tabState.equipment = Array.from(eqList.querySelectorAll(".equipment-text")).map(function (el) { return el.value; });
        }

        const limitsBody = document.querySelector('[data-limits="' + tabKey + '"]');
        if (limitsBody) {
            tabState.limits = Array.from(limitsBody.querySelectorAll("tr")).map(function (tr) {
                return {
                    key: tr.querySelector(".limit-key-input").value,
                    value: tr.querySelector(".limit-value-input").value
                };
            });
        }

        const sectionsContainer = document.querySelector('[data-sections="' + tabKey + '"]');
        tabState.sections = sectionsContainer ? Array.from(sectionsContainer.querySelectorAll(".checklist-section")).map(function (box) {
            return {
                title: box.querySelector(".section-title-input").value,
                singleColumn: box.dataset.singleColumn === "1",
                icon: box.dataset.icon || "",
                items: Array.from(box.querySelectorAll("tbody tr")).map(function (tr) {
                    const checkbox = tr.querySelector(".item-checked");
                    // singleColumn-rader (se createItemRow) har ingen .item-text-celle i det hele tatt -
                    // uten denne guarden ville .value på null kastet en TypeError og knekt HELE lagringen
                    // (også de andre, ikke-relaterte fanene) hver gang brukeren skrev noe som helst.
                    const textEl = tr.querySelector(".item-text");
                    return {
                        text: textEl ? textEl.value : "",
                        target: tr.querySelector(".item-target").value,
                        checked: checkbox ? checkbox.checked : false,
                        variant: tr.dataset.variant || "",
                        aircraft: tr.dataset.aircraft || ""
                    };
                })
            };
        }) : [];

        state[tabKey] = tabState;
    });
    return state;
}

function saveState() {
    const state = getState();
    state.__version = SCHEMA_VERSION;
    localStorage.setItem(STORAGE_KEY, JSON.stringify(state));
}

// Kategorien (MÅK/spesifikk - ALDRI "custom") som styrer variant-filtreringen (se tr[data-variant] i
// css/style.css) - egen, separat attributt fra selve nedtrekksmeny-tilstanden (data-template, se
// setNormalTemplate under) etter en BUG (rapportert av brukeren, med skjermbilder: "plutselig er C2 link
// sjekklisten doblet innholdet", senere "fikk fortsatt doblett innholdet ... etter hard refresh") - se
// den fulle forklaringen i css/style.css sin kommentar ved samme filtrering. Satt på BEGGE panelene
// (normal OG contingency, se DEFAULTS.contingency sin "Tap av C2 link") - det er samme
// operasjonskategori for HELE sjekklistesettet, ikke noe normal-fanen "eier" alene, selv om
// nedtrekksmenyen selv kun finnes der.
function setActiveCategory(category) {
    const panel = document.getElementById("tabPanel-normal");
    if (panel) panel.setAttribute("data-category", category);
    const contingencyPanel = document.getElementById("tabPanel-contingency");
    if (contingencyPanel) contingencyPanel.setAttribute("data-category", category);
}

// Fartøytypen (Generisk/FX-10) som filtrerer aircraft-merkede punkter (se tr[data-aircraft] i
// createItemRow/css/style.css) - samme delt-attributt-prinsipp som setActiveCategory over, satt på
// BÅDE normal- og contingency-panelet, slik at et fartøyspesifikt contingency-punkt (f.eks. "Uventet
// oppførsel" -> QLOITER/QRTL, kun relevant for FX-10) kan vises/skjules basert på hvilken mal som er
// valgt på normal-fanen, selv om selve nedtrekksmenyen bare finnes der.
function setActiveAircraft(aircraft) {
    const panel = document.getElementById("tabPanel-normal");
    if (panel) panel.setAttribute("data-aircraft", aircraft);
    const contingencyPanel = document.getElementById("tabPanel-contingency");
    if (contingencyPanel) contingencyPanel.setAttribute("data-aircraft", aircraft);
}

// Setter selve nedtrekksmenyens tilstand (mak/spesifikk/fx10-mak/fx10-spesifikk/egendefinert) -
// data-template-attributtet på tabPanel-normal alene, KUN normal-fanens egen "har jeg avveket fra
// standardmalen"-status (brukt av f.eks. templateSelect-lytteren for å vite hva den skal tilbakestille
// TIL hvis brukeren avbryter en bekreftelsesdialog). Oppdaterer filtreringskategorien OG -fartøytypen
// (se setActiveCategory/setActiveAircraft) KUN når malvalget faktisk ER en av de fire offisielle malene
// (se TEMPLATE_CONFIG), ALDRI når det går til "custom" (se markNormalCustom) - en redigering av
// normal-fanens EGET innhold skal ikke lenger kunne slå av MÅK/spesifikk- eller fartøytype-
// filtreringen på contingency-fanen som en utilsiktet sideeffekt.
function setNormalTemplate(template) {
    const panel = document.getElementById("tabPanel-normal");
    if (panel) panel.setAttribute("data-template", template);
    const config = TEMPLATE_CONFIG[template];
    if (config) {
        setActiveCategory(config.category);
        setActiveAircraft(config.aircraft);
    }
    const select = document.getElementById("normal-template-select");
    if (select && select.value !== template) select.value = template;
}

// Kalles på enhver redigering av selve sjekklisteinnholdet (utstyr, begrensninger, sjekklistedeler/
// -punkter) på normal-fanen - i det øyeblikket brukeren begynner å tilpasse innholdet er det ikke
// lenger den offisielle malen den startet som, så valget hopper automatisk til "Egendefinert".
// Gjør ingenting hvis malen allerede står på egendefinert (unngår unødvendig re-rendering/lagring).
// Rører (via setNormalTemplate sin egen guard) IKKE selve filtreringskategorien - se der.
function markNormalCustom() {
    const panel = document.getElementById("tabPanel-normal");
    if (panel && panel.getAttribute("data-template") !== "custom") {
        setNormalTemplate("custom");
    }
}

// Tømmer én fanes redigerbare innhold (utstyr, begrensninger, sjekklistedeler) i DOM-et - tittel/drone/
// autorisasjonsnummer røres ikke her, siden renderTab uansett overskriver disse direkte (de er enkeltverdi-
// felt, ikke rader som legges til). Delt av reloadNormalFromDefaults, resetTab og importStateFromJson,
// som alle på ulikt vis skal erstatte en fanes innhold fullstendig.
function clearTabContent(tabKey) {
    document.querySelectorAll('[data-equipment="' + tabKey + '"] li').forEach(function (li) { li.remove(); });
    document.querySelectorAll('[data-limits="' + tabKey + '"] tr').forEach(function (tr) { tr.remove(); });
    const sectionsContainer = document.querySelector('[data-sections="' + tabKey + '"]');
    if (sectionsContainer) sectionsContainer.innerHTML = "";
}

// Gjenoppretter normal-fanens utstyrsliste, begrensninger og sjekklistedeler fra standardmalen - brukes
// når man aktivt velger en av de fire offisielle malene i nedtrekksmenyen (se TEMPLATE_CONFIG), slik at
// man faktisk får tilbake det opprinnelige innholdet (inkl. eventuelle rader man har slettet), ikke bare
// et filter over det som måtte stå der fra før. Drone/autorisasjonsnummer røres ikke - det er
// identifiserende info, ikke sjekklisteinnhold. ALL sjekklistedel-tekst (utstyr, begrensninger,
// sjekkpunkter) overskrives, så kalles kun etter bekreftelse (se templateSelect-lytteren i
// DOMContentLoaded).
function reloadNormalFromDefaults(template) {
    const config = TEMPLATE_CONFIG[template] || TEMPLATE_CONFIG.mak;
    const content = AIRCRAFT_CONTENT[config.aircraft] || AIRCRAFT_CONTENT[DEFAULT_AIRCRAFT];

    clearTabContent("normal");

    content.equipment.forEach(function (text) { addEquipment("normal", text); });
    content.limits.forEach(function (limit) { addLimit("normal", limit.key, limit.value); });
    content.sections.forEach(function (section) { addSection("normal", section); });

    setNormalTemplate(template);
    saveState();
}

// Nullstiller KUN én fane (brukt av "Nullstill skjema"-knappen, som nå er fane-lokal, ikke global) -
// tittel, drone/autorisasjonsnummer (kun normal), utstyr, begrensninger og sjekklistedeler settes
// tilbake til standardmalen for den fanen. De andre fanene røres ikke.
function resetTab(tabKey) {
    const titleInput = document.getElementById(tabKey + "-title");
    if (titleInput) titleInput.value = "";

    if (tabKey === "normal") {
        const droneInput = document.getElementById("normal-drone");
        if (droneInput) droneInput.value = "";
        const approvalInput = document.getElementById("normal-approval-number");
        if (approvalInput) approvalInput.value = "";
        checkNormalField(droneInput, null, document.getElementById("normal-drone-note"), false);
        checkNormalField(approvalInput, document.querySelector(".approval-number-group"), document.getElementById("normal-approval-number-note"), false);
    }

    clearTabContent(tabKey);
    renderTab(tabKey, DEFAULTS[tabKey]);
    saveState();
}

// Laster sjekklisteinnhold fra en tidligere nedlastet JSON-fil tilbake inn i skjemaet - motstykket til
// "Last ned som JSON"-knappen, slik at man kan hente frem en lagret/delt sjekkliste og gjøre justeringer
// i stedet for å måtte bygge den opp på nytt fra bunnen. Erstatter ALT innhold i ALLE faner (tittel,
// drone/autorisasjonsnummer, utstyr, begrensninger, sjekklistedeler), derfor bekreftet av brukeren først
// (se uploadJsonInput-lytteren i DOMContentLoaded). Manglende faner i filen (f.eks. en eldre eksport fra
// før ERP-fanen fantes) tømmes i stedet for å falle tilbake til standardmalen, så man ikke uventet får
// inn nytt standardinnhold man aldri ba om.
function importStateFromJson(data) {
    TAB_KEYS.forEach(function (tabKey) {
        const titleInput = document.getElementById(tabKey + "-title");
        if (titleInput) titleInput.value = "";
        if (tabKey === "normal") {
            const droneInput = document.getElementById("normal-drone");
            if (droneInput) droneInput.value = "";
            const approvalInput = document.getElementById("normal-approval-number");
            if (approvalInput) approvalInput.value = "";
        }
        clearTabContent(tabKey);
        renderTab(tabKey, data[tabKey] || { title: "", sections: [] });
    });
    saveState();
    switchTab("normal");
    validateNormalRequiredFields(false);
}

/* ---------- Påkrevde felt: drone + autorisasjonsnummer ----------
   Sjekklisten skal ikke kunne lastes ned/eksporteres uten at drone og autorisasjonsnummer er fylt ut -
   feltene markeres lyserøde og en liten tekst dukker opp under, i stedet for kun å stole på fargen
   alene (tilgjengelighet). "highlight" avgjør om et TOMT felt skal markeres med en gang (brukt når
   feltet mister fokus, eller når man forsøker å laste ned/eksportere) - mens man skriver skal feltet
   bare rydde opp i seg selv når det blir gyldig, ikke bli markert rødt underveis. */
function checkNormalField(inputEl, groupEl, noteEl, highlight) {
    const valid = !!(inputEl && inputEl.value.trim());
    const target = groupEl || inputEl;
    if (target) target.classList.toggle("field-invalid", highlight && !valid);
    if (noteEl) noteEl.classList.toggle("visible", highlight && !valid);
    return valid;
}

function validateNormalRequiredFields(highlight) {
    const droneInput = document.getElementById("normal-drone");
    const droneNote = document.getElementById("normal-drone-note");
    const approvalInput = document.getElementById("normal-approval-number");
    const approvalGroup = document.querySelector(".approval-number-group");
    const approvalNote = document.getElementById("normal-approval-number-note");

    const droneValid = checkNormalField(droneInput, null, droneNote, highlight);
    const approvalValid = checkNormalField(approvalInput, approvalGroup, approvalNote, highlight);
    return droneValid && approvalValid;
}

// Kjøres før nedlasting (PDF/JSON) - bytter til normal-fanen og fokuserer første ufylte felt hvis
// noe mangler, og returnerer false slik at selve nedlastingen avbrytes.
function requireNormalFieldsBeforeExport() {
    const valid = validateNormalRequiredFields(true);
    if (!valid) {
        switchTab("normal");
        const droneInput = document.getElementById("normal-drone");
        const approvalInput = document.getElementById("normal-approval-number");
        const firstInvalid = (droneInput && !droneInput.value.trim()) ? droneInput : approvalInput;
        if (firstInvalid) firstInvalid.focus();
    }
    return valid;
}

function renderTab(tabKey, data) {
    const titleInput = document.getElementById(tabKey + "-title");
    if (titleInput) titleInput.value = data.title || "";

    if (tabKey === "normal") {
        const droneInput = document.getElementById("normal-drone");
        if (droneInput) droneInput.value = data.drone || "";
        const approvalInput = document.getElementById("normal-approval-number");
        if (approvalInput) approvalInput.value = data.approvalNumber || "";
        setNormalTemplate(data.template || "mak");
        // Gjenoppretter filtreringskategorien OG -fartøytypen EKSPLISITT her, uansett hva data.template
        // måtte være (også "custom" - der ville setNormalTemplate sin egen guard (se der) IKKE rørt dem i
        // det hele tatt) - uten dette ville en lagret "custom"-tilstand (fra FØR denne fiksen, se BUG-
        // kommentaren ved buildSectionBoxesForPrint) latt kategorien/fartøytypen stå på HTML-ens
        // hardkodede startverdi i stedet for brukerens faktisk sist valgte.
        setActiveCategory(data.category || "mak");
        setActiveAircraft(data.aircraft || DEFAULT_AIRCRAFT);
    }

    if (Array.isArray(data.equipment)) {
        data.equipment.forEach(function (text) { addEquipment(tabKey, text); });
    } else if (data.equipment && typeof data.equipment === "object") {
        // Bakoverkompatibilitet: eldre lagret tilstand hadde utstyr delt i required/optional -
        // slå sammen til én flat liste siden malen ikke lenger skiller mellom disse.
        [].concat(data.equipment.required || [], data.equipment.optional || [])
            .forEach(function (text) { addEquipment(tabKey, text); });
    }
    if (data.limits) {
        data.limits.forEach(function (limit) { addLimit(tabKey, limit.key, limit.value); });
    }
    (data.sections || []).forEach(function (section) { addSection(tabKey, section); });
}

function loadDefaults() {
    TAB_KEYS.forEach(function (tabKey) { renderTab(tabKey, DEFAULTS[tabKey]); });
}

function loadState() {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return false;
    let state;
    try {
        state = JSON.parse(raw);
    } catch (e) {
        return false;
    }
    if (!state || typeof state !== "object") return false;
    // Utdatert skjemaversjon (fra før en oppdatering av standard sjekkpunkter) - IKKE bruk den lagrede
    // tilstanden, last heller inn den ferske standardmalen (se loadDefaults i DOMContentLoaded).
    if (state.__version !== SCHEMA_VERSION) return false;

    let hadContent = false;
    TAB_KEYS.forEach(function (tabKey) {
        const data = state[tabKey];
        if (!data) return;
        renderTab(tabKey, data);
        hadContent = true;
    });
    return hadContent;
}

function downloadJson(filename, dataObj) {
    const blob = new Blob([JSON.stringify(dataObj, null, 2)], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
}

/* ---------- Utskrift (kompakt, tokolonnes A4-utsendelse for feltbruk) ----------
   Bygger et eget, avstrippet print-view (se .print-view i css/style.css) i stedet for å printe det
   redigerbare skjemaet direkte - gir full kontroll på layouten (tette bokser i to kolonner per side,
   fargekode per sjekklistetype) uavhengig av hvordan redigeringsvisningen ser ut. */

// sidebarLabel (BEREDSKAP/NØD) FJERNET (brukerønske: "kan fjernes. så er det mer plass i bredden for
// sjekklistene") - den loddrette fargede stripen (se buildPrintSidebar) tok en fast 9mm + 5mm mellomrom
// fra hver av contingency/emergency sine allerede trange halvsider (se .print-columns-half). Satt til
// null (samme som normal-fanen, som aldri hadde noen sidefelt) i stedet for å fjerne selve
// buildPrintSidebar-mekanismen - den er fortsatt i bruk til bl.a. erp, og kan gjenbrukes igjen for andre
// faner ved å sette en tekst her på nytt. .print-columns-half (flex:1, eneste gjenværende barn i
// .print-page-body når sidebar er null) utvider seg automatisk til å fylle plassen sidebaren tidligere
// tok - ingen ekstra CSS-endring nødvendig.
const TAB_META = {
    normal: { label: "Normal", sidebarLabel: null },
    contingency: { label: "Contingency", sidebarLabel: null },
    emergency: { label: "Emergency", sidebarLabel: null },
    erp: { label: "ERP", sidebarLabel: "ERP" }
};

// BUG (funnet ved å faktisk rendre en test-JSON gjennom en skjult nettleser og lese PDF-en - se
// git-historikken for dette funnet, og PRINT_TABLE_BUG-kommentaren ved buildSectionBoxesForPrint) - en
// ekte <table> inni en boks med break-inside:avoid, flytende (float) inni .print-grid sin flow-root-
// kontekst, trigger en reell Chromium-utskriftsbug: HELE resten av siden (alt etter tittelbanneret)
// hopper i sin helhet til neste fysiske side, uansett hvor mye ledig plass som faktisk er igjen -
// isolert og bekreftet ved å bytte akkurat DENNE ene tabellen ut med et vanlig, ustrukturert element,
// som fjernet symptomet fullstendig. buildPrintRow/tabell-byggerne under bygger derfor nå en CSS Grid-
// basert "tabell" (div-er, ikke <table>/<tr>/<td>) i stedet - visuelt identisk (se .print-rows i
// css/style.css), men uten ekte tabell-semantikk som trigger buggen. .print-row er display:contents
// (se CSS) - den ligger fortsatt i DOM-treet (så :last-child osv. treffer riktig rad), men gir ingen
// egen visuell boks; cellene blir dermed direkte grid-celler i .print-rows sin rutenettflyt, akkurat
// som <td>-er ville blitt direkte tabell-celler.
function buildPrintRow(cells, extraClass) {
    const row = document.createElement("div");
    row.className = "print-row";
    cells.forEach(function (cell) {
        const cellEl = document.createElement("div");
        cellEl.className = "print-cell " + (cell.className || "");
        if (cell.iconClass) {
            // Nødnummer-ikon (se ERP_EMERGENCY_CONTACTS/buildLimitsBox) - egen <i>-node + tekst-node i
            // stedet for cell.html under, slik at selve nummer-/kontaktnavnet fortsatt settes trygt via
            // textContent (ingen HTML-injeksjon av det brukeren har skrevet inn i feltet).
            const icon = document.createElement("i");
            icon.className = "fa-solid " + cell.iconClass;
            cellEl.appendChild(icon);
            cellEl.appendChild(document.createTextNode(" " + (cell.text || "")));
        } else if (cell.html !== undefined) cellEl.innerHTML = cell.html;
        else cellEl.textContent = cell.text || "";
        row.appendChild(cellEl);
    });
    if (extraClass) row.className += " " + extraClass;
    return row;
}

// Illustrerende ikon per boks-tittel - kjente standardtitler slår opp et treffende ikon; alt annet
// (egendefinerte/omdøpte deler) faller tilbake til et generisk sjekkliste-ikon i stedet for å stå uten.
const BOX_ICONS = {
    "Utstyrsliste": "fa-toolbox",
    "Begrensninger": "fa-ruler",
    "Kontaktliste": "fa-phone",
    "Før avreise": "fa-clipboard-check",
    "På operasjonsområdet": "fa-map-location-dot",
    "Før avgang": "fa-plane-departure",
    "Etter avgang": "fa-arrow-trend-up",
    "Overvåking under flyging": "fa-eye",
    "Etter landing": "fa-flag-checkered",
    "Tap av C2 link": "fa-wifi",
    "Tap av GNSS": "fa-satellite-dish",
    "Lavt batterinivå varsel": "fa-battery-quarter",
    "Uventet oppførsel": "fa-compass",
    "Fly-away / kontrolltap": "fa-triangle-exclamation",
    "Ulykke": "fa-car-burst",
    "Fly-away": "fa-magnifying-glass-location"
};
const DEFAULT_BOX_ICON = "fa-clipboard-list";

// Brukerønske ("kan man også gjøre det mulig å endre på ikoner i headerne til tabellene?") - BOX_ICONS
// over slår opp et ikon basert på et KJENT, eksakt sjekklistedel-navn - en umdøpt eller egendefinert del
// (og alle deler forøvrig, hvis brukeren rett og slett vil ha et annet ikon enn det automatiske) hadde
// ingen måte å velge et annet ikon på. ICON_PICKER_CHOICES er valgene i selve ikonvelgeren (se
// buildIconPicker under) - de samme ikonene som allerede brukes i BOX_ICONS (gjenkjennelige fra
// utskriften) pluss noen få generelle ekstra, i stedet for å true inn HELE Font Awesome-biblioteket i en
// uoversiktlig liste.
const ICON_PICKER_CHOICES = [
    "fa-clipboard-list", "fa-clipboard-check", "fa-list-check", "fa-toolbox", "fa-ruler", "fa-phone",
    "fa-map-location-dot", "fa-plane-departure", "fa-plane-arrival", "fa-arrow-trend-up", "fa-eye",
    "fa-flag-checkered", "fa-wifi", "fa-satellite-dish", "fa-battery-quarter", "fa-battery-full",
    "fa-compass", "fa-triangle-exclamation", "fa-car-burst", "fa-magnifying-glass-location", "fa-cloud",
    "fa-wind", "fa-bolt", "fa-shield-halved", "fa-users", "fa-gear", "fa-camera", "fa-route",
    "fa-clock", "fa-circle-check", "fa-house", "fa-helicopter"
];
// Sentinelverdi for "ingen ikon i det hele tatt" (brukerønske: "det må også være mulig å velge å ikke ha
// noe ikon") - lagres i box.dataset.icon/section.icon som ETHVERT annet ikonvalg (en vanlig streng, ikke
// tom/manglende - siden tom/manglende nå betyr "ikke lagret ennå" og løses til det vanlige BOX_ICONS-
// ikonet, se createSectionEl). setBoxHeaderContent (under) dropper selve <i>-ikonet fullstendig når
// verdien er nettopp denne.
const NO_ICON = "__none__";

// Bygger selve ikonvelgeren - en liten knapp (viser delens GJELDENDE ikon) som åpner et rutenett med
// HELE ICON_PICKER_CHOICES-utvalget å velge mellom. box.dataset.icon holder alltid en KONKRET verdi (satt
// til det vanlige/automatiske ikonet allerede når delen opprettes, se createSectionEl) - lest av getState
// (se der) og skrevet tilbake av renderTab/createSectionEl ved innlasting. no-print (som resten av selve
// byggegrensesnittet) - ikke noe utskriften skal vise, kun selve valget den allerede påvirker.
// FORENKLET (brukerønske: "Automatisk ble ikke intuitivt. Bare ha de standard ikonene der som default") -
// hadde tidligere et eget "Automatisk"-valg (tomt dataset.icon, tolket som "slå opp BOX_ICONS på nytt
// hver gang") - droppet til fordel for at DET STANDARD ikonet rett og slett ER startverdien, uten noe eget
// "automatisk"-modus å forholde seg til; brukeren velger deretter et konkret ikon som ethvert annet felt.
function buildIconPicker(box, onEdit) {
    const wrap = document.createElement("div");
    wrap.className = "icon-picker no-print";

    const btn = document.createElement("button");
    btn.type = "button";
    btn.className = "icon-picker-btn";
    btn.title = "Velg ikon for denne delen";
    const btnIcon = document.createElement("i");
    wrap.appendChild(btn);
    btn.appendChild(btnIcon);

    const menu = document.createElement("div");
    menu.className = "icon-picker-menu";
    wrap.appendChild(menu);

    function refreshBtnIcon() {
        // fa-ban - tydelig "ingen ikon"-symbol på selve VELGERKNAPPEN (kun her, i selve
        // byggegrensesnittet) når NO_ICON er valgt - printet header viser da rett og slett ingen ikon i
        // det hele tatt, se setBoxHeaderContent.
        btnIcon.className = "fa-solid " + (box.dataset.icon === NO_ICON ? "fa-ban" : (box.dataset.icon || DEFAULT_BOX_ICON));
    }
    refreshBtnIcon();

    function setIcon(value) {
        box.dataset.icon = value;
        refreshBtnIcon();
        menu.classList.remove("open");
        onEdit();
    }

    const noneOption = document.createElement("button");
    noneOption.type = "button";
    noneOption.className = "icon-picker-none";
    noneOption.innerHTML = '<i class="fa-solid fa-ban"></i> Ingen ikon';
    noneOption.addEventListener("click", function () { setIcon(NO_ICON); });
    menu.appendChild(noneOption);

    const grid = document.createElement("div");
    grid.className = "icon-picker-grid";
    ICON_PICKER_CHOICES.forEach(function (iconClass) {
        const option = document.createElement("button");
        option.type = "button";
        option.className = "icon-picker-option";
        option.innerHTML = '<i class="fa-solid ' + iconClass + '"></i>';
        option.addEventListener("click", function () { setIcon(iconClass); });
        grid.appendChild(option);
    });
    menu.appendChild(grid);

    btn.addEventListener("click", function (e) {
        e.stopPropagation();
        const willOpen = !menu.classList.contains("open");
        // Lukk enhver ANNEN åpen ikonvelger på siden først - kun én åpen om gangen, samme prinsipp som
        // sim-dropdown-menyene ellers på nettstedet.
        document.querySelectorAll(".icon-picker-menu.open").forEach(function (m) { m.classList.remove("open"); });
        if (willOpen) menu.classList.add("open");
    });
    return wrap;
}
// Lukker en åpen ikonvelger ved klikk utenfor - én global lytter (ikke én per velger) er nok siden kun
// åpne menyer i det hele tatt reagerer.
document.addEventListener("click", function () {
    document.querySelectorAll(".icon-picker-menu.open").forEach(function (m) { m.classList.remove("open"); });
});

// ERP-kontaktlisten - brukerønske ("Ha nødnumrene øverst. noe indikasjon på at de er nødnumre, f.eks
// med farge eller bold. Men ikke for skrikete... Gjerne med relevant ikon"). Nøkkelen MÅ matche
// limit.key EKSAKT (samme "kjente standardnavn slår opp et ikon"-prinsipp som BOX_ICONS over) - dette
// treffer dermed kun de tre REELLE nødnumrene (110/112/113), ikke f.eks. "Politi (ikke nød)" eller
// "Operativ leder", som bevisst IKKE skal ha samme fremhevede stil (de er reelle kontakter, men ikke
// nødnumre). Brukt av BÅDE selve sjekklistebyggeren (se createLimitRow) og utskriften (se
// buildLimitsBox) - samme kilde, ingen risiko for at de to driver fra hverandre.
const ERP_EMERGENCY_CONTACTS = {
    "Brann": "fa-fire",
    "Ambulanse": "fa-truck-medical",
    "Politi": "fa-shield-halved"
};

// Setter ikon + tittel i en boks-header trygt (via DOM-noder, ikke innerHTML-sammenslåing) - tittelen
// kan være fri tekst brukeren har skrevet selv. iconOverride (valgfri) - brukerens eget ikonvalg for
// nettopp DENNE sjekklistedelen (se ICON_PICKER_CHOICES/buildIconPicker), foran det automatiske
// BOX_ICONS-oppslaget på tittelen - Utstyrsliste/Begrensninger/Kontaktliste har ingen egen velger og
// sender derfor aldri denne, så de beholder alltid sitt faste BOX_ICONS-ikon uendret.
function setBoxHeaderContent(header, title, iconOverride) {
    // NO_ICON (brukerønske: "det må også være mulig å velge å ikke ha noe ikon") - dropper selve
    // <i>-elementet fullstendig, i stedet for å falle tilbake til DEFAULT_BOX_ICON slik en ukjent/tom
    // verdi ellers ville gjort - et EKSPLISITT "ingen ikon"-valg skal faktisk ikke vise noe ikon.
    if (iconOverride === NO_ICON) {
        header.appendChild(document.createTextNode(title));
        return;
    }
    const icon = document.createElement("i");
    icon.className = "fa-solid " + (iconOverride || BOX_ICONS[title] || DEFAULT_BOX_ICON);
    header.appendChild(icon);
    header.appendChild(document.createTextNode(" " + title));
}

function buildEquipmentBox(equipment) {
    const items = (equipment || []).filter(function (t) { return t && t.trim(); });
    if (!items.length) return null;

    const box = document.createElement("div");
    box.className = "print-box";
    const header = document.createElement("div");
    header.className = "print-box-header print-box-header-reference";
    setBoxHeaderContent(header, "Utstyrsliste");
    box.appendChild(header);

    const table = document.createElement("div");
    table.className = "print-rows print-rows-1col";
    items.forEach(function (text) {
        table.appendChild(buildPrintRow([
            { className: "print-label", text: text }
        ]));
    });
    box.appendChild(table);
    return box;
}

// isContactList (nytt) - KUN sann for ERP sin Kontaktliste (se buildErpPage), ikke normal-fanens
// Begrensninger som gjenbruker samme funksjon - se ERP_EMERGENCY_CONTACTS-kommentaren for hvorfor
// nødnumrene (Brann/Ambulanse/Politi) der skal skille seg visuelt ut (ikon + fet/farget navn) i
// utskriften, samme fremheving som selve sjekklistebyggeren gir live (se createLimitRow).
function buildLimitsBox(limits, title, isContactList) {
    const rows = (limits || []).filter(function (l) { return l.key && l.key.trim(); });
    if (!rows.length) return null;

    const box = document.createElement("div");
    box.className = "print-box";
    const header = document.createElement("div");
    header.className = "print-box-header print-box-header-reference";
    setBoxHeaderContent(header, title || "Begrensninger");
    box.appendChild(header);

    const table = document.createElement("div");
    // print-rows-contacts (KUN kontaktlisten, se isContactList) - brukerønske: første kolonne (kontakt-
    // navnet) skulle være litt bredere enn Begrensninger/Utstyrsliste sin auto-fordelte bredde, uten å
    // gjøre generisk .print-rows fastlåst for de andre boksene som fortsatt deler samme tabellklasse.
    table.className = "print-rows" + (isContactList ? " print-rows-contacts" : "");
    rows.forEach(function (l) {
        const emergencyIcon = isContactList ? ERP_EMERGENCY_CONTACTS[l.key.trim()] : null;
        const keyCell = { className: "print-label print-key" + (emergencyIcon ? " print-key-emergency" : ""), text: l.key };
        if (emergencyIcon) keyCell.iconClass = emergencyIcon;
        table.appendChild(buildPrintRow([
            keyCell,
            { className: "print-value", text: l.value }
        ]));
    });
    box.appendChild(table);
    return box;
}

// Sjekklistedeler med bare ÉTT punkt: overskriften er allerede beskrivende for situasjonen ("Tap av
// GNSS", "Kollisjon / krasj" osv.), så en egen situasjon-kolonne ved siden av tiltaket er "smør på
// flesk" - vis kun selve tiltaket/handlingen. Ved flere punkter trengs fortsatt situasjon+tiltak per
// rad for å skille dem fra hverandre.
// startNumber (valgfri, default 1) - hvilket tall FØRSTE rad skal få. Brukt av splitSectionBoxToFit
// (se layoutNormalColumns) slik at en "... forts."-fortsettelsesboks fortsetter nummereringen der den
// opprinnelige delen slapp (rapportert: "forts."-boksen startet på nytt fra 1) i stedet for at hver av
// de to delboksene teller fra 1 hver for seg.
function buildSectionBox(section, showCheckbox, startNumber) {
    startNumber = startNumber || 1;
    // singleColumn-deler (se createItemRow-kommentaren) legger ALT innholdet i item.target, item.text er
    // alltid tom der - BUG (ville filtrert bort ALLE punktene i f.eks. ERP sin "Ulykke"/"Fly-away" og vist
    // "Ingen sjekkpunkter" i utskriften) hvis dette fortsatt filtrerte kun på it.text slik det gjorde før
    // singleColumn fantes. Filtrerer derfor på target i stedet når singleColumn er satt.
    const singleColumn = !!section.singleColumn;
    const items = (section.items || []).filter(function (it) {
        return singleColumn ? (it.target && it.target.trim()) : (it.text && it.text.trim());
    });
    if (!items.length && !(section.title && section.title.trim())) return null;

    const box = document.createElement("div");
    box.className = "print-box";
    const header = document.createElement("div");
    header.className = "print-box-header";
    setBoxHeaderContent(header, section.title && section.title.trim() ? section.title : "Uten navn", section.icon);
    box.appendChild(header);

    if (!items.length) {
        const empty = document.createElement("div");
        empty.className = "print-empty-note";
        empty.textContent = "Ingen sjekkpunkter";
        box.appendChild(empty);
        return box;
    }

    if (items.length === 1) {
        const single = document.createElement("div");
        single.className = "print-single-action";
        single.textContent = items[0].target && items[0].target.trim() ? items[0].target : items[0].text;
        box.appendChild(single);
        return box;
    }

    const table = document.createElement("div");
    // print-rows-situasjon (nytt, KUN her - ikke utstyrsliste/kontaktliste, som gjenbruker generisk
    // .print-rows uendret) - BUG (rapportert av brukeren: "høyre stiplede kant for langt ut til høyre" på
    // en boks): situasjon/sjekkpunkt-kolonnen har white-space:nowrap (se .print-label-situation i
    // css/style.css) for at KORTE, faste kategorinavn ("Batteri", "Vær") aldri skal brekke unødvendig -
    // men et langt, egendefinert sjekkpunkt (hele setninger, ikke en kort kategori) kan da ikke brekke
    // heller, og siden bare denne tabellen manglet table-layout:fixed fikk den lov til å bli bredere enn
    // boksen for å få plass til den ubrutte teksten - den stiplede kanten (som følger tabellens egen
    // bredde) ble dermed dratt lenger ut til høyre enn boks-headeren over. print-rows-situasjon gir denne
    // tabellen (og KUN denne, se .print-rows-situasjon i css/style.css) en fast kolonnebredde i stedet,
    // slik at et uvanlig langt sjekkpunkt brekker til flere linjer i stedet for å presse tabellen ut over
    // boksens kant.
    // print-rows-single (singleColumn-varianten, se under) og print-rows-checkbox (showCheckbox) - egne
    // modifikator-klasser (i stedet for én felles "print-rows-situasjon" som før) siden CSS Grid, i
    // motsetning til den gamle <table>, må vite EKSAKT hvor mange kolonner raden har på forhånd (se
    // .print-rows-varianter i css/style.css) - antall celler per rad er uansett fast for HELE denne
    // tabellen (samme singleColumn/showCheckbox for alle rader i ett buildSectionBox-kall), så dette er
    // trygt å avgjøre én gang her.
    table.className = "print-rows " + (singleColumn ? "print-rows-single" : "print-rows-situasjon") + (showCheckbox ? " print-rows-checkbox" : "");
    items.forEach(function (item, index) {
        const cells = [];
        if (showCheckbox) {
            cells.push({ className: "print-check", html: '<span class="print-checkbox-box' + (item.checked ? " checked" : "") + '"></span>' });
        }
        // BUG (rapportert av brukeren: "tallet 3. og 2. skal være stilt øverst i raden. ikke midten") -
        // .print-rows td er (med god grunn, se kommentaren ved regelen i css/style.css) MIDTJUSTERT som
        // standard for de vanlige situasjon/tiltak-radene - fint for en kort label ved siden av en lengre
        // verdi, men et flerlinjes singleColumn-trinn (f.eks. "Ikke nødvendigvis i rekkefølge:" med fire
        // understreker under) gjorde selve raden mye høyere, og nummeret havnet synlig midt i, ikke ved
        // siden av FØRSTE linje slik en vanlig nummerert liste ellers ville gjort. print-number-top (KUN
        // lagt på for singleColumn) toppjusterer akkurat disse radene, uten å røre den bevisste
        // midtjusteringen på resten av utskriften.
        cells.push({ className: "print-number" + (singleColumn ? " print-number-top" : ""), text: (startNumber + index) + "." });
        if (singleColumn) {
            // Ren, bred énkolonne-celle (se createItemRow-kommentaren) - ingen situasjon/tiltak-splitt,
            // kun selve prosedyretrinnet. print-value-wide (se css/style.css) bruker white-space:pre-line
            // slik at manuelle linjeskift i teksten (f.eks. understreker under "Ikke nødvendigvis i
            // rekkefølge:") faktisk vises som egne linjer i utskriften, ikke slås sammen til én lang linje.
            cells.push({ className: "print-value print-value-wide", text: item.target || item.text || "" });
        } else {
            // print-label-situation (BUG: rapportert av brukeren - "Drone og propeller" på normal-sidens
            // "Før avgang"/"Etter landing" brøt til to linjer i utskriften selv om det burde vært plass) -
            // ren tabell-auto-layout deler kolonnebredden mellom label/verdi basert på ALT innhold i HELE
            // tabellen (alle rader under ett), ikke rad for rad - en annen rads lange verditekst kunne
            // dermed stjele nok bredde til å presse EN KORT label i en helt annen rad til å brekke, selv om
            // den totale radbredden i seg selv hadde vært mer enn nok om kolonnene ble fordelt annerledes.
            // Situasjon/Sjekkpunkt-siden (item.text) er alltid en kort, fast kategoribetegnelse ("Batteri",
            // "System", "Drone og propeller" ...) som ALDRI skal brekke - se .print-label-situation i
            // css/style.css (white-space:nowrap, KUN på denne kombinasjonen, ikke generisk .print-label -
            // den brukes også av f.eks. utstyrslisten, der lange varenavn fortsatt skal få brekke normalt).
            // Verdi-siden (item.target) kan derimot være en hel setning - den skal fortsatt kunne brekke.
            cells.push({ className: "print-label print-label-situation", text: item.text });
            cells.push({ className: "print-value", text: item.target || "" });
        }
        table.appendChild(buildPrintRow(cells));
    });
    box.appendChild(table);
    return box;
}

// Overskriften er nå kun én linje (tittelen) - den viste tidligere også et eget "Contingency"/
// "Emergency"-merke ved siden av, som gjorde at f.eks. "Contingency-sjekkliste" + "Contingency" sto der
// to ganger. Datoen er flyttet ut til ett enkelt hjørnestempel, bygget kun én gang for hele utskriften og
// vist på hver fysiske side (se buildPageDateStamp/buildAllPrintPages), i stedet for gjentatt i hver
// overskrift/halvdel.
// Reservetittel hvis navnefeltet er tomt - "ERP-sjekkliste" var misvisende, ERP er navnet på selve
// planen (Emergency Response Plan), ikke en type "sjekkliste".
const FALLBACK_TITLES = { contingency: "Contingency-sjekkliste", emergency: "Emergency-sjekkliste", erp: "Emergency Response Plan (ERP)" };

function buildPrintHeader(tabKey, data) {
    const header = document.createElement("div");
    header.className = "print-header";
    const h1 = document.createElement("h1");
    // Normal-siden har fast tittel "Sjekkliste" - contingency/emergency/erp beholder sitt eget,
    // redigerbare navnefelt.
    h1.textContent = tabKey === "normal" ? "Sjekkliste" : (data.title && data.title.trim() ? data.title : FALLBACK_TITLES[tabKey]);

    // Dronenavn inn i selve tittelbanneret (hvit skrift, høyrestilt) - "Sjekkliste" venstrestilt i
    // kolonne 1, dronenavnet høyrestilt i kolonne 2 (se .print-header-with-meta i css/style.css), i
    // stedet for tidligere sentrert tittel + dronenavn klemt inn i en smal balanse-kolonne til venstre -
    // brukerønske ("droner med lange navn får ikke plass i toppteksten") ga dronenavnet dermed langt mer
    // bredde å boltre seg på. text-overflow:ellipsis er beholdt i CSS-en som siste sikkerhetsnett for de
    // aller lengste navnene.
    // Autorisasjonsnummeret sto tidligere her også (høyrestilt) - brukerønske ("FFI-UAS nummeret blir
    // ikke skrevet ut nå, ender bare med ... siden det ikke er plass") etter at banneren ble halvert
    // (se .print-header-half i css/style.css) - for lite bredde igjen til BÅDE dronenavn og nummer ved
    // siden av tittelen. Flyttet til et eget, grått hjørnestempel i stedet, se buildApprovalStamp -
    // samme plass-uavhengige løsning som allerede fantes for datoen (buildPageDateStamp).
    const drone = tabKey === "normal" && data.drone && data.drone.trim() ? data.drone : "";
    if (drone) {
        header.classList.add("print-header-with-meta");
        const droneSpan = document.createElement("span");
        droneSpan.className = "print-header-meta print-header-meta-right";
        droneSpan.textContent = drone;
        header.appendChild(h1);
        header.appendChild(droneSpan);
    } else {
        header.appendChild(h1);
    }
    return header;
}

// FFI-UAS-autorisasjonsnummeret sto tidligere flankert i selve tittelbanneret (se buildPrintHeader) -
// flyttet til et eget, grått hjørnestempel (samme idé som buildPageDateStamp, men nederst til VENSTRE i
// stedet for høyre, så de to stemplene ikke kolliderer) - bygget kun én gang og vist på ALLE sider (ikke
// bare normal-siden der selve feltet ligger, se buildAllPrintPages), siden nummeret gjelder hele
// oppdraget/utskriften, ikke bare normal-fanens egen sjekkliste. Returnerer null (ikke et tomt stempel)
// når feltet er tomt - samme "vis kun det som faktisk finnes"-prinsipp som resten av utskriften.
function buildApprovalStamp(approvalNumber) {
    if (!approvalNumber || !approvalNumber.trim()) return null;
    const el = document.createElement("div");
    el.className = "print-page-approval";
    el.textContent = "FFI-UAS-" + approvalNumber.trim();
    return el;
}

// DD.MM.ÅÅÅÅ med ledende null - toLocaleDateString("no-NO") gir riktig rekkefølge, men ikke nødvendigvis
// ledende null på ettsifret dag/måned (f.eks. "6.8.2026" i stedet for "06.08.2026").
function formatDateNorwegian(date) {
    const dd = String(date.getDate()).padStart(2, "0");
    const mm = String(date.getMonth() + 1).padStart(2, "0");
    return dd + "." + mm + "." + date.getFullYear();
}

// Filnavnbasis (UTEN filendelse) delt av BÅDE JSON- og PDF-nedlasting - brukerønske: "PDF som lagres.
// samme filnavn som json. bare med pdf etternavnet selvfølgelig". Egen, delt funksjon i stedet for å
// bygge samme mønster to steder (nedtrekksmeny-lytteren for JSON og downloadPdf under, se
// DOMContentLoaded) - unngår at de to skulle kunne drive fra hverandre over tid.
// "Sjekkliste-<drone>-DD-MM-ÅÅÅÅ" (dronenavnet droppes helt når feltet er tomt, ikke en misvisende
// fallback-tekst) - se downloadJsonBtn-lytteren for opprinnelig begrunnelse av selve mønsteret.
function buildExportFilenameBase(state) {
    const slug = function (s) { return (s || "").trim().replace(/\s+/g, "_"); };
    const dronePart = slug(state.normal && state.normal.drone);
    const datePart = formatDateNorwegian(new Date()).replace(/\./g, "-");
    return ["Sjekkliste"].concat(dronePart ? [dronePart] : []).concat([datePart]).join("-");
}

// Ett datostempel nede i høyre hjørne av selve arket (.print-page har position:relative) - vises kun
// én gang per side, ikke én gang per halvdel på den delte contingency/emergency-siden.
function buildPageDateStamp() {
    const dateEl = document.createElement("div");
    dateEl.className = "print-page-date";
    dateEl.textContent = formatDateNorwegian(new Date());
    return dateEl;
}

// Bygger print-boksene (utstyr/begrensninger utelatt - kun sjekklistedelene) for én sjekkliste. For
// normal-fanen filtreres punkter etter aktivt malvalg (MÅK/spesifikk) først - punkter merket for det
// andre malvalget skal ikke dukke opp i utskriften, samme regel som CSS-filtreringen på skjermen.
function buildSectionBoxesForPrint(tabKey, data) {
    const showCheckbox = ITEM_LABELS[tabKey].checkbox;
    // BUG-historikk (rapportert av brukeren, med skjermbilder - "Tap av C2 link" viste ALLE 9 punktene på
    // selve nettsiden, MEN "Ingen sjekkpunkter" i PDF-forhåndsvisningen, stikk motsatt; deretter fortsatt
    // 9 punkter på skjermen selv etter hard refresh) - filtreringen leste TIDLIGERE data.template, som
    // OGSÅ representerte normal-fanens egen "egendefinert"-tilstand (satt automatisk av markNormalCustom()
    // ved ENHVER redigering på normal-fanen, se der) - en helt urelatert redigering flippet dermed
    // filtreringen for BÅDE normal og contingency til "custom" som en utilsiktet sideeffekt, og
    // "hard refresh" endret ingenting siden det bare lastet den samme (allerede lagrede) "custom"-
    // tilstanden på nytt fra localStorage. Leser nå data.category i stedet - et EGET felt som KUN
    // oppdateres når brukeren faktisk velger MÅK/spesifikk (se setActiveCategory/getState), aldri av
    // markNormalCustom - filtreringen er dermed uavhengig av om normal-fanens eget innhold er tilpasset.
    const template = data.category || "mak";
    const aircraft = data.aircraft || DEFAULT_AIRCRAFT;
    const boxes = [];
    (data.sections || []).forEach(function (section) {
        // Variant-filtreringen gjaldt FØR kun normal-fanen (den eneste som hadde mak/spesifikk-merkede
        // punkter). Nå har contingency-fanen også slike (se "Tap av C2 link" i DEFAULTS.contingency), så
        // filteret kjøres nå uansett tabKey - harmløst for faner uten variant-merkede punkter (it.variant
        // er da alltid tom, og filteret slipper gjennom alt akkurat som før). aircraft-filteret (nytt)
        // følger samme prinsipp - harmløst for punkter uten it.aircraft satt.
        const originalItemCount = (section.items || []).length;
        const items = (section.items || []).filter(function (it) {
            if (it.variant) {
                if (template === "mak" || template === "spesifikk") {
                    if (it.variant !== template) return false;
                }
            }
            if (it.aircraft && it.aircraft !== aircraft) return false;
            return true;
        });
        // Har filtreringen fjernet SAMTLIGE punkter i en del som opprinnelig HADDE punkter (f.eks. et
        // fartøyspesifikt contingency-punkt vist for feil fartøytype) - hopp over hele delen i stedet
        // for å vise en tom "Ingen sjekkpunkter"-boks for noe som rett og slett ikke gjelder akkurat nå.
        // (En del som var tom FRA FØR, uten noen egne punkter i det hele tatt, vises fortsatt som normalt
        // - se buildSectionBox sin egen "Ingen sjekkpunkter"-visning for DEN situasjonen.)
        if (originalItemCount > 0 && !items.length) return;
        const printSection = { title: section.title, items: items, singleColumn: section.singleColumn, icon: section.icon };
        const sectionBox = buildSectionBox(printSection, showCheckbox);
        if (sectionBox) {
            // Brukt av layoutNormalColumns/splitSectionBoxToFit til å bygge en DELT versjon av boksen
            // (se der) hvis den ikke får plass i sin helhet - selve DOM-boksen alene har ikke nok
            // informasjon til å gjenoppbygges mindre, siden f.eks. filtrerte punkter allerede er bakt inn.
            sectionBox.__printSection = printSection;
            sectionBox.__showCheckbox = showCheckbox;
            boxes.push(sectionBox);
        }
    });
    return boxes;
}

// Normal-siden: egen, frittstående A4-side med fast tittel "Sjekkliste" (drone/autorisasjonsnummer vises
// nå inni selve tittelbanneret, se buildPrintHeader), og et eksplisitt to-kolonners grid (sjekklistedeler
// fast til venstre, utstyr/begrensninger fast til høyre - se .builder-grid i css/style.css for samme
// oppdeling på skjermen).
// Returnerer null hvis fanen er helt tom (ingen utstyr, begrensninger eller sjekklistedeler) - samme
// "ingen tomme sider"-prinsipp som ERP-siden (se buildErpPage), i stedet for å skrive ut en hel side med
// kun teksten "Denne sjekklisten er tom." (f.eks. rett etter "Nullstill denne fanen").
function buildNormalPage(data) {
    const leftCol = document.createElement("div");
    leftCol.className = "print-col";
    const rightCol = document.createElement("div");
    rightCol.className = "print-col";

    if (data.equipment) {
        const eqBox = buildEquipmentBox(data.equipment);
        if (eqBox) rightCol.appendChild(eqBox);
    }
    if (data.limits) {
        const limitsBox = buildLimitsBox(data.limits);
        if (limitsBox) rightCol.appendChild(limitsBox);
    }
    // layoutNormalColumns (ikke et rent .forEach(...leftCol.appendChild) lenger) - brukerønske ("den
    // sjekklisteboksen... havner på neste side. Men her passer det vel bedre å ha den over på neste
    // kolonne når det er plass der?") - se kommentaren ved funksjonen (rett før combinedHalvesFit) for
    // hvordan den avgjør om en sjekklistedel som ikke får plass i venstre kolonne heller kan flyttes
    // nederst i høyre kolonne, i stedet for å alltid hoppe rett til en ny side.
    layoutNormalColumns(leftCol, rightCol, buildSectionBoxesForPrint("normal", data));

    if (!leftCol.children.length && !rightCol.children.length) return null;

    const page = document.createElement("div");
    page.className = "print-page";
    page.setAttribute("data-theme", "normal");
    // print-header-half (se css/style.css) - banneren skal IKKE strekke seg over hele sidebredden her,
    // siden arket brettes i to på langs rett mellom disse to grid-kolonnene (venstre/høyre under) - en
    // fullbredde-banner ville ligget rett over selve bretten.
    const header = buildPrintHeader("normal", data);
    header.classList.add("print-header-half");
    page.appendChild(header);

    const grid = document.createElement("div");
    grid.className = "print-grid";
    grid.appendChild(leftCol);
    grid.appendChild(rightCol);

    const body = document.createElement("div");
    body.className = "print-page-body";
    body.appendChild(grid);
    page.appendChild(body);
    return page;
}

function buildPrintSidebar(tabKey) {
    const label = TAB_META[tabKey].sidebarLabel;
    if (!label) return null;
    const sidebar = document.createElement("div");
    sidebar.className = "print-sidebar";
    label.split("").forEach(function (ch) {
        const span = document.createElement("span");
        span.textContent = ch;
        sidebar.appendChild(span);
    });
    return sidebar;
}

// Én halvdel av den delte contingency/emergency-siden (se buildCombinedPage) - egen funksjon fordi den
// også brukes til å MÅLE hvor høy halvdelen faktisk blir (se combinedHalvesFit) før man bestemmer om
// den får plass ved siden av den andre, eller om de to trenger hver sin fulle side i stedet.
function buildCombinedHalf(tabKey, data) {
    const half = document.createElement("div");
    half.className = "print-combined-half";
    half.setAttribute("data-theme", tabKey);
    half.appendChild(buildPrintHeader(tabKey, data));

    const columns = document.createElement("div");
    columns.className = "print-columns-half";
    const boxes = buildSectionBoxesForPrint(tabKey, data);
    if (!boxes.length) {
        const empty = document.createElement("p");
        empty.className = "print-empty-note";
        empty.textContent = "Denne sjekklisten er tom.";
        columns.appendChild(empty);
    } else {
        boxes.forEach(function (box) { columns.appendChild(box); });
    }

    const body = document.createElement("div");
    body.className = "print-page-body";
    body.appendChild(columns);

    const sidebar = buildPrintSidebar(tabKey);
    if (sidebar) body.appendChild(sidebar);

    half.appendChild(body);
    return half;
}

// Selve raden (contingency/emergency side ved side) - egen funksjon (tidligere inlinet i
// buildCombinedPage) slik at BÅDE selve siden og målingene (combinedHalvesFit/combinedAllFit) bygger
// nøyaktig samme markup, ikke to steder som kan drifte fra hverandre.
function buildCombinedRow(contingencyData, emergencyData) {
    const row = document.createElement("div");
    row.className = "print-combined-row";
    row.appendChild(buildCombinedHalf("contingency", contingencyData));
    row.appendChild(buildCombinedHalf("emergency", emergencyData));
    return row;
}

// Contingency og emergency deler én side, hver sin halvdel (med egen farget tittelbanner og sidefelt) -
// i stedet for hver sin egen fulle A4-side. .print-columns-half bruker column-width (ikke et fast
// column-count) slik at halvdelen selv kan falle tilbake til én kolonne i det trange rommet, eller
// bruke flere kolonner hvis det skulle være plass. Kalles kun når combinedHalvesFit sier det er nok
// plass (og combinedAllFit IKKE strakk til for å få ERP med også, se buildAllPrintPages) - ERP får da sin
// egen side i stedet, se buildErpPage.
function buildCombinedPage(contingencyData, emergencyData) {
    const page = document.createElement("div");
    page.className = "print-page print-page-combined";
    page.appendChild(buildCombinedRow(contingencyData, emergencyData));
    return page;
}

// Brukerønske ("Hvis det er plass kan ERP være på samme side som contingency og emergency. ta nedre
// halvdel av siden f.eks.") - én og samme side med contingency+emergency øverst (som buildCombinedPage)
// OG hele ERP-innholdet (sjekklistedeler + kontaktliste) stablet under, i stedet for at ERP alltid får
// sin egen fulle side for seg selv. Kalles kun når combinedAllFit sier alt tre faktisk får plass - ellers
// faller buildAllPrintPages tilbake til den vanlige to-delte combined-siden (eller enda lenger ned,
// separate sider) pluss en egen ERP-side, se der.
function buildCombinedTriplePage(contingencyData, emergencyData, erpData) {
    const page = document.createElement("div");
    page.className = "print-page print-page-combined";
    page.appendChild(buildCombinedRow(contingencyData, emergencyData));
    const erpBody = buildErpBody(erpData);
    if (erpBody) {
        erpBody.classList.add("print-combined-bottom");
        page.appendChild(erpBody);
    }
    return page;
}

// Fallback når contingency/emergency IKKE får plass side ved side (se combinedHalvesFit): egen,
// frittstående A4-side per fane, samme oppbygning som ERP-siden (full bredde, ikke halvdelt).
function buildSingleTabPage(tabKey, data) {
    const boxes = buildSectionBoxesForPrint(tabKey, data);
    if (!boxes.length) return null;

    const page = document.createElement("div");
    page.className = "print-page";
    page.setAttribute("data-theme", tabKey);
    // print-header-half (se kommentaren i buildNormalPage over, og css/style.css) - samme brette-hensyn
    // gjelder her, selv om .print-columns under er en auto-flytende (ikke fast delt) tokolonners layout.
    const header = buildPrintHeader(tabKey, data);
    header.classList.add("print-header-half");
    page.appendChild(header);

    const columns = document.createElement("div");
    columns.className = "print-columns";
    boxes.forEach(function (box) { columns.appendChild(box); });

    const body = document.createElement("div");
    body.className = "print-page-body";
    body.appendChild(columns);

    const sidebar = buildPrintSidebar(tabKey);
    if (sidebar) body.appendChild(sidebar);

    page.appendChild(body);
    return page;
}

// Måler (i et skjult, men layoutet "probe"-element - display:none-elementer kan ikke måles) om begge
// halvdelene faktisk får plass innenfor det som er igjen av arkhøyden når header/dato er trukket fra.
// mm er en absolutt CSS-lengdeenhet (96px/25.4mm) og regnes likt om på skjerm som i utskrift, så et mål
// tatt på skjermen (usynlig, plassert utenfor synsfeltet) stemmer overens med faktisk trykk. Terskelen
// er satt med noe margin siden font-rendering kan variere marginalt mellom nettlesere/OS.
const MM_TO_PX = 96 / 25.4;
// HEVET fra 210 (brukerønske: "contingency, emergency og ERP må få plass på samme side... som før") -
// samme "et for lavt tak feilbedømmer ekte innhold som 'får ikke plass'"-resonnement som
// NORMAL_COLUMN_MAX_HEIGHT_MM over: bekreftet empirisk (constrainCombinedColumnsForMeasurement ga en
// realistisk måling på ~228mm for et normalt Contingency+Emergency-innhold, mot det gamle 210mm-taket)
// at 210 rett og slett var for stramt til vanlig innhold, ikke en beskyttelse mot noen reell
// Chrome-pagineringsbug (denne siden bruker CSS multi-kolonne, ikke float/flow-root, se
// .print-columns-half i css/style.css - en annen mekanisme enn den som rammet Normal-siden).
const COMBINED_HALF_MAX_HEIGHT_MM = 260;
// Full sidebredde tilgjengelig for INNHOLD (A4 210mm - @page-margen 15mm x 2 - .print-page sin egen
// padding 2mm x 2, se css/style.css) - samme breddemål probene under (og selve den ferdige siden) faktisk
// får, slik at en probe-måling stemmer overens med virkeligheten.
const PRINT_CONTENT_WIDTH_MM = 176;
// .print-page sin egen ytre bredde (INNHOLD + dens 2mm padding på hver side) - brukt til å gi
// createMeasurePage-riggen under samme reelle bredde som #printView faktisk har mellom @page-margene,
// slik at .print-page sin egen (ellers udefinerte - den arver bare 100% av #printView) bredde blir
// riktig under målingen, akkurat som når den faktisk skrives ut.
const PRINT_PAGE_WIDTH_MM = PRINT_CONTENT_WIDTH_MM + 4;
// Tilgjengelig høyde for INNHOLDET i én kolonne (277mm sideminstehøyde - nå 10mm topp-/bunnmarg på
// @page, se style.css - 4mm topp-/10mm bunnpadding på .print-page - ca. 18-19mm for selve
// tittelbanneret+dens egen bunnmarg, se buildPrintHeader/.print-header i css/style.css) - med noe
// margin, samme "font-rendering kan variere marginalt"-begrunnelse som COMBINED_HALF_MAX_HEIGHT_MM
// under.
// HISTORIKK: senket til 190 i en tidligere runde (rapportert: en 19-punkts FX-10-del ("Før avgang")
// havnet alene på en helt egen side) - men det viste seg IKKE å være et kalibreringsproblem i det hele
// tatt: den faktiske feilen var at overflow-boksene ble vurdert som ÉN alt-eller-ingenting-bunt i
// stedet for hver for seg (se den grådige boks-for-boks-løkken i layoutNormalColumns under) - en liten
// "... forts."-boks ble dermed avvist sammen med to STØRRE bokser den var bundet til, selv om den
// alene ville fått fint plass. Med den reelle feilen rettet var 190 unødvendig strengt (rapportert:
// en kort "Etter landing"-del (4 punkter) ble likevel dyttet til en nesten tom side 2, til tross for
// tydelig synlig ledig plass i høyre kolonne på side 1) - hevet tilbake til nær opprinnelig 220, pluss
// de 10mm ekstra sidehøyde marg-reduksjonen over nå gir.
// HISTORIKK-runde 2: forsøkt hevet til 245 (fra 230) etter at både <table>-utskriftsbuggen (se
// buildPrintRow) OG selve probe-strukturen (se createColumnMeasureRig) ble rettet ved roten - selve
// MÅLINGEN var da endelig presis nok til at 230mm+15mm-marginen så unødvendig streng ut (rapportert:
// venstre kolonne fikk kun plass til ÉN sjekklistedel, resten dyttet til høyre kolonne - markant
// misforhold, om enn fortsatt korrekt LESEREKKEFØLGE). 245 GJENÅPNET (bekreftet empirisk, rendret
// faktisk gjennom en skjult nettleser) den ORIGINALE "foreldreløs overskrift"-buggen på Normal-siden -
// denne gangen UTEN noen <table> involvert, altså en HELT EGEN, udokumentert Chromium-paginerings-
// svakhet. break-after:avoid på selve banneret (også testet empirisk) hadde INGEN effekt.
// HISTORIKK-runde 3: videre empirisk testing (samme skjult-nettleser-metode) avdekket at buggen ikke er
// knyttet til selve TERSKELVERDIEN, men til den RESULTERENDE fordelingen mellom kolonnene den gir: en
// "medium" ubalanse (her: venstre kolonne rundt 215mm, høyre rundt 358mm - ingen av dem åpenbart kort
// eller åpenbart dominerende) er nettopp det Chrome sitt pagineringsoppsett håndterer dårlig. En EKSTREM
// ubalanse derimot - enten en kort venstre/lang høyre (bekreftet trygt, se den opprinnelige 230mm-
// testen) ELLER en lang venstre/kortere høyre (bekreftet trygt her, med et mye høyere tak) - lar Chrome
// paginere korrekt. 400mm er derfor bevisst satt HØYT (godt over det noen reell sjekklistedel-samling
// trenger på égen hånd) - IKKE fordi én kolonne noensinne trenger så mye plass i praksis, men for å la
// algoritmen fritt finne det NATURLIGE stoppunktet (der neste hele del rett og slett ikke får plass i det
// hele tatt) i stedet for å kunstig kutte den av midt i den farlige "medium"-sonen. Blir én kolonne likevel
// for høy for én fysisk side i praksis, faller den tilbake til vanlig, allerede bekreftet trygg
// side-2-overflow (samme flyt-mekanisme som gjorde det opprinnelige 230mm-taket trygt).
const NORMAL_COLUMN_MAX_HEIGHT_MM = 400;

// Brukerønske ("Når [en sjekklistedel] blir veldig lang, må den på printen få fortsette i neste kolonne
// på samme side. Ikke havne på en helt ny side. Og kanskje da... få som overskriftslinje 'Før avgang
// forts.'") - en enkelt sjekklistedel kan bli lengre enn det som er igjen i venstre kolonne (f.eks. en
// fartøyspesifikk "Før avgang" med 15-20 punkter, se AIRCRAFT_CONTENT.fx10), og da er den nesten
// uunngåelig LENGRE enn hele resten av venstre kolonnes eget budsjett også isolert - layoutNormalColumns
// sitt vanlige "flytt HELE boksen til høyre kolonne hvis den ikke får plass i venstre" (se under) løser
// dermed IKKE dette tilfellet alene, siden boksen fortsatt ikke får plass i sin helhet noe sted. Splitter
// derfor akkurat DEN boksen som først overskrider budsjettet i to - se splitSectionBoxToFit - i stedet
// for å la den forbli udelt og falle igjennom til gammel "hopp til ny side"-oppførsel.
// Binærsøk ville vært raskere, men et enkelt lineært søk (fra flest til færrest punkter) er mer enn
// raskt nok for realistiske sjekklistedel-lengder (typisk under 20 punkter) og enklere å resonnere
// riktig om. Returnerer null (ingen deling mulig) hvis ikke engang ÉTT punkt pluss overskriften får
// plass i det tilgjengelige rommet - kalleren faller da tilbake til den gamle "hele boksen flyttes/
// overflower"-oppførselen, aldri verre enn før denne utvidelsen.
//
// probe MÅ være DET SAMME probe-elementet som kalleren allerede har brukt til å bekrefte at boksene
// FØR denne (0..i-1, se layoutNormalColumns) faktisk får plass - IKKE et eget, tomt probe-element med
// etterfølgende subtraksjons-regnestykke (maxHeightPx - allerede brukt) slik denne funksjonen gjorde
// tidligere. BUG (rapportert av brukeren, med skjermbilde: en splittet dels "forts."-halvdel dukket opp
// PÅ SIDE 1, mens hoveddelen (lavere numre) først dukket opp øverst på SIDE 2 - stikk motsatt
// leserekkefølge) - subtraksjon antar at høyder er rent additive, men CSS-marger mellom tilstøtende
// bokser (se .print-box sin margin-bottom) kollapser/oppfører seg ikke alltid helt likt i en ISOLERT
// måling som i den FAKTISKE, kumulative flyten - et par mm avvik der var nok til at en kandidat denne
// funksjonen trodde fikk plass, likevel overflowet til side 2 når den faktisk ble limt inn sammen med
// alt det andre i venstre kolonne. Ved i stedet å måle kandidaten OPPÅ nøyaktig den samme, allerede
// fylte proben (samme DOM-kontekst, ingen egen isolert måling å avvike fra) matcher denne funksjonens
// "får plass"-vurdering nøyaktig den samme kumulative konteksten resten av venstre kolonne allerede ble
// godkjent i.
// PRINT_FIT_SAFETY_MARGIN_MM: ekstra buffer for "får plass"-avgjørelser i layoutNormalColumns - BÅDE
// for selve delingsgrensen (splitSectionBoxToFit under) OG for den vanlige hele-boks-avgjørelsen i
// hovedløkken (se der). Rapportert FLERE GANGER nå, med skjermbilde hver gang - senest en HEL, USPLITTET
// "Etter avgang" (kun 2 punkter) som proben mente fikk plass i venstre kolonne, men som likevel landet
// alene på en egen side ved faktisk utskrift. Først antatt å KUN gjelde delingsgrensen spesifikt (se
// forrige runde), men et helt UDELT, lite punkt som feilbedømmes samme vei tyder på at avviket sitter i
// selve MÅLETEKNIKKEN (mistanke: proben er en ren, absolutt-posisjonert div, mens den VIRKELIGE kolonnen
// er et float-element inni flow-root/print-grid - subtile forskjeller i hvordan nettleseren løser bredde/
// høyde-beregning der er trolig involvert), ikke noe som er unikt for delte bokser. Gjelder derfor nå
// BEGGE stedene. Bevisst asymmetrisk: å vurdere noe som "ikke får plass" én rad for tidlig er en uskyldig
// kosmetisk detalj (havner i høyre kolonne/neste side i stedet, fortsatt korrekt LESEREKKEFØLGE), mens å
// vurdere det feil andre veien gir nettopp den forvirrende, FEILAKTIGE leserekkefølgen som er rapportert.
// Hevet fra 8 til 15 (brukerens NYESTE skjermbilde: en "Etter landing"-del ble delt i fitted+forts. der
// selve den ANGIVELIG "fitted" halvdelen (rad 1-2) likevel overfløt til side 2 ved faktisk utskrift, mens
// "forts."-halvdelen (rad 3-4, allerede plassert i høyre kolonne av steg 3 under) ble stående igjen alene
// på side 1 - nøyaktig samme "leserekkefølge baklengs"-symptom som denne marginen opprinnelig ble innført
// for å unngå (se kommentaren over), bare at 8mm viste seg utilstrekkelig for et tilfelle med FLERE
// sjekklistedeler foran seg i samme kolonne (avviket her er trolig kumulativt - hver tidligere del i
// venstre kolonne bidrar med sitt eget lille, isolert-probe-vs-faktisk-flyt-avvik, og med nok deler foran
// overstiger summen 8mm selv om ÉN enkelt del alene ikke ville gjort det).
// HISTORIKK-runde 2: forsøkt senket til 5 (fra 15), sammen med å heve NORMAL_COLUMN_MAX_HEIGHT_MM - se
// den store historikk-kommentaren ved den konstanten for hvorfor begge MÅTTE REVERTERES: den "isolerte
// probe vs. virkelig kontekst"-mistanken over var riktig og ER rettet ved roten (createColumnMeasureRig),
// og en ekte <table>-utskriftsbug (buildPrintRow) er OGSÅ rettet - men selv med begge rettet, og selv om
// den TALLMESSIGE målingen nå er presis, oppfører selve Chrome-utskriftsmotoren seg fortsatt
// uforutsigbart (bekreftet empirisk) når innholdet presses nær den reelle sidegrensen: "foreldreløs
// overskrift"-buggen kom tilbake, uten noen tabell involvert denne gangen. 15mm er derfor beholdt - ikke
// lenger som plaster på en unøyaktig måling, men som bevisst slakk mot en Chrome-svakhet målingen alene
// ikke kan forutsi eller kompensere for.
const PRINT_FIT_SAFETY_MARGIN_MM = 15;
// BUG (rapportert av brukeren: "sjekkliste Etter landing starter nederst i kolonnen med bare et par
// linjer. da bør den heller starte på ny kolonne. unngå oppdeling av tabellene") - løkken under prøvde
// tidligere n helt ned til 1, altså aksepterte den også en "fitted"-del med kun ÉTT punkt hvis akkurat
// det var alt som var igjen av plass - en slik minimal flis nederst i kolonnen ser ut som et avkuttet
// avsnitt, ikke en meningsfull "del 1 av 2". Splitting er ment for LANGE deler der mesteparten allerede
// får plass og bare et par punkter spiller over (se kommentaren ved funksjonen over) - ikke for å
// klemme en håndfull linjer inn i en fillern rest av kolonnen. MIN_SPLIT_FRACTION krever nå at den
// "fittede" delen faktisk utgjør en fornuftig ANDEL av hele seksjonen (minst ca. en tredjedel, og minst
// to punkter) - er det ikke plass til det, returneres null (ingen deling), og HELE boksen flyttes i
// stedet til neste kolonne/side via den vanlige overflow-håndteringen i layoutNormalColumns, nøyaktig
// oppførselen brukeren ba om.
const MIN_SPLIT_FRACTION = 1 / 3;
function splitSectionBoxToFit(printSection, showCheckbox, probe, maxHeightPx) {
    const items = printSection.items;
    if (items.length < 2) return null;
    const minFittedItems = Math.max(2, Math.ceil(items.length * MIN_SPLIT_FRACTION));
    const safeMaxHeightPx = maxHeightPx - PRINT_FIT_SAFETY_MARGIN_MM * MM_TO_PX;
    let result = null;
    for (let n = items.length - 1; n >= minFittedItems; n--) {
        const candidate = buildSectionBox({ title: printSection.title, items: items.slice(0, n), singleColumn: printSection.singleColumn, icon: printSection.icon }, showCheckbox);
        if (!candidate) continue;
        probe.appendChild(candidate);
        const fits = probe.scrollHeight <= safeMaxHeightPx;
        probe.removeChild(candidate);
        if (fits) {
            const remainder = buildSectionBox(
                { title: (printSection.title || "Uten navn") + " forts.", items: items.slice(n), singleColumn: printSection.singleColumn, icon: printSection.icon },
                showCheckbox,
                n + 1 // fortsetter nummereringen der "candidate" (rad 1..n) slapp, se buildSectionBox
            );
            result = { fitted: candidate, remainder: remainder };
            break;
        }
    }
    return result;
}

// Brukerønske ("jeg lurer på overflowen til den sjekklisteboksen. den havner på neste side. Men her passer
// det vel bedre å ha den over på neste kolonne når det er plass der?") - venstre kolonne (sjekklistedeler)
// kan bli høyere enn det som er igjen av arkhøyden, mens høyre kolonne (utstyr+begrensninger) ofte er langt
// kortere og har ledig plass igjen nederst. I stedet for å la sjekklistedelen(e) som ikke får plass hoppe
// rett til en ny, nesten tom side, måles (samme skjulte probe-teknikk som combinedHalvesFit/combinedAllFit
// under) om de heller får plass i høyre kolonne. KUN de sjekklistedelene som faktisk ikke får plass i
// venstre kolonne flyttes (aldri en fri, auto-balanserende flyt av ALT innhold på tvers av kolonnene - se
// combinedHalvesFit-kommentaren for hvorfor ren CSS-multikolonne unngås i utskrift), uendret rekkefølge på
// sjekklistedelene seg imellom.
// Brukerpresisering ("den sjekklisten som overflower skal komme OVER utstyrsliste og begrensninger. utstyr
// og begrensninger skal ikke være øverst hvis det er sjekklisteboks i samme kolonne") - overflow-boksene
// settes derfor inn ØVERST i høyre kolonne (foran utstyr/begrensninger), ikke nederst. Sjekklistedelen(e)
// er det aktive, tidskritiske innholdet - utstyr/begrensninger er ren referanseinfo som naturlig kan stå
// under når kolonnen må deles.
// Får overflow-boksene heller ikke plass i høyre kolonne (sjelden, men mulig hvis begge kolonner allerede
// er nesten fulle), beholdes de i venstre kolonne som før denne fiksen - samme "hopp til ny side"-oppførsel
// som tidligere, aldri verre enn utgangspunktet.
// BUG-historikk (rapportert flere ganger, senest: "ERP kom nå sammen med overskriften sin, men på en
// helt egen side, selv om det er en halv side ledig plass på forrige side", og "overskriften til
// sjekklisten helt øverst for seg selv alene på en side") - mistanken uttrykt ved PRINT_FIT_SAFETY_MARGIN_MM
// over ("proben er en ren, absolutt-posisjonert div, mens den VIRKELIGE kolonnen er et float-element inni
// flow-root/print-grid") stemmer: en probe som bare kopierer BREDDEN for hånd, men verken selve
// klassekjeden (.print-page > .print-page-body > .print-grid > .print-col) eller float-konteksten disse
// klassene faktisk styrer (margin-collapse, box-sizing osv.), måler et lett ANNET tre enn det som til
// slutt skrives ut - derav de stadig tilbakevendende, retningsløse avvikene selv etter gjentatte
// marginjusteringer. Bygger nå en skjult, men STRUKTURELT IDENTISK kopi av selve sidehierarkiet å måle i
// i stedet - se createColumnMeasureRig - slik at CSS-en som faktisk STYRER høyden (float-bredden, flow-
// root-konteksten osv.) er nøyaktig den samme under målingen som ved selve utskriften, i stedet for en
// isolert tilnærming som må kompenseres for med stadig voksende "sikkerhetsmarginer".
function createColumnMeasureRig() {
    const page = document.createElement("div");
    page.className = "print-page";
    // BUG (funnet ved faktisk å rendre en test-JSON gjennom en skjult nettleser og lese PDF-en - se
    // git-historikken for dette funnet - eksakt samme "lesrekkefølge baklengs"-symptom som tidligere
    // rapportert dukket fortsatt opp: "Etter landing"-delens fitted FØRSTE halvdel (uten "forts.") havnet
    // alene på side 2, mens "forts."-halvdelen havnet FØR den, i kolonne 2 på side 1) - denne riggen
    // manglet en eksplisitt bredde (i motsetning til createMeasurePage under). .print-page har ingen egen
    // CSS-bredde (arver normalt 100% av #printView) - en position:absolute boks UTEN bredde OG uten
    // "right" satt får i stedet shrink-to-fit-bredde etter spec, som for en boks med prosent-baserte barn
    // (.print-col er 50% av .print-grid, som er 100% av denne) faller tilbake til den TILGJENGELIGE
    // bredden (i praksis nær hele viewportet) - langt bredere enn de virkelige 180mm siden faktisk får
    // mellom @page-margene. En bredere kolonne bryter tekst til FÆRRE linjer, så målingen ble
    // systematisk for LAV sammenlignet med virkeligheten - stikk motsatt av den brede, kompenserende
    // sikkerhetsmarginen (PRINT_FIT_SAFETY_MARGIN_MM) dette skulle vært en STRUKTURELL erstatning for.
    page.style.cssText = "position:absolute; visibility:hidden; left:-9999px; top:0; width:" + PRINT_PAGE_WIDTH_MM + "mm;";
    const body = document.createElement("div");
    body.className = "print-page-body";
    const grid = document.createElement("div");
    grid.className = "print-grid";
    const col1 = document.createElement("div");
    col1.className = "print-col";
    const col2 = document.createElement("div");
    col2.className = "print-col";
    grid.appendChild(col1);
    grid.appendChild(col2);
    body.appendChild(grid);
    page.appendChild(body);
    document.body.appendChild(page);
    return { page: page, col1: col1, col2: col2 };
}

function layoutNormalColumns(leftCol, rightCol, sectionBoxes) {
    if (!sectionBoxes.length) return;
    const maxHeightPx = NORMAL_COLUMN_MAX_HEIGHT_MM * MM_TO_PX;
    const rig = createColumnMeasureRig();
    // col1 måler venstre kolonne (sjekklistedeler), col2 måler høyre (utstyr/begrensninger + evt.
    // overflow) - to EKTE, side-ved-side flytende .print-col i samme .print-grid, akkurat som den
    // ferdige siden, i stedet for én gjenbrukt, isolert probe-div.
    const leftProbe = rig.col1;
    const rightProbe = rig.col2;

    // 1) Høyre kolonnes egen, faste høyde (utstyr+begrensninger, allerede satt inn av buildNormalPage) -
    // FLYTTES inn i rightProbe (appendChild FLYTTER, ikke kopierer) og blir STÅENDE der gjennom resten av
    // funksjonen (ikke bare målt og lagt tilbake med det samme som før) - trinn 3 under måler dermed en
    // overflow-kandidat OPPÅ nøyaktig samme, allerede fylte kontekst, i stedet for én isolert høyde-måling
    // etterfulgt av subtraksjonsregning (samme "additiv"-antakelse som splitSectionBoxToFit-kommentaren
    // over advarer mot - CSS-marger kollapser ikke alltid rent additivt).
    const rightBoxes = Array.from(rightCol.children);
    rightBoxes.forEach(function (box) { rightProbe.appendChild(box); });

    // 2) Hvor mange sjekklistedeler (i rekkefølge, fra toppen) får plass i venstre kolonne innenfor
    // sidebudsjettet? Boksene som allerede er bekreftet å få plass BLIR STÅENDE i leftProbe mens de
    // følgende testes - splitSectionBoxToFit trenger nettopp DENNE kumulative konteksten for å måle en
    // delt kandidat presist, se kommentaren der.
    const safeMaxHeightPx = maxHeightPx - PRINT_FIT_SAFETY_MARGIN_MM * MM_TO_PX;
    let fitCount = sectionBoxes.length;
    let splitResult = null; // { index, fitted, remainder } - satt kun hvis boksen som stanset skanningen faktisk lot seg dele
    for (let i = 0; i < sectionBoxes.length; i++) {
        leftProbe.appendChild(sectionBoxes[i]);
        if (leftProbe.scrollHeight > safeMaxHeightPx) {
            leftProbe.removeChild(sectionBoxes[i]); // leftProbe inneholder nå nøyaktig boksene 0..i-1, se over
            const meta = sectionBoxes[i].__printSection;
            const split = meta ? splitSectionBoxToFit(meta, sectionBoxes[i].__showCheckbox, leftProbe, maxHeightPx) : null;
            fitCount = i;
            if (split) splitResult = { index: i, fitted: split.fitted, remainder: split.remainder };
            break;
        }
    }

    // leftProbe inneholder nå nøyaktig de boksene som fikk plass (0..fitCount-1) - flyttes rett over i
    // den EKTE venstre kolonnen i samme rekkefølge (appendChild flytter), i stedet for å hente dem på nytt
    // fra sectionBoxes via indeks.
    Array.from(leftProbe.children).forEach(function (box) { leftCol.appendChild(box); });

    let overflowing;
    if (splitResult) {
        // Selve originalboksen (sectionBoxes[splitResult.index]) er nå fullstendig erstattet av de to
        // nye, mindre boksene - den brukes ikke videre og havner verken i venstre eller høyre kolonne.
        leftCol.appendChild(splitResult.fitted);
        overflowing = [splitResult.remainder].concat(sectionBoxes.slice(splitResult.index + 1));
    } else {
        overflowing = sectionBoxes.slice(fitCount);
    }

    if (overflowing.length) {
        // 3) Får overflow-delen(e) plass i høyre kolonne, sammen med det som allerede står der (utstyr/
        // begrensninger)? GRÅDIG, boks for boks - prøver hver overflow-boks for seg, i rekkefølge, og
        // stopper ved FØRSTE som ikke får plass. Målt DIREKTE i rightProbe (som allerede inneholder
        // utstyr/begrensninger fra trinn 1) - ingen egen isolert probe2 eller
        // "maxHeightPx - rightUsedPx"-subtraksjon lenger.
        //
        // BUG (rapportert av brukeren, med skjermbilde: "på operasjonsområdet" - andre delen i rekken -
        // dukket opp NEDERST i høyre kolonne, mens "før avgang"/"overvåking under flyging"/"etter
        // landing" (tredje/fjerde/femte delen) fortsatt sto i venstre kolonne OVER den i lesevisningen -
        // stikk motsatt leserekkefølge) - resten (de som IKKE fikk plass i høyre kolonne) ble tidligere
        // sendt TILBAKE til venstre kolonne (naturlig side-2-overflow der) i stedet for å bli værende i
        // høyre kolonne. Siden overflow-boksene alltid er en SAMMENHENGENDE hale av opprinnelig
        // rekkefølge (sectionBoxes[fitCount..]; se over), og kolonne 1 allerede er lest FØR kolonne 2 i
        // naturlig lesevisning, MÅ en tidligere del i denne halen (f.eks. "på operasjonsområdet") aldri
        // havne i venstre kolonne mens en SENERE del (f.eks. "før avgang") blir værende i høyre kolonne -
        // det garanterte nettopp IKKE forrige oppførsel. Retten fikser dette ved i stedet å la ALT som
        // ikke fikk plass i selve probe-budsjettet BLI VÆRENDE i høyre kolonne også - naturlig
        // side-2-overflow der fungerer akkurat som i venstre kolonne (samme float-paginering, se
        // .print-col-kommentaren i css/style.css), og rekkefølgen forblir dermed korrekt uansett hvor
        // mye som faktisk får plass på selve side 1.
        const rightAnchor = rightProbe.firstChild; // først av de OPPRINNELIGE rightBoxes, før overflow settes inn foran dem
        overflowing.forEach(function (box) { rightProbe.insertBefore(box, rightAnchor); });
        // Brukerønske ("er det plass i kolonnen så kan det være litt mer luft under siste
        // sjekklistedelen... siden disse delene ikke direkte er del av normale sjekklister") - når en
        // overflow-boks havner rett over utstyr/begrensninger (rightAnchor finnes, dvs. de opprinnelige
        // rightBoxes ikke var tomme), får den SISTE av dem litt ekstra luft ned til den referanseinfo-
        // boksen (Utstyrsliste/Begrensninger/Kontaktliste) som følger - markerer det semantiske skillet
        // mellom faktiske sjekklistepunkter og ren referanseinfo, se .print-box-checklist-end i
        // css/style.css.
        if (rightAnchor) overflowing[overflowing.length - 1].classList.add("print-box-checklist-end");
    }

    // rightProbe inneholder nå den endelige høyre kolonnen (opprinnelig utstyr/begrensninger, med
    // eventuelle overflow-bokser satt inn øverst, foran dem - brukerpresisering: sjekklisteboksen som
    // overflower skal komme OVER utstyr/begrensninger) - flyttes samlet til den EKTE høyre kolonnen.
    Array.from(rightProbe.children).forEach(function (box) { rightCol.appendChild(box); });

    document.body.removeChild(rig.page);
}

// Skjult, men reelt IDENTISK .print-page (samme klasse/padding/bredde som selve utskriften bruker, se
// css/style.css) å måle i - i stedet for en flat probe-div der kun bredden er kopiert inn for hånd (se
// createColumnMeasureRig-kommentaren for samme "isolert probe vs. virkelig kontekst"-resonnement).
// .print-page har ingen egen CSS-bredde (arver 100% av #printView, som ved faktisk utskrift er begrenset
// av @page-margene) - PRINT_PAGE_WIDTH_MM gir riggen samme reelle bredde eksplisitt.
function createMeasurePage() {
    const page = document.createElement("div");
    page.className = "print-page";
    page.style.cssText = "position:absolute; visibility:hidden; left:-9999px; top:0; width:" + PRINT_PAGE_WIDTH_MM + "mm;";
    document.body.appendChild(page);
    return page;
}

// BUG (funnet ved å faktisk rendre en test-JSON gjennom en skjult nettleser og lese PDF-en - se
// git-historikken for dette funnet) - .print-columns-half bruker CSS multi-kolonne (column-width, ikke
// floats som resten av utskriften, se .print-columns-half-kommentaren i css/style.css) med
// column-fill:auto - column-fill:auto er SPESIFIKT laget for PAGINERTE kontekster (der elementets egen
// høyde er begrenset av selve den fysiske siden, slik det alltid er ved faktisk utskrift) og har
// upraktisk/udefinert oppførsel når høyden er UBEGRENSET (auto), som denne isolerte proben ellers ville
// gitt: uten en eksplisitt høyde å fylle IMOT, endte innholdet opp i ÉN altfor høy "kolonne" i stedet for
// å fordele seg over flere slik det ville gjort på ekte utskrift - target-halvdelenes scrollHeight ble
// dermed grovt OVERvurdert (målt empirisk: nesten hele arkhøyden, for et par korte sjekklistedeler som
// tydelig skulle hatt god plass). Gir derfor hver .print-columns-half en eksplisitt høyde lik budsjettet
// FØR målingen (samme høyde column-fill:auto uansett ville fått fra selve sidefragmenteringen ved ekte
// utskrift) - scrollHeight (som fortsatt fanger opp reelt overflow forbi denne satte høyden, siden
// overflow er visible, ikke hidden) gir da en presis "får faktisk plass"-avgjørelse i stedet for en
// grovt oppblåst en.
// COMBINED_HALF_COLUMN_FILL_PROBE_MM - IKKE selve plassbudsjettet (se COMBINED_HALF_MAX_HEIGHT_MM under
// for det) - kun en høyde å GI column-fill:auto å fylle imot under selve MÅLINGEN. Empirisk oppdaget
// (rendret faktisk gjennom en skjult nettleser, gjentatte ganger med ulike verdier): jo HØYERE denne
// settes, jo FÆRRE kolonner velger column-fill:auto å bruke (den fyller "kolonne 1" lenger før den gir
// opp og går videre) - og FÆRRE, høyere kolonner gir en STØRRE total målt scrollHeight, ikke mindre
// (motsatt av hva man skulle tro). En LAV verdi her (150mm - godt under selve budsjettet) tvinger
// algoritmen til å faktisk bruke FLERE av kolonnene bredden gir plass til, og gir dermed en langt mer
// realistisk (lavere) samlet høyde - nærmere det ekte utskrift-fragmenterte resultatet enn en "fysisk
// riktig" full sidehøyde ga (bekreftet: 260mm ga scrollHeight 278mm - FEIL, tydelig for høyt for et par
// korte sjekklistedeler - mens 150mm ga 168mm, som stemte overens med det faktiske utskriftsresultatet).
const COMBINED_HALF_COLUMN_FILL_PROBE_MM = 150;
function constrainCombinedColumnsForMeasurement(row) {
    row.querySelectorAll(".print-columns-half").forEach(function (cols) {
        cols.style.height = COMBINED_HALF_COLUMN_FILL_PROBE_MM + "mm";
    });
}

function combinedHalvesFit(contingencyData, emergencyData) {
    const page = createMeasurePage();
    const row = buildCombinedRow(contingencyData, emergencyData);
    page.appendChild(row);
    constrainCombinedColumnsForMeasurement(row);

    const maxHeightPx = COMBINED_HALF_MAX_HEIGHT_MM * MM_TO_PX;
    let fits = true;
    row.querySelectorAll(".print-combined-half").forEach(function (half) {
        if (half.scrollHeight > maxHeightPx) fits = false;
    });

    document.body.removeChild(page);
    return fits;
}

// Brukerønske ("Hvis det er plass kan ERP være på samme side som contingency og emergency. ta nedre
// halvdel av siden f.eks. Pass på at det ikke blir med tomme sider på utskriften") - samme måle-prinsipp
// som combinedHalvesFit over, men på HELE den foreslåtte tredelte siden (rad + ERP stablet under) under
// ETT, siden det her er snakk om total sidehøyde, ikke to uavhengige halvdeler side ved side.
// PRINT_ALL_MAX_HEIGHT_MM er satt lavere enn selve .print-page sin egen 267mm-budsjett (se der) - samme
// "noe margin siden font-rendering kan variere"-begrunnelse som COMBINED_HALF_MAX_HEIGHT_MM over, men
// mindre margin enn den (57mm) trenger, siden risikoen her hovedsakelig gjelder selve
// contingency/emergency-raden (samme kolonnebasert-layout-usikkerhet som over) - ERP sin egen
// .print-grid er en vanlig, forutsigbar to-kolonners grid, ikke en auto-balanserende multi-kolonne-layout.
// HEVET fra 245 til 260 (brukerønske: "contingency, emergency og ERP må få plass på samme side... som
// før") - nær .print-page sitt fysiske 267mm-tak (kan ikke settes høyere enn det uansett - da ville
// "fits" bli sant for innhold som umulig kan få plass på ett fysisk ark). Samme begrunnelse som
// COMBINED_HALF_MAX_HEIGHT_MM over: det gamle, lavere taket feilbedømte ekte, faktisk plassbesparende
// innhold som "får ikke plass".
const PRINT_ALL_MAX_HEIGHT_MM = 260;

// BUG (rapportert av brukeren: FX-10-malen - klart den lengste av alle sjekklistene, dermed nærmest denne
// terskelen av samtlige maler - fikk ERP-boksen til å hoppe til neste side i sin helhet selv med
// buildCombinedTriplePage valgt, mens de kortere generiske malene printet helt fint) - samme feilklasse
// som PRINT_FIT_SAFETY_MARGIN_MM lenger opp ble innført for: denne proben er en isolert,
// absolutt-posisjonert div utenfor selve #printView/.print-page sin egen kontekst, og har vist seg å
// UNDERVURDERE den virkelige renderte høyden med noen millimeter nær grensen - "fungerer for de andre
// malene" er nettopp det man forventer av en terskel som bommer med noen få mm: kun den lengste, mest
// grensetilfelle-utsatte malen rammes. Samme asymmetriske begrunnelse som før gjelder: å vurdere
// kombinasjonen som "får IKKE plass" litt for tidlig er ufarlig (ERP faller da bare tilbake til sin egen,
// allerede fungerende, separate side, se buildAllPrintPages) - å vurdere feil andre veien er nettopp
// buggen som er rapportert.
// Senket fra 8 til 4 - denne målingen skjer nå i en strukturelt reell .print-page (se createMeasurePage),
// ikke lenger en flat, isolert probe-div - mesteparten av avviket margenen opprinnelig kompenserte for
// (se historikk-kommentaren over) kommer nettopp fra den strukturelle mismatchen, som nå er fjernet ved
// roten. Rapportert MOTSATT vei nå ("ERP kom nå sammen med overskriften sin, men på en helt egen side,
// selv om det er en halv side ledig plass på forrige side") - en for høy margin her gjorde denne
// funksjonen unødvendig pessimistisk og kastet bort nettopp den ledige halve siden brukeren så. Beholder
// en liten rest-margin (4mm) for genuin font-rendering-usikkerhet, ikke som plaster på en strukturell
// målefeil lenger.
const PRINT_ALL_SAFETY_MARGIN_MM = 4;

function combinedAllFit(contingencyData, emergencyData, erpData) {
    const page = createMeasurePage();
    const wrapper = document.createElement("div");
    const combinedRow = buildCombinedRow(contingencyData, emergencyData);
    wrapper.appendChild(combinedRow);
    const erpBody = buildErpBody(erpData);
    if (erpBody) {
        erpBody.classList.add("print-combined-bottom");
        wrapper.appendChild(erpBody);
    }
    page.appendChild(wrapper);
    // Se constrainCombinedColumnsForMeasurement-kommentaren ved combinedHalvesFit - samme
    // column-fill:auto-uten-høydekontekst-problem gjelder her.
    constrainCombinedColumnsForMeasurement(combinedRow);

    const fits = wrapper.scrollHeight <= (PRINT_ALL_MAX_HEIGHT_MM - PRINT_ALL_SAFETY_MARGIN_MM) * MM_TO_PX;

    document.body.removeChild(page);
    return fits;
}

// Selve ERP-innholdet (tittelbanner + sjekklistedeler til venstre/Kontaktliste til høyre) - egen
// funksjon (tidligere inlinet i buildErpPage) slik at det samme innholdet kan gjenbrukes BÅDE på ERP sin
// egen, frittstående side (buildErpPage under) OG stablet nederst på en delt side med contingency/
// emergency (buildCombinedTriplePage) - uten å bygge to steder som kan drifte fra hverandre. data-theme
// er satt på selve wrapper-elementet her (ikke lenger på en ytre .print-page), samme mønster som
// buildCombinedHalf allerede bruker for contingency/emergency sine halvdeler - CSS-selektorene
// ([data-theme="erp"] .print-header) treffer uansett hvor dypt/grunt attributtet sitter i treet, se
// buildPrintHeader. Returnerer null hvis fanen er helt tom (ingen sjekkpunkter og ingen kontakter) - ERP
// skal da ikke være med i utskriften i det hele tatt, verken som egen side eller stablet.
function buildErpBody(data) {
    const leftCol = document.createElement("div");
    leftCol.className = "print-col";
    const rightCol = document.createElement("div");
    rightCol.className = "print-col";

    if (data.limits) {
        const contactBox = buildLimitsBox(data.limits, "Kontaktliste", true);
        if (contactBox) rightCol.appendChild(contactBox);
    }
    // layoutNormalColumns (ikke et rent .forEach(...leftCol.appendChild) som før) - ERP-siden bruker
    // nøyaktig samme faste venstre/høyre-oppdeling (sjekklistedeler / Kontaktliste) som Normal-siden (se
    // buildNormalPage), og Kontaktliste er ofte langt kortere enn sjekklistedelene til venstre - samme
    // "det er ledig plass i høyre kolonne, men en sjekklistedel hopper likevel til en helt ny side"-bug
    // som ble rapportert og fikset for Normal-siden (se kommentaren ved funksjonen). ERP kalte tidligere
    // rett .forEach her og manglet dermed helt denne overflow-til-høyre-kolonne-håndteringen - selv om
    // funksjonsnavnet fortsatt sier "Normal", er selve implementasjonen allerede generisk (tar inn
    // leftCol/rightCol/sectionBoxes som parametre, ingen Normal-spesifikk logikk inni), og ERP-siden har
    // identisk sidebredde/kolonnebredde/toppmarg (samme print-header-half + print-grid-oppsett), så den
    // gjenbrukes direkte i stedet for å bygge en egen, parallell kopi som kan drifte fra denne.
    layoutNormalColumns(leftCol, rightCol, buildSectionBoxesForPrint("erp", data));

    if (!leftCol.children.length && !rightCol.children.length) return null;

    const wrapper = document.createElement("div");
    wrapper.setAttribute("data-theme", "erp");
    // print-header-half (se kommentaren i buildNormalPage over, og css/style.css) - samme brette-hensyn
    // som normal-siden, siden ERP-siden bruker akkurat samme faste venstre/høyre-grid (checklist/
    // kontaktliste) under.
    const header = buildPrintHeader("erp", data);
    header.classList.add("print-header-half");
    wrapper.appendChild(header);

    const grid = document.createElement("div");
    grid.className = "print-grid";
    grid.appendChild(leftCol);
    grid.appendChild(rightCol);

    const body = document.createElement("div");
    body.className = "print-page-body";
    body.appendChild(grid);
    wrapper.appendChild(body);
    return wrapper;
}

// ERP som egen, frittstående A4-side - fallback når den IKKE fikk plass sammen med contingency/emergency
// (se combinedAllFit/buildAllPrintPages). Returnerer null hvis fanen er helt tom, se buildErpBody.
function buildErpPage(data) {
    const erpBody = buildErpBody(data);
    if (!erpBody) return null;

    const page = document.createElement("div");
    page.className = "print-page";
    page.appendChild(erpBody);
    return page;
}

function buildAllPrintPages() {
    const state = getState();
    const container = document.getElementById("printView");
    container.innerHTML = "";
    const normalPage = buildNormalPage(state.normal);
    if (normalPage) container.appendChild(normalPage);

    // Autorisasjonsnummeret ligger kun på normal-fanens data (se getState), men gjelder utskriften som
    // helhet, akkurat som datostempelet - se stempel-kommentaren nederst i denne funksjonen.
    const approvalNumber = state.normal && state.normal.approvalNumber ? state.normal.approvalNumber : "";

    // Contingency/emergency deler én side hvis det er plass - hvis ikke (for mye innhold til å få plass
    // side ved side), får de hver sin fulle side i stedet. Er BEGGE helt tomme, hoppes den delte siden
    // over helt (samme "ingen blanke sider"-prinsipp som ERP-siden under) i stedet for å vise en side med
    // to "tom"-meldinger.
    const hasContingency = buildSectionBoxesForPrint("contingency", state.contingency).length > 0;
    const hasEmergency = buildSectionBoxesForPrint("emergency", state.emergency).length > 0;
    const hasErp = !!buildErpBody(state.erp);

    // BUG (rapportert av brukeren, med skjermbilde: "en helt tom side etter normale sjekklisten og før
    // contingency") - dato-/autorisasjonsstempelet ble TIDLIGERE bygget og satt inn PER side, posisjonert
    // absolutt i forhold til den ENKELTE .print-page sin egen (nå, etter float-fiksen over, marginalt
    // upresise) totalhøyde - en side som ble bare noen millimeter høyere enn én fysisk arkhøyde dro dermed
    // stempelet, alene, med seg over på en ellers blank ekstra side. Stemplene viser uansett IDENTISK
    // innhold på hver side (samme dato, samme autorisasjonsnummer for HELE utskriften, ikke per fane) - de
    // trengte dermed aldri å bygges på nytt per side i utgangspunktet. Bygges nå kun ÉN gang og settes inn
    // direkte på selve #printView (ikke på en spesifikk .print-page), med position:fixed (se
    // .print-page-date/.print-page-approval i css/style.css) - en fixed-posisjonert utskriftselement
    // gjentas automatisk av nettleseren i nøyaktig samme hjørne på HVER fysiske utskriftsside, uavhengig av
    // hvor høy hver enkelt side sitt eget innhold blir. Løser dermed bugen strukturelt (stempelet kan ikke
    // lenger "overflowe" til en egen side) i stedet for nok en gjettet mm-justering.
    container.appendChild(buildPageDateStamp());
    const approvalStamp = buildApprovalStamp(approvalNumber);
    if (approvalStamp) container.appendChild(approvalStamp);

    // Brukerønske ("Hvis det er plass kan ERP være på samme side som contingency og emergency. ta nedre
    // halvdel av siden f.eks. Pass på at det ikke blir med tomme sider på utskriften") - forsøkes FØRST,
    // før den vanlige contingency/emergency-sidelogikken under: får alle tre plass stablet på ÉN side
    // (se combinedAllFit), er den siden komplett i seg selv - hverken en egen contingency/emergency-side
    // eller en egen ERP-side skal da også lages (return under). Er det ikke plass til alle tre (eller ERP
    // er tom), faller det helt tilbake til den opprinnelige logikken - contingency/emergency på egen delt
    // side (eller separate sider) OG en egen ERP-side, akkurat som før denne endringen.
    if ((hasContingency || hasEmergency) && hasErp && combinedAllFit(state.contingency, state.emergency, state.erp)) {
        container.appendChild(buildCombinedTriplePage(state.contingency, state.emergency, state.erp));
        return;
    }

    if (hasContingency || hasEmergency) {
        if (combinedHalvesFit(state.contingency, state.emergency)) {
            container.appendChild(buildCombinedPage(state.contingency, state.emergency));
        } else {
            const contingencyPage = buildSingleTabPage("contingency", state.contingency);
            if (contingencyPage) container.appendChild(contingencyPage);
            const emergencyPage = buildSingleTabPage("emergency", state.emergency);
            if (emergencyPage) container.appendChild(emergencyPage);
        }
    }

    const erpPage = buildErpPage(state.erp);
    if (erpPage) container.appendChild(erpPage);
}

/* ---------- Faner ---------- */

function switchTab(tabKey) {
    document.querySelectorAll(".checklist-tab-btn").forEach(function (btn) {
        btn.classList.toggle("active", btn.getAttribute("data-tab-btn") === tabKey);
    });
    document.querySelectorAll(".tab-panel").forEach(function (panel) {
        panel.classList.toggle("active", panel.getAttribute("data-tab") === tabKey);
    });
    // Sjekkpunkt-tekstfeltene i faner som IKKE var aktive ved sideinnlasting kunne ikke måles riktig av
    // autoGrowTextarea da de ble bygget - display:none-elementer har ingen layout, så scrollHeight var 0,
    // og feltene fikk feil (for lav) høyde permanent. Mål dem på nytt nå som fanen faktisk vises.
    const panel = document.querySelector('.tab-panel[data-tab="' + tabKey + '"]');
    if (panel) panel.querySelectorAll("textarea").forEach(autoGrowTextarea);
}

document.addEventListener("DOMContentLoaded", function () {
    const hadSavedContent = loadState();
    if (!hadSavedContent) loadDefaults();

    document.querySelectorAll(".checklist-tab-btn").forEach(function (btn) {
        btn.addEventListener("click", function () { switchTab(btn.getAttribute("data-tab-btn")); });
    });

    document.querySelectorAll(".checklist-title-input, .normal-meta-input").forEach(function (el) {
        el.addEventListener("input", saveState);
    });

    // Påkrevde felt (drone/autorisasjonsnummer): rydd opp den lyserøde markeringen med en gang feltet
    // fylles ut, men marker det først når feltet mister fokus uten innhold - ikke mens man skriver.
    const droneInput = document.getElementById("normal-drone");
    const droneNote = document.getElementById("normal-drone-note");
    const approvalInput = document.getElementById("normal-approval-number");
    const approvalGroup = document.querySelector(".approval-number-group");
    const approvalNote = document.getElementById("normal-approval-number-note");
    if (droneInput) {
        droneInput.addEventListener("input", function () { checkNormalField(droneInput, null, droneNote, false); });
        droneInput.addEventListener("blur", function () { checkNormalField(droneInput, null, droneNote, true); });
    }
    if (approvalInput) {
        approvalInput.addEventListener("input", function () { checkNormalField(approvalInput, approvalGroup, approvalNote, false); });
        approvalInput.addEventListener("blur", function () { checkNormalField(approvalInput, approvalGroup, approvalNote, true); });
    }

    const templateSelect = document.getElementById("normal-template-select");
    if (templateSelect) {
        templateSelect.addEventListener("change", function () {
            const chosen = templateSelect.value;
            const panel = document.getElementById("tabPanel-normal");
            const current = (panel && panel.getAttribute("data-template")) || "mak";

            // De fire offisielle malene (se TEMPLATE_CONFIG) er ment å faktisk gi deg tilbake det
            // opprinnelige innholdet (også det du evt. har slettet), ikke bare filtrere det som måtte stå
            // der fra før - derfor bekreftelse først. Egendefinert trenger ingen bekreftelse - det er
            // alltid trygt (viser bare alt, sletter ingenting).
            const config = TEMPLATE_CONFIG[chosen];
            if (config) {
                if (confirm("Bytte til " + config.label + "-malen? Dette gjenoppretter standard utstyrsliste, begrensninger og sjekklistedeler på normal-fanen, og overskriver eventuelle egne endringer der.")) {
                    reloadNormalFromDefaults(chosen);
                } else {
                    templateSelect.value = current;
                }
            } else {
                setNormalTemplate(chosen);
                saveState();
            }
        });
    }

    document.querySelectorAll("[data-add-equipment]").forEach(function (btn) {
        btn.addEventListener("click", function () {
            addEquipment(btn.getAttribute("data-add-equipment"), "");
            saveState();
            markNormalCustom();
        });
    });

    document.querySelectorAll("[data-add-limit]").forEach(function (btn) {
        btn.addEventListener("click", function () {
            const tabKey = btn.getAttribute("data-add-limit");
            addLimit(tabKey, "", "");
            saveState();
            if (tabKey === "normal") markNormalCustom();
        });
    });

    document.querySelectorAll("[data-sections]").forEach(initSectionDragAndDrop);

    document.querySelectorAll("[data-add-section]").forEach(function (btn) {
        btn.addEventListener("click", function () {
            const tabKey = btn.getAttribute("data-add-section");
            addSection(tabKey, { title: "", items: [] });
            saveState();
            if (tabKey === "normal") markNormalCustom();
        });
    });

    // Nettlesernes "Skriv ut / Lagre som PDF"-dialog foreslår et filnavn basert på document.title, IKKE
    // noe denne appen kan sette direkte selv (window.print() har ingen egen filnavn-parameter) -
    // brukerønske: "PDF som lagres. samme filnavn som json. bare med pdf etternavnet selvfølgelig".
    // Løsningen er derfor å MIDLERTIDIG endre selve sidetittelen til samme mønster som JSON-filnavnet
    // (se buildExportFilenameBase) rett før window.print() kalles, og sette den tilbake til den
    // opprinnelige ("Sjekkliste-bygger - FFI UAS", fra <title> i HTML-en) etterpå - fanget ÉN gang her
    // (ikke lest på nytt hver gang) siden document.title uansett aldri endres av noe ANNET i appen.
    const originalDocumentTitle = document.title;
    function downloadPdf() {
        if (!requireNormalFieldsBeforeExport()) return;
        buildAllPrintPages();
        document.title = buildExportFilenameBase(getState());
        document.body.classList.add("printing-checklist");
        window.print();
    }
    document.getElementById("downloadPdfBtn").addEventListener("click", downloadPdf);
    const downloadPdfBtnTop = document.getElementById("downloadPdfBtnTop");
    if (downloadPdfBtnTop) downloadPdfBtnTop.addEventListener("click", downloadPdf);

    window.addEventListener("afterprint", function () {
        document.body.classList.remove("printing-checklist");
        // Tilbakestiller sidetittelen (se downloadPdf over) - "afterprint" fyres uansett om brukeren
        // faktisk lagret en PDF eller bare avbrøt dialogen, så fanebladet viser aldri det midlertidige
        // filnavnet lenger enn selve utskriftsdialogen er åpen.
        document.title = originalDocumentTitle;
    });

    function downloadJsonNow() {
        if (!requireNormalFieldsBeforeExport()) return;
        const state = getState();
        const data = Object.assign({ skjema: "Sjekkliste-generator" }, state);
        downloadJson(buildExportFilenameBase(state) + ".json", data);
    }
    document.getElementById("downloadJsonBtn").addEventListener("click", downloadJsonNow);
    // Samme "Last ned som JSON"-knapp også oppe ved "Last ned som PDF" (brukerønske: "Ha en last ned
    // json knapp oppe ved last ned pdf knappen også") - egen knapp, samme handler, i stedet for å flytte
    // den nederste (som fortsatt trengs der den er, ved siden av "Last inn JSON"/"Nullstill").
    const downloadJsonBtnTop = document.getElementById("downloadJsonBtnTop");
    if (downloadJsonBtnTop) downloadJsonBtnTop.addEventListener("click", downloadJsonNow);

    // Selve innlesingen av en JSON-fil - delt av BÅDE den skjulte fil-inputen (vanlig filvelger) OG
    // dra-og-slipp-lytterne under (se dropJsonOverlay), i stedet for å bygge samme parse/valider/
    // bekreft-logikk to steder som kan drifte fra hverandre. "Last opp JSON" byttet nylig navn til
    // "Last inn JSON" (brukerønske - "skal ikke være last opp json, men last inn json") - selve filen
    // kommer jo FRA brukerens PC og INN i skjemaet, ikke opp til noen ekstern tjeneste, så "last inn"
    // beskriver bedre hva som faktisk skjer enn "last opp" (som klinger som en nettverksopplasting).
    function handleJsonFile(file) {
        if (!file) return;
        const reader = new FileReader();
        reader.onload = function () {
            let data;
            try {
                data = JSON.parse(reader.result);
            } catch (err) {
                alert("Kunne ikke lese filen - dette ser ikke ut som en gyldig JSON-fil.");
                return;
            }
            const hasKnownTab = data && typeof data === "object" && TAB_KEYS.some(function (k) { return data[k]; });
            if (!hasKnownTab) {
                alert("Fant ikke gjenkjennelig sjekklisteinnhold i filen. Bruk en JSON-fil lastet ned med \"Last ned som JSON\" herfra.");
                return;
            }
            if (confirm("Laste inn sjekklisten fra denne filen? Alt innhold i alle faner blir erstattet med innholdet i filen.")) {
                importStateFromJson(data);
            }
        };
        reader.onerror = function () {
            alert("Kunne ikke lese filen.");
        };
        reader.readAsText(file);
    }

    // "Last inn JSON" - knappen åpner en skjult fil-input (ingen synlig, stygg standard-filvelger på
    // selve knappen); selve innlesingen skjer via handleJsonFile over.
    document.getElementById("uploadJsonBtn").addEventListener("click", function () {
        document.getElementById("uploadJsonInput").click();
    });
    document.getElementById("uploadJsonInput").addEventListener("change", function (e) {
        const input = e.target;
        const file = input.files && input.files[0];
        handleJsonFile(file);
        input.value = ""; // samme fil kan velges på nytt senere uten at "change" uteblir
    });

    // Dra-og-slipp av en JSON-fil hvor som helst i vinduet (brukerønske: "må være mulig å dra json fil
    // fra PC over i vinduet for å laste inn") - i stedet for å måtte omveien om "Last inn JSON"-knappen +
    // den skjulte filvelgeren. dragCounter (ikke bare ett enkelt boolsk flagg) fordi dragenter/dragleave
    // fyres for HVERT element musen krysser inn i/ut av mens man drar over siden (barn-elementer inni
    // .container teller også) - uten telleren ville en dragleave på et INDRE element (mens man fortsatt
    // er over vinduet som helhet) slukket overlayen for tidlig.
    const dropJsonOverlay = document.getElementById("dropJsonOverlay");
    let dragCounter = 0;
    function dragHasFiles(e) {
        return !!(e.dataTransfer && Array.from(e.dataTransfer.types || []).indexOf("Files") !== -1);
    }
    window.addEventListener("dragenter", function (e) {
        if (!dragHasFiles(e)) return;
        e.preventDefault();
        dragCounter++;
        if (dropJsonOverlay) dropJsonOverlay.classList.add("active");
    });
    window.addEventListener("dragover", function (e) {
        if (!dragHasFiles(e)) return;
        // Uten preventDefault her tolker nettleseren dette som "ikke et gyldig droppmål", og selve
        // "drop"-hendelsen ville aldri fyrt (i stedet ville filen bare åpnet seg i en ny fane).
        e.preventDefault();
    });
    window.addEventListener("dragleave", function (e) {
        if (!dragHasFiles(e)) return;
        dragCounter = Math.max(0, dragCounter - 1);
        if (dragCounter === 0 && dropJsonOverlay) dropJsonOverlay.classList.remove("active");
    });
    window.addEventListener("drop", function (e) {
        if (!dragHasFiles(e)) return;
        e.preventDefault();
        dragCounter = 0;
        if (dropJsonOverlay) dropJsonOverlay.classList.remove("active");
        const file = e.dataTransfer.files && e.dataTransfer.files[0];
        if (!file) return;
        if (!/\.json$/i.test(file.name) && file.type !== "application/json") {
            alert("Dra inn en JSON-fil (.json) - " + file.name + " ser ikke ut til å være det.");
            return;
        }
        handleJsonFile(file);
    });

    document.getElementById("resetFormBtn").addEventListener("click", function () {
        const activePanel = document.querySelector(".tab-panel.active");
        const tabKey = activePanel ? activePanel.getAttribute("data-tab") : "normal";
        const tabLabel = (TAB_META[tabKey] && TAB_META[tabKey].label) || tabKey;
        if (confirm("Er du sikker på at du vil nullstille " + tabLabel + "-fanen? Alt innhold der blir slettet og erstattet med standardmalen. De andre fanene påvirkes ikke.")) {
            resetTab(tabKey);
        }
    });
});
