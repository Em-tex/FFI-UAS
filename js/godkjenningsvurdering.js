/* js/godkjenningsvurdering.js */

const STORAGE_KEY = "ffi-uas:godkjenningsvurdering";
// Lagret tilstand kobles til sjekkpunktene via ID (se CHECKPOINTS under), ikke via posisjon i listen -
// nye sjekkpunkter kan derfor legges til, og gamle fjernes, uten at en pågående vurdering i nettleseren
// blir ubrukelig: ukjente ID-er i den lagrede tilstanden ignoreres, og nye punkter starter som "Velg...".
// Derfor trengs heller ingen SCHEMA_VERSION-invalidering slik sjekkliste-byggeren har.
const STATE_VERSION = 1;

const FIELD_IDS = ["reviewSubject", "reviewApplicant", "reviewer", "reviewDate"];

// Statusvalgene. "" (Velg...) er standard og vises grått (se .review-status.is-empty i style.css) slik at
// det er lett å se hvilke punkter som ikke er vurdert ennå. "ok" og "na" teller begge som ferdig vurdert
// i helhetsvurderingen (se updateVerdict) - "N/A" betyr at punktet ikke er relevant for denne
// operasjonen, ikke at det gjenstår.
const STATUS_OPTIONS = [
    { value: "", label: "Velg..." },
    { value: "ok", label: "OK" },
    { value: "forbedring", label: "Trenger forbedring" },
    { value: "na", label: "N/A" }
];
const STATUS_LABELS = { ok: "OK", forbedring: "Trenger forbedring", na: "N/A" };

// Sjekkpunktene. Et toppnivåpunkt med "children" blir en gruppe (egen overskriftsrad med underpunkter
// under seg) - kun underpunktene har egen status, gruppen viser en oppsummering av barna sine.
// "guide" er teksten som vises i veiledningsboksen når man klikker på selve navnet (se openGuide).
// Nye punkter legges til her - HTML-en og resten av logikken er datadrevet og trenger ingen endring.
//
// Innholdet følger FFIs egne styrende dokumenter for UAS-aktivitet: operativ leder UAS skal vurdere
// AKTIVITETENS OMFANG, SYSTEMBESKRIVELSEN AV UAS og selve RISIKOVURDERINGEN, og kan pålegge at
// aktiviteten bare gjennomføres dersom ytterligere risikoreduserende tiltak iverksettes. Derfor er
// gruppene lagt opp etter nettopp de tre - de tre siste punktene (pilotliste, sjekkliste,
// kompetansekrav) er de konkrete vedleggene som skal følge søknaden.
const CHECKPOINTS = [
    {
        id: "operasjon",
        title: "Beskrivelse av operasjonen",
        guide: {
            intro: "Aktivitetens omfang. Søknaden skal beskrive aktiviteten godt nok til at du som operativ leder kan danne deg et bilde av hva som faktisk skal gjøres, hvor, når og med hvilket materiell - uten å måtte ringe søkeren.",
            points: [
                "FFIs UAS-aktivitet spenner fra eksperimentell forskning med høy sannsynlighet for havari til operasjoner med modne systemer sammen med eksterne aktører - det må gå fram hvor i dette spennet aktiviteten ligger.",
                "Er beskrivelsen konkret, eller er den så generell at den kan dekke nesten hva som helst?",
                "Henger underpunktene sammen - passer valgt system, område og flymodus til formålet?",
                "Er det noe som avviker fra tidligere godkjenninger for tilsvarende aktivitet?"
            ]
        },
        children: [
            {
                id: "operasjon-formaal",
                title: "Formål og aktivitetstype",
                guide: {
                    intro: "Hva aktiviteten skal oppnå, hvilken type UAS-aktivitet det er, og hvordan den gjennomføres rent praktisk.",
                    points: [
                        "Aktivitetstypen går fram: intern utvikling og eksperimentering i kontrollert og avgrenset område, tilsvarende sammen med Forsvaret eller andre aktører, eller aktivitet på øvelse der teknologien inngår i en større operativ helhet.",
                        "Formålet er beskrevet (forskning, utvikling, validering av teknologi eller operasjonskonsept, opplæring).",
                        "Gjennomføringen er beskrevet steg for steg, med antall flyginger og varighet.",
                        "Bemanning går fram - fjernpilot, eventuell observatør og andre roller.",
                        "Det går fram om aktiviteten faller innenfor kategorien \"Specific\"."
                    ]
                }
            },
            {
                id: "operasjon-drone",
                title: "Drone og systembeskrivelse",
                guide: {
                    intro: "Systembeskrivelsen av UAS - hva slags fartøy det er, hvor modent det er, og hvor godt det er dokumentert.",
                    points: [
                        "Systemet er identifisert (type, eventuelt serienummer / ID), og det går fram om det er egenutviklet, modifisert eller levert av en samarbeidspartner.",
                        "Modenhet: er dette et eksperimentelt design med høy sannsynlighet for havari, eller et modent system?",
                        "Dokumentasjon av design og bevis for flygedyktighet står i forhold til modenheten.",
                        "Vekt, ytelse og eventuell nyttelast (kamera, sensor) er beskrevet.",
                        "Skal materiell fra FFI brukes av en avdeling i Forsvaret, må materiellet ha teknisk og forvaltningsmessig godkjenning (TFG), eller avdelingen må være en godkjent testorganisasjon.",
                        "Termineringssystem og automatisk feilhåndtering er beskrevet der det er relevant."
                    ]
                }
            },
            {
                id: "operasjon-omraade",
                title: "Område og tidsrom",
                guide: {
                    intro: "Hvor og når det skal flys - og om området faktisk er kontrollert og avgrenset.",
                    points: [
                        "Området er avgrenset (kart, koordinater eller navngitt område), ikke bare \"i nærheten av\".",
                        "Det går fram hvordan området holdes kontrollert, og hvem som har adgang.",
                        "Tidsrom er angitt, inkludert om det skal flys i mørke.",
                        "Grunneier / eier av området er avklart der det kreves.",
                        "Maks høyde og avstand er angitt."
                    ]
                }
            },
            {
                id: "operasjon-luftrom",
                title: "Luftrom og dronesoner",
                guide: {
                    intro: "Luftrommet aktiviteten foregår i, og hvem som eventuelt må varsles eller koordineres med.",
                    points: [
                        "Luftromsklasse og eventuelle dronesoner er sjekket (dronesoner.no).",
                        "Behov for koordinering eller klarering med lufttrafikktjenesten er beskrevet.",
                        "Koordinering med Forsvaret eller andre brukere av området er beskrevet der aktiviteten skjer sammen med dem.",
                        "Nærhet til flyplass, landingsplass for helikopter eller annen luftfart er vurdert.",
                        "ATO / HemsWX er nevnt der det er relevant for kategorien."
                    ]
                }
            },
            {
                id: "operasjon-risiko",
                title: "Tredjepart og verdier",
                guide: {
                    intro: "Hvem og hva som kan bli rammet dersom noe går galt. Risikovurderinger ved FFI skal forebygge tap av liv og helse, tap av sensitiv informasjon, tap av verdier og tap av omdømme.",
                    points: [
                        "Hvem som kan befinne seg i og rundt området, og hvordan de holdes utenfor.",
                        "Avstand til folkemengder, bebyggelse og vei er vurdert.",
                        "Sensitiv informasjon: hva systemet samler inn, lagrer og sender, og hvordan det håndteres.",
                        "Omdømme: er aktiviteten synlig for utenforstående, og er det behov for varsling eller informasjon på forhånd?"
                    ]
                }
            }
        ]
    },
    {
        id: "risikovurdering",
        title: "Risikovurdering",
        guide: {
            intro: "\"(O) Mal for risikovurdering og godkjenning av UAS-aktivitet\" skal være fylt ut etter dialog med operativ leder UAS og sendt inn via søkerens nærmeste leder. Overordnet metode og prosess står i \"(T) Rutine for UAS-aktivitet og risikohåndtering\".",
            points: [
                "Riktig mal er brukt, og den er fullstendig utfylt.",
                "Den er sendt via nærmeste leder, ikke direkte fra søkeren.",
                "Innholdet henger sammen med aktivitetsbeskrivelsen - ingen risikoområder som er beskrevet ett sted og glemt det andre."
            ]
        },
        children: [
            {
                id: "risiko-omraader",
                title: "Risikoområder og vurdering",
                guide: {
                    intro: "Hensikten med risikovurderingen er å identifisere de enkelte risikoområdene og vurdere dem ut fra sannsynlighet og konsekvens.",
                    points: [
                        "Risikoområdene dekker liv og helse, sensitiv informasjon, verdier og omdømme.",
                        "Sannsynlighet og konsekvens er faktisk vurdert for hvert område, ikke bare listet opp.",
                        "Havari er vurdert som et realistisk utfall der systemet er eksperimentelt.",
                        "Vurderingene er gjenkjennelige for akkurat denne aktiviteten, ikke gjenbrukt ordrett fra en tidligere søknad."
                    ]
                }
            },
            {
                id: "risiko-tiltak",
                title: "Risikoreduserende tiltak",
                guide: {
                    intro: "ALARP: risikoen skal være så lav som praktisk og økonomisk mulig. Har et tiltak en reell nytte, og kostnaden ikke står i grovt misforhold til nytten, skal tiltaket gjennomføres - et svært kostbart tiltak med minimal risikoreduksjon skal ikke gjennomføres. Du kan som operativ leder pålegge ytterligere tiltak som vilkår for godkjenning.",
                    points: [
                        "Krav til dokumentasjon av design.",
                        "Operasjonsmanual for aktiviteten: begrensninger og ytelser, brukerinstruks (flightmanual), pre- og post-flight sjekklister, normal-, nød- og beredskapsprosedyrer, vedlikeholdsprogram.",
                        "Krav til opplæring / currency.",
                        "Krav til testprogram.",
                        "Krav til airworthiness - bevis for flygedyktighet.",
                        "Krav til termineringssystem og automatisk feilhåndtering.",
                        "Krav til produksjonsgrunnlag og prosedyrer.",
                        "Tiltakene er konkrete og etterprøvbare - hvem gjør hva, og når."
                    ]
                }
            },
            {
                id: "risiko-restrisiko",
                title: "Restrisiko og aksept",
                guide: {
                    intro: "Helhetsvurderingen: er restrisikoen ved aktiviteten akseptabel?",
                    points: [
                        "Søknaden konkluderer selv på restrisikoen - den er ikke bare listet opp uten svar.",
                        "Tap av materiell: leder med resultatansvar (forskningsleder) har ansvaret for hvor stor risiko som aksepteres her, og aksepten skal komme tydelig fram.",
                        "Restrisikoen for liv og helse er innenfor det som kan aksepteres for denne aktivitetstypen.",
                        "Vilkår du selv setter, skrives ned som krav i godkjenningen - ikke som en muntlig forutsetning."
                    ]
                }
            }
        ]
    },
    {
        id: "pilotliste",
        title: "Pilotliste",
        guide: {
            intro: "Listen over hvem som skal fly på denne godkjenningen skal være lagt ved søknaden.",
            points: [
                "Pilotliste er faktisk vedlagt.",
                "Alle piloter er navngitt - ingen åpne formuleringer som \"og andre\".",
                "Hver pilot har gyldig kompetansebevis og er kjent for FFI UAS.",
                "Rollen til hver enkelt går fram (fjernpilot, observatør, instruktør).",
                "Currency: pilotene har fløyet nok på systemet i det siste til å gjennomføre aktiviteten trygt."
            ]
        }
    },
    {
        id: "sjekkliste",
        title: "Sjekkliste",
        guide: {
            intro: "Sjekklisten som følger søknaden skal passe til akkurat denne aktiviteten, ikke bare være en generisk mal. Sjekklistene er en del av operasjonsmanualen for aktiviteten, sammen med begrensninger, ytelser og brukerinstruks.",
            points: [
                "Normal-sjekklisten dekker det som faktisk er spesielt for denne aktiviteten - pre- og post-flight.",
                "Contingency og emergency dekker de realistiske feilsituasjonene for akkurat dette systemet.",
                "ERP med kontaktliste er på plass og oppdatert.",
                "Begrensninger (vind, høyde, avstand, batteri) er angitt og realistiske.",
                "Sjekklisten henger sammen med de risikoreduserende tiltakene i risikovurderingen."
            ]
        }
    },
    {
        id: "kompetansekrav",
        title: "Kompetansekrav",
        guide: {
            intro: "Kompetansekravene i søknaden skal stå i forhold til dronetypen og flymodusene som skal brukes - krav til opplæring og currency er selv et risikoreduserende tiltak.",
            points: [
                "Kravene er tilpasset dronetypen (multirotor, fixed-wing, VTOL) og systemets modenhet.",
                "Flymodusene som skal brukes er dekket av kravene - både manuelle moduser og autonome oppdrag.",
                "Krav til opplæring, trening og vedlikeholdstrening (currency) er angitt.",
                "Krav til eventuelt testprogram er dekket der aktiviteten er utvikling eller testing.",
                "Kravene er verken strengere eller løsere enn for tilsvarende godkjenninger."
            ]
        }
    }
];

// Alle sjekkpunkter som faktisk har en status - gruppene selv teller ikke med, de oppsummerer bare barna.
function leafCheckpoints() {
    const leaves = [];
    CHECKPOINTS.forEach(function (cp) {
        if (cp.children) {
            cp.children.forEach(function (child) { leaves.push(child); });
        } else {
            leaves.push(cp);
        }
    });
    return leaves;
}

function findCheckpoint(id) {
    let found = null;
    CHECKPOINTS.forEach(function (cp) {
        if (cp.id === id) found = cp;
        (cp.children || []).forEach(function (child) {
            if (child.id === id) found = child;
        });
    });
    return found;
}

/* ---------- Bygging av radene ---------- */

function createRow(cp, isSub) {
    const row = document.createElement("div");
    row.className = "review-row" + (isSub ? " review-row-sub" : "");
    row.setAttribute("data-id", cp.id);

    const main = document.createElement("div");
    main.className = "review-row-main";

    // Selve navnet er en knapp, ikke ren tekst - hele poenget er at det skal være klikkbart for
    // veiledning (se openGuide). Spørsmålstegn-ikonet er der bare for å gjøre synlig AT det er
    // klikkbart, det er ingen egen knapp.
    const titleBtn = document.createElement("button");
    titleBtn.type = "button";
    titleBtn.className = "review-title-btn no-print";
    titleBtn.innerHTML = '<span></span> <i class="fa-regular fa-circle-question"></i>';
    titleBtn.querySelector("span").textContent = cp.title;
    titleBtn.addEventListener("click", function () { openGuide(cp.id); });
    main.appendChild(titleBtn);

    // Knappen over er no-print (en utskrift skal ikke vise en knapp) - navnet må derfor finnes som ren
    // tekst også, samme swap-mønster som resten av nettstedet bruker (se .print-value i style.css).
    const titlePrint = document.createElement("span");
    titlePrint.className = "print-value review-title-print";
    titlePrint.textContent = cp.title;
    main.appendChild(titlePrint);

    const controls = document.createElement("div");
    controls.className = "review-row-controls";

    const select = document.createElement("select");
    select.className = "review-status no-print is-empty";
    STATUS_OPTIONS.forEach(function (opt) {
        const option = document.createElement("option");
        option.value = opt.value;
        option.textContent = opt.label;
        select.appendChild(option);
    });
    select.addEventListener("change", function () {
        applyStatusStyling(row);
        updateVerdict();
        saveState();
    });
    controls.appendChild(select);

    const statusPrint = document.createElement("span");
    statusPrint.className = "print-value review-status-print";
    controls.appendChild(statusPrint);

    const commentBtn = document.createElement("button");
    commentBtn.type = "button";
    commentBtn.className = "review-comment-btn no-print";
    commentBtn.title = "Legg til kommentar";
    commentBtn.innerHTML = '<i class="fa-regular fa-comment"></i>';
    controls.appendChild(commentBtn);

    main.appendChild(controls);
    row.appendChild(main);

    const commentWrap = document.createElement("div");
    commentWrap.className = "review-comment-wrap";
    const comment = document.createElement("textarea");
    comment.className = "review-comment no-print";
    comment.rows = 2;
    comment.placeholder = "Kommentar til dette punktet";
    comment.addEventListener("input", function () {
        autoGrow(comment);
        updateCommentBtn(row);
        saveState();
    });
    commentWrap.appendChild(comment);
    const commentPrint = document.createElement("span");
    commentPrint.className = "print-value review-comment-print";
    commentWrap.appendChild(commentPrint);
    row.appendChild(commentWrap);

    commentBtn.addEventListener("click", function () {
        const willOpen = !commentWrap.classList.contains("open");
        commentWrap.classList.toggle("open", willOpen);
        if (willOpen) {
            autoGrow(comment);
            comment.focus();
        }
        updateCommentBtn(row);
    });

    return row;
}

function autoGrow(textarea) {
    textarea.style.height = "auto";
    textarea.style.height = textarea.scrollHeight + "px";
}

// Snakkeboble-ikonet: en utfylt kommentar markeres (fylt ikon, FFI-blå) slik at en kommentar som ligger
// i en sammenslått boks ikke blir usynlig for den som blar gjennom skjemaet.
function updateCommentBtn(row) {
    const btn = row.querySelector(".review-comment-btn");
    const hasText = !!row.querySelector(".review-comment").value.trim();
    const isOpen = row.querySelector(".review-comment-wrap").classList.contains("open");
    btn.classList.toggle("has-comment", hasText);
    btn.classList.toggle("is-open", isOpen);
    btn.querySelector("i").className = hasText ? "fa-solid fa-comment-dots" : "fa-regular fa-comment";
    btn.title = hasText ? "Vis / skjul kommentar" : "Legg til kommentar";
}

function applyStatusStyling(row) {
    const select = row.querySelector(".review-status");
    // is-empty gir "Velg..." grå skrift (se style.css) - uten den ser et uvurdert punkt like "ferdig"
    // ut som et vurdert et. data-status fargelegger selve raden.
    select.classList.toggle("is-empty", select.value === "");
    row.setAttribute("data-status", select.value);
}

function renderCheckpoints() {
    const list = document.getElementById("reviewList");
    list.innerHTML = "";
    CHECKPOINTS.forEach(function (cp) {
        if (cp.children) {
            const head = document.createElement("div");
            head.className = "review-group-head";
            head.setAttribute("data-group", cp.id);

            const headBtn = document.createElement("button");
            headBtn.type = "button";
            headBtn.className = "review-group-btn no-print";
            headBtn.innerHTML = '<span></span> <i class="fa-regular fa-circle-question"></i>';
            headBtn.querySelector("span").textContent = cp.title;
            headBtn.addEventListener("click", function () { openGuide(cp.id); });
            head.appendChild(headBtn);

            const headPrint = document.createElement("span");
            headPrint.className = "print-value review-group-print";
            headPrint.textContent = cp.title;
            head.appendChild(headPrint);

            const pill = document.createElement("span");
            pill.className = "review-group-pill no-print";
            head.appendChild(pill);

            list.appendChild(head);
            cp.children.forEach(function (child) { list.appendChild(createRow(child, true)); });
        } else {
            list.appendChild(createRow(cp, false));
        }
    });
    document.querySelectorAll(".review-row").forEach(applyStatusStyling);
}

/* ---------- Helhetsvurdering ---------- */

function statusOf(id) {
    const row = document.querySelector('.review-row[data-id="' + id + '"]');
    return row ? row.querySelector(".review-status").value : "";
}

// Helhetsvurderingen øverst. Tre tilstander:
//   - noen punkter trenger forbedring  -> kan ikke godkjennes ennå
//   - alt er OK eller N/A              -> "Kan godkjennes"
//   - ellers                           -> ikke ferdig vurdert, med antall punkter som gjenstår
// "Trenger forbedring" vinner med vilje over "ikke ferdig vurdert": så snart ett punkt er underkjent er
// konklusjonen den samme uansett hvor mange av de andre punktene operativ leder har rukket å gå gjennom.
function updateVerdict() {
    const leaves = leafCheckpoints();
    const statuses = leaves.map(function (cp) { return statusOf(cp.id); });
    const needsWork = statuses.filter(function (s) { return s === "forbedring"; }).length;
    const remaining = statuses.filter(function (s) { return s === ""; }).length;

    const box = document.getElementById("verdictBox");
    const icon = document.getElementById("verdictIcon");
    const title = document.getElementById("verdictTitle");
    const sub = document.getElementById("verdictSub");

    box.classList.remove("state-open", "state-blocked", "state-ok");

    if (needsWork > 0) {
        box.classList.add("state-blocked");
        icon.className = "fa-solid fa-circle-exclamation";
        title.textContent = "Kan ikke godkjennes ennå";
        sub.textContent = needsWork === 1
            ? "1 sjekkpunkt trenger forbedring."
            : needsWork + " sjekkpunkter trenger forbedring.";
    } else if (remaining === 0) {
        box.classList.add("state-ok");
        icon.className = "fa-solid fa-circle-check";
        title.textContent = "Kan godkjennes";
        sub.textContent = "Alle sjekkpunkter er OK eller ikke relevante (N/A).";
    } else {
        box.classList.add("state-open");
        icon.className = "fa-solid fa-hourglass-half";
        title.textContent = "Ikke ferdig vurdert";
        sub.textContent = remaining === leaves.length
            ? "Ingen sjekkpunkter er vurdert ennå."
            : remaining + " av " + leaves.length + " sjekkpunkter gjenstår.";
    }

    updateGroupPills();
}

function updateGroupPills() {
    CHECKPOINTS.forEach(function (cp) {
        if (!cp.children) return;
        const head = document.querySelector('.review-group-head[data-group="' + cp.id + '"]');
        if (!head) return;
        const pill = head.querySelector(".review-group-pill");
        const statuses = cp.children.map(function (child) { return statusOf(child.id); });
        const needsWork = statuses.filter(function (s) { return s === "forbedring"; }).length;
        const remaining = statuses.filter(function (s) { return s === ""; }).length;
        pill.classList.remove("pill-open", "pill-blocked", "pill-ok");
        if (needsWork > 0) {
            pill.classList.add("pill-blocked");
            pill.textContent = needsWork + " trenger forbedring";
        } else if (remaining === 0) {
            pill.classList.add("pill-ok");
            pill.textContent = "Alle punkter vurdert";
        } else {
            pill.classList.add("pill-open");
            pill.textContent = remaining + " av " + cp.children.length + " gjenstår";
        }
    });
}

/* ---------- Veiledningsboks ---------- */

function openGuide(id) {
    const cp = findCheckpoint(id);
    if (!cp || !cp.guide) return;
    document.getElementById("guideTitle").textContent = cp.title;
    document.getElementById("guideIntro").textContent = cp.guide.intro || "";
    const ul = document.getElementById("guidePoints");
    ul.innerHTML = "";
    (cp.guide.points || []).forEach(function (point) {
        const li = document.createElement("li");
        li.textContent = point;
        ul.appendChild(li);
    });
    document.getElementById("guideOverlay").classList.add("open");
}

function closeGuide() {
    document.getElementById("guideOverlay").classList.remove("open");
}

/* ---------- Lagring, eksport og import ---------- */

function getState() {
    const state = { skjema: "Vurdering av godkjenninger", __version: STATE_VERSION, felt: {}, punkter: {} };
    FIELD_IDS.forEach(function (id) {
        const el = document.getElementById(id);
        if (el) state.felt[id] = el.value;
    });
    document.querySelectorAll(".review-row").forEach(function (row) {
        state.punkter[row.getAttribute("data-id")] = {
            status: row.querySelector(".review-status").value,
            kommentar: row.querySelector(".review-comment").value
        };
    });
    return state;
}

function saveState() {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(getState()));
}

function applyState(state) {
    if (!state || typeof state !== "object") return;
    Object.entries(state.felt || {}).forEach(function (entry) {
        const el = document.getElementById(entry[0]);
        if (el) el.value = entry[1];
    });
    document.querySelectorAll(".review-row").forEach(function (row) {
        // Ukjente ID-er i den lagrede/innlastede filen ignoreres, og punkter som mangler der nullstilles
        // (se kommentaren ved STATE_VERSION) - en import skal erstatte HELE vurderingen, ikke smelte to
        // vurderinger sammen til noe ingen av dem var.
        const saved = (state.punkter || {})[row.getAttribute("data-id")] || {};
        const select = row.querySelector(".review-status");
        const comment = row.querySelector(".review-comment");
        select.value = STATUS_LABELS[saved.status] ? saved.status : "";
        comment.value = saved.kommentar || "";
        row.querySelector(".review-comment-wrap").classList.toggle("open", !!comment.value.trim());
        autoGrow(comment);
        applyStatusStyling(row);
        updateCommentBtn(row);
    });
    updateVerdict();
}

function loadState() {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return;
    let state;
    try {
        state = JSON.parse(raw);
    } catch (e) {
        return;
    }
    applyState(state);
}

function sanitizeForFilename(s) {
    // Samme regel som i leksjonsskjema.js: fjerner tegn som er ugyldige i filnavn på Windows/macOS/Linux.
    return (s || "").replace(/[\\/:*?"<>|]/g, "").trim();
}

function formatDateForFilename(isoDate) {
    if (!isoDate) return "";
    const parts = isoDate.split("-"); // <input type="date"> gir alltid ÅÅÅÅ-MM-DD
    return parts.length === 3 ? parts[2] + "-" + parts[1] + "-" + parts[0] : isoDate;
}

// Delt av JSON-nedlastingen og PDF-utskriften - nettleseren bruker document.title som forslag til filnavn
// i "Skriv ut / Lagre som PDF"-dialogen, så PDF-en får samme navn som JSON-filen (se beforeprint under).
function buildStandardFilename() {
    const subject = sanitizeForFilename(document.getElementById("reviewSubject").value) || "Uten navn";
    const applicant = sanitizeForFilename(document.getElementById("reviewApplicant").value) || "Ukjent søker";
    const date = formatDateForFilename(document.getElementById("reviewDate").value) || "udatert";
    return "Vurdering av godkjenning - " + subject + " - " + applicant + " - " + date;
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

/* ---------- Utskrift ----------
   Samme "bytt skjemaelementene ut med ren tekst"-mønster som leksjonsskjema.html (se .print-value i
   css/style.css) - uten det ville utskriften vist selve nedtrekksmenyene og tekstboksene med kantlinjer
   og nedtrekkspil i stedet for lesbar tekst. */

function syncPrintFields() {
    FIELD_IDS.forEach(function (id) {
        const el = document.getElementById(id);
        const target = document.querySelector('[data-print-for="' + id + '"]');
        if (!el || !target) return;
        target.textContent = (id === "reviewDate" ? formatDateForFilename(el.value) : el.value) || " ";
    });
    document.querySelectorAll(".review-row").forEach(function (row) {
        const status = row.querySelector(".review-status").value;
        row.querySelector(".review-status-print").textContent = STATUS_LABELS[status] || "Ikke vurdert";
        const comment = row.querySelector(".review-comment").value.trim();
        row.querySelector(".review-comment-print").textContent = comment;
        // Kommentarbokser uten innhold skal ikke legge igjen en tom linje i utskriften. Klassen leses kun
        // av @media print-reglene (se .review-comment-wrap.print-empty i style.css), så den kan trygt bli
        // liggende igjen på skjermen etterpå.
        row.querySelector(".review-comment-wrap").classList.toggle("print-empty", !comment);
    });
}

document.addEventListener("DOMContentLoaded", function () {
    renderCheckpoints();
    loadState();
    updateVerdict();

    const dateInput = document.getElementById("reviewDate");
    if (!dateInput.value) {
        dateInput.value = new Date().toISOString().split("T")[0];
    }

    FIELD_IDS.forEach(function (id) {
        const el = document.getElementById(id);
        if (!el) return;
        el.addEventListener("input", saveState);
        el.addEventListener("change", saveState);
    });

    // Veiledningsboksen lukkes med krysset, med klikk utenfor selve kortet og med Escape - alle tre er
    // forventet oppførsel for en slik boks, og et lite kryss er lett å bomme på med berøringsskjerm.
    document.getElementById("guideCloseBtn").addEventListener("click", closeGuide);
    document.getElementById("guideOverlay").addEventListener("click", function (e) {
        if (e.target === this) closeGuide();
    });
    document.addEventListener("keydown", function (e) {
        if (e.key === "Escape") closeGuide();
    });

    const ORIGINAL_TITLE = document.title;
    window.addEventListener("beforeprint", function () {
        syncPrintFields();
        document.title = buildStandardFilename();
    });
    window.addEventListener("afterprint", function () {
        // "afterprint" fyres uansett om brukeren faktisk lagret en PDF eller bare lukket dialogen, så
        // fanen viser aldri det midlertidige filnavnet lenger enn utskriftsdialogen står åpen.
        document.title = ORIGINAL_TITLE;
    });

    document.getElementById("downloadPdfBtn").addEventListener("click", function () {
        window.print();
    });

    document.getElementById("downloadJsonBtn").addEventListener("click", function () {
        downloadJson(buildStandardFilename() + ".json", getState());
    });

    // Selve innlesingen av en JSON-fil - delt av den skjulte filvelgeren og dra-og-slipp-lytterne under,
    // i stedet for to sett med parse/valider/bekreft-logikk som kan drifte fra hverandre.
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
            if (!data || typeof data !== "object" || !data.punkter) {
                alert("Fant ikke gjenkjennelig innhold i filen. Bruk en JSON-fil lastet ned med \"Last ned som JSON\" herfra.");
                return;
            }
            if (confirm("Laste inn vurderingen fra denne filen? Alt utfylt innhold i skjemaet blir erstattet.")) {
                applyState(data);
                saveState();
            }
        };
        reader.onerror = function () {
            alert("Kunne ikke lese filen.");
        };
        reader.readAsText(file);
    }

    document.getElementById("uploadJsonBtn").addEventListener("click", function () {
        document.getElementById("uploadJsonInput").click();
    });
    document.getElementById("uploadJsonInput").addEventListener("change", function (e) {
        const input = e.target;
        handleJsonFile(input.files && input.files[0]);
        input.value = ""; // samme fil kan velges på nytt senere uten at "change" uteblir
    });

    // Dra-og-slipp av en JSON-fil hvor som helst i vinduet - samme mekanikk som sjekkliste-byggeren
    // (dragCounter, ikke et enkelt boolsk flagg, fordi dragenter/dragleave fyres for HVERT element musen
    // krysser inn i og ut av mens man drar over siden).
    const dropJsonOverlay = document.getElementById("dropJsonOverlay");
    let dragCounter = 0;
    function dragHasFiles(e) {
        return !!(e.dataTransfer && Array.from(e.dataTransfer.types || []).indexOf("Files") !== -1);
    }
    window.addEventListener("dragenter", function (e) {
        if (!dragHasFiles(e)) return;
        e.preventDefault();
        dragCounter++;
        dropJsonOverlay.classList.add("active");
    });
    window.addEventListener("dragover", function (e) {
        if (!dragHasFiles(e)) return;
        // Uten preventDefault her regnes vinduet som et ugyldig droppmål, og filen ville bare åpnet seg i
        // en ny fane i stedet for å utløse "drop" under.
        e.preventDefault();
    });
    window.addEventListener("dragleave", function (e) {
        if (!dragHasFiles(e)) return;
        dragCounter = Math.max(0, dragCounter - 1);
        if (dragCounter === 0) dropJsonOverlay.classList.remove("active");
    });
    window.addEventListener("drop", function (e) {
        if (!dragHasFiles(e)) return;
        e.preventDefault();
        dragCounter = 0;
        dropJsonOverlay.classList.remove("active");
        const file = e.dataTransfer.files && e.dataTransfer.files[0];
        if (!file) return;
        if (!/\.json$/i.test(file.name) && file.type !== "application/json") {
            alert("Dra inn en JSON-fil (.json) - " + file.name + " ser ikke ut til å være det.");
            return;
        }
        handleJsonFile(file);
    });

    document.getElementById("resetFormBtn").addEventListener("click", function () {
        if (confirm("Er du sikker på at du vil nullstille skjemaet? Alt utfylt innhold blir slettet.")) {
            localStorage.removeItem(STORAGE_KEY);
            location.reload();
        }
    });
});
