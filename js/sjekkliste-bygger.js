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
const SCHEMA_VERSION = 23;

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

const DEFAULTS = {
    normal: {
        drone: "",
        approvalNumber: "",
        template: "mak",
        equipment: ["Drone", "Fjernkontroll", "Batterier"],
        limits: [
            { key: "Maks vind", value: "10 m/s" },
            { key: "Maks høyde", value: "120 m" },
            { key: "Maks hastighet", value: "" },
            { key: "Maks rekkevidde", value: "VLOS" },
            { key: "Vær", value: "Ingen nedbør" },
            // "RTH-batterinivå" fjernet - brukerønske ("kan variere med operasjonen").
            { key: "Oppvisning for publikum", value: "Forbudt" }
        ],
        sections: [
            {
                title: "Før avreise", items: [
                    { text: "FFI-godkjenning", target: "Gyldig" },
                    { text: "MÅK underkategori", target: "Definert", variant: "mak" },
                    { text: "ATO skjema", target: "Meldt inn", variant: "spesifikk" },
                    { text: "Vær", target: "Innenfor begrensninger" },
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
                title: "Lavt batterinivå varsel", items: [
                    { text: "Auto RTH", target: "Avbryt" },
                    { text: "Drone", target: "Fly hjem og land" }
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
            { key: "Operativ leder", value: "" },
            { key: "UAS SITS", value: "520 7563 / 692 37 563 (sivilt innvalg)" },
            { key: "UAS Autorisasjonstelefon", value: "458 72 017" }
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
                // "Fly-away / mistet kontakt" - samme singleColumn-behandling som "Ulykke" over, se
                // kommentaren der.
                title: "Fly-away / mistet kontakt", singleColumn: true, items: [
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
    // createItemRow/autoGrowTextarea - en uvanlig lang parametertekst ("Oppvisning for publikum") får
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
    item = item || { text: "", target: "", checked: false, variant: "" };
    const tr = document.createElement("tr");
    const onEdit = isNormal ? function () { saveState(); markNormalCustom(); } : saveState;
    // Malvalget (MÅK/spesifikk) et sjekkpunkt hører til er satt av standardmalen, ikke redigerbart per
    // rad - kun nedtrekksmenyen øverst på fanen styrer hvilke rader som vises, se setNormalTemplate og
    // CSS-filtreringen på tr[data-variant] i style.css.
    tr.dataset.variant = item.variant || "";

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
    TAB_KEYS.forEach(function (tabKey) {
        const tabState = {
            title: (document.getElementById(tabKey + "-title") || {}).value || "",
            category: activeCategory
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
                        variant: tr.dataset.variant || ""
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

// Setter selve nedtrekksmenyens tilstand (MÅK/spesifikk/egendefinert) - data-template-attributtet på
// tabPanel-normal alene, KUN normal-fanens egen "har jeg avveket fra standardmalen"-status (brukt av
// f.eks. templateSelect-lytteren for å vite hva den skal tilbakestille TIL hvis brukeren avbryter en
// bekreftelsesdialog). IKKE lenger det samme som filtreringskategorien (se setActiveCategory over) -
// oppdaterer kategorien KUN når malvalget faktisk ER en av de to offisielle malene (mak/spesifikk), ALDRI
// når det går til "custom" (se markNormalCustom) - en redigering av normal-fanens EGET innhold skal ikke
// lenger kunne slå av MÅK/spesifikk-filtreringen på "Tap av C2 link" som et utilsiktet sideeffekt.
function setNormalTemplate(template) {
    const panel = document.getElementById("tabPanel-normal");
    if (panel) panel.setAttribute("data-template", template);
    if (template === "mak" || template === "spesifikk") setActiveCategory(template);
    const select = document.getElementById("normal-template-select");
    if (select && select.value !== template) select.value = template;
}

// Kalles på enhver redigering av selve sjekklisteinnholdet (utstyr, begrensninger, sjekklistedeler/
// -punkter) på normal-fanen - i det øyeblikket brukeren begynner å tilpasse innholdet er det ikke
// lenger den offisielle MÅK- eller spesifikk-malen, så valget hopper automatisk til "Egendefinert".
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

// Gjenoppretter normal-fanens utstyrsliste, begrensninger og sjekklistedeler fra standardmalen -
// brukes når man aktivt velger MÅK/spesifikk i nedtrekksmenyen, slik at man faktisk får tilbake det
// opprinnelige innholdet (inkl. eventuelle rader man har slettet), ikke bare et filter over det som
// måtte stå der fra før. Drone/autorisasjonsnummer røres ikke - det er identifiserende info, ikke
// sjekklisteinnhold. ALL sjekklistedel-tekst (utstyr, begrensninger, sjekkpunkter) overskrives, så
// kalles kun etter bekreftelse (se templateSelect-lytteren i DOMContentLoaded).
function reloadNormalFromDefaults(template) {
    clearTabContent("normal");

    DEFAULTS.normal.equipment.forEach(function (text) { addEquipment("normal", text); });
    DEFAULTS.normal.limits.forEach(function (limit) { addLimit("normal", limit.key, limit.value); });
    DEFAULTS.normal.sections.forEach(function (section) { addSection("normal", section); });

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
        // Gjenoppretter filtreringskategorien EKSPLISITT her, uansett hva data.template måtte være (også
        // "custom" - der ville setNormalTemplate sin egen guard (se der) IKKE rørt kategorien i det hele
        // tatt) - uten denne ville en lagret "custom"-tilstand (fra FØR denne fiksen, se BUG-kommentaren
        // ved buildSectionBoxesForPrint) latt kategorien stå på HTML-ens hardkodede startverdi ("mak") i
        // stedet for brukerens faktisk sist valgte MÅK/spesifikk.
        setActiveCategory(data.category || "mak");
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

const TAB_META = {
    normal: { label: "Normal", sidebarLabel: null },
    contingency: { label: "Contingency", sidebarLabel: "BEREDSKAP" },
    emergency: { label: "Emergency", sidebarLabel: "NØD" },
    erp: { label: "ERP", sidebarLabel: "ERP" }
};

function buildPrintRow(cells, extraClass) {
    const tr = document.createElement("tr");
    cells.forEach(function (cell) {
        const td = document.createElement("td");
        td.className = cell.className || "";
        if (cell.iconClass) {
            // Nødnummer-ikon (se ERP_EMERGENCY_CONTACTS/buildLimitsBox) - egen <i>-node + tekst-node i
            // stedet for cell.html under, slik at selve nummer-/kontaktnavnet fortsatt settes trygt via
            // textContent (ingen HTML-injeksjon av det brukeren har skrevet inn i feltet).
            const icon = document.createElement("i");
            icon.className = "fa-solid " + cell.iconClass;
            td.appendChild(icon);
            td.appendChild(document.createTextNode(" " + (cell.text || "")));
        } else if (cell.html !== undefined) td.innerHTML = cell.html;
        else td.textContent = cell.text || "";
        tr.appendChild(td);
    });
    if (extraClass) tr.className = extraClass;
    return tr;
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
    "Overvåking under flyging": "fa-eye",
    "Etter landing": "fa-flag-checkered",
    "Tap av C2 link": "fa-wifi",
    "Tap av GNSS": "fa-satellite-dish",
    "Lavt batterinivå varsel": "fa-battery-quarter",
    "Fly-away / kontrolltap": "fa-triangle-exclamation",
    "Ulykke": "fa-car-burst",
    "Fly-away / mistet kontakt": "fa-magnifying-glass-location"
};
const DEFAULT_BOX_ICON = "fa-clipboard-list";

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
// kan være fri tekst brukeren har skrevet selv.
function setBoxHeaderContent(header, title) {
    const icon = document.createElement("i");
    icon.className = "fa-solid " + (BOX_ICONS[title] || DEFAULT_BOX_ICON);
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

    const table = document.createElement("table");
    table.className = "print-rows";
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

    const table = document.createElement("table");
    table.className = "print-rows";
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
function buildSectionBox(section, showCheckbox) {
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
    setBoxHeaderContent(header, section.title && section.title.trim() ? section.title : "Uten navn");
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

    const table = document.createElement("table");
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
    table.className = "print-rows print-rows-situasjon";
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
        cells.push({ className: "print-number" + (singleColumn ? " print-number-top" : ""), text: (index + 1) + "." });
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
    const boxes = [];
    (data.sections || []).forEach(function (section) {
        // Variant-filtreringen gjaldt FØR kun normal-fanen (den eneste som hadde mak/spesifikk-merkede
        // punkter). Nå har contingency-fanen også slike (se "Tap av C2 link" i DEFAULTS.contingency), så
        // filteret kjøres nå uansett tabKey - harmløst for faner uten variant-merkede punkter (it.variant
        // er da alltid tom, og filteret slipper gjennom alt akkurat som før).
        const items = (section.items || []).filter(function (it) {
            if (!it.variant) return true;
            if (template !== "mak" && template !== "spesifikk") return true;
            return it.variant === template;
        });
        const sectionBox = buildSectionBox({ title: section.title, items: items, singleColumn: section.singleColumn }, showCheckbox);
        if (sectionBox) boxes.push(sectionBox);
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
    buildSectionBoxesForPrint("normal", data).forEach(function (box) { leftCol.appendChild(box); });

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
const COMBINED_HALF_MAX_HEIGHT_MM = 210;
// Full sidebredde tilgjengelig for INNHOLD (A4 210mm - @page-margen 15mm x 2 - .print-page sin egen
// padding 2mm x 2, se css/style.css) - samme breddemål probene under (og selve den ferdige siden) faktisk
// får, slik at en probe-måling stemmer overens med virkeligheten.
const PRINT_CONTENT_WIDTH_MM = 176;

function combinedHalvesFit(contingencyData, emergencyData) {
    const probe = document.createElement("div");
    probe.style.cssText = "position:absolute; visibility:hidden; left:-9999px; top:0; width:" + PRINT_CONTENT_WIDTH_MM + "mm; display:block;";
    document.body.appendChild(probe);

    const row = buildCombinedRow(contingencyData, emergencyData);
    probe.appendChild(row);

    const maxHeightPx = COMBINED_HALF_MAX_HEIGHT_MM * MM_TO_PX;
    let fits = true;
    row.querySelectorAll(".print-combined-half").forEach(function (half) {
        if (half.scrollHeight > maxHeightPx) fits = false;
    });

    document.body.removeChild(probe);
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
const PRINT_ALL_MAX_HEIGHT_MM = 245;

function combinedAllFit(contingencyData, emergencyData, erpData) {
    const probe = document.createElement("div");
    probe.style.cssText = "position:absolute; visibility:hidden; left:-9999px; top:0; width:" + PRINT_CONTENT_WIDTH_MM + "mm; display:block;";
    document.body.appendChild(probe);

    const wrapper = document.createElement("div");
    wrapper.appendChild(buildCombinedRow(contingencyData, emergencyData));
    const erpBody = buildErpBody(erpData);
    if (erpBody) {
        erpBody.classList.add("print-combined-bottom");
        wrapper.appendChild(erpBody);
    }
    probe.appendChild(wrapper);

    const fits = wrapper.scrollHeight <= PRINT_ALL_MAX_HEIGHT_MM * MM_TO_PX;

    document.body.removeChild(probe);
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
    buildSectionBoxesForPrint("erp", data).forEach(function (box) { leftCol.appendChild(box); });

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

            // MÅK/spesifikk er ment å være de offisielle malene - å velge en av dem skal faktisk gi deg
            // tilbake det opprinnelige innholdet (også det du evt. har slettet), ikke bare filtrere det
            // som måtte stå der fra før. Egendefinert trenger ingen bekreftelse - det er alltid trygt
            // (viser bare alt, sletter ingenting).
            if (chosen === "mak" || chosen === "spesifikk") {
                const label = chosen === "mak" ? "MÅK" : "Spesifikk";
                if (confirm("Bytte til " + label + "-malen? Dette gjenoppretter standard utstyrsliste, begrensninger og sjekklistedeler på normal-fanen, og overskriver eventuelle egne endringer der.")) {
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

    document.getElementById("downloadJsonBtn").addEventListener("click", function () {
        if (!requireNormalFieldsBeforeExport()) return;
        const state = getState();
        const data = Object.assign({ skjema: "Sjekkliste-generator" }, state);
        downloadJson(buildExportFilenameBase(state) + ".json", data);
    });

    // "Last opp JSON" - knappen åpner en skjult fil-input (ingen synlig, stygg standard-filvelger på
    // selve knappen); selve innlesingen skjer i input-ets change-lytter under.
    document.getElementById("uploadJsonBtn").addEventListener("click", function () {
        document.getElementById("uploadJsonInput").click();
    });
    document.getElementById("uploadJsonInput").addEventListener("change", function (e) {
        const input = e.target;
        const file = input.files && input.files[0];
        if (!file) return;

        const reader = new FileReader();
        reader.onload = function () {
            let data;
            try {
                data = JSON.parse(reader.result);
            } catch (err) {
                alert("Kunne ikke lese filen - dette ser ikke ut som en gyldig JSON-fil.");
                input.value = "";
                return;
            }
            const hasKnownTab = data && typeof data === "object" && TAB_KEYS.some(function (k) { return data[k]; });
            if (!hasKnownTab) {
                alert("Fant ikke gjenkjennelig sjekklisteinnhold i filen. Bruk en JSON-fil lastet ned med \"Last ned som JSON\" herfra.");
                input.value = "";
                return;
            }
            if (confirm("Laste inn sjekklisten fra denne filen? Alt innhold i alle faner blir erstattet med innholdet i filen.")) {
                importStateFromJson(data);
            }
            input.value = "";
        };
        reader.onerror = function () {
            alert("Kunne ikke lese filen.");
            input.value = "";
        };
        reader.readAsText(file);
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
