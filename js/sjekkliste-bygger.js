/* js/sjekkliste-bygger.js */

const STORAGE_KEY = "ffi-uas:sjekkliste-generator";
// Bumpes hver gang DEFAULTS-innholdet endres vesentlig. Lagret tilstand fra en eldre versjon blir da
// IKKE lastet inn (se loadState) - i stedet lastes den ferske standardmalen, slik at gammelt
// mellomlagret innhold i nettleseren (fra før en oppdatering av sjekkpunktene) ikke stille overskygger
// nytt innhold. Egne redigeringer forsvinner riktignok samtidig, men det er en villet avveining så lenge
// malen fortsatt er under aktiv utvikling.
const SCHEMA_VERSION = 2;

// Kolonneoverskrifter i sjekkpunkt-tabellen er ulike for normal- vs. contingency/emergency/erp-
// sjekklister: en normal preflight-sjekk sammenligner mot en forventet status, mens de andre beskriver
// hvilket tiltak som skal iverksettes for en gitt situasjon. Ingen av fanene bruker sjekkboks lenger -
// ingen av disse punktene er noe som fysisk skal tikkes av på papiret.
const ITEM_LABELS = {
    normal: { text: "Sjekkpunkt", target: "Status / grense", checkbox: false, showHeader: false },
    contingency: { text: "Situasjon", target: "Tiltak", checkbox: false, showHeader: true },
    emergency: { text: "Situasjon", target: "Tiltak", checkbox: false, showHeader: true },
    erp: { text: "Situasjon", target: "Tiltak", checkbox: false, showHeader: true }
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
            { key: "RTH-batterinivå", value: "20 %" },
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
                title: "Tap av fjernkontroll-lenke", items: [
                    { text: "Lenke tapt i 3 sekunder", target: "UA klatrer til failsafe-høyde, fortsetter til Home" },
                    { text: "Lenke fortsatt tapt", target: "Følg innstilt failsafe-modus (RTH / landing / hover)" }
                ]
            },
            {
                title: "Tap av GNSS", items: [
                    { text: "GNSS mistet under flyging", target: "Gå til manuell/attitude-modus, fly til åpent område og land" }
                ]
            },
            {
                title: "Lavt batterinivå", items: [
                    { text: "Batterivarsel utløst", target: "Avslutt operasjon og initier retur" },
                    { text: "Ingen handling etter 10 sekunder", target: "RTH utløses automatisk" }
                ]
            },
            {
                title: "Annet luftfartøy / trafikk i området", items: [
                    { text: "Bemannet luftfartøy observert", target: "Vurder å lande eller fly unna" }
                ]
            },
            {
                title: "Kraftig endring i vær/vind", items: [
                    { text: "Vind/vær nær eller over grenseverdi", target: "Avbryt operasjon og land" }
                ]
            },
            {
                title: "Personer i bakkeområdet", items: [
                    { text: "Ikke-involvert personell kommer inn i kontrollert bakkeområde", target: "Fly unna / vurder å avbryte og lande" }
                ]
            }
        ]
    },
    // Emergency: kun det som må håndteres UMIDDELBART mens man er i lufta, for å hindre en ulykke -
    // holdes bevisst enkelt (killswitch, velg område uten personer). Det som skjer ETTER en ulykke
    // (krasj, personskade, rapportering) hører hjemme på ERP-fanen, ikke her.
    emergency: {
        title: "Emergency-sjekkliste",
        sections: [
            {
                title: "Fly-away / mister kontroll", items: [
                    { text: "Ingen respons fra fjernkontroll", target: "Killswitch/motor av, helst over område uten personer" }
                ]
            },
            {
                title: "Motorfeil i lufta", items: [
                    { text: "Motor stopper eller feiler under flyging", target: "Killswitch/kontrollert nedstyrting, velg område uten personer" }
                ]
            },
            {
                title: "Batteribrann i lufta", items: [
                    { text: "Batteri ryker/brenner under flyging", target: "Killswitch/nødlanding snarest, velg område uten personer" }
                ]
            }
        ]
    },
    // ERP (Emergency Response Plan): det som gjøres ETTER en ulykke - sikring, førstehjelp, varsling og
    // rapportering på bakken. Motstykket til emergency, som kun dekker det umiddelbare i lufta.
    erp: {
        title: "Emergency Response Plan (ERP)",
        limits: [
            { key: "Brann", value: "110" },
            { key: "Ambulanse", value: "113" },
            { key: "Politi", value: "112" },
            { key: "Politi (ikke nød)", value: "02800" },
            { key: "Operativ leder", value: "" },
            { key: "Ansvarlig leder", value: "" }
        ],
        sections: [
            {
                title: "Kollisjon / krasj", items: [
                    { text: "Luftfartøy krasjet", target: "Sikre området, sjekk for skadde, varsle ansvarlig" }
                ]
            },
            {
                title: "Personskade", items: [
                    { text: "Person skadet av luftfartøy", target: "Iverksett førstehjelp, varsle nødetater (113)" }
                ]
            },
            {
                title: "Brann i batteri (på bakken)", items: [
                    { text: "Batteri ryker/brenner etter landing/krasj", target: "Flytt til brannsikkert sted om mulig, varsle brannvesen (110)" }
                ]
            },
            {
                title: "Nødlanding utenfor godkjent område", items: [
                    { text: "Nødlanding gjennomført utenfor område", target: "Sikre luftfartøyet, varsle grunneier/ansvarlig, rapporter hendelse" }
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

// Samme rad-type brukes til to ting: Operasjonsbegrensninger (normal) og Kontaktliste (erp) - derfor
// parametriserte placeholder-tekster i stedet for hardkodet "Maks vind"-eksempel begge steder.
function createLimitRow(key, value, keyPlaceholder, valuePlaceholder, isNormal) {
    const tr = document.createElement("tr");
    const onEdit = isNormal ? function () { saveState(); markNormalCustom(); } : saveState;

    const tdKey = document.createElement("td");
    tdKey.className = "limit-key";
    const keyInput = document.createElement("input");
    keyInput.type = "text";
    keyInput.className = "limit-key-input";
    keyInput.placeholder = keyPlaceholder || "Parameter (f.eks. Maks vind)";
    keyInput.value = key || "";
    keyInput.addEventListener("input", onEdit);
    tdKey.appendChild(keyInput);

    const tdValue = document.createElement("td");
    const valueInput = document.createElement("input");
    valueInput.type = "text";
    valueInput.className = "limit-value-input";
    valueInput.placeholder = valuePlaceholder || "Verdi (f.eks. 20 knop)";
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

function createItemRow(item, showCheckbox, isNormal) {
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

    // Textarea (ikke input) - vokser vertikalt med innholdet i stedet for å klippe/scrolle lang tekst
    // (typisk "Tiltak" på contingency/emergency/erp) inni et enlinjes felt, se autoGrowTextarea.
    const tdText = document.createElement("td");
    const textInput = document.createElement("textarea");
    textInput.rows = 1;
    textInput.className = "item-text";
    textInput.placeholder = "Sjekkpunkt";
    textInput.value = item.text || "";
    textInput.addEventListener("input", function () { autoGrowTextarea(textInput); onEdit(); });
    tdText.appendChild(textInput);

    const tdTarget = document.createElement("td");
    const targetInput = document.createElement("textarea");
    targetInput.rows = 1;
    targetInput.className = "item-target";
    targetInput.placeholder = "Status / tiltak";
    targetInput.value = item.target || "";
    targetInput.addEventListener("input", function () { autoGrowTextarea(targetInput); onEdit(); });
    tdTarget.appendChild(targetInput);

    const tdRemove = document.createElement("td");
    tdRemove.className = "col-remove";
    const removeBtn = document.createElement("button");
    removeBtn.type = "button";
    removeBtn.className = "remove-row-btn no-print";
    removeBtn.title = "Fjern sjekkpunkt";
    removeBtn.innerHTML = '<i class="fa-solid fa-trash"></i>';
    removeBtn.addEventListener("click", function () {
        if (confirmRowRemoval(textInput.value)) {
            tr.remove();
            onEdit();
        }
    });
    tdRemove.appendChild(removeBtn);

    tr.appendChild(tdText);
    tr.appendChild(tdTarget);
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

    const header = document.createElement("div");
    header.className = "section-header";

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

    header.appendChild(titleInput);
    header.appendChild(removeSectionBtn);

    const table = document.createElement("table");
    table.className = "builder-item-table";
    if (labels.showHeader) {
        table.innerHTML =
            '<thead><tr>' +
            (labels.checkbox ? '<th class="col-check"></th>' : '') +
            '<th>' + labels.text + '</th>' +
            '<th>' + labels.target + '</th>' +
            '<th class="col-remove"></th>' +
            '</tr></thead>';
    }
    const tbody = document.createElement("tbody");
    table.appendChild(tbody);

    (section.items || []).forEach(function (item) {
        tbody.appendChild(createItemRow(item, labels.checkbox, isNormal));
    });

    const footer = document.createElement("div");
    footer.className = "no-print";
    footer.style.padding = "12px 20px";
    const addItemBtn = document.createElement("button");
    addItemBtn.type = "button";
    addItemBtn.className = "btn btn-secondary add-row-btn";
    addItemBtn.innerHTML = '<i class="fa-solid fa-plus"></i> Legg til sjekkpunkt';
    addItemBtn.addEventListener("click", function () {
        const tr = createItemRow(null, labels.checkbox, isNormal);
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

/* ---------- Legg til-knapper ---------- */

function addEquipment(listKey, text) {
    const ul = document.querySelector('[data-equipment="' + listKey + '"]');
    if (!ul) return;
    ul.appendChild(createEquipmentRow(text));
}

const LIMIT_PLACEHOLDERS = {
    normal: { key: "Parameter (f.eks. Maks vind)", value: "Verdi (f.eks. 20 knop)" },
    erp: { key: "Navn (f.eks. Operativ leder)", value: "Telefonnummer" }
};

function addLimit(tabKey, key, value) {
    const tbody = document.querySelector('[data-limits="' + tabKey + '"]');
    if (!tbody) return;
    const placeholders = LIMIT_PLACEHOLDERS[tabKey] || {};
    tbody.appendChild(createLimitRow(key, value, placeholders.key, placeholders.value, tabKey === "normal"));
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
    TAB_KEYS.forEach(function (tabKey) {
        const tabState = {
            title: (document.getElementById(tabKey + "-title") || {}).value || ""
        };

        if (tabKey === "normal") {
            tabState.drone = (document.getElementById("normal-drone") || {}).value || "";
            tabState.approvalNumber = (document.getElementById("normal-approval-number") || {}).value || "";
            const normalPanel = document.getElementById("tabPanel-normal");
            tabState.template = (normalPanel && normalPanel.getAttribute("data-template")) || "mak";
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
                items: Array.from(box.querySelectorAll("tbody tr")).map(function (tr) {
                    const checkbox = tr.querySelector(".item-checked");
                    return {
                        text: tr.querySelector(".item-text").value,
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

// Setter aktivt malvalg (MÅK/spesifikk/egendefinert) på normal-fanen: data-template-attributtet på
// fane-panelet styrer hvilke sjekkpunkter som vises (se CSS-filtreringen på tr[data-variant] - "egendefinert"
// treffer ingen av filtrene, så da vises alt). Brukes både ved innlasting av lagret tilstand,
// nedtrekksvalg og automatisk når brukeren begynner å redigere innholdet (se markNormalCustom).
function setNormalTemplate(template) {
    const panel = document.getElementById("tabPanel-normal");
    if (panel) panel.setAttribute("data-template", template);
    const select = document.getElementById("normal-template-select");
    if (select && select.value !== template) select.value = template;
}

// Kalles på enhver redigering av selve sjekklisteinnholdet (utstyr, begrensninger, sjekklistedeler/
// -punkter) på normal-fanen - i det øyeblikket brukeren begynner å tilpasse innholdet er det ikke
// lenger den offisielle MÅK- eller spesifikk-malen, så valget hopper automatisk til "Egendefinert".
// Gjør ingenting hvis malen allerede står på egendefinert (unngår unødvendig re-rendering/lagring).
function markNormalCustom() {
    const panel = document.getElementById("tabPanel-normal");
    if (panel && panel.getAttribute("data-template") !== "custom") {
        setNormalTemplate("custom");
    }
}

// Gjenoppretter normal-fanens utstyrsliste, begrensninger og sjekklistedeler fra standardmalen -
// brukes når man aktivt velger MÅK/spesifikk i nedtrekksmenyen, slik at man faktisk får tilbake det
// opprinnelige innholdet (inkl. eventuelle rader man har slettet), ikke bare et filter over det som
// måtte stå der fra før. Drone/autorisasjonsnummer røres ikke - det er identifiserende info, ikke
// sjekklisteinnhold. ALL sjekklistedel-tekst (utstyr, begrensninger, sjekkpunkter) overskrives, så
// kalles kun etter bekreftelse (se templateSelect-lytteren i DOMContentLoaded).
function reloadNormalFromDefaults(template) {
    document.querySelectorAll('[data-equipment="normal"] li').forEach(function (li) { li.remove(); });
    document.querySelectorAll('[data-limits="normal"] tr').forEach(function (tr) { tr.remove(); });
    const sectionsContainer = document.querySelector('[data-sections="normal"]');
    if (sectionsContainer) sectionsContainer.innerHTML = "";

    DEFAULTS.normal.equipment.forEach(function (text) { addEquipment("normal", text); });
    DEFAULTS.normal.limits.forEach(function (limit) { addLimit("normal", limit.key, limit.value); });
    DEFAULTS.normal.sections.forEach(function (section) { addSection("normal", section); });

    setNormalTemplate(template);
    saveState();
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
        if (cell.html !== undefined) td.innerHTML = cell.html;
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
    "Operasjonsbegrensninger": "fa-ruler",
    "Kontaktliste": "fa-phone",
    "Før avreise": "fa-clipboard-check",
    "På operasjonsområdet": "fa-map-location-dot",
    "Før avgang": "fa-plane-departure",
    "Overvåking under flyging": "fa-eye",
    "Etter landing": "fa-flag-checkered",
    "Tap av fjernkontroll-lenke": "fa-wifi",
    "Tap av GNSS": "fa-satellite-dish",
    "Lavt batterinivå": "fa-battery-quarter",
    "Annet luftfartøy / trafikk i området": "fa-plane",
    "Kraftig endring i vær/vind": "fa-cloud-bolt",
    "Personer i bakkeområdet": "fa-person-walking",
    "Fly-away / mister kontroll": "fa-triangle-exclamation",
    "Motorfeil i lufta": "fa-gear",
    "Batteribrann i lufta": "fa-fire",
    "Kollisjon / krasj": "fa-car-burst",
    "Personskade": "fa-user-injured",
    "Brann i batteri (på bakken)": "fa-fire-extinguisher",
    "Nødlanding utenfor godkjent område": "fa-location-dot"
};
const DEFAULT_BOX_ICON = "fa-clipboard-list";

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

function buildLimitsBox(limits, title) {
    const rows = (limits || []).filter(function (l) { return l.key && l.key.trim(); });
    if (!rows.length) return null;

    const box = document.createElement("div");
    box.className = "print-box";
    const header = document.createElement("div");
    header.className = "print-box-header print-box-header-reference";
    setBoxHeaderContent(header, title || "Operasjonsbegrensninger");
    box.appendChild(header);

    const table = document.createElement("table");
    table.className = "print-rows";
    rows.forEach(function (l) {
        table.appendChild(buildPrintRow([
            { className: "print-label print-key", text: l.key },
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
    const items = (section.items || []).filter(function (it) { return it.text && it.text.trim(); });
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
    table.className = "print-rows";
    items.forEach(function (item) {
        const cells = [];
        if (showCheckbox) {
            cells.push({ className: "print-check", html: '<span class="print-checkbox-box' + (item.checked ? " checked" : "") + '"></span>' });
        }
        cells.push({ className: "print-label", text: item.text });
        cells.push({ className: "print-value", text: item.target || "" });
        table.appendChild(buildPrintRow(cells));
    });
    box.appendChild(table);
    return box;
}

// Overskriften er nå kun én linje (tittelen) - den viste tidligere også et eget "Contingency"/
// "Emergency"-merke ved siden av, som gjorde at f.eks. "Contingency-sjekkliste" + "Contingency" sto der
// to ganger. Datoen er flyttet ut til ett enkelt hjørnestempel per side (se buildPageDateStamp), i
// stedet for gjentatt i hver overskrift/halvdel.
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

    // Drone/autorisasjonsnummer inn i selve tittelbanneret (hvit skrift, ikke en egen mørk linje under) -
    // drone venstrestilt, nummeret (uten "Autorisasjonsnr:"-etikett) høyrestilt, tittelen fortsatt
    // sentrert midt i mellom via grid (1fr auto 1fr - midtkolonnen krymper til tittelens egen bredde, så
    // de to like store flankekolonnene holder den visuelt sentrert uansett hvor lange drone/nummer er).
    const drone = tabKey === "normal" && data.drone && data.drone.trim() ? data.drone : "";
    const approval = tabKey === "normal" && data.approvalNumber && data.approvalNumber.trim() ? "FFI-UAS-" + data.approvalNumber : "";
    if (drone || approval) {
        header.classList.add("print-header-with-meta");
        const droneSpan = document.createElement("span");
        droneSpan.className = "print-header-meta print-header-meta-left";
        droneSpan.textContent = drone;
        const approvalSpan = document.createElement("span");
        approvalSpan.className = "print-header-meta print-header-meta-right";
        approvalSpan.textContent = approval;
        header.appendChild(droneSpan);
        header.appendChild(h1);
        header.appendChild(approvalSpan);
    } else {
        header.appendChild(h1);
    }
    return header;
}

// Ett datostempel nede i høyre hjørne av selve arket (.print-page har position:relative) - vises kun
// én gang per side, ikke én gang per halvdel på den delte contingency/emergency-siden.
function buildPageDateStamp() {
    const dateEl = document.createElement("div");
    dateEl.className = "print-page-date";
    dateEl.textContent = new Date().toLocaleDateString("no-NO");
    return dateEl;
}

// Bygger print-boksene (utstyr/begrensninger utelatt - kun sjekklistedelene) for én sjekkliste. For
// normal-fanen filtreres punkter etter aktivt malvalg (MÅK/spesifikk) først - punkter merket for det
// andre malvalget skal ikke dukke opp i utskriften, samme regel som CSS-filtreringen på skjermen.
function buildSectionBoxesForPrint(tabKey, data) {
    const showCheckbox = ITEM_LABELS[tabKey].checkbox;
    const template = data.template || "mak";
    const boxes = [];
    (data.sections || []).forEach(function (section) {
        let items = section.items || [];
        if (tabKey === "normal") {
            items = items.filter(function (it) { return !it.variant || it.variant === template; });
        }
        const sectionBox = buildSectionBox({ title: section.title, items: items }, showCheckbox);
        if (sectionBox) boxes.push(sectionBox);
    });
    return boxes;
}

// Normal-siden: egen, frittstående A4-side med fast tittel "Sjekkliste" (drone/autorisasjonsnummer vises
// nå inni selve tittelbanneret, se buildPrintHeader), og et eksplisitt to-kolonners grid (sjekklistedeler
// fast til venstre, utstyr/begrensninger fast til høyre - se .builder-grid i css/style.css for samme
// oppdeling på skjermen).
function buildNormalPage(data) {
    const page = document.createElement("div");
    page.className = "print-page";
    page.setAttribute("data-theme", "normal");
    page.appendChild(buildPrintHeader("normal", data));

    const grid = document.createElement("div");
    grid.className = "print-grid";

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

    if (!leftCol.children.length && !rightCol.children.length) {
        const empty = document.createElement("p");
        empty.className = "print-empty-note";
        empty.textContent = "Denne sjekklisten er tom.";
        grid.appendChild(empty);
    } else {
        grid.appendChild(leftCol);
        grid.appendChild(rightCol);
    }

    const body = document.createElement("div");
    body.className = "print-page-body";
    body.appendChild(grid);
    page.appendChild(body);
    page.appendChild(buildPageDateStamp());
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

// Contingency og emergency deler én side, hver sin halvdel (med egen farget tittelbanner og sidefelt) -
// i stedet for hver sin egen fulle A4-side. .print-columns-half bruker column-width (ikke et fast
// column-count) slik at halvdelen selv kan falle tilbake til én kolonne i det trange rommet, eller
// bruke flere kolonner hvis det skulle være plass. Kalles kun når combinedHalvesFit sier det er nok
// plass - se buildAllPrintPages.
function buildCombinedPage(contingencyData, emergencyData) {
    const page = document.createElement("div");
    page.className = "print-page print-page-combined";

    const row = document.createElement("div");
    row.className = "print-combined-row";
    row.appendChild(buildCombinedHalf("contingency", contingencyData));
    row.appendChild(buildCombinedHalf("emergency", emergencyData));
    page.appendChild(row);
    page.appendChild(buildPageDateStamp());
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
    page.appendChild(buildPrintHeader(tabKey, data));

    const columns = document.createElement("div");
    columns.className = "print-columns";
    boxes.forEach(function (box) { columns.appendChild(box); });

    const body = document.createElement("div");
    body.className = "print-page-body";
    body.appendChild(columns);

    const sidebar = buildPrintSidebar(tabKey);
    if (sidebar) body.appendChild(sidebar);

    page.appendChild(body);
    page.appendChild(buildPageDateStamp());
    return page;
}

// Måler (i et skjult, men layoutet "probe"-element - display:none-elementer kan ikke måles) om begge
// halvdelene faktisk får plass innenfor det som er igjen av arkhøyden når header/dato er trukket fra.
// mm er en absolutt CSS-lengdeenhet (96px/25.4mm) og regnes likt om på skjerm som i utskrift, så et mål
// tatt på skjermen (usynlig, plassert utenfor synsfeltet) stemmer overens med faktisk trykk. Terskelen
// er satt med noe margin siden font-rendering kan variere marginalt mellom nettlesere/OS.
const MM_TO_PX = 96 / 25.4;
const COMBINED_HALF_MAX_HEIGHT_MM = 210;

function combinedHalvesFit(contingencyData, emergencyData) {
    const probe = document.createElement("div");
    probe.style.cssText = "position:absolute; visibility:hidden; left:-9999px; top:0; width:176mm; display:block;";
    document.body.appendChild(probe);

    const row = document.createElement("div");
    row.className = "print-combined-row";
    row.appendChild(buildCombinedHalf("contingency", contingencyData));
    row.appendChild(buildCombinedHalf("emergency", emergencyData));
    probe.appendChild(row);

    const maxHeightPx = COMBINED_HALF_MAX_HEIGHT_MM * MM_TO_PX;
    let fits = true;
    row.querySelectorAll(".print-combined-half").forEach(function (half) {
        if (half.scrollHeight > maxHeightPx) fits = false;
    });

    document.body.removeChild(probe);
    return fits;
}

// ERP (etter-ulykke-siden): samme oppbygning som normal-siden - sjekklistedeler fast til venstre,
// men med Kontaktliste (telefonnumre) i stedet for utstyr/begrensninger til høyre. Returnerer null hvis
// fanen er helt tom (ingen sjekkpunkter og ingen kontakter) - ERP-siden skal da ikke være med i det
// hele tatt i utskriften, ikke vises som en side med kun en "tom"-melding.
function buildErpPage(data) {
    const leftCol = document.createElement("div");
    leftCol.className = "print-col";
    const rightCol = document.createElement("div");
    rightCol.className = "print-col";

    if (data.limits) {
        const contactBox = buildLimitsBox(data.limits, "Kontaktliste");
        if (contactBox) rightCol.appendChild(contactBox);
    }
    buildSectionBoxesForPrint("erp", data).forEach(function (box) { leftCol.appendChild(box); });

    if (!leftCol.children.length && !rightCol.children.length) return null;

    const page = document.createElement("div");
    page.className = "print-page";
    page.setAttribute("data-theme", "erp");
    page.appendChild(buildPrintHeader("erp", data));

    const grid = document.createElement("div");
    grid.className = "print-grid";
    grid.appendChild(leftCol);
    grid.appendChild(rightCol);

    const body = document.createElement("div");
    body.className = "print-page-body";
    body.appendChild(grid);
    page.appendChild(body);
    page.appendChild(buildPageDateStamp());
    return page;
}

function buildAllPrintPages() {
    const state = getState();
    const container = document.getElementById("printView");
    container.innerHTML = "";
    container.appendChild(buildNormalPage(state.normal));

    // Contingency/emergency deler én side hvis det er plass - hvis ikke (for mye innhold til å få plass
    // side ved side), får de hver sin fulle side i stedet. Se combinedHalvesFit. Er BEGGE helt tomme,
    // hoppes den delte siden over helt (samme "ingen blanke sider"-prinsipp som ERP-siden under) i
    // stedet for å vise en side med to "tom"-meldinger.
    const hasContingency = buildSectionBoxesForPrint("contingency", state.contingency).length > 0;
    const hasEmergency = buildSectionBoxesForPrint("emergency", state.emergency).length > 0;
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

    document.querySelectorAll("[data-add-section]").forEach(function (btn) {
        btn.addEventListener("click", function () {
            const tabKey = btn.getAttribute("data-add-section");
            addSection(tabKey, { title: "", items: [] });
            saveState();
            if (tabKey === "normal") markNormalCustom();
        });
    });

    function downloadPdf() {
        if (!requireNormalFieldsBeforeExport()) return;
        buildAllPrintPages();
        document.body.classList.add("printing-checklist");
        window.print();
    }
    document.getElementById("downloadPdfBtn").addEventListener("click", downloadPdf);
    const downloadPdfBtnTop = document.getElementById("downloadPdfBtnTop");
    if (downloadPdfBtnTop) downloadPdfBtnTop.addEventListener("click", downloadPdf);

    window.addEventListener("afterprint", function () {
        document.body.classList.remove("printing-checklist");
    });

    document.getElementById("downloadJsonBtn").addEventListener("click", function () {
        if (!requireNormalFieldsBeforeExport()) return;
        const state = getState();
        const data = Object.assign({ skjema: "Sjekkliste-generator" }, state);
        const slug = function (s) { return (s || "").trim().replace(/\s+/g, "_"); };
        const titlePart = slug(state.normal && state.normal.drone) || "sjekklister";
        const datePart = new Date().toISOString().split("T")[0];
        downloadJson(titlePart + "-" + datePart + ".json", data);
    });

    document.getElementById("resetFormBtn").addEventListener("click", function () {
        if (confirm("Er du sikker på at du vil nullstille sjekklisten? Alt innhold i alle faner blir slettet og erstattet med standardmalen.")) {
            localStorage.removeItem(STORAGE_KEY);
            location.reload();
        }
    });
});
