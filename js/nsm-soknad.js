/* js/nsm-soknad.js */

const STORAGE_KEY = "ffi-uas:nsm-soknad";
const STATE_VERSION = 1;

// Alle enkeltfelt på siden, i samme rekkefølge som i skjemaet. Hvert felt har en søsken-"print-value"
// i HTML-en (data-print-for="<id>") som fylles rett før utskrift - se syncPrintFields. UAS-dataene og
// de ekstra kontaktradene står IKKE her: de kan det være flere av, og bygges av JS (se addUasBlock og
// createContactRow).
const FIELD_IDS = [
    "nsmOrgName", "nsmOrgNumber", "nsmAddress", "nsmOperatorNumbers",
    "nsmContactName", "nsmContactRole", "nsmContactPhone", "nsmContactEmail",
    "nsmArea", "nsmStartDate", "nsmEndDate",
    "nsmPurpose",
    "nsmMapDescription",
    "nsmAreaOwnerDetails",
    "nsmOpsContactRole", "nsmOpsContactName", "nsmOpsContactPhone", "nsmOpsContactEmail"
];
const DATE_FIELD_IDS = ["nsmStartDate", "nsmEndDate"];

// Virksomhetens navn, organisasjonsnummer, postadresse og FFIs UAS-operatørnumre er like i hver eneste
// søknad herfra, og er offentlig kjent informasjon (brukerønske) - de fylles derfor ut på forhånd. Det
// samme gjelder funksjonsteksten på de to faste kontaktradene, som skal kunne endres, men som nesten
// alltid er den samme. Alt ANNET i skjemaet står bevisst tomt, ikke engang som eksempeltekst.
// Standardverdiene legges inn i felt som står TOMME (se applyDefaults) - de overskriver aldri noe
// brukeren selv har skrevet eller lastet inn fra en JSON-fil.
const DEFAULT_FIELDS = {
    nsmOrgName: "Forsvarets forskningsinstitutt - FFI",
    nsmOrgNumber: "970 963 340",
    nsmAddress: "FFI, Postboks 25, 2027 Kjeller",
    nsmOperatorNumbers: "MAA-NOR: NOR.MIL.UASOA.001\nFlydrone.no: NORmyr7obz9mbh4b",
    nsmContactRole: "Kontaktperson for søknaden",
    nsmOpsContactRole: "Kontaktpunkt under UAS-operasjonene"
};

// Standardtekster som er endret i ettertid. Et skjema som allerede ligger mellomlagret i nettleseren
// har den GAMLE teksten i feltet, og siden standardverdier bare fylles inn i tomme felt (se
// applyDefaults) ville den blitt stående til noen nullstilte skjemaet manuelt. Står feltet fortsatt
// nøyaktig til en gammel standardtekst, er den åpenbart ikke skrevet av brukeren selv - da byttes den
// ut. Har brukeren skrevet noe annet, røres den ikke.
const RENAMED_DEFAULTS = {
    nsmOpsContactRole: [
        "Kontaktperson under UAS-operasjoner",
        "Kontaktperson under UAS-operasjonene"
    ]
};

// Feltene i én UAS-blokk. Samme liste brukes til å bygge blokken, lese den av og skrive den ut.
const UAS_FIELDS = [
    { key: "fabrikat", label: "Fabrikat og modell", cls: "uas-make" },
    { key: "serienummer", label: "Serienummer", cls: "uas-serial" },
    { key: "flyvekt", label: "Flyvekt", cls: "uas-weight" },
    // Flerlinjefelt: et UAS har typisk flere sensorer, ført opp under hverandre.
    { key: "sensor", label: "Sensor", cls: "uas-sensor", multiline: true }
];

// Kolonnene i kontakttabellen (de to faste radene ligger i HTML-en og er ikke med her).
const CONTACT_FIELDS = [
    { key: "funksjon", cls: "contact-role", cellClass: "col-role" },
    { key: "navn", cls: "contact-name" },
    { key: "telefon", cls: "contact-phone" },
    { key: "epost", cls: "contact-email" }
];

// Kartutsnittet lagres som en data-URL i localStorage sammen med resten av skjemaet. Store skjermbilder
// (et fullskjerms kartutsnitt kan fort bli flere megabyte) ville sprengt lagringskvoten i nettleseren,
// så bildet skaleres ned til denne bredden og lagres som JPEG før det tas vare på - se readImageFile.
// 1600 px er rikelig for en A4-bredde i utskrift (ca. 190 mm ≈ 1600 px ved 215 dpi).
const MAP_MAX_WIDTH = 1600;
const MAP_JPEG_QUALITY = 0.85;

// Kartutsnittene: { bilde: dataUrl, tegning: { lat, lon, zoom, lag, punkter } | null }. "tegning" er
// visningen bak et kart laget med js/kart-tegner.js, og lagres sammen med bildet slik at tegningen kan
// hentes fram og endres senere. Et opplastet bilde har ingen tegning bak seg.
let mapItems = [];
// Hvilken oppføring filvelgeren skal skrive til: -1 betyr "legg til en ny".
let mapUploadTarget = -1;
// Kontaktpunkt-raden (den faste andreraden i kontakttabellen) kan slettes - da skjules den i stedet
// for å fjernes fra DOM-en, siden feltene der har faste id-er resten av koden regner med finnes.
let opsContactRowVisible = true;
let uasCounter = 0;

/* ---------- UAS-blokker ---------- */

function createUasBlock(data) {
    data = data || {};
    uasCounter++;
    const uid = uasCounter;

    const block = document.createElement("div");
    block.className = "nsm-uas-block";

    const head = document.createElement("div");
    head.className = "nsm-uas-head";
    const title = document.createElement("span");
    title.className = "nsm-uas-title";
    head.appendChild(title);
    const removeBtn = document.createElement("button");
    removeBtn.type = "button";
    removeBtn.className = "remove-row-btn no-print";
    removeBtn.title = "Fjern dette UAS-et";
    removeBtn.innerHTML = '<i class="fa-solid fa-trash"></i>';
    removeBtn.addEventListener("click", function () {
        block.remove();
        ensureOneUasBlock();
        renumberUasBlocks();
        saveState();
    });
    head.appendChild(removeBtn);
    block.appendChild(head);

    const rows = document.createElement("div");
    rows.className = "nsm-rows";
    UAS_FIELDS.forEach(function (field) {
        const row = document.createElement("div");
        row.className = "nsm-row";

        // Egen id per felt per blokk, slik at <label for> fortsatt peker på riktig felt når det
        // finnes flere UAS i skjemaet. uasCounter gjenbrukes aldri, heller ikke etter en sletting.
        const fieldId = "uas-" + uid + "-" + field.key;
        const label = document.createElement("label");
        label.className = "nsm-row-label";
        label.setAttribute("for", fieldId);
        label.textContent = field.label;
        row.appendChild(label);

        const value = document.createElement("div");
        value.className = "nsm-row-value";
        // Også enlinjefeltene er <textarea> med klassen nsm-oneline: da brytes en lang verdi om og
        // feltet vokser i høyden i stedet for å skjule begynnelsen av teksten slik en <input> gjør
        // (se autoGrow og Enter-håndteringen i DOMContentLoaded).
        const input = document.createElement("textarea");
        input.rows = field.multiline ? 2 : 1;
        input.id = fieldId;
        input.className = field.cls + " no-print" + (field.multiline ? "" : " nsm-oneline");
        input.value = data[field.key] || "";
        input.addEventListener("input", function () {
            autoGrow(input);
            saveState();
        });
        value.appendChild(input);
        const print = document.createElement("span");
        print.className = "print-value" + (field.multiline ? " print-value-multiline" : "");
        value.appendChild(print);
        row.appendChild(value);

        rows.appendChild(row);
    });
    block.appendChild(rows);

    return block;
}

function addUasBlock(data) {
    document.getElementById("uasBlocks").appendChild(createUasBlock(data));
    renumberUasBlocks();
}

// Skjemaet skal aldri stå helt uten UAS-blokk - da ville seksjonen sett tom og ødelagt ut.
function ensureOneUasBlock() {
    if (!document.querySelector(".nsm-uas-block")) addUasBlock();
}

function renumberUasBlocks() {
    const blocks = document.querySelectorAll(".nsm-uas-block");
    blocks.forEach(function (block, i) {
        block.querySelector(".nsm-uas-title").textContent = "UAS " + (i + 1);
        // Sletteknappen er skjult når det bare finnes én blokk: den kan uansett ikke fjernes (se
        // ensureOneUasBlock), og en knapp som bare tømmer feltene ville vært misvisende.
        block.querySelector(".remove-row-btn").hidden = blocks.length === 1;
    });
}

function getUasList() {
    const list = [];
    document.querySelectorAll(".nsm-uas-block").forEach(function (block) {
        const uas = {};
        UAS_FIELDS.forEach(function (field) {
            uas[field.key] = block.querySelector("." + field.cls).value;
        });
        list.push(uas);
    });
    return list;
}

/* ---------- Kontaktrader ---------- */

function createContactRow(data) {
    data = data || {};
    const tr = document.createElement("tr");

    CONTACT_FIELDS.forEach(function (field) {
        const td = document.createElement("td");
        if (field.cellClass) td.className = field.cellClass;
        // <textarea> med nsm-oneline, av samme grunn som i UAS-blokkene: en lang e-postadresse skal
        // brytes om og gjøre cellen høyere, ikke forsvinne ut til siden.
        const input = document.createElement("textarea");
        input.rows = 1;
        input.className = field.cls + " no-print nsm-oneline";
        input.value = data[field.key] || "";
        input.addEventListener("input", function () {
            autoGrow(input);
            saveState();
        });
        td.appendChild(input);
        // Samme swap-til-ren-tekst-mønster som resten av nettstedet bruker under utskrift (se
        // .print-value i css/style.css) - en <input> med kantlinje og padding ser ut som en boks på
        // papiret, ikke som en utfylt tabellcelle.
        const print = document.createElement("span");
        print.className = "print-value";
        td.appendChild(print);
        tr.appendChild(td);
    });

    const tdRemove = document.createElement("td");
    tdRemove.className = "col-remove no-print";
    const removeBtn = document.createElement("button");
    removeBtn.type = "button";
    removeBtn.className = "remove-row-btn";
    removeBtn.title = "Fjern raden";
    removeBtn.innerHTML = '<i class="fa-solid fa-trash"></i>';
    removeBtn.addEventListener("click", function () {
        tr.remove();
        saveState();
    });
    tdRemove.appendChild(removeBtn);
    tr.appendChild(tdRemove);

    return tr;
}

function addContactRow(data) {
    document.getElementById("contactRows").appendChild(createContactRow(data));
}

// Skjuler eller viser kontaktpunkt-raden. Skjult rad blir heller ikke med i utskriften ([hidden] i
// css/style.css) - og feltene tømmes ved sletting, slik at en gammel verdi ikke ligger igjen usynlig
// i en nedlastet JSON-fil.
function toggleOpsContactRow() {
    document.getElementById("opsContactRow").hidden = !opsContactRowVisible;
    document.getElementById("addOpsContactBtn").hidden = opsContactRowVisible;
}

function getContacts() {
    const persons = [];
    document.querySelectorAll("#contactRows tr").forEach(function (tr) {
        const person = {};
        CONTACT_FIELDS.forEach(function (field) {
            person[field.key] = tr.querySelector("." + field.cls).value;
        });
        persons.push(person);
    });
    return persons;
}

/* ---------- Områdeeier (ja/nei) ---------- */

function getAreaOwnerAnswer() {
    const checked = document.querySelector('input[name="nsmAreaOwner"]:checked');
    return checked ? checked.value : "";
}

function setAreaOwnerAnswer(value) {
    document.querySelectorAll('input[name="nsmAreaOwner"]').forEach(function (radio) {
        radio.checked = radio.value === value;
    });
    toggleAreaOwnerDetails();
}

// Detaljfeltet gir bare mening når svaret er "ja". Teksten beholdes (ikke tømmes) når man svarer
// "nei" - da mister man ingenting ved et feilklikk - men en skjult boks blir heller ikke med i
// utskriften, så et "nei" tar aldri med seg en gammel avtaletekst inn i søknaden.
function toggleAreaOwnerDetails() {
    document.getElementById("areaOwnerDetails").hidden = getAreaOwnerAnswer() !== "ja";
}

/* ---------- Kartutsnitt ---------- */

// Bygger opp listen over kartutsnitt på nytt. Med bare ett utsnitt får det ingen overskrift (den ville
// bare vært støy); fra og med to nummereres de, slik at beskrivelsen kan vise til "kartutsnitt 2".
function renderMaps() {
    const list = document.getElementById("mapList");
    list.innerHTML = "";

    mapItems.forEach(function (item, index) {
        const wrap = document.createElement("div");
        wrap.className = "nsm-map-item";

        if (mapItems.length > 1) {
            const title = document.createElement("div");
            title.className = "nsm-map-item-title";
            title.textContent = "Kartutsnitt " + (index + 1);
            wrap.appendChild(title);
        }

        const img = document.createElement("img");
        img.src = item.bilde;
        img.alt = "Kartutsnitt " + (index + 1);
        wrap.appendChild(img);

        const actions = document.createElement("div");
        actions.className = "nsm-map-actions no-print";

        const replaceBtn = document.createElement("button");
        replaceBtn.type = "button";
        replaceBtn.className = "btn btn-secondary";
        replaceBtn.innerHTML = '<i class="fa-solid fa-upload"></i> Last opp nytt bilde';
        replaceBtn.addEventListener("click", function () {
            mapUploadTarget = index;
            document.getElementById("mapInput").click();
        });
        actions.appendChild(replaceBtn);

        // Kom bildet fra tegneverktøyet, kan tegningen hentes fram og endres videre - da skal knappen
        // si nettopp det. Et opplastet bilde har ingen tegning bak seg, og knappen tilbyr i stedet å
        // lage et kart fra bunnen som erstatter bildet.
        const drawBtn = document.createElement("button");
        drawBtn.type = "button";
        drawBtn.className = "btn btn-secondary";
        drawBtn.innerHTML = item.tegning
            ? '<i class="fa-solid fa-draw-polygon"></i> Rediger tegningen'
            : '<i class="fa-solid fa-draw-polygon"></i> Tegn område på kart';
        drawBtn.addEventListener("click", function () { openMapDrawer(index); });
        actions.appendChild(drawBtn);

        const removeBtn = document.createElement("button");
        removeBtn.type = "button";
        removeBtn.className = "btn btn-danger";
        removeBtn.innerHTML = '<i class="fa-solid fa-trash"></i> Fjern';
        removeBtn.addEventListener("click", function () {
            if (!confirm("Fjerne dette kartutsnittet?")) return;
            mapItems.splice(index, 1);
            renderMaps();
            saveState();
        });
        actions.appendChild(removeBtn);

        wrap.appendChild(actions);
        list.appendChild(wrap);
    });

    // Droppsonen med de to "kom i gang"-knappene vises bare når det ikke finnes noen utsnitt ennå;
    // deretter overtar "Legg til"-knappene under listen.
    document.getElementById("mapDrop").hidden = mapItems.length > 0;
    document.getElementById("mapAddActions").hidden = mapItems.length === 0;
}

// Åpner tegneverktøyet (js/kart-tegner.js) og tar imot det ferdige bildet. index er hvilken oppføring
// som skal erstattes, eller -1 for å legge til en ny.
function openMapDrawer(index) {
    if (typeof openKartTegner !== "function") {
        alert("Kartverktøyet kunne ikke lastes.");
        return;
    }
    const existing = index >= 0 ? mapItems[index] : null;
    openKartTegner(existing ? existing.tegning : null, function (dataUrl, drawing) {
        const item = { bilde: dataUrl, tegning: drawing };
        if (existing) mapItems[index] = item;
        else mapItems.push(item);
        renderMaps();
        saveState();
    });
}

// Leser en bildefil (fra filvelger, dra-og-slipp eller utklippstavlen), skalerer den ned til
// MAP_MAX_WIDTH og gir tilbake en JPEG data-URL - se kommentaren ved MAP_MAX_WIDTH for hvorfor.
function readImageFile(file, done) {
    const reader = new FileReader();
    reader.onload = function () {
        const img = new Image();
        img.onload = function () {
            const scale = Math.min(1, MAP_MAX_WIDTH / img.naturalWidth);
            const canvas = document.createElement("canvas");
            canvas.width = Math.round(img.naturalWidth * scale);
            canvas.height = Math.round(img.naturalHeight * scale);
            const ctx = canvas.getContext("2d");
            // Hvit bunn: et PNG-skjermbilde kan ha gjennomsiktige partier, og de blir svarte i JPEG
            // uten dette.
            ctx.fillStyle = "#ffffff";
            ctx.fillRect(0, 0, canvas.width, canvas.height);
            ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
            done(canvas.toDataURL("image/jpeg", MAP_JPEG_QUALITY));
        };
        img.onerror = function () {
            alert("Kunne ikke lese bildefilen.");
        };
        img.src = reader.result;
    };
    reader.onerror = function () {
        alert("Kunne ikke lese bildefilen.");
    };
    reader.readAsDataURL(file);
}

// index: hvilken oppføring bildet skal erstatte, eller -1 for å legge til et nytt kartutsnitt.
function handleImageFile(file, index) {
    if (!file) return;
    readImageFile(file, function (dataUrl) {
        // Et opplastet bilde har ingen tegning bak seg - erstatter det et tegnet kart, forsvinner
        // også muligheten til å redigere den tegningen videre.
        const item = { bilde: dataUrl, tegning: null };
        if (typeof index === "number" && index >= 0 && mapItems[index]) mapItems[index] = item;
        else mapItems.push(item);
        renderMaps();
        saveState();
    });
}

/* ---------- Lagring, eksport og import ---------- */

function getState() {
    // "skjema" er kun en merkelapp til hjelp for et menneske som åpner JSON-filen - innlasting
    // gjenkjenner filen på "felt", ikke på dette navnet (se handleJsonFile).
    const state = {
        skjema: "NSM søknadskjema",
        __version: STATE_VERSION,
        felt: {},
        omradeeier: getAreaOwnerAnswer(),
        kontaktpunkt: opsContactRowVisible,
        uas: getUasList(),
        kontakter: getContacts(),
        kartutsnitt: mapItems
    };
    FIELD_IDS.forEach(function (id) {
        const el = document.getElementById(id);
        if (el) state.felt[id] = el.value;
    });
    return state;
}

let storageWarningShown = false;
function saveState() {
    try {
        localStorage.setItem(STORAGE_KEY, JSON.stringify(getState()));
    } catch (e) {
        // Lagringskvoten i nettleseren er full - i praksis bare mulig med et svært stort kartutsnitt.
        // Skjemaet fungerer fortsatt (og kan lastes ned som PDF/JSON), men innholdet overlever da ikke
        // en oppfriskning av siden, og det må brukeren få vite - én gang, ikke ved hvert tastetrykk.
        if (!storageWarningShown) {
            storageWarningShown = true;
            alert("Nettleseren klarte ikke å mellomlagre skjemaet - kartutsnittet er sannsynligvis for stort. Skjemaet virker som normalt, men innholdet blir ikke husket hvis du laster siden på nytt. Last ned som JSON for å ta vare på det.");
        }
    }
}

function applyState(state) {
    if (!state || typeof state !== "object") return;
    const felt = state.felt || {};
    Object.entries(felt).forEach(function (entry) {
        const el = document.getElementById(entry[0]);
        if (el) el.value = entry[1];
    });

    // UAS-blokker. Overgang fra den første utgaven av siden, der UAS-dataene var fire faste felt uten
    // "Legg til UAS": har et lagret skjema de gamle feltene og ingen "uas"-liste, løftes de over i én
    // blokk i stedet for å gå tapt.
    let uasList = state.uas || [];
    if (!uasList.length && (felt.nsmMake || felt.nsmSerial || felt.nsmWeight || felt.nsmSensor)) {
        uasList = [{
            fabrikat: felt.nsmMake || "",
            serienummer: felt.nsmSerial || "",
            flyvekt: felt.nsmWeight || "",
            sensor: felt.nsmSensor || ""
        }];
    }
    const uasContainer = document.getElementById("uasBlocks");
    uasContainer.innerHTML = "";
    uasList.forEach(function (uas) { addUasBlock(uas); });
    ensureOneUasBlock();
    renumberUasBlocks();

    // Ekstra kontaktpersoner. "personell" og "piloter" er navnene listen hadde i tidligere utgaver av
    // siden - de leses fortsatt, slik at et skjema lagret før omleggingen ikke mister radene sine.
    const contacts = state.kontakter || state.personell || state.piloter || [];
    const tbody = document.getElementById("contactRows");
    tbody.innerHTML = "";
    contacts.forEach(function (contact) { addContactRow(contact); });

    // Kontaktpunkt-raden er med som standard; bare et lagret "false" skjuler den.
    opsContactRowVisible = state.kontaktpunkt !== false;
    toggleOpsContactRow();

    setAreaOwnerAnswer(state.omradeeier || "");

    // Kartutsnitt. "kart"/"kartTegning" er det ene utsnittet siden hadde før den tok imot flere -
    // leses fortsatt, slik at et skjema lagret før den endringen beholder kartet sitt.
    if (state.kartutsnitt) {
        mapItems = state.kartutsnitt.filter(function (item) { return item && item.bilde; });
    } else if (state.kart) {
        mapItems = [{ bilde: state.kart, tegning: state.kartTegning || null }];
    } else {
        mapItems = [];
    }
    renderMaps();
    document.querySelectorAll("textarea").forEach(autoGrow);
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

// Fyller standardverdiene inn i felt som står tomme - kjøres ETTER at en eventuell lagret tilstand er
// lastet inn, slik at et skjema som ble lagret før et av disse feltene fantes også får verdien.
// Et felt brukeren selv har skrevet noe i, røres ikke.
function applyDefaults() {
    Object.entries(RENAMED_DEFAULTS).forEach(function (entry) {
        const el = document.getElementById(entry[0]);
        if (el && entry[1].indexOf(el.value.trim()) !== -1) el.value = "";
    });
    Object.entries(DEFAULT_FIELDS).forEach(function (entry) {
        const el = document.getElementById(entry[0]);
        if (el && !el.value) el.value = entry[1];
    });
}

function autoGrow(textarea) {
    textarea.style.height = "auto";
    textarea.style.height = textarea.scrollHeight + "px";
}

function sanitizeForFilename(s) {
    // Samme regel som i de andre skjemaene: fjerner tegn som er ugyldige i filnavn på
    // Windows/macOS/Linux, men beholder ellers teksten slik brukeren skrev den.
    return (s || "").replace(/[\\/:*?"<>|]/g, "").trim();
}

function formatDate(isoDate, separator) {
    if (!isoDate) return "";
    const parts = isoDate.split("-"); // <input type="date"> gir alltid ÅÅÅÅ-MM-DD
    return parts.length === 3 ? parts[2] + separator + parts[1] + separator + parts[0] : isoDate;
}

// Delt av JSON-nedlastingen og PDF-utskriften - nettleseren bruker document.title som forslag til
// filnavn i "Skriv ut / Lagre som PDF"-dialogen, så PDF-en får samme navn som JSON-filen.
function buildStandardFilename() {
    const org = sanitizeForFilename(document.getElementById("nsmOrgName").value) || "Uten navn";
    const date = formatDate(document.getElementById("nsmStartDate").value, "-") || "udatert";
    return "NSM søknadskjema - " + org + " - " + date;
}

// "Åpne e-post"-knappen øverst: en mailto-lenke med begge adressene og et ferdig emnefelt, slik at
// Outlook (eller det som er standard e-postklient) åpner seg med mottaker, kopi og emne på plass -
// selve PDF-en må brukeren legge ved selv, en mailto-lenke kan ikke legge ved filer. Emnet følger
// mønsteret "FFI - Søknad om flyging med sensor i sensorforbudsområde - [Område]" (brukerønske);
// området hentes fra punkt 3 og utelates så lenge feltet står tomt.
function updateMailLink() {
    const area = document.getElementById("nsmArea").value.trim();
    let subject = "FFI - Søknad om flyging med sensor i sensorforbudsområde";
    if (area) subject += " - " + area;
    // Kort standardtekst i selve meldingen, så e-posten ikke åpner seg helt tom. CRLF (ikke bare \n)
    // fordi Outlook er den e-postklienten dette havner i, og den er den kresne på linjeskift i en
    // mailto-lenke.
    const body = "Hei,\r\n\r\nSe vedlagt søknad.";
    const href = "mailto:luft@nsm.no?cc=rpas@ffi.no" +
        "&subject=" + encodeURIComponent(subject) +
        "&body=" + encodeURIComponent(body);
    // Knappen finnes to steder (øverst i stripen og nederst ved nedlastingsknappene) - begge merket
    // med js-mail-link, slik at de aldri kan komme ut av synk med hverandre.
    document.querySelectorAll(".js-mail-link").forEach(function (link) { link.href = href; });
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
   css/style.css) - uten det ville utskriften vist selve inputboksene med kantlinjer og datovelger-ikon
   i stedet for en ren, utfylt skjematabell. */

function syncPrintFields() {
    FIELD_IDS.forEach(function (id) {
        const el = document.getElementById(id);
        const target = document.querySelector('[data-print-for="' + id + '"]');
        if (!el || !target) return;
        const value = DATE_FIELD_IDS.indexOf(id) !== -1 ? formatDate(el.value, ".") : el.value;
        // Mellomrommet (ikke tom streng) holder radhøyden oppe for felt som ikke er fylt ut, slik at
        // tabellen ser like hel ut på papiret som på skjermen.
        target.textContent = value || " ";
    });

    const areaOwner = getAreaOwnerAnswer();
    document.getElementById("nsmAreaOwnerPrint").textContent =
        areaOwner === "ja" ? "Ja" : (areaOwner === "nei" ? "Nei" : " ");

    document.querySelectorAll(".nsm-uas-block").forEach(function (block) {
        UAS_FIELDS.forEach(function (field) {
            const input = block.querySelector("." + field.cls);
            input.parentNode.querySelector(".print-value").textContent = input.value || " ";
        });
    });

    document.querySelectorAll("#contactRows tr").forEach(function (tr) {
        let isEmpty = true;
        CONTACT_FIELDS.forEach(function (field) {
            const input = tr.querySelector("." + field.cls);
            input.parentNode.querySelector(".print-value").textContent = input.value || " ";
            if (input.value.trim()) isEmpty = false;
        });
        // Helt tomme rader (typisk en "Legg til kontaktperson" som aldri ble fylt ut) skrives ikke ut -
        // se .row-empty-print i css/style.css. De to faste kontaktradene ligger i HTML-en og er
        // ikke med her: de skal alltid stå i utskriften.
        tr.classList.toggle("row-empty-print", isEmpty);
    });
}

document.addEventListener("DOMContentLoaded", function () {
    ensureOneUasBlock();
    renumberUasBlocks();
    loadState();
    applyDefaults();
    // Kjøres også her, ikke bare fra applyState: et skjema uten lagret tilstand har ingen applyState
    // å gå gjennom, og "Legg til kontaktpunkt"-knappen må skjules fra start.
    toggleOpsContactRow();
    // Lagrer med én gang, slik at standardverdiene (og en utbyttet gammel standardtekst, se
    // RENAMED_DEFAULTS) også står riktig i det mellomlagrede skjemaet - ikke bare i feltet på skjermen.
    saveState();
    toggleAreaOwnerDetails();
    updateMailLink();

    FIELD_IDS.forEach(function (id) {
        const el = document.getElementById(id);
        if (!el) return;
        el.addEventListener("input", function () {
            if (el.tagName === "TEXTAREA") autoGrow(el);
            saveState();
        });
        el.addEventListener("change", saveState);
    });
    // Emnefeltet i "Åpne e-post" inneholder området - det må følge med mens brukeren skriver.
    document.getElementById("nsmArea").addEventListener("input", updateMailLink);
    document.querySelectorAll('input[name="nsmAreaOwner"]').forEach(function (radio) {
        radio.addEventListener("change", function () {
            toggleAreaOwnerDetails();
            saveState();
        });
    });
    document.querySelectorAll("textarea").forEach(autoGrow);

    // Enlinjefeltene (nsm-oneline) er <textarea> for å kunne vokse i høyden med lang tekst, men skal
    // ellers oppføre seg som et vanlig enlinjefelt: Enter skal ikke lage et linjeskift i et navn eller
    // en e-postadresse. Lytteren ligger på document, slik at den også gjelder felt som blir laget
    // senere (nye UAS-blokker og kontaktrader).
    document.addEventListener("keydown", function (e) {
        if (e.key === "Enter" && e.target.classList && e.target.classList.contains("nsm-oneline")) {
            e.preventDefault();
        }
    });

    document.getElementById("addUasBtn").addEventListener("click", function () {
        addUasBlock();
        saveState();
    });
    document.getElementById("addContactBtn").addEventListener("click", function () {
        addContactRow();
        saveState();
    });
    document.getElementById("removeOpsContactBtn").addEventListener("click", function () {
        if (!confirm("Fjerne raden med kontaktpunkt under UAS-operasjonene?")) return;
        ["nsmOpsContactRole", "nsmOpsContactName", "nsmOpsContactPhone", "nsmOpsContactEmail"]
            .forEach(function (id) { document.getElementById(id).value = ""; });
        opsContactRowVisible = false;
        toggleOpsContactRow();
        saveState();
    });
    document.getElementById("addOpsContactBtn").addEventListener("click", function () {
        opsContactRowVisible = true;
        toggleOpsContactRow();
        // Funksjonsteksten ble tømt ved slettingen - applyDefaults setter den tilbake til standarden.
        applyDefaults();
        document.querySelectorAll("#opsContactRow textarea").forEach(autoGrow);
        saveState();
    });

    /* ---------- Kartutsnitt: filvelger, dra-og-slipp og innliming ---------- */

    // Droppsonen har ingen egen klikkhandler: den inneholder to knapper, og et klikk på "Tegn område
    // på kart" ville ellers ALLTID også ha åpnet filvelgeren (klikket bobler opp).
    // Knappene på hvert enkelt kartutsnitt kobles opp i renderMaps.
    const mapInput = document.getElementById("mapInput");
    function chooseImageFile() {
        mapUploadTarget = -1;
        mapInput.click();
    }
    document.getElementById("mapUploadBtn").addEventListener("click", chooseImageFile);
    document.getElementById("mapAddUploadBtn").addEventListener("click", chooseImageFile);
    document.getElementById("mapDrawBtn").addEventListener("click", function () { openMapDrawer(-1); });
    document.getElementById("mapAddDrawBtn").addEventListener("click", function () { openMapDrawer(-1); });
    mapInput.addEventListener("change", function (e) {
        const input = e.target;
        handleImageFile(input.files && input.files[0], mapUploadTarget);
        mapUploadTarget = -1;
        input.value = ""; // samme fil kan velges på nytt senere uten at "change" uteblir
    });

    // Lim inn et skjermbilde rett i skjemaet (Ctrl+V) - den raskeste veien fra et kartutsnitt i
    // Norgeskart til skjemaet, uten omveien om å lagre en fil først.
    document.addEventListener("paste", function (e) {
        const items = (e.clipboardData && e.clipboardData.items) || [];
        for (let i = 0; i < items.length; i++) {
            if (items[i].type && items[i].type.indexOf("image/") === 0) {
                const file = items[i].getAsFile();
                if (file) {
                    e.preventDefault();
                    handleImageFile(file, -1);
                }
                return;
            }
        }
    });

    /* ---------- Utskrift og eksport ---------- */

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

    function downloadPdf() { window.print(); }
    document.getElementById("downloadPdfBtn").addEventListener("click", downloadPdf);
    document.getElementById("downloadPdfBtnTop").addEventListener("click", downloadPdf);

    function downloadJsonNow() {
        downloadJson(buildStandardFilename() + ".json", getState());
    }
    document.getElementById("downloadJsonBtn").addEventListener("click", downloadJsonNow);
    document.getElementById("downloadJsonBtnTop").addEventListener("click", downloadJsonNow);

    // Selve innlesingen av en JSON-fil - delt av den skjulte filvelgeren og dra-og-slipp-lytterne
    // under, i stedet for to sett med parse/valider/bekreft-logikk som kan drifte fra hverandre.
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
            if (!data || typeof data !== "object" || !data.felt) {
                alert("Fant ikke gjenkjennelig skjemainnhold i filen. Bruk en JSON-fil lastet ned med \"Last ned som JSON\" herfra.");
                return;
            }
            if (confirm("Laste inn skjemaet fra denne filen? Alt utfylt innhold blir erstattet.")) {
                applyState(data);
                saveState();
                updateMailLink();
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
        input.value = "";
    });

    // Dra-og-slipp hvor som helst i vinduet - samme mekanikk som de andre skjemaene (dragCounter, ikke
    // et enkelt boolsk flagg, fordi dragenter/dragleave fyres for HVERT element musen krysser inn i og
    // ut av). Her kan filen være to ting: en JSON-fil (hele skjemaet) eller et bilde (kartutsnittet).
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
        // Uten preventDefault her regnes vinduet som et ugyldig droppmål, og filen ville bare åpnet seg
        // i en ny fane i stedet for å utløse "drop" under.
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
        if (file.type && file.type.indexOf("image/") === 0) {
            handleImageFile(file, -1);
        } else if (/\.json$/i.test(file.name) || file.type === "application/json") {
            handleJsonFile(file);
        } else {
            alert("Dra inn en JSON-fil (.json) eller et bilde - " + file.name + " ser ikke ut til å være noen av delene.");
        }
    });

    document.getElementById("resetFormBtn").addEventListener("click", function () {
        if (confirm("Er du sikker på at du vil nullstille skjemaet? Alt utfylt innhold, inkludert kartutsnittet, blir slettet.")) {
            localStorage.removeItem(STORAGE_KEY);
            location.reload();
        }
    });
});
