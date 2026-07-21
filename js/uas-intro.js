/* js/uas-intro.js */

const STORAGE_KEY = "ffi-uas:uas-intro";
let sigPads = {};

function getFormState() {
    const state = { fields: {}, checks: {}, signatures: {} };
    document.querySelectorAll("#pilotName, #instructorName, #trainingDate").forEach(function (el) {
        state.fields[el.id] = el.value;
    });
    document.querySelectorAll(".checklist input[type=checkbox]").forEach(function (el) {
        state.checks[el.id] = el.checked;
    });
    Object.keys(sigPads).forEach(function (id) {
        state.signatures[id] = sigPads[id].toDataURL();
    });
    return state;
}

function saveState() {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(getFormState()));
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
    Object.entries(state.fields || {}).forEach(function (entry) {
        const el = document.getElementById(entry[0]);
        if (el) el.value = entry[1];
    });
    Object.entries(state.checks || {}).forEach(function (entry) {
        const el = document.getElementById(entry[0]);
        if (el) el.checked = entry[1];
    });
    Object.entries(state.signatures || {}).forEach(function (entry) {
        if (sigPads[entry[0]] && entry[1]) sigPads[entry[0]].fromDataURL(entry[1]);
    });
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

function buildSectionExport() {
    const sections = [];
    document.querySelectorAll(".section").forEach(function (sectionEl) {
        const h2 = sectionEl.querySelector(".section-header h2");
        if (!h2) return;
        const items = [];
        sectionEl.querySelectorAll(".checklist li").forEach(function (li) {
            const checkbox = li.querySelector('input[type="checkbox"]');
            const label = li.querySelector("label");
            if (!checkbox || !label) return;
            const hint = label.querySelector(".hint");
            let text = label.textContent.trim();
            if (hint) {
                text = text.replace(hint.textContent.trim(), "").trim();
            }
            items.push({ id: checkbox.id, tekst: text, utfort: checkbox.checked });
        });
        if (items.length > 0) {
            sections.push({ tittel: h2.textContent.trim(), punkter: items });
        }
    });
    return sections;
}

document.addEventListener("DOMContentLoaded", function () {
    const pilotInput = document.getElementById("pilotName");
    const instructorInput = document.getElementById("instructorName");
    const dateInput = document.getElementById("trainingDate");

    sigPads = initSignaturePads(["sigPilot", "sigInstructor"], saveState);

    loadState();

    if (!dateInput.value) {
        dateInput.value = new Date().toISOString().split("T")[0];
    }

    document.querySelectorAll(".form-field input, .checklist input[type=checkbox]").forEach(function (el) {
        el.addEventListener("input", saveState);
        el.addEventListener("change", saveState);
    });

    const printFields = document.getElementById("printFields");
    const printPilot = document.getElementById("printPilotName");
    const printInstructor = document.getElementById("printInstructorName");
    const printDate = document.getElementById("printTrainingDate");

    function syncPrintFields() {
        printPilot.textContent = pilotInput.value || " ";
        printInstructor.textContent = instructorInput.value || " ";
        printDate.textContent = dateInput.value || " ";
        printFields.style.display = "grid";
    }

    function hidePrintFields() {
        printFields.style.display = "none";
    }

    window.addEventListener("beforeprint", syncPrintFields);
    window.addEventListener("afterprint", hidePrintFields);

    document.getElementById("downloadPdfBtn").addEventListener("click", function () {
        window.print();
    });

    document.getElementById("downloadJsonBtn").addEventListener("click", function () {
        const data = {
            skjema: "UAS Intro",
            dronepilot: pilotInput.value,
            instruktor: instructorInput.value,
            dato: dateInput.value,
            seksjoner: buildSectionExport(),
            signaturer: {
                dronepilot: sigPads.sigPilot ? sigPads.sigPilot.toDataURL() : "",
                instruktor: sigPads.sigInstructor ? sigPads.sigInstructor.toDataURL() : ""
            }
        };
        const namePart = (pilotInput.value || "uas-intro").trim().replace(/\s+/g, "_");
        const datePart = dateInput.value || "udatert";
        downloadJson("uas-intro_" + datePart + "_" + namePart + ".json", data);
    });
});
