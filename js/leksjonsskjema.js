/* js/leksjonsskjema.js */

const STORAGE_KEY = "ffi-uas:leksjonsskjema";
const FIELD_IDS = ["lessonTitle", "lessonDate", "pilotName", "uasSystem", "instructorName", "remainingNotes"];
const GRADE_OPTIONS = ["", "D", "1", "2", "3"];
const DEFAULT_EXERCISES = [
    { exercise: "Inspeksjon av fartøyet" },
    { exercise: "Avgang" },
    { exercise: "Kontroll" },
    { exercise: "Innflyging" },
    { exercise: "Avbrutt landing" },
    { exercise: "Landing" },
    { exercise: "Forberedelser", generic: true },
    { exercise: "Sjekklistebruk", generic: true },
    { exercise: "Situasjonsforståelse", generic: true }
];
let sigPads = {};

function createRow(data) {
    data = data || { exercise: "", grade: "", comment: "" };
    const tr = document.createElement("tr");
    if (data.generic) tr.classList.add("generic-row");

    const tdNum = document.createElement("td");
    tdNum.className = "row-num";
    tr.appendChild(tdNum);

    const tdName = document.createElement("td");
    const nameInput = document.createElement("input");
    nameInput.type = "text";
    nameInput.className = "ex-name";
    nameInput.placeholder = "Øvelsesnavn";
    nameInput.value = data.exercise || "";
    tdName.appendChild(nameInput);
    tr.appendChild(tdName);

    const tdGrade = document.createElement("td");
    tdGrade.className = "col-grade";
    const select = document.createElement("select");
    select.className = "ex-grade";
    GRADE_OPTIONS.forEach(function (val) {
        const opt = document.createElement("option");
        opt.value = val;
        opt.textContent = val === "" ? "–" : val;
        if (data.grade === val) opt.selected = true;
        select.appendChild(opt);
    });
    tdGrade.appendChild(select);
    tr.appendChild(tdGrade);

    const tdComment = document.createElement("td");
    const commentInput = document.createElement("textarea");
    commentInput.className = "ex-comment";
    commentInput.rows = 1;
    commentInput.placeholder = "Kommentar";
    commentInput.value = data.comment || "";
    commentInput.addEventListener("input", function () { autoGrow(commentInput); });
    tdComment.appendChild(commentInput);
    tr.appendChild(tdComment);

    const tdRemove = document.createElement("td");
    tdRemove.className = "col-remove";
    const removeBtn = document.createElement("button");
    removeBtn.type = "button";
    removeBtn.className = "remove-row-btn no-print";
    removeBtn.title = "Fjern øvelse";
    removeBtn.innerHTML = '<i class="fa-solid fa-trash"></i>';
    removeBtn.addEventListener("click", function () {
        tr.remove();
        renumberRows();
        saveState();
    });
    tdRemove.appendChild(removeBtn);
    tr.appendChild(tdRemove);

    tr.querySelectorAll("input, select, textarea").forEach(function (el) {
        el.addEventListener("input", saveState);
        el.addEventListener("change", saveState);
    });

    return tr;
}

function autoGrow(textarea) {
    textarea.style.height = "auto";
    textarea.style.height = textarea.scrollHeight + "px";
}

function renumberRows() {
    let n = 0;
    document.querySelectorAll("#exerciseRows tr").forEach(function (tr) {
        const numCell = tr.querySelector(".row-num");
        if (tr.classList.contains("generic-row")) {
            numCell.textContent = "";
        } else {
            n++;
            numCell.textContent = n;
        }
    });
}

function addRow(data) {
    const tr = createRow(data);
    const tbody = document.getElementById("exerciseRows");
    const firstGeneric = tbody.querySelector(".generic-row");
    if (!(data && data.generic) && firstGeneric) {
        tbody.insertBefore(tr, firstGeneric);
    } else {
        tbody.appendChild(tr);
    }
    autoGrow(tr.querySelector(".ex-comment"));
    renumberRows();
}

function getState() {
    const state = { fields: {}, rows: [], signatures: {} };
    FIELD_IDS.forEach(function (id) {
        const el = document.getElementById(id);
        if (el) state.fields[id] = el.value;
    });
    document.querySelectorAll("#exerciseRows tr").forEach(function (tr) {
        state.rows.push({
            exercise: tr.querySelector(".ex-name").value,
            grade: tr.querySelector(".ex-grade").value,
            comment: tr.querySelector(".ex-comment").value,
            generic: tr.classList.contains("generic-row")
        });
    });
    Object.keys(sigPads).forEach(function (id) {
        state.signatures[id] = sigPads[id].toDataURL();
    });
    return state;
}

function saveState() {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(getState()));
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

function loadState() {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return false;
    let state;
    try {
        state = JSON.parse(raw);
    } catch (e) {
        return false;
    }

    Object.entries(state.fields || {}).forEach(function (entry) {
        const el = document.getElementById(entry[0]);
        if (el) el.value = entry[1];
    });
    (state.rows || []).forEach(function (row) { addRow(row); });
    Object.entries(state.signatures || {}).forEach(function (entry) {
        if (sigPads[entry[0]] && entry[1]) sigPads[entry[0]].fromDataURL(entry[1]);
    });
    return (state.rows || []).length > 0;
}

document.addEventListener("DOMContentLoaded", function () {
    sigPads = initSignaturePads(["sigInstructor", "sigPilot"], saveState);

    const hadSavedRows = loadState();

    const dateInput = document.getElementById("lessonDate");
    if (!dateInput.value) {
        dateInput.value = new Date().toISOString().split("T")[0];
    }

    if (!hadSavedRows) {
        DEFAULT_EXERCISES.forEach(function (item) { addRow(item); });
    }

    document.getElementById("addRowBtn").addEventListener("click", function () {
        addRow();
        saveState();
    });

    document.querySelectorAll(".form-field input, #remainingNotes").forEach(function (el) {
        el.addEventListener("input", saveState);
        el.addEventListener("change", saveState);
    });

    function syncPrintFields() {
        document.getElementById("printLessonTitle").textContent = document.getElementById("lessonTitle").value || " ";
        document.getElementById("printLessonTitleWrap").style.display = "block";
        document.getElementById("printLessonDate").textContent = document.getElementById("lessonDate").value || " ";
        document.getElementById("printPilotName").textContent = document.getElementById("pilotName").value || " ";
        document.getElementById("printUasSystem").textContent = document.getElementById("uasSystem").value || " ";
        document.getElementById("printInstructorName").textContent = document.getElementById("instructorName").value || " ";
        document.getElementById("printFields").style.display = "grid";
    }

    function hidePrintFields() {
        document.getElementById("printFields").style.display = "none";
        document.getElementById("printLessonTitleWrap").style.display = "none";
    }

    window.addEventListener("beforeprint", syncPrintFields);
    window.addEventListener("afterprint", hidePrintFields);

    document.getElementById("downloadPdfBtn").addEventListener("click", function () {
        window.print();
    });

    document.getElementById("downloadJsonBtn").addEventListener("click", function () {
        const state = getState();
        const data = Object.assign({ skjema: "Leksjonsskjema" }, state);
        const slug = function (s) { return (s || "").trim().replace(/\s+/g, "_"); };
        const titlePart = slug(state.fields.lessonTitle) || "leksjon";
        const pilotPart = slug(state.fields.pilotName) || "pilot";
        const datePart = state.fields.lessonDate || "udatert";
        downloadJson(titlePart + "-" + pilotPart + "-" + datePart + ".json", data);
    });

    document.getElementById("resetFormBtn").addEventListener("click", function () {
        if (confirm("Er du sikker på at du vil nullstille skjemaet? Alt utfylt innhold blir slettet.")) {
            localStorage.removeItem(STORAGE_KEY);
            location.reload();
        }
    });
});
