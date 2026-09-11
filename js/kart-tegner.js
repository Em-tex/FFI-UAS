/* js/kart-tegner.js
   Enkel kartvisning med områdetegning, som gir fra seg et ferdig bilde.

   Bygget uten kartbibliotek (ingen Leaflet e.l.): hele kartet tegnes i en <canvas> som vi uansett må
   ha for å kunne eksportere resultatet som et bilde. Med et vanlig kartbibliotek ligger flisene som
   <img>-elementer i DOM-en, og de må uansett rasteriseres til en canvas før de kan lagres - da er det
   enklere å tegne rett i canvas fra starten.

   Flisene hentes fra Kartverkets åpne WMTS-cache, som sender "Access-Control-Allow-Origin: *". Det er
   avgjørende: uten CORS-headeren ville canvasen blitt "tainted", og toDataURL() ville kastet en
   SecurityError i stedet for å gi oss bildet (se exportImage). Derfor lastes flisene med
   crossOrigin = "anonymous". */

// Kartlagene man kan velge mellom. Gråtone er standard (brukerønske): et dempet bakgrunnskart gjør det
// markerte området tydelig, mens fargekartet fort konkurrerer med markeringen om oppmerksomheten.
// Fargekart og flybilde er med som valg - terrenget er lettere å kjenne igjen der.
//
// Alle tre MÅ sende "Access-Control-Allow-Origin" (alle gjør det, sjekket), ellers blir canvasen
// tainted og bildet kan ikke eksporteres - se kommentaren øverst i filen.
//
// Flybildet er Esri World Imagery, ikke Googles fliser (mt1.google.com/vt): Google-endepunktet svarer
// riktignok med CORS-header og ville fungert teknisk, men det er Google Maps' interne flistjeneste og
// ligger utenfor vilkårene for bruk utenfor Googles egne API-er. Bildet havner her i en formell søknad
// til NSM, så kilden bør være en vi har lov til å bruke. Esri World Imagery er åpent tilgjengelig mot
// kildehenvisning (se attribution under, som tegnes inn i selve bildet).
const KART_LAYERS = {
    graatone: {
        label: "Gråtone",
        url: "https://cache.kartverket.no/v1/wmts/1.0.0/topograatone/default/webmercator/{z}/{y}/{x}.png",
        attribution: "© Kartverket"
    },
    farger: {
        label: "Farger",
        url: "https://cache.kartverket.no/v1/wmts/1.0.0/topo/default/webmercator/{z}/{y}/{x}.png",
        attribution: "© Kartverket"
    },
    flybilde: {
        label: "Flybilde",
        url: "https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attribution: "© Esri, Maxar, Earthstar Geographics"
    }
};
const KART_DEFAULT_LAYER = "graatone";
const KART_SEARCH_URL = "https://ws.geonorge.no/stedsnavn/v1/sted";

// Hele Norge som utgangspunkt når brukeren ikke har tegnet noe fra før.
const KART_DEFAULT_VIEW = { lat: 64.8, lon: 13.5, zoom: 4 };
const KART_MIN_ZOOM = 3;
const KART_MAX_ZOOM = 17;

// Flisene hentes fra ett zoomnivå DYPERE enn kartet vises i, og tegnes på halv størrelse (128 css-px
// i stedet for 256). Sammen med KART_SCALE (to interne piksler per css-piksel) gir det et skarpt
// bilde både på skjerm og i utskrift - samme triks som "retina"-fliser i vanlige kartbiblioteker.
const KART_TILE_CSS = 128;
const KART_SCALE = 2;

// Fargene man kan tegne området med. Fyllet er gjennomsiktig slik at kartet fortsatt synes gjennom
// markeringen, mens kantlinjen er i full farge. Flere farger fordi en søknad kan vise flere ting i
// samme kart (f.eks. flyområde i grønt og et område som skal unngås i rødt), og fordi en farge som
// funker på gråtonekartet ikke nødvendigvis skiller seg ut på et flybilde.
const KART_COLORS = [
    { key: "gronn", label: "Grønn", stroke: "#1b5e20", fill: "rgba(46, 125, 50, 0.28)" },
    { key: "bla", label: "Blå", stroke: "#0d47a1", fill: "rgba(21, 101, 192, 0.28)" },
    { key: "rod", label: "Rød", stroke: "#b71c1c", fill: "rgba(211, 47, 47, 0.28)" },
    { key: "oransje", label: "Oransje", stroke: "#e65100", fill: "rgba(245, 124, 0, 0.32)" },
    { key: "lilla", label: "Lilla", stroke: "#4a148c", fill: "rgba(123, 31, 162, 0.28)" },
    { key: "svart", label: "Svart", stroke: "#000000", fill: "rgba(0, 0, 0, 0.22)" }
];
const KART_DEFAULT_COLOR = "gronn";

function kartColor(key) {
    for (let i = 0; i < KART_COLORS.length; i++) {
        if (KART_COLORS[i].key === key) return KART_COLORS[i];
    }
    return KART_COLORS[0];
}

/* ---------- Projeksjon (Web Mercator) ----------
   Alle "world"-koordinater er css-piksler på det aktuelle zoomnivået, der hele verden er
   256 * 2^zoom piksler bred. */

function kartLonToWorldX(lon, zoom) {
    return (lon + 180) / 360 * 256 * Math.pow(2, zoom);
}

function kartLatToWorldY(lat, zoom) {
    const s = Math.sin(lat * Math.PI / 180);
    return (0.5 - Math.log((1 + s) / (1 - s)) / (4 * Math.PI)) * 256 * Math.pow(2, zoom);
}

function kartWorldXToLon(x, zoom) {
    return x / (256 * Math.pow(2, zoom)) * 360 - 180;
}

function kartWorldYToLat(y, zoom) {
    const n = Math.PI - 2 * Math.PI * y / (256 * Math.pow(2, zoom));
    return 180 / Math.PI * Math.atan(0.5 * (Math.exp(n) - Math.exp(-n)));
}

/* ---------- Selve verktøyet ----------
   openKartTegner(startvisning, nårLagret) åpner en dialog over siden. "startvisning" er enten null
   (nytt kart) eller det som ble lagret sist ({ lat, lon, zoom, punkter }), slik at en tidligere
   tegning kan redigeres videre i stedet for å måtte tegnes på nytt. Når brukeren lagrer, kalles
   nårLagret(bildeDataUrl, visningen) - siden som bruker verktøyet bestemmer selv hva bildet skal
   brukes til. */

function openKartTegner(start, onSave) {
    start = start || {};
    const view = {
        lat: typeof start.lat === "number" ? start.lat : KART_DEFAULT_VIEW.lat,
        lon: typeof start.lon === "number" ? start.lon : KART_DEFAULT_VIEW.lon,
        zoom: typeof start.zoom === "number" ? start.zoom : KART_DEFAULT_VIEW.zoom,
        lag: KART_LAYERS[start.lag] ? start.lag : KART_DEFAULT_LAYER,
        farge: start.farge || KART_DEFAULT_COLOR
    };
    let points = (start.punkter || []).map(function (p) { return { lat: p.lat, lon: p.lon }; });

    /* ----- DOM ----- */

    const overlay = document.createElement("div");
    overlay.className = "mapdraw-overlay no-print";
    overlay.innerHTML =
        '<div class="mapdraw-card" role="dialog" aria-modal="true" aria-label="Tegn område på kart">' +
            '<div class="mapdraw-head">' +
                '<h3>Tegn område på kart</h3>' +
                '<button type="button" class="mapdraw-close" title="Lukk"><i class="fa-solid fa-xmark"></i></button>' +
            '</div>' +
            '<div class="mapdraw-search">' +
                '<i class="fa-solid fa-magnifying-glass mapdraw-search-icon"></i>' +
                '<input type="text" class="mapdraw-search-input" placeholder="Søk etter sted" autocomplete="off">' +
                '<div class="mapdraw-suggest" hidden></div>' +
            '</div>' +
            '<div class="mapdraw-canvas-wrap">' +
                '<canvas class="mapdraw-canvas"></canvas>' +
                '<div class="mapdraw-zoom">' +
                    '<button type="button" class="mapdraw-zoom-btn" data-zoom="1" title="Zoom inn">+</button>' +
                    '<button type="button" class="mapdraw-zoom-btn" data-zoom="-1" title="Zoom ut">&minus;</button>' +
                '</div>' +
                '<div class="mapdraw-layers">' +
                    '<button type="button" class="mapdraw-layer-btn" data-layer="graatone">Gråtone</button>' +
                    '<button type="button" class="mapdraw-layer-btn" data-layer="farger">Farger</button>' +
                    '<button type="button" class="mapdraw-layer-btn" data-layer="flybilde">Flybilde</button>' +
                '</div>' +
                // Smal stripe øverst i kartet, ikke et dekkende lag: kommer det inn fliser likevel,
                // skal de være synlige og kartet fullt brukbart.
                '<div class="mapdraw-offline" hidden>' +
                    '<i class="fa-solid fa-triangle-exclamation"></i>' +
                    '<span>Fikk ikke lastet kartfliser. Sjekk nettforbindelsen, prøv et annet kartlag, eller last opp et skjermbilde i stedet.</span>' +
                '</div>' +
            '</div>' +
            '<p class="mapdraw-hint">' +
                'Klikk i kartet for å sette hjørnene i området. Dra for å flytte kartet, og bruk rullehjulet for å zoome.' +
            '</p>' +
            '<div class="mapdraw-actions">' +
                '<div class="mapdraw-colors" role="group" aria-label="Farge på området">' +
                    KART_COLORS.map(function (c) {
                        return '<button type="button" class="mapdraw-color-btn" data-color="' + c.key +
                            '" title="' + c.label + '" style="background:' + c.stroke + '"></button>';
                    }).join("") +
                '</div>' +
                '<button type="button" class="btn btn-secondary mapdraw-undo"><i class="fa-solid fa-rotate-left"></i> Angre punkt</button>' +
                '<button type="button" class="btn btn-secondary mapdraw-clear"><i class="fa-solid fa-eraser"></i> Tøm området</button>' +
                '<span class="mapdraw-actions-spacer"></span>' +
                '<button type="button" class="btn btn-secondary mapdraw-cancel">Avbryt</button>' +
                '<button type="button" class="btn mapdraw-save"><i class="fa-solid fa-check"></i> Bruk som kartutsnitt</button>' +
            '</div>' +
        '</div>';
    document.body.appendChild(overlay);

    const canvas = overlay.querySelector(".mapdraw-canvas");
    const ctx = canvas.getContext("2d");
    const searchInput = overlay.querySelector(".mapdraw-search-input");
    const suggestBox = overlay.querySelector(".mapdraw-suggest");
    const offlineBox = overlay.querySelector(".mapdraw-offline");

    let viewW = 0;
    let viewH = 0;

    // Canvasen får en indre oppløsning på det dobbelte av visningsstørrelsen (KART_SCALE), slik at
    // bildet som lagres er stort nok til å være skarpt i en A4-utskrift.
    function sizeCanvas() {
        const rect = canvas.getBoundingClientRect();
        viewW = Math.round(rect.width);
        viewH = Math.round(rect.height);
        canvas.width = viewW * KART_SCALE;
        canvas.height = viewH * KART_SCALE;
        draw();
    }

    /* ----- Fliser ----- */

    const tiles = {};
    let pendingTiles = 0;
    let loadedTiles = 0;
    let drawQueued = false;

    function queueDraw() {
        if (drawQueued) return;
        drawQueued = true;
        requestAnimationFrame(function () {
            drawQueued = false;
            draw();
        });
    }

    function getTile(z, x, y) {
        // Flisene fra de to kartlagene er ulike bilder og må caches hver for seg.
        const key = view.lag + "/" + z + "/" + x + "/" + y;
        if (tiles[key]) return tiles[key];
        const img = new Image();
        // MÅ settes før src: uten dette blir canvasen tainted og bildet kan ikke eksporteres.
        img.crossOrigin = "anonymous";
        img.dataset.ready = "";
        pendingTiles++;
        img.onload = function () {
            img.dataset.ready = "1";
            pendingTiles--;
            loadedTiles++;
            queueDraw();
        };
        img.onerror = function () {
            pendingTiles--;
        };
        img.src = KART_LAYERS[view.lag].url
            .replace("{z}", z)
            .replace("{y}", y)
            .replace("{x}", x);
        tiles[key] = img;
        return img;
    }

    // Advarselen om manglende kartfliser styres av ÉN regel, vurdert på nytt i hver opptegning (se
    // draw): den vises bare når ventetiden er ute OG ingen fliser i det hele tatt har kommet inn.
    // Enkeltfliser kan feile helt normalt (f.eks. utenfor dekningsområdet), så en onerror alene er
    // ingen grunn til å advare - og like viktig: advarselen forsvinner av seg selv i det den første
    // flisen kommer, uansett hvilken rekkefølge tidtakeren og innlastingene skjer i.
    let warnDelayPassed = false;
    setTimeout(function () {
        warnDelayPassed = true;
        queueDraw();
    }, 5000);

    /* ----- Tegning ----- */

    function topLeftWorld() {
        return {
            x: kartLonToWorldX(view.lon, view.zoom) - viewW / 2,
            y: kartLatToWorldY(view.lat, view.zoom) - viewH / 2
        };
    }

    function latLonToScreen(lat, lon) {
        const tl = topLeftWorld();
        return {
            x: kartLonToWorldX(lon, view.zoom) - tl.x,
            y: kartLatToWorldY(lat, view.zoom) - tl.y
        };
    }

    function screenToLatLon(x, y) {
        const tl = topLeftWorld();
        return {
            lat: kartWorldYToLat(tl.y + y, view.zoom),
            lon: kartWorldXToLon(tl.x + x, view.zoom)
        };
    }

    function draw(options) {
        options = options || {};
        if (!viewW || !viewH) return;
        ctx.setTransform(KART_SCALE, 0, 0, KART_SCALE, 0, 0);
        ctx.fillStyle = "#e8eef2";
        ctx.fillRect(0, 0, viewW, viewH);

        const tileZ = Math.min(KART_MAX_ZOOM + 1, view.zoom + 1);
        const tileCount = Math.pow(2, tileZ);
        const tl = topLeftWorld();
        const firstX = Math.floor(tl.x / KART_TILE_CSS);
        const firstY = Math.floor(tl.y / KART_TILE_CSS);
        const lastX = Math.floor((tl.x + viewW) / KART_TILE_CSS);
        const lastY = Math.floor((tl.y + viewH) / KART_TILE_CSS);

        for (let ty = firstY; ty <= lastY; ty++) {
            if (ty < 0 || ty >= tileCount) continue;
            for (let tx = firstX; tx <= lastX; tx++) {
                // Kartet går rundt jorda i øst-vest-retning, så x-indeksen wrappes i stedet for å
                // kuttes - ellers ville kartet blitt svart hvis man drar forbi datolinjen.
                const wrappedX = ((tx % tileCount) + tileCount) % tileCount;
                const img = getTile(tileZ, wrappedX, ty);
                if (!img.dataset.ready) continue;
                const sx = tx * KART_TILE_CSS - tl.x;
                const sy = ty * KART_TILE_CSS - tl.y;
                // +1 på bredde/høyde: uten det kan avrundingen mellom to nabofliser gi en tynn, lys
                // strek mellom dem (synlige "sømmer" i kartet, også i det lagrede bildet).
                ctx.drawImage(img, sx, sy, KART_TILE_CSS + 1, KART_TILE_CSS + 1);
            }
        }

        drawArea(options.forExport);
        drawAttribution();
        offlineBox.hidden = !(warnDelayPassed && loadedTiles === 0);
    }

    function drawArea(forExport) {
        if (!points.length) return;
        const color = kartColor(view.farge);
        const screen = points.map(function (p) { return latLonToScreen(p.lat, p.lon); });

        ctx.beginPath();
        screen.forEach(function (p, i) {
            if (i === 0) ctx.moveTo(p.x, p.y);
            else ctx.lineTo(p.x, p.y);
        });
        if (screen.length > 2) {
            ctx.closePath();
            ctx.fillStyle = color.fill;
            ctx.fill();
        }
        ctx.strokeStyle = color.stroke;
        ctx.lineWidth = 2;
        ctx.lineJoin = "round";
        ctx.stroke();

        // Hjørnemarkørene er et redigeringshjelpemiddel og skal ikke være med i bildet som lagres.
        if (forExport) return;
        screen.forEach(function (p) {
            ctx.beginPath();
            ctx.arc(p.x, p.y, 4, 0, Math.PI * 2);
            ctx.fillStyle = "#ffffff";
            ctx.fill();
            ctx.strokeStyle = color.stroke;
            ctx.lineWidth = 2;
            ctx.stroke();
        });
    }

    // Kartdataene er åpne, men krever kildehenvisning - den tegnes inn i selve bildet, slik at den
    // følger med i PDF-en søknaden sendes som. Teksten følger valgt kartlag.
    function drawAttribution() {
        ctx.font = "11px 'Overpass', sans-serif";
        const text = KART_LAYERS[view.lag].attribution;
        const w = ctx.measureText(text).width + 10;
        ctx.fillStyle = "rgba(255, 255, 255, 0.75)";
        ctx.fillRect(viewW - w - 4, viewH - 20, w, 16);
        ctx.fillStyle = "#333";
        ctx.textBaseline = "middle";
        ctx.fillText(text, viewW - w + 1, viewH - 12);
    }

    /* ----- Panorering, zoom og punkter ----- */

    let dragging = false;
    let moved = false;
    let lastX = 0;
    let lastY = 0;

    function canvasPos(e) {
        const rect = canvas.getBoundingClientRect();
        return { x: e.clientX - rect.left, y: e.clientY - rect.top };
    }

    canvas.addEventListener("pointerdown", function (e) {
        dragging = true;
        moved = false;
        const p = canvasPos(e);
        lastX = p.x;
        lastY = p.y;
        canvas.setPointerCapture(e.pointerId);
    });

    canvas.addEventListener("pointermove", function (e) {
        if (!dragging) return;
        const p = canvasPos(e);
        const dx = p.x - lastX;
        const dy = p.y - lastY;
        // Noen få piksler regnes fortsatt som et klikk - ellers ville et lite skjelv med musen gjort
        // det umulig å sette et punkt.
        if (Math.abs(dx) > 3 || Math.abs(dy) > 3) moved = true;
        lastX = p.x;
        lastY = p.y;
        const tl = topLeftWorld();
        view.lon = kartWorldXToLon(tl.x - dx + viewW / 2, view.zoom);
        view.lat = kartWorldYToLat(tl.y - dy + viewH / 2, view.zoom);
        queueDraw();
    });

    canvas.addEventListener("pointerup", function (e) {
        if (!dragging) return;
        dragging = false;
        if (moved) return;
        const p = canvasPos(e);
        const geo = screenToLatLon(p.x, p.y);
        points.push(geo);
        draw();
    });

    canvas.addEventListener("pointercancel", function () { dragging = false; });

    // Et dobbeltklikk for å "avslutte" tegningen er en innarbeidet vane - flaten lukkes automatisk her,
    // så det eneste dobbeltklikket ellers ville gjort er å legge inn et ekstra punkt oppå det forrige.
    canvas.addEventListener("dblclick", function () {
        if (points.length > 1) {
            points.pop();
            draw();
        }
    });

    canvas.addEventListener("wheel", function (e) {
        e.preventDefault();
        const p = canvasPos(e);
        zoomBy(e.deltaY < 0 ? 1 : -1, p.x, p.y);
    }, { passive: false });

    // Zoomer rundt et punkt: koordinatet under musepekeren (eller midt i kartet) skal bli liggende der
    // det er, slik at man zoomer INN PÅ det man ser på og ikke mister det ut av bildet.
    function zoomBy(step, anchorX, anchorY) {
        const ax = typeof anchorX === "number" ? anchorX : viewW / 2;
        const ay = typeof anchorY === "number" ? anchorY : viewH / 2;
        const geo = screenToLatLon(ax, ay);
        const newZoom = Math.max(KART_MIN_ZOOM, Math.min(KART_MAX_ZOOM, view.zoom + step));
        if (newZoom === view.zoom) return;
        view.zoom = newZoom;
        const wx = kartLonToWorldX(geo.lon, view.zoom);
        const wy = kartLatToWorldY(geo.lat, view.zoom);
        view.lon = kartWorldXToLon(wx - ax + viewW / 2, view.zoom);
        view.lat = kartWorldYToLat(wy - ay + viewH / 2, view.zoom);
        draw();
    }

    overlay.querySelectorAll(".mapdraw-zoom-btn").forEach(function (btn) {
        btn.addEventListener("click", function () {
            zoomBy(parseInt(btn.getAttribute("data-zoom"), 10));
        });
    });

    const layerBtns = overlay.querySelectorAll(".mapdraw-layer-btn");
    function updateLayerButtons() {
        layerBtns.forEach(function (btn) {
            btn.classList.toggle("active", btn.getAttribute("data-layer") === view.lag);
        });
    }
    layerBtns.forEach(function (btn) {
        btn.addEventListener("click", function () {
            view.lag = btn.getAttribute("data-layer");
            updateLayerButtons();
            draw();
        });
    });
    updateLayerButtons();

    const colorBtns = overlay.querySelectorAll(".mapdraw-color-btn");
    function updateColorButtons() {
        colorBtns.forEach(function (btn) {
            btn.classList.toggle("active", btn.getAttribute("data-color") === view.farge);
        });
    }
    colorBtns.forEach(function (btn) {
        btn.addEventListener("click", function () {
            view.farge = btn.getAttribute("data-color");
            updateColorButtons();
            draw();
        });
    });
    updateColorButtons();

    overlay.querySelector(".mapdraw-undo").addEventListener("click", function () {
        points.pop();
        draw();
    });

    overlay.querySelector(".mapdraw-clear").addEventListener("click", function () {
        points = [];
        draw();
    });

    /* ----- Stedsnavnsøk -----
       Kartverkets stedsnavn-API gir en liste med treff, og de vises som forslag mens man skriver.
       Navnet alene er sjelden nok - det finnes f.eks. mange "Storvatnet" - så hvert forslag viser
       også hva slags sted det er, og hvilken kommune og hvilket fylke det ligger i. */

    // Hvert treff i API-et har navnene sine i en egen liste (hovednavn, samiske navn, eldre
    // skrivemåter ...). Vi viser det første, som er det prioriterte hovednavnet.
    function hitName(hit) {
        const names = hit.stedsnavn || [];
        return (names[0] && names[0].skrivemåte) || "Uten navn";
    }

    function hitPlace(hit) {
        const parts = [];
        if (hit.navneobjekttype) parts.push(hit.navneobjekttype);
        (hit.kommuner || []).forEach(function (k) { if (k.kommunenavn) parts.push(k.kommunenavn); });
        (hit.fylker || []).forEach(function (f) { if (f.fylkesnavn) parts.push(f.fylkesnavn); });
        return parts.join(" · ");
    }

    function hideSuggestions() {
        suggestBox.hidden = true;
        suggestBox.innerHTML = "";
    }

    function showMessage(text) {
        suggestBox.innerHTML = '<div class="mapdraw-suggest-msg"></div>';
        suggestBox.firstChild.textContent = text;
        suggestBox.hidden = false;
    }

    function goToHit(hit) {
        if (!hit || !hit.representasjonspunkt) return;
        // Søkefeltet viser hva man faktisk valgte - ellers blir det stående med det halvskrevne ordet
        // man søkte med, og det er ikke lenger til å se hvilket sted kartet står på.
        searchInput.value = hitName(hit);
        view.lat = hit.representasjonspunkt.nord;
        view.lon = hit.representasjonspunkt.øst;
        // Zoomer inn på stedet hvis kartet står langt ute, men zoomer aldri UT fra et nivå brukeren
        // selv har valgt.
        view.zoom = Math.max(view.zoom, 13);
        hideSuggestions();
        draw();
    }

    function renderSuggestions(hits) {
        suggestBox.innerHTML = "";
        hits.forEach(function (hit) {
            const item = document.createElement("button");
            item.type = "button";
            item.className = "mapdraw-suggest-item";
            const name = document.createElement("span");
            name.className = "mapdraw-suggest-name";
            name.textContent = hitName(hit);
            const place = document.createElement("span");
            place.className = "mapdraw-suggest-place";
            place.textContent = hitPlace(hit);
            item.appendChild(name);
            item.appendChild(place);
            item.addEventListener("click", function () { goToHit(hit); });
            suggestBox.appendChild(item);
        });
        suggestBox.hidden = hits.length === 0;
    }

    let searchTimer = null;
    let searchSeq = 0;
    let lastHits = [];

    function search(q) {
        // Hvert søk får et løpenummer: svar fra et eldre søk som kommer inn etter et nyere skal
        // ignoreres, ellers kan forslagene til det man skrev for to tastetrykk siden vinne til slutt.
        const seq = ++searchSeq;
        // "*" gir treff på begynnelsen av navnet mens man skriver; fuzzy=false holder listen relevant.
        fetch(KART_SEARCH_URL + "?sok=" + encodeURIComponent(q + "*") + "&fuzzy=false&treffPerSide=6&side=1")
            .then(function (res) { return res.json(); })
            .then(function (data) {
                if (seq !== searchSeq) return;
                lastHits = (data && data.navn) || [];
                if (!lastHits.length) {
                    showMessage("Fant ingen steder.");
                    return;
                }
                renderSuggestions(lastHits);
            })
            .catch(function () {
                if (seq !== searchSeq) return;
                lastHits = [];
                showMessage("Søket feilet - ingen nettforbindelse?");
            });
    }

    searchInput.addEventListener("input", function () {
        const q = searchInput.value.trim();
        clearTimeout(searchTimer);
        if (q.length < 2) {
            hideSuggestions();
            return;
        }
        // Liten forsinkelse: ett søk per tastetrykk ville gitt et kall til Kartverket for hver bokstav.
        searchTimer = setTimeout(function () { search(q); }, 250);
    });

    searchInput.addEventListener("keydown", function (e) {
        if (e.key === "Enter") {
            e.preventDefault();
            // Enter velger det øverste forslaget - den vanligste snarveien.
            if (lastHits.length) goToHit(lastHits[0]);
        } else if (e.key === "Escape") {
            hideSuggestions();
        }
    });

    // Et klikk i forslaget rekker å bli registrert før listen lukkes (blur kommer først).
    searchInput.addEventListener("blur", function () {
        setTimeout(hideSuggestions, 150);
    });

    /* ----- Lagring og lukking ----- */

    function close() {
        window.removeEventListener("resize", sizeCanvas);
        document.removeEventListener("keydown", onKeyDown);
        overlay.remove();
    }

    function onKeyDown(e) {
        if (e.key === "Escape") close();
    }

    // Venter til flisene som er på vei inn har kommet, slik at bildet ikke lagres med hull i kartet.
    // Tidsavbruddet er der for at en flis som aldri kommer ikke skal låse knappen for godt.
    function exportWhenReady(done) {
        const started = Date.now();
        (function wait() {
            if (pendingTiles === 0 || Date.now() - started > 5000) {
                draw({ forExport: true });
                let dataUrl = "";
                try {
                    dataUrl = canvas.toDataURL("image/jpeg", 0.9);
                } catch (err) {
                    // Skjer bare hvis en flis er lastet uten CORS-header (se kommentaren øverst).
                    alert("Klarte ikke å lage bilde av kartet. Ta et skjermbilde av kartet og last det opp i stedet.");
                }
                draw();
                done(dataUrl);
                return;
            }
            setTimeout(wait, 120);
        })();
    }

    const saveBtn = overlay.querySelector(".mapdraw-save");
    saveBtn.addEventListener("click", function () {
        saveBtn.disabled = true;
        saveBtn.innerHTML = '<i class="fa-solid fa-spinner fa-spin"></i> Lagrer ...';
        exportWhenReady(function (dataUrl) {
            if (!dataUrl) {
                saveBtn.disabled = false;
                saveBtn.innerHTML = '<i class="fa-solid fa-check"></i> Bruk som kartutsnitt';
                return;
            }
            close();
            onSave(dataUrl, {
                lat: view.lat,
                lon: view.lon,
                zoom: view.zoom,
                lag: view.lag,
                farge: view.farge,
                punkter: points
            });
        });
    });

    overlay.querySelector(".mapdraw-cancel").addEventListener("click", close);
    overlay.querySelector(".mapdraw-close").addEventListener("click", close);
    overlay.addEventListener("click", function (e) {
        if (e.target === overlay) close();
    });
    document.addEventListener("keydown", onKeyDown);
    window.addEventListener("resize", sizeCanvas);

    sizeCanvas();
    searchInput.focus();
}
