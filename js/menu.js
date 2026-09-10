/* js/menu.js */

document.addEventListener("DOMContentLoaded", function() {
    const menuItems = [
        { name: "Hjem", link: "index.html", icon: "fa-house" },
        {
            // "UAS Intro" og "Leksjonsskjema" var to flate lenker rett under Trening - slått sammen i en
            // egen nestet "Skjema"-undermeny (brukerens krav), samme mønster som Simulator-undermenyen
            // rett under (renderDropdownChild/renderItem håndterer nøstede "children" generisk allerede).
            name: "Trening", icon: "fa-graduation-cap", children: [
                {
                    name: "Skjema", icon: "fa-clipboard-list", children: [
                        { name: "UAS Intro", link: "uas-intro.html", icon: "fa-list-check" },
                        { name: "Leksjonsskjema", link: "leksjonsskjema.html", icon: "fa-clipboard-user" }
                    ]
                },
                {
                    name: "Simulator", icon: "fa-gamepad", children: [
                        {
                            // Quadcopter fikk et nivå til (brukerønske): selve simulatoren, pluss egne
                            // lenker rett inn i hver øvelseskategori. Lenkene bruker ?exercises=<kategori>
                            // - se dyplenke-håndteringen nederst i js/simulator.js, som godtar både
                            // program ("initial"/"recurrent") og kategorinøkkel.
                            name: "Quadcopter", icon: "fa-helicopter", children: [
                                { name: "Åpne simulator", link: "simulator.html", icon: "fa-play" },
                                { name: "Quad intro", link: "simulator.html?exercises=initialQuad", icon: "fa-graduation-cap" },
                                { name: "Acro intro", link: "simulator.html?exercises=initialAcro", icon: "fa-bolt" },
                                { name: "Recurrent quad", link: "simulator.html?exercises=recurrentQuad", icon: "fa-rotate" },
                                { name: "Recurrent acro", link: "simulator.html?exercises=recurrentAcro", icon: "fa-rotate" }
                            ]
                        },
                        { name: "Fixed-wing", link: "simulator-fixedwing.html", icon: "fa-plane" },
                        { name: "VTOL", link: "simulator-vtol.html", icon: "fa-plane-up" }
                    ]
                }
            ]
        },
        {
            // Het tidligere "Risikovurdering" - omdøpt til "Godkjenning for å fly" (brukerønske: mer
            // beskrivende for hva verktøyene under faktisk brukes til). Selve filnavnet
            // risikovurdering.html er bevisst IKKE endret - eventuelle bokmerker og delte lenker til
            // oversikten skal fortsatt virke.
            name: "Godkjenning for å fly", icon: "fa-circle-check", children: [
                { name: "Oversikt", link: "risikovurdering.html", icon: "fa-house" },
                { name: "Sjekkliste-bygger", link: "sjekkliste-bygger.html", icon: "fa-list-check" },
                { name: "Vurdering av godkjenninger", link: "godkjenningsvurdering.html", icon: "fa-clipboard-check" }
            ]
        }
        // Legg til flere sider her etter hvert, f.eks.:
        // { name: "Om", link: "about.html", icon: "fa-circle-info" }
    ];

    const path = window.location.pathname;
    const page = path.split("/").pop() || "index.html";
    // Full adresse inkludert spørrestreng - lenkene til øvelseskategoriene skiller seg KUN på den
    // (simulator.html?exercises=initialQuad osv.), så uten den ville alle fem Quadcopter-lenkene blitt
    // markert aktive samtidig.
    const pageWithQuery = page + window.location.search;
    // Filnavnet i en lenke, uten spørrestreng - brukes til å markere FORELDRE-menyene som aktive.
    function linkPage(link) { return (link || "").split("?")[0]; }

    // En undermeny-oppføring kan selv ha "children" (f.eks. Simulator under Trening) - da rendres den
    // som en egen nestet dropdown-knapp i stedet for en ren lenke.
    function isDescendantActive(children) {
        return children.some(child => linkPage(child.link) === page || (child.children && isDescendantActive(child.children)));
    }

    function renderDropdownChild(child) {
        if (child.children) {
            const childActive = isDescendantActive(child.children);
            let html = `<div class="dropdown-submenu${childActive ? ' active' : ''}">
                <button type="button" class="dropdown-subtoggle">
                    <i class="fa-solid ${child.icon}"></i> ${child.name}
                    <i class="fa-solid fa-chevron-right" style="font-size:0.6rem; margin-left:auto;"></i>
                </button>
                <div class="dropdown-menu dropdown-menu-nested">`;
            child.children.forEach(grandchild => {
                html += renderDropdownChild(grandchild);
            });
            html += `</div></div>`;
            return html;
        }
        const isActive = (pageWithQuery === child.link) ? ' active' : '';
        return `<a href="${child.link}" class="${isActive.trim()}"><i class="fa-solid ${child.icon}"></i> ${child.name}</a>`;
    }

    function renderItem(item) {
        if (item.children) {
            const childActive = isDescendantActive(item.children);
            let html = `<li class="nav-dropdown${childActive ? ' active' : ''}">
                <button type="button" class="dropdown-toggle">
                    <i class="fa-solid ${item.icon}"></i> ${item.name}
                    <i class="fa-solid fa-chevron-down" style="font-size:0.65rem;"></i>
                </button>
                <div class="dropdown-menu">`;
            item.children.forEach(child => {
                html += renderDropdownChild(child);
            });
            html += `</div></li>`;
            return html;
        }
        const isActive = (pageWithQuery === item.link) ? 'class="active"' : '';
        const target = item.target ? `target="${item.target}"` : '';
        return `<li><a href="${item.link}" ${isActive} ${target}><i class="fa-solid ${item.icon}"></i> ${item.name}</a></li>`;
    }

    let menuHtml = `
    <nav>
        <div class="brand">
            FFI UAS
        </div>
        <ul>`;

    menuItems.forEach(item => {
        menuHtml += renderItem(item);
    });

    menuHtml += `
        </ul>
        <div></div>
    </nav>`;

    document.body.insertAdjacentHTML("afterbegin", menuHtml);

    document.querySelectorAll(".nav-dropdown > .dropdown-toggle").forEach(function (btn) {
        btn.addEventListener("click", function (e) {
            e.stopPropagation();
            const menu = btn.nextElementSibling;
            const isOpen = menu.classList.contains("open");
            document.querySelectorAll(".dropdown-menu.open").forEach(function (m) { m.classList.remove("open"); });
            if (!isOpen) menu.classList.add("open");
        });
    });

    // Nestet undermeny (f.eks. Simulator inni Trening) - egen håndtering som IKKE lukker foreldre-
    // dropdownen den ligger inni (stopPropagation), kun sin egen nestede meny.
    document.querySelectorAll(".dropdown-subtoggle").forEach(function (btn) {
        btn.addEventListener("click", function (e) {
            e.stopPropagation();
            const menu = btn.nextElementSibling;
            const isOpen = menu.classList.contains("open");
            document.querySelectorAll(".dropdown-menu-nested.open").forEach(function (m) {
                // Lukk kun menyer som IKKE inneholder knappen vi nettopp klikket. Med tre nivåer
                // (Quadcopter inni Simulator inni Trening) er foreldremenyen selv en
                // .dropdown-menu-nested - uten denne testen lukket et klikk på tredje nivå sin egen
                // foreldremeny, og undermenyen ble åpnet inni noe usynlig.
                if (!m.contains(btn)) m.classList.remove("open");
            });
            if (!isOpen) menu.classList.add("open");
        });
    });

    document.addEventListener("click", function () {
        document.querySelectorAll(".dropdown-menu.open").forEach(function (m) { m.classList.remove("open"); });
    });
});
