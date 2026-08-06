/* js/menu.js */

document.addEventListener("DOMContentLoaded", function() {
    const menuItems = [
        { name: "Hjem", link: "index.html", icon: "fa-house" },
        {
            name: "Trening", icon: "fa-graduation-cap", children: [
                { name: "UAS Intro", link: "uas-intro.html", icon: "fa-list-check" },
                { name: "Leksjonsskjema", link: "leksjonsskjema.html", icon: "fa-clipboard-user" },
                {
                    name: "Simulator", icon: "fa-gamepad", children: [
                        { name: "Quadcopter", link: "simulator.html", icon: "fa-helicopter" },
                        { name: "Fixed-wing", link: "simulator-fixedwing.html", icon: "fa-plane" },
                        { name: "VTOL", link: "simulator-vtol.html", icon: "fa-plane-up" }
                    ]
                }
            ]
        },
        {
            name: "Risikovurdering", icon: "fa-triangle-exclamation", children: [
                { name: "Oversikt", link: "risikovurdering.html", icon: "fa-house" },
                { name: "Sjekkliste-bygger", link: "sjekkliste-bygger.html", icon: "fa-list-check" }
            ]
        }
        // Legg til flere sider her etter hvert, f.eks.:
        // { name: "Om", link: "about.html", icon: "fa-circle-info" }
    ];

    const path = window.location.pathname;
    const page = path.split("/").pop() || "index.html";

    // En undermeny-oppføring kan selv ha "children" (f.eks. Simulator under Trening) - da rendres den
    // som en egen nestet dropdown-knapp i stedet for en ren lenke.
    function isDescendantActive(children) {
        return children.some(child => child.link === page || (child.children && isDescendantActive(child.children)));
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
        const isActive = (page === child.link) ? ' active' : '';
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
        const isActive = (page === item.link) ? 'class="active"' : '';
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
            document.querySelectorAll(".dropdown-menu-nested.open").forEach(function (m) { m.classList.remove("open"); });
            if (!isOpen) menu.classList.add("open");
        });
    });

    document.addEventListener("click", function () {
        document.querySelectorAll(".dropdown-menu.open").forEach(function (m) { m.classList.remove("open"); });
    });
});
