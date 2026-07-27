/* js/menu.js */

document.addEventListener("DOMContentLoaded", function() {
    const menuItems = [
        { name: "Hjem", link: "index.html", icon: "fa-house" },
        {
            name: "Trening", icon: "fa-graduation-cap", children: [
                { name: "UAS Intro", link: "uas-intro.html", icon: "fa-list-check" },
                { name: "Leksjonsskjema", link: "leksjonsskjema.html", icon: "fa-clipboard-user" }
            ]
        },
        {
            name: "Risikovurdering", icon: "fa-triangle-exclamation", children: [
                { name: "Oversikt", link: "risikovurdering.html", icon: "fa-house" }
            ]
        }
        // Legg til flere sider her etter hvert, f.eks.:
        // { name: "Om", link: "about.html", icon: "fa-circle-info" }
    ];

    const path = window.location.pathname;
    const page = path.split("/").pop() || "index.html";

    function renderItem(item) {
        if (item.children) {
            const childActive = item.children.some(child => child.link === page);
            let html = `<li class="nav-dropdown${childActive ? ' active' : ''}">
                <button type="button" class="dropdown-toggle">
                    <i class="fa-solid ${item.icon}"></i> ${item.name}
                    <i class="fa-solid fa-chevron-down" style="font-size:0.65rem;"></i>
                </button>
                <div class="dropdown-menu">`;
            item.children.forEach(child => {
                const isActive = (page === child.link) ? ' active' : '';
                html += `<a href="${child.link}" class="${isActive.trim()}"><i class="fa-solid ${child.icon}"></i> ${child.name}</a>`;
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

    document.addEventListener("click", function () {
        document.querySelectorAll(".dropdown-menu.open").forEach(function (m) { m.classList.remove("open"); });
    });
});
