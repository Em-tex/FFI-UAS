# FFI-UAS

Nettside for FFI UAS-prosjektet.

## Oppsett

Statisk nettside i vanilla HTML/CSS/JS - ingen byggverktøy eller avhengigheter kreves.

```
FFI-UAS/
├── css/
│   └── style.css      # Felles styling (FFI-farger, layout, komponenter)
├── js/
│   └── menu.js         # Bygger navigasjonsmenyen dynamisk på alle sider
└── index.html           # Startside
```

For å legge til en ny side:

1. Opprett en ny `.html`-fil i rotmappen, basert på `index.html`.
2. Legg til siden i `menuItems`-listen i [js/menu.js](js/menu.js) slik at den dukker opp i navigasjonen.

## Kjøre lokalt

Åpne `index.html` direkte i nettleseren, eller server mappen med f.eks.:

```
npx serve .
```
