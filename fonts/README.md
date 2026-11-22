# Self-hosted fonts

Latin subsets of the three families used by `index.html`, downloaded from
Google Fonts on 2026-11-22. They are served from this directory instead of
`fonts.googleapis.com`, which removes two third-party origins and a
render-blocking stylesheet from the page's critical path.

| File | Family | Variable axes | Google Fonts source |
|---|---|---|---|
| `bricolage-grotesque-latin.woff2` | Bricolage Grotesque | `opsz 12..96`, `wght 500..700` | `Bricolage+Grotesque:opsz,wght@12..96,500..700` |
| `instrument-sans-latin.woff2` | Instrument Sans | `wght 400..600` | `Instrument+Sans:wght@400..600` |
| `jetbrains-mono-latin.woff2` | JetBrains Mono | `wght 400..500` | `JetBrains+Mono:wght@400..500` |

To refresh, request the CSS from
`https://fonts.googleapis.com/css2?family=...&display=swap` with a browser
user agent, copy the `latin` `src` URL for each family, and download it. Keep
the `@font-face` weight ranges in `index.html` in sync with the axes above.

All three families are licensed under the SIL Open Font License 1.1; see
`OFL.txt` for the copyright notices and the full license text.
