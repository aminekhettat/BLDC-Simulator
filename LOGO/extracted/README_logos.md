# Spinotor — Dossier des logos et icônes

## Fichiers présents

| Fichier | Usage | Description |
|---|---|---|
| `spinotor_logo_v1_violet_plein.svg` | Site web, documentation, présentation | Logo carré fond violet plein, texte SPINOTOR violet sous l'hexagone |
| `spinotor_logo_v2_fond_clair.svg` | Impression, fond blanc, papier | Logo carré fond clair, contour et flèche violets |
| `spinotor_logo_v3_fond_nuit.svg` | README dark mode, slides sombres | Logo carré fond nuit profond, flèche lavande |
| `spinotor_favicon_32x32.svg` | Onglet navigateur, bookmark | Icône carrée 32×32px sans texte |
| `spinotor_icone_app_256x256.svg` | Exécutable Windows/macOS, PyPI | Icône complète 256×256px avec texte |
| `spinotor_badge_pypi_readme.svg` | Badge README GitHub, PyPI | Format horizontal avec icône + nom + tagline |

## Conversions recommandées

Pour obtenir des fichiers .ico (Windows) et .icns (macOS) à partir des SVG :

### Favicon .ico
```bash
# Avec Inkscape
inkscape spinotor_favicon_32x32.svg --export-png=favicon.png --export-width=32
# Puis convertir en .ico avec ImageMagick
convert favicon.png -resize 32x32 favicon.ico
```

### Icône Windows .ico multi-résolution
```bash
inkscape spinotor_icone_app_256x256.svg --export-png=icon_256.png --export-width=256
inkscape spinotor_icone_app_256x256.svg --export-png=icon_128.png --export-width=128
inkscape spinotor_icone_app_256x256.svg --export-png=icon_64.png  --export-width=64
inkscape spinotor_icone_app_256x256.svg --export-png=icon_32.png  --export-width=32
convert icon_256.png icon_128.png icon_64.png icon_32.png spinotor.ico
```

### Icône macOS .icns
```bash
# Créer le dossier iconset
mkdir spinotor.iconset
inkscape spinotor_icone_app_256x256.svg --export-png=spinotor.iconset/icon_256x256.png --export-width=256
inkscape spinotor_icone_app_256x256.svg --export-png=spinotor.iconset/icon_128x128.png --export-width=128
# Générer le .icns (macOS uniquement)
iconutil -c icns spinotor.iconset
```

## Couleurs officielles Spinotor

| Rôle | Hex | Usage |
|---|---|---|
| Violet principal | `#534AB7` | Fond logo, texte principal |
| Violet foncé | `#3C3489` | Contour, ombre |
| Violet nuit | `#26215C` | Fond dark mode |
| Lavande | `#7F77DD` | Contour dark mode |
| Lavande clair | `#AFA9EC` | Éléments sur fond nuit |
| Blanc ivoire | `#EEEDFE` | Flèche et hub sur fond violet |

## Police officielle

Arial / Helvetica (système) — poids 600-700, letter-spacing 4-6px.
Pour une version plus raffinée, utiliser **Inter** (Google Fonts, open source).

---
© 2026 Amine Khettat — Spinotor Non-Commercial Academic License 1.0
