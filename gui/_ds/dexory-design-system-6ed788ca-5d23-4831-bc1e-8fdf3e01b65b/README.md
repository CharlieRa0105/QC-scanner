# Dexory Design System

A design system for **DexoryView** — the commercial evidence layer for warehouse robotics by Dexory. Use this as the source of truth for colors, typography, components, copywriting, and UI kits when producing designs, mockups, slides or throwaway prototypes for Dexory.

## Product context

**Dexory** builds autonomous mobile robots that scan warehouses. **DexoryView** is the software layer customers spend their time in — the web platform where a day's scan becomes exceptions to work, evidence to show, and trends over time.

### Users
- **Operational** — warehouse operatives, supervisors, area managers. On the floor, mobile, time-pressured. Need one clear action, fast.
- **Analytical** — inventory controllers, process improvement leads, compliance specialists. In the office, in the data, daily. Need depth, filters, export.
- **Strategic** — operations managers, plant managers, 3PL account managers, execs. In the boardroom. Need clarity and evidence they can present.
- **External** — clients, auditors, compliance bodies receiving reports *from* DexoryView. Need structured, trustworthy, professional output.

### Product pillars
- **Reduce manual effort** — pull Excel / PowerBI / VLOOKUP workflows back inside the platform.
- **Make compliance effortless** — cycle-count coverage, scan rates, location correctness, task resolution evidenced without extraction.
- **Surface performance over time** — not just today's snapshot, but trend and ownership.
- **Scale from floor to boardroom** — same underlying data, adapted presentation.
- **Prove and grow the value of Dexory** — DexoryView is the commercial evidence layer.

## Sources

- **Figma file:** *Dexory Design System [V2]* — mounted as a virtual filesystem for this project. Foundation pages (`/Foundation/Colors`, `.../Header`, `.../Body`, `.../Label`, `.../Spacing-System`, `.../Elevation`, `.../Icons`) and all `/Core-components/*` were the primary sources.
- **Additional notes:** Product-strategy brief (user cohorts, pillars, tone guidelines) shared alongside the Figma file — reproduced above and in `CONTENT-FUNDAMENTALS` below.

The pseudocode JSX under `/Foundation` and `/Core-components` was read directly as the design source of truth. SVG icon sprites were copied rather than redrawn.

## Index

| File | What it is |
|---|---|
| `README.md` | This file — context, content + visual foundations, iconography, index |
| `SKILL.md` | Agent-skill manifest — how to invoke this system as a design skill |
| `colors_and_type.css` | All design tokens as CSS custom properties + typography classes |
| `assets/` | Logos (wordmark + logomark) and key SVG icons |
| `preview/` | Small HTML cards that populate the Design System review tab |
| `ui_kits/dexoryview/` | React-JSX recreation of the DexoryView app — sidebar, topbar, data table, badges, modals, full sample screens |

---

## Content fundamentals

**Voice: clear, confident, human, contextual, proactive.** DexoryView tells busy warehouse users what's wrong and what to do about it. It never makes them interpret or guess.

### Tone rules
- **Clear over clever.** If a location has an error, say "error". If a task is overdue, say "overdue". No jargon, no marketing language inside the product.
- **Confidence-inspiring.** Users present this data to clients and auditors. The UI must never feel hedged or uncertain. Phrase findings as facts, not opinions.
- **Contextual, not overwhelming.** A supervisor needs one next step; an inventory controller needs the full dataset. Surface the right detail for the surface — don't dump everything on everyone.
- **Proactive, not reactive.** Flag high-priority tasks, approaching cycle-count deadlines, ready exports — before the user goes looking.
- **Human and respectful.** Performance data names people. Frame it as coaching, never surveillance. Neutral, factual, non-punitive.

### Casing, person, grammar
- **Sentence case everywhere** — page titles, buttons, tabs, menu items. No Title Case. "Create task", not "Create Task".
- **"Cycle count", "task", "location", "scan rate"** — domain nouns stay lowercase unless they start a sentence.
- **Second person** where we address the user: "You have 3 tasks overdue." First person is avoided.
- **Imperative for actions:** "Resolve", "Assign", "Export to CSV", "Mark as complete".
- **No exclamation marks.** Positive outcomes are stated plainly: "Task resolved.", not "Task resolved!".
- **Numerals for all quantities** — "3 tasks", "14 days", "98.2%". Spell out zero only in prose ("No errors found").
- **UK English** — "Colour" lives in the platform copy but we keep CSS token names US-spelled (`color-*`) to match the code.

### What it sounds like
Good examples drawn from the system:
- Section header: "📦 Main components [Published]" — internal working copy; production UI strips the emoji.
- Status badge copy: **Error label / Warning label / Success label** — short, literal, never "Oops!" or "All good!".
- Empty state: state what's missing and what the user can do next. No mascots.
- Confirmation: "Task resolved." → muted success toast, auto-dismiss.

### Emoji
Not used in production UI copy or end-user strings. The Figma source uses working-doc markers like 📦 🟢 on section headings — these are **doc hygiene only**, do not carry into product or collateral. The only icon-like characters that ship are literal warehouse symbols if needed, and even those should use the SVG icon set.

---

## Visual foundations

Dexory's visual language is **clean, evidentiary, operational**. Think: a well-lit warehouse control room — lots of white, firm black type, a single high-voltage accent (Dexory Lime), and feedback colors that are loud enough to spot across a floor.

### Color

- **Monochrome foundation.** `#FFFFFF` background, `#000000` primary text, a 9-step **Charcoal** ramp from 50 → 900 for surfaces, dividers and secondary text. Nothing brand-adjacent is blue-gray or slate — it's all neutral Charcoal.
- **One accent: Dexory Lime** (`#C3EF00` / lime-500). Used sparingly — the selected nav dot, the robot avatar, moments where the product wants to feel alive. Hover is **lime-600** `#ABC900`. On dark surfaces, the lime reads almost neon. Never use it for destructive or warning states.
- **Secondary accent: Blue** (`#5631EA` / blue-500, `#111F4C` dark-blue-900 for deep surfaces). Used for informational highlights, links, data-viz primary series.
- **Semantic feedback** is non-negotiable and literal:
  - **Error** — red-500 `#DA1E28` on red-50 `#FFF1F1` tint
  - **Warning** — amber-900 `#7B3306` on amber-50 `#FEF3C6` tint
  - **Success** — green-900 `#125D1A` on green-50 `#E7F5E8` tint
  - **Info / neutral** — gray-800 `#2F2F2D` on gray-50 `#F6F7F4` tint
- **Dark backgrounds** — **Charcoal 900** `#101727` (the app menu) and **Dark Blue 900** `#111F4C` (marketing / hero). Lime pops against both.

### Typography

- **Inter** for everything on-screen — Regular, Medium (default), Semi Bold, Bold. Letter-spacing `-0.02em` on display sizes (30px+).
- **Roboto Mono** for numeric data in tables, timestamps, IDs — Regular / Medium / Bold.
- **Scale:** Header Large 32/40, Header Small 20/30; Body Medium 16/24, Body Small 14/20; Label Medium 14/20, Small 14/20, Extra-small 12/16. All from the Figma pages.
- **Weight discipline.** Medium (500) is default body. Semibold (600) earns emphasis. Bold (700) is reserved for display and table headers.

### Spacing & layout

- **8-point grid.** Tokens: `xs 8` · `sm 16` · `md 24` · `lg 40` · `xl 64` · `xxl 80`.
- **Screen padding:** 80px horizontal on the foundation/reference frames; 24–32px inside app cards.
- **Cards** are flat rectangles: `background: white`, `border: 1px solid charcoal-100`, **no border-radius on cards** in the app body, **4px radius on artwork / badges**, **5px dashed purple** `#8A38F5` for in-Figma debug outlines only (do not ship).
- **Dividers** are a single line `rgb(236,237,234)` — charcoal-100 — stretched edge-to-edge, not inset.

### Elevation

Two levels, no more:
- **Medium** — `box-shadow: 0 4px 16px rgba(0,0,0,0.12)`. Dropdowns, date pickers, floating buttons, location drawers.
- **High** — `box-shadow: 0 8px 32px rgba(0,0,0,0.16)`. Modals, dialogs.
Plus two near-zero shadows for subtle lift: `rgba(16,24,40,0.05)` and `rgba(16,24,40,0.06)` used on inputs and popovers.

### Corners & borders

- **Buttons**: 6px radius.
- **Inputs**: 6px radius.
- **Badges / chips**: 4px radius.
- **Cards**: 0 (square) — the system trusts flat rectangles over rounding.
- **Borders**: 1px solid gray-100 `#ECEDEA` or charcoal-100 `#ECEDEA` as the default; 1px solid black for emphasis containers.

### Motion & states

- **No bounces, no spring.** Transitions are short and utilitarian — 150ms ease-out on hover, 200ms on modal enter/exit.
- **Hover** darkens by one step: white → gray-50, blue-500 → blue-600, lime-500 → lime-600, black → gray-800. Never an opacity fade.
- **Pressed** drops one further step; no scale transform.
- **Focus** — 2px outline in blue-500 `#5631EA`, 2px offset.
- **Disabled** — 40% opacity, no color shift.
- **Loading** — muted, inline spinners; never full-screen overlays unless blocking.

### Backgrounds, imagery, texture

- **Predominantly plain white.** The application chrome is white; the menu is charcoal-900; hero marketing surfaces are dark-blue-900.
- **No gradients** inside the product. The logomark has a subtle vertical gradient — everything else is flat fill.
- **Photography** — real warehouse imagery (pallet aisles, robots on the floor). Warm daylight tones, un-stylised. No duotones, no heavy grain.
- **No illustrations** in the system. Empty states use icons + copy, not character art.
- **No patterns, textures, noise.** (The Figma file has a `NOISE` effect on one exploration — it is not part of the shipped system.)

### Transparency & blur

Used sparingly. Modal backdrops `rgba(16,24,40,0.5)` with no blur. Badge tints are solid colors from the 50-step ramp, not alpha'd versions of the 500-step. Blur is not part of the production aesthetic.

### Data visualization

- Primary series: **blue-500**. Positive delta: **green-700**. Negative delta: **red-500**. Neutral: **charcoal-400**.
- Grid lines: charcoal-100, 1px, dashed optional.
- Axis labels: 12px Inter Medium, charcoal-600.
- Numeric labels on marks: **Roboto Mono** 12/16.

---

## Iconography

- **Custom SVG set** drawn in-house, 24×24 and 16×16 viewboxes, **2px outline stroke** on outlined icons, `fill="currentColor"` so they inherit color from the parent. Filled variants exist for selected states (sidebar etc).
- The Figma file's `/Foundation/Icons` page is the authoritative source. Key icons copied into `assets/icons/` include: **check-circle**, **alert-triangle**, **error / x-circle**, **slash**, **info-circle**, **help-circle**, **chevron-down/up/left/right**, **chevron-selector-vertical**, **search**, **plus**, **minus**, **arrow-up/down**, **calendar**, **eye**, **refresh**, **dots-vertical**, **more-vertical**, **settings**, **file**, **mail**, **message-square**, **atom** (data/AI), **cube** (inventory), **scan**, **star**, **lightning**, **thumbs-up**, **zoom-in**, **hourglass**, **radio-button-checked/unchecked**, **check**.
- **Stroke weight fidelity** — all outlined icons use 2px. Do not mix in hairline or 1.5px sets.
- **No emoji in production UI**. The Figma source uses emoji as document markers (📦 🟢 ⚫) — strip them when recreating real surfaces.
- **No icon fonts** are used; everything is inline SVG. When an icon is missing from `assets/icons/`, substitute with the closest **Lucide** icon (CDN) — the Dexory set is Lucide-compatible in stroke style and geometry — and flag the substitution.
- **Brand marks:** `assets/logo-wordmark.svg` (horizontal Dexory mark) and `assets/logomark.svg` (square logomark, for app favicons & tight spaces). Do not recolor the wordmark outside of black / white.

---

## UI kits

- **`ui_kits/dexoryview/`** — the core warehouse management app: sidebar nav, topbar, data tables, badges, tabs, modal, and a sample "Tasks" and "Cycle counts" screen.

## Caveats & substitutions

- **Inter:** brand-supplied variable font files are loaded via `@font-face` from `/fonts/Inter-VariableFont_opsz_wght__1_.ttf` (upright) and `/fonts/Inter-Italic-VariableFont_opsz_wght.ttf` (italic). The variable axis covers weights 100–900, so the 40+ static `.ttf` files also in `/fonts/` are redundant but kept for parity with the upload.
- **Roboto Mono:** still loaded from Google Fonts — no brand copy was supplied. Drop a licensed file into `/fonts/` and swap the `@import` for a `@font-face` stanza if needed.
- **Icons:** copied as SVGs where available; marketing-specific icons fall back to Lucide from CDN.
- **Imagery:** no real photography was included in the Figma export; marketing/hero screens reserve space for it with neutral placeholders.
