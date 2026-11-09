# BUG-10: Command Deck Button Typography & Emoji Alignment Fix

> **Category**: 🐛 Bug | **Priority**: Normal | **State**: todo  
> **Operational Inspiration**: Emoji characters in GCS command deck buttons cause vertical misalignment and line-height glitches

---

## 1. Operational Overview & Objectives

In `GCS/frontend/index.html`, several command deck and auxiliary mission action buttons contain multi-byte Unicode emoji glyphs (e.g., `⚡ PROBE T&G`, `🔍 RECALL MULE`, `📦 DROP REPEATER`, `🚨 E-LAND`).

**The Bug**:
- Under standard Windows browser rendering (Chrome/Edge), multi-byte emojis force fallback to the *Segoe UI Emoji* font family.
- *Segoe UI Emoji* has radically different font metrics (bounding box, ascender, descender, line-height) compared to the tactical HUD's monospace fonts (`'JetBrains Mono'`, `'Share Tech Mono'`).
- This causes button text to be vertically misaligned, forces uneven button heights across the grid, and introduces awkward padding artifacts that break the crisp cyberpunk aesthetic.

**Objective**:
- Standardize command deck button typography.
- Either replace emojis with sleek text tags/Unicode symbols (e.g. `[T&G]`, `[RECALL]`, `[DROP]`, `[E-LAND]`, or standard geometric glyphs `▶`, `▲`, `◆`) or enclose icons in dedicated flex spans with explicit sizing and baseline alignment.
- Apply CSS flexbox vertical alignment (`display: inline-flex; align-items: center; justify-content: center; gap: 6px; line-height: 1;`) to ensure all buttons maintain uniform height and perfect vertical centering.

---

## 2. Target Files & Architectural Scope

* `GCS/frontend/index.html`: Review all button markup in the bottom control deck, aux mission panel, and recovery deck.
* `GCS/frontend/css/style.css`: Ensure `.btn`, `.btn-tactical`, and deck button classes enforce flex centering, consistent heights, and monospace typography.

---

## 3. Implementation Blueprint

1. **HTML Button Markup**:
   - Refactor buttons to use consistent tactical syntax or icon containers:
     ```html
     <button id="btn-touch-and-go" class="tactical-btn"><span class="btn-glyph">⚡</span> PROBE T&G</button>
     ```
     Or replace with tactical tag prefixes like `[PROBE]`, `[MULE]`, `[DROP]`, `[E-LAND]`.
2. **CSS Normalization**:
   - In `style.css`:
     ```css
     .tactical-btn, .cmd-btn {
         display: inline-flex;
         align-items: center;
         justify-content: center;
         vertical-align: middle;
         line-height: 1;
         font-family: 'JetBrains Mono', 'Share Tech Mono', monospace;
         height: 32px;
         box-sizing: border-box;
     }
     .btn-glyph {
         display: inline-block;
         line-height: 1;
         font-size: 1.1em;
     }
     ```

---

## 4. Acceptance Criteria & Invariants

* [x] Command deck buttons maintain uniform vertical alignment and line-height across all rows.
* [x] No font-metric jumping or uneven padding caused by emoji fallback.
* [x] Preserves cyberpunk aesthetic and clean monospace HUD readability.
