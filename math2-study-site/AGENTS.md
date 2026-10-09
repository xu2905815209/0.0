# AGENTS.md

## Mandatory first reads
1. `CLAUDE.md` — canonical project instructions and editorial admission policy.
2. `PROJECT_STATE.md` — current published lessons and deployment status.

Do not start coding or adding course content until both are read.

## Key principle
**This is an exam-focused curated representative-problem AND concept library, not an automatic archive of every question asked in ChatGPT.**
Most questions should be answered conversationally. Publish a new lesson only when it meets all four admission gates in `CLAUDE.md`: exam relevance, transferability, mathematical accuracy, and pedagogical quality.

## Architecture
- Public catalog: `content/catalog.json`. Every course must have a complete `problemTitle`, `problem`, `problemMath` (MathML), and a linked `points[]` array.
- Homepage has separate “精选题目” and “考点速查” entry points, and each concept links back to its representative question.
- Lesson HTML: `content/kNNN.html`.
- Frontend: `index.html`, `assets/styles.css`, `assets/site.js`.
- Mastery state: localStorage `math2_mastered_v1`, including migration from old q001/q002 keys.
- Production: https://xu2905815209.github.io/0.0/.

Never restore the retired one-off Q003 scoring entry as a published lesson.
