// Static deployment gate: core files exist, curated catalog references valid lessons.
// Run with: node math2-study-site/scripts/validate.mjs
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),"..");
const read=file=>fs.readFileSync(path.join(root,file),"utf8");
const index=read("index.html");
const css=read("assets/styles.css");
const js=read("assets/site.js");
const items=JSON.parse(read("content/catalog.json"));

assert(Array.isArray(items)&&items.length>=1,"Catalog must be a nonempty array");
assert.equal(new Set(items.map(x=>x.id)).size,items.length,"Duplicate topic ID");
assert(index.includes('href="assets/styles.css"')&&index.includes('src="assets/site.js"'),"Asset paths missing from index");
assert(index.includes("数二精华课"),"Expected site identity is absent");
assert(css.length>3000 && js.length>3000,"CSS/JS unexpectedly empty");
assert(!items.some(x=>x.id==="q003"||/考研多少分|这题多少分|估分/i.test(x.title)),"One-off exam-score question must not be published as a course");

for (const t of items) {
  assert(/^k\d{3}$/.test(t.id),"Bad curated topic ID "+t.id);
  assert(t.title&&t.summary&&Array.isArray(t.tags)&&t.tags.length,"Missing metadata for "+t.id);
  assert(!/为什么|这题多少分|考研多少分/.test(t.title),"Course heading resembles a raw question: "+t.title);
  const lesson=read("content/"+t.id+".html");
  assert(lesson.includes('<article class="prose">')&&lesson.includes("</article>"),t.id+": invalid lesson structure");
  assert(lesson.includes('class="quiz"')&&lesson.includes("data-quiz-correct"),t.id+": missing active recall exercise");
  assert(lesson.includes("<math")&&lesson.includes("</math>"),t.id+": missing MathML formulas");
  const count=(str,needle)=>str.split(needle).length-1;
  assert.equal(count(lesson,"<math"),count(lesson,"</math>"),t.id+": unbalanced MathML");
  assert(!/这道连乘、n 次根号极限，考研数二算几分/.test(lesson),t.id+": old one-off question leaked into lesson");
}

console.log("Validated "+items.length+" curated lessons; catalog, MathML, quizzes, asset links and editorial exclusion passed.");
