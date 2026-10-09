/**
 * 考研数学二精选课程站
 * 内容：content/catalog.json + content/kNNN.html
 * 规则：先读 CLAUDE.md 与 PROJECT_STATE.md；网站只收录经筛选的代表题及对应核心考点。
 */
(() => {
  "use strict";
  const $ = (sel, scope=document) => scope.querySelector(sel);
  const $$ = (sel, scope=document) => [...scope.querySelectorAll(sel)];
  const storageKey = "math2_mastered_v1"; // 兼容旧站用户进度
  let catalog = [], mastery = {}, onlyPending = false, lastReview = -1, pendingLesson = 0;

  const reviewQuestions = [
    {
      topic: "导数图像与原函数增量",
      question: "题目给出导数图像、围成的面积和一个函数值，最先考虑什么？",
      options: ["先猜原函数的解析式", "把导数积分，求原函数增量", "先用洛必达"],
      answer: 1,
      explain: "“导数图像 + 面积 + 一个函数值”是牛顿—莱布尼茨公式的典型信号。"
    },
    {
      topic: "分式函数的 n 阶导数",
      question: "求分母含 x(1−x) 的分式函数的 n 阶导数，第一步通常怎么做？",
      options: ["拆成两个简单分式后分别求导", "连续使用商法则 n 次", "直接用麦克劳林展开"],
      answer: 0,
      explain: "先部分分式分解，把复杂函数变成两个简单负幂模板。"
    },
    {
      topic: "导数图像与原函数增量",
      question: "导数图像位于横轴下方，且该段几何面积为 3，那么函数增量是多少？",
      options: ["+3", "−3", "无法判断"],
      answer: 1,
      explain: "定积分是带符号面积：横轴下方必须记负号，所以函数值减少 3。"
    },
    {
      topic: "分式函数的 n 阶导数",
      question: "为什么每次对 (1−x) 的负幂求导，结果的符号都是正的？",
      options: ["高阶导永远是正的", "外层负幂和内层导数两个负号抵消", "因为泰勒展开系数是正的"],
      answer: 1,
      explain: "每次使用链式法则，外层给一个负号，内层 (1−x) 的导数又是 −1，因此负负得正。"
    }
  ];

  function getMastery() {
    try {
      const value = JSON.parse(localStorage.getItem(storageKey)||"{}");
      if (value && typeof value==="object") {
        if ("q001" in value && !("k001" in value)) value.k001=!!value.q001;
        if ("q002" in value && !("k002" in value)) value.k002=!!value.q002;
        return value;
      }
    } catch (e) { /* 无法存储时仍能阅读网站 */ }
    return {};
  }
  function saveMastery() {
    try {localStorage.setItem(storageKey, JSON.stringify(mastery));}
    catch (e) { /* Safari 隐私模式可能限制存储 */ }
  }
  function safe(s) { return String(s).replace(/[&<>"']/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c])); }

  function updateProgress() {
    const total=catalog.length, done=catalog.filter(item=>!!mastery[item.id]).length;
    $("#total-count").textContent=total;
    $("#done-count").textContent=done;
    $("#review-count").textContent=total-done;
    const percent=total?Math.round(100*done/total):0;
    $("#progress-label").textContent=percent+"%";
    $("#progress-fill").style.width=percent+"%";
  }

  function renderCatalog() {
    const query=($("#search").value||"").trim().toLowerCase();
    const list=catalog.filter(item=>{
      const pointNames=(item.points||[]).map(point=>point.name+" "+point.hint).join(" ");
      const terms=[
        item.title,item.problemTitle,item.problem,item.summary,item.chapter,
        ...item.tags,...(item.keywords||[]),pointNames
      ].join(" ").toLowerCase();
      return (!query || terms.includes(query)) && (!onlyPending || !mastery[item.id]);
    });
    $("#empty-state").hidden=list.length>0;
    $("#catalog").innerHTML=list.map(item=>{
      const done=!!mastery[item.id];
      const chips=(item.points||[]).map(point=>"<span>"+safe(point.name)+"</span>").join("");
      const math=(item.problemMath||"");
      return '<article class="card" data-topic="'+safe(item.id)+'">'+
        '<div class="card-meta"><span class="chapter">'+safe(item.chapter)+'</span><span>'+safe(item.number)+'</span></div>'+
        '<h3>'+safe(item.problemTitle||item.title)+'</h3>'+
        '<p>典型题型 · '+safe(item.title)+'</p>'+
        '<div class="card-problem">'+
          '<div class="problem-badge">REPRESENTATIVE PROBLEM · 代表题</div>'+
          '<p class="card-problem-desc">'+safe(item.problem)+'</p>'+
          (math?'<div class="card-math">'+math+'</div>':"")+
        '</div>'+
        '<div class="card-concepts"><h4>本题对应考点</h4>'+chips+'</div>'+
        '<div class="card-bottom">'+
        '<button type="button" class="btn primary" data-open="'+safe(item.id)+'">先做题 · 看解析 →</button>'+
        '<button type="button" class="btn status" data-done="'+done+'" data-toggle="'+safe(item.id)+'">'+(done?"✓ 已掌握":"○ 未掌握")+'</button>'+
        '</div></article>';
    }).join("");
  }

  function renderPoints() {
    const index=new Map();
    catalog.forEach(item=>{
      (item.points||[]).forEach(p=>{
        const name=typeof p==="string"?p:p.name;
        const hint=typeof p==="string"?"":p.hint;
        if(!index.has(name))index.set(name,{name,hint,topics:[]});
        index.get(name).topics.push(item);
      });
    });
    $("#point-index").innerHTML=[...index.values()].map(p=>{
      const item=p.topics[0];
      const ids=p.topics.map(t=>t.number).join(" · ");
      const done=!!mastery[item.id];
      return '<button type="button" class="point-card" data-open="'+safe(item.id)+'">'+
        '<span class="point-kicker">核心考点 '+(done?' · 所属题已掌握':'')+'</span>'+
        '<strong>'+safe(p.name)+'</strong>'+
        '<span class="point-hint">'+safe(p.hint)+'</span>'+
        '<span class="point-link">对应：'+safe(item.problemTitle||item.title)+' ↗</span>'+
        '</button>';
    }).join("");
  }

  function toggleMastery(id) {
    if (!catalog.some(x=>x.id===id)) return;
    mastery[id]=!mastery[id];saveMastery();updateProgress();renderCatalog();renderPoints();
    const active=idFromHash();
    if(active===id) updateLessonButton(id);
  }

  function updateLessonButton(id) {
    const b=$("#lesson-mark");
    const done=!!mastery[id];
    b.textContent=done?"✓ 已掌握 · 点击取消":"标记已掌握";
    b.classList.toggle("status",done);
    b.dataset.done=String(done);
  }

  function idFromHash() {
    const hash=decodeURIComponent(location.hash.slice(1)).toLowerCase();
    const aliases={"q001":"k001","q002":"k002"};
    return aliases[hash]||hash;
  }

  function goHome(section) {
    if(location.hash) history.pushState(null,"",location.pathname+location.search);
    showHome(section);
  }
  function showHome(section) {
    pendingLesson++;
    $("#home-view").hidden=false;
    $("#lesson-view").hidden=true;
    $("#nav-courses").classList.toggle("active",section!=="review"&&section!=="points");
    $("#nav-points").classList.toggle("active",section==="points");
    $("#nav-review").classList.toggle("active",section==="review");
    if(section==="review") {
      setTimeout(()=>$("#review-section").scrollIntoView({behavior:"smooth",block:"start"}),30);
    } else if (section==="catalog") {
      setTimeout(()=>$("#catalog-section").scrollIntoView({behavior:"smooth",block:"start"}),30);
    } else if(section==="points") {
      setTimeout(()=>$("#points-section").scrollIntoView({behavior:"smooth",block:"start"}),30);
    } else {
      window.scrollTo({top:0,behavior:"smooth"});
    }
  }
  function openLesson(id) {
    if(!catalog.some(item=>item.id===id)) return;
    location.hash=id;
    if(idFromHash()===id) renderRoute();
  }

  async function showLesson(item) {
    $("#home-view").hidden=true;
    $("#lesson-view").hidden=false;
    $("#nav-courses").classList.remove("active");
    $("#nav-points").classList.remove("active");
    $("#nav-review").classList.remove("active");
    $("#lesson-number").textContent=item.number;
    $("#lesson-title").textContent=item.title;
    $("#lesson-summary").textContent=item.summary;
    $("#lesson-tags").innerHTML=item.tags.map(x=>"<span>"+safe(x)+"</span>").join("");
    $("#lesson-mark").dataset.lessonId=item.id;
    updateLessonButton(item.id);
    $("#lesson-content").innerHTML="<p>正在读取完整讲解……</p>";
    window.scrollTo({top:0,behavior:"smooth"});

    const ticket=++pendingLesson;
    try {
      // Absolute URLs resolved from the site root survive query strings and hash routes.
      const url=new URL("content/"+item.id+".html", document.baseURI);
      const resp=await fetch(url.toString());
      if(!resp.ok) throw new Error("HTTP "+resp.status+" · "+url.pathname);
      const data=await resp.text();
      if(ticket!==pendingLesson) return;
      $("#lesson-content").innerHTML=data;
    } catch (e) {
      if(ticket!==pendingLesson)return;
      console.error("Lesson file load failed:",item.id,e);
      $("#lesson-content").innerHTML=
        '<div class="callout warning"><strong>讲解文件暂时无法读取。</strong>错误：'+safe(e.message||String(e))+
        '。你可以稍后重试，或<a href="content/'+encodeURIComponent(item.id)+'.html" target="_blank" rel="noopener noreferrer">直接查看讲解文件</a>。</div>';
      return;
    }
    // Widget exceptions must NEVER replace an already-loaded lesson with "load failed".
    attachLessonWidgets();
  }

  function renderRoute() {
    const id=idFromHash();
    const item=catalog.find(x=>x.id===id);
    if(item) showLesson(item);
    else showHome();
  }

  function attachLessonWidgets() {
    const widgets=[
      ["自测练习",()=>setupQuiz($("#lesson-content"))],
      ["图像交互",setupAreaButtons],
      ["高阶导互动",setupDerivativeLab],
      ["部分分式互动",setupDecompLab]
    ];
    for(const [name,init] of widgets){
      try { init(); }
      catch(e) {
        console.error("Interactive widget failed: "+name,e);
        const note=document.createElement("div");
        note.className="callout warning";
        note.textContent="讲解内容已正常加载，但「"+name+"」功能暂不可用。";
        $("#lesson-content").prepend(note);
      }
    }
  }

  function setupQuiz(scope) {
    $$(".quiz",scope).forEach(quiz => {
      const buttons=$$(".options button",quiz);
      const answer=Number(quiz.dataset.quizCorrect);
      const explain=quiz.dataset.quizExplain||"";
      const feedback=$(".quiz-feedback",quiz);
      buttons.forEach((btn,i)=>btn.addEventListener("click",()=>{
        if(buttons[0].disabled)return;
        buttons.forEach(x=>x.disabled=true);
        btn.classList.add(i===answer?"correct":"wrong");
        if(i!==answer)buttons[answer].classList.add("correct");
        feedback.textContent=(i===answer?"答对了。":"再注意这里。")+explain;
      }));
    });
  }

  function setupAreaButtons() {
    const buttons=$$("[data-area-btn]",$("#lesson-content"));
    if(!buttons.length)return;
    const texts=[
      "0→1：f′(x)<0，定积分等于 −3，所以 f(1)=f(0)−3=−2。",
      "1→3：f′(x)>0，定积分等于 +4，所以 f(3)=f(1)+4=2。",
      "3→4：f′(x)<0，定积分等于 −2，所以 f(4)=f(3)−2=0。"
    ];
    function choose(i){
      buttons.forEach((btn,k)=>btn.classList.toggle("active",k===i));
      $$(".garea",$("#lesson-content")).forEach((p,k)=>p.classList.toggle("active",k===i));
      const explain=$("#area-explain");
      if(explain)explain.textContent=texts[i];
    }
    buttons.forEach((btn,i)=>btn.addEventListener("click",()=>choose(i)));
    choose(0);
  }

  function setupDerivativeLab() {
    const input=$("#order-slider"),display=$("#derivative-display"),hint=$("#derivative-hint");
    if(!input || !display || !hint)return;
    let kind="reciprocal";
    const factorial=n=>Array.from({length:n},(_,i)=>i+1).reduce((a,b)=>a*b,1);
    function formula(n,type){
      const f=factorial(n);
      const minus=(type==="reciprocal"&&n%2===1)?"<mo>−</mo>":"";
      const denominator=type==="reciprocal"?
        (n===0?"<mi>x</mi>":"<msup><mi>x</mi><mn>"+(n+1)+"</mn></msup>"):
        (n===0?"<mrow><mo>(</mo><mn>1</mn><mo>−</mo><mi>x</mi><mo>)</mo></mrow>":
          "<msup><mrow><mo>(</mo><mn>1</mn><mo>−</mo><mi>x</mi><mo>)</mo></mrow><mn>"+(n+1)+"</mn></msup>");
      const num="<mn>"+f+"</mn>";
      return '<math xmlns="http://www.w3.org/1998/Math/MathML" display="block">'+
        '<msup><mi>f</mi><mrow><mo>(</mo><mn>'+n+'</mn><mo>)</mo></mrow></msup><mo>(</mo><mi>x</mi><mo>)</mo>'+
        '<mo>=</mo>'+minus+'<mfrac>'+num+denominator+'</mfrac></math>';
    }
    function draw(){
      const n=Number(input.value);
      $("#order-label").textContent=n;
      display.innerHTML=formula(n,kind);
      if(n===0){
        hint.textContent="第 0 阶就是原函数，尚未求导。";
      } else if(kind==="reciprocal"){
        hint.textContent="第 "+n+" 阶：符号"+(n%2?"为负":"为正")+"，系数 "+n+"! = "+factorial(n)+"，分母的 x 次数为 "+(n+1)+"。";
      } else {
        hint.textContent="第 "+n+" 阶：链式法则的两个负号始终抵消，系数 "+n+"! = "+factorial(n)+"，分母次数为 "+(n+1)+"。";
      }
    }
    input.addEventListener("input",draw);
    $$("[data-derivative-fn]",$("#lesson-content")).forEach(btn=>btn.addEventListener("click",()=>{
      kind=btn.dataset.derivativeFn;
      $$("[data-derivative-fn]",$("#lesson-content")).forEach(x=>x.classList.toggle("active",x===btn));
      draw();
    }));
    draw();
  }

  function setupDecompLab(){
    const lab=$("[data-decomp-lab]",$("#lesson-content"));
    if(!lab)return;
    const buttons=$$("[data-root]",lab),box=$("#decomp-result");
    if(!box)return;
    function choose(root){
      buttons.forEach(b=>b.classList.toggle("active",b.dataset.root===root));
      if(root==="one"){
        box.innerHTML='<div class="math"><math xmlns="http://www.w3.org/1998/Math/MathML" display="block">'+
          '<mn>2</mn><mo>=</mo><mn>2</mn><mi>A</mi><mo>+</mo><mn>0</mn><mi>B</mi>'+
          '<mspace width="1em"/><mo>⟹</mo><mspace width=".5em"/><mi>A</mi><mo>=</mo><mn>1</mn>'+
          '</math></div><p>t=1 时，t−1=0，B 项消失，所以直接得到 A=1。</p>';
      }else{
        box.innerHTML='<div class="math"><math xmlns="http://www.w3.org/1998/Math/MathML" display="block">'+
          '<mn>2</mn><mo>=</mo><mn>0</mn><mi>A</mi><mo>−</mo><mn>2</mn><mi>B</mi>'+
          '<mspace width="1em"/><mo>⟹</mo><mspace width=".5em"/><mi>B</mi><mo>=</mo><mo>−</mo><mn>1</mn>'+
          '</math></div><p>t=−1 时，t+1=0，A 项消失，所以直接得到 B=−1。</p>';
      }
    }
    buttons.forEach(b=>b.addEventListener("click",()=>choose(b.dataset.root)));
    choose("one");
  }

  function newReviewQuestion() {
    const pool=reviewQuestions;
    let i=Math.floor(Math.random()*pool.length);
    if(pool.length>1&&i===lastReview)i=(i+1)%pool.length;
    lastReview=i;
    const q=pool[i], el=$("#review-box");
    el.innerHTML='<div class="quiz"><span class="eyebrow">'+safe(q.topic)+'</span>'+
      '<h3 style="margin-top:10px">'+safe(q.question)+'</h3>'+
      '<div class="options">'+q.options.map((x,j)=>'<button type="button" data-review-answer="'+j+'">'+safe(x)+'</button>').join("")+'</div>'+
      '<div class="quiz-feedback" aria-live="polite">先选一个答案。</div></div>';
    const buttons=$$("[data-review-answer]",el);
    buttons.forEach((btn,j)=>btn.addEventListener("click",()=>{
      if(buttons[0].disabled)return;
      buttons.forEach(x=>x.disabled=true);
      btn.classList.add(j===q.answer?"correct":"wrong");
      if(j!==q.answer)buttons[q.answer].classList.add("correct");
      $(".quiz-feedback",el).textContent=(j===q.answer?"答对了。":"这一步需要再练习。")+q.explain;
    }));
  }

  function registerEvents() {
    $("#brand-home").addEventListener("click",()=>goHome());
    $("#nav-courses").addEventListener("click",()=>goHome("catalog"));
    $("#nav-points").addEventListener("click",()=>goHome("points"));
    $("#nav-review").addEventListener("click",()=>goHome("review"));
    $("#back-home").addEventListener("click",()=>goHome("catalog"));
    $("#search").addEventListener("input",renderCatalog);
    $("#only-unlearned")?.addEventListener("click",e=>{
      onlyPending=!onlyPending;
      e.currentTarget.textContent=onlyPending?"显示全部":"只看未掌握";
      e.currentTarget.classList.toggle("primary",onlyPending);
      renderCatalog();
    });
    $("#next-review").addEventListener("click",newReviewQuestion);
    $("#lesson-mark").addEventListener("click",e=>toggleMastery(e.currentTarget.dataset.lessonId));
    $("#catalog").addEventListener("click",e=>{
      const open=e.target.closest("[data-open]");
      if(open){openLesson(open.dataset.open);return;}
      const toggle=e.target.closest("[data-toggle]");
      if(toggle) toggleMastery(toggle.dataset.toggle);
    });
    $("#point-index").addEventListener("click",e=>{
      const open=e.target.closest("[data-open]");
      if(open)openLesson(open.dataset.open);
    });
    window.addEventListener("hashchange",renderRoute);
    window.addEventListener("popstate",renderRoute);
  }

  async function init() {
    mastery=getMastery();
    registerEvents();
    try{
      const r=await fetch("content/catalog.json");
      if(!r.ok)throw Error("HTTP "+r.status);
      catalog=await r.json();
      if(!Array.isArray(catalog)) throw Error("目录不是数组");
      saveMastery();
      updateProgress();
      renderCatalog();
      renderPoints();
      newReviewQuestion();
      renderRoute();
    }catch(e){
      $("#catalog").innerHTML='<div class="callout warning">精选专题暂时无法读取，请刷新重试。'+safe(e.message)+'</div>';
      $("#review-box").textContent="专题目录加载失败。";
    }
  }
  init();
})();
