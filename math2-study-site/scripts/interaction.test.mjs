// Browser-like runtime regression without external dependencies.
// This catches failures where loading a lesson succeeds but initializing a widget throws.
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import vm from "node:vm";
import {fileURLToPath} from "node:url";

const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),"..");
const read=relative=>fs.readFileSync(path.join(root,relative),"utf8");
const catalog=JSON.parse(read("content/catalog.json"));
const script=read("assets/site.js");

class Element {
  constructor(dataset={}) {
    this.dataset=dataset;
    this.innerHTML="";
    this.textContent="";
    this.value="";
    this.disabled=false;
    this.hidden=false;
    this.style={};
    this.listeners={};
    this.classList={
      add(){},
      remove(){},
      toggle(){},
      contains(){return false;}
    };
  }
  addEventListener(event,fn){(this.listeners[event]??=[]).push(fn);}
  click(){for(const fn of this.listeners.click??[])fn({currentTarget:this,target:this});}
  scrollIntoView(){}
  prepend(){}
  get parentNode(){return null;}
}

async function checkLesson(id){
  const registry=new Map();
  const getNode=selector=>{
    if(!registry.has(selector))registry.set(selector,new Element());
    return registry.get(selector);
  };
  const rootButtons=[
    new Element({root:"one"}),
    new Element({root:"minus-one"})
  ];
  const lab=new Element();
  const output=new Element();
  let renderedLesson="";
  const lessonContent=getNode("#lesson-content");
  Object.defineProperty(lessonContent,"innerHTML",{
    get(){return renderedLesson;},
    set(value){renderedLesson=value;}
  });

  const document={
    baseURI:"https://xu2905815209.github.io/0.0/",
    querySelector(selector){
      if(selector==="[data-decomp-lab]"){
        return renderedLesson.includes('data-decomp-lab')?lab:null;
      }
      if(selector==="#decomp-result"){
        return renderedLesson.includes('id="decomp-result"')?output:null;
      }
      if(selector==="#order-slider")return null;
      return getNode(selector);
    },
    querySelectorAll(selector){
      if(selector==="[data-root]")return rootButtons;
      return [];
    },
    createElement(){return new Element();}
  };
  const errors=[];
  const mockFetch=async url=>{
    const pathname=String(url);
    if(pathname==="content/catalog.json"){
      return {ok:true,json:async()=>catalog};
    }
    if(pathname.includes("content/"+id+".html")){
      return {ok:true,status:200,text:async()=>read("content/"+id+".html")};
    }
    throw new Error("Unexpected fetch "+url);
  };
  const location={hash:"#"+id,pathname:"/0.0/",search:""};
  const history={pushState(){}};
  const localStorage={
    getItem(){return null;},
    setItem(){}
  };
  const context={
    document,
    window:{scrollTo(){},addEventListener(){}},
    location,history,localStorage,
    fetch:mockFetch,URL,setTimeout,clearTimeout,
    console:{error:(...args)=>errors.push(args.join(" ")),log(){}}
  };

  vm.runInNewContext(script,context,{filename:"assets/site.js"});
  await new Promise(resolve=>setTimeout(resolve,60));
  assert(renderedLesson.includes('data-role="representative-problem"'),id+": lesson did not render; fragment prefix="+renderedLesson.slice(0,250)+"; errors="+errors.join(" | ")+"; catalog box="+getNode("#catalog").innerHTML.slice(0,250));
  assert(!renderedLesson.includes("讲解文件暂时无法读取"),id+": render failed");
  assert.equal(errors.length,0,id+": JavaScript widget error: "+errors.join(" | "));

  if(id==="k003"){
    assert(output.innerHTML.includes("<math"),"K003: initial coefficient result missing");
    assert(output.innerHTML.includes("<mi>A</mi>"),"K003: A result missing");
    assert((rootButtons[1].listeners.click||[]).length>0,"K003: second coefficient button inactive");
    rootButtons[1].click();
    assert(output.innerHTML.includes("<mi>B</mi>"),"K003: switching to B failed");
    assert(output.innerHTML.includes("<mo>−</mo><mn>1</mn>"),"K003: B=-1 answer wrong");
  }
  console.log("Lesson "+id+": content loaded, widget initialization OK");
}

for(const item of catalog)await checkLesson(item.id);
console.log("Interactive lesson smoke tests passed.");
