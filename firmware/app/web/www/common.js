"use strict";
/* Shared by every page (plans/web-config-server.md, "Pages").
 * Every request has a timeout: a Wi-Fi link that dies without a TCP reset
 * would otherwise leave a fetch hanging for minutes and the page showing
 * stale data as live. Polling stops while the tab is hidden, since each
 * open /…json request holds one of the unit's few transfer slots (web.c). */
const OG=(()=>{
  const TIMEOUT_MS=2500;
  async function req(url,opt,timeout){
    const c=new AbortController(),timer=setTimeout(()=>c.abort(),timeout||TIMEOUT_MS);
    try{
      const r=await fetch(url,Object.assign({cache:"no-store",signal:c.signal},opt));
      const text=await r.text();
      let j=null;
      try{j=JSON.parse(text);}catch(e){}
      if(j===null)throw new Error(r.ok?"unexpected reply":"HTTP "+r.status);
      /* A document that outgrew WEB_DOC_MAX is replaced by {"error":…}
       * (web.c); POST replies carry "ok", documents don't. */
      if(typeof j==="object"&&!("ok" in j)&&typeof j.error==="string")throw new Error(j.error);
      /* Its size in bytes, against the unit's WEB_DOC_MAX (web.h). */
      if(typeof j==="object")Object.defineProperty(j,"$bytes",{value:new TextEncoder().encode(text).length});
      return j;
    }finally{clearTimeout(timer);}
  }
  const getJson=(url,timeout)=>req(url,{},timeout);
  /* The unit takes a POST body of at most WEB_POST_MAX bytes (web.h). */
  const POST_MAX=1024;
  /* Each POST carries a request ID (rid). Until a form gets a reply from
   * the unit, sending the same form again reuses its ID, so a command the
   * unit did take is not queued a second time: config_web.c answers it as
   * a repeat ("repeat": "pending" or "done"). Kept per form, the last few. */
  const unanswered=new Map();
  async function postForm(url,params,timeout){
    const body=new URLSearchParams(params).toString();
    let rid=unanswered.get(body);
    if(!rid){
      rid=(crypto.getRandomValues(new Uint32Array(1))[0]>>>1)+1;
      unanswered.set(body,rid);
      while(unanswered.size>8)unanswered.delete(unanswered.keys().next().value);
    }
    const full=body+"&rid="+rid;
    if(full.length>POST_MAX){unanswered.delete(body);throw new Error("form too long for the unit ("+full.length+" of "+POST_MAX+" bytes): save fewer changes at once");}
    const j=await req(url,{method:"POST",
      headers:{"Content-Type":"application/x-www-form-urlencoded"},body:full},timeout);
    /* A reply: the unit has decided on this one. */
    unanswered.delete(body);
    if(typeof j==="object")Object.defineProperty(j,"$rid",{value:rid});
    return j;
  }
  const sleep=ms=>new Promise(r=>setTimeout(r,ms));
  /* The outcome of a command that POST /config accepted (reply j), from
   * last_result in /config.json, whose rid names the request it belongs to
   * (another tab's command is not taken for this one): {text, doc}, or null
   * when none came within 5 s in all. */
  async function commandResult(j){
    if(j.repeat==="done"&&j.result_seq===undefined)
      return {text:"the unit had already taken this command, before its last restart",doc:null};
    const end=Date.now()+5000;
    for(;;){
      const left=end-Date.now();
      if(left<=0)return null;
      try{
        const d=await getJson("/config.json",Math.min(left,TIMEOUT_MS)),r=d.last_result;
        if(r.rid===j.$rid)return {text:r.text,doc:d};
        /* A repeat already done whose result a later command replaced. */
        if(j.repeat==="done"&&r.seq>j.result_seq)return {text:"the unit had already done this command",doc:d};
      }catch(e){}
      await sleep(Math.min(250,Math.max(0,end-Date.now())));
    }
  }
  /* Calls async fn, then again period ms after it finished, while the page
   * is visible. Returns {now()} to poll at once. */
  function poll(fn,period){
    let timer=0,running=false;
    const hidden=()=>document.visibilityState==="hidden";
    async function tick(){
      timer=0;
      if(hidden())return;
      running=true;
      /* A timeout or lost link is normal here; log only real faults. */
      try{await fn();}catch(e){if(e.name!=="AbortError"&&!(e instanceof TypeError))console.error(e);}
      running=false;
      if(!hidden()&&!timer)timer=setTimeout(tick,period);
    }
    document.addEventListener("visibilitychange",()=>{if(!hidden()&&!timer&&!running)tick();});
    tick();
    return {now(){if(running)return;clearTimeout(timer);tick();}};
  }
  /* The clipboard API needs a secure context (https or localhost), which a
   * page served from the unit over plain http is not: fall back to a
   * selected textarea and execCommand. Resolves true when copied. */
  function copyText(t){
    if(navigator.clipboard&&window.isSecureContext)return navigator.clipboard.writeText(t).then(()=>true,()=>fallback(t));
    return Promise.resolve(fallback(t));
  }
  function fallback(t){
    const a=document.createElement("textarea");
    a.value=t;a.setAttribute("readonly","");
    a.style.cssText="position:fixed;top:0;left:0;opacity:0";
    document.body.appendChild(a);
    a.select();a.setSelectionRange(0,t.length);
    let ok=false;
    try{ok=document.execCommand("copy");}catch(e){}
    a.remove();
    return ok;
  }
  return {getJson,postForm,commandResult,poll,copyText};
})();
