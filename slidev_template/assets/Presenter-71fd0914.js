import{l as d,m as k,ai as e,A as y,as as H,t as h,f as T,h as B,j as P,at as S,au as v,ar as z,y as t,_ as D,av as A,aw as I,c as b,K as j,q,ax as F,ay as L,w as R,az as E,aA as O,aB as G,n as u,aj as K,al as x,am as N,aC as $,aD as V,aq as U,d as W,ao as Z,aE as w,af as J,aF as Q,F as X,aG as Y,aH as ee,b as M,aI as te,aJ as se,aK as oe,aL as ae,aM as le,aN as ne,aO as ie,aP as re}from"./nav-17a90c75.js";import{N as ce}from"./NoteViewer-1051d102.js";import ue from"./DrawingControls-fb814f69.js";import{u as de}from"./index-69be57b6.js";const me={class:"slidev-icon",viewBox:"0 0 32 32",width:"1.2em",height:"1.2em"},_e=e("path",{fill:"currentColor",d:"M12 10H6.78A11 11 0 0 1 27 16h2A13 13 0 0 0 6 7.68V4H4v8h8zm8 12h5.22A11 11 0 0 1 5 16H3a13 13 0 0 0 23 8.32V28h2v-8h-8z"},null,-1),pe=[_e];function ve(a,n){return d(),k("svg",me,pe)}const he={name:"carbon-renew",render:ve},fe={class:"slidev-icon",viewBox:"0 0 32 32",width:"1.2em",height:"1.2em"},ge=e("path",{fill:"currentColor",d:"M16 30a14 14 0 1 1 14-14a14 14 0 0 1-14 14Zm0-26a12 12 0 1 0 12 12A12 12 0 0 0 16 4Z"},null,-1),xe=e("path",{fill:"currentColor",d:"M20.59 22L15 16.41V7h2v8.58l5 5.01L20.59 22z"},null,-1),we=[ge,xe];function ye(a,n){return d(),k("svg",fe,we)}const Se={name:"carbon-time",render:ye},ke="/talks/slide-template/assets/logo-title-horizontal-96c3c915.png";function Ce(){const a=y(Date.now()),n=H({interval:1e3}),m=h(()=>{const s=(n.value-a.value)/1e3,l=Math.floor(s%60).toString().padStart(2,"0");return`${Math.floor(s/60).toString().padStart(2,"0")}:${l}`});function _(){a.value=n.value}return{timer:m,resetTimer:_}}const be=T({__name:"NoteStatic",props:{class:{type:String,required:!1}},setup(a){const n=a;B(P);const m=h(()=>{var s,l,o;return(o=(l=(s=S.value)==null?void 0:s.meta)==null?void 0:l.slide)==null?void 0:o.note}),_=h(()=>{var s,l,o;return(o=(l=(s=S.value)==null?void 0:s.meta)==null?void 0:l.slide)==null?void 0:o.notesHTML});return(s,l)=>(d(),v(ce,{class:z(n.class),note:t(m),"note-html":t(_)},null,8,["class","note","note-html"]))}}),Ne=D(be,[["__file","/home/feiyuxiao/Documents/Notes/Slides/slidev-theme-academic/node_modules/@slidev/client/internals/NoteStatic.vue"]]),f=a=>(Y("data-v-574fd206"),a=a(),ee(),a),$e={class:"bg-main h-full slidev-presenter"},Ve={class:"grid-container"},Me={class:"grid-section top flex"},Te=f(()=>e("img",{src:ke,class:"ml-2 my-auto h-10 py-1 lg:h-14 lg:py-2"},null,-1)),Be=f(()=>e("div",{class:"flex-auto"},null,-1)),Pe={class:"text-2xl pl-2 pr-6 my-auto tabular-nums"},ze=f(()=>e("div",{class:"context"}," current ",-1)),De=f(()=>e("div",{class:"context"}," next ",-1)),He={class:"grid-section note overflow-auto"},Ae={class:"grid-section bottom"},Ie={class:"progress-bar"},je=T({__name:"Presenter",setup(a){B(P);const n=y();A(),I(n);const m=b.titleTemplate.replace("%s",b.title||"Slidev");de({title:`Presenter - ${m}`});const{timer:_,resetTimer:s}=Ce(),l=y([]),o=h(()=>M.value<te.value?{route:S.value,clicks:M.value+1}:se.value?{route:oe.value,clicks:0}:null);return j(()=>{const C=n.value.querySelector("#slide-content"),r=q(F()),g=L();R(()=>{if(!g.value||O.value||!G.value)return;const c=C.getBoundingClientRect(),i=(r.x-c.left)/c.width*100,p=(r.y-c.top)/c.height*100;if(!(i<0||i>100||p<0||p>100))return{x:i,y:p}},c=>{E.cursor=c})}),(C,r)=>{const g=Se,c=he;return d(),k(X,null,[e("div",$e,[e("div",Ve,[e("div",Me,[Te,Be,e("div",{class:"timer-btn my-auto relative w-22px h-22px cursor-pointer text-lg",opacity:"50 hover:100",onClick:r[0]||(r[0]=(...i)=>t(s)&&t(s)(...i))},[u(g,{class:"absolute"}),u(c,{class:"absolute opacity-0"})]),e("div",Pe,K(t(_)),1)]),e("div",{ref_key:"main",ref:n,class:"relative grid-section main flex flex-col p-2 lg:p-4",style:x(t(N))},[u(V,{key:"main",class:"h-full w-full"},{default:$(()=>[u(ae,{context:"presenter"})]),_:1}),ze],4),e("div",{class:"relative grid-section next flex flex-col p-2 lg:p-4",style:x(t(N))},[t(o)?(d(),v(V,{key:"next",class:"h-full w-full"},{default:$(()=>{var i;return[u(t(ne),{is:(i=t(o).route)==null?void 0:i.component,"clicks-elements":l.value,"onUpdate:clicks-elements":r[1]||(r[1]=p=>l.value=p),clicks:t(o).clicks,"clicks-disabled":!1,class:z(t(le)(t(o).route)),route:t(o).route,context:"previewNext"},null,8,["is","clicks-elements","clicks","class","route"])]}),_:1})):U("v-if",!0),De],4),e("div",He,[(d(),v(Ne,{key:1,class:"w-full h-full overflow-auto p-2 lg:p-4"}))]),e("div",Ae,[u(ie,{persist:!0})]),(d(),v(ue,{key:0}))]),e("div",Ie,[e("div",{class:"progress h-2px bg-primary transition-all",style:x({width:`${(t(W)-1)/(t(Z)-1)*100}%`})},null,4)])]),u(re),u(Q,{modelValue:t(w),"onUpdate:modelValue":r[2]||(r[2]=i=>J(w)?w.value=i:null)},null,8,["modelValue"])],64)}}});const Ee=D(je,[["__scopeId","data-v-574fd206"],["__file","/home/feiyuxiao/Documents/Notes/Slides/slidev-theme-academic/node_modules/@slidev/client/internals/Presenter.vue"]]);export{Ee as default};
