import{f as d,h as _,j as h,ag as p,c as m,ah as u,l as n,m as i,ai as t,aj as o,y as s,F as f,ak as g,al as v,am as x,an as y,ao as b,ap as N,n as k,aq as w,_ as P}from"./nav-72150665.js";import{N as S}from"./NoteViewer-6811fa8a.js";import{u as V}from"./index-ce73241b.js";const j={class:"m-4"},L={class:"mb-10"},T={class:"text-4xl font-bold mt-2"},B={class:"opacity-50"},C={class:"text-lg"},D={class:"font-bold flex gap-2"},H={class:"opacity-50"},z=t("div",{class:"flex-auto"},null,-1),F={key:0,class:"border-gray-400/50 mb-8"},M=d({__name:"PresenterPrint",setup(q){_(h),p(`
@page {
  size: A4;
  margin-top: 1.5cm;
  margin-bottom: 1cm;
}
* {
  -webkit-print-color-adjust: exact;
}
html,
html body,
html #app,
html #page-root {
  height: auto;
  overflow: auto !important;
}
`),V({title:`Notes - ${m.title}`});const r=u(()=>y.slice(0,-1).map(a=>{var l;return(l=a.meta)==null?void 0:l.slide}).filter(a=>a!==void 0&&a.notesHTML!==""));return(a,l)=>(n(),i("div",{id:"page-root",style:v(s(x))},[t("div",j,[t("div",L,[t("h1",T,o(s(m).title),1),t("div",B,o(new Date().toLocaleString()),1)]),(n(!0),i(f,null,g(s(r),(e,c)=>(n(),i("div",{key:c,class:"flex flex-col gap-4 break-inside-avoid-page"},[t("div",null,[t("h2",C,[t("div",D,[t("div",H,o(e==null?void 0:e.no)+"/"+o(s(b)),1),N(" "+o(e==null?void 0:e.title)+" ",1),z])]),k(S,{"note-html":e.notesHTML,class:"max-w-full"},null,8,["note-html"])]),c<s(r).length-1?(n(),i("hr",F)):w("v-if",!0)]))),128))])],4))}}),W=P(M,[["__file","/home/feiyuxiao/Documents/Notes/Slides/slidev-theme-academic/node_modules/@slidev/client/internals/PresenterPrint.vue"]]);export{W as default};
