
(function(){
try{
var KEY='atariSioVisualSettingsF38', KEY32='atariSioVisualSettingsF32', CUSTOM='atariSioCustomThemesF38', OV='atariSioThemeOverridesF38';
var builtin={
'blue-pro':{themePreset:'blue-pro',primaryColor:'#1ea7ff',accentColor:'#59d7ff',bgColor:'#020817',accentGlow:18,fontFamily:'system'},
'classic-red':{themePreset:'classic-red',primaryColor:'#ef4444',accentColor:'#fb923c',bgColor:'#120607',accentGlow:18,fontFamily:'segoe'},
'fuji-cyan':{themePreset:'fuji-cyan',primaryColor:'#00d0ff',accentColor:'#00f5d4',bgColor:'#031019',accentGlow:20,fontFamily:'system'},
'atari-green':{themePreset:'atari-green',primaryColor:'#22c55e',accentColor:'#86efac',bgColor:'#02110a',accentGlow:16,fontFamily:'atari'},
'amber-retro':{themePreset:'amber-retro',primaryColor:'#f59e0b',accentColor:'#fde68a',bgColor:'#120a02',accentGlow:15,fontFamily:'courier'},
'purple-neon':{themePreset:'purple-neon',primaryColor:'#a855f7',accentColor:'#22d3ee',bgColor:'#09071a',accentGlow:22,fontFamily:'system'},
'mono-classic':{themePreset:'mono-classic',primaryColor:'#94a3b8',accentColor:'#e2e8f0',bgColor:'#050607',accentGlow:10,fontFamily:'mono'}
};
var defaults={themePreset:'blue-pro',primaryColor:'#1ea7ff',accentColor:'#59d7ff',bgColor:'#020817',accentGlow:18,libraryCoverWidth:180,libraryCoverHeight:190,fullscreenWidth:86,fullscreenHeight:86,fontFamily:'system'};
function read(k,f){try{var raw=localStorage.getItem(k); return raw?JSON.parse(raw):f;}catch(e){return f;}}
function rgb(hex){hex=String(hex||'').trim();var m=/^#?([a-f0-9]{6})$/i.exec(hex); if(!m)return{r:30,g:167,b:255};var n=parseInt(m[1],16);return{r:(n>>16)&255,g:(n>>8)&255,b:n&255};}
function clamp(v,min,max){v=Number(v); return Number.isFinite(v)?Math.max(min,Math.min(max,v)):min;}
function font(v){switch(String(v||'system')){case 'segoe':return '"Segoe UI", system-ui, -apple-system, sans-serif';case 'arial':return 'Arial, Helvetica, sans-serif';case 'verdana':return 'Verdana, Geneva, sans-serif';case 'mono':return 'ui-monospace, SFMono-Regular, Menlo, Consolas, monospace';case 'courier':return '"Courier New", Courier, monospace';case 'atari':return '"Atari Classic", "Atari 8-bit", "Atari 400/800", "AtariSmall", "Press Start 2P", "Courier New", ui-monospace, monospace';default:return 'system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';}}
var stored=read(KEY,null)||read(KEY32,null)||{};
var overrides=read(OV,{}); if(!overrides||Array.isArray(overrides)) overrides={};
var custom=read(CUSTOM,[]); if(!Array.isArray(custom)) custom=[];
var id=stored.themePreset||defaults.themePreset;
var base=(builtin[id]?Object.assign({},builtin[id],overrides[id]||{}):null) || custom.find(function(t){return t&&t.id===id;}) || builtin['blue-pro'];
var s=Object.assign({},defaults,base,stored);
var p=rgb(s.primaryColor), a=rgb(s.accentColor), b=rgb(s.bgColor), r=document.documentElement;
function set(n,v){r.style.setProperty(n,v,'important');}
set('--accent',s.primaryColor); set('--accent2',s.accentColor); set('--accent-rgb',p.r+','+p.g+','+p.b); set('--accent2-rgb',a.r+','+a.g+','+a.b);
set('--accent-soft','rgba('+p.r+','+p.g+','+p.b+',.16)'); set('--accent-strong','rgba('+p.r+','+p.g+','+p.b+',.46)');
set('--app-bg',s.bgColor); set('--app-bg-rgb',b.r+','+b.g+','+b.b); set('--ui-glow-alpha',String(clamp(s.accentGlow,5,35)/100));
set('--lib-cover-w',clamp(s.libraryCoverWidth,120,420)+'px'); set('--lib-cover-h',clamp(s.libraryCoverHeight,120,420)+'px'); set('--lib-card-min',Math.max(170,clamp(s.libraryCoverWidth,120,420)+28)+'px');
set('--fullscreen-img-w',clamp(s.fullscreenWidth,40,96)+'vw'); set('--fullscreen-img-h',clamp(s.fullscreenHeight,40,96)+'vh'); set('--app-font',font(s.fontFamily));
r.dataset.themeBoot='f44';
}catch(e){}

})();



function $(id) { return document.getElementById(id); }
function updateFileLabel(inputId, labelId, emptyText) {
const input = $(inputId);
const label = $(labelId);
if (!input || !label) return;
const files = Array.from(input.files || []);
if (!files.length) { label.textContent = emptyText; return; }
if (files.length === 1) { label.textContent = files[0].name; return; }
label.textContent = `${files.length} archivos seleccionados: ${files.slice(0,3).map(f => f.name).join(", ")}${files.length > 3 ? "..." : ""}`;
}
function bindFileLabels() {
const atr = $("atrFile");
if (atr) atr.addEventListener("change", () => updateFileLabel("atrFile", "atrFileName", "Ningún archivo seleccionado"));
const glyph = $("glyphFile");
if (glyph) glyph.addEventListener("change", () => updateFileLabel("glyphFile", "glyphFileName", "Ningún JSON seleccionado"));
}
function toggleUploadSegment() {
const s = $("atrUploadPageBlock");
if (!s) return;
s.classList.toggle("upload-collapsed");
}
function applyRouteLayout() {
const path = (location.pathname || "/").toLowerCase();
document.body.classList.remove("route-home","route-disk","route-atr","route-library","route-cassette","route-bt","route-tnfs","route-printer","route-glyphs","route-settings");
let route = "home";
if (path.indexOf("/disk") === 0 || path.indexOf("/editor/disks") === 0 || path.indexOf("/editor/disk") === 0) route = "disk";
else if (path.indexOf("/atr") === 0 || path.indexOf("/editor/web-atr") === 0 || path.indexOf("/editor/webatr") === 0) route = "atr";
else if (path.indexOf("/upload") === 0) { location.replace("/library"); return; }
else if (path.indexOf("/library") === 0 || path.indexOf("/editor/library") === 0) route = "library";
else if (path.indexOf("/cassette") === 0 || path.indexOf("/cas") === 0 || path.indexOf("/editor/cassette") === 0 || path.indexOf("/editor/cas") === 0) route = "cassette";
else if (path.indexOf("/bt") === 0 || path.indexOf("/editor/bt") === 0) route = "bt";
else if (path.indexOf("/tnfs") === 0 || path.indexOf("/editor/tnfs") === 0) route = "tnfs";
else if (path.indexOf("/printer") === 0 || path.indexOf("/editor/printer") === 0) route = "printer";
else if (path.indexOf("/glyphs") === 0 || path.indexOf("/editor/glyphs") === 0 || path.indexOf("/editor/glifos") === 0) route = "glyphs";
else if (path.indexOf("/settings") === 0 || path.indexOf("/appearance") === 0 || path.indexOf("/apariencia") === 0 || path.indexOf("/editor/settings") === 0 || path.indexOf("/editor/appearance") === 0 || path.indexOf("/editor/apariencia") === 0) route = "settings";
else if (path.indexOf("/config") === 0 || path.indexOf("/editor/config") === 0) { location.replace("/editor/disks"); return; }
document.body.classList.add("route-" + route);
const nav = { home:"navHome", disk:"navDisk", atr:"navAtr", library:"navLibrary", cassette:"navCassette", bt:"navBt", tnfs:"navTnfs", printer:"navPrinter", glyphs:"navGlyphs", settings:"navSettings" };
Object.keys(nav).forEach(k => { const a=$(nav[k]); if (a) a.classList.toggle("active", k===route); });
const title = document.querySelector(".page-title");
const subtitle = document.querySelector(".page-subtitle");
const rIcon = $("routeIcon");
const routeMeta = {
home:    ["Inicio", "Interfaz web de control para Atari SIO", "⌂"],
disk:    ["Disqueteras", "Administra unidades SIO, perfiles y parámetros de comunicación", "▣"],
atr:     ["WEB-ATR", "Montaje rápido de imágenes ATR y XEX por unidad", "◎"],
library: ["Biblioteca", "Archivos ATR/XEX/CAS en la SD/Flash con carátulas y montaje rápido", "▤"],
cassette:["Cassette C:", "Control manual y turbo para archivos .CAS preparados desde Biblioteca", "▧"],
bt:      ["Bluetooth SIO2PC", "Pasarela experimental compatible con RespeQt/AspeQt", "◌"],
tnfs:    ["Servidores TNFS", "Explora servidores remotos al estilo FujiNet", "◎"],
printer: ["Servicio de Impresión", "Imprime desde Atari SIO usando IPP, Gateway o spool virtual", "▥"],
glyphs:  ["Editor de Glifos", "Crea y edita caracteres personalizados para tu Atari SIO", "▦"],
settings:["Apariencia", "Personaliza colores, logo, letra y tamaño de carátulas", "◈"]
};
const meta = routeMeta[route] || routeMeta.home;
if (title) title.textContent = meta[0];
if (subtitle) subtitle.textContent = meta[1];
if (rIcon) rIcon.textContent = meta[2];
const cardTitle = document.querySelector("#configCard > h2");
const cardSmall = document.querySelector("#configCard > small");
if (cardTitle) {
if (route === "printer") cardTitle.innerHTML = "<span class='icon'>🖨️</span> Impresora";
else if (route === "atr") cardTitle.innerHTML = "<span class='icon'>📀</span> WEB-ATR Gateway";
else if (route === "upload") cardTitle.innerHTML = "<span class='icon'>⬆️</span> Subir archivo";
else if (route === "library") cardTitle.innerHTML = "<span class='icon'>⬆️📚</span> Biblioteca";
else if (route === "bt") cardTitle.innerHTML = "<span class='icon'>🔵</span> Bluetooth SIO2PC";
else if (route === "tnfs") cardTitle.innerHTML = "<span class='icon'>🌐</span> Servidores TNFS";
else if (route === "settings") cardTitle.innerHTML = "<span class='icon'>◈</span> Apariencia";
else cardTitle.innerHTML = "<span class='icon'>⚙️</span> Parámetros SIO";
}
if (cardSmall) {
if (route === "printer") cardSmall.textContent = "Perfiles de impresión, papel, tamaño de letra y orientación.";
else if (route === "atr") cardSmall.textContent = "Montaje por unidad; Subir y Biblioteca están juntos en la página Biblioteca.";
else if (route === "upload") cardSmall.textContent = "Carga archivos y administra la biblioteca del MASTER.";
else if (route === "library") cardSmall.textContent = "Subida, listado y acciones de biblioteca en una sola página.";
else if (route === "bt") cardSmall.textContent = "Pasarela experimental compatible con RespeQt/AspeQt por Bluetooth SPP.";
else if (route === "tnfs") cardSmall.textContent = "Agrega hosts TNFS y navega directorios remotos.";
else if (route === "settings") cardSmall.textContent = "Colores, logo, tipografía y tamaños visuales de la interfaz.";
else cardSmall.textContent = "Tiempos SIO, comunicación, verificación WRITE, prefetch y auto-ajuste por unidad.";
}
const imp = $("glyphImportExportBlock");
const ed = $("glyphsPageBlock");
if (imp) imp.style.display = "none";
if (ed) ed.style.display = "none";
if (route === "glyphs" && typeof buildGlyphV27Layout === "function") {
buildGlyphV27Layout();
glyphEnsureGridEditorReadyV38();
if (typeof glyphLoadCurrent === "function") setTimeout(() => { glyphLoadCurrent().finally(() => glyphEnsureGridEditorReadyV38()); }, 50);
}
}
// V30: se difiere la inicialización de ruta para evitar que /editor/glyphs
// ejecute buildGlyphV27Layout() antes de inicializar GLYPH5X7_V27.
// Ese orden dejaba la página de glifos sin JS funcional.
setTimeout(applyRouteLayout, 0);
function applyF28UiTweaks(){
const body = document.body;
const cfg = document.querySelector('#configCard > h2');
const cfgSmall = document.querySelector('#configCard > small');
if (body.classList.contains('route-printer')) {
if (cfg) cfg.innerHTML = "<span class='icon'>🖨️</span> Impresión";
if (cfgSmall) cfgSmall.textContent = "Impresión moderna desde Atari, usando Gateway Windows, perfiles y spool virtual.";
}
if (body.classList.contains('route-disk')) {
const disk = document.querySelector('#diskDrivesCard > h2');
const small = document.querySelector('#diskDrivesCard > small');
if (disk) disk.style.display = "none";
if (small) small.style.display = "none";
}
if (body.classList.contains('route-tnfs')) {
const status = document.getElementById('tnfsStatus');
if (status && !status.dataset.f28) { status.dataset.f28='1'; status.textContent = 'Selecciona un servidor para explorar carpetas; ATR monta en D:, CAS prepara C:, XEX/COM/EXE/BAS descarga crudo.'; }
}
}
setTimeout(applyF28UiTweaks, 40);
/* =============================================================
V27 GLYPH VISUAL LAYOUT
============================================================= */
const GLYPH5X7_V27 = {
"0":[14,17,19,21,25,17,14], "1":[4,12,4,4,4,4,14], "2":[14,17,1,2,4,8,31], "3":[30,1,1,14,1,1,30],
"4":[2,6,10,18,31,2,2], "5":[31,16,30,1,1,17,14], "6":[6,8,16,30,17,17,14], "7":[31,1,2,4,8,8,8],
"8":[14,17,17,14,17,17,14], "9":[14,17,17,15,1,2,12],
"A":[14,17,17,31,17,17,17], "B":[30,17,17,30,17,17,30], "C":[14,17,16,16,16,17,14], "D":[30,17,17,17,17,17,30],
"E":[31,16,16,30,16,16,31], "F":[31,16,16,30,16,16,16], "G":[14,17,16,23,17,17,14], "H":[17,17,17,31,17,17,17],
"I":[14,4,4,4,4,4,14], "J":[1,1,1,1,17,17,14], "K":[17,18,20,24,20,18,17], "L":[16,16,16,16,16,16,31],
"M":[17,27,21,21,17,17,17], "N":[17,25,21,19,17,17,17], "O":[14,17,17,17,17,17,14], "P":[30,17,17,30,16,16,16],
"Q":[14,17,17,17,21,18,13], "R":[30,17,17,30,20,18,17], "S":[15,16,16,14,1,1,30], "T":[31,4,4,4,4,4,4],
"U":[17,17,17,17,17,17,14], "V":[17,17,17,17,17,10,4], "W":[17,17,17,21,21,21,10], "X":[17,17,10,4,10,17,17],
"Y":[17,17,10,4,4,4,4], "Z":[31,1,2,4,8,16,31],
"a":[0,0,14,1,15,17,15], "b":[16,16,30,17,17,17,30], "c":[0,0,14,16,16,17,14], "d":[1,1,15,17,17,17,15],
"e":[0,0,14,17,31,16,14], "f":[6,8,8,30,8,8,8], "g":[0,0,15,17,17,15,1], "h":[16,16,30,17,17,17,17],
"i":[4,0,12,4,4,4,14], "j":[2,0,6,2,2,18,12], "k":[16,16,18,20,24,20,18], "l":[12,4,4,4,4,4,14],
"m":[0,0,26,21,21,17,17], "n":[0,0,30,17,17,17,17], "o":[0,0,14,17,17,17,14], "p":[0,0,30,17,17,30,16],
"q":[0,0,15,17,17,15,1], "r":[0,0,22,24,16,16,16], "s":[0,0,15,16,14,1,30], "t":[8,8,30,8,8,8,6],
"u":[0,0,17,17,17,19,13], "v":[0,0,17,17,17,10,4], "w":[0,0,17,17,21,21,10], "x":[0,0,17,10,4,10,17],
"y":[0,0,17,17,17,15,1], "z":[0,0,31,2,4,8,31],
"@":[14,17,23,21,23,16,14], "#":[10,10,31,10,31,10,10], "$": [4,15,20,14,5,30,4], "%":[24,25,2,4,8,19,3],
"&":[12,18,20,8,21,18,13], "!": [4,4,4,4,4,0,4], "?": [14,17,1,2,4,0,4], "+":[0,4,4,31,4,4,0],
"-": [0,0,0,31,0,0,0], "=": [0,31,0,31,0,0,0], "/":[1,2,4,8,16,0,0], "\\":[16,8,4,2,1,0,0],
"[": [14,8,8,8,8,8,14], "]": [14,2,2,2,2,2,14], "(": [2,4,8,8,8,4,2], ")": [8,4,2,2,2,4,8],
"<": [2,4,8,16,8,4,2], ">": [8,4,2,1,2,4,8], "*": [0,10,4,31,4,10,0], ":": [0,4,0,0,4,0,0],
";": [0,4,0,0,4,4,8], ".": [0,0,0,0,0,12,12], ",": [0,0,0,0,12,4,8], "'": [4,4,8,0,0,0,0],
'"': [10,10,0,0,0,0,0], "_": [0,0,0,0,0,0,31], "|": [4,4,4,4,4,4,4], "~": [0,0,8,21,2,0,0]
};
const GLYPH_CONTROL_8X8_V34 = {
0x00:["18","3C","7E","FF","7E","3C","18","00"],
0x01:["10","30","7F","7F","30","10","00","00"],
0x02:["04","06","7F","7F","06","04","00","00"],
0x03:["18","3C","7E","18","18","18","18","00"],
0x04:["18","18","18","18","7E","3C","18","00"],
0x05:["18","3C","7E","DB","18","18","18","00"],
0x06:["18","18","18","DB","7E","3C","18","00"],
0x07:["00","66","FF","FF","7E","3C","18","00"],
0x08:["18","3C","7E","FF","FF","66","00","00"],
0x09:["18","3C","7E","FF","7E","3C","18","00"],
0x0A:["18","3C","7E","DB","FF","24","66","00"],
0x0B:["18","3C","7E","FF","DB","18","3C","00"],
0x0C:["18","18","7E","18","18","00","7E","00"],
0x0D:["00","00","00","7E","7E","00","00","00"],
0x0E:["00","18","3C","7E","18","18","00","00"],
0x0F:["00","18","18","7E","3C","18","00","00"],
0x10:["08","18","38","7F","38","18","08","00"],
0x11:["10","18","1C","FE","1C","18","10","00"],
0x12:["00","36","7F","7F","3E","1C","08","00"],
0x13:["08","1C","3E","7F","3E","1C","08","00"],
0x14:["1C","3E","7F","7F","7F","36","14","00"],
0x15:["08","1C","3E","7F","1C","1C","3E","00"],
0x16:["00","18","3C","7E","7E","3C","18","00"],
0x17:["FF","E7","C3","81","81","C3","E7","FF"],
0x18:["00","3C","42","81","81","42","3C","00"],
0x19:["FF","C3","BD","7E","7E","BD","C3","FF"],
0x1A:["0F","07","0F","7D","CC","CC","CC","78"],
0x1B:["3C","66","66","66","3C","18","7E","18"],
0x1C:["3F","33","3F","30","30","70","F0","E0"],
0x1D:["7F","63","7F","63","63","67","E6","C0"],
0x1E:["18","DB","3C","E7","E7","3C","DB","18"],
0x1F:["80","E0","F8","FE","F8","E0","80","00"],
0x7F:["FF","FF","FF","FF","FF","FF","FF","FF"]
};
function glyphRowsForCodeV27(code) {
code = Number(code || 0) & 0x7F;
const cached = glyphRowsFromCacheV30(code);
if (cached && cached.length) return cached;
if (GLYPH_CONTROL_8X8_V34[code]) return GLYPH_CONTROL_8X8_V34[code].slice();
const ch = String.fromCharCode(code);
const rows5 = GLYPH5X7_V27[ch] || GLYPH5X7_V27[ch.toUpperCase?.() || ch];
if (rows5) {
const out = [0];
rows5.forEach(v => out.push((Number(v || 0) & 31) << 1));
while (out.length < 8) out.push(0);
return out.slice(0,8).map(v => v.toString(16).toUpperCase().padStart(2,"0"));
}
const out = [];
for (let y=0; y<8; y++) {
let v = 0;
for (let x=0; x<8; x++) {
const on = ((code + y*13 + x*7) & 11) === 0 || (code >= 0x70 && ((x+y+code)&1)===0);
if (on) v |= (1 << (7-x));
}
out.push(v.toString(16).toUpperCase().padStart(2,"0"));
}
return out;
}
function glyphPaintRowsV27(el, rows, cls) {
if (!el) return;
rows = (rows && rows.length) ? rows : glyphRowsForCodeV27(0x41);
el.className = cls || el.className || "glyph-preview-pixels";
el.innerHTML = "";
for (let y=0; y<8; y++) {
const v = parseInt(rows[y] || "0", 16) || 0;
for (let x=0; x<8; x++) {
const i = document.createElement("i");
if (v & (1 << (7-x))) i.className = "on";
el.appendChild(i);
}
}
}
function glyphCharLabelV27(code) {
code = Number(code || 0) & 0x7F;
if (code >= 32 && code <= 126) return String.fromCharCode(code);
return "·";
}
let glyphToolV30 = "pencil";
let glyphPointerPaintingV31 = false;
let glyphPaintTargetStateV33 = true;
window.addEventListener("pointerup", () => { glyphPointerPaintingV31 = false; glyphPaintTargetStateV33 = true; });
window.addEventListener("pointercancel", () => { glyphPointerPaintingV31 = false; glyphPaintTargetStateV33 = true; });
window.glyphRowsCacheV30 = window.glyphRowsCacheV30 || {};
function glyphHexV30(code) {
return (Number(code || 0) & 0x7F).toString(16).toUpperCase().padStart(2, "0");
}
function glyphRowToHex8V53(v) {
if (typeof v === "number") {
const n = Number.isFinite(v) ? Math.max(0, Math.min(255, Math.round(v))) : 0;
return n.toString(16).toUpperCase().padStart(2, "0");
}
let s = String(v ?? "0").trim();
if (!s) return "00";
if (/^0b[01]{1,16}$/i.test(s)) return (parseInt(s.slice(2), 2) & 0xFF).toString(16).toUpperCase().padStart(2, "0");
if (/^[01]{8,16}$/.test(s)) return (parseInt(s, 2) & 0xFF).toString(16).toUpperCase().padStart(2, "0");
if (/^0x[0-9a-f]{1,4}$/i.test(s)) return (parseInt(s, 16) & 0xFF).toString(16).toUpperCase().padStart(2, "0");
if (/^[0-9a-f]{1,2}$/i.test(s)) return (parseInt(s, 16) & 0xFF).toString(16).toUpperCase().padStart(2, "0");
if (/^\d{3,5}$/.test(s)) return (Math.max(0, Math.min(255, parseInt(s, 10) || 0)) & 0xFF).toString(16).toUpperCase().padStart(2, "0");
s = s.toUpperCase().replace(/[^0-9A-F]/g, "");
if (!s) return "00";
return (parseInt(s.slice(-2), 16) & 0xFF).toString(16).toUpperCase().padStart(2, "0");
}
function glyphNormalizeRowsV30(rows) {
const out = [];
if (Array.isArray(rows)) rows.slice(0, 8).forEach(r => out.push(glyphRowToHex8V53(r)));
while (out.length < 8) out.push("00");
return out.slice(0, 8);
}
function glyphCacheRowsV30(code, rows) {
const hex = glyphHexV30(code);
const norm = glyphNormalizeRowsV30(rows);
window.glyphRowsCacheV30[hex] = norm;
return norm;
}
function glyphRowsFromCacheV30(code) {
const hex = glyphHexV30(code);
return (window.glyphRowsCacheV30 && window.glyphRowsCacheV30[hex]) || null;
}
function glyphSetToolV30(tool) {
glyphToolV30 = tool || "pencil";
document.querySelectorAll("[data-glyph-tool]").forEach(btn => {
const t = btn.getAttribute("data-glyph-tool");
btn.classList.toggle("active", t === glyphToolV30 && (t === "pencil" || t === "eraser"));
});
glyphSetEditorStatus(tool === "eraser" ? "Herramienta: borrador" : "Herramienta: lápiz", true);
}
function glyphApplyGridCellV30(cell, isDrag) {
if (!cell) return;
if (glyphToolV30 === "eraser") {
cell.classList.remove("on");
} else if (glyphToolV30 === "invert") {
if (!isDrag) cell.classList.toggle("on");
} else {
// V33: el lápiz permite agregar y también borrar al tocar un punto ya encendido.
// En arrastre mantiene el estado calculado al iniciar el trazo.
cell.classList.toggle("on", !!glyphPaintTargetStateV33);
}
glyphUpdatePreview();
}
function glyphFillGridV30(on) {
const grid = $("glyphGrid");
if (!grid) return;
Array.from(grid.children || []).forEach(cell => cell.classList.toggle("on", !!on));
glyphUpdatePreview();
glyphSetEditorStatus(on ? "Glifo rellenado" : "Glifo borrado", true);
}
function glyphInvertGridV30() {
const grid = $("glyphGrid");
if (!grid) return;
Array.from(grid.children || []).forEach(cell => cell.classList.toggle("on"));
glyphUpdatePreview();
glyphSetEditorStatus("Glifo invertido", true);
}
function glyphParseRowsMapFromJsonV30(txt) {
const map = {};
try {
const json = JSON.parse(txt);
const fonts = Array.isArray(json.fonts) ? json.fonts : [];
fonts.forEach(font => {
const glyphs = Array.isArray(font.glyphs) ? font.glyphs : [];
glyphs.forEach(g => {
const code = glyphCodeFromAnyV31(g.code ?? g.codeHex ?? g.id);
if (!Number.isFinite(code) || code < 0 || code > 127) return;
const rows = glyphNormalizeRowsV30(g.rowsHex || g.rows || []);
map[glyphHexV30(code)] = rows;
});
});
} catch (e) {
console.warn("No se pudo interpretar JSON de glifos", e);
}
return map;
}
async function refreshGlyphMatrixFromDeviceV30() {
try {
const r = await fetch("/glyphs_export?reload=1&default=1&ts=" + Date.now(), { cache: "no-store" });
if (!r.ok) throw new Error("HTTP " + r.status);
const txt = await r.text();
const map = glyphParseRowsMapFromJsonV30(txt);
Object.keys(map).forEach(hex => {
const code = parseInt(hex, 16);
glyphCacheRowsV30(code, map[hex]);
glyphUpdateMatrixCellV28(code, map[hex]);
});
const count = Object.keys(map).length;
if (count) glyphSetEditorStatus("Glifos cargados: " + count, true);
} catch (e) {
console.warn("Matriz de glifos usando base local", e);
}
}
function buildGlyphMatrixV27(force) {
const grid = $("glyphMatrixV27");
if (!grid) return;
if (force) { grid.dataset.ready = "0"; grid.innerHTML = ""; }
if (grid.dataset.ready === "1" && grid.dataset.layout === "f58") return;
grid.dataset.ready = "1";
grid.dataset.layout = "f58";
grid.innerHTML = "";
for (let row=0; row<16; row++) {
for (let col=0; col<8; col++) {
const code = col*16 + row;
const hex = code.toString(16).toUpperCase().padStart(2,"0");
const b = document.createElement("button");
b.type = "button";
b.className = "glyph-code-cell";
b.dataset.code = String(code);
b.dataset.codehex = hex;
b.title = "0x" + hex + " · " + glyphControlNameV33(code);
const lab = document.createElement("span");
lab.className = "glyph-code-label-f58";
lab.textContent = hex;
const pix = document.createElement("div");
glyphPaintRowsV27(pix, glyphRowsForCodeV27(code), "glyph-mini-pixels");
b.appendChild(lab);
b.appendChild(pix);
b.addEventListener("click", () => glyphSelectCodeV27(code, true));
b.addEventListener("dblclick", (ev) => { ev.preventDefault(); glyphFullscreenOpenV29(code, glyphRowsForCodeV27(code)); });
grid.appendChild(b);
}
}
}
function glyphSelectCodeV27(code, load) {
code = Number(code || 0) & 0x7F;
const hex = code.toString(16).toUpperCase().padStart(2,"0");
if ($("glyphEditCode")) $("glyphEditCode").value = hex;
if ($("glyphEditName")) $("glyphEditName").value = glyphCharLabelV27(code);
if ($("glyphCodeInputV27")) $("glyphCodeInputV27").value = "0x" + hex + " (" + code + ")";
if ($("glyphCharInputV27")) $("glyphCharInputV27").value = glyphCharLabelV27(code);
document.querySelectorAll(".glyph-code-cell").forEach(el => el.classList.toggle("active", Number(el.dataset.code) === code));
if (typeof glyphUpdateViewActiveV32 === "function") glyphUpdateViewActiveV32(code);
if (load && typeof glyphLoadCurrent === "function") glyphLoadCurrent();
}
function glyphSyncV27(code, glyph) {
code = Number(code || 0) & 0x7F;
const rows = (glyph && glyph.rowsHex) ? glyph.rowsHex : glyphRowsFromGrid();
glyphSelectCodeV27(code, false);
glyphPaintRowsV27($("glyphSelectedPreviewV27"), rows, "glyph-preview-pixels small");
glyphPaintRowsV27($("glyphZoomPreviewV27"), rows, "glyph-preview-pixels zoom");
glyphUpdateMatrixCellV28(code, rows);
if (glyphFullscreenIsOpenV29(code)) glyphFullscreenPaintV29(rows);
}
function glyphUpdateMatrixCellV28(code, rows) {
const cell = document.querySelector('.glyph-code-cell[data-code="' + (Number(code || 0) & 0x7F) + '"]');
if (!cell) return;
const pix = cell.querySelector('.glyph-mini-pixels') || document.createElement('div');
if (!pix.parentNode) cell.appendChild(pix);
const finalRows = rows && rows.length ? rows : glyphRowsForCodeV27(code);
glyphPaintRowsV27(pix, finalRows, "glyph-mini-pixels");
if (typeof glyphUpdateHexRowV32 === "function") glyphUpdateHexRowV32(code, finalRows);
const chCard = document.querySelector('.glyph-char-card[data-code="' + (Number(code || 0) & 0x7F) + '"] .glyph-mini-pixels');
if (chCard) glyphPaintRowsV27(chCard, finalRows, 'glyph-mini-pixels');
}
function glyphCurrentSelectedCodeV32() {
try { return glyphHexCodeValue(); } catch(e) { return 0x41; }
}
function glyphUpdateViewActiveV32(code) {
code = Number(code || glyphCurrentSelectedCodeV32()) & 0x7F;
document.querySelectorAll('.glyph-char-card,.glyph-hex-row').forEach(el => {
el.classList.toggle('active', Number(el.dataset.code) === code);
});
}
function glyphBuildCharsViewV32(force) {}
function glyphBuildHexViewV32(force) {}
function glyphMoveEditorV33(view, code) {
const editor = $('glyphEditorPanelV33');
const home = $('glyphEditorHomeV33');
if (!editor || !home) return;
const v = view || (document.querySelector('[data-glyph-view].active')?.dataset.glyphView || 'grid');
if (v === 'chars' || v === 'hex') {
const selector = v === 'chars' ? '.glyph-char-card.active' : '.glyph-hex-row.active';
const active = document.querySelector(selector);
if (active && active.parentNode) {
editor.classList.add('glyph-inline-editor-v33');
active.insertAdjacentElement('afterend', editor);
return;
}
}
editor.classList.remove('glyph-inline-editor-v33');
if (home.nextElementSibling !== editor) home.insertAdjacentElement('afterend', editor);
}
function glyphControlNameV33(code) {
code = Number(code || 0) & 0x7F;
const names = ["NUL","SOH","STX","ETX","EOT","ENQ","ACK","BEL","BS","TAB","LF","VT","FF","CR","SO","SI","DLE","DC1","DC2","DC3","DC4","NAK","SYN","ETB","CAN","EM","SUB","ESC","FS","GS","RS","US"];
if (code < 0x20) return names[code] || ('0x' + glyphHexV30(code));
if (code === 0x7F) return 'DEL';
return glyphCharLabelV27(code);
}
function glyphSwitchViewV32(view) { buildGlyphMatrixV27(false); glyphEnsureGridEditorReadyV38(); }
function glyphUpdateHexRowV32(code, rows) { glyphUpdateViewActiveV32(code); }
function glyphEnsureGridEditorReadyV38(options) {
options = options || {};
const editor = $('glyphEditorPanelV33');
const home = $('glyphEditorHomeV33');
const mount = $('glyphBitmapEditorV27');
const bitmapGrid = $('glyphGrid');
if (!editor || !home) return;
if (bitmapGrid && mount && bitmapGrid.parentNode !== mount) {
mount.appendChild(bitmapGrid);
}
document.querySelectorAll('[data-glyph-view]').forEach(tab => {
tab.classList.toggle('active', (tab.dataset.glyphView || 'grid') === 'grid');
});
const panels = { grid:'glyphViewGridV32', chars:'glyphViewCharsV32', hex:'glyphViewHexV32' };
Object.keys(panels).forEach(k => {
const el = $(panels[k]);
if (el) el.classList.toggle('active', k === 'grid');
});
editor.classList.remove('glyph-inline-editor-v33');
editor.classList.remove('glyph-v27-hidden');
editor.classList.add('glyph-grid-ready-v38');
editor.style.display = 'grid';
editor.style.visibility = 'visible';
editor.style.opacity = '1';
if (home.nextElementSibling !== editor) home.insertAdjacentElement('afterend', editor);
let code = 0x41;
try { code = glyphHexCodeValue(); } catch(e) { code = 0x41; }
code = Number(code);
if (!Number.isFinite(code)) code = 0x41;
code = code & 0x7F;
if ($('glyphEditCode')) $('glyphEditCode').value = glyphHexV30(code);
if ($('glyphEditName')) $('glyphEditName').value = glyphCharLabelV27(code);
if ($('glyphCodeInputV27')) $('glyphCodeInputV27').value = '0x' + glyphHexV30(code) + ' (' + code + ')';
if ($('glyphCharInputV27')) $('glyphCharInputV27').value = glyphCharLabelV27(code);
const currentGrid = $('glyphGrid');
const hasBitmapCells = !!(currentGrid && currentGrid.children && currentGrid.children.length >= 64);
const rows = options.forceRows || (hasBitmapCells ? glyphRowsFromGrid() : glyphRowsForCodeV27(code));
if (!hasBitmapCells || options.forceRender) glyphRenderGrid(8, 8, rows);
else {
glyphPaintRowsV27($('glyphSelectedPreviewV27'), rows, 'glyph-preview-pixels small');
glyphPaintRowsV27($('glyphZoomPreviewV27'), rows, 'glyph-preview-pixels zoom');
glyphUpdateMatrixCellV28(code, rows);
}
glyphSelectCodeV27(code, false);
glyphSetToolV30('pencil');
glyphSetEditorStatus('Listo · edición 8x8 activa', true);
}
function buildGlyphV27Layout() {
if ($("glyphV29Page") || $("glyphV28Page")) {
const page = $("glyphV29Page");
if (page) page.querySelectorAll(".glyph-v27-tabs,.glyph-v27-section-title,#glyphViewCharsV32,#glyphViewHexV32").forEach(el => el.remove());
buildGlyphMatrixV27(false);
glyphEnsureGridEditorReadyV38();
return;
}
const section = document.createElement("section");
section.id = "glyphV29Page";
section.className = "glyph-v27-page";
section.innerHTML = `
<div id="glyphViewGridV32" class="glyph-view-panel glyph-v27-map active"><div id="glyphMatrixV27" class="glyph-map-grid"></div></div>
<div id="glyphEditorHomeV33"></div>
<div id="glyphEditorPanelV33" class="glyph-v27-editor">
<div><div class="glyph-v27-box-title">Glifo seleccionado</div><div class="glyph-v27-preview-box"><div id="glyphSelectedPreviewV27" class="glyph-preview-pixels small"></div></div></div>
<div class="glyph-v27-form">
<label>Código <input id="glyphCodeInputV27" value="0x41 (65)" readonly></label>
<label>Carácter <input id="glyphCharInputV27" value="A" maxlength="1"></label>
</div>
<div><div class="glyph-v27-box-title">Vista previa ampliada</div><div class="glyph-v27-preview-box"><div id="glyphZoomPreviewV27" class="glyph-preview-pixels zoom"></div></div></div>
<div><div class="glyph-v27-box-title">Mapa de bits (8x8)</div><div id="glyphBitmapEditorV27" class="glyph-v27-bitmap-shell"></div></div>
<div><div class="glyph-v27-box-title">Herramientas</div><div class="glyph-v27-tools"><button type="button" class="active" data-glyph-tool="pencil">✎ Lápiz</button><button type="button" data-glyph-tool="eraser">⌫ Borrador</button><button type="button" data-glyph-tool="fill">◩ Relleno</button><button type="button" data-glyph-tool="invert">▣ Invertir</button></div></div>
</div>
<div class="glyph-v27-actions">
<button id="btnGlyphSaveV27" type="button">💾 Guardar todos</button>
<button id="btnGlyphImportV27" type="button">⬆ Importar</button>
<button id="btnGlyphExportV27" type="button">⬇ Exportar</button>
<button id="btnGlyphRestoreV27" type="button">↩ Restaurar valores</button>
</div>
<div id="glyphFullscreenV29" class="glyph-fullscreen" aria-hidden="true" title="Presiona nuevamente para cerrar">
<button id="glyphFullscreenCloseV29" class="glyph-fullscreen-close" type="button" aria-label="Cerrar">×</button>
<div class="glyph-fullscreen-card">
<div class="glyph-fullscreen-subtitle">Vista a pantalla completa</div>
<div id="glyphFullscreenTitleV29" class="glyph-fullscreen-title">0x41 · A</div>
<div id="glyphFullscreenPixelsV29" class="glyph-fullscreen-pixels"></div>
<div class="glyph-fullscreen-hint">Presiona el glifo, el fondo o ESC para cerrar.</div>
</div>
</div>`;
const grid = document.querySelector(".app > .grid");
if (grid && grid.parentNode) grid.parentNode.insertBefore(section, grid);
buildGlyphMatrixV27();
const oldGrid = $("glyphGrid");
const mount = $("glyphBitmapEditorV27");
if (oldGrid && mount && oldGrid.parentNode !== mount) mount.appendChild(oldGrid);
const oldPreview = $("glyphPreview"); if (oldPreview) oldPreview.classList.add("glyph-v27-hidden");
const oldImport = $("glyphImportExportBlock"); if (oldImport) oldImport.classList.add("glyph-v27-hidden");
const oldEditor = $("glyphsPageBlock"); if (oldEditor) oldEditor.style.display = "none";
const file = $("glyphFile");
// V30: el input de importación no debe quedar dentro de un bloque display:none,
// porque algunos navegadores bloquean file.click() si el padre está oculto.
if (file && section && file.parentNode !== section) section.appendChild(file);
if (file && !file.dataset.v27ImportBound) {
file.dataset.v27ImportBound = "1";
file.addEventListener("change", () => { if (file.files && file.files.length && typeof importGlyphs === "function") importGlyphs(); });
}
glyphSwitchViewV32("grid");
document.querySelectorAll("[data-glyph-tool]").forEach(btn => {
if (btn.dataset.bound === "1") return;
btn.dataset.bound = "1";
btn.addEventListener("click", () => {
const tool = btn.getAttribute("data-glyph-tool") || "pencil";
if (tool === "fill") glyphFillGridV30(true);
else if (tool === "invert") glyphInvertGridV30();
else glyphSetToolV30(tool);
});
});
const charInput = $("glyphCharInputV27");
if (charInput && !charInput.dataset.bound) {
charInput.dataset.bound = "1";
charInput.addEventListener("change", () => {
const v = (charInput.value || "A").charAt(0);
const code = Math.max(0, Math.min(127, v.charCodeAt(0) || 65));
glyphSelectCodeV27(code, true);
});
}
const save = $("btnGlyphSaveV27"); if (save && !save.dataset.bound) { save.dataset.bound="1"; save.addEventListener("click", () => glyphSaveAllCurrentV34()); }
const imp = $("btnGlyphImportV27"); if (imp && !imp.dataset.bound) { imp.dataset.bound="1"; imp.addEventListener("click", () => { const f=$("glyphFile"); if (f && (!f.files || !f.files.length)) f.click(); else importGlyphs(); }); }
const exp = $("btnGlyphExportV27"); if (exp && !exp.dataset.bound) { exp.dataset.bound="1"; exp.addEventListener("click", () => exportGlyphs()); }
const restore = $("btnGlyphRestoreV27"); if (restore && !restore.dataset.bound) { restore.dataset.bound="1"; restore.addEventListener("click", () => glyphRestoreCurrent()); }
["glyphSelectedPreviewV27","glyphZoomPreviewV27"].forEach(id => {
const el = $(id);
if (el && !el.dataset.fullscreenBound) {
el.dataset.fullscreenBound = "1";
el.style.cursor = "zoom-in";
el.title = "Click para ver a pantalla completa";
el.addEventListener("dblclick", (ev) => ev.preventDefault());
el.addEventListener("click", () => {
const code = glyphHexCodeValue();
glyphFullscreenOpenV29(code, glyphRowsFromGrid());
});
}
});
glyphSelectCodeV27(0x41, false);
// V38: al entrar a Vista de cuadrícula, el card/editor de modificación
// queda visible inmediatamente y con el glifo A / 0x41 listo para editar.
const initialGlyphRowsV38 = glyphRowsForCodeV27(0x41);
glyphEnsureGridEditorReadyV38({ forceRows: initialGlyphRowsV38, forceRender: true });
glyphFullscreenBindV29();
setTimeout(() => {
refreshGlyphMatrixFromDeviceV30().finally(() => glyphEnsureGridEditorReadyV38());
}, 250);
}
function glyphFullscreenIsOpenV29(code) {
const overlay = $("glyphFullscreenV29");
return !!(overlay && overlay.classList.contains("open") && overlay.dataset.code === String(Number(code || 0) & 0x7F));
}
function glyphFullscreenPaintV29(rows) {
const target = $("glyphFullscreenPixelsV29");
if (!target) return;
target.innerHTML = "";
rows = (rows && rows.length) ? rows : glyphRowsForCodeV27(0x41);
for (let y=0; y<8; y++) {
const v = parseInt(rows[y] || "0", 16) || 0;
for (let x=0; x<8; x++) {
const p = document.createElement("i");
if (v & (1 << (7-x))) p.className = "on";
target.appendChild(p);
}
}
}
function glyphFullscreenOpenV29(code, rows) {
code = Number(code || 0) & 0x7F;
const overlay = $("glyphFullscreenV29");
if (!overlay) return;
const hex = code.toString(16).toUpperCase().padStart(2,"0");
const title = $("glyphFullscreenTitleV29");
if (title) title.textContent = "0x" + hex + " (" + code + ") · " + glyphCharLabelV27(code);
overlay.dataset.code = String(code);
glyphFullscreenPaintV29(rows && rows.length ? rows : glyphRowsForCodeV27(code));
overlay.classList.add("open");
overlay.setAttribute("aria-hidden", "false");
document.body.style.overflow = "hidden";
}
function glyphFullscreenCloseV29() {
const overlay = $("glyphFullscreenV29");
if (!overlay) return;
overlay.classList.remove("open");
overlay.setAttribute("aria-hidden", "true");
overlay.dataset.code = "";
document.body.style.overflow = "";
}
function glyphFullscreenBindV29() {
const overlay = $("glyphFullscreenV29");
if (!overlay || overlay.dataset.bound === "1") return;
overlay.dataset.bound = "1";
overlay.addEventListener("click", (ev) => {
if (ev.target === overlay || ev.target.id === "glyphFullscreenPixelsV29" || ev.target.closest("#glyphFullscreenPixelsV29")) glyphFullscreenCloseV29();
});
const close = $("glyphFullscreenCloseV29");
if (close) close.addEventListener("click", (ev) => { ev.preventDefault(); glyphFullscreenCloseV29(); });
document.addEventListener("keydown", (ev) => {
if (ev.key === "Escape") glyphFullscreenCloseV29();
});
}
function atrOpenCoverFullscreen(src, title) {
const overlay = $("coverFullscreenV34");
const img = $("coverFullscreenImgV34");
const t = $("coverFullscreenTitleV34");
if (!overlay || !img) return;
img.src = src || "";
if (t) t.textContent = title || "Carátula";
overlay.classList.add("open");
overlay.setAttribute("aria-hidden", "false");
document.body.style.overflow = "hidden";
}
function atrCloseCoverFullscreen() {
const overlay = $("coverFullscreenV34");
const img = $("coverFullscreenImgV34");
if (!overlay) return;
overlay.classList.remove("open");
overlay.setAttribute("aria-hidden", "true");
if (img) img.removeAttribute("src");
document.body.style.overflow = "";
}
function atrBindCoverFullscreenV34() {
const overlay = $("coverFullscreenV34");
if (!overlay || overlay.dataset.bound === "1") return;
overlay.dataset.bound = "1";
overlay.addEventListener("click", () => atrCloseCoverFullscreen());
document.addEventListener("keydown", (ev) => { if (ev.key === "Escape") atrCloseCoverFullscreen(); });
}
function setStatus(msg, ok = true) {
const dot = $("statusDot");
const text = $("statusMsg");
text.textContent = msg;
if (ok) dot.classList.remove("err");
else dot.classList.add("err");
}
function printerModeText(mode) {
const m = Number(mode || 1);
if (m === 2) return "Gateway Windows";
if (m === 4) return "IPP directo / JPEG estable";
return "IPP directo / JPEG estable";
}
function applyPrinterStatus(prn) {
if (!prn) return;
$("printerEnabled").checked = !!prn.enabled;
$("printerDev").value = String(prn.sioDev ?? 64);
$("printerMode").value = String(prn.mode ?? 1);
$("printerIp").value = prn.ip ?? "";
$("printerPort").value = prn.port ?? 9100;
$("printerGateway").value = prn.gateway ?? "";
$("printerName").value = prn.name ?? "";
$("printerAtascii").checked = !!prn.atascii;
$("printerCrLf").checked = !!prn.crlf;
$("printerCut40").checked = !!prn.cut40;
if ($("printerProfile")) $("printerProfile").value = String(prn.virtualProfile ?? 0);
if ($("printerFont")) $("printerFont").value = String(prn.virtualFont ?? 0);
if ($("printerFontScale")) $("printerFontScale").value = String(prn.fontScale ?? 0);
if ($("printerOrientation")) $("printerOrientation").value = String(prn.pageOrientation ?? 0);
if ($("printerPaper")) $("printerPaper").value = String(prn.paperSize ?? 0);
if ($("printerCompose")) $("printerCompose").value = String(prn.composeMode ?? 1);
if ($("printerRenderQuality")) $("printerRenderQuality").value = String(prn.renderQuality ?? 0);
if ($("printerAutoPrint")) $("printerAutoPrint").value = String(prn.autoPrintSpool ? 1 : 0);
if ($("printerAutoDelay")) $("printerAutoDelay").value = String(prn.autoPrintDelayMs ?? 5000);
if ($("glyphStatus")) {
$("glyphStatus").textContent = (prn.glyphJsonLoaded ? "JSON OK" : "JSON OFF") + " · " + (prn.fontName || "ATASCII matriz 8x8");
$("glyphStatus").classList.toggle("ok", !!prn.glyphJsonLoaded);
}
$("printerRawFF").checked = prn.rawTestFormFeed !== 0;
$("printerRawDelay").value = prn.rawCloseDelayMs ?? 500;
$("printerStaEnabled").checked = !!prn.staEnabled;
$("printerStaSsid").value = prn.staSsid ?? "";
$("printerStaPass").value = "";
const staSummary = $("printerStaSummary");
if (staSummary) {
const staTxt = prn.staConnected ? `STA ${prn.staIp || "OK"}` : (prn.staEnabled ? `STA ${prn.wifiStatus || "OFF"}` : "STA OFF");
staSummary.textContent = staTxt;
staSummary.classList.toggle("ok", !!prn.staConnected);
staSummary.classList.toggle("bad", !!prn.staEnabled && !prn.staConnected);
}
const status = $("printerStatus");
status.textContent = prn.enabled ? "READY" : "OFF";
status.classList.toggle("ok", !!prn.enabled);
status.classList.toggle("bad", !prn.enabled);
const devNum = Number(prn.sioDev ?? 64);
$("printerDevPill").textContent = "DEV 0x" + devNum.toString(16).toUpperCase().padStart(2, "0");
$("printerOutSummary").textContent = printerModeText(prn.mode);
$("printerIpSummary").textContent = prn.ip || "—";
$("printerPortSummary").textContent = String(prn.port ?? "—");
$("printerLinesSummary").textContent = `${prn.linesRx || 0} recibidas / ${prn.linesOk || 0} OK / ${prn.linesErr || 0} ERR`;
$("printerLastText").textContent = prn.lastText || "—";
const log = [];
log.push(`[PRN] enabled=${prn.enabled ? 1 : 0} dev=0x${devNum.toString(16).toUpperCase().padStart(2, "0")}`);
log.push(`[PRN] salida=${printerModeText(prn.mode)} ip=${prn.ip || "—"}:${prn.port || "—"}`);
log.push(`[PRN] perfil virtual=${prn.profileName || "—"} preset=${prn.profileColumns || 0}x${prn.profileRows || 0}`);
log.push(`[PRN-LAYOUT] papel=${prn.paperName || "A4"} media=${prn.paperMedia || "—"} efectivo=${prn.effectiveColumns || 0}x${prn.effectiveRows || 0} flush=${prn.effectiveFlushRows || 0} orient=${prn.orientationName || "Auto"} letra=${prn.fontScale || 12}pt`);
log.push(`[PRN-COMPOSE] modo=${prn.composeModeName || "Composición extendida"}`);
log.push(`[PRN-QUALITY] caracteres=${prn.renderQualityName || "Compatibilidad JPEG estable"}`);
log.push(`[PRN-AUTO] imprimir=${prn.autoPrintSpool ? 1 : 0} espera=${prn.autoPrintDelayMs || 5000}ms autoReq=${prn.autoPrintRequested ? 1 : 0}`);
log.push(`[PRN] fuente=${prn.fontName || "ATASCII matriz 8x8"} json=${prn.glyphJsonLoaded ? 1 : 0} ${prn.glyphJsonStatus || ""}`);
log.push(`[PRN] wifi sta=${prn.staEnabled ? 1 : 0} status=${prn.wifiStatus || "—"} ip=${prn.staIp || "—"}`);
log.push(`[PRN] líneas rx=${prn.linesRx || 0} ok=${prn.linesOk || 0} err=${prn.linesErr || 0}`);
log.push(`[PRN-SPOOL] pendiente=${prn.bufferPending ? 1 : 0} líneas=${prn.bufferLines || 0} bytes=${prn.bufferBytes || 0} páginas=${prn.spoolPagesPrinted || 0}`);
if (prn.lastBytes) log.push(`[PRN] último envío RAW: ${prn.lastBytes} bytes`);
if (prn.lastText) log.push(`[PRN] última línea: ${prn.lastText}`);
if (prn.lastError) log.push(`[PRN] último error: ${prn.lastError}`);
log.push(`[PRN-DIAG] status=${prn.diagStatus || 0} write=${prn.diagWrite || 0} ok=${prn.diagWriteOk || 0} qSent=${prn.diagQSent || 0} qDrop=${prn.diagQDrop || 0}`);
log.push(`[PRN-DIAG] toData=${prn.diagTimeoutData || 0} toChk=${prn.diagTimeoutChk || 0} chkErr=${prn.diagChecksumErr || 0} unsup=${prn.diagUnsupported || 0}`);
$("printerLog").textContent = log.join("\n");
}
function collectPrinterConfig() {
return {
enabled: $("printerEnabled").checked ? 1 : 0,
dev: Number($("printerDev").value) || 64,
mode: Number($("printerMode").value) || 2,
ip: $("printerIp").value || "",
port: Number($("printerPort").value) || 9100,
gateway: $("printerGateway").value || "",
name: $("printerName").value || "",
atascii: $("printerAtascii").checked ? 1 : 0,
crlf: $("printerCrLf").checked ? 1 : 0,
cut40: $("printerCut40").checked ? 1 : 0,
profile: Number($("printerProfile").value) || 0,
font: Number($("printerFont").value) || 0,
fontScale: Number($("printerFontScale").value) || 0,
orientation: Number($("printerOrientation").value) || 0,
paper: Number($("printerPaper").value) || 0,
compose: Number($("printerCompose").value),
renderQuality: 1,
autoPrint: Number($("printerAutoPrint").value) || 0,
autoDelay: Number($("printerAutoDelay").value) || 5000,
rawFF: $("printerRawFF").checked ? 1 : 0,
rawDelay: Number($("printerRawDelay").value) || 0,
staEnabled: $("printerStaEnabled").checked ? 1 : 0,
staSsid: $("printerStaSsid").value || "",
staPass: $("printerStaPass").value || ""
};
}
function printerUrlFromConfig(prn) {
return `/set_printer?enabled=${prn.enabled}` +
`&dev=${encodeURIComponent(prn.dev)}` +
`&mode=${encodeURIComponent(prn.mode)}` +
`&ip=${encodeURIComponent(prn.ip)}` +
`&port=${encodeURIComponent(prn.port)}` +
`&gateway=${encodeURIComponent(prn.gateway)}` +
`&name=${encodeURIComponent(prn.name)}` +
`&atascii=${prn.atascii}` +
`&crlf=${prn.crlf}` +
`&cut40=${prn.cut40}` +
`&profile=${encodeURIComponent(prn.profile)}` +
`&font=${encodeURIComponent(prn.font)}` +
`&fontScale=${encodeURIComponent(prn.fontScale)}` +
`&orientation=${encodeURIComponent(prn.orientation)}` +
`&paper=${encodeURIComponent(prn.paper)}` +
`&compose=${encodeURIComponent(prn.compose)}` +
`&renderQuality=${encodeURIComponent(prn.renderQuality)}` +
`&autoPrint=${encodeURIComponent(prn.autoPrint)}` +
`&autoDelay=${encodeURIComponent(prn.autoDelay)}` +
`&rawFF=${prn.rawFF}` +
`&rawDelay=${encodeURIComponent(prn.rawDelay)}` +
`&staEnabled=${prn.staEnabled}` +
`&staSsid=${encodeURIComponent(prn.staSsid)}` +
`&staPass=${encodeURIComponent(prn.staPass)}`;
}
async function savePrinterConfig() {
try {
setStatus("Guardando configuración de impresora...", true);
const r = await fetch(printerUrlFromConfig(collectPrinterConfig()));
if (!r.ok) throw new Error("Error en /set_printer (" + r.status + ")");
await loadStatus();
setStatus("Configuración de impresora guardada.", true);
} catch (e) {
console.error(e);
setStatus("Error al guardar impresora: " + e.message, false);
}
}
async function testPrinter() {
try {
setStatus("Enviando prueba de impresión...", true);
const r = await fetch("/printer_test");
const body = await r.text();
if (!r.ok) throw new Error("Error en /printer_test (" + r.status + "): " + body);
await loadStatus();
setStatus("Prueba de impresión enviada.", true);
} catch (e) {
console.error(e);
setStatus("Error en prueba de impresión: " + e.message, false);
}
}
async function testIppPrinter() {
try {
setStatus("Enviando prueba IPP directa...", true);
const r = await fetch("/printer_ipp_test");
const body = await r.text();
if (!r.ok) throw new Error("Error en /printer_ipp_test (" + r.status + "): " + body);
await loadStatus();
setStatus("Prueba IPP enviada: " + body, true);
} catch (e) {
console.error(e);
setStatus("Error en prueba IPP: " + e.message, false);
}
}
async function testVirtualPrinter() {
try {
setStatus("Generando prueba ATASCII/JPEG...", true);
await savePrinterConfig();
const r = await fetch("/printer_virtual_test");
const body = await r.text();
if (!r.ok) throw new Error("Error en /printer_virtual_test (" + r.status + "): " + body);
await loadStatus();
setStatus("Prueba virtual enviada: " + body, true);
} catch (e) {
console.error(e);
setStatus("Error en prueba virtual: " + e.message, false);
}
}
async function testAtasciiTablePrinter() {
try {
setStatus("Generando tabla ATASCII/JPEG...", true);
await savePrinterConfig();
const r = await fetch("/printer_atascii_table_test");
const body = await r.text();
if (!r.ok) throw new Error("Error en /printer_atascii_table_test (" + r.status + "): " + body);
await loadStatus();
setStatus("Tabla ATASCII enviada: " + body, true);
} catch (e) {
console.error(e);
setStatus("Error en tabla ATASCII: " + e.message, false);
}
}
async function printAtariBuffer() {
try {
setStatus("Imprimiendo buffer Atari P:...", true);
await savePrinterConfig();
const r = await fetch("/printer_buffer_print");
const body = await r.text();
if (!r.ok) throw new Error("Error en /printer_buffer_print (" + r.status + "): " + body);
await loadStatus();
setStatus("Buffer Atari enviado: " + body, true);
} catch (e) {
console.error(e);
setStatus("Error al imprimir buffer Atari: " + e.message, false);
}
}
async function exportGlyphs() {
window.open("/glyphs_export", "_blank");
}
function glyphCodeFromAnyV31(value) {
if (typeof value === "number") return Number.isFinite(value) ? (value & 0x7F) : NaN;
let s = String(value ?? "").trim();
if (!s) return NaN;
if (/^0x[0-9a-f]+$/i.test(s)) return parseInt(s, 16) & 0x7F;
// En el JSON ATASCII usamos códigos de 2 dígitos en HEX: "41" = A.
if (/^[0-9a-f]{1,2}$/i.test(s)) return parseInt(s, 16) & 0x7F;
const n = Number(s);
return Number.isFinite(n) ? (n & 0x7F) : NaN;
}
function glyphRowsFromAnyV31(value) {
if (!Array.isArray(value)) return [];
return glyphNormalizeRowsV30(value.map(v => String(v ?? "00")));
}
function compactGlyphImportFromText(txt) {
const lines = [];
const seen = new Set();
const pushGlyph = (codeAny, rowsAny) => {
const codeNum = glyphCodeFromAnyV31(codeAny);
if (!Number.isFinite(codeNum) || codeNum < 0 || codeNum > 127) return;
const code = glyphHexV30(codeNum);
if (seen.has(code)) return;
const rows = glyphRowsFromAnyV31(rowsAny);
if (rows.length === 8) {
lines.push(code + ":" + rows.join(","));
seen.add(code);
}
};
try {
const json = JSON.parse(txt);
const fonts = Array.isArray(json.fonts) ? json.fonts : [];
fonts.forEach(font => {
const glyphs = Array.isArray(font.glyphs) ? font.glyphs : [];
glyphs.forEach(g => pushGlyph(g.code ?? g.codeHex ?? g.id, g.rowsHex ?? g.rows));
});
if (lines.length) return lines.join("\n");
} catch (e) {
console.warn("Importación JSON directa falló, usando parser tolerante", e);
}
// Fallback tolerante para archivos con formato raro o parcialmente dañado.
const re = /"code"\s*:\s*(?:"([^"\\]*(?:\\.[^"\\]*)*)"|(\d+))[\s\S]*?"(?:rowsHex|rows)"\s*:\s*\[([\s\S]*?)\]/g;
let m;
while ((m = re.exec(txt)) !== null) {
const rawCode = m[1] || Number(m[2]);
const rows = [];
const arr = m[3];
const rh = /"([^"\\]*(?:\\.[^"\\]*)*)"|(\d+)/g;
let rm;
while ((rm = rh.exec(arr)) !== null && rows.length < 8) rows.push(glyphRowToHex8V53(rm[1] !== undefined ? rm[1] : Number(rm[2])));
pushGlyph(rawCode, rows);
}
return lines.join("\n");
}
async function importGlyphs() {
try {
const f = $("glyphFile").files && $("glyphFile").files[0];
if (!f) throw new Error("Selecciona atascii_glyphs.json primero");
const txt = await f.text();
const compact = compactGlyphImportFromText(txt);
if (!compact || compact.length < 8) throw new Error("No se encontraron glifos rowsHex válidos en el archivo");
const importedLines = compact.split(/\n+/).filter(Boolean);
importedLines.forEach(line => {
const parts = line.split(":");
if (parts.length >= 2) glyphCacheRowsV30(parseInt(parts[0],16), parts[1].split(","));
});
setStatus("Importando glifos ATASCII...", true);
const r = await fetch("/glyphs_import_compact", {
method: "POST",
headers: { "Content-Type": "text/plain; charset=utf-8", "Cache-Control": "no-store" },
body: compact
});
const body = await r.text();
if (!r.ok) throw new Error(body || ("HTTP " + r.status));
await loadStatus();
setStatus("Glifos importados: " + body, true);
if ($("glyphFile")) $("glyphFile").value = "";
await refreshGlyphMatrixFromDeviceV30();
buildGlyphMatrixV27(true);
if (importedLines.length) {
const firstCode = parseInt(importedLines[0].split(":")[0], 16) & 0x7F;
glyphSelectCodeV27(firstCode, true);
} else {
await glyphLoadCurrent();
}
} catch (e) {
console.error(e);
setStatus("Error importando glifos: " + e.message, false);
}
}
function glyphSetEditorStatus(msg, ok = true) {
const el = $("glyphEditorStatus");
if (!el) return;
el.textContent = msg;
el.classList.toggle("ok", !!ok);
el.classList.toggle("bad", !ok);
}
function glyphHexCodeValue() {
let v = ($("glyphEditCode").value || "41").trim().toUpperCase().replace(/^0X/, "");
v = v.replace(/[^0-9A-F]/g, "");
if (v.length === 0) v = "00";
if (v.length > 2) v = v.slice(-2);
let n = parseInt(v, 16);
if (!Number.isFinite(n)) n = 0x41;
n = n & 0x7F;
$("glyphEditCode").value = n.toString(16).toUpperCase().padStart(2, "0");
return n;
}
function glyphCurrentDimensions() {
return { font: 0, width: 8, height: 8 };
}
function glyphRenderGrid(width, height, rowsHex) {
const grid = $("glyphGrid");
if (!grid) return;
grid.innerHTML = "";
grid.style.gridTemplateColumns = `repeat(${width}, 18px)`;
for (let y = 0; y < height; y++) {
const rowVal = parseInt((rowsHex && rowsHex[y]) || "0", 16) || 0;
for (let x = 0; x < width; x++) {
const mask = 1 << (width - 1 - x);
const cell = document.createElement("div");
cell.className = "glyph-cell" + ((rowVal & mask) ? " on" : "");
cell.dataset.x = String(x);
cell.dataset.y = String(y);
cell.addEventListener("pointerdown", (ev) => {
ev.preventDefault();
ev.stopPropagation();
glyphPointerPaintingV31 = true;
glyphPaintTargetStateV33 = glyphToolV30 === "pencil" ? !cell.classList.contains("on") : false;
glyphApplyGridCellV30(cell, false);
});
cell.addEventListener("pointerenter", (ev) => {
if (!glyphPointerPaintingV31) return;
ev.preventDefault();
ev.stopPropagation();
glyphApplyGridCellV30(cell, true);
});
cell.addEventListener("click", (ev) => {
ev.preventDefault();
ev.stopPropagation();
});
cell.addEventListener("keydown", (ev) => {
if (ev.key === "Enter" || ev.key === " ") {
ev.preventDefault();
ev.stopPropagation();
glyphPaintTargetStateV33 = glyphToolV30 === "pencil" ? !cell.classList.contains("on") : false;
glyphApplyGridCellV30(cell, false);
}
});
grid.appendChild(cell);
}
}
glyphUpdatePreview();
}
function glyphRowsFromGrid() {
const { width, height } = glyphCurrentDimensions();
const rows = [];
const cells = $("glyphGrid") ? Array.from($("glyphGrid").children) : [];
const hexLen = Math.ceil(width / 4);
for (let y = 0; y < height; y++) {
let v = 0;
for (let x = 0; x < width; x++) {
const idx = y * width + x;
if (cells[idx] && cells[idx].classList.contains("on")) v |= (1 << (width - 1 - x));
}
rows.push(v.toString(16).toUpperCase().padStart(hexLen, "0"));
}
return rows;
}
function glyphUpdatePreview() {
const rows = glyphRowsFromGrid();
const code = glyphHexCodeValue();
glyphCacheRowsV30(code, rows);
const p = $("glyphPreview");
if (p) p.textContent = "rowsHex: [" + rows.map(r => '"' + r + '"').join(",") + "]";
if (typeof glyphPaintRowsV27 === "function") {
glyphPaintRowsV27($("glyphSelectedPreviewV27"), rows, "glyph-preview-pixels small");
glyphPaintRowsV27($("glyphZoomPreviewV27"), rows, "glyph-preview-pixels zoom");
}
glyphUpdateMatrixCellV28(code, rows);
if (glyphFullscreenIsOpenV29(code)) glyphFullscreenPaintV29(rows);
}
async function glyphLoadCurrent() {
try {
const { font } = glyphCurrentDimensions();
const code = glyphHexCodeValue();
glyphSetEditorStatus("Cargando...", true);
const r = await fetch(`/glyph_get?font=${font}&code=0x${glyphHexV30(code)}`);
const body = await r.text();
if (!r.ok) throw new Error(body || `HTTP ${r.status}`);
const g = JSON.parse(body);
if ($("glyphEditName")) $("glyphEditName").value = g.name || ("0x" + $("glyphEditCode").value);
const rows = glyphCacheRowsV30(code, g.rowsHex || []);
glyphRenderGrid(g.width || 8, g.height || 8, rows);
if (typeof glyphSyncV27 === "function") glyphSyncV27(code, Object.assign({}, g, { rowsHex: rows }));
glyphSetEditorStatus((g.valid ? "Personalizado" : "Base") + " · " + (g.source || ""), true);
} catch (e) {
console.error(e);
const code = glyphHexCodeValue();
const rows = glyphRowsForCodeV27(code);
glyphRenderGrid(8, 8, rows);
if (typeof glyphSyncV27 === "function") glyphSyncV27(code, { rowsHex: rows, name: glyphCharLabelV27(code) });
glyphSetEditorStatus("Base local", true);
setStatus("Glifo cargado desde base local porque el endpoint no respondió: " + e.message, false);
}
}
function glyphCompactFromCacheV34() {
try {
const currentCode = glyphHexCodeValue();
glyphCacheRowsV30(currentCode, glyphRowsFromGrid());
} catch(e) {}
const lines = [];
for (let code=0; code<128; code++) {
const rows = glyphNormalizeRowsV30(glyphRowsFromCacheV30(code) || glyphRowsForCodeV27(code));
lines.push(glyphHexV30(code) + ":" + rows.join(","));
}
return lines.join("\n");
}
async function glyphSaveAllCurrentV34() {
const compact = glyphCompactFromCacheV34();
if (!compact || compact.length < 8) {
return glyphSaveCurrent();
}
try {
glyphSetEditorStatus("Guardando glifos...", true);
const r = await fetch("/glyphs_import_compact", {
method:"POST",
headers:{ "Content-Type":"text/plain; charset=utf-8", "Cache-Control":"no-store" },
body:compact
});
const body = await r.text();
if (!r.ok) throw new Error(body || ("HTTP " + r.status));
await refreshGlyphMatrixFromDeviceV30();
buildGlyphMatrixV27(true);
glyphEnsureGridEditorReadyV38({ forceRender:true });
glyphSetEditorStatus("Glifos guardados · 128/128", true);
setStatus("Glifos actualizados/persistidos: " + body, true);
} catch(e) {
console.error(e);
glyphSetEditorStatus("Error", false);
setStatus("Error guardando glifos: " + e.message, false);
}
}
async function glyphSaveCurrent() {
try {
const { font } = glyphCurrentDimensions();
const code = glyphHexCodeValue();
const rows = glyphRowsFromGrid();
const name = encodeURIComponent(($("glyphEditName").value || "").trim());
glyphSetEditorStatus("Guardando...", true);
const r = await fetch(`/glyph_set?font=${font}&code=0x${glyphHexV30(code)}&name=${name}&rows=${encodeURIComponent(rows.join(","))}`);
const body = await r.text();
if (!r.ok) throw new Error(body || `HTTP ${r.status}`);
await loadStatus();
glyphCacheRowsV30(code, rows);
glyphSetEditorStatus("Guardado", true);
glyphUpdateMatrixCellV28(code, rows);
setStatus("Glifo guardado: " + body, true);
} catch (e) {
console.error(e);
glyphSetEditorStatus("Error", false);
setStatus("Error guardando glifo: " + e.message, false);
}
}
async function glyphRestoreCurrent() {
try {
const { font } = glyphCurrentDimensions();
const code = glyphHexCodeValue();
glyphSetEditorStatus("Restaurando...", true);
const r = await fetch(`/glyph_restore?font=${font}&code=0x${glyphHexV30(code)}`);
const body = await r.text();
if (!r.ok) throw new Error(body || `HTTP ${r.status}`);
await loadStatus();
await glyphLoadCurrent();
await refreshGlyphMatrixFromDeviceV30();
setStatus("Glifo restaurado: " + body, true);
} catch (e) {
console.error(e);
glyphSetEditorStatus("Error", false);
setStatus("Error restaurando glifo: " + e.message, false);
}
}
async function clearPrinterQueue() {
try {
const r = await fetch("/printer_clear");
if (!r.ok) throw new Error("Error en /printer_clear (" + r.status + ")");
await loadStatus();
setStatus("Estado de impresora limpiado.", true);
} catch (e) {
console.error(e);
setStatus("Error al limpiar impresora: " + e.message, false);
}
}
function renderDrives(drives) {
const container = $("drives");
if (!container) return;
window.lastStatusDrivesV32 = Array.isArray(drives) ? drives.slice() : [];
const base = Array.isArray(drives) ? drives.slice() : [];
const byDev = {};
base.forEach(d => { if (d && d.dev) byDev[String(d.dev).toUpperCase()] = d; });
const mask = visibleUnitMaskFromStatus(lastDriveVisibility || {});
const visible = [];
for (let i=1; i<=UI_MAX_UNITS; i++) {
if (unitVisible(mask, i)) {
const dev = "D" + i;
visible.push(Object.assign({ dev, present:false, supports256:false, prefetch:false, prefetchSectors:3, autoEnabled:false }, byDev[dev] || {}));
}
}
container.innerHTML = "";
const onlineCount = visible.filter(d => !!d.present).length;
const prefetchCount = visible.filter(d => !!d.prefetch).length;
if ($("diskOnlineCount")) $("diskOnlineCount").textContent = `${onlineCount} online`;
if ($("diskVisibleCount")) $("diskVisibleCount").textContent = `${visible.length || 0} visibles`;
if ($("diskPrefetchCount")) $("diskPrefetchCount").textContent = `${prefetchCount} activo${prefetchCount === 1 ? "" : "s"}`;
if ($("homeDriveOnline")) $("homeDriveOnline").textContent = `${onlineCount} online`;
if ($("homeDriveVisible")) $("homeDriveVisible").textContent = visible.length ? `D1-D${visible.length}` : "—";
if ($("diskProfileState")) {
const p = ($("autoProfile") && $("autoProfile").value) ? $("autoProfile").value : "1";
$("diskProfileState").textContent = (p === "0") ? "Conservador" : (p === "2" ? "Agresivo" : "Normal");
}
if (!visible.length) {
container.innerHTML = "<small>No hay disqueteras visibles aún.</small>";
return;
}
visible.forEach(d => {
const div = document.createElement("div");
div.className = "drive-item atr-mount-item disk-unit-row";
const present = !!d.present;
const supports256 = !!d.supports256;
const prefetch = !!d.prefetch;
const prefetchSectors = Number(d.prefetchSectors || 3);
const autoEnabled = (typeof d.autoEnabled === "boolean") ? d.autoEnabled : false;
const mac = d.mac || "";
div.innerHTML = `
<div class="drive-header">
<div class="atr-mount-title">
<span class="drive-name">${escapeHtml(d.dev || "DX")}</span>
<span class="pill ${present ? "ok" : "bad"}">${present ? "ONLINE" : "OFFLINE"}</span>
<span class="pill">${supports256 ? "DD 256B" : "SD 128B"}</span>
</div>
<span class="pill ${autoEnabled ? "ok" : ""}">${autoEnabled ? "Auto ON" : "Auto OFF"}</span>
</div>
<div class="drive-body"></div>
<div class="disk-row-controls">
<div class="control-cluster">
<label class="switch">
<input type="checkbox" data-dev="${escapeHtml(d.dev)}" class="pf-toggle" ${prefetch ? "checked" : ""}>
<span class="switch-slider"></span><span>Prefetch</span>
</label>
<span style="font-size:0.75rem;">Sectores</span>
<input type="number" min="0" max="16" step="1" value="${prefetchSectors}" class="small-input pf-count" data-dev="${escapeHtml(d.dev)}" />
</div>
<div class="control-cluster">
<label class="switch">
<input type="checkbox" data-dev="${escapeHtml(d.dev)}" class="auto-toggle" ${autoEnabled ? "checked" : ""}>
<span class="switch-slider"></span><span>Auto-ajuste</span>
</label>
</div>
</div>
`;
container.appendChild(div);
});
}
function renderDevices(devices) {
const container = $("devices");
if (!container) return;
container.innerHTML = "";
if (!devices || devices.length === 0) {
container.innerHTML = "<small>No hay equipos detectados aún.</small>";
return;
}
devices.forEach(dev => {
const div = document.createElement("div");
div.className = "drive-item device-item";
const mac = dev.mac || "—";
const physical = dev.physical || "—";
const logical = dev.logical || "—";
const present = !!dev.present;
const supports256 = !!dev.supports256;
div.innerHTML = `
<div class="drive-header">
<div class="drive-name" style="font-size:0.9rem;">${mac}</div>
<div style="display:flex; gap:4px; align-items:center;">
<span class="pill ${present ? "ok" : "bad"}">${present ? "ONLINE" : "OFFLINE"}</span>
<span class="pill">${supports256 ? "DD 256B" : "SD 128B"}</span>
</div>
</div>
<div class="drive-body">
<div><span class="label">Unidad física XF: </span><span class="value">${physical}</span></div>
<div><span class="label">Unidad lógica asignada: </span><span class="value">${logical}</span></div>
</div>
<div class="drive-prefetch">
<span style="font-size:0.75rem;">Asignar como:</span>
<select class="small-input" data-mac="${mac}">
<option value="D1">D1</option>
<option value="D2">D2</option>
<option value="D3">D3</option>
<option value="D4">D4</option>
</select>
<button class="secondary">Asignar</button>
</div>
`;
const sel = div.querySelector("select");
const btn = div.querySelector("button");
if (sel && logical && logical !== "—") sel.value = logical;
if (btn && sel) {
btn.addEventListener("click", async () => {
try {
const to = sel.value;
setStatus("Guardando mapeo lógico...", true);
const r = await fetch(`/set_logic?mac=${encodeURIComponent(mac)}&to=${encodeURIComponent(to)}`);
if (!r.ok) {
const t = await r.text();
throw new Error(t || `HTTP ${r.status}`);
}
await loadStatus();
setStatus("Mapeo lógico actualizado.", true);
} catch (e) {
setStatus("Error al mapear: " + e.message, false);
}
});
}
container.appendChild(div);
});
}
function updateHomeDashboard(data) {
try {
if (!data) return;
if ($("homeSysState")) {
const hasDevices = Array.isArray(data.devices) && data.devices.some(d => d && d.present);
$("homeSysState").textContent = hasDevices ? "ONLINE" : "LISTO";
$("homeSysState").className = "pill ok";
}
if ($("homeWifiIp")) {
const ip = data.ip || data.wifiIp || data.localIp || data.apIp || "192.168.4.1";
$("homeWifiIp").textContent = "IP: " + ip;
}
const drives = Array.isArray(data.drives) ? data.drives : [];
const online = drives.filter(d => d && d.present).length;
if ($("homeDriveOnline")) $("homeDriveOnline").textContent = online + " online";
if ($("homeDriveVisible")) {
const visible = (typeof visibleUnitMaskFromStatus === "function") ? visibleUnitMaskFromStatus(lastDriveVisibility || {}) : 0x0F;
let maxVisible = 0;
for (let i=1; i<=UI_MAX_UNITS; i++) if (unitVisible(visible, i)) maxVisible = i;
$("homeDriveVisible").textContent = maxVisible > 0 ? ("D1-D" + maxVisible) : "D1-D4";
}
} catch(e) { console.warn("home dashboard", e); }
}
function updateHomeLibraryCountFromAtr(data) {
try {
const files = data && Array.isArray(data.files) ? data.files.length : null;
if ($("homeLibraryCount") && files !== null) $("homeLibraryCount").textContent = files + " archivo" + (files === 1 ? "" : "s");
} catch(e) {}
}
async function loadStatus() {
try {
setStatus("Cargando estado...", true);
const res = await fetch("/api/status?ts=" + Date.now(), { cache:"no-store" });
if (!res.ok) throw new Error("HTTP " + res.status);
const data = await res.json();
updateHomeDashboard(data);
renderDrives(data.drives || []);
renderDevices(data.devices || []);
if (data.timings) {
$("ackToComplete").value  = data.timings.ackToComplete ?? "";
$("completeToData").value = data.timings.completeToData ?? "";
$("dataToChk").value      = data.timings.dataToChk ?? "";
$("chunkDelay").value     = data.timings.chunkDelay ?? "";
if (typeof data.timings.autoProfile !== "undefined") {
$("autoProfile").value = String(data.timings.autoProfile);
}
}
if (data.comm) {
$("uartBaud").value = String(data.comm.uartBaud ?? "460800");
$("rpSioBaud").value  = String(data.comm.rpSioBaud ?? "19200");
$("xfSioBaud").value  = String(data.comm.xfSioBaud ?? "19200");
$("netDelayUs").value = data.comm.netDelayUs ?? "0";
}
if (data.verify) {
$("verAll").checked  = !!data.verify.all;
$("verBoot").checked = !!data.verify.boot;
$("verVtoc").checked = !!data.verify.vtoc;
$("ver57").checked   = !!data.verify.w57;
}
if (data.printer) {
applyPrinterStatus(data.printer);
}
setStatus("Estado actualizado.");
} catch (e) {
console.error(e);
setStatus("Error al obtener estado: " + e.message, false);
}
}
function collectConfig() {
const drives = [];
document.querySelectorAll("#drives .drive-item").forEach(div => {
const devName = div.querySelector(".drive-name").textContent.trim(); // "D1"
const togglePf = div.querySelector(".pf-toggle");
const inputCount = div.querySelector(".pf-count");
const autoToggle = div.querySelector(".auto-toggle");
let sectors = inputCount ? Number(inputCount.value) || 0 : 0;
const pfEnabled = togglePf ? togglePf.checked : false;
if (!pfEnabled) sectors = 0;
const autoEnabled = autoToggle ? autoToggle.checked : true;
drives.push({
dev: devName,
prefetch: sectors > 0,
prefetchSectors: sectors,
autoEnabled: autoEnabled
});
});
const timings = {
ackToComplete:  Number($("ackToComplete").value)  || 0,
completeToData: Number($("completeToData").value) || 0,
dataToChk:      Number($("dataToChk").value)      || 0,
chunkDelay:     Number($("chunkDelay").value)     || 0
};
const autoProfile = Number($("autoProfile").value) || 1;
const comm = {
uart:   Number($("uartBaud").value) || 0,
sio_rp: Number($("rpSioBaud").value) || 0,
sio_xf: Number($("xfSioBaud").value) || 0,
net_us: Number($("netDelayUs").value) || 0
};
const verify = {
v_all:  $("verAll").checked ? 1 : 0,
v_boot: $("verBoot").checked ? 1 : 0,
v_vtoc: $("verVtoc").checked ? 1 : 0,
v_57:   $("ver57").checked ? 1 : 0
};
return { drives, timings, autoProfile, comm, verify, printer: collectPrinterConfig() };
}
function mapPrefetchByDev(drives) {
const out = { D1: 0, D2: 0, D3: 0, D4: 0 };
drives.forEach(d => {
const dev = d.dev;
if (out.hasOwnProperty(dev)) {
out[dev] = d.prefetchSectors || 0;
}
});
return out;
}
function mapAutoByDev(drives) {
const out = { D1: 1, D2: 1, D3: 1, D4: 1 };
drives.forEach(d => {
const dev = d.dev;
if (out.hasOwnProperty(dev)) {
out[dev] = d.autoEnabled ? 1 : 0;
}
});
return out;
}
async function saveConfig() {
try {
const cfg = collectConfig();
const pf  = mapPrefetchByDev(cfg.drives);
const auto = mapAutoByDev(cfg.drives);
setStatus("Guardando configuración...", true);
const urlTiming =
`/set_timing?t_ack=${cfg.timings.ackToComplete}` +
`&t_comp=${cfg.timings.completeToData}` +
`&t_chk=${cfg.timings.dataToChk}` +
`&t_chunk=${cfg.timings.chunkDelay}`;
const urlPrefetch =
`/set_prefetch?pf1=${pf.D1}&pf2=${pf.D2}&pf3=${pf.D3}&pf4=${pf.D4}`;
const urlAuto =
`/set_auto?d1=${auto.D1}&d2=${auto.D2}&d3=${auto.D3}&d4=${auto.D4}` +
`&profile=${cfg.autoProfile}`;
const urlComm =
`/set_comm?uart=${cfg.comm.uart}` +
`&sio_rp=${cfg.comm.sio_rp}` +
`&sio_xf=${cfg.comm.sio_xf}` +
`&net_us=${cfg.comm.net_us}`;
const urlVerify =
`/set_verify?v_all=${cfg.verify.v_all}` +
`&v_boot=${cfg.verify.v_boot}` +
`&v_vtoc=${cfg.verify.v_vtoc}` +
`&v_57=${cfg.verify.v_57}`;
const urlPrinter = printerUrlFromConfig(cfg.printer);
const r1 = await fetch(urlTiming);
if (!r1.ok) throw new Error("Error en /set_timing (" + r1.status + ")");
const r2 = await fetch(urlPrefetch);
if (!r2.ok) throw new Error("Error en /set_prefetch (" + r2.status + ")");
const r3 = await fetch(urlAuto);
if (!r3.ok) throw new Error("Error en /set_auto (" + r3.status + ")");
const r4 = await fetch(urlComm);
if (!r4.ok) throw new Error("Error en /set_comm (" + r4.status + ")");
const r5 = await fetch(urlVerify);
if (!r5.ok) throw new Error("Error en /set_verify (" + r5.status + ")");
const r6 = await fetch(urlPrinter);
if (!r6.ok) throw new Error("Error en /set_printer (" + r6.status + ")");
setStatus("Configuración guardada. Recargando estado...");
await loadStatus();
} catch (e) {
console.error(e);
setStatus("Error al guardar configuración: " + e.message, false);
}
}
function escapeHtml(v) {
return String(v ?? "").replace(/[&<>"']/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;","\"":"&quot;","'":"&#39;"}[c] || c));
}
function escapeAttr(v) {
return escapeHtml(v).replace(/`/g, "&#96;");
}
function humanBytes(n) {
n = Number(n || 0);
if (n <= 0) return "—";
if (n < 1024) return n + " B";
if (n < 1024*1024) return (n/1024).toFixed(1) + " KB";
return (n/(1024*1024)).toFixed(2) + " MB";
}
const UI_MAX_UNITS = 7;
let lastDriveVisibility = null;
function visibleUnitMaskFromStatus(obj) { return Number(obj?.driveVisibleMask ?? obj?.visibleMask ?? lastDriveVisibility?.visibleMask ?? 15); }
function unitVisible(mask, i) { return i <= 4 || !!(Number(mask || 15) & (1 << (i-1))); }
function maskToUnits(mask) {
mask = Number(mask || 0);
const out = [];
for (let i=1;i<=UI_MAX_UNITS;i++) if (mask & (1 << (i-1))) out.push(`D${i}`);
return out.length ? out.join("+") : "—";
}
function setDriveButtonsState(vis) {
const nextAdd = Number(vis?.nextAddUnit || 0);
const nextRemove = Number(vis?.nextRemoveUnit || 0);
["btnBtAddDrive","btnAtrAddDrive","btnDiskAddDrive"].forEach(id => { const el=$(id); if (el) { el.disabled = !nextAdd; el.title = nextAdd ? `Agrega D${nextAdd}` : "D1-D7 ya están visibles"; } });
["btnBtRemoveDrive","btnAtrRemoveDrive","btnDiskRemoveDrive"].forEach(id => { const el=$(id); if (el) { el.disabled = !nextRemove; el.title = nextRemove ? `Quita D${nextRemove}` : "No hay unidades opcionales libres para quitar"; } });
for (let i=5;i<=UI_MAX_UNITS;i++) {
const u = (vis?.units || []).find(x => Number(x.unit) === i) || {};
const removable = !!u.removable;
[$("atrRemoveDrive"+i), $("btRemoveDrive"+i)].forEach(el => {
if (!el) return;
el.style.display = u.visible ? "" : "none";
el.disabled = !removable;
el.title = removable ? `Quitar D${i}` : (u.busyReason ? `No se puede quitar: ${u.busyReason}` : `D${i} debe estar vacía para quitarla`);
});
}
}
function updateVisibleUnitCards(mask, units) {
mask = Number(mask || 15);
const labels = [];
for (let i=1;i<=UI_MAX_UNITS;i++) {
const vis = unitVisible(mask, i);
if (vis) labels.push(`D${i}`);
const a = $(`atrDriveItem${i}`); if (a) a.style.display = vis ? "" : "none";
const b = $(`btDriveItem${i}`); if (b) b.style.display = vis ? "" : "none";
}
const txt = labels.length ? `${labels.join(", ")} visibles` : "D1-D4 visibles";
if ($("atrDriveVisibleState")) $("atrDriveVisibleState").textContent = txt;
if ($("btDriveVisibleState")) $("btDriveVisibleState").textContent = txt;
if ($("diskDriveVisibleState")) $("diskDriveVisibleState").textContent = txt;
if (units) setDriveButtonsState({ visibleMask: mask, units, nextAddUnit: lastDriveVisibility?.nextAddUnit || 0, nextRemoveUnit: lastDriveVisibility?.nextRemoveUnit || 0 });
if ($("drives") && Array.isArray(window.lastStatusDrivesV32)) renderDrives(window.lastStatusDrivesV32);
try {
const p = (location.pathname || "").toLowerCase();
if ((p.indexOf("/tnfs") === 0 || p.indexOf("/editor/tnfs") === 0) && typeof tnfsRefreshMountButtonsF22 === "function") tnfsRefreshMountButtonsF22();
} catch(e) {}
}
async function loadDriveVisibilityStatus() {
try {
const r = await fetch("/api/drives/visible", { cache:"no-store" });
if (!r.ok) throw new Error(await r.text());
const v = await r.json();
lastDriveVisibility = v;
updateVisibleUnitCards(Number(v.visibleMask || 15), v.units || []);
setDriveButtonsState(v);
if ($("drives") && Array.isArray(window.lastStatusDrivesV32)) renderDrives(window.lastStatusDrivesV32);
return v;
} catch(e) {
console.warn("No se pudo leer visibilidad de unidades", e);
return null;
}
}
async function applyDriveVisibilityEndpoint(url, okMsg) {
try {
setStatus("Actualizando unidades...", true);
const r = await fetch(url, { cache:"no-store" });
if (!r.ok) throw new Error(await r.text());
const v = await r.json();
lastDriveVisibility = v;
updateVisibleUnitCards(Number(v.visibleMask || 15), v.units || []);
setDriveButtonsState(v);
if ($("drives") && Array.isArray(window.lastStatusDrivesV32)) renderDrives(window.lastStatusDrivesV32);
// Recargar solo datos dependientes de unidades; no hay reload de página.
if (location.pathname.toLowerCase().indexOf("/atr") === 0) await loadAtrStatus({ files:false, force:true });
if (location.pathname.toLowerCase().indexOf("/bt") === 0) await loadBtSioStatus();
const pth = location.pathname.toLowerCase();
if (pth.indexOf("/disk") === 0 || pth.indexOf("/editor/disks") === 0 || pth.indexOf("/editor/disk") === 0 || pth.indexOf("/config") === 0 || location.pathname === "/") await loadStatus();
setStatus(okMsg || "Unidades actualizadas", true);
} catch(e) { setStatus("Error actualizando unidades: " + e.message, false); }
}
async function setOptionalDriveVisible(add) {
await applyDriveVisibilityEndpoint(add ? "/add_drive_visible" : "/remove_drive_visible", add ? "Disquetera agregada" : "Disquetera quitada");
}
async function removeOptionalDrive(unit) {
await applyDriveVisibilityEndpoint(`/remove_drive_visible?unit=${Number(unit)}`, `D${Number(unit)} quitada`);
}
let lastAtrStatus = null;
let atrLibraryViewMode = localStorage.getItem("atrLibraryViewMode") || "flat";
function atrFileOptions(files, selected){
const listAll = Array.isArray(files) ? files : [];
const allowed = listAll.filter(f => {
const name = String((f && f.name) || "");
const t = String((f && (f.type || f.ext)) || (name.split('.').pop() || "")).replace(/^\./,'').toUpperCase();
return t === "ATR" || t === "XEX";
}).sort((a,b)=>String(a.name||"").localeCompare(String(b.name||""), "es", {sensitivity:"base", numeric:true}));
const atrCount = allowed.filter(f => String((f.type || f.ext || String(f.name || "").split('.').pop() || "")).replace(/^\./,'').toUpperCase() === "ATR").length;
const xexCount = allowed.filter(f => String((f.type || f.ext || String(f.name || "").split('.').pop() || "")).replace(/^\./,'').toUpperCase() === "XEX").length;
let html = `<option value="">— sin montar — (${allowed.length} ATR/XEX)</option>`;
html += `<option value="" disabled>Disponibles: ${allowed.length} · ATR: ${atrCount} · XEX: ${xexCount}</option>`;
let selectedFound = !selected;
allowed.forEach(f => {
const name = String(f.name || "");
const sel = name === selected ? " selected" : "";
if (sel) selectedFound = true;
const t = String((f.type || f.ext || name.split('.').pop() || "")).replace(/^\./,'').toUpperCase();
html += `<option value="${escapeHtml(name)}"${sel}>[${escapeHtml(t)}] ${escapeHtml(name)}</option>`;
});
if (selected && !selectedFound) html += `<option value="${escapeHtml(selected)}" selected>[montado] ${escapeHtml(selected)}</option>`;
return html;
}
function atrUnitMaskFromUi(prefix){
let mask = 0;
for (let i=1;i<=UI_MAX_UNITS;i++) if ($(`${prefix}${i}`)?.checked) mask |= (1 << (i-1));
return mask;
}
const ATR_IMAGE_LOAD_MAX = 1;
const ATR_THUMB_MISS_TTL_MS_F49Z41 = 30 * 60 * 1000;
let atrImageLoadActive = 0;
const atrImageLoadQueue = [];
const atrThumbMissingF49Z41 = new Map();
function atrThumbKeyF49Z41(fileName){
try { return atrCoverCacheKey(fileName || ''); }
catch(e) { return String(fileName || '').toLowerCase(); }
}
function atrThumbMissedF49Z41(fileName){
const k = atrThumbKeyF49Z41(fileName);
const t = atrThumbMissingF49Z41.get(k) || 0;
if (!t) return false;
if (Date.now() - t > ATR_THUMB_MISS_TTL_MS_F49Z41) { atrThumbMissingF49Z41.delete(k); return false; }
return true;
}
function atrThumbMarkMissingF49Z41(fileName){
const k = atrThumbKeyF49Z41(fileName);
if (k) atrThumbMissingF49Z41.set(k, Date.now());
}
function atrThumbClearMissingF49Z41(){ atrThumbMissingF49Z41.clear(); }
let atrLazyCoverObserver = null;
let atrLibraryLastRenderSig = "";
let atrLibraryLastRenderQ = "";
let atrLibraryLastRenderMode = "";
let atrLibraryFilesCache = [];
let atrLibraryFilesCacheSig = "";
let atrSelectOptionsLastSig = "";
let atrSlotCoverNameLast = Array(UI_MAX_UNITS + 1).fill("");
function atrImageQueuePump() {
while (atrImageLoadActive < ATR_IMAGE_LOAD_MAX && atrImageLoadQueue.length) {
const img = atrImageLoadQueue.shift();
if (!img || img.dataset.loaded === "1") continue;
const src = img.dataset.src || "";
if (!src) continue;
atrImageLoadActive++;
img.dataset.loaded = "1";
const done = () => {
atrImageLoadActive = Math.max(0, atrImageLoadActive - 1);
setTimeout(atrImageQueuePump, 45);
};
img.addEventListener("load", done, { once:true });
img.addEventListener("error", () => {
atrThumbMarkMissingF49Z41(img.dataset.coverTitle || img.dataset.coverName || "");
done();
}, { once:true });
img.src = src;
}
}
function atrQueueImageLoad(img) {
if (!img || img.dataset.queued === "1" || img.dataset.loaded === "1") return;
img.dataset.queued = "1";
atrImageLoadQueue.push(img);
atrImageQueuePump();
}
function atrInitLazyCovers() {
const imgs = Array.from(document.querySelectorAll("img.atr-lazy-cover[data-src]:not([data-loaded='1'])"));
if (!imgs.length) return;
if ("IntersectionObserver" in window) {
if (atrLazyCoverObserver) atrLazyCoverObserver.disconnect();
atrLazyCoverObserver = new IntersectionObserver((entries) => {
entries.forEach(entry => {
if (entry.isIntersecting) {
atrLazyCoverObserver.unobserve(entry.target);
atrQueueImageLoad(entry.target);
}
});
}, { root:null, rootMargin:"80px 0px", threshold:0.01 });
imgs.forEach(img => atrLazyCoverObserver.observe(img));
} else {
imgs.forEach((img, i) => setTimeout(() => atrQueueImageLoad(img), i * 90));
}
}
function atrLibraryFilesSignature(files) {
return (files || []).map(f => [f.name || "", f.type || "", f.fileSize || 0, f.mountedMask || 0, f.casMounted || 0, f.protected ? 1 : 0].join(":" )).join("|");
}
function atrLibraryCoverUrl(fileName) {
// F10: la biblioteca carga primero /MINI_COVERS.
// Si no existe miniatura, el backend entrega la carátula completa y el navegador genera la miniatura en segundo plano.
return atrCoverThumbNameSrc(fileName);
}
function atrLibraryCoverHtml(fileName) {
const safe = atrCoverEscapeAttr(fileName || "");
const src = atrLibraryCoverUrl(fileName || "");
const full = atrCoverProxyNameSrc(fileName || "");
// F49Z41: si ya sabemos que no hay mini/local para este título en esta sesión,
// no hacemos otro HTTP 404 por cada render/página.
if (atrThumbMissedF49Z41(fileName || "")) {
return `<div class="atr-lib-cover-box">Sin<br>carátula</div>`;
}
return `<img class="atr-lib-cover atr-lazy-cover" loading="lazy" decoding="async" fetchpriority="low" alt="Carátula" `+
`data-src="${src}" data-cover-title="${safe}" data-cover-name="${safe}" data-cover-full="${full}" `+
`onclick="atrOpenCoverFullscreen(this.dataset.coverFull || this.src, this.dataset.coverTitle || '')" `+
`onload="atrEnsureMiniCoverFromImage(this, this.dataset.coverTitle || '')" `+
`onerror="atrThumbMarkMissingF49Z41(this.dataset.coverName || this.dataset.coverTitle || ''); this.style.display='none'; const n=this.nextElementSibling; if(n)n.style.display='flex';">`+
`<div class="atr-lib-cover-box" style="display:none;">Sin<br>carátula</div>`;
}
function atrLibraryRowHtml(f) {
const isMounted = Number(f.mountedMask || 0) !== 0;
const mounted = isMounted ? ` · montado: ${maskToUnits(f.mountedMask)}` : " · sin montar";
const type = f.type || "ATR";
const enc = encodeURIComponent(f.name);
const protectedInfo = f.protected ? " · incluido/protegido" : "";
const info = `Ext: ${escapeHtml(type)} · ${humanBytes(f.fileSize)} · sector ${f.sectorSize || "?"}B · ${f.totalSectors || "?"} sectores${mounted}${protectedInfo}`;
let actions = "";
if (isMounted) {
actions = `<span class="mounted-badge" title="Desmonta la imagen desde su unidad para volver a montar o eliminar">Montado: ${escapeHtml(maskToUnits(f.mountedMask))}</span>`;
} else {
const deleteAction = f.protected
? `<span class="protected-badge" title="ATR incluido en el firmware; no se puede eliminar desde la web">Protegido</span>`
: `<button type="button" class="secondary trash-btn" onclick="deleteAtr('${enc}')" title="Borrar">🗑</button>`;
const visibleMask = visibleUnitMaskFromStatus(lastAtrStatus);
let unitButtons = "";
for (let i=1;i<=UI_MAX_UNITS;i++) {
if (unitVisible(visibleMask, i)) unitButtons += `<button type="button" class="secondary" onclick="mountAtrQuick('${enc}',${i})" title="Montar en D${i}">D${i}</button>`;
}
actions = unitButtons + deleteAction;
}
return `<div class="atr-file-row${isMounted ? ' mounted' : ''}">`+
`<div class="atr-file-main with-cover">`+
`<div class="atr-lib-row-content">`+
atrLibraryCoverHtml(f.name)+
`<div class="atr-lib-text">`+
`<div class="atr-file-title" title="${escapeHtml(f.name)}">${escapeHtml(f.name)}</div>`+
`<div class="atr-file-info">${info}</div>`+
atrLibraryCoverActionsHtml(f.name)+
`</div>`+
`</div>`+
`<span class="atr-file-actions">${actions}</span>`+
`</div>`+
`</div>`;
}
function atrGroupKey(name) {
const c = String(name || "").trim().charAt(0).toUpperCase();
if (!c) return "#";
if (c >= "0" && c <= "9") return "0-9";
if (c >= "A" && c <= "Z") return c;
return "#";
}
function atrLibraryResetPageF49Z20() {
window.atrLibraryPageF49Z20 = 0;
atrLibraryLastRenderSigF27 = "";
}
function atrLibraryRenderPagerF49Z20(total, page, pages, start, end) {
const pager = document.getElementById("atrLibraryPager");
if (!pager) return;
if (!total || total <= ATR_LIBRARY_PAGE_SIZE_F49Z20) {
pager.style.display = "none";
pager.innerHTML = "";
return;
}
pager.style.display = "flex";
pager.innerHTML =
`<button id="btnAtrLibraryPrevPage" type="button" class="secondary" ${page <= 0 ? "disabled" : ""}>◀ Anterior</button>`+
`<span class="library-page-info">Página ${page + 1} de ${pages} · ${start + 1}-${end} de ${total}</span>`+
`<button id="btnAtrLibraryNextPage" type="button" class="secondary" ${page >= pages - 1 ? "disabled" : ""}>Siguiente ▶</button>`+
`<span class="library-page-size">20 archivos por página</span>`;
const prev = document.getElementById("btnAtrLibraryPrevPage");
const next = document.getElementById("btnAtrLibraryNextPage");
if (prev) prev.onclick = () => { window.atrLibraryPageF49Z20 = Math.max(0, Number(window.atrLibraryPageF49Z20 || 0) - 1); renderAtrLibrary(atrLibraryCurrentFilesF27(), { force:true }); };
if (next) next.onclick = () => { window.atrLibraryPageF49Z20 = Math.min(pages - 1, Number(window.atrLibraryPageF49Z20 || 0) + 1); renderAtrLibrary(atrLibraryCurrentFilesF27(), { force:true }); };
}
function renderAtrLibrary(files, opts = {}) {
const lib = $("atrLibrary");
if (!lib) return;
let list = (files || []).slice();
const q = String($("atrLibrarySearch")?.value || "").trim().toLowerCase();
if (q) list = list.filter(f => String(f.name || "").toLowerCase().includes(q));
const mode = atrLibraryViewMode === "folders" ? "folders" : "flat";
const sig = atrLibraryFilesSignature(list);
if (!opts.force && sig === atrLibraryLastRenderSig && q === atrLibraryLastRenderQ && mode === atrLibraryLastRenderMode) {
return;
}
atrLibraryLastRenderSig = sig;
atrLibraryLastRenderQ = q;
atrLibraryLastRenderMode = mode;
// F49Z41: al cambiar página/filtro, cancelar cargas de carátulas anteriores.
atrImageLoadQueue.length = 0;
if (atrLazyCoverObserver) { try { atrLazyCoverObserver.disconnect(); } catch(e){} }
if (!list.length) { lib.innerHTML = q ? "Sin resultados para la búsqueda." : "Sin ATRs/XEX cargados."; return; }
if (mode === "flat") {
lib.innerHTML = list.map(atrLibraryRowHtml).join("");
setTimeout(atrInitLazyCovers, 0);
return;
}
const groups = {};
list.forEach(f => {
const k = atrGroupKey(f.name);
if (!groups[k]) groups[k] = [];
groups[k].push(f);
});
const order = ["0-9", ..."ABCDEFGHIJKLMNOPQRSTUVWXYZ".split(""), "#"];
lib.innerHTML = order.filter(k => groups[k]?.length).map(k => {
const items = groups[k];
return `<details class="atr-folder-group">`+
`<summary class="atr-folder-title"><span>${escapeHtml(k)}</span><span class="pill">${items.length}</span></summary>`+
`<div class="atr-folder-body">${items.map(atrLibraryRowHtml).join("")}</div>`+
`</details>`;
}).join("");
setTimeout(atrInitLazyCovers, 0);
}
let btSioSaveTimer = null;
let btSioSaving = false;
let btSioLastApplyMs = 0;
function btSioSetAutoSaveState(text, cls) {
const el = $("btSioAutoSaveState");
if (!el) return;
el.textContent = text;
el.className = "pill " + (cls || "ok");
}
function btSioMaskFromUi() {
let m = 0;
for (let i=1;i<=UI_MAX_UNITS;i++) if ($(`btSioEn${i}`)?.checked) m |= (1 << (i-1));
return m;
}
function btSioForceMaskFromUi() {
let m = 0;
for (let i=1;i<=UI_MAX_UNITS;i++) if ($(`btSioEn${i}`)?.checked && $(`btSioForce${i}`)?.checked) m |= (1 << (i-1));
return m;
}
function btSioProfileParamsFromUi() {
const parts = [];
for (let i=1;i<=UI_MAX_UNITS;i++) parts.push(`p${i}=${encodeURIComponent($(`btSioProfile${i}`)?.value || "fast")}`);
return parts.join("&");
}
async function loadBtSioStatus() {
try {
const r = await fetch("/api/bt/status", { cache:"no-store" });
if (!r.ok) throw new Error("HTTP " + r.status);
const b = await r.json();
if ($("btSioState")) $("btSioState").textContent = b.sio2pcEnabled ? `ON mask=0x${Number(b.sio2pcMask||0).toString(16)}` : "OFF";
if ($("btSioBaudState")) $("btSioBaudState").textContent = String(b.baud || "—");
if ($("btnBtSioRefresh")) $("btnBtSioRefresh").textContent = "🔄 Estado leído";
setTimeout(() => { if ($("btnBtSioRefresh")) $("btnBtSioRefresh").textContent = "🔄 Leer estado"; }, 900);
if ($("btSioReadOk")) $("btSioReadOk").textContent = `${b.sio2pcReadOk||0} READ · ${b.sio2pcStatusOk||0} STATUS`;
if ($("btSioErr")) $("btSioErr").textContent = `${b.sio2pcNak||0} NAK · ${b.sio2pcTimeout||0} TO · ${b.sio2pcBadChk||0} CHK`;
if ($("btSioSpeed")) $("btSioSpeed").textContent = `avg ${b.sio2pcAvgMs||0} ms · max ${b.sio2pcMaxMs||0} ms · retry ${b.sio2pcRetry||0} · cache ${b.sio2pcCacheHit||0}/${b.sio2pcCacheMiss||0} · buf ${b.sio2pcRxBuffer||0}/${b.sio2pcTxBuffer||0}`;
if ($("btSioLastErr")) $("btSioLastErr").textContent = b.sio2pcLastErr || "—";
const mask = Number(b.sio2pcMask || 0);
const force = Number(b.sio2pcForceMask || 0);
const profiles = b.sio2pcUnitProfiles || Array(UI_MAX_UNITS).fill(b.sio2pcProfile || "fast");
const visibleMask = visibleUnitMaskFromStatus(b);
updateVisibleUnitCards(visibleMask, lastDriveVisibility?.units || null);
loadDriveVisibilityStatus();
for (let i=1;i<=UI_MAX_UNITS;i++) {
const enabled = !!(mask & (1 << (i-1)));
const forced = !!(force & (1 << (i-1)));
const prof = profiles[i-1] || "fast";
if ($(`btSioEn${i}`)) $(`btSioEn${i}`).checked = enabled;
if ($(`btSioForce${i}`)) $(`btSioForce${i}`).checked = forced;
if ($(`btSioProfile${i}`)) $(`btSioProfile${i}`).value = prof;
const route = $(`btRoute${i}`);
if (route) { route.textContent = enabled ? "BT-SIO2PC ON" : "BT OFF"; route.className = enabled ? "pill ok" : "pill bad"; }
const fb = $(`btFallback${i}`);
if (fb) { fb.textContent = forced ? "sin fallback" : "fallback físico"; fb.className = forced ? "pill warn" : "pill"; }
if ($(`btProfileName${i}`)) $(`btProfileName${i}`).textContent = prof;
if ($(`btProfilePill${i}`)) { $(`btProfilePill${i}`).textContent = prof; $(`btProfilePill${i}`).className = prof === "turbo" ? "pill warn" : "pill"; }
if ($(`btUnitMeta${i}`)) $(`btUnitMeta${i}`).textContent = enabled ? (forced ? "enrutado solo a Bluetooth" : "Bluetooth con fallback físico") : "deshabilitado";
}
if ($("btSioBaud")) $("btSioBaud").value = b.baud || 115200;
if ($("btSioQuiet")) $("btSioQuiet").checked = !!(b.sio2pcQuietLog ?? 1);
} catch(e) { setStatus("Error Bluetooth SIO2PC: " + e.message, false); }
}
function btSioApplyLightStatusFast(b) {
if (!b) return;
const enabled = !!Number(b.sio2pcEnabled || 0);
const mask = Number(b.sio2pcMask || 0);
const force = Number(b.sio2pcForceMask || 0);
const profiles = b.unitProfiles || b.sio2pcUnitProfiles || Array(UI_MAX_UNITS).fill(b.profile || b.sio2pcProfile || "fast");
if ($("btSioState")) $("btSioState").textContent = enabled ? `ON mask=0x${mask.toString(16)}` : "OFF";
if ($("btSioBaudState")) $("btSioBaudState").textContent = String(b.baud || "—");
for (let i=1;i<=UI_MAX_UNITS;i++) {
const unitEnabled = !!(mask & (1 << (i-1)));
const unitForced = !!(force & (1 << (i-1)));
const prof = profiles[i-1] || "fast";
if ($(`btSioEn${i}`)) $(`btSioEn${i}`).checked = unitEnabled;
if ($(`btSioForce${i}`)) $(`btSioForce${i}`).checked = unitForced;
if ($(`btSioProfile${i}`)) $(`btSioProfile${i}`).value = prof;
const route = $(`btRoute${i}`);
if (route) { route.textContent = unitEnabled ? "BT-SIO2PC ON" : "BT OFF"; route.className = unitEnabled ? "pill ok" : "pill bad"; }
const fb = $(`btFallback${i}`);
if (fb) { fb.textContent = unitForced ? "sin fallback" : "fallback físico"; fb.className = unitForced ? "pill warn" : "pill"; }
if ($(`btProfileName${i}`)) $(`btProfileName${i}`).textContent = prof;
if ($(`btProfilePill${i}`)) { $(`btProfilePill${i}`).textContent = prof; $(`btProfilePill${i}`).className = prof === "turbo" ? "pill warn" : "pill"; }
if ($(`btUnitMeta${i}`)) $(`btUnitMeta${i}`).textContent = unitEnabled ? (unitForced ? "enrutado solo a Bluetooth" : "Bluetooth con fallback físico") : "deshabilitado";
}
if ($("btSioBaud") && b.baud) $("btSioBaud").value = b.baud;
if ($("btSioQuiet") && typeof b.quiet !== "undefined") $("btSioQuiet").checked = !!Number(b.quiet || 0);
}
async function setBtSio2pc(en, opts = {}) {
const mask = en ? btSioMaskFromUi() : 0;
const baud = Number($("btSioBaud")?.value || 115200);
const force = en ? btSioForceMaskFromUi() : 0;
const quiet = $("btSioQuiet")?.checked ? 1 : 0;
const profParams = btSioProfileParamsFromUi();
const auto = !!opts.auto;
try {
btSioSaving = true;
if (auto) btSioSetAutoSaveState("Guardando...", "warn");
const r = await fetch(`/set_bt_sio2pc?en=${en?1:0}&mask=${mask}&force=${force}&baud=${baud}&quiet=${quiet}&${profParams}`, { cache:"no-store" });
if (!r.ok) throw new Error(await r.text());
const b = await r.json().catch(() => null);
btSioLastApplyMs = Date.now();
btSioApplyLightStatusFast(b);
setTimeout(() => loadBtSioStatus(), 1200);
btSioSetAutoSaveState(b?.deferredSave ? "Guardado / NVS pendiente" : "Guardado", "ok");
setStatus(en ? (auto ? "Bluetooth guardado automáticamente" : "BT-SIO2PC actualizado") : "BT-SIO2PC apagado", true);
} catch(e) {
btSioSetAutoSaveState("Error al guardar", "bad");
setStatus("Error configurando BT-SIO2PC: " + e.message, false);
} finally {
btSioSaving = false;
if (auto) setTimeout(() => btSioSetAutoSaveState("Auto-guardado activo", "ok"), 1200);
}
}
function scheduleBtSioAutoSave() {
if (btSioSaveTimer) clearTimeout(btSioSaveTimer);
btSioSetAutoSaveState("Cambio pendiente...", "warn");
btSioSaveTimer = setTimeout(() => {
const enabled = btSioMaskFromUi() !== 0;
setBtSio2pc(enabled, { auto:true });
}, 650);
}
function bindBtSioAutoSave() {
const ids = ["btSioQuiet", "btSioBaud"];
for (let i=1;i<=UI_MAX_UNITS;i++) ids.push(`btSioEn${i}`, `btSioForce${i}`, `btSioProfile${i}`);
ids.forEach(id => {
const el = $(id);
if (!el || el.dataset.btAutoSaveBound === "1") return;
el.dataset.btAutoSaveBound = "1";
const ev = (id === "btSioBaud") ? "change" : "change";
el.addEventListener(ev, () => {
const m = String(id).match(/^btSioEn(\d+)$/);
if (m && el.checked) {
const force = $("btSioForce" + m[1]);
if (force) force.checked = true;
}
scheduleBtSioAutoSave();
});
});
}
let atrSaveTimer = null;
let atrSaving = false;
function atrSetAutoSaveState(txt, cls) {
const el = $("atrAutoSaveState");
if (!el) return;
el.textContent = txt;
el.className = "pill " + (cls || "ok");
}
function atrBuildMountParamsFromUi() {
const mask = atrUnitMaskFromUi("atrEn");
const force = atrUnitMaskFromUi("atrForce");
const params = new URLSearchParams();
params.set("en", mask ? "1" : "0");
params.set("mask", String(mask));
params.set("force", String(force));
for (let i=1;i<=UI_MAX_UNITS;i++) params.set(`d${i}`, $(`atrSel${i}`)?.value || "");
return { params, enabled: mask !== 0 };
}
function scheduleAtrAutoSave() {
if (atrSaveTimer) clearTimeout(atrSaveTimer);
atrSetAutoSaveState("Cambio pendiente...", "warn");
atrSaveTimer = setTimeout(() => applyAtrMount({ auto:true }), 350);
}
function bindAtrAutoSave() {
const ids = [];
for (let i=1;i<=UI_MAX_UNITS;i++) ids.push(`atrSel${i}`, `atrEn${i}`, `atrForce${i}`);
ids.forEach(id => {
const el = $(id);
if (!el || el.dataset.atrAutoSaveBound === "1") return;
el.dataset.atrAutoSaveBound = "1";
el.addEventListener("change", () => {
const m = String(id).match(/^atrEn(\d+)$/);
if (m && el.checked) {
const force = $("atrForce" + m[1]);
if (force) force.checked = true;
}
scheduleAtrAutoSave();
});
});
}
let atrCoverCatalog = null;
let atrCoverCatalogPromise = null;
const ATR_LIBRETRO_BOXART_BASE = "https://thumbnails.libretro.com/Atari%20-%208-bit/Named_Boxarts/";
function normalizeAtrCoverTitle(name) {
return String(name || "")
.replace(/^atr[_\-\s]*/i, "")
.replace(/\.(atr|xex|com|exe)$/i, "")
.replace(/\[[^\]]*\]/g, " ")
.replace(/\([^\)]*\)/g, " ")
.replace(/[_\.]+/g, " ")
.replace(/\b(disk|disc|side|boot|crack|fixed|trainer|pal|ntsc|atari|xe|xl|arcade)\b/ig, " ")
.replace(/\s+/g, " ")
.trim();
}
function normalizeAtrCoverKey(name) {
return normalizeAtrCoverTitle(name)
.toLowerCase()
.normalize("NFD").replace(/[\u0300-\u036f]/g, "")
.replace(/&/g, "and")
.replace(/[^a-z0-9]+/g, " ")
.replace(/\s+/g, " ")
.trim();
}
async function loadAtrCoverCatalog() {
if (atrCoverCatalog) return atrCoverCatalog;
if (atrCoverCatalogPromise) return atrCoverCatalogPromise;
atrCoverCatalogPromise = fetch("/covers_libretro.json", { cache:"no-store" })
.then(r => r.ok ? r.json() : { items:{}, aliases:{} })
.then(j => {
atrCoverCatalog = j || { items:{}, aliases:{} };
atrCoverCatalog.items = atrCoverCatalog.items || {};
atrCoverCatalog.aliases = atrCoverCatalog.aliases || {};
return atrCoverCatalog;
})
.catch(() => {
atrCoverCatalog = { items:{}, aliases:{} };
return atrCoverCatalog;
});
return atrCoverCatalogPromise;
}
function atrCoverNormalizeEntry(entry, fallbackTitle) {
if (!entry) return null;
if (typeof entry === "string") return { title: fallbackTitle, image: entry };
return {
title: entry.title || fallbackTitle,
image: entry.image || entry.img || entry.url || "",
page: entry.page || entry.pageUrl || entry.tcp || "",
source: entry.source || ""
};
}
function atrCoverFindEntry(catalog, fileName) {
const title = normalizeAtrCoverTitle(fileName);
let key = normalizeAtrCoverKey(fileName);
if (!key) return null;
const aliases = catalog.aliases || {};
if (aliases[key]) key = normalizeAtrCoverKey(aliases[key]);
const items = catalog.items || {};
const candidates = [key, title, fileName].filter(Boolean);
for (const c of candidates) {
if (items[c]) return atrCoverNormalizeEntry(items[c], title);
const ck = normalizeAtrCoverKey(c);
if (items[ck]) return atrCoverNormalizeEntry(items[ck], title);
}
for (const rawKey of Object.keys(items)) {
const k = normalizeAtrCoverKey(rawKey);
if (!k) continue;
if (k === key || key.includes(k) || k.includes(key)) return atrCoverNormalizeEntry(items[rawKey], title);
}
return { title, image:"", page:"" };
}
function atrCoverSearchUrl(title) {
// Libretro no entrega API de búsqueda; abrimos el índice directo de boxarts.
return ATR_LIBRETRO_BOXART_BASE;
}
const atrCoverResolvedCache = {};
const atrCoverImgLastSrc = {};
function atrCoverCacheKey(fileName) {
return typeof normalizeAtrCoverKey === "function"
? normalizeAtrCoverKey(fileName || "")
: String(fileName || "").toLowerCase().trim();
}
function atrCoverIsLocalPathClient(url) {
const u = String(url || "").trim().replace(/\\/g, "/");
return u.indexOf("/COVERS/") === 0 || u.indexOf("/MINI_COVERS/") === 0;
}
function atrCoverSetImage(unit, fileName, url, sourceText, useProxy) {
const panel = $(`atrCoverPanel${unit}`);
const img = $(`atrCoverImg${unit}`);
const ph = $(`atrCoverPlaceholder${unit}`);
const titleEl = $(`atrCoverTitle${unit}`);
const subEl = $(`atrCoverSub${unit}`);
const actions = $(`atrCoverActions${unit}`);
if (!panel || !img || !ph || !titleEl || !subEl || !actions) return;
const cleanTitle = normalizeAtrCoverTitle(fileName) || fileName || ("D" + unit);
const cleanUrl = atrNormalizeImageUrl(url);
const hasImageRef = !!cleanUrl;
// F12: al montar discos, nunca se usa directo /COVERS/archivo ni una URL remota como src.
// Siempre se pasa por el MASTER para que primero sirva /MINI_COVERS, luego /COVERS,
// y solo como último recurso intente descargar/proxy. Esto evita carátulas rotas al montar.
const finalSrc = hasImageRef
? atrCoverThumbUrlSrc(cleanUrl, fileName)
: atrCoverThumbNameSrc(fileName);
const fullSrc = hasImageRef
? atrCoverProxySrc(cleanUrl)
: atrCoverProxyNameSrc(fileName);
panel.classList.add("show");
titleEl.textContent = cleanTitle;
subEl.textContent = sourceText || "Fuente: MASTER / SD";
actions.innerHTML = atrCoverActionsHtml(unit, fileName, true);
if (atrCoverImgLastSrc[unit] === finalSrc && img.getAttribute("src") === finalSrc) {
img.style.display = "block";
ph.style.display = "none";
return;
}
atrCoverImgLastSrc[unit] = finalSrc;
ph.style.display = "flex";
ph.textContent = "Cargando";
img.style.display = "none";
img.dataset.coverFull = fullSrc;
img.onclick = () => atrOpenCoverFullscreen(img.dataset.coverFull || img.src, cleanTitle);
img.title = "Click para ver carátula a pantalla completa";
img.onload = () => {
img.style.display = "block";
ph.style.display = "none";
atrEnsureMiniCoverFromImage(img, fileName || cleanTitle);
};
img.onerror = () => {
img.style.display = "none";
ph.style.display = "flex";
ph.textContent = "Sin carátula";
subEl.textContent = "No se encontró miniatura ni carátula local en SD.";
};
img.src = finalSrc;
}
function atrUseProxyCover(unit, fileName) {
const key = atrCoverCacheKey(fileName);
const cached = atrCoverResolvedCache[key] || atrManualCoverGet(fileName);
if (cached && cached.image) {
atrCoverSetImage(unit, fileName, cached.image, "Fuente: proxy MASTER", true);
return;
}
atrCoverSetImage(unit, fileName, atrCoverProxyNameSrc(fileName), "Fuente: proxy MASTER", false);
}
async function updateAtrCover(unit, fileName, present) {
const panel = $(`atrCoverPanel${unit}`);
const img = $(`atrCoverImg${unit}`);
const ph = $(`atrCoverPlaceholder${unit}`);
const titleEl = $(`atrCoverTitle${unit}`);
const subEl = $(`atrCoverSub${unit}`);
const actions = $(`atrCoverActions${unit}`);
if (!panel || !img || !ph || !titleEl || !subEl || !actions) return;
if (!fileName || fileName === "—") {
panel.classList.remove("show");
img.removeAttribute("src");
atrCoverImgLastSrc[unit] = "";
img.style.display = "none";
ph.style.display = "flex";
actions.innerHTML = "";
subEl.textContent = "";
return;
}
const key = atrCoverCacheKey(fileName);
panel.classList.add("show");
titleEl.textContent = normalizeAtrCoverTitle(fileName) || fileName;
actions.innerHTML = atrCoverActionsHtml(unit, fileName, false);
if (atrCoverResolvedCache[key] && atrCoverResolvedCache[key].image) {
const c = atrCoverResolvedCache[key];
atrCoverSetImage(unit, fileName, c.image, c.sourceText || "Fuente: caché", false);
return;
}
const manualCover0 = atrManualCoverGet(fileName);
if (manualCover0 && manualCover0.image) {
atrCoverResolvedCache[key] = { image: manualCover0.image, sourceText: "Fuente: URL manual local" };
atrCoverSetImage(unit, fileName, manualCover0.image, "Fuente: URL manual local", false);
return;
}
const serverCover = await atrServerCoverResolve(fileName);
if (serverCover && serverCover.image) {
atrCoverResolvedCache[key] = {
image: serverCover.image,
sourceText: "Fuente: " + (serverCover.source || "MASTER")
};
atrCoverSetImage(unit, fileName, serverCover.image, atrCoverResolvedCache[key].sourceText, false);
return;
}
const catalog = await loadAtrCoverCatalog();
const entry = atrCoverFindEntry(catalog, fileName);
if (entry && entry.image) {
atrCoverResolvedCache[key] = {
image: entry.image,
sourceText: entry.source ? `Fuente: ${entry.source}` : "Fuente: Libretro"
};
atrCoverSetImage(unit, fileName, entry.image, atrCoverResolvedCache[key].sourceText, false);
return;
}
img.removeAttribute("src");
atrCoverImgLastSrc[unit] = "";
img.style.display = "none";
ph.style.display = "flex";
ph.textContent = "Sin\\ncarátula";
subEl.textContent = "No está en Libretro local";
actions.innerHTML = atrCoverActionsHtml(unit, fileName, false);
}
function casFmtTimeF49C(seconds) {
seconds = Math.max(0, Math.floor(Number(seconds) || 0));
const m = Math.floor(seconds / 60), s = seconds % 60;
return `${String(m).padStart(2,"0")}:${String(s).padStart(2,"0")}`;
}
function casTurboCfgF49C() {
try { return JSON.parse(localStorage.getItem("casTurboCfgF49C") || "{}"); } catch(e) { return {}; }
}
function casSaveTurboCfgF49C(cfg) {
localStorage.setItem("casTurboCfgF49C", JSON.stringify(cfg || {}));
}
function casBaudProfileLabelF49Z11(baud) {
const b = Number(baud || 600);
if (b === 0) return "Auto recomendado";
const labels = {
600:"Atari normal", 800:"Turbo Software", 1200:"Iron Turbo", 2270:"Turbo 2000",
2600:"Turbo 2600", 3000:"KSO / T2000F", 3995:"Injektor", 6000:"Turbo 6000 / Chaos"
};
return labels[b] || "Personalizado";
}
function casAutoRecommendedBaudF49Z11() {
const a = window.casLastStatusF49J || {};
return Number(a.casAutoInitialBaud || a.casAutoTurboBaud || 600) || 600;
}
function casCurrentTurboCfgF49C() {
const cfg = { baud:600, mult:100, delay:0, turbo:0, auto:1 };
const activeBtn = (group) => document.querySelector(`.cas-segment[data-cas-group="${group}"] button.active`);
const baudBtn = activeBtn("baud");
cfg.auto = baudBtn && baudBtn.dataset.auto === "1" ? 1 : 0;
cfg.baud = cfg.auto ? casAutoRecommendedBaudF49Z11() : (baudBtn ? Number(baudBtn.dataset.val || 600) : 600);
cfg.mult = 100;
const delayBtn = activeBtn("delay");
cfg.delay = delayBtn ? Number(delayBtn.dataset.val || 0) : 0;
cfg.turbo = (!cfg.auto && cfg.baud > 600) ? 1 : 0;
return cfg;
}
function casApplyTurboUiF49C(cfg) {
cfg = {baud:0,mult:100,delay:0,turbo:0,auto:1,...(cfg||{})};
document.querySelectorAll(`.cas-segment[data-cas-group="baud"] button`).forEach(btn => {
const isAuto = btn.dataset.auto === "1";
btn.classList.toggle("active", cfg.auto ? isAuto : (!isAuto && Number(btn.dataset.val||0) === Number(cfg.baud||0)));
});
document.querySelectorAll(`.cas-segment[data-cas-group="delay"] button`).forEach(btn => {
btn.classList.toggle("active", Number(btn.dataset.val||0) === Number(cfg.delay||0));
});
}
function casBindTurboUiF49C() {
if (window.__casTurboUiF49C) return;
window.__casTurboUiF49C = true;
casApplyTurboUiF49C({baud:0,mult:100,delay:0,turbo:0,auto:1,...casTurboCfgF49C()});
document.querySelectorAll(".cas-segment button").forEach(btn => {
btn.addEventListener("click", () => {
const group = btn.closest(".cas-segment")?.dataset.casGroup;
if (!group) return;
btn.closest(".cas-segment").querySelectorAll("button").forEach(b => b.classList.remove("active"));
btn.classList.add("active");
casSaveTurboCfgF49C(casCurrentTurboCfgF49C());
updateCasManualPanelF49(lastAtrStatus || {});
});
});
[].forEach(id => {
const el = $(id);
if (el) el.addEventListener("change", () => {
casSaveTurboCfgF49C(casCurrentTurboCfgF49C());
updateCasManualPanelF49(lastAtrStatus || {});
});
});
}
function casSoundCfgF49I() {
try { return {enabled:0, volume:0, profile:"disabled"}; } catch(e) { return {enabled:0, volume:0, profile:"disabled"}; }
}
function casSaveSoundCfgF49I(cfg) {
localStorage.setItem("casSoundCfgF49I", JSON.stringify({enabled: Number(cfg.enabled ? 1 : 0), volume: Number(cfg.volume || 0), profile: String(cfg.profile || "original")}));
}
function casCurrentSoundCfgF49I() {
return { enabled:false, volume:0, profile:"disabled" };
}
function casApplySoundUiF49I(cfg) {
cfg = {enabled:0, volume:0, profile:"disabled", ...(cfg || {})};
if ($("casSoundEnabled")) $("casSoundEnabled").checked = !!Number(cfg.enabled);
if ($("casSoundVolume")) $("casSoundVolume").value = String(Number(cfg.volume || 35));
if ($("casSoundVolumeLbl")) $("casSoundVolumeLbl").textContent = `${Number(cfg.volume || 0)}%`;
if ($("casSoundState")) { $("casSoundState").textContent = ""; $("casSoundState").className = "pill"; }
}
function casBindSoundUiF49I() {
if (window.__casSoundUiF49I) return;
window.__casSoundUiF49I = true;
casApplySoundUiF49I(casSoundCfgF49I());
if ($("casSoundEnabled")) $("casSoundEnabled").addEventListener("change", () => { const cfg = casCurrentSoundCfgF49I(); casSaveSoundCfgF49I(cfg); casApplySoundUiF49I(cfg); if (!cfg.enabled) casSoundStopF49I(true); });
if ($("casSoundVolume")) $("casSoundVolume").addEventListener("input", () => { const cfg = casCurrentSoundCfgF49I(); casSaveSoundCfgF49I(cfg); casApplySoundUiF49I(cfg); casSoundSetVolumeF49I(cfg.volume); });
}
window.casAudioF49I = window.casAudioF49I || { ctx:null, node:null, gain:null, filter:null, phase:0, lfsr:0xACE1, bitSamples:80, bitRemain:0, bit:1, playing:false };
function casSoundSetVolumeF49I(vol) {
const a = window.casAudioF49I;
if (!a || !a.gain || !a.ctx) return;
const safe = Math.max(0, Math.min(100, Number(vol || 0))) / 100;
const target = safe * 0.085;
try { a.gain.gain.setTargetAtTime(target, a.ctx.currentTime, 0.025); } catch(e) {}
}
async function casSoundEnsureCtxF49I() {
const a = window.casAudioF49I;
if (!a.ctx) a.ctx = new (window.AudioContext || window.webkitAudioContext)();
if (a.ctx.state === "suspended") await a.ctx.resume();
return a.ctx;
}
function casSoundNextBitF49K(a) {
// LFSR pseudo-data so the monitor sound resembles a real Atari tape stream.
let x = a.lfsr || 0xACE1;
x ^= (x << 7) & 0xFFFF;
x ^= (x >> 9);
x ^= (x << 8) & 0xFFFF;
a.lfsr = x & 0xFFFF;
// Bias toward mark tone like real gaps/leader, with enough data variation to avoid a flat beep.
const leader = (Math.random() < 0.18) ? 1 : 0;
return leader ? 1 : (x & 1);
}
async function casSoundStartF49I() {
try {
return;
casBindSoundUiF49I();
const cfg = casCurrentSoundCfgF49I();
if (!cfg.enabled || Number(cfg.volume || 0) <= 0) return;
const a = window.casAudioF49I;
const ctx = await casSoundEnsureCtxF49I();
if (a.playing && a.node) { casSoundSetVolumeF49I(cfg.volume); return; }
casSoundStopF49I(false);
const gain = ctx.createGain();
gain.gain.value = 0;
const filter = ctx.createBiquadFilter();
filter.type = "bandpass";
filter.frequency.value = 4650;
filter.Q.value = 0.95;
const node = ctx.createScriptProcessor(1024, 0, 1);
a.phase = 0;
a.bit = 1;
a.bitRemain = 0;
a.bitSamples = Math.max(1, Math.round(ctx.sampleRate / 600));
a.lfsr = (Date.now() ^ 0xACE1) & 0xFFFF;
node.onaudioprocess = function(ev) {
const out = ev.outputBuffer.getChannelData(0);
const sr = ctx.sampleRate || 44100;
for (let i = 0; i < out.length; i++) {
if (--a.bitRemain <= 0) {
a.bit = casSoundNextBitF49K(a);
a.bitRemain = a.bitSamples;
}
// Original Atari tape FSK: space≈3995 Hz, mark≈5327 Hz. Add tiny wow/flutter.
const base = a.bit ? 5327 : 3995;
const flutter = Math.sin((ctx.currentTime + i / sr) * 2 * Math.PI * 2.1) * 23 + (Math.random() - 0.5) * 10;
const freq = base + flutter;
a.phase += (2 * Math.PI * freq) / sr;
if (a.phase > Math.PI * 2) a.phase -= Math.PI * 2;
const s = Math.sin(a.phase);
const clipped = s > 0 ? 0.92 : -0.92;
out[i] = (0.72 * clipped + 0.20 * s + (Math.random() - 0.5) * 0.018);
}
};
node.connect(filter);
filter.connect(gain);
gain.connect(ctx.destination);
a.ctx = ctx; a.node = node; a.filter = filter; a.gain = gain; a.playing = true;
casSoundSetVolumeF49I(cfg.volume);
} catch(e) { console.warn("cas sound start", e); }
}
function casSoundStopF49I(fade) {
const a = window.casAudioF49I;
if (!a) return;
try {
if (a.gain && a.ctx) {
if (fade) a.gain.gain.setTargetAtTime(0, a.ctx.currentTime, 0.035);
else a.gain.gain.value = 0;
}
const node = a.node, filter = a.filter, gain = a.gain;
setTimeout(() => {
try { if (node) { node.disconnect(); node.onaudioprocess = null; } } catch(e) {}
try { if (filter) filter.disconnect(); } catch(e) {}
try { if (gain) gain.disconnect(); } catch(e) {}
}, fade ? 180 : 0);
} catch(e) {}
a.node = null; a.filter = null; a.gain = null; a.playing = false;
}
async function casSoundRewindF49I() {
try {
return;
casBindSoundUiF49I();
const cfg = casCurrentSoundCfgF49I();
if (!cfg.enabled || Number(cfg.volume || 0) <= 0) return;
casSoundStopF49I(false);
const ctx = await casSoundEnsureCtxF49I();
const gain = ctx.createGain();
gain.gain.value = Math.max(0, Math.min(100, Number(cfg.volume || 35))) / 100 * 0.075;
gain.connect(ctx.destination);
const osc = ctx.createOscillator();
osc.type = "sawtooth";
osc.frequency.setValueAtTime(5327, ctx.currentTime);
osc.frequency.exponentialRampToValueAtTime(3995, ctx.currentTime + 0.18);
osc.frequency.exponentialRampToValueAtTime(900, ctx.currentTime + 0.46);
osc.connect(gain);
osc.start();
gain.gain.setTargetAtTime(0, ctx.currentTime + 0.30, 0.045);
setTimeout(() => { try { osc.stop(); osc.disconnect(); gain.disconnect(); } catch(e) {} }, 620);
} catch(e) { console.warn("cas sound rewind", e); }
}
async function casSoundTestF49I() {
await casSoundStartF49I();
setTimeout(() => casSoundStopF49I(true), 1800);
}
function updateCasManualPanelF49(a) {
try {
casBindTurboUiF49C();
if (typeof casBindSoundUiF49I === "function") casBindSoundUiF49I();
a = a || {};
const mounted = !!Number(a.casMounted || a.mounted || 0);
const playing = !!Number(a.casPlaying || a.playing || 0);
const paused = !!Number(a.casPaused || a.paused || 0);
const eof = !!Number(a.casEof || a.eof || 0);
const name = a.casName || a.name || "";
const pos = Number(a.casPlayPos ?? a.playPos ?? 0);
const size = Number(a.casPlaySize ?? a.playSize ?? 0);
const payloadTotal = Number(a.casPayloadBytes ?? a.casAutoPayloadBytes ?? 0);
const fileTotal = Number(a.casFileBytes ?? size ?? 0);
const queued = Number(a.casBytesQueued ?? a.bytesQueued ?? 0);
const played = Number(a.casRpBytesPlayed ?? a.rpBytesPlayed ?? 0);
const rx = Number(a.casRpBytesRx ?? a.rpBytesRx ?? 0);
const baud = Number(a.casBaud ?? a.baud ?? 0);
const rpBaud = Number(a.casRpBaud ?? a.rpBaud ?? 0);
const rpUsed = Number(a.casRpBufferUsed ?? a.rpBufferUsed ?? 0);
const rpFree = Number(a.casRpBufferFree ?? a.rpBufferFree ?? 0);
const blocks = Number(a.casDataBlocks ?? a.dataBlocks ?? 0);
const chunks = Number(a.casChunksQueued ?? a.chunksQueued ?? 0);
const err = a.casLastError || a.lastError || "";
const tapeCounter = Number(a.casTapeCounter ?? a.tapeCounter ?? (pos > 0 ? Math.floor(pos / 128) : 0));
window.casLastStatusF49J = a;
const cfg = casCurrentTurboCfgF49C();
const autoProfile = a.casAutoProfile || "—";
const autoConfidence = a.casAutoConfidence || "—";
const autoInitial = Number(a.casAutoInitialBaud || 600);
const autoTurbo = Number(a.casAutoTurboBaud || autoInitial || 600);
const autoTotalChunks = Number(a.casAutoTotalChunks || 0);
const autoFskChunks = Number(a.casAutoFskChunks || 0);
const autoPwmChunks = Number(a.casAutoPwmChunks || 0);
const autoPwmState = Number(a.casAutoPwmStateChunks || 0);
const autoPwmControl = Number(a.casAutoPwmControlChunks || 0);
const autoPwmData = Number(a.casAutoPwmDataChunks || 0);
const autoPwmLong = Number(a.casAutoPwmLongChunks || 0);
const autoMinIrg = Number(a.casAutoMinIrgMs || 0);
const autoMaxIrg = Number(a.casAutoMaxIrgMs || 0);
const autoLastIrg = Number(a.casAutoLastIrgMs || 0);
const autoPauseCandidates = Number(a.casAutoAutoPauseCandidates || 0);
const autoSignals = [autoPwmChunks ? `PWM(${autoPwmChunks})` : "", autoFskChunks ? `FSK(${autoFskChunks})` : "", a.casAutoTextIron ? "IRON" : "", a.casAutoTextTurbo ? "TURBO" : ""].filter(Boolean).join(" · ") || "Estándar";
const state = !mounted ? "Sin cassette" : (paused ? "Pausado" : (playing ? "Reproduciendo" : (eof ? "Terminado" : "Preparado")));
const pct = size > 0 ? Math.max(0, Math.min(100, (pos * 100) / size)) : 0;
const rpTotal = rpUsed + rpFree;
const rpPct = rpTotal > 0 ? Math.round((rpUsed * 100) / rpTotal) : 0;
const elapsed = playing && a.casPlayStartedMs ? 0 : 0;
const effectiveBps = (rpBaud || baud || cfg.baud || 600);
const payloadDen = payloadTotal > 0 ? payloadTotal : (queued || size || 0);
const eta = (size > pos && effectiveBps > 0) ? casFmtTimeF49C(((size - pos) * 10) / effectiveBps) + " restante" : (eof ? "Terminado" : "—");
document.body.classList.toggle("cas-playing", playing && !paused);
document.body.classList.toggle("cas-paused", paused);
if ((!playing || paused) && typeof casSoundStopF49I === "function") casSoundStopF49I(true);
if ($("casControlPill")) { $("casControlPill").textContent = mounted ? "C: preparado" : "Sin cassette"; $("casControlPill").className = mounted ? "pill ok" : "pill"; }
if ($("casControlName")) $("casControlName").textContent = mounted ? name : "—";
if ($("casControlState")) $("casControlState").textContent = err ? (state + " · " + err) : state;
if ($("casControlProgress")) $("casControlProgress").textContent = (size > 0 ? `${humanBytes(pos)} / ${humanBytes(size)}` : (queued ? `${humanBytes(queued)} enviados` : "—")) + (baud ? ` · ${baud} baud` : "");
if ($("casControlRp")) $("casControlRp").textContent = (rpUsed || rpFree) ? `buffer ${rpUsed}/${rpUsed + rpFree}` : "—";
if ($("casPagePill")) { $("casPagePill").textContent = mounted ? (paused ? "Pausado" : (playing ? "Reproduciendo" : "C: preparado")) : "Sin cassette"; $("casPagePill").className = mounted ? (playing && !paused ? "pill ok" : "pill warn") : "pill"; }
if ($("casTapeFileName")) { $("casTapeFileName").textContent = mounted ? name : "Sin cassette"; $("casTapeFileName").title = mounted ? name : ""; }
if ($("casPageState")) { $("casPageState").textContent = err ? (state + " · " + err) : state; $("casPageState").className = (playing && !paused) ? "ok" : (mounted ? "warn" : ""); }
if ($("casPageMode")) { const lm = (a.casLoadMode || a.loadMode || ($("casLoadMode") ? $("casLoadMode").value : "cload")); const speedLabel = cfg.auto ? `Auto · ${autoProfile} (${autoConfidence}) · ${cfg.baud} bps` : `${cfg.baud} bps · ${casBaudProfileLabelF49Z11(cfg.baud)}`; $("casPageMode").textContent = `${lm === "boot" ? "Boot cassette" : (lm === "auto" ? "Auto" : "CLOAD")} · ${speedLabel}`; }
if ($("casPageProgressFill")) $("casPageProgressFill").style.width = `${pct}%`;
if ($("casProgressText")) $("casProgressText").innerHTML = size > 0 ? `<b>${Math.round(pct)}%</b> archivo · ${humanBytes(pos)} / ${humanBytes(size)} · datos ${humanBytes(played || 0)}${payloadDen ? ` / ${humanBytes(payloadDen)}` : ""}${paused ? " · PAUSA" : ""}` : (mounted ? "Preparado, esperando Play manual" : "Sin cassette preparado");
if ($("casTapeCounter")) $("casTapeCounter").textContent = String(Math.max(0, Math.floor(tapeCounter))).padStart(4,"0");
if ($("casLiveRefreshPill")) { const ms = (typeof casLiveDelayF49J === "function") ? casLiveDelayF49J(playing && !paused) : ((playing && !paused) ? 750 : 2500); $("casLiveRefreshPill").innerHTML = `<span class="cas-live-dot"></span>Auto ${(ms/1000).toFixed(ms<1000?2:1)}s`; $("casLiveRefreshPill").className = (playing && !paused) ? "pill ok" : "pill"; }
if ($("casLoadMode")) {
const mode = $("casLoadMode").value || "cload";
if ($("casCompatNote")) $("casCompatNote").innerHTML = mode === "boot" ? `<b>Boot cassette:</b> carga desde arranque.` : (mode === "auto" ? `<b>Auto/diagnóstico:</b> usa detección automática.` : `<b>CLOAD BASIC:</b> BASIC/CSAVE.`);
}
if ($("casPageTimeLeft")) $("casPageTimeLeft").textContent = size > 0 ? casFmtTimeF49C((pos * 10) / Math.max(1,effectiveBps)) : "00:00";
if ($("casPageTimeRight")) $("casPageTimeRight").textContent = size > 0 ? casFmtTimeF49C((size * 10) / Math.max(1,effectiveBps)) : "--:--";
if ($("casSignalPill")) { $("casSignalPill").textContent = paused ? "Pausa" : (playing ? "Actividad detectada" : "Sin actividad"); $("casSignalPill").className = (playing && !paused) ? "pill ok" : "pill"; }
if ($("casSignalBaud")) $("casSignalBaud").textContent = `${effectiveBps} bps`;
if ($("casSignalLevel")) $("casSignalLevel").textContent = (playing && !paused) ? "OK" : (paused ? "PAUSA" : "—");
if ($("casSignalMode")) $("casSignalMode").textContent = autoPwmChunks ? "PWM turbo" : (autoFskChunks ? "FSK" : "Normal");
if ($("casSignalRp")) $("casSignalRp").textContent = rpTotal ? `${rpPct}%` : "—";
if ($("casAutoPill")) { $("casAutoPill").textContent = a.casAutoValid ? autoConfidence : "pendiente"; $("casAutoPill").className = a.casAutoValid ? (String(autoConfidence).toUpperCase() === "ALTA" ? "pill ok" : "pill warn") : "pill"; }
if ($("casAutoFormat")) $("casAutoFormat").textContent = a.casAutoFuji ? "FUJI-CAS" : (a.casAutoRaw ? "RAW/BIN" : (mounted ? "CAS" : "—"));
if ($("casAutoProfile")) $("casAutoProfile").textContent = autoProfile;
if ($("casAutoConfidence")) $("casAutoConfidence").textContent = autoConfidence;
if ($("casAutoInitialBaud")) $("casAutoInitialBaud").textContent = `${autoInitial || 600} bps`;
if ($("casAutoTurboBaud")) $("casAutoTurboBaud").textContent = `${autoTurbo || autoInitial || 600} bps`;
if ($("casAutoBlocks")) $("casAutoBlocks").textContent = `${a.casAutoDataBlocks || 0} data · ${a.casAutoBaudChunks || 0} baud · ${autoFskChunks} fsk · ${autoPwmChunks} pwm`;
if ($("casAutoSignals")) $("casAutoSignals").textContent = autoSignals;
if ($("casAutoGaps")) $("casAutoGaps").textContent = `${a.casAutoLongGaps || 0} largos · ${a.casAutoShortGaps || 0} cortos · ${autoPauseCandidates} ≥3s`;
if ($("casAutoChunks")) $("casAutoChunks").textContent = a.casAutoChunkSummary || (autoTotalChunks ? `${autoTotalChunks} chunks` : "—");
if ($("casAutoPwm")) $("casAutoPwm").textContent = autoPwmChunks ? `pwms ${autoPwmState} · pwmc ${autoPwmControl} · pwmd ${autoPwmData} · pwml ${autoPwmLong}` : "Sin PWM";
if ($("casAutoIrg")) $("casAutoIrg").textContent = autoMaxIrg ? `min ${autoMinIrg || 0} ms · max ${autoMaxIrg} ms · último ${autoLastIrg || 0} ms` : "—";
if ($("casAutoPauseBase")) { $("casAutoPauseBase").textContent = autoPauseCandidates ? `Base lista · ${autoPauseCandidates} gaps ≥3s · OFF` : "Sin candidatos · OFF"; $("casAutoPauseBase").className = autoPauseCandidates ? "warn" : ""; }
if ($("casAutoNotes")) $("casAutoNotes").textContent = a.casAutoNotes || "Monta un .CAS para analizarlo.";
if ($("casTurboState")) { $("casTurboState").textContent = cfg.auto ? `Auto: ${autoProfile}` : casBaudProfileLabelF49Z11(cfg.baud); $("casTurboState").className = cfg.auto ? "pill ok" : (cfg.baud > 600 ? "pill warn" : "pill"); }
if ($("casDiagQueued")) $("casDiagQueued").textContent = humanBytes(queued || rx || 0);
if ($("casDiagPlayed")) $("casDiagPlayed").textContent = played ? `${humanBytes(played)}${payloadDen ? ` (${Math.round((played/Math.max(1,payloadDen))*100)}%)` : ""}` : "—";
if ($("casDiagBufferBar")) $("casDiagBufferBar").style.width = `${rpPct}%`;
if ($("casDiagBufferTxt")) $("casDiagBufferTxt").textContent = rpTotal ? `${rpPct}% · ${rpUsed}/${rpTotal}` : "—";
if ($("casDiagPos")) $("casDiagPos").textContent = size > 0 ? `${humanBytes(pos)} / ${humanBytes(size)}` : "—";
if ($("casDiagSize")) $("casDiagSize").textContent = (fileTotal || size) ? humanBytes(fileTotal || size) : "—";
if ($("casDiagPayload")) $("casDiagPayload").textContent = payloadDen ? `${humanBytes(payloadDen)}${fileTotal && payloadDen && fileTotal > payloadDen ? ` · overhead ${humanBytes(fileTotal - payloadDen)}` : ""}` : "—";
if ($("casDiagBlocks")) $("casDiagBlocks").textContent = `${blocks || 0} bloques · ${chunks || 0} chunks`;
if ($("casDiagBaud")) $("casDiagBaud").textContent = `${effectiveBps} bps`;
if ($("casDiagEta")) $("casDiagEta").textContent = eta;
if ($("casDiagError")) $("casDiagError").textContent = err || "—";
if ($("casDiagSdrive")) $("casDiagSdrive").textContent = a.sdriveExactMode ? "SDrive seguro" : (a.sdriveRawRecordMode ? "SDrive RAW" : "—");
if ($("casDiagRaw")) $("casDiagRaw").textContent = a.sdriveRawRecordMode ? `${a.rawRecordsBuilt || 0} registros · marcador ${a.rawLastMarker || 0}` : "—";
if ($("casDiagWav")) $("casDiagWav").textContent = a.wavDirectSupported ? "ON" : "No directo";
["btnCasBootPage","btnCasPlayPage","btnCasStopPage","btnCasRewindPage","btnCasPausePage","btnCasBack3Page","btnCasUnmountPage"].forEach(id => { if ($(id)) $(id).disabled = !mounted; });
["btnCasPlayPage"].forEach(id => { if ($(id)) $(id).textContent = playing ? "▶ Reenviar CLOAD" : "▶ Play CLOAD BASIC"; });
if ($("btnCasBootPage")) $("btnCasBootPage").textContent = playing ? "🚀 Reenviar Boot" : "🚀 Play Boot cassette";
if ($("btnCasPausePage")) { $("btnCasPausePage").disabled = !mounted || !playing; $("btnCasPausePage").textContent = paused ? "▶ Continuar" : "⏸ Pausa"; }
if ($("btnCasBack3Page")) { $("btnCasBack3Page").disabled = !mounted || !playing; }
} catch(e) { console.warn("cas panel", e); }
}
async function loadCasStatusF49C() {
try {
const r = await fetch("/api/cas/status", { cache:"no-store" });
if (!r.ok) throw new Error("HTTP " + r.status);
const a = await r.json();
window.casLastStatusF49J = a;
updateCasManualPanelF49(a);
return a;
} catch(e) { setStatus("Error leyendo Cassette: " + e.message, false); }
}
// F49J: refresco dinámico dedicado para Cassette.
// No depende del refresco global de 15s; mientras reproduce consulta /api/cas/status cada 750 ms.
window.casLiveRefreshF49J = window.casLiveRefreshF49J || { timer:null, busy:false, running:false };
function casIsRouteF49J(){
try { return (typeof routeKindF44 === "function" ? routeKindF44(location.pathname) : "") === "cassette" || document.body.classList.contains("route-cassette"); } catch(e) { return false; }
}
function casLiveDelayF49J(playing){ return playing ? 750 : 2500; }
function casLastPlayingF49J(){
const a = window.casLastStatusF49J || {};
return !!Number(a.casPlaying || a.playing || 0);
}
function casScheduleLiveRefreshF49J(delay){
const st = window.casLiveRefreshF49J;
if (st.timer) clearTimeout(st.timer);
if (!casIsRouteF49J()) { st.running = false; return; }
st.running = true;
st.timer = setTimeout(casLiveRefreshTickF49J, Math.max(300, Number(delay || casLiveDelayF49J(casLastPlayingF49J()))));
}
async function casLiveRefreshTickF49J(){
const st = window.casLiveRefreshF49J;
if (!casIsRouteF49J() || document.hidden) { casScheduleLiveRefreshF49J(2500); return; }
if (st.busy) { casScheduleLiveRefreshF49J(700); return; }
st.busy = true;
try {
if (typeof loadCasStatusF49C === "function") await loadCasStatusF49C();
} catch(e) { console.warn("cas live", e); }
finally {
st.busy = false;
casScheduleLiveRefreshF49J(casLiveDelayF49J(casLastPlayingF49J()));
}
}
function casStartLiveRefreshF49J(force){
if (!casIsRouteF49J()) return;
if (force && typeof loadCasStatusF49C === "function") loadCasStatusF49C();
casScheduleLiveRefreshF49J(force ? 500 : casLiveDelayF49J(casLastPlayingF49J()));
}
function casStopLiveRefreshF49J(){
const st = window.casLiveRefreshF49J;
if (st.timer) clearTimeout(st.timer);
st.timer = null; st.running = false;
}
(function patchCasLiveRefreshF49J(){
if (window.__casLiveRefreshF49J) return;
window.__casLiveRefreshF49J = true;
const bindMode = () => {
try {
const sel = $("casLoadMode");
if (sel && !sel.dataset.boundF49J) {
sel.dataset.boundF49J = "1";
sel.value = localStorage.getItem("casLoadModeF49J") || "boot";
sel.addEventListener("change", () => { localStorage.setItem("casLoadModeF49J", sel.value || "cload"); updateCasManualPanelF49(window.casLastStatusF49J || {}); });
}
} catch(e) {}
};
const oldLoadRoute = window.loadRouteDataF44;
if (typeof oldLoadRoute === "function") {
window.loadRouteDataF44 = function(kind){
const r = oldLoadRoute.apply(this, arguments);
bindMode();
if (kind === "cassette") casStartLiveRefreshF49J(true); else casStopLiveRefreshF49J();
return r;
};
}
const oldRoute = window.applyRouteLayout;
if (typeof oldRoute === "function") {
window.applyRouteLayout = function(){
const r = oldRoute.apply(this, arguments);
bindMode();
if (casIsRouteF49J()) casStartLiveRefreshF49J(true); else casStopLiveRefreshF49J();
return r;
};
}
document.addEventListener("visibilitychange", () => { if (!document.hidden && casIsRouteF49J()) casStartLiveRefreshF49J(true); });
const boot = () => { bindMode(); if (casIsRouteF49J()) casStartLiveRefreshF49J(true); };
if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", boot); else boot();
})();
async function loadAtrStatus(opts = {}) {
try {
const pth = (location.pathname || "").toLowerCase();
// F14: WEB-ATR no necesita lista completa en cada refresco.
// La lista se pide solo en carga inicial/botón refrescar/biblioteca/subida.
const routeNeedsFiles = pth.indexOf("/library") === 0 || pth.indexOf("/editor/library") === 0 || pth.indexOf("/upload") === 0 || pth.indexOf("/atr") === 0 || pth.indexOf("/editor/atr") === 0;
const needFiles = (typeof opts.files === "boolean") ? opts.files : routeNeedsFiles;
const refreshFiles = !!opts.refreshFiles;
const fastStatus = !!opts.fast;
const url = "/api/atr/status?files=" + (needFiles ? "1" : "0") + (refreshFiles ? "&refresh=1" : "") + (fastStatus ? "&fast=1" : "");
const r = await fetch(url, { cache: "no-store" });
if (!r.ok) throw new Error("HTTP " + r.status);
const a = await r.json();
if (!Array.isArray(a.files)) {
a.files = atrLibraryFilesCache.slice();
} else {
atrLibraryFilesCache = a.files.slice();
atrLibraryFilesCacheSig = atrLibraryFilesSignature(atrLibraryFilesCache);
}
lastAtrStatus = a;
updateCasManualPanelF49(a);
if (typeof atrUpdatePriorityUiV30 === 'function') atrUpdatePriorityUiV30(a);
const fsUsedTxt = `${humanBytes(a.fsUsed || 0)} / ${humanBytes(a.fsTotal || 0)}`;
const fsFreeTxt = humanBytes(a.fsFree || 0);
["atrFsState", "atrUploadFsState"].forEach(id => { if ($(id)) $(id).textContent = fsUsedTxt; });
["atrFsFree", "atrUploadFsFree", "atrMountFsFree"].forEach(id => { if ($(id)) $(id).textContent = fsFreeTxt; });
["atrEnabledState", "atrMountEnabledState"].forEach(id => { if ($(id)) $(id).textContent = a.enabled ? "Sí" : "No"; });
if ($("atrReads")) $("atrReads").textContent = a.reads ?? "—";
const files = (a.files || []).slice().sort((x, y) => String(x.name || "").localeCompare(String(y.name || ""), "es", { sensitivity: "base", numeric: true }));
const slots = a.slots || [];
const visibleMask = visibleUnitMaskFromStatus(a);
updateVisibleUnitCards(visibleMask, lastDriveVisibility?.units || null);
if (opts.visibility !== false && needFiles) loadDriveVisibilityStatus();
const filesSig = atrLibraryFilesSignature(files);
const shouldRefreshSelectOptions = !!opts.forceSelectOptions || needFiles || filesSig !== atrSelectOptionsLastSig;
if (shouldRefreshSelectOptions) atrSelectOptionsLastSig = filesSig;
for (let i=1;i<=UI_MAX_UNITS;i++) {
const sl = slots[i-1] || {};
const sel = $(`atrSel${i}`);
if (sel && shouldRefreshSelectOptions) sel.innerHTML = atrFileOptions(files, sl.name || "");
if (sel && !shouldRefreshSelectOptions && sel.value !== (sl.name || "")) sel.value = sl.name || "";
if ($(`atrEn${i}`)) $(`atrEn${i}`).checked = !!sl.enabled;
if ($(`atrForce${i}`)) $(`atrForce${i}`).checked = !!sl.forced;
const name = sl.name || "—";
const meta = sl.present ? `${humanBytes(sl.fileSize || 0)}, ${sl.sectorSize || "?"}B, ${sl.totalSectors || "?"} sectores` : "sin ATR válido";
if ($(`atrSlot${i}`)) $(`atrSlot${i}`).textContent = sl.present ? "ATR OK" : (sl.name ? "ATR inválido" : "Sin ATR");
if ($(`atrSlotName${i}`)) $(`atrSlotName${i}`).textContent = name;
if ($(`atrSlotMeta${i}`)) $(`atrSlotMeta${i}`).textContent = meta;
const coverName = sl.name || "";
if (atrSlotCoverNameLast[i] !== coverName || opts.forceCovers) {
atrSlotCoverNameLast[i] = coverName;
updateAtrCover(i, coverName, !!(coverName || sl.present));
}
const route = $(`atrRoute${i}`);
if (route) {
route.textContent = sl.enabled ? "WEB-ATR ON" : "WEB-ATR OFF";
route.className = sl.enabled ? "pill ok" : "pill bad";
}
const fb = $(`atrFallback${i}`);
if (fb) {
fb.textContent = sl.forced ? "sin fallback" : "fallback físico";
fb.className = sl.forced ? "pill warn" : "pill";
}
}
const countText = files.length === 1 ? "1 archivo" : `${files.length} archivos`;
["atrLibraryCount", "atrUploadLibraryCount", "atrMountLibraryCount"].forEach(id => { if ($(id)) $(id).textContent = countText; });
if ($("homeLibraryCount")) $("homeLibraryCount").textContent = countText;
const libSummary = $("atrLibrarySummary");
if (libSummary) libSummary.textContent = files.length ? `${files.length} archivo(s)` : `Sin archivos`;
const viewSel = $("atrLibraryView");
if (viewSel) viewSel.value = atrLibraryViewMode === "folders" ? "folders" : "flat";
if (needFiles || opts.renderLibrary) renderAtrLibrary(files, { force: !!opts.forceLibrary });
} catch(e) {
console.error(e);
if ($("atrFsState")) $("atrFsState").textContent = "Error";
setStatus("Error WEB-ATR: " + e.message, false);
}
}
function atrApplyLightStatusFast(a, opts = {}) {
if (!a || !Array.isArray(a.slots)) return;
lastAtrStatus = Object.assign({}, lastAtrStatus || {}, a);
const slots = a.slots || [];
const visibleMask = visibleUnitMaskFromStatus(a);
updateVisibleUnitCards(visibleMask, lastDriveVisibility?.units || null);
for (let i=1;i<=UI_MAX_UNITS;i++) {
const sl = slots[i-1] || {};
const sel = $(`atrSel${i}`);
if (sel && typeof sl.name === "string" && sel.value !== sl.name) sel.value = sl.name;
if ($(`atrEn${i}`)) $(`atrEn${i}`).checked = !!sl.enabled;
if ($(`atrForce${i}`)) $(`atrForce${i}`).checked = !!sl.forced;
const name = sl.name || "—";
if ($(`atrSlot${i}`)) $(`atrSlot${i}`).textContent = sl.present ? "ATR OK" : (sl.name ? "Montado / validando" : "Sin ATR");
if ($(`atrSlotName${i}`)) $(`atrSlotName${i}`).textContent = name;
if ($(`atrSlotMeta${i}`)) $(`atrSlotMeta${i}`).textContent = sl.name ? "Montaje aplicado" : "sin ATR válido";
const route = $(`atrRoute${i}`);
if (route) {
route.textContent = sl.enabled ? "WEB-ATR ON" : "WEB-ATR OFF";
route.className = sl.enabled ? "pill ok" : "pill bad";
}
const fb = $(`atrFallback${i}`);
if (fb) {
fb.textContent = sl.forced ? "sin fallback" : "fallback físico";
fb.className = sl.forced ? "pill warn" : "pill";
}
const coverName = sl.name || "";
if (atrSlotCoverNameLast[i] !== coverName || opts.forceCovers) {
atrSlotCoverNameLast[i] = coverName;
updateAtrCover(i, coverName, !!coverName);
}
}
}
function atrSchedulePostMountRefresh(opts = {}) {
const delay = Number.isFinite(opts.delay) ? opts.delay : 1600;
setTimeout(() => loadAtrStatus({ files:false, visibility:false, force:true, fast:true }), delay);
if (opts.bt !== false) setTimeout(() => loadBtSioStatus(), delay + 250);
const pth = (location.pathname || "").toLowerCase();
if (opts.library && pth.indexOf("/library") === 0) {
setTimeout(() => loadAtrStatus({ files:true, refreshFiles:false, forceLibrary:true }), delay + 650);
}
}
async function applyAtrMount(opts = {}) {
const auto = !!opts.auto;
const built = atrBuildMountParamsFromUi();
try {
atrSaving = true;
if (auto) atrSetAutoSaveState("Guardando...", "warn");
built.params.set("reply", "light");
const r = await fetch(`/set_webatr?${built.params.toString()}`, { cache: "no-store" });
if (!r.ok) throw new Error(await r.text());
const a = await r.json().catch(() => null);
atrApplyLightStatusFast(a, { forceCovers:true });
atrSchedulePostMountRefresh({ bt:false, delay:1200 });
atrSetAutoSaveState(a?.deferredSave ? "Guardado / NVS pendiente" : "Guardado", "ok");
setStatus(auto ? "Montaje WEB-ATR aplicado" : "Montaje WEB-ATR guardado", true);
} catch(e) {
atrSetAutoSaveState("Error al guardar", "bad");
setStatus("Error guardando montaje: " + e.message, false);
} finally {
atrSaving = false;
if (auto) setTimeout(() => atrSetAutoSaveState("Auto-guardado activo", "ok"), 1500);
}
}
async function setWebAtr(en, mask, force) {
try {
const r = await fetch(`/set_webatr?en=${en}&mask=${mask}&force=${force}&reply=light`, { cache: "no-store" });
if (!r.ok) throw new Error(await r.text());
const a = await r.json().catch(() => null);
atrApplyLightStatusFast(a, { forceCovers:true });
atrSchedulePostMountRefresh({ bt:false, delay:1600 });
setStatus("WEB-ATR actualizado", true);
} catch(e) { setStatus("Error configurando WEB-ATR: " + e.message, false); }
}
function atrBootHintF49Z23(fileName, unit) {
const n = String(fileName || "");
const u = Number(unit || 0);
const lower = n.toLowerCase();
if (/\.cas$/i.test(n)) return `C: preparado: ${n}. Entra a Cassette para Play/Boot.`;
if (u !== 1) return `ATR montado en D${u}. Para arrancar desde el Atari usa D1; D${u} sirve desde DOS/programas.`;
if (/\.(xex|com|exe)(\.atr)?$/i.test(n) || lower.endsWith(".xex.atr") || lower.endsWith(".com.atr") || lower.endsWith(".exe.atr")) {
return "XEX montable en D1. Si el Atari queda en BASIC o muestra OPTION/RESET, reinicia manteniendo OPTION.";
}
return "ATR montado en D1. Si aparece OPTION/RESET después de varios beeps, reinicia manteniendo OPTION.";
}
async function mountAtrQuick(encodedName, unit) {
const name = decodeURIComponent(encodedName || "");
if (/\.cas$/i.test(name) && typeof mountCasQuick === "function") {
return mountCasQuick(encodedName);
}
const mask = (lastAtrStatus?.mask || 0) | (1 << (unit-1));
const force = (lastAtrStatus?.forceMask || 0) | (1 << (unit-1));
const params = new URLSearchParams();
params.set("en", "1"); params.set("mask", String(mask)); params.set("force", String(force)); params.set("unit", String(unit)); params.set("file", name); params.set("reply", "light");
try {
const r = await fetch(`/set_webatr?${params.toString()}`, { cache: "no-store" });
if (!r.ok) throw new Error(await r.text());
const a = await r.json().catch(() => null);
atrApplyLightStatusFast(a, { forceCovers:true });
atrSchedulePostMountRefresh({ bt:false, library:true, delay:1200 });
atrSetAutoSaveState("Guardado", "ok");
setStatus(atrBootHintF49Z23(name, unit), true);
} catch(e) { setStatus("Error montando ATR: " + e.message, false); }
}
async function unmountAtrUnit(unit) {
try {
const r = await fetch(`/atr/unmount?unit=${unit}&reply=light`, { cache: "no-store" });
if (!r.ok) throw new Error(await r.text());
const a = await r.json().catch(() => null);
atrApplyLightStatusFast(a, { forceCovers:true });
atrSchedulePostMountRefresh({ bt:false, library:true, delay:1600 });
atrSetAutoSaveState("Guardado", "ok");
setStatus(`ATR desmontado de D${unit}`, true);
} catch(e) { setStatus("Error desmontando ATR: " + e.message, false); }
}
async function deleteAtr(encodedName) {
const name = decodeURIComponent(encodedName || "");
if (!name) return;
const meta = (lastAtrStatus?.files || []).find(f => f.name === name);
if (meta?.protected) {
setStatus("Este ATR viene incluido en el MASTER y no se puede eliminar desde la web.", false);
return;
}
if (!confirm(`¿Borrar ${name} de la flash interna?`)) return;
try {
const r = await fetch(`/atr/delete?file=${encodeURIComponent(name)}&reply=light`, { cache: "no-store" });
if (!r.ok) throw new Error(await r.text());
// F49Z46: no reconstruir indice automaticamente al borrar.
setStatus("Archivo borrado. Pulsa Refrescar biblioteca para actualizar la lista.", true);
} catch(e) { setStatus("Error borrando ATR: " + e.message, false); }
}
function setUploadUi(percent, text, active) {
const wrap = $("atrUploadProgressWrap");
const bar = $("atrUploadProgressBar");
const label = $("atrUploadProgressText");
const state = $("atrUploadState");
if (wrap) wrap.style.display = active ? "block" : "none";
if (bar) bar.style.width = Math.max(0, Math.min(100, percent || 0)) + "%";
if (label) label.textContent = text || "";
if (state) state.textContent = text || "Listo";
}
function bindAtrUploadForm() {
const form = $("atrUploadForm");
if (!form || form.dataset.boundUpload === "1") return;
form.dataset.boundUpload = "1";
form.addEventListener("submit", async (ev) => {
ev.preventDefault();
const input = $("atrFile");
if (!input || !input.files || input.files.length === 0) {
setStatus("Selecciona al menos un archivo para subir.", false);
return;
}
const btn = $("btnAtrUpload");
if (btn) btn.disabled = true;
const files = Array.from(input.files || []);
let okCount = 0;
try {
for (let idx = 0; idx < files.length; idx++) {
const file = files[idx];
await new Promise((resolve, reject) => {
const data = new FormData();
data.append("atr", file, file.name);
const xhr = new XMLHttpRequest();
xhr.upload.onprogress = (e) => {
const base = (idx / files.length) * 100;
const span = 100 / files.length;
const local = e.lengthComputable ? (e.loaded / e.total) : 0.08;
const pct = Math.round(base + span * Math.max(0, Math.min(1, local)));
setUploadUi(pct, `Subiendo ${idx + 1}/${files.length}: ${file.name} · ${pct}%`, true);
};
xhr.onerror = () => reject(new Error("Error de red"));
xhr.onload = () => {
if (xhr.status >= 200 && xhr.status < 400) { okCount++; resolve(); }
else reject(new Error(xhr.responseText || ("HTTP " + xhr.status)));
};
xhr.open("POST", (form.action || "/atr/upload") + "?json=1", true);
xhr.setRequestHeader("X-Requested-With", "XMLHttpRequest");
xhr.setRequestHeader("X-Upload-Mode", "single-json");
xhr.send(data);
});
}
setUploadUi(100, `Subida lista: ${okCount}/${files.length}`, true);
form.reset();
updateFileLabel("atrFile", "atrFileName", "Ningún archivo seleccionado");
// F49Z46: no actualizar automaticamente la Biblioteca ni el JSON.
// Solo el boton "Refrescar biblioteca" escanea la SD y reconstruye el indice.
setStatus(`Archivos subidos correctamente: ${okCount}. Pulsa Refrescar biblioteca para actualizar la lista.`, true);
} catch(e) {
setUploadUi(0, "Error al subir", true);
setStatus("Error al subir archivo: " + e.message, false);
} finally {
if (btn) btn.disabled = false;
}
});
}
// Sincronización externa: permite que la web refleje cambios hechos desde la app Android.
// No recarga la página; solo vuelve a leer el estado real del MASTER.
let webExternalSyncBusy = false;
function webExternalSyncRoute() {
const p = (location.pathname || "/").toLowerCase();
if (p.indexOf("/atr") === 0) return "atr";
if (p.indexOf("/upload") === 0) return "upload";
if (p.indexOf("/library") === 0) return "library";
if (p.indexOf("/bt") === 0) return "bt";
if (p.indexOf("/tnfs") === 0 || p.indexOf("/editor/tnfs") === 0) return "tnfs";
if (p.indexOf("/settings") === 0 || p.indexOf("/appearance") === 0 || p.indexOf("/apariencia") === 0 || p.indexOf("/editor/settings") === 0 || p.indexOf("/editor/appearance") === 0 || p.indexOf("/editor/apariencia") === 0) return "settings";
if (p.indexOf("/disk") === 0) return "disk";
if (p === "/" || p === "") return "home";
return "other";
}
function webExternalSyncUserEditing() {
const el = document.activeElement;
if (!el) return false;
const tag = String(el.tagName || "").toUpperCase();
if (["INPUT", "SELECT", "TEXTAREA"].includes(tag)) return true;
if (el.isContentEditable) return true;
return false;
}
async function webExternalSyncTick() {
if (document.hidden) return;
if (webExternalSyncBusy) return;
if (webExternalSyncUserEditing()) return;
if (atrSaving || btSioSaving) return;
webExternalSyncBusy = true;
try {
const route = webExternalSyncRoute();
if (route === "settings" || route === "other") return;
await loadDriveVisibilityStatus();
if (route === "atr" || route === "upload" || route === "library") {
await loadAtrStatus({ files:false, visibility:false });
if (route === "atr") await loadBtSioStatus();
} else if (route === "cassette") {
if (typeof loadCasStatusF49C === "function") await loadCasStatusF49C();
else await loadAtrStatus({ files:false, visibility:false });
} else if (route === "bt") {
await loadBtSioStatus();
await loadAtrStatus({ files:false, visibility:false });
} else if (route === "disk") {
await loadStatus();
}
} catch(e) {
console.warn("Sincronización externa falló", e);
} finally {
webExternalSyncBusy = false;
}
}
function startWebExternalSync() {
// F49Z80: usa setTimeout encadenado en vez de setInterval para evitar solapamientos y warnings repetidos.
if (window.__webExternalSyncStartedF49Z80) return;
window.__webExternalSyncStartedF49Z80 = true;
const loop = async () => {
  try { await webExternalSyncTick(); } catch(e) {}
  setTimeout(loop, document.hidden ? 30000 : 15000);
};
setTimeout(loop, 15000);
}
// FIX3: selector visible conservando layout original
const ATR_COVER_MODE_KEY_FIX3 = "atrCoverModeV29Fix3";
let atrCoverModeFix3 = localStorage.getItem(ATR_COVER_MODE_KEY_FIX3) || "visible";
function atrCoverModeFix3Apply(mode) {
if (!["off","visible","all"].includes(mode)) mode = "visible";
atrCoverModeFix3 = mode;
localStorage.setItem(ATR_COVER_MODE_KEY_FIX3, mode);
["atrCoverMode","atrCoverModeAtr"].forEach(id => {
const el = $(id);
if (el) el.value = mode;
});
const txt = mode === "off" ? "Carátulas OFF" : (mode === "all" ? "Todas lentamente" : "Solo visibles");
["atrCoverPerfState","atrCoverPerfStateAtr"].forEach(id => {
const el = $(id);
if (el) el.textContent = txt;
});
document.body.classList.toggle("cover-mode-off", mode === "off");
if (mode === "off") {
document.querySelectorAll("img.atr-lazy-cover,img.atr-cover-thumb,img.atr-lib-cover").forEach(img => {
img.removeAttribute("src");
img.dataset.loaded = "";
img.dataset.queued = "";
});
}
if (typeof renderAtrLibrary === "function" && window.lastAtrStatus && Array.isArray(window.lastAtrStatus.files)) {
try { renderAtrLibrary(window.lastAtrStatus.files, { force:true }); } catch(e) {}
}
}
function atrCoverModeFix3LoadVisible() {
if (atrCoverModeFix3 === "off") atrCoverModeFix3Apply("visible");
if (typeof atrQueueImageLoad === "function") {
document.querySelectorAll("img.atr-lazy-cover[data-src]:not([data-loaded='1'])").forEach(img => {
const r = img.getBoundingClientRect();
if (r.bottom >= -100 && r.top <= (window.innerHeight + 180)) atrQueueImageLoad(img, true);
});
}
}
function atrCoverModeFix3Clear() {
try {
if (typeof atrThumbClearMissingF49Z41 === "function") atrThumbClearMissingF49Z41();
localStorage.removeItem("atrCoverMissingV29");
localStorage.removeItem("atrCoverOkV29");
localStorage.removeItem("atrCoverModeV29");
} catch(e) {}
if (typeof setStatus === "function") setStatus("Cache local de carátulas limpiada", true);
if (typeof loadAtrStatus === "function") loadAtrStatus({ files:true, force:true, forceCovers:true, forceLibrary:true });
}
function atrCoverModeFix3Bind() {
atrCoverModeFix3Apply(atrCoverModeFix3);
["atrCoverMode","atrCoverModeAtr"].forEach(id => {
const el = $(id);
if (el && !el.dataset.fix3Bound) {
el.dataset.fix3Bound = "1";
el.addEventListener("change", e => atrCoverModeFix3Apply(e.target.value));
}
});
["btnLoadVisibleCovers","btnLoadVisibleCoversAtr"].forEach(id => {
const el = $(id);
if (el && !el.dataset.fix3Bound) {
el.dataset.fix3Bound = "1";
el.addEventListener("click", atrCoverModeFix3LoadVisible);
}
});
["btnClearCoverCache","btnClearCoverCacheAtr"].forEach(id => {
const el = $(id);
if (el && !el.dataset.fix3Bound) {
el.dataset.fix3Bound = "1";
el.addEventListener("click", atrCoverModeFix3Clear);
}
});
}
// V30: prioridad carga Atari y read-ahead configurable.
let atrLoadPriorityFix30 = localStorage.getItem("atrLoadPriorityFix30") || "1";
let atrReadAheadCountFix30 = localStorage.getItem("atrReadAheadCountFix30Lite") || "0";
let atrLastBusyFix30 = false;
async function atrApplyFastLoadV30() {
try {
const p = new URLSearchParams();
p.set("enabled", "1");
p.set("priority", atrLoadPriorityFix30);
p.set("ra", atrReadAheadCountFix30);
const r = await fetch("/atr/fastload?" + p.toString(), { cache:"no-store" });
if (r.ok) {
const j = await r.json().catch(() => null);
if (j) atrUpdatePriorityUiV30(j);
}
} catch(e) {
console.warn("No se pudo aplicar prioridad ATR", e);
}
}
function atrUpdatePriorityUiV30(st) {
const priority = String(st && st.priorityMode !== undefined ? st.priorityMode : atrLoadPriorityFix30);
const ra = String(st && st.readAheadCountCfg !== undefined ? st.readAheadCountCfg : atrReadAheadCountFix30);
atrLoadPriorityFix30 = priority === "0" ? "0" : "1";
atrReadAheadCountFix30 = ["0","1","2","4"].includes(ra) ? ra : "1";
localStorage.setItem("atrLoadPriorityFix30", atrLoadPriorityFix30);
localStorage.setItem("atrReadAheadCountFix30Lite", atrReadAheadCountFix30);
["atrLoadPriorityMode","atrLoadPriorityModeAtr"].forEach(id => { const el=$(id); if(el) el.value = atrLoadPriorityFix30; });
["atrReadAheadCount","atrReadAheadCountAtr"].forEach(id => { const el=$(id); if(el) el.value = atrReadAheadCountFix30; });
const busy = !!(st && Number(st.atariBusy || 0));
atrLastBusyFix30 = busy;
const txt = busy ? "Atari cargando" : (atrLoadPriorityFix30 === "1" ? "Prioridad ON" : "Prioridad OFF");
["atrLoadPriorityState","atrLoadPriorityStateAtr"].forEach(id => {
const el=$(id);
if(el) {
el.textContent = txt;
el.className = busy ? "pill warn" : "pill ok";
}
});
}
function atrBindPriorityControlsV30() {
["atrLoadPriorityMode","atrLoadPriorityModeAtr"].forEach(id => {
const el=$(id);
if (el && !el.dataset.v30Bound) {
el.dataset.v30Bound = "1";
el.value = atrLoadPriorityFix30;
el.addEventListener("change", () => {
atrLoadPriorityFix30 = el.value === "0" ? "0" : "1";
localStorage.setItem("atrLoadPriorityFix30", atrLoadPriorityFix30);
atrApplyFastLoadV30();
});
}
});
["atrReadAheadCount","atrReadAheadCountAtr"].forEach(id => {
const el=$(id);
if (el && !el.dataset.v30Bound) {
el.dataset.v30Bound = "1";
el.value = atrReadAheadCountFix30;
el.addEventListener("change", () => {
atrReadAheadCountFix30 = ["0","1","2","4"].includes(el.value) ? el.value : "1";
localStorage.setItem("atrReadAheadCountFix30Lite", atrReadAheadCountFix30);
atrApplyFastLoadV30();
});
}
});
atrUpdatePriorityUiV30({ priorityMode: atrLoadPriorityFix30, readAheadCountCfg: atrReadAheadCountFix30, atariBusy: 0 });
}
// Si el Atari está leyendo, no cargar más carátulas por la cola.
if (typeof atrQueueImageLoad === "function" && !window.__atrQueueImageLoadV30Wrapped) {
window.__atrQueueImageLoadV30Wrapped = true;
const __atrQueueImageLoadOrigV30 = atrQueueImageLoad;
atrQueueImageLoad = function(img, force) {
if (atrLastBusyFix30 && !force) return;
return __atrQueueImageLoadOrigV30(img, force);
};
}
// ===== V31 WEB LIGHT MODE JS =====
// Enfocado solo en página web: evita que WEB-ATR/Biblioteca se sientan pesados.
window.WEB_LIGHT_MODE_V31 = true;
document.addEventListener("DOMContentLoaded", () => {
document.body.classList.add("web-light-mode");
});
// F15: en WEB-ATR las carátulas quedan visibles siempre al entrar.
// En F14 podía quedar guardado OFF en localStorage y la unidad mostraba "OFF"
// aunque el selector visual dijera "Solo visibles".
function atrIsWebAtrRouteF15() {
const p = (location.pathname || "").toLowerCase();
return p.indexOf("/atr") === 0 || p.indexOf("/editor/web-atr") === 0 || p.indexOf("/editor/webatr") === 0;
}
function atrForceCoversVisibleF15(reason) {
try {
localStorage.setItem("atrCoverModeV29Fix3", "visible");
localStorage.setItem("atrCoverModeV29", "visible");
if (atrIsWebAtrRouteF15()) localStorage.removeItem("atrCoverModeV31Touched");
} catch(e) {}
try {
if (typeof atrCoverModeFix3Apply === "function") atrCoverModeFix3Apply("visible");
} catch(e) {}
try {
document.body.classList.remove("cover-mode-off");
} catch(e) {}
}
try {
if (atrIsWebAtrRouteF15() || !localStorage.getItem("atrCoverModeV31Touched")) {
atrForceCoversVisibleF15("startup");
}
} catch(e) {}
function webLightV31MarkTouched() {
try { localStorage.setItem("atrCoverModeV31Touched", "1"); } catch(e) {}
}
setTimeout(() => {
["atrCoverMode","atrCoverModeAtr"].forEach(id => {
const el = typeof $ === "function" ? $(id) : document.getElementById(id);
if (el && !el.dataset.v31TouchedBound) {
el.dataset.v31TouchedBound = "1";
el.addEventListener("change", webLightV31MarkTouched);
}
});
}, 500);
if (typeof atrQueueImageLoad === "function" && !window.__atrQueueImageLoadV31Wrapped) {
window.__atrQueueImageLoadV31Wrapped = true;
const __atrQueueImageLoadOrigV31 = atrQueueImageLoad;
atrQueueImageLoad = function(img, force) {
if (typeof atrIsWebAtrRouteF15 === "function" && atrIsWebAtrRouteF15()) {
if (typeof atrForceCoversVisibleF15 === "function") atrForceCoversVisibleF15("queue");
}
const mode1 = localStorage.getItem("atrCoverModeV29Fix3") || "";
const mode2 = localStorage.getItem("atrCoverModeV29") || "";
if (!force && !(typeof atrIsWebAtrRouteF15 === "function" && atrIsWebAtrRouteF15()) && (mode1 === "off" || mode2 === "off")) return;
return __atrQueueImageLoadOrigV31(img, force);
};
}
if (typeof updateAtrCover === "function" && !window.__updateAtrCoverV31Wrapped) {
window.__updateAtrCoverV31Wrapped = true;
const __updateAtrCoverOrigV31 = updateAtrCover;
updateAtrCover = async function(unit, fileName, present) {
if (typeof atrIsWebAtrRouteF15 === "function" && atrIsWebAtrRouteF15()) {
if (typeof atrForceCoversVisibleF15 === "function") atrForceCoversVisibleF15("cover");
}
const mode1 = localStorage.getItem("atrCoverModeV29Fix3") || "";
const mode2 = localStorage.getItem("atrCoverModeV29") || "";
if (!(typeof atrIsWebAtrRouteF15 === "function" && atrIsWebAtrRouteF15()) && (mode1 === "off" || mode2 === "off")) {
const panel = typeof $ === "function" ? $(`atrCoverPanel${unit}`) : null;
const img = typeof $ === "function" ? $(`atrCoverImg${unit}`) : null;
const ph = typeof $ === "function" ? $(`atrCoverPlaceholder${unit}`) : null;
const titleEl = typeof $ === "function" ? $(`atrCoverTitle${unit}`) : null;
const subEl = typeof $ === "function" ? $(`atrCoverSub${unit}`) : null;
if (panel && fileName && fileName !== "—") panel.classList.add("show");
if (img) { img.removeAttribute("src"); img.style.display = "none"; }
if (ph) { ph.style.display = "flex"; ph.textContent = "OFF"; }
if (titleEl) titleEl.textContent = fileName || "—";
if (subEl) subEl.textContent = "Carátulas desactivadas para rendimiento.";
return;
}
return __updateAtrCoverOrigV31(unit, fileName, present);
};
}
let webLightV31LastLibraryLoad = 0;
if (typeof loadAtrStatus === "function" && !window.__loadAtrStatusV31Wrapped) {
window.__loadAtrStatusV31Wrapped = true;
const __loadAtrStatusOrigV31 = loadAtrStatus;
loadAtrStatus = async function(...args) {
const path = (location.pathname || "").toLowerCase();
const now = Date.now();
const opts = args && args[0] ? args[0] : {};
const forced = !!(opts.force || opts.refreshFiles || opts.forceLibrary);
if (!forced && (path.indexOf("/library") === 0 || path.indexOf("/editor/library") === 0) &&
webLightV31LastLibraryLoad &&
(now - webLightV31LastLibraryLoad) < 15000) {
return;
}
if (path.indexOf("/library") === 0 || path.indexOf("/editor/library") === 0) {
webLightV31LastLibraryLoad = now;
}
return __loadAtrStatusOrigV31.apply(this, args);
};
}
if (typeof renderAtrLibrary === "function" && !window.__renderAtrLibraryV31Wrapped) {
window.__renderAtrLibraryV31Wrapped = true;
const __renderAtrLibraryOrigV31 = renderAtrLibrary;
let __lastSigV31 = "";
renderAtrLibrary = function(files, opts) {
try {
const q = (typeof $ === "function" && $("atrLibrarySearch")) ? $("atrLibrarySearch").value : "";
const sig = JSON.stringify((files || []).map(f => [f.name, f.size || f.fileSize || 0, f.mounted || f.drive || ""])) + "|" + q;
if (sig === __lastSigV31 && !(opts && opts.force)) return;
__lastSigV31 = sig;
} catch(e) {}
return __renderAtrLibraryOrigV31(files, opts);
};
}
// ===== F23 TNFS Browser JS + barra de avance descarga/montaje =====
let tnfsCurrentUrl = localStorage.getItem("tnfsCurrentUrlF18") || "tnfs://tnfs.fujinet.online/";
let tnfsCurrentPath = localStorage.getItem("tnfsCurrentPathF18") || "/";
let tnfsCurrentOffset = Number(localStorage.getItem("tnfsCurrentOffsetF25") || "0");
const TNFS_PAGE_SIZE_F25 = 60;
function tnfsSetState(text, ok) {
const st = $("tnfsState");
const line = $("tnfsStatus");
if (st) { st.textContent = text || "—"; st.className = ok === false ? "bad" : ""; }
if (line && text) line.textContent = text;
}
function tnfsNormalizeInputUrl(v) {
v = String(v || "").trim();
if (!v) return "";
if (!/^tnfs:\/\//i.test(v)) v = "tnfs://" + v;
return v;
}
function tnfsJoinPath(base, name) {
base = String(base || "/");
name = String(name || "").trim();
if (!name || name === ".") return base || "/";
if (name === "..") return tnfsParentPath(base);
if (!base.startsWith("/")) base = "/" + base;
if (!base.endsWith("/")) base += "/";
return (base + name).replace(/\/+/g,"/");
}
function tnfsParentPath(p) {
p = String(p || "/").replace(/\/+/g,"/");
if (p.length <= 1) return "/";
if (p.endsWith("/")) p = p.slice(0,-1);
const i = p.lastIndexOf("/");
return i <= 0 ? "/" : p.slice(0,i);
}
function tnfsIsAtrName(name) {
return String(name || "").toLowerCase().endsWith(".atr");
}
function tnfsIsXexLikeName(name) {
const n = String(name || "").toLowerCase();
return n.endsWith(".xex") || n.endsWith(".com") || n.endsWith(".exe") || n.endsWith(".bas");
}
function tnfsIsCasName(name) {
return String(name || "").toLowerCase().endsWith(".cas");
}
function tnfsIsMountableName(name) {
return tnfsIsAtrName(name) || tnfsIsXexLikeName(name) || tnfsIsCasName(name);
}
function tnfsVisibleDriveUnits() {
const max = (typeof UI_MAX_UNITS === "number" && UI_MAX_UNITS > 0) ? UI_MAX_UNITS : 7;
const mask = (typeof visibleUnitMaskFromStatus === "function") ? visibleUnitMaskFromStatus(lastDriveVisibility || {}) : Number(lastDriveVisibility?.visibleMask || 15);
const out = [];
for (let i = 1; i <= max; i++) {
const visible = (typeof unitVisible === "function") ? unitVisible(mask, i) : (i <= 4 || !!(Number(mask || 15) & (1 << (i - 1))));
if (visible) out.push(i);
}
return out.length ? out : [1,2,3,4];
}
function tnfsVisibleDriveLabel() {
const units = tnfsVisibleDriveUnits();
let contiguous = true;
for (let i = 0; i < units.length; i++) { if (units[i] !== i + 1) { contiguous = false; break; } }
if (contiguous) return `D1-D${units[units.length - 1]}`;
return units.map(u => `D${u}`).join(", ");
}
function tnfsMountButtonsHtml(name) {
if (!tnfsIsMountableName(name)) {
return `<button type="button" class="secondary" data-tnfs-open="${escapeAttr(name)}">Abrir como carpeta</button>`;
}
if (tnfsIsCasName(name)) {
return `<button type="button" class="secondary tnfs-mount-btn tnfs-cas-btn" data-tnfs-mount="${escapeAttr(name)}" data-unit="1" title="Descargar a /CAS y preparar C:">Preparar C:</button>`;
}
if (tnfsIsXexLikeName(name)) {
return `<button type="button" class="secondary tnfs-mount-btn" data-tnfs-mount="${escapeAttr(name)}" data-unit="0" title="Descargar crudo sin convertir a ATR">Descargar</button>`;
}
let out = "";
tnfsVisibleDriveUnits().forEach(i => {
out += `<button type="button" class="secondary tnfs-mount-btn" data-tnfs-mount="${escapeAttr(name)}" data-unit="${i}" title="Descargar a /ATR y montar en D${i}">D${i}</button>`;
});
return out;
}
function tnfsBindContentButtons() {
const content = $("tnfsContent");
if (!content) return;
content.querySelectorAll("button[data-tnfs-open]").forEach(btn => {
if (btn.dataset.tnfsBound === "1") return;
btn.dataset.tnfsBound = "1";
btn.addEventListener("click", () => tnfsBrowse(tnfsJoinPath(tnfsCurrentPath, btn.dataset.tnfsOpen || "")));
});
content.querySelectorAll("button[data-tnfs-mount]").forEach(btn => {
if (btn.dataset.tnfsBound === "1") return;
btn.dataset.tnfsBound = "1";
btn.addEventListener("click", () => tnfsDownloadMount(btn.dataset.tnfsMount || "", btn.dataset.unit || "1", btn));
});
}
function tnfsRefreshMountButtonsF22() {
const content = $("tnfsContent");
if (!content) return;
content.querySelectorAll(".tnfs-row-actions[data-tnfs-file]").forEach(box => {
const name = box.dataset.tnfsFile || "";
box.innerHTML = tnfsMountButtonsHtml(name);
});
tnfsBindContentButtons();
const label = tnfsVisibleDriveLabel();
if ($("tnfsStatus")) {
const txt = $("tnfsStatus").textContent || "";
if (txt.indexOf("botones") >= 0 || txt.indexOf("D1-D") >= 0) $("tnfsStatus").textContent = `TNFS listo · unidades visibles ${label}`;
}
}
function tnfsFormatBytes(bytes) {
bytes = Number(bytes || 0);
if (bytes >= 1048576) return (bytes / 1048576).toFixed(2) + " MB";
if (bytes >= 1024) return (bytes / 1024).toFixed(1) + " KB";
return bytes + " bytes";
}
function tnfsProgressPctFromBytes(bytes, phase) {
bytes = Number(bytes || 0);
if (phase === "start") return 6;
if (phase === "skip") return 72;
if (phase === "convert") return 86;
if (phase === "save") return 90;
if (phase === "mount") return 96;
if (phase === "done") return 100;
if (bytes <= 0) return 10;
const kb = Math.max(1, bytes / 1024);
return Math.max(12, Math.min(84, Math.round(12 + Math.log2(kb + 1) * 8)));
}
function tnfsEnsureProgressWrap() {
let wrap = $("tnfsDownloadProgressWrap");
if (!wrap) {
wrap = document.createElement("div");
wrap.id = "tnfsDownloadProgressWrap";
wrap.className = "upload-progress-wrap tnfs-download-progress";
wrap.setAttribute("aria-live", "polite");
wrap.innerHTML = `<div class="upload-progress-track"><div id="tnfsDownloadProgressBar" class="upload-progress-bar tnfs-progress-bar"></div></div><div id="tnfsDownloadProgressText" class="upload-progress-text">Preparando descarga TNFS...</div>`;
const content = $("tnfsContent");
if (content && content.parentNode) content.parentNode.insertBefore(wrap, content);
else if ($("tnfsPageBlock")) $("tnfsPageBlock").appendChild(wrap);
else document.body.appendChild(wrap);
}
return wrap;
}
function tnfsPlaceProgressUnderButton(sourceButton) {
const wrap = tnfsEnsureProgressWrap();
const content = $("tnfsContent");
if (!wrap || !content) return wrap;
content.querySelectorAll(".tnfs-row-active-download").forEach(row => row.classList.remove("tnfs-row-active-download"));
content.querySelectorAll(".tnfs-row-progress").forEach(slot => {
if (slot.contains(wrap)) {
const fallbackHost = $("tnfsPageBlock") || document.body;
fallbackHost.appendChild(wrap);
}
slot.remove();
});
const row = sourceButton && sourceButton.closest ? sourceButton.closest(".tnfs-row") : null;
if (row) {
let slot = row.querySelector(".tnfs-row-progress");
if (!slot) {
slot = document.createElement("div");
slot.className = "tnfs-row-progress";
row.appendChild(slot);
}
row.classList.add("tnfs-row-active-download");
slot.appendChild(wrap);
}
return wrap;
}
function tnfsShowProgress(text, pct, indeterminate) {
const wrap = tnfsEnsureProgressWrap(), bar = $("tnfsDownloadProgressBar"), label = $("tnfsDownloadProgressText");
if (wrap) wrap.classList.add("active");
if (bar) {
bar.style.width = Math.max(0, Math.min(100, Number(pct || 0))) + "%";
bar.classList.toggle("indeterminate", !!indeterminate);
}
if (label) label.textContent = text || "Procesando TNFS...";
}
function tnfsHideProgressSoon(delay) {
setTimeout(() => {
const wrap = $("tnfsDownloadProgressWrap"), bar = $("tnfsDownloadProgressBar");
if (wrap) {
wrap.classList.remove("active");
const row = wrap.closest ? wrap.closest(".tnfs-row") : null;
if (row) row.classList.remove("tnfs-row-active-download");
}
if (bar) { bar.style.width = "0%"; bar.classList.remove("indeterminate"); }
}, Number(delay || 450));
}
function tnfsHandleProgressEvent(evt, name, unit) {
if (!evt) return null;
const phase = evt.phase || evt.type || "download";
const bytes = Number(evt.bytes || 0);
let txt = evt.message || "";
if (!txt) {
const isCas = tnfsIsCasName(name);
const isRaw = !isCas && tnfsIsXexLikeName(name);
if (phase === "start") txt = isCas ? `Preparando ${name} para C:...` : (isRaw ? `Preparando descarga cruda de ${name}...` : `Preparando ${name} para D${unit}...`);
else if (phase === "skip") txt = isCas ? `El archivo ya existe en /CAS. Preparando C:...` : (isRaw ? `El archivo ya existe. No se convierte a ATR.` : `El archivo ya existe en /ATR. Montando en D${unit}...`);
else if (phase === "convert") txt = `Convirtiendo a ATR montable... ${tnfsFormatBytes(bytes)}`;
else if (phase === "save") txt = isRaw ? `Guardando archivo original... ${tnfsFormatBytes(bytes)}` : `Guardando... ${tnfsFormatBytes(bytes)}`;
else if (phase === "mount") txt = isCas ? `Preparando C:...` : `Montando en D${unit}...`;
else if (phase === "done") txt = isCas ? `Cassette listo en C:.` : (isRaw ? `Archivo descargado crudo en Biblioteca.` : `Montaje listo en D${unit}.`);
else txt = `Descargando ${name}: ${tnfsFormatBytes(bytes)}`;
}
tnfsShowProgress(txt, tnfsProgressPctFromBytes(bytes, phase), phase === "download");
if (phase === "done") return evt;
return null;
}
async function tnfsFetchMountStreaming(qs, name, unit) {
qs.set("stream", "1");
const r = await fetch("/api/tnfs/fetch_mount?" + qs.toString(), { cache:"no-store" });
const ct = (r.headers.get("content-type") || "").toLowerCase();
if (!r.body || !window.TextDecoder || ct.indexOf("json") >= 0 && ct.indexOf("x-ndjson") < 0) {
const j = await r.json();
return j;
}
const reader = r.body.getReader();
const decoder = new TextDecoder();
let buf = "";
let finalEvent = null;
while (true) {
const { value, done } = await reader.read();
if (value) {
buf += decoder.decode(value, { stream: !done });
let lines = buf.split("\n");
buf = lines.pop() || "";
for (const line of lines) {
const t = line.trim();
if (!t) continue;
try {
const evt = JSON.parse(t);
const maybeFinal = tnfsHandleProgressEvent(evt, name, unit);
if (maybeFinal) finalEvent = maybeFinal;
} catch(e) {}
}
}
if (done) break;
}
const tail = buf.trim();
if (tail) {
try {
const evt = JSON.parse(tail);
const maybeFinal = tnfsHandleProgressEvent(evt, name, unit);
if (maybeFinal) finalEvent = maybeFinal;
} catch(e) {}
}
return finalEvent || { ok:0, error:"TNFS_STREAM_WITHOUT_FINAL" };
}
async function tnfsDownloadMount(name, unit, sourceButton) {
name = String(name || "").trim();
const isCas = tnfsIsCasName(name);
const isRaw = !isCas && tnfsIsXexLikeName(name);
unit = Number(unit || 1);
if (!name || !tnfsIsMountableName(name)) { tnfsSetState("Solo se admite CAS, ATR, XEX, COM, EXE o BAS desde TNFS", false); return; }
const remotePath = tnfsJoinPath(tnfsCurrentPath || "/", name);
const content = $("tnfsContent");
const targetLabel = isCas ? "C:" : (isRaw ? "Biblioteca" : `D${unit}`);
tnfsSetState(`TNFS: preparando ${name} para ${targetLabel}...`, true);
tnfsPlaceProgressUnderButton(sourceButton);
tnfsShowProgress(`Preparando ${name} para ${targetLabel}...`, 6, true);
if (content) {
content.querySelectorAll("button[data-tnfs-mount]").forEach(b => b.disabled = true);
}
try {
const qs = new URLSearchParams();
qs.set("url", tnfsCurrentUrl || "");
qs.set("path", remotePath);
qs.set("unit", String(unit));
const j = await tnfsFetchMountStreaming(qs, name, unit);
if (!j.ok) throw new Error(j.error || j.status || "TNFS_DOWNLOAD_MOUNT_FAILED");
if (!isCas && !isRaw && typeof atrApplyLightStatusFast === "function") atrApplyLightStatusFast(j, { forceCovers:true });
if (!isRaw && typeof atrSchedulePostMountRefresh === "function") atrSchedulePostMountRefresh({ bt:false, library:true, delay:1400 });
let msg = "";
if (isCas) {
msg = j.skipped ? `Ya existía en /CAS, preparado en C: ${j.localName || j.casName || name}` : `Descargado a /CAS y preparado en C: ${j.localName || j.casName || name}`;
} else if (isRaw) {
msg = j.skipped ? `Ya existía archivo original: ${j.localName || name}` : `Descargado archivo original sin convertir: ${j.localName || name}`;
} else {
msg = j.skipped ? `Ya existía en /ATR, montado en D${unit}: ${j.localName || name}` : `Descargado a /ATR y montado en D${unit}: ${j.localName || name} (ATR)`;
}
tnfsShowProgress(`Listo: ${msg}`, 100, false);
tnfsSetState(`${msg} · ${j.elapsedMs || 0} ms`, true);
if (typeof setStatus === "function") setStatus(msg, true);
tnfsHideProgressSoon(700);
} catch(e) {
tnfsShowProgress("Error TNFS: " + e.message, 100, false);
tnfsSetState("Error descargando/montando TNFS: " + e.message, false);
if (typeof setStatus === "function") setStatus("Error TNFS: " + e.message, false);
} finally {
if (content) content.querySelectorAll("button[data-tnfs-mount]").forEach(b => b.disabled = false);
}
}
function tnfsUpdateHeader(j) {
if ($("tnfsCurrentServer")) $("tnfsCurrentServer").textContent = tnfsCurrentUrl.replace(/^tnfs:\/\//i, "");
if ($("tnfsCurrentPath")) $("tnfsCurrentPath").textContent = tnfsCurrentPath || "/";
if ($("tnfsPathBox")) $("tnfsPathBox").textContent = tnfsCurrentPath || "/";
const count = j && typeof j.count !== "undefined" ? Number(j.count || 0) : 0;
const offset = j && typeof j.offset !== "undefined" ? Number(j.offset || 0) : Number(tnfsCurrentOffset || 0);
const limit = j && typeof j.limit !== "undefined" ? Number(j.limit || TNFS_PAGE_SIZE_F25) : TNFS_PAGE_SIZE_F25;
const page = Math.floor(offset / Math.max(1, limit)) + 1;
const hasMore = !!(j && Number(j.hasMore || 0));
const from = count ? offset + 1 : offset;
const to = offset + count;
if ($("tnfsEntryCount")) $("tnfsEntryCount").textContent = j && typeof j.count !== "undefined" ? `${count} entradas` : "—";
if ($("tnfsSummary")) $("tnfsSummary").textContent = j && j.ok ? `Pág. ${page} · ${count} entradas · ${j.elapsedMs || 0} ms` : "Sin conexión";
if ($("tnfsPageInfo")) $("tnfsPageInfo").textContent = j && j.ok ? `Página ${page} · ${from}-${to}${hasMore ? "+" : ""}` : `Página ${page}`;
if ($("btnTnfsPrevPage")) $("btnTnfsPrevPage").disabled = !(j && j.ok && offset > 0);
if ($("btnTnfsNextPage")) $("btnTnfsNextPage").disabled = !(j && j.ok && hasMore);
}
function tnfsSetOffsetF25(offset) {
tnfsCurrentOffset = Math.max(0, Number(offset || 0));
localStorage.setItem("tnfsCurrentOffsetF25", String(tnfsCurrentOffset));
}
function tnfsResetOffsetF25() { tnfsSetOffsetF25(0); }
function tnfsNextPageF25() { tnfsSetOffsetF25(tnfsCurrentOffset + TNFS_PAGE_SIZE_F25); return tnfsBrowse(null, { keepOffset:true }); }
function tnfsPrevPageF25() { tnfsSetOffsetF25(Math.max(0, tnfsCurrentOffset - TNFS_PAGE_SIZE_F25)); return tnfsBrowse(null, { keepOffset:true }); }
async function tnfsLoadServers() {
const list = $("tnfsServerList");
if (list) list.innerHTML = "Cargando servidores...";
try {
const r = await fetch("/api/tnfs/servers", { cache:"no-store" });
const j = await r.json();
const servers = j.servers || [];
if (!servers.length) {
if (list) list.innerHTML = "No hay servidores registrados.";
return;
}
if (list) list.innerHTML = servers.map(s => {
const label = String(s.url || "").replace(/^tnfs:\/\//i, "");
const src = s.source === "custom" ? "personal" : "default";
return `<div class="tnfs-server-item">` +
`<button type="button" class="secondary tnfs-server-btn" data-url="${escapeAttr(s.url || "")}"><span>🌐 ${escapeHtml(label)}</span><br><small>${src}</small></button>` +
`<button type="button" class="secondary tnfs-delete-btn" data-tnfs-delete="${escapeAttr(s.url || "")}" title="Eliminar URL TNFS">✕</button>` +
`</div>`;
}).join("");
list.querySelectorAll("button.tnfs-server-btn[data-url]").forEach(btn => {
btn.addEventListener("click", () => {
tnfsCurrentUrl = btn.dataset.url || tnfsCurrentUrl;
tnfsCurrentPath = "/";
tnfsResetOffsetF25();
localStorage.setItem("tnfsCurrentUrlF18", tnfsCurrentUrl);
localStorage.setItem("tnfsCurrentPathF18", tnfsCurrentPath);
if ($("tnfsUrlInput")) $("tnfsUrlInput").value = tnfsCurrentUrl;
tnfsBrowse();
});
});
list.querySelectorAll("button[data-tnfs-delete]").forEach(btn => {
btn.addEventListener("click", async (ev) => {
ev.preventDefault();
ev.stopPropagation();
await tnfsDeleteServer(btn.dataset.tnfsDelete || "");
});
});
} catch(e) {
if (list) list.innerHTML = "Error cargando servidores.";
tnfsSetState("Error cargando servidores TNFS", false);
}
}
async function tnfsDeleteServer(url) {
url = tnfsNormalizeInputUrl(url || "");
if (!url) return;
const label = url.replace(/^tnfs:\/\//i, "");
if (!confirm(`¿Eliminar esta URL TNFS?\n${label}`)) return;
tnfsSetState("Eliminando URL TNFS...", true);
try {
const r = await fetch("/api/tnfs/delete?url=" + encodeURIComponent(url), { cache:"no-store" });
const j = await r.json();
if (!j.ok) throw new Error(j.error || "DELETE_FAILED");
if ((tnfsCurrentUrl || "").toLowerCase() === url.toLowerCase()) {
tnfsCurrentUrl = "";
tnfsCurrentPath = "/";
tnfsResetOffsetF25();
localStorage.removeItem("tnfsCurrentUrlF18");
localStorage.setItem("tnfsCurrentPathF18", "/");
if ($("tnfsUrlInput")) $("tnfsUrlInput").value = "";
if ($("tnfsContent")) $("tnfsContent").innerHTML = "";
tnfsUpdateHeader({count:0});
}
await tnfsLoadServers();
tnfsSetState("URL TNFS eliminada. Selecciona otro servidor o agrégala nuevamente si la necesitas.", true);
} catch(e) {
tnfsSetState("No se pudo eliminar URL TNFS: " + e.message, false);
}
}
async function tnfsAddServer() {
const input = $("tnfsUrlInput");
const url = tnfsNormalizeInputUrl(input ? input.value : "");
if (!url) { tnfsSetState("Ingresa una URL TNFS", false); return; }
tnfsSetState("Agregando servidor...", true);
try {
const r = await fetch("/api/tnfs/add?url=" + encodeURIComponent(url), { cache:"no-store" });
const j = await r.json();
if (!j.ok) throw new Error(j.error || "ADD_FAILED");
tnfsCurrentUrl = j.url || url;
tnfsCurrentPath = "/";
tnfsResetOffsetF25();
localStorage.setItem("tnfsCurrentUrlF18", tnfsCurrentUrl);
localStorage.setItem("tnfsCurrentPathF18", tnfsCurrentPath);
if (input) input.value = tnfsCurrentUrl;
await tnfsLoadServers();
await tnfsBrowse();
} catch(e) {
tnfsSetState("No se pudo agregar servidor: " + e.message, false);
}
}
async function tnfsBrowse(path, opts) {
opts = opts || {};
if (path !== null && typeof path !== "undefined") { tnfsCurrentPath = path; if (!opts.keepOffset) tnfsResetOffsetF25(); }
if (!tnfsCurrentUrl) tnfsCurrentUrl = tnfsNormalizeInputUrl($("tnfsUrlInput") ? $("tnfsUrlInput").value : "");
if (!tnfsCurrentUrl) { tnfsSetState("Selecciona un servidor TNFS", false); return; }
localStorage.setItem("tnfsCurrentUrlF18", tnfsCurrentUrl);
localStorage.setItem("tnfsCurrentPathF18", tnfsCurrentPath || "/");
localStorage.setItem("tnfsCurrentOffsetF25", String(tnfsCurrentOffset || 0));
tnfsUpdateHeader();
const content = $("tnfsContent");
if (content) content.innerHTML = "";
tnfsSetState("Leyendo directorio TNFS...", true);
try {
const qs = "url=" + encodeURIComponent(tnfsCurrentUrl) + "&path=" + encodeURIComponent(tnfsCurrentPath || "/") + "&limit=" + encodeURIComponent(String(TNFS_PAGE_SIZE_F25)) + "&offset=" + encodeURIComponent(String(tnfsCurrentOffset || 0));
const r = await fetch("/api/tnfs/list?" + qs, { cache:"no-store" });
const j = await r.json();
if (!j.ok) {
const msg = j.error === "WIFI_STA_NOT_CONNECTED" ? "WiFi STA no conectado. Configura WiFi cliente para salir a Internet." : (j.error || "TNFS_LIST_FAILED");
tnfsSetState(msg, false);
if (content) content.innerHTML = `<div class="tnfs-row"><div><div class="tnfs-row-title">${escapeHtml(msg)}</div><div class="tnfs-row-sub">${escapeHtml(j.hint || j.status || "")}</div></div></div>`;
tnfsUpdateHeader(j);
return;
}
tnfsCurrentPath = j.path || tnfsCurrentPath || "/";
tnfsSetOffsetF25(Number(j.offset || 0));
tnfsUpdateHeader(j);
const items = j.items || [];
if (!items.length) {
if (content) content.innerHTML = `<div class="tnfs-row"><div><div class="tnfs-row-title">${tnfsCurrentOffset > 0 ? "Sin más archivos en esta página" : "Directorio vacío"}</div><div class="tnfs-row-sub">${escapeHtml(tnfsCurrentPath)}</div></div></div>`;
} else if (content) {
content.innerHTML = items.map(it => {
const name = String(it.name || "");
const isBack = name === "..";
const isAtr = tnfsIsAtrName(name);
const isXex = tnfsIsXexLikeName(name);
const isCas = tnfsIsCasName(name);
const isMountable = isAtr || isXex || isCas;
const ico = isBack ? "⬆" : (isCas ? "📼" : (isAtr ? "💾" : (isXex ? "⚡" : "📄")));
const unitLabel = tnfsVisibleDriveLabel();
const sub = isCas ? `CAS remoto · descarga a /CAS y prepara C:` : (isAtr ? `ATR remoto · ${unitLabel} descarga a /ATR y monta` : (isXex ? `XEX/COM/EXE/BAS remoto · descarga cruda, sin convertir a ATR` : "TNFS · usa Abrir como carpeta para navegar"));
return `<div class="tnfs-row"><div><div class="tnfs-row-title">${ico} ${escapeHtml(name)}</div><div class="tnfs-row-sub">${escapeHtml(sub)}</div></div><div class="tnfs-row-actions" data-tnfs-file="${escapeAttr(name)}">${tnfsMountButtonsHtml(name)}</div></div>`;
}).join("");
tnfsBindContentButtons();
}
tnfsSetState(`Listado OK · página ${Math.floor(Number(j.offset || 0) / Math.max(1, Number(j.limit || TNFS_PAGE_SIZE_F25))) + 1} · ${j.count || 0} entradas · ${j.elapsedMs || 0} ms · unidades ${tnfsVisibleDriveLabel()}`, true);
} catch(e) {
tnfsSetState("Error TNFS: " + e.message, false);
}
}
function bindTnfsBrowserF18() {
if ($("tnfsUrlInput")) $("tnfsUrlInput").value = tnfsCurrentUrl;
if ($("btnTnfsAdd")) $("btnTnfsAdd").addEventListener("click", tnfsAddServer);
if ($("btnTnfsRefreshServers")) $("btnTnfsRefreshServers").addEventListener("click", tnfsLoadServers);
if ($("btnTnfsReload")) $("btnTnfsReload").addEventListener("click", () => tnfsBrowse(null, { keepOffset:true }));
if ($("btnTnfsPrevPage")) $("btnTnfsPrevPage").addEventListener("click", tnfsPrevPageF25);
if ($("btnTnfsNextPage")) $("btnTnfsNextPage").addEventListener("click", tnfsNextPageF25);
if ($("btnTnfsRoot")) $("btnTnfsRoot").addEventListener("click", () => tnfsBrowse("/"));
if ($("btnTnfsUp")) $("btnTnfsUp").addEventListener("click", () => tnfsBrowse(tnfsParentPath(tnfsCurrentPath)));
if ($("tnfsUrlInput")) $("tnfsUrlInput").addEventListener("keydown", ev => { if (ev.key === "Enter") tnfsAddServer(); });
tnfsUpdateHeader();
}
document.addEventListener("DOMContentLoaded", () => {
bindFileLabels();
bindTnfsBrowserF18();
if (typeof atrForceCoversVisibleF15 === "function" && typeof atrIsWebAtrRouteF15 === "function" && atrIsWebAtrRouteF15()) atrForceCoversVisibleF15("dom");
atrCoverModeFix3Bind();
atrBindPriorityControlsV30();
atrBindCoverFullscreenV34();
$("btnReload").addEventListener("click", loadStatus);
$("btnSave").addEventListener("click", saveConfig);
$("btnPrinterSave").addEventListener("click", savePrinterConfig);
$("btnPrinterTest").addEventListener("click", testPrinter);
$("btnPrinterIppTest").addEventListener("click", testIppPrinter);
$("btnPrinterVirtualTest").addEventListener("click", testVirtualPrinter);
$("btnPrinterAtasciiTableTest").addEventListener("click", testAtasciiTablePrinter);
$("btnPrinterBufferPrint").addEventListener("click", printAtariBuffer);
$("btnPrinterClear").addEventListener("click", clearPrinterQueue);
if ($("btnGlyphImport")) $("btnGlyphImport").addEventListener("click", importGlyphs);
if ($("btnGlyphExport")) $("btnGlyphExport").addEventListener("click", exportGlyphs);
if ($("btnGlyphLoad")) $("btnGlyphLoad").addEventListener("click", glyphLoadCurrent);
if ($("btnGlyphSave")) $("btnGlyphSave").addEventListener("click", glyphSaveAllCurrentV34);
if ($("btnGlyphRestore")) $("btnGlyphRestore").addEventListener("click", glyphRestoreCurrent);
if ($("glyphEditFont")) $("glyphEditFont").addEventListener("change", glyphLoadCurrent);
if ($("glyphEditCode")) $("glyphEditCode").addEventListener("change", glyphLoadCurrent);
if ($("btnAtrRefresh")) $("btnAtrRefresh").addEventListener("click", () => loadAtrStatus({ files:false, refreshFiles:false, force:true, forceCovers:true }));
if ($("btnLibraryRefresh")) $("btnLibraryRefresh").addEventListener("click", () => { if (typeof window.loadLibraryPageF49Z45 === "function") window.loadLibraryPageF49Z45({ refresh:true, force:true }); else loadAtrStatus({ files:true, refreshFiles:true, force:true, forceLibrary:true }); });
if ($("btnBtSioOff")) $("btnBtSioOff").addEventListener("click", () => setBtSio2pc(false));
if ($("btnBtSioRefresh")) $("btnBtSioRefresh").addEventListener("click", loadBtSioStatus);
if ($("btnBtAddDrive")) $("btnBtAddDrive").addEventListener("click", () => setOptionalDriveVisible(true));
if ($("btnBtRemoveDrive")) $("btnBtRemoveDrive").addEventListener("click", () => setOptionalDriveVisible(false));
if ($("btnAtrAddDrive")) $("btnAtrAddDrive").addEventListener("click", () => setOptionalDriveVisible(true));
if ($("btnAtrRemoveDrive")) $("btnAtrRemoveDrive").addEventListener("click", () => setOptionalDriveVisible(false));
if ($("btnDiskAddDrive")) $("btnDiskAddDrive").addEventListener("click", () => setOptionalDriveVisible(true));
if ($("btnDiskRemoveDrive")) $("btnDiskRemoveDrive").addEventListener("click", () => setOptionalDriveVisible(false));
bindBtSioAutoSave();
bindAtrAutoSave();
if ($("btnAtrOff")) $("btnAtrOff").addEventListener("click", () => setWebAtr(0, 0, 0));
bindAtrUploadForm();
if ($("atrLibrarySearch")) $("atrLibrarySearch").addEventListener("input", () => { if (typeof atrLibraryResetPageF49Z20 === "function") atrLibraryResetPageF49Z20(); if (lastAtrStatus) renderAtrLibrary((lastAtrStatus.files || []).slice().sort((x, y) => String(x.name || "").localeCompare(String(y.name || ""), "es", { sensitivity: "base", numeric: true }))); });
if ($("atrLibraryView")) $("atrLibraryView").addEventListener("change", () => { atrLibraryViewMode = $("atrLibraryView").value || "flat"; localStorage.setItem("atrLibraryViewMode", atrLibraryViewMode); if (typeof atrLibraryResetPageF49Z20 === "function") atrLibraryResetPageF49Z20(); if (typeof window.loadLibraryPageF49Z45 === "function") window.loadLibraryPageF49Z45({ force:true }); else if (lastAtrStatus) loadAtrStatus({ files:true, refreshFiles:false, forceLibrary:true }); });
if ((location.pathname || "").toLowerCase().indexOf("/editor/glyphs") === 0 || (location.pathname || "").toLowerCase().indexOf("/glyphs") === 0) { buildGlyphV27Layout(); glyphEnsureGridEditorReadyV38(); }
updateVisibleUnitCards(15);
loadDriveVisibilityStatus();
const currentPath = (location.pathname || "/").toLowerCase();
const isAtrRoute = currentPath.indexOf("/atr") === 0 || currentPath.indexOf("/editor/web-atr") === 0 || currentPath.indexOf("/editor/webatr") === 0;
const isBtRoute = currentPath.indexOf("/bt") === 0 || currentPath.indexOf("/editor/bt") === 0;
const isLibraryRoute = currentPath.indexOf("/library") === 0 || currentPath.indexOf("/editor/library") === 0 || currentPath.indexOf("/upload") === 0;
const isTnfsRoute = currentPath.indexOf("/tnfs") === 0 || currentPath.indexOf("/editor/tnfs") === 0;
if (isAtrRoute || isBtRoute || isLibraryRoute || isTnfsRoute) setTimeout(loadStatus, 1200);
else loadStatus();
if (isAtrRoute) {
// F15: WEB-ATR fuerza carátulas visibles aunque el navegador haya guardado OFF.
if (typeof atrForceCoversVisibleF15 === "function") atrForceCoversVisibleF15("route");
// F14: primero muestra estado y unidades sin lista pesada; luego carga selectores.
loadAtrStatus({ files:false, refreshFiles:false, force:true, forceCovers:true });
setTimeout(() => loadAtrStatus({ files:true, refreshFiles:false, forceSelectOptions:true, forceCovers:true }), 250);
}
else if (isLibraryRoute) loadAtrStatus({ files:true, refreshFiles:false, force:true });
else if (isTnfsRoute) { tnfsLoadServers(); setTimeout(() => tnfsBrowse(), 250); }
if (currentPath.indexOf("/bt") === 0 || currentPath.indexOf("/editor/bt") === 0) {
// Carga automática del estado real de D1-D4 al entrar a Bluetooth.
// Sin refresco periódico: los cambios se guardan al instante y el estado se lee manualmente si hace falta.
loadBtSioStatus();
}
startWebExternalSync();
if ((location.pathname || "").toLowerCase().indexOf("/editor/glyphs") === 0 || (location.pathname || "").toLowerCase().indexOf("/glyphs") === 0) setTimeout(() => { glyphLoadCurrent().finally(() => glyphEnsureGridEditorReadyV38()); }, 400);
});
async function atrServerCoverResolve(fileName) {
try {
const r = await fetch("/api/covers/resolve?name=" + encodeURIComponent(fileName || ""), { cache: "no-store" });
if (!r.ok) return null;
const j = await r.json();
if (j && j.ok && j.image) return j;
} catch(e) {}
return null;
}
async function atrServerCoverSet(name, url, title) {
const params = new URLSearchParams();
params.set("name", name || "");
params.set("url", atrNormalizeImageUrl(url || ""));
params.set("title", title || name || "");
const r = await fetch("/api/covers/set", {
method: "POST",
headers: { "Content-Type": "application/x-www-form-urlencoded" },
body: params.toString()
});
const txt = await r.text();
let j = null;
try { j = txt ? JSON.parse(txt) : null; } catch(e) {}
if (!r.ok) throw new Error((j && j.reason) ? j.reason : (txt || ("HTTP " + r.status)));
return j || {};
}
async function atrServerCoverDelete(name) {
const params = new URLSearchParams();
params.set("name", name || "");
const r = await fetch("/api/covers/delete", {
method: "POST",
headers: { "Content-Type": "application/x-www-form-urlencoded" },
body: params.toString()
});
if (!r.ok) throw new Error("HTTP " + r.status);
return await r.json();
}
// ===== WEB-ATR manual cover URL support =====
function atrManualCoverKeyFromName(name) {
if (typeof normalizeAtrCoverKey === "function") return normalizeAtrCoverKey(name || "");
return String(name || "").toLowerCase().trim();
}
function atrManualCoverMapLoad() {
try {
return JSON.parse(localStorage.getItem("webAtrManualCovers") || "{}") || {};
} catch(e) {
return {};
}
}
function atrManualCoverMapSave(map) {
try {
localStorage.setItem("webAtrManualCovers", JSON.stringify(map || {}));
} catch(e) {}
}
function atrManualCoverGet(name) {
const key = atrManualCoverKeyFromName(name);
if (!key) return null;
const map = atrManualCoverMapLoad();
return map[key] || null;
}
function atrManualCoverSet(name, url, title) {
const key = atrManualCoverKeyFromName(name);
if (!key || !url) return false;
const map = atrManualCoverMapLoad();
map[key] = {
title: title || name || key,
image: atrNormalizeImageUrl(url),
source: "Manual local",
savedAt: new Date().toISOString()
};
atrManualCoverMapSave(map);
return true;
}
function atrManualCoverDelete(name) {
const key = atrManualCoverKeyFromName(name);
if (!key) return false;
const map = atrManualCoverMapLoad();
if (map[key]) {
delete map[key];
atrManualCoverMapSave(map);
return true;
}
return false;
}
function atrNormalizeImageUrl(url) {
return String(url || "").trim().replace(/\s/g, "%20");
}
function atrCoverProxySrc(url) {
const clean = atrNormalizeImageUrl(url);
if (!clean) return "";
// F10: incluso si la imagen ya quedó en /COVERS de la SD, se pide al MASTER.
// El backend sirve local primero y solo usa internet si no existe caché.
return "/api/covers/image?url=" + encodeURIComponent(clean);
}
function atrCoverProxyNameSrc(fileName) {
return "/api/covers/image?name=" + encodeURIComponent(fileName || "");
}
function atrCoverThumbNameSrc(fileName) {
fileName = String(fileName || "").trim();
if (!fileName) return "";
return "/api/covers/thumb?name=" + encodeURIComponent(fileName);
}
function atrCoverThumbUrlSrc(url, name) {
const q = new URLSearchParams();
if (name) q.set("name", name);
if (url) q.set("url", atrNormalizeImageUrl(url));
return "/api/covers/thumb?" + q.toString();
}
const atrMiniCoverPending = new Set();
let atrMiniCoverBusy = false;
const atrMiniCoverQueue = [];
function atrEnsureMiniCoverFromImage(img, fileName) {
const key = atrCoverCacheKey(fileName || "");
if (!key || !img || !img.naturalWidth || !img.naturalHeight) return;
// Si ya es una miniatura pequeña, no la volvemos a procesar ni subir.
if (img.naturalWidth <= 190 && img.naturalHeight <= 250) return;
const doneKey = "miniCoverOk:" + key + ":" + img.naturalWidth + "x" + img.naturalHeight;
if (localStorage.getItem(doneKey) === "1" || atrMiniCoverPending.has(doneKey)) return;
atrMiniCoverPending.add(doneKey);
atrMiniCoverQueue.push({ img, fileName, key, doneKey });
atrMiniCoverPump();
}
function atrMiniCoverPump() {
if (atrMiniCoverBusy || !atrMiniCoverQueue.length) return;
const item = atrMiniCoverQueue.shift();
atrMiniCoverBusy = true;
const run = () => atrGenerateMiniCoverNow(item)
.catch(e => console.warn("No se pudo generar mini cover", e))
.finally(() => {
atrMiniCoverBusy = false;
setTimeout(atrMiniCoverPump, 350);
});
if (window.requestIdleCallback) requestIdleCallback(run, { timeout: 1500 });
else setTimeout(run, 400);
}
async function atrGenerateMiniCoverNow(item) {
const { img, fileName, key, doneKey } = item || {};
if (!img || !img.complete || !img.naturalWidth || !img.naturalHeight) return;
const maxW = 150;
const maxH = 200;
let w = img.naturalWidth;
let h = img.naturalHeight;
const ratio = Math.min(maxW / w, maxH / h, 1);
if (ratio >= 1) return;
w = Math.max(1, Math.round(w * ratio));
h = Math.max(1, Math.round(h * ratio));
const canvas = document.createElement("canvas");
canvas.width = w;
canvas.height = h;
const ctx = canvas.getContext("2d", { alpha:false });
if (!ctx) return;
ctx.imageSmoothingEnabled = true;
ctx.imageSmoothingQuality = "medium";
ctx.drawImage(img, 0, 0, w, h);
const blob = await new Promise(resolve => {
if (canvas.toBlob) canvas.toBlob(resolve, "image/jpeg", 0.68);
else resolve(null);
});
if (!blob || blob.size <= 0 || blob.size > 196608) return;
const fd = new FormData();
fd.append("mini", blob, key + ".jpg");
const r = await fetch("/api/covers/mini_upload?name=" + encodeURIComponent(fileName || key), { method:"POST", body:fd });
if (r.ok) localStorage.setItem(doneKey, "1");
}
function atrShowCoverDirect(unit, fileName, url, sourceText) {
atrCoverSetImage(unit, fileName, url, sourceText || "Fuente: URL manual", false);
}
function atrCoverEscapeAttr(v) {
return String(v || "")
.replace(/&/g, "&amp;")
.replace(/"/g, "&quot;")
.replace(/'/g, "&#39;")
.replace(/</g, "&lt;")
.replace(/>/g, "&gt;");
}
function atrCoverActionsHtml(unit, fileName, hasCover) {
// V33: estos botones ya no pertenecen a WEB-ATR; se muestran solo en Biblioteca.
return "";
}
function atrLibraryCoverActionsHtml(fileName) {
const safeName = atrCoverEscapeAttr(fileName || "");
let h = `<div class="atr-cover-library-actions">`;
h += `<button type="button" class="atr-cover-mini-btn" onclick="atrPromptManualCover(0, this.dataset.name)" data-name="${safeName}">Agregar URL</button>`;
h += `<button type="button" class="atr-cover-mini-btn" onclick="atrUploadLocalCover(this.dataset.name)" data-name="${safeName}">Subir imagen</button>`;
h += `<button type="button" class="atr-cover-mini-btn" onclick="window.open(ATR_LIBRETRO_BOXART_BASE, '_blank')">Abrir Libretro</button>`;
h += `<button type="button" class="atr-cover-mini-btn" onclick="atrUseProxyCoverLibrary(this.dataset.name)" data-name="${safeName}">Guardar proxy SD</button>`;
if (atrManualCoverGet(fileName)) {
h += `<button type="button" class="atr-cover-mini-btn" onclick="atrRemoveManualCover(0, this.dataset.name)" data-name="${safeName}">Quitar manual</button>`;
}
h += `</div>`;
return h;
}
async function atrUseProxyCoverLibrary(fileName) {
try {
setStatus("Buscando y guardando carátula en SD...", true);
delete atrCoverResolvedCache[atrCoverCacheKey(fileName)];
const found = await atrServerCoverResolve(fileName);
if (!found || !found.image) throw new Error("No se encontró carátula para este archivo");
const saved = await atrServerCoverSet(fileName, found.image, found.title || fileName);
if (!saved || !saved.ok) throw new Error(saved && saved.reason ? saved.reason : "No se pudo guardar en SD");
atrManualCoverSet(fileName, saved.image || found.image, saved.title || found.title || fileName);
atrCoverResolvedCache[atrCoverCacheKey(fileName)] = { image: saved.image || found.image, sourceText: "Fuente: SD" };
if (typeof loadAtrStatus === "function") await loadAtrStatus({ files:true, force:true, forceCovers:true, forceLibrary:true });
setStatus("Carátula guardada en SD.", true);
} catch(e) {
setStatus("No se pudo guardar carátula en SD: " + e.message, false);
alert("No se pudo guardar la carátula en la SD. Detalle: " + e.message);
}
}
function atrUploadLocalCover(fileName) {
const input = document.createElement("input");
input.type = "file";
input.accept = "image/png,image/jpeg,image/webp,image/gif,image/bmp";
input.style.display = "none";
input.addEventListener("change", async () => {
const file = input.files && input.files[0];
input.remove();
if (!file) return;
try {
setStatus("Subiendo imagen a SD...", true);
const fd = new FormData();
fd.append("cover", file, file.name);
const url = "/api/covers/upload?name=" + encodeURIComponent(fileName || file.name) + "&title=" + encodeURIComponent(fileName || file.name);
const r = await fetch(url, { method:"POST", body:fd });
const txt = await r.text();
let j = null;
try { j = JSON.parse(txt); } catch(e) {}
if (!r.ok || !j || !j.ok) throw new Error((j && j.reason) ? j.reason : (txt || ("HTTP " + r.status)));
atrManualCoverSet(fileName, j.image, j.title || fileName);
delete atrCoverResolvedCache[atrCoverCacheKey(fileName)];
if (typeof loadAtrStatus === "function") await loadAtrStatus({ files:true, force:true, forceCovers:true, forceLibrary:true });
setStatus("Imagen guardada en SD.", true);
} catch(e) {
setStatus("Error subiendo imagen: " + e.message, false);
alert("No se pudo subir la imagen a la SD. Detalle: " + e.message);
}
});
document.body.appendChild(input);
input.click();
}
async function atrPromptManualCover(unit, fileName) {
const current = atrManualCoverGet(fileName);
const msg = "Pega la URL directa de la carátula. Se descargará y guardará en la SD para:\n\n" + (fileName || ("D" + unit));
const url = prompt(msg, current && current.image ? current.image : "");
if (!url) return;
const clean = atrNormalizeImageUrl(url);
if (!/^https?:\/\//i.test(clean)) {
alert("La URL debe comenzar con http:// o https://");
return;
}
const key = atrCoverCacheKey(fileName);
delete atrCoverResolvedCache[key];
atrManualCoverSet(fileName, clean, fileName);
atrCoverResolvedCache[key] = { image: clean, sourceText: "Fuente: URL manual" };
if (Number(unit) > 0) atrCoverSetImage(unit, fileName, clean, "Fuente: URL manual", false);
try {
const resp = await atrServerCoverSet(fileName, clean, fileName);
if (!resp || !resp.ok) throw new Error(resp && resp.reason ? resp.reason : "No se pudo guardar");
const savedImg = resp.image || clean;
const savedOnSd = !(resp.cacheWarning || String(resp.source || "").indexOf("remote") >= 0);
atrManualCoverSet(fileName, savedImg, resp.title || fileName);
atrCoverResolvedCache[key] = { image: savedImg, sourceText: savedOnSd ? "Fuente: SD" : "Fuente: URL guardada en MASTER" };
if (Number(unit) > 0) atrCoverSetImage(unit, fileName, savedImg, savedOnSd ? "Fuente: SD" : "Fuente: URL guardada en MASTER", false);
const subEl = $(`atrCoverSub${unit}`);
if (subEl) subEl.textContent = savedOnSd ? "Fuente: imagen guardada en SD" : "Fuente: URL guardada en MASTER";
if (resp.cacheWarning) setStatus("URL guardada en MASTER. La cache SD no se pudo crear ahora.", true);
if (Number(unit) <= 0 && typeof loadAtrStatus === "function") loadAtrStatus({ files:true, force:true, forceCovers:true, forceLibrary:true });
} catch(e) {
if (Number(unit) <= 0 && typeof loadAtrStatus === "function") loadAtrStatus({ files:true, force:true, forceCovers:true, forceLibrary:true });
alert("No se pudo guardar en el MASTER. Quedó guardada solo en este navegador. Detalle: " + e.message);
}
}
async function atrRemoveManualCover(unit, fileName) {
if (!confirm("¿Eliminar la URL manual de carátula para " + (fileName || ("D" + unit)) + "?")) return;
try {
await atrServerCoverDelete(fileName);
} catch(e) {}
atrManualCoverDelete(fileName);
delete atrCoverResolvedCache[atrCoverCacheKey(fileName)];
if (typeof loadAtrStatus === "function") loadAtrStatus({ files:true, force:true, forceCovers:true, forceLibrary:true });
}
// =============================================================
// F27 - Render Biblioteca tipo mockup real
// Reemplaza solo la presentación de Biblioteca. No cambia montaje,
// descarga, API ni lógica crítica SIO/TNFS.
// =============================================================
function casApplyLibraryMountStateF49Z13(fileName, status) {
try {
fileName = String(fileName || "");
if (!fileName && status) fileName = String(status.casName || status.name || "");
if (!fileName) return;
const apply = (arr) => {
if (!Array.isArray(arr)) return arr;
arr.forEach(f => {
if (!f) return;
const isCas = String((f.type || f.ext || String(f.name || "").split('.').pop() || "")).replace(/^\./,'').toUpperCase() === "CAS";
if (isCas) f.casMounted = String(f.name || "").toLowerCase() === fileName.toLowerCase() ? 1 : 0;
if (isCas) f.casPlaying = 0;
});
return arr;
};
if (typeof lastAtrStatus !== "undefined" && lastAtrStatus) {
lastAtrStatus.casMounted = 1;
lastAtrStatus.casPlaying = 0;
lastAtrStatus.casName = fileName;
lastAtrStatus.casPath = status?.path || status?.casPath || lastAtrStatus.casPath || "";
apply(lastAtrStatus.files);
}
if (window.lastAtrStatus) {
window.lastAtrStatus.casMounted = 1;
window.lastAtrStatus.casPlaying = 0;
window.lastAtrStatus.casName = fileName;
window.lastAtrStatus.casPath = status?.path || status?.casPath || window.lastAtrStatus.casPath || "";
apply(window.lastAtrStatus.files);
}
if (Array.isArray(window.atrLibraryFilesCache)) apply(window.atrLibraryFilesCache);
if (typeof atrLibraryFilesCache !== "undefined" && Array.isArray(atrLibraryFilesCache)) apply(atrLibraryFilesCache);
if (window.libraryStateF49Z47) { apply(window.libraryStateF49Z47.files); if (window.libraryStateF49Z47.lastPayload) apply(window.libraryStateF49Z47.lastPayload.files); }
if (typeof updateCasManualPanelF49 === "function") updateCasManualPanelF49(Object.assign({ casMounted:1, casPlaying:0, casName:fileName, name:fileName }, status || {}));
let touched = 0;
if (typeof atrLibraryCardHtmlF27 === "function") {
const files = atrLibraryCurrentFilesF27();
document.querySelectorAll('.mock-lib-card[data-file-type="CAS"]').forEach(card => {
const n = card.getAttribute('data-file-name') || "";
if (n.toLowerCase() === fileName.toLowerCase() || card.classList.contains('mounted')) {
const f = files.find(x => String(x?.name || "").toLowerCase() === n.toLowerCase());
if (f) { card.outerHTML = atrLibraryCardHtmlF27(f); touched++; }
}
});
if (touched && typeof atrInitLazyCovers === "function") setTimeout(atrInitLazyCovers, 0);
}
if (!touched && !(document.body && document.body.classList.contains("route-library")) && typeof renderAtrLibrary === "function") renderAtrLibrary(atrLibraryCurrentFilesF27(), { force:true });
} catch(e) { console.warn("cas fast library update", e); }
}
async function mountCasQuick(enc) {
const name = decodeURIComponent(enc || "");
try {
setStatus("Preparando C: " + name + "...", true);
const res = await fetch(`/cas/mount?file=${enc}&fast=1`, { cache:"no-store" });
const txt = await res.text();
let j = null;
try { j = txt ? JSON.parse(txt) : null; } catch(e) {}
if (!res.ok || (j && j.ok === false)) throw new Error((j && j.error) ? j.error : (txt || ("HTTP " + res.status)));
casApplyLibraryMountStateF49Z13(name, j || {});
setStatus("C: preparado: " + name + ". Entra a Cassette para Play/Boot.", true);
} catch(e) { alert("No se pudo preparar el cassette: " + (e.message || e)); }
}
function casApplyLibraryUnmountStateF49Z16(status) {
try {
const apply = (arr) => {
if (!Array.isArray(arr)) return arr;
arr.forEach(f => {
if (!f) return;
const isCas = String((f.type || f.ext || String(f.name || "").split('.').pop() || "")).replace(/^\./,'').toUpperCase() === "CAS";
if (isCas) { f.casMounted = 0; f.casPlaying = 0; }
});
return arr;
};
if (typeof lastAtrStatus !== "undefined" && lastAtrStatus) {
lastAtrStatus.casMounted = 0;
lastAtrStatus.casPlaying = 0;
lastAtrStatus.casName = "";
lastAtrStatus.casPath = "";
apply(lastAtrStatus.files);
}
if (window.lastAtrStatus) {
window.lastAtrStatus.casMounted = 0;
window.lastAtrStatus.casPlaying = 0;
window.lastAtrStatus.casName = "";
window.lastAtrStatus.casPath = "";
apply(window.lastAtrStatus.files);
}
if (Array.isArray(window.atrLibraryFilesCache)) apply(window.atrLibraryFilesCache);
if (typeof atrLibraryFilesCache !== "undefined" && Array.isArray(atrLibraryFilesCache)) apply(atrLibraryFilesCache);
if (window.libraryStateF49Z47) { apply(window.libraryStateF49Z47.files); if (window.libraryStateF49Z47.lastPayload) apply(window.libraryStateF49Z47.lastPayload.files); }
if (typeof updateCasManualPanelF49 === "function") updateCasManualPanelF49(Object.assign({ casMounted:0, casPlaying:0, casName:"", name:"" }, status || {}));
document.querySelectorAll('.mock-lib-card[data-file-type="CAS"]').forEach(card => {
card.classList.remove('mounted');
const actions = card.querySelector('.mock-card-actions');
const n = card.getAttribute('data-file-name') || "";
const enc = encodeURIComponent(n);
if (actions && n) {
const files = (typeof atrLibraryCurrentFilesF27 === "function") ? atrLibraryCurrentFilesF27() : (window.atrLibraryFilesCache || []);
const f = files.find(x => String(x?.name || "").toLowerCase() === n.toLowerCase());
if (f && typeof atrLibraryCardHtmlF27 === "function") card.outerHTML = atrLibraryCardHtmlF27(f);
else actions.innerHTML = `<button type="button" class="mock-unit-btn" onclick="mountCasQuick('${enc}')" title="Preparar cassette C:">Preparar C:</button>`;
}
});
} catch(e) { console.warn("cas unmount local library update", e); }
}
async function unmountCasQuick() {
try {
const res = await fetch(`/cas/unmount?fast=1`, { cache:"no-store" });
const txt = await res.text();
let j = null;
try { j = txt ? JSON.parse(txt) : null; } catch(e) {}
if (!res.ok || (j && j.ok === false)) throw new Error((j && j.error) ? j.error : (txt || ("HTTP " + res.status)));
if (typeof casSoundStopF49I === "function") casSoundStopF49I(true);
const onLibrary = !!(document.body && document.body.classList && document.body.classList.contains("route-library"));
if (onLibrary) {
  casApplyLibraryUnmountStateF49Z16(j || {});
} else {
  if (typeof updateCasManualPanelF49 === "function") updateCasManualPanelF49(Object.assign({ casMounted:0, mounted:0, casPlaying:0, playing:0, casName:"", name:"" }, j || {}));
  if (window.casLastStatusF49J) Object.assign(window.casLastStatusF49J, { casMounted:0, mounted:0, casPlaying:0, playing:0, casName:"", name:"" });
}
if (typeof loadCasStatusF49C === "function") await loadCasStatusF49C({lite:true, force:true});
if (typeof casStartLiveRefreshF49J === "function") casStartLiveRefreshF49J(true);
setStatus("C: liberado. Biblioteca conservada.", true);
} catch(e) { alert("No se pudo soltar el cassette: " + (e.message || e)); }
}
async function casPlayModeQuick(mode) {
try {
const casSt = window.casLastStatusF49J || {};
const casMountedNow = !!Number(casSt.casMounted || casSt.mounted || 0);
if (!casMountedNow) {
if (typeof setStatus === "function") setStatus("No hay cassette preparado en C:. Selecciona un .CAS desde Biblioteca.", false);
alert("No hay CAS montado en C:. Selecciona/prepara un .CAS desde Biblioteca antes de usar Play.");
if (typeof updateCasManualPanelF49 === "function") updateCasManualPanelF49(Object.assign({}, casSt, { casMounted:0, mounted:0, casPlaying:0, playing:0 }));
return;
}
mode = (mode === "boot" || mode === "auto") ? mode : "cload";
if ($("casLoadMode")) $("casLoadMode").value = mode;
const cfg = (typeof casCurrentTurboCfgF49C === "function") ? casCurrentTurboCfgF49C() : {baud:600,mult:100,delay:0,turbo:0};
if (typeof casSaveTurboCfgF49C === "function") casSaveTurboCfgF49C(cfg);
const qs = new URLSearchParams();
qs.set("mode", mode);
qs.set("baud", String(cfg.baud || 600));
qs.set("mult", String(cfg.mult || 100));
qs.set("auto", String(cfg.auto ? 1 : 0));
// F49X: Boot usa modo SDrive-MAX con ventana de drenaje dinámica y transporte confirmado.
qs.set("delay", String(Number(cfg.delay || 0)));
qs.set("turbo", String(cfg.turbo ? 1 : 0));
const res = await fetch(`/cas/play?${qs.toString()}`, { cache:"no-store" });
if (!res.ok) throw new Error(await res.text());
if (typeof casSoundStartF49I === "function") await casSoundStartF49I();
if (typeof loadCasStatusF49C === "function") await loadCasStatusF49C();
if (typeof casStartLiveRefreshF49J === "function") casStartLiveRefreshF49J(true);
if (typeof loadAtrStatus === "function" && (typeof routeKindF44 !== "function" || routeKindF44(location.pathname) === "library")) await loadAtrStatus({ files:true, refreshFiles:false, force:true, forceLibrary:true });
} catch(e) { alert("No se pudo iniciar cassette: " + (e.message || e)); }
}
async function casPlayCloadQuick() { return casPlayModeQuick("cload"); }
async function casPlayBootQuick() { return casPlayModeQuick("boot"); }
async function casPlayQuick() { return casPlayModeQuick(($("casLoadMode") && $("casLoadMode").value) || "boot"); }
async function casStopQuick() {
try {
const res = await fetch(`/cas/stop`, { cache:"no-store" });
if (!res.ok) throw new Error(await res.text());
if (typeof casSoundStopF49I === "function") casSoundStopF49I(true);
if (typeof loadCasStatusF49C === "function") await loadCasStatusF49C();
if (typeof casStartLiveRefreshF49J === "function") casStartLiveRefreshF49J(true);
if (typeof loadAtrStatus === "function" && (typeof routeKindF44 !== "function" || routeKindF44(location.pathname) === "library")) await loadAtrStatus({ files:true, refreshFiles:false, force:true, forceLibrary:true });
} catch(e) { alert("No se pudo detener cassette: " + (e.message || e)); }
}
async function casPauseQuick() {
try {
const st = window.casLastStatusF49J || {};
const paused = !!Number(st.casPaused || st.paused || 0);
const res = await fetch(`/cas/pause?state=${paused ? 0 : 1}`, { cache:"no-store" });
if (!res.ok) throw new Error(await res.text());
if (typeof loadCasStatusF49C === "function") await loadCasStatusF49C();
if (typeof casStartLiveRefreshF49J === "function") casStartLiveRefreshF49J(true);
} catch(e) { alert("No se pudo pausar/continuar cassette: " + (e.message || e)); }
}
async function casCounterBackQuick() {
try {
let turns = Number(($("casCounterBackInput") && $("casCounterBackInput").value) || 3);
if (!Number.isFinite(turns) || turns < 1) turns = 3;
if (turns > 20) turns = 20;
const res = await fetch(`/cas/seek_back?turns=${encodeURIComponent(turns)}`, { cache:"no-store" });
if (!res.ok) throw new Error(await res.text());
if (typeof casSoundRewindF49I === "function") await casSoundRewindF49I();
if (typeof loadCasStatusF49C === "function") await loadCasStatusF49C();
if (typeof casStartLiveRefreshF49J === "function") casStartLiveRefreshF49J(true);
} catch(e) { alert("No se pudo volver el contador: " + (e.message || e)); }
}
async function casRewindQuick() {
try {
const res = await fetch(`/cas/rewind`, { cache:"no-store" });
if (!res.ok) throw new Error(await res.text());
if (typeof casSoundRewindF49I === "function") await casSoundRewindF49I();
if (typeof loadCasStatusF49C === "function") await loadCasStatusF49C();
if (typeof casStartLiveRefreshF49J === "function") casStartLiveRefreshF49J(true);
if (typeof loadAtrStatus === "function" && (typeof routeKindF44 !== "function" || routeKindF44(location.pathname) === "library")) await loadAtrStatus({ files:true, refreshFiles:false, force:true, forceLibrary:true });
} catch(e) { alert("No se pudo rebobinar cassette: " + (e.message || e)); }
}
window.atrLibraryTypeFilterF27 = window.atrLibraryTypeFilterF27 || "all";
window.atrLibrarySortF27 = window.atrLibrarySortF27 || "name";
const ATR_LIBRARY_PAGE_SIZE_F49Z20 = 20;
window.atrLibraryPageF49Z20 = Number(window.atrLibraryPageF49Z20 || 0);
let atrLibraryLastRenderSigF27 = "";
function atrLibraryTypeF27(f) {
const name = String((f && f.name) || "");
const ext = String((name.match(/\.([^\.\/\\]+)$/) || [])[1] || "").replace(/^\./,'').toUpperCase();
// F49Z50: el filtro de Biblioteca usa la extensión real. Antes COM/EXE podían
// quedar agrupados como XEX y los botones mostraban archivos cruzados.
if (["ATR","XEX","COM","EXE","CAS","SEC"].includes(ext)) return ext;
const t = String((f && (f.type || f.ext)) || ext || "OTHER").replace(/^\./,'').toUpperCase();
return (["ATR","XEX","COM","EXE","CAS","SEC"].includes(t)) ? t : "OTHER";
}
function atrLibrarySubtitleF27(f) {
const name = String((f && f.name) || "");
const path = String((f && (f.path || f.folder || f.dir)) || "");
if (path && path !== "/" && path !== name) return path.replace(/^\/+/, "");
const base = name.replace(/\.[^.]+$/, "").replace(/[_\-]+/g, " ").trim();
return base || "Biblioteca Atari";
}
function atrLibrarySortListF27(list) {
const mode = window.atrLibrarySortF27 || "name";
const collator = new Intl.Collator("es", { sensitivity:"base", numeric:true });
return list.sort((a,b) => {
if (mode === "size") return Number(b.fileSize || b.size || 0) - Number(a.fileSize || a.size || 0);
if (mode === "type") {
const t = collator.compare(atrLibraryTypeF27(a), atrLibraryTypeF27(b));
return t || collator.compare(String(a.name || ""), String(b.name || ""));
}
if (mode === "mounted") return Number(b.mountedMask || 0) - Number(a.mountedMask || 0) || collator.compare(String(a.name || ""), String(b.name || ""));
return collator.compare(String(a.name || ""), String(b.name || ""));
});
}
function atrLibraryApplyTypeFilterF27(list) {
const filter = String(window.atrLibraryTypeFilterF27 || "all").toUpperCase();
if (filter === "ALL") return list;
return list.filter(f => atrLibraryTypeF27(f) === filter);
}
function atrLibraryTypeCountsF27(files) {
const c = { ALL:(files || []).length };
(files || []).forEach(f => { const t = atrLibraryTypeF27(f); c[t] = (c[t] || 0) + 1; });
return c;
}
function atrLibraryCurrentFilesF27() {
try { if (typeof lastAtrStatus !== "undefined" && lastAtrStatus && Array.isArray(lastAtrStatus.files) && lastAtrStatus.files.length) return lastAtrStatus.files; } catch(e) {}
try { if (window.lastAtrStatus && Array.isArray(window.lastAtrStatus.files) && window.lastAtrStatus.files.length) return window.lastAtrStatus.files; } catch(e) {}
try { if (typeof atrLibraryFilesCache !== "undefined" && Array.isArray(atrLibraryFilesCache) && atrLibraryFilesCache.length) return atrLibraryFilesCache; } catch(e) {}
try { if (Array.isArray(window.atrLibraryFilesCache) && window.atrLibraryFilesCache.length) return window.atrLibraryFilesCache; } catch(e) {}
try { if (typeof lastAtrStatus !== "undefined" && lastAtrStatus && Array.isArray(lastAtrStatus.files)) return lastAtrStatus.files; } catch(e) {}
try { if (window.lastAtrStatus && Array.isArray(window.lastAtrStatus.files)) return window.lastAtrStatus.files; } catch(e) {}
return [];
}
function atrLibraryScaffoldF27(files) {
const bar = document.querySelector("#atrLibraryPageBlock .library-toolbar-main");
if (!bar || bar.dataset.f27 === "1") return;
bar.dataset.f27 = "1";
const chips = document.createElement("div");
chips.className = "mock-filter-chips";
chips.id = "mockFilterChipsF27";
chips.innerHTML = ["all","ATR","XEX","COM","EXE","CAS"].map(t =>
`<button type="button" class="mock-filter-chip" data-filter="${t}">${t === "all" ? "Todos" : t}</button>`
).join("");
bar.insertBefore(chips, bar.firstChild);
const sort = document.createElement("label");
sort.className = "mock-sort-wrap";
sort.innerHTML = `Ordenar por
<select id="atrLibrarySortF27" class="small-input">
<option value="name">Nombre (A-Z)</option>
<option value="type">Tipo</option>
<option value="size">Tamaño</option>
<option value="mounted">Montados primero</option>
</select>`;
bar.appendChild(sort);
chips.addEventListener("click", ev => {
const btn = ev.target.closest("button[data-filter]");
if (!btn) return;
window.atrLibraryTypeFilterF27 = btn.dataset.filter || "all";
atrLibraryResetPageF49Z20();
renderAtrLibrary(atrLibraryCurrentFilesF27(), { force:true });
});
const sel = document.getElementById("atrLibrarySortF27");
if (sel) sel.addEventListener("change", () => {
window.atrLibrarySortF27 = sel.value || "name";
atrLibraryResetPageF49Z20();
renderAtrLibrary(atrLibraryCurrentFilesF27(), { force:true });
});
}
function atrLibraryUpdateChipsF27(files) {
const counts = atrLibraryTypeCountsF27(files || []);
document.querySelectorAll("#mockFilterChipsF27 .mock-filter-chip").forEach(btn => {
const f = String(btn.dataset.filter || "all");
const key = f.toUpperCase();
const label = f === "all" ? "Todos" : f;
const n = counts[key] || 0;
btn.textContent = label + (n ? ` ${n}` : "");
btn.classList.toggle("active", String(window.atrLibraryTypeFilterF27 || "all").toLowerCase() === f.toLowerCase());
});
const sel = document.getElementById("atrLibrarySortF27");
if (sel && sel.value !== (window.atrLibrarySortF27 || "name")) sel.value = window.atrLibrarySortF27 || "name";
}
function atrLibraryMenuHtmlF27(f, enc, isMounted) {
let body = atrLibraryCoverActionsHtml(f.name || "");
const type = atrLibraryTypeF27(f);
if (type === "CAS") {
if (Number(f.casMounted || 0)) {
body += `<a class="button-link" href="/cassette">📼 Ver cassette</a>`;
} else {
body += `<button type="button" class="secondary" onclick="mountCasQuick('${enc}')">📼 Preparar C:</button>`;
}
}
if (!isMounted && !Number(f.casMounted || 0)) {
if (f.protected) body += `<span class="protected-badge">Protegido</span>`;
else body += `<button type="button" class="secondary trash-btn" onclick="deleteAtr('${enc}')">🗑 Borrar archivo</button>`;
}
return `<details class="mock-card-menu"><summary title="Acciones">⋮</summary><div class="mock-card-menu-body">${body}</div></details>`;
}
function atrLibraryCardHtmlF27(f) {
const isMounted = Number(f.mountedMask || 0) !== 0;
const isCasMounted = Number(f.casMounted || 0) !== 0;
const type = atrLibraryTypeF27(f);
const isCas = type === "CAS";
const enc = encodeURIComponent(f.name || "");
const visibleMask = visibleUnitMaskFromStatus((typeof lastAtrStatus !== "undefined" && lastAtrStatus) ? lastAtrStatus : window.lastAtrStatus);
let unitButtons = "";
if (isCas) {
unitButtons = isCasMounted
? `<span class="mounted-badge mock-mounted-badge">Preparado: C:</span><a class="mock-unit-btn" href="/cassette" title="Ver control cassette">Ver cassette</a>`
: `<button type="button" class="mock-unit-btn" onclick="mountCasQuick('${enc}')" title="Preparar cassette C:">Preparar C:</button>`;
} else if (isMounted) {
unitButtons = `<span class="mounted-badge mock-mounted-badge">Montado: ${escapeHtml(maskToUnits(f.mountedMask))}</span>`;
} else {
for (let i=1;i<=UI_MAX_UNITS;i++) {
if (unitVisible(visibleMask, i)) unitButtons += `<button type="button" class="mock-unit-btn" onclick="mountAtrQuick('${enc}',${i})" title="Montar en D${i}">D${i}</button>`;
}
}
const size = humanBytes(f.fileSize || f.size || 0);
const sector = isCas ? "C:" : (f.sectorSize ? `${f.sectorSize}B` : "—");
const sectors = isCas ? "Cassette" : (f.totalSectors ? `${f.totalSectors} sec.` : "—");
const protectedBadge = f.protected ? `<span>Protegido</span>` : "";
return `<article class="mock-lib-card${(isMounted || isCasMounted) ? ' mounted' : ''}" data-file-name="${escapeHtml(f.name || '')}" data-file-type="${escapeHtml(type)}">`+
`<div class="mock-lib-cover-wrap">`+
`<span class="mock-lib-type">${escapeHtml(type)}</span>`+
atrLibraryCoverHtml(f.name || "")+
`</div>`+
`<div class="mock-lib-title" title="${escapeHtml(f.name || '')}">${escapeHtml(f.name || '')}</div>`+
`<div class="mock-lib-subtitle" title="${escapeHtml(atrLibrarySubtitleF27(f))}">${escapeHtml(atrLibrarySubtitleF27(f))}</div>`+
`<div class="mock-lib-meta"><span>${size}</span><span>${sector}</span><span>${sectors}</span>${protectedBadge}</div>`+
`<div class="mock-lib-units">${unitButtons}${atrLibraryMenuHtmlF27(f, enc, isMounted)}</div>`+
`</article>`;
}
function renderAtrLibrary(files, opts = {}) {
const lib = $("atrLibrary");
if (!lib) return;
atrLibraryScaffoldF27(files || []);
atrLibraryUpdateChipsF27(files || []);
let list = (files || []).slice();
const q = String($("atrLibrarySearch")?.value || "").trim().toLowerCase();
if (q) list = list.filter(f => String(f.name || "").toLowerCase().includes(q) || atrLibraryTypeF27(f).toLowerCase().includes(q));
list = atrLibraryApplyTypeFilterF27(list);
if (!list.length && (files || []).length && !q && String(window.atrLibraryTypeFilterF27 || "all").toLowerCase() !== "all") {
// F49Z17: si quedó un filtro de tipo activo y oculta todo, volver a Todos para que Biblioteca muestre archivos.
window.atrLibraryTypeFilterF27 = "all";
list = (files || []).slice();
}
list = atrLibrarySortListF27(list);
const mode = atrLibraryViewMode === "folders" ? "folders" : "flat";
const totalFilteredF49Z20 = list.length;
const pagesF49Z20 = Math.max(1, Math.ceil(totalFilteredF49Z20 / ATR_LIBRARY_PAGE_SIZE_F49Z20));
let pageF49Z20 = Math.max(0, Math.min(Number(window.atrLibraryPageF49Z20 || 0), pagesF49Z20 - 1));
window.atrLibraryPageF49Z20 = pageF49Z20;
const startF49Z20 = pageF49Z20 * ATR_LIBRARY_PAGE_SIZE_F49Z20;
const endF49Z20 = Math.min(totalFilteredF49Z20, startF49Z20 + ATR_LIBRARY_PAGE_SIZE_F49Z20);
const pageListF49Z20 = list.slice(startF49Z20, endF49Z20);
const sig = JSON.stringify(list.map(f => [f.name || "", f.type || "", f.fileSize || f.size || 0, f.mountedMask || 0, f.casMounted || 0, f.protected ? 1 : 0])) + "|" + q + "|" + mode + "|" + window.atrLibraryTypeFilterF27 + "|" + window.atrLibrarySortF27 + "|p" + pageF49Z20;
if (!opts.force && sig === atrLibraryLastRenderSigF27) return;
atrLibraryLastRenderSigF27 = sig;
const summary = $("atrLibrarySummary");
if (summary) summary.textContent = totalFilteredF49Z20 > ATR_LIBRARY_PAGE_SIZE_F49Z20
? `${totalFilteredF49Z20} de ${(files || []).length} archivos · página ${pageF49Z20 + 1}/${pagesF49Z20}`
: `${totalFilteredF49Z20} de ${(files || []).length} archivos`;
lib.className = "atr-help atr-library-list mock-lib-grid";
if (!list.length) {
atrLibraryRenderPagerF49Z20(0, 0, 1, 0, 0);
lib.innerHTML = `<div class="mock-lib-empty">${q ? "Sin resultados para la búsqueda." : "Sin archivos disponibles en la biblioteca."}</div>`;
return;
}
atrLibraryRenderPagerF49Z20(totalFilteredF49Z20, pageF49Z20, pagesF49Z20, startF49Z20, endF49Z20);
if (mode === "flat") {
lib.innerHTML = pageListF49Z20.map(atrLibraryCardHtmlF27).join("");
setTimeout(atrInitLazyCovers, 0);
return;
}
const groups = {};
pageListF49Z20.forEach(f => {
const k = atrGroupKey(f.name);
if (!groups[k]) groups[k] = [];
groups[k].push(f);
});
const order = ["0-9", ..."ABCDEFGHIJKLMNOPQRSTUVWXYZ".split(""), "#"];
lib.innerHTML = order.filter(k => groups[k]?.length).map(k => {
const items = groups[k];
return `<details class="mock-folder-group" open>`+
`<summary><span>${escapeHtml(k)}</span><span class="pill">${items.length}</span></summary>`+
`<div class="mock-folder-grid">${items.map(atrLibraryCardHtmlF27).join("")}</div>`+
`</details>`;
}).join("");
setTimeout(atrInitLazyCovers, 0);
}
(function initF27LibraryAfterLoad(){
const run = () => {
try { atrLibraryScaffoldF27(atrLibraryCurrentFilesF27()); } catch(e) {}
};
if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", run);
else run();
})();
/* =============================================================
F49Z17 - Biblioteca: asegurar render de archivos al entrar/refrescar
============================================================= */
(function libraryRenderFixF49Z17(){
if (window.__libraryRenderFixF49Z17) return;
window.__libraryRenderFixF49Z17 = true;
function isLibraryRouteF49Z17(){
const p = String(location.pathname || "").toLowerCase();
return p.indexOf("/library") === 0 || p.indexOf("/editor/library") === 0 || p.indexOf("/upload") === 0;
}
function libraryHasCardsF49Z17(){
const lib = typeof $ === "function" ? $("atrLibrary") : document.getElementById("atrLibrary");
return !!(lib && lib.querySelector(".mock-lib-card,.atr-file-row"));
}
function showCachedLibraryF49Z17(force){
try {
if (!isLibraryRouteF49Z17()) return false;
const files = (typeof atrLibraryCurrentFilesF27 === "function") ? atrLibraryCurrentFilesF27() : [];
if (!Array.isArray(files) || !files.length || typeof renderAtrLibrary !== "function") return false;
renderAtrLibrary(files, { force: !!force });
return true;
} catch(e) { console.warn("library cached render F49Z17", e); return false; }
}
window.forceShowLibraryF49Z17 = function(refresh){
try {
if (typeof window.loadLibraryPageF49Z45 === "function") return window.loadLibraryPageF49Z45({ refresh:!!refresh, force:true });
if (typeof loadAtrStatus === "function") return loadAtrStatus({ files:true, refreshFiles:false, force:true, forceLibrary:true });
} catch(e) {}
};
const oldLoad = (typeof loadAtrStatus === "function") ? loadAtrStatus : null;
if (oldLoad && !oldLoad.__f49z17Wrapped) {
const wrapped = async function(opts = {}) {
const libRoute = isLibraryRouteF49Z17();
if (libRoute) showCachedLibraryF49Z17(false);
const r = await oldLoad.apply(this, arguments);
if (libRoute) {
showCachedLibraryF49Z17(!!(opts && (opts.force || opts.forceLibrary || opts.files || opts.refreshFiles)));
setTimeout(() => { if (!libraryHasCardsF49Z17()) window.forceShowLibraryF49Z17(false); }, 650);
}
return r;
};
wrapped.__f49z17Wrapped = true;
loadAtrStatus = wrapped;
}
function bindLibraryRefreshF49Z17(){
const btn = typeof $ === "function" ? $("btnLibraryRefresh") : document.getElementById("btnLibraryRefresh");
if (btn && !btn.dataset.f49z17) {
btn.dataset.f49z17 = "1";
btn.addEventListener("click", ev => {
ev.preventDefault();
ev.stopImmediatePropagation();
window.atrLibraryTypeFilterF27 = "all";
if (typeof atrLibraryResetPageF49Z20 === "function") atrLibraryResetPageF49Z20();
window.forceShowLibraryF49Z17(true);
}, true);
}
}
function kickLibraryF49Z17(){
bindLibraryRefreshF49Z17();
if (!isLibraryRouteF49Z17()) return;
showCachedLibraryF49Z17(true);
setTimeout(() => { if (!libraryHasCardsF49Z17()) window.forceShowLibraryF49Z17(false); }, 900);
}
if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", kickLibraryF49Z17);
else kickLibraryF49Z17();
window.addEventListener("popstate", () => setTimeout(kickLibraryF49Z17, 80));
})();
/* =============================================================
F49Z22 - Aviso corto OPTION/RESET
F49Z23 - CAS no muestra aviso OPTION/RESET y se detecta por extension
- No cambia el protocolo SIO ni el montaje.
- Solo orienta cuando un ATR requiere BASIC OFF en XL/XE.
============================================================= */
/* =============================================================
F49Z20 - Biblioteca paginada
- Muestra 20 archivos por pagina con Anterior/Siguiente.
- Selector de subida en Biblioteca colapsable por defecto.
============================================================= */
/* =============================================================
F49H - Biblioteca CAS solo prepara / Ver cassette
- Elimina de Biblioteca acciones Play, Stop, Rewind y Soltar C:.
- Si C: está preparado, muestra Ver cassette para entrar a /cassette.
============================================================= */
/* ===== F38 Visual Settings runtime: editable themes + image width/height ===== */
const VISUAL_SETTINGS_KEY_F38 = "atariSioVisualSettingsF38";
const VISUAL_SETTINGS_KEY_F32 = "atariSioVisualSettingsF32";
const VISUAL_CUSTOM_THEMES_KEY_F38 = "atariSioCustomThemesF38";
const VISUAL_THEME_OVERRIDES_KEY_F38 = "atariSioThemeOverridesF38";
const VISUAL_BUILTIN_THEMES_F38 = {
"blue-pro": {id:"blue-pro", builtin:true, name:"Azul profesional / actual", note:"Actual", primaryColor:"#1ea7ff", accentColor:"#59d7ff", bgColor:"#020817", accentGlow:18, fontFamily:"system"},
"classic-red": {id:"classic-red", builtin:true, name:"Rojo clásico / versión anterior", note:"Versión anterior", primaryColor:"#ef4444", accentColor:"#fb923c", bgColor:"#120607", accentGlow:18, fontFamily:"segoe"},
"fuji-cyan": {id:"fuji-cyan", builtin:true, name:"Cian FujiNet", note:"Red/Internet", primaryColor:"#00d0ff", accentColor:"#00f5d4", bgColor:"#031019", accentGlow:20, fontFamily:"system"},
"atari-green": {id:"atari-green", builtin:true, name:"Verde Atari / terminal", note:"Terminal", primaryColor:"#22c55e", accentColor:"#86efac", bgColor:"#02110a", accentGlow:16, fontFamily:"atari"},
"amber-retro": {id:"amber-retro", builtin:true, name:"Ámbar retro", note:"Monitor antiguo", primaryColor:"#f59e0b", accentColor:"#fde68a", bgColor:"#120a02", accentGlow:15, fontFamily:"courier"},
"purple-neon": {id:"purple-neon", builtin:true, name:"Morado neón", note:"Moderno", primaryColor:"#a855f7", accentColor:"#22d3ee", bgColor:"#09071a", accentGlow:22, fontFamily:"system"},
"mono-classic": {id:"mono-classic", builtin:true, name:"Monocromo técnico", note:"Sobrio", primaryColor:"#94a3b8", accentColor:"#e2e8f0", bgColor:"#050607", accentGlow:10, fontFamily:"mono"}
};
const VISUAL_DEFAULTS_F38 = {
themePreset:"blue-pro",
primaryColor:"#1ea7ff",
accentColor:"#59d7ff",
bgColor:"#020817",
accentGlow:18,
libraryCoverWidth:180,
libraryCoverHeight:190,
fullscreenWidth:86,
fullscreenHeight:86,
fontFamily:"system",
logoTitle:"Atari SIO",
logoSubtitle:"Controller",
logoUrl:"",
logoData:""
};
function clampF32(v,min,max){ v=Number(v); if(!Number.isFinite(v)) return min; return Math.max(min, Math.min(max, v)); }
function hexToRgbF32(hex){
hex=String(hex||"").trim();
const m=/^#?([a-f0-9]{6})$/i.exec(hex);
if(!m) return {r:30,g:167,b:255};
const n=parseInt(m[1],16);
return {r:(n>>16)&255,g:(n>>8)&255,b:n&255};
}
function fontValueF32(key){
switch(String(key||"system")){
case "segoe": return '"Segoe UI", system-ui, -apple-system, sans-serif';
case "arial": return 'Arial, Helvetica, sans-serif';
case "verdana": return 'Verdana, Geneva, sans-serif';
case "mono": return 'ui-monospace, SFMono-Regular, Menlo, Consolas, monospace';
case "courier": return '"Courier New", Courier, monospace';
case "atari": return '"Atari Classic", "Atari 8-bit", "Atari 400/800", "AtariSmall", "Press Start 2P", "Courier New", ui-monospace, monospace';
default: return 'system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';
}
}
function visualStorageReadF38(key, fallback){
try{ const raw=localStorage.getItem(key); return raw ? JSON.parse(raw) : fallback; }catch(e){ return fallback; }
}
function visualStorageWriteF38(key, val){
try{ localStorage.setItem(key, JSON.stringify(val)); }catch(e){}
}
function visualCustomThemesF38(){
const arr=visualStorageReadF38(VISUAL_CUSTOM_THEMES_KEY_F38, []);
return Array.isArray(arr) ? arr.filter(t=>t && t.id && t.name) : [];
}
function saveVisualCustomThemesF38(arr){ visualStorageWriteF38(VISUAL_CUSTOM_THEMES_KEY_F38, Array.isArray(arr)?arr:[]); }
function visualThemeOverridesF38(){
const obj=visualStorageReadF38(VISUAL_THEME_OVERRIDES_KEY_F38, {});
return obj && typeof obj === "object" && !Array.isArray(obj) ? obj : {};
}
function saveVisualThemeOverridesF38(obj){ visualStorageWriteF38(VISUAL_THEME_OVERRIDES_KEY_F38, obj && typeof obj==="object" ? obj : {}); }
function getAllThemesF38(){
const overrides=visualThemeOverridesF38();
const built=Object.keys(VISUAL_BUILTIN_THEMES_F38).map(id=>({...VISUAL_BUILTIN_THEMES_F38[id], ...(overrides[id]||{}), id, builtin:true}));
const custom=visualCustomThemesF38().map(t=>({...t, builtin:false}));
return [...built, ...custom];
}
function getThemeByIdF38(id){ return getAllThemesF38().find(t=>t.id===id) || getAllThemesF38()[0] || VISUAL_BUILTIN_THEMES_F38["blue-pro"]; }
function slugThemeIdF38(name){
const base=String(name||"tema").normalize("NFD").replace(/[\u0300-\u036f]/g,"").toLowerCase().replace(/[^a-z0-9]+/g,"-").replace(/^-+|-+$/g,"").slice(0,28) || "tema";
const used=new Set(getAllThemesF38().map(t=>t.id));
let id="custom-"+base, n=2;
while(used.has(id)){ id="custom-"+base+"-"+(n++); }
return id;
}
function currentVisualFormF38(){
const val=(id,fallback)=>{ const el=$(id); return el ? el.value : fallback; };
return {
themePreset: val("uiThemePreset", visualSettingsF38.themePreset || "blue-pro"),
name: String(val("uiThemeName", "Tema personalizado")||"Tema personalizado").trim().slice(0,32),
primaryColor: val("uiPrimaryColor", VISUAL_DEFAULTS_F38.primaryColor),
accentColor: val("uiAccentColor", VISUAL_DEFAULTS_F38.accentColor),
bgColor: val("uiBgColor", VISUAL_DEFAULTS_F38.bgColor),
accentGlow: clampF32(val("uiAccentGlow", VISUAL_DEFAULTS_F38.accentGlow),5,35),
libraryCoverWidth: clampF32(val("uiLibraryCoverWidth", VISUAL_DEFAULTS_F38.libraryCoverWidth),120,420),
libraryCoverHeight: clampF32(val("uiLibraryCoverHeight", VISUAL_DEFAULTS_F38.libraryCoverHeight),120,420),
fullscreenWidth: clampF32(val("uiFullscreenWidth", VISUAL_DEFAULTS_F38.fullscreenWidth),40,96),
fullscreenHeight: clampF32(val("uiFullscreenHeight", VISUAL_DEFAULTS_F38.fullscreenHeight),40,96),
fontFamily: val("uiFontFamily", VISUAL_DEFAULTS_F38.fontFamily),
logoTitle: val("uiLogoTitle", VISUAL_DEFAULTS_F38.logoTitle),
logoSubtitle: val("uiLogoSubtitle", VISUAL_DEFAULTS_F38.logoSubtitle),
logoUrl: val("uiLogoUrl", ""),
logoData: visualSettingsF38.logoData || ""
};
}
function loadVisualSettingsF32(){ return loadVisualSettingsF38(); }
function loadVisualSettingsF38(){
const existing=visualStorageReadF38(VISUAL_SETTINGS_KEY_F38, null);
if(existing) return {...VISUAL_DEFAULTS_F38, ...existing};
const old=visualStorageReadF38(VISUAL_SETTINGS_KEY_F32, null);
if(old){
return {
...VISUAL_DEFAULTS_F38,
...old,
libraryCoverWidth: old.libraryCoverWidth || Math.max(160, Number(old.libraryCardSize||220)-40),
libraryCoverHeight: old.libraryCoverHeight || old.libraryCoverSize || VISUAL_DEFAULTS_F38.libraryCoverHeight,
fullscreenWidth: old.fullscreenWidth || old.fullscreenSize || VISUAL_DEFAULTS_F38.fullscreenWidth,
fullscreenHeight: old.fullscreenHeight || old.fullscreenSize || VISUAL_DEFAULTS_F38.fullscreenHeight
};
}
return {...VISUAL_DEFAULTS_F38};
}
function saveVisualSettingsF32(s){ saveVisualSettingsF38(s); }
function saveVisualSettingsF38(s){ visualStorageWriteF38(VISUAL_SETTINGS_KEY_F38, {...VISUAL_DEFAULTS_F38, ...s}); }
let visualSettingsF38 = loadVisualSettingsF38();
let visualSettingsF32 = visualSettingsF38;
function applyVisualSettingsF32(settings){ applyVisualSettingsF38(settings); }
function applyVisualSettingsF38(settings){
const s={...VISUAL_DEFAULTS_F38, ...(settings||{})};
const root=document.documentElement;
const p=hexToRgbF32(s.primaryColor), a=hexToRgbF32(s.accentColor), b=hexToRgbF32(s.bgColor);
root.style.setProperty("--accent", s.primaryColor, "important");
root.style.setProperty("--accent-soft", `rgba(${p.r},${p.g},${p.b},.16)`, "important");
root.style.setProperty("--accent-strong", `rgba(${p.r},${p.g},${p.b},.46)`, "important");
root.style.setProperty("--accent2", s.accentColor, "important");
root.style.setProperty("--accent-rgb", `${p.r},${p.g},${p.b}`, "important");
root.style.setProperty("--accent2-rgb", `${a.r},${a.g},${a.b}`, "important");
root.style.setProperty("--app-bg", s.bgColor, "important");
root.style.setProperty("--app-bg-rgb", `${b.r},${b.g},${b.b}`, "important");
root.style.setProperty("--ui-glow-alpha", String(clampF32(s.accentGlow,5,35)/100), "important");
const coverW=clampF32(s.libraryCoverWidth,120,420), coverH=clampF32(s.libraryCoverHeight,120,420);
root.style.setProperty("--lib-cover-w", `${coverW}px`, "important");
root.style.setProperty("--lib-cover-h", `${coverH}px`, "important");
root.style.setProperty("--lib-card-min", `${Math.max(170, coverW + 28)}px`, "important");
root.style.setProperty("--fullscreen-img-w", `${clampF32(s.fullscreenWidth,40,96)}vw`, "important");
root.style.setProperty("--fullscreen-img-h", `${clampF32(s.fullscreenHeight,40,96)}vh`, "important");
root.style.setProperty("--app-font", fontValueF32(s.fontFamily), "important");
document.body.classList.toggle("font-atari", String(s.fontFamily||"") === "atari");
// F39: todos los colores de tema se propagan por variables CSS.
// Se remueven clases antiguas para evitar que reglas heredadas fuercen azul u otros colores.
document.body.classList.remove("theme-blue-pro","theme-classic-red","theme-fuji-cyan","theme-atari-green","theme-amber-retro","theme-purple-neon","theme-mono-classic");
const brandTitle=document.querySelector(".brand-title");
const brandSub=document.querySelector(".brand-subtitle");
const logoImg=$("brandLogoImg");
if(brandTitle) brandTitle.textContent = String(s.logoTitle || VISUAL_DEFAULTS_F38.logoTitle).slice(0,32);
if(brandSub) brandSub.textContent = String(s.logoSubtitle || VISUAL_DEFAULTS_F38.logoSubtitle).slice(0,32);
const logoSrc = s.logoData || s.logoUrl || "";
if(logoImg && logoSrc){ logoImg.src = logoSrc; document.body.classList.add("has-custom-logo"); }
else { if(logoImg) logoImg.removeAttribute("src"); document.body.classList.remove("has-custom-logo"); }
updatePreviewPanelF38(s);
}
function updatePreviewPanelF38(s){
const title=$("uiPreviewTitle"), sub=$("uiPreviewSubtitle"), cover=$("uiPreviewCover"), glow=$("uiAccentGlowValue");
const primary=String(s.primaryColor||VISUAL_DEFAULTS_F38.primaryColor);
const accent=String(s.accentColor||VISUAL_DEFAULTS_F38.accentColor);
const bg=String(s.bgColor||VISUAL_DEFAULTS_F38.bgColor);
const p=hexToRgbF32(primary);
if(title) title.textContent=String(s.logoTitle||VISUAL_DEFAULTS_F38.logoTitle).slice(0,32);
if(sub) sub.textContent=String(s.logoSubtitle||VISUAL_DEFAULTS_F38.logoSubtitle).slice(0,32);
if(cover){
cover.style.width=`min(100%, ${clampF32(s.libraryCoverWidth,120,420)}px)`;
cover.style.height=`${clampF32(s.libraryCoverHeight,120,420)}px`;
cover.style.borderColor=`rgba(${p.r},${p.g},${p.b},.38)`;
cover.style.background=`linear-gradient(145deg, rgba(${p.r},${p.g},${p.b},.22), ${bg})`;
}
const sw1=$("f38PreviewPrimary"), sw2=$("f38PreviewAccent"), sw3=$("f38PreviewBg");
if(sw1) sw1.style.background=primary;
if(sw2) sw2.style.background=accent;
if(sw3) sw3.style.background=bg;
const logo=document.querySelector(".f38-preview-logo");
if(logo){
logo.style.background=`linear-gradient(180deg, ${primary}, ${accent})`;
logo.style.boxShadow=`0 14px 30px rgba(${p.r},${p.g},${p.b},${clampF32(s.accentGlow,5,35)/100})`;
}
const preview=document.querySelector(".f38-side-preview");
if(preview){
preview.style.borderColor=`rgba(${p.r},${p.g},${p.b},.30)`;
preview.style.boxShadow=`0 18px 45px rgba(${p.r},${p.g},${p.b},${clampF32(s.accentGlow,5,35)/160})`;
}
document.querySelectorAll(".f38-preview-actions span").forEach(x=>{
x.style.background=`linear-gradient(180deg, ${primary}, ${accent})`;
x.style.borderColor=`rgba(${p.r},${p.g},${p.b},.82)`;
});
if(glow) glow.textContent=`${clampF32(s.accentGlow,5,35)}%`;
}
function populateThemeSelectF38(){
const sel=$("uiThemePreset"); if(!sel) return;
const current=sel.value || visualSettingsF38.themePreset || "blue-pro";
sel.innerHTML=getAllThemesF38().map(t=>`<option value="${escapeHtml(t.id)}">${escapeHtml(t.name)}${t.builtin?"":" · personalizado"}</option>`).join("");
sel.value=getThemeByIdF38(current).id;
}
function syncVisualSettingsFormF32(){ syncVisualSettingsFormF38(); }
function syncVisualSettingsFormF38(){
populateThemeSelectF38();
const s=visualSettingsF38;
const set=(id,val)=>{ const el=$(id); if(el) el.value=val; };
set("uiThemePreset", s.themePreset || "blue-pro");
const theme=getThemeByIdF38(s.themePreset || "blue-pro");
set("uiThemeName", theme.name || s.name || "");
set("uiPrimaryColor", s.primaryColor);
set("uiAccentColor", s.accentColor);
set("uiBgColor", s.bgColor);
set("uiAccentGlow", clampF32(s.accentGlow,5,35));
set("uiLibraryCoverWidth", clampF32(s.libraryCoverWidth,120,420));
set("uiLibraryCoverHeight", clampF32(s.libraryCoverHeight,120,420));
set("uiFullscreenWidth", clampF32(s.fullscreenWidth,40,96));
set("uiFullscreenHeight", clampF32(s.fullscreenHeight,40,96));
set("uiFontFamily", s.fontFamily);
set("uiLogoTitle", s.logoTitle);
set("uiLogoSubtitle", s.logoSubtitle);
set("uiLogoUrl", s.logoUrl || "");
updatePreviewPanelF38(s);
}
function updateVisualSettingsFromFormF32(save=true){ updateVisualSettingsFromFormF38(save); }
function updateVisualSettingsFromFormF38(save=true){
const form=currentVisualFormF38();
visualSettingsF38={...visualSettingsF38, ...form};
visualSettingsF32=visualSettingsF38;
applyVisualSettingsF38(visualSettingsF38);
if(save) saveVisualSettingsF38(visualSettingsF38);
}
function setVisualStatusF32(msg){ setVisualStatusF38(msg); }
function setVisualStatusF38(msg){ const el=$("uiSettingsStatus"); if(el) el.textContent=msg; }
function applyThemePresetF36(key, save=true){ selectThemeF38(key, save); }
function renderThemeChipsF36(){ /* F38: las cards de temas quedan eliminadas. */ }
function selectThemeF38(id, save=true){
const t=getThemeByIdF38(id || "blue-pro");
visualSettingsF38={...visualSettingsF38, ...t, themePreset:t.id, name:t.name, logoTitle:visualSettingsF38.logoTitle, logoSubtitle:visualSettingsF38.logoSubtitle, logoUrl:visualSettingsF38.logoUrl, logoData:visualSettingsF38.logoData};
visualSettingsF32=visualSettingsF38;
syncVisualSettingsFormF38();
applyVisualSettingsF38(visualSettingsF38);
if(save) saveVisualSettingsF38(visualSettingsF38);
setVisualStatusF38("Tema visual aplicado: "+(t.name||t.id)+".");
}
function saveCurrentThemeF38(){
updateVisualSettingsFromFormF38(false);
const form=currentVisualFormF38();
const id=form.themePreset || "blue-pro";
const theme={
id,
name:form.name || getThemeByIdF38(id).name || "Tema",
primaryColor:form.primaryColor,
accentColor:form.accentColor,
bgColor:form.bgColor,
accentGlow:form.accentGlow,
fontFamily:form.fontFamily
};
if(String(id).startsWith("custom-")){
const arr=visualCustomThemesF38();
const ix=arr.findIndex(t=>t.id===id);
if(ix>=0) arr[ix]={...arr[ix], ...theme, builtin:false}; else arr.push({...theme, builtin:false});
saveVisualCustomThemesF38(arr);
}else{
const overrides=visualThemeOverridesF38();
overrides[id]=theme;
saveVisualThemeOverridesF38(overrides);
}
visualSettingsF38={...visualSettingsF38, ...theme, themePreset:id};
visualSettingsF32=visualSettingsF38;
saveVisualSettingsF38(visualSettingsF38);
populateThemeSelectF38();
syncVisualSettingsFormF38();
applyVisualSettingsF38(visualSettingsF38);
setVisualStatusF38("Tema guardado: "+theme.name+".");
}
function saveCurrentThemeAsF38(){
updateVisualSettingsFromFormF38(false);
const form=currentVisualFormF38();
let name=String(form.name||"").trim();
if(!name){ name=prompt("Nombre del nuevo tema:", "Mi tema azul"); }
if(!name) { setVisualStatusF38("No se creó el tema porque falta el nombre."); return; }
const id=slugThemeIdF38(name);
const theme={id, name:name.slice(0,32), primaryColor:form.primaryColor, accentColor:form.accentColor, bgColor:form.bgColor, accentGlow:form.accentGlow, fontFamily:form.fontFamily, builtin:false};
const arr=visualCustomThemesF38();
arr.push(theme);
saveVisualCustomThemesF38(arr);
visualSettingsF38={...visualSettingsF38, ...theme, themePreset:id};
visualSettingsF32=visualSettingsF38;
saveVisualSettingsF38(visualSettingsF38);
populateThemeSelectF38();
syncVisualSettingsFormF38();
applyVisualSettingsF38(visualSettingsF38);
setVisualStatusF38("Nuevo tema creado y guardado: "+theme.name+".");
}
function deleteOrResetThemeF38(){
const id=$("uiThemePreset")?.value || visualSettingsF38.themePreset || "blue-pro";
const t=getThemeByIdF38(id);
if(String(id).startsWith("custom-")){
if(!confirm("¿Eliminar el tema personalizado '"+(t.name||id)+"'?")) return;
saveVisualCustomThemesF38(visualCustomThemesF38().filter(x=>x.id!==id));
selectThemeF38("blue-pro", true);
setVisualStatusF38("Tema personalizado eliminado. Se volvió al tema azul profesional.");
return;
}
const overrides=visualThemeOverridesF38();
if(overrides[id]){
if(!confirm("¿Restablecer los colores originales de '"+(VISUAL_BUILTIN_THEMES_F38[id]?.name||id)+"'?")) return;
delete overrides[id];
saveVisualThemeOverridesF38(overrides);
selectThemeF38(id, true);
setVisualStatusF38("Tema predefinido restablecido.");
}else{
setVisualStatusF38("Este tema predefinido no tiene cambios guardados para restablecer.");
}
}
function bindVisualSettingsF32(){ bindVisualSettingsF38(); }
function bindVisualSettingsF38(){
if(window.__f38VisualBound) return;
window.__f38VisualBound=true;
const themeSel=$("uiThemePreset");
if(themeSel) themeSel.addEventListener("change",()=>selectThemeF38(themeSel.value, true));
["uiThemeName","uiPrimaryColor","uiAccentColor","uiBgColor","uiAccentGlow","uiLibraryCoverWidth","uiLibraryCoverHeight","uiFullscreenWidth","uiFullscreenHeight","uiFontFamily","uiLogoTitle","uiLogoSubtitle","uiLogoUrl"].forEach(id=>{
const el=$(id);
if(el) el.addEventListener("input",()=>{ updateVisualSettingsFromFormF38(true); setVisualStatusF38("Cambios aplicados en vivo. Usa 'Guardar cambios del tema' para actualizar la paleta seleccionada."); });
});
const saveBtn=$("btnThemeSave"); if(saveBtn) saveBtn.addEventListener("click",saveCurrentThemeF38);
const saveAsBtn=$("btnThemeSaveAs"); if(saveAsBtn) saveAsBtn.addEventListener("click",saveCurrentThemeAsF38);
const delBtn=$("btnThemeDelete"); if(delBtn) delBtn.addEventListener("click",deleteOrResetThemeF38);
const resetBtn=$("btnUiReset");
if(resetBtn) resetBtn.addEventListener("click",()=>{
if(!confirm("¿Restablecer apariencia completa al tema Azul profesional?")) return;
visualSettingsF38={...VISUAL_DEFAULTS_F38};
visualSettingsF32=visualSettingsF38;
saveVisualSettingsF38(visualSettingsF38);
syncVisualSettingsFormF38();
applyVisualSettingsF38(visualSettingsF38);
setVisualStatusF38("Apariencia restablecida al tema azul profesional.");
});
const clearLogo=$("btnUiLogoClear");
if(clearLogo) clearLogo.addEventListener("click",()=>{
visualSettingsF38.logoData=""; visualSettingsF38.logoUrl="";
const url=$("uiLogoUrl"); if(url) url.value="";
const file=$("uiLogoFile"); if(file) file.value="";
saveVisualSettingsF38(visualSettingsF38);
applyVisualSettingsF38(visualSettingsF38);
syncVisualSettingsFormF38();
setVisualStatusF38("Logo personalizado eliminado.");
});
const logoFile=$("uiLogoFile");
if(logoFile) logoFile.addEventListener("change",()=>{
const file=logoFile.files && logoFile.files[0];
if(!file) return;
if(file.size > 256*1024){ setVisualStatusF38("El logo es muy grande. Usa una imagen menor a 256 KB."); logoFile.value=""; return; }
const reader=new FileReader();
reader.onload=()=>{
visualSettingsF38.logoData=String(reader.result||"");
visualSettingsF38.logoUrl="";
const url=$("uiLogoUrl"); if(url) url.value="";
saveVisualSettingsF38(visualSettingsF38);
applyVisualSettingsF38(visualSettingsF38);
syncVisualSettingsFormF38();
setVisualStatusF38("Logo local aplicado y guardado en este navegador.");
};
reader.onerror=()=>setVisualStatusF38("No se pudo cargar el logo local.");
reader.readAsDataURL(file);
});
}
(function initVisualSettingsF38(){
applyVisualSettingsF38(visualSettingsF38);
const run=()=>{
populateThemeSelectF38();
syncVisualSettingsFormF38();
bindVisualSettingsF38();
applyVisualSettingsF38(visualSettingsF38);
};
if(document.readyState === "loading") document.addEventListener("DOMContentLoaded", run);
else run();
})();
/* ===== F40 runtime patch: aplicación real de colores + mapa de glifos solo letra ===== */
function applyThemeVarsF40(settings){
const s={...VISUAL_DEFAULTS_F38, ...(settings||visualSettingsF38||{})};
const p=hexToRgbF32(s.primaryColor), a=hexToRgbF32(s.accentColor), b=hexToRgbF32(s.bgColor);
[document.documentElement, document.body].forEach(target=>{
if(!target) return;
target.style.setProperty('--accent', s.primaryColor, 'important');
target.style.setProperty('--accent-soft', `rgba(${p.r},${p.g},${p.b},.16)`, 'important');
target.style.setProperty('--accent-strong', `rgba(${p.r},${p.g},${p.b},.46)`, 'important');
target.style.setProperty('--accent2', s.accentColor, 'important');
target.style.setProperty('--accent-rgb', `${p.r},${p.g},${p.b}`, 'important');
target.style.setProperty('--accent2-rgb', `${a.r},${a.g},${a.b}`, 'important');
target.style.setProperty('--app-bg', s.bgColor, 'important');
target.style.setProperty('--app-bg-rgb', `${b.r},${b.g},${b.b}`, 'important');
target.style.setProperty('--ui-glow-alpha', String(clampF32(s.accentGlow,5,35)/100), 'important');
});
}
function updatePreviewPanelF40(settings){
const s={...VISUAL_DEFAULTS_F38, ...(settings||visualSettingsF38||{})};
const p=hexToRgbF32(s.primaryColor), a=hexToRgbF32(s.accentColor), b=hexToRgbF32(s.bgColor);
const primary=s.primaryColor, accent=s.accentColor, bg=s.bgColor;
const setImp=(el,prop,val)=>{ if(el) el.style.setProperty(prop,val,'important'); };
setImp($('f38PreviewPrimary'),'background',primary);
setImp($('f38PreviewAccent'),'background',accent);
setImp($('f38PreviewBg'),'background',bg);
const cover=$('uiPreviewCover');
if(cover){
setImp(cover,'width',`min(100%, ${clampF32(s.libraryCoverWidth,120,420)}px)`);
setImp(cover,'height',`${clampF32(s.libraryCoverHeight,120,420)}px`);
setImp(cover,'border-color',`rgba(${p.r},${p.g},${p.b},.44)`);
setImp(cover,'background',`linear-gradient(145deg, rgba(${p.r},${p.g},${p.b},.22), rgba(${b.r},${b.g},${b.b},.96))`);
}
const preview=document.querySelector('.f38-side-preview');
if(preview){
setImp(preview,'border-color',`rgba(${p.r},${p.g},${p.b},.32)`);
setImp(preview,'box-shadow',`0 18px 45px rgba(${p.r},${p.g},${p.b},${clampF32(s.accentGlow,5,35)/180})`);
}
const card=document.querySelector('.f38-preview-card');
if(card){
setImp(card,'border-color',`rgba(${p.r},${p.g},${p.b},.34)`);
setImp(card,'background',`radial-gradient(circle at 20% 0%, rgba(${p.r},${p.g},${p.b},.20), transparent 44%), linear-gradient(180deg, rgba(${b.r},${b.g},${b.b},.96), rgba(5,10,20,.98))`);
}
const logo=document.querySelector('.f38-preview-logo');
if(logo){
setImp(logo,'background',`rgba(${p.r},${p.g},${p.b},.12)`);
setImp(logo,'color',primary);
setImp(logo,'border-color',`rgba(${p.r},${p.g},${p.b},.42)`);
setImp(logo,'box-shadow','none');
}
document.querySelectorAll('.f38-preview-actions span').forEach(x=>{
setImp(x,'background',`linear-gradient(180deg, ${primary}, ${accent})`);
setImp(x,'border-color',`rgba(${p.r},${p.g},${p.b},.82)`);
setImp(x,'color','#fff');
});
const title=$('uiPreviewTitle'), sub=$('uiPreviewSubtitle');
if(title) title.textContent=String(s.logoTitle||VISUAL_DEFAULTS_F38.logoTitle).slice(0,32);
if(sub) sub.textContent=String(s.logoSubtitle||VISUAL_DEFAULTS_F38.logoSubtitle).slice(0,32);
}
(function patchVisualSettingsF40(){
if(window.__f40ThemePatch) return;
window.__f40ThemePatch=true;
const oldApply=window.applyVisualSettingsF38;
window.applyVisualSettingsF38=function(settings){
if(typeof oldApply==='function') oldApply(settings);
applyThemeVarsF40(settings||visualSettingsF38);
updatePreviewPanelF40(settings||visualSettingsF38);
};
const oldUpdatePreview=window.updatePreviewPanelF38;
window.updatePreviewPanelF38=function(settings){
if(typeof oldUpdatePreview==='function') oldUpdatePreview(settings);
applyThemeVarsF40(settings||visualSettingsF38);
updatePreviewPanelF40(settings||visualSettingsF38);
};
const oldBuild=window.buildGlyphMatrixV27;
window.buildGlyphMatrixV27=function(force){
const grid=$('glyphMatrixV27');
if(!grid){ if(typeof oldBuild==='function') return oldBuild(force); return; }
if(force){ grid.dataset.ready='0'; grid.innerHTML=''; }
if(grid.dataset.ready==='1') return;
grid.dataset.ready='1';
const blank=document.createElement('div'); blank.className='glyph-axis'; blank.textContent=''; grid.appendChild(blank);
for(let c=0;c<16;c++){ const h=document.createElement('div'); h.className='glyph-axis'; h.textContent=c.toString(16).toUpperCase().padStart(2,'0'); grid.appendChild(h); }
for(let r=0;r<8;r++){
const row=document.createElement('div'); row.className='glyph-axis'; row.textContent=(r*16).toString(16).toUpperCase().padStart(2,'0'); grid.appendChild(row);
for(let c=0;c<16;c++){
const code=r*16+c;
const b=document.createElement('button');
b.type='button';
b.className='glyph-code-cell glyph-code-letter-only';
b.dataset.code=String(code);
b.dataset.codehex=code.toString(16).toUpperCase().padStart(2,'0');
b.title='0x'+b.dataset.codehex+' · '+glyphCharLabelV27(code);
const label=document.createElement('span');
label.className='glyph-code-letter';
label.textContent=glyphCharLabelV27(code);
b.appendChild(label);
b.addEventListener('click',()=>{
const wasOpen=glyphFullscreenIsOpenV29(code);
glyphSelectCodeV27(code,true);
setTimeout(()=>{
if(wasOpen) glyphFullscreenCloseV29();
else glyphFullscreenOpenV29(code,glyphRowsForCodeV27(code));
},40);
});
grid.appendChild(b);
}
}
};
const bindExtra=()=>{
['uiPrimaryColor','uiAccentColor','uiBgColor','uiAccentGlow','uiLibraryCoverWidth','uiLibraryCoverHeight','uiFullscreenWidth','uiFullscreenHeight'].forEach(id=>{
const el=$(id);
if(el && !el.dataset.f40Bound){
el.dataset.f40Bound='1';
['input','change'].forEach(ev=>el.addEventListener(ev,()=>{
const form=typeof currentVisualFormF38==='function'?currentVisualFormF38():visualSettingsF38;
visualSettingsF38={...visualSettingsF38,...form};
visualSettingsF32=visualSettingsF38;
saveVisualSettingsF38(visualSettingsF38);
applyVisualSettingsF38(visualSettingsF38);
setVisualStatusF38('Cambios aplicados en vivo. Usa Guardar cambios del tema para actualizar la paleta seleccionada.');
}));
}
});
applyVisualSettingsF38(visualSettingsF38);
const gm=$('glyphMatrixV27');
if(gm && gm.dataset.ready==='1' && gm.querySelector('.glyph-mini-pixels')) buildGlyphMatrixV27(true);
};
if(document.readyState==='loading') document.addEventListener('DOMContentLoaded',bindExtra); else bindExtra();
setTimeout(bindExtra,350);
})();
/* ===== F41 runtime: propagación total de tema y auditoría de páginas ===== */
function applyThemeVarsF41(settings){
const s={...VISUAL_DEFAULTS_F38, ...(settings||visualSettingsF38||{})};
const p=hexToRgbF32(s.primaryColor), a=hexToRgbF32(s.accentColor), b=hexToRgbF32(s.bgColor);
const important=(el,name,val)=>{ if(el) el.style.setProperty(name,val,'important'); };
[document.documentElement, document.body].forEach(el=>{
important(el,'--accent',s.primaryColor);
important(el,'--accent2',s.accentColor);
important(el,'--accent-rgb',`${p.r},${p.g},${p.b}`);
important(el,'--accent2-rgb',`${a.r},${a.g},${a.b}`);
important(el,'--accent-soft',`rgba(${p.r},${p.g},${p.b},.16)`);
important(el,'--accent-strong',`rgba(${p.r},${p.g},${p.b},.46)`);
important(el,'--app-bg',s.bgColor);
important(el,'--app-bg-rgb',`${b.r},${b.g},${b.b}`);
important(el,'--ui-glow-alpha',String(clampF32(s.accentGlow,5,35)/100));
important(el,'--lib-cover-w',`${clampF32(s.libraryCoverWidth,120,420)}px`);
important(el,'--lib-cover-h',`${clampF32(s.libraryCoverHeight,120,420)}px`);
important(el,'--lib-card-min',`${Math.max(170,clampF32(s.libraryCoverWidth,120,420)+28)}px`);
important(el,'--fullscreen-img-w',`${clampF32(s.fullscreenWidth,40,96)}vw`);
important(el,'--fullscreen-img-h',`${clampF32(s.fullscreenHeight,40,96)}vh`);
important(el,'--app-font',fontValueF32(s.fontFamily));
});
}
function updatePreviewPanelF41(settings){
const s={...VISUAL_DEFAULTS_F38, ...(settings||visualSettingsF38||{})};
const p=hexToRgbF32(s.primaryColor), a=hexToRgbF32(s.accentColor), b=hexToRgbF32(s.bgColor);
const set=(el,prop,val)=>{ if(el) el.style.setProperty(prop,val,'important'); };
set($('f38PreviewPrimary'),'background',s.primaryColor);
set($('f38PreviewAccent'),'background',s.accentColor);
set($('f38PreviewBg'),'background',s.bgColor);
const card=document.querySelector('.f38-preview-card');
if(card){
set(card,'border-color',`rgba(${p.r},${p.g},${p.b},.34)`);
set(card,'background',`radial-gradient(circle at 20% 0%, rgba(${p.r},${p.g},${p.b},.22), transparent 44%), linear-gradient(180deg, rgba(${b.r},${b.g},${b.b},.94), rgba(5,10,20,.98))`);
}
const logo=document.querySelector('.f38-preview-logo');
if(logo){
set(logo,'background',`rgba(${p.r},${p.g},${p.b},.12)`);
set(logo,'color',s.primaryColor);
set(logo,'border-color',`rgba(${p.r},${p.g},${p.b},.42)`);
set(logo,'box-shadow','none');
}
const cover=$('uiPreviewCover');
if(cover){
set(cover,'width',`min(100%, ${clampF32(s.libraryCoverWidth,120,420)}px)`);
set(cover,'height',`${clampF32(s.libraryCoverHeight,120,420)}px`);
set(cover,'border-color',`rgba(${p.r},${p.g},${p.b},.42)`);
set(cover,'background',`linear-gradient(145deg, rgba(${p.r},${p.g},${p.b},.22), rgba(${b.r},${b.g},${b.b},.96))`);
}
document.querySelectorAll('.f38-preview-actions span').forEach(x=>{
set(x,'background',`linear-gradient(180deg, ${s.primaryColor}, ${s.accentColor})`);
set(x,'border-color',`rgba(${p.r},${p.g},${p.b},.82)`);
set(x,'color','#fff');
});
const title=$('uiPreviewTitle'), sub=$('uiPreviewSubtitle'), glow=$('uiAccentGlowValue');
if(title) title.textContent=String(s.logoTitle||VISUAL_DEFAULTS_F38.logoTitle).slice(0,32);
if(sub) sub.textContent=String(s.logoSubtitle||VISUAL_DEFAULTS_F38.logoSubtitle).slice(0,32);
if(glow) glow.textContent=`${clampF32(s.accentGlow,5,35)}%`;
}
function applyVisualSettingsF41(settings, save=false){
const s={...VISUAL_DEFAULTS_F38, ...(settings||visualSettingsF38||{})};
visualSettingsF38=s;
visualSettingsF32=s;
applyThemeVarsF41(s);
updatePreviewPanelF41(s);
if(save) saveVisualSettingsF38(s);
}
(function patchThemeAuditF41(){
if(window.__f41ThemeAudit) return;
window.__f41ThemeAudit=true;
const oldApply=window.applyVisualSettingsF38;
window.applyVisualSettingsF38=function(settings){
if(typeof oldApply==='function') oldApply(settings);
applyVisualSettingsF41(settings||visualSettingsF38,false);
};
const oldPreview=window.updatePreviewPanelF38;
window.updatePreviewPanelF38=function(settings){
if(typeof oldPreview==='function') oldPreview(settings);
updatePreviewPanelF41(settings||visualSettingsF38);
};
const oldSelect=window.selectThemeF38;
window.selectThemeF38=function(id, save=true){
if(typeof oldSelect==='function') oldSelect(id, save);
applyVisualSettingsF41(visualSettingsF38, !!save);
setTimeout(()=>applyVisualSettingsF41(visualSettingsF38,false),80);
};
const bind=()=>{
const ids=['uiPrimaryColor','uiAccentColor','uiBgColor','uiAccentGlow','uiLibraryCoverWidth','uiLibraryCoverHeight','uiFullscreenWidth','uiFullscreenHeight','uiFontFamily'];
ids.forEach(id=>{
const el=$(id);
if(!el || el.dataset.f41Bound) return;
el.dataset.f41Bound='1';
['input','change','blur','keyup'].forEach(ev=>el.addEventListener(ev,()=>{
const form=typeof currentVisualFormF38==='function'?currentVisualFormF38():visualSettingsF38;
visualSettingsF38={...visualSettingsF38,...form};
visualSettingsF32=visualSettingsF38;
saveVisualSettingsF38(visualSettingsF38);
applyVisualSettingsF41(visualSettingsF38,false);
if(typeof setVisualStatusF38==='function') setVisualStatusF38('Cambios aplicados en vivo. Usa Guardar cambios del tema para actualizar la paleta seleccionada.');
}));
});
applyVisualSettingsF41(visualSettingsF38,false);
};
if(document.readyState==='loading') document.addEventListener('DOMContentLoaded',bind); else bind();
setTimeout(bind,250);
setTimeout(()=>applyVisualSettingsF41(visualSettingsF38,false),700);
try{
const obs=new MutationObserver(()=>applyThemeVarsF41(visualSettingsF38));
obs.observe(document.body,{childList:true,subtree:true});
}catch(e){}
})();
/* ===== F43 runtime: aplicar tema también a nodos dinámicos y limpiar residuos visuales ===== */
function applyThemeCleanupF43(settings){
const s={...VISUAL_DEFAULTS_F38, ...(settings||visualSettingsF38||{})};
const p=hexToRgbF32(s.primaryColor), a=hexToRgbF32(s.accentColor), b=hexToRgbF32(s.bgColor);
const set=(el,prop,val)=>{ if(el) el.style.setProperty(prop,val,'important'); };
[document.documentElement, document.body].forEach(el=>{
set(el,'--accent',s.primaryColor);
set(el,'--accent2',s.accentColor);
set(el,'--accent-rgb',`${p.r},${p.g},${p.b}`);
set(el,'--accent2-rgb',`${a.r},${a.g},${a.b}`);
set(el,'--accent-soft',`rgba(${p.r},${p.g},${p.b},.16)`);
set(el,'--accent-strong',`rgba(${p.r},${p.g},${p.b},.46)`);
set(el,'--app-bg',s.bgColor);
set(el,'--app-bg-rgb',`${b.r},${b.g},${b.b}`);
set(el,'--ui-glow-alpha',String(clampF32(s.accentGlow,5,35)/100));
set(el,'--lib-cover-w',`${clampF32(s.libraryCoverWidth,120,420)}px`);
set(el,'--lib-cover-h',`${clampF32(s.libraryCoverHeight,120,420)}px`);
set(el,'--lib-card-min',`${Math.max(170,clampF32(s.libraryCoverWidth,120,420)+28)}px`);
set(el,'--fullscreen-img-w',`${clampF32(s.fullscreenWidth,40,96)}vw`);
set(el,'--fullscreen-img-h',`${clampF32(s.fullscreenHeight,40,96)}vh`);
});
// Corrige nodos generados después, especialmente TNFS/Biblioteca/Glifos.
document.querySelectorAll('.glyph-code-cell,.glyph-char-card,.glyph-hex-row').forEach(el=>{
set(el,'background','transparent');
set(el,'border-color',`rgba(${p.r},${p.g},${p.b},.34)`);
set(el,'box-shadow','none');
});
document.querySelectorAll('.glyph-code-cell .glyph-mini-pixels').forEach(el=>set(el,'display','none'));
document.querySelectorAll('.glyph-cell:not(.on),.glyph-preview-pixels i:not(.on),.glyph-fullscreen-pixels i:not(.on)').forEach(el=>{
set(el,'background','transparent');
set(el,'box-shadow','none');
});
document.querySelectorAll('.glyph-cell.on,.glyph-preview-pixels i.on,.glyph-fullscreen-pixels i.on').forEach(el=>{
set(el,'background',`rgba(${p.r},${p.g},${p.b},.86)`);
set(el,'border-color',`rgba(${a.r},${a.g},${a.b},.74)`);
set(el,'box-shadow','none');
});
document.querySelectorAll('.mock-lib-cover-wrap').forEach(el=>{
set(el,'width',`min(100%, ${clampF32(s.libraryCoverWidth,120,420)}px)`);
set(el,'height',`${clampF32(s.libraryCoverHeight,120,420)}px`);
set(el,'min-height',`${clampF32(s.libraryCoverHeight,120,420)}px`);
set(el,'border-color',`rgba(${p.r},${p.g},${p.b},.32)`);
});
}
(function patchThemeCleanupF43(){
if(window.__f43ThemeCleanup) return;
window.__f43ThemeCleanup=true;
const oldApply=window.applyVisualSettingsF38;
window.applyVisualSettingsF38=function(settings){
if(typeof oldApply==='function') oldApply(settings);
applyThemeCleanupF43(settings||visualSettingsF38);
};
const oldAudit=window.applyVisualSettingsF41;
if(typeof oldAudit==='function'){
window.applyVisualSettingsF41=function(settings, save=false){
oldAudit(settings, save);
applyThemeCleanupF43(settings||visualSettingsF38);
};
}
const run=()=>applyThemeCleanupF43(visualSettingsF38);
if(document.readyState==='loading') document.addEventListener('DOMContentLoaded',run); else run();
setTimeout(run,250);
setTimeout(run,900);
try{
const obs=new MutationObserver(()=>applyThemeCleanupF43(visualSettingsF38));
obs.observe(document.body,{childList:true,subtree:true,attributes:true,attributeFilter:['class','style']});
}catch(e){}
})();
/* ===== F44 runtime: evita flash azul al cambiar de pantalla ===== */
function applyThemeImmediateF44(){
try{
const s={...VISUAL_DEFAULTS_F38, ...(visualSettingsF38||{})};
if(typeof applyThemeVarsF41==='function') applyThemeVarsF41(s);
if(typeof applyThemeCleanupF43==='function') applyThemeCleanupF43(s);
if(typeof updatePreviewPanelF41==='function') updatePreviewPanelF41(s);
}catch(e){}
}
function routeKindF44(path){
path=String(path||location.pathname||'/').toLowerCase();
if(path==='/' || path==='') return 'home';
if(path.indexOf('/disk')===0 || path.indexOf('/editor/disks')===0 || path.indexOf('/editor/disk')===0) return 'disk';
if(path.indexOf('/atr')===0 || path.indexOf('/editor/web-atr')===0 || path.indexOf('/editor/webatr')===0) return 'atr';
if(path.indexOf('/library')===0 || path.indexOf('/editor/library')===0) return 'library';
if(path.indexOf('/cassette')===0 || path.indexOf('/cas')===0 || path.indexOf('/editor/cassette')===0 || path.indexOf('/editor/cas')===0) return 'cassette';
if(path.indexOf('/bt')===0 || path.indexOf('/editor/bt')===0) return 'bt';
if(path.indexOf('/tnfs')===0 || path.indexOf('/editor/tnfs')===0) return 'tnfs';
if(path.indexOf('/printer')===0 || path.indexOf('/editor/printer')===0) return 'printer';
if(path.indexOf('/glyphs')===0 || path.indexOf('/editor/glyphs')===0 || path.indexOf('/editor/glifos')===0) return 'glyphs';
if(path.indexOf('/settings')===0 || path.indexOf('/appearance')===0 || path.indexOf('/apariencia')===0 || path.indexOf('/editor/settings')===0 || path.indexOf('/editor/appearance')===0 || path.indexOf('/editor/apariencia')===0) return 'settings';
return '';
}
function loadRouteDataF44(kind){
try{
if(kind !== 'cassette' && typeof casStopLiveRefreshF49J === 'function') casStopLiveRefreshF49J();
if(kind==='atr'){
if(typeof atrForceCoversVisibleF15==='function') atrForceCoversVisibleF15('route-f44');
if(typeof loadAtrStatus==='function'){
loadAtrStatus({files:false, refreshFiles:false, force:true, forceCovers:true});
setTimeout(()=>loadAtrStatus({files:true, refreshFiles:false, forceSelectOptions:true, forceCovers:true}),180);
}
}else if(kind==='library'){
if(typeof loadAtrStatus==='function') loadAtrStatus({files:true, refreshFiles:false, force:true, forceLibrary:true});
}else if(kind==='cassette'){
if(typeof loadCasStatusF49C==='function') loadCasStatusF49C();
else if(typeof loadAtrStatus==='function') loadAtrStatus({files:false, refreshFiles:false, force:true});
if(typeof casStartLiveRefreshF49J==='function') casStartLiveRefreshF49J(true);
}else if(kind==='bt'){
if(typeof loadBtSioStatus==='function') loadBtSioStatus();
}else if(kind==='tnfs'){
if(typeof tnfsLoadServers==='function') tnfsLoadServers();
if(typeof tnfsBrowse==='function') setTimeout(()=>tnfsBrowse(null,{keepOffset:true}),120);
}else if(kind==='glyphs'){
if(typeof buildGlyphV27Layout==='function') buildGlyphV27Layout();
if(typeof glyphEnsureGridEditorReadyV38==='function') glyphEnsureGridEditorReadyV38();
if(typeof glyphLoadCurrent==='function') setTimeout(()=>glyphLoadCurrent().finally(()=>{ if(typeof glyphEnsureGridEditorReadyV38==='function') glyphEnsureGridEditorReadyV38(); }),160);
}else if(kind==='disk' || kind==='home' || kind==='printer'){
if(typeof loadStatus==='function') loadStatus();
}
}catch(e){}
}
function navigateInternalF44(path, replace){
const kind=routeKindF44(path);
if(!kind) return false;
document.documentElement.classList.add('theme-route-changing');
applyThemeImmediateF44();
try{
if(replace) history.replaceState({route:kind},'',path);
else history.pushState({route:kind},'',path);
}catch(e){ location.href=path; return true; }
if(typeof applyRouteLayout==='function') applyRouteLayout();
if(typeof applyF28UiTweaks==='function') applyF28UiTweaks();
applyThemeImmediateF44();
loadRouteDataF44(kind);
requestAnimationFrame(()=>{ applyThemeImmediateF44(); document.documentElement.classList.remove('theme-route-changing'); });
setTimeout(applyThemeImmediateF44,60);
setTimeout(applyThemeImmediateF44,250);
return true;
}
(function patchRouteThemeF44(){
if(window.__f44RouteTheme) return;
window.__f44RouteTheme=true;
const oldRoute=window.applyRouteLayout;
if(typeof oldRoute==='function'){
window.applyRouteLayout=function(){
document.documentElement.classList.add('theme-route-changing');
applyThemeImmediateF44();
const r=oldRoute.apply(this,arguments);
applyThemeImmediateF44();
requestAnimationFrame(()=>{ applyThemeImmediateF44(); document.documentElement.classList.remove('theme-route-changing'); });
return r;
};
}
document.addEventListener('click',function(ev){
const a=ev.target && ev.target.closest ? ev.target.closest('a[href]') : null;
if(!a || ev.defaultPrevented || ev.button!==0 || ev.metaKey || ev.ctrlKey || ev.shiftKey || ev.altKey || a.target) return;
let url;
try{ url=new URL(a.getAttribute('href'), location.origin); }catch(e){ return; }
if(url.origin!==location.origin) return;
const path=url.pathname+url.search+url.hash;
if(!routeKindF44(url.pathname)) return;
ev.preventDefault();
if(path===(location.pathname+location.search+location.hash)){
applyThemeImmediateF44();
if(typeof applyRouteLayout==='function') applyRouteLayout();
return;
}
navigateInternalF44(path,false);
},true);
window.addEventListener('popstate',()=>navigateInternalF44(location.pathname+location.search+location.hash,true));
const boot=()=>{ applyThemeImmediateF44(); setTimeout(applyThemeImmediateF44,80); setTimeout(applyThemeImmediateF44,400); };
if(document.readyState==='loading') document.addEventListener('DOMContentLoaded',boot); else boot();
})();
/* ===== F45 runtime: fuerza tema en botones con reglas antiguas específicas ===== */
function applySpecificButtonsThemeF45(settings){
try{
const s={...VISUAL_DEFAULTS_F38, ...(settings||visualSettingsF38||{})};
const p=hexToRgbF32(s.primaryColor), a=hexToRgbF32(s.accentColor);
const glow=clampF32(s.accentGlow,5,35)/100;
const primary=`linear-gradient(180deg, ${s.primaryColor}, ${s.accentColor})`;
const shadow=`0 12px 30px rgba(${p.r},${p.g},${p.b},${glow*.72})`;
const border=`rgba(${p.r},${p.g},${p.b},.82)`;
const targets=[
'#btnSave',
'body.route-disk #configCard #btnSave',
'body.route-disk .actions.main-actions #btnSave',
'body.route-library #btnAtrUpload',
'body.route-library label[for="atrFile"].file-button',
'body.route-library #btnUploadCollapse.upload-collapse-toggle'
];
document.querySelectorAll(targets.join(',')).forEach(el=>{
el.style.setProperty('background',primary,'important');
el.style.setProperty('border-color',border,'important');
el.style.setProperty('color','#fff','important');
el.style.setProperty('box-shadow',shadow,'important');
});
document.querySelectorAll('body.route-library .upload-drop-zone, body.route-disk #configCard, body.route-disk #diskDrivesCard, body.route-disk #drives .drive-item.disk-unit-row, body.route-disk #drives .disk-row-controls').forEach(el=>{
el.style.setProperty('border-color',`rgba(${p.r},${p.g},${p.b},.34)`,'important');
});
document.querySelectorAll('body.route-library #atrUploadProgressBar, body.route-library .upload-progress-bar').forEach(el=>{
el.style.setProperty('background',`linear-gradient(90deg, ${s.primaryColor}, ${s.accentColor})`,'important');
el.style.setProperty('box-shadow',`0 0 18px rgba(${p.r},${p.g},${p.b},${glow*.72})`,'important');
});
}catch(e){}
}
(function patchSpecificButtonsThemeF45(){
if(window.__f45SpecificButtonsTheme) return;
window.__f45SpecificButtonsTheme=true;
const oldImmediate=window.applyThemeImmediateF44;
if(typeof oldImmediate==='function'){
window.applyThemeImmediateF44=function(){
const r=oldImmediate.apply(this,arguments);
applySpecificButtonsThemeF45(visualSettingsF38);
return r;
};
}
const oldApply=window.applyVisualSettingsF38;
if(typeof oldApply==='function'){
window.applyVisualSettingsF38=function(settings){
const r=oldApply.apply(this,arguments);
applySpecificButtonsThemeF45(settings||visualSettingsF38);
return r;
};
}
const run=()=>applySpecificButtonsThemeF45(visualSettingsF38);
if(document.readyState==='loading') document.addEventListener('DOMContentLoaded',run); else run();
setTimeout(run,100); setTimeout(run,500); setTimeout(run,1200);
try{
const obs=new MutationObserver(run);
obs.observe(document.body,{childList:true,subtree:true,attributes:true,attributeFilter:['class','style']});
}catch(e){}
})();
/* =============================================================
F49Z21 - Biblioteca: total y lista robusta después de cargar/refrescar
============================================================= */
(function libraryListCountFixF49Z21(){
if (window.__libraryListCountFixF49Z21) return;
window.__libraryListCountFixF49Z21 = true;
function isLibraryRouteF49Z21(){
const p = String(location.pathname || '').toLowerCase();
return p.indexOf('/library') === 0 || p.indexOf('/editor/library') === 0 || p.indexOf('/upload') === 0;
}
function hasLibraryCardsF49Z21(){
const lib = document.getElementById('atrLibrary');
return !!(lib && lib.querySelector('.mock-lib-card,.atr-file-row'));
}
function typeFromNameF49Z21(name){
const m = String(name || '').match(/\.([^.]+)$/);
return (m ? m[1] : 'ATR').toUpperCase();
}
function buildFileObjF49Z21(name, previous){
const p = previous || {};
return {
name: String(name || p.name || ''),
type: String(p.type || typeFromNameF49Z21(name || p.name || '')),
fileSize: Number(p.fileSize || p.size || 0),
size: Number(p.size || p.fileSize || 0),
sectorSize: Number(p.sectorSize || 0),
totalSectors: Number(p.totalSectors || 0),
mountedMask: Number(p.mountedMask || 0),
casMounted: Number(p.casMounted || 0),
casPlaying: Number(p.casPlaying || 0),
protected: !!p.protected,
path: String(p.path || '')
};
}
function setCurrentLibraryFilesF49Z21(files){
files = Array.isArray(files) ? files.filter(f => f && f.name) : [];
try { atrLibraryFilesCache = files.slice(); } catch(e) {}
try { window.atrLibraryFilesCache = files.slice(); } catch(e) {}
try {
if (typeof lastAtrStatus !== 'undefined') {
lastAtrStatus = Object.assign({}, lastAtrStatus || {}, { files: files.slice() });
}
} catch(e) {}
try {
window.lastAtrStatus = Object.assign({}, window.lastAtrStatus || {}, { files: files.slice() });
} catch(e) {}
return files;
}
function currentFilesF49Z21(){
try { if (typeof atrLibraryCurrentFilesF27 === 'function') return atrLibraryCurrentFilesF27() || []; } catch(e) {}
try { if (Array.isArray(atrLibraryFilesCache)) return atrLibraryFilesCache; } catch(e) {}
try { if (Array.isArray(window.atrLibraryFilesCache)) return window.atrLibraryFilesCache; } catch(e) {}
return [];
}
function updateCountsF49Z21(files, status){
files = Array.isArray(files) ? files : [];
let n = files.length;
if (!n && status && status.filesCache && Number(status.filesCache.count || 0) > 0) n = Number(status.filesCache.count || 0);
if (!n && status && Number(status.filesCount || 0) > 0) n = Number(status.filesCount || 0);
const txt = n === 1 ? '1 archivo' : `${n} archivos`;
['atrLibraryCount','atrUploadLibraryCount','atrMountLibraryCount','homeLibraryCount'].forEach(id => {
const el = document.getElementById(id); if (el) el.textContent = txt;
});
const summary = document.getElementById('atrLibrarySummary');
if (summary && n && !files.length) summary.textContent = `${txt} · cargando lista...`;
return n;
}
async function fetchLibraryNamesF49Z21(){
const r = await fetch('/api/library?_=' + Date.now(), { cache:'no-store' });
if (!r.ok) throw new Error('HTTP ' + r.status);
const j = await r.json();
const arr = Array.isArray(j.files) ? j.files : [];
return arr.map(x => typeof x === 'string' ? x : (x && x.name)).filter(Boolean);
}
async function fallbackLoadLibraryF49Z21(force){
if (!isLibraryRouteF49Z21()) return false;
let files = currentFilesF49Z21();
if (Array.isArray(files) && files.length) {
updateCountsF49Z21(files, (typeof lastAtrStatus !== 'undefined' ? lastAtrStatus : window.lastAtrStatus));
if (typeof renderAtrLibrary === 'function') renderAtrLibrary(files, { force: !!force });
return true;
}
try {
const names = await fetchLibraryNamesF49Z21();
const prev = currentFilesF49Z21();
const byName = new Map((prev || []).map(f => [String(f.name || '').toLowerCase(), f]));
files = names.map(n => buildFileObjF49Z21(n, byName.get(String(n).toLowerCase())));
setCurrentLibraryFilesF49Z21(files);
updateCountsF49Z21(files, (typeof lastAtrStatus !== 'undefined' ? lastAtrStatus : window.lastAtrStatus));
if (typeof atrLibraryResetPageF49Z20 === 'function' && force) atrLibraryResetPageF49Z20();
if (typeof renderAtrLibrary === 'function') renderAtrLibrary(files, { force:true });
return files.length > 0;
} catch(e) {
console.warn('Fallback /api/library F49Z21', e);
return false;
}
}
const oldLoad = (typeof loadAtrStatus === 'function') ? loadAtrStatus : null;
if (oldLoad && !oldLoad.__f49z21Wrapped) {
const wrapped = async function(opts = {}) {
const r = await oldLoad.apply(this, arguments);
if (isLibraryRouteF49Z21()) {
const files = currentFilesF49Z21();
updateCountsF49Z21(files, r || (typeof lastAtrStatus !== 'undefined' ? lastAtrStatus : window.lastAtrStatus));
if ((opts.files || opts.forceLibrary || opts.refreshFiles || opts.force) && (!files.length || !hasLibraryCardsF49Z21())) {
await fallbackLoadLibraryF49Z21(!!(opts.force || opts.forceLibrary || opts.refreshFiles));
}
}
return r;
};
wrapped.__f49z21Wrapped = true;
loadAtrStatus = wrapped;
}
function bindRefreshF49Z21(){
const btn = document.getElementById('btnLibraryRefresh');
if (btn && !btn.dataset.f49z21) {
btn.dataset.f49z21 = '1';
btn.addEventListener('click', async (ev) => {
ev.preventDefault();
ev.stopImmediatePropagation();
try { window.atrLibraryTypeFilterF27 = 'all'; } catch(e) {}
try { if (typeof atrLibraryResetPageF49Z20 === 'function') atrLibraryResetPageF49Z20(); } catch(e) {}
if (typeof window.loadLibraryPageF49Z45 === 'function') await window.loadLibraryPageF49Z45({ refresh:true, force:true });
else if (typeof loadAtrStatus === 'function') await loadAtrStatus({ files:true, refreshFiles:true, force:true, forceLibrary:true });
else await fallbackLoadLibraryF49Z21(true);
}, true);
}
}
function kickF49Z21(){
bindRefreshF49Z21();
if (!isLibraryRouteF49Z21()) return;
const files = currentFilesF49Z21();
updateCountsF49Z21(files, (typeof lastAtrStatus !== 'undefined' ? lastAtrStatus : window.lastAtrStatus));
setTimeout(async () => {
if (typeof loadAtrStatus === 'function') await loadAtrStatus({ files:true, refreshFiles:false, force:true, forceLibrary:true });
await fallbackLoadLibraryF49Z21(false);
}, 120);
setTimeout(() => { if (!hasLibraryCardsF49Z21()) fallbackLoadLibraryF49Z21(true); }, 900);
}
if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', kickF49Z21); else kickF49Z21();
window.addEventListener('popstate', () => setTimeout(kickF49Z21, 80));
})();
/* =============================================================
F49Z24 - Optimización liviana Web/App
- Coalesce de lecturas de estado para evitar ráfagas duplicadas.
- Biblioteca con render de página más barato y debounce de búsqueda.
- Cola de carátulas limpia al cambiar de página/filtro para no cargar imágenes antiguas.
- content-visibility/contain para tarjetas pesadas.
============================================================= */
(function perfOptimizationsF49Z24(){
if (window.__perfOptimizationsF49Z24) return;
window.__perfOptimizationsF49Z24 = true;
const now = () => Date.now();
const idle = (fn, timeout = 700) => {
if (typeof requestIdleCallback === 'function') return requestIdleCallback(fn, { timeout });
return setTimeout(fn, Math.min(timeout, 220));
};
const path = () => String(location.pathname || '').toLowerCase();
const isLibraryRoute = () => path().indexOf('/library') === 0 || path().indexOf('/editor/library') === 0 || path().indexOf('/upload') === 0;
function coalesceAsync(name, getFn, setFn, keyFn, windowMsFn) {
try {
const original = getFn();
if (typeof original !== 'function' || original.__f49z24Coalesced) return;
let lastKey = '';
let lastAt = 0;
let lastPromise = null;
const wrapped = async function(...args) {
const opts = args[0] || {};
const key = keyFn ? keyFn(args) : JSON.stringify(args || []);
const ms = windowMsFn ? Number(windowMsFn(args, opts) || 0) : 450;
const t = now();
if (lastPromise && key === lastKey && (t - lastAt) < ms) return lastPromise;
lastKey = key;
lastAt = t;
lastPromise = Promise.resolve(original.apply(this, args)).finally(() => {
setTimeout(() => { if (lastPromise && key === lastKey) lastPromise = null; }, ms);
});
return lastPromise;
};
wrapped.__f49z24Coalesced = true;
setFn(wrapped);
} catch(e) { console.warn('F49Z24 coalesce ' + name, e); }
}
coalesceAsync('loadAtrStatus', () => loadAtrStatus, v => { loadAtrStatus = v; }, (args) => {
const o = args[0] || {};
return 'atr|' + [!!o.files, !!o.refreshFiles, !!o.forceLibrary, !!o.fast, !!o.visibility].join('|') + '|' + path();
}, (args, o) => o && o.refreshFiles ? 350 : (o && o.files ? 1100 : 650));
coalesceAsync('loadStatus', () => loadStatus, v => { loadStatus = v; }, () => 'status|' + path(), () => 700);
coalesceAsync('loadBtSioStatus', () => loadBtSioStatus, v => { loadBtSioStatus = v; }, () => 'bt|' + path(), () => 900);
function clearCoverWorkF49Z24() {
try { if (atrLazyCoverObserver) atrLazyCoverObserver.disconnect(); } catch(e) {}
try { if (Array.isArray(atrImageLoadQueue)) atrImageLoadQueue.length = 0; } catch(e) {}
try { if (Array.isArray(atrMiniCoverQueue)) atrMiniCoverQueue.length = Math.min(atrMiniCoverQueue.length, 1); } catch(e) {}
}
try {
const originalInitLazy = atrInitLazyCovers;
if (typeof originalInitLazy === 'function' && !originalInitLazy.__f49z24Wrapped) {
atrInitLazyCovers = function() {
if (document.hidden) return;
const lib = document.getElementById('atrLibrary');
if (isLibraryRoute() && lib) {
try { if (atrLazyCoverObserver) atrLazyCoverObserver.disconnect(); } catch(e) {}
const imgs = Array.from(lib.querySelectorAll("img.atr-lazy-cover[data-src]:not([data-loaded='1'])"));
if (!imgs.length) return;
const schedule = img => {
if (!img || !document.body.contains(img)) return;
idle(() => { if (document.body.contains(img) && typeof atrQueueImageLoad === 'function') atrQueueImageLoad(img); }, 900);
};
if ('IntersectionObserver' in window) {
atrLazyCoverObserver = new IntersectionObserver((entries) => {
entries.forEach(entry => {
if (entry.isIntersecting) {
atrLazyCoverObserver.unobserve(entry.target);
schedule(entry.target);
}
});
}, { root:null, rootMargin:'90px 0px', threshold:0.01 });
imgs.forEach(img => atrLazyCoverObserver.observe(img));
} else {
imgs.slice(0, 8).forEach((img, i) => setTimeout(() => schedule(img), i * 120));
}
return;
}
return originalInitLazy.apply(this, arguments);
};
atrInitLazyCovers.__f49z24Wrapped = true;
}
} catch(e) { console.warn('F49Z24 lazy covers', e); }
try {
const originalMini = atrEnsureMiniCoverFromImage;
if (typeof originalMini === 'function' && !originalMini.__f49z24Wrapped) {
atrEnsureMiniCoverFromImage = function(img, fileName) {
if (!img || document.hidden) return;
idle(() => {
if (!img || !document.body.contains(img)) return;
originalMini.call(this, img, fileName);
}, 2600);
};
atrEnsureMiniCoverFromImage.__f49z24Wrapped = true;
}
} catch(e) { console.warn('F49Z24 mini covers', e); }
try {
const originalRender = renderAtrLibrary;
if (typeof originalRender === 'function' && !originalRender.__f49z24Wrapped) {
let lastSig = '';
renderAtrLibrary = function(files, opts = {}) {
const lib = (typeof $ === 'function') ? $('atrLibrary') : document.getElementById('atrLibrary');
if (!lib) return;
files = Array.isArray(files) ? files : [];
if (typeof atrLibraryScaffoldF27 === 'function') atrLibraryScaffoldF27(files);
if (typeof atrLibraryUpdateChipsF27 === 'function') atrLibraryUpdateChipsF27(files);
let list = files.slice();
const q = String((typeof $ === 'function' && $('atrLibrarySearch')) ? $('atrLibrarySearch').value : '').trim().toLowerCase();
if (q) list = list.filter(f => String(f.name || '').toLowerCase().includes(q) || String(atrLibraryTypeF27(f) || '').toLowerCase().includes(q));
list = atrLibraryApplyTypeFilterF27(list);
if (!list.length && files.length && !q && String(window.atrLibraryTypeFilterF27 || 'all').toLowerCase() !== 'all') {
window.atrLibraryTypeFilterF27 = 'all';
list = files.slice();
if (typeof atrLibraryUpdateChipsF27 === 'function') atrLibraryUpdateChipsF27(files);
}
list = atrLibrarySortListF27(list);
const mode = atrLibraryViewMode === 'folders' ? 'folders' : 'flat';
const pageSize = Number(window.ATR_LIBRARY_PAGE_SIZE_F49Z20 || 20) || 20;
const total = list.length;
const pages = Math.max(1, Math.ceil(total / pageSize));
let page = Math.max(0, Math.min(Number(window.atrLibraryPageF49Z20 || 0), pages - 1));
window.atrLibraryPageF49Z20 = page;
const start = page * pageSize;
const end = Math.min(total, start + pageSize);
const pageList = list.slice(start, end);
const pageSig = pageList.map(f => [f.name || '', f.type || '', f.fileSize || f.size || 0, f.mountedMask || 0, f.casMounted || 0, f.casPlaying || 0].join(':')).join('|');
const sig = [files.length, total, q, mode, window.atrLibraryTypeFilterF27 || 'all', window.atrLibrarySortF27 || 'name', page, pageSig].join('§');
if (!opts.force && sig === lastSig) return;
lastSig = sig;
try { atrLibraryLastRenderSigF27 = sig; } catch(e) {}
clearCoverWorkF49Z24();
const summary = (typeof $ === 'function') ? $('atrLibrarySummary') : document.getElementById('atrLibrarySummary');
if (summary) summary.textContent = total > pageSize ? `${total} de ${files.length} archivos · página ${page + 1}/${pages}` : `${total} de ${files.length} archivos`;
lib.className = 'atr-help atr-library-list mock-lib-grid';
if (!total) {
if (typeof atrLibraryRenderPagerF49Z20 === 'function') atrLibraryRenderPagerF49Z20(0, 0, 1, 0, 0);
lib.innerHTML = `<div class="mock-lib-empty">${q ? 'Sin resultados para la búsqueda.' : 'Sin archivos disponibles en la biblioteca.'}</div>`;
return;
}
if (typeof atrLibraryRenderPagerF49Z20 === 'function') atrLibraryRenderPagerF49Z20(total, page, pages, start, end);
if (mode === 'flat') {
lib.innerHTML = pageList.map(atrLibraryCardHtmlF27).join('');
} else {
const groups = {};
pageList.forEach(f => {
const k = atrGroupKey(f.name);
if (!groups[k]) groups[k] = [];
groups[k].push(f);
});
const order = ['0-9', ...'ABCDEFGHIJKLMNOPQRSTUVWXYZ'.split(''), '#'];
lib.innerHTML = order.filter(k => groups[k] && groups[k].length).map(k => {
const items = groups[k];
return `<details class="mock-folder-group" open>`+
`<summary><span>${escapeHtml(k)}</span><span class="pill">${items.length}</span></summary>`+
`<div class="mock-folder-grid">${items.map(atrLibraryCardHtmlF27).join('')}</div>`+
`</details>`;
}).join('');
}
idle(() => { if (typeof atrInitLazyCovers === 'function') atrInitLazyCovers(); }, 500);
};
renderAtrLibrary.__f49z24Wrapped = true;
}
} catch(e) { console.warn('F49Z24 render library', e); }
function bindFastLibraryControls() {
const inp = document.getElementById('atrLibrarySearch');
if (inp && !inp.dataset.f49z24) {
inp.dataset.f49z24 = '1';
let timer = 0;
inp.addEventListener('input', ev => {
ev.stopImmediatePropagation();
clearTimeout(timer);
timer = setTimeout(() => {
if (typeof atrLibraryResetPageF49Z20 === 'function') atrLibraryResetPageF49Z20();
if (typeof renderAtrLibrary === 'function') renderAtrLibrary(atrLibraryCurrentFilesF27(), { force:true });
}, 170);
}, true);
}
const btn = document.getElementById('btnLibraryRefresh');
if (btn && !btn.dataset.f49z24) {
btn.dataset.f49z24 = '1';
btn.addEventListener('click', ev => {
ev.preventDefault();
ev.stopImmediatePropagation();
try { window.atrLibraryTypeFilterF27 = 'all'; } catch(e) {}
try { if (typeof atrLibraryResetPageF49Z20 === 'function') atrLibraryResetPageF49Z20(); } catch(e) {}
if (typeof window.loadLibraryPageF49Z45 === 'function') window.loadLibraryPageF49Z45({ refresh:true, force:true });
else if (typeof loadAtrStatus === 'function') loadAtrStatus({ files:true, refreshFiles:true, force:true, forceLibrary:true });
}, true);
}
}
function injectPerfCssF49Z24() {
if (document.getElementById('f49z24PerfCss')) return;
const st = document.createElement('style');
st.id = 'f49z24PerfCss';
st.textContent = `
body.route-library .mock-lib-card,
body.route-library .mock-folder-group,
body.route-library .atr-file-row{content-visibility:auto;contain-intrinsic-size:260px 360px;contain:layout paint style;}
body.route-library .mock-lib-grid{contain:layout style;}
.atr-lib-cover,.atr-cover-thumb{background:rgba(255,255,255,.035);}
@media (prefers-reduced-motion: reduce){*,*::before,*::after{animation-duration:.001ms!important;transition-duration:.001ms!important;scroll-behavior:auto!important;}}
`;
document.head.appendChild(st);
}
function boot() {
injectPerfCssF49Z24();
bindFastLibraryControls();
if (isLibraryRoute() && typeof renderAtrLibrary === 'function') {
idle(() => renderAtrLibrary(atrLibraryCurrentFilesF27(), { force:true }), 400);
}
}
if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot);
else boot();
window.addEventListener('popstate', () => setTimeout(boot, 80));
})();
/* =============================================================
F49Z25 - Estado ATR/CAS con timeout, single-flight y backoff
- Evita cadenas de wrappers acumuladas sobre loadAtrStatus.
- En ruta Cassette no dispara /api/atr/status?files=1.
- /api/cas/status queda con una sola petición activa, timeout corto y reintento suave.
- Si el ESP32/SD está ocupado, conserva la última UI válida y no satura la red.
============================================================= */
(function statusSingleFlightF49Z25(){
if (window.__statusSingleFlightF49Z25) return;
window.__statusSingleFlightF49Z25 = true;
const NET_F49Z25 = {
atr: { p:null, key:'', next:0, fail:0, last:null, lastErrorAt:0 },
cas: { p:null, key:'', next:0, fail:0, last:null, lastErrorAt:0 }
};
const nowF49Z25 = () => Date.now();
function kindF49Z25(){
try { return (typeof routeKindF44 === 'function') ? routeKindF44(location.pathname) : ''; }
catch(e) { return ''; }
}
function isCassetteRouteF49Z25(){ return kindF49Z25() === 'cassette' || /\/cas(sette)?/i.test(location.pathname || ''); }
function isLibraryRouteF49Z25(){ const k = kindF49Z25(); return k === 'library' || k === 'upload'; }
function backoffMsF49Z25(fail){ return Math.min(18000, 1800 + Math.max(0, fail - 1) * 3200); }
function quietWarnF49Z25(bucket, msg, err){
if (err && (err.silent || err.name === 'AbortError' || err.name === 'TimeoutError' || /abort/i.test(String(err.message || err)))) return;
const st = NET_F49Z25[bucket];
const t = nowF49Z25();
if (!st || (t - (st.lastErrorAt || 0)) > 5000) {
if (st) st.lastErrorAt = t;
console.warn(msg, err && (err.message || err));
}
}
async function fetchJsonTimeoutF49Z25(url, timeoutMs){
const ctrl = (typeof AbortController !== 'undefined') ? new AbortController() : null;
const timer = ctrl ? setTimeout(() => { try { ctrl.abort('F49Z45_TIMEOUT'); } catch(e){ try { ctrl.abort(); } catch(_){} } }, Math.max(650, Number(timeoutMs || 1600))) : 0;
try {
const r = await fetch(url, { cache:'no-store', signal: ctrl ? ctrl.signal : undefined });
if (!r.ok) throw new Error('HTTP ' + r.status);
return await r.json();
} catch(e) {
if (ctrl && ctrl.signal && ctrl.signal.aborted) {
const er = new Error('timeout');
er.name = 'TimeoutError';
er.silent = true;
throw er;
}
throw e;
} finally {
if (timer) clearTimeout(timer);
}
}
async function loadCasStatusCoreF49Z25(opts = {}) {
const st = NET_F49Z25.cas;
const t = nowF49Z25();
const force = !!(opts.force || opts.refresh || opts.manual);
if (!force && st.next && t < st.next) return st.last || window.casLastStatusF49J || {};
if (st.p) return st.p;
st.p = (async () => {
try {
const casUrl = '/api/cas/status' + (opts.full || opts.analyze ? '' : '?lite=1');
const a = await fetchJsonTimeoutF49Z25(casUrl, Number(opts.timeout || 1200));
st.fail = 0;
st.next = 0;
st.last = a;
window.casLastStatusF49J = a;
if (typeof updateCasManualPanelF49 === 'function') updateCasManualPanelF49(a);
return a;
} catch(e) {
st.fail++;
st.next = nowF49Z25() + backoffMsF49Z25(st.fail);
quietWarnF49Z25('cas', 'CAS status ocupado/timeout F49Z25', e);
return st.last || window.casLastStatusF49J || {};
} finally {
st.p = null;
}
})();
return st.p;
}
async function loadAtrStatusCoreF49Z25(opts = {}) {
opts = opts || {};
const st = NET_F49Z25.atr;
const route = kindF49Z25();
if (isCassetteRouteF49Z25() && !opts.allowAtrOnCassette && !opts.forceLibrary && !opts.refreshFiles) {
return st.last || (typeof lastAtrStatus !== 'undefined' ? lastAtrStatus : {}) || {};
}
const pth = String(location.pathname || '').toLowerCase();
const routeNeedsFiles = isLibraryRouteF49Z25() || pth.indexOf('/upload') === 0 || pth.indexOf('/atr') === 0 || pth.indexOf('/editor/atr') === 0;
const needFiles = (typeof opts.files === 'boolean') ? !!opts.files : routeNeedsFiles;
const refreshFiles = !!opts.refreshFiles;
const fastStatus = !!opts.fast || (!needFiles && route !== 'library');
const url = '/api/atr/status?files=' + (needFiles ? '1' : '0') + (refreshFiles ? '&refresh=1' : '') + (fastStatus ? '&fast=1' : '');
const key = url + '|route=' + route;
const t = nowF49Z25();
const force = !!(opts.force || opts.forceLibrary || opts.forceCovers || opts.forceSelectOptions || opts.renderLibrary || refreshFiles);
if (!force && st.next && t < st.next) return st.last || (typeof lastAtrStatus !== 'undefined' ? lastAtrStatus : {}) || {};
if (st.p && st.key === key) return st.p;
st.key = key;
st.p = (async () => {
try {
const timeout = needFiles ? Number(opts.timeout || 3200) : Number(opts.timeout || 1450);
const a = await fetchJsonTimeoutF49Z25(url, timeout);
st.fail = 0;
st.next = 0;
if (!Array.isArray(a.files)) {
try { a.files = Array.isArray(atrLibraryFilesCache) ? atrLibraryFilesCache.slice() : []; } catch(e) { a.files = []; }
} else {
try { atrLibraryFilesCache = a.files.slice(); atrLibraryFilesCacheSig = atrLibraryFilesSignature(atrLibraryFilesCache); } catch(e) {}
}
st.last = a;
try { lastAtrStatus = a; } catch(e) { window.lastAtrStatus = a; }
try { if (typeof atrUpdatePriorityUiV30 === 'function') atrUpdatePriorityUiV30(a); } catch(e) {}
const fsUsedTxt = `${humanBytes(a.fsUsed || 0)} / ${humanBytes(a.fsTotal || 0)}`;
const fsFreeTxt = humanBytes(a.fsFree || 0);
['atrFsState','atrUploadFsState'].forEach(id => { const el = typeof $ === 'function' ? $(id) : document.getElementById(id); if (el) el.textContent = fsUsedTxt; });
['atrFsFree','atrUploadFsFree','atrMountFsFree'].forEach(id => { const el = typeof $ === 'function' ? $(id) : document.getElementById(id); if (el) el.textContent = fsFreeTxt; });
['atrEnabledState','atrMountEnabledState'].forEach(id => { const el = typeof $ === 'function' ? $(id) : document.getElementById(id); if (el) el.textContent = a.enabled ? 'Sí' : 'No'; });
const readsEl = typeof $ === 'function' ? $('atrReads') : document.getElementById('atrReads');
if (readsEl) readsEl.textContent = a.reads ?? '—';
const files = (a.files || []).slice().sort((x,y) => String(x.name || '').localeCompare(String(y.name || ''), 'es', { sensitivity:'base', numeric:true }));
const slots = a.slots || [];
try {
const visibleMask = visibleUnitMaskFromStatus(a);
updateVisibleUnitCards(visibleMask, (typeof lastDriveVisibility !== 'undefined' && lastDriveVisibility) ? lastDriveVisibility.units : null);
if (opts.visibility !== false && needFiles && typeof loadDriveVisibilityStatus === 'function') loadDriveVisibilityStatus();
} catch(e) {}
let filesSig = '';
try { filesSig = atrLibraryFilesSignature(files); } catch(e) {}
const shouldRefreshSelectOptions = !!opts.forceSelectOptions || needFiles || filesSig !== (typeof atrSelectOptionsLastSig !== 'undefined' ? atrSelectOptionsLastSig : '');
try { if (shouldRefreshSelectOptions) atrSelectOptionsLastSig = filesSig; } catch(e) {}
try {
for (let i=1;i<=UI_MAX_UNITS;i++) {
const sl = slots[i-1] || {};
const sel = $(`atrSel${i}`);
if (sel && shouldRefreshSelectOptions) sel.innerHTML = atrFileOptions(files, sl.name || '');
if (sel && !shouldRefreshSelectOptions && sel.value !== (sl.name || '')) sel.value = sl.name || '';
if ($(`atrEn${i}`)) $(`atrEn${i}`).checked = !!sl.enabled;
if ($(`atrForce${i}`)) $(`atrForce${i}`).checked = !!sl.forced;
const name = sl.name || '—';
const meta = sl.present ? `${humanBytes(sl.fileSize || 0)}, ${sl.sectorSize || '?'}B, ${sl.totalSectors || '?'} sectores` : 'sin ATR válido';
if ($(`atrSlot${i}`)) $(`atrSlot${i}`).textContent = sl.present ? 'ATR OK' : (sl.name ? 'ATR inválido' : 'Sin ATR');
if ($(`atrSlotName${i}`)) $(`atrSlotName${i}`).textContent = name;
if ($(`atrSlotMeta${i}`)) $(`atrSlotMeta${i}`).textContent = meta;
const coverName = sl.name || '';
try { if (atrSlotCoverNameLast[i] !== coverName || opts.forceCovers) { atrSlotCoverNameLast[i] = coverName; updateAtrCover(i, coverName, !!(coverName || sl.present)); } } catch(e) {}
const routeEl = $(`atrRoute${i}`);
if (routeEl) { routeEl.textContent = sl.enabled ? 'WEB-ATR ON' : 'WEB-ATR OFF'; routeEl.className = sl.enabled ? 'pill ok' : 'pill bad'; }
const fb = $(`atrFallback${i}`);
if (fb) { fb.textContent = sl.forced ? 'sin fallback' : 'fallback físico'; fb.className = sl.forced ? 'pill warn' : 'pill'; }
}
} catch(e) {}
const countText = files.length === 1 ? '1 archivo' : `${files.length} archivos`;
['atrLibraryCount','atrUploadLibraryCount','atrMountLibraryCount'].forEach(id => { const el = typeof $ === 'function' ? $(id) : document.getElementById(id); if (el) el.textContent = countText; });
const homeCnt = typeof $ === 'function' ? $('homeLibraryCount') : document.getElementById('homeLibraryCount');
if (homeCnt) homeCnt.textContent = countText;
const libSummary = typeof $ === 'function' ? $('atrLibrarySummary') : document.getElementById('atrLibrarySummary');
if (libSummary) libSummary.textContent = files.length ? `${files.length} archivo(s)` : 'Sin archivos';
const viewSel = typeof $ === 'function' ? $('atrLibraryView') : document.getElementById('atrLibraryView');
if (viewSel) viewSel.value = atrLibraryViewMode === 'folders' ? 'folders' : 'flat';
if (needFiles || opts.renderLibrary || opts.forceLibrary) {
if (typeof renderAtrLibrary === 'function') renderAtrLibrary(files, { force: !!(opts.forceLibrary || opts.force || opts.refreshFiles) });
}
return a;
} catch(e) {
st.fail++;
st.next = nowF49Z25() + backoffMsF49Z25(st.fail);
quietWarnF49Z25('atr', 'ATR status ocupado/timeout F49Z25', e);
const cached = st.last || (typeof lastAtrStatus !== 'undefined' ? lastAtrStatus : null);
if (cached) return cached;
const el = typeof $ === 'function' ? $('atrFsState') : document.getElementById('atrFsState');
if (el) el.textContent = 'ESP32 ocupado';
return {};
} finally {
st.p = null;
}
})();
return st.p;
}
try { window.__loadAtrStatusPreviousF49Z25 = (typeof loadAtrStatus === 'function') ? loadAtrStatus : null; } catch(e) {}
loadAtrStatus = loadAtrStatusCoreF49Z25;
window.loadAtrStatus = loadAtrStatusCoreF49Z25;
try { window.__loadCasStatusPreviousF49Z25 = (typeof loadCasStatusF49C === 'function') ? loadCasStatusF49C : null; } catch(e) {}
loadCasStatusF49C = loadCasStatusCoreF49Z25;
window.loadCasStatusF49C = loadCasStatusCoreF49Z25;
const routeLoaderF49Z25 = function(kind){
try {
const currentKind = kindF49Z25();
kind = currentKind || kind || '';
if (kind !== 'cassette' && typeof casStopLiveRefreshF49J === 'function') casStopLiveRefreshF49J();
if (kind === 'cassette') {
loadCasStatusCoreF49Z25({ force:false });
if (typeof casStartLiveRefreshF49J === 'function') casStartLiveRefreshF49J(false);
} else if (kind === 'library') {
if (typeof window.loadLibraryPageF49Z45 === 'function') window.loadLibraryPageF49Z45({ force:true });
else loadAtrStatusCoreF49Z25({ files:true, refreshFiles:false, force:true, forceLibrary:true });
} else if (kind === 'atr') {
try { if (typeof atrForceCoversVisibleF15 === 'function') atrForceCoversVisibleF15('route-f49z25'); } catch(e) {}
loadAtrStatusCoreF49Z25({ files:false, refreshFiles:false, force:true, forceCovers:true });
} else if (kind === 'bt') {
if (typeof loadBtSioStatus === 'function') loadBtSioStatus();
} else if (kind === 'tnfs') {
if (typeof tnfsLoadServers === 'function') tnfsLoadServers();
if (typeof tnfsBrowse === 'function') setTimeout(() => tnfsBrowse(null, { keepOffset:true }), 160);
} else if (kind === 'glyphs') {
if (typeof buildGlyphV27Layout === 'function') buildGlyphV27Layout();
if (typeof glyphEnsureGridEditorReadyV38 === 'function') glyphEnsureGridEditorReadyV38();
if (typeof glyphLoadCurrent === 'function') setTimeout(() => glyphLoadCurrent().finally(() => { if (typeof glyphEnsureGridEditorReadyV38 === 'function') glyphEnsureGridEditorReadyV38(); }), 180);
} else if (kind === 'disk' || kind === 'home' || kind === 'printer') {
if (typeof loadStatus === 'function') loadStatus();
}
} catch(e) { console.warn('route loader F49Z25', e); }
};
try { loadRouteDataF44 = routeLoaderF49Z25; window.loadRouteDataF44 = routeLoaderF49Z25; } catch(e) {}
try { casLiveDelayF49J = function(playing){ return playing ? 2200 : 6500; }; window.casLiveDelayF49J = casLiveDelayF49J; } catch(e) {}
try {
casStartLiveRefreshF49J = function(force){
if (typeof casIsRouteF49J === 'function' && !casIsRouteF49J()) return;
const st = window.casLiveRefreshF49J || (window.casLiveRefreshF49J = { timer:null, busy:false, running:false });
if (force && !st.busy) loadCasStatusCoreF49Z25({ force:false });
if (typeof casScheduleLiveRefreshF49J === 'function') casScheduleLiveRefreshF49J(force ? 1600 : casLiveDelayF49J(false));
};
window.casStartLiveRefreshF49J = casStartLiveRefreshF49J;
} catch(e) {}
})();
/* =============================================================
F31 - Biblioteca: puente legacy neutralizado
- Se eliminaron las capas antiguas F49Z45/F49Z47/F49Z48/F49Z50 que
  seguían ejecutando loadLibraryPageF47() y generaban F49Z47_TIMEOUT.
- Cualquier llamada legacy a loadLibraryPageF49Z45/loadLibraryPageF49Z47
  delega en el controlador vigente F22/F30.
============================================================= */
(function(){
  if(window.__libraryLegacyNeutralizerF31)return; window.__libraryLegacyNeutralizerF31=1;
  function delegate(opts){
    try{
      if(typeof window.loadLibraryPageF22==='function') return window.loadLibraryPageF22(opts||{});
      if(typeof window.loadLibraryPageF21==='function') return window.loadLibraryPageF21(opts||{});
    }catch(e){try{console.warn('F31 legacy library delegate', e&&e.message?e.message:e)}catch(_){}}
    return Promise.resolve(null);
  }
  window.loadLibraryPageF49Z45=delegate;
  window.loadLibraryPageF49Z47=delegate;
  window.libraryStateF49Z47=window.libraryStateF49Z47||{files:[],lastPayload:null,page:0,pageSize:24,total:0,pages:1,busy:false};
  try{console.info('F31: capas legacy Biblioteca F49Z45/F49Z47/F49Z48/F49Z50 neutralizadas.');}catch(e){}
})();

/* =============================================================
F49Z51 - Router final de navegacion
- Corrige JS roto en filtro por tipo y estabiliza clicks de iconos/menu.
- Fuerza route-* y visibilidad de bloques por pantalla.
============================================================= */
(function navFinalRouterF49Z51(){
if(window.__navFinalRouterF49Z51) return;
window.__navFinalRouterF49Z51=true;
const ROUTES=['home','disk','atr','library','cassette','bt','tnfs','printer','glyphs','settings'];
const NAV={home:'navHome',disk:'navDisk',atr:'navAtr',library:'navLibrary',cassette:'navCassette',bt:'navBt',tnfs:'navTnfs',printer:'navPrinter',glyphs:'navGlyphs',settings:'navSettings'};
const META={home:['Inicio','Interfaz web de control para Atari SIO','⌂'],disk:['Disqueteras','Administra unidades SIO, perfiles y parámetros de comunicación','▣'],atr:['WEB-ATR','Montaje rápido de imágenes ATR y XEX por unidad','◎'],library:['Biblioteca','Archivos ATR/XEX/CAS en la SD/Flash con carátulas y montaje rápido','▤'],cassette:['Cassette C:','Control manual para archivos .CAS preparados desde Biblioteca','▧'],bt:['Bluetooth SIO2PC','Pasarela compatible con RespeQt/AspeQt','◌'],tnfs:['Servidores TNFS','Explora servidores remotos al estilo FujiNet','◎'],printer:['Servicio de Impresión','Imprime desde Atari SIO usando IPP, Gateway o spool virtual','▥'],glyphs:['Editor de Glifos','Crea y edita caracteres personalizados para tu Atari SIO','▦'],settings:['Apariencia','Personaliza colores, logo, letra y tamaño de carátulas','◈']};
const BLOCKS=['visualSettingsPageBlock','btSio2pcPageBlock','tnfsPageBlock','atrUploadPageBlock','atrLibraryPageBlock','webAtrPageBlock','cassettePageBlock','diskDrivesCard','configCard','diskConfigBlock','printerConfigBlock','glyphImportExportBlock','glyphsPageBlock'];
function el(id){return document.getElementById(id)}
function disp(id,v){const e=el(id); if(e) e.style.setProperty('display',v,'important');}
function show(id,v){disp(id,v||'block')}
function hide(id){disp(id,'none')}
function kind(path){
 path=String(path||location.pathname||'/').toLowerCase();
 if(path==='/'||path==='') return 'home';
 if(path.indexOf('/editor/disks')===0||path.indexOf('/editor/disk')===0||path.indexOf('/disk')===0) return 'disk';
 if(path.indexOf('/atr')===0||path.indexOf('/editor/web-atr')===0||path.indexOf('/editor/webatr')===0) return 'atr';
 if(path.indexOf('/library')===0||path.indexOf('/upload')===0||path.indexOf('/editor/library')===0) return 'library';
 if(path.indexOf('/cassette')===0||path.indexOf('/cas')===0||path.indexOf('/editor/cassette')===0||path.indexOf('/editor/cas')===0) return 'cassette';
 if(path.indexOf('/bt')===0||path.indexOf('/editor/bt')===0) return 'bt';
 if(path.indexOf('/tnfs')===0||path.indexOf('/editor/tnfs')===0) return 'tnfs';
 if(path.indexOf('/printer')===0||path.indexOf('/editor/printer')===0) return 'printer';
 if(path.indexOf('/glyphs')===0||path.indexOf('/editor/glyphs')===0||path.indexOf('/editor/glifos')===0) return 'glyphs';
 if(path.indexOf('/settings')===0||path.indexOf('/appearance')===0||path.indexOf('/apariencia')===0||path.indexOf('/editor/settings')===0||path.indexOf('/editor/appearance')===0||path.indexOf('/editor/apariencia')===0) return 'settings';
 return '';
}
function setHero(k){const m=META[k]||META.home;const t=document.querySelector('.page-title'),s=document.querySelector('.page-subtitle'),i=el('routeIcon');if(t)t.textContent=m[0];if(s)s.textContent=m[1];if(i)i.textContent=m[2];Object.keys(NAV).forEach(r=>{const a=el(NAV[r]);if(a)a.classList.toggle('active',r===k);});}
function load(k){try{if(k!=='cassette'&&typeof casStopLiveRefreshF49J==='function')casStopLiveRefreshF49J();}catch(e){}try{if(k==='library'){if(typeof window.loadLibraryPageF49Z45==='function')window.loadLibraryPageF49Z45({force:true});else if(typeof window.loadLibraryPageF49Z47==='function')window.loadLibraryPageF49Z47({force:true});}else if(k==='cassette'){if(typeof loadCasStatusF49C==='function')loadCasStatusF49C({force:false});if(typeof casStartLiveRefreshF49J==='function')casStartLiveRefreshF49J(false);}else if(k==='atr'){if(typeof atrForceCoversVisibleF15==='function')atrForceCoversVisibleF15('route-f49z51');if(typeof loadAtrStatus==='function')loadAtrStatus({files:true,refreshFiles:false,force:true,forceCovers:true,forceSelectOptions:true});}else if(k==='bt'){if(typeof loadBtSioStatus==='function')loadBtSioStatus();}else if(k==='tnfs'){if(typeof tnfsLoadServers==='function')tnfsLoadServers();if(typeof tnfsBrowse==='function')setTimeout(()=>tnfsBrowse(null,{keepOffset:true}),120);}else if(k==='glyphs'){if(typeof buildGlyphV27Layout==='function')buildGlyphV27Layout();if(typeof glyphEnsureGridEditorReadyV38==='function')glyphEnsureGridEditorReadyV38();if(typeof glyphLoadCurrent==='function')setTimeout(()=>glyphLoadCurrent().finally(()=>{try{glyphEnsureGridEditorReadyV38()}catch(e){}}),120);}else if(k==='disk'||k==='home'||k==='printer'){if(typeof loadStatus==='function')loadStatus();}}catch(e){try{console.warn('route load F49Z51',e&&e.message?e.message:e)}catch(_){}}}
function apply(k,doLoad){k=k||kind(location.pathname)||'home';document.body.classList.remove(...ROUTES.map(r=>'route-'+r));document.body.classList.add('route-'+k);setHero(k);BLOCKS.forEach(hide);document.querySelectorAll('.home-panel').forEach(p=>p.style.setProperty('display',k==='home'?'grid':'none','important'));const grid=document.querySelector('.app > .grid');if(grid)grid.style.setProperty('display',k==='home'?'none':'grid','important');if(k==='settings')show('visualSettingsPageBlock');else if(k==='bt')show('btSio2pcPageBlock');else if(k==='tnfs')show('tnfsPageBlock');else if(k==='library'){show('atrUploadPageBlock');show('atrLibraryPageBlock');}else if(k==='atr')show('webAtrPageBlock');else if(k==='cassette')show('cassettePageBlock');else if(k==='disk'){show('diskDrivesCard');show('configCard');show('diskConfigBlock');}else if(k==='printer'){show('configCard');show('printerConfigBlock');}else if(k==='glyphs'){show('configCard');show('printerConfigBlock');show('glyphImportExportBlock');show('glyphsPageBlock');}try{if(typeof applyF28UiTweaks==='function')applyF28UiTweaks();}catch(e){}try{if(typeof applyThemeImmediateF44==='function')applyThemeImmediateF44();}catch(e){}if(doLoad!==false)load(k);return k;}
function go(path,replace){const k=kind(path);if(!k)return false;const target=new URL(path,location.origin);const targetPath=target.pathname+target.search+target.hash;try{if(targetPath!==(location.pathname+location.search+location.hash)){if(replace)history.replaceState({route:k},'',targetPath);else history.pushState({route:k},'',targetPath);}}catch(e){location.href=targetPath;return true;}apply(k,true);return true;}
window.routeKindF49Z51=kind;window.applyRouteLayoutF49Z51=function(){return apply(kind(location.pathname),true)};window.navigateInternalF49Z51=go;try{window.navigateInternalF44=function(path,replace){return go(path,replace)}}catch(e){}try{window.applyRouteLayout=function(){return apply(kind(location.pathname),false)}}catch(e){}
document.addEventListener('click',function(ev){const a=ev.target&&ev.target.closest?ev.target.closest('a[href]'):null;if(!a||ev.button!==0||ev.metaKey||ev.ctrlKey||ev.shiftKey||ev.altKey||a.target)return;let u;try{u=new URL(a.getAttribute('href'),location.origin)}catch(e){return}if(u.origin!==location.origin)return;if(!kind(u.pathname))return;ev.preventDefault();if(ev.stopImmediatePropagation)ev.stopImmediatePropagation();else ev.stopPropagation();go(u.pathname+u.search+u.hash,false);},true);
window.addEventListener('popstate',()=>apply(kind(location.pathname),true));
const boot=()=>{apply(kind(location.pathname)||'home',true);setTimeout(()=>apply(kind(location.pathname)||'home',false),80)};if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',boot);else boot();
})();




/* F49Z56 - Preparar C sin refrescar Biblioteca + glifos por code/id */
(function(){
 if(window.__f49z56)return;window.__f49z56=1;
 const byId=id=>document.getElementById(id), now=()=>Date.now();
 const enc=v=>encodeURIComponent(String(v||''));
 const dec=v=>{try{return decodeURIComponent(String(v||''));}catch(e){return String(v||'');}};
 const norm=v=>String(v||'').toLowerCase();
 const isLib=()=>{try{return !!(document.body&&document.body.classList&&document.body.classList.contains('route-library'))||(location.pathname||'').toLowerCase().indexOf('/library')===0;}catch(e){return false;}};
 let protectUntil=0,snapHtml='',snapPager='',snapSummary='',snapCount=0;
 function protectedNow(){return isLib()&&now()<protectUntil;}
 function setProtect(ms){protectUntil=now()+Math.min(2200,Math.max(700,Number(ms||1200)));window.__libraryFreezeUntilF49Z56=protectUntil;}
 function isManual(args){try{const o=args&&args[0];return !!(o&&(o.refresh||o.manualRefresh||o.fromRefreshButton||o.allowRefresh||o.force||o.forceLibrary||o.refreshFiles));}catch(e){return false;}}
 function cardCount(){try{return document.querySelectorAll('#atrLibrary .mock-lib-card').length;}catch(e){return 0;}}
 function snap(){if(!isLib())return;const l=byId('atrLibrary'),p=byId('atrLibraryPager'),s=byId('atrLibrarySummary');if(l){snapHtml=l.innerHTML;snapCount=cardCount();}if(p)snapPager=p.innerHTML;if(s)snapSummary=s.textContent||'';}
 function restoreIfLost(){if(!isLib())return;const l=byId('atrLibrary'),p=byId('atrLibraryPager'),s=byId('atrLibrarySummary');if(l&&snapHtml){const c=cardCount();const txt=(l.textContent||'').toLowerCase();if((snapCount&&c<snapCount)||txt.indexOf('sin archivos')>=0||txt.indexOf('error biblioteca')>=0||txt.indexOf('cargando biblioteca')>=0){l.innerHTML=snapHtml;}}if(p&&snapPager&&!p.innerHTML)p.innerHTML=snapPager;if(s&&snapSummary&&(s.textContent||'').toLowerCase().indexOf('cargando')>=0)s.textContent=snapSummary;}
 function arrays(fn){const apply=a=>{if(Array.isArray(a))a.forEach(fn)};try{if(window.libraryStateF49Z47){apply(window.libraryStateF49Z47.files);if(window.libraryStateF49Z47.lastPayload)apply(window.libraryStateF49Z47.lastPayload.files);}}catch(e){}try{apply(window.atrLibraryFilesCache)}catch(e){}try{if(typeof atrLibraryFilesCache!=='undefined')apply(atrLibraryFilesCache)}catch(e){}try{if(window.lastAtrStatus)apply(window.lastAtrStatus.files)}catch(e){}try{if(typeof lastAtrStatus!=='undefined'&&lastAtrStatus)apply(lastAtrStatus.files)}catch(e){}}
 function markCas(name,mounted){const t=norm(name);arrays(f=>{if(!f)return;const typ=String(f.type||f.ext||String(f.name||'').split('.').pop()||'').replace(/^\./,'').toUpperCase();if(typ==='CAS'){f.casMounted=mounted&&(norm(f.name)===t)?1:0;f.casPlaying=0;}});try{if(window.libraryStateF49Z47&&window.libraryStateF49Z47.lastPayload){window.libraryStateF49Z47.lastPayload.casMounted=mounted?1:0;window.libraryStateF49Z47.lastPayload.casName=mounted?name:'';}}catch(e){}try{if(window.lastAtrStatus){window.lastAtrStatus.casMounted=mounted?1:0;window.lastAtrStatus.casPlaying=0;window.lastAtrStatus.casName=mounted?name:'';}}catch(e){}try{if(typeof lastAtrStatus!=='undefined'&&lastAtrStatus){lastAtrStatus.casMounted=mounted?1:0;lastAtrStatus.casPlaying=0;lastAtrStatus.casName=mounted?name:'';}}catch(e){}}
 function safeFileAttr(s){return String(s||'').replace(/&/g,'&amp;').replace(/"/g,'&quot;').replace(/</g,'&lt;');}
 function menuFor(n,mounted){try{return typeof atrLibraryMenuHtmlF27==='function'?atrLibraryMenuHtmlF27({name:n,type:'CAS',casMounted:mounted?1:0},enc(n),false):'';}catch(e){return '';}}
 function updateCards(name,mounted){if(!isLib())return;const t=norm(name);document.querySelectorAll('.mock-lib-card[data-file-type="CAS"]').forEach(card=>{const n=card.getAttribute('data-file-name')||'';const units=card.querySelector('.mock-lib-units')||card.querySelector('.mock-card-actions');if(!units)return;const hit=mounted&&norm(n)===t;if(hit){card.classList.add('mounted');units.innerHTML='<span class="mounted-badge mock-mounted-badge">Preparado: C:</span><a class="mock-unit-btn" href="/cassette" title="Ver control cassette">Ver cassette</a>'+menuFor(n,true);}else if(!mounted||card.classList.contains('mounted')){card.classList.remove('mounted');units.innerHTML='<button type="button" class="mock-unit-btn" data-cas-prepare="1" data-file="'+safeFileAttr(n)+'" title="Preparar cassette C:">Preparar C:</button>'+menuFor(n,false);}});}
 async function mountCasStable(arg,ev){if(ev){try{ev.preventDefault();ev.stopPropagation();ev.stopImmediatePropagation&&ev.stopImmediatePropagation();}catch(e){}}const name=dec(arg||'');snap();setProtect(15000);try{if(typeof setStatus==='function')setStatus('Preparando C: '+name+'...',true);const r=await fetch('/cas/mount?file='+arg+'&fast=1',{cache:'no-store'});const txt=await r.text();let j=null;try{j=txt?JSON.parse(txt):null;}catch(e){}if(!r.ok||(j&&j.ok===false))throw new Error((j&&j.error)?j.error:(txt||('HTTP '+r.status)));markCas(name,true);restoreIfLost();updateCards(name,true);try{if(typeof updateCasManualPanelF49==='function')updateCasManualPanelF49(Object.assign({casMounted:1,mounted:1,casPlaying:0,playing:0,casName:name,name:name},j||{}));}catch(e){}try{window.casLastStatusF49J=Object.assign(window.casLastStatusF49J||{},{casMounted:1,mounted:1,casPlaying:0,playing:0,casName:name,name:name});}catch(e){}if(typeof setStatus==='function')setStatus('C: preparado: '+name+'. Biblioteca conservada.',true);[150,650,1800,4200].forEach(ms=>setTimeout(()=>{restoreIfLost();updateCards(name,true);},ms));return j||{};}catch(e){restoreIfLost();if(typeof alert==='function')alert('No se pudo preparar el cassette: '+(e&&e.message?e.message:e));return null;}finally{setTimeout(()=>{protectUntil=0;},900);}}
 async function unmountCasStable(ev){if(ev){try{ev.preventDefault();ev.stopPropagation();ev.stopImmediatePropagation&&ev.stopImmediatePropagation();}catch(e){}}snap();setProtect(8000);try{const r=await fetch('/cas/unmount?fast=1',{cache:'no-store'});const txt=await r.text();let j=null;try{j=txt?JSON.parse(txt):null;}catch(e){}if(!r.ok||(j&&j.ok===false))throw new Error((j&&j.error)?j.error:(txt||('HTTP '+r.status)));markCas('',false);restoreIfLost();updateCards('',false);try{if(typeof updateCasManualPanelF49==='function')updateCasManualPanelF49(Object.assign({casMounted:0,mounted:0,casPlaying:0,playing:0,casName:'',name:''},j||{}));}catch(e){}try{if(window.casLastStatusF49J)Object.assign(window.casLastStatusF49J,{casMounted:0,mounted:0,casPlaying:0,playing:0,casName:'',name:''});}catch(e){}if(typeof setStatus==='function')setStatus('C: liberado. Biblioteca conservada.',true);[150,650,1800].forEach(ms=>setTimeout(()=>{restoreIfLost();updateCards('',false);},ms));return j||{};}catch(e){restoreIfLost();if(typeof alert==='function')alert('No se pudo soltar el cassette: '+(e&&e.message?e.message:e));return null;}finally{setTimeout(()=>{protectUntil=0;},900);}}
 try{window.mountCasQuick=mountCasStable;mountCasQuick=mountCasStable;}catch(e){}
 try{window.unmountCasQuick=unmountCasStable;unmountCasQuick=unmountCasStable;}catch(e){}
 try{window.casApplyLibraryMountStateF49Z13=function(name,st){markCas(name,true);restoreIfLost();updateCards(name,true);try{if(typeof updateCasManualPanelF49==='function')updateCasManualPanelF49(Object.assign({casMounted:1,casPlaying:0,casName:name,name:name},st||{}));}catch(e){}};}catch(e){}
 function wrap(name){try{const old=window[name]||(typeof globalThis[name]==='function'?globalThis[name]:null);if(typeof old==='function'&&!old.__f49z56){const w=function(){if(protectedNow()&&!isManual(arguments))return Promise.resolve(window.libraryStateF49Z47&&window.libraryStateF49Z47.lastPayload||null);return old.apply(this,arguments);};w.__f49z56=1;window[name]=w;try{globalThis[name]=w;}catch(e){}}}catch(e){}}
 ['loadLibraryPageF49Z45','loadLibraryPageF49Z47','loadAtrStatus','loadStatus','loadCasStatusF49C'].forEach(wrap);
 try{if(typeof renderAtrLibrary==='function'&&!renderAtrLibrary.__f49z56){const old=renderAtrLibrary;renderAtrLibrary=function(files,opts){opts=opts||{};if(protectedNow()&&!opts.refresh&&!opts.manualRefresh){restoreIfLost();return;}return old.apply(this,arguments);};renderAtrLibrary.__f49z56=1;window.renderAtrLibrary=renderAtrLibrary;}}catch(e){}
 document.addEventListener('click',function(ev){const btn=ev.target&&ev.target.closest?ev.target.closest('button[data-cas-prepare],button[onclick*="mountCasQuick"],#btnCasUnmountPage'):null;if(!btn)return;if(btn.id==='btnCasUnmountPage'){unmountCasStable(ev);return;}const raw=btn.getAttribute('data-file')||'';const oc=btn.getAttribute('onclick')||'';const m=/mountCasQuick\('([^']+)'\)/.exec(oc);const arg=m?m[1]:enc(raw);if(arg)mountCasStable(arg,ev);},true);
 // Glifos: ATASCII se asocia por valor hexadecimal. Ej.: code "37" => 0x37 => '7'; code "3D" => '='.
 function codeFromField(v,field){if(typeof v==='number')return Number.isFinite(v)?(v&0x7F):NaN;let s=String(v??'').trim();if(!s)return NaN;if(/^0x[0-9a-f]+$/i.test(s))return parseInt(s,16)&0x7F;if(/^[0-9a-f]{1,2}$/i.test(s))return parseInt(s,16)&0x7F;if(/^\d+$/.test(s))return parseInt(s,10)&0x7F;return NaN;}
 try{window.glyphCodeFromJsonFieldF56=codeFromField;}catch(e){}
 if(typeof compactGlyphImportFromText==='function'){
  window.compactGlyphImportFromText=compactGlyphImportFromText=function(txt){const lines=[],seen=new Set();const push=(code,rows)=>{code=Number(code);if(!Number.isFinite(code)||code<0||code>127)return;const hx=(typeof glyphHexV30==='function'?glyphHexV30(code):(code&127).toString(16).toUpperCase().padStart(2,'0'));if(seen.has(hx))return;const rr=(typeof glyphNormalizeRowsV30==='function'?glyphNormalizeRowsV30(rows||[]):[]);if(rr.length===8){lines.push(hx+':'+rr.join(','));seen.add(hx);}};try{const json=JSON.parse(txt);(Array.isArray(json.fonts)?json.fonts:[]).forEach(font=>(Array.isArray(font.glyphs)?font.glyphs:[]).forEach(g=>{let c=NaN;if(g.id!==undefined)c=codeFromField(g.id,'id');else if(g.code!==undefined)c=codeFromField(g.code,'code');else if(g.codeHex!==undefined)c=codeFromField(g.codeHex,'codeHex');push(c,g.rowsHex||g.rows);}));if(lines.length)return lines.join('\n');}catch(e){}const re=/"(id|code|codeHex)"\s*:\s*(?:"([^"\\]*(?:\\.[^"\\]*)*)"|(\d+))[\s\S]*?"(?:rowsHex|rows)"\s*:\s*\[([\s\S]*?)\]/g;let m;while((m=re.exec(txt))){const raw=m[2]!==undefined?m[2]:m[3];const c=codeFromField(raw,m[1]);const rows=[];const rh=/"([^"\\]*(?:\\.[^"\\]*)*)"|(\d+)/g;let rm;while((rm=rh.exec(m[4]))&&rows.length<8)rows.push(typeof glyphRowToHex8V53==='function'?glyphRowToHex8V53(rm[1]!==undefined?rm[1]:Number(rm[2])):(rm[1]||rm[2]));push(c,rows);}return lines.join('\n');};
 }
})();



(function(){
 if(window.__glyphRestorePreviousFormatF49Z59)return;window.__glyphRestorePreviousFormatF49Z59=1;
 function q(id){return document.getElementById(id)}
 function numCode(v){
  if(v===null||v===undefined)return null;
  if(typeof v==='number'&&isFinite(v))return Math.max(0,Math.min(127,Math.round(v)));
  let s=String(v).trim(); if(!s)return null;
  if(/^0x/i.test(s))return Math.max(0,Math.min(127,parseInt(s,16)||0));
  if(/^[0-9A-Fa-f]{2}$/.test(s))return Math.max(0,Math.min(127,parseInt(s,16)||0));
  const n=parseInt(s,10); return isFinite(n)?Math.max(0,Math.min(127,n)):null;
 }
 function normalizeImportedGlyphsByCode(){
  try{
   const src=window.glyphMapV27||window.glyphRowsByCodeV27||window.glyphsByCode||null;
   if(!src)return;
   const out={};
   if(Array.isArray(src)){
    src.forEach((g,i)=>{
     const c=numCode(g&&((g.code!==undefined?g.code:null)??(g.id!==undefined?g.id:null)??(g.codeHex!==undefined?g.codeHex:null)??i));
     if(c===null)return;
     out[c]=g;
    });
   }else if(typeof src==='object'){
    Object.keys(src).forEach(k=>{
     const g=src[k];
     const c=numCode((g&&((g.code!==undefined?g.code:null)??(g.id!==undefined?g.id:null)??(g.codeHex!==undefined?g.codeHex:null)))??k);
     if(c===null)return;
     out[c]=g;
    });
   }
   window.glyphsByCodeF49Z59=out;
  }catch(e){}
 }
 const oldSwitch=window.glyphSwitchViewV32;
 window.glyphSwitchViewV32=glyphSwitchViewV32=function(view){
  view=view||'grid';
  if(typeof oldSwitch==='function')return oldSwitch.call(this,view);
  const panels={grid:'glyphViewGridV32',chars:'glyphViewCharsV32',hex:'glyphViewHexV32'};
  Object.keys(panels).forEach(k=>{const el=q(panels[k]);if(el)el.classList.toggle('active',k===view)});
  document.querySelectorAll('[data-glyph-view]').forEach(b=>b.classList.toggle('active',(b.dataset.glyphView||'grid')===view));
 };
 const oldBuild=window.buildGlyphV27Layout;
 window.buildGlyphV27Layout=buildGlyphV27Layout=function(){
  normalizeImportedGlyphsByCode();
  if(typeof oldBuild==='function')oldBuild.apply(this,arguments);
  try{window.glyphSwitchViewV32('grid')}catch(e){}
 };
 const oldSave=window.glyphSaveAllCurrentV34;
 if(typeof oldSave==='function')window.glyphSaveAllCurrentV34=glyphSaveAllCurrentV34=function(){
  normalizeImportedGlyphsByCode();
  return oldSave.apply(this,arguments);
 };
 const oldImport=window.importGlyphs;
 if(typeof oldImport==='function'&&!oldImport.__f49z59){
  const w=function(){const r=oldImport.apply(this,arguments);try{Promise.resolve(r).finally(()=>{normalizeImportedGlyphsByCode();try{if(typeof buildGlyphMatrixV27==='function')buildGlyphMatrixV27(true)}catch(e){};try{window.glyphSwitchViewV32('grid')}catch(e){}})}catch(e){}return r};
  w.__f49z59=1;window.importGlyphs=importGlyphs=w;
 }
 document.addEventListener('DOMContentLoaded',()=>{setTimeout(()=>{if(document.body&&document.body.classList.contains('route-glyphs')){try{window.glyphSwitchViewV32('grid')}catch(e){}}},80)});
})();



(function(){
 if(window.__glyphBitmapByCodeF49Z60)return;window.__glyphBitmapByCodeF49Z60=1;
 function q(id){return document.getElementById(id)}
 function hx(c){try{return (Number(c||0)&127).toString(16).toUpperCase().padStart(2,'0')}catch(e){return'00'}}
 function paint(el,code){try{if(typeof glyphPaintRowsV27==='function'&&typeof glyphRowsForCodeV27==='function')glyphPaintRowsV27(el,glyphRowsForCodeV27(code),'glyph-mini-pixels');}catch(e){}}
 function build(force){
  const grid=q('glyphMatrixV27'); if(!grid)return;
  if(force){grid.dataset.ready='0';grid.innerHTML='';}
  if(grid.dataset.ready==='1'&&grid.dataset.layout==='f60')return;
  grid.dataset.ready='1';grid.dataset.layout='f60';grid.innerHTML='';
  const blank=document.createElement('div');blank.className='glyph-axis';blank.textContent='';grid.appendChild(blank);
  for(let c=0;c<16;c++){const h=document.createElement('div');h.className='glyph-axis';h.textContent=hx(c);grid.appendChild(h);}
  for(let r=0;r<8;r++){
   const row=document.createElement('div');row.className='glyph-axis';row.textContent=hx(r*16);grid.appendChild(row);
   for(let c=0;c<16;c++){
    const code=r*16+c,hex=hx(code);const b=document.createElement('button');
    b.type='button';b.className='glyph-code-cell glyph-code-bitmap-f60';b.dataset.code=String(code);b.dataset.codehex=hex;b.title='0x'+hex;
    const pix=document.createElement('div');paint(pix,code);b.appendChild(pix);
    b.addEventListener('click',function(){try{glyphSelectCodeV27(code,true)}catch(e){};});
    b.addEventListener('dblclick',function(ev){ev.preventDefault();try{glyphFullscreenOpenV29(code,glyphRowsForCodeV27(code))}catch(e){}});
    grid.appendChild(b);
   }
  }
 }
 window.buildGlyphMatrixV27=build;
 const oldUpdate=window.glyphUpdateMatrixCellV28;
 window.glyphUpdateMatrixCellV28=function(code,rows){
  try{const cell=document.querySelector('.glyph-code-cell[data-code="'+(Number(code||0)&127)+'"]');if(cell){let pix=cell.querySelector('.glyph-mini-pixels');if(!pix){pix=document.createElement('div');cell.innerHTML='';cell.appendChild(pix);}glyphPaintRowsV27(pix,(rows&&rows.length)?rows:glyphRowsForCodeV27(code),'glyph-mini-pixels');}}
  catch(e){}
  if(typeof oldUpdate==='function')try{return oldUpdate.apply(this,arguments)}catch(e){}
 };
 const oldRefresh=window.refreshGlyphMatrixFromDeviceV30;
 if(typeof oldRefresh==='function')window.refreshGlyphMatrixFromDeviceV30=function(){const p=oldRefresh.apply(this,arguments);try{Promise.resolve(p).finally(function(){try{build(true)}catch(e){}})}catch(e){}return p};
 document.addEventListener('DOMContentLoaded',function(){setTimeout(function(){if(document.body&&document.body.classList.contains('route-glyphs')){try{build(true)}catch(e){};try{refreshGlyphMatrixFromDeviceV30()}catch(e){}}},120)});
})();



(function(){
 if(window.__glyphBitmapVisibilityFixF49Z61)return;window.__glyphBitmapVisibilityFixF49Z61=1;
 function isGlyphs(){try{return document.body&&document.body.classList.contains('route-glyphs')}catch(e){return false}}
 function repaint(){
  if(!isGlyphs())return;
  try{if(typeof buildGlyphMatrixV27==='function')buildGlyphMatrixV27(true)}catch(e){}
  try{document.querySelectorAll('.glyph-code-cell.glyph-code-bitmap-f60 .glyph-mini-pixels').forEach(function(el){el.style.setProperty('display','grid','important');el.style.setProperty('visibility','visible','important');el.style.setProperty('opacity','1','important');});}catch(e){}
 }
 const oldApply=window.applyRouteLayout;
 if(typeof oldApply==='function'&&!oldApply.__f49z61){
  window.applyRouteLayout=function(){const r=oldApply.apply(this,arguments);setTimeout(repaint,40);setTimeout(repaint,180);return r};
  window.applyRouteLayout.__f49z61=1;
 }
 const oldBuild=window.buildGlyphV27Layout;
 if(typeof oldBuild==='function'&&!oldBuild.__f49z61){
  window.buildGlyphV27Layout=function(){const r=oldBuild.apply(this,arguments);setTimeout(repaint,40);return r};
  window.buildGlyphV27Layout.__f49z61=1;
 }
 document.addEventListener('DOMContentLoaded',function(){setTimeout(repaint,120);setTimeout(repaint,500)});
 document.addEventListener('click',function(ev){if(ev.target&&ev.target.closest&&ev.target.closest('[data-route="glyphs"],a[href="/glyphs"],a[href="/editor/glyphs"]'))setTimeout(repaint,160)},true);
})();



(function(){
 if(window.__glyphCodeHexRuntimeFixF49Z62)return;window.__glyphCodeHexRuntimeFixF49Z62=1;
 function hx(c){return (Number(c||0)&127).toString(16).toUpperCase().padStart(2,'0')}
 function blankRows(rows){try{return !Array.isArray(rows)||rows.length<8||rows.slice(0,8).every(function(r){return (parseInt(String(r||'0').replace(/^0x/i,''),16)||0)===0})}catch(e){return true}}
 function fallbackRows(code){
  code=Number(code||0)&127;
  try{if(typeof GLYPH_CONTROL_8X8_V34!=='undefined'&&GLYPH_CONTROL_8X8_V34[code])return GLYPH_CONTROL_8X8_V34[code].slice()}catch(e){}
  try{
   const ch=String.fromCharCode(code);const rows5=(typeof GLYPH5X7_V27!=='undefined')&&(GLYPH5X7_V27[ch]||GLYPH5X7_V27[(ch.toUpperCase&&ch.toUpperCase())||ch]);
   if(rows5){const out=[0];rows5.forEach(function(v){out.push((Number(v||0)&31)<<1)});while(out.length<8)out.push(0);return out.slice(0,8).map(function(v){return v.toString(16).toUpperCase().padStart(2,'0')})}
  }catch(e){}
  return ['00','00','00','00','00','00','00','00'];
 }
 const oldRows=window.glyphRowsForCodeV27;
 window.glyphRowsForCodeV27=function(code){
  code=Number(code||0)&127;
  let rows=null;try{rows=oldRows?oldRows(code):null}catch(e){}
  // Si una cache dañada dejó todo en 00, no se debe tapar el glifo real. La excepción es 0x20 espacio.
  if(code!==0x20 && blankRows(rows)) rows=fallbackRows(code);
  return rows;
 };
 function repaintCell(code){
  try{const cell=document.querySelector('.glyph-code-cell[data-code="'+(Number(code||0)&127)+'"]');if(!cell)return;let pix=cell.querySelector('.glyph-mini-pixels');if(!pix){pix=document.createElement('div');cell.innerHTML='';cell.appendChild(pix)}glyphPaintRowsV27(pix,window.glyphRowsForCodeV27(code),'glyph-mini-pixels')}catch(e){}
 }
 function repaintAll(){
  try{for(let code=0;code<128;code++)repaintCell(code)}catch(e){}
 }
 const oldBuild=window.buildGlyphMatrixV27;
 if(typeof oldBuild==='function'&&!oldBuild.__f49z62){
  window.buildGlyphMatrixV27=function(){const r=oldBuild.apply(this,arguments);setTimeout(repaintAll,20);setTimeout(repaintAll,180);return r};
  window.buildGlyphMatrixV27.__f49z62=1;
 }
 const oldRefresh=window.refreshGlyphMatrixFromDeviceV30;
 if(typeof oldRefresh==='function'&&!oldRefresh.__f49z62){
  window.refreshGlyphMatrixFromDeviceV30=function(){const p=oldRefresh.apply(this,arguments);try{Promise.resolve(p).finally(function(){setTimeout(repaintAll,60)})}catch(e){}return p};
  window.refreshGlyphMatrixFromDeviceV30.__f49z62=1;
 }
 document.addEventListener('DOMContentLoaded',function(){setTimeout(repaintAll,350);setTimeout(repaintAll,900)});
})();



(function(){
 if(window.__glyphHexAssocNoFreezeF49Z65)return;window.__glyphHexAssocNoFreezeF49Z65=1;
 function hx(code){return (Number(code||0)&127).toString(16).toUpperCase().padStart(2,'0')}
 function parseCode(raw,field){
  if(typeof raw==='number')return Number.isFinite(raw)?(raw&0x7F):NaN;
  var s=String(raw==null?'':raw).trim(); if(!s)return NaN;
  if(/^0x[0-9a-f]+$/i.test(s))return parseInt(s,16)&0x7F;
  if(/^[0-9a-f]{1,2}$/i.test(s))return parseInt(s,16)&0x7F; // "37" => 0x37, "3D" => 0x3D
  if(/^\d+$/.test(s))return parseInt(s,10)&0x7F;
  return NaN;
 }
 function label(code){
  code=Number(code||0)&127;
  if(code===0x20)return '␠';
  if(code>=33&&code<=126)return String.fromCharCode(code);
  try{if(typeof glyphControlNameV33==='function')return glyphControlNameV33(code)}catch(e){}
  return '0x'+hx(code);
 }
 function normRows(rows){
  try{if(typeof glyphNormalizeRowsV30==='function')return glyphNormalizeRowsV30(rows||[])}catch(e){}
  var out=[];(Array.isArray(rows)?rows:[]).slice(0,8).forEach(function(r){var v=String(r||'00').trim().replace(/^0x/i,'').toUpperCase();out.push(v.padStart(2,'0').slice(-2));});while(out.length<8)out.push('00');return out;
 }
 function applyCellLabels(root){
  try{(root||document).querySelectorAll('.glyph-code-cell[data-code]').forEach(function(cell){var c=Number(cell.dataset.code||0)&127;var h=hx(c);cell.dataset.codehex=h;cell.dataset.chr=label(c);cell.title='code '+h+' = 0x'+h+' ('+c+') · '+label(c);});}catch(e){}
 }
 var oldAny=window.glyphCodeFromAnyV31;
 window.glyphCodeFromAnyV31=function(value,field){var c=parseCode(value,field);if(Number.isFinite(c))return c;return (typeof oldAny==='function')?oldAny(value):NaN};
 window.glyphCodeFromJsonFieldF49Z65=parseCode;
 window.glyphParseRowsMapFromJsonV30=function(txt){
  var map={};
  try{var json=JSON.parse(txt);(Array.isArray(json.fonts)?json.fonts:[]).forEach(function(font){(Array.isArray(font.glyphs)?font.glyphs:[]).forEach(function(g){var raw=(g.code!==undefined)?g.code:((g.id!==undefined)?g.id:g.codeHex);var field=(g.code!==undefined)?'code':((g.id!==undefined)?'id':'codeHex');var c=parseCode(raw,field);if(!Number.isFinite(c)||c<0||c>127)return;map[hx(c)]=normRows(g.rowsHex||g.rows||[]);});});}
  catch(e){try{console.warn('No se pudo interpretar JSON de glifos',e)}catch(_){} }
  return map;
 };
 try{glyphParseRowsMapFromJsonV30=window.glyphParseRowsMapFromJsonV30}catch(e){}
 window.compactGlyphImportFromText=function(txt){
  var lines=[],seen={};
  function push(codeAny,rowsAny,field){var c=parseCode(codeAny,field);if(!Number.isFinite(c)||c<0||c>127)return;var h=hx(c);if(seen[h])return;var rows=normRows(rowsAny);if(rows.length===8){lines.push(h+':'+rows.join(','));seen[h]=1;}}
  try{var json=JSON.parse(txt);(Array.isArray(json.fonts)?json.fonts:[]).forEach(function(font){(Array.isArray(font.glyphs)?font.glyphs:[]).forEach(function(g){var raw=(g.code!==undefined)?g.code:((g.id!==undefined)?g.id:g.codeHex);var field=(g.code!==undefined)?'code':((g.id!==undefined)?'id':'codeHex');push(raw,g.rowsHex||g.rows||[],field);});});if(lines.length)return lines.join('\n');}catch(e){}
  var re=/"(code|id|codeHex)"\s*:\s*(?:"([^"\\]*(?:\\.[^"\\]*)*)"|(\d+))[\s\S]*?"(?:rowsHex|rows)"\s*:\s*\[([\s\S]*?)\]/g,m;
  while((m=re.exec(txt))){var raw=m[2]!==undefined?m[2]:m[3],rows=[],rh=/"([^"\\]*(?:\\.[^"\\]*)*)"|(\d+)/g,rm;while((rm=rh.exec(m[4]))&&rows.length<8)rows.push(rm[1]!==undefined?rm[1]:Number(rm[2]));push(raw,rows,m[1]);}
  return lines.join('\n');
 };
 try{compactGlyphImportFromText=window.compactGlyphImportFromText}catch(e){}
 var oldBuild=window.buildGlyphMatrixV27;
 if(typeof oldBuild==='function'&&!oldBuild.__f49z65){window.buildGlyphMatrixV27=function(){var r=oldBuild.apply(this,arguments);applyCellLabels(document);return r};window.buildGlyphMatrixV27.__f49z65=1;}
 var oldUpdate=window.glyphUpdateMatrixCellV28;
 if(typeof oldUpdate==='function'&&!oldUpdate.__f49z65){window.glyphUpdateMatrixCellV28=function(){var r=oldUpdate.apply(this,arguments);applyCellLabels(document);return r};window.glyphUpdateMatrixCellV28.__f49z65=1;}
 document.addEventListener('DOMContentLoaded',function(){setTimeout(function(){applyCellLabels(document)},120);setTimeout(function(){applyCellLabels(document)},600);});
 document.addEventListener('click',function(ev){if(ev.target&&ev.target.closest&&ev.target.closest('a[href="/editor/glyphs"],a[href="/editor/glifos"],a[href="/glyphs"],#navGlyphs'))setTimeout(function(){applyCellLabels(document)},160);},true);
})();



(function(){
 if(window.__glyphCanonicalHexCacheF49Z66)return;window.__glyphCanonicalHexCacheF49Z66=1;
 function hx(code){return (Number(code||0)&0x7F).toString(16).toUpperCase().padStart(2,'0');}
 function norm(rows){
  try{if(typeof glyphNormalizeRowsV30==='function')return glyphNormalizeRowsV30(rows||[]);}catch(e){}
  var out=[];(Array.isArray(rows)?rows:[]).slice(0,8).forEach(function(r){var v=String(r==null?'00':r).trim().replace(/^0x/i,'').toUpperCase();out.push(v.padStart(2,'0').slice(-2));});
  while(out.length<8)out.push('00');return out.slice(0,8);
 }
 function canonicalizeCache(){
  var old=window.glyphRowsCacheV30||{};
  var clean={};
  for(var code=0;code<128;code++){
   var h=hx(code);
   if(old[h]) clean[h]=norm(old[h]);
  }
  window.glyphRowsCacheV30=clean;
  return clean;
 }
 window.glyphCacheRowsV30=function(code,rows){
  var h=hx(code);
  var n=norm(rows);
  if(!window.glyphRowsCacheV30)window.glyphRowsCacheV30={};
  window.glyphRowsCacheV30[h]=n;
  return n;
 };
 window.glyphRowsFromCacheV30=function(code){
  var h=hx(code);
  return (window.glyphRowsCacheV30&&window.glyphRowsCacheV30[h])||null;
 };
 window.glyphCanonicalizeCacheF49Z66=canonicalizeCache;
 var oldCompact=window.glyphCompactFromCacheV34;
 window.glyphCompactFromCacheV34=function(){
  canonicalizeCache();
  try{var currentCode=(typeof glyphHexCodeValue==='function')?glyphHexCodeValue():0x41;window.glyphCacheRowsV30(currentCode,(typeof glyphRowsFromGrid==='function')?glyphRowsFromGrid():[]);}catch(e){}
  var lines=[];
  for(var code=0;code<128;code++){
   var rows=window.glyphRowsFromCacheV30(code);
   if(!rows&&typeof glyphRowsForCodeV27==='function')rows=glyphRowsForCodeV27(code);
   rows=norm(rows);
   lines.push(hx(code)+':'+rows.join(','));
  }
  return lines.join('\n');
 };
 var oldRefresh=window.refreshGlyphMatrixFromDeviceV30;
 if(typeof oldRefresh==='function'&&!oldRefresh.__f49z66){
  window.refreshGlyphMatrixFromDeviceV30=function(){
   var r=oldRefresh.apply(this,arguments);
   try{Promise.resolve(r).finally(function(){canonicalizeCache();});}catch(e){canonicalizeCache();}
   return r;
  };
  window.refreshGlyphMatrixFromDeviceV30.__f49z66=1;
 }
 var oldSave=window.glyphSaveAllCurrentV34;
 if(typeof oldSave==='function'&&!oldSave.__f49z66){
  window.glyphSaveAllCurrentV34=function(){canonicalizeCache();return oldSave.apply(this,arguments);};
  window.glyphSaveAllCurrentV34.__f49z66=1;
 }
 document.addEventListener('DOMContentLoaded',function(){setTimeout(canonicalizeCache,80);setTimeout(canonicalizeCache,500);});
})();



(function(){
 if(window.__glyphPersistVerifyF49Z70)return;window.__glyphPersistVerifyF49Z70=1;
 var oldSave=window.glyphSaveAllCurrentV34;
 if(typeof oldSave==='function'&&!oldSave.__f49z70){
  window.glyphSaveAllCurrentV34=async function(){
   var r=await oldSave.apply(this,arguments);
   try{
    var ex=await fetch('/glyphs_export?verify=1&ts='+Date.now(),{cache:'no-store'});
    var txt=await ex.text();
    var count=(txt.match(/"code"\s*:/g)||[]).length;
    if(typeof glyphSetEditorStatus==='function')glyphSetEditorStatus('Glifos guardados y verificados · '+count+'/128', count>=128);
    if(typeof setStatus==='function')setStatus('Glifos persistidos en SD y verificados: '+count+'/128', count>=128);
   }catch(e){try{if(typeof setStatus==='function')setStatus('Glifos guardados; no se pudo verificar export: '+(e.message||e),false)}catch(_){}}
   return r;
  };
  window.glyphSaveAllCurrentV34.__f49z70=1;
 }
})();



(function(){
 if(window.__glyphDefaultReloadF49Z74)return;window.__glyphDefaultReloadF49Z74=1;
 window.__glyphBusyF49Z74=false;
 window.__glyphReloadBlockedUntilF49Z74=0;
 function hx(code){return (Number(code||0)&127).toString(16).toUpperCase().padStart(2,'0');}
 async function reloadDefault(){
  try{
   if(window.__glyphBusyF49Z74 || (window.__glyphReloadBlockedUntilF49Z74 && Date.now()<window.__glyphReloadBlockedUntilF49Z74)) return 0;
   if(typeof buildGlyphV27Layout==='function')buildGlyphV27Layout();
   var r=await fetch('/glyphs_export?reload=1&default=1&ts='+Date.now(),{cache:'no-store'});
   var txt=await r.text();
   if(!r.ok)throw new Error(txt||('HTTP '+r.status));
   var map=(typeof glyphParseRowsMapFromJsonV30==='function')?glyphParseRowsMapFromJsonV30(txt):{};
   var count=0;
   Object.keys(map||{}).forEach(function(h){
    var code=parseInt(h,16)&127;
    if(typeof glyphCacheRowsV30==='function')glyphCacheRowsV30(code,map[h]);
    if(typeof glyphUpdateMatrixCellV28==='function')glyphUpdateMatrixCellV28(code,map[h]);
    count++;
   });
   if(typeof buildGlyphMatrixV27==='function')buildGlyphMatrixV27(true);
   Object.keys(map||{}).forEach(function(h){
    var code=parseInt(h,16)&127;
    if(typeof glyphUpdateMatrixCellV28==='function')glyphUpdateMatrixCellV28(code,map[h]);
   });
   var current=0x41;
   try{current=(typeof glyphHexCodeValue==='function')?glyphHexCodeValue():0x41;}catch(e){}
   current=Number(current||0x41)&127;
   if(typeof glyphRenderGrid==='function'&&typeof glyphRowsForCodeV27==='function')glyphRenderGrid(8,8,glyphRowsForCodeV27(current));
   if(typeof glyphSyncV27==='function')glyphSyncV27(current,{rowsHex:(typeof glyphRowsForCodeV27==='function')?glyphRowsForCodeV27(current):[]});
   if(typeof glyphSetEditorStatus==='function')glyphSetEditorStatus('Default cargado desde ESP32/SD · '+count+'/128',count>0);
   if(typeof setStatus==='function')setStatus('Glifos default cargados desde SD: '+count+'/128',count>0);
   return count;
  }catch(e){
   try{if(typeof glyphSetEditorStatus==='function')glyphSetEditorStatus('No se pudo recargar default: '+(e.message||e),false)}catch(_){}
   try{console.warn('glyph default reload F49Z74',e)}catch(_){}
   return 0;
  }
 }
 window.glyphReloadDefaultFromMasterF49Z74=reloadDefault;
 var oldImport=window.importGlyphs;
 if(typeof oldImport==='function'&&!oldImport.__f49z74){
  window.importGlyphs=async function(){
   window.__glyphBusyF49Z74=true;
   try{
    var r=await oldImport.apply(this,arguments);
    window.__glyphReloadBlockedUntilF49Z74=Date.now()+800;
    return r;
   }finally{
    setTimeout(async function(){
     window.__glyphBusyF49Z74=false;
     window.__glyphReloadBlockedUntilF49Z74=0;
     await reloadDefault();
    },220);
   }
  };
  window.importGlyphs.__f49z74=1;
 }
 var oldSave=window.glyphSaveAllCurrentV34;
 if(typeof oldSave==='function'&&!oldSave.__f49z74){
  window.glyphSaveAllCurrentV34=async function(){
   window.__glyphBusyF49Z74=true;
   try{
    var r=await oldSave.apply(this,arguments);
    window.__glyphReloadBlockedUntilF49Z74=Date.now()+600;
    return r;
   }finally{
    setTimeout(async function(){
     window.__glyphBusyF49Z74=false;
     window.__glyphReloadBlockedUntilF49Z74=0;
     await reloadDefault();
    },220);
   }
  };
  window.glyphSaveAllCurrentV34.__f49z74=1;
 }
 document.addEventListener('DOMContentLoaded',function(){setTimeout(function(){if(document.body&&document.body.classList.contains('route-glyphs'))reloadDefault();},350);});
 document.addEventListener('click',function(ev){try{var a=ev.target&&ev.target.closest&&ev.target.closest('a[href]');if(!a)return;var href=String(a.getAttribute('href')||'').toLowerCase();if((href.indexOf('/glyph')>=0||href.indexOf('/glifo')>=0) && !window.__glyphBusyF49Z74)setTimeout(reloadDefault,220);}catch(e){}},true);
})();



(function(){
 if(window.__glyphImportHardLockF49Z76)return;window.__glyphImportHardLockF49Z76=1;
 function hx(code){return (Number(code||0)&127).toString(16).toUpperCase().padStart(2,'0');}
 function norm(rows){
  try{if(typeof glyphNormalizeRowsV30==='function')return glyphNormalizeRowsV30(rows||[]);}catch(e){}
  var out=[];(Array.isArray(rows)?rows:[]).slice(0,8).forEach(function(r){var v=String(r==null?'00':r).trim().replace(/^0x/i,'').toUpperCase();out.push(v.padStart(2,'0').slice(-2));});
  while(out.length<8)out.push('00');return out.slice(0,8);
 }
 function compactToMap(compact){
  var map={};
  String(compact||'').split(/\n+/).forEach(function(line){
   line=String(line||'').trim(); if(!line)return;
   var sep=line.indexOf(':'); if(sep<0)sep=line.indexOf('='); if(sep<=0)return;
   var h=String(line.slice(0,sep)).trim().replace(/^0x/i,'').toUpperCase().padStart(2,'0').slice(-2);
   var code=parseInt(h,16); if(!Number.isFinite(code)||code<0||code>127)return;
   var rows=norm(line.slice(sep+1).split(','));
   if(rows.length===8)map[hx(code)]=rows;
  });
  return map;
 }
 function protect(map,ms){
  window.__glyphProtectMapF49Z76=map||window.__glyphProtectMapF49Z76||{};
  window.__glyphProtectUntilF49Z76=Date.now()+(ms||15000);
  window.__glyphBusyF49Z74=true;
  window.__glyphReloadBlockedUntilF49Z74=Date.now()+(ms||15000);
 }
 var __glyphApplyingProtectedF49Z76=false;
 function isProtected(){return window.__glyphProtectMapF49Z76&&Date.now()<(window.__glyphProtectUntilF49Z76||0);}
 function applyProtectedMap(){
  if(__glyphApplyingProtectedF49Z76)return 0;
  __glyphApplyingProtectedF49Z76=true;
  var map=window.__glyphProtectMapF49Z76||{};
  var keys=Object.keys(map); if(!keys.length){__glyphApplyingProtectedF49Z76=false;return 0;}
  try{if(typeof buildGlyphV27Layout==='function')buildGlyphV27Layout();}catch(e){}
  keys.forEach(function(h){var code=parseInt(h,16)&127;try{if(typeof glyphCacheRowsV30==='function')glyphCacheRowsV30(code,map[h]);}catch(e){}});
  try{if(typeof buildGlyphMatrixV27==='function')buildGlyphMatrixV27(true);}catch(e){}
  keys.forEach(function(h){var code=parseInt(h,16)&127;try{if(typeof glyphUpdateMatrixCellV28==='function')glyphUpdateMatrixCellV28(code,map[h]);}catch(e){}});
  var current=0x41;try{current=(typeof glyphHexCodeValue==='function')?glyphHexCodeValue():parseInt(keys[0],16);}catch(e){current=parseInt(keys[0],16);}
  current=Number(current||parseInt(keys[0],16)||0x41)&127;
  var ch=hx(current);
  var rows=map[ch]||map[keys[0]];
  if(rows){
   try{if(typeof glyphRenderGrid==='function')glyphRenderGrid(8,8,rows);}catch(e){}
   try{if(typeof glyphSyncV27==='function')glyphSyncV27(current,{rowsHex:rows});}catch(e){}
  }
  try{if(typeof glyphSetEditorStatus==='function')glyphSetEditorStatus('Glifos importados activos en RAM · '+keys.length+'/128',keys.length>0);}catch(e){}
  __glyphApplyingProtectedF49Z76=false;
  return keys.length;
 }
 var oldRefresh=window.refreshGlyphMatrixFromDeviceV30;
 window.refreshGlyphMatrixFromDeviceV30=async function(){
  if(isProtected())return applyProtectedMap();
  return (typeof oldRefresh==='function')?oldRefresh.apply(this,arguments):0;
 };
 var oldLoadCurrent=window.glyphLoadCurrent;
 window.glyphLoadCurrent=async function(){
  if(isProtected()){
   var code=0x41;try{code=(typeof glyphHexCodeValue==='function')?glyphHexCodeValue():0x41;}catch(e){}
   var rows=(window.__glyphProtectMapF49Z76||{})[hx(code)];
   if(rows){
    try{if(typeof glyphCacheRowsV30==='function')glyphCacheRowsV30(code,rows);}catch(e){}
    try{if(typeof glyphRenderGrid==='function')glyphRenderGrid(8,8,rows);}catch(e){}
    try{if(typeof glyphSyncV27==='function')glyphSyncV27(code,{rowsHex:rows});}catch(e){}
    try{if(typeof glyphSetEditorStatus==='function')glyphSetEditorStatus('Glifo importado activo · 0x'+hx(code),true);}catch(e){}
    return;
   }
  }
  return (typeof oldLoadCurrent==='function')?oldLoadCurrent.apply(this,arguments):undefined;
 };
 var oldBuild=window.buildGlyphMatrixV27;
 if(typeof oldBuild==='function'){
  window.buildGlyphMatrixV27=function(){var r=oldBuild.apply(this,arguments);if(isProtected()&&!__glyphApplyingProtectedF49Z76)setTimeout(applyProtectedMap,0);return r;};
 }
 window.glyphReloadDefaultFromMasterF49Z74=async function(){
  if(isProtected())return applyProtectedMap();
  return (typeof oldRefresh==='function')?oldRefresh.apply(this,arguments):0;
 };
 window.importGlyphs=async function(){
  try{
   var input=document.getElementById('glyphFile');
   var f=input&&input.files&&input.files[0];
   if(!f)throw new Error('Selecciona atascii_glyphs.json primero');
   var txt=await f.text();
   var compact=(typeof compactGlyphImportFromText==='function')?compactGlyphImportFromText(txt):'';
   if(!compact||compact.length<8)throw new Error('No se encontraron glifos rowsHex válidos en el archivo');
   var map=compactToMap(compact);
   var count=Object.keys(map).length;
   if(!count)throw new Error('Import sin glifos válidos');
   protect(map,20000);
   applyProtectedMap();
   try{if(typeof setStatus==='function')setStatus('Importando glifos como default...',true);}catch(e){}
   var r=await fetch('/glyphs_import_compact?ts='+Date.now(),{method:'POST',headers:{'Content-Type':'text/plain; charset=utf-8','Cache-Control':'no-store'},body:compact});
   var body=await r.text();
   if(!r.ok)throw new Error(body||('HTTP '+r.status));
   protect(map,20000);
   applyProtectedMap();
   if(input)input.value='';
   try{if(typeof setStatus==='function')setStatus('Glifos importados y fijados como default: '+count+'/128 · '+body,true);}catch(e){}
   setTimeout(function(){window.__glyphBusyF49Z74=false;},20500);
  }catch(e){
   window.__glyphBusyF49Z74=false;
   try{if(typeof setStatus==='function')setStatus('Error importando glifos: '+(e.message||e),false);}catch(_){}
   try{console.error(e);}catch(_){}
  }
 };
 window.glyphImportHardLockApplyF49Z76=applyProtectedMap;
})();



(function(){
 if(window.__glyphExportSourceTruthF49Z78)return;window.__glyphExportSourceTruthF49Z78=1;
 const PROTECTED_MS=25000;
 function isGlyphRoute(){try{return document.body&&document.body.classList.contains('route-glyphs')}catch(e){return false}}
 function hx(code){return (Number(code||0)&127).toString(16).toUpperCase().padStart(2,'0')}
 function norm(rows){
  try{if(typeof glyphNormalizeRowsV30==='function')return glyphNormalizeRowsV30(rows||[])}catch(e){}
  const out=[];(Array.isArray(rows)?rows:[]).slice(0,8).forEach(function(r){let v=String(r==null?'00':r).trim().replace(/^0x/i,'').toUpperCase();out.push(v.padStart(2,'0').slice(-2));});
  while(out.length<8)out.push('00');return out.slice(0,8);
 }
 function mapCount(map){return Object.keys(map||{}).length}
 function compactToMap(compact){
  const map={};
  String(compact||'').split(/\n+/).forEach(function(line){
   line=String(line||'').trim(); if(!line)return;
   let sep=line.indexOf(':'); if(sep<0)sep=line.indexOf('='); if(sep<=0)return;
   const h=String(line.slice(0,sep)).trim().replace(/^0x/i,'').toUpperCase().padStart(2,'0').slice(-2);
   const c=parseInt(h,16); if(!Number.isFinite(c)||c<0||c>127)return;
   const rows=norm(line.slice(sep+1).split(','));
   if(rows.length===8)map[hx(c)]=rows;
  });
  return map;
 }
 function applyMap(map,opts){
  map=map||window.__glyphDefaultMapF49Z78||{};
  const keys=Object.keys(map); if(!keys.length)return 0;
  window.__glyphDefaultMapF49Z78=map;
  try{keys.forEach(function(h){const c=parseInt(h,16)&127;if(typeof glyphCacheRowsV30==='function')glyphCacheRowsV30(c,map[h]);});}catch(e){}
  if(!opts||opts.rebuild!==false){try{if(typeof buildGlyphMatrixV27==='function')buildGlyphMatrixV27(true)}catch(e){}}
  try{keys.forEach(function(h){const c=parseInt(h,16)&127;if(typeof glyphUpdateMatrixCellV28==='function')glyphUpdateMatrixCellV28(c,map[h]);});}catch(e){}
  let code=0x41;try{code=(typeof glyphHexCodeValue==='function')?glyphHexCodeValue():0x41}catch(e){}
  code=Number(code);if(!Number.isFinite(code))code=0x41;code=code&127;
  const rows=map[hx(code)]||map[keys[0]];
  if(rows){
   try{if(typeof glyphRenderGrid==='function')glyphRenderGrid(8,8,rows)}catch(e){}
   try{if(typeof glyphSyncV27==='function')glyphSyncV27(code,{rowsHex:rows,source:'SD/default'})}catch(e){}
  }
  return keys.length;
 }
 async function loadDefaultFromExport(reason){
  try{
   if(window.__glyphProtectMapF49Z76 && Date.now()<(window.__glyphProtectUntilF49Z76||0)){
    return applyMap(window.__glyphProtectMapF49Z76,{rebuild:true});
   }
   const url='/glyphs_export?reload=1&default=1&source=sd&ts='+Date.now();
   const r=await fetch(url,{cache:'no-store',headers:{'Cache-Control':'no-store'}});
   const txt=await r.text();
   if(!r.ok)throw new Error(txt||('HTTP '+r.status));
   const map=(typeof glyphParseRowsMapFromJsonV30==='function')?glyphParseRowsMapFromJsonV30(txt):{};
   const count=mapCount(map);
   if(count>0){
    window.__glyphMasterDefaultLoadedF49Z78=true;
    window.__glyphDefaultMapF49Z78=map;
    applyMap(map,{rebuild:true});
    try{if(typeof glyphSetEditorStatus==='function')glyphSetEditorStatus('Default SD aplicado desde /glyphs_export · '+count+'/128',count>=128)}catch(e){}
    try{if(typeof setStatus==='function')setStatus('Glifos default aplicados desde SD/export: '+count+'/128',count>=128)}catch(e){}
   }
   return count;
  }catch(e){
   try{console.warn('glyph export source truth F49Z78',reason,e)}catch(_){}
   return 0;
  }
 }
 window.glyphLoadDefaultFromExportF49Z78=loadDefaultFromExport;
 window.glyphApplyDefaultMapF49Z78=applyMap;
 const oldRows=window.glyphRowsForCodeV27;
 window.glyphRowsForCodeV27=function(code){
  const map=window.__glyphDefaultMapF49Z78||window.__glyphProtectMapF49Z76||{};
  const rows=map[hx(code)];
  if(rows&&rows.length)return rows.slice();
  return (typeof oldRows==='function')?oldRows.apply(this,arguments):norm([]);
 };
 const oldLoadCurrent=window.glyphLoadCurrent;
 window.glyphLoadCurrent=async function(){
  const map=window.__glyphDefaultMapF49Z78||window.__glyphProtectMapF49Z76||{};
  let code=0x41;try{code=(typeof glyphHexCodeValue==='function')?glyphHexCodeValue():0x41}catch(e){}
  code=Number(code);if(!Number.isFinite(code))code=0x41;code=code&127;
  const rows=map[hx(code)];
  if(rows&&rows.length){
   try{if(typeof glyphCacheRowsV30==='function')glyphCacheRowsV30(code,rows)}catch(e){}
   try{if(typeof glyphRenderGrid==='function')glyphRenderGrid(8,8,rows)}catch(e){}
   try{if(typeof glyphSyncV27==='function')glyphSyncV27(code,{rowsHex:rows,source:'SD/default'})}catch(e){}
   try{if(typeof glyphSetEditorStatus==='function')glyphSetEditorStatus('Glifo desde default SD · 0x'+hx(code),true)}catch(e){}
   return;
  }
  const loaded=await loadDefaultFromExport('glyphLoadCurrent');
  if(loaded){return window.glyphLoadCurrent.apply(this,arguments)}
  return (typeof oldLoadCurrent==='function')?oldLoadCurrent.apply(this,arguments):undefined;
 };
 const oldBuild=window.buildGlyphMatrixV27;
 if(typeof oldBuild==='function'){
  window.buildGlyphMatrixV27=function(){const r=oldBuild.apply(this,arguments);setTimeout(function(){applyMap(window.__glyphDefaultMapF49Z78||window.__glyphProtectMapF49Z76,{rebuild:false})},0);return r};
 }
 window.glyphReloadDefaultFromMasterF49Z74=async function(){return loadDefaultFromExport('reloadDefaultOverride')};
 const oldRefresh=window.refreshGlyphMatrixFromDeviceV30;
 window.refreshGlyphMatrixFromDeviceV30=async function(){const n=await loadDefaultFromExport('refreshOverride'); if(n)return n; return (typeof oldRefresh==='function')?oldRefresh.apply(this,arguments):0};
 const oldImport=window.importGlyphs;
 window.importGlyphs=async function(){
  try{
   const input=document.getElementById('glyphFile');
   const f=input&&input.files&&input.files[0];
   if(f){
    const txt=await f.text();
    const compact=(typeof compactGlyphImportFromText==='function')?compactGlyphImportFromText(txt):'';
    const map=compactToMap(compact);
    if(mapCount(map)>0){
     window.__glyphProtectMapF49Z76=map;
     window.__glyphProtectUntilF49Z76=Date.now()+PROTECTED_MS;
     window.__glyphDefaultMapF49Z78=map;
     window.__glyphBusyF49Z74=true;
     window.__glyphReloadBlockedUntilF49Z74=Date.now()+PROTECTED_MS;
     applyMap(map,{rebuild:true});
    }
   }
  }catch(e){}
  const r=(typeof oldImport==='function')?await oldImport.apply(this,arguments):undefined;
  setTimeout(function(){loadDefaultFromExport('postImport')},900);
  return r;
 };
 function scheduleLoads(reason){
  if(!isGlyphRoute())return;
  [60,280,900,1800].forEach(function(ms){setTimeout(function(){if(isGlyphRoute())loadDefaultFromExport(reason+'-'+ms)},ms)});
 }
 document.addEventListener('DOMContentLoaded',function(){scheduleLoads('dom')});
 window.addEventListener('load',function(){scheduleLoads('windowLoad')});
 document.addEventListener('click',function(ev){try{const a=ev.target&&ev.target.closest&&ev.target.closest('a[href],#navGlyphs');if(!a)return;const href=String(a.getAttribute&&a.getAttribute('href')||'').toLowerCase();if(a.id==='navGlyphs'||href.indexOf('glyph')>=0||href.indexOf('glifo')>=0)setTimeout(function(){scheduleLoads('nav')},80)}catch(e){}},true);
 // F49Z80: eliminado repaint periódico de Glifos; ahora solo se actualiza por navegación/importar/guardar.
})();



(function(){
 if(window.__glyphSdSingleSourceF49Z81)return;window.__glyphSdSingleSourceF49Z81=1;
 const BUILD='F49Z83';
 let applying=false;
 function isRoute(){try{return document.body&&document.body.classList.contains('route-glyphs')}catch(e){return false}}
 function hx(code){return (Number(code||0)&127).toString(16).toUpperCase().padStart(2,'0')}
 function norm(rows){try{if(typeof glyphNormalizeRowsV30==='function')return glyphNormalizeRowsV30(rows||[])}catch(e){} const out=[];(Array.isArray(rows)?rows:[]).slice(0,8).forEach(function(r){let s=String(r==null?'00':r).trim().replace(/^0x/i,'').toUpperCase().replace(/[^0-9A-F]/g,'');out.push((s||'00').padStart(2,'0').slice(-2));}); while(out.length<8)out.push('00');return out.slice(0,8);}
 function parseMapFromJson(txt){const map={};try{const j=JSON.parse(txt);(Array.isArray(j.fonts)?j.fonts:[]).forEach(function(f){(Array.isArray(f.glyphs)?f.glyphs:[]).forEach(function(g){let raw=(g.code!=null?g.code:(g.codeHex!=null?g.codeHex:g.id));let code=NaN;if(typeof raw==='number')code=raw&127;else{let s=String(raw==null?'':raw).trim();if(/^0x[0-9a-f]+$/i.test(s))code=parseInt(s,16)&127;else if(/^[0-9a-f]{1,2}$/i.test(s))code=parseInt(s,16)&127;else if(/^dec:\d+$/i.test(s))code=parseInt(s.slice(4),10)&127;} if(!Number.isFinite(code)||code<0||code>127)return;map[hx(code)]=norm(g.rowsHex||g.rows||[]);})});}catch(e){try{console.warn('F49Z80 parse glyph JSON',e)}catch(_){}} return map;}
 function mapToCompact(map){const lines=[];for(let code=0;code<128;code++){const h=hx(code);lines.push(h+':'+norm((map&&map[h])||[]).join(','));}return lines.join('\n');}
 function compactToMap(compact){const map={};String(compact||'').split(/\n+/).forEach(function(line){line=String(line||'').trim();if(!line)return;let sep=line.indexOf(':');if(sep<0)sep=line.indexOf('=');if(sep<=0)return;const h=String(line.slice(0,sep)).trim().replace(/^0x/i,'').toUpperCase().padStart(2,'0').slice(-2);const c=parseInt(h,16);if(!Number.isFinite(c)||c<0||c>127)return;map[hx(c)]=norm(line.slice(sep+1).split(','));});return map;}
 function setMap(map,label){map=map||{};const clean={};for(let code=0;code<128;code++){const h=hx(code);if(map[h])clean[h]=norm(map[h]);}window.__glyphSdMapF49Z80=clean;window.__glyphDefaultMapF49Z78=clean;window.__glyphProtectMapF49Z76=clean;window.__glyphProtectUntilF49Z76=Date.now()+1500;window.glyphRowsCacheV30={};try{for(let code=0;code<128;code++){const h=hx(code);if(clean[h]&&typeof glyphCacheRowsV30==='function')glyphCacheRowsV30(code,clean[h]);}}catch(e){} if(label){try{if(typeof glyphSetEditorStatus==='function')glyphSetEditorStatus(label+' · '+Object.keys(clean).length+'/128',Object.keys(clean).length>0)}catch(e){}try{if(typeof setStatus==='function')setStatus(label+' · '+Object.keys(clean).length+'/128',Object.keys(clean).length>0)}catch(e){}} return clean;}
 function applyMap(opts){if(applying)return 0;applying=true;try{const map=window.__glyphSdMapF49Z80||window.__glyphDefaultMapF49Z78||window.__glyphProtectMapF49Z76||{};const keys=Object.keys(map);if(!keys.length)return 0;if(!opts||opts.rebuild!==false){try{if(typeof __glyphOriginalBuildMatrixF49Z79==='function')__glyphOriginalBuildMatrixF49Z79(true);else if(typeof buildGlyphMatrixV27==='function')buildGlyphMatrixV27(true)}catch(e){}}keys.forEach(function(h){const code=parseInt(h,16)&127;try{if(typeof glyphCacheRowsV30==='function')glyphCacheRowsV30(code,map[h]);}catch(e){}try{if(typeof glyphUpdateMatrixCellV28==='function')glyphUpdateMatrixCellV28(code,map[h]);}catch(e){}});let code=0x41;try{code=(typeof glyphHexCodeValue==='function')?glyphHexCodeValue():0x41}catch(e){}code=Number(code);if(!Number.isFinite(code))code=0x41;code=code&127;const rows=map[hx(code)]||map[keys[0]];if(rows){try{if(typeof glyphRenderGrid==='function')glyphRenderGrid(8,8,rows)}catch(e){}try{if(typeof glyphSyncV27==='function')glyphSyncV27(code,{rowsHex:rows,source:'SD/RAM '+BUILD})}catch(e){}}return keys.length;}finally{applying=false;}}
 async function loadSd(reason){try{const r=await fetch('/glyphs_export?reload=1&default=1&source=sd&build=81&ts='+Date.now(),{cache:'no-store',headers:{'Cache-Control':'no-store'}});const txt=await r.text();if(!r.ok)throw new Error(txt||('HTTP '+r.status));const map=parseMapFromJson(txt);const count=Object.keys(map).length;if(count){setMap(map,'Glifos desde SD/RAM');applyMap({rebuild:true});}return count;}catch(e){try{console.warn('F49Z80 loadSd '+reason,e)}catch(_){}return 0;}}
 window.glyphLoadSdDefaultF49Z80=loadSd;
 const oldRows=window.glyphRowsForCodeV27;window.glyphRowsForCodeV27=function(code){const map=window.__glyphSdMapF49Z80||window.__glyphDefaultMapF49Z78||window.__glyphProtectMapF49Z76||{};const rows=map[hx(code)];if(rows&&rows.length)return rows.slice();return(typeof oldRows==='function')?oldRows.apply(this,arguments):norm([]);};
 const oldBuild=window.buildGlyphMatrixV27;window.__glyphOriginalBuildMatrixF49Z79=oldBuild;if(typeof oldBuild==='function')window.buildGlyphMatrixV27=function(){const r=oldBuild.apply(this,arguments);setTimeout(function(){applyMap({rebuild:false})},0);return r;};
 window.refreshGlyphMatrixFromDeviceV30=async function(){const n=await loadSd('refresh');if(n)return n;return applyMap({rebuild:true});};window.glyphReloadDefaultFromMasterF49Z74=window.refreshGlyphMatrixFromDeviceV30;
 window.glyphLoadCurrent=async function(){const map=window.__glyphSdMapF49Z80||{};let code=0x41;try{code=(typeof glyphHexCodeValue==='function')?glyphHexCodeValue():0x41}catch(e){}code=Number(code);if(!Number.isFinite(code))code=0x41;code=code&127;let rows=map[hx(code)];if(!rows){await loadSd('loadCurrent');rows=(window.__glyphSdMapF49Z80||{})[hx(code)];}if(rows){try{if(typeof glyphCacheRowsV30==='function')glyphCacheRowsV30(code,rows)}catch(e){}try{if(typeof glyphRenderGrid==='function')glyphRenderGrid(8,8,rows)}catch(e){}try{if(typeof glyphSyncV27==='function')glyphSyncV27(code,{rowsHex:rows,source:'SD/RAM '+BUILD})}catch(e){}try{if(typeof glyphSetEditorStatus==='function')glyphSetEditorStatus('Glifo desde SD/RAM · 0x'+hx(code),true)}catch(e){}return;}};
 window.importGlyphs=async function(){try{const input=document.getElementById('glyphFile');const f=input&&input.files&&input.files[0];if(!f)throw new Error('Selecciona atascii_glyphs.json primero');const txt=await f.text();const compact=(typeof compactGlyphImportFromText==='function')?compactGlyphImportFromText(txt):mapToCompact(parseMapFromJson(txt));const map=compactToMap(compact);const count=Object.keys(map).length;if(!count)throw new Error('No se encontraron glifos válidos');setMap(map,'Importado activo');applyMap({rebuild:true});const r=await fetch('/glyphs_import_compact?build=81&ts='+Date.now(),{method:'POST',headers:{'Content-Type':'text/plain; charset=utf-8','Cache-Control':'no-store'},body:compact});const body=await r.text();if(!r.ok)throw new Error(body||('HTTP '+r.status));if(input)input.value='';await loadSd('postImport');try{if(typeof setStatus==='function')setStatus('Glifos importados, guardados en SD y aplicados: '+count+'/128 · '+body,true)}catch(e){}}catch(e){try{if(typeof setStatus==='function')setStatus('Error importando glifos: '+(e.message||e),false)}catch(_){}try{console.error(e)}catch(_){}}};
 window.glyphSaveAllCurrentV34=async function(){try{const map=Object.assign({},window.__glyphSdMapF49Z80||window.__glyphDefaultMapF49Z78||{});let code=0x41;try{code=(typeof glyphHexCodeValue==='function')?glyphHexCodeValue():0x41}catch(e){}code=Number(code);if(!Number.isFinite(code))code=0x41;code=code&127;try{if(typeof glyphRowsFromGrid==='function')map[hx(code)]=norm(glyphRowsFromGrid())}catch(e){}for(let c=0;c<128;c++){const h=hx(c);if(!map[h])map[h]=norm((typeof glyphRowsForCodeV27==='function')?glyphRowsForCodeV27(c):[]);}setMap(map,'Guardando en SD');const compact=mapToCompact(map);const r=await fetch('/glyphs_import_compact?build=81&ts='+Date.now(),{method:'POST',headers:{'Content-Type':'text/plain; charset=utf-8','Cache-Control':'no-store'},body:compact});const body=await r.text();if(!r.ok)throw new Error(body||('HTTP '+r.status));await loadSd('postSave');applyMap({rebuild:true});try{if(typeof setStatus==='function')setStatus('Glifos guardados como default en SD: '+body,true)}catch(e){}}catch(e){try{if(typeof setStatus==='function')setStatus('Error guardando glifos: '+(e.message||e),false)}catch(_){}try{console.error(e)}catch(_){}}};
 function schedule(){if(!isRoute())return;[50,250,900].forEach(function(ms){setTimeout(function(){if(isRoute())loadSd('schedule'+ms)},ms)});}document.addEventListener('DOMContentLoaded',schedule);window.addEventListener('load',schedule);document.addEventListener('click',function(ev){try{const a=ev.target&&ev.target.closest&&ev.target.closest('a[href],#navGlyphs');if(!a)return;const href=String(a.getAttribute&&a.getAttribute('href')||'').toLowerCase();if(a.id==='navGlyphs'||href.indexOf('glyph')>=0||href.indexOf('glifo')>=0)setTimeout(schedule,80);}catch(e){}},true);// F49Z81: sin setInterval y con código 0x00 preservado; evita pisar 0x00 con 0x41.
})();



(function(){
 if(window.__uiPerfFixF49Z80)return; window.__uiPerfFixF49Z80=1;
 try{console.info('F31: Glifos SD/RAM activo; cache HTML limpia.');}catch(e){}
})();



(function(){
  if(window.__libraryFinalControllerF22)return;
  window.__libraryFinalControllerF22=1;
  window.__libraryFinalControllerF21=1;
  const TAG='F31_LIBRARY_LEGACY_NEUTRALIZED_FIX';
  const TYPES=['ALL','ATR','XEX','COM','EXE','BAS','CAS','SEC','OTHER'];
  const MOUNT_TYPES=['ATR','XEX','COM','EXE','SEC'];
  const state={files:[],payload:null,page:0,pageSize:24,type:'ALL',query:'',sort:'name',refreshing:false,timer:null,lastJsonOk:false,lastJsonCount:0,visibleMask:15,statusMask:0,statusForceMask:0,coverQueue:[],coverActive:0,rendering:false};
  const $=id=>document.getElementById(id);
  const esc=v=>String(v==null?'':v).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  const enc=v=>encodeURIComponent(String(v==null?'':v));
  const isLibrary=()=>!!(document.body&&document.body.classList&&document.body.classList.contains('route-library'))||location.pathname.indexOf('/library')===0||location.pathname.indexOf('/upload')===0||location.pathname.indexOf('/editor/library')===0;
  const isAtr=()=>!!(document.body&&document.body.classList&&document.body.classList.contains('route-webatr'))||location.pathname.indexOf('/atr')===0||location.pathname.indexOf('/webatr')===0||location.pathname.indexOf('/editor/web-atr')===0||location.pathname.indexOf('/editor/webatr')===0;
  function base(p){p=String(p||'').split(String.fromCharCode(92)).join('/'); const i=p.lastIndexOf('/'); return i>=0?p.slice(i+1):p;}
  function noExt(n){n=String(n||''); const i=n.lastIndexOf('.'); return i>0?n.slice(0,i):n;}
  function normKey(v){return base(String(v||'')).trim().toLowerCase();}
  function extOf(n){n=base(n); const i=n.lastIndexOf('.'); return i>=0?n.slice(i+1).toUpperCase():'';}
  function typeOf(f){let src=String((f&&f.name)||base(f&&f.path)||f||''); let t=extOf(src)||String((f&&f.type)||'').replace('.','').toUpperCase(); return TYPES.includes(t)?t:'OTHER';}
  function norm(f){f=f||{}; const p=String(f.path||f.fullPath||f.file||'').trim(); const n=String(f.name||f.filename||base(p)||'').trim(); return {raw:f,name:n,path:p||n,type:typeOf({name:n,path:p,type:f.type}),valid:(f.valid==null?null:Number(f.valid||0)),fileSize:Number(f.fileSize!=null?f.fileSize:(f.size||0))||0,sectorSize:Number(f.sectorSize||0)||0,totalSectors:Number(f.totalSectors||0)||0,mountedMask:Number(f.mountedMask||0)||0,casMounted:Number(f.casMounted||0)||0,protected:!!f.protected,pendingCas:false,pendingMount:0};}
  function fileKey(f){return f.path||f.name||'';}
  function sameFile(f,key){const k=String(key||'').toLowerCase(); const bk=normKey(k); if(!f||!k)return false; return String(f.path||'').toLowerCase()===k||String(f.name||'').toLowerCase()===k||normKey(f.path)===bk||normKey(f.name)===bk;}
  function sorter(a,b){if(state.sort==='type'){const c=typeOf(a).localeCompare(typeOf(b),'es'); if(c)return c;} if(state.sort==='size'){const c=(b.fileSize||0)-(a.fileSize||0); if(c)return c;} return String(a.name||'').localeCompare(String(b.name||''),'es',{numeric:true,sensitivity:'base'});}
  function setFiles(files,payload){const map=new Map(); (Array.isArray(files)?files:[]).map(norm).filter(f=>f.name).forEach(f=>map.set(String(f.path||f.name).toLowerCase(),f)); const old=state.files||[]; state.files=Array.from(map.values()).map(f=>{const prev=old.find(o=>sameFile(o,f.path)||sameFile(o,f.name)); if(prev){f.casMounted=prev.casMounted||f.casMounted; f.pendingCas=prev.pendingCas||false; f.pendingMount=prev.pendingMount||0; f.mountedMask=Number(prev.mountedMask||0)||Number(f.mountedMask||0);} return f;}).sort(sorter); state.payload=payload||state.payload; if(payload&&payload.driveVisibleMask!=null)state.visibleMask=Number(payload.driveVisibleMask)||15; try{window.libraryFilesF22=state.files.slice(); window.libraryFilesF21=state.files.slice(); window.libraryFilesF20=state.files.slice(); window.atrLibraryFilesCache=state.files.slice(); if(typeof atrLibraryFilesCache!=='undefined')atrLibraryFilesCache=state.files.slice();}catch(e){} return state.files;}
  function counts(list){const c={ALL:list.length,ATR:0,XEX:0,COM:0,EXE:0,BAS:0,CAS:0,SEC:0,OTHER:0}; list.forEach(f=>{const t=typeOf(f); c[t]=(c[t]||0)+1;}); return c;}
  function filtered(){let arr=state.files.slice(); const q=String(state.query||'').trim().toLowerCase(); if(state.type&&state.type!=='ALL')arr=arr.filter(f=>typeOf(f)===state.type); if(q)arr=arr.filter(f=>String(f.name||'').toLowerCase().includes(q)||String(f.path||'').toLowerCase().includes(q)||typeOf(f).toLowerCase().includes(q)); return arr.sort(sorter);}
  function human(n){n=Number(n||0); if(!n)return '—'; if(n<1024)return n+' B'; if(n<1048576)return (n/1024).toFixed(n<10240?1:0)+' KB'; return (n/1048576).toFixed(1)+' MB';}
  function maskToUnitsLocal(mask){mask=Number(mask||0); const a=[]; for(let i=1;i<=7;i++)if(mask&(1<<(i-1)))a.push('D'+i); return a.length?a.join('+'):'—';}
  function isWebAtrMountableFile(f){const t=typeOf(f); const valid=(f&&f.valid==null)?(t==='ATR'):(Number(f&&f.valid||0)!==0); return t==='ATR' ? valid : (MOUNT_TYPES.includes(t)&&valid&&!!(Number(f.sectorSize||0)||Number(f.totalSectors||0)));}
  function visibleUnits(){let units=[]; let mask=Number(state.visibleMask||15); for(let i=1;i<=7;i++){if(mask & (1<<(i-1)))units.push(i);} if(!units.length)units=[1,2,3,4]; return units;}
  function coverUrl(name){try{if(typeof atrCoverThumbNameSrc==='function')return atrCoverThumbNameSrc(name);}catch(e){} return '/api/covers/thumb?name='+enc(name);}
  function coverFullUrl(name){try{if(typeof atrCoverProxyNameSrc==='function')return atrCoverProxyNameSrc(name);}catch(e){} return '/api/covers/proxy?name='+enc(name);}
  function cover(name){const safe=esc(name||''); return '<img class="atr-lib-cover f22-lazy-cover" loading="lazy" decoding="async" fetchpriority="low" alt="Carátula" data-f22-cover="1" data-src="'+esc(coverUrl(name))+'" data-full="'+esc(coverFullUrl(name))+'" data-title="'+safe+'"><div class="atr-lib-cover-box f22-cover-miss" style="display:none">Sin<br>carátula</div>';}
  function unitsHtml(f){const t=typeOf(f); const key=fileKey(f); let html='';
    if(t==='CAS'){
      if(f.pendingCas) html='<span class="mounted-badge mock-mounted-badge">Preparando C:...</span>';
      else if(f.casMounted) html='<span class="mounted-badge mock-mounted-badge">Preparado: C:</span><a class="mock-unit-btn" href="/cassette">Ver cassette</a>';
      else html='<button type="button" class="mock-unit-btn" data-f22-action="cas" data-file="'+esc(key)+'">Preparar C:</button>';
    } else if(isWebAtrMountableFile(f)){
      if(f.pendingMount) html='<span class="mounted-badge mock-mounted-badge">Montando D'+Number(f.pendingMount)+'...</span>';
      else if(f.mountedMask) html='<span class="mounted-badge mock-mounted-badge">Montado: '+esc(maskToUnitsLocal(f.mountedMask))+'</span>';
      else visibleUnits().forEach(i=>{html+='<button type="button" class="mock-unit-btn" data-f22-action="mount" data-unit="'+i+'" data-file="'+esc(key)+'">D'+i+'</button>';});
    } else html='<span class="pill">Solo listado</span>';
    if(!f.protected && !f.mountedMask && !f.casMounted && !f.pendingCas && !f.pendingMount)html+='<button type="button" class="mock-unit-btn trash-btn" data-f22-action="delete" data-file="'+esc(key)+'">🗑</button>';
    return html;
  }
  function card(f){const t=typeOf(f); const sub=esc(f.path&&f.path!==f.name?f.path:noExt(f.name)); return '<article class="mock-lib-card" data-file-name="'+esc(f.name)+'" data-file-path="'+esc(f.path)+'" data-file-type="'+esc(t)+'"><div class="mock-lib-cover-wrap"><span class="mock-lib-type">'+esc(t)+'</span>'+cover(f.name)+'</div><div class="mock-lib-title" title="'+esc(f.name)+'">'+esc(f.name)+'</div><div class="mock-lib-subtitle" title="'+sub+'">'+sub+'</div><div class="mock-lib-meta"><span>'+human(f.fileSize)+'</span><span>'+(t==='CAS'?'Cassette':(f.sectorSize?esc(f.sectorSize+'B'):'—'))+'</span><span>'+(f.totalSectors?esc(f.totalSectors+' sec.'):'—')+'</span>'+(f.protected?'<span>Protegido</span>':'')+'</div><div class="mock-lib-units">'+unitsHtml(f)+'</div></article>';}
  function ensureStyle(){if($('libraryF22Style'))return; const st=document.createElement('style'); st.id='libraryF22Style'; st.textContent='#atrLibrary,#atrLibraryPager,body.route-library #mockFilterChipsF27,body.route-library #libraryGridF19,body.route-library #libraryGridF20,body.route-library #libraryGridF21,body.route-library #libraryF19Pager,body.route-library #libraryF20Pager,body.route-library #libraryF21Pager,body.route-library #libraryF19Chips,body.route-library #libraryF20Chips,body.route-library #libraryF21Chips,body.route-library #libraryF19Diag,body.route-library #libraryF20Diag,body.route-library #libraryF21Diag{display:none!important} #libraryF22Chips{display:flex;flex-wrap:wrap;gap:8px;margin:10px 0} #libraryF22Pager{display:flex;flex-wrap:wrap;gap:10px;align-items:center;justify-content:center;margin:12px 0} #libraryGridF22.mock-lib-grid{display:grid!important;grid-template-columns:repeat(auto-fill,minmax(var(--lib-card-min,220px),1fr));gap:16px!important;margin-top:10px} #libraryF22Diag{font-size:12px;color:#9fb6c8;margin:6px 0 0}.f22-lazy-cover[data-loaded="1"]{display:block}.f22-lazy-cover[data-error="1"]{display:none}.mock-filter-chip[hidden]{display:none!important}'; document.head.appendChild(st);}
  function ensureUi(){ensureStyle(); const block=$('atrLibraryPageBlock'); if(!block)return; ['atrLibrary','atrLibraryPager','libraryGridF19','libraryGridF20','libraryGridF21','libraryF19Pager','libraryF20Pager','libraryF21Pager'].forEach(id=>{const el=$(id); if(el){try{el.innerHTML=''; el.remove();}catch(e){el.style.display='none';}}}); let chips=$('libraryF22Chips'); if(!chips){chips=document.createElement('div'); chips.id='libraryF22Chips'; chips.className='mock-filter-chips'; const row=block.querySelector('.library-refresh-row')||block.querySelector('.library-toolbar-main')||block.firstChild; if(row&&row.parentNode)row.parentNode.insertBefore(chips,row.nextSibling); else block.appendChild(chips);} let diag=$('libraryF22Diag'); if(!diag){diag=document.createElement('div'); diag.id='libraryF22Diag'; chips.parentNode.insertBefore(diag,chips.nextSibling);} let sort=$('atrLibrarySortF22'); if(!sort){const lab=document.createElement('label'); lab.className='label'; lab.style.cssText='display:flex;align-items:center;gap:6px;'; lab.innerHTML='Orden <select id="atrLibrarySortF22" class="small-input"><option value="name">Nombre</option><option value="type">Tipo</option><option value="size">Tamaño</option></select>'; const tb=block.querySelector('.library-toolbar-main')||block; tb.appendChild(lab);} let pager=$('libraryF22Pager'); if(!pager){pager=document.createElement('div'); pager.id='libraryF22Pager'; pager.className='library-pager'; const oldPager=$('libraryF21Pager')||$('libraryF20Pager')||$('libraryF19Pager')||$('atrLibraryPager'); if(oldPager&&oldPager.parentNode)oldPager.parentNode.insertBefore(pager,oldPager.nextSibling); else block.appendChild(pager);} let grid=$('libraryGridF22'); if(!grid){grid=document.createElement('div'); grid.id='libraryGridF22'; grid.className='mock-lib-grid'; const old=$('libraryGridF21')||$('libraryGridF20')||$('libraryGridF19')||$('atrLibrary'); if(old&&old.parentNode)old.parentNode.insertBefore(grid,old); else block.appendChild(grid);} setCoverDefaultAll();}
  function setCoverDefaultAll(){try{localStorage.setItem('atrCoverModeV29Fix3','all'); localStorage.setItem('atrCoverModeV29','all'); localStorage.setItem('atrCoverModeF22Default','all'); document.body.classList.remove('cover-mode-off'); ['atrCoverMode','atrCoverModeAtr'].forEach(id=>{const el=$(id); if(el)el.value='all';}); ['atrCoverPerfState','atrCoverPerfStateAtr'].forEach(id=>{const el=$(id); if(el)el.textContent='Todas lentamente';});}catch(e){}}
  function updateCounts(c){if(state.type!=='ALL' && !Number(c[state.type]||0)){state.type='ALL'; state.page=0;} ['atrLibraryCount','atrUploadLibraryCount','atrMountLibraryCount','homeLibraryCount'].forEach(id=>{const el=$(id); if(el)el.textContent=state.files.length+' archivo'+(state.files.length===1?'':'s')}); const chips=$('libraryF22Chips'); if(chips){chips.innerHTML=TYPES.filter(t=>t==='ALL'||Number(c[t]||0)>0).map(t=>'<button type="button" class="mock-filter-chip '+(state.type===t?'active':'')+'" data-f22-filter="'+t+'">'+(t==='ALL'?'Todos':t)+' <span>'+Number(c[t]||0)+'</span></button>').join('');}}
  function render(reason){if(!isLibrary())return; ensureUi(); state.rendering=true; try{ const search=$('atrLibrarySearch'); if(search)state.query=search.value||''; const sort=$('atrLibrarySortF22'); if(sort)state.sort=sort.value||state.sort; const grid=$('libraryGridF22'); if(!grid)return; const c=counts(state.files); updateCounts(c); const list=filtered(); const total=list.length; const pages=Math.max(1,Math.ceil(total/state.pageSize)); if(state.page>=pages)state.page=pages-1; if(state.page<0)state.page=0; const start=state.page*state.pageSize; const pageList=list.slice(start,start+state.pageSize); const summary=$('atrLibrarySummary'); if(summary)summary.textContent=(state.files.length?total+' de '+state.files.length+' archivos':'Sin archivos')+' · ATR '+c.ATR+' · XEX '+c.XEX+' · COM '+c.COM+' · EXE '+c.EXE+' · BAS '+c.BAS+' · CAS '+c.CAS+' · unidades visibles '+visibleUnits().map(i=>'D'+i).join(', ')+' · página '+(state.page+1)+'/'+pages; const diag=$('libraryF22Diag'); if(diag)diag.textContent='Render F30 · '+(reason||'')+' · WEB-ATR sync mask=0x'+Number(state.statusMask||0).toString(16).toUpperCase()+' · JSON SD '+(state.lastJsonOk?'OK '+state.lastJsonCount+'/'+(state.payload&&state.payload.indexCount||state.files.length):'pendiente'); const pager=$('libraryF22Pager'); if(pager){pager.innerHTML='<button id="btnLibraryPrevF22" type="button" class="secondary" '+(state.page<=0?'disabled':'')+'>◀ Anterior</button><span class="library-page-info">Página '+(state.page+1)+' / '+pages+' · '+total+' resultado'+(total===1?'':'s')+'</span><button id="btnLibraryNextF22" type="button" class="secondary" '+(state.page>=pages-1?'disabled':'')+'>Siguiente ▶</button><label class="library-page-size">Por página <select id="atrLibraryPageSizeF22" class="small-input"><option value="12">12</option><option value="24">24</option><option value="48">48</option><option value="96">96</option></select></label>'; const ps=$('atrLibraryPageSizeF22'); if(ps)ps.value=String(state.pageSize);} if(!state.files.length)grid.innerHTML='<div class="mock-lib-empty">Sin archivos cargados. Presiona Refrescar biblioteca para construir el índice SD.</div>'; else if(!pageList.length)grid.innerHTML='<div class="mock-lib-empty">Sin resultados para la búsqueda/filtro actual.</div>'; else grid.innerHTML=pageList.map(f=>{try{return card(f)}catch(e){return '<article class="mock-lib-card"><b>'+esc(f&&f.name||'archivo')+'</b><small>Error tarjeta: '+esc(e&&e.message?e.message:e)+'</small></article>'}}).join(''); loadCovers(); populateWebAtr(); } finally {state.rendering=false;} }
  function updateVisibleCardUnits(){const grid=$('libraryGridF22'); if(!grid)return; grid.querySelectorAll('.mock-lib-card').forEach(card=>{const key=card.getAttribute('data-file-path')||card.getAttribute('data-file-name')||''; const f=state.files.find(x=>sameFile(x,key)); const units=card.querySelector('.mock-lib-units'); if(f&&units)units.innerHTML=unitsHtml(f); card.classList.toggle('mounted',!!(f&&(f.mountedMask||f.casMounted)));}); populateWebAtr();}
  function populateWebAtr(){const list=state.files.filter(isWebAtrMountableFile).sort(sorter); const c=counts(list); let html='<option value="">— sin montar — ('+list.length+' archivos)</option><option value="" disabled>Total: '+list.length+' · ATR: '+c.ATR+' · XEX: '+c.XEX+' · COM: '+c.COM+' · EXE: '+c.EXE+'</option>'; list.forEach(f=>html+='<option value="'+esc(fileKey(f))+'">['+esc(typeOf(f))+'] '+esc(f.name)+'</option>'); for(let i=1;i<=7;i++){const sel=$('atrSel'+i); if(sel){const val=sel.value||''; sel.innerHTML=html; if(val)sel.value=val;}}}
  function loadCovers(){setCoverDefaultAll(); const grid=$('libraryGridF22'); if(!grid)return; const imgs=Array.from(grid.querySelectorAll('img.f22-lazy-cover[data-src]:not([data-loaded="1"]):not([data-error="1"])')); imgs.forEach(img=>{if(!img.dataset.queued){img.dataset.queued='1'; state.coverQueue.push(img);}}); pumpCovers();}
  function pumpCovers(){while(state.coverActive<3&&state.coverQueue.length){const img=state.coverQueue.shift(); if(!img||!document.body.contains(img))continue; state.coverActive++; const done=(ok)=>{state.coverActive=Math.max(0,state.coverActive-1); if(ok){img.dataset.loaded='1'; try{if(typeof atrEnsureMiniCoverFromImage==='function')setTimeout(()=>atrEnsureMiniCoverFromImage(img,img.dataset.title||''),1200)}catch(e){}} else {img.dataset.error='1'; img.style.display='none'; const n=img.nextElementSibling; if(n)n.style.display='flex';} setTimeout(pumpCovers,180);}; img.onload=()=>done(true); img.onerror=()=>done(false); img.src=img.dataset.src;}}
  function xhrJson(url){return new Promise((resolve,reject)=>{const x=new XMLHttpRequest(); x.open('GET',url,true); x.setRequestHeader('Cache-Control','no-store'); x.onreadystatechange=function(){if(x.readyState!==4)return; let p=null; try{p=JSON.parse(x.responseText||'{}')}catch(e){return reject(new Error('Respuesta JSON inválida en '+url))} if(x.status<200||x.status>=300||(p&&p.ok===false))return reject(new Error((p&&p.error)||('HTTP '+x.status))); resolve(p);}; x.onerror=()=>reject(new Error('Error de red en '+url)); x.send();});}
  async function refreshVisibleMask(){try{const v=await xhrJson('/api/drives/visible?_='+Date.now()); if(v&&v.visibleMask!=null)state.visibleMask=Number(v.visibleMask)||15;}catch(e){}}
  function syncFromAtrStatus(st){if(!st||!Array.isArray(st.slots))return false; if(st.driveVisibleMask!=null)state.visibleMask=Number(st.driveVisibleMask)||state.visibleMask; if(st.mask!=null)state.statusMask=Number(st.mask)||0; if(st.forceMask!=null)state.statusForceMask=Number(st.forceMask)||0; state.files.forEach(f=>{if(MOUNT_TYPES.includes(typeOf(f))){f.mountedMask=0; f.pendingMount=0;}}); st.slots.forEach(sl=>{const unit=Number(sl.unit||0); if(unit<1||unit>7)return; const bit=1<<(unit-1); const nm=String(sl.name||''); if(!nm)return; const active=(Number(sl.enabled||0)||Number(sl.forced||0)||Number(sl.present||0)); if(!active)return; const f=state.files.find(x=>sameFile(x,nm)); if(f)f.mountedMask=Number(f.mountedMask||0)|bit;}); return true;}
  async function syncDriveState(){try{const st=await xhrJson('/api/atr/status?files=0&fast=1&_='+Date.now()); if(syncFromAtrStatus(st)){try{window.lastAtrStatus=Object.assign(window.lastAtrStatus||{},st);}catch(e){} updateVisibleCardUnits(); return st;}}catch(e){console.warn(TAG,'syncDriveState',e&&e.message?e.message:e);} return null;}
  async function readIndexFallback(expected){
  try{
    const txt = await new Promise((resolve,reject)=>{
      const x=new XMLHttpRequest();
      x.open('GET','/api/library_index_json?_='+Date.now(),true);
      x.setRequestHeader('Cache-Control','no-store');
      x.onreadystatechange=function(){
        if(x.readyState!==4)return;
        if(x.status>=200&&x.status<300)resolve(String(x.responseText||''));
        else reject(new Error('HTTP '+x.status+' '+String(x.responseText||'').slice(0,120)));
      };
      x.onerror=function(){reject(new Error('network'))};
      x.send();
    });
    let raw=String(txt||'').trim();
    let idx=null;
    if(raw){
      if(raw[0]==='[') idx=JSON.parse(raw);
      else {
        const p=JSON.parse(raw);
        if(Array.isArray(p)) idx=p;
        else if(Array.isArray(p.files)) idx=p.files;
        else if(Array.isArray(p.index)) idx=p.index;
        else if(typeof p.raw==='string') idx=JSON.parse(p.raw);
      }
    }
    if(Array.isArray(idx)){
      state.lastJsonOk=(idx.length===Number(expected||idx.length));
      state.lastJsonCount=idx.length;
      return idx;
    }
    throw new Error('Respuesta JSON válida pero no es arreglo de archivos');
  }catch(e){
    state.lastJsonOk=false;
    console.warn(TAG,'library_index_json no usable; se mantiene files[] de /api/library',e&&e.message?e.message:e);
  }
  return null;
}
  function logIndex(payload,idx){const expected=Number(payload&&payload.indexCount||payload&&payload.total||state.files.length||0); console.groupCollapsed('[Biblioteca F30] /CONFIG/library_index.json después de Refrescar'); console.log('Respuesta /api/library?refresh=1:',payload); console.log('Archivo JSON leído desde SD:',idx); console.log('Verificación cliente:',{jsonReadOk:Array.isArray(idx),jsonCount:Array.isArray(idx)?idx.length:0,expected,match:Array.isArray(idx)&&idx.length===expected}); console.groupEnd();}
  function overlay(){let o=$('libraryRefreshFloatF22'); if(o)return o; o=document.createElement('div'); o.id='libraryRefreshFloatF22'; o.style.cssText='position:fixed;right:18px;bottom:18px;z-index:100000;width:min(390px,calc(100vw - 36px));padding:14px;border:1px solid rgba(56,189,248,.38);border-radius:18px;background:rgba(2,6,23,.97);box-shadow:0 20px 60px rgba(0,0,0,.45);color:#e5f6ff;font:13px system-ui,sans-serif;display:none'; o.innerHTML='<b>Actualizando biblioteca SD...</b><div id="libraryRefreshMsgF22" style="margin-top:6px;color:#a9c6d8">Escaneando...</div><div style="height:10px;border-radius:999px;background:rgba(255,255,255,.12);overflow:hidden;margin-top:10px"><div id="libraryRefreshBarF22" style="height:100%;width:4%;background:linear-gradient(90deg,#38bdf8,#22c55e);transition:width .18s ease"></div></div><div id="libraryRefreshTimeF22" style="margin-top:6px;color:#8fb2c8;font-size:12px">0s</div>'; document.body.appendChild(o); return o;}
  function progress(on,msg){const o=overlay(),bar=$('libraryRefreshBarF22'),m=$('libraryRefreshMsgF22'),t=$('libraryRefreshTimeF22'); if(!on){clearInterval(state.timer); if(bar)bar.style.width='100%'; if(m)m.textContent=msg||'Listo'; if(t)t.textContent='Ver consola para detalle'; setTimeout(()=>o.style.display='none',4200); return;} o.style.display='block'; let pct=4,start=Date.now(); if(bar)bar.style.width='4%'; if(m)m.textContent='Escaneando SD y reconstruyendo índice'; clearInterval(state.timer); state.timer=setInterval(()=>{pct=Math.min(94,pct+(pct<60?3:1)); if(bar)bar.style.width=pct+'%'; if(t)t.textContent='Tiempo transcurrido: '+Math.round((Date.now()-start)/1000)+'s';},650);}
  async function load(refresh){if(refresh&&state.refreshing)return; ensureUi(); if(refresh){state.refreshing=true; progress(true); const b=$('btnLibraryRefresh'); if(b){b.disabled=true;b.textContent='⏳ Actualizando...';}} try{await refreshVisibleMask(); const p=await xhrJson(refresh?'/api/library?refresh=1':'/api/library?refresh=0'); if(p&&p.driveVisibleMask!=null)state.visibleMask=Number(p.driveVisibleMask)||state.visibleMask; let files=Array.isArray(p.files)?p.files:[]; let idx=null; if((!files.length)&&Number(p.indexCount||p.total||0)>0){idx=await readIndexFallback(p.indexCount||p.total); if(Array.isArray(idx)&&idx.length)files=idx;} else if(refresh){idx=await readIndexFallback(p.indexCount||p.total||files.length);} setFiles(files,p); await syncDriveState(); state.page=0; render(refresh?'manual-refresh':'load-index'); if(refresh){await logIndex(p,idx); const ok=Array.isArray(idx)&&idx.length===Number(p.indexCount||files.length); progress(false,'Biblioteca actualizada: '+state.files.length+' archivo(s). '+(ok?'JSON leído OK desde SD.':'Listado cargado; revisar JSON en consola.'));} return p;}catch(e){console.warn(TAG,e&&e.message?e.message:e); if(refresh){progress(false,'Error: '+(e&&e.message?e.message:e)); alert('Error actualizando biblioteca: '+(e&&e.message?e.message:e));} if(state.files.length)render('error-cache'); return null;}finally{if(refresh){state.refreshing=false; const b=$('btnLibraryRefresh'); if(b){b.disabled=false;b.textContent='🔄 Refrescar biblioteca';}}}}
  async function prepareCas(file){const f=state.files.find(x=>sameFile(x,file)); if(!f)return; f.pendingCas=true; updateVisibleCardUnits(); try{const res=await xhrJson('/cas/mount?file='+enc(file)+'&fast=1'); state.files.forEach(x=>{if(typeOf(x)==='CAS'){x.casMounted=0;x.pendingCas=false;}}); const fx=state.files.find(x=>sameFile(x,file)); if(fx){fx.casMounted=1;fx.pendingCas=false;} try{if(typeof updateCasManualPanelF49==='function')updateCasManualPanelF49(Object.assign({casMounted:1,casPlaying:0,casName:base(file),name:base(file)},res||{}));}catch(e){} try{window.casLastStatusF49J=Object.assign(window.casLastStatusF49J||{},{casMounted:1,mounted:1,casPlaying:0,playing:0,casName:base(file),name:base(file)});}catch(e){} try{if(typeof setStatus==='function')setStatus('C: preparado: '+base(file),true);}catch(e){} updateVisibleCardUnits();}catch(e){const fx=state.files.find(x=>sameFile(x,file)); if(fx)fx.pendingCas=false; updateVisibleCardUnits(); alert('No se pudo preparar el cassette: '+(e&&e.message?e.message:e));}}
  async function mountFile(file,unit){const bit=1<<(Number(unit)-1); const f=state.files.find(x=>sameFile(x,file)); if(f){f.pendingMount=unit; updateVisibleCardUnits();} try{await syncDriveState(); const curMask=Number(state.statusMask||0)||0; const curForce=Number(state.statusForceMask||0)||0; const params=new URLSearchParams(); params.set('en','1'); params.set('mask',String(curMask|bit)); params.set('force',String(curForce|bit)); params.set('unit',String(unit)); params.set('file',String(file||'')); params.set('reply','light'); const a=await xhrJson('/set_webatr?'+params.toString()); if(!syncFromAtrStatus(a)){state.files.forEach(x=>{x.pendingMount=0; if(MOUNT_TYPES.includes(typeOf(x)))x.mountedMask=Number(x.mountedMask||0)&(~bit);}); const fx=state.files.find(x=>sameFile(x,file)); if(fx)fx.mountedMask=Number(fx.mountedMask||0)|bit; state.statusMask=curMask|bit; state.statusForceMask=curForce|bit;} try{window.lastAtrStatus=Object.assign(window.lastAtrStatus||{},a||{});}catch(e){} try{if(typeof atrApplyLightStatusFast==='function')atrApplyLightStatusFast(a,{forceCovers:false});}catch(e){} try{if(typeof atrSetAutoSaveState==='function')atrSetAutoSaveState('Guardado','ok');}catch(e){} try{if(typeof setStatus==='function')setStatus('Montado en D'+unit+': '+base(file),true);}catch(e){} updateVisibleCardUnits();}catch(e){state.files.forEach(x=>x.pendingMount=0); updateVisibleCardUnits(); alert('No se pudo montar D'+unit+': '+(e&&e.message?e.message:e));}}
  function bind(){ensureUi(); const old=$('btnLibraryRefresh'); if(old&&!old.dataset.f22Ready){const b=old.cloneNode(true); b.dataset.f22Ready='1'; b.textContent='🔄 Refrescar biblioteca'; old.replaceWith(b); b.addEventListener('click',ev=>{ev.preventDefault();ev.stopPropagation();ev.stopImmediatePropagation();load(true);return false;},true);} const s=$('atrLibrarySearch'); if(s&&!s.dataset.f22Ready){s.dataset.f22Ready='1'; s.addEventListener('input',ev=>{ev.stopPropagation();state.query=s.value||'';state.page=0;render('search');},true);} const sort=$('atrLibrarySortF22'); if(sort&&!sort.dataset.f22Ready){sort.dataset.f22Ready='1'; sort.addEventListener('change',ev=>{ev.stopPropagation();state.sort=sort.value||'name';state.page=0;render('sort');},true);} ['atrCoverMode','atrCoverModeAtr'].forEach(id=>{const el=$(id); if(el&&!el.dataset.f22Default){el.dataset.f22Default='1'; el.value='all'; el.addEventListener('change',()=>{try{localStorage.setItem('atrCoverModeV29Fix3',el.value);localStorage.setItem('atrCoverModeV29',el.value);}catch(e){}},true);}});}
  document.addEventListener('click',ev=>{const t=ev.target&&ev.target.closest?ev.target.closest('[data-f22-cover],[data-f22-filter],#btnLibraryPrevF22,#btnLibraryNextF22,[data-f22-action]'):null; if(!t)return; ev.preventDefault(); ev.stopPropagation(); ev.stopImmediatePropagation(); if(t.dataset.f22Cover){try{atrOpenCoverFullscreen(t.dataset.full||t.src,t.dataset.title||'')}catch(e){} return false;} if(t.dataset.f22Filter){state.type=String(t.dataset.f22Filter||'ALL').toUpperCase(); if(!TYPES.includes(state.type))state.type='ALL'; state.page=0; render('filter'); return false;} if(t.id==='btnLibraryPrevF22'){state.page=Math.max(0,state.page-1);render('prev');return false;} if(t.id==='btnLibraryNextF22'){state.page++;render('next');return false;} const act=t.dataset.f22Action,file=t.dataset.file||''; if(act==='cas'){prepareCas(file);return false;} if(act==='mount'){mountFile(file,Number(t.dataset.unit||1));return false;} if(act==='delete'){if(confirm('¿Eliminar '+file+'?')){if(typeof deleteAtr==='function')deleteAtr(enc(file)); else xhrJson('/atr/delete?file='+enc(file)).then(()=>load(false));}return false;}},true);
  document.addEventListener('change',ev=>{const t=ev.target; if(t&&t.id==='atrLibraryPageSizeF22'){ev.preventDefault();ev.stopPropagation();state.pageSize=Number(t.value||24)||24;state.page=0;render('page-size');}},true);
  window.renderAtrLibrary=function(files){if(Array.isArray(files)&&files.length){setFiles(files,state.payload);syncDriveState().finally(()=>render('external-nonempty'));}else if(state.files.length&&isLibrary())render('external-empty-guard');}; try{renderAtrLibrary=window.renderAtrLibrary;}catch(e){}
  const oldApply=window.atrApplyLightStatusFast; if(typeof oldApply==='function'&&!oldApply.__f22Wrapped){window.atrApplyLightStatusFast=function(st,opts){const r=oldApply.apply(this,arguments); try{if(syncFromAtrStatus(st)&&state.files.length){updateVisibleCardUnits(); if(isLibrary())render('status-sync');}}catch(e){} return r;}; window.atrApplyLightStatusFast.__f22Wrapped=1; try{atrApplyLightStatusFast=window.atrApplyLightStatusFast;}catch(e){}}
  window.loadLibraryPageF22=function(opts){opts=opts||{}; bind(); return load(!!opts.refresh);}; window.loadLibraryPageF21=window.loadLibraryPageF22; window.loadLibraryPageF20=window.loadLibraryPageF22; window.loadLibraryPageF19=window.loadLibraryPageF22; window.loadLibraryPageF18=window.loadLibraryPageF22; window.loadLibraryPageF16=window.loadLibraryPageF22; window.loadLibraryPageF14=window.loadLibraryPageF22; window.loadLibraryPageF49Z83=window.loadLibraryPageF22; window.loadLibraryPageF49Z45=window.loadLibraryPageF22; window.loadLibraryPageF49Z47=window.loadLibraryPageF22;
  try{window.atrLibraryCurrentFilesF27=function(){return state.files.slice();};}catch(e){}
  const oldNav=window.navigateInternalF49Z51; if(typeof oldNav==='function'&&!oldNav.__f22){window.navigateInternalF49Z51=function(){const r=oldNav.apply(this,arguments); setTimeout(()=>{bind(); if(isLibrary())load(false); if(isAtr())load(false).then(populateWebAtr);},220); return r;}; window.navigateInternalF49Z51.__f22=1;}
  window.addEventListener('popstate',()=>setTimeout(()=>{bind(); if(isLibrary())load(false); if(isAtr())load(false).then(populateWebAtr);},220));
  window.addEventListener('focus',()=>{if(isLibrary()&&state.files.length)syncDriveState().then(()=>render('focus-sync'));});
  function boot(){bind(); setCoverDefaultAll(); if(isLibrary())load(false); if(isAtr())load(false).then(populateWebAtr); const block=$('atrLibraryPageBlock'); if(block&&window.MutationObserver){const mo=new MutationObserver(()=>{if(isLibrary()&&state.files.length){const g=$('libraryGridF22'); if(g&&!g.querySelector('.mock-lib-card'))setTimeout(()=>render('observer-restore'),40);}}); mo.observe(block,{childList:true,subtree:true});}}
  if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',boot); else boot();
})();



(function(){try{console.info('F31_LIBRARY_LEGACY_NEUTRALIZED_FIX activo · no debe aparecer Biblioteca F49Z47_TIMEOUT.');}catch(e){}})();
