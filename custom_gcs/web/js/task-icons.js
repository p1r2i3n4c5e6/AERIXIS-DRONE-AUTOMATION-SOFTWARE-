/* Local vector icons and restrained accents describe task type, never execution status. */
(() => {
 const paths={
  drone:'M8 8l8 8M16 8l-8 8M10 8l2-2 2 2 1 6-3 3-3-3zM10 11h4M12 7v2M5 1a4 3 0 1 0 0 6 4 3 0 0 0 0-6M19 1a4 3 0 1 0 0 6 4 3 0 0 0 0-6M5 17a4 3 0 1 0 0 6 4 3 0 0 0 0-6M19 17a4 3 0 1 0 0 6 4 3 0 0 0 0-6M2 4h6M16 4h6M2 20h6M16 20h6M9 17l-1 3M15 17l1 3',
  takeoff:'M12 17V3m-5 5 5-5 5 5M4 17v4h16v-4',
  land:'M12 3v14m-5-5 5 5 5-5M4 18v3h16v-3',
  shield:'M12 3 4 6v6c0 5 8 9 8 9s8-4 8-9V6zM8 12l3 3 5-6',
  route:'M5 17a3 3 0 1 0 0 6 3 3 0 0 0 0-6M19 1a3 3 0 1 0 0 6 3 3 0 0 0 0-6M5 17V8c0-4 14 8 14-1',
  search:'M16 16l5 5M10 3a7 7 0 1 0 0 14 7 7 0 0 0 0-14M7 10h6M10 7v6',
  camera:'M3 7h5l2-3h4l2 3h5v13H3zM12 9a4 4 0 1 0 0 8 4 4 0 0 0 0-8',
  payload:'m3 7 9-4 9 4-9 5zM3 7v10l9 4 9-4V7M12 12v9M7 5l10 5',
  swarm:'M12 1a3 3 0 1 0 0 6 3 3 0 0 0 0-6M4 16a3 3 0 1 0 0 6 3 3 0 0 0 0-6M20 16a3 3 0 1 0 0 6 3 3 0 0 0 0-6M10 7l-4 8m8-8 4 8M7 19h10',
  clock:'M12 2a10 10 0 1 0 0 20 10 10 0 0 0 0-20M12 6v6l4 3',
  condition:'m12 2 10 10-10 10L2 12zM9 12l2 2 4-4',
  sequence:'M9 2h6v5H9zM2 17h6v5H2zM16 17h6v5h-6zM12 7v5M5 17v-5h14v5',
  loop:'M19 7a8 8 0 0 0-14 3M5 17a8 8 0 0 0 14-3M19 2v5h-5M5 22v-5h5',
  radio:'M9 12a3 3 0 1 0 6 0 3 3 0 0 0-6M6 6a9 9 0 0 0 0 12M18 6a9 9 0 0 1 0 12M12 15v7',
  action:'m12 3 9 9-9 9-9-9zM8 12h8M12 8l4 4-4 4'
 };
 function describe(type='',label='',kind=''){
  const text=(type+' '+label).toLowerCase();if(type==='drone')return {icon:'drone',tone:'cyan'};
  if(/takeoff|take.off/.test(text))return {icon:'takeoff',tone:'cyan'};
  if(/land/.test(text))return {icon:'land',tone:'blue'};
  if(/arm|healthy|safety|hold|abort/.test(text)&&!/swarm/.test(text))return {icon:'shield',tone:'amber'};
  if(/payload|deliver|release|package|aid/.test(text))return {icon:'payload',tone:'amber'};
  if(/swarm|peer|assign|separation/.test(text))return {icon:'swarm',tone:'violet'};
  if(/camera|gimbal|track|person|geotag|observation/.test(text))return {icon:'camera',tone:'violet'};
  if(/search|survey|coverage/.test(text))return {icon:'search',tone:'cyan'};
  if(/goto|waypoint|path|route|origin|position/.test(text))return {icon:'route',tone:'blue'};
  if(/wait|sleep|delay|timeout/.test(text))return {icon:'clock',tone:'amber'};
  if(/offboard|radio|event/.test(text))return {icon:'radio',tone:'cyan'};
  if(kind==='Control')return {icon:'sequence',tone:'violet'};
  if(kind==='Condition')return {icon:'condition',tone:'amber'};
  if(kind==='Decorator')return {icon:'loop',tone:'blue'};
  return {icon:'action',tone:'cyan'};
 }
 function element(type,label,kind){const task=describe(type,label,kind),svg=document.createElementNS('http://www.w3.org/2000/svg','svg');svg.setAttribute('viewBox','0 0 24 24');svg.setAttribute('fill','none');svg.setAttribute('stroke','currentColor');svg.setAttribute('stroke-width','1.5');svg.setAttribute('stroke-linecap','round');svg.setAttribute('stroke-linejoin','round');svg.setAttribute('aria-hidden','true');const path=document.createElementNS(svg.namespaceURI,'path');path.setAttribute('d',paths[task.icon]);svg.append(path);return svg;}
 window.GCSTaskIcons={paths,describe,element};
})();
