import React,{useEffect,useRef,useState} from 'react';
import {createRoot} from 'react-dom/client';
import {createPortal} from 'react-dom';
function InterfaceTools(){
 const [open,setOpen]=useState(false),[events,setEvents]=useState([]);const panel=useRef(null),trigger=useRef(null);
 useEffect(()=>{
  const stack=document.getElementById('gcs-toasts');if(!stack)return;
  const observer=new MutationObserver(records=>{for(const record of records)for(const node of record.addedNodes){if(node.nodeType!==1||!node.classList.contains('gcs-toast'))continue;const message=node.querySelectorAll('span')[1]?.textContent||node.textContent;setEvents(prev=>[{message,time:new Date().toLocaleTimeString(),error:node.classList.contains('ERROR')},...prev].slice(0,100));}});
  observer.observe(stack,{childList:true});return()=>observer.disconnect();
 },[]);
 useEffect(()=>{if(!open)return;panel.current?.focus();const close=e=>{if(e.key==='Escape'){setOpen(false);trigger.current?.focus();}if(e.type==='pointerdown'&&!panel.current?.contains(e.target)&&!trigger.current?.contains(e.target))setOpen(false);};document.addEventListener('keydown',close);document.addEventListener('pointerdown',close);return()=>{document.removeEventListener('keydown',close);document.removeEventListener('pointerdown',close);};},[open]);
 return <div className="interface-tools"><button ref={trigger} onClick={()=>setOpen(!open)} aria-expanded={open} aria-controls="interface-notifications"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.6" aria-hidden="true"><path d="M18 8a6 6 0 00-12 0c0 7-3 7-3 9h18c0-2-3-2-3-9M10 21h4"/></svg>Notifications {events.length>0&&<span className="count">{events.length}</span>}</button>{open&&createPortal(<section ref={panel} tabIndex={-1} id="interface-notifications" className="notification-drawer" role="region" aria-label="Notification history"><div className="drawer-head"><div><h2>Notifications</h2><p>Updates from this browser session</p></div><button onClick={()=>{setOpen(false);trigger.current?.focus();}} aria-label="Close notifications">×</button></div>{!events.length?<p style={{padding:'24px 0'}}>No notifications yet. Connection and mission updates will appear here.</p>:events.map((event,i)=><article key={i}><time>{event.time} · {event.error?'Attention':'Update'}</time><p>{event.message}</p></article>)}</section>,document.body)}</div>;
}
createRoot(document.getElementById('interface-tools')).render(<InterfaceTools/>);
// Observe presentation changes only; existing navigation and command handlers remain owners.
const views=document.querySelectorAll('.tab-view');const observer=new MutationObserver(records=>{for(const record of records){const view=record.target;if(view.classList.contains('active')&&!record.oldValue?.split(' ').includes('active')){view.classList.remove('ui-arrive');void view.offsetWidth;view.classList.add('ui-arrive');}}});views.forEach(view=>observer.observe(view,{attributes:true,attributeFilter:['class'],attributeOldValue:true}));
const stack=document.getElementById('gcs-toasts');if(stack){stack.setAttribute('role','status');stack.setAttribute('aria-live','polite');stack.setAttribute('aria-relevant','additions');}
