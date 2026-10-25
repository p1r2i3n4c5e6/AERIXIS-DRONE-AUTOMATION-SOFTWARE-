/* Optional geodetic display contract. Missing/old coordinates must never become live markers. */
(function(root,factory){const model=factory();if(typeof module==='object'&&module.exports)module.exports=model;else root.GCSMapPosition=model;})(typeof window==='undefined'?globalThis:window,()=>{
 'use strict';
 function read(data){
  const gps=data?.gps;if(!gps||typeof gps!=='object')return null;
  const {latitude:lat,longitude:lon,age_s:age,fix_type:fix}=gps;
  if(gps.valid===false||![lat,lon,age,fix].every(v=>typeof v==='number'&&Number.isFinite(v))||Math.abs(lat)>90||Math.abs(lon)>180||age<0||age>2.5||fix<2)return null;
  return {lat,lon,age,heading:typeof gps.heading_deg==='number'&&Number.isFinite(gps.heading_deg)?((gps.heading_deg%360)+360)%360:null,altitude:typeof gps.altitude_m==='number'&&Number.isFinite(gps.altitude_m)?gps.altitude_m:null};
 }
 function fleetPosition(drone){return drone?.fresh&&drone.connection==='CONNECTED'?read(drone.data):null;}
 function localPosition(state){if(!state?.connected)return null;return read(state)||read({gps:{latitude:state.latitude,longitude:state.longitude,age_s:state.gps_age_s,fix_type:state.gps_fix_type,heading_deg:state.heading_deg,altitude_m:state.altitude}});}
 return {read,fleetPosition,localPosition};
});
