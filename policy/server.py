"""Loopback-only native-input test surface. One worker owns physics and policy."""
from http.server import ThreadingHTTPServer,BaseHTTPRequestHandler
from pathlib import Path
from threading import Thread,Lock
from queue import Queue,Empty
import json,time,os,hashlib
from evidence_store import EvidenceStore,EvidenceConflict
PolicySimulation=None
OUT=Path(__file__).resolve().parents[1]/'evidence'
OUT.mkdir(parents=True,exist_ok=True)
SERVER_HASH=hashlib.sha256(Path(__file__).read_bytes()+Path(__file__).with_name('evidence_store.py').read_bytes()).hexdigest()
commands=Queue();lock=Lock();latest={'ready':False};frames=[]

def metadata(sim):
 return {'checkpoint':sim.checkpoint,'names':sim.names,'radius':sim.radius,'requestedSpeed':sim.speed,
         'requestedHeight':float(getattr(sim,'height',0.)),'controlHz':sim.contract['controlHz'],'physicsHz':sim.contract['physicsHz'],'circle':sim.circle,'circleCenter':[float(v) for v in getattr(sim,'circle_center',[0,0])]}
def worker():
 global latest,frames
 evidence=EvidenceStore(OUT)
 try:
  sim=PolicySimulation(2.5,True);paused=True
  with lock:latest={**sim.snapshot(),**metadata(sim),'ready':True,'paused':True}
  while True:
   start=time.perf_counter()
   try:
    while True:
     command=commands.get_nowait()
     if command.get('reset'):
      sim.reset();frames=[];evidence.reset()
     if 'speed' in command:sim.speed=max(0,min(5,float(command['speed'])))
     if 'circle' in command:sim.circle=bool(command['circle']) and (getattr(sim,'allow_circle',False) or not hasattr(sim,'disturb'))
     if 'disturbance' in command and hasattr(sim,'disturb'):sim.disturb(str(command['disturbance']),float(command.get('disturbanceSpeed',3.)))
     if command.get('jump') and hasattr(sim,'request_jump'):sim.request_jump()
     if command.get('traverse') and hasattr(sim,'request_traversal'):sim.request_traversal()
     if 'contacts' in command and hasattr(sim,'set_contacts'):sim.set_contacts(command['contacts'])
     if 'crouch' in command and hasattr(sim,'height'):sim.height=sim.standing_height-((.15 if os.environ.get('LOCOMOTION_CONTROLLER')=='contact-muscle' else .18) if command['crouch'] else 0.)
     if 'balance' in command and hasattr(sim,'set_balance_target'):sim.set_balance_target(command['balance'])
     if command.get('step') and hasattr(sim,'request_step'):sim.request_step()
     if 'paused' in command:
      paused=bool(command['paused'])
      if paused:
       try:
        filename=evidence.save({'checkpoint':sim.checkpoint,'names':sim.names,'frames':frames,'radius':sim.radius,'serverSourceSha256':SERVER_HASH},command.get('evidenceTag'))
        if filename:
         with lock:latest={**latest,'evidenceFile':filename,'evidenceConflict':False}
       except EvidenceConflict as error:
        with lock:latest={**latest,'evidenceConflict':str(error)}
     if command.get('reset'):
      with lock:latest={**sim.snapshot(),**metadata(sim),'ready':True,'paused':paused}
   except Empty:pass
   if not paused:
    step_start=time.perf_counter();state=sim.step();finished=time.perf_counter();state.update({'requestedSpeed':sim.speed,'circle':sim.circle,'wallTime':finished,'stepWallSeconds':finished-step_start});frames.append(state)
    if len(frames)>5400:frames=frames[-5400:]
    if os.environ.get('LOCOMOTION_CONTROLLER') in ['mpc','centroidal','motor','contact-muscle','foundation','spherical-muscles','contact-scene','unified-contact']:
     torso=sim.names.index('torso');q=state['rotations'][torso]
     if state.get('physicalFailure') or (not hasattr(sim,'disturb') and (state['root'][2]<.4 or 1-2*(q[0]**2+q[1]**2)<.7)):
      paused=True;state['testFailure']='Lost upright physical support; test stopped, no correction applied'
      (OUT/'policy-native-latest.json').write_text(json.dumps({'checkpoint':sim.checkpoint,'names':sim.names,'frames':frames,'testFailure':state['testFailure']},separators=(',',':')))
    with lock:latest={**state,**metadata(sim),'ready':True,'paused':paused}
   else:
    with lock:latest={**latest,'paused':True}
   time.sleep(max(.001,1/sim.contract['controlHz']-(time.perf_counter()-start)))
 except Exception as error:
  if 'sim' in locals():(OUT/'policy-native-latest.json').write_text(json.dumps({'checkpoint':sim.checkpoint,'names':sim.names,'frames':frames,'error':str(error)},separators=(',',':')))
  with lock:latest={**latest,'ready':False,'paused':True,'error':str(error)}
  raise
class Handler(BaseHTTPRequestHandler):
 def log_message(self,*args):pass
 def reply(self,status,value):
  blob=json.dumps(value,separators=(',',':')).encode();self.send_response(status);self.send_header('Content-Type','application/json');self.send_header('Access-Control-Allow-Origin','http://127.0.0.1:5188');self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(blob)));self.end_headers();self.wfile.write(blob)
 def do_GET(self):
  # Network I/O must never hold the physics publication lock.
  with lock:state=latest
  try:self.reply(200,state)
  except (ConnectionAbortedError,ConnectionResetError,BrokenPipeError):pass
 def do_OPTIONS(self):
  self.send_response(204);self.send_header('Access-Control-Allow-Origin','http://127.0.0.1:5188');self.send_header('Access-Control-Allow-Headers','Content-Type');self.send_header('Access-Control-Allow-Methods','GET,POST,OPTIONS');self.end_headers()
 def do_POST(self):
  if self.path!='/intent':return self.reply(404,{'error':'Unknown route'})
  size=int(self.headers.get('Content-Length','0'))
  if size>1024:return self.reply(413,{'error':'Intent too large'})
  try:commands.put(json.loads(self.rfile.read(size)));self.reply(202,{'queued':True})
  except (ValueError,TypeError):self.reply(400,{'error':'Invalid intent'})
def serve(controller=None):
 global PolicySimulation
 if controller is None:raise ValueError('Start with python -m unified_runtime.serve')
 if controller is not None:PolicySimulation=controller
 port=int(os.environ.get('LOCOMOTION_PORT','5190'));assert port in [5190,5191]
 Thread(target=worker,daemon=True).start();print('Private policy test: http://127.0.0.1:'+str(port),flush=True);ThreadingHTTPServer(('127.0.0.1',port),Handler).serve_forever()
if __name__=='__main__':serve()
