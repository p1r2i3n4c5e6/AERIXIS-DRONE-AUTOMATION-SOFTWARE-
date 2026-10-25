"""Authenticated companion-computer mission API. TLS required beyond loopback."""
import argparse
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import signal
import ssl
import threading
import urllib.parse

from .manager import MissionManager, load_registry
from .status_delta import compact


def make_handler(manager, token):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def setup(self):
            super().setup();self.connection.settimeout(10)
        def respond(self,value,status=200):
            data=json.dumps(value,allow_nan=False,separators=(',',':')).encode()
            self.send_response(status)
            self.send_header('Content-Type','application/json')
            self.send_header('Content-Length',str(len(data)))
            self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data)
        def authorized(self):
            return hmac.compare_digest(self.headers.get('Authorization','').encode(),('Bearer '+token).encode())
        def do_GET(self):
            if not self.authorized():return self.respond({'error':'Unauthorized'},401)
            path,_,query=self.path.partition('?')
            if path == '/api/status':
                params=dict(urllib.parse.parse_qsl(query))
                state=manager.snapshot()
                return self.respond(compact(state,params) if params.get('lite')=='1' else state)
            if self.path == '/api/registry':return self.respond({'hash':manager.registry_hash,'nodes':manager.registry})
            return self.respond({'error':'Not found'},404)
        def do_POST(self):
            if not self.authorized():return self.respond({'error':'Unauthorized'},401)
            try:
                size=int(self.headers.get('Content-Length','0'))
                if not 0 < size <= 1_100_000:raise ValueError('Invalid request size')
                data=json.loads(self.rfile.read(size))
                if not isinstance(data,dict):raise ValueError('Expected a JSON object')
                methods={'/api/mission/stage':manager.stage,'/api/mission/start':manager.start,
                         '/api/mission/cancel':manager.cancel,'/api/avoidance':manager.set_avoidance}
                if self.path in methods:return self.respond({'ok':True,'result':methods[self.path](data)})
                if self.path == '/api/mission/reconcile':return self.respond(manager.reconcile())
                return self.respond({'error':'Not found'},404)
            except (ValueError,KeyError,TypeError) as exc:return self.respond({'error':str(exc)},400)
            except Exception as exc:
                manager.log('ERROR',str(exc));return self.respond({'error':'Onboard operation failed; inspect logs'},500)
    return Handler


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drone-id',required=True)
    parser.add_argument('--state-dir',required=True)
    parser.add_argument('--runner',required=True)
    parser.add_argument('--token-file',required=True)
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=8790)
    parser.add_argument('--tls-cert');parser.add_argument('--tls-key')
    parser.add_argument('--allow-flight',action='store_true')
    args=parser.parse_args()
    if args.host not in ('127.0.0.1','localhost','::1') and not (args.tls_cert and args.tls_key):
        parser.error('TLS certificate and key required for network exposure; use loopback for bench testing')
    token=Path(args.token_file).read_text().strip()
    if len(token) < 32:parser.error('Token file must contain at least 32 characters')
    import rclpy
    from .observer import RosObserver
    rclpy.init(args=[])
    observer=RosObserver(args.drone_id)
    manager=MissionManager(args.drone_id,args.state_dir,args.runner,load_registry(args.runner),
                           observer,args.allow_flight)
    server=ThreadingHTTPServer((args.host,args.port),make_handler(manager,token))
    if args.tls_cert and args.tls_key:
        context=ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(args.tls_cert,args.tls_key)
        server.socket=context.wrap_socket(server.socket,server_side=True)
    def stop(*unused):threading.Thread(target=server.shutdown,daemon=True).start()
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    print(f'Onboard mission manager for {args.drone_id} listening on {args.host}:{args.port}',flush=True)
    try:server.serve_forever()
    finally:
        manager.close();observer.close();server.server_close()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
