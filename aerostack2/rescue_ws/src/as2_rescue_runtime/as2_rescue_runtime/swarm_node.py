"""One base-station/drone authority for responder ownership, independent of GCS.
A radio bridge must carry these messages; this node does not configure RF mesh.
"""
import fcntl
import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .swarm import AssignmentAuthority
from .node_main import run as run_node

class Authority(Node):
    def __init__(self):
        super().__init__('swarm_authority');self.declare_parameter('state_file',str(Path.home()/'.local/share/as2-swarm/assignments.json'))
        path=Path(self.get_parameter('state_file').value);path.parent.mkdir(parents=True,exist_ok=True,mode=0o700)
        self.lock=open(path.with_suffix('.lock'),'a+');fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        self.clock=lambda:self.get_clock().now().nanoseconds/1e9
        self.authority=AssignmentAuthority(path,self.clock)
        self.pub=self.create_publisher(String,'swarm/assignments',10)
        self.create_subscription(String,'swarm/request',self.request,10);self.create_timer(.2,self.publish)
    def request(self,msg):
        try:
            data=json.loads(msg.data)
            if not 0<=self.clock()-data['stamp']<=1:raise ValueError('Stale swarm request')
            if data['operation']=='claim':self.authority.claim(data['mission_id'],data['target_id'],data['owner'])
            elif data['operation']=='finish':
                # Completion requires a separate trusted hold report from the owner adapter.
                self.authority.finish(data['mission_id'],data['target_id'],data['owner'],data.get('hold_verified') is True)
        except (ValueError,TypeError,KeyError) as exc:self.get_logger().warning(str(exc))
    def publish(self):
        self.authority.expire();self.pub.publish(String(data=json.dumps(dict(stamp=self.clock(),assignments=list(self.authority.items.values())))))

def main(args=None):
    run_node(Authority, args=args, cleanup=lambda node: node.lock.close())

