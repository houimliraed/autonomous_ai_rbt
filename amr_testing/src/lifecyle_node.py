import rclpy
from rclpy.lifecycle import Node,State,TransitionCallbackReturn
from std_msgs.msg import String

class lifecycle(Node):
    def __init__(self,node_name,**kwargs):
        super.__init__(node_name,**kwargs)


    def on_configure(self, state : State) -> TransitionCallbackReturn:
        self.sub_ = self.create_subscription(String,"chatter",self.msgCallback,10)
        self.get_logger().info("LifeCycle Node on_configure() called")
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state : State) -> TransitionCallbackReturn:
        self.destroy_subscription(self.sub_)
        self.get_logger().info("LifeCycle Node on_shutdown() called")
        return TransitionCallbackReturn.SUCCESS    
