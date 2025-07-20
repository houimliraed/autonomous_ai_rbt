import rclpy 
from rclpy.node import Node
from std_msgs.msg import String



class qualityOfServicePub(Node):
    def __init__(self):
        self.pub_ = self.create_publisher(String,"chatter",10)
        self.counter_ = 0
        self.frequency_ = 1.0
        self.get_logger().info("publishing at %d Hz" % self.frequency_)


        self.timer_ =  self.create_timer(self.frequency_,self.timercallback)


    def timercallback(self):
        msg = String
        msg.data = "Hello from ros2 - counter %d"% self.counter_
        self.pub_.publish(msg)
        self.counter_ += 1


def main():
    rclpy.init()

    QualityOfServicePub = qualityOfServicePub()
    rclpy.spin(QualityOfServicePub)

    QualityOfServicePub.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()    