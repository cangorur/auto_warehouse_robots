from morse.middleware.ros import ROSPublisher
from auto_smart_factory.msg import TraySensor
import rospy

class TraySensorPublisher(ROSPublisher):
    ros_class = TraySensor
    
    def initialize(self):
        ROSPublisher.initialize(self)
        
        # last occupied state. Used to detect changes of the state
        self.occupied_state = None

    def default(self, ci='unused'):
        occupied = bool(self.data['near_objects'])
        
        if (self.occupied_state == None) or (occupied != self.occupied_state):
            self.occupied_state = occupied
        
            msg = TraySensor()
            msg.stamp = rospy.Time.now()
            msg.tray_id = self.kwargs['tray_id']
            msg.occupied = occupied
            
            if occupied:
                msg.package = list(self.data['near_objects'].keys())[0]
            else:
                msg.package = ''
            
            self.publish(msg)
