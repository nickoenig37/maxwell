import rclpy
from sensor_msgs.msg import NavSatFix
import math
import time  # Import the time module
from nav_msgs.msg import Odometry


#for possible mathematic reference in code ref: https://www.movable-type.co.uk/scripts/latlong.html 
#https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/

class HeadingCalculatorNode:
    def __init__(self):
        self.node = rclpy.create_node('heading_calculator_node')
        self.subscription_gps = self.node.create_subscription(
            NavSatFix,
            '/gps/filtered',  #subscription to the filtered GPS data from navsat_transform node
            self.gps_callback,
            10  # QoS profile
        )

        self.prev_lat = None
        self.prev_lon = None
        self.del_pos = 0.0

        # was thinking about using the odometry data to calculate heading but currently just using navsat_transform
        self.subscription_odom = self.node.create_subscription(
            Odometry,
            '/odometry/local',  #local odometry message
            self.odom_callback,
            10  # QoS profile
        )

        self.taken_headingoff = False #boolean to check if heading has been taken off

    def gps_callback(self, msg):
        if (self.prev_lat is not None and self.prev_lon is not None):
            if (self.calculate_distance(msg.latitude, msg.longitude, self.prev_lat, self.prev_lon) > 1.0):
                # Calculate heading
                heading = self.calculate_heading(
                    self.prev_lat, self.prev_lon,
                    msg.latitude, msg.longitude
                )
                print(f'Rover heading: {heading} degrees')
                # Update previous GPS data
                self.prev_lat = msg.latitude
                self.prev_lon = msg.longitude
        
        else:
            # Update previous GPS data when initial is none 
            self.prev_lat = msg.latitude
            self.prev_lon = msg.longitude
        
    def odom_callback(self, msg):
        #only want to do this once
        if (self.taken_headingoff == False):
            if (math.sqrt((msg.pose.pose.position.x)**2 + (msg.pose.pose.position.y)**2) > 3.0):
                # Calculate heading error 
                heading_error = math.atan2(msg.pose.pose.position.y, msg.pose.pose.position.x)
                print(f'Rover heading error: {heading_error} degrees')
                self.taken_headingoff = True

    # gives heading of the robot in degrees
    def calculate_heading(self, lat1, lon1, lat2, lon2):
        # Calculate change in latitude and longitude
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)

        # Calculate heading using arctangent
        heading_rad = math.atan2(delta_lon, delta_lat)

        # Convert heading to degrees
        heading_deg = math.degrees(heading_rad)

        # Ensure the heading is in the range [0, 360)
        heading_deg = (heading_deg + 360) % 360

        return heading_deg
    

    #This function calculates the distance travelled between two GPS coordinates
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        # Calculate distance using Haversine formula
        R = 6371000  # Earth radius in meters
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c

        return distance

def main():
    rclpy.init()
    node = HeadingCalculatorNode()
    rclpy.spin(node.node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
