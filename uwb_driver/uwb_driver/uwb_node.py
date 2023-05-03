#!/usr/bin/env python3
from pyuwb import UwbModule, find_uwb_serial_ports
from uwb_msgs.msg import RangeStamped, PassiveStamped
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
from uwb_driver.schedulers import CommonListScheduler
from time import sleep
import threading

class UwbModuleNode(Node):
    def __init__(self):
        super().__init__("uwb_node")

        self.declare_parameter("scheduler", "slow")
        self.declare_parameter("max_id", 12)
        self.declare_parameter("frequency", 100)
        self.declare_parameter("from_id_sequence", rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter("to_id_sequence", rclpy.Parameter.Type.INTEGER_ARRAY)

    def start(self):

        ports = find_uwb_serial_ports()
        while len(ports) == 0 and rclpy.ok():
            self.get_logger().warn("No UWB modules found. Will keep trying.")
            sleep(3)
            if rclpy.ok():
                ports = find_uwb_serial_ports()

        self.modules = [UwbModule(port, timeout=0.1, verbose=False, threaded=False) for port in ports]

        # Get my ids
        self.my_ids = [module.get_id()["id"] for module in self.modules]
        if None in self.my_ids:
            self.get_logger().warn("Issue with at least one UWB module: cannot get ID.")

        # Do neighbor discovery until we find someone
        self.neighbour_ids = self.discover()
        while len(self.neighbour_ids) == 0 and rclpy.ok():
            self.get_logger().warn(
                "Could not find any neighbours. Will keep trying"
            )
            sleep(3)
            if rclpy.ok():
                self.neighbour_ids = self.discover()

        # Create single publisher for the range measurements
        self.range_pub = self.create_publisher(RangeStamped,"uwb/range",1)

        # Create single publisher for the passive measurements
        self.passive_pub = self.create_publisher(PassiveStamped, "uwb/passing", 1) 

        # Output neighbour-discovery results 
        self.get_logger().info(
            "UWB node init success. Tags on machine: "
            + str(self.my_ids)
            + ". Neighbors: "
            + str(self.neighbour_ids)
        )

        if self.get_parameter("scheduler").value == "slow":
            self.start_slow_scheduler()
        elif self.get_parameter("scheduler").value == "common_list":
            self.start_common_list_scheduler()

    def discover(self):
        """
        Performs neighbor discovery
        """
        # Get neighbours
        neighbour_ids = []
        max_id = self.get_parameter("max_id").value

        rate = self.create_rate(3)
        for i in range(max_id + 1):
            if i in self.my_ids:
                idx = self.my_ids.index(i)
                uwb = self.modules[idx]
                tag_ids = uwb.do_discovery(
                    possible_ids = range(max_id + 1)
                )
                self.get_logger().debug(
                    "Tag "
                    + str(self.my_ids[idx])
                    + " has discovered "
                    + str(tag_ids)
                )
                neighbour_ids.extend(tag_ids)
                
            if not rclpy.ok():
                break

            rate.sleep()

        return list(set(neighbour_ids))  # remove duplicates

    def publish_range(self, pair_ids, range_data):
        if range_data["is_valid"]:
            range_msg = RangeStamped()
            range_msg.header.stamp = self.get_clock().now().to_msg()
            range_msg.from_id = pair_ids[0]
            range_msg.to_id = pair_ids[1]
            range_msg.range = range_data["range"]
            range_msg.tx1 = range_data["tx1"]
            range_msg.rx1 = range_data["rx1"]
            range_msg.tx2 = range_data["tx2"]
            range_msg.rx2 = range_data["rx2"]
            range_msg.tx3 = range_data["tx3"]
            range_msg.rx3 = range_data["rx3"]
            range_msg.power1 = range_data["Pr1"]
            range_msg.power2 = range_data["Pr2"]
            self.range_pub.publish(range_msg)

    def publish_passive(self, pair_ids, passive_data, my_id):
        passive_msg = PassiveStamped()
        passive_msg.header.stamp = self.get_clock().now().to_msg()
        passive_msg.my_id = my_id
        passive_msg.from_id = pair_ids[0]
        passive_msg.to_id = pair_ids[1]
        passive_msg.rx1 = passive_data["rx1"]
        passive_msg.rx2 = passive_data["rx2"]
        passive_msg.rx3 = passive_data["rx3"]
        passive_msg.tx1_n = passive_data["tx1_n"]
        passive_msg.rx1_n = passive_data["rx1_n"]
        passive_msg.tx2_n = passive_data["tx2_n"]
        passive_msg.rx2_n = passive_data["rx2_n"]
        passive_msg.tx3_n = passive_data["tx3_n"]
        passive_msg.rx3_n = passive_data["rx3_n"]
        passive_msg.pr1 = passive_data["Pr1"]
        passive_msg.pr2 = passive_data["Pr2"]
        passive_msg.pr3 = passive_data["Pr3"]
        passive_msg.pr1_n = passive_data["Pr1_n"]
        passive_msg.pr2_n = passive_data["Pr2_n"]
        self.passive_pub.publish(passive_msg)

    def shutdown_hook(self):
        for uwb in self.modules:
            uwb.close()


    def start_slow_scheduler(self):
        """
        Starts the naive "slow" scheduling scheme where we simply range at such
        a low frequency that the odds of collisions are low.
        """
        range_freq = self.get_parameter("frequency").value
        rate = self.create_rate(range_freq)
        while rclpy.ok():
 
            # Loop through all the neighbors
            for nb_id in self.neighbour_ids:

                # Loop through the  modules attached to our machine
                for i, uwb in enumerate(self.modules):
                    my_id = self.my_ids[i]
                    if nb_id not in self.my_ids: # No self-ranging.

                        range_data = uwb.do_twr(target_id=nb_id, mult_twr=True)
                        if range_data["is_valid"]:
                            self.publish_range((my_id, nb_id), range_data)

                    try:  # Prevent garbage in console output when killed
                        rate.sleep()
                    except ROSInterruptException:
                        pass

    def start_common_list_scheduler(self):
        """
        Starts the distributed scheduling scheme.
        """
        seq = list(zip(
            self.get_parameter("from_id_sequence").get_parameter_value().integer_array_value,
            self.get_parameter("to_id_sequence").get_parameter_value().integer_array_value
        ))
        scheduler = CommonListScheduler(
            self.modules, 
            self.my_ids, 
            seq, 
            self.publish_range, 
            self.publish_passive,
        )

        scheduler._latest_pair = None
        for i, uwb in enumerate(scheduler._modules):
            # Just for type hinting
            uwb: UwbModule = uwb
            
            my_id = scheduler._my_ids[i]
            uwb.toggle_passive(True)
            uwb.register_listening_callback(scheduler.listening_callback, my_id)
            uwb.register_range_callback(scheduler.ranging_with_me_callback, my_id)
            uwb.device.timeout = 0.003

        # Initialize the scheme
        # below may be confusing. we are sending this function the last pair
        # in the sequence so that the next pair is the first in the sequence.
        # We are reusing this function for starting the scheme
        scheduler.handle_ranging_event(scheduler._sequence[-1])

        while rclpy.ok():
 
            # Keep checking the modules for external messages until a timeout
            # expires.
            start_time = self.get_clock().now().nanoseconds/1e9
            timeout = 0.07
            while (self.get_clock().now().nanoseconds/1e9 - start_time) < timeout and rclpy.ok():

                for uwb in scheduler._modules:
                    uwb.wait_for_messages()

                    if scheduler._ranging_event:
                        # Reset the message event flag
                        scheduler._ranging_event = False

                        # Initiate next ranging if it is our turn
                        scheduler.handle_ranging_event(scheduler._latest_pair)                        

                        # Reset the timeout. Stays in this while loop
                        start_time = self.get_clock().now().nanoseconds/1e9
                    
            # If we got here, its been a while since we have heard a range meas.
            # Advance to the next item on the list and initiate if it is our
            # turn
            scheduler._latest_pair = scheduler.get_next_pair(scheduler._latest_pair)
            self.get_logger().debug(
                "Ranging timeout. Restarting from latest pair: " \
                + str(scheduler._latest_pair)
            )

            # Internally, the below function will check if we need to
            # initate ranging.
            scheduler.handle_ranging_event(scheduler._latest_pair)

def main():
    rclpy.init()

    # Spin in a separate thread
    # https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
    # Wow, ROS2 API hurts right now
    node = UwbModuleNode()
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()


    node.start()


if __name__ == "__main__":
    main()