#!/usr/bin/env python3
from pyuwb import UwbModule, find_uwb_serial_ports
from uwb_msgs.msg import RangeStamped, PassiveStamped
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
from uwb_driver.schedulers import CommonListScheduler
from time import sleep
import threading
import pypozyx


class PozyxModule:
    """
    Wrapper class to give the pozyx module the same interface as the UwbModule.
    """
    def __init__(self, port: str, timeout=0.1):
        self._pozyx = pypozyx.PozyxSerial(port, timeout=timeout)

    def get_id(self):
        who_am_i = pypozyx.NetworkID()
        status = self._pozyx.getWhoAmI(who_am_i)
        if status == pypozyx.POZYX_SUCCESS:
            return {"id": who_am_i.id, "is_valid": True}
        else:
            return {"id": None, "is_valid": False}
        
    def do_discovery(self, *_, **__): # Discover all other pozyx devices
        self._pozyx.clearDevices()
        self._pozyx.doDiscoveryAll()
        device_list_size = pypozyx.SingleRegister()
        self._pozyx.getDeviceListSize(device_list_size, )
        device_list = pypozyx.DeviceList(list_size=device_list_size.value)
        self._pozyx.getDeviceIds(device_list)
        return [device_list.data[i] for i in range(device_list_size.value)]
    
    def do_twr(self, target_id, *_, **__):
        range_data = pypozyx.DeviceRange()
        status = self._pozyx.doRanging(target_id, range_data)
        if status == pypozyx.POZYX_SUCCESS:
            return {
                "range": range_data.distance/1000,
                "is_valid": True,
                "tx1": range_data.timestamp,
                "rx1": 0,
                "tx2": 0,
                "rx2": 0,
                "tx3": 0,
                "rx3": 0,
                "Pr1": float(range_data.RSS),
                "Pr2": 0.0, 
                }
        else:
            return {
                "range": None,
                "is_valid": False,
            }

class UwbModuleNode(Node):
    def __init__(self):
        super().__init__("uwb_node")

        self.declare_parameter("scheduler", "slow")
        self.declare_parameter("max_id", 15)
        self.declare_parameter("frequency", 200)
        self.declare_parameter("from_id_sequence", rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter("to_id_sequence", rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter("timeout", 0.1)

    def start(self):

        self.modules = []

        # Give priority to our modules. If not found, check pozyx.
        while len(self.modules) == 0 and rclpy.ok():

            self.get_logger().info("Checking for UWB modules...")
            ports = find_uwb_serial_ports()
            timeout = self.get_parameter("timeout").value
            if len(ports) > 0:
                self.get_logger().info("Found UWB modules: " + str(ports))
                self.modules = [UwbModule(port, timeout=timeout, verbose=False, threaded=False) for port in ports]
                break

            self.get_logger().info("Checking for pozyx modules...")
            pozyx_ports = pypozyx.get_serial_ports()
            pozyx_ports = [p.device for p in pozyx_ports]
            if len(pozyx_ports) > 0:
                self.get_logger().info("Found pozyx modules: " + str(pozyx_ports))
                self.modules = [PozyxModule(p, timeout=timeout) for p in pozyx_ports]
                break

            self.get_logger().warn("No UWB modules found. Will keep trying.")
            sleep(3)

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
        self.passive_pub = self.create_publisher(PassiveStamped, "uwb/passive", 1) 

        # Output neighbour-discovery results 
        self.get_logger().info(
            "UWB node initialization success. Tags on machine: "
            + str(self.my_ids)
            + ". Neighbors: "
            + str(self.neighbour_ids)
        )

        self.get_logger().info("Starting scheduler...")

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
        for i, uwb in enumerate(self.modules):
            tag_ids = uwb.do_discovery(list(range(max_id+1)))
            self.get_logger().info(
                "Tag "
                + str(self.my_ids[i])
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
            range_msg.fpp1 = range_data["fpp1"]
            range_msg.fpp2 = range_data["fpp2"]
            range_msg.skew1 = range_data["skew1"]
            range_msg.skew2 = range_data["skew2"]
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
        passive_msg.pr1 = passive_data["fpp1"]
        passive_msg.pr2 = passive_data["fpp2"]
        passive_msg.pr3 = passive_data["fpp3"]
        passive_msg.skew1 = passive_data["skew1"]
        passive_msg.skew2 = passive_data["skew2"]
        passive_msg.skew3 = passive_data["skew3"]
        passive_msg.pr1_n = passive_data["fpp1_n"]
        passive_msg.pr2_n = passive_data["fpp2_n"]
        passive_msg.skew1_n = passive_data["skew1_n"]
        passive_msg.skew2_n = passive_data["skew2_n"]
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
        last_ranged = {}
        active_tags = self.neighbour_ids
        lost_tags = []
        count = 0
        while rclpy.ok():
 
            # Loop through all the neighbors
            for nb_id in active_tags:

                # Loop through the  modules attached to our machine
                for i, uwb in enumerate(self.modules):
                    my_id = self.my_ids[i]

                    # Dont range between same tags on machine, or any dropped
                    # tags.
                    if nb_id not in self.my_ids and nb_id not in lost_tags:
                        
                        # If its been too long since last ranged, tag is considered lost
                        if nb_id in last_ranged and (self.get_clock().now() - last_ranged[nb_id]).nanoseconds > 1e9:
                            lost_tags.append(nb_id)
                            active_tags.remove(nb_id)
                            self.get_logger().info("Tag " + str(nb_id) + " lost!")
                            break
                        

                        range_data = uwb.do_twr(target_id=nb_id, mult_twr=True)
                        if range_data["is_valid"]:
                            last_ranged[nb_id] = self.get_clock().now()
                            self.publish_range((my_id, nb_id), range_data)

                    try:  # Prevent garbage in console output when killed
                        rate.sleep()
                    except ROSInterruptException:
                        pass

            count += 1 
            if count == 100:
                for nb_id in lost_tags:
                    uwb = self.modules[0]
                    range_data = uwb.do_twr(target_id=nb_id, mult_twr=True)
                    if range_data["is_valid"]:
                        last_ranged[nb_id] = self.get_clock().now()
                        self.publish_range((self.my_ids[0], nb_id), range_data)
                        self.get_logger().info("Tag " + str(nb_id) + " regained!")
                        lost_tags.remove(nb_id)
                        active_tags.append(nb_id)
                count = 0

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