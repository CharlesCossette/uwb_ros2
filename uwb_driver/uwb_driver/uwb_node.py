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
        self.declare_parameter("max_id", 10)
        self.declare_parameter("frequency", 200)

    def start(self):

        self.modules = []

        # Give priority to our modules. If not found, check pozyx.
        while len(self.modules) == 0 and rclpy.ok():

            self.get_logger().info("Checking for UWB modules...")
            ports = find_uwb_serial_ports()
            if len(ports) > 0:
                self.get_logger().info("Found UWB modules: " + str(ports))
                self.modules = [UwbModule(port, timeout=0.1, verbose=False, threaded=False) for port in ports]
                break

            self.get_logger().info("Checking for pozyx modules...")
            pozyx_ports = pypozyx.get_serial_ports()
            pozyx_ports = [p.device for p in pozyx_ports]
            if len(pozyx_ports) > 0:
                self.get_logger().info("Found pozyx modules: " + str(pozyx_ports))
                self.modules = [PozyxModule(p) for p in pozyx_ports]
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
        self.passive_pub = self.create_publisher(PassiveStamped, "uwb/passing", 1) 

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
            # seq = [
            #     (x["from_id"], x["to_id"])
            #     for x in rospy.get_param("/uwb/sequence")
            # ]
            # scheduler = CommonListScheduler(
            #     self.modules, self.my_ids, seq, self.publish_range, self.publish_passive
            # )
            # scheduler.start()
            pass

    def discover(self):
        """
        Performs neighbor discovery
        """
        # Get neighbours
        neighbour_ids = []
        max_id = self.get_parameter("max_id").value

        rate = self.create_rate(3)
        for i, uwb in enumerate(self.modules):
            tag_ids = uwb.do_discovery(list(range(max_id)))
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