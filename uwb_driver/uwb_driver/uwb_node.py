#!/usr/bin/env python3
from pyuwb import UwbModule, find_uwb_serial_ports
from uwb_msgs.msg import RangeStamped, PassiveStamped
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
from schedulers import CommonListScheduler
from time import sleep
import threading

class UwbModuleNode(Node):
    def __init__(self):
        super().__init__("uwb")

        
    # def load_params(self):
    #     if not rospy.has_param("/uwb/scheduler"):
    #         rospy.loginfo(
    #             "Did not detect required params on parameter server. Loading from YAML file."
    #         )
    #         rp = rospkg.RosPack()
    #         path = rp.get_path("uwb_ros")
    #         filename = path + "/config/params.yaml"

    #         with open(filename) as file:
    #             params = yaml.safe_load(file)

    #         if not "ports" in params.keys():
    #             params["ports"] = []

    #         if params["ports"] is None:
    #             params["ports"] = []


    #         rospy.set_param("/uwb/", params)
    def start(self):
        # self.load_params()

        ports = find_uwb_serial_ports()
        while len(ports) == 0 and rclpy.ok():
            self.get_logger().warn("No UWB modules found. Will keep trying.", once=True)
            sleep(2)
            if rclpy.ok():
                ports = find_uwb_serial_ports()

        self.modules = [UwbModule(port, timeout=0.1, verbose=False, threaded=False) for port in ports]\

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
            sleep(5)
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

        # if not rospy.has_param("/uwb/scheduler"):
        #     rospy.logerr("No scheduler specified in param file. Exiting.")
        # else:
        #     scheduler = rospy.get_param("/uwb/scheduler")
        #     if scheduler == "slow":
        self.start_slow_scheduler()

        #     elif scheduler == "common_list":
        # seq = [
        #     (x["from_id"], x["to_id"])
        #     for x in rospy.get_param("/uwb/sequence")
        # ]
        # scheduler = CommonListScheduler(
        #     self.modules, self.my_ids, seq, self.publish_range, self.publish_passive
        # )
        # scheduler.start()

    def discover(self):
        """
        Performs neighbor discovery
        """
        # Get neighbours
        neighbour_ids = []
        # if rospy.has_param("/uwb/max_id"):
        #     max_id = rospy.get_param("/uwb/max_id")
        # else:
        #     rospy.logwarn("Max tag ID not specified in param file. Using 10.")
        max_id = 10

        rate = self.create_rate(1/1.5)
        for i in range(max_id):
            if i in self.my_ids:
                idx = self.my_ids.index(i)
                uwb = self.modules[idx]
                tag_ids = uwb.do_discovery()
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
        rate = self.create_rate(50 / (len(self.neighbour_ids) + 1))
        while rclpy.ok():
            for i, uwb in enumerate(self.modules):
                my_id = self.my_ids[i]
                for nb_id in self.neighbour_ids:
                    if nb_id != my_id:

                        range_data = uwb.do_twr(target_id=nb_id, mult_twr=True)
                        if range_data["is_valid"]:
                            self.publish_range((my_id, nb_id), range_data)

                    try:  # Prevent garbage in console output when killed
                        rate.sleep()
                    except ROSInterruptException:
                        pass


if __name__ == "__main__":
    rclpy.init()

    # Spin in a separate thread
    # https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
    # Wow, ROS2 API hurts right now
    node = UwbModuleNode()
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()


    node.start()
