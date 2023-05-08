#!/usr/bin/env python3
from typing import Callable, List
from pyuwb import UwbModule
import rclpy 
import random

class CommonListScheduler:
    """
    The "common list" scheduling scheme depends on a pre-determined list of
    pairs (the sequence) that each agent should have. By listening to all
    other measurements, each agent can determine where the team is in this
    sequence of range measurements that need to be performed.
    """

    def __init__(
        self,
        modules: List[UwbModule],
        my_ids: List[int],
        sequence: List[tuple],
        publish_func_range: Callable,
        publish_func_passive: Callable,
    ):
        self._my_ids = my_ids
        self._modules = modules
        self._sequence = sequence
        self._publish_range = publish_func_range
        self._publish_passive = publish_func_passive
        self._logger = rclpy.logging.get_logger("rclpy")
        self._latest_pair = None
        self._ranging_event = False

    def listening_callback(self, data, my_id):
        """
        This callback is triggered when a range measurement between OTHER
        two tags was heard.
        """
        self._latest_pair = (data[0], data[1])
        self._ranging_event = True

        if data[2] != 0: 
            # Data is valid
            msg = {}
            msg['rx1'] = data[2]
            msg['rx2'] = data[3]
            msg['rx3'] = data[4]
            msg['tx1_n'] = data[5]
            msg['rx1_n'] = data[6]
            msg['tx2_n'] = data[7]
            msg['rx2_n'] = data[8]
            msg['tx3_n'] = data[9]
            msg['rx3_n'] = data[10]
            msg['fpp1'] = data[11]
            msg['fpp2'] = data[12]
            msg['fpp3'] = data[13]
            msg['skew1'] = data[14]
            msg['skew2'] = data[15]
            msg['skew3'] = data[16]
            msg['fpp1_n'] = data[17]
            msg['fpp2_n'] = data[18]
            msg['skew1_n'] = data[19]
            msg['skew2_n'] = data[20]
            self._publish_passive(self._latest_pair, msg, my_id)

    def ranging_with_me_callback(self, data, my_id):
        """
        This callback is triggered when someone initiates ranging with us.
        """
        self._latest_pair = (data[0], my_id)
        self._ranging_event = True

        if data[0] not in self._my_ids:
            msg = {}
            msg["is_valid"] = True
            msg['range'] = data[1]
            msg['tx1'] = data[2]
            msg['rx1'] = data[3]
            msg['tx2'] = data[4]
            msg['rx2'] = data[5]
            msg['tx3'] = data[6]
            msg['rx3'] = data[7]
            msg['fpp1'] = data[8]
            msg['fpp2'] = data[9]
            msg['skew1'] = data[10]
            msg['skew2'] = data[11]
            self._publish_range(self._latest_pair, msg)

    def handle_ranging_event(self, most_recent_pair):
        """
        This is the main function which executes the core functionality in the
        decentralized scheme. By referring to the sequence, we can determine
        whether it is our turn to do ranging.
        """

        # Look up where we are in the sequence
        next_pair = self.get_next_pair(most_recent_pair)

        # If its our turn, do ranging.
        if next_pair[0] in self._my_ids:
            idx = self._my_ids.index(next_pair[0])
            uwb = self._modules[idx]

            # Initiate ranging with another tag.
            range_data = uwb.do_twr(
                target_id=next_pair[1], meas_at_target=True, mult_twr=True
            )

            self._latest_pair = next_pair
            self._ranging_event = True

            if range_data["is_valid"]:
                # Ranged successfully.
                self._publish_range(next_pair, range_data)
                self._latest_pair = next_pair
            else:
                self._logger.warn("TWR failed for pair " + str(next_pair))

    def get_next_pair(self, current_pair):
        """
        Cycles through the sequence list.
        """
        if current_pair is None:
            next_pair = self._sequence[0]

        elif current_pair in self._sequence:
            idx = self._sequence.index(current_pair)
            if idx == len(self._sequence) - 1:
                idx = 0
            else:
                idx += 1
            next_pair = self._sequence[idx]

        else:
            self._logger.warn(
                "Detected a ranging pair that is not in my sequence. \
                Starting from beginning."
            )
            next_pair = self._sequence[0]
            
        return next_pair