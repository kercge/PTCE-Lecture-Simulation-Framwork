#!/usr/bin/env python3

# Import Python:
import time
import numpy as np
from typing import List

import rclpy
from rclpy.node import Node
from decision_making_interface.msg import (
    EhmiDecisionResult,
    PedestrianIntentionStateList,
)

from std_msgs.msg import Float64

from crossing_gui_ros2_interfaces.msg import SimulationOutput
from crossing_gui_ros2_interfaces.srv import GetGuiParam

from .vehicle_class_definition import Vehicle
from .pedestrian_class_definition import Pedestrian

# Constants 
TIMER_PERIOD = 0.2
PREDICTION_HORIZON = 10
Index = 0

class DistDecisionMakingNode(Node):
    def __init__(self):
        super().__init__("decision_making_node")

        # ===== Creating Subscriber and Publisher ===== #
        self.ped_intention_subscription = self.create_subscription(PedestrianIntentionStateList, "relevant_pedestrian/intention_position", self.pedestrian_intention_state_list_callback, 10)

        self.decision_result_publisher = self.create_publisher(EhmiDecisionResult, "negotiation_node/out/decision_result", 10)

        self.get_gui_param_client = self.create_client(GetGuiParam, '/gui/get_gui_param')
        while not self.get_gui_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/gui/get_gui_param service not available, waiting...')

        # Initialize the decision result message:
        self.ehmi_decision_result_message= EhmiDecisionResult()
        self.ehmi_decision_result_message.vehicle_acceleration_desired = 0.0 # float64

        self.integral_error = 0.0
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        

        # Initialize the vehicle and pedestrian objects: 
        self.vehicle = Vehicle(5.0, 0.0, 0.0, 10.0, 0.0, 0.0, "decision node init")
        self.pedestrian = Pedestrian(40.0, 0.0, 0.0, 15.0, 0.0, 0.0, 0.0, "decision node init", TIMER_PERIOD)
        

        self.vehicle_collision_radius = 3.0
        self.pedestrian_personal_space_radius = 0.5

        self.is_there_a_pedestrian = False
        self.Index = 0


    # Pedestrian intention state list callback
    def pedestrian_intention_state_list_callback(self, ped_list: PedestrianIntentionStateList) -> None:
        """
        Callback function for handling PedestrianIntentionStateList.

        Args:
            ped_list (PedestrianIntentionStateList): List of pedestrian intention states.

        PedestrianIntentionStateList message definition:
            std_msgs/Header header
            PedestrianIntentionState[] pedestrian_states
                std_msgs/Header header
                string id
                float64 intention
                float64[2] position
                float64[2] velocity
        """

        # Check how long the PedestrianIntentionStateList is:
        num_pedestrians = len(ped_list.pedestrian_states)
        if num_pedestrians == 0:
            # No data.
            self.get_logger().error("No data received, where am I? - existential crisis.")

        elif num_pedestrians > 0:
            # Only pedestrian crossing position is available.

            # First get the relative position and velocity of the vehicle and pedestrian to the pedestrian crossing:
            # 0 means relative to vehicle
            # 1 means relative to pedestrian crossing
            # Rule: v_P20 = v_P21 + V_10 + w_10 x rho_P21 # see Dinamika jegyzet - relativ kinematika p93

            # Since the pedestrian crossing is a fixed point, its relative position and velocity are just the 
            # opposite of the vehicles relative position and velocity to the pedestrian crossing:

            # Vector from vehicle to pedestrian crossing:
            r_pedcross_0 = np.array([ped_list.pedestrian_states[0].position[0],
                                ped_list.pedestrian_states[0].position[1]])

            # Vector from pedestrian crossing to vehicle:
            r_veh_0 = - r_pedcross_0

            # Vehicle velocity vector relative to the pedestrian crossing:
            v_veh_0 = - np.array([ped_list.pedestrian_states[0].velocity[0],
                                ped_list.pedestrian_states[0].velocity[1]])
            
            # The orientation of the vehicle coord sys and the pedestrian crossing coord sys is the same, so
            # transformation in this case is not necessary:
            r_veh_1 = r_veh_0
            v_veh_1 = v_veh_0

            # Update the vehicle state:
            self.vehicle.state = np.array([ r_veh_1[0],     # x
                                            v_veh_1[0],     # xd
                                            r_veh_1[1],     # y
                                            v_veh_1[1] ])   # yd

            if num_pedestrians > 1:
                # Pedestrian crossing and at least 1 pedestrian data are available.
                # We assume that the first pedestrian in the list is the relevant one.
                self.is_there_a_pedestrian = True

                 # Vector from vehicle to pedestrian:
                r_ped_0 = np.array([ped_list.pedestrian_states[1].position[0],
                                    ped_list.pedestrian_states[1].position[1]])

                # Vector from pedestrian crossing to pedestrian:
                r_ped_1 = r_ped_0 - r_pedcross_0
                
                # Velocity vector of the pedestrian relative to the vehicle:
                v_ped_0 = np.array([ped_list.pedestrian_states[1].velocity[0],
                                    ped_list.pedestrian_states[1].velocity[1]])

                # Velocity vector of the pedestrian relative to the pedestrian crossing:
                # (Trafo from veh coord sys to ped cros coord sys is again not necessary.)
                v_ped_1 = v_ped_0 + v_veh_0 + np.cross([0.0, 0.0, 0.0], [r_ped_0[0], r_ped_0[1], 0.0])[:2]

                # Update the pedestrian state:
                self.pedestrian.update_pedestrian_state([r_ped_1[0], # x
                                                        v_ped_1[0], # xd
                                                        r_ped_1[1], # y
                                                        v_ped_1[1], # yd
                                                        ped_list.pedestrian_states[1].intention, # intention
                                                        ped_list.pedestrian_states[1].id]) # displaymessage

            # --- Decision logic --- #
            distance_ped_to_cross = np.linalg.norm(r_ped_1)
            distance_veh_to_cross = np.linalg.norm(r_veh_0)

            # Adding intention value
            intention_value = float(self.pedestrian.intention)
            intention_value = np.clip(intention_value, 0.0, 1.0)

            # Distance-based risk factor (0..1) 
            max_dist = 20.0   # detection distance [m]
            distance_risk = np.clip(1.0 - (distance_ped_to_cross / max_dist), 0.0, 1.0)


            if np.linalg.norm(v_ped_1) > 0:
                if distance_veh_to_cross <= 30:
                    if np.dot(v_ped_1, r_ped_1) < 0:
                        pedestrian_state = "approaching"

                        # --- Intention-aware zone classification --- #
                        if distance_ped_to_cross > 10.0:
                            zone = "safe"
                            zone_index = 0
                        elif 5.0 < distance_ped_to_cross <= 10.0:
                            zone = "near"
                            zone_index = 1
                        elif 1.0 < distance_ped_to_cross <= 5.0:
                            zone = "danger"
                            zone_index = 2
                        else:
                            zone = "collision"
                            zone_index = 3

                        # --- Intention effect ---
                        if intention_value > 0.7:
                            zone_index = min(zone_index + 1, 3)
                        elif intention_value < 0.3:
                            zone_index = max(zone_index - 1, 0)

                        # The distance risk also modifies the zone index 
                        # If distance_risk > 0.6 → strong braking (increase index)
                        if distance_risk > 0.6:
                            zone_index = min(zone_index + 1, 3)
                        elif distance_risk < 0.2:
                            zone_index = max(zone_index - 1, 0)

                        self.Index = zone_index

                    elif abs(distance_ped_to_cross) <= 1.0:
                        pedestrian_state = "crossing"
                        zone = "crossing"
                        self.Index = 3

                    elif np.dot(v_ped_1, r_ped_1) > 0 and distance_ped_to_cross > 1.0:
                        pedestrian_state = "passed"
                        zone = "passed"
                        self.Index = 4

                else:
                    pedestrian_state = "far"
                    zone = "safe"
                    self.Index = 0

            else:
                pedestrian_state = "idle"
                zone = "safe"
                self.Index = 0


            # LOG
            self.get_logger().info(
                f"Pedestrian: {pedestrian_state}, zone: {zone}, "
                f"Intention={intention_value:.2f}, DistanceRisk={distance_risk:.2f}, Index={self.Index}"
            )


    # PI Controller
    def simple_controller_and_publish(self):
        """
        Adaptive PI controller tuned to pedestrian proximity, intention, and state.
        """
        dt = TIMER_PERIOD

        # Define control parameters based on Index 
        if self.Index == 0:  # Safe zone / far / idle
            base_speed = 8.0 
            Kp = 0.4 
            Ki = 0.1 

        elif self.Index == 1:  # Near zone
            base_speed = 4.0 
            Kp = 0.5 
            Ki = 0.15 

        elif self.Index == 2:  # Danger zone
            base_speed = 1.5 
            Kp = 0.6 
            Ki = 0.2 

        elif self.Index == 3:  # Collision zone / Crossing pedestrian
            base_speed = 0.0
            Kp = 0.8
            Ki = 0.25

        elif self.Index == 4:  # Passed  
            base_speed = 6.0 
            Kp = 0.35 
            Ki = 0.08 

        else:
            base_speed = 8.0
            Kp = 0.4
            Ki = 0.1

        # Distance_risk integration 
        # The closer the pedestrian → the lower the desired speed will be
        # --- NEW: distance-based damping ---
        # distance_risk = np.clip(1.0 - distance_ped_to_cross / 15.0, 0.0, 1.0)
        # desired_speed = desired_speed * (1.0 - 0.7 * distance_risk)
        # --- NEW: Distance factor (smooth braking control) --- #
        # pedestrian_distance = np.linalg.norm(self.pedestrian.position)
        # pedestrian_distance = np.clip(pedestrian_distance, 0.5, 20.0)
        #
        # distance_factor = (20.0 - pedestrian_distance) / 20.0
        # distance_factor = np.clip(distance_factor, 0.0, 1.0)

        # The closer the pedestrian, the stronger the braking
        # desired_speed *= (1.0 - 0.7 * distance_factor)

        # --- PI control --- #
        # error = desired_speed - self.vehicle.x_speed  # use this when commented out upper logic is active
        error = base_speed - self.vehicle.x_speed
        self.integral_error += error * dt
        self.integral_error = max(-10.0, min(self.integral_error, 10.0))


        
        vehicle_input = Kp * error + Ki * self.integral_error
        vehicle_input = max(-6.0, min(vehicle_input, 1.5))

        # Publish control command
        self.ehmi_decision_result_message.vehicle_acceleration_desired = vehicle_input
        self.decision_result_publisher.publish(self.ehmi_decision_result_message)



    def timer_callback(self):
        self.simple_controller_and_publish()



def main(args=None):
    rclpy.init(args=args)
    node = DistDecisionMakingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()