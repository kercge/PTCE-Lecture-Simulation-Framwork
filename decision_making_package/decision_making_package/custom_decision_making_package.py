#!/usr/bin/env python

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


# Constants:
TIMER_PERIOD = 0.2
PREDICTION_HORIZON = 10
Index = 0

class DecisionMakingNode(Node):
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

        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

        # Initialize the vehicle and pedestrian objects: 
        self.vehicle = Vehicle(5.0, 0.0, 0.0, 10.0, 0.0, 0.0, "decision node init")
        self.pedestrian = Pedestrian(40.0, 0.0, 0.0, 15.0, 0.0, 0.0, 0.0, "decision node init", TIMER_PERIOD)
        

        self.vehicle_collision_radius = 3.0
        self.pedestrian_personal_space_radius = 0.5

        self.is_there_a_pedestrian = False
        self.Index = 0
        # PI controller state
        self.integral_error = 0.0
        self.prev_index = self.Index
        # actuator limits
        self._accel_min = -6.0
        self._accel_max = 1.5
        # Pedestrian phase: 'none', 'approaching', 'on_crossing', 'passed'
        self.ped_phase = 'none'
        self.distance_ped_to_cross = float('inf')


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
                # Calculate distances to determine the relationship
            distance_ped_to_cross = np.linalg.norm(r_ped_1)
            # store distance for controller use
            self.distance_ped_to_cross = distance_ped_to_cross
            # classify pedestrian phase relative to the crossing
            on_cross_threshold = self.pedestrian_personal_space_radius + 0.5
            if distance_ped_to_cross <= on_cross_threshold:
                self.ped_phase = 'on_crossing'
            elif np.dot(v_ped_1, r_ped_1) < 0:
                # moving towards the crosswalk
                self.ped_phase = 'approaching'
            else:
                # moving away from crosswalk or already crossed
                self.ped_phase = 'passed'
            distance_veh_to_cross = np.linalg.norm(r_veh_1)
            if np.linalg.norm(v_ped_1) > 0:
                if distance_veh_to_cross <= 20:

                    if np.dot(v_veh_1, r_veh_1) < 0 and distance_ped_to_cross <= distance_veh_to_cross:
                    

                # Pedestrian is closer to the crosswalk
                        if np.dot(v_ped_1, r_ped_1) < 0:
                # Pedestrian is moving towards the crosswalk
                            self.Index = 0 #--
                        else:
                            self.Index = 1 #-
                    else:
                        self.Index = 2 #+
                else:
                    self.Index = 2
            else:
                self.Index = 2


        else:
            # Something went wrong.
            self.get_logger().error("Invalid PedestrianIntentionStateList length")

    
    # def simple_controller_and_publish(self):
            
            
        # Simple P controller to keep the desired velocity.
                
    def simple_controller_and_publish(self):
        """PI controller with anti-windup and reset on mode (Index) change.

        Modes (Index):
          0 -> brake to 0.0 m/s (strong braking)
          1 -> brake to 0.0 m/s (mild braking)
          2 -> accelerate to 8.0 m/s
        """
        # Reset integral on mode change
        if self.Index != self.prev_index:
            self.integral_error = 0.0
            self.prev_index = self.Index

        # set defaults
        setpoint = self.vehicle.x_speed
        Kp = 0.0
        Ki = 0.0

        if self.Index == 0:
            setpoint = 0.0
            Kp = 0.5
            Ki = 0.1
        elif self.Index == 1:
            setpoint = 0.0
            Kp = 0.3
            Ki = 0.08
        elif self.Index == 2:
            setpoint = 8.0
            Kp = 0.4
            Ki = 0.15
        else:
            # unknown mode
            self.ehmi_decision_result_message.vehicle_acceleration_desired = 0.0
            self.decision_result_publisher.publish(self.ehmi_decision_result_message)
            return

        # Adjust behaviour based on pedestrian phase (before/on/after crossing)
        # and the pedestrian's intention value (0..1 where higher = more likely to cross).
        # - approaching: slow down earlier (lower setpoint, higher gains) if intention is high
        # - on_crossing: full stop (always)
        # - passed: resume to normal behaviour
        ped_intention = getattr(self.pedestrian, 'intention', None)
        # thresholds
        intent_to_cross_thresh = 0.6
        intent_uncertain_thresh = 0.4

        if hasattr(self, 'ped_phase'):
            if self.ped_phase == 'approaching':
                # If pedestrian likely intends to cross, be more conservative.
                if ped_intention is not None and ped_intention >= intent_to_cross_thresh:
                    setpoint = min(setpoint, 2.0)  # slow down more
                    Kp *= 1.4
                    Ki *= 1.4
                elif ped_intention is not None and ped_intention >= intent_uncertain_thresh:
                    setpoint = min(setpoint, 3.0)
                    Kp *= 1.2
                    Ki *= 1.2
                else:
                    # low intention: mild reduction only if very close
                    if self.distance_ped_to_cross < 2.0:
                        setpoint = min(setpoint, 3.5)
                        Kp *= 1.1
                        Ki *= 1.05
            elif self.ped_phase == 'on_crossing':
                # pedestrian is on the crosswalk: stop the vehicle
                setpoint = 0.0
                # make controller more aggressive to ensure stopping
                Kp = max(Kp, 0.9)
                Ki = max(Ki, 0.25)
                # if pedestrian intention is very low despite on_crossing, still stop but with slightly lower integral
                if ped_intention is not None and ped_intention < intent_uncertain_thresh:
                    Ki *= 0.8
            elif self.ped_phase == 'passed':
                # pedestrian has passed: be willing to accelerate back to cruising
                Ki *= 1.1

        # compute error and tentative integral
        error = setpoint - self.vehicle.x_speed
        tentative_integral = self.integral_error + error * TIMER_PERIOD

        # unsaturated control
        u_unsat = Kp * error + Ki * tentative_integral

        # saturate
        u_sat = max(self._accel_min, min(u_unsat, self._accel_max))

        # anti-windup: only integrate if it doesn't drive further into saturation
        if not ((u_unsat > self._accel_max and error > 0) or (u_unsat < self._accel_min and error < 0)):
            self.integral_error = tentative_integral

        vehicle_input = u_sat

        # Publish
        self.ehmi_decision_result_message.vehicle_acceleration_desired = vehicle_input
        self.decision_result_publisher.publish(self.ehmi_decision_result_message)


    def timer_callback(self):
        # HERE IS THE CONTROLLER:
        self.simple_controller_and_publish()
        



def main(args=None):
    rclpy.init(args=args)

    decision_making_node_instance = DecisionMakingNode()
    rclpy.spin(decision_making_node_instance)
    decision_making_node_instance.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


