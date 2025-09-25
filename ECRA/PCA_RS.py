import numpy as np
import random
import argparse
import pandas as pd
from utils import ResourceSelectionInitial, CounterConsecutiveNumber, Delay_list, Distance, RSSI, RSSIratePercent, \
    CalculateSINR_fading
from random import choice
import math
import time
from collections import defaultdict, deque


class EnhancedMetrics:
    """performance metrics collection"""

    def __init__(self):
        self.hidden_node_collisions_avoided = 0
        self.merging_collisions_avoided = 0

        self.boundary_segment_activations = 0
        self.resource_reuse_efficiency = []
        self.hidden_node_detection_count = 0
        # self.detection_accuracy = 0
        self.temp_resource_usage_count = 0
        self.collision_predictions = []

    def update(self, metric_type, value=1):
        if metric_type == 'hidden_node_avoided':
            self.hidden_node_collisions_avoided += value
        elif metric_type == 'merging_avoided':

            self.merging_collisions_avoided += value
        elif metric_type == 'boundary_activated':
            self.boundary_segment_activations += value
        elif metric_type == 'hidden_node_detected':
            self.hidden_node_detection_count += value
            # self.hidden_node_detection_count = self.hidden_node_detection_count + value
        elif metric_type == 'temp_resource_used':
            self.temp_resource_usage_count += value

    def add_efficiency_sample(self, total_resources, used_resources):
        if total_resources > 0:
            efficiency = len(set(used_resources)) / total_resources

            self.resource_reuse_efficiency.append(efficiency)

    def get_summary(self):
        avg_efficiency = np.mean(self.resource_reuse_efficiency) if self.resource_reuse_efficiency else 0
        # avg_eff = sum(self.resource_reuse_efficiency)/len(self.resource_reuse_efficiency) if self.resource_reuse_efficiency else 0

        return {
            'hidden_collisions_avoided': self.hidden_node_collisions_avoided,
            'merging_collisions_avoided': self.merging_collisions_avoided,
            'boundary_activations': self.boundary_segment_activations,
            'hidden_nodes_detected': self.hidden_node_detection_count,
            'temp_resource_usage': self.temp_resource_usage_count,
            'avg_resource_efficiency': avg_efficiency
        }


class ECRA_RS_Enhanced:

    def __init__(self, epsilon=0.1, delta=0.1, c_threshold=5, rssi_threshold=0.5,
                 lane_spacing=5, sensing_range=300):
        """
        Initialize ECRA-RS parameters
        """
        self.epsilon = epsilon
        self.delta = delta

        self.c_threshold = c_threshold
        self.rssi_threshold = rssi_threshold
        self.lane_spacing = lane_spacing
        self.sensing_range = sensing_range
        # self.sensing_distance = sensing_range

        # boundary segment length s, initially 0
        self.boundary_segment_lengths = defaultdict(float)

        # platoon information storage
        self.platoon_info = {}
        self.hidden_node_info = {}
        self.neighbor_history = defaultdict(deque)  # neighbor history records

        # half duplex constraint tracking
        self.transmission_history = defaultdict(list)

        # performance metrics
        self.metrics = EnhancedMetrics()

    def resource_partition(self, total_subchannels, vehicle_direction):
        # Resource partition: divide resources into Psi_W and Psi_E
        half_point = total_subchannels // 2
        # half = int(total_subchannels/2)

        if vehicle_direction == 0:  # westbound
            return list(range(0, half_point))

        else:  # eastbound
            return list(range(half_point, total_subchannels))

    def calculate_boundary_segments(self, platoon_id, leader_pos, last_member_pos,
                                    leader_lane, num_lanes, is_westbound):
        """
        calculate boundary segments for platoon leader
        Returns:
            boundary segment list, each element is (start_x, end_x, y_coord, lane)
        """
        boundary_segments = []

        s = self.boundary_segment_lengths.get(platoon_id, 0)  # get boundary segment length for this platoon

        # calculate max lane offset for possible boundary segments
        max_m = min(num_lanes, int(self.sensing_range / self.lane_spacing) + leader_lane - 1)

        for m in range(1, max_m + 1):
            # calc vertical distance to opposite lane
            vertical_distance = (m + leader_lane - 1) * self.lane_spacing
            if vertical_distance >= self.sensing_range:
                continue
            horizontal_distance = math.sqrt(self.sensing_range ** 2 - vertical_distance ** 2)

            if is_westbound:
                y_coord = self.lane_spacing * (m + leader_lane - 1) / 2
                # y_position = y_coord
                # boundary segment x coordinates (at edge of last member sensing range)
                x_start = last_member_pos[0] + horizontal_distance - s
                x_end = last_member_pos[0] + horizontal_distance

            else:
                y_coord = -self.lane_spacing * (m + leader_lane - 1) / 2
                x_start = last_member_pos[0] - horizontal_distance
                x_end = last_member_pos[0] - horizontal_distance + s

            boundary_segments.append({
                'x_start': x_start,
                'x_end': x_end,
                'y_coord': y_coord,
                'lane_offset': m
            })

        return boundary_segments

    def is_in_boundary_segment(self, vehicle_pos, boundary_segments):
        """check if vehicle is in any boundary segment"""
        x, y = vehicle_pos[0], vehicle_pos[1]

        # x_pos = x
        # y_pos = y

        for segment in boundary_segments:
            x_in_range = min(segment['x_start'], segment['x_end']) <= x <= max(segment['x_start'], segment['x_end'])
            y_in_range = abs(y - segment['y_coord']) < self.lane_spacing / 2

            if x_in_range and y_in_range:
                self.metrics.update('boundary_activated')

                return True

        return False

    def detect_hidden_nodes(self, leader_pos, last_member_pos, neighbors_info):
        # detect hidden nodes
        hidden_nodes = []

        for neighbor in neighbors_info:
            neighbor_pos = neighbor.get('position', [0, 0])

            # calc distance to leader and last member
            dist_to_leader = math.sqrt((neighbor_pos[0] - leader_pos[0]) ** 2 +
                                       (neighbor_pos[1] - leader_pos[1]) ** 2)
            # distance_leader = dist_to_leader
            dist_to_last = math.sqrt((neighbor_pos[0] - last_member_pos[0]) ** 2 +
                                     (neighbor_pos[1] - last_member_pos[1]) ** 2)

            # determine if hidden node
            if dist_to_last < self.sensing_range and dist_to_leader > self.sensing_range:
                hidden_nodes.append(neighbor)

                self.metrics.update('hidden_node_detected')

        return hidden_nodes

    def check_half_duplex_constraint(self, vehicle_id, resource, current_time, time_window=1000):
        """check half duplex constraint"""
        history = self.transmission_history.get(vehicle_id, [])

        # check if there are transmissions within time window
        for past_time, past_resource in history:
            if current_time - past_time < time_window:
                # cannot receive if transmitting within time window
                return False
        return True

    def last_member_process(self, last_member_id, platoon_id, leader_id,
                            ResourceSelectionall, NumVehicle, VehicleLocation,
                            SubchannelNum, TransmitPower_mw, current_time):
        # platoon last member resource selection process
        last_member_pos = VehicleLocation[last_member_id]
        leader_pos = VehicleLocation[leader_id]

        # S1: collect neighbor info and measure RSRP
        neighbor_info = []
        excluded_resources = set()

        for j in range(NumVehicle):
            if j == last_member_id:
                continue

            dist = Distance(last_member_id, j, VehicleLocation)
            if dist < self.sensing_range:
                neighbor_data = {
                    'id': j,
                    'position': list(VehicleLocation[j]),
                    'resource': ResourceSelectionall[j],
                    'distance': dist,
                    'timestamp': current_time
                }
                neighbor_info.append(neighbor_data)
                # neighbors.append(neighbor_data)

                if 0 <= ResourceSelectionall[j] < SubchannelNum:
                    # use path loss model
                    pl = 128.1 + 37.6 * np.log10(dist / 1000.0) if dist > 0 else 128.1
                    rsrp = 10 * np.log10(TransmitPower_mw) - pl

                    if rsrp > self.rssi_threshold:
                        excluded_resources.add(ResourceSelectionall[j])

        # S2: detect hidden nodes
        hidden_nodes = self.detect_hidden_nodes(leader_pos, last_member_pos, neighbor_info)

        self.hidden_node_info[platoon_id] = {
            'last_member_id': last_member_id,
            'neighbors': neighbor_info,
            'hidden_nodes': hidden_nodes,
            'timestamp': current_time,
            'excluded_by_half_duplex': []
        }

        is_westbound = last_member_pos[0] > leader_pos[0]
        direction = 0 if is_westbound else 1

        directional_resources = self.resource_partition(SubchannelNum, direction)

        # S3: create available resource list
        available_resources = [r for r in directional_resources if r not in excluded_resources]

        # S4: check available resource count
        rssi_threshold_adjustment = 0
        while len(available_resources) < max(1, int(self.epsilon * SubchannelNum)):
            rssi_threshold_adjustment += 3  # increase 3dB
            # rssi_adjust = rssi_threshold_adjustment + 3

            available_resources = []
            for r in directional_resources:
                if r not in excluded_resources:
                    available_resources.append(r)

                elif rssi_threshold_adjustment > 0:
                    # recheck with adjusted threshold
                    max_rsrp = -200
                    for neighbor in neighbor_info:
                        if neighbor['resource'] == r:
                            dist = neighbor['distance']

                            if dist > 0:
                                pl = 128.1 + 37.6 * np.log10(dist / 1000.0)
                                rsrp = 10 * np.log10(TransmitPower_mw) - pl
                                max_rsrp = max(max_rsrp, rsrp)

                    if max_rsrp <= self.rssi_threshold + rssi_threshold_adjustment:
                        available_resources.append(r)

            # prevent infinite loop
            if rssi_threshold_adjustment > 20:
                available_resources = directional_resources
                break

        # S5-S7: RSSI based resource selection
        if available_resources:
            resource_rssi = []

            for resource in available_resources:
                rssi_sum = 0
                count = 0

                for neighbor in neighbor_info:
                    if neighbor['resource'] == resource:
                        dist = neighbor['distance']
                        if dist > 0:
                            pl = 128.1 + 37.6 * np.log10(dist / 1000.0)

                            rssi = 10 * np.log10(TransmitPower_mw) - pl
                            rssi_sum += rssi
                            count += 1

                avg_rssi = rssi_sum / count if count > 0 else -110
                resource_rssi.append((resource, avg_rssi))

            # sort and select
            resource_rssi.sort(key=lambda x: x[1])
            num_candidates = max(1, int(self.epsilon * len(resource_rssi)))
            candidate_resources = [r[0] for r in resource_rssi[:num_candidates]]

            selected_resource = random.choice(candidate_resources)

        else:
            selected_resource = random.choice(directional_resources)

        # record transmission history
        self.transmission_history[last_member_id].append((current_time, selected_resource))

        return selected_resource

    def leader_process(self, leader_id, platoon_id, last_member_id,
                       ResourceSelectionall, NumVehicle, VehicleLocation,
                       SubchannelNum, TransmitPower_mw, leader_lane,
                       num_lanes, current_time):
        """platoon leader resource selection process"""

        leader_pos = VehicleLocation[leader_id]
        last_member_pos = VehicleLocation[last_member_id]

        # determine direction
        is_westbound = leader_pos[0] > last_member_pos[0]
        direction = 0 if is_westbound else 1

        # S1: check neighbor SCI messages
        excluded_resources = set()
        neighbor_rssi = {}

        for j in range(NumVehicle):
            if j == leader_id:
                continue

            dist = Distance(leader_id, j, VehicleLocation)

            if dist < self.sensing_range and 0 <= ResourceSelectionall[j] < SubchannelNum:
                pl = 128.1 + 37.6 * np.log10(dist / 1000.0) if dist > 0 else 128.1
                rsrp = 10 * np.log10(TransmitPower_mw) - pl

                if rsrp > self.rssi_threshold:
                    excluded_resources.add(ResourceSelectionall[j])
                    neighbor_rssi[ResourceSelectionall[j]] = rsrp

        # S2: process hidden node info
        hidden_resources = set()
        boundary_segments = []

        if platoon_id in self.hidden_node_info:
            hidden_info = self.hidden_node_info[platoon_id]

            # calculate boundary segments
            boundary_segments = self.calculate_boundary_segments(
                platoon_id, leader_pos, last_member_pos,
                leader_lane, num_lanes, is_westbound
            )

            # process hidden nodes
            for hidden_node in hidden_info.get('hidden_nodes', []):
                resource = hidden_node.get('resource', -1)

                if 0 <= resource < SubchannelNum:
                    node_pos = hidden_node.get('position', [0, 0])

                    # check if in boundary segment
                    if not self.is_in_boundary_segment(node_pos, boundary_segments):
                        hidden_resources.add(resource)
                        self.metrics.update('hidden_node_avoided')

        excluded_resources.update(hidden_resources)

        # S3: create available resource list
        directional_resources = self.resource_partition(SubchannelNum, direction)
        available_resources = []

        for resource in directional_resources:
            if resource not in excluded_resources:
                available_resources.append(resource)

        if boundary_segments:
            for j in range(NumVehicle):
                if j != leader_id and 0 <= ResourceSelectionall[j] < SubchannelNum:

                    if self.is_in_boundary_segment(VehicleLocation[j], boundary_segments):
                        if ResourceSelectionall[j] not in available_resources:
                            available_resources.append(ResourceSelectionall[j])

        # S4: check available resource count
        while len(available_resources) < max(1, int(self.epsilon * SubchannelNum)):
            # increase boundary segment length
            self.boundary_segment_lengths[platoon_id] += self.delta

            if platoon_id in self.hidden_node_info:
                boundary_segments = self.calculate_boundary_segments(
                    platoon_id, leader_pos, last_member_pos,
                    leader_lane, num_lanes, is_westbound
                )

                # re-evaluate available resources
                for j in range(NumVehicle):
                    if j != leader_id and 0 <= ResourceSelectionall[j] < SubchannelNum:
                        if self.is_in_boundary_segment(VehicleLocation[j], boundary_segments):

                            if ResourceSelectionall[j] not in available_resources:
                                available_resources.append(ResourceSelectionall[j])

            # prevent infinite loop
            if self.boundary_segment_lengths[platoon_id] > 100:
                break

        # S5-S7: RSSI based resource selection
        if available_resources:
            resource_rssi = []

            for resource in available_resources:
                rssi_sum = 0
                count = 0

                for j in range(NumVehicle):
                    if j != leader_id and ResourceSelectionall[j] == resource:
                        dist = Distance(leader_id, j, VehicleLocation)

                        if 0 < dist < self.sensing_range:
                            pl = 128.1 + 37.6 * np.log10(dist / 1000.0)
                            rssi = 10 * np.log10(TransmitPower_mw) - pl
                            rssi_sum += rssi
                            count += 1

                avg_rssi = rssi_sum / count if count > 0 else -110
                resource_rssi.append((resource, avg_rssi))

            # sort and select
            resource_rssi.sort(key=lambda x: x[1])
            num_candidates = max(1, int(self.epsilon * len(resource_rssi)))

            candidate_resources = [r[0] for r in resource_rssi[:num_candidates]]

            selected_resource = random.choice(candidate_resources)
        else:
            selected_resource = random.choice(directional_resources)

        self.transmission_history[leader_id].append((current_time, selected_resource))
        self.metrics.add_efficiency_sample(SubchannelNum, available_resources)

        return selected_resource

    def non_platoon_vehicle_process(self, vehicle_id, current_position,
                                    ResourceSelectionall, NumVehicle, VehicleLocation,
                                    SubchannelNum, TransmitPower_mw, current_time,
                                    in_detection_mode=False, temp_resource=-1,
                                    original_resource=-1):
        """non-platoon vehicle resource selection - includes merging collision detection"""

        # collect current neighbor info
        current_neighbors = []
        for j in range(NumVehicle):
            if j != vehicle_id:
                dist = Distance(vehicle_id, j, VehicleLocation)

                if dist < self.sensing_range:
                    current_neighbors.append({
                        'id': j,
                        'resource': ResourceSelectionall[j],
                        'distance': dist,
                        'position': list(VehicleLocation[j])
                    })

        # get history neighbor info
        history = self.neighbor_history[vehicle_id]
        if len(history) > 0:
            last_record = history[-1]

            last_position = last_record['position']
            last_neighbors = last_record['neighbors']

            y_change = abs(current_position[1] - last_position[1]) > 0.1

            current_ids = {n['id'] for n in current_neighbors}
            last_ids = {n['id'] for n in last_neighbors}

            neighbor_change = len(current_ids.symmetric_difference(last_ids))

            if (y_change or neighbor_change >= self.c_threshold) and not in_detection_mode:
                # enter detection mode
                self.metrics.update('temp_resource_used')

                # select temp resource
                is_westbound = current_position[0] < last_position[0]
                direction = 0 if is_westbound else 1

                directional_resources = self.resource_partition(SubchannelNum, direction)

                # exclude neighbor occupied resources
                occupied_resources = {n['resource'] for n in current_neighbors
                                      if 0 <= n['resource'] < SubchannelNum}

                available_temp = [r for r in directional_resources
                                  if r not in occupied_resources and r != ResourceSelectionall[vehicle_id]]

                if available_temp:
                    temp_resource = random.choice(available_temp)

                    # record neighbor history
                    self.neighbor_history[vehicle_id].append({
                        'position': list(current_position),
                        'neighbors': current_neighbors,
                        'timestamp': current_time
                    })

                    # maintain history record length
                    if len(self.neighbor_history[vehicle_id]) > 10:
                        self.neighbor_history[vehicle_id].popleft()

                    return temp_resource, True, ResourceSelectionall[vehicle_id]

        # if in detection mode
        if in_detection_mode:
            collision_detected = False

            for n in current_neighbors:
                if n.get('resource', -1) == original_resource and n.get('distance', 1000) < 100:
                    collision_detected = True
                    self.metrics.update('merging_avoided')
                    break

            if collision_detected:
                # need to reselect resource
                return self._standard_resource_selection(
                    vehicle_id, current_position, ResourceSelectionall,
                    NumVehicle, VehicleLocation, SubchannelNum,
                    TransmitPower_mw, current_time
                ), False, -1
            else:
                # restore original resource
                return original_resource, False, -1

        selected_resource = self._standard_resource_selection(
            vehicle_id, current_position, ResourceSelectionall,
            NumVehicle, VehicleLocation, SubchannelNum,
            TransmitPower_mw, current_time
        )

        # record neighbor history
        self.neighbor_history[vehicle_id].append({
            'position': list(current_position),
            'neighbors': current_neighbors,
            'timestamp': current_time
        })

        # maintain history record length
        if len(self.neighbor_history[vehicle_id]) > 10:
            self.neighbor_history[vehicle_id].popleft()

        return selected_resource, False, -1

    def _standard_resource_selection(self, vehicle_id, position, ResourceSelectionall,
                                     NumVehicle, VehicleLocation, SubchannelNum,
                                     TransmitPower_mw, current_time):
        # standard resource selection process (for non-platoon and platoon regular members)

        # determine direction
        is_westbound = position[0] > VehicleLocation[0][0] if NumVehicle > 1 else True
        direction = 0 if is_westbound else 1
        directional_resources = self.resource_partition(SubchannelNum, direction)

        rssi_per_resource = defaultdict(list)
        excluded_resources = set()

        for j in range(NumVehicle):
            if j == vehicle_id:
                continue

            dist = Distance(vehicle_id, j, VehicleLocation)

            if dist < self.sensing_range and 0 <= ResourceSelectionall[j] < SubchannelNum:
                # calculate RSRP
                pl = 128.1 + 37.6 * np.log10(dist / 1000.0) if dist > 0 else 128.1
                rsrp = 10 * np.log10(TransmitPower_mw) - pl

                rssi_per_resource[ResourceSelectionall[j]].append(rsrp)

                if rsrp > self.rssi_threshold:
                    excluded_resources.add(ResourceSelectionall[j])
                    # excluded.add(ResourceSelectionall[j])

        # create available resource list
        available_resources = [r for r in directional_resources if r not in excluded_resources]

        # ensure enough available resources
        if len(available_resources) < max(1, int(self.epsilon * SubchannelNum)):
            available_resources = directional_resources
        resource_rssi = []
        for resource in available_resources:
            rssi_list = rssi_per_resource.get(resource, [-110])

            avg_rssi = np.mean(rssi_list)
            resource_rssi.append((resource, avg_rssi))

        # select lowest RSSI top epsilon*N resources
        resource_rssi.sort(key=lambda x: x[1])
        num_candidates = max(1, int(self.epsilon * len(resource_rssi)))

        candidate_resources = [r[0] for r in resource_rssi[:num_candidates]]

        selected_resource = random.choice(candidate_resources)
        # final_resource = selected_resource

        # record transmission history
        self.transmission_history[vehicle_id].append((current_time, selected_resource))

        return selected_resource

    def get_statistics(self):
        return self.metrics.get_summary()


def SimulationWithECRARS_Enhanced(ResourceSelectionini, DesiredDistance, RClist, NumVehicle, StartTime,
                                  VehicleLocation, RCrange, platoon_index, non_platoon, platoon_index_ori,
                                  LeadingNum, FollowNum, SubchannelNum, TransmitPower_mw, TransmitInterval,
                                  sinr_th, fading, scale_param_omega, maximalTime, SimulationTime,
                                  lane_info, sensing_range=300):
    ecra_rs = ECRA_RS_Enhanced(
        epsilon=0.1,
        delta=0.1,
        c_threshold=5,
        rssi_threshold=-90.5,
        lane_spacing=5,
        sensing_range=sensing_range
    )

    # initialize variables
    lowerbound, higherbound = RCrange
    ave_rc = int(np.average(RCrange))

    RSSIEachStatistic = [[] for _ in range(NumVehicle)]

    PlatoonPacketCollision = 0
    PacketCollision = 0
    Platoonalltrans = 0

    alltrans = 0

    ResourceSelectionallEachRound = ResourceSelectionini.copy()
    ResourceSelectionall = ResourceSelectionini.copy()

    platoon_mapping = {}  # leader_id -> platoon_id
    platoon_members = {}  # platoon_id -> [member_ids]

    for idx, leader_id in enumerate(platoon_index):
        platoon_id = f"platoon_{idx}"
        platoon_mapping[leader_id] = platoon_id
        members = []
        for i in range(leader_id - LeadingNum, leader_id + FollowNum + 1):
            if i in platoon_index_ori:
                members.append(i)
        platoon_members[platoon_id] = members

    non_platoon_state = {}
    for i in non_platoon:
        non_platoon_state[i] = {
            'last_position': list(VehicleLocation[i]),
            'in_detection_mode': False,

            'temp_resource': -1,
            'original_resource': -1,
            'detection_start_time': -1
        }

    state_list_r2l2 = [[1 for _ in range(SimulationTime - StartTime)] for _ in range(len(platoon_index))]

    for t in range(1, SimulationTime):
        VehicleLocation = observe_vehicles[t]

        # process each vehicle
        for i in range(NumVehicle):
            RClist[i] = RClist[i] - 1
            RSSIEach = RSSI(i, ResourceSelectionall, SubchannelNum, NumVehicle, VehicleLocation, TransmitPower_mw)
            RSSIEachStatistic[i].append(RSSIEach)
            if t < ave_rc:
                sumRSSI = np.sum(RSSIEachStatistic[i], axis=0)

                AverageRSSI = [m / t for m in sumRSSI]
            else:
                sumRSSI = np.sum(RSSIEachStatistic[i][t - ave_rc + 1:], axis=0)
                AverageRSSI = [m / ave_rc for m in sumRSSI]

            if RClist[i] == 0:
                RClist[i] = random.randint(lowerbound, higherbound + 1)

                # determine vehicle type and apply corresponding resource selection strategy
                if i in platoon_index:
                    platoon_id = platoon_mapping[i]
                    members = platoon_members[platoon_id]

                    last_member_id = members[-1] if members else i
                    leader_lane = lane_info.get('lane_positions', {}).get(i, 1)
                    num_lanes = lane_info.get('num_lanes', 2)

                    new_resource = ecra_rs.leader_process(
                        i, platoon_id, last_member_id,
                        ResourceSelectionall, NumVehicle, VehicleLocation,
                        SubchannelNum, TransmitPower_mw, leader_lane,
                        num_lanes, t
                    )

                    ResourceSelectionallEachRound[i] = new_resource

                elif i in platoon_index_ori:

                    is_last_member = False
                    leader_id = -1

                    platoon_id = None

                    for pid, members in platoon_members.items():
                        if i == members[-1]:
                            is_last_member = True
                            leader_id = members[0]

                            platoon_id = pid
                            break

                    if is_last_member and leader_id >= 0:
                        new_resource = ecra_rs.last_member_process(
                            i, platoon_id, leader_id,
                            ResourceSelectionall, NumVehicle, VehicleLocation,
                            SubchannelNum, TransmitPower_mw, t
                        )

                        ResourceSelectionallEachRound[i] = new_resource
                    else:
                        # other platoon members (use standard SPS)
                        new_resource = ecra_rs._standard_resource_selection(
                            i, VehicleLocation[i], ResourceSelectionall,
                            NumVehicle, VehicleLocation, SubchannelNum,
                            TransmitPower_mw, t
                        )
                        ResourceSelectionallEachRound[i] = new_resource


                elif i in non_platoon:
                    # non-platoon vehicle
                    vehicle_state = non_platoon_state[i]

                    new_resource, in_detection, original = ecra_rs.non_platoon_vehicle_process(
                        i, VehicleLocation[i],
                        ResourceSelectionall, NumVehicle, VehicleLocation,
                        SubchannelNum, TransmitPower_mw, t,
                        vehicle_state['in_detection_mode'],
                        vehicle_state['temp_resource'],
                        vehicle_state['original_resource']
                    )

                    # update state
                    vehicle_state['in_detection_mode'] = in_detection
                    vehicle_state['temp_resource'] = new_resource if in_detection else -1
                    vehicle_state['original_resource'] = original if in_detection else -1

                    vehicle_state['last_position'] = list(VehicleLocation[i])

                    ResourceSelectionallEachRound[i] = new_resource
                else:
                    # other vehicles (standard resource selection)
                    temp = RSSIratePercent(i, AverageRSSI, ResourceSelectionall, SubchannelNum, 0.2)

                    ResourceSelectionallEachRound[i] = choice(temp) if temp else random.randint(0, SubchannelNum - 1)

        # update all vehicle resource selections
        ResourceSelectionall = ResourceSelectionallEachRound.copy()

        # performance evaluation
        if t >= StartTime:
            # evaluate platoon vehicle communication
            for idx, i in enumerate(platoon_index):
                Platoonalltrans += 1
                transmission_success = True
                platoon_id = platoon_mapping[i]
                members = platoon_members[platoon_id]

                # check if all members successfully receive
                for j in members:
                    if i == j:
                        continue

                    alltrans += 1
                    sinr = CalculateSINR_fading(i, j, ResourceSelectionall, NumVehicle,
                                                VehicleLocation, TransmitPower_mw, fading, scale_param_omega)

                    if sinr < sinr_th:
                        PlatoonPacketCollision += 1

                        PacketCollision += 1
                        state_list_r2l2[idx][t - StartTime] = 0
                        transmission_success = False
                        break

                if transmission_success:
                    delay_in_this_period = (ResourceSelectionall[i] // 2 + 1)

                    state_list_r2l2[idx][t - StartTime] = delay_in_this_period

    ecra_stats = ecra_rs.get_statistics()

    plr = PlatoonPacketCollision / Platoonalltrans if Platoonalltrans > 0 else 0

    non_platoon_plr = PacketCollision / alltrans if alltrans > 0 else 0
    goodput = Platoonalltrans - PlatoonPacketCollision

    accumulate_1_r2l2 = [0] * len(platoon_index)
    sum_fail_r2l2 = [0] * len(platoon_index)

    accum_dd_0and1_list = [[] for _ in range(len(platoon_index))]

    for i in range(len(platoon_index)):
        accum_0_list_r2l2, accumulate_1_r2l2[i] = CounterConsecutiveNumber(state_list_r2l2[i])
        accum_dd_0and1_list[i] = Delay_list(state_list_r2l2[i], TransmitInterval)

        sum_fail_r2l2[i] = sum(x * (x > maximalTime) for x in accum_0_list_r2l2)

    sum_all_r2l2 = sum(sum_fail_r2l2) + sum(accumulate_1_r2l2)
    dop_r2l2 = sum(sum_fail_r2l2) / sum_all_r2l2 if sum_all_r2l2 > 0 else 0

    sum_coop_delay = sum(accum_dd_0and1_list, [])
    average_delay = np.mean(sum_coop_delay) if sum_coop_delay else 0

    return {
        'goodput': goodput,
        'plr': plr,
        'non_platoon_plr': non_platoon_plr,
        'dop': dop_r2l2,
        'average_delay': average_delay,

        'ecra_statistics': ecra_stats
    }


def main():
    """Enhanced PCA-RS scheme main function"""
    parser = argparse.ArgumentParser(description='PCA-RS platoon communication simulation')

    parser.add_argument('--lvn', type=int, default=2, help='leading vehicles in IFT')
    parser.add_argument('--fn', type=int, default=2, help='following vehicles in IFT')
    parser.add_argument('--r', type=int, default=60, help='simulation runs')
    parser.add_argument('--est', type=int, default=30, help='end simulation time')
    parser.add_argument('--sst', type=int, default=10, help='start sampling time')

    parser.add_argument('--td', type=float, default=200, help='beacon message target distance')
    parser.add_argument('--db', type=float, default=100, help='delay boundary')
    parser.add_argument('--fade', type=str, default='on', help='fading enabled')
    parser.add_argument('--itv', type=int, default=100, help='transmission interval')

    args = parser.parse_args()

    global NumVehicle, observe_vehicles
    sumo_data_time = 30
    observe_vehicles = [[] for _ in range(sumo_data_time)]
    data_all = np.array(pd.read_csv("realmap_vehicle_location.csv", header=None)).tolist()

    NumVehicle = int(len(data_all) / sumo_data_time)

    for i in range(sumo_data_time):
        observe_vehicles[i] = data_all[int(i * NumVehicle):int((i + 1) * NumVehicle)]

    # set simulation parameters
    LeadingNum = args.lvn
    FollowNum = args.fn
    runningtime = args.r

    SimulationTime = args.est
    StartTime = args.sst
    DesiredDistance = args.td
    threshold = args.db
    fading = args.fade
    transmission_interval = args.itv

    BeaconRate = int(1000 / transmission_interval)

    SubchannelNum = int(2 * (1000 / BeaconRate))
    TransmitInterval = 1000 / BeaconRate
    maximalTime = np.floor(threshold / TransmitInterval)

    sinr_th_db = 2.76
    sinr_th = 10 ** (sinr_th_db / 10)

    TransmitPowerdBm = 23
    TransmitPower_mw = 10 ** (TransmitPowerdBm / 10)

    if BeaconRate == 10:
        RCrange = [5, 7]

    else:
        RCrange = [10, 30]

    scale_param_omega = 1

    lane_info = {
        'num_lanes': 2,  # 2 lanes per direction
        'lane_positions': {}
    }

    for v in range(NumVehicle):
        lane_info['lane_positions'][v] = random.randint(1, 2)

    VehicleLocation = observe_vehicles[0]

    # load platoon and non-platoon vehicle sets
    set_of_platoon = np.array(pd.read_csv("set_of_platoon.csv", header=None)).tolist()[0]

    set_of_non_platoon = np.array(pd.read_csv("set_of_non_platoon.csv", header=None)).tolist()[0]

    # accumulate results
    all_results = []

    for s in range(runningtime):
        platoon_index_ori = set_of_platoon
        non_platoon = set_of_non_platoon
        platoon_index = platoon_index_ori[LeadingNum:-FollowNum]

        lowerbound, higherbound = RCrange
        RClist = [random.randint(lowerbound, higherbound) for _ in range(NumVehicle)]
        ResourceSelectionini = ResourceSelectionInitial(NumVehicle, SubchannelNum, False)

        results = SimulationWithECRARS_Enhanced(
            ResourceSelectionini, DesiredDistance, RClist, NumVehicle, StartTime,
            VehicleLocation, RCrange, platoon_index, non_platoon, platoon_index_ori,
            LeadingNum, FollowNum, SubchannelNum, TransmitPower_mw, TransmitInterval,
            sinr_th, fading, scale_param_omega, maximalTime, SimulationTime, lane_info
        )

        all_results.append(results)

    avg_plr = np.mean([r['plr'] for r in all_results])
    std_plr = np.std([r['plr'] for r in all_results])

    avg_non_platoon_plr = np.mean([r['non_platoon_plr'] for r in all_results])
    avg_dop = np.mean([r['dop'] for r in all_results])
    avg_goodput = np.mean([r['goodput'] for r in all_results])
    avg_delay = np.mean([r['average_delay'] for r in all_results])

    total_hidden_avoided = sum([r['ecra_statistics']['hidden_collisions_avoided'] for r in all_results])

    total_merging_avoided = sum([r['ecra_statistics']['merging_collisions_avoided'] for r in all_results])
    total_boundary_activations = sum([r['ecra_statistics']['boundary_activations'] for r in all_results])
    total_hidden_detected = sum([r['ecra_statistics']['hidden_nodes_detected'] for r in all_results])
    avg_resource_efficiency = np.mean([r['ecra_statistics']['avg_resource_efficiency'] for r in all_results])

    # print results
    print("\nPCA-RS Simulation Results:")
    print(f"PLR: {avg_plr:.4f} ± {std_plr:.4f}")
    print(f"PDR: {1 - avg_plr:.4f}")
    print(f"DOP: {avg_dop:.9f}")
    print(f"GOODPUT: {avg_goodput:.1f} packets/simulation")
    print(f"Average delay: {avg_delay:.2f} ms")


if __name__ == "__main__":
    main()