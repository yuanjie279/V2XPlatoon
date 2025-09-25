import numpy as np
import random
import argparse
import pandas as pd
from utils import ResourceSelectionInitial, CounterConsecutiveNumber, Delay_list, Distance, RSSI, RSSIratePercent, \
    CalculateSINR_fading
from random import choice



# vehicle role constants
LEADER = "L"  # leader vehicle
KEY_FOLLOWER = "KF"  # key follower vehicle
STANDARD_FOLLOWER = "SF"  # standard follower vehicle


# error type constants
ACK = "00"  # successful reception
RCE = "01"  # resource conflict error
CIE = "10"  # channel interference error
SAE = "11"  # signal attenuation error


# half-duplex collision detection function from user
def HDcollision(r1, r2):
    """
    detect half-duplex collision between two resources
    collision happens when resources are adjacent and smaller index is even
    """
    if abs(r1 - r2) == 1 and min(r1, r2) % 2 == 0:
        return 1
    else:
        return 0

def NeighIndexSet(i, FirstIndex, PlatoonLen, num):
    """
    create neighbor vehicle index set within platoon range
    """
    neighset = list(range(i - num, i + num + 1))
    neighset_new = []
    for item in neighset:
        if FirstIndex <= item < FirstIndex + PlatoonLen and item != i:
            neighset_new.append(item)
    return neighset_new


def detect_RCE_advanced(i, j, platoon_index_ori, ResourceSelectionall, NumVehicle,
                        VehicleLocation, TransmitPower_mw, sinr_th, fading, scale_param_omega):
    """
    improved RCE error detection function, based on advanced collision detection logic
    """
    # use neighbor set instead of global detection, platoon communication is more important
    FirstIndex = min(platoon_index_ori)
    PlatoonLen = len(platoon_index_ori)
    # usually consider 3-4 vehicles communication distance in platoon
    neighbor_range = 2  # this might need tweaking later
    # neighbor_range = 3  # tried this but too aggressive
    neighset = NeighIndexSet(i, FirstIndex, PlatoonLen, neighbor_range)
    # if no neighbors, fall back to basic detection
    if not neighset:
        # basic RCE detection - direct resource conflict
        for k in range(NumVehicle):
            if k != i and k != j:
                if ResourceSelectionall[i] == ResourceSelectionall[k]:
                    return True
        return False
    # improved RCE detection logic
    CollisionDetectedfromPM = 0
    Loss_from_collision = 0

    # # alternative approach - maybe too complex
    # collision_threshold = 0.5
    # weighted_collision_score = 0
    for neigh in neighset:
        # detect signal interference
        if CalculateSINR_fading(i, neigh, ResourceSelectionall, NumVehicle,
                                VehicleLocation, TransmitPower_mw, fading, scale_param_omega) < sinr_th:
            # check if it might be RCE (not other error types)
            if (CalculateSINR_fading(neigh, i, ResourceSelectionall, NumVehicle,
                                     VehicleLocation, TransmitPower_mw, fading, scale_param_omega) >= sinr_th and
                    (ResourceSelectionall[i] == ResourceSelectionall[neigh] or
                     HDcollision(ResourceSelectionall[i], ResourceSelectionall[neigh]) == 1)):
                CollisionDetectedfromPM += 1
                # weighted_collision_score += 1.0 / Distance(i, neigh, VehicleLocation)

        # SINR detection without considering fading - confirm it's resource issue not random fading
        if CalculateSINR_fading(i, j, ResourceSelectionall, NumVehicle, VehicleLocation, TransmitPower_mw, fading,
                                scale_param_omega) < sinr_th:
            if (ResourceSelectionall[i] == ResourceSelectionall[neigh] or
                    HDcollision(ResourceSelectionall[i], ResourceSelectionall[neigh]) == 1):
                Loss_from_collision += 1

    # verify collision and reduce false alarm rate
    if CollisionDetectedfromPM >= 1 and Loss_from_collision > 0:
        return True

    # finally check vehicle j for communication
    if j in neighset:
        if (ResourceSelectionall[i] == ResourceSelectionall[j] or
                HDcollision(ResourceSelectionall[i], ResourceSelectionall[j]) == 1):
            return True

    return False

def classify_vehicle_role(vehicle_idx, platoon_index_ori, leading_num, follow_num):
    """
    classify vehicle role based on position in platoon
    """
    if vehicle_idx not in platoon_index_ori:
        return None  # not in platoon

    platoon_position = platoon_index_ori.index(vehicle_idx)
    if platoon_position == 0:
        return LEADER
    elif 1 <= platoon_position <= leading_num:
        return KEY_FOLLOWER
    else:
        return STANDARD_FOLLOWER


def evaluate_resource_quality(role, rssi_norm, sinr_norm, stability_norm=0):
    """
    evaluate resource quality based on vehicle role
    """
    if role == LEADER:
        return 0.6 * (1 - rssi_norm) + 0.4 * sinr_norm
    elif role == KEY_FOLLOWER:
        return 0.5 * (1 - rssi_norm) + 0.3 * sinr_norm + 0.2 * stability_norm
    else:  # STANDARD_FOLLOWER
        return 0.4 * (1 - rssi_norm) + 0.4 * sinr_norm + 0.2 * stability_norm

def is_resource_collision(i, j, ResourceSelectionall):
    """detect resource conflict between vehicle i and j"""
    # check if resources are same or in half-duplex collision range
    if ResourceSelectionall[i] == ResourceSelectionall[j]:
        return True
    # check half-duplex collision (adjacent resource blocks)
    if abs(ResourceSelectionall[i] - ResourceSelectionall[j]) == 1 and min(ResourceSelectionall[i],
                                                                           ResourceSelectionall[j]) % 2 == 0:
        return True
    return False

def detect_error_type(i, j, ResourceSelectionall, NumVehicle, VehicleLocation,
                      TransmitPower_mw, fading, scale_param_omega, sinr_th,
                      rssi_history, sinr_history, platoon_index_ori=None):

    # 1. first check resource conflict (using improved advanced detection) - highest priority
    if platoon_index_ori and detect_RCE_advanced(i, j, platoon_index_ori, ResourceSelectionall,
                                                 NumVehicle, VehicleLocation, TransmitPower_mw,
                                                 sinr_th, fading, scale_param_omega):
        return RCE
    # if no platoon info provided, fall back to simple detection
    elif not platoon_index_ori:
        for k in range(NumVehicle):
            if k != i and k != j:
                if is_resource_collision(i, k, ResourceSelectionall):
                    return RCE

    # calculate current SINR
    sinr = CalculateSINR_fading(i, j, ResourceSelectionall, NumVehicle,
                                VehicleLocation, TransmitPower_mw, fading, scale_param_omega)
    # if SINR above threshold, no error
    if sinr >= sinr_th:
        return ACK

    # if SINR below threshold, use fuzzy logic to distinguish CIE and SAE
    # CIE has higher priority than SAE, based on membership and threshold
    mu_cie = calculate_cie_membership(i, j, rssi_history, sinr_history, VehicleLocation)
    mu_sae = calculate_sae_membership(i, j, rssi_history, sinr_history, VehicleLocation)
    # CIE needs lower membership threshold than SAE, reflecting priority
    if mu_cie > 0.4:  # CIE threshold set lower
        return CIE
    elif mu_sae > 0.6:  # SAE threshold set higher
        return SAE
    else:
        # if both not obvious, judge comprehensively based on difference and ratio
        if mu_cie >= mu_sae * 0.8:  # CIE only needs 80% of SAE to be judged as CIE
            return CIE
        else:
            return SAE

def calculate_cie_membership(i, j, rssi_history, sinr_history, VehicleLocation):
    """
    calculate channel interference error (CIE) membership
    """
    # initialize feature values
    rssi_stability = 0.5
    sinr_pattern = 0.5
    sinr_distance_correlation = 0.5

    # feature extraction when historical data is sufficient
    if len(rssi_history) >= 3 and len(sinr_history) >= 3:
        # 1. RSSI stability - CIE usually shows relatively stable RSSI
        rssi_std = np.std(rssi_history)
        rssi_mean = np.mean(np.abs(rssi_history)) + 1e-6  # prevent division by zero
        rssi_cv = rssi_std / rssi_mean  # coefficient of variation

        # RSSI stability normalization (low coefficient of variation indicates high stability)
        rssi_stability = max(0, min(1, 1 - rssi_cv / 0.2))  # 0.2 is empirical threshold

        # 2. SINR fluctuation pattern - CIE usually shows sudden SINR drop
        # calculate first-order difference of SINR
        sinr_diff = np.diff(sinr_history)

        # detect sudden drop
        has_sudden_drop = np.any(sinr_diff < -3.0) if sinr_diff.size > 0 else False
        # has_sudden_drop = np.any(sinr_diff < -2.5) if sinr_diff.size > 0 else False  # tried this threshold

        # calculate recent change magnitude
        recent_change = 0
        if len(sinr_history) >= 2:
            recent_change = sinr_history[-1] - sinr_history[-2]

        # sudden drop feature
        sinr_pattern = 0.7 if has_sudden_drop else 0.3
        # if recent drop is obvious, enhance feature value
        if recent_change < -2.0:  # 2dB is empirical threshold
            sinr_pattern = min(1.0, sinr_pattern + 0.3)

        # 3. SINR distance correlation - CIE SINR change has little relation with distance
        # current distance
        current_distance = Distance(i, j, VehicleLocation)

        # if distance change small but SINR change obvious, CIE possibility is high
        sinr_range = max(sinr_history) - min(sinr_history)
        if sinr_range > 5.0:  # 5dB is empirical threshold
            # close distance but large SINR fluctuation, more likely CIE
            if current_distance < 100:  # 100m is empirical threshold
                sinr_distance_correlation = 0.8
            else:
                sinr_distance_correlation = 0.6
        else:
            sinr_distance_correlation = 0.4

    # CIE weights - adjust to more reasonable values
    w_rssi_stability = 0.3
    w_sinr_pattern = 0.5
    w_sinr_distance = 0.2

    # calculate final membership
    mu_cie = (w_rssi_stability * rssi_stability +
              w_sinr_pattern * sinr_pattern +
              w_sinr_distance * sinr_distance_correlation)

    return mu_cie

def calculate_sae_membership(i, j, rssi_history, sinr_history, VehicleLocation):
    """
    calculate signal attenuation error (SAE) membership
    """
    # initialize feature values
    rssi_declining_trend = 0.5
    sinr_declining_trend = 0.5
    distance_correlation = 0.5
    consistency_factor = 0.5

    # feature extraction when historical data is sufficient
    if len(rssi_history) >= 3 and len(sinr_history) >= 3:
        # 1. RSSI declining trend - SAE usually shows gradual RSSI weakening
        # use linear regression to calculate RSSI trend
        x = np.arange(len(rssi_history))
        rssi_slope, _ = np.polyfit(x, rssi_history, 1)
        # # alternative polynomial fitting - maybe overkill
        # rssi_coeff = np.polyfit(x, rssi_history, 2)
        # rssi_trend_alt = rssi_coeff[0]

        # calculate RSSI declining trend feature
        # negative slope indicates declining trend, steeper slope means more obvious decline
        rssi_declining_trend = max(0, min(1, -rssi_slope / 2.0))  # normalize, 2.0 is empirical value
        # 2. SINR smooth declining trend - SAE SINR usually shows smooth decline not mutation
        # similar to RSSI, use linear regression to calculate SINR trend
        sinr_slope, _ = np.polyfit(x, sinr_history, 1)

        # smoothness: use residuals of fitted linear model to measure
        sinr_fit = sinr_slope * x + _
        residuals = sinr_history - sinr_fit
        smoothness = 1 - min(1, np.std(residuals) / 3.0)  # small std indicates smooth, 3.0 is empirical value

        # declining trend feature: combine slope and smoothness
        sinr_declining_trend = max(0, min(1, -sinr_slope / 2.0)) * smoothness

        # 3. distance correlation - SAE highly correlated with distance
        current_distance = Distance(i, j, VehicleLocation)

        # distance and SINR attenuation correlation
        # long distance and low SINR situation more likely SAE
        if current_distance > 150:  # 150m is long distance threshold
            distance_correlation = 0.9
        elif current_distance > 100:
            distance_correlation = 0.7
        elif current_distance > 50:
            distance_correlation = 0.5
        else:
            distance_correlation = 0.3

        # 4. RSSI and SINR trend consistency - SAE both usually decline simultaneously
        consistency_factor = 0.8 if (rssi_slope < 0 and sinr_slope < 0) else 0.2

        # if trend consistent and smooth, enhance consistency feature
        if (rssi_slope < 0 and sinr_slope < 0) and smoothness > 0.7:
            consistency_factor = min(1.0, consistency_factor + 0.2)

    # SAE weights - readjust
    w_rssi_trend = 0.25
    w_sinr_trend = 0.25
    w_distance = 0.3  # distance factor still important, but not overly dominant
    w_consistency = 0.2  # new consistency factor

    # calculate final membership
    mu_sae = (w_rssi_trend * rssi_declining_trend +
              w_sinr_trend * sinr_declining_trend +
              w_distance * distance_correlation +
              w_consistency * consistency_factor)

    return mu_sae

def select_resource_based_on_error(vehicle_idx, role, error_type, ResourceSelectionall, SubchannelNum,
                                   AverageRSSI, VehicleLocation, TransmitPower_mw, platoon_index_ori):
    """
    select new resource based on error type and vehicle role
    """
    # for all vehicles, select from lowest 20% RSSI resources
    available_ratio = 0.2
    # available_ratio = 0.25  # tried this but performance was worse
    candidate_resources = RSSIratePercent(vehicle_idx, AverageRSSI, ResourceSelectionall, SubchannelNum,
                                          available_ratio)
    # resource quality grading
    resources_sorted = sorted(range(SubchannelNum), key=lambda r: AverageRSSI[r])
    hq_threshold = int(SubchannelNum * 0.25)
    mq_threshold = int(SubchannelNum * 0.75)

    high_quality = resources_sorted[:hq_threshold]
    medium_quality = resources_sorted[hq_threshold:mq_threshold]
    basic_quality = resources_sorted[mq_threshold:]

    # different strategies based on role and error type
    if error_type == RCE:
        if role == LEADER:
            # leader: select from high quality resource pool, avoid resources used by key followers
            kf_resources = [ResourceSelectionall[i] for i in platoon_index_ori[1:]
                            if i < len(ResourceSelectionall) and
                            classify_vehicle_role(i, platoon_index_ori, 2, 2) == KEY_FOLLOWER]
            candidates = [r for r in high_quality if r not in kf_resources]
            if candidates:
                return min(candidates, key=lambda r: AverageRSSI[r])
            return choice(high_quality) if high_quality else choice(candidate_resources)

        elif role == KEY_FOLLOWER:
            # key follower: select resource at least 2 subchannels away from leader
            leader_resource = ResourceSelectionall[platoon_index_ori[0]] if platoon_index_ori and platoon_index_ori[
                0] < len(ResourceSelectionall) else 0
            candidates = [r for r in high_quality + medium_quality
                          if abs(r - leader_resource) >= 2]
            # candidates = [r for r in high_quality + medium_quality
            #               if abs(r - leader_resource) >= 3]  # tried 3 but too restrictive
            if candidates:
                return min(candidates, key=lambda r: AverageRSSI[r])
            return choice(high_quality + medium_quality) if (high_quality + medium_quality) else choice(
                candidate_resources)

        else:  # STANDARD_FOLLOWER
            return choice(medium_quality) if medium_quality else choice(candidate_resources)

    elif error_type == CIE:
        if role == LEADER:
            return min(high_quality, key=lambda r: AverageRSSI[r]) if high_quality else choice(candidate_resources)

        elif role == KEY_FOLLOWER:
            leader_resource = ResourceSelectionall[platoon_index_ori[0]] if platoon_index_ori and platoon_index_ori[
                0] < len(ResourceSelectionall) else 0
            leader_band = leader_resource // 2
            candidates = [r for r in high_quality + medium_quality
                          if r // 2 == leader_band and r != leader_resource]
            if candidates:
                return min(candidates, key=lambda r: AverageRSSI[r])
            return choice(high_quality + medium_quality) if (high_quality + medium_quality) else choice(
                candidate_resources)

        else:  # STANDARD_FOLLOWER
            # standard follower: select available resource with minimum interference
            return min(range(SubchannelNum), key=lambda r: AverageRSSI[r])

    elif error_type == SAE:
        # for SAE, return same resource, power adjustment handled elsewhere
        return ResourceSelectionall[vehicle_idx]

    else:  # ACK or default
        return ResourceSelectionall[vehicle_idx]


def adjust_power_for_sae(role, current_power_dbm):
    """
    adjust transmit power for signal attenuation error (SAE) based on vehicle role
    """
    if role == LEADER:
        # leader: increase 3dB, max 33dBm
        new_power_dbm = min(33, current_power_dbm + 3)
    elif role == KEY_FOLLOWER:
        # key follower: increase 2dB, max 30dBm
        new_power_dbm = min(30, current_power_dbm + 2)
        # new_power_dbm = min(31, current_power_dbm + 2.5)  # tried this but regulatory issues
    else:  # STANDARD_FOLLOWER
        # standard follower: increase 1dB, max 28dBm
        new_power_dbm = min(28, current_power_dbm + 1)

    return new_power_dbm

def calculate_ccri(cbr_norm, sinr_norm, role):
    if role == LEADER:
        alpha, beta = 0.6, 0.4
    elif role == KEY_FOLLOWER:
        alpha, beta = 0.5, 0.5
    else:  # STANDARD_FOLLOWER
        alpha, beta = 0.4, 0.6

    return alpha * (1 - cbr_norm) + beta * sinr_norm
def adjust_waiting_window(role, current_window, ccri, ccri_prev):
    delta_ccri = ccri - ccri_prev
    sign_delta = 1 if delta_ccri > 0 else (-1 if delta_ccri < 0 else 0)

    if role == LEADER:
        w_min, w_max = 1, 4
        # w_min, w_max = 1, 5  # tried larger max but latency increased
        if ccri < 0.35:
            return max(w_min, current_window - 2)
        elif ccri >= 0.65:
            # simplified dynamic max calculation
            return min(w_max, current_window + 1)
        else:
            return current_window + sign_delta

    elif role == KEY_FOLLOWER:
        w_min, w_max = 2, 5
        if ccri < 0.32:
            return max(w_min, current_window - 2)
        elif ccri >= 0.68:
            return min(w_max, current_window + 1)
        else:
            return current_window + sign_delta

    else:  # STANDARD_FOLLOWER
        w_min, w_max = 2, 6
        if ccri < 0.3:
            return max(w_min, current_window - 2)
        elif ccri >= 0.7:
            return min(w_max, current_window + 1)
        else:
            return current_window + sign_delta

def initialize_rc_and_window(role):
    """
    initialize resource counter (RC) and waiting window (W) based on vehicle role
    """
    if role == LEADER:
        rc_init = random.randint(5, 15)
        w_init = 2
    elif role == KEY_FOLLOWER:
        rc_init = random.randint(10, 30)
        # rc_init = random.randint(8, 25)  # tried this range
        w_init = 3
    else:  # STANDARD_FOLLOWER
        rc_init = random.randint(15, 45)
        w_init = 4

    return rc_init, w_init

def SimulationWithECRA(ResourceSelectionini, DesiredDistance, RClist, NumVehicle, StartTime,
                       VehicleLocation, RCrange, platoon_index, non_platoon, platoon_index_ori,
                       LeadingNum, FollowNum, SubchannelNum, TransmitPower_mw, TransmitInterval,
                       sinr_th, fading, scale_param_omega, maximalTime, SimulationTime):
    """
    ECRA protocol simulation implementation
    """
    lowerbound, higherbound = RCrange
    ave_rc = int(np.average(RCrange))
    RSSIEachStatistic = [[] for _ in range(NumVehicle)]
    PlatoonPacketCollision = 0
    PlatoonPacketCollision_i2j = 0
    PacketCollision = 0
    Platoonalltrans = 0
    Platoonalltrans_i2j = 0
    alltrans = 0
    platoon_resource_collision_count = 0
    platoon_total_transmissions = 0
    ResourceSelectionallEachRound = ResourceSelectionini.copy()
    ResourceSelectionall = ResourceSelectionini.copy()
    TransmitPower_dbm = 23  # default 23 dBm
    TransmitPower_per_vehicle_dbm = [TransmitPower_dbm] * NumVehicle
    TransmitPower_per_vehicle_mw = [TransmitPower_mw] * NumVehicle
    vehicle_roles = [classify_vehicle_role(i, platoon_index_ori, LeadingNum, FollowNum) for i in range(NumVehicle)]
    waiting_windows = [0] * NumVehicle
    sinr_history = [[] for _ in range(NumVehicle)]
    rssi_history = [[] for _ in range(NumVehicle)]
    ccri_prev = [0.5] * NumVehicle  # initial CCRI values

    for i in range(NumVehicle):
        if i in platoon_index_ori:
            role = vehicle_roles[i]
            if role:  # ensure role is defined
                RClist[i], waiting_windows[i] = initialize_rc_and_window(role)

    # initialize state lists (for performance tracking)
    state_list_r2l2 = [[1 for _ in range(SimulationTime - StartTime)] for _ in range(len(platoon_index))]
    state_list_i2j = [[[1 for _ in range(SimulationTime - StartTime)] for _ in range(LeadingNum + FollowNum)]
                      for _ in range(len(platoon_index_ori))]

    # initialize collision statistics
    pc_list_i2j = [[0 for _ in range(LeadingNum + FollowNum)] for _ in range(len(platoon_index_ori))]
    alltrans_i2j = [[0 for _ in range(LeadingNum + FollowNum)] for _ in range(len(platoon_index_ori))]

    # main simulation loop
    for t in range(1, SimulationTime):
        VehicleLocation = observe_vehicles[t]
        for i in range(NumVehicle):
            RClist[i] = RClist[i] - 1
            RSSIEach = RSSI(i, ResourceSelectionall, SubchannelNum, NumVehicle, VehicleLocation,
                            TransmitPower_per_vehicle_mw[i])
            RSSIEachStatistic[i].append(RSSIEach)
            if t < ave_rc:
                sumRSSI = np.sum(RSSIEachStatistic[i], axis=0)
                AverageRSSI = [m / t for m in sumRSSI]
            else:
                sumRSSI = np.sum(RSSIEachStatistic[i][t - ave_rc + 1:], axis=0)
                AverageRSSI = [m / ave_rc for m in sumRSSI]
            if i in platoon_index_ori:
                role = vehicle_roles[i]
                if not role:  # if role not defined, skip
                    continue
                if RClist[i] == 0:
                    RClist[i] = random.randint(lowerbound, higherbound + 1)
                    p = random.random()
                    if p > 0:  # always execute
                        temp = RSSIratePercent(i, AverageRSSI, ResourceSelectionall, SubchannelNum, 0.2)
                        subchselected = choice(temp)
                        ResourceSelectionallEachRound[i] = subchselected
                elif waiting_windows[i] == 0:
                    receivers = [j for j in platoon_index_ori if -LeadingNum <= (j - i) <= FollowNum and j != i]
                    error_detected = False
                    error_type = ACK
                    # print(f"Debug: vehicle {i}, receivers: {receivers}")  # temp debug

                    for j in receivers:

                        sinr = CalculateSINR_fading(i, j, ResourceSelectionall, NumVehicle,
                                                    VehicleLocation, TransmitPower_per_vehicle_mw[i], fading,
                                                    scale_param_omega)
                        sinr_db = 10 * np.log10(sinr) if sinr > 0 else -100
                        sinr_history[i].append(sinr_db)
                        sinr_norm = min(1.0, sinr / 30)  # normalize, assume 30 as max SINR
                        rssi = TransmitPower_per_vehicle_mw[i] * Distance(i, j, VehicleLocation) ** (-3.68)
                        rssi_dbm = 10 * np.log10(rssi) if rssi > 0 else -100
                        rssi_history[i].append(rssi_dbm)
                        rssi_norm = min(1.0, rssi / 1e6)  # normalize
                        # check for errors
                        if sinr < sinr_th:
                            current_error = detect_error_type(i, j, ResourceSelectionall, NumVehicle,
                                                              VehicleLocation, TransmitPower_per_vehicle_mw[i],
                                                              fading, scale_param_omega, sinr_th,
                                                              rssi_history[i][-10:] if len(rssi_history[i]) > 10 else
                                                              rssi_history[i],
                                                              sinr_history[i][-10:] if len(sinr_history[i]) > 10 else
                                                              sinr_history[i],
                                                              platoon_index_ori)

                            error_detected = True
                            error_type = current_error
                            break
                    if error_detected:
                        if error_type == RCE or error_type == CIE:
                            new_resource = select_resource_based_on_error(
                                i, role, error_type, ResourceSelectionall, SubchannelNum,
                                AverageRSSI, VehicleLocation, TransmitPower_per_vehicle_mw[i], platoon_index_ori
                            )
                            ResourceSelectionallEachRound[i] = new_resource
                        elif error_type == SAE:
                            TransmitPower_per_vehicle_dbm[i] = adjust_power_for_sae(role,
                                                                                    TransmitPower_per_vehicle_dbm[i])
                            TransmitPower_per_vehicle_mw[i] = 10 ** (TransmitPower_per_vehicle_dbm[i] / 10)
                    # calculate CBR and CCRI for window adjustment
                    rssi_threshold = -85  # dBm, typical threshold
                    # rssi_threshold = -88  # tried this but too sensitive
                    resources_above_threshold = sum(
                        1 for r in AverageRSSI if r > 0 and 10 * np.log10(r) >= rssi_threshold)
                    cbr = resources_above_threshold / SubchannelNum
                    cbr_norm = min(1.0, cbr)
                    avg_sinr = np.mean(sinr_history[i][-10:]) if len(sinr_history[i]) > 10 else np.mean(
                        sinr_history[i]) if sinr_history[i] else 0
                    avg_sinr_norm = min(1.0, avg_sinr / 30)  # normalize
                    ccri = calculate_ccri(cbr_norm, avg_sinr_norm, role)
                    waiting_windows[i] = adjust_waiting_window(role, waiting_windows[i], ccri, ccri_prev[i])
                    ccri_prev[i] = ccri
                waiting_windows[i] = max(0, waiting_windows[i] - 1)

            else:
                if RClist[i] == 0:
                    RClist[i] = random.randint(lowerbound, higherbound + 1)

                    p = random.random()
                    if p > 0:  # always execute
                        temp = RSSIratePercent(i, AverageRSSI, ResourceSelectionall, SubchannelNum, 0.2)
                        subchselected = choice(temp)
                        ResourceSelectionallEachRound[i] = subchselected

        # update resource selection for all vehicles
        ResourceSelectionall = ResourceSelectionallEachRound.copy()
        if t >= StartTime:
            FirstIndex = platoon_index[0]
            for i in platoon_index:
                platoon_total_transmissions += 1
                neighset = NeighIndexSet(i, FirstIndex, PlatoonLen=8, num=2)
                for j in neighset:
                    if CalculateSINR_fading(i, j, ResourceSelectionall, NumVehicle, VehicleLocation, TransmitPower_mw,
                                            fading,
                                            scale_param_omega) < sinr_th:
                        if CalculateSINR_fading(j, i, ResourceSelectionall, NumVehicle, VehicleLocation,
                                                TransmitPower_mw, fading,
                                                scale_param_omega) >= sinr_th and HDcollision(ResourceSelectionall[i],
                                                                                              ResourceSelectionall[
                                                                                                  j]) != 1:
                            platoon_resource_collision_count += 1
            for i in platoon_index:
                Platoonalltrans += 1
                transmission_success = True
                for j in platoon_index_ori:
                    if i == j:
                        continue
                    if -LeadingNum <= (j - i) <= FollowNum:
                        alltrans += 1
                        sinr = CalculateSINR_fading(i, j, ResourceSelectionall, NumVehicle,
                                                    VehicleLocation, TransmitPower_per_vehicle_mw[i], fading,
                                                    scale_param_omega)
                        if sinr < sinr_th:
                            PlatoonPacketCollision += 1
                            PacketCollision += 1
                            state_list_r2l2[i - platoon_index[0]][t - StartTime] = 0
                            transmission_success = False
                            break
                if transmission_success:
                    delay_in_this_period = (ResourceSelectionall[i] // 2 + 1)
                    state_list_r2l2[i - platoon_index[0]][t - StartTime] = delay_in_this_period

            # i->j transmission statistics
            for i in platoon_index_ori:
                for j in platoon_index_ori:
                    if i == j:
                        continue
                    if -LeadingNum <= (j - i) <= FollowNum:
                        Platoonalltrans_i2j += 1
                        alltrans += 1

                        sinr = CalculateSINR_fading(i, j, ResourceSelectionall, NumVehicle,
                                                    VehicleLocation, TransmitPower_per_vehicle_mw[i], fading,
                                                    scale_param_omega)
                        if sinr < sinr_th:
                            PlatoonPacketCollision_i2j += 1
                            PacketCollision += 1
                            idx = j - i + LeadingNum if j < i else j - i + LeadingNum - 1
                            state_list_i2j[i - platoon_index_ori[0]][idx][t - StartTime] = 0
                            pc_list_i2j[i - platoon_index_ori[0]][idx] += 1
                            alltrans_i2j[i - platoon_index_ori[0]][idx] += 1
                        else:
                            delay_in_this_period = (ResourceSelectionall[i] // 2 + 1)
                            idx = j - i + LeadingNum if j < i else j - i + LeadingNum - 1
                            state_list_i2j[i - platoon_index_ori[0]][idx][t - StartTime] = delay_in_this_period
                            alltrans_i2j[i - platoon_index_ori[0]][idx] += 1
            for i in non_platoon:
                for j in range(NumVehicle):
                    if i == j:
                        continue

                    if Distance(i, j, VehicleLocation) < DesiredDistance:
                        alltrans += 1

                        sinr = CalculateSINR_fading(i, j, ResourceSelectionall, NumVehicle,
                                                    VehicleLocation, TransmitPower_per_vehicle_mw[i], fading,
                                                    scale_param_omega)
                        if sinr < sinr_th:
                            PacketCollision += 1
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
    accum_dd_i2j_list = [[] for _ in range(LeadingNum + FollowNum)]
    for i in range(len(platoon_index)):
        for j in range(LeadingNum + FollowNum):
            accum_dd_i2j_list[j] += Delay_list(state_list_i2j[i][j], TransmitInterval)

    plr = PlatoonPacketCollision / Platoonalltrans if Platoonalltrans > 0 else 0
    goodput = Platoonalltrans_i2j - PlatoonPacketCollision_i2j

    # calculate new packet collision metric
    platoon_collision_rate = platoon_resource_collision_count / platoon_total_transmissions if platoon_total_transmissions > 0 else 0
    # calculate average delay
    average_delay = np.mean(sum_coop_delay) if sum_coop_delay else 0

    return {
        'goodput': goodput,
        'plr': plr,
        'dop': dop_r2l2,
        'state_list': state_list_r2l2,
        'delay_list': sum_coop_delay,
        'i2j_delay_list': accum_dd_i2j_list,
        'average_delay': average_delay,
        'platoon_packet_collision_rate': platoon_collision_rate
    }

def run_ecra_simulation(VehicleLocation, DesiredDistance, RCrange, LeadingNum, FollowNum,
                        SubchannelNum, TransmitPower_mw, TransmitInterval, sinr_th, fading,
                        scale_param_omega, maximalTime, SimulationTime, StartTime, runningtime):
    """
    run multiple ECRA simulations and collect results
    """
    # initialize result containers
    plr_list = []
    dop_list = []
    goodput_list = []
    delay_list = []
    platoon_packet_collision_list = []
    # avg_delay_list = []  # hmm maybe not needed
    # throughput_list = []  # actually let's keep it simple

    # load platoon and non-platoon vehicle sets
    set_of_platoon = np.array(pd.read_csv("set_of_platoon.csv", header=None)).tolist()[0]
    set_of_non_platoon = np.array(pd.read_csv("set_of_non_platoon.csv", header=None)).tolist()[0]

    # run simulations
    for s in range(runningtime):

        platoon_index_ori = set_of_platoon
        non_platoon = set_of_non_platoon
        platoon_index = platoon_index_ori[LeadingNum:-FollowNum]

        # initialize resource selection
        lowerbound, higherbound = RCrange
        RClist = [random.randint(lowerbound, higherbound) for _ in range(NumVehicle)]
        ResourceSelectionini = ResourceSelectionInitial(NumVehicle, SubchannelNum, False)

        # run simulation
        results = SimulationWithECRA(
            ResourceSelectionini, DesiredDistance, RClist, NumVehicle, StartTime,
            VehicleLocation, RCrange, platoon_index, non_platoon, platoon_index_ori,
            LeadingNum, FollowNum, SubchannelNum, TransmitPower_mw, TransmitInterval,
            sinr_th, fading, scale_param_omega, maximalTime, SimulationTime
        )
        plr_list.append(results['plr'])
        dop_list.append(results['dop'])
        goodput_list.append(results['goodput'])
        delay_list.append(results['average_delay'])
        platoon_packet_collision_list.append(results['platoon_packet_collision_rate'])
        # print(f"Run {s+1}: PLR={results['plr']:.4f}")  # temp debug, keep for now
    plr_avg = np.mean(plr_list)
    dop_avg = np.mean(dop_list)
    goodput_avg = np.mean(goodput_list)
    delay_avg = np.mean(delay_list)
    platoon_packet_collision_avg = np.mean(platoon_packet_collision_list)

    return {
        'plr_avg': plr_avg,
        'dop_avg': dop_avg,
        'goodput_avg': goodput_avg,
        'platoon_packet_collision_avg': platoon_packet_collision_avg,
        'delay_avg': delay_avg
    }


def main():
    """
    ECRA protocol main function
    """
    # parse command line arguments
    parser = argparse.ArgumentParser(description='ECRA protocol simulation')
    parser.add_argument('--lvn', type=int, default=2, help='number of leading vehicles in IFT')
    parser.add_argument('--fn', type=int, default=2, help='number of following vehicles in IFT')
    parser.add_argument('--r', type=int, default=60, help='number of simulation runs')  # keep multiple runs
    parser.add_argument('--est', type=int, default=30, help='end simulation time')
    parser.add_argument('--sst', type=int, default=10, help='start sampling time')
    parser.add_argument('--td', type=float, default=200, help='target distance for beacon messages')
    parser.add_argument('--db', type=float, default=100, help='delay boundary')
    parser.add_argument('--fade', type=str, default='on', help='whether to consider fading')
    parser.add_argument('--itv', type=int, default=100, help='transmission interval')
    parser.add_argument('--inp', type=float, default=10, help='inter-platoon spacing')

    args = parser.parse_args()
    global NumVehicle, observe_vehicles
    sumo_data_time = 30
    observe_vehicles = [[] for _ in range(sumo_data_time)]
    data_all = np.array(pd.read_csv("realmap_vehicle_location.csv", header=None)).tolist()
    NumVehicle = int(len(data_all) / sumo_data_time)

    print("NumVehicle", NumVehicle)  # keep this debug info

    for i in range(sumo_data_time):
        observe_vehicles[i] = data_all[int(i * NumVehicle):int((i + 1) * NumVehicle)]


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
    # TransmitPower_w = TransmitPower_mw / 1000  # convert to watts


    # set resource counter range based on beacon rate
    if BeaconRate == 10:
        RCrange = [5, 15]
    # elif BeaconRate == 20:  # commented out, looks like we tried different values
    #     RCrange = [10, 30]
    # elif BeaconRate == 50:
    #     RCrange = [25, 75]  # this was too aggressive I think
    else:
        RCrange = [5, 15]  # default value works fine

    scale_param_omega = 1  # not sure if this needs tuning
    # scale_param_omega = 0.8  # tried different scaling

    # run simulation
    VehicleLocation = observe_vehicles[0]
    results = run_ecra_simulation(
        VehicleLocation, DesiredDistance, RCrange, LeadingNum, FollowNum,
        SubchannelNum, TransmitPower_mw, TransmitInterval, sinr_th, fading,
        scale_param_omega, maximalTime, SimulationTime, StartTime, runningtime
    )


    # print results - removed error ranges and role-based stats
    # print("ECRA protocol simulation results:")
    print(f"PLR: {results['plr_avg']:.4f}")
    print(f"DOP: {results['dop_avg']:.9f}")
    print(f"PDR: {1 - results['plr_avg']:.4f}")  # packet delivery rate
    print(f"Throughput: {results['goodput_avg']:.1f}")
    print(f"Packet Collision: {results['platoon_packet_collision_avg']:.4f}")
    print(f"Average Delay: {results['delay_avg']:.2f} ms")



if __name__ == "__main__":
    main()