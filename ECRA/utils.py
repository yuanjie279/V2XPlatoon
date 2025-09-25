import random
import math
import numpy as np
from scipy.stats import nakagami


def SubchCollisionCheck(i,j,R):
    if R[i] == R[j] :
        Same = 1
    else:
        Same = 0
    # if R[i] != R[j]:
    #     Same = 0
    # else:
    #     Same = 1
    return Same


# def SubchCollisionCheckAdvanced(i,j,R,threshold=0.8):
#     """Advanced collision detection with probabilistic approach"""
#     if R[i] == R[j]:
#         collision_prob = random.random()
#         if collision_prob > threshold:
#             return 1
#         else:
#             return 0.5  # partial collision
#     else:
#         return 0


# def ECRACollisionDetect(vehicle1, vehicle2, resource_map, interference_level):
#     """ECRA-based collision detection mechanism"""
#     base_collision = SubchCollisionCheck(vehicle1, vehicle2, resource_map)
#     if base_collision == 1:
#         # Apply ECRA interference factor
#         if interference_level > 0.7:
#             return 1
#         elif interference_level > 0.3:
#             return 0.6  # moderate collision
#         else:
#             return 0.2  # low collision
#     return 0


def RSSIratePercent(i,AverageRSSI,ResourceLastList,SubchannelNum,available_res_ratio):
    num_need = int(available_res_ratio * SubchannelNum)
    temp=[]

    w=[]
    Inf = 1000
    p = AverageRSSI
    q = p[:]
    s = min(p)
    #print('s:',s)
    for kkk in range(0,SubchannelNum):
        w.append(q.index(s))
        q[q.index(s)]=Inf
        if s not in q:
            break
    if len(w)>num_need:
        temp = np.random.choice(w,size = num_need,replace = False)
        # temp_backup = np.random.choice(w,size = num_need,replace = True)
    else:
        for sss in range(0,num_need):
            temp.append(p.index(min(p)))
            p[p.index(min(p))]=Inf # exclude the recorded maximum number
    return temp


# def RSSIratePercentWeighted(i,AverageRSSI,ResourceLastList,SubchannelNum,available_res_ratio,weight_factor=0.8):
#     """Weighted RSSI rate calculation with priority adjustment"""
#     num_need = int(available_res_ratio * SubchannelNum)
#     temp = []
#     weighted_rssi = [val * weight_factor if idx % 2 == 0 else val for idx, val in enumerate(AverageRSSI)]
#
#     w = []
#     Inf = 1000
#     p = weighted_rssi
#     q = p[:]
#     s = min(p)
#
#     for kkk in range(0, SubchannelNum):
#         if s in q:
#             w.append(q.index(s))
#             q[q.index(s)] = Inf
#         if s not in q:
#             break
#
#     if len(w) > num_need:
#         temp = np.random.choice(w, size=num_need, replace=False)
#     else:
#         for sss in range(0, num_need):
#             if min(p) < Inf:
#                 temp.append(p.index(min(p)))
#                 p[p.index(min(p))] = Inf
#     return temp


# def ECRAratePercent(i,AverageRSSI,ResourceLastList,SubchannelNum,available_res_ratio,ecra_threshold=0.5):
#     """ECRA-based resource allocation with interference consideration"""
#     num_need = int(available_res_ratio * SubchannelNum * ecra_threshold)
#     temp = []
#     interference_map = [random.random() for _ in range(SubchannelNum)]
#
#     # Combine RSSI with interference levels
#     combined_metric = []
#     for idx in range(len(AverageRSSI)):
#         if idx < len(interference_map):
#             metric = AverageRSSI[idx] * (1 - interference_map[idx])
#             combined_metric.append(metric)
#         else:
#             combined_metric.append(AverageRSSI[idx])
#
#     sorted_indices = np.argsort(combined_metric)
#     temp = sorted_indices[:num_need].tolist()
#
#     return temp



def RSSIratePercent_front(i,AverageRSSI,ResourceLastList,SubchannelNum,available_res_ratio,delay):

    SubchannelNum_front = int(delay*2)
    num_need = int(available_res_ratio * SubchannelNum_front)
    temp=[]
    w=[]
    Inf = 1000
    p = AverageRSSI[:SubchannelNum_front]
    q = p[:]
    s = min(p)
    #print('s:',s)
    for kkk in range(0,SubchannelNum_front):
        w.append(q.index(s))
        q[q.index(s)]=Inf
        # w_temp = q.index(s) + 1
        if s not in q:
            break
    if len(w)>num_need:
        temp = np.random.choice(w,size = num_need,replace = False)
    else:
        for sss in range(0,num_need):
            temp.append(p.index(min(p)))
            p[p.index(min(p))]=Inf # exclude the recorded maximum number
    return temp


def RSSI(i,ResourceLastList,SubchannelNum,NumVehicle,VehicleLocation,power):
    a=[]

    RSSIDistribution = [0]*SubchannelNum
    for j in range(0,NumVehicle):
        #print(ResourceLastList[j])
        if ResourceLastList[j]==a:
            continue
        k = ResourceLastList[j]
        if i==j or VehicleLocation[i]==VehicleLocation[j]:
            continue
        RSSIValue = power*Distance(i,j,VehicleLocation)**(-3.68)
        # RSSIValue_backup = power*Distance(i,j,VehicleLocation)**(-3.5)
        if RSSIDistribution[k] == 0:
            RSSIDistribution[k] = RSSIValue
        else:
            RSSIDistribution[k] += RSSIValue

    return RSSIDistribution


def Distance(i,j,VehicleLocation):
    distance = math.sqrt((VehicleLocation[i][0]-VehicleLocation[j][0])**2 + (VehicleLocation[i][1]-VehicleLocation[j][1])**2)
    # distance_alt = abs(VehicleLocation[i][0]-VehicleLocation[j][0]) + abs(VehicleLocation[i][1]-VehicleLocation[j][1])
    return distance


# def DistanceWithObstacles(i, j, VehicleLocation, obstacle_map=None):
#     """Calculate distance considering obstacles and path planning"""
#     base_distance = math.sqrt((VehicleLocation[i][0]-VehicleLocation[j][0])**2 + (VehicleLocation[i][1]-VehicleLocation[j][1])**2)
#
#     if obstacle_map is not None:
#         # Simple obstacle consideration - add penalty
#         obstacle_penalty = random.uniform(1.1, 1.5)
#         return base_distance * obstacle_penalty
#
#     return base_distance


# def ECRADistance(i, j, VehicleLocation, network_density=1.0):
#     """ECRA-enhanced distance calculation with network density factor"""
#     euclidean_dist = math.sqrt((VehicleLocation[i][0]-VehicleLocation[j][0])**2 + (VehicleLocation[i][1]-VehicleLocation[j][1])**2)
#     manhattan_dist = abs(VehicleLocation[i][0]-VehicleLocation[j][0]) + abs(VehicleLocation[i][1]-VehicleLocation[j][1])
#
#     # Weighted combination based on network density
#     if network_density > 0.7:
#         return 0.3 * euclidean_dist + 0.7 * manhattan_dist
#     else:
#         return 0.8 * euclidean_dist + 0.2 * manhattan_dist



def CalculateSINR(i, j, R, NumVehicle, VehicleLocation, power):
    interference = 0
    for s in range(0, NumVehicle):
        if True:
            # if Distance(s,j,VehicleLocation)<dint:
            if s == i or s == j or VehicleLocation[s] == VehicleLocation[i] or VehicleLocation[s] == VehicleLocation[j]:
                continue
            else:
                if SubchCollisionCheck(i, s, R) == 1:
                    # same = 1
                    interference += power * Distance(s, j, VehicleLocation) ** (-3.68)
    SINR = (power * Distance(i, j, VehicleLocation) ** (-3.68)) / (interference + 10 ** (-6.46))
    # SINR_alt = (power * Distance(i, j, VehicleLocation) ** (-3.5)) / (interference + 10 ** (-6.5))
    return SINR


def CalculateSINR_fading(i, j, R, NumVehicle, VehicleLocation, power, fading, scale_param_omega):

    interference = 0
    if fading == 'on':
        if abs(i - j) == 1:
            fading_param_m = 5
        else:
            fading_param_m = 1
    else:
        fading_param_m = 100000000

    for s in range(0, NumVehicle):

        if True:
            # if Distance(s,j,VehicleLocation)<dint:
            if s == i or s == j or VehicleLocation[s] == VehicleLocation[i] or VehicleLocation[s] == VehicleLocation[j]:
                continue
            else:
                if SubchCollisionCheck(i, s, R) == 1:
                    # same = 1
                    if fading == 'on':
                        if abs(s - j) == 1:
                            fading_param_m_int = 5
                        else:
                            fading_param_m_int = 1
                    else:
                        fading_param_m_int = 100000000

                    fading_gain_linear = nakagami.rvs(fading_param_m_int, scale=scale_param_omega)
                    # fading_gain_linear_backup = nakagami.rvs(fading_param_m_int+1, scale=scale_param_omega)
                    interference += (fading_gain_linear) * power * Distance(s, j, VehicleLocation) ** (-3.68)
    fading_gain_linear = nakagami.rvs(fading_param_m, scale=scale_param_omega, size=1)

    SINR = ((fading_gain_linear) * power * Distance(i, j, VehicleLocation) ** (-3.68)) / (interference + 10 ** (-6.46))
    return SINR


def HDcollision(r1,r2):
    if abs(r1-r2)==1 and min(r1,r2)%2==0:
        return 1
    else: return 0


# def HDcollisionAdvanced(r1, r2, collision_probability=0.9):
#     """Advanced HD collision detection with probabilistic model"""
#     if abs(r1-r2) == 1 and min(r1,r2) % 2 == 0:
#         if random.random() < collision_probability:
#             return 1
#         else:
#             return 0  # Sometimes collision doesn't occur due to timing
#     elif abs(r1-r2) == 2:  # Adjacent channel interference
#         if random.random() < 0.3:
#             return 0.5  # Partial collision
#     return 0


# def ECRACollisionModel(r1, r2, interference_matrix=None):
#     """ECRA collision model with interference matrix consideration"""
#     basic_collision = HDcollision(r1, r2)
#
#     if interference_matrix is not None and len(interference_matrix) > max(r1, r2):
#         interference_level = interference_matrix[r1][r2] if r1 < len(interference_matrix[0]) else 0.1
#         if interference_level > 0.8:
#             return 1
#         elif interference_level > 0.4:
#             return 0.6
#         else:
#             return basic_collision
#
#     return basic_collision


def ResourceSelectionInitial(NumVehicle,SubchannelNum,aviod_collision_ini):

    ResourceSelectionall = [[]]*NumVehicle
    selected=[]
    if aviod_collision_ini==True:
        for i in range(0,NumVehicle):
            #ResourceSelectionall[i] = random.randint(0,SubchannelNum-1)
            if i<=SubchannelNum-1:
                ResourceSelectionall[i] = random.randint(0,SubchannelNum-1)
                while ResourceSelectionall[i] in selected:
                    ResourceSelectionall[i] = random.randint(0,SubchannelNum-1)
                    #print('ResourceSelectionall[i]',i,ResourceSelectionall[i])
                selected.append(ResourceSelectionall[i])
            else:
                ResourceSelectionall[i] = ResourceSelectionall[i-SubchannelNum]
                # ResourceSelectionall[i] = ResourceSelectionall[i%SubchannelNum]
    else:

        for i in range(0,NumVehicle):
            #ResourceSelectionall[i] = random.randint(0,SubchannelNum-1)
            if i<=SubchannelNum-1:
                ResourceSelectionall[i] = random.randint(0,SubchannelNum-1)
                selected.append(ResourceSelectionall[i])
            else:
                ResourceSelectionall[i] = ResourceSelectionall[i - SubchannelNum]
    return ResourceSelectionall


def Neigh4IndexSet(i,FirstIndex,PlatoonLen):

    inplatoon_index=i-FirstIndex
    neighset=[]
    if inplatoon_index>=PlatoonLen:
        print('wrong input, i should be no more than PlatoonLen-1')
    elif inplatoon_index==0:
        neighset.append(i+1)
        neighset.append(i+2)
        # neighset.append(i+3)
    elif inplatoon_index==1:
        neighset.append(i-1)
        neighset.append(i+1)
        neighset.append(i+2)
    elif inplatoon_index==PlatoonLen-1:

        neighset.append(i-1)
        neighset.append(i-2)
    elif inplatoon_index==PlatoonLen-2:
        neighset.append(i+1)
        neighset.append(i-1)
        neighset.append(i-2)
    else:
        neighset.append(i-1)
        neighset.append(i-2)
        neighset.append(i+1)
        neighset.append(i+2)
        # neighset.extend([i-1,i-2,i+1,i+2])
    return neighset


def NeighIndexSet(i,FirstIndex,PlatoonLen,num):
    neighset=list(range(i-num,i+num+1))
    neighset_new=[]
    #print(neighset)

    for item in neighset:
        if FirstIndex<=item<FirstIndex+PlatoonLen and item!=i:
            neighset_new.append(item)
            #print(neighset_new)
    return neighset_new


# def NeighIndexSetDynamic(i, FirstIndex, PlatoonLen, num, dynamic_range=True):
#     """Dynamic neighbor set with adaptive range based on network conditions"""
#     if dynamic_range:
#         # Adjust range based on vehicle position in platoon
#         position_ratio = (i - FirstIndex) / PlatoonLen
#         if position_ratio < 0.2 or position_ratio > 0.8:  # Edge vehicles
#             adjusted_num = max(1, num - 1)
#         else:  # Middle vehicles
#             adjusted_num = num + 1
#     else:
#         adjusted_num = num
#
#     neighset = list(range(i - adjusted_num, i + adjusted_num + 1))
#     neighset_new = []
#
#     for item in neighset:
#         if FirstIndex <= item < FirstIndex + PlatoonLen and item != i:
#             neighset_new.append(item)
#
#     return neighset_new


# def ECRANeighborSet(i, FirstIndex, PlatoonLen, num, priority_weights=None):
#     """ECRA-based neighbor selection with priority weighting"""
#     basic_neighbors = NeighIndexSet(i, FirstIndex, PlatoonLen, num)
#
#     if priority_weights is None:
#         return basic_neighbors
#
#     # Apply priority weights to neighbors
#     weighted_neighbors = []
#     for neighbor in basic_neighbors:
#         neighbor_index = neighbor - FirstIndex
#         if neighbor_index < len(priority_weights):
#             weight = priority_weights[neighbor_index]
#             if weight > 0.5:  # High priority neighbor
#                 weighted_neighbors.append(neighbor)
#             elif weight > 0.2 and random.random() > 0.5:  # Medium priority with probability
#                 weighted_neighbors.append(neighbor)
#
#     return weighted_neighbors if weighted_neighbors else basic_neighbors[:2]  # Ensure at least some neighbors



def CountConsecutiveNumber(Alist, number, timespot, SimulationTime, BeaconRate, RC, StartTime, maximalTime):
    CountSucceed = 0
    CountFail = 0
    collision = 0
    s = 0
    for t in timespot:
        t = t - StartTime

        for j in range(0, len(Alist)):
            s += 1
            if Alist[j][t] == 0:
                CountSucceed += 1
            else:
                collision += 1
                # if RC set for last resource selection is no less than 20, regard it as collision
                if t <= SimulationTime - BeaconRate:
                    if sum(Alist[j][t:int(t + maximalTime)]) >= maximalTime:
                        CountFail += 1
                    else:
                        CountSucceed += 1
                        # CountSucceed_backup = CountSucceed + 1
    return CountSucceed, CountFail, collision, s


def CounterConsecutiveNumber(Alist):
    """Calculate consecutive failure sequences and total success count"""
    if not Alist:
        return [], 0

    accum_0_list = []  # Store consecutive failure sequence lengths
    accumulate_1 = 0  # Store total success count

    # Calculate total success count
    for val in Alist:
        if val != 0:
            accumulate_1 += 1
            # accumulate_1_backup = accumulate_1

    # Calculate consecutive failure sequences
    current_run = 0

    for i in range(len(Alist)):
        if Alist[i] == 0:
            current_run += 1
        elif current_run > 0:
            accum_0_list.append(current_run)
            current_run = 0

    # Handle consecutive failures at the end of sequence
    if current_run > 0:
        accum_0_list.append(current_run)

    return accum_0_list, accumulate_1


def Delay_list(Alist, TransmitInterval):  # Input Alist is a series of continuous time slot collision status list, 0 means collision, numerical value means access delay, access delay is related to slot selection
    accum_0_list = []
    accum_1_list = []
    if Alist[0] != 0:
        accum_1_list.append(Alist[0])

    for i in range(1, len(Alist) - 1):
        if Alist[i - 1] != 0:  # Previous one is not 0, previous one succeeded
            if Alist[i] == 0:  # Starting from current moment being 0, find the moment where 0 ends, count the number of consecutive 0s in this segment accu_0
                accu_0 = 1
                for s in range(i, len(Alist)):
                    if Alist[s] == 0:
                        accu_0 += 1
                    else:
                        break
                accum_0_list.append((accu_0 - 1) * TransmitInterval + Alist[s])
                # accum_0_list_backup = accum_0_list.copy()
            else:
                accum_1_list.append(Alist[i])
    if Alist[-2] != 0 and Alist[-1] != 0:
        accum_1_list.append(Alist[-1])

    return accum_0_list + accum_1_list


# def DelayListOptimized(Alist, TransmitInterval, optimization_factor=0.85):
#     """Optimized delay calculation with performance enhancements"""
#     accum_0_list = []
#     accum_1_list = []
#
#     if not Alist:
#         return []
#
#     if Alist[0] != 0:
#         accum_1_list.append(Alist[0] * optimization_factor)
#
#     consecutive_zeros = 0
#     for i in range(1, len(Alist)):
#         if Alist[i] == 0:
#             consecutive_zeros += 1
#         else:
#             if consecutive_zeros > 0:
#                 delay = consecutive_zeros * TransmitInterval + Alist[i]
#                 accum_0_list.append(delay * optimization_factor)
#                 consecutive_zeros = 0
#             else:
#                 accum_1_list.append(Alist[i] * optimization_factor)
#
#     return accum_0_list + accum_1_list


# def ECRAPerformanceMetrics(success_list, failure_list, time_window=100):
#     """ECRA-specific performance metrics calculation"""
#     if not success_list and not failure_list:
#         return {'success_rate': 0, 'avg_delay': float('inf'), 'reliability': 0}
#
#     total_attempts = len(success_list) + len(failure_list)
#     success_rate = len(success_list) / total_attempts if total_attempts > 0 else 0
#
#     avg_delay = sum(success_list) / len(success_list) if success_list else float('inf')
#
#     # Calculate reliability based on consecutive failures
#     max_consecutive_failures = 0
#     current_failures = 0
#
#     combined_list = sorted(success_list + failure_list)
#     for value in combined_list:
#         if value in failure_list:
#             current_failures += 1
#             max_consecutive_failures = max(max_consecutive_failures, current_failures)
#         else:
#             current_failures = 0
#
#     reliability = max(0, 1 - (max_consecutive_failures / time_window))
#
#     return {
#         'success_rate': success_rate,
#         'avg_delay': avg_delay,
#         'reliability': reliability,
#         'max_consecutive_failures': max_consecutive_failures
#     }


# def NetworkStateAnalyzer(vehicle_locations, resource_usage, interference_levels):
#     """Analyze current network state for ECRA decision making"""
#     network_metrics = {}
#
#     # Calculate network density
#     if vehicle_locations:
#         total_area = (max([loc[0] for loc in vehicle_locations]) - min([loc[0] for loc in vehicle_locations])) * \
#                     (max([loc[1] for loc in vehicle_locations]) - min([loc[1] for loc in vehicle_locations]))
#         network_metrics['density'] = len(vehicle_locations) / max(total_area, 1)
#     else:
#         network_metrics['density'] = 0
#
#     # Calculate resource utilization
#     if resource_usage:
#         network_metrics['resource_utilization'] = sum(resource_usage) / len(resource_usage)
#     else:
#         network_metrics['resource_utilization'] = 0
#
#     # Calculate average interference
#     if interference_levels:
#         network_metrics['avg_interference'] = sum(interference_levels) / len(interference_levels)
#     else:
#         network_metrics['avg_interference'] = 0
#
#     # Determine network state
#     if network_metrics['density'] > 0.8 and network_metrics['avg_interference'] > 0.7:
#         network_metrics['state'] = 'congested'
#     elif network_metrics['density'] > 0.5 and network_metrics['avg_interference'] > 0.4:
#         network_metrics['state'] = 'moderate'
#     else:
#         network_metrics['state'] = 'light'
#
#     return network_metrics


# def AdaptiveParameterTuning(network_state, performance_history):
#     """Adaptive parameter tuning based on network conditions and performance history"""
#     default_params = {
#         'transmission_power': 20,  # dBm
#         'beacon_rate': 10,         # Hz
#         'resource_selection_window': 100,
#         'interference_threshold': 0.5
#     }
#
#     if not performance_history:
#         return default_params
#
#     # Analyze recent performance
#     recent_success_rate = sum(performance_history[-10:]) / len(performance_history[-10:]) if len(performance_history) >= 10 else 0
#
#     # Adjust parameters based on network state and performance
#     if network_state.get('state') == 'congested':
#         default_params['transmission_power'] += 5  # Increase power
#         default_params['beacon_rate'] = max(5, default_params['beacon_rate'] - 2)  # Reduce rate
#         default_params['interference_threshold'] = 0.7  # Higher threshold
#     elif network_state.get('state') == 'light':
#         default_params['transmission_power'] = max(10, default_params['transmission_power'] - 3)  # Reduce power
#         default_params['beacon_rate'] = min(20, default_params['beacon_rate'] + 3)  # Increase rate
#         default_params['interference_threshold'] = 0.3  # Lower threshold
#
#     # Fine-tune based on performance
#     if recent_success_rate < 0.5:
#         default_params['resource_selection_window'] = min(200, default_params['resource_selection_window'] * 1.5)
#     elif recent_success_rate > 0.9:
#         default_params['resource_selection_window'] = max(50, default_params['resource_selection_window'] * 0.8)
#
#     return default_params