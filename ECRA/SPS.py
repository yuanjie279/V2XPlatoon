# ecra.py - Enhanced Cooperative Resource Allocation (ECRA) baseline implementation

import numpy as np
import random
import argparse
import pandas as pd
from utils import ResourceSelectionInitial, CounterConsecutiveNumber, Delay_list, Distance, RSSI, RSSIratePercent, \
    CalculateSINR_fading
from random import choice


def is_resource_collision(i, j, ResourceSelectionall):
    """Detect if there is resource collision between vehicle i and j"""
    # Check if resources are the same or within half-duplex collision range
    if ResourceSelectionall[i] == ResourceSelectionall[j]:
        return True
    # Check half-duplex collision (adjacent resource blocks)
    if abs(ResourceSelectionall[i] - ResourceSelectionall[j]) == 1 and min(ResourceSelectionall[i],
                                                                           ResourceSelectionall[j]) % 2 == 0:
        return True
    return False


def SimulationWithECRA(ResourceSelectionini, DesiredDistance, RClist, NumVehicle, StartTime,
                       VehicleLocation, RCrange, platoon_index, non_platoon, platoon_index_ori,
                       LeadingNum, FollowNum, SubchannelNum, TransmitPower_mw, TransmitInterval,
                       sinr_th, fading, scale_param_omega, maximalTime, SimulationTime):
    """
    ECRA baseline solution simulation implementation
    """
    # Initialize variables
    lowerbound, higherbound = RCrange
    ave_rc = int(np.average(RCrange))
    RSSIEachStatistic = [[] for _ in range(NumVehicle)]
    PlatoonPacketCollision = 0

    PlatoonPacketCollision_i2j = 0
    PacketCollision = 0
    Platoonalltrans = 0
    Platoonalltrans_i2j = 0
    alltrans = 0

    # Add real resource collision counter
    true_collision_count = 0
    platoon_total_transmissions = 0
    # Resource selection initialization
    ResourceSelectionallEachRound = ResourceSelectionini.copy()
    ResourceSelectionall = ResourceSelectionini.copy()

    # State list initialization
    state_list_r2l2 = [[1 for _ in range(SimulationTime - StartTime)] for _ in range(len(platoon_index))]
    state_list_i2j = [[[1 for _ in range(SimulationTime - StartTime)] for _ in range(LeadingNum + FollowNum)]
                      for _ in range(len(platoon_index_ori))]

    # Collision statistics initialization
    pc_list_i2j = [[0 for _ in range(LeadingNum + FollowNum)] for _ in range(len(platoon_index_ori))]
    alltrans_i2j = [[0 for _ in range(LeadingNum + FollowNum)] for _ in range(len(platoon_index_ori))]

    # Main simulation loop
    for t in range(1, SimulationTime):
        VehicleLocation = observe_vehicles[t]

        # Process each vehicle
        for i in range(NumVehicle):
            # Decrease resource counter
            RClist[i] = RClist[i] - 1
            # RClist[i] = max(0, RClist[i] - 1)  # Alternative approach

            # Calculate and store RSSI
            RSSIEach = RSSI(i, ResourceSelectionall, SubchannelNum, NumVehicle, VehicleLocation, TransmitPower_mw)
            RSSIEachStatistic[i].append(RSSIEach)

            # Calculate average RSSI
            if t < ave_rc:
                sumRSSI = np.sum(RSSIEachStatistic[i], axis=0)
                AverageRSSI = [m / t for m in sumRSSI]
            else:
                sumRSSI = np.sum(RSSIEachStatistic[i][t - ave_rc + 1:], axis=0)
                AverageRSSI = [m / ave_rc for m in sumRSSI]
                # AverageRSSI = [m / min(ave_rc, t) for m in sumRSSI]  # More robust calculation

            # Resource reselection when counter reaches 0
            if RClist[i] == 0:
                RClist[i] = random.randint(lowerbound, higherbound + 1)

                # Resource selection based on RSSI
                # In actual ECRA implementation, persistence probability is used, here set to 0, always reselect resources
                p = random.random()
                if p > 0:  # Always execute
                    temp = RSSIratePercent(i, AverageRSSI, ResourceSelectionall, SubchannelNum, 0.2)
                    subchselected = choice(temp)
                    ResourceSelectionallEachRound[i] = subchselected
                    # ResourceSelectionallEachRound[i] = random.choice(temp)  # Alternative selection

        # Update all vehicles' resource selection
        ResourceSelectionall = ResourceSelectionallEachRound.copy()

        def NeighIndexSet(i, FirstIndex, PlatoonLen, num):
            neighset = list(range(i - num, i + num + 1))
            neighset_new = []
            for item in neighset:
                if FirstIndex <= item < FirstIndex + PlatoonLen and item != i:
                    neighset_new.append(item)
            return neighset_new

        def HDcollision(r1, r2):
            if abs(r1 - r2) == 1 and min(r1, r2) % 2 == 0:
                return 1
            else:
                return 0

        # Performance evaluation - only within StartTime to SimulationTime range
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
                            true_collision_count += 1

            # Evaluate platoon vehicle communications
            for i in platoon_index:
                Platoonalltrans += 1
                transmission_success = True

                for j in platoon_index_ori:
                    if i == j:
                        continue

                    if -LeadingNum <= (j - i) <= FollowNum:
                        alltrans += 1
                        sinr = CalculateSINR_fading(i, j, ResourceSelectionall, NumVehicle,
                                                    VehicleLocation, TransmitPower_mw, fading, scale_param_omega)

                        if sinr < sinr_th:
                            PlatoonPacketCollision += 1
                            # PacketCollision += 1
                            # if CalculateSINR_fading(j, i, ResourceSelectionall, NumVehicle, VehicleLocation, TransmitPower_mw,
                            #                         fading, scale_param_omega) >= sinr_th and HDcollision(
                            #         ResourceSelectionall[i], ResourceSelectionall[j]) != 1:
                            #     true_collision_count += 1

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
                                                    VehicleLocation, TransmitPower_mw, fading, scale_param_omega)
                        # alternative_sinr = CalculateSINR_fading(j, i, ResourceSelectionall, NumVehicle, VehicleLocation, TransmitPower_mw, fading, scale_param_omega)

                        if sinr < sinr_th:
                            PlatoonPacketCollision_i2j += 1
                            # PacketCollision += 1
                            idx = j - i + LeadingNum if j < i else j - i + LeadingNum - 1
                            state_list_i2j[i - platoon_index_ori[0]][idx][t - StartTime] = 0
                            pc_list_i2j[i - platoon_index_ori[0]][idx] += 1
                            alltrans_i2j[i - platoon_index_ori[0]][idx] += 1
                        else:
                            delay_in_this_period = (ResourceSelectionall[i] // 2 + 1)
                            idx = j - i + LeadingNum if j < i else j - i + LeadingNum - 1
                            state_list_i2j[i - platoon_index_ori[0]][idx][t - StartTime] = delay_in_this_period
                            alltrans_i2j[i - platoon_index_ori[0]][idx] += 1

            # Non-platoon vehicle statistics
            for i in non_platoon:
                for j in range(NumVehicle):
                    if i == j:
                        continue

                    if Distance(i, j, VehicleLocation) < DesiredDistance:
                        alltrans += 1
                        if CalculateSINR_fading(i, j, ResourceSelectionall, NumVehicle, VehicleLocation,
                                                TransmitPower_mw, fading, scale_param_omega) < sinr_th:
                            PacketCollision += 1
                            # additional_collision_check = is_resource_collision(i, j, ResourceSelectionall)

    # Calculate performance metrics
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

    # Calculate final metrics
    plr = PlatoonPacketCollision / Platoonalltrans if Platoonalltrans > 0 else 0
    # Calculate real collision rate
    collision_rate = true_collision_count / platoon_total_transmissions if platoon_total_transmissions > 0 else 0
    non_platoon_plr = PacketCollision / alltrans if alltrans > 0 else 0
    goodput = Platoonalltrans_i2j - PlatoonPacketCollision_i2j

    # Calculate average delay
    average_delay = np.mean(sum_coop_delay) if sum_coop_delay else 0

    return {
        'goodput': goodput,
        'plr': plr,
        'non_platoon_plr': non_platoon_plr,
        'dop': dop_r2l2,
        'state_list': state_list_r2l2,
        'delay_list': sum_coop_delay,
        'i2j_delay_list': accum_dd_i2j_list,
        'collision_rate': collision_rate,  # Add real collision rate
        'average_delay': average_delay
    }


def run_ecra_simulation(VehicleLocation, DesiredDistance, RCrange, LeadingNum, FollowNum,
                        SubchannelNum, TransmitPower_mw, TransmitInterval, sinr_th, fading,
                        scale_param_omega, maximalTime, SimulationTime, StartTime, runningtime):
    """
    Run multiple ECRA simulations and collect results
    """
    # Initialize result containers
    plr_list = []

    non_platoon_plr_list = []
    dop_list = []
    collision_list = []
    goodput_list = []
    delay_list = []

    # Load platoon and non-platoon vehicle sets
    set_of_platoon = np.array(pd.read_csv("set_of_platoon.csv", header=None)).tolist()[0]
    set_of_non_platoon = np.array(pd.read_csv("set_of_non_platoon.csv", header=None)).tolist()[0]
    # alternative_platoon_set = list(range(0, 8))  # Alternative platoon definition

    # Run simulation
    for s in range(runningtime):
        # Set indices
        platoon_index_ori = set_of_platoon
        non_platoon = set_of_non_platoon
        platoon_index = platoon_index_ori[LeadingNum:-FollowNum]

        # Initialize resource selection
        lowerbound, higherbound = RCrange
        RClist = [random.randint(lowerbound, higherbound) for _ in range(NumVehicle)]
        ResourceSelectionini = ResourceSelectionInitial(NumVehicle, SubchannelNum, False)
        # ResourceSelectionini = [random.randint(0, SubchannelNum-1) for _ in range(NumVehicle)]  # Alternative initialization

        # Run simulation
        results = SimulationWithECRA(
            ResourceSelectionini, DesiredDistance, RClist, NumVehicle, StartTime,
            VehicleLocation, RCrange, platoon_index, non_platoon, platoon_index_ori,
            LeadingNum, FollowNum, SubchannelNum, TransmitPower_mw, TransmitInterval,
            sinr_th, fading, scale_param_omega, maximalTime, SimulationTime
        )

        # Collect results
        plr_list.append(results['plr'])
        non_platoon_plr_list.append(results['non_platoon_plr'])
        dop_list.append(results['dop'])
        goodput_list.append(results['goodput'])
        collision_list.append(results['collision_rate'])
        delay_list.append(results['average_delay'])

    # Calculate average values only (no standard deviation)
    plr_avg = np.mean(plr_list)
    non_platoon_plr_avg = np.mean(non_platoon_plr_list)
    dop_avg = np.mean(dop_list)
    goodput_avg = np.mean(goodput_list)
    collision_avg = np.mean(collision_list)
    delay_avg = np.mean(delay_list)

    return {
        'plr_avg': plr_avg,
        'non_platoon_plr_avg': non_platoon_plr_avg,
        'dop_avg': dop_avg,
        'goodput_avg': goodput_avg,
        'collision_avg': collision_avg,
        'delay_avg': delay_avg
    }


def main():
    """
    ECRA solution main function
    """
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='ECRA baseline solution simulation')
    parser.add_argument('--lvn', type=int, default=2, help='Number of leading vehicles in IFT')
    parser.add_argument('--fn', type=int, default=2, help='Number of following vehicles in IFT')
    parser.add_argument('--r', type=int, default=60, help='Simulation run count')

    parser.add_argument('--est', type=int, default=30, help='End simulation time')
    parser.add_argument('--sst', type=int, default=10, help='Start sampling time')
    parser.add_argument('--td', type=float, default=200, help='Beacon message target distance')
    parser.add_argument('--db', type=float, default=100, help='Delay boundary')
    parser.add_argument('--fade', type=str, default='on', help='Whether to consider fading')
    parser.add_argument('--itv', type=int, default=100, help='Transmission interval')
    parser.add_argument('--inp', type=float, default=10, help='Inter-platoon vehicle distance')

    args = parser.parse_args()

    # Set global parameters
    global NumVehicle, observe_vehicles

    # Load vehicle location data
    sumo_data_time = 30
    observe_vehicles = [[] for _ in range(sumo_data_time)]
    data_all = np.array(pd.read_csv("realmap_vehicle_location.csv", header=None)).tolist()
    NumVehicle = int(len(data_all) / sumo_data_time)
    # alternative_vehicle_count = len(data_all) // sumo_data_time  # Alternative calculation

    print("NumVehicle", NumVehicle)

    for i in range(sumo_data_time):
        observe_vehicles[i] = data_all[int(i * NumVehicle):int((i + 1) * NumVehicle)]

    # Set simulation parameters
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
    # alternative_power = 10 ** ((TransmitPowerdBm - 30) / 10)  # Convert to Watts

    # Set resource counter range based on beacon rate
    if BeaconRate == 10:
        RCrange = [5, 15]
    # elif BeaconRate == 20:
    #     RCrange = [10, 30]
    # elif BeaconRate == 50:
    #     RCrange = [25, 75]
    else:
        RCrange = [5, 15]  # Default value

    scale_param_omega = 1

    # Run simulation
    VehicleLocation = observe_vehicles[0]

    results = run_ecra_simulation(
        VehicleLocation, DesiredDistance, RCrange, LeadingNum, FollowNum,
        SubchannelNum, TransmitPower_mw, TransmitInterval, sinr_th, fading,
        scale_param_omega, maximalTime, SimulationTime, StartTime, runningtime
    )

    # Print results
    # print("ECRA baseline solution simulation results:")
    # print(f"non_platoon_plr: {results['non_platoon_plr_avg']:.4f}")

    print(f"PLR: {results['plr_avg']:.4f}")
    print(f"DOP: {results['dop_avg']:.9f}")
    print(f"PDR: {1 - results['plr_avg']:.4f}")
    print(f"GOODPUT: {results['goodput_avg']:.1f}")
    print(f"Packet Collision: {results['collision_avg']:.4f}")
    print(f"Average Delay: {results['delay_avg']:.2f} ms")


if __name__ == "__main__":
    main()