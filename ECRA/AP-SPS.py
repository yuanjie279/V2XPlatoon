import numpy as np
import math
import random
import argparse
import time
import pandas as pd
import matplotlib.pyplot as plt
from utils import ResourceSelectionInitial, CounterConsecutiveNumber, Delay_list, Distance, RSSI, RSSIratePercent, \
    CalculateSINR_fading
from random import choice
import os

os.makedirs("results", exist_ok=True)

os.makedirs("results/data", exist_ok=True)
os.makedirs("results/figures", exist_ok=True)


def is_resource_collision(i, j, ResourceSelectionall):
    """detect resource conflicts between vehicle i and j"""
    # check if resources are same or in half duplex conflict range
    if ResourceSelectionall[i] == ResourceSelectionall[j]:
        return True
    # check half duplex conflict (adjacent resource blocks)
    if abs(ResourceSelectionall[i] - ResourceSelectionall[j]) == 1 and min(ResourceSelectionall[i],
                                                                           ResourceSelectionall[j]) % 2 == 0:
        return True
    return False


def plot_cdf(data, xlabel, title, filename, save_data=True):
    """
    Plot CDF curve and save image/data
    """

    # sort data and calculate cumulative probability
    sorted_data = np.sort(data)

    cdf = np.arange(1, len(sorted_data) + 1) / len(sorted_data)

    # create figure
    plt.figure(figsize=(10, 6))
    plt.plot(sorted_data, cdf, marker='.', linestyle='-', linewidth=2)
    plt.xlabel(xlabel)

    plt.ylabel('Cumulative Probability')
    plt.title(title)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()

    plt.savefig(f"results/figures/{filename}.png", dpi=300)

    plt.savefig(f"results/figures/{filename}.pdf")
    plt.close()

    if save_data:
        df = pd.DataFrame({
            'value': sorted_data,
            'cdf': cdf
        })

        df.to_csv(f"results/data/{filename}.csv", index=False)

    print(f"CDF curve saved: {filename}")


def CalculateOverlapRatio(vehicle_index, ResSelectionall, VehicleNum, ResNum):
    """calculate selection window overlap ratio for given vehicle"""
    current_window = ResSelectionall[vehicle_index]
    overlap_count = 0

    total_window_count = 0

    for other_vehicle in range(VehicleNum):
        if other_vehicle == vehicle_index:
            continue

        other_window = ResSelectionall[other_vehicle]

        # check if selection windows overlap
        if abs(current_window - other_window) < ResNum / 2:  # example condition, adjust as needed
            overlap_count += 1
            # overlap_count = overlap_count + 1

        total_window_count += 1

    # calculate overlap ratio
    return overlap_count / total_window_count if total_window_count > 0 else 0


def SimulationWithSPS(ResourceSelectionini, DesiredDistance, RClist, NumVehicle, StartTime,
                      VehicleLocation, RCrange, platoon_index, non_platoon, platoon_index_ori,
                      LeadingNum, FollowNum, SubchannelNum, TransmitPower_mw, TransmitInterval,
                      sinr_th, fading, scale_param_omega, maximalTime, SimulationTime):
    """SPS baseline simulation implementation"""

    # initialize variables
    lowerbound, higherbound = RCrange
    ave_rc = int(np.average(RCrange))

    RSSIEachStatistic = [[] for _ in range(NumVehicle)]
    PlatoonPacketCollision = 0
    PlatoonPacketCollision_i2j = 0
    PacketCollision = 0
    Platoonalltrans = 0

    Platoonalltrans_i2j = 0
    alltrans = 0

    # resource selection initialization
    ResourceSelectionallEachRound = ResourceSelectionini.copy()
    ResourceSelectionall = ResourceSelectionini.copy()

    # add real resource collision count
    true_collision_count = 0

    # state list initialization
    state_list_r2l2 = [[1 for _ in range(SimulationTime - StartTime)] for _ in range(len(platoon_index))]
    state_list_i2j = [[[1 for _ in range(SimulationTime - StartTime)] for _ in range(LeadingNum + FollowNum)]
                      for _ in range(len(platoon_index_ori))]

    # collision statistics initialization
    pc_list_i2j = [[0 for _ in range(LeadingNum + FollowNum)] for _ in range(len(platoon_index_ori))]

    alltrans_i2j = [[0 for _ in range(LeadingNum + FollowNum)] for _ in range(len(platoon_index_ori))]
    RClist_rechosen = ResourceSelectionini

    # main simulation loop
    for t in range(1, SimulationTime):
        VehicleLocation = observe_vehicles[t]

        for i in range(NumVehicle):
            # decrease resource counter
            RClist[i] = RClist[i] - 1

            # calculate and store RSSI
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

                ratio = CalculateOverlapRatio(i, ResourceSelectionall, NumVehicle, SubchannelNum)

                # dynamically adjust persistence probability based on overlap ratio
                if ratio >= 0.6:
                    ProbabilityofPersistance = 0
                elif ratio >= 0.4:
                    ProbabilityofPersistance = 0.2

                elif ratio >= 0.2:
                    ProbabilityofPersistance = 0.4
                elif ratio > 0:
                    ProbabilityofPersistance = 0.6
                else:
                    ProbabilityofPersistance = 0.8

                if i in platoon_index:
                    RClist_rechosen[i] = RClist[i]
                    # RC_rechosen[i] = RClist[i]
                p = random.random()

                if p > ProbabilityofPersistance:
                    temp = RSSIratePercent(i, AverageRSSI, ResourceSelectionall, SubchannelNum, 0.2)
                    subchselected = choice(temp)
                    ResourceSelectionallEachRound[i] = subchselected

        ResourceSelectionall = ResourceSelectionallEachRound.copy()
        if t >= StartTime:
            for i in platoon_index:
                Platoonalltrans += 1
                transmission_success = True
                for j in platoon_index_ori:
                    if i == j:
                        continue

                    if -LeadingNum <= (j - i) <= FollowNum:
                        sinr = CalculateSINR_fading(i, j, ResourceSelectionall, NumVehicle,
                                                    VehicleLocation, TransmitPower_mw, fading, scale_param_omega)

                        if sinr < sinr_th:
                            PlatoonPacketCollision += 1
                            # check if real resource collision
                            if is_resource_collision(i, j, ResourceSelectionall):
                                true_collision_count += 1

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
                        sinr = CalculateSINR_fading(i, j, ResourceSelectionall, NumVehicle,
                                                    VehicleLocation, TransmitPower_mw, fading, scale_param_omega)

                        if sinr < sinr_th:
                            PlatoonPacketCollision_i2j += 1

                            idx = j - i + LeadingNum if j < i else j - i + LeadingNum - 1
                            state_list_i2j[i - platoon_index_ori[0]][idx][t - StartTime] = 0
                            pc_list_i2j[i - platoon_index_ori[0]][idx] += 1
                            alltrans_i2j[i - platoon_index_ori[0]][idx] += 1

                        else:
                            delay_in_this_period = (ResourceSelectionall[i] // 2 + 1)
                            idx = j - i + LeadingNum if j < i else j - i + LeadingNum - 1
                            state_list_i2j[i - platoon_index_ori[0]][idx][t - StartTime] = delay_in_this_period
                            alltrans_i2j[i - platoon_index_ori[0]][idx] += 1

            # non-platoon vehicle statistics
            for i in non_platoon:
                for j in range(NumVehicle):
                    if i == j:
                        continue

                    if Distance(i, j, VehicleLocation) < DesiredDistance:

                        alltrans += 1
                        sinr = CalculateSINR_fading(i, j, ResourceSelectionall, NumVehicle,
                                                    VehicleLocation, TransmitPower_mw, fading, scale_param_omega)

                        if sinr < sinr_th:
                            PacketCollision += 1

    # calculate performance metrics
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

    # calculate final metrics
    plr = PlatoonPacketCollision / Platoonalltrans if Platoonalltrans > 0 else 0
    non_platoon_plr = PacketCollision / alltrans if alltrans > 0 else 0
    goodput = Platoonalltrans_i2j - PlatoonPacketCollision_i2j
    collision_rate = true_collision_count / Platoonalltrans if Platoonalltrans > 0 else 0
    average_delay = np.mean(sum_coop_delay) if sum_coop_delay else 0

    # collect PLR for each vehicle pair (i->j PLR) for CDF curve
    link_plr_list = []

    for i in range(len(platoon_index_ori)):
        for j in range(LeadingNum + FollowNum):
            if alltrans_i2j[i][j] > 0:
                link_plr = pc_list_i2j[i][j] / alltrans_i2j[i][j]
                link_plr_list.append(link_plr)
    return {
        'goodput': goodput,
        'plr': plr,
        'non_platoon_plr': non_platoon_plr,
        'dop': dop_r2l2,
        'state_list': state_list_r2l2,

        'delay_list': sum_coop_delay,
        'i2j_delay_list': accum_dd_i2j_list,
        'average_delay': average_delay,
        'collision_rate': collision_rate,  # add real collision rate
        'link_plr_list': link_plr_list  # PLR for each link
    }


def run_sps_simulation(VehicleLocation, DesiredDistance, RCrange, LeadingNum, FollowNum,
                       SubchannelNum, TransmitPower_mw, TransmitInterval, sinr_th, fading,
                       scale_param_omega, maximalTime, SimulationTime, StartTime, runningtime):
    plr_list = []
    non_platoon_plr_list = []
    dop_list = []

    goodput_list = []
    delay_list = []
    collision_list = []
    all_delays = []  # collect all delay values
    all_link_plrs = []  # collect all link PLRs

    # load platoon and non-platoon vehicle sets
    set_of_platoon = np.array(pd.read_csv("set_of_platoon.csv", header=None)).tolist()[0]

    set_of_non_platoon = np.array(pd.read_csv("set_of_non_platoon.csv", header=None)).tolist()[0]

    for s in range(runningtime):
        platoon_index_ori = set_of_platoon
        non_platoon = set_of_non_platoon
        platoon_index = platoon_index_ori[LeadingNum:-FollowNum]

        # initialize resource selection
        lowerbound, higherbound = RCrange
        RClist = [random.randint(lowerbound, higherbound) for _ in range(NumVehicle)]
        ResourceSelectionini = ResourceSelectionInitial(NumVehicle, SubchannelNum, False)

        # run simulation
        results = SimulationWithSPS(
            ResourceSelectionini, DesiredDistance, RClist, NumVehicle, StartTime,
            VehicleLocation, RCrange, platoon_index, non_platoon, platoon_index_ori,
            LeadingNum, FollowNum, SubchannelNum, TransmitPower_mw, TransmitInterval,
            sinr_th, fading, scale_param_omega, maximalTime, SimulationTime
        )

        # collect results
        plr_list.append(results['plr'])

        non_platoon_plr_list.append(results['non_platoon_plr'])
        dop_list.append(results['dop'])
        goodput_list.append(results['goodput'])
        delay_list.append(results['average_delay'])
        collision_list.append(results['collision_rate'])

        # collect all delays and link PLRs for CDF plotting
        all_delays.extend(results['delay_list'])
        all_link_plrs.extend(results['link_plr_list'])

    # plot CDF curves
    plot_cdf(all_delays, 'Delay (ms)', 'Delay Cumulative Distribution Function (CDF)', 'sps_delay_cdf')

    plot_cdf(all_link_plrs, 'Packet Loss Rate', 'PLR Cumulative Distribution Function (CDF)', 'sps_plr_cdf')

    # calculate averages and standard deviations
    plr_avg = np.mean(plr_list)
    plr_std = np.std(plr_list)
    non_platoon_plr_avg = np.mean(non_platoon_plr_list)

    non_platoon_plr_std = np.std(non_platoon_plr_list)
    dop_avg = np.mean(dop_list)
    dop_std = np.std(dop_list)
    goodput_avg = np.mean(goodput_list)
    goodput_std = np.std(goodput_list)

    collision_avg = np.mean(collision_list)
    collision_std = np.std(collision_list)
    delay_avg = np.mean(delay_list)
    delay_std = np.std(delay_list)

    return {
        'plr_avg': plr_avg,

        'plr_std': plr_std,
        'non_platoon_plr_avg': non_platoon_plr_avg,
        'non_platoon_plr_std': non_platoon_plr_std,
        'dop_avg': dop_avg,
        'dop_std': dop_std,
        'goodput_avg': goodput_avg,

        'goodput_std': goodput_std,
        'delay_avg': delay_avg,
        'delay_std': delay_std,
        'collision_avg': collision_avg,
        'collision_std': collision_std,

        'all_delays': all_delays,
        'all_link_plrs': all_link_plrs
    }


def main():
    """SPS scheme main function"""
    # parse command line arguments
    parser = argparse.ArgumentParser(description='SPS baseline scheme simulation')

    parser.add_argument('--lvn', type=int, default=2, help='leading vehicles number in IFT')
    parser.add_argument('--fn', type=int, default=2, help='following vehicles number in IFT')
    parser.add_argument('--r', type=int, default=60, help='simulation run times')
    parser.add_argument('--est', type=int, default=30, help='end simulation time')
    parser.add_argument('--sst', type=int, default=10, help='start sampling time')

    parser.add_argument('--td', type=float, default=200, help='beacon message target distance')
    parser.add_argument('--db', type=float, default=100, help='delay boundary')
    parser.add_argument('--fade', type=str, default='on', help='whether to consider fading')
    parser.add_argument('--itv', type=int, default=100, help='transmission interval')
    parser.add_argument('--inp', type=float, default=10, help='platoon vehicle spacing')

    args = parser.parse_args()

    # set global parameters
    global NumVehicle, observe_vehicles

    # load vehicle location data
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

    # set resource counter range based on beacon rate
    if BeaconRate == 10:
        RCrange = [5, 15]

    elif BeaconRate == 20:
        RCrange = [10, 30]
    elif BeaconRate == 50:
        RCrange = [25, 75]
    else:
        RCrange = [10, 30]

    scale_param_omega = 1

    VehicleLocation = observe_vehicles[0]

    print(f"Running SPS baseline scheme simulation (runs: {runningtime})...")
    results = run_sps_simulation(
        VehicleLocation, DesiredDistance, RCrange, LeadingNum, FollowNum,
        SubchannelNum, TransmitPower_mw, TransmitInterval, sinr_th, fading,
        scale_param_omega, maximalTime, SimulationTime, StartTime, runningtime
    )

    summary_df = pd.DataFrame({
        'metric': ['PLR', 'PDR', 'DOP', 'Goodput', 'Average Delay'],
        'average': [results['plr_avg'], 1 - results['plr_avg'], results['dop_avg'],
                    results['goodput_avg'], results['delay_avg']],
        'std_dev': [results['plr_std'], results['plr_std'], results['dop_std'],
                    results['goodput_std'], results['delay_std']]
    })

    summary_df.to_csv("results/data/sps_summary_metrics.csv", index=False)

    # print results
    print("\nAP-SPS Baseline Scheme Simulation Results:")
    print(f"PLR: {results['plr_avg']:.4f} ± {results['plr_std']:.4f}")

    print(f"DOP: {results['dop_avg']:.9f} ± {results['dop_std']:.9f}")
    print(f"PDR: {1 - results['plr_avg']:.4f} ± {results['plr_std']:.4f}")
    print(f"GOODPUT: {results['goodput_avg']:.1f} ± {results['goodput_std']:.1f}")
    print(f"Packet Collision: {results['collision_avg']:.4f} ± {results['collision_std']:.4f}")
    print(f"Average Delay: {results['delay_avg']:.2f} ± {results['delay_std']:.2f} ms")


if __name__ == "__main__":
    main()