import numpy as np
import math
import random
import argparse
import time
import pandas as pd
from utils import ResourceSelectionInitial, CounterConsecutiveNumber, Delay_list, Distance, RSSI, RSSIratePercent, \
    CalculateSINR_fading
from random import choice

from ErrorDetection import ErrorDetection


def SimulationWithCRR(ResourceSelectionini, DesiredDistance, RClist, NumVehicle, StartTime,
                      VehicleLocation, RCrange, platoon_index, non_platoon, platoon_index_ori,
                      LeadingNum, FollowNum, SubchannelNum, TransmitPower_mw, TransmitInterval,
                      sinr_th, fading, scale_param_omega, maximalTime, SimulationTime):
    lowerbound, higherbound = RCrange
    ave_rc = int(np.average(RCrange))

    RSSIEachStatistic = [[] for _ in range(NumVehicle)]
    PlatoonPacketCollision = 0
    PlatoonPacketCollision_i2j = 0
    PacketCollision = 0

    Platoonalltrans = 0
    Platoonalltrans_i2j = 0
    alltrans = 0

    ResourceSelectionallEachRound = ResourceSelectionini.copy()
    ResourceSelectionall = ResourceSelectionini.copy()

    true_collision_count = 0

    state_list_r2l2 = [[1 for _ in range(SimulationTime - StartTime)] for _ in range(len(platoon_index))]
    state_list_i2j = [[[1 for _ in range(SimulationTime - StartTime)] for _ in range(LeadingNum + FollowNum)]
                      for _ in range(len(platoon_index_ori))]

    pc_list_i2j = [[0 for _ in range(LeadingNum + FollowNum)] for _ in range(len(platoon_index_ori))]

    alltrans_i2j = [[0 for _ in range(LeadingNum + FollowNum)] for _ in range(len(platoon_index_ori))]

    RClist_rechosen = ResourceSelectionini.copy()
    change_num = 0
    false_num = 0

    # main simulation loop
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

            # reselect resource when counter is 0
            if RClist[i] == 0:
                RClist[i] = random.randint(lowerbound, higherbound + 1)

                if i in platoon_index:
                    RClist_rechosen[i] = RClist[i]
                    # RClist_reselect[i] = RClist[i]

                # resource reselection
                temp = RSSIratePercent(i, AverageRSSI, ResourceSelectionall, SubchannelNum, 0.2)
                subchselected = choice(temp)

                ResourceSelectionallEachRound[i] = subchselected

        # update all vehicle resource selections
        ResourceSelectionall = ResourceSelectionallEachRound.copy()

        FirstIndex = platoon_index[0]

        for i in platoon_index:
            if t >= 3 and (2 <= RClist_rechosen[i] - RClist[i] <= RCrange[0]):
                cc, false_alarm = ErrorDetection(i, FirstIndex, len(platoon_index), ResourceSelectionall,
                                                 NumVehicle, VehicleLocation, TransmitPower_mw, sinr_th,
                                                 max(LeadingNum, FollowNum), fading, scale_param_omega)

                if cc == 1:
                    change_num += 1
                    if false_alarm == 1:
                        false_num += 1

                    # recalculate RSSI and select new resource
                    if t < ave_rc:
                        sumRSSI = np.sum(RSSIEachStatistic[i], axis=0)

                        AverageRSSI = [m / t for m in sumRSSI]
                    else:
                        sumRSSI = np.sum(RSSIEachStatistic[i][t - ave_rc + 1:], axis=0)
                        AverageRSSI = [m / ave_rc for m in sumRSSI]

                    temp = RSSIratePercent(i, AverageRSSI, ResourceSelectionall, SubchannelNum, 0.2)

                    subchselected = choice(temp)
                    ResourceSelectionallEachRound[i] = subchselected

        # update resource selection
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
            # delay_list.append(accum_dd_i2j_list[j])

    # calculate final metrics
    plr = PlatoonPacketCollision / Platoonalltrans if Platoonalltrans > 0 else 0

    non_platoon_plr = PacketCollision / alltrans if alltrans > 0 else 0
    goodput = Platoonalltrans_i2j - PlatoonPacketCollision_i2j
    # calculate real collision rate
    collision_rate = true_collision_count / Platoonalltrans if Platoonalltrans > 0 else 0
    # calculate average delay
    average_delay = np.mean(sum_coop_delay) if sum_coop_delay else 0

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
        'change_num': change_num,
        'false_num': false_num
    }


def run_crr_simulation(VehicleLocation, DesiredDistance, RCrange, LeadingNum, FollowNum,
                       SubchannelNum, TransmitPower_mw, TransmitInterval, sinr_th, fading,
                       scale_param_omega, maximalTime, SimulationTime, StartTime, runningtime):
    """run multiple simulations and collect results"""

    plr_list = []
    non_platoon_plr_list = []
    dop_list = []

    goodput_list = []
    delay_list = []
    collision_list = []
    change_num_list = []
    false_num_list = []

    # load platoon and non-platoon vehicle sets
    set_of_platoon = np.array(pd.read_csv("set_of_platoon.csv", header=None)).tolist()[0]

    set_of_non_platoon = np.array(pd.read_csv("set_of_non_platoon.csv", header=None)).tolist()[0]

    for s in range(runningtime):
        print(f'Simulation round {s + 1}/{runningtime}')

        # set indices
        platoon_index_ori = set_of_platoon
        non_platoon = set_of_non_platoon
        platoon_index = platoon_index_ori[LeadingNum:-FollowNum]

        # initialize resource selection
        lowerbound, higherbound = RCrange

        RClist = [random.randint(lowerbound, higherbound) for _ in range(NumVehicle)]
        ResourceSelectionini = ResourceSelectionInitial(NumVehicle, SubchannelNum, False)

        # run simulation
        results = SimulationWithCRR(
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
        change_num_list.append(results['change_num'])
        false_num_list.append(results['false_num'])

    # calculate averages and standard deviations
    plr_avg = np.mean(plr_list)
    plr_std = np.std(plr_list)
    non_platoon_plr_avg = np.mean(non_platoon_plr_list)

    non_platoon_plr_std = np.std(non_platoon_plr_list)
    dop_avg = np.mean(dop_list)
    dop_std = np.std(dop_list)
    goodput_avg = np.mean(goodput_list)
    goodput_std = np.std(goodput_list)

    delay_avg = np.mean(delay_list)
    delay_std = np.std(delay_list)
    collision_avg = np.mean(collision_list)
    collision_std = np.std(collision_list)
    change_num_avg = np.mean(change_num_list)

    false_num_avg = np.mean(false_num_list)

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
        'change_num_avg': change_num_avg,
        'false_num_avg': false_num_avg
    }


def is_resource_collision(i, j, ResourceSelectionall):
    # detect resource collision between vehicle i and j
    if ResourceSelectionall[i] == ResourceSelectionall[j]:
        return True
    if abs(ResourceSelectionall[i] - ResourceSelectionall[j]) == 1 and min(ResourceSelectionall[i],
                                                                           ResourceSelectionall[j]) % 2 == 0:
        return True

    return False


def main():
    parser = argparse.ArgumentParser(description='CRR Protocol Simulation')

    parser.add_argument('--lvn', type=int, default=2, help='leading vehicles in IFT')
    parser.add_argument('--fn', type=int, default=2, help='following vehicles in IFT')
    parser.add_argument('--r', type=int, default=60, help='simulation runs')
    parser.add_argument('--est', type=int, default=30, help='end simulation time')

    parser.add_argument('--sst', type=int, default=10, help='start sampling time')
    parser.add_argument('--td', type=float, default=200, help='beacon message target distance')
    parser.add_argument('--db', type=float, default=100, help='delay constraint')
    parser.add_argument('--fade', type=str, default='on', help='fading enabled')
    parser.add_argument('--itv', type=int, default=100, help='transmission interval')

    parser.add_argument('--inp', type=float, default=10, help='platoon vehicle spacing')

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

    intd_platoon = args.inp

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
        # RC_range = [5, 15]
    elif BeaconRate == 20:
        RCrange = [10, 30]

    elif BeaconRate == 50:
        RCrange = [25, 75]
    else:
        RCrange = [10, 30]  # default value

    scale_param_omega = 1

    VehicleLocation = observe_vehicles[0]
    results = run_crr_simulation(
        VehicleLocation, DesiredDistance, RCrange, LeadingNum, FollowNum,
        SubchannelNum, TransmitPower_mw, TransmitInterval, sinr_th, fading,
        scale_param_omega, maximalTime, SimulationTime, StartTime, runningtime
    )

    print("\nCRR Protocol Simulation Results:")
    print(f"PLR: {results['plr_avg']:.4f} ± {results['plr_std']:.4f}")
    print(f"DOP: {results['dop_avg']:.9f} ± {results['dop_std']:.9f}")
    print(f"PDR: {1 - results['plr_avg']:.4f} ± {results['plr_std']:.4f}")
    print(f"GOODPUT: {results['goodput_avg']:.1f} ± {results['goodput_std']:.1f}")
    print(f"Packet Collision: {results['collision_avg']:.4f} ± {results['collision_std']:.4f}")
    print(f"Average Delay: {results['delay_avg']:.2f} ± {results['delay_std']:.2f} ms")


if __name__ == "__main__":
    main()