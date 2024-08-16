import numpy as np
import os
import matplotlib.pyplot as plt
from datetime import datetime
from scipy.signal import find_peaks
import json

def load_npz(file):
    data = np.load(file)
    return {key: data[key].item() if data[key].shape == () else data[key] for key in data.keys()}

def simulate(veh, tr, simname, logid):
    global timer_solver_start
    timer_solver_start = datetime.now()

    # HUD
    print('Simulation started.')
    logid.write('Simulation started.\n')

    # Maximum speed curve (assuming pure lateral condition)
    v_max = np.zeros(tr['n'], dtype=np.float32)
    bps_v_max = np.zeros(tr['n'], dtype=np.float32)
    tps_v_max = np.zeros(tr['n'], dtype=np.float32)
    for i in range(tr['n']):
        v_max[i], tps_v_max[i], bps_v_max[i] = vehicle_model_lat(veh, tr, i)

    # HUD
    print('Maximum speed calculated at all points.')
    logid.write('Maximum speed calculated at all points.\n')

    # Finding apexes
    v_apex, apex = find_peaks(-v_max)
    v_apex = -v_apex
    if tr['info']['config'] == 'Open':
        if apex[0] != 0:
            apex = np.insert(apex, 0, 0)
            v_apex = np.insert(v_apex, 0, 0)
        else:
            v_apex[0] = 0
    if len(apex) == 0:
        apex = np.argmin(v_max)
        v_apex = v_max[apex]

    apex_table = np.column_stack((v_apex, apex))
    apex_table = apex_table[apex_table[:, 0].argsort()]
    v_apex = apex_table[:, 0]
    apex = apex_table[:, 1].astype(int)
    tps_apex = tps_v_max[apex]
    bps_apex = bps_v_max[apex]

    # HUD
    print('Found all apexes on track.')
    logid.write('Found all apexes on track.\n')

    # Simulation
    N = len(apex)
    flag = np.zeros((tr['n'], 2), dtype=bool)
    v = np.full((tr['n'], N, 2), np.inf, dtype=np.float32)
    ax = np.zeros((tr['n'], N, 2), dtype=np.float32)
    ay = np.zeros((tr['n'], N, 2), dtype=np.float32)
    tps = np.zeros((tr['n'], N, 2), dtype=np.float32)
    bps = np.zeros((tr['n'], N, 2), dtype=np.float32)

    # HUD
    print('Starting acceleration and deceleration.')
    logid.write('Starting acceleration and deceleration.\n')
    prg_size = 30
    prg_pos = logid.tell()
    print(f"Running: [{' ' * prg_size}] 0 [%]")
    logid.write(f"Running: [{' ' * prg_size}] 0 [%]\n")
    logid.write('________________________________________________\n')
    logid.write('|_Apex__|_Point_|_Mode__|___x___|___v___|_vmax_|\n')

    for i in range(N):
        for k in range(2):
            if k == 0:  # acceleration
                mode = 1
                k_rest = 1
            else:  # deceleration
                mode = -1
                k_rest = 0

            if not (tr['info']['config'] == 'Open' and mode == -1 and i == 0):
                i_rest = other_points(i, N)
                if len(i_rest) == 0:
                    i_rest = i
                j = apex[i]
                v[j, i, k] = v_apex[i]
                ay[j, i, k] = v_apex[i] ** 2 * tr['r'][j]
                tps[j, :, 0] = tps_apex[i]
                bps[j, :, 0] = bps_apex[i]
                tps[j, :, 1] = tps_apex[i]
                bps[j, :, 1] = bps_apex[i]
                flag[j, k] = True

                j_next = next_point(j, tr['n'], mode, tr['info']['config'])[1]
                if not (tr['info']['config'] == 'Open' and mode == 1 and i == 0):
                    v[j_next, i, k] = v[j, i, k]
                    j_next, j = next_point(j, tr['n'], mode, tr['info']['config'])

                while True:
                    logid.write(f'{i:7d}\t{j:7d}\t{k:7d}\t{tr["x"][j]:7.1f}\t{v[j, i, k]:7.2f}\t{v_max[j]:7.2f}\n')
                    v[j_next, i, k], ax[j, i, k], ay[j, i, k], tps[j, i, k], bps[j, i, k], overshoot = vehicle_model_comb(
                        veh, tr, v[j, i, k], v_max[j_next], j, mode)
                    if overshoot:
                        break
                    if flag[j, k] or flag[j, k_rest]:
                        if max(v[j_next, i, k] >= v[j_next, i_rest, k]) or max(v[j_next, i, k] > v[j_next, i_rest, k_rest]):
                            break
                    flag = flag_update(flag, j, k, prg_size, logid, prg_pos)
                    j_next, j = next_point(j, tr['n'], mode, tr['info']['config'])
                    if tr['info']['config'] == 'Closed' and j == apex[i]:
                        break
                    if tr['info']['config'] == 'Open' and (j == tr['n'] or j == 1):
                        flag = flag_update(flag, j, k, prg_size, logid, prg_pos)
                        break

    progress_bar(np.max(flag, axis=1), prg_size, logid, prg_pos)
    print('\nVelocity profile calculated.')
    print(f'Solver time is: {(datetime.now() - timer_solver_start).total_seconds()} [s]')
    print('Post-processing initialised.')
    logid.write('________________________________________________\n')
    logid.write('Velocity profile calculated.\n')
    logid.write(f'Solver time is: {(datetime.now() - timer_solver_start).total_seconds()} [s]\n')
    logid.write('Post-processing initialised.\n')

    # Post-processing results
    V = np.zeros(tr['n'])
    AX = np.zeros(tr['n'])
    AY = np.zeros(tr['n'])
    TPS = np.zeros(tr['n'])
    BPS = np.zeros(tr['n'])
    for i in range(tr['n']):
        IDX = v.shape[1]
        V[i], idx = min((v[i, :, 0], v[i, :, 1]), key=lambda x: np.min(x))
        if idx <= IDX:
            AX[i] = ax[i, idx, 0]
            AY[i] = ay[i, idx, 0]
            TPS[i] = tps[i, idx, 0]
            BPS[i] = bps[i, idx, 0]
        else:
            AX[i] = ax[i, idx - IDX, 1]
            AY[i] = ay[i, idx - IDX, 1]
            TPS[i] = tps[i, idx - IDX, 1]
            BPS[i] = bps[i, idx - IDX, 1]

    print('Correct solution selected from modes.')
    logid.write('Correct solution selected from modes.\n')

    if tr['info']['config'] == 'Open':
        time = np.cumsum([tr['dx'][1] / V[1]] + list(tr['dx'][1:] / V[1:]))
    else:
        time = np.cumsum(tr['dx'] / V)
    sector_time = np.array([np.max(time[tr['sector'] == i]) - np.min(time[tr['sector'] == i]) for i in range(1, np.max(tr['sector']) + 1)])
    laptime = time[-1]

    print('Laptime calculated.')
    logid.write('Laptime calculated.\n')

    # Calculating forces
    M = veh['M']
    g = 9.81
    A = np.sqrt(AX ** 2 + AY ** 2)
    Fz_mass = -M * g * np.cos(np.radians(tr['bank'])) * np.cos(np.radians(tr['incl']))
    Fz_aero = 0.5 * veh['rho'] * veh['factor_Cl'] * veh['Cl'] * veh['A'] * V ** 2
    Fz_total = Fz_mass + Fz_aero
    Fx_aero = 0.5 * veh['rho'] * veh['factor_Cd'] * veh['Cd'] * veh['A'] * V ** 2
    Fx_roll = veh['Cr'] * np.abs(Fz_total)

    print('Forces calculated.')
    logid.write('Forces calculated.\n')

    # Calculating yaw motion, vehicle slip angle, and steering input
    yaw_rate = V * tr['r']
    delta = np.zeros(tr['n'])
    beta = np.zeros(tr['n'])
    for i in range(tr['n']):
        B = np.array([M * V[i] ** 2 * tr['r'][i] + M * g * np.sin(np.radians(tr['bank'][i])), 0])
        sol = np.linalg.solve(veh['C'], B)
        delta[i] = sol[0] + np.degrees(np.arctan(veh['L'] * tr['r'][i]))
        beta[i] = sol[1]
    steer = delta * veh['rack']

    print('Yaw motion calculated.')
    print('Steering angles calculated.')
    print('Vehicle slip angles calculated.')
    logid.write('Yaw motion calculated.\n')
    logid.write('Steering angles calculated.\n')
    logid.write('Vehicle slip angles calculated.\n')

    # Calculating engine metrics
    wheel_torque = TPS * np.interp(V, veh['vehicle_speed'], veh['wheel_torque'])
    Fx_eng = wheel_torque / veh['tyre_radius']
    engine_torque = TPS * np.interp(V, veh['vehicle_speed'], veh['engine_torque'])
    engine_power = TPS * np.interp(V, veh['vehicle_speed'], veh['engine_power'])
    engine_speed = np.interp(V, veh['vehicle_speed'], veh['engine_speed'])
    gear = np.interp(V, veh['vehicle_speed'], veh['gear'], left=None, right=None, period=None)
    fuel_cons = np.cumsum(wheel_torque / veh['tyre_radius'] * tr['dx'] / veh['n_primary'] / veh['n_gearbox'] / veh['n_final'] / veh['n_thermal'] / veh['fuel_LHV'])
    fuel_cons_total = fuel_cons[-1]

    print('Engine metrics calculated.')
    logid.write('Engine metrics calculated.\n')

    # Calculating KPIs
    percent_in_corners = np.sum(tr['r'] != 0) / tr['n'] * 100
    percent_in_accel = np.sum(TPS > 0) / tr['n'] * 100
    percent_in_decel = np.sum(BPS > 0) / tr['n'] * 100
    percent_in_coast = np.sum((BPS == 0) & (TPS == 0)) / tr['n'] * 100
    percent_in_full_tps = np.sum(TPS == 1) / tr['n'] * 100
    percent_in_gear = [np.sum(gear == i) / tr['n'] * 100 for i in range(veh['nog'])]
    energy_spent_fuel = fuel_cons * veh['fuel_LHV']
    energy_spent_mech = energy_spent_fuel * veh['n_thermal']
    gear_shifts = np.sum(np.abs(np.diff(gear)))
    ay_max = np.max(np.abs(AY))
    ax_max = np.max(AX)
    ax_min = np.min(AX)
    sector_v_max = [np.max(V[tr['sector'] == i]) for i in range(1, np.max(tr['sector']) + 1)]
    sector_v_min = [np.min(V[tr['sector'] == i]) for i in range(1, np.max(tr['sector']) + 1)]

    print('KPIs calculated.')
    print('Post-processing finished.')
    logid.write('KPIs calculated.\n')
    logid.write('Post-processing finished.\n')

    # Saving results in sim structure
    sim = {
        'sim_name': {'data': simname},
        'distance': {'data': tr['x'], 'unit': 'm'},
        'time': {'data': time, 'unit': 's'},
        'N': {'data': N, 'unit': None},
        'apex': {'data': apex, 'unit': None},
        'speed_max': {'data': v_max, 'unit': 'm/s'},
        'flag': {'data': flag, 'unit': None},
        'v': {'data': v, 'unit': 'm/s'},
        'Ax': {'data': ax, 'unit': 'm/s^2'},
        'Ay': {'data': ay, 'unit': 'm/s^2'},
        'tps': {'data': tps, 'unit': None},
        'bps': {'data': bps, 'unit': None},
        'elevation': {'data': tr['Z'], 'unit': 'm'},
        'speed': {'data': V, 'unit': 'm/s'},
        'yaw_rate': {'data': yaw_rate, 'unit': 'rad/s'},
        'long_acc': {'data': AX, 'unit': 'm/s^2'},
        'lat_acc': {'data': AY, 'unit': 'm/s^2'},
        'sum_acc': {'data': A, 'unit': 'm/s^2'},
        'throttle': {'data': TPS, 'unit': 'ratio'},
        'brake_pres': {'data': BPS, 'unit': 'Pa'},
        'brake_force': {'data': BPS * veh['phi'], 'unit': 'N'},
        'steering': {'data': steer, 'unit': 'deg'},
        'delta': {'data': delta, 'unit': 'deg'},
        'beta': {'data': beta, 'unit': 'deg'},
        'Fz_aero': {'data': Fz_aero, 'unit': 'N'},
        'Fx_aero': {'data': Fx_aero, 'unit': 'N'},
        'Fx_eng': {'data': Fx_eng, 'unit': 'N'},
        'Fx_roll': {'data': Fx_roll, 'unit': 'N'},
        'Fz_mass': {'data': Fz_mass, 'unit': 'N'},
        'Fz_total': {'data': Fz_total, 'unit': 'N'},
        'wheel_torque': {'data': wheel_torque, 'unit': 'N.m'},
        'engine_torque': {'data': engine_torque, 'unit': 'N.m'},
        'engine_power': {'data': engine_power, 'unit': 'W'},
        'engine_speed': {'data': engine_speed, 'unit': 'rpm'},
        'gear': {'data': gear, 'unit': None},
        'fuel_cons': {'data': fuel_cons, 'unit': 'kg'},
        'fuel_cons_total': {'data': fuel_cons_total, 'unit': 'kg'},
        'laptime': {'data': laptime, 'unit': 's'},
        'sector_time': {'data': sector_time, 'unit': 's'},
        'percent_in_corners': {'data': percent_in_corners, 'unit': '%'},
        'percent_in_accel': {'data': percent_in_accel, 'unit': '%'},
        'percent_in_decel': {'data': percent_in_decel, 'unit': '%'},
        'percent_in_coast': {'data': percent_in_coast, 'unit': '%'},
        'percent_in_full_tps': {'data': percent_in_full_tps, 'unit': '%'},
        'percent_in_gear': {'data': percent_in_gear, 'unit': '%'},
        'v_min': {'data': np.min(V), 'unit': 'm/s'},
        'v_max': {'data': np.max(V), 'unit': 'm/s'},
        'v_ave': {'data': np.mean(V), 'unit': 'm/s'},
        'energy_spent_fuel': {'data': energy_spent_fuel, 'unit': 'J'},
        'energy_spent_mech': {'data': energy_spent_mech, 'unit': 'J'},
        'gear_shifts': {'data': gear_shifts, 'unit': None},
        'lat_acc_max': {'data': ay_max, 'unit': 'm/s^2'},
        'long_acc_max': {'data': ax_max, 'unit': 'm/s^2'},
        'long_acc_min': {'data': ax_min, 'unit': 'm/s^2'},
        'sector_v_max': {'data': sector_v_max, 'unit': 'm/s'},
        'sector_v_min': {'data': sector_v_min, 'unit': 'm/s'},
    }

    print('Simulation results saved.')
    print('Simulation completed.')
    logid.write('Simulation results saved.\n')
    logid.write('Simulation completed.\n')

    return sim


def vehicle_model_lat(veh, tr, p):
    g = 9.81
    r = tr['r'][p]
    incl = tr['incl'][p]
    bank = tr['bank'][p]
    factor_grip = tr['factor_grip'][p] * veh['factor_grip']

    factor_drive = veh['factor_drive']
    factor_aero = veh['factor_aero']
    driven_wheels = veh['driven_wheels']
    M = veh['M']

    Wz = M * g * np.cos(np.radians(bank)) * np.cos(np.radians(incl))
    Wy = -M * g * np.sin(np.radians(bank))
    Wx = M * g * np.sin(np.radians(incl))

    if r == 0:
        v = veh['v_max']
        tps = 1
        bps = 0
    else:
        D = -0.5 * veh['rho'] * veh['factor_Cl'] * veh['Cl'] * veh['A']
        dmy = factor_grip * veh['sens_y']
        muy = factor_grip * veh['mu_y']
        Ny = veh['mu_y_M'] * g

        dmx = factor_grip * veh['sens_x']
        mux = factor_grip * veh['mu_x']
        Nx = veh['mu_x_M'] * g

        a = -np.sign(r) * dmy / 4 * D ** 2
        b = np.sign(r) * (muy * D + (dmy / 4) * (Ny * 4) * D - 2 * (dmy / 4) * Wz * D) - M * r
        c = np.sign(r) * (muy * Wz + (dmy / 4) * (Ny * 4) * Wz - (dmy / 4) * Wz ** 2) + Wy

        if a == 0:
            v = np.sqrt(-c / b)
        elif b ** 2 - 4 * a * c >= 0:
            if (-b + np.sqrt(b ** 2 - 4 * a * c)) / 2 / a >= 0:
                v = np.sqrt((-b + np.sqrt(b ** 2 - 4 * a * c)) / 2 / a)
            elif (-b - np.sqrt(b ** 2 - 4 * a * c)) / 2 / a >= 0:
                v = np.sqrt((-b - np.sqrt(b ** 2 - 4 * a * c)) / 2 / a)
            else:
                raise ValueError(f'No real roots at point index: {p}')
        else:
            raise ValueError(f'Discriminant <0 at point index: {p}')

        v = min(v, veh['v_max'])
        adjust_speed = True
        while adjust_speed:
            Aero_Df = 0.5 * veh['rho'] * veh['factor_Cl'] * veh['Cl'] * veh['A'] * v ** 2
            Aero_Dr = 0.5 * veh['rho'] * veh['factor_Cd'] * veh['Cd'] * veh['A'] * v ** 2
            Roll_Dr = veh['Cr'] * (-Aero_Df + Wz)
            Wd = (factor_drive * Wz + (-factor_aero * Aero_Df)) / driven_wheels
            ax_drag = (Aero_Dr + Roll_Dr + Wx) / M
            ay_max = np.sign(r) / M * (muy + dmy * (Ny - (Wz - Aero_Df) / 4)) * (Wz - Aero_Df)
            ay_needed = v ** 2 * r + g * np.sin(np.radians(bank))

            if ax_drag <= 0:
                ax_tyre_max_acc = 1 / M * (mux + dmx * (Nx - Wd)) * Wd * driven_wheels
                ax_power_limit = 1 / M * np.interp(v, veh['vehicle_speed'], veh['factor_power'] * veh['fx_engine'])
                ay = ay_max * np.sqrt(1 - (ax_drag / ax_tyre_max_acc) ** 2)
                ax_acc = ax_tyre_max_acc * np.sqrt(1 - (ay_needed / ay_max) ** 2)
                scale = min([-ax_drag, ax_acc]) / ax_power_limit
                tps = max(min(1, scale), 0)
                bps = 0
            else:
                ax_tyre_max_dec = -1 / M * (mux + dmx * (Nx - (Wz - Aero_Df) / 4)) * (Wz - Aero_Df)
                ay = ay_max * np.sqrt(1 - (ax_drag / ax_tyre_max_dec) ** 2)
                ax_dec = ax_tyre_max_dec * np.sqrt(1 - (ay_needed / ay_max) ** 2)
                fx_tyre = max([ax_drag, -ax_dec]) * M
                bps = max(fx_tyre, 0) * veh['beta']
                tps = 0

            if ay / ay_needed < 1:
                v = np.sqrt((ay - g * np.sin(np.radians(bank))) / r) - 1E-3
            else:
                adjust_speed = False

    return v, tps, bps


def vehicle_model_comb(veh, tr, v, v_max_next, j, mode):
    overshoot = False
    dx = tr['dx'][j]
    r = tr['r'][j]
    incl = tr['incl'][j]
    bank = tr['bank'][j]
    factor_grip = tr['factor_grip'][j] * veh['factor_grip']
    g = 9.81

    if mode == 1:
        factor_drive = veh['factor_drive']
        factor_aero = veh['factor_aero']
        driven_wheels = veh['driven_wheels']
    else:
        factor_drive = 1
        factor_aero = 1
        driven_wheels = 4

    M = veh['M']
    Wz = M * g * np.cos(np.radians(bank)) * np.cos(np.radians(incl))
    Wy = -M * g * np.sin(np.radians(bank))
    Wx = M * g * np.sin(np.radians(incl))

    Aero_Df = 0.5 * veh['rho'] * veh['factor_Cl'] * veh['Cl'] * veh['A'] * v ** 2
    Aero_Dr = 0.5 * veh['rho'] * veh['factor_Cd'] * veh['Cd'] * veh['A'] * v ** 2
    Roll_Dr = veh['Cr'] * (-Aero_Df + Wz)
    Wd = (factor_drive * Wz + (-factor_aero * Aero_Df)) / driven_wheels

    ax_max = mode * (v_max_next ** 2 - v ** 2) / (2 * dx)
    ax_drag = (Aero_Dr + Roll_Dr + Wx) / M
    ax_needed = ax_max - ax_drag

    ay = v ** 2 * r + g * np.sin(np.radians(bank))

    dmy = factor_grip * veh['sens_y']
    muy = factor_grip * veh['mu_y']
    Ny = veh['mu_y_M'] * g

    dmx = factor_grip * veh['sens_x']
    mux = factor_grip * veh['mu_x']
    Nx = veh['mu_x_M'] * g

    if ay != 0:
        ay_max = 1 / M * (np.sign(ay) * (muy + dmy * (Ny - (Wz - Aero_Df) / 4)) * (Wz - Aero_Df) + Wy)
        if abs(ay / ay_max) > 1:
            ellipse_multi = 0
        else:
            ellipse_multi = np.sqrt(1 - (ay / ay_max) ** 2)
    else:
        ellipse_multi = 1

    if ax_needed >= 0:
        ax_tyre_max = 1 / M * (mux + dmx * (Nx - Wd)) * Wd * driven_wheels
        ax_tyre = ax_tyre_max * ellipse_multi
        ax_power_limit = 1 / M * np.interp(v, veh['vehicle_speed'], veh['factor_power'] * veh['fx_engine'], left=None, right=None, period=None)
        scale = min([ax_tyre, ax_needed]) / ax_power_limit
        tps = max(min(1, scale), 0)
        bps = 0
        ax_com = tps * ax_power_limit
    else:
        ax_tyre_max = -1 / M * (mux + dmx * (Nx - (Wz - Aero_Df) / 4)) * (Wz - Aero_Df)
        ax_tyre = ax_tyre_max * ellipse_multi
        fx_tyre = min(-[ax_tyre, ax_needed]) * M
        bps = max(fx_tyre, 0) * veh['beta']
        tps = 0
        ax_com = -min(-[ax_tyre, ax_needed])

    ax = ax_com + ax_drag
    v_next = np.sqrt(v ** 2 + 2 * mode * ax * tr['dx'][j])

    if tps > 0 and v / veh['v_max'] >= 0.999:
        tps = 1

    if v_next / v_max_next > 1:
        overshoot = True
        v_next = np.inf
        ax = 0
        ay = 0
        tps = -1
        bps = -1
        return v_next, ax, ay, tps, bps, overshoot

    return v_next, ax, ay, tps, bps, overshoot


def next_point(j, j_max, mode, tr_config):
    if mode == 1:  # acceleration
        if tr_config == 'Closed':
            if j == j_max - 1:
                j = j_max
                j_next = 1
            elif j == j_max:
                j = 1
                j_next = j + 1
            else:
                j += 1
                j_next = j + 1
        else:
            j += 1
            j_next = j + 1
    else:  # deceleration
        if tr_config == 'Closed':
            if j == 1:
                j = j_max
                j_next = j - 1
            elif j == j_max:
                j = 1
                j_next = j - 1
            else:
                j -= 1
                j_next = j - 1
        else:
            j -= 1
            j_next = j - 1

    return j_next, j


def other_points(i, i_max):
    return np.delete(np.arange(i_max), i)


def flag_update(flag, j, k, prg_size, logid, prg_pos):
    p = np.sum(flag) / flag.size
    n_old = int(p * prg_size)

    flag[j, k] = True
    p = np.sum(flag) / flag.size
    n = int(p * prg_size)

    if n > n_old:
        progress_bar(flag, prg_size, logid, prg_pos)

    return flag


def progress_bar(flag, prg_size, logid, prg_pos):
    p = np.sum(flag) / flag.size
    n = int(p * prg_size)
    e = prg_size - n

    print(f"\rRunning: [{'|' * n}{' ' * e}] {p * 100:.0f} [%]", end='')
    logid.seek(prg_pos)
    logid.write(f"Running: [{'|' * n}{' ' * e}] {p * 100:.0f} [%]\n")
    logid.seek(0, os.SEEK_END)


def disp_logo(logid):
    logo = [
        '_______                    _____________________ ',
        '__  __ \\______________________  /___    |__  __ \\',
        '_  / / /__  __ \\  _ \\_  __ \\_  / __  /| |_  /_/ /',
        '/ /_/ /__  /_/ /  __/  / / /  /___  ___ |  ____/ ',
        '\\____/ _  .___/\\___//_/ /_//_____/_/  |_/_/      ',
        '       /_/                                       '
    ]
    print('\n'.join(logo))
    logid.write('\n'.join(logo) + '\n')


def export_report(veh, tr, sim, freq, logid):
    freq = round(freq)
    all_names = list(sim.keys())
    S = 0
    I = []

    for i, name in enumerate(all_names):
        s = sim[name]['data'].shape
        if len(s) == 1 and s[0] == tr['n']:
            S += 1
            I.append(i)

    channel_names = [all_names[i] for i in I]
    data = np.zeros((tr['n'], S), dtype=np.float32)
    channel_units = [None] * len(I)

    for i, idx in enumerate(I):
        data[:, i] = sim[all_names[idx]]['data']
        channel_units[i] = sim[all_names[idx]]['unit']

    t = np.arange(0, sim['laptime']['data'], 1 / freq)
    j = channel_names.index("time")
    time_data = np.zeros((len(t), len(I)), dtype=np.float32)

    for i in range(len(I)):
        if i == j:
            time_data[:, i] = t
        else:
            if channel_names[i] == "gear":
                time_data[:, i] = np.interp(t, data[:, j], data[:, i], left=None, right=None, period=None)
            else:
                time_data[:, i] = np.interp(t, data[:, j], data[:, i], left=None, right=None, period=None)

    print('Export initialised.')
    logid.write('Export initialised.\n')
    filename = f"{sim['sim_name']['data']}.csv"

    with open(filename, 'w') as f:
        f.write("Format,OpenLAP Export\n")
        f.write(f"Venue,{tr['info']['name']}\n")
        f.write(f"Vehicle,{veh['name']}\n")
        f.write("Driver,OpenLap\n")
        f.write("Device\n")
        f.write("Comment\n")
        f.write(f"Date,{datetime.now().strftime('%d/%m/%Y')}\n")
        f.write(f"Time,{datetime.now().strftime('%H:%M:%S')}\n")
        f.write(f"Frequency,{freq}\n\n\n\n\n")
        f.write(','.join(channel_names) + '\n')
        f.write(','.join(channel_names) + '\n')
        f.write(','.join(channel_units) + '\n\n\n')

        form = ','.join(['%f'] * len(I)) + '\n'
        for row in time_data:
            f.write(form % tuple(row))

    print('Exported .csv file successfully.')
    logid.write('Exported .csv file successfully.\n')


def main():
    trackfile = 'OpenTRACK_Tracks/OpenTRACK_Spa-Francorchamps_Closed_Forward.npz'
    vehiclefile = 'OpenVEHICLE_Vehicles/OpenVEHICLE_Formula_1_Open_Wheel.npz'

    tr = load_npz(trackfile)
    veh = load_npz(vehiclefile)

    freq = 50
    use_date_time_in_name = False

    if use_date_time_in_name:
        date_time = f"_{datetime.now().strftime('%Y_%m_%d')}_{datetime.now().strftime('%H_%M_%S')}"
    else:
        date_time = ""

    simname = f"OpenLAP Sims/OpenLAP_{veh['name']}_{tr['info']['name']}{date_time}"
    logfile = f"{simname}.log"

    os.makedirs('OpenLAP Sims', exist_ok=True)

    if os.path.exists(logfile):
        os.remove(logfile)

    with open(logfile, 'w') as logid:
        disp_logo(logid)
        print('=' * 50)
        print(f"Vehicle: {veh['name']}")
        print(f"Track:   {tr['info']['name']}")
        print(f"Date:    {datetime.now().strftime('%d/%m/%Y')}")
        print(f"Time:    {datetime.now().strftime('%H:%M:%S')}")
        print('=' * 50)

        logid.write('=' * 50 + '\n')
        logid.write(f"Vehicle: {veh['name']}\n")
        logid.write(f"Track:   {tr['info']['name']}\n")
        logid.write(f"Date:    {datetime.now().strftime('%d/%m/%Y')}\n")
        logid.write(f"Time:    {datetime.now().strftime('%H:%M:%S')}\n")
        logid.write('=' * 50 + '\n')

        sim = simulate(veh, tr, simname, logid)

        print(f'Laptime: {sim["laptime"]["data"]:.3f} [s]')
        logid.write(f'Laptime: {sim["laptime"]["data"]:.3f} [s]\n')
        for i in range(1, max(tr['sector']) + 1):
            print(f'Sector {i}: {sim["sector_time"]["data"][i-1]:.3f} [s]')
            logid.write(f'Sector {i}: {sim["sector_time"]["data"][i-1]:.3f} [s]\n')

        plt.figure(figsize=(12, 12))
        plt.suptitle(f"OpenLAP: {veh['name']} @ {tr['info']['name']} - Date & Time: {datetime.now().strftime('%Y/%m/%d %H:%M:%S')}")
        rows, cols = 7, 2
        xlimit = [tr['x'][0], tr['x'][-1]]

        plt.subplot(rows, cols, (1, 2))
        plt.plot(tr['x'], sim['speed']['data'] * 3.6)
        plt.legend(['Speed'], loc='east')
        plt.xlabel('Distance [m]')
        plt.xlim(xlimit)
        plt.ylabel('Speed [km/h]')
        plt.grid(True)

        plt.subplot(rows, cols, (3, 4))
        plt.plot(tr['x'], tr['Z'], label='Elevation')
        plt.plot(tr['x'], tr['r'], label='Curvature')
        plt.legend(['Elevation', 'Curvature'], loc='east')
        plt.xlabel('Distance [m]')
        plt.xlim(xlimit)
        plt.grid(True)

        plt.subplot(rows, cols, (5, 6))
        plt.plot(tr['x'], sim['long_acc']['data'], label='LonAcc')
        plt.plot(tr['x'], sim['lat_acc']['data'], label='LatAcc')
        plt.plot(tr['x'], sim['sum_acc']['data'], 'k:', label='GSum')
        plt.legend(loc='east')
        plt.xlabel('Distance [m]')
        plt.xlim(xlimit)
        plt.ylabel('Acceleration [m/s^2]')
        plt.grid(True)

        plt.subplot(rows, cols, (7, 8))
        plt.plot(tr['x'], sim['throttle']['data'] * 100, label='tps')
        plt.plot(tr['x'], sim['brake_pres']['data'] / 1e5, label='bps')
        plt.legend(loc='east')
        plt.xlabel('Distance [m]')
        plt.xlim(xlimit)
        plt.ylabel('input [%]')
        plt.grid(True)
        plt.ylim([-10, 110])

        plt.subplot(rows, cols, (9, 10))
        plt.plot(tr['x'], sim['steering']['data'], label='Steering wheel')
        plt.plot(tr['x'], sim['delta']['data'], label='Steering δ')
        plt.plot(tr['x'], sim['beta']['data'], label='Vehicle slip angle β')
        plt.legend(loc='east')
        plt.xlabel('Distance [m]')
        plt.xlim(xlimit)
        plt.ylabel('angle [deg]')
        plt.grid(True)

        ax = plt.subplot(rows, cols, (11, 13), projection='3d')
        ax.scatter(sim['lat_acc']['data'], sim['long_acc']['data'], sim['speed']['data'] * 3.6, c='r', label='OpenLAP')
        ax.plot_surface(veh['GGV'][:, :, 2], veh['GGV'][:, :, 0], veh['GGV'][:, :, 3] * 3.6, alpha=0.8)
        ax.set_xlabel('LatAcc [m/s^2]')
        ax.set_ylabel('LonAcc [m/s^2]')
        ax.set_zlabel('Speed [km/h]')
        plt.grid(True)
        plt.legend(loc='northeast')

        plt.subplot(rows, cols, (12, 14))
        plt.scatter(tr['X'], tr['Y'], c=sim['speed']['data'] * 3.6)
        plt.plot(tr['arrow'][:, 0], tr['arrow'][:, 1], 'k', linewidth=2)
        plt.legend(['Track Map'], loc='northeast')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.colorbar()
        plt.grid(True)
        plt.axis('equal')

        plt.savefig(f"{simname}.png")

        export_report(veh, tr, sim, freq, logid)

        with open(f"{simname}.npz", 'wb') as f:
            np.savez(f, veh=veh, tr=tr, sim=sim)

        print(f'Elapsed time is: {(datetime.now() - timer_solver_start).total_seconds()} [s]')
        logid.write(f'Elapsed time is: {(datetime.now() - timer_solver_start).total_seconds()} [s]\n')


if __name__ == "__main__":
    main()
