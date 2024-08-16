import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt
from datetime import datetime
import yaml
from mpl_toolkits.mplot3d import Axes3D  # Import for 3D plotting

def output_vehicle_dict(vehicle_filepath : str):
    with open(vehicle_filepath, 'r') as vehicle_yaml:
        vehicle_params = yaml.safe_load(vehicle_yaml)

    # Extracting variables from the YAML file
    name = vehicle_params['name']
    type_vehicle = vehicle_params['type']

    M = float(vehicle_params['total_mass'])
    df = float(vehicle_params['front_mass_distribution']) / 100
    L = float(vehicle_params['wheelbase']) / 1000
    rack = float(vehicle_params['steering_rack_ratio'])
    Cl = float(vehicle_params['lift_coefficient_cl'])
    Cd = float(vehicle_params['drag_coefficient_cd'])
    factor_Cl = float(vehicle_params['cl_scale_multiplier'])
    factor_Cd = float(vehicle_params['cd_scale_multiplier'])
    da = float(vehicle_params['front_aero_distribution']) / 100
    A = float(vehicle_params['frontal_area'])
    rho = float(vehicle_params['air_density'])
    br_disc_d = float(vehicle_params['disc_outer_diameter']) / 1000
    br_pad_h = float(vehicle_params['pad_height']) / 1000
    br_pad_mu = float(vehicle_params['pad_friction_coefficient'])
    br_nop = float(vehicle_params['caliper_number_of_pistons'])
    br_pist_d = float(vehicle_params['caliper_piston_diameter']) / 1000
    br_mast_d = float(vehicle_params['master_cylinder_piston_diameter']) / 1000
    br_ped_r = float(vehicle_params['pedal_ratio'])
    factor_grip = float(vehicle_params['grip_factor_multiplier'])
    tyre_radius = float(vehicle_params['tyre_radius']) / 1000
    Cr = float(vehicle_params['rolling_resistance'])
    mu_x = float(vehicle_params['longitudinal_friction_coefficient'])
    mu_x_M = float(vehicle_params['longitudinal_friction_load_rating'])
    sens_x = float(vehicle_params['longitudinal_friction_sensitivity'])
    mu_y = float(vehicle_params['lateral_friction_coefficient'])
    mu_y_M = float(vehicle_params['lateral_friction_load_rating'])
    sens_y = float(vehicle_params['lateral_friction_sensitivity'])
    CF = float(vehicle_params['front_cornering_stiffness'])
    CR = float(vehicle_params['rear_cornering_stiffness'])
    factor_power = float(vehicle_params['power_factor_multiplier'])
    n_thermal = float(vehicle_params['thermal_efficiency'])
    fuel_LHV = float(vehicle_params['fuel_lower_heating_value'])
    drive = vehicle_params['drive_type']
    shift_time = float(vehicle_params['gear_shift_time'])
    n_primary = float(vehicle_params['primary_gear_efficiency'])
    n_final = float(vehicle_params['final_gear_efficiency'])
    n_gearbox = float(vehicle_params['gearbox_efficiency'])
    ratio_primary = float(vehicle_params['primary_gear_reduction'])
    ratio_final = float(vehicle_params['final_gear_reduction'])
    ratio_gearbox = np.array(vehicle_params['gear_ratios'])
    nog = len(ratio_gearbox)

    engine_torque_data = vehicle_params['engine_torque']

    # Debug output
    print(f"{vehicle_filepath}")
    print('File read successfully')
    print(f"Name: {name}")
    print(f"Type: {type_vehicle}")
    print(f"Date: {datetime.now().strftime('%d/%m/%Y')}")
    print(f"Time: {datetime.now().strftime('%H:%M:%S')}")
    print('Vehicle generation started.')

    # Brake Model
    br_pist_a = br_nop * np.pi * (br_pist_d / 1000)**2 / 4
    br_mast_a = np.pi * (br_mast_d / 1000)**2 / 4
    beta = tyre_radius / (br_disc_d / 2 - br_pad_h / 2) / br_pist_a / br_pad_mu / 4
    phi = br_mast_a / br_ped_r * 2
    print("Braking model generated successfully.")

    # Steering Model
    a = (1 - df) * L
    b = -df * L
    C_matrix = 2 * np.array([[CF, CF + CR], [CF * a, CF * a + CR * b]])
    print("Steering model generated successfully.")

    # Driveline Model
    # Convert engine_torque_data to a NumPy array
    engine_torque_array = np.array(engine_torque_data)

    # Extract speeds and torques from the array
    speeds = engine_torque_array[:, 0]
    torques = engine_torque_array[:, 1]

    en_speed_curve = speeds
    en_torque_curve = torques
    en_power_curve = en_torque_curve * en_speed_curve * 2 * np.pi / 60
    
    # Memory preallocation
    wheel_speed_gear = np.zeros((len(en_speed_curve), nog))
    vehicle_speed_gear = np.zeros((len(en_speed_curve), nog))
    wheel_torque_gear = np.zeros((len(en_torque_curve), nog))

    for i in range(nog):
        wheel_speed_gear[:, i] = en_speed_curve / ratio_primary / ratio_gearbox[i] / ratio_final
        vehicle_speed_gear[:, i] = wheel_speed_gear[:, i] * 2 * np.pi / 60 * tyre_radius
        wheel_torque_gear[:, i] = en_torque_curve * ratio_primary * ratio_gearbox[i] * ratio_final * n_primary * n_gearbox * n_final

    v_min = np.min(vehicle_speed_gear)
    v_max = np.max(vehicle_speed_gear)
    dv = 0.5 / 3.6
    vehicle_speed = np.arange(v_min, v_max + dv, dv)

    gear = np.zeros(len(vehicle_speed))
    fx_engine = np.zeros(len(vehicle_speed))
    fx = np.zeros((len(vehicle_speed), nog))

    for i in range(len(vehicle_speed)):
        for j in range(nog):
            fx[i, j] = np.interp(vehicle_speed[i], vehicle_speed_gear[:, j], wheel_torque_gear[:, j] / tyre_radius, left=0, right=0)
        fx_engine[i], gear[i] = np.max(fx[i, :]), np.argmax(fx[i, :]) + 1

    vehicle_speed = np.insert(vehicle_speed, 0, 0)
    gear = np.insert(gear, 0, gear[0])
    fx_engine = np.insert(fx_engine, 0, fx_engine[0])
    engine_speed = ratio_final * ratio_gearbox[(gear - 1).astype(int)] * ratio_primary * vehicle_speed / tyre_radius * 60 / (2 * np.pi)
    wheel_torque = fx_engine * tyre_radius

    engine_torque = wheel_torque / (ratio_final * ratio_gearbox[(gear - 1).astype(int)] * ratio_primary * n_primary * n_gearbox * n_final)
    engine_power = engine_torque * engine_speed * 2 * np.pi / 60

    print("Driveline model generated successfully")

    # Shifting Points and Rev Drops
    gear_change = np.diff(gear)
    gear_change = np.insert(gear_change, 0, gear_change[0])
    gear_change = np.logical_or(gear_change, np.roll(gear_change, 1))

    engine_speed_gear_change = engine_speed[gear_change]
    shift_points = engine_speed_gear_change[0::2]
    arrive_points = engine_speed_gear_change[1::2]
    rev_drops = shift_points - arrive_points

    index_labels = [f'{i}-{i+1}' for i in range(1, len(shift_points) + 1)]

    shifting = pd.DataFrame({
        'shift_points': shift_points,
        'arrive_points': arrive_points,
        'rev_drops': rev_drops
    }, index=index_labels)
    print('Shift points calculated successfully.')

    # Force model
    g = 9.81
    if drive == 'RWD':
        factor_drive = 1 - df
        factor_aero = 1 - da
        driven_wheels = 2
    elif drive == 'FWD':
        factor_drive = df
        factor_aero = da
        driven_wheels = 2
    else:  # AWD
        factor_drive = 1
        factor_aero = 1
        driven_wheels = 4

    fz_mass = -M * g
    fz_aero = 0.5 * rho * factor_Cl * Cl * A * vehicle_speed**2
    fz_total = fz_mass + fz_aero
    fz_tyre = (factor_drive * fz_mass + factor_aero * fz_aero) / driven_wheels

    fx_aero = 0.5 * rho * factor_Cd * Cd * A * vehicle_speed**2
    fx_roll = Cr * np.abs(fz_total)
    fx_tyre = driven_wheels * (mu_x + sens_x * (mu_x_M * g - np.abs(fz_tyre))) * np.abs(fz_tyre)

    print("Forces calculated successfully.")

    # GGV Map
    bank = 0
    incl = 0
    dmy = factor_grip * sens_y
    muy = factor_grip * mu_y
    Ny = mu_y_M * g
    dmx = factor_grip * sens_x
    mux = factor_grip * mu_x
    Nx = mu_x_M * g
    Wz = M * g * np.cos(np.radians(bank)) * np.cos(np.radians(incl))
    Wy = -M * g * np.sin(np.radians(bank))
    Wx = M * g * np.sin(np.radians(incl))

    v = np.arange(0, v_max + dv, dv)
    if v[-1] != v_max:
        v = np.append(v, v_max)

    N = 45
    GGV = np.zeros((len(v), 2 * N - 1, 3))

    for i in range(len(v)):
        Aero_Df = 0.5 * rho * factor_Cl * Cl * A * v[i]**2
        Aero_Dr = 0.5 * rho * factor_Cd * Cd * A * v[i]**2
        Roll_Dr = Cr * np.abs(-Aero_Df + Wz)
        Wd = (factor_drive * Wz + (-factor_aero * Aero_Df)) / driven_wheels
        ax_drag = (Aero_Dr + Roll_Dr + Wx) / M
        ay_max = 1 / M * (muy + dmy * (Ny - (Wz - Aero_Df) / 4)) * (Wz - Aero_Df)
        ax_tyre_max_acc = 1 / M * (mux + dmx * (Nx - Wd)) * Wd * driven_wheels
        ax_tyre_max_dec = -1 / M * (mux + dmx * (Nx - (Wz - Aero_Df) / 4)) * (Wz - Aero_Df)
        ax_power_limit = 1 / M * np.interp(v[i], vehicle_speed, factor_power * fx_engine)
        ax_power_limit = np.full(N, ax_power_limit)
        ay = ay_max * np.cos(np.linspace(0, np.pi, N))
        ax_tyre_acc = ax_tyre_max_acc * np.sqrt(1 - (ay / ay_max)**2)
        ax_acc = np.minimum(ax_tyre_acc, ax_power_limit) + ax_drag
        ax_dec = ax_tyre_max_dec * np.sqrt(1 - (ay / ay_max)**2) + ax_drag
        GGV[i, :, 0] = np.concatenate([ax_acc, ax_dec[1:]])
        GGV[i, :, 1] = np.concatenate([ay, np.flip(ay[1:])])
        GGV[i, :, 2] = v[i] * np.ones(2 * N - 1)
    
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle(name)

    # Subplots
    axs = [fig.add_subplot(2, 2, i+1, projection='3d') if i == 1 else fig.add_subplot(2, 2, i+1) for i in range(4)]

    # 1. Engine Curve
    axs[0].plot(en_speed_curve, factor_power * en_torque_curve, 'b')
    axs[0].set_xlabel('Engine Speed [rpm]')
    axs[0].set_ylabel('Engine Torque [Nm]', color='b')
    axs[0].tick_params(axis='y', labelcolor='b')

    ax2 = axs[0].twinx()
    ax2.plot(en_speed_curve, factor_power * en_power_curve / 745.7, 'r')
    ax2.set_ylabel('Engine Power [Hp]', color='r')
    ax2.tick_params(axis='y', labelcolor='r')

    axs[0].set_title('Engine Curve')
    axs[0].grid(True)

    # 2. GGV Map (3D Plot)
    # Convert the GGV map into three separate arrays for plotting
    lat_acc = GGV[:, :, 1]
    long_acc = GGV[:, :, 0]
    speeds = GGV[:, :, 2]
    
    # 3D GGV Plot
    ax_ggv_3d = axs[1]
    #ax_ggv_3d = fig.add_subplot(121, projection='3d')  # Add subplot for the 3D GGV plot
    ax_ggv_3d.plot_surface(lat_acc, long_acc, speeds, cmap='viridis')

    # Set axis labels
    ax_ggv_3d.set_xlabel('Lateral Acceleration [m/s^2]')
    ax_ggv_3d.set_ylabel('Longitudinal Acceleration [m/s^2]')
    ax_ggv_3d.set_zlabel('Speed [m/s]')
    ax_ggv_3d.set_title('3D GGV Map')

    # 3. Gearing
    axs[2].plot(vehicle_speed, engine_speed, 'b')
    axs[2].set_xlabel('Speed [m/s]')
    axs[2].set_ylabel('Engine Speed [rpm]', color='b')
    axs[2].tick_params(axis='y', labelcolor='b')

    ax2 = axs[2].twinx()
    ax2.plot(vehicle_speed, gear, 'r')
    ax2.set_ylabel('Gear [-]', color='r')
    ax2.tick_params(axis='y', labelcolor='r')

    axs[2].set_title('Gearing')
    axs[2].grid(True)

    # 4. Traction Model
    axs[3].plot(vehicle_speed, factor_power * fx_engine, 'k', linewidth=4, label='Engine tractive force')
    axs[3].plot(vehicle_speed, np.minimum(factor_power * fx_engine, fx_tyre), 'r', linewidth=2, label='Final tractive force')
    axs[3].plot(vehicle_speed, -fx_aero, label='Aero drag')
    axs[3].plot(vehicle_speed, -fx_roll, label='Rolling resistance')
    axs[3].plot(vehicle_speed, fx_tyre, label='Max tyre tractive force')
    for i in range(nog):
        axs[3].plot(vehicle_speed[1:], fx[:, i], 'k--', label=f'Engine tractive force per gear' if i == 0 else '')

    axs[3].set_title('Traction Model')
    axs[3].set_xlabel('Speed [m/s]')
    axs[3].set_ylabel('Force [N]')
    axs[3].grid(True)
    axs[3].legend()
    axs[3].set_xlim([vehicle_speed[0], vehicle_speed[-1]])

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.show()
    return GGV, engine_speed, wheel_torque, engine_torque, engine_power
