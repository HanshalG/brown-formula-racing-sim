import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt
from datetime import datetime

# Vehicle file selection
filename = 'Formula 1.xlsx'

# Reading vehicle file
def read_info(workbookFile, sheetName, startRow=2, endRow=10000):
    # Read specified range and columns from Excel
    data = pd.read_excel(workbookFile, sheet_name=sheetName, usecols="B:C", skiprows=startRow - 1, nrows=endRow - startRow + 1)
    # Drop any rows with missing data
    data.dropna(inplace=True)
    return data


def read_torque_curve(workbookFile, sheetName, startRow=2, endRow=10000):
    # Read specified range and columns from Excel
    data = pd.read_excel(workbookFile, sheet_name=sheetName, usecols="A:B", skiprows=startRow - 1, nrows=endRow - startRow + 1)
    # Drop any rows with missing data
    data.dropna(inplace=True)
    return data


info = read_info(filename, 'Info')
data = read_torque_curve(filename, 'Torque Curve')

# Getting variables
name = info.iloc[0, 1]
type_vehicle = info.iloc[1, 1]

i = 2  # index starts at 2 for Python
M = float(info.iloc[i, 1]); i += 1
df = float(info.iloc[i, 1]) / 100; i += 1
L = float(info.iloc[i, 1]) / 1000; i += 1
rack = float(info.iloc[i, 1]); i += 1
Cl = float(info.iloc[i, 1]); i += 1
Cd = float(info.iloc[i, 1]); i += 1
factor_Cl = float(info.iloc[i, 1]); i += 1
factor_Cd = float(info.iloc[i, 1]); i += 1
da = float(info.iloc[i, 1]) / 100; i += 1
A = float(info.iloc[i, 1]); i += 1
rho = float(info.iloc[i, 1]); i += 1
br_disc_d = float(info.iloc[i, 1]) / 1000; i += 1
br_pad_h = float(info.iloc[i, 1]) / 1000; i += 1
br_pad_mu = float(info.iloc[i, 1]); i += 1
br_nop = float(info.iloc[i, 1]); i += 1
br_pist_d = float(info.iloc[i, 1]); i += 1
br_mast_d = float(info.iloc[i, 1]); i += 1
br_ped_r = float(info.iloc[i, 1]); i += 1
factor_grip = float(info.iloc[i, 1]); i += 1
tyre_radius = float(info.iloc[i, 1]) / 1000; i += 1
Cr = float(info.iloc[i, 1]); i += 1
mu_x = float(info.iloc[i, 1]); i += 1
mu_x_M = float(info.iloc[i, 1]); i += 1
sens_x = float(info.iloc[i, 1]); i += 1
mu_y = float(info.iloc[i, 1]); i += 1
mu_y_M = float(info.iloc[i, 1]); i += 1
sens_y = float(info.iloc[i, 1]); i += 1
CF = float(info.iloc[i, 1]); i += 1
CR = float(info.iloc[i, 1]); i += 1
factor_power = float(info.iloc[i, 1]); i += 1
n_thermal = float(info.iloc[i, 1]); i += 1
fuel_LHV = float(info.iloc[i, 1]); i += 1
drive = info.iloc[i, 1]; i += 1
shift_time = float(info.iloc[i, 1]); i += 1
n_primary = float(info.iloc[i, 1]); i += 1
n_final = float(info.iloc[i, 1]); i += 1
n_gearbox = float(info.iloc[i, 1]); i += 1
ratio_primary = float(info.iloc[i, 1]); i += 1
ratio_final = float(info.iloc[i, 1]); i += 1
ratio_gearbox = info.iloc[i:, 1].astype(float).values
nog = len(ratio_gearbox)

# HUD
os.makedirs('OpenVEHICLE Vehicles', exist_ok=True)
vehname = f"OpenVEHICLE Vehicles/OpenVEHICLE_{name}_{type_vehicle}"
log_filename = f"{vehname}.log"
if os.path.exists(log_filename):
    os.remove(log_filename)

with open(log_filename, "w") as log_file:
    log_file.write(f"{filename}\n")
    log_file.write('File read successfully\n')
    log_file.write(f"Name: {name}\n")
    log_file.write(f"Type: {type_vehicle}\n")
    log_file.write(f"Date: {datetime.now().strftime('%d/%m/%Y')}\n")
    log_file.write(f"Time: {datetime.now().strftime('%H:%M:%S')}\n")
    log_file.write('Vehicle generation started.\n')

# Brake Model
br_pist_a = br_nop * np.pi * (br_pist_d / 1000)**2 / 4
br_mast_a = np.pi * (br_mast_d / 1000)**2 / 4
beta = tyre_radius / (br_disc_d / 2 - br_pad_h / 2) / br_pist_a / br_pad_mu / 4
phi = br_mast_a / br_ped_r * 2
with open(log_filename, "a") as log_file:
    log_file.write('Braking model generated successfully.\n')

# Steering Model
a = (1 - df) * L
b = -df * L
C_matrix = 2 * np.array([[CF, CF + CR], [CF * a, CF * a + CR * b]])
with open(log_filename, "a") as log_file:
    log_file.write('Steering model generated successfully.\n')

# Driveline Model
en_speed_curve = data.iloc[:, 0].values
en_torque_curve = data.iloc[:, 1].values
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

with open(log_filename, "a") as log_file:
    log_file.write('Driveline model generated successfully.\n')

# Shifting Points and Rev Drops
gear_change = np.diff(gear)
gear_change = np.logical_or(gear_change, np.roll(gear_change, 1))
engine_speed_gear_change = engine_speed[gear_change]
shift_points = engine_speed_gear_change[0::2]
arrive_points = engine_speed_gear_change[1::2]
rev_drops = shift_points - arrive_points

shifting = pd.DataFrame({
    'shift_points': shift_points,
    'arrive_points': arrive_points,
    'rev_drops': rev_drops
}, index=[f'{i}-{i+1}' for i in range(1, nog)])

with open(log_filename, "a") as log_file:
    log_file.write('Shift points calculated successfully.\n')

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

with open(log_filename, "a") as log_file:
    log_file.write('Forces calculated successfully.\n')

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

with open(log_filename, "a") as log_file:
    log_file.write('GGV map generated successfully.\n')

# Saving vehicle
np.savez_compressed(f"{vehname}.npz", GGV=GGV, engine_speed=engine_speed, wheel_torque=wheel_torque,
                    engine_torque=engine_torque, engine_power=engine_power)

# Plot
fig, axs = plt.subplots(4, 2, figsize=(12, 12))
fig.suptitle(name)

# Engine curves
axs[0, 0].plot(en_speed_curve, factor_power * en_torque_curve)
axs[0, 0].set_title('Engine Curve')
axs[0, 0].set_xlabel('Engine Speed [rpm]')
axs[0, 0].set_ylabel('Engine Torque [Nm]')
axs[0, 0].grid(True)
axs[0, 0].twinx().plot(en_speed_curve, factor_power * en_power_curve / 745.7, 'r')
axs[0, 0].set_ylabel('Engine Power [Hp]')

# Gearing
axs[1, 0].plot(vehicle_speed, engine_speed)
axs[1, 0].set_title('Gearing')
axs[1, 0].set_xlabel('Speed [m/s]')
axs[1, 0].set_ylabel('Engine Speed [rpm]')
axs[1, 0].grid(True)
axs[1, 0].twinx().plot(vehicle_speed, gear, 'r')
axs[1, 0].set_ylabel('Gear [-]')

# Traction model
axs[2, 0].plot(vehicle_speed, factor_power * fx_engine, 'k', linewidth=4)
axs[2, 0].plot(vehicle_speed, np.minimum(factor_power * fx_engine, fx_tyre), 'r', linewidth=2)
axs[2, 0].plot(vehicle_speed, -fx_aero)
axs[2, 0].plot(vehicle_speed, -fx_roll)
axs[2, 0].plot(vehicle_speed, fx_tyre)
for i in range(nog):
    axs[2, 0].plot(vehicle_speed[1:], fx[:, i], 'k--')
axs[2, 0].set_title('Traction Model')
axs[2, 0].set_xlabel('Speed [m/s]')
axs[2, 0].set_ylabel('Force [N]')
axs[2, 0].grid(True)

# GGV map
ax_ggv = axs[1, 1].contourf(GGV[:, :, 1], GGV[:, :, 0], GGV[:, :, 2], cmap='viridis')
fig.colorbar(ax_ggv, ax=axs[1, 1])
axs[1, 1].set_title('GGV Map')
axs[1, 1].set_xlabel('Lat acc [m/s^2]')
axs[1, 1].set_ylabel('Long acc [m/s^2]')
axs[1, 1].grid(True)

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.savefig(f"{vehname}.png")
plt.show()

with open(log_filename, "a") as log_file:
    log_file.write('Plots created and saved.\n')
    log_file.write('Vehicle generated successfully.\n')
