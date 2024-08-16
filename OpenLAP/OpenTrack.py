import numpy as np
import pandas as pd
import os
from datetime import datetime
import matplotlib.pyplot as plt
from scipy.signal import find_peaks, savgol_filter

# Function Definitions

def read_info(workbookFile, sheetName, startRow=1, endRow=7):
    """Reads the 'Info' sheet and returns track information."""
    data = pd.read_excel(workbookFile, sheet_name=sheetName, usecols="A:B", skiprows=startRow - 1, nrows=endRow - startRow + 1)
    return {
        'name': data.iloc[0, 1],
        'country': data.iloc[1, 1],
        'city': data.iloc[2, 1],
        'type': data.iloc[3, 1],
        'config': data.iloc[4, 1],
        'direction': data.iloc[5, 1],
        'mirror': data.iloc[6, 1]
    }

def read_shape_data(workbookFile, sheetName, startRow=2, endRow=10000):
    """Reads the 'Shape' sheet and returns the track shape data."""
    data = pd.read_excel(workbookFile, sheet_name=sheetName, usecols="A:C", skiprows=startRow - 1, nrows=endRow - startRow + 1)
    data.columns = ["Type", "SectionLength", "CornerRadius"]
    return data

def read_data(workbookFile, sheetName, startRow=2, endRow=10000):
    """Reads the 'Elevation', 'Banking', 'Grip Factors', and 'Sectors' sheets."""
    data = pd.read_excel(workbookFile, sheet_name=sheetName, usecols="A:B", skiprows=startRow - 1, nrows=endRow - startRow + 1)
    return data

def read_logged_data(filename, header_startRow=1, header_endRow=12, data_startRow=14, data_endRow=None):
    """Reads logged data from a CSV file."""
    header = pd.read_csv(filename, skiprows=header_startRow - 1, nrows=header_endRow - header_startRow + 1)
    data = pd.read_csv(filename, skiprows=data_startRow - 1, nrows=data_endRow)
    return header, data

def rotz(angle_deg):
    """Returns a rotation matrix for a rotation around the z-axis by the given angle in degrees."""
    angle_rad = np.deg2rad(angle_deg)
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

# Main script

# Track file selection
filename = 'OpenTRACK_FSAE_UK_Endurance_2015.xlsx'

# Mode selection
mode = 'shape data'
log_mode = 'speed & latacc'

# Settings
mesh_size = 1  # [m]
filter_dt = 0.1  # [s]
rotation = 0  # [deg]
lambda_factor = 1  # [-]
kappa = 1000  # [deg]

# Reading file
print(f"Reading track file: {filename}")

if mode == 'logged data':
    # From logged data
    header, data = read_logged_data(filename)
    info = {
        'name': header.iloc[1, 1],
        'country': header.iloc[2, 1],
        'city': header.iloc[3, 1],
        'type': header.iloc[4, 1],
        'config': header.iloc[5, 1],
        'direction': header.iloc[6, 1],
        'mirror': header.iloc[7, 1]
    }
    channels = header.iloc[10, :]
    units = header.iloc[11, :]
    freq = float(header.iloc[8, 1])

    # Data columns
    col_dist = 0
    col_vel = 1
    col_yaw = 2
    col_ay = 3
    col_el = 4
    col_bk = 5
    col_gf = 6
    col_sc = 7

    # Extracting data
    x = data.iloc[:, col_dist].to_numpy()
    v = data.iloc[:, col_vel].to_numpy()
    w = data.iloc[:, col_yaw].to_numpy()
    ay = data.iloc[:, col_ay].to_numpy()
    el = data.iloc[:, col_el].to_numpy()
    bk = data.iloc[:, col_bk].to_numpy()
    gf = data.iloc[:, col_gf].to_numpy()
    sc = data.iloc[:, col_sc].to_numpy()

    # Converting units
    if units[col_dist] != "m":
        if units[col_dist] == 'km':
            x *= 1000
        elif units[col_dist] == 'miles':
            x *= 1609.34
        elif units[col_dist] == 'ft':
            x *= 0.3048
        else:
            print("Check distance units.")

    if units[col_vel] != "m/s":
        if units[col_vel] == 'km/h':
            v /= 3.6
        elif units[col_vel] == 'mph':
            v *= 0.44704
        else:
            print("Check speed units.")

    if units[col_yaw] != "rad/s":
        if units[col_yaw] == 'deg/s':
            w *= 2 * np.pi / 360
        elif units[col_yaw] == 'rpm':
            w *= 2 * np.pi / 60
        elif units[col_yaw] == 'rps':
            w *= 2 * np.pi
        else:
            print("Check yaw velocity units.")

    if units[col_ay] != "m/s^2":
        if units[col_ay] == 'G':
            ay *= 9.81
        elif units[col_ay] == 'ft/s^2':
            ay *= 0.3048
        else:
            print("Check lateral acceleration units.")

    if units[col_el] != "m":
        if units[col_el] == 'km':
            el *= 1000
        elif units[col_el] == 'miles':
            el *= 1609.34
        elif units[col_el] == 'ft':
            el *= 0.3048
        else:
            print("Check elevation units.")

    if units[col_bk] != "deg":
        if units[col_bk] == 'rad':
            bk = bk * 180 / np.pi
        else:
            print("Check banking units.")

else:
    # From shape data
    info = read_info(filename, 'Info')
    table_shape = read_shape_data(filename, 'Shape')
    table_el = read_data(filename, 'Elevation')
    table_bk = read_data(filename, 'Banking')
    table_gf = read_data(filename, 'Grip Factors')
    table_sc = read_data(filename, 'Sectors')

# Track model name
os.makedirs('OpenTRACK Tracks', exist_ok=True)
trackname = f"OpenTRACK Tracks/OpenTRACK_{info['name']}_{info['config']}_{info['direction']}"
if info['mirror'] == "On":
    trackname += "_Mirrored"

# HUD
log_filename = f"{trackname}.log"
if os.path.exists(log_filename):
    os.remove(log_filename)

with open(log_filename, "w") as log_file:
    log_file.write(f"{filename}\n")
    log_file.write("File read successfully\n")
    log_file.write(f"Name:          {info['name']}\n")
    log_file.write(f"City:          {info['city']}\n")
    log_file.write(f"Country:       {info['country']}\n")
    log_file.write(f"Type:          {info['type']}\n")
    log_file.write(f"Configuration: {info['config']}\n")
    log_file.write(f"Direction:     {info['direction']}\n")
    log_file.write(f"Mirror:        {info['mirror']}\n")
    log_file.write(f"Date:          {datetime.now().strftime('%d/%m/%Y')}\n")
    log_file.write(f"Time:          {datetime.now().strftime('%H:%M:%S')}\n")
    log_file.write("Track generation started.\n")

# Pre-processing
if mode == 'logged data':
    x, indices = np.unique(x, return_index=True)
    v = savgol_filter(v[indices], int(freq * filter_dt), 3)
    w = savgol_filter(w[indices], int(freq * filter_dt), 3)
    ay = savgol_filter(ay[indices], int(freq * filter_dt), 3)
    el = savgol_filter(el[indices], int(freq * filter_dt), 3)
    bk = savgol_filter(bk[indices], int(freq * filter_dt), 3)
    gf = gf[indices]
    sc = sc[indices]

    # Shifting position vector for 0 value at start
    x = x - x[0]

    # Curvature
    if log_mode == 'speed & yaw':
        r = lambda_factor * w / v
    elif log_mode == 'speed & latacc':
        r = lambda_factor * ay / v**2

    r = savgol_filter(r, int(freq * filter_dt), 3)

    # Mirroring if needed
    if info['mirror'] == "On":
        r = -r

    # Track length
    L = x[-1]

    # Saving coarse position vectors
    xx = x.copy()
    xe = x.copy()
    xb = x.copy()
    xg = x.copy()
    xs = x.copy()

else:
    # From shape data
    R = table_shape["CornerRadius"].to_numpy()
    l = table_shape["SectionLength"].to_numpy()
    type_tmp = table_shape["Type"].astype(str).to_numpy()

    # Correcting straight segment radius
    R[R == 0] = np.inf

    # Total length
    L = np.sum(l)

    # Segment type variable conversion to number
    type_numeric = np.zeros(len(l))
    type_numeric[type_tmp == "Straight"] = 0
    type_numeric[type_tmp == "Left"] = 1
    type_numeric[type_tmp == "Right"] = -1

    if info['mirror'] == "On":
        type_numeric = -type_numeric

    # Removing segments with zero length
    non_zero_indices = l > 0
    R = R[non_zero_indices]
    l = l[non_zero_indices]
    type_numeric = type_numeric[non_zero_indices]

    # Injecting points at long corners
    angle_seg = np.rad2deg(l / R)
    RR = []
    ll = []
    tt = []

    for i in range(len(l)):
        if angle_seg[i] > kappa:
            l_inj = min(ll[i] / 3, np.deg2rad(kappa) * R[i])
            RR.extend([R[i]] * 3)
            ll.extend([l_inj, ll[i] - 2 * l_inj, l_inj])
            tt.extend([type_numeric[i]] * 3)
        else:
            RR.append(R[i])
            ll.append(l[i])
            tt.append(type_numeric[i])

    R = np.array(RR)
    l = np.array(ll)
    type_numeric = np.array(tt)

    # Replacing consecutive straights
    straight_indices = np.where(type_numeric == 0)[0]
    i = 0
    while i < len(straight_indices) - 1:
        if l[straight_indices[i]] != -1:
            l[straight_indices[i]] += l[straight_indices[i + 1]]
            l[straight_indices[i + 1]] = -1
        i += 1

    non_minus_one_indices = l != -1
    R = R[non_minus_one_indices]
    l = l[non_minus_one_indices]
    type_numeric = type_numeric[non_minus_one_indices]

    # Final segment point calculation
    X = np.cumsum(l)
    XC = np.cumsum(l) - l / 2

    x = []
    r = []

    for i in range(len(X)):
        if R[i] == np.inf:
            x.extend([X[i] - l[i], X[i]])
            r.extend([0, 0])
        else:
            x.append(XC[i])
            r.append(type_numeric[i] / R[i])

    x = np.array(x)
    r = np.array(r)

    # Getting data from tables and ignoring points with x > L
    el = table_el.to_numpy()
    el = el[el[:, 0] <= L]

    bk = table_bk.to_numpy()
    bk = bk[bk[:, 0] <= L]

    gf = table_gf.to_numpy()
    gf = gf[gf[:, 0] <= L]

    sc = table_sc.to_numpy()
    sc = sc[sc[:, 0] < L]
    sc = np.vstack([sc, [L, sc[-1, -1]]])

    # Saving coarse position vectors
    xx = x.copy()
    xe = el[:, 0]
    xb = bk[:, 0]
    xg = gf[:, 0]
    xs = sc[:, 0]

    # Saving coarse topology
    el = el[:, 1]
    bk = bk[:, 1]
    gf = gf[:, 1]
    sc = sc[:, 1]

# HUD
print('Pre-processing completed.')

# Meshing
if L % 1 != 0:
    x = np.concatenate((np.arange(0, np.floor(L), mesh_size), [L]))
else:
    x = np.arange(0, L, mesh_size)

dx = np.diff(x, append=x[-1])
n = len(x)
r = np.interp(x, xx, r)
Z = np.interp(x, xe, el)
bank = np.interp(x, xb, bk)
incl = -np.degrees(np.arctan2(np.diff(Z, append=Z[-1]), dx))
factor_grip = np.interp(x, xg, gf)
sector = np.interp(x, xs, sc, left=sc[0], right=sc[-1])

print(f"Fine meshing completed with mesh size: {mesh_size} [m]")

# Map generation
X = np.zeros(n)
Y = np.zeros(n)

angle_seg = np.degrees(dx * r)
angle_head = np.cumsum(angle_seg)

if info['config'] == 'Closed':
    dh = np.mod(angle_head[-1], np.sign(angle_head[-1]) * 360)
    angle_head -= x / L * dh

angle_head -= angle_head[0]

for i in range(1, n):
    prev_point = np.array([X[i - 1], Y[i - 1], 0])
    xyz = rotz(angle_head[i - 1]) @ np.array([dx[i - 1], 0, 0]) + prev_point
    X[i], Y[i], _ = xyz

# Apexes
_, apex = find_peaks(np.abs(r))
r_apex = r[apex]

print('Apex calculation completed.')

# Map edit
if info['direction'] == 'Backward':
    x = x[-1] - np.flip(x)
    r = -np.flip(r)
    apex = len(x) - np.flip(apex)
    r_apex = -np.flip(r_apex)
    incl = -np.flip(incl)
    bank = -np.flip(bank)
    factor_grip = np.flip(factor_grip)
    sector = np.flip(sector)
    X = np.flip(X)
    Y = np.flip(Y)
    Z = np.flip(Z)

# Track rotation
xyz = rotz(rotation) @ np.array([X, Y, Z])
X, Y, Z = xyz[0], xyz[1], xyz[2]

print('Track rotated.')

# Closing map if necessary
if info['config'] == 'Closed':
    print('Closing fine mesh map.')
    DX = x / L * (X[0] - X[-1])
    DY = x / L * (Y[0] - Y[-1])
    DZ = x / L * (Z[0] - Z[-1])
    db = x / L * (bank[0] - bank[-1])
    X += DX
    Y += DY
    Z += DZ
    bank += db
    incl = -np.degrees(np.arctan2(np.diff(Z, append=Z[-1]), dx))
    print('Fine mesh map closed.')

incl = savgol_filter(incl, 51, 3)
print('Fine mesh map created.')

# Plotting Results

# Finish line arrow
factor_scale = 25
half_angle = 40
scale = max(max(X) - min(X), max(Y) - min(Y)) / factor_scale
arrow_n = np.array([X[0] - X[1], Y[0] - Y[1], Z[0] - Z[1]]) / np.linalg.norm([X[0] - X[1], Y[0] - Y[1], Z[0] - Z[1]])
arrow_1 = scale * rotz(half_angle) @ arrow_n + np.array([X[0], Y[0], Z[0]])
arrow_c = np.array([X[0], Y[0], Z[0]])
arrow_2 = scale * rotz(-half_angle) @ arrow_n + np.array([X[0], Y[0], Z[0]])
arrow_x = np.array([arrow_1[0], arrow_c[0], arrow_2[0]])
arrow_y = np.array([arrow_1[1], arrow_c[1], arrow_2[1]])
arrow_z = np.array([arrow_1[2], arrow_c[2], arrow_2[2]])
arrow = np.array([arrow_x, arrow_y, arrow_z])

# Plotting
fig, axs = plt.subplots(5, 2, figsize=(12, 12))
fig.suptitle(f"OpenTRACK - Track Name: {info['name']}, Configuration: {info['config']}, Mirror: {info['mirror']}, Date & Time: {datetime.now().strftime('%Y/%m/%d %H:%M:%S')}", fontsize=16)

# 3D Map
axs[0, 0].set_title('3D Map')
axs[0, 0].scatter(X, Y, c=sector, cmap='viridis')
axs[0, 0].plot(arrow_x, arrow_y, 'k', linewidth=2)
axs[0, 0].grid(True)
axs[0, 0].axis('equal')
axs[0, 0].set_xlabel('x [m]')
axs[0, 0].set_ylabel('y [m]')

# Curvature
axs[0, 1].set_title('Curvature')
axs[0, 1].plot(x, r)
axs[0, 1].scatter(x[apex], r_apex, color='red')
axs[0, 1].grid(True)
axs[0, 1].set_xlabel('position [m]')
axs[0, 1].set_ylabel('curvature [m^-1]')

# Elevation
axs[1, 1].set_title('Elevation')
axs[1, 1].plot(x, Z)
axs[1, 1].grid(True)
axs[1, 1].set_xlabel('position [m]')
axs[1, 1].set_ylabel('elevation [m]')

# Inclination
axs[2, 1].set_title('Inclination')
axs[2, 1].plot(x, incl)
axs[2, 1].grid(True)
axs[2, 1].set_xlabel('position [m]')
axs[2, 1].set_ylabel('inclination [deg]')

# Banking
axs[3, 1].set_title('Banking')
axs[3, 1].plot(x, bank)
axs[3, 1].grid(True)
axs[3, 1].set_xlabel('position [m]')
axs[3, 1].set_ylabel('banking [deg]')

# Grip Factors
axs[4, 1].set_title('Grip Factor')
axs[4, 1].plot(x, factor_grip)
axs[4, 1].grid(True)
axs[4, 1].set_xlabel('position [m]')
axs[4, 1].set_ylabel('grip factor [-]')

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.savefig(f"{trackname}.png")
plt.show()

print('Plots created and saved.')

# Saving circuit
np.savez_compressed(f"{trackname}.npz", info=info, x=x, dx=dx, n=n, r=r, bank=bank, incl=incl, factor_grip=factor_grip, sector=sector, r_apex=r_apex, apex=apex, X=X, Y=Y, Z=Z, arrow=arrow)

print('Track generated successfully.')

# ASCII map
charh = 15  # font height [pixels]
charw = 8  # font width [pixels]
linew = 66  # log file character width
mapw = max(X) - min(X)  # map width
YY = np.round(Y / (charh / charw) / mapw * linew).astype(int)  # scales y values
XX = np.round(X / mapw * linew).astype(int)  # scales x values
YY = -YY - min(-YY)  # flipping y and shifting to positive space
XX = XX - min(XX)  # shifting x to positive space
p = np.unique(np.vstack((XX, YY)).T, axis=0)  # getting unique points
XX, YY = p[:, 0], p[:, 1]
maph = max(YY)  # getting new map height [lines]
mapw = max(XX)  # getting new map width [columns]
map = np.full((maph, mapw), ' ')  # character map preallocation

# Looping through characters
for i, j in zip(XX, YY):
    map[j, i] = 'o'

map_str = '\n'.join(''.join(row) for row in map)
print('Map:')
print(map_str)

with open(log_filename, "a") as log_file:
    log_file.write(map_str)

print('Track generation completed.')
