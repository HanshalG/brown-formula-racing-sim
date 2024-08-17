##### HAVEN'T PORTED LOOGGED DATA, MIRROR, ROTATION
###################################################

import numpy as np
import pandas as pd
import os
from datetime import datetime
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from scipy.interpolate import PchipInterpolator, interp1d


def find_peaks_np(data):
    return np.where((data[:-2] < data[1:-1]) & (data[1:-1] > data[2:]))[0] + 1

def read_info(workbookFile, sheetName, startRow=1, endRow=7):
    """Reads the 'Info' sheet and returns track information."""
    data = pd.read_excel(workbookFile, sheet_name=sheetName, usecols="A:B", nrows=endRow - startRow + 1, header=None)
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
    data = pd.read_excel(workbookFile, sheet_name=sheetName, usecols="A:C", nrows=endRow - startRow + 1)
    data.columns = ["Type", "SectionLength", "CornerRadius"]
    return data

def read_data(workbookFile, sheetName, startRow=2, endRow=10000):
    """Reads the 'Elevation', 'Banking', 'Grip Factors', and 'Sectors' sheets."""
    data = pd.read_excel(workbookFile, sheet_name=sheetName, usecols="A:B", nrows=endRow - startRow + 1)
    return data

def read_logged_data(filename, header_startRow=1, header_endRow=12, data_startRow=14, data_endRow=None):
    """Reads logged data from a CSV file."""
    header = pd.read_csv(filename, nrows=header_endRow - header_startRow + 1)
    data = pd.read_csv(filename, nrows=data_endRow)
    return header, data

def rotz(angle_deg):
    """Returns a rotation matrix for a rotation around the z-axis by the given angle in degrees."""
    angle_rad = np.deg2rad(angle_deg)
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def output_track_dict(track_filepath):
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
    print(f"Reading track file: {track_filepath}")

    if mode == 'shape data':
        # From shape data
        info = read_info(track_filepath, 'Info')
        table_shape = read_shape_data(track_filepath, 'Shape')
        table_el = read_data(track_filepath, 'Elevation')
        table_bk = read_data(track_filepath, 'Banking')
        table_gf = read_data(track_filepath, 'Grip Factors')
        table_sc = read_data(track_filepath, 'Sectors')

    print(f"{track_filepath}")
    print("File read successfully")
    print(f"Name:          {info['name']}")
    print(f"City:          {info['city']}")
    print(f"Country:       {info['country']}")
    print(f"Type:          {info['type']}")
    print(f"Configuration: {info['config']}")
    print(f"Direction:     {info['direction']}")
    print(f"Mirror:        {info['mirror']}")
    print(f"Date:          {datetime.now().strftime('%d/%m/%Y')}")
    print(f"Time:          {datetime.now().strftime('%H:%M:%S')}")
    print("Track generation started.")

    if mode == 'shape data':
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
    # r = np.interp(x, xx, r)

    # print("Elevation Table:\n", table_el)
    # print("Banking Table:\n", table_bk)
    # print("Grip Factors Table:\n", table_gf)
    # print("Sectors Table:\n", table_sc)

    # print("Filtered Elevation Data:\n", el)
    # print("Filtered Banking Data:\n", bk)
    # print("Filtered Grip Factors Data:\n", gf)
    # print("Filtered Sectors Data:\n", sc)

    # Z = np.interp(x, xe, el)
    # bank = np.interp(x, xb, bk)
    # incl = -np.degrees(np.arctan2(np.diff(Z, append=Z[-1]), dx))
    # factor_grip = np.interp(x, xg, gf)
    # sector = np.interp(x, xs, sc, left=sc[0], right=sc[-1])

    # Fine curvature vector
    pchip_interpolator = PchipInterpolator(xx, r, extrapolate=True)
    r = pchip_interpolator(x)

    #np.set_printoptions(threshold=np.inf)
    #print("Arr ", r)
    #np.set_printoptions(threshold=2)

    # Elevation
    elevation_interpolator = interp1d(xe, el, kind='linear', fill_value='extrapolate')
    Z = elevation_interpolator(x)

    # Banking
    bank_interpolator = interp1d(xb, bk, kind='linear', fill_value='extrapolate')
    bank = bank_interpolator(x)

    # Inclination
    dZ_dx = np.diff(Z) / np.diff(x)
    incl = -np.degrees(np.arctan(dZ_dx))
    incl = np.append(incl, incl[-1])  # Append the last value

    # Grip factor
    grip_factor_interpolator = interp1d(xg, gf, kind='linear', fill_value='extrapolate')
    factor_grip = grip_factor_interpolator(x)

    # Sector
    sector_interpolator = interp1d(xs, sc, kind='previous', fill_value='extrapolate')
    sector = sector_interpolator(x)

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
    #_, apex = find_peaks(np.abs(r))
    apex = find_peaks_np(np.abs(r))
    r_apex = r[apex]

    print('Apex calculation completed.')

    # # Closing map if necessary
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
    fig, axs = plt.subplots(3, 2, figsize=(12, 15))
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
    axs[1, 0].set_title('Elevation')
    axs[1, 0].plot(x, Z)
    axs[1, 0].grid(True)
    axs[1, 0].set_xlabel('position [m]')
    axs[1, 0].set_ylabel('elevation [m]')

    # Inclination
    axs[1, 1].set_title('Inclination')
    axs[1, 1].plot(x, incl)
    axs[1, 1].grid(True)
    axs[1, 1].set_xlabel('position [m]')
    axs[1, 1].set_ylabel('inclination [deg]')

    # Banking
    axs[2, 0].set_title('Banking')
    axs[2, 0].plot(x, bank)
    axs[2, 0].grid(True)
    axs[2, 0].set_xlabel('position [m]')
    axs[2, 0].set_ylabel('banking [deg]')

    # Grip Factors
    axs[2, 1].set_title('Grip Factor')
    axs[2, 1].plot(x, factor_grip)
    axs[2, 1].grid(True)
    axs[2, 1].set_xlabel('position [m]')
    axs[2, 1].set_ylabel('grip factor [-]')

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    #plt.savefig(f"{track_filepath}.png")
    #plt.show()
    plt.clf()
    print('Plots created and saved.')

    tr = {
        'info': info,
        'n': n,
        'r': r,
        'x': x,
        'Z': Z,
        'dx': dx,
        'sector': sector,
        'bank': bank,
        'incl': incl,
        'factor_grip': factor_grip,
        'X': X,
        'Y': Y,
        'Z': Z,
        'arrow': arrow
    }

    return tr
