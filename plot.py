import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.patches import Polygon, Rectangle, Arrow, Circle
from matplotlib import rc
from matplotlib.lines import Line2D
from matplotlib.transforms import Affine2D
from matplotlib.ticker import LogLocator
import json
import csv
import sys

from lib.control import get_distance_NE, offset
from lib.simulation import simulate_primitive
from lib.plot import plot_primitive

def get_solution_wps(path='route/sol.txt'):
    with open(path) as f:
        raw = json.load(f)
        wps = []
        for f in raw['features']:
            coords = f['geometry']['coordinates']
            wps += [{'lat': coords[1], 'lng': coords[0]}]
    return wps

def get_simulated(wp, path):
    origin = [wp['lat'], wp['lng']]
    with open(path) as f:
        raw = json.load(f)
        sim = np.zeros((len(raw['features']), 2))
        for i, f in enumerate(raw['features']):
            coords = f['geometry']['coordinates']
            sim[i, :] = get_distance_NE(origin, [coords[1], coords[0]])
    return sim

def read_obstacles(path):
    with open('{}/obstacles.txt'.format(path)) as f:
        raw = f.read()
        lines = raw.split('\n')[:-1]
        obstacles = []
        for line in lines:
            values = line.split(' ')[:-1]
            obst = np.zeros((0, 2))
            for v in values:
                coords = v.split(',')
                p = np.array([float(coords[1]), float(coords[0])]).reshape(1, -1)
                obst = np.append(obst, p, axis=0)
            obstacles += [obst]
        return obstacles

def read_landing(path):
    with open('{}/landing.txt'.format(path)) as f:
        raw = f.read()
        data = raw.split('\n')[0].split(' ')
        x, y, w, h, r = [float(d) for d in data]
        r_new = 90-r
        hdg = np.radians(r + 90)
        x_new, y_new = np.array([x, y]) + h*np.array([np.cos(hdg), np.sin(hdg)])
        return x_new, y_new, w, h, r_new

def read_grid():
    with open('route/grid_search.txt') as f:
        raw = f.read()
        lines = raw.split('\n')[:-1]
        nx = len(lines)
        ny = len(lines[0].split(' ')[:-1])
        grid = np.zeros((nx, ny))
        for i, line in enumerate(lines):
            values = line.split(' ')[:-1]
            grid[i, :] = np.array([np.min([float(v), 1000]) if float(v) > 0 else np.nan for v in values ])
        return grid

def read_hlut():
    with open('route/hlut_viz.txt') as f:
        raw = f.read()
        lines = raw.split('\n')[:-1]
        nx = len(lines)
        ny = len(lines[0].split(' ')[:-1])
        grid = np.zeros((nx, ny))
        for i, line in enumerate(lines):
            values = line.split(' ')[:-1]
            grid[i, :] = np.array([v for v in values])
        return grid

def plot_prims_from_file(path="", color="blue", axis=None):
    with open('route/primitives/visual{}.txt'.format(path)) as f:
        raw = f.read()
        for line in raw.split('\n')[:-1]:
            prim = np.zeros((0, 2))
            vals = line.split(' ')[:-1]
            wp = vals[0]
            x, y = wp.split(',')
            axis.scatter(float(y), float(x), color='red')
            axis.plot([0, float(y)], [0, float(x)], color="red", linestyle=":")
            for val in vals[1:]:
                x, y = val.split(',')
                prim = np.append(prim, np.array([float(x), float(y)]).reshape(1, -1), axis=0)
            axis.plot(prim[:, 1], prim[:, 0], color=color)

def drone_polygon(ax, color, position=(0,0), rotation=0):
    poly = Polygon((
        (-5,-5),
        (0, 5),
        (5,-5),
        (0,-2),
        (-5,-5)
    ), color=color)
    
    dx, dy = position
    t = Affine2D().translate(dy, dx)
    r = Affine2D().rotate_deg_around(dy, dx, -rotation)
    poly.set_transform(t + r + ax.transData)
    
    return poly

def grid_main():
    rc('text', usetex=True)
    grid = read_grid()
    step = 2.5
    x_ticks = step*np.arange(1, grid.shape[1]+1, 1)
    y_ticks = step*np.arange(-grid.shape[1], grid.shape[1]+1, 1)

    X, Y = np.meshgrid(x_ticks, y_ticks)
    fig, ax = plt.subplots()
    c = ax.contourf(X, Y, grid, 15)
    #c = ax.imshow(grid[:, :], extent=[step, 160*step, -160*step, 160*step], origin='lower')
    cb = fig.colorbar(c, ticks=np.linspace(100, 1000, 10))
    cb.set_label('$J$', rotation=0)
    
    u = [50, 90]

    traj = get_simulated({'lat': 0, 'lng': 0}, "route/opt_test_traj.txt")
    ax.plot(traj[:, 1], traj[:, 0], label='trajectory')

    start_poly = drone_polygon(ax, 'green')
    ax.add_patch(start_poly)

    end_pos = traj[-1, :]
    end_poly = drone_polygon(ax, 'red', position=end_pos, rotation=115)
    ax.add_patch(end_poly)

    ax.scatter(u[1], u[0], color='red', label='$u$')
    ax.plot([0, u[1]], [0, u[0]], color='red', linestyle=':')

    wind_dir = np.radians(0)
    ax.quiver(50, -75, np.sin(wind_dir), np.cos(wind_dir), scale=10)
    ax.text(55, -62.5, '$\mathbf{w}$')

    ax.set_xlim([-10, 300])
    ax.set_ylim([-155, 155])
    ax.set_xlabel('$y_E$ [m]')
    ax.set_ylabel('$x_N$ [m]')
    
    plt.legend()
    plt.show()

def prim_diff_main():
    rc('text', usetex=True)
    sim_1 = simulate_primitive(-180, 0, wind_dir=90, wind_spd=5, init_yaw=5)
    sim_2 = simulate_primitive(-180, 0, wind_dir=90, wind_spd=5, init_yaw=-5)
    _, ax = plt.subplots()
    plot_primitive(sim_1, ax=ax, label="$\psi_0=5$")
    plot_primitive(sim_2, ax=ax, label="$\psi_0=-5$")
    ax.scatter(0, -180, color="red", label="reference")
    ax.plot([0,0], [0,-180], color="red", linestyle=":")
    wind_rad = np.pi/2
    plot_wind(ax, 90, 100, -150)
    ax.set_xlabel('$y_E$ [m]')
    ax.set_ylabel('$x_N$ [m]')

    plt.legend()
    plt.show()

def plot_wind(ax, wind_dir, x, y):
    rad = np.radians(wind_dir)
    ax.quiver(x, y, np.sin(rad), np.cos(rad), scale=10)
    ax.text(x+7.5, y+7.5, '$\mathbf{w}$')

def prim_main():
    rc('text', usetex=True)
    cmap = cm.get_cmap('tab10')
    _, ax = plt.subplots()
    plot_prims_from_file(path="_low", color=cmap(0), axis=ax)
    plot_prims_from_file(axis=ax, color=cmap(0.1))
    plot_prims_from_file(path="_high", color=cmap(0.2), axis=ax)
    plt.scatter(0, 0, color="red")
    ax.axis('equal')

    wind_dir = np.radians(0)
    wind_dir = np.radians(0)
    plot_wind(ax, 0, 0, -160)
    test = ax.scatter([], [], color='red')
    legend_elements = [
        Line2D([0], [0], color=cmap(0)), 
        Line2D([0], [0], color=cmap(0.1)),
        Line2D([0], [0], color=cmap(0.2)),
        test
    ]
    ax.legend(legend_elements, ["$W_{max}$", "$W$", "$W_{min}$", "$u$"])
    ax.set_xlabel('$y_E$ [m]')
    ax.set_ylabel('$x_N$ [m]')
    plt.show()

def improve_main():
    wps_init = get_solution_wps(path='route/plot_data/sol_unfiltered.txt')
    wps_improved = get_solution_wps(path='route/plot_data/sol.txt')
    origin = [wps_init[0]['lat'], wps_init[0]['lng']]

    traj_init = get_simulated(wps_init[0], 'route/plot_data/sol_unfiltered_traj.txt')
    traj_improved = get_simulated(wps_init[0], 'route/plot_data/sol_traj.txt')

    local_wps_improved = np.zeros((len(wps_improved), 2))
    local_wps_init = np.zeros((len(wps_init), 2))

    for i, wp in enumerate(wps_init):
        local_wps_init[i, :] = get_distance_NE(origin, [wp['lat'], wp['lng']])
    for i, wp in enumerate(wps_improved):
        local_wps_improved[i, :] = get_distance_NE(origin, [wp['lat'], wp['lng']])

    fig, ax = plt.subplots()
    cmap = cm.get_cmap('tab10')

    color_init = cmap(0)
    color_improved = cmap(0.1)

    ax.scatter(local_wps_init[:, 1], local_wps_init[:, 0], color=color_init, label='initial')
    ax.plot(local_wps_init[:, 1], local_wps_init[:, 0], linestyle=':', color=color_init)
    ax.plot(traj_init[:, 1], traj_init[:, 0], color=color_init)
    ax.plot()

    ax.scatter(local_wps_improved[:, 1], local_wps_improved[:, 0], color=color_improved, label='improved')
    ax.plot(local_wps_improved[:, 1], local_wps_improved[:, 0], linestyle=':', color=color_improved)
    ax.plot(traj_improved[:, 1], traj_improved[:, 0], color=color_improved)

    obst = [[ 103.701, -323.76 ],
            [ 223.861, -323.76 ],
            [223.861, -229.161],
            [ 103.701, -229.161]]

    ax.add_patch(Polygon(obst, alpha=0.8, facecolor='red', label='obstacle'))
    ax.axis('equal')

    ax.set_xlabel('$y_E$ [m]')
    ax.set_ylabel('$x_N$ [m]')

    ax.legend()
    plt.show()

def hlut_main():
    hlut = read_hlut()

    _, ax = plt.subplots()
    c = ax.imshow(hlut, extent=[-400, 400, -400, 400], origin='lower')
    plt.show()

def local_points(origin, points):
    local = []
    for p in points:
        lp = get_distance_NE(origin, [p['lat'], p['lng']])
        local += [lp]
    return np.array(local)

def read_log(path):
    log = []
    with open(path) as csv_file:
        csv_reader = csv.DictReader(csv_file)
        for row in csv_reader:
            log += [{
                'lat': float(row['Lat']),
                'lng': float(row['Lng'])
            }]
    return log

def read_sim(path):
    sim = []
    with open(path) as json_file:
        json_obj = json.load(json_file)
        for feature in json_obj["features"]:
            lng, lat = feature["geometry"]["coordinates"]
            sim += [{
                "lat": lat,
                "lng": lng
            }]
    return sim

def find_entry_idx(local_path):
    best_idx = 0
    best_diff = np.inf

    #center_point = [-456.699, 111.603]
    #Rc = 101.543
    Rc = 109.11
    center_point = [37.21042763, -36.647221830]

    for (i, p) in enumerate(local_path):
        dist = np.linalg.norm(p - center_point)
        diff = np.abs(dist - Rc)
        if diff < best_diff:
            best_diff = diff
            best_idx = i
    return best_idx

def find_idx(local_path, point):
    best_idx = 0
    best_diff = np.inf
    for (i, p) in enumerate(local_path):
        diff = np.linalg.norm(p - point)
        if diff < best_diff:
            best_diff = diff
            best_idx = i
    return best_idx

wind_dir = 118.5
#base_dir = 'route/plot_data/sim_eval/dir_{}'.format(wind_dir)
base_dir = 'route/plot_data/real_eval'
#origin = offset([57.6432, 11.8630], 400, 0)
origin = [57.485744, 11.931237]
end_ofs = 1938

def get_alt():
    alt = []
    ts = []
    with open('{}/log.csv'.format(base_dir)) as csv_file:
        csv_reader = csv.DictReader(csv_file)
        first = True
        for row in csv_reader:
            if first:
                init_ts = float(row["TimeUS"])
                first = False
            alt += [float(row['RelHomeAlt'])]
            ts += [(float(row["TimeUS"]) - init_ts)/1e6]
    return alt, ts

def get_wind(init_idx = 0):
    wind_spd = []
    wind_dir = []
    ts = []
    with open('{}/log.csv'.format(base_dir)) as csv_file:
        csv_reader = csv.DictReader(csv_file)
        first = True
        for (i, row) in enumerate(csv_reader):
            if i >= init_idx:
                if first:
                    init_ts = float(row["TimeUS"])
                    first = False
                wind_spd += [float(row['WindSpd'])]
                wind_dir += [float(row['WindDir'])]
                ts += [(float(row["TimeUS"]) - init_ts)/1e6]
    return wind_spd, wind_dir, ts

def get_landed_idx():
    best_ts = 0
    with open('{}/stat.csv'.format(base_dir)) as csv_file:
        csv_reader = csv.DictReader(csv_file)
        for row in csv_reader:
            if int(row["isFlying"]) > 0:
                best_ts = int(row["TimeUS"])
    best_idx = 0
    best_diff = np.inf
    with open('{}/log.csv'.format(base_dir)) as csv_file:
        csv_reader = csv.DictReader(csv_file)
        for (i, row) in enumerate(csv_reader):
            diff = np.abs(int(row["TimeUS"]) - best_ts)
            if diff < best_diff:
                best_diff = diff
                best_idx = i
    return best_idx

def alt_profile_main(ax, land_ofs, end_ofs):
    rc('text', usetex=True)
    log = read_log('{}/log.csv'.format(base_dir))
    alt, ts = get_alt()

    local_path = local_points(origin, log)
    entry_idx = find_entry_idx(local_path)
    print("err_h: ", alt[entry_idx])

    ts_0 = []
    for t in ts[land_ofs:end_ofs]:
        ts_0 += [t - ts[land_ofs]]


    s_alt = [10]*(entry_idx) + [0]*(end_ofs-entry_idx)
    print(len(alt))
    print(len(s_alt))
    ax.plot(ts_0, alt[land_ofs:end_ofs], label="$h$")
    ax.plot(ts_0, s_alt[land_ofs:end_ofs], label="$h_{safe}$")

    ax.set_xlabel("$t$ [s]")
    ax.set_ylabel("$h$ [m]")
    ax.legend()

def sim_main(ax, local_wps, local_path, local_path_sim, start_ofs, end_ofs):
    rc('text', usetex=True)
    x, y, w, h, r = read_landing(base_dir)
    obst = read_obstacles(base_dir)

    for i, o in enumerate(obst):
        if i==0:
            ax.add_patch(Polygon(o, alpha=0.5, facecolor='red', label='$\mathcal{X}_{obst}$'))
        else:
            ax.add_patch(Polygon(o, alpha=0.5, facecolor='red'))
    ax.add_patch(Rectangle((y,x), w, h, r, alpha=0.5, color='green', label='$\mathcal{A}$'))   

    ax.scatter(local_wps[:, 1], local_wps[:, 0], label="reference", color="red")
    ax.plot(local_wps[:, 1], local_wps[:, 0], color="red", linestyle=':')
    ax.scatter(local_wps[-2, 1], local_wps[-2, 0], color="black", marker="x", label="$\mathbf{p}_a$", zorder=100)
    ax.scatter(local_wps[-1, 1], local_wps[-1, 0], color="black", marker="*", label="$\mathbf{p}_l$", zorder=100)
    ax.scatter(local_wps[0, 1], local_wps[0, 0], color="black", marker="+", label="$\mathbf{p}_{loiter}$", zorder=100)

    ax.plot(local_path[start_ofs:end_ofs, 1], local_path[start_ofs:end_ofs, 0], label="trajectory")
    ax.plot(local_path_sim[:,1], local_path_sim[:, 0], linestyle=":", label="simulated")
    print("err_land: ", np.linalg.norm(local_wps[-1] - local_path[end_ofs]))
    plot_wind(ax, wind_dir, -100, 150)

    ax.set_xlabel('$y_E$ [m]')
    ax.set_ylabel('$x_N$ [m]')
    ax.axis('equal')
    ax.legend()

def wind_main():
    rc('text', usetex=True)
    wind_spd, wind_dir, ts = get_wind(880)
    _, ax = plt.subplots(1, 2, figsize=(12,5))

    ax[0].plot(ts, wind_spd)
    ax[0].set_ylabel('$W$ [m/s]')
    ax[0].set_xlabel('$t$ [s]')

    ax[1].plot(ts, wind_dir)
    ax[1].set_ylabel('$\psi_w$ [deg]')
    ax[1].set_xlabel('$t$ [s]')

    plt.show()

    

def main():
    rc('text', usetex=True)
    wps = get_solution_wps('{}/sol.txt'.format(base_dir))
    log = read_log('{}/log.csv'.format(base_dir))
    sim = read_sim('route/simulated_mission.txt')
    local_wps = local_points(origin, wps)
    local_path = local_points(origin, log)
    local_path_sim = local_points(origin, sim)

    start_ofs = 1300# find_idx(local_path, local_wps[0])
    land_ofs = find_idx(local_path, local_wps[-2])

    _, ax = plt.subplots(1, 2, figsize=(12,5))
    sim_main(ax[0], local_wps, local_path, local_path_sim, start_ofs, end_ofs)
    
    alt_profile_main(ax[1], land_ofs, end_ofs)
    plt.show()

if __name__ == '__main__':
    main()
    