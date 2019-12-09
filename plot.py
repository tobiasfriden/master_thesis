import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.patches import Polygon, Rectangle
from matplotlib import rc
import json

from lib.control import get_distance_NE
from lib.simulation import simulate_primitive
from lib.plot import plot_primitive

def get_solution_wps():
    with open('route/sol.txt') as f:
        raw = json.load(f)
        wps = []
        for f in raw['features']:
            coords = f['geometry']['coordinates']
            wps += [{'lat': coords[1], 'lng': coords[0]}]
    return wps

def read_obstacles():
    with open('route/obstacles.txt') as f:
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

def read_landing():
    with open('route/landing.txt') as f:
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
            grid[i, :] = np.array([float(v) if float(v) > 0 else np.nan for v in values ])
        return grid

def grid_main():
    rc('text', usetex=True)
    grid = read_grid()
    step = 5
    x_ticks = step*np.arange(1, grid.shape[1]+1, 1)
    y_ticks = step*np.arange(-grid.shape[1], grid.shape[1]+1, 1)

    X, Y = np.meshgrid(x_ticks, y_ticks)
    _, ax = plt.subplots()
    ax.contourf(X, Y, grid, 50)

    sim_df = simulate_primitive(70, 180, wind_spd=5, wind_dir=0)
    plot_primitive(sim_df, ax=ax, label='trajectory')
    ax.scatter(180, 70, color='red', label='$u$')

    wind_dir = np.radians(0)
    ax.quiver(100, -100, np.sin(wind_dir), np.cos(wind_dir), scale=10, label='$\mathbf{w}$')

    ax.set_xlim([-10, 500])
    ax.set_ylim([-250, 250])
    ax.set_xlabel('$y_E$ [m]')
    ax.set_ylabel('$x_N$ [m]')
    
    plt.legend()
    plt.show()

def prim_diff_main():
    sim_1 = simulate_primitive(-180, 0, wind_dir=90, wind_spd=5, init_yaw=5)
    sim_2 = simulate_primitive(-180, 0, wind_dir=90, wind_spd=5, init_yaw=-5)
    _, ax = plt.subplots()
    plot_primitive(sim_1, ax=ax, label='$\psi_i=5\degree$')
    plot_primitive(sim_2, ax=ax, label='$\psi_i=-5\degree$')
    wind_rad = np.pi/2
    ax.quiver(100,-150, np.sin(wind_rad), np.cos(wind_rad), scale=10, label='wind')
    ax.set_xlabel('$y_E$ [m]')
    ax.set_ylabel('$x_N$ [m]')

    plt.legend()
    plt.show()

def main():
    rc('text', usetex=True)
    wp_df = pd.read_csv('results.csv')
    land = wp_df[wp_df.FlightTime == wp_df.FlightTime.unique()[-4]].reset_index(drop=True).copy()
    wps = get_solution_wps()
    obst = read_obstacles()
    x, y, w, h, r = read_landing()

    origin = [wps[0]['lat'], wps[0]['lng']]
    local_path = np.zeros((land.shape[0], 2))
    for i,row in land.iterrows():
        local_path[i, :] = get_distance_NE(origin, [row.Lat, row.Lng])
    local_wps = np.zeros((len(wps), 2))
    for i, wp in enumerate(wps):
        local_wps[i, :] = get_distance_NE(origin, [wp['lat'], wp['lng']])

    fig, ax = plt.subplots()
    for i, o in enumerate(obst):
        if i==0:
            ax.add_patch(Polygon(o, alpha=0.5, facecolor='red', label='$\mathcal{X}_{obst}$'))
        else:
            ax.add_patch(Polygon(o, alpha=0.5, facecolor='red'))
    ax.add_patch(Rectangle((y,x), w, h, r, alpha=0.5, color='green', label='$\mathcal{A}$'))

    ax.scatter(local_wps[:, 1], local_wps[:, 0], color='red')
    ax.plot(local_wps[:, 1], local_wps[:, 0], color='red', label='reference')
    ax.plot(local_path[:, 1], local_path[:, 0], label='trajectory')
    ax.axis('equal')
    wind_dir = np.radians(90)
    ax.quiver(-400, -300, np.sin(wind_dir), np.cos(wind_dir), scale=10, label='$\mathbf{w}$')

    ax.set_xlabel('$p_E$')
    ax.set_ylabel('$p_N$')
    plt.legend()
    plt.show()



if __name__ == '__main__':
    prim_diff_main()
    