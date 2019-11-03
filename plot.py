import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.patches import Polygon, Rectangle
from matplotlib import rc
import json

from lib.control import get_distance_NE

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
    main()
    