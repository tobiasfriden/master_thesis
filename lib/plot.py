import matplotlib.pyplot as plt
import numpy as np
from lib.simulation import get_distance_NE

def plot_trajectory(df, bound=0.001, ax=None):
    if ax is None:
        _, ax = plt.subplots()
    ax.plot(df.Lng, df.Lat)
    ax.scatter(df.Lng_c, df.Lat_c)
    ax.scatter(df.Lng_n, df.Lat_n)
    ax.set_xlim([df.Lng.min() - bound, df.Lng.max() + bound])
    ax.set_ylim([df.Lat.min() - bound, df.Lat.max() + bound])
    plt.show()

def plot_primitive(prim, origin=[0,0], ax=None, label='', color=None):
    if ax is None:
        _, ax = plt.subplots()
    local_path = np.zeros((prim.shape[0], 2))
    for i,row in prim.iterrows():
        local_path[i, :] = get_distance_NE([0, 0], [row.Lat_p, row.Lng_p])
    ax.plot(local_path[:, 1], local_path[:, 0], label=label, color=color)