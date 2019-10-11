import matplotlib.pyplot as plt

def plot_trajectory(df, bound=0.001, ax=None):
    if ax is None:
        _, ax = plt.subplots()
    ax.plot(df.Lng, df.Lat)
    ax.scatter(df.Lng_c, df.Lat_c)
    ax.scatter(df.Lng_n, df.Lat_n)
    ax.set_xlim([df.Lng.min() - bound, df.Lng.max() + bound])
    ax.set_ylim([df.Lat.min() - bound, df.Lat.max() + bound])
    plt.show()