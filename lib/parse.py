import pandas as pd
import numpy as np
from google.cloud import firestore as fs

def parse_flight_data(flight_data, col_names):
    cols = [flight_data['Timestamp'], flight_data['FlightTime']] + [flight_data[c].apply(pd.Series) for c in col_names]
    return pd.concat(cols, axis=1)

def get_wind(row):
    wind_dir = (np.degrees(np.arctan2(row.North, row.East) + 3*np.pi/2) % 360 + 360) % 360
    wind_spd = np.sqrt(row.East**2 + row.North**2)
    return pd.Series({'Wind_dir': wind_dir, 'Wind_spd': wind_spd})

def apply_concat(df, func):
    return pd.concat([df, df.apply(func, axis=1)], axis=1)

def get_missions(client, droneid):
    ref = client.collection('mission').where('droneID', '==', droneid).order_by('ts', 'DESCENDING').stream()
    missions = {}
    for doc in ref:
        m = doc.to_dict()
        r = {}
        if 'reached' in m:
            for k, v in m['reached'].items():
                r[v] = int(k)
        if 'waypoints' in m['mission']:
            missions[m['ts']] = {
                'wps': m['mission']['waypoints'],
                'reached': r
            }
            if 'home' in m['mission']:
                missions[m['ts']]['home_alt'] = m['mission']['home']['alt']
    return missions  

def get_current_wps(missions, parsed, timestamp, flighttime):
    for k in sorted(missions.keys()):
        if pd.Timestamp(k) > timestamp:
            break
        mission_time = k
    m = missions[mission_time]
    reached_time = None
    for k in sorted(m['reached'].keys()):
        if pd.Timestamp(k) > timestamp:
            break
        reached_time = k
    if reached_time is None:
        return None, None, np.nan, pd.Timestamp(reached_time)
    reached = m['reached'][reached_time]
    if reached == len(m['wps']) - 1:
        return None, None, np.nan, pd.Timestamp(reached_time)
    wps = m['wps'][reached:reached+2]
    if len(wps) != 2:
        return None, None, np.nan, pd.Timestamp(reached_time)
    home_alt = np.nan
    if 'home_alt' in m:
        start_alt = parsed[(parsed.FlightTime == flighttime) & (parsed.Alt > 0)].Alt.iloc[1]
        home_alt = np.min([start_alt, m['home_alt']])
    return (
        wps[0] if 'lat' in wps[0] and 'type' in wps[0] else None,
        wps[1] if 'lat' in wps[1] and 'type' in wps[1] else None,
        home_alt,
        pd.Timestamp(reached_time)
    )

def parse_wp_data(missions, parsed, row):
    curr_wp, next_wp, home_alt, ts = get_current_wps(missions, parsed, row.Timestamp, row.FlightTime)
    if curr_wp is not None and next_wp is not None:
        return pd.Series({
            'Lat_c': curr_wp['lat'],
            'Lng_c': curr_wp['lng'],
            'Lat_n': next_wp['lat'],
            'Lng_n': next_wp['lng'],
            'Home_alt': home_alt,
            'ReachedTime': ts,
            'Type_c': curr_wp['type'],
            'Type_n': next_wp['type']
        })
    else:
        return pd.Series({
            'Lat_c': np.nan,
            'Lng_c': np.nan,
            'Lat_n': np.nan,
            'Lng_n': np.nan,
            'Home_alt': home_alt,
            'ReachedTime': ts,
            'Type_c': '',
            'Type_n': ''
        })