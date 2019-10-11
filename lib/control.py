import numpy as np

def longitude_scale(Lat):
    scale = np.cos(np.radians(Lat))
    return np.max([scale, 0.01])

def get_distance_NE(pos1, pos2):
    scaling = 0.011131884502145034*1e7
    Lat1, Lng1 = pos1
    Lat2, Lng2 = pos2
    
    return np.array([
        (Lat2 - Lat1)*scaling,
        (Lng2 - Lng1)*scaling*longitude_scale(Lat1)
    ])

def passed_point(point, pos1, pos2):
    pos_diff = get_distance_NE(pos1, pos2)
    point_diff = get_distance_NE(pos1, point)
    dsquared = pos_diff[0]**2 + pos_diff[1]**2
    if dsquared < 1:
        dsquared = 1
    return np.dot(pos_diff, point_diff)/dsquared >= 1

def offset(pos, dNorth, dEast):
    scaling = 1/(0.011131884502145034*1e7)
    Lat, Lng = pos
    dLat = dNorth*scaling
    dLng = (dEast*scaling)/longitude_scale(Lat)
    return np.array([
        Lat + dLat,
        Lng + dLng
    ])

def bearing_to(pos1, pos2):
    Lat1, Lng1 = pos1
    Lat2, Lng2 = pos2
    
    off_x = Lng2 - Lng1
    off_y = (Lat2 - Lat1)/longitude_scale(Lat2)
    bearing = 90 + np.degrees(np.arctan2(-off_y, off_x))
    if bearing < 0:
        bearing += 360
    return bearing

def norm(vector):
    return np.linalg.norm(vector)

def cross(v1, v2):
    x1, y1 = v1
    x2, y2 = v2
    return x1*y2 - x2*y1

def constrain(val, _min, _max):
    if val < _min:
        return _min
    elif val > _max:
        return _max
    else:
        return val

def L1_controller(
    Lat, # Current position
    Lng,
    Lat_c, # Reached waypoint
    Lng_c,
    Lat_n, # Next waypoint
    Lng_n,
    V, # Airspeed, assumed larger than 0.1
    hdg, # Heading
    wind_spd,
    wind_dir
):
    # parameters
    L1_period = 16
    L1_damping = 0.75
    
    pos = np.array([
        Lat,
        Lng,
    ])
    wp_a = np.array([
        Lat_c,
        Lng_c
    ])
    wp_b = np.array([
        Lat_n,
        Lng_n
    ])
    groundspeed = V*np.array([
        np.cos(np.radians(hdg)),
        np.sin(np.radians(hdg))
    ])
    
    groundspeed += wind_spd*np.array([
        np.cos(np.radians(wind_dir)),
        np.sin(np.radians(wind_dir))
    ])
    
    K_L1 = 4*L1_damping*L1_damping
    
    L1_dist = np.max([
        1/np.pi*L1_damping*L1_period*V,
        0.1 # Check min dist parameter
    ])
    
    ab = get_distance_NE(wp_a, wp_b)
    if norm(ab) < 1e-6:
        ab = get_distance_NE(pos, wp_b)
    ab_length = norm(ab)
    ab_unit = ab/ab_length
    
    a_pos = get_distance_NE(wp_a, pos)
    xtrack_error = cross(a_pos, ab_unit)
    wp_a_dist = norm(a_pos)
    
    along_track_dist = np.dot(a_pos, ab_unit)
    # Fly towards wp A
    if wp_a_dist > L1_dist and \
        along_track_dist/np.max([wp_a_dist, 1]) < -0.7071:
        a_pos_unit = a_pos/norm(a_pos)
        xtrack_v = cross(groundspeed, -1*a_pos_unit)
        ltrack_v = np.dot(groundspeed, -1*a_pos_unit)
        Nu = np.arctan2(xtrack_v, ltrack_v)
    # Fly back to wp B
    elif along_track_dist > (ab_length + 3*V):
        b_pos = get_distance_NE(wp_b, pos)
        b_pos_unit = b_pos/norm(b_pos)
        xtrack_v = cross(groundspeed, -1*b_pos_unit)
        ltrack_v = np.dot(groundspeed, -1*b_pos_unit)
        Nu = np.arctan2(xtrack_v, ltrack_v)
    # Follow line between wp A and B
    else:
        xtrack_v = cross(groundspeed, ab_unit)
        ltrack_v = np.dot(groundspeed, ab_unit)
        Nu2 = np.arctan2(xtrack_v, ltrack_v)
        
        sine_Nu1 = xtrack_error/np.max([L1_dist, 0.1])
        sine_Nu1 = constrain(sine_Nu1, -0.7071, 0.7071)
        Nu1 = np.arcsin(sine_Nu1)
        
        Nu = Nu1 + Nu2
        
    Nu = constrain(Nu, -np.pi/2, np.pi/2)
    # Return lateral demanded acceleration
    return K_L1*V*V/L1_dist*np.sin(Nu), L1_dist
    
