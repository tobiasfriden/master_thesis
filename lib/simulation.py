import numpy as np
import pandas as pd

from lib.control import L1_controller, offset, get_distance_NE, passed_point

class Simulator:
    
    def __init__(self, init_row, opts, model, ss, mm, na):
        self.yaw = init_row.Yaw
        self.roll = np.zeros(na)
        self.roll[-1] = init_row.Roll
        self.l1_acc = np.zeros(na)
        self.l1_acc[-1] = init_row.L1_acc
        self.lat = init_row.Lat
        self.lng = init_row.Lng
        self.l1_dist = np.inf
        self.ts = init_row.Timestamp
        
        self.V = opts['V']
        self.dt = opts['dt']
        assert len(opts['wps']) >= 2, 'must input minimum 2 waypoints'
        self.wps = opts['wps']
        self.wp_R = opts['wp_R']
        self.next_wp()
        
        self.model = model
        self.ss = ss
        self.mm = mm
        
        if 'wind_spd' in opts:
            self.wind_spd = opts['wind_spd']
        else:
            self.wind_spd = 0
        if 'wind_dir' in opts:
            self.wind_dir = opts['wind_dir']
        else:
            self.wind_dir = 0
     
    # Forward wp counter, return True if reached
    def next_wp(self):
        # Last wp reached
        if len(self.wps) == 1:
            return True
        
        wp_c, self.wps = self.wps[0], self.wps[1:]
        wp_n = self.wps[0]
        self.lat_c = wp_c['lat']
        self.lng_c = wp_c['lng']
        self.lat_n = wp_n['lat']
        self.lng_n = wp_n['lng']
        return False
        
    
    # Save predicted values to use in next prediction
    def save(self, yaw, roll, lat, lng, l1_acc, l1_dist, ts):
        self.yaw = yaw
        self.lat = lat
        self.lng = lng
        self.roll[:-1] = self.roll[1:]
        self.roll[-1] = roll
        self.l1_acc[:-1] = self.l1_acc[1:]
        self.l1_acc[-1] = l1_acc
        self.l1_dist = l1_dist
        self.ts = ts
        
    # Predict rates with nn model
    # x = [yaw, roll]
    def predict(self):
        # Stack old values to create feature, transform with StandardScaler
        in_tf = self.ss.transform(np.hstack((self.l1_acc, self.roll)).reshape(1, -1))
        out = self.model.predict(in_tf)
        out_itr = self.mm.inverse_transform(out)
        yrate_r, rrate_r = out_itr[0]
        return np.array([
            np.degrees(yrate_r),
            np.degrees(rrate_r)
        ])
    
    def predict_simple(self, l1_acc, V):
        yawrate = l1_acc/V
        #0.38411117
        if np.abs(yawrate) > 0.3:
            yawrate = np.sign(yawrate)*0.3
        return np.array([
            np.degrees(yawrate),
            0
        ])
    
    def step(self, simple=False):
        dt = self.dt
        V = self.V
        l1_acc, l1_dist = L1_controller(
                    self.lat, # Current position
                    self.lng,
                    self.lat_c, # Reached waypoint
                    self.lng_c,
                    self.lat_n, # Next waypoint
                    self.lng_n,
                    V, # Airspeed, assumed larger than 0.1
                    self.yaw, # Heading,
                    self.wind_spd,
                    self.wind_dir
                )
        
        #Euler forward integration
        if simple:
            yrate, rrate = self.predict_simple(l1_acc, V)
        else:
            yrate, rrate = self.predict()
        Yaw_next = self.yaw + yrate*dt
        if Yaw_next < 0:
            Yaw_next += 360
        if Yaw_next >= 360:
            Yaw_next -= 360
        Roll_next = self.roll[-1] + rrate*dt

        V_ground = np.array([
            V*np.cos(np.radians(self.yaw))+ self.wind_spd*np.cos(np.radians(self.wind_dir)),
            V*np.sin(np.radians(self.yaw))+ self.wind_spd*np.sin(np.radians(self.wind_dir))
        ])
        
        Lat_next, Lng_next = offset(
            [self.lat, self.lng],
            (V_ground[0])*dt,
            (V_ground[1])*dt
        )
        
        ts_next = self.ts + pd.Timedelta(microseconds=dt*1e6)
        
        self.save(Yaw_next, Roll_next, Lat_next, Lng_next, l1_acc, l1_dist, ts_next)
        return {
            'Yaw_p': Yaw_next,
            'Roll_p': Roll_next,
            'YawRate_p': np.radians(yrate),
            'RollRate_p': np.radians(rrate),
            'Lat_p': Lat_next,
            'Lng_p': Lng_next,
            'L1_acc_p': l1_acc,
            'Groundspeed_p': np.linalg.norm(V_ground),
            'Timestamp': ts_next
        }
    
        
    
    def simulate_mission(self, simple=False):
        result = []
        
        while True:
            next_dist = np.linalg.norm(get_distance_NE([self.lat, self.lng], [self.lat_n, self.lng_n]))
            passed = passed_point([self.lat, self.lng], [self.lat_c, self.lng_c], [self.lat_n, self.lng_n])
            #if passed: print('passed')
            if  next_dist <= self.wp_R or passed:
                if self.next_wp():
                    break
            result.append(self.step(simple))
        return pd.DataFrame(result)
