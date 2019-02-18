from pid import PID
from lowpass import LowPassFilter
import rospy
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

LAT_MIN_NUM=-10.0
LAT_MAX_NUM=-LAT_MIN_NUM
lat_control_steer_angle_limit = 2.0
#### parameters for lat pid
# ok 1.0/0.0/0.1 und 2.0 limit
# ruhig 2.0/0.0/0.5 und 2.0 limit
# gut, etwas unruhig 10.0/0.0/0.5 und 2.0 limit
# schlecht, nervoes 20.0/0.0/0.5 und 2.0 limit
# favorit: gut 5.0/0.0/0.1
# gut 5.0/0.0/0.2
lat_kp=5.0
lat_ki=0.0
lat_kd=0.5

##### long
SPEED_MIN_NUM=0.0
SPEED_MAX_NUM=1.0
speed_kp=0.2
speed_ki=0.05
speed_kd=0.1

 

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.last_steer = 0.0
        self.last_time = rospy.get_time()
        self.time= 0.0
        self.max_steer_angle = kwargs.get('max_steer_angle')
        self.pid_lateral=PID(lat_kp, lat_ki, lat_kd, mn=LAT_MIN_NUM, mx=LAT_MAX_NUM)
        self.pid_speed=PID(speed_kp, speed_ki, speed_kd, mn=SPEED_MIN_NUM, mx=SPEED_MAX_NUM)
        self.debug_last_sample_time=0.0
        #self.lowpass_lateral=LowPassFilter(tau, ts)
        pass
    
    def control(self, *args, **kwargs):
        target_speed = kwargs.get('target_speed')
        target_yaw_rate=kwargs.get('target_yaw_rate')
        current_speed=kwargs.get('current_speed')
        current_yaw_rate=kwargs.get('current_yaw_rate')
        dbw_status=kwargs.get('dbw_status')
        rospy.loginfo("target: %s , % s , current: %s, %s , status: %s",target_speed,  target_yaw_rate, current_speed, current_yaw_rate, dbw_status )
        self.time = rospy.get_time()
        sample_time=self.time - self.last_time
        self.last_time = self.time
        
        throttle = 0.0
        brake = 0.0
        steer = 0.0
        if dbw_status:
            # handle time jitter
            delta_sample_time=self.debug_last_sample_time- sample_time
            self.debug_last_sample_time = sample_time
            if delta_sample_time > (0.02 *1.0): # delta_sample time more than 100%
                rospy.logwarn("big time jitter (more than 0.2sec) detected with delta_sample_time: %s",delta_sample_time)
            time_normalization=(sample_time/0.2) # handle time jitter
            ######################################################
            # longitudinal control
            brake_scaler = 40 
            if not current_speed:
                current_speed = 0.0
            if not target_speed:
                target_speed = 0.0  
            error_speed = current_speed - target_speed

            if error_speed < 0: # too slow
                #rospy.loginfo("current_speed < target_speed: %s %s",current_speed, target_speed)
                throttle = self.pid_speed.step(-error_speed, sample_time)
            else: # too fast
                brake = error_speed*brake_scaler    
            # ensure standstill
            if target_speed < 1/3.6:
                brake=100.0

            ######################################################
            # lateral control
            if not current_yaw_rate:
                current_yaw_rate =0.0
            if not target_yaw_rate:
                target_yaw_rate =0.0  
            error_yaw_rate=current_yaw_rate - target_yaw_rate
            #error_yaw_rate = self.lowpass_lateral.filt(error_yaw_rate)
            rospy.loginfo("error_yaw_rate: %s", error_yaw_rate)
            delta_steer = self.pid_lateral.step(-error_yaw_rate, sample_time)* time_normalization
            rospy.loginfo("delta_steer: %s", delta_steer)
            if abs(self.last_steer + delta_steer) < lat_control_steer_angle_limit:
                steer = self.last_steer + delta_steer
            else:
                steer = self.last_steer 
                rospy.loginfo("steer angle limit reached!: %s", lat_control_steer_angle_limit)
            #rospy.loginfo("steer angle limit set to: %s", self.max_steer_angle)
            rospy.loginfo("sample_time: %s", sample_time)
            self.last_steer = steer
        else:
            self.pid_lateral.reset()
            self.pid_speed.reset()
            
        return throttle, brake, steer

