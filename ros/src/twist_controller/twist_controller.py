from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

LAT_MIN_NUM=-0.5#-10.0
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
lat_ki=0.05#0.0
lat_kd=1#0.5

##### long
speed_kp=0.5#0.2
speed_ki=0.02#0.05
speed_kd=0.2#0.1


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.last_steer = 0.0
        self.last_time = rospy.get_time()
        self.time= 0.0
        self.vehicle_mass = kwargs.get("vehicle_mass")
        self.fuel_capacity = kwargs.get("fuel_capacity")
        self.brake_deadband = kwargs.get("brake_deadband")
        self.wheel_radius = kwargs.get("wheel_radius")
        self.max_steer_angle = kwargs.get('max_steer_angle')
        self.accel_limit = kwargs.get('accel_limit')
        self.decel_limit = kwargs.get('decel_limit')
        self.max_lat_accel = kwargs.get("max_lat_accel")
        self.wheel_base = kwargs.get("wheel_base")
        self.steer_ratio = kwargs.get("steer_ratio")
        self.min_speed = 0.0
        self.max_steer_angle = kwargs.get('max_steer_angle')
        self.pid_lateral=PID(lat_kp, lat_ki, lat_kd, mn=LAT_MIN_NUM, mx=LAT_MAX_NUM)
        self.pid_speed=PID(speed_kp, speed_ki, speed_kd, mn=self.decel_limit, mx=self.accel_limit)
        self.debug_last_sample_time=0.0
        tau = 0.2
        ts = 0.1
        self.lowpass_lateral=LowPassFilter(tau, ts)
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, self.min_speed,
                                            self.max_lat_accel, self.max_steer_angle)
    
    def control(self, *args, **kwargs):
        target_speed = kwargs.get('target_speed')
        target_yaw_rate = kwargs.get('target_yaw_rate')
        current_speed = kwargs.get('current_speed')
        current_yaw_rate = kwargs.get('current_yaw_rate')
        dbw_status = kwargs.get('dbw_status')
        self.time = rospy.get_time()
        sample_time = self.time - self.last_time
        self.last_time = self.time
        
        throttle = 0.0
        brake = 0.0
        steer = 0.0
        if dbw_status:
            ######################################################
            # longitudinal control
            if not current_speed:
                current_speed = 0.0
            if not target_speed:
                target_speed = 0.0
            error_speed = target_speed - current_speed

            vel = self.pid_speed.step(error_speed, sample_time)
            vel = self.lowpass_lateral.filt(vel)

            if target_speed < 0.001 and current_speed < 0.447:
                brake = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius * abs(
                    self.decel_limit)
                throttle = 0
            elif vel > 0:
                throttle = vel
                brake = 0
            else:
                brake = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius * abs(vel)
                throttle = 0

            ######################################################
            # lateral control
            if not target_yaw_rate:
                target_yaw_rate = 0.0
            error_yaw_rate = target_yaw_rate
            steer = self.pid_lateral.step(error_yaw_rate, sample_time)

        else:
            self.pid_lateral.reset()
            self.pid_speed.reset()
            
        return throttle, brake, steer
