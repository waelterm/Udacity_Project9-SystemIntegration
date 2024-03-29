
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        kp = 0.3
        ki = 0.1
        kd = 0.0
        mn = 0.0 # Minimum throttle value
        mx = 0.2 # Maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)
        tau = 0.5 # cutoff frequency 
        ts = 0.02 #Sample Time
        self.vel_lpf = LowPassFilter(tau, ts)
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.last_time = rospy.get_time()

    def control(self, linear_velocity, angular_velocity, dbw_enabled, current_linear_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if dbw_enabled:
            current_filtered_velocity = self.vel_lpf.filt(current_linear_velocity)
            steering = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_filtered_velocity)
            #print("Steering: {}".format(steering))
            velocity_error = linear_velocity - current_filtered_velocity
            self.last_velocity = current_filtered_velocity
            current_time = rospy.get_time()
            sample_time = current_time- self.last_time
            self.last_time = current_time
            throttle = self.throttle_controller.step(velocity_error, sample_time)
            brake = 0
            if linear_velocity == 0 and current_filtered_velocity <0.1:
                # Stationary vehicle (Hold strong brake)
                throttle = 0
                brake = 700
            elif throttle < 0.1 and velocity_error < 0:
                throttle = 0
                decel = max(velocity_error, self.decel_limit) # finding desired deceleration
                brake = abs(decel)*self.vehicle_mass * self.wheel_radius # Calculating Torque in N*m
        else:
            # If driver takes over, reset the integral term of the throttle controller
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0
        
        
        return throttle, brake, steering
