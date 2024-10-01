import numpy as np

# Function to read steering and throttle data from the file
def read_data(file_name):
    try:
        with open(file_name, 'r') as file:
            data = file.readlines()
        
        steering_data = []
        steering_errors = []
        throttle_data = []
        throttle_errors = []
        
        for line in data:
            # Assuming the file is formatted as: steering, steering_error, throttle, throttle_error
            steering, steering_error, throttle, throttle_error = map(float, line.strip().split(","))
            steering_data.append(steering)
            steering_errors.append(steering_error)
            throttle_data.append(throttle)
            throttle_errors.append(throttle_error)
        
        return np.array(steering_data), np.array(steering_errors), np.array(throttle_data), np.array(throttle_errors)
    
    except FileNotFoundError:
        print(f"File '{file_name}' not found.")
        return None

# Function to run the Ziegler–Nichols method for tuning
def ziegler_nichols_tuning(steering_errors, throttle_errors, dt, Ku_steering, Ku_throttle, Pu_steering, Pu_throttle):
    # Ziegler–Nichols tuning formulas for PID
    def get_pid_params(Ku, Pu):
        Kp = 0.6 * Ku
        Ti = Pu / 2  
        Td = Pu / 8  
        Ki = Kp / Ti
        Kd = Kp * Td
        return Kp, Ki, Kd
    
    # Tuning for steering
    Kp_steering, Ki_steering, Kd_steering = get_pid_params(Ku_steering, Pu_steering)
    
    # Tuning for throttle
    Kp_throttle, Ki_throttle, Kd_throttle = get_pid_params(Ku_throttle, Pu_throttle)
    
    return {
        "steering": {"Kp": Kp_steering, "Ki": Ki_steering, "Kd": Kd_steering},
        "throttle": {"Kp": Kp_throttle, "Ki": Ki_throttle, "Kd": Kd_throttle}
    }
  
def twiddle(p, dp, min_dp_threshold, sum_dp_threshold, throttle_op_mode, steer_op_mode):
    """
    Optimize a set of parameters using the twiddle algorithm
    
    p: Parameters to optimize
    dp: Initial magnitude of change to try when optimizing parameters (one for each parameter in p)
    min_dp_threshold: Smallest dp value for which to execute a simulation
    sum_dp_threshold: Minimum sum of values in dp when for starting a new iteration of the optimization
    throttle_op_mode: Throttle operation mode of the simulation. "constant_speed" for trying to maintain a constant speed 
                        without the controller, "constant_speed_reference" for asking the controller to maintain a constant speed or "normal"
                        for passing the output of the behavior planner and asking the controller to maintain that.
    steer_op_mode: Steer operation mode of the simulation. "straight" for fixed 0 steer output or "normal" for using the steer controller.
    """
    assert(len(p) == len(dp))
    dp_increase_ratio = 1.5
    dp_decrease_ratio = 0.5
    
    num_of_evaluations = 1
    # best_error = run_with_params(p, throttle_op_mode, steer_op_mode)
    print(f"{best_error:.03f} Initial error, keeping params")
    
    while sum(dp) > sum_dp_threshold:
        for i in range(len(p)):
            # Skip execution if dp for this particular parameter is already too small (others might still be ok)
            if dp[i] <= min_dp_threshold:
                continue
            
            original_p = p[i]

            p[i] += dp[i]
            # error = run_with_params(p, throttle_op_mode, steer_op_mode)
            num_of_evaluations += 1
            
            if error < best_error:
                best_error = error
                dp[i] *= dp_increase_ratio
                print(f"{error:.03f} Lower error, keeping new params")
            else:
                print(f"{error:.03f} Higher error, dropping params")
                p[i] = original_p
                p[i] -= dp[i]
                error = 0.1
                # error = run_with_params(p, throttle_op_mode, steer_op_mode)
                num_of_evaluations += 1

                if error < best_error:
                    best_error = error
                    dp[i] *= dp_increase_ratio
                    print(f"{error:.03f} Lower error, keeping new params")
                else:
                    p[i] = original_p
                    dp[i] *= dp_decrease_ratio
                    print(f"{error:.03f} Higher error, dropping params")
    return p, best_error, num_of_evaluations
        

# Load data
steering_data, steering_errors, throttle_data, throttle_errors = read_data("throttle_data.txt") # set this file name for steering and throttle

# If data is loaded successfully, run Ziegler-Nichols tuning
if steering_data is not None:
    # Time step (sample period)
    dt = 0.06  

    #assumend Ku and Pu variables
    Ku_steering = 0.12  
    Pu_steering = 0.06 
    Ku_throttle = 0.12  
    Pu_throttle = 0.06  

    # Run Ziegler-Nichols tuning
    tuning_params = ziegler_nichols_tuning(steering_errors, throttle_errors, dt, Ku_steering, Ku_throttle, Pu_steering, Pu_throttle)

    # Display tuned parameters
    print("Tuned PID parameters:")
    print("Steering:", tuning_params['steering'])
    print("Throttle:", tuning_params['throttle'])

