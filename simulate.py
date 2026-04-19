import numpy as np

def generate_simulation_data(duration=10.0, dt=0.05):
    """
    Generate synthetic data for HSSP simulation.
    duration: seconds
    dt: sample time
    """
    np.random.seed(42)
    n_steps = int(duration / dt)
    time_arr = np.linspace(0, duration, n_steps)
    
    # 1. IMU Data
    # 0-3s: stable, 3-6s: sloshing/shaking, 6-10s: stable tilting
    gyro_x = np.zeros(n_steps)
    gyro_y = np.zeros(n_steps)
    gyro_z = np.zeros(n_steps)
    accel_x = np.zeros(n_steps)
    accel_y = np.zeros(n_steps)
    accel_z = np.ones(n_steps) # 1g normally
    
    true_water_level = np.ones(n_steps) * 60.0 # true volume height in mm
    
    lidar_histograms = [] # n_steps x 8 x 8 list of peaks (distance, intensity)
    
    for i, t in enumerate(time_arr):
        state = "stable"
        if 3.0 <= t < 6.0:
            state = "sloshing"
            gyro_x[i] = np.sin(t * 10) * 100 + np.random.normal(0, 5) # high gyro
            accel_z[i] = 1.0 + np.cos(t * 15) * 0.5
        elif 6.0 <= t <= 10.0:
            state = "tilting" # stable but tilted
            tilt_angle = 20.0 * (np.pi / 180.0) # 20 degrees
            accel_y[i] = np.sin(tilt_angle)
            accel_z[i] = np.cos(tilt_angle)
            
        # Add basic sensor noise
        gyro_x[i] += np.random.normal(0, 1)
        gyro_y[i] += np.random.normal(0, 1)
        gyro_z[i] += np.random.normal(0, 1)
        
        # 2. LiDAR Data (8x8 array)
        frame = np.zeros((8, 8, 3, 2)) # 3 possible peaks (dist, int)
        for row in range(8):
            for col in range(8):
                # Peak 0: condensation (always there, near field)
                frame[row, col, 0, 0] = np.random.uniform(1.0, 4.0) # 1-4mm
                frame[row, col, 0, 1] = 50 + np.random.normal(0, 5) # medium intensity
                
                # Peak 1: Water surface
                base_dist = true_water_level[i]
                
                if state == "sloshing":
                    dist = base_dist + np.random.normal(0, 15) # noisy sloshing
                    intensity = 30 + np.random.normal(0, 5) # lower intensity due to scattering
                elif state == "tilting":
                    # Gradient across the col
                    dist = base_dist + (col - 3.5) * 10
                    intensity = 80 + np.random.normal(0, 5)
                else:
                    dist = base_dist + np.random.normal(0, 1)
                    intensity = 90 + np.random.normal(0, 5)
                    
                frame[row, col, 1, 0] = dist
                frame[row, col, 1, 1] = intensity
                
                # Peak 2: Metal reflection (deep)
                frame[row, col, 2, 0] = 120.0 + np.random.normal(0, 1)
                frame[row, col, 2, 1] = 10 # low intensity since water blocks it mostly
                
        lidar_histograms.append(frame)
        
    return {
        "time": time_arr,
        "gyro": np.stack([gyro_x, gyro_y, gyro_z], axis=-1),
        "accel": np.stack([accel_x, accel_y, accel_z], axis=-1),
        "lidar_histograms": lidar_histograms, # list of 8x8x3x2
        "true_water_level": true_water_level
    }
