import numpy as np

class HSSPFilter:
    def __init__(self):
        self.gyro_threshold = 40.0 # deg/s
        self.tilt_threshold = 15.0 # deg
        self.metal_intensity_threshold = 100.0
        self.water_refractive_index = 1.33
        self.last_valid_surface = None

    def imu_gate(self, gyro, accel):
        """
        Check if we should pause LiDAR acquisition.
        gyro: [x, y, z] deg/s
        accel: [x, y, z] g
        return: True if stable enough to scan, False if gated (paused)
        """
        if np.max(np.abs(gyro)) > self.gyro_threshold:
            return False
            
        # Calculate tilt from accel_z
        # If accel_z is < cos(15 deg), tilt is > 15 deg
        norm = np.linalg.norm(accel)
        if norm > 0:
            tilt_rad = np.arccos(np.clip(accel[2] / norm, -1.0, 1.0))
            if np.degrees(tilt_rad) > self.tilt_threshold:
                return False
                
        return True

    def process_lidar_frame(self, frame_histograms):
        """
        frame_histograms: 8x8x3x2 array (zone, peak, [dist, intensity])
        Selects the true water peak, ignoring condensation (0-5mm).
        Applies refractive index correction.
        """
        water_surface_grid = np.zeros((8, 8))
        raw_grid = np.zeros((8, 8))
        condensation_rejected = False
        
        for row in range(8):
            for col in range(8):
                peaks = frame_histograms[row, col]
                # Sort peaks by distance
                peaks = peaks[peaks[:, 0].argsort()]
                
                # The "raw" sensor value normally just takes highest intensity or first return
                # We'll simulate finding the highest intensity for raw
                highest_idx = np.argmax(peaks[:, 1])
                raw_grid[row, col] = peaks[highest_idx, 0]
                
                # Filter logic:
                valid_peaks = []
                for p in peaks:
                    dist, intensity = p[0], p[1]
                    if dist <= 5.0:
                        condensation_rejected = True
                        continue # Multi-peak rejection of near-field
                    if intensity > 0:
                        valid_peaks.append(p)
                        
                if not valid_peaks:
                    water_surface_grid[row, col] = np.nan
                    continue
                    
                # Take highest remaining peak
                valid_peaks = np.array(valid_peaks)
                best_peak = valid_peaks[np.argmax(valid_peaks[:, 1])]
                
                dist_val = best_peak[0]
                intensity_val = best_peak[1]
                
                # Refractive correction
                if intensity_val > self.metal_intensity_threshold:
                    # Empty metal
                    water_surface_grid[row, col] = dist_val
                else:
                    # Water detected, correct speed of light delay
                    water_surface_grid[row, col] = dist_val / self.water_refractive_index
                    
        return raw_grid, water_surface_grid, condensation_rejected

    def calculate_metrics(self, water_surface_grid):
        """ Calculate mean depth, volume, and SNR """
        valid_depths = water_surface_grid[~np.isnan(water_surface_grid)]
        if len(valid_depths) == 0:
            return 0.0, 0.0, 0.0
            
        mean_depth = np.mean(valid_depths)
        # Approximate volume mapping
        volume = max(0, (150 - mean_depth) * 5) # mocked conversion to ml
        snr_mock = np.mean(valid_depths) / (np.std(valid_depths) + 0.1) * 10
        
        if np.std(valid_depths) < 2.0:
            tilt_corrected_pct = 100.0
        else:
            tilt_corrected_pct = 85.0
            
        return mean_depth, volume, snr_mock, tilt_corrected_pct
