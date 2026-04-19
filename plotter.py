import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from simulate import generate_simulation_data
from hssp import HSSPFilter

def main():
    print("Generating simulation data...")
    sim_data = generate_simulation_data(duration=10.0, dt=0.05)
    time_arr = sim_data["time"]
    gyro = sim_data["gyro"]
    accel = sim_data["accel"]
    histograms = sim_data["lidar_histograms"]
    n_frames = len(time_arr)
    
    # Process data through HSSP filter
    print("Processing data through HSSP algorithms...")
    filter_obj = HSSPFilter()
    
    gating_flags = []
    raw_grids = []
    filtered_grids = []
    metrics_list = []
    
    for i in range(n_frames):
        gated = not filter_obj.imu_gate(gyro[i], accel[i])
        gating_flags.append(gated)
        if gated:
            raw, h_grid, cond_rej = filter_obj.process_lidar_frame(histograms[i])
            # If gated, hold previous good values? We'll simulate LiDAR holding last valid state
            if len(filtered_grids) > 0:
                raw_grids.append(raw_grids[-1])
                filtered_grids.append(filtered_grids[-1])
                metrics_list.append(metrics_list[-1])
            else:
                raw_grids.append(np.zeros((8,8)))
                filtered_grids.append(np.zeros((8,8)))
                metrics_list.append((0, 0, 0, 0, False))
        else:
            raw, h_grid, cond_rej = filter_obj.process_lidar_frame(histograms[i])
            raw_grids.append(raw)
            filtered_grids.append(h_grid)
            mean_d, vol, snr, tilt_pct = filter_obj.calculate_metrics(h_grid)
            metrics_list.append((mean_d, vol, snr, tilt_pct, cond_rej))
            
    print("Setting up visualization...")
    fig = plt.figure(figsize=(14, 8))
    fig.canvas.manager.set_window_title("HSSP Real-Time Dashboard")
    
    # Grid layout: 2 rows, 2 columns
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 1], width_ratios=[1, 1])
    
    ax_motion = fig.add_subplot(gs[0, 0])
    ax_lidar = fig.add_subplot(gs[0, 1])
    ax_cross = fig.add_subplot(gs[1, 0])
    ax_metrics = fig.add_subplot(gs[1, 1])
    ax_metrics.axis('off')
    
    plt.tight_layout(pad=4.0)
    
    # Panel A: Motion
    ax_motion.set_title("Panel A: IMU Motion & Gating")
    ax_motion.set_xlim(0, 10.0)
    ax_motion.set_ylim(-150, 150)
    ax_motion.set_ylabel("Gyro (deg/s) / Accel (g)")
    ax_motion.set_xlabel("Time (s)")
    line_gyro, = ax_motion.plot([], [], label='Gyro X', color='blue')
    line_accel, = ax_motion.plot([], [], label='Accel Z (x100)', color='orange')
    gate_fill = None
    ax_motion.legend(loc="upper right")
    
    # Panel B: LiDAR Point Cloud (heatmap)
    ax_lidar.set_title("Panel B: LiDAR 8x8 Surface (Filtered depth mm)")
    im_lidar = ax_lidar.imshow(np.zeros((8,8)), vmin=30, vmax=120, cmap='viridis')
    plt.colorbar(im_lidar, ax=ax_lidar)
    
    # Panel C: Metrics
    metrics_text = ax_metrics.text(0.1, 0.8, "", fontsize=14, family='monospace', va='top')
    
    # Panel D: Cross section animation
    ax_cross.set_title("Panel D: Live Water Level & Projected Volume")
    ax_cross.set_xlim(0, 10)
    ax_cross.set_ylim(150, 0) # Flipped so 0 is top
    ax_cross.set_ylabel("Depth (mm)")
    ax_cross.set_xticks([])
    
    # Define bottle walls
    ax_cross.plot([2, 8], [150, 150], color='black', lw=3)
    ax_cross.plot([2, 2], [0, 150], color='black', lw=3)
    ax_cross.plot([8, 8], [0, 150], color='black', lw=3)
    
    # SLOSH LINE
    water_line, = ax_cross.plot([], [], lw=2, color='cyan', label='Live Surface')
    water_fill = ax_cross.fill_between([], [], 150, color='cyan', alpha=0.3)
    # VOLUME PROJECTION
    vol_line = ax_cross.axhline(y=0, color='red', linestyle='--', linewidth=2, label='Projected Volume')
    ax_cross.legend(loc="lower left")

    def init():
        line_gyro.set_data([], [])
        line_accel.set_data([], [])
        im_lidar.set_array(np.zeros((8,8)))
        metrics_text.set_text("")
        water_line.set_data([], [])
        return line_gyro, line_accel, im_lidar, metrics_text, water_line

    def animate(frame):
        # Update Motion
        t_slice = time_arr[:frame+1]
        line_gyro.set_data(t_slice, gyro[:frame+1, 0])
        line_accel.set_data(t_slice, accel[:frame+1, 2]*100 - 100) # Offset for visibility
        
        # Shade gated regions
        [p.remove() for p in reversed(ax_motion.collections)]
        # Re-add fill_between
        gate_arr = np.array(gating_flags[:frame+1]) * 150
        ax_motion.fill_between(t_slice, -150, 150, where=gate_arr>0, color='red', alpha=0.2)
        
        # Update LiDAR
        f_grid = filtered_grids[frame]
        im_lidar.set_array(f_grid)
        
        # Update Metrics
        mean_d, vol, snr, tilt_pct, cond_rej = metrics_list[frame]
        raw_val = float(np.max(raw_grids[frame]))
        
        text_str = (
            "Real-Time Metrics:\n\n"
            f"State:       {'[PAUSED - SLOSH]' if gating_flags[frame] else '[STABLE - SCANNING]'}\n"
            f"Raw Max (mm): {raw_val:.1f}\n"
            f"Flt Depth(mm):{mean_d:.1f}\n"
            f"Volume (ml):  {vol:.1f}\n"
            f"Current SNR:  {snr:.1f}\n"
            f"Tilt Correct: {tilt_pct:.1f}%\n"
            f"Cond Rejected:{'YES' if cond_rej else 'NO'}\n"
        )
        metrics_text.set_text(text_str)
        
        # Update Cross Section
        # Extract middle row for side-profile
        middle_row = f_grid[3, :]
        x_vals = np.linspace(2.0, 8.0, 8)
        val_cleaned = np.nan_to_num(middle_row, nan=mean_d)
        water_line.set_data(x_vals, val_cleaned)
        
        # Update shading
        [p.remove() for p in reversed(ax_cross.collections)]
        ax_cross.fill_between(x_vals, val_cleaned, 150, color='cyan', alpha=0.3)
        
        # Update Vol Projection
        vol_line.set_ydata([mean_d, mean_d])

        return line_gyro, line_accel, im_lidar, metrics_text, water_line

    print("Animating...")
    ani = animation.FuncAnimation(fig, animate, init_func=init,
                                  frames=n_frames, interval=50, blit=False)
    
    out_file = "hssp_demo.gif"
    print(f"Saving to {out_file}...")
    ani.save(out_file, writer='pillow', fps=20)
    print("Done!")

if __name__ == "__main__":
    main()
