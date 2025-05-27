import os
import serial
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
from collections import deque

# --- Configuration ---
SERIAL_PORT = 'COM3'  # <<< UPDATE THIS to your Arduino's serial port
SERIAL_BAUD = 115200
MAX_SENSOR_RANGE_MM = 6000 # 6-meter sensor range
PLOT_UPDATE_INTERVAL_MS = 100
FILTER_WINDOW_SIZE = 5      # Median filter window (odd number recommended)
SAVE_PLOT_EVERY_N_FRAMES = 20 # Frequency to save plot images

PLOT_SAVE_DIR = 'plots'
os.makedirs(PLOT_SAVE_DIR, exist_ok=True)

# --- Globals for Plotting & Data ---
fig, ax = plt.subplots()
scatter = ax.scatter([], [], c='red', s=10) # Plot points as red, size 10
obstacle_points = [] # Stores filtered (x,y) coordinates for plotting
raw_points_buffer = deque(maxlen=FILTER_WINDOW_SIZE) # Buffer for median filter
plot_save_frame_counter = 0

def setup_plot():
    ax.set_xlabel("X-coordinate (meters)")
    ax.set_ylabel("Y-coordinate (meters)")
    ax.set_title(f"2D Sonar Map (Range: {MAX_SENSOR_RANGE_MM/1000:.1f}m, Filter: {FILTER_WINDOW_SIZE}pts)")
    ax.grid(True)
    ax.set_aspect('equal', adjustable='box')
    plot_range_m = MAX_SENSOR_RANGE_MM / 1000.0
    # X-axis can go negative/positive, Y mainly positive if sweeping 0-120 deg forward
    ax.set_xlim(-plot_range_m - 0.5, plot_range_m + 0.5)
    ax.set_ylim(-1.0, plot_range_m + 0.5) # Allow some negative Y for flexibility

def save_current_plot(frame_number):
    filename = os.path.join(PLOT_SAVE_DIR, f"sonar_map_frame_{frame_number:04d}.png")
    try:
        plt.savefig(filename)
        # print(f"Saved plot: {filename}") # Optional: uncomment for save confirmation
    except Exception as e:
        print(f"Error saving plot {filename}: {e}")

def update_plot(animation_frame_index):
    global obstacle_points, raw_points_buffer, plot_save_frame_counter
    
    new_data_received = False
    try:
        while ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line: continue
            new_data_received = True
            try:
                parts = line.split(',')
                if len(parts) == 2:
                    angle_deg = int(parts[0])
                    distance_mm = int(parts[1])

                    if 0 <= distance_mm <= MAX_SENSOR_RANGE_MM:
                        distance_m = distance_mm / 1000.0
                        angle_rad = math.radians(angle_deg)
                        
                        # Standard Cartesian conversion (sensor at origin)
                        x_raw = distance_m * math.cos(angle_rad)
                        y_raw = distance_m * math.sin(angle_rad)
                        raw_points_buffer.append((x_raw, y_raw))
            except (ValueError, IndexError): 
                pass # Silently ignore parsing errors
            except Exception:
                pass # Silently ignore other line processing errors

        if new_data_received and raw_points_buffer:
            current_buffer_list = list(raw_points_buffer)
            buffer_len = len(current_buffer_list)

            x_values_in_buffer = sorted([p[0] for p in current_buffer_list])
            y_values_in_buffer = sorted([p[1] for p in current_buffer_list])

            median_x = x_values_in_buffer[buffer_len // 2]
            median_y = y_values_in_buffer[buffer_len // 2]
            obstacle_points.append((median_x, median_y))
        
        if obstacle_points:
            # Keep only a limited number of points to avoid clutter, e.g., last 500
            # This helps if the servo sweeps back and forth over the same area.
            # Adjust as needed for your desired visualization.
            MAX_DISPLAY_POINTS = 500 
            if len(obstacle_points) > MAX_DISPLAY_POINTS:
                obstacle_points = obstacle_points[-MAX_DISPLAY_POINTS:]

            x_coords, y_coords = zip(*obstacle_points)
            scatter.set_offsets(list(zip(x_coords, y_coords)))
        else:
            scatter.set_offsets([])

        plot_save_frame_counter += 1
        if plot_save_frame_counter % SAVE_PLOT_EVERY_N_FRAMES == 0:
            save_current_plot(plot_save_frame_counter)

    except serial.SerialException:
        plt.close() 
        return 
    except Exception:
        pass 

    return scatter,

if __name__ == "__main__":
    # Ensure matplotlib and pyserial are installed
    # pip install matplotlib pyserial

    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
        print(f"Attempting to connect to {SERIAL_PORT} at {SERIAL_BAUD} baud...")
        time.sleep(2) # Allow time for Arduino to reset after serial connection
        if ser.is_open:
            print(f"Successfully connected to {SERIAL_PORT}.")
            ser.flushInput() # Clear any old data in the buffer
        else:
            print(f"Failed to open {SERIAL_PORT}.")
            exit()
    except serial.SerialException as e:
        print(f"Serial port error on {SERIAL_PORT}: {e}")
        print("Please check the port name, ensure it's not in use (e.g., by Arduino IDE Serial Monitor), and check permissions.")
        exit()

    setup_plot()
    ani = FuncAnimation(fig, update_plot, interval=PLOT_UPDATE_INTERVAL_MS, blit=True, cache_frame_data=False)
    
    try:
        plt.show()
    finally:
        if 'ser' in locals() and ser.is_open:
            print("Closing serial port.")
            ser.close()
        if plot_save_frame_counter > 0 : # Save one last plot if any frames were processed
             save_current_plot(plot_save_frame_counter + 1) 
        print("Application terminated.")

