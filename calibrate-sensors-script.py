#!/usr/bin/env python3
"""
RTFCS Sensor Calibration Tool
Calibrate IMU, magnetometer, and other sensors
"""

import argparse
import numpy as np
import time
import yaml
import sys
import os
from collections import deque
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import socket
import struct
import threading

class SensorCalibrator:
    def __init__(self, host='localhost', port=5760):
        self.host = host
        self.port = port
        self.socket = None
        self.running = False
        self.data_queue = deque(maxlen=10000)
        
        # Calibration results
        self.accel_offset = np.zeros(3)
        self.accel_scale = np.ones(3)
        self.gyro_offset = np.zeros(3)
        self.mag_offset = np.zeros(3)
        self.mag_scale = np.ones(3)
        
    def connect(self):
        """Connect to flight controller telemetry stream"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            print(f"Connected to flight controller at {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from flight controller"""
        if self.socket:
            self.socket.close()
            self.socket = None
    
    def read_sensor_data(self):
        """Read sensor data from telemetry stream"""
        # Packet format: timestamp(8), accel(3*4), gyro(3*4), mag(3*4)
        packet_size = 8 + 3*4 + 3*4 + 3*4
        
        while self.running:
            try:
                data = self.socket.recv(packet_size)
                if len(data) == packet_size:
                    # Unpack data
                    values = struct.unpack('<Q9f', data)
                    timestamp = values[0]
                    accel = values[1:4]
                    gyro = values[4:7]
                    mag = values[7:10]
                    
                    self.data_queue.append({
                        'timestamp': timestamp,
                        'accel': accel,
                        'gyro': gyro,
                        'mag': mag
                    })
            except Exception as e:
                print(f"Error reading data: {e}")
                break
    
    def calibrate_accelerometer(self):
        """Calibrate accelerometer using 6-point tumble test"""
        print("\n=== Accelerometer Calibration ===")
        print("Place the vehicle on each of its 6 faces (stable and level)")
        print("Press ENTER when ready for each position\n")
        
        positions = [
            "Level (top up)",
            "Inverted (top down)",
            "Left side down",
            "Right side down",
            "Nose down",
            "Nose up"
        ]
        
        measurements = []
        
        for i, position in enumerate(positions):
            input(f"Position {i+1}/6: {position} - Press ENTER when ready...")
            
            # Collect samples
            print("Collecting data...")
            samples = self.collect_samples(duration=3.0, sensor='accel')
            
            if len(samples) > 0:
                mean_accel = np.mean(samples, axis=0)
                measurements.append(mean_accel)
                print(f"  Measured: {mean_accel}")
            else:
                print("  ERROR: No data collected!")
                return False
        
        # Calculate calibration parameters
        measurements = np.array(measurements)
        
        # Expected values for each position (in g)
        expected = np.array([
            [0, 0, -1],  # Level
            [0, 0, 1],   # Inverted
            [1, 0, 0],   # Left down
            [-1, 0, 0],  # Right down
            [0, 1, 0],   # Nose down
            [0, -1, 0]   # Nose up
        ]) * 9.81  # Convert to m/s²
        
        # Solve for offset and scale
        # Model: measured = scale * (true + offset)
        self.accel_offset = np.zeros(3)
        self.accel_scale = np.ones(3)
        
        for axis in range(3):
            # Find measurements where this axis should be ±9.81 or 0
            high_idx = np.where(expected[:, axis] > 5)[0]
            low_idx = np.where(expected[:, axis] < -5)[0]
            
            if len(high_idx) > 0 and len(low_idx) > 0:
                high_val = measurements[high_idx[0], axis]
                low_val = measurements[low_idx[0], axis]
                
                self.accel_scale[axis] = 2 * 9.81 / (high_val - low_val)
                self.accel_offset[axis] = -(high_val + low_val) / 2
        
        print(f"\nAccelerometer calibration complete:")
        print(f"  Offset: {self.accel_offset}")
        print(f"  Scale: {self.accel_scale}")
        
        return True
    
    def calibrate_gyroscope(self):
        """Calibrate gyroscope bias"""
        print("\n=== Gyroscope Calibration ===")
        print("Keep the vehicle completely still")
        input("Press ENTER when ready...")
        
        print("Collecting data...")
        samples = self.collect_samples(duration=10.0, sensor='gyro')
        
        if len(samples) > 0:
            self.gyro_offset = np.mean(samples, axis=0)
            gyro_std = np.std(samples, axis=0)
            
            print(f"\nGyroscope calibration complete:")
            print(f"  Offset: {self.gyro_offset} rad/s")
            print(f"  Noise (std): {gyro_std} rad/s")
            
            # Check if noise is acceptable
            if np.any(gyro_std > 0.01):
                print("  WARNING: High noise detected! Was the vehicle moving?")
            
            return True
        else:
            print("ERROR: No data collected!")
            return False
    
    def calibrate_magnetometer(self):
        """Calibrate magnetometer using sphere fitting"""
        print("\n=== Magnetometer Calibration ===")
        print("Rotate the vehicle slowly in all directions")
        print("Try to cover all possible orientations")
        input("Press ENTER to start (30 seconds)...")
        
        print("Collecting data...")
        samples = self.collect_samples(duration=30.0, sensor='mag')
        
        if len(samples) < 100:
            print("ERROR: Not enough data collected!")
            return False
        
        samples = np.array(samples)
        
        # Fit ellipsoid to data
        # Model: (x-x0)²/a² + (y-y0)²/b² + (z-z0)²/c² = 1
        center, radii, rotation = self.fit_ellipsoid(samples)
        
        self.mag_offset = center
        self.mag_scale = 1.0 / radii  # Normalize to unit sphere
        
        print(f"\nMagnetometer calibration complete:")
        print(f"  Offset: {self.mag_offset}")
        print(f"  Scale: {self.mag_scale}")
        
        # Visualize calibration
        self.plot_magnetometer_calibration(samples, center, radii)
        
        return True
    
    def collect_samples(self, duration, sensor):
        """Collect sensor samples for specified duration"""
        samples = []
        start_time = time.time()
        
        # Clear queue
        self.data_queue.clear()
        
        while time.time() - start_time < duration:
            if len(self.data_queue) > 0:
                data = self.data_queue.popleft()
                if sensor == 'accel':
                    samples.append(data['accel'])
                elif sensor == 'gyro':
                    samples.append(data['gyro'])
                elif sensor == 'mag':
                    samples.append(data['mag'])
            time.sleep(0.001)
        
        return np.array(samples)
    
    def fit_ellipsoid(self, points):
        """Fit ellipsoid to 3D points"""
        # Algebraic fit to ellipsoid
        # Form: ax² + by² + cz² + 2fxy + 2gxz + 2hyz + 2px + 2qy + 2rz + d = 0
        
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        
        # Build design matrix
        D = np.column_stack([
            x**2, y**2, z**2,
            2*x*y, 2*x*z, 2*y*z,
            2*x, 2*y, 2*z,
            np.ones_like(x)
        ])
        
        # Solve using SVD
        _, _, V = np.linalg.svd(D)
        coeffs = V[-1, :]
        
        # Extract center and radii (simplified for axis-aligned ellipsoid)
        a, b, c = coeffs[0:3]
        p, q, r = coeffs[6:9]
        d = coeffs[9]
        
        center = np.array([-p/a, -q/b, -r/c])
        radii = np.sqrt(np.array([
            (p**2/a + q**2/b + r**2/c - d) / a,
            (p**2/a + q**2/b + r**2/c - d) / b,
            (p**2/a + q**2/b + r**2/c - d) / c
        ]))
        
        return center, radii, np.eye(3)  # Simplified - no rotation
    
    def plot_magnetometer_calibration(self, raw_data, center, radii):
        """Visualize magnetometer calibration"""
        fig = plt.figure(figsize=(12, 5))
        
        # Raw data
        ax1 = fig.add_subplot(121, projection='3d')
        ax1.scatter(raw_data[:, 0], raw_data[:, 1], raw_data[:, 2], 
                   c='blue', alpha=0.5, s=1)
        ax1.set_xlabel('X (Gauss)')
        ax1.set_ylabel('Y (Gauss)')
        ax1.set_zlabel('Z (Gauss)')
        ax1.set_title('Raw Magnetometer Data')
        
        # Calibrated data
        ax2 = fig.add_subplot(122, projection='3d')
        calibrated = (raw_data - center) / radii
        ax2.scatter(calibrated[:, 0], calibrated[:, 1], calibrated[:, 2], 
                   c='green', alpha=0.5, s=1)
        
        # Draw unit sphere
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        x_sphere = np.outer(np.cos(u), np.sin(v))
        y_sphere = np.outer(np.sin(u), np.sin(v))
        z_sphere = np.outer(np.ones(np.size(u)), np.cos(v))
        ax2.plot_surface(x_sphere, y_sphere, z_sphere, alpha=0.2, color='gray')
        
        ax2.set_xlabel('X (normalized)')
        ax2.set_ylabel('Y (normalized)')
        ax2.set_zlabel('Z (normalized)')
        ax2.set_title('Calibrated Magnetometer Data')
        ax2.set_xlim([-1.5, 1.5])
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_zlim([-1.5, 1.5])
        
        plt.tight_layout()
        plt.show()
    
    def save_calibration(self, filename):
        """Save calibration to YAML file"""
        calibration = {
            'accelerometer': {
                'offset': self.accel_offset.tolist(),
                'scale': self.accel_scale.tolist()
            },
            'gyroscope': {
                'offset': self.gyro_offset.tolist()
            },
            'magnetometer': {
                'offset': self.mag_offset.tolist(),
                'scale': self.mag_scale.tolist()
            },
            'calibration_date': time.strftime('%Y-%m-%d %H:%M:%S')
        }
        
        with open(filename, 'w') as f:
            yaml.dump(calibration, f, default_flow_style=False)
        
        print(f"\nCalibration saved to {filename}")
    
    def run_calibration(self, sensors=['accel', 'gyro', 'mag']):
        """Run full calibration sequence"""
        if not self.connect():
            return False
        
        # Start data collection thread
        self.running = True
        data_thread = threading.Thread(target=self.read_sensor_data)
        data_thread.start()
        
        try:
            # Calibrate each sensor
            if 'gyro' in sensors:
                if not self.calibrate_gyroscope():
                    return False
            
            if 'accel' in sensors:
                if not self.calibrate_accelerometer():
                    return False
            
            if 'mag' in sensors:
                if not self.calibrate_magnetometer():
                    return False
            
            return True
            
        finally:
            # Stop data collection
            self.running = False
            data_thread.join()
            self.disconnect()

def main():
    parser = argparse.ArgumentParser(description='RTFCS Sensor Calibration Tool')
    parser.add_argument('--host', default='localhost', 
                        help='Flight controller IP address')
    parser.add_argument('--port', type=int, default=5760, 
                        help='Telemetry port')
    parser.add_argument('--output', default='sensor_calibration.yaml',
                        help='Output calibration file')
    parser.add_argument('--sensors', nargs='+', 
                        default=['accel', 'gyro', 'mag'],
                        choices=['accel', 'gyro', 'mag'],
                        help='Sensors to calibrate')
    
    args = parser.parse_args()
    
    print("RTFCS Sensor Calibration Tool")
    print("=============================\n")
    
    calibrator = SensorCalibrator(args.host, args.port)
    
    if calibrator.run_calibration(args.sensors):
        calibrator.save_calibration(args.output)
        print("\nCalibration completed successfully!")
        
        # Generate config snippet
        print("\nAdd this to your config file:")
        print("sensors:")
        print("  imu:")
        print(f"    accel_offset: {calibrator.accel_offset.tolist()}")
        print(f"    gyro_offset: {calibrator.gyro_offset.tolist()}")
        print("  magnetometer:")
        print(f"    offset: {calibrator.mag_offset.tolist()}")
        print(f"    scale:")
        print(f"      - {calibrator.mag_scale.tolist()}")
    else:
        print("\nCalibration failed!")
        sys.exit(1)

if __name__ == '__main__':
    main()