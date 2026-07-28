#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
3D Flight EKF Simulation and Error Analysis
===========================================
This script simulates the 3D flight trajectory of a rocket based on true 1D flight data,
models the physical barometric pressure lag and sensor noise (official datasheet + 30%),
runs a 3D Extended Kalman Filter (EKF) matching the firmware implementation,
and analyzes the difference between the EKF state estimation and the true trajectory.
"""

import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg') # Non-interactive backend
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Configure matplotlib for dark theme and high quality
plt.style.use('dark_background')
matplotlib.rcParams['font.sans-serif'] = ['Arial', 'Helvetica', 'sans-serif']
matplotlib.rcParams['axes.unicode_minus'] = False

# =========================================================================
# 1. Quaternion and Rotation Utilities
# =========================================================================
def quat_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return np.array([w, x, y, z])

def quat_inv(q):
    w, x, y, z = q
    norm_sq = w**2 + x**2 + y**2 + z**2
    if norm_sq < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return np.array([w, -x, -y, -z]) / norm_sq

def euler_to_quat(yaw, pitch, roll):
    # yaw (Z), pitch (Y), roll (X) in radians
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return np.array([qw, qx, qy, qz])

def quat_to_rot(q):
    qw, qx, qy, qz = q
    r11 = 1.0 - 2.0 * (qy**2 + qz**2)
    r12 = 2.0 * (qx*qy - qw*qz)
    r13 = 2.0 * (qx*qz + qw*qy)
    r21 = 2.0 * (qx*qy + qw*qz)
    r22 = 1.0 - 2.0 * (qx**2 + qz**2)
    r23 = 2.0 * (qy*qz - qw*qx)
    r31 = 2.0 * (qx*qz - qw*qy)
    r32 = 2.0 * (qy*qz + qw*qx)
    r33 = 1.0 - 2.0 * (qx**2 + qy**2)
    return np.array([[r11, r12, r13],
                     [r21, r22, r23],
                     [r31, r32, r33]])

def quat_to_euler(q):
    qw, qx, qy, qz = q
    # roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = np.sign(sinp) * np.pi / 2
    else:
        pitch = np.arcsin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return yaw, pitch, roll

# =========================================================================
# 2. Physical Baro Bay Pressure Lag Model
# =========================================================================
def alt_to_press(h):
    ratio = 1.0 - 0.0065 * h / 288.15
    ratio = np.clip(ratio, 0.01, 1.5)
    return 1013.25 * (ratio ** 5.255877)

def press_to_alt(P):
    ratio = np.clip(P / 1013.25, 0.001, 2.0)
    return 44330.7692 * (1.0 - (ratio ** 0.190263))

def simulate_baro_lag(t_sim, alt_true, N=4):
    """
    Simulates the physical barometric lag inside the avionics bay using RK4.
    N: number of vent holes (default 4)
    """
    V = 9.8e-5 # avionics bay volume in m^3
    Q_spec = 9.1667e-6 # vent hole volumetric parameter in m^3/s
    dt = 0.001
    n = len(t_sim)
    
    P_out = alt_to_press(alt_true)
    P_in = np.zeros(n)
    P_in[0] = P_out[0] # assume starting at equilibrium
    
    C = (N * Q_spec) / (V * 70.0)
    
    for i in range(n - 1):
        P_curr = P_in[i]
        P_ext_curr = P_out[i]
        P_ext_next = P_out[i+1]
        P_ext_mid = 0.5 * (P_ext_curr + P_ext_next)
        
        # RK4
        k1 = - C * P_curr * (P_curr - P_ext_curr)
        P_half1 = P_curr + 0.5 * dt * k1
        k2 = - C * P_half1 * (P_half1 - P_ext_mid)
        P_half2 = P_curr + 0.5 * dt * k2
        k3 = - C * P_half2 * (P_half2 - P_ext_mid)
        P_next = P_curr + dt * k3
        k4 = - C * P_next * (P_next - P_ext_next)
        
        P_in[i+1] = P_curr + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        
    return press_to_alt(P_in)

# =========================================================================
# 3. EKF Class Matching C-Firmware Implementation
# =========================================================================
class EKF3D:
    def __init__(self, q_pos=0.005, q_vel=0.1, r_baro=0.36, r_gps=6.25):
        # State vector: [px, py, pz, vx, vy, vz]^T
        self.x = np.zeros(6)
        # Covariance P
        self.P = np.zeros((6, 6))
        for i in range(3):
            self.P[i, i] = 1.0
            self.P[i+3, i+3] = 2.0
        # Attitude quaternion
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.r_baro = r_baro
        self.r_gps = r_gps
        
        # 1000Hz circular history buffer for 20ms barometer delay compensation
        self.z_history = np.zeros(25)
        self.z_history_idx = 0
        self.in_flight = False
        
        # Stats bookkeeping
        self.baro_updates = 0
        self.baro_rejections = 0

    def attitude_update(self, gx, gy, gz, ax, ay, az, dt, fsm_state):
        # Mahony filter gravity feedback during pad and descent phase (FSM state >= 5)
        if not self.in_flight or fsm_state >= 5:
            norm_a = np.sqrt(ax**2 + ay**2 + az**2)
            if norm_a > 0.01 and abs(norm_a - 9.80665) < 1.0:
                ax_n = ax / norm_a
                ay_n = ay / norm_a
                az_n = az / norm_a
                
                qw, qx, qy, qz = self.q
                # Predicted gravity direction in the body frame (third row of R(q))
                vx_g = 2.0 * (qx*qz - qw*qy)
                vy_g = 2.0 * (qy*qz + qw*qx)
                vz_g = 1.0 - 2.0 * (qx**2 + qy**2)
                
                # Cross product error
                ex = (ay_n * vz_g - az_n * vy_g)
                ey = (az_n * vx_g - ax_n * vz_g)
                ez = (ax_n * vy_g - ay_n * vx_g)
                
                Kp = 2.0
                gx += Kp * ex
                gy += Kp * ey
                gz += Kp * ez
                
        # Re-integrate quaternion
        qw, qx, qy, qz = self.q
        d_qw = 0.5 * dt * (-qx * gx - qy * gy - qz * gz)
        d_qx = 0.5 * dt * ( qw * gx + qy * gz - qz * gy)
        d_qy = 0.5 * dt * ( qw * gy - qx * gz + qz * gx)
        d_qz = 0.5 * dt * ( qw * gz + qx * gy - qy * gx)
        
        self.q += np.array([d_qw, d_qx, d_qy, d_qz])
        norm = np.linalg.norm(self.q)
        if norm > 1e-6:
            self.q /= norm

    def predict(self, ax, ay, az, dt):
        R = quat_to_rot(self.q)
        # Rotate acceleration into navigation frame and subtract gravity
        a_nav = R @ np.array([ax, ay, az]) - np.array([0, 0, 9.80665])
        
        # Propagate state vector x
        dt2 = 0.5 * dt * dt
        self.x[0:3] += self.x[3:6] * dt + a_nav * dt2
        self.x[3:6] += a_nav * dt
        
        # Save predicted Z-altitude to circular history buffer (matching C task)
        self.z_history[self.z_history_idx] = self.x[2]
        self.z_history_idx = (self.z_history_idx + 1) % 25
        
        # Analytical covariance propagation: P = F * P * F^T + Q
        P_pp = self.P[0:3, 0:3]
        P_pv = self.P[0:3, 3:6]
        P_vp = self.P[3:6, 0:3]
        P_vv = self.P[3:6, 3:6]
        
        P_pp_new = P_pp + dt * (P_vp + P_pv) + dt*dt * P_vv
        P_pv_new = P_pv + dt * P_vv
        P_vp_new = P_vp + dt * P_vv
        
        self.P[0:3, 0:3] = P_pp_new
        self.P[0:3, 3:6] = P_pv_new
        self.P[3:6, 0:3] = P_vp_new
        
        # Add process noise Q to diagonal of position and velocity
        for i in range(3):
            self.P[i, i] += self.q_pos
            self.P[i+3, i+3] += self.q_vel
            
        # Clamp covariance diagonals to prevent divergence (ekf_guard)
        for i in range(6):
            if self.P[i, i] > 5000.0:
                self.P[i, i] = 5000.0

    def update_baro(self, baro_alt):
        self.baro_updates += 1
        # Look back 20ms (20 samples at 1000Hz)
        # z_history_idx is the index of the next write (oldest sample).
        # To look back 20 samples:
        hist_idx = (self.z_history_idx + 25 - 20) % 25
        z_pred = self.z_history[hist_idx]
            
        y = baro_alt - z_pred
        S = self.P[2, 2] + self.r_baro
        if S < 1e-6:
            return
            
        # Innovation gating matching C code (|y| > max(5*sigma, 25m))
        sigma = np.sqrt(S)
        gate = max(5.0 * sigma, 25.0)
        if abs(y) > gate:
            self.baro_rejections += 1
            return # Rejected outlier
            
        invS = 1.0 / S
        K = self.P[:, 2] * invS
        self.x += K * y
        
        # Joseph Form covariance update
        I = np.eye(6)
        H = np.zeros(6)
        H[2] = 1.0
        KH = np.outer(K, H)
        self.P = (I - KH) @ self.P @ (I - KH).T + np.outer(K, K) * self.r_baro

    def update_gps(self, meas_E, meas_N):
        self.update_scalar_pos(0, meas_E, self.r_gps)
        self.update_scalar_pos(1, meas_N, self.r_gps)

    def update_scalar_pos(self, idx, z, R):
        y = z - self.x[idx]
        S = self.P[idx, idx] + R
        if S < 1e-6:
            return
        invS = 1.0 / S
        K = self.P[:, idx] * invS
        self.x += K * y
        
        I = np.eye(6)
        H = np.zeros(6)
        H[idx] = 1.0
        KH = np.outer(K, H)
        self.P = (I - KH) @ self.P @ (I - KH).T + np.outer(K, K) * R

# =========================================================================
# 4. Simulation Execution
# =========================================================================
def run_simulation(csv_path, case_name, noise_cfg):
    print(f"[Simulation] Running EKF for: {case_name} ...")
    
    # Load 1D CSV data
    data = []
    with open(csv_path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.strip().split(',')
            if len(parts) < 5:
                continue
            try:
                data.append([float(parts[0]), float(parts[1]), float(parts[2]), float(parts[4])])
            except ValueError:
                continue
    data = np.array(data)
    t_csv, alt_csv, vel_csv, acc_csv = data[:, 0], data[:, 1], data[:, 2], data[:, 3]
    
    # Resample to 1000Hz (1ms interval)
    t_max = 35.0 # Focus on first 35 seconds (boost, apogee, and early parachute descent)
    t_sim = np.arange(0, t_max, 0.001)
    alt_true_z = np.interp(t_sim, t_csv, alt_csv)
    vel_true_z = np.interp(t_sim, t_csv, vel_csv)
    acc_true_z = np.interp(t_sim, t_csv, acc_csv)
    
    n_points = len(t_sim)
    dt = 0.001
    
    # Detect flight phases
    apogee_idx = np.argmax(alt_true_z)
    t_apogee = t_sim[apogee_idx]
    
    # Generate 3D True Trajectory (tilt, roll, wind drift)
    # Pitch: tilts to 3.5 deg during launch, reaches 12 deg at apogee, then swings
    theta_true = np.zeros(n_points)
    # Yaw: 45 deg (North-East) during ascent, drifts slightly during coast, shifts during descent
    psi_true = np.zeros(n_points)
    # Roll: rocket spins at 2 Hz during ascent, then stabilizes under parachute
    phi_true = np.zeros(n_points)
    
    for i in range(n_points):
        t = t_sim[i]
        if t < 1.0: # Pad stay
            theta_true[i] = np.radians(3.0)
            psi_true[i] = np.radians(45.0)
            phi_true[i] = 0.0
        elif t < 6.0: # Boost phase
            # Tilt increases from 3.0 to 5.0 degrees
            theta_true[i] = np.radians(3.0 + 2.0 * (t - 1.0) / 5.0)
            psi_true[i] = np.radians(45.0)
            phi_true[i] = 2.0 * 2.0 * np.pi * (t - 1.0) # 2 Hz spin
        elif t < t_apogee: # Coast phase
            # Tilt increases to 12 degrees
            theta_true[i] = np.radians(5.0 + 7.0 * (t - 6.0) / (t_apogee - 6.0))
            psi_true[i] = np.radians(45.0 + 10.0 * (t - 6.0) / (t_apogee - 6.0))
            phi_true[i] = phi_true[i-1] + 2.0 * np.pi * 1.5 * dt # Slows down to 1.5 Hz
        else: # Parachute descent
            # Swings around vertical (0 deg tilt) with 8 deg amplitude
            theta_true[i] = np.radians(8.0 * np.sin(0.8 * (t - t_apogee)))
            psi_true[i] = psi_true[apogee_idx] + np.radians(3.0 * (t - t_apogee)) # slow yaw drift
            phi_true[i] = phi_true[apogee_idx] + np.radians(15.0 * np.sin(0.3 * (t - t_apogee))) # roll oscillation
            
    # Calculate True Position and Velocity (ENU)
    pos_true = np.zeros((n_points, 3))
    vel_true = np.zeros((n_points, 3))
    
    # Wind configuration
    wind_E = 4.0 # m/s East
    wind_N = 2.0 # m/s North
    
    for i in range(n_points):
        t = t_sim[i]
        theta = theta_true[i]
        psi = psi_true[i]
        
        # Ascent: velocity is along the rocket longitudinal axis
        if t < t_apogee:
            vel_true[i, 2] = vel_true_z[i] * np.cos(theta)
            vel_true[i, 0] = vel_true_z[i] * np.sin(theta) * np.cos(psi)
            vel_true[i, 1] = vel_true_z[i] * np.sin(theta) * np.sin(psi)
        else:
            # Under parachute: descends vertically but drifts with wind and swings
            vel_true[i, 2] = vel_true_z[i] # vertical velocity
            vel_true[i, 0] = wind_E + 0.8 * np.sin(0.5 * (t - t_apogee))
            vel_true[i, 1] = wind_N + 0.8 * np.cos(0.5 * (t - t_apogee))
            
    # Integrate position
    for i in range(1, n_points):
        pos_true[i] = pos_true[i-1] + 0.5 * (vel_true[i-1] + vel_true[i]) * dt
        
    # True inertial acceleration
    acc_true = np.zeros((n_points, 3))
    for i in range(1, n_points - 1):
        acc_true[i] = (vel_true[i+1] - vel_true[i-1]) / (2 * dt)
    acc_true[0] = (vel_true[1] - vel_true[0]) / dt
    acc_true[-1] = (vel_true[-1] - vel_true[-2]) / dt
    
    # True quaternion
    q_true = np.zeros((n_points, 4))
    for i in range(n_points):
        q_true[i] = euler_to_quat(psi_true[i], theta_true[i], phi_true[i])
        
    # True angular velocity (body frame)
    q_dot = np.zeros_like(q_true)
    q_dot[1:-1] = (q_true[2:] - q_true[:-2]) / (2 * dt)
    q_dot[0] = (q_true[1] - q_true[0]) / dt
    q_dot[-1] = (q_true[-1] - q_true[-2]) / dt

    omega_true = np.zeros((n_points, 3))
    for i in range(n_points):
        q_inv = quat_inv(q_true[i])
        prod = quat_mult(q_inv, q_dot[i])
        omega_true[i] = 2.0 * prod[1:]
        
    # True specific force in nav frame (inertial accel + gravity)
    spec_force_nav = acc_true + np.tile([0, 0, 9.80665], (n_points, 1))
    # Rotate to body frame: f_body = R(q)^T * f_nav
    spec_force_body = np.zeros((n_points, 3))
    for i in range(n_points):
        R = quat_to_rot(q_true[i])
        spec_force_body[i] = R.T @ spec_force_nav[i]
        
    # Simulate Barometric Lag
    alt_baro_true = simulate_baro_lag(t_sim, pos_true[:, 2], N=4)
    # Add a sensor group delay of 20ms
    baro_delay_steps = 20
    alt_baro_delayed = np.zeros(n_points)
    for i in range(n_points):
        if i >= baro_delay_steps:
            alt_baro_delayed[i] = alt_baro_true[i - baro_delay_steps]
        else:
            alt_baro_delayed[i] = alt_baro_true[0]
            
    # Add noise to simulate sensor readings
    np.random.seed(42) # For repeatability
    
    # 1. Accelerometer noise
    acc_noisy = spec_force_body + np.random.normal(0, noise_cfg['accel_std'], (n_points, 3))
    # 2. Gyroscope noise
    gyro_noisy = omega_true + np.random.normal(0, noise_cfg['gyro_std'], (n_points, 3))
    # 3. Barometer noise
    baro_noisy = alt_baro_delayed + np.random.normal(0, noise_cfg['baro_std'], n_points)
    # 4. GPS noise (1Hz updates)
    gps_noisy = pos_true[:, 0:2] + np.random.normal(0, noise_cfg['gps_std'], (n_points, 2))
    
    # Run EKF
    ekf = EKF3D(q_pos=0.005, q_vel=0.1, r_baro=0.36, r_gps=6.25)
    
    # Initialize EKF attitude from TRIAD-like true attitude at pad
    ekf.q = q_true[0].copy()
    
    est_pos = np.zeros((n_points, 3))
    est_vel = np.zeros((n_points, 3))
    est_q = np.zeros((n_points, 4))
    
    # FSM state index tracking for EKF attitude complementary filter logic
    # STATE_PAD=1, STATE_BOOST=2, STATE_COAST=3, STATE_DESCENT=5
    fsm_state = 1
    
    for i in range(n_points):
        t = t_sim[i]
        
        # FSM phase transitions
        if t < 1.0:
            fsm_state = 1 # PAD
        elif t < 6.0:
            fsm_state = 2 # BOOST
            ekf.in_flight = True
        elif t < t_apogee:
            fsm_state = 3 # COAST
        else:
            fsm_state = 5 # DESCENT
            
        # 1. EKF Attitude Update
        ekf.attitude_update(gyro_noisy[i, 0], gyro_noisy[i, 1], gyro_noisy[i, 2],
                            acc_noisy[i, 0], acc_noisy[i, 1], acc_noisy[i, 2],
                            dt, fsm_state)
        
        # 2. EKF Prediction step (this increments the z_history_idx and saves x[2])
        ekf.predict(acc_noisy[i, 0], acc_noisy[i, 1], acc_noisy[i, 2], dt)
        
        # 3. Barometer Update (200Hz = every 5ms)
        if i % 5 == 0:
            ekf.update_baro(baro_noisy[i])
            
        # 4. GPS Update (1Hz = every 1000ms)
        # Only update in flight, match C logic
        if ekf.in_flight and i % 1000 == 0:
            ekf.update_gps(gps_noisy[i, 0], gps_noisy[i, 1])
            
        est_pos[i] = ekf.x[0:3]
        est_vel[i] = ekf.x[3:6]
        est_q[i] = ekf.q.copy()
        
    return {
        't': t_sim,
        'pos_true': pos_true,
        'vel_true': vel_true,
        'q_true': q_true,
        'pos_est': est_pos,
        'vel_est': est_vel,
        'q_est': est_q,
        't_apogee': t_apogee,
        'apogee_true_alt': alt_true_z[apogee_idx],
        'baro_updates': ekf.baro_updates,
        'baro_rejections': ekf.baro_rejections
    }

# =========================================================================
# 5. Main Script Configuration and Evaluation
# =========================================================================
if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_path = os.path.join(script_dir, "..", "flight_data", "台灣盃2026時間高度垂直速度合速度合加速度.csv")
    
    # 1. Case 1: Official Datasheet + 30% Noise Configuration
    noise_datasheet_30 = {
        'accel_std': 0.093,
        'gyro_std': 0.010,
        'baro_std': 0.16,
        'gps_std': 2.21
    }
    
    # 2. Case 2: Flight Vibration + 30% Noise Configuration
    noise_vibration_30 = {
        'accel_std': 2.55,
        'gyro_std': 0.050,
        'baro_std': 0.40,
        'gps_std': 4.00
    }
    
    # Run simulations
    res_ds = run_simulation(csv_path, "Official Datasheet + 30% (Clean)", noise_datasheet_30)
    res_vib = run_simulation(csv_path, "Flight Vibration + 30% (Real Flight)", noise_vibration_30)
    
    # =========================================================================
    # Error Calculation and Statistics
    # =========================================================================
    def calculate_stats(res):
        t = res['t']
        pos_err = res['pos_est'] - res['pos_true']
        vel_err = res['vel_est'] - res['vel_true']
        
        # Position error magnitudes
        pos_err_mag = np.linalg.norm(pos_err, axis=1)
        vel_err_mag = np.linalg.norm(vel_err, axis=1)
        
        # RMS errors
        pos_rmse = np.sqrt(np.mean(pos_err**2, axis=0))
        vel_rmse = np.sqrt(np.mean(vel_err**2, axis=0))
        pos_rmse_total = np.sqrt(np.mean(pos_err_mag**2))
        vel_rmse_total = np.sqrt(np.mean(vel_err_mag**2))
        
        # Max errors
        pos_max = np.max(np.abs(pos_err), axis=0)
        vel_max = np.max(np.abs(vel_err), axis=0)
        
        # Apogee estimation
        # Find index of max estimated altitude
        est_apogee_idx = np.argmax(res['pos_est'][:, 2])
        t_apogee_est = t[est_apogee_idx]
        alt_apogee_est = res['pos_est'][est_apogee_idx, 2]
        
        t_apogee_err = t_apogee_est - res['t_apogee']
        alt_apogee_err = alt_apogee_est - res['apogee_true_alt']
        
        return {
            'pos_rmse': pos_rmse,
            'vel_rmse': vel_rmse,
            'pos_rmse_total': pos_rmse_total,
            'vel_rmse_total': vel_rmse_total,
            'pos_max': pos_max,
            'vel_max': vel_max,
            't_apogee_est': t_apogee_est,
            'alt_apogee_est': alt_apogee_est,
            't_apogee_err': t_apogee_err,
            'alt_apogee_err': alt_apogee_err
        }
        
    stats_ds = calculate_stats(res_ds)
    stats_vib = calculate_stats(res_vib)
    
    # Print results
    print("\n" + "="*50)
    print("SIMULATION STATISTICAL RESULTS")
    print("="*50)
    print(f"Apogee (True): Time = {res_ds['t_apogee']:.3f} s, Altitude = {res_ds['apogee_true_alt']:.2f} m")
    
    print("\n[Case 1: Official Datasheet + 30% Noise]")
    print(f"  Baro Updates    : {res_ds['baro_updates']} (Rejections: {res_ds['baro_rejections']} = {res_ds['baro_rejections']/res_ds['baro_updates']*100:.1f}%)")
    print(f"  Apogee Estimate : Time = {stats_ds['t_apogee_est']:.3f} s (Error: {stats_ds['t_apogee_err']+.0001:+.3f} s)")
    print(f"                    Alt  = {stats_ds['alt_apogee_est']:.2f} m (Error: {stats_ds['alt_apogee_err']+.0001:+.2f} m)")
    print(f"  Position RMSE   : East = {stats_ds['pos_rmse'][0]:.3f} m, North = {stats_ds['pos_rmse'][1]:.3f} m, Up (Alt) = {stats_ds['pos_rmse'][2]:.3f} m")
    print(f"  Velocity RMSE   : East = {stats_ds['vel_rmse'][0]:.3f} m/s, North = {stats_ds['vel_rmse'][1]:.3f} m/s, Up = {stats_ds['vel_rmse'][2]:.3f} m/s")
    print(f"  Max Pos Error   : East = {stats_ds['pos_max'][0]:.3f} m, North = {stats_ds['pos_max'][1]:.3f} m, Up = {stats_ds['pos_max'][2]:.3f} m")
    print(f"  Max Vel Error   : East = {stats_ds['vel_max'][0]:.3f} m/s, North = {stats_ds['vel_max'][1]:.3f} m/s, Up = {stats_ds['vel_max'][2]:.3f} m/s")
    
    print("\n[Case 2: Flight Vibration + 30% Noise]")
    print(f"  Baro Updates    : {res_vib['baro_updates']} (Rejections: {res_vib['baro_rejections']} = {res_vib['baro_rejections']/res_vib['baro_updates']*100:.1f}%)")
    print(f"  Apogee Estimate : Time = {stats_vib['t_apogee_est']:.3f} s (Error: {stats_vib['t_apogee_err']+.0001:+.3f} s)")
    print(f"                    Alt  = {stats_vib['alt_apogee_est']:.2f} m (Error: {stats_vib['alt_apogee_err']+.0001:+.2f} m)")
    print(f"  Position RMSE   : East = {stats_vib['pos_rmse'][0]:.3f} m, North = {stats_vib['pos_rmse'][1]:.3f} m, Up (Alt) = {stats_vib['pos_rmse'][2]:.3f} m")
    print(f"  Velocity RMSE   : East = {stats_vib['vel_rmse'][0]:.3f} m/s, North = {stats_vib['vel_rmse'][1]:.3f} m/s, Up = {stats_vib['vel_rmse'][2]:.3f} m/s")
    print(f"  Max Pos Error   : East = {stats_vib['pos_max'][0]:.3f} m, North = {stats_vib['pos_max'][1]:.3f} m, Up = {stats_vib['pos_max'][2]:.3f} m")
    print(f"  Max Vel Error   : East = {stats_vib['vel_max'][0]:.3f} m/s, North = {stats_vib['vel_max'][1]:.3f} m/s, Up = {stats_vib['vel_max'][2]:.3f} m/s")
    print("="*50)
    
    # =========================================================================
    # 6. Plotting Results
    # =========================================================================
    output_dir = os.path.join(script_dir, "..", "flight_plots")
    os.makedirs(output_dir, exist_ok=True)
    
    # Path for conversation artifacts directory
    artifact_dir = "/Users/laizhiquan/.gemini/antigravity/brain/da9bb700-0668-4d6e-a71d-8caa8c4f181b"
    os.makedirs(artifact_dir, exist_ok=True)
    
    # Plot 1: 3D Trajectory Comparison
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.view_init(elev=20, azim=35)
    
    t = res_ds['t']
    pos_true = res_ds['pos_true']
    pos_est_ds = res_ds['pos_est']
    pos_est_vib = res_vib['pos_est']
    apogee_idx = np.argmax(pos_true[:, 2])
    
    # Subsample for cleaner 3D plot
    sub = 100 # Draw every 100th point
    ax.plot(pos_true[::sub, 0], pos_true[::sub, 1], pos_true[::sub, 2],
            color='#ffffff', label='True Trajectory', linewidth=3.0, linestyle='--')
    ax.plot(pos_est_ds[::sub, 0], pos_est_ds[::sub, 1], pos_est_ds[::sub, 2],
            color='#00f2fe', label='EKF - Datasheet + 30%', linewidth=2.0)
    ax.plot(pos_est_vib[::sub, 0], pos_est_vib[::sub, 1], pos_est_vib[::sub, 2],
            color='#c084fc', label='EKF - Vibration + 30%', linewidth=2.0)
    
    # Annotate key points
    ax.scatter(pos_true[0, 0], pos_true[0, 1], pos_true[0, 2], color='green', s=50, label='Launchpad')
    ax.scatter(pos_true[apogee_idx, 0], pos_true[apogee_idx, 1], pos_true[apogee_idx, 2], color='red', s=50, label='True Apogee')
    
    ax.set_title("3D Rocket Trajectory: EKF Estimation vs True Path", fontsize=14, fontweight='bold', pad=15)
    ax.set_xlabel("East (X) [m]")
    ax.set_ylabel("North (Y) [m]")
    ax.set_zlabel("Altitude (Z) [m]")
    ax.legend(loc='upper left')
    ax.grid(True, alpha=0.3)
    
    fig.tight_layout()
    plt.savefig(os.path.join(output_dir, "ekf_3d_trajectory.png"), dpi=150)
    plt.savefig(os.path.join(artifact_dir, "ekf_3d_trajectory.png"), dpi=150)
    plt.close()
    
    # Plot 2: Position and Velocity Errors
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    
    # Positions
    axes[0, 0].plot(t, pos_est_ds[:, 0] - pos_true[:, 0], color='#00f2fe', label='Datasheet + 30%', alpha=0.9)
    axes[0, 0].plot(t, pos_est_vib[:, 0] - pos_true[:, 0], color='#c084fc', label='Vibration + 30%', alpha=0.8)
    axes[0, 0].set_ylabel("East Error (X) [m]")
    axes[0, 0].grid(True, alpha=0.2)
    axes[0, 0].legend()
    axes[0, 0].set_title("Position Errors (Estimated - True)")
    
    axes[1, 0].plot(t, pos_est_ds[:, 1] - pos_true[:, 1], color='#00f2fe', alpha=0.9)
    axes[1, 0].plot(t, pos_est_vib[:, 1] - pos_true[:, 1], color='#c084fc', alpha=0.8)
    axes[1, 0].set_ylabel("North Error (Y) [m]")
    axes[1, 0].grid(True, alpha=0.2)
    
    axes[2, 0].plot(t, pos_est_ds[:, 2] - pos_true[:, 2], color='#00f2fe', alpha=0.9)
    axes[2, 0].plot(t, pos_est_vib[:, 2] - pos_true[:, 2], color='#c084fc', alpha=0.8)
    axes[2, 0].set_ylabel("Altitude Error (Z) [m]")
    axes[2, 0].set_xlabel("Time [s]")
    axes[2, 0].grid(True, alpha=0.2)
    
    # Velocities
    axes[0, 1].plot(t, res_ds['vel_est'][:, 0] - res_ds['vel_true'][:, 0], color='#00f2fe', alpha=0.9)
    axes[0, 1].plot(t, res_vib['vel_est'][:, 0] - res_vib['vel_true'][:, 0], color='#c084fc', alpha=0.8)
    axes[0, 1].set_ylabel("Vx Error (East) [m/s]")
    axes[0, 1].grid(True, alpha=0.2)
    axes[0, 1].set_title("Velocity Errors (Estimated - True)")
    
    axes[1, 1].plot(t, res_ds['vel_est'][:, 1] - res_ds['vel_true'][:, 1], color='#00f2fe', alpha=0.9)
    axes[1, 1].plot(t, res_vib['vel_est'][:, 1] - res_vib['vel_true'][:, 1], color='#c084fc', alpha=0.8)
    axes[1, 1].set_ylabel("Vy Error (North) [m/s]")
    axes[1, 1].grid(True, alpha=0.2)
    
    axes[2, 1].plot(t, res_ds['vel_est'][:, 2] - res_ds['vel_true'][:, 2], color='#00f2fe', alpha=0.9)
    axes[2, 1].plot(t, res_vib['vel_est'][:, 2] - res_vib['vel_true'][:, 2], color='#c084fc', alpha=0.8)
    axes[2, 1].set_ylabel("Vz Error (Up) [m/s]")
    axes[2, 1].set_xlabel("Time [s]")
    axes[2, 1].grid(True, alpha=0.2)
    
    fig.suptitle("3D EKF Error Analysis: Estimation Errors vs Time", fontsize=15, fontweight='bold')
    fig.tight_layout()
    plt.savefig(os.path.join(output_dir, "ekf_errors.png"), dpi=150)
    plt.savefig(os.path.join(artifact_dir, "ekf_errors.png"), dpi=150)
    plt.close()
    
    print("\n[Plots] Plots saved successfully in:")
    print(f"  - {os.path.join(output_dir, 'ekf_3d_trajectory.png')}")
    print(f"  - {os.path.join(output_dir, 'ekf_errors.png')}")
    print(f"  - {os.path.join(artifact_dir, 'ekf_3d_trajectory.png')}")
    print(f"  - {os.path.join(artifact_dir, 'ekf_errors.png')}")
