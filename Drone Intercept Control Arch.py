
import subprocess
import socket
import json
import time
import sys
import os
import numpy as np

# ================= PARAMETERS =================
mass = 1.5
g = 9.81
max_thrust_per_motor = 20.0

K_R = np.array([0.05, 0.02, 0.05])
K_w = np.array([0.025, 0.005, 0.025])

# Guidance
Kp = 1.6
Kd = 3.2
NAV_GAIN = 6.5
MAX_ACCEL = 15.5
MAX_TILT_DEG = 35.0
MAX_TILT_RAD = np.radians(MAX_TILT_DEG)

alpha = 0.4
prev_acc_cmd = np.zeros(3)

# ================= KALMAN FILTER =================
class TargetKalman:
    def __init__(self):
        self.x = np.zeros(6)  # [x,y,z,vx,vy,vz]
        self.P = np.eye(6) * 10.0

        q_pos = 0.1
        q_vel = 1.0
        self.Q = np.diag([q_pos, q_pos, q_pos, q_vel, q_vel, q_vel])

        r_pos = 0.5
        self.R = np.eye(3) * r_pos

        self.H = np.zeros((3, 6))
        self.H[:, :3] = np.eye(3)

        self.initialized = False

    def predict(self, dt):
        F = np.eye(6)
        F[0,3] = dt
        F[1,4] = dt
        F[2,5] = dt

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):
        if not self.initialized:
            self.x[:3] = z
            self.initialized = True
            return

        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

    def step(self, z, dt):
        self.predict(dt)
        self.update(z)
        return self.x[:3], self.x[3:]

# ================= UTIL =================
def clamp(x):
    return np.clip(x, 0.0, 1.0)

def vee(M):
    return np.array([M[2,1], M[0,2], M[1,0]])

def euler_to_R(pitch, yaw, roll):
    Rx = np.array([
        [1,0,0],
        [0,np.cos(pitch),-np.sin(pitch)],
        [0,np.sin(pitch), np.cos(pitch)]
    ])

    Ry = np.array([
        [np.cos(yaw),0,-np.sin(yaw)],
        [0,1,0],
        [np.sin(yaw),0,np.cos(yaw)]
    ])

    Rz = np.array([
        [np.cos(roll),-np.sin(roll),0],
        [np.sin(roll), np.cos(roll),0],
        [0,0,1]
    ])

    return Rz @ Ry @ Rx

def read_state(sock, buffer):
    chunk = sock.recv(4096).decode("utf-8")
    buffer += chunk

    while "\n" in buffer:
        line, buffer = buffer.split("\n",1)
        if line.strip():
            return json.loads(line), buffer

    return None, buffer

def guidance(pos, vel, target, target_vel):
    r = target - pos
    v_rel = target_vel - vel

    # Navigation acceleration only
    acc_nav = Kp * r + Kd * v_rel

    dist = np.linalg.norm(r)
    if dist > 10.0:
        r_hat = r / (dist + 1e-6)
        omega_los = np.cross(r, v_rel) / (dist**2 + 1e-6)
        Vc = -np.dot(r, v_rel) / (dist + 1e-6)

        acc_nav += NAV_GAIN * Vc * np.cross(omega_los, r_hat)

    # Limit ONLY navigation acceleration
    nav_norm = np.linalg.norm(acc_nav)
    if nav_norm > MAX_ACCEL:
        acc_nav = acc_nav / nav_norm * MAX_ACCEL

    # Add gravity AFTER limiting
    acc_cmd = acc_nav + np.array([0, g, 0])

    return acc_cmd, r

# ================= MAIN =================
def main():
    global prev_acc_cmd

    last_time = None
    port = 8080 if len(sys.argv) < 2 else int(sys.argv[1])

    script_dir = os.path.dirname(os.path.abspath(__file__))
    sim_exe = os.path.join(script_dir, "build/Release/DroneSimulator.exe")

    print("Starting simulation...")
    sim_process = subprocess.Popen([sim_exe, str(port)])
    time.sleep(3)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(("localhost", port))
    sock.settimeout(0.2)

    print("Connected.")

    buffer = ""

    # Initialize Kalman filter
    kf = TargetKalman()

    try:
        while True:
            state, buffer = read_state(sock, buffer)
            if state is None:
                continue

            pos = np.array(state["position"])
            vel = np.array(state["velocity"])
            target_meas = np.array(state["target_position"])

            pitch, yaw, roll = np.radians(state["rotation"])
            ang_vel = np.array(state["angular_vel_body"])

            now = time.time()
            dt = 0.05 if last_time is None else now - last_time
            last_time = now
            dt = np.clip(dt, 0.005, 0.2)

            # ===== Kalman Filter =====
            target, target_vel = kf.step(target_meas, dt)

            # ===== Guidance =====
            acc_cmd, r = guidance(pos, vel, target, target_vel)

            acc_cmd = (1 - alpha)*prev_acc_cmd + alpha*acc_cmd
            prev_acc_cmd = acc_cmd.copy()

            vertical = acc_cmd[1]

            horizontal = np.array([
                acc_cmd[0],
                0.0,
                acc_cmd[2]
            ])

            hmag = np.linalg.norm(horizontal)

            max_horizontal = abs(vertical) * np.tan(MAX_TILT_RAD)

            if hmag > max_horizontal and hmag > 1e-6:
                horizontal *= max_horizontal / hmag

            acc_cmd = np.array([
                horizontal[0],
                vertical,
                horizontal[2]
            ])

            # ===== Attitude =====
            R = euler_to_R(pitch, yaw, roll)

            b2_des = acc_cmd / (np.linalg.norm(acc_cmd)+1e-6)

            yaw_des = 0.0

            b1_des = np.array([np.cos(yaw_des), 0, np.sin(yaw_des)])

            b3_des = np.cross(b1_des, b2_des)
            b3_des /= np.linalg.norm(b3_des)+1e-6
            b1_des = np.cross(b2_des, b3_des)

            R_d = np.column_stack((b1_des, b2_des, b3_des))

            omega = ang_vel
            e_R = 0.5 * vee(R_d.T @ R - R.T @ R_d)
            e_w = omega

            tau = -K_R * e_R - K_w * e_w

            # ===== Thrust =====
            hover_thrust = mass * g

            thrust_total = hover_thrust + mass * (acc_cmd[1] - g)

            thrust_total = np.clip(
                thrust_total,
                0.0,
                4.0 * max_thrust_per_motor
            )

            thrust_norm = thrust_total / (4.0 * max_thrust_per_motor)

            # ===== Mixer =====
            roll_cmd  = -tau[2]
            pitch_cmd =  tau[0]
            yaw_cmd   =  tau[1]

            FR = thrust_norm - roll_cmd + pitch_cmd - yaw_cmd
            FL = thrust_norm + roll_cmd + pitch_cmd + yaw_cmd
            BR = thrust_norm - roll_cmd - pitch_cmd + yaw_cmd
            BL = thrust_norm + roll_cmd - pitch_cmd - yaw_cmd

            pwm = clamp(np.array([FR, FL, BR, BL]))

            sock.send((json.dumps({"pwm": pwm.tolist()}) + "\n").encode())

    except KeyboardInterrupt:
        print("\nShutting down...")

    finally:
        sock.close()
        sim_process.terminate()
        sim_process.wait()

if __name__ == "__main__":
    main()

