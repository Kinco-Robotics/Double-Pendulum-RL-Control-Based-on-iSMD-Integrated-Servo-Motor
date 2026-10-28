import argparse
import os
import sys
import time
import numpy as np
import torch

_HARDWARE_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..",
    "source", "iSMD60D_DoublePendulum_3DP", "iSMD60D_DoublePendulum_3DP", "hardware",
)
sys.path.insert(0, os.path.abspath(_HARDWARE_DIR))
from mcontrol_driver import MControl, SerialConfig

_OBS_PER_FRAME = 7
_OBS_HISTORY_LEN = 3
OBS_DIM = _OBS_PER_FRAME * _OBS_HISTORY_LEN
ACT_DIM = 1

SERIAL_PORT = "COM3"
SERIAL_STATION = 127

TORQUE_SCALE = 3.4
CONTROL_FREQ = 90
CONTROL_DT = 1.0 / CONTROL_FREQ
NUM_JOINTS = 2
VEL_SCALE = (1875.0 / 512.0) * (np.pi / 30.0)
ENCODER_RES_J1 = 65536  # Joint1 (lower arm) encoder pulses per revolution
ENCODER_RES_J2 = 10000  # Joint2 (upper arm) encoder pulses per revolution
REG_J1_POS = 0x606300
REG_J2_POS = 0x250A07
REG_J1_VEL = 0x606C00
REG_ENCODER_RESET = 0x269000
REG_DEVICE_RESTART = 0x2FFF00
REG_OP_MODE = 0x606000
REG_MAX_CURRENT = 0x651003
REG_TORQUE_COEFF = 0x64100F
REG_MAX_SPEED = 0x607F00
REG_CONTROL_WORD = 0x604000
REG_CMD_Q = 0x60F608
REG_PULSE_MODE = 0x250803
REG_SAVE_PARAMETERS = 0x101001
REG_COMM232_BAUDRATE = 0x2FE000
REG_COMM232_TIMEOUTGUARD = 0x2FF024
REG_TARGET_CURRENTLIMIT = 0x607300

def _is_preprocessor_key(key: str) -> bool:
    return ("observation_preprocessor" in key) or ("state_preprocessor" in key)


class PolicyNetwork(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.net = torch.nn.Sequential(
            torch.nn.Linear(OBS_DIM, 32),
            torch.nn.ELU(),
            torch.nn.Linear(32, 32),
            torch.nn.ELU(),
        )
        self.mean = torch.nn.Linear(32, ACT_DIM)

    def forward(self, obs):
        features = self.net(obs)
        return self.mean(features)


def load_policy(checkpoint_path: str) -> tuple[PolicyNetwork, dict | None]:
    checkpoint = torch.load(checkpoint_path, map_location="cpu")

    policy = PolicyNetwork()
    policy.eval()

    key_map = {
        "net_container.0.weight": "net.0.weight",
        "net_container.0.bias":   "net.0.bias",
        "net_container.2.weight": "net.2.weight",
        "net_container.2.bias":   "net.2.bias",
        "net_container.4.weight": "mean.weight",
        "net_container.4.bias":   "mean.bias",
        "mean_action_actor.weight": "mean.weight",
        "mean_action_actor.bias":   "mean.bias",
        "policy_layer.weight": "mean.weight",
        "policy_layer.bias":   "mean.bias",
    }

    ckpt_data = checkpoint.get("policy", checkpoint)
    if "policy" in checkpoint:
        print("[INFO] Detected skrl v2.x checkpoint format")

    loaded = {}
    obs_preprocessor = None
    log_std_val = None
    for skrl_key, tensor in ckpt_data.items():
        if skrl_key in key_map:
            loaded[key_map[skrl_key]] = tensor
        elif skrl_key in ("log_std_actor", "log_std_parameter"):
            log_std_val = tensor.numpy()
        elif _is_preprocessor_key(skrl_key):
            if obs_preprocessor is None:
                obs_preprocessor = {}
            obs_preprocessor[skrl_key] = tensor

    if obs_preprocessor is None:
        for skrl_key, tensor in checkpoint.items():
            if _is_preprocessor_key(skrl_key):
                if obs_preprocessor is None:
                    obs_preprocessor = {}
                obs_preprocessor[skrl_key] = tensor

    if log_std_val is not None:
        print(f"[INFO] log_std = {log_std_val}")

    policy.load_state_dict(loaded, strict=False)
    expected_policy_keys = {"net.0.weight", "net.0.bias", "net.2.weight", "net.2.bias", "mean.weight", "mean.bias"}
    actual_policy_keys = set(policy.state_dict().keys())
    not_loaded = [k for k in expected_policy_keys if k not in actual_policy_keys]
    no_source = [k for k in expected_policy_keys if k not in loaded]
    if not_loaded or no_source:
        print(f"[WARN] Policy parameters not loaded: {not_loaded}")
        print(f"[WARN] No corresponding source keys: {no_source}")
        print("[WARN] Policy will run with random weights, deployment outcome is unpredictable！")
    else:
        print("[INFO] All policy parameters loaded successfully")
    print(f"[INFO] Policy network loaded | "
          f"Input {OBS_DIM} dims → Hidden [32,32] → Output {ACT_DIM} dims")
    if obs_preprocessor:
        print(f"[INFO] Observation normalization parameters loaded")
    return policy, obs_preprocessor


class ObservationHistory:
    def __init__(self):
        self._buf = np.zeros((_OBS_HISTORY_LEN, _OBS_PER_FRAME), dtype=np.float32)
        self._idx = 0

    def reset(self, frame: np.ndarray):
        for i in range(_OBS_HISTORY_LEN):
            self._buf[i] = frame
        self._idx = 0

    def push(self, frame: np.ndarray):
        self._buf[self._idx] = frame
        self._idx = (self._idx + 1) % _OBS_HISTORY_LEN

    def get(self) -> np.ndarray:
        idxs = [(self._idx + i) % _OBS_HISTORY_LEN for i in range(_OBS_HISTORY_LEN)]
        frames = self._buf[idxs]
        sincos = frames[:, :4].flatten()
        vel = frames[:, 4:6].flatten()
        actions = frames[:, 6:7].flatten()
        return np.concatenate([sincos, vel, actions])


def build_observation_frame(joint_pos: np.ndarray, joint_vel: np.ndarray,
                      last_action: np.ndarray) -> np.ndarray:
    joint_pos = np.nan_to_num(joint_pos, nan=0.0, posinf=0.0, neginf=0.0)
    joint_vel = np.nan_to_num(joint_vel, nan=0.0, posinf=0.0, neginf=0.0)
    sin_pos = np.sin(joint_pos)
    cos_pos = np.cos(joint_pos)
    return np.concatenate([sin_pos, cos_pos, joint_vel, last_action]).astype(np.float32)


class RunningStandardScaler:
    def __init__(self, state_dict: dict | None):
        if state_dict is None:
            self._mean = np.zeros(OBS_DIM, dtype=np.float32)
            self._std = np.ones(OBS_DIM, dtype=np.float32)
            self._enabled = False
            return

        self._enabled = True
        self._mean = np.zeros(OBS_DIM, dtype=np.float32)
        self._std = np.ones(OBS_DIM, dtype=np.float32)

        def _extract(d, prefix=""):
            for key, val in d.items():
                full_key = f"{prefix}.{key}" if prefix else key
                if isinstance(val, dict):
                    _extract(val, full_key)
                elif hasattr(val, "cpu"):
                    arr = val.cpu().numpy().flatten()
                    _apply(full_key, arr)
                elif isinstance(val, np.ndarray):
                    _apply(full_key, val.flatten())

        def _apply(key, arr):
            if "mean" in key:
                self._mean[:] = arr[:OBS_DIM]
            elif "var" in key:
                self._std[:] = np.sqrt(arr[:OBS_DIM] + 1e-8)
            elif "standard_deviation" in key:
                self._std[:] = arr[:OBS_DIM]

        _extract(state_dict)

        has_nan = np.any(np.isnan(self._mean)) or np.any(np.isnan(self._std))
        if has_nan or np.any(self._std == 0):
            print("[WARN] Normalization parameters invalid, skipping normalization")
            self._enabled = False
            self._mean = np.zeros(OBS_DIM, dtype=np.float32)
            self._std = np.ones(OBS_DIM, dtype=np.float32)

        if self._enabled:
            print(f"[INFO] Observation normalization mean: {self._mean}")
            print(f"[INFO] Observation normalization standard deviation: {self._std}")

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        if not self._enabled:
            return obs
        return (obs - self._mean) / self._std


class JointHardwareInterface:
    def __init__(self, joint1_offset_deg: float = 0.0, joint2_offset_deg: float = 0.0):
        self._joint1_offset_deg = joint1_offset_deg
        self._joint2_offset_deg = joint2_offset_deg
        self._ctrl = None
        self._prev_pos = None
        self._initial_p0_deg = None
        self._initial_p1_deg = None
        self._safety_triggered = False
        self._torque_to_cmd = 0.0
        self._last_write_time = None
        self._joint_pos = np.zeros(NUM_JOINTS, dtype=np.float64)
        self._joint_vel = np.zeros(NUM_JOINTS, dtype=np.float64)

    @property
    def safety_triggered(self) -> bool:
        return self._safety_triggered

    def connect(self):
        self._ctrl = None
        code = None
        for baud in (1000000, 500000, 115200, 38400, 19200, 9600):       
            try:
                c = MControl()
                c.open_serial(SerialConfig(port=SERIAL_PORT, baudrate=baud, station=SERIAL_STATION))
                code = c.read_int(REG_COMM232_BAUDRATE, "Uint16")  # Read baud rate  
                print(f"[INFO] link OK @ {baud}, code={code}")
                self._ctrl = c
                break
            except Exception as e:
                print(f"[WARN ] {SERIAL_PORT} @ {baud}: {type(e).__name__}: {e}")
                try: c.close()
                except Exception: pass
        if self._ctrl is None:
            raise RuntimeError(f"Failed to connect to {SERIAL_PORT} (tried 1000000/500000/115200/38400/19200/9600, station {SERIAL_STATION})")

        if code != 10:                          
            self._ctrl.write_int(REG_COMM232_BAUDRATE, "Uint16", 10)  # Set baud rate to 1 Mbps  
            self._ctrl.write_int(REG_SAVE_PARAMETERS, "Uint32", 0x65766173)  # Save parameters  
            time.sleep(3)                            
            self._ctrl.write_int(REG_DEVICE_RESTART, "Uint16", 0xAA55)  # Reboot drive      
            time.sleep(3)
            self._ctrl.close()
            self._ctrl = MControl()
            self._ctrl.open_serial(SerialConfig(port=SERIAL_PORT, baudrate=1000000, station=SERIAL_STATION))

        self._ctrl.write_int(REG_ENCODER_RESET, "Uint8", 10)  # Encoder reset
        self._ctrl.write_int(REG_DEVICE_RESTART, "Uint16", 0xAA55)
        time.sleep(3.0)

        pulse_mode = self._ctrl.read_int(REG_PULSE_MODE, "Uint8")
        if pulse_mode != 2:
            self._ctrl.write_int(REG_PULSE_MODE, "Uint8", 2)  # Set pulse mode to 2 to obtain the Joint2 (upper arm) angle
            self._ctrl.write_int(REG_SAVE_PARAMETERS, "Uint32", 0x65766173)
            time.sleep(3.0)
            self._ctrl.write_int(REG_DEVICE_RESTART, "Uint16", 0xAA55)
            time.sleep(3.0)
            pulse_mode = self._ctrl.read_int(REG_PULSE_MODE, "Uint8")
            if pulse_mode != 2:
                print(f"[WARN] 250803 Save failed, current value: {pulse_mode}")
        else:
            print(f"[INFO] Pulse mode already 2, skipping save")
        

        self._ctrl.write_int(REG_OP_MODE, "Uint8", 4)  # Set current mode

        self._i_max = self._ctrl.read_int(REG_MAX_CURRENT, "Uint16")  # Read max current
        self._kt_raw = self._ctrl.read_int(REG_TORQUE_COEFF, "Int16")  # Read torque coefficient
        self._torque_to_cmd = 141.42135624 * 20480.0 / (self._i_max * self._kt_raw)

        speed_raw = int(120 * 512 * ENCODER_RES_J1 / 1875)
        self._ctrl.write_int(REG_MAX_SPEED, "Uint32", speed_raw)  # Set max speed to 120 RPM

        self._current_raw = int(9 * 2048 / (self._i_max / 10))
        self._ctrl.write_int(REG_TARGET_CURRENTLIMIT, "Uint16", self._current_raw)  # Set target current limit to 9 Ap

        raw_p0 = self._ctrl.read_int(REG_J1_POS, "Int32")  # Read Joint1 (lower arm) initial angle
        raw_p1 = self._ctrl.read_int(REG_J2_POS, "Int32")  # Read Joint2 (upper arm) initial angle
        self._initial_p0_deg = (raw_p0 / ENCODER_RES_J1) * 360
        self._initial_p1_deg = (raw_p1 / ENCODER_RES_J2) * 360

        self._ctrl.write_int(REG_CONTROL_WORD, "Uint16", 0x06)  # Disable
        self._ctrl.write_int(REG_CONTROL_WORD, "Uint16", 0x0F)  # Enable

        # 232 communication timeout protection (unit: ms, default 0 = off, max 5000).
        # If no data frame is received on the 232 serial port within the set time,
        # the drive will automatically be disabled.
        self._ctrl.write_int(REG_COMM232_TIMEOUTGUARD, "Uint16", 200)

    def write_command(self, target_torque: float) -> tuple[np.ndarray, np.ndarray]:
        if not np.isfinite(target_torque):
            target_torque = 0.0

        joint_pos = self._joint_pos
        joint_vel = self._joint_vel
        assert self._ctrl is not None

        cmd_q = int(np.sign(target_torque) * np.floor(abs(target_torque * self._torque_to_cmd) + 0.5))
        cmd_q = int(np.clip(cmd_q, -self._current_raw, self._current_raw))
        self._ctrl.write_int(REG_CMD_Q, "Int16", cmd_q)  # Send target current (driver-internal unit)

        raw_p0 = self._ctrl.read_int(REG_J1_POS, "Int32")
        raw_p0_deg = (raw_p0 / ENCODER_RES_J1) * 360
        actual_p1 = self._ctrl.read_int(REG_J2_POS, "Int32")
        actual_p1_deg = (actual_p1 / ENCODER_RES_J2) * 360

        raw_vel = self._ctrl.read_int(REG_J1_VEL, "Int32")  # Read Joint1 (lower arm) actual velocity (driver-internal unit)
        joint_pos[0] = np.radians(raw_p0_deg - self._initial_p0_deg + self._joint1_offset_deg)
        joint_pos[1] = np.radians(actual_p1_deg - self._initial_p1_deg + self._joint2_offset_deg)
        joint_vel[0] = raw_vel * VEL_SCALE / ENCODER_RES_J1  # Convert Joint1 velocity to rad/s
        now = time.perf_counter()
        if self._prev_pos is not None and self._last_write_time is not None:
            dt = now - self._last_write_time
            joint_vel[1] = (joint_pos[1] - self._prev_pos[1]) / dt
        self._prev_pos = joint_pos.copy()
        self._last_write_time = now

        if np.abs(joint_pos[0]) > np.radians(80):
            self._ctrl.write_int(REG_CONTROL_WORD, "Uint16", 0x06)
            if not self._safety_triggered:
                self._safety_triggered = True
                
        return joint_pos, joint_vel

    def disconnect(self):
        assert self._ctrl is not None
        self._ctrl.write_int(REG_CONTROL_WORD, "Uint16", 0x06)  # Disable
        self._ctrl.close()
        print("[INFO] Motor disabled, session closed")


def parse_arguments():
    parser = argparse.ArgumentParser(description="Deploy the trained PPO model to iSMD60D")
    parser.add_argument("--checkpoint", type=str, required=True,
                        help="Model checkpoint file path (.pt)")
    parser.add_argument("--joint1-offset", type=float, default=0.0,
                        help="Initial angle offset for joint1 (deg)")
    parser.add_argument("--joint2-offset", type=float, default=0.0,
                        help="Encoder offset for joint2 (deg)")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Run duration (s), 0 for infinite")
    return parser.parse_args()


def main():
    args = parse_arguments()

    checkpoint_path = os.path.abspath(args.checkpoint)
    if not os.path.exists(checkpoint_path):
        print(f"[ERROR] Checkpoint does not exist: {checkpoint_path}")
        return 1

    print(f"[INFO] Loading checkpoint: {checkpoint_path}")
    policy, preprocessor_state = load_policy(checkpoint_path)

    normalizer = RunningStandardScaler(preprocessor_state)

    hardware = JointHardwareInterface(
        joint1_offset_deg=args.joint1_offset,
        joint2_offset_deg=args.joint2_offset,
    )
    try:
        hardware.connect()
    except Exception as e:
        print(f"[ERROR] Hardware initialization failed: {e}")
        return 1

    real_elapsed_sec = 0.0
    last_action = np.zeros(ACT_DIM, dtype=np.float32)

    try:
        joint_pos, joint_vel = hardware.write_command(0.0)
        obs_history = ObservationHistory()
        frame = build_observation_frame(joint_pos, joint_vel, last_action)
        obs_history.reset(frame)

        real_start = time.perf_counter()
        last_control_time = real_start
        while True:
            current_time = time.perf_counter()
            real_elapsed_sec = current_time - real_start
            control_flag = (current_time - last_control_time) >= CONTROL_DT

            if (args.duration > 0) and (real_elapsed_sec >= args.duration):
                break

            if control_flag:
                frame = build_observation_frame(joint_pos, joint_vel, last_action)
                obs_history.push(frame)
                obs_21d = obs_history.get()
                obs = normalizer(obs_21d)

                with torch.inference_mode():
                    obs_tensor = torch.from_numpy(obs).unsqueeze(0)
                    action_tensor = policy(obs_tensor)
                    action_raw = action_tensor.squeeze(0).cpu().numpy()

                action = np.clip(action_raw, -1.0, 1.0)

                target_torque = action[0] * TORQUE_SCALE

                joint_pos, joint_vel = hardware.write_command(target_torque)
                if hardware.safety_triggered:
                    break

                last_action = action_raw

                last_control_time = current_time
            else:
                pass

    except KeyboardInterrupt:
        print("\n[INFO] User interrupted (Ctrl+C)")

    finally:
        print(f"[INFO] Time: {real_elapsed_sec:.1f}s")
        hardware.disconnect()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
