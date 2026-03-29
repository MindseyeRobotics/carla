#!/usr/bin/env python3
"""
px4_carla_bridge.py — CARLA <-> PX4 SITL Hardware-In-the-Loop (HIL) bridge.

Replaces Gazebo as the physics engine for PX4 SITL:
  CARLA (physics) --HIL_SENSOR/HIL_GPS--> PX4 SITL (autopilot)
       ^                                          |
       +---------- HIL_ACTUATOR_CONTROLS ---------+

PX4 SITL thinks it is running inside a hardware-in-the-loop rig. CARLA
provides sensor truth; PX4 computes control outputs; we apply them back
to the CARLA vehicle every sim tick.

Usage:
  python px4_carla_bridge.py [options]

  --host          CARLA server host          (default: localhost)
  --port          CARLA server port          (default: 2000)
  --mavlink       MAVLink connection string  (default: udpin:0.0.0.0:14550)
  --spawn-point   Spawn point index          (default: 0)
  --drone-bp      Blueprint id for vehicle   (default: auto-detect smallest)
  --takeoff-alt   Takeoff altitude in metres (default: 10.0)
  --timeout       Seconds to wait for PX4 HB (default: 30)
  --verbose       Enable debug logging
"""

import argparse
import logging
import math
import signal
import sys
import threading
import time
from typing import List, Optional

try:
    import carla
except ImportError:
    sys.exit("carla Python package not found. Install with: pip install carla==0.9.15")

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink not found. Install with: pip install pymavlink")

log = logging.getLogger("px4_carla_bridge")


# ---------------------------------------------------------------------------
# Sensor data containers (written by CARLA callbacks, read by HIL loop)
# ---------------------------------------------------------------------------

class IMUData:
    """Thread-safe container for the latest IMU reading from CARLA."""

    def __init__(self):
        self._lock = threading.Lock()
        self.accelerometer = (0.0, 0.0, -9.81)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        self.timestamp_us = 0

    def update(self, imu_event):
        with self._lock:
            a = imu_event.accelerometer
            g = imu_event.gyroscope
            # CARLA left-handed (Y right) -> ROS/NED (Y left): negate Y
            self.accelerometer = (a.x, -a.y, a.z)
            self.gyroscope = (g.x, -g.y, g.z)
            self.compass = imu_event.compass
            self.timestamp_us = int(imu_event.timestamp * 1e6)

    def snapshot(self):
        with self._lock:
            return self.accelerometer, self.gyroscope, self.compass, self.timestamp_us


class GNSSData:
    """Thread-safe container for the latest GNSS reading from CARLA."""

    def __init__(self):
        self._lock = threading.Lock()
        # Default: Zurich airfield (matches PX4 SITL default home)
        self.lat = 47.397742
        self.lon = 8.545594
        self.alt = 488.0
        self.timestamp_us = 0

    def update(self, gnss_event):
        with self._lock:
            self.lat = gnss_event.latitude
            self.lon = gnss_event.longitude
            self.alt = gnss_event.altitude
            self.timestamp_us = int(gnss_event.timestamp * 1e6)

    def snapshot(self):
        with self._lock:
            return self.lat, self.lon, self.alt, self.timestamp_us


# ---------------------------------------------------------------------------
# MAVLink HIL bridge
# ---------------------------------------------------------------------------

class HILBridge:
    """
    Manages the MAVLink HIL link to PX4 SITL.

    - Publishes HIL_SENSOR + HIL_GPS at ~50 Hz
    - Receives HIL_ACTUATOR_CONTROLS and exposes them via consume_controls()
    - Sends heartbeats at 2 Hz
    """

    HIL_RATE_HZ = 50
    HB_RATE_HZ = 2

    def __init__(self, conn_str: str, imu: IMUData, gnss: GNSSData, verbose: bool = False):
        self._conn_str = conn_str
        self._imu = imu
        self._gnss = gnss
        self._verbose = verbose

        self._mav = None
        self._running = False

        self._act_controls: List[float] = [0.0] * 16
        self._act_armed = False
        self._new_controls = False
        self._ctrl_lock = threading.Lock()

        self.connected = False
        self.px4_sys = 1
        self.px4_comp = 1

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def connect(self, timeout: float = 30.0) -> bool:
        log.info("Connecting to PX4 SITL at %s ...", self._conn_str)
        self._mav = mavutil.mavlink_connection(
            self._conn_str,
            source_system=255,
            source_component=190,
        )
        # Announce ourselves so PX4 learns our UDP return address
        for _ in range(5):
            self._mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )
            time.sleep(0.2)

        log.info("Waiting for PX4 heartbeat (timeout %.0f s) ...", timeout)
        hb = self._mav.wait_heartbeat(timeout=timeout)
        if hb is None:
            log.error("No PX4 heartbeat received — is SITL running?")
            return False

        self.px4_sys = self._mav.target_system
        self.px4_comp = self._mav.target_component
        log.info("PX4 connected — sys=%d comp=%d", self.px4_sys, self.px4_comp)
        self.connected = True
        return True

    # ------------------------------------------------------------------
    # Background threads
    # ------------------------------------------------------------------

    def start(self):
        self._running = True
        threading.Thread(target=self._heartbeat_loop, daemon=True, name="hil-hb").start()
        threading.Thread(target=self._sensor_loop,    daemon=True, name="hil-sensor").start()
        threading.Thread(target=self._receive_loop,   daemon=True, name="hil-rx").start()
        log.info("HIL threads started (sensor @ %d Hz, HB @ %d Hz)", self.HIL_RATE_HZ, self.HB_RATE_HZ)

    def stop(self):
        self._running = False

    def _heartbeat_loop(self):
        interval = 1.0 / self.HB_RATE_HZ
        while self._running:
            try:
                self._mav.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0,
                )
            except Exception as exc:
                log.warning("HB send failed: %s", exc)
            time.sleep(interval)

    def _sensor_loop(self):
        interval = 1.0 / self.HIL_RATE_HZ
        while self._running:
            try:
                self._send_hil_sensor()
                self._send_hil_gps()
            except Exception as exc:
                log.warning("HIL sensor send error: %s", exc)
            time.sleep(interval)

    def _receive_loop(self):
        while self._running:
            try:
                msg = self._mav.recv_match(blocking=True, timeout=0.1)
                if msg is None:
                    continue
                mtype = msg.get_type()
                if mtype == "HIL_ACTUATOR_CONTROLS":
                    with self._ctrl_lock:
                        self._act_controls = list(msg.controls)
                        # ARMED flag lives in bit 7 of msg.mode
                        self._act_armed = bool(msg.mode & 0x80)
                        self._new_controls = True
                    if self._verbose:
                        log.debug(
                            "ACT: roll=%.3f pitch=%.3f yaw=%.3f thr=%.3f armed=%s",
                            msg.controls[0], msg.controls[1], msg.controls[2], msg.controls[3],
                            self._act_armed,
                        )
                elif mtype == "COMMAND_ACK" and self._verbose:
                    log.debug("CMD_ACK: cmd=%d result=%d", msg.command, msg.result)
            except Exception as exc:
                log.debug("MAVLink recv error: %s", exc)

    # ------------------------------------------------------------------
    # HIL messages
    # ------------------------------------------------------------------

    def _send_hil_sensor(self):
        accel, gyro, compass, ts_us = self._imu.snapshot()
        ts = ts_us if ts_us else int(time.time() * 1e6)

        # Simplified magnetic field from compass heading
        mag_x = math.cos(math.radians(compass))
        mag_y = math.sin(math.radians(compass))
        mag_z = 0.4  # gauss (typical vertical component)

        # fields_updated = 0x1FFF means all sensor fields updated
        self._mav.mav.hil_sensor_send(
            ts,
            float(accel[0]), float(accel[1]), float(accel[2]),   # acc  m/s^2
            float(gyro[0]),  float(gyro[1]),  float(gyro[2]),    # gyro rad/s
            float(mag_x),   float(mag_y),    float(mag_z),       # mag  gauss
            1013.25,   # abs_pressure  mbar
            0.0,       # diff_pressure mbar
            488.0,     # pressure_alt  m  (matches SITL home alt)
            25.0,      # temperature   C
            0x1FFF,    # fields_updated: all
            0,         # sensor id
        )

    def _send_hil_gps(self):
        lat, lon, alt, ts_us = self._gnss.snapshot()
        ts = ts_us if ts_us else int(time.time() * 1e6)

        self._mav.mav.hil_gps_send(
            ts,
            3,                    # fix_type: 3D fix
            int(lat * 1e7),       # lat  degE7
            int(lon * 1e7),       # lon  degE7
            int(alt * 1000),      # alt  mm MSL
            100,                  # eph  cm  (1 m accuracy)
            100,                  # epv  cm
            0,                    # vel  cm/s
            0,                    # vn   cm/s
            0,                    # ve   cm/s
            0,                    # vd   cm/s
            0,                    # cog  cdeg
            10,                   # satellites_visible
            0,                    # id
        )

    # ------------------------------------------------------------------
    # Control output
    # ------------------------------------------------------------------

    def consume_controls(self):
        """Return (throttle, roll, pitch, yaw, armed, new_flag) and clear flag."""
        with self._ctrl_lock:
            new = self._new_controls
            self._new_controls = False
            c = self._act_controls
            return c[3], c[0], c[1], c[2], self._act_armed, new

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    def arm(self, force: bool = True):
        param2 = 21196.0 if force else 0.0
        self._mav.mav.command_long_send(
            self.px4_sys, self.px4_comp,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1.0, param2, 0, 0, 0, 0, 0,
        )
        log.info("Arm command sent (force=%s)", force)

    def takeoff(self, altitude_m: float):
        # Switch to AUTO mode first
        self._mav.mav.set_mode_send(
            self.px4_sys,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            (4 << 16) | (0 << 24),  # AUTO main=4
        )
        time.sleep(0.15)
        self._mav.mav.command_long_send(
            self.px4_sys, self.px4_comp,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0,
            float("nan"),   # yaw: keep current
            float("nan"),   # lat: current
            float("nan"),   # lon: current
            float(altitude_m),
        )
        log.info("Takeoff command sent — target %.1f m", altitude_m)

    def set_hil_mode(self):
        """Tell PX4 to enter HIL mode (required before HIL_SENSOR is accepted)."""
        self._mav.mav.param_set_send(
            self.px4_sys, self.px4_comp,
            b"HIL_STATE",
            1.0,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        )
        # Also disable RC loss so pure-MAVLink operation works
        for name, value in [
            (b"COM_RCL_EXCEPT", 4),
            (b"COM_RC_IN_MODE", 1),
            (b"CBRK_ARMING_CHK", 162128),
        ]:
            self._mav.mav.param_set_send(
                self.px4_sys, self.px4_comp,
                name, float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_INT32,
            )
            time.sleep(0.05)
        log.info("HIL mode and safety params set")


# ---------------------------------------------------------------------------
# CARLA control mapping
# ---------------------------------------------------------------------------

def px4_to_carla_control(throttle: float, roll: float,
                         pitch: float, yaw: float) -> "carla.VehicleControl":
    """
    Map PX4 normalized actuator outputs to a CARLA VehicleControl.

    Ground-vehicle proxy mapping (CARLA 0.9.x has no native multirotor BP):
      throttle [0,1]  -> ctrl.throttle
      roll    [-1,1]  -> ctrl.steer   (positive roll = right)
      pitch   [-1,1]  -> forward bias (negative pitch = forward in NED)
      yaw              -> ignored for ground proxy

    For a true drone simulation, replace this with set_target_velocity().
    """
    ctrl = carla.VehicleControl()
    ctrl.throttle   = max(0.0, min(1.0, float(throttle)))
    ctrl.steer      = max(-1.0, min(1.0, float(roll)))
    ctrl.brake      = 0.0
    ctrl.hand_brake = False
    ctrl.reverse    = float(pitch) > 0.3  # positive pitch -> slow / reverse
    return ctrl


# ---------------------------------------------------------------------------
# Vehicle + sensor setup
# ---------------------------------------------------------------------------

PREFERRED_BPS = [
    "vehicle.micro.microlino",
    "vehicle.carlamotors.firetruck",   # fallback: smallest available
    "vehicle.tesla.model3",
    "vehicle.*",
]


def find_vehicle_bp(world: "carla.World", preferred: Optional[str]) -> "carla.ActorBlueprint":
    lib = world.get_blueprint_library()

    if preferred:
        matches = lib.filter(preferred)
        if matches:
            bp = matches[0]
            log.info("Using blueprint: %s", bp.id)
            return bp
        log.warning("Blueprint '%s' not found — trying fallbacks", preferred)

    for pattern in PREFERRED_BPS:
        matches = lib.filter(pattern)
        if matches:
            bp = matches[0]
            log.info("Using blueprint: %s", bp.id)
            return bp

    raise RuntimeError("No vehicle blueprint found in CARLA world")


def attach_imu(world: "carla.World", vehicle: "carla.Actor",
               imu_data: IMUData) -> "carla.Actor":
    lib = world.get_blueprint_library()
    bp = lib.find("sensor.other.imu")
    bp.set_attribute("sensor_tick", "0.02")  # 50 Hz
    transform = carla.Transform(carla.Location(x=0, y=0, z=0))
    sensor = world.spawn_actor(bp, transform, attach_to=vehicle)
    sensor.listen(imu_data.update)
    log.info("IMU sensor attached")
    return sensor


def attach_gnss(world: "carla.World", vehicle: "carla.Actor",
                gnss_data: GNSSData) -> "carla.Actor":
    lib = world.get_blueprint_library()
    bp = lib.find("sensor.other.gnss")
    bp.set_attribute("sensor_tick", "0.1")  # 10 Hz
    transform = carla.Transform(carla.Location(x=0, y=0, z=0))
    sensor = world.spawn_actor(bp, transform, attach_to=vehicle)
    sensor.listen(gnss_data.update)
    log.info("GNSS sensor attached")
    return sensor


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description="CARLA <-> PX4 SITL HIL bridge")
    p.add_argument("--host",        default=os.environ.get("CARLA_HOST", "localhost"))
    p.add_argument("--port",        default=int(os.environ.get("CARLA_PORT", "2000")), type=int)
    p.add_argument("--mavlink",     default=os.environ.get("MAVLINK_CONNECTION", "udpin:0.0.0.0:14550"))
    p.add_argument("--spawn-point", default=0, type=int, dest="spawn_point")
    p.add_argument("--drone-bp",    default=os.environ.get("DRONE_BP", None), dest="drone_bp")
    p.add_argument("--takeoff-alt", default=float(os.environ.get("TAKEOFF_ALT", "10.0")), type=float, dest="takeoff_alt")
    p.add_argument("--timeout",     default=30, type=int)
    p.add_argument("--verbose",     action="store_true")
    return p.parse_args()


def main():
    args = parse_args()

    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        format="[%(asctime)s] %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
        level=level,
    )

    vehicle: Optional[carla.Actor] = None
    sensors: List[carla.Actor] = []
    world: Optional[carla.World] = None
    original_settings = None
    hil: Optional[HILBridge] = None

    def cleanup(signum=None, frame=None):
        nonlocal hil
        log.info("Cleaning up ...")
        if hil:
            hil.stop()
        if original_settings and world:
            world.apply_settings(original_settings)
        for s in sensors:
            try:
                s.destroy()
            except Exception:
                pass
        if vehicle:
            try:
                vehicle.destroy()
            except Exception:
                pass
        log.info("Done. Bye!")
        sys.exit(0)

    signal.signal(signal.SIGINT,  cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # ------------------------------------------------------------------
    # Connect to CARLA
    # ------------------------------------------------------------------
    log.info("Connecting to CARLA at %s:%d ...", args.host, args.port)
    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)
    world = client.get_world()
    log.info("Connected to CARLA — map: %s", world.get_map().name)

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.02  # 50 Hz sim tick
    world.apply_settings(settings)
    log.info("Synchronous mode enabled at 50 Hz")

    # Traffic manager must also be sync
    tm = client.get_trafficmanager()
    tm.set_synchronous_mode(True)

    # ------------------------------------------------------------------
    # Spawn vehicle
    # ------------------------------------------------------------------
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        cleanup()
        raise RuntimeError("No spawn points found in map")

    idx = min(args.spawn_point, len(spawn_points) - 1)
    spawn_tf = spawn_points[idx]

    bp = find_vehicle_bp(world, args.drone_bp)
    if bp.has_attribute("role_name"):
        bp.set_attribute("role_name", "px4_drone")

    vehicle = world.spawn_actor(bp, spawn_tf)
    log.info("Vehicle spawned: %s at %s", bp.id, spawn_tf.location)

    # ------------------------------------------------------------------
    # Attach sensors
    # ------------------------------------------------------------------
    imu_data  = IMUData()
    gnss_data = GNSSData()
    sensors.append(attach_imu(world, vehicle, imu_data))
    sensors.append(attach_gnss(world, vehicle, gnss_data))

    # Warm up: tick a few times so sensor callbacks fire
    for _ in range(5):
        world.tick()
    log.info("Sensor warm-up complete")

    # ------------------------------------------------------------------
    # Connect to PX4 SITL via MAVLink HIL
    # ------------------------------------------------------------------
    hil = HILBridge(args.mavlink, imu_data, gnss_data, verbose=args.verbose)
    if not hil.connect(timeout=args.timeout):
        cleanup()
        sys.exit(1)

    hil.set_hil_mode()
    hil.start()

    # Allow HIL sensor stream to stabilise before arming
    log.info("Streaming HIL sensors for 5 s before arm ...")
    start = time.time()
    while time.time() - start < 5.0:
        world.tick()
        time.sleep(0.02)

    # ------------------------------------------------------------------
    # Arm and takeoff
    # ------------------------------------------------------------------
    hil.arm(force=True)
    time.sleep(1.0)
    hil.takeoff(args.takeoff_alt)
    log.info("Arm + takeoff commands sent — now running main loop")

    # ------------------------------------------------------------------
    # Main simulation loop
    # ------------------------------------------------------------------
    tick_count = 0
    try:
        while True:
            world.tick()
            tick_count += 1

            # Poll for new actuator controls from PX4
            throttle, roll, pitch, yaw, armed, new = hil.consume_controls()

            if new and vehicle and vehicle.is_alive:
                ctrl = px4_to_carla_control(throttle, roll, pitch, yaw)
                vehicle.apply_control(ctrl)
                if args.verbose and tick_count % 50 == 0:
                    log.debug(
                        "tick=%d ctrl: thr=%.2f steer=%.2f armed=%s",
                        tick_count, ctrl.throttle, ctrl.steer, armed,
                    )

    except KeyboardInterrupt:
        pass
    finally:
        cleanup()


if __name__ == "__main__":
    import os
    main()
