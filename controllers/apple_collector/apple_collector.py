"""Stage 1: TurtleBot3Burger drive/sensor checkout (not the apple mission yet).

Run from Webots, not ordinary Python. No third-party packages are needed.
Default: a bounded automatic checkout; use controllerArgs ["--manual"] for idle.
Click the Webots 3D view for keyboard control:
  W/S: hold to drive; A/D: hold to turn; M: manual/stop;
  Space/X: latched emergency stop; R: release into idle manual mode;
  T: restart checkout. A motion key cancels checkout.
Outputs (beside this script): logs/stage1_latest.jsonl and stage1_camera.png.
The log is overwritten on each simulation reset. It contains sensor/command
evidence only: encoder motion does NOT prove collision-free physical travel.
"""

import argparse
import json
import math
from pathlib import Path


WHEEL_RADIUS = 0.033
AXLE_LENGTH = 0.160
DRIVE_SPEED = 0.066           # m/s, deliberately slow for the first checkout
TURN_SPEED = 0.55            # rad/s
STOP_DISTANCE = 0.26         # from robot origin, not from body surface
SLOW_DISTANCE = 0.55
CORRIDOR_HALF_WIDTH = 0.16  # swept body width plus a conservative margin
ROTATION_CLEARANCE = 0.16
LIDAR_X = -0.03             # TurtleBot extension slot relative to robot origin


def wheel_speeds(linear, angular, limit):
    left = (linear - 0.5 * AXLE_LENGTH * angular) / WHEEL_RADIUS
    right = (linear + 0.5 * AXLE_LENGTH * angular) / WHEEL_RADIUS
    scale = max(1.0, abs(left) / limit, abs(right) / limit)
    return left / scale, right / scale


def scan_geometry(ranges, maximum):
    """LDS-01: back=0, left=n/4, front=n/2, right=3n/4.

    Positive infinity is a valid no-return ray. NaN, -inf and nonpositive
    ranges are invalid. A completely empty scan is conservatively rejected.
    """
    if len(ranges) < 16:
        return None
    if any(math.isnan(r) or r <= 0.0 for r in ranges):
        return None
    points = []
    for i, distance in enumerate(ranges):
        if math.isfinite(distance) and distance < maximum:
            angle = math.pi - 2.0 * math.pi * i / len(ranges)
            points.append((LIDAR_X + distance * math.cos(angle),
                           distance * math.sin(angle)))
    if not points:
        return None
    front = min((x for x, y in points
                 if x >= 0.0 and abs(y) <= CORRIDOR_HALF_WIDTH), default=maximum)
    rear = min((-x for x, y in points
                if x < 0.0 and abs(y) <= CORRIDOR_HALF_WIDTH), default=maximum)
    around = min(math.hypot(x, y) for x, y in points)
    return {"front": front, "rear": rear, "around": around}


def protect_motion(linear, angular, scan):
    """Stage-1 stop guard, NOT a complete obstacle-avoidance planner.

    LiDAR only observes its scan plane. Low objects and drop-offs are not
    covered. Never promise collision-free operation from this guard alone.
    """
    if scan is None:
        return 0.0, 0.0, "SENSOR_NOT_READY"
    if abs(angular) > 1e-6 and scan["around"] < ROTATION_CLEARANCE:
        return 0.0, 0.0, "ROTATION_BLOCKED"
    if abs(linear) > 1e-6:
        clearance = scan["front"] if linear > 0 else scan["rear"]
        if clearance <= STOP_DISTANCE:
            return 0.0, 0.0, "FRONT_BLOCKED" if linear > 0 else "REAR_BLOCKED"
        factor = min(1.0, max(0.20, (clearance - STOP_DISTANCE) /
                             (SLOW_DISTANCE - STOP_DISTANCE)))
        if factor < 1.0:
            return linear * factor, angular * factor, "SLOWING"
    return linear, angular, "CLEAR"


class Checkout:
    """Bounded sequence measured by encoders; no world coordinates/ Supervisor."""

    PHASES = ("FORWARD", "STOP_1", "TURN_LEFT", "STOP_2", "TURN_RIGHT",
              "STOP_3", "APPROACH", "DONE")

    def __init__(self, now, encoders):
        self.index = 0
        self.started = now
        self.origin = encoders
        self.result = "running"

    @property
    def phase(self):
        return self.PHASES[self.index]

    def advance(self, now, encoders):
        self.index += 1
        self.started = now
        self.origin = encoders

    def command(self, now, encoders, scan):
        dl = (encoders[0] - self.origin[0]) * WHEEL_RADIUS
        dr = (encoders[1] - self.origin[1]) * WHEEL_RADIUS
        distance = (dl + dr) * 0.5
        angle = (dr - dl) / AXLE_LENGTH
        elapsed = now - self.started
        phase = self.phase
        if phase == "DONE":
            return 0.0, 0.0
        if scan is None:
            self.index = len(self.PHASES) - 1
            self.result = "sensor_fault"
            return 0.0, 0.0
        if phase.startswith("STOP"):
            if elapsed >= 0.65:
                self.advance(now, encoders)
            return 0.0, 0.0
        if phase.startswith("TURN"):
            if scan["around"] < ROTATION_CLEARANCE:
                self.index = len(self.PHASES) - 1
                self.result = "rotation_blocked"
                return 0.0, 0.0
            if abs(angle) >= math.pi / 4:
                self.advance(now, encoders)
                return 0.0, 0.0
            if elapsed > 6.0:
                self.index = len(self.PHASES) - 1
                self.result = "turn_timeout"
                return 0.0, 0.0
            return 0.0, TURN_SPEED if phase == "TURN_LEFT" else -TURN_SPEED
        if phase == "FORWARD":
            if distance >= 0.20 or scan["front"] <= STOP_DISTANCE:
                self.advance(now, encoders)
                return 0.0, 0.0
            if elapsed > 8.0:
                self.index = len(self.PHASES) - 1
                self.result = "forward_timeout"
                return 0.0, 0.0
        if phase == "APPROACH":
            if scan["front"] <= STOP_DISTANCE:
                self.advance(now, encoders)
                self.result = "obstacle_stop"
                return 0.0, 0.0
            if distance >= 2.0 or elapsed >= 45.0:
                self.advance(now, encoders)
                self.result = "travel_limit_no_obstacle"  # not an obstacle-stop pass
                return 0.0, 0.0
        return DRIVE_SPEED, 0.0


class EventLog:
    def __init__(self):
        self.file = None
        try:
            self.directory = Path(__file__).resolve().parent / "logs"
            self.directory.mkdir(exist_ok=True)
            self.file = (self.directory / "stage1_latest.jsonl").open("w", encoding="utf-8")
        except OSError as exc:
            print(f"[stage1] File logging unavailable: {exc}", flush=True)

    def write(self, event, **values):
        record = {"event": event, **values}
        if self.file:
            try:
                self.file.write(json.dumps(record, allow_nan=False) + "\n")
                self.file.flush()
            except OSError:
                self.file.close()
                self.file = None
        if event != "sample":
            print("[stage1] " + json.dumps(record), flush=True)

    def close(self):
        if self.file:
            self.file.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manual", action="store_true", help="start stationary in manual mode")
    args = parser.parse_args()
    from controller import Robot, Keyboard

    robot = Robot()  # deliberately not Supervisor: no ground-truth pose access
    log = EventLog()
    motors = []
    try:
        timestep = int(robot.getBasicTimeStep())
        devices = {robot.getDeviceByIndex(i).getName(): robot.getDeviceByIndex(i)
                   for i in range(robot.getNumberOfDevices())}
        required = ("left wheel motor", "right wheel motor", "LDS-01", "camera",
                    "gyro", "accelerometer", "compass")
        missing = [name for name in required if name not in devices]
        if missing:
            raise RuntimeError("Missing robot devices: " + ", ".join(missing))
        motors = [devices["left wheel motor"], devices["right wheel motor"]]
        encoders = [motor.getPositionSensor() for motor in motors]
        if any(sensor is None for sensor in encoders):
            raise RuntimeError("Missing wheel encoders")
        for motor in motors:
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)
        lidar = devices["LDS-01"]
        camera = devices["camera"]
        gyro = devices["gyro"]
        accelerometer = devices["accelerometer"]
        compass = devices["compass"]
        for sensor in [*encoders, lidar, gyro, accelerometer, compass]:
            sensor.enable(timestep)
        camera.enable(timestep * 3)
        keyboard = robot.getKeyboard()
        keyboard.enable(timestep)
        limit = min(motor.getMaxVelocity() for motor in motors)
        log.write("startup", timestep_ms=timestep, devices=sorted(devices),
                  camera=[camera.getWidth(), camera.getHeight()],
                  lidar_points=lidar.getHorizontalResolution(),
                  mode="manual" if args.manual else "checkout")
        print("[stage1] T=checkout M=manual WASD=hold to move SPACE/X=STOP R=release", flush=True)
        ready = False
        estop = False
        checkout = None
        auto_start = not args.manual
        started = robot.getTime()
        last_sample = -1.0
        previous_status = None
        previous_keys = set()
        while robot.step(timestep) != -1:
            now = robot.getTime()
            positions = tuple(sensor.getValue() for sensor in encoders)
            inertial = {"gyro": gyro.getValues(), "accel": accelerometer.getValues(),
                        "compass": compass.getValues()}
            camera_data = camera.getImage()
            ranges = lidar.getRangeImage()
            scan = scan_geometry(ranges, lidar.getMaxRange())
            sensors_ok = (all(math.isfinite(v) for v in positions) and
                          all(math.isfinite(v) for values in inertial.values() for v in values) and
                          camera_data is not None and
                          len(camera_data) == camera.getWidth() * camera.getHeight() * 4 and
                          scan is not None)
            keys = set()
            key = keyboard.getKey()
            while key != -1:
                keys.add(key & Keyboard.KEY)
                key = keyboard.getKey()
            # Webots uses upper-case character key codes. Edge-trigger mode keys.
            new_keys = keys - previous_keys
            previous_keys = keys
            if ord("X") in keys or ord(" ") in keys:
                estop = True
                auto_start = False
                checkout = None
            elif ord("R") in new_keys:
                estop = False
                checkout = None
                auto_start = False
            if ord("M") in keys:
                checkout = None
                auto_start = False
            if not sensors_ok:
                for motor in motors:
                    motor.setVelocity(0.0)
                if ready:
                    estop = True
                    checkout = None
                    auto_start = False
                if previous_status != "SENSOR_NOT_READY":
                    log.write("state", time=now, state="SENSOR_NOT_READY")
                    previous_status = "SENSOR_NOT_READY"
                continue
            if not ready and now - started >= 0.5:
                ready = True
                log.write("sensors_ready", time=now, scan=scan, encoders=positions)
                if log.file:
                    result = camera.saveImage(str(log.directory / "stage1_camera.png"), 90)
                    log.write("camera_snapshot", time=now, saved=result == 0)
            if not ready:
                continue
            if not estop and (auto_start or ord("T") in new_keys):
                checkout = Checkout(now, positions)
                auto_start = False
                log.write("checkout_started", time=now)
            motion_keys = keys.intersection({ord(c) for c in "WASD"})
            if motion_keys:
                checkout = None
                auto_start = False
            linear = angular = 0.0
            mode = "MANUAL_IDLE"
            if estop:
                mode = "EMERGENCY_STOP"
            elif checkout:
                linear, angular = checkout.command(now, positions, scan)
                mode = checkout.phase
            elif motion_keys:
                # Separate translation and rotation in this basic checkout.
                if ord("W") in keys and ord("S") not in keys:
                    linear = DRIVE_SPEED
                elif ord("S") in keys and ord("W") not in keys:
                    linear = -DRIVE_SPEED
                elif ord("A") in keys and ord("D") not in keys:
                    angular = TURN_SPEED
                elif ord("D") in keys and ord("A") not in keys:
                    angular = -TURN_SPEED
                mode = "MANUAL"
            linear, angular, guard = protect_motion(linear, angular, scan)
            speeds = wheel_speeds(linear, angular, limit)
            for motor, speed in zip(motors, speeds):
                motor.setVelocity(speed)
            status = (mode, guard)
            if status != previous_status:
                log.write("state", time=now, state=mode, guard=guard, scan=scan,
                          encoders=positions, wheels=speeds,
                          result=checkout.result if checkout else None)
                previous_status = status
            if now - last_sample >= 0.5:
                log.write("sample", time=now, state=mode, scan=scan, encoders=positions,
                          wheels=speeds, **inertial)
                last_sample = now
    except Exception as exc:
        log.write("error", message=str(exc))
        raise
    finally:
        for motor in motors:
            motor.setVelocity(0.0)
        log.close()


if __name__ == "__main__":
    main()
