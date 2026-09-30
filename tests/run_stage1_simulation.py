"""Run a physical stop check in an isolated Webots project, never the user's world.

Usage: python tests/run_stage1_simulation.py --webots <path-to-webots.exe>
Creates .tmp/stage1_sim, copies the production controller, and adds a test-only
Supervisor observer. The production controller has no Supervisor privileges.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


WORLD = '''#VRML_SIM R2025a utf8
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/robotis/turtlebot/protos/TurtleBot3Burger.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/devices/robotis/protos/RobotisLds01.proto"
WorldInfo { basicTimeStep 64 }
Viewpoint { orientation 0.2 0.6 0.77 2.4 position 2 2 3 }
Background { skyColor [ 0.6 0.7 0.8 ] }
DirectionalLight { direction -1 -1 -2 }
Solid {
  translation 0 0 -0.05
  children [ Shape { appearance PBRAppearance { baseColor 0.5 0.5 0.5 roughness 1 metalness 0 } geometry Box { size 5 5 0.1 } } ]
  boundingObject Box { size 5 5 0.1 }
}
Solid {
  translation 0.85 0 0.25
  children [ Shape { appearance PBRAppearance { baseColor 0.1 0.2 0.8 roughness 1 metalness 0 } geometry Box { size 0.1 2 0.5 } } ]
  boundingObject Box { size 0.1 2 0.5 }
}
DEF TEST_ROBOT TurtleBot3Burger {
  translation 0 0 0
  rotation 0 0 1 0
  controller "apple_collector"
  controllerArgs [ "--checkout" ]
  extensionSlot [
    Camera { translation 0.05 0 -0.08 fieldOfView 1.0472 width 640 height 480 }
    RobotisLds01 { }
  ]
}
Robot { supervisor TRUE controller "stage1_observer" }
'''


# Ground truth is confined to this independent test observer, not navigation.
OBSERVER = '''import json
import math
from pathlib import Path
from controller import Supervisor

robot = Supervisor()
step = int(robot.getBasicTimeStep())
target = robot.getFromDef("TEST_ROBOT")
root = Path(__file__).resolve().parents[2]
log = root / "controllers/apple_collector/logs/stage1_latest.jsonl"
start = None
done_time = None
done_position = None
contacts = 0
states = {}
max_yaw = 0.0
result = None
drift = None
while robot.step(step) != -1:
    now = robot.getTime()
    position = target.getPosition()
    if start is None and now >= 0.5:
        start = list(position)
    orientation = target.getOrientation()
    yaw = math.atan2(orientation[3], orientation[0])
    max_yaw = max(max_yaw, yaw)
    # Floor contact is expected; a raised contact means a physical obstacle hit.
    contacts += sum(p.point[2] > 0.04 for p in target.getContactPoints(True))
    if log.exists():
        for line in log.read_text(encoding="utf-8").splitlines():
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue  # writer may be in the middle of its last record
            if event["event"] == "state":
                phase = event["state"]
                if phase not in states:
                    states[phase] = {"position": list(position), "yaw": yaw}
                if phase == "DONE" and done_time is None:
                    done_time = now
                    done_position = list(position)
                    result = event.get("result")
    if done_time is not None and now - done_time >= 1.0:
        drift = math.dist(position[:2], done_position[:2])
        break
    if now > 65:
        result = "timeout"
        break

forward = (math.dist(states["STOP_1"]["position"][:2], start[:2])
           if start and "STOP_1" in states else None)
checks = {
    "forward_20cm": forward is not None and 0.17 < forward < 0.25,
    "left_turn": 0.65 < max_yaw < 0.95,
    "right_turn": "STOP_3" in states and abs(states["STOP_3"]["yaw"]) < 0.12,
    "obstacle_stop": result == "obstacle_stop",
    "no_obstacle_contacts": contacts == 0,
    "stationary_after_stop": drift is not None and drift < 0.005,
    "wall_clearance": position[0] < 0.60,
}
summary = {"checks": checks, "passed": all(checks.values()),
           "sim_time": now, "result": result, "forward_m": forward,
           "max_yaw_rad": max_yaw, "stop_drift_m": drift,
           "final_position": list(position), "raised_contacts": contacts,
           "states": states}
(root / "result.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
robot.simulationQuit(0 if summary["passed"] else 1)
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--webots", required=True)
    args = parser.parse_args()
    repository = Path(__file__).resolve().parents[1]
    project = repository / ".tmp" / "stage1_sim"
    worlds = project / "worlds"
    controller = project / "controllers" / "apple_collector"
    observer = project / "controllers" / "stage1_observer"
    for folder in (worlds, controller, observer):
        folder.mkdir(parents=True, exist_ok=True)
    shutil.copy2(repository / "controllers/apple_collector/apple_collector.py", controller)
    (observer / "stage1_observer.py").write_text(OBSERVER, encoding="utf-8")
    world = worlds / "stage1_check.wbt"
    world.write_text(WORLD, encoding="utf-8")
    result_file = project / "result.json"
    result_file.unlink(missing_ok=True)
    # Prevent a previous run's DONE record from reaching the observer on startup.
    (controller / "logs/stage1_latest.jsonl").unlink(missing_ok=True)
    kwargs = {}
    if os.name == "nt":
        startup = subprocess.STARTUPINFO()
        startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        startup.wShowWindow = 0
        kwargs = {"startupinfo": startup, "creationflags": subprocess.CREATE_NO_WINDOW}
    with (project / "webots.log").open("w", encoding="utf-8") as output:
        process = subprocess.Popen(
            [args.webots, "--batch", "--mode=fast", "--minimize", "--stdout", "--stderr",
             "--port=1243", str(world)], stdout=output, stderr=subprocess.STDOUT, **kwargs)
        try:
            process.wait(timeout=240)
        except subprocess.TimeoutExpired:
            if os.name == "nt":
                subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"],
                               capture_output=True)
            else:
                process.terminate()
            raise RuntimeError("Webots timed out; inspect " + str(project / "webots.log"))
    if not result_file.exists():
        raise RuntimeError("Observer produced no result; inspect " + str(project / "webots.log"))
    result = json.loads(result_file.read_text(encoding="utf-8"))
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
