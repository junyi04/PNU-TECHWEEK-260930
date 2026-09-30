"""Stationary fixture: two red apples and three distractors, real Webots camera/YOLO.

Test-only observer owns simulationQuit; never open .tmp worlds for normal use.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
from run_stage1_simulation import WORLD

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--webots', required=True)
    args = parser.parse_args()
    project = ROOT/'.tmp/perception_sim'
    for part in ('worlds', 'controllers/observer'):
        (project/part).mkdir(parents=True, exist_ok=True)
    shutil.copytree(ROOT/'controllers/apple_collector', project/'controllers/apple_collector',
                    dirs_exist_ok=True, ignore=shutil.ignore_patterns('logs', '__pycache__'))
    shutil.copytree(ROOT/'protos', project/'protos', dirs_exist_ok=True)
    world = WORLD.replace('controllerArgs [ "--checkout" ]', 'controllerArgs [ "--manual" ]')
    world = world.replace('extensionSlot [', 'extensionSlot [ Display { name "detections" width 640 height 480 } Display { name "map" width 480 height 480 }', 1)
    world = world.replace('Robot { supervisor TRUE controller "stage1_observer" }',
                          'Robot { supervisor TRUE controller "observer" }')
    # Move checkout wall behind the objects, retaining bounded LiDAR returns.
    world = world.replace('translation 0.85 0 0.25', 'translation 2 0 0.25')
    externs = ''.join(f'EXTERNPROTO "../protos/{c}Apple.proto"\n' for c in ('Red', 'Green', 'Orange', 'Purple'))
    world = world.replace('WorldInfo', externs+'WorldInfo', 1)
    for color, x, y in [('Red', .65, .20), ('Red', .65, -.20), ('Green', .8, 0),
                        ('Orange', 1.1, .12), ('Purple', 1.1, -.12)]:
        world += f'\n{color}Apple {{ translation {x} {y} .05 }}\n'
    (project/'worlds/check.wbt').write_text(world, encoding='utf-8')
    (project/'controllers/observer/observer.py').write_text('''import json, math
from pathlib import Path
from controller import Supervisor
r=Supervisor()
root=Path(__file__).resolve().parents[2]
while r.step(64)!=-1:
    if r.getTime()>40:
        source=root/'controllers/apple_collector/logs/targets_latest.json'
        targets=json.loads(source.read_text()) if source.exists() else []
        expected=[(.65,.20),(.65,-.20)]
        errors=[min((math.dist(p,t['position']) for t in targets),default=999) for p in expected]
        passed=len(targets)==2 and max(errors)<.10
        (root/'result.json').write_text(json.dumps({'targets':targets,'position_errors_m':errors,'passed':passed},indent=2))
        r.simulationQuit(0)
        break
''', encoding='utf-8')
    (project/'result.json').unlink(missing_ok=True)
    options = {}
    if os.name == 'nt':
        startup = subprocess.STARTUPINFO()
        startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        startup.wShowWindow = 0
        options = dict(startupinfo=startup, creationflags=subprocess.CREATE_NO_WINDOW)
    with (project/'webots.log').open('w') as output:
        p = subprocess.Popen([args.webots, '--batch', '--mode=realtime', '--minimize',
                              '--stdout', '--stderr', '--port=1246', str(project/'worlds/check.wbt')],
                             stdout=output, stderr=output, **options)
        try:
            p.wait(timeout=240)
        except subprocess.TimeoutExpired:
            if os.name == 'nt':
                subprocess.run(['taskkill', '/PID', str(p.pid), '/T', '/F'], capture_output=True)
            else:
                p.terminate()
            raise
    result = json.loads((project/'result.json').read_text())
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
