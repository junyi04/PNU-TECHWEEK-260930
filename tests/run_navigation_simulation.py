"""Isolated physical navigation validation; Supervisor exists ONLY in test code."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
OBSERVER = '''import json, math
from pathlib import Path
from controller import Supervisor
r=Supervisor()
step=int(r.getBasicTimeStep())
target=r.getFromDef('TEST_ROBOT')
root=Path(__file__).resolve().parents[2]
log=root/'controllers/apple_collector/logs/navigation_latest.jsonl'
start=None
heading=0
contacts=0
max_distance=0
sample={}
read_at=0
mode=''
done=None
while r.step(step)!=-1:
    now=r.getTime()
    p=target.getPosition()
    if start is None and now>=.5:
        start=list(p)
        o=target.getOrientation()
        heading=math.atan2(o[3],o[0])
    if start:
        max_distance=max(max_distance,math.dist(start[:2],p[:2]))
    contacts+=sum(c.point[2]>.04 for c in target.getContactPoints(True))
    if log.exists():
        with log.open('rb') as f:
            f.seek(read_at)
            lines=f.readlines()
            for line in lines:
                try: event=json.loads(line)
                except json.JSONDecodeError: break
                read_at+=len(line)
                if event['event']=='sample': sample=event
                if event['event']=='state': mode=event['state']
    if mode in ('HOME','STOPPED','EMERGENCY_STOP') and done is None: done=now
    if done and now-done>1: break
    if now>TIME_LIMIT: break
distance=math.dist(start[:2],p[:2]) if start else 999
dx,dy=p[0]-start[0],p[1]-start[1]
truth=[math.cos(heading)*dx+math.sin(heading)*dy,-math.sin(heading)*dx+math.cos(heading)*dy]
pose=sample.get('pose',[999,999,999])
checks={'home':mode=='HOME','explored':max_distance>.65,'physical_return':distance<.35,
        'no_obstacle_contact':contacts==0,'map_created':sample.get('mapped_m2',0)>5}
summary={'passed':all(checks.values()),'checks':checks,'mode':mode,'sim_time':now,
         'max_distance':max_distance,'return_error_m':distance,'contacts':contacts,
         'pose_error_m':math.dist(truth,pose[:2]),'sample':sample}
(root/'result.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
r.simulationQuit(0 if summary['passed'] else 1)
'''


def wall(x, y, sx, sy):
    return f'''Solid {{ translation {x} {y} 0.3
      children [ Shape {{ appearance PBRAppearance {{ baseColor .4 .5 .6 }}
      geometry Box {{ size {sx} {sy} .6 }} }} ]
      boundingObject Box {{ size {sx} {sy} .6 }} }}\n'''


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--webots',required=True)
    parser.add_argument('--apartment',action='store_true')
    parser.add_argument('--seconds',type=float,default=45)
    args=parser.parse_args()
    project=ROOT/'.tmp'/('nav_apartment' if args.apartment else 'nav_smoke')
    for part in ('worlds','controllers/apple_collector','controllers/observer'):
        (project/part).mkdir(parents=True,exist_ok=True)
    for filename in ('apple_collector.py','navigation.py'):
        shutil.copy2(ROOT/'controllers/apple_collector'/filename,project/'controllers/apple_collector')
    if args.apartment:
        world=(ROOT/'worlds/apartment.wbt').read_text(encoding='utf-8')
        world=world.replace('TurtleBot3Burger {','DEF TEST_ROBOT TurtleBot3Burger {',1)
        shutil.copytree(ROOT/'protos',project/'protos',dirs_exist_ok=True)
    else:
        world='''#VRML_SIM R2025a utf8
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/robotis/turtlebot/protos/TurtleBot3Burger.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/devices/robotis/protos/RobotisLds01.proto"
WorldInfo { basicTimeStep 64 }
Viewpoint { orientation 0.2 0.6 0.77 2.4 position 4 4 6 }
Background { skyColor [ .6 .7 .8 ] }
DirectionalLight { direction -1 -1 -2 }
Solid { translation 0 0 -.05 children [ Shape { geometry Box { size 8 8 .1 } } ]
boundingObject Box { size 8 8 .1 } }
DEF TEST_ROBOT TurtleBot3Burger {
translation 0 0 0 rotation 0 0 1 0 controller "apple_collector"
extensionSlot [ RobotisLds01 {} Display { name "map" width 480 height 480 } ] }
'''
        world+=wall(-3,0,.1,6)+wall(3,0,.1,6)+wall(0,-3,6,.1)+wall(0,3,6,.1)
        world+=wall(1.2,0,.12,2)
    world=world.replace('controller "apple_collector"',f'controller "apple_collector" controllerArgs ["--explore-seconds={args.seconds}"]',1)
    world+='\nRobot { supervisor TRUE controller "observer" }\n'
    (project/'worlds/check.wbt').write_text(world,encoding='utf-8')
    (project/'controllers/observer/observer.py').write_text(
        OBSERVER.replace('TIME_LIMIT',str(args.seconds+180)),encoding='utf-8')
    for path in (project/'result.json',project/'controllers/apple_collector/logs/navigation_latest.jsonl'):
        path.unlink(missing_ok=True)
    options={}
    if os.name=='nt':
        startup=subprocess.STARTUPINFO()
        startup.dwFlags|=subprocess.STARTF_USESHOWWINDOW
        startup.wShowWindow=0
        options={'startupinfo':startup,'creationflags':subprocess.CREATE_NO_WINDOW}
    with (project/'webots.log').open('w',encoding='utf-8') as output:
        p=subprocess.Popen([args.webots,'--batch','--mode=fast','--minimize','--no-rendering','--stdout','--stderr',
                            '--port=1245',str(project/'worlds/check.wbt')],stdout=output,stderr=output,**options)
        try: p.wait(timeout=600)
        except subprocess.TimeoutExpired:
            if os.name=='nt': subprocess.run(['taskkill','/PID',str(p.pid),'/T','/F'],capture_output=True)
            else: p.terminate()
            raise RuntimeError('Simulation timeout: '+str(project))
    summary=json.loads((project/'result.json').read_text(encoding='utf-8'))
    print(json.dumps(summary,indent=2))
    return 0 if summary['passed'] else 1


if __name__=='__main__': sys.exit(main())
