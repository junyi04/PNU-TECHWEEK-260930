"""End-to-end production controller test; truth is confined to observer.

--apartment uses a copy of the unmodified task layout, never a prebuilt map.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
from run_stage1_simulation import WORLD

ROOT = Path(__file__).resolve().parents[1]
OBSERVER = '''import json, math
from pathlib import Path
from controller import Supervisor
r=Supervisor()
root=Path(__file__).resolve().parents[2]
robot=r.getFromDef('TEST_ROBOT')
apples=[r.getFromDef('TEST_APPLE_1'),r.getFromDef('TEST_APPLE_2')]
start=None
start_heading=0
closest=[999,999]
contacts=0
sample={}
read_at=0
started=None
last_truth=-1
last_visited=0
physical_visits=set()
truth_file=(root/'truth.jsonl').open('w')
while r.step(64)!=-1:
    now=r.getTime()
    p=robot.getPosition()
    if start is None and now>.5:
        start=list(p)
        orientation=robot.getOrientation()
        start_heading=math.atan2(orientation[3],orientation[0])
    if start:
        closest=[min(d,math.dist(p[:2],a.getPosition()[:2])) for d,a in zip(closest,apples)]
        contacts+=sum(c.point[2]>.04 for c in robot.getContactPoints(True))
    log=root/'controllers/apple_collector/logs/navigation_latest.jsonl'
    if log.exists():
        with log.open('rb') as f:
            f.seek(read_at)
            for line in f.readlines():
                try: event=json.loads(line)
                except json.JSONDecodeError: break
                read_at+=len(line)
                if event['event']=='sample': sample=event
    state=sample.get('mission_state')
    if sample.get('visited_count',0)>last_visited:
        # Identify the REPORTED target, not the apple nearest to the chassis.
        # Both apples can legitimately be within arrival range simultaneously.
        for visit in sample['visited'][last_visited:]:
            x,y=visit['position']
            c,s=math.cos(start_heading),math.sin(start_heading)
            reported=[start[0]+c*x-s*y,start[1]+s*x+c*y]
            target_errors=[math.dist(reported,a.getPosition()[:2]) for a in apples]
            nearest=min(range(2),key=lambda i:target_errors[i])
            if target_errors[nearest]<.35 and math.dist(p[:2],apples[nearest].getPosition()[:2])<.60:
                physical_visits.add(nearest)
        last_visited=sample['visited_count']
    if now-last_truth>=1:
        truth_file.write(json.dumps(dict(time=now,position=p,orientation=robot.getOrientation(),
                                        contacts=contacts,closest=closest,estimated=sample.get('pose')))+'\\n')
        truth_file.flush()
        last_truth=now
    if state not in (None,'IDLE') and started is None: started=now
    if state in ('SUCCEEDED','FAILED','INCOMPLETE') or (started is not None and now-started>SIM_LIMIT):
        distance=math.dist(start[:2],p[:2]) if start else 999
        checks={'mission_success':state=='SUCCEEDED','visited_both_physically':len(physical_visits)==2,
                'return_error':distance<.35,'no_obstacle_contacts':contacts==0}
        result=dict(passed=all(checks.values()),checks=checks,closest_m=closest,return_error_m=distance,
                    contacts=contacts,sim_time=now,elapsed=now-started,sample=sample)
        (root/'result.json').write_text(json.dumps(result,indent=2))
        truth_file.close()
        r.simulationQuit(0)
        break
'''


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--webots',required=True)
    parser.add_argument('--apartment',action='store_true')
    parser.add_argument('--seconds',type=float,default=1200)
    parser.add_argument('--fast',action='store_true')
    args=parser.parse_args()
    project=ROOT/'.tmp'/('mission_apartment' if args.apartment else 'mission_smoke')
    for part in ('worlds','controllers/observer'):
        (project/part).mkdir(parents=True,exist_ok=True)
    shutil.copytree(ROOT/'controllers/apple_collector',project/'controllers/apple_collector',
                    dirs_exist_ok=True,ignore=shutil.ignore_patterns('logs','__pycache__'))
    shutil.copytree(ROOT/'protos',project/'protos',dirs_exist_ok=True)
    if args.apartment:
        world=(ROOT/'worlds/apartment.wbt').read_text(encoding='utf-8')
        world=world.replace('TurtleBot3Burger {','DEF TEST_ROBOT TurtleBot3Burger {',1)
        world=world.replace('RedApple {','DEF TEST_APPLE_1 RedApple {',1)
        # Only the second occurrence, not the newly named first one.
        pos=world.index('\nRedApple {')
        world=world[:pos]+world[pos:].replace('RedApple {','DEF TEST_APPLE_2 RedApple {',1)
        world+='\nRobot { supervisor TRUE controller "observer" }\n'
    else:
        world=WORLD.replace('controllerArgs [ "--checkout" ]','')
        world=world.replace('translation 0.85 0 0.25','translation 2 0 0.25')
        world=world.replace('controller "stage1_observer"','controller "observer"')
        externs=''.join(f'EXTERNPROTO "../protos/{c}Apple.proto"\n' for c in ('Red','Green','Orange','Purple'))
        world=world.replace('WorldInfo',externs+'WorldInfo',1)
        world+='\nDEF TEST_APPLE_1 RedApple { translation .65 .20 .05 }\n'
        world+='\nDEF TEST_APPLE_2 RedApple { translation .65 -.20 .05 }\n'
        world+='\nGreenApple { translation .8 0 .05 }\n'
        world+='\nOrangeApple { translation 1.1 .12 .05 }\n'
        world+='\nPurpleApple { translation 1.1 -.12 .05 }\n'
    (project/'worlds/check.wbt').write_text(world,encoding='utf-8')
    (project/'controllers/observer/observer.py').write_text(OBSERVER.replace('SIM_LIMIT',str(args.seconds)),encoding='utf-8')
    for file in ('result.json','controllers/apple_collector/logs/navigation_latest.jsonl'):
        (project/file).unlink(missing_ok=True)
    options={}
    if os.name=='nt':
        startup=subprocess.STARTUPINFO()
        startup.dwFlags|=subprocess.STARTF_USESHOWWINDOW
        startup.wShowWindow=0
        options=dict(startupinfo=startup,creationflags=subprocess.CREATE_NO_WINDOW)
    with (project/'webots.log').open('w') as out:
        p=subprocess.Popen([args.webots,'--batch','--mode='+('fast' if args.fast else 'realtime'),
                            '--minimize','--no-rendering','--stdout','--stderr','--port=1247',str(project/'worlds/check.wbt')],
                           stdout=out,stderr=out,**options)
        try: p.wait(timeout=3600)
        except subprocess.TimeoutExpired:
            if os.name=='nt': subprocess.run(['taskkill','/PID',str(p.pid),'/T','/F'],capture_output=True)
            else: p.terminate()
            raise
    if not (project/'result.json').exists():
        raise RuntimeError(f'Webots exited before observer result (exit {p.returncode}); see {project}/webots.log')
    result=json.loads((project/'result.json').read_text())
    print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
