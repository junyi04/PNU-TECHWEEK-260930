"""Asynchronous YOLO apple detection. No Webots or ground-truth access here."""
import math
import multiprocessing as mp
from pathlib import Path
from queue import Empty, Full
import numpy as np


def red_region(bgr, box):
    import cv2
    h, w = bgr.shape[:2]
    x1, y1, x2, y2 = [int(v) for v in box]
    x1, y1, x2, y2 = max(0, x1), max(0, y1), min(w, x2), min(h, y2)
    if x2 <= x1 or y2 <= y1:
        return None
    hsv = cv2.cvtColor(bgr[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
    mask = (((hsv[:, :, 0] <= 10) | (hsv[:, :, 0] >= 170)) &
            (hsv[:, :, 1] >= 90) & (hsv[:, :, 2] >= 50)).astype('uint8')
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours or mask.mean() < .25:
        return None
    contour = max(contours, key=cv2.contourArea)
    x, y, cw, ch = cv2.boundingRect(contour)
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    if (area < 16 or not .65 < cw/ch < 1.45 or area/(cw*ch) > .88
            or 4*math.pi*area/max(perimeter**2, 1) < .65):
        return None
    return [x+x1, y+y1, x+x1+cw, y+y1+ch]


def ground_position(box, width, height, fov, pose):
    """Known .10 m apple diameter gives range; floor contact is a plausibility gate.

    Uses object geometry, never world placement. Approximate under occlusion/tilt.
    """
    x1, y1, x2, y2 = box
    f = width / (2 * math.tan(fov / 2))
    below = y2 - (height-1)/2
    if below <= 2 or y2 >= height-1 or x1 <= 0 or x2 >= width-1:
        return None
    depth = f * .10 / max(x2-x1, 1)
    apparent_floor_height = below*depth/f
    if not .15 <= depth <= 4 or not .035 <= apparent_floor_height <= .14:
        return None
    x, y = depth+.02, -((x1+x2)/2-(width-1)/2)*depth/f
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return [pose[0]+c*x-s*y, pose[1]+s*x+c*y]


class TargetTracker:
    def __init__(self):
        self.tracks = []
        self.last_time = -1
        self.next_id = 1

    @property
    def confirmed(self):
        return [dict(t) for t in self.tracks if t['hits'] >= 3 and t['last']-t['first'] >= .3]

    def update(self, detections, timestamp):
        if timestamp <= self.last_time:
            return
        self.last_time = timestamp
        confirmed_ids = {t['id'] for t in self.confirmed}
        self.tracks = [t for t in self.tracks if t['id'] in confirmed_ids or timestamp-t['last'] < 3]
        used = set()
        points = []
        boxes = []
        for d in sorted(detections, key=lambda d: -d['confidence']):
            xy = d.get('position')
            if xy is None or any(math.dist(xy, p) < .12 for p in points):
                continue
            box = d.get('box')
            if box is not None:
                def overlaps(old):
                    intersection = max(0, min(box[2], old[2])-max(box[0], old[0])) * max(0, min(box[3], old[3])-max(box[1], old[1]))
                    smaller = min((box[2]-box[0])*(box[3]-box[1]), (old[2]-old[0])*(old[3]-old[1]))
                    return intersection/max(smaller, 1) > .65
                if any(overlaps(old) for old in boxes):
                    continue
                boxes.append(box)
            points.append(xy)
            # The smoothed location lags when range estimates change on approach.
            # Associate against the last observation too, avoiding a fresh ID
            # for the same continuously observed apple.
            def association_distance(t):
                return min(math.dist(t['position'], xy), math.dist(t.get('last_position', t['position']), xy))
            candidates = [t for t in self.tracks if t['id'] not in used and association_distance(t) < .35]
            if candidates:
                t = min(candidates, key=association_distance)
                weight = .65 if d.get('observed_distance', 999.) <= .55 else .25
                t['position'] = [(1-weight)*a+weight*b for a, b in zip(t['position'], xy)]
                t['hits'] += 1
                t['last'] = timestamp
                t['confidence'] = d['confidence']
                t['source'] = d.get('source', 'yolo_apple')
                t['observed_distance'] = d.get('observed_distance', 999.)
            else:
                t = dict(id=self.next_id, position=xy, hits=1, first=timestamp,
                         last=timestamp, confidence=d['confidence'], source=d.get('source', 'yolo_apple'),
                         observed_distance=d.get('observed_distance', 999.))
                self.next_id += 1
                self.tracks.append(t)
            t['last_position'] = list(xy)
            used.add(t['id'])
        # Separate boxes observed together are evidence of distinct objects.
        for t in self.tracks:
            if t['id'] in used:
                t['distinct_from'] = sorted(set(t.get('distinct_from', [])) | (used-{t['id']}))


def _latest(queue, value):
    try:
        queue.put_nowait(value)
    except Full:
        try:
            queue.get_nowait()
        except Empty:
            pass
        try:
            queue.put_nowait(value)
        except Full:
            pass


def _worker(jobs, results, stop, model_path, requested):
    try:
        import cv2
        import torch
        from ultralytics import YOLO
        torch.set_num_threads(2)
        device = ('0' if torch.cuda.is_available() else 'cpu') if requested == 'auto' else requested
        if not Path(model_path).is_file():
            raise FileNotFoundError(model_path)
        model = YOLO(model_path)
        apple = next(k for k, v in model.names.items() if v == 'apple')
        # COCO weights confuse untextured Webots fruit with these round objects.
        proposal_classes = [k for k, v in model.names.items()
                            if v in ('apple', 'orange', 'sports ball', 'frisbee')]
        def predict(frame):
            return model.predict(frame, classes=proposal_classes, conf=.20, imgsz=640,
                                 device=device, verbose=False)[0]
        try:
            predict(np.zeros((480, 640, 3), np.uint8))
        except Exception:
            if requested != 'auto' or device == 'cpu':
                raise
            device = 'cpu'
            model = YOLO(model_path)
            predict(np.zeros((480, 640, 3), np.uint8))
        _latest(results, dict(status='ready', device=device))
        while not stop.is_set():
            try:
                job = jobs.get(timeout=.2)
            except Empty:
                continue
            frame = job.pop('frame')
            detections = []
            result = predict(frame)
            for box, confidence, cls in zip(result.boxes.xyxy.cpu().tolist(), result.boxes.conf.cpu().tolist(),
                                            result.boxes.cls.cpu().tolist()):
                red = red_region(frame, box)
                if red is None:
                    continue
                position = ground_position(red, frame.shape[1], frame.shape[0], job['fov'], job['pose'])
                if any(math.dist([(red[0]+red[2])/2, (red[1]+red[3])/2],
                                 [(d['box'][0]+d['box'][2])/2, (d['box'][1]+d['box'][3])/2]) < 10
                       for d in detections):
                    continue
                source = 'yolo_apple' if int(cls) == apple else 'appearance_candidate'
                detections.append(dict(box=red, confidence=confidence, position=position,
                                       observed_distance=math.dist(position, job['pose'][:2]) if position else None,
                                       source=source, yolo_class=model.names[int(cls)]))
                x1, y1, x2, y2 = red
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, f'red {model.names[int(cls)]} {confidence:.2f}', (x1, max(20, y1-5)),
                            cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 255), 1)
            _latest(results, dict(**job, status='running', device=device,
                                  detections=detections, rgb=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)))
    except Exception as exc:
        _latest(results, dict(status='error', error=f'{type(exc).__name__}: {exc}'))


class Vision:
    def __init__(self, device='auto'):
        ctx = mp.get_context('spawn')
        self.jobs, self.results = ctx.Queue(1), ctx.Queue(2)
        self.stop = ctx.Event()
        self.process = ctx.Process(target=_worker, args=(self.jobs, self.results, self.stop,
            str(Path(__file__).parent / 'models' / 'yolo11n.pt'), device), daemon=True)
        self.process.start()
        self.tracker = TargetTracker()
        self.status = 'loading'
        self.device = 'pending'
        self.last_submit = -1
        self.last_result_time = None

    def submit(self, camera, pose, now):
        if now-self.last_submit < .192 or self.status == 'error':
            return
        raw = camera.getImage()
        if raw is None:
            return
        frame = np.frombuffer(raw, np.uint8).reshape(camera.getHeight(), camera.getWidth(), 4)[:, :, :3].copy()
        _latest(self.jobs, dict(frame=frame, time=now, pose=list(pose), fov=camera.getFov()))
        self.last_submit = now

    def poll(self, now):
        latest = None
        for _ in range(3):
            try:
                result = self.results.get_nowait()
            except Empty:
                break
            self.status = result['status']
            self.device = result.get('device', self.device)
            if 'error' in result:
                print('[vision] ' + result['error'], flush=True)
            if 'detections' in result and 0 <= now-result['time'] <= 3:
                self.tracker.update(result['detections'], result['time'])
                self.last_result_time = result['time']
                latest = result
        if not self.process.is_alive() and self.status != 'error':
            self.status = 'error'
            print('[vision] worker exited; detection unavailable', flush=True)
        if self.status == 'running' and self.last_result_time is not None and now-self.last_result_time > 5:
            self.status = 'stale'
        return latest

    def close(self):
        self.stop.set()
        self.process.join(timeout=1)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=1)
        for q in (self.jobs, self.results):
            q.cancel_join_thread()
            q.close()
