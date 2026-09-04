#!/usr/bin/env python3
"""Serve a V4L2 camera as a browser-compatible MJPEG stream."""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import threading
import time
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
from scipy.spatial import Delaunay
import yaml


class Camera:
    def __init__(self, device, width, height, fps):
        self.capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(cv2.CAP_PROP_FPS, fps)
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        if not self.capture.isOpened():
            raise RuntimeError(f'cannot open camera: {device}')
        self.lock = threading.Lock()

    def jpeg(self):
        with self.lock:
            ok, frame = self.capture.read()
        if not ok:
            return None
        ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return encoded.tobytes() if ok else None


class RosCamera:
    def __init__(self, topic):
        import rclpy
        from cv_bridge import CvBridge
        from rclpy.node import Node
        from sensor_msgs.msg import Image

        self.rclpy = rclpy
        self.bridge = CvBridge()
        self.frame = None
        self.lock = threading.Lock()
        rclpy.init()
        self.node = Node('camera_mjpeg_server')

        def on_image(message):
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            with self.lock:
                self.frame = frame

        self.node.create_subscription(Image, topic, on_image, 2)
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()

    def jpeg(self):
        with self.lock:
            frame = None if self.frame is None else self.frame.copy()
        if frame is None:
            return None
        ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return encoded.tobytes() if ok else None


class Calibration:
    def __init__(self, config_path, width, height):
        self.lock = threading.Lock()
        self.collecting = False
        self.image_points = []
        self.saved = False
        self.redo_index = None
        self.validation = None
        self.homography = None
        self.polynomial = None
        self.normalization = None
        self.triangulation = None
        self.piecewise_coefficients = None
        self.mean_homography_error_mm = None
        self.max_homography_error_mm = None
        self.message = 'Press START, then click P1 through P7 in order.'
        with open(config_path, encoding='utf-8') as stream:
            params = yaml.safe_load(stream)[
                'camera_homography_7point_calibration']['ros__parameters']
        self.reference_points = np.asarray(
            params['reference_points_link0'], dtype=np.float64).reshape((7, 2))
        self.output_file = os.path.abspath(str(params['output_file']))
        self.warning_error_mm = float(params.get('warning_mean_error_mm', 8.0))
        self.pixel_normalization = np.asarray(
            [width / 2.0, height / 2.0, width / 2.0, height / 2.0],
            dtype=np.float64)
        if os.path.exists(self.output_file):
            storage = cv2.FileStorage(self.output_file, cv2.FILE_STORAGE_READ)
            existing = storage.getNode('image_points').mat()
            references = storage.getNode('reference_points_link0').mat()
            self.homography = storage.getNode('homography').mat()
            polynomial = storage.getNode('polynomial_coefficients').mat()
            normalization = storage.getNode('pixel_normalization').mat()
            homography_errors = storage.getNode('homography_reprojection_errors_mm').mat()
            model = storage.getNode('coordinate_model').string()
            storage.release()
            if existing is not None and existing.shape == (7, 2):
                self.image_points = [tuple(point) for point in existing.tolist()]
                self.saved = True
                if homography_errors is not None:
                    values = homography_errors.reshape(-1).astype(float)
                    self.mean_homography_error_mm = float(np.mean(values))
                    self.max_homography_error_mm = float(np.max(values))
                if (model == 'piecewise_affine_v1' and references is not None):
                    self.triangulation = Delaunay(existing)
                    reference_array = references.reshape((-1, 2)).astype(float)
                    self.piecewise_coefficients = [
                        np.linalg.solve(
                            np.column_stack((np.ones(3), existing[vertices])),
                            reference_array[vertices])
                        for vertices in self.triangulation.simplices]
                elif (polynomial is not None and polynomial.shape == (6, 2) and
                      normalization is not None and normalization.size == 4):
                    self.polynomial = polynomial.astype(float)
                    self.normalization = normalization.reshape(4).astype(float)
                if self.mean_homography_error_mm is None:
                    self.message = 'Loaded saved calibration. Click an independent point to validate.'
                else:
                    quality = ('WARNING' if self.mean_homography_error_mm >
                               self.warning_error_mm else 'OK')
                    self.message = (
                        f'Loaded ({quality}) homography mean='
                        f'{self.mean_homography_error_mm:.2f}mm, '
                        f'max={self.max_homography_error_mm:.2f}mm. '
                        'Click an independent point to validate.')

    def start(self):
        with self.lock:
            self.collecting = True
            self.image_points = []
            self.saved = False
            self.redo_index = None
            self.validation = None
            self.message = 'Click P1.'

    def redo(self, point_number):
        with self.lock:
            if len(self.image_points) != 7 or not 1 <= point_number <= 7:
                return False
            self.collecting = True
            self.saved = False
            self.redo_index = point_number - 1
            self.validation = None
            self.message = f'Click P{point_number} again.'
            return True

    def undo(self):
        with self.lock:
            if self.image_points:
                self.image_points.pop()
            self.collecting = True
            self.saved = False
            self.redo_index = None
            self.validation = None
            self.message = f'Click P{len(self.image_points) + 1}.'

    def reset(self):
        with self.lock:
            self.collecting = False
            self.image_points = []
            self.saved = False
            self.redo_index = None
            self.validation = None
            self.message = 'Reset. Press START to begin.'

    def click(self, x, y):
        with self.lock:
            if not self.collecting:
                if self.saved:
                    world = self.world((float(x), float(y)))
                    if np.all(np.isfinite(world)):
                        self.validation = {
                            'pixel': [float(x), float(y)],
                            'link0': [float(world[0]), float(world[1])],
                        }
                        self.message = (
                            f'VALIDATE pixel=({x:.1f}, {y:.1f}) -> '
                            f'link0=({world[0]:+.5f}, {world[1]:+.5f})m')
                    else:
                        self.validation = None
                        self.message = 'Validation point is outside the calibrated area.'
                return
            if self.redo_index is not None:
                self.image_points[self.redo_index] = (float(x), float(y))
                self.redo_index = None
            else:
                if len(self.image_points) >= 7:
                    return
                self.image_points.append((float(x), float(y)))
                if len(self.image_points) < 7:
                    self.message = f'Captured P{len(self.image_points)}. Click P{len(self.image_points) + 1}.'
                    return
            image = np.asarray(self.image_points, dtype=np.float64)
            matrix, _mask = cv2.findHomography(image, self.reference_points, 0)
            if matrix is None or not np.isfinite(matrix).all():
                self.collecting = False
                self.message = 'Homography failed. Press START and try again.'
                return
            projected = cv2.perspectiveTransform(
                image.reshape((-1, 1, 2)), matrix).reshape((-1, 2))
            homography_errors = np.linalg.norm(
                projected - self.reference_points, axis=1) * 1000.0
            cx, cy, sx, sy = self.pixel_normalization
            u = (image[:, 0] - cx) / sx
            v = (image[:, 1] - cy) / sy
            design = np.column_stack((
                np.ones(len(image)), u, v, u * u, u * v, v * v))
            coefficients = np.linalg.lstsq(
                design, self.reference_points, rcond=None)[0]
            polynomial_projected = design @ coefficients
            quadratic_errors = np.linalg.norm(
                polynomial_projected - self.reference_points, axis=1) * 1000.0
            triangles = Delaunay(image).simplices.astype(np.int32)
            errors = np.zeros(len(image), dtype=np.float64)
            os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
            temporary = self.output_file + '.tmp'
            storage = cv2.FileStorage(temporary, cv2.FILE_STORAGE_WRITE)
            storage.write('homography', matrix)
            storage.write('image_points', image)
            storage.write('reference_points_link0', self.reference_points)
            storage.write('reprojection_errors_mm', errors.reshape((-1, 1)))
            storage.write(
                'homography_reprojection_errors_mm',
                homography_errors.reshape((-1, 1)))
            storage.write('pixel_normalization', self.pixel_normalization.reshape((1, 4)))
            storage.write('polynomial_coefficients', coefficients)
            storage.write(
                'quadratic_reprojection_errors_mm',
                quadratic_errors.reshape((-1, 1)))
            storage.write('piecewise_triangles', triangles)
            storage.write('coordinate_model', 'piecewise_affine_v1')
            storage.release()
            os.replace(temporary, self.output_file)
            self.homography = matrix
            self.polynomial = coefficients
            self.normalization = self.pixel_normalization.copy()
            self.triangulation = Delaunay(image)
            self.piecewise_coefficients = [
                np.linalg.solve(
                    np.column_stack((np.ones(3), image[vertices])),
                    self.reference_points[vertices])
                for vertices in self.triangulation.simplices]
            self.mean_homography_error_mm = float(np.mean(homography_errors))
            self.max_homography_error_mm = float(np.max(homography_errors))
            self.collecting = False
            self.saved = True
            self.validation = None
            quality = ('WARNING' if self.mean_homography_error_mm > self.warning_error_mm
                       else 'OK')
            self.message = (
                f'SAVED ({quality}) homography mean={self.mean_homography_error_mm:.2f}mm, '
                f'max={self.max_homography_error_mm:.2f}mm. Click an independent point.')

    def world(self, point):
        pixel = np.asarray(point, dtype=float)
        if self.triangulation is not None:
            simplex = int(self.triangulation.find_simplex(pixel))
            if simplex < 0:
                return np.asarray([float('nan'), float('nan')])
            return (np.asarray([1.0, pixel[0], pixel[1]]) @
                    self.piecewise_coefficients[simplex])
        if self.polynomial is not None and self.normalization is not None:
            cx, cy, sx, sy = self.normalization
            u, v = (pixel[0] - cx) / sx, (pixel[1] - cy) / sy
            return np.asarray([1.0, u, v, u*u, u*v, v*v]) @ self.polynomial
        if self.homography is None:
            return np.asarray([float('nan'), float('nan')])
        transformed = self.homography @ np.asarray([pixel[0], pixel[1], 1.0])
        if abs(float(transformed[2])) < 1.0e-12:
            return np.asarray([float('nan'), float('nan')])
        return transformed[:2] / transformed[2]

    def status(self):
        with self.lock:
            return {
                'collecting': self.collecting,
                'points': list(self.image_points),
                'references': self.reference_points.tolist(),
                'saved': self.saved,
                'validation': self.validation,
                'mean_homography_error_mm': self.mean_homography_error_mm,
                'max_homography_error_mm': self.max_homography_error_mm,
                'warning_mean_error_mm': self.warning_error_mm,
                'message': self.message,
                'output_file': self.output_file,
            }


def make_handler(camera, fps, calibration=None):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path in ('/', '/index.html'):
                body = """<!doctype html><html><head><meta charset="utf-8">
<title>OMX unload calibration</title><style>
body{margin:0;background:#111;color:#eee;font:16px sans-serif;text-align:center}
#view{position:relative;width:640px;height:480px;margin:12px auto}
#view img,#view canvas{position:absolute;left:0;top:0;width:640px;height:480px}
#view canvas{cursor:crosshair}button{font-size:16px;margin:5px;padding:8px 16px}
#status{margin:8px;color:#7CFC90}#refs{font:14px monospace;white-space:pre}
</style></head><body><div id="status">Loading...</div>
<div id="view"><img id="cam" src="/stream.mjpg"><canvas id="overlay" width="640" height="480"></canvas></div>
<button onclick="command('start')">START</button>
<button onclick="command('undo')">UNDO</button>
<button onclick="command('reset')">RESET</button>
<div>REDO: <button onclick="command('redo','?point=1')">P1</button>
<button onclick="command('redo','?point=2')">P2</button>
<button onclick="command('redo','?point=3')">P3</button>
<button onclick="command('redo','?point=4')">P4</button>
<button onclick="command('redo','?point=5')">P5</button>
<button onclick="command('redo','?point=6')">P6</button>
<button onclick="command('redo','?point=7')">P7</button></div>
<div>Independent point actual link0: X <input id="expectedX" size="8" placeholder="meters">
Y <input id="expectedY" size="8" placeholder="meters">
<button onclick="checkError()">CHECK ERROR</button></div>
<div id="validation"></div><div id="refs"></div>
<script>
const canvas=document.getElementById('overlay'),ctx=canvas.getContext('2d');
let state={points:[],references:[]};
async function command(name,query=''){await fetch('/'+name+query,{method:'POST'});await update();}
canvas.onclick=e=>{const r=canvas.getBoundingClientRect();
 const x=(e.clientX-r.left)*640/r.width,y=(e.clientY-r.top)*480/r.height;
 command('click','?x='+x+'&y='+y);};
function checkError(){const out=document.getElementById('validation');
 if(!state.validation){out.textContent='Click an independent image point first.';return;}
 const x=Number(document.getElementById('expectedX').value),y=Number(document.getElementById('expectedY').value);
 if(!Number.isFinite(x)||!Number.isFinite(y)){out.textContent='Enter measured X and Y in meters.';return;}
 const p=state.validation.link0,e=Math.hypot(p[0]-x,p[1]-y)*1000;
 out.textContent='Independent validation error: '+e.toFixed(2)+' mm (predicted X='+p[0].toFixed(5)+', Y='+p[1].toFixed(5)+')';}
async function update(){state=await (await fetch('/status',{cache:'no-store'})).json();
 document.getElementById('status').textContent=state.message;
 document.getElementById('refs').textContent=state.references.map((p,i)=>
  'P'+(i+1)+'  X='+p[0].toFixed(5)+'m  Y='+p[1].toFixed(5)+'m').join('\\n');
 ctx.clearRect(0,0,640,480);ctx.font='bold 18px sans-serif';
 state.points.forEach((p,i)=>{ctx.fillStyle='#ffff00';ctx.beginPath();ctx.arc(p[0],p[1],7,0,Math.PI*2);ctx.fill();
  ctx.fillText('P'+(i+1),p[0]+9,p[1]-9);});
 if(state.validation){const p=state.validation.pixel;ctx.strokeStyle='#00ffff';ctx.lineWidth=3;
  ctx.beginPath();ctx.arc(p[0],p[1],10,0,Math.PI*2);ctx.stroke();}}
setInterval(update,500);update();
</script></body></html>""".encode()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path == '/status':
                body = json.dumps(
                    calibration.status() if calibration else
                    {'message': 'Viewer only', 'points': [], 'references': []}
                ).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path != '/stream.mjpg':
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header(
                'Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            delay = 1.0 / max(1, fps)
            try:
                while True:
                    frame = camera.jpeg()
                    if frame is None:
                        time.sleep(0.05)
                        continue
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n')
                    self.wfile.write(
                        f'Content-Length: {len(frame)}\r\n\r\n'.encode())
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
                    time.sleep(delay)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            if calibration is None:
                self.send_error(400, 'calibration is not configured')
                return
            parsed = urlparse(self.path)
            if parsed.path == '/start':
                calibration.start()
            elif parsed.path == '/undo':
                calibration.undo()
            elif parsed.path == '/reset':
                calibration.reset()
            elif parsed.path == '/redo':
                query = parse_qs(parsed.query)
                if not calibration.redo(int(query['point'][0])):
                    self.send_error(400, 'cannot redo this point')
                    return
            elif parsed.path == '/click':
                query = parse_qs(parsed.query)
                calibration.click(float(query['x'][0]), float(query['y'][0]))
            else:
                self.send_error(404)
                return
            self.send_response(204)
            self.end_headers()

        def log_message(self, pattern, *args):
            print(f'{self.client_address[0]} {pattern % args}', flush=True)

    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', default='/dev/video2')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8088)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=int, default=15)
    parser.add_argument(
        '--ros-topic', default='',
        help='Read sensor_msgs/Image from this ROS topic instead of opening V4L2')
    parser.add_argument(
        '--calibration-config', default='',
        help='ROS YAML containing seven reference points and output_file')
    args = parser.parse_args()
    camera = (
        RosCamera(args.ros_topic) if args.ros_topic else
        Camera(args.device, args.width, args.height, args.fps))
    calibration = (
        Calibration(args.calibration_config, args.width, args.height)
        if args.calibration_config else None)
    server = ThreadingHTTPServer(
        (args.host, args.port), make_handler(camera, args.fps, calibration))
    print(
        f'Camera {args.ros_topic or args.device} available at '
        f'http://{args.host}:{args.port}/',
        flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
