import os
import cv2
import numpy as np
import time
import math
import base64
import requests
import threading
import queue
from datetime import datetime

import pyodbc
from djitellopy import Tello


FLOOR_MARKER_ID1_ALIGN = 11

FLOOR_MARKER_ID2_ALIGN = 2

FLOOR_MARKER_ID3_ALIGN = 3

FLOOR_MARKER_ID4_ALIGN = 4


RC_PERIOD = 0.05

EMA_ALPHA = 0.35


CENTER_BAND_RATIO_X = 0.075

CENTER_BAND_RATIO_Y = 0.075


MOVE_UP_CM = 150


MOVE_RIGHT_AFTER_ID1_PART1_CM = 200

MOVE_RIGHT_AFTER_ID1_PART2_CM = 330

MOVE_RIGHT_AFTER_ID2_PART1_CM = 200

MOVE_RIGHT_AFTER_ID2_PART2_CM = 330

MOVE_RIGHT_AFTER_ID3_PART1_CM = 200

MOVE_RIGHT_AFTER_ID3_PART2_CM = 330


MOVE_LEFT_RETURN_TO_ID3_PART1_CM = 200

MOVE_LEFT_RETURN_TO_ID3_PART2_CM = 330

MOVE_LEFT_RETURN_TO_ID2_PART1_CM = 200

MOVE_LEFT_RETURN_TO_ID2_PART2_CM = 330

MOVE_LEFT_RETURN_TO_ID1_PART1_CM = 200

MOVE_LEFT_RETURN_TO_ID1_PART2_CM = 330


HOVER_BEFORE_UP_S = 3.0


HOVER_AFTER_ID1_ALIGN_S = 35.0

HOVER_AFTER_ID2_ALIGN_S = 35.0

HOVER_AFTER_ID3_ALIGN_S = 35.0

HOVER_AFTER_ID4_ALIGN_S = 35.0

HOVER_AFTER_RETURN_ID3_ALIGN_S = 35.0

HOVER_AFTER_RETURN_ID2_ALIGN_S = 35.0

HOVER_AFTER_RETURN_ID1_ALIGN_S = 35.0


SEARCH_TIMEOUT_ID1_S = 15.0

SEARCH_TIMEOUT_ID2_S = 15.0

SEARCH_TIMEOUT_ID3_S = 15.0

SEARCH_TIMEOUT_ID4_S = 15.0

SEARCH_STABLE_N = 1


FLOOR_TARGET_EDGE_DEG = 90.0

FLOOR_TARGET_DIAG_DEG = FLOOR_TARGET_EDGE_DEG + 45.0


YAW_KP = 2.0

MAX_YAW_RC = 30

MIN_YAW_RC = 10

YAW_EXIT_DEG = 3.0

STABLE_N = 8


ALIGN_TIMEOUT_S = 30.0

LOST_TIMEOUT_S = 0.30


YAW_SLEW_STEP = 6

YAW_SIGN = 1


FB_KP = 35.0

MAX_FB_RC = 12

MIN_FB_RC = 6

FB_SLEW_STEP = 3


LR_KP = 35.0

MAX_LR_RC = 12

MIN_LR_RC = 6

LR_SLEW_STEP = 3

LR_SIGN_Y = 1


MOVE_SPEED_CM_S = 30

MOVE_MARGIN_S = 0.5


AUTO_LAND_ON_EXIT = False


VIDEO_SAVE_DIR = "frontcam_recordings"

VIDEO_FPS = 30.0

VIDEO_EXT = ".mp4"
VIDEO_FRAME_INTERVAL = 1.0 / VIDEO_FPS


FRONT_RECORD_DELAY_S = 1.0


FRONT_MIN_W = 640

FRONT_MIN_H = 480


SERVER_BASE_URL = ""

SERVER_URL = ""

UPLOAD_VIDEO_URL = ""

SERVER_TOKEN = os.getenv("SERVER_TOKEN", "")
STREAM_VIDEO_BY_MARKER_URL_TEMPLATE = ""


CAM_ID = 0

MAX_SEND_FPS = 5

SERVER_RESIZE_W = 640

SERVER_JPEG_QUALITY = 65
VIDEO_UPLOAD_CONNECT_TIMEOUT = 3
VIDEO_UPLOAD_READ_TIMEOUT = 120
sess = requests.Session()
video_upload_sess = requests.Session()


DB_HOST = ""

DB_PORT = 1433

DB_NAME = ""

DB_USER = ""

DB_PASS = os.getenv("DB_PASS", "")


DRONE_ID_FIXED = "DR001"

CONNECTED_FIXED = "true"

INTERVAL_SEC = 5


send_q = queue.Queue(maxsize=1)

upload_q = queue.Queue(maxsize=10)

stop_evt = threading.Event()

tello_lock = threading.Lock()
th_sender = None
th_db = None
th_uploader = None


cameraMatrix = np.array([
    [920.0,   0.0, 480.0],
    [  0.0, 920.0, 360.0],
    [  0.0,   0.0,   1.0]
], dtype=np.float32)


distCoeffs = np.array([-0.18, 0.06, 0.000, 0.000, -0.01], dtype=np.float32).reshape(-1, 1)


aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, params)


def clamp_int(x, low, high):
\
\
\
\

    return int(max(low, min(high, x)))


def ema(prev, x, a):
\
\
\
\
\

    return x if prev is None else (a * x + (1.0 - a) * prev)


def slew(prev, target, step):
\
\
\
\
\

    if prev is None:
        return target
    if target > prev + step:
        return prev + step
    if target < prev - step:
        return prev - step
    return target


def parallel_err_deg(angle_deg):
\
\

    return ((angle_deg + 90.0) % 180.0) - 90.0


def undistort_pts(px_pts_4x2):
\
\
\
\
\

    pts = px_pts_4x2.reshape(-1, 1, 2).astype(np.float32)
    und = cv2.undistortPoints(pts, cameraMatrix, distCoeffs)
    return und.reshape(-1, 2)


def put_latest(q: queue.Queue, frame):
\
\
\
\
\

    try:
        q.put_nowait(frame)
    except queue.Full:
        try:
            _ = q.get_nowait()
        except queue.Empty:
            pass
        try:
            q.put_nowait(frame)
        except queue.Full:
            pass


def now_yyyymmddhhmmss():
\
\
\
\

    return datetime.now().strftime("%Y%m%d%H%M%S")


def connect_db():
\
\
\
\
\

    if not DB_PASS:
        raise RuntimeError(
            '환경변수 DB_PASS가 없습니다.\n'
            'PowerShell: setx DB_PASS "비번" 후 새 터미널에서 재실행하세요.'
        )

    conn_str = (
        "Driver={ODBC Driver 18 for SQL Server};"
        f"Server=tcp:{DB_HOST},{DB_PORT};"
        f"Database={DB_NAME};"
        f"Uid={DB_USER};"
        f"Pwd={DB_PASS};"
        "Encrypt=yes;"
        "TrustServerCertificate=yes;"
        "Connection Timeout=5;"
    )
    return pyodbc.connect(conn_str)


def insert_status(conn, drone_id: str, update_time: str, armed: str, connected: str, percent: float):
\
\
\
\

    sql = """
    INSERT INTO dbo.T_DRONE_STATUS ([DRONE_ID], [UPDATE_TIME], [ARMED], [CONNECTED], [PERCENT])
    VALUES (?, ?, ?, ?, ?)
    """
    cur = conn.cursor()
    cur.execute(sql, (drone_id, update_time, armed, connected, float(percent)))
    conn.commit()


def safe_set_downvision(tello_obj, enable: bool):
\
\
\
\
\

    try:
        cmd = f"downvision {1 if enable else 0}"

        with tello_lock:
            resp = tello_obj.send_command_with_return(cmd)
        print(cmd, "->", resp)
        return resp
    except Exception as e:
        print("[WARN] downvision switch failed:", e)
        return None


def start_sdk_move(tello_obj, move_func, dist_cm):
\
\
\
\
\

    try:

        with tello_lock:
            tello_obj.send_rc_control(0, 0, 0, 0)
    except Exception:
        pass
    time.sleep(0.1)

    try:
        with tello_lock:
            tello_obj.set_speed(int(MOVE_SPEED_CM_S))
    except Exception as e:
        print("[WARN] set_speed failed:", e)

    with tello_lock:
        move_func(int(dist_cm))
    return time.time() + MOVE_MARGIN_S


def start_sdk_move_split(tello_obj, move_func, distances_cm, label: str):
\
\
\
\
\

    try:
        with tello_lock:
            tello_obj.send_rc_control(0, 0, 0, 0)
    except Exception:
        pass
    time.sleep(0.1)

    try:
        with tello_lock:
            tello_obj.set_speed(int(MOVE_SPEED_CM_S))
    except Exception as e:
        print("[WARN] set_speed failed:", e)

    print(f"[TEST] {label} " + " + ".join(str(int(d)) for d in distances_cm))
    for idx, dist_cm in enumerate(distances_cm):
        with tello_lock:
            move_func(int(dist_cm))
        if idx != len(distances_cm) - 1:
            time.sleep(0.5)

    return time.time() + MOVE_MARGIN_S


def make_video_path(marker_id: int):
\
\
\
\
\

    os.makedirs(VIDEO_SAVE_DIR, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d%H%M")
    base = os.path.join(VIDEO_SAVE_DIR, f"{ts}_id{marker_id}{VIDEO_EXT}")

    if not os.path.exists(base):
        return base

    idx = 1
    while True:
        alt = os.path.join(VIDEO_SAVE_DIR, f"{ts}_id{marker_id}_{idx}{VIDEO_EXT}")
        if not os.path.exists(alt):
            return alt
        idx += 1


def queue_video_for_upload(video_path: str, marker_id: int):
    if not video_path:
        return

    item = {
        "path": video_path,
        "marker_id": marker_id,
        "drone_id": DRONE_ID_FIXED,
        "cam_id": CAM_ID,
        "saved_at": now_yyyymmddhhmmss(),
    }

    try:
        upload_q.put_nowait(item)
        print(f"[UPLOAD] queued -> {video_path}")
    except queue.Full:
        print(f"[WARN] upload queue full, drop -> {video_path}")


def stop_video_recording():

    global video_writer, video_recording_path, video_recording_marker_id, video_recording_size
    global last_video_write_t

    saved_path = video_recording_path
    saved_marker_id = video_recording_marker_id

    if video_writer is not None:
        try:


            video_writer.release()
            time.sleep(0.5)

            print(f"[REC] saved -> {saved_path}")

            if saved_path and os.path.exists(saved_path):
                print(f"[REC] file size -> {os.path.getsize(saved_path)} bytes")
            else:
                print(f"[WARN] recorded file missing -> {saved_path}")

            queue_video_for_upload(saved_path, saved_marker_id)
        except Exception as e:
            print("[WARN] release video failed:", e)

    video_writer = None
    video_recording_path = None
    video_recording_marker_id = None
    video_recording_size = None
    last_video_write_t = 0.0


def arm_front_record(marker_id: int):
\
\
\
\
\

    global front_record_armed, front_record_arm_t, front_record_target_id
    stop_video_recording()
    front_record_armed = (marker_id is not None)
    front_record_target_id = marker_id
    front_record_arm_t = time.time()


def disarm_front_record():
\
\
\
\

    global front_record_armed, front_record_arm_t, front_record_target_id
    front_record_armed = False
    front_record_arm_t = 0.0
    front_record_target_id = None
    stop_video_recording()


def update_video_recording(frame_bgr, should_record: bool, marker_id: int):
\
\
\
\
\
\

    global video_writer, video_recording_path, video_recording_marker_id, video_recording_size
    global last_video_write_t

    if (not should_record) or (marker_id is None):
        stop_video_recording()
        return

    h, w = frame_bgr.shape[:2]
    curr_size = (w, h)

    if (
        video_writer is None
        or video_recording_marker_id != marker_id
        or video_recording_size != curr_size
    ):
        stop_video_recording()

        path = make_video_path(marker_id)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(path, fourcc, VIDEO_FPS, curr_size)

        if not writer.isOpened():
            print("[WARN] video writer open failed:", path)
            return

        video_writer = writer
        video_recording_path = path
        video_recording_marker_id = marker_id
        video_recording_size = curr_size
        last_video_write_t = 0.0
        print(f"[REC] start -> {video_recording_path} size={curr_size}")

    now = time.time()

    if last_video_write_t != 0.0 and (now - last_video_write_t) < VIDEO_FRAME_INTERVAL:
        return

    try:
        video_writer.write(frame_bgr)
        last_video_write_t = now
    except Exception as e:
        print("[WARN] video write failed:", e)
        stop_video_recording()


def switch_to_front_camera(tello_obj):
\
\
\
\
\

    global is_downvision_active
    safe_set_downvision(tello_obj, False)
    is_downvision_active = False
    time.sleep(0.3)


def switch_to_down_camera(tello_obj):
\
\
\
\
\

    global is_downvision_active
    disarm_front_record()
    safe_set_downvision(tello_obj, True)
    is_downvision_active = True
    time.sleep(0.3)


def marker_video_view_url(marker_id: int):
\
\
\
\

    if marker_id is None:
        return None
    return STREAM_VIDEO_BY_MARKER_URL_TEMPLATE.format(marker_id=marker_id)


def wait_until_file_ready(path: str, checks: int = 3, interval: float = 0.3) -> int:
\
\
\
\
\

\
\
\

    last_size = -1
    stable_count = 0

    for _ in range(20):
        if not os.path.exists(path):
            time.sleep(interval)
            continue

        size = os.path.getsize(path)

        if size > 0 and size == last_size:
            stable_count += 1
            if stable_count >= checks:
                return size
        else:
            stable_count = 0
            last_size = size

        time.sleep(interval)

    if os.path.exists(path):
        return os.path.getsize(path)
    return 0

def upload_video_file(video_path: str, marker_id: int, drone_id: str, cam_id: int, saved_at: str):
\
\
\
\
\
\

\
\
\
\
\
\
\
\
\
\
\
\

    file_size = wait_until_file_ready(video_path)
    if file_size <= 0:
        raise RuntimeError(f"video file is empty or not ready: {video_path}")

    safe_filename = os.path.basename(video_path).replace("\\", "_").replace("/", "_")

    with open(video_path, "rb") as f:
        video_bytes = f.read()

    if len(video_bytes) != file_size:
        print(f"[UPLOAD] size changed while reading: stat={file_size}, read={len(video_bytes)}")
        file_size = len(video_bytes)

    data = {
        "marker_id": "" if marker_id is None else str(marker_id),
        "drone_id": str(drone_id),
        "cam_id": str(cam_id),
        "saved_at": str(saved_at),
    }

    files = {
        "file": (safe_filename, video_bytes, "video/mp4")
    }


    headers = {
        "X-API-Key": str(SERVER_TOKEN),
    }

    req = requests.Request(
        "POST",
        UPLOAD_VIDEO_URL,
        files=files,
        data=data,
        headers=headers,
    )
    prepared = req.prepare()

    content_type = prepared.headers.get("Content-Type", "")
    content_length = prepared.headers.get("Content-Length", "")

    print("[UPLOAD] POST ->", UPLOAD_VIDEO_URL)
    print("[UPLOAD] file ->", video_path)
    print("[UPLOAD] safe_filename ->", safe_filename)
    print("[UPLOAD] size ->", file_size)
    print("[UPLOAD] data ->", data)
    print("[UPLOAD] prepared Content-Type ->", content_type)
    print("[UPLOAD] prepared Content-Length ->", content_length)

    if "multipart/form-data" not in content_type or "boundary=" not in content_type:
        raise RuntimeError(
            "multipart Content-Type boundary was not generated. "
            f"Content-Type={content_type!r}"
        )


    s = requests.Session()
    s.trust_env = False

    r = s.send(
        prepared,
        timeout=(VIDEO_UPLOAD_CONNECT_TIMEOUT, VIDEO_UPLOAD_READ_TIMEOUT),
    )

    print(f"[UPLOAD] response: {r.status_code} {r.text[:500]}")
    r.raise_for_status()
    return r


def upload_worker():
\
\
\
\
\

    while True:

        if stop_evt.is_set() and upload_q.empty():
            break

        try:
            item = upload_q.get(timeout=0.5)
        except queue.Empty:
            continue

        video_path = item.get("path")
        marker_id = item.get("marker_id")
        drone_id = item.get("drone_id", DRONE_ID_FIXED)
        cam_id = item.get("cam_id", CAM_ID)
        saved_at = item.get("saved_at", now_yyyymmddhhmmss())

        try:
            if not video_path or not os.path.exists(video_path):
                print(f"[WARN] upload file missing -> {video_path}")
                continue

            resp = upload_video_file(video_path, marker_id, drone_id, cam_id, saved_at)
            print(f"[UPLOAD] success -> {video_path} | status={resp.status_code}")
            view_url = marker_video_view_url(marker_id)
            if view_url:
                print(f"[UPLOAD] view -> {view_url}")
        except Exception as e:
            print(f"[UPLOAD] failed -> {video_path} | {e}")

        finally:
            try:
                upload_q.task_done()
            except Exception:
                pass

    print("[UPLOAD] worker stopped.")


def sender_worker():
\
\
\
\
\

    min_interval = 1.0 / max(1, MAX_SEND_FPS)
    last_sent = 0.0

    while not stop_evt.is_set():
        try:
            frame = send_q.get(timeout=0.5)
        except queue.Empty:
            continue

        now = time.time()
        if now - last_sent < min_interval:
            continue

        frame = frame.copy()


        h, w = frame.shape[:2]
        if SERVER_RESIZE_W and w > SERVER_RESIZE_W:
            new_h = int(h * (SERVER_RESIZE_W / w))
            frame = cv2.resize(frame, (SERVER_RESIZE_W, new_h))

        ok, buf = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), SERVER_JPEG_QUALITY],
        )
        if not ok:
            continue

        payload = {
            "cam_id": int(CAM_ID),
            "image": base64.b64encode(buf).decode("utf-8"),
        }

        try:
            r = sess.post(SERVER_URL, json=payload, headers={"X-API-Key": str(SERVER_TOKEN)}, timeout=(0.7, 2.0))
            if r.status_code != 200:
                print("server status:", r.status_code, "body:", r.text[:200])
        except Exception as e:
            print("post error:", e)

        finally:
            last_sent = time.time()


def db_worker():
\
\
\
\
\

    global tello, is_flying

    conn = None
    try:
        conn = connect_db()
        print("DB connected.")
    except Exception as e:
        print("DB connect error:", e)
        conn = None

    while not stop_evt.is_set():
        if stop_evt.wait(INTERVAL_SEC):
            break

        if conn is None:
            try:
                conn = connect_db()
                print("DB reconnected.")
            except Exception as e:
                print("DB reconnect error:", e)
                continue

        try:
            with tello_lock:
                update_time = now_yyyymmddhhmmss()
                try:
                    battery = tello.get_battery()
                except Exception:
                    battery = -1
                armed = "true" if is_flying else "false"

            insert_status(conn, DRONE_ID_FIXED, update_time, armed, CONNECTED_FIXED, battery)

            print(
                f"Inserted: DRONE_ID={DRONE_ID_FIXED}, UPDATE_TIME={update_time}, "
                f"ARMED={armed}, CONNECTED={CONNECTED_FIXED}, PERCENT={battery}"
            )
        except Exception as e:
            print("DB insert error:", e)
            try:
                conn.close()
            except Exception:
                pass
            conn = None

    try:
        if conn is not None:
            conn.close()
    except Exception:
        pass

    print("DB worker stopped.")


MODE_IDLE             = "IDLE"

MODE_HOVER_PRE_UP     = "HOVER_PRE_UP"

MODE_SWITCH_DOWNCAM   = "SWITCH_DOWNVISION"

MODE_MOVE_UP          = "MOVE_UP_130"


MODE_SEARCH_ID1       = "SEARCH_ID1"

MODE_ALIGN_ID1        = "ALIGN_ID1 (YAW->FB->LR)"

MODE_HOVER_AFTER_ID1  = "HOVER_AFTER_ID1_10"

MODE_MOVE_RIGHT_270   = "MOVE_RIGHT_270"


MODE_SEARCH_ID2       = "SEARCH_ID2"

MODE_ALIGN_ID2        = "ALIGN_ID2 (YAW->FB->LR)"

MODE_HOVER_AFTER_ID2  = "HOVER_AFTER_ID2_10"

MODE_MOVE_RIGHT_AFTER_ID2 = "MOVE_RIGHT_AFTER_ID2_180"


MODE_SEARCH_ID3       = "SEARCH_ID3"

MODE_ALIGN_ID3        = "ALIGN_ID3 (YAW->FB->LR)"

MODE_HOVER_AFTER_ID3  = "HOVER_AFTER_ID3_10"

MODE_MOVE_RIGHT_AFTER_ID3 = "MOVE_RIGHT_AFTER_ID3_180"


MODE_SEARCH_ID4       = "SEARCH_ID4"

MODE_ALIGN_ID4        = "ALIGN_ID4 (YAW->FB->LR)"

MODE_HOVER_AFTER_ID4  = "HOVER_AFTER_ID4_10"


MODE_MOVE_LEFT_AFTER_ID4         = "MOVE_LEFT_AFTER_ID4_180"

MODE_SEARCH_RETURN_ID3           = "SEARCH_RETURN_ID3"

MODE_ALIGN_RETURN_ID3            = "ALIGN_RETURN_ID3 (YAW->FB->LR)"

MODE_HOVER_AFTER_RETURN_ID3      = "HOVER_AFTER_RETURN_ID3_10"

MODE_MOVE_LEFT_AFTER_RETURN_ID3  = "MOVE_LEFT_AFTER_RETURN_ID3_180"

MODE_SEARCH_RETURN_ID2           = "SEARCH_RETURN_ID2"

MODE_ALIGN_RETURN_ID2            = "ALI이N_RETURN_ID2 (YAW->FB->LR)"

MODE_HOVER_AFTER_RETURN_ID2      = "HOVER_AFTER_RETURN_ID2_10"

MODE_MOVE_LEFT_AFTER_RETURN_ID2  = "MOVE_LEFT_AFTER_RETURN_ID2_270"

MODE_SEARCH_RETURN_ID1           = "SEARCH_RETURN_ID1"

MODE_ALIGN_RETURN_ID1            = "ALIGN_RETURN_ID1 (YAW->FB->LR)"

MODE_HOVER_AFTER_RETURN_ID1      = "HOVER_AFTER_RETURN_ID1_10"


MODE_SEARCH_POST_RECORD          = "SEARCH_POST_RECORD"

MODE_ALIGN_POST_RECORD           = "ALIGN_POST_RECORD (YAW->FB->LR)"


MODE_LAND             = "LAND"


mode = MODE_IDLE


tello = Tello()
with tello_lock:
    tello.connect()
    print("Battery:", tello.get_battery(), "%")
    tello.streamon()
    frame_read = tello.get_frame_read()

is_flying = False
is_downvision_active = False
last_rc_sent = 0.0


hover_start_t = None
move_end_t = None

search_start_t = None
search_seen_cnt = 0
search_target_id = FLOOR_MARKER_ID1_ALIGN
search_timeout_s = SEARCH_TIMEOUT_ID1_S

align_start_t = None
last_seen_t = 0.0
yaw_f = None
stable_cnt = 0
target_yaw_cmd = 0
target_fb_cmd = 0
target_lr_cmd = 0

prev_yaw = 0
prev_fb = 0
prev_lr = 0

last_seen_marker_id = None

video_writer = None
video_recording_path = None
video_recording_marker_id = None
video_recording_size = None
last_video_write_t = 0.0

front_record_armed = False
front_record_arm_t = 0.0
front_record_target_id = None

post_record_realign_id = None
post_record_next_mode = None

marker_cx = marker_cy = None
control_mode = ""
active_align_id = None

print("\n[KEYS] t=takeoff(start), q=land, ESC=quit")
print("[FLOW]")
print("  Takeoff -> HOVER 3s (front cam, no record) -> downvision ON -> UP 130cm")
print("  SEARCH ID1 -> ALIGN ID1 -> remember ID1 -> HOVER 10s (front cam, record) -> RE-ALIGN ID1 -> RIGHT 135+135")
print("  SEARCH ID2 -> ALIGN ID2 -> remember ID2 -> HOVER 10s (front cam, record) -> RE-ALIGN ID2 -> RIGHT 90+90")
print("  SEARCH ID3 -> ALIGN ID3 -> remember ID3 -> HOVER 10s (front cam, record) -> RE-ALIGN ID3 -> RIGHT 90+90")
print("  SEARCH ID4 -> ALIGN ID4 -> remember ID4 -> HOVER 10s (front cam, record) -> RE-ALIGN ID4 -> LEFT 90+90 -> SEARCH RETURN ID3")
print("  SEARCH RETURN ID3 -> ALIGN RETURN ID3 -> remember ID3 -> HOVER 10s (front cam, record) -> RE-ALIGN ID3 -> LEFT 90+90 -> SEARCH RETURN ID2")
print("  SEARCH RETURN ID2 -> ALIGN RETURN ID2 -> remember ID2 -> HOVER 10s (front cam, record) -> RE-ALIGN ID2 -> LEFT 135+135 -> SEARCH RETURN ID1")
print("  SEARCH RETURN ID1 -> ALIGN RETURN ID1 -> remember ID1 -> HOVER 10s (front cam, record) -> RE-ALIGN ID1 -> LAND\n")


def reset_align_state():
\
\
\
\

    global align_start_t, last_seen_t, yaw_f, stable_cnt
    global target_yaw_cmd, target_fb_cmd, target_lr_cmd
    global prev_yaw, prev_fb, prev_lr
    global marker_cx, marker_cy, control_mode

    align_start_t = time.time()
    last_seen_t = time.time()
    yaw_f = None
    stable_cnt = 0
    target_yaw_cmd = 0
    target_fb_cmd = 0
    target_lr_cmd = 0
    prev_yaw = prev_fb = prev_lr = 0
    marker_cx = marker_cy = None
    control_mode = ""


def begin_post_record_realign(marker_id: int, next_mode: str, timeout_s: float):
\
\
\

    global mode, post_record_realign_id, post_record_next_mode
    global search_target_id, search_timeout_s, search_start_t, search_seen_cnt
    global hover_start_t, move_end_t


    switch_to_down_camera(tello)
    post_record_realign_id = marker_id
    post_record_next_mode = next_mode

    mode = MODE_SEARCH_POST_RECORD
    search_target_id = marker_id
    search_timeout_s = timeout_s
    search_start_t = time.time()
    search_seen_cnt = 0

    hover_start_t = None
    move_end_t = None


th_sender = threading.Thread(target=sender_worker, daemon=True)
th_db = threading.Thread(target=db_worker, daemon=True)
th_uploader = threading.Thread(target=upload_worker, daemon=True)

th_sender.start()

th_db.start()

th_uploader.start()

try:

    while True:

        frame = frame_read.frame

        if frame is None:
            continue


        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        h, w = frame.shape[:2]

        cx = w / 2.0

        cy = h / 2.0

        band_x = CENTER_BAND_RATIO_X * w
        left_band = int(cx - band_x)
        right_band = int(cx + band_x)

        band_y = CENTER_BAND_RATIO_Y * h
        top_band = int(cy - band_y)
        bottom_band = int(cy + band_y)


        need_detection = mode in [
            MODE_SEARCH_ID1, MODE_ALIGN_ID1,
            MODE_SEARCH_ID2, MODE_ALIGN_ID2,
            MODE_SEARCH_ID3, MODE_ALIGN_ID3,
            MODE_SEARCH_ID4, MODE_ALIGN_ID4,
            MODE_SEARCH_RETURN_ID3, MODE_ALIGN_RETURN_ID3,
            MODE_SEARCH_RETURN_ID2, MODE_ALIGN_RETURN_ID2,
            MODE_SEARCH_RETURN_ID1, MODE_ALIGN_RETURN_ID1,
            MODE_SEARCH_POST_RECORD, MODE_ALIGN_POST_RECORD,
        ]

        corners = ids = None

        id1_visible = False
        id1_c4 = None
        id2_visible = False
        id2_c4 = None
        id3_visible = False
        id3_c4 = None
        id4_visible = False
        id4_c4 = None

        if need_detection:

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            corners, ids, _ = detector.detectMarkers(gray)

            if ids is not None and len(ids) > 0:

                ids_list = [int(x) for x in ids.flatten().tolist()]

                if FLOOR_MARKER_ID1_ALIGN in ids_list:
                    idx = ids_list.index(FLOOR_MARKER_ID1_ALIGN)
                    id1_visible = True
                    id1_c4 = corners[idx][0]

                if FLOOR_MARKER_ID2_ALIGN in ids_list:
                    idx = ids_list.index(FLOOR_MARKER_ID2_ALIGN)
                    id2_visible = True
                    id2_c4 = corners[idx][0]

                if FLOOR_MARKER_ID3_ALIGN in ids_list:
                    idx = ids_list.index(FLOOR_MARKER_ID3_ALIGN)
                    id3_visible = True
                    id3_c4 = corners[idx][0]

                if FLOOR_MARKER_ID4_ALIGN in ids_list:
                    idx = ids_list.index(FLOOR_MARKER_ID4_ALIGN)
                    id4_visible = True
                    id4_c4 = corners[idx][0]

        if mode in [MODE_SEARCH_ID1, MODE_ALIGN_ID1, MODE_SEARCH_RETURN_ID1, MODE_ALIGN_RETURN_ID1] and id1_visible:

            last_seen_marker_id = FLOOR_MARKER_ID1_ALIGN
        elif mode in [MODE_SEARCH_ID2, MODE_ALIGN_ID2, MODE_SEARCH_RETURN_ID2, MODE_ALIGN_RETURN_ID2] and id2_visible:

            last_seen_marker_id = FLOOR_MARKER_ID2_ALIGN
        elif mode in [MODE_SEARCH_ID3, MODE_ALIGN_ID3, MODE_SEARCH_RETURN_ID3, MODE_ALIGN_RETURN_ID3] and id3_visible:

            last_seen_marker_id = FLOOR_MARKER_ID3_ALIGN
        elif mode in [MODE_SEARCH_ID4, MODE_ALIGN_ID4] and id4_visible:

            last_seen_marker_id = FLOOR_MARKER_ID4_ALIGN
        elif mode in [MODE_SEARCH_POST_RECORD, MODE_ALIGN_POST_RECORD]:
            if post_record_realign_id == FLOOR_MARKER_ID1_ALIGN and id1_visible:

                last_seen_marker_id = FLOOR_MARKER_ID1_ALIGN
            elif post_record_realign_id == FLOOR_MARKER_ID2_ALIGN and id2_visible:

                last_seen_marker_id = FLOOR_MARKER_ID2_ALIGN
            elif post_record_realign_id == FLOOR_MARKER_ID3_ALIGN and id3_visible:

                last_seen_marker_id = FLOOR_MARKER_ID3_ALIGN
            elif post_record_realign_id == FLOOR_MARKER_ID4_ALIGN and id4_visible:

                last_seen_marker_id = FLOOR_MARKER_ID4_ALIGN

        if mode in [
            MODE_SEARCH_ID1, MODE_SEARCH_ID2, MODE_SEARCH_ID3, MODE_SEARCH_ID4,
            MODE_SEARCH_RETURN_ID3, MODE_SEARCH_RETURN_ID2, MODE_SEARCH_RETURN_ID1,
            MODE_SEARCH_POST_RECORD,
        ] and ids is not None:

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)


        if mode in [
            MODE_SEARCH_ID1, MODE_SEARCH_ID2, MODE_SEARCH_ID3, MODE_SEARCH_ID4,
            MODE_SEARCH_RETURN_ID3, MODE_SEARCH_RETURN_ID2, MODE_SEARCH_RETURN_ID1,
            MODE_SEARCH_POST_RECORD,
        ] and is_flying:

            if search_start_t is None:
                search_start_t = time.time()

            if mode in [MODE_SEARCH_ID1, MODE_SEARCH_RETURN_ID1]:
                found = id1_visible
            elif mode in [MODE_SEARCH_ID2, MODE_SEARCH_RETURN_ID2]:
                found = id2_visible
            elif mode in [MODE_SEARCH_ID3, MODE_SEARCH_RETURN_ID3]:
                found = id3_visible
            elif mode == MODE_SEARCH_ID4:
                found = id4_visible
            else:
                if post_record_realign_id == FLOOR_MARKER_ID1_ALIGN:
                    found = id1_visible
                elif post_record_realign_id == FLOOR_MARKER_ID2_ALIGN:
                    found = id2_visible
                elif post_record_realign_id == FLOOR_MARKER_ID3_ALIGN:
                    found = id3_visible
                else:
                    found = id4_visible


            search_seen_cnt = search_seen_cnt + 1 if found else 0


            if search_seen_cnt >= SEARCH_STABLE_N:
                if mode == MODE_SEARCH_ID1:
                    mode = MODE_ALIGN_ID1
                    active_align_id = FLOOR_MARKER_ID1_ALIGN

                    reset_align_state()
                elif mode == MODE_SEARCH_ID2:
                    mode = MODE_ALIGN_ID2
                    active_align_id = FLOOR_MARKER_ID2_ALIGN

                    reset_align_state()
                elif mode == MODE_SEARCH_ID3:
                    mode = MODE_ALIGN_ID3
                    active_align_id = FLOOR_MARKER_ID3_ALIGN

                    reset_align_state()
                elif mode == MODE_SEARCH_ID4:
                    mode = MODE_ALIGN_ID4
                    active_align_id = FLOOR_MARKER_ID4_ALIGN

                    reset_align_state()
                elif mode == MODE_SEARCH_RETURN_ID3:
                    mode = MODE_ALIGN_RETURN_ID3
                    active_align_id = FLOOR_MARKER_ID3_ALIGN

                    reset_align_state()
                elif mode == MODE_SEARCH_RETURN_ID2:
                    mode = MODE_ALIGN_RETURN_ID2
                    active_align_id = FLOOR_MARKER_ID2_ALIGN

                    reset_align_state()
                elif mode == MODE_SEARCH_RETURN_ID1:
                    mode = MODE_ALIGN_RETURN_ID1
                    active_align_id = FLOOR_MARKER_ID1_ALIGN

                    reset_align_state()
                else:
                    mode = MODE_ALIGN_POST_RECORD
                    active_align_id = post_record_realign_id

                    reset_align_state()

                search_start_t = None
                search_seen_cnt = 0


            if search_start_t is not None and (time.time() - search_start_t) >= search_timeout_s:
                print(f"[INFO] ID{search_target_id} not found within {search_timeout_s}s -> LAND")
                mode = MODE_LAND
                search_start_t = None
                search_seen_cnt = 0


        if mode in [
            MODE_ALIGN_ID1, MODE_ALIGN_ID2, MODE_ALIGN_ID3, MODE_ALIGN_ID4,
            MODE_ALIGN_RETURN_ID3, MODE_ALIGN_RETURN_ID2, MODE_ALIGN_RETURN_ID1,
            MODE_ALIGN_POST_RECORD,
        ] and is_flying:

            if align_start_t is not None and (time.time() - align_start_t) >= ALIGN_TIMEOUT_S:
                print(f"[INFO] ID{active_align_id} align timeout -> LAND")
                mode = MODE_LAND
                target_yaw_cmd = target_fb_cmd = target_lr_cmd = 0
            else:
                if mode in [MODE_ALIGN_ID1, MODE_ALIGN_RETURN_ID1]:
                    visible = id1_visible
                    c4 = id1_c4
                    mid = FLOOR_MARKER_ID1_ALIGN
                elif mode in [MODE_ALIGN_ID2, MODE_ALIGN_RETURN_ID2]:
                    visible = id2_visible
                    c4 = id2_c4
                    mid = FLOOR_MARKER_ID2_ALIGN
                elif mode in [MODE_ALIGN_ID3, MODE_ALIGN_RETURN_ID3]:
                    visible = id3_visible
                    c4 = id3_c4
                    mid = FLOOR_MARKER_ID3_ALIGN
                elif mode == MODE_ALIGN_ID4:
                    visible = id4_visible
                    c4 = id4_c4
                    mid = FLOOR_MARKER_ID4_ALIGN
                else:
                    mid = active_align_id
                    if mid == FLOOR_MARKER_ID1_ALIGN:
                        visible = id1_visible
                        c4 = id1_c4
                    elif mid == FLOOR_MARKER_ID2_ALIGN:
                        visible = id2_visible
                        c4 = id2_c4
                    elif mid == FLOOR_MARKER_ID3_ALIGN:
                        visible = id3_visible
                        c4 = id3_c4
                    else:
                        visible = id4_visible
                        c4 = id4_c4


                if visible and (c4 is not None):
                    last_seen_t = time.time()


                    marker_cx = float(np.mean(c4[:, 0]))

                    marker_cy = float(np.mean(c4[:, 1]))

                    centered_x = (left_band <= marker_cx <= right_band)

                    centered_y = (top_band <= marker_cy <= bottom_band)


                    und = undistort_pts(c4)

                    tl = und[0]

                    br = und[2]

                    v = br - tl

                    diag_angle = math.degrees(math.atan2(float(v[1]), float(v[0])))


                    err = parallel_err_deg(diag_angle - FLOOR_TARGET_DIAG_DEG)

                    yaw_f = ema(yaw_f, err, EMA_ALPHA)


                    if abs(yaw_f) > YAW_EXIT_DEG:
                        stable_cnt = 0
                        raw_yaw = YAW_SIGN * (YAW_KP * yaw_f)
                        cmd_yaw = clamp_int(raw_yaw, -MAX_YAW_RC, MAX_YAW_RC)
                        if cmd_yaw != 0 and abs(cmd_yaw) < MIN_YAW_RC:
                            cmd_yaw = int(math.copysign(MIN_YAW_RC, cmd_yaw))
                        target_yaw_cmd = cmd_yaw
                        target_fb_cmd = 0
                        target_lr_cmd = 0
                        control_mode = "YAW_ONLY"


                    elif not centered_x:
                        stable_cnt = 0
                        target_yaw_cmd = 0
                        target_lr_cmd = 0

                        x_err = (marker_cx - cx)
                        x_norm = x_err / cx
                        raw_fb = -FB_KP * x_norm
                        cmd_fb = clamp_int(raw_fb, -MAX_FB_RC, MAX_FB_RC)
                        if cmd_fb != 0 and abs(cmd_fb) < MIN_FB_RC:
                            cmd_fb = int(math.copysign(MIN_FB_RC, cmd_fb))
                        target_fb_cmd = cmd_fb
                        target_lr_cmd = 0
                        control_mode = "FB_ONLY(X)"


                    elif not centered_y:
                        stable_cnt = 0
                        target_yaw_cmd = 0
                        target_fb_cmd = 0

                        y_err = (marker_cy - cy)
                        y_norm = y_err / cy
                        raw_lr = LR_SIGN_Y * (-LR_KP * y_norm)
                        cmd_lr = clamp_int(raw_lr, -MAX_LR_RC, MAX_LR_RC)
                        if cmd_lr != 0 and abs(cmd_lr) < MIN_LR_RC:
                            cmd_lr = int(math.copysign(MIN_LR_RC, cmd_lr))
                        target_lr_cmd = cmd_lr
                        control_mode = "LR_ONLY(Y)"

                    else:
                        target_yaw_cmd = 0
                        target_fb_cmd = 0
                        target_lr_cmd = 0

                        stable_cnt += 1
                        control_mode = "HOLD(STABLE)"

                    cv2.putText(
                        frame,
                        f"ID{mid} ALIGN [{control_mode}] diag={diag_angle:+.1f} yaw_err={yaw_f:+.1f} "
                        f"yaw={target_yaw_cmd:+d} fb={target_fb_cmd:+d} lr={target_lr_cmd:+d} "
                        f"cx={marker_cx:.0f} cy={marker_cy:.0f} Xok={int(centered_x)} Yok={int(centered_y)} "
                        f"stable={stable_cnt}/{STABLE_N}",
                        (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.60,
                        (0, 255, 255),
                        2,
                    )


                    if stable_cnt >= STABLE_N:
                        try:
                            with tello_lock:
                                tello.send_rc_control(0, 0, 0, 0)
                        except Exception:
                            pass

                        last_seen_marker_id = mid

                        if mode == MODE_ALIGN_ID1:

                            switch_to_front_camera(tello)

                            arm_front_record(mid)
                            print("[STATE] ID1 aligned -> remember ID1 -> switch FRONT CAM -> HOVER 10s (record) -> RE-ALIGN same ID -> RIGHT 135+135 -> SEARCH ID2")
                            mode = MODE_HOVER_AFTER_ID1
                            hover_start_t = time.time()
                        elif mode == MODE_ALIGN_ID2:

                            switch_to_front_camera(tello)

                            arm_front_record(mid)
                            print("[STATE] ID2 aligned -> remember ID2 -> switch FRONT CAM -> HOVER 10s (record) -> RE-ALIGN same ID -> RIGHT 90+90 -> SEARCH ID3")
                            mode = MODE_HOVER_AFTER_ID2
                            hover_start_t = time.time()
                        elif mode == MODE_ALIGN_ID3:

                            switch_to_front_camera(tello)

                            arm_front_record(mid)
                            print("[STATE] ID3 aligned -> remember ID3 -> switch FRONT CAM -> HOVER 10s (record) -> RE-ALIGN same ID -> RIGHT 90+90 -> SEARCH ID4")
                            mode = MODE_HOVER_AFTER_ID3
                            hover_start_t = time.time()
                        elif mode == MODE_ALIGN_ID4:

                            switch_to_front_camera(tello)

                            arm_front_record(mid)
                            print("[STATE] ID4 aligned -> remember ID4 -> switch FRONT CAM -> HOVER 10s (record) -> RE-ALIGN same ID -> LEFT 90+90 -> SEARCH RETURN ID3")
                            mode = MODE_HOVER_AFTER_ID4
                            hover_start_t = time.time()
                        elif mode == MODE_ALIGN_RETURN_ID3:

                            switch_to_front_camera(tello)

                            arm_front_record(mid)
                            print("[STATE] RETURN ID3 aligned -> remember ID3 -> switch FRONT CAM -> HOVER 10s (record) -> RE-ALIGN same ID -> LEFT 90+90 -> SEARCH RETURN ID2")
                            mode = MODE_HOVER_AFTER_RETURN_ID3
                            hover_start_t = time.time()
                        elif mode == MODE_ALIGN_RETURN_ID2:

                            switch_to_front_camera(tello)

                            arm_front_record(mid)
                            print("[STATE] RETURN ID2 aligned -> remember ID2 -> switch FRONT CAM -> HOVER 10s (record) -> RE-ALIGN same ID -> LEFT 135+135 -> SEARCH RETURN ID1")
                            mode = MODE_HOVER_AFTER_RETURN_ID2
                            hover_start_t = time.time()
                        elif mode == MODE_ALIGN_RETURN_ID1:

                            switch_to_front_camera(tello)

                            arm_front_record(mid)
                            print("[STATE] RETURN ID1 aligned -> remember ID1 -> switch FRONT CAM -> HOVER 10s (record) -> RE-ALIGN same ID -> LAND")
                            mode = MODE_HOVER_AFTER_RETURN_ID1
                            hover_start_t = time.time()
                        else:
                            print(f"[STATE] ID{mid} re-aligned after recording -> {post_record_next_mode}")
                            mode = post_record_next_mode
                            post_record_realign_id = None
                            post_record_next_mode = None
                            active_align_id = None
                            move_end_t = None
                else:

                    if (time.time() - last_seen_t) > LOST_TIMEOUT_S:
                        target_yaw_cmd = target_fb_cmd = target_lr_cmd = 0
                        stable_cnt = 0


        if mode == MODE_HOVER_PRE_UP and is_flying:
            if hover_start_t is None:
                hover_start_t = time.time()
            if (time.time() - hover_start_t) >= HOVER_BEFORE_UP_S:
                mode = MODE_SWITCH_DOWNCAM
                hover_start_t = None
                move_end_t = None


        if mode == MODE_SWITCH_DOWNCAM and is_flying:

            switch_to_down_camera(tello)
            mode = MODE_MOVE_UP


        if mode == MODE_MOVE_UP and is_flying:
            if move_end_t is None:
                try:

                    move_end_t = start_sdk_move(tello, tello.move_up, MOVE_UP_CM)
                except Exception as e:
                    print("[WARN] move_up failed:", e)
                    mode = MODE_LAND
            else:
                if time.time() >= move_end_t:
                    move_end_t = None
                    mode = MODE_SEARCH_ID1
                    search_target_id = FLOOR_MARKER_ID1_ALIGN
                    search_timeout_s = SEARCH_TIMEOUT_ID1_S
                    search_start_t = time.time()
                    search_seen_cnt = 0


        if mode == MODE_HOVER_AFTER_ID1 and is_flying:
            if hover_start_t is None:
                hover_start_t = time.time()
            if (time.time() - hover_start_t) >= HOVER_AFTER_ID1_ALIGN_S:

                begin_post_record_realign(FLOOR_MARKER_ID1_ALIGN, MODE_MOVE_RIGHT_270, SEARCH_TIMEOUT_ID1_S)


        if mode == MODE_MOVE_RIGHT_270 and is_flying:
            if move_end_t is None:
                try:
                    move_end_t = start_sdk_move_split(
                        tello,
                        tello.move_right,
                        [MOVE_RIGHT_AFTER_ID1_PART1_CM, MOVE_RIGHT_AFTER_ID1_PART2_CM],
                        "move_right",
                    )
                except Exception as e:
                    print("[WARN] move_right(135 + 135) failed:", e)
                    mode = MODE_LAND
            else:
                if time.time() >= move_end_t:
                    move_end_t = None
                    mode = MODE_SEARCH_ID2
                    search_target_id = FLOOR_MARKER_ID2_ALIGN
                    search_timeout_s = SEARCH_TIMEOUT_ID2_S
                    search_start_t = time.time()
                    search_seen_cnt = 0


        if mode == MODE_HOVER_AFTER_ID2 and is_flying:
            if hover_start_t is None:
                hover_start_t = time.time()
            if (time.time() - hover_start_t) >= HOVER_AFTER_ID2_ALIGN_S:

                begin_post_record_realign(FLOOR_MARKER_ID2_ALIGN, MODE_MOVE_RIGHT_AFTER_ID2, SEARCH_TIMEOUT_ID2_S)


        if mode == MODE_MOVE_RIGHT_AFTER_ID2 and is_flying:
            if move_end_t is None:
                try:
                    move_end_t = start_sdk_move_split(
                        tello,
                        tello.move_right,
                        [MOVE_RIGHT_AFTER_ID2_PART1_CM, MOVE_RIGHT_AFTER_ID2_PART2_CM],
                        "move_right",
                    )
                except Exception as e:
                    print("[WARN] move_right after ID2 failed:", e)
                    mode = MODE_LAND
            else:
                if time.time() >= move_end_t:
                    move_end_t = None
                    mode = MODE_SEARCH_ID3
                    search_target_id = FLOOR_MARKER_ID3_ALIGN
                    search_timeout_s = SEARCH_TIMEOUT_ID3_S
                    search_start_t = time.time()
                    search_seen_cnt = 0


        if mode == MODE_HOVER_AFTER_ID3 and is_flying:
            if hover_start_t is None:
                hover_start_t = time.time()
            if (time.time() - hover_start_t) >= HOVER_AFTER_ID3_ALIGN_S:

                begin_post_record_realign(FLOOR_MARKER_ID3_ALIGN, MODE_MOVE_RIGHT_AFTER_ID3, SEARCH_TIMEOUT_ID3_S)


        if mode == MODE_MOVE_RIGHT_AFTER_ID3 and is_flying:
            if move_end_t is None:
                try:
                    move_end_t = start_sdk_move_split(
                        tello,
                        tello.move_right,
                        [MOVE_RIGHT_AFTER_ID3_PART1_CM, MOVE_RIGHT_AFTER_ID3_PART2_CM],
                        "move_right",
                    )
                except Exception as e:
                    print("[WARN] move_right after ID3 failed:", e)
                    mode = MODE_LAND
            else:
                if time.time() >= move_end_t:
                    move_end_t = None
                    mode = MODE_SEARCH_ID4
                    search_target_id = FLOOR_MARKER_ID4_ALIGN
                    search_timeout_s = SEARCH_TIMEOUT_ID4_S
                    search_start_t = time.time()
                    search_seen_cnt = 0


        if mode == MODE_HOVER_AFTER_ID4 and is_flying:
            if hover_start_t is None:
                hover_start_t = time.time()
            if (time.time() - hover_start_t) >= HOVER_AFTER_ID4_ALIGN_S:

                begin_post_record_realign(FLOOR_MARKER_ID4_ALIGN, MODE_MOVE_LEFT_AFTER_ID4, SEARCH_TIMEOUT_ID4_S)


        if mode == MODE_MOVE_LEFT_AFTER_ID4 and is_flying:
            if move_end_t is None:
                try:
                    move_end_t = start_sdk_move_split(
                        tello,
                        tello.move_left,
                        [MOVE_LEFT_RETURN_TO_ID3_PART1_CM, MOVE_LEFT_RETURN_TO_ID3_PART2_CM],
                        "move_left",
                    )
                except Exception as e:
                    print("[WARN] move_left after ID4 failed:", e)
                    mode = MODE_LAND
            else:
                if time.time() >= move_end_t:
                    move_end_t = None
                    mode = MODE_SEARCH_RETURN_ID3
                    search_target_id = FLOOR_MARKER_ID3_ALIGN
                    search_timeout_s = SEARCH_TIMEOUT_ID3_S
                    search_start_t = time.time()
                    search_seen_cnt = 0


        if mode == MODE_HOVER_AFTER_RETURN_ID3 and is_flying:
            if hover_start_t is None:
                hover_start_t = time.time()
            if (time.time() - hover_start_t) >= HOVER_AFTER_RETURN_ID3_ALIGN_S:

                begin_post_record_realign(FLOOR_MARKER_ID3_ALIGN, MODE_MOVE_LEFT_AFTER_RETURN_ID3, SEARCH_TIMEOUT_ID3_S)


        if mode == MODE_MOVE_LEFT_AFTER_RETURN_ID3 and is_flying:
            if move_end_t is None:
                try:
                    move_end_t = start_sdk_move_split(
                        tello,
                        tello.move_left,
                        [MOVE_LEFT_RETURN_TO_ID2_PART1_CM, MOVE_LEFT_RETURN_TO_ID2_PART2_CM],
                        "move_left",
                    )
                except Exception as e:
                    print("[WARN] move_left after RETURN ID3 failed:", e)
                    mode = MODE_LAND
            else:
                if time.time() >= move_end_t:
                    move_end_t = None
                    mode = MODE_SEARCH_RETURN_ID2
                    search_target_id = FLOOR_MARKER_ID2_ALIGN
                    search_timeout_s = SEARCH_TIMEOUT_ID2_S
                    search_start_t = time.time()
                    search_seen_cnt = 0


        if mode == MODE_HOVER_AFTER_RETURN_ID2 and is_flying:
            if hover_start_t is None:
                hover_start_t = time.time()
            if (time.time() - hover_start_t) >= HOVER_AFTER_RETURN_ID2_ALIGN_S:

                begin_post_record_realign(FLOOR_MARKER_ID2_ALIGN, MODE_MOVE_LEFT_AFTER_RETURN_ID2, SEARCH_TIMEOUT_ID2_S)


        if mode == MODE_MOVE_LEFT_AFTER_RETURN_ID2 and is_flying:
            if move_end_t is None:
                try:
                    move_end_t = start_sdk_move_split(
                        tello,
                        tello.move_left,
                        [MOVE_LEFT_RETURN_TO_ID1_PART1_CM, MOVE_LEFT_RETURN_TO_ID1_PART2_CM],
                        "move_left",
                    )
                except Exception as e:
                    print("[WARN] move_left after RETURN ID2 failed:", e)
                    mode = MODE_LAND
            else:
                if time.time() >= move_end_t:
                    move_end_t = None
                    mode = MODE_SEARCH_RETURN_ID1
                    search_target_id = FLOOR_MARKER_ID1_ALIGN
                    search_timeout_s = SEARCH_TIMEOUT_ID1_S
                    search_start_t = time.time()
                    search_seen_cnt = 0


        if mode == MODE_HOVER_AFTER_RETURN_ID1 and is_flying:
            if hover_start_t is None:
                hover_start_t = time.time()
            if (time.time() - hover_start_t) >= HOVER_AFTER_RETURN_ID1_ALIGN_S:

                begin_post_record_realign(FLOOR_MARKER_ID1_ALIGN, MODE_LAND, SEARCH_TIMEOUT_ID1_S)


        if mode == MODE_LAND and is_flying:
            try:
                with tello_lock:
                    tello.send_rc_control(0, 0, 0, 0)
            except Exception:
                pass
            try:
                with tello_lock:
                    tello.land()
            except Exception:
                pass

            disarm_front_record()
            post_record_realign_id = None
            post_record_next_mode = None
            active_align_id = None
            is_flying = False

            mode = MODE_IDLE
            safe_set_downvision(tello, False)
            is_downvision_active = False


        lr_cmd = fb_cmd = yaw_cmd = 0
        if is_flying:
            now = time.time()

            move_in_progress = (
                (mode == MODE_MOVE_UP and move_end_t is not None and now < move_end_t)
                or (mode == MODE_MOVE_RIGHT_270 and move_end_t is not None and now < move_end_t)
                or (mode == MODE_MOVE_RIGHT_AFTER_ID2 and move_end_t is not None and now < move_end_t)
                or (mode == MODE_MOVE_RIGHT_AFTER_ID3 and move_end_t is not None and now < move_end_t)
                or (mode == MODE_MOVE_LEFT_AFTER_ID4 and move_end_t is not None and now < move_end_t)
                or (mode == MODE_MOVE_LEFT_AFTER_RETURN_ID3 and move_end_t is not None and now < move_end_t)
                or (mode == MODE_MOVE_LEFT_AFTER_RETURN_ID2 and move_end_t is not None and now < move_end_t)
            )


            if not move_in_progress and (now - last_rc_sent) >= RC_PERIOD:

                tgt_lr = tgt_fb = tgt_yaw = 0
                if mode in [
                    MODE_ALIGN_ID1, MODE_ALIGN_ID2, MODE_ALIGN_ID3, MODE_ALIGN_ID4,
                    MODE_ALIGN_RETURN_ID3, MODE_ALIGN_RETURN_ID2, MODE_ALIGN_RETURN_ID1,
                    MODE_ALIGN_POST_RECORD,
                ]:
                    tgt_lr = clamp_int(target_lr_cmd, -MAX_LR_RC, MAX_LR_RC)
                    tgt_fb = clamp_int(target_fb_cmd, -MAX_FB_RC, MAX_FB_RC)
                    tgt_yaw = clamp_int(target_yaw_cmd, -MAX_YAW_RC, MAX_YAW_RC)


                lr_cmd = int(slew(prev_lr, tgt_lr, LR_SLEW_STEP))

                fb_cmd = int(slew(prev_fb, tgt_fb, FB_SLEW_STEP))

                yaw_cmd = int(slew(prev_yaw, tgt_yaw, YAW_SLEW_STEP))
                prev_lr, prev_fb, prev_yaw = lr_cmd, fb_cmd, yaw_cmd

                with tello_lock:

                    tello.send_rc_control(lr_cmd, fb_cmd, 0, yaw_cmd)
                last_rc_sent = now


        record_hover_mode = mode in [
            MODE_HOVER_AFTER_ID1, MODE_HOVER_AFTER_ID2, MODE_HOVER_AFTER_ID3, MODE_HOVER_AFTER_ID4,
            MODE_HOVER_AFTER_RETURN_ID3, MODE_HOVER_AFTER_RETURN_ID2, MODE_HOVER_AFTER_RETURN_ID1,
        ]

        front_frame_ready = (w >= FRONT_MIN_W and h >= FRONT_MIN_H)


        should_record_front = (
            is_flying
            and record_hover_mode
            and (not is_downvision_active)
            and front_record_armed
            and (front_record_target_id is not None)
            and ((time.time() - front_record_arm_t) >= FRONT_RECORD_DELAY_S)
            and front_frame_ready
        )


        update_video_recording(frame, should_record_front, front_record_target_id)


        if should_record_front:

            put_latest(send_q, frame)


        if not is_flying:
            step_text = "STEP: LANDED (press 't' to takeoff)"
        else:

            if mode == MODE_HOVER_PRE_UP:
                step_text = "STEP: HOVER 3s (FRONT CAM before UP 130, NO RECORD)"
            elif mode == MODE_SWITCH_DOWNCAM:
                step_text = "STEP: SWITCH CAMERA -> DOWNVISION"
            elif mode == MODE_MOVE_UP:
                step_text = "STEP: MOVE UP 130cm"
            elif mode == MODE_SEARCH_ID1:
                step_text = "STEP: SEARCH ID1 (DOWNVISION)"
            elif mode == MODE_ALIGN_ID1:
                step_text = f"STEP: ALIGN ID1 | stable={stable_cnt}/{STABLE_N}"
            elif mode == MODE_HOVER_AFTER_ID1:
                step_text = "STEP: HOVER 10s (FRONT CAM, RECORD ID1) -> RE-ALIGN ID1 -> RIGHT 135+135"
            elif mode == MODE_MOVE_RIGHT_270:
                step_text = "STEP: MOVE RIGHT 135+135 (after ID1)"
            elif mode == MODE_SEARCH_ID2:
                step_text = "STEP: SEARCH ID2 (DOWNVISION)"
            elif mode == MODE_ALIGN_ID2:
                step_text = f"STEP: ALIGN ID2 | stable={stable_cnt}/{STABLE_N}"
            elif mode == MODE_HOVER_AFTER_ID2:
                step_text = "STEP: HOVER 10s (FRONT CAM, RECORD ID2) -> RE-ALIGN ID2 -> RIGHT 90+90"
            elif mode == MODE_MOVE_RIGHT_AFTER_ID2:
                step_text = "STEP: MOVE RIGHT 90+90 (after ID2)"
            elif mode == MODE_SEARCH_ID3:
                step_text = "STEP: SEARCH ID3 (DOWNVISION)"
            elif mode == MODE_ALIGN_ID3:
                step_text = f"STEP: ALIGN ID3 | stable={stable_cnt}/{STABLE_N}"
            elif mode == MODE_HOVER_AFTER_ID3:
                step_text = "STEP: HOVER 10s (FRONT CAM, RECORD ID3) -> RE-ALIGN ID3 -> RIGHT 90+90"
            elif mode == MODE_MOVE_RIGHT_AFTER_ID3:
                step_text = "STEP: MOVE RIGHT 90+90 (after ID3)"
            elif mode == MODE_SEARCH_ID4:
                step_text = "STEP: SEARCH ID4 (DOWNVISION)"
            elif mode == MODE_ALIGN_ID4:
                step_text = f"STEP: ALIGN ID4 | stable={stable_cnt}/{STABLE_N}"
            elif mode == MODE_HOVER_AFTER_ID4:
                step_text = "STEP: HOVER 10s (FRONT CAM, RECORD ID4) -> RE-ALIGN ID4 -> LEFT 90+90"
            elif mode == MODE_MOVE_LEFT_AFTER_ID4:
                step_text = "STEP: MOVE LEFT 90+90 (after ID4) -> SEARCH RETURN ID3"
            elif mode == MODE_SEARCH_RETURN_ID3:
                step_text = "STEP: SEARCH RETURN ID3 (DOWNVISION)"
            elif mode == MODE_ALIGN_RETURN_ID3:
                step_text = f"STEP: ALIGN RETURN ID3 | stable={stable_cnt}/{STABLE_N}"
            elif mode == MODE_HOVER_AFTER_RETURN_ID3:
                step_text = "STEP: HOVER 10s (FRONT CAM, RECORD RETURN ID3) -> RE-ALIGN ID3 -> LEFT 90+90"
            elif mode == MODE_MOVE_LEFT_AFTER_RETURN_ID3:
                step_text = "STEP: MOVE LEFT 90+90 (after RETURN ID3) -> SEARCH RETURN ID2"
            elif mode == MODE_SEARCH_RETURN_ID2:
                step_text = "STEP: SEARCH RETURN ID2 (DOWNVISION)"
            elif mode == MODE_ALIGN_RETURN_ID2:
                step_text = f"STEP: ALIGN RETURN ID2 | stable={stable_cnt}/{STABLE_N}"
            elif mode == MODE_HOVER_AFTER_RETURN_ID2:
                step_text = "STEP: HOVER 10s (FRONT CAM, RECORD RETURN ID2) -> RE-ALIGN ID2 -> LEFT 135+135"
            elif mode == MODE_MOVE_LEFT_AFTER_RETURN_ID2:
                step_text = "STEP: MOVE LEFT 135+135 (after RETURN ID2) -> SEARCH RETURN ID1"
            elif mode == MODE_SEARCH_RETURN_ID1:
                step_text = "STEP: SEARCH RETURN ID1 (DOWNVISION)"
            elif mode == MODE_ALIGN_RETURN_ID1:
                step_text = f"STEP: ALIGN RETURN ID1 | stable={stable_cnt}/{STABLE_N}"
            elif mode == MODE_HOVER_AFTER_RETURN_ID1:
                step_text = "STEP: HOVER 10s (FRONT CAM, RECORD RETURN ID1) -> RE-ALIGN ID1 -> LAND"
            elif mode == MODE_SEARCH_POST_RECORD:
                step_text = f"STEP: RE-SEARCH ID{search_target_id} AFTER RECORD (DOWNVISION)"
            elif mode == MODE_ALIGN_POST_RECORD:
                step_text = f"STEP: RE-ALIGN ID{active_align_id} AFTER RECORD | stable={stable_cnt}/{STABLE_N}"
            elif mode == MODE_LAND:
                step_text = "STEP: LANDING"
            else:
                step_text = f"STEP: {mode}"

        cv2.putText(frame, step_text, (10, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 3)

        cv2.line(frame, (left_band, 0), (left_band, h), (255, 255, 255), 1)
        cv2.line(frame, (right_band, 0), (right_band, h), (255, 255, 255), 1)
        cv2.line(frame, (int(cx), 0), (int(cx), h), (255, 255, 255), 1)

        cv2.line(frame, (0, top_band), (w, top_band), (255, 255, 255), 1)
        cv2.line(frame, (0, bottom_band), (w, bottom_band), (255, 255, 255), 1)
        cv2.line(frame, (0, int(cy)), (w, int(cy)), (255, 255, 255), 1)

        if marker_cx is not None and marker_cy is not None and mode in [
            MODE_ALIGN_ID1, MODE_ALIGN_ID2, MODE_ALIGN_ID3, MODE_ALIGN_ID4,
            MODE_ALIGN_RETURN_ID3, MODE_ALIGN_RETURN_ID2, MODE_ALIGN_RETURN_ID1,
            MODE_ALIGN_POST_RECORD,
        ]:
            cv2.circle(frame, (int(marker_cx), int(marker_cy)), 6, (0, 255, 0), -1)

        cam_status = "DOWN" if is_downvision_active else "FRONT"
        rec_status = "ON" if video_writer is not None else "OFF"
        arm_status = "ARMED" if front_record_armed else "DISARMED"
        last_id_text = f"id{last_seen_marker_id}" if last_seen_marker_id is not None else "None"

        status = "FLYING" if is_flying else "LANDED"
        cv2.putText(frame, f"Status:{status} | Cam:{cam_status} | Mode:{mode}",
                    (10, h - 95), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        cv2.putText(frame, f"Cmd: lr={lr_cmd:+d} fb={fb_cmd:+d} yaw={yaw_cmd:+d}",
                    (10, h - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        cv2.putText(frame, f"FrontRec:{rec_status} | Arm:{arm_status} | LastSeen:{last_id_text}",
                    (10, h - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)


        cv2.imshow("Tello ArUco - ID1 -> ID2 -> ID3 -> ID4 -> RETURN ID3 -> RETURN ID2 -> RETURN ID1", frame)


        key = cv2.waitKey(1) & 0xFF

        if key == 27:
            print("[USER] ESC -> quit")
            break


        if key == ord('t'):
            if not is_flying:
                print("[USER] Takeoff requested")
                with tello_lock:
                    tello.takeoff()
                time.sleep(2)
                is_flying = True
                last_rc_sent = 0.0
                prev_lr = prev_fb = prev_yaw = 0

                last_seen_marker_id = None
                disarm_front_record()
                last_video_write_t = 0.0


                switch_to_front_camera(tello)

                mode = MODE_HOVER_PRE_UP
                hover_start_t = time.time()
                move_end_t = None

                search_start_t = None
                search_seen_cnt = 0
                search_target_id = FLOOR_MARKER_ID1_ALIGN
                search_timeout_s = SEARCH_TIMEOUT_ID1_S

                active_align_id = None
                post_record_realign_id = None
                post_record_next_mode = None
                yaw_f = None
                stable_cnt = 0
                target_yaw_cmd = target_fb_cmd = target_lr_cmd = 0
            else:
                print("[INFO] Already flying")


        if key == ord('q'):
            if is_flying:
                print("[USER] Land requested")
                try:
                    with tello_lock:
                        tello.send_rc_control(0, 0, 0, 0)
                except Exception:
                    pass
                try:
                    with tello_lock:
                        tello.land()
                except Exception:
                    pass

                disarm_front_record()
                post_record_realign_id = None
                post_record_next_mode = None
                active_align_id = None
                is_flying = False

                mode = MODE_IDLE
                safe_set_downvision(tello, False)
                is_downvision_active = False
            else:
                print("[INFO] Already landed")


finally:


    try:
        disarm_front_record()
    except Exception as e:
        print("[WARN] disarm_front_record failed:", e)


    stop_evt.set()

    try:
        if th_sender is not None:
            th_sender.join(timeout=2.0)
        if th_db is not None:
            th_db.join(timeout=2.0)
        if th_uploader is not None:
            th_uploader.join(timeout=180.0)
    except Exception as e:
        print("[WARN] thread join failed:", e)

    try:

        cv2.destroyAllWindows()
    except Exception:
        pass

    try:
        with tello_lock:
            tello.send_rc_control(0, 0, 0, 0)
    except Exception:
        pass

    if AUTO_LAND_ON_EXIT and is_flying:
        try:
            with tello_lock:
                tello.land()
        except Exception:
            pass

    try:
        safe_set_downvision(tello, False)
    except Exception:
        pass

    try:
        with tello_lock:

            tello.streamoff()
    except Exception:
        pass

    try:
        with tello_lock:

            tello.end()
    except Exception:
        pass
