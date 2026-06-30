# 전체 동작 요약
# 1. DJI Tello 드론에 연결하고 영상 스트림을 켭니다.
# 2. 사용자가 키보드에서 't'를 누르면 이륙합니다.
# 3. 전면 카메라 상태에서 잠깐 호버링한 뒤, 바닥 카메라(downvision)로 전환합니다.
# 4. 바닥의 ArUco 마커 ID1 → ID2 → ID3 → ID4를 순서대로 찾고 정렬합니다.
# 5. 각 마커에 정렬되면 전면 카메라로 전환해 일정 시간 녹화합니다.
# 6. 녹화 후 다시 바닥 카메라로 전환하여 방금 봤던 마커에 재정렬합니다.
# 7. 지정 거리만큼 좌/우 이동하여 다음 마커를 찾습니다.
# 8. ID4까지 갔다가 다시 ID3 → ID2 → ID1 방향으로 복귀합니다.
# 9. 마지막 복귀 ID1 녹화/재정렬 후 착륙합니다.
# 10. 별도 스레드가 실시간 프레임 전송, 녹화 영상 업로드, DB 상태 기록을 담당합니다.

# 주요 구조
# - 메인 스레드: 영상 수신, ArUco 검출, 상태머신, 드론 RC/SDK 명령, UI 표시
# - sender_worker: 전면 카메라 녹화 구간의 최신 프레임을 서버로 전송
# - upload_worker: 저장된 mp4 영상을 서버에 multipart/form-data로 업로드
# - db_worker: 일정 주기로 드론 배터리/비행 상태를 DB에 insert


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


# ===== 설정 / 파라미터 =====
# =========================

# 첫 번째로 찾아 정렬할 바닥 ArUco 마커 ID
FLOOR_MARKER_ID1_ALIGN = 11
# 두 번째 이동 지점의 바닥 ArUco 마커 ID
FLOOR_MARKER_ID2_ALIGN = 2
# 세 번째 이동 지점의 바닥 ArUco 마커 ID
FLOOR_MARKER_ID3_ALIGN = 3 
# 네 번째이자 우측 진행 구간의 마지막 바닥 ArUco 마커 ID
FLOOR_MARKER_ID4_ALIGN = 4

# RC 명령을 보내는 최소 간격이다. 만약, 0.05초이면 초당 최대 약 20회 전송합니다.
RC_PERIOD = 0.05
# yaw 오차 EMA 필터의 새 측정값 반영 비율이다. 클수록 빠르게 반응하고 작을수록 부드럽디.
EMA_ALPHA = 0.35

# 화면 너비 대비 중앙 허용 영역의 반폭 비율이다. 마커 중심 x가 이 범위 안이면 X 정렬 완료로 본다.
CENTER_BAND_RATIO_X = 0.075
# 화면 높이 대비 중앙 허용 영역의 반폭 비율이다.마커 중심 y가 이 범위 안이면 Y 정렬 완료로 본다.
CENTER_BAND_RATIO_Y = 0.075

# 이륙 후 바닥 카메라 검색을 시작하기 전에 상승할 높이(cm)이다다
MOVE_UP_CM = 150

# ID1 이후 오른쪽 이동 첫 번째 구간 거리(cm)다. 
MOVE_RIGHT_AFTER_ID1_PART1_CM = 200
# ID1 이후 오른쪽 이동 두 번째 구간 거리(cm)다.
MOVE_RIGHT_AFTER_ID1_PART2_CM = 330
# ID2 이후 오른쪽 이동 첫 번째 구간 거리(cm)다.
MOVE_RIGHT_AFTER_ID2_PART1_CM = 200
# ID2 이후 오른쪽 이동 두 번째 구간 거리(cm)다.
MOVE_RIGHT_AFTER_ID2_PART2_CM = 330
# ID3 이후 오른쪽 이동 첫 번째 구간 거리(cm)다.
MOVE_RIGHT_AFTER_ID3_PART1_CM = 200
# ID3 이후 오른쪽 이동 두 번째 구간 거리(cm)다.
MOVE_RIGHT_AFTER_ID3_PART2_CM = 330

# ===== 복귀(좌측) 이동 =====
# ID4까지 오른쪽으로 이동한 뒤, 다시 ID3 → ID2 → ID1 쪽으로 돌아올 때 사용하는 좌측 이동 거리다.
# ID4에서 복귀 ID3 방향으로 왼쪽 이동할 첫 번째 구간 거리(cm)다.
MOVE_LEFT_RETURN_TO_ID3_PART1_CM = 200
# ID4에서 복귀 ID3 방향으로 왼쪽 이동할 두 번째 구간 거리(cm)다.
MOVE_LEFT_RETURN_TO_ID3_PART2_CM = 330
# 복귀 ID3에서 복귀 ID2 방향으로 왼쪽 이동할 첫 번째 구간 거리(cm)다.
MOVE_LEFT_RETURN_TO_ID2_PART1_CM = 200
# 복귀 ID3에서 복귀 ID2 방향으로 왼쪽 이동할 두 번째 구간 거리(cm)다.
MOVE_LEFT_RETURN_TO_ID2_PART2_CM = 330
# 복귀 ID2에서 복귀 ID1 방향으로 왼쪽 이동할 첫 번째 구간 거리(cm)다.
MOVE_LEFT_RETURN_TO_ID1_PART1_CM = 200
# 복귀 ID2에서 복귀 ID1 방향으로 왼쪽 이동할 두 번째 구간 거리(cm)다.
MOVE_LEFT_RETURN_TO_ID1_PART2_CM = 330

# ===== 호버 시간 =====
# 각 마커에 정렬된 후 전면 카메라로 영상을 찍기 위해 제자리 비행하는 시간이다.
# 이륙 후 상승 전 전면 카메라 상태로 제자리 비행하는 시간(초)다.
HOVER_BEFORE_UP_S = 3.0

# ID1 정렬 후 전면 카메라로 촬영하며 머무는 시간(초)이다.
HOVER_AFTER_ID1_ALIGN_S = 35.0
# ID2 정렬 후 전면 카메라로 촬영하며 머무는 시간(초)이다.
HOVER_AFTER_ID2_ALIGN_S = 35.0
# ID3 정렬 후 전면 카메라로 촬영하며 머무는 시간(초)이다.
HOVER_AFTER_ID3_ALIGN_S = 35.0
# ID4 정렬 후 전면 카메라로 촬영하며 머무는 시간(초)이다.
HOVER_AFTER_ID4_ALIGN_S = 35.0
# 복귀 ID3 정렬 후 전면 카메라로 촬영하며 머무는 시간(초)이다.
HOVER_AFTER_RETURN_ID3_ALIGN_S = 35.0
# 복귀 ID2 정렬 후 전면 카메라로 촬영하며 머무는 시간(초)이다.
HOVER_AFTER_RETURN_ID2_ALIGN_S = 35.0
# 복귀 ID1 정렬 후 전면 카메라로 촬영하며 머무는 시간(초)이다.
HOVER_AFTER_RETURN_ID1_ALIGN_S = 35.0

# ID1 검색 제한 시간(초)다. 초과하면 LAND 상태로 넘어간다.
SEARCH_TIMEOUT_ID1_S = 15.0
# ID2 검색 제한 시간(초)다. 초과하면 LAND 상태로 넘어간다.
SEARCH_TIMEOUT_ID2_S = 15.0
# ID3 검색 제한 시간(초)다. 초과하면 LAND 상태로 넘어간다.
SEARCH_TIMEOUT_ID3_S = 15.0
# ID4 검색 제한 시간(초)다. 초과하면 LAND 상태로 넘어간다.
SEARCH_TIMEOUT_ID4_S = 15.0
# SEARCH 성공으로 인정하기 위해 목표 마커가 연속으로 보여야 하는 프레임 수입니다.
SEARCH_STABLE_N = 1

# ===== 바닥 yaw 정렬(대각 135도 기준) =====
# 1. aruco marker의 4개의 꼭짓점을 검출함.
# 2. 그중에서 "좌상단 꼭짓점"과 "우하단 꼭짓점"을 사용해서 대각선 벡터를 만듬
# 3. 계산된 벡터가 얼마나 FLOOR_TARGET_EDGE_DEG와 차이나는지 계산한다.
FLOOR_TARGET_EDGE_DEG = 90.0
# 마커 대각선 기준 목표 각도이다. 현재 설정은 90도 + 45도 = 135도이다.
FLOOR_TARGET_DIAG_DEG = FLOOR_TARGET_EDGE_DEG + 45.0  # 135deg

# ====== yaw 제어 속도 파라미터 =======
# 
# yaw오차를 RC회전 명령으로 바꾸는 비율.
YAW_KP = 2.0 
# yaw RC 명령의 최대 절댓값이다. 이는 회전이 너무 세지 않게 제한해준다.
MAX_YAW_RC = 30
# yaw 보정이 필요할 때 최소로 보낼 RC 명령 절댓값이다. 너무 작은 명령이 무시되는 것을 방지함.
MIN_YAW_RC = 10
# yaw 오차가 이 각도 이하이면 yaw 정렬이 끝났다고 보고, 위치 보정 단계로 넘어간다.
YAW_EXIT_DEG = 3.0
# yaw/x/y 정렬이 모두 맞은 상태가 연속으로 유지되어야 하는 프레임 수이다.
STABLE_N = 8

# ALIGN 단계 제한 시간(초)이다. 초과하면 착륙.
ALIGN_TIMEOUT_S = 30.0
# ALIGN 중 마커를 화면에서 잃어버린 뒤 명령을 0으로 만드는 대기 시간(초)이다. 이는 마커가 사라지면 어떤 것을 기준으로 움직일지 모르니 넣어야함
LOST_TIMEOUT_S = 0.30

# yaw RC 명령을 한 주기마다 최대 얼마씩 바꿀지 정하는 변화율 제한값이다.
YAW_SLEW_STEP = 6
# yaw 제어 방향 부호이다. 실제 드론 회전 방향이 반대이면 -1로 바꿔 튜닝한면 된다.
YAW_SIGN = 1

# FB로 X 맞추기 (forward -> marker 오른쪽)
# 화면 x축 오차를 전후 이동 명령으로 변환하는 비례 제어 게인이다. ( 비례 제어 게인: 오차가 생겼을때, 그 오차에 얼마나 세게 반응할지 정하는값)
FB_KP = 35.0
# 전후 RC 명령의 최대 절댓값이다.
MAX_FB_RC = 12
# 전후 보정이 필요할 때 최소로 보낼 RC 명령 절댓값이다.
MIN_FB_RC = 6
# 전후 RC 명령 변화율 제한값이다. FB_KP가 목표 전후 명령을 계산한다면, 이거는 해당 목표 명령까지 부드럽게 접근하게 만드는값.
FB_SLEW_STEP = 3

# LR로 Y 맞추기 (드론 오른쪽 -> 마커 아래)
# 화면 y축 오차를 좌우 이동 명령으로 변환하는 비례 제어 게인이다.
LR_KP = 35.0
# 좌우 RC 명령의 최대 절댓값이다.
MAX_LR_RC = 12
# 좌우 보정이 필요할 때 최소로 보낼 RC 명령 절댓값이다.
MIN_LR_RC = 6
# 좌우 RC 명령 변화율 제한값이다.
LR_SLEW_STEP = 3
# 화면 y축 오차와 실제 좌우 이동 방향의 부호를 맞추는 값이다.
LR_SIGN_Y = 1

# SDK move 명령에 사용할 이동 속도(cm/s)이다.
MOVE_SPEED_CM_S = 30
# SDK 이동 명령 직후 안정화를 위해 추가로 기다리는 여유 시간.
MOVE_MARGIN_S = 0.5

# 프로그램 종료 시 자동 착륙할지 여부다. False이면 종료 시 강제 착륙을 하지 않는다.
AUTO_LAND_ON_EXIT = False

# ===== 동영상 저장 설정 =====
# 전면 카메라로 촬영한 영상을 mp4 파일로 저장하기 위한 설정이다.
# downvision 프레임이 실수로 저장되지 않도록 최소 해상도 조건도 함께 둔다
# 전면 카메라 녹화 mp4 파일을 저장할 폴더.
VIDEO_SAVE_DIR = "frontcam_recordings"
# 저장 영상의 목표 FPS.
VIDEO_FPS = 30.0
# 저장 영상 파일 확장자.
VIDEO_EXT = ".mp4"
VIDEO_FRAME_INTERVAL = 1.0 / VIDEO_FPS

# 전면 카메라 전환 직후 바로 저장하지 않도록 대기
# 전면 카메라 전환 직후 불안정한 프레임을 피하기 위한 녹화 시작 지연 시간이다.
FRONT_RECORD_DELAY_S = 1.0

# downvision(대개 320x240) 프레임을 잘못 저장하지 않기 위한 최소 크기
# 전면 카메라 프레임으로 인정할 최소 너비.
FRONT_MIN_W = 640
# 전면 카메라 프레임으로 인정할 최소 높이.
FRONT_MIN_H = 480

# =========================
# ===== 서버 전송 설정 =====
# 실시간 프레임 JPEG 전송과 녹화 영상 업로드에 필요한 서버 설정.
# 현재 URL 값들은 빈 문자열이므로 실제 서버 주소를 넣어야 HTTP 전송이 성공된다.
# =========================
# 서버 기본 주소다.
SERVER_BASE_URL = ""
# 실시간 JPEG 프레임을 POST할 서버 엔드포인트다.
SERVER_URL = ""
# mp4 영상 파일을 업로드할 서버 엔드포인트다.
UPLOAD_VIDEO_URL = ""
# 서버 인증용 API 키다. 환경변수 SERVER_TOKEN에서 읽는다.
SERVER_TOKEN = os.getenv("SERVER_TOKEN", "")
STREAM_VIDEO_BY_MARKER_URL_TEMPLATE = ""

# 서버/DB에서 카메라를 구분하기 위한 ID다.
CAM_ID = 0
# 서버로 실시간 프레임을 보내는 최대 FPS다.
MAX_SEND_FPS = 5
# 서버 전송 전 프레임 폭을 이 값 이하로 줄인다. 네트워크 부하를 낮추기 위한 설정임.
SERVER_RESIZE_W = 640
# 서버 전송 JPEG 압축 품질. 낮을수록 용량은 작지만 화질이 떨어짐.
SERVER_JPEG_QUALITY = 65
VIDEO_UPLOAD_CONNECT_TIMEOUT = 3
VIDEO_UPLOAD_READ_TIMEOUT = 120
sess = requests.Session()
video_upload_sess = requests.Session()

# =========================
# ===== DB Insert 설정 =====
# 드론 상태를 주기적으로 SQL Server 테이블에 저장하기 위한 설정.
# DB 비밀번호는 보안을 위해 코드가 아니라 환경변수 DB_PASS에서 읽는다.
# =========================
# SQL Server 호스트 주소
DB_HOST = ""
# SQL Server 포트. 기본값 1433을 사용.
DB_PORT = 1433
# 접속할 데이터베이스 이름.
DB_NAME = ""
# DB 사용자 계정.
DB_USER = ""
# DB 비밀번호. 환경변수 DB_PASS에서 읽는다.
DB_PASS = os.getenv("DB_PASS", "")

# DB와 업로드 메타데이터에 사용할 고정 드론 ID다.
DRONE_ID_FIXED = "DR001"
# DB에 기록할 연결 상태 문자열이다.
CONNECTED_FIXED = "true"
# DB 상태 기록 주기(초)이다.
INTERVAL_SEC = 5

# =========================
# ===== 스레드 / 큐 =====
# 메인 루프가 영상 처리와 드론 제어에 집중할 수 있도록, 느린 네트워크/DB 작업은 별도 스레드로 분리한다.
# Queue는 스레드 간 데이터를 안전하게 전달하며, Event는 전체 종료 신호로 사용된다.
# =========================
# 실시간 서버 전송용 프레임 큐다. maxsize=1로 최신 프레임만 유지한다.
send_q = queue.Queue(maxsize=1)
# 저장된 영상 파일 업로드 작업 큐이다.
upload_q = queue.Queue(maxsize=10)
# 모든 백그라운드 스레드에 종료를 알리는 이벤트다.
stop_evt = threading.Event()
# Tello SDK 명령을 스레드 안전하게 보내기 위한 lock이다.
tello_lock = threading.Lock()
th_sender = None
th_db = None
th_uploader = None

# =========================
# ===== 카메라 파라미터 =====
cameraMatrix = np.array([
    [920.0,   0.0, 480.0],
    [  0.0, 920.0, 360.0],
    [  0.0,   0.0,   1.0]
], dtype=np.float32)

# 렌즈 왜곡 계수다. undistortPoints에서 사용합니다.
distCoeffs = np.array([-0.18, 0.06, 0.000, 0.000, -0.01], dtype=np.float32).reshape(-1, 1)

# ===== ArUco =====
# OpenCV ArUco 검출기를 초기화. 여기서는 4x4_50 딕셔너리의 ID 마커를 사용한다.
# detectMarkers()가 각 프레임에서 마커 ID와 네 꼭짓점 좌표를 반환한다.
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, params)

# =========================
# ===== 유틸 =====
# 여러 곳에서 반복해서 쓰는 작은 보조 함수들이다.
# 값 제한, EMA 필터, 명령 변화율 제한, DB 연결, 영상 파일명 생성 등을 담당한다.
# =========================
def clamp_int(x, low, high):
    """
    x 값을 low 이상 high 이하로 제한한 뒤 int로 변환합니다.
    드론 RC 명령은 보통 -100~100 범위의 정수로 보내야 하므로,
    제어 계산에서 나온 실수값이 너무 커지지 않도록 상한/하한을 강제로 적용합니다.
    """
    return int(max(low, min(high, x)))

# yaw오차를 조정할때 쓰는 함수. a = EMA_ALPHA = 0.35
def ema(prev, x, a):
    """
    지수이동평균(EMA, Exponential Moving Average)을 계산한다.
    prev가 None이면 첫 측정값 x를 그대로 사용하고,
    이후에는 새 값 x와 이전 필터값 prev를 alpha 비율로 섞어준다.
    yaw 오차가 프레임마다 흔들리는 것을 줄여 드론이 덜 떨리게 만드는 역할이다.
    """
    return x if prev is None else (a * x + (1.0 - a) * prev)


def slew(prev, target, step):
    """
    명령값이 한 번에 너무 크게 바뀌지 않도록 변화량을 제한한다.

    예를 들어 yaw 명령이 0에서 30으로 바로 튀면 드론이 급하게 회전할 수도 있다.
    이 함수는 이전 명령 prev에서 target까지 step만큼씩 천천히 따라가도록 만든다.
    """
    if prev is None:
        return target
    if target > prev + step:
        return prev + step
    if target < prev - step:
        return prev - step
    return target

# 각도 차이를 -90도 ~ +90도로 지정하는 함수.

def parallel_err_deg(angle_deg):
    """

    """
    return ((angle_deg + 90.0) % 180.0) - 90.0


def undistort_pts(px_pts_4x2):
    """
    픽셀 좌표 4개를 카메라 보정 파라미터로 왜곡 보정합니다.
    
    ArUco 꼭짓점 좌표는 렌즈 왜곡의 영향을 받습니다.
    왜곡을 줄인 좌표에서 대각선 각도를 계산하면 yaw 정렬이 더 정확해집니다.
    """
    pts = px_pts_4x2.reshape(-1, 1, 2).astype(np.float32)
    und = cv2.undistortPoints(pts, cameraMatrix, distCoeffs)
    return und.reshape(-1, 2)


def put_latest(q: queue.Queue, frame):
    """
    큐에 최신 프레임 하나만 유지하도록 넣습니다.
    
    send_q는 maxsize=1입니다. 서버 전송이 느려도 오래된 프레임이 밀리지 않게,
    큐가 가득 차 있으면 기존 프레임을 버리고 최신 프레임으로 교체합니다.
    """
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
    """
    현재 시간을 YYYYMMDDHHMMSS 문자열로 반환합니다.
    
    DB UPDATE_TIME, 업로드 메타데이터 saved_at 등에 동일한 포맷의 시간을 쓰기 위한 함수입니다.
    """
    return datetime.now().strftime("%Y%m%d%H%M%S")


def connect_db():
    """
    SQL Server에 연결하고 pyodbc connection 객체를 반환합니다.
    
    DB_PASS 환경변수가 없으면 즉시 예외를 발생시켜, 비밀번호 누락을 명확히 알려줍니다.
    ODBC Driver 18, 암호화 연결, TrustServerCertificate=yes 옵션을 사용합니다.
    """
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
    """
    드론 상태 한 건을 dbo.T_DRONE_STATUS 테이블에 INSERT합니다.
    
    drone_id, update_time, armed, connected, battery percent 값을 받아 DB에 저장하고 commit합니다.
    """
    sql = """
    INSERT INTO dbo.T_DRONE_STATUS ([DRONE_ID], [UPDATE_TIME], [ARMED], [CONNECTED], [PERCENT])
    VALUES (?, ?, ?, ?, ?)
    """
    cur = conn.cursor()
    cur.execute(sql, (drone_id, update_time, armed, connected, float(percent)))
    conn.commit()


def safe_set_downvision(tello_obj, enable: bool):
    """
    Tello의 바닥 카메라 모드(downvision)를 켜거나 끕니다.
    
    enable=True이면 downvision 1, False이면 downvision 0 명령을 보냅니다.
    실패해도 프로그램이 바로 죽지 않도록 예외를 잡아 경고만 출력합니다.
    """
    try:
        cmd = f"downvision {1 if enable else 0}"
        # 드론 SDK 호출은 네트워크 I/O가 포함되므로, 동시에 여러 스레드가 접근하지 못하게 lock으로 보호합니다.
        with tello_lock:
            resp = tello_obj.send_command_with_return(cmd)
        print(cmd, "->", resp)
        return resp
    except Exception as e:
        print("[WARN] downvision switch failed:", e)
        return None


def start_sdk_move(tello_obj, move_func, dist_cm):
    """
    Tello SDK의 단일 이동 명령(move_up 등)을 안전하게 시작합니다.
    
    먼저 RC 명령을 0으로 보내 드론을 멈춘 뒤, 속도를 설정하고 move_func(dist_cm)을 호출합니다.
    반환값은 '이동이 끝났다고 간주할 수 있는 시간'입니다.
    """
    try:
        # 드론 SDK 호출은 네트워크 I/O가 포함되므로, 동시에 여러 스레드가 접근하지 못하게 lock으로 보호합니다.
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
    """
    긴 이동을 여러 거리로 나누어 순서대로 실행합니다.
    
    Tello SDK 이동 명령은 환경에 따라 긴 거리에서 실패하거나 오차가 커질 수 있으므로,
    200cm + 330cm처럼 분할 이동하면 테스트와 보정이 쉬워집니다.
    """
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
    """
    마커 ID와 현재 시각을 이용해 mp4 저장 경로를 만듭니다.
    
    같은 분 안에 같은 marker_id로 여러 파일이 생기면 _1, _2처럼 번호를 붙여
    기존 파일을 덮어쓰지 않도록 합니다.
    """
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

# 저장한 영상을 업로드 큐에 넣는 것.
# 실제 http업로드는 upload worker 스레드가 담당하니, 메인루프는 파일 경로와 메타데이터만 큐에 넣고, 바로 다음 작업을 계속할 수 있다.
# put_wnowait를 사용해서, 큐가 꽉 찬 경우 에러를 발생시킴 
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

# 전면 카메라 녹화를 종료하는 함수.
def stop_video_recording():
    # video_writer는 현재 열려있는 open cv 객체. path는 저장경로 , marker_id는 지금 어떤 마커를 녹화하는지. 현재 녹화중인 프레임 크기 
    global video_writer, video_recording_path, video_recording_marker_id, video_recording_size
    global last_video_write_t # 마지막으로 프레임을 영상에 write한 시간이다.

    saved_path = video_recording_path
    saved_marker_id = video_recording_marker_id

    if video_writer is not None:
        try:
            # VideoWriter.release() 직후 바로 POST하면 파일 flush가 덜 끝나서
            # 서버에서 multipart body parsing error가 날 수 있으므로 잠깐 안정화한다.
            video_writer.release() # opencv의 videoCapture의 장치를 닫고, 메모리를 해제한다.
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
    """
    특정 marker_id에 대한 전면 카메라 녹화를 준비 상태로 만듭니다.
    
    바로 녹화하지 않고 front_record_arm_t에 현재 시간을 저장합니다.
    이후 FRONT_RECORD_DELAY_S가 지난 뒤 프레임 해상도 조건까지 만족하면 실제 녹화가 시작됩니다.
    """
    global front_record_armed, front_record_arm_t, front_record_target_id
    stop_video_recording()
    front_record_armed = (marker_id is not None)
    front_record_target_id = marker_id
    front_record_arm_t = time.time()


def disarm_front_record():
    """
    전면 녹화 준비 상태를 해제하고, 진행 중인 녹화가 있으면 종료합니다.
    
    카메라를 바닥 카메라로 바꾸거나 착륙/종료할 때 호출하여 잘못된 구간이 녹화되지 않게 합니다.
    """
    global front_record_armed, front_record_arm_t, front_record_target_id
    front_record_armed = False
    front_record_arm_t = 0.0
    front_record_target_id = None
    stop_video_recording()


def update_video_recording(frame_bgr, should_record: bool, marker_id: int):
    """
    현재 프레임을 영상 파일에 쓸지 판단하고 VideoWriter를 관리합니다.
    
    should_record가 False이면 녹화를 종료합니다.
    True이면 marker_id와 프레임 크기에 맞는 VideoWriter를 열고,
    VIDEO_FPS에 맞춰 일정 간격으로 프레임을 저장합니다.
    """
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
    """
    Tello 카메라를 전면 카메라로 전환합니다.
    
    downvision을 끄고 is_downvision_active 플래그를 False로 바꿉니다.
    카메라 전환 직후 프레임이 안정될 시간을 주기 위해 짧게 대기합니다.
    """
    global is_downvision_active
    safe_set_downvision(tello_obj, False)
    is_downvision_active = False
    time.sleep(0.3)


def switch_to_down_camera(tello_obj):
    """
    Tello 카메라를 바닥 카메라(downvision)로 전환합니다.
    
    전면 녹화를 먼저 해제한 뒤 downvision을 켭니다.
    마커 검색/정렬은 바닥 카메라에서 수행됩니다.
    """
    global is_downvision_active
    disarm_front_record()
    safe_set_downvision(tello_obj, True)
    is_downvision_active = True
    time.sleep(0.3)


def marker_video_view_url(marker_id: int):
    """
    특정 마커 ID에 대한 서버 영상 조회 URL을 만듭니다.
    
    STREAM_VIDEO_BY_MARKER_URL_TEMPLATE 값이 비어 있으면 실질적인 URL은 만들어지지 않습니다.
    """
    if marker_id is None:
        return None
    return STREAM_VIDEO_BY_MARKER_URL_TEMPLATE.format(marker_id=marker_id)



def wait_until_file_ready(path: str, checks: int = 3, interval: float = 0.3) -> int:
    """
    영상 파일 크기가 안정될 때까지 기다립니다.
    
    VideoWriter.release() 직후에는 파일이 아직 완전히 디스크에 기록되지 않았을 수 있습니다.
    파일 크기가 연속으로 checks번 동일하면 업로드 가능한 상태라고 판단합니다.
    """
    """
    VideoWriter.release() 직후 파일이 완전히 flush될 때까지 기다린다.
    파일 크기가 연속으로 같은 값이면 안정화된 것으로 보고 반환한다.
    """
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
    """
    저장된 mp4 파일을 서버의 영상 업로드 엔드포인트로 전송합니다.
    
    파일을 bytes로 읽어 multipart/form-data 요청을 만들고,
    Content-Type은 requests가 boundary까지 자동 생성하도록 직접 지정하지 않습니다.
    FastAPI에서 multipart 파싱 오류가 나는 것을 막기 위한 핵심 부분입니다.
    """
    """
    서버는 건드리지 않고 기존 FastAPI endpoint 그대로 사용:
      POST /video_feed
      Header: X-API-Key
      multipart/form-data:
        file, marker_id, drone_id, cam_id, saved_at

    중요:
    - Content-Type을 직접 지정하지 않는다.
    - requests가 boundary 포함 multipart/form-data Content-Type을 만들게 한다.
    - 파일은 bytes로 먼저 읽어서 Content-Length가 정확히 잡히게 한다.
    - trust_env=False로 프록시/환경변수 간섭을 피한다.
    """
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

    # 절대 Content-Type을 직접 넣지 말 것.
    # requests가 boundary 포함 Content-Type을 자동 생성해야 FastAPI가 파싱 가능하다.
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

    # 환경 프록시/세션 헤더 간섭 방지
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
    """
    upload_q에 들어온 영상 업로드 작업을 백그라운드에서 처리합니다.
    
    stop_evt가 설정되어도 큐에 남아 있는 영상은 모두 전송하려고 시도한 뒤 종료합니다.
    업로드 실패는 전체 프로그램을 죽이지 않고 로그만 남깁니다.
    """
    while True:
        # 종료 신호가 와도 upload_q에 남은 영상은 끝까지 보낸다.
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
        # 예외 발생, ESC 종료 등 어떤 방식으로 루프가 끝나도 장비/스레드/파일을 정리하기 위한 블록입니다.
        finally:
            try:
                upload_q.task_done()
            except Exception:
                pass

    print("[UPLOAD] worker stopped.")


def sender_worker():
    """
    send_q에 들어온 최신 프레임을 서버로 전송하는 백그라운드 스레드입니다.
    
    MAX_SEND_FPS로 전송 빈도를 제한하고, 필요하면 프레임 폭을 SERVER_RESIZE_W로 줄입니다.
    JPEG로 압축한 뒤 base64 문자열로 JSON payload에 담아 POST합니다.
    """
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

        # 현재 프레임의 높이와 너비를 얻어 중심선, 허용 밴드, 해상도 조건 계산에 사용합니다.
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
        # 예외 발생, ESC 종료 등 어떤 방식으로 루프가 끝나도 장비/스레드/파일을 정리하기 위한 블록입니다.
        finally:
            last_sent = time.time()


def db_worker():
    """
    일정 간격으로 드론 상태를 DB에 기록하는 백그라운드 스레드입니다.
    
    배터리 잔량, 비행 여부, 연결 상태를 읽어 DB에 INSERT합니다.
    연결이 끊기거나 INSERT 실패가 발생하면 다음 루프에서 재연결을 시도합니다.
    """
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


# =========================
# ===== 상태머신 =====
# 메인 while 루프는 현재 mode 값에 따라 SEARCH, ALIGN, HOVER, MOVE, LAND 중 필요한 작업만 실행한다.
# =========================
# 대기 상태입니다. 드론이 착륙해 있고 사용자의 t 키 입력을 기다리는것.
MODE_IDLE             = "IDLE"
# 이륙 직후 전면 카메라 상태에서 잠시 호버링하는 상태이다.
MODE_HOVER_PRE_UP     = "HOVER_PRE_UP"
# 바닥 마커 검색을 위해 downvision 카메라로 전환하는 상태이다.
MODE_SWITCH_DOWNCAM   = "SWITCH_DOWNVISION"
# 바닥 카메라 검색 전에 지정 높이만큼 상승하는 상태이다.
MODE_MOVE_UP          = "MOVE_UP_130"

# 진행 방향 첫 번째 마커 ID1을 검색하는 상태이다.
MODE_SEARCH_ID1       = "SEARCH_ID1"
# ID1 기준으로 yaw, x, y 정렬을 수행하는 상태이다.
MODE_ALIGN_ID1        = "ALIGN_ID1 (YAW->FB->LR)"
# ID1 정렬 후 전면 카메라로 촬영하며 호버링하는 상태이다.
MODE_HOVER_AFTER_ID1  = "HOVER_AFTER_ID1_10"
# ID1 촬영/재정렬 후 오른쪽으로 분할 이동하는 상태이다.
MODE_MOVE_RIGHT_270   = "MOVE_RIGHT_270"

# 진행 방향 두 번째 마커 ID2를 검색하는 상태이다.
MODE_SEARCH_ID2       = "SEARCH_ID2"
# ID2 기준으로 정렬하는 상태이다.
MODE_ALIGN_ID2        = "ALIGN_ID2 (YAW->FB->LR)"
# ID2 정렬 후 전면 카메라로 촬영하며 호버링하는 상태이다.
MODE_HOVER_AFTER_ID2  = "HOVER_AFTER_ID2_10"
# ID2 촬영/재정렬 후 오른쪽으로 이동하는 상태이다.
MODE_MOVE_RIGHT_AFTER_ID2 = "MOVE_RIGHT_AFTER_ID2_180"

# 진행 방향 세 번째 마커 ID3을 검색하는 상태이다.
MODE_SEARCH_ID3       = "SEARCH_ID3"
# ID3 기준으로 정렬하는 상태이다.
MODE_ALIGN_ID3        = "ALIGN_ID3 (YAW->FB->LR)"
# ID3 정렬 후 전면 카메라로 촬영하며 호버링하는 상태이다.
MODE_HOVER_AFTER_ID3  = "HOVER_AFTER_ID3_10"
# ID3 촬영/재정렬 후 오른쪽으로 이동하는 상태이다.
MODE_MOVE_RIGHT_AFTER_ID3 = "MOVE_RIGHT_AFTER_ID3_180"

# 진행 방향 마지막 마커 ID4를 검색하는 상태이다.
MODE_SEARCH_ID4       = "SEARCH_ID4"
# ID4 기준으로 정렬하는 상태이다.
MODE_ALIGN_ID4        = "ALIGN_ID4 (YAW->FB->LR)"
# ID4 정렬 후 전면 카메라로 촬영하며 호버링하는 상태이다.
MODE_HOVER_AFTER_ID4  = "HOVER_AFTER_ID4_10"

# ID4 촬영/재정렬 후 복귀 방향으로 왼쪽 이동하는 상태이다.
MODE_MOVE_LEFT_AFTER_ID4         = "MOVE_LEFT_AFTER_ID4_180"
# 복귀 경로에서 ID3을 다시 검색하는 상태이다.
MODE_SEARCH_RETURN_ID3           = "SEARCH_RETURN_ID3"
# 복귀 경로 ID3 기준으로 정렬하는 상태이다.
MODE_ALIGN_RETURN_ID3            = "ALIGN_RETURN_ID3 (YAW->FB->LR)"
# 복귀 ID3 정렬 후 전면 카메라로 촬영하는 상태이다.
MODE_HOVER_AFTER_RETURN_ID3      = "HOVER_AFTER_RETURN_ID3_10"
# 복귀 ID3 촬영/재정렬 후 왼쪽 이동하는 상태이다.
MODE_MOVE_LEFT_AFTER_RETURN_ID3  = "MOVE_LEFT_AFTER_RETURN_ID3_180"
# 복귀 경로에서 ID2를 검색하는 상태이다.
MODE_SEARCH_RETURN_ID2           = "SEARCH_RETURN_ID2"
# 복귀 경로 ID2 기준으로 정렬하는 상태이다.
MODE_ALIGN_RETURN_ID2            = "ALI이N_RETURN_ID2 (YAW->FB->LR)"
# 복귀 ID2 정렬 후 전면 카메라로 촬영하는 상태이다.
MODE_HOVER_AFTER_RETURN_ID2      = "HOVER_AFTER_RETURN_ID2_10"
# 복귀 ID2 촬영/재정렬 후 왼쪽 이동해 ID1 방향으로 가는 상태다.
MODE_MOVE_LEFT_AFTER_RETURN_ID2  = "MOVE_LEFT_AFTER_RETURN_ID2_270"
# 복귀 경로에서 ID1을 검색하는 상태다.
MODE_SEARCH_RETURN_ID1           = "SEARCH_RETURN_ID1"
# 복귀 경로 ID1 기준으로 정렬하는 상태다.
MODE_ALIGN_RETURN_ID1            = "ALIGN_RETURN_ID1 (YAW->FB->LR)"
# 복귀 ID1 정렬 후 마지막 전면 카메라 촬영을 하는 상태다.
MODE_HOVER_AFTER_RETURN_ID1      = "HOVER_AFTER_RETURN_ID1_10"

# 녹화 후 이동 전에 방금 본 마커를 다시 검색하는 공통 재검색 상태이다.
MODE_SEARCH_POST_RECORD          = "SEARCH_POST_RECORD"
# 녹화 후 이동 전에 방금 본 마커에 다시 정렬하는 공통 재정렬 상태이다.
MODE_ALIGN_POST_RECORD           = "ALIGN_POST_RECORD (YAW->FB->LR)"

# 착륙 상태입니다. RC 정지 후 land 명령을 보내고 IDLE로 돌아감.
MODE_LAND             = "LAND"

# 현재 상태머신의 시작 상태다. 아직 이륙하지 않았으므로 IDLE에서 대기한다 
mode = MODE_IDLE

# =========================
# ===== Tello 연결 =====
# 프로그램 시작 시 드론 객체를 만들고 SDK 연결, 배터리 확인, 영상 스트림 시작을 수행한다.
# tello_lock을 사용해 여러 스레드가 동시에 드론 SDK 명령을 보내지 않도록 보호한다.
# =========================
# Tello 객체를 생성한다. 이후 모든 SDK 명령은 이 객체를 통해 전송된다.
tello = Tello()
with tello_lock:
    tello.connect()
    print("Battery:", tello.get_battery(), "%")
    tello.streamon()
    frame_read = tello.get_frame_read()

is_flying = False
is_downvision_active = False
last_rc_sent = 0.0

# =========================
# ===== 런타임 상태 =====
# 실행 중 계속 바뀌는 상태 변수들.....
# 타이머, 검색 횟수, 정렬 필터값, 녹화 객체, 마지막으로 본 마커 ID 등을 저장한다.
# =========================
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
    """
    새로운 ALIGN 단계에 들어갈 때 정렬 관련 누적 상태를 초기화한다.
    yaw 필터값, stable_cnt, 목표 RC 명령, 이전 RC 명령, 마커 중심 좌표 등을 리셋하여
    이전 마커에서 남은 값이 다음 정렬에 영향을 주지 않게 해준다.
    """
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
    """
    전면 카메라 녹화가 끝난 뒤 같은 마커를 다시 찾고 재정렬하는 절차를 시작한다.
    이동 명령을 내리기 전에 downvision으로 돌아가서 같은 marker_id에 다시 정렬한다.
    """
    global mode, post_record_realign_id, post_record_next_mode
    global search_target_id, search_timeout_s, search_start_t, search_seen_cnt
    global hover_start_t, move_end_t

    # 다음 마커 검색/재정렬을 위해 바닥 카메라로 전환한다.
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
# 실시간 프레임 서버 전송 스레드를 시작.
th_sender.start()
# 드론 상태 DB 기록 스레드를 시작.
th_db.start()
# 녹화 영상 파일 업로드 스레드를 시작.
th_uploader.start()

try:
    # 메인 루프. 프로그램이 종료될 때까지 매 프레임 영상 처리와 상태머신을 반복한다.
    while True:
        # dji tello py가 최신으로 갱신해 둔 카메라 프레임을 가져온다.
        frame = frame_read.frame
        # 프레임이 아직 준비되지 않았거나 일시적으로 끊겼다면 이번 반복은 건너뛴다.
        if frame is None:
            continue

        # Tello 프레임은 RGB로 들어오므로, OpenCV 표시/저장에 맞게 BGR로 변환한다.
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        # 현재 프레임의 높이와 너비를 얻어 중심선, 허용 밴드, 해상도 조건 계산에 사용한다.
        h, w = frame.shape[:2]
        # 화면 중심 x좌표입니다. 마커 중심이 이 근처에 오도록 전후 이동을 제어한다.
        cx = w / 2.0
        # 화면 중심 y좌표입니다. 마커 중심이 이 근처에 오도록 좌우 이동을 제어한다.
        cy = h / 2.0

        band_x = CENTER_BAND_RATIO_X * w
        left_band = int(cx - band_x)
        right_band = int(cx + band_x)

        band_y = CENTER_BAND_RATIO_Y * h
        top_band = int(cy - band_y)
        bottom_band = int(cy + band_y)

        # =========================
        # ===== 마커 검출 =====
# 현재 상태가 SEARCH/ALIGN일 때만 ArUco 검출을 수행한다.
        # =========================
        # 현재 상태가 마커 검색/정렬 관련 상태인지 확인합니다. True일 때만 ArUco 검출을 수행합니다.
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
        # ArUco 검출 결과를 저장할 변수. 검출하지 않거나 실패하면 None으로 남는다
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
            # ArUco 검출은 보통 흑백 영상에서 수행하므로 BGR 프레임을 grayscale로 변환.
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # 현재 프레임에서 ArUco 마커의 꼭짓점 좌표(corners)와 ID(ids)를 검출.
            corners, ids, _ = detector.detectMarkers(gray)

            if ids is not None and len(ids) > 0:
                # OpenCV가 반환한 ids 배열을 일반 파이썬 int 리스트로 바꿔 ID 포함 여부를 쉽게 검사합니다.
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
            # 현재 상태에서 목표 ID1이 보였으므로 마지막으로 확인한 마커 ID를 갱신합니다.
            last_seen_marker_id = FLOOR_MARKER_ID1_ALIGN
        elif mode in [MODE_SEARCH_ID2, MODE_ALIGN_ID2, MODE_SEARCH_RETURN_ID2, MODE_ALIGN_RETURN_ID2] and id2_visible:
            # 현재 상태에서 목표 ID2가 보였으므로 마지막으로 확인한 마커 ID를 갱신합니다.
            last_seen_marker_id = FLOOR_MARKER_ID2_ALIGN
        elif mode in [MODE_SEARCH_ID3, MODE_ALIGN_ID3, MODE_SEARCH_RETURN_ID3, MODE_ALIGN_RETURN_ID3] and id3_visible:
            # 현재 상태에서 목표 ID3이 보였으므로 마지막으로 확인한 마커 ID를 갱신합니다.
            last_seen_marker_id = FLOOR_MARKER_ID3_ALIGN
        elif mode in [MODE_SEARCH_ID4, MODE_ALIGN_ID4] and id4_visible:
            # 현재 상태에서 목표 ID4가 보였으므로 마지막으로 확인한 마커 ID를 갱신합니다.
            last_seen_marker_id = FLOOR_MARKER_ID4_ALIGN
        elif mode in [MODE_SEARCH_POST_RECORD, MODE_ALIGN_POST_RECORD]:
            if post_record_realign_id == FLOOR_MARKER_ID1_ALIGN and id1_visible:
                # 현재 상태에서 목표 ID1이 보였으므로 마지막으로 확인한 마커 ID를 갱신합니다.
                last_seen_marker_id = FLOOR_MARKER_ID1_ALIGN
            elif post_record_realign_id == FLOOR_MARKER_ID2_ALIGN and id2_visible:
                # 현재 상태에서 목표 ID2가 보였으므로 마지막으로 확인한 마커 ID를 갱신합니다.
                last_seen_marker_id = FLOOR_MARKER_ID2_ALIGN
            elif post_record_realign_id == FLOOR_MARKER_ID3_ALIGN and id3_visible:
                # 현재 상태에서 목표 ID3이 보였으므로 마지막으로 확인한 마커 ID를 갱신합니다.
                last_seen_marker_id = FLOOR_MARKER_ID3_ALIGN
            elif post_record_realign_id == FLOOR_MARKER_ID4_ALIGN and id4_visible:
                # 현재 상태에서 목표 ID4가 보였으므로 마지막으로 확인한 마커 ID를 갱신합니다.
                last_seen_marker_id = FLOOR_MARKER_ID4_ALIGN

        if mode in [
            MODE_SEARCH_ID1, MODE_SEARCH_ID2, MODE_SEARCH_ID3, MODE_SEARCH_ID4,
            MODE_SEARCH_RETURN_ID3, MODE_SEARCH_RETURN_ID2, MODE_SEARCH_RETURN_ID1,
            MODE_SEARCH_POST_RECORD,
        ] and ids is not None:
            # SEARCH 상태에서는 화면에 검출된 마커 테두리와 ID를 그려 디버깅을 돕습니다.
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # =========================
        # ===== SEARCH =====
# SEARCH 상태는 목표 마커가 화면에 보이는지 확인하는 단계입니다.
# 목표 ID가 안정적으로 보이면 ALIGN 상태로 넘어가고, 제한 시간 안에 못 찾으면 안전하게 LAND로 전환합니다.
        # =========================
        if mode in [
            MODE_SEARCH_ID1, MODE_SEARCH_ID2, MODE_SEARCH_ID3, MODE_SEARCH_ID4,
            MODE_SEARCH_RETURN_ID3, MODE_SEARCH_RETURN_ID2, MODE_SEARCH_RETURN_ID1,
            MODE_SEARCH_POST_RECORD,
        ] and is_flying:
            # 검색 시작 시간이 아직 없다면 지금 시간을 기록해 타임아웃 계산 기준으로 삼습니다.
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

            # 목표 마커가 연속으로 보이면 카운트를 올리고, 한 프레임이라도 놓치면 0으로 리셋합니다.
            search_seen_cnt = search_seen_cnt + 1 if found else 0

            # 목표 마커가 필요한 프레임 수만큼 안정적으로 보였으면 검색 성공으로 보고 ALIGN 단계로 넘어갑니다.
            if search_seen_cnt >= SEARCH_STABLE_N:
                if mode == MODE_SEARCH_ID1:
                    mode = MODE_ALIGN_ID1
                    active_align_id = FLOOR_MARKER_ID1_ALIGN
                    # 새 마커 정렬을 시작하기 전에 이전 정렬에서 남은 필터/명령값을 모두 초기화합니다.
                    reset_align_state()
                elif mode == MODE_SEARCH_ID2:
                    mode = MODE_ALIGN_ID2
                    active_align_id = FLOOR_MARKER_ID2_ALIGN
                    # 새 마커 정렬을 시작하기 전에 이전 정렬에서 남은 필터/명령값을 모두 초기화합니다.
                    reset_align_state()
                elif mode == MODE_SEARCH_ID3:
                    mode = MODE_ALIGN_ID3
                    active_align_id = FLOOR_MARKER_ID3_ALIGN
                    # 새 마커 정렬을 시작하기 전에 이전 정렬에서 남은 필터/명령값을 모두 초기화합니다.
                    reset_align_state()
                elif mode == MODE_SEARCH_ID4:
                    mode = MODE_ALIGN_ID4
                    active_align_id = FLOOR_MARKER_ID4_ALIGN
                    # 새 마커 정렬을 시작하기 전에 이전 정렬에서 남은 필터/명령값을 모두 초기화합니다.
                    reset_align_state()
                elif mode == MODE_SEARCH_RETURN_ID3:
                    mode = MODE_ALIGN_RETURN_ID3
                    active_align_id = FLOOR_MARKER_ID3_ALIGN
                    # 새 마커 정렬을 시작하기 전에 이전 정렬에서 남은 필터/명령값을 모두 초기화합니다.
                    reset_align_state()
                elif mode == MODE_SEARCH_RETURN_ID2:
                    mode = MODE_ALIGN_RETURN_ID2
                    active_align_id = FLOOR_MARKER_ID2_ALIGN
                    # 새 마커 정렬을 시작하기 전에 이전 정렬에서 남은 필터/명령값을 모두 초기화합니다.
                    reset_align_state()
                elif mode == MODE_SEARCH_RETURN_ID1:
                    mode = MODE_ALIGN_RETURN_ID1
                    active_align_id = FLOOR_MARKER_ID1_ALIGN
                    # 새 마커 정렬을 시작하기 전에 이전 정렬에서 남은 필터/명령값을 모두 초기화합니다.
                    reset_align_state()
                else:
                    mode = MODE_ALIGN_POST_RECORD
                    active_align_id = post_record_realign_id
                    # 새 마커 정렬을 시작하기 전에 이전 정렬에서 남은 필터/명령값을 모두 초기화합니다.
                    reset_align_state()

                search_start_t = None
                search_seen_cnt = 0

            # 제한 시간 안에 목표 마커를 못 찾으면 더 이상 헤매지 않고 안전 착륙 단계로 전환합니다.
            if search_start_t is not None and (time.time() - search_start_t) >= search_timeout_s:
                print(f"[INFO] ID{search_target_id} not found within {search_timeout_s}s -> LAND")
                mode = MODE_LAND
                search_start_t = None
                search_seen_cnt = 0

        # =========================
        # ===== ALIGN =====
# ALIGN 상태는 마커를 기준으로 드론 자세와 위치를 맞추는 핵심 제어 단계입니다.
# 순서: yaw 회전 정렬 → 화면 X 중심 맞춤(FB) → 화면 Y 중심 맞춤(LR) → 안정 프레임 누적.
        # =========================
        if mode in [
            MODE_ALIGN_ID1, MODE_ALIGN_ID2, MODE_ALIGN_ID3, MODE_ALIGN_ID4,
            MODE_ALIGN_RETURN_ID3, MODE_ALIGN_RETURN_ID2, MODE_ALIGN_RETURN_ID1,
            MODE_ALIGN_POST_RECORD,
        ] and is_flying:
            # 정렬 자체도 제한 시간을 둡니다. 너무 오래 보정하다 실패하면 LAND로 전환합니다.
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

                # 목표 마커가 실제로 보이고 꼭짓점 좌표도 있을 때만 정렬 계산을 수행합니다.
                if visible and (c4 is not None):
                    last_seen_t = time.time()

                    # 마커 네 꼭짓점의 x좌표 평균입니다. 화면상 마커 중심 x 위치로 사용합니다.
                    marker_cx = float(np.mean(c4[:, 0]))
                    # 마커 네 꼭짓점의 y좌표 평균입니다. 화면상 마커 중심 y 위치로 사용합니다.
                    marker_cy = float(np.mean(c4[:, 1]))
                    # 마커 중심 x가 화면 중앙 허용 밴드 안에 들어왔는지 확인합니다.
                    centered_x = (left_band <= marker_cx <= right_band)
                    # 마커 중심 y가 화면 중앙 허용 밴드 안에 들어왔는지 확인합니다.
                    centered_y = (top_band <= marker_cy <= bottom_band)

                    # 렌즈 왜곡을 보정한 꼭짓점 좌표로 yaw 각도를 계산합니다.
                    und = undistort_pts(c4)
                    # ArUco 꼭짓점 순서 중 첫 번째는 일반적으로 좌상단(top-left)입니다.
                    tl = und[0]
                    # 세 번째 꼭짓점은 일반적으로 우하단(bottom-right)입니다. 대각선 벡터 계산에 사용합니다.
                    br = und[2]
                    # 좌상단에서 우하단으로 향하는 대각선 벡터입니다. 이 방향으로 마커 회전각을 추정합니다.
                    v = br - tl
                    # 대각선 벡터의 atan2 각도를 도 단위로 변환합니다.
                    diag_angle = math.degrees(math.atan2(float(v[1]), float(v[0])))

                    # 현재 대각선 각도와 목표 대각선 각도의 차이를 -90~+90도 범위의 오차로 계산합니다.
                    err = parallel_err_deg(diag_angle - FLOOR_TARGET_DIAG_DEG)
                    # yaw 오차를 EMA로 부드럽게 만들어 프레임 노이즈로 인한 떨림을 줄입니다.
                    yaw_f = ema(yaw_f, err, EMA_ALPHA)

                    # yaw 오차가 허용 각도보다 크면 위치 보정보다 회전 보정을 먼저 수행합니다.
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

                    # yaw가 충분히 맞은 뒤, 화면 x축 중심이 맞지 않으면 전후 이동으로 x 위치를 보정합니다.
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

                    # yaw와 x 중심이 맞은 뒤, 화면 y축 중심이 맞지 않으면 좌우 이동으로 y 위치를 보정합니다.
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
                        # 안정 상태가 연속으로 유지된 프레임 수를 누적합니다. STABLE_N에 도달해야 정렬 완료로 인정합니다.
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

                    # 정렬이 충분히 안정적으로 유지되면 다음 상태로 넘어갑니다.
                    if stable_cnt >= STABLE_N:
                        try:
                            with tello_lock:
                                tello.send_rc_control(0, 0, 0, 0)
                        except Exception:
                            pass

                        last_seen_marker_id = mid

                        if mode == MODE_ALIGN_ID1:
                            # 마커 정렬이 끝났으므로 촬영을 위해 전면 카메라로 전환합니다.
                            switch_to_front_camera(tello)
                            # 현재 정렬된 마커 ID를 기준으로 전면 카메라 녹화를 준비합니다.
                            arm_front_record(mid)
                            print("[STATE] ID1 aligned -> remember ID1 -> switch FRONT CAM -> HOVER 10s (record) -> RE-ALIGN same ID -> RIGHT 135+135 -> SEARCH ID2")
                            mode = MODE_HOVER_AFTER_ID1
                            hover_start_t = time.time()
                        elif mode == MODE_ALIGN_ID2:
                            # 마커 정렬이 끝났으므로 촬영을 위해 전면 카메라로 전환합니다.
                            switch_to_front_camera(tello)
                            # 현재 정렬된 마커 ID를 기준으로 전면 카메라 녹화를 준비합니다.
                            arm_front_record(mid)
                            print("[STATE] ID2 aligned -> remember ID2 -> switch FRONT CAM -> HOVER 10s (record) -> RE-ALIGN same ID -> RIGHT 90+90 -> SEARCH ID3")
                            mode = MODE_HOVER_AFTER_ID2
                            hover_start_t = time.time()
                        elif mode == MODE_ALIGN_ID3:
                            # 마커 정렬이 끝났으므로 촬영을 위해 전면 카메라로 전환합니다.
                            switch_to_front_camera(tello)
                            # 현재 정렬된 마커 ID를 기준으로 전면 카메라 녹화를 준비합니다.
                            arm_front_record(mid)
                            print("[STATE] ID3 aligned -> remember ID3 -> switch FRONT CAM -> HOVER 10s (record) -> RE-ALIGN same ID -> RIGHT 90+90 -> SEARCH ID4")
                            mode = MODE_HOVER_AFTER_ID3
                            hover_start_t = time.time()
                        elif mode == MODE_ALIGN_ID4:
                            # 마커 정렬이 끝났으므로 촬영을 위해 전면 카메라로 전환합니다.
                            switch_to_front_camera(tello)
                            # 현재 정렬된 마커 ID를 기준으로 전면 카메라 녹화를 준비합니다.
                            arm_front_record(mid)
                            print("[STATE] ID4 aligned -> remember ID4 -> switch FRONT CAM -> HOVER 10s (record) -> RE-ALIGN same ID -> LEFT 90+90 -> SEARCH RETURN ID3")
                            mode = MODE_HOVER_AFTER_ID4
                            hover_start_t = time.time()
                        elif mode == MODE_ALIGN_RETURN_ID3:
                            # 마커 정렬이 끝났으므로 촬영을 위해 전면 카메라로 전환합니다.
                            switch_to_front_camera(tello)
                            # 현재 정렬된 마커 ID를 기준으로 전면 카메라 녹화를 준비합니다.
                            arm_front_record(mid)
                            print("[STATE] RETURN ID3 aligned -> remember ID3 -> switch FRONT CAM -> HOVER 10s (record) -> RE-ALIGN same ID -> LEFT 90+90 -> SEARCH RETURN ID2")
                            mode = MODE_HOVER_AFTER_RETURN_ID3
                            hover_start_t = time.time()
                        elif mode == MODE_ALIGN_RETURN_ID2:
                            # 마커 정렬이 끝났으므로 촬영을 위해 전면 카메라로 전환합니다.
                            switch_to_front_camera(tello)
                            # 현재 정렬된 마커 ID를 기준으로 전면 카메라 녹화를 준비합니다.
                            arm_front_record(mid)
                            print("[STATE] RETURN ID2 aligned -> remember ID2 -> switch FRONT CAM -> HOVER 10s (record) -> RE-ALIGN same ID -> LEFT 135+135 -> SEARCH RETURN ID1")
                            mode = MODE_HOVER_AFTER_RETURN_ID2
                            hover_start_t = time.time()
                        elif mode == MODE_ALIGN_RETURN_ID1:
                            # 마커 정렬이 끝났으므로 촬영을 위해 전면 카메라로 전환합니다.
                            switch_to_front_camera(tello)
                            # 현재 정렬된 마커 ID를 기준으로 전면 카메라 녹화를 준비합니다.
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
                    # ALIGN 중 마커가 잠깐 사라질 수 있으므로 LOST_TIMEOUT_S를 넘겼을 때만 명령을 0으로 리셋합니다.
                    if (time.time() - last_seen_t) > LOST_TIMEOUT_S:
                        target_yaw_cmd = target_fb_cmd = target_lr_cmd = 0
                        stable_cnt = 0

        # =========================
        # ===== TAKEOFF 시퀀스 =====
# 이륙 직후 바로 바닥 카메라를 쓰지 않고 전면 카메라 상태로 잠시 호버링한 뒤 상승/검색 절차로 들어갑니다.
# 각 HOVER/MOVE 상태는 시간이 끝나거나 이동 명령이 끝났다고 판단되면 다음 상태로 전환합니다.
        # =========================
        # 이륙 직후 사전 호버 시간이 끝났는지 확인하는 상태 처리입니다.
        if mode == MODE_HOVER_PRE_UP and is_flying:
            if hover_start_t is None:
                hover_start_t = time.time()
            if (time.time() - hover_start_t) >= HOVER_BEFORE_UP_S:
                mode = MODE_SWITCH_DOWNCAM
                hover_start_t = None
                move_end_t = None

        # downvision 전환 상태입니다. 전환 후 바로 상승 상태로 넘어갑니다.
        if mode == MODE_SWITCH_DOWNCAM and is_flying:
            # 다음 마커 검색/재정렬을 위해 바닥 카메라로 전환합니다.
            switch_to_down_camera(tello)
            mode = MODE_MOVE_UP

        # 상승 이동 상태입니다. move_up 명령은 한 번만 시작하고, 이후 move_end_t로 완료를 판단합니다.
        if mode == MODE_MOVE_UP and is_flying:
            if move_end_t is None:
                try:
                    # 설정된 높이만큼 상승합니다. 이후 move_end_t가 지나면 다음 검색 상태로 넘어갑니다.
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

        # ID1 촬영 호버 상태입니다. 지정 시간이 지나면 ID1 재정렬 절차를 시작합니다.
        if mode == MODE_HOVER_AFTER_ID1 and is_flying:
            if hover_start_t is None:
                hover_start_t = time.time()
            if (time.time() - hover_start_t) >= HOVER_AFTER_ID1_ALIGN_S:
                # ID1 녹화가 끝났으므로 ID1을 다시 찾아 정렬한 뒤 오른쪽 이동 상태로 넘어갑니다.
                begin_post_record_realign(FLOOR_MARKER_ID1_ALIGN, MODE_MOVE_RIGHT_270, SEARCH_TIMEOUT_ID1_S)

        # ID1 이후 오른쪽 분할 이동 상태입니다. 이동 완료 후 ID2 검색으로 넘어갑니다.
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

        # ID2 촬영 호버 상태입니다. 지정 시간이 지나면 ID2 재정렬 절차를 시작합니다.
        if mode == MODE_HOVER_AFTER_ID2 and is_flying:
            if hover_start_t is None:
                hover_start_t = time.time()
            if (time.time() - hover_start_t) >= HOVER_AFTER_ID2_ALIGN_S:
                # ID2 녹화가 끝났으므로 ID2를 다시 찾아 정렬한 뒤 다음 오른쪽 이동 상태로 넘어갑니다.
                begin_post_record_realign(FLOOR_MARKER_ID2_ALIGN, MODE_MOVE_RIGHT_AFTER_ID2, SEARCH_TIMEOUT_ID2_S)

        # ID2 이후 오른쪽 분할 이동 상태입니다. 이동 완료 후 ID3 검색으로 넘어갑니다.
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

        # ID3 촬영 호버 상태입니다. 지정 시간이 지나면 ID3 재정렬 절차를 시작합니다.
        if mode == MODE_HOVER_AFTER_ID3 and is_flying:
            if hover_start_t is None:
                hover_start_t = time.time()
            if (time.time() - hover_start_t) >= HOVER_AFTER_ID3_ALIGN_S:
                # ID3 녹화가 끝났으므로 ID3을 다시 찾아 정렬한 뒤 다음 오른쪽 이동 상태로 넘어갑니다.
                begin_post_record_realign(FLOOR_MARKER_ID3_ALIGN, MODE_MOVE_RIGHT_AFTER_ID3, SEARCH_TIMEOUT_ID3_S)

        # ID3 이후 오른쪽 분할 이동 상태입니다. 이동 완료 후 ID4 검색으로 넘어갑니다.
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

        # ID4 촬영 호버 상태입니다. 지정 시간이 지나면 ID4 재정렬 후 복귀 이동을 시작합니다.
        if mode == MODE_HOVER_AFTER_ID4 and is_flying:
            if hover_start_t is None:
                hover_start_t = time.time()
            if (time.time() - hover_start_t) >= HOVER_AFTER_ID4_ALIGN_S:
                # ID4 녹화가 끝났으므로 ID4를 다시 찾아 정렬한 뒤 복귀 방향인 왼쪽 이동으로 넘어갑니다.
                begin_post_record_realign(FLOOR_MARKER_ID4_ALIGN, MODE_MOVE_LEFT_AFTER_ID4, SEARCH_TIMEOUT_ID4_S)

        # ID4 이후 첫 복귀 이동 상태입니다. 이동 완료 후 복귀 ID3 검색으로 넘어갑니다.
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

        # 복귀 ID3 촬영 호버 상태입니다. 지정 시간이 지나면 복귀 ID3 재정렬 절차를 시작합니다.
        if mode == MODE_HOVER_AFTER_RETURN_ID3 and is_flying:
            if hover_start_t is None:
                hover_start_t = time.time()
            if (time.time() - hover_start_t) >= HOVER_AFTER_RETURN_ID3_ALIGN_S:
                # 복귀 ID3 녹화가 끝났으므로 다시 ID3에 정렬한 뒤 왼쪽으로 이동합니다.
                begin_post_record_realign(FLOOR_MARKER_ID3_ALIGN, MODE_MOVE_LEFT_AFTER_RETURN_ID3, SEARCH_TIMEOUT_ID3_S)

        # 복귀 ID3 이후 왼쪽 분할 이동 상태입니다. 이동 완료 후 복귀 ID2 검색으로 넘어갑니다.
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

        # 복귀 ID2 촬영 호버 상태입니다. 지정 시간이 지나면 복귀 ID2 재정렬 절차를 시작합니다.
        if mode == MODE_HOVER_AFTER_RETURN_ID2 and is_flying:
            if hover_start_t is None:
                hover_start_t = time.time()
            if (time.time() - hover_start_t) >= HOVER_AFTER_RETURN_ID2_ALIGN_S:
                # 복귀 ID2 녹화가 끝났으므로 다시 ID2에 정렬한 뒤 왼쪽으로 이동합니다.
                begin_post_record_realign(FLOOR_MARKER_ID2_ALIGN, MODE_MOVE_LEFT_AFTER_RETURN_ID2, SEARCH_TIMEOUT_ID2_S)

        # 복귀 ID2 이후 왼쪽 분할 이동 상태입니다. 이동 완료 후 복귀 ID1 검색으로 넘어갑니다.
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

        # 복귀 ID1 촬영 호버 상태입니다. 지정 시간이 지나면 마지막 재정렬 후 착륙합니다.
        if mode == MODE_HOVER_AFTER_RETURN_ID1 and is_flying:
            if hover_start_t is None:
                hover_start_t = time.time()
            if (time.time() - hover_start_t) >= HOVER_AFTER_RETURN_ID1_ALIGN_S:
                # 복귀 ID1 녹화가 끝났으므로 마지막으로 ID1에 재정렬한 뒤 착륙합니다.
                begin_post_record_realign(FLOOR_MARKER_ID1_ALIGN, MODE_LAND, SEARCH_TIMEOUT_ID1_S)

        # 착륙 상태 처리입니다. 드론을 멈추고 land 명령을 보낸 뒤 내부 상태를 초기화합니다.
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
            # 현재 상태머신의 시작 상태입니다. 아직 이륙하지 않았으므로 IDLE에서 대기합니다.
            mode = MODE_IDLE
            safe_set_downvision(tello, False)
            is_downvision_active = False

        # =========================
        # ===== RC 명령 전송 (ALIGN 중만) =====
# send_rc_control은 짧은 주기로 계속 보내야 하는 수동 제어 명령입니다.
# SDK move_up/move_left/move_right가 진행 중일 때는 RC 명령을 보내지 않아 명령 충돌을 피합니다.
        # =========================
        lr_cmd = fb_cmd = yaw_cmd = 0
        if is_flying:
            now = time.time()
            # SDK 이동 명령이 진행 중인지 판단합니다. 이동 중에는 RC 제어 명령을 추가로 보내지 않습니다.
            move_in_progress = (
                (mode == MODE_MOVE_UP and move_end_t is not None and now < move_end_t)
                or (mode == MODE_MOVE_RIGHT_270 and move_end_t is not None and now < move_end_t)
                or (mode == MODE_MOVE_RIGHT_AFTER_ID2 and move_end_t is not None and now < move_end_t)
                or (mode == MODE_MOVE_RIGHT_AFTER_ID3 and move_end_t is not None and now < move_end_t)
                or (mode == MODE_MOVE_LEFT_AFTER_ID4 and move_end_t is not None and now < move_end_t)
                or (mode == MODE_MOVE_LEFT_AFTER_RETURN_ID3 and move_end_t is not None and now < move_end_t)
                or (mode == MODE_MOVE_LEFT_AFTER_RETURN_ID2 and move_end_t is not None and now < move_end_t)
            )

            # 이동 명령이 없고 RC 주기가 지났을 때만 새 RC 명령을 계산해 보냅니다.
            if not move_in_progress and (now - last_rc_sent) >= RC_PERIOD:
                # 기본 목표 RC 명령은 모두 0입니다. ALIGN 상태가 아니면 드론은 제자리 유지 명령을 받습니다.
                tgt_lr = tgt_fb = tgt_yaw = 0
                if mode in [
                    MODE_ALIGN_ID1, MODE_ALIGN_ID2, MODE_ALIGN_ID3, MODE_ALIGN_ID4,
                    MODE_ALIGN_RETURN_ID3, MODE_ALIGN_RETURN_ID2, MODE_ALIGN_RETURN_ID1,
                    MODE_ALIGN_POST_RECORD,
                ]:
                    tgt_lr = clamp_int(target_lr_cmd, -MAX_LR_RC, MAX_LR_RC)
                    tgt_fb = clamp_int(target_fb_cmd, -MAX_FB_RC, MAX_FB_RC)
                    tgt_yaw = clamp_int(target_yaw_cmd, -MAX_YAW_RC, MAX_YAW_RC)

                # 좌우 명령을 목표값으로 바로 바꾸지 않고 지정 step만큼 부드럽게 변화시킵니다.
                lr_cmd = int(slew(prev_lr, tgt_lr, LR_SLEW_STEP))
                # 전후 명령도 급격히 변하지 않도록 slew 제한을 적용합니다.
                fb_cmd = int(slew(prev_fb, tgt_fb, FB_SLEW_STEP))
                # yaw 회전 명령도 부드럽게 변화시켜 급회전을 줄입니다.
                yaw_cmd = int(slew(prev_yaw, tgt_yaw, YAW_SLEW_STEP))
                prev_lr, prev_fb, prev_yaw = lr_cmd, fb_cmd, yaw_cmd

                with tello_lock:
                    # 최종 RC 명령을 Tello에 보냅니다. 상하(up/down)는 이 코드에서 ALIGN 중 사용하지 않아 0입니다.
                    tello.send_rc_control(lr_cmd, fb_cmd, 0, yaw_cmd)
                last_rc_sent = now

        # =========================
        # ===== 녹화 조건 판단 =====
# 전면 카메라 + 호버 상태 + 녹화 arm 상태 + 해상도 조건이 모두 만족될 때만 mp4 녹화를 수행합니다.
# 같은 구간의 프레임만 서버 실시간 전송 큐에도 넣습니다.
        # =========================
        # 현재 상태가 전면 카메라 녹화 대상 호버 상태인지 확인합니다.
        record_hover_mode = mode in [
            MODE_HOVER_AFTER_ID1, MODE_HOVER_AFTER_ID2, MODE_HOVER_AFTER_ID3, MODE_HOVER_AFTER_ID4,
            MODE_HOVER_AFTER_RETURN_ID3, MODE_HOVER_AFTER_RETURN_ID2, MODE_HOVER_AFTER_RETURN_ID1,
        ]
        # downvision의 낮은 해상도 프레임이 실수로 전면 영상 파일에 들어가지 않게 해상도를 확인합니다.
        front_frame_ready = (w >= FRONT_MIN_W and h >= FRONT_MIN_H)

        # 실제 녹화를 시작/유지하기 위한 모든 조건을 하나로 묶어 판단합니다.
        should_record_front = (
            is_flying
            and record_hover_mode
            and (not is_downvision_active)
            and front_record_armed
            and (front_record_target_id is not None)
            and ((time.time() - front_record_arm_t) >= FRONT_RECORD_DELAY_S)
            and front_frame_ready
        )

        # 녹화 조건에 따라 VideoWriter를 열고, 프레임을 쓰거나, 녹화를 종료합니다.
        update_video_recording(frame, should_record_front, front_record_target_id)

        # 서버에는 실제 전면카메라 녹화 구간만 전송
        # 실제 전면 녹화 중인 프레임만 서버 실시간 전송 큐에 넣습니다.
        if should_record_front:
            # 서버 전송 큐에는 가장 최신 프레임 하나만 유지하여 지연을 줄입니다.
            put_latest(send_q, frame)

        # =========================
        # ===== HUD / UI =====
# OpenCV 창에 현재 단계, 정렬 상태, 중심 가이드라인, RC 명령, 녹화 상태 등을 표시합니다.
# 이 정보는 현장 테스트 중 드론이 왜 움직이는지 파악하는 데 중요합니다.
        # =========================
        if not is_flying:
            step_text = "STEP: LANDED (press 't' to takeoff)"
        else:
            # 이륙 직후 사전 호버 시간이 끝났는지 확인하는 상태 처리입니다.
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

        # 최종 HUD가 그려진 프레임을 OpenCV 창에 표시합니다.
        cv2.imshow("Tello ArUco - ID1 -> ID2 -> ID3 -> ID4 -> RETURN ID3 -> RETURN ID2 -> RETURN ID1", frame)

        # =========================
        # ===== 키 입력 =====
# 키보드 입력으로 이륙(t), 착륙(q), 종료(ESC)를 처리합니다.
# 사용자가 언제든 수동으로 착륙을 요청할 수 있도록 q 키를 둡니다.
        # =========================
        # 1ms 동안 키 입력을 확인합니다. 반환값을 0xFF와 AND하여 키 코드만 추출합니다.
        key = cv2.waitKey(1) & 0xFF

        if key == 27:
            print("[USER] ESC -> quit")
            break

        # t 키: 이륙 및 자동 비행 시퀀스를 시작합니다.
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

                # 마커 정렬이 끝났으므로 촬영을 위해 전면 카메라로 전환합니다.
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

        # q 키: 사용자가 즉시 착륙을 요청하는 수동 안전 동작입니다.
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
                # 현재 상태머신의 시작 상태입니다. 아직 이륙하지 않았으므로 IDLE에서 대기합니다.
                mode = MODE_IDLE
                safe_set_downvision(tello, False)
                is_downvision_active = False
            else:
                print("[INFO] Already landed")

# 예외 발생, ESC 종료 등 어떤 방식으로 루프가 끝나도 장비/스레드/파일을 정리하기 위한 블록입니다.
finally:
    # 중요: stop_evt를 먼저 켜면 업로더 스레드가 먼저 종료될 수 있다.
    # 그래서 먼저 녹화 파일을 닫고 upload_q에 넣은 뒤, 종료 신호를 보낸다.
    try:
        disarm_front_record()
    except Exception as e:
        print("[WARN] disarm_front_record failed:", e)

    # 백그라운드 스레드들에게 종료 신호를 보냅니다.
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
        # OpenCV로 띄운 모든 창을 닫습니다.
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
            # Tello 영상 스트림을 끕니다.
            tello.streamoff()
    except Exception:
        pass

    try:
        with tello_lock:
            # Tello SDK 세션을 종료하고 내부 리소스를 정리합니다.
            tello.end()
    except Exception:
        pass
