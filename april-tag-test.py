import pygame
import cv2
import sys
import math
from pupil_apriltags import Detector

# ============================
# 1) Pygame 초기 설정
# ============================
pygame.init()
WINDOW_WIDTH, WINDOW_HEIGHT = 1000, 600
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Real-time Apriltag Parking")
clock = pygame.time.Clock()

# ============================
# 2) OpenCV 카메라 초기화
# ============================
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("❌ 카메라 열기 실패")
    sys.exit()

# OpenCV 창 크기를 고정(300×200)
WEBCAM_WIDTH, WEBCAM_HEIGHT = 300, 200
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WEBCAM_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, WEBCAM_HEIGHT)

# ============================
# 3) Apriltag 탐지기 설정
# ============================
at_detector = Detector(families='tag36h11')

# ============================
# 4) 색상 및 전역 변수 정의
# ============================
WHITE      = (255, 255, 255)    # Pygame 배경색
TRACK_C    = (255, 105, 180)    # 추적 경로 색상
PLANNING_C = (173, 216, 230)    # 계획 경로 색상
CAR_C      = (0, 0, 0)          # 차량 차체 색상
BOX_C      = (0, 0, 200)        # 주차 박스 테두리 색상
TAG_COLOR  = (0, 200, 0)        # Pygame 화면 태그 라벨 색상
BUTTON_BG  = (200, 0, 0)        # 리셋 버튼 배경색
BUTTON_FG  = (255, 255, 255)    # 리셋 버튼 글자색

# ============================
# 4-1) 차량 초기 상태 설정
# ============================
start_x, start_y = 100, 500     # 초기 차량 위치
car_x, car_y = start_x, start_y
car_heading = math.radians(0)   # 차량 시작 방향(라디안)
SPEED = 2                       # 차량 속도(픽셀/프레임)
arrived = False                 # 주차 완료 여부 플래그
driving = False                 # 주차 경로 따라 이동 중 여부
last_tag_id = None              # 마지막으로 인식된 태그 ID 저장

planning_path = []              # 베지어 곡선에 따른 주차 경로 리스트
tracking_path = []              # 실제 차량 이동 궤적 리스트
path_idx = 0                    # 경로 인덱스

# ============================
# 5) 주차 영역(paking_boxes) 설정
# ============================
parking_boxes = {}
spacing = 140   # 주차 칸 간 가로 간격(픽셀)
base_x = 300    # 첫 번째 주차 칸 중앙 x 좌표
base_y = 200    # 주차 칸 중앙 y 좌표 (고정)

for i in range(5):      # i = 0, 1, 2, 3, 4
    cx = base_x + i * spacing
    parking_boxes[i] = (cx, base_y)

# ============================
# 6) 3차 베지어 곡선 함수 정의
# ============================
def bezier3(a, b, c, d, steps=200):
    """
    네 개 제어점(a, b, c, d)에 대해 3차 베지어 곡선을 계산하여
    (steps+1)개의 궤적 점을 리스트로 반환합니다.
    """
    pts = []
    for i in range(steps + 1):
        t = i / steps
        u = 1 - t
        x = (
            u**3 * a[0]
            + 3 * u*u * t * b[0]
            + 3 * u * t*t * c[0]
            + t**3 * d[0]
        )
        y = (
            u**3 * a[1]
            + 3 * u*u * t * b[1]
            + 3 * u * t*t * c[1]
            + t**3 * d[1]
        )
        pts.append((x, y))
    return pts

# ============================
# 7) 차량 렌더링 함수 정의
# ============================
def draw_car(x, y, heading):
    """
    (x, y) 위치에 heading 방향으로 회전된 차량 이미지를 Pygame 화면에 그립니다.
    """
    w, h = 70, 45
    surf = pygame.Surface((w, h), pygame.SRCALPHA)
    pygame.draw.rect(surf, CAR_C, (0, 0, w, h))
    pygame.draw.polygon(
        surf, (255, 255, 0),
        [(w, h // 2), (w - 10, h // 2 - 5), (w - 10, h // 2 + 5)]
    )
    rot = pygame.transform.rotate(surf, -math.degrees(heading))
    rect = rot.get_rect(center=(x, y))
    screen.blit(rot, rect)

# ============================
# 8) 리셋 버튼(Rect) 정의
# ============================
# Pygame 창 왼쪽 상단에 위치시키며, 클릭 시 차량 초기화
button_font = pygame.font.SysFont(None, 24)
reset_rect = pygame.Rect(10, 10, 80, 30)  # x=10, y=10, 너비=80, 높이=30

# ============================
# 9) 메인 루프
# ============================
while True:
    # ----------------------------
    # 9-1) OpenCV로 웹캠 프레임 읽기 및 Apriltag 인식
    # ----------------------------
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    results = at_detector.detect(gray)

    # ----------------------------
    # 9-2) OpenCV 창으로 웹캠 영상 띄우기 (태그 ID 텍스트 오버레이 포함)
    # ----------------------------
    if results:
        for det in results:
            tag_id = det.tag_id
            cx, cy = det.center
            cv2.putText(
                frame,
                f"Tag {tag_id}",
                (int(cx) - 20, int(cy) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA
            )
    cv2.imshow("Webcam (300x200)", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # ----------------------------
    # 9-3) 태그 인식 시 주차 경로 생성 (Pygame 로직)
    # ----------------------------
    if results:
        tag_id = results[0].tag_id
        if tag_id != last_tag_id and tag_id in parking_boxes:
            print(f"🧭 Tag {tag_id} 인식 → 주차 경로 생성")
            goal = parking_boxes[tag_id]
            goal_orientation = math.radians(-90)
            d1, d2 = 150, 150

            p0 = (car_x, car_y)
            p1 = (
                car_x + d1 * math.cos(car_heading),
                car_y + d1 * math.sin(car_heading)
            )
            p3 = goal
            p2 = (
                p3[0] - d2 * math.cos(goal_orientation),
                p3[1] - d2 * math.sin(goal_orientation)
            )

            planning_path = bezier3(p0, p1, p2, p3)
            tracking_path = []
            path_idx = 0
            driving = True
            arrived = False
            last_tag_id = tag_id

    # ----------------------------
    # 9-4) Pygame 이벤트 처리
    # ----------------------------
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            cap.release()
            cv2.destroyAllWindows()
            sys.exit()

        # 리셋 버튼 클릭 처리
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:  # 왼쪽 클릭
            mouse_pos = event.pos
            if reset_rect.collidepoint(mouse_pos):
                # 리셋: 차량 위치와 상태 모두 초기화
                car_x, car_y = start_x, start_y
                car_heading = math.radians(0)
                arrived = False
                driving = False
                last_tag_id = None
                planning_path = []
                tracking_path = []
                path_idx = 0
                print("🔄 차량을 초기 위치로 리셋했습니다.")

    # ----------------------------
    # 9-5) Pygame 화면 초기화
    # ----------------------------
    screen.fill(WHITE)

    # ----------------------------
    # 9-6) 리셋 버튼 그리기
    # ----------------------------
    pygame.draw.rect(screen, BUTTON_BG, reset_rect)  # 버튼 배경
    text_surf = button_font.render("Reset", True, BUTTON_FG)
    text_rect = text_surf.get_rect(center=reset_rect.center)
    screen.blit(text_surf, text_rect)

    # ----------------------------
    # 9-7) 주차 구역(박스) 표시
    # ----------------------------
    font = pygame.font.SysFont(None, 24)
    for t_id, (cx, cy) in parking_boxes.items():
        rect = pygame.Rect(cx - 40, cy - 40, 80, 80)
        pygame.draw.rect(screen, BOX_C, rect, 3)
        pygame.draw.rect(screen, (0, 0, 0), rect, 1)
        label = font.render(f"Tag {t_id}", True, TAG_COLOR)
        screen.blit(label, (cx - label.get_width() // 2, cy + 45))

    # ----------------------------
    # 9-8) 계획 경로 및 실제 주행 궤적 시각화
    # ----------------------------
    if len(planning_path) > 1:
        pygame.draw.lines(screen, PLANNING_C, False, planning_path, 3)
    if len(tracking_path) > 1:
        pygame.draw.lines(screen, TRACK_C, False, tracking_path, 2)

    # ----------------------------
    # 9-9) 차량 이동 제어
    # ----------------------------
    if driving and not arrived:
        tx, ty = planning_path[path_idx]
        dx, dy = tx - car_x, ty - car_y
        dist = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)

        if dist < 2:
            path_idx += 1
            if path_idx >= len(planning_path):
                arrived = True
                driving = False
        else:
            car_x += SPEED * math.cos(heading)
            car_y += SPEED * math.sin(heading)
        car_heading = heading
        tracking_path.append((car_x, car_y))

    # ----------------------------
    # 9-10) 차량 렌더링
    # ----------------------------
    draw_car(car_x, car_y, car_heading)

    # ----------------------------
    # 9-11) Pygame 화면 업데이트 및 프레임 제어
    # ----------------------------
    pygame.display.flip()
    clock.tick(60)

# ============================
# 프로그램 종료 시 리소스 해제
# ============================
cap.release()
cv2.destroyAllWindows()
pygame.quit()
sys.exit()
