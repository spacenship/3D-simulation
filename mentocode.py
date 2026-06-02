import sys

try:
    import pkg_resources
except ImportError:
    import pip._vendor.pkg_resources as pkg_resources
    sys.modules['pkg_resources'] = pkg_resources

from vpython import *
import numpy as np

# =========================
# 화면 설정
# =========================

scene = canvas(
    title="Cycloid Roller Coaster",
    width=1200,
    height=700,
    background=color.black
)

# =========================
# 사이클로이드 레일 생성
# =========================

R = 2
points = []

# 사이클로이드 3개 연결
for n in range(3):

    for t in np.linspace(0, 2*pi, 300):

        x = R*(t - sin(t)) + n*(2*pi*R)
        y = -R*(1 - cos(t))

        points.append(vector(x, y, 0))

# 레일 그리기
rail = curve(
    pos=points,
    radius=0.08,
    color=color.white
)

# =========================
# 구슬 생성
# =========================

ball_radius = 0.25

ball = sphere(
    pos=points[0],
    radius=ball_radius,
    color=color.red,
    make_trail=True,
    retain=500
)

# 회전 확인용 막대
marker = cylinder(
    pos=ball.pos,
    axis=vector(ball_radius, 0, 0),
    radius=0.03,
    color=color.yellow
)

# =========================
# 애니메이션
# =========================

i = 0

while True:

    rate(120)

    current = points[i]
    nxt = points[(i + 1) % len(points)]

    # 구슬 이동
    ball.pos = current

    # 이동 거리
    ds = mag(nxt - current)

    # 굴림 조건
    angle = ds / ball_radius

    # 접선 방향
    tangent = norm(nxt - current)

    # 회전축
    axis = cross(tangent, vector(0, 0, 1))

    # 회전 표시 막대
    marker.pos = ball.pos

    marker.rotate(
        angle=angle,
        axis=axis,
        origin=ball.pos
    )

    i += 1

    if i >= len(points):
        i = 0