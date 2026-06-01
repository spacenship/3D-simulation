from vpython import *
import math

# =====================================================================
# 1. 물리 파라미터
# =====================================================================
g      = 9.81
R_loop = 0.1       # 루프 반지름 (m)

D_ball = 0.015;  R_ball = D_ball / 2
m      = 7850 * (4/3) * math.pi * R_ball**3   # ≈ 0.01384 kg

# 구름 관성 보정: 속이 찬 구체 I = 2/5·m·R²  →  유효 가속도 = (5/7)·(F/m)
ROLL_FACTOR = 5.0 / 7.0

w_rail = 0.01
mu_r   = 0.002     # 쇠공-PVC 구름 마찰 계수
k_drag = 0.5 * 1.225 * 0.47 * math.pi * R_ball**2 / m  # 공기저항 계수 (1/m)

# 경사로 파라미터
alpha    = math.radians(40)          # 경사각 40°
H_start  = 0.35                      # 시작 높이 (m) — 루프 완주 최소 ≈ 0.27 m
L_ramp   = H_start / math.sin(alpha) # 경사면 따른 길이
L_ramp_h = H_start / math.tan(alpha) # 수평 투영 길이

# 트랙 주요 지점
P_left  = vector(-L_ramp_h, H_start, 0)  # 왼쪽 경사로 꼭대기
P_bot   = vector(0, 0, 0)                # 경사로-루프 접점 (루프 최하단)
P_right = vector( L_ramp_h, H_start, 0)  # 오른쪽 경사로 꼭대기

# =====================================================================
# 2. 시각화
# =====================================================================
scene = canvas(title='<b>Roller Coaster Simulation (RK4)</b>',
               width=900, height=500,
               center=vector(0, H_start * 0.45, 0),
               background=color.black)
scene.forward = vector(0, -0.15, -1)

# 레일 그리기 헬퍼: 두 점 사이에 앞/뒤 레일 실린더 쌍 생성
def draw_rail(p1, p2):
    for zz in [w_rail/2, -w_rail/2]:
        cylinder(pos=p1 + vector(0, 0, zz), axis=p2 - p1,
                 radius=0.0015, color=vector(0.9, 0.9, 0.9))

draw_rail(P_left, P_bot)    # 왼쪽 경사로
draw_rail(P_bot,  P_right)  # 오른쪽 경사로

for zz in [w_rail/2, -w_rail/2]:
    ring(pos=vector(0, R_loop, zz), axis=vector(0, 0, 1),
         radius=R_loop, thickness=0.002, color=vector(0.9, 0.9, 0.9))

box(pos=vector(0, -R_ball, 0),
    size=vector(2*L_ramp_h + 0.1, 0.01, 0.06),
    color=vector(0.3, 0.3, 0.3))

ball = sphere(pos=P_left, radius=R_ball,
              color=vector(0.7, 0.75, 0.8), shininess=1.0,
              make_trail=True, trail_color=color.red, trail_radius=0.001)

info_label = label(pos=vector(0, H_start + 0.06, 0), text='',
                   height=12, border=4, font='monospace',
                   color=color.white, background=color.black, opacity=0.8)

graph_win = graph(title="Velocity vs Time",
                  xtitle="Time (s)", ytitle="Velocity (m/s)",
                  width=900, height=250)
v_curve = gcurve(color=color.cyan, graph=graph_win)

# =====================================================================
# 3. 운동 방정식
# =====================================================================
def accel_ramp(v, going_down):
    """
    경사로 위 접선 가속도 (구름 관성 반영).
    going_down=True : 내려오는 경사 (중력이 전진 방향)
    going_down=False: 올라가는 경사 (중력이 전진 반대 방향)
    """
    N_ramp = m * g * math.cos(alpha)
    f_roll = mu_r * N_ramp
    sv     = (1 if v > 0 else -1) if v != 0 else 0
    grav   = g * math.sin(alpha) * (1 if going_down else -1)
    return ROLL_FACTOR * (grav - (f_roll / m) * sv - k_drag * v**2 * sv)

def accel_loop(theta, v):
    """루프 위 접선 가속도 (구름 관성 반영)."""
    N = m * (v**2 / R_loop + g * math.cos(theta))
    f_roll = mu_r * max(N, 0.0)
    sv = (1 if v > 0 else -1) if v != 0 else 0
    return ROLL_FACTOR * (-g * math.sin(theta) - (f_roll / m) * sv - k_drag * v**2 * sv)

def rk4_ramp(s, v, dt, going_down):
    a = lambda vv: accel_ramp(vv, going_down)
    k1s, k1v = v,            a(v)
    k2s, k2v = v + k1v*dt/2, a(v + k1v*dt/2)
    k3s, k3v = v + k2v*dt/2, a(v + k2v*dt/2)
    k4s, k4v = v + k3v*dt,   a(v + k3v*dt)
    return (s + dt/6 * (k1s + 2*k2s + 2*k3s + k4s),
            v + dt/6 * (k1v + 2*k2v + 2*k3v + k4v))

def rk4_loop(theta, v, dt):
    ft = lambda _, vv: vv / R_loop
    fv = accel_loop
    k1t, k1v = ft(theta, v),                         fv(theta, v)
    k2t, k2v = ft(theta+k1t*dt/2, v+k1v*dt/2),       fv(theta+k1t*dt/2, v+k1v*dt/2)
    k3t, k3v = ft(theta+k2t*dt/2, v+k2v*dt/2),       fv(theta+k2t*dt/2, v+k2v*dt/2)
    k4t, k4v = ft(theta+k3t*dt,   v+k3v*dt),         fv(theta+k3t*dt,   v+k3v*dt)
    return (theta + dt/6 * (k1t + 2*k2t + 2*k3t + k4t),
            v     + dt/6 * (k1v + 2*k2v + 2*k3v + k4v))

# =====================================================================
# 4. 메인 시뮬레이션 루프
# =====================================================================
RAMP_L, LOOP, RAMP_R = 0, 1, 2

phase        = RAMP_L
s            = 0.0   # 경사로: 시작점에서 경사면 따른 거리 (m)
theta        = 0.0   # 루프: 각도 (rad)
v            = 0.0   # 초기 속도: 꼭대기에서 정지 출발
dt           = 0.001
t            = 0.0

running = True
def toggle_pause(_):
    global running
    running = not running
scene.bind('click', toggle_pause)
scene.append_to_caption('\n\n클릭: 일시정지 / 재생')

PH = {RAMP_L: '왼쪽 경사로', LOOP: '루프', RAMP_R: '오른쪽 경사로'}

while t < 30:
    rate(40)
    if not running:
        continue

    # ── 왼쪽 경사로 ──────────────────────────────────────
    if phase == RAMP_L:
        s, v = rk4_ramp(s, v, dt, going_down=True)
        t += dt
        ball.pos = P_left + min(s / L_ramp, 1.0) * (P_bot - P_left)
        if s >= L_ramp:
            phase, theta = LOOP, 0.0

    # ── 루프 ─────────────────────────────────────────────
    elif phase == LOOP:
        theta, v = rk4_loop(theta, v, dt)
        t += dt
        ball.pos = vector(R_loop * math.sin(theta),
                          R_loop * (1 - math.cos(theta)), 0)
        N_now = m * (v**2 / R_loop + g * math.cos(theta))
        if N_now < 0 and math.cos(theta) < 0:
            print(f'[{t:.2f}s] 속도 부족 — 레일 이탈!')
            break
        if theta >= 2 * math.pi:
            phase, s = RAMP_R, 0.0

    # ── 오른쪽 경사로 ─────────────────────────────────────
    elif phase == RAMP_R:
        s, v = rk4_ramp(s, v, dt, going_down=False)
        t += dt
        ball.pos = P_bot + min(s / L_ramp, 1.0) * (P_right - P_bot)
        if v <= 0:
            print(f'[{t:.2f}s] 오른쪽 경사로에서 정지.')
            break
        if s >= L_ramp:
            print(f'[{t:.2f}s] 오른쪽 경사로 끝 도달.')
            break

    info_label.text = (f't = {t:.2f} s  [{PH[phase]}]\n'
                       f'v = {v:.3f} m/s')
    v_curve.plot(t, v)
