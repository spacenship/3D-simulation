from vpython import *
import math

# =====================================================================
# 1. 물리 파라미터 및 재질 설정
# =====================================================================
g = 9.81              # 중력 가속도 (m/s²)
R_loop = 0.1          # 루프 반지름 (m)

# 쇠공 (Steel Solid Ball) 설정
D_ball = 0.015        # 직경 15mm
R_ball = D_ball / 2   # 반지름 7.5mm
rho_steel = 7850      # 강철 밀도 (kg/m³)
V_ball = (4/3) * math.pi * R_ball**3
m = rho_steel * V_ball  # ≈ 0.01384 kg

# 구름 관성 보정: 속이 찬 구체 I = (2/5)·m·R²
# 구름 구속 조건 적용 시, 유효 가속도 = (5/7)·(접선 합력 / m)
ROLL_FACTOR = 5.0 / 7.0

# PVC 레일 설정
w_rail = 0.01         # 레일 간격 10mm
mu_r   = 0.002        # 쇠공-PVC 구름 마찰 계수

# 공기 저항 설정
C_d     = 0.47                          # 구의 항력 계수
rho_air = 1.225                         # 공기 밀도 (kg/m³)
A       = math.pi * R_ball**2           # 단면적 (m²)
k_drag  = 0.5 * rho_air * C_d * A / m  # 단위 질량당 공기저항 계수 (1/m)

# =====================================================================
# 2. VPython 3D 시각화 환경 구축
# =====================================================================
scene = canvas(title='<b>RK4 3D Roller Coaster (Steel on PVC)</b>',
               width=800, height=500,
               center=vector(0, R_loop, 0),
               background=color.black)
scene.forward = vector(0, -0.2, -1)  # 카메라 방향: 루프 정면에서 약간 내려다봄

# 레일 (PVC 재질, 앞/뒤 두 링)
rail_front = ring(pos=vector(0, R_loop,  w_rail/2), axis=vector(0, 0, 1),
                  radius=R_loop, thickness=0.003, color=vector(0.9, 0.9, 0.9))
rail_back  = ring(pos=vector(0, R_loop, -w_rail/2), axis=vector(0, 0, 1),
                  radius=R_loop, thickness=0.003, color=vector(0.9, 0.9, 0.9))

# 쇠공
ball = sphere(pos=vector(0, 0, 0), radius=R_ball,
              color=vector(0.7, 0.75, 0.8), shininess=1.0,
              make_trail=True, trail_color=color.red, trail_radius=0.001)

# 바닥 — color.darkgray 는 VPython 상수에 없으므로 RGB 벡터로 직접 지정
ground = box(pos=vector(0, -R_ball, 0), size=vector(1.5, 0.01, 0.5),
             color=vector(0.3, 0.3, 0.3))

# 실시간 상태 표시 레이블
info_label = label(pos=vector(0, 2*R_loop + 0.07, 0),
                   text='', height=12, border=4,
                   font='monospace', color=color.white,
                   background=color.black, opacity=0.8)

# 속도-시간 그래프
graph_win = graph(title="Velocity vs Time",
                  xtitle="Time (s)", ytitle="Velocity (m/s)",
                  width=800, height=250)
v_curve = gcurve(color=color.cyan, graph=graph_win)

# =====================================================================
# 3. 비선형 운동 방정식 및 RK4 알고리즘
# =====================================================================
def f_theta(theta, v):
    """각도 변화율 dθ/dt = v / R"""
    return v / R_loop

def f_v(theta, v):
    """
    접선 방향 가속도 dv/dt.

    수직항력:  N = m·(v²/R + g·cosθ)
      - θ=0 (최하단): N = m·(v²/R + g)  → 항상 양수
      - θ=π (최상단): N = m·(v²/R - g)  → v가 작으면 음수(이탈)

    구름 마찰: F_roll = μ_r · N  (N > 0 인 경우만 작용)

    구름 관성 보정: 속이 찬 구체가 미끄러지지 않고 굴러갈 때
      유효 방정식 → (7/5)·m·a = F_접선  →  a = (5/7)·(F/m)
    """
    N = m * (v**2 / R_loop + g * math.cos(theta))
    f_roll = mu_r * max(N, 0.0)          # N < 0이면 마찰 없음
    sign_v = (1 if v > 0 else -1) if v != 0 else 0
    a_raw = (-g * math.sin(theta)
             - (f_roll / m) * sign_v
             - k_drag * v**2 * sign_v)
    return ROLL_FACTOR * a_raw

def update_rk4(theta, v, dt):
    k1_t = f_theta(theta,              v             )
    k1_v = f_v    (theta,              v             )
    k2_t = f_theta(theta + k1_t*dt/2, v + k1_v*dt/2)
    k2_v = f_v    (theta + k1_t*dt/2, v + k1_v*dt/2)
    k3_t = f_theta(theta + k2_t*dt/2, v + k2_v*dt/2)
    k3_v = f_v    (theta + k2_t*dt/2, v + k2_v*dt/2)
    k4_t = f_theta(theta + k3_t*dt,   v + k3_v*dt  )
    k4_v = f_v    (theta + k3_t*dt,   v + k3_v*dt  )

    new_theta = theta + (dt / 6) * (k1_t + 2*k2_t + 2*k3_t + k4_t)
    new_v     = v     + (dt / 6) * (k1_v + 2*k2_v + 2*k3_v + k4_v)
    return new_theta, new_v

# =====================================================================
# 4. 메인 시뮬레이션 루프
# =====================================================================
theta = 0.0   # 시작: 루프 최하단 (0 rad)
v     = 4.0   # 초기 속도 (m/s) — 루프 완주 최소 속도 √(5gR) ≈ 3.84 m/s 이상으로 설정
dt    = 0.001 # RK4 타임 스텝 (s)
t     = 0.0   # 시뮬레이션 시간 (s)

running = True
def toggle_pause(_):
    global running
    running = not running
scene.bind('click', toggle_pause)
scene.append_to_caption('\n\n화면을 클릭하면 시뮬레이션이 일시정지/재생 됩니다.')

while t < 20:
    rate(40)
    if not running:
        continue

    # RK4로 다음 상태 계산
    theta, v = update_rk4(theta, v, dt)
    t += dt

    # 이탈 조건 검사 (업데이트 후 상태 기준)
    N_now = m * (v**2 / R_loop + g * math.cos(theta))
    if N_now < 0 and math.cos(theta) < 0:
        print(f'[{t:.2f}s] 속도 부족 — 레일에서 이탈!')
        break

    # 3D 위치 업데이트 (최하단 = 원점)
    x = R_loop * math.sin(theta)
    y = R_loop * (1 - math.cos(theta))
    ball.pos = vector(x, y, 0)

    # 상태 레이블 업데이트
    angle_deg = math.degrees(theta % (2 * math.pi))
    info_label.text = (f't = {t:.2f} s\n'
                       f'θ = {angle_deg:.1f}°\n'
                       f'v = {v:.3f} m/s\n'
                       f'N = {N_now:.4f} N')

    v_curve.plot(t, v)
