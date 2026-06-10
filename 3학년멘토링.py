import sys
import math

try:
    import pkg_resources
except ImportError:
    import pip._vendor.pkg_resources as pkg_resources
    sys.modules['pkg_resources'] = pkg_resources

from vpython import *

# =====================================================================
# 1. 실물 규격 입력 및 물리 파라미터 설정 (cm 단위 입력 -> m 단위 변환)
# =====================================================================
g = 9.81              

# [수정] 입력 데이터 분석 및 cm 기반 변수 선언
LOOP_DIAMETER_CM = 9.0            # 원형 레일 지름: 90mm = 9.0cm
CYCLOID_HEIGHT_CM = 25.6          # 최고점-최저점 높이 차이: 256mm = 25.6cm
ELEVATION_Y_CM = 8.0              # 레일-바닥 이격 거리: 8cm

# 물리 엔진 적용을 위한 SI 단위계(m) 자동 변환 연산
R_loop = (LOOP_DIAMETER_CM / 2.0) / 100.0      # 루프 반지름 (0.045 m)
R_cycloid = (CYCLOID_HEIGHT_CM / 2.0) / 100.0  # 사이클로이드 원 반지름 (0.128 m)
ELEVATION_Y = ELEVATION_Y_CM / 100.0           # 하단 여백 고도 오프셋 (0.08 m)

# 쇠구슬(Steel Ball) 규격 및 역학 설정 (cm 기반 설계)
D_ball_cm = 1.5                   # 구슬 지름 1.5cm
R_ball = (D_ball_cm / 2.0) / 100.0 
rho_steel = 7850                  # 강철 밀도 (7850 kg/m³)
V_ball = (4/3) * math.pi * R_ball**3
m = rho_steel * V_ball  

# 구체의 유효 관성 질량을 반영한 회전 인자 (M_eff = 7/5 * m)
ROLL_FACTOR = 5.0 / 7.0 

# 레일 정밀 치수 및 구름 마찰 계수 (cm 기반 설계)
w_rail_cm = 1.2
w_rail = w_rail_cm / 100.0        
mu_r   = 0.0015       

# 공기 저항 계수 및 유체역학 파라미터
C_d     = 0.47        
rho_air = 1.225       
A       = math.pi * R_ball**2 
k_drag  = 0.5 * rho_air * C_d * A / m 

# 변환된 단위를 기반으로 한 궤도 호의 길이(s) 연산
L_drop = 4 * R_cycloid            # 사이클로이드 드롭 구간 총 길이
L_loop = 2 * math.pi * R_loop     # 루프 구간 총 길이         
L_exit_cm = 30.0                  # 탈출 직진 코스 30cm
L_exit = L_exit_cm / 100.0        
S_MAX = L_loop + L_exit 

# =====================================================================
# 2. 궤도 기하학 수치 해석 함수
# =====================================================================
def track_info(s):
    if s < 0:
        val = -s / L_drop
        val = max(0.0, min(1.0, val)) 
        
        alpha = math.asin(s / L_drop) 
        sin_half = math.sqrt(1.0 - val**2)
        
        if sin_half > 1e-4:
            kappa = 1.0 / (L_drop * sin_half)
        else:
            kappa = 0.0 
        return alpha, kappa
    elif s <= L_loop:
        return s / R_loop, 1.0 / R_loop
    else:
        return 0.0, 0.0

def get_pos(s):
    if s < 0:
        val = -s / L_drop
        val = max(0.0, min(1.0, val))
        theta = 2 * math.acos(val)
        
        x = R_cycloid * (theta - math.pi - math.sin(theta))
        y = R_cycloid * (1 + math.cos(theta)) + ELEVATION_Y  
        z = 0.0
        return vector(x, y, z)
    elif s <= L_loop:
        x = R_loop * math.sin(s / R_loop)
        y = R_loop * (1 - math.cos(s / R_loop)) + ELEVATION_Y 
        z = 0.04 * (s / L_loop) 
        return vector(x, y, z)
    else:
        s_exit = s - L_loop
        x = s_exit
        y = 0 + ELEVATION_Y                                  
        z = 0.04 
        return vector(x, y, z)

# =====================================================================
# 3. VPython 3D 그래픽스 및 시각화 환경 렌더링
# =====================================================================
scene = canvas(title='<b>3D Realistic Coaster (Custom Dimension Spline Model)</b>',
               width=900, height=550,
               center=vector(0.05, R_loop + ELEVATION_Y, 0), background=vector(0.1, 0.1, 0.12))
scene.forward = vector(0.3, -0.3, -1) 
scene.ambient = color.gray(0.4) 

# 궤도 스플라인 포인트 생성
pts_front, pts_back = [], []
for s_val in arange(-L_drop, S_MAX + 0.01, 0.005):
    pos = get_pos(s_val)
    pts_front.append(pos + vector(0, -R_ball, w_rail/2))
    pts_back.append(pos + vector(0, -R_ball, -w_rail/2))

# 가이드 레일 시각화
rail_color = vector(0.5, 0.5, 0.55)
rail_front_curve = curve(pos=pts_front, radius=0.0015, color=rail_color, shininess=0.9)
rail_back_curve  = curve(pos=pts_back, radius=0.0015, color=rail_color, shininess=0.9)

# 트랙 침목(Cross-ties) 구조물 배치
for i in range(0, len(pts_front), 8): 
    cylinder(pos=pts_back[i], axis=pts_front[i]-pts_back[i], 
             radius=0.0008, color=vector(0.25, 0.25, 0.25))

# 스타팅 플랫폼 및 지지 타워
start_pos = get_pos(-L_drop)
box(pos=start_pos + vector(-0.02, 0, 0), size=vector(0.04, 0.01, 0.04), color=vector(0.4, 0.4, 0.4), shininess=0.8)
cylinder(pos=vector(start_pos.x, ELEVATION_Y/2, 0), axis=vector(0, start_pos.y - ELEVATION_Y/2, 0), radius=0.003, color=color.gray(0.3))

# 쇠구슬 오브젝트 생성 (은빛 금속 질감, 불꽃 같은 주황색 트레일)
ball = sphere(pos=start_pos, radius=R_ball, 
              color=vector(0.8, 0.8, 0.85), opacity=1.0, shininess=1.0, 
              make_trail=True, trail_color=color.orange, trail_radius=0.0008)

# 기준 지면(Ground Plane) 생성 (레일 시스템과 정확히 8cm 단차 유지)
ground = box(pos=vector(0.1, -R_ball - 0.01 + ELEVATION_Y, 0.02), size=vector(1.5, 0.01, 0.5), color=vector(0.2, 0.2, 0.2))

# 정보 디스플레이 패널
info_label = label(pos=vector(0, 2.5*R_loop + ELEVATION_Y, 0), text='', height=12, border=4,
                   font='monospace', color=color.white, background=color.black, opacity=0.8)

# 실시간 분석 그래프 윈도우 생성 (0.01초 주기 제어)
graph_win = graph(title="Steel Ball Velocity vs Time (Sampled at 0.01s)", xtitle="Time (s)", ytitle="Velocity (m/s)", width=900, height=250)
v_curve = gcurve(color=color.orange, label="Instantaneous Velocity", graph=graph_win)

# =====================================================================
# 4. 물리 동역학 연산 (Runge-Kutta 4th Order) 및 상태 관리 시스템
# =====================================================================
def f_v(s, v):
    theta, kappa = track_info(s)
    N = m * (v**2 * kappa + g * math.cos(theta))
    f_roll = mu_r * max(N, 0.0)
    sign_v = (1 if v > 0 else -1) if v != 0 else 0
    a_raw = -g * math.sin(theta) - (f_roll / m) * sign_v - k_drag * v**2 * sign_v
    return ROLL_FACTOR * a_raw

def update_rk4(s, v, dt):
    k1_s = v
    k1_v = f_v(s, v)
    k2_s = v + k1_s*dt/2
    k2_v = f_v(s + k1_s*dt/2, v + k1_v*dt/2)
    k3_s = v + k2_s*dt/2
    k3_v = f_v(s + k2_s*dt/2, v + k2_v*dt/2)
    k4_s = v + k3_v*dt
    k4_v = f_v(s + k3_s*dt, v + k3_v*dt)

    new_s = s + (dt / 6) * (k1_s + 2*k2_s + 2*k3_s + k4_s)
    new_v = v + (dt / 6) * (k1_v + 2*k2_v + 2*k3_v + k4_v)
    return new_s, new_v

# 초기 제어 변수 선언
s, v, t, N_now = -L_drop, 0.0, 0.0, 0.0
dt = 0.001                       # 초정밀 물리 연산 타임스텝 (1ms)
step_count = 0                   # 이산 데이터 플로팅 카운터
sim_state = 'running' 

def on_mouse_click(_):
    global sim_state, s, v, t, N_now, step_count
    if sim_state == 'running':
        sim_state = 'paused'
        info_label.text += '\n\n[일시정지 중 - 다시 클릭하여 재생]'
    elif sim_state == 'paused':
        sim_state = 'running'
    elif sim_state in ['finished', 'failed']:
        s, v, t, N_now = -L_drop, 0.0, 0.0, 0.0
        step_count = 0           
        v_curve.data = []      
        ball.clear_trail()     
        sim_state = 'running'

scene.bind('click', on_mouse_click)
scene.append_to_caption('\n\n마우스 클릭 시 일시정지 되며, 동작 종료 후 클릭하면 시스템이 초기화됩니다.')

# 기준선 대비 초기 위치 역학적 에너지 보존 검증용 상수 연산
initial_pos = get_pos(-L_drop)
E_initial = m * g * (initial_pos.y - ELEVATION_Y) 

# =====================================================================
# 5. 데시메이션(Decimation) 필터 기반 메인 루프
# =====================================================================
while True:
    rate(300) 
    
    if sim_state != 'running':
        continue

    # 초정밀 물리 해석 코어 업데이트 (dt = 0.001s)
    s, v = update_rk4(s, v, dt)
    t += dt
    step_count += 1              

    theta, kappa = track_info(s)
    N_now = m * (v**2 * kappa + g * math.cos(theta))

    ball.pos = get_pos(s)
    
    # 10스텝(0.001초 * 10 = 0.01초)마다 그래프에 데이터 적재
    if step_count % 10 == 0:
        v_curve.plot(t, v)

    # 역학적 에너지 보존 법칙 실시간 수치 분석 시스템
    E_k = 0.5 * (m / ROLL_FACTOR) * v**2                  
    E_p = m * g * (ball.pos.y - ELEVATION_Y)               
    E_total = E_k + E_p
    energy_loss_pct = ((E_initial - E_total) / E_initial) * 100 if E_initial > 0 else 0

    info_label.text = (f't = {t:.3f} s\n'
                       f'Pos = {s:.3f} m\n'
                       f'v = {v:.3f} m/s\n'
                       f'N = {N_now:.4f} N\n'
                       f'Energy Loss = {energy_loss_pct:.1f}%')

    # 이탈 조건 검증 로직
    if kappa > 0 and N_now < 0 and math.cos(theta) < 0:
        sim_state = 'failed'
        info_label.text += '\n\n[궤도 이탈! 임계 수직항력 도달 실패]'

    # 완주 조건 검증 로직
    if s >= S_MAX:
        sim_state = 'finished'
        info_label.text += '\n\n[완주 성공! 실시간 연산 데이터 기록 완료]'