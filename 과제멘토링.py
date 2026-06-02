import sys
import math

try:
    import pkg_resources
except ImportError:
    import pip._vendor.pkg_resources as pkg_resources
    sys.modules['pkg_resources'] = pkg_resources

from vpython import *

# =====================================================================
# 1. 물리 파라미터 및 재질 설정
# =====================================================================
g = 9.81              
R_loop = 0.1          # 루프 반지름 (m)

# 유리 구슬(Glass Ball) 설정
D_ball = 0.015        
R_ball = D_ball / 2 
rho_glass = 2500      
V_ball = (4/3) * math.pi * R_ball**3
m = rho_glass * V_ball  

ROLL_FACTOR = 5.0 / 7.0 

# 레일 및 마찰 설정
w_rail = 0.012        
mu_r   = 0.0015       

# 공기 저항 설정
C_d     = 0.47        
rho_air = 1.225       
A       = math.pi * R_ball**2 
k_drag  = 0.5 * rho_air * C_d * A / m 

# [NEW] 사이클로이드 궤도 파라미터 
R_cycloid = 0.15             # 사이클로이드 곡선을 만드는 가상의 원 반지름 (m)
L_drop = 4 * R_cycloid       # 사이클로이드 곡선 전체의 호의 길이 (m)
L_loop = 2 * math.pi * R_loop              
L_exit = 0.3          
S_MAX = L_loop + L_exit 

# =====================================================================
# 2. 연속 궤도 기하학 함수 (사이클로이드 드롭 -> 루프 -> 직진)
# =====================================================================
def track_info(s):
    if s < 0:
        # [NEW] 사이클로이드 궤도의 각도(alpha)와 곡률(kappa) 계산
        val = -s / L_drop
        val = max(0.0, min(1.0, val)) # 수학적 도메인 에러 방지
        
        alpha = math.asin(s / L_drop) 
        
        sin_half = math.sqrt(1.0 - val**2)
        # 시작점(Cusp)에서는 곡률이 무한대가 되므로 예외 처리
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
        # [NEW] 사이클로이드 매개변수 방정식 적용
        # 루프 시작점(0,0)에 부드럽게 수평으로 연결되도록 위치를 역산하여 시프트
        val = -s / L_drop
        val = max(0.0, min(1.0, val))
        theta = 2 * math.acos(val)
        
        x = R_cycloid * (theta - math.pi - math.sin(theta))
        y = R_cycloid * (1 + math.cos(theta))
        z = 0.0
        return vector(x, y, z)
    elif s <= L_loop:
        x = R_loop * math.sin(s / R_loop)
        y = R_loop * (1 - math.cos(s / R_loop))
        z = 0.04 * (s / L_loop) 
        return vector(x, y, z)
    else:
        s_exit = s - L_loop
        x = s_exit
        y = 0
        z = 0.04 
        return vector(x, y, z)

# =====================================================================
# 3. VPython 3D 시각화 환경 구축
# =====================================================================
scene = canvas(title='<b>3D Realistic Coaster (Cycloid Drop)</b>',
               width=800, height=500,
               center=vector(0.05, R_loop, 0), background=vector(0.1, 0.1, 0.15))
scene.forward = vector(0.3, -0.3, -1) 
scene.ambient = color.gray(0.4) 

# 궤도 포인트 계산
pts_front, pts_back = [], []
for s_val in arange(-L_drop, S_MAX + 0.01, 0.005):
    pos = get_pos(s_val)
    pts_front.append(pos + vector(0, -R_ball, w_rail/2))
    pts_back.append(pos + vector(0, -R_ball, -w_rail/2))

# 실제 레일 질감
rail_color = vector(0.45, 0.45, 0.5)
rail_front_curve = curve(pos=pts_front, radius=0.0015, color=rail_color, shininess=0.9)
rail_back_curve  = curve(pos=pts_back, radius=0.0015, color=rail_color, shininess=0.9)

# 롤러코스터 침목 (Cross-ties)
for i in range(0, len(pts_front), 8): 
    cylinder(pos=pts_back[i], axis=pts_front[i]-pts_back[i], 
             radius=0.0008, color=vector(0.3, 0.3, 0.3))

# 출발 지점 플랫폼
start_pos = get_pos(-L_drop)
box(pos=start_pos + vector(-0.02, 0, 0), size=vector(0.04, 0.01, 0.04), color=vector(0.5, 0.5, 0.5), shininess=0.8)
cylinder(pos=vector(start_pos.x, 0, 0), axis=vector(0, start_pos.y, 0), radius=0.003, color=color.gray(0.3))

# 유리 구슬(Glass Bead)
ball = sphere(pos=start_pos, radius=R_ball, 
              color=vector(0.6, 0.9, 0.95), opacity=0.5, shininess=1.0, 
              make_trail=True, trail_color=color.cyan, trail_radius=0.001)

ground = box(pos=vector(0.1, -R_ball - 0.01, 0.02), size=vector(1.5, 0.01, 0.5), color=vector(0.2, 0.2, 0.2))

info_label = label(pos=vector(0, 2.5*R_loop, 0), text='', height=12, border=4,
                   font='monospace', color=color.white, background=color.black, opacity=0.8)

graph_win = graph(title="Velocity vs Time", xtitle="Time (s)", ytitle="Velocity (m/s)", width=800, height=250)
v_curve = gcurve(color=color.cyan, graph=graph_win)

# =====================================================================
# 4. 물리 연산 및 상태 머신(재시작 로직)
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
    k2_s = v + k1_v*dt/2
    k2_v = f_v(s + k1_s*dt/2, v + k1_v*dt/2)
    k3_s = v + k2_v*dt/2
    k3_v = f_v(s + k2_s*dt/2, v + k2_v*dt/2)
    k4_s = v + k3_v*dt
    k4_v = f_v(s + k3_s*dt, v + k3_v*dt)

    new_s = s + (dt / 6) * (k1_s + 2*k2_s + 2*k3_s + k4_s)
    new_v = v + (dt / 6) * (k1_v + 2*k2_v + 2*k3_v + k4_v)
    return new_s, new_v

s, v, t, N_now = -L_drop, 0.0, 0.0, 0.0
dt = 0.001
sim_state = 'running' 

def on_mouse_click(_):
    global sim_state, s, v, t, N_now
    if sim_state == 'running':
        sim_state = 'paused'
        info_label.text += '\n\n[일시정지 중 - 다시 클릭하여 재생]'
    elif sim_state == 'paused':
        sim_state = 'running'
    elif sim_state in ['finished', 'failed']:
        s, v, t, N_now = -L_drop, 0.0, 0.0, 0.0
        v_curve.data = []      
        ball.clear_trail()     
        sim_state = 'running'

scene.bind('click', on_mouse_click)
scene.append_to_caption('\n\n클릭 시 일시정지 되며, 끝난 후 클릭하면 처음부터 다시 재생됩니다.')

# =====================================================================
# 5. 무한 시뮬레이션 루프
# =====================================================================
while True:
    rate(300) 
    
    if sim_state != 'running':
        continue

    s, v = update_rk4(s, v, dt)
    t += dt

    theta, kappa = track_info(s)
    N_now = m * (v**2 * kappa + g * math.cos(theta))

    ball.pos = get_pos(s)
    v_curve.plot(t, v)

    info_label.text = (f't = {t:.2f} s\n'
                       f'Pos = {s:.3f} m\n'
                       f'v = {v:.3f} m/s\n'
                       f'N = {N_now:.4f} N')

    if kappa > 0 and N_now < 0 and math.cos(theta) < 0:
        sim_state = 'failed'
        info_label.text += '\n\n[궤도 이탈! 화면을 클릭해 재시작]'

    if s >= S_MAX:
        sim_state = 'finished'
        info_label.text += '\n\n[완주 성공! 화면을 클릭해 재시작]'