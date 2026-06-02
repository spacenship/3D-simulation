import sys
# 에러 방지용 코드
try:
    import pkg_resources
except ImportError:
    import pip._vendor.pkg_resources as pkg_resources
    sys.modules['pkg_resources'] = pkg_resources




from vpython import *
import math

# =====================================================================
# 1. 실제 실험용 물리 파라미터 및 유리구슬 설정
# =====================================================================
g = 9.81              # 중력 가속도 (m/s²)
h_offset = 0.05       # 트랙을 지면에서 띄울 높이 (5cm)
z_gap = 0.04          # 레일 충돌을 막기 위해 옆으로 비껴낼 간격 (4cm)

# 유리구슬 (Glass Marble) 설정 — 무게 10g 기준
m = 0.010              
rho_glass = 2500       
V_ball = m / rho_glass 
R_ball = (V_ball / ((4/3) * math.pi))**(1/3)  

a = 0.18              # 사이클로이드 반경 (18cm) -> 시작 높이 H = 36cm
s_ramp_end = 4 * a    

# 클로소이드 루프 파라미터 정의 (물방울 모양 제어)
R_top = 0.07          # 루프 꼭대기에서의 최소 곡률 반지름 (7cm)
R_base = 0.25         # 루프 진입/진출부에서의 최대 곡률 반지름 (25cm)
s_loop_len = 0.70     # 루프 전체 곡선 길이 (약 70cm)

s_loop_end = s_ramp_end + s_loop_len
L_exit = 0.3          
s_max = s_loop_end + L_exit

ROLL_FACTOR = 5.0 / 7.0  
w_rail = 0.012         # 레일 간격 12mm
mu_r   = 0.005         

C_d     = 0.47         
rho_air = 1.225        
A       = math.pi * R_ball**2           
k_drag  = 0.5 * rho_air * C_d * A / m  

# =====================================================================
# 2. 클로소이드 궤적 테이블 미리 생성
# =====================================================================
N_samples = 1000
loop_profile = []
current_pos = vector(0, h_offset, 0)
current_theta = 0.0  

ds_sample = s_loop_len / N_samples
for i in range(N_samples + 1):
    s_l = i * ds_sample
    ratio = s_l / s_loop_len
    
    if ratio <= 0.5:
        k = (1.0/R_base) + ((1.0/R_top) - (1.0/R_base)) * (ratio * 2)
    else:
        k = (1.0/R_top) - ((1.0/R_top) - (1.0/R_base)) * ((ratio - 0.5) * 2)
        
    loop_profile.append((vec(current_pos.x, current_pos.y, current_pos.z), current_theta, k))
    
    current_theta += k * ds_sample
    current_pos += vector(math.cos(current_theta), math.sin(current_theta), 0) * ds_sample
    current_pos.z = - z_gap * (s_l / s_loop_len)

# 최종 생성된 클로소이드 루프의 끝점 저장
loop_end_pos = vec(loop_profile[-1][0].x, loop_profile[-1][0].y, loop_profile[-1][0].z)

# =====================================================================
# 2-1. 트랙 기하학 함수 (에러 수정 버전)
# =====================================================================
def get_track_geometry(s):
    # 변수 초기화로 UnboundLocalError 원천 차단
    track_pos = vector(0,0,0)
    tangent = vector(1,0,0)
    normal = vector(0,1,0)
    curvature = 0.0

    if s <= s_ramp_end:
        # 1. 사이클로이드 하강 경사면
        s_clamp = max(1e-5, min(s, s_ramp_end - 1e-5))
        phi = 2 * math.acos(1 - s_clamp / (4 * a))
        x = a * (phi - math.sin(phi)) - a * math.pi
        y = a * (1 + math.cos(phi)) + h_offset
        track_pos = vector(x, y, 0)
        
        sin_half = math.sin(phi / 2)
        cos_half = math.cos(phi / 2)
        tangent = vector(sin_half, -cos_half, 0)
        normal = vector(cos_half, sin_half, 0)
        curvature = 1.0 / (4 * a * sin_half)
        
    elif s <= s_loop_end:
        # 2. 클로소이드 루프 구간 (변수명 명확히 매칭)
        s_loop = s - s_ramp_end
        idx = min(int((s_loop / s_loop_len) * N_samples), N_samples)
        
        # loop_profile에서 꺼내온 위치 값을 track_pos에 정확히 대입합니다.
        p_profile, theta, curvature = loop_profile[idx]
        track_pos = vec(p_profile.x, p_profile.y, p_profile.z)
        
        dx_ds = math.cos(theta)
        dy_ds = math.sin(theta)
        dz_ds = - z_gap / s_loop_len
        tangent = hat(vector(dx_ds, dy_ds, dz_ds))
        normal = hat(vector(-math.sin(theta), math.cos(theta), 0))
        
    else:
        # 3. 직선 탈출 구간
        s_exit = s - s_loop_end
        track_pos = loop_end_pos + vector(s_exit, 0, 0)
        tangent = vector(1, 0, 0)
        normal = vector(0, 1, 0)
        curvature = 0.0
        
    return track_pos, tangent, normal, curvature

# =====================================================================
# 3. VPython 3D 시각화 환경 구축
# =====================================================================
scene = canvas(title='<b>Real Clothoid(Teardrop) Loop Roller Coaster Simulation</b>',
               width=900, height=500,
               center=vector(-a*math.pi/2 + 0.1, a, -z_gap/2),
               background=color.black)
scene.forward = vector(0.2, -0.15, -0.9)

ground = box(pos=vector(-a*math.pi/2 + 0.2, -0.0025, -z_gap/2), size=vector(1.8, 0.005, 0.5), color=vector(0.2, 0.2, 0.2))

rail_left = curve(radius=0.002, color=vector(0.9, 0.4, 0.1), shininess=0.8)
rail_right = curve(radius=0.002, color=vector(0.9, 0.4, 0.1), shininess=0.8)
for i in range(600):
    s_t = (s_max * i) / 599
    pos_t, _, _, _ = get_track_geometry(s_t)
    rail_left.append(pos_t + vector(0, 0, w_rail/2))
    rail_right.append(pos_t + vector(0, 0, -w_rail/2))

# 구조물 지지대 배치
cylinder(pos=vector(-a*math.pi, 0, 0), axis=vector(0, 2*a + h_offset, 0), radius=0.005, color=vector(0.5, 0.5, 0.5))
cylinder(pos=vector(0, 0, 0), axis=vector(0, h_offset, 0), radius=0.005, color=vector(0.5, 0.5, 0.5))
cylinder(pos=vector(loop_end_pos.x, 0, -z_gap), axis=vector(0, loop_end_pos.y, 0), radius=0.005, color=vector(0.5, 0.5, 0.5))

ball = sphere(pos=vector(0, 0, 0), radius=R_ball,
              color=vector(0.4, 0.7, 1.0), opacity=0.7, shininess=1.0,
              make_trail=True, trail_color=color.green, trail_radius=0.001)

info_label = label(pos=vector(-a*math.pi/2, 2*a + h_offset + 0.05, 0), text='', height=12, font='monospace')

# =====================================================================
# 4. 수치 해석 및 시뮬레이션 실행 (무한 반복 및 포물선 추락 내장)
# =====================================================================
def f_v(s, v):
    pos, tangent, normal, curvature = get_track_geometry(s)
    g_vec = vector(0, -g, 0)
    g_t = g_vec.dot(tangent)
    g_n = g_vec.dot(normal)
    
    a_n = (v**2) * curvature
    N = m * (a_n - g_n)
    
    f_roll = mu_r * max(N, 0.0)
    sign_v = (1 if v > 0 else -1) if v != 0 else 0
    a_raw = g_t - (f_roll / m) * sign_v - k_drag * v**2 * sign_v
    return ROLL_FACTOR * a_raw

def update_rk4(s, v, dt):
    k1_s = v; k1_v = f_v(s, v)
    k2_s = v + k1_v * dt / 2; k2_v = f_v(s + k1_s * dt / 2, v + k1_v * dt / 2)
    k3_s = v + k2_v * dt / 2; k3_v = f_v(s + k2_s * dt / 2, v + k2_v * dt / 2)
    s_next_half = min(s + k2_s * dt / 2, s_max - 1e-5)
    k4_s = v + k3_v * dt; k4_v = f_v(s_next_half, v + k3_v * dt)
    return s + (dt/6)*(k1_s + 2*k2_s + 2*k3_s + k4_s), v + (dt/6)*(k1_v + 2*k2_v + 2*k3_v + k4_v)

s = 0.002; v = 0.0; dt = 0.001; t = 0.0
trial_count = 1
is_falling = False       
vel_3d = vector(0, 0, 0)  

while True:
    rate(150)
    
    if is_falling:
        t += dt
        speed_3d = mag(vel_3d)
        drag_acc_3d = - k_drag * speed_3d * vel_3d if speed_3d > 0 else vector(0,0,0)
        acc_3d = vector(0, -g, 0) + drag_acc_3d
        vel_3d += acc_3d * dt
        ball.pos += vel_3d * dt
        
        if ball.pos.y <= R_ball:
            ball.pos.y = R_ball  
            info_label.text = f'Trial: #{trial_count}\nZone: 💥 CRASHED!\nSpeed: {mag(vel_3d):.3f} m/s'
            sleep(1.5)           
            s = 0.002; v = 0.0; t = 0.0
            trial_count += 1
            is_falling = False
            ball.clear_trail()
            continue
    else:
        s, v = update_rk4(s, v, dt)
        t += dt
        
        if s >= s_max - 0.005:
            s = 0.002; v = 0.0; trial_count += 1
            ball.clear_trail()
            continue
            
        pos_now, tangent_now, normal_now, curvature_now = get_track_geometry(s)
        ball.pos = pos_now
        
        a_n_now = (v**2) * curvature_now
        g_n_now = vector(0, -g, 0).dot(normal_now)
        N_now = m * (a_n_now - g_n_now)
        
        if s_ramp_end < s < s_loop_end:
            if N_now < 0 and ball.pos.y > (h_offset + R_top):
                is_falling = True
                vel_3d = v * tangent_now 
                continue
                
        zone = "1. Cycloid Ramp" if s <= s_ramp_end else ("2. Clothoid Loop" if s <= s_loop_end else "3. Exit Straight")
        info_label.text = (f'Trial: #{trial_count}\n'
                           f'Zone: {zone}\n'
                           f'Speed: {v:.3f} m/s\n'
                           f'Normal Force: {N_now:.4f} N')