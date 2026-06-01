from vpython import *


ball = sphere(pos=vec(0,0,0),
           radius=0.5, color=color.red)
v = vec(0.5, 0, 0)   # 속도 벡터


while True:
    rate(60)          # 60 FPS
    dt = 0.01   #시간 간격(단위)
    ball.pos += v * dt # 위치 업데이트
