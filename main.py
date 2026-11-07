angle = 10.0
swing = 0.0
total = 0.0

kp = 0.1
ki = 0.01
kd = 0.4
wind = 0.3
limit = 0.5
steps = 40

for step in range(steps):
    error = 0 - angle
    push = kp * error + ki * total - kd * swing
    if abs(push) < limit:
        total += error  # antiwindup
    push = max(-limit, min(limit, push))  # clamp
    swing += push + wind  # physics
    angle += swing
    print(f"{angle:7.2f} {total:7.2f} " + " " * int(angle + 15) + "o")