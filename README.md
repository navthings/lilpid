# lilpid

a rocket that balances itself. tilt it and the fan in the tail pushes it back up. for hack club fallen (shenzhen, dec 2026)

![rocket](img/assembly.png)

an IMU reads the tilt, a Pico 2 runs a PD loop, two servos point the fan.

made it because a friend and i want to fly a real one, and this one i can poke on a desk first.

## use

plug in usb + 12V, hold it upright 3 seconds, then over serial: `g` fan on, `s` fan off, `p` swap P / PD.

## gimbal

![gimbal](img/gimbal.png)
![fusion](img/fusion.jpg)

## pcb

![pcb](img/pcb.png)
![wiring](img/wiring.png)

## files

- `cad/` step files
- `print/` stls
- `controller/` kicad + gerbers
- `firmware/code.py` untested
- `main.py` pid sim
- `bom.csv`
