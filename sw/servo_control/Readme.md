# Servo Documentation

This controller is dedicated to control UART servos (**AX-12A** *DYNAMIXEL*) with the output of the flight controller (*PixHawk 6X*) PMW output.

For this purpose we use an Arduino Nano Every that reads the PWM and sends a command to the servos accordingly.

# Guide 

- Configure the PWM input channels of the arduino

```c
PWM servo1(2); // port D2
PWM servo2(3); // port D3
```

- Build and flash the sketch to and **arduino UNO**

- Connect the `tx` pin of the arduino to the signal pin of one of the servos (they should be in series and need only one connection to the MCU to work cf. [AX12 documentation](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/))

- Supply the two other pins of the servo with 9-12V

- Supply power to the arduino




