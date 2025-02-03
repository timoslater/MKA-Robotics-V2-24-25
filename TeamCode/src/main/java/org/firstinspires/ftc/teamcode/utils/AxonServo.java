package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

public class AxonServo {
    private Servo servo;
    private AnalogInput analogInput;

    public AxonServo(Servo servo, AnalogInput analogInput) {
        this.servo = servo;
        this.analogInput = analogInput;
    }

    public AxonServo(Servo servo) {
        this.servo = servo;
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }

    public void setDirection(Servo.Direction direction) {
        servo.setDirection(direction);
    }

    public double getPosition() throws Exception {
        if (analogInput == null) {
            throw new Exception("No Analog Input Assigned");
        }

        return analogInput.getVoltage() / 3.3 * 360;
    }
}
