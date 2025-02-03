package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class AxonServo {
    private ServoImplEx servo;
    private AnalogInput analogInput;

    public AxonServo(ServoImplEx servo, AnalogInput analogInput) {
        this.servo = servo;
        this.analogInput = analogInput;
    }

    public AxonServo(ServoImplEx servo) {
        this.servo = servo;
    }

    public void activate() { servo.setPwmEnable(); }

    public void deactivate() { servo.setPwmDisable();}

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
