package org.firstinspires.ftc.teamcode.WIP.common.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BucketSubsystem extends SubsystemBase {

    private final Servo bucketRotateLeft;
    private final Servo bucketRotateRight;
    private final Servo pixelHolder;
    
    public BucketSubsystem(final HardwareMap hardwareMap, final String left, final String right, final String pixel) {
        bucketRotateLeft = hardwareMap.get(Servo.class, left);
        bucketRotateRight = hardwareMap.get(Servo.class, right);
        pixelHolder = hardwareMap.get(Servo.class, pixel);
    }

    public void backdrop() {
        bucketRotateRight.setPosition(0.56);
        bucketRotateLeft.setPosition(0.23);
    }

    public void loading() {
        bucketRotateRight.setPosition(0.23);
        bucketRotateLeft.setPosition(0.56);
    }

    public void latchPixels() {
        pixelHolder.setPosition(0.43);
    }

    public void releasePixels() {
        pixelHolder.setPosition(0.56);
    }

    public void releaseFirstPixel() {
        pixelHolder.setPosition(0.51);
    }

}
