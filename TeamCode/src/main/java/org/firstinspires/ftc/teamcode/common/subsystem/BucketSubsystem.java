package org.firstinspires.ftc.teamcode.common.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.common.util.Bucket;
import org.firstinspires.ftc.teamcode.common.util.packager.VSubsystem;

import org.firstinspires.ftc.teamcode.common.hardware.RobotH;
import org.jetbrains.annotations.NotNull;

public class BucketSubsystem extends VSubsystem {

    private RobotH robot = RobotH.getInstance();
    private int backdropHeight = 0;

    public enum BucketState {
        OPEN,
        CLOSE
    }

    public enum BucketPosition {
        UP,
        DOWN
    }

    public BucketState bucketState = BucketState.CLOSE;
    public BucketPosition bucketPosition = BucketPosition.UP;

    public BucketSubsystem() {
        this.robot = RobotH.getInstance();
    }

    public void updateState(@NotNull BucketState state, @NotNull BucketPosition position) {
        double pos = getBucketStatePosition(state, position);
        switch (position) {
            case UP:
                robot.bucketServo.setPosition(pos);
                this.bucketState = state;
                break;
            case DOWN:
                robot.bucketServo.setPosition(pos);
                break;
        }
    }

    public void updateState(@NotNull BucketPosition position) {
        this.bucketPosition = position;
    }

    private double getBucketStatePosition(BucketState state, BucketPosition position) {
        switch (state) {
            case OPEN:
                switch (position) {
                    case UP:
                        return 0.07;
                    case DOWN:
                        return 0.15;
                }
            case CLOSE:
                switch (position) {
                    case UP:
                        return .15;
                    case DOWN:
                        return .12;
                }
            default:
                return 0.0;
        }
    }

    public BucketState getBucketState(BucketPosition position) {
        switch (position) {
            case UP:
                return BucketState.CLOSE;
            case DOWN:
                return BucketState.OPEN;
            default:
                return BucketState.CLOSE;
        }

    }

    @Override
    public void periodic() {

    }

    @Override
    public void read() {

    }

    @Override
    public void write() {

    }

    @Override
    public void reset() {

    }
}
