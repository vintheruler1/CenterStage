package org.firstinspires.ftc.teamcode.common.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double kP, kI, kD;
    private ElapsedTime timer = new ElapsedTime();
    private double targetPos;
    private double lastError = 0;
    private double accumulatedError = 0;
    private double lastTime = -1;
    private double lastSlope = 0;
    private boolean isAngle;

    public PIDController(double target, double p, double i, double d, boolean isAngle) {
        kP = p;
        kI = i;
        kD = d;
        targetPos = target;
        this.isAngle = isAngle;
    }

    public void setTargetPosition(double target) {
        targetPos = target;
    }

    public double update(double currentPos) {
        double error = calculateError(currentPos);

        updateAccumulatedError(error);
        double slope = calculateSlope(error);
        updateLastValues(error, slope);

        return calculateMotorPower(error, slope);
    }

    private double calculateError(double currentPos) {
        double error = targetPos - currentPos;

        if (isAngle) {
            error = normalizeAngle(error);
        }

        return error;
    }

    private double normalizeAngle(double angle) {
        angle %= 360;
        angle += 360;
        angle %= 360;
        if (angle > 180) {
            angle -= 360;
        }
        return angle;
    }

    private void updateAccumulatedError(double error) {
        accumulatedError *= Math.signum(error);
        accumulatedError += error;
        if (Math.abs(error) < 2) {
            accumulatedError = 0;
        }
    }

    private double calculateSlope(double error) {
        double slope = 0;
        if (lastTime > 0) {
            slope = (error - lastError) / (timer.milliseconds() - lastTime);
        }
        return slope;
    }

    private void updateLastValues(double error, double slope) {
        lastSlope = slope;
        lastError = error;
        lastTime = timer.milliseconds();
    }

    private double calculateMotorPower(double error, double slope) {
        return 0.1 * Math.signum(error) +
                0.9 * Math.tanh(kP * error + kI * accumulatedError - kD * slope);
    }

    public double getLastSlope() {
        return lastSlope;
    }
}
