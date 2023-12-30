package org.firstinspires.ftc.teamcode.common.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.Globals;
import org.firstinspires.ftc.teamcode.common.util.packager.VSubsystem;

public class RobotH {

    // Drivetrain
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    // Linear Slides
    public DcMotorEx leftLinearSlide;
    public DcMotorEx rightLinearSlide;

    // Conveyor
    public DcMotorEx conveyor;

    // Servos


    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage = 12.0;

    public static RobotH instance = null;
    public boolean active;

    private BNO055IMU imu;

    private double angle;

    private ArrayList<VSubsystem> subsystems;

    // add subsystems for bucket, conveyor, drivetrain, drone, lift, etc

    public static RobotH getInstance() {
        if (instance == null) {
            instance = new RobotH();
        }
        instance.active = true;
        return instance;
    }

    public void init(final HardwareMap hardwareMap, final Telemetry telemetry) {

        this.hardwareMap = hardwareMap;
        this.telemetry = (Globals.USING_DASHBOARD) ? new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry()) : telemetry;

        this.backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.backRight = hardwareMap.get(DcMotorEx.class, "back_right");
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // do the same for all the other motors

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

    }

    public void update() {
        // all the extension, intake, drive train update;
    }

    public void restart() {
        for (VSubsystem i : subsystems) {
            i.reset();
        }
    }



}
