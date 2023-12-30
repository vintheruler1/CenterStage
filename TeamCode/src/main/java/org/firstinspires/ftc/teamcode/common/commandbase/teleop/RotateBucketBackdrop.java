package org.firstinspires.ftc.teamcode.WIP.common.commandbase.teleop;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.WIP.common.subsystem.BucketSubsystem;

public class RotateBucketBackdrop extends CommandBase {

    public final BucketSubsystem bucket;

    public RotateBucketBackdrop(BucketSubsystem subsystem) {
        bucket = subsystem;
        addRequirements(bucket);
    }

    @Override
    public void initialize() {
        bucket.backdrop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
