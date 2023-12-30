package org.firstinspires.ftc.teamcode.WIP.common.commandbase.group;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.WIP.common.commandbase.teleop.RotateBucketBackdrop;
import org.firstinspires.ftc.teamcode.WIP.common.subsystem.BucketSubsystem;

public class BucketBackdrop extends SequentialCommandGroup {

    public BucketBackdrop(BucketSubsystem bucket) {
        addCommands(
                new RotateBucketBackdrop(bucket)
        );

        addRequirements(bucket);
    }

}
