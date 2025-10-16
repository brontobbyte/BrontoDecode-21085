package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.robocol.Command;

import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;

public class indexer implements Subsystem {
    public static final indexer INSTANCE = new indexer();
    private indexer() { }

    // Use CRServo for continuous rotation
    private final CRServo indexerServo = hardwareMap.get(CRServo.class, "indexer_servo");

    // Commands for control
    public LambdaCommand empurra = new LambdaCommand("Spin Forward")
            .setStart(() -> indexerServo.setPower(0.8)) // Adjust speed as needed
            .requires(this);

    public LambdaCommand puxa = new LambdaCommand("Spin Reverse")
            .setStart(() -> indexerServo.setPower(0.2)) // Adjust speed as needed
            .requires(this);

    public LambdaCommand STOP = new LambdaCommand("Stop")
            .setStart(() -> indexerServo.setPower(0))
            .requires(this);

    @Override
    public void periodic() {
        // Periodic logic if needed
    }
}