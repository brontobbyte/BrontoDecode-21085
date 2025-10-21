package org.firstinspires.ftc.teamcode.Programs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Subsystems.IndexerSolvers;


@Config
@TeleOp(name = "TeleOpTesteSolvers")
public class TeleOpDecodeSolvers extends CommandOpMode {
    private final CRServo mechRotation;

    public TeleOpDecodeSolvers(final HardwareMap hMap, final String name) {
        mechRotation = hMap.get(CRServo.class, "servo_indexer");
    }
    public class Index extends CommandBase {

        // The subsystem the command runs on
        private final IndexerSolvers m_IndexerSubsystem;

        public Index(IndexerSolvers subsystem) {
            m_IndexerSubsystem = subsystem;
            addRequirements(m_IndexerSubsystem);
        }

        @Override
        public void initialize() {
            m_IndexerSubsystem.release();
        }

        @Override
        public boolean isFinished() {
            return true;
        }

    }

    public class Outdex extends CommandBase {

        // The subsystem the command runs on
        private final IndexerSolvers m_IndexerSubsystem;

        public Outdex(IndexerSolvers subsystem) {
            m_IndexerSubsystem = subsystem;
            addRequirements(m_IndexerSubsystem);
        }

        @Override
        public void initialize() {
            m_IndexerSubsystem.grab();
        }

        @Override
        public boolean isFinished() {
            return true;
        }

    }

    private IndexerSolvers indexer;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().run();
        indexer = new IndexerSolvers(hardwareMap, "servo_indexer");
        GamepadEx toolOp = new GamepadEx(gamepad1);
        GamepadButton exampleButton = new GamepadButton(
                toolOp, GamepadKeys.Button.A
        );
        GamepadButton exampleButton2 = new GamepadButton(
                toolOp, GamepadKeys.Button.B
        );
        toolOp.getGamepadButton(GamepadKeys.Button.A);
        toolOp.getGamepadButton(GamepadKeys.Button.B);
        exampleButton2.whenPressed(new InstantCommand(() -> mechRotation.setPower(1)));
        exampleButton2.whenReleased(new InstantCommand(() -> mechRotation.setPower(0)));
        exampleButton.whenReleased(new InstantCommand(indexer::release, indexer));
        exampleButton.whenPressed(new InstantCommand(indexer::grab, indexer));
        register(indexer);

    }
}
