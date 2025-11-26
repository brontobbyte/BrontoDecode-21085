/*package org.firstinspires.ftc.teamcode.Subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;


public class Turret2 implements Subsystem {
    public static final Turret2 INSTANCE = new Turret2();
    private Turret2() {}
    private final MotorEx Turret = new MotorEx("motor_turret");
    private final MotorEx Turret2 = new MotorEx("motor_turret");

    public Command dirTurret = new SetPower(Turret,0.3).requires(this);
    public Command esqTurret = new SetPower(Turret2,-0.3).requires(this);

    public Command runContinuosrt = new LambdaCommand()

            .setStart(() -> Turret.setPower(0.5))
            .setStop(interrupted ->Turret.setPower(0))
            .setIsDone(() -> false)
            .requires(this)
            .named("Run Turret direita");

}

 */