package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import android.graphics.Color;
@Configurable
@TeleOp(name = "Leitor Verde e Roxo", group = "Sensor")
public class LeitorVerdeRoxo extends LinearOpMode {

    NormalizedColorSensor colorSensor; // variável pro sensor
    int verde = 140;
    int reconhece;

    @Override
    public void runOpMode() {

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        waitForStart();

        float[] hsv = new float[3]; // guarda as cores que o sensor vê; hue, saturation e value

        while (opModeIsActive()) {

            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            Color.colorToHSV(colors.toColor(), hsv); // Converte os valores RGB do sensor para hsv

            float hue = hsv[0]; // pega só a matriz da cor

            String corDetectada = "nada";

            if (hue > verde && hue < 170) { // entre valores x é verde
                corDetectada = "v";
            }
            else if (hue > 260 && hue < 310) {
                corDetectada = "r";
            }

            telemetry.addData("Hue", hue);
            telemetry.addData("Cor Detectada", corDetectada);
            telemetry.update();
        }
    }
}