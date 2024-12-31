package org.firstinspires.ftc.teamcode.utils.localization;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.vision.processors.Sample;

public class SampleLimeLightWrapper extends LimeLightWrapper{
    private Pose3D targetPosition;
    public SampleLimeLightWrapper(Limelight3A limelight3A) {
        super(limelight3A);
    }
    public void setTargetSampleColor(Sample color) {
        super.start(color.ordinal()+1);
    }
    public void newTargetPosition() {
        LLResult result = super.getVaildResult();
        if(result!=null) {
            targetPosition = result.getBotpose();
        }
    }
    /**NEEDS TUNING**/
    public double distanceToStrafe() {
        return 0;
    }
    public boolean needsToBackUp(Pose3D pose) {
        return false;
    }
}
