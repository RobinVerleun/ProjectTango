package com.example.salim.objectfollower;

import com.google.atap.tangoservice.TangoPoseData;
import org.rajawali3d.math.Quaternion;
import org.rajawali3d.math.vector.Vector3;

import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Created by Robin on 2016-06-28.
 *
 * The goal of the class is to provide tools and define the relationships between the Tango device
 * and virtual objects in the game world.
 */

public final class MovementExtrinsics {

    private static final MovementExtrinsics INSTANCE = new MovementExtrinsics();

    public static final int INDEX_PITCH = 0;
    public static final int INDEX_YAW = 1;
    public static final int INDEX_ROLL = 2;
    private static final float OBJECT_SPEED = 0.004f;
    private static final float OBJECT_THRESHOLD = 0.15f;

    private double mHorizontalFOV, mVerticalFOV, HorizontalCameraAngle, VerticalCameraAngle, oldVertAngle;
    private static AtomicBoolean firstRun = new AtomicBoolean(true);

    private MovementExtrinsics(){
        if(INSTANCE != null){
            throw new IllegalStateException("Already instantiated.");
        }
    }

    public static MovementExtrinsics getInstance(){
        return INSTANCE;
    }

    public void set_FOV(double _HorizontalFOV, double _VerticalFOV){
        mHorizontalFOV = _HorizontalFOV;
        mVerticalFOV = _VerticalFOV;
    }

    public Vector3 calculateTravel(TangoPoseData mDevicePose, Vector3 ObjCoord){
        float dampening_Factor = OBJECT_THRESHOLD * OBJECT_SPEED;
        double ResultX, ResultY, ResultZ;

        if(Math.abs((mDevicePose.translation[0] - ObjCoord.x) * (OBJECT_SPEED)) > dampening_Factor) {
            ResultX = ((mDevicePose.translation[0] - ObjCoord.x) * (OBJECT_SPEED)); //Horizontal Movement
        } else { ResultX = 0; }
        if(Math.abs((mDevicePose.translation[1] - ObjCoord.z) * (OBJECT_SPEED)) > dampening_Factor) {
            ResultY = ((-1 * mDevicePose.translation[1] - ObjCoord.z) * (OBJECT_SPEED)); //Forward back Movement; Note: Tango Z-axis is negative of object Z-axis
        } else { ResultY = 0; }
        if(Math.abs((mDevicePose.translation[2] - ObjCoord.y) * (OBJECT_SPEED)) > dampening_Factor) {
            ResultZ = ((mDevicePose.translation[2] - ObjCoord.y) * (OBJECT_SPEED)); //Vertical Movement
        } else { ResultZ = 0; }

        return new Vector3(ResultX, ResultZ, ResultY);
    }

    public double calculateDistance(TangoPoseData mDevicePose, Vector3 ObjCoord){
        double ResultX, ResultY, ResultZ;
        ResultX = ((mDevicePose.translation[0] - ObjCoord.x)); //Horizontal Movement
        ResultY = ((mDevicePose.translation[1] - ObjCoord.z)); //Forward back Movement; Note: Tango Z-axis is negative of object Z-axis
        ResultZ = ((mDevicePose.translation[2] - ObjCoord.y)); //Vertical Movement
        return new Vector3(ResultX,ResultY,ResultZ).length();
    }

    public boolean calculateOnScreen(TangoPoseData tPose, Vector3 objLocation){
        //Get coordinates of tango and object. Note: Vector3 takes coordinates as X,Y,Z and accesses them as such. TangoPose provides coordinates as an X,Z,Y system and must be entered slightly differently.
        Vector3 tangoLocation = new Vector3(tPose.translation[0], tPose.translation[2], tPose.translation[1]);

        //Calculate Vector between tango and object
        Vector3 V_ObjTango = new Vector3(objLocation.x - tangoLocation.x, objLocation.y - tangoLocation.y, (-1) * objLocation.z - tangoLocation.z);

        //Get the value of the angles using the sin formula for horizontal and vertical planes. Angle between object and tango on the xz-plane, with respect to the z-axis
        double m_XZAngle = getHorizontalAngleBetweenObjects(V_ObjTango);

        //Angle between object and tango on the xy-plane, with respect to the y-axis
        double m_XYAngle = getVerticalAngleBetweenObjects(V_ObjTango);

        //Determine where the screen is currently looking - we flip the vertical angle to ease calculations later.
        HorizontalCameraAngle = getYawfromTangoPose(tPose);

        if(firstRun.getAndSet(false)){
            VerticalCameraAngle = 180 - getRollfromTangoPose(tPose);
        } else {
            CheckandSetAngleDifference(180 - getRollfromTangoPose(tPose));
        }
        oldVertAngle = VerticalCameraAngle;

        if(HorizontalCameraAngle < 0){
            HorizontalCameraAngle = 180 + (HorizontalCameraAngle + 180);
        }

        //Calculate limits for each side of the screen
        double leftLimit = (HorizontalCameraAngle + mHorizontalFOV/2) % 360;
        double rightLimit = (HorizontalCameraAngle - mHorizontalFOV/2) % 360;
        double lowerLimit = (VerticalCameraAngle - mVerticalFOV/2 ) % 360;
        double upperLimit = (VerticalCameraAngle + mVerticalFOV) % 360;

        if(rightLimit < 0){
            rightLimit = 360 + rightLimit;
        }

        //  System.out.println("Lower: " + lowerLimit + ", Mine: " + m_XYAngle + ", Upper: " + upperLimit);

        //Calculate if the sphere is on the screen vertically
        if(lowerLimit < m_XYAngle && upperLimit > m_XYAngle) {
            if (leftLimit > mHorizontalFOV) {
                if (leftLimit > m_XZAngle && rightLimit < m_XZAngle) {
                    return true;
                }
            } else if (leftLimit - m_XZAngle > 0) {
                return true;
            } else if (m_XZAngle > rightLimit) {
                return true;
            }
        }
        return false;
    }

    public double getVerticalAngleBetweenObjects(Vector3 m3DVector){
        Vector3 yUnit = new Vector3(0,-1,0);
        return yUnit.angle(m3DVector);
    }

    public double getHorizontalAngleBetweenObjects(Vector3 m3DVector){
        Vector3 zUnit = new Vector3(0,1,0);
        Vector3 mXZVector = new Vector3(m3DVector.x, m3DVector.z, 0);
        if(m3DVector.x >= 0){
            return  360 - zUnit.angle(mXZVector);
        } else {
            return zUnit.angle(mXZVector);
        }
    }

    public void CheckandSetAngleDifference(double currentAngle) {
        if(Math.abs(oldVertAngle - currentAngle) / oldVertAngle < 0.1) {
            VerticalCameraAngle = currentAngle;
        }
    }

    public double[] getEulerAnglesfromTangoPose(TangoPoseData tangoPose){
        Quaternion mQuat = new Quaternion
                (tangoPose.rotation[0], tangoPose.rotation[1], tangoPose.rotation[2], tangoPose.rotation[3]);
        double[] EuelerAngles = {Math.toDegrees(mQuat.getPitch()), Math.toDegrees(mQuat.getYaw()), Math.toDegrees(mQuat.getRoll())};
        return EuelerAngles;
    }

    public double getPitchfromTangoPose(TangoPoseData tangoPose){
        Quaternion mQuat = new Quaternion
                (tangoPose.rotation[0], tangoPose.rotation[1], tangoPose.rotation[2], tangoPose.rotation[3]);
        return Math.toDegrees(mQuat.getPitch());
    }

    public double getYawfromTangoPose(TangoPoseData tangoPose){
        Quaternion mQuat = new Quaternion
                (tangoPose.rotation[0], tangoPose.rotation[1], tangoPose.rotation[2], tangoPose.rotation[3]);
        return Math.toDegrees(mQuat.getYaw());
    }

    public double getRollfromTangoPose(TangoPoseData tangoPose){
        Quaternion mQuat = new Quaternion
                (tangoPose.rotation[0], tangoPose.rotation[1], tangoPose.rotation[2], tangoPose.rotation[3]);
        return Math.toDegrees(mQuat.getRoll());
    }
}
