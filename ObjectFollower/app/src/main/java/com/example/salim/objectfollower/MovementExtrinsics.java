package com.example.salim.objectfollower;

import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoPoseData;
import com.projecttango.rajawali.Pose;
import org.rajawali3d.math.Quaternion;
import org.rajawali3d.math.vector.Vector2;
import org.rajawali3d.math.vector.Vector3;

/**
 * Created by Robin on 2016-06-28.
 *
 * The goal of the class is to provide tools and define the relationships between the Tango device
 * and virtual objects in the game world.
 */

public class MovementExtrinsics {

    public static final int INDEX_PITCH = 0;
    public static final int INDEX_YAW = 1;
    public static final int INDEX_ROLL = 2;

    private double mHorizontalFOV, mVerticalFOV;

    public MovementExtrinsics(double _HorizontalFOV, double _VerticalFOV){
        mHorizontalFOV = _HorizontalFOV;
        mVerticalFOV = _VerticalFOV;
    }

    public void set_FOV(double _HorizontalFOV, double _VerticalFOV){
        mHorizontalFOV = _HorizontalFOV;
        mVerticalFOV = _VerticalFOV;
    }

    public void calculateTravel(){
        //TODO:Port code from Renderer to here.
    }

    public boolean calculateOnScreen(TangoPoseData tPose, Vector3 objLocation){
        /*
         Get coordinates of tango and object. Note: Vector3 takes coordinates as X,Y,Z and accesses
         them as such. TangoPose provides coordinates as an X,Z,Y system and must be entered slightly differently.
         */
        Vector3 tangoLocation = new Vector3(tPose.translation[0], tPose.translation[2], tPose.translation[1]);

        /*
         Calculate Vector between tango and object
         */
        Vector3 V_ObjTango = new Vector3(objLocation.x - tangoLocation.x, objLocation.y - tangoLocation.y, objLocation.z - (-1) * tangoLocation.z);
        //System.out.println("ObjVector X: " + V_ObjTango.x + ",  ObjVector Y: " + V_ObjTango.y + ",  ObjVector Z: " + V_ObjTango.z);
        /*
         Get the value of the angles using the sin formula for horizontal and vertical planes.
        Angle between object and tango on the xz-plane, with respect to the z-axis
         */
        double m_XZAngle = getHorizontalAngleBetweenObjects(V_ObjTango);

        //Angle between object and tango on the xy-plane, with respect to the y-axis
        //TODO: Calculate vertical angle

        //Determine where the screen is currently looking
        double cameraAngle = getYawfromTangoPose(tPose);
        if(cameraAngle < 0){
            cameraAngle = 180 + (cameraAngle + 180);
        }
        double leftLimit = (cameraAngle + mHorizontalFOV/2) % 360;
        double rightLimit = (cameraAngle - mHorizontalFOV/2) % 360;

        if(leftLimit > mHorizontalFOV){
            if(leftLimit > m_XZAngle && rightLimit < m_XZAngle){
                return true;
            }
        } else if(leftLimit - m_XZAngle > 0){
            return true;
        } else if(m_XZAngle > rightLimit){
            return true;
        }
        return false;
    }

    public double getHorizontalAngleBetweenObjects(Vector3 m3DVector){
        Vector2 yUnit = new Vector2(0,1);
        Vector2 m2DVector = new Vector2(m3DVector.x, m3DVector.z);

        double dotResult = ( (yUnit.getX() * m2DVector.getX()) + (yUnit.getY() * m2DVector.getY()) );
        double magnitudeResult =
                (Math.sqrt(Math.pow(yUnit.getX(),2) + Math.pow(yUnit.getY(),2))) *
                        (Math.sqrt(Math.pow(m2DVector.getX(), 2) + Math.pow(m2DVector.getY(),2)));

        double mAngle = Math.toDegrees(Math.acos(dotResult/magnitudeResult));

        //Check to see if the angle of the vectors is in the right coordinate and convert.
        if(m3DVector.x > 0){
            mAngle = 360 - mAngle;
        }
        //System.out.println(mAngle);
        return mAngle;
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
