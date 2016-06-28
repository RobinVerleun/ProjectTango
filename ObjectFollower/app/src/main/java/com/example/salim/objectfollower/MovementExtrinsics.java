package com.example.salim.objectfollower;

import com.google.atap.tangoservice.TangoPoseData;
import com.projecttango.rajawali.Pose;
import org.rajawali3d.math.Quaternion;

/**
 * Created by Robin on 2016-06-28.
 *
 * The goal of the class is to provide tools and define the relationships between the Tango device
 * and virtual objects in the game world.
 */

public class MovementExtrinsics {



    public MovementExtrinsics(){ }

    public void calculateTravel(){
        //TODO:Port code from Renderer to here.
    }

    public void calculateOnScreen(TangoPoseData tPose, Pose oPose, double[] EulerRotation){
        //TODO: Use the yaw to calculate rotation around the axis. Determine if
        //TODO: the vision coincides with the angle between the object and tango using the sin formula and dot product. 
    }

}
