/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.example.salim.objectfollower;

import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoPoseData;

import android.content.Context;

import android.graphics.drawable.GradientDrawable;
import android.transition.Scene;
import android.util.Log;
import android.view.MotionEvent;

import org.rajawali3d.Object3D;
import org.rajawali3d.lights.DirectionalLight;
import org.rajawali3d.materials.Material;
import org.rajawali3d.materials.methods.DiffuseMethod;
import org.rajawali3d.materials.textures.ATexture;
import org.rajawali3d.materials.textures.StreamingTexture;
import org.rajawali3d.materials.textures.Texture;
import org.rajawali3d.math.Matrix4;
import org.rajawali3d.math.Quaternion;
import org.rajawali3d.math.vector.Vector3;
import org.rajawali3d.primitives.Cube;
import org.rajawali3d.primitives.ScreenQuad;
import org.rajawali3d.primitives.Sphere;
import org.rajawali3d.renderer.RajawaliRenderer;

import javax.microedition.khronos.opengles.GL10;

import com.projecttango.rajawali.DeviceExtrinsics;
import com.projecttango.rajawali.Pose;
import com.projecttango.rajawali.ScenePoseCalculator;
import com.projecttango.tangosupport.TangoSupport;

/**
 * Very simple example augmented reality renderer which displays a cube fixed in place.
 * The position of the cube in the OpenGL world is updated using the {@code updateObjectPose}
 * method.
 */
public class ObjectFollowerRenderer extends RajawaliRenderer {
    private static final String TAG = ObjectFollowerRenderer.class.getSimpleName();
    private static final float OBJECT_SPEED = 0.007f;
    private static final float OBJECT_THRESHOLD = 0.15f;


    // Augmented Reality related fields
    private ATexture mTangoCameraTexture;
    private boolean mSceneCameraConfigured;

    private Object3D mObject;
    private Pose mObjectPose;
    private boolean mObjectPoseUpdated = false;

    private TangoPoseData mDevicePose;

    public ObjectFollowerRenderer(Context context) {
        super(context);
    }

    @Override
    protected void initScene() {
        // Create a quad covering the whole background and assign a texture to it where the
        // Tango color camera contents will be rendered.
        ScreenQuad backgroundQuad = new ScreenQuad();
        Material tangoCameraMaterial = new Material();
        tangoCameraMaterial.setColorInfluence(0);
        // We need to use Rajawali's {@code StreamingTexture} since it sets up the texture
        // for GL_TEXTURE_EXTERNAL_OES rendering
        mTangoCameraTexture =
                new StreamingTexture("camera", (StreamingTexture.ISurfaceListener) null);
        try {
            tangoCameraMaterial.addTexture(mTangoCameraTexture);
            backgroundQuad.setMaterial(tangoCameraMaterial);
        } catch (ATexture.TextureException e) {
            Log.e(TAG, "Exception creating texture for RGB camera contents", e);
        }
        getCurrentScene().addChildAt(backgroundQuad, 0);

        // Add a directional light in an arbitrary direction.
        DirectionalLight light = new DirectionalLight(1, 0.2, -1);
        light.setColor(1, 1, 1);
        light.setPower(0.8f);
        light.setPosition(3, 2, 4);
        getCurrentScene().addLight(light);

        // Set-up a material: green with application of the light and
        // instructions.
        Material material = new Material();
        material.setColor(0xff009900);
        try {
            Texture t = new Texture("instructions", R.drawable.instructions);
            material.addTexture(t);
        } catch (ATexture.TextureException e) {
            e.printStackTrace();
        }
        material.setColorInfluence(0.1f);
        material.enableLighting(true);
        material.setDiffuseMethod(new DiffuseMethod.Lambert());

        // Build a Cube and place it initially in the origin.
        mObject = new Sphere(0.075f,24,24);
        mObject.setMaterial(material);
        mObject.setPosition(0, 0, -3);
        mObject.setRotation(Vector3.Axis.Z, 180);
        getCurrentScene().addChild(mObject);
    }

    @Override
    protected void onRender(long elapsedRealTime, double deltaTime) {
        // Update the AR object if necessary
        // Synchronize against concurrent access with the setter below.
        synchronized (this) {
            if (mObjectPoseUpdated) {
                // Place the 3D object in the location of the detected plane.
                mObject.setPosition(mObjectPose.getPosition());
                mObject.setOrientation(mObjectPose.getOrientation());
                mObjectPoseUpdated = false;
            }

        }


        super.onRender(elapsedRealTime, deltaTime);
    }

    /**
     * Save the updated plane fit pose to update the AR object on the next render pass.
     * This is synchronized against concurrent access in the render loop above.
     */
    public synchronized void updateObjectPose(Vector3 onTouchPose) {
        mObjectPose = new Pose(onTouchPose, new Quaternion(0.0, 0.0, 0.0, 0.0));
        mObjectPoseUpdated = true;

    }

    public synchronized void moveSphere(TangoPoseData currentPose){
        Vector3 coordinates = calculateTravel(currentPose);

        mObject.moveForward(coordinates.z);
        mObject.moveRight(coordinates.x);
        mObject.moveUp(coordinates.y);
    }

    /**
     * Update the scene camera based on the provided pose in Tango start of service frame.
     * The device pose should match the pose of the device at the time the last rendered RGB
     * frame, which can be retrieved with this.getTimestamp();
     * <p/>
     * NOTE: This must be called from the OpenGL render thread - it is not thread safe.
     */
    public void updateRenderCameraPose(TangoPoseData devicePose, DeviceExtrinsics extrinsics) {
        Pose cameraPose = ScenePoseCalculator.toOpenGlCameraPose(devicePose, extrinsics);
        getCurrentCamera().setRotation(cameraPose.getOrientation());
        getCurrentCamera().setPosition(cameraPose.getPosition());
    }

    /**
     * It returns the ID currently assigned to the texture where the Tango color camera contents
     * should be rendered.
     * NOTE: This must be called from the OpenGL render thread - it is not thread safe.
     */
    public int getTextureId() {
        return mTangoCameraTexture == null ? -1 : mTangoCameraTexture.getTextureId();
    }

    /**
     * We need to override this method to mark the camera for re-configuration (set proper
     * projection matrix) since it will be reset by Rajawali on surface changes.
     */
    @Override
    public void onRenderSurfaceSizeChanged(GL10 gl, int width, int height) {
        super.onRenderSurfaceSizeChanged(gl, width, height);
        mSceneCameraConfigured = false;
    }

    public boolean isSceneCameraConfigured() {
        return mSceneCameraConfigured;
    }

    /**
     * Sets the projection matrix for the scen camera to match the parameters of the color camera,
     * provided by the {@code TangoCameraIntrinsics}.
     */
    public void setProjectionMatrix(TangoCameraIntrinsics intrinsics) {
        Matrix4 projectionMatrix = ScenePoseCalculator.calculateProjectionMatrix(
                intrinsics.width, intrinsics.height,
                intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy);
        getCurrentCamera().setProjectionMatrix(projectionMatrix);
    }

    @Override
    public void onOffsetsChanged(float xOffset, float yOffset,
                                 float xOffsetStep, float yOffsetStep,
                                 int xPixelOffset, int yPixelOffset) {
    }

    /*
     * Method to calculate the amount the sphere should move when following the tango device.
     */
    private Vector3 calculateTravel(TangoPoseData mDevicePose){
        float dampening_Factor = OBJECT_THRESHOLD * OBJECT_SPEED;
        double ResultX, ResultY, ResultZ;

        Vector3 ObjCoord = mObject.getPosition();

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

    /*
    private boolean isLookingAtObject(Object3D object, TangoPoseData poseData) {

        Vector3 v = new Vector3(poseData.translation[0], poseData.translation[1], poseData.translation[3]);
        v.multiply(object.getModelViewMatrix());

        float pitch = (float) Math.atan2(v.y, -v.z);
        float yaw = (float) Math.atan2(v.x, -v.z);

        return (Math.abs(pitch) < ccIntrinsics.height) && (Math.abs(yaw) < ccIntrinsics.width);
    }
    */

    @Override
    public void onTouchEvent(MotionEvent event) {

    }

    public TangoPoseData getDevicePose() {
        return mDevicePose;
    }

    public void setDevicePose(TangoPoseData mDevicePose) {
        this.mDevicePose = mDevicePose;
    }
}
