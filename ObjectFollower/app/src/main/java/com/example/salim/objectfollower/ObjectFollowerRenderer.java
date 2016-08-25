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

import java.util.ArrayList;
import java.util.Iterator;
import java.util.Random;
import java.util.concurrent.ThreadLocalRandom;

/**
 * Very simple example augmented reality renderer which displays a cube fixed in place.
 * The position of the cube in the OpenGL world is updated using the {@code updateObjectPose}
 * method.
 */
public class ObjectFollowerRenderer extends RajawaliRenderer {
    private static final String TAG = ObjectFollowerRenderer.class.getSimpleName();

    // Augmented Reality related fields
    private ATexture mTangoCameraTexture;
    private boolean mSceneCameraConfigured;

    //private Object3D mObject;
    private ArrayList<Object3D> mObjects = new ArrayList<Object3D>();
    
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
        
        mObjects.add(new Sphere(0.075f,24,24));
        mObjects.get(0).setMaterial(material);
        mObjects.get(0).setPosition(0.5,0,-3);
        mObjects.get(0).setRotation(Vector3.Axis.Z,180);
        getCurrentScene().addChild(mObjects.get(0));


        mObjects.add(new Sphere(0.075f,24,24));
        mObjects.get(1).setMaterial(material);
        mObjects.get(1).setPosition(-0.5,0,-3);
        mObjects.get(1).setRotation(Vector3.Axis.Z, 180);
        getCurrentScene().addChild(mObjects.get(1));

    }

    @Override
    protected void onRender(long elapsedRealTime, double deltaTime) {
        // Update the AR object if necessary
        // Synchronize against concurrent access with the setter below.
        synchronized (this) {
            if (mObjectPoseUpdated) {

                double start = -2;
                double end = 2;
                Random random = new Random();
                // Place the 3D object in the location of the detected point.
                mObjects.get(0).setPosition(start + (random.nextDouble() * (end - start)), start + (random.nextDouble() * (end - start)), start + (random.nextDouble() * (end - start)));
                mObjects.get(0).setOrientation(new Quaternion(0, 0, 0, 0));

                mObjects.get(1).setPosition(start + (random.nextDouble() * (end - start)), start + (random.nextDouble() * (end - start)), start + (random.nextDouble() * (end - start)));
                mObjects.get(1).setOrientation(new Quaternion(0, 0, 0, 0));

                mObjectPoseUpdated = false;
            }
        }
        super.onRender(elapsedRealTime, deltaTime);
    }

    /**
     * Save the updated plane fit pose to update the AR object on the next render pass.
     * This is synchronized against concurrent access in the render loop above.
     */
    public synchronized void spawnObjects() {
        mObjectPoseUpdated = true;
    }

    public synchronized void moveSphere(TangoPoseData currentPose, int index){

        Vector3 coordinates = MovementExtrinsics.getInstance().calculateTravel(currentPose, mObjects.get(index).getPosition());
        mObjects.get(index).moveForward(coordinates.z);
        mObjects.get(index).moveRight(coordinates.x);
        mObjects.get(index).moveUp(coordinates.y);

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

    @Override
    public void onTouchEvent(MotionEvent event) {

    }

    public TangoPoseData getDevicePose() {
        return mDevicePose;
    }

    public void setDevicePose(TangoPoseData mDevicePose) {
        this.mDevicePose = mDevicePose;
    }

    public Vector3 getObjectPose(int index) {
        return mObjects.get(index).getPosition();
    }

    public int getNumberofObjects(){
        return mObjects.size();
    }
}
