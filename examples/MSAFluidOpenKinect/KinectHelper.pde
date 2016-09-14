/**
 * NOISE INK
 * Created by Trent Brooks, http://www.trentbrooks.com
 * Applying different forces to perlin noise via optical flow 
 * generated from kinect depth image. 
 *
 * CREDIT
 * Special thanks to Daniel Shiffman for the openkinect libraries 
 * (https://github.com/shiffman/libfreenect/tree/master/wrappers/java/processing)
 * Generative Gestaltung (http://www.generative-gestaltung.de/) for 
 * perlin noise articles. Patricio Gonzalez Vivo ( http://www.patriciogonzalezvivo.com )
 * & Hidetoshi Shimodaira (shimo@is.titech.ac.jp) for Optical Flow example
 * (http://www.openprocessing.org/visuals/?visualID=10435). 
 * Memotv (http://www.memo.tv/msafluid_for_processing) for inspiration.
 * 
 * Creative Commons Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0)
 * http://creativecommons.org/licenses/by-sa/3.0/
 *
 *
 **/

import org.openkinect.*;
import org.openkinect.processing.*;


class KinectHelper {

  Kinect kinect;
  int kWidth  = 640; // TODO these could be removed and use displayWidth displayHight
  int kHeight = 480;
  float invKWidth = 1.0f/kWidth;
  float invKHeight = 1.0f/kHeight;

  int kAngle = -10; // angle to tilt the kinect 
  boolean isKinected = false;
  int[] rawDepth;
  int minDepth = 600;
  int maxDepth = 800;//995;//982;//818;//860;
  int thresholdRange = 2047;

  PImage depthImg;

  public KinectHelper(PApplet parent) 
  {
    try {
      kinect = new Kinect(parent);
      kinect.initDepth();
      //kinect.start();
      //kinect.enableDepth(true);
      kinect.setTilt(kAngle);

      //kinect.processDepthImage(false);

      isKinected = true;
      println("KINECT IS INITIALISED");
    } 
    catch (Throwable t) {
      isKinected = false;
      println("KINECT NOT INITIALISED");
    }

    depthImg = new PImage(kWidth, kHeight);
    rawDepth = new int[kWidth*kHeight];
  }

  public void updateKinectDepth(boolean updateDepthPixels)
  {
    // quick exit if Kinect is not connected
    if (!isKinected) return;

    // checks raw depth of kinect: if within certain depth range - color everything white, else black
    rawDepth = kinect.getRawDepth();
    for (int i=0; i < kWidth*kHeight; i++) {
      if (rawDepth[i] >= minDepth && rawDepth[i] <= maxDepth) {
        if (updateDepthPixels) depthImg.pixels[i] = 0xFFFFFFFF;
        rawDepth[i] = 255;
      } 
      else {
        if (updateDepthPixels) depthImg.pixels[i] = 0;
        rawDepth[i] = 0;
      }
    }

    // update the thresholded image
    if (updateDepthPixels) depthImg.updatePixels();
    //image(depthImg, kWidth, 0);
  }

  public void tiltUp() {
    kAngle++;
    tilt();
  }
  public void tiltDown() {
    kAngle--;
    tilt();
  }
  private void tilt() {
    int maxAngle = 30;
    int minAngle = -30;
    kAngle = constrain(kAngle, minAngle, maxAngle);
    kinect.setTilt(kAngle);
  }

  private void setMinDepth(int deltaDepth) {
    minDepth = constrain(minDepth + deltaDepth, 0, thresholdRange);
    println("minimum depth: " + minDepth);
  }
  
  private void setMaxDepth(int deltaDepth) {
    maxDepth = constrain(maxDepth + deltaDepth, 0, thresholdRange);
    println("maximum depth: " + maxDepth);
  }


/**
 * CONTROLS
 * Up and Down tilts the kinect
 * a,z = adjust minimum kinect depth
 * s,x = adjust maximum kinect depth
 **/
  void keyPressed(int key, int keyCode) {
    println("*** KinectHelp called: " + key);
    if (key == CODED) {
      if (keyCode == UP) {
        tiltUp();
      } 
      else if (keyCode == DOWN) {
        tiltDown();
      }
    } 
    else {
      int depthDelta = 10;
      if (key == 'a' || key == 'A') // 65
      {
        // a pressed add to minimum depth
        setMinDepth(depthDelta);
      }
      else if (key == 'z' || key == 'Z') // 90
      {
        // z pressed subtract to minimum depth
        setMinDepth(-depthDelta);
      }
      else if (key == 's' || key == 'S') // 83
      {
        // s pressed add to maximum depth
        setMaxDepth(depthDelta);
      }
      else if (key ==  'x' || key == 'X') // 88
      {
        // x pressed subtract to maximum depth
        setMaxDepth(-depthDelta);
      }
    }
  }

  public void quit() {
    if (isKinected) 
      kinect.stopDepth();
  }
}