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

/**
 * MODIFICATIONS TO HIDETOSHI'S OPTICAL FLOW
 * modified to use kinect camera image & optimised a fair bit as rgb calculations are not required - still needs work.
 * note class requires depth image from kinectH: kinectH.depthImg
 *
 **/

class KinectFlow {
  KinectHelper kinectH;
  boolean drawOpticalFlow=true; 
  boolean showSettings=true; 

  // A flow field is a two dimensional array of PVectors
  PVector[][] field;

  int cols, rows; // Columns and Rows
  int resolution; // How large is each "cell" of the flow field


  int avSize; //as;  // window size for averaging (-as,...,+as)
  float df;

  // regression vectors
  float[] fx, fy, ft;
  int fm=3*9; // length of the vectors

  // smoothing parameters
  float wflow= 0.05; //0.1;//0.05; // smaller value for longer smoothing 0.1

  // internally used variables
  float ar, ag, ab; // used as return value of pixave
  //float ag;  // used as return value of pixave greyscale
  float[] dtr, dtg, dtb; // differentiation by t (red,gree,blue)
  float[] dxr, dxg, dxb; // differentiation by x (red,gree,blue)
  float[] dyr, dyg, dyb; // differentiation by y (red,gree,blue)
  float[] par, pag, pab; // averaged grid values (red,gree,blue)
  float[] flowx, flowy; // computed optical flow
  float[] sflowx, sflowy; // slowly changing version of the flow
  int clockNow, clockPrev, clockDiff; // for timing check

  KinectFlow(int r, PApplet parent) {
    // instansiate helper class for kinect
    kinectH = new KinectHelper(parent);

    resolution = r;
    // Determine the number of columns and rows based on sketch's width and height
    cols = kinectH.kWidth/resolution;
    rows = kinectH.kHeight/resolution;
    field = new PVector[cols][rows];

    avSize=resolution*2;
    int fps=30;
    float predsec=0.5; // prediction time (sec): larger for longer vector 0.5

    df=predsec*fps;

    // arrays
    par = new float[cols*rows];
    pag = new float[cols*rows];
    pab = new float[cols*rows];
    dtr = new float[cols*rows];
    dtg = new float[cols*rows];
    dtb = new float[cols*rows];
    dxr = new float[cols*rows];
    dxg = new float[cols*rows];
    dxb = new float[cols*rows];
    dyr = new float[cols*rows];
    dyg = new float[cols*rows];
    dyb = new float[cols*rows];
    flowx = new float[cols*rows];
    flowy = new float[cols*rows];
    sflowx = new float[cols*rows];
    sflowy = new float[cols*rows];

    fx = new float[fm];
    fy = new float[fm];
    ft = new float[fm];

    init();
    update();
  }

  void init() {
    // Reseed noise so we get a new flow field every time
    //noiseSeed((int)random(10000));
    float xoff = 0;
    for (int i = 0; i < cols; i++) {
      float yoff = 0;
      for (int j = 0; j < rows; j++) {
        // Use perlin noise to get an angle between 0 and 2 PI
        float theta = map(noise(xoff, yoff), 0, 1, 0, TWO_PI);
        // Polar to cartesian coordinate transformation to get x and y components of the vector
        field[i][j] = new PVector(cos(theta), sin(theta));
        yoff += 0.1;
      }
      xoff += 0.1;
    }
  }

  void draw() {
    // updates the kinect raw depth + pixels
    kinectH.updateKinectDepth(showSettings);
    // updates the optical flow vectors from the kinecter depth image (want to update optical flow before particles)
    update();
    if (showSettings) {
      // display instructions for adjusting kinect depth image
      instructionScreen();
    }
  }

  void instructionScreen() {
    // show kinect depth image
    image(kinectH.depthImg, 0, 0); 

    // instructions under depth image in gray box
    fill(50);
    rect(0, 490, 640, 85);
    fill(255);
    text("Press keys 'a' and 'z' to adjust minimum depth: " + kinectH.minDepth, 5, 505);
    text("Press keys 's' and 'x' to adjust maximum depth: " + kinectH.maxDepth, 5, 520);

    text("> Adjust depths until you get a white silhouette of your whole body with everything else black.", 5, 550);
    text("PRESS SPACE TO CONTINUE", 5, 565);
  }

  void update() {
    //clockNow = millis();
    //clockDiff = clockNow - clockPrev;
    //clockPrev = clockNow; 

    difT();
    difXY();
    solveFlow();
    //drawColorFlow();
  }

  // calculate average pixel value (r,g,b) for rectangle region
  void pixaveGreyscale(int x1, int y1, int x2, int y2) {
    //float sumr,sumg,sumb;
    float sumg;
    color pix;
    //int r,g,b;
    float g;
    int n;

    if (x1<0) x1=0;
    if (x2>=kinectH.kWidth) x2=kinectH.kWidth-1;
    if (y1<0) y1=0;
    if (y2>=kinectH.kHeight) y2=kinectH.kHeight-1;

    //sumr=sumg=sumb=0.0;
    sumg = 0.0;
    for (int y=y1; y<=y2; y++) {
      for (int i=kinectH.kWidth*y+x1; i<=kinectH.kWidth*y+x2; i++) {

        // WORK WITH RAW DEPTH INSTEAD
        sumg += kinectH.rawDepth[i];
      }
    }
    n = (x2-x1+1)*(y2-y1+1); // number of pixels
    // the results are stored in static variables
    //ar = sumr/n; 
    //ag = sumg/n; 
    //ab = sumb/n;

    ar = sumg/n; 
    ag = ar; 
    ab = ar;
  }
  // TODO remove, not used
  // calculate average pixel value (r,g,b) for rectangle region
  void pixave(int x1, int y1, int x2, int y2) {
    float sumr, sumg, sumb;
    color pix;
    int r, g, b;
    int n;

    if (x1<0) x1=0;
    if (x2>=kinectH.kWidth) x2=kinectH.kWidth-1;
    if (y1<0) y1=0;
    if (y2>=kinectH.kHeight) y2=kinectH.kHeight-1;

    sumr=sumg=sumb=0.0;
    for (int y=y1; y<=y2; y++) {
      for (int i=kinectH.kWidth*y+x1; i<=kinectH.kWidth*y+x2; i++) {
        pix= kinectH.depthImg.pixels[i];
        b=pix & 0xFF; // blue
        pix = pix >> 8;
        g=pix & 0xFF; // green
        pix = pix >> 8;
        r=pix & 0xFF; // red
        //if( random(0, 150000) > 149000 && r > 0) println("r " + r + " b " + b + " g " + g);
        // averaging the values
        sumr += b;//r;//g;//r;
        sumg += b;//r;//g;
        sumb += b;//r;//g;//b;
      }
    }
    n = (x2-x1+1)*(y2-y1+1); // number of pixels
    // the results are stored in static variables
    ar = sumr/n; 
    ag=sumg/n; 
    ab=sumb/n;
  }

  // extract values from 9 neighbour grids
  void getnext9(float x[], float y[], int i, int j) {
    y[j+0] = x[i+0];
    y[j+1] = x[i-1];
    y[j+2] = x[i+1];
    y[j+3] = x[i-cols];
    y[j+4] = x[i+cols];
    y[j+5] = x[i-cols-1];
    y[j+6] = x[i-cols+1];
    y[j+7] = x[i+cols-1];
    y[j+8] = x[i+cols+1];
  }

  // solve optical flow by least squares (regression analysis)
  void solveSectFlow(int ig) {
    float xx, xy, yy, xt, yt;
    float a, u, v, w;

    // prepare covariances
    xx=xy=yy=xt=yt=0.0;
    for (int i=0;i<fm;i++) {
      xx += fx[i]*fx[i];
      xy += fx[i]*fy[i];
      yy += fy[i]*fy[i];
      xt += fx[i]*ft[i];
      yt += fy[i]*ft[i];
    }

    // regularization term for regression
    float fc=pow(10, 8); // larger values for noisy video

    // least squares computation
    a = xx*yy - xy*xy + fc; // fc is for stable computation
    u = yy*xt - xy*yt; // x direction
    v = xx*yt - xy*xt; // y direction

    // write back
    flowx[ig] = -2*resolution*u/a; // optical flow x (pixel per frame)
    flowy[ig] = -2*resolution*v/a; // optical flow y (pixel per frame)
  }

  void difT() {
    for (int ix=0;ix<cols;ix++) {
      int x0=ix*resolution+resolution/2;
      for (int iy=0;iy<rows;iy++) {
        int y0=iy*resolution+resolution/2;
        int ig=iy*cols+ix;
        // compute average pixel at (x0,y0)
        pixaveGreyscale(x0-avSize, y0-avSize, x0+avSize, y0+avSize);
        // compute time difference
        dtr[ig] = ar-par[ig]; // red
        //dtg[ig] = ag-pag[ig]; // green
        //dtb[ig] = ab-pab[ig]; // blue
        // save the pixel
        par[ig]=ar;
        //pag[ig]=ag;
        //pab[ig]=ab;
      }
    }
  }


  // 2nd sweep : differentiations by x and y
  void difXY() {
    for (int ix=1;ix<cols-1;ix++) {
      for (int iy=1;iy<rows-1;iy++) {
        int ig=iy*cols+ix;
        // compute x difference
        dxr[ig] = par[ig+1]-par[ig-1]; // red
        //dxg[ig] = pag[ig+1]-pag[ig-1]; // green
        //dxb[ig] = pab[ig+1]-pab[ig-1]; // blue
        // compute y difference
        dyr[ig] = par[ig+cols]-par[ig-cols]; // red
        //dyg[ig] = pag[ig+cols]-pag[ig-cols]; // green
        //dyb[ig] = pab[ig+cols]-pab[ig-cols]; // blue
      }
    }
  }



  // 3rd sweep : solving optical flow
  void solveFlow() {
    for (int ix=1;ix<cols-1;ix++) {
      int x0=ix*resolution+resolution/2;
      for (int iy=1;iy<rows-1;iy++) {
        int y0=iy*resolution+resolution/2;
        int ig=iy*cols+ix;
        //y0Z=iy*resolution+resolution/2;
        //igz=iy*cols+ix;

        // prepare vectors fx, fy, ft
        getnext9(dxr, fx, ig, 0); // dx red
        //getnext9(dxg,fx,ig,9); // dx green
        //getnext9(dxb,fx,ig,18);// dx blue
        getnext9(dyr, fy, ig, 0); // dy red
        //getnext9(dyg,fy,ig,9); // dy green
        //getnext9(dyb,fy,ig,18);// dy blue
        getnext9(dtr, ft, ig, 0); // dt red
        //getnext9(dtg,ft,ig,9); // dt green
        //getnext9(dtb,ft,ig,18);// dt blue

        // solve for (flowx, flowy) such that
        // fx flowx + fy flowy + ft = 0
        solveSectFlow(ig);

        // smoothing
        sflowx[ig]+=(flowx[ig]-sflowx[ig])*wflow;
        sflowy[ig]+=(flowy[ig]-sflowy[ig])*wflow;

        float u=df*sflowx[ig];
        float v=df*sflowy[ig];
        //float u=df*sflowx[ig];
        //float v=df*sflowy[ig];

        float a=sqrt(u*u+v*v);

        // setting both to the same value is intereseting
        float minDrawParticlesFlowVelocity = 20.0f;
        float minRegisterFlowVelocity = 2.0f;


        // register new vectors
        if (a>= minRegisterFlowVelocity) 
        {
          field[ix][iy] = new PVector(u, v);

          // REMOVED FROM drawColorFlow() to here
          if (a>=minDrawParticlesFlowVelocity) { 

            // display flow when debugging
            if (drawOpticalFlow) 
            {
              stroke(255.0f, 0.0f, 0.0f);
              line(x0, y0, x0+u, y0+v);
            } 

            // same syntax as memo's fluid solver (http://memo.tv/msafluid_for_processing)
            float kinectNormX = (x0+u) * kinectH.invKWidth;// / kWidth;
            float kinectNormY = (y0+v) * kinectH.invKHeight; // kHeight;
            float kinectVelX = ((x0+u) - x0) * kinectH.invKWidth;// / kWidth;
            float kinectVelY = ((y0+v) - y0) * kinectH.invKHeight;// / kHeight;         

            addForce(1-kinectNormX, kinectNormY, -kinectVelX, kinectVelY);
          }
        }
      }
    }
  }

  // TODO remove this function ... not used.
  // dont need to seperate from solveFlow
  void drawColorFlow() {
    /*
    for(int ix=0;ix<cols;ix++) {
     int x0=ix*resolution+resolution/2;
     //int lerpyCount = 0;
     for(int iy=0;iy<rows;iy++) {
     int y0=iy*resolution+resolution/2;
     int ig=iy*cols+ix;
     
     float u=df*sflowx[ig];
     float v=df*sflowy[ig];
     
     // draw the line segments for optical flow
     float a=sqrt(u*u+v*v);
     //if(a>=2.0) { // draw only if the length >=2.0
     if(a>=minRegisterFlowVelocity) { // draw only if the length >=2.0
     //float r=0.5*(1.0+u/(a+0.1));
     //float g=0.5*(1.0+v/(a+0.1));
     //float b=0.5*(2.0-(r+g));
     
     //stroke(255*r,255*g,255*b);
     
     // draw the optical flow field red!
     if (drawOpticalFlow) 
     {
     stroke(255.0f, 0.0f, 0.0f);
     line(x0,y0,x0+u,y0+v);
     }
     
     
     // same syntax as memo's fluid solver (http://memo.tv/msafluid_for_processing)
     float mouseNormX = (x0+u) * invKWidth;// / kWidth;
     float mouseNormY = (y0+v) * invKHeight; // kHeight;
     float mouseVelX = ((x0+u) - x0) * invKWidth;// / kWidth;
     float mouseVelY = ((y0+v) - y0) * invKHeight;// / kHeight;         
     
     addForce(1-mouseNormX, mouseNormY, -mouseVelX, mouseVelY);
     }
     }
     }
     */
  }
  // TODO remove this function ... not used.
  // Draw every vector
  void display() {
    for (int i = 0; i < cols; i++) {
      for (int j = 0; j < rows; j++) {
        drawVector(field[i][j], i*resolution, j*resolution, resolution-2);
      }
    }
  }

  // TODO remove this function ... not used.
  // Renders a vector object 'v' as an arrow and a location 'x,y'
  void drawVector(PVector v, float x, float y, float scayl) {
    pushMatrix();
    float arrowsize = 4;
    // Translate to location to render vector
    translate(x, y);
    stroke(100);
    // Call vector heading function to get direction (note that pointing up is a heading of 0) and rotate
    rotate(v.heading2D());
    // Calculate length of vector & scale it to be bigger or smaller if necessary
    float len = v.mag()*scayl;
    // Draw three lines to make an arrow (draw pointing up since we've rotate to the proper direction)
    line(0, 0, len, 0);
    line(len, 0, len-arrowsize, +arrowsize/2);
    line(len, 0, len-arrowsize, -arrowsize/2);
    popMatrix();
  }
  // what happens if the key is pressed. 
  void keyPressed(int key, int keyCode) {
    if (key == ' ') 
    { 
      println("KinectFlow space pressed");
      // space bar for settings to adjust kinect depth
      color bgColor = color(0);
      background(bgColor);
      // Inverse the value of boolean
      showSettings ^= true;
      drawOpticalFlow ^= true;
      println("showSettings " + showSettings);
      println("drawOpticalFlow " + drawOpticalFlow);
    }
    // call the kinect helper keyPressed
    kinectH.keyPressed(key, keyCode);
  }

  PVector lookup(PVector lookup) {
    int i = (int) constrain(lookup.x/resolution, 0, cols-1);
    int j = (int) constrain(lookup.y/resolution, 0, rows-1);
    return field[i][j].get();
  }


  void stop() {
    kinectH.quit();
  }
}

