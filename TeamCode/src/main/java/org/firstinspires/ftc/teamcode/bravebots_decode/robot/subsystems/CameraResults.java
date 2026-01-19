package org.firstinspires.ftc.teamcode.bravebots_decode.robot.subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.concurrent.locks.ReentrantLock;

public class CameraResults implements SubsystemInterface {

    public static OpenCvCamera camera;
    public static volatile boolean opened, starting, streaming; //??
    volatile Mat currentFrame;
    private static final int WIDTH= 960, HEIGHT= 600, MINIMUM_PIXELS= 20000;
    //    static Mat hsv;
//    static Mat green, purple;
    public static ReentrantLock lock;

    @Deprecated
    public static CameraResults getInstance(){
        return null;
    }

    static Scalar lowerGreen = new Scalar(40, 40, 40);
    static Scalar upperGreen = new Scalar(80, 255, 255);
    static Scalar lowerPurple = new Scalar(125, 40, 40);
    static Scalar upperPurple = new Scalar(155, 255, 255);

    static volatile boolean hasPurple, hasGreen;
    public static double greenPixels, purplePixels;
    public volatile static boolean running;

    static boolean set= false;
    public static synchronized  void start(){

        try{
            new Thread(()->{


                lock= new ReentrantLock();
                CameraResults instance= new CameraResults();
                //CameraLogic instance= getInstance();
                running= true;
                instance.currentFrame= new Mat();
                instance.hsv= new Mat();
                instance.green= new Mat();
                instance.purple= new Mat();




                camera.setPipeline(new OpenCvPipeline() {
                    @Override
                    public Mat processFrame(Mat input) {
                        lock.lock();
                        try {
                            input.copyTo(instance.currentFrame);
                            //instance.update(); //?
                        }
                        finally {
                            lock.unlock();
                        }

                        return input;
                    }
                });

                camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        opened = true;
                        if (!running) { // stop a fost cerut înainte să se deschidă
                            starting = false;
                            android.util.Log.i("opened", "yes");
                            return;
                        }
                        camera.startStreaming(960, 600);
                        streaming = true;
                        starting = false;
                    }
                    //640 480

                    @Override
                    public void onError(int errorCode) {
                        opened = false;
                        streaming = false;
                        running = false;
                        starting = false;
                        android.util.Log.e("eroare", Integer.toString(errorCode) );

                    }
                });

                while (running) {

                    try {

                        instance.update();
                        Thread.sleep(20);
                    }catch (InterruptedException e){
                        Thread.currentThread().interrupt();
                    }

                }
            }).start();

        }
        catch (Throwable t){
            android.util.Log.e("EROARE",  "sincer tot confuz sunt", t);
        }







    }

    public static synchronized void stop(){
        running = false;
        starting = false;

        if (camera == null) return;

        if (opened && streaming) {
            try { camera.stopStreaming(); } catch (Exception ignored) {}
            streaming = false;
        }
        //camera.closeCameraDevice();
        opened = false;

    }

    public volatile  Mat hsv;
    public volatile  Mat purple;
    public volatile  Mat green;
    public volatile  Mat local;
    public static Mat q,w,e,r,t,y;



    public static void initializeRandomMats(){
        q= new Mat();
        w=new Mat();
        e= new Mat();
        r= new Mat();
        t= new Mat();
        y= new Mat();
    }

    public static boolean oneUpdate= false;
    public  synchronized void update() {


        if(local== null)
            local= new Mat();



        lock.lock();
        try {
            if (currentFrame== null || currentFrame.empty()) return;
            currentFrame.copyTo(local);
        } finally {
            lock.unlock();
        }

        Imgproc.cvtColor(local, hsv, Imgproc.COLOR_BGR2HSV);
        Core.inRange(hsv, lowerGreen, upperGreen, green);
        Core.inRange(hsv, lowerPurple, upperPurple, purple);

        greenPixels= Core.countNonZero(green);
        purplePixels= Core.countNonZero(purple);
        if (greenPixels > purplePixels) {
            hasGreen  = greenPixels  > MINIMUM_PIXELS;
            hasPurple = false;
        } else {
            hasPurple = purplePixels > MINIMUM_PIXELS;
            hasGreen  = false;
        }
//        hasGreen  = greenPixels > MINIMUM_PIXELS;
//        hasPurple = purplePixels > MINIMUM_PIXELS;

        // local.release();




    }

    public static synchronized  boolean isGreen(){
//        if(!running)
//        {
//            throw new RuntimeException("Camera nu streameaza");
//        }
        return hasGreen;
    }
    public static synchronized  boolean isPurple(){
//        if(!running)
//        {
//            throw new RuntimeException("Camera nu streameaza");
//        }
        return hasPurple;
    }
}