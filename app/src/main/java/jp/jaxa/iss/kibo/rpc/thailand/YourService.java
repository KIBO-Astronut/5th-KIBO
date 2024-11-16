package jp.jaxa.iss.kibo.rpc.thailand;

import gov.nasa.arc.astrobee.Kinematics;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.util.Log;
import java.util.*;
import gov.nasa.arc.astrobee.Result;
import android.os.SystemClock;

import org.opencv.aruco.*;
import org.opencv.calib3d.Calib3d;
import gov.nasa.arc.astrobee.types.Quaternion;
import org.opencv.core.Mat;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.aruco.Dictionary;
import org.tensorflow.lite.support.image.TensorImage;
import org.tensorflow.lite.task.vision.detector.Detection;
import org.tensorflow.lite.task.vision.detector.ObjectDetector;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.lang.Math;


public class YourService extends KiboRpcService {
    public Quaternion computeQuaternionFromAngles(List<Double> degrees) { //meiji 36-222//
        QuaternionUtils.computeQuaternionFromAngles(degrees);
        return new Quaternion( QuaternionUtils.computeQuaternionFromAngles(degrees).getX(),QuaternionUtils.computeQuaternionFromAngles(degrees).getY(),QuaternionUtils.computeQuaternionFromAngles(degrees).getZ(),QuaternionUtils.computeQuaternionFromAngles(degrees).getW());
    }
    @Override
    protected void runPlan1(){
        Map<Integer, gov.nasa.arc.astrobee.types.Point> dictionary = new HashMap<Integer, gov.nasa.arc.astrobee.types.Point>();
        dictionary.put(1, new gov.nasa.arc.astrobee.types.Point(10.66,-9.79,4.905));//Tar1
        dictionary.put(2, new gov.nasa.arc.astrobee.types.Point(10.91,-8.875,4.55));//Tar2
        dictionary.put(3, new gov.nasa.arc.astrobee.types.Point(10.535,-7.88,4.55));//Tar3
        dictionary.put(4, new gov.nasa.arc.astrobee.types.Point(10.66,-6.8525,4.95)); //Tar4
        dictionary.put(5, new gov.nasa.arc.astrobee.types.Point(10.91,-7.925,4.55));
        dictionary.put(10, new gov.nasa.arc.astrobee.types.Point(11.005,-6.808,4.965));//Area4 As
        dictionary.put(99, new gov.nasa.arc.astrobee.types.Point(10.785,-9.25,4.65));
        Map<Integer, Quaternion> orientations = new HashMap<Integer, Quaternion>();
        orientations.put(1, new Quaternion(-0.009f, -0.224f,  -0.455f,  0.862f));
        orientations.put(2, new Quaternion( 0.000f,  0.707f,  0.000f, 0.707f));
        orientations.put(3, new Quaternion(0f,  0.609f,  0f,   0.793f));
        orientations.put(4, new Quaternion(  0.009f, -0.001f,  -0.996f,  0.087f));
        orientations.put(5, new Quaternion(0f,  0.609f,  0f,   0.793f));
        orientations.put(   10, new Quaternion(0f, 0f, -0.707f, 0.707f));
        orientations.put(11, new Quaternion(-0.105f,  0.000f,   -0.995f,   0f));

        api.startMission();
        List<List<Object>> output = new ArrayList<>();


        //moveToWrapper(11.035d,-9.6d,5.1d,-0.123f, -0.123f, -0.696f,  0.696f);
        moveToWrapper(10.985d,-9.6d,5.1d,-0.123f, -0.123f, -0.696f,  0.696f); //old_path
        ARResult check_tar1 =AR_cropping(1,0);
        List<List<Double>> angle_list = new ArrayList<>();
        Kinematics pos_now = api.getRobotKinematics();
        Log.i("Confident", String.valueOf(pos_now.getConfidence()));
        Log.i("Position", String.valueOf(pos_now.getPosition()));
        angle_list.add(stab(1,0));
        //moveToWrapper(11.035d,-9.4d,5.1d,0f, 0f, -0.707f, 0.707f);

        if(check_tar1.arComplete==0){
            moveToWrapper(11.035d,-9.4d,5.1d,0f, 0f, -0.707f, 0.707f);
            AR_cropping(1,0);
        }
        int check_tar2_2=0;

        //moveToWrapper(10.91,-8.84,4.645, 0.500f,  0.500f,  -0.500f, 0.500f);
        moveToWrapper(10.925d, -8.49, 4.867, 0.500f,  0.500f,  -0.500f, 0.500f); //old_path
        try {
            output.add(wrapper_predict(1));
        } catch (IOException e) {
            Log.i("IMAGE PREDICT ERROR:","IMAGE 1 CATCH ERROR.");
            Log.i("ERROR1",String.valueOf(e));
        }
        Mat emr_1 =api.getMatNavCam();
        api.saveMatImage(emr_1,"EMR-2.png");
        SystemClock.sleep(3000);
        ARResult check_tar2 = AR_cropping(2,0);

        pos_now = api.getRobotKinematics();
        Log.i("Confident", String.valueOf(pos_now.getConfidence()));
        Log.i("Position", String.valueOf(pos_now.getPosition()));
        if(check_tar2.arComplete==2) {
            try {
                output.add(wrapper_predict(2));
            } catch (IOException e) {
                Log.i("IMAGE PREDICT ERROR:", "IMAGE 2 CATCH ERROR.");
                Log.i("ERROR1", String.valueOf(e));
            }
            //moveToWrapper(10.91,-7.84,4.645, 0.500f,  0.500f,  -0.500f, 0.500f);
            // check_tar3 = AR_cropping(3, 0);
            pos_now = api.getRobotKinematics();
            Log.i("Confident", String.valueOf(pos_now.getConfidence()));
            Log.i("Position", String.valueOf(pos_now.getPosition()));
        }else{
            check_tar2_2=2;
        }
        ARResult check_tar3 = AR_cropping(3,0);
        if(check_tar2.detectedIdsCount < 2 || check_tar3.detectedIdsCount < 2){
            SystemClock.sleep(1500);
            check_tar2 = AR_cropping(2,0);
            check_tar3 = AR_cropping(3,0);
            if(check_tar2.detectedIdsCount < 2 || check_tar3.detectedIdsCount < 2){
                handleMissing();
                moveToWrapper(10.635d,-6.808d,4.71,-0.105f,  0.000f,   -0.995f,   0f);
            }else{
                moveToWrapper(11.01d,-6.805d,5.2d,-0.017f,  0.001f,   0.999f,   0.044f);


            }
        }
        else{
            moveToWrapper(11.01d,-6.805d,5.2d,-0.017f,  0.001f,   0.999f,   0.044f);
        }




        //moveToWrapper(11.003d,-6.808d,5.212d,-0.105f,  0.000f,   -0.995f,   0f); //old_path
        //moveToWrapper(10.635d,-6.808d,4.71,-0.105f,  0.000f,   -0.995f,   0f);
        if(check_tar2.arComplete==2 && check_tar2_2==2) {
            try {
                output.add(wrapper_predict(2));
            } catch (IOException e) {
                Log.i("IMAGE PREDICT ERROR:", "IMAGE 2 CATCH ERROR.");
                Log.i("ERROR1", String.valueOf(e));
            }
            //moveToWrapper(10.91,-7.84,4.645, 0.500f,  0.500f,  -0.500f, 0.500f);
            // check_tar3 = AR_cropping(3, 0);
            pos_now = api.getRobotKinematics();
            Log.i("Confident", String.valueOf(pos_now.getConfidence()));
            Log.i("Position", String.valueOf(pos_now.getPosition()));
        }
        try {
            output.add(wrapper_predict(3));
        } catch (IOException e) {
            Log.i("IMAGE PREDICT ERROR:","IMAGE 3 CATCH ERROR.");
            Log.i("ERROR1",String.valueOf(e));
        }


        ARResult check_tar4=AR_cropping(4,0);
        pos_now = api.getRobotKinematics();
        Log.i("Confident", String.valueOf(pos_now.getConfidence()));
        Log.i("Position", String.valueOf(pos_now.getPosition()));

        if(check_tar4.arComplete==0){
            moveToWrapper(dictionary.get(10), orientations.get(11));
            AR_cropping(4,0);
            moveToWrapper(dictionary.get(10), orientations.get(10));
        }
        if(check_pos()>0.3) {
            moveToWrapper(dictionary.get(10), orientations.get(10));
        }else{
            moveToWrapper(11.01d,-6.805d,5.2d,-0.031f, -0.031f, -0.706f, 0.706f);
        }
        try {
            output.add(wrapper_predict(4));
        } catch (IOException e) {
            Log.i("IMAGE PREDICT ERROR:","IMAGE 4 CATCH ERROR.");
            Log.i("ERROR1",String.valueOf(e));
        }
        api.setAreaInfo(1, (String) output.get(0).get(0), (Integer) output.get(0).get(1));
        api.setAreaInfo(2, (String) output.get(1).get(0), (Integer) output.get(1).get(1));
        api.setAreaInfo(3, (String) output.get(2).get(0), (Integer) output.get(2).get(1));
        api.setAreaInfo(4, (String) output.get(3).get(0), (Integer) output.get(3).get(1));
        api.reportRoundingCompletion();

        angle_list.add(stab(2,0));
        angle_list.add(stab(3,0));
        angle_list.add(stab(4,0));
        angle_list.add(stab(5,0));
        Mat image6 =new Mat();
        for(int m =0; m<20 ;m++) {
            image6 = api.getMatDockCam();
            api.saveMatImage(image6,"5(Nav).png");
            Mat TargetImage = api.getMatDockCam();
            Dictionary arucoDict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
            List<Mat> cornersList = new ArrayList<>();
            Mat ids = new Mat();
            Aruco.detectMarkers(TargetImage, arucoDict, cornersList, ids);
            Log.i("AR_pic:", "Dictionary DONE");

            if (!ids.empty()) {
                SystemClock.sleep(3000);
                AR_cropping(5,0);
                break;
            }else{
                SystemClock.sleep(500);
                Log.i("AR_pic:", "try:"+m);
                if(m==19){
                    if(AR_cropping(5,1).arComplete==0){
                        moveToWrapper(dictionary.get(10), orientations.get(10));
                        AR_cropping(5,0);
                    }
                }

            }

        }
        try {
            output.add(wrapper_predict(5));
        } catch (IOException e) {
            Log.i("IMAGE PREDICT ERROR:","IMAGE 5 CATCH ERROR.");
            Log.i("ERROR5",String.valueOf(e));
        }



        api.notifyRecognitionItem();  //meiji 36-222//



        int MoreThanOne = MoreThanOne(output);
        boolean Passed = false;
        if(MoreThanOne == 0){
            Passed = true;
            report(output,dictionary,angle_list);
        }
        if(Objects.equals(output.get(4).get(0),"Not Found")){
            Passed = true;
            handleAstroError(output,dictionary,angle_list);
        }
        if(MoreThanOne ==1){
            handleRepetitiveObj(output,dictionary,angle_list);
        }
        else if(!Passed || MoreThanOne == 2){
            handleAstroError(output,dictionary,angle_list);
        }


        /* ********************************************************** */
        /* Write your code to recognize which item the astronaut has. */
        /* ********************************************************** */

        // Let's notify the astronaut when you recognize it.


        /* ******************************************************************************************************* */
        /* Write your code to move Astrobee to the location of the target item (what the astronaut is looking for) */
        /* ******************************************************************************************************* */

        // Take a snapshot of the target item.

    }



    @Override
    protected void runPlan2(){
    }
    @Override
    protected void runPlan3(){
    }
    private boolean checker() {

        return false;
    }
    private void handleMissing(){
        moveToWrapper(10.91,-8.84,4.645, 0.500f,  0.500f,  -0.500f, 0.500f);
        AR_cropping(2,0);
        moveToWrapper(10.91,-7.84,4.645, 0.500f,  0.500f,  -0.500f, 0.500f);
        AR_cropping(3,0);
    }
    private double check_pos(){ //meiji//
        double distance = 0.0;
        Kinematics kinematics = api.getRobotKinematics();
        gov.nasa.arc.astrobee.types.Point pos = kinematics.getPosition();;
        distance= Math.sqrt((Math.pow(11.143-pos.getX(),2))+(Math.pow(-6.7607-pos.getY(),2))+(Math.pow(4.9654-pos.getZ(),2)));
        Log.i("distance_as:", String.valueOf(distance));
        return distance;
    }
    private boolean check_con(int target){ //meiji//
        boolean check = false;
        Kinematics kinematics = api.getRobotKinematics();
        gov.nasa.arc.astrobee.types.Point pos = kinematics.getPosition();
        if(target==99){
            if(pos.getY()<-9.68 && pos.getZ()>4.2){
                check = true;
            }
        }
        Log.i("distance_as:", String.valueOf(check));
        return check;
    }

    private void report(List<List<Object>> output, Map<Integer ,gov.nasa.arc.astrobee.types.Point> dictionary,List<List<Double>> angle_list){
        int foundTarget = 0;
        List<Integer> notFoundTarget= new ArrayList<>();
        int NotFoundCount = NotFoundCheck(output);

        for (int areaK = 0; areaK < 4; areaK++) {
            String currentItem = (String) output.get(areaK).get(0);
            String targetItem = (String) output.get(4).get(0);

            if (Objects.equals(currentItem, targetItem)) {
                logAndMove(areaK,dictionary,angle_list);
                foundTarget = 1;
                api.takeTargetItemSnapshot();
                break;
            } else if ("Not Found".equals(currentItem)) {
                notFoundTarget.add(areaK);
                logNotFound(areaK, currentItem);
            }
        }

        if (foundTarget == 0) {
            if (NotFoundCount == 1){
                handleNotFound(notFoundTarget,dictionary,angle_list);
            }
            else{
                goNearestArea(notFoundTarget,dictionary,angle_list);
            }
        }
    }
    private int NotFoundCheck(List<List<Object>> output){
        int NotFoundcount = 0;

        for (int areaK = 0; areaK < 4; areaK++) {
            String currentItem = (String) output.get(areaK).get(0);

            if ("Not Found".equals(currentItem)) {
                NotFoundcount++;
            }
        }
        for (int areaK = 0; areaK < 4; areaK++) {
            String currentItem = (String) output.get(areaK).get(0);
            String targetItem = (String) output.get(4).get(0);

            if (Objects.equals(currentItem, targetItem)){
                NotFoundcount++;
            }
        }
        return NotFoundcount;
    }
    private void handleAstroError(List<List<Object>> output, Map<Integer ,gov.nasa.arc.astrobee.types.Point> dictionary,List<List<Double>> angle_list){

        moveToWrapper(dictionary.get(4), eulerToQuaternion(angle_list.get(3)));
        Mat image7 = api.getMatNavCam();
        api.saveMatImage(image7, "target.png");
        Mat image8 = api.getMatNavCam();
        api.saveMatImage(image8, "LOCKON.png"); //use when post_5 no result
    }
    private void handleRepetitiveObj(List<List<Object>> output, Map<Integer ,gov.nasa.arc.astrobee.types.Point> dictionary,List<List<Double>> angle_list){
        List<Integer> RepetitiveObj= new ArrayList<>();
        for (int areaK = 0; areaK < 4; areaK++) {
            String currentItem = (String) output.get(areaK).get(0);
            String targetItem = (String) output.get(4).get(0);
            if (Objects.equals(currentItem, targetItem)) {
                RepetitiveObj.add(areaK);
            }
        }
        goNearestArea(RepetitiveObj,dictionary,angle_list);
    }
    private int MoreThanOne(List<List<Object>> output){
        int count = 0;
        int check =0;
        for (int areaK = 0; areaK < 4; areaK++) {
            String currentItem = (String) output.get(areaK).get(0);
            String targetItem = (String) output.get(4).get(0);
            if (Objects.equals(currentItem, targetItem)){
                count++;

            }
            for (int j =0; j<4;j++){
                if (Objects.equals(output.get(areaK).get(0), output.get(j).get(0))){
                    check++;

                }
            }
        }
        Log.i("counting","Same_Count : "+count);
        if(count == 1){
            return 0;
        }
        else if(count>1){
            return 1;
        }
        else{
            return 2;
        }



    }
    private void logAndMove(int areaK , Map<Integer ,gov.nasa.arc.astrobee.types.Point> dictionary,List<List<Double>> angle_list) {
        Log.i("Predict_result", "item_index :" + areaK);

        Mat image7 = api.getMatNavCam();
        api.saveMatImage(image7, "target.png");

        if (areaK + 1 == 1) {
            moveToWrapper(dictionary.get(99), eulerToQuaternion(angle_list.get(areaK)));
        }
        moveToWrapper(dictionary.get(areaK + 1), eulerToQuaternion(angle_list.get(areaK)));
        if(check_pos()<0.4){
            moveToWrapper(11.15d,-7.55d,5.25d,0f, 0f, -0.707f, 0.707f);
            if (areaK + 1 == 1) {
                moveToWrapper(dictionary.get(2), eulerToQuaternion(angle_list.get(areaK)));
            }
            if (areaK + 1 == 3) {
                areaK=4;
            }
            moveToWrapper(dictionary.get(areaK + 1), eulerToQuaternion(angle_list.get(areaK)));
        }
        Kinematics pos_now = api.getRobotKinematics();
        Log.i("Confident_main", String.valueOf(pos_now.getConfidence()));
        Log.i("Position", String.valueOf(pos_now.getPosition()));
        SystemClock.sleep(3000);
        moveToWrapper(dictionary.get(areaK + 1), computeQuaternionFromAngles(Final_turn(areaK + 1,0)));
        Mat image8 = api.getMatNavCam();
        api.saveMatImage(image8, "LOCKON.png");
    }

    private void logNotFound(int areaK, String currentItem) {
        Log.i("Predict_result", "Not Found");
        Log.i("Predict_result", currentItem);
        Log.i("Predict_result", "current_index :" + areaK);
    }

    private void handleNotFound(List<Integer> notFoundTarget ,Map<Integer, gov.nasa.arc.astrobee.types.Point> dictionary,List<List<Double>> angle_list) {
        Mat image7 = api.getMatNavCam();
        api.saveMatImage(image7, "target.png");
        int target=notFoundTarget.get(0);
        if (notFoundTarget.get(0) + 1 == 1) {
            api.getRobotKinematics();
            moveToWrapper(dictionary.get(99), eulerToQuaternion(angle_list.get(notFoundTarget.get(0))));
        }
        moveToWrapper(dictionary.get(notFoundTarget.get(0) + 1), eulerToQuaternion(angle_list.get(notFoundTarget.get(0))));
        if(check_pos()<0.4) {
            moveToWrapper(11.15d,-7.55d,5.25d,0f, 0f, -0.707f, 0.707f);
            if (notFoundTarget.get(0) + 1 == 1) {
                api.getRobotKinematics();
                moveToWrapper(dictionary.get(2), eulerToQuaternion(angle_list.get(notFoundTarget.get(0))));
            }
            if(notFoundTarget.get(0)==2){
                target=4;
            }

            moveToWrapper(dictionary.get(target+1), eulerToQuaternion(angle_list.get(target)));
        }
        Kinematics pos_now = api.getRobotKinematics();
        Log.i("Confident_main", String.valueOf(pos_now.getConfidence()));
        Log.i("Position", String.valueOf(pos_now.getPosition()));
        SystemClock.sleep(3000);

        moveToWrapper(dictionary.get(notFoundTarget.get(0)  + 1), computeQuaternionFromAngles(Final_turn(notFoundTarget.get(0)  + 1,0)));
        Mat image8 = api.getMatNavCam();
        api.saveMatImage(image8, "LOCKON.png");
        api.takeTargetItemSnapshot();
    }



    private void goNearestArea(List<Integer> notFoundTarget ,Map<Integer, gov.nasa.arc.astrobee.types.Point> dictionary,List<List<Double>> angle_list){
        int lastIndex = notFoundTarget.get(notFoundTarget.size() - 1);
        Mat image7 = api.getMatNavCam();
        api.saveMatImage(image7, "target.png");

        if (lastIndex + 1 == 1) {
            api.getRobotKinematics();
            moveToWrapper(dictionary.get(99), eulerToQuaternion(angle_list.get(lastIndex)));
        }
        moveToWrapper(dictionary.get(lastIndex + 1), eulerToQuaternion(angle_list.get(lastIndex)));
        if(check_pos()<0.4) {
            moveToWrapper(11.15d,-7.55d,5.25d,0f, 0f, -0.707f, 0.707f);
            if (lastIndex + 1 == 1) {
                api.getRobotKinematics();
                moveToWrapper(dictionary.get(2), eulerToQuaternion(angle_list.get(lastIndex)));
            }
            if(lastIndex+1==3){
                lastIndex=4;
            }
            moveToWrapper(dictionary.get(lastIndex + 1), eulerToQuaternion(angle_list.get(lastIndex)));
        }
        Kinematics pos_now = api.getRobotKinematics();
        Log.i("Confident_main", String.valueOf(pos_now.getConfidence()));
        Log.i("Position", String.valueOf(pos_now.getPosition()));
        SystemClock.sleep(3000);
        moveToWrapper(dictionary.get(lastIndex + 1), computeQuaternionFromAngles(Final_turn(lastIndex + 1,0)));
        Mat image8 = api.getMatNavCam();
        api.saveMatImage(image8, "LOCKON.png");
        api.takeTargetItemSnapshot();
    }




    public List<Double> stab(int target_num, int emr) { //meiji//
        List<Double> changeAngle = new ArrayList<>();
        double changeof_y = 0.0;

        if (target_num == 3) {
            //    changeof_y=(center_x-430)/8;
            Log.i("AR_pic(coordinate):", Double.toString(changeof_y));
            changeAngle.add(0.0);
            changeAngle.add(75.0);
            changeAngle.add(90.0);
        }else if(target_num == 1){

            changeAngle.add(-90.000);
            changeAngle.add(-25.000);
            changeAngle.add(0.000);

        }else if(target_num == 2){
            // changeof_y = ((center_x-555) / 13.5);
            Log.i("AR_pic(coordinate):", Double.toString(changeof_y));
            changeAngle.add(0.000);
            changeAngle.add(90.0);
            changeAngle.add(90.0);
        }
        else if(target_num == 5){
            // changeof_y = ((center_x-555) / 13.5);
            Log.i("AR_pic(coordinate):", Double.toString(changeof_y));
            changeAngle.add(0.000);
            changeAngle.add(90.0);
            changeAngle.add(90.0);
        }else if(target_num == 4){
            //  changeof_y=((center_y-450)/6.5);
            Log.i("AR_pic(coordinate):", Double.toString(changeof_y));
            changeAngle.add(-180.000);
            changeAngle.add(0.0);
            changeAngle.add(0.0);
        }



        return changeAngle;

    }
    private boolean moveToWrapper(double pos_x, double pos_y, double pos_z,
                                  double qua_x, double qua_y, double qua_z,
                                  double qua_w) { //meiji//

        final gov.nasa.arc.astrobee.types.Point point = new gov.nasa.arc.astrobee.types.Point(pos_x, pos_y, pos_z);
        final gov.nasa.arc.astrobee.types.Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);

        Result result = api.moveTo(point, quaternion, false);
        int loopCounter = 0;
        while (!result.hasSucceeded() && loopCounter < 3) {
            result = api.moveTo(point, quaternion, false);
            loopCounter++;
        }

        return true;
    }
    private boolean moveToWrapper( gov.nasa.arc.astrobee.types.Point positionMap,  gov.nasa.arc.astrobee.types.Quaternion QuaternionMap ) { //meiji//
        Result result = api.moveTo(positionMap,QuaternionMap, false);
        int loopCounter = 0;
        while (!result.hasSucceeded() && loopCounter < 3) {
            result = api.moveTo(positionMap,QuaternionMap, false);
            loopCounter++;
        }
        return true;
    }
    public ARResult AR_cropping (int target_num,int emr){ //meiji//
        int ar_complete =0;
        boolean check=false;
        Mat kernel = new Mat(3, 3, CvType.CV_32F);
        kernel.put(0, 0, 0);
        kernel.put(0, 1, -1);
        kernel.put(0, 2, 0);
        kernel.put(1, 0, -1);
        kernel.put(1, 1, 5);
        kernel.put(1, 2, -1);
        kernel.put(2, 0, 0);
        kernel.put(2, 1, -1);
        kernel.put(2, 2, 0);
        double[][] cameraParam = api.getNavCamIntrinsics();
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        Mat dstMatrix = new Mat(1, 5, CvType.CV_32FC1);
        cameraMatrix.put(0, 0, cameraParam[0]);
        dstMatrix.put(0, 0, cameraParam[1]);
        Mat src = new Mat();
        Mat sharpened= new Mat();
        Mat undistort=new Mat();
        if(target_num==5){
            src =api.getMatDockCam() ;
        }else {
            src = api.getMatNavCam();
        }

        Calib3d.undistort(src, undistort, cameraMatrix, dstMatrix);
        if(emr ==1){
            undistort=src;

        }
        Imgproc.filter2D(undistort, sharpened, -1, kernel);
        api.saveMatImage(sharpened, "Pre-" + target_num + ".png");
        Dictionary arucoDict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        Mat ids = new Mat();

        List<Mat> corners = new ArrayList<>();


        Aruco.detectMarkers(sharpened, arucoDict, corners, ids);
        Log.i("ids count","ids counting :"+ ids.rows());


        if (!corners.isEmpty() && check == false) {
            float markerLength = 0.05f;
            // Find the marker with the desired y-coordinate
            int selectedIndex = 0;
            double selectedY = corners.get(0).get(0, 1)[1];  // y-coordinate of the first corner of the first marker

            for (int i = 1; i < corners.size(); i++) {
                double y = corners.get(i).get(0, 1)[1];
                if ((target_num == 2 && y > selectedY) || (target_num == 3 && y < selectedY)) {
                    selectedY = y;
                    selectedIndex = i;
                }
            }

            // Use the marker with the desired y-coordinate for further processing
            Mat selectedCorner = corners.get(selectedIndex);
            int selectedId = (int) ids.get(selectedIndex, 0)[0];
            List<Mat> selectedCorners = new ArrayList<>();
            selectedCorners.add(selectedCorner);
            // Estimate pose of the selected marker
            Mat rvecs = new Mat();
            Mat tvecs = new Mat();
            Aruco.estimatePoseSingleMarkers(selectedCorners, markerLength, cameraMatrix, dstMatrix, rvecs, tvecs);

            MatOfPoint2f cornerPoints = new MatOfPoint2f(selectedCorner);
            Point[] cornerArray = cornerPoints.toArray();

            // Calculate the Euclidean distances
            double pixelDistance1 = Core.norm(new MatOfPoint2f(cornerArray[0]), new MatOfPoint2f(cornerArray[1]));
            double pixelDistance2 = Core.norm(new MatOfPoint2f(cornerArray[0]), new MatOfPoint2f(cornerArray[3]));
            double pixelDistance3 = Core.norm(new MatOfPoint2f(cornerArray[1]), new MatOfPoint2f(cornerArray[2]));
            double pixelDistance4 = Core.norm(new MatOfPoint2f(cornerArray[2]), new MatOfPoint2f(cornerArray[3]));
            double pixelDistance = (pixelDistance1 + pixelDistance2 + pixelDistance3 + pixelDistance4) / 4;

            // Calculate the ratio
            double pixelToMRatio = pixelDistance / markerLength;


            // Print the pose information
            double TL = pixelToMRatio * 0.28;
            double TR = pixelToMRatio * 0.015;
            double TH = pixelToMRatio * 0.06;
            double BH = pixelToMRatio * 0.17;
            double angle = Math.atan2(cornerArray[0].y - cornerArray[1].y, cornerArray[0].x - cornerArray[1].x);
            double angleDegrees = Math.toDegrees(angle);
            Point center = new Point((cornerArray[0].x + cornerArray[2].x) / 2, (cornerArray[0].y + cornerArray[2].y) / 2);
            Mat rotationMatrix = Imgproc.getRotationMatrix2D(center, angleDegrees, -1.0);
            Mat rotatedImage = new Mat();
            Imgproc.warpAffine(sharpened, rotatedImage, rotationMatrix, undistort.size());

            // Compute the new corner points after rotation
            MatOfPoint2f newCorners = new MatOfPoint2f();
            Core.perspectiveTransform(cornerPoints, newCorners, rotationMatrix);
            Point[] newCornerArray = newCorners.toArray();
            int centerX = (int) center.x;
            int centerY = (int) center.y;
            Log.i("AR_pic(coordinate):", Integer.toString(centerX) + ',' + Integer.toString(centerY));
            int xMin = (int) (centerX - TL);
            int yMin = (int) (centerY - TH);
            int xMax = (int) (centerX + TR);
            int yMax = (int) (centerY + BH);

            // Ensure the coordinates are within image bounds
            xMin = Math.max(0, xMin);
            yMin = Math.max(0, yMin);
            xMax = Math.min(rotatedImage.width(), xMax);
            yMax = Math.min(rotatedImage.height(), yMax);

            // Crop the image
            Rect roi = new Rect(xMin, yMin, xMax - xMin, yMax - yMin);
            Mat croppedImage = new Mat(rotatedImage, roi);
            ar_complete=target_num;
            check=true;
            api.saveMatImage(croppedImage,"post_" + target_num + ".png");

        }else if(check==false &&emr!=1){
            Log.i("AR","not found");
            AR_cropping(target_num, 1);

        }

        return new ARResult(ar_complete, ids.rows());
    }





    public List<Object> wrapper_predict(int numberImage) throws IOException {
        String defaultFile = getImagePath(numberImage);
        File imageFile = new File(defaultFile);
        File modelFile = getModelFile(numberImage);

        List<Object> output = new ArrayList<>();
        float threshold = 0.3f;
        ObjectDetector.ObjectDetectorOptions options = modelConfig(threshold);
        ObjectDetector detector = ObjectDetector.createFromFileAndOptions(modelFile, options);

        if (!imageFile.exists()) {
            handleMissingImage(numberImage, output);

        }else {

            Bitmap bitmap = loadBitmapFromSDCard(defaultFile);
            TensorImage imageTensor = TensorImage.fromBitmap(bitmap);
            List<Detection> results = detector.detect(imageTensor);

            processResults(results, bitmap, numberImage, threshold, output, options);
        }
        return output;
    }

    private String getImagePath(int numberImage) {
        return "sdcard/data/jp.jaxa.iss.kibo.rpc.thailand/immediate/DebugImages/post_" + numberImage + ".png";
    }

    private File getModelFile(int numberImage) throws IOException {
        String modelName = numberImage == 5 ? "koonpolz5.tflite" : "koonpolz12.tflite";
        return convertModelFileFromAssetsToTempFile(modelName);
    }

    private void handleMissingImage(int numberImage,List<Object> output) {
        Log.i("EMR(Image):", "IMG Missing");
        output.add("Not Found");
        output.add(0);


    }

    private void processResults(List<Detection> results, Bitmap bitmap, int numberImage, float threshold, List<Object> output, ObjectDetector.ObjectDetectorOptions options) throws IOException {
        if (results.isEmpty()) {
            handleNoResults(numberImage, output, options,threshold);
            return;
        }

        List<Object> filterResult = filter(results, bitmap, numberImage, threshold);
        if (isReDetectionNeeded(filterResult)) {
            performReDetection(bitmap, output, options ,numberImage ,threshold);
        } else {
            output.add(filterResult.get(0));
            output.add(filterResult.get(1));
        }

        Log.i("Predicted", "Have " + results.size() + " items");
        Log.i("Predicted", "Detected item(Result): " + results.toString());
    }

    private boolean isReDetectionNeeded(List<Object> filterResult) {
        return filterResult.get(1).equals(1) &&
                !Arrays.asList("watch", "top", "kapton_tape", "goggle", "beaker").contains(filterResult.get(0));
    }

    private void performReDetection(Bitmap bitmap, List<Object> output, ObjectDetector.ObjectDetectorOptions options ,int numberImage, float threshold) throws IOException {
        File modelFile = convertModelFileFromAssetsToTempFile("koonpolz12.tflite");
        ObjectDetector reDetector = ObjectDetector.createFromFileAndOptions(modelFile, options);
        List<Detection> newResults = reDetector.detect(TensorImage.fromBitmap(bitmap));

        if (newResults.isEmpty()) {
            output.add("Not Found");
            output.add(0);
        } else {
            List<Object> newFilterResult = filter(newResults, bitmap, numberImage, threshold);
            output.add(newFilterResult.get(0));
            output.add(newFilterResult.get(1));
        }
    }

    private void handleNoResults(int numberImage, List<Object> output, ObjectDetector.ObjectDetectorOptions options, float threshold) throws IOException {
        Log.i("Predicted", "Failed");
        File modelFile;
        modelFile = convertModelFileFromAssetsToTempFile("koonpolz5.tflite");
        String defaultFile = getImagePath(numberImage);
        Log.i("EMR(Image):", "Missing");
        ObjectDetector detector = ObjectDetector.createFromFileAndOptions(modelFile, options);
        Bitmap bitmap = loadBitmapFromSDCard(defaultFile);
        TensorImage imageTensor = TensorImage.fromBitmap(bitmap);

        List<Detection> results = detector.detect(imageTensor);
        if (results.isEmpty()) {
            output.add("Not Found");
            output.add(0);
        } else {
            //processResults(results, bitmap, numberImage, threshold, output, options);
            List<Object> newFilterResult = filter(results, bitmap, numberImage, threshold);
            output.add(newFilterResult.get(0));
            output.add(newFilterResult.get(1));
        }
    }
    private ObjectDetector.ObjectDetectorOptions modelConfig(float threshold) {
        return ObjectDetector.ObjectDetectorOptions.builder()
                .setScoreThreshold(threshold)
                .build();
    }
    public List<Object> filter(List<Detection> results,Bitmap image,int number_image,float threshold){
        List<Object> output = new ArrayList<>();
        List<Object> check = new ArrayList<>();
        List<Object> index = new ArrayList<>();
        List<BoundingBoxWithScore> boundScoreList = new ArrayList<>();
        for (Detection detection : results) {
            String classidtest = detection.getCategories().get(0).getLabel();

            RectF boundingBox = detection.getBoundingBox();
            float score = detection.getCategories().get(0).getScore();
            Log.i("Predicted", "bounding box :" + boundingBox);
            Log.i("Predicted", "Class_named :" + classidtest);
            boundScoreList.add(new BoundingBoxWithScore(boundingBox, score, classidtest));

        }
        Bitmap resultImage = drawBoundingBoxes(image, boundScoreList);
        api.saveBitmapImage(resultImage,"before_iou_"+number_image+".png");

        float imageWidth = image.getWidth();
        Log.i("image", "ImageWidth :" + imageWidth);
        float imageHeight = image.getHeight();
        Log.i("image", "ImageHeight :" + imageHeight);
        float margin = 2.73f;

        List<BoundingBoxWithScore> filteredResults = NonMaxSuppression.softNonMaxSuppression(boundScoreList, 0.34f, 0.6f,0.2f, imageWidth, imageHeight, margin,2);

        // Count the number of detections per category after filtering
        Map<String, Integer> categoryCount = new HashMap<>();
        for (BoundingBoxWithScore bbox : filteredResults) {
            check.add(bbox);
            index.add(bbox.getCategory());
            Log.i("bbox", "bbox :"+ check.add(bbox));
            Log.i("bbox", "items :" +  index.add(bbox.getCategory()));


        }
        output.add(index.get(0));
        output.add(filteredResults.size());
        Log.i("test_new item", "Category: " + output.get(0));
        Log.i("test_new amount", "items: " + output.get(1))    ;
        // Print the count of each category
        Bitmap resultImage1 = drawBoundingBoxes(image, filteredResults);
        api.saveBitmapImage(resultImage1,"after_iou_"+number_image+".png");

        return output;
    }
    public Bitmap drawBoundingBoxes(Bitmap bitmap, List<BoundingBoxWithScore> boundingBoxes) {
        Bitmap mutableBitmap = bitmap.copy(Bitmap.Config.ARGB_8888, true);
        Canvas canvas = new Canvas(mutableBitmap);
        Paint paint = new Paint();
        paint.setColor(Color.RED);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(3.0f);

        for (BoundingBoxWithScore bbox : boundingBoxes) {
            canvas.drawRect(bbox.getRect(), paint);
        }

        return mutableBitmap;
    }
    private File convertModelFileFromAssetsToTempFile(String modelFileName) {
        try {
            // Open the TFLite model file from the assets directory
            InputStream inputStream = getAssets().open(modelFileName);

            // Create a temporary file
            File tempFile = File.createTempFile(modelFileName, null);
            tempFile.deleteOnExit(); // Delete the temporary file when the JVM exits

            // Write the model data to the temporary file
            try (FileOutputStream outputStream = new FileOutputStream(tempFile)) {
                byte[] buffer = new byte[4 * 1024]; // 4K buffer
                int bytesRead;
                while ((bytesRead = inputStream.read(buffer)) != -1) {
                    outputStream.write(buffer, 0, bytesRead);
                }
            }

            return tempFile;
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }

    public static Bitmap loadBitmapFromSDCard(String filePath) {
        File imgFile = new File(filePath);
        if (imgFile.exists()) {
            return BitmapFactory.decodeFile(imgFile.getAbsolutePath());
        }
        return null;
    }
    public  Quaternion eulerToQuaternion_use(double x, double y, double z) { //meiji//
        double yaw = Math.toRadians(z); //radian = degree*PI/180
        double pitch = Math.toRadians(y);
        double roll = Math.toRadians(x);

        double cy = Math.cos(yaw * 0.5);
        double sy = Math.sin(yaw * 0.5);
        double cp = Math.cos(pitch * 0.5);
        double sp = Math.sin(pitch * 0.5);
        double cr = Math.cos(roll * 0.5);
        double sr = Math.sin(roll * 0.5);

        double qx = sr * cp * cy - cr * sp * sy;
        double qy = cr * sp * cy + sr * cp * sy;
        double qz = cr * cp * sy - sr * sp * cy;
        double qw = cr * cp * cy + sr * sp * sy;

        return new Quaternion((float) qx, (float) qy, (float) qz, (float) qw);
    }





    private Quaternion eulerToQuaternion(List<Double> degree) { //meiji//
        double yaw = Math.toRadians(degree.get(0)); //radian = degree*PI/180
        double pitch = Math.toRadians(degree.get(1));
        double roll = Math.toRadians(degree.get(2));

        double cy = Math.cos(yaw * 0.5);
        double sy = Math.sin(yaw * 0.5);
        double cp = Math.cos(pitch * 0.5);
        double sp = Math.sin(pitch * 0.5);
        double cr = Math.cos(roll * 0.5);
        double sr = Math.sin(roll * 0.5);

        double qx = sr * cp * cy - cr * sp * sy;
        double qy = cr * sp * cy + sr * cp * sy;
        double qz = cr * cp * sy - sr * sp * cy;
        double qw = cr * cp * cy + sr * sp * sy;

        return new Quaternion((float) qx, (float) qy, (float) qz, (float) qw);
    }

    private List<Double> Final_turn(int target_num, int emr) { //meiji//
        boolean check = false;
        int center_x = 0;
        int center_y = 0;
        double pixelToMRatio = 0;
        Mat kernel = new Mat(3, 3, CvType.CV_32F);
        kernel.put(0, 0, 0);
        kernel.put(0, 1, -1);
        kernel.put(0, 2, 0);
        kernel.put(1, 0, -1);
        kernel.put(1, 1, 5);
        kernel.put(1, 2, -1);
        kernel.put(2, 0, 0);
        kernel.put(2, 1, -1);
        kernel.put(2, 2, 0);
        double[][] cameraParam = api.getNavCamIntrinsics();
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        Mat dstMatrix = new Mat(1, 5, CvType.CV_32FC1);
        cameraMatrix.put(0, 0, cameraParam[0]);
        dstMatrix.put(0, 0, cameraParam[1]);
        Mat src = new Mat();
        Mat sharpened = new Mat();
        Mat undistort = new Mat();
        src = api.getMatNavCam();
        Calib3d.undistort(src, undistort, cameraMatrix, dstMatrix);
        if (emr == 1) {
            undistort = src;
            check = true;
        }
        Imgproc.filter2D(undistort, sharpened, -1, kernel);
        api.saveMatImage(sharpened, "Final" + target_num + ".png");
        Dictionary arucoDict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        Mat ids = new Mat();

        List<Mat> corners = new ArrayList<>();
        Aruco.detectMarkers(sharpened, arucoDict, corners, ids);
        if (!corners.isEmpty()) {
            check = true;
            float markerLength = 0.05f;

            // Find the marker with the desired y-coordinate
            int selectedIndex = 0;
            double selectedY = corners.get(0).get(0, 1)[0];  // y-coordinate of the first corner of the first marker

            for (int i = 1; i < corners.size(); i++) {
                double y = corners.get(i).get(0, 1)[1];
                if ((target_num == 2 && y > selectedY) || (target_num == 3 && y < selectedY)) {
                    selectedY = y;
                    selectedIndex = i;
                }
            }

            // Use the marker with the desired y-coordinate for further processing
            Mat selectedCorner = corners.get(selectedIndex);
            List<Mat> selectedCorners = new ArrayList<>();
            selectedCorners.add(selectedCorner);
            // Estimate pose of the selected marker
            Mat rvecs = new Mat();
            Mat tvecs = new Mat();
            Aruco.estimatePoseSingleMarkers(selectedCorners, markerLength, cameraMatrix, dstMatrix, rvecs, tvecs);

            MatOfPoint2f cornerPoints = new MatOfPoint2f(selectedCorner);
            Point[] cornerArray = cornerPoints.toArray();

            // Calculate the Euclidean distances
            double pixelDistance1 = Core.norm(new MatOfPoint2f(cornerArray[0]), new MatOfPoint2f(cornerArray[1]));
            double pixelDistance2 = Core.norm(new MatOfPoint2f(cornerArray[0]), new MatOfPoint2f(cornerArray[3]));
            double pixelDistance3 = Core.norm(new MatOfPoint2f(cornerArray[1]), new MatOfPoint2f(cornerArray[2]));
            double pixelDistance4 = Core.norm(new MatOfPoint2f(cornerArray[2]), new MatOfPoint2f(cornerArray[3]));
            double pixelDistance = (pixelDistance1 + pixelDistance2 + pixelDistance3 + pixelDistance4) / 4;

            // Calculate the ratio
            pixelToMRatio = pixelDistance / markerLength;


            // Print the pose information

            Point center = new Point((cornerArray[0].x + cornerArray[2].x) / 2, (cornerArray[0].y + cornerArray[2].y) / 2);
            center_x = (int) center.x;
            center_y = (int) center.y;
            Log.i("AR", String.valueOf(center_x) + "," + String.valueOf(center_y));
        } else if (check == false) {
            Log.i("AR", "not found");

            Final_turn(target_num, 1);

        }
        List<Double> changeAngle = new ArrayList<>();
        double changeof_y = 0.0;
        double changeof_x = 0.0;
        Kinematics kinematics = api.getRobotKinematics();
        gov.nasa.arc.astrobee.types.Point pos_as = kinematics.getPosition();
        if (target_num == 3) {
            changeof_y = Math.atan(((640 - center_x) / pixelToMRatio) / (pos_as.getZ() - 3.76093)) * 57.296;
            changeof_x = (Math.atan(((480 - center_y) / pixelToMRatio) / (pos_as.getZ() - 3.76093)) * 57.296) * 0.75;
            if (pixelToMRatio == 0) {
                changeof_y = 0;
                changeof_x = 0;
            }
            Log.i("AR_pic(coordinate):", Double.toString(changeof_y));
            Log.i("AR_pic(coordinate):", Double.toString(changeof_x));
            changeAngle.add(0.0+changeof_x);
            changeAngle.add(75.0 + changeof_y);
            changeAngle.add(90.0);
        } else if (target_num == 1) {
            changeof_y = Math.atan(((640 - center_x) / pixelToMRatio) / (pos_as.getY() + 10.58)) * 57.296;
            changeof_x = (Math.atan(((480 - center_y) / pixelToMRatio) / (pos_as.getY() + 10.58)) * 57.296)*0.75;
            Log.i("AR_pic(coordinate):", Double.toString(changeof_y));
            Log.i("AR_pic(coordinate):", Double.toString(changeof_x));
            if (pixelToMRatio == 0) {
                changeof_y = 0;
                changeof_x = 0;
            }
            changeAngle.add(-90.000 - changeof_y);
            changeAngle.add(-25.000+changeof_x);
            changeAngle.add(0.000);

        } else if (target_num == 2) {
            changeof_y = Math.atan(((640 - center_x) / pixelToMRatio) / (pos_as.getZ() - 3.76203)) * 57.296;
            changeof_x = (Math.atan(((480 - center_y) / pixelToMRatio) / (pos_as.getZ() - 3.76203)) * 57.296)*0.75;
            if (pixelToMRatio == 0) {
                changeof_y = 0;
                changeof_x = 0;
            }
            Log.i("AR_pic(coordinate):", Double.toString(changeof_y));
            Log.i("AR_pic(coordinate):", Double.toString(changeof_x));
            changeAngle.add(0.0+changeof_x);
            changeAngle.add(90.0 + changeof_y);
            changeAngle.add(90.0);
        } else if (target_num == 5) {
            changeof_y = Math.atan(((640 - center_x) / pixelToMRatio) / (pos_as.getZ() - 3.76093)) * 57.296;
            changeof_x = (Math.atan(((480 - center_y) / pixelToMRatio) / (pos_as.getZ() - 3.76093)) * 57.296)*0.75;
            if (pixelToMRatio == 0) {
                changeof_y = 0;
                changeof_x = 0;
            }
            Log.i("AR_pic(coordinate):", Double.toString(changeof_y));
            Log.i("AR_pic(coordinate):", Double.toString(changeof_x));
            changeAngle.add(0.0+changeof_x);
            changeAngle.add(90.0 + changeof_y);
            changeAngle.add(90.0);
        } else if (target_num == 4) {
            changeof_y = Math.atan(((480 - center_y) / pixelToMRatio) / (pos_as.getX() - 9.866984)) * 57.296;
            changeof_x = (Math.atan(((640 - center_x) / pixelToMRatio) / (pos_as.getX() - 9.866984)) * 57.296)*0.75;
            if (pixelToMRatio == 0) {
                changeof_y = 0;
                changeof_x = 0;
            }
            Log.i("AR_pic(coordinate):", Double.toString(changeof_y));
            Log.i("AR_pic(coordinate):", Double.toString(changeof_x));
            changeAngle.add(-180.000 -changeof_x);
            changeAngle.add(0.0 - changeof_y);
            changeAngle.add(0.0);
        }


        return changeAngle;
    }

}

