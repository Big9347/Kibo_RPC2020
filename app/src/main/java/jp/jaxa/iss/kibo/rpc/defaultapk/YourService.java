package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
import android.util.Log;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.ChecksumException;
import com.google.zxing.FormatException;
import com.google.zxing.LuminanceSource;
import com.google.zxing.NotFoundException;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
//    static {
//        Loader.load();
//    }

    private static void warmZxing(QRCodeReader reader) {
        int w = 1280, h = 960;
        Bitmap.Config conf = Bitmap.Config.ARGB_8888; // see other conf types
        Bitmap bMap = Bitmap.createBitmap(w, h, conf);
        int[] intArray = new int[bMap.getWidth() * bMap.getHeight()];
        //copy pixel data from the Bitmap into the 'intArray' array
        bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());

        LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArray);
        source = source.crop(490, 330, 300, 300);
        BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));

        try {
            reader.decode(bitmap);
        } catch (NotFoundException | ChecksumException | FormatException e) {
            e.printStackTrace();
        }
    }

    private static Mat putAll(int row, int col, double[] data) {
        Mat matrix = new Mat(row, col, CvType.CV_64F);
        int k = 0;

        for (int i = 0; i < row; i++) {
            for (int j = 0; j < col; j++) {
                matrix.put(i, j, data[k]);
                k++;
            }
        }
        return matrix;
    }
    /*private static Mat multiplyMatrix(Mat rotateMat, Mat posMat,int m,int n,int k){
        Mat result = new Mat(m,n,CvType.CV_64F);
        for (int i=0;i<m;i++){
            for (int j=0;j<n;j++){
                double c=0;
                for (int q=0;q<k;q++){
                    double a= rotateMat.get(i,q)[0];
                    double b= posMat.get(q,j)[0];
                    c += (a*b);

                }
                result.put(i,j,c);
            }

        }
        return result;
    }
    private static double[][] multiplyMatrix(double[][] rotateMat, double[][] posMat,int m,int n,int k){
        double[][] result = new double[m][n];
        for (int i=0;i<m;i++){
            for (int j=0;j<n;j++){
                double c=0;
                for (int q=0;q<k;q++){
                    double a= rotateMat[i][q];
                    double b= posMat[q][j];
                    c += (a*b);

                }
                result[i][j]=c;
            }

        }
        return result;
    }
    private static double[][] multiplyMatrixWithNumber(double[][] matrix, double n, int row , int col){
        for (int i=0;i<row;i++){
            for (int j=0;j<col;j++){
                matrix[i][j] *= n;
            }
        }
        return matrix;
    }*/

    @Override
    protected void runPlan1() {
        try {
            QRCodeReader reader = new QRCodeReader();
            final int LOOP_MAX = 3;
            int loop_counter = -3;
            Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
            java.util.List<Mat> cornerList = new ArrayList<>();
            Mat ids = new Mat();
            java.util.List<Mat> rejectedImg = new ArrayList<>();
            double[] data_d = {-0.152963, 0.017530, -0.001107, -0.000210, 0.000000};
            Mat discof = putAll(1, 5, data_d);
            data_d = new double[]{344.173397, 0.000000, 630.793795, 0.000000, 344.277922, 487.033834, 0.000000, 0.000000, 1.000000};
            Mat camera_matrix = putAll(3, 3, data_d);

            DetectorParameters detectorParameters = DetectorParameters.create();
            detectorParameters.set_minDistanceToBorder(50);
            // detectorParameters.set_minMarkerPerimeterRate(0.05);
            //   detectorParameters.set_maxMarkerPerimeterRate(0.34);
            //  detectorParameters.se
            //  detectorParameters.set_polygonalApproxAccuracyRate(0.1);

//            detectorParameters.set_adaptiveThreshWinSizeMax(16);
            //          detectorParameters.set_adaptiveThreshWinSizeMin(10);
            // detectorParameters.set_adaptiveThreshWinSizeStep(5);
            try {
                Mat warmAR = new Mat(1280, 960, CvType.CV_8UC1);
                Aruco.detectMarkers(warmAR, dictionary, cornerList, ids, detectorParameters, rejectedImg, camera_matrix, discof);

            } catch (Exception e) {
                e.printStackTrace();
            }


            warmZxing(reader);

            try {
                final Point point = new Point(11.3939, -5.754, 4.5064);
                final Quaternion quaternion = new Quaternion(0, 0, 1, 0);

                Result result;
                int loopCounter = 0;
                api.judgeSendStart();
                do {
                    result = api.moveTo(point, quaternion, false);
                    try {
                        if (result.hasSucceeded())
                            break;
                    } catch (NullPointerException e) {
                        e.printStackTrace();
                        continue;
                    }
                    ++loopCounter;

                } while (loopCounter < LOOP_MAX);
            } catch (Exception e) {
                e.printStackTrace();
            }
            loop_counter = 0;
            //Move to P1-1
            //  moveToWrapper(11.3823, -5.6578, 4.5826, 0, 0, 0, 1, 3)
//            moveToWrapper(11.3823, -5.6578, 4.5826, 0, 0, 0, 1, 3); //q0
            //   moveToWrapper(11.3939, -5.754, 4.5064, 0, 0, 1, 0, 3);
            //Scan QR NO.0
            String pos_x = readQRImageDock(reader);
            while (pos_x == null && loop_counter < LOOP_MAX) {
//                moveToWrapper(11.3823, -5.6578, 4.5826, 0, 0, 0, 1, 3); //q0
                moveToWrapper(11.3939, -5.754, 4.5064, 0, 0, 1, 0, 3);
                pos_x = readQRImageDock(reader);
                loop_counter++;
            }
            Log.d("Qrcode", pos_x);
            api.judgeSendDiscoveredQR(0, pos_x);
            loop_counter = 0;
            //Move to P1-3
            // moveToWrapper(11.0826, -5.4578, 4.4477 + 0.08, 0, 0.7071068f, 0, 0.7071068f, 3); //q2
//            moveToWrapper(11.0826, -5.4578, 4.4477, 0, 0.7071068f, 0, 0.7071068f, 6); //q2
//            moveToWrapper(11.054, -5.4936, 4.4361+0.05, 0, -0.7071068f, 0, 0.7071068f, 6);
            moveToWrapper(10.9936, -5.446, 4.4361, 0, -0.7071068f, 0, 0.7071068f, 6);
            //Scan QR NO.2
            String pos_z = readQRImageDock(reader);
            while (pos_z == null && loop_counter < LOOP_MAX) {
//                moveToWrapper(11.0826, -5.4578, 4.4477, 0, 0.7071068f, 0, 0.7071068f, 3); //q2
                moveToWrapper(10.9936, -5.446, 4.4361, 0, -0.7071068f, 0, 0.7071068f, 3);
                pos_z = readQRImageDock(reader);
                loop_counter++;
            }
            Log.d("Qrcode", pos_z);
            api.judgeSendDiscoveredQR(2, pos_z);
            loop_counter = 0;
            //Move to P1-2
            //   moveToWrapper(10.9174, -5.9578, 5.4323 -  0.08, 0, -0.7071068f, 0, 0.7071068f, 3);//q1
            moveToWrapper(10.9174, -5.9578, 5.4323, 0, -0.7071068f, 0, 0.7071068f, 6);//q1
            //Scan QR NO.1
            String pos_y = readQRImage(reader);
            while (pos_y == null && loop_counter < LOOP_MAX) {
                moveToWrapper(10.9174, -5.9578, 5.4323, 0, -0.7071068f, 0, 0.7071068f, 3);//q1
                pos_y = readQRImage(reader);
                loop_counter++;
            }
            Log.d("Qrcode", pos_y);
            api.judgeSendDiscoveredQR(1, pos_y);


            //Passing KOZ from Bay 3 to Bay 5
           /* moveToWrapmper(10.45, -6.4, 4.7826, 0, 0, 0, 0, 5);
            moveToWrapper(11.15, -7.15, 4.7826, 0, 0, 0, 0, 7);*/
            moveToWrapper(10.50, -6.5, 5.4439, 0, 0, 0, 0, 5);
            moveToWrapper(11.10, -7.2, 5.4439, 0, 0, 0, 0, 7);

            loop_counter = 0;
            //Move to P2-3
            //  moveToWrapper(10.9174, -7.6578, 5.4323 - 0.08, 0, -0.7071068f, 0, 0.7071068f, 3);//q5
//            moveToWrapper(10.9174, -7.6578, 5.4323, 0, -0.7071068f, 0, 0.7071068f, 6);//q5
            // moveToWrapper(11.1061, -7.7064, 5.4439-0.05, 0, 0.7071068f, 0, 0.7071068f, 6); tham
            moveToWrapper(11.0064, -7.646, 5.4439, 0, 0.7071068f, 0, 0.7071068f, 6);
            //Scan QR NO.5
            String qua_z = readQRImageDock(reader);
            while (qua_z == null && loop_counter < LOOP_MAX) {
//                moveToWrapper(10.9174, -7.6578, 5.4323, 0, -0.7071068f, 0, 0.7071068f, 3);//q5
                moveToWrapper(11.0064, -7.646, 5.4439, 0, 0.7071068f, 0, 0.7071068f, 3);
                qua_z = readQRImageDock(reader);
                loop_counter++;
            }
            Log.d("Qrcode", qua_z);
            api.judgeSendDiscoveredQR(5, qua_z);
            loop_counter = 0;
            //Move to P2-1
            //   moveToWrapper(10.4177 + 0.08, -7.5422, 4.7826, 0, 0, 1, 0, 3); //q3
            moveToWrapper(10.4177, -7.5422, 4.7826, 0, 0, 1, 0, 6); //q3
            //Scan QR NO.3
            String qua_x = readQRImage(reader);
            while (qua_x == null && loop_counter < LOOP_MAX) {
                moveToWrapper(10.4177, -7.5422, 4.7826, 0, 0, 1, 0, 3); //q3
                qua_x = readQRImage(reader);
                loop_counter++;
            }
            Log.d("Qrcode", qua_x);
            api.judgeSendDiscoveredQR(3, qua_x);

            //Move to P2-2
            //  moveToWrapper(11.5 - 0.1177 - 0.08, -8 + 0.0422, 5 + 0.0826, 0, 0, 0, 1, 3); //q4
            moveToWrapper(11.3823, -7.9578, 5.0826, 0, 0, 0, 1, 6); //q4
            //Scan QR NO.4
            String qua_y = readQRImage(reader);
            while (qua_y == null && loop_counter < LOOP_MAX) {
                moveToWrapper(11.3823, -7.9578, 5.0826, 0, 0, 0, 1, 3); //q4
                qua_y = readQRImage(reader);
                loop_counter++;
            }
            Log.d("Qrcode", qua_y);
            api.judgeSendDiscoveredQR(4, qua_y);
            loop_counter = 0;


            //Passing KOZ from Bay 5 to Bay 7
//            moveToWrapper(11.15+0.24, -8.4-0.24, 5.1+0.24, 0, 0, 0,0, 5);
         /*   //Extract a floating value from the variables.
            final double pos_x_f = Double.parseDouble(pos_x.split(" ")[1]);
            final double pos_y_f = Double.parseDouble(pos_y.split(" ")[1]);
            final double pos_z_f = Double.parseDouble(pos_z.split(" ")[1]);
            final double qua_x_f = Double.parseDouble(qua_x.split(" ")[1]);
            final double qua_y_f = Double.parseDouble(qua_y.split(" ")[1]);
            final double qua_z_f = Double.parseDouble(qua_z.split(" ")[1]);
            //Calculate qua_w by normalize the quaternion to a unit quaternion.
            double qua_w_f = Math.sqrt(1 - (qua_x_f * qua_x_f) - (qua_y_f * qua_y_f) - (qua_z_f * qua_z_f));*/
        /*    Log.d("myTag",String.valueOf(pos_x_f));
            Log.d("myTag",String.valueOf(pos_y_f));
            Log.d("myTag",String.valueOf(pos_z_f));
            Log.d("myTag",String.valueOf(qua_x_f));
            Log.d("myTag",String.valueOf(qua_y_f));
            Log.d("myTag",String.valueOf(qua_z_f));
            Log.d("myTag",String.valueOf(qua_w_f));*/

            //Move to P3
            moveToWrapper(10.88, -9.5, 5.3, 0, 0, 0.7071068f, -0.7071068f, 7);


//            byte loopCounter = 0;
//            gov.nasa.arc.astrobee.Result result;
//            Point point_ar = new Point(10.85, -9.5, 5.3);
//            final Quaternion quaternion_ar = new Quaternion(0, 0, 0.7071068f, -0.7071068f);
//            do {
//                result = api.moveTo(point_ar, quaternion_ar, true);
//                if (result.getMessage().endsWith("tolerance violated")) {
//                    continue;
//                } else if (result.getMessage().equals("Move goal failed with response: Keep in zone violation")) {
//                    moveToWrapper(11.25, -8.8, 5.35, 0, 0, 0, 0,6);
//
//                    loopCounter = 0;
//                    gov.nasa.arc.astrobee.Result result;
//                    Point point_ar = new Point(ar_pos_x_num + change_x_ar, ar_pos_y_num + 0.05, ar_pos_z_num + change_z_ar);
//                    final Quaternion quaternion_ar = new Quaternion(0, 0, -0.7071068f, 0.7071068f);
//                    do {
//                        result = api.moveTo(point_ar, quaternion_ar, true);
//                        if (result.getMessage().endsWith("tolerance violated")) {
//                            continue;
//                        } else if (result.getMessage().equals("Move goal failed with response: Keep in zone violation")) {
//                            moveToWrapper(11.25, -8.8, 5.35, 0, 0, 0, 0);
//                            continue;
//                        }
//                        loopCounter++;
//                    } while (!result.hasSucceeded() && loopCounter < LOOP_MAX);
//                    continue;
//                }
//                loopCounter++;
//            } while (!result.hasSucceeded() && loopCounter < LOOP_MAX);


            Mat nav_mat;
            final int L_M = 5;
            int l_c = 0;
            //Scan AR Tag
            do {//Log.d("Ar","Start");
                nav_mat = api.getMatNavCam();
                if (nav_mat == null)
                    continue;

                Aruco.detectMarkers(nav_mat, dictionary, cornerList, ids, detectorParameters, rejectedImg, camera_matrix, discof);
                l_c++;
                // Log.d("Ar","End");
            } while (ids.empty() && l_c < L_M);
            if (ids.empty()) {
              /*  l_c = 3;
                //Scan AR Tag
                do {Log.d("Ar","adjust brightness");
                    nav_mat = api.getMatNavCam();
                    if (nav_mat == null)
                        continue;
                    Mat dst=new Mat();
                    Imgproc.equalizeHist(nav_mat,dst);
                    dst.convertTo(dst,-1,1,50);
                    Aruco.detectMarkers(dst, dictionary, cornerList, ids, detectorParameters, rejectedImg, camera_matrix, discof);
                    l_c++;
                    // Log.d("Ar","End");
                } while (ids.empty() && l_c < L_M);
*/

                Log.d("Ar", "adjust brightness & parameter loop");

                //Scan AR Tag

                while (nav_mat == null){
                    nav_mat = api.getMatNavCam();}

                int brt = 50;
                for (int a = 250; a < 901; a++) {
                    for (int b = 280; b < 1001; b++) {
                        double img_data = nav_mat.get(a, b)[0];
                        if (img_data < brt - 1) {
                            if (img_data + brt > 255) {
                                nav_mat.put(a, b, img_data + brt - 255);
                            } else {
                                nav_mat.put(a, b, img_data + brt);
                            }
                        }
                    }
                }//280 250 720 650 row col
                Aruco.detectMarkers(nav_mat, dictionary, cornerList, ids, detectorParameters, rejectedImg, camera_matrix, discof);
                    if (ids.empty()) {
                        int a=14;
                        detectorParameters.set_adaptiveThreshWinSizeMax(a + 9);
                        detectorParameters.set_adaptiveThreshWinSizeMin(a);
                        detectorParameters.set_adaptiveThreshWinSizeStep(1);
                        Aruco.detectMarkers(nav_mat, dictionary, cornerList, ids, detectorParameters, rejectedImg, camera_matrix, discof);
                    }


            }

            //Send AR's ID
            try {
                if (ids.empty()) {
                    api.judgeSendFinishSimulation();
                }
                final String ar = String.valueOf((int) (ids.get(0, 0)[0]));
                api.judgeSendDiscoveredAR(ar);
            } catch (Exception e) {
                e.printStackTrace();

            }
            //Finding the target position.
            //runplan2();
            Mat rvecs = new Mat();
            Mat tvecs = new Mat();
            Aruco.estimatePoseSingleMarkers(cornerList, 0.05f, camera_matrix, discof, rvecs, tvecs);
            moveToWrapper(10.88 + 0.20 * Math.sin(Math.toRadians(45)) - 0.0572 + tvecs.get(0, 0)[0] - 0.0422, -9.5 - 0.05, 5.3 + (0.20 * Math.sin(Math.toRadians(45))) + 0.1111 + tvecs.get(0, 0)[1] - 0.0826, 0, 0, 0.7071068f, -0.7071068f, 7);

        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            api.laserControl(true);
            api.judgeSendFinishSimulation();
            api.shutdownFactory();
        }


    }

    @Override
    protected void runPlan2() {

        //Pose estimation 0f a ArUco marker relative to camera frame.
   /*
    data_d = new double[]{0.20 * Math.sin(Math.toRadians(45)), -(0.20 * Math.sin(Math.toRadians(45))), 0};
            Mat target_InArFrame = putAll(3, 1, data_d);
            Vector3 vec_laserpointerInCamFrame = new Vector3(0.1302, 0.0572, -0.1111);
            vec_laserpointerInCamFrame.normalise();
            Mat rvecs = new Mat();
            Mat tvecs = new Mat();
   Aruco.estimatePoseSingleMarkers(cornerList, 0.05f, camera_matrix, discof, rvecs, tvecs);

            rmat = Rotation matrix of AR tag relative to the navigation camera frame. (Mat)
            Calib3d.Rodrigues = The function of Opencv lib for transform rotation vector to rotation matrix.
            rvec and tvec are the rotation and translation vector that transforms a 3D point expressed in the ArUco marker frame into the camera frame.

        Mat rmat = new Mat();
        Calib3d.Rodrigues(rvecs, rmat);

         Transform the target position located 45° and 20cm lower right from AR tag in Aruco marker frame to Navcam frame.

        Mat target_IncameraFrame = multiplyMatrix(rmat, target_InArFrame, 3, 1, 3);

            Get the position vector of target in Astrobee's frame with axis:
            x-right
            y-down
            z-forward

        double vec_x_targertIndefaultCamFrame = -0.0422 + target_IncameraFrame.get(0, 0)[0] + tvecs.get(0, 0)[0];
       double vec_y_targertIndefaultCamFrame = -0.0826 + target_IncameraFrame.get(1, 0)[0] + tvecs.get(0, 0)[1];
        double vec_z_targertIndefaultCamFrame = 0.1177 + target_IncameraFrame.get(2, 0)[0] + tvecs.get(0, 0)[2];
             Get the position vector of target in Astrobee's frame with axis:
            y-right
            z-down
            x-forward

        //Swap axis
        Vector3 vec_target_IncameraFrame = new Vector3(vec_z_targertIndefaultCamFrame, vec_x_targertIndefaultCamFrame, vec_y_targertIndefaultCamFrame);
        //normalize to a unit vector.
        vec_target_IncameraFrame.normalise();
        //Create a quaternion rotation from two vectors(Rotates  from laser pointer vector to the target point vector). quaforpointlaser
        QuaternionImp quaImpForRotateToTargetInCamFrame = createQuaFrom2Vecs2nd(vec_laserpointerInCamFrame, vec_target_IncameraFrame);

        QuaternionImp quaImp_P3 = new QuaternionImp(qua_x_f, qua_y_f, qua_z_f, qua_w_f);
        //Multiply the quaternion rotation by the quaternion of P3 to rotate the laser pointer to the target position.
        QuaternionImp quaImp_final = quaImpForRotateToTargetInCamFrame.mulThis(quaImp_P3);

        Quaternion qua_final = new Quaternion((float) quaImp_final.getX(), (float) quaImp_final.getY(), (float) quaImp_final.getZ(), (float) quaImp_final.getW());
        Point p_final = new Point(pos_x_f, pos_y_f, pos_z_f);
        moveToWrapper(p_final, qua_final, 5);  */
    }

    @Override
    protected void runPlan3() {

    }

    // You can add your method
    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                               float qua_x, float qua_y, float qua_z,
                               float qua_w, final int LOOP_MAX) {

        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion(qua_x, qua_y,
                qua_z, qua_w);

        Result result;
        int loopCounter = 0;
        do {
            result = api.moveTo(point, quaternion, false);
            try {
                if (result.hasSucceeded())
                    return;
            } catch (NullPointerException e) {
                e.printStackTrace();
                continue;
            }
            ++loopCounter;

        } while (loopCounter < LOOP_MAX);

    }

    private String readQRImage(QRCodeReader reader) {
        Bitmap bMap;
        final int LOOP_MAX = 3;
        String contents = null;
        int loop_counter = 0;
        int[] intArray = new int[1228800];
        do {
            //   Log.d("Qr","Start");
            bMap = api.getBitmapNavCam();
            if (bMap == null)
                continue;
            bMap.getPixels(intArray, 0, 1280, 0, 0, 1280, 960);

            LuminanceSource source = new RGBLuminanceSource(1280, 960, intArray);
            source = source.crop(490, 330, 300, 300);
            BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));

            try {
                com.google.zxing.Result result = reader.decode(bitmap);
                contents = result.getText();
                if (contents != null)
                    return contents;
            } catch (NotFoundException e) {
                e.printStackTrace();
                break;
            } catch (ChecksumException | FormatException e) {
                // Log.d("QrERROR",bitmapToString(bMap));
                e.printStackTrace();
            }
            loop_counter++;
            //  Log.d("Qr","End");
        } while (loop_counter < LOOP_MAX);

        return contents;
    }

    private String readQRImageDock(QRCodeReader reader) {
        Bitmap bMap;
        final int LOOP_MAX = 3;
        String contents = null;
        int loop_counter = 0;
        int[] intArray = new int[1228800];
        do {
            //   Log.d("Qr","Start");
            bMap = api.getBitmapDockCam();
            if (bMap == null)
                continue;
            bMap.getPixels(intArray, 0, 1280, 0, 0, 1280, 960);

            LuminanceSource source = new RGBLuminanceSource(1280, 960, intArray);
            source = source.crop(490, 330, 300, 300);
            BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));

            try {
                com.google.zxing.Result result = reader.decode(bitmap);
                contents = result.getText();
                if (contents != null)
                    return contents;
            } catch (NotFoundException e) {
                e.printStackTrace();
                break;
            } catch (ChecksumException | FormatException e) {
                // Log.d("QrERROR",bitmapToString(bMap));
                e.printStackTrace();
            }
            loop_counter++;
            //  Log.d("Qr","End");
        } while (loop_counter < LOOP_MAX);

        return contents;
    }

}
