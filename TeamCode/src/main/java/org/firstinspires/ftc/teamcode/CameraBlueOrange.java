/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class CameraBlueOrange
{
    OpenCvInternalCamera phoneCam;
    SEDPipeline pipeline;
    HardwareMap hardwareMap;

    public CameraBlueOrange(HardwareMap hwMap) {
        hardwareMap = hwMap;
    }


    public void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SEDPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though. m mm
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    public String getStringPosition() {
        return pipeline.getStringAnalysis();
    }

    public static class SEDPipeline extends OpenCvPipeline {
        /*
         * An enum to define the skystone position
         */
        public enum SignalPosition
        {
            LEFT,
            CENTER,
            RIGHT
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar BLACK = new Scalar(0, 0, 0);
        static final Scalar ORANGE = new Scalar(255, 69, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(150,40);
        static final int REGION1_WIDTH = 25;
        static final int REGION1_HEIGHT = 15;

        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(200,40);
        static final int REGION2_WIDTH = 25;
        static final int REGION2_HEIGHT = 15;

        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(100,40);
        static final int REGION3_WIDTH = 25;
        static final int REGION3_HEIGHT = 15;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);

        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION2_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION2_HEIGHT);

        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION3_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION3_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Y, region1_Cr, region1_Cb;
        Mat region2_Y, region2_Cr, region2_Cb;
        Mat region3_Y, region3_Cr, region3_Cb;

        int avgY1, avgCr1, avgCb1;
        int avgY2, avgCr2, avgCb2;
        int avgY3, avgCr3, avgCb3;

        Mat YCrCb = new Mat();
        Mat YImage = new Mat();
        Mat CrImage = new Mat();
        Mat CbImage = new Mat();

//        Mat YCrCb2 = new Mat();
//        Mat YImage2 = new Mat();
//        Mat CrImage2 = new Mat();
//        Mat CbImage2 = new Mat();
//
//        Mat YCrCb3 = new Mat();
//        Mat YImage3 = new Mat();
//        Mat CrImage3 = new Mat();
//        Mat CbImage3 = new Mat();



        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile SignalPosition position = SignalPosition.RIGHT;
        private volatile String StringPos = "CENTER";

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Y channel to the 'Y' variable
         */
        void inputToChannels(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, YImage, 0);
            Core.extractChannel(YCrCb, CrImage, 1);
            Core.extractChannel(YCrCb, CbImage, 2);
        }

        @Override
        public void init(Mat firstFrame) {
            /*
             * We need to call this in order to make sure the 'Y'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToChannels(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Y = YImage.submat(new Rect(region1_pointA, region1_pointB));
            region1_Cr = CrImage.submat(new Rect(region1_pointA, region1_pointB));
            region1_Cb = CbImage.submat(new Rect(region1_pointA, region1_pointB));

            region2_Y = YImage.submat(new Rect(region2_pointA, region2_pointB));
            region2_Cr = CrImage.submat(new Rect(region2_pointA, region2_pointB));
            region2_Cb = CbImage.submat(new Rect(region2_pointA, region2_pointB));

            region3_Y = YImage.submat(new Rect(region3_pointA, region3_pointB));
            region3_Cr = CrImage.submat(new Rect(region3_pointA, region3_pointB));
            region3_Cb = CbImage.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToChannels(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avgY1 = (int) Core.mean(region1_Y).val[0];
            avgCr1 = (int) Core.mean(region1_Cr).val[0];
            avgCb1 = (int) Core.mean(region1_Cb).val[0];

            avgY2 = (int) Core.mean(region2_Y).val[0];
            avgCr2 = (int) Core.mean(region2_Cr).val[0];
            avgCb2 = (int) Core.mean(region2_Cb).val[0];

            avgY3 = (int) Core.mean(region3_Y).val[0];
            avgCr3 = (int) Core.mean(region3_Cr).val[0];
            avgCb3 = (int) Core.mean(region3_Cb).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Now that we found the min, we actually need to go and
             * figure out which sample region that value was from
             */
            if(avgCb1 - avgCr1 > 20) {
                position = SignalPosition.LEFT; // Record our analysis
                StringPos = "RIGHT";

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        BLUE, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(avgCr1 - avgCb1 > 20) {
                position = SignalPosition.RIGHT; // Record our analysis
                StringPos = "LEFT";

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        ORANGE, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else{
                position = SignalPosition.CENTER; // Record our analysis
                StringPos = "CENTER";

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        BLACK, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public SignalPosition getAnalysis()
        {
            return position;
        }

        public String getStringAnalysis()
        {
            return StringPos;
        }

        public int getAvgY1() { return avgY1; }
        public int getAvgCr1() { return avgCr1; }
        public int getAvgCb1() { return avgCb1; }
    }
}
