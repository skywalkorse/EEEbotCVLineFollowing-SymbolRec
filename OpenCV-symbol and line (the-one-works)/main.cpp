// Include files for required libraries
#include <stdio.h>

#include "opencv_aee.hpp"
#include "main.hpp"     // You can use this file for declaring defined values and functions
#include "pi2c.h"
#include <stdlib.h>

float currentError = 0;
float prevError = 0;
float errorSum = 0;
float Kp = 12;
float Ki = 0;
float Kd = 0;
float PIDNum = 0;
int steerCenter = 89;
int speedNormal = 85;
Pi2c ESP32(0x04); // set up the I2C slave with address 0x10
int lowH,highH,lowS,highS,lowV,highV;
int motorsFlag = 1;


void setup(void)
{
    setupCamera(320, 240);  // Enable the camera for OpenCV
}
void colourPicker(int colourNum)
{
    switch (colourNum)
    {
    case 0: // Red
        lowH = 0;
        highH = 40;
        lowS = 140;
        highS = 255;
        lowV = 90;
        highV = 135;
        break;
    case 1: // Green
        lowH = 75;
        highH = 90;
        lowS = 140;
        highS = 255;
        lowV = 65;
        highV = 255;
        break;
    case 2: // Blue
        lowH = 95;
        highH = 110;
        lowS = 150;
        highS =255;
        lowV = 0;
        highV = 255;
        break;
    case 3: // Yellow
        break;
    case 4: //Black
        lowH = 40;
        highH = 90;
        lowS = 75;
        highS = 165;
        lowV = 0;
        highV = 110;
        break;
    }
}
void runMotors(int leftMotorSpeed, int rightMotorSpeed, int steeringAngle)
{
    if (steeringAngle < 30)
        steeringAngle = 30;
    if (steeringAngle > 150)   // Constrain angles
        steeringAngle = 150;
    if((leftMotorSpeed !=0) || (rightMotorSpeed != 0))
    {
        if (leftMotorSpeed > (speedNormal + 85))
            leftMotorSpeed = (speedNormal + 85);
        if (rightMotorSpeed > (speedNormal + 85))
            rightMotorSpeed = speedNormal + 85;


    }
    char dataSending[6];
    dataSending[0] = (char)((leftMotorSpeed & 0x0000FF00) >> 8);
    dataSending[1] = (char)(leftMotorSpeed & 0x000000FF);   // Inputting both bytes of leftMotorSpeed to the dataSending array of bytes (char)
    dataSending[2] = (char)((rightMotorSpeed & 0x0000FF00) >> 8);
    dataSending[3] = (char)(rightMotorSpeed & 0x000000FF);  // Inputting both bytes of rightMotorSpeed to the dataSending array of bytes (char)
    dataSending[4] = (char)((steeringAngle & 0x0000FF00) >> 8);
    dataSending[5] = (char)(steeringAngle & 0x000000FF);    // Inputting both bytes of steeringAngle to the dataSending array of bytes (char)
    ESP32.i2cWrite(dataSending,6);
}
void lineLoop()
{
    long pixelNum [10];
    int pixelNumber;
    int colPointer = 0;



    while (1)
    {
        Mat frame;
        while(frame.empty())
            frame = captureFrame(); // Capture a frame from the camera and store in a new matrix variable
        cv::flip(frame,frame,-1);

        Mat frameHSV;       // Convert the frame to HSV and apply the limits
        Mat frameBin;


        cv::cvtColor(frame, frameHSV, COLOR_BGR2HSV);
        cv::inRange(frameHSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), frameHSV);
        cv::threshold(frameHSV, frameBin, 1, 255, THRESH_BINARY);

        std::vector<Vec4i> hierarchy;
        for (int c = 0; c < frameBin.cols; c +=32)
        {
            cv::Mat tile = frameBin(cv::Range(200, min(240, frameBin.rows)), cv::Range(c, min(c + 32, frameBin.cols)));//no data copying here
            //cv::Mat tileCopy = img(cv::Range(r, min(r + N, img.rows)),
            //cv::Range(c, min(c + N, img.cols))).clone();//with data copying

            //tile can be smaller than NxN if image size is not a factor of N
            pixelNum[colPointer] = cv::countNonZero(tile);
            colPointer ++;
        }
        colPointer = 0;
        int key = cv::waitKey(1);   // Wait 1ms for a keypress (required to update windows)
        int xS = 0;
        double sumVal = 0;
        for (int i = 0; i < 10; i++)
        {
            xS += (i + 1) * pixelNum[i];
            sumVal += pixelNum[i];
        }

        float weightedAvg = (float)xS / (float)sumVal;
        //Setpoint is 3.5, inbetween the 3rd and 4th LED
        //Calculating PID u error
        currentError = 5.5 - weightedAvg;
        if (cv::countNonZero(frameBin) > 100)
        {
            PIDNum = (Kp * currentError) + (Ki * errorSum) + (Kd * (currentError - prevError));
            prevError = currentError;
            errorSum += currentError;
            runMotors((speedNormal - (0.85 * PIDNum)), (speedNormal + (0.85 * PIDNum)), (steerCenter - PIDNum));
            std::cout << "runMotors" << std::endl;
            //runMotors(0,0,(steerCenter - PIDNum));
        }

        else if (cv::countNonZero(frameBin)<25)
        {
            break;
        }
        key = cv::waitKey(1);
        key = (key==255) ? -1 : key;    // Check if the ESC key has been pressed
        if (key == 27)
        {
            runMotors(0,0,steerCenter);
            break;
        }
        Mat img;
        cv::cvtColor(frameHSV, frameHSV, COLOR_GRAY2BGR);   // In range returns the equivalent of a grayscale image so we need to convert this before concatenation
        cv::hconcat(frame, frameHSV, img);
        cv::imshow("Coloured Line",img);

    }
}

float symbol_check(Mat image, Mat trans, Mat cam)
{
    std::vector<float> matcher;
    cv::cvtColor(image, image, COLOR_BGR2GRAY);   // In range returns the equivalent of a grayscale image so we need to convert this before concatenation
    cv::threshold(image, image, 170, 255, THRESH_BINARY_INV);

    //Find contours in new transformed image
    std::vector<std::vector<cv::Point>> trans_contours;
    cv::findContours(trans, trans_contours, RETR_CCOMP, CHAIN_APPROX_SIMPLE);


    //Find contours in symbol that is checked
    std::vector<std::vector<cv::Point>> symbol_contours;
    float matcherSum =0;

    cv::findContours(image, symbol_contours, RETR_CCOMP, CHAIN_APPROX_SIMPLE);


    for(int i = 0; i<symbol_contours.size(); i++)
    {
        if(symbol_contours.size()==trans_contours.size())
        {

            matcher.push_back (cv::matchShapes(trans_contours[i],symbol_contours[i],1,0.0));
            matcherSum+=matcher[i];

        }



    }

    float matcherAve = matcherSum/matcher.size();
    matcher.clear();

    return matcherAve;
}
int main( int argc, char** argv )
{
    setup();    // Call a setup function to prepare IO and devices
    cv::namedWindow("HSV Tester");   // Create a GUI window called photo
    cv::namedWindow("Pink Symbol");
    lowH = 111, highH = 179, lowS = 10, highS = 148, lowV = 0, highV = 255;    // Initialise some variables for HSV limits
    int pinkLowH = 111, pinkHighH = 179, pinkLowS = 10, pinkHighS = 148, pinkLowV = 0, pinkHighV =255;

    cv::createTrackbar("Low Hue", "HSV Tester", &pinkLowH, 179, NULL);      // Create trackbar controls for each HSV limit
    cv::createTrackbar("High Hue", "HSV Tester", &pinkHighH, 179, NULL);

    cv::createTrackbar("Low Sat", "HSV Tester", &pinkLowS, 255, NULL);
    cv::createTrackbar("High Sat", "HSV Tester", &pinkHighS, 255, NULL);

    cv::createTrackbar("Low Value", "HSV Tester", &pinkLowV, 255, NULL);
    cv::createTrackbar("High Value", "HSV Tester", &pinkHighV, 255, NULL);

    //cv::createTrackbar("Colour Selector", "HSV Tester", &colourPicker, 4, NULL);
    cv::namedWindow("Photo");   // Create a GUI window called photo

    //PID Numbers
    colourPicker(4);
    long pixelNum [10];
    int pixelNumber;
    int colPointer = 0; // nums for image processing
    while(1)    // Main loop to perform image processing
    {
        std::cout << "Motor Flag" << motorsFlag << std::endl;
        Mat frame;
        while(frame.empty())
            frame = captureFrame(); // Capture a frame from the camera and store in a new matrix variable
        cv::flip(frame,frame,-1);
        // cv::imshow("Photo", frame); //Display the image in the window
        pinkLowH = cv::getTrackbarPos("Low Hue", "HSV Tester");        // Update the variables with the trackbar setting
        pinkHighH = cv::getTrackbarPos("High Hue", "HSV Tester");
        pinkLowS = cv::getTrackbarPos("Low Sat", "HSV Tester");
        pinkHighS = cv::getTrackbarPos("High Sat", "HSV Tester");
        pinkLowV = cv::getTrackbarPos("Low Value", "HSV Tester");
        pinkHighV = cv::getTrackbarPos("High Value", "HSV Tester");
        //colourPicker = cv::getTrackbarPos("Colour Selector", "HSV Tester");

        Mat frameHSV;       // Convert the frame to HSV and apply the limits
        Mat frameBin;
        Mat pinkIm;
        Mat pinkBin;

        std::vector<std::vector<Point>> pinkCon;

        cv::cvtColor(frame, frameHSV, COLOR_BGR2HSV);
//        std::vector<Mat> channels;
//        cv::split(frameHSV,channels);
//        cv::equalizeHist(channels[2], channels[2]); // Equalise the Value channel
//        cv::merge(channels, frameHSV); // Merge back into a single image
//        cvtColor(frameHSV, frameEQ,COLOR_HSV2BGR); // Convert back to BGR



        cv::inRange(frameHSV, Scalar(pinkLowH, pinkLowS, pinkLowV), Scalar(pinkHighH, pinkHighS, pinkHighV), pinkIm);
        cv::inRange(frameHSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), frameHSV);


        cv::threshold(frameHSV, frameBin, 1, 255, THRESH_BINARY);
        cv::threshold(pinkIm, pinkBin, 1, 255, THRESH_BINARY);
        std::vector<Vec4i> hierarchy;
        cv::findContours(pinkBin,pinkCon,RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        //find largest of the countours
        int largestContourIndex = -1;
        double largestContourArea = 0;
        for (int i = 0; i < pinkCon.size(); i++)
        {
            double area = cv::contourArea(pinkCon[i]);
            if (area > largestContourArea)
            {
                largestContourIndex = i;
                largestContourArea = area;
            }
        }
        for (int c = 0; c < frameBin.cols; c +=32)
        {
            cv::Mat tile = frameBin(cv::Range(200, min(240, frameBin.rows)), cv::Range(c, min(c + 32, frameBin.cols)));//no data copying here
            //cv::Mat tileCopy = img(cv::Range(r, min(r + N, img.rows)),
            //cv::Range(c, min(c + N, img.cols))).clone();//with data copying

            //tile can be smaller than NxN if image size is not a factor of N
            pixelNum[colPointer] = cv::countNonZero(tile);
            colPointer ++;
        }




        colPointer = 0;
        int key = cv::waitKey(1);   // Wait 1ms for a keypress (required to update windows)
        int xS = 0;
        double sumVal = 0;
        for (int i = 0; i < 10; i++)
        {
            xS += (i + 1) * pixelNum[i];
            sumVal += pixelNum[i];
        }
        if (sumVal > 100 )
        {
            float weightedAvg = (float)xS / (float)sumVal;
            //Setpoint is 3.5, inbetween the 3rd and 4th LED
            //Calculating PID u error
            currentError = 5.5 - weightedAvg;
            PIDNum = (Kp * currentError) + (Ki * errorSum) + (Kd * (currentError - prevError));
            prevError = currentError;
            errorSum += currentError;
            if(motorsFlag > 0)
            {
                runMotors((speedNormal - (0.85 * PIDNum)), (speedNormal + (0.85 * PIDNum)), (steerCenter - PIDNum));

            }
            if(motorsFlag <= 0)
            {
                runMotors(0,0,steerCenter);

            }
        }

//        std::cout << PIDNum << "\t runMotors \n" << std::endl;

        Mat img;     // Join the two into a single image
        Mat contours;
        std::vector<std::vector<cv::Point>>approxedcontours(pinkCon.size());


        cv::cvtColor(frameHSV, frameHSV, COLOR_GRAY2BGR);   // In range returns the equivalent of a grayscale image so we need to convert this before concatenation

        cv::vconcat(frame, frameHSV, img);

        std::vector<cv::Point2f> approx;
        for (int i = 0; i < pinkCon.size(); i++)
        {
            if (pinkCon[i].size()>4)
            {
                cv::approxPolyDP(pinkCon[i], approx, cv::arcLength(pinkCon[i], true) * 0.02, true);
            }

            if (approx.size() == 4)
            {
                break;
            }
        }



        cv::drawContours(img, pinkCon, largestContourIndex, Scalar(0, 0, 255),2);

        cv::Mat dst(350, 350, CV_8UC3, cv::Scalar(0,0,0));
        std::cout << "pink pixel \t" << cv::countNonZero(pinkBin) << std::endl;



        if(cv::countNonZero(pinkBin) > 100)
        {

            Mat pink_crop = pinkIm;



            std::vector<cv::Point2f> src_pts, dst_pts;
            src_pts.push_back(approx[0]);
            src_pts.push_back(approx[1]);
            src_pts.push_back(approx[2]);
            src_pts.push_back(approx[3]);
            dst_pts.push_back(cv::Point2f(0,0));
            dst_pts.push_back(cv::Point2f(349,0));
            dst_pts.push_back(cv::Point2f(349,349));
            dst_pts.push_back(cv::Point2f(0,349));
            cv::Mat transform = cv::getPerspectiveTransform(src_pts, dst_pts);
            cv::warpPerspective(pink_crop, dst, transform, dst.size());
            cv::imshow("transformed", dst); //Display the image in the window
            Mat circle = cv::imread("circle.png");
            Mat star = cv::imread("star.png");
            Mat triangle = cv::imread("triangle.png");
            Mat umbrella = cv::imread("umbrella.png");
            float circleCheck = symbol_check(circle, dst, img);
            float starCheck = symbol_check(star, dst, img);
            float triangleCheck = symbol_check(triangle, dst, img);
            float umbrellaCheck = symbol_check(umbrella, dst, img);
            std::cout << "Circle Check \t" << circleCheck << std::endl;
            std::cout << "Star Check \t" << starCheck << std::endl;
            std::cout << "Triangle Check \t" << triangleCheck << std::endl;

            if (!std::isnan(circleCheck))
            {

                if (circleCheck < 0.002 )
                {

                    cv::putText (img, "Circle is Red Line", cv::Point(100,200), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0));
                    colourPicker(0);

                    lineLoop();
                }

            }
            if (!std::isnan(starCheck))
            {

                if (starCheck < 0.015 )
                {

                    cv::putText (img, "Star is Green Line", cv::Point(100,200), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0));
                    colourPicker(1);
                    lineLoop();
                }

            }
            if (!std::isnan(triangleCheck))
            {

                if (triangleCheck < 0.015 )
                {

                    cv::putText (img, "Triangle is Blue Line", cv::Point(100,200), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0));
                    colourPicker(2);
                    std::cout << "triangle" << std::endl;
                    lineLoop();
                }


            }
            if (!std::isnan(umbrellaCheck))
            {

                if (umbrellaCheck < 0.027 )
                {

                    cv::putText (img, "Umbrella is Yellow Line", cv::Point(100,200), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0));
                    colourPicker(3);
                    lineLoop();
                }

            }

            cv::imshow("Pink Symbol", pink_crop);

        }
        else
        {
            cv::putText (img, "Following black line", cv::Point(100,200), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0));
            colourPicker(4);

        }


        cv::imshow("HSV Tester", img); //Display the image in the window












        key = (key==255) ? -1 : key;    // Check if the ESC key has been pressed
        if (key == 27)
        {
            runMotors(0,0,steerCenter);
            break;
        }
    }

    closeCV();  // Disable the camera and close any windows

    return 0;
}

