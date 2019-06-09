#include <iostream>
#include "opencv2/opencv.hpp"
// #include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "math.h"
// #include "opencv2/imgcodecs.hpp"
// #include "opencv2/highgui.hpp"

// #include "fast_line_detector.hpp"
using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    VideoCapture cap(1); // open the default camera
    if (!cap.isOpened()) // check if we succeeded
        return -1;
    Mat frame, edges, blur, gray, res, thresh;
    // namedWindow("frame");
    // namedWindow("blur");
    // namedWindow("edges");
    // namedWindow("gray");
    // frame = imread("./bl1.png");
    int wer = (int)argv[1][0] - 48;
    cout << wer << endl;
    while (true)
    {
        // for (int wer = 0; wer < 9; wer++)
        // {
        // in = parser.get<string>("@input");
        // frame = imread("./bl1.png");
        cap >> frame;
        res = frame;
        cv::Size frame_size = frame.size();
        // gray = frame.clone();
        // if( frame.empty() )
        // {
        //     return -1;
        // }
        imshow("frame", frame);
        cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        // imshow("gray", gray);
        GaussianBlur(gray, blur, Size(3, 3), 1.2, 1.2);
        // medianBlur(gray, blur, 3);
        // vector<vector<Point> > contours;
        // vector<Vec4i> hierarchy;
        // threshold( gray, thresh, 95, 255,cv::THRESH_BINARY_INV );
        // findContours( thresh, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, Point(0, 0) );
        // for( size_t i = 0; i< contours.size(); i++ )
        // {

        //     drawContours( res, contours, (int)i, Scalar( 0, 255, 0), 2, 8, hierarchy, 0, Point() );

        // }
        // imshow("blur", blur);
        int erosion_size = 3;
        int erosion_type = MORPH_CROSS;
        Mat element = getStructuringElement( erosion_type,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );
        // erode( blur, blur, element );
        imshow("blur", blur);
        Canny(blur, edges, 95, 200);
        // Specify size on vertical axis
        // int vertical_size = edges.rows / 80;
        // // Create structure element for extracting vertical lines through morphology operations
        // Mat verticalStructure = getStructuringElement(MORPH_RECT, Size(1, vertical_size));
        // // Apply morphology operations
        // erode(edges, edges, verticalStructure, Point(-1, -1));
        // dilate(edges, edges, verticalStructure, Point(-1, -1));
        
        
        imshow("edges", edges);

        vector<Vec4i> linesP; // will hold the results of the detection
        vector<Vec4i> linesS;
        HoughLinesP(edges, linesP, 2, CV_PI / 180, 20, 20, 50); // runs the actual detection
        // vector<Vec2f> lines;
        // HoughLines(edges, lines, 1, CV_PI / 180, 130, 0, 0);
        // Draw the lines
        for (size_t i = 0; i < linesP.size(); i++)
        {
            Vec4i l = linesP[i];
            Point p1, p2;
            p1 = Point(l[0], l[1]);
            p2 = Point(l[2], l[3]);
        // for (size_t i = 0; i < lines.size(); i++)
        // {
            // calculate angle in radian,  if you need it in degrees just do angle * 180 / PI
            // float rho = lines[i][0], theta = lines[i][1];
            // Point p1, p2;
            // double a = cos(theta), b = sin(theta);
            // double x0 = a * rho, y0 = b * rho;
            // p1.x = cvRound(x0 + 1000 * (-b));
            // p1.y = cvRound(y0 + 1000 * (a));
            // p2.x = cvRound(x0 - 1000 * (-b));
            // p2.y = cvRound(y0 - 1000 * (a));
            // Vec4i l;
            // l[0] = p1.x;
            // l[1] = p1.y;
            // l[2] = p2.x;
            // l[3] = p2.y;
            // circle(res, Point(a, b), 10, Scalar(255, 0, 0), CV_FILLED, 8, 0);
            // circle(res, Point(x0, y0), 10, Scalar(255, 0, 0), CV_FILLED, 8, 0);
            float angle = atan2(p1.y - p2.y, p1.x - p2.x) * 180 / CV_PI;
            // cout << angle << endl;
            // float a_qwe = abs(frame_size.height - y0);

            if (abs(abs(angle) - wer * 45) < 30)
            {
                int length = cv::norm(p2 - p1);
                if (length > 70)
                {
                    linesS.insert(linesS.end(), l);

                    // circle(frame, l2_p1,5, Scalar(0,255,0),  1, LINE_AA);
                    
                    line(res, p1, p2, Scalar(0, 0, 255), 3, LINE_AA);
                    // circle(res, p1,7, Scalar(0,255,0),  -1);
                    // circle(res, p2,7, Scalar(0,255,0),  -1);
                    circle(res, (p1 + p2) / 2, 10, Scalar(255, 0, 255), CV_FILLED, 8, 0);
                    circle(res, p1, 10, Scalar(255, 255, 0), CV_FILLED, LINE_AA);
                    circle(res, p2, 10, Scalar(255, 255, 0), CV_FILLED, LINE_AA);
                    // circle(res, p2 - Point(11, 11),10, Scalar(255,0,255),CV_FILLED, 8,0);
                }
            }
        }
        float median_x1 = 0;
        float median_x2 = 0;
        // TODO: remove 2 times check
        int q = 0;
        // cout << "                                                     " << linesS.size() << endl;
        vector<Point> median_points;
        Vec4i median_line;
        for (size_t i = 0; i < linesS.size(); i++)
        {
            // if (linesS.size()-i < i+1)
            //     break;
            Vec4i l1 = linesS[i];
            Point l1_p1, l1_p2;

            l1_p1 = Point(l1[0], l1[1]);
            l1_p2 = Point(l1[2], l1[3]);
            for (size_t j = 0; j < linesS.size(); j++)
            {
                // if (linesS.size()-i == i)
                //     continue;
                // if (linesS.size() - i - 1 < i)
                // if (true)
                // {
                // Vec4i l2 = linesS[linesS.size() - i - 1];
                Vec4i l2 = linesS[j];
                Point l2_p1, l2_p2;

                l2_p1 = Point(l2[0], l2[1]);
                l2_p2 = Point(l2[2], l2[3]);
                cout << "                                                         " << abs(l1_p1.x - l2_p1.x) << endl;
                // circle(res, l1_p1,7, Scalar(0,255,0),  -1);
                // circle(res, l1_p2,7, Scalar(0,255,0),  -1);
                // circle(res, l2_p1,7, Scalar(0,255,0),  -1);
                // circle(res, l2_p2,7, Scalar(0,255,0),  -1);

                if (
                    abs(cv::norm(l1_p1 - l2_p1)) < 50 &&
                    // abs(l1_p2.x - l2_p2.x) < 150 &&
                    abs(cv::norm(l1_p1 - l2_p1)) > 5
                    // abs(l1_p2.x - l2_p2.x) > 15
                )
                {
                    // median_points.push_back(Point(l1[0], l1[1]));
                    // median_points.push_back(Point(l1[2], l1[3]));
                    // median_points.push_back(Point(l2[0], l2[1]));
                    // median_points.push_back(Point(l2[2], l2[3]));
                    circle(res, l1_p1, 10, Scalar(255, 0, 0), CV_FILLED, 8, 0);
                    circle(res, l1_p2, 10, Scalar(255, 0, 0), CV_FILLED, 8, 0);
                    circle(res, l2_p1, 10, Scalar(255, 0, 0), CV_FILLED, 8, 0);
                    circle(res, l2_p2, 10, Scalar(255, 0, 0), CV_FILLED, 8, 0);
                    int x_1 = (l1_p1.x + l2_p1.x) / 2;
                    int y_1 = (l1_p1.y + l2_p1.y) / 2;
                    int x_2 = (l1_p2.x + l2_p2.x) / 2;
                    int y_2 = (l1_p2.y + l2_p2.y) / 2;
                    median_points.push_back(Point(x_1, y_1));
                    median_points.push_back(Point(x_2, y_2));
                    // median_points.push_back(Point(l2[0], l2[1]));
                    // median_points.push_back(Point(l2[2], l2[3]));
                    // // median_x1 = ((l1_p1.x + median_x1)/2 + l2_p1.x) /2;
                    // // median_x2 = ((l2_p2.x + median_x2)/2 + l2_p2.x) /2;
                    // // q++;q++;
                    line(res, Point(x_1, y_1), Point(x_2, y_2), Scalar(0, 255, 0), 3, LINE_AA);
                }
            }
            // }g
        }
        cout << "x2:" << median_points << endl;
        if (median_points.size() > 0)
        {
            cv::fitLine(median_points, median_line, cv::DIST_L2, 0, 0.01, 0.01);
            cout << "x1:" << median_line << endl;
            float righty, lefty;
            // if (median_line[0] != 0)
            // {
            //     lefty = (-median_line[2] * median_line[1] / median_line[0]) + median_line[3];
            //     righty = ((frame_size.width - median_line[2]) * median_line[1] / median_line[0]) + median_line[3];
            // }
            // else
            // {
            //     lefty = (median_line[2] * median_line[1]) + median_line[3];
            //     righty = -((frame_size.width - median_line[2]) * median_line[1]);
            // }
            int x0 = median_line[2]; // a point on the line
            int y0 = median_line[3];
            int x1 = x0 - 200 * median_line[0]; // add a vector of length 200
            int y1 = y0 - 200 * median_line[1];

            // int median_m = median_line[1] / median_line[0];
            // Point median_b = cv::Point(median_line[2], median_line[3]);
            // double median_ini_x = ((ini_y - left_b.y) / left_m) + left_b.x;
            // double median_fin_x = ((fin_y - left_b.y) / left_m) + left_b.x;

            // output[0] = cv::Point(right_ini_x, ini_y);
            // output[1] = cv::Point(right_fin_x, fin_y);
            // oPoint median_st = cv::Point(median_ini_x, ini_y);
            // output[3] = cv::Point(left_fin_x, fin_y);

            // median_x1 = median_x1 / q;
            // median_x2 = median_x2 / q;

            line(res, Point(x0, y0), Point(x1, y1), Scalar(0, 255, 255), 3, LINE_AA);
        }
        imshow("res", res);
        waitKey(1);
    }
    return 0;
}
// using namespace cv::ximgproc;
/*
int main(int argc, char** argv)
{
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;
    Mat image;
    while (true){
        // in = parser.get<string>("@input");
        // Mat image = imread("./b3.jpeg", IMREAD_GRAYSCALE);
        cap >> image;
        if( image.empty() )
        {
            return -1;
        }
        cvtColor(image, image, cv::COLOR_BGR2GRAY);
        // Create LSD detector
        Ptr<LineSegmentDetector> lsd = createLineSegmentDetector();
        vector<Vec4f> lines_lsd;
        // Create FLD detector
        // Param               Default value   Description
        // length_threshold    10            - Segments shorter than this will be discarded
        // distance_threshold  1.41421356    - A point placed from a hypothesis line
        //                                     segment farther than this will be
        //                                     regarded as an outlier
        // canny_th1           50            - First threshold for
        //                                     hysteresis procedure in Canny()
        // canny_th2           50            - Second threshold for
        //                                     hysteresis procedure in Canny()
        // canny_aperture_size 3             - Aperturesize for the sobel
        //                                     operator in Canny()
        // do_merge            false         - If true, incremental merging of segments
        //                                     will be perfomred
        int length_threshold = 10;
        float distance_threshold = 1.41421356f;
        double canny_th1 = 50.0;
        double canny_th2 = 50.0;
        int canny_aperture_size = 3;
        bool do_merge = false;
        Ptr<cv::ximgproc::FastLineDetector> fld = cv::ximgproc::createFastLineDetector(length_threshold,
                distance_threshold, canny_th1, canny_th2, canny_aperture_size,
                do_merge);
        vector<Vec4f> lines_fld;
        // Because of some CPU's power strategy, it seems that the first running of
        // an algorithm takes much longer. So here we run both of the algorithmes 10
        // times to see each algorithm's processing time with sufficiently warmed-up
        // CPU performance.
        for(int run_count = 0; run_count < 10; run_count++) {
            // lines_lsd.clear();
            // int64 start_lsd = getTickCount();
            // lsd->detect(image, lines_lsd);
            // Detect the lines with LSD
            double freq = getTickFrequency();
            // double duration_ms_lsd = double(getTickCount() - start_lsd) * 1000 / freq;
            // std::cout << "Elapsed time for LSD: " << duration_ms_lsd << " ms." << std::endl;
            lines_fld.clear();
            int64 start = getTickCount();
            // Detect the lines with FLD
            fld->detect(image, lines_fld);
            double duration_ms = double(getTickCount() - start) * 1000 / freq;
            std::cout << "Ealpsed time for FLD " << duration_ms << " ms." << std::endl;
        }
        // Show found lines with LSD
        // Mat line_image_lsd(image);
        // lsd->drawSegments(line_image_lsd, lines_lsd);
        // imshow("LSD result", line_image_lsd);
        // Show found lines with FLD
        Mat line_image_fld(image);
        fld->drawSegments(line_image_fld, lines_fld);
        imshow("FLD result", line_image_fld);
        waitKey(1);
    }
    return 0;
}
*/