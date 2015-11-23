#include <cv.h>
#include <highgui.h>
#include <iostream>

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
    Mat image;
    image = imread("/home/moro/ros_workcell/src/rsd3_ROS_workcell_stack/brick_check/res/all.png", CV_LOAD_IMAGE_COLOR);   // Read the file

    if(! image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

    //Rect ROI(image.cols/2-120-10 ,image.rows-150, 240, 150);
    //image = image(ROI);

    Mat hsv, red, blue, yellow;
    vector<Mat> channels(3);
    cvtColor(image, hsv, CV_BGR2HSV);
    split(hsv, channels);

    threshold(channels[0], red, 10, 255, THRESH_BINARY_INV);
    threshold(channels[0], blue, 100, 255, THRESH_BINARY);
    threshold(channels[0], yellow, 35, 255, THRESH_BINARY_INV);
    threshold(yellow, yellow, 60, 255, THRESH_BINARY);

    double total = channels[0].cols * channels[0].rows;

    cout << "Red: " << countNonZero(red)/total << endl;
    cout << "Blue: " << countNonZero(blue)/total << endl;
    cout << "Yellow: " << countNonZero(yellow)/total << endl;

    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", red );                   // Show our image inside it.
    waitKey(0);
    imshow("Display window", blue);
    waitKey(0);
    imshow("Display window", yellow);

    waitKey(0);                                          // Wait for a keystroke in the window
    return 0;
}
