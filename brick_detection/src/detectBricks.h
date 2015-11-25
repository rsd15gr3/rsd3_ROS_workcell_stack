#include <cv.h>
#include <highgui.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <ctime>
#include <math.h>

using namespace cv;

const double pixWidth=0.000289;

struct brick {
	Point2f center;
	double orientation;
	string color;
};

//Counts how many pixels in image img that has value val, within the rectangle defined by points pt1 and pt2
int countpixelVal(Mat img, int val, Point pt1, Point pt2){
    int pixels=0;
    for(int x=pt1.x;x< pt2.x;x++){
        for(int y=pt1.y; y<pt2.y;y++){
            if((int)img.at<uchar>( y,x)==val){
                pixels++;
            }
        }
    }
    return pixels;
}

//Calculates center of mass for all pixelvalues val in image src
Point centerOfMass(Mat src, int val){
    Point p;
    p.x=0;
    p.y=0;
    int numPixels = countpixelVal(src, val,Point(0,0),Point(src.cols-1,src.rows-1));
    int sumX=0, sumY=0;
    if(numPixels>0){
        for(int x=0;x< src.cols;x++){
            for(int y=0; y<src.rows;y++){
                if((int)src.at<uchar>(y, x)==val){
                    sumX=sumX+x;
                    sumY=sumY+y;
                }
            }
        }
        //std::cout<<sumX<<", "<<numPixels<<std::endl;
        p.x=sumX/numPixels;
        p.y=sumY/numPixels;
    }
    return p;
}

//Detects bricks
std::vector<brick> detectBrick(Mat src){

	Mat dst=src;
	//Closing of the image and medianblur helps smooth away light reflections on bricks
	int i=6;
	Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 2*i + 1, 2*i + 1 ), Point( i, i ));
	morphologyEx( src, dst, MORPH_CLOSE, element);
	medianBlur( src, dst, 4*i+1);

	const float meanTol(0.4*255);

	Mat_<Vec3f> original = dst.clone();
	Mat r = Mat::zeros(original.size(),CV_8UC1);
	Mat g = Mat::zeros(original.size(),CV_8UC1);
	Mat b = Mat::zeros(original.size(),CV_8UC1);

	Mat redMean=(Mat_<float>(3, 1) << 50.673786, 59.929203, 231.64235);
	Mat yellowMean=(Mat_<float>(3, 1) << 117.14671, 245.57028, 245.99557);
	Mat blueMean=(Mat_<float>(3, 1) << 193.45268, 104.25221, 17.144825);

	for(int x=0;x< original.cols;x++){
		for(int y=0; y<original.rows;y++){
			Vec3f& bgr = original(y,x);
			Mat_<float> bgrm(bgr,false);
			if(norm(bgrm-redMean)<=meanTol){
				r.at<uchar>(y,x)=255;
			}
			else{
				r.at<uchar>(y,x)=0;
			}
			if(norm(bgrm-yellowMean)<=meanTol){
				g.at<uchar>(y,x)=255;
			}
			else{
				g.at<uchar>(y,x)=0;
			}
			if(norm(bgrm-blueMean)<=meanTol){
				b.at<uchar>(y,x)=255;
			}
			else{
				b.at<uchar>(y,x)=0;
			}
		}
	}

	//Remove bricks that extend out the region of interest in order to avoid miscalculations
	for(int y=0;y< dst.rows;y++){
		if((int)r.at<uchar>(y,0)>0){
            floodFill(r, Point(0,y), 0);
		}
		if((int)r.at<uchar>(y,r.cols-1)>0){
		    floodFill(r, Point(r.cols-1,y), 0);
		}
		if((int)g.at<uchar>(y,0)>0){
            floodFill(g, Point(0,y), 0);
		}
		if((int)g.at<uchar>(y,g.cols-1)>0){
		    floodFill(g, Point(g.cols-1,y), 0);
		}
		if((int)b.at<uchar>(y,0)>0){
            floodFill(b, Point(0,y), 0);
		}
		if((int)b.at<uchar>(y,b.cols-1)>0){
		    floodFill(b, Point(b.cols-1,y), 0);
		}
	}

	//Fills the legobricks with individual color to seperate the bricks and count how many detected bricks
	int red_replacementcolor=1;
	int green_replacementcolor=1;
	int blue_replacementcolor=1;
	for(int x=0;x< r.cols;x++){
        for(int y=0; y<r.rows;y++){
            if((int)r.at<uchar>(y, x)==255){
            	//fills replacement color
                floodFill(r, Point(x,y), red_replacementcolor);
                //if theres less pixels of this replacement color than 2500 then it is too small to be a brick
                if(countpixelVal(r,red_replacementcolor,Point(0,0), Point(r.cols,r.rows))<2500){
                	//therefore blacks it to erase
                	floodFill(r, Point(x,y), 0);
                }
                //else if there's more pixels than 2500 then it's a brick and replacementcolor is incremented
                else
                {
                	red_replacementcolor++;
                }
            }
            //same for green which will correspond to yellow brick
            if((int)g.at<uchar>(y, x)==255){
            	floodFill(g, Point(x,y), green_replacementcolor);
                if(countpixelVal(g,green_replacementcolor,Point(0,0), Point(g.cols,g.rows))<2500){
                	floodFill(g, Point(x,y), 0);
                }
                else
                {
                	green_replacementcolor++;
                }
           	}
            //and blue
            if((int)b.at<uchar>(y, x)==255){
            	floodFill(b, Point(x,y), blue_replacementcolor);
                if(countpixelVal(b,blue_replacementcolor,Point(0,0), Point(b.cols,b.rows))<2500){
                	floodFill(b, Point(x,y), 0);
                }
                else
                {
                	blue_replacementcolor++;
                }
           	}
        }
	}

	//Declarations of Vecf4 arrays for storing orientations of each brick
	std::vector<Vec4f> red_lines;
	std::vector<Vec4f> green_lines;
	std::vector<Vec4f> blue_lines;

	//Declarations of Point arrays for storing location of each brick
	std::vector<Point> red_points;
	std::vector<Point> green_points;
	std::vector<Point> blue_points;

	//Finds location and orientation of each brick
	if(red_replacementcolor>0){
		//The current value of replacementcolor-1 is equal to amount of bricks found
		//n increments through all detected red bricks, first brick will have pixelvalue 1, second 2, etc...
		for(int n=1;n<red_replacementcolor;n++){

			//Finds center of mass for all pixels of value n, all those pixels will belong to the same brick
			//The center of mass of pixel n is thus the location of the n'th red brick in the image
			red_points.push_back(centerOfMass(r,n));

			//All the pixels of a particular red brick is stored as points in a point array
			//int nn=0;
			//int numPix = countpixelVal(r, n, Point(0,0), Point(r.cols,r.rows));
			vector<Point> m;
			for(int x=0;x< r.cols;x++){
		        for(int y=0; y<r.rows;y++){
		            if((int)r.at<uchar>(y,x)==n){
		            	m.push_back(Point(x,y));
		            	//nn++;
		            }
		        }
			}
			//fitLine does linear regression on all points in the point array to find the best line through those points
			//The line will have the same orientation as the brick
			Vec4f line;
			fitLine(m,line,CV_DIST_L2,0,0.1,0.1);
			//puts resulting line in array
			red_lines.push_back(line);
		}
	}
	//Same for yellow brick
	if(green_replacementcolor>0){
		for(int n=1;n<green_replacementcolor;n++){
			green_points.push_back(centerOfMass(g,n));
			//int nn=0;
			//int numPix = countpixelVal(g, n, Point(0,0), Point(g.cols,g.rows));
			vector<Point> m;
			for(int x=0;x< g.cols;x++){
		        for(int y=0; y<g.rows;y++){
		            if((int)g.at<uchar>(y,x)==n){
		            	m.push_back(Point(x,y));
		            	//nn++;
		            }
		        }
			}
			Vec4f line;
			fitLine(m,line,CV_DIST_L2,0,0.1,0.1);
			green_lines.push_back(line);
		}
	}
	//...and blue
	if(blue_replacementcolor>0){
		for(int n=1;n<blue_replacementcolor;n++){
			blue_points.push_back(centerOfMass(b,n));
			//int nn=0;
			//int numPix = countpixelVal(b, n, Point(0,0), Point(b.cols,b.rows));
			vector<Point> m;
			for(int x=0;x< b.cols;x++){
		        for(int y=0; y<b.rows;y++){
		            if((int)b.at<uchar>(y,x)==n){
		            	m.push_back(Point(x,y));
		            	//nn++;
		            }
		        }
			}
			Vec4f line;
			fitLine(m,line,CV_DIST_L2,0,0.1,0.1);
			blue_lines.push_back(line);
		}
	}

	//Puts all bricks in vector of the brick struct
	//int numberOfBricks=red_lines.size()+green_lines.size()+blue_lines.size();
	std::vector<brick> detected;
	for(int i = 0; i< (int)red_lines.size(); i++){
		brick red_found;
		red_found.center=red_points[i];
		red_found.center.x=(red_found.center.x-360)*pixWidth;
		red_found.center.y=(310-red_found.center.y)*pixWidth;
		red_found.color="red";
		double tanred = (double)red_lines[i][1]/(double)red_lines[i][0];
		red_found.orientation=atan(tanred)-(M_PI/2);
		if(red_found.orientation<-(M_PI/2)){
			red_found.orientation=red_found.orientation+M_PI;
		}
		if(red_found.orientation>(M_PI/2)){
			red_found.orientation=red_found.orientation-M_PI;
		}
		detected.push_back(red_found);
	}
	for(int i = 0; i< (int)green_lines.size(); i++){
		brick yellow_found;
		yellow_found.center=green_points[i];
		yellow_found.center.x=(yellow_found.center.x-360)*pixWidth;
		yellow_found.center.y=(310-yellow_found.center.y)*pixWidth;
		yellow_found.color="yellow";
		double tanyellow = (double)green_lines[i][1]/(double)green_lines[i][0];
		yellow_found.orientation=atan(tanyellow)-(M_PI/2);
		if(yellow_found.orientation<-(M_PI/2)){
			yellow_found.orientation=yellow_found.orientation+M_PI;
		}
		if(yellow_found.orientation>(M_PI/2)){
			yellow_found.orientation=yellow_found.orientation-M_PI;
		}
		detected.push_back(yellow_found);
	}
	for(int i = 0; i< (int)blue_lines.size(); i++){
		brick blue_found;
		blue_found.center=blue_points[i];
		blue_found.center.x=(blue_found.center.x-360)*pixWidth;
		blue_found.center.y=(310-blue_found.center.y)*pixWidth;
		blue_found.color="Blue";
		double tanblue = (double)blue_lines[i][1]/(double)blue_lines[i][0];
		blue_found.orientation=atan(tanblue)-(M_PI/2)+(M_PI/2);
		if(blue_found.orientation<-(M_PI/2)){
			blue_found.orientation=blue_found.orientation+M_PI;
		}
		if(blue_found.orientation>(M_PI/2)){
			blue_found.orientation=blue_found.orientation-M_PI;
		}
		if(blue_found.orientation>-0.01&&blue_found.orientation<0.01){
			blue_found.orientation=blue_found.orientation-M_PI/2;
		}
		detected.push_back(blue_found);
	}

	return detected;
}

