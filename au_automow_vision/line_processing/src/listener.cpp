#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <cstdio>
#include <cstdlib>
#include <highgui.h>
#include <cv.h>

#define MAX_TRENDS 3
#define MAX_ANGLE_SEP 0.1745 // 0.1745 rad = 10 degrees

#define SUBTRACT_GREEN_RED(img) (cvSubS(img, cvScalar(0, 255, 255, 0), img, NULL))
#define THRESHOLD_IMAGE(img1, img2) (cvThreshold(img1, img2, maxPixVal-1, 255, CV_THRESH_BINARY))

sensor_msgs::CvBridge bridge_;

struct lineInfo {
	int total;
	float rho[MAX_TRENDS];
	float theta[MAX_TRENDS];
};

int cmp_theta(const void *_a, const void *_b, void *userdata)
{
	float *a = (float*)_a;
	float *b = (float*)_b;
	
	return a[1] > b[1] ? 1:a[1] < b[1] ? -1:0;
};

void test(int num) {printf("Test Point: %d\n", num);};

void displayLineInfo(struct lineInfo *lines){
	printf("total lines: %d\n", lines->total);
	
	for(int n=0; n< lines->total; n++) {
		printf("rho: %f\ttheta: %f\n", lines->rho[n], lines->theta[n]);
	}
	printf("\n");
};

void displayCvSeq(CvSeq *seq) {
	printf("total lines: %d\n", seq->total);
	
	for(int n=0; n < seq->total; n++) {
		float *line = (float*)cvGetSeqElem(seq, n);
		
		printf("rho: %f\ttheta: %f\n", line[0], line[1]);
	}
	
	printf("\n");
};

void drawLines(IplImage *img, struct lineInfo *lines_to_draw)
{
	for(int line_num = 0; line_num < lines_to_draw->total; line_num++)
	{
		CvPoint pt1, pt2;
		double a = cos(lines_to_draw->theta[line_num]), b = sin(lines_to_draw->theta[line_num]);
		double x0 = a*lines_to_draw->rho[line_num], y0 = b*lines_to_draw->rho[line_num];
		pt1.x = cvRound(x0 + 1500*(-b));
		pt1.y = cvRound(y0 + 1500*(a));
		pt2.x = cvRound(x0 - 1500*(-b));
		pt2.y = cvRound(y0 - 1500*(a));
		
		cvLine(img, pt1,pt2, CV_RGB(0,0,255), 3, CV_AA, 0);
	}
}

void displayLineDetection(char *window_name, IplImage *img, struct lineInfo *trend_lines) {
	drawLines(img, trend_lines);
	
	// display image
	cvShowImage(window_name, img);
};

void sortLinesByTheta(CvSeq *unsorted, CvSeq *sorted) {
	float *line;
	
	// correct for negative rho
	for(int n=0; n < unsorted->total; n++)
	{		
		line = (float*)cvGetSeqElem(unsorted, n);
		
		if(line[0] < 0)
		{
			line[0] = -line[0];
			line[1] = line[1] - CV_PI;
		}
		
		cvSeqPush(sorted, line);
	}
		
	cvSeqSort(sorted, cmp_theta, 0);
};

int maxPixelValue(IplImage *img) {
	int max = 0;
	for(int n=0; n < img->height; n++)
	{
		uchar *ptr = (uchar *)(img->imageData + n * img->widthStep);
		
		for(int k=0; k < img->width; k++)
		{
			if(max < *ptr)
				max = *ptr;
			
		}
	}
	
	return max;
};

CvSeq* findLinesInImage(IplImage *img, CvMemStorage *line_storage) {
	int maxPixVal;
	
	// data for hough
	float rho = 1;
	float theta = CV_PI/180;
	int threshold = 200;
	
	// image pointers
	IplImage *img_1chan = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
	IplImage *img_thresh = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
	
	CvSeq *lines;
		
	/////////////////////////////////////// Begin Line Detection /////////////////////////////////////////
	SUBTRACT_GREEN_RED(img);
	
	cvCvtColor(img, img_1chan, CV_RGB2GRAY);
	
	maxPixVal = maxPixelValue(img_1chan);
	
	THRESHOLD_IMAGE(img_1chan, img_thresh);
				
	lines = cvHoughLines2(img_thresh, line_storage, CV_HOUGH_STANDARD, rho, theta, threshold, 0, 0);
	
	cvNamedWindow("thresh", 1);
	cvShowImage("thresh", img_thresh);
			
	cvReleaseImage(&img_1chan);
	cvReleaseImage(&img_thresh);
	
	return lines;
};

void findTrendLines(CvSeq *raw_hough_lines, struct lineInfo *trend_lines, CvMemStorage *sorted_storage) {
	// create CvSeq for the sorted list of raw hough lines
	CvSeq *sorted_lines = cvCreateSeq(0, sizeof(CvSeq), 2*sizeof(float), sorted_storage);
	
	// averaging variables
	float *line;
	float rho_sum[MAX_TRENDS] = {0};
	float theta_sum[MAX_TRENDS] = {0};
	int cnt_avg[MAX_TRENDS] = {0};
	
	// trend variables
	int cnt_trend = 0;
	float theta_hold;
	int initialLine = 0;
			
	/////////////////////////////////////// Begin Trend Line Discovery /////////////////////////////////////////
	sortLinesByTheta(raw_hough_lines, sorted_lines);
					
	for(int n=0; n < sorted_lines->total; n++)
	{		
		line = (float*)cvGetSeqElem(sorted_lines, n);
				
		if(!initialLine)
		{
			if(line[1] > theta_hold + MAX_ANGLE_SEP && cnt_trend+1 < MAX_TRENDS)
				cnt_trend++;
		}
		else
			initialLine++;
				
		theta_hold = line[1];
		
		rho_sum[cnt_trend] += line[0];
		theta_sum[cnt_trend] += line[1];
		cnt_avg[cnt_trend]++;
	}
	
	for(int trend=0; trend < cnt_trend+1; trend++) {				
		trend_lines->rho[trend] = rho_sum[trend]/cnt_avg[trend];
		trend_lines->theta[trend] = theta_sum[trend]/cnt_avg[trend];
	}

	trend_lines->total = cnt_trend+1;
	
	cvClearSeq(sorted_lines);
};



void listenCallback(const sensor_msgs::ImageConstPtr& msg) {
  ROS_INFO("Recieved"); 
	struct lineInfo boundaryLines;

	CvMemStorage *sorted_storage = cvCreateMemStorage(0);
		
  IplImage *img_captured, *img_copy;
		
	/////////////////////////////////////// Main Loop /////////////////////////////////////////
  CvSeq *detected_lines;
  CvMemStorage *line_storage = cvCreateMemStorage(0);
  
  img_captured = bridge_.imgMsgToCv(msg); // cvQueryFrame(macBookWebcam);
  img_copy = cvCreateImage(cvGetSize(img_captured), IPL_DEPTH_8U, 3);
  cvCopy(img_captured, img_copy, NULL);
          
  detected_lines = findLinesInImage(img_captured, line_storage);
          
  findTrendLines(detected_lines, &boundaryLines, sorted_storage);
      
  displayLineDetection("trend line", img_copy, &boundaryLines);

  displayLineInfo(&boundaryLines);

  cvClearSeq(detected_lines);
  cvReleaseMemStorage(&line_storage);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe("image", 10, listenCallback);
  ros::spin();
  return 0;
}

