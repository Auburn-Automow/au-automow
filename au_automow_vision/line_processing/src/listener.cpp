#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


#define MAX_TRENDS 3
#define MAX_ANGLE_SEP 0.1745 // 0.1745 rad = 10 degrees

#define SORT_BY_RHO 0
#define SORT_BY_THETA 1

#define SUBTRACT_GREEN_RED(img) (cvSubS(img, cvScalar(0, 255, 255, 0), img, NULL))
#define THRESHOLD_IMAGE(img1, img2) (cvThreshold(img1, img2, maxPixVal-1, 255, CV_THRESH_BINARY))


struct window_density {
	uchar *index;
	int density;
};

int findMaxRho(IplImage *img) {
	return sqrt(pow(img->height,2) + pow(img->width,2));
};

int cmpSort(const void *_a, const void *_b, void *userdata)
{
	float *a = (float*)_a;
	float *b = (float*)_b;
	int *sort_type = (int*)userdata;

	if(*sort_type == SORT_BY_THETA)
		return a[1] > b[1] ? 1:a[1] < b[1] ? -1:0;
	else
		return a[0] > b[0] ? 1:a[0] < b[0] ? -1:0;
};

void test(int num) {printf("Test Point: %d\n", num);};

void displayCvSeq(CvSeq *seq) {
	printf("total lines: %d\n", seq->total);
	
	for(int n=0; n < seq->total; n++) {
		float *line = (float*)cvGetSeqElem(seq, n);
		
		printf("rho: %.3f\ttheta: %.3f\n", line[0], line[1]*180/CV_PI);
	}
	
	printf("\n");
};

void drawLines(IplImage *img, CvSeq *lines_to_draw)
{
	for(int line_num = 0; line_num < lines_to_draw->total; line_num++)
	{
		float *line = (float*)cvGetSeqElem(lines_to_draw, line_num);
		
		CvPoint pt1, pt2;
		double a = cos(line[1]), b = sin(line[1]);
		double x0 = a*line[0], y0 = b*line[0];
		pt1.x = cvRound(x0 + 1500*(-b));
		pt1.y = cvRound(y0 + 1500*(a));
		pt2.x = cvRound(x0 - 1500*(-b));
		pt2.y = cvRound(y0 - 1500*(a));
		
		cvLine(img, pt1,pt2, CV_RGB(0,0,255), 3, CV_AA, 0);
	}
}

void displayLineDetection(char *window_name, IplImage *img, CvSeq *trend_lines) {
	drawLines(img, trend_lines);
	
	// display image
	cvShowImage(window_name, img);
};

void correctNegativeRho(CvSeq *lines) {
	float line[2];

	for(int n=0; n < lines->total; n++) {
		cvSeqPopFront(lines, &line);

		if(line[0] < 0)
		{
			line[0] = -line[0];
			line[1] = line[1] - CV_PI;
		}

		cvSeqPush(lines, line);
	}
};

void sortLines(CvSeq *unsorted, int sort_type) {
	if(sort_type != SORT_BY_RHO && sort_type != SORT_BY_THETA)
		printf("sort type error!\n");

	cvSeqSort(unsorted, cmpSort, &sort_type);
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

int binaryPixelDensityFinder(IplImage *img, int window_height, int window_width, struct window_density window_densities[]) {
	int max_density = 0;
	int density = 0;
	int index = 0;
	
	for(int n=0; n < img->height; n+=window_height) { // shift down rows to next row-window
		uchar *ptr1 = (uchar*)(img->imageData + n*img->widthStep);
		
		for(int m=0; m < img->width; m+=window_width) { // shift along columns to next window
			uchar *ptr2 = ptr1 + m;

			for(int k=0; k < window_height; k++) { // shift down by single rows within window
				uchar *ptr3 = ptr2 + k*img->widthStep;

				for(int w=0; w < window_width; w++) // sum along columns in row
						density+= *(ptr3+w);
			}
			
			if(density > max_density)
				max_density = density;
			
			window_densities[index].index = ptr2;
			window_densities[index++].density = density;

			density = 0;
		}
	}

	return max_density;
};

void DensityFilter(struct window_density densities[], int max_density, int window_height, int window_width, int widthStep) {
	for(int n=0; n < 768; n++) {
		if(densities[n].density < max_density*0.75) {
			for(int k=0; k < window_height; k++) {
				uchar *ptr = densities[n].index + k*widthStep;

				for(int m=0; m < window_width; m++)
					*(ptr+m) = 0;
			}
		}
	}
};

void averageToTrends(CvSeq *group, CvSeq *trend_lines) {
	float rho_sum = 0;
	float theta_sum = 0;
	float bundle[2];

	for(int line_num=0; line_num < group->total; line_num++) {
		float *line = (float*)cvGetSeqElem(group, line_num);
		rho_sum += line[0];
		theta_sum += line[1];
	}

	bundle[0] = rho_sum/group->total;
	bundle[1] = theta_sum/group->total;
	cvSeqPush(trend_lines, bundle);
};

void discoverTrendLines(CvSeq *lines_to_be_averaged, CvSeq *trend_lines) {
	CvMemStorage *storage_theta_group = cvCreateMemStorage(0);
	CvSeq *theta_group = cvCreateSeq(0, sizeof(CvSeq), 2*sizeof(float), storage_theta_group);

	int initial_line = 1;
	float theta_hold;

	sortLines(lines_to_be_averaged, SORT_BY_THETA);

	for(int line_num=0; line_num < lines_to_be_averaged->total; line_num++) {
		float *line = (float*)cvGetSeqElem(lines_to_be_averaged, line_num);
		float theta = line[1];

		if(initial_line) {
			theta_hold = theta;
			cvSeqPush(theta_group, line);
			initial_line = 0;
		}
		else {
			if(theta > theta_hold + MAX_ANGLE_SEP || line_num+1 == lines_to_be_averaged->total) {
				averageToTrends(theta_group, trend_lines);
				cvClearSeq(theta_group);
				initial_line = 1;
			}
			else {
				theta_hold = theta;
				cvSeqPush(theta_group, line);
			}
		}
	}

	cvReleaseMemStorage(&storage_theta_group);
};

CvSeq* findLinesInImage(IplImage *img, CvMemStorage *line_storage) {
	int maxPixVal;
	
	// data for hough
	float rho = 1;
	float theta = CV_PI/180;
	int threshold = img->height/2;
	
	// image pointers
	IplImage *img_1chan = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
	IplImage *img_thresh = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
	
	CvSeq *lines;

	struct window_density densities[768];
	int max_density;
		
	/////////////////////////////////////// Begin Line Detection /////////////////////////////////////////
	SUBTRACT_GREEN_RED(img);
	
	cvCvtColor(img, img_1chan, CV_RGB2GRAY);
	
	maxPixVal = maxPixelValue(img_1chan);
		
	THRESHOLD_IMAGE(img_1chan, img_thresh);

	cvNamedWindow("threshold", 1);
	cvShowImage("threshold", img_thresh);

	max_density = binaryPixelDensityFinder(img_thresh, 40, 40, densities);
	DensityFilter(densities, max_density, 40, 40, img_thresh->widthStep);
	
	cvNamedWindow("filter", 1);
	cvShowImage("filter", img_thresh);
	
	/*printf("densities and indices:\n");
	for(int n=0; n < 768; n++) {
		printf("index: %p\tdensity: %d\n", densities[n].index, densities[n].density);
	}
		
	printf("max_density: %d\n", max_density);*/
				
	lines = cvHoughLines2(img_thresh, line_storage, CV_HOUGH_STANDARD, rho, theta, threshold, 0, 0);
	
	return lines;
};

void findTrendLines(CvSeq *found_lines, CvSeq *trend_lines, int max_rho) {
	CvMemStorage *storage_rho_group = cvCreateMemStorage(0);
	
	CvSeq *rho_group = cvCreateSeq(0, sizeof(CvSeq), 2*sizeof(float), storage_rho_group);
	
	// trend variables
	float rho_hold;
	int initial_line = 1;
			
	/////////////////////////////////////// Begin Trend Line Discovery /////////////////////////////////////////
	correctNegativeRho(found_lines);
	sortLines(found_lines, SORT_BY_RHO);
					
	for(int line_num=0; line_num < found_lines->total; line_num++)
	{
		float *line = (float*)cvGetSeqElem(found_lines, line_num);
		float rho = line[0];
				
		if(initial_line) {
			rho_hold = rho;
			cvSeqPush(rho_group, line);
			initial_line = 0;
		}
		else {
			float diff = rho - rho_hold;

			if(diff < max_rho/2.0)
				cvSeqPush(rho_group, line);
			
			if(diff >= max_rho/2.0 || line_num+1 == found_lines->total) {
				discoverTrendLines(rho_group, trend_lines);

				cvClearSeq(rho_group);
				initial_line = 1;
			}
		}	
	}

	cvReleaseMemStorage(&storage_rho_group);
};


int main(int argc, char *argv[]) {
	CvMemStorage *line_storage = cvCreateMemStorage(0);
	CvMemStorage *trends_storage = cvCreateMemStorage(0);

	CvSeq *detected_lines;
	CvSeq *trend_lines = cvCreateSeq(0, sizeof(CvSeq), 2*sizeof(float), trends_storage);

	CvCapture *macBookWebcam = cvCreateCameraCapture(0);
	//CvCapture *externalWebcam = cvCreateCameraCapture(1);
	
	//IplImage *img_captured = cvQueryFrame(macBookWebcam);
	IplImage *img_captured = cvLoadImage(argv[1], CV_LOAD_IMAGE_UNCHANGED);
	IplImage *img_copy = cvCreateImage(cvGetSize(img_captured), IPL_DEPTH_8U, 3);

	int max_rho = findMaxRho(img_captured);
	/////////////////////////////////////// Main Sequence /////////////////////////////////////////
	//img_captured = cvQueryFrame(macBookWebcam);
		
	cvCopy(img_captured, img_copy, NULL);
						
	detected_lines = findLinesInImage(img_captured, line_storage);
					
	findTrendLines(detected_lines, trend_lines, max_rho);
			
	displayLineDetection("trend line", img_copy, trend_lines);
	
	cvReleaseMemStorage(&line_storage);

	cvWaitKey(0);
	/////////////////////////////////////// End Main Sequence /////////////////////////////////////////

	return 0;
}

void listenCallback(const std_msgs::StringConstPtr& msg) {
  ROS_INFO("Recieved"); 
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("image", 10, listenCallback);
  ros::spin();
  return 0;
}

