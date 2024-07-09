#include <opencv2\opencv.hpp>
#include <opencv2\imgproc.hpp>
#include <opencv2\highgui.hpp>
#include <stdio.h>
#include <string.h>
#include <baseapi.h>
#include <allheaders.h>

using std::vector;

bool compareContourAreas(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2);
cv::Mat processingImage(cv::Mat image, double threshold1, double threshold2);
cv::Mat closedContours(cv::Mat image, const int ksize);
vector<vector<cv::Point>> approxImage(cv::Mat image);
vector<vector<cv::Point>> approxCroppedImage(cv::Mat image);
cv::Mat ROI(cv::Mat image, vector<vector<cv::Point>> approxContours, int biasX = 0,  int biasY = 0, int biasWidth = 0, int biasHeight = 0);
cv::Mat ROI(cv::Mat image, int biasX, int biasY, int biasWidth, int biasHeight);
std::string checkIsDigit(char* text);
void renameImage(cv::Mat image, std::string num);

int main() {
    cv::Mat canny, closed;
    cv::Mat croppedImagecanny;
    cv::Mat resultImage1, resultImage2;

    vector<vector<cv::Point>> approxContours1;
    vector<vector<cv::Point>> approxContours2;

    cv::Mat image = cv::imread("img_test.jpg");
    cv::resize(image, image, cv::Size(640, 480));
    
    canny = processingImage(image, 100, 250);
    closed = closedContours(canny, 2);
 
    approxContours1 = approxImage(closed);
    std::sort(approxContours1.begin(), approxContours1.end(), compareContourAreas);
    
    resultImage1 = ROI(image, approxContours1, -8, -8, 12, 12);
    cv::imshow("roi", resultImage1);

    cv::resize(resultImage1, resultImage1, cv::Size(200, 200));
    croppedImagecanny = processingImage(resultImage1, 100, 250);

    approxContours2 = approxCroppedImage(croppedImagecanny);
    std::sort(approxContours2.begin(), approxContours2.end(), compareContourAreas);

    resultImage2 = ROI(resultImage1, approxContours2);
    cv::imshow("res", resultImage2);
    
    cv::resize(resultImage2, resultImage2, cv::Size(600, 150));
    cv::cvtColor(resultImage2, resultImage2, cv::COLOR_BGR2GRAY);
    cv::threshold(resultImage2, resultImage2, 150, 200, cv::THRESH_BINARY);

    cv::Mat numberImage = ROI(resultImage2, 27, 7, -80, -30);
  
    tesseract::TessBaseAPI ocr;
    ocr.Init("./tessdata", "eng");  // Set the language to English
    Pix* pix = pixRead("new_num.jpg");
    ocr.SetImage(pix);
    char* result = ocr.GetUTF8Text();

    std::string num = checkIsDigit(result);
    renameImage(image, num);

    ocr.End();
    pixDestroy(&pix);

    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}

bool compareContourAreas(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2) {
    double i = fabs(contourArea(cv::Mat(contour1)));
    double j = fabs(contourArea(cv::Mat(contour2)));
    return (i > j);
}

cv::Mat processingImage(cv::Mat image, double threshold1, double threshold2) {
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    cv::Canny(image, image, threshold1, threshold2);
    return image;
}

cv::Mat closedContours(cv::Mat image, const int ksize) {
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(ksize, ksize));
    cv::morphologyEx(image, image, cv::MORPH_CLOSE, kernel);
    return image;
}

vector<vector<cv::Point>> approxImage(cv::Mat image) {
    vector<vector<cv::Point>> contours;
    vector<cv::Point> approx;
    vector<vector<cv::Point>> approxContours;

    cv::findContours(image, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    for (auto& contr : contours) {
        double eps = 0.02 * cv::arcLength(contr, true);
        cv::approxPolyDP(contr, approx, eps, true);

        if (approx.size() == 4) {
            approxContours.push_back(approx);
        }
    }
    return approxContours;
}

vector<vector<cv::Point>> approxCroppedImage(cv::Mat image) {
    vector<vector<cv::Point>> contoursCroppedImage;
    vector<vector<cv::Point>> approxContoursCroppedImage;
    vector<cv::Point> approxCroppedImage;

    cv::findContours(image, contoursCroppedImage, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    for (auto& contr : contoursCroppedImage) {
        double eps = 0.02 * cv::arcLength(contr, true);
        cv::approxPolyDP(contr, approxCroppedImage, eps, true);

        if (approxCroppedImage.size() == 4) {
            double a = fabs(cv::norm(approxCroppedImage[1] - approxCroppedImage[0]));
            double b = fabs(cv::norm(approxCroppedImage[2] - approxCroppedImage[1]));

            if (std::abs(a - b) > 55 && std::abs(a - b) < 65)
                approxContoursCroppedImage.push_back(approxCroppedImage);
        }
    }
    return approxContoursCroppedImage;
}

cv::Mat ROI(cv::Mat image, vector<vector<cv::Point>> approxContours, int biasX, int biasY ,  int biasWidth , int biasHeight) {
    cv::Rect rect = cv::boundingRect(approxContours[0]);

    cv::Mat roiRect(image, cv::Rect(rect.x + biasX, rect.y + biasY, rect.width + biasWidth, rect.height + biasHeight));
    cv::Mat croppedImage;
    roiRect.copyTo(croppedImage);

    return croppedImage;
}

cv::Mat ROI(cv::Mat image, int biasX, int biasY, int biasWidth, int biasHeight) {
    cv::Rect rect = cv::boundingRect(image);
    cv::rectangle(image, rect, cv::Scalar(255, 0, 0), 3);

    cv::Mat roiRect(image, cv::Rect(rect.x + biasX, rect.y + biasY, rect.width + biasWidth, rect.height + biasHeight));
    cv::Mat croppedImage;
    roiRect.copyTo(croppedImage);

    return croppedImage;
}

std::string checkIsDigit(char* text) {
    std::string numbers;
    for (char* c = text; *c; c++) {
        if (isdigit(*c)) {
            numbers += *c;
        }
    }
    return numbers;
}

void renameImage(cv::Mat image, std::string num) {
    if (num.length() == 12 && std::all_of(num.begin(), num.end(), ::isdigit)) {
        std::cout << "Number: " << num << std::endl;
        cv::imwrite(num + ".jpg", image);
    }
    else {
        std::cout << "Error number" << std::endl;
    }
}