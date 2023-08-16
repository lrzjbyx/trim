#include "NumCpp.hpp"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
//#include "trim.cpp"
#include <numeric>
#include <opencv2/imgproc.hpp>
#include <memory>

/*
 * 返回角度
 * */
double fineTuneAngle(std::shared_ptr<cv::Mat> &image){
    // 灰度处理
    cv::Mat gray,binary,dilation,erosion;
    cv::cvtColor(*image, gray,cv::COLOR_RGB2GRAY);

    // 二值化处理
    auto ret = cv::threshold(gray,binary,60,255,cv::THRESH_BINARY);
    auto height = binary.size().height;
    auto width = binary.size().width;

//    cv::imwrite("/Users/ruizhuti/Documents/code/cpp/clion/trim/binary.png",binary);
    // 膨胀
    auto kernel = nc::ones<nc::uint8>(5, 5);
    auto dilationKernel = cv::Mat(kernel.numRows(), kernel.numCols(), CV_8UC1, kernel.data());
    cv::dilate(binary,dilation,dilationKernel);

    //腐蚀
    auto kernel2 = nc::ones<nc::uint8>(3, 3);
    auto erosionKernel = cv::Mat(kernel2.numRows(), kernel2.numCols(), CV_8UC1, kernel2.data());
    cv::erode(dilation,erosion,erosionKernel);

    //轮廓
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(erosion,contours,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);


    cv::Mat drawing = cv::Mat::zeros( erosion.size(), CV_8UC1 );
    std::vector<cv::RotatedRect> box(contours.size());
    cv::Point2f rect[4];
    std::vector<double> angles;

    for (int i = 0; i < contours.size(); i++) {
        box[i] = cv::minAreaRect(cv::Mat(contours[i]));
        box[i].points(rect);          //最小外接矩形的4个端点

        auto area  = cv::contourArea(contours[i]);

        if (area < int(width*height*0.6)){
            continue;
        }

        for (int j = 0; j < 4; j++)
        {
            // 绘制直线
            cv::line(drawing, rect[j], rect[(j + 1) % 4], cv::Scalar(255, 0, 255), 8, 8);

            float radian =0.0;
            float angle = 0.0;
            if (j==2  or j ==3){
                // 倾斜角度
                radian = atan2((rect[j].y - rect[(j + 1) % 4].y), (rect[j].x - rect[(j + 1) % 4].x));//弧度   该函数返回值范围是[-pi,pi]
            }else{
                radian = atan2((rect[(j + 1) % 4].y - rect[j].y), (rect[(j + 1) % 4].x-rect[j].x));//弧度   该函数返回值范围是[-pi,pi]
            }

            angle = radian * 180 / 3.1415926;//角度

            angles.push_back(angle);
        }

    }

    //方向 默认左倾斜 true
    bool direction = false;
    if ((angles[0]>-45 and angles[2]>-45) and (angles[1]> 45 and angles[3]>45)){
        direction = true;
    }

    // 角度
    for(auto &angle: angles){
        if(angle > 90 ){
            angle = 180 - angle;

        } else if(angle<-90){
            angle = 180+ angle;
        }
        else if(angle>-90 and angle < 0){
            angle = 90 +angle;
        }

        if(angle > 45){
            angle = 90- angle;
        }

    }


    double angle = angles[0];

    if (direction){
        angle = -angle;
    }

    std::cout<<angle<<std::endl;
    return angle;
}

/*
 * 图片旋转
 * */
std::pair<cv::Mat,cv::Mat> rotateImage(std::shared_ptr<cv::Mat>& image,double angle){

    // get rotation matrix for rotating the image around its center in pixel coordinates
    cv::Point2f center((image->cols-1)/2.0, (image->rows-1)/2.0);
    cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
    // determine bounding rectangle, center not relevant
    cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), image->size(), angle).boundingRect2f();
    // adjust transformation matrix
    rot.at<double>(0,2) += bbox.width/2.0 - image->cols/2.0;
    rot.at<double>(1,2) += bbox.height/2.0 - image->rows/2.0;


// 不填充空白区域
    cv::Mat rotated = cv::Mat::zeros( image->size(), CV_8UC1 );
    cv::warpAffine(*image, rotated, rot, bbox.size(),cv::INTER_LINEAR,cv::BORDER_CONSTANT,cv::Scalar(0,0,0));

// 临近填充
    cv::Mat rotatedInter = cv::Mat::zeros( image->size(), CV_8UC1 );
    cv::warpAffine(*image, rotatedInter,rot, bbox.size(), cv::INTER_LINEAR, cv::BORDER_REPLICATE);


    return std::make_pair(rotated,rotatedInter);
}

/*
 * 白色边界裁剪
 * */
std::shared_ptr<cv::Mat> whiteBoundaryCut(std::shared_ptr<cv::Mat>& image){

//    cv::Rect rect(50, 50, image->size().width-100, image->size().height-100);
    cv::Rect rect(25, 25, image->size().width-50, image->size().height-50);
    cv::Mat mat(*image,rect);
    auto result = std::make_shared<cv::Mat>(mat);
    return result;
}

/*
 * 包裹图片
 * */
std::shared_ptr<cv::Mat> wrapImageBoundary(std::shared_ptr<cv::Mat>&image){
    cv::Mat mat;
    cv::copyMakeBorder(*image,mat,50,50,50,50,cv::BORDER_ISOLATED,cv::Scalar(0,0,0));
    return std::make_shared<cv::Mat>(mat);
}

/*
 * 边界裁剪
 * */
std::shared_ptr<cv::Mat> boundaryCut(std::shared_ptr<cv::Mat>&image,std::shared_ptr<cv::Mat>&imageInter){
    // 灰度处理
    cv::Mat gray,binary,dilation,erosion;
    cv::cvtColor(*image, gray,cv::COLOR_RGB2GRAY);

    // 二值化处理
    auto ret = cv::threshold(gray,binary,60,255,cv::THRESH_BINARY);
    auto height = binary.size().height;
    auto width = binary.size().width;



    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(binary,contours,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);


    std::vector<cv::Rect> rects;

    for (size_t i = 0; i < contours.size();++i) {
        // 最大外接轮廓
        cv::Rect rect = cv::boundingRect(contours[i]);

        auto area  = cv::contourArea(contours[i]);
        if (area < int(width*height*0.4)){
            continue;
        }

        std::cout<<rect.x<<"\t"<<rect.y<<"\t"<<rect.width<<"\t"<<rect.height<<"\t"<<std::endl;
        cv::rectangle(binary, rect, cv::Scalar(255, 255, 0), 2, cv::LINE_8);
        rects.push_back(rect);
    }

    cv::imwrite("/Users/ruizhuti/Documents/code/cpp/clion/trim/binary.png",binary);

    std::shared_ptr<cv::Mat> result ;
    if(rects.size() == 1){
        cv::Rect roi(rects[0].x+5, rects[0].y+5, rects[0].width-10, rects[0].height-10);
        cv::Mat mat(*imageInter,roi);
        result = std::make_shared<cv::Mat>(mat);
//        cv::imwrite("/Users/ruizhuti/Documents/code/cpp/clion/trim/binary22.png",*result);
        return result;
    }else{
        return std::make_shared<cv::Mat>();
    }
}

/*
 * 裁边
 * */
std::shared_ptr<cv::Mat> borderClipping(std::shared_ptr<cv::Mat>&src){
    // 白色边界裁截
    auto image = whiteBoundaryCut(src);
    // 包裹图片
    auto wrapImage = wrapImageBoundary(image);
    // 倾斜角度
    auto angle = fineTuneAngle(wrapImage);
    // 旋转后的图片
    auto rotateResult =  rotateImage(image,angle);

    auto rotated = std::make_shared<cv::Mat>(rotateResult.first);
    auto rotatedInter = std::make_shared<cv::Mat>(rotateResult.second);
    // 自动裁边后的图片
    auto dstImage = boundaryCut(rotated,rotatedInter);

    if(dstImage){
        return dstImage;
    }
    return rotatedInter;
}

int main()
{

    cv::Mat mat;
    mat = cv::imread("/Users/ruizhuti/Documents/code/cpp/clion/trim/333.jpg");
//    mat = cv::imread("/Users/ruizhuti/Documents/code/cpp/clion/trim/bsq.png");
//    mat = cv::imread("/Users/ruizhuti/Documents/code/cpp/clion/trim/333.jpg");
//    mat = cv::imread("/Users/ruizhuti/Documents/code/cpp/clion/trim/123.jpg");

    auto image = std::make_shared<cv::Mat>(mat);
    auto result = borderClipping(image);

    cv::imwrite("/Users/ruizhuti/Documents/code/cpp/clion/trim/result.png",*result);

    return 0;
}