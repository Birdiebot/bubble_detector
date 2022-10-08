/*
 * @Author: Ligcox
 * @Date: 2022-02-19 10:07:56
 * @FilePath: /bubble_bringup/home/nvidia/Desktop/bubble/src/bubble_detector/bubble_rune/include/detector.hpp
 * @LastEditors: HarryWen
 * @LastEditTime: 2022-08-03 12:51:47
 * License: GNU General Public License v3.0. See LICENSE file in root directory.
 * Copyright (c) 2022 Birdiebot R&D Department
 * Shanghai University Of Engineering Science. All Rights Reserved
 */
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <algorithm>
#include <limits>

typedef struct
{
    cv::Point2f hammerStrip[4] = {};
    cv::Point2f armourStrip[4] = {};
    cv::Point2f circleStrip[4] = {};
} RUNESTRIP;

inline float calPointDistance(cv::Point point1, cv::Point point2)
{
    float distance = sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
    return distance;
}

inline double calLineDegree(cv::Point2f point1, cv::Point2f point2)
{

    float angle = 0;
    int height = point1.y - point2.y;
    int weight = point1.x - point2.x;
    if (height == 0)
        angle = 0;
    else if (weight == 0)
        angle = 90;
    else
        angle = atan(height / weight) * 180 / M_PI;
    return angle;
}

inline std::vector<float> calRectWidthHeight(std::vector<cv::Point2f> rect)
{
    std::vector<float> widthHeightVector;
    double degree1 = calLineDegree(rect[0], rect[3]);
    double degree2 = calLineDegree(rect[0], rect[1]);
    float width = calPointDistance(rect[0], rect[3]);
    float height = calPointDistance(rect[0], rect[1]);
    if (abs(degree1) > abs(degree2))
    {
        float temp = width;
        width = height;
        height = temp;
    }
    widthHeightVector.push_back(width);
    widthHeightVector.push_back(height);
    return widthHeightVector;
}

inline cv::Mat enhanceContour(cv::Mat &image, int morphologyexIter)
{
    int niters = 1;
    cv::Mat temp;
    cv::Mat kernel = cv::getStructuringElement(2, cv::Size(3, 3), cv::Point(-1, -1)); // 0为MORPH_RECT
    cv::erode(image, temp, kernel, cv::Point(-1, -1), niters);
    cv::morphologyEx(image, temp, 3, kernel, cv::Point(-1, -1), morphologyexIter); // 闭操作
    cv::dilate(temp, temp, kernel, cv::Point(-1, -1), 3);
    // cv::GaussianBlur(temp, temp, cv::Size(3, 3), 3, 3);                //效果如果不好可删除
    // cv::erode(image, temp, kernel, cv::Point(-1, -1), niters);
    // cv::morphologyEx(image, temp, 3, kernel, cv::Point(-1, -1), niters); // 闭操作
    // cv::Canny(temp, edge, edgeLowThresh, edgeHighThresh, 3);
    return temp;
}

std::vector<std::vector<cv::Point>> filterArea(std::vector<std::vector<cv::Point>> contours, int minArea, int maxArea, bool temp)
{
    std::vector<std::vector<cv::Point>> matchContoursList;
    std::vector<int> hierarchyIdList;

    for (int contours_id = 0; contours_id < contours.size(); contours_id += 1)
    {

        const std::vector<cv::Point> &contour = contours[contours_id];
        double area = cv::contourArea(cv::Mat(contour));

        if (minArea < area && area < maxArea)
        {
            if (temp )
            {   
                if (contours_id < contours.size())
                    contours_id += 1;
                double area = cv::contourArea(cv::Mat(contours[contours_id]));
                if (area > 150 && area < 400)
                {
                    matchContoursList.push_back(contours[contours_id]);
                    hierarchyIdList.push_back(contours_id);
                    // std::cout << minArea << " " << area << " " << maxArea << " " << hierarchyIdList.size() << std::endl;
                }
            }
            else
            {
                matchContoursList.push_back(contours[contours_id]);
                hierarchyIdList.push_back(contours_id);
                // std::cout << minArea << " " << area << " " << maxArea << " " << hierarchyIdList.size() << std::endl;

            }
        }
        // std::cout << minArea << " " << area << " " << maxArea << " " << hierarchyIdList.size() << std::endl; // 查看面积

    }

    return matchContoursList;
}

void findRect(std::vector<std::vector<cv::Point>> contours, std::string mode, std::string shape, cv::Point2f target_roi_box[4])
{
    double minArea, maxArea;
    cv::RotatedRect target_rect;
    cv::RotatedRect area_rect;
    std::vector<cv::Point> minAreaContour;
    std::vector<cv::Point> maxAreaContour;
    float lower_ratio = 1;
    float upper_ratio = 3;
    if (shape == "square")
    {
        lower_ratio = 1; 
        upper_ratio = 1.5;
    }

    else if (shape == "rect")
    {
        lower_ratio = 1;
        upper_ratio = 3;
    }

    for (int i = 0; i < 4; i++)
    {
        cv::Point pt;
        pt.x = 0;
        pt.y = 0;
        minAreaContour.push_back(pt);
        maxAreaContour.push_back(pt);
    }

    for (int contours_id = 0; contours_id < contours.size(); contours_id += 1)
    {

        area_rect = cv::minAreaRect(contours[contours_id]);
        float width = area_rect.size.width;
        float height = area_rect.size.height;

        float min_value = std::min(width,height);

        if (min_value == 0)
            min_value += 1;
        float aspect_ratio = std::max(width, height) / min_value;
        // std::cout << shape << " " << aspect_ratio << " " << width << " " << height << std::endl; //查看长宽比
        if (lower_ratio <= aspect_ratio && aspect_ratio <= upper_ratio)
        {
            auto contour = contours[contours_id];
            if (contours_id == 0)
            {
                {
                    minAreaContour = contour;
                    maxAreaContour = contour;
                    minArea = contourArea(minAreaContour);
                    maxArea = contourArea(maxAreaContour);
                }
            }
            else
            {
                if (contourArea(contour) < minArea)
                {
                    minArea = contourArea(contour);
                    // std::cout << "min area" << minArea << std::endl;
                    minAreaContour = contour;
                }
                else if (contourArea(contour) > maxArea)
                {
                    maxArea = contourArea(contour);
                    maxAreaContour = contour;
                }
            }
        }
    }

    if (mode == "min")
    {
        target_rect = cv::minAreaRect(minAreaContour);
        target_rect.points(target_roi_box);
    }
    else if (mode == "max")
    {
        target_rect = cv::minAreaRect(maxAreaContour);
        target_rect.points(target_roi_box);
    }
}

template <typename T>
std::vector<RUNESTRIP> findStrip(std::vector<std::vector<cv::Point>> contours, T node_)
{
    std::vector<RUNESTRIP> strip;
    RUNESTRIP a;
    std::string mode = "min";
    std::string shape = "rect";
    std::vector<cv::Point> minAreaContour;

    std::vector<std::vector<cv::Point>> hammerStrips = filterArea(contours, node_->lowerHammerStrip, node_->upperHammerStrip, true);
    findRect(hammerStrips, mode, "rect", a.hammerStrip);

    std::vector<std::vector<cv::Point>> circleStrip = filterArea(contours, node_->lowerCircleStrip, node_->upperCircleStrip, false);
    findRect(circleStrip, mode, "square", a.circleStrip);

    // std::vector<std::vector<cv::Point>> armourStrip = filterArea(contours, node_->lowerArmourStrip, node_->upperArmourStrip,temp = false);
    // findRect(armourStrip, mode, a.armourStrip);

    strip.push_back(a);
    return strip;
}
