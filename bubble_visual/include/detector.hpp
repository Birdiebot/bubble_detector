/*
 * @Author: Ligcox
 * @Date: 2022-02-19 10:07:56
 * @FilePath: /bubble_bringup/home/nvidia/Desktop/bubble/src/bubble_contrib/bubble_visual/include/detector.hpp
 * @LastEditors: HarryWen
 * @LastEditTime: 2022-06-19 23:42:44
 * License: GNU General Public License v3.0. See LICENSE file in root directory.
 * Copyright (c) 2022 Birdiebot R&D Department
 * Shanghai University Of Engineering Science. All Rights Reserved
 */
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <algorithm>
#include <limits>

typedef struct strip
{
    cv::RotatedRect ROI_RECT;
    cv::Point2f ROI_BOX[4];
} STRIP;

typedef struct
{
    cv::RotatedRect ROI_RECT;
    cv::Point2f ROI_BOX[4];
    cv::Point2f LIGHTSTRIP1_BOX[4];
    cv::Point2f LIGHTSTRIP2_BOX[4];
} ARMOUR;

typedef struct
{
    cv::Point2f hammerStrip[4];
    cv::Point2f armourStrip[4];
    cv::Point2f circleStrip[4];
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

inline bool getCv2IsVerticalRect(std::vector<cv::Point2f> rect)
{

    std::vector<float> widthHeight = calRectWidthHeight(rect);
    float w = widthHeight[0];
    float h = widthHeight[1];
    if (w > 1000)
        return false;
    if (1.0 <= h / w && h / w <= 100)
        return true;
    return false;
}

inline float getCv2RotatedRectOrientation(cv::RotatedRect rect)
{
    float val = rect.angle;
    float w = rect.size.width;
    float h = rect.size.height;
    if (w > h)
        val += 90;
    while (val > 90)
        val -= 180;
    while (val <= -90)
        val += 180;
    return val;
}

inline double getCv2RotatedRectAngleDifference(cv::RotatedRect rect1, cv::RotatedRect rect2)
{
    double val = getCv2RotatedRectOrientation(rect1) - getCv2RotatedRectOrientation(rect2);
    return std::abs(val);
}

inline double getCv2RotatedRectDistanceRatio(cv::RotatedRect rect1, cv::RotatedRect rect2)
{
    double val;
    try
    {
        double distance = std::sqrt((rect1.center.x - rect2.center.x) * (rect1.center.x - rect2.center.x) + (rect1.center.y - rect2.center.y) * (rect1.center.y - rect2.center.y));
        double rect1_length = std::max(rect1.size.width, rect1.size.height);
        double rect2_length = std::max(rect2.size.width, rect2.size.height);
        val = distance / std::max(rect1_length, rect2_length);
    }
    catch (const std::exception &e)
    {
        std::cerr << "error" << e.what() << '\n';
        val = std::numeric_limits<double>::max();
    }
    return val;
}

inline double getCv2RotatedRectAreaDifferenceRatio(cv::RotatedRect rect1, cv::RotatedRect rect2)
{
    float rect1_size = rect1.size.width * rect1.size.height;
    float rect2_size = rect2.size.width * rect2.size.height;
    double bigger_size = std::max(rect1_size, rect2_size);
    double smaller_size = std::min(rect1_size, rect2_size);
    float size_ratio = bigger_size/smaller_size;
    return size_ratio;
    
}


template <typename T>
std::vector<STRIP> detectLightStrip(cv::Mat &image, T node_)
{
    std::vector<STRIP> strip_list;
    cv::Mat gray_img;
    try
    {
        if (image.channels() == 3)
            cv::cvtColor(image, gray_img, cv::COLOR_BGR2GRAY);
        else if (image.channels() == 1)
            gray_img = image;
        else
        {
            std::cout << "Image Shape " << image.channels() << "Unexpected" << std::endl;
            return strip_list;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "error" << e.what() << '\n';
        return strip_list;
    }
    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(gray_img, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    double contour_area_threshold = node_->contourAreaThreshold;

    for (unsigned int i = 0; i < contours.size(); i++)
    {
        std::vector<cv::Point> c = contours[i];
        if ((int)c.size() > contour_area_threshold)
        {
            std::vector<cv::Point2f> pointVector;
            STRIP strip_data;
            strip_data.ROI_RECT = cv::minAreaRect(c);
            strip_data.ROI_RECT.points(strip_data.ROI_BOX);
            for (auto item : strip_data.ROI_BOX)
            {
                pointVector.push_back(item);
            }
            if (!getCv2IsVerticalRect(pointVector))
                continue;
            strip_list.push_back(strip_data);
        }
    }
    return strip_list;
}

template <typename T>
std::vector<ARMOUR> detectArmour(std::vector<STRIP> lightstrip_list, T node_)
{
    std::vector<ARMOUR> armour_list;
    std::vector<ARMOUR> final_armour_list;
    for (unsigned int i = 0; i < lightstrip_list.size();
         i++)
    {
        std::vector<float> lightstripRelation;
        float min_angle_difference = std::numeric_limits<float>::max();
        for (unsigned int j = i + 1; j < lightstrip_list.size(); j++)
        {
            float angle_difference = getCv2RotatedRectAngleDifference(lightstrip_list[i].ROI_RECT, lightstrip_list[j].ROI_RECT);
            float distance_ratio = getCv2RotatedRectDistanceRatio(lightstrip_list[i].ROI_RECT, lightstrip_list[j].ROI_RECT);
            float size_difference = getCv2RotatedRectAreaDifferenceRatio(lightstrip_list[i].ROI_RECT, lightstrip_list[j].ROI_RECT);
            if (angle_difference <= node_->lightStripAngleDiff &&
                node_->lightStripDisplacementDiffMin <= distance_ratio &&
                distance_ratio <= node_->lightStripDisplacementDiffMax &&
                distance_ratio <= min_angle_difference && 
                size_difference <= node_->lightStripSizeDiffRatio)
            {
                min_angle_difference = distance_ratio;
                lightstripRelation.push_back(i);
                lightstripRelation.push_back(j);
                lightstripRelation.push_back(angle_difference);
                lightstripRelation.push_back(distance_ratio);
            }
        }
        // get min distance_ratio of armour
        if (lightstripRelation.size() == 4)
        {
            ARMOUR armour_data;
            lightstrip_list[int(lightstripRelation[0])].ROI_RECT.points(armour_data.LIGHTSTRIP1_BOX);
            lightstrip_list[int(lightstripRelation[1])].ROI_RECT.points(armour_data.LIGHTSTRIP2_BOX);
            std::vector<cv::Point2f> hconcatMat;
            for (int k = 0; k < 4; k++)
            {
                hconcatMat.push_back(armour_data.LIGHTSTRIP1_BOX[k]);
                hconcatMat.push_back(armour_data.LIGHTSTRIP2_BOX[k]);
            }
            armour_data.ROI_RECT = cv::minAreaRect(hconcatMat);
            armour_data.ROI_RECT.points(armour_data.ROI_BOX);
            armour_list.push_back(armour_data);
        }
    }

    for (unsigned int i = 0; i < armour_list.size(); i++)
    {      
        ARMOUR temp_armour_data;
        temp_armour_data = armour_list[i];
        // cv::Point2f lightstrip1_box[4],lightstrip2_box[4];

        for (unsigned int j = i + 1; j < armour_list.size(); j++)
        {   
            if (temp_armour_data.LIGHTSTRIP1_BOX[0].x == armour_list[j].LIGHTSTRIP1_BOX[0].x ||
             temp_armour_data.LIGHTSTRIP1_BOX[0].x == armour_list[j].LIGHTSTRIP2_BOX[0].x ||
              temp_armour_data.LIGHTSTRIP2_BOX[0].x == armour_list[j].LIGHTSTRIP1_BOX[0].x ||
               temp_armour_data.LIGHTSTRIP2_BOX[0].x == armour_list[j].LIGHTSTRIP2_BOX[0].x)
            {   
                std::vector<cv::Point2f> temp1 ,temp2,temp3,temp4;
                for (int k = 0 ;k<4;k++)
                {
                    temp1.push_back(armour_list[j].LIGHTSTRIP1_BOX[k]);
                    temp2.push_back(armour_list[j].LIGHTSTRIP2_BOX[k]);
                    temp3.push_back(temp_armour_data.LIGHTSTRIP1_BOX[k]);
                    temp4.push_back(temp_armour_data.LIGHTSTRIP2_BOX[k]);
                }
                cv::RotatedRect rect_1 = cv::minAreaRect(temp1);
                cv::RotatedRect rect_2 = cv::minAreaRect(temp2);
                cv::RotatedRect rect_3 = cv::minAreaRect(temp3);
                cv::RotatedRect rect_4 = cv::minAreaRect(temp4);
                
                float angle_difference_1 = getCv2RotatedRectAngleDifference(rect_1, rect_2);
                float angle_difference_2 = getCv2RotatedRectAngleDifference(rect_3, rect_4);
                if (angle_difference_1 <= angle_difference_2)
                {
                    temp_armour_data = armour_list[j];
                    armour_list[i] = temp_armour_data;
                }
                else
                {
                    temp_armour_data = temp_armour_data;
                     armour_list[j] = temp_armour_data;
                }
            }

        }
        final_armour_list.push_back(temp_armour_data);
    }

    return final_armour_list;
}


inline cv::Mat enhanceContour(cv::Mat &image, int morphologyexIter)
{
    int niters = 1;
    cv::Mat temp;
    cv::Mat kernel = cv::getStructuringElement(2, cv::Size(3, 3), cv::Point(-1, -1)); // 0为MORPH_RECT
    cv::erode(image, temp, kernel, cv::Point(-1, -1), niters);
    cv::morphologyEx(image, temp, 3, kernel, cv::Point(-1, -1), morphologyexIter); // 闭操作
    cv::dilate(temp,temp,kernel,cv::Point(-1,-1),3);
    // cv::GaussianBlur(temp, temp, cv::Size(3, 3), 3, 3);                //效果如果不好可删除
    // cv::erode(image, temp, kernel, cv::Point(-1, -1), niters);
    // cv::morphologyEx(image, temp, 3, kernel, cv::Point(-1, -1), niters); // 闭操作
    // cv::Canny(temp, edge, edgeLowThresh, edgeHighThresh, 3);
    return temp;
}



std::vector<std::vector<cv::Point>> filterArea(std::vector<std::vector<cv::Point>> contours, int minArea, int maxArea,bool temp)
{
    std::vector<std::vector<cv::Point>> matchContoursList;
    std::vector<int> hierarchyIdList;
    
    for (int contours_id = 0; contours_id < contours.size()-1; contours_id+=1)
    {   

        const std::vector<cv::Point> &contour = contours[contours_id];

        double area = cv::contourArea(cv::Mat(contour));

        if (minArea < area && area < maxArea)
        {   
            if (temp)
            {
                contours_id+=1;
                double area = cv::contourArea(cv::Mat(contours[contours_id]));
                if (area >500)
                {
                    matchContoursList.push_back(contours[contours_id]);
                    hierarchyIdList.push_back(contours_id);
                }
            }
            matchContoursList.push_back(contours[contours_id]);
            hierarchyIdList.push_back(contours_id);
        }
    }
    
    return matchContoursList;
}

void findRect(std::vector<std::vector<cv::Point>> contours, std::string mode,cv::Point2f target_roi_box[4] )
{
    cv::RotatedRect target_rect;
    std::vector<cv::Point> minAreaContour;
    std::vector<cv::Point> maxAreaContour;

    double minArea;
    double maxArea;
    for (int contours_id = 0; contours_id < contours.size(); contours_id += 1)
    {
        if (contours_id == 0)
        {
            minAreaContour = contours[contours_id];
            maxAreaContour = contours[contours_id];
            minArea = contourArea(minAreaContour);
            maxArea = contourArea(maxAreaContour);
        }
        else
        {
            auto contour = contours[contours_id];
            if (contourArea(contour) < minArea)
            {   
                minArea = contourArea(contours[contours_id]);
                minAreaContour = contours[contours_id];
            }
            else
            {   
                maxArea = contourArea(contour);
                maxAreaContour = contours[contours_id];
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
    bool temp;
    std::vector<std::vector<cv::Point>> hammerStrips = filterArea(contours, node_->lowerHammerStrip, node_->upperHammerStrip,temp = true);

    findRect(hammerStrips, mode, a.hammerStrip); 

    // std::vector<std::vector<cv::Point>> armourStrip = filterArea(contours, node_->lowerArmourStrip, node_->upperArmourStrip,temp = false);
    // findRect(armourStrip, mode, a.armourStrip);
    std::vector<std::vector<cv::Point>> circleStrip = filterArea(contours, node_->lowerCircleStrip, node_->upperCircleStrip,temp = false);
    findRect(circleStrip, mode, a.circleStrip);
    strip.push_back(a);
    return strip;
}
