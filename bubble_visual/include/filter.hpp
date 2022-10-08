/*
 * @Author: Ligcox
 * @Date: 2022-02-14 05:33:40
 * @FilePath: /bubble/src/bubble_contrib/bubble_visual/include/filter.hpp
 * @LastEditors: HarryWen
 * @LastEditTime: 2022-06-02 19:44:19
 * License: GNU General Public License v3.0. See LICENSE file in root directory.
 * Copyright (c) 2022 Birdiebot R&D Department
 * Shanghai University Of Engineering Science. All Rights Reserved
 */
#pragma once
#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>

template <typename T>
cv::Mat HSVFilter(cv::Mat &image, T *node_)
{
    static const int NUM_CHANNEL = 3;
    std::vector<int> targetColorMin{node_->hueMin, node_->saturationMin, node_->valueMin};
    std::vector<int> targetColorMax{node_->hueMax, node_->saturationMax, node_->valueMax};

    std::vector<std::vector<int>> channel_target;
    for (int i = 0; i < NUM_CHANNEL; i++)
    {
        channel_target.push_back(std::vector<int>{});
        channel_target[i].push_back(std::max(0, targetColorMin[i]));
        channel_target[i].push_back(std::min(255, targetColorMax[i]));
    }

    std::vector<cv::Mat> channel_masks;
    cv::Mat final_mask = cv::Mat(image.rows, image.cols, CV_8UC1, cv::Scalar(255, 255, 255));
    std::vector<cv::Mat> hsvChannel;
    cv::split(image, hsvChannel);

    for (int channel = 0; channel < NUM_CHANNEL; channel++)
    {
        cv::Mat low_mask, high_mask;
        cv::threshold(hsvChannel[channel], low_mask, channel_target[channel][0]-1, 255, cv::THRESH_BINARY);
        cv::threshold(hsvChannel[channel], high_mask, channel_target[channel][1]+1, 255, cv::THRESH_BINARY_INV);
        cv::Mat mask;
        cv::bitwise_and(low_mask, high_mask, mask);
        channel_masks.push_back(mask);

        cv::bitwise_and(final_mask, channel_masks[channel], final_mask);
    }
    cv::Mat output;
    cv::bitwise_and(image, image, output, final_mask);
    return final_mask;
}

cv::Mat GrayFilter(cv::Mat &image, bool mode, int grayThres)
{
    cv::Mat output;
    cv::Mat grayImg;
    cv::Mat channels[3];
    cv::split(image, channels);
    if (mode)
    {
        grayImg = channels[2] - channels[0];
    }
    else
    {
        grayImg = channels[0] - channels[2];
    }
    cv::threshold(grayImg, output, grayThres, 255, cv::THRESH_BINARY);
    return output;
}
