/*
 * @Author: Ligcox
 * @Date: 2022-02-14 00:44:40
 * @FilePath: /bubble_bringup/home/nvidia/Desktop/bubble/src/bubble_contrib/bubble_visual/src/filter.cpp
 * @LastEditors: Ligcox
 * @LastEditTime: 2022-05-14 20:58:43
 * License: GNU General Public License v3.0. See LICENSE file in root directory.
 * Copyright (c) 2022 Birdiebot R&D Department
 * Shanghai University Of Engineering Science. All Rights Reserved
 */

// #include "filter.hpp"

// template <typename T>
// cv::Mat HSVFilter(cv::Mat &image, T *node_)
// {
//     static const int NUM_CHANNEL = 3;
//     std::vector<int> targetColorMin{node_->hueMin, node_->saturationMin, node_->valueMin};
//     std::vector<int> targetColorMax{node_->hueMax, node_->saturationMax, node_->valueMax};

//     static std::vector<std::vector<int>> channel_target;
//     for (int i = 0; i < NUM_CHANNEL; i++)
//     {
//         channel_target.push_back(std::vector<int>());
//         channel_target[i].push_back(std::max(0, targetColorMin[i]));
//         channel_target[i].push_back(std::min(255, targetColorMax[i]));
//     }

//     std::vector<cv::Mat> channel_masks{};
//     cv::Mat final_mask = cv::Mat(image.rows, image.cols, CV_8UC1, cv::Scalar(255, 255, 255));
//     std::vector<cv::Mat> hsvChannel;
//     cv::split(image, hsvChannel);

//     for (int channel = 0; channel < NUM_CHANNEL; channel++)
//     {
//         int test = channel_target[channel][0];
//         std::cout << test << std::endl;
//         cv::Mat low_mask, high_mask;
//         cv::threshold(hsvChannel[channel], low_mask, channel_target[channel][0], 255, cv::THRESH_BINARY);
//         cv::threshold(hsvChannel[channel], high_mask, channel_target[channel][1], 255, cv::THRESH_BINARY_INV);
//         cv::Mat mask;
//         cv::bitwise_and(low_mask, high_mask, mask);
//         channel_masks.push_back(mask);
//         if (channel == 1 || channel == 0)
//             continue;
//         cv::bitwise_and(final_mask, channel_masks[channel], final_mask);
//     }
//     cv::Mat output;
//     cv::bitwise_and(image, image, output, final_mask);

//     cv::imshow("channel_masks", channel_masks[2]);
//     cv::imshow("HSVFiler", output);
//     cv::waitKey(1);
//     // _node.debugImg(final_mask, channel_masks);
//     return output;
// }