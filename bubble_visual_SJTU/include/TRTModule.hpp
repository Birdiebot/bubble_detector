/*
 * @Author: Ligcox
 * @Date: 2022-06-14 16:13:28
 * @FilePath: /bubble/src/bubble_contrib/bubble_visual_SJTU/include/TRTModule.hpp
 * @LastEditors: Ligcox
 * @LastEditTime: 2022-07-02 16:13:19
 * License: GNU General Public License v3.0. See LICENSE file in root directory.
 * Copyright (c) 2022 Birdiebot R&D Department
 * Shanghai University Of Engineering Science. All Rights Reserved
 */
//
// Created by xinyang on 2021/4/8.
//

#ifndef _ONNXTRTMODULE_HPP_
#define _ONNXTRTMODULE_HPP_

#include <opencv2/core.hpp>
#include <NvInfer.h>

struct alignas(4) bbox_t
{
    cv::Point2f pts[4]; // [pt0, pt1, pt2, pt3]
    float confidence;
    int color_id; // 0: blue, 1: red, 2: gray
    int tag_id;   // 0: guard, 1-5: number, 6: base
};

/*
 * 四点模型
 */
class TRTModule
{
    static constexpr int TOPK_NUM = 128;
    static constexpr float KEEP_THRES = 0.1f;

public:
    explicit TRTModule();
    ~TRTModule();
    TRTModule(const TRTModule &) = delete;

    TRTModule operator=(const TRTModule &) = delete;
    std::vector<bbox_t> doinfer(const cv::Mat &src) const;
    void create_module(const std::string &onnx_file);

private:
    void build_engine_from_onnx(const std::string &onnx_file);
    void build_engine_from_cache(const std::string &cache_file);
    void cache_engine(const std::string &cache_file);

    nvinfer1::ICudaEngine *engine;
    nvinfer1::IExecutionContext *context;
    mutable void *device_buffer[2];
    float *output_buffer;
    cudaStream_t stream;
    int input_idx, output_idx;
    size_t input_sz, output_sz;
};

#endif /* _ONNXTRTMODULE_HPP_ */
