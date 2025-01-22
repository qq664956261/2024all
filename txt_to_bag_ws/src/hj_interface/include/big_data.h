// @file big_data.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2024-6-28)
#ifndef INCLUDE_BIG_DATA_H_  // NOLINT
#define INCLUDE_BIG_DATA_H_  // NOLINT

#include <memory>
#include <string>

namespace big_data {
constexpr char kBigdataCmdTopicName[] = "/big_data_cmd";
enum BigdataCmd { kBigdataImmediate = 1, kBigdataPack = 2, kBigdataDelete = 4 };
// void insertBigdata(std::string key, std::string value, BigdataCmd cmd = BigdataCmd::kBigdataImmediate);
/**
 * @brief 插入大数据（埋点数据）
 *
 * @param value 大数据已组好的内容
 * @param param 带压缩包上传时，压缩包路径
 * @param cmd
 * kBigdataImmediate：立刻发送，当前版本默认带，kBigdataPack：需要带压缩包时，带上这个参数，用法：kBigdataImmediate|kBigdataPack
 */
void InsertBigdata(const std::string& value, const std::string& param = "",
                   uint8_t cmd = BigdataCmd::kBigdataImmediate);
void Init();
}  // namespace big_data
#endif  //  INCLUDE_BIG_DATA_H_  // NOLINT
