#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2025; SaiShu.Lcc.; HC; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习,请勿商用).
 *            The code ADAPTS the corresponding hardware circuit board(代码适配百度Edgeboard-智能汽车赛事版),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file layby.cpp
 * @author HC (sasu@saishukeji.com)
 * @brief 临时停车区
 * @version 0.1
 * @date 2025/03/04 20:29:04
 * @copyright  :Copyright (c) 2024
 * @note 具体功能模块:
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "../../include/detection.hpp"
#include "../recognition/tracking.cpp"
#include "../motion.cpp"
using namespace cv;
using namespace std;

class Layby
{
public:
    bool stopEnable = false; // 停车使能标志

    bool process(Tracking &track, Mat &image, vector<PredictResult> predict, Motion &motion)
    {
        if (laybyEnable) // 进入临时停车状态
        {
            Mat edges;
            Mat blurred;

            // curtailTracking(track, leftEnable); // 缩减优化车道线（双车道→单车道）
            //  直线检测
            GaussianBlur(image, blurred, Size(3, 3), 0); // 添加高斯模糊预处理
            Canny(blurred, edges, 80, 160, 3);           // 调整Canny参数，使用3x3 Sobel算子

            // 霍夫变换检测直线
            vector<Vec4i> lines;
            HoughLinesP(edges, lines, 1, // rho
                        CV_PI / 180,     // theta
                        25,              // threshold：降低阈值
                        30,              // minLineLength：减小最小线段长度
                        30);             // maxLineGap：增大间隙容忍度

            // 存储合并后的线段
            mergedLines.clear();

            // 对检测到的线段进行排序（按y坐标）
            sort(lines.begin(), lines.end(),
                 [](const Vec4i &a, const Vec4i &b)
                 {
                     return (a[1] + a[3]) / 2 < (b[1] + b[3]) / 2;
                 });

            // 找到左边界最左侧的点
            int leftMostX = COLSIMAGE;
            for (size_t i = 0; i < track.pointsEdgeLeft.size(); i++)
            {
                if (track.pointsEdgeLeft[i].x < leftMostX)
                {
                    leftMostX = track.pointsEdgeLeft[i].x;
                }
            }

            // 合并距离相近的线段
            for (const Vec4i &line : lines)
            {
                Point pt1(line[0], line[1]);
                Point pt2(line[2], line[3]);

                // 计算线段中点的y坐标
                int midY = (pt1.y + pt2.y) / 2;

                double slope = abs(static_cast<double>(pt2.y - pt1.y) /
                                   (pt2.x - pt1.x + 1e-5));

                if (slope > 0.5 || midY <= 30 || midY >= 200 || pt1.x < leftMostX || pt2.x < leftMostX)
                    continue; // 跳过非水平线;跳过切行图像

                bool shouldMerge = false;
                // 检查是否有距离相近的已存在线段
                for (Vec4i &merged : mergedLines)
                {
                    int y1 = (merged[1] + merged[3]) / 2; // 已有线段的平均y值
                    int y2 = (pt1.y + pt2.y) / 2;         // 当前线段的平均y值

                    if (abs(y1 - y2) < 20) // 如果y方向距离小于20像素
                    {
                        // 合并并延长线段
                        int minX = min(min(merged[0], merged[2]), min(line[0], line[2]));
                        int maxX = max(max(merged[0], merged[2]), max(line[0], line[2]));
                        int avgY = (y1 + y2) / 2; // 使用平均y值

                        // 更新为合并后的线段
                        merged[0] = minX;
                        merged[1] = avgY;
                        merged[2] = maxX;
                        merged[3] = avgY;

                        shouldMerge = true;
                        break;
                    }
                }

                if (!shouldMerge)
                {
                    mergedLines.push_back(line);
                }
            }
            counterSession++;
            cout << "mergedLines.size()" << mergedLines.size() << endl;

            if (counterSession > all_time) // 结束临时停车状态
            {
                cout << "状态退出" << endl;
                counterRec = 0;
                counterSession = 0;
                laybyEnable = false;
                stopEnable = false;    // 停车使能
                searchingLine = false; // 搜索直线标志
            }
            // else if(mergedLines.size() == 1 )
            // {
            //     cout << "mergedLines[0][0]" << mergedLines[0][0] << endl; // 起点y
            //     cout << "mergedLines[0][1]" << mergedLines[0][1] << endl; // 起点x
            //     cout << "mergedLines[0][2]" << mergedLines[0][2] << endl;
            //     cout << "mergedLines[0][3]" << mergedLines[0][3] << endl;

            // }
            if (mergedLines.size() == 2) // 检测到两条直线
            {
                searchingLine = true; // 搜索直线标志
                count = 1;
                std::cout << "检测到两条线" << std::endl;
                cout << "mergedLines[0][0]" << mergedLines[0][0] << endl; // 起点y
                cout << "mergedLines[0][1]" << mergedLines[0][1] << endl; // 起点x
                cout << "mergedLines[0][2]" << mergedLines[0][2] << endl;
                cout << "mergedLines[0][3]" << mergedLines[0][3] << endl;
                cout << "mergedLines[1][0]" << mergedLines[1][0] << endl; // 起点y
                cout << "mergedLines[1][1]" << mergedLines[1][1] << endl; // 起点x
                cout << "mergedLines[1][2]" << mergedLines[1][2] << endl;
                cout << "mergedLines[1][3]" << mergedLines[1][3] << endl;
                // circle(image, Point(mergedLines[0][1],mergedLines[0][0] ), 5,
                //    Scalar(255, 0, 0), -1); // 绿色点
                //    circle(image, Point(mergedLines[0][3],mergedLines[0][2] ), 5,
                //    Scalar(255, 0, 0), -1); // 绿色点
                //    circle(image, Point(mergedLines[1][1],mergedLines[1][0] ), 5,
                //    Scalar(255, 0, 0), -1); // 绿色点
                //    circle(image, Point(mergedLines[1][3],mergedLines[1][2] ), 5,
                //    Scalar(255, 0, 0), -1); // 绿色点

                // drawImage(track,image);
            }
            else if (mergedLines.size() == 0) // 检测到两条直线
            {
                searchingLine = true; // 搜索直线标志
                count = 2;
            }

            else if (searchingLine && mergedLines.size() == 1 && mergedLines[0][1] > 113 && mergedLines[0][1] < 160 && count > 1)
            {
                cout << "mergedLines[0][0]" << mergedLines[0][0] << endl; // 起点y
                cout << "mergedLines[0][1]" << mergedLines[0][1] << endl; // 起点x
                cout << "mergedLines[0][2]" << mergedLines[0][2] << endl;
                cout << "mergedLines[0][3]" << mergedLines[0][3] << endl;
                std::cout << "开始临时停车" << std::endl;
                // for (int i = 0; i < 1000; i++)
                // {
                //     for (int j = 0; j < 1000; j++)
                //         ;
                // }
                // waitKey(750);
                stopEnable = true; // 停车使能
                searchingLine = false;
                counterSession = 0;
            }

            return true;
        }
        else if (stopEnable)
        {
            counterSession++;
            if (counterSession > stopTime)
            {
                counterSession = 0;
                counterRec = 0;
                counterSession = 0;
                laybyEnable = false;
                searchingLine = false; // 搜索直线标志
            }
            return true;
        }
        else // 检测标志
        {
            for (size_t i = 0; i < predict.size(); i++)
            {
                if (((predict[i].type == LABEL_SCHOOL || predict[i].type == LABEL_COMPANY) && predict[i].score >= 0.42) && (predict[i].y + predict[i].height) > ROWSIMAGE * 0.2)
                {
                    counterRec++;
                    if (predict[i].x < COLSIMAGE / 2) // 标识牌在左侧
                        leftEnable = true;
                    else
                        leftEnable = false;
                    break;
                }
            }

            if (counterRec)
            {
                counterSession++;
                if (counterRec >= 1 && counterSession < 3)
                {
                    counterRec = 0;
                    counterSession = 0;
                    laybyEnable = true; // 检测到标识牌子
                    return true;
                }
                else if (counterSession >= 3)
                {
                    counterRec = 0;
                    counterSession = 0;
                }
            }

            return false;
        }
    }

    /**
     * @brief 识别结果图像绘制
     *
     */
    void drawImage(Tracking track, Mat &image)
    {
        // 赛道边缘
        for (size_t i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(image, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 1,
                   Scalar(0, 255, 0), -1); // 绿色点
        }
        for (size_t i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(image, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1,
                   Scalar(0, 255, 255), -1); // 黄色点
        }

        // 绘制合并后的结果
        for (const Vec4i &line : mergedLines)
        {
            cv::line(image, Point(line[0], line[1]), Point(line[2], line[3]), Scalar(0, 0, 255), 2);
        }

        if (laybyEnable)
            putText(image, "[1] Layby - ENABLE", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    }

    /**
     * @brief 缩减优化车道线（双车道→单车道）
     *
     * @param track
     * @param left
     */
    void curtailTracking(Tracking &track, bool left)
    {
        if (left) // 向左侧缩进
        {
            if (track.pointsEdgeRight.size() > track.pointsEdgeLeft.size())
                track.pointsEdgeRight.resize(track.pointsEdgeLeft.size());

            for (size_t i = 0; i < track.pointsEdgeRight.size(); i++)
            {
                track.pointsEdgeRight[i].y = (track.pointsEdgeRight[i].y + track.pointsEdgeLeft[i].y) / 2;
            }
        }
        else // 向右侧缩进
        {
            if (track.pointsEdgeRight.size() < track.pointsEdgeLeft.size())
                track.pointsEdgeLeft.resize(track.pointsEdgeRight.size());

            for (size_t i = 0; i < track.pointsEdgeLeft.size(); i++)
            {
                track.pointsEdgeLeft[i].y = (track.pointsEdgeRight[i].y + track.pointsEdgeLeft[i].y) / 2;
            }
        }
    }

private:
    uint16_t counterSession = 0; // 图像场次计数器
    uint16_t counterRec = 0;     // 标识牌检测计数器
    bool laybyEnable = false;    // 临时停车区域使能标志
    bool leftEnable = true;      // 标识牌在左侧
    bool searchingLine = false;  // 搜索直线标志
    vector<Vec4i> mergedLines;   // 合并后的线段用于绘制
    int moment = 40;             // 停车时机，屏幕上方的像素值，值越大越越晚停车
    int stopTime = 10;           // 停车时间 40帧
    int count = 0;
    int all_time = 100;
};