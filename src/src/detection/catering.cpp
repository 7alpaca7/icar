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
 * @file catering.cpp
 * @author HC (sasu@saishukeji.com)
 * @brief 餐饮区
 * @version 0.1
 * @date 2025/03/03 09:53:17
 * @copyright  :Copyright (c) 2024
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "../../include/detection.hpp"
#include "../recognition/tracking.cpp"

using namespace cv;
using namespace std;

class Catering
{
public:

    bool stopEnable = false;        // 停车使能标志
    bool noRing = true;            // 用来区分环岛路段

    bool process(Tracking &track, Mat &image, vector<PredictResult> predict)
    {
        if (cateringEnable) // 进入岔路
        {   
            if (!stopEnable && turning)
            {
                for (size_t i = 0; i < predict.size(); i++)
                {
                    if (predict[i].type == LABEL_BURGER)
                    {
                        burgerY = predict[i].y;   // 计算汉堡最高高度
                    }
                }

                // 边缘检测
                Mat edges;
                Mat blurred;
                GaussianBlur(image, blurred, Size(3, 3), 0);  // 添加高斯模糊预处理
                Canny(blurred, edges, 30, 150, 3);  // 调整Canny参数，使用3x3 Sobel算子

                // 霍夫变换检测直线
                vector<Vec4i> lines;
                HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 50, 10);

                // 遍历检测到的直线
                for (size_t i = 0; i < lines.size(); i++) {
                    Vec4i line = lines[i];
                    Point pt1(line[0], line[1]); // 直线起点
                    Point pt2(line[2], line[3]); // 直线终点

                    // 计算直线的斜率
                    double slope = static_cast<double>(pt2.y - pt1.y) / (pt2.x - pt1.x + 1e-5); // 避免除零
                    
                    int maxY = max(line[1],line[3]); // 直线最左侧(最下方)的Y坐标
                    // std::cout << "直线最左侧(最下方)的Y坐标: " << maxY << "汉堡Y: " << burgerY << std::endl;
                    if (maxY > burgerY) // 如果找到的直线低于汉堡则跳过
                        continue;
                    
                    // 限定斜率
                    if ((slope > -0.3 || slope < -1) && burgerLeft) {
                        continue; // 跳过不符合斜率条件的直线
                    }
                    else if ((slope < 0.3 || slope > 1) && !burgerLeft ) {
                        continue; // 跳过不符合斜率条件的直线
                    }
                    
                    int y3 = slope * (0 - pt1.x) + pt1.y;        // 延长起点的Y坐标
                    int y4 = slope * (COLSIMAGE - pt1.x) + pt1.y;// 延长终点的Y坐标
                        
                    Point start(0, y3);           // 延长起点
                    Point end(COLSIMAGE, y4);     // 延长终点
                    
                    if (burgerLeft)
                        track.pointsEdgeLeft.clear(); // 清空原始点集
                    else
                        track.pointsEdgeRight.clear(); // 清空原始点集

                    for (int x = start.x; x <= end.x; x++) {
                        int y = static_cast<int>(start.y + slope * (x - start.x)); // 根据斜率计算 y 值
                        POINT pt;
                        pt.x = y; // 将 cv::Point 的 x 赋值给 POINT 的 y
                        pt.y = x; // 将 cv::Point 的 y 赋值给 POINT 的 x
                        if (burgerLeft)
                            track.pointsEdgeLeft.push_back(pt); // 将 POINT 存入点集
                        else
                            track.pointsEdgeRight.push_back(pt); // 将 POINT 存入点集
                    }

                    // 如果找到符合条件的直线，绘制并输出
                    // Mat imgRes = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
                    // cv::line(imgRes, start, end, Scalar(0, 255, 0), 2); // 用黄色绘制符合条件的直线
                    // std::cout << "检测到符合条件的斜线: (" << pt1.x << "," << pt1.y << ") -> (" << pt2.x << "," << pt2.y << ")" << endl;
                    // 显示结果
                    // imshow("Detected Lines", imgRes);
                    // waitKey(0);
                }
            }

            counterSession++;
            if (counterSession > (truningTime + travelTime + stopTime))  // 结束餐饮区域
            {
                counterRec = 0;
                counterSession = 0;
                cateringEnable = false;
                turning = true;       // 转向标志
                stopEnable = false;  // 停车使能
                noRing = false;      // 区分环岛
            }
            else if (counterSession > (truningTime + travelTime)) // 驶入餐饮区
                stopEnable = true;  // 停车使能
            else if (counterSession > truningTime) // 进入岔路
                turning = false;   // 关闭转向标志

            return true;
        }
        else // 检测汉堡标志
        {
            for (size_t i = 0; i < predict.size(); i++)
            {
                if (predict[i].type == LABEL_BURGER && predict[i].score > 0.4 && (predict[i].y + predict[i].height) > ROWSIMAGE * 0.3)
                {
                    counterRec++;
                    noRing = true;
                    if (predict[i].x < COLSIMAGE / 2)   // 汉堡在左侧
                        burgerLeft = true;
                    else
                        burgerLeft = false;
                    break;
                }
            }

            if (counterRec)
            {
                counterSession++;
                if (counterRec >= 3 && counterSession < 8)
                {
                    counterRec = 0;
                    counterSession = 0;
                    cateringEnable = true; // 检测到汉堡标志
                    return true;
                }
                else if (counterSession >= 8)
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
        
        if (cateringEnable)
            putText(image, "[1] Burger - ENABLE", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    }

private:
    uint16_t counterSession = 0;    // 图像场次计数器
    uint16_t counterRec = 0;        // 汉堡标志检测计数器
    bool cateringEnable = false;    // 岔路区域使能标志
    bool burgerLeft = true;         // 汉堡在左侧
    bool turning = true;            // 转向标志
    int burgerY = 0;                // 汉堡高度
    int truningTime = 25;           // 转弯时间 25帧
    int travelTime = 10;            // 行驶时间 10帧 在斜线路段的行驶时间
    int stopTime = 25;              // 停车时间 25帧
};