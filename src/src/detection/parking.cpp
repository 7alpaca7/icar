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
 * @file parking.cpp
 * @author HC (sasu@saishukeji.com)
 * @brief 充电停车场
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

using namespace cv;
using namespace std;

class Parking
{
public:

    /**
     * @brief 停车步骤
     *
     */
    enum ParkStep
    {
        none = 0, // 未知状态
        enable,   // 停车场使能
        turning,  // 入库转向
        stop,     // 停车
        trackout  // 出库
    };
    
    ParkStep step = ParkStep::none; // 停车步骤

    bool process(Tracking &track, Mat &image, vector<PredictResult> predict)
    {
        counterSession++;
        if (step!= ParkStep::none && counterSession > 80) // 超时退出
        {
            counterRec = 0;
            counterSession = 0;
            step = ParkStep::none;   // 退出状态.
            startTurning = false;    // 恢复状态
            garageFirst = true;      // 进入一号车库
            lineY = 0;               // 直线高度
            ptA = Point(0, 0);       // 清空线段的两个端点
            ptB = Point(0, 0);
            std::cout << "退出停车场" << std::endl;
        }
        switch (step)
        {
            case ParkStep::none: // AI未识别
            {
                for (size_t i = 0; i < predict.size(); i++)
                {
                    if ((predict[i].type == LABEL_BATTERY) && predict[i].score > 0.4)
                    {
                        counterRec++;
                        break;
                    }
                }
                if (counterRec) // 检测到一帧后开始连续监测AI标志是否满足条件
                {
                    if (counterRec >= 3 && counterSession < 8)
                    {
                        counterRec = 0;
                        counterSession = 0;
                        step = ParkStep::enable; // 检测到停车场标志
                        std::cout << "进入停车场" << std::endl;
                        return true;
                    }
                    else if (counterSession >= 8)
                    {
                        counterRec = 0;
                        counterSession = 0;
                    }
                }
    
                return false;
                break;
            }
            case ParkStep::enable: // 停车场使能
            {
                int carY = ROWSIMAGE;
                int batteryY = ROWSIMAGE;     // 充电站标识高度
                
                for (size_t i = 0; i < predict.size(); i++)
                {
                    if (predict[i].type == LABEL_CAR && predict[i].score > 0.6)
                    {
                        carY = (predict[i].y + predict[i].height)/2;   // 计算智能车的中心高度
                    }
                    else if ((predict[i].type == LABEL_BATTERY) && predict[i].score > 0.6)
                    {
                        batteryY = predict[i].y ;   // 计算标识牌最高高度
                    }
                }
                // 图像预处理
                Mat edges;
                Canny(image, edges, 50, 150);

                // 霍夫变换检测直线
                vector<Vec4i> lines;
                HoughLinesP(edges, lines, 1, CV_PI/180, 40, 20, 10);

                vector<Vec4i> horizontalLines;
                Mat imgRes = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
                for(const Vec4i& line : lines) 
                {
                    Point pt1(line[0], line[1]);
                    Point pt2(line[2], line[3]);
                    
                    double angle = atan2(pt2.y - pt1.y, pt2.x - pt1.x) * 180.0 / CV_PI;
                    int midX = (line[0] + line[2]) / 2;
                    int midY = (line[1] + line[3]) / 2;

                    // 筛选直线，直线只出现在右侧并且在充电标识牌的上方
                    if(abs(angle) < 30 && angle < 0 && midY < batteryY && midY < 200 &&  midX > COLSIMAGE/2) 
                    { // 接近水平
                        horizontalLines.push_back(line);
                        cv::line(imgRes, Point(line[0], line[1]), Point(line[2], line[3]), Scalar(0, 0, 255), 2);
                    }
                }
                // imshow("Detected Lines", imgRes);
                // waitKey(0);
                
                if (horizontalLines.size() >= 2)
                {
                    // 查找平行且距离大于20的线对
                    for(size_t i = 0; i < horizontalLines.size(); i++) {

                        for(size_t j = i + 1; j < horizontalLines.size(); j++) {
                            // 计算两条线的中点y坐标
                            int midY1 = (horizontalLines[i][1] + horizontalLines[i][3]) / 2;
                            int midY2 = (horizontalLines[j][1] + horizontalLines[j][3]) / 2;
                            
                            Vec4i line1 = horizontalLines[i];
                            Vec4i line2 = horizontalLines[j];

                            // 计算两条线的角度
                            double angle1 = atan2(line1[3] - line1[1], line1[2] - line1[0]) * 180.0 / CV_PI;
                            double angle2 = atan2(line2[3] - line2[1], line2[2] - line2[0]) * 180.0 / CV_PI;

                            // 如果两条线距离大于20像素并且角度差小于25度
                            if(abs(midY1 - midY2) > 20 && abs(angle1 - angle2) < 25 && step == ParkStep::enable)
                            {

                                if (carY > max(midY1, midY2))
                                {
                                    garageFirst = true;         // 进入一号车库
                                    lineY = min(midY1, midY2);  // 获取距离最远的线控制车入库
                                    step =  ParkStep::turning; // 开始入库
                                    counterSession = 0;
                                    std::cout << "1号车库" << std::endl;
                                }
                                else if (carY < min(midY1, midY2))
                                {
                                    garageFirst = false;        // 进入二号车库
                                    lineY = min(midY1, midY2);  // 获取距离最远的线控制车入库
                                    step =  ParkStep::turning; // 开始入库
                                    counterSession = 0;
                                    std::cout << "2号车库" << std::endl;
                                }
                                else
                                {
                                    counterSession = 0;
                                    step =  ParkStep::turning; // 开始入库
                                    lineY = min(midY1, midY2);  // 获取距离最远的线控制车入库
                                    std::cout << "1号车库" << std::endl;
                                }
                                
                                break;
                            }
                            // else
                            // {
                            //     std::cout << "abs(midY1 - midY2): " << abs(midY1 - midY2) << std::endl;
                            //     std::cout << "abs(angle1 - angle2): " << abs(angle1 - angle2) << std::endl;
                            // }
                        }
                    }
                }
                break;
            }
            case ParkStep::turning: // 入库转向
            {
                // 图像预处理
                Mat edges;
                Canny(image, edges, 50, 150);

                // 霍夫变换检测直线
                vector<Vec4i> lines;
                HoughLinesP(edges, lines, 1, CV_PI/180, 40, 40, 10);

                vector<Vec4i> horizontalLines;
                Mat imgRes = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
                for(const Vec4i& line : lines) {
                    Point pt1(line[0], line[1]);
                    Point pt2(line[2], line[3]);
                    
                    int midX = (line[0] + line[2]) / 2;

                    double angle = atan2(pt2.y - pt1.y, pt2.x - pt1.x) * 180.0 / CV_PI;
                    
                    // 接近水平并且在右侧
                    if(abs(angle) < 40 && angle < 0 &&  midX > COLSIMAGE/2) 
                    { 
                        horizontalLines.push_back(line);
                        cv::line(imgRes, Point(line[0], line[1]), Point(line[2], line[3]), Scalar(0, 0, 255), 2);
                        int midY = (line[1] + line[3]) / 2;     // 计算直线中点y坐标
                        if (midY > lineY && (midY - lineY) <= 10) // 限制线段增加值
                        {
                            lineY = midY; // 更新直线高度
                            ptA = pt1;    // 更新端点
                            ptB = pt2;
                        }
                    }
                }
                // imshow("Detected Lines", imgRes);
                // waitKey(0);
                
                if (lineY > ROWSIMAGE * swerveTime) // 控制转弯时机
                {
                    if (!startTurning)
                    {
                        counterSession = 0;
                        startTurning = true; // 已经开始转弯
                    }
                    std::cout << "控制转弯" << std::endl;
                    // 计算直线的斜率
                    double slope = static_cast<double>(ptB.y - ptA.y) / (ptB.x - ptA.x + 1e-5); // 避免除零

                    int y3 = slope * (0 - ptA.x) + ptA.y;        // 延长起点的Y坐标
                    int y4 = slope * (COLSIMAGE - ptA.x) + ptA.y;// 延长终点的Y坐标

                    Point start(0, y3);           // 延长起点
                    Point end(COLSIMAGE, y4);     // 延长终点
                    
                    track.pointsEdgeLeft.clear(); // 清空原始点集

                    for (int x = start.x; x <= end.x; x++) {
                        int y = static_cast<int>(start.y + slope * (x - start.x)); // 根据斜率计算 y 值
                        POINT pt;
                        pt.x = y; // 将 cv::Point 的 x 赋值给 POINT 的 y
                        pt.y = x; // 将 cv::Point 的 y 赋值给 POINT 的 x
                        track.pointsEdgeLeft.push_back(pt); // 将 POINT 存入点集
                    }

                    pathsEdgeLeft.push_back(track.pointsEdgeLeft); // 记录进厂轨迹
                    pathsEdgeRight.push_back(track.pointsEdgeRight);
                }
                if (counterSession > truningTime && startTurning) // 开始停车状态
                {
                    std::cout << "开始停车" << std::endl;
                    step =  ParkStep::stop; // 开始停车
                }
                break;
            }
            case ParkStep::stop: // 停车
            {
                if (counterSession > stopTime) // 倒车状态
                    step =  ParkStep::trackout; // 开始倒车
                break;
            }
            case ParkStep:: trackout: // 出库
            {
                std::cout << "开始倒车" << std::endl;
                if(pathsEdgeLeft.empty() || pathsEdgeRight.empty()) 
                {
                    counterRec = 0;
                    counterSession = 0;
                    step = ParkStep::none;   // 退出状态.
                    startTurning = false;    // 恢复状态
                    garageFirst = true;      // 进入一号车库
                    lineY = 0;               // 直线高度
                    ptA = Point(0, 0);       // 清空线段的两个端点
                    ptB = Point(0, 0);
                    std::cout << "退出停车场" << std::endl;
                }
                track.pointsEdgeLeft = pathsEdgeLeft[pathsEdgeLeft.size() - 1];
                track.pointsEdgeRight = pathsEdgeRight[pathsEdgeRight.size() - 1];
                pathsEdgeLeft.pop_back();
                pathsEdgeRight.pop_back();
                if (counterSession > 40 && (pathsEdgeLeft.size() < 1 || pathsEdgeRight.size() < 1))
                {
                    counterRec = 0;
                    counterSession = 0;
                    step = ParkStep::none;   // 退出状态.
                    startTurning = false;    // 恢复状态
                    garageFirst = true;      // 进入一号车库
                    lineY = 0;               // 直线高度
                    ptA = Point(0, 0);       // 清空线段的两个端点
                    ptB = Point(0, 0);
                    pathsEdgeRight.clear();
                    pathsEdgeLeft.clear();
                    std::cout << "退出停车场" << std::endl;
                }
                break;
            }
        }

        return true;
        
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
        
        if (step != ParkStep::none)
            putText(image, "[1] BATTERY - ENABLE", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
        
    }

private:
    uint16_t counterSession = 0;  // 图像场次计数器
    uint16_t counterRec = 0;      // 加油站标志检测计数器
    bool garageFirst = true;      // 进入一号车库
    int lineY = 0;                // 直线高度
    bool startTurning = false;    // 开始转弯
    vector<vector<POINT>> pathsEdgeLeft; // 记录入库路径
    vector<vector<POINT>> pathsEdgeRight;
    Point ptA = Point(0, 0);      // 记录线段的两个端点
    Point ptB = Point(0, 0);
    int truningTime = 21;             // 转弯时间 21帧
    int stopTime = 40;                // 停车时间 40帧
    float swerveTime = 0.2;           // 转向时机 0.2 （转弯线出现在屏幕上方0.2处）
};