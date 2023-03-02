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
    bool stopEnable = false; // 停车使能标志
    bool noRing = false;     // 用来区分环岛路段

    int defence(const int &now, const int min, const int max)
    {
        int x = now;
        if (now < min)
            x = min;
        else if (now > max)
            x = max;
        return x;
    }

    int findUP(const vector<POINT> &edge, const vector<POINT> &spurroad, const int start, const int end, const int &EP)
    {
        for (int i = start; i < end; i++)
        {
            int here = spurroad[i].x - EP, pre = spurroad[i].x - EP - jian, next = spurroad[i].x - EP + jian;
            here = defence(here, 0, edge.size() - 1);
            pre = defence(pre, 0, edge.size() - 1);
            next = defence(next, 0, edge.size() - 1);
            int ah = edge[here].y, ap = edge[pre].y, an = edge[an].y;
            if (abs(ah - an) > jian_judge && abs(ah - ap) < jian_judge)
            {
                return i;
            }
        }
        return -1;
    }

    int findDown(const vector<POINT> &spurroad, const int start, const int end, const int dui)
    {
        for (int i = end - 1; i >= start; i--)
        {
            if (abs(spurroad[i].x - dui) <= buger_jian)
            {
                return i;
            }
        }
        return -1;
    }

    void clean(vector<POINT> &edge, const int hang, int &P)
    {
        int i = 0;
        bool flag = false;
        for (; i < edge.size(); i++)
        {
            if (edge[i].x >= hang)
            {
                flag = true;
                break;
            }
        }
        if (flag)
        {
            edge.erase(edge.begin(), edge.begin() + i + 5);
            P = edge[0].x;
        }
    }

    void Bu_UD(vector<POINT> &alert, POINT up, POINT EStart,int &AP)
    {
        if (textDebug)
            cout << "开始Bu_UD" << endl;
        vector<POINT> huan = {up, POINT(up.x, EStart.y), EStart};
        huan = Bezier(0.01, huan);
        alert.clear();
        alert = huan;
    }

    void Bu_D(vector<POINT> &alert, POINT up, POINT EStart, const int hang, int &AP)
    {
        if (textDebug)
            cout << "开始Bu_D" << endl;
        vector<POINT> huan = {EStart, POINT(up.x, EStart.y), up};
        huan = Bezier(0.01, huan);
        clean(alert, hang, AP);
        reverse(alert.begin(), alert.end());
        alert.insert(alert.end(), huan.begin(), huan.end());
        reverse(alert.begin(), alert.end());
    }

    void burger_go(vector<POINT> &keep, vector<POINT> &alert, int &KP, int &AP, const int start, const int end, const int &dui, const vector<POINT> &spurroad, const POINT &origin)
    {
        flagDown = findDown(spurroad, start, end, dui), flagUP;
        if (flagDown)
        {
            if (textDebug)
                cout << "找到下角点,flagDown: " << flagDown << endl;
            flagUP = findUP(keep, spurroad, start, flagDown, KP);
        }
        else
        {
            if (textDebug)
                cout << "没找到下角点" << endl;
            flagUP = findUP(keep, spurroad, start, end, KP);
        }
        if (flagUP)
        {
            if (textDebug)
                cout << "找到上角点" << endl;
            clean(keep, spurroad[flagUP].x, KP);
            Bu_UD(alert, spurroad[flagUP], origin,AP);
        }
        else
        {
            if (textDebug)
                cout << "找到下角点" << endl;
            Bu_D(alert, POINT(0, spurroad[flagDown].y), POINT(spurroad[flagDown].x, alert[spurroad[flagDown].x - AP].y), spurroad[flagDown].x, AP);
        }
    }

    bool process(Tracking &track, Mat &image, vector<PredictResult> predict)
    {
        if (cateringEnable) // 进入岔路
        {
            if (textDebug)
                printf("流程中,nowStep: %2d\n", nowStep);
            if (nowStep == CStep::guai)
            {
                for (size_t i = 0; i < predict.size(); i++)
                {
                    if (predict[i].type == LABEL_BURGER)
                    {
                        burgerY = predict[i].y + predict[i].height / 2;
                        if (textDebug)
                        {
                            printf("burgerX:%3d,burgerY:%3d,burgerHeight:%3d", predict[i].x, predict[i].y, burgerY);
                        }
                        break;
                    }
                }
                if (textDebug)
                    printf("now burgerY: %3d\n", burgerY);
                if (burgerLeft)
                {
                    if (textDebug)
                        cout << "开始左汉堡" << endl;
                    burger_go(track.pointsEdgeRight, track.pointsEdgeLeft, track.RP, track.LP, track.LGnum, track.spurroad.size(), burgerY, track.spurroad, track.R_Start);
                }
                else
                {
                    if (textDebug)
                        cout << "开始右汉堡" << endl;
                    burger_go(track.pointsEdgeLeft, track.pointsEdgeRight, track.LP, track.RP, 0, track.LGnum, burgerY, track.spurroad, track.L_Start);
                }
                if (flagDown == -1)
                {
                    downDiu++;
                }
                else
                {
                    downDiu = 0;
                }
                if (downDiu > allowDiu)
                {
                    zhen = 0;
                    nowStep = CStep::stop;
                }
            }
            else if (nowStep == CStep::stop)
            {
                zhen++;
                if (zhen > travelTime)
                {
                    zhen = 0;
                    stopEnable = true;
                    nowStep = CStep::out;
                }
            }
            else if (nowStep == CStep::out)
            {
                zhen++;
                if (zhen > stopTime)
                {
                    stopEnable = false;
                    zhen = 0;
                    nowStep = CStep::finish;
                }
            }
            else if (nowStep == CStep::finish)
            {
                zhen++;
                if (zhen > allowDiu)
                {
                    zhen = 0;
                    nowStep = CStep::nothing;
                    counterRec = 0;
                    counterSession = 0;
                    cateringEnable = false;
                }
            }
            // counterSession++;
            // if (counterSession > (truningTime + travelTime + stopTime)) // 结束餐饮区域
            // {
            // counterRec = 0;
            // counterSession = 0;
            // cateringEnable = false;
            //     turning = true;     // 转向标志
            //     stopEnable = false; // 停车使能
            //     // noRing = false;     // 区分环岛
            // }
            // else if (counterSession > (truningTime + travelTime)) // 驶入餐饮区
            //     stopEnable = true;                                // 停车使能
            // else if (counterSession > truningTime)                // 进入岔路
            //     turning = false;                                  // 关闭转向标志

            return true;
        }
        else // 检测汉堡标志
        {
            for (size_t i = 0; i < predict.size(); i++)
            {
                if (predict[i].type == LABEL_BURGER && predict[i].score > 0.4 && (predict[i].y + predict[i].height) > ROWSIMAGE * 0.4)
                {
                    counterRec++;
                    // noRing = true;
                    if (predict[i].x < COLSIMAGE / 2) // 汉堡在左侧
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
                    // burger_flag = true;
                    nowStep = CStep::guai;
                    if (textDebug)
                        cout << "检测到汉堡开始走流程" << endl;
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
    void
    drawImage(Tracking track, Mat &image)
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
    uint16_t counterSession = 0; // 图像场次计数器
    uint16_t counterRec = 0;     // 汉堡标志检测计数器
    bool cateringEnable = false; // 岔路区域使能标志
    bool burgerLeft = true;      // 汉堡在左侧
    // bool turning = true;         // 转向标志
    int burgerY = 0; // 汉堡高度
    // int truningTime = 25;        // 转弯时间 25帧
    int travelTime = 10; // 行驶时间 10帧 在斜线路段的行驶时间
    int stopTime = 25;   // 停车时间 25帧
    int jian = 5;        // 前后对比的间隔
    int jian_judge = 10, buger_jian = 20;
    bool textDebug = true;
    bool burger_flag = false;
    int downDiu = 0;
    int allowDiu = 5;
    int flagDown, flagUP;
    enum CStep
    {
        nothing = 0,
        guai,
        stop,
        out,
        finish
    };
    CStep nowStep = CStep::nothing;
    int zhen = 0;
};