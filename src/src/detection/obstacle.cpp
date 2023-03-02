/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习,请勿商用).
 *            The code ADAPTS the corresponding hardware circuit board(代码适配百度Edgeboard-智能汽车赛事版),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file obstacle.cpp
 * @author Leo
 * @brief 障碍区AI识别与路径规划
 * @version 0.1
 * @date 2024-01-09
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "../../include/detection.hpp" // Ai模型预测

using namespace std;
using namespace cv;

/**
 * @brief 障碍区AI识别与路径规划类
 *
 */
class Obstacle
{

public:
    /**
     * @brief 障碍区AI识别与路径规划处理
     *
     * @param track 赛道识别结果
     * @param predict AI检测结果
     * @return true
     * @return false
     */
    bool process(Tracking &track, vector<PredictResult> predict, PredictResult &icar_predict)
    {
        enable = false; // 场景检测使能标志
        if (track.pointsEdgeLeft.size() < ROWSIMAGE / 2 || track.pointsEdgeRight.size() < ROWSIMAGE / 2)
            return enable;

        vector<PredictResult> resultsObs; // 锥桶AI检测数据
        for (size_t i = 0; i < predict.size(); i++)
        {
            if ((predict[i].type == LABEL_CONE || predict[i].type == LABEL_BLOCK || predict[i].type == LABEL_PEDESTRIAN) && (predict[i].y + predict[i].height) > ROWSIMAGE * 0.4) // AI标志距离计算
                resultsObs.push_back(predict[i]);
        }

        if (resultsObs.size() <= 0)
            return enable;

        // 选取距离最近的锥桶
        int areaMax = 0; // 框面积
        int index = 0;   // 目标序号
        for (size_t i = 0; i < resultsObs.size(); i++)
        {
            int area = resultsObs[i].width * resultsObs[i].height;
            if (area >= areaMax)
            {
                index = i;
                areaMax = area;
            }
        }
        resultObs = resultsObs[index];
        enable = true; // 场景检测使能标志

        if (resultsObs[index].type == LABEL_BLOCK)
        {
            icar_predict.type = LABEL_BLOCK;
        }
        else if (resultsObs[index].type == LABEL_PEDESTRIAN)
        {
            icar_predict.type = LABEL_PEDESTRIAN;
        }
        else if (resultsObs[index].type == LABEL_CONE)
        {
            icar_predict.type = LABEL_CONE;
        }
        reverse(track.pointsEdgeLeft.begin(), track.pointsEdgeLeft.end());
        reverse(track.pointsEdgeRight.begin(), track.pointsEdgeRight.end());
        // 障碍物方向判定（左/右）
        // row是下标
        // int row = track.pointsEdgeLeft.size() - (resultsObs[index].y + resultsObs[index].height - track.rowCutUp);
        // 这里将row变为识别物的x位置
        int row = resultsObs[index].y + resultsObs[index].height;
        // 当row减去偏移量后，大于两条边线的点集数量后为不需要重新规划
        if (row - track.LP >= track.pointsEdgeLeft.size() && row - track.RP >= track.pointsEdgeRight.size()) // 无需规划路径
            return enable;

        int disLeft = resultsObs[index].x + resultsObs[index].width - track.pointsEdgeLeft[row - track.LP].y;
        int disRight = track.pointsEdgeRight[row - track.RP].y - resultsObs[index].x;
        if (resultsObs[index].x + resultsObs[index].width > track.pointsEdgeLeft[row - track.LP].y &&
            track.pointsEdgeRight[row - track.RP].y > resultsObs[index].x &&
            disLeft <= disRight) //[1] 障碍物靠左
        {
            if (resultsObs[index].type == LABEL_BLOCK) // 黑色路障特殊处理
            {
                curtailTracking(track, false); // 缩减优化车道线（双车道→单车道）
            }
            else if (resultsObs[index].type == LABEL_PEDESTRIAN) // 行人特殊处理
            {
                vector<POINT> points(4); // 三阶贝塞尔曲线
                points[0] = track.pointsEdgeLeft[(row - track.LP) / 2];
                points[1] = {resultsObs[index].y + resultsObs[index].height, resultsObs[index].x + resultsObs[index].width * 2};
                points[2] = {(resultsObs[index].y + resultsObs[index].height + resultsObs[index].y) / 2, resultsObs[index].x + resultsObs[index].width * 2};
                if (resultsObs[index].y > track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].x)
                    points[3] = track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1];
                else
                    points[3] = {resultsObs[index].y, resultsObs[index].x + resultsObs[index].width};

                track.pointsEdgeLeft.resize((size_t)(row - track.LP) / 2); // 删除错误路线
                vector<POINT> repair = Bezier(0.01, points);               // 重新规划车道线
                for (size_t i = 0; i < repair.size(); i++)
                    track.pointsEdgeLeft.push_back(repair[i]);
                curtailTracking(track, false); // 缩减优化车道线（双车道→单车道）
            }
            else
            {
                vector<POINT> points(4); // 三阶贝塞尔曲线
                points[0] = track.pointsEdgeLeft[(row - track.LP) / 2];
                points[1] = {resultsObs[index].y + resultsObs[index].height, resultsObs[index].x + resultsObs[index].width * 2};
                points[2] = {(resultsObs[index].y + resultsObs[index].height + resultsObs[index].y) / 2, resultsObs[index].x + resultsObs[index].width * 2};
                if (resultsObs[index].y > track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].x)
                    points[3] = track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1];
                else
                    points[3] = {resultsObs[index].y, resultsObs[index].x + resultsObs[index].width};

                track.pointsEdgeLeft.resize((size_t)(row - track.LP) / 2); // 删除错误路线
                vector<POINT> repair = Bezier(0.01, points);               // 重新规划车道线
                for (size_t i = 0; i < repair.size(); i++)
                    track.pointsEdgeLeft.push_back(repair[i]);
            }
        }
        else if (resultsObs[index].x + resultsObs[index].width > track.pointsEdgeLeft[row - track.LP].y &&
                 track.pointsEdgeRight[row - track.RP].y > resultsObs[index].x &&
                 disLeft > disRight) //[2] 障碍物靠右
        {
            if (resultsObs[index].type == LABEL_BLOCK) // 黑色路障特殊处理
            {
                curtailTracking(track, true); // 缩减优化车道线（双车道→单车道）
            }
            else if (resultsObs[index].type == LABEL_PEDESTRIAN) // 行人特殊处理
            {
                vector<POINT> points(4); // 三阶贝塞尔曲线
                points[0] = track.pointsEdgeRight[(row - track.RP) / 2];
                points[1] = {resultsObs[index].y + resultsObs[index].height, resultsObs[index].x - resultsObs[index].width};
                points[2] = {(resultsObs[index].y + resultsObs[index].height + resultsObs[index].y) / 2, resultsObs[index].x - resultsObs[index].width};
                if (resultsObs[index].y > track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x)
                    points[3] = track.pointsEdgeRight[track.pointsEdgeRight.size() - 1];
                else
                    points[3] = {resultsObs[index].y, resultsObs[index].x};

                track.pointsEdgeRight.resize((size_t)(row - track.RP) / 2); // 删除错误路线
                vector<POINT> repair = Bezier(0.01, points);                // 重新规划车道线
                for (size_t i = 0; i < repair.size(); i++)
                    track.pointsEdgeRight.push_back(repair[i]);
                curtailTracking(track, true); // 缩减优化车道线（双车道→单车道）
            }
            else
            {
                vector<POINT> points(4); // 三阶贝塞尔曲线
                points[0] = track.pointsEdgeRight[(row - track.RP) / 2];
                points[1] = {resultsObs[index].y + resultsObs[index].height, resultsObs[index].x - resultsObs[index].width};
                points[2] = {(resultsObs[index].y + resultsObs[index].height + resultsObs[index].y) / 2, resultsObs[index].x - resultsObs[index].width};
                if (resultsObs[index].y > track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x)
                    points[3] = track.pointsEdgeRight[track.pointsEdgeRight.size() - 1];
                else
                    points[3] = {resultsObs[index].y, resultsObs[index].x};

                track.pointsEdgeRight.resize((size_t)(row - track.RP) / 2); // 删除错误路线
                vector<POINT> repair = Bezier(0.01, points);                // 重新规划车道线
                for (size_t i = 0; i < repair.size(); i++)
                    track.pointsEdgeRight.push_back(repair[i]);
            }
        }
        reverse(track.pointsEdgeLeft.begin(), track.pointsEdgeLeft.end());
        reverse(track.pointsEdgeRight.begin(), track.pointsEdgeRight.end());
        return enable;
    }

    /**
     * @brief 图像绘制禁行区识别结果
     *
     * @param img 需要叠加显示的图像
     */
    void drawImage(Mat &img)
    {
        if (enable)
        {
            putText(img, "[2] Obstacle - ENABLE", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
            cv::Rect rect(resultObs.x, resultObs.y, resultObs.width, resultObs.height);
            cv::rectangle(img, rect, cv::Scalar(0, 0, 255), 1);
        }
    }

private:
    bool enable = false;     // 场景检测使能标志
    PredictResult resultObs; // 避障目标锥桶

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
};
