#pragma once
#include <vector>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <fstream>
#include <iostream>
#include <set>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
// #include "./common.hpp" // 赛道点集类
#define COLSIMAGE 320   // 图像的列数
#define ROWSIMAGE 240   // 图像的行数
#define WHITE 255
#define BLACK 0
#define PI (3.14159265)
using namespace std;
using namespace cv;

class Tracking
{
private:
    const int L_direction[8][2] = {{1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}, {0, 1}, {1, 1}};
    const int R_direction[8][2] = {{1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
    Mat imagePath; // 赛道搜索图像
public:
    vector<POINT> pointsEdgeLeft;  // 左边线
    vector<POINT> pointsEdgeRight; // 右边线
    vector<POINT> pointsEdgeZhong; // 中间线
    vector<POINT> pointsLeft, pointsRight;
    vector<float> left_angle, right_angle;
    // vector<POINT> pointsLGuai;                    // 左线拐点集
    // vector<POINT> pointsRGuai;                    // 右线拐点集
    vector<POINT> widthBlock;                     // 色块宽度=终-起（每行）
    vector<POINT> spurroad;                       // 保存岔路信息
    double stdevLeft = 0.0;                       // 左边缘斜率方差
    double stdevRight = 0.0;                      // 右边缘斜率方差
    int validRowsLeft = 1000;                     // 边缘有效行数（左）
    int validRowsRight = 1000;                    // 边缘有效行数（右）
    int MaxLeftY = 0;                             // 左边线最大y坐标
    int MinRightY = COLSIMAGE - 1;                // 右边线最小y坐标
    int diuLeft = 0;                              // 左边丢线数量
    int diuRight = 0;                             // 右边丢线数量
    __uint16_t rowCutUp = 3;                      // 图像顶部切行
    __uint16_t rowCutBottom = 50;                 // 图像底部切行
    __uint16_t rowEnd = ROWSIMAGE - rowCutBottom; // 底部起始
    bool save = false;                            // 是否打印图像
    int jian = 7;
    int window = 5;                // nms抑制的窗口大小
    int point_num = 200, dist = 7; // 等间距采样的点数和间距
    int kernel = 5;                // 滤波的窗口

    void trackRecognition(bool isResearch, uint16_t rowStart)
    {
        if (!isResearch)
        {
            pointsEdgeLeft.clear();
            pointsEdgeRight.clear();
            pointsEdgeZhong.clear();
            // pointsLGuai.clear();
            // pointsRGuai.clear();
            pointsLeft.clear();
            pointsRight.clear();
            spurroad.clear();
            validRowsLeft = 1000;
            validRowsRight = 1000;
            widthBlock.clear();
            left_angle.clear();
            right_angle.clear();
        }
        else
        {
            if (pointsEdgeLeft.size() > rowStart)
                pointsEdgeLeft.resize(rowStart);
            if (pointsEdgeRight.size() > rowStart)
                pointsEdgeRight.resize(rowStart);
            if (pointsEdgeZhong.size() > rowStart)
                pointsEdgeZhong.resize(rowStart);
            if (widthBlock.size() > rowStart)
            {
                widthBlock.resize(rowStart);
                if (rowStart > 1)
                    rowStart = widthBlock[rowStart - 1].x - 2;
            }
        }

        // 清理图像边界
        for (int i = 0; i < ROWSIMAGE; i++)
        {
            imagePath.at<uchar>(i, 0) = BLACK;
            imagePath.at<uchar>(i, 1) = BLACK;
            imagePath.at<uchar>(i, COLSIMAGE - 1) = BLACK;
            imagePath.at<uchar>(i, COLSIMAGE - 2) = BLACK;
        }
        for (int i = 0; i < COLSIMAGE; i++)
        {
            imagePath.at<uchar>(0, i) = BLACK;
            imagePath.at<uchar>(1, i) = BLACK;
        }

        POINT L_Start, R_Start;
        bool leftFound = false, rightFound = false;

        for (int i = COLSIMAGE / 2; i > 0; i--)
        {
            if (rowEnd >= 0 && rowEnd < ROWSIMAGE && i + 1 < COLSIMAGE)
            {
                if (imagePath.at<uchar>(rowEnd, i) == BLACK && imagePath.at<uchar>(rowEnd, i + 1) == WHITE)
                {
                    L_Start = POINT(rowEnd, i, 4);
                    leftFound = true;
                    break;
                }
            }
        }

        for (int i = COLSIMAGE / 2; i < COLSIMAGE - 1; i++)
        {
            if (rowEnd >= 0 && rowEnd < ROWSIMAGE && i + 1 < COLSIMAGE)
            {
                if (imagePath.at<uchar>(rowEnd, i) == WHITE && imagePath.at<uchar>(rowEnd, i + 1) == BLACK)
                {
                    R_Start = POINT(rowEnd, i + 1, 4);
                    rightFound = true;
                    break;
                }
            }
        }

        if (!leftFound || !rightFound)
        {
            cout << "起始点未找到，trackRecognition退出" << endl;
            return;
        }

        auto idx = [](int x, int y)
        { return x * COLSIMAGE + y; };

        // 替换 vector<bool> 为 vector<uint8_t>
        vector<uint8_t> visitedLeft(ROWSIMAGE * COLSIMAGE, 0);
        vector<uint8_t> visitedRight(ROWSIMAGE * COLSIMAGE, 0);

        int L_dir = L_Start.direction, R_dir = R_Start.direction;
        int count = 1000;
        bool l_flag = true, r_flag = true, grow_l = true, grow_r = true;

        POINT leftPoint = L_Start;
        POINT rightPoint = R_Start;

        pointsLeft.clear();
        pointsRight.clear();

        pointsLeft.push_back(leftPoint);
        pointsRight.push_back(rightPoint);
        visitedLeft[idx(leftPoint.x, leftPoint.y)] = 1;
        visitedRight[idx(rightPoint.x, rightPoint.y)] = 1;

        while (count-- > 0)
        {
            if (leftPoint.y >= rightPoint.y)
                break;

            // 左边线生长（黑->白，取黑）
            if (grow_l)
            {
                if (l_flag)
                {
                    for (int i = 0; i < 8; i++)
                    {
                        int dir = (L_dir + i) % 8;
                        int next_dir = (L_dir + i + 1) % 8;
                        int x = leftPoint.x + L_direction[dir][0];
                        int y = leftPoint.y + L_direction[dir][1];
                        int nx = leftPoint.x + L_direction[next_dir][0];
                        int ny = leftPoint.y + L_direction[next_dir][1];

                        if (x < 0 || x >= ROWSIMAGE || y < 0 || y >= COLSIMAGE)
                            continue;
                        if (nx < 0 || nx >= ROWSIMAGE || ny < 0 || ny >= COLSIMAGE)
                            continue;
                        int pos = idx(x, y);
                        if (imagePath.at<uchar>(x, y) == BLACK &&
                            imagePath.at<uchar>(nx, ny) == WHITE &&
                            visitedLeft[pos] == 0 &&
                            y < rightPoint.y)
                        {
                            leftPoint = POINT(x, y, dir);
                            visitedLeft[pos] = 1;
                            L_dir = dir;
                            l_flag = false;
                            break;
                        }
                    }
                }
            }

            // 右边线生长（白->黑，取黑）
            if (grow_r)
            {
                if (r_flag)
                {
                    for (int i = 0; i < 8; i++)
                    {
                        int dir = (R_dir + i) % 8;
                        int next_dir = (R_dir + i + 1) % 8;
                        int x = rightPoint.x + R_direction[dir][0];
                        int y = rightPoint.y + R_direction[dir][1];
                        int nx = rightPoint.x + R_direction[next_dir][0];
                        int ny = rightPoint.y + R_direction[next_dir][1];
                        if (x < 0 || x >= ROWSIMAGE || y < 0 || y >= COLSIMAGE)
                            continue;
                        if (nx < 0 || nx >= ROWSIMAGE || ny < 0 || ny >= COLSIMAGE)
                            continue;
                        int pos = idx(x, y);
                        if (imagePath.at<uchar>(nx, ny) == WHITE &&
                            imagePath.at<uchar>(x, y) == BLACK &&
                            visitedRight[pos] == 0 &&
                            y > leftPoint.y)
                        {
                            rightPoint = POINT(x, y, dir);
                            visitedRight[pos] = 1;
                            R_dir = dir;
                            r_flag = false;
                            break;
                        }
                    }
                }
            }

            // 左右边线等高处理
            if (leftPoint.x > rightPoint.x)
            {
                grow_r = false;
                grow_l = true;
                if (!l_flag)
                {
                    l_flag = true;
                    pointsLeft.push_back(leftPoint);
                }
            }
            else if (leftPoint.x < rightPoint.x)
            {
                grow_l = false;
                grow_r = true;
                if (!r_flag)
                {
                    r_flag = true;
                    pointsRight.push_back(rightPoint);
                }
            }
            else
            {
                grow_l = true;
                grow_r = true;
                if (!l_flag)
                {
                    l_flag = true;
                    pointsLeft.push_back(leftPoint);
                }
                if (!r_flag)
                {
                    r_flag = true;
                    pointsRight.push_back(rightPoint);
                }
            }
        }
        // cout << count << endl;
        get_line(pointsLeft, pointsRight);

        resample_points(pointsLeft, point_num, dist);
        resample_points(pointsRight, point_num, dist);
        local_angle(pointsRight, right_angle);
        local_angle(pointsLeft, left_angle);

        nms_angle(left_angle, window);
        nms_angle(right_angle, window);
        add_supproad(left_angle, pointsLeft);
        add_supproad(right_angle, pointsRight);

        // medianFilterPointsY(pointsEdgeLeft, 5);
        // medianFilterPointsY(pointsEdgeRight, 5);

        // zhong_line();
        // for (size_t i = 0; i < pointsEdgeLeft.size(); ++i)
        //     slopeCal(pointsEdgeLeft, i);
        // for (size_t i = 0; i < pointsEdgeRight.size(); ++i)
        //     slopeCal(pointsEdgeRight, i);

        // detectGuaiDianCombined(pointsEdgeLeft);  // 左线
        // detectGuaiDianCombined(pointsEdgeRight); // 右线
        // detectCornerByVarianceLight(pointsEdgeLeft, spurroad);
        // detectCornerByVarianceLight(pointsEdgeRight, spurroad);

        // // drawImage(imagePath);
        cout << "pointsLeft.size()=" << pointsEdgeLeft.size() << endl;
        cout << "pointsRight.size()=" << pointsEdgeRight.size() << endl;
        cout << "pointsEdgeLeft.size()=" << pointsEdgeLeft.size() << endl;
        cout << "pointsEdgeRight.size()=" << pointsEdgeRight.size() << endl;
        // // cout << "pointsLGuai.size()=" << pointsLGuai.size() << endl;
        // // cout << "pointsRGuai.size()=" << pointsRGuai.size() << endl;
        cout << "spurroad.size()=" << spurroad.size() << endl;
        stdevLeft = stdevEdgeCal(pointsEdgeLeft, ROWSIMAGE);
        stdevRight = stdevEdgeCal(pointsEdgeRight, ROWSIMAGE);
        cout << "stdevLeft=" << stdevLeft << endl;
        cout << "stdevRight=" << stdevRight << endl;
        widthBlock.resize(pointsEdgeLeft.size(),POINT(0,0));
    }

    // 提取的左右边线，是从上至下的
    void get_line(vector<POINT> &left, vector<POINT> &right)
    {
        vector<bool> leftVis(ROWSIMAGE + 1, false);
        vector<bool> rightVis(ROWSIMAGE + 1, false);
        for (int i = left.size() - 1; i >= 0; i--)
        {
            if (left.size() >= 0 && leftVis[left[i].x] == false && left[i].x <= rowEnd && left[i].x >= 3)
            {
                leftVis[left[i].x] = true;
                pointsEdgeLeft.push_back(left[i]);
            }
            // else if (left.size() >= 0 && leftVis[left[i].x] == true && left[i].x <= rowEnd && left[i].x >= 3)
            // {
            //     left.erase(left.begin() + i);
            //     // i+=1;
            // }
        }
        for (int i = right.size() - 1; i >= 0; i--)
        {
            if (right.size() > 0 && rightVis[right[i].x] == false && right[i].x <= rowEnd && right[i].x >= 3)
            {
                rightVis[right[i].x] = true;
                pointsEdgeRight.push_back(right[i]);
            }
            // else if (right.size() > 0 && rightVis[right[i].x] == true && right[i].x <= rowEnd && right[i].x >= 3)
            // {
            //     right.erase(right.begin() + i);
            //     // i+=1;
            // }
        }
        // for(int i=0;i)

        int half = kernel / 2;
        int t;
        vector<POINT> leftEdge(pointsEdgeLeft.size(), POINT(0, 0));
        for (int i = 0; i < leftEdge.size(); i++)
        {
            for (int j = -half; j <= half; j++)
            {
                if (i + j < 0)
                    t = 0;
                else if (i + j >= leftEdge.size())
                    t = leftEdge.size() - 1;
                else
                    t = i + j;
                leftEdge[i].x += pointsEdgeLeft[t].x * (half + 1 - fabs(j));
                leftEdge[i].y += pointsEdgeLeft[t].y * (half + 1 - fabs(j));
            }
            leftEdge[i].x /= (2 * half + 2) * (half + 1) / 2;
            leftEdge[i].y /= (2 * half + 2) * (half + 1) / 2;
        }
        pointsEdgeLeft = leftEdge;
        left = leftEdge;
        vector<POINT> rightEdge(pointsEdgeRight.size(), POINT(0, 0));
        for (int i = 0; i < rightEdge.size(); i++)
        {
            for (int j = -half; j <= half; j++)
            {
                if (i + j < 0)
                    t = 0;
                else if (i + j >= rightEdge.size())
                    t = rightEdge.size() - 1;
                else
                    t = i + j;
                rightEdge[i].x += pointsEdgeRight[t].x * (half + 1 - fabs(j));
                rightEdge[i].y += pointsEdgeRight[t].y * (half + 1 - fabs(j));
            }
            rightEdge[i].x /= (2 * half + 2) * (half + 1) / 2;
            rightEdge[i].y /= (2 * half + 2) * (half + 1) / 2;
        }
        pointsEdgeRight = rightEdge;
        right = rightEdge;
        // // 等距采样，jian为采样间距
        // for (int i = 0; i < left.size(); i += jian)
        // {
        //     if (left[i].x > rowEnd || left[i].x < 3 || leftVis[left[i].x] == false)
        //         continue;
        //     leftVis[left[i].x] = true; // 标记已访问
        //     pointsEdgeLeft.push_back(left[i]);
        // }
        // for (int i = 0; i < right.size(); i += jian)
        // {
        //     if (right[i].x > rowEnd || right[i].x < 3 || rightVis[right[i].x] == false)
        //         continue;
        //     rightVis[right[i].x] = true; // 标记已访问
        //     pointsEdgeRight.push_back(right[i]);
        // }
    }

    // 三点夹角法检测拐点
    // 参数:
    //   edge: 输入点集（按顺序）
    //   corners: 输出角点集
    //   angleThresholdDeg: 夹角阈值（度），夹角小于该值即判为拐点
    void detectCornerByThreePointAngle(const vector<POINT> &edge, vector<POINT> &corners, int sample = 5, float angleThresholdDeg = 60.0f)
    {
        if (edge.size() < 2 * sample + 1)
            return;

        float thresholdRad = angleThresholdDeg * CV_PI / 180.0f;

        vector<float> angles(edge.size(), CV_PI); // 默认最大角度
        for (size_t i = sample; i < edge.size() - sample; ++i)
        {
            float ax = edge[i].x - edge[i - sample].x;
            float ay = edge[i].y - edge[i - sample].y;
            float bx = edge[i + sample].x - edge[i].x;
            float by = edge[i + sample].y - edge[i].y;

            float lenA = sqrt(ax * ax + ay * ay);
            float lenB = sqrt(bx * bx + by * by);

            if (lenA < 1e-5 || lenB < 1e-5)
                continue;

            float dot = ax * bx + ay * by;
            float cosTheta = dot / (lenA * lenB);
            cosTheta = std::max(-1.0f, std::min(1.0f, cosTheta)); // 防止浮点误差

            float angle = acos(cosTheta); // 弧度
            angles[i] = angle;
        }

        // 非极大值抑制：只保留局部最小的尖角
        for (size_t i = sample + 1; i < edge.size() - sample - 1; ++i)
        {
            if (angles[i] < thresholdRad &&
                angles[i] < angles[i - 1] &&
                angles[i] < angles[i + 1])
            {
                corners.push_back(edge[i]);
            }
        }
    }

    void add_supproad(vector<float> &angle, const vector<POINT> &edge)
    {
        if (angle.empty())
            return;
        int index = 0;
        for (int i = 0; i < angle.size(); i++)
        {
            if (angle[i] == 0)
                continue;
            index++;
            int pre = i - 1 <= 0 ? 0 : i - 1 >= angle.size() ? angle.size() - 1
                                                             : i - 1;
            int next = i + 1 >= angle.size() ? angle.size() - 1 : i + 1 <= 0 ? 0
                                                                             : i + 1;
            float conf = fabs(angle[i]) - (fabs(angle[pre]) + fabs(angle[next])) / 2;
            cout << "index=" << index << ",x:" << edge[i].x << ",y:" << edge[i].y << ",angle:" << angle[i] << ",conf=" << conf;
            // if (40. / 180. * PI < conf && conf < 140. / 180. * PI)
            if (conf > 0.35 && conf < 2.5)
            {
                if (edge[i].y > 5 && edge[i].y < COLSIMAGE - 5)
                {
                    spurroad.push_back(edge[i]);
                    cout << ",*****************";
                }
            }
            cout << endl;
        }
    }

    void resample_points(vector<POINT> &edge, int num, float dist)
    {
        int remain = 0, len = 0;
        vector<POINT> line;
        for (int i = 0; i < edge.size() - 1 && len < num; i++)
        {
            float x0 = edge[i].x;
            float y0 = edge[i].y;
            float dx = edge[i + 1].x - x0;
            float dy = edge[i + 1].y - y0;
            float dn = sqrt(dx * dx + dy * dy);
            dx /= dn;
            dy /= dn;
            while (remain < dn && line.size() < num)
            {
                x0 += dx * remain;
                y0 += dy * remain;
                line.push_back(POINT(x0, y0));
                // line[len].x = x0;
                // line[len].y = y0;
                len++;
                dn -= remain;
                remain = dist;
            }
            remain -= dn;
        }
        // line.resize(len);
        edge = line;
    }

    void local_angle(vector<POINT> &edge, vector<float> &angle)
    {
        for (int i = 0; i < edge.size(); i++)
        {
            if (i <= 0 || i >= edge.size() - 1)
            {
                angle.push_back(0);
                continue;
            }
            float dx1 = edge[i].x - edge[i - 1].x;
            float dy1 = edge[i].y - edge[i - 1].y;
            float dn1 = sqrt(dx1 * dx1 + dy1 * dy1);
            float dx2 = edge[i + 1].x - edge[i].x;
            float dy2 = edge[i + 1].y - edge[i].y;
            float dn2 = sqrt(dx2 * dx2 + dy2 * dy2);
            float c1 = dx1 / dn1;
            float s1 = dy1 / dn1;
            float c2 = dx2 / dn2;
            float s2 = dy2 / dn2;
            angle.push_back(atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1));
        }
    }

    void nms_angle(vector<float> &angle, int window)
    {
        vector<float> nms_angle(angle.size(), 0);
        for (int i = 0; i < angle.size(); i++)
        {
            nms_angle[i] = angle[i]; // 初始化为原始角度
            for (int j = -1 * window; j <= window; j++)
            {
                // if(j==0)
                //     continue; // 跳过当前点
                // int num = i + j < 0 ? 0 : i + j >= angle.size() ? angle.size() - 1
                //                                                  : i + j;
                int num = i + j;
                if (i + j < 0 || i + j >= angle.size())
                    continue; // 跳过越界点
                if (fabs(angle[num]) > fabs(nms_angle[i]))
                {
                    nms_angle[i] = 0;
                    break;
                }
            }
        }
        angle = nms_angle;
    }

    void medianFilterPointsY(std::vector<POINT> &points, int windowSize = 5)
    {
        if (points.size() < windowSize)
            return;

        std::vector<__uint16_t> filteredY(points.size());
        int halfWindow = windowSize / 2;

        for (size_t i = 0; i < points.size(); ++i)
        {
            std::vector<__uint16_t> windowValues;

            // 取窗口内y值，窗口边界时用边界点补齐
            for (int w = -halfWindow; w <= halfWindow; ++w)
            {
                int idx = static_cast<int>(i) + w;
                if (idx < 0)
                    idx = 0;
                else if (idx >= (int)points.size())
                    idx = points.size() - 1;

                windowValues.push_back(points[idx].y);
            }

            // 取中值
            std::nth_element(windowValues.begin(), windowValues.begin() + windowValues.size() / 2, windowValues.end());
            filteredY[i] = windowValues[windowValues.size() / 2];
        }

        // 赋值回去
        for (size_t i = 0; i < points.size(); ++i)
        {
            points[i].y = filteredY[i];
        }
    }

    void detectCornerByVarianceLight(const vector<POINT> &edge, vector<POINT> &corners, int window = 2,
                                     double dirVarThreshold = 0.5, double slopeVarThreshold = 0.3)
    {

        int half = window / 2;
        int n = (int)edge.size();
        if (n < window)
            return;

        for (int i = half; i < n - half; ++i)
        {
            vector<int> dirVals;
            vector<float> slopeVals;
            for (int j = i - half; j <= i + half; ++j)
            {
                dirVals.push_back(edge[j].direction);
                slopeVals.push_back(edge[j].slope);
            }

            // 简化方向差平方均值（用离散方向差）
            double dirMean = 0;
            for (auto d : dirVals)
                dirMean += d;
            dirMean /= dirVals.size();

            double dirVar = 0;
            for (auto d : dirVals)
            {
                int diff = abs(d - (int)dirMean);
                if (diff > 4)
                    diff = 8 - diff; // 环绕修正
                dirVar += diff * diff;
            }
            dirVar /= dirVals.size();

            // 斜率方差
            double slopeMean = 0;
            for (auto s : slopeVals)
                slopeMean += s;
            slopeMean /= slopeVals.size();

            double slopeVar = 0;
            for (auto s : slopeVals)
            {
                double diff = s - slopeMean;
                slopeVar += diff * diff;
            }
            slopeVar /= slopeVals.size();

            if (dirVar > dirVarThreshold && slopeVar > slopeVarThreshold)
                corners.push_back(edge[i]);
        }
    }

    void detectGuaiDianCombined(const vector<POINT> &edge)
    {
        const int dirThreshold = 2;
        const float slopeDiffThreshold = 2;

        for (size_t i = min(edge.size() - 5, edge.size()); i > 5; i--)
        {
            if (edge[i].y < 3 || edge[i].y > COLSIMAGE - 3)
                continue;
            int dir_now = edge[i].direction;
            int dir_prev = edge[i - 1].direction;
            int dir_diff = abs(dir_now - dir_prev);
            if (dir_diff > 4)
                dir_diff = 8 - dir_diff;

            float slope_now = edge[i].slope;
            float slope_prev = edge[i - 1].slope;
            float slope_diff = abs(slope_now - slope_prev);

            if (dir_diff >= dirThreshold && slope_diff >= slopeDiffThreshold)
            {
                spurroad.push_back(edge[i]);
            }
        }
    }

    // void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel)
    // {
    //     assert(kernel % 2 == 1);

    //     int half = kernel / 2;
    //     for (int i = 0; i < num; i++)
    //     {
    //         pts_out[i][0] = pts_out[i][1] = 0;
    //         for (int j = -half; j <= half; j++)
    //         {
    //             pts_out[i][0] += pts_in[clip(i + j, 0, num - 1)][0] * (half + 1 - abs(j));
    //             pts_out[i][1] += pts_in[clip(i + j, 0, num - 1)][1] * (half + 1 - abs(j));
    //         }
    //         pts_out[i][0] /= (2 * half + 2) * (half + 1) / 2;
    //         pts_out[i][1] /= (2 * half + 2) * (half + 1) / 2;
    //     }
    // }

    // void get_line(const vector<POINT> &left, const vector<POINT> &right)
    // {

    //     if (left.empty() || right.empty())
    //     {
    //         cout << "边缘点为空，无法获取赛道线" << endl;
    //         return;
    //     }
    //     vector<int> leftVis(ROWSIMAGE + 1, -1);
    //     vector<int> rightVis(ROWSIMAGE + 1, -1);
    //     for (int i = 0; i < left.size() - 1; i++)
    //     {
    //         if (left[i].x > rowEnd)
    //             continue; // 忽略超过底部的点
    //         if (leftVis[left[i].x] == -1)
    //         {
    //             leftVis[left[i].x] = pointsEdgeLeft.size();
    //             pointsEdgeLeft.push_back(POINT(left[i]));
    //         }
    //         else
    //         {
    //             // leftVis[left[i].x] = pointsEdgeLeft.size();
    //             pointsEdgeLeft[leftVis[left[i].x]].x = left[i].x;
    //             pointsEdgeLeft[leftVis[left[i].x]].y = left[i].y;
    //             pointsEdgeLeft[leftVis[left[i].x]].direction = left[i].direction;
    //         }
    //     }
    //     for (int i = 0; i < right.size() - 1; i++)
    //     {
    //         if (right[i].x > rowEnd)
    //             continue; // 忽略超过底部的点
    //         if (rightVis[right[i].x] == -1)
    //         {
    //             rightVis[right[i].x] = pointsEdgeRight.size();
    //             pointsEdgeRight.push_back(POINT(right[i]));
    //         }
    //         else
    //         {
    //             // rightVis[right[i].x] = pointsEdgeRight.size();
    //             pointsEdgeRight[rightVis[right[i].x]].x = right[i].x;
    //             pointsEdgeRight[rightVis[right[i].x]].y = right[i].y;
    //             pointsEdgeRight[rightVis[right[i].x]].direction = right[i].direction;
    //         }
    //     }
    //     for (int i = 0; i < min(pointsEdgeLeft.size(), pointsEdgeRight.size()); i++)
    //     {
    //         if (pointsEdgeLeft[i].y != 0)
    //             validRowsLeft++;
    //         else
    //             diuLeft++;
    //         if (pointsEdgeRight[i].y != COLSIMAGE - 1)
    //             validRowsRight++;
    //         else
    //             diuRight++;
    //         if (pointsEdgeLeft[i].y > MaxLeftY)
    //             MaxLeftY = pointsEdgeLeft[i].y;
    //         if (pointsEdgeRight[i].y < MinRightY)
    //             MinRightY = pointsEdgeRight[i].y;
    //         widthBlock.push_back(POINT(pointsEdgeLeft[i].x, pointsEdgeRight[i].y - pointsEdgeLeft[i].y));
    //     }
    // }
    void zhong_line()
    {
        int left_index = 0, right_index = 0;
        while (left_index < pointsEdgeLeft.size() && right_index < pointsEdgeRight.size())
        {
            pointsEdgeZhong.push_back(POINT(pointsEdgeLeft[left_index].x, (pointsEdgeLeft[left_index].y + pointsEdgeRight[right_index].y) / 2));
            left_index++;
            right_index++;
        }
    }

    void img_print()
    {
        ofstream file("./first.xls");
        if (!file.is_open())
        {
            cout << "open file error!" << endl;
            return;
        }

        for (int i = 0; i < ROWSIMAGE; i++)
        {
            for (int j = 0; j < COLSIMAGE; j++)
                file << (int)imagePath.at<uchar>(i, j) << '\t';
            file << endl;
        }
        file.close();
    }

    void slopeCal(vector<POINT> &edge, int index)
    {
        if (index <= 4)
        {
            return;
        }
        float temp_slop1 = 0.0, temp_slop2 = 0.0;
        if (edge[index].x - edge[index - 2].x != 0)
        {
            temp_slop1 = (float)(edge[index].y - edge[index - 2].y) * 1.0f /
                         ((edge[index].x - edge[index - 2].x) * 1.0f);
        }
        else
        {
            temp_slop1 = edge[index].y > edge[index - 2].y ? 255 : -255;
        }
        if (edge[index].x - edge[index - 4].x != 0)
        {
            temp_slop2 = (float)(edge[index].y - edge[index - 4].y) * 1.0f /
                         ((edge[index].x - edge[index - 4].x) * 1.0f);
        }
        else
        {
            edge[index].slope = edge[index].y > edge[index - 4].y ? 255 : -255;
        }
        if (abs(temp_slop1) != 255 && abs(temp_slop2) != 255)
        {
            edge[index].slope = (temp_slop1 + temp_slop2) * 1.0 / 2;
        }
        else if (abs(temp_slop1) != 255)
        {
            edge[index].slope = temp_slop1;
        }
        else
        {
            edge[index].slope = temp_slop2;
        }
    }

    // double stdevEdgeCal(vector<POINT> &v_edge, int img_height)
    // {
    //     if (v_edge.size() < static_cast<size_t>(img_height / 4))
    //     {
    //         return 1000;
    //     }
    //     vector<int> v_slope;
    //     int step = 10; // v_edge.size()/10;
    //     for (size_t i = step; i < v_edge.size(); i += step)
    //     {
    //         if (v_edge[i].x - v_edge[i - step].x)
    //             v_slope.push_back((v_edge[i].y - v_edge[i - step].y) * 100 /
    //                               (v_edge[i].x - v_edge[i - step].x));
    //     }
    //     if (v_slope.size() > 1)
    //     {
    //         double sum = accumulate(begin(v_slope), end(v_slope), 0.0);
    //         double mean = sum / v_slope.size(); // 均值
    //         double accum = 0.0;
    //         for_each(begin(v_slope), end(v_slope),
    //                  [&](const double d)
    //                  { accum += (d - mean) * (d - mean); });

    //         return sqrt(accum / (v_slope.size() - 1)); // 方差
    //     }
    //     else
    //         return 0;
    // }
    double stdevEdgeCal(vector<POINT> &v_edge, int img_height)
    {
        if (v_edge.size() < static_cast<size_t>(img_height / 4))
            return 1000;

        vector<double> v_slope;
        int step = max(3, static_cast<int>(v_edge.size() / 10));
        for (size_t i = step; i < v_edge.size(); i += step)
        {
            int x1 = v_edge[i - step].x;
            int y1 = v_edge[i - step].y;
            int x2 = v_edge[i].x;
            int y2 = v_edge[i].y;

            // 排除上下边界5行以内的点
            if (x1 <= 5 || x1 >= img_height - 5)
                continue;
            if (x2 <= 5 || x2 >= img_height - 5)
                continue;

            int dx = x2 - x1;
            int dy = y2 - y1;
            if (dx == 0)
                continue;

            v_slope.push_back(static_cast<double>(dy) / dx);
        }

        if (v_slope.size() > 1)
        {
            double mean = accumulate(v_slope.begin(), v_slope.end(), 0.0) / v_slope.size();
            double accum = 0.0;
            for (auto slope : v_slope)
                accum += (slope - mean) * (slope - mean);
            return sqrt(accum / (v_slope.size() - 1));
        }
        else
            return 0;
    }

    void trackRecognition(Mat &imageBinary)
    {
        imagePath = imageBinary;
        if (save)
            savePicture(imageBinary, false);
        trackRecognition(false, 0);
        // track(imagePath, img, true); // 赛道线识别
    }

    /**
     * @brief 显示赛道线识别结果
     *
     * @param trackImage 需要叠加显示的图像
     */
    void drawImage(Mat &trackImage)
    {
        // Point（列号，行号）
        // for (size_t i = 0; i < pointsEdgeLeft.size(); i++)
        // {
        //     circle(trackImage, Point(pointsEdgeLeft[i].y, pointsEdgeLeft[i].x), 1,
        //            Scalar(0, 255, 0), -1); // 绿色点
        // }
        // for (size_t i = 0; i < pointsEdgeRight.size(); i++)
        // {
        //     circle(trackImage, Point(pointsEdgeRight[i].y, pointsEdgeRight[i].x), 1,
        //            Scalar(0, 255, 255), -1); // 黄色点
        // }
        for (size_t i = 0; i < pointsLeft.size(); i++)
        {
            circle(trackImage, Point(pointsLeft[i].y, pointsLeft[i].x), 1,
                   Scalar(0, 255, 0), -1); // 绿色点
        }
        for (size_t i = 0; i < pointsRight.size(); i++)
        {
            circle(trackImage, Point(pointsRight[i].y, pointsRight[i].x), 1,
                   Scalar(0, 255, 255), -1); // 黄色点
        }
        for (size_t i = 0; i < pointsEdgeZhong.size(); i++)
        {
            circle(trackImage, Point(pointsEdgeZhong[i].y, pointsEdgeZhong[i].x), 1,
                   Scalar(255, 0, 0), -1); // 蓝色点
        }
        // for (size_t i = 0; i < pointsLGuai.size(); i++)
        // {
        //     circle(trackImage, Point(pointsLGuai[i].y, pointsLGuai[i].x), 5,
        //            Scalar(255, 0, 255), -1); // 紫色点
        // }
        // for (size_t i = 0; i < pointsRGuai.size(); i++)
        // {
        //     circle(trackImage, Point(pointsRGuai[i].y, pointsRGuai[i].x), 5,
        //            Scalar(255, 255, 0), -1); // 青色点
        // }
        for (size_t i = 0; i < left_angle.size(); i++)
        {
            if (left_angle[i] != 0)
            {
                circle(trackImage, Point(pointsLeft[i].y, pointsLeft[i].x), 1,
                       Scalar(255, 0, 255), -1); // 绿色点
            }
        }
        for (size_t i = 0; i < right_angle.size(); i++)
        {
            if (right_angle[i] != 0)
            {
                circle(trackImage, Point(pointsRight[i].y, pointsRight[i].x), 1,
                       Scalar(255, 0, 255), -1); // 黄色点
            }
        }
        for (size_t i = 0; i < spurroad.size(); i++)
        {
            circle(trackImage, Point(spurroad[i].y, spurroad[i].x), 4,
                   Scalar(255, 255, 0), -1); // 紫色点
        }
        if (save)
            savePicture(trackImage, true); // 保存图像
    }
    void savePicture(Mat &image, bool flag)
    {
        // 存图
        string name = ".jpg";
        static int counter = 0;
        counter++;
        string imgPath = "./";
        if (flag)
            imgPath += "cai";
        imgPath += "tu/";
        name = imgPath + to_string(counter) + ".jpg";
        imwrite(name, image);
    }
};