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
#include <opencv2/imgproc/imgproc_c.h>
#define COLSIMAGE 320 // 图像的列数
#define ROWSIMAGE 240 // 图像的行数
#define WHITE 255
#define BLACK 0
using namespace std;
using namespace cv;

// typedef struct POINT
// {
//     __uint16_t x = 0, y = 0;
//     float slope = 0.0;
//     __uint16_t direction;
//     POINT() {};
//     POINT(int x, int y) : x(x), y(y) {};
//     POINT(int x, int y, int direction) : x(x), y(y), direction(direction) {};
//     bool operator<(const POINT &p) const
//     {
//         if (x == p.x)
//             return y < p.y;
//         return x < p.x;
//     }
// } POINT;

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

    void trackRecognition(bool isResearch, uint16_t rowStart)
    {
        if (!isResearch)
        {
            pointsEdgeLeft.clear();
            pointsEdgeRight.clear();
            pointsEdgeZhong.clear();
            // pointsLGuai.clear();
            // pointsRGuai.clear();
            spurroad.clear();
            validRowsLeft = 1000;
            validRowsRight = 1000;
            widthBlock.clear();
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

        vector<POINT> pointsLeft, pointsRight;

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
        medianFilterPointsY(pointsEdgeLeft, 5);
        medianFilterPointsY(pointsEdgeRight, 5);

        stdevLeft = stdevEdgeCal(pointsEdgeLeft, ROWSIMAGE); // 计算边缘方差
        stdevRight = stdevEdgeCal(pointsEdgeRight, ROWSIMAGE);

        validRowsCal(); // 有效行计算
        // zhong_line();
        for (size_t i = 0; i < pointsEdgeLeft.size(); ++i)
            slopeCal(pointsEdgeLeft, i);
        for (size_t i = 0; i < pointsEdgeRight.size(); ++i)
            slopeCal(pointsEdgeRight, i);

        detectGuaiDianCombined(pointsEdgeLeft);  // 左线
        detectGuaiDianCombined(pointsEdgeRight); // 右线
        // detectCornerByVarianceLight(pointsEdgeLeft, spurroad);
        // detectCornerByVarianceLight(pointsEdgeRight, spurroad);

        drawImage(imagePath);
        // cout << "pointsLeft.size()=" << pointsEdgeLeft.size() << endl;
        // cout << "pointsRight.size()=" << pointsEdgeRight.size() << endl;
        // cout << "LeftLine.size()=" << LeftLine.size() << endl;
        // cout << "RightLine.size()=" << RightLine.size() << endl;
        // cout << "pointsEdgeLeft.size()=" << pointsEdgeLeft.size() << endl;
        // cout << "pointsEdgeRight.size()=" << pointsEdgeRight.size() << endl;
        // cout << "pointsLGuai.size()=" << pointsLGuai.size() << endl;
        // cout << "pointsRGuai.size()=" << pointsRGuai.size() << endl;
        // cout << "spurroad.size()=" << spurroad.size() << endl;
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

    // void detectCornerByVarianceLight(const vector<POINT> &edge, vector<POINT> &corners, int window = 2,
    //                                  double dirVarThreshold = 0.5, double slopeVarThreshold = 0.3)
    // {

    //     int half = window / 2;
    //     int n = (int)edge.size();
    //     if (n < window)
    //         return;

    //     for (int i = half; i < n - half; ++i)
    //     {
    //         vector<int> dirVals;
    //         vector<float> slopeVals;
    //         for (int j = i - half; j <= i + half; ++j)
    //         {
    //             dirVals.push_back(edge[j].direction);
    //             slopeVals.push_back(edge[j].slope);
    //         }

    //         // 简化方向差平方均值（用离散方向差）
    //         double dirMean = 0;
    //         for (auto d : dirVals)
    //             dirMean += d;
    //         dirMean /= dirVals.size();

    //         double dirVar = 0;
    //         for (auto d : dirVals)
    //         {
    //             int diff = abs(d - (int)dirMean);
    //             if (diff > 4)
    //                 diff = 8 - diff; // 环绕修正
    //             dirVar += diff * diff;
    //         }
    //         dirVar /= dirVals.size();

    //         // 斜率方差
    //         double slopeMean = 0;
    //         for (auto s : slopeVals)
    //             slopeMean += s;
    //         slopeMean /= slopeVals.size();

    //         double slopeVar = 0;
    //         for (auto s : slopeVals)
    //         {
    //             double diff = s - slopeMean;
    //             slopeVar += diff * diff;
    //         }
    //         slopeVar /= slopeVals.size();

    //         if (dirVar > dirVarThreshold && slopeVar > slopeVarThreshold)
    //             corners.push_back(edge[i]);
    //     }
    // }

    void detectGuaiDianCombined(const vector<POINT> &edge)
    {
        const int dirThreshold = 2;
        const float slopeDiffThreshold = 2;

        for (size_t i = 5; i < edge.size(); ++i)
        {
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

    void get_line(const vector<POINT> &left, const vector<POINT> &right)
    {

        if (left.empty() || right.empty())
        {
            cout << "边缘点为空，无法获取赛道线" << endl;
            return;
        }
        vector<int> leftVis(ROWSIMAGE + 1, -1);
        vector<int> rightVis(ROWSIMAGE + 1, -1);
        for (int i = 0; i < left.size() - 1; i++)
        {
            if (left[i].x > rowEnd)
                continue; // 忽略超过底部的点
            if (leftVis[left[i].x] == -1)
            {
                leftVis[left[i].x] = pointsEdgeLeft.size();
                pointsEdgeLeft.push_back(POINT(left[i]));
            }
            else
            {
                // leftVis[left[i].x] = pointsEdgeLeft.size();
                pointsEdgeLeft[leftVis[left[i].x]].x = left[i].x;
                pointsEdgeLeft[leftVis[left[i].x]].y = left[i].y;
                pointsEdgeLeft[leftVis[left[i].x]].direction = left[i].direction;
            }
        }
        for (int i = 0; i < right.size() - 1; i++)
        {
            if (right[i].x > rowEnd)
                continue; // 忽略超过底部的点
            if (rightVis[right[i].x] == -1)
            {
                rightVis[right[i].x] = pointsEdgeRight.size();
                pointsEdgeRight.push_back(POINT(right[i]));
            }
            else
            {
                // rightVis[right[i].x] = pointsEdgeRight.size();
                pointsEdgeRight[rightVis[right[i].x]].x = right[i].x;
                pointsEdgeRight[rightVis[right[i].x]].y = right[i].y;
                pointsEdgeRight[rightVis[right[i].x]].direction = right[i].direction;
            }
        }
        for (int i = 0; i < min(pointsEdgeLeft.size(), pointsEdgeRight.size()); i++)
        {
            // if (pointsEdgeLeft[i].y != 0)
            //     validRowsLeft++;
            // else
            //     diuLeft++;
            // if (pointsEdgeRight[i].y != COLSIMAGE - 1)
            //     validRowsRight++;
            // else
            //     diuRight++;
            // if (pointsEdgeLeft[i].y > MaxLeftY)
            //     MaxLeftY = pointsEdgeLeft[i].y;
            // if (pointsEdgeRight[i].y < MinRightY)
            //     MinRightY = pointsEdgeRight[i].y;
            widthBlock.push_back(POINT(pointsEdgeLeft[i].x, pointsEdgeRight[i].y - pointsEdgeLeft[i].y));
        }
    }
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

    double stdevEdgeCal(vector<POINT> &v_edge, int img_height)
    {
        if (v_edge.size() < static_cast<size_t>(img_height / 4))
        {
            return 1000;
        }
        vector<int> v_slope;
        int step = 10; // v_edge.size()/10;
        for (size_t i = step; i < v_edge.size(); i += step)
        {
            if (v_edge[i].x - v_edge[i - step].x)
                v_slope.push_back((v_edge[i].y - v_edge[i - step].y) * 100 /
                                  (v_edge[i].x - v_edge[i - step].x));
        }
        if (v_slope.size() > 1)
        {
            double sum = accumulate(begin(v_slope), end(v_slope), 0.0);
            double mean = sum / v_slope.size(); // 均值
            double accum = 0.0;
            for_each(begin(v_slope), end(v_slope),
                     [&](const double d)
                     { accum += (d - mean) * (d - mean); });

            return sqrt(accum / (v_slope.size() - 1)); // 方差
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
    }

    /**
     * @brief 显示赛道线识别结果
     *
     * @param trackImage 需要叠加显示的图像
     */
    void drawImage(Mat &trackImage)
    {
        // Point（列号，行号）
        for (size_t i = 0; i < pointsEdgeLeft.size(); i++)
        {
            circle(trackImage, Point(pointsEdgeLeft[i].y, pointsEdgeLeft[i].x), 1,
                   Scalar(0, 255, 0), -1); // 绿色点
        }
        for (size_t i = 0; i < pointsEdgeRight.size(); i++)
        {
            circle(trackImage, Point(pointsEdgeRight[i].y, pointsEdgeRight[i].x), 1,
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
        for (size_t i = 0; i < spurroad.size(); i++)
        {
            circle(trackImage, Point(spurroad[i].y, spurroad[i].x), 5,
                   Scalar(255, 0, 255), -1); // 紫色点
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
    /**
     * @brief 边缘有效行计算：左/右
     *
     */
    void validRowsCal(void)
    {
        // 左边有效行
        validRowsLeft = 0;
        if (pointsEdgeLeft.size() > 1)
        {
            for (size_t i = pointsEdgeLeft.size() - 1; i >= 1; i--)
            {
                if (pointsEdgeLeft[i].y > 2 && pointsEdgeLeft[i - 1].y >= 2)
                {
                    validRowsLeft = i + 1;
                    break;
                }
                if (pointsEdgeLeft[i].y < 2 && pointsEdgeLeft[i - 1].y >= 2)
                {
                    validRowsLeft = i + 1;
                    break;
                }
            }
        }

        // 右边有效行
        validRowsRight = 0;
        if (pointsEdgeRight.size() > 1)
        {
            for (size_t i = pointsEdgeRight.size() - 1; i >= 1; i--)
            {
                if (pointsEdgeRight[i].y <= COLSIMAGE - 2 &&
                    pointsEdgeRight[i - 1].y <= COLSIMAGE - 2)
                {
                    validRowsRight = i + 1;
                    break;
                }
                if (pointsEdgeRight[i].y >= COLSIMAGE - 2 &&
                    pointsEdgeRight[i - 1].y < COLSIMAGE - 2)
                {
                    validRowsRight = i + 1;
                    break;
                }
            }
        }
    }

    /**
     * @brief 冒泡法求取集合中值
     *
     * @param vec 输入集合
     * @return int 中值
     */
    int getMiddleValue(vector<int> vec)
    {
        if (vec.size() < 1)
            return -1;
        if (vec.size() == 1)
            return vec[0];

        int len = vec.size();
        while (len > 0)
        {
            bool sort = true; // 是否进行排序操作标志
            for (int i = 0; i < len - 1; ++i)
            {
                if (vec[i] > vec[i + 1])
                {
                    swap(vec[i], vec[i + 1]);
                    sort = false;
                }
            }
            if (sort) // 排序完成
                break;

            --len;
        }

        return vec[(int)vec.size() / 2];
    }
};