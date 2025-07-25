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
 * @file common.hpp
 * @author HC
 * @brief 通用方法类
 * @version 0.1
 * @date 2025-02-28
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "json.hpp"
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署

using namespace std;
using namespace cv;

#define COLSIMAGE 320    // 图像的列数
#define ROWSIMAGE 240    // 图像的行数
#define COLSIMAGEIPM 320 // IPM图像的列数
#define ROWSIMAGEIPM 400 // IPM图像的行数
#define PWMSERVOMAX 2100 // 舵机PWM最大值（左）1840
#define PWMSERVOMID 0 // 舵机PWM中值 1520 error=0
#define PWMSERVOMIN 1200 // 舵机PWM最小值（右）1200

#define LABEL_BATTERY 0    // AI标签：充电站
#define LABEL_BLOCK 1      // AI标签：障碍物
#define LABEL_BRIDGE 2     // AI标签：坡道
#define LABEL_BURGER 3     // AI标签：汉堡
#define LABEL_CAR 4        // AI标签：道具车
#define LABEL_COMPANY 5    // AI标签：公司
#define LABEL_CONE 6       // AI标签：锥桶
#define LABEL_CROSSWALK 7  // AI标签：斑马线
#define LABEL_PEDESTRIAN 8 // AI标签：行人
#define LABEL_SCHOOL 9     // AI标签：学校

/**
 * @brief 场景类型（路况）
 *
 */
enum Scene
{
    NormalScene = 0, // 基础赛道
    CrossScene,      // 十字道路
    RingScene,       // 环岛道路
    BridgeScene,     // 坡道区
    ObstacleScene,   // 障碍区
    CateringScene,   // 快餐店
    LaybyScene,      // 临时停车区
    ParkingScene,    // 停车区
    StopScene        // 停车（结束）
};

/**
 * @brief Get the Scene object
 *
 * @param scene
 * @return string
 */
string getScene(Scene scene)
{
    switch (scene)
    {
    case Scene::NormalScene:
        return "Normal";
    case Scene::CrossScene:
        return "Crossroad";
    case Scene::RingScene:
        return "Ring";
    case Scene::BridgeScene:
        return "Bridge";
    case Scene::ObstacleScene:
        return "Obstacle";
    case Scene::CateringScene:
        return "Catering";
    case Scene::LaybyScene:
        return "Layby";
    case Scene::ParkingScene:
        return "Parking";
    case Scene::StopScene:
        return "Stop";
    default:
        return "Error";
    }
}

/**
 * @brief 构建二维坐标
 *
 */
typedef struct POINT
{
    int x = 0, y = 0;
    float slope = 0.0;
    int direction;
    POINT() {};
    POINT(int x, int y) : x(x), y(y) {};
    POINT(int x, int y, int direction) : x(x), y(y), direction(direction) {};
    bool operator<(const POINT &p) const
    {
        if (x == p.x)
            return y < p.y;
        return x < p.x;
    }
} POINT;

/**
 * @brief 存储图像至本地
 *
 * @param image 需要存储的图像
 */
void savePicture(Mat &image)
{
    // 存图
    string name = ".jpg";
    static int counter = 0;
    counter++;
    string imgPath = "../res/samples/train/";
    name = imgPath + to_string(counter) + ".jpg";
    imwrite(name, image);
}

//--------------------------------------------------[公共方法]----------------------------------------------------
/**
 * @brief int集合平均值计算
 *
 * @param arr 输入数据集合
 * @return double
 */
double average(vector<int> vec)
{
    if (vec.size() < 1)
        return -1;

    double sum = 0;
    for (size_t i = 0; i < vec.size(); i++)
    {
        sum += vec[i];
    }

    return (double)sum / vec.size();
}

/**
 * @brief int集合数据方差计算
 *
 * @param vec Int集合
 * @return double
 */
double sigma(vector<int> vec)
{
    if (vec.size() < 1)
        return 0;

    double aver = average(vec); // 集合平均值
    double sigma = 0;
    for (size_t i = 0; i < vec.size(); i++)
    {
        sigma += (vec[i] - aver) * (vec[i] - aver);
    }
    sigma /= (double)vec.size();
    return sigma;
}

/**
 * @brief 赛道点集的方差计算
 *
 * @param vec
 * @return double
 */
double sigma(vector<POINT> vec)
{
    if (vec.size() < 1)
        return 0;

    double sum = 0;
    for (size_t i = 0; i < vec.size(); i++)
    {
        sum += vec[i].y;
    }
    double aver = (double)sum / vec.size(); // 集合平均值

    double sigma = 0;
    for (size_t i = 0; i < vec.size(); i++)
    {
        sigma += (vec[i].y - aver) * (vec[i].y - aver);
    }
    sigma /= (double)vec.size();
    return sigma;
}

/**
 * @brief 阶乘计算
 *
 * @param x
 * @return int
 */
int factorial(int x)
{
    int f = 1;
    for (int i = 1; i <= x; i++)
    {
        f *= i;
    }
    return f;
}

/**
 * @brief 贝塞尔曲线
 *
 * @param dt
 * @param input
 * @return vector<POINT>
 */
vector<POINT> Bezier(double dt, vector<POINT> input)
{
    vector<POINT> output;

    double t = 0;
    while (t <= 1)
    {
        POINT p;
        double sumX = 0.0;
        double sumY = 0.0;
        int i = 0;
        int n = input.size() - 1;
        while (i <= n)
        {
            double k =
                factorial(n) / (factorial(i) * factorial(n - i)) * pow(t, i) * pow(1 - t, n - i);
            sumX += k * input[i].x;
            sumY += k * input[i].y;
            i++;
        }
        p.x = sumX;
        p.y = sumY;
        output.push_back(p);
        t += dt;
    }
    return output;
}

auto formatDoble2String(double val, int fixed)
{
    auto str = std::to_string(val);
    return str.substr(0, str.find(".") + fixed + 1);
}

/**
 * @brief 点到直线的距离计算
 *
 * @param a 直线的起点
 * @param b 直线的终点
 * @param p 目标点
 * @return double
 */
double distanceForPoint2Line(POINT a, POINT b, POINT p)
{
    double ab_distance =
        sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    double ap_distance =
        sqrt((a.x - p.x) * (a.x - p.x) + (a.y - p.y) * (a.y - p.y));
    double bp_distance =
        sqrt((p.x - b.x) * (p.x - b.x) + (p.y - b.y) * (p.y - b.y));

    double half = (ab_distance + ap_distance + bp_distance) / 2;
    double area = sqrt(half * (half - ab_distance) * (half - ap_distance) * (half - bp_distance));

    return (2 * area / ab_distance);
}

/**
 * @brief 两点之间的距离
 *
 * @param a
 * @param b
 * @return double
 */
double distanceForPoints(POINT a, POINT b)
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

/**
 * @brief UI综合图像绘制
 *
 */
class Display
{
private:
    bool enable = false;   // 显示窗口使能
    int sizeWindow = 1;    // 窗口数量
    cv::Mat imgShow;       // 窗口图像
    bool realShow = false; // 实时更新画面
public:
    int index = 0;      // 图像序号
    int indexLast = -1; // 图像序号
    int frameMax = 0;   // 视频总帧数
    bool save = false;  // 图像存储

    /**
     * @brief 显示窗口初始化
     *
     * @param size 窗口数量(1~7)
     */
    void init(const int size)
    {
        if (size <= 0 || size > 7)
            return;

        cv::namedWindow("ICAR", WINDOW_NORMAL);     // 图像名称
        cv::resizeWindow("ICAR", 480 * 2, 320 * 2); // 分辨率

        imgShow = cv::Mat::zeros(ROWSIMAGE * 2, COLSIMAGE * 2, CV_8UC3);
        enable = true;
        sizeWindow = size;
    }

    /**
     * @brief 设置新窗口属性
     *
     * @param index 窗口序号
     * @param name 窗口名称
     * @param img 显示图像
     */
    void setNewWindow(int index, string name, Mat img)
    {
        // 数据溢出保护
        if (!enable || index <= 0 || index > sizeWindow)
            return;

        if (img.cols <= 0 || img.rows <= 0)
            return;

        Mat imgDraw = img.clone();

        if (imgDraw.type() == CV_8UC1) // 非RGB类型的图像
            cvtColor(imgDraw, imgDraw, cv::COLOR_GRAY2BGR);

        // 图像缩放
        if (imgDraw.cols != COLSIMAGE || imgDraw.rows != ROWSIMAGE)
        {
            float fx = COLSIMAGE / imgDraw.cols;
            float fy = ROWSIMAGE / imgDraw.rows;
            if (fx <= fy)
                resize(imgDraw, imgDraw, Size(COLSIMAGE, ROWSIMAGE), fx, fx);
            else
                resize(imgDraw, imgDraw, Size(COLSIMAGE, ROWSIMAGE), fy, fy);
        }

        // 限制图片标题长度
        string text = "[" + to_string(index) + "] ";
        if (name.length() > 15)
            text = text + name.substr(0, 15);
        else
            text = text + name;

        putText(imgDraw, text, Point(10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 0, 0), 0.5);

        if (index <= 2)
        {
            Rect placeImg = Rect(COLSIMAGE * (index - 1), 0, COLSIMAGE, ROWSIMAGE);
            imgDraw.copyTo(imgShow(placeImg));
        }

        else
        {
            Rect placeImg = Rect(COLSIMAGE * (index - 3), ROWSIMAGE, COLSIMAGE, ROWSIMAGE);
            imgDraw.copyTo(imgShow(placeImg));
        }

        if (save)
            savePicture(img); // 保存图像
    }

    /**
     * @brief 融合后的图像显示
     *
     */
    void show(void)
    {
        if (enable)
        {
            putText(imgShow, "Frame:" + to_string(index), Point(COLSIMAGE / 2 - 50, ROWSIMAGE * 2 - 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);
            imshow("ICAR", imgShow);

            char key = waitKey(1);
            if (key != -1)
            {
                if (key == 32) // 空格
                    realShow = !realShow;
            }
            if (realShow)
            {
                index++;
                if (index < 0)
                    index = 0;
                if (index > frameMax)
                    index = frameMax;
            }
        }
    }
};