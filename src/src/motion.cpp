/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo;
 *https://bjsstech.com 版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial
 *transactions(开源学习,请勿商用). The code ADAPTS the corresponding hardware
 *circuit board(代码适配百度Edgeboard-智能汽车赛事版), The specific details
 *consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file motion.cpp
 * @author Leo
 * @brief 运动控制器：PD姿态控制||速度控制
 * @version 0.1
 * @date 2023-12-26
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include "../include/common.hpp"
#include "../include/json.hpp"
#include "controlcenter.cpp"
#include "recognition/tracking.cpp"
#include <cmath>
#include <fstream>
#include <iostream>

using namespace std;
using namespace cv;

/**
 * @brief 运动控制器
 *
 */
class Motion

{
private:
  int countShift = 0; // 变速计数器

public:
  /**
   * @brief 初始化：加载配置文件
   *
   */
  Motion()
  {
    string jsonPath = "../src/config/config.json";
    std::ifstream config_is(jsonPath);
    if (!config_is.good())
    {
      std::cout << "Error: Params file path:[" << jsonPath << "] not find .\n";
      exit(-1);
    }

    nlohmann::json js_value;
    config_is >> js_value;

    try
    {
      params = js_value.get<Params>();
    }
    catch (const nlohmann::detail::exception &e)
    {
      std::cerr << "Json Params Parse failed :" << e.what() << '\n';
      exit(-1);
    }

    speed = params.speedLow;
    cout << "--- runP1Left:" << params.runP1Left << " | runP2Left:" << params.runP2Left
         << " | runP1Right:" << params.runP1Right << " | runP2Right:" << params.runP2Right << endl;
    cout << "--- turnP:" << params.turnP << " | turnD:" << params.turnD << endl;
    cout << "--- speedLow:" << params.speedLow
         << "m/s  |  speedHigh:" << params.speedHigh << "m/s" << endl;
  };

  /**
   * @brief 控制器核心参数
   *
   */
  struct Params
  {
    float speedLow = 0.8;      // 智能车最低速
    float speedHigh = 0.8;     // 智能车最高速
    float speedBridge = 0.6;   // 坡道速度
    float speedCatering = 0.6; // 快餐店速度
    float speedLayby = 0.6;    // 临时停车区速度
    float speedObstacle = 0.6; // 障碍区速度
    float speedParking = 0.6;  // 停车场速度
    float speedRing = 0.6;     // 环岛速度
    float speedDown = 0.5;     // 特殊区域降速速度
    float runP1 = 0.9;         // 一阶比例系数：直线控制量
    float runP2 = 0.018;       // 二阶比例系数：弯道控制量
    float runP3 = 0.0;         // 三阶比例系数：弯道控制量
    float turnP = 3.5;         // 一阶比例系数：转弯控制量
    float turnD = 3.5;         // 一阶微分系数：转弯控制量
    float runP1Left = 0.3;
    float runP2Left = 0.002;
    float runP1Right = 1.8;
    float runP2Right = 0.008;
    bool debug = false;         // 调试模式使能
    bool saveImg = false;       // 存图使能
    uint16_t rowCutUp = 10;     // 图像顶部切行
    uint16_t rowCutBottom = 10; // 图像顶部切行
    bool bridge = true;         // 坡道区使能
    bool catering = true;       // 快餐店使能
    bool layby = true;          // 临时停车区使能
    bool obstacle = true;       // 障碍区使能
    bool parking = true;        // 停车场使能
    bool ring = true;           // 环岛使能
    bool cross = true;          // 十字道路使能
    bool stop = true;           // 停车区使能

    float score = 0.5;                                 // AI检测置信度
    string model = "../res/model/yolov3_mobilenet_v1"; // 模型路径
    string video = "../res/samples/demo.mp4";          // 视频路径
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(Params, speedLow, speedHigh, speedBridge,
                                   speedCatering, speedLayby, speedObstacle,
                                   speedParking, speedRing, speedDown, runP1, runP2, runP3,
                                   turnP, turnD, runP1Left, runP2Left, runP1Right, runP2Right, debug, saveImg, rowCutUp,
                                   rowCutBottom, bridge, catering, layby, obstacle,
                                   parking, ring, cross, stop, score, model,
                                   video); // 添加构造函数
  };

  Params params;                   // 读取控制参数
  uint16_t servoPwm = PWMSERVOMID; // 发送给舵机的PWM
  float speed = 0.3;               // 发送给电机的速度
  Scene scene = Scene::NormalScene;
  float error;
  /**
   * @brief 姿态PD控制器
   *
   * @param controlCenter 智能车控制中心
   */
  void poseCtrl(int controlCenter, Scene scene, PredictResult &predict)
  {

    cout << "进入poseCtrl函数，当前场景: " << getScene(scene) << ",predict.type = " << predict.type << endl;
    cout.flush();

    error = controlCenter - COLSIMAGE / 2; // 图像控制中心转换偏差
    static int errorLast = 0;

    if (scene == Scene::ObstacleScene && predict.type == LABEL_BLOCK)
    {
      cout << "检测到黑块" << endl;
      cout << "obs_error1:" << error << endl;

      if (abs(error - errorLast) > COLSIMAGE / 10)
      {
        error = error > errorLast ? errorLast + COLSIMAGE / 10
                                  : errorLast - COLSIMAGE / 10;
      }
      cout << "obs_error2:" << error << endl;

      if (error > 0)
      { // 右转时
        params.turnP = abs(error) * params.runP2Right + params.runP1Right;
      }
      else
      { // 左转时
        params.turnP = abs(error) * params.runP2Left + params.runP1Left;
      }
      int pwmDiff = (error * params.turnP) + (error - errorLast) * params.turnD;
      float pwmDiff1 = 0.9 * pwmDiff;
      errorLast = error;
      cout << "pwmDiff1" << pwmDiff1 << endl;

      if (error > 0 && error <= 60)
        servoPwm = (uint16_t)(1740 - pwmDiff1);
      else if (error > 60)
        servoPwm = (uint16_t)(PWMSERVOMID - pwmDiff1 - 150); // PWM转换
      else if (error < 0 && error > -50)                     //
        servoPwm = (uint16_t)(PWMSERVOMID - pwmDiff1);
      else if (error <= -50) // error<-12
        servoPwm = (uint16_t)(PWMSERVOMID - pwmDiff1 + 370);
      else
        servoPwm = PWMSERVOMID;
    }

    else if (scene == Scene::ObstacleScene && predict.type == LABEL_CONE)
    {

      cout << "检测到锥桶" << endl;
      cout << "obs_error1:" << error << endl;

      if (abs(error - errorLast) > COLSIMAGE / 10)
      {
        error = error > errorLast ? errorLast + COLSIMAGE / 10
                                  : errorLast - COLSIMAGE / 10;
      }
      cout << "obs_error2:" << error << endl;

      if (error > 0)
      { // 右转时
        params.turnP = abs(error) * params.runP2Right + params.runP1Right;
      }
      else
      { // 左转时
        params.turnP = abs(error) * params.runP2Left + params.runP1Left;
      }
      int pwmDiff = (error * params.turnP) + (error - errorLast) * params.turnD;
      float pwmDiff1 = 0.9 * pwmDiff;
      errorLast = error;
      cout << "pwmDiff1" << pwmDiff1 << endl;

      if (error > 0 && error < 20)
        servoPwm = (uint16_t)(1740 - pwmDiff1);
      else if (error >= 20)
        servoPwm = (uint16_t)(PWMSERVOMID - pwmDiff1 - 150); // PWM转换
      else if (error < 0 && error > -20)                     // error<-12
        servoPwm = (uint16_t)(PWMSERVOMID - pwmDiff1);
      else if (error <= -20) // error<-12
        servoPwm = (uint16_t)(PWMSERVOMID - pwmDiff1 + 370);
      else
        servoPwm = PWMSERVOMID;
    }

    else if (scene == Scene::ObstacleScene && predict.type == LABEL_PEDESTRIAN)
    {

      cout << "检测到行人" << endl;
      cout << "obs_error1:" << error << endl;

      if (abs(error - errorLast) > COLSIMAGE / 10)
      {
        error = error > errorLast ? errorLast + COLSIMAGE / 10
                                  : errorLast - COLSIMAGE / 10;
      }
      cout << "obs_error2:" << error << endl;

      if (error > 0)
      { // 右转时
        params.turnP = abs(error) * params.runP2Right + params.runP1Right;
      }
      else
      { // 左转时
        params.turnP = abs(error) * params.runP2Left + params.runP1Left;
      }
      int pwmDiff = (error * params.turnP) + (error - errorLast) * params.turnD;
      float pwmDiff1 = 0.9 * pwmDiff;
      errorLast = error;
      cout << "pwmDiff1" << pwmDiff1 << endl;

      if (error > 0 && error < 65)
        servoPwm = (uint16_t)(1740 - pwmDiff1);
      else if (error >= 65)
        servoPwm = (uint16_t)(PWMSERVOMID - pwmDiff1 - 150); // PWM转换
      else if (error < 0 && error > -68)                     // error<-12
        servoPwm = (uint16_t)(PWMSERVOMID - pwmDiff1);
      else if (error <= -68) // error<-12
        servoPwm = (uint16_t)(PWMSERVOMID - pwmDiff1 + 370);
      else
        servoPwm = PWMSERVOMID;
    }



    else if (scene == Scene::LaybyScene)
    {

      if (abs(error - errorLast) > COLSIMAGE / 10)
      {
        error = error > errorLast ? errorLast + COLSIMAGE / 10
                                  : errorLast - COLSIMAGE / 10;
      }
      cout << "obs_error:" << error << endl;

      if (error > 0)
      { // 右转时
        params.turnP = abs(error) * params.runP2Right + params.runP1Right;
      }
      else
      { // 左转时
        params.turnP = abs(error) * params.runP2Left + params.runP1Left;
      }
      int pwmDiff = (error * params.turnP) + (error - errorLast) * params.turnD;
      errorLast = error;
      if (error > 0 && error <= 30)
        servoPwm = (uint16_t)(1740 - 1.15*pwmDiff);
      else if (error > 30 && error <= 50)
        servoPwm = (uint16_t)(1740 -  pwmDiff-150);
      else if (error < 0 && error > -30) // error<-12
        servoPwm = (uint16_t)(PWMSERVOMID -0.9* pwmDiff);
      else if (error >= -50 && error <= -30)
        servoPwm = (uint16_t)(PWMSERVOMID - 0.9*pwmDiff+370);
      else if(error < -50||error>50)
        servoPwm = PWMSERVOMID;
    }


    else
    {

      cout << "controlCenter" << controlCenter << endl;
      cout << "error1:" << error << endl;
      if (abs(error - errorLast) > COLSIMAGE / 10)
      {
        error = error > errorLast ? errorLast + COLSIMAGE / 10
                                  : errorLast - COLSIMAGE / 10;
      }
      cout << "error2:" << error << endl;

      if (error > 0)
      { // 右转时
        params.turnP = abs(error) * params.runP2Right + params.runP1Right;
      }
      else
      { // 左转时
        params.turnP = abs(error) * params.runP2Left + params.runP1Left;
      }
      int pwmDiff = (error * params.turnP) + (error - errorLast) * params.turnD;
      errorLast = error;
      cout << "pwmDiff" << pwmDiff << endl;

      if (error > 0 && error <= 42)
        servoPwm = (uint16_t)(1740 - pwmDiff);
      else if (error > 42)
      {
        servoPwm = (uint16_t)(PWMSERVOMID - pwmDiff - 150); // PWM转换
      }

      else if (error < 0 && error > -25) // error<-12
        servoPwm = (uint16_t)(PWMSERVOMID - pwmDiff);
      else if (error >= -68 && error <= -25)
        servoPwm = (uint16_t)(PWMSERVOMID - pwmDiff+370);
      else if (error < -68) // error<-12
      {
        servoPwm = (uint16_t)(PWMSERVOMID - pwmDiff + 370);
      }
      else
        servoPwm = PWMSERVOMID;
      if (servoPwm > 2045 || controlCenter < 83)
        servoPwm = 2045;
      if (servoPwm > 2045 || controlCenter < 73)
          servoPwm = 1810;
      if (servoPwm < 1280 || controlCenter > 230)
        servoPwm = 1280;
      if (servoPwm < 1280 || controlCenter > 247)
       servoPwm = 1490;

    

      
    }
  }

  /**
   * @brief 变加速控制
   *
   * @param enable 加速使能
   * @param control
   */
  // void speedCtrl(bool enable, bool slowDown, ControlCenter control)
  // {
  //   // 控制率
  //   uint8_t controlLow = 0;   // 速度控制下限
  //   uint8_t controlMid = 5;   // 控制率
  //   uint8_t controlHigh = 10; // 速度控制上限

  //   if (slowDown)
  //   {
  //     countShift = controlLow;
  //     speed = params.speedDown;
  //   }
  //   else if (enable) // 加速使能
  //   {
  //     if (control.centerEdge.size() < 10)
  //     {
  //       speed = params.speedLow;
  //       countShift = controlLow;
  //       return;
  //     }
  //     if (control.centerEdge[control.centerEdge.size() - 1].x > ROWSIMAGE / 2)
  //     {
  //       speed = params.speedLow;
  //       countShift = controlLow;
  //       return;
  //     }
  //     if (abs(control.sigmaCenter) < 100.0)
  //     {
  //       countShift++;
  //       if (countShift > controlHigh)
  //         countShift = controlHigh;
  //     }
  //     else
  //     {
  //       countShift--;
  //       if (countShift < controlLow)
  //         countShift = controlLow;
  //     }

  //     if (countShift > controlMid)
  //       speed = params.speedHigh;
  //     else
  //       speed = params.speedLow;
  //   }
  //   else
  //   {
  //     countShift = controlLow;
  //     speed = params.speedLow;
  //   }
  // }
   void speedCtrl(bool enable, bool slowDown, ControlCenter control)
  {
    // 控制率 - 增大上限和调整中间阈值
    uint8_t controlLow = 0;   // 速度控制下限
    uint8_t controlMid = 3;   // 降低阈值，更快切换到高速
    uint8_t controlHigh = 15; // 提高上限，允许更大调整范围

    // 加速步长 - 增大每次调整的幅度
    uint8_t accelStep = 2; // 原为1，现在每次调整+2/-2

    // 加速步长 - 增大每次调整的幅度
    uint8_t accelStep = 2; // 原为1，现在每次调整+2/-2
    float value = abs(control.sigmaCenter)/50.0f;
    accelStep = static_cast<uint8_t>(value < 1 ? 1 : (value > 4 ? 4 : value));
    if (slowDown)
    {//紧急减速模式
      countShift = controlLow;
      speed = params.speedDown;
    }
    else if (enable) // 加速使能
    {//正常加速控制
      if (control.centerEdge.size() < 10)
      {
        speed = params.speedLow;
        countShift = controlLow;
        return;
      }

      if (control.centerEdge[control.centerEdge.size() - 1].x > ROWSIMAGE/2)
      {
        speed = params.speedLow;
        countShift = controlLow;
        return;
      }


      if (abs(control.sigmaCenter) < 100.0)
      {
        countShift += accelStep; // 加速时增加步长
        countShift += accelStep; // 加速时增加步长
        if (countShift > controlHigh)
          countShift = controlHigh;
      }
      else
      {
        countShift -= accelStep; // 减速时同样增加步长
        if (countShift < controlLow)
          countShift = controlLow;
      }

      if (countShift > controlMid)
        speed = params.speedHigh;
      else
        speed = params.speedLow;
    }
    else
    {
      countShift = controlLow;
      speed = params.speedLow;
    }
  }
};