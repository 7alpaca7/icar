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
 * @file icar.cpp
 * @author Leo
 * @brief 智能汽车-顶层框架（TOP）
 * @version 0.1
 * @date 2023-12-25
 * @copyright Copyright (c) 2024
 *
 */

#include "../include/common.hpp"     //公共类方法文件
#include "../include/detection.hpp"  //百度Paddle框架移动端部署
#include "../include/uart.hpp"       //串口通信驱动
#include "controlcenter.cpp"         //控制中心计算类
#include "detection/bridge.cpp"      //AI检测：坡道区
#include "detection/obstacle.cpp"    //AI检测：障碍区
#include "detection/catering.cpp"    //AI检测：餐饮区
#include "detection/layby.cpp"       //AI检测：临时停车区
#include "detection/parking.cpp"     //AI检测：充电停车场
#include "detection/crosswalk.cpp"   //AI检测：停车区
#include "motion.cpp"                //智能车运动控制类
#include "preprocess.cpp"            //图像预处理类
#include "recognition/crossroad.cpp" //十字道路识别与路径规划类
#include "recognition/ring.cpp"      //环岛道路识别与路径规划类
#include "recognition/tracking.cpp"  //赛道识别基础类
#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include <signal.h>
#include <unistd.h>

using namespace std;
using namespace cv;

void mouseCallback(int event, int x, int y, int flags, void *userdata);
Display display; // 初始化UI显示窗口

int main(int argc, char const *argv[])
{
  Preprocess preprocess;    // 图像预处理类
  Motion motion;            // 运动控制类
  Tracking tracking;        // 赛道识别类
  Crossroad crossroad;      // 十字道路识别类
  Ring ring;                // 环岛识别类
  Bridge bridge;            // 坡道区检测类
  Catering catering;        // 快餐店检测类
  Obstacle obstacle;        // 障碍区检测类
  Layby layby;              // 临时停车区检测类
  Parking parking;          // 充电停车场检测类
  StopArea stopArea;        // 停车区识别与路径规划类
  ControlCenter ctrlCenter; // 控制中心计算类
  VideoCapture capture;     // Opencv相机类
  int countInit = 0;        // 初始化计数器
  bool flag = false;

  // 目标检测类(AI模型文件)
  shared_ptr<Detection> detection = make_shared<Detection>(motion.params.model);
  detection->score = motion.params.score; // AI检测置信度

  // USB转串口初始化： /dev/ttyUSB0
  shared_ptr<Uart> uart = make_shared<Uart>("/dev/ttyUSB0"); // 初始化串口驱动
  int ret = uart->open();
  if (ret != 0)
  {
    printf("[Error] Uart Open failed!\n");
    return -1;
  }
  uart->startReceive(); // 启动数据接收子线程

  // USB摄像头初始化
  if (motion.params.debug)
    capture = VideoCapture(motion.params.video); // 打开本地视频
  else
    capture = VideoCapture("/dev/video0"); // 打开摄像头
  if (!capture.isOpened())
  {
    printf("can not open video device!!!\n");
    return 0;
  }
  capture.set(CAP_PROP_FRAME_WIDTH, COLSIMAGE);  // 设置图像分辨率
  capture.set(CAP_PROP_FRAME_HEIGHT, ROWSIMAGE); // 设置图像分辨率
  capture.set(CAP_PROP_FPS, 30);                 // 设置帧率

  if (motion.params.debug)
  {
    cout << "qidong" << endl;
    display.init(4); // 调试UI初始化
    cout << "chushihua" << endl;
    display.frameMax = capture.get(CAP_PROP_FRAME_COUNT) - 1;
    // createTrackbar("Frame", "ICAR", &display.index, display.frameMax, [](int, void *) {}); // 创建Opencv图像滑条控件
    // setMouseCallback("ICAR", mouseCallback);
  }
  cin>>flag;
  // 等待按键发车
  if (!motion.params.debug)
  {
    printf("--------------[等待按键发车!]-------------------\n");
    //uart->buzzerSound(uart->BUZZER_START);//提示音
    //cout << "llai" << endl;
    while (!uart->keypress)
      waitKey(200);
    cout << "fa" << endl;
    uart->carControl(0, PWMSERVOMID);
    while (ret < 10) // 延时3s
    {
      uart->carControl(0, PWMSERVOMID); // 通信控制车辆停止运动
      // cout<<motion.servoPwm<<endl;
      waitKey(150);
      ret++;
    }
    cout << "an" << endl;
    uart->keypress = false;
    uart->buzzerSound(uart->BUZZER_START); // 祖传提示音效
  }
  cout << "go" << endl;
  // 初始化参数
  Scene scene = Scene::NormalScene;     // 初始化场景：常规道路
  Scene sceneLast = Scene::NormalScene; // 记录上一次场景状态
  PredictResult predict;
  long preTime;
  Mat img;
  cout << motion.params.runP1Left << " " << motion.params.runP1Right << endl;
  while (1)
  {
    //[01] 视频源读取
    if (motion.params.debug) // 综合显示调试UI窗口
    {
      cout << "jin" << endl;
      if (display.indexLast == display.index) // 图像帧未更新
      {
        display.show();     // 显示综合绘图
        // usleep(300 * 1000); // us延迟
        waitKey(1); // ms延迟
        continue;
      }
      preTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
      capture.set(cv::CAP_PROP_POS_FRAMES, display.index); // 设置读取帧
      if (!capture.read(img))
        continue;
      display.indexLast = display.index;
    }
    else if (!capture.read(img))
      continue;

    if (motion.params.saveImg && !motion.params.debug) // 存储原始图像
      savePicture(img);
    // display.save = true;
    else if (motion.params.saveImg && motion.params.debug) // 存储调式图像
      display.save = true;
    // 无论是不是debug都存调试图
    // if (motion.params.saveImg)
    //   // display.save = true;
    //   savePicture(img);

    //[02] 图像预处理
    Mat imgCorrect = preprocess.correction(img);         // 图像矫正
    Mat imgBinary = preprocess.binaryzation(imgCorrect); // 图像二值化

    //[03] 启动AI推理
    detection->inference(imgCorrect);

    //[04] 赛道识别
    tracking.rowCutUp = motion.params.rowCutUp;         // 图像顶部切行（前瞻距离）
    tracking.rowCutBottom = motion.params.rowCutBottom; // 图像底部切行（盲区距离）
    tracking.trackRecognition(imgBinary);
    // cout << "start" << endl;
    if (motion.params.debug) // 综合显示调试UI窗口
    {
      Mat imgTrack = imgCorrect.clone();
      tracking.drawImage(imgTrack); // 图像绘制赛道识别结果
      display.setNewWindow(2, "Track", imgTrack);
    }

    //[05] 停车区检测
    if (motion.params.stop)
    {
      if (stopArea.process(detection->results))
      {
        scene = Scene::StopScene;
        if (stopArea.countExit > 20)
        {
          uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
          sleep(1);
          printf("-----> [parking] System Exit!!! <-----\n");
          exit(0); // 程序退出
        }
      }
    }

    //[06] 快餐店检测
    if ((scene == Scene::NormalScene || scene == Scene::CateringScene) &&
        motion.params.catering)
    {
      if (catering.process(tracking, imgBinary, detection->results)) // 传入二值化图像进行再处理
        scene = Scene::CateringScene;
      else
        scene = Scene::NormalScene;
    }

    //[07] 临时停车区检测
    if ((scene == Scene::NormalScene || scene == Scene::LaybyScene) &&
        motion.params.catering)
    {
      if (layby.process(tracking, imgBinary, detection->results)) // 传入二值化图像进行再处理
        scene = Scene::LaybyScene;
      else
        scene = Scene::NormalScene;
    }

    //[08] 充电停车场检测
    if ((scene == Scene::NormalScene || scene == Scene::ParkingScene) &&
        motion.params.parking)
    {
      if (parking.process(tracking, imgBinary, detection->results)) // 传入二值化图像进行再处理
        scene = Scene::ParkingScene;
      else
        scene = Scene::NormalScene;
    }

    //[09] 坡道区检测
    if ((scene == Scene::NormalScene || scene == Scene::BridgeScene) &&
        motion.params.bridge)
    {
      if (bridge.process(tracking, detection->results))
        scene = Scene::BridgeScene;
      else
        scene = Scene::NormalScene;
    }

    // [10] 障碍区检测
    if ((scene == Scene::NormalScene || scene == Scene::ObstacleScene) &&
        motion.params.obstacle)
    {
      if (obstacle.process(tracking, detection->results))
      {
        //uart->buzzerSound(uart->BUZZER_DING); // 祖传提示音效
        scene = Scene::ObstacleScene;
      }
      else
        scene = Scene::NormalScene;
    }

    //[11] 十字道路识别与路径规划
    if ((scene == Scene::NormalScene || scene == Scene::CrossScene) &&
        motion.params.cross)
    {
      if (crossroad.crossRecognition(tracking))
        scene = Scene::CrossScene;
      else
        scene = Scene::NormalScene;
    }

    //[12] 环岛识别与路径规划
    if ((scene == Scene::NormalScene || scene == Scene::RingScene) &&
        motion.params.ring && catering.noRing)
    {
      if (ring.process(tracking))
        scene = Scene::RingScene;
      else
        scene = Scene::NormalScene;
    }

    //[13] 车辆控制中心拟合
    ctrlCenter.fitting(tracking);

    if (scene != Scene::ParkingScene)
    {
      if (ctrlCenter.derailmentCheck(tracking)) // 车辆冲出赛道检测（保护车辆）
      {
        uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
        sleep(1);
        printf("-----> [rush out] System Exit!!! <-----\n");
        exit(0); // 程序退出
      }
    }

    //[14] 运动控制(速度+方向)
    if (!motion.params.debug && countInit > 30) // 非调试模式下
    {
      // 触发停车
      if ((catering.stopEnable && scene == Scene::CateringScene) || (layby.stopEnable && scene == Scene::LaybyScene) || (parking.step == parking.ParkStep::stop))
      {
        motion.speed = 0;
      }
      else if (scene == Scene::CateringScene)
        motion.speed = motion.params.speedCatering;
      else if (scene == Scene::LaybyScene)
        motion.speed = motion.params.speedLayby;
      else if (scene == Scene::ParkingScene && parking.step == parking.ParkStep::trackout) // 倒车出库
        motion.speed = -motion.params.speedDown;
      else if (scene == Scene::ParkingScene) // 减速
        motion.speed = motion.params.speedParking;
      else if (scene == Scene::BridgeScene) // 坡道速度
        motion.speed = motion.params.speedBridge;
      else if (scene == Scene::ObstacleScene)
     
        motion.speed = motion.params.speedObstacle;// 危险区速度

      else if (scene == Scene::RingScene) // 环岛速度
        motion.speed = motion.params.speedRing;
      else if (scene == Scene::StopScene)
        motion.speed = motion.params.speedDown;
      else
        motion.speedCtrl(true, false, ctrlCenter); // 车速控制
      /*********************************************************************************/
      detection->drawBox(imgCorrect); // 图像绘制AI结果
      //
      motion.poseCtrl(ctrlCenter.controlCenter,scene,predict); // 姿态控制（舵机）
      /*********************************************************************************/
      ctrlCenter.drawImage(tracking, imgCorrect); // 图像绘制路径计算结果（控制中心）
      //
      uart->carControl(motion.speed, motion.servoPwm); // 串口通信控制车辆
      /*********************************************************************************/
      if(flag){
        imshow("show", imgCorrect);
      waitKey(1);
      }
      printf("servo:%d,speed:%lf\n", motion.servoPwm, motion.speed);
      //
    }
    else
      countInit++;

    //[15] 综合显示调试UI窗口
    if (motion.params.debug)
    {
      // 帧率计算
      auto startTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
      printf(">> FrameTime: %ldms | %.2ffps \n", startTime - preTime, 1000.0 / (startTime - preTime));

      detection->drawBox(imgCorrect);             // 图像绘制AI结果
      ctrlCenter.drawImage(tracking, imgCorrect); // 图像绘制路径计算结果（控制中心）
      putText(imgCorrect, formatDoble2String(motion.speed, 1) + "m/s", Point(COLSIMAGE - 70, 80),
              FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 显示车速

      display.setNewWindow(1, "Binary", imgBinary);
      Mat imgRes = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像

      switch (scene)
      {
      case Scene::NormalScene:
        break;
      case Scene::CrossScene:                  // [ 十字区 ]
        // crossroad.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
        circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40, Scalar(40, 120, 250), -1);
        putText(imgCorrect, "+", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27), FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
        break;
      case Scene::RingScene:              // [ 环岛 ]
        // ring.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
        circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40, Scalar(40, 120, 250), -1);
        putText(imgCorrect, "H", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27), FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
        break;
      case Scene::CateringScene:              // [ 餐饮区 ]
        catering.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
        circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40, Scalar(40, 120, 250), -1);
        putText(imgCorrect, "C", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27), FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
        break;
      case Scene::LaybyScene:              // [ 临时停车区 ]
        layby.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
        circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40, Scalar(40, 120, 250), -1);
        putText(imgCorrect, "T", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27), FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
        break;
      case Scene::ParkingScene:              // [ 充电停车场 ]
        parking.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
        circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40, Scalar(40, 120, 250), -1);
        putText(imgCorrect, "P", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27), FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
        break;
      case Scene::BridgeScene:              // [ 坡道区 ]
        bridge.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
        circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40, Scalar(40, 120, 250), -1);
        putText(imgCorrect, "S", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27), FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
        break;
      case Scene::ObstacleScene:    //[ 障碍区 ]
        obstacle.drawImage(imgRes); // 图像绘制特殊赛道识别结果
        circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40, Scalar(40, 120, 250), -1);
        putText(imgCorrect, "X", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27), FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
        break;
      default: // 常规道路场景：无特殊路径规划
        break;
      }

      display.setNewWindow(3, getScene(scene), imgRes); // 图像绘制特殊场景识别结果
      display.setNewWindow(4, "Ctrl", imgCorrect);
      display.show(); // 显示综合绘图
      waitKey(1);
      cout << "1" << endl;
    }
    //[16] 状态复位
    if (sceneLast != scene)
    {
      if (scene == Scene::NormalScene)
        uart->buzzerSound(uart->BUZZER_DING); // 祖传提示音效
      else
        uart->buzzerSound(uart->BUZZER_OK); // 祖传提示音效
    }
    sceneLast = scene; // 记录当前状态
    if (scene == Scene::ObstacleScene)
      scene = Scene::NormalScene;
    else if (scene == Scene::CrossScene)
      scene = Scene::NormalScene;
    else if (scene == Scene::RingScene)
      scene = Scene::NormalScene;
    else if (scene == Scene::CateringScene)
      scene = Scene::NormalScene;
    else if (scene == Scene::LaybyScene)
      scene = Scene::NormalScene;
    else if (scene == Scene::ParkingScene)
      scene = Scene::NormalScene;
    else if (scene == Scene::StopScene)
      scene = Scene::NormalScene;

    //[17] 按键退出程序
    if (uart->keypress)
    {
      uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
      sleep(1);
      printf("-----> System Exit!!! <-----\n");
      exit(0); // 程序退出
    }
  }

  uart->close(); // 串口通信关闭
  capture.release();
  return 0;
}

/**
 * @brief 鼠标的事件回调函数
 *
 */
void mouseCallback(int event, int x, int y, int flags, void *userdata)
{
  double value;
  switch (event)
  {
  case EVENT_MOUSEWHEEL: // 鼠标滑球
  {
    value = getMouseWheelDelta(flags); // 获取滑球滚动值
    if (value > 0)
      display.index++;
    else if (value < 0)
      display.index--;

    if (display.index < 0)
      display.index = 0;
    if (display.index > display.frameMax)
      if (display.index > display.frameMax)
        display.index = display.frameMax;
    break;
  }
  default:
    break;
  }
}