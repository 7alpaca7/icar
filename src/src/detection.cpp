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
 * @file detection.cpp
 * @author HC
 * @brief 目标检测
 * @version 0.1
 * @date 2025-02-28
 * @copyright Copyright (c) 2024
 *
 */
#include "../include/common.hpp"     //公共类方法文件
#include "../include/detection.hpp"  //百度Paddle框架移动端部署
#include "motion.cpp"                //智能车运动控制类
#include "preprocess.cpp"            //图像预处理类
#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include <signal.h>
#include <unistd.h>

using namespace std;
using namespace cv;

int main(int argc, char const *argv[]) {
  Preprocess preprocess;    // 图像预处理类
  Motion motion;            // 运动控制类
  VideoCapture capture;     // Opencv相机类

  // 目标检测类(AI模型文件)
  shared_ptr<Detection> detection = make_shared<Detection>(motion.params.model);
  detection->score = motion.params.score; // AI检测置信度

  // USB摄像头初始化
  capture = VideoCapture("../res/samples/sample.mp4"); // 打开摄像头
  if (!capture.isOpened()) {
    printf("can not open video device!!!\n");
    return 0;
  }
  capture.set(CAP_PROP_FRAME_WIDTH, COLSIMAGE);  // 设置图像分辨率
  capture.set(CAP_PROP_FRAME_HEIGHT, ROWSIMAGE); // 设置图像分辨率
  capture.set(CAP_PROP_FPS, 30);                 // 设置帧率

  // 初始化参数
  Mat img;

  while (1) {
    //[01] 视频源读取
    if (!capture.read(img))
      continue;
    if (motion.params.saveImg && !motion.params.debug) // 存储原始图像
      savePicture(img);

    //[02] 图像预处理
    // Mat imgCorrect = preprocess.correction(img);         // 图像矫正

    //[03] 启动AI推理
    detection->inference(img);

    detection->drawBox(img); // 图像绘制AI结果
    imshow("detection", img);
    waitKey(10);    // 等待显示

  }

  capture.release();
  return 0;
}