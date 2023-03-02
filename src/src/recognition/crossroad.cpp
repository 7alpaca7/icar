#pragma once
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "./tracking.cpp"
#include <iostream>
using namespace std;
using namespace cv;

class Crossroad
{
private:
    POINT LU, LD, RU, RD;                                    // 左上，左下，右上，右下
    bool LUF = false, LDF = false, RUF = false, RDF = false; // 四个点是否找到的标志
    int jian = 10;                                           // 找角点时，前后点用的间隔
    int width1 = 10, width2 = 30;                            // 宽度差阈值
    int height1 = 20;                                        // 高度差阈值
    int search = 30;                                         // 连续贴合图像边缘的搜索范围
    int square = 1;                                          // 贴边的阈值
    int yes = 3;                                             // 连续贴边的次数
    bool textDebug = false;                                   // 文字debug
    int diuYu = 20;                                          // 丢线阈值

public:
    /*
     * V2.0
     * 该版本更正了用widthblock的方式来判断角点的逻辑，改为用原始点集的方式判断。
     * @brief 十字路口识别与补线
     * @param track 赛道识别结果
     * @return bool 是否成功识别十字路口
     * 1.先检查边线数量够不够，不够图像一半就false
     * 2.然后找四个角点，角点的初步判断为当前点的后三行与当前行的widthblock差值绝对值超过一个阈值，前三行的widthblock差值绝对值相近
     * 3.如果找到一对角点，如左上左下角点，遍历左上左下就角点的widthblock是否连续为与图像宽度相近的阈值，能达成就两点补线
     * 4.如果找不到一对角点，假设右边只有右下角点，不补；假设右边找到右上角点，则遍历右下角点到边线起始点的widthblock是否连续为与图像宽度相近的阈值，能达成就两点补线
     * 5.判断十字必须要有上两角点，没有就false
     * 6.找到的一对角点，比如左上左下，需要比较一下y是否相近，在一个阈值内，不在的话左下就用起始点，同时x坐标也不能太近（如果用左下起始点的时候只需比较一下x坐标不能太近）。
     * 7.找到的左上右上角点也需要比较一下x是否相近，同时y坐标也不能太近。左下右下就没必要了。
     */
    bool crossRecognition(Tracking &track)
    {
        if (textDebug)
            cout << "开始十字识别" << endl;
        // 1.先判断一手点集数量够不够
        if (track.pointsEdgeLeft.size() < ROWSIMAGE / 2 || track.pointsEdgeRight.size() < ROWSIMAGE / 2)
        {
            if (textDebug)
                cout << "边线点不够" << endl;
            return false;
        }
        // 2.再来一手拐点是否够，左右打底一个以上
        if (track.LGnum < 1 || track.RGnum < 1)
        {
            if (textDebug)
                cout << "拐点不够" << endl;
            return false;
        }
        // 3.如果左右都没丢线，那还判什么十字
        if (track.diuLeft <= diuYu || track.diuRight <= diuYu)
        {
            if (textDebug)
                cout << "左右边线都没丢线，无法判断十字" << endl;
            return false;
        }
        // 4.先找上俩角点
        LUF = LDF = RUF = RDF = false; // 补线点标志
        // 左上
        if (textDebug)
            cout << "左上：" << endl;
        for (int i = 0; i < track.LGnum; i++)
        {
            int pre = track.spurroad[i].x - jian - track.LP, next = track.spurroad[i].x + jian - track.LP, here = track.spurroad[i].x - track.LP;
            // int dx = track.L_Start.x - track.spurroad[i].x;
            // printf("Check spurroad[%d] = (%d,%d), dx=%d\n", i, track.spurroad[i].x, track.spurroad[i].y, dx);
            if ((here < track.L_Start.x - track.LP && here + height1 > track.L_Start.x - track.LP) || (here > track.L_Start.x - track.LP))
                continue;
            /*这里防止下标爆的限制，有问题，待优化*/
            if (pre < 0)
                pre = 0;
            else if (pre >= track.pointsEdgeLeft.size())
                pre = track.pointsEdgeLeft.size() - 1;
            if (next < 0)
                next = 0;
            else if (next >= track.pointsEdgeLeft.size())
                next = track.pointsEdgeLeft.size() - 1;
            if (here < 0)
                here = 0;
            else if (here >= track.pointsEdgeLeft.size())
                here = track.pointsEdgeLeft.size() - 1;
            int ap = track.pointsEdgeLeft[pre].y, an = track.pointsEdgeLeft[next].y, ah = track.pointsEdgeLeft[here].y;
            if (ap < 1)
                ap = 1;
            else if (ap > COLSIMAGE - 2)
                ap = COLSIMAGE - 2;
            if (an < 1)
                an = 1;
            else if (an > COLSIMAGE - 2)
                an = COLSIMAGE - 2;
            if (ah < 1)
                ah = 1;
            else if (ah > COLSIMAGE - 2)
                ah = COLSIMAGE - 2;
            if (textDebug)
            {
                printf("ap=%d, an=%d, ah=%d\n", ap, an, ah);
                printf("pre=%d, next=%d, here=%d\n", pre, next, here);
                printf("fabs(ap - ah) = %f, fabs(an - ah) = %f\n", fabs(ap - ah), fabs(an - ah));
            }
            if (fabs(ap - ah) <= width1 && fabs(an - ah) >= width2)
            // if((ah - ap>=0 && ah - ap<=width1)&&(an-ah>0 &&))
            // if (fabs(ap - ah) < fabs(an - ah))
            // if (fabs(ap - ah) <= 150 && fabs(an - ah) >= 50 && an > ap)
            {
                LU = POINT(track.spurroad[i].x, track.spurroad[i].y);
                LUF = true;
                break;
            }
        }
        // 右上
        if (textDebug)
            cout << "右上" << endl;
        for (int i = track.LGnum; i < track.spurroad.size(); i++)
        {
            int pre = track.spurroad[i].x - jian - track.RP, next = track.spurroad[i].x + jian - track.RP, here = track.spurroad[i].x - track.RP;
            // int dx = track.R_Start.x - track.spurroad[i].x;
            // printf("Check spurroad[%d] = (%d,%d), dx=%d\n", i, track.spurroad[i].x, track.spurroad[i].y, dx);
            if ((here < track.R_Start.x - track.RP && here + height1 > track.R_Start.x - track.RP) || (here > track.R_Start.x - track.RP))
                continue;
            if (pre < 0)
                pre = 0;
            else if (pre >= track.pointsEdgeRight.size())
                pre = track.pointsEdgeRight.size() - 1;
            if (next < 0)
                next = 0;
            else if (next >= track.pointsEdgeRight.size())
                next = track.pointsEdgeRight.size() - 1;
            if (here < 0)
                here = 0;
            else if (here >= track.pointsEdgeRight.size())
                here = track.pointsEdgeRight.size() - 1;
            int ap = track.pointsEdgeRight[pre].y, an = track.pointsEdgeRight[next].y, ah = track.pointsEdgeRight[here].y;
            if (ap < 1)
                ap = 1;
            else if (ap > COLSIMAGE - 2)
                ap = COLSIMAGE - 2;
            if (an < 1)
                an = 1;
            else if (an > COLSIMAGE - 2)
                an = COLSIMAGE - 2;
            if (ah < 1)
                ah = 1;
            else if (ah > COLSIMAGE - 2)
                ah = COLSIMAGE - 2;
            if (textDebug)
            {
                printf("ap=%d, an=%d, ah=%d\n", ap, an, ah);
                printf("pre=%d, next=%d, here=%d\n", pre, next, here);
                printf("fabs(ap - ah) = %f, fabs(an - ah) = %f\n", fabs(ap - ah), fabs(an - ah));
            }
            if (fabs(ap - ah) <= width1 && fabs(an - ah) >= width2)
            // if (fabs(ap - ah) < fabs(an - ah))
            // if (fabs(ap - ah) <= 150 && fabs(an - ah) >= 50 && an > ap)
            {
                RU = POINT(track.spurroad[i].x, track.spurroad[i].y);
                RUF = true;
                break;
            }
        }
        // 上俩都没救没法判断十字
        if (!LUF || !RUF)
        {
            if (textDebug)
            {
                if (!LUF)
                    cout << "左上角点未找到，无法判断十字" << endl;
                if (!RUF)
                    cout << "右上角点未找到，无法判断十字" << endl;
            }
            return false;
        }
        if (textDebug)
        {
            cout << "左上和右上两都找到了" << endl;
            cout << "左上角点坐标为(" << LU.x << "," << LU.y << ")" << endl;
            cout << "右上角点坐标为(" << RU.x << "," << RU.y << ")" << endl;
        }
        // 5.象征性的找下下俩兄弟，没有就用起始点
        // 左下
        for (int i = track.LGnum - 1; i >= 0; i--)
        {
            int pre = track.spurroad[i].x - jian - track.LP, next = track.spurroad[i].x + jian - track.LP, here = track.spurroad[i].x - track.LP;
            if (here <= LU.x - track.LP)
                break;
            if (pre < 0)
                pre = 0;
            else if (pre >= track.pointsEdgeLeft.size())
                pre = track.pointsEdgeLeft.size() - 1;
            if (next < 0)
                next = 0;
            else if (next >= track.pointsEdgeLeft.size())
                next = track.pointsEdgeLeft.size() - 1;
            if (here < 0)
                here = 0;
            else if (here >= track.pointsEdgeLeft.size())
                here = track.pointsEdgeLeft.size() - 1;
            int ap = track.pointsEdgeLeft[pre].y, an = track.pointsEdgeLeft[next].y, ah = track.pointsEdgeLeft[here].y;
            if (ap < 1)
                ap = 1;
            else if (ap > COLSIMAGE - 2)
                ap = COLSIMAGE - 2;
            if (an < 1)
                an = 1;
            else if (an > COLSIMAGE - 2)
                an = COLSIMAGE - 2;
            if (ah < 1)
                ah = 1;
            else if (ah > COLSIMAGE - 2)
                ah = COLSIMAGE - 2;
            if (fabs(ap - ah) >= width2 && fabs(an - ah) <= width1)
            /*这里感觉也有点微妙，虽然后续有上下点丢线兜底*/
            // if (fabs(ap - ah) > fabs(an - ah))
            // if (fabs(an - ah) <= 150 && fabs(ap - ah) >= 50 && an < ap)
            {
                LD = POINT(track.spurroad[i].x, track.spurroad[i].y);
                LDF = true;
                break;
            }
        }
        // 右下
        for (int i = track.spurroad.size() - 1; i > track.LGnum - 1; i++)
        {
            int pre = track.spurroad[i].x - jian - track.RP, next = track.spurroad[i].x + jian - track.RP, here = track.spurroad[i].x - track.RP;
            if (here <= RU.x)
                break;
            if (pre < 0)
                pre = 0;
            else if (pre >= track.pointsEdgeRight.size())
                pre = track.pointsEdgeRight.size() - 1;
            if (next < 0)
                next = 0;
            else if (next >= track.pointsEdgeRight.size())
                next = track.pointsEdgeRight.size() - 1;
            if (here < 0)
                here = 0;
            else if (here >= track.pointsEdgeRight.size())
                here = track.pointsEdgeRight.size() - 1;
            int ap = track.pointsEdgeRight[pre].y, an = track.pointsEdgeRight[next].y, ah = track.pointsEdgeRight[here].y;
            if (ap < 1)
                ap = 1;
            else if (ap > COLSIMAGE - 2)
                ap = COLSIMAGE - 2;
            if (an < 1)
                an = 1;
            else if (an > COLSIMAGE - 2)
                an = COLSIMAGE - 2;
            if (ah < 1)
                ah = 1;
            else if (ah > COLSIMAGE - 2)
                ah = COLSIMAGE - 2;
            if (fabs(ap - ah) >= width2 && fabs(an - ah) <= width1)
            // if (fabs(ap - ah) > fabs(an - ah))
            // if (fabs(an - ah) <= 150 && fabs(ap - ah) >= 50 && an < ap)
            {
                RD = POINT(track.spurroad[i].x, track.spurroad[i].y);
                RDF = true;
                break;
            }
        }
        // 没找到就起始点
        /*这里逻辑感觉还能再优化一下，虽然找不到下俩点就直接拿起始点也挺合理的*/
        if (!LDF)
        {
            LD = POINT(track.L_Start.x, track.L_Start.y);
            if (textDebug)
                cout << "左下角点未找到，使用起始点代替" << endl;
        }
        else
        {
            if (textDebug)
                cout << "左下角点找到，坐标为(" << LD.x << "," << LD.y << ")" << endl;
        }
        if (!RDF)
        {
            RD = POINT(track.R_Start.x, track.R_Start.y);
            if (textDebug)
                cout << "右下角点未找到，使用起始点代替" << endl;
        }
        else
        {
            if (textDebug)
                cout << "右下角点找到，坐标为(" << RD.x << "," << RD.y << ")" << endl;
        }
        // 6.开始判断这一对之间有没有连续贴边的边线
        // 左
        bool l_judge = false;
        int count = 0;
        for (int i = 1; i <= search; i++)
        {
            if (LU.x + i >= LD.x)
                break;
            if (track.pointsEdgeLeft[LU.x + i - track.LP].y <= square)
            {
                count++;
            }
            else
            {
                count = 0;
            }
            if (count >= yes)
            {
                l_judge = true;
                break;
            }
        }
        if (!l_judge)
        {
            if (textDebug)
                cout << "左边线不连续贴边，无法判断十字" << endl;
            return false;
        }
        // 右
        bool r_judge = false;
        count = 0;
        for (int i = 1; i <= search; i++)
        {
            if (RU.x + i >= RD.x)
                break;
            if (track.pointsEdgeRight[RU.x + i - track.RP].y >= COLSIMAGE - 1 - square)
            {
                count++;
            }
            else
            {
                count = 0;
            }
            if (count >= yes)
            {
                r_judge = true;
                break;
            }
        }
        if (!r_judge)
        {
            if (textDebug)
                cout << "右边线不连续贴边，无法判断十字" << endl;
            return false;
        }
        // 7.到这可以判断为十字了，开始补线
        float k1 = (float)(LD.y - LU.y) / (LD.x - LU.x); // 左边线斜率
        for (int i = LU.x + 1; i < LD.x; i++)
            track.pointsEdgeLeft[i - track.LP].y = round(LU.y + k1 * (i - LU.x)); // 左边线补线
        float k2 = (float)(RD.y - RU.y) / (RD.x - RU.x);                          // 右边线斜率
        for (int i = RU.x + 1; i < RD.x; i++)
            track.pointsEdgeRight[i - track.RP].y = round(RU.y + k2 * (i - RU.x)); // 右边线补线
        if (textDebug)
            cout << "十字识别补线成功" << endl;
        return true;
    }
};
