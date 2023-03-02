#pragma once
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "./tracking.cpp"
#include <iostream>
using namespace std;
using namespace cv;

/*
 * V1.0
 * 环岛识别与行径规划
 * @param track 基础赛道识别结果
 * @return bool 是否成功识别环岛
 * 环岛的初步识别与判断：
 * 1. 老套路先检查边线数量够不够，不够图像一半就false
 * 2. 这里要加一个判断，左右不可能同时有角点，左右也不可能同时丢线（这个会被起始点爬线影响，先不加入判断），如果有同时角点则false
 * 3. 根据角点出现的一侧，判断是左环还是右环，同时角点的数量起码要大于等于2个，小于两个就false
 * 4. 进行圆环一定也会有丢线，如果没丢线也false
 * ===============================================================================================================
 * 到这里已经判断圆环是左是右了，接下来是圆环的两个整体流程：
 * 1. 初入圆环-》将入圆环-》进入圆环-》出圆环
 * 2. 将入圆环-》进入圆环-》出圆环-》完成圆环
 * 注意事项：
 * 1.完成第一个流程的关键在于，能不能找到出环口的点，和内环的点，将状态推入下一步
 * 2.完成第二个流程的关键在于，能不能找到内环的点，和入环口的点，补线将车送进环内，把状态推入下一步
 * 3.环岛的补线逻辑有斜率补线和贝塞尔曲线补线两种方式，测试用斜率补线，实际用贝塞尔曲线补线，更侧重用贝塞尔曲线补线
 * ===============================================================================================================
 * 无状态
 * 选取角点的方式：
 * 1.先找上下角点，上角点的判断为当前点的下连续n行为补线点，下角点的判断为当前点的上连续n行为补线点
 * 2.若没找到上下角点，上角点的标记位置为0，下角点的标记位置为单侧角点数量-1
 * 3.然后通过上下角点标记的范围遍历角点数组找中间角点，判断中间角点的方式为：当前点上连续n行y递减然后到连续补线处，当前点下连续n行y递减然后到连续补线处
 * 4.这个中间角点采取隔几个点取一个点的方式，一直递减，然后到达连续补线处时一个一个采用连续n次
 * 5.初入圆环识别成功在于有无找到下角点和中间角点，下中用斜率补线，而上角点的识别属于是顺便，方便将入圆环的处理，进入初入圆环的状态
 * 6.如果识别到上角点和中间角点，则与中间角点同一行的另外一侧边线点作为补线的起点中补点，上中补点补线，进入将入圆环状态
 * 7.如果没有识别到中间点则false
 * ===============================================================================================================
 * 初入圆环
 * 1.找中下角点，如果连续找到中下角点m次，即m帧，则认为找到了中下角点，中下角点补线
 * 2.当识别不到下角点时，补线结束，将状态推入将入圆环状态
 * ===============================================================================================================
 * 将入圆环
 * 1.找中上角点，如果连续找到中上角点m次，即m帧，则认为找到了中上角点
 * 2.与中角点同一行的另一侧边线点选取为补线的起点，称为中补点，上角点与中补点补线 ****（这里测试先选用斜率补线，实际改为贝塞尔曲线）****
 * 3.当中角点识别不到时，以防车没有进入环内，采用另一侧起始点与上角点补线，直到上角点识别不到后，将状态推入进入圆环状态
 * ===============================================================================================================
 * 进入圆环，就正常走
 * ===============================================================================================================
 * 出环
 * 1.入环一侧的另外一侧一定是出环侧，选择出环侧拐点中y值最小的点作为出环点补点的起始点，用贝塞尔曲线补线到入环侧第一个点的位置
 * 2.出环侧拐点消失后，选择出环侧起始点补到入环侧第一个点的位置
 * 3.基于第二步感觉可以加个判断，就是让第二步一直补到看见入环口第一个角点处，看见后就立马第一个角点与入环侧起始点补线，待入环侧点消失后，结束整个圆环，将圆环状态初始化
 * ===============================================================================================================
 * 注意事项：
 * 1.如果只代入正常圆环识别的话，理论上是可行的，但是由于中间角点的判断与斜入十字的L角点判断有些类似，会产生误判
 * 2.正入圆环道路的特点：一侧有角点且丢线，另一侧无角点且丢线，没角点的那侧即使有丢线，也是因为起始行爬线导致的，理论上可以用阈值限制
 * 3.通过观察可知，内圆上由于不受点覆盖的影响，它的点集分布是比较均匀的，而L型角点呈现的是上下部分不均匀的显示
 */
class Ring
{
private:
    enum RingType
    {
        None = 0,
        RingLeft, // 左入环
        RingRight // 右入环
    };
    enum RingStep
    {
        NoRing = 0, // 没判断圆环状态
        IsRing,     // 初入圆环
        Entering,   // 将入圆环
        Inside,     // 进入圆环
        Exiting,    // 出圆环
        Finish      // 结束
    };
    RingStep nowType = RingStep::NoRing;         // 当前状态
    RingType ringSide = RingType::None;          // 圆环侧
    POINT RUPoint, RZPoint, RDPoint, ROPoint;    // 分别为入环上角点，入环中角点，入环下角点，出环角点
    bool textDeBug = false;                      // 日志输出debug开关
    int diuYu = 20;                              // 丢线允许的阈值
    int zhen = 10;                                // 连续出现帧数的阈值
    int ZDcount = 0, UDcount = 0, TypeCount = 0; // 当前中下角点连续出现的帧数，中上角点连续出现的帧数
    int JieLeft = 1, JieRight = COLSIMAGE - 2;   // 图像左右两边的边界
    int jian = 10;                               // 判断上下角点前后的间隔
    int width1 = 10, width2 = 30;                // 宽度阈值
    int search = 30;                             // 贴边搜索的范围
    int tieCount = 3;                            // 连续贴边的次数
    bool checkStatus = false;                    // 用来保护已经检查的状态
    bool out_flag = false;

    /**
     * @brief 限定范围
     * @param now 当前值
     * @param min_num 最小值
     * @param max_num 最大值
     */
    int defence(const int &now, const int &min_num, const int &max_num)
    {
        int x = now;
        if (x < min_num)
            x = min_num;
        else if (x > max_num)
            x = max_num;
        return x;
    }

    /**
     * @brief 找上角点
     * @param edge 赛道边线
     * @param spurroad 角点集
     * @param GStart 角点起始搜索下标
     * @param GEnd 角点终止搜索下标
     * @param EP 边线集的偏移量
     * @param p 存储找到角点位置
     * @param L 经过初步判断后，退出函数前连续贴边的x在图像坐标
     * @param bian 贴边的数值
     * @return 返回找到角点的下标位置，如果没有就返回-1
     */
    int findUp(const vector<POINT> &edge, const vector<POINT> &spurroad, const int &GStart, const int &GEnd, const int &EP, POINT &p, int &L, const int &bian)
    {
        int findFlag = -1;
        // 初步找点
        for (int i = GStart; i < GEnd; i++)
        {
            // 当前角点的下标，当前角点往前jian边线集的下标，当前角点往后jian边线集的下标
            int here = spurroad[i].x, pre = spurroad[i].x - jian, next = spurroad[i].x + jian;
            // 防止越界
            here = defence(here - EP, 0, edge.size() - 1);
            pre = defence(pre - EP, 0, edge.size() - 1);
            next = defence(next - EP, 0, edge.size() - 1);
            // 得到点y值
            int ah = edge[here].y, an = edge[next].y, ap = edge[pre].y;
            ah = defence(ah, 1, COLSIMAGE - 2);
            an = defence(an, 1, COLSIMAGE - 2);
            ap = defence(ap, 1, COLSIMAGE - 2);
            if (textDeBug)
            {
                cout << endl;
                printf("here=%4d,pre=%4d,next=%4d\n", here, pre, next);
                printf("ah=%4d,ap=%4d,an=%4d\n", ah, ap, an);
                printf("abs(ah-ap)=%4d,abs(ah-an)=%4d,width1=%4d,width2=%4d", abs(ah - ap), abs(ah - an), width1, width2);
            }
            if (abs(ah - ap) <= width1 && abs(ah - an) >= width2)
            {
                p = POINT(here + EP, ah);
                findFlag = i;
                break;
            }
        }
        if (findFlag == -1)
        {
            L = edge[0].x;
            if (textDeBug)
                cout << "找上角点的初步判断没成功" << endl;
            return findFlag;
        }
        // 进一步判断这个角点是否有丢线
        int c = 0;
        for (int i = 1; i <= search; i++)
        {
            int x = defence(spurroad[findFlag].x - EP + i, 0, edge.size() - 1);
            int y = defence(edge[x].y, 1, COLSIMAGE - 2);
            if (y == bian)
                c++;
            else
                c = 0;
            if (c >= tieCount)
            {
                L = x;
                return findFlag;
            }
        }
        L = edge[0].x;
        if (textDeBug)
            cout << "找上角点的第二次判断没成功" << endl;
        return -1;
    }

    /**
     * @brief 判断是否分布是否连续
     * @param edge 边线集
     * @param index 起始下标
     * @param L x坐标最小值
     * @param E x坐标最大值
     * @return 不在限制范围就false，不满足变化趋势连续也false，满足变化趋势连续为true
     */
    bool JudgeDeFu(const vector<POINT> &edge, const int &index, const int &L, const int &E)
    {
        if (edge[index].x <= L || edge[index].x >= E)
        {
            if (textDeBug)
                printf("\nx=%4d,不在限制%4d~%4d范围\n", index, L, E);
            return false;
        }
        if (textDeBug)
        {
            cout << endl
                 << "JudgeDeFu" << endl;
            cout << "L = " << L << ", E = " << E << endl;
            cout << "Before 5: ";
            for (int i = 1; i <= 5; i++)
                cout << edge[index - i].y << " ";
            cout << endl
                 << "Zhong:" << edge[index].y << endl;
            cout << "After 5: ";
            for (int i = 1; i <= 5; i++)
                cout << edge[index + i].y << ' ';
            cout << endl
                 << (index + 5 < edge.size() && index - 5 >= 0 && abs(abs(edge[index - 1].y - edge[index].y) - abs(edge[index + 1].y - edge[index].y)) <= 1 && abs(abs(edge[index - 2].y - edge[index - 1].y) - abs(edge[index + 1].y - edge[index + 2].y)) <= 1 && abs(abs(edge[index - 2].y - edge[index - 3].y) - abs(edge[index + 3].y - edge[index + 2].y)) <= 1 && abs(abs(edge[index - 4].y - edge[index - 3].y) - abs(edge[index + 4].y - edge[index + 3].y)) <= 1 && abs(abs(edge[index - 5].y - edge[index - 4].y) - abs(edge[index + 5].y - edge[index + 4].y)) <= 1) << endl;
        }
        return (index + 5 < edge.size() && index - 5 >= 0 && abs(abs(edge[index - 1].y - edge[index].y) - abs(edge[index + 1].y - edge[index].y)) <= 1 && abs(abs(edge[index - 2].y - edge[index - 1].y) - abs(edge[index + 1].y - edge[index + 2].y)) <= 1 && abs(abs(edge[index - 2].y - edge[index - 3].y) - abs(edge[index + 3].y - edge[index + 2].y)) <= 1 && abs(abs(edge[index - 4].y - edge[index - 3].y) - abs(edge[index + 4].y - edge[index + 3].y)) <= 1 && abs(abs(edge[index - 5].y - edge[index - 4].y) - abs(edge[index + 5].y - edge[index + 4].y)) <= 1);
    }

    /**
     * @brief 找内圆中点
     * @param edge 边线集
     * @param spurroad 角点集
     * @param GStart 角点集起始下标
     * @param GEnd 角点集终止下标
     * @param p 存储角点信息
     * @param L 图像限制x最小值
     * @param E 图像限制x最大值
     * @param bian 图像贴边数值
     * @return 满足条件的情况下找到内圆中点为true，其余false
     */
    int findZhong(const vector<POINT> &edge, const vector<POINT> &spurroad, const int &GStart, const int &GEnd, const int &EP, POINT &p, int &L, int &E, const int &bian)
    {
        // 缩小一下坐标范围，如果缩小一下L与E相遇，说明就没有内环
        while (L < edge.size() && L <= E && edge[L].y == bian)
            L++;
        if (L >= E)
        {
            if (textDeBug)
                printf("L增大时,L与E相遇,L=%4d,E=%4d", L, E);
            return -1;
        }
        while (E >= 0 && E >= L && edge[E].y == bian)
            E--;
        if (L >= E)
        {
            if (textDeBug)
                printf("E减小时,L与E相遇,L=%4d,E=%4d", L, E);
            return -1;
        }
        // 开始判内环点
        for (int i = GStart; i < GEnd; i++)
        {
            int x = spurroad[i].x - EP;
            if (JudgeDeFu(edge, x, L, E))
            {
                p = POINT(x, edge[x].y);
                return i;
            }
        }
        return -1;
    }

    /**
     * @brief 找下角点
     * @param edge 赛道边线
     * @param spurroad 角点集
     * @param GStart 角点起始搜索下标
     * @param GEnd 角点终止搜索下标
     * @param EP 边线集的偏移量
     * @param p 存储找到角点位置
     * @param L 经过初步判断后，退出函数前连续贴边的x在图像坐标
     * @param bian 贴边的数值
     * @return 返回找到角点的下标位置，如果没有就返回-1
     */
    int findDown(const vector<POINT> &edge, const vector<POINT> &spurroad, const int &GStart, const int &GEnd, const int &EP, POINT &p, int &L, const int &bian)
    {
        int findFlag = -1;
        for (int i = GStart; i >= GEnd; i--)
        {
            // 当前角点的下标，当前角点往前jian边线集的下标，当前角点往后jian边线集的下标
            int here = spurroad[i].x, pre = spurroad[i].x - jian, next = spurroad[i].x + jian;
            // 防止越界
            here = defence(here - EP, 0, edge.size() - 1);
            pre = defence(pre - EP, 0, edge.size() - 1);
            next = defence(next - EP, 0, edge.size() - 1);
            // 得到点y值
            int ah = edge[here].y, an = edge[next].y, ap = edge[pre].y;
            ah = defence(ah, 1, COLSIMAGE - 2);
            an = defence(an, 1, COLSIMAGE - 2);
            ap = defence(ap, 1, COLSIMAGE - 2);
            if (textDeBug)
            {
                cout << endl;
                printf("here=%4d,pre=%4d,next=%4d\n", here, pre, next);
                printf("ah=%4d,ap=%4d,an=%4d\n", ah, ap, an);
                printf("abs(ah-ap)=%4d,abs(ah-an)=%4d,width1=%4d,width2=%4d", abs(ah - ap), abs(ah - an), width1, width2);
            }
            if (abs(ah - an) <= width1 && abs(ah - ap) >= width2)
            {
                p = POINT(here + EP, ah);
                findFlag = i;
                break;
            }
        }
        if (findFlag == -1)
        {
            L = edge.back().x;
            if (textDeBug)
                cout << "找下角点的初步判断没成功" << endl;
            return findFlag;
        }
        // 进一步判断这个角点是否有丢线
        int c = 0;
        for (int i = 1; i <= search; i++)
        {
            int x = defence(spurroad[findFlag].x - EP - i, 0, edge.size() - 1);
            int y = defence(edge[x].y, 1, COLSIMAGE - 2);
            if (y == bian)
                c++;
            else
                c = 0;
            if (c >= tieCount)
            {
                L = x;
                return findFlag;
            }
        }
        L = edge.back().x;
        if (textDeBug)
            cout << "找下角点的第二次判断没成功" << endl;
        return -1;
    }

    /**
     * @brief 补线
     * @param edge 边线集
     * @param start 补线的起始点
     * @param end 补线的终点
     */
    void addLine(vector<POINT> &edge, const POINT &start, POINT &end)
    {
        // 斜率
        float k = (end.y - start.y) * 1.0 / (end.x - start.x);
        // 补线
        for (int i = start.x; i <= end.x; i++)
        {
            if (out_flag)
                edge[i].x = i;
            edge[i].y = round(start.y + k * (i - start.x));
        }
    }

    /**
     * @brief 初步检测当前图像条件
     */
    bool is_OK(Tracking &track)
    {
        // 第一个边线点集不够
        if (track.pointsEdgeLeft.size() < ROWSIMAGE / 2 || track.pointsEdgeRight.size() < ROWSIMAGE / 2)
        {
            if (textDeBug)
                cout << "左右边线数量不够" << endl;
            return false;
        }
        // // 第二个没丢线直接清，只允许一边丢线，另一边丢线只能是阈值内允许
        // // if ((track.diuLeft <= diuYu && track.diuRight <= diuYu) || (track.diuLeft > diuYu && track.diuRight > diuYu))
        // if (track.diuLeft < diuYu && track.diuRight <= diuYu)
        // {
        //     if (textDeBug)
        //         cout << "丢线不符合要求" << endl;
        //     return false;
        // }
        // // 第三个角点数量同时小于等于1或同时都大于1一块清
        // // if ((track.LGnum <= 1 && track.RGnum <= 1) || (track.LGnum > 1 && track.RGnum > 1))
        // if (track.LGnum <= 1 && track.RGnum <= 1)
        // {
        //     if (textDeBug)
        //         cout << "角点数量不符合要求" << endl;
        //     return false;
        // }
        /* 这里感觉限制的有点刻意了，虽然在看见环的情况下，一边几乎不丢线 */
        if(textDeBug){
            cout<<"track.diuLeft:"<<track.diuLeft<<", track.LGnum:"<<track.LGnum<<endl;
        cout<<"track.diuRight:"<<track.diuRight<<", track.RGnum:"<<track.RGnum<<endl;
        cout<<"diuYu:"<<diuYu<<endl;
        }
        // if ((track.diuLeft >= diuYu && track.LGnum >= 2 && track.diuLeft > track.diuRight) || (track.diuRight >= diuYu && track.RGnum >= 2 && track.diuRight > track.diuLeft))
        if ((track.diuLeft >= diuYu && track.LGnum >= 2 && track.diuLeft > track.diuRight) || (track.diuRight >= diuYu && track.RGnum >= 2 && track.diuRight > track.diuLeft))
            return true;
        return false;
    }

    bool findUZD(Tracking &track, int &up_index, int &down_index, int &zhong_index)
    {
        int L, E;
        // 左环
        if (ringSide == RingType::RingLeft)
        {
            up_index = findUp(track.pointsEdgeLeft, track.spurroad, 0, track.LGnum, track.LP, RUPoint, L, 1);
            down_index = findDown(track.pointsEdgeLeft, track.spurroad, track.LGnum - 1, max(up_index, 0), track.LP, RDPoint, E, 1);
        }
        // 右环
        else
        {
            up_index = findUp(track.pointsEdgeRight, track.spurroad, track.LGnum, track.spurroad.size(), track.RP, RUPoint, L, COLSIMAGE - 2);
            down_index = findDown(track.pointsEdgeRight, track.spurroad, track.spurroad.size() - 1, max(down_index, track.LGnum), track.RP, RDPoint, E, COLSIMAGE - 2);
        }
        if (textDeBug)
        {
            printf("L = %3d, E = %3d\n", L, E);
            printf("up_index = %3d, down_index = %3d\n", up_index, down_index);
        }
        // 上角点和下角点同时没有，或者下角点比上角点大，或者上下角点下标相邻
        // if ((up_index == -1 && down_index == -1) || ((down_index >= 0 && up_index >= 0 && down_index <= up_index) || (down_index >= 0 && up_index >= 0 && down_index - up_index == 1)))
        if ((up_index == -1 && down_index == -1) || (up_index >= 0 && down_index >= 0 && abs(up_index - down_index) == 1))
        {
            if (textDeBug)
                printf("上下角点不满足条件,up_index=%4d,down_index=%4d", up_index, down_index);
            ringSide = RingType::None;
            return false;
        }
        // 最后找中点
        if (ringSide == RingType::RingLeft)
        {
            zhong_index = findZhong(track.pointsEdgeLeft, track.spurroad, max(up_index, 0), (down_index == -1 ? track.LGnum : down_index), track.LP, RZPoint, L, E, 1);
        }
        else
        {
            zhong_index = findZhong(track.pointsEdgeRight, track.spurroad, max(up_index, track.LGnum), (down_index == -1 ? track.spurroad.size() : down_index), track.RP, RZPoint, L, E, COLSIMAGE - 2);
        }
        if (textDeBug)
            printf("zhong_index = %3d\n", zhong_index);
        if (zhong_index == -1)
        {
            if (textDeBug)
                cout << "没有找到内圆中点" << endl;
            return false;
        }
        return true;
    }

    bool findOut(const vector<POINT> &edge, const vector<POINT> &spurroad, const int &EP, const int &GStart, const int &GEnd)
    {
        for (int i = GStart; i >= GEnd; i--)
        {
            int here = spurroad[i].x, pre = spurroad[i].x - jian, next = spurroad[i].x + jian;
            here = defence(here - EP, 0, edge.size() - 1);
            pre = defence(pre - EP, 0, edge.size() - 1);
            next = defence(next - EP, 0, edge.size() - 1);
            int ah, ap, an;
            ah = defence(edge[here].y, 1, COLSIMAGE - 2);
            ap = defence(edge[pre].y, 1, COLSIMAGE - 2);
            an = defence(edge[next].y, 1, COLSIMAGE - 2);
            if (textDeBug)
            {
                cout << endl;
                printf("here=%4d,pre=%4d,next=%4d\n", here, pre, next);
                printf("ah=%4d,ap=%4d,an=%4d\n", ah, ap, an);
            }
            if (ah < ap && ah < an)
            {
                ROPoint = POINT(here + EP, ah);
                return i;
            }
        }
        return -1;
    }

    void OutBu(vector<POINT> &edgeW, vector<POINT> &edgeN, int bian, POINT start, POINT end)
    {
        addLine(edgeW, start, end);
        // for (int i = 0; i < edgeN.size(); i++)
        // {
        //     if (edgeN[i].y == bian)
        //         break;
        //     edgeN[i].x = i;
        //     edgeN[i].y = bian;
        // }
    }
    void clean(vector<POINT> &edge, int i)
    {
        int j = 0;
        while (j < edge.size() && edge[j].x <= i)
            j++;
        edge.erase(edge.begin(), edge.begin() + j);
    }
    void addPoint(vector<POINT> &edge, POINT &p1, POINT &p2, POINT &p3, bool del)
    {
        vector<POINT> input = {p1, p2, p3};
        input = Bezier(0.02, input);
        if (del)
        {
            clean(edge, p3.x + 5);
            reverse(edge.begin(), edge.end());
            for (int i = input.size() - 1; i >= 0; i--)
                edge.push_back(input[i]);
            reverse(edge.begin(), edge.end());
        }
        else
        {
            edge.clear();
            edge = input;
        }
    }

public:
    bool process(Tracking &track)
    {
        if (textDeBug)
            cout << endl
                 << "开始识别圆环," << nowType << endl;
        int up_index = -1, down_index = -1, zhong_index = -1, out_index = -1;
        // 没识别圆环
        if (nowType == RingStep::NoRing)
        {
            // 1.识别第一步判环条件不够直接清
            if (!is_OK(track))
                {
                    if(textDeBug)
                    cout<<"fw,条件都不满足"<<endl;
                return false;
                }
                
            // 2.判环方向
            if (track.LGnum > track.RGnum)
                ringSide = RingType::RingLeft;
            else
                ringSide = RingType::RingRight;
            if (textDeBug)
                cout << "现在判断为" << (ringSide == RingType::RingLeft ? "左环" : "右环") << endl;
            // 3.开始找点
            // 先找上下角点，上下角点如果都没有也是false
            if (!findUZD(track, up_index, down_index, zhong_index))
                return false;
            // 4.判阶段
            if (down_index != -1 && zhong_index != -1)
                ZDcount++;
            else
                ZDcount = 0;
            if (zhong_index != -1 && up_index != -1)
                UDcount++;
            else
                UDcount = 0;
            if (ZDcount >= zhen)
            {
                nowType = RingStep::IsRing;
                TypeCount = 0;
                return true;
            }
            else if (UDcount >= zhen)
            {
                nowType = RingStep::Entering;
                TypeCount = 0;
                return true;
            }
            return false;
        }
        // 初入圆环
        else if (nowType == RingStep::IsRing)
        {
            // if (!is_OK(track))
            //     return false;
            if (ringSide == RingType::RingLeft)
            {
                if (track.pointsEdgeLeft.size() < ROWSIMAGE / 2 || track.LGnum == 0)
                {
                    TypeCount = 0;
                    return false;
                }
            }
            else
            {
                if (track.pointsEdgeRight.size() < ROWSIMAGE / 2 || track.RGnum == 0)
                {
                    TypeCount = 0;
                    return false;
                }
            }
            int L, E;
            // 找一手上中下三点，虽然主要是补中下的线，但是如果看见上面的也可以蛮补一手
            // 左环
            if (ringSide == RingType::RingLeft)
            {
                up_index = findUp(track.pointsEdgeLeft, track.spurroad, 0, track.LGnum, track.LP, RUPoint, L, 1);
                down_index = findDown(track.pointsEdgeLeft, track.spurroad, track.LGnum - 1, max(up_index, 0), track.LP, RDPoint, E, 1);
            }
            // 右环
            else
            {
                up_index = findUp(track.pointsEdgeRight, track.spurroad, track.LGnum, track.spurroad.size(), track.RP, RUPoint, L, COLSIMAGE - 2);
                down_index = findDown(track.pointsEdgeRight, track.spurroad, track.spurroad.size() - 1, max(track.LGnum, up_index), track.RP, RDPoint, E, COLSIMAGE - 2);
            }
            if (textDeBug)
            {
                printf("L = %3d, E = %3d\n", L, E);
                printf("up_index = %3d, down_index = %3d\n", up_index, down_index);
            }

            // 最后找中点
            if (ringSide == RingType::RingLeft)
            {
                zhong_index = findZhong(track.pointsEdgeLeft, track.spurroad, max(up_index, 0), (down_index == -1 ? track.LGnum : down_index), track.LP, RZPoint, L, E, 1);
            }
            else
            {
                zhong_index = findZhong(track.pointsEdgeRight, track.spurroad, max(up_index, track.LGnum), (down_index == -1 ? track.spurroad.size() : down_index), track.RP, RZPoint, L, E, COLSIMAGE - 2);
            }
            if (textDeBug)
                printf("zhong_index = %3d\n", zhong_index);
            if (down_index == -1)
            {
                TypeCount++;
            }
            else
            {
                TypeCount = 0;
            }
            if (TypeCount >= zhen)
            {
                TypeCount = 0;
                nowType = RingStep::Entering;
                return true;
            }
            if (zhong_index != -1)
            {
                if (ringSide == RingType::RingLeft)
                {
                    addLine(track.pointsEdgeLeft, track.spurroad[zhong_index], track.spurroad[down_index]);
                }
                else
                {
                    addLine(track.pointsEdgeRight, track.spurroad[zhong_index], track.spurroad[down_index]);
                }
                if (up_index != -1)
                {
                    if (ringSide == RingType::RingLeft)
                    {
                        clean(track.pointsEdgeLeft, track.spurroad[up_index].x);
                        addLine(track.pointsEdgeRight, track.spurroad[up_index], track.pointsEdgeRight[track.spurroad[zhong_index].x - track.LP]);
                        clean(track.pointsEdgeRight, track.spurroad[up_index].x);
                    }
                    else
                    {
                        clean(track.pointsEdgeRight, track.spurroad[up_index].x);
                        addLine(track.pointsEdgeLeft, track.spurroad[up_index], track.pointsEdgeLeft[track.spurroad[zhong_index].x - track.RP]);
                        clean(track.pointsEdgeLeft, track.spurroad[up_index].x);
                    }
                }
            }
        }
        // 将入圆环
        else if (nowType == RingStep::Entering)
        {
            // if (!is_OK(track))
            //     return false;
            if (ringSide == RingType::RingLeft)
            {
                if (track.pointsEdgeLeft.size() < ROWSIMAGE / 2)
                {
                    TypeCount = 0;
                    return false;
                }
            }
            else
            {
                if (track.pointsEdgeRight.size() < ROWSIMAGE / 2)
                {
                    TypeCount = 0;
                    return false;
                }
            }
            int L;
            if (ringSide == RingType::RingLeft)
            {
                up_index = findUp(track.pointsEdgeLeft, track.spurroad, 0, track.LGnum, track.LP, RUPoint, L, 1);
                zhong_index = findZhong(track.pointsEdgeLeft, track.spurroad, max(up_index, 0), track.LGnum, track.LP, RUPoint, L, track.pointsEdgeLeft.back().x, 1);
            }
            else
            {
                up_index = findUp(track.pointsEdgeRight, track.spurroad, track.LGnum, track.spurroad.size(), track.RP, RUPoint, L, COLSIMAGE - 2);
                zhong_index = findZhong(track.pointsEdgeRight, track.spurroad, max(up_index, track.LGnum), track.spurroad.size(), track.RP, RZPoint, L, track.pointsEdgeRight.back().x, COLSIMAGE - 2);
            }
            if (zhong_index == -1 && up_index == -1)
            {
                TypeCount++;
            }
            else
            {
                TypeCount = 0;
            }
            if (TypeCount >= zhen)
            {
                TypeCount = 0;
                nowType = RingStep::Inside;
                return true;
            }
            if (up_index != -1)
            {

                if (zhong_index != -1)
                {

                    if (ringSide == RingType::RingLeft)
                    {

                        // addLine(track.pointsEdgeRight, track.spurroad[up_index], track.pointsEdgeRight[track.spurroad[zhong_index].x - track.LP]);
                        // clean(track.pointsEdgeRight, track.spurroad[zhong_index].x + 5);
                        // clean(track.pointsEdgeLeft, track.spurroad[up_index].x + 5);
                        // POINT p1 = track.spurroad[up_index], p2 = track.pointsEdgeRight[track.spurroad[up_index].x - track.LP], p3 = track.pointsEdgeRight[track.spurroad[zhong_index].x-track.LP];
                        clean(track.pointsEdgeLeft, track.spurroad[up_index].x + 5);
                        POINT p1 = track.spurroad[up_index];
                        POINT p2 = track.pointsEdgeRight[track.spurroad[up_index].x - track.LP];
                        POINT p3 = track.R_Start;
                        addPoint(track.pointsEdgeRight, p1, p2, p3, false);
                    }
                    else
                    {

                        // addLine(track.pointsEdgeLeft, track.spurroad[up_index], track.pointsEdgeLeft[track.spurroad[zhong_index].x - track.RP]);
                        // clean(track.pointsEdgeRight, track.spurroad[up_index].x + 5);
                        // clean(track.pointsEdgeLeft, track.spurroad[zhong_index].x + 5);
                        clean(track.pointsEdgeRight, track.spurroad[up_index].x + 5);
                        POINT p1 = track.spurroad[up_index];
                        POINT p2 = track.pointsEdgeLeft[track.spurroad[up_index].x - track.RP];
                        POINT p3 = track.L_Start;
                        addPoint(track.pointsEdgeLeft, p1, p2, p3, false);
                    }
                }
                else if (zhong_index == -1)
                {
                    if (ringSide == RingType::RingLeft)
                    {

                        // addLine(track.pointsEdgeRight, track.spurroad[up_index], track.R_Start);
                        // clean(track.pointsEdgeRight, track.spurroad[up_index].x + 5);
                        // clean(track.pointsEdgeLeft, track.spurroad[up_index].x + 5);
                        clean(track.pointsEdgeLeft, track.spurroad[up_index].x + 5);
                        POINT p1 = track.spurroad[up_index];
                        POINT p2 = track.pointsEdgeRight[track.spurroad[up_index].x - track.LP];
                        POINT p3 = track.R_Start;
                        addPoint(track.pointsEdgeRight, p1, p2, p3, false);
                    }
                    else
                    {

                        // addLine(track.pointsEdgeLeft, track.spurroad[up_index], track.L_Start);
                        // clean(track.pointsEdgeRight, track.spurroad[up_index].x + 5);
                        // clean(track.pointsEdgeLeft, track.spurroad[up_index].x + 5);
                        clean(track.pointsEdgeRight, track.spurroad[up_index].x + 5);
                        POINT p1 = track.spurroad[up_index];
                        POINT p2 = track.pointsEdgeLeft[track.spurroad[up_index].x - track.RP];
                        POINT p3 = track.L_Start;
                        addPoint(track.pointsEdgeLeft, p1, p2, p3, false);
                    }
                }
            }
        }
        // 进入圆环
        else if (nowType == RingStep::Inside)
        {
            if (ringSide == RingType::RingLeft)
            {
                if (track.RGnum == 0)
                    out_index = -1;
                else
                    out_index = findOut(track.pointsEdgeRight, track.spurroad, track.RP, track.spurroad.size() - 1, track.LGnum);
            }
            else
            {
                if (track.LGnum == 0)
                    out_index = -1;
                else
                    out_index = findOut(track.pointsEdgeLeft, track.spurroad, track.LP, track.LGnum - 1, 0);
            }
            if (out_index != -1)
            {
                TypeCount++;
            }
            else
            {
                TypeCount = 0;
            }
            if (TypeCount >= zhen)
            {
                TypeCount = 0;
                nowType = RingStep::Exiting;
                return true;
            }
        }
        // 出圆环
        else if (nowType == RingStep::Exiting)
        {
            out_flag = true;
            if (ringSide == RingType::RingLeft)
            {
                if (track.RGnum == 0)
                    out_index = -1;
                else
                    out_index = findOut(track.pointsEdgeRight, track.spurroad, track.RP, track.spurroad.size() - 1, track.LGnum);
            }
            else
            {
                if (track.LGnum == 0)
                    out_index = -1;
                else
                    out_index = findOut(track.pointsEdgeLeft, track.spurroad, track.LP, track.LGnum - 1, 0);
            }
            if (out_index == -1)
            {
                TypeCount++;
            }
            else
            {
                TypeCount = 0;
            }
            if (textDeBug)
                cout << "TypeCount=" << TypeCount << " ,out_index=" << out_index << endl;
            if (TypeCount >= zhen+10)
            {
                nowType = RingStep::Finish;
                TypeCount = 0;
                out_flag = false;
                return true;
            }
            POINT p1 = POINT(ROWSIMAGE / 2, 0);
            POINT p2 = POINT(ROWSIMAGE - 1, 0);
            POINT p3 = POINT(ROWSIMAGE / 2, COLSIMAGE - 1);
            POINT p4 = POINT(ROWSIMAGE - 1, COLSIMAGE - 1);
            if (ringSide == RingType::RingLeft)
            {
                // addLine(track.pointsEdgeRight, POINT(0, 0), track.spurroad[out_index]);
                // OutBu(track.pointsEdgeRight, track.pointsEdgeLeft, JieLeft, POINT(0, 0), POINT(ROWSIMAGE - 1, JieRight));
                track.pointsEdgeLeft.clear();
                track.pointsEdgeLeft = {p1, p2};
                track.pointsEdgeLeft = Bezier(0.01, track.pointsEdgeLeft);
                addPoint(track.pointsEdgeRight, p1, p3, p4, false);
            }
            else
            {
                // addLine(track.pointsEdgeLeft, POINT(0, COLSIMAGE - 1), track.spurroad[out_index]);
                // OutBu(track.pointsEdgeLeft, track.pointsEdgeRight, JieRight, POINT(0, COLSIMAGE - 1), POINT(ROWSIMAGE - 1, JieLeft));
                track.pointsEdgeRight.clear();
                track.pointsEdgeRight = {p3, p4};
                track.pointsEdgeRight = Bezier(0.01, track.pointsEdgeRight);
                addPoint(track.pointsEdgeLeft, p3, p1, p2, false);
            }
            // if (out_index != -1)
            // {
            //     if (ringSide == RingType::RingLeft)
            //     {
            //         // addLine(track.pointsEdgeRight, POINT(0, 0), track.spurroad[out_index]);
            //         OutBu(track.pointsEdgeRight, track.pointsEdgeLeft, JieLeft, POINT(0, 0), track.spurroad[out_index]);
            //     }
            //     else
            //     {
            //         // addLine(track.pointsEdgeLeft, POINT(0, COLSIMAGE - 1), track.spurroad[out_index]);
            //         OutBu(track.pointsEdgeLeft, track.pointsEdgeRight, JieRight, POINT(0, COLSIMAGE - 1), track.spurroad[out_index]);
            //     }
            // }
            // else
            // {
            //     if (ringSide == RingType::RingLeft)
            //     {
            //         // addLine(track.pointsEdgeRight, POINT(0, 0), track.spurroad[out_index]);
            //         OutBu(track.pointsEdgeRight, track.pointsEdgeLeft, JieLeft, POINT(0, 0), POINT(track.rowEnd, JieRight));
            //     }
            //     else
            //     {
            //         // addLine(track.pointsEdgeLeft, POINT(0, COLSIMAGE - 1), track.spurroad[out_index]);
            //         OutBu(track.pointsEdgeLeft, track.pointsEdgeRight, JieRight, POINT(0, COLSIMAGE - 1), POINT(track.rowEnd, JieLeft));
            //     }
            // }
        }
        else if (nowType == RingStep::Finish)
        {
            TypeCount++;
            if (TypeCount >= 50)
            {
                TypeCount = 0;
                nowType = RingStep::NoRing;
                ringSide = RingType::None;
                if (textDeBug)
                    cout << "圆环完成" << endl;
            }
        }
        return true;
    }
};