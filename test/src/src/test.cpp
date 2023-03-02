#include <iostream>
#include <memory>
#include <signal.h>
#include <unistd.h>
#include "./uart.hpp"
using namespace std;
int main()
{
    shared_ptr<Uart> uart = make_shared<Uart>("/dev/ttyUSB0"); // 初始化串口驱动
    int ret = uart->open();
    if (ret != 0)
    {
        printf("[Error] Uart Open failed!\n");
        return -1;
    }
    uart->startReceive(); // 启动数据接收子线程
    float speed;
    while (1)
    {
        cout << "输入：";
        cin >> speed;
        uart->carControl(speed, 1550);
        cout << "speed：" << speed << endl;
    }
    uart->close();
    cout << "结束" << endl;
    return 0;
}