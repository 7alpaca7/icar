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
    int zhuang;
    while (1)
    {
        cout << "可以输入：";
        cin >> zhuang;
        uart->carControl(0, zhuang);
        cout << "朝向：" << zhuang << endl;
    }
    uart->close();
    cout << "结束" << endl;
    return 0;
}