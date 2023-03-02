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
    int servo;
    while (1)
    {
        cout << "servo输入：";
        cin >> servo;
        uart->carControl(0, servo);
        cout << "servo:" << servo << endl;
    }
    uart->close();
    cout << "结束" << endl;
    return 0;
}