/*
 * @Description: Test program to read and print joint states from EtherCAT
 * @Author: Test
 * @Date: 2024
 */

extern "C" {
#include "ethercat.h"
#include "motor_control.h"
#include "transmit.h"
}
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <signal.h>
#include <cstdio>
#include <cstring>
#include <ctime>

int main(int argc, char* argv[])
{
  int ec_slavecount = EtherCAT_Init("enp86s0");
  
  if (ec_slavecount <= 0)
  {
    std::cerr << "[Test Error] Failed to initialize EtherCAT!\n";
    return -1;
  }

  std::cout << "[Test] EtherCAT initialized successfully!\n";
  std::cout << "[Test] Found " << ec_slavecount << " slave(s)\n\n";


  YKSMotorData yksSendcmdzero_[12]={};

  EtherCAT_Send_Command((YKSMotorData*)yksSendcmdzero_);

  EtherCAT_Get_State();
  
  while (true)
  {
    EtherCAT_Get_State();

    for (int i = 0; i < 12; i++)
    {
      std::cout << std::fixed << std::setprecision(4) << motorDate_recv[i].pos_;
      if (i < 11) std::cout << " ";
    }
    std::cout << std::endl;

    EtherCAT_Send_Command((YKSMotorData*)yksSendcmdzero_);
    usleep(2000); // 2ms
  }

  return 0;
}

