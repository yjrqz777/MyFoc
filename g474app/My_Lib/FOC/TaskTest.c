/***************************************************************************************************
 * Author: yjrqz777 3210551161@qq.com
 * Date: 2025-05-26 21:14:34
 * LastEditTime: 2025-05-29 21:42:44
 * LastEditors: yjrqz777 3210551161@qq.com
 * Description: 
 * FilePath: \g474app\My_Lib\FOC\TaskTest.c
 * @YJRQZ777
***************************************************************************************************/
#include "Task.h"



unsigned int PT_TICK[TASK_MAX]={0};

void time1ms(void)
{
    TASK_TICK_UPDATE();
}









static int PT_TASK_TEST()
{
  /* A protothread function must begin with PT_BEGIN() which takes a
     pointer to a struct pt. */
  PT_BEGIN()
  {
        printf("01\n");
  }

  /* We loop forever here. */
  while(1) {
        printf("1\n");
    /* Wait until the other protothread has set its flag. */
    PT_WAIT_UNTIL(100);
    printf("2\n");
    PT_WAIT_UNTIL(500);
        printf("3\n");
    PT_WAIT_UNTIL(1500);
        printf("4\n");
    /* And we loop. */
  }
      printf("5\n");
  PT_END();
}


static int PT_TASK_TEST2()
{
  /* A protothread function must begin with PT_BEGIN() which takes a
     pointer to a struct pt. */
     static unsigned int i = 0;
  PT_BEGIN()
  {
    printf("02\n");
     i = 2;
  }

  /* We loop forever here. */
  while(1) {
    /* Wait until the other protothread has set its flag. */
    PT_WAIT_UNTIL(1000);
    printf("Protothread 2 running %d\n",i++);

    if (i == 6)
    {
        // PT_RESTART();
        PT_EXIT();
    }
    
    /* And we loop. */
  }
  PT_END();
}


void TaskTest(void)
{
    PT_TASK_REG(0,PT_TASK_TEST);
    PT_TASK_REG(9,PT_TASK_TEST2);
}


