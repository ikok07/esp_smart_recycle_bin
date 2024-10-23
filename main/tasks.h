//
// Created by kok on 12.10.24.
//

#ifndef TASKS_H
#define TASKS_H

/* --------- CORE 0 --------- */

#define POWER_APP_TASK_PRIORITY          5
#define POWER_APP_TASK_STACK_SIZE       2048
#define POWER_APP_TASK_CORE_ID           0

#define SENSOR_APP_TASK_PRIORITY         4
#define SENSOR_APP_TASK_STACK_SIZE       4096
#define SENSOR_APP_TASK_CORE_ID          0

/* --------- CORE 1 --------- */

#define PWM_APP_TASK_PRIORITY            5
#define PWM_APP_TASK_STACK_SIZE          4096
#define PWM_APP_TASK_CORE_ID             1

#endif //TASKS_H
