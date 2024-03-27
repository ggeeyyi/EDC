/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "zigbee_edc25.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int count;
int count1;
int count2;
int count3;
float YAW;
float startYAW;
int state = 0;
pid_str pidstr;
pid_str pidstr1;
pid_str pidstr2;
pid_str pidstr3;
#define PID_MIN -400
#define PID_MAX 400
#define LENGTH 8
int dir;
extern int printf_port;
void runrunrun();

uint8_t u1_RX_Buf[6] = {0};

// 定义全局变量
typedef struct
{
    int x;
    int y;
} Position;
Position des_pos;
// 定义全局变量
int startFlag = 0;
Position my_home_pos;
Position op_home_pos;
Position my_pos;
Position op_pos;
int map_height[8][8];
uint8_t map[8][8] = {
    0,
    2,
    0,
    1,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    1,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    1,
    0,
    0,
    0,
    0,
    0,
    2,
    0,
    0,
};
// 可调参数
int stole_height = 3;
int stole_wool_num = 8;
int stole_step = 8;
int move_wool_num = 16;
int mine_wool_num = 16;
int safe_home_height = 5;
int agility_limit = 20;
int strength_limit = 20;
int health_limit = 5;
int health_height_limit = 35;

// 地图信息
// 需要调控好顺序，靠近自己这方的放在前后
int gold_num;
int dia_num;
int iron_num;
bool gold_judge = true;
bool dia_judge = true;
bool iron_judge = true;
// 在全局作用域内声明指针并用NULL初始化
Position *gold_pos_list = NULL;
Position *dia_pos_list = NULL;
Position *iron_pos_list = NULL;
// 函数声明
//  辅助函数
//  将chunk_id转换为坐标
//  设定目标位置
void Destination();
void Destination2();
void DestinationSet(int x, int y);
// 获得目标位置
Position_edc25 DestinationGet();
// 启动小车
void StartMoving();
// 停止小车
void StopMoving();
void getHeightAllChunks();
int getHeightOfChunk(int chunk_id);
void getMyPosition(Position *my_pos);
void getMyOpponentPosition(Position *op_pos);
Position_edc25 chunk_id_to_pos(int chunk_id);
// 将坐标转换为chunk_id
int pos_to_chunk_id(Position pos);
// 计算两点之间的曼哈顿距离
int get_distance(Position pos1, Position pos2);
// 确定己方以及对方家的位置，初始化时完成
void get_home_pos();
// 释放分配的内存
void freePosPath(Position_edc25 *pos_path);
// 回溯法生成所有路径
void backtrack(char **paths, int *pathCount, char *path, int left_steps, int up_steps, int index);
char **generatePaths(int x, int y, int *pathCount);
Position *find_path(Position start, Position end, int *wool_need);
bool track(Position my_pos, Position des_pos, int wool_num);
void attack();
// 黄金矿
int gold_mining();
// 铁矿
int iron_mining();
// 偷家函数
bool steal();
// 死亡循迹函数
bool move_to_pos(Position my_home_pos);
// 死亡函数
void die();
void turn_left();
void turn_right();
void moveOneBlock(Position now, Position target);
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();
    MX_USART3_UART_Init();
    HAL_UART_Receive_DMA(&huart1, u1_RX_Buf, 1);
    /* USER CODE BEGIN 2 */
    jy62_Init(&huart3);
    zigbee_Init(&huart2);
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    pidstr.Kd = 70;
    pidstr.Ki = 7;
    pidstr.Kp = 40;
    pidstr.lr = 0;
    pidstr.sum = 0;
    pidstr1.Kd = 70;
    pidstr1.Ki = 7;
    pidstr1.Kp = 40;
    pidstr1.lr = 0;
    pidstr1.sum = 0;
    pidstr2.Kd = 70;
    pidstr2.Ki = 7;
    pidstr2.Kp = 40;
    pidstr2.lr = 0;
    pidstr2.sum = 0;
    pidstr3.Kd = 70;
    pidstr3.Ki = 7;
    pidstr3.Kp = 40;
    pidstr3.lr = 0;
    pidstr3.sum = 0;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    SetBaud(115200);
    SetHorizontal();
    InitAngle();
    Calibrate();
    SleepOrAwake();
    uint8_t hp = 0;
    uint8_t Aglity = 0;
    uint8_t count = 0;
    uint8_t time = 0;
    Position_edc25 Pos;
    Position_edc25 OpponentPos;
    uint8_t Height[64];
    gold_pos_list = (Position *)malloc(gold_num * sizeof(Position));
    dia_pos_list = (Position *)malloc(dia_num * sizeof(Position));
    iron_pos_list = (Position *)malloc(iron_num * sizeof(Position));
    bool judge;
    bool first = true;
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        // __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
        // __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
        // __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
        // __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 0);
        printf_port = 1;
        float roll = GetRoll();
        float pitch = GetPitch();
        float yaw = GetYaw();
        YAW = yaw;
        printf("ROW: %f, PITCH:%f, YAW:%f\r\n", roll, pitch, yaw);
        hp = getHealth();
        time = getGameTime();
        Aglity = getAglity();
        count = getWoolCount();
        if (time != 0 && time < 2000)
        {
            printf("time:%d\r\n", time);
            printf("hp:%d\r\n", hp);
            printf("Aglity:%d\r\n", Aglity);
        }
        while (getGameStage() == 0)
        {
            my_home_pos.x = 0;
            my_home_pos.y = 0;
        }

        // 游戏阶段
        while (getGameStage() == 1)
        {
            Position_edc25 tp_pos;
            getPosition(&tp_pos);
            my_home_pos.x = tp_pos.posx;
            my_home_pos.y = tp_pos.posy;
            move_to_pos(my_home_pos);
        }
        // trade_id(1);
        // HAL_Delay(10000);
        // turn_left();
        // DestinationSet(0, 1);
        // }
        // getPosition(&Pos);
        // if (startFlag == 1)
        // {
        //     int countYellow = 0;
        //     int countBlue = 0;
        //     for (int i = 0; i < 8; i++)
        //     {
        //         for (int j = 0; j < 8; j++)
        //         {
        //             if (map[i][j] == 1)
        //             {
        //                 Position pos = {i, j};
        //                 gold_pos_list[countYellow] = pos;
        //             }
        //             if (map[i][j] == 2)
        //             {
        //                 Position pos = {i, j};
        //                 dia_pos_list[countBlue] = pos;
        //             }
        //         }
        //     }

        //     以下为策略代码

        //     while (getGameStage() == 1)
        //     {
        //         die();
        //         超过10分钟，进入对抗阶段
        //         if (getGameTime() >= 600)
        //         {
        //             break;
        //         }
        //         能偷家就偷
        //         if (steal(map_height))
        //         {
        //             getHeightAllChunks();
        //             getMyPosition(&my_pos);
        //             judge = track(my_pos, my_home_pos, getWoolCount());
        //             if (judge == false)
        //             {
        //                 move_to_pos(my_home_pos);
        //             }
        //             continue;
        //         }
        //         前期采矿
        //         if (getWoolCount() < move_wool_num)
        //         {
        //             if (first)
        //             {
        //                 dia_mining_first();
        //                 first = false;
        //             }
        //             else
        //             {
        //                 dia_mining();
        //             }
        //             getHeightAllChunks();
        //             getMyPosition(&my_pos);
        //             judge = track(my_pos, my_home_pos, getWoolCount());
        //             if (judge == false)
        //             {
        //                 move_to_pos(my_home_pos);
        //             }
        //             while (getEmeraldCount() >= 2)
        //             {
        //                 trade_id(3);
        //             }
        //             while (getHeightOfChunk(pos_to_chunk_id(my_home_pos)) <= safe_home_height && getWoolCount() > 0)
        //             {
        //                 place_block_id(pos_to_chunk_id(my_home_pos));
        //             }
        //             gold_mining();
        //             getHeightAllChunks();
        //             getMyPosition(&my_pos);
        //             judge = track(my_pos, my_home_pos, getWoolCount());
        //             if (judge == false)
        //             {
        //                 move_to_pos(my_home_pos);
        //             }
        //             while (getEmeraldCount() >= 2)
        //             {
        //                 trade_id(3);
        //             }
        //             while (getHeightOfChunk(pos_to_chunk_id(my_home_pos)) <= safe_home_height && getWoolCount() > 0)
        //             {
        //                 place_block_id(pos_to_chunk_id(my_home_pos));
        //             }
        //         }
        //         后期采矿
        //         else
        //         {
        //             dia_mining();
        //             gold_mining();
        //             getHeightAllChunks();
        //             getMyPosition(&my_pos);
        //             judge = track(my_pos, my_home_pos, getWoolCount());
        //             if (judge == false)
        //             {
        //                 move_to_pos(my_home_pos);
        //             }
        //             while (getHeightOfChunk(pos_to_chunk_id(my_home_pos)) <= safe_home_height && getWoolCount() > 0)
        //             {
        //                 place_block_id(pos_to_chunk_id(my_home_pos));
        //             }
        //             while (getEmeraldCount() >= 32 && getAglity() < agility_limit)
        //             {
        //                 trade_id(0);
        //             }
        //             while (getEmeraldCount() >= 64 && getStrength() < strength_limit)
        //             {
        //                 trade_id(2);
        //             }
        //             while (getEmeraldCount() >= 32 && getMaxHealth() < health_height_limit)
        //             {
        //                 trade_id(4);
        //             }
        //         }
        //     }

        //     战斗
        //     while (getGameStage() == 1)
        //     {
        //         if (getHealth() < 5)
        //         {
        //             attack();
        //             getHeightAllChunks();
        //             getMyPosition(&my_pos);
        //             judge = track(my_pos, my_home_pos, getWoolCount());
        //             if (judge == false)
        //             {
        //                 StopMoving();
        //                 continue;
        //             }
        //             while (getEmeraldCount() >= 32)
        //             {
        //                 trade_id(1);
        //             }
        //             while (getEmeraldCount() >= 64 && getStrength() < strength_limit)
        //             {
        //                 trade_id(2);
        //             }
        //             while (getEmeraldCount() >= 32 && getMaxHealth() < health_height_limit)
        //             {
        //                 trade_id(4);
        //             }
        //         }
        //         攻击不到对方，窝家里
        //         getHeightAllChunks();
        //         getMyPosition(&my_pos);
        //         getMyOpponentPosition(&op_pos);
        //         int judge = track(my_pos, op_pos, getWoolCount());
        //         if (judge == 0)
        //         {
        //             StopMoving();
        //             attack();
        //             continue;
        //         }
        //         attack();
        //     }

        //     结束
        //     while (getGameStage() == 2)
        //     {
        //         StopMoving();
        //     }
        // }
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

void Code1()
{
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == 0) // 判断A为下降沿
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == 0) // 正转
        {
            count += 1;
        }
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == 1) // 反转
        {
            count -= 1;
        }
    }
    else
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == 0) // 正转
        {
            count -= 1;
        }
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == 1) // 反转
        {
            count += 1;
        }
    }
}
void Code0()
{
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == 0) // 判断A为下降沿
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == 0) // 正转
        {
            count -= 1;
        }
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == 1) // 反转
        {
            count += 1;
        }
    }
    else
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == 0) // 正转
        {
            count += 1;
        }
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == 1) // 反转
        {
            count -= 1;
        }
    }
}
void Code2()
{
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == 0) // 判断A为下降沿
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 0) // 正转
        {
            count1 += 1;
        }
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 1) // 反转
        {
            count1 -= 1;
        }
    }
    else
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 0) // 正转
        {
            count1 -= 1;
        }
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 1) // 反转
        {
            count1 += 1;
        }
    }
}
void Code3()
{
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == 0) // 判断A为下降沿
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 0) // 正转
        {
            count1 -= 1;
        }
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 1) // 反转
        {
            count1 += 1;
        }
    }
    else
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 0) // 正转
        {
            count1 += 1;
        }
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 1) // 反转
        {
            count1 -= 1;
        }
    }
}
void code5()
{
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == 0) // 判断A为下降沿
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == 0) // 正转
        {
            count2 += 1;
        }
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == 1) // 反转
        {
            count2 -= 1;
        }
    }
    else
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == 0) // 正转
        {
            count2 -= 1;
        }
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == 1) // 反转
        {
            count2 += 1;
        }
    }
}
void code6()
{
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == 0) // 判断A为下降沿
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == 0) // 正转
        {
            count2 -= 1;
        }
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == 1) // 反转
        {
            count2 += 1;
        }
    }
    else
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == 0) // 正转
        {
            count2 += 1;
        }
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == 1) // 反转
        {
            count2 -= 1;
        }
    }
}
void code7()
{
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) == 0) // 判断A为下降沿
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == 0) // 正转
        {
            count3 += 1;
        }
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == 1) // 反转
        {
            count3 -= 1;
        }
    }
    else
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == 0) // 正转
        {
            count3 -= 1;
        }
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == 1) // 反转
        {
            count3 += 1;
        }
    }
}
void code8()
{
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) == 0) // 判断A为下降沿
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == 0) // 正转
        {
            count3 -= 1;
        }
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == 1) // 反转
        {
            count3 += 1;
        }
    }
    else
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == 0) // 正转
        {
            count3 += 1;
        }
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == 1) // 反转
        {
            count3 -= 1;
        }
    }
}

float pid(pid_str *p, float dr)
{
    if (p->cset > 4)
        p->cset = 4;
    if (p->cset < -4)
        p->cset = -4;
    p->sum = p->sum + dr;
    if (p->sum > 500 || p->sum < -500)
        p->sum = 0;
    float pwm = p->Kp * dr + p->Ki * p->sum + p->Kd * (dr - p->lr);
    p->lr = dr;
    if (pwm >= PID_MAX)
        return PID_MAX;
    else if (pwm <= PID_MIN)
        return PID_MIN;
    return pwm;
}

void runrunrun()
{
    float cnow = count / 10.592;
    float cnow1 = count1 / 10.602;
    float cnow2 = count2 / 10.647;
    float cnow3 = count3 / 10.61;
    count = 0;
    count1 = 0;
    count2 = 0;
    count3 = 0;
    float dr = pidstr.cset - cnow;
    float dr1 = pidstr1.cset - cnow1;
    float dr2 = pidstr2.cset - cnow2;
    float dr3 = pidstr3.cset - cnow3;
    float pwm = pid(&pidstr, dr);
    float pwm1 = pid(&pidstr1, dr1);
    float pwm2 = pid(&pidstr2, dr2);
    float pwm3 = pid(&pidstr3, dr3);
    if (pidstr.cset > 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pwm);
    }
    if (pidstr.cset < 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pwm);
    }
    if (pidstr.cset == 0)
    {
        pidstr.lr = 0;
        pidstr.sum = 0;
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
    }
    if (pidstr1.cset > 0)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, pwm1);
    }
    if (pidstr1.cset < 0)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, pwm1);
    }
    if (pidstr1.cset == 0)
    {
        pidstr1.lr = 0;
        pidstr1.sum = 0;
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
    }
    if (pidstr2.cset > 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwm2);
    }
    if (pidstr2.cset < 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwm2);
    }
    if (pidstr2.cset == 0)
    {
        pidstr2.lr = 0;
        pidstr2.sum = 0;
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
    }
    if (pidstr3.cset > 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwm3);
    }
    if (pidstr3.cset < 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwm3);
    }
    if (pidstr3.cset == 0)
    {
        pidstr3.lr = 0;
        pidstr3.sum = 0;
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 0);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_2)
    {
        Code1();
    }
    if (GPIO_Pin == GPIO_PIN_3)
    {
        Code0();
    }
    if (GPIO_Pin == GPIO_PIN_0)
    {
        Code2();
    }
    if (GPIO_Pin == GPIO_PIN_1)
    {
        Code3();
    }
    if (GPIO_Pin == GPIO_PIN_10)
    {
        code5();
    }
    if (GPIO_Pin == GPIO_PIN_15)
    {
        code6();
    }
    if (GPIO_Pin == GPIO_PIN_11)
    {
        code7();
    }
    if (GPIO_Pin == GPIO_PIN_12)
    {
        code8();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&htim2))
    {
        printf_port = 1;
        runrunrun();
        // printf("count = %d\r\n", count / 4);
        // printf("count1 = %d\r\n", count1 / 4);
        // printf("count2 = %d\r\n", count2 / 4);
        // printf("count3 = %d\r\n", count3 / 4);
    }
}
void forward()
{
    pidstr.cset = 1.5;
    pidstr1.cset = -1.5;
    pidstr2.cset = 1.5;
    pidstr3.cset = -1.5;
}
void backward()
{
    pidstr.cset = -1.5;
    pidstr1.cset = 1.5;
    pidstr2.cset = -1.5;
    pidstr3.cset = 1.5;
}
void stop()
{
    pidstr.cset = 0;
    pidstr1.cset = 0;
    pidstr2.cset = 0;
    pidstr3.cset = 0;
}
void rightward()
{
    pidstr.cset = 1.5;
    pidstr1.cset = 1.5;
    pidstr2.cset = 1.5;
    pidstr3.cset = 1.5;
}
void leftward()
{
    pidstr.cset = -1.5;
    pidstr1.cset = -1.5;
    pidstr2.cset = -1.5;
    pidstr3.cset = -1.5;
}
void Destination()
{
    HAL_Delay(1000);
    place_block_id(1);
    forward();
    Position_edc25 temp; // 用于存储当前位置
    getPosition(&temp);  // 获取当前位置
    while (temp.posx < 1.4)
    {
        getPosition(&temp); // 获取当前位置
    }
    stop();
    HAL_Delay(500);

    backward();
    getPosition(&temp); // 获取当前位置
    while (temp.posx >= 0.8)
    {
        getPosition(&temp); // 获取当前位置
    }
    stop();
    HAL_Delay(1000);
    for (int i = 0; i < 5; i++)
    {
        trade_id(3);
        HAL_Delay(50);
    }
    HAL_Delay(1000);

    forward();
    getPosition(&temp); // 获取当前位置
    while (temp.posx < 1.4)
    {
        getPosition(&temp); // 获取当前位置
    }
    stop();
    HAL_Delay(500);

    place_block_id(2);
    forward();
    getPosition(&temp); // 获取当前位置
    while (temp.posx < 2.4)
    {
        getPosition(&temp); // 获取当前位置
    }
    stop();
    HAL_Delay(500);

    place_block_id(3);
    forward();
    getPosition(&temp); // 获取当前位置
    while (temp.posx < 3.4)
    {
        getPosition(&temp); // 获取当前位置
    }
    stop();
    HAL_Delay(500);

    place_block_id(11);
    rightward();
    getPosition(&temp); // 获取当前位置
    while (temp.posy < 1.5)
    {
        getPosition(&temp); // 获取当前位置
    }
    stop();
    HAL_Delay(500);

    leftward();
    getPosition(&temp); // 获取当前位置
    while (temp.posy >= 0.5)
    {
        getPosition(&temp); // 获取当前位置
    }
    stop();
    HAL_Delay(500);

    backward();
    getPosition(&temp); // 获取当前位置
    while (temp.posx >= 1.8)
    {
        getPosition(&temp); // 获取当前位置
    }
    stop();
    HAL_Delay(500);

    backward();
    getPosition(&temp); // 获取当前位置
    while (temp.posx >= 0.8)
    {
        getPosition(&temp); // 获取当前位置
    }
    stop();
    HAL_Delay(500);

    trade_id(1);

    while (1)
    {
        if (getGameTime() > 200)
            break;
        if (getGameTime() <= 200)
        {
            if (getHeightOfChunk(1) == 0)
            {
                place_block_id(1);
            }
            forward();
            getPosition(&temp); // 获取当前位置
            while (temp.posx < 1.4)
            {
                getPosition(&temp); // 获取当前位置
            }
            stop();
            HAL_Delay(500);

            if (getHeightOfChunk(2) == 0)
            {
                place_block_id(2);
            }
            forward();
            getPosition(&temp); // 获取当前位置
            while (temp.posx < 2.4)
            {
                getPosition(&temp); // 获取当前位置
            }
            stop();
            HAL_Delay(500);

            if (getHeightOfChunk(3) == 0)
            {
                place_block_id(3);
            }
            forward();
            getPosition(&temp); // 获取当前位置
            while (temp.posx < 3.4)
            {
                getPosition(&temp); // 获取当前位置
            }
            stop();
            HAL_Delay(500);

            if (getHeightOfChunk(11) == 0)
            {
                place_block_id(11);
            }
            rightward();
            getPosition(&temp); // 获取当前位置
            while (temp.posy < 1.5)
            {
                getPosition(&temp); // 获取当前位置
            }
            stop();
            HAL_Delay(500);

            leftward();
            getPosition(&temp); // 获取当前位置
            while (temp.posy >= 0.5)
            {
                getPosition(&temp); // 获取当前位置
            }
            stop();
            HAL_Delay(500);

            backward();
            getPosition(&temp); // 获取当前位置
            while (temp.posx >= 1.8)
            {
                getPosition(&temp); // 获取当前位置
            }
            stop();
            HAL_Delay(500);

            backward();
            getPosition(&temp); // 获取当前位置
            while (temp.posx >= 0.8)
            {
                getPosition(&temp); // 获取当前位置
            }
            stop();
            HAL_Delay(500);

            trade_id(1);
        }
    }
}
// place_block_id(11);
// rightward();
// getPosition(&temp); // 获取当前位置
// while (temp.posy < 1.5)
// {
//     getPosition(&temp); // 获取当前位置
// }
// stop();
// HAL_Delay(500);

// place_block_id(19);
// rightward();
// getPosition(&temp); // 获取当前位置
// while (temp.posy < 2.5)
// {
//     getPosition(&temp); // 获取当前位置
// }
// stop();
// HAL_Delay(500);

// place_block_id(18);
// backward();
// getPosition(&temp); // 获取当前位置
// while (temp.posx >= 2.8)
// {
//     getPosition(&temp); // 获取当前位置
// }
// stop();
// HAL_Delay(500);

// place_block_id(17);
// backward();
// getPosition(&temp); // 获取当前位置
// while (temp.posx >= 1.8)
// {
//     getPosition(&temp); // 获取当前位置
// }
// stop();
// HAL_Delay(500);

// place_block_id(16);
// backward();
// getPosition(&temp); // 获取当前位置
// while (temp.posx >= 0.8)
// {
//     getPosition(&temp); // 获取当前位置
// }
// stop();
// HAL_Delay(500);

// place_block_id(8);
// leftward();
// getPosition(&temp); // 获取当前位置
// while (temp.posy >= 0.5)
// {
//     getPosition(&temp); // 获取当前位置
// }
// stop();
// HAL_Delay(500);

void Destination2()
{
    place_block_id(55);
    forward();
    Position_edc25 temp; // 用于存储当前位置
    getPosition(&temp);  // 获取当前位置
    while (temp.posy > 6.4)
    {
        getPosition(&temp); // 获取当前位置
    }
    stop();
    HAL_Delay(500);

    backward();
    getPosition(&temp); // 获取当前位置
    while (temp.posy <= 6.8)
    {
        getPosition(&temp); // 获取当前位置
    }
    for (int i = 0; i < 5; i++)
    {
        trade_id(3);
    }
    HAL_Delay(500);

    forward();
    getPosition(&temp); // 获取当前位置
    while (temp.posy > 6.4)
    {
        getPosition(&temp); // 获取当前位置
    }
    stop();
    HAL_Delay(500);

    place_block_id(47);
    forward();
    getPosition(&temp); // 获取当前位置
    while (temp.posy > 5.4)
    {
        getPosition(&temp); // 获取当前位置
    }
    stop();
    HAL_Delay(500);

    place_block_id(39);
    forward();
    getPosition(&temp); // 获取当前位置
    while (temp.posy > 4.4)
    {
        getPosition(&temp); // 获取当前位置
    }
    stop();
    HAL_Delay(500);

    backward();
    getPosition(&temp); // 获取当前位置
    while (temp.posy < 6.8)
    {
        getPosition(&temp); // 获取当前位置
    }
    stop();
    HAL_Delay(500);

    backward();
    getPosition(&temp); // 获取当前位置
    while (temp.posy < 7.8)
    {
        getPosition(&temp); // 获取当前位置
    }
    stop();
    HAL_Delay(500);

    while (1)
    {
        if (getGameTime() > 600)
            break;
        if (getGameTime() < 600)
        {
            if (getHeightOfChunk(55) == 0)
            {
                place_block_id(55);
            }
            forward();
            Position_edc25 temp; // 用于存储当前位置
            getPosition(&temp);  // 获取当前位置
            while (temp.posy > 6.4)
            {
                getPosition(&temp); // 获取当前位置
            }
            stop();
            HAL_Delay(500);

            if (getHeightOfChunk(47) == 0)
            {
                place_block_id(47);
            }
            forward();
            getPosition(&temp); // 获取当前位置
            while (temp.posy > 5.4)
            {
                getPosition(&temp); // 获取当前位置
            }
            stop();
            HAL_Delay(500);

            if (getHeightOfChunk(39) == 0)
            {
                place_block_id(39);
            }
            forward();
            getPosition(&temp); // 获取当前位置
            while (temp.posx > 4.4)
            {
                getPosition(&temp); // 获取当前位置
            }
            stop();
            HAL_Delay(500);

            backward();
            getPosition(&temp); // 获取当前位置
            while (temp.posy < 6.8)
            {
                getPosition(&temp); // 获取当前位置
            }
            stop();
            HAL_Delay(500);

            backward();
            getPosition(&temp); // 获取当前位置
            while (temp.posy < 7.8)
            {
                getPosition(&temp); // 获取当前位置
            }
            stop();
            HAL_Delay(500);
        }
    }
}

void turn_right()
{
    startYAW = YAW;
    pidstr.cset += 1.5;
    pidstr1.cset -= 1.5;
    pidstr2.cset -= 1.5;
    pidstr3.cset += 1.5;
    while (!((startYAW <= 270 && YAW >= startYAW + 90) || (startYAW >= 270 && YAW >= startYAW - 270 && YAW < 270)))
    {
        YAW = GetYaw();
    }
    pidstr.cset = 0;
    pidstr1.cset = 0;
    pidstr2.cset = 0;
    pidstr3.cset = 0;
    dir++;
}
void turn_left()
{
    startYAW = YAW;
    pidstr.cset -= 1.5;
    pidstr1.cset += 1.5;
    pidstr2.cset += 1.5;
    pidstr3.cset -= 1.5;
    while (!((startYAW <= 270 && YAW >= startYAW + 90) || (startYAW >= 270 && YAW >= startYAW - 270 && YAW < 270)))
    {
        YAW = GetYaw();
    }
    pidstr.cset = 0;
    pidstr1.cset = 0;
    pidstr2.cset = 0;
    pidstr3.cset = 0;
    HAL_Delay(10000);
}
void moveOneBlock(Position now, Position target)
{
    pidstr.cset += 1.5;
    pidstr1.cset -= 1.5;
    pidstr2.cset += 1.5;
    pidstr3.cset -= 1.5;
    Position_edc25 temp; // 用于存储当前位置
    getPosition(&temp);  // 获取当前位置
    if (temp.posy < target.y)
    {
        while (temp.posy >= target.y + 0.5) // 当前位置不等于目标位置时
        {
            getPosition(&temp); // 获取当前位置
        }
    }
    else
    {
        while (temp.posy <= target.y + 0.5) // 当前位置不等于目标位置时
        {
            getPosition(&temp); // 获取当前位置
        }
    }
    pidstr.cset = 0;
    pidstr1.cset = 0;
    pidstr2.cset = 0;
    pidstr3.cset = 0;
    HAL_Delay(500);
}
// 获得目标位置
Position_edc25 DestinationGet()
{
    Position_edc25 pos;
    return pos;
}

// 启动小车
void StartMoving()
{
    Position_edc25 temp_pos = DestinationGet();
    Position pos;
    pos.x = temp_pos.posx;
    pos.y = temp_pos.posy;
    des_pos = pos;
    Position_edc25 my_edc_pos;
    getPosition(&my_edc_pos);
    my_pos.x = my_edc_pos.posx;
    my_pos.y = my_edc_pos.posy;
    if (pos.x > my_pos.x)
    {
        // 向右移动
        rightward();
    }
    else if (pos.x < my_pos.x)
    {
        // 向左移动
        leftward();
    }
    else if (pos.y > my_pos.y)
    {
        // 向上移动
        forward();
    }
    else if (pos.y < my_pos.y)
    {
        // 向下移动
        backward();
    }
    if (pos.x == my_pos.x && pos.y == my_pos.y)
    {
        return;
    }
    else
    {
        StartMoving();
    }
    // printf("StartMoving\n");
}

// 停止小车
void StopMoving()
{
    // printf("StopMoving\n");
}
void getHeightAllChunks()
{
    uint8_t height[64];
    getHeightOfAllChunks(height);
    for (int i = 0; i < 64; ++i)
    {
        map_height[i / 8][i % 8] = height[i];
    }
}
int getHeightOfChunk(int chunk_id)
{
    return map_height[chunk_id / 8][chunk_id % 8];
}
void getMyPosition(Position *my_pos)
{
    Position_edc25 temp;
    getPosition(&temp);
    my_pos->x = (int)temp.posx;
    my_pos->y = (int)temp.posy;
}
void getMyOpponentPosition(Position *op_pos)
{
    Position_edc25 temp;
    getPositionOpponent(&temp);
    op_pos->x = (int)temp.posx;
    op_pos->y = (int)temp.posy;
}

Position_edc25 chunk_id_to_pos(int chunk_id)
{
    int x = chunk_id / 8;
    int y = chunk_id % 8;
    Position_edc25 pos = {x, y};
    return pos;
}

// 将坐标转换为chunk_id
int pos_to_chunk_id(Position pos)
{
    int chunk_id = pos.x * 8 + pos.y;
    return chunk_id;
}

void DestinationSet(int x, int y)
{
    des_pos.x = x;
    des_pos.y = y;
}
// 计算两点之间的曼哈顿距离
int get_distance(Position pos1, Position pos2)
{
    int distance = abs(pos1.x - pos2.x) + abs(pos1.y - pos2.y);
    return distance;
}
// 确定己方以及对方家的位置，初始化时完成
void get_home_pos()
{
    getMyPosition(&my_home_pos);
    getMyOpponentPosition(&op_home_pos);
}
// 释放分配的内存
void freePosPath(Position_edc25 *pos_path)
{
    free(pos_path);
}
// 回溯法生成所有路径
void backtrack(char **paths, int *pathCount, char *path, int left_steps, int up_steps, int index)
{
    if (left_steps == 0 && up_steps == 0)
    {
        paths[*pathCount] = (char *)malloc(strlen(path) + 1); // 分配足够的内存
        strcpy(paths[*pathCount], path);                      // 复制字符串到新内存
        (*pathCount)++;
        return;
    }

    if (left_steps > 0)
    {
        path[index] = '0';
        backtrack(paths, pathCount, path, left_steps - 1, up_steps, index + 1);
    }

    if (up_steps > 0)
    {
        path[index] = '1';
        backtrack(paths, pathCount, path, left_steps, up_steps - 1, index + 1);
    }
}

char **generatePaths(int x, int y, int *pathCount)
{
    int totalPaths = 1 << (x + y); // 2^(x+y) paths
    char **paths = (char **)malloc(sizeof(char *) * totalPaths);

    char *path = (char *)malloc(sizeof(char) * (x + y + 1));
    path[x + y] = '\0'; // Null-terminate the path

    *pathCount = 0;
    backtrack(paths, pathCount, path, x, y, 0);

    free(path);
    return paths;
}

Position *find_path(Position start, Position end, int *wool_need)
{
    int x = abs(end.x - start.x);
    int y = abs(end.y - start.y);
    // printf("%d %d\n", x, y);
    Position pos = start;
    Position *pos_path = (Position *)malloc(sizeof(Position) * (x + y));
    int count;
    int least_wool = x + y;
    int pathCount;
    char *best_path = (char *)malloc(sizeof(char) * (x + y + 1));
    char **paths = generatePaths(x, y, &pathCount);
    // printf("pathCount %d\n", pathCount);
    for (int i = 0; i < pathCount; ++i)
    {
        count = 0;
        pos = start;
        // printf("pos %d %d\n", pos.x, pos.y);
        // printf("path %s\n", paths[i]);
        for (int j = 0; j < strlen(paths[i]); ++j)
        {
            if (paths[i][j] == '0' && pos.x < end.x)
            {
                pos.x += 1;
            }
            else if (paths[i][j] == '0' && pos.x > end.x)
            {
                pos.x -= 1;
            }
            else if (paths[i][j] == '1' && pos.y < end.y)
            {
                pos.y += 1;
            }
            else if (paths[i][j] == '1' && pos.y > end.y)
            {
                pos.y -= 1;
            }
            if (map_height[pos.x][pos.y] == 0)
            {
                count++;
                // printf("map_height %d \n", map_height[pos.x][pos.y]);
                // printf("%d %d\n", pos.x, pos.y);
            }
        }
        // printf("count %d\n", count);
        // printf("least %d\n", least_wool);
        if (count <= least_wool)
        {
            least_wool = count;
            // printf("least %d\n", least_wool);
            for (int k = 0; k < strlen(paths[i]); ++k)
            {
                best_path[k] = paths[i][k];
            }
            // printf("best %s\n", best_path);
        }
    }
    *wool_need = least_wool;
    pos = start;
    for (int i = 0; i < strlen(best_path); ++i)
    {
        if (best_path[i] == '0' && pos.x < end.x)
        {
            pos.x += 1;
        }
        else if (best_path[i] == '0' && pos.x > end.x)
        {
            pos.x -= 1;
        }
        else if (best_path[i] == '1' && pos.y < end.y)
        {
            pos.y += 1;
        }
        else if (best_path[i] == '1' && pos.y > end.y)
        {
            pos.y -= 1;
        }
        // printf("%d %d\n", pos.x, pos.y);
        pos_path[i] = pos;
    }

    // 释放分配的内存
    for (int i = 0; i < pathCount; ++i)
    {
        free(paths[i]);
    }
    free(best_path);
    return pos_path;
}
bool track(Position my_pos, Position des_pos, int wool_num)
{
    int wool_need = 16;
    getHeightAllChunks();
    Position *path = find_path(my_pos, des_pos, &wool_need);
    // printf("wool_need %d\n", wool_need);
    // printf("wool_num %d\n", wool_num);
    // 如果所需羊毛数量大于拥有的羊毛数量，返回False
    if (wool_need > wool_num)
    {
        free(path);
        return false;
    }
    // printf("path %d %d\n", path[0].x, path[0].y);
    // 循环遍历路径
    for (int i = 0; i < sizeof(path) / sizeof(path[0]); ++i)
    {
        // 获取区块ID并检查高度
        int chunk_id = pos_to_chunk_id(path[i]);
        if (getHeightOfChunk(chunk_id) == 0)
        {
            place_block_id(pos_to_chunk_id((Position){path[i].x, path[i].y}));
        }
        DestinationSet(path[i].x, path[i].y);
        StartMoving();
        // 等待移动到指定位置
        getMyPosition(&my_pos);
        while (my_pos.x != path[i].x || my_pos.y != path[i].y)
        {
            // usleep(100000);
            getMyPosition(&my_pos);
        }
    }
    // 释放分配的内存
    free(path);
    return true;
}
// 采矿函数
// 钻石矿
int dia_mining()
{
    int num = 0;
    if (dia_judge == true)
    {
        dia_judge = false;
        for (int i = 0; i < sizeof(dia_pos_list) / sizeof(dia_pos_list[0]); ++i)
        {
            Position dia_pos = dia_pos_list[i];
            // printf("%d %d\n", dia_pos.x, dia_pos.y);
            getHeightAllChunks();
            getMyPosition(&my_pos);
            // printf("%d %d\n", my_pos.x, my_pos.y);
            bool judge = track(my_pos, dia_pos, getWoolCount());
            if (judge == false)
            {
                return num;
            }
            num += 1;
            if (getWoolCount() <= mine_wool_num)
            {
                return num;
            }
        }
    }
    else
    {
        dia_judge = true;
        for (int i = sizeof(dia_pos_list) / sizeof(dia_pos_list[0]) - 1; i >= 0; --i)
        {
            Position dia_pos = dia_pos_list[i];
            // printf("%d %d\n", dia_pos.x, dia_pos.y);
            getMyPosition(&my_pos);
            // printf("%d %d\n", my_pos.x, my_pos.y);
            bool judge = track(my_pos, dia_pos, getWoolCount());
            if (judge == false)
            {
                return num;
            }
            num += 1;
            if (getWoolCount() <= mine_wool_num)
            {
                return num;
            }
        }
    }
    return num;
}

// 黄金矿
int gold_mining()
{
    int num = 0;
    if (gold_judge == true)
    {
        gold_judge = false;
        for (int i = 0; i < sizeof(gold_pos_list) / sizeof(gold_pos_list[0]); ++i)
        {
            Position gold_pos = gold_pos_list[i];
            printf("%d %d\n", gold_pos.x, gold_pos.y);
            getHeightAllChunks(map_height);
            getMyPosition(&my_pos);
            printf("%d %d\n", my_pos.x, my_pos.y);
            bool judge = track(my_pos, gold_pos, getWoolCount());
            if (judge == false)
            {
                return num;
            }
            num += 1;
            if (getWoolCount() <= mine_wool_num)
            {
                return num;
            }
        }
    }
    else
    {
        gold_judge = true;
        for (int i = sizeof(gold_pos_list) / sizeof(gold_pos_list[0]) - 1; i >= 0; --i)
        {
            Position gold_pos = gold_pos_list[i];
            // printf("%d %d\n", gold_pos.x, gold_pos.y);
            getMyPosition(&my_pos);
            // printf("%d %d\n", my_pos.x, my_pos.y);
            bool judge = track(my_pos, gold_pos, getWoolCount());
            if (judge == false)
            {
                return num;
            }
            num += 1;
            if (getWoolCount() <= mine_wool_num)
            {
                return num;
            }
        }
    }
    return num;
}

// 铁矿
int iron_mining()
{
    int num = 0;
    if (iron_judge == true)
    {
        iron_judge = false;
        for (int i = 0; i < sizeof(iron_pos_list) / sizeof(iron_pos_list[0]); ++i)
        {
            Position iron_pos = iron_pos_list[i];
            printf("%d %d\n", iron_pos.x, iron_pos.y);
            getHeightAllChunks();
            getMyPosition(&my_pos);
            printf("%d %d\n", my_pos.x, my_pos.y);
            bool judge = track(my_pos, iron_pos, getWoolCount());
            if (judge == false)
            {
                return num;
            }
            num += 1;
            if (getWoolCount() <= mine_wool_num)
            {
                return num;
            }
        }
    }
    else
    {
        iron_judge = true;
        for (int i = sizeof(iron_pos_list) / sizeof(iron_pos_list[0]) - 1; i >= 0; --i)
        {
            Position iron_pos = iron_pos_list[i];
            printf("%d %d\n", iron_pos.x, iron_pos.y);
            getMyPosition(&my_pos);
            printf("%d %d\n", my_pos.x, my_pos.y);
            bool judge = track(my_pos, iron_pos, getWoolCount());
            if (judge == false)
            {
                return num;
            }
            num += 1;
            if (getWoolCount() <= mine_wool_num)
            {
                return num;
            }
        }
    }
    return num;
}

// 偷家函数
bool steal()
{
    // printf("steal\n");
    getMyPosition(&my_pos);
    getMyOpponentPosition(&op_pos);
    // printf("%d %d\n", my_pos.x, my_pos.y);
    // printf("%d %d\n", op_pos.x, op_pos.y);
    if (abs(op_pos.x - op_home_pos.x) + abs(op_pos.y - op_home_pos.y) >= stole_step && getWoolCount() >= get_distance(my_pos, op_home_pos) + stole_wool_num && getHeightOfChunk(pos_to_chunk_id(op_home_pos)) < stole_height)
    {
        // 到达对方家的位置九宫内的位置
        Position attack_pos;
        if (op_home_pos.x == 0 && op_home_pos.y == 0)
        {
            attack_pos.x = 1;
            attack_pos.y = 0;
        }
        else if (op_home_pos.x == 7 && op_home_pos.y == 7)
        {
            attack_pos.x = 6;
            attack_pos.y = 7;
        }
        // printf("%d %d\n", attack_pos.x, attack_pos.y);
        getHeightAllChunks(map_height);
        track(my_pos, attack_pos, getWoolCount());
        int judge_height;
        judge_height = getHeightOfChunk(pos_to_chunk_id(op_home_pos));
        attack_id(pos_to_chunk_id(op_home_pos));
        getMyOpponentPosition(&op_pos);
        while (getHeightOfChunk(pos_to_chunk_id(op_home_pos)) > 0 && abs(op_pos.x - op_home_pos.x) + abs(op_pos.y - op_home_pos.y) > 2)
        {
            int now_height = getHeightOfChunk(pos_to_chunk_id(op_home_pos));
            if (now_height < judge_height)
            {
                judge_height = now_height;
                getMyOpponentPosition(&op_pos);
                continue;
            }
            else
            {
                return false;
            }
        }
        if (getHeightOfChunk(pos_to_chunk_id(op_home_pos)) == 0)
        {
            return true;
        }
    }

    return false;
}

// 攻击函数
void attack()
{
    if (op_pos.x - my_pos.x <= 1 && op_pos.y - my_pos.y <= 1)
    {
        attack_id(pos_to_chunk_id(op_pos));
    }
}

// 死亡循迹函数
bool move_to_pos(Position my_home_pos)
{
    // printf("move_to_pos\n");
    // printf("%d %d\n", my_pos.x, my_pos.y);
    getMyPosition(&my_pos);
    // printf("%d\n", a);
    Position next_pos;
    // printf("%d %d\n", my_pos.x, my_pos.y);
    // printf("%d %d\n", my_home_pos.x, my_home_pos.y);
    while (my_home_pos.x != my_pos.x || my_home_pos.y != my_pos.y)
    {
        if (my_home_pos.x < my_pos.x)
        {
            DestinationSet(my_pos.x - 1, my_pos.y);
            next_pos.x = my_pos.x - 1;
            next_pos.y = my_pos.y;
        }
        else if (my_home_pos.x > my_pos.x)
        {
            DestinationSet(my_pos.x + 1, my_pos.y);
            next_pos.x = my_pos.x + 1;
            next_pos.y = my_pos.y;
        }
        else if (my_home_pos.y < my_pos.y)
        {
            DestinationSet(my_pos.x, my_pos.y - 1);
            next_pos.x = my_pos.x;
            next_pos.y = my_pos.y - 1;
        }
        else if (my_home_pos.y > my_pos.y)
        {
            DestinationSet(my_pos.x, my_pos.y + 1);
            next_pos.x = my_pos.x;
            next_pos.y = my_pos.y + 1;
        }

        StartMoving();

        // 等待移动到指定位置
        while (my_pos.x != next_pos.x || my_pos.y != next_pos.y)
        {
            // HAL_Delay(100);
            getMyPosition(&my_pos);
            // printf("%d %d\n", my_pos.x, my_pos.y);
        }
    }

    // 假设有一个获取当前血量的函数 getHealth
    return true; // 返回 1 表示成功
}

// 死亡函数
void die()
{
    if (getHealth() == 0)
    {
        move_to_pos(my_home_pos);
    }
}

// 钻石矿(首次采矿)
int dia_mining_first()
{
    int init_emerald = getEmeraldCount();
    int num = 0;
    if (dia_judge == true)
    {
        dia_judge = false;
        for (int i = 0; i < sizeof(dia_pos_list) / sizeof(dia_pos_list[0]); ++i)
        {
            Position dia_pos = dia_pos_list[i];
            // printf("%d %d\n", dia_pos.x, dia_pos.y);
            getHeightAllChunks(map_height);
            getMyPosition(&my_pos);
            // printf("%d %d\n", my_pos.x, my_pos.y);
            bool judge = track(my_pos, dia_pos, getWoolCount());
            if (judge == false)
            {
                continue;
            }
            num += 1;
            getMyPosition(&my_pos);
            getMyOpponentPosition(&op_pos);
            // 地图判断出错
            if (getEmeraldCount() < init_emerald + 16 && get_distance(my_pos, op_pos) >= 6)
            {
                move_to_pos(my_home_pos);
                StopMoving();
                while (getGameStage() == 1)
                {
                    attack();
                    while (getEmeraldCount() >= 2 && getWoolCount() <= 3)
                    {
                        trade_id(3);
                    }
                    while (getWoolCount() > 0 && getHeightOfChunk(pos_to_chunk_id(my_home_pos)) <= 3)
                    {
                        place_block_id(pos_to_chunk_id(my_home_pos));
                    }
                    while (getEmeraldCount() >= 32 && getAglity() < agility_limit)
                    {
                        trade_id(0);
                    }
                    while (getEmeraldCount() >= 64 && getStrength() < strength_limit)
                    {
                        trade_id(2);
                    }
                    while (getEmeraldCount() >= 32 && getMaxHealth() < health_height_limit)
                    {
                        trade_id(4);
                    }
                }
            }
            if (getWoolCount() <= mine_wool_num)
            {
                return num;
            }
        }
    }
    return num;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
