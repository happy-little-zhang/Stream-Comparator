#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>             // socket通信相关, 函数原型         int socket(int af, int type, int protocol);
#include <sys/time.h>
#include <linux/can.h>              // Linux CAN 相关
#include <linux/can/raw.h>          // Linux CAN 相关
#include <termio.h>                 // 串口驱动头文件，扫描键盘事件相关
#include <pthread.h>

#include<iostream>
#include<string>
#include<memory>


#include "common.h"
#include "log.h"

#define SPEED_UPDATE_INTERVAL         1                 // 速度更新间隔 1ms

static int s0,s1;                                 // socket通信相关，socket套接字
static struct sockaddr_can addr0,addr1;           // socket通信相关
static struct ifreq ifr0,ifr1;                    // socket通信相关

static int g_attack_command;                // 攻击指令
static pthread_mutex_t mutex_attack_flag = PTHREAD_MUTEX_INITIALIZER;        //互斥线程锁及用宏定义初始化

static vehicle_status_t g_vehicle_status_can0;    // CAN0 通道的车载状态
static vehicle_status_t g_vehicle_status_can1;    // CAN1 通道的车载状态

static pthread_mutex_t mutex_vehicle_status_can0 = PTHREAD_MUTEX_INITIALIZER;        //互斥线程锁及用宏定义初始化,报文入队
static pthread_mutex_t mutex_vehicle_status_can1 = PTHREAD_MUTEX_INITIALIZER;        //互斥线程锁及用宏定义初始化,报文入队

// 实时接收CAN0报文
void *can0_receive(void *param)
{
    int nbytes;
    struct can_frame frame;
    
    memset(&frame, 0, sizeof(struct can_frame));

	struct thread_arg_t *arg = (struct thread_arg_t *)param; 
	
    while(arg->run) 
	{
        nbytes = read(s0, &frame, sizeof(frame));
		
        if(nbytes > 0) 
		{
			// 打印CAN报文
			// can_frame_show(&frame, CAN_CHANNEL_CAN0);

            // 解析报文
            pthread_mutex_lock(&mutex_vehicle_status_can0);
            parser_can_func(frame.can_id, &frame.data[0], &g_vehicle_status_can0);
			pthread_mutex_unlock(&mutex_vehicle_status_can0);
	
        }
        else 
		{
            LOG_DEBUG("no can frame received!");		
			// break;
        }

        //LOG_DEBUG("can0 brake: %lf, acc: %lf\n", g_brake_pedal_can0,  g_accelerator_pedal_can0);
		
		select_sleep_us(10);
    }
    
	pthread_exit(NULL);
	
}


// 实时接收CAN1报文
void *can1_receive(void *param)
{
    int nbytes;
    struct can_frame frame;
    
    memset(&frame, 0, sizeof(struct can_frame));

	struct thread_arg_t *arg = (struct thread_arg_t *)param; 
	
    while(arg->run) 
	{
        nbytes = read(s1, &frame, sizeof(frame));

        if(nbytes > 0) 
		{
			// 打印CAN报文
			//can_frame_show(&frame, CAN_CHANNEL_CAN1);		

            pthread_mutex_lock(&mutex_vehicle_status_can1);
            parser_can_func(frame.can_id, &frame.data[0], &g_vehicle_status_can1);
			pthread_mutex_unlock(&mutex_vehicle_status_can1);	
        }
        else 
		{
            LOG_DEBUG("no can frame received!");
			// break;
        }

        //LOG_DEBUG("can1 brake: %lf, acc: %lf\n", g_brake_pedal_can1,  g_accelerator_pedal_can1);
		
		select_sleep_us(10);
    }
    
	pthread_exit(NULL);
	
}

// 速度更新
int speed_update()
{
	double dvalue = -0.02;        // 假设正常下摩擦力提供加速度为 -0.02
	
	pthread_mutex_lock(&mutex_vehicle_status_can1);
	double brake_pedal = g_vehicle_status_can1.brake_pedal_value;
	double accelerator_pedal = g_vehicle_status_can1.accelerator_pedal_value;
	double vehicle_speed = g_vehicle_status_can1.front_axle_average_speed;
	if(brake_pedal == 0 && accelerator_pedal >= 0)
	{
		dvalue += accelerator_pedal;		  
	}
	else
	{
		dvalue += -brake_pedal;
	}
    double delta_time = SPEED_UPDATE_INTERVAL;

	vehicle_speed = vehicle_speed + dvalue * delta_time/1000.0;
	vehicle_speed = vehicle_speed>250?  250:vehicle_speed;
	vehicle_speed = vehicle_speed<0?  0:vehicle_speed;
	g_vehicle_status_can1.front_axle_average_speed = vehicle_speed;
	
    pthread_mutex_unlock(&mutex_vehicle_status_can1);

    return 0;
}

// ECU 1 设定为同时向两个CAN口发送相同的报文
int can_send_normal(int &count)
{
    if(g_vehicle_status_can0.auto_drive_flag == 0)
    {
        return 0;
    }


    int nbytes0, nbytes1;
    struct can_frame frame;
    
    memset(&frame, 0, sizeof(struct can_frame));

	frame.can_id = 0xB1;
	frame.can_dlc = 8;
	frame.data[0] = 0x00;								  
	frame.data[1] = 0x00;								  
	frame.data[2] = 0x00;
	frame.data[3] = 0x00;
	frame.data[4] = 0x00;
	frame.data[5] = 0x00;
	frame.data[6] = 0x00;
	//frame.data[6] = count;
	frame.data[7] = 0x00;                  

    //pthread_mutex_lock(&mutex_vehicle_status);
    //double vehicle_speed = g_vehicle_speed;
	//pthread_mutex_unlock(&mutex_vehicle_status);
	double vehicle_speed = g_vehicle_status_can1.front_axle_average_speed;
	data_to_can(vehicle_speed, &frame.data[0], 0, 16, (double)(1.0/256.0), 0.0);

    vehicle_status_show(&g_vehicle_status_can1);
		
	//can_frame_show(&frame);
	
	//printf("count: %d\n", count);
	count = (count+1)%256;


	// 发送报文
	nbytes0 = write(s0, &frame, sizeof(frame)); 
	nbytes1 = write(s1, &frame, sizeof(frame)); 
	if(nbytes0 != sizeof(frame) || nbytes1 != sizeof(frame)) 
	{
		if(nbytes0 != sizeof(frame))
		{
			// LOG_DEBUG("Send	can0 frame failed!");
		}
	
		if(nbytes1 != sizeof(frame))
		{
			// LOG_DEBUG("Send	can1 frame failed!");
		}
	
	}

    return 0;
}


// 报文注入攻击
int can_send_injection()
{
    if(g_attack_command != e_attack_inject)
    {
        return 0;
    }

    int nbytes;
    struct can_frame frame;
    
    memset(&frame, 0, sizeof(struct can_frame));

	frame.can_id = 0x0A2;
	frame.can_dlc = 8;
	frame.data[0] = 0x32;
	frame.data[1] = 0x32;								  
	frame.data[2] = 0x32;
	frame.data[3] = 0x32;
	frame.data[4] = 0x32;
	frame.data[5] = 0x32;
	frame.data[6] = 0x32;
	frame.data[7] = 0x00;                    

	// 发送报文
	nbytes = write(s0, &frame, sizeof(frame)); 
	if(nbytes != sizeof(frame)) 
	{
		// LOG_DEBUG("Send frame failed!");
	}
	else
	{
		// LOG_DEBUG("Send frame successful!");
		// can_frame_show(&frame);
	}

	//fprintf(g_file_attack_record, "%ld, %d\n", get_system_us_time(), e_attack_inject);

    return 0;
}


// 初始化资源
int init(void)
{
	int ret;
	
    // 打开设备CAN0, 波特率500K
    system("sudo ip link set can0 type can bitrate 500000");
    system("sudo ifconfig can0 up");
	
	// 打开设备CAN1, 波特率500K
	system("sudo ip link set can1 type can bitrate 500000");
    system("sudo ifconfig can1 up");
    printf("this is a can receive demo\r\n");

    // 创建套接字 1.Create socket
    s0 = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s0 < 0) {
        perror("socket s0 PF_CAN failed");
        return 1;
    }
    s1 = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s1 < 0) {
        perror("socket s1 PF_CAN failed");
        return 1;
    }

    // 2.Specify can device
    strcpy(ifr0.ifr_name, "can0");
    ret = ioctl(s0, SIOCGIFINDEX, &ifr0);
    if (ret < 0) {
        perror("ioctl failed");
        return 1;
    }
    strcpy(ifr1.ifr_name, "can1");
    ret = ioctl(s1, SIOCGIFINDEX, &ifr1);
    if (ret < 0) {
        perror("ioctl failed");
        return 1;
    }


    // 3.Bind the socket to can
    addr0.can_family = AF_CAN;
    addr0.can_ifindex = ifr0.ifr_ifindex;
    ret = bind(s0, (struct sockaddr *)&addr0, sizeof(addr0));
    if (ret < 0) {
        perror("bind can0 failed");
        return 1;
    }
    addr1.can_family = AF_CAN;
    addr1.can_ifindex = ifr1.ifr_ifindex;
    ret = bind(s1, (struct sockaddr *)&addr1, sizeof(addr1));
    if (ret < 0) {
        perror("bind can1 failed");
        return 1;
    }

    // 设置接收超时时间 4.set receive timeout 
    struct timeval rcv_timeout = { 5, 0};   //{s, us}
    setsockopt(s0, SOL_SOCKET, SO_RCVTIMEO, (char*)&rcv_timeout, sizeof( struct timeval));
	setsockopt(s1, SOL_SOCKET, SO_RCVTIMEO, (char*)&rcv_timeout, sizeof( struct timeval));

	g_attack_command = e_attack_initial;
	

	return RET_OK;
	
}

// 释放资源
int deinit(void)
{
	// 关闭套接字和CAN设备 6.Close the socket and can0
    close(s0);
    close(s1);	
    system("sudo ifconfig can0 down");
    system("sudo ifconfig can1 down");
	
    return RET_OK;
}

int main()
{
	int ret;
	
    // 资源初始化
	init();
	
    // 线程参数声明
    struct thread_arg_t can0_rcv_thread_arg;                        // can0报文接收线程
    struct thread_arg_t can1_rcv_thread_arg;                        // can1报文接收线程
    
    // 线程ID声明
    pthread_t can0_rcv_thread_id;                                   // can0报文接收线程号
    pthread_t can1_rcv_thread_id;                                   // can1报文接收线程号    

    // 创建can0报文接收线程
	can0_rcv_thread_arg.run = TRUE;
	ret = pthread_create(&can0_rcv_thread_id, NULL, can0_receive, &can0_rcv_thread_arg);
	if ( ret == 0) 
	{
		printf("create can0_receive thread successful \n");
	} 
	else
	{
		printf("create can0_receive thread failed! \n");
	}
    // 创建can1报文接收线程
	can1_rcv_thread_arg.run = TRUE;
	ret = pthread_create(&can1_rcv_thread_id, NULL, can1_receive, &can1_rcv_thread_arg);
	if ( ret == 0) 
	{
		printf("create can1_receive thread successful \n");
	} 
	else
	{
		printf("create can1_receive thread failed! \n");
	}


	//周期性执行定时任务
	Timer t_send_normal;
	int count = 0;
	t_send_normal.StartTimer(ECU2_SEND_CYCLE, std::bind(can_send_normal, count) );


	//周期性执行定时任务
	Timer t_send_injection;
	t_send_injection.StartTimer(ECU1_SEND_CYCLE, std::bind(can_send_injection) );

    // 周期性根据运动学公式更新速度
    Timer t_speed_update;
	t_speed_update.StartTimer(SPEED_UPDATE_INTERVAL, std::bind(speed_update) );


	// 获取键盘按键,根据键盘按键响应事件	(按q退出)
	while(1)
	{
		int res = scanKeyboard();
		printf("Keyboard_value: %d \n", res);
		
		if(res == 'a')
		{
		    break;
		}
		else
		{
		    pthread_mutex_lock(&mutex_attack_flag);
			// todo
            g_attack_command = res - '0';
			pthread_mutex_unlock(&mutex_attack_flag);

		}
		
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	// 退出报文接收线程
	can0_rcv_thread_arg.run = FALSE;
    pthread_join(can0_rcv_thread_id, NULL);

	// 退出报文接收线程
	can1_rcv_thread_arg.run = FALSE;
    pthread_join(can1_rcv_thread_id, NULL);

	std::cout << "try to expire timer!" << std::endl;
	t_send_normal.Expire();
	t_send_injection.Expire();
	t_speed_update.Expire();

    // 释放资源
	deinit();
	
    return 0;
}
