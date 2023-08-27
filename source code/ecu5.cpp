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

#include <iostream>
#include <string>
#include <memory>
#include <vector>

#include "common.h"
#include "log.h"
#include "SimpleIni.h"              // 模型参数存储与读取相关库


// CAN通信相关
static int s0,s1;                                 // socket通信相关，socket套接字
static struct sockaddr_can addr0,addr1;           // socket通信相关
static struct ifreq ifr0,ifr1;                    // socket通信相关

//// TCP通信服务端
//static int server_fd;                       // 服务端套接字
//static int client_fd;                       // 客户端套接字 
//static struct sockaddr_in server_addr;     // 服务端地址
//static struct sockaddr_in client_addr;     // 服务端地址

static vector<UINT> g_vec_monitor_id_list;        // 监测器的ID列表
static can_queue_t **g_can_queue;                 // 分布式监测队列
static can_queue_t *g_can_queue_baseline;         // 对比方法监测需要的报文采集队列

static vehicle_status_t g_vehicle_status_can0;    // CAN0 通道的车载状态
static vehicle_status_t g_vehicle_status_can1;    // CAN1 通道的车载状态

static pthread_mutex_t mutex_can_queue = PTHREAD_MUTEX_INITIALIZER;        //互斥线程锁及用宏定义初始化,报文入队
static pthread_mutex_t mutex_vehicle_status_can0 = PTHREAD_MUTEX_INITIALIZER;        //互斥线程锁及用宏定义初始化,报文入队
static pthread_mutex_t mutex_vehicle_status_can1 = PTHREAD_MUTEX_INITIALIZER;        //互斥线程锁及用宏定义初始化,报文入队

static FILE* g_file_detection_record;              // 用于记录一致性检查的检测结果
static FILE* g_file_singal_record;                 // 用于记录两个CAN总线接收的信号值(plot画图需要)
static FILE* g_file_time_interval_record;          // 用于记录两个通道接收相同报文的时间间隔     
static FILE* g_file_baseline_detection_record;     // 用于记录对比方法的检测结果
//static FILE* g_file_attack_record;                 // 用于记录接收的攻击标识

static int* g_attack_flag;                         // socket接收的攻击指令
static pthread_mutex_t mutex_attack_flag = PTHREAD_MUTEX_INITIALIZER;        //互斥线程锁及用宏定义初始化, 攻击标记


// 实时接收CAN0报文并进行报文入队
// 将CAN0作为正常的总线通道，部署对比实验(基于熵、基于汉明距离、基于时间间隔、基于频率、ID矩阵)
void *can0_receive(void *param)
{
    int nbytes;
    struct can_frame frame;
    
    memset(&frame, 0, sizeof(struct can_frame));

	struct thread_arg_t *arg = (struct thread_arg_t *)param; 

	int vec_size = (int)g_vec_monitor_id_list.size();
	long int count[vec_size] = {0};   
	
    while(arg->run) 
	{
        nbytes = read(s0, &frame, sizeof(frame));

        if(nbytes > 0) 
		{
			// 打印CAN报文
			//can_frame_show(&frame, CAN_CHANNEL_CAN0);

            // 解析报文
            pthread_mutex_lock(&mutex_vehicle_status_can0);
            parser_can_func(frame.can_id, &frame.data[0], &g_vehicle_status_can0);
			pthread_mutex_unlock(&mutex_vehicle_status_can0);

            // 报文入队，用于baseline 检测
            can_queue_enqueue(g_can_queue_baseline, frame, CAN_CHANNEL_CAN0);

			// 报文入队，冗余检测队列
			int pos = get_map_id2num(g_vec_monitor_id_list, frame.can_id);
			if(pos != RET_ERROR)
			{
			    pthread_mutex_lock(&mutex_can_queue);
				can_queue_enqueue(g_can_queue[pos], frame, CAN_CHANNEL_CAN0);
				pthread_mutex_unlock(&mutex_can_queue);

				count[pos]++;
			}

        }
        else 
		{
            LOG_DEBUG("no can frame received!");
        }
		
		select_sleep_us(1);
		
    }


    // 辅助打印-各个ID报文接收的数目
    char buffer[1024];
	for(int i = 0; i < vec_size; i++)
	{
	    snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "ID: %0X, count: %ld\n", get_map_num2id(g_vec_monitor_id_list, i), count[i] );
	}
	LOG_DEBUG(YELLOW "\n%s" NONE, buffer);


	pthread_exit(NULL);
	
}


// 实时接收CAN1报文并进行报文入队
void *can1_receive(void *param)
{
    int nbytes;
    struct can_frame frame;
    
    memset(&frame, 0, sizeof(struct can_frame));

	struct thread_arg_t *arg = (struct thread_arg_t *)param; 

	int vec_size = (int)g_vec_monitor_id_list.size();
	long int count[vec_size] = {0}; 

	
    while(arg->run) 
	{
        nbytes = read(s1, &frame, sizeof(frame));

        if(nbytes > 0) 
		{
			// 打印CAN报文
			//can_frame_show(&frame, CAN_CHANNEL_CAN0);

            // 解析报文
            pthread_mutex_lock(&mutex_vehicle_status_can1);
            parser_can_func(frame.can_id, &frame.data[0], &g_vehicle_status_can1);
			pthread_mutex_unlock(&mutex_vehicle_status_can1);

			// 报文入队
			int pos = get_map_id2num(g_vec_monitor_id_list, frame.can_id);
			if(pos != RET_ERROR)
			{
			    pthread_mutex_lock(&mutex_can_queue);
				can_queue_enqueue(g_can_queue[pos], frame, CAN_CHANNEL_CAN1);
				pthread_mutex_unlock(&mutex_can_queue);

				count[pos]++;
			}

        }
        else 
		{
            LOG_DEBUG("no can frame received!");
        }
		
		select_sleep_us(1);
    }

    char buffer[1024];
	for(int i = 0; i < vec_size; i++)
	{
	    snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "ID: %0X, count: %ld\n", get_map_num2id(g_vec_monitor_id_list, i), count[i] );
	}

	LOG_DEBUG(YELLOW "\n%s" NONE, buffer);

	pthread_exit(NULL);
	
}


// 实时检测报文队列
void *monitor_detection(void *param)
{
	struct thread_arg_t *arg = (struct thread_arg_t *)param; 

    long long tp = 0;
	long long tn = 0;
	long long fp = 0;
	long long fn = 0;

    // 检测标记，0-正常 1-异常
	int vec_size = (int)g_vec_monitor_id_list.size();
	 
	int anomaly_flag[vec_size] = {0};

    while(arg->run) 
	{
        int new_frame_flag = 0;                    // 是否有新报文的标志
	
		pthread_mutex_lock(&mutex_can_queue);

		for(int pos = 0; pos < vec_size; pos++)
		{
				
			can_node_t* can_node = NULL;
			can_node = g_can_queue[pos]->front->next;	
			
		    int can_length = g_can_queue[pos]->length;
			if(can_length >= 2)
			{
			    new_frame_flag = 1;
				
				can_node_t* temp_node = can_node->next;
				
				if( can_node_compare(can_node, temp_node) == true )
				{
					if(g_attack_flag[pos] == e_attack_free)
					{
						tn++;
					}
					else if(g_attack_flag[pos] != e_attack_free)
					{
						fn++;
					}

					// 用于记录时间间隔 ms
					//fprintf(g_file_time_interval_record, "%ld\n", abs(can_node->system_time-temp_node->system_time));

					can_queue_dequeue(g_can_queue[pos]);
					can_queue_dequeue(g_can_queue[pos]);

					// 检测标记，0-正常 1-异常
                    anomaly_flag[pos] = 0;     
				}
				else
				{
					LOG_DEBUG(RED "detect anomaly!!!" NONE "\n");
					// can_queue_show(g_can_queue[pos]);

					// 仅检测CAN通道0的报文
					if(can_node->can_channel == CAN_CHANNEL_CAN0)
					{
						if(g_attack_flag[pos] != e_attack_inject)
						{
							fp++;
						}
						else if(g_attack_flag[pos] == e_attack_inject)
						{
							tp++;
						}					    
					}

					can_queue_dequeue(g_can_queue[pos]);

					// 检测标记，0-正常 1-异常
					anomaly_flag[pos] = 1;
				}
				
			}

			
			//LOG_DEBUG("%08X can_queue_length: %d", get_map_num2id(g_vec_monitor_id_list, pos) , can_length); 
		}
		pthread_mutex_unlock(&mutex_can_queue);

		if(new_frame_flag)
		{
			fprintf(g_file_detection_record, "%ld", get_system_us_time());
			for(int i = 0; i < vec_size; i++)
			{
				fprintf(g_file_detection_record, ",%d", anomaly_flag[i]);
			}
			fprintf(g_file_detection_record, "\n"); 		
		}

		select_sleep_us(5);
    }

	
    double detection_rate = (double)tp/(double)(tp+fn);
	double false_alarm_rate = (double)fp/(double)(fp+tn);

    LOG_DEBUG(RED "tp: %ld, tn: %ld, fp: %ld, fn: %ld, detection_rate: %lf, false_alarm: %lf" NONE "\n", 
		tp, tn, fp, fn, detection_rate, false_alarm_rate);


	pthread_exit(NULL);

}

// 对比方法实时检测
void *baseline_detection(void *param)
{
	vector<UINT> vec_can_id;					        // CAN_ID列表
	vector<ID_Profile> id_profile;                      // id轮廓
 	
	CSimpleIniA configurator;                           // 配置文件

    // 初始化参数前继等相关标记
    int last_can_id_valid = INVALID;
	UINT last_can_id = 0;   

    // 打开模型参数文件
    char configuration_path[100] = "";
	snprintf(configuration_path, sizeof(configuration_path), DATA_PATH  "model.init");    
	SI_Error rc = configurator.LoadFile(configuration_path);
	if (rc < 0) 
	{
	    LOG_DEBUG(RED "load init file failed !" NONE);
	}
	else
	{
	    LOG_DEBUG(GREEN "load init file OK !" NONE);	
	}

    int res = RET_ERROR;
    // 从配置文件读取ID轮廓，加载各方法参数 
    res = read_id_profile(id_profile, vec_can_id, configurator);
	if (res == RET_ERROR)
	{
		LOG_DEBUG(RED "read_id_profile failed !" NONE);
		
	    pthread_exit(NULL);
	}
	show_id_profile(id_profile, vec_can_id);

    // 用于计算检测率的变量
    long long tp = 0;
	long long tn = 0;
	long long fp = 0;
	long long fn = 0;

	struct thread_arg_t *arg = (struct thread_arg_t *)param; 

    // 遍历报文采集队列，并进行检测
    while(arg->run) 
	{
		// 检测标记，0-正常 1-异常
		int baseline_methods_num = 3;
		int is_anomaly[baseline_methods_num] = {0}; 

		int can_length = g_can_queue_baseline->length;
		
		// 有新的报文
		if(can_length > 0)
		{
			can_node_t* can_node = NULL;
			can_node = g_can_queue_baseline->front->next;
			
		    int pos = get_map_id2num(vec_can_id, can_node->frame.can_id);

			// 寻找报文ID对应的序号
			int can_id_num = NOT_EXIST;
			can_id_num = get_map_id2num(vec_can_id, can_node->frame.can_id);
			
			if(can_id_num == NOT_EXIST)
			{
			    LOG_DEBUG(RED "unknown frame %08x" NONE, can_node->frame.can_id);
			}
			else
			{
				// 汉明距离检测
				hamming_distance_detect(id_profile[pos], &can_node->frame.data[0], is_anomaly[0]);
			    
				// ID 矩阵检测
				if(last_can_id_valid == VALID)
				{
				    int last_can_id_num = get_map_id2num(vec_can_id, last_can_id);
					
					id_matrix_detect(id_profile[last_can_id_num], can_node->frame.can_id, is_anomaly[1]);
				}
				// 更新上一次报文ID
				last_can_id = can_node->frame.can_id;
				last_can_id_valid = VALID;	

				// 时间间隔检测
				time_interval_detect(id_profile[pos], can_node->system_time, is_anomaly[2]);	

                // 打印检测结果
                LOG_DEBUG("%d, %d, %d", is_anomaly[0], is_anomaly[1], is_anomaly[2]);

				// 检测结果记录
				fprintf(g_file_baseline_detection_record, "%ld", get_system_us_time());
				for(int i = 0; i < baseline_methods_num; i++)
				{
					fprintf(g_file_baseline_detection_record, ",%d", is_anomaly[i]);
				}
				fprintf(g_file_baseline_detection_record, "\n");

			}
			
			//todo...
		
			// 报文采集队列出队
			can_queue_dequeue(g_can_queue_baseline);	  
		}


		select_sleep_us(5);
    }

    //double detection_rate = (double)tp/(double)(tp+fn);
	//double false_alarm_rate = (double)fp/(double)(fp+tn);

    //LOG_DEBUG(RED "tp: %ld, tn: %ld, fp: %ld, fn: %ld, detection_rate: %lf, false_alarm: %lf" NONE "\n", 
	//	tp, tn, fp, fn, detection_rate, false_alarm_rate);


	pthread_exit(NULL);

}


// 对比方法在线训练
void *baseline_training(void *param)
{
	vector<UINT> vec_can_id;					        // CAN_ID列表
	vector<ID_Profile> id_profile;                      // ID轮廓

	CSimpleIniA configurator;
	// 设置配置文件编码模式
	configurator.SetUnicode();

    int can_id_size = (int)vec_can_id.size();

    // 初始化参数前继等相关标记
    int last_can_id_valid = INVALID;
	UINT last_can_id = 0;    
	
	// 初始化前继变量是否有效
    for(int i = 0; i < can_id_size; i++)
    {
		id_profile[i].previous_payload_valid = INVALID;
		id_profile[i].previous_time_valid = INVALID;
    }

    long frame_count = 0;               // 用于统计报文数量

	struct thread_arg_t *arg = (struct thread_arg_t *)param; 

    // 遍历报文采集队列，并进行在线训练
    while(arg->run) 
	{		
	
		
		int can_length = g_can_queue_baseline->length;
			
		// 有新的报文
		if(can_length > 0)
		{
			can_node_t* can_node = NULL;
			can_node = g_can_queue_baseline->front->next;
			
			frame_count++;			
				
			// 寻找报文ID对应的序号
			int can_id_num = NOT_EXIST;
			can_id_num = get_map_id2num(vec_can_id, can_node->frame.can_id);
			
			if(can_id_num == NOT_EXIST)
			{
				// 新CAN_ID入队
				vec_can_id.push_back(can_node->frame.can_id);
				LOG_DEBUG(YELLOW "new id: %x, id_size(%d)" NONE, can_node->frame.can_id, vec_can_id.size());
			
				// ID轮廓数目增加
				ID_Profile temp_id_profile;
				id_profile.push_back(temp_id_profile);			
			
				// 更新CAN_ID_NUM
				can_id_num = get_map_id2num(vec_can_id, can_node->frame.can_id);
			}
			
			// 更新ID转移矩阵
			if(last_can_id_valid == VALID)
			{
				int last_id_num = get_map_id2num(vec_can_id, last_can_id);
			
				int legal_next_num = NOT_EXIST;
				legal_next_num = get_map_id2num(id_profile[last_id_num].legal_next_id, can_node->frame.can_id);
			
				// 避免重复入队
				if(legal_next_num == NOT_EXIST)
				{
					id_profile[last_id_num].legal_next_id.push_back(can_node->frame.can_id);
				}
			}
			last_can_id = can_node->frame.can_id;
			last_can_id_valid = VALID;
			
			
			// 更新汉明距离轮廓
			if(id_profile[can_id_num].previous_payload_valid == VALID)
			{
				// 计算新旧报文负载的汉明距离
				int hamming_dist = calculate_hamming_distance(id_profile[can_id_num].previous_payload, can_node->frame.data);
			
				if(hamming_dist > id_profile[can_id_num].max_hamming_dist)
				{
					id_profile[can_id_num].max_hamming_dist = hamming_dist;
				}
			
				if(hamming_dist < id_profile[can_id_num].min_hamming_dist)
				{
					id_profile[can_id_num].min_hamming_dist = hamming_dist;
				}
			}		
			// 更新存储的新负载
			for(int i = 0; i < FRAME_DLC; i++)
			{
				id_profile[can_id_num].previous_payload[i] = can_node->frame.data[i];
			}
			id_profile[can_id_num].previous_payload_valid = VALID;
			
			
			// 时间间隔参数
			int64_t timestamp = can_node->system_time;
			if(id_profile[can_id_num].previous_time_valid == VALID)
			{
				// 计算新旧报文负载的汉明距离
				int64_t time_interval = timestamp - id_profile[can_id_num].previous_time;
			
				if(time_interval > id_profile[can_id_num].max_time_interval)
				{
					id_profile[can_id_num].max_time_interval = time_interval;
				}
			
				if(time_interval < id_profile[can_id_num].min_time_interval)
				{
					id_profile[can_id_num].min_time_interval = time_interval;
				}
			}		
			// 更新存储的新时间
			id_profile[can_id_num].previous_time = timestamp;
			id_profile[can_id_num].previous_time_valid = VALID;
		
			// 报文采集队列出队
			can_queue_dequeue(g_can_queue_baseline);	  
		}


		select_sleep_us(5);
    }

    // 打印ID轮廓参数
    show_id_profile(id_profile, vec_can_id);

	// 将ID轮廓参数写入模型配置
    save_id_profile(vec_can_id, id_profile, configurator);

    // 保存模型参数文件
    char configuration_path[100] = "";
	snprintf(configuration_path, sizeof(configuration_path), DATA_PATH  "model.init");
	configurator.SaveFile(configuration_path);

	pthread_exit(NULL);

}



// 定时信号记录,同时负责指令发送(CAN通讯控制其他ECU)
int singal_record()
{
	//LOG_DEBUG("auto_drive_flag: %d", g_vehicle_status_can0.auto_drive_flag);


    // 发送原型系统整体控制指令
    if(g_vehicle_status_can0.auto_drive_flag == 0)
    {
        return 0;
    }
	else
	{
		int nbytes;
		struct can_frame frame;
		
		memset(&frame, 0, sizeof(struct can_frame));
		
		// 发送控制台指令
		frame.can_id = 0xE1;
		frame.can_dlc = 8;
		frame.data[0] = 0x00;                          // 自动驾驶使能
		frame.data[1] = 0x00;								  
		frame.data[2] = 0x00;
		frame.data[3] = 0x00;
		frame.data[4] = 0x00;
		frame.data[5] = 0x00;
		frame.data[6] = 0x00;
		frame.data[7] = 0x00;							

        data_to_can((double)g_vehicle_status_can0.auto_drive_flag, &frame.data[0], 0, 16, 1.0, 0.0);                 
		
		// 发送报文
		nbytes = write(s0, &frame, sizeof(frame)); 
		if(nbytes != sizeof(frame)) 
		{
			//LOG_DEBUG("Send frame failed!");
		}
		else
		{
			// LOG_DEBUG("Send frame successful!");
			// can_frame_show(&frame);
		}

	}

    // 记录信号值
    //pthread_mutex_lock(&mutex_vehicle_status_can0);
    fprintf(g_file_singal_record, "%ld,%lf,%lf,%lf,%lf", get_system_us_time(),
		g_vehicle_status_can0.front_axle_average_speed, 
			g_vehicle_status_can0.accelerator_pedal_value, 
		    	g_vehicle_status_can0.brake_pedal_value, 
		        	g_vehicle_status_can0.bms_soc);	
	//pthread_mutex_unlock(&mutex_vehicle_status_can0);

    //pthread_mutex_lock(&mutex_vehicle_status_can1);
    fprintf(g_file_singal_record, ",%lf,%lf,%lf,%lf\n", 
		g_vehicle_status_can1.front_axle_average_speed, 
		    g_vehicle_status_can1.accelerator_pedal_value, 
		        g_vehicle_status_can1.brake_pedal_value, 
		        	g_vehicle_status_can1.bms_soc);	
	//pthread_mutex_unlock(&mutex_vehicle_status_can1);


    return 0;
}


//// 实时接收攻击信号
//void* socket_recv_attack_flag(void *param)
//{
//    struct thread_arg_t *arg = (struct thread_arg_t *)param; 
//
//	char recvbuffer[1024] = {0};
//
//	while(arg->run)
//	{
//        //获取客户端的攻击时间数据
//        int lens = read(client_fd, recvbuffer, sizeof(recvbuffer));
//		
//		// 解析攻击目标和攻击标记
//		if(lens>0)
//        {
//            int64_t last_time = 0;
//			UINT target_id = 0;
//			int recv_flag = 0;
//            // LOG_DEBUG("%s", recvbuffer);
//            
//			// 使用分割函数
//			char *ptr = NULL;
//			char *saveptr = NULL;
//			char *endptr = NULL;
//			ptr = strtok_r(recvbuffer, ",", &saveptr);
//
//			if(ptr != NULL)
//			{
//			    // 攻击时间
//				last_time = strtol(ptr, &endptr, 10);		   // 参数10表示转化为10进制
//				ptr = strtok_r(NULL, ",", &saveptr);
//            	//LOG_DEBUG("time: %ld", last_time);			
//			}
//
//			if(ptr != NULL)
//			{
//			    // 攻击报文id
//				target_id = strtol(ptr, &endptr, 16);
//				ptr = strtok_r(NULL, ",", &saveptr);
//            	//LOG_DEBUG("id: %0X", target_id);			
//			}
//
//			if(ptr != NULL)
//			{
//			    // 攻击标识
//				recv_flag = strtol(ptr, &endptr, 10);
//            	//LOG_DEBUG("flag: %d", recv_flag);			
//			}
//
//
//            int pos = get_map_id2num(g_vec_monitor_id_list, target_id);
//			if(pos>=0)
//			{
//            	//LOG_DEBUG("recv client data: %s", recvbuf);
//            	pthread_mutex_lock(&mutex_attack_flag);
//				g_attack_flag[pos] = recv_flag;          // atoi 字符串转整数
//				pthread_mutex_unlock(&mutex_attack_flag);			
//			}
//        
//        }
//		else if(lens==0)
//        {
//            //表示客户端断开连接,攻击标记置为0
//            //LOG_DEBUG("client closed...");
//            pthread_mutex_lock(&mutex_attack_flag);
//			memset(g_attack_flag, 0, sizeof(int) * g_vec_monitor_id_list.size());
//			pthread_mutex_unlock(&mutex_attack_flag);                    
//        }
//		else
//		{
//		    //LOG_DEBUG("no data recv...");
//		}	    
//	}
//
//    return 0;
//}


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

//    // TCP 通信
//    //1.创建socket(用于监听的套接字)
//    server_fd = socket(AF_INET, SOCK_STREAM, 0);
//
//    if(server_fd==-1)
//    {
//        perror("server socket error");
//        return RET_ERROR;
//    }
//
//    //2.绑定
//    server_addr.sin_family = PF_INET;
//    server_addr.sin_addr.s_addr = INADDR_ANY; //0.0.0.0
//    server_addr.sin_port = htons(9999);
//    ret = bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));
//    if(ret == -1)
//    {
//        perror("bind error");
//        return RET_ERROR;
//    }
//
//    //3.监听
//    listen(server_fd, 5);
//    if(ret==-1)
//    {
//        perror("listen error");
//        return RET_ERROR;
//    }
//
//    //4.接受客户端连接
//    socklen_t len = sizeof(client_addr);
//    client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &len);
//
//    if(client_fd==-1)
//    {
//        perror("accept");
//        return RET_ERROR;
//    }
//
//    //输出客户端的信息
//    char cip[16];
//    inet_ntop(AF_INET, &client_addr.sin_addr.s_addr, cip, sizeof(cip));
//    unsigned short cport = ntohs(client_addr.sin_port);
//    LOG_DEBUG("client ip is %s, port is %d", cip, cport);
//
//    // 设置接收超时时间 4.set receive timeout 
//    rcv_timeout = { 0, 5};   //{s, us}
//    setsockopt(server_fd, SOL_SOCKET, SO_RCVTIMEO, (char*)&rcv_timeout, sizeof( struct timeval));


    // 初始化监测ID列表
    UINT can_id = 0xA1;
	g_vec_monitor_id_list.push_back(can_id);
	can_id = 0xB1;
	g_vec_monitor_id_list.push_back(can_id);

	// 根据监测列表的长度创建多个独立队列
    int length = (int)g_vec_monitor_id_list.size();
	g_can_queue = MALLOC(struct can_queue_t*, length);
	memset(g_can_queue, 0, sizeof(struct can_queue_t*) * length);
	for(int i = 0; i < length; i++)
	{
	    g_can_queue[i] = can_queue_create();
	}

    // 用于baseline对比实验检测的报文采集队列
    g_can_queue_baseline = can_queue_create();


    g_attack_flag = MALLOC(int, length);
	memset(g_attack_flag, 0, sizeof(int) * length);

    // 打开文件(如果文件存在，先删除记录文件)
    if( access(DATA_PATH "detection_record.csv", F_OK)== 0)
    {
        if( remove(DATA_PATH "detection_record.csv") == 0)
        {
            LOG_DEBUG("remove" DATA_PATH "detection_record.csv");
        }
    }
	g_file_detection_record = fopen("../dataset/detection_record.csv", MODE_A_PLUS);

    if( access(DATA_PATH "singal_record.csv", F_OK)== 0)
    {
        if( remove(DATA_PATH "singal_record.csv") == 0)
        {
            LOG_DEBUG("remove" DATA_PATH "singal_record.csv");
        }

	}
	g_file_singal_record = fopen("../dataset/singal_record.csv", MODE_A_PLUS);

    if( access(DATA_PATH "time_interval_record.csv", F_OK)== 0)
    {
        if( remove(DATA_PATH "time_interval_record.csv") == 0)
        {
            LOG_DEBUG("remove" DATA_PATH "time_interval_record.csv");
        }

	}
	g_file_time_interval_record = fopen("../dataset/time_interval_record.csv", MODE_A_PLUS);

    if( access(DATA_PATH "baseline_detection_record.csv", F_OK)== 0)
    {
        if( remove(DATA_PATH "baseline_detection_record.csv") == 0)
        {
            LOG_DEBUG("remove" DATA_PATH "baseline_detection_record.csv");
        }
	}    
    g_file_baseline_detection_record = fopen("../dataset/baseline_detection_record.csv", MODE_A_PLUS);

//    if( access(DATA_PATH "attack_record.csv", F_OK)== 0)
//    {
//        remove(DATA_PATH "attack_record.csv");
//    } 
//	g_file_attack_record = fopen("../dataset/attack_record.csv", MODE_A_PLUS);

    // 车载状态初始化
    memset(&g_vehicle_status_can0, 0, sizeof(struct vehicle_status_t));
	g_vehicle_status_can0.bms_soc = 100;
	memset(&g_vehicle_status_can1, 0, sizeof(struct vehicle_status_t));
	g_vehicle_status_can1.bms_soc = 100;


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

//  // TCP 通信 关闭套接字
//  close(server_fd);
//	close(client_fd);

	// 根据监测列表的长度释放独立队列
    int length = (int)g_vec_monitor_id_list.size();
	for(int i = 0; i < length; i++)
	{
		can_queue_destroy(g_can_queue[i]);
	}
	free(g_can_queue);
	g_can_queue = NULL;


	// 释放用于对比方法监测的报文采集队列
    can_queue_destroy(g_can_queue_baseline);


    free(g_attack_flag);
	g_attack_flag = NULL;
	
    // 关闭文件
	fclose(g_file_detection_record);      
	fclose(g_file_singal_record);      
	fclose(g_file_time_interval_record); 
	fclose(g_file_baseline_detection_record);
//    fclose(g_file_attack_record);
	
    return RET_OK;
}


// IDS 检监测节点
int main()
{
	int ret;
	
    // 资源初始化
	init();
	
    // 线程参数声明
    struct thread_arg_t can0_rcv_thread_arg;                        // can0报文接收线程
    struct thread_arg_t can1_rcv_thread_arg;                        // can1报文接收线程
    struct thread_arg_t monitor_thread_arg;
	//struct thread_arg_t socket_rcv_thread_arg;
	struct thread_arg_t ids_baseline_thread_arg; 
    
    // 线程ID声明
    pthread_t can0_rcv_thread_id;                                   // can0报文接收线程号
    pthread_t can1_rcv_thread_id;                                   // can1报文接收线程号
	pthread_t monitor_thread_id;
	//pthread_t socket_rcv_thread_id;
	pthread_t ids_baseline_thread_id;

	
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

    // 创建监测线程
	monitor_thread_arg.run = TRUE;
	ret = pthread_create(&monitor_thread_id, NULL, monitor_detection, &monitor_thread_arg);
	if ( ret == 0) 
	{
		printf("create monitor thread successful \n");
	} 
	else
	{
		printf("create monitor thread failed! \n");
	}

    // 创建ids_baseline对比试验线程
	ids_baseline_thread_arg.run = TRUE;
	// 判断参数文件是否存在，存在创建在线检测线程，不存在则创建在线训练线程
	if( access(DATA_PATH "model.init", F_OK)== 0)       
	{
		ret = pthread_create(&ids_baseline_thread_id, NULL, baseline_detection, &ids_baseline_thread_arg);
		
		if ( ret == 0) 
		{
			printf("create ids baseline_detection thread successful \n");
		} 
		else
		{
			printf("create ids baseline_detection thread failed! \n");
		}
	}
	else
	{
		ret = pthread_create(&ids_baseline_thread_id, NULL, baseline_training, &ids_baseline_thread_arg);
				
		if ( ret == 0) 
		{
			printf("create ids baseline_training thread successful \n");
		} 
		else
		{
			printf("create ids baseline_training thread failed! \n");
		}

		
	}
	

//    // 创建攻击标记接收线程
//	socket_rcv_thread_arg.run = TRUE;
//	ret = pthread_create(&socket_rcv_thread_id, NULL, socket_recv_attack_flag, &socket_rcv_thread_arg);
//	if ( ret == 0) 
//	{
//		printf("create socket_rcv thread successful \n");
//	} 
//	else
//	{
//		printf("create socket_rcv thread failed! \n");
//	}


    // 信号记录定时任务
    Timer t_singal_record;
	t_singal_record.StartTimer(10, std::bind(singal_record));

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
		    pthread_mutex_lock(&mutex_vehicle_status_can0);
			// todo
            g_vehicle_status_can0.auto_drive_flag = res - '0';
			pthread_mutex_unlock(&mutex_vehicle_status_can0);

		}
		
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	// 退出报文接收线程
	can0_rcv_thread_arg.run = FALSE;
    pthread_join(can0_rcv_thread_id, NULL);

	// 退出报文接收线程
	can1_rcv_thread_arg.run = FALSE;
    pthread_join(can1_rcv_thread_id, NULL);

    // 退出监测线程
	monitor_thread_arg.run = FALSE;
    pthread_join(monitor_thread_id, NULL);

    // 退出ids_baseline对比试验线程
	ids_baseline_thread_arg.run = FALSE;
    pthread_join(ids_baseline_thread_id, NULL);

    // 退出socket攻击标记接收线程
//	socket_rcv_thread_arg.run = FALSE;
//    pthread_join(socket_rcv_thread_id, NULL);


    // 退出信号记录定时任务
	std::cout << "try to expire timer!" << std::endl;
	t_singal_record.Expire();


    // 释放资源
	deinit();
	
    return 0;
}
