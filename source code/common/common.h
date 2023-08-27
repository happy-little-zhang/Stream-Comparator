#ifndef COMMON_H_INCLUDED
#define COMMON_H_INCLUDED

#include <cstdlib>
#include <stdio.h>					// 标准C输入输出
#include <stdint.h>                 // C99中标准C库头文件（定义了 int64_t、uint8_t等数据类型）
#include <unistd.h>
#include <time.h>
#include <string.h>                 // 字符串库
#include <math.h>                   // 数学计算相关库
#include <signal.h>                 // 信号中断库
#include <linux/can.h>              // Linux CAN 相关
#include <linux/can/raw.h>          // Linux CAN 相关
#include <string.h>
#include <memory.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>				// socket通信相关, 函数原型         int socket(int af, int type, int protocol);
#include <sys/time.h>
#include <sys/types.h>              // 基本系统数据类型
#include <termios.h>                // 串口驱动头文件，扫描键盘事件相关
#include <pthread.h>                // 多线程库
#include <inttypes.h>
#include <limits.h>                 // 最大最小值
#include <float.h>                  // 浮点运算

#include <iostream>
#include <vector>
#include <functional>
#include <chrono>
#include <thread>
#include <atomic>
#include <memory>
#include <mutex>
#include <condition_variable>


#include "log.h"               // 日志模块库函数
#include "SimpleIni.h"         // 模型参数存储与读取相关库

using namespace std;

// Linux socket can 格式
// struct can_frame {
//		canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
//		__u8	can_dlc; /* frame payload length in byte (0 .. 8) */
//		__u8	__pad;	 /* padding */
//		__u8	__res0;  /* reserved / padding */
//		__u8	__res1;  /* reserved / padding */
//		__u8	data[8] __attribute__((aligned(8)));
// };


#define TRUE            true
#define FALSE           false
typedef bool            BOOL;
typedef char            CHAR;
typedef unsigned char   UCHAR;
typedef unsigned char   BYTE;
typedef unsigned char   *PBYTE;
typedef unsigned short  WORD;
typedef unsigned short  USHORT;
typedef int             INT;
typedef unsigned int    UINT;
typedef unsigned int    DWORD;
typedef unsigned int    *LPDWORD;
typedef unsigned long        ULONG;
typedef unsigned long long   ULONG64;
typedef void            *PVOID;
typedef void            *LPVOID;

// 返回值 
#define RET_OK                            0
#define RET_ERROR                         -1

// 文件打开方式
#define MODE_A_PLUS                "a+"
#define MODE_R_PLUS                "r+"
#define MODE_W_PLUS                "w+"


// 打印不同字体颜色的颜色宏定义 
#define NONE                              "\e[0m"
#define BLACK                             "\e[0;30m"
#define L_BLACK                           "\e[1;30m"
#define RED                               "\e[0;31m"
#define L_RED                             "\e[1;31m"
#define GREEN                             "\e[0;32m"
#define L_GREEN                           "\e[1;32m"
#define BROWN                             "\e[0;33m"
#define YELLOW                            "\e[1;33m"
#define BLUE                              "\e[0;34m"
#define L_BLUE                            "\e[1;34m"
#define PURPLE                            "\e[0;35m"
#define L_PURPLE                          "\e[1;35m"
#define CYAN                              "\e[0;36m"
#define L_CYAN                            "\e[1;36m"
#define GRAY                              "\e[0;37m"
#define WHITE                             "\e[1;37m"

#define BOLD                              "\e[1m"
#define UNDERLINE                         "\e[4m"
#define BLINK                             "\e[5m"
#define REVERSE                           "\e[7m"
#define HIDE                              "\e[8m"
#define CLEAR                             "\e[2J"
#define CLRLINE                           "\r\e[K" /* or "\e[1K\r" */

// 指针相关操作宏 
#define MALLOC(type,l)                   ((type*)malloc(sizeof(type)*(l)))
#define FREE(p)                          if(p) free(p); (p)=NULL
#define RETURN_IF_NULL(p)                if(!(p)) return

// 快捷运算宏定义 
#define ABS(x)                            ( (x)>0? (x):-(x) )     /* 绝对值 宏定义 */
#define SQR(x)                            ( (x) * (x) )           /* 平方 宏定义 */
#define MAX(a,b)                      ( (a)>(b)? (a):(b) )    /* 两者取大 */
#define MIN(a,b)                      ( (a)>(b)? (b):(a) )    /* 两者取小 */

#define FRAME_DLC             8                               // 报文数据长度
#define DATA_PATH             "../dataset/"
#define PARAM_PATH            "../param_config/"    // 模型参数文件路径前缀                       


// 异常监测相关
#define IDS_TRAIN_DATA                   "../dataset/raw_data_info.csv"
#define WHITE_LIST_DATA                  "../param_config/white_list.txt"
#define TIME_FOR_INIT_WHITE_LIST         10000                        // 初始化CAN_id列表时读取报文数据的时间10*1000ms
#define TRAIN_LOADING_CYCLE              5000000                      // 加载训练数据的时间提示单位us，


#define VALID                 1                // 变量有效
#define INVALID               0                // 变量无效
#define NOT_EXIST             -1               // 不存咋
#define ABNORMAL              1                // 异常
#define NORMAL                0                // 正常

//  配置文件相关宏定义
#define SECTION_NAME_ID_RPOFILE                 "id_profile"
#define KEY_NAME_CAN_ID_LIST                    "can_id_list"



#define BASIC_DEBUG           1

// 报文周期
#define GEAR_SHIFT_CMD_CYCLE                   20                // 档位切换控制报文周期(ms)
#define STEERING_CONTROL_CMD_CYCLE             20                // 转向控制报文周期(ms)
#define DRIVE_CONTROL_CMD_CYCLE                20                // 驱动控制报文周期(ms)
#define BRAKE_CONTROL_CMD_CYCLE                20                // 制动控制报文周期(ms)

// 报文周期
#define ECU1_SEND_CYCLE                        50                // auto --> VCU ctl    周期(ms)
#define ECU2_SEND_CYCLE                        100               // vcu --> velocity    周期(ms)
#define ECU3_SEND_CYCLE                        500               // SOC    周期(ms)
#define ECU4_SEND_CYCLE                        200               // door control    周期(ms)

// CAN通道号
#define CAN_CHANNEL_CAN0                       0
#define CAN_CHANNEL_CAN1                       1

// 自动驾驶限速
#define MAX_SPEED    5                 // 远程控制限速5km/h

enum enum_attack_flag {
    e_attack_initial = 0,              // 等待指令输入
    e_attack_free = 1,                 // 无攻击
	e_attack_dos = 2,                  // 拒绝服务攻击
	e_attack_inject = 3,               // 注入攻击
	e_attack_drop = 4,                 // 阻断攻击
	e_attack_tamper = 5,               // 篡改攻击
	
};

// 用于复现已有方法的结构体，汉明距离、ID矩阵、时间间隔
class ID_Profile {
public:
	// 汉明距离方法使用的变量
	int min_hamming_dist;                 // 最小汉明距离
	int max_hamming_dist;                 // 最大汉明距离
    int previous_payload_valid;           // 上一个负载是否有效
	BYTE previous_payload[FRAME_DLC];     // 上一个负载内容

    // ID矩阵方法使用的变量
	vector<UINT> legal_next_id;           // 合法后继ID,用于ID_matrix

    // 时间间隔方法使用的变量
    int64_t min_time_interval;                // 最小时间间隔 单位微秒
	int64_t max_time_interval;
    int previous_time_valid;              // 上一个接收时间是否有效
	int64_t previous_time;                   // 上一个报文接收时间


	// 构造函数
    ID_Profile()
	{
	    // 最大最小汉明距离初始化
		min_hamming_dist = 64;
		max_hamming_dist = 0;

        // 最大最小时间间隔初始化
		min_time_interval = 999999999;
	    max_time_interval = 0;
	};

	// 析构函数
    ~ID_Profile()
	    {};	
};


// 车载控制指令vehicle_control
struct vehicle_control_t {
    double acc_ctl;                       // 加速控制        加速>0，不加速=0
	double brake_ctl;                     // 制动控制        刹车>0 ，不刹车=0
    double eps_ctl;                       // 转向控制        左正右负
	int gear_ctl;                         // 档位信息        01：驻车P；02：空挡N；03：前进D；04：后退R；05：无效；
};

// 车载状态结构体
struct vehicle_status_t {
//    // 整车状态报文 0x180182a7       (100ms)
//	int vechicle_control_status;       // 整车受控状态       0-自动驾驶 1-人工 2-异常
//
//    // VCU控制报文 0x1801a782 (50ms)
    int auto_drive_flag;               // 自动驾驶使能信号
    double brake_pedal_value;          // 制动踏板开度
    double accelerator_pedal_value;    // 加速踏板开度
//    int gear_info;                     // 档位信息     0-空档 1-前进档 2-倒档 3-P档
//    int epb_flag;                      // 电子驻车信号 0-释放 1-驻车
//
//	// 电机转速转矩 0x18f120f0 (1000ms)
//	double motor_current_speed;        // 电机当前转速
//	double motor_real_speed;           // 电机实际转速
//
//	// 电机控制器状态 0x18f121f0 (1000ms)
//	double motor_status;               // 电机控制器状态         0000-停止   0010-运行   0101-故障
//	double motor_error_level;          // 电机控制器故障等级
//
//	// 整车故障信息 0x0cf101a7 (20ms)
//	double fail_info;

	// 防抱死系统 ABS 车轮速度信息报文 0x18febf0b (100ms)
	double front_axle_average_speed;    // 前轴平均速度
//	double front_left_speed;            // 前轴左轮相对速度
//	double front_right_speed;           // 前轴右轮相对速度
//	double rear_left_speed;             // 后轴左轮相对速度
//	double rear_right_speed;            // 后轴右轮相对速度
//
//	// 电子驻车系统 epb 系统状态 0x18f10183 (100ms)
//	double epb_status;                  // epb开关状态 00-常态 01-拉紧 10-释放 11-无效
//
//	// IBS控制报文 0x089f6265 (20ms)
//	double ibs_control_value;           // ibs目标压力
//
//	// IBS状态反馈报文 0x089b6562 (20ms)
//	double ibs_feedback_value;          // ibs反馈压力
//
//    // eps控制报文 0x08eb65ca (10ms)
//    double eps_control_value;           // 方向盘转向目标角度
//
//    // EPS状态反馈报文	0x08ee65ca (10ms)
//	double eps_feedback_value;          // 方向盘反馈角度

	// 门控灯光报文 0x1801a783 (100ms)

	// 车身控制BCM报文 0x18a70017 (200ms)

	// 电池管理系统BMS报文 0x18f13df3 (500ms)
	double bms_soc;                   // 电池剩余电量

//	// 汽车加速度
//	double acc_x;                       // 汽车横向加速度
//	double acc_y;                       // 汽车纵向加速度

//	// 超声波雷达UltrasonicRadarSystem
//	double urs_front_left_distance;            // 前左距离
//	double urs_front_left_center_distance;     // 前左中距离
//    double urs_front_right_center_distance;    // 前右中距离
//    double urs_front_right_distance;           // 前右距离
//	double urs_rear_left_distance;             // 后左距离
//	double urs_rear_left_center_distance;      // 后左中距离
//    double urs_rear_right_center_distance;     // 后右中距离
//    double urs_rear_right_distance;            // 后右距离
//    double urs_left_front_distance;            // 左侧距离（前）
//	double urs_left_rear_distance;             // 左侧距离（后）
//    double urs_right_front_distance;           // 右侧距离（前）
//	double urs_right_rear_distance;            // 右侧距离（后）	 
	
};

// 线程运行结构体
struct thread_arg_t {
    int run;                 // 线程运行标志  0-停止  1-运行
    int thread_id;           // 线程号
};

// 报文监控节点结构体
struct can_node_t {
	int64_t system_time;               // 报文接收时间us
	int can_channel;                   // CAN 通道号
	struct can_frame frame;            // 报文内容
	struct can_node_t *next;           // 下节点指针               
};

// 报文队列结构体/监测列表结构体
struct can_queue_t {
    struct can_node_t *front;
	struct can_node_t *rear;
	int length;
};


// 定时器类
class Timer {
public:
	Timer() :expired_(true), try_to_expire_(false)
	{
	}
 
	Timer(const Timer& t)
	{
		expired_ = t.expired_.load();
		try_to_expire_ = t.try_to_expire_.load();
	}
	
	~Timer()
    {
		Expire();
		//		std::cout << "timer destructed!" << std::endl;
	}
 
	void StartTimer(int interval, std::function<void()> task)     // 时间间隔毫秒ms
	{
		if (expired_ == false)
		{
			//			std::cout << "timer is currently running, please expire it first..." << std::endl;
			return;
		}
		expired_ = false;
		std::thread([this, interval, task]()
		{
			while (!try_to_expire_)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(interval));
				task();
			}
			//			std::cout << "stop task..." << std::endl;
			{
				std::lock_guard<std::mutex> locker(mutex_);
				expired_ = true;
				expired_cond_.notify_one();
			}
		}).detach();
	}
 
	void Expire()
	{
		if (expired_)
		{
			return;
		}
 
		if (try_to_expire_)
		{
			//			std::cout << "timer is trying to expire, please wait..." << std::endl;
			return;
		}
		try_to_expire_ = true;
		{
			std::unique_lock<std::mutex> locker(mutex_);
			expired_cond_.wait(locker, [this]{return expired_ == true; });
			if (expired_ == true)
			{
				//				std::cout << "timer expired!" << std::endl;
				try_to_expire_ = false;
			}
		}
	}
 
	template<typename callable, class... arguments>
	void SyncWait(int after, callable&& f, arguments&&... args)
	{
		std::function<typename std::result_of<callable(arguments...)>::type()> task
			(std::bind(std::forward<callable>(f), std::forward<arguments>(args)...));
		std::this_thread::sleep_for(std::chrono::milliseconds(after));
		task();
	}
	template<typename callable, class... arguments>
	void AsyncWait(int after, callable&& f, arguments&&... args)
	{
		std::function<typename std::result_of<callable(arguments...)>::type()> task
			(std::bind(std::forward<callable>(f), std::forward<arguments>(args)...));
 
		std::thread([after, task]()
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(after));
			task();
		}).detach();
	}
 
private:
	std::atomic<bool> expired_;
	std::atomic<bool> try_to_expire_;
	std::mutex mutex_;
	std::condition_variable expired_cond_;
	
};
//this is a timer test
//  Timer t;
//  //周期性执行定时任务
//  t.StartTimer(1000, std::bind(EchoFunc,"hello world!"));
//  std::this_thread::sleep_for(std::chrono::seconds(4));
//  std::cout << "try to expire timer!" << std::endl;
//  t.Expire();
//
//  //周期性执行定时任务
//  t.StartTimer(1000, std::bind(EchoFunc,	"hello c++11!"));
//  std::this_thread::sleep_for(std::chrono::seconds(4));
//  std::cout << "try to expire timer!" << std::endl;
//  t.Expire();
//
//  std::this_thread::sleep_for(std::chrono::seconds(2));
//
//  //只执行一次定时任务
//  //同步
//  t.SyncWait(1000, EchoFunc, "hello world!");
//  //异步
//  t.AsyncWait(1000, EchoFunc, "hello c++11!");
//
//  std::this_thread::sleep_for(std::chrono::seconds(2));


// PID类 见自动驾驶课程
class PID {

public:
    /*
    * Errors
    */
    double error_proportional_;
    double error_integral_;
    double error_derivative_;

    /*
    * Coefficients
    */
    double Kp_;
    double Ki_;
    double Kd_;

    /*
    * Constructor
    */
    PID()
    {
		error_proportional_ = 0.0;
		error_integral_ 	= 0.0;
		error_derivative_	= 0.0;    
    }
	

    /*
    * Destructor.
    */
    virtual ~PID() { }

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd)
    {
		Kp_ = Kp;
		Ki_ = Ki;
		Kd_ = Kd;
    }

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte)
    {
		error_integral_ 	+= cte;
		error_derivative_	 = cte - error_proportional_;
		error_proportional_  = cte;
    }

    /*
    * Calculate the total PID error.
    */
    double TotalError()
    {
		return -(Kp_ * error_proportional_ + Ki_ * error_integral_ + Kd_ * error_derivative_);    
    }
};

//int pid_test()
//{
//	FILE *pid_file = fopen("../dataset/speed_record.csv", MODE_A_PLUS);
//
//    double target = 20;               // km/h 
//    double value_now = 0;
//	double value_last = 0;
//	double dvalue = 0;
//	double dvalue_max = 6;            // 假设最大加速度为 6m/s^2
//	double dvalue_min = -8;           // 最大制动力为          -8m/s^2
//    int count = 0;
//	double dt = 0.1;
//
//    PID pid;
//	pid.Init(5, 0, 0);
//	
//    while(1)
//    {
//        value_now = value_last + dvalue*dt;
//			
//		fprintf(pid_file, "%ld, %lf\n", get_system_us_time(), value_now);
//
//        double cte = value_now - target;
//        pid.UpdateError(cte);
//		dvalue = pid.TotalError();
//		dvalue = dvalue > dvalue_max?  dvalue_max : dvalue;
//		dvalue = dvalue < dvalue_min?  dvalue_min : dvalue;
//
//		printf("value_now: %lf, error_p: %lf, error_i: %lf, error_d: %lf, dvalue: %lf \n", 
//			value_now, pid.error_proportional_, pid.error_integral_, pid.error_derivative_, dvalue);
//        value_last = value_now;
//
//		std::this_thread::sleep_for(std::chrono::milliseconds(100));
//
//        count++;
//        if(count > 200)
//        {
//            break;
//        }
//    }
//
//    fclose(pid_file);
//	
//    return 0;
//}





// 字节数组转换字符串 
int byte_to_hex_str(char *dest, const char *src, int src_len);

// 获取系统毫秒时间ms
int64_t get_system_ms_time();

// 获取系统微秒时间us
int64_t get_system_us_time();

// select 精确睡眠,单位ms 
int select_sleep(int ms_time);

// select 精确睡眠,单位us
int select_sleep_us(int us_time);

// 扫描键盘,返回值是该键的ASCII码值，不需要回车的，
int scanKeyboard();

// 复制CAN报文
int can_frame_duplicate(struct can_frame *destination, struct can_frame *source);

// 打印输出CAN报文
int can_frame_show(struct can_frame *frame, int channel);

// 向指定文件写原始报文
int can_frame_write(struct can_frame *frame, int channel, FILE *target_file);

// 将报文数据解析成实际数值（解析原子函数)
double can_to_data(BYTE *data, int start_bit, int length, double radio, double offset);

// 逆解析，将实际数值转换到相对应的字节数组中
void data_to_can(double value, BYTE *data, int start_bit, int length, double radio, double offset);

// 创建车载状态结构体
vehicle_status_t *vehicle_status_create(void);

// 打印车载状态
int vehicle_status_show(struct vehicle_status_t *vehicle_status);

// 向文件中写车载状态
int vehicle_status_write(struct vehicle_status_t * vehicle_status, FILE * target_file);

// 解析CAN报文
int parser_can_func(UINT can_id, BYTE *data, vehicle_status_t* real_vehicle_status);

// 创建一个空can队列
struct can_queue_t *can_queue_create(void);

// 清空can报文队列
int can_queue_clear(struct can_queue_t *queue);

// 销毁can报文队列
int can_queue_destroy(struct can_queue_t *queue);

// 判断CAN队列是否为空，空则返回1
int can_queue_empty(struct can_queue_t *queue);

// 返回can队列大小
int can_queue_get_length(struct can_queue_t *queue);

// 从can队列队尾入队：复制一份内存入队，原数据在调用此接口完成入队后需销毁内存
int can_queue_enqueue(struct can_queue_t *queue, struct can_frame frame, int can_channel);

// 从can队列队首出队，出队的元素需销毁内存
int can_queue_dequeue(struct can_queue_t *queue);

// 复制can队列;由于入队操作会申请内存，故复制时无需申请；失败时头指针内存无需销毁，其余元素需销毁
int can_queue_duplicate(struct can_queue_t *dst, struct can_queue_t *src);

// 遍历can队列中的元素
int can_queue_show(struct can_queue_t *queue);

// 根据报文ID获取下标映射
int get_map_id2num(vector<UINT> &vec_can_id, UINT frame_id);

// 根据下标映射获取报文ID
UINT get_map_num2id(vector<UINT> &vec_can_id, int num);

// 比较两个CAN节点是否相似
bool can_node_compare(can_node_t *node0, can_node_t *node1);

// 读取字节指定bit位的值
int read_bit_value(uint8_t *data, int bit_pos);

// 给字节指定bit位赋值(这里的value不是0就是1)
int write_bit_value(uint8_t *data, int bit_pos, int value);

// 读取ID轮廓
int read_id_profile(vector<ID_Profile> &id_profile, vector<UINT> &vec_can_id, CSimpleIniA &configurator);

// 打印ID轮廓
int show_id_profile(vector <ID_Profile> & id_profile, vector <UINT> & vec_can_id);

// 保存轮廓数据
int save_id_profile(vector<UINT> &vec_can_id, vector<ID_Profile> &id_profile, CSimpleIniA & configurator);

// 合法ID后继检测器
int id_matrix_detect(ID_Profile &id_profile, UINT can_id, int &is_anomaly);

// 计算两个8字节报文汉明距离
// @ 输入两个字节序列
int calculate_hamming_distance(uint8_t *data_previous, uint8_t *data_current);

// hamming距离检测器
int hamming_distance_detect(ID_Profile &id_profile, BYTE *current_payload, int &is_anomaly);

// 时间间隔检测器
int time_interval_detect(ID_Profile &id_profile, int64_t current_time, int &is_anomaly);

// 解析从csv文件读取的CAN报文   
// timestamp,id,data0 data1 data2 data3 data4 data5 data6 data 7
int parse_read_buffer(char *data_buffer, int64_t *timestamp, struct can_frame *new_can);

// 将CAN报文以固定格式存储到文件中
int save_read_buffer(FILE *data_file, int64_t *timestamp, struct can_frame *new_can);

#endif  // COMMON_H_INCLUDED
