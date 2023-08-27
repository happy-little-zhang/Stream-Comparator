#include "common.h"
#include "log.h"

/* 字节数组转换字符串 */
int byte_to_hex_str(char *dest, const char *src, int src_len)
{
    if (!dest || !src || 0 >= src_len)
    {
        return RET_ERROR;
    }

    for (int i = 0, j = 0; j < src_len; j++)
    {
        snprintf(dest + i, 3, "%02x", (unsigned char)src[j]);
        i += 2;
    }

    return RET_OK;
}

/****************************************
 * 函数名：int64_t get_system_ms_time()
 * 函数功能：获取系统时间
 * 输入：无
 * 输出：系统时间 单位 ms   返回值最好是int64_t，long long应该也可以
 *****************************************/
int64_t get_system_ms_time()
{
    struct timeval tv;
	memset(&tv, 0, sizeof(struct timeval));
    gettimeofday(&tv,NULL);    //该函数在sys/time.h头文件中
    //printf("sys time: %ld s, %ld us\n", tv.tv_sec , tv.tv_usec);
    //printf("sys time: %lld ms\n", (unsigned long long)tv.tv_sec * 1000 + (unsigned long long)tv.tv_usec / 1000);
    return (unsigned long long)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

/*获取系统微秒时间us*/
int64_t get_system_us_time()
{
    struct timeval tv;
    gettimeofday(&tv,NULL);    //该函数在sys/time.h头文件中
    return tv.tv_sec * 1000 * 1000 + tv.tv_usec;
}

/* select 精确睡眠,单位ms */
int select_sleep(int ms_time)
{
    /*select()函数做睡眠*/
    struct timeval msgrcv_timeout={0, ms_time * 1000};   //{秒,微秒}
    select(0, NULL, NULL, NULL, &msgrcv_timeout);
    return RET_OK;
}

// select 精确睡眠,单位us
int select_sleep_us(int us_time)
{
    // select()函数做睡眠
    struct timeval msgrcv_timeout={0, us_time};   //{秒,微秒}
    select(0, NULL, NULL, NULL, &msgrcv_timeout);
    return RET_OK;
}


// 扫描键盘,返回值是该键的ASCII码值，不需要回车的，
int scanKeyboard()
{
	int in;
	struct termios new_settings;
	struct termios stored_settings;
	
	tcgetattr(0, &stored_settings);
	
	new_settings = stored_settings;
	new_settings.c_lflag &= (~ICANON);
	
	new_settings.c_cc[VTIME] = 0;
	tcgetattr(0, &stored_settings);
	
	new_settings.c_cc[VMIN] = 1;
	tcsetattr(0, TCSANOW, &new_settings);
	 
	in = getchar();
	 
	tcsetattr(0, TCSANOW, &stored_settings);
	
	return in;
	
}


// 复制CAN报文 
int can_frame_duplicate(struct can_frame *destination, struct can_frame *source)
{
    if(!destination || !source)
    {
        LOG_ERROR("invalid input: destination[%p], source[%p]!", destination, source);
        return RET_ERROR;
    }

    memcpy(destination, source, sizeof(struct can_frame));
    return RET_OK;
}

// 打印输出CAN报文 
int can_frame_show(struct can_frame *frame, int channel)
{
    if (!frame)
    {
        LOG_ERROR("invalid input!");
        return RET_ERROR;
    }
    char szPrintInfo[512] = {0};
    char szInfo[64] = {0};
	
    sprintf(szInfo, "%ld %d %08X", get_system_us_time(), channel, frame->can_id);
    strcat(szPrintInfo, szInfo);
    
    for (int i = 0; i < frame->can_dlc; ++i)
    {
        char szTemp[8] = {0};
        sprintf(szTemp, " %02X", frame->data[i]);
        strcat(szPrintInfo, szTemp);
    }
    LOG_DEBUG("%s", szPrintInfo);

    return RET_OK;
}

// 向指定文件写原始报文
int can_frame_write(struct can_frame *frame, int channel, FILE *target_file)
{
    if (!frame || !target_file)
    {
        LOG_ERROR("invalid input: frame[%p] target_file[%p]!", frame, target_file);
        return RET_ERROR;        
    }

	char frame_info[512] = {0};
	char id_info[64] = {0};
    sprintf(id_info, "%ld, %d, %08X", get_system_us_time(), channel, frame->can_id);
    strcat(frame_info, id_info);

    for (int i = 0; i < frame->can_dlc; ++i)
    {
        char data_info[8] = {0};
        sprintf(data_info, ",%02X", frame->data[i]);
        strcat(frame_info, data_info);
    }

	fprintf(target_file, "%s\n", frame_info);

    return RET_OK;
}



// 将报文数据解析成实际数值（解析原子函数) 
double can_to_data(BYTE *data, int start_bit, int length, double radio, double offset)
{
    int data_length = 8;      /* 默认CAN报文的数据数组长度均为8 */
    DWORD m_data = 0;         /* CAN报文的数值（总线上传输的数值） */
    BYTE m_data_byte[8];      /* 暂存CAN报文数据的数组 */
    double ret = 0;           /* 返回的结果 */
    int k = 0;

    int start_byte, start_bit_offset;
    int end_byte, end_bit_offset;

    start_byte = start_bit / 8;                           /* 起始字节 */
    start_bit_offset = start_bit % 8;                     /* 起始字节位偏移 */
    end_byte = (start_bit + length - 1) / 8;              /* 终了字节 */
    end_bit_offset = (start_bit + length - 1) % 8;        /* 终了字节位偏移 */

    for (int i = 0; i < data_length; i++){
        m_data_byte[i] = data[i];
    }

    for (int j = end_byte; j >= start_byte; j--){
        m_data = m_data | m_data_byte[j];
        if (j != start_byte)
            m_data = m_data << 8;
        k = k + 1;
    }

    if (k > 4){
        LOG_ERROR("data length is bigger than 4, please check data!");
    }else{
        m_data = m_data << (4 * 8 - k * 8 + (7 - end_bit_offset));
        m_data = m_data >> ((4 * 8 - k * 8 + (7 - end_bit_offset)) + start_bit_offset);
    }

    ret = m_data * radio + offset;    /* 实际数值=总线传输量*分辨率+偏移量 */
    return ret;
}

// 逆解析，将实际数值转换到相对应的字节数组中 
void data_to_can(double value, BYTE *data, int start_bit, int length, double radio, double offset)
{
    DWORD m_data = 0;
    m_data = (value-offset)/radio;   /* 计算CAN总线传输数值 */

    int start_byte, start_bit_offset;
    int end_byte;
    //int end_bit_offset;

    start_byte = start_bit / 8;                          /* 起始字节 */
    start_bit_offset = start_bit % 8;                    /* 起始字节位偏移 */
    end_byte = (start_bit + length - 1) / 8;             /* 终了字节 */
    //end_bit_offset = (start_bit + length - 1) % 8;     /* 终了字节位偏移 */

    m_data = m_data << start_bit_offset;      /* 填充起始字节偏移 */

    for(int i = start_byte; i <= end_byte; i++){
        data[i] = m_data | data[i];
        if (i != end_byte)
            m_data = m_data >> 8;
    }

    return;
}

// 创建车载状态结构体
vehicle_status_t *vehicle_status_create(void)
{
    struct vehicle_status_t *new_p = NULL;

    new_p = MALLOC(struct vehicle_status_t, 1);
    if(NULL == new_p)
    {
        LOG_ERROR("malloc vehicle_status_t failed!");

        return NULL;
    }   

    memset(new_p, 0, sizeof(vehicle_status_t));
	
	return new_p;
}


// 打印车载状态
int vehicle_status_show(struct vehicle_status_t * vehicle_status)
{
    if (!vehicle_status)
    {
        LOG_ERROR("invalid input: vehicle_status[%p]!", vehicle_status);
        return RET_ERROR;        
    }

    // 依次打印车载状态信息
    char info_buff[1024] = "real_time_vehicle_status_info: \n";
    snprintf(info_buff + strlen(info_buff), sizeof(info_buff) - strlen(info_buff), "(1) vehicle_speed: %.2lf km/h \n", vehicle_status->front_axle_average_speed);
	snprintf(info_buff + strlen(info_buff), sizeof(info_buff) - strlen(info_buff), "(2) accelerator_pedal_value: %.2lf bit \n", vehicle_status->accelerator_pedal_value);
	snprintf(info_buff + strlen(info_buff), sizeof(info_buff) - strlen(info_buff), "(3) brake_pedal_value: %.2lf bit \n", vehicle_status->brake_pedal_value);
	snprintf(info_buff + strlen(info_buff), sizeof(info_buff) - strlen(info_buff), "(4) state_of_charge_value: %.2lf percent \n", vehicle_status->bms_soc);	
	
//    snprintf(info_buff + strlen(info_buff), sizeof(info_buff) - strlen(info_buff), "(1) vehicle_speed: %.2lf km/h \n", vehicle_status->front_axle_average_speed);
//	snprintf(info_buff + strlen(info_buff), sizeof(info_buff) - strlen(info_buff), "(2) accelerator_pedal_value: %.2lf bit \n", vehicle_status->accelerator_pedal_value);
//	snprintf(info_buff + strlen(info_buff), sizeof(info_buff) - strlen(info_buff), "(3) brake_pedal_value: %.2lf bit \n", vehicle_status->brake_pedal_value);
//	snprintf(info_buff + strlen(info_buff), sizeof(info_buff) - strlen(info_buff), "(4) ibs_control_value: %.2lf bar \n", vehicle_status->ibs_control_value);
//	snprintf(info_buff + strlen(info_buff), sizeof(info_buff) - strlen(info_buff), "(5) ibs_feedback_value: %.2lf bar \n", vehicle_status->ibs_feedback_value);
//	snprintf(info_buff + strlen(info_buff), sizeof(info_buff) - strlen(info_buff), "(6) eps_control_value: %.2lf degree \n", vehicle_status->eps_control_value);
//	snprintf(info_buff + strlen(info_buff), sizeof(info_buff) - strlen(info_buff), "(7) eps_feedback_value: %.2lf degree \n", vehicle_status->eps_feedback_value);
//	snprintf(info_buff + strlen(info_buff), sizeof(info_buff) - strlen(info_buff), "(8) state_of_charge_value: %.2lf percent \n", vehicle_status->bms_value);
//	snprintf(info_buff + strlen(info_buff), sizeof(info_buff) - strlen(info_buff), "(9) gear_info: %d \n", vehicle_status->gear_info);
//	snprintf(info_buff + strlen(info_buff), sizeof(info_buff) - strlen(info_buff), "(10) urs_radar_info: [front] %.2lf, %.2lf, %.2lf, %.2lf [rear] %.2lf, %.2lf, %.2lf, %.2lf [left] %.2lf, %.2lf [right] %.2lf, %.2lf \n", 
//		    vehicle_status->urs_front_left_distance, vehicle_status->urs_front_left_center_distance, vehicle_status->urs_front_right_center_distance, 
//		        vehicle_status->urs_front_right_distance, vehicle_status->urs_rear_left_distance, vehicle_status->urs_rear_left_center_distance, 
//		            vehicle_status->urs_rear_right_center_distance, vehicle_status->urs_rear_right_distance, vehicle_status->urs_left_front_distance,
//		                vehicle_status->urs_left_rear_distance, vehicle_status->urs_right_front_distance, vehicle_status->urs_right_rear_distance);
	
	LOG_DEBUG("%s", info_buff);

	return RET_OK;
}

// 向文件中写车载状态
int vehicle_status_write(struct vehicle_status_t * vehicle_status, FILE * target_file)
{
    if (!vehicle_status || !target_file)
    {
        LOG_ERROR("invalid input: vehicle_status[%p] target_file[%p] !", vehicle_status, target_file);
        return RET_ERROR;        
    }

    // 依次写入车载状态信息
    char info_buff[1024] = "";
//    snprintf(info_buff + strlen(info_buff), sizeof(info_buff) - strlen(info_buff), "%ld,", get_system_ms_time());	
//    snprintf(info_buff + strlen(info_buff), sizeof(info_buff) - strlen(info_buff), "%lf,", vehicle_status->front_axle_average_speed);
//	snprintf(info_buff + strlen(info_buff), sizeof(info_buff) - strlen(info_buff), "%lf,", vehicle_status->accelerator_pedal_value);
//	snprintf(info_buff + strlen(info_buff), sizeof(info_buff) - strlen(info_buff), "%lf,", vehicle_status->brake_pedal_value);
//	snprintf(info_buff + strlen(info_buff), sizeof(info_buff) - strlen(info_buff), "%lf,", vehicle_status->ibs_control_value);
//	snprintf(info_buff + strlen(info_buff), sizeof(info_buff) - strlen(info_buff), "%lf,", vehicle_status->ibs_feedback_value);
//	snprintf(info_buff + strlen(info_buff), sizeof(info_buff) - strlen(info_buff), "%lf,", vehicle_status->eps_control_value);
//	snprintf(info_buff + strlen(info_buff), sizeof(info_buff) - strlen(info_buff), "%lf\n", vehicle_status->eps_feedback_value);

	fprintf(target_file, "%s", info_buff); 
	
	return RET_OK;
}

// 解析CAN报文(基于协议规则检测报文内容)
int parser_can_func(UINT can_id, BYTE *data, vehicle_status_t* real_vehicle_status)
{
    switch(can_id){
		
		// auto control
		case 0xA1:			
			real_vehicle_status->brake_pedal_value = can_to_data(data, 0, 16, 0.1, 0.0);                 // 制动踏板
			real_vehicle_status->accelerator_pedal_value = can_to_data(data, 16, 16, 0.1, 0.0);          // 加速踏板
			break;
			
		// vcu speed
	    case 0xB1:			
			real_vehicle_status->front_axle_average_speed = can_to_data(data, 0, 16, (double)(1.0/256.0), 0.0);  
			break;
		
		// bms soc
	    case 0xC1:			
			real_vehicle_status->bms_soc = can_to_data(data, 0, 16, 0.01, 0.0);  
			break;
		
		// monitor control
	    case 0xE1:			
			real_vehicle_status->auto_drive_flag = can_to_data(data, 0, 16, 1.0, 0.0);  
			break;


//		// gear档位控制
//		case 0xA1:
//			//real_vehicle_status->gear_info = can_to_data(data, 8, 8, 1, 0);    // 转向角值
//			break;
//	
//		// 转向角控制
//		case 0xA2:
//			real_vehicle_status->eps_control_value = can_to_data(data, 8, 16, 0.0008545, -28);    // 转向角值
//			break;
//		
//		// 驱动控制
//		case 0xA3:
//			real_vehicle_status->accelerator_pedal_value = can_to_data(data, 8, 16, 0.001525, 0);    // 加速踏板开度
//			break;
//
//		// 制动控制
//		case 0xA4:
//			real_vehicle_status->ibs_control_value = can_to_data(data, 8, 16, 0.001525, 0);          // 制动踏板开度
//			break;
//		
//		// 驻车控制
//		case 0xA5:
//			real_vehicle_status->epb_flag = can_to_data(data, 8, 8, 1, 0);
//			break;
//
//		// vcu反馈
//		case 0xC1:
//			real_vehicle_status->front_axle_average_speed = can_to_data(data, 0, 16, 0.01, 0);      // 车速
//			real_vehicle_status->eps_feedback_value = can_to_data(data, 16, 16, 0.0008545, -28);    // 转向角
//			real_vehicle_status->gear_info = can_to_data(data, 32, 8, 1, 0);                        // 档位信息
//			real_vehicle_status->accelerator_pedal_value =  can_to_data(data, 40, 8, 0.19607, 0);   // 当前力矩信息
//			break;		
//
//		// bms反馈
//		case 0xC2:
//			real_vehicle_status->bms_value = can_to_data(data, 0, 8, 1, 0);       // 剩余电量   soc   state of charge
//			break;
//
//		// 超声波雷达距离检测（前后）m
//		case 0x5C8:
//			real_vehicle_status->urs_front_left_center_distance = can_to_data(data, 40, 8, 0.02, 0);     // 前左中距离
//			real_vehicle_status->urs_front_right_center_distance = can_to_data(data, 48, 8, 0.02, 0);    // 前右中距离
//			real_vehicle_status->urs_rear_left_center_distance = can_to_data(data, 16, 8, 0.02, 0);      // 后左中距离
//			real_vehicle_status->urs_rear_right_center_distance = can_to_data(data, 8, 8, 0.02, 0);     // 后右中距离
//			break;
//		// 超声波雷达距离检测（左右）m
//		case 0x5C9:
//			real_vehicle_status->urs_front_left_distance = can_to_data(data, 40, 8, 0.02, 0);            // 前左距离
//			real_vehicle_status->urs_front_right_distance = can_to_data(data, 8, 8, 0.02, 0);            // 前右距离
//			real_vehicle_status->urs_rear_left_distance = can_to_data(data, 48, 8, 0.02, 0);             // 后左距离
//			real_vehicle_status->urs_rear_right_distance = can_to_data(data, 16, 8, 0.02, 0);            // 后右距离		
//			real_vehicle_status->urs_left_front_distance = can_to_data(data, 32, 8, 0.02, 0);            // 左侧距离（前）
//			real_vehicle_status->urs_left_rear_distance = can_to_data(data, 56, 8, 0.02, 0);             // 左侧距离（后）	
//			real_vehicle_status->urs_right_front_distance = can_to_data(data, 0, 8, 0.02, 0);            // 右侧距离（前）
//			real_vehicle_status->urs_right_rear_distance = can_to_data(data, 24, 8, 0.02, 0);            // 右侧距离（后）	
//			break;		
		default:
		    // 未知报文
		    // LOG_WARNING("-------unknow ID message[%08X]!-------", can_id);
			// LOG_DEBUG(RED "-------unknow ID message[%08X]!-------" NONE, can_id);
		    break;
			
	}

	return RET_OK;
}

// 创建一个空can队列 
struct can_queue_t *can_queue_create(void)
{
    struct can_queue_t *lp = NULL;

    lp = MALLOC(struct can_queue_t, 1);
    if(NULL == lp)
    {
        LOG_ERROR("malloc can_queue_t failed!");

        return NULL;
    }
    memset(lp, 0, sizeof(struct can_queue_t));

    lp->front = MALLOC(struct can_node_t, 1);
    if (NULL == lp->front)
    {
        LOG_ERROR("malloc can_id_head_pointer failed!");
        free(lp);
        lp = NULL;

        return NULL;
    }
	
    lp->front->next = NULL;                                   // 头节点后面连接队列元素
    memset(&lp->front->frame, 0, sizeof(struct can_frame));    //头结点的报文内容为0
    lp->rear = lp->front;                                     // 初始队尾指针指向头节点 

    return lp;
}

// 清空can报文队列 
int can_queue_clear(struct can_queue_t *queue)
{
    if (queue && queue->front)
    {
        while (queue->front->next)
        {
            can_queue_dequeue(queue);
        }
    }

    return RET_OK;
}


// 销毁can_id报文队列 
int can_queue_destroy(struct can_queue_t *queue)
{
    if (queue)
    {
        can_queue_clear(queue);
		
        // 释放头节点所占空间
        if (queue->front)
        {
            free(queue->front);
            queue->front = NULL;
        }

        free(queue);
        queue = NULL;
    }

    return RET_OK;
}

// 判断CAN_ID队列是否为空，空则返回1 
int can_queue_empty(struct can_queue_t *queue)
{
	if (queue)
	{
		return queue->length == 0? 1:0;
	}

	LOG_ERROR("invalid input!");

	return 1;
}


// 返回can_id队列大小 
int can_queue_get_length(struct can_queue_t *queue)
{
    if (queue)
    {
        return queue->length;
    }

    LOG_ERROR("invalid input!");
	
    return RET_ERROR;
}

// 从can队列队尾入队：复制一份内存入队，原数据在调用此接口完成入队后需销毁内存 
int can_queue_enqueue(struct can_queue_t *queue, struct can_frame frame, int can_channel)
{
    int ret = RET_ERROR;

    if (!queue)
    {
        LOG_ERROR("invalid input: queue[%p]!", queue);

        return ret;
    }

    if (!queue->front || !queue->rear)
    {
        LOG_ERROR("invalid queue: front[%p], rear[%p]!", queue->front, queue->rear);

        return ret;
    }

    // 创建一个新的节点
    struct can_node_t *temp = NULL;
    temp = MALLOC(struct can_node_t, 1);
    if (NULL == temp)
    {
        LOG_ERROR("malloc can_node_t failed!");

        return ret;
    }
    memset(temp, 0, sizeof(struct can_node_t));
	temp->system_time = get_system_us_time();
	temp->can_channel = can_channel;
	can_frame_duplicate(&temp->frame, &frame);
    temp->next = NULL;

    // 将节点加入到队列尾部 
    queue->rear->next = temp;
    queue->rear = temp;
    queue->length++;
	
    return RET_OK;
}

// 从can队列队首出队，出队的元素需销毁内存 
int can_queue_dequeue(struct can_queue_t *queue)
{
    int ret = RET_ERROR;

    if (!queue)
    {
        LOG_ERROR("invalid input!");

        return ret;
    }

    if (!queue->front || !queue->rear)
    {
        LOG_ERROR("invalid queue: front[%p], rear[%p]!", queue->front, queue->rear);

        return ret;
    }

    if (!queue->front->next)
    {
        LOG_ERROR("there is no valid element in queue!");
        queue->rear = queue->front;

        return ret;
    }

    struct can_node_t *temp = NULL;

    // 如果被删除的节点是尾节点，则改变尾节点的指向(以防出现野指针)
    if (queue->front->next == queue->rear)
    {
        queue->rear = queue->front;
    }
    temp = queue->front->next;
    queue->front->next = temp->next;

    // 释放出队元素的内存
    free(temp);
    temp = NULL;
    if (0 < queue->length)
    {
        queue->length--;
    }

    return RET_OK;
}

// 复制can队列;由于入队操作会申请内存，故复制时无需申请；失败时头指针内存无需销毁，其余元素需销毁 
int can_queue_duplicate(struct can_queue_t *dst, struct can_queue_t *src)
{
    int ret = RET_ERROR;
    struct can_node_t *ne = NULL;

    if (!dst || !src || !src->front)
    {
        LOG_ERROR("invalid input: dst[%p], src[%p] or src->front!", dst, src);
        return ret;
    }

    // 遍历队列，复制到目的队列中 
    ne = src->front;
    while (ne)
    {
        // 头节点系统时间为0
        if (ne->system_time == 0 )
        {
            ne = ne->next;
            continue;
        }

        ret = can_queue_enqueue(dst, ne->frame, ne->can_channel);
        if (RET_OK != ret)
        {
            LOG_ERROR("can_queue_enqueue failed!");
            can_queue_clear(dst);

            return ret;
        }

        ne = ne->next;
    }

    if (dst->length != src->length)
    {
        LOG_ERROR("duplicate error, dst->length = %ld, src->length = %ld", dst->length, src->length);
        can_queue_destroy(dst);

        return RET_ERROR;
    }

    return RET_OK;
}

// 遍历can队列中的元素 
int can_queue_show(struct can_queue_t *queue)
{
    if (!queue)
    {
        LOG_ERROR("invalid input!");
        return RET_ERROR;
    }

    // 打印队列长度 
    LOG_DEBUG("can_queue_length: %ld", queue->length);

    // 打印队列元素 
    LOG_DEBUG("can_queue_element:");

    struct can_node_t *q = NULL;
    q = queue->front->next;
	int count = 0;
	
    if (queue->length)
    {
        while (q)
        {
			printf("num: %d systime: %ld can_channel: %d, can_id: %08X", count, q->system_time, q->can_channel, q->frame.can_id);
			for (int i = 0; i < q->frame.can_dlc; i++) {
			    printf(" %02X", q->frame.data[i]);
			}
			printf("\n");        
            q = q->next;
            count++;
        }
    }
    else
    {
        LOG_DEBUG("null element!");
    }

    return RET_OK;
}

// 根据报文ID获取下标映射
int get_map_id2num(vector<UINT> &vec_can_id, UINT frame_id)
{
    int res = RET_ERROR;

	// 遍历容器，寻找frameID
    for(size_t i = 0; i < vec_can_id.size(); i++)
    {
        if(vec_can_id[i] == frame_id)
        {
            res = i;
		    break;
        }
    }

    return res;    
}

// 根据下标映射获取报文ID
UINT get_map_num2id(vector<UINT> &vec_can_id, int num)
{
    int ret = RET_ERROR;

    if(num < 0 || num >= (int)vec_can_id.size())
    {
        LOG_ERROR("invalid num: num[%d] !", num);
        return ret; 
    }

    return vec_can_id[num];        
}


// 比较两个CAN节点是否相似
bool can_node_compare(can_node_t *node0, can_node_t *node1)
{
    if( (node0 != NULL && node1 == NULL) || (node0 == NULL && node1 != NULL) )
    {
        LOG_DEBUG("node0: %p, node1: %p", node0, node1);
		
        return false;
    }
	else if(node0 == NULL && node1 == NULL)
	{
	    return true;
	}
	else
	{
	    // 时间差大于2000us认为是异常
		if(ABS(node1->system_time - node0->system_time) > 2000)
		{
		    LOG_DEBUG("time0: %ld, time1: %ld", node0->system_time, node1->system_time);
			
		    return false;
		}
		
		if(node0->can_channel == node1->can_channel)
		{
		    LOG_DEBUG("channel_0: %d, channel_1: %d", node0->can_channel, node1->can_channel);
			
		    return false;
		}

	    if(node0->frame.can_id != node1->frame.can_id)
	    {
	        LOG_DEBUG("id0: %08X, id1: %08X", node0->frame.can_id, node1->frame.can_id);
			
	        return false;
	    }

	    if(node0->frame.can_dlc != node1->frame.can_dlc)
	    {
	        LOG_DEBUG("dlc0: %d, dlc1: %d", node0->frame.can_dlc, node1->frame.can_dlc);
			
	        return false;
	    }

		for(int i = 0; i < node0->frame.can_dlc; i++)
		{
		    if(node0->frame.data[i] != node1->frame.data[i])
		    {
                LOG_DEBUG("data0: %02X, data1: %02X", node0->frame.data[i], node1->frame.data[i]);
			
		        return false;
		    }
		}	
	}

    return true;
}


// 读取字节指定bit位的值
int read_bit_value(uint8_t *data, int bit_pos)
{
    int ret = 0;           // 返回的结果 

    uint8_t temp_data = *data;

	temp_data = temp_data << (bit_pos);       // 字节左移pos位，将需访问的bit移至最高位以清空多余数值
	temp_data = temp_data >> 7;               // 左移7位，将需访问的bit移至最低位以读取数值

	ret = (int)temp_data;

    return ret;
}

// 给字节指定bit位赋值(这里的value不是0就是1)
int write_bit_value(uint8_t *data, int bit_pos, int value)
{
    if(value == 0)
    {
        uint8_t temp_data = 1;        
	
		temp_data = temp_data << (7-bit_pos);     // 00010000
		temp_data = ~temp_data;                   // ~按位取反
		*data = *data & temp_data;               // &与操作   
    }
	else if(value == 1)
	{
        uint8_t temp_data = 1;
 		temp_data = temp_data << (7-bit_pos);       // 字节左移7-bit_pos位，将需访问的bit移至最高位以清空多余数值
		*data = *data | temp_data;                  // |或操作  	
	}

    return RET_OK;
}


// 读取ID轮廓
int read_id_profile(vector<ID_Profile> &id_profile, vector<UINT> &vec_can_id, CSimpleIniA &configurator)
{
	char section_name[20] = SECTION_NAME_ID_RPOFILE; 					   
	char keys_name[36]; 										   
	char values[1024 * 8];    


	// 读取报文ID列表
	memset(keys_name, 0, sizeof(keys_name));
    memset(values, 0, sizeof(values));
	snprintf(keys_name, sizeof(keys_name), KEY_NAME_CAN_ID_LIST);
	const char *res;
	res = configurator.GetValue(section_name, keys_name);
	if(res != NULL)
	{
		strcpy(values, res);
	}
	else
	{
		LOG_DEBUG(RED "configurator getvalue faild!" NONE );
	
		return RET_ERROR;
	}
	// 使用分割函数
	char *ptr = NULL;
    char *saveptr = NULL;
    char *endptr = NULL;
	ptr = strtok_r(values, ",", &saveptr);
	while(ptr != NULL)
	{
	    UINT can_id = strtol(ptr, &endptr, 16);            // 参数16表示转化为16进制
        vec_can_id.push_back(can_id);
		ptr = strtok_r(NULL, ",", &saveptr);
	}

    int length_of_white_list = (int)vec_can_id.size();
    for(int i = 0; i < length_of_white_list; i++)
    {
        ID_Profile temp_id_profile;
		id_profile.push_back(temp_id_profile);
    }

    // 读取配置文件ID转移矩阵
	for(int i = 0; i < length_of_white_list; i++)
	{
		memset(keys_name, 0, sizeof(keys_name));
		memset(values, 0, sizeof(values));
		snprintf(keys_name, sizeof(keys_name), "0x%08x.legal_next_id", get_map_num2id(vec_can_id, i));
		const char *res;
		res = configurator.GetValue(section_name, keys_name);
		if(res != NULL)
		{
			strcpy(values, res);
		}
		else
		{
			LOG_DEBUG(RED "configurator getvalue faild!" NONE );
		
			return RET_ERROR;
		}
		
		// 使用分割函数
		char *ptr = NULL;
		char *saveptr = NULL;
		char *endptr = NULL;
		ptr = strtok_r(values, ",", &saveptr);
		while(ptr != NULL)
		{
			UINT can_id = strtol(ptr, &endptr, 16); 		   // 参数16表示转化为16进制
			id_profile[i].legal_next_id.push_back(can_id);
			ptr = strtok_r(NULL, ",", &saveptr);
		} 
	
	}
   
    
	// 读取配置文件hamming_dist
	for(int i = 0; i < length_of_white_list; i++)
	{
		memset(keys_name, 0, sizeof(keys_name));
		memset(values, 0, sizeof(values));
		snprintf(keys_name, sizeof(keys_name), "0x%08x.hamming_dist", get_map_num2id(vec_can_id, i));
		const char *res;
        res = configurator.GetValue(section_name, keys_name);
		if(res != NULL)
		{
			strcpy(values, res);
		}
		else
		{
			LOG_DEBUG(RED "configurator getvalue faild!" NONE );
		
			return RET_ERROR;
		}		

		// 使用分割函数
		char *ptr = NULL;
		char *saveptr = NULL;
		char *endptr = NULL;
		ptr = strtok_r(values, ",", &saveptr);
		int min_value = strtol(ptr, &endptr, 10);
		ptr = strtok_r(NULL, ",", &saveptr);
		int max_value = strtol(ptr, &endptr, 10);
		id_profile[i].min_hamming_dist = min_value;
		id_profile[i].max_hamming_dist = max_value;

	}

	// 读取配置文件time_interval
	for(int i = 0; i < length_of_white_list; i++)
	{
		memset(keys_name, 0, sizeof(keys_name));
		memset(values, 0, sizeof(values));
		snprintf(keys_name, sizeof(keys_name), "0x%08x.time_interval", get_map_num2id(vec_can_id, i));
		const char *res;
        res = configurator.GetValue(section_name, keys_name);
		if(res != NULL)
		{
			strcpy(values, res);
		}
		else
		{
			LOG_DEBUG(RED "configurator getvalue faild!" NONE );
		
			return RET_ERROR;
		}		

		// 使用分割函数
		char *ptr = NULL;
		char *saveptr = NULL;
		char *endptr = NULL;
		ptr = strtok_r(values, ",", &saveptr);
		int64_t min_value = strtol(ptr, &endptr, 10);
		ptr = strtok_r(NULL, ",", &saveptr);
		int64_t max_value = strtol(ptr, &endptr, 10);
		id_profile[i].min_time_interval = min_value;
		id_profile[i].max_time_interval = max_value;

	}


    
    return RET_OK;
}

// 打印ID轮廓
int show_id_profile(vector <ID_Profile> & id_profile, vector <UINT> & vec_can_id)
{
    char buffer[65500] = {0};
	
    int length_of_white_list = (int)vec_can_id.size();
	
    // 打印ID转移矩阵
//    for(int i = 0; i < length_of_white_list; i++)
//    {
//	    snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "0x%08x.legal_next_id = ", get_map_num2id(vec_can_id, i));
//
//		int legal_next_id_size = id_profile[i].legal_next_id.size();
//	    for(int j = 0; j < legal_next_id_size; j++)
//		{
//			// 第一次写入
//			if(j == 0)
//			{
//				snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "%x", id_profile[i].legal_next_id[j]);
//			}
//			else
//			{
//				snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), ",%x", id_profile[i].legal_next_id[j]);
//			}			     
//		}    
//
//		snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "\n");
//    } 

    for(int i = 0; i < length_of_white_list; i++)
    {
        for(int j = 0; j < length_of_white_list; j++)
        {
           
            UINT can_id = get_map_num2id(vec_can_id, j);
			
            int find = NOT_EXIST;
			find = get_map_id2num(id_profile[i].legal_next_id, can_id);
			if(find == NOT_EXIST)
			{
			    snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "%d", INVALID);
			}
			else
			{
				snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "%d", VALID);
			}
        }
    
		snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "\n");
    } 
	
    // 打印配置文件hamming距离
    for(int i = 0; i < length_of_white_list; i++)
    {
		snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), 
			"0x%08x.hamming_dist = [%d, %d]\n", get_map_num2id(vec_can_id, i), id_profile[i].min_hamming_dist, id_profile[i].max_hamming_dist);
    } 

    // 打印配置文件time_interval距离
    for(int i = 0; i < length_of_white_list; i++)
    {
		snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), 
			"0x%08x.time_interval = [%ld, %ld]\n", get_map_num2id(vec_can_id, i), id_profile[i].min_time_interval, id_profile[i].max_time_interval);
    } 

	LOG_DEBUG("\n%s", buffer);

    return RET_OK;
}

// 保存轮廓数据
int save_id_profile(vector<UINT> &vec_can_id, vector<ID_Profile> &id_profile, CSimpleIniA & configurator)
{
	// 将字节值相关参数转换成结构化字符串
	char section_name[20] = SECTION_NAME_ID_RPOFILE;       						  // 配置文件中的“节名”
	char keys_name[100] = {0};                                            // 键名
	char values[1024 * 8]= {0};                                           // 键名对应的值

    int length_of_white_list = (int)vec_can_id.size(); 

    // 存储报文ID列表
	memset(keys_name, 0, sizeof(keys_name));
    memset(values, 0, sizeof(values));
	snprintf(keys_name, sizeof(keys_name), KEY_NAME_CAN_ID_LIST);
	
	for(int i = 0; i < length_of_white_list; i++)
	{
	    // 第一次写入
	    if(i == 0)
	    {
	        snprintf(values + strlen(values), sizeof(values) - strlen(values), "%x", vec_can_id[i]);
	    }
		else
		{
			snprintf(values + strlen(values), sizeof(values) - strlen(values), ",%x", vec_can_id[i]);
		}
	    
	}
	configurator.SetValue(section_name, keys_name, values);



    // 保存合法后继ID
	for(int i = 0; i < length_of_white_list; i++)
	{
		memset(keys_name, 0, sizeof(keys_name));
		memset(values, 0, sizeof(values));
	    snprintf(keys_name, sizeof(keys_name), "0x%08x.legal_next_id", get_map_num2id(vec_can_id, i));
		
		int legal_next_id_size = id_profile[i].legal_next_id.size();
		
	    for(int j = 0; j < legal_next_id_size; j++)
		{
			// 第一次写入
			if(j == 0)
			{
				snprintf(values + strlen(values), sizeof(values) - strlen(values), "%x", id_profile[i].legal_next_id[j]);
			}
			else
			{
				snprintf(values + strlen(values), sizeof(values) - strlen(values), ",%x", id_profile[i].legal_next_id[j]);
			}			     
		}
		configurator.SetValue(section_name, keys_name, values);
	}
	
    // 保存hanmming距离信息
	for(int i = 0; i < length_of_white_list; i++)
	{
		memset(keys_name, 0, sizeof(keys_name));
		memset(values, 0, sizeof(values));
		snprintf(keys_name, sizeof(keys_name), "0x%08x.hamming_dist", get_map_num2id(vec_can_id, i));
		snprintf(values, sizeof(values), "%d,%d", id_profile[i].min_hamming_dist, id_profile[i].max_hamming_dist);
		configurator.SetValue(section_name, keys_name, values);
	}

    // 保存时间间隔信息
	for(int i = 0; i < length_of_white_list; i++)
	{
		memset(keys_name, 0, sizeof(keys_name));
		memset(values, 0, sizeof(values));
		snprintf(keys_name, sizeof(keys_name), "0x%08x.time_interval", get_map_num2id(vec_can_id, i));
		snprintf(values, sizeof(values), "%ld,%ld", id_profile[i].min_time_interval, id_profile[i].max_time_interval);
		configurator.SetValue(section_name, keys_name, values);
	}


    return RET_OK;
}

// 合法ID后继检测器
int id_matrix_detect(ID_Profile &id_profile, UINT can_id, int &is_anomaly)
{
    int find_flag = NOT_EXIST;

	find_flag = get_map_id2num(id_profile.legal_next_id, can_id);

	if(find_flag == NOT_EXIST)
	{
	    is_anomaly = ABNORMAL;
	}
	else
	{
	    is_anomaly = NORMAL;
	}

    return RET_OK;
}

// 计算两个8字节报文汉明距离
// @ 输入两个字节序列
int calculate_hamming_distance(uint8_t *data_previous, uint8_t *data_current)
{
    int res = 0;    

    for(int i = 0; i < FRAME_DLC; i++)           // 8个字节
    {
        for(int j = 0; j < FRAME_DLC; j++)      // 1个字节8个bit位
        {
            int value_a = read_bit_value(&data_previous[i], j);
			int value_b = read_bit_value(&data_current[i], j);

			if(value_a != value_b)
			{
			    res++;
			}
        }
    }

    return res;
}


// hamming距离检测器
int hamming_distance_detect(ID_Profile &id_profile, BYTE *current_payload, int &is_anomaly)
{
    if(id_profile.previous_payload_valid == VALID)
    {
		int hamming_dist = calculate_hamming_distance(id_profile.previous_payload, current_payload);
		
		if(hamming_dist >= id_profile.min_hamming_dist && hamming_dist <= id_profile.max_hamming_dist)
		{
			is_anomaly = NORMAL;
		}
		else
		{
			LOG_DEBUG(RED "hamming_dist[%d] over range[%d, %d] !" NONE, hamming_dist, id_profile.min_hamming_dist, id_profile.max_hamming_dist);
		
			is_anomaly = ABNORMAL;
		}
    }
	
    // 更新存储的新负载
	id_profile.previous_payload_valid = VALID;
	for(int i = 0; i < FRAME_DLC; i++)
	{
	    id_profile.previous_payload[i] = current_payload[i];
	}

    return RET_OK;
}



// 时间间隔检测器
int time_interval_detect(ID_Profile &id_profile, int64_t current_time, int &is_anomaly)
{
    if(id_profile.previous_time_valid == VALID)
    {
		int time_interval = current_time - id_profile.previous_time;
		
		if(time_interval >= id_profile.min_time_interval && time_interval <= id_profile.max_time_interval)
		{
			is_anomaly = NORMAL;			
		}
		else
		{
			LOG_DEBUG(RED "time_interval[%ld] over range[%ld, %ld] !" NONE, time_interval, id_profile.min_time_interval, id_profile.max_time_interval);		
			is_anomaly = ABNORMAL;
		}
    }
    // 更新上一次报文到达时间
	id_profile.previous_time_valid = VALID;
    id_profile.previous_time = current_time;


    return RET_OK;
}


// 解析从csv文件读取的CAN报文
// timestamp,id,data0 data1 data2 data3 data4 data5 data6 data 7
int parse_read_buffer(char *data_buffer, int64_t *timestamp, struct can_frame *new_can)
{
	char *ptr = NULL;
    char *saveptr = NULL;
    char *endptr = NULL;
	// 报文时间戳
	ptr = strtok_r(data_buffer, ",", &saveptr);             
	*timestamp = strtol(ptr, &endptr, 10);            // 参数10表示转化为10进制

	// 报文ID
	ptr = strtok_r(NULL, ",", &saveptr);
	new_can->can_id = strtol(ptr, &endptr, 16);            // 参数16表示转化为16进制

    // 报文负载，8字节值
	ptr = strtok_r(NULL, " ", &saveptr);                  // 跳过第一个空格
	int byte_pos = 0;
	while(ptr != NULL)
	{
	    new_can->data[byte_pos] = strtol(ptr, &endptr, 16);            // 参数16表示转化为16进制
		ptr = strtok_r(NULL, " ", &saveptr);
		byte_pos++;
	}    

    return 0;
}

// 将CAN报文以固定格式存储到文件中
int save_read_buffer(FILE *data_file, int64_t *timestamp, struct can_frame *new_can)
{
	fprintf(data_file, "%ld,%08x,", *timestamp, new_can->can_id);
	for(int i = 0; i < FRAME_DLC; i++)
	{
		if(i == 0)
		{
			fprintf(data_file, "%02x", new_can->data[i]);
		}
		else
		{
			fprintf(data_file, " %02x", new_can->data[i]);
		}
	}
	fprintf(data_file, "\n"); 

    return RET_OK;
}




