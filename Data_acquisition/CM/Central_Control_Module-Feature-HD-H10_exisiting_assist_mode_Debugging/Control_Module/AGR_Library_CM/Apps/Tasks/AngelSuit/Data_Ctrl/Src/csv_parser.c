#include "csv_parser.h"

#ifdef SUIT_MINICM_ENABLED

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */




/**
 *------------------------------------------------------------
 *                          VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

uint8_t ms_id = 0;

extern RobotSettingFileInfo RS_File;			// Info. Structure : Robot Setting File
extern FSMFileInfo 			FSM_File;			// Info. Structure : FSM File
extern DMSFileInfo 			DMS_File;			// Info. Structure : DMS File
extern MotionMapFileInfo 	MotionMap_File;		// Info. Structure : Motion Map File


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */


static int Contents_FileProcessing(uint8_t contents_rx_data[64]);

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

IOIF_fCMD_res_t tReadRes = FR_NOT_READY;
IOIF_fCMD_res_t tCloseRes = FR_NOT_READY;
uint32_t CF_rByte = 0;

IOIF_fCMD_res_t tReadRes2 = FR_NOT_READY;
//char test_tbuf[100];
//bool eof = false;
static uint32_t total_rByte = 0;

int ParseOneRow_CSV(IOIF_FILE_t* FileObject, uint8_t* Filename, char* dest)
{
    //IOIF_fCMD_res_t tReadRes = FR_NOT_READY;
    bool quotation = false;
    bool linefeed = false;
    bool f_err = false;
    char buf[CONTENT_CELL_MAX] = {0,};
    int idx = 0;

    //static uint32_t total_rByte = 0;

//    tReadRes2 = IOIF_fgets(test_tbuf, sizeof(test_tbuf) ,FileObject);

    while (true && (idx < CONTENT_CELL_MAX))
    {
    	//buf[idx] = fgetc(fp);
    	if(IOIF_fRead(FileObject, &buf[idx], 1, &CF_rByte) == FR_OK)
    	{
    		total_rByte += CF_rByte;
    		f_lseek(FileObject, total_rByte);
    	}
    	else
    	{
    		f_err = true;				// return file error
    		break;
    	}

    	if (buf[idx] == '"')
    	{
    		if (quotation == false)
    		{
    			quotation = true;
    			continue;
    		}
    		else
    		{
    			quotation = false;
    			continue;
    		}
    	}
    	else if ((buf[idx] == ',') && (quotation == false))
    	{
    		break;
    	}

    	else if((buf[idx] == '\r'))
    	{
    		continue;			//ignore carraige return
    	}

    	else if (buf[idx] == '\n')
    	{
    		if (quotation == false)
    		{
    			linefeed = true;
    			break;
    		}
    		else
    			continue;
    	}
    	idx++;
    }

    memset(dest, 0x0, CONTENT_CELL_MAX);
    strncpy(dest, buf, idx);

    if (f_err)
    	return CONTENT_FILE_ERR;
    if (linefeed)
    	return LINE_ENDING;

    return PARSE_SUCCESS;
}

int create_content_info_packet(uint8_t* buf, CONTENT_TYPE type, uint8_t robot_id, contents_file_version_t file_ver)
{
    int crc_sum = 0;
    memset(buf, 0x0, FSM_BUF_SIZE);
    if ((type >= ROBOT_SETTING) && (type <= MOTION_MAP)) {
        buf[CONTENT_FILE_ID_IDX] = (uint8_t)type;
        buf[CONTENT_DATA_ID_IDX] = 0x01;
        buf[CONTENT_ROBOT_ID_IDX] = robot_id;
        buf[CONTENT_FILE_VERSION_IDX] = file_ver.major;
        buf[CONTENT_FILE_VERSION_IDX +1] = file_ver.minor;
        buf[CONTENT_FILE_VERSION_IDX +2] = file_ver.patch;
        for (int i = 0; i < CONTENT_CRC_IDX; i++)
            crc_sum += (int)buf[i];

        buf[CONTENT_CRC_IDX] = (uint8_t) ((crc_sum >> 24) & 0xff);
        buf[CONTENT_CRC_IDX+1] = (uint8_t) ((crc_sum >> 16) & 0xff);
        buf[CONTENT_CRC_IDX+2] = (uint8_t) ((crc_sum >> 8) & 0xff);
        buf[CONTENT_CRC_IDX+3] = (uint8_t) (crc_sum & 0xff);
    } else
        return -1;

    return 0;
}


int create_mm_data_packet(uint8_t* buf, char(*csv_read)[CONTENT_CELL_MAX], int row, int mm_index)
{
    int crc_sum = 0;
    memset(buf, 0x0, FSM_BUF_SIZE);

    if (row < MOTION_MAP_START_IDX)
        return -1;

    // FILE ID
    buf[MM_FILE_ID_IDX] = (uint8_t)MOTION_MAP;
    // DATA ID
    buf[MM_DATA_ID_IDX] = (uint8_t)mm_index + 1;
        
    // MS ID
    uint8_t motion_id = (uint8_t)atoi(csv_read[0]);
    if (motion_id != 0)
        ms_id = motion_id;
    buf[MM_MS_ID_IDX] = ms_id;
    // MAX vector length
    buf[MM_MAX_P_V_LENGTH_IDX] = 10;
    buf[MM_MAX_F_V_LENGTH_IDX] = 10;
    buf[MM_MAX_I_V_LENGTH_IDX] = 10;

    // MD ID
    if ((csv_read[2][0] == 'M') && (csv_read[2][1] == 'D')) {
        //"0" = 0x30, 문자를 숫차로 치환
    	if(csv_read[2][3] >= 0x30)		//MD10, MD11, ...
    		buf[MM_MD_ID_IDX] = ((uint8_t) csv_read[2][2] - 0x30) * 10 + ((uint8_t) csv_read[2][3] - 0x30);
    	else
    		buf[MM_MD_ID_IDX] = (uint8_t) csv_read[2][2] - 0x30;

//        if(buf[MM_MD_ID_IDX] >= 0)
//        {
//        	//test breakpoint
//        }

    }

    if (mm_index == 1) {
        // Ctrl setting (Gain)
        buf[MM_FF_IDX] = (uint8_t)atoi(csv_read[3]);
        buf[MM_PD_IDX] = (uint8_t)atoi(csv_read[4]);
        buf[MM_IC_IDX] = (uint8_t)atoi(csv_read[5]);
        buf[MM_DOB_IDX] = (uint8_t)atoi(csv_read[6]);
        buf[MM_IRC_IDX] = (uint8_t)atoi(csv_read[7]);
        buf[MM_FC_IDX] = (uint8_t)atoi(csv_read[8]);
    } else {
        // Motion Map protocol(byte array)의 vector index 는 9부터 시작하고, 6개로 분할하여 byte array 를 각각 전송
        // 각 byte array 는 mm_index(Motion Map index_ 로 구분하고, index 에 따라 vector 의 범위를 구분
        // 각 vector 는 motion map csv 파일의 column index 와 일대일 매칭
        // 각 vector 는 bytearray 로 변환할 경우 6byte 의 값을 가지기 때문 byte array 내에 6byte 단위로 구분
        // mm_index        vector            column index(csv)     index of each vector(byte array)
        // 2               P[0]~P[4]         9  ~  13              7(P[0]), 13, 19, 25, 31(P[4])
        // 3               P[5]~P[9]         14 ~  18              7(P[5]), 13, 19, 25, 31(P[9])
        // 4               F[0]~F[4]         19 ~  23              7(F[0]), 13, 19, 25, 31(F[4])
        // 5               F[5]~F[9]         24 ~  28              7(F[5]), 13, 19, 25, 31(F[9])
        // 6               I[0]~I[4]         29 ~  33              7(I[0]), 13, 19, 25, 31(I[4])
        // 7               I[5]~I[9]         34 ~  38              7(I[5]), 13, 19, 25, 31(I[9])
        int column_index = 9 + (5 * (mm_index - 2));
        char *token; char *next_ptr;
        int token_index = 0;
        // byte array 내의 각 vector 는 6 byte, 
        // byte_array_vector_index는 byte array 내의 vector index(0~4, motion map byte array 내의 vector는 5개로 제한됨)
        // 각 vector에 해당하는 csv 파일의 값을 읽어와 벡터를 구성하는 요소에 해당하는 byte의 수만큼 변환하여 저장
        for (int byte_array_vector_index = 0; byte_array_vector_index < 5; byte_array_vector_index++) {
            // column index(csv) 에서 각 vector 의 값을 읽어옴, 각 vector 를 구성하는 값은 ',' 로 구분됨 (Ex. P vector : [100,2000,0,0])
            // token_index 는 각 vector의 byte array(6 bytes) index, token 은 vector 의 각 요소를 ',' 를 바탕으로 구분한 각 요소의 값
            token_index = 0; 
            //token = strtok(csv_read[column_index++], ",");
            token = strtok_r(csv_read[column_index++], ",", &next_ptr);
            while (token != NULL) {
                if (token_index >= 8)
                    break;
                // P vector [yd(2bytes), L(2bytes), s0(1byte), sd(1byte)]
                if (mm_index < 4) {
                    // yd(2bytes), L(2bytes)
                	if (token_index >= 6)
                		break;
                	else if (token_index < 4) {
                        int value = (uint16_t)atoi(token);
                        buf[MM_VECTOR_IDX + byte_array_vector_index*6 + token_index] = (uint8_t)((value >> 8) & 0xff);
                        buf[MM_VECTOR_IDX + byte_array_vector_index*6 + token_index + 1] = (uint8_t)(value & 0xff);
                        //token = strtok(NULL, ",");
                        token = strtok_r(NULL, ",", &next_ptr);
                        token_index += 2;
                    } else { // s0(1byte), sd(1byte)
                        buf[MM_VECTOR_IDX + byte_array_vector_index*6 + token_index] = (uint8_t)atoi(token);
                        //token = strtok(NULL, ",");
                        token = strtok_r(NULL, ",", &next_ptr);
                        token_index += 1;
                    }
                } else if (mm_index < 6) { // F vector [mode_idx(2bytes), coefficient(2bytes), global variable index(2bytes), delay(2bytes)]
                    if (token_index >= 8)
                    	break;
					int value = (uint16_t)atoi(token);
					buf[MM_VECTOR_IDX + byte_array_vector_index*8 + token_index] = (uint8_t)((value >> 8) & 0xff);
					buf[MM_VECTOR_IDX + byte_array_vector_index*8 + token_index + 1] = (uint8_t)(value & 0xff);
					//token = strtok(NULL, ",");
					token = strtok_r(NULL, ",", &next_ptr);
					token_index += 2;
                } else { // I vector [epsilon(1byte), Kp(1byte), Kd(1byte), lambda(1byte), duration(2bytes)]
                    // epsilon(1byte), Kp(1byte), Kd(1byte), lambda(1byte)
                	if (token_index >= 6)
                		break;
                	else if (token_index < 4) {
                        buf[MM_VECTOR_IDX + byte_array_vector_index*6 + token_index] = (uint8_t)atoi(token);
                        //token = strtok(NULL, ",");
                        token = strtok_r(NULL, ",", &next_ptr);
                        token_index += 1;
                    } else { // duration(2bytes)
                        int value = (uint16_t)atoi(token);
                        buf[MM_VECTOR_IDX + byte_array_vector_index*6 + token_index] = (uint8_t)((value >> 8) & 0xff);
                        buf[MM_VECTOR_IDX + byte_array_vector_index*6 + token_index + 1] = (uint8_t)(value & 0xff);
                        //token = strtok(NULL, ",");
                        token = strtok_r(NULL, ",", &next_ptr);
                        token_index += 2;
                    }
                }
            }
        }
    }

    // CRC
    for (int i = 0; i < FSM_CRC_IDX; i++)
        crc_sum += (int)buf[i];
    buf[MM_CRC_IDX] = (uint8_t) ((crc_sum >> 24) & 0xff);
    buf[MM_CRC_IDX+1] = (uint8_t) ((crc_sum >> 16) & 0xff);
    buf[MM_CRC_IDX+2] = (uint8_t) ((crc_sum >> 8) & 0xff);
    buf[MM_CRC_IDX+3] = (uint8_t) (crc_sum & 0xff);

    return 0;
}

//IOIF_fCMD_res_t tOpenRes = FR_NOT_READY;
//uint8_t content_info_packet[FSM_BUF_SIZE];
//uint8_t content_data_packet[FSM_BUF_SIZE];

//char parse_buf[CONTENT_COLUMN_MAX][CONTENT_CELL_MAX];


int ReadContentFile(IOIF_FILE_t* FileObject, uint8_t* Filename, CONTENT_TYPE type)
{
	IOIF_fCMD_res_t tOpenRes = FR_NOT_READY;
	//int status = -1;
	uint8_t robot_id = 0;
	//uint8_t file_version = 0;
	int csv_row = 0;
	int csv_column = 0;
	char parse_buf[CONTENT_COLUMN_MAX][CONTENT_CELL_MAX];
	tOpenRes = IOIF_FileOpenReadOnly(FileObject, Filename);

	contents_file_version_t contents_file_version;		//contents file version structure

	if (tOpenRes != FR_OK) {
		// printf("file[%s] open error\n", file_name);
		return tOpenRes;
	}

	while (1) {
		int status = ParseOneRow_CSV(FileObject, Filename, parse_buf[csv_column]);
		// CSV 의 하나의 행(row)의 파싱이 종료되면 parse_buf 의 내용으로 byte array 를 생성

		if (status == LINE_ENDING) {
			if (csv_row == CSV_ROBOT_ID_ROW) {
				robot_id = (uint8_t)atoi(parse_buf[1]);
				// printf("robot_id = %d\n", robot_id);
			} else if (csv_row == CSV_FILE_VERSION_ROW) {
				//file_version = (uint8_t)atoi(parse_buf[1]);
				parse_version((const char*)parse_buf[1], &contents_file_version);
				// printf("file_version = %d\n", file_version);
				uint8_t content_info_packet[FSM_BUF_SIZE];
				if(create_content_info_packet(content_info_packet, type, robot_id, contents_file_version) == 0) {
//					for (int i = 0; i < FSM_BUF_SIZE; i++) {
//						// printf("%x ", content_info_packet[i]);
//					}
					// printf("\n");
					Contents_FileProcessing(content_info_packet);
					//ToDo. send content_info_packet(byte array) to CM
				}
			} else {
				uint8_t content_data_packet[FSM_BUF_SIZE];
				// Motion Map 은 csv 파일의 하나의 row 를 7개의 byte array 로 분할
				if (type == MOTION_MAP) {
					for (int mm_index = 1; mm_index < 8; mm_index++) {
						if (create_mm_data_packet(content_data_packet, parse_buf, csv_row, mm_index) == 0) {
//							for (int i=0; i<FSM_BUF_SIZE; i++) {
//								//    printf("%x ", content_data_packet[i]);
//							}
//							// printf("\n");
							//ToDo. send content_data_packet(byte array) to CM
							Contents_FileProcessing(content_data_packet);
						} else
							break;
					}
				} else if (create_content_data_packet(content_data_packet, parse_buf, csv_row, type) == 0) {
//					for (int i = 0; i < FSM_BUF_SIZE; i++) {
//						// printf("%x ", content_data_packet[i]);
//					}
					// printf("\n");
					//ToDo. send content_data_packet(byte array) to CM
					Contents_FileProcessing(content_data_packet);
				}
			}

			csv_row++;
			csv_column = 0;
			continue;

		} else if (status == CONTENT_FILE_ERR){		// return error

			//IOIF_USBD_Printf((uint8_t*)"File Error!\r\n");
			break;

			//} else if (status == END_OF_FILE) { // CSV 파일의 읽기가 끝나면 종료
		} else if (f_eof(FileObject) != 0) { 	  // CSV 파일의 읽기가 끝나면 종료
			// printf("Done parsing CSV file\n");
			total_rByte = 0;

			break;
		}
		csv_column++;
	}
	// fclose(fp);

	tCloseRes = IOIF_FileClose(FileObject);

//	if(!status && !tCloseRes &&!tOpenRes)
//		status == READ_CONTENTS_OK;
//	else
//		status == READ_CONTENTS_ERROR;

	return 0;
}


int create_content_data_packet(uint8_t *buf, char (*csv_read)[CONTENT_CELL_MAX], int row, CONTENT_TYPE type) {
	int crc_sum = 0;
	memset(buf, 0x0, FSM_BUF_SIZE);

	char *next_ptr;
	char *next_ptr2;

	if ((type == ROBOT_SETTING) && (row >= ROBOT_SETTING_START_IDX)) {
		// FILE ID
		buf[RS_FILE_ID_IDX] = (uint8_t) type;
		// DATA ID
		buf[RS_DATA_ID_IDX] = 0x02;
		// ROW index
		int row_index = row - ROBOT_SETTING_START_IDX;
		buf[RS_CURRENT_ROW_IDX] = (uint8_t) ((row_index >> 8) & 0xff);
		buf[RS_CURRENT_ROW_IDX + 1] = (uint8_t) (row_index & 0xff);
		// 장비 사용 여부
		if ((csv_read[1][0] == 'O') || (csv_read[1][0] == 'o'))
			buf[RS_USAGE_IDX] = 0x01;
		else
			buf[RS_USAGE_IDX] = 0x00;
		// 장비 ID
		if (strcmp(csv_read[2], RS_ID_FIXED) == 0) {
			buf[RS_DEVICE_ID_IDX] = 0x00;
			buf[RS_DEVICE_ID_IDX] = 0x01;
		} else if ((csv_read[2][0] == '0') && (csv_read[2][1] == 'x')) {
			int num = strtol(csv_read[2], NULL, 16);
			buf[RS_DEVICE_ID_IDX] = (uint8_t) ((num >> 8) & 0xff);
			buf[RS_DEVICE_ID_IDX + 1] = (uint8_t) (num & 0xff);
		}
		// FDCAN Channel
		if (csv_read[3][0] == NOT_AVAILABLE)
			buf[RS_CH_IDX] = 0;
		else if ((csv_read[3][0] >= '0') && (csv_read[3][0] <= '9'))
			buf[RS_CH_IDX] = (uint8_t) atoi(csv_read[3]);
		// Name
		memcpy(&buf[RS_NAME_IDX], &csv_read[4][0], 20);

		// CRC
		for (int i = 0; i < RS_CRC_IDX; i++)
			crc_sum += (int) buf[i];

		buf[RS_CRC_IDX] = (uint8_t) ((crc_sum >> 24) & 0xff);
		buf[RS_CRC_IDX + 1] = (uint8_t) ((crc_sum >> 16) & 0xff);
		buf[RS_CRC_IDX + 2] = (uint8_t) ((crc_sum >> 8) & 0xff);
		buf[RS_CRC_IDX + 3] = (uint8_t) (crc_sum & 0xff);
	} else if (((type == OPERATION_FSM) && (row >= FSM_START_IDX))
			|| ((type == SERVICE_FSM) && (row >= FSM_START_IDX))) {
		// FILE ID
		buf[FSM_FILE_ID_IDX] = (uint8_t) type;
		// DATA ID
		buf[FSM_DATA_ID_IDX] = 0x02;
		// ROW index
		int row_index = row - FSM_START_IDX;
		buf[FSM_CURRENT_ROW_IDX] = (uint8_t) ((row_index >> 8) & 0xff);
		buf[FSM_CURRENT_ROW_IDX + 1] = (uint8_t) (row_index & 0xff);
		// State ID
		buf[FSM_STATE_ID_IDX] = (uint8_t) atoi(csv_read[0]);
		// MOTION SET ID
		buf[FSM_MOTION_SET_ID_IDX] = (uint8_t) atoi(csv_read[2]);
		// ACTION (LED ID)
		if (csv_read[3][0] == NOT_AVAILABLE)
			buf[FSM_ACTION_LED_ID_IDX] = 255;
		else if ((csv_read[3][0] >= '0') && (csv_read[3][0] <= '9'))
			buf[FSM_ACTION_LED_ID_IDX] = (uint8_t) atoi(csv_read[3]);
		// ACTION (AUDIO ID)
		if (csv_read[4][0] == NOT_AVAILABLE)
			buf[FSM_ACTION_AUDIO_ID_IDX] = 255;
		else if ((csv_read[4][0] >= '0') && (csv_read[4][0] <= '9'))
			buf[FSM_ACTION_AUDIO_ID_IDX] = (uint8_t) atoi(csv_read[4]);
		// ACTION (TABLET Message ID)
		if (csv_read[5][0] == NOT_AVAILABLE)
			buf[FSM_ACTION_TABLET_MSG_ID_IDX] = 255;
		else if ((csv_read[5][0] >= '0') && (csv_read[5][0] <= '9'))
			buf[FSM_ACTION_TABLET_MSG_ID_IDX] = (uint8_t) atoi(csv_read[5]);
		// ACTION (Empty)
		if (csv_read[6][0] == NOT_AVAILABLE)
			buf[FSM_ACTION_EMPTY_ID_IDX] = 255;
		else if ((csv_read[6][0] >= '0') && (csv_read[6][0] <= '9'))
			buf[FSM_ACTION_EMPTY_ID_IDX] = (uint8_t) atoi(csv_read[6]);
		// Tablet mode id
		buf[FSM_TABLET_MODE_ID_IDX] = (uint8_t) atoi(csv_read[7]);
		// Timeout
		int timeout = (uint16_t) atoi(csv_read[8]);
		buf[FSM_TIMEOUT_IDX] = (uint8_t) ((timeout >> 8) & 0xff);
		buf[FSM_TIMEOUT_IDX + 1] = (uint8_t) (timeout & 0xff);
		// Default Target State ID
		buf[FSM_DEFAULT_TARGET_STATE_IDX] = (uint8_t) atoi(csv_read[9]);
		// Exit Condition ID
		//char *token = strtok(csv_read[10], ",");

		char *token = strtok_r(csv_read[10], ",", &next_ptr);

		int token_index = 0;
		while (token != NULL) {
			if (token_index >= 20)
				break;
			buf[FSM_EXIT_CONDITION_IDX + token_index] = (uint8_t) atoi(token);
			//token = strtok(NULL, ",");
			token = strtok_r(NULL, ",", &next_ptr);
			token_index++;
		}
		// Target State ID

		//char *token2 = strtok(csv_read[11], ",");
		char *token2 = strtok_r(csv_read[11], ",", &next_ptr2);

		token_index = 0;
		while (token2 != NULL) {
			if (token_index >= 20)
				break;
			buf[FSM_TARGET_STATE_IDX + token_index] = (uint8_t) atoi(token2);
			token2 = strtok_r(NULL, ",", &next_ptr2);
			token_index++;
		}

		// CRC
		for (int i = 0; i < FSM_CRC_IDX; i++)
			crc_sum += (int) buf[i];

		buf[FSM_CRC_IDX] = (uint8_t) ((crc_sum >> 24) & 0xff);
		buf[FSM_CRC_IDX + 1] = (uint8_t) ((crc_sum >> 16) & 0xff);
		buf[FSM_CRC_IDX + 2] = (uint8_t) ((crc_sum >> 8) & 0xff);
		buf[FSM_CRC_IDX + 3] = (uint8_t) (crc_sum & 0xff);

	} else if ((type == DMS) && (row >= DMS_START_IDX)) {
		// FILE ID
		buf[DMS_FILE_ID_IDX] = (uint8_t) type;
		// DATA ID
		buf[DMS_DATA_ID_IDX] = 0x02;
		// ROW index
		int row_index = row - DMS_START_IDX;
		buf[DMS_CURRENT_ROW_IDX] = (uint8_t) ((row_index >> 8) & 0xff);
		buf[DMS_CURRENT_ROW_IDX + 1] = (uint8_t) (row_index & 0xff);
		// Data 측정 여부
		if ((csv_read[3][0] == 'O') || (csv_read[3][0] == 'o'))
			buf[DMS_CHECK_FOR_MEASUREMENT_IDX] = 0x01;
		else
			buf[DMS_CHECK_FOR_MEASUREMENT_IDX] = 0x00;
		// 측정 장비 명 (AM, CM..), 장비별 값 정의 하지 않아 0으로 함
		buf[DMS_MEASURING_DEVICE_IDX] = 0x00;
		// CM에 Data 저장여부(USB, SD card 등)
		if ((csv_read[5][0] == 'O') || (csv_read[5][0] == 'o'))
			buf[DMS_SAVE_TO_CM_IDX] = 0x01;
		else
			buf[DMS_SAVE_TO_CM_IDX] = 0x00;
		// AM에 실시간 Data 전송 여부
		if ((csv_read[6][0] == 'O') || (csv_read[6][0] == 'o'))
			buf[DMS_SEND_TO_AM_IDX] = 0x01;
		else
			buf[DMS_SEND_TO_AM_IDX] = 0x00;

		// CRC
		for (int i = 0; i < DMS_CRC_IDX; i++)
			crc_sum += (int) buf[i];

		buf[DMS_CRC_IDX] = (uint8_t) ((crc_sum >> 24) & 0xff);
		buf[DMS_CRC_IDX + 1] = (uint8_t) ((crc_sum >> 16) & 0xff);
		buf[DMS_CRC_IDX + 2] = (uint8_t) ((crc_sum >> 8) & 0xff);
		buf[DMS_CRC_IDX + 3] = (uint8_t) (crc_sum & 0xff);
	} else {
		return -1;
	}

	return 0;
}


void parse_version(const char* version_str, contents_file_version_t* filever) {

    char buffer[16];

    strncpy(buffer, version_str, sizeof(buffer));
    buffer[sizeof(buffer) - 1] = '\0';  				// null-terminated

    char* token = strtok(buffer, ".");
    if (token != NULL) filever->major = (uint8_t)atoi(token);

    token = strtok(NULL, ".");
    if (token != NULL) filever->minor = (uint8_t)atoi(token);

    token = strtok(NULL, ".");
    if (token != NULL) filever->patch = (uint8_t)atoi(token);
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static int equality;

static uint32_t flag4;
static uint8_t Ending_Message[64];
static uint8_t RxData_Pre[64];

static int Contents_FileProcessing(uint8_t contents_rx_data[64])
{
	//flag4 = 0;

	for(int i = 0; i < 64; i++) {
		Ending_Message[i] = 63 - i;
	}

	for(int i = 0; i < 64; i++) {
		if (contents_rx_data[i] == Ending_Message[i]) {
			flag4 += 1;
		}
	}
	if (flag4 == 64) {
		equality = 1;
	}
	else {
		equality = 0;
	}

/*********************************(Robot Setting File)******************************************/

	if (contents_rx_data[0] == ID_ROBOT_SETTING) {             // Robot setting data
		if (contents_rx_data[1] == CONTENTSFILE_HEADER) {         // Initiator packet for robot setting data
			RS_File.robot_id     = contents_rx_data[2];
			RS_File.file_version_major = contents_rx_data[3];
			RS_File.file_version_minor = contents_rx_data[4];
			RS_File.file_version_patch = contents_rx_data[5];

		}
		else if (contents_rx_data[1] == CONTENTSFILE_DATA) {    // Data packet for robot setting data
			uint16_t t_row = (contents_rx_data[2] << 8) | (contents_rx_data[3]);
			RS_File.vec[t_row].usage    = contents_rx_data[4];
			RS_File.vec[t_row].FDCAN_ID = (contents_rx_data[5] << 8) | (contents_rx_data[6]);
			RS_File.vec[t_row].FDCAN_CH = contents_rx_data[7];
			for (int i = 0; i < 20; i++) {
				RS_File.vec[t_row].name[i] = contents_rx_data[i+8];
			}
		}
	}
	/* Save & Download : Robot Setting Files */
	//	if ((RxData_Pre[0] == 1) && (equality == 1))
	//	{
	//		Save_RobotSetting();
	//		Download_RobotSetting();
	//	}

///*********************************(FSM File)******************************************/

	if (contents_rx_data[0] == ID_OPERATIONAL_FSM) {             // FSM data
		if (contents_rx_data[1] == CONTENTSFILE_HEADER) {         // Initiator packet for FSM data
			FSM_File.robot_id =     contents_rx_data[2];
			FSM_File.file_version_major = contents_rx_data[3];
			FSM_File.file_version_minor = contents_rx_data[4];
			FSM_File.file_version_patch = contents_rx_data[5];
		}
		else if (contents_rx_data[1] == CONTENTSFILE_DATA) {    // Data packet for FSM data
			uint16_t t_row = (contents_rx_data[2] << 8) | (contents_rx_data[3]);
			FSM_File.vec[t_row].StateID              = contents_rx_data[4];
			FSM_File.vec[t_row].MotionSetID          = contents_rx_data[5];
			/* Action ID : LED ID, Audio ID, Tablet Msg, Empty */
			FSM_File.vec[t_row].ActionID             = (contents_rx_data[6] << 24) | (contents_rx_data[7] << 16) | (contents_rx_data[8] << 8) | (contents_rx_data[9]);
			FSM_File.vec[t_row].TabletModeID         = contents_rx_data[10];
			FSM_File.vec[t_row].TimeOut              = (contents_rx_data[11] << 8) | (contents_rx_data[12]);
			FSM_File.vec[t_row].DefaultTargetStateID = contents_rx_data[13];
			for (int i = 0; i < MAX_N_TRANSITION; i++) {
				FSM_File.vec[t_row].ExitConditionID[i] = contents_rx_data[i+14];
			}
			for (int i = 0; i < MAX_N_TRANSITION; i++) {
				FSM_File.vec[t_row].TargetStateID[i] = contents_rx_data[i+34];
			}
		}
	}
	//        if ((RxData_Pre[0] == 2) & (equality == 1)) {   // End of transmission and saving
	//        	Save_FSM1();
	//        	Download_FSM1();
	//        }

/*********************************(DMS File)******************************************/

        if (contents_rx_data[0] == ID_DATA_MANAGEMENT_SETTING) {             // DMS data
        	if (contents_rx_data[1] == CONTENTSFILE_HEADER) {         // Initiator packet for DMS data
        		DMS_File.robot_id =     contents_rx_data[2];
        		DMS_File.file_version_major = contents_rx_data[3];
        		DMS_File.file_version_minor = contents_rx_data[4];
        		DMS_File.file_version_patch = contents_rx_data[5];
        	}
        	else if (contents_rx_data[1] == CONTENTSFILE_DATA) {    // Data packet for DMS data
        		uint16_t t_row = (contents_rx_data[2] << 8) | (contents_rx_data[3]);
        		DMS_File.vec[t_row].Enable      = contents_rx_data[4];
        		DMS_File.vec[t_row].DeviceID    = contents_rx_data[5];
        		DMS_File.vec[t_row].CM_Save_Opt = contents_rx_data[6];
        		DMS_File.vec[t_row].AM_Send_Opt = contents_rx_data[7];
        	}
        }
//        if ((RxData_Pre[0] == 4) & (equality == 1)) {   // End of transmission and saving
//        	Save_DMS();
//        	Download_DMS();
//        }
//

/*********************************(MotionMap File)******************************************/

        if (contents_rx_data[0] == ID_MOTION_MAP) {               // Motion Map data
        	if (contents_rx_data[1] == 1) {           // Initiator packet for Motion Map data
        		MotionMap_File.robot_id =     contents_rx_data[2];
        		MotionMap_File.file_version_major = contents_rx_data[3];
        		MotionMap_File.file_version_minor = contents_rx_data[4];
        		MotionMap_File.file_version_patch = contents_rx_data[5];
        	}
        	else if (contents_rx_data[1] == 2) {      // Control setting packet for Motion Map data
        		uint16_t t_row_MS = contents_rx_data[2]-1;
        		uint16_t t_row_MD = contents_rx_data[6]-1;
        		MotionMap_File.MS[t_row_MS].MS_ID             = contents_rx_data[2];
        		MotionMap_File.MS[t_row_MS].max_p_vectors_len = contents_rx_data[3];
        		MotionMap_File.MS[t_row_MS].max_f_vectors_len = contents_rx_data[4];
        		MotionMap_File.MS[t_row_MS].max_i_vectors_len = contents_rx_data[5];
        		MotionMap_File.MS[t_row_MS].MD[t_row_MD].c_vector[0].FF_gain  = contents_rx_data[7];
        		MotionMap_File.MS[t_row_MS].MD[t_row_MD].c_vector[0].PD_gain  = contents_rx_data[8];
        		MotionMap_File.MS[t_row_MS].MD[t_row_MD].c_vector[0].IC_gain = contents_rx_data[9];
        		MotionMap_File.MS[t_row_MS].MD[t_row_MD].c_vector[0].DOB_gain  = contents_rx_data[10];
        		MotionMap_File.MS[t_row_MS].MD[t_row_MD].c_vector[0].IRC_gain = contents_rx_data[11];
        		MotionMap_File.MS[t_row_MS].MD[t_row_MD].c_vector[0].FC_gain = contents_rx_data[12];
        	}
        	else if (contents_rx_data[1] == 3) {
        		uint16_t t_row_MS = contents_rx_data[2]-1;
        		uint16_t t_row_MD = contents_rx_data[6]-1;
        		MotionMap_File.MS[t_row_MS].MS_ID             = contents_rx_data[2];
        		MotionMap_File.MS[t_row_MS].max_p_vectors_len = contents_rx_data[3];
        		MotionMap_File.MS[t_row_MS].max_f_vectors_len = contents_rx_data[4];
        		MotionMap_File.MS[t_row_MS].max_i_vectors_len = contents_rx_data[5];
        		for (int i = 0; i < HALF_N_P_VECTORS; i++) {
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].p_vector[i].yd = (contents_rx_data[6*i+7] << 8) | (contents_rx_data[6*i+8]);
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].p_vector[i].L  = (contents_rx_data[6*i+9] << 8) | (contents_rx_data[6*i+10]);
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].p_vector[i].s0 = contents_rx_data[6*i+11];
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].p_vector[i].sd = contents_rx_data[6*i+12];
        		}

        	}
        	else if (contents_rx_data[1] == 4) {
        		uint16_t t_row_MS = contents_rx_data[2]-1;
        		uint16_t t_row_MD = contents_rx_data[6]-1;
        		MotionMap_File.MS[t_row_MS].MS_ID             = contents_rx_data[2];
        		MotionMap_File.MS[t_row_MS].max_p_vectors_len = contents_rx_data[3];
        		MotionMap_File.MS[t_row_MS].max_f_vectors_len = contents_rx_data[4];
        		MotionMap_File.MS[t_row_MS].max_i_vectors_len = contents_rx_data[5];
        		for (int i = HALF_N_P_VECTORS; i < MAX_N_P_VECTORS; i++) {
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].p_vector[i].yd = (contents_rx_data[6*(i-5)+7] << 8) | (contents_rx_data[6*(i-5)+8]);
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].p_vector[i].L  = (contents_rx_data[6*(i-5)+9] << 8) | (contents_rx_data[6*(i-5)+10]);
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].p_vector[i].s0 = contents_rx_data[6*(i-5)+11];
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].p_vector[i].sd = contents_rx_data[6*(i-5)+12];
        		}

        	}
        	else if (contents_rx_data[1] == 5) {
        		uint16_t t_row_MS = contents_rx_data[2]-1;
        		uint16_t t_row_MD = contents_rx_data[6]-1;
        		MotionMap_File.MS[t_row_MS].MS_ID             = contents_rx_data[2];
        		MotionMap_File.MS[t_row_MS].max_p_vectors_len = contents_rx_data[3];
        		MotionMap_File.MS[t_row_MS].max_f_vectors_len = contents_rx_data[4];
        		MotionMap_File.MS[t_row_MS].max_i_vectors_len = contents_rx_data[5];
        		for (int i = 0; i < HALF_N_F_VECTORS; i++) {
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].f_vector[i].mode_idx				= (contents_rx_data[8*i+7] << 8) | (contents_rx_data[8*i+8]);
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].f_vector[i].coefficient			= (contents_rx_data[8*i+9] << 8)  | (contents_rx_data[8*i+10]);
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].f_vector[i].global_variable_index	= (contents_rx_data[8*i+11] << 8) | (contents_rx_data[8*i+12]);
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].f_vector[i].delay					= (contents_rx_data[8*i+13] << 8) | (contents_rx_data[8*i+14]);
        		}

        	}
        	else if (contents_rx_data[1] == 6) {
        		uint16_t t_row_MS = contents_rx_data[2]-1;
        		uint16_t t_row_MD = contents_rx_data[6]-1;
        		MotionMap_File.MS[t_row_MS].MS_ID             = contents_rx_data[2];
        		MotionMap_File.MS[t_row_MS].max_p_vectors_len = contents_rx_data[3];
        		MotionMap_File.MS[t_row_MS].max_f_vectors_len = contents_rx_data[4];
        		MotionMap_File.MS[t_row_MS].max_i_vectors_len = contents_rx_data[5];
        		for (int i = HALF_N_F_VECTORS; i < MAX_N_F_VECTORS; i++) {
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].f_vector[i].mode_idx				= (contents_rx_data[8*(i-5)+7] << 8) | (contents_rx_data[8*(i-5)+8]);
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].f_vector[i].coefficient			= (contents_rx_data[8*(i-5)+9] << 8)  | (contents_rx_data[8*(i-5)+10]);
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].f_vector[i].global_variable_index	= (contents_rx_data[8*(i-5)+11] << 8) | (contents_rx_data[8*(i-5)+12]);
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].f_vector[i].delay					= (contents_rx_data[8*(i-5)+13] << 8) | (contents_rx_data[8*(i-5)+14]);
        		}

        	}
        	else if (contents_rx_data[1] == 7) {
        		uint16_t t_row_MS = contents_rx_data[2]-1;
        		uint16_t t_row_MD = contents_rx_data[6]-1;
        		MotionMap_File.MS[t_row_MS].MS_ID             = contents_rx_data[2];
        		MotionMap_File.MS[t_row_MS].max_p_vectors_len = contents_rx_data[3];
        		MotionMap_File.MS[t_row_MS].max_f_vectors_len = contents_rx_data[4];
        		MotionMap_File.MS[t_row_MS].max_i_vectors_len = contents_rx_data[5];
        		for (int i = 0; i < HALF_N_I_VECTORS; i++) {
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].i_vector[i].epsilon_target = contents_rx_data[6*i+7];
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].i_vector[i].Kp_target      = contents_rx_data[6*i+8];
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].i_vector[i].Kd_target      = contents_rx_data[6*i+9];
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].i_vector[i].lambda_target  = contents_rx_data[6*i+10];
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].i_vector[i].duration       = (contents_rx_data[6*i+11] << 8) | (contents_rx_data[6*i+12]);
        		}

        	}
        	else if (contents_rx_data[1] == 8) {
        		uint16_t t_row_MS = contents_rx_data[2]-1;
        		uint16_t t_row_MD = contents_rx_data[6]-1;
        		MotionMap_File.MS[t_row_MS].MS_ID             = contents_rx_data[2];
        		MotionMap_File.MS[t_row_MS].max_p_vectors_len = contents_rx_data[3];
        		MotionMap_File.MS[t_row_MS].max_f_vectors_len = contents_rx_data[4];
        		MotionMap_File.MS[t_row_MS].max_i_vectors_len = contents_rx_data[5];
        		for (int i = HALF_N_I_VECTORS; i < MAX_N_I_VECTORS; i++) {
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].i_vector[i].epsilon_target = contents_rx_data[6*(i-5)+7];
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].i_vector[i].Kp_target      = contents_rx_data[6*(i-5)+8];
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].i_vector[i].Kd_target      = contents_rx_data[6*(i-5)+9];
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].i_vector[i].lambda_target  = contents_rx_data[6*(i-5)+10];
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].i_vector[i].duration       = (contents_rx_data[6*(i-5)+11] << 8) | (contents_rx_data[6*(i-5)+12]);
        		}

        	}
        }
//        if ((RxData_Pre[0] == 5) & (equality == 1)) {   // End of transmission and saving
//        	flag3 += 1;

//        	Save_MotionMap();
//        	Download_MotionMap();
//        	Get_Max_PFI_Vectors_Length();
//
//        }
//
        if (contents_rx_data[0] != 255) {
        	memcpy(RxData_Pre, contents_rx_data,64);
        }

        return 0;
}




#endif /* SUIT_MINICM_ENABLED */
