/*
 * @file cli.c
 * @date Created on: Apr 16, 2024
 * @author AngelRobotics FW Team
 * @brief
 * @ref
 */

#include "cli.h"

#ifdef _USE_DEBUG_CLI


/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */

#define CLI_KEY_BACK              0x08
#define CLI_KEY_DEL               0x7F
#define CLI_KEY_ENTER             0x0D
#define CLI_KEY_ESC               0x1B
#define CLI_KEY_LEFT              0x44
#define CLI_KEY_RIGHT             0x43
#define CLI_KEY_UP                0x41
#define CLI_KEY_DOWN              0x42
#define CLI_KEY_HOME              0x31
#define CLI_KEY_END               0x34

#define CLI_ARGS_MAX              32					// parameter 최대 갯수
#define CLI_PRINT_BUF_MAX         256					// cli_printf buffer 최대 갯수


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

typedef enum
{
  CLI_RX_NORMAL,						// special key 가 아닌 경우,
  CLI_RX_SP_KEY1,
  CLI_RX_SP_KEY2,
  CLI_RX_SP_KEY3,
  CLI_RX_SP_KEY4,						// 최대 4개까지 입력 받음 (VT100 기준)
} cli_rx_sp_key;						// special key (home, delete, enter 등..) 처리를 위한 enum index


typedef struct
{
  char   cmd_str[CLI_CMD_NAME_MAX];		//명령어 string 배열 (이름 최대 길이)
  void (*cmd_func)(cli_args_t *);		//명령어 string 분석 후 실행할 함수 포인터 (callback 함수)
} cli_cmd_t;							// cli command 구조체


typedef struct
{
  uint8_t buf[CLI_LINE_BUF_MAX];		// cli line 입력 buffer
  uint8_t buf_len;						// line buffer 길이
  uint8_t cursor;						// line cursor 위치
  uint8_t count;						// line 입력 갯수
} cli_line_t;							// cli 라인 입력 구조체


typedef struct
{
  bool     	is_busy;							// cli is busy?
  uint8_t  	state;								// cli state
  char     	print_buffer[CLI_PRINT_BUF_MAX];	// cli_printf 출력 buffer
  uint16_t  argc;								// 가변인자 갯수
  char     *argv[CLI_ARGS_MAX];					// 가변인자 문자열


  bool        hist_line_new;					// line history 처리 변수
  int8_t      hist_line_i;						// line history 처리 변수
  uint8_t     hist_line_last;					// line history 처리 변수
  uint8_t     hist_line_count;					// line history 처리 변수

  cli_line_t  line_buf[CLI_LINE_HIS_MAX];		// line buffer
  cli_line_t  line;								// 현재 line

  uint16_t    cmd_count;						// 현재 저장된 명령어 갯수
  cli_cmd_t   cmd_list[CLI_CMD_LIST_MAX];		// 현재 저장된 명령어 리스트
  cli_args_t  cmd_args;							// cmd 구조체
} cli_t;

cli_t   cli_node;

/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */




/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* CLI Internal CMD */
static bool cliUpdate(cli_t *p_cli, uint8_t rx_data);
static void cliShowPrompt(cli_t *p_cli);
static bool cliRunCmd(cli_t *p_cli);

/* CLI Line Control */
static void cliLineClean(cli_t *p_cli);
static void cliLineAdd(cli_t *p_cli);
static void cliLineChange(cli_t *p_cli, int8_t key_up);

/* CLI utils */
static void cliToUpper(char *str);
static bool cliParseArgs(cli_t *p_cli);

/* CLI Get & Transform Data */
static int32_t  cliArgsGetData(uint8_t index);
static float    cliArgsGetFloat(uint8_t index);
static char    *cliArgsGetStr(uint8_t index);
static bool     cliArgsCmpStr(uint8_t index, const char *p_str);



/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */


bool CLI_Init(void)
{
	cli_node.is_busy = false;
	cli_node.state   = CLI_RX_NORMAL;

	cli_node.hist_line_i     = 0;
	cli_node.hist_line_last  = 0;
	cli_node.hist_line_count = 0;
	cli_node.hist_line_new   = false;

	cli_node.cmd_args.getData  = cliArgsGetData;
	cli_node.cmd_args.getFloat = cliArgsGetFloat;
	cli_node.cmd_args.getStr   = cliArgsGetStr;
	cli_node.cmd_args.cmpStr    = cliArgsCmpStr;

	cliLineClean(&cli_node);

	CLI_CMDAdd("show", cliShowList);
	CLI_CMDAdd("cls" , cliClearScreen);
	CLI_CMDAdd("md"  , cliMemoryDump);
	CLI_CMDAdd("off" , cliSystemOff);
	CLI_CMDAdd("\r\n" , cliSystemOff);
	CLI_CMDAdd("<Module List>" , cliSystemOff);

	return true;
}

//uint8_t test_cli_rx_byte = 0;

bool CLI_Run(void)
{
	uint8_t rx_byte;

	if (IOIF_USBD_BufferIsAvailable() > 0)				//USB RX ring buffer 에 수신된 바이트가 있을 경우
	{
			IOIF_USBD_BufferByteRead(&rx_byte, 1);			//read RX byte from ring buffer
			cliUpdate(&cli_node, rx_byte);

//		IOIF_USBD_BufferByteRead(&test_cli_rx_byte, 1);			//read RX byte from ring buffer
//		cliUpdate(&cli_node, test_cli_rx_byte);
	}

	return true;
}


uint32_t CLI_IsAvailable(void)
{
	return IOIF_USBD_BufferIsAvailable();
}


bool CLI_RunStr(const char *fmt, ...)
{
	bool ret;
	va_list arg;
	va_start (arg, fmt);
	cli_t *p_cli = &cli_node;

	vsnprintf((char *)p_cli->line.buf, CLI_LINE_BUF_MAX, fmt, arg);
	va_end (arg);

	ret = cliRunCmd(p_cli);

	return ret;
}


void CLI_Printf(const char *fmt, ...)
{

	char buf[256] = {0,};

	va_list arg;
	va_start (arg, fmt);
	int32_t len;
	memcpy(buf, cli_node.print_buffer, sizeof(buf));

	len = vsnprintf(buf, 256, fmt, arg);
	va_end (arg);

	IOIF_USBD_Write((uint8_t *)buf, len, 1000);
	CLI_Delay(1);
}


bool CLI_KeepLoop(void)
{
  if (IOIF_USBD_BufferIsAvailable() == 0)			//key 입력값이 없으면, loop 를 유지
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool CLI_CMDAdd(const char *cmd_str, void (*p_func)(cli_args_t *))
{
	bool ret = true;
	cli_t *p_cli = &cli_node;
	uint16_t index;

	if (p_cli->cmd_count >= CLI_CMD_LIST_MAX)
	{
		return false;
	}

	index = p_cli->cmd_count;

	strcpy(p_cli->cmd_list[index].cmd_str, cmd_str);
	p_cli->cmd_list[index].cmd_func = p_func;

	cliToUpper(p_cli->cmd_list[index].cmd_str);

	p_cli->cmd_count++;

	return ret;
}



void CLI_Delay(uint32_t ms)
{
#ifdef _USE_OS_RTOS
		osDelay(ms);
#else
		HAL_Delay(ms);
#endif
}


void cliClearScreen(cli_args_t *args)
{
    IOIF_USBD_Printf((uint8_t *)"\033[2J");			//clear screen
    CLI_Delay(1);
    IOIF_USBD_Printf((uint8_t *)"\033[H");			//home
    CLI_Delay(1);
}



void cliShowList(cli_args_t *args)
{
	cli_t *p_cli = &cli_node;


	CLI_Printf("\r\n");
	CLI_Printf("These indicates which hardware modules are currently supported: \r\n");
	CLI_Printf("\r\n");
	CLI_Printf("---------- <Command List> ---------\r\n");

	for (int i=0; i<p_cli->cmd_count; i++)
	{
		CLI_Printf(p_cli->cmd_list[i].cmd_str);
		CLI_Printf("\r\n");
	}

	CLI_Printf("-----------------------------------\r\n");
}


void cliMemoryDump(cli_args_t *args)
{
	int idx, size = 16;
	unsigned int *addr;
	int idx1, i;
	unsigned int *ascptr;
	unsigned char asc[4];

	int    argc = args->argc;
	char **argv = args->argv;


	if(args->argc < 1)				// help
	{
		CLI_Printf("- Usage : md [addr(hex)] [size] \r\n");
		CLI_Printf("- a functions that performs memory dump.\r\n");
		CLI_Printf("- example : md 0x8000000 12 \n");
		return;
	}

	if(argc > 1)
	{
		size = (int)strtoul((const char * ) argv[1], (char **)NULL, (int) 0);
	}
	addr   = (unsigned int *)strtoul((const char * ) argv[0], (char **)NULL, (int) 0);
	ascptr = (unsigned int *)addr;

	CLI_Printf("\n   ");
	for (idx = 0; idx<size; idx++)
	{
		if((idx%4) == 0)
		{
			CLI_Printf(" 0x%08X: ", (unsigned int)addr);
		}
		CLI_Printf(" 0x%08X", *(addr));

		if ((idx%4) == 3)
		{
			CLI_Printf ("  |");
			for (idx1= 0; idx1< 4; idx1++)
			{
				memcpy((char *)asc, (char *)ascptr, 4);
				for (i=0;i<4;i++)
				{
					if (asc[i] > 0x1f && asc[i] < 0x7f)
					{
						CLI_Printf("%c", asc[i]);
					}
					else
					{
						CLI_Printf(".");
					}
				}
				ascptr+=1;
			}
			CLI_Printf("|\r\n   ");

			CLI_Delay(1);
		}
		addr++;
	}
}

void cliSystemOff()
{
	CLI_Printf(" System off in 1s. \r\n");
	CLI_Delay(800);
	HAL_GPIO_WritePin(GPIOG, MCU_SW_CLR_Pin, GPIO_PIN_RESET);
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */


static bool cliUpdate(cli_t *p_cli, uint8_t rx_data)
{
  bool ret = false;
  uint8_t tx_buf[8];
  cli_line_t *line;

  line = &p_cli->line;


  if (p_cli->state == CLI_RX_NORMAL)
  {
    switch(rx_data)
    {
      // Key Input : Enter
      //
      case CLI_KEY_ENTER:
        if (line->count > 0)
        {
          cliLineAdd(p_cli);
          cliRunCmd(p_cli);
        }

        line->count = 0;
        line->cursor = 0;
        line->buf[0] = 0;
        cliShowPrompt(p_cli);
        break;


      case CLI_KEY_ESC:
        p_cli->state = CLI_RX_SP_KEY1;
        break;


      // Key Input : Delete
      //
      case CLI_KEY_DEL:
        if (line->cursor < line->count)
        {
          uint8_t mov_len;

          mov_len = line->count - line->cursor;
          for (int i=1; i<mov_len; i++)
          {
            line->buf[line->cursor + i - 1] = line->buf[line->cursor + i];
          }

          line->count--;
          line->buf[line->count] = 0;

          IOIF_USBD_Printf((uint8_t *)"\x1B[1P");
          CLI_Delay(1);
        }
        break;


      // Key Input : Backspace
      //
      case CLI_KEY_BACK:
        if (line->count > 0 && line->cursor > 0)
        {
          if (line->cursor == line->count)
          {
            line->count--;
            line->buf[line->count] = 0;
          }

          if (line->cursor < line->count)
          {
            uint8_t mov_len;

            mov_len = line->count - line->cursor;

            for (int i=0; i<mov_len; i++)
            {
              line->buf[line->cursor + i - 1] = line->buf[line->cursor + i];
            }

            line->count--;
            line->buf[line->count] = 0;
          }
        }

        if (line->cursor > 0)
        {
          line->cursor--;
          IOIF_USBD_Printf((uint8_t *)"\b \b\x1B[1P");
          CLI_Delay(1);
        }
        break;


      default:
        if ((line->count + 1) < line->buf_len)
        {
          if (line->cursor == line->count)
          {
            //IOIF_USBD_Write(&rx_data, 1, 1000);
        	  IOIF_USBD_Printf( (uint8_t*)"%c", rx_data);
        	  CLI_Delay(1);


            line->buf[line->cursor] = rx_data;
            line->count++;
            line->cursor++;
            line->buf[line->count] = 0;
          }
          if (line->cursor < line->count)
          {
            uint8_t mov_len;

            mov_len = line->count - line->cursor;
            for (int i=0; i<mov_len; i++)
            {
              line->buf[line->count - i] = line->buf[line->count - i - 1];
            }
            line->buf[line->cursor] = rx_data;
            line->count++;
            line->cursor++;
            line->buf[line->count] = 0;

            IOIF_USBD_Printf((uint8_t *)"\x1B[4h%c\x1B[4l", rx_data);
            CLI_Delay(1);
          }
        }
        break;
    }
  }

  switch(p_cli->state)
  {
    case CLI_RX_SP_KEY1:
      p_cli->state = CLI_RX_SP_KEY2;
      break;

    case CLI_RX_SP_KEY2:
      p_cli->state = CLI_RX_SP_KEY3;
      break;

    case CLI_RX_SP_KEY3:
      p_cli->state = CLI_RX_NORMAL;

      if (rx_data == CLI_KEY_LEFT)
      {
        if (line->cursor > 0)
        {
          line->cursor--;
          tx_buf[0] = 0x1B;
          tx_buf[1] = 0x5B;
          tx_buf[2] = rx_data;
          IOIF_USBD_Write(tx_buf, 3, 1000);
          CLI_Delay(1);
        }
      }

      if (rx_data == CLI_KEY_RIGHT)
      {
        if (line->cursor < line->count)
        {
          line->cursor++;

          tx_buf[0] = 0x1B;
          tx_buf[1] = 0x5B;
          tx_buf[2] = rx_data;
          IOIF_USBD_Write(tx_buf, 3, 1000);
          CLI_Delay(1);
        }
      }

      if (rx_data == CLI_KEY_UP)
      {
        cliLineChange(p_cli, true);
        IOIF_USBD_Printf((uint8_t *)p_cli->line.buf);
        CLI_Delay(1);
      }

      if (rx_data == CLI_KEY_DOWN)
      {
        cliLineChange(p_cli, false);
        IOIF_USBD_Printf((uint8_t *)p_cli->line.buf);
        CLI_Delay(1);
      }

      if (rx_data == CLI_KEY_HOME)
      {
        IOIF_USBD_Printf((uint8_t *)"\x1B[%dD", line->cursor);
        line->cursor = 0;

        p_cli->state = CLI_RX_SP_KEY4;
        CLI_Delay(1);
      }

      if (rx_data == CLI_KEY_END)
      {
        uint16_t mov_len;

        if (line->cursor < line->count)
        {
          mov_len = line->count - line->cursor;
          IOIF_USBD_Printf((uint8_t *)"\x1B[%dC", mov_len);
          CLI_Delay(1);
        }
        if (line->cursor > line->count)
        {
          mov_len = line->cursor - line->count;
          IOIF_USBD_Printf((uint8_t *)"\x1B[%dD", mov_len);
          CLI_Delay(1);
        }
        line->cursor = line->count;
        p_cli->state = CLI_RX_SP_KEY4;
      }
      break;

    case CLI_RX_SP_KEY4:
      p_cli->state = CLI_RX_NORMAL;
      break;
  }

  return ret;
}



static void cliShowPrompt(cli_t *p_cli)
{
	IOIF_USBD_Printf((uint8_t *)"\n\r");
	CLI_Delay(1);
	IOIF_USBD_Printf((uint8_t *)"[cli@AGR ]# ");
	CLI_Delay(1);
}


static bool cliRunCmd(cli_t *p_cli)
{
  bool ret = false;


  if (cliParseArgs(p_cli) == true)
  {
    //CLI_Printf("\r\n");
	IOIF_USBD_Printf((uint8_t*) "\r\n");
    CLI_Delay(1);

    cliToUpper(p_cli->argv[0]);

    p_cli->is_busy = true;
    for (int i=0; i<p_cli->cmd_count; i++)
    {
      if (strcmp(p_cli->argv[0], p_cli->cmd_list[i].cmd_str) == 0)
      {
        p_cli->cmd_args.argc =  p_cli->argc - 1;
        p_cli->cmd_args.argv = &p_cli->argv[1];
        p_cli->cmd_list[i].cmd_func(&p_cli->cmd_args);

        break;
      }
    }
    p_cli->is_busy = false;
  }

  return ret;
}



static void cliLineClean(cli_t *p_cli)
{
	p_cli->line.count   = 0;
	p_cli->line.cursor  = 0;
	p_cli->line.buf_len = CLI_LINE_BUF_MAX - 1;
	p_cli->line.buf[0]  = 0;
}


static void cliLineAdd(cli_t *p_cli)
{

	p_cli->line_buf[p_cli->hist_line_last] = p_cli->line;

	if (p_cli->hist_line_count < CLI_LINE_HIS_MAX)
	{
		p_cli->hist_line_count++;
	}

	p_cli->hist_line_i    = p_cli->hist_line_last;
	p_cli->hist_line_last = (p_cli->hist_line_last + 1) % CLI_LINE_HIS_MAX;
	p_cli->hist_line_new  = true;
}



static void cliLineChange(cli_t *p_cli, int8_t key_up)
{
	uint8_t change_i;


	if (p_cli->hist_line_count == 0)
	{
		return;
	}


	if (p_cli->line.cursor > 0)
	{
		IOIF_USBD_Printf((uint8_t *)"\x1B[%dD", p_cli->line.cursor);
		CLI_Delay(1);
	}
	if (p_cli->line.count > 0)
	{
		IOIF_USBD_Printf((uint8_t *)"\x1B[%dP", p_cli->line.count);
		CLI_Delay(1);
	}


	if (key_up == true)
	{
		if (p_cli->hist_line_new == true)
		{
			p_cli->hist_line_i = p_cli->hist_line_last;
		}
		p_cli->hist_line_i = (p_cli->hist_line_i + p_cli->hist_line_count - 1) % p_cli->hist_line_count;
		change_i = p_cli->hist_line_i;
	}
	else
	{
		p_cli->hist_line_i = (p_cli->hist_line_i + 1) % p_cli->hist_line_count;
		change_i = p_cli->hist_line_i;
	}

	p_cli->line = p_cli->line_buf[change_i];
	p_cli->line.cursor = p_cli->line.count;

	p_cli->hist_line_new = false;
}


//static bool cliParseArgs(cli_t *p_cli)			// origin code
//{
//	bool ret = false;
//	char *tok;
//	char *next_ptr;
//	uint16_t argc = 0;
//	static const char *delim = " \f\n\r\t\v";
//	char *cmdline;
//	char **argv;
//
//	p_cli->argc = 0;
//
//	cmdline = (char *)p_cli->line.buf;
//	argv    = p_cli->argv;
//
//	argv[argc] = NULL;
//
//	for (tok = strtok_r(cmdline, delim, &next_ptr); tok; tok = strtok_r(NULL, delim, &next_ptr))
//	{
//		argv[argc++] = tok;
//	}
//
//	p_cli->argc = argc;
//
//	if (argc > 0)
//	{
//		ret = true;
//	}
//
//	return ret;
//}




static bool cliParseArgs(cli_t *p_cli)
{
    bool ret = false;
    char *tok;
    char *next_ptr;
    uint16_t argc = 0;
    static const char *delim = " \f\n\r\t\v";
    char *cmdline;
    char **argv;

    bool sp_parse = false;

    if (!p_cli || !p_cli->line.buf || !p_cli->argv)
        return false; // 유효성 검사

    p_cli->argc = 0;

    cmdline = (char*)p_cli->line.buf;
    argv    = p_cli->argv;

    argv[argc] = NULL;

    tok = strtok_r(cmdline, delim, &next_ptr);

    while (tok != NULL) {

    	if(sp_parse == true){
    		argv[argc] = tok;
    		next_ptr++;								// 공백 이동
    		sp_parse = false;
    	}
    	else {
    		argv[argc] = tok; 						// 일반 토큰
    	}
    	argc++;
    	if (*next_ptr == '\"')
    	{
    		tok = strtok_r(NULL, "\"", &next_ptr);
    		sp_parse = true;

    	}
    	else
    		tok = strtok_r(NULL, delim, &next_ptr); // 다음 토큰으로 이동
    }

    p_cli->argc = argc;
    if (argc > 0) {
    	ret = true;
    }

    return ret;
}



static void cliToUpper(char *str)
{
	uint16_t i;
	uint8_t  str_ch;

	for (i=0; i<CLI_CMD_NAME_MAX; i++)
	{
		str_ch = str[i];

		if (str_ch == 0)
		{
			break;
		}

		if ((str_ch >= 'a') && (str_ch <= 'z'))
		{
			str_ch = str_ch - 'a' + 'A';
		}
		str[i] = str_ch;
	}

	if (i == CLI_CMD_NAME_MAX)
	{
		str[i-1] = 0;
	}
}


static int32_t cliArgsGetData(uint8_t index)
{
	int32_t ret = 0;
	cli_t *p_cli = &cli_node;


	if (index >= p_cli->cmd_args.argc)
	{
		return 0;
	}

	ret = (int32_t)strtoul((const char * ) p_cli->cmd_args.argv[index], (char **)NULL, (int) 0);

	return ret;
}

static float cliArgsGetFloat(uint8_t index)
{
	float ret = 0.0;
	cli_t *p_cli = &cli_node;


	if (index >= p_cli->cmd_args.argc)
	{
		return 0;
	}

	ret = (float)strtof((const char * ) p_cli->cmd_args.argv[index], (char **)NULL);

	return ret;
}

static char *cliArgsGetStr(uint8_t index)
{
	char *ret = NULL;
	cli_t *p_cli = &cli_node;


	if (index >= p_cli->cmd_args.argc)
	{
		return 0;
	}

	ret = p_cli->cmd_args.argv[index];

	return ret;
}

static bool cliArgsCmpStr(uint8_t index, const char *p_str)
{
	bool ret = false;
	cli_t *p_cli = &cli_node;


	if (index >= p_cli->cmd_args.argc)
	{
		return 0;
	}

	if(strcmp(p_str, p_cli->cmd_args.argv[index]) == 0)
	{
		ret = true;
	}

	return ret;
}


#endif  /* _USE_DEBUG_CLI */
