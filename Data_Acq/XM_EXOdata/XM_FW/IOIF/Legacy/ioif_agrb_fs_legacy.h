#include "ioif_agrb_defs.h"

#if !defined(AGRB_IOIF_FILESYSTEM_DISABLE)

#ifndef _IOIF_AGRB_FILESYSTEM_EX_H_
#define _IOIF_AGRB_FILESYSTEM_EX_H_

#include <stdint.h>
#include <stdbool.h> 

#include "fatfs.h" //For handle

#if defined(HAL_HCD_MODULE_ENABLED)
#include "usbh_diskio.h"
#endif

#if (_USE_LFN == 0)
#error "_USE_LFN must be set to 1 or 2 to use ioif_agrb_filesystem module for long file name support"
#endif

//STM32H7의 FATS는 하나의 인스턴스만 지원하므로 핸들러에 ID를 별도로 지정하지 않음
//다만, 열려있는 파일에 대한 접근은 ID로 구분함

typedef uint32_t IOIF_FILEx_t; //File handle ID type

#define IOIF_FILESYSTEM_INVALID_FILE_ID    (0xDEADF11E) //Invalid file ID

//#define IOIF_FILESYSTEM_USE_DEBUG_LOGGING    //Enable debug logging for filesystem operations

#define IOIF_FILESYSTEM_DEFAULT_ROOTPATH  "0" //Default root path for SD card

#define IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH   (256) //폴더 + '/' + 파일이름 + '.' + 확장자 + NULL 문자 포함
#define IOIF_FILESYSTEM_MAX_FULLPATH_LENGTH   (160) //NULL 문자 포함
#define IOIF_FILESYSTEM_MAX_FOLDER_LENGTH     (128) //NULL 문자 포함
#define IOIF_FILESYSTEM_MAX_FILENAME_LENGTH   (64) //확장자 제외, NULL 문자 포함
#define IOIF_FILESYSTEM_MAX_POSTFIX_LENGTH    (8)  //파일명 뒤에 붙는 번호 등, NULL 문자 포함
#define IOIF_FILESYSTEM_MAX_EXTENSION_LENGTH  (16)  //NULL 문자 포함

#define IOIF_FILESYSTEM_READ_BOUNCE_BUFFER_SIZE    (4096) //4KB
#define IOIF_FILESYSTEM_WRITE_BOUNCE_BUFFER_SIZE   (4096) //4KB
#if (_FS_LOCK != 0 )
#define IOIF_FILESYSTEM_MAX_OPENED_FILES          (_FS_LOCK)   /*최대 8개 파일까지 열 수 있음*/
#else
#define IOIF_FILESYSTEM_MAX_OPENED_FILES          (8)   /* 최대 8 */
#endif

#define IOIF_FILESYSTEM_BACKUPFILE_EXTENSION            "bak" //백업 파일 확장자
#define IOIF_FILESYSTEM_BACKUPFILE_EXTENSION_DOT        ".bak" //백업 파일 확장자 (점 포함)
#define IOIF_FILESYSTEM_BACKUPFILE_EXTENSION_LENGTH   (5) //NULL 문자 포함

#define IOIF_FILESYSTEM_MAX_FILECOUNT_PER_PATH   (128) //하나의 경로에 최대 N개 파일까지 생성가능
#define IOIF_FILESYSTEM_MAX_FILE_INCREMENT_TRY    (64) //파일 이름 자동 증가 시도 최대 횟수

//Singletone structure for filesystem driver
//온갖 콜백 함수를 여기에 만든다

typedef enum
{
    IOIF_FileSystem_Access_UNKNOWN = 0,
    IOIF_FileSystem_Access_READONLY,    //읽기 전용
    //IOIF_SDCard_Acess_WRITE,       //덮어쓰기. 지원하지 않음. STREAM으로 할거 아니면 사용하지 말 것
    IOIF_FileSystem_Access_APPEND,      //파일 끝에 추가... 

} IOIF_FileSystem_AccessMode_e;

typedef enum
{
    IOIF_FileSystem_CreateMode_UNKNOWN = 0,

    IOIF_FileSystem_CreateMode_EXCLUSIVE,       //파일이 존재하면 생성 거부
    IOIF_FileSystem_CreateMode_APPEND,              //파일이 존재하면 기존 파일 뒤에 이어서 작업
    IOIF_FileSystem_CreateMode_OVERWRITE,           //파일이 존재하면 지우고 새로 만듦
    IOIF_FileSystem_CreateMode_INCREMENT,           //파일이 존재하면 끝에 postfix로 번호를 붙여 새로 만듦 (예: test.txt -> test_1.txt)

} IOIF_FileSystem_CreateMode_e;

typedef enum
{
    IOIF_FileSystem_DeviceType_SDCard,
    IOIF_FileSystem_DeviceType_USBH_MSC, //USB Host Mass Storage Class

} IOIF_FileSystem_DeviceType_e;

typedef enum
{
    IOIF_FileSystem_OK,
    IOIF_FileSystem_DRIVER_ERROR,
    IOIF_FileSystem_PARAM_ERROR,
    IOIF_FileSystem_PATH_INVALID,
    IOIF_FileSystem_INTERNAL,

    IOIF_FileSystem_SEMAPHORE_NOT_INITIALIZED,
    IOIF_FileSystem_SEMAPHORE_ERROR,

    IOIF_FileSystem_TIMEOUT,
    IOIF_FileSystem_OVERWRITE_PROTECTED,
    IOIF_FileSystem_NOT_EXIST,
    IOIF_FileSystem_SIZE_EXCEED,
    IOIF_FileSystem_FILECOUNT_EXCEED,
    IOIF_FileSystem_FILEINSTANCE_FULL,
    IOIF_FileSystem_FILEINSTANCE_INVALID,
    IOIF_FileSystem_ACCESS_MODE_INVALID,
    IOIF_FileSystem_ACCESS_ALREADY_OPENED,

} IOIF_FileSystem_Error_e;

typedef enum
{
    IOIF_FileSystem_NamingRule_OK = 0,
    IOIF_FileSystem_NamingRule_PARAM_ERROR,
    IOIF_FileSystem_NamingRule_INVALID_LENGTH,
    IOIF_FileSystem_NamingRule_INVALID_CHARACTER,
    IOIF_FileSystem_NamingRule_CONSECUTIVE_SLASHES,
    IOIF_FileSystem_NamingRule_START_WITH_DOT,
    IOIF_FileSystem_NamingRule_END_WITH_DOT,
    IOIF_FileSystem_NamingRule_NOT_START_WITH_SLASH,
    IOIF_FileSystem_NamingRule_END_WITH_SLASH,
    IOIF_FileSystem_NamingRule_START_WITH_SPACE,
    IOIF_FileSystem_NamingRule_END_WITH_SPACE,

} IOIF_FileSystem_NamingRule_e;

typedef IOIF_FileSystem_Error_e AGRBFileSystemStatusDef;

typedef struct
{
    // file count
    uint32_t block_size; //in bytes
    uint32_t total_blocks;
    uint32_t free_blocks;

    uint32_t total_size; //in bytes
    uint32_t free_size;  //in bytes

    bool is_initialized;
    
} IOIF_FileSystem_Inspect_t;


typedef struct {    
    IOIF_FileSystem_DeviceType_e device_type;

    AGRBFileSystemStatusDef (*init)(IOIF_FileSystem_DeviceType_e device_type);
    AGRBFileSystemStatusDef (*deinit)(void);
    AGRBFileSystemStatusDef (*open_readonly)(IOIF_FILEx_t* id, const char* path, FRESULT* out_fresult);
    AGRBFileSystemStatusDef (*open_write)(IOIF_FILEx_t* id, const char* path, IOIF_FileSystem_CreateMode_e mode, FRESULT* out_fresult);
    AGRBFileSystemStatusDef (*write)(IOIF_FILEx_t id, const void* data, uint32_t size, FRESULT* out_fresult);
    AGRBFileSystemStatusDef (*read)(IOIF_FILEx_t id, uint32_t offset, void* data, uint32_t size, FRESULT* out_fresult);
    AGRBFileSystemStatusDef (*close)(IOIF_FILEx_t id, FRESULT* out_fresult);
    AGRBFileSystemStatusDef (*delete)(const char* fullpath, FRESULT* out_fresult);
    AGRBFileSystemStatusDef (*get_size)(IOIF_FILEx_t id, uint32_t* size);

} AGRBFileSystem_t;

extern AGRBFileSystem_t AGRBFileSystem;

//AGRBFileSystemStatusDef ioif_filesystem_initialize(IOIF_FileSystem_DeviceType_e device_type);
//void ioif_filesystem_deinitialize(void);
//
//IOIF_FileSystem_NamingRule_e ioif_filessystem_check_path_available(const char* path);
//
//AGRBFileSystemStatusDef ioif_filesystem_get_inspect(IOIF_FileSystem_Inspect_t* inspect);
//
//AGRBFileSystemStatusDef ioif_filesystem_get_file_count(const char* path, uint32_t* file_count);
//AGRBFileSystemStatusDef ioif_filesystem_get_file_list(const char* path, char filenames[][IOIF_FILESYSTEM_MAX_FILENAME_LENGTH], uint32_t* file_count, uint32_t max_files);
//
//AGRBFileSystemStatusDef ioif_filesystem_delete_path(const char* path);
//AGRBFileSystemStatusDef ioif_filesystem_delete_file(const char* path, const char* filename, const char* ext);
//
////Just create a new file with binary data. If the file exists, handle it according to 'mode'
////One-shot operation. No need to open/close the file separately.
////But it can be used to create a text file by passing text data.
//
//AGRBFileSystemStatusDef ioif_filesystem_create_binary_file(const char* path, const char* filename, const char* ext, void* data, size_t size, IOIF_FileSystem_CreateMode_e mode);
//AGRBFileSystemStatusDef ioif_filesystem_create_binary_file_ex(const char* fullpath, void* data, size_t size, IOIF_FileSystem_CreateMode_e mode);
//AGRBFileSystemStatusDef ioif_filesystem_create_text_file(const char* path, const char* filename, const char* ext, const char* text, IOIF_FileSystem_CreateMode_e mode);
//AGRBFileSystemStatusDef ioif_filesystem_create_text_file_ex(const char* fullpath, const char* text, IOIF_FileSystem_CreateMode_e mode);
//
////Open an existing file. If the file does not exist, create a new file.
////The file ID is returned in 'id' parameter. Use this ID for subsequent operations.
////Multiple files can be opened simultaneously, up to IOIF_SDCARD_MAX_OPENED_FILES.
////The mode parameter specifies how the file is opened.
////AGRBFileSystemStatusDef ioif_filesystem_open_file(IOIF_FILEx_t* id, const char* path, const char* filename, const char* ext, IOIF_FileSystem_AccessMode_e mode);
//AGRBFileSystemStatusDef ioif_filesystem_open_file_readonly(IOIF_FILEx_t* id, const char* path, const char* filename, const char* ext);
//AGRBFileSystemStatusDef ioif_filesystem_open_file_write(IOIF_FILEx_t* id, const char* path, const char* filename, const char* ext, IOIF_FileSystem_CreateMode_e mode);
//
//AGRBFileSystemStatusDef ioif_filesystem_get_file_size(IOIF_FILEx_t id, uint32_t* size);
//
//
//AGRBFileSystemStatusDef ioif_filesystem_append_data(IOIF_FILEx_t id, const void* data, uint32_t size);
//AGRBFileSystemStatusDef ioif_filesystem_seek(IOIF_FILEx_t id, uint32_t offset); //파일의 특정 위치로 이동
//AGRBFileSystemStatusDef ioif_filesystem_read(IOIF_FILEx_t id, void* data, uint32_t size, uint32_t* read_size); //현재 위치에서 size만큼 읽음. 위치는 자동으로 이동
//AGRBFileSystemStatusDef ioif_filesystem_close_file(IOIF_FILEx_t id);
//AGRBFileSystemStatusDef ioif_filesystem_get_file_size(IOIF_FILEx_t id, uint32_t* size);
//
 

/******************************* */
#endif /* _IOIF_AGRB_SDCARD_EX_H_ */


#endif // AGRB_IOIF_FILESYSTEM_DISABLE
