#include "ioif_agrb_fs.h"

#if defined(AGRB_IOIF_FILESYSTEM_ENABLE)

#include <string.h>
#include "ff.h"
#include "diskio.h"
#include "ioif_agrb_gpio.h" 


#if !defined(_FS_REENTRANT)
#error "_FS_REENTRANT must be defined to use ioif_agrb_filesystem module"
#elif (_FS_REENTRANT == 0)
#error "_FS_REENTRANT must be set to 1 to use ioif_agrb_filesystem module"
#endif

#define IOIF_FILESYSTEM_DEFAULT_TIMEOUT    (pdMS_TO_TICKS(1000U))

typedef struct
{
    bool is_initialized;
    FIL file;

    struct { 
        IOIF_FileSystem_CreateMode_e create_mode;
        
    } meta;

    IOIF_FileSystem_AccessMode_e mode;
    char filename[IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH];

} IOIF_FileSystem_FileInstance_t;

#if defined(USE_FREERTOS_DMA)
__attribute__((aligned(32)))
#endif
static IOIF_FileSystem_FileInstance_t _file_instances[IOIF_FILESYSTEM_MAX_OPENED_FILES] = {0, };

static uint8_t _path_bounce_buffer[IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH];//Scratch buffer for full path 
static const char* _backup_extension = IOIF_FILESYSTEM_BACKUPFILE_EXTENSION; //Backup file extension

#if defined(USE_FREERTOS_DMA)
static SemaphoreHandle_t _filesystem_semaphore = NULL; //To protect SD card access
static SemaphoreHandle_t _rw_semaphore = NULL; //To signal read/write completion

__attribute__((section(IOIF_DMA_SECTION), aligned(32)))
static uint8_t  _filesystem_dma_rw_buffer[IOIF_FILESYSTEM_READ_BOUNCE_BUFFER_SIZE]; //For read bounce buffer
static uint32_t _filesystem_sema_timeout_count = 0; //For debug
static uint32_t _filesystem_rw_timeout_count = 0; //For debug
#define IOIF_FILESYSTEM_ACQUIRE_SEMAPHORE()  do {                           \
    if (_filesystem_semaphore != NULL) {                                    \
        if ( xSemaphoreTake(_filesystem_semaphore,                          \
            IOIF_FILESYSTEM_DEFAULT_TIMEOUT) != pdTRUE ) {                  \
            _filesystem_sema_timeout_count++;                               \
            return IOIF_FileSystem_TIMEOUT;                                 \
        }                                                                   \
    } else {                                                                \
        return IOIF_FileSystem_SEMAPHORE_NOT_INITIALIZED;                   \
    }                                                                       \
} while(0)

#define IOIF_FILESYSTEM_RELEASE_SEMAPHORE()  do {                           \
    if (_filesystem_semaphore != NULL) {                                    \
        if ( xSemaphoreGive(_filesystem_semaphore) != pdTRUE ) {            \
            return IOIF_FileSystem_SEMAPHORE_ERROR;                          \
        }                                                               \
    } else {                                                            \
        return IOIF_FileSystem_SEMAPHORE_NOT_INITIALIZED;                              \
    }                                                                   \
} while(0)

#define IOIF_FILESYSTEM_RELEASE_SEMAPHORE_ISR()  do {                       \
    if (_filesystem_semaphore != NULL) {                                    \
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;                  \
        if ( xSemaphoreGiveFromISR(_filesystem_semaphore,                   \
            &xHigherPriorityTaskWoken) != pdTRUE ) {                    \
            return IOIF_FileSystem_SEMAPHORE_ERROR;                          \
        }                                                               \
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);                   \
    } else {                                                            \
        return IOIF_FileSystem_SEMAPHORE_NOT_INITIALIZED;                              \
    }                                                                   \
} while(0)


#define IOIF_FILESYSTEM_WAIT_RW_COMPLETE() do {                                 \
    if ( AGRBFileSystem.device_type == IOIF_FileSystem_DeviceType_SDCard ) {    \
        if (_rw_semaphore != NULL) {                                            \
            if ( xSemaphoreTake(_rw_semaphore,                                  \
                IOIF_FILESYSTEM_DEFAULT_TIMEOUT) != pdTRUE ) {                  \
                IOIF_FILESYSTEM_RELEASE_SEMAPHORE();                            \
                _filesystem_rw_timeout_count++;                                 \
                return IOIF_FileSystem_TIMEOUT;                                 \
            }                                                                   \
            IOIF_FILESYSTEM_RELEASE_SEMAPHORE();                                \
            return IOIF_FileSystem_SEMAPHORE_NOT_INITIALIZED;                   \
        }                                                                       \
    }                                                                           \
} while(0)

#define IOIF_FILESYSTEM_SIGNAL_RW_COMPLETE() do {                               \
    if ( AGRBFileSystem.device_type == IOIF_FileSystem_DeviceType_SDCard ) {    \
        if (_rw_semaphore != NULL) {                                            \
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;                      \
            if ( xSemaphoreGiveFromISR(_rw_semaphore,                           \
                &xHigherPriorityTaskWoken) != pdTRUE ) {                        \
                return IOIF_FileSystem_SEMAPHORE_ERROR;                         \
            }                                                                   \
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);                       \
        } else {                                                                \
            return IOIF_FileSystem_SEMAPHORE_NOT_INITIALIZED;                   \
        }                                                                       \
    }                                                                           \
} while(0) 


#else
#define IOIF_FILESYSTEM_ACQUIRE_SEMAPHORE()
#define IOIF_FILESYSTEM_RELEASE_SEMAPHORE()
#define IOIF_FILESYSTEM_RELEASE_SEMAPHORE_ISR()

#define IOIF_FILESYSTEM_WAIT_RW_COMPLETE()
#define IOIF_FILESYSTEM_SIGNAL_RW_COMPLETE()

#define IOIF_FILESYSTEM_ACQUIRE_FILE_SEMAPHORE(file_instance_ptr)
#define IOIF_FILESYSTEM_RELEASE_FILE_SEMAPHORE(file_instance_ptr)

#endif

#define __ASSERT_DRIVER_INITIALIZED()               \
    do {                                            \
        /* if (_filesystem == NULL) { */            \
        /*if ( AGRBFileSystem.driver == NULL ) {  */\
        if ( _driver == NULL ) {                    \
            return IOIF_FileSystem_DRIVER_ERROR;    \
        }                                           \
    } while(0)

#define BACKUP_FILE_NAME_RULE(backup_path, original_path)  do{                  \
    if (backup_path == NULL || original_path == NULL) while(1);                 \
    if (sizeof(backup_path) < IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH) while(1);    \
    memset(backup_path, 0, sizeof(backup_path));                                \
    snprintf((char*)backup_path, IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH, "%s%s",   \
        original_path, IOIF_FILESYSTEM_BACKUPFILE_EXTENSION_DOT);               \
} while(0)
    
/***************************************************/
static AGRBFileSystemStatusDef ioif_filesystem_initialize(IOIF_FileSystem_DeviceType_e device_type);
static AGRBFileSystemStatusDef ioif_filesystem_deinitialize(void);
/***************************************************/
static bool _ioif_filesystem_check_pathname_vaildation(const char* fullname);
static bool _ioif_filesystem_check_file_exist(const char* fullname);
static bool _ioif_filesystem_check_path_exist(const char* path);

static bool _ioif_filesystem_check_filecount(const char* path, size_t* out_file_count);

/* * */
static uint32_t _get_path_file_count(const char* fullpath, FRESULT* out_fresult);
static bool _file_open_write(FIL* file, const char* path, IOIF_FileSystem_CreateMode_e mode, FRESULT* out_fresult);
static bool _file_open_write_append(FIL* file, const char* path, FRESULT* out_fresult);
static bool _file_open_write_overwrite(FIL* file, const char* path, FRESULT* out_fresult);
static bool _file_open_write_exclusive(FIL* file, const char* path, FRESULT* out_fresult);
static bool _file_open_write_increment(FIL* file, const char* path, FRESULT* out_fresult);
/* * */

static bool _ioif_filesystem_parse_path(char* out_path, size_t out_path_size, const char* fullname);
static bool _ioif_filesystem_parse_filename(char* out_filename, size_t out_filename_size, const char* fullname);
static bool _ioif_filesystem_parse_extension(char* out_extension, size_t out_extension_size, const char* fullname);
static bool _ioif_filesystem_make_directories(const char* path);
static bool _ioif_filesystem_create_filename_increment(char* out_filename, size_t out_filename_size, const char* base_filename);

static bool _ioif_filesystem_create_path_recursively(const char* path);
static bool _ioif_filesystem_construct_fullname(char* out_fullname, size_t max_len, const char* path, const char* filename, const char* ext);
static Diskio_drvTypeDef* _ioif_filesystem_get_diskio_driver(IOIF_FileSystem_DeviceType_e device_type);
static AGRBFileSystemStatusDef _ioif_filesystem_file_deinitialize(IOIF_FILEx_t id);
static IOIF_FILEx_t _find_free_file_instance_slot(void);

/**************************************************/
static AGRBFileSystemStatusDef _ioif_filesystem_mkdirs(const char* fullpath, FRESULT* out_fresult);
//Simple하게 가자... 진짜 공개될 함수들
static AGRBFileSystemStatusDef ioif_filesystem_open_readonly_ex(IOIF_FILEx_t* id, const char* path, FRESULT* out_fresult);
static AGRBFileSystemStatusDef ioif_filesystem_open_write_ex(IOIF_FILEx_t* id, const char* path, IOIF_FileSystem_CreateMode_e mode, FRESULT* out_fresult);
static AGRBFileSystemStatusDef ioif_filesystem_write_ex(IOIF_FILEx_t id, const void* data, uint32_t size, FRESULT* out_fresult);
static AGRBFileSystemStatusDef ioif_filesystem_read_ex(IOIF_FILEx_t id, uint32_t offset, void* data, uint32_t size, FRESULT* out_fresult);
static AGRBFileSystemStatusDef ioif_filesystem_close_ex(IOIF_FILEx_t id, FRESULT* out_fresult);
static AGRBFileSystemStatusDef ioif_filesystem_delete_ex(const char* fullpath, FRESULT* out_fresult);
static AGRBFileSystemStatusDef ioif_filesystem_get_size_ex(IOIF_FILEx_t id, uint32_t* size);
/***************************************************/
//파일시스템 전역 인스턴스

static FATFS _fatfs; //FATFS 핸들러
static uint8_t _root_path[8];
static Diskio_drvTypeDef* _driver;

//align 32 for optimize
__attribute__((aligned(32)))
AGRBFileSystem_t AGRBFileSystem = {

    .init = ioif_filesystem_initialize,
    .deinit = ioif_filesystem_deinitialize,
    .open_readonly = ioif_filesystem_open_readonly_ex,
    .open_write = ioif_filesystem_open_write_ex,
    .write = ioif_filesystem_write_ex,
    .read = ioif_filesystem_read_ex,
    .close = ioif_filesystem_close_ex,
    .delete = ioif_filesystem_delete_ex,
    .get_size = ioif_filesystem_get_size_ex,
};

/**************************************************/


//해당 경로가 유효한지 검사. 존재하지 않아도 유효한 것으로 간주
static bool _ioif_filesystem_check_pathname_vaildation(const char* fullname)
{
    //This Work Must be called with semaphore acquired.    
    if (fullname == NULL) return false;    
    size_t path_len = strnlen(fullname, IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH);
    if (path_len == 0 || path_len >= IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH) return false;
    const char* invalid_chars = "\\:*?\"<>|"; //Windows invalid chars
    for (size_t i = 0; i < path_len; i++) {
        if (strchr(invalid_chars, fullname[i]) != NULL) return false;
    }    
    return true;
}

//해당 파일이 존재하는지 검사
static bool _ioif_filesystem_check_file_exist(const char* fullname)
{
    //This Work Must be called with semaphore acquired.
    //if (_filesystem == NULL) return false; //Not initialized
    ///if (AGRBFileSystem.driver == NULL) return false; //Not initialized
    if (_driver == NULL) return false; //Not initialized
    if (fullname == NULL) return false;

    FILINFO fno;
    FRESULT res;
    res = f_stat(fullname, &fno);
    if (res == FR_OK) {
        if (!(fno.fattrib & AM_DIR)) { 
            return true; //File exists and is not a directory
        }
    }
    return false;
}

//해당 경로가 존재하는지 검사. 없으면 생성 시도
static bool _ioif_filesystem_check_path_exist(const char* path)
{
    //This Work Must be called with semaphore acquired.
    //if (_filesystem == NULL) return false; //Not initialized
    //if (AGRBFileSystem.driver == NULL) return false; //Not initialized
    if (_driver == NULL) return false; //Not initialized

    if (path == NULL) return false;
    size_t path_len = strnlen(path, IOIF_FILESYSTEM_MAX_FULLPATH_LENGTH);
    if (path_len == 0 || path_len >= IOIF_FILESYSTEM_MAX_FULLPATH_LENGTH) return false;

    switch( f_mkdir(path) ) {
        case FR_OK: //Directory created successfully, so it did not exist before but...exist now, so what?
        case FR_EXIST: //Directory already exists
            return true; 
        default:
            return false; //Other error
    }
}

//현재 파일 카운트가 정해진 한도를 초과하는지 검사
static bool _ioif_filesystem_check_filecount(const char* path, size_t* out_file_count)
{
    //This Work Must be called with semaphore acquired.
    //if (_filesystem == NULL) return false;
    //if (AGRBFileSystem.driver == NULL) return false; //Not initialized
    if (_driver == NULL) return false; //Not initialized
    if (path == NULL) return false;

    size_t path_len = strnlen(path, IOIF_FILESYSTEM_MAX_FULLPATH_LENGTH);
    if (path_len == 0 || path_len >= IOIF_FILESYSTEM_MAX_FULLPATH_LENGTH) return false;
    DIR dir;
    FILINFO fno;
    FRESULT res;
    size_t file_count = 0;

    res = f_opendir(&dir, path);
    if (res != FR_OK) return false; //Failed to open directory

    for (;;) {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == 0) break; //Error or end of dir
        if (!(fno.fattrib & AM_DIR)) {
            file_count++;
        }
    }

    f_closedir(&dir);

    if ( out_file_count != NULL ) *out_file_count = file_count;
    
    return (IOIF_FILESYSTEM_MAX_FILECOUNT_PER_PATH > file_count);
}

static bool _ioif_filesystem_create_path_recursively(const char* path)
{
    //This Work Must be called with semaphore acquired.
    //This Work May modify _path_bounce_buffer and overhead
    if (path == NULL) return false;

    if ( !_ioif_filesystem_check_pathname_vaildation(path) ) return false; //Invalid path name
    if ( _ioif_filesystem_check_path_exist(path) ) return true; //Path already exists
    
    memset(_path_bounce_buffer, 0, sizeof(_path_bounce_buffer));
    
    size_t path_len = strnlen(path, IOIF_FILESYSTEM_MAX_FULLPATH_LENGTH);
    for( size_t i = 0; i < path_len-1; i++ ) {        
        if ( path[i] == '/' ) {
            _path_bounce_buffer[i] = '\0'; //Temporarily terminate string here
            //Create directory up to this point
            if ( !_ioif_filesystem_check_path_exist(_path_bounce_buffer) ) {
                //실패했으면 어우 좀 문제인데?
                return false;
            }
        }
        _path_bounce_buffer[i] = path[i];
    }

    return true;
}
 
static bool _ioif_filesystem_construct_fullname(char* out_fullname, size_t max_len, const char* path, const char* filename, const char* ext)
{
    //This Work Must be called with semaphore acquired.
    if (out_fullname == NULL || path == NULL || filename == NULL || ext == NULL) return false;
    size_t path_len = strnlen(path, IOIF_FILESYSTEM_MAX_FULLPATH_LENGTH);
    size_t filename_len = strnlen(filename, IOIF_FILESYSTEM_MAX_FILENAME_LENGTH);
    size_t ext_len = strnlen(ext, IOIF_FILESYSTEM_MAX_EXTENSION_LENGTH);
    if (path_len >= IOIF_FILESYSTEM_MAX_FULLPATH_LENGTH) return false;
    if (filename_len == 0 || filename_len >= IOIF_FILESYSTEM_MAX_FILENAME_LENGTH) return false;
    if (ext_len >= IOIF_FILESYSTEM_MAX_EXTENSION_LENGTH) return false; //확장자는 없어도 됨
    if( path[0] != '/' ) path_len++; //Leading '/' is required, so add it if not present

    if (path_len + 1 + filename_len + (ext_len > 0 ? (1 + ext_len) : 0) + 1 > max_len) return false; //Fullname too long   
 
    //Construct full path
    memset(out_fullname, '\0', max_len); 

    if (path[0] != '/') {
        out_fullname[0] = '/';
        strncpy(&out_fullname[1], path, path_len-1);
    } else {
        strncpy(out_fullname, path, path_len);
    }

    if (out_fullname[path_len-1] != '/') {
        out_fullname[path_len] = '/';
        path_len++;
    } 
    
    strncpy(&out_fullname[path_len], filename, filename_len);
    if (ext_len > 0) {
        out_fullname[path_len + filename_len] = '.';
        strncpy(&out_fullname[path_len + filename_len + 1], ext, ext_len);
    } 
    out_fullname[path_len + filename_len + (ext_len > 0 ? (1 + ext_len) : 0)] = '\0';

    if ( !_ioif_filesystem_check_pathname_vaildation(out_fullname) ) return false; //Invalid path name

    return true;
}


/** @defgroup SDCard 
  * @brief SD Card HAL I/O module driver
  * @{
  */

FRESULT _debug_ioif_fs_mount;
uint8_t _debug_ioif_fs_link_drv;
FIL _debug_fs_test_file;

AGRBFileSystemStatusDef ioif_filesystem_initialize(IOIF_FileSystem_DeviceType_e device_type)
{
    //if ( AGRBFileSystem.driver != NULL ) AGRBStatus_BUSY;
    //if (_filesystem != NULL) return AGRBStatus_BUSY; //Already assigned
    if ( _driver != NULL ) return AGRBStatus_BUSY;

    uint8_t current_linked = FATFS_GetAttachedDriversNbr();
    while( current_linked-- ) FATFS_UnLinkDriver('0');

#if defined(USE_FREERTOS_DMA)
    if (_filesystem_semaphore == NULL) {
        _filesystem_semaphore = xSemaphoreCreateBinary();
        if (_filesystem_semaphore == NULL) {
            //_filesystem = NULL;
            //AGRBFileSystem.driver = NULL;
            _driver = NULL;
            return IOIF_FileSystem_SEMAPHORE_ERROR; //Failed to create semaphore
        }
        xSemaphoreGive(_filesystem_semaphore); //Initial state
    }
    if (_rw_semaphore == NULL) {
        _rw_semaphore = xSemaphoreCreateBinary();
        if (_rw_semaphore == NULL) {
            if (_filesystem_semaphore != NULL) {
                vSemaphoreDelete(_filesystem_semaphore);
                _filesystem_semaphore = NULL;
            }
            //_filesystem = NULL;
            //AGRBFileSystem.driver = NULL;
            _driver = NULL;
            return IOIF_FileSystem_SEMAPHORE_ERROR; //Failed to create semaphore
        }
    }
#endif

    //AGRBFileSystem.driver = _ioif_filesystem_get_diskio_driver(device_type);
    _driver = _ioif_filesystem_get_diskio_driver(device_type);

    //if (AGRBFileSystem.driver == NULL) return IOIF_FileSystem_DRIVER_ERROR; //Invalid parameter
    if (_driver == NULL) return IOIF_FileSystem_DRIVER_ERROR; //Invalid parameter
    
    //_debug_ioif_fs_link_drv = FATFS_LinkDriver(AGRBFileSystem.driver, AGRBFileSystem.root_path);
    _debug_ioif_fs_link_drv = FATFS_LinkDriver(_driver, _root_path);

    //_debug_ioif_fs_mount = f_mount(&(AGRBFileSystem.fatfs), (TCHAR const*)AGRBFileSystem.root_path, 0);
    _debug_ioif_fs_mount = f_mount(&_fatfs, (TCHAR const*)_root_path, 0);

    AGRBFileSystem.device_type = device_type;
    //_ioif_fatfs_device_type = device_type; 

    return IOIF_FileSystem_OK;
}

AGRBFileSystemStatusDef ioif_filesystem_deinitialize(void)
{
    __ASSERT_DRIVER_INITIALIZED();
    
    IOIF_FILESYSTEM_ACQUIRE_SEMAPHORE();
    {
        //_filesystem = NULL;
        //AGRBFileSystem.driver = NULL;
        _driver = NULL;
        //Clear all file instances
        memset(_file_instances, 0, sizeof(_file_instances));
    }
    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();
    
#if defined(USE_FREERTOS_DMA)   
    if (_filesystem_semaphore != NULL) {
        vSemaphoreDelete(_filesystem_semaphore);
        _filesystem_semaphore = NULL;
    }
    if (_rw_semaphore != NULL) {
        vSemaphoreDelete(_rw_semaphore);
        _rw_semaphore = NULL;
    }
#endif


    return;
}


AGRBFileSystemStatusDef ioif_filesystem_get_inspect(IOIF_FileSystem_Inspect_t* inspect) //현재 파일 시스템 상태 검사
{
    __ASSERT_DRIVER_INITIALIZED();

    if (inspect == NULL) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter

    AGRBFileSystemStatusDef result = IOIF_FileSystem_OK;
    memset(inspect, 0, sizeof(IOIF_FileSystem_Inspect_t));    

    IOIF_FILESYSTEM_ACQUIRE_SEMAPHORE();
    do {    
//        FATFS* fs = &(AGRBFileSystem.fatfs);
        FATFS* fs = &_fatfs;
        DWORD fre_clust;
        FRESULT res;

        //Get volume information and free clusters
        res = f_getfree("", &fre_clust, &fs);
        if (res != FR_OK) {
            result = IOIF_FileSystem_INTERNAL; //Failed to get filesystem info
            break;
        }

        //Calculate total and free blocks
        inspect->block_size = fs->csize * 512; //Cluster size in bytes
        inspect->total_blocks = (fs->n_fatent - 2); //Total clusters
        inspect->free_blocks = fre_clust; //Free clusters

        //Calculate total and free size
        inspect->total_size = inspect->total_blocks * inspect->block_size;
        inspect->free_size = inspect->free_blocks * inspect->block_size;

        inspect->is_initialized = true;
    } while(0);  
    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();

    return result;
}

//AGRBFileSystemStatusDef ioif_filesystem_get_file_count(const char* path, uint32_t* file_count)
//{
//    __ASSERT_DRIVER_INITIALIZED();
//
//    if (path == NULL || file_count == NULL) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter
//    AGRBFileSystemStatusDef result = IOIF_FileSystem_OK;
//
//    IOIF_FILESYSTEM_ACQUIRE_SEMAPHORE();
//    do {
//        size_t found_file_count = 0;
//        if ( !_ioif_filesystem_check_filecount(path, &found_file_count) ) {
//            result = IOIF_FileSystem_INTERNAL; //Failed to get file count
//            break;
//        }
//        *file_count = (uint32_t)found_file_count;        
//    } while(0);
//    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();
//    return result;
//}

//AGRBFileSystemStatusDef ioif_filesystem_get_file_list(const char* path, char filenames[][IOIF_FILESYSTEM_MAX_FILENAME_LENGTH], uint32_t* file_count, uint32_t max_files)
//{
//    __ASSERT_DRIVER_INITIALIZED();
//
//    if (path == NULL || filenames == NULL || file_count == NULL) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter
//    if (max_files == 0) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter
//
//    //현재 경로에 파일의 리스트를 얻어옴
//    //폴더는 무시함
//
//    AGRBFileSystemStatusDef result = IOIF_FileSystem_OK;
//    uint32_t found_files = 0;    
//
//    IOIF_FILESYSTEM_ACQUIRE_SEMAPHORE();
//    do {    
//        DIR dir;
//        FILINFO fno;
//        FRESULT res;
//
//        size_t path_len = strnlen(path, IOIF_FILESYSTEM_MAX_FULLPATH_LENGTH);
//        if (path_len == 0 || path_len >= IOIF_FILESYSTEM_MAX_FULLPATH_LENGTH) {
//            result = IOIF_FileSystem_PATH_INVALID; //Invalid path
//            break;
//        }
//
//        res = f_opendir(&dir, path);
//        if (res != FR_OK) {
//            result = IOIF_FileSystem_PATH_INVALID; //Invalid path
//            break;
//        }
//
//        for (;;) {
//            res = f_readdir(&dir, &fno);
//            if (res != FR_OK || fno.fname[0] == 0) break; //Error or end of dir
//            if (!(fno.fattrib & AM_DIR)) {
//                //파일인 경우에만 추가
//                strncpy(filenames[found_files], fno.fname, IOIF_FILESYSTEM_MAX_FILENAME_LENGTH);
//                if (filenames[found_files][IOIF_FILESYSTEM_MAX_FILENAME_LENGTH-1] != '\0') {
//                    filenames[found_files][IOIF_FILESYSTEM_MAX_FILENAME_LENGTH-1] = '\0'; //Ensure null termination
//                }
//                found_files++;
//                if (found_files >= max_files) break; //Reached max files
//            }
//        }
//
//        f_closedir(&dir);
//
//        *file_count = found_files;        
//        
//    } while(0);
//    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();
//    
//    return result;
//}
//
//AGRBFileSystemStatusDef ioif_filesystem_delete_file(const char* path, const char* filename, const char* ext)
//{
//    __ASSERT_DRIVER_INITIALIZED();
//    
//    IOIF_FILESYSTEM_ACQUIRE_SEMAPHORE();
//    {    
//        FILINFO fno;
//        FRESULT res;
//
//        if ( !_ioif_filesystem_construct_fullname((char*)_path_bounce_buffer, sizeof(_path_bounce_buffer), path, filename, ext) ) {
//            IOIF_FILESYSTEM_RELEASE_SEMAPHORE();
//            return IOIF_FileSystem_PATH_INVALID; //Invalid path/filename/ext
//        }
//
//        res = f_stat((const char*)_path_bounce_buffer, &fno);
//        if (res != FR_OK) {
//            IOIF_FILESYSTEM_RELEASE_SEMAPHORE();
//            return IOIF_FileSystem_NOT_EXIST; //File does not exist
//        }
//
//        res = f_unlink((const char*)_path_bounce_buffer);
//        if (res != FR_OK) {
//            IOIF_FILESYSTEM_RELEASE_SEMAPHORE();
//            return IOIF_FileSystem_INTERNAL; //Failed to delete file
//        }
//    }
//    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();
//
//    return IOIF_FileSystem_OK;
//}
//
//static FRESULT _debug_ioif_fs_create_file_error_stage;
//AGRBFileSystemStatusDef ioif_filesystem_create_binary_file(const char* path, const char* filename, const char* ext, void* data, size_t size, IOIF_FileSystem_CreateMode_e mode)
//{
//    __ASSERT_DRIVER_INITIALIZED();
//
//    uint8_t path_construct_buffer[IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH];
//    uint8_t path_construct_buffer_tmp[IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH];
// 
//    if (path == NULL || filename == NULL || ext == NULL) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter
//    if (data == NULL || size == 0) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter
//    if (mode >= IOIF_FileSystem_CreateMode_UNKNOWN) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter
//    //if path str len is '0', it's considered as root path, so it's INVALID Position
//    if ( strnlen(path, IOIF_FILESYSTEM_MAX_FULLPATH_LENGTH) >= IOIF_FILESYSTEM_MAX_FULLPATH_LENGTH ) {
//        return IOIF_FileSystem_PATH_INVALID; //Invalid path
//    }
//    if ( strnlen(filename, IOIF_FILESYSTEM_MAX_FILENAME_LENGTH) == 0 ||
//         strnlen(filename, IOIF_FILESYSTEM_MAX_FILENAME_LENGTH) >= IOIF_FILESYSTEM_MAX_FILENAME_LENGTH ) {
//        return IOIF_FileSystem_PATH_INVALID; //Invalid filename
//    }
//
//    #if defined(USE_FREERTOS_DMA)
//    if ( size > IOIF_FILESYSTEM_WRITE_BOUNCE_BUFFER_SIZE ) {
//        return IOIF_FileSystem_SIZE_EXCEED; //Size exceeds bounce buffer
//    }
//    #endif
//
//    uint8_t* send_data = (uint8_t*)data;
//    uint8_t _filename[IOIF_FILESYSTEM_MAX_FILENAME_LENGTH];
//    strncpy((char*)_filename, filename, IOIF_FILESYSTEM_MAX_FILENAME_LENGTH);
//
//    IOIF_FILESYSTEM_ACQUIRE_SEMAPHORE();
//    {   
//        //이름 구성
//        if ( !_ioif_filesystem_construct_fullname((char*)path_construct_buffer, IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH, path, _filename, ext) ) {
//            IOIF_FILESYSTEM_RELEASE_SEMAPHORE();
//            return IOIF_FileSystem_PATH_INVALID; //Invalid path/filename/ext
//        } 
//
//        //경로 생성
//        if ( !_ioif_filesystem_create_path_recursively(path) ) {
//            IOIF_FILESYSTEM_RELEASE_SEMAPHORE();
//            return IOIF_FileSystem_INTERNAL; //Failed to create path
//        }
//
//        //현재 경로에 파일이 일정 범위를 벗어났는지 검사        
//        if ( _ioif_filesystem_check_filecount(path, NULL) == false ) {
//            IOIF_FILESYSTEM_RELEASE_SEMAPHORE();
//            return IOIF_FileSystem_FILECOUNT_EXCEED; //Too many files in path
//        }
//        
//        //파일 존재 검사
//        bool is_backup_file = false;
//        uint32_t postfix_num = 1;
//        while ( _ioif_filesystem_check_file_exist((const char*)path_construct_buffer) && !is_backup_file )
//        {
//            switch (mode)
//            {
//            case IOIF_FileSystem_CreateMode_EXCLUSIVE:
//                IOIF_FILESYSTEM_RELEASE_SEMAPHORE();
//                return IOIF_FileSystem_OVERWRITE_PROTECTED; //File already exists
//                break;
//            case IOIF_FileSystem_CreateMode_OVERWRITE:
//                //덮어쓰기 모드이므로 그냥 진행. 다만 쓰기 과정에서의 오류를 방지하기 위해 우선 백업 파일에 작성함
//                if ( !_ioif_filesystem_construct_fullname((char*)path_construct_buffer, IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH, path, _filename, _backup_extension) ) {
//                    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();
//                    return IOIF_FileSystem_PATH_INVALID; //Invalid path/filename/ext
//                }
//                //만약 지금 만든 이름의 백업 파일도 존재한다면, 그걸 지우고 새로 만듦
//                if ( _ioif_filesystem_check_file_exist((const char*)path_construct_buffer) ) {
//                    f_unlink((const char*)path_construct_buffer); //실패해도 무시
//                }
//                is_backup_file = true;                
//                break;
//            case IOIF_FileSystem_CreateMode_INCREMENT:
//                //파일명 뒤에 postfix 붙이기
//                if (postfix_num > 100) {
//                    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();
//                    return IOIF_FileSystem_FILECOUNT_EXCEED; //Too many files with same name
//                }
//                memset(_filename, 0, sizeof(_filename));
//                snprintf((char*)_filename, sizeof(_filename), "%s(%lu)", filename, (unsigned long)postfix_num++);
//
//                if ( strnlen((char*)_filename, IOIF_FILESYSTEM_MAX_FILENAME_LENGTH) >= IOIF_FILESYSTEM_MAX_FILENAME_LENGTH ) {
//                    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();
//                    return IOIF_FileSystem_PATH_INVALID; //Filename too long
//                }
//
//                if ( !_ioif_filesystem_construct_fullname((char*)path_construct_buffer, IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH, path, _filename, ext) ) {
//                    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();
//                    return IOIF_FileSystem_PATH_INVALID; //Invalid path/filename/ext
//                }
//            default:
//                break;
//            }
//        }
//
//        //파일 생성
//        FIL file;
//        FRESULT res;
//        memcpy(_path_bounce_buffer, path_construct_buffer, IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH);
//        res = f_open(&file, (const char*)path_construct_buffer, FA_WRITE | FA_CREATE_ALWAYS);
//        if (res != FR_OK) {
//            IOIF_FILESYSTEM_RELEASE_SEMAPHORE();
//            _debug_ioif_fs_create_file_error_stage = res;
//            return IOIF_FileSystem_INTERNAL; //Failed to open/create file
//        }
//        
//        uint32_t bytes_written = 0; 
//
//        //데이터 캐시 문제 방지를 위해 DMA 전송 후 완료 신호를 받을 때까지 대기
//        
//        if ( send_data != NULL && size > 0 ) {            
//            #if defined(USE_FREERTOS_DMA)
//            memset(_filesystem_dma_rw_buffer, 0, IOIF_FILESYSTEM_READ_BOUNCE_BUFFER_SIZE);
//            memcpy(_filesystem_dma_rw_buffer, data, (size > IOIF_FILESYSTEM_WRITE_BOUNCE_BUFFER_SIZE ? IOIF_FILESYSTEM_WRITE_BOUNCE_BUFFER_SIZE : size));
//            SCB_CleanDCache_by_Addr((uint32_t*)_filesystem_dma_rw_buffer, (size > IOIF_FILESYSTEM_WRITE_BOUNCE_BUFFER_SIZE ? IOIF_FILESYSTEM_WRITE_BOUNCE_BUFFER_SIZE : size));
//            send_data = _filesystem_dma_rw_buffer; 
//            #endif
//
//            res = f_write(&file, send_data, size, &bytes_written);
//            IOIF_FILESYSTEM_WAIT_RW_COMPLETE();
//        }
//
//        if (res != FR_OK) {//|| bytes_written != size) {
//            f_close(&file);
//            IOIF_FILESYSTEM_RELEASE_SEMAPHORE(); 
//            return IOIF_FileSystem_INTERNAL; //Failed to write data
//        }
//
//        res = f_close(&file);
//        if (res != FR_OK) {
//            IOIF_FILESYSTEM_RELEASE_SEMAPHORE(); 
//            return IOIF_FileSystem_INTERNAL; //Failed to close file
//        }
//
//        //덮어쓰기 모드였으면 백업 파일을 진짜 파일로 교체
//        if (is_backup_file) {
//            //기존 파일 삭제
//            if ( !_ioif_filesystem_construct_fullname((char*)path_construct_buffer, IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH, path, filename, ext) ) {
//                IOIF_FILESYSTEM_RELEASE_SEMAPHORE();
//                return IOIF_FileSystem_PATH_INVALID; //Invalid path/filename/ext
//            }
//            f_unlink((const char*)path_construct_buffer); //실패해도 무시
//
//            //백업 파일을 진짜 파일로 이름 변경
//            if ( !_ioif_filesystem_construct_fullname((char*)path_construct_buffer, IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH, path, _filename, _backup_extension) ) {
//                IOIF_FILESYSTEM_RELEASE_SEMAPHORE();
//                return IOIF_FileSystem_PATH_INVALID; //Invalid path/filename/ext
//            }
//            if ( !_ioif_filesystem_construct_fullname((char*)path_construct_buffer_tmp, IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH, path, filename, ext) ) {
//                IOIF_FILESYSTEM_RELEASE_SEMAPHORE();
//                return IOIF_FileSystem_PATH_INVALID; //Invalid path/filename/ext
//            }
//            res = f_rename((const char*)path_construct_buffer, (const char*)path_construct_buffer_tmp);
//            if (res != FR_OK) {
//                IOIF_FILESYSTEM_RELEASE_SEMAPHORE();
//                return IOIF_FileSystem_INTERNAL; //Failed to rename file
//            }
//        }
//    }
//    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();
//
//    return AGRBStatus_OK;
//}
//
//AGRBFileSystemStatusDef ioif_filesystem_create_text_file(const char* path, const char* filename, const char* ext, const char* text, IOIF_FileSystem_CreateMode_e mode)
//{
//    __ASSERT_DRIVER_INITIALIZED();
//
//    if (text == NULL) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter
//    
//    size_t text_len = strnlen(text, IOIF_FILESYSTEM_WRITE_BOUNCE_BUFFER_SIZE);
//    //if there is no-ascii character, do not allow
//    for ( int i = 0; i < text_len; i++ ) {
//        if ( text[i] < 0x20 && text[i] != '\r' && text[i] != '\n' && text[i] != '\t' ) {
//            return IOIF_FileSystem_PATH_INVALID; //Non-ascii character found
//        }
//        if ( text[i] > 0x7E ) {
//            return IOIF_FileSystem_PATH_INVALID; //Non-ascii character found
//        }
//    }
//
//    if (text_len == 0 || text_len >= IOIF_FILESYSTEM_WRITE_BOUNCE_BUFFER_SIZE) return IOIF_FileSystem_PATH_INVALID; //Invalid parameter
//
//    return ioif_filesystem_create_binary_file(path, filename, ext, (void*)text, text_len, mode);
//}
//static uint32_t _ioif_filesystem_open_file_error_stage;
//static bool _break_point = false;
//
//FRESULT _debug_ioif_fs_open_file_error;
//AGRBFileSystemStatusDef ioif_filesystem_open_file_readonly(IOIF_FILEx_t* id, const char* path, const char* filename, const char* ext)
//{
//    __ASSERT_DRIVER_INITIALIZED();
//
//    if (id == NULL) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter
//
//    AGRBFileSystemStatusDef result = IOIF_FileSystem_FILEINSTANCE_FULL;
//    IOIF_FILEx_t new_id = IOIF_FILESYSTEM_INVALID_FILE_ID;
//    IOIF_FileSystem_FileInstance_t *file_instance = NULL;
//
//    IOIF_FILESYSTEM_ACQUIRE_SEMAPHORE();
//    do {
//        //Find a free file instance slot
//        new_id = _find_free_file_instance_slot();
//        if ( new_id == IOIF_FILESYSTEM_INVALID_FILE_ID ) break; //No free slot
//
//        file_instance = &_file_instances[new_id];
//        memset(file_instance, 0, sizeof(IOIF_FileSystem_FileInstance_t));
//
//        //파일 이름 구성
//        if ( !_ioif_filesystem_construct_fullname((char*)file_instance->filename, IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH, path, filename, ext) ) {
//            result = IOIF_FileSystem_PATH_INVALID; //Invalid path/filename/ext
//            break;
//        }
//
//        //파일 열기
//        FRESULT res;
//        res = f_open(&(file_instance->file), (const char*)(file_instance->filename), (FA_READ));
//        if (res != FR_OK) {
//            _debug_ioif_fs_open_file_error = res;
//            result = IOIF_FileSystem_INTERNAL; //Failed to open file
//            break;
//        }
// 
//        file_instance->is_initialized = true;
//        file_instance->mode = IOIF_FileSystem_Access_READONLY;
//
//        result = IOIF_FileSystem_OK;
//        id[0] = new_id;
//    } while(0);
//    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();
//
//    return result;
//}
//

IOIF_FileSystem_NamingRule_e ioif_filessystem_check_path_available(const char* path)
{
    //문자열이 유효한지 검사    
    if (path == NULL) return IOIF_FileSystem_NamingRule_PARAM_ERROR;
    //문자열 길이가 255자를 초과하면 안됨    
    size_t path_len = strnlen(path, IOIF_FILESYSTEM_MAX_FULLPATH_LENGTH);
    if (path_len == 0 || path_len >= IOIF_FILESYSTEM_MAX_FULLPATH_LENGTH) return IOIF_FileSystem_NamingRule_INVALID_LENGTH;
    //문자열에 금지된 문자가 포함되어 있는지 검사
    const char* invalid_chars = " \\:*?\"<>|"; //Windows invalid chars
    for (size_t i = 0; i < path_len; i++) {
        if (strchr(invalid_chars, path[i]) != NULL) return IOIF_FileSystem_NamingRule_INVALID_CHARACTER;
    }
    //연속된 슬래시 검사
    if ( strstr(path, "//") != NULL ) return IOIF_FileSystem_NamingRule_CONSECUTIVE_SLASHES;

    //시작/끝 공백 및 슬래시, . 검사
    if ( path[0] != '/' ) return IOIF_FileSystem_NamingRule_NOT_START_WITH_SLASH;
    if ( path[0] == '.' ) return IOIF_FileSystem_NamingRule_START_WITH_DOT;
    if ( path[0] == ' ' ) return IOIF_FileSystem_NamingRule_START_WITH_SPACE;
    if ( path[path_len - 1] == '/' ) return IOIF_FileSystem_NamingRule_END_WITH_SLASH;
    if ( path[path_len - 1] == '.' ) return IOIF_FileSystem_NamingRule_END_WITH_DOT;    
    if ( path[path_len - 1] == ' ' ) return IOIF_FileSystem_NamingRule_END_WITH_SPACE;
    
    return IOIF_FileSystem_NamingRule_OK;
}

AGRBFileSystemStatusDef ioif_filesystem_open_readonly_ex(IOIF_FILEx_t* id, const char* path, FRESULT* out_fresult)
{
    __ASSERT_DRIVER_INITIALIZED();

    if (id == NULL) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter
    if ( ioif_filessystem_check_path_available(path) != IOIF_FileSystem_NamingRule_OK ) return IOIF_FileSystem_PATH_INVALID; //Invalid path

    AGRBFileSystemStatusDef result = IOIF_FileSystem_INTERNAL;
    FRESULT fres;

    IOIF_FILESYSTEM_ACQUIRE_SEMAPHORE();
    do {
        IOIF_FILEx_t new_id = _find_free_file_instance_slot();
        if ( new_id == IOIF_FILESYSTEM_INVALID_FILE_ID ) {
            result = IOIF_FileSystem_FILEINSTANCE_FULL; //No free slot
            break;
        }

        IOIF_FileSystem_FileInstance_t *file_instance = &_file_instances[new_id];
        memset(file_instance, 0, sizeof(IOIF_FileSystem_FileInstance_t)); 
        
        strncpy((char*)file_instance->filename, path, IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH);
        file_instance->mode = IOIF_FileSystem_Access_READONLY;
        file_instance->is_initialized = true;

        fres = f_open(&(file_instance->file), path, (FA_READ));
        if (fres != FR_OK) break;
 
        id[0] = new_id;

        result = IOIF_FileSystem_OK;

    } while(0);
    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();

    if ( out_fresult != NULL ) out_fresult[0] = fres;

    return result;
}

static AGRBFileSystemStatusDef _ioif_filesystem_mkdirs(const char* fullpath, FRESULT* out_fresult)
{
    uint32_t len = strnlen(fullpath, IOIF_FILESYSTEM_MAX_FULLPATH_LENGTH);

    if ( len == 0 ) return true;//No path to create
    if ( len >= IOIF_FILESYSTEM_MAX_FULLPATH_LENGTH ) return false; //Invalid path

    FRESULT res;
    char temp_path[IOIF_FILESYSTEM_MAX_FULLPATH_LENGTH+1];
    uint8_t use_offset = ( fullpath[0] == '/' ) ? 1 : 0; //Leading '/' is present
    
    memset(temp_path, 0, sizeof(temp_path));

    for ( uint32_t i = 0; i < len; i++ ) {
        temp_path[i] = fullpath[i];
        if ( temp_path[i] == '/' ) {
            //Create directory up to this point
            temp_path[i] = '\0'; //Temporarily terminate string here
            res = f_mkdir((const char*)(temp_path+use_offset));
        }
    }

    ////Create final directory
    //FRESULT res = f_mkdir((const char*)temp_path);
    //if ( res != FR_OK && res != FR_EXIST ) return false; //Failed to create directory.. 이건 심각한 오류

    return true;


}

AGRBFileSystemStatusDef ioif_filesystem_open_write_ex(IOIF_FILEx_t* id, const char* path, IOIF_FileSystem_CreateMode_e mode, FRESULT* out_fresult)
{
    __ASSERT_DRIVER_INITIALIZED();
    
    if (id == NULL) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter
    if ( ioif_filessystem_check_path_available(path) != IOIF_FileSystem_NamingRule_OK ) return IOIF_FileSystem_PATH_INVALID; //Invalid path

    FRESULT fres;

    if ( !_ioif_filesystem_mkdirs(path, &fres)) {
        if ( out_fresult != NULL ) out_fresult[0] = fres;
        return IOIF_FileSystem_PATH_INVALID; //Failed to create path
    }

    // IOIF_FileSystem_FILECOUNT_EXCEED CHECK
    uint32_t file_count = _get_path_file_count(path, &fres);
    if (fres != FR_OK) {
        if ( out_fresult != NULL ) out_fresult[0] = fres;
        return IOIF_FileSystem_INTERNAL; //Failed to get file count
    }

    if ( file_count >= IOIF_FILESYSTEM_MAX_FILECOUNT_PER_PATH ) {
        if ( out_fresult != NULL ) out_fresult[0] = FR_OK;
        return IOIF_FileSystem_FILECOUNT_EXCEED; //Too many files in path
    }

    AGRBFileSystemStatusDef result = IOIF_FileSystem_OK;

    IOIF_FILESYSTEM_ACQUIRE_SEMAPHORE();
    do {
        IOIF_FILEx_t new_id = _find_free_file_instance_slot();
        if ( new_id == IOIF_FILESYSTEM_INVALID_FILE_ID ) {
            result = IOIF_FileSystem_FILEINSTANCE_FULL; //No free slot
            break;
        }

        IOIF_FileSystem_FileInstance_t *file_instance = &_file_instances[new_id];
        memset(file_instance, 0, sizeof(IOIF_FileSystem_FileInstance_t));

        strncpy((char*)file_instance->filename, path, IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH);

        if ( _file_open_write(&(file_instance->file),   \
            (const char*)file_instance->filename,       \
            mode, &fres) != true ) {

            result = IOIF_FileSystem_INTERNAL; //Failed to open file
            break;
        }

        file_instance->mode = IOIF_FileSystem_Access_APPEND;
        file_instance->meta.create_mode = mode;        

        file_instance->is_initialized = true;

        id[0] = new_id;
    } while(0);
    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();

    if ( out_fresult != NULL ) out_fresult[0] = fres;
    
    return result;
}

AGRBFileSystemStatusDef ioif_filesystem_write_ex(IOIF_FILEx_t id, const void* data, uint32_t size, FRESULT* out_fresult)
{
    __ASSERT_DRIVER_INITIALIZED();

    if (data == NULL || size == 0) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter
    if (id >= IOIF_FILESYSTEM_MAX_OPENED_FILES) return IOIF_FileSystem_PARAM_ERROR; //Invalid file ID
    
    AGRBFileSystemStatusDef result = IOIF_FileSystem_OK;
    IOIF_FileSystem_FileInstance_t* file_instance = &_file_instances[id];
    if ( file_instance->is_initialized == false ) return IOIF_FileSystem_FILEINSTANCE_INVALID; //Invalid file ID

    #if defined(USE_FREERTOS_DMA)
    if ( size > IOIF_FILESYSTEM_WRITE_BOUNCE_BUFFER_SIZE ) return IOIF_FileSystem_SIZE_EXCEED; //Size exceeds bounce buffer    
    #endif

    if ( file_instance->mode != IOIF_FileSystem_Access_APPEND ) {
        return IOIF_FileSystem_ACCESS_MODE_INVALID; //File not opened in write/append mode
    }

    FRESULT res;
    uint32_t bytes_written = 0;
    uint8_t* write_data = (uint8_t*)data;

    IOIF_FILESYSTEM_ACQUIRE_SEMAPHORE();
    do {
        #if defined(USE_FREERTOS_DMA)
        switch( AGRBFileSystem.device_type )
        {
            case IOIF_FileSystem_DeviceType_USBH_MSC:
            break;
            default:
            {
                memset(_filesystem_dma_rw_buffer, 0, IOIF_FILESYSTEM_WRITE_BOUNCE_BUFFER_SIZE);
                memcpy(_filesystem_dma_rw_buffer, data, size);
                SCB_CleanDCache_by_Addr((uint32_t*)_filesystem_dma_rw_buffer, size);
                write_data = _filesystem_dma_rw_buffer;
            }
            break;
        }        
        #endif
        
        res = f_write( &(file_instance->file), (const void*)write_data, size, &bytes_written );
        IOIF_FILESYSTEM_WAIT_RW_COMPLETE();
    } while(0);
    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();

    if ( out_fresult != NULL ) out_fresult[0] = res;
    
    return (res == FR_OK && bytes_written == size) ? IOIF_FileSystem_OK : IOIF_FileSystem_INTERNAL;
}

AGRBFileSystemStatusDef ioif_filesystem_read_ex(IOIF_FILEx_t id, uint32_t offset, void* data, uint32_t size, FRESULT* out_fresult)
{
    __ASSERT_DRIVER_INITIALIZED();

    if (data == NULL || size == 0) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter
    if (id >= IOIF_FILESYSTEM_MAX_OPENED_FILES) return IOIF_FileSystem_PARAM_ERROR; //Invalid file ID
    //if (read_size == NULL) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter

    AGRBFileSystemStatusDef result = IOIF_FileSystem_INTERNAL;
    IOIF_FileSystem_FileInstance_t* file_instance = &_file_instances[id];

    if ( file_instance->is_initialized == false ) return IOIF_FileSystem_FILEINSTANCE_INVALID; //Invalid file ID
    //offset 값과 size 값이 파일 크기를 초과하는지 검사
    if ( (offset + size) > f_size(&(file_instance->file)) ) return IOIF_FileSystem_SIZE_EXCEED; //Read size exceeds file size

    #if defined(USE_FREERTOS_DMA)
    if ( size > IOIF_FILESYSTEM_READ_BOUNCE_BUFFER_SIZE ) return IOIF_FileSystem_SIZE_EXCEED; //Size exceeds bounce buffer    
    #endif
    
    FRESULT res = FR_OK;
    uint32_t bytes_read = 0;
    uint8_t* read_data = (uint8_t*)data;

    IOIF_FILESYSTEM_ACQUIRE_SEMAPHORE();
    do {
        //파일 오프셋 설정
        res = f_lseek( &(file_instance->file), offset );
        if (res != FR_OK) break; //Failed to set file offset

        #if defined(USE_FREERTOS_DMA)
        switch( AGRBFileSystem.device_type )
        {
            case IOIF_FileSystem_DeviceType_USBH_MSC:
            break;
            default: read_data = _filesystem_dma_rw_buffer;
            break;
        }        
        #endif

        res = f_read( &(file_instance->file), read_data, size, &bytes_read );
        IOIF_FILESYSTEM_WAIT_RW_COMPLETE();
        if (res != FR_OK) break;

        #if defined(USE_FREERTOS_DMA)
        switch( AGRBFileSystem.device_type )
        {
            case IOIF_FileSystem_DeviceType_USBH_MSC:
            break;
            default:
                SCB_InvalidateDCache_by_Addr((uint32_t*)_filesystem_dma_rw_buffer, size);
                memcpy(data, _filesystem_dma_rw_buffer, size);
            break;
        }        
        #endif

        result = IOIF_FileSystem_OK;;
        
    } while(0);
    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();

    if ( out_fresult != NULL ) out_fresult[0] = res;

    return result;
}

FRESULT _debug_close_file_error;
AGRBFileSystemStatusDef ioif_filesystem_close_ex(IOIF_FILEx_t id, FRESULT* out_fresult)
{
    __ASSERT_DRIVER_INITIALIZED();

    if (id >= IOIF_FILESYSTEM_MAX_OPENED_FILES) return IOIF_FileSystem_PARAM_ERROR; //Invalid file ID

    char filename[IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH];
    IOIF_FileSystem_FileInstance_t* file_instance = &_file_instances[id];
    if ( file_instance->is_initialized == false ) return IOIF_FileSystem_FILEINSTANCE_INVALID; //Invalid file ID

    FRESULT res;
    IOIF_FILESYSTEM_ACQUIRE_SEMAPHORE();
    do {
        //일단 끔.
        if ( file_instance->mode == IOIF_FileSystem_Access_APPEND ) {
            //쓰기 모드였으면 데이터 플러시
            res = f_sync( &(file_instance->file) );
            if (res != FR_OK) break;
        }

        res = f_close( &(file_instance->file) );
        _debug_close_file_error = res;
        if (res != FR_OK) break;

        //읽기 전용 모드였으면 그냥 닫기
        if ( file_instance->mode == IOIF_FileSystem_Access_READONLY ) 
        {
            memset(file_instance, 0, sizeof(IOIF_FileSystem_FileInstance_t));
            break;
        }

        //쓰기 모드라면 작성 모드에 따라 추가 작업 수행
        switch ( file_instance->meta.create_mode )
        {
        case IOIF_FileSystem_CreateMode_OVERWRITE:
        //현재 파일은 .BAK에 저장되고 있으며, 이는 원래 만들고자 했던 파일을 대체해야한다
        {
            BACKUP_FILE_NAME_RULE(filename, file_instance->filename);
            //기존 파일 삭제
            f_unlink((const char*)file_instance->filename); //실패해도 무시
            //백업 파일을 진짜 파일로 이름 변경
            res = f_rename((const char*)filename, (const char*)file_instance->filename);
        } break;
        case IOIF_FileSystem_CreateMode_INCREMENT:
        case IOIF_FileSystem_CreateMode_EXCLUSIVE:
        case IOIF_FileSystem_CreateMode_APPEND:
        default:
            /* Do nothing */
            break;
        }
        //파일 인스턴스 초기화
        memset(file_instance, 0, sizeof(IOIF_FileSystem_FileInstance_t));

    } while(0);
    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();

    if ( out_fresult != NULL ) out_fresult[0] = res;

    return (res == FR_OK) ? IOIF_FileSystem_OK : IOIF_FileSystem_INTERNAL;
}

//파일의 삭제
AGRBFileSystemStatusDef ioif_filesystem_delete_ex(const char* fullpath, FRESULT* out_fresult)
{    
    __ASSERT_DRIVER_INITIALIZED();

    if (fullpath == NULL) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter
    AGRBFileSystemStatusDef result = IOIF_FileSystem_OK;
    FRESULT res;

    IOIF_FILESYSTEM_ACQUIRE_SEMAPHORE();
    do {
        res = f_unlink((const char*)fullpath);
        if (res != FR_OK) {
            result = IOIF_FileSystem_INTERNAL; //Failed to delete file
            break;
        }
    } while(0);
    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();

    if ( out_fresult != NULL ) out_fresult[0] = res;

    return result;
}

static AGRBFileSystemStatusDef ioif_filesystem_get_size_ex(IOIF_FILEx_t id, uint32_t* size)
{
    __ASSERT_DRIVER_INITIALIZED();

    if (id >= IOIF_FILESYSTEM_MAX_OPENED_FILES) return IOIF_FileSystem_PARAM_ERROR; //Invalid file ID
    if (size == NULL) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter

    AGRBFileSystemStatusDef result = IOIF_FileSystem_FILEINSTANCE_INVALID;

    IOIF_FILESYSTEM_ACQUIRE_SEMAPHORE();
    do {
        IOIF_FileSystem_FileInstance_t* file_instance = &_file_instances[id];
        if ( file_instance->is_initialized == false ) break;

        size[0] = f_size( &(file_instance->file) );

        result = IOIF_FileSystem_OK;
    } while(0);
    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();

    return result;
}

AGRBFileSystemStatusDef ioif_filesystem_open_file_write(IOIF_FILEx_t* id, const char* path, const char* filename, const char* ext)
{
    __ASSERT_DRIVER_INITIALIZED();

    if (id == NULL) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter

    AGRBFileSystemStatusDef result = IOIF_FileSystem_FILEINSTANCE_FULL;
    IOIF_FILEx_t new_id = IOIF_FILESYSTEM_INVALID_FILE_ID;
    IOIF_FileSystem_FileInstance_t *file_instance = NULL;

    IOIF_FILESYSTEM_ACQUIRE_SEMAPHORE();
    do {
        //Find a free file instance slot
        new_id = _find_free_file_instance_slot();
        if ( new_id == IOIF_FILESYSTEM_INVALID_FILE_ID ) break; //No free slot

        file_instance = &_file_instances[new_id];
        memset(file_instance, 0, sizeof(IOIF_FileSystem_FileInstance_t));

        //파일 이름 구성
        if ( !_ioif_filesystem_construct_fullname((char*)file_instance->filename, IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH, path, filename, ext) ) {
            result = IOIF_FileSystem_PATH_INVALID; //Invalid path/filename/ext
            break;
        }

        //파일이 존재하는지 확인        

        //파일 열기
        FRESULT res;
        res = f_open(&(file_instance->file), (const char*)(file_instance->filename), (FA_WRITE | FA_OPEN_APPEND));
        if (res != FR_OK) {
            result = IOIF_FileSystem_INTERNAL; //Failed to open file
            break;
        }
 
        file_instance->is_initialized = true;
        file_instance->mode = IOIF_FileSystem_Access_APPEND;

        result = IOIF_FileSystem_OK;
        id[0] = new_id;
    } while(0);
    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();

    return result;
}

#if (0)
AGRBFileSystemStatusDef ioif_filesystem_open_file(IOIF_FILEx_t* id, const char* path, const char* filename, const char* ext, IOIF_FileSystem_AccessMode_e mode)
{
    _ioif_filesystem_open_file_error_stage = 0;

    __ASSERT_DRIVER_INITIALIZED();

    if (id == NULL) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter

    AGRBFileSystemStatusDef result = IOIF_FileSystem_OK;
    IOIF_FILEx_t assigned_id = IOIF_FILESYSTEM_INVALID_FILE_ID;
    IOIF_FileSystem_FileInstance_t* file_instance = NULL;

    _ioif_filesystem_open_file_error_stage = 1;

    IOIF_FILESYSTEM_ACQUIRE_SEMAPHORE();
    do {  
        for ( int i = 0 ; i < IOIF_FILESYSTEM_MAX_OPENED_FILES; i++ ) {
            if ( _file_instances[i].is_initialized == false ) {
                //Found a free slot
                memset(&_file_instances[i], 0, sizeof(IOIF_FileSystem_FileInstance_t));
                _file_instances[i].is_initialized = true;
                file_instance = &_file_instances[i];
                assigned_id = (IOIF_FILEx_t)(i); //File handle ID is index
                break;
            }
        }
        
        _ioif_filesystem_open_file_error_stage = 2;
        
        if ( file_instance == NULL ) {
            result = IOIF_FileSystem_FILEINSTANCE_FULL; //No free file slot
            break;
        }
        
        _ioif_filesystem_open_file_error_stage = 3;
        
        //파일 이름 구성
        if ( !_ioif_filesystem_construct_fullname((char*)file_instance->filename, IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH, path, filename, ext) ) {
            file_instance->is_initialized = false;
            result = IOIF_FileSystem_PATH_INVALID; //Invalid path/filename/ext
            break;
        }

        _ioif_filesystem_open_file_error_stage = 5;

        if ( result != IOIF_FileSystem_OK ) break;

        file_instance->mode = mode;

        _ioif_filesystem_open_file_error_stage = 6;
        
        //파일 열기
        FRESULT res;
        BYTE fmode = 0;
        switch (file_instance->mode)
        {
        case IOIF_FileSystem_Access_READONLY:
            fmode = FA_READ;
            break;
        case IOIF_FileSystem_Access_APPEND:
            fmode = ( FA_WRITE | FA_OPEN_APPEND );
            break;
        default:
            file_instance->is_initialized = false;
            result = IOIF_FileSystem_ACCESS_MODE_INVALID; //Invalid access mode
            break;
        }
        
        if ( result != IOIF_FileSystem_OK ) break;

        _ioif_filesystem_open_file_error_stage = 8;
        

        res = f_open(&(file_instance->file), (const char*)(file_instance->filename), fmode);

        if (res != FR_OK) {
            file_instance->is_initialized = false;
            result = IOIF_FileSystem_INTERNAL; //Failed to open file
            break;
        }

        _ioif_filesystem_open_file_error_stage = 9;

        res = f_sync(&(file_instance->file)); //Ensure file is synced
        if (res != FR_OK) {
            f_close(&(file_instance->file));
            file_instance->is_initialized = false;
            result = IOIF_FileSystem_INTERNAL; //Failed to sync file
            break;
        }
        result = IOIF_FileSystem_OK;
        
    } while(0);
    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();

    if (result != IOIF_FileSystem_OK) assigned_id = IOIF_FILESYSTEM_INVALID_FILE_ID;

    id[0] = assigned_id;
    
    return result;
}

AGRBFileSystemStatusDef ioif_filesystem_append_data(IOIF_FILEx_t id, const void* data, uint32_t size)
{
    __ASSERT_DRIVER_INITIALIZED();

    if (id >= IOIF_FILESYSTEM_MAX_OPENED_FILES) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter
    if (data == NULL || size == 0) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter
    
    AGRBFileSystemStatusDef result = IOIF_FileSystem_OK;
    IOIF_FileSystem_FileInstance_t* file_instance = &_file_instances[id];
    if ( file_instance->is_initialized == false ) return IOIF_FileSystem_FILEINSTANCE_INVALID; //Invalid file instance

    #if defined(USE_FREERTOS_DMA)
    if ( size > IOIF_FILESYSTEM_WRITE_BOUNCE_BUFFER_SIZE ) {
        return IOIF_FileSystem_SIZE_EXCEED; //Size exceeds bounce buffer
    }
    #endif
    
    if ( file_instance->mode != IOIF_FileSystem_Access_APPEND ) return IOIF_FileSystem_ACCESS_MODE_INVALID; //File not opened in append mode

    IOIF_FILESYSTEM_ACQUIRE_SEMAPHORE();
    do {
        FRESULT res;
        UINT bytes_written = 0;
        uint8_t* send_data = (uint8_t*)data;

        #if defined(USE_FREERTOS_DMA)
        //현재 장치가 DMA 전송을 지원하므로, DMA용 버퍼를 사용
        //다만 USB 인 경우, 이 버퍼를 사용하지 않음(USB 드라이버가 자체 버퍼를 사용하므로)
        if ( AGRBFileSystem.device_type != IOIF_FileSystem_DeviceType_USBH_MSC ) 
        {
            memset(_filesystem_dma_rw_buffer, 0, IOIF_FILESYSTEM_READ_BOUNCE_BUFFER_SIZE);
            memcpy(_filesystem_dma_rw_buffer, data, (size > IOIF_FILESYSTEM_WRITE_BOUNCE_BUFFER_SIZE ? IOIF_FILESYSTEM_WRITE_BOUNCE_BUFFER_SIZE : size));
            SCB_CleanDCache_by_Addr((uint32_t*)_filesystem_dma_rw_buffer, (size > IOIF_FILESYSTEM_WRITE_BOUNCE_BUFFER_SIZE ? IOIF_FILESYSTEM_WRITE_BOUNCE_BUFFER_SIZE : size));
            send_data = _filesystem_dma_rw_buffer; 
        }
        #endif

        res = f_write(&(file_instance->file), send_data, size, &bytes_written);
        IOIF_FILESYSTEM_WAIT_RW_COMPLETE();

        if (res != FR_OK || bytes_written != size) {
            result = IOIF_FileSystem_INTERNAL; //Failed to write data
            break;
        }
        
    } while(0);
    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();

    return result;
}

AGRBFileSystemStatusDef ioif_filesystem_seek(IOIF_FILEx_t id, uint32_t offset) //파일 포인터를 특정 위치로 이동
{
    __ASSERT_DRIVER_INITIALIZED();

    if (id >= IOIF_FILESYSTEM_MAX_OPENED_FILES) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter
    
    AGRBFileSystemStatusDef result = IOIF_FileSystem_OK;
    IOIF_FileSystem_FileInstance_t* file_instance = &_file_instances[id];
    if ( file_instance->is_initialized == false ) return IOIF_FileSystem_FILEINSTANCE_INVALID;

    IOIF_FILESYSTEM_ACQUIRE_SEMAPHORE();
    do {
        FRESULT res;
        res = f_lseek(&(file_instance->file), (FSIZE_t)offset);
        if (res != FR_OK) {
            result = IOIF_FileSystem_INTERNAL; //Failed to seek
            break;
        }
    } while(0);
    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();

    return IOIF_FileSystem_OK;
}

static FRESULT _debug_ioif_fs_read_result;
AGRBFileSystemStatusDef ioif_filesystem_read(IOIF_FILEx_t id, void* data, uint32_t size, uint32_t* read_size) //현재 파일 포인터 위치에서 데이터 읽기 시작
{
    __ASSERT_DRIVER_INITIALIZED();

    if (id >= IOIF_FILESYSTEM_MAX_OPENED_FILES) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter
    if (data == NULL || size == 0) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter
    if (size > IOIF_FILESYSTEM_READ_BOUNCE_BUFFER_SIZE) return IOIF_FileSystem_SIZE_EXCEED; //Size exceeds bounce buffer

    AGRBFileSystemStatusDef result = IOIF_FileSystem_OK;
    IOIF_FileSystem_FileInstance_t* file_instance = &_file_instances[id];
    if ( file_instance->is_initialized == false ) return IOIF_FileSystem_FILEINSTANCE_INVALID; //Invalid file instance

    #if defined(USE_FREERTOS_DMA)
    if ( size > IOIF_FILESYSTEM_READ_BOUNCE_BUFFER_SIZE ) {
        return IOIF_FileSystem_SIZE_EXCEED; //Size exceeds bounce buffer
    }   
    #endif

    uint8_t* recv_data = (uint8_t*)data;

    IOIF_FILESYSTEM_ACQUIRE_SEMAPHORE();
    do {
        FRESULT res;
        UINT bytes_read = 0;

        #if defined(USE_FREERTOS_DMA)
        memset(_filesystem_dma_rw_buffer, 0, IOIF_FILESYSTEM_READ_BOUNCE_BUFFER_SIZE);
        recv_data = _filesystem_dma_rw_buffer; 
        #endif

        res = f_read(&(file_instance->file), recv_data, size, &bytes_read);
        IOIF_FILESYSTEM_WAIT_RW_COMPLETE();

        if (res != FR_OK) {
            _debug_ioif_fs_read_result = res;
            result = IOIF_FileSystem_INTERNAL; //Failed to read data
            break;
        }

        #if defined(USE_FREERTOS_DMA)
        SCB_CleanDCache_by_Addr((uint32_t*)_filesystem_dma_rw_buffer, bytes_read);
        memcpy(data, _filesystem_dma_rw_buffer, bytes_read);
        #endif

        if ( read_size != NULL ) {
            *read_size = (uint32_t)bytes_read;
        }
    } while(0);
    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();

    return result;
}

AGRBFileSystemStatusDef ioif_filesystem_close_file(IOIF_FILEx_t id) // 열려있는 파일 닫기. 파일 인스턴스 해제
{
    __ASSERT_DRIVER_INITIALIZED();

    if (id >= IOIF_FILESYSTEM_MAX_OPENED_FILES) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter
    AGRBFileSystemStatusDef result = IOIF_FileSystem_OK;
    IOIF_FileSystem_FileInstance_t* file_instance = &_file_instances[id];
    if ( file_instance->is_initialized == false ) return IOIF_FileSystem_FILEINSTANCE_INVALID; //Invalid file instance
    IOIF_FILESYSTEM_ACQUIRE_SEMAPHORE();
    do {
        FRESULT res;
        res = f_close(&(file_instance->file));
        if (res != FR_OK) {
            result = IOIF_FileSystem_INTERNAL; //Failed to close file?...so what?
        }
        memset(file_instance, 0, sizeof(IOIF_FileSystem_FileInstance_t));
    } while(0);
    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();

    return IOIF_FileSystem_OK;
}


AGRBFileSystemStatusDef ioif_filesystem_get_file_size(IOIF_FILEx_t id, uint32_t* size)
{
    __ASSERT_DRIVER_INITIALIZED();
    if (id >= IOIF_FILESYSTEM_MAX_OPENED_FILES) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter
    if (size == NULL) return IOIF_FileSystem_PARAM_ERROR; //Invalid parameter
    
    IOIF_FILESYSTEM_ACQUIRE_SEMAPHORE();
    {
        IOIF_FileSystem_FileInstance_t* file_instance = &_file_instances[id];
        if ( file_instance->is_initialized == false ) return IOIF_FileSystem_FILEINSTANCE_INVALID; //Invalid file instance
        do {
            *size = (uint32_t)(file_instance->file.obj.objsize);
        } while(0);
    }
    IOIF_FILESYSTEM_RELEASE_SEMAPHORE();

    return IOIF_FileSystem_OK;
}
#endif
/*****************************************************/

static bool _ioif_filesystem_parse_path(char* out_path, size_t out_path_size, const char* fullname)
{
    uint32_t len = strnlen(fullname, IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH);
    if ( len == 0 || len >= IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH ) return false; //Invalid fullname
    if ( out_path == NULL || out_path_size == 0 ) return false; //Invalid output buffer
    memset(out_path, 0, out_path_size);
    const char* last_slash = strrchr(fullname, '/');
    if ( last_slash == NULL ) {
        //No path, only filename
        out_path[0] = '\0';
        return true;
    } 
    size_t path_len = (size_t)(last_slash - fullname);
    if ( path_len >= out_path_size ) return false; //Output buffer too small
    memcpy(out_path, fullname, path_len);
    out_path[path_len] = '\0';
    return true;
}

static bool _ioif_filesystem_parse_filename(char* out_filename, size_t out_filename_size, const char* fullname)
{
    uint32_t len = strnlen(fullname, IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH);
    if ( len == 0 || len >= IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH ) return false; //Invalid fullname
    if ( out_filename == NULL || out_filename_size == 0 ) return false; //Invalid output buffer
    memset(out_filename, 0, out_filename_size);

    const char* last_slash = strrchr(fullname, '/');
    const char* start_ptr = ( last_slash != NULL ) ? (last_slash + 1) : fullname;
    const char* last_dot = strrchr(start_ptr, '.');
    size_t filename_len = 0;
    if ( last_dot != NULL ) {
        filename_len = (size_t)(last_dot - start_ptr);
    } else {
        filename_len = strnlen(start_ptr, IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH);
    }

    if ( filename_len >= out_filename_size ) return false; //Output buffer too small
    memcpy(out_filename, start_ptr, filename_len);
    out_filename[filename_len] = '\0';
    return true;
}
static bool _ioif_filesystem_parse_extension(char* out_extension, size_t out_extension_size, const char* fullname)
{
    uint32_t len = strnlen(fullname, IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH);
    if ( len == 0 || len >= IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH ) return false; //Invalid fullname
    if ( out_extension == NULL || out_extension_size == 0 ) return false; //Invalid output buffer
    memset(out_extension, 0, out_extension_size);

    const char* last_slash = strrchr(fullname, '/');
    const char* start_ptr = ( last_slash != NULL ) ? (last_slash + 1) : fullname;
    const char* last_dot = strrchr(start_ptr, '.');
    if ( last_dot == NULL ) {
        //No extension
        out_extension[0] = '\0';
        return true;
    }

    size_t ext_len = strnlen(last_dot + 1, IOIF_FILESYSTEM_MAX_EXTENSION_LENGTH);
    if ( ext_len >= out_extension_size ) return false; //Output buffer too small
    memcpy(out_extension, last_dot + 1, ext_len);
    out_extension[ext_len] = '\0';
    return true;
}

//path는 루트부터 시작하는 전체 경로이며, 파일 및 확장자는 제외함
static bool _ioif_filesystem_make_directories(const char* path)
{
    uint32_t len = strnlen(path, IOIF_FILESYSTEM_MAX_FULLPATH_LENGTH);
    if ( len == 0 ) return true;//No path to create
    if ( len >= IOIF_FILESYSTEM_MAX_FULLPATH_LENGTH ) return false; //Invalid path

    char temp_path[IOIF_FILESYSTEM_MAX_FULLPATH_LENGTH];
    memset(temp_path, 0, sizeof(temp_path));

    for ( uint32_t i = 0; i < len; i++ ) {
        temp_path[i] = path[i];
        if ( path[i] == '/' ) {
            //Create directory up to this point
            FRESULT res = f_mkdir((const char*)temp_path);
            if ( res != FR_OK && res != FR_EXIST ) return false; //Failed to create directory.. 이건 심각한 오류
        }
    }

    //Create final directory
    FRESULT res = f_mkdir((const char*)temp_path);
    if ( res != FR_OK && res != FR_EXIST ) return false; //Failed to create directory.. 이건 심각한 오류

    return true;
}




static bool _ioif_filesystem_create_filename_increment(char* out_filename, size_t out_filename_size, const char* base_filename)
{   
    if ( out_filename == NULL || out_filename_size == 0 ) return false; //Invalid output buffer

    char path[IOIF_FILESYSTEM_MAX_FULLPATH_LENGTH];
    char filename[IOIF_FILESYSTEM_MAX_FILENAME_LENGTH];
    char ext[IOIF_FILESYSTEM_MAX_EXTENSION_LENGTH];
    uint32_t postfix_num = 1;
    
    if ( strnlen(base_filename, IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH) == 0 
    || strnlen(base_filename, IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH) >= IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH ) return false; //Invalid base filename

    //파싱하기
    if ( ioif_filessystem_check_path_available(base_filename) != IOIF_FileSystem_NamingRule_OK ) return false; //Invalid path

    if ( !_ioif_filesystem_parse_path(path, sizeof(path), base_filename) ) return false; //Failed to parse path
    if ( !_ioif_filesystem_parse_filename(filename, sizeof(filename), base_filename) ) return false; //Failed to parse filename
    if ( !_ioif_filesystem_parse_extension(ext, sizeof(ext), base_filename) ) return false; //Failed to parse extension

    //디렉토리 생성
    if ( !_ioif_filesystem_make_directories(path) ) return false; //Failed to create directories

    memset(out_filename, 0, out_filename_size);
    memcpy(out_filename, base_filename, out_filename_size);

    do {
        //out_filename으로 새로운 파일명 생성 시도하기
        if ( _ioif_filesystem_check_file_exist(out_filename) == false ) {
            return true; //Successfully created unique filename
        }

        //파일이 이미 존재하므로, postfix를 붙여서 다시 시도...
        memset(out_filename, 0, out_filename_size);
        snprintf(out_filename, out_filename_size, "%s (%lu)%s%s", filename, (unsigned long)postfix_num++, ( strnlen(ext, IOIF_FILESYSTEM_MAX_EXTENSION_LENGTH) > 0 ) ? "." : "", ext);

    } while( postfix_num < IOIF_FILESYSTEM_MAX_FILE_INCREMENT_TRY );

    return false;//Failed to create unique filename
}

static IOIF_FILEx_t _find_free_file_instance_slot(void)
{
    for ( int i = 0 ; i < IOIF_FILESYSTEM_MAX_OPENED_FILES; i++ ) {
        if ( _file_instances[i].is_initialized == false ) {
            return (IOIF_FILEx_t)(i); //File handle ID is index
        }
    }
    return IOIF_FILESYSTEM_INVALID_FILE_ID;
} 

static Diskio_drvTypeDef* _ioif_filesystem_get_diskio_driver(IOIF_FileSystem_DeviceType_e device_type)
{
    switch (device_type)
    {
    #if defined(HAL_HCD_MODULE_ENABLED)
    case IOIF_FileSystem_DeviceType_USBH_MSC:
        return &USBH_Driver;
        break;
    #endif
    #if defined(HAL_SD_MODULE_ENABLED) || defined(HAL_MMC_MODULE_ENABLED)
    case IOIF_FileSystem_DeviceType_SDCARD:
        return &SD_Driver;
        break;
    #endif
    default:
        return NULL;
        break;
    }
}

//파일 이름이 포함된 경로에 파일이 몇 개 있는지 계산
static uint32_t _get_path_file_count(const char* fullpath, FRESULT* out_fresult)
{
    uint32_t count = 0;
    FILINFO fno;
    DIR dir;
    FRESULT res ;
    char path[IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH];

    //디렉토리 경로 파싱
    if ( !_ioif_filesystem_parse_path(path, sizeof(path), fullpath) ) {
        if ( out_fresult != NULL ) out_fresult[0] = FR_INVALID_NAME;
        return 0; //Invalid path
    }
    
    do {
        res = f_opendir(&dir, path);
        if ( res != FR_OK )  break;

        for (;;) 
        {
            res = f_readdir(&dir, &fno);
            if ( res != FR_OK || fno.fname[0] == 0 ) break; //Error or end of dir
            //It's a file
            if ( !(fno.fattrib & AM_DIR) ) count++;
        }
        f_closedir(&dir);
    } while(0);
    if ( out_fresult != NULL ) out_fresult[0] = res;

    return count;
}

static bool _file_open_write(FIL* file, const char* path, IOIF_FileSystem_CreateMode_e mode, FRESULT* out_fresult)
{
    switch( mode ) 
    {
        case IOIF_FileSystem_CreateMode_APPEND:
            return _file_open_write_append(file, path, out_fresult);
        case IOIF_FileSystem_CreateMode_OVERWRITE:
            return _file_open_write_overwrite(file, path, out_fresult);
        case IOIF_FileSystem_CreateMode_EXCLUSIVE:
            return _file_open_write_exclusive(file, path, out_fresult);
        case IOIF_FileSystem_CreateMode_INCREMENT:
            return _file_open_write_increment(file, path, out_fresult);
        default:
            if ( out_fresult != NULL ) out_fresult[0] = FR_INVALID_PARAMETER;
            return false; //Invalid access mode
    }
}

static bool _file_open_write_append(FIL* file, const char* path, FRESULT* out_fresult)
{
    if ( file == NULL || path == NULL ) return false;    
    if ( ioif_filessystem_check_path_available(path) != IOIF_FileSystem_NamingRule_OK ) return false; //Invalid path

    FRESULT res = f_open(file, path, (FA_WRITE | FA_OPEN_APPEND));
    if ( out_fresult != NULL ) out_fresult[0] = res;
    
    return (res == FR_OK);
}

//기존 파일이 존재하면 백업 파일로 덮어쓰기(close 시점에 rename 수행)
static bool _file_open_write_overwrite(FIL* file, const char* path, FRESULT* out_fresult)
{
    if ( file == NULL || path == NULL ) return false;
    if ( ioif_filessystem_check_path_available(path) != IOIF_FileSystem_NamingRule_OK ) return false; //Invalid path

    char backup_path[IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH];    
    uint32_t len = strnlen(path, IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH);
    if ( len == 0 || (len + IOIF_FILESYSTEM_BACKUPFILE_EXTENSION_LENGTH) >= IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH ) return false; //Invalid path

    //백업 패스 생성하기
    BACKUP_FILE_NAME_RULE(backup_path, path);

    //기존에 백업 경로에 파일이 존재하는 경우, 이를 무시하고 새로 작성한다
    f_unlink((const char*)backup_path); //실패해도 무시
    
    //백업 경로로 파일 OPEN, 기존 파일이 존재하면 덮어쓰기. rename은 close 시점에 수행
    FRESULT res = f_open(file, (const char*)backup_path, (FA_WRITE | FA_CREATE_ALWAYS));
    if ( out_fresult != NULL ) out_fresult[0] = res;

    return (res == FR_OK);
}

static bool _file_open_write_exclusive(FIL* file, const char* path, FRESULT* out_fresult)
{
    if ( file == NULL || path == NULL ) return false;
    if ( ioif_filessystem_check_path_available(path) != IOIF_FileSystem_NamingRule_OK ) return false; //Invalid path

    FRESULT res = f_open(file, path, (FA_WRITE | FA_CREATE_NEW));
    if ( out_fresult != NULL ) out_fresult[0] = res;

    return (res == FR_OK);
}

static bool _file_open_write_increment(FIL* file, const char* path, FRESULT* out_fresult)
{
    if ( file == NULL || path == NULL ) return false;
    if ( ioif_filessystem_check_path_available(path) != IOIF_FileSystem_NamingRule_OK ) return false; //Invalid path

    uint32_t postfix_num = 1;

    char output[IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH];//저장할 최종 경로
    char basepath[IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH];//확장자 이전까지의 고유한 경로
    char filename[IOIF_FILESYSTEM_MAX_FILENAME_LENGTH];//파일 이름
    char extension[IOIF_FILESYSTEM_MAX_EXTENSION_LENGTH];//확장자

    if ( !_ioif_filesystem_parse_path(basepath, sizeof(basepath), path) ) return false; //파싱하기
    if ( !_ioif_filesystem_parse_filename(filename, sizeof(filename), path) ) return false; //파싱하기
    if ( !_ioif_filesystem_parse_extension(extension, sizeof(extension), path) ) return false; //파싱하기    

    memset(output, 0, sizeof(output));
    memcpy(output, path, sizeof(output));

    do {
        //언제나 NEW로 파일 열기 시도
        FRESULT res = f_open(file, (const char*)output, (FA_WRITE | FA_CREATE_NEW));

        switch( res ) 
        {
            case FR_OK: //성공적으로 열림
            {
                if ( out_fresult != NULL ) out_fresult[0] = res;
                return true; //Successfully opened unique filename
            } break;
            case FR_EXIST: //파일이 이미 존재함
            {
                //파일이 이미 존재하므로, postfix를 붙여서 다시 시도...
                memset(output, 0, sizeof(output));
                snprintf(output, sizeof(output), "%s/%s (%lu)%s%s", basepath, filename, (unsigned long)postfix_num++, ( strnlen(extension, IOIF_FILESYSTEM_MAX_EXTENSION_LENGTH) > 0 ) ? "." : "", extension);
                //만약 경로 길이가 초과되면 실패
                if ( strnlen(output, IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH) >= IOIF_FILESYSTEM_MAX_FULLNAME_LENGTH ) {
                    if ( out_fresult != NULL ) out_fresult[0] = FR_INVALID_NAME ;
                    return false; //Output path too long
                }
            } break;
            default: //다른 오류라면...
            {
                if ( out_fresult != NULL ) out_fresult[0] = res;
                return false; //Other error
            } break;
        }
    } while( postfix_num < IOIF_FILESYSTEM_MAX_FILE_INCREMENT_TRY );

    if ( out_fresult != NULL ) out_fresult[0] = FR_INVALID_NAME;
    
    return false;//Failed to create unique filename
}

/** EVENT CALLBACK **/
#if defined(HAL_HCD_MODULE_ENABLED)


#elif defined(HAL_SD_MODULE_ENABLED)
void BSP_SD_AbortCallback(void)
{
    //TODO: Implement if needed
    IOIF_FILESYSTEM_SIGNAL_RW_COMPLETE();
}

/**
  * @brief BSP Tx Transfer completed callback
  * @retval None
  * @note empty (up to the user to fill it in or to remove it if useless)
  */
void BSP_SD_WriteCpltCallback(void)
{
    //TODO: Implement if needed
    IOIF_FILESYSTEM_SIGNAL_RW_COMPLETE();
}

/**
  * @brief BSP Rx Transfer completed callback
  * @retval None
  * @note empty (up to the user to fill it in or to remove it if useless)
  */
void BSP_SD_ReadCpltCallback(void)
{
    //TODO: Implement if needed
    IOIF_FILESYSTEM_SIGNAL_RW_COMPLETE();
}
/* USER CODE END CallBacksSection_C */

/**
 * @brief  Detects if SD card is correctly plugged in the memory slot or not.
 * @param  None
 * @retval Returns if SD is detected or not
 */
__weak uint8_t BSP_SD_IsDetected(void)
{
  __IO uint8_t status = SD_PRESENT;

  /* USER CODE BEGIN IsDetectedSection */
  /* user code can be inserted here */
  /* USER CODE END IsDetectedSection */

  return status;
}

#endif // HAL_SD_MODULE_ENABLED

#endif // AGRB_IOIF_FILESYSTEM_ENABLE
