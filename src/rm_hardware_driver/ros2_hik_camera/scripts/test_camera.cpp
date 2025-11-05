// 简单的HIKVision相机测试程序
// 用于验证相机SDK基本功能

#include <MvCameraControl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

int main()
{
    printf("==============================================\n");
    printf("HIKVision Camera Test Program\n");
    printf("==============================================\n\n");

    // 枚举设备
    printf("1. Enumerating devices...\n");
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
        printf("   [ERROR] Enum Devices fail! nRet [0x%x]\n", nRet);
        return -1;
    }
    
    printf("   [OK] Found %d device(s)\n", stDeviceList.nDeviceNum);
    if (stDeviceList.nDeviceNum == 0) {
        printf("   [ERROR] No camera found!\n");
        return -1;
    }

    // 创建句柄
    printf("\n2. Creating handle...\n");
    void* handle = NULL;
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
    if (MV_OK != nRet) {
        printf("   [ERROR] Create Handle fail! nRet [0x%x]\n", nRet);
        return -1;
    }
    printf("   [OK] Handle created\n");

    // 打开设备
    printf("\n3. Opening device...\n");
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet) {
        printf("   [ERROR] Open Device fail! nRet [0x%x]\n", nRet);
        MV_CC_DestroyHandle(handle);
        return -1;
    }
    printf("   [OK] Device opened\n");

    // 设置触发模式为off
    printf("\n4. Setting trigger mode to OFF...\n");
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (MV_OK != nRet) {
        printf("   [WARN] Set TriggerMode fail! nRet [0x%x]\n", nRet);
    } else {
        printf("   [OK] Trigger mode set to OFF\n");
    }

    // 设置帧率
    printf("\n4b. Setting frame rate...\n");
    nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", true);
    if (MV_OK == nRet) {
        printf("   [OK] Frame rate control enabled\n");
        nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", 10.0); // 降低到10fps测试
        if (MV_OK == nRet) {
            printf("   [OK] Frame rate set to 10 fps\n");
        } else {
            printf("   [WARN] Set frame rate fail! nRet [0x%x]\n", nRet);
        }
    } else {
        printf("   [WARN] Enable frame rate fail! nRet [0x%x]\n", nRet);
    }

    // 开始取流
    printf("\n5. Starting grabbing...\n");
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet) {
        printf("   [ERROR] Start Grabbing fail! nRet [0x%x]\n", nRet);
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return -1;
    }
    printf("   [OK] Grabbing started\n");

    // 等待相机稳定
    printf("\n6. Waiting for camera to stabilize...\n");
    sleep(1);

    // 获取图像数据
    printf("\n7. Getting image frames (10 attempts)...\n");
    MV_FRAME_OUT_INFO_EX stImageInfo;
    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
    if (MV_OK != nRet) {
        printf("   [ERROR] Get PayloadSize fail! nRet [0x%x]\n", nRet);
    } else {
        printf("   Payload size: %d bytes\n", stParam.nCurValue);
    }

    unsigned char * pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
    if (NULL == pData) {
        printf("   [ERROR] Malloc fail!\n");
        MV_CC_StopGrabbing(handle);
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return -1;
    }

    int success_count = 0;
    for (int i = 0; i < 10; i++) {
        memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
        nRet = MV_CC_GetOneFrameTimeout(handle, pData, stParam.nCurValue, &stImageInfo, 2000); // 增加超时到2秒
        if (MV_OK == nRet) {
            printf("   [%d/10] [OK] Got frame: %dx%d, size=%d bytes\n", 
                   i+1, stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameLen);
            success_count++;
        } else {
            printf("   [%d/10] [ERROR] Get frame fail! nRet [0x%x]\n", i+1, nRet);
        }
        // 根据帧率等待：10fps = 100ms间隔，但我们等待150ms以确保有新帧
        usleep(150000); // 150ms delay
    }

    // 清理
    printf("\n8. Cleaning up...\n");
    free(pData);
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    
    printf("   [OK] Cleanup complete\n");

    printf("\n==============================================\n");
    printf("Test Result: %d/10 frames captured successfully\n", success_count);
    if (success_count >= 5) {
        printf("Status: PASSED\n");
        return 0;
    } else {
        printf("Status: FAILED\n");
        return -1;
    }
}
