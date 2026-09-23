#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include "MvCameraControl.h"

int main() {
    setvbuf(stdout, NULL, _IONBF, 0);
    int nRet = MV_OK;
    void* handle = NULL;

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (nRet != MV_OK || stDeviceList.nDeviceNum == 0) {
        printf("[ERROR] No Hikrobot camera found.\n");
        return -1;
    }

    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
    nRet = MV_CC_OpenDevice(handle);
    if (nRet != MV_OK) {
        printf("[ERROR] OpenDevice failed: 0x%x\n", nRet);
        return -1;
    }

    // Configure Line 2 as Trigger Source
    MV_CC_SetEnumValue(handle, "LineSelector", 2);
    MV_CC_SetEnumValue(handle, "LineMode", 0); // Input

    MV_CC_SetEnumValue(handle, "TriggerMode", 1);   // Trigger On
    MV_CC_SetEnumValue(handle, "TriggerSource", 2); // Line 2 (TTL GPIO)
    MV_CC_SetEnumValue(handle, "TriggerActivation", 0); // 0 = RisingEdge (0V -> 3.3V)

    nRet = MV_CC_StartGrabbing(handle);
    if (nRet != MV_OK) {
        printf("[ERROR] StartGrabbing failed: 0x%x\n", nRet);
        return -1;
    }

    printf("\n========================================================\n");
    printf("  Hikrobot Line 2 Hardware Trigger Shutter Test\n");
    printf("========================================================\n");
    printf("Camera is waiting for hardware trigger on LINE 2...\n");
    printf("Whenever Pin 40 pulses from LOW to HIGH, a photo is captured!\n");
    printf("Press Ctrl+C to stop.\n\n");

    int frame_cnt = 0;
    while (true) {
        MV_FRAME_OUT stImageInfo = {0};
        nRet = MV_CC_GetImageBuffer(handle, &stImageInfo, 500); // 500ms timeout
        if (nRet == MV_OK) {
            frame_cnt++;
            printf(">>> [SHUTTER TRIGGER SUCCESS!] Captured Frame #%d (FrameNum: %d, %dx%d) <<<\n",
                   frame_cnt,
                   stImageInfo.stFrameInfo.nFrameNum,
                   stImageInfo.stFrameInfo.nExtendWidth,
                   stImageInfo.stFrameInfo.nExtendHeight);
            MV_CC_FreeImageBuffer(handle, &stImageInfo);
        } else {
            printf("... waiting for trigger pulse on Line 2 (frame count: %d) ...\n", frame_cnt);
        }
    }

    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    return 0;
}
