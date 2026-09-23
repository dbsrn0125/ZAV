#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include "MvCameraControl.h"

int main() {
    setvbuf(stdout, NULL, _IONBF, 0);
    int nRet = MV_OK;
    void* handle = NULL;

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (nRet != MV_OK || stDeviceList.nDeviceNum == 0) {
        printf("[ERROR] No Hikrobot camera found. Please check USB connection.\n");
        return -1;
    }

    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
    if (nRet != MV_OK) {
        printf("[ERROR] CreateHandle failed: 0x%x\n", nRet);
        return -1;
    }

    nRet = MV_CC_OpenDevice(handle);
    if (nRet != MV_OK) {
        printf("[ERROR] OpenDevice failed: 0x%x\n", nRet);
        MV_CC_DestroyHandle(handle);
        return -1;
    }

    // Set Trigger Source Line 0
    MV_CC_SetEnumValue(handle, "TriggerMode", 1);
    MV_CC_SetEnumValue(handle, "TriggerSource", 0);
    MV_CC_SetEnumValue(handle, "LineSelector", 0); // Line 0 (Opto-isolated In)

    printf("\n========================================================\n");
    printf("  Hikrobot Real-time Voltage & Line Monitor (v2)\n");
    printf("========================================================\n");
    printf("Monitoring real-time physical electrical state on Line 0...\n");
    printf("Press Ctrl+C to stop.\n\n");

    bool last_status0 = false;
    MV_CC_SetEnumValue(handle, "LineSelector", 0);
    MV_CC_GetBoolValue(handle, "LineStatus", &last_status0);

    bool last_status2 = false;
    MV_CC_SetEnumValue(handle, "LineSelector", 2);
    MV_CC_GetBoolValue(handle, "LineStatus", &last_status2);

    printf("[CURRENT STATE] Line 0 (Opto-in): %s | Line 2 (GPIO): %s\n\n",
           last_status0 ? "HIGH" : "LOW",
           last_status2 ? "HIGH" : "LOW");

    int count = 0;
    while (true) {
        // Read Line 0
        MV_CC_SetEnumValue(handle, "LineSelector", 0);
        bool status0 = false;
        int ret0 = MV_CC_GetBoolValue(handle, "LineStatus", &status0);

        if (ret0 == MV_OK && status0 != last_status0) {
            printf("\n>>> [ELECTRICAL EVENT] Line 0 CHANGED: %s -> %s <<<\n\n",
                   last_status0 ? "HIGH" : "LOW",
                   status0 ? "HIGH (Voltage Injected!)" : "LOW (0V Ground)");
            last_status0 = status0;
        }

        // Read Line 2
        MV_CC_SetEnumValue(handle, "LineSelector", 2);
        bool status2 = false;
        int ret2 = MV_CC_GetBoolValue(handle, "LineStatus", &status2);

        if (ret2 == MV_OK && status2 != last_status2) {
            printf("\n>>> [ELECTRICAL EVENT] Line 2 CHANGED: %s -> %s <<<\n\n",
                   last_status2 ? "HIGH" : "LOW",
                   status2 ? "HIGH" : "LOW");
            last_status2 = status2;
        }

        count++;
        if (count % 100 == 0) {
            printf("... live monitoring: Line 0 = %s | Line 2 = %s ...\n",
                   last_status0 ? "HIGH" : "LOW",
                   last_status2 ? "HIGH" : "LOW");
        }

        usleep(10000); // 10ms polling
    }

    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    return 0;
}
