#include <stdio.h>
#include "MvCameraControl.h"

int main() {
    void* handle = NULL;
    MV_CC_DEVICE_INFO_LIST stList = {0};
    MV_CC_EnumDevices(MV_USB_DEVICE, &stList);
    if (stList.nDeviceNum == 0) return 1;
    MV_CC_CreateHandle(&handle, stList.pDeviceInfo[0]);
    MV_CC_OpenDevice(handle);

    MVCC_ENUMVALUE enumVal = {0};
    MV_CC_GetEnumValue(handle, "TriggerSource", &enumVal);
    printf("TriggerSource supported num: %d\n", enumVal.nSupportedNum);
    for (int i = 0; i < enumVal.nSupportedNum; i++) {
        printf("TriggerSource[%d] = %d\n", i, enumVal.nSupportValue[i]);
    }

    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    return 0;
}
