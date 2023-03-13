#ifndef _WIN32
#include <unistd.h>
#define Sleep(msec) usleep(msec*1000)
#define __int64 long long
#else
#include <windows.h>
#include <dos.h>
#include <conio.h>
#endif

#include "../../MMDevice/ModuleInterface.h"
//
//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//
//#include "resource.h"
//#include "mhdefin.h"
//#include "errorcodes.h"
////#include "mhlib.h"

#include <windows.h>
#include <iostream>
#include <vector>
#include "MultiHarp.h"


/////////////////////////////////////////////////////////////////////////////////
//// Exported MMDevice API
/////////////////////////////////////////////////////////////////////////////////
//MODULE_API void InitializeModuleData()
//{
//	RegisterDevice(g_MHDeviceName, MM::GenericDevice, "PicoQuant TCSPC box: MultiHarp");
//}
//
//MODULE_API MM::Device* CreateDevice(const char* deviceName)
//{
//	if (deviceName == 0)
//		return 0;
//
//	if (strcmp(deviceName, g_MHDeviceName) == 0)
//	{
//		return new MultiHarp();
//	}
//
//	return 0;
//}
//
//MODULE_API void DeleteDevice(MM::Device* pDevice)
//{
//	delete pDevice;
//}
//
//
///////////////////////////////////////////////////////////
////For the piezo pump
///////////////////////////////////////////////////////////
//
//
//MultiHarp::MultiHarp() : CGenericBase<MultiHarp>(),
//	initialized_(false)
//{
//	// From some uM base functions included?
//	InitializeDefaultErrorMessages();
//}
//
//
//MultiHarp::~MultiHarp()
//{
//}
//
//void MultiHarp::GetName(char* name) const
//{
//	// Return the name used to refer to this device adapter
//	CDeviceUtils::CopyLimitedString(name, g_MHDeviceName);
//}
//
//int MultiHarp::Initialize()
//{
//	if (initialized_)
//		return DEVICE_OK;
//
//	// Name
//	int ret = CreateProperty(MM::g_Keyword_Name, g_MHDeviceName, MM::String, true);
//	if (DEVICE_OK != ret)
//		return ret;
//
//	// Description
//	ret = CreateProperty(MM::g_Keyword_Description, "Bartels piezo pump - QuadKey", MM::String, true);
//	if (DEVICE_OK != ret)
//		return ret;
//
//	//Adding options exposed to the MM GUI here:
//	ret = CreateStringProperty(g_PropName_VER, "VERSION", false, new CPropertyAction(this, &MultiHarp::Get_Ver));
//	if (DEVICE_OK != ret)
//		return ret;
//
//	//changedTime_ = GetCurrentMMTime();
//	initialized_ = true;
//	return DEVICE_OK;
//}
//
//int MultiHarp::Shutdown()
//{
//	if (initialized_)
//	{
//		initialized_ = false;
//	}
//	return DEVICE_OK;
//}
//
//bool MultiHarp::Busy()
//{
//	/*if (GetCurrentMMTime() - changedTime_ < command_wait_time) {
//		return true;
//	}
//	else {*/
//		return false;
//	//}
//}
//
//
//int MultiHarp::Get_Ver(MM::PropertyBase* pProp, MM::ActionType eAct)
//{
//	return DEVICE_OK;
//}


int main(int argc, char* argv[])
{
	char LIB_Version[8];
	

	printf("\nMultiHarp MHLib Demo Application                      PicoQuant GmbH, 2021");
	printf("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
	MH_GetLibraryVersion(LIB_Version);
	printf("\nLibrary version is %s\n", LIB_Version);
}


