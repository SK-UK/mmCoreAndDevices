//#pragma once
//#ifndef _WIN32
//#define _stdcall
//#endif

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include "../../MMDevice/MMDevice.h"
#include "../../MMDevice/DeviceBase.h"
#include "../../MMDevice/DeviceUtils.h"
#include "../../MMDevice/ModuleInterface.h"

static const char* g_MHDeviceName = "PicoQuant MultiHarp";
static const char* g_PropName_VER = "DLL VERSION";

/////////////////////////////////////////////
// Predefined constants
//////////////////////////////////////////
#define command_wait_time				100

class MultiHarp : public CGenericBase<MultiHarp>
{
public:
	MultiHarp();
	~MultiHarp();

	// MMDevice API
	// ------------
	// Functions here are found in the .cpp file
	int Initialize();
	int Shutdown();
	void GetName(char* pszName) const;
	bool Busy();

	//Action interface
	//----------------
	int Get_Ver(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
	bool initialized_;

	//MM::MMTime changedTime_;
};

