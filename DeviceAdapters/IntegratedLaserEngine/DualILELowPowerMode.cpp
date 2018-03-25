///////////////////////////////////////////////////////////////////////////////
// FILE:          DualILELowPowerMode.cpp
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------

#include "DualILELowPowerMode.h"
#include "IntegratedLaserEngine.h"
#include "PortsConfiguration.h"
#include "ALC_REV.h"
#include <exception>

const char* const g_PropertyBaseName = "Low Power Mode [X 0.1]";
const char* const g_On = "On";
const char* const g_Off = "Off";

CDualILELowPowerMode::CDualILELowPowerMode( IALC_REV_ILEPowerManagement* Unit1PowerInterface, IALC_REV_ILEPowerManagement* Unit2PowerInterface, const CPortsConfiguration* PortsConfiguration, CIntegratedLaserEngine* MMILE ) :
  Unit1PowerInterface_( Unit1PowerInterface ),
  Unit2PowerInterface_( Unit2PowerInterface ),
  PortsConfiguration_( PortsConfiguration ),
  MMILE_( MMILE )
{
  if ( Unit1PowerInterface_ == nullptr && Unit2PowerInterface_ == nullptr )
  {
    throw std::logic_error( "CDualILELowPowerMode: Pointers to ILE Power interface for both units are invalid" );
  }
  if ( PortsConfiguration_ == nullptr )
  {
    throw std::logic_error( "CDualILELowPowerMode: Pointer to Ports configuration invalid" );
  }
  if ( MMILE_ == nullptr )
  {
    throw std::logic_error( "CDualILELowPowerMode: Pointer to main class invalid" );
  }

  int vUnit1LowPowerPortIndex = 0, vUnit2LowPowerPortIndex = 0;
  bool vUnit1Active = false, vUnit2Active = false;
  if ( Unit1PowerInterface_ )
  {
    if ( !Unit1PowerInterface_->GetLowPowerPort( &vUnit1LowPowerPortIndex ) )
    {
      throw std::runtime_error( "ILE Power GetLowPowerPort for unit1 failed" );
    }
    if ( vUnit1LowPowerPortIndex < 1 )
    {
      throw std::runtime_error( "Low Power port index for unit1 invalid [" + std::to_string( static_cast<long long>( vUnit1LowPowerPortIndex ) ) + "]" );
    }
    if ( !Unit1PowerInterface_->GetLowPowerState( &vUnit1Active ) )
    {
      throw std::runtime_error( "ILE Power GetLowPowerState for unit1 failed" );
    }
    MMILE_->LogMMMessage( "Low power mode port for unit1: " + std::to_string( static_cast<long long>( vUnit1LowPowerPortIndex ) ) + " - " + ( vUnit1Active ? g_On : g_Off ), true );
  }
  if ( Unit2PowerInterface_ )
  {
    if ( !Unit2PowerInterface_->GetLowPowerPort( &vUnit2LowPowerPortIndex ) )
    {
      throw std::runtime_error( "ILE Power GetLowPowerPort for unit2 failed" );
    }
    if ( vUnit2LowPowerPortIndex < 1 )
    {
      throw std::runtime_error( "Low Power port index for unit2 invalid [" + std::to_string( static_cast<long long>( vUnit2LowPowerPortIndex ) ) + "]" );
    }
    if ( !Unit2PowerInterface_->GetLowPowerState( &vUnit2Active ) )
    {
      throw std::runtime_error( "ILE Power GetLowPowerState for unit2 failed" );
    }
    MMILE_->LogMMMessage( "Low power mode port for unit2: " + std::to_string( static_cast<long long>( vUnit2LowPowerPortIndex ) ) + " - " + ( vUnit2Active ? g_On : g_Off ), true );
  }

  PortNames_ = PortsConfiguration_->GetPortList();

  // Create properties
  if ( !PortNames_.empty() )
  {
    std::vector<std::string> vAllowedValues;
    vAllowedValues.push_back( g_On );
    vAllowedValues.push_back( g_Off );

    std::vector<std::string>::const_iterator vPort = PortNames_.begin();
    std::vector<int> vUnitsPorts;
    long vPropertyIndex = 0;
    while ( vPort != PortNames_.end() )
    {
      std::vector<int> vUnitsProperty;
      PortsConfiguration_->GetUnitPortsForMergedPort( *vPort, &vUnitsPorts );
      if ( Unit1PowerInterface_ && vUnitsPorts[0] == vUnit1LowPowerPortIndex )
      {
        vUnitsProperty.push_back( 0 );
        MMILE_->LogMMMessage( "Port " + *vPort + " low power mode available for unit1", true );
      }
      if ( Unit2PowerInterface_ && vUnitsPorts[1] == vUnit2LowPowerPortIndex )
      {
        vUnitsProperty.push_back( 1 );
        MMILE_->LogMMMessage( "Port " + *vPort + " low power mode available for unit2", true );
      }
      if ( !vUnitsProperty.empty() )
      {
        // Create a property if at least one of the units' port in the current virtual port supports low power mode
        std::string vPropertyName = std::string( "Port " ) + *vPort + "-" + g_PropertyBaseName;
        CPropertyActionEx* vAct = new CPropertyActionEx( this, &CDualILELowPowerMode::OnValueChange, vPropertyIndex );
        MMILE_->CreateStringProperty( vPropertyName.c_str(), ( ( vUnit1Active || vUnit2Active ) ? g_On : g_Off ), false, vAct );
        MMILE_->SetAllowedValues( vPropertyName.c_str(), vAllowedValues );
        ++vPropertyIndex;
        UnitsPropertyMap_.push_back( vUnitsProperty );
      }
      ++vPort;
    }
  }
}

CDualILELowPowerMode::~CDualILELowPowerMode()
{
}

int CDualILELowPowerMode::OnValueChange( MM::PropertyBase * Prop, MM::ActionType Act, long UnitsPropertyIndex )
{
  int vRet = DEVICE_OK;
  if ( Act == MM::AfterSet )
  {
    if ( Unit1PowerInterface_ == nullptr && Unit2PowerInterface_ == nullptr )
    {
      return ERR_DEVICE_NOT_CONNECTED;
    }
    if ( UnitsPropertyIndex >= UnitsPropertyMap_.size() )
    {
      MMILE_->LogMMMessage( "Low Power Mode: Invalid unit property index. Index = " + std::to_string( static_cast<long long>( UnitsPropertyIndex ) ) + " - Map size = " + std::to_string( static_cast<long long>( UnitsPropertyMap_.size() ) ) );
      return DEVICE_ERR;
    }
    std::string vValue;
    Prop->Get( vValue );
    bool vEnabled = ( vValue == g_On );
    std::vector<int>* vUnitsProperty = &(UnitsPropertyMap_[UnitsPropertyIndex]);
    std::vector<int>::const_iterator vUnitsIt = vUnitsProperty->begin();
    bool vPowerStateUpdated = false;
    while ( vUnitsIt != vUnitsProperty->end() )
    {
      IALC_REV_ILEPowerManagement* vPowerInterface = nullptr;
      if ( *vUnitsIt == 0 )
      {
        vPowerInterface = Unit1PowerInterface_;
      }
      if ( *vUnitsIt == 1 )
      {
        vPowerInterface = Unit2PowerInterface_;
      }
      if ( vPowerInterface && vPowerInterface->SetLowPowerState( vEnabled ) )
      {
        vPowerStateUpdated = true;
      }
      else
      {
        std::string vErrorLogBase = std::string( vEnabled ? "Enabling" : "Disabling" ) + " low power state for unit" + std::to_string( static_cast<long long>( *vUnitsIt + 1 ) ) + " FAILED.";
        if ( vPowerInterface )
        {
          MMILE_->LogMMMessage( vErrorLogBase + " Pointer to ILE power interface invalid." );
        }
        else
        {
          MMILE_->LogMMMessage( vErrorLogBase );
        }
        vRet = ERR_LOWPOWERMODE_SET;
      }
      ++vUnitsIt;
    }
    if ( vPowerStateUpdated )
    {
      MMILE_->CheckAndUpdateLasers();
    }
  }
  return vRet;
}

void CDualILELowPowerMode::UpdateILEInterface( IALC_REV_ILEPowerManagement* Unit1PowerInterface, IALC_REV_ILEPowerManagement* Unit2PowerInterface )
{
  Unit1PowerInterface_ = Unit1PowerInterface;
  Unit2PowerInterface_ = Unit2PowerInterface;
}