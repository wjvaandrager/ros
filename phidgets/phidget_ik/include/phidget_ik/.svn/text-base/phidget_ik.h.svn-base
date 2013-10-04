/**
 * \file phidget_ik.h
 * \brief PhidgetInterfaceKit interface class header file
 * 
 * Based on the TextLCD class by Tully Foote. 
 * Most member documentation was copied from the Phidget API manual.
 * 
 * \author Wouter Caarls <w.caarls@tudelft.nl>
 */

#include <phidgetspp.hh>

/// Class for interfacing with the PhidgetInterfaceKit
/** If you wish to use the InterfaceKit API's callback functions,
 *  you should subclass this and override the Handler functions:
 *  attachHandler(), inputChangeHandler(), outputChangeHandler() and sensorChangeHandler().
 */
class PhidgetIK : public Phidget
{
  public:
    /// The constructor for the PhidgetInterfaceKit.
    /** Create a new PhidgetIK interface class */
	PhidgetIK() :
	  Phidget((CPhidgetHandle*)&ik_handle_),
	  ik_handle_(0)
	{
      last_error_ = CPhidgetInterfaceKit_create(&ik_handle_);
      
      if (!last_error_)
      {
    	CPhidget_set_OnAttach_Handler((CPhidgetHandle)ik_handle_, PhidgetIK::attachDelegate, this);
    	CPhidgetInterfaceKit_set_OnInputChange_Handler(ik_handle_, PhidgetIK::inputChangeDelegate, this);
    	CPhidgetInterfaceKit_set_OnOutputChange_Handler(ik_handle_, PhidgetIK::outputChangeDelegate, this);
    	last_error_ = CPhidgetInterfaceKit_set_OnSensorChange_Handler(ik_handle_, PhidgetIK::sensorChangeDelegate, this);
      }
	}
	
    /// Initialize and connect to a device.
	/** This will connect to any or a specific PhidgetInterfaceKit
        @param serial_number The serial number to which to connect (-1 for any) */
    int init(int serial_number)
    {
      return (last_error_ = open(serial_number));
    }
    
    /// Gets the number of digital inputs supported by this board. 
    int getInputCount()
    {
      int count=-1;
      
      last_error_ = CPhidgetInterfaceKit_getInputCount(ik_handle_, &count);
      
      return count;
    }
    
    /// Gets the state of a digital input. 
    int getInputState(int index)
    {
      int state=-1;
      
      last_error_ = CPhidgetInterfaceKit_getInputState(ik_handle_, index, &state);
      
      return state;
    }
    
    /// Gets the number of digital outputs supported by this board. 
    int getOutputCount()
    {
      int count=-1;
      
      last_error_ = CPhidgetInterfaceKit_getOutputCount(ik_handle_, &count);
      
      return count;
    }
    
    /// Gets the state of a digital output. 
    int getOutputState(int index)
    {
      int state=-1;
      
      last_error_ = CPhidgetInterfaceKit_getOutputState(ik_handle_, index, &state);
      
      return state;
    }
    
    /// Sets the state of a digital output. 
    int setOutputState(int index, int state)
    {
      return (last_error_ = CPhidgetInterfaceKit_setOutputState(ik_handle_, index, state));
    }

    /// Gets the number of sensor (analog) inputs supported by this board. 
    int getSensorCount()
    {
      int count=-1;
      
      last_error_ = CPhidgetInterfaceKit_getSensorCount(ik_handle_, &count);
      
      return count;
    }
    
    /// Gets a sensor value (0-1000). 
    int getSensorValue(int index)
    {
      int value=-1;
      
      last_error_ = CPhidgetInterfaceKit_getSensorValue(ik_handle_, index, &value);
      
      return value;
    }
    
    /// Gets a sensor raw value (12-bit). 
    int getSensorRawValue(int index)
    {
      int value=-1;
      
      last_error_ = CPhidgetInterfaceKit_getSensorRawValue(ik_handle_, index, &value);
      
      return value;
    }

    /// Gets a sensor change trigger. 
    int getSensorChangeTrigger(int index)
    {
      int trigger=-1;
      
      last_error_ = CPhidgetInterfaceKit_getSensorChangeTrigger(ik_handle_, index, &trigger);
      
      return trigger;
    }
    
    /// Sets a sensor change trigger. 
    int setSensorChangeTrigger(int index, int trigger)
    {
      return (last_error_ = CPhidgetInterfaceKit_setSensorChangeTrigger(ik_handle_, index, trigger));
    }
    
    /// Gets the ratiometric state for this board. 
    int getRatiometric()
    {
      int ratiometric=-1;
      
      last_error_ = CPhidgetInterfaceKit_getRatiometric(ik_handle_, &ratiometric);
      
      return ratiometric;
    }
    
    /// Sets the ratiometric state for this board. 
    int setRatiometric(int ratiometric)
    {
      return (last_error_ = CPhidgetInterfaceKit_setRatiometric(ik_handle_, ratiometric));
    }
    
    /// Gets the Data Rate for an analog input. 
    int getDataRate(int index)
    {
      int datarate=-1;
      
      last_error_ = CPhidgetInterfaceKit_getDataRate(ik_handle_, index, &datarate);
      
      return datarate;
    }
    
    /// Sets the Data Rate (in ms) for an analog input. 
    int setDataRate(int index, int datarate)
    {
      return (last_error_ = CPhidgetInterfaceKit_setDataRate(ik_handle_, index, datarate));
    }
    
    /// Gets the maximum supported data rate (in ms) for an analog input. 
    int getDataRateMax(int index)
    {
      int max=-1;
      
      last_error_ = CPhidgetInterfaceKit_getDataRateMax(ik_handle_, index, &max);
      
      return max;
    }
    
    /// Gets the minimum supported data rate (in ms) for an analog input. 
    int getDataRateMin(int index)
    {
      int min=-1;
      
      last_error_ = CPhidgetInterfaceKit_getDataRateMin(ik_handle_, index, &min);
      
      return min;
    }
    
    /// Returns the last error generated by a CPhidget function.
    int getLastError()
    {
      return last_error_;
    }

  protected:
    /// Storage for the CPhidget handle.
    CPhidgetInterfaceKitHandle ik_handle_;
    
    /// Last error generated by a CPhidget function.
    int last_error_;
   
    /// This is called when a PhidgetInterfaceKit is attached
    /** For overriding. */
    virtual int attachHandler()
    {
      return 0;
    };

    /// This is called when a digital input changes.
    /** For overriding. */
    virtual int inputChangeHandler(int index, int inputState)
    {
      return 0;
    }
    
    /// This is called when a digital output changes.
    /** For overriding. */
    virtual int outputChangeHandler(int index, int outputState)
    {
      return 0;
    }
    
    /// This is called when a sensor value changes by more then the change trigger. 
    /** For overriding. */
    virtual int sensorChangeHandler(int index, int sensorValue)
    {
      return 0;
    }
    
  private:
    /// Delegate for calling attachHandler method.
    static int attachDelegate(CPhidgetHandle phid, void *userptr)
    {
      return ((PhidgetIK*)userptr)->attachHandler();
    };

    /// Delegate for calling inputChangeHandler method.
    static int inputChangeDelegate(CPhidgetInterfaceKitHandle phid, void *userPtr, int index, int inputState)
    {
      return ((PhidgetIK*)userPtr)->inputChangeHandler(index, inputState);
    }
    
    /// Delegate for calling outputChangeHandler method.
    static int outputChangeDelegate(CPhidgetInterfaceKitHandle phid, void *userPtr, int index, int outputState)
    {
      return ((PhidgetIK*)userPtr)->outputChangeHandler(index, outputState);
    }
    
    /// Delegate for calling sensorChangeHandler method.
    static int sensorChangeDelegate(CPhidgetInterfaceKitHandle phid, void *userPtr, int index, int sensorValue)
    {
      return ((PhidgetIK*)userPtr)->sensorChangeHandler(index, sensorValue);
    }
};
