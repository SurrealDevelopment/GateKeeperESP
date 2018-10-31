cmake_minimum_required(VERSION 3.5)

set(COMPONENT_ADD_INCLUDEDIRS ".")

list(APPEND COMPONENT_SRCDIRS 
	    ble
        obd2/can
		obd2/protocol
	.)



register_component()
