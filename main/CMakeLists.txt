cmake_minimum_required(VERSION 3.5)

set(COMPONENT_ADD_INCLUDEDIRS ".")

list(APPEND COMPONENT_SRCDIRS
	    ble
	    .)



register_component()
