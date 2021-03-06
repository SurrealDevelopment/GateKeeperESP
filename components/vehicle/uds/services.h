//
// Created by Justin Hoogestraat on 10/31/18.
//

/**
 *  Copyright (C) 2018 Surreal Development LLC
 *  
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef FDOMESP_SERVICES_H
#define FDOMESP_SERVICES_H




/**
 * Contains deffinitions for Unified Diagnostic Services SIDs (service identifier)
 * Although many OEMs don't follow UDS to the teeth most
 * implement some of the common features such
 * as service $22, $3E, $2C,
 *
 * See ISO 14229-1 or https://en.wikipedia.org/wiki/Unified_Diagnostic_Services
 */

// Diagnostic and Communications Management
#define UDS_DIAGNOSTIC_SESSION_CONTROL 0x10
#define UDS_DIAGNOSTIC_SESSION_CONTROL_RESPONSE 0x50
#define UDS_ECU_RESET 0x11
#define UDS_ECU_RESET_RESPONSE 0x51
#define UDS_SECURITY_ACCESS 0x27
#define UDS_SECURITY_ACCESS_RESPONSE 0x67
#define UDS_COMMUNICATION_CONTROL 0x28
#define UDS_COMMUNICATION_CONTROL_RESPONSE 0x68
#define UDS_TESTER_PRESENT 0x3E
#define UDS_TESTER_PRESENT_RESPONSE 0x7E
#define UDS_ACCESS_TIMING_PARAMETERS 0x83
#define UDS_ACCESS_TIMING_PARAMETERS_RESPONSE 0xC3
#define UDS_CONTROL_DTC_SETTINGS 0x85
#define UDS_CONTROL_DTC_SETTINGS_RESPONSE 0xC5
#define UDS_RESPONSE_ON_LINKED_EVENT 0x86
#define UDS_RESPONSE_ON_LINKED_EVENT_RESPONSE 0x86
#define UDS_LINK_CONTROL 0x87
#define UDS_LINK_CONTROL_RESPONSE 0x87

// Data Transmission
#define UDS_READ_DATA_BY_IDENTIFIER 0x22
#define UDS_READ_DATA_BY_IDENTIFIER_RESPONSE 0x62
#define UDS_READ_MEMORY_BY_ADDRESS 0x23
#define UDS_READ_MEMORY_BY_ADDRESS_RESPONSE 0x63
#define UDS_READ_SCALING_DATA_BY_IDENTIFIER 0x24
#define UDS_READ_SCALING_DATA_BY_IDENTIFIER_RESPONSE 0x64
#define UDS_READ_DATA_BY_IDENTIFIER_PERIODIC 0x2A
#define UDS_READ_DATA_BY_IDENTIFIER_PERIODIC_RESPONSE 0x6A
#define UDS_DYNAMICALLY_DEFINE_DATA_IDENTIFIER 0x2C
#define UDS_DYNAMICALLY_DEFINE_DATA_IDENTIFIER_RESPONSE 0x6C
#define UDS_WRITE_DATA_BY_IDENTIFIER 0x2E
#define UDS_WRITE_DATA_BY_IDENTIFIER_RESPONSE 0x6E
#define UDS_WRITE_MEMORY_BY_ADDRESS 0x3D
#define UDS_WRITE_MEMORY_BY_ADDRESS_RESPONSE 0x7D

// Stored data transmission
#define UDS_CLEAR_DIAGNOSTIC_INFORMATION 0x14
#define UDS_CLEAR_DIAGNOSTIC_INFORMATION_RESPONSE 0x54
#define UDS_READ_DTC_INFORMATION 0x19
#define UDS_READ_DTC_INFORMATION_RESPONSE 0x19

//Input/Output Control
#define UDS_INPUT_OUTPUT_CONTROL 0x2F
#define UDS_INPUT_OUTPUT_CONTROL_RESPONSE 0x6F

// Remote activiation of routine
#define UDS_REMOTE_ACTIVATION_OF_ROUTINE 0x31
#define UDS_REMOTE_ACTIVATION_OF_ROUTINE_RESPONSE 0x71

// Upload/Download
#define UDS_REQUEST_DOWNLOAD 0x34
#define UDS_REQUEST_DOWNLOAD_RESPONSE 0x74
#define UDS_REQUEST_UPLOAD 0x35
#define UDS_REQUEST_UPLOAD_RESPONSE 0x75
#define UDS_TRANSFER_DATA 0x36
#define UDS_TRANSFER_DATA_RESPONSE 0x76
#define UDS_REQUEST_TRANSFER_EXIT 0x37
#define UDS_REQUEST_TRANSFER_EXIT_RESPONSE 0x77
#define UDS_REQUEST_FILE_TRANSFER 0x38
#define UDS_REQUEST_FILE_TRANSFER_RESPONSE 0x78


/**
 * SAE J1979
 */
#define SAE_SHOW_CURRENT_DATA 0x01
#define SAE_SHOW_CURRENT_DATA_RESPONSE 0x41
#define SAE_SHOW_FREEZE_FRAME_DATA 0x02
#define SAE_SHOW_FREEZE_FRAME_DATA_RESPONSE 0x42
#define SAE_SHOW_STORED_DIAGNOSTIC_TROUBLE_CODES 0x03
#define SAE_SHOW_STORED_DIAGNOSTIC_TROUBLE_CODES_RESPONSE 0x43
#define SAE_CLEAR_DIAGNOSTIC_TROUBLE_CODES_AND_STORED_VALUES 0x04
#define SAE_CLEAR_DIAGNOSTIC_TROUBLE_CODES_AND_STORED_VALUES_RESPONSE 0x44
#define SAE_TEST_RESULTS_OXYGEN_SENSOR 0x05 // Non can only
#define SAE_TEST_RESULTS 0x06
#define SAE_TEST_RESULTS_RESPONSE 0x46
#define SAE_SHOW_PENDING_TROUBLE_CODES 0x07
#define SAE_SHOW_PENDING_TROUBLE_CODES_RESPONSE 0x47
#define SAE_CONTROL_OPERATION 0x08
#define SAE_CONTROL_OPERATION_RESPONSE 0x48
#define SAE_REQUEST_VEHICLE_INFORMATION 0x09
#define SAE_REQUEST_VEHICLE_INFORMATION_RESPONSE 0x49


/**
 * GMLAN - General Motors Local Area Network, used on all GM vehicles
 * from around 2005 to 2018.
 * GM is based on ISO-TP and thus works just like UDS and J1979
 * But it has its own set of services, some of which cross over with UDS
 *
 * Main reference for these will be GMW3110
 *
 * GMLAN is deprecated as of 2018, as for what is replacing it is unknown
 * at this time.
 */

#define GMLAN_READ_DATA_BY_IDENTIFIER UDS_READ_DATA_BY_IDENTIFIER
#define GMLAN_READ_DATA_BY_IDENTIFIER_RESPONSE UDS_READ_DATA_BY_IDENTIFIER_RESPONSE



#endif //FDOMESP_SERVICES_Hr
