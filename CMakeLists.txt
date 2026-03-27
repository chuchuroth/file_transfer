cmake_minimum_required(VERSION 3.16)
project(modbus_rtu_protocol VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options
option(BUILD_EXAMPLES "Build example applications" ON)
option(ENABLE_LOGGING "Enable debug logging" OFF)

# Logging flag
if(ENABLE_LOGGING)
    add_compile_definitions(ENABLE_MODBUS_RTU_LOGGING)
endif()

# Header-only library target
add_library(modbus_rtu INTERFACE)
target_include_directories(modbus_rtu INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Required system libraries
find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
# NeuraSync communication layer
# ---------------------------------------------------------------------------
# Dependencies (system-installed):
#   neura_sync >= 1.7  (wraps FastDDS 3.x — provides ParticipantFactory,
#                        PublisherFactory, SubscriberFactory, NeuraSyncNode)
#   fastcdr >= 2.3.4   (CDR serialization for generated IDL types)
#   fastdds >= 3.4.1   (DDS middleware — transitive dep of neura_sync)
# ---------------------------------------------------------------------------
find_package(neura_sync REQUIRED)
find_package(fastcdr 2 REQUIRED)
find_package(fastdds 3 REQUIRED)
find_package(nlohmann_json 3.11 REQUIRED)

# ---------------------------------------------------------------------------
# IDL-generated DDS type support (pre-generated)
# ---------------------------------------------------------------------------
# The type support files were generated offline by a FastDDS 3.x-compatible
# fastddsgen (v4.x) and committed to neurasync/generated/.  We use them
# directly to avoid requiring a compatible fastddsgen at build time — the
# system-packaged fastddsgen 2.3.0 produces output incompatible with
# FastDDS 3.4.1 headers.
set(NEURASYNC_GEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/neurasync/generated")

# Shared DDS types library (generated from ModbusRTUBridge.idl)
add_library(neurasync_types SHARED
    ${NEURASYNC_GEN_DIR}/ModbusRTUBridgePubSubTypes.cxx
    ${NEURASYNC_GEN_DIR}/ModbusRTUBridgeTypeObjectSupport.cxx
)
target_include_directories(neurasync_types PUBLIC ${NEURASYNC_GEN_DIR})
target_link_libraries(neurasync_types PUBLIC fastcdr fastdds)

# Gripper IDL generation + types library
set(GRIPPER_IDL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/neurasync/idl")
set(GRIPPER_GEN_DIR "${NEURASYNC_GEN_DIR}/gripper")
set(GRIPPER_COMPAT_DIR "${CMAKE_CURRENT_BINARY_DIR}/gripper_idl_compat")

# Prefer Fast-DDS-Gen 4.2.0 from the vendored source tree to match
# Fast DDS 3.4.1 / Fast CDR 2.3.4 APIs. Falls back to system fastddsgen.
set(FASTDDSGEN_42_JAR "${CMAKE_CURRENT_SOURCE_DIR}/Fast-DDS-Gen/build/libs/fastddsgen.jar")
find_package(Java COMPONENTS Runtime)
if(Java_FOUND AND EXISTS "${FASTDDSGEN_42_JAR}")
    set(FASTDDSGEN_CMD ${Java_JAVA_EXECUTABLE} -jar "${FASTDDSGEN_42_JAR}")
else()
    find_program(FASTDDSGEN_EXECUTABLE fastddsgen REQUIRED)
    set(FASTDDSGEN_CMD ${FASTDDSGEN_EXECUTABLE})
endif()

set(GRIPPER_IDL_FILES
    "${GRIPPER_IDL_DIR}/GripperCapabilities.idl"
    "${GRIPPER_IDL_DIR}/GripperCommand.idl"
    "${GRIPPER_IDL_DIR}/GripperState.idl"
)

set(GRIPPER_GEN_OUTPUTS
    "${GRIPPER_GEN_DIR}/GripperCapabilities.hpp"
    "${GRIPPER_GEN_DIR}/GripperCapabilitiesPubSubTypes.cxx"
    "${GRIPPER_GEN_DIR}/GripperCapabilitiesPubSubTypes.hpp"
    "${GRIPPER_GEN_DIR}/GripperCapabilitiesTypeObjectSupport.cxx"
    "${GRIPPER_GEN_DIR}/GripperCapabilitiesTypeObjectSupport.hpp"
    "${GRIPPER_GEN_DIR}/GripperCommand.hpp"
    "${GRIPPER_GEN_DIR}/GripperCommandPubSubTypes.cxx"
    "${GRIPPER_GEN_DIR}/GripperCommandPubSubTypes.hpp"
    "${GRIPPER_GEN_DIR}/GripperCommandTypeObjectSupport.cxx"
    "${GRIPPER_GEN_DIR}/GripperCommandTypeObjectSupport.hpp"
    "${GRIPPER_GEN_DIR}/GripperState.hpp"
    "${GRIPPER_GEN_DIR}/GripperStatePubSubTypes.cxx"
    "${GRIPPER_GEN_DIR}/GripperStatePubSubTypes.hpp"
    "${GRIPPER_GEN_DIR}/GripperStateTypeObjectSupport.cxx"
    "${GRIPPER_GEN_DIR}/GripperStateTypeObjectSupport.hpp"
)

add_custom_command(
    OUTPUT ${GRIPPER_GEN_OUTPUTS}
    COMMAND ${CMAKE_COMMAND} -E make_directory "${GRIPPER_GEN_DIR}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${GRIPPER_COMPAT_DIR}"
    COMMAND ${CMAKE_COMMAND} -E env bash -lc "sed -E 's#//.*$$##; s/= *[0-9]+//g' '${GRIPPER_IDL_DIR}/GripperCapabilities.idl' > '${GRIPPER_COMPAT_DIR}/GripperCapabilities.idl'"
    COMMAND ${CMAKE_COMMAND} -E env bash -lc "sed -E 's#//.*$$##; s/= *[0-9]+//g' '${GRIPPER_IDL_DIR}/GripperCommand.idl' > '${GRIPPER_COMPAT_DIR}/GripperCommand.idl'"
    COMMAND ${CMAKE_COMMAND} -E env bash -lc "sed -E 's#//.*$$##; s/= *[0-9]+//g' '${GRIPPER_IDL_DIR}/GripperState.idl' > '${GRIPPER_COMPAT_DIR}/GripperState.idl'"
    COMMAND ${FASTDDSGEN_CMD} -replace -d "${GRIPPER_GEN_DIR}" "${GRIPPER_COMPAT_DIR}/GripperCapabilities.idl"
    COMMAND ${FASTDDSGEN_CMD} -replace -d "${GRIPPER_GEN_DIR}" "${GRIPPER_COMPAT_DIR}/GripperCommand.idl"
    COMMAND ${FASTDDSGEN_CMD} -replace -d "${GRIPPER_GEN_DIR}" "${GRIPPER_COMPAT_DIR}/GripperState.idl"
    DEPENDS ${GRIPPER_IDL_FILES}
    COMMENT "Generating gripper DDS types via fastddsgen"
    VERBATIM
)

add_custom_target(gripper_idl_gen DEPENDS ${GRIPPER_GEN_OUTPUTS})

add_library(neurasync_gripper_types SHARED
    "${GRIPPER_GEN_DIR}/GripperCapabilitiesPubSubTypes.cxx"
    "${GRIPPER_GEN_DIR}/GripperCapabilitiesTypeObjectSupport.cxx"
    "${GRIPPER_GEN_DIR}/GripperCommandPubSubTypes.cxx"
    "${GRIPPER_GEN_DIR}/GripperCommandTypeObjectSupport.cxx"
    "${GRIPPER_GEN_DIR}/GripperStatePubSubTypes.cxx"
    "${GRIPPER_GEN_DIR}/GripperStateTypeObjectSupport.cxx"
)
add_dependencies(neurasync_gripper_types gripper_idl_gen)
target_include_directories(neurasync_gripper_types PUBLIC
    ${NEURASYNC_GEN_DIR}
    ${GRIPPER_GEN_DIR}
)
target_link_libraries(neurasync_gripper_types PUBLIC fastcdr fastdds)

# ---------------------------------------------------------------------------
# Application targets
# ---------------------------------------------------------------------------

# qnc_bridge: Pi CM5 firmware bridge — subscribes to DDS commands from the
# robot computer via NeuraSync, forwards them to the RS485 field device,
# and publishes responses + stats back over DDS.
add_executable(qnc_bridge deploy_on_qnc/qnc_bridge.cpp)
# Build the subscriber implementation into the same executable.
# Absolute workspace path requested by integration requirements.
set(GRIPPER_COMMAND_SUBSCRIBER_SRC
    "/home/chuchu.xu/Documents/PROTOTYPE_WORKSPACE_PLAN/neurasync-robot-client/qnc-control-panel/GripperCommandSubscriber.cpp"
)
target_sources(qnc_bridge PRIVATE ${GRIPPER_COMMAND_SUBSCRIBER_SRC})
target_compile_definitions(qnc_bridge PRIVATE QNC_BRIDGE_EMBED_SUBSCRIBER=1)
target_include_directories(qnc_bridge PRIVATE
    ${NEURASYNC_GEN_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
add_dependencies(qnc_bridge gripper_idl_gen)
target_link_libraries(qnc_bridge PRIVATE
    neurasync_types
    neurasync_gripper_types
    neura_sync::neura_sync
    nlohmann_json::nlohmann_json
    Threads::Threads
)
message(STATUS "  qnc_bridge: enabled (QNC RS485<->DDS bridge via NeuraSync)")

# dds_command_publisher: pure DDS command runner (no local serial I/O) —
# publishes command topics via NeuraSync, relies on qnc_bridge for hardware.
add_executable(dds_command_publisher src/gripper_control_dds.cpp)
target_include_directories(dds_command_publisher PRIVATE ${NEURASYNC_GEN_DIR})
target_link_libraries(dds_command_publisher PRIVATE
    neurasync_types
    neura_sync::neura_sync
    Threads::Threads
)
message(STATUS "  dds_command_publisher: enabled (pure NeuraSync command runner)")

# gripper_runtime_validator: runtime validation library for gripper hardware
# Provides pub/sub validation for gripper command/state feedback loops.
add_library(gripper_validator SHARED
    src/gripper_runtime_validator.cpp
)
target_include_directories(gripper_validator PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${NEURASYNC_GEN_DIR}
)
target_link_libraries(gripper_validator PUBLIC
    neurasync_gripper_types
    neura_sync::neura_sync
    Threads::Threads
)
message(STATUS "  gripper_validator: enabled (DDS pub/sub validation library)")

# gripper_validation_test: executable for runtime gripper hardware validation
# Tests responsiveness, command/state correlation, and parameter matching.
add_executable(gripper_validation_test src/gripper_validation_test.cpp)
target_include_directories(gripper_validation_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${NEURASYNC_GEN_DIR}
)
add_dependencies(gripper_validation_test gripper_idl_gen)
target_link_libraries(gripper_validation_test PRIVATE
    gripper_validator
    neurasync_gripper_types
    neura_sync::neura_sync
    Threads::Threads
)
message(STATUS "  gripper_validation_test: enabled (runtime validation executable)")

# ---------------------------------------------------------------------------
# Modbus RTU Bridge Tests and Examples
# ---------------------------------------------------------------------------

# modbus_slave_test: unit tests + serial listener for the Modbus RTU layer
# Tests frame parsing (FC03, FC06, FC10), CRC validation, exception handling.
# Modes: --listen /dev/ttyUSB0 115200 (plain) or --discover (wide register bank)
add_executable(modbus_slave_test src/modbus_rtu_slave_test.cpp)
target_include_directories(modbus_slave_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(modbus_slave_test PRIVATE Threads::Threads)
message(STATUS "  modbus_slave_test: enabled (Modbus RTU frame tests + serial listener)")

# modbus_bridge_test: integration test for Modbus bridge adapter
# Tests serial I/O, register synchronization, callbacks, data sync
add_executable(modbus_bridge_test src/modbus_bridge_test.cpp)
target_include_directories(modbus_bridge_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(modbus_bridge_test PRIVATE Threads::Threads)
message(STATUS "  modbus_bridge_test: enabled (Modbus bridge integration tests)")

# Install targets
install(TARGETS qnc_bridge DESTINATION bin)
install(TARGETS dds_command_publisher DESTINATION bin)
install(TARGETS gripper_validation_test DESTINATION bin)
install(TARGETS gripper_validator DESTINATION lib)
install(FILES include/modbus_rtu_common.hpp DESTINATION include)
install(FILES include/gripper_runtime_validator.hpp DESTINATION include)
# Install Modbus RTU bridge test
install(TARGETS modbus_slave_test DESTINATION bin)
install(TARGETS modbus_bridge_test DESTINATION bin)

# Install Modbus headers
install(FILES include/modbus_rtu_slave.hpp DESTINATION include)
install(FILES include/modbus_bridge.hpp DESTINATION include)
install(FILES include/modbus_data_sync.hpp DESTINATION include)
install(FILES include/serial_utils.hpp DESTINATION include)
install(FILES neurasync/idl/ModbusRTUBridge.idl DESTINATION share/idl)

# Package configuration
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/modbus_rtu_protocol-config-version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Summary
message(STATUS "")
message(STATUS "QNC Protocol Bridge Configuration:")
message(STATUS "  Version:          ${PROJECT_VERSION}")
message(STATUS "  Build examples:   ${BUILD_EXAMPLES}")
message(STATUS "  Enable logging:   ${ENABLE_LOGGING}")
message(STATUS "")
