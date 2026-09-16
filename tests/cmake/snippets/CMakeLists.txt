# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.28.0)

set(SNIPPET_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/extra_snippet_root;${CMAKE_CURRENT_SOURCE_DIR}")
set(BOARD_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/extra_board_root")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_snippets)

# Settings under a shield regular expression must be applied exactly once, no
# matter how many of the shields in use match it. That is not observable from
# Kconfig, so count the files the snippets contributed instead.
if(DEFINED TEST_EXPECTED_SNIPPET_CONF_COUNT)
  zephyr_get_scoped(test_snippet_conf_files snippets EXTRA_CONF_FILE)
  list(LENGTH test_snippet_conf_files test_snippet_conf_count)
  if(NOT test_snippet_conf_count EQUAL TEST_EXPECTED_SNIPPET_CONF_COUNT)
    message(FATAL_ERROR
      "expected ${TEST_EXPECTED_SNIPPET_CONF_COUNT} snippet EXTRA_CONF_FILE "
      "entries, got ${test_snippet_conf_count}: ${test_snippet_conf_files}"
    )
  endif()
endif()

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
