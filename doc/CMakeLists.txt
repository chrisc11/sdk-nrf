# Copyright (c) 2019-2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause

# Builds combined documentation for all documentation sets: nRF (including
# Doxygen documentation), Zephyr, MCUboot, etc.
#
# We use our own Sphinx configuration files when building the documentation set
# for each repository, instead of reusing configuration files. See e.g.
# doc/nrf/conf.py and doc/zephyr/conf.py.
#
# Intersphinx (http://www.sphinx-doc.org/en/master/usage/extensions/intersphinx.html)
# is used to link documentation sets together. It is configured in the Sphinx
# configuration files.

cmake_minimum_required(VERSION 3.15)
project(nrf-connect-sdk-doc LANGUAGES NONE)

set(NO_BOILERPLATE TRUE)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE} ..)

#-------------------------------------------------------------------------------
# Options

set(SPHINXOPTS_DEFAULT -j auto CACHE INTERNAL "Default Sphinx Options")
set(SPHINXOPTS_EXTRA "" CACHE STRING "Extra Sphinx Options")

#-------------------------------------------------------------------------------
# Dependencies

find_package(PythonInterp 3.4)
set(DOXYGEN_SKIP_DOT True)
find_package(Doxygen REQUIRED)

find_program(SPHINXBUILD sphinx-build)
if(NOT SPHINXBUILD)
  message(FATAL_ERROR "The 'sphinx-build' command was not found")
endif()

#-------------------------------------------------------------------------------
# Functions

# Add a new docset.
#
# Args:
# - name: Docset name.
# - env: Environment passed to the Sphinx build.
#
# This function configures multiple targets which can be used to build a docset.
# The docset configuration (conf.py) is expected to be at the ${name} folder
# (relative to the current directory). The sources are taken from the
# ${name}/src folder (relative to the build directory). This means that docsets
# need to make use of the external_content extension in order to gather all
# docset sources into that folder.
#
# Configured targets:
# - ${name}: Alias for ${name}-html.
# - ${name}-inventory: Run Sphinx "inventory" build. It requires to enable
#   the "inventory" builder on the docset conf.py. This target can be used
#   to solve circular dependencies between docsets.
# - ${name}-html: Run Sphinx "html" build.
# - ${name}-linkcheck: Run Sphinx "linkcheck" target.
# - ${name}-clean: Clean build artifacts.
#
function(add_docset name env)
  set(DOCSET_CFG_DIR ${CMAKE_CURRENT_LIST_DIR}/${name})
  set(DOCSET_BUILD_DIR ${CMAKE_BINARY_DIR}/${name})
  set(DOCSET_SRC_DIR ${CMAKE_BINARY_DIR}/${name}/src)
  set(DOCSET_HTML_DIR ${CMAKE_BINARY_DIR}/html/${name})
  set(DOCSET_MAKE_DIRS ${DOCSET_BUILD_DIR};${DOCSET_SRC_DIR})
  set(DOCSET_CLEAN_DIRS ${DOCSET_BUILD_DIR};${DOCSET_HTML_DIR})

  add_doc_target(
    ${name}-inventory
    COMMAND ${CMAKE_COMMAND} -E make_directory ${DOCSET_MAKE_DIRS}
    COMMAND ${CMAKE_COMMAND} -E env ${env}
    ${SPHINXBUILD}
      -b inventory
      -c ${DOCSET_CFG_DIR}
      -w ${DOCSET_BUILD_DIR}/inventory.log
      ${SPHINXOPTS_DEFAULT}
      ${DOCSET_SRC_DIR}
      ${DOCSET_HTML_DIR}
    USES_TERMINAL
  )

  add_doc_target(
    ${name}-html
    COMMAND ${CMAKE_COMMAND} -E make_directory ${DOCSET_MAKE_DIRS}
    COMMAND ${CMAKE_COMMAND} -E env ${env}
    ${SPHINXBUILD}
      -b html
      -c ${DOCSET_CFG_DIR}
      -w ${DOCSET_BUILD_DIR}/html.log
      ${SPHINXOPTS_DEFAULT}
      ${SPHINXOPTS_EXTRA}
      ${DOCSET_SRC_DIR}
      ${DOCSET_HTML_DIR}
    USES_TERMINAL
  )

  add_custom_target(
    ${name}-linkcheck
    COMMAND ${CMAKE_COMMAND} -E make_directory ${DOCSET_MAKE_DIRS}
    COMMAND ${CMAKE_COMMAND} -E env ${env}
    ${SPHINXBUILD}
      -b linkcheck
      -c ${DOCSET_CFG_DIR}
      -w ${DOCSET_BUILD_DIR}/linkcheck.log
      ${SPHINXOPTS_DEFAULT}
      ${SPHINXOPTS_EXTRA}
      ${DOCSET_SRC_DIR}
      ${DOCSET_BUILD_DIR}
    USES_TERMINAL
  )

  set_target_properties(
    ${name}-inventory ${name}-inventory-all
    ${name}-html ${name}-html-all
    ${name}-linkcheck
    PROPERTIES
      ADDITIONAL_CLEAN_FILES "${DOCSET_CLEAN_DIRS}"
  )

  if(${CMAKE_VERSION} VERSION_LESS 3.17)
    add_custom_target(
      ${name}-clean
      COMMAND ${CMAKE_COMMAND} -E remove_directory ${DOCSET_CLEAN_DIRS}
    )
  else()
    add_custom_target(
      ${name}-clean
      COMMAND ${CMAKE_COMMAND} -E rm -rf ${DOCSET_CLEAN_DIRS}
    )
  endif()

  add_custom_target(${name})
  add_dependencies(${name} ${name}-html)
endfunction()

# Create a custom doc target.
#
# This function has the same signature as `add_custom_target()`
#
# The function will create two targets for the doc build system.
# - Target 1 named: `<name>`
# - Target 2 named: `<name>-all`
#
# Both targets will produce same result, but target 1 is useful when only
# wanting to build a subset of the docs and missing references to other targets
# are acceptable (warnings will be generated).
#
# Target 2 is used for complete docset builds where it is important that build
# order of each target is under full control.
#
function(add_doc_target name)
  add_custom_target(${name} ${ARGN})
  add_custom_target(${name}-all ${ARGN})
endfunction()

#-------------------------------------------------------------------------------
# Environment & Paths

# Set various *_BASE variables pointing to the nrf/, zephyr/, etc.,
# directories. Derive them automatically if they're not set in the environment,
# by assuming that e.g. nrfxlib can be found at ../../nrfxlib/. Also add them
# to the environment if they're not there.
#

get_filename_component(NRF_BASE ${CMAKE_CURRENT_LIST_DIR}../ DIRECTORY)
set(ENV{NRF_BASE} ${NRF_BASE})

if(NOT DEFINED ENV{MCUBOOT_BASE})
  get_filename_component(MCUBOOT_BASE ${CMAKE_CURRENT_LIST_DIR}/../../bootloader/mcuboot/ REALPATH)
  set(ENV{MCUBOOT_BASE} ${MCUBOOT_BASE})
endif()

if(NOT DEFINED ENV{NRFXLIB_BASE})
  get_filename_component(NRFXLIB_BASE ${CMAKE_CURRENT_LIST_DIR}/../../nrfxlib/ REALPATH)
  set(ENV{NRFXLIB_BASE} ${NRFXLIB_BASE})
endif()

message(STATUS "ZEPHYR_BASE: ${ZEPHYR_BASE}")
message(STATUS "NRF_BASE: $ENV{NRF_BASE}")
message(STATUS "MCUBOOT_BASE: $ENV{MCUBOOT_BASE}")
message(STATUS "NRFXLIB_BASE: $ENV{NRFXLIB_BASE}")

set(ZEPHYR_BINARY_DIR ${CMAKE_BINARY_DIR}/zephyr)
set(NRF_BINARY_DIR ${CMAKE_BINARY_DIR}/nrf)
set(NRFXLIB_BINARY_DIR ${CMAKE_BINARY_DIR}/nrfxlib)
set(KCONFIG_BINARY_DIR ${CMAKE_BINARY_DIR}/Kconfig)

# HTML output directory
set(HTML_DIR ${CMAKE_BINARY_DIR}/html)
file(MAKE_DIRECTORY ${HTML_DIR})

#-------------------------------------------------------------------------------
# docset: Zephyr

# Add the 'zephyr' target for building the Zephyr documentation. We reuse
# doc/CMakeLists.txt from the Zephyr repository, but use our own Sphinx
# configuration from doc/zephyr/conf.py. The generated HTML is placed in the
# common Sphinx HTML output folder.

# Parameters to doc/CMakeLists.txt in Zephyr. KCONFIG_OUTPUT is used to find
# objects.inv from the Kconfig docs, to link the Zephyr docs to the Kconfig
# docs.
set(SPHINXOPTS ${SPHINXOPTS_DEFAULT} -c ${NRF_BASE}/doc/zephyr)
set(SPHINX_OUTPUT_DIR ${HTML_DIR}/zephyr)
set(KCONFIG_OUTPUT ${HTML_DIR}/kconfig)
set(GEN_DEVICETREE_REST_ZEPHYR_DOCSET " ")

# Get access to the 'html' target from doc/CMakeLists.txt in Zephyr
set(MODULES_EXT_ROOT ${NRF_BASE})
add_subdirectory(${ZEPHYR_BASE}/doc ${ZEPHYR_BINARY_DIR})

add_custom_target(zephyr)
add_dependencies(zephyr html)

add_custom_target(zephyr-html)
add_dependencies(zephyr-html html)

add_custom_target(zephyr-html-all)
add_dependencies(zephyr-html-all html)

set_target_properties(
  zephyr-html zephyr-html-all
  PROPERTIES
    ADDITIONAL_CLEAN_FILES "${ZEPHYR_BINARY_DIR};${SPHINX_OUTPUT_DIR}"
)

#-------------------------------------------------------------------------------
# docset: nrf

set(tools_version_files
    ${NRF_BASE}/scripts/tools-versions-minimum.txt
    ${NRF_BASE}/scripts/tools-versions-darwin.txt
    ${NRF_BASE}/scripts/tools-versions-win10.txt
    ${NRF_BASE}/scripts/tools-versions-linux.txt
)

set(pip_requirements_files
    ${ZEPHYR_BASE}/scripts/requirements-base.txt
    ${ZEPHYR_BASE}/scripts/requirements-doc.txt
    ${MCUBOOT_BASE}/scripts/requirements.txt
    ${NRF_BASE}/scripts/requirements-base.txt
    ${NRF_BASE}/scripts/requirements-doc.txt
    ${NRF_BASE}/scripts/requirements-build.txt
)

string(REPLACE ";" "\\;" tools_files_escaped "${tools_version_files}")
string(REPLACE ";" "\\;" pip_files_escaped "${pip_requirements_files}")

set(NRF_DOC_DIR ${NRF_BASE}/doc/nrf)

add_custom_command(
  OUTPUT ${NRF_BINARY_DIR}/src/versions.txt
  COMMAND ${CMAKE_COMMAND}
    -DTOOLS_VERSION_FILES="${tools_files_escaped}"
    -DPIP_REQUIREMENTS_FILES="${pip_files_escaped}"
    -DVERSION_IN=${NRF_DOC_DIR}/versions.txt.in
    -DVERSION_OUT=${NRF_BINARY_DIR}/src/versions.txt
    -P ${NRF_BASE}/doc/update_versions.cmake
  DEPENDS
    ${NRF_DOC_DIR}/versions.txt.in
    ${tools_version_files}
    ${pip_requirements_files}
)

set(nrf_env
  ZEPHYR_BASE=${ZEPHYR_BASE}
  NRF_BASE=${NRF_BASE}
  NRF_BUILD=${NRF_BINARY_DIR}
  DOXYGEN_EXECUTABLE=${DOXYGEN_EXECUTABLE}
)

add_custom_target(
  nrf-versions
  DEPENDS
    ${NRF_BINARY_DIR}/src/versions.txt
)

add_docset(nrf "${nrf_env}")
add_dependencies(nrf-html nrf-versions)
add_dependencies(nrf-html-all nrf-versions)

#-------------------------------------------------------------------------------
# docset: mcuboot

set(mcuboot_env
  NRF_BASE=${NRF_BASE}
  MCUBOOT_BASE=${MCUBOOT_BASE}
)

add_docset(mcuboot "${mcuboot_env}")

#-------------------------------------------------------------------------------
# docset: nrfxlib

set(nrfxlib_env
  NRF_BASE=${NRF_BASE}
  NRFXLIB_BASE=${NRFXLIB_BASE}
  NRFXLIB_BUILD=${NRFXLIB_BINARY_DIR}
  DOXYGEN_EXECUTABLE=${DOXYGEN_EXECUTABLE}
)

add_docset(nrfxlib "${nrfxlib_env}")

#-------------------------------------------------------------------------------
# docset: kconfig

# The Kconfig documentation is a separate documentation set that's shared
# between all modules (nRF, Zephyr, etc.). This makes it possible to link to
# Kconfig symbols regardless of where they are defined.
#
# We rely on the Zephyr Kconfig files pulling in the other Kconfig files, and
# use the --modules option to gen_kconfig_rest.py to split the documentation
# into a separate page for each module.

if(WIN32)
  set(SEP $<SEMICOLON>)
else()
  set(SEP :)
endif()

set(KCONFIG_RST_OUT ${KCONFIG_BINARY_DIR}/src)
get_directory_property(ZEPHYR_KCONFIG_MODULES DIRECTORY ${ZEPHYR_BASE}/doc DEFINITION ZEPHYR_KCONFIG_MODULES)

# The 'kconfig-content' target uses gen_kconfig_rest.py to generate .rst files
# for all Kconfig symbols, as well as index pages that point to them
add_custom_target(
  kconfig-content
  COMMAND ${CMAKE_COMMAND} -E make_directory ${KCONFIG_RST_OUT}
  COMMAND ${CMAKE_COMMAND} -E env
    PYTHONPATH=${ZEPHYR_BASE}/scripts/kconfig${SEP}$ENV{PYTHONPATH}
    ZEPHYR_BASE=${ZEPHYR_BASE}
    srctree=${ZEPHYR_BASE}
    BOARD_DIR=boards/*/*/
    ARCH=*
    ARCH_DIR=arch/
    SOC_DIR=soc/
    KCONFIG_BINARY_DIR=${KCONFIG_BINARY_DIR}
    KCONFIG_WARN_UNDEF=y
    KCONFIG_TURBO_MODE=${KCONFIG_TURBO_MODE}
    KCONFIG_DOC_MODE=1
    ${ZEPHYR_KCONFIG_MODULES}
      ${PYTHON_EXECUTABLE}
        ${ZEPHYR_BASE}/doc/_scripts/gen_kconfig_rest.py ${KCONFIG_RST_OUT}
        --separate-all-index
        --modules Zephyr,zephyr,${ZEPHYR_BASE}
                  nRF,nrf,${NRF_BASE}
                  nrfxlib,nrfxlib,${NRFXLIB_BASE}
        --no-index-modules BuildDir,${CMAKE_BINARY_DIR}
                           MCUboot,${MCUBOOT_BASE}
  VERBATIM
)

# No 'kconfig' target exists, because it clashes with the imported 'kconfig'
# target from the Zephyr repository

add_custom_target(
  kconfig-html
  COMMAND ${CMAKE_COMMAND} -E env
    ZEPHYR_BASE=${ZEPHYR_BASE}
    ZEPHYR_BUILD=${CMAKE_CURRENT_BINARY_DIR}
      ${SPHINXBUILD}
        -b html
        -c ${NRF_BASE}/doc/kconfig
        -w ${KCONFIG_BINARY_DIR}/sphinx.log
        # The Kconfig reference build doesn't use Breathe, so we can safely
        # parallelize it
        -j auto
        ${SPHINXOPTS_EXTRA}
        ${KCONFIG_RST_OUT}
        ${KCONFIG_OUTPUT}
  USES_TERMINAL
)

add_dependencies(kconfig-html kconfig-content)

add_custom_target(kconfig-html-all)
add_dependencies(kconfig-html-all kconfig-html)

set_target_properties(
  kconfig-html kconfig-html-all
  PROPERTIES
    ADDITIONAL_CLEAN_FILES "${KCONFIG_BINARY_DIR};${KCONFIG_OUTPUT}"
)

#-------------------------------------------------------------------------------
# Global targets

add_custom_target(
  copy-extra-content
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_LIST_DIR}/_static/html/index.html ${HTML_DIR}
  COMMAND ${CMAKE_COMMAND} -E copy ${NRF_BASE}/doc/versions.json ${HTML_DIR}
)

# FIXME: remove once Zephyr CMake is taken out
add_dependencies(html kconfig-html-all)

add_dependencies(zephyr-html-all kconfig-html-all)
add_dependencies(mcuboot-html-all kconfig-html-all)
add_dependencies(nrfxlib-inventory-all kconfig-html-all)
add_dependencies(nrf-html-all nrfxlib-inventory-all kconfig-html-all)
add_dependencies(nrfxlib-html-all nrf-html-all)

add_custom_target(build-all ALL)
add_dependencies(build-all
    copy-extra-content
    nrf-html-all
    nrfxlib-html-all
    zephyr-html-all
    mcuboot-html-all
    kconfig-html-all
)

add_custom_target(linkcheck)
add_dependencies(linkcheck
    nrf-linkcheck
    nrfxlib-linkcheck
    mcuboot-linkcheck
)
