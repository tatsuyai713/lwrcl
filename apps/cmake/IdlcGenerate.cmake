# cmake/IdlcGenerate.cmake
# Usage:
#   idlc_generate(
#     NAME <unique_name>                 # ターゲット名の接頭辞（ユニークに）
#     ALIAS <include_alias>              # 生成物の #include に使う別名（例: ROSCustomTypeImagePubSub）
#     IDL_ROOT <path/to/msg>             # *.idl が直下にあるディレクトリ
#     IDLC <path/to/idlc>                # idlc 実行ファイル
#     UPDATE_HEADERS_SCRIPT <path.sh>    # update_headers.sh のパス
#   )
#
# Exports to parent scope:
#   <NAME>_FINAL_SRCS
#   <NAME>_GEN_PKG_DIR
#   <NAME>_GEN_MSG_DIR
#   <NAME>_IDL_FILES

function(idlc_generate)
  cmake_parse_arguments(IDLCG "" "NAME;ALIAS;IDL_ROOT;IDLC;UPDATE_HEADERS_SCRIPT" "" ${ARGN})

  foreach(req NAME IDL_ROOT IDLC UPDATE_HEADERS_SCRIPT)
    if(NOT IDLCG_${req})
      message(FATAL_ERROR "[idlc_generate] Missing required argument: ${req}")
    endif()
  endforeach()

  if(NOT IDLCG_ALIAS)
    set(IDLCG_ALIAS "${IDLCG_NAME}")
  endif()

  # 生成先ディレクトリ
  set(GEN_PKG_DIR "${CMAKE_BINARY_DIR}/generated/${IDLCG_ALIAS}")
  set(GEN_MSG_DIR "${GEN_PKG_DIR}/msg")

  # *.idl を直下から列挙
  file(GLOB IDL_FILES "${IDLCG_IDL_ROOT}/*.idl")
  if(IDL_FILES STREQUAL "")
    message(FATAL_ERROR "[idlc_generate] No IDL files in ${IDLCG_IDL_ROOT}. Place *.idl directly under it.")
  endif()

  # 作業ディレクトリ作成
  add_custom_target(${IDLCG_NAME}_gen_msg_dir
    COMMAND ${CMAKE_COMMAND} -E make_directory "${GEN_MSG_DIR}"
    COMMENT "[${IDLCG_NAME}] Create ${GEN_MSG_DIR}"
    VERBATIM
  )

  # IDL -> C++ 生成
  set(GENERATED_SOURCES)
  set(GENERATED_HEADERS)

  foreach(IDL_FILE IN LISTS IDL_FILES)
    get_filename_component(IDL_BASENAME "${IDL_FILE}" NAME_WE)
    set(GEN_CPP "${GEN_MSG_DIR}/${IDL_BASENAME}.cpp")
    set(GEN_HPP "${GEN_MSG_DIR}/${IDL_BASENAME}.hpp")
    
    add_custom_command(
      OUTPUT "${GEN_CPP}"
      WORKING_DIRECTORY "${GEN_MSG_DIR}"
      COMMAND "${IDLCG_IDLC}"
              -l cxx
              -fcase-sensitive
              -I "${IDLCG_IDL_ROOT}"
              -I "${CMAKE_CURRENT_SOURCE_DIR}"
              -I "/opt/cyclonedds-libs/include"
              "${IDL_FILE}"
      # 生成ヘッダの最小パッチ（return "pkg::msg::Type"; → return "pkg::msg::dds_::Type_";）
      COMMAND ${CMAKE_COMMAND} -E env bash -c
              "sed -E -i 's/(return[[:space:]]*\")([[:alnum:]_]+::msg::)([[:alnum:]_]+)(\";)/\\1\\2dds_::\\3_\\4/' \"${GEN_HPP}\""
      DEPENDS ${IDLCG_NAME}_gen_msg_dir "${IDL_FILE}"
      BYPRODUCTS "${GEN_HPP}"
      COMMENT "[${IDLCG_NAME}] idlc: ${IDL_BASENAME} -> ${GEN_MSG_DIR}"
      VERBATIM
    )

    list(APPEND GENERATED_SOURCES "${GEN_CPP}")
    list(APPEND GENERATED_HEADERS "${GEN_HPP}")
  endforeach()

  add_custom_target(${IDLCG_NAME}_idl_gen ALL DEPENDS ${GENERATED_SOURCES})
  add_dependencies(${IDLCG_NAME}_idl_gen ${IDLCG_NAME}_gen_msg_dir)

  # update_headers.sh 実行
  add_custom_target(${IDLCG_NAME}_update_headers
    COMMAND ${CMAKE_COMMAND} -E echo "[${IDLCG_NAME}] Running update_headers.sh in ${GEN_PKG_DIR}"
    COMMAND ${CMAKE_COMMAND} -E env bash "${IDLCG_UPDATE_HEADERS_SCRIPT}"
    WORKING_DIRECTORY "${GEN_PKG_DIR}"
    DEPENDS ${IDLCG_NAME}_idl_gen
    COMMENT "[${IDLCG_NAME}] Updating headers & cpp includes"
    VERBATIM
  )

  # _final.cpp 作成（update 後にコピー）
  set(FINAL_SRCS)
  foreach(SRC IN LISTS GENERATED_SOURCES)
    get_filename_component(SRC_DIR  "${SRC}" DIRECTORY)
    get_filename_component(SRC_NAME "${SRC}" NAME_WE)
    set(FINAL_SRC "${SRC_DIR}/${SRC_NAME}_final.cpp")

    add_custom_command(
      OUTPUT "${FINAL_SRC}"
      COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SRC}" "${FINAL_SRC}"
      DEPENDS ${IDLCG_NAME}_update_headers "${SRC}"
      COMMENT "[${IDLCG_NAME}] Create final: ${SRC_NAME}_final.cpp"
      VERBATIM
    )
    list(APPEND FINAL_SRCS "${FINAL_SRC}")
  endforeach()

  add_custom_target(${IDLCG_NAME}_final_sources DEPENDS ${FINAL_SRCS})

  # 親スコープへエクスポート
  set(${IDLCG_NAME}_FINAL_SRCS "${FINAL_SRCS}" PARENT_SCOPE)
  set(${IDLCG_NAME}_GEN_PKG_DIR "${GEN_PKG_DIR}" PARENT_SCOPE)
  set(${IDLCG_NAME}_GEN_MSG_DIR "${GEN_MSG_DIR}" PARENT_SCOPE)
  set(${IDLCG_NAME}_IDL_FILES  "${IDL_FILES}"  PARENT_SCOPE)
endfunction()
