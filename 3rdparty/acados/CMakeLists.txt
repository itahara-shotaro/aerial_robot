cmake_minimum_required(VERSION 2.8.3)
project(acados)
include(ExternalProject)

# must match TERA_VERSION in <acados>/interfaces/acados_template/acados_template/utils.py
set(TERA_RENDERER_VERSION 0.0.34)

ExternalProject_Add(
  acados
  GIT_REPOSITORY https://github.com/acados/acados.git
  GIT_TAG v0.5.0
  CMAKE_ARGS -DACADOS_WITH_QPOASES=ON  -DACADOS_INSTALL_DIR=${CMAKE_INSTALL_PREFIX}
  LOG_DOWNLOAD ON
  LOG_CONFIGURE ON
  LOG_BUILD ON
  LOG_INSTALL ON
  )

# acados_template (and rh_base.py) look for the libraries and headers under
# $ACADOS_SOURCE_DIR, i.e. the source dir, not under CMAKE_INSTALL_PREFIX.
# So install a second time with the source dir as the prefix.
ExternalProject_Add_Step(acados install_in_source_dir
  COMMAND ${CMAKE_COMMAND} --install <BINARY_DIR> --prefix <SOURCE_DIR>
  COMMENT "Installing acados into <SOURCE_DIR> (= ACADOS_SOURCE_DIR)"
  DEPENDEES install
  )

# acados ships no tera renderer; acados_template would prompt for its download on
# first use, which fails with EOFError during a non-interactive build. Fetch the
# binary here instead, following the manual instructions in get_tera().
set(TERA_RENDERER_FILE ${CMAKE_CURRENT_BINARY_DIR}/t_renderer)
if(NOT EXISTS ${TERA_RENDERER_FILE})
  set(TERA_RENDERER_URL "https://github.com/acados/tera_renderer/releases/download/v${TERA_RENDERER_VERSION}/t_renderer-v${TERA_RENDERER_VERSION}-linux")
  message(STATUS "Downloading tera renderer from ${TERA_RENDERER_URL}")
  file(DOWNLOAD ${TERA_RENDERER_URL} ${TERA_RENDERER_FILE} STATUS TERA_RENDERER_STATUS)
  list(GET TERA_RENDERER_STATUS 0 TERA_RENDERER_ERROR)
  if(TERA_RENDERER_ERROR)
    file(REMOVE ${TERA_RENDERER_FILE})
    message(FATAL_ERROR "Failed to download tera renderer: ${TERA_RENDERER_STATUS}")
  endif()
endif()
file(CHMOD ${TERA_RENDERER_FILE} PERMISSIONS
  OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

ExternalProject_Add_Step(acados tera_renderer
  COMMAND ${CMAKE_COMMAND} -E make_directory <SOURCE_DIR>/bin
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${TERA_RENDERER_FILE} <SOURCE_DIR>/bin/t_renderer
  COMMENT "Installing tera renderer into <SOURCE_DIR>/bin"
  DEPENDEES install
  )

## generate python interface
#ExternalProject_Add_Step(acados pip_install
#        COMMAND pip install -e <BINARY_DIR>/../acados/interfaces/acados_template
#        DEPENDEES install
#)
#
#
## add path to acados to .bashrc
#execute_process(
#        COMMAND grep -q "ACADOS_SOURCE_DIR" $ENV{HOME}/.bashrc
#        RESULT_VARIABLE result_bashrc
#)
#if(${result_bashrc})
#    ExternalProject_Add_Step(acados export_paths
#            COMMAND echo "" >> $ENV{HOME}/.bashrc
#            COMMAND echo "# >>>>> Automatically adding ACADOS_SOURCE_DIR to .bashrc" >> $ENV{HOME}/.bashrc
#            COMMAND echo "export ACADOS_SOURCE_DIR=<INSTALL_DIR>/src/acados" >> $ENV{HOME}/.bashrc
#            COMMAND echo "" >> $ENV{HOME}/.bashrc
#            DEPENDEES pip_install
#    )
#    message("ACADOS_SOURCE_DIR not found in .bashrc, automatically adding it.")
#endif()
#
## add path to acados to .zshrc
#execute_process(
#        COMMAND grep -q "ACADOS_SOURCE_DIR" $ENV{HOME}/.zshrc
#        RESULT_VARIABLE result_zshrc
#)
#if(${result_zshrc})
#    ExternalProject_Add_Step(acados export_paths_zsh
#            COMMAND echo "" >> $ENV{HOME}/.bashrc
#            COMMAND echo "# >>>>> Automatically adding ACADOS_SOURCE_DIR to .zshrc" >> $ENV{HOME}/.zshrc
#            COMMAND echo "export ACADOS_SOURCE_DIR=<INSTALL_DIR>/src/acados" >> $ENV{HOME}/.zshrc
#            COMMAND echo "" >> $ENV{HOME}/.zshrc
#            DEPENDEES pip_install
#    )
#    message("ACADOS_SOURCE_DIR not found in .zshrc, automatically adding it.")
#endif()


install(CODE "execute_process(COMMAND make install WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/acados-prefix/src/acados-build)")
