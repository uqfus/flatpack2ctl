idf_component_register(SRCS "app_main.c" "fp2charger.c"
                    INCLUDE_DIRS "."
                    EMBED_FILES "test-favicon.ico")

add_custom_command(
    OUTPUT 
        non-existing.txt # force to execute on every build (https://gitlab.kitware.com/cmake/cmake/-/issues/17071)
        "${COMPONENT_DIR}/index.html.gz"
    WORKING_DIRECTORY ${COMPONENT_DIR}
    COMMAND python -c "import gzip,shutil;f_in=open('index.html','rb');f_out=gzip.open('index.html.gz','wb');shutil.copyfileobj(f_in,f_out)"
    VERBATIM
)

target_add_binary_data( ${COMPONENT_LIB} "index.html.gz" BINARY DEPENDS "${COMPONENT_DIR}/index.html.gz" )
