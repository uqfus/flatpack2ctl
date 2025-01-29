idf_component_register(SRCS "app_main.c" "fp2charger.c"
                    INCLUDE_DIRS "."
                    EMBED_FILES "test-favicon.ico")

add_custom_command(
    OUTPUT 
        non-existing.txt # force to execute on every build (https://gitlab.kitware.com/cmake/cmake/-/issues/17071)
        "${COMPONENT_DIR}/index.html.gz"
    WORKING_DIRECTORY ${COMPONENT_DIR}
    COMMAND "C:\\Program Files\\7-Zip\\7z.exe" a -tgzip -mx9 index.html.gz index.html
    VERBATIM
)

target_add_binary_data( ${COMPONENT_LIB} "index.html.gz" BINARY DEPENDS "${COMPONENT_DIR}/index.html.gz" )
