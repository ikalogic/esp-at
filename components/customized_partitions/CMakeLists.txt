register_component()

idf_build_get_property(build_dir BUILD_DIR)
idf_build_get_property(project_dir PROJECT_DIR)

set(flash_customized_partition_args ${build_dir}/flash_customized_partition_args)

STRING(REGEX REPLACE ".*/(.*)\.csv" "\\1" customize_bin ${CONFIG_AT_CUSTOMIZED_PARTITION_TABLE_FILE})
set(customize_bin_full_name ${customize_bin}.bin)
execute_process (
    COMMAND ${PYTHON} ${IDF_PATH}/components/partition_table/gen_esp32part.py 
        ${project_dir}/${CONFIG_AT_CUSTOMIZED_PARTITION_TABLE_FILE} ${build_dir}/${customize_bin_full_name}
)
esptool_py_flash_project_args(${customize_bin} ${CONFIG_AT_CUSTOMIZED_PARTITION_TABLE_OFFSET} ${build_dir}/${customize_bin_full_name} FLASH_IN_PROJECT)

execute_process (
    COMMAND ${PYTHON} ${COMPONENT_DIR}/raw_data/raw_data_bin_gen.py
        --sdkconfig_file ${project_dir}/sdkconfig
        --dependency_file ${COMPONENT_DIR}/AT_partition_dependency
        --partition_file ${project_dir}/${CONFIG_AT_CUSTOMIZED_PARTITION_TABLE_FILE}
        --tools_dir ${COMPONENT_DIR}/generation_tools
        --output_dir ${build_dir}/${COMPONENT_NAME}
        --flash_args_file ${flash_customized_partition_args}
)

file(STRINGS ${flash_customized_partition_args} customized_partition_args)

foreach(partition_args ${customized_partition_args})
    string(STRIP ${partition_args} args_value)
    string(REPLACE " " ";" args_lists ${args_value})
    list(GET args_lists 0 offset)
    list(GET args_lists 1 file_path)
    
    STRING(REGEX REPLACE ".*/(.*)\.bin" "\\1" file_name ${file_path})

    esptool_py_flash_project_args(${file_name} ${offset} ${file_path} FLASH_IN_PROJECT)
endforeach(partition_args)
