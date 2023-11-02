#!/bin/sh

input_directory="inputs"
output_directory="outputs"

for file in "${input_directory}"/*.in; do
    if [ -f "${file}" ]; then
        input="${file}"
        output="${input%.in}.out"
        output="${output_directory}"/"${output#*/}"
        ./bin/runner < "${input}" > "${output}"
    fi
done
