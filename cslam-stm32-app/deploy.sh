#!/usr/bin/sh

set -e # The script shall fail if there is an error.

# script_dir="$(dirname "$(readlink -f "$0")")"
# cd "${script_dir}"

addr0="radio://0/100/2M/E7E7E7E7EA"
addr1="radio://0/100/2M/E7E7E7E7EB"
addr2="radio://0/100/2M/E7E7E7E7EC"
addr3="radio://0/100/2M/E7E7E7E7ED"

make all
make cload CLOAD_CMDS="-w ${addr0}"  

make all
make cload CLOAD_CMDS="-w ${addr1}"

make all
make cload CLOAD_CMDS="-w ${addr2}"

make all
make cload CLOAD_CMDS="-w ${addr3}"
