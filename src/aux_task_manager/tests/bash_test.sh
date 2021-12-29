#!/bin/bash

PY_FILE=$1
echo "python3 ${PY_FILE}"
python3 $PY_FILE >&1

# function proc_start {
#     PY_FILE=$1
#     echo "python3 ${PY_FILE}"
#     python3 $PY_FILE

#     # while true
#     # do
#     #     sleep 10
#     # done
# }

# function proc_exit {
#     exit 0
# }


# trap proc_exit TERM INT
# proc_start

