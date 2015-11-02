#!/bin/bash

[[ $# -eq 1 ]] || { echo "Provide a revision sha or other revision id (see 'man gitrevisions' for details)"; exit 1; }

git --no-pager log "$1"..HEAD | grep -v -E "commit|Author|Date|^$" | sed -r -e 's/^ +/* /g'
