#!/bin/bash

out="$1"
[[ -n "$out" ]] || out='output.csv'
grep 'tag:' Player.log | cut -f 3- -d ' ' > "$1"
