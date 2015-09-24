#!/bin/bash

tag="tag:"
while [[ $# > 0 ]]
do
	arg=$1
	case $arg in
	-t|--tag)
		tag="$tag $2"
		shift
		;;
	*)
		out=$1
		break
		;;
	esac
	shift
done
[[ -n "$out" ]] || out='output.csv'

echo "looking for: '$tag'" 
echo "out: $out"

grep "$tag" Player.log | cut -f 3- -d ' ' > "$out"
