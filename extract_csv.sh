#!/bin/bash

help()
{
	echo "usage: extract_csv.sh [-h] [-t|-l] output.csv"
	echo "-h, --help: print this help message"
	echo "-t, --tag:  additional tag to filter csv rows"
	echo "-l, --log:  logfile location; default is ./Player.log"
	exit 0
}

tag="tag:"
log=Player.log

while [[ $# > 0 ]]
do
	arg=$1
	case $arg in
	-h|--help)
		help
		;;
	-t|--tag)
		tag="$tag $2"
		shift
		;;
	-l|--log)
		log=$2
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
[[ -d $(dirname "$out") ]] || { echo "No such directory: $(dirname "$out")"; exit 1; }

echo "looking for '$tag' in $log" 
echo "output: $out"

grep "$tag" "$log" | cut -f 2- -d ',' > "$out"
