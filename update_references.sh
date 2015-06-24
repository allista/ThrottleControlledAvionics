#!/bin/bash

cd $(dirname "$0")
pwd

[[ -n "$1" ]] || { echo "You should provide the path to the new KSP version"; exit 1; }

KSPv=$(grep System.dll ThrottleControlledAvionics.csproj | cut -d '\' -f3)
echo "Currently configured KSP version is: $KSPv"

[[ "$KSPv" == "$1" ]] && { echo "No need to change"; exit 0; }

files="$(find ./ -name "*.csproj")"
[[ -n "$files" ]] || { echo "No .csproj files was found"; exit; }

for f in $files; do
	before=$(md5sum "$f")
	#change the version
	perl -i -0pe "s/$KSPv/$1/g" "$f"
	#check the result
	after=$(md5sum "$f")
	[[ "$before" == "$after" ]] || { echo "Changed references to KSP libraries in $f:"; echo $before; echo $after; }
done
