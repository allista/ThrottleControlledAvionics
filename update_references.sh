#!/bin/bash

cd $(dirname "$0")
pwd

[[ -n "$1" ]] || { echo "You should provide the path to the new KSP version"; exit 1; }

nKSPv=$(echo "$1" | sed -E 's/\//\\/g')
KSPv=$(grep System.dll ThrottleControlledAvionics.csproj | sed -E 's/(<\/?HintPath>)|(\s+)|(KSP_Data\\Managed\\System.dll)//g')
echo "Currently configured KSP version is: $KSPv"
echo "Changing to: $nKSPv"

[[ "$KSPv" == "$1" ]] && { echo "No need to change"; exit 0; }

escape_regexp()
{ 
	echo $(echo "$1" | sed -E 's/\\/\\\\/g' | sed -E 's/\./\\./g') 
}

nKSPv=$(escape_regexp $nKSPv)
KSPv=$(escape_regexp $KSPv)

files="$(find ./ -name "*.csproj")"
[[ -n "$files" ]] || { echo "No .csproj files was found"; exit; }

for f in $files; do
	before=$(md5sum "$f")
	#change the version
	perl -i -0pe "s/$KSPv/$nKSPv/g" "$f"
	#check the result
	after=$(md5sum "$f")
	[[ "$before" == "$after" ]] || { echo "Changed references to KSP libraries in $f:"; echo $before; echo $after; }
done
