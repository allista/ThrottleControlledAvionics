#1/bin/bash

git daemon --export-all --base-path=../ --enable=receive-pack --timeout=600 &
