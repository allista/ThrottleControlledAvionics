#!/bin/bash

cd $(dirname "$0")

./make_release -e '*/config.xml' '*/TCA.conf' '*/TCA.macro' '*/ThrottleControlledAvionics.user' '*.orig'
