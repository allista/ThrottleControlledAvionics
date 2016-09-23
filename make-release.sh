#!/bin/bash

cd $(dirname "$0")

../../PyKSPutils/make_mod_release \
-e '*/config.xml' '*/TCA.conf' '*/TCA.macro' '*.user' '*.orig' '*.mdb' \
'*/001_AnisotropicPartResizer*' \
'*/002_MultiAnimators.dll' \
'*/AnimatedConverters.dll' \
'*/ConfigurableContainers.dll' \
-i '../AT_Utils/GameData'

