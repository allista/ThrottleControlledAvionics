#!/bin/bash

cd $(dirname "$0")

../../PyKSPutils/make_mod_release \
-e '*/config.xml' '*/TCA.conf' '*/TCA.macro' '*.user' '*.orig' '*.mdb' '*.pdb' \
'*/System.*.dll' '*/Mono.*.dll' '*/Unity*.dll' \
'*/001_AnisotropicPartResizer*' \
'*/002_MultiAnimators.dll' \
'*/AnimatedConverters.dll' \
'*/ConfigurableContainers.dll' \
'*/SubmodelResizer.dll' \
'GameData/000_AT_Utils/ResourceHack.cfg' \
-i '../AT_Utils/GameData'
