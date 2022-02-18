#!/bin/bash

cd $(dirname "$0")

make_mod_release \
-e '*/config.xml' '*/TCA.conf' '*/TCA.macro' '*.user' '*.orig' \
'*/001_AnisotropicPartResizer*' \
'*/002_MultiAnimators.dll' \
'*/AnimatedConverters.dll' \
'*/ConfigurableContainers.dll' \
'GameData/000_AT_Utils/ResourceHack.cfg' \
-i ../AT_Utils/GameData \
-o ~/Dropbox/TCA_Beta

