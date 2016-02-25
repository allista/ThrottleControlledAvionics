'''
Created on Feb 24, 2016

@author: Allis Tauri <allista@gmail.com>
'''

import os

def datapath(*paths): return os.path.join(GAMEDATA, *paths)

KSPDIR = '/home/storage/Games/KSP_linux/PluginsArchives/Development/KSP-test/KSP_test_1.0.5'
GAMEDATA = os.path.join(KSPDIR, 'GameData')
SQUAD = datapath('Squad')