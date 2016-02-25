#!/usr/bin/python2.7
# encoding: utf-8
'''
Created on Nov 26, 2015

@author: Allis Tauri
'''

import sys, os, re

header = '''//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

'''

if __name__ == '__main__':
    skipped = re.compile(r'^(\xEF\xBB\xBF)*(//)+.*|\s*$')
    for dirpath, dirs, files in os.walk("../"):
        for f in files:
            if not f.endswith('.cs'): continue
            filename = os.path.join(dirpath, f)
            with open(filename, 'r') as cs:
                #read a file and change the header in memory
                first_line = cs.readline()
                if first_line.startswith("/*"): continue
                cs.seek(0)
                output = header
                old_skipped = False
                for l in cs:
                    if not old_skipped and skipped.match(l): continue
                    old_skipped = True
                    output += l
            #write back to the file
            with open(filename, 'w') as cs:
                print '='*90
                print output
                cs.write(output)
