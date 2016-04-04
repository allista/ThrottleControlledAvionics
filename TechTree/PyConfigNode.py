# Copyright (c) 2016 Allis Tauri
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy 
# of this software and associated documentation files (the "Software"), to deal 
# in the Software without restriction, including without limitation the rights 
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
# copies of the Software, and to permit persons to whom the Software is 
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in 
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
# THE SOFTWARE.

"""
Created on Feb 23, 2016

@author: Allis Tauri <allista@gmail.com>
"""


class ConfigNode(object):
    """
    Simple KSP ConfigNode reader/writer
    """
    
    class Value(object):
        def __init__(self, name, value):
            self.name = name
            self.value = value

        def __str__(self):
            return '%s = %s' % (self.name, self.value)

    def __init__(self, name=""):
        self.name = name
        self.values   = []
        self.subnodes = []
        self._vindex  = {}
        self._nindex  = {}

    def Clone(self, other):
        self.name = other.name
        self.values   = other.values
        self.subnodes = other.subnodes
        self._vindex  = other._vindex
        self._nindex  = other._nindex
        
    def __getitem__(self, key):
        if isinstance(key, int):
            return self.values[key].value
        elif isinstance(key, str):
            return self.values[self._vindex[key][0]].value
    
    def __len__(self): return len(self.values)
    
    def __nonzero__(self): return bool(self.values) or bool(self.subnodes)
    
    @staticmethod
    def _register_index(db, key, index):
        lst = db.get(key, None)
        if lst: lst.append(index)
        else: db[key] = [index]
    
    def AddNode(self, node):
        if isinstance(node, str):
            new_node = ConfigNode(node)
        elif isinstance(node, ConfigNode):
            new_node = node
        else: raise ValueError("node should be either a string or ConfigNode object")
        self.subnodes.append(new_node)
        self._register_index(self._nindex, new_node.name, len(self.subnodes)-1)
        return new_node
    
    def AddValue(self, name, value):
        self.values.append(self.Value(name, value))
        self._register_index(self._vindex, name, len(self.values)-1)
        
    def GetNode(self, name):
        idx = self._nindex.get(name, [])
        return self.subnodes[idx[0]] if idx else None
    
    def GetNodes(self, name):
        idx = self._nindex.get(name, [])
        return [self.subnodes[i] for i in idx]
    
    def GetValue(self, name):
        idx = self._vindex.get(name, [])
        return self.values[idx[0]].value if idx else None
    
    def GetValues(self, name):
        idx = self._vindex.get(name, [])
        return [self.values[i].value for i in idx]
        
    def Parse(self, text):
        self.values = []
        self.subnodes = []
        lines = self._preformat(text.splitlines())
        self._parse(lines, self)
        if len(self.values) == 0 and len(self.subnodes) == 1:
            self.Clone(self.subnodes[0])
            
    @classmethod
    def Load(cls, filename):
        node = cls()
        with open(filename) as inp:
            node.Parse(inp.read())
        return node
    
    def Save(self, filename):
        with open(filename, 'w') as out:
            out.write(str(self))
    
    @classmethod
    def _parse(cls, lines, node, index=0):
        nlines = len(lines)
        while index < nlines:
            line = lines[index]
            if len(line) == 2:
                node.AddValue(*line)
                index += 1
            elif line[0] == '{':
                subnode = node.AddNode("")
                index = cls._parse(lines, subnode, index+1)
            else:
                if line[0] == '}': return index+1
                if index < nlines-1 and lines[index+1][0] == '{':
                    subnode = node.AddNode(line[0])
                    index = cls._parse(lines, subnode, index+2)
                else: index += 1
        return index
    
    @staticmethod
    def _split_by(sym, l, lines):
        line = lines[l]
        try:
            idx = line.index(sym)
            if idx == 0 and len(line) == 1: return l
            if idx > 0:
                lines.insert(l, line[:idx])
                line = line[idx:]
                l += 1; idx = 0
                lines[l] = line
            if idx < len(line)-1:
                lines.insert(l+1, line[1:])
                lines[l] = sym
                l += 2
        except ValueError: pass
        return l
        
    def _preformat(self, lines):
        l = len(lines)
        while l >= 0:
            l -= 1
            line = lines[l]
            try:
                idx = line.index('//')
                if idx == 0: 
                    del lines[l]
                    continue
                else: lines[l] = line[:idx]
            except ValueError: pass
            line = line.strip()
            if not line: 
                del lines[l]
                continue
            lines[l] = line
            l = self._split_by('}', l, lines)
            l = self._split_by('{', l, lines)
        return [[w.strip() for w in line.split('=')] for line in lines]
        
    def __str__(self):
        s = '%s\n{\n' % self.name
        v = '\n'.join('    %s' % v for v in self.values)
        n = '\n'.join('    %s' % l for n in self.subnodes
                      for l in str(n).splitlines())
        if v and n: v += '\n'  
        return s+v+n+'\n}'
    
#tests
if __name__ == '__main__':
    #create
    n = ConfigNode('test')
    n.AddValue('a', 1)
    n.AddValue('b', 2)
    n1 = n.AddNode('sub')
    n1.AddValue('c', 3)
    n1.AddValue('d', 4)
    print n
    print
    #parse
    n2 = ConfigNode('parsed')
    n2.Parse(str(n)+str(n))
    print n2
    print
    #get
    print n['a']
    print n.GetValue('b')
    print n.GetNode('sub')
