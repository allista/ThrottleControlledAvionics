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

'''
Created on Feb 22, 2016

@author: Allis Tauri <allista@gmail.com>
'''

import os, csv
import networkx as nx
from openpyxl import load_workbook

from config import datapath
from PyConfigNode import ConfigNode

def all_successors(node, G):
    nbunch = set([node])
    for s in G.successors_iter(node):
        nbunch.add(s)
        nbunch.update(all_successors(s, G))
    return nbunch

def draw_graph(graph, name, prog='dot'):
    ag = nx.nx_agraph.to_agraph(graph)
    ag.layout(prog='dot')
    filename = name+'.png' 
    ag.draw(filename)
    print 'Saved %s' % filename
    
def draw_node_deps(n, G):
    deps = G.subgraph(all_successors(n, G))
    draw_graph(deps, n)
    
def draw_all_deps(G, outdir, allname='AllNodes'):
    if not os.path.isdir(outdir):
        os.mkdir(outdir)
    os.chdir(outdir)
    for n in G.nodes_iter():
        draw_node_deps(n, G)
    draw_graph(G, allname)
    os.chdir('..')

class TechTreeUpdater(object):
    part_template = ''
    
    def __init__(self, modfile, partfile):
        self._modfile = modfile
        self._partfile = partfile
        self.modules = None
        self.parts = None
        self._parse_modules_db()
        self._parse_part_defs()
        self._get_part_template()
        
    def _parse_modules_db(self):
        self.modules = nx.DiGraph()
        self.parts = nx.DiGraph()
        with open(self._modfile) as inp:
            reader = csv.reader(inp)
            for row in reader:
                part = row[0]
                module = row[1]
                self.modules.add_node(module,
                                      shape='box',
                                      style='rounded')
                if part in self.parts:
                    self.parts.node[part]['modules'].add(module)
                else:
                    self.parts.add_node(part,
                                        shape='box',
                                        style='rounded,filled', 
                                        fillcolor='#ddffdd',
                                        modules=set([module]))
                optional = False
                for dep in row[2:]:
                    if not dep: continue
                    if dep == '|':
                        optional = True
                        continue
                    self.modules.add_edge(module, dep,
                                          optional = optional,
                                          style = 'dashed' if optional else 'solid',
                                          color = 'darkgreen' if optional else 'blue')
    
    def _part_modules(self, part):
        return self.parts.node[part].get('modules', set())
    
    def _parse_part_defs(self):
        #load the table
        wb = load_workbook(self._partfile)
        ws = wb.active
        h  = ws.rows[0]
        #parse part definitions
        for row in ws.rows[1:]:
            part = {}
            for i, attr in enumerate(h):
                part[attr.value] = row[i].value or ''
            if not part['name']: continue
            if part['name'] in self.parts:
                self.parts.node[part['name']]['data'] = part
                self.parts.node[part['name']]['label'] = part['title']
            else: self.parts.add_node(part['name'], label=part['title'], 
                                      modules=set(), data=part)
        #add dependency information
        for p in self.parts:
            if 'data' not in self.parts.node[p]:
                raise RuntimeError("%s has no definition in the Parts table" % p)
            modules = self.parts.node[p]['modules']
            for m in modules:
                deps = [d for d in self.modules.successors_iter(m) 
                        if not self.modules.edge[m][d]['optional']]
                for dep in deps:
                    for p1 in self.parts:
                        if p1 == p: continue
                        if dep in self._part_modules(p1):
                            self.parts.add_edge(p, p1)
        self._remove_redundant_edges(self.parts)
                
    @staticmethod
    def _remove_redundant_edges(G):
        for p in G:
            for p1 in G:
                if p == p1: continue
                if p1 not in G.successors(p): continue
                paths = list(nx.all_simple_paths(G, p, p1))
                if len(paths) < 2: continue
                G.remove_edge(p, p1)
        
    @classmethod
    def _get_part_template(cls):
        if not cls.part_template:
            with open(datafile(templatefile)) as inp:
                cls.part_template = inp.read()

    def draw_module_dependencies(self, outdir):
        if not os.path.isdir(outdir) or \
        os.stat(outdir).st_mtime < os.stat(self._modfile).st_mtime:
            draw_all_deps(self.modules, outdir, 'AllModules')
            
    def draw_part_dependencies(self, outdir):
        do_draw = not os.path.isdir(outdir)
        if not do_draw:
            mtime = os.stat(outdir).st_mtime
            do_draw = mtime < os.stat(self._modfile).st_mtime or mtime < os.stat(self._partfile).st_mtime
        if do_draw: draw_all_deps(self.parts, outdir, 'AllParts')
    
    _module_desc = 'Upgrades TCA (tm) with the {title}.'
    def _make_part(self, part):
        data = dict(self.parts.node[part]['data'].items())
        if self.parts.node[part]['modules']:
            add_description = data['description']
            data['description'] = self._module_desc.format(**data)
            if add_description: data['description'] += ' '+add_description 
        dep_str = ', '.join(self.parts.node[dep]['data']['title']
                          for dep in self.parts.successors_iter(part))
        if dep_str: data['description'] += ' Requires: %s.' % dep_str
        return self.part_template.format(name=data.pop('name'), 
                                         title=data.pop('title'), 
                                         description=data.pop('description'),
                                         cost=int(data.pop('cost', 10000)),
                                         model=data.pop('model', 'Squad/Parts/Command/probeCoreOcto2/model'),
                                         node=data.pop('node', 'specializedControl'))


    def write_tree_parts(self, filename):
        with open(filename, 'w') as out:
            ttree = ''.join(self._make_part(p) for p in self.parts)
            out.write(ttree)
            
    def annotate_tree(self, filename):
        treenode = ConfigNode.Load(techtree)
        #build tree graph
        tree = nx.DiGraph()
        treedict = {}
        for n in treenode.subnodes:
            name = n['id']
            treedict[name] = n
            parts = [p for p, attrs in self.parts.nodes_iter(data=True)
                     if attrs['data']['node'] == name]
            tree.add_node(name, 
                          shape='box',
                          style='rounded'+(',filled' if parts else ''),
                          fillcolor='#ddffdd' if parts else '',
                          label='<<b>%s</b>:<br/>%s>' % (n['title'], '<br/>'.join(self.parts.node[p]['data']['title'] for p in parts)) if parts else n['title'],
                          parts=parts)
            for p in n.GetNodes('Parent'): 
                tree.add_edge(p['parentID'], name)
        #dump tree nodes
        with open('TechTreeNodes.csv', 'w') as out:
            writer = csv.writer(out)
            for n in nx.topological_sort(tree):
                node = treedict[n]
                writer.writerow([node['id'], node['title'], node['description']])
        #draw the graph
        draw_graph(tree, 'TechTreeGraph')
        #add part dependencies
        for n in tree:
            for p in tree.node[n].get('parts', []):
                for d in self.parts.successors(p):
                    for n1 in tree:
                        found = False
                        for p1 in tree.node[n1].get('parts', []):
                            if p1 == d and not tree.has_edge(n, n1):
#                                 if not nx.has_path(tree, n1, n):
                                tree.add_edge(n, n1, 
                                              color='red',
                                              constraint='false')
                                found = True
                                break
                        if found: break
        draw_graph(tree, 'TechTreeGraph_PartDeps')
        

datadir       = 'data'
modulesfile   = 'ModuleDatabase.csv'
partsfile     = 'Parts.xlsx'
modulesdir    = 'ModuleDependencies'
partsdir      = 'PartDependencies'
templatefile  = 'DummyPartTemplate.cfg'
techtreeparts = datapath('ThrottleControlledAvionics', 'TCATechTree.cfg')
techtree      = datapath('Squad', 'Resources', 'TechTree.cfg')

datafile = lambda f: os.path.join(datadir, f)

if __name__ == '__main__':
    updater = TechTreeUpdater(datafile(modulesfile),
                              datafile(partsfile))
    updater.draw_module_dependencies(modulesdir)
    updater.draw_part_dependencies(partsdir)
    updater.write_tree_parts(techtreeparts)
    updater.annotate_tree(techtree)
    print 'Done'