#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import datetime
import os
from StringIO import StringIO
from utils import UniqueStringGenerator


class Graph(object):
    def __init__(self):
        self.s = StringIO()
        self.num = 1
        self.table = {}
    def node(self, name, label=None):
        self.table[name] = str(self.num)
        self.num += 1
        self.s.write("  %s" % self.table[name])
        if label is None:
            label = name
        label = label.replace("<", "\<").replace(">", "\>")
        self.s.write(" [label=\"%s\"]" % label)
        self.s.write(";" + os.linesep)
    def edge(self, f, t):
        self.s.write(" %s -> %s" % (self.table[f], self.table[t]))
        self.s.write(";" + os.linesep)
    def save(self, fn):
        with open(fn, 'w') as f:
            f.write("digraph sample {" + os.linesep)
            f.write("  node [shape = record];" + os.linesep)
            f.write(self.s.getvalue())
            f.write("}" + os.linesep)
    def preview(self, fn):
        fn = os.path.abspath(fn)
        fn, _ = os.path.splitext(fn)
        dotfn = fn + ".dot"
        pdffn = fn + ".pdf"
        self.save(dotfn)
        os.system("dot -Kdot -Tpdf %s > %s" % (dotfn, pdffn))
        os.system("gnome-open %s" % pdffn)


class GraphWriter(object):
    def __init__(self, root_node):
        self.root = root_node
        self.graph = Graph()
    def node_attr(self, n):
        return n.name + "\\n" + "\\l".join(["%s: %s" % (k, str(v)) for k,v in n.properties.items()]) + "\\l"
    def parse_node(self, n):
        self.graph.node(n.name, label=self.node_attr(n))
        if n.parent is not None:
            self.graph.edge(n.name, n.parent.name)
        for c in n.children:
            self.parse_node(c)
    def parse(self):
        self.parse_node(self.root)
    def save_pdf(self, dest):
        self.graph.preview(dest)
