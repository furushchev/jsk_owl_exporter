#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import lxml.etree
import lxml.builder
import sys

class OWLWriterMeta(object):
    VERSION = 0.93

class OWLWriter(OWLWriterMeta):
    def __init__(self, nsmap=None):
        self.nsmap = {
            None: "http://knowrob.org/kb/cram_log.owl#",
            "owl": "http://www.w3.org/2002/07/owl#",
            "xsd": "http://www.w3.org/2001/XMLSchema#",
            "knowrob": "http://knowrob.org/kb/knowrob.owl#",
            "rdfs": "http://www.w3.org/2000/01/rdf-schema#",
            "rdf": "http://www.w3.org/1999/02/22-rdf-syntax-ns#",
            "log": "http://knowrob.org/kb/cram_log.owl#",
        }
        if nsmap is not None:
            self.nsmap.update(nsmap)
        self.rdf = lxml.builder.ElementMaker(namespace=self.nsmap["rdf"],
                                             nsmap=self.nsmap)
        self.owl = lxml.builder.ElementMaker(namespace=self.nsmap["owl"],
                                             nsmap=self.nsmap)
        self.doc = self.rdf.RDF()
        self.doc.base = "http://knowrob.org/kb/cram_log.owl"

    def add_attrib(self, elem, ns, name, val_ns, val):
        if val_ns is not None:
            val = "&%s;%s" % (val_ns, val)
        elem.attrib["{%s}%s" % (self.nsmap[ns], name)] = val
        return elem

    def add_

    def gen_owl_import(self, uris):
        o = self.owl.Ontology()
        self.add_attrib(o, "rdf", "about", None, self.nsmap[None])
        for uri in uris:
            i = self.owl.imports()
            self.add_attrib(i, "rdf", "resource", None, uri)
            o.append(i)
        self.doc.append(o)

    def gen_property_definitions(self, props):
        self.doc.append(lxml.etree.Comment("Property Definitions"))

        op = self.owl.ObjectProperty()
        for prop in props:
            self.add_attrib(op, "rdf", "about", prop)
        self.doc.append(op)

    def gen_class_definitions(self, props):
        self.doc.append(lxml.etree.Comment("Class Definitions"))

        cls = self.owl.Class()
        for prop in props:
            self.add_attrib(cls, "rdf", "about", prop)
        self.doc.append(cls)

    def gen_event_individuals(self):
        pass

    def gen_object_individuals(self):
        pass

    def gen_image_individuals(self):
        pass

    def gen_designator_individuals(self):
        pass

    def gen_failure_individuals(self):
        pass

    def gen_timepoint_individuals(self):
        pass

    def gen_metadata_individual(self, creator, description, experiment, experimentName, robot, start_time, end_time):
        pass

    def to_string(self, pretty_print=True, doctype=None):
        if doctype is None:
            doctype = """<!DOCTYPE rdf:RDF [
    <!ENTITY owl "http://www.w3.org/2002/07/owl#" >
    <!ENTITY xsd "http://www.w3.org/2001/XMLSchema#" >
    <!ENTITY knowrob "http://knowrob.org/kb/knowrob.owl#" >
    <!ENTITY rdfs "http://www.w3.org/2000/01/rdf-schema#" >
    <!ENTITY rdf "http://www.w3.org/1999/02/22-rdf-syntax-ns#" >
    <!ENTITY log "http://knowrob.org/kb/cram_log.owl#" >
]>
"""
        self.gen_owl_import([
            "package://knowrob_common/owl/knowrob.owl"
        ])

        # TODO: generate other elements

        return lxml.etree.tostring(self.doc,
                                   encoding="utf-8",
                                   xml_declaration=True,
                                   pretty_print=pretty_print,
                                   doctype=doctype)

    def to_file(self, dest, pretty_print=True, doctype=None):
        with open(dest, "w") as f:
            f.write(self.to_string(pretty_print=pretty_print, doctype=doctype))

if __name__ == '__main__':
    w = OWLWriter()
    print w.to_string()
