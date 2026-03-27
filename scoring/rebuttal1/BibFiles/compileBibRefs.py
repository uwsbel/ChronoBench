#!/usr/bin/env python2
# -*- coding: utf-8 -*-
""" This file is used to extract the citations of a paper from 
    a .tex file. It then looks into your copy of BibResources folder 
    and copy the corresponding entries form *.bib into a single output 
    file.
    This is handy when you like to give the source files to someone 
    who does not have access to our SVN and you don't want to send them 
    all the *.bib files in the BibResources folder just so that they can
    compile your latex source.
    Created by Milad Rakhsha.
    Contributors: Dan Negrut.
"""
import sys
import os
import subprocess
import re


def parse_Tex(inputFile):
    # grep -wo '\\cite{[^}]*}' file.tex| sed 's/,/\n /g' | sed 's/\\cite{//g' | sed 's/}//g'
    string_1 = r"'\\cite{[^}]*}'"
    string_2 = r"s/,/,/g"
    string_3 = r"'s/\\cite{//g'"
    string_4 = r"'s/}//g'"
    cmd = r'grep -wo %s %s | sed "%s" | sed %s | sed %s' % (
        string_1, inputFile, string_2, string_3, string_4)
    # print cmd
    # print (
    #     r"grep -wo '\\cite{[^}]*}' productionVersion.tex | sed 's/,/\n/g' | sed 's/\\cite{//g' | sed 's/}//g'")
    buffer = subprocess.check_output(str(cmd), shell=True, stdin=subprocess.PIPE,
                                     universal_newlines=True,
                                     )
    out = re.sub(r',', r'\n', buffer)
    citations = []
    for l in out.split("\n"):
        if (l not in citations and l != ""):
            citations.append(l)

    print('citations in your tex files:', citations)
    return citations


def prepare_entry(bib_DIR, output_name):
    file = open(output_name, "w")
    i = 1
    for en in citations:

        cmd = r'grep -h -A 50 "%s," %s/* ' % (en, bib_DIR)
        # print cmd
        try:
            text = subprocess.check_output(str(cmd), shell=True,
                                           universal_newlines=True
                                           ).replace("\n", "_ENDLINE_")

            #print text
            input = "@.*?%s[^@]*" % en
            #print input
            cmd = r'grep -l "%s" %s/* ' % (en, bib_DIR)
            source_file = subprocess.check_output(cmd, shell=True)

            try:
                out = re.match(input, text).group().replace("_ENDLINE_", "\n")
                file.write(out)
                # print "\n\n\n-------", i, "--------------\n", out
                i = i+1
            except Exception as exception:
                # print text
                print("\nPROBLEM with the %s in the %s" % (en, source_file))
        except Exception as exception:
            print("Reference %s was not found in the bib source[s] provided" % en)

    file.close()


assert len(sys.argv) >= 3, '\n\nUser expected to provide at least two command line arguments.\n'\
    'Run with >> python3 compileBibRefs.py {SOURCE.tex} {PATH to BibResources folder} {output.bib}'
texFile = str(sys.argv[1])
print('\n\nProcessing the following tex file for all the citations : %s' % texFile)
citations = parse_Tex(texFile)
print('found %d distincdistinct citations...' % len(citations))
bib_folder = str(sys.argv[2])
if(len(sys.argv) > 3):
    output_bib = str(sys.argv[3])
else:
    output_bib = "refs.bib"
print('Greping from the source in  %s' % bib_folder)
prepare_entry(bib_DIR=bib_folder, output_name=output_bib)
print('References compiled in a new bib file called %s.' % output_bib)
