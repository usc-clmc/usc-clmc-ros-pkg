#!/usr/bin/python

from optparse import OptionParser

class CommandLineUtilities:
    def __init__(self): pass
        
    def getInput(self, querry, default = 'y'):
        decisions = '[Y/n]'
        if default == 'n':
            decisions = '[y/N]'
        ans = raw_input(querry + ' ' + decisions + ' :')
        while ans != '' and ans != 'y' and ans != 'Y' and ans != 'n':
            ans = raw_input(querry + ' ' + decisions + ' :')
        return (ans == 'y' or ans == 'Y' or (ans == '' and default != 'n'))
