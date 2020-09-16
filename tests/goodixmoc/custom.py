#!/usr/bin/python3

import gi
gi.require_version('FPrint', '2.0')
from gi.repository import FPrint, GLib

ctx = GLib.main_context_default()

c = FPrint.Context()
c.enumerate()
devices = c.get_devices()

d = devices[0]
del devices

assert d.get_driver() == "goodixmoc"

d.open_sync()

template = FPrint.Print.new(d)

def enroll_progress(*args):
    print('enroll progress: ' + str(args))

# List, enroll, list, verify, identify, delete
print("enrolling")
p = d.enroll_sync(template, None, enroll_progress, None)
print("enroll done")

print("listing")
stored = d.list_prints_sync()
print("listing done")
assert len(stored) == 1
assert stored[0].equal(p)
print("verifying")
verify_res, verify_print = d.verify_sync(p)
print("verify done")
assert verify_res == True
print("identifying")
identify_match, identify_print = d.identify_sync(stored)
print("identify done")
assert identify_match.equal(identify_print)

print("deleting")
d.delete_print_sync(p)
print("delete done")
d.close_sync()

del d
del c
