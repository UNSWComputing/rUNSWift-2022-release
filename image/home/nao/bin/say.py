#!/usr/bin/python -u

"""
This script is run from runswift and things to say are piped
 in via stdin.  Sayings also appear on runswift's stderr

CLI Example

  echo "hello world" | say.py

Why not done in runswift?
- because that requires linking runswift against
  aldebaran libraries and i don't want to do that

Why not in daemon.py?
- because that requires message passing
- because then we can't use stderr in simulation
- UNIX philosophy #1: Make each program do one thing well. To do a new job,
  build afresh rather than complicate old programs by adding new "features".

Why python and not c++?
- because this is easier
- because this is hopefully more maintainable

Why not the `say` that comes with the nao?
- because this uses less CPU by running persistently
- because this falls back to flite

Why threads and not select?
- because the select implementation was buggy as - see the git history
"""

from __future__ import print_function
import os
import sys
import traceback
import threading

sys.path.append("/opt/aldebaran/lib/python2.7/site-packages/")  # noqa

try:
    from naoqi import ALProxy
except ImportError as e:
    sys.stderr.write("%s\n" % e)
    ALProxy = None

IP = "localhost"
tts = None
something_to_say = None
condition = threading.Condition()
is_running = True


def init():
    if ALProxy:
        try:
            global tts
            tts = ALProxy("ALTextToSpeech", IP, 9559)
        except RuntimeError:
            # just assume it is the below and don't do anything
            # we used to print it but it's just too noisy
            #         ALProxy::ALProxy
            #         Can't find service: ALTextToSpeech
            pass
        except BaseException:
            traceback.print_exc()


def say(text, *args, **kwargs):
    sys.stderr.write("SAY:\t\t%s\n" % text)
    sys.stderr.flush()
    if not tts:
        init()
    if tts:
        tts.say(text, *args, **kwargs)
    else:
        os.system("echo '" + text + "' | /home/nao/2.8/bin/flite")


def say_thread():
    global something_to_say
    while is_running:
        condition.acquire()
        condition.wait()
        line = something_to_say
        something_to_say = None
        # have to release while saying
        condition.release()
        if line:
            say(line)


def start_say_thread():
    threading.Thread(target=say_thread).start()


def stop_say_thread():
    global is_running
    condition.acquire()
    is_running = False
    condition.notify()
    condition.release()


start_say_thread()
try:
    for line in iter(sys.stdin.readline, ""):
        condition.acquire()
        something_to_say = line.strip()
        condition.notify()
        condition.release()
except KeyboardInterrupt:
    pass
except Exception as e:
    print(e)
stop_say_thread()
