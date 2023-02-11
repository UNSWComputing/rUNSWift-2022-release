getting cmake to work
=====================

if you haphazardly try random things and get the build to work, please take 
some extra time to clean up and minimize your changes.  we tend to copy 
changes from ctc to ctc so it becomes a big mess with the frequency that 
aldebaran change things.

we have plans to move to ubuntu, so that should reduce the amount of 
non-standard things we have to do for the ctc.

for 2.1 and earlier, everything should compile against cross-compiled 
libraries, so that tools like offnao and vatnao running on your computer 
work exactly the same as runswift running on the robot

for 2.8 and later, only runswift not libagent should compile with the 
ctc.  the tools should interact with runswift in such a way that 
everything is running in the runswift executable.  in this way, we 
aren't tied to a specific version of framework, compiler, or even 
language.  offnao & vatnao should not link against libsoccer.

for "native", offnao & vatnao are linking against libsoccer and at the time of 
writing, there are no known cross-compaibility issues with the behaviour of 
libraries in ubuntu 18.04 and Nao OS 2.8.  the native build will be run on the 
robot under ubuntu so there should be no issues in the future.

style
=====

CMake supports putting notes in the `else` and `endif` statements.  
don't use them for else, but leave a comment so we know the 
corresponding `if`.  your resulting statement should look like:

```
if(condition)
   ...
else() # not condition
  ...
endif(condition)
```
