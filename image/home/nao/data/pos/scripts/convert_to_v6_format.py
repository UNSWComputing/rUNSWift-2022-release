from __future__ import print_function
import sys

# Argument: pos file to be converted

if len(sys.argv) != 2:
    print("please specify file!")
    exit()

fPos = open(sys.argv[1], "r")

out = ""

for line in fPos:
    firstChar = " "
    if "#" in line:
        out += line
        continue
        firstChar = "#"

    line = line.replace("$", "")
    splitLine = line.split()
    if len(splitLine) == 25:
        out += "$ "
        LH = splitLine.pop(7)
        splitLine.insert(23, LH)
        for word in splitLine:
            out += word.ljust(6)
        out += "\n"
    elif len(splitLine) == 26:
        out += "! "
        LH = splitLine.pop(7)
        splitLine.insert(23, LH)
        for word in splitLine:
            out += word.ljust(6)
        out += "\n"
    else:
        out += line

fPos.close()

fPos = open(sys.argv[1], "w")
fPos.write(out)
fPos.close()
