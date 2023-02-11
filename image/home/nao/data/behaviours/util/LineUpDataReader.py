import os

line_up_data_dir = ""
if "RUNSWIFT_CHECKOUT_DIR" in os.environ:
    line_up_data_dir = os.environ["RUNSWIFT_CHECKOUT_DIR"] + "/image"
line_up_data_dir += "/home/nao/data/"


def readData(filename="line_up_data.lud"):
    result = []
    filepath = line_up_data_dir + filename
    f = open(filepath)
    for line in f:
        line = line.split()
        line_result = []
        for item in line:
            if item == "-":
                line_result.append(0)
            elif item == "/":
                line_result.append(1)
            elif item == "o":
                line_result.append(2)
            else:
                line_result.append(0)
        result.append(line_result[::-1])

    return result[::-1], len(result), len(result[0])
